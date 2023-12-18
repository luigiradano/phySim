#include "main.h"
#include "constraintSolve.h"


#define CORR_WEI 1E5
#define CORR_EXP 1 
#define OSC_CUTOUT 1
#define MAX_FORCE 1E20

//JACOBIAN COMPUTATION INSTRUCTIOS
double getTraj(double x, double y){
	return x*x + y*y - 4;
}
void getJacob(double x, double y, Matrix *RES){
	double temp;
	temp = 2*x + y*y - 4;
	setElement(RES, 0, 0, temp);
	temp = 2*y + x*x - 4;
	setElement(RES, 0, 1, temp);
}
void getJacob2(double x, double y, Matrix *RES){
	setElement(RES, 0, 0, 1);
	setElement(RES, 0, 1, -1);
}



//Get distance from bottom of id to top of i
float getDistanceTopBtm(SolidRect *rectSet, int id, int i){
	if(id < 0 || id >= MAX_OBJS || i < 0 || i >= MAX_OBJS)
		return 0;
	return (rectSet[id].yPos-rectSet[id].dispRect.h) - (rectSet[i].yPos);
}

float getDistanceBtmBtm(SolidRect *rectSet, int id, int i){
	if(id < 0 || id >= MAX_OBJS || i < 0 || i >= MAX_OBJS)	
		return 0;
	return (rectSet[id].yPos+rectSet[id].dispRect.h) - (rectSet[i].yPos+rectSet[i].dispRect.h);
}

float getDistanceTopTop(SolidRect *rectSet , int id, int i){
	return 0;
	if(id < 0 || id >= MAX_OBJS || i < 0 || i >= MAX_OBJS)	
		return 0;
	return rectSet[i].yPos - rectSet[id].yPos;
}

float getDistanceBtmTop(SolidRect *rectSet, int id, int i){
	if(id < 0 || id >= MAX_OBJS || i < 0 || i >= MAX_OBJS)	
		return 0;
	return rectSet[id].yPos - (rectSet[i].yPos-rectSet[i].dispRect.h);
}

//Constraints two lines from overlapping, distance between lines specified in distFun
double overlapConstraint(SolidRect *rectSet, int id, int i, float (*distFun)(SolidRect*, int, int)){
	double dist = (*distFun)(rectSet, id, i);
	if(dist > 0)
		return 0;
	else
		return -1 * dist;
}
//Constraints id into the dispRect of i vertically
double boxedConstraint(SolidRect *rectSet, int id, int i){
	double distTT = getDistanceTopTop(rectSet, id, i);
	double distBB = getDistanceBtmBtm(rectSet, id, i);
	if(distTT < 0 ){
		return -1 * distTT;
	}
	else if(distBB < 0 ){
		return  distBB;
	}
	else
		return 0;
}

void setForce(float forceMat[][MAX_OBJS], int id, int i, float force){

		forceMat[id][i] = force;
		forceMat[i][id] = -1 * force;
}

//Where the magic happens, exchanges forces on id an i to satisfy constraints
void forceCorrector(SolidRect *rectSet, float forceMat[][MAX_OBJS], int id, int i){
	double feedBack;
	if( id == 0){
		feedBack = pow(boxedConstraint(rectSet, id, i), CORR_EXP) * CORR_WEI;
	}
	else{
		feedBack = -1 * pow(overlapConstraint(rectSet, id, i, &getDistanceTopBtm), CORR_EXP) * CORR_WEI;
	}
	if(feedBack > MAX_FORCE)
		feedBack = MAX_FORCE;
	else if(feedBack < -1 * MAX_FORCE)
		feedBack = -1 * MAX_FORCE;

setForce(forceMat, id, i, feedBack);
}
// SERIOUS CONSTRAINT SOLVER AHEAD
void initConstraints(Constraint *con, double (*getC)(double x, double y), void (*getJacobian)(double x, double y, Matrix *RES), void (*getJacobian2)(double x, double y, Matrix *RES)){
	con->getC = getC;
	con->getJacobian = getJacobian;
	con->getJacobian2 = getJacobian2;
}

void solveConstraints(Constraint *con[], unsigned int consCount, RigidState *state, double forceMat[][MAX_OBJS][DIMENSIONS]){
		unsigned int i = 0; 	
		Matrix jacob;	
		initMatrix(&jacob, 2, 2);
		Matrix jacobInv;
		initMatrix(&jacobInv, 2, 2);
		Matrix jacob2; 
		initMatrix(&jacob2, 2, 1);
		Matrix mass;
		initMatrix(&mass, 1, 1);
		Matrix velocity;
		initMatrix(&velocity, 2, 1);
		Matrix force;
		initMatrix(&force, 2, 1);
	
		Matrix tmp;
		initMatrix(&tmp, 1, 1);
		
		getJacob( state->xPos, state->yPos, &jacob);
		getJacob2( state->xPos, state->yPos, &jacob2);		
		
		transpose(&jacob);
		copyMat(&jacob, &jacobInv);
		transpose(&jacob);
			
		matrixMultiply(&jacob, &jacobInv, &tmp);
		setElement(&tmp, 0, 0, getElement(&tmp, 0, 0) / state->mass);
		
		setElement(&force, 0, 0, forceMat[1][1][0]);
		setElement(&force, 0, 1, forceMat[1][1][1]);
		setElement(&velocity, 0, 0, state->xSpe);
		setElement(&velocity, 0, 1, state->ySpe);
		
		double left, right;
		left = getElement(&tmp, 0, 0);
		
		matrixMultiply(&jacob2, &velocity, &tmp);
		right = getElement(&tmp, 0, 0) * -1;
		
		matrixMultiply(&jacob, &force, &tmp);
		right -= getElement(&tmp, 0, 0) / state->mass;
				
		printf("%.2f\t%.2f\t%.2f\t%.2f\n", left, right, right/left, 0);
			
			

} 



