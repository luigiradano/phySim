#include "main.h"
#include "constraintSolve.h"


#define CORR_WEI 1E5
#define CORR_EXP 1 
#define OSC_CUTOUT 1
#define MAX_FORCE 1E20

#define TRAJ_SMOOTH 1
#define SPED_SMOOTH 0.1

#define DEBUG_FORCES


//JACOBIAN COMPUTATION INSTRUCTIOS
void getTraj(double x, double y, Matrix *RES){
	setElement(RES, 0, 0, pow(x, 2) + pow(y, 2) - 4);
}
void getJacob(double x, double y, Matrix *RES){
	double temp;
	temp = 2*x;
	setElement(RES, 0, 0, temp);
	temp = 2*y;
	setElement(RES, 0, 1, temp);
}
void getJacob2(double x, double y, Matrix *RES){
	setElement(RES, 0, 0, 2*(x));
	setElement(RES, 0, 1, 2*(y));
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

void solveConstraints(Constraint *con, unsigned int consCount, RigidState *state, double forceMat[][MAX_OBJS][DIMENSIONS]){
		unsigned int i = 0;
		bool errFlag = false;

		system("CLS");
		//ToDo: Move this stuff oudside cause allocation takes time (compiler probably optimizes this anyway)
		Matrix jacob;	
		initMatrix(&jacob, 1, 2);
		Matrix jacob2; 
		initMatrix(&jacob2, 1, 2);

		Matrix mass;
		initMatrix(&mass, 2, 2);
		Matrix velocity;
		initMatrix(&velocity, 2, 1);
		Matrix force;
		initMatrix(&force, 2, 1);

		Matrix left, right, right1, right2, corrector;
		initMatrix(&left, 1, 1);
		initMatrix(&right, 1, 1);
		initMatrix(&right1, 1, 1);
		initMatrix(&right2, 1, 1);
		initMatrix(&corrector, 1, 1);

		Matrix tmp;
		initMatrix(&tmp, 1, 2);

		errFlag |= setElement(&force, 0, 0, forceMat[EXTERNAL_FORCE][0][X_DIM]);
		errFlag |= setElement(&force, 1, 0, forceMat[EXTERNAL_FORCE][0][Y_DIM]);
		errFlag |= setElement(&velocity, 0, 0, state->xSpe);
		errFlag |= setElement(&velocity, 1, 0, state->ySpe);
		errFlag |= setElement(&mass, 0, 0, 1/state->mass);
		errFlag |= setElement(&mass, 1, 1, 1/state->mass);

		if(errFlag){
			printf("Error assigning input data!\n");
			errFlag = false;
		}

#ifdef DEBUG_INPUTS
		printf("Force :\n");
		printMatrix(&force);
		
		printf("Velocity :\n");
		printMatrix(&velocity);
#endif

		getJacob( state->xPos, state->yPos, &jacob );
		getJacob2(state->xSpe, state->ySpe, &jacob2);		
		getTraj(  state->xPos, state->yPos, &corrector);

#ifdef DEBUG_JACOB
		printf("Jacobian :\n");
		printMatrix(&jacob);
		printf("Jacobian 2:\n");
		printMatrix(&jacob2);
#endif

		//Left side computation
		errFlag |= matrixMultiply(&jacob, &mass, &tmp);

#ifdef DEBUG_JACOB
		printf("J * W:\n");
		printMatrix(&tmp);
#endif
	
		transpose(&jacob);
		errFlag |= matrixMultiply(&tmp, &jacob, &left);

		transpose(&jacob);

		if(errFlag){
			printf("Error in left computation!\n");
			errFlag = false;
		}

#ifdef DEBUG_LEFT
		printf("Error Flag: %d\tLeft :\n", errFlag);
		printMatrix(&left);
#endif


		//Right side computation
		errFlag |= matrixMultiply(&jacob2, &velocity, &right2);
		errFlag |= matrixMultiply(&tmp, &force, &right1);
		errFlag |= addMatrix(&right1, &right2, &right);
		//Feedback corrections
		errFlag |= scaleMat(&corrector, &corrector, TRAJ_SMOOTH); //Trajectory drift correction
		errFlag |= addMatrix(&right, &corrector, &right);

		errFlag |= matrixMultiply(&jacob, &velocity, &corrector);
		errFlag |= scaleMat(&corrector, &corrector, SPED_SMOOTH); //Trajectory drift correction
		errFlag |= addMatrix(&right, &corrector, &right);

		errFlag |= scaleMat(&right, &right, -1);

		if(errFlag){
			printf("Error in right computation!\n");
			errFlag = false;
		}

#ifdef DEBUG_RIGHT
		printf("Error Flag: %d\tRight :\n", errFlag);
		printMatrix(&right);
#endif

		//Solve system
		double leSi[1][1], riSi[1], res[1];

		mat2doubleVec(&right, riSi);
		mat2double(&left, leSi);

#ifdef DEBUG_SYSTEM
		printf("System Right eq:\n");
		printMat(1, left.rows, riSi);
		printf("System Left eq:\n");
		printMat(left.cols, left.rows, leSi);
#endif
		
		if(solveSystem(left.cols, left.rows, leSi, riSi, res) == ERROR){
			printf("Unsolvable system! Exiting\n");
			return;
		}
			

		if( abs(getResidual(left.cols, left.rows, leSi, riSi, res)) > 20){
			printf("Residual Too High! Exiting\n");
			return;
		}

#ifdef DEBUG_SYSTEM
		printf("Residual: %.2f\nSolution:\n", getResidual(left.cols, left.rows, leSi, riSi, res));
		printMat(1, left.rows, res);
#endif

		double tmpD;
		tmpD =  getElement(&jacob, 0, 0);
		forceMat[CONSTRAINT_FORCE][0][X_DIM] =  res[0] * tmpD;
		tmpD =  getElement(&jacob, 0, 1);
		forceMat[CONSTRAINT_FORCE][0][Y_DIM] =  res[0] * tmpD;

#ifdef DEBUG_FORCES
	printf("%.3f Lambda\t%.3f X\t%.3f Y\n", res[0], forceMat[CONSTRAINT_FORCE][0][X_DIM], forceMat[CONSTRAINT_FORCE][0][Y_DIM]);
#endif
	
} 




