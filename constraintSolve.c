#include "main.h"
#include "constraintSolve.h"


#define CORR_WEI 1E5
#define CORR_EXP 1 
#define OSC_CUTOUT 1
#define MAX_FORCE 1E20

#define TRAJ_SMOOTH 5
#define SPED_SMOOTH 0.001

/*
#define DEBUG_LEFT
#define DEBUG_JACOB
#define DEBUG_RIGHT
#define DEBUG_RESULT
*/

//JACOBIAN COMPUTATION INSTRUCTIOS
void getTraj(double xA, double yA, double xB, double yB, Matrix *RES){
	setElement(RES, 0, 0, pow(xA, 2) + pow(yA, 2) - 0.25);
	setElement(RES, 0, 1, pow(xA-xB, 2) + pow(yA-yB, 2) - 0.25);
}
void getJacob(double xA, double yA, double xB, double yB, Matrix *RES){
	setElement(RES, 0, 0, 2 * xA);
	setElement(RES, 0, 1, 2 * yA);
	setElement(RES, 0, 2, 0);
	setElement(RES, 0, 3, 0);
	setElement(RES, 1, 0, 2 * (xA - xB));
	setElement(RES, 1, 1, 2 * (yA - yB));
	setElement(RES, 1, 2, 2 * (xB - xA));
	setElement(RES, 1, 3, 2 * (yB - yA));
}
void getJacob2(double vxA, double vyA, double vxB, double vyB, Matrix *RES){
	setElement(RES, 0, 0, 2 * vxA);
	setElement(RES, 0, 1, 2 * vyA);
	setElement(RES, 0, 2, 0);
	setElement(RES, 0, 3, 0);
	setElement(RES, 1, 0, 2 * (vxA - vxB));
	setElement(RES, 1, 1, 2 * (vyA - vyB));
	setElement(RES, 1, 2, 2 * (vxB - vxA));
	setElement(RES, 1, 3, 2 * (vyB - vyA));
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

}

Matrix jacob, jacob2, mass, velocity, force, left, right, right1, right2, corrector, tmp, solVec;

void initContraintMats( unsigned int objCount){
		initMatrix(&jacob, objCount, objCount * DIMENSIONS);//Jacobian matrix
		initMatrix(&jacob2, objCount, objCount * DIMENSIONS);//Time derivative of jacobian
		initMatrix(&mass, DIMENSIONS * objCount, DIMENSIONS * objCount); //Mass matrix (ToBeImproved)
		initMatrix(&velocity, DIMENSIONS * objCount, 1);//Velocity vector
		initMatrix(&force, DIMENSIONS * objCount, 1);//Force vector
		initMatrix(&left, objCount, objCount);//Left Matrix
		initMatrix(&right, objCount, 1);//Right Matrix
		initMatrix(&right1, objCount, 1);//Right 1st computation
		initMatrix(&right2, objCount, 1);//Right 2nd computation
		initMatrix(&corrector, objCount, 1);//Correction matrix
		initMatrix(&tmp, objCount, DIMENSIONS * objCount); //Jacobian * Mass holder
		initMatrix(&solVec, objCount, 1); //Solution vector for solved system

}

void solveConstraints(RigidState *state[], double forceMat[][MAX_OBJS][DIMENSIONS], unsigned int objCount){
		bool errFlag = false;

		unsigned int i, j;

		for(i = 0; i < objCount * DIMENSIONS; i += 2){
			errFlag |= setElement(&force, i, 0, forceMat[EXTERNAL_FORCE][i/2][X_DIM]);
			errFlag |= setElement(&force, i+1, 0, forceMat[EXTERNAL_FORCE][i/2][Y_DIM]);
			errFlag |= setElement(&velocity, i, 0, state[i/2]->xSpe);
			errFlag |= setElement(&velocity, i+1, 0, state[i/2]->ySpe);
			errFlag |= setElement(&mass, i, i, 1/state[i/2]->mass);
			errFlag |= setElement(&mass, i+1, i+1, 1/state[i/2]->mass);
		}


		if(errFlag){
			printf("Error assigning input data!\n");
			errFlag = false;
		}

#ifdef DEBUG_INPUTS
		printf("Force :\n");
		printMatrix(&force);
		
		printf("Velocity :\n");
		printMatrix(&velocity);

		printf("Masses :\n");
		printMatrix(&mass);
#endif

		getJacob( state[0]->xPos, state[0]->yPos, state[1]->xPos, state[1]->yPos, &jacob );
		getJacob2( state[0]->xSpe, state[0]->ySpe, state[1]->xSpe, state[1]->ySpe, &jacob2 );
		getTraj( state[0]->xPos, state[0]->yPos, state[1]->xPos, state[1]->yPos, &corrector );

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
		errFlag |= scaleMat(&corrector, &corrector, SPED_SMOOTH * simulationTime); //Trajectory drift correction
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

#ifdef DEBUG_SYSTEM
		printf("System Right eq:\n");
		printMat(1, left.rows, riSi);
		printf("System Left eq:\n");
		printMat(left.cols, left.rows, leSi);
#endif
		
		if(solveSystemMatrix(&right, &left, &solVec) == ERROR){
			printf("Unsolvable system! Exiting\n");
			return;
		}
			
		

#ifdef DEBUG_RESULT
		printf("Result:\n");
		printMatrix(&solVec);
#endif

		forceMat[CONSTRAINT_FORCE][0][X_DIM] =  getElement(&solVec, 0, 0) * getElement(&jacob, 0, X_DIM);
		forceMat[CONSTRAINT_FORCE][0][Y_DIM] =  getElement(&solVec, 0, 0) * getElement(&jacob, 1, Y_DIM);
		
		forceMat[CONSTRAINT_FORCE][0][X_DIM] -=  getElement(&solVec, 1, 0) * getElement(&jacob, 0, X_DIM);
		forceMat[CONSTRAINT_FORCE][0][Y_DIM] -=  getElement(&solVec, 1, 0) * getElement(&jacob, 1, Y_DIM);
		
		forceMat[CONSTRAINT_FORCE][1][X_DIM] =   getElement(&solVec, 1, 0) * getElement(&jacob, 0, X_DIM);
		forceMat[CONSTRAINT_FORCE][1][Y_DIM] =   getElement(&solVec, 1, 0) * getElement(&jacob, 1, Y_DIM);


		for(i = 0; i < objCount; i ++){
			
		}

#ifdef DEBUG_FORCES
	printf("%.3f Lambda\t%.3f X\t%.3f Y\n", res[0], forceMat[CONSTRAINT_FORCE][0][X_DIM], forceMat[CONSTRAINT_FORCE][0][Y_DIM]);
#endif
	
} 




