#ifndef CONSTRAINT_H
#define CONSTRAINT_H
#include "matrixOps.h"
#include "rigidBody.h"

typedef struct {
	double (*getC)(double x, double y); //Function to get the constraint value
	void (*getJacobian)(double x, double y, Matrix *RES); //Function to get the jacobian of the constraint
	void (*getJacobian2)(double x, double y, Matrix *RES); //Function to get the time derivative of the jacobian 

	
} Constraint;


void solveConstraints(Constraint *con, unsigned int consCount, RigidState *state, double forceMat[][MAX_OBJS][DIMENSIONS]);
void initConstraints(Constraint *con, double (*getC)(double x, double y), void (*getJacobian)(double x, double y, Matrix *RES), void (*getJacobian2)(double x, double y, Matrix *RES));

double getTraj(double x, double y);
void getJacob(double x, double y, Matrix *RES);
void getJacob2(double x, double y, Matrix *RES);

#endif
