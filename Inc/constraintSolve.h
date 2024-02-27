#ifndef CONSTRAINT_H
#define CONSTRAINT_H
#include "matrixOps.h"
#include "rigidBody.h"
#include "gSolve.h"

extern double residual;

typedef enum {
	SET_RADIUS,
} ConstraintType;



typedef struct {
	ConstraintType type;
	Matrix jacobTrans;
	Matrix jacob2Trans;
} Constraint;


void solveConstraints(RigidState *state[], double forceMat[][MAX_OBJS][DIMENSIONS], unsigned int objCount);
void initConstraints(Constraint *con, double (*getC)(double x, double y), void (*getJacobian)(double x, double y, Matrix *RES), void (*getJacobian2)(double x, double y, Matrix *RES));
void initContraintMats( unsigned int objCount);

void getJacob2(double vxA, double vyA, double vxB, double vyB, Matrix *RES);
void getJacob(double xA, double yA, double xB, double yB, Matrix *RES);
void getTraj(double xA, double yA, double xB, double yB, Matrix *RES);


#endif
