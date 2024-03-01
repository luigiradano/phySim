#include "main.h"
#include "jacobians.h"
#include "matrixOps.h"

// Jacobian and trajectory residual definition

double circumResisdual(double xCenter, double yCenter, double x, double y, double radius){
    return (x - xCenter)*(x - xCenter) + (y - yCenter)*(y - yCenter) - radius*radius;
}

void circumJacobian(double xCenter, double yCenter, double x, double y, double result[CIRC_JAC_DIMS]){
	result[0] = 2*(x - xCenter);
	result[1] = 2*(y - yCenter);
	result[2] = 2*(xCenter - x);
	result[3] = 2*(yCenter - y);
}

void circumJacobian2(double xCentSpe, double yCentSpe, double xSpe, double ySpe, double result[CIRC_JAC_DIMS]){
	result[0] = 2 * (xSpe - xCentSpe);
	result[1] = 2 * (ySpe - yCentSpe);
	result[2] = 2 * (xCentSpe - xSpe);
	result[3] = 2 * (yCentSpe - ySpe);
}

double hingeResisdual(double xCenter, double yCenter, double x, double y){
    return (x - xCenter)*(x - xCenter) + (y - yCenter)*(y - yCenter);
}

void hingeJacobian(double xCenter, double yCenter, double x, double y, double result[HINGE_JAC_DIMS]){
	result[0] = -2*(xCenter - x);
	result[1] = -2*(yCenter - y);
}

void hingeJacobian2(double xCentSpe, double yCentSpe, double xSpe, double ySpe, double result[HINGE_JAC_DIMS]){
	result[0] = 2 * (xCentSpe - xSpe);
	result[1] = 2 * (yCentSpe - ySpe);
}
