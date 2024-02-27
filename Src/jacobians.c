#include "main.h"
#include "jacobians.h"

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

void curcumJacobian2(double xCentSpe, double yCentSpe, double xSpe, double ySpe, double result[CIRC_JAC_DIMS]){
	result[0] = 2 * (xSpe - xCentSpe);
	result[1] = 2 * (ySpe - yCentSpe);
	result[2] = 2 * (xCentSpe - xSpe);
	result[3] = 2 * (yCentSpe - ySpe);
}