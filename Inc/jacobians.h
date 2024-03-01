#ifndef JACOBIAN_H
#define JACOBIAN_H

// Jacobian definitions
#define CIRC_JAC_DIMS 4
#define HINGE_JAC_DIMS 2

double circumResisdual(double xCenter, double yCenter, double x, double y, double radius);
void circumJacobian(double xCenter, double yCenter, double x, double y, double result[CIRC_JAC_DIMS]);
void circumJacobian2(double xCentSpe, double yCentSpe, double xSpe, double ySpe, double result[CIRC_JAC_DIMS]);

void hingeJacobian(double xCenter, double yCenter, double x, double y, double result[HINGE_JAC_DIMS]);
void hingeJacobian2(double xCentSpe, double yCentSpe, double xSpe, double ySpe, double result[HINGE_JAC_DIMS]);
double hingeResisdual(double xCenter, double yCenter, double x, double y);


#endif