#ifndef JACOBIAN_H
#define JACOBIAN_H

// Jacobian definitions
#define CIRC_JAC_DIMS 4

double circumResisdual(double xCenter, double yCenter, double x, double y, double radius);
void circumJacobian(double xCenter, double yCenter, double x, double y, double result[CIRC_JAC_DIMS]);
void curcumJacobian2(double xCentSpe, double yCentSpe, double xSpe, double ySpe, double result[CIRC_JAC_DIMS]);

#endif