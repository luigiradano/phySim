#ifndef CONST_LIST_H
#define CONST_LIST_H
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
#endif
