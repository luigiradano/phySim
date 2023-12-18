#ifndef MATRIX_H
#define MATRIX_H
typedef struct {
	double *data;
	unsigned int rows;
	unsigned int cols;
	unsigned char transposed;	
} Matrix;

//Function definitions
void printMatrix(Matrix *mat);
void initMatrix(Matrix *mat, unsigned int rows, unsigned int cols);
void freeMatrix(Matrix *mat);
double getElement(Matrix *mat, unsigned int row, unsigned int col);
bool setElement(Matrix *mat, unsigned int row, unsigned int col, double val);
bool matrixMultiply(Matrix *A, Matrix *B, Matrix *RES);
void transpose(Matrix *A);
bool copyMat(Matrix *A, Matrix *B);
#endif
