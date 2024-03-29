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
bool addMatrix(Matrix *A, Matrix *B, Matrix *C);
bool scaleMat(Matrix *A, Matrix *B, double scaling);
void transpose(Matrix *A);
void mat2double(Matrix *A, double out[A->rows][A->cols]);
void mat2doubleVec(Matrix *A, double out[]);
bool copyMat(Matrix *A, Matrix *B);
bool swapColumn(Matrix *A, uint32_t colA, uint32_t colB);
bool swapRow(Matrix *A, uint32_t rowA, uint32_t rowB);
int getColMaxAbs(Matrix *A, uint32_t colIndex);
int getRowMaxAbs(Matrix *A, uint32_t rowIndex);
double getNorm2(Matrix *A);
uint8_t vec2row(double array[], Matrix *A, uint32_t row, uint32_t size, uint32_t offset);
#endif
