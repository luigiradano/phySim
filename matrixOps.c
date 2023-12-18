#include "main.h"
#include "matrixOps.h"


void initMatrix(Matrix *mat, unsigned int rows, unsigned int cols){
	double* tmp = (double*) malloc(sizeof(double) * rows * cols);
	if(tmp == NULL)
		return;
	mat->data = tmp;
	mat->rows = rows;
	mat->cols = cols;
	mat->transposed = 0;
}

void freeMatrix(Matrix *mat){
	free(mat->data);
}

double getElement(Matrix *mat, unsigned int row, unsigned int col){
	unsigned int index;
	
	if(mat->transposed)
		index = row + col * mat->cols;
	else
		index = col + row * mat->cols;
	
	if(index < mat->cols*mat->rows)
		return *(mat->data+index);
	else	
		return 0;
}

bool setElement(Matrix *mat, unsigned int row, unsigned int col, double val){
	unsigned int index;
	if(mat->transposed)
		index = row + col * mat->cols;
	else
		index = col + row * mat->cols;
		
	if(index < mat->cols * mat->rows){
		*(mat->data+index) = val;
		return true;
	}
	else
		return false;
}

bool matrixMultiply(Matrix *A, Matrix *B, Matrix *RES){
	unsigned int i, j, iOut, jOut;
	double tempSum = 0;
	
	//Check if multiplication is possible
	if(A->cols != B->rows || A->rows != B->cols)
		return false;
	//Do not check the result matrix as it could be larger then in needs

	for(iOut = 0; iOut < A->rows; iOut ++){
		for(jOut = 0; jOut < B->rows; jOut++){
			tempSum = 0;
			for(i = 0; i < A->cols; i++)
				tempSum += getElement(A, i, jOut) * getElement(B, iOut, i);
			setElement(RES, iOut, jOut, tempSum);
		}	
	}

	return true;
}

void transpose(Matrix *A){
	A->transposed = !A->transposed; //Invert trasposed flag
	unsigned int tmp;
	tmp = A->cols; //Invert dimensions
	A->cols = A->rows;
	A->rows = tmp;
}

bool copyMat(Matrix *A, Matrix *B){
	unsigned int i, j;
	if(A->cols != B->cols || A->rows != B->rows)	
		return false;
	
	for( i = 0; i < A-> cols; i ++){
		for( j = 0; j < A->rows; j ++){
			setElement(B, i, j, getElement(A, i, j));
		}	
	}	
	return true;
}
