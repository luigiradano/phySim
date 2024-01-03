#include "main.h"
#include "matrixOps.h"

void printMatrix(Matrix *mat){
	unsigned int i, j;
	for( i = 0; i < mat->rows; i++){
		for( j = 0; j < mat->cols; j++){
			printf("\t%.3f", getElement(mat, i, j));
		}
		printf("\n");
	}
}
void initMatrix(Matrix *mat, unsigned int rows, unsigned int cols){
	double* tmp = (double*) calloc(sizeof(double), rows * cols);
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
	
	if( col < mat->cols && row < mat->rows)
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
		return NO_ERROR;
	}
	else
		return ERROR;
}

bool matrixMultiply(Matrix *A, Matrix *B, Matrix *RES){
	unsigned int i, j, iOut, jOut;
	double tempSum = 0;
	
	//Check if multiplication is possible
	if(A->cols != B->rows)
		return ERROR;
	//Do not check the result matrix as it could be larger then in needs

	for(iOut = 0; iOut < A->rows; iOut ++){
		for(jOut = 0; jOut < B->cols; jOut++){
			tempSum = 0;
			double a, b;
			for(i = 0; i < A->cols; i++){
				a = getElement(A, iOut, i);
				b = getElement(B, i, jOut);
				tempSum += a * b;
			}
			setElement(RES, iOut, jOut, tempSum);
		}	
	}

	return NO_ERROR;
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
		return  ERROR;
	
	for( i = 0; i < A->rows; i ++){
		for( j = 0; j < A->cols; j ++){
			setElement(B, i, j, getElement(A, i, j));
		}	
	}	
	return  NO_ERROR;
}

bool scaleMat(Matrix *A, Matrix *B, double scaling){
	unsigned int i, j;
	if(A->cols != B->cols || A->rows != B->rows)	
		return  ERROR;
	
	for( i = 0; i < A->rows; i ++){
		for( j = 0; j < A->cols; j ++){
			setElement(B, i, j, scaling * getElement(A, i, j));
		}	
	}	
	return  NO_ERROR;
}

bool addMatrix(Matrix *A, Matrix *B, Matrix *C){
	unsigned int i, j;
	
	if(A->cols != B->cols || A->rows != B->rows)
		return  ERROR;

	for( i = 0; i < A->rows; i ++){
		for( j = 0; j < A->cols; j ++){
			setElement(C, i, j, getElement(A, i, j) + getElement(B, i, j));
		}	
	}
	return  NO_ERROR;	
}

void mat2double(Matrix *A, double out[A->rows][A->cols]){
	unsigned int i, j;
	for( i = 0; i < A->rows; i ++){
		for( j = 0; j < A->cols; j ++){
			out[i][j] = getElement(A, i, j);
		}	
	}
}

void mat2doubleVec(Matrix *A, double out[]){
	unsigned int i;
	for( i = 0; i < A->rows; i ++){
			out[i] = getElement(A, i, 0);
	}
}