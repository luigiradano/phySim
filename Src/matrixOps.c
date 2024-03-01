#include "main.h"
#include "matrixOps.h"

void printMatrix(Matrix *mat){
	unsigned int i, j;
	for( i = 0; i < mat->rows; i++){
		for( j = 0; j < mat->cols; j++){
			printf("\t%.9f", getElement(mat, i, j));
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

bool swapColumn(Matrix *A, uint32_t colA, uint32_t colB){
	uint32_t i;
	double tmp;
	if(colA > A->cols || colB > A->cols)
		return ERROR;
	for(i = 0; i < A->rows; i ++){
		tmp = getElement(A, i, colA);
		setElement(A, i, colA, getElement(A, i, colB));
		setElement(A, i, colB, tmp);
	}
	return NO_ERROR;
}

bool swapRow(Matrix *A, uint32_t rowA, uint32_t rowB) {
  uint32_t j;
  double tmp;

  if (rowA >= A->rows || rowB >= A->rows) 
    return ERROR;
  
  for (j = 0; j < A->cols; j++) {
    tmp = getElement(A, rowA, j);
    setElement(A, rowA, j, getElement(A, rowB, j));
    setElement(A, rowB, j, tmp);
  }

  return NO_ERROR;
}

/*
	@brief Puts a double array in place of a matrix row, using the specified offset to shift the elements of the vector towards the right of the matrix
*/
uint8_t vec2row(double array[], Matrix *A, uint32_t row, uint32_t size, uint32_t offset){
	bool warnFlag = false;

	if(size != A->cols)
		warnFlag = 1;

	uint32_t i;
	for( i = 0; i < size; i ++){
		setElement(A, row, i+offset, array[i]);
	}

	if(warnFlag)
		return WARN;
		
	return NO_ERROR;
}

/*
	@brief Returns the Row Index of the max in absolute value inside the specified column
*/
int getColMaxAbs(Matrix *A, uint32_t colIndex){
	uint32_t i;
	int maxIndex = -1;
	double max = -1;

	if(colIndex > A->cols)
		return ERROR;
	
	for(i = 0; i < A->rows; i ++){
		if( abs(getElement(A, i, colIndex)) > max){
			maxIndex = i;
			max = abs(getElement(A, i, colIndex));
		}
	}

	return maxIndex;
}
/*
    @brief Returns the Row Index of the max in absolute value inside the specified row
*/
int getRowMaxAbs(Matrix *A, uint32_t rowIndex){
    uint32_t j;
    int maxIndex = -1;
    double max = -1;

    if(rowIndex > A->rows)
        return ERROR;
    
    for(j = 0; j < A->cols; j ++){
        if(abs(getElement(A, rowIndex, j)) > max){
            maxIndex = j;
            max = abs(getElement(A, rowIndex, j));
        }
    }

    return maxIndex;
}
/*
    @brief Returns the L2 norm of the matrix
*/
double getNorm2(Matrix *A){
    double norm2 = 0;
    for(uint32_t i = 0; i < A->rows; i ++){
        for(uint32_t j = 0; j < A->cols; j ++){
            norm2 += getElement(A, i, j) * getElement(A, i, j);
        }
    }

    return sqrt(norm2);
}