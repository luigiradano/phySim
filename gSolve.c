#include "main.h"
#include "gSolve.h"
//#define DEBUG_GAUSS

void printMat(uint32_t cols, uint32_t rows, double mat[rows][cols]){
	int i, j;
	for(i=0; i<rows; i++){
		for(j=0; j<cols; j++){
			printf("%f\t|", mat[i][j]);
		}
		printf("\n");
	}
}

bool solveSystem(uint32_t cols, uint32_t rows, double knownMat[rows][cols], double knownVec[], double xVec[]){
	
	uint32_t i, j, k, rowFromBtm;
	double tempCoeff;
	double lineTot = 0;
#ifdef DEBUG_GAUSS
	printf("Initial matrix:\n");
	printMat(cols, rows, knownMat);
#endif
//Gaussian elimination
	for(k = 0; k < rows; k ++){
		for(i = k+1; i < rows; i ++){
			//Cycle trough rows
			if(knownMat[k][k] == 0)
				return ERROR;
			tempCoeff = -1* knownMat[i][k]/knownMat[k][k];	
			//Do ERO
			knownVec[i] += knownVec[k]*tempCoeff;
			for( j = 0; j < cols; j ++){
				knownMat[i][j] += (knownMat[k][j] * tempCoeff);
			}
		}
	}
#ifdef DEBUG_GAUSS
	printf("Reduced matrix:\n");
	printMat(cols, rows, knownMat);
#endif
//We can now solve
	j = cols-1;
	for(i = 0; i < rows ; i++){
		//Compute the sum of the row with known terms
		rowFromBtm = rows-i-1;
		for(k = 0; k < i; k ++){
			lineTot += knownMat[rowFromBtm][cols-k-1] * xVec[cols-k-1];
		}
		
		if(knownMat[rowFromBtm][j] == 0)
				return ERROR;
		xVec[rowFromBtm] = (knownVec[rowFromBtm]-lineTot) / knownMat[rowFromBtm][j];
		lineTot = 0;
		j--;
	}
	return NO_ERROR;
}

#ifdef MATRIX_H //Only compile this if MatrixOps.h is available
bool solveSystemMatrix(Matrix *Right, Matrix *Left, Matrix *Result){
	
	uint32_t i, j, k, rowFromBtm;
	double tempCoeff;
	double lineTot = 0;
#ifdef DEBUG_GAUSS
	printf("Initial matrix:\n");
	printMatrix(Left);
#endif
//Gaussian elimination
	for(k = 0; k < Right->rows; k ++){
		for(i = k+1; i < Right->rows; i ++){
			//Cycle trough rows
			if(getElement(Left, k, k) == 0)
				return ERROR;
			tempCoeff = -1* getElement(Left, i, k)/getElement(Left, k, k);	
			//Do ERO
			setElement(Right, i, 0, getElement(Right, i, 0) + (getElement(Right, k, 0) * tempCoeff));

			for( j = 0; j < Left->cols; j ++){
				setElement(Left, i, j, getElement(Left, i, j) + (getElement(Left, k, j) * tempCoeff));
			}
		}
	}
#ifdef DEBUG_GAUSS
	printf("Reduced matrix:\n");
	printMatrix(Left);
#endif
//We can now solve
	j = Left->cols - 1;
	for(i = 0; i < Right->rows ; i++){
		//Compute the sum of the row with known terms
		rowFromBtm = Right->rows - i - 1;
		for(k = 0; k < i; k ++){
			lineTot += getElement(Left, rowFromBtm, Left->cols - k - 1) * getElement(Result, Left->cols - k - 1, 0);
		}
		
		if(getElement(Left, rowFromBtm, j) == 0)
				return ERROR;
	
		setElement(Result, rowFromBtm, 0, (getElement(Right, rowFromBtm, 0) - lineTot) / getElement(Left, rowFromBtm, j));

		lineTot = 0;
		j--;
	}
#ifdef DEBUG_GAUSS
	printf("Result:\n");
	printMatrix(Result);
#endif
	return NO_ERROR;
}
#endif

double getResidual(uint32_t cols, uint32_t rows, double knownMat[cols][rows], double knownVec[], double xVec[]){
	int i, j;
	double tempSum = 0;
	double residualSum = 0, resultSum = 0.0001;
	//Do kownMat * xVec and sum the square of the result in one step
	for(i = 0; i < rows; i++){
		for(j = 0; j < cols; j ++){
			tempSum += knownMat[i][j] * xVec[j]; //Compute the output for every row
		}
		residualSum += pow(tempSum - knownVec[i], 2); //compute the difference from real value
		resultSum += pow(knownVec[i], 2); //compute the difference from real value
		tempSum = 0;
	}	
	return 100 * (sqrt(residualSum)/sqrt(resultSum));
}
