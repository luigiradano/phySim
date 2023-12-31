#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#ifndef GSOLVE
#define GSOLVE

#define MAX_MATRIX_COLS 10


bool solveSystem(uint32_t cols, uint32_t rows, double knownMat[rows][cols], double knownVec[], double xVec[]);
void printMat(uint32_t cols, uint32_t rows, double mat[rows][cols]);
double getResidual(uint32_t cols, uint32_t rows, double knownMat[rows][cols], double knownVec[], double xVec[]);

#endif