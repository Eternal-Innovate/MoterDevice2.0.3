#ifndef __MAGNETOMETER_H__
#define __MAGNETOMETER_H__
#include "main.h"
#include "thread.h"

void matrix_muti(float *A, float *B, float *C, int n, int m, int p);
void matrix_T(float *A, float *B, int n, int m);
void matrix_inverse(float *A, float *A_inv, int n);
void LeastSquaresFit(float *X, float *beta, int Num, int M);
#endif 