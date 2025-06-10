#ifndef __MAGNETOMETER_H__
#define __MAGNETOMETER_H__
#include "main.h"

void matrix_muti(double *A, double *B, double *C, int n, int m, int p);
void matrix_T(double *A, double *B, int n, int m);
void matrix_inverse(double *A, double *A_inv, int n);
void LeastSquaresFit(double *X, double *beta, int Num, int M);
#endif 