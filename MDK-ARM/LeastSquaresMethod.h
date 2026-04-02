#ifndef JACOBI
#define JACOBI

#include "Jacobi.h"
#include "Sort.h"
#include "magnetometer.h"
#include "rtthread.h"
#include "thread.h"

void leastSquareMethod_SvdSolve(float *A, float *U);
void remove_hard_iron_offset(float *A, float *x);
void remove_soft_iron_distortion(float *A, float *B, float *Pt);
#endif