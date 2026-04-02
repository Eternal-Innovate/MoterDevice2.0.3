#ifndef JACOBIC
#define JACOBIC

#include "main.h"
#include <math.h>
#include "rtthread.h"
#include "calculate.h"

void jacobiCall(float *A, float *V, int N);
void adjustMatrix(float *A, float *V, int N);
#endif