#include "Sort.h"
void swap(float *a, float *b)
{
	double tmp = *a; 
	*a = *b;
	*b = tmp;
}

void quickSort(float *arr, int L, int R)
{
	if(L>=R) return ;
	int posL = L, posR = R;
	float tmp = arr[L];
	while(posL<posR)
	{
		while(arr[posR]>tmp&&posR>posL) --posR;
		if(posL<posR)
		{
			arr[posL++] = arr[posR];
		} 
		while(arr[posL]<=tmp&&posL<posR) ++posL;
		if(posL<posR)
		{
			arr[posR--] = arr[posL];
		}
	} 
	arr[posL] = tmp;
	quickSort(arr, L, posL-1);
	quickSort(arr, posL+1, R);
}

void quickSortPointer(float *arr, float **p, int L, int R)
{
	if(L>=R) return ;
	int posL = L, posR = R;
	float tmp = arr[L];
	float *tmpp = p[L];
	while(posL<posR)
	{
		while(arr[posR]>tmp&&posR>posL) --posR;
		if(posL<posR)
		{
			arr[posL] = arr[posR];
			p[posL++] = p[posR];
		} 
		while(arr[posL]<=tmp&&posL<posR) ++posL;
		if(posL<posR)
		{
			arr[posR] = arr[posL];
			p[posR--] = p[posL];
		}
	} 
	arr[posL] = tmp;
	p[posL] = tmpp;
	quickSortPointer(arr, p, L, posL-1);
	quickSortPointer(arr, p, posL+1, R);
}