#include "magnetometer.h"

//[NxM] * [MxP] 
void matrix_muti(float *A, float *B, float *C, int n, int m, int p)
{
	for(int i=0;i<=n-1;++i)
	{
		for(int j=0;j<=p-1;++j)
		{
			for(int k=0;k<=m-1;++k)
			{
				if(k==0) C[i*p+j] = A[i*m+k]*B[k*p+j];
				else C[i*p+j] += A[i*m+k]*B[k*p+j];
			}
		}
	}
}
//[NxM] -> [MxN]
void matrix_T(float *A, float *B, int n, int m)
{
	for(int i=0;i<=n-1;++i)
	{
		for(int j=0;j<=m-1;++j)
		{
			B[j*n+i] = A[i*m+j];
		}
	}
}
//[N*N] ^ (-1)
void matrix_inverse(float *A, float *A_inv, int n)
{
	float *aug = rt_malloc((n*n*2)*sizeof(float));
	//float aug[n*n*2];//3*2*3 n*2*n
	//make aug
	for(int i=0;i<=n-1;++i)
	{
		for(int j=0;j<=n-1;++j)
		{
			aug[i*2*n+j] = A[i*n+j];
		}
		for(int j=n;j<=2*n-1;++j)
		{
			aug[i*2*n+j] = (j-n==i?1:0);
		}
	}
	
//	debug_print("aug\n");
//	for(int j=0;j<=n-1;++j)
//	{
//		for(int i=0;i<=2*n-1;++i)
//		{
//			debug_print("%f ", aug[j*(2*n)+i]);
//			rt_thread_delay(20);
//		}
//	}
//	debug_print("\n");
//	rt_thread_delay(20);
	
	//guass  
	for(int i=0;i<=n-1;++i)
	{
		int max_id = i;
		float max_val = aug[i*2*n+i];
		for(int j=i+1;j<=n-1;++j)
		{
			if(max_val<aug[j*2*n+i])
			{
				max_val = aug[j*2*n+i];
				max_id = j;
			}
		}
		
		for(int k=i;k<=2*n-1;++k)
		{
			float tmp = aug[i*2*n+k];
			aug[i*2*n+k] = aug[max_id*2*n+k];
			aug[max_id*2*n+k] = tmp;
		}
		//!0
		float frac = aug[i*2*n+i];
		for(int k=i;k<=2*n-1;++k)
		{
			aug[i*2*n+k] /= frac;
		}
		
		for(int j=i+1;j<=2*n-1;++j)
		{
			frac = aug[j*2*n+i];
			for(int k=i;k<=2*n-1;++k)
			{
				aug[j*2*n+k] -= frac*aug[i*2*n+k];
			}
		}
		
//		debug_print("aug\n");
//		for(int j=0;j<=n-1;++j)
//		{
//			for(int i=0;i<=2*n-1;++i)
//			{
//				debug_print("%f ", aug[j*(2*n)+i]);
//				rt_thread_delay(20);
//			}
//		}
//		debug_print("\n");
//		rt_thread_delay(20);
	}
	
	for(int i=0;i<=n-1;++i)
	{
		for(int j=0;j<=n-1;++j)
		{
			A_inv[i*n+j] = aug[i*2*n+(j+n)];
		}
	}
	
	rt_free(aug);
}

void LeastSquaresFit(float *X, float *beta, int Num, int M)
{
	float Xt[9000];
	float XX[81], X_[81];
	float B[1000];
	for(int i=0;i<=999;++i)
	{
		B[i] = 1;
	}
	for(int i=0;i<=Num-1;++i)
	{
		X[i*M+8] = X[i*M+2];
		X[i*M+7] = X[i*M+1];
		X[i*M+6] = X[i*M+0];
		X[i*M+5] = X[i*M+0]*X[i*M+2];
		X[i*M+4] = X[i*M+1]*X[i*M+2];
		X[i*M+3] = X[i*M+0]*X[i*M+1];
		X[i*M+0] = X[i*M+0]*X[i*M+0];
		X[i*M+1] = X[i*M+1]*X[i*M+1];
		X[i*M+2] = X[i*M+2]*X[i*M+2];
	}
	matrix_T(X, Xt, Num, M); //9000
	matrix_muti(Xt, X, XX, M, Num, M); //9000*9
	matrix_inverse(XX, X_, M);
	matrix_muti(X_, Xt, X, M, M, Num); //10000 // 100us
	matrix_muti(X, B, beta, Num, 1, 1);
}