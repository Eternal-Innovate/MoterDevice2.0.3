#include "Jacobi.h"
struct Node
{
	float A[101], V[101]; //N*N+1
}node[11];//N+1
void eInit(float *V, int N)
{
	for(int i=1;i<=N;++i)
	{
		for(int j=1;j<=N;++j)
		{
			if(i==j) V[(i-1)*N+j] = 1;
			else V[(i-1)*N+j] = 0;
		}
	}
}

void matrix_Muti(float *A, float *B, int n, int m, int p, int f)
{
	float *C = rt_malloc((n*p+1)*sizeof(float));
	//float C[101];
	
	for(int i=1;i<=n;++i)
	{
		for(int j=1;j<=p;++j)
		{
			for(int k=1;k<=m;++k)
			{
				if(k==1) C[(i-1)*p+j] = A[(i-1)*m+k]*B[(k-1)*p+j];
				else C[(i-1)*p+j] += A[(i-1)*m+k]*B[(k-1)*p+j];
			} 
		}
	}
	if(f==0) for(int i=1;i<=n*p;++i) A[i] = C[i];
	else for(int i=1;i<=n*p;++i) B[i] = C[i];
	
	rt_free(C);
}

void matrix_Tran(float *A, float *B, int n, int m)
{
	for(int i=1;i<=n;i++)
	{
		for(int j=i;j<=m;++j)
		{
			if(j>=i+1)
			{
				B[(i-1)*n+j] = A[(j-1)*m+i];
				B[(j-1)*n+i] = A[(i-1)*m+j];	
			}
			else if(j==i)
			{
				B[(i-1)*n+j] = A[(i-1)*m+j];
			}
		}	
	}	
} 

void eMatrix(int p, int q, float *R, float theta, int N)
{
	for(int i=1;i<=N;++i)
	{
		for(int j=1;j<=N;++j)
		{
			if(i==j)
			{
				if(i==p||i==q) R[(i-1)*N+j] = cos(theta); 
				else R[(i-1)*N+j] = 1;
			}
			else 
			{
				if(i==p&&j==q)
				{
					R[(i-1)*N+j] = -sin(theta);
				}
				else if(i==q&&j==p)
				{
					R[(i-1)*N+j] = sin(theta);
				}
				else R[(i-1)*N+j] = 0;
			} 
		}
	}
} 

void jacobiSolve(float *Rt, float *A, float *R, int N)
{
	matrix_Muti(A, R, N, N, N, 0);
	matrix_Muti(Rt, A, N, N, N, 1);
}

void jacobiCall(float *A, float *V, int N)
{	
	int flag;
	int Maxi = -1, Maxj = -1;
	float eps = 1e-7;
	float *Rt = rt_malloc((N*N+1)*sizeof(float));
	float *R = rt_malloc((N*N+1)*sizeof(float));//float R[101], Rt[101];
	eInit(V, N);
	while(1)
	{
		Maxi = Maxj = -1;
		flag = 1;
		for(int i=1;i<=N;++i)
		{
			for(int j=i+1;j<=N;++j)
			{
				float tmp = fabs(A[(i-1)*N+j]);
				if(tmp<=eps)
				{
					continue;
				}
				flag = 0;
				if(Maxi==-1)
				{
					Maxi = i;
					Maxj = j;
				}
				else if(tmp>fabs(A[(Maxi-1)*N+Maxj]))
				{
					Maxi = i;
					Maxj = j;
				}
			}
		}
		
		if(flag==1)
		{
			break;
		}
		float theta;
		if(fabs(A[(Maxj-1)*N+Maxj]-A[(Maxi-1)*N+Maxi])<=1e-7)
		{
			theta = PI/4;
		}
		else
		{
			theta = -atan(2*A[(Maxi-1)*N+Maxj]/(A[(Maxj-1)*N+Maxj]-A[(Maxi-1)*N+Maxi]))/2;
		}

		eMatrix(Maxi, Maxj, R, theta, N);
		
		matrix_Muti(V, R, N, N, N, 0);
		
		matrix_Tran(R, Rt, N, N);
		
		jacobiSolve(Rt, A, R, N);
		
		
	}
	rt_free(Rt);
	rt_free(R);
}

void adjustMatrix(float *A, float *V, int N)
{
	for(int i=1;i<=N;++i)
	{
		if(A[(i-1)*N+i]<0)
		{
			A[(i-1)*N+i] = -A[(i-1)*N+i];
			for(int j=1;j<=N;++j)
			{
				V[(j-1)*N+i] = -V[(j-1)*N+i];
			}
		}
	}
}
