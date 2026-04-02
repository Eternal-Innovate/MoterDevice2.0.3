#include "LeastSquaresMethod.h"
int m = 500, num = 3, n = 10;
void leastSquareMethod_SvdSolve(float *A, float *U)
{
	/* A^T * A */
	
	float *At = rt_malloc(m*n*sizeof(float));//float At[5000]
	matrix_T(A, At, m, n);
	float *tmp = rt_malloc((m+1)*sizeof(float));//float tmp[100]
	matrix_muti(At, A, tmp, n, m, n);
	rt_free(At);

	float *N = rt_malloc((m+1)*sizeof(float));//float N[101]
	for(int i=1;i<=n;++i)
	{
		for(int j=1;j<=n;++j)
		{
			N[(i-1)*n+j] = tmp[(i-1)*n+(j-1)];
		}
	}
	rt_free(tmp);
	
	/* Svd + Jacobi */
	float *V = rt_malloc((m+1)*sizeof(float));//	double V[101];
	jacobiCall(N, V, n);
//	debug_print("N\n");
//	for(int i=1;i<=n;++i)
//	{
//		for(int j=1;j<=n;++j)
//		{
//			debug_print("%f ", N[(i-1)*n+j]);
//			rt_thread_delay(100);
//		}
//		debug_print("\n");
//		rt_thread_delay(20);
//	}
//	debug_print("V\n");
//	for(int i=1;i<=n;++i)
//	{
//		for(int j=1;j<=n;++j)
//		{
//			debug_print("%f ", V[(i-1)*n+j]);
//			rt_thread_delay(100);
//		}
//		debug_print("\n");
//		rt_thread_delay(20);
//	}
	adjustMatrix(N, V, n);
	/* Sort */
	float Num[n+1];
	for(int i=1;i<=n;++i) Num[i] = N[(i-1)*n+i];
//	rt_thread_delay(20);
	float *p[n+1];
	float *Vt = rt_malloc((n*n+1)*sizeof(float));
	//float Vt[101];
	for(int i=1;i<=n;++i)
	{
		for(int j=1;j<=n;++j)
		{
			Vt[(j-1)*n+i] = V[(i-1)*n+j];
		}
	}
//	debug_print("Vt\n");
//	for(int i=1;i<=n;++i)
//	{
//		for(int j=1;j<=n;++j)
//		{
//			debug_print("%f ", Vt[(i-1)*n+j]);
//			rt_thread_delay(100);
//		}
//		debug_print("\n");
//		rt_thread_delay(20);
//	}
	
	for(int i=1;i<=n;++i)
	{
		p[i] = &Vt[(i-1)*n+1]; 
	}
//	debug_print("p\n");
//	for(int i=1;i<=n;++i)
//	{
//		for(int j=1;j<=n;++j)
//		{
//			debug_print("%f ", p[i][j-1]);
//			rt_thread_delay(100);
//		}
//		debug_print("\n");
//		rt_thread_delay(20);
//	}
	quickSortPointer(Num, p,1, n);
//	debug_print("p2\n");
//	for(int i=1;i<=n;++i)
//	{
//		for(int j=1;j<=n;++j)
//		{
//			debug_print("%f ", p[i][j-1]);
//			rt_thread_delay(100);
//		}
//		debug_print("\n");
//		rt_thread_delay(20);
//	}
	for(int i=1;i<=n;++i)
	{
		U[i] = p[1][i-1];
	}
	rt_free(Vt);
	rt_free(V);
	rt_free(N);
	/* p[1] is the answer*/
}

void remove_hard_iron_offset(float *A, float *x)
{
	debug_print("hard_iron_offset\n");
	rt_thread_delay(20);
	
	int N = num;
	float *Q = rt_malloc((N*N)*sizeof(float));
	float *invQ = rt_malloc((N*N)*sizeof(float));
	int tot = 0;
	for(int j=0;j<=N-1;++j)
	{
		for(int i=0;i<=(N-1)-j;++i)
		{
			Q[(i*N+i)+j] = A[tot++];
			if(j==0) Q[(i*N+i)+j] *= 2;
		}
	}
	for(int i=0;i<=N-1;++i)
	{
		for(int j=i+1;j<=N-1;++j)
		{
			Q[j*N+i] = Q[i*N+j];
		}
	}
	
//	debug_print("Q\n");
//	rt_thread_delay(20);
//	for(int j=0;j<=N-1;++j)
//	{
//		for(int i=0;i<=N-1;++i)
//		{
//			debug_print("%f ", Q[j*N+i]);
//			rt_thread_delay(20);
//		}
//	}
//	debug_print("\n");
//	rt_thread_delay(20);
	
	float *T = rt_malloc(N*sizeof(float));
	for(int j=0;j<=N-1;++j)
	{
		T[j] = -A[tot++];
	}
	
//	debug_print("T\n");
//	rt_thread_delay(20);
//	for(int j=0;j<=N-1;++j)
//	{
//		debug_print("%f ", T[j]);
//		rt_thread_delay(20);
//	}
//	debug_print("\n");
//	rt_thread_delay(20);
	
	matrix_inverse(Q, invQ, N);
	
//	debug_print("invQ\n");
//	for(int j=0;j<=N-1;++j)
//	{
//		for(int i=0;i<=N-1;++i)
//		{
//			debug_print("%f ", invQ[j*N+i]);
//			rt_thread_delay(20);
//		}
//	}
//	debug_print("\n");
//	rt_thread_delay(20);
	
//	float *E = rt_malloc((N*N)*sizeof(float));
//	matrix_muti(invQ, Q, E, N, N, N); 
//	
//	debug_print("E\n");
//	for(int j=0;j<=N-1;++j)
//	{
//		for(int i=0;i<=N-1;++i)
//		{
//			debug_print("%f ", E[j*N+i]);
//			rt_thread_delay(20);
//		}
//	}
//	debug_print("\n");
//	rt_thread_delay(20);
//	
//	rt_free(E);
//	
	matrix_muti(invQ, T, x, N, N, 1);
	
	rt_free(invQ);
	
//	debug_print("x\n");
//	rt_thread_delay(20);
//	for(int j=0;j<=N-1;++j)
//	{
//		debug_print("%f ", x[j]);
//		rt_thread_delay(20);
//	}
//	debug_print("\n");
//	rt_thread_delay(20);
	
//	debug_print("x, y, z\n");
//	rt_thread_delay(20);
//	for(int i=0;i<=N-1;++i)
//	{
//		debug_print("%f ", x[i]);
//		rt_thread_delay(20);
//	}
//	debug_print("\n");
//	rt_thread_delay(20);
	
	for(int i=0;i<=N-1;++i)
	{
		float sum = 0;
		for(int j=0;j<=N-1;++j)
		{
			sum += Q[i*N+j]*x[j];
		}
		debug_print("%f\n", sum+T[i]);
		rt_thread_delay(20);
	}
	rt_free(T);
	rt_free(Q);
}
void remove_soft_iron_distortion(float *A, float *B, float *Pt)
{
	debug_print("remove_soft_iron_distortion\n");
	rt_thread_delay(20);
	
	int N = num;
	float *Q = rt_malloc((N*N)*sizeof(float));
	int tot = 0;
	for(int j=0;j<=N-1;++j)
	{
		for(int i=0;i<=(N-1)-j;++i)
		{
			Q[(i*N+i)+j] = A[tot++];
		}
	}
	for(int i=0;i<=N-1;++i)
	{
		for(int j=i+1;j<=N-1;++j)
		{
			Q[j*N+i] = Q[i*N+j];
		}
	}
	
	debug_print("Q\n");
	rt_thread_delay(20);
	for(int j=0;j<=N-1;++j)
	{
		for(int i=0;i<=N-1;++i)
		{
			debug_print("%f ", Q[j*N+i]);
			rt_thread_delay(20);
		}
	}
	debug_print("\n");
	rt_thread_delay(20);
	
	float *tmpQ = rt_malloc((N*N+1)*sizeof(float));
	for(int i=0;i<=N*N-1;++i)
	{
		tmpQ[i+1] = Q[i];
	}
	rt_free(Q);
	float *V = rt_malloc((N*N+1)*sizeof(float));//	double V[3*3+1];
	jacobiCall(tmpQ, V, N);
	//debug_print("jacobi\n");
	rt_thread_delay(20);
	float *tmpV = rt_malloc((N*N)*sizeof(float));//	double V[3*3+1];
	for(int i=0;i<=N-1;++i)
	{
		for(int j=0;j<=N-1;++j)
		{
			tmpV[i*N+j] = V[i*N+j+1];
		}
	}
	matrix_T(tmpV, Pt, N, N);
	rt_free(tmpV);
	rt_free(V);
	for(int i=0;i<=N-1;++i)
	{
		for(int j=0;j<=N-1;++j)
		{
			B[i*N+j] = tmpQ[i*N+j+1];
		}
	}
//	debug_print("B\n");
//	rt_thread_delay(20);
//	for(int j=0;j<=N-1;++j)
//	{
//		for(int i=0;i<=N-1;++i)
//		{
//			debug_print("%f ", B[j*N+i]);
//			rt_thread_delay(20);
//		}
//	}
//	debug_print("\n");
//	rt_thread_delay(20);
	
	rt_free(tmpQ);
	
//	for(int i=0;i<=N-1;++i)
//	{
//		for(int j=0;j<=N-1;++j)
//		{
//			debug_print("%f ", Vt[i*N+j]);
//			rt_thread_delay(20);
//		}
//		debug_print("\n");
//		rt_thread_delay(20);
//	}
	
	

}