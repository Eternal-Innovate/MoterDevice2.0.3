#include "thread.h"
#include "math.h"
#include "stdio.h"
#include "rtthread.h"
#include "arm_math.h"
#include "LeastSquaresMethod.h"
#include <stdarg.h>
#include "testData.h"
#define ARM_MATH_CM4
/*


		char byte[20];
		snprintf(byte, sizeof(byte), "test:%.3f, %.3f\n", mid, yaw);
		HAL_UART_Transmit_IT(&huart1, (uint8_t*)byte, 20);

*/


//int tot2 = 1;
#define DEBUG_BUFFER_SIZE 512
static char debug_buffer[DEBUG_BUFFER_SIZE];
static volatile int write_index = 0;
static volatile int read_index = 0;
static rt_mutex_t buffer_mutex;
static rt_sem_t uart_tx_sem = RT_NULL;
int data_busy;
static int buffer_free_space()
{
	if(write_index>=read_index)
	{
		return (DEBUG_BUFFER_SIZE-(write_index-read_index))-1;//Leave 1 space because 
	}
	else
	{
		return (read_index-write_index)-1;//Extra minus 1 because Leave 1 space 
	}
}

static int buffer_get(char *data, int max_len)
{
	rt_mutex_take(buffer_mutex, RT_WAITING_FOREVER);
	int available;
	if(write_index>=read_index)
	{
		available = write_index-read_index;
	}
	else
	{
		available = DEBUG_BUFFER_SIZE - read_index + write_index;
	}
	int to_read;
	if(available < max_len)
	{
		to_read = available;
	}
	else to_read = max_len;
	for(int i=0;i<to_read;i++) {
		data[i] = debug_buffer[read_index];
		read_index = (read_index + 1) % DEBUG_BUFFER_SIZE;
  }
	
	rt_mutex_release(buffer_mutex);
  return to_read;
}

static void buffer_put(const char *data, int len)
{
	rt_mutex_take(buffer_mutex, RT_WAITING_FOREVER);
	if(len>buffer_free_space())
	{//discard this data because there is no available space 
		rt_mutex_release(buffer_mutex);
		return ;
	}
	
	for(int i=0;i<len;++i)
	{
		debug_buffer[write_index] = data[i];
		write_index = (write_index+1)%DEBUG_BUFFER_SIZE;
	}
	rt_mutex_release(buffer_mutex);
}


void debug_print(const char *format, ...)
{
	int len;
	char byte[80];
	va_list args;
	
	va_start(args, format);
	len = vsnprintf(byte, sizeof(byte), format, args);
	va_end(args);
	
	buffer_put(byte, len);
  data_busy = 0;
}
//We need to ensure that the write speed cannot keep up with the data being read.
#define DEBUG_MAX_MESSAGE 128
void debug_thread2_entry(void *parameter)
{
	buffer_mutex = rt_mutex_create("dbg_mutex", RT_IPC_FLAG_FIFO);
	
	char tx_buffer[DEBUG_MAX_MESSAGE];
	while(1)
	{	
			if(!data_busy)
			{
				rt_sem_take(uart_tx_sem, RT_WAITING_FOREVER);
				int len = buffer_get(tx_buffer, DEBUG_MAX_MESSAGE);
			
				if(len>0){
					HAL_StatusTypeDef status = HAL_UART_Transmit_IT(&huart1, 
									(uint8_t*)tx_buffer, len);
							
							if (status != HAL_OK) {
									rt_sem_release(uart_tx_sem);
									debug_print("UART send failed: %d\n", status);
							}
				}
				data_busy = 1;
			}
			
			rt_thread_delay(10);
	}
}
//		if(svdFlag==0) len = snprintf(byte, sizeof(byte), "test:%d, %d\n", x, y);
//		else if(svdFlag==1)
//		{
//			len = snprintf(byte, sizeof(byte), "test:%d----%f\n", tot2, U[tot2]);
//			tot2 = (tot2)%10+1;
//		}


int flag_thread4 = 0;
const double e = 0.01;
double sumx, sumy;
double pi;
double v, w;
double kp, ki, kd;
double vl, vr;
float yaw = 0;
float mid;
double posW;		
double E[4] = {0, 0, 0, 0};
double eLast[4] = {0, 0, 0, 0};
uint16_t cx, cy, cz;
int16_t x, y, z;
double jiaodu, tx, ty;
double O_x, O_y;

double calMagnetometer(double x, double y)
{
  double num = atan2(y*1.0, x*1.0)/pi*180;
  return num;        
}

void pidControl(double v, int id)
{
	double v0 = counterFloatValue[id];
	double e = v-v0;  
	E[id] += e;
	double de = e-eLast[id];
	int pwmVal = kp*e+ki*E[id]+kd*(de);
	eLast[id] = e;
	if(pwmVal>1000) pwmVal = 1000;
	if(pwmVal<0) pwmVal = 0;
	ControlPwm(pwmVal, id);
}

void change(double nw, double nv)
{
	w = nw;
	v = nv;
	for(int i=0;i<=3;++i)
	{
		E[i] = eLast[i] = 0;
	} 
	CalculateLR(v, w/180.0*pi, &vl, &vr);
	dirControl(vr, 0);
	dirControl(vr, 1);
	dirControl(vl, 2);
	dirControl(vl, 3);
	
	if(vr<0) vr = -vr;
	if(vl<0) vl = -vl;
}
void vLRControl(double vl, double vr)
{
	pidControl(vl, 0);
	pidControl(vl, 1);
	pidControl(vr, 2);
	pidControl(vr, 3);
}

arm_cfft_radix4_instance_f32 scfft;
void Init()
{
	arm_cfft_radix4_init_f32(&scfft, 1024, 0, 1);
	//Imu
//	char byte[50];
//  __disable_irq();
//	i2cWrite(0xd0, 0x6b, 0x80);
//	HAL_Delay(100);
//	i2cWrite(0xd0, 0x6b, 0x00);
//	HAL_Delay(100);
//	i2cWrite(0xd0, 0x1b, 0x18);
//	i2cWrite(0xd0, 0x1c, 0x00);
//	__enable_irq();
//	mid = 2.81;//5ms 0.005
//	for(int i=1;i<=10000;++i)
//	{
//		int16_t gx = i2cRead(0xd0, 0x48)+(i2cRead(0xd0, 0x47)<<8);
//		float dyaw = gx * NN * 1000;
//		yaw = yaw + (dyaw+mid)*0.001; //
//		if(dyaw+mid>0) mid -= 0.01;
//		else if(dyaw+mid<0) mid += 0.01;

//		int len = snprintf(byte, sizeof(byte), "test:%d\n", gx);
//		HAL_UART_Transmit_IT(&huart1, (uint8_t*)byte, len);
//		rt_thread_delay(1);
//	}	
	
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);//PWM ON
	HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);//PWM ON
	HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);//PWM ON
	HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);//PWM ON
	 
	kp = 2; //2
	ki = 1*0.3; //1
	kd = 0.065/0.03;
	
	i2cWrite(0x3C, 0x00, 0x58); // 0 10 110 00 = 0101 1000 = 58
	HAL_Delay(10);
	i2cWrite(0x3C, 0x01, 0x00); // 0000 0000
	HAL_Delay(10);
	i2cWrite(0x3C, 0x02, 0x00);
	HAL_Delay(10);
}

void posCalculate(double dX, double dY, double W, double *v, double *w)
{
	double pi = PI;
	posW = atan2(dX, dY);
	*v = (sqrt(dX*dX+dY*dY));
	*w = (posW-W); 
}

void thread3_entry(void *parameter)
{
//	yaw = 0;
//	int cnt = 0;
//	while(1)
//	{
//		int16_t gx = i2cRead(0xd0, 0x48)+(i2cRead(0xd0, 0x47)<<8);
//		float dyaw = gx * NN * 1000;
//		yaw = yaw + (dyaw+mid)*0.001;
//		y += v*cos(yaw);
//		x += v*sin(yaw);
//		rt_thread_delay(1);
//	}
}
int initVector = -1;
float initNode[3];
void adjustUnitVector(float *node, int n)
{
	float sum = 0;
	for(int i=0;i<=n-1;++i)
	{
		sum += node[i]*node[i];
	}
	sum = sqrt(sum);
	if(fabs(sum)<=1e-7)
	{
		for(int i=0;i<=n-1;++i)
		{
			node[i] = -1;
		}
		return ;
	}
	for(int i=0;i<=n-1;++i)
	{
		node[i] = node[i]/sum;
	}
}

float getDeg(float *node, int n)
{
	adjustUnitVector(node, n);
	float sum = 0;
	for(int i=0;i<=n-1;++i)
	{
		sum += initNode[i]*node[i];
	}
	sum = fmaxf(-1.0, fminf(1.0, sum));
	float deg = acos(sum)/PI*180;
	return deg;
}
float U[11];
int svdFlag = 0;
void thread4_entry(void *parameter)
{
	//Magnetometer
	int m, n, num;
	m = 500;
	num = 3;
	n = 2*num+num*(num-1)/2+1;
	float *X = rt_malloc(m*n*sizeof(float));
	float *center = rt_malloc(num*sizeof(float));
	float *Pt = rt_malloc((num*num)*sizeof(float));//	double V[3*3];
	float *Q = rt_malloc((num*num)*sizeof(float));
	float tmpX[num];
	float *Point = rt_malloc(num*sizeof(float));//	double X[3*1]
	float *endPoint = rt_malloc(num*sizeof(float));
	if(X==RT_NULL)
	{
		debug_print("error1\n");
	}
	else 
	{
		debug_print("malloc X normal\n");
	}
	rt_thread_delay(100);
	int cnt = 0;	
	int tot = 0;
	while(1){
//		if(tick_time2<3000)
//		{
//			tot = 0;
//			rt_thread_delay(20);
//			continue;
//		}
		
		uint8_t reg = 0x03;
		uint8_t buffer[6];
		HAL_I2C_Master_Transmit(&hi2c2, 0x3C, &reg, 1, 100);
		HAL_I2C_Master_Receive(&hi2c2, 0x3C	, buffer, sizeof(buffer), 100);
		x = (buffer[0] << 8) | buffer[1];
		y = (buffer[4] << 8) | buffer[5];
		z = (buffer[2] << 8) | buffer[3];
		if(svdFlag==1)
		{
			Point[0] = (float)x-center[0];
			Point[1] = (float)y-center[1];
			Point[2] = (float)z-center[2];
			matrix_muti(Pt, Point, endPoint, num, num, 1);
			
//			debug_print("test: %f, %f, %f\n", tmpX, tmpY, tmpZ
//			debug_print("x, y, z: %d, %d, %d\n", x, y, z);
//			rt_thread_delay(20);
			for(int i=0;i<=num-1;++i)
			{
				endPoint[i] = endPoint[i]/tmpX[i];
			}
			adjustUnitVector(endPoint, num);

			if(initVector==-1)
			{
				initVector = 1;
				for(int i=0;i<=num-1;++i)
				{
					initNode[i] = endPoint[i];
				}
			}
			float theta;
			theta = getDeg(endPoint, num);
			
			debug_print("test: %f, %f, %f, %f\n", (float)endPoint[0], (float)endPoint[1], (float)endPoint[2], theta);
			
			rt_thread_delay(5000);
			continue; 
		}
		//double jiaodu = calMagnetometer(x, y);
		
		++cnt;
		if(cnt==20&&tot<m)
		{
			x = test_ellipsoid[tot][0];
			y = test_ellipsoid[tot][1];
			z = test_ellipsoid[tot][2];
			//debug_print("test:%d, %d\n", cnt, tot);
			X[tot*10+0] = (float)(x*x);
			X[tot*10+1] = (float)(y*y);
			X[tot*10+2] = (float)(z*z);
			X[tot*10+3] = (float)(x*y);
			X[tot*10+4] = (float)(y*z);
			X[tot*10+5] = (float)(x*z);
			X[tot*10+6] = (float)x;
			X[tot*10+7] = (float)y;
			X[tot*10+8] = (float)z;
			X[tot*10+9] = (float)1;
			debug_print("test: %d, %d, %d\n", x, y, z);
//			X[tot*n+0] = (float)(x*x);
//			X[tot*n+1] = (float)(y*y);
//			X[tot*n+2] = (float)(x*y);
//			X[tot*n+3] = (float)x;
//			X[tot*n+4] = (float)y;
//			X[tot*n+5] = (float)1;
//			debug_print("test: %d, %d\n", x, y);
//			for(int i=0;i<=n;++i)
//			{
//				debug_print("%d ", (int)X[tot*n+i]);
//				rt_thread_delay(20);
//			}
////			debug_print("\n");
//			rt_thread_delay(20);
			cnt = 0;
			++tot;
		}
		if(tot==500)
		{
			leastSquareMethod_SvdSolve(X, U);
			debug_print("U\n");
			rt_thread_delay(20);
			for(int i=1;i<=n;++i)
			{
				debug_print("%f ", U[i]);
				rt_thread_delay(1000);
			}
			debug_print("\n");
			if(U[1]<0&&fabs(U[1])>1e-9)
			{
					for(int i=1;i<=n;++i) U[i] = -U[i];
			}
//			debug_print("-----\n");
//			for(int i=1;i<=m;++i)//m
//			{
//				float sum = 0;
//				for(int j=1;j<=n;++j)
//				{
//					sum += X[(i-1)*n+(j-1)]*U[j];
//				}
//				debug_print("%f ", sum);
//				rt_thread_delay(20);
//			}
//			debug_print("\n");
//				rt_thread_delay(20);
			svdFlag = 1;
			remove_hard_iron_offset(&U[1], center);
			debug_print("x0, y0: %f, %f, %f\n", center[0], center[1], center[2]);
			rt_thread_delay(20);
			remove_soft_iron_distortion(&U[1], Q, Pt);
//			debug_print("Q2:\n");
//			rt_thread_delay(20);
//			for(int i=0;i<=2;++i)
//			{
//				for(int j=0;j<=2;++j)
//				{
//					debug_print("%f ", Q[i*n/3+j]);
//					rt_thread_delay(20);
//				}
//				debug_print("\n");
//				rt_thread_delay(20);
//			}
			debug_print("tmpX:\n");
			rt_thread_delay(20);
			for(int i=0;i<=num-1;++i)
			{
				tmpX[i] = sqrt(Q[i*num+i]);
				debug_print("%f ", tmpX[i]);
				rt_thread_delay(20);
			}
			debug_print("\n");
			rt_thread_delay(20);
			rt_free(X);
		}
		rt_thread_delay(20);
	}
	
}

void thread1_entry(void *parameter)
{
	change(30, 0); //-30 , 0.1
	while(1)
	{
//		if(flag_thread4==1)
//		{
//			change(0, 0);
//			break;
//		}
		rt_thread_delay(20);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM14) {
    HAL_IncTick();
  }
	else if(htim->Instance == TIM1) {
		__disable_irq();
		int now = __HAL_TIM_GET_COUNTER(&htim2);
		for(int i=0;i<=3;++i)
		{
			if(time_interval[i]>=1e9)
			{
				if(now-last_state[i]>=1e5)
				{
					last_state[i] = now;
					counterFloatValue[i] = 0;
				}					
			}
		}
		vLRControl(vl*100, vr*100);
		__enable_irq();
		
	}
}

void thread_create()
{
	pi = PI;
	
	uart_tx_sem = rt_sem_create("uart_tx_sem", 1, RT_IPC_FLAG_FIFO);
	data_busy = 1;
	Init();	
		
	rt_thread_init(&thread2, "thread2", debug_thread2_entry, RT_NULL, &thread2_stack[0], sizeof(thread2_stack), 3, 1000000);
	rt_thread_startup(&thread2);
	
	rt_thread_delay(100);
	
	
	rt_thread_t tid1 = rt_thread_create("thread1", thread1_entry, RT_NULL, 512, 0, 1000000);
	rt_thread_startup(tid1);
	
	rt_thread_delay(100);
	
	debug_print("test\n");
	rt_thread_t tid4 = rt_thread_create("thread4", thread4_entry, RT_NULL, 8092, 2, 1000000);
	rt_thread_startup(tid4);
	
//	rt_thread_t tid3 = rt_thread_create("thread3", thread3_entry, RT_NULL, 512, 5, 1000000);
//	rt_thread_startup(tid3);	
	
	

}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        rt_sem_release(uart_tx_sem);
    }
}