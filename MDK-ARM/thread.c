#include "thread.h"
#include "math.h"
#include "stdio.h"
#include "rtthread.h"
/*

		char byte[20];
		snprintf(byte, sizeof(byte), "test:%.3f, %.3f\n", mid, yaw);
		HAL_UART_Transmit_IT(&huart1, (uint8_t*)byte, 20);

*/
int tot, flag_thread4 = 0;
static rt_sem_t uart_tx_sem = RT_NULL;
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

void Init()
{
	
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
//		float dyaw = gx * N * 1000;
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
//		float dyaw = gx * N * 1000;
//		yaw = yaw + (dyaw+mid)*0.001;
//		y += v*cos(yaw);
//		x += v*sin(yaw);
//		rt_thread_delay(1);
//	}
}

void thread4_entry(void *parameter)
{
	//Magnetometer
	int cnt = 0;
	double X[9000], beta[1000];
	tot = 0;
	tick_time = 0;
	while(1){
		if(tick_time<3000)
		{
			tot = 0;
			rt_thread_delay(20);
			continue;
		}
		else if(tick_time>5000) //5ms * 1000 * 5 = 25s 5s
		{
			break;
		}
		uint8_t reg = 0x03;
		uint8_t buffer[6];
		HAL_I2C_Master_Transmit(&hi2c2, 0x3C, &reg, 1, 100);
		HAL_I2C_Master_Receive(&hi2c2, 0x3C	, buffer, sizeof(buffer), 100);
		x = (buffer[0] << 8) | buffer[1];
    y = (buffer[4] << 8) | buffer[5];
		z = (buffer[2] << 8) | buffer[3];
		//double jiaodu = calMagnetometer(x, y);
//		++cnt;
//		if(cnt==5&&tot<1000)
//		{
//			X[tot*9+0] = (double)x;
//			X[tot*9+1] = (double)y;
//			X[tot*9+2] = (double)z;
//			++tot;
//		}
		rt_thread_delay(20);
	}
//	LeastSquaresFit(X, beta, tot, 9);
//	flag_thread4 = 1;
//	O_x = sumx/tot;
//	O_y = sumy/tot;
//	while(1)
//	{
//		uint8_t reg = 0x03;
//		uint8_t buffer[6];
//		HAL_I2C_Master_Transmit(&hi2c2, 0x3C, &reg, 1, 100);
//		HAL_I2C_Master_Receive(&hi2c2, 0x3C	, buffer, sizeof(buffer), 100);
//		x = (buffer[0] << 8) | buffer[1];
//		y = (buffer[4] << 8) | buffer[5];
//		z = (buffer[2] << 8) | buffer[3];
//		tx = x-O_x;
//		ty = y-O_y;
//		jiaodu = calMagnetometer(tx, ty);
//		rt_thread_delay(20);
//	}
}

void thread1_entry(void *parameter)
{
	change(30, 0); //-30 , 0.1
	while(1)
	{
		if(flag_thread4==1)
		{
			change(0, 0);
			break;
		}
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
		++tick_time;
		if(tick_time>=50000)
		{
			tick_time = 0;
		}
		__enable_irq();
		
	}
}

void thread2_entry(void *parameter)
{
	char byte[80];
	while(1)
	{
		
		rt_sem_take(uart_tx_sem, RT_WAITING_FOREVER);
		__disable_irq();
    int len = snprintf(byte, sizeof(byte), "test:%d, %d\n", x, y);
    HAL_UART_Transmit_IT(&huart1, (uint8_t*)byte, len);
		__enable_irq();
		rt_thread_delay(10);
	}
}


void thread_create()
{
	pi = PI;
	uart_tx_sem = rt_sem_create("uart_tx_sem", 1, RT_IPC_FLAG_FIFO);

	Init();	
	
	rt_thread_t tid1 = rt_thread_create("thread1", thread1_entry, RT_NULL, 512, 0, 1000000);
	rt_thread_startup(tid1);
	
	rt_thread_t tid4 = rt_thread_create("thread4", thread4_entry, RT_NULL, 512, 2, 1000000);
	rt_thread_startup(tid4);
	
	rt_thread_init(&thread2, "thread2", thread2_entry, RT_NULL, &thread2_stack[0], sizeof(thread2_stack), 3, 1000000);
	rt_thread_startup(&thread2);
	
	
		

	

	
//	rt_thread_t tid3 = rt_thread_create("thread3", thread3_entry, RT_NULL, 512, 5, 1000000);
//	rt_thread_startup(tid3);	
	
	

}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        rt_sem_release(uart_tx_sem);
    }
}