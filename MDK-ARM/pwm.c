#include "pwm.h"
#include "math.h"

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;
int pwmVal[4] = {0, 0, 0, 0};
GPIO_TypeDef* GPIO_Id[4] = {GPIOC, GPIOE, GPIOF, GPIOF};
uint16_t GPIO_Group[4][2] = {{GPIO_PIN_2, GPIO_PIN_1}, {GPIO_PIN_1, GPIO_PIN_2}, {GPIO_PIN_3, GPIO_PIN_4}, {GPIO_PIN_2, GPIO_PIN_1}};
TIM_HandleTypeDef *htim_list[4] = {&htim8, &htim9, &htim10, &htim11};
int direct[4] = {0, 0, 0, 0};

void dirControl(double v, int id)
{
	if(v>0) ControlDir(-1, id);
	else if(v<0) ControlDir(1, id);
	else ControlDir(0, id);
}

void ControlDir(int dir, int id)
{
  if(dir==0)
  {
    HAL_GPIO_WritePin(GPIO_Id[id], GPIO_Group[id][0], GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIO_Id[id], GPIO_Group[id][1], GPIO_PIN_RESET);
		direct[id] = 0;
  }
  else if(dir>0)
  {
     HAL_GPIO_WritePin(GPIO_Id[id], GPIO_Group[id][0], GPIO_PIN_RESET);
     HAL_GPIO_WritePin(GPIO_Id[id], GPIO_Group[id][1], GPIO_PIN_SET);
		 direct[id] = 1;
	}
  else if(dir<0)
  {
    HAL_GPIO_WritePin(GPIO_Id[id], GPIO_Group[id][0], GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_Id[id], GPIO_Group[id][1], GPIO_PIN_RESET);
		direct[id] = -1;
	}
}

void ControlDirALL(int dir)
{
	for(int i=0;i<=3;++i) ControlDir(dir, i);
}

void ControlPwm(int val, int id)
{
   if (val > 1000)
   {
     val = 1000;
		 pwmVal[id] = 1000;
   }
	 if(val<0)
	 {
		 val = 0;
		 pwmVal[id] = 0;
	 }
		pwmVal[id] = val;
			__HAL_TIM_SET_COMPARE(htim_list[id], TIM_CHANNEL_1, pwmVal[id]);
}

void ControlPwmALL(int val)
{
	if(val>1000)
	{
		val = 1000;
	}
	for(int i=0;i<=3;++i)
	{
		ControlPwm(val, i);
	}
}
