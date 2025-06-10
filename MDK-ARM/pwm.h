#ifndef PWMCONTROL
#define PWMCONTROL

#include "main.h"
#include "calculate.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim9;
extern TIM_HandleTypeDef htim10;
extern TIM_HandleTypeDef htim11;

extern int direct[4];
extern int pwmVal[4];
extern GPIO_TypeDef* GPIO_Id[4];
extern uint16_t GPIO_Group[4][2];
extern TIM_HandleTypeDef *htim_list[4];

void dirControl(double v, int id);
void ControlPwm(int val, int id);
void ControlDirALL(int dir);
void ControlDir(int dir, int id);
void ControlPwmALL(int val);
#endif