#ifndef THREADCONTROL
#define THREADCONTROL
#include "rtthread.h"
#include "pwm.h"
#include "usart.h"
#include "calculate.h"
#include "i2cImu.h"
#include "magnetometer.h"


static struct rt_thread thread2;
static char thread2_stack[1024];
void thread_create();
void vLRControl(double vl, double vr);
#endif