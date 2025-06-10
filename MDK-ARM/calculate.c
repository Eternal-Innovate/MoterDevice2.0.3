#include "calculate.h"
#include "math.h"
#include "usart.h"
#include "stdio.h"
const double D = (5+5.5+13)/100;
void CalculateLR(double Vx, double w, double *vl, double *vr)
{
   //???w??, ???????? w=3.14
   *vl = Vx + w*D/2;
   *vr = Vx - w*D/2;   
}


