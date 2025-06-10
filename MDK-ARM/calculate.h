#ifndef CALCULATE
#define CALCULATE

extern const double D;

struct vLR
{
	double vl;
	double vr;
	
};

#define PI ((16 * atan(1.0 / 5)) - (4 * atan(1.0 / 239)))
void CalculateLR(double Vx, double w, double *vl, double *vr);
void posCalculate(double dX, double dY, double W, double *v, double *w);

#endif