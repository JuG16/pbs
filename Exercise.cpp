//=============================================================================
//  Physically-based Simulation in Computer Graphics
//  ETH Zurich
//=============================================================================

#include "Utilities/Vector2T.h"

// Gravitational acceleration (9.81 m/s^2)
static const double g = 9.81;

// Exercise 1
// Hanging mass point
void AdvanceTimeStep1(double k, double m, double d, double L, double dt, int method, double p1, double v1, double& p2, double& v2)
{
	// Remark: The parameter 'dt' is the duration of the time step, unless the analytic 
	//         solution is requested, in which case it is the absolute time.
	switch (method)
	{
		double v3;
		case 1:
			//explicit Euler:
			p2 = p2 + dt*v2;
			v2 = v2 + dt*(((-k*(p2 - L) + (m*g)) - d*v2) / m);
			break;
		case 2:
			//symplectic Euler:
			p2 = p2 + dt*v2;
		case 3:
			//explicit midpoint:
			p2 = p2 + dt*v2;
			v3 = v2 + (dt/2)*(((-k*(p2 - L) + (m*g)) - d*v2) / m);
			v2=v2+dt*(((-k*(p2 - L) + (m*g)) - d*v3) / m);
			break;
		case 4:
			//semi-implicit Euler:
			p2 = p2 + dt*v2;
		default:
			break;
	}
}

// Exercise 3
// Falling triangle
void AdvanceTimeStep3(double k, double m, double d, double L, double dt,
                      Vec2& p1, Vec2& v1, Vec2& p2, Vec2& v2, Vec2& p3, Vec2& v3)
{
	p1 += Vec2(1,1);
}
