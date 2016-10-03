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
		v2 = v2 + dt*((-k*(p2 + L) - m*g - d*v2) / m);
		break;
	case 2:
		//symplectic Euler:
		v2 = v2 + dt*((-k*(p2 + L) - m*g - d*v2) / m);
		p2 = p2 + dt*v2;
		break;
	case 3:
		//explicit midpoint:
		p2 = p2 + dt*v2;
		v3 = v2 + (dt / 2)*((-k*(p2 + L) - m*g - d*v2) / m);
		v2 = v2 + dt*(((-k*(p2 + L) - (m*g)) - d*v3) / m);
		break;
	case 4:
		//implicit Euler
		v2 = (v2*m/dt -k*(p2 + L) - m*g)/(m/dt+d);
		p2 = p2 + dt*v2;
		break;
	case 5:
	{
		//analytic
		//p2 = p2 + dt*v2;
		//v2 = v2 + dt*(((-k*(p2 - L) + (m*g)) - d*v2) / m);
		double alpha = -d / 2 * m;
		double beta = std::sqrt(4 * k*m - d*d) / (2 * m);
		p2 = m*g / k*std::exp(alpha*dt)*std::cos(beta*dt) + d*g / (2 * k)*std::exp(alpha*dt)*std::sin(beta*dt) - L - m*g / k;
	}

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
