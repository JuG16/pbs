//=============================================================================
//  Physically-based Simulation in Computer Graphics
//  ETH Zurich
//=============================================================================

#include "gauss_seidel.h"
#include "fluid2d.h"
#include "Utilities/Array2T.h"
#include "Utilities/Vector2T.h"
#include <iomanip>



typedef Vector2T<double> Vec2;
inline double bilinearinterpolation(double x1, double y1, double x2, double y2, double &val11, double &val12, double &val21, double &val22, Vec2 &pos)
{
	double f1 = (x2 - pos.x()) / (x2 - x1)*val11 + (pos.x() - x1) / (x2 - x1)*val21;
	double f2 = (x2 - pos.x()) / (x2 - x1)*val12 + (pos.x() - x1) / (x2 - x1)*val22;
	return (y2 - pos.y()) / (y2 - y1)*f1 + (pos.y() - y1) / (y2 - y1)*f2;
}

inline void project(double &xold, double &yold, double dx)
{
	if (xold <1.5*dx)
	{
		xold = 1.5*dx;
	}
	else if (xold > 1. - 1.5*dx)
	{
		xold = 1. - 1.5*dx;
	}

	if (yold < 1.5*dx)
	{
		yold = 1.5*dx;
	}
	else if (yold > 1. - 1.5*dx)
	{
		yold = 1. - 1.5*dx;
	}
}
// Problem 1
void ExSolvePoisson(int _xRes, int _yRes, int _iterations, double _accuracy, Array2d &_field, Array2d &_b)
{
	
	double dx = 1.0 / _xRes;
	// far better convergence without zero intitial condition
	//_field.zero();
	for(int it = 0; it < _iterations; ++it)
	{
		// Note that the boundaries are handles by the framework, so you iterations should be similar to:
		for (int y = 1; y < _yRes - 1; y++)
		{
			for (int x = 1; x < _xRes - 1; x++)
			{
				// Formula according to exercise slide 5, d_{i,j} is given with _b
				_field(x,y) = ( dx*dx*_b(x,y) + _field(x-1,y) + _field(x,y-1) + _field(x+1,y) + _field(x,y+1) ) / 4.0;
			}
		}
		// compute the residual, needs a second loop
		double residual = 0;
		for (int y = 1; y < _yRes - 1; y++)
		{
			for (int x = 1; x < _xRes - 1; x++)
			{
				residual += std::abs(_b(x,y)
							- (4*_field(x,y) - _field(x+1,y) - _field(x,y+1) - _field(x-1,y) - _field(x,y-1))/(dx*dx));
			}
		}
		residual /= (_yRes-2)*(_xRes-2);	// averaging
		// For your debugging, and ours, please add these prints after every iteration
		if(it == _iterations - 1) 
			printf("Pressure solver: it=%d , res=%f \n", it, residual);
		if(residual < _accuracy)
		{
			printf("Pressure solver: it=%d , res=%f, converged \n", it, residual);
			break;	// terminate iterative solver
		}
	}

	/*for (int i = 0; i < _xRes; ++i)
	{
		for (int j = 0; j < _yRes; ++j)
		{
			std::cout << std::setw(15) << _field(i, j);
		}
		std::cout << std::endl;
	}*/
	
	
}

// Problem 2
//how to do interpolation if not allowed to read values of boundary
void ExCorrectVelocities(int _xRes, int _yRes, double _dt, const Array2d &_pressure, Array2d &_xVelocity, Array2d &_yVelocity)
{
	double dx = 1.0 / _xRes;

	for (int i = 0; i < _xRes; ++i)
	{
		for (int j = 0; j < _yRes; ++j)
		{
			_xVelocity(i + 1, j) -= _dt*(_pressure(i + 1, j) - _pressure(i, j)) / dx;
		}
	}
	for (int i = 0; i < _xRes; ++i)
	{
		for (int j = 0; j < _yRes; ++j)
		{
			_yVelocity(i, j + 1) -= _dt*(_pressure(i, j + 1) - _pressure(i, j)) / dx;
		}
	}

	// Note: velocity u_{i+1/2} is practically stored at i+1
}

// Problem 3
void ExAdvectWithSemiLagrange(int _xRes, int _yRes, double _dt, Array2d &_xVelocity, Array2d &_yVelocity, Array2d &_density, Array2d &_densityTemp, Array2d &_xVelocityTemp, Array2d &_yVelocityTemp)
{
	//dy==dx always?
	double dx = 1. / _xRes;
	//pressure

	for (int i = 1; i < _xRes-1; ++i)
	{
		for (int j = 1; j < _yRes-1; ++j)
		{
			//bilinear interpolation gets reduced to linear interpolation (midpoint)
			double xold = (i + 0.5)*dx - _dt*(_xVelocity(i, j) + _xVelocity(i + 1, j)) / 2.;
			double yold = (j + 0.5)*dx - _dt*(_yVelocity(i, j) + _yVelocity(i, j + 1)) / 2.;
			project(xold, yold, dx);
			int ifloor = std::floor(0.5 + xold / dx);
			int jfloor = std::floor(0.5 + yold / dx);
			_densityTemp(i, j) = bilinearinterpolation((ifloor - 0.5)*dx, (jfloor - 0.5)*dx, (ifloor+0.5)*dx, (jfloor+0.5)*dx, _density(ifloor - 1, jfloor - 1), _density(ifloor - 1, jfloor), _density(ifloor, jfloor - 1), _density(ifloor, jfloor), Vec2(xold, yold));
		}
	}



	// Note: velocity u_{i+1/2} is practically stored at i+1
	//xvelocity
	for (int i = 1; i < _xRes; ++i)
	{
		for (int j = 1; j < _yRes-1; ++j)
		{
			double xold = i*dx-_dt*_xVelocity(i,j);
			double yold = (j + 0.5)*dx - _dt*(_yVelocity(i - 1, j) + _yVelocity(i - 1, j + 1) + _yVelocity(i, j) + _yVelocity(i, j + 1)) / 4;
			project(xold, yold, dx);
			int ifloor = std::floor(xold / dx);
			int jfloor = std::floor(0.5 + yold / dx);
			_xVelocityTemp(i, j) = bilinearinterpolation(ifloor*dx, (jfloor - 0.5)*dx, (ifloor+1)*dx, (jfloor+0.5)*dx, _xVelocity(ifloor, jfloor-1), _xVelocity(ifloor, jfloor), _xVelocity(ifloor + 1, jfloor-1), _xVelocity(ifloor + 1, jfloor), Vec2(xold, yold));
		}
	}

	//yvelocity
	for (int i = 1; i < _xRes-1; ++i)
	{
		for (int j = 1; j < _yRes; ++j)
		{
			double xold = (i + 0.5)*dx - _dt*(_xVelocity(i, j - 1) + _xVelocity(i, j) + _xVelocity(i + 1, j - 1) + _xVelocity(i + 1, j)) / 4;
			double yold = j*dx - _dt*_yVelocity(i, j);
			project(xold, yold, dx);
			int ifloor = std::floor(0.5 + xold / dx);
			int jfloor = std::floor(yold / dx);
			_yVelocityTemp(i, j) = bilinearinterpolation((ifloor-0.5)*dx,  jfloor*dx, (ifloor+0.5)*dx, (jfloor+1)*dx, _yVelocity(ifloor-1, jfloor), _yVelocity(ifloor-1, jfloor + 1), _yVelocity(ifloor, jfloor), _yVelocity(ifloor, jfloor + 1), Vec2(xold, yold));
		}
	}

	
	std::swap(_densityTemp, _density);
	std::swap(_xVelocityTemp, _xVelocity);
	std::swap(_yVelocityTemp, _yVelocity);

	for (int i = 0; i < _xRes; ++i)
	{
		_density(i, 0) = _density(i, 1);
		_density(i, _yRes - 1) = _density(i, _yRes - 2);
	}
	for (int i = 0; i < _yRes; ++i)
	{
		_density(0, i) = _density(1, i);
		_density(_xRes - 1, i) = _density(_xRes - 2, i);
	}
	/*for (int i = 1; i < _xRes-1; ++i)
	{
		for (int j = 1; j < _yRes-1; ++j)
		{
			_density(i, j) = _densityTemp(i, j);
		}
	}
	for (int i = 1; i < _xRes; ++i)
	{
		for (int j = 1; j < _yRes-1; ++j)
		{
			_xVelocity(i, j) = _xVelocityTemp(i, j);
		}
	}
	for (int i = 1; i < _xRes-1; ++i)
	{
		for (int j = 1; j < _yRes; ++j)
		{
			_yVelocity(i, j) = _yVelocityTemp(i, j);
		}
	}*/

	
	
}
