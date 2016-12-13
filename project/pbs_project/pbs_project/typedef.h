#pragma once
#include <irrlicht.h>
#include <cmath>

#include "../include/eigen3/Eigen/Dense"
#include "../include/eigen3/Eigen/Sparse"
#include "../include/eigen3/Eigen/Geometry"
#include "../include/eigen3/Eigen/IterativeLinearSolvers"

typedef double real_t;
typedef int int_t;
typedef Eigen::Matrix<real_t, Eigen::Dynamic, Eigen::Dynamic> matrix_t; //matrix with dynamic size
typedef Eigen::SparseMatrix<real_t> smatrix_t; //sparse matrix type, use resize(rows, cols) before setting values (and reserve() for performance)
typedef Eigen::Matrix<real_t, Eigen::Dynamic, 1> vector_t; //col vector or as vector?
typedef Eigen::Matrix<real_t, 3, 3> mat3d; //3x3 matrix
typedef Eigen::Matrix<real_t, 3, 1> vec3d; //3 dimensional column vector
typedef Eigen::Quaternion<real_t,2> quaternion_t; //4 dimensional column vector (i have no idea why to put the 2 in template but doesnt work without)
//typedef Eigen::SparseLU<Eigen::SparseMatrix<real_t>> ssolver; //solver for smatrix_t type
typedef Eigen::ConjugateGradient<Eigen::SparseMatrix<real_t>> ssolver;


//explicit?
using namespace irr;
using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;


inline irr::core::vector3df quattoirr(quaternion_t const &quat)
{
	irr::core::vector3df vec;
	irr::core::quaternion quati(quat.x(), quat.y(), quat.z(), quat.w());
	//
	quati.toEuler(vec);

	// convert radin to degree
	vec.X *= 57.2957795f;
	vec.Y *= 57.2957795f;
	vec.Z *= 57.2957795f;

	return vec;
}


inline quaternion_t eultoquat(vec3d const &eul)
{
	real_t c1 = std::cos(eul.y() / 2.);
	real_t s1 = std::sin(eul.y() / 2.);
	real_t c2 = std::cos(eul.z() / 2.);
	real_t s2 = std::sin(eul.z() / 2.);
	real_t c3 = std::cos(eul.x() / 2.);
	real_t s3 = std::sin(eul.x() / 2.);
	return quaternion_t(c1*c2*c3 - s1*s2*s3, c1*c2*s3 + s1*s2*c3, s1*c2*c3 - c1*s2*s3, c1*s2*c3 - s1*c2*s3);

}