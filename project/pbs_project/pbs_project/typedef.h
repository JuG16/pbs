#pragma once

#include "../include/eigen3/Eigen/Dense"

typedef double real_t;
typedef int int_t;
typedef Eigen::Matrix<real_t, Eigen::Dynamic, Eigen::Dynamic> matrix_t; //matrix with dynamic size
typedef Eigen::Matrix<real_t, 1, Eigen::Dynamic> vector_t; //row vector or as vector?
typedef Eigen::Matrix<real_t, 3, 1> vec3d; //3 dimensional column vector