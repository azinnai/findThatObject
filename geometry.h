#ifndef GEOMETRY_H_INCLUDED
#define GEOMETRY_H_INCLUDED

#include <string>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Core>

using namespace Eigen;

Matrix4d v2t(const VectorXd& x);
Matrix3d Rx(double alpha);
Matrix3d Ry(double alpha);
Matrix3d Rz(double alpha);

#endif // GEOMETRY_H_INCLUDED
