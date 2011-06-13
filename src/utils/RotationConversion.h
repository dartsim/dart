/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sehoon Ha
  Date		06/12/2011
*/

#ifndef UTILS_ROTATION_CONVERSION_H
#define UTILS_ROTATION_CONVERSION_H

// External Libraries
#include <Eigen/Dense>
using namespace Eigen;
// Local Headers
#include "Misc.h"

namespace utils {
    namespace rot_conv {

        enum RotationOrder {XYZ, XZY, YZX, YXZ, ZXY, ZYX};

        Quaterniond matrixToQuat(Matrix3d& m);	// forms the Quaterniond from a rotation matrix
        Quaterniond expToQuat(Vector3d& v);
        Vector3d quatToExp(Quaterniond& q);
        Matrix3d quatToMatrix(Quaterniond& q);

        // Note: xyz order means matrix is Rz*Ry*Rx i.e a point as transformed as Rz*Ry*Rx(p)
        // coord sys transformation as in GL will be written as glRotate(z); glRotate(y); glRotate(x)
        Vector3d matrixToEuler(Matrix3d& m, RotationOrder _order);
        Matrix3d eulerToMatrix(Vector3d& v, RotationOrder _order);

        Matrix3d eulerToMatrixx(double x);
        Matrix3d eulerToMatrixy(double y);
        Matrix3d eulerToMatrixz(double z);

        // Note: retrieves the derivative matrix given a quaternion
        // This is not functionality that is part of Eigen
        Matrix3d getDerivativeMatrix(const Quaterniond& q, int el);
        Matrix3d getDerivativeMatrix(const Quaterniond& q, int el1, int el2);

    } // namespace rot_conv
} // namespace utils

#endif // #ifndef UTILS_ROTATION_CONVERSION_H

