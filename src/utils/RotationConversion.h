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
// Local Headers
#include "Misc.h"

namespace utils {
    namespace rot_conv {

        enum RotationOrder {UNKNOWN, XYZ, XZY, YZX, YXZ, ZXY, ZYX};

        Eigen::Quaterniond matrixToQuat(Eigen::Matrix3d& m);	// forms the Quaterniond from a rotation matrix
        Eigen::Quaterniond expToQuat(Eigen::Vector3d& v);
        Eigen::Vector3d quatToExp(Eigen::Quaterniond& q);
        Eigen::Matrix3d quatToMatrix(Eigen::Quaterniond& q);

        // Note: xyz order means matrix is Rz*Ry*Rx i.e a point as transformed as Rz*Ry*Rx(p)
        // coord sys transformation as in GL will be written as glRotate(z); glRotate(y); glRotate(x)
        Eigen::Vector3d matrixToEuler(Eigen::Matrix3d& m, RotationOrder _order);
        Eigen::Matrix3d eulerToMatrix(Eigen::Vector3d& v, RotationOrder _order);

        Eigen::Matrix3d eulerToMatrixx(double x);
        Eigen::Matrix3d eulerToMatrixy(double y);
        Eigen::Matrix3d eulerToMatrixz(double z);

        // Yuting: I don't think we need it, and it shoudn't be here even if we do
        // Note: retrieves the derivative matrix given a quaternion
        // This is not functionality that is part of Eigen
        Eigen::Matrix3d getDerivativeMatrix(const Eigen::Quaterniond& q, int el);
        Eigen::Matrix3d getDerivativeMatrix(const Eigen::Quaterniond& q, int el1, int el2);

    } // namespace rot_conv
} // namespace utils

#endif // #ifndef UTILS_ROTATION_CONVERSION_H

