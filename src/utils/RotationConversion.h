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
        Eigen::Matrix3d quatToMatrix(Eigen::Quaterniond& q);

        Eigen::Quaterniond expToQuat(Eigen::Vector3d& v);
        Eigen::Vector3d quatToExp(Eigen::Quaterniond& q);


        // Note: xyz order means matrix is Rz*Ry*Rx i.e a point as transformed as Rz*Ry*Rx(p)
        // coord sys transformation as in GL will be written as glRotate(z); glRotate(y); glRotate(x)
        Eigen::Vector3d matrixToEuler(Eigen::Matrix3d& m, RotationOrder _order);
        Eigen::Matrix3d eulerToMatrix(Eigen::Vector3d& v, RotationOrder _order);

        Eigen::Matrix3d eulerToMatrixX(double x);
        Eigen::Matrix3d eulerToMatrixY(double y);
        Eigen::Matrix3d eulerToMatrixZ(double z);

        Eigen::Vector3d rotatePoint(const Eigen::Quaterniond& q, const Eigen::Vector3d& pt);
        Eigen::Vector3d rotatePoint(const Eigen::Quaterniond& q, double x, double y, double z);

        // TODO: remove from here
        Eigen::Matrix3d getDerivativeMatrix(const Eigen::Quaterniond& q, int el);
        Eigen::Matrix3d getDerivativeMatrix(const Eigen::Quaterniond& q, int el1, int el2);


        // compute expmap stuff
        Eigen::Matrix3d expMapRot(const Eigen::Vector3d &_expmap); ///< computes the Rotation matrix from a given expmap vector
        Eigen::Matrix3d expMapJac(const Eigen::Vector3d &_expmap);  ///< computes the Jacobian of the expmap
        Eigen::Matrix3d expMapJacDot(const Eigen::Vector3d &_expmap, const Eigen::Vector3d &_qdot); ///< computes the time derivative of the expmap Jacobian
        Eigen::Matrix3d expMapJacDeriv(const Eigen::Vector3d &_expmap, int _qi);    ///< computes the derivative of the Jacobian of the expmap wrt to _qi indexed dof; _qi \in {0,1,2}

    } // namespace rot_conv
} // namespace utils

#endif // #ifndef UTILS_ROTATION_CONVERSION_H

