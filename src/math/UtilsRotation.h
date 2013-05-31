/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@gmail.com>
 * Date: 06/12/2011
 *
 * Geoorgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef MATH_UTILSROTATION_H
#define MATH_UTILSROTATION_H

// External Libraries
#include <Eigen/Dense>
// Local Headers
#include "utils/Misc.h"

namespace dart_math {

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

    // quaternion stuff
    Eigen::Matrix3d quatDeriv(const Eigen::Quaterniond& q, int el);
    Eigen::Matrix3d quatSecondDeriv(const Eigen::Quaterniond& q, int el1, int el2);


    // compute expmap stuff
    Eigen::Matrix3d expMapRot(const Eigen::Vector3d &_expmap); ///< computes the Rotation matrix from a given expmap vector
    Eigen::Matrix3d expMapJac(const Eigen::Vector3d &_expmap);  ///< computes the Jacobian of the expmap
    Eigen::Matrix3d expMapJacDot(const Eigen::Vector3d &_expmap, const Eigen::Vector3d &_qdot); ///< computes the time derivative of the expmap Jacobian
    Eigen::Matrix3d expMapJacDeriv(const Eigen::Vector3d &_expmap, int _qi);    ///< computes the derivative of the Jacobian of the expmap wrt to _qi indexed dof; _qi \in {0,1,2}

} // namespace utils

#endif // #ifndef UTILS_UTILSROTATION_H

