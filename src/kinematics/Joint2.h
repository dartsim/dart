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

#ifndef DART_KINEMATICS_JOINT2_H
#define DART_KINEMATICS_JOINT2_H

#include <vector>
#include <Eigen/Dense>

#include "math/UtilsMath.h"
#include "math/UtilsRotation.h"
#include "kinematics/System.h"

namespace kinematics {

class Dof;
class BodyNode;
class Transformation;

/// @brief
///
/// [Members]
/// T: local transformation (4x4 matrix)
/// S: local Jacobian (6xm matrix)
/// dS: localJacobianDerivative (6xm matrix)
/// q: generalized coordinates (configuration) (scalar)
/// dq: generalized velocity (scalar)
/// ddq: generalized acceleration (scalar)
/// tau: generalized force (torque) (scalar)
class Joint2 : public System {
public:
    // We need this aligned allocator because we have Matrix4d as members in
    // this class.
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    enum JointType {
        J_UNKNOWN,
        J_FREEEULER,
        J_FREEEXPMAP,
        J_BALLEULER,
        J_BALLEXPMAP,
        J_HINGE,
        J_UNIVERSAL,
        J_TRANS
    };

    /// @brief
    Joint2();

    /// @brief
    virtual ~Joint2();

protected:

private:

};

} // namespace kinematics

#endif // #ifndef DART_KINEMATICS_JOINT_H

