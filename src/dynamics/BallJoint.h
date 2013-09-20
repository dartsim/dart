/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
 * Date: 05/21/2013
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

#ifndef DART_DYNAMICS_BALL_JOINT_H
#define DART_DYNAMICS_BALL_JOINT_H

#include <Eigen/Dense>

#include "dynamics/GenCoord.h"
#include "dynamics/Joint.h"

namespace dart {
namespace dynamics {

class BallJoint : public Joint
{
public:
    /// @brief Default constructor.
    BallJoint(BodyNode* _parent = NULL, BodyNode* _child = NULL,
              const std::string& _name = "Ball joint");

    /// @brief Default destructor.
    virtual ~BallJoint();

    /// @brief Get a point on the rotation axis in world coordinate frame.
    /// ODE calls this as anchor.
    Eigen::Vector3d getWorldOrigin() const;

    // Documentation is inherited.
    virtual double getPotentialEnergy() const { return 0.0; }

protected:
    // Document inherited.
    virtual void _updateTransform();

    // Document inherited.
    virtual void _updateVelocity();

    // Document inherited.
    virtual void _updateAcceleration();

    /// @brief Generalized coordinates.
    GenCoord mCoordinate[3];
};

} // namespace dynamics
} // namespace dart

#endif // #ifndef DART_DYNAMICS_BALL_JOINT_H

