/*
 * Copyright (c) 2013-2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
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

#include "dart/dynamics/PrismaticJoint.h"

#include <string>

#include "dart/math/Geometry.h"
#include "dart/math/Helpers.h"
#include "dart/dynamics/BodyNode.h"

namespace dart {
namespace dynamics {

//==============================================================================
PrismaticJoint::PrismaticJoint(const Eigen::Vector3d& axis,
                               const std::string& _name)
  : SingleDofJoint(_name),
    mAxis(axis.normalized())
{
}

//==============================================================================
PrismaticJoint::~PrismaticJoint()
{
}

//==============================================================================
void PrismaticJoint::setAxis(const Eigen::Vector3d& _axis)
{
  mAxis = _axis.normalized();
}

//==============================================================================
const Eigen::Vector3d& PrismaticJoint::getAxis() const
{
  return mAxis;
}

//==============================================================================
void PrismaticJoint::updateLocalTransform() const
{
  mT = mT_ParentBodyToJoint
       * Eigen::Translation3d(mAxis * getPositionStatic())
       * mT_ChildBodyToJoint.inverse();

  // Verification
  assert(math::verifyTransform(mT));
}

//==============================================================================
void PrismaticJoint::updateLocalJacobian(bool _mandatory) const
{
  if(_mandatory)
  {
    // TODO(JS): This should be updated when mT_ChildBodyToJoint or mAxis.
    mJacobian = math::AdTLinear(mT_ChildBodyToJoint, mAxis);

    // Verification
    assert(!math::isNan(mJacobian));
  }
}

//==============================================================================
void PrismaticJoint::updateLocalJacobianTimeDeriv() const
{
  // Time derivative of prismatic joint is always zero
  assert(mJacobianDeriv == Eigen::Vector6d::Zero());
}

}  // namespace dynamics
}  // namespace dart
