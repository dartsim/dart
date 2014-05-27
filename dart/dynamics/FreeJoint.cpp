/*
 * Copyright (c) 2013-2014, Georgia Tech Research Corporation
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

#include "dart/dynamics/FreeJoint.h"

#include <string>

#include "dart/math/Helpers.h"
#include "dart/math/Geometry.h"

namespace dart {
namespace dynamics {

//==============================================================================
FreeJoint::FreeJoint(const std::string& _name)
  : MultiDofJoint(_name),
    mQ(Eigen::Isometry3d::Identity())
{
  // Jacobian
  Eigen::Matrix6d J = Eigen::Matrix6d::Identity();
  mJacobian.col(0) = math::AdT(mT_ChildBodyToJoint, J.col(0));
  mJacobian.col(1) = math::AdT(mT_ChildBodyToJoint, J.col(1));
  mJacobian.col(2) = math::AdT(mT_ChildBodyToJoint, J.col(2));
  mJacobian.col(3) = math::AdT(mT_ChildBodyToJoint, J.col(3));
  mJacobian.col(4) = math::AdT(mT_ChildBodyToJoint, J.col(4));
  mJacobian.col(5) = math::AdT(mT_ChildBodyToJoint, J.col(5));
  assert(!math::isNan(mJacobian));

  // TODO(JS): Deprecated
  mS = mJacobian;

  // Time derivative of Jacobian is always zero

  // TODO(JS): Deprecated
  mdS = mJacobianDeriv;
}

//==============================================================================
FreeJoint::~FreeJoint()
{
}

//==============================================================================
void FreeJoint::setTransformFromChildBodyNode(const Eigen::Isometry3d& _T)
{
  Joint::setTransformFromChildBodyNode(_T);

  Eigen::Matrix6d J = Eigen::Matrix6d::Identity();

  mJacobian.col(0) = math::AdT(mT_ChildBodyToJoint, J.col(0));
  mJacobian.col(1) = math::AdT(mT_ChildBodyToJoint, J.col(1));
  mJacobian.col(2) = math::AdT(mT_ChildBodyToJoint, J.col(2));
  mJacobian.col(3) = math::AdT(mT_ChildBodyToJoint, J.col(3));
  mJacobian.col(4) = math::AdT(mT_ChildBodyToJoint, J.col(4));
  mJacobian.col(5) = math::AdT(mT_ChildBodyToJoint, J.col(5));

  // TODO(JS): Deprecated
  mS = mJacobian;

  assert(!math::isNan(mJacobian));
}

//==============================================================================
void FreeJoint::integrateConfigs(double _dt)
{
  mQ = mQ * math::expMap(getGenVels() * _dt);

  GenCoordSystem::setConfigs(math::logMap(mQ));
}

//==============================================================================
void FreeJoint::updateLocalTransform()
{
  mQ = math::expMap(getConfigs());

  mT = mT_ParentBodyToJoint * mQ * mT_ChildBodyToJoint.inverse();

  assert(math::verifyTransform(mT));
}

//==============================================================================
void FreeJoint::updateLocalJacobian()
{
  // Do nothing since Jacobian is constant
}

//==============================================================================
void FreeJoint::updateLocalJacobianTimeDeriv()
{
  // Time derivative of Jacobian is constant
  assert(mJacobianDeriv == (Eigen::Matrix6d::Zero()));
}

}  // namespace dynamics
}  // namespace dart
