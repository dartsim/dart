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

#include "dart/dynamics/BallJoint.h"

#include <string>

#include "dart/math/Helpers.h"
#include "dart/math/Geometry.h"

namespace dart {
namespace dynamics {

//==============================================================================
BallJoint::BallJoint(const std::string& _name)
  : MultiDofJoint(_name),
    mR(Eigen::Isometry3d::Identity())
{
  updateDegreeOfFreedomNames();
}

//==============================================================================
BallJoint::~BallJoint()
{
}

//==============================================================================
Eigen::Isometry3d BallJoint::convertToTransform(
    const Eigen::Vector3d& _positions)
{
  return Eigen::Isometry3d(convertToRotation(_positions));
}

//==============================================================================
Eigen::Matrix3d BallJoint::convertToRotation(const Eigen::Vector3d& _positions)
{
  return math::expMapRot(_positions);
}

//==============================================================================
void BallJoint::integratePositions(double _dt)
{
  mR.linear() = mR.linear() * convertToRotation(mJacobian.topRows<3>()
                                                * mVelocities * _dt);

  mPositions = convertToPositions(mR.linear());
}

//==============================================================================
void BallJoint::updateDegreeOfFreedomNames()
{
  if(!mDofs[0]->isNamePreserved())
    mDofs[0]->setName(mName + "_x", false);
  if(!mDofs[1]->isNamePreserved())
    mDofs[1]->setName(mName + "_y", false);
  if(!mDofs[2]->isNamePreserved())
    mDofs[2]->setName(mName + "_z", false);
}

//==============================================================================
void BallJoint::updateLocalTransform()
{
  mR.linear() = convertToRotation(mPositions);

  mT = mT_ParentBodyToJoint * mR * mT_ChildBodyToJoint.inverse();

  assert(math::verifyTransform(mT));
}

//==============================================================================
void BallJoint::updateLocalJacobian()
{
  Eigen::Matrix<double, 6, 3> J;
  J.topRows<3>()    = math::expMapJac(mPositions).transpose();
  J.bottomRows<3>() = Eigen::Matrix3d::Zero();

  mJacobian = math::AdTJacFixed(mT_ChildBodyToJoint, J);

  assert(!math::isNan(mJacobian));
}

//==============================================================================
void BallJoint::updateLocalJacobianTimeDeriv()
{
  Eigen::Matrix<double, 6, 3> dJ;
  dJ.topRows<3>()    = math::expMapJacDot(mPositions, mVelocities).transpose();
  dJ.bottomRows<3>() = Eigen::Matrix3d::Zero();

  mJacobianDeriv = math::AdTJacFixed(mT_ChildBodyToJoint, dJ);

  assert(!math::isNan(mJacobianDeriv));
}

}  // namespace dynamics
}  // namespace dart

