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
  updateDegreeOfFreedomNames();
}

//==============================================================================
FreeJoint::~FreeJoint()
{
}

//==============================================================================
void FreeJoint::setPositions(const Eigen::VectorXd& _positions)
{
  mQ.linear() = math::expMapRot(_positions.head<3>());
  mQ.translation() = _positions.tail<3>();

  MultiDofJoint::setPositions(_positions);
}

//==============================================================================
Eigen::VectorXd FreeJoint::getPositionDifferences(
    const Eigen::VectorXd& _q0, const Eigen::VectorXd& _q1) const
{
  Eigen::Vector6d dq;

  const Eigen::Matrix3d Jw  = getLocalJacobian(_q0).topLeftCorner<3,3>();
  const Eigen::Matrix3d R0T = math::expMapRot(-_q0.head<3>());
  const Eigen::Matrix3d R1  = math::expMapRot( _q1.head<3>());

  dq.head<3>() = Jw.inverse() * math::logMap(R0T * R1);
  dq.tail<3>() = _q1.tail<3>() - _q0.tail<3>();

  return dq;
}

//==============================================================================
math::Jacobian FreeJoint::getLocalJacobian(
    const Eigen::VectorXd& _positions) const
{
  // Jacobian expressed in the Joint frame
  Eigen::Matrix6d J = Eigen::Matrix6d::Identity();
  J.topLeftCorner<3,3>() = math::expMapJac(-_positions.head<3>());

  // Transform the reference frame to the child BodyNode frame
  J.leftCols<3>()  = math::AdTJacFixed(mT_ChildBodyToJoint, J.leftCols<3>());
  J.bottomRightCorner<3,3>()
      = mT_ChildBodyToJoint.linear() * math::expMapRot(-_positions.head<3>());

  // Note that the top right 3x3 block of J is always zero
  assert((J.topRightCorner<3,3>()) == Eigen::Matrix3d::Zero());

  assert(!math::isNan(J));

  return J;
}

//==============================================================================
void FreeJoint::integratePositions(double _dt)
{
  mQ.linear()      = mQ.linear() * math::expMapRot(mJacobian.topRows<3>()
                                                   * mVelocities * _dt);
  mQ.translation() = mQ.translation() + mVelocities.tail<3>() * _dt;

  mPositions.head<3>() = math::logMap(mQ.linear());
  mPositions.tail<3>() = mQ.translation();
}

//==============================================================================
void FreeJoint::updateDegreeOfFreedomNames()
{
  if(!mDofs[0]->isNamePreserved())
    mDofs[0]->setName(mName + "_rot_x", false);
  if(!mDofs[1]->isNamePreserved())
    mDofs[1]->setName(mName + "_rot_y", false);
  if(!mDofs[2]->isNamePreserved())
    mDofs[2]->setName(mName + "_rot_z", false);
  if(!mDofs[3]->isNamePreserved())
    mDofs[3]->setName(mName + "_pos_x", false);
  if(!mDofs[4]->isNamePreserved())
    mDofs[4]->setName(mName + "_pos_y", false);
  if(!mDofs[5]->isNamePreserved())
    mDofs[5]->setName(mName + "_pos_z", false);
}

//==============================================================================
void FreeJoint::updateLocalTransform()
{
  mQ.linear()      = math::expMapRot(mPositions.head<3>());
  mQ.translation() = mPositions.tail<3>();

  mT = mT_ParentBodyToJoint * mQ * mT_ChildBodyToJoint.inverse();

  assert(math::verifyTransform(mT));
}

//==============================================================================
void FreeJoint::updateLocalJacobian()
{
  mJacobian = getLocalJacobian(mPositions);
}

//==============================================================================
void FreeJoint::updateLocalJacobianTimeDeriv()
{
  Eigen::Matrix<double, 6, 3> J;
  J.topRows<3>()    = Eigen::Matrix3d::Zero();
  J.bottomRows<3>() = Eigen::Matrix3d::Identity();

  Eigen::Matrix<double, 6, 3> dJ;
  dJ.topRows<3>()    = math::expMapJacDot(mPositions.head<3>(),
                                          mVelocities.head<3>()).transpose();
  dJ.bottomRows<3>() = Eigen::Matrix3d::Zero();

  const Eigen::Isometry3d T = mT_ChildBodyToJoint
                              * math::expAngular(-mPositions.head<3>());

  mJacobianDeriv.leftCols<3>() = math::AdTJacFixed(mT_ChildBodyToJoint, dJ);
  mJacobianDeriv.col(3)
      = -math::ad(mJacobian.leftCols<3>() * mVelocities.head<3>(),
                  math::AdT(T, J.col(0)));
  mJacobianDeriv.col(4)
      = -math::ad(mJacobian.leftCols<3>() * mVelocities.head<3>(),
                  math::AdT(T, J.col(1)));
  mJacobianDeriv.col(5)
      = -math::ad(mJacobian.leftCols<3>() * mVelocities.head<3>(),
                  math::AdT(T, J.col(2)));
}

}  // namespace dynamics
}  // namespace dart
