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
  : Joint(_name)
{
  mGenCoords.push_back(&mCoordinate[0]);
  mGenCoords.push_back(&mCoordinate[1]);
  mGenCoords.push_back(&mCoordinate[2]);
  mGenCoords.push_back(&mCoordinate[3]);
  mGenCoords.push_back(&mCoordinate[4]);
  mGenCoords.push_back(&mCoordinate[5]);

  mS = Eigen::Matrix<double, 6, 6>::Zero();
  Eigen::Matrix6d J = Eigen::Matrix6d::Identity();
  mS.col(0) = math::AdT(mT_ChildBodyToJoint, J.col(0));
  mS.col(1) = math::AdT(mT_ChildBodyToJoint, J.col(1));
  mS.col(2) = math::AdT(mT_ChildBodyToJoint, J.col(2));
  mS.col(3) = math::AdT(mT_ChildBodyToJoint, J.col(3));
  mS.col(4) = math::AdT(mT_ChildBodyToJoint, J.col(4));
  mS.col(5) = math::AdT(mT_ChildBodyToJoint, J.col(5));
  assert(!math::isNan(mS));

  mdS = Eigen::Matrix<double, 6, 6>::Zero();

  mSpringStiffness.resize(6, 0.0);
  mDampingCoefficient.resize(6, 0.0);
  mRestPosition.resize(6, 0.0);
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

  mS.col(0) = math::AdT(mT_ChildBodyToJoint, J.col(0));
  mS.col(1) = math::AdT(mT_ChildBodyToJoint, J.col(1));
  mS.col(2) = math::AdT(mT_ChildBodyToJoint, J.col(2));
  mS.col(3) = math::AdT(mT_ChildBodyToJoint, J.col(3));
  mS.col(4) = math::AdT(mT_ChildBodyToJoint, J.col(4));
  mS.col(5) = math::AdT(mT_ChildBodyToJoint, J.col(5));

  assert(!math::isNan(mS));
}

//==============================================================================
void FreeJoint::integrateConfigs(double _dt)
{
  mQ = mQ * math::expMap(getGenVels() * _dt);

  GenCoordSystem::setConfigs(math::logMap(mQ));
}

//==============================================================================
void FreeJoint::updateTransform()
{
  mQ = math::expMap(getConfigs());

  mT = mT_ParentBodyToJoint * mQ * mT_ChildBodyToJoint.inverse();

  assert(math::verifyTransform(mT));
}

//==============================================================================
void FreeJoint::updateJacobian()
{
  // Jacobian is constant
}

//==============================================================================
void FreeJoint::updateJacobianTimeDeriv()
{
  // Time derivative of Jacobian is constant
  assert(mdS == Eigen::Matrix6d::Zero());
}

}  // namespace dynamics
}  // namespace dart
