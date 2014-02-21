/*
 * Copyright (c) 2013, Georgia Tech Research Corporation
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

FreeJoint::FreeJoint(const std::string& _name)
  : Joint(FREE, _name),
    mT_Joint(Eigen::Isometry3d::Identity()) {
  mGenCoords.push_back(&mCoordinate[0]);
  mGenCoords.push_back(&mCoordinate[1]);
  mGenCoords.push_back(&mCoordinate[2]);
  mGenCoords.push_back(&mCoordinate[3]);
  mGenCoords.push_back(&mCoordinate[4]);
  mGenCoords.push_back(&mCoordinate[5]);

  mS = Eigen::Matrix<double, 6, 6>::Zero();
  mdS = Eigen::Matrix<double, 6, 6>::Zero();

  mSpringStiffness.resize(6, 0.0);
  mDampingCoefficient.resize(6, 0.0);
  mRestPosition.resize(6, 0.0);
}

FreeJoint::~FreeJoint() {
}

void FreeJoint::updateTransform() {
  // TODO(JS): This is workaround for Issue #122.
  mT_Joint = math::expMap(get_q());

  mT = mT_ParentBodyToJoint * mT_Joint * mT_ChildBodyToJoint.inverse();

  assert(math::verifyTransform(mT));
}

void FreeJoint::updateTransform_Issue122(double _timeStep) {
  mT_Joint = mT_Joint * math::expMap(_timeStep * get_dq());

  set_q(math::logMap(mT_Joint));

  mT = mT_ParentBodyToJoint * mT_Joint * mT_ChildBodyToJoint.inverse();

  assert(math::verifyTransform(mT));
}

void FreeJoint::updateJacobian() {
  Eigen::Vector3d q(mCoordinate[0].get_q(),
                    mCoordinate[1].get_q(),
                    mCoordinate[2].get_q());

  Eigen::Matrix3d J = math::expMapJac(q);

  Eigen::Vector6d J0;
  Eigen::Vector6d J1;
  Eigen::Vector6d J2;
  Eigen::Vector6d J3;
  Eigen::Vector6d J4;
  Eigen::Vector6d J5;

  J0 << J(0, 0), J(0, 1), J(0, 2), 0, 0, 0;
  J1 << J(1, 0), J(1, 1), J(1, 2), 0, 0, 0;
  J2 << J(2, 0), J(2, 1), J(2, 2), 0, 0, 0;
  J3 << 0, 0, 0, 1, 0, 0;
  J4 << 0, 0, 0, 0, 1, 0;
  J5 << 0, 0, 0, 0, 0, 1;

  mS.col(0) = math::AdT(mT_ChildBodyToJoint, J0);
  mS.col(1) = math::AdT(mT_ChildBodyToJoint, J1);
  mS.col(2) = math::AdT(mT_ChildBodyToJoint, J2);
  mS.col(3) = math::AdT(mT_ChildBodyToJoint * math::expAngular(-q), J3);
  mS.col(4) = math::AdT(mT_ChildBodyToJoint * math::expAngular(-q), J4);
  mS.col(5) = math::AdT(mT_ChildBodyToJoint * math::expAngular(-q), J5);

  assert(!math::isNan(mS));
}

void FreeJoint::updateJacobian_Issue122() {
  Eigen::Vector6d J0 = Eigen::Vector6d::Zero();
  Eigen::Vector6d J1 = Eigen::Vector6d::Zero();
  Eigen::Vector6d J2 = Eigen::Vector6d::Zero();
  Eigen::Vector6d J3 = Eigen::Vector6d::Zero();
  Eigen::Vector6d J4 = Eigen::Vector6d::Zero();
  Eigen::Vector6d J5 = Eigen::Vector6d::Zero();

  J0[0] = 1.0;
  J1[1] = 1.0;
  J2[2] = 1.0;
  J3[3] = 1.0;
  J4[4] = 1.0;
  J5[5] = 1.0;

  mS.col(0) = math::AdT(mT_ChildBodyToJoint, J0);
  mS.col(1) = math::AdT(mT_ChildBodyToJoint, J1);
  mS.col(2) = math::AdT(mT_ChildBodyToJoint, J2);
  mS.col(3) = math::AdT(mT_ChildBodyToJoint, J3);
  mS.col(4) = math::AdT(mT_ChildBodyToJoint, J4);
  mS.col(5) = math::AdT(mT_ChildBodyToJoint, J5);

  assert(!math::isNan(mS));
}

void FreeJoint::updateJacobianTimeDeriv() {
  Eigen::Vector3d q(mCoordinate[0].get_q(),
                    mCoordinate[1].get_q(),
                    mCoordinate[2].get_q());
  Eigen::Vector3d dq(mCoordinate[0].get_dq(),
                     mCoordinate[1].get_dq(),
                     mCoordinate[2].get_dq());

  Eigen::Matrix3d dJ = math::expMapJacDot(q, dq);

  Eigen::Vector6d dJ0;
  Eigen::Vector6d dJ1;
  Eigen::Vector6d dJ2;
  Eigen::Vector6d J3;
  Eigen::Vector6d J4;
  Eigen::Vector6d J5;

  dJ0 << dJ(0, 0), dJ(0, 1), dJ(0, 2), 0, 0, 0;
  dJ1 << dJ(1, 0), dJ(1, 1), dJ(1, 2), 0, 0, 0;
  dJ2 << dJ(2, 0), dJ(2, 1), dJ(2, 2), 0, 0, 0;
  J3 << 0, 0, 0, 1, 0, 0;
  J4 << 0, 0, 0, 0, 1, 0;
  J5 << 0, 0, 0, 0, 0, 1;

  mdS.col(0) = math::AdT(mT_ChildBodyToJoint, dJ0);
  mdS.col(1) = math::AdT(mT_ChildBodyToJoint, dJ1);
  mdS.col(2) = math::AdT(mT_ChildBodyToJoint, dJ2);
  mdS.col(3) =
      -math::ad(mS.leftCols<3>() * get_dq().head<3>(),
                math::AdT(mT_ChildBodyToJoint * math::expAngular(-q), J3));
  mdS.col(4) =
      -math::ad(mS.leftCols<3>() * get_dq().head<3>(),
                math::AdT(mT_ChildBodyToJoint * math::expAngular(-q), J4));
  mdS.col(5) =
      -math::ad(mS.leftCols<3>() * get_dq().head<3>(),
                math::AdT(mT_ChildBodyToJoint * math::expAngular(-q), J5));

  assert(!math::isNan(mdS));
}

void FreeJoint::updateJacobianTimeDeriv_Issue122() {
  // mdS == 0
  assert(mdS == Eigen::MatrixXd::Zero(6, 6));
}

void FreeJoint::clampRotation() {
  for (int i = 0; i < 3; i++) {
    if (mCoordinate[i].get_q() > M_PI)
      mCoordinate[i].set_q(mCoordinate[i].get_q() - 2*M_PI);
    if (mCoordinate[i].get_q() < -M_PI)
      mCoordinate[i].set_q(mCoordinate[i].get_q() + 2*M_PI);
  }
}

}  // namespace dynamics
}  // namespace dart
