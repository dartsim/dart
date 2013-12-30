/*
 * Copyright (c) 2011-2013, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Karen Liu
 * Date:
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

#include "dart/constraint/ClosedLoopConstraint.h"

#include <vector>

#include "dart/math/Helpers.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Skeleton.h"

namespace dart {
namespace constraint {

ClosedLoopConstraint::ClosedLoopConstraint(dynamics::BodyNode *_body1,
                                           dynamics::BodyNode *_body2,
                                           const Eigen::Vector3d& _offset1,
                                           const Eigen::Vector3d& _offset2,
                                           int _skelIndex1, int _skelIndex2)
{
  mBody1 = _body1;
  mBody2 = _body2;
  mOffset1 = _offset1;
  mOffset2 = _offset2;
  mJ1 = Eigen::MatrixXd::Zero(3, mBody1->getSkeleton()->getNumGenCoords());
  mJ2 = Eigen::MatrixXd::Zero(3, mBody2->getSkeleton()->getNumGenCoords());
  mNumRows = 3;
  mSkelIndex1 = _skelIndex1;
  mSkelIndex2 = _skelIndex2;
}

ClosedLoopConstraint::~ClosedLoopConstraint()
{
}

void ClosedLoopConstraint::updateDynamics(std::vector<Eigen::MatrixXd>* _J,
                                          Eigen::VectorXd* _C,
                                          Eigen::VectorXd* _CDot,
                                          int _rowIndex)
{
  getJacobian();
  _J->at(mSkelIndex2).block(_rowIndex, 0, 3,
                            mBody2->getSkeleton()->getNumGenCoords()).setZero();
  _J->at(mSkelIndex1).block(_rowIndex, 0, 3,
                            mBody1->getSkeleton()->getNumGenCoords()) = mJ1;
  _J->at(mSkelIndex2).block(_rowIndex, 0, 3,
                            mBody2->getSkeleton()->getNumGenCoords()) += mJ2;

  Eigen::Vector3d worldP1 = mBody1->getWorldTransform() * mOffset1;
  Eigen::Vector3d worldP2 = mBody2->getWorldTransform() * mOffset2;
  Eigen::VectorXd qDot1 = mBody1->getSkeleton()->get_dq();
  Eigen::VectorXd qDot2 = mBody2->getSkeleton()->get_dq();
  _C->segment(_rowIndex, 3) = worldP1 - worldP2;
  _CDot->segment(_rowIndex, 3) = mJ1 * qDot1 + mJ2 * qDot2;
}

void ClosedLoopConstraint::getJacobian()
{
  Eigen::MatrixXd JBody1 =
      mBody1->getWorldJacobian(
        mOffset1 - mBody1->getWorldTransform().translation()).bottomRows<3>();
  for (int i = 0; i < mBody1->getNumDependentGenCoords(); i++)
  {
    int dofIndex = mBody1->getDependentGenCoord(i);
    mJ1.col(dofIndex) = JBody1.col(dofIndex);
  }
  Eigen::MatrixXd JBody2 =
      mBody2->getWorldJacobian(
        mOffset2 - mBody2->getWorldTransform().translation()).bottomRows<3>();
  for (int i = 0; i < mBody2->getNumDependentGenCoords(); i++)
  {
    int dofIndex = mBody2->getDependentGenCoord(i);
    mJ2.col(dofIndex) = JBody2.col(dofIndex);
  }
}

}  // namespace constraint
}  // namespace dart
