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

#include "dart/constraint/PointConstraint.h"

#include <vector>

#include "dart/math/Helpers.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/dynamics/BodyNode.h"

namespace dart {
namespace constraint {

PointConstraint::PointConstraint(dynamics::BodyNode* _body,
                                 const Eigen::Vector3d& _offset,
                                 const Eigen::Vector3d& _target,
                                 int _skelIndex)
{
  mBody = _body;
  mOffset = _offset;
  mTarget = _target;
  mSkelIndex = _skelIndex;
  mJ = Eigen::MatrixXd::Zero(3, mBody->getSkeleton()->getNumGenCoords());
  mNumRows = 3;
}

PointConstraint::~PointConstraint()
{
}

void PointConstraint::updateDynamics(std::vector<Eigen::MatrixXd>* _J,
                                     Eigen::VectorXd* _C,
                                     Eigen::VectorXd* _CDot,
                                     int _rowIndex)
{
  getJacobian();
  dynamics::Skeleton *skel = mBody->getSkeleton();
  _J->at(mSkelIndex).block(_rowIndex, 0, 3, skel->getNumGenCoords()) = mJ;
  Eigen::Vector3d worldP = mBody->getWorldTransform() * mOffset;
  Eigen::VectorXd qDot = skel->get_dq();
  _C->segment(_rowIndex, 3) = worldP - mTarget;
  _CDot->segment(_rowIndex, 3) = mJ * qDot;
}

void PointConstraint::getJacobian()
{
  Eigen::MatrixXd JBody =
      mBody->getWorldJacobian(
        mOffset - mBody->getWorldTransform().translation()).bottomRows<3>();
  for (int i = 0; i < mBody->getNumDependentGenCoords(); i++)
  {
    int dofIndex = mBody->getDependentGenCoord(i);
    mJ.col(dofIndex) = JBody.col(i);
  }
}

}  // namespace constraint
}  // namespace dart
