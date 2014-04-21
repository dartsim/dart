/*
 * Copyright (c) 2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
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

#include "dart/constraint/BallJointConstraint.h"

namespace dart {
namespace constraint {

//==============================================================================
BallJointConstraint::BallJointConstraint(dynamics::BodyNode* _body,
                                         const Eigen::Vector3d& _offset)
  : JointConstraint(_body),
    mOffset1(_offset),
    mOffset2(Eigen::Vector3d::Zero())
{

}

//==============================================================================
BallJointConstraint::BallJointConstraint(dynamics::BodyNode* _body1,
                                         dynamics::BodyNode* _body2,
                                         const Eigen::Vector3d& _offset1,
                                         const Eigen::Vector3d& _offset2)
  : JointConstraint(_body1, _body2),
    mOffset1(_offset1),
    mOffset2(_offset2)
{

}

//==============================================================================
BallJointConstraint::~BallJointConstraint()
{

}

//==============================================================================
void BallJointConstraint::update()
{
  // mBodyNode1 should not be null pointer ever
  assert(mBodyNode1);

//  if (mBodyNode2)
//  {
//    const Eigen::Isometry3d& violationT
//        = mBodyNode2->getWorldTransform().inverse()
//          * mBodyNode1->getWorldTransform();

//    mViolation = math::logMap(violationT);
//  }
//  else
//  {
//    const Eigen::Isometry3d& violationT
//        = mBodyNode1->getWorldTransform().inverse() * mRelativeTransform;

//    mViolation = math::logMap(violationT);
//  }
}

//==============================================================================
void BallJointConstraint::getLCPVectors(ConstraintInfo* _info)
{

}

//==============================================================================
void BallJointConstraint::applyUnitImpulse(size_t _index)
{

}

//==============================================================================
void BallJointConstraint::getVelocityChange(double* _vel, bool _withCfm)
{

}

//==============================================================================
void BallJointConstraint::excite()
{

}

//==============================================================================
void BallJointConstraint::unexcite()
{

}

//==============================================================================
void BallJointConstraint::applyConstraintImpulse(double* _lambda)
{

}

} // namespace constraint
} // namespace dart
