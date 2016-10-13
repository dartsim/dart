/*
 * Copyright (c) 2014-2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2014-2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
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

#include "dart/constraint/WeldJointConstraint.hpp"

#include "dart/external/odelcpsolver/lcp.h"

#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/Skeleton.hpp"

namespace dart {
namespace constraint {

//==============================================================================
WeldJointConstraint::WeldJointConstraint(dynamics::BodyNode* _body)
  : JointConstraint(_body),
    mRelativeTransform(_body->getTransform()),
    mViolation(Eigen::Vector6d::Zero()),
    mJacobian1(Eigen::Matrix6d::Identity()),
    mAppliedImpulseIndex(0)
{
  mDim = 6;

  mOldX[0] = 0.0;
  mOldX[1] = 0.0;
  mOldX[2] = 0.0;
  mOldX[3] = 0.0;
  mOldX[4] = 0.0;
  mOldX[5] = 0.0;
}

//==============================================================================
WeldJointConstraint::WeldJointConstraint(dynamics::BodyNode* _body1,
                                         dynamics::BodyNode* _body2)
  : JointConstraint(_body1, _body2),
    mRelativeTransform(_body2->getTransform().inverse()
                       * _body1->getTransform()),
    mViolation(Eigen::Vector6d::Zero()),
    mJacobian1(Eigen::Matrix6d::Identity()),
    mAppliedImpulseIndex(0)
{
  // The bodies should be different bodies.
  assert(_body1 != _body2);

  mDim = 6;

  mOldX[0] = 0.0;
  mOldX[1] = 0.0;
  mOldX[2] = 0.0;
  mOldX[3] = 0.0;
  mOldX[4] = 0.0;
  mOldX[5] = 0.0;
}

//==============================================================================
WeldJointConstraint::~WeldJointConstraint()
{
}

//==============================================================================
void WeldJointConstraint::update()
{
  // mBodyNode1 should not be null pointer ever
  assert(mBodyNode1);

  // Update Jacobian for body2
  if (mBodyNode2)
  {
    Eigen::Isometry3d T12 = mBodyNode1->getTransform().inverse()
                            * mBodyNode2->getTransform();
    mJacobian2 = math::AdTJac(T12, mJacobian1);
  }

  // Update position constraint error
  if (mBodyNode2)
  {
    const Eigen::Isometry3d& violationT
        = mRelativeTransform.inverse()
          * mBodyNode2->getTransform().inverse()
          * mBodyNode1->getTransform();

    mViolation = math::logMap(violationT);
  }
  else
  {
    const Eigen::Isometry3d& violationT
        = mRelativeTransform.inverse() * mBodyNode1->getTransform();

    mViolation = math::logMap(violationT);
  }
}

//==============================================================================
void WeldJointConstraint::getInformation(ConstraintInfo* _lcp)
{
  assert(isActive());

  assert(_lcp->w[0] == 0.0);
  assert(_lcp->w[1] == 0.0);
  assert(_lcp->w[2] == 0.0);
  assert(_lcp->w[3] == 0.0);
  assert(_lcp->w[4] == 0.0);
  assert(_lcp->w[5] == 0.0);

  assert(_lcp->findex[0] == -1);
  assert(_lcp->findex[1] == -1);
  assert(_lcp->findex[2] == -1);
  assert(_lcp->findex[3] == -1);
  assert(_lcp->findex[4] == -1);
  assert(_lcp->findex[5] == -1);

  _lcp->lo[0] = -dInfinity;
  _lcp->lo[1] = -dInfinity;
  _lcp->lo[2] = -dInfinity;
  _lcp->lo[3] = -dInfinity;
  _lcp->lo[4] = -dInfinity;
  _lcp->lo[5] = -dInfinity;

  _lcp->hi[0] = dInfinity;
  _lcp->hi[1] = dInfinity;
  _lcp->hi[2] = dInfinity;
  _lcp->hi[3] = dInfinity;
  _lcp->hi[4] = dInfinity;
  _lcp->hi[5] = dInfinity;

  _lcp->x[0] = mOldX[0];
  _lcp->x[1] = mOldX[1];
  _lcp->x[2] = mOldX[2];
  _lcp->x[3] = mOldX[3];
  _lcp->x[4] = mOldX[4];
  _lcp->x[5] = mOldX[5];

  Eigen::Vector6d negativeVel = -mBodyNode1->getSpatialVelocity();
  if (mBodyNode2)
    negativeVel += mJacobian2 * mBodyNode2->getSpatialVelocity();

  mViolation *= mErrorReductionParameter * _lcp->invTimeStep;

  _lcp->b[0] = negativeVel[0] - mViolation[0];
  _lcp->b[1] = negativeVel[1] - mViolation[1];
  _lcp->b[2] = negativeVel[2] - mViolation[2];
  _lcp->b[3] = negativeVel[3] - mViolation[3];
  _lcp->b[4] = negativeVel[4] - mViolation[4];
  _lcp->b[5] = negativeVel[5] - mViolation[5];
}

//==============================================================================
void WeldJointConstraint::applyUnitImpulse(std::size_t _index)
{
  assert(_index < mDim && "Invalid Index.");
  assert(isActive());

  if (mBodyNode2)
  {
    assert(mBodyNode1->isReactive() || mBodyNode2->isReactive());

    // Weld joint between two bodies in one skeleton
    if (mBodyNode1->getSkeleton() == mBodyNode2->getSkeleton())
    {
      mBodyNode1->getSkeleton()->clearConstraintImpulses();

      if (mBodyNode1->isReactive())
      {
        if (mBodyNode2->isReactive())
        {
          mBodyNode1->getSkeleton()->updateBiasImpulse(
                mBodyNode1, mJacobian1.row(_index),
                mBodyNode2, -mJacobian2.row(_index));
        }
        else
        {
          mBodyNode1->getSkeleton()->updateBiasImpulse(
                mBodyNode1, mJacobian1.row(_index));
        }
      }
      else
      {
        if (mBodyNode2->isReactive())
        {
          mBodyNode2->getSkeleton()->updateBiasImpulse(
                mBodyNode2, -mJacobian2.row(_index));
        }
        else
        {
          assert(0);
        }
      }

      mBodyNode1->getSkeleton()->updateVelocityChange();
    }
    // Weld joint between two bodies in different skeletons
    else
    {
      if (mBodyNode1->isReactive())
      {
        mBodyNode1->getSkeleton()->clearConstraintImpulses();
        mBodyNode1->getSkeleton()->updateBiasImpulse(
              mBodyNode1, mJacobian1.row(_index));
        mBodyNode1->getSkeleton()->updateVelocityChange();
      }

      if (mBodyNode2->isReactive())
      {
        mBodyNode2->getSkeleton()->clearConstraintImpulses();
        mBodyNode2->getSkeleton()->updateBiasImpulse(
              mBodyNode2, -mJacobian2.row(_index));
        mBodyNode2->getSkeleton()->updateVelocityChange();
      }
    }
  }
  else
  {
    assert(mBodyNode1->isReactive());

    mBodyNode1->getSkeleton()->clearConstraintImpulses();
    mBodyNode1->getSkeleton()->updateBiasImpulse(
          mBodyNode1, mJacobian1.row(_index));
    mBodyNode1->getSkeleton()->updateVelocityChange();
  }

  mAppliedImpulseIndex = _index;
}

//==============================================================================
void WeldJointConstraint::getVelocityChange(double* _vel, bool _withCfm)
{
  assert(_vel != nullptr && "Null pointer is not allowed.");
  assert(isActive());

  Eigen::Vector6d velChange = Eigen::Vector6d::Zero();
  if (mBodyNode1->getSkeleton()->isImpulseApplied()
      && mBodyNode1->isReactive())
  {
    velChange += mBodyNode1->getBodyVelocityChange();
  }

  if (mBodyNode2
      && mBodyNode2->getSkeleton()->isImpulseApplied()
      && mBodyNode2->isReactive())
  {
    velChange -= mJacobian2 * mBodyNode2->getBodyVelocityChange();
  }

  for (std::size_t i = 0; i < mDim; ++i)
    _vel[i] = velChange[i];

  // Add small values to diagnal to keep it away from singular, similar to cfm
  // varaible in ODE
  if (_withCfm)
  {
    _vel[mAppliedImpulseIndex] += _vel[mAppliedImpulseIndex]
                                  * mConstraintForceMixing;
  }
}

//==============================================================================
void WeldJointConstraint::excite()
{
  if (mBodyNode1->isReactive())
    mBodyNode1->getSkeleton()->setImpulseApplied(true);

  if (mBodyNode2 == nullptr)
    return;

  if (mBodyNode2->isReactive())
    mBodyNode2->getSkeleton()->setImpulseApplied(true);
}

//==============================================================================
void WeldJointConstraint::unexcite()
{
  if (mBodyNode1->isReactive())
    mBodyNode1->getSkeleton()->setImpulseApplied(false);

  if (mBodyNode2 == nullptr)
    return;

  if (mBodyNode2->isReactive())
    mBodyNode2->getSkeleton()->setImpulseApplied(false);
}

//==============================================================================
void WeldJointConstraint::applyImpulse(double* _lambda)
{
  mOldX[0] = _lambda[0];
  mOldX[1] = _lambda[1];
  mOldX[2] = _lambda[2];
  mOldX[3] = _lambda[3];
  mOldX[4] = _lambda[4];
  mOldX[5] = _lambda[5];

  Eigen::Vector6d imp;
  imp << _lambda[0],
         _lambda[1],
         _lambda[2],
         _lambda[3],
         _lambda[4],
         _lambda[5];

  mBodyNode1->addConstraintImpulse(imp);

  if (mBodyNode2)
    mBodyNode2->addConstraintImpulse(mJacobian2.transpose() * -imp);
}

//==============================================================================
dynamics::SkeletonPtr WeldJointConstraint::getRootSkeleton() const
{
  if (mBodyNode1->isReactive())
    return mBodyNode1->getSkeleton()->mUnionRootSkeleton.lock();

  if (mBodyNode2)
  {
    if (mBodyNode2->isReactive())
    {
      return mBodyNode2->getSkeleton()->mUnionRootSkeleton.lock();
    }
    else
    {
      assert(0);
      return nullptr;
    }
  }
  else
  {
    assert(0);
    return nullptr;
  }
}

//==============================================================================
void WeldJointConstraint::uniteSkeletons()
{
  if (mBodyNode2 == nullptr)
    return;

  if (!mBodyNode1->isReactive() || !mBodyNode2->isReactive())
    return;

  if (mBodyNode1->getSkeleton() == mBodyNode2->getSkeleton())
    return;

  const dynamics::SkeletonPtr& unionId1
      = ConstraintBase::compressPath(mBodyNode1->getSkeleton());
  const dynamics::SkeletonPtr& unionId2
      = ConstraintBase::compressPath(mBodyNode2->getSkeleton());

  if (unionId1 == unionId2)
    return;

  if (unionId1->mUnionSize < unionId2->mUnionSize)
  {
    // Merge root1 --> root2
    unionId1->mUnionRootSkeleton = unionId2;
    unionId2->mUnionSize += unionId1->mUnionSize;
  }
  else
  {
    // Merge root2 --> root1
    unionId2->mUnionRootSkeleton = unionId1;
    unionId1->mUnionSize += unionId2->mUnionSize;
  }
}

//==============================================================================
bool WeldJointConstraint::isActive() const
{
  if (mBodyNode1->isReactive())
    return true;

  if (mBodyNode2)
  {
    if (mBodyNode2->isReactive())
      return true;
    else
      return false;
  }
  else
  {
    return false;
  }
}

} // namespace constraint
} // namespace dart
