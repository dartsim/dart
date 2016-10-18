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
#include <iostream>
#include "dart/constraint/BallJointConstraint.hpp"

#include "dart/external/odelcpsolver/lcp.h"

#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/Skeleton.hpp"

namespace dart {
namespace constraint {

//==============================================================================
BallJointConstraint::BallJointConstraint(dynamics::BodyNode* _body,
                                         const Eigen::Vector3d& _jointPos)
  : JointConstraint(_body),
    mOffset1(_body->getTransform().inverse() * _jointPos),
    mOffset2(_jointPos),
    mViolation(Eigen::Vector3d::Zero()),
    mAppliedImpulseIndex(0)
{
  mDim = 3;

  mOldX[0] = 0.0;
  mOldX[1] = 0.0;
  mOldX[2] = 0.0;

  Eigen::Matrix3d ssm1 = dart::math::makeSkewSymmetric(-mOffset1);
  mJacobian1.leftCols<3>()  = ssm1;
  mJacobian1.rightCols<3>() = Eigen::Matrix3d::Identity();
}

//==============================================================================
BallJointConstraint::BallJointConstraint(dynamics::BodyNode* _body1,
                                         dynamics::BodyNode* _body2,
                                         const Eigen::Vector3d& _jointPos)
  : JointConstraint(_body1, _body2),
    mOffset1(_body1->getTransform().inverse() * _jointPos),
    mOffset2(_body2->getTransform().inverse() * _jointPos),
    mViolation(Eigen::Vector3d::Zero()),
    mAppliedImpulseIndex(0)
{
  mDim = 3;

  mOldX[0] = 0.0;
  mOldX[1] = 0.0;
  mOldX[2] = 0.0;

  Eigen::Matrix3d ssm1 = dart::math::makeSkewSymmetric(-mOffset1);
  Eigen::Matrix3d ssm2 = dart::math::makeSkewSymmetric(-mOffset2);
  mJacobian1.leftCols<3>()  = ssm1;
  mJacobian1.rightCols<3>() = Eigen::Matrix3d::Identity();
  mJacobian2.leftCols<3>()  = ssm2;
  mJacobian2.rightCols<3>() = Eigen::Matrix3d::Identity();
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

  // Update Jacobian for body2
  if (mBodyNode2)
  {
    Eigen::Isometry3d T12 = mBodyNode1->getTransform().inverse()
                            * mBodyNode2->getTransform();
    Eigen::Vector3d p2 = T12.inverse() * mOffset1;

    Eigen::Matrix<double, 3, 6> J2;
    J2.leftCols<3>()  = math::makeSkewSymmetric(-p2);
    J2.rightCols<3>() = Eigen::Matrix3d::Identity();

    mJacobian2 = T12.linear() * J2;
  }

  // Update position constraint error
  if (mBodyNode2)
  {
    mViolation = mOffset1 - mBodyNode1->getTransform().inverse()
                            * mBodyNode2->getTransform() * mOffset2;
  }
  else
  {
    mViolation = mOffset1 - mBodyNode1->getTransform().inverse() * mOffset2;
  }

  //  std::cout << "mViolation = " << mViolation << std::endl;
}

//==============================================================================
void BallJointConstraint::getInformation(ConstraintInfo* _lcp)
{
  assert(_lcp->w[0] == 0.0);
  assert(_lcp->w[1] == 0.0);
  assert(_lcp->w[2] == 0.0);

  assert(_lcp->findex[0] == -1);
  assert(_lcp->findex[1] == -1);
  assert(_lcp->findex[2] == -1);

  _lcp->lo[0] = -dInfinity;
  _lcp->lo[1] = -dInfinity;
  _lcp->lo[2] = -dInfinity;

  _lcp->hi[0] = dInfinity;
  _lcp->hi[1] = dInfinity;
  _lcp->hi[2] = dInfinity;

  _lcp->x[0] = mOldX[0];
  _lcp->x[1] = mOldX[1];
  _lcp->x[2] = mOldX[2];

  Eigen::Vector3d negativeVel = -mJacobian1 * mBodyNode1->getSpatialVelocity();
  if (mBodyNode2)
    negativeVel += mJacobian2 * mBodyNode2->getSpatialVelocity();

  mViolation *= mErrorReductionParameter * _lcp->invTimeStep;

  _lcp->b[0] = negativeVel[0] - mViolation[0];
  _lcp->b[1] = negativeVel[1] - mViolation[1];
  _lcp->b[2] = negativeVel[2] - mViolation[2];

  //  std::cout << "b: " << _lcp->b[0] << " " << _lcp->b[1] << " " << _lcp->b[2] << " " << std::endl;
}

//==============================================================================
void BallJointConstraint::applyUnitImpulse(std::size_t _index)
{
  assert(_index < mDim && "Invalid Index.");
  assert(isActive());

  if (mBodyNode2)
  {
    assert(mBodyNode1->isReactive() || mBodyNode2->isReactive());

    // Self collision case
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
    // Colliding two distinct skeletons
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
void BallJointConstraint::getVelocityChange(double* _vel, bool _withCfm)
{
  assert(_vel != nullptr && "Null pointer is not allowed.");

  for (std::size_t i = 0; i < mDim; ++i)
   _vel[i] = 0.0;

  if (mBodyNode1->getSkeleton()->isImpulseApplied()
      && mBodyNode1->isReactive())
  {
    Eigen::Vector3d v1 = mJacobian1 * mBodyNode1->getBodyVelocityChange();
    // std::cout << "velChange " << mBodyNode1->getBodyVelocityChange() << std::endl;
    // std::cout << "v1: " << v1 << std::endl;
    for (std::size_t i = 0; i < mDim; ++i)
      _vel[i] += v1[i];
  }

  if (mBodyNode2
      && mBodyNode2->getSkeleton()->isImpulseApplied()
      && mBodyNode2->isReactive())
  {
    Eigen::Vector3d v2 = mJacobian2 * mBodyNode2->getBodyVelocityChange();
    // std::cout << "v2: " << v2 << std::endl;
    for (std::size_t i = 0; i < mDim; ++i)
      _vel[i] -= v2[i];
  }

  // Add small values to diagnal to keep it away from singular, similar to cfm
  // varaible in ODE
  if (_withCfm)
  {
    _vel[mAppliedImpulseIndex] += _vel[mAppliedImpulseIndex]
                                  * mConstraintForceMixing;
  }
}

//==============================================================================
void BallJointConstraint::excite()
{
  if (mBodyNode1->isReactive())
    mBodyNode1->getSkeleton()->setImpulseApplied(true);

  if (mBodyNode2 == nullptr)
    return;

  if (mBodyNode2->isReactive())
    mBodyNode2->getSkeleton()->setImpulseApplied(true);
}

//==============================================================================
void BallJointConstraint::unexcite()
{
  if (mBodyNode1->isReactive())
    mBodyNode1->getSkeleton()->setImpulseApplied(false);

  if (mBodyNode2 == nullptr)
    return;

  if (mBodyNode2->isReactive())
    mBodyNode2->getSkeleton()->setImpulseApplied(false);
}

//==============================================================================
void BallJointConstraint::applyImpulse(double* _lambda)
{
  mOldX[0] = _lambda[0];
  mOldX[1] = _lambda[1];
  mOldX[2] = _lambda[2];

  Eigen::Vector3d imp(_lambda[0], _lambda[1], _lambda[2]);

  // std::cout << "lambda: " << _lambda[0] << " " << _lambda[1] << " " << _lambda[2] << std::endl;

  mBodyNode1->addConstraintImpulse(mJacobian1.transpose() * imp);

  if (mBodyNode2)
    mBodyNode2->addConstraintImpulse(-mJacobian2.transpose() * imp);
}

//==============================================================================
dynamics::SkeletonPtr BallJointConstraint::getRootSkeleton() const
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
void BallJointConstraint::uniteSkeletons()
{
  if (mBodyNode2 == nullptr)
    return;

  if (!mBodyNode1->isReactive() || !mBodyNode2->isReactive())
    return;

  if (mBodyNode1->getSkeleton() == mBodyNode2->getSkeleton())
    return;

  dynamics::SkeletonPtr unionId1
      = ConstraintBase::compressPath(mBodyNode1->getSkeleton());
  dynamics::SkeletonPtr unionId2
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
bool BallJointConstraint::isActive() const
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

