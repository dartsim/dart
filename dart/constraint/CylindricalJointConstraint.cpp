/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
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

#include "dart/constraint/CylindricalJointConstraint.hpp"

#include "dart/common/Macros.hpp"
#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/lcpsolver/dantzig/lcp.h"

#include <cmath>

namespace dart {
namespace constraint {

namespace {

Eigen::Vector3d normalizeOrFallback(const Eigen::Vector3d& _axis)
{
  const double norm = _axis.norm();
  if (norm < 1e-9)
    return Eigen::Vector3d::UnitZ();

  return _axis / norm;
}

Eigen::Matrix<double, 3, 6> makeAnchorJacobian(const Eigen::Vector3d& _offset)
{
  Eigen::Matrix<double, 3, 6> jacobian;
  jacobian.leftCols<3>() = dart::math::makeSkewSymmetric(-_offset);
  jacobian.rightCols<3>() = Eigen::Matrix3d::Identity();
  return jacobian;
}

} // namespace

//==============================================================================
CylindricalJointConstraint::CylindricalJointConstraint(
    dynamics::BodyNode* _body,
    const Eigen::Vector3d& _jointPos,
    const Eigen::Vector3d& _axis)
  : DynamicJointConstraint(_body),
    mOffset1(_body->getTransform().inverse() * _jointPos),
    mOffset2(_jointPos),
    mAxis1(
        _body->getTransform().linear().transpose()
        * normalizeOrFallback(_axis)),
    mAxis2(normalizeOrFallback(_axis)),
    mWorldAxis1(Eigen::Vector3d::UnitZ()),
    mWorldAxis2(normalizeOrFallback(_axis)),
    mViolation(Eigen::Matrix<double, 4, 1>::Zero()),
    mAppliedImpulseIndex(0)
{
  mDim = 4;

  mJacobian1.setZero();
  mJacobian2.setZero();

  mOldX[0] = 0.0;
  mOldX[1] = 0.0;
  mOldX[2] = 0.0;
  mOldX[3] = 0.0;
}

//==============================================================================
CylindricalJointConstraint::CylindricalJointConstraint(
    dynamics::BodyNode* _body1,
    dynamics::BodyNode* _body2,
    const Eigen::Vector3d& _jointPos,
    const Eigen::Vector3d& _axis1,
    const Eigen::Vector3d& _axis2)
  : DynamicJointConstraint(_body1, _body2),
    mOffset1(_body1->getTransform().inverse() * _jointPos),
    mOffset2(_body2->getTransform().inverse() * _jointPos),
    mAxis1(
        _body1->getTransform().linear().transpose()
        * normalizeOrFallback(_axis1)),
    mAxis2(
        _body2->getTransform().linear().transpose()
        * normalizeOrFallback(_axis2)),
    mWorldAxis1(Eigen::Vector3d::UnitZ()),
    mWorldAxis2(Eigen::Vector3d::UnitZ()),
    mViolation(Eigen::Matrix<double, 4, 1>::Zero()),
    mAppliedImpulseIndex(0)
{
  DART_ASSERT(_body1 != _body2);

  mDim = 4;

  mJacobian1.setZero();
  mJacobian2.setZero();

  mOldX[0] = 0.0;
  mOldX[1] = 0.0;
  mOldX[2] = 0.0;
  mOldX[3] = 0.0;
}

//==============================================================================
CylindricalJointConstraint::~CylindricalJointConstraint()
{
  // Do nothing
}

//==============================================================================
const std::string& CylindricalJointConstraint::getType() const
{
  return getStaticType();
}

//==============================================================================
const std::string& CylindricalJointConstraint::getStaticType()
{
  static const std::string name = "CylindricalJointConstraint";
  return name;
}

//==============================================================================
void CylindricalJointConstraint::updatePerpendicularBasis()
{
  const Eigen::Vector3d axis = normalizeOrFallback(mWorldAxis1);
  const Eigen::Vector3d ref = (std::abs(axis.x()) < 0.9)
                                  ? Eigen::Vector3d::UnitX()
                                  : Eigen::Vector3d::UnitY();
  Eigen::Vector3d perp1 = axis.cross(ref);
  if (perp1.norm() < 1e-9)
    perp1 = axis.cross(Eigen::Vector3d::UnitZ());
  perp1.normalize();
  const Eigen::Vector3d perp2 = axis.cross(perp1);

  mPerpBasis.row(0) = perp1.transpose();
  mPerpBasis.row(1) = perp2.transpose();
}

//==============================================================================
void CylindricalJointConstraint::update()
{
  // mBodyNode1 should not be null pointer ever
  DART_ASSERT(mBodyNode1);

  const Eigen::Isometry3d& T1 = mBodyNode1->getTransform();
  const Eigen::Matrix3d& R1 = T1.linear();
  mWorldAxis1 = normalizeOrFallback(R1 * mAxis1);

  Eigen::Vector3d anchor2World = mOffset2;
  if (mBodyNode2) {
    const Eigen::Isometry3d& T2 = mBodyNode2->getTransform();
    mWorldAxis2 = normalizeOrFallback(T2.linear() * mAxis2);
    anchor2World = T2 * mOffset2;
  } else {
    mWorldAxis2 = normalizeOrFallback(mAxis2);
  }

  updatePerpendicularBasis();

  mJacobian1.setZero();
  mJacobian1.topRows<2>() = mPerpBasis * R1 * makeAnchorJacobian(mOffset1);
  mJacobian1.block<2, 3>(2, 0) = mPerpBasis * R1;

  if (mBodyNode2) {
    const Eigen::Matrix3d& R2 = mBodyNode2->getTransform().linear();
    mJacobian2.setZero();
    mJacobian2.topRows<2>() = mPerpBasis * R2 * makeAnchorJacobian(mOffset2);
    mJacobian2.block<2, 3>(2, 0) = mPerpBasis * R2;
  }

  const Eigen::Vector3d anchor1World = T1 * mOffset1;
  const Eigen::Vector3d linearError = anchor1World - anchor2World;
  mViolation.head<2>() = mPerpBasis * linearError;

  const Eigen::Vector3d axisError = mWorldAxis1.cross(mWorldAxis2);
  mViolation.tail<2>() = mPerpBasis * axisError;
}

//==============================================================================
void CylindricalJointConstraint::getInformation(ConstraintInfo* _lcp)
{
  DART_ASSERT(isActive());

  DART_ASSERT(_lcp->w[0] == 0.0);
  DART_ASSERT(_lcp->w[1] == 0.0);
  DART_ASSERT(_lcp->w[2] == 0.0);
  DART_ASSERT(_lcp->w[3] == 0.0);

  DART_ASSERT(_lcp->findex[0] == -1);
  DART_ASSERT(_lcp->findex[1] == -1);
  DART_ASSERT(_lcp->findex[2] == -1);
  DART_ASSERT(_lcp->findex[3] == -1);

  _lcp->lo[0] = -dInfinity;
  _lcp->lo[1] = -dInfinity;
  _lcp->lo[2] = -dInfinity;
  _lcp->lo[3] = -dInfinity;

  _lcp->hi[0] = dInfinity;
  _lcp->hi[1] = dInfinity;
  _lcp->hi[2] = dInfinity;
  _lcp->hi[3] = dInfinity;

  _lcp->x[0] = mOldX[0];
  _lcp->x[1] = mOldX[1];
  _lcp->x[2] = mOldX[2];
  _lcp->x[3] = mOldX[3];

  Eigen::Matrix<double, 4, 1> negativeVel
      = -mJacobian1 * mBodyNode1->getSpatialVelocity();
  if (mBodyNode2)
    negativeVel += mJacobian2 * mBodyNode2->getSpatialVelocity();

  mViolation *= mErrorReductionParameter * _lcp->invTimeStep;

  for (std::size_t i = 0; i < mDim; ++i)
    _lcp->b[i] = negativeVel[i] - mViolation[i];
}

//==============================================================================
void CylindricalJointConstraint::applyUnitImpulse(std::size_t _index)
{
  DART_ASSERT(_index < mDim && "Invalid Index.");
  DART_ASSERT(isActive());

  if (mBodyNode2) {
    DART_ASSERT(mBodyNode1->isReactive() || mBodyNode2->isReactive());

    if (mBodyNode1->getSkeleton() == mBodyNode2->getSkeleton()) {
      mBodyNode1->getSkeleton()->clearConstraintImpulses();

      if (mBodyNode1->isReactive()) {
        if (mBodyNode2->isReactive()) {
          mBodyNode1->getSkeleton()->updateBiasImpulse(
              mBodyNode1,
              mJacobian1.row(_index),
              mBodyNode2,
              -mJacobian2.row(_index));
        } else {
          mBodyNode1->getSkeleton()->updateBiasImpulse(
              mBodyNode1, mJacobian1.row(_index));
        }
      } else {
        if (mBodyNode2->isReactive()) {
          mBodyNode2->getSkeleton()->updateBiasImpulse(
              mBodyNode2, -mJacobian2.row(_index));
        } else {
          DART_ASSERT(0);
        }
      }

      mBodyNode1->getSkeleton()->updateVelocityChange();
    } else {
      if (mBodyNode1->isReactive()) {
        mBodyNode1->getSkeleton()->clearConstraintImpulses();
        mBodyNode1->getSkeleton()->updateBiasImpulse(
            mBodyNode1, mJacobian1.row(_index));
        mBodyNode1->getSkeleton()->updateVelocityChange();
      }

      if (mBodyNode2->isReactive()) {
        mBodyNode2->getSkeleton()->clearConstraintImpulses();
        mBodyNode2->getSkeleton()->updateBiasImpulse(
            mBodyNode2, -mJacobian2.row(_index));
        mBodyNode2->getSkeleton()->updateVelocityChange();
      }
    }
  } else {
    DART_ASSERT(mBodyNode1->isReactive());

    mBodyNode1->getSkeleton()->clearConstraintImpulses();
    mBodyNode1->getSkeleton()->updateBiasImpulse(
        mBodyNode1, mJacobian1.row(_index));
    mBodyNode1->getSkeleton()->updateVelocityChange();
  }

  mAppliedImpulseIndex = _index;
}

//==============================================================================
void CylindricalJointConstraint::getVelocityChange(double* _vel, bool _withCfm)
{
  DART_ASSERT(_vel != nullptr && "Null pointer is not allowed.");
  DART_ASSERT(isActive());

  for (std::size_t i = 0; i < mDim; ++i)
    _vel[i] = 0.0;

  if (mBodyNode1->getSkeleton()->isImpulseApplied()
      && mBodyNode1->isReactive()) {
    const Eigen::Matrix<double, 4, 1> v1
        = mJacobian1 * mBodyNode1->getBodyVelocityChange();
    for (std::size_t i = 0; i < mDim; ++i)
      _vel[i] += v1[i];
  }

  if (mBodyNode2 && mBodyNode2->getSkeleton()->isImpulseApplied()
      && mBodyNode2->isReactive()) {
    const Eigen::Matrix<double, 4, 1> v2
        = mJacobian2 * mBodyNode2->getBodyVelocityChange();
    for (std::size_t i = 0; i < mDim; ++i)
      _vel[i] -= v2[i];
  }

  if (_withCfm) {
    _vel[mAppliedImpulseIndex]
        += _vel[mAppliedImpulseIndex] * mConstraintForceMixing
           + mConstraintForceMixing;
  }
}

//==============================================================================
void CylindricalJointConstraint::excite()
{
  if (mBodyNode1->isReactive())
    mBodyNode1->getSkeleton()->setImpulseApplied(true);

  if (mBodyNode2 == nullptr)
    return;

  if (mBodyNode2->isReactive())
    mBodyNode2->getSkeleton()->setImpulseApplied(true);
}

//==============================================================================
void CylindricalJointConstraint::unexcite()
{
  if (mBodyNode1->isReactive())
    mBodyNode1->getSkeleton()->setImpulseApplied(false);

  if (mBodyNode2 == nullptr)
    return;

  if (mBodyNode2->isReactive())
    mBodyNode2->getSkeleton()->setImpulseApplied(false);
}

//==============================================================================
void CylindricalJointConstraint::applyImpulse(double* _lambda)
{
  mOldX[0] = _lambda[0];
  mOldX[1] = _lambda[1];
  mOldX[2] = _lambda[2];
  mOldX[3] = _lambda[3];

  Eigen::Matrix<double, 4, 1> imp;
  imp << _lambda[0], _lambda[1], _lambda[2], _lambda[3];

  mBodyNode1->addConstraintImpulse(mJacobian1.transpose() * imp);

  if (mBodyNode2)
    mBodyNode2->addConstraintImpulse(-mJacobian2.transpose() * imp);
}

//==============================================================================
dynamics::SkeletonPtr CylindricalJointConstraint::getRootSkeleton() const
{
  if (mBodyNode1->isReactive())
    return ConstraintBase::getRootSkeleton(
        mBodyNode1->getSkeleton()->getSkeleton());

  if (mBodyNode2) {
    if (mBodyNode2->isReactive()) {
      return ConstraintBase::getRootSkeleton(
          mBodyNode2->getSkeleton()->getSkeleton());
    } else {
      DART_ASSERT(0);
      return nullptr;
    }
  } else {
    DART_ASSERT(0);
    return nullptr;
  }
}

//==============================================================================
void CylindricalJointConstraint::uniteSkeletons()
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

  if (unionId1->mUnionSize < unionId2->mUnionSize) {
    unionId1->mUnionRootSkeleton = unionId2;
    unionId2->mUnionSize += unionId1->mUnionSize;
  } else {
    unionId2->mUnionRootSkeleton = unionId1;
    unionId1->mUnionSize += unionId2->mUnionSize;
  }
}

//==============================================================================
bool CylindricalJointConstraint::isActive() const
{
  if (mBodyNode1->isReactive())
    return true;

  if (mBodyNode2) {
    if (mBodyNode2->isReactive())
      return true;
    else
      return false;
  } else {
    return false;
  }
}

} // namespace constraint
} // namespace dart
