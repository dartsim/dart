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

#include "dart/constraint/coupler_constraint.hpp"

#include "dart/common/logging.hpp"
#include "dart/common/macros.hpp"
#include "dart/dynamics/body_node.hpp"
#include "dart/dynamics/joint.hpp"
#include "dart/dynamics/skeleton.hpp"

#include <algorithm>

#include <cmath>

namespace {

constexpr inline double kConstraintForceMixing = 1e-6;
constexpr inline double kDefaultForceLimit = 800.0;
constexpr inline double kDefaultVelocityLimit = 50.0;
constexpr inline double kDefaultErp = 0.4;

} // namespace

namespace dart {
namespace constraint {

double CouplerConstraint::mConstraintForceMixing = kConstraintForceMixing;

//==============================================================================
CouplerConstraint::CouplerConstraint(
    dynamics::Joint* joint,
    std::span<const dynamics::MimicDofProperties> mimicDofProperties)
  : ConstraintBase(),
    mJoint(joint),
    mMimicProps(mimicDofProperties.begin(), mimicDofProperties.end()),
    mBodyNode(joint->getChildBodyNode()),
    mAppliedImpulseIndex(0)
{
  DART_ASSERT(joint);
  DART_ASSERT(joint->getNumDofs() <= mMimicProps.size());
  DART_ASSERT(mBodyNode);

  std::fill(mLifeTime, mLifeTime + 6, 0);
  std::fill(mActive, mActive + 6, false);
}

//==============================================================================
CouplerConstraint::~CouplerConstraint()
{
  // Do nothing
}

//==============================================================================
std::string_view CouplerConstraint::getType() const
{
  return getStaticType();
}

//==============================================================================
std::string_view CouplerConstraint::getStaticType()
{
  static constexpr std::string_view name = "CouplerConstraint";
  return name;
}

//==============================================================================
void CouplerConstraint::setConstraintForceMixing(double cfm)
{
  double clamped = cfm;
  if (clamped < 1e-9) {
    DART_WARN(
        "[CouplerConstraint::setConstraintForceMixing] Constraint force "
        "mixing parameter[{}] is lower than 1e-9. It is set to 1e-9.",
        cfm);
    clamped = 1e-9;
  }

  mConstraintForceMixing = clamped;
}

//==============================================================================
double CouplerConstraint::getConstraintForceMixing()
{
  return mConstraintForceMixing;
}

//==============================================================================
void CouplerConstraint::update()
{
  // Reset dimension
  mDim = 0;

  std::size_t dof = mJoint->getNumDofs();
  for (std::size_t i = 0; i < dof; ++i) {
    const auto& mimicProp = mMimicProps[i];

    if (mJoint->getActuatorType(i) != dynamics::Joint::MIMIC
        || mimicProp.mReferenceJoint == nullptr) {
      mActive[i] = false;
      continue;
    }

    double timeStep = mJoint->getSkeleton()->getTimeStep();
    double velLower = mJoint->getVelocityLowerLimit(i);
    double velUpper = mJoint->getVelocityUpperLimit(i);
    if (!std::isfinite(velLower))
      velLower = -kDefaultVelocityLimit;
    if (!std::isfinite(velUpper))
      velUpper = kDefaultVelocityLimit;
    double qError
        = mimicProp.mReferenceJoint->getPosition(mimicProp.mReferenceDofIndex)
              * mimicProp.mMultiplier
          + mimicProp.mOffset - mJoint->getPosition(i);
    const double erp = kDefaultErp;
    double desiredVelocity
        = math::clip((erp * qError) / timeStep, velLower, velUpper);

    mNegativeVelocityError[i] = desiredVelocity - mJoint->getVelocity(i);

    if (mNegativeVelocityError[i] != 0.0) {
      double upper = mJoint->getForceUpperLimit(i);
      double lower = mJoint->getForceLowerLimit(i);
      if (!std::isfinite(upper))
        upper = kDefaultForceLimit;
      if (!std::isfinite(lower))
        lower = -kDefaultForceLimit;

      mUpperBound[i] = upper * timeStep;
      mLowerBound[i] = lower * timeStep;

      if (mActive[i]) {
        ++(mLifeTime[i]);
      } else {
        mActive[i] = true;
        mLifeTime[i] = 0;
      }

      ++mDim;
    } else {
      mActive[i] = false;
    }
  }
}

//==============================================================================
void CouplerConstraint::getInformation(ConstraintInfo* lcp)
{
  std::size_t index = 0;
  std::size_t dof = mJoint->getNumDofs();
  for (std::size_t i = 0; i < dof; ++i) {
    if (mActive[i] == false)
      continue;

    DART_ASSERT(lcp->w[index] == 0.0);

    lcp->b[index] = mNegativeVelocityError[i];
    lcp->lo[index] = mLowerBound[i];
    lcp->hi[index] = mUpperBound[i];

    DART_ASSERT(lcp->findex[index] == -1);

    if (mLifeTime[i])
      lcp->x[index] = mOldX[i];
    else
      lcp->x[index] = 0.0;

    index++;
  }
}

//==============================================================================
void CouplerConstraint::applyUnitImpulse(std::size_t index)
{
  DART_ASSERT(index < mDim && "Invalid Index.");

  std::size_t localIndex = 0;
  const dynamics::SkeletonPtr& dependentSkeleton = mJoint->getSkeleton();

  std::size_t dof = mJoint->getNumDofs();
  for (std::size_t i = 0; i < dof; ++i) {
    if (mActive[i] == false)
      continue;

    if (localIndex == index) {
      const auto& mimicProp = mMimicProps[i];
      dynamics::Joint* referenceJoint
          = const_cast<dynamics::Joint*>(mimicProp.mReferenceJoint);
      dynamics::Skeleton* referenceSkeleton = const_cast<dynamics::Skeleton*>(
          mimicProp.mReferenceJoint->getSkeleton().get());
      dynamics::BodyNode* referenceBodyNode = const_cast<dynamics::BodyNode*>(
          mimicProp.mReferenceJoint->getChildBodyNode());

      DART_ASSERT(referenceJoint != nullptr);
      DART_ASSERT(referenceSkeleton != nullptr);
      DART_ASSERT(referenceBodyNode != nullptr);

      dependentSkeleton->clearConstraintImpulses();
      if (referenceSkeleton != dependentSkeleton.get())
        referenceSkeleton->clearConstraintImpulses();

      double impulse = 1.0;
      mJoint->setConstraintImpulse(i, impulse);

      referenceJoint->setConstraintImpulse(
          mimicProp.mReferenceDofIndex, -impulse * mimicProp.mMultiplier);

      dependentSkeleton->updateBiasImpulse(mBodyNode);
      referenceSkeleton->updateBiasImpulse(referenceBodyNode);

      dependentSkeleton->updateVelocityChange();
      referenceSkeleton->updateVelocityChange();

      mJoint->setConstraintImpulse(i, 0.0);
      referenceJoint->setConstraintImpulse(mimicProp.mReferenceDofIndex, 0.0);
    }

    ++localIndex;
  }

  mAppliedImpulseIndex = index;
}

//==============================================================================
void CouplerConstraint::getVelocityChange(double* delVel, bool withCfm)
{
  DART_ASSERT(delVel != nullptr && "Null pointer is not allowed.");

  std::size_t localIndex = 0;
  std::size_t dof = mJoint->getNumDofs();
  for (std::size_t i = 0; i < dof; ++i) {
    if (mActive[i] == false)
      continue;

    if (mJoint->getSkeleton()->isImpulseApplied())
      delVel[localIndex] = mJoint->getVelocityChange(i);
    else
      delVel[localIndex] = 0.0;

    ++localIndex;
  }

  if (withCfm) {
    delVel[mAppliedImpulseIndex]
        += delVel[mAppliedImpulseIndex] * mConstraintForceMixing;
  }

  DART_ASSERT(localIndex == mDim);
}

//==============================================================================
void CouplerConstraint::excite()
{
  mJoint->getSkeleton()->setImpulseApplied(true);
  for (const auto& mimicProp : mMimicProps) {
    const_cast<dynamics::Skeleton*>(
        mimicProp.mReferenceJoint->getSkeleton().get())
        ->setImpulseApplied(true);
  }
}

//==============================================================================
void CouplerConstraint::unexcite()
{
  mJoint->getSkeleton()->setImpulseApplied(false);
  for (const auto& mimicProp : mMimicProps) {
    const_cast<dynamics::Skeleton*>(
        mimicProp.mReferenceJoint->getSkeleton().get())
        ->setImpulseApplied(false);
  }
}

//==============================================================================
void CouplerConstraint::applyImpulse(double* lambda)
{
  std::size_t localIndex = 0;
  std::size_t dof = mJoint->getNumDofs();
  for (std::size_t i = 0; i < dof; ++i) {
    if (mActive[i] == false)
      continue;

    mJoint->setConstraintImpulse(
        i, mJoint->getConstraintImpulse(i) + lambda[localIndex]);

    auto& mimicProp = mMimicProps[i];
    dynamics::Joint* referenceJoint
        = const_cast<dynamics::Joint*>(mimicProp.mReferenceJoint);
    DART_ASSERT(referenceJoint != nullptr);
    referenceJoint->setConstraintImpulse(
        mimicProp.mReferenceDofIndex,
        referenceJoint->getConstraintImpulse(mimicProp.mReferenceDofIndex)
            - lambda[localIndex] * mimicProp.mMultiplier);

    mOldX[i] = lambda[localIndex];

    ++localIndex;
  }
}

//==============================================================================
dynamics::SkeletonPtr CouplerConstraint::getRootSkeleton() const
{
  return ConstraintBase::getRootSkeleton(mJoint->getSkeleton()->getSkeleton());
}

//==============================================================================
void CouplerConstraint::uniteSkeletons()
{
  if (mJoint == nullptr || mBodyNode == nullptr)
    return;

  if (!mBodyNode->isReactive())
    return;

  auto dependentSkeleton = mJoint->getSkeleton();
  if (!dependentSkeleton)
    return;

  auto dependentRoot = ConstraintBase::compressPath(dependentSkeleton);

  for (std::size_t i = 0; i < mJoint->getNumDofs(); ++i) {
    if (i >= mMimicProps.size())
      break;

    const auto& mimicProp = mMimicProps[i];

    if (mJoint->getActuatorType(i) != dynamics::Joint::MIMIC
        || mimicProp.mReferenceJoint == nullptr)
      continue;

    auto referenceBody = mimicProp.mReferenceJoint->getChildBodyNode();
    if (referenceBody == nullptr || !referenceBody->isReactive())
      continue;

    auto referenceSkeletonConst = mimicProp.mReferenceJoint->getSkeleton();
    if (!referenceSkeletonConst)
      continue;

    auto referenceSkeleton
        = std::const_pointer_cast<dynamics::Skeleton>(referenceSkeletonConst);
    if (!referenceSkeleton)
      continue;

    if (referenceSkeleton == dependentSkeleton)
      continue;

    auto referenceRoot = ConstraintBase::compressPath(referenceSkeleton);
    dependentRoot = ConstraintBase::compressPath(dependentSkeleton);

    if (dependentRoot == referenceRoot)
      continue;

    if (dependentRoot->mUnionSize < referenceRoot->mUnionSize) {
      dependentRoot->mUnionRootSkeleton = referenceRoot;
      referenceRoot->mUnionSize += dependentRoot->mUnionSize;
      dependentRoot = referenceRoot;
      dependentSkeleton = referenceRoot;
    } else {
      referenceRoot->mUnionRootSkeleton = dependentRoot;
      dependentRoot->mUnionSize += referenceRoot->mUnionSize;
    }
  }
}

//==============================================================================
bool CouplerConstraint::isActive() const
{
  const auto dof = std::min(mJoint->getNumDofs(), mMimicProps.size());
  for (std::size_t i = 0; i < dof; ++i) {
    if (mJoint->getActuatorType(i) == dynamics::Joint::MIMIC
        && mMimicProps[i].mReferenceJoint != nullptr) {
      return true;
    }
  }

  return false;
}

} // namespace constraint
} // namespace dart
