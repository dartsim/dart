/*
 * Copyright (c) 2011-2025, The DART development contributors
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

#include "dart/constraint/CouplerConstraint.hpp"

#include "dart/common/Logging.hpp"
#include "dart/common/Macros.hpp"
#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/Joint.hpp"
#include "dart/dynamics/Skeleton.hpp"

#include <algorithm>
#include <mutex>

#define DART_CFM 1e-9

namespace {

// Matches MimicMotorConstraint's default, so a violated mimic relationship
// settles within a few dozen steps.
constexpr double kCouplerErrorReductionParameter = 0.4;

// Caps the position-correction velocity, as JointConstraint does by default,
// unless the follower's velocity limits are tighter.
constexpr double kCouplerMaxErrorReductionVelocity = 10.0;

// Keep the public owning vector while retaining its empty storage between
// rebuilt constraints. Synchronization also covers destruction on another
// thread; each constructor copies the current properties into its own buffer.
struct CouplerMimicPropertyStorage
{
  std::mutex mutex;
  std::vector<std::vector<dart::dynamics::MimicDofProperties>> buffers;
  std::size_t bufferCount = 0u;
};

CouplerMimicPropertyStorage& getCouplerMimicPropertyStorage()
{
  static auto* storage = new CouplerMimicPropertyStorage;
  return *storage;
}

std::vector<dart::dynamics::MimicDofProperties> copyCouplerMimicProperties(
    const std::vector<dart::dynamics::MimicDofProperties>& properties)
{
  std::vector<dart::dynamics::MimicDofProperties> result;
  auto& storage = getCouplerMimicPropertyStorage();
  {
    const std::lock_guard<std::mutex> lock(storage.mutex);
    if (storage.buffers.empty()) {
      // Reserve a return slot before handing out new storage, so destruction
      // remains allocation-free when all live constraints return their vectors.
      storage.buffers.reserve(storage.bufferCount + 1u);
      ++storage.bufferCount;
    } else {
      auto buffer = std::min_element(
          storage.buffers.begin(),
          storage.buffers.end(),
          [size = properties.size()](const auto& a, const auto& b) {
            const bool aFits = a.capacity() >= size;
            const bool bFits = b.capacity() >= size;
            if (aFits != bFits)
              return aFits;
            return aFits ? a.capacity() < b.capacity()
                         : a.capacity() > b.capacity();
          });
      result.swap(*buffer);
      buffer->swap(storage.buffers.back());
      storage.buffers.pop_back();
    }
  }
  result.assign(properties.begin(), properties.end());
  return result;
}

void releaseCouplerMimicProperties(
    std::vector<dart::dynamics::MimicDofProperties>& properties)
{
  properties.clear();
  auto& storage = getCouplerMimicPropertyStorage();
  const std::lock_guard<std::mutex> lock(storage.mutex);
  // Implicit copies of the public constraint also own vectors, but do not
  // acquire a return slot. Drop excess buffers instead of growing in a
  // destructor.
  if (storage.buffers.size() < storage.buffers.capacity())
    storage.buffers.push_back(std::move(properties));
}

// Returns the leader's skeleton if the leader coordinate responds to impulses.
// An immobile or motion-prescribed leader takes no reaction impulse.
dart::dynamics::Skeleton* getCouplerReactiveReferenceSkeleton(
    const dart::dynamics::MimicDofProperties& mimicProp)
{
  const auto* body = mimicProp.mReferenceJoint->getChildBodyNode();
  if (body == nullptr || !body->isReactive())
    return nullptr;

  return const_cast<dart::dynamics::Skeleton*>(body->getSkeleton().get());
}

} // namespace

namespace dart {
namespace constraint {

double CouplerConstraint::mConstraintForceMixing = DART_CFM;

//==============================================================================
CouplerConstraint::CouplerConstraint(
    dynamics::Joint* joint,
    const std::vector<dynamics::MimicDofProperties>& mimicDofProperties)
  : ConstraintBase(),
    mJoint(joint),
    mMimicProps(copyCouplerMimicProperties(mimicDofProperties)),
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
  releaseCouplerMimicProperties(mMimicProps);
}

//==============================================================================
const std::string& CouplerConstraint::getType() const
{
  return getStaticType();
}

//==============================================================================
const std::string& CouplerConstraint::getStaticType()
{
  static const std::string name = "CouplerConstraint";
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

  const bool isFollowerReactive = mBodyNode->isReactive();
  const double timeStep = mJoint->getSkeleton()->getTimeStep();
  const std::size_t dof = mJoint->getNumDofs();
  for (std::size_t i = 0; i < dof; ++i) {
    const auto& mimicProp = mMimicProps[i];

    if (!isFollowerReactive
        || mJoint->getActuatorType(i) != dynamics::Joint::MIMIC
        || mimicProp.mReferenceJoint == nullptr) {
      mActive[i] = false;
      continue;
    }

    // Each row is the bilateral constraint J v = v_f - m v_r, driven toward
    // the scaled position error. Its unit impulse moves both joints.
    const auto* referenceJoint = mimicProp.mReferenceJoint;
    const std::size_t referenceDof = mimicProp.mReferenceDofIndex;
    const double qError
        = referenceJoint->getPosition(referenceDof) * mimicProp.mMultiplier
          + mimicProp.mOffset - mJoint->getPosition(i);
    // The correction never outpaces the follower's own velocity limits.
    const double correctionVelocity = math::clip(
        kCouplerErrorReductionParameter * qError / timeStep,
        std::max(
            mJoint->getVelocityLowerLimit(i),
            -kCouplerMaxErrorReductionVelocity),
        std::min(
            mJoint->getVelocityUpperLimit(i),
            kCouplerMaxErrorReductionVelocity));
    // An immobile leader keeps its position, whatever velocity it stores.
    const double referenceVelocity
        = referenceJoint->getSkeleton()->isMobile()
              ? referenceJoint->getVelocity(referenceDof)
              : 0.0;

    mNegativeVelocityError[i] = correctionVelocity
                                - (mJoint->getVelocity(i)
                                   - mimicProp.mMultiplier * referenceVelocity);
    mUpperBound[i] = mJoint->getForceUpperLimit(i) * timeStep;
    mLowerBound[i] = mJoint->getForceLowerLimit(i) * timeStep;

    // Satisfied rows stay active: other constraints' impulses could violate
    // them otherwise.
    if (mActive[i]) {
      ++(mLifeTime[i]);
    } else {
      mActive[i] = true;
      mLifeTime[i] = 0;
    }

    ++mDim;
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

  dynamics::Skeleton* dependentSkeleton = mJoint->getSkeleton().get();
  const std::size_t dof = mJoint->getNumDofs();

  // Every row reads its leader's response, so clear all leaders; otherwise a
  // row could read a response left over from probing another row.
  dependentSkeleton->clearConstraintImpulses();
  for (std::size_t i = 0; i < dof; ++i) {
    if (!mActive[i])
      continue;

    auto* referenceSkeleton
        = getCouplerReactiveReferenceSkeleton(mMimicProps[i]);
    if (referenceSkeleton != nullptr && referenceSkeleton != dependentSkeleton)
      referenceSkeleton->clearConstraintImpulses();
  }

  std::size_t localIndex = 0;
  for (std::size_t i = 0; i < dof; ++i) {
    if (mActive[i] == false)
      continue;

    if (localIndex == index) {
      const auto& mimicProp = mMimicProps[i];
      auto* referenceJoint
          = const_cast<dynamics::Joint*>(mimicProp.mReferenceJoint);
      auto* referenceSkeleton = getCouplerReactiveReferenceSkeleton(mimicProp);

      const double impulse = 1.0;
      mJoint->setConstraintImpulse(i, impulse);
      dependentSkeleton->updateBiasImpulse(mBodyNode);
      if (referenceSkeleton != nullptr) {
        referenceJoint->setConstraintImpulse(
            mimicProp.mReferenceDofIndex, -impulse * mimicProp.mMultiplier);
        referenceSkeleton->updateBiasImpulse(
            referenceJoint->getChildBodyNode());
      }

      dependentSkeleton->updateVelocityChange();
      if (referenceSkeleton != nullptr
          && referenceSkeleton != dependentSkeleton)
        referenceSkeleton->updateVelocityChange();

      mJoint->setConstraintImpulse(i, 0.0);
      if (referenceSkeleton != nullptr)
        referenceJoint->setConstraintImpulse(mimicProp.mReferenceDofIndex, 0.0);
      break;
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

    // J dv = dv_f - m dv_r, reading only skeletons the probe excited.
    double velocityChange = 0.0;
    if (mJoint->getSkeleton()->isImpulseApplied())
      velocityChange = mJoint->getVelocityChange(i);

    const auto& mimicProp = mMimicProps[i];
    const auto* referenceSkeleton
        = getCouplerReactiveReferenceSkeleton(mimicProp);
    if (referenceSkeleton != nullptr && referenceSkeleton->isImpulseApplied()) {
      velocityChange -= mimicProp.mMultiplier
                        * mimicProp.mReferenceJoint->getVelocityChange(
                            mimicProp.mReferenceDofIndex);
    }

    delVel[localIndex] = velocityChange;
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
  for (std::size_t i = 0; i < mJoint->getNumDofs(); ++i) {
    if (!mActive[i])
      continue;

    if (auto* skeleton = getCouplerReactiveReferenceSkeleton(mMimicProps[i]))
      skeleton->setImpulseApplied(true);
  }
}

//==============================================================================
void CouplerConstraint::unexcite()
{
  mJoint->getSkeleton()->setImpulseApplied(false);
  for (std::size_t i = 0; i < mJoint->getNumDofs(); ++i) {
    if (!mActive[i])
      continue;

    if (auto* skeleton = getCouplerReactiveReferenceSkeleton(mMimicProps[i]))
      skeleton->setImpulseApplied(false);
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

    const auto& mimicProp = mMimicProps[i];
    if (getCouplerReactiveReferenceSkeleton(mimicProp) != nullptr) {
      auto* referenceJoint
          = const_cast<dynamics::Joint*>(mimicProp.mReferenceJoint);
      referenceJoint->setConstraintImpulse(
          mimicProp.mReferenceDofIndex,
          referenceJoint->getConstraintImpulse(mimicProp.mReferenceDofIndex)
              - lambda[localIndex] * mimicProp.mMultiplier);
    }

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

  for (const auto& mimicProp : mMimicProps) {
    if (mimicProp.mReferenceJoint == nullptr)
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
  return mDim != 0;
}

} // namespace constraint
} // namespace dart

#undef DART_CFM
