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
 *   * This code incorporates portions of Open Dynamics Engine
 *     (Copyright (c) 2001-2004, Russell L. Smith. All rights
 *     reserved.) and portions of FCL (Copyright (c) 2011, Willow
 *     Garage, Inc. All rights reserved.), which were released under
 *     the same BSD license as below
 *
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

#include "dart/simulation/World.hpp"

#include "dart/collision/CollisionDetector.hpp"
#include "dart/collision/CollisionFilter.hpp"
#include "dart/collision/CollisionGroup.hpp"
#include "dart/collision/fcl/FCLCollisionDetector.hpp"
#include "dart/common/Console.hpp"
#include "dart/common/Logging.hpp"
#include "dart/common/Macros.hpp"
#include "dart/common/Profile.hpp"
#include "dart/common/String.hpp"
#include "dart/constraint/BoxedLcpConstraintSolver.hpp"
#include "dart/constraint/ConstrainedGroup.hpp"
#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/DegreeOfFreedom.hpp"
#include "dart/dynamics/FreeJoint.hpp"
#include "dart/dynamics/Skeleton.hpp"

#include <algorithm>
#include <condition_variable>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <type_traits>
#include <typeinfo>
#include <vector>

#include <cmath>

namespace dart {
namespace simulation {

namespace {

using dart::collision::CollisionDetector;
using dart::collision::CollisionDetectorPtr;

constexpr double kFinalSleepLinearRatio = 0.1;
constexpr double kFinalSleepAngularRatio = 0.2;

std::string toCollisionDetectorKey(CollisionDetectorType type)
{
  switch (type) {
    case CollisionDetectorType::Dart:
      return "dart";
    case CollisionDetectorType::Fcl:
      return "fcl";
    case CollisionDetectorType::Bullet:
      return "bullet";
    case CollisionDetectorType::Ode:
      return "ode";
  }

  DART_FATAL(
      "Encountered unsupported CollisionDetectorType value: {}.",
      static_cast<int>(type));
  return "fcl";
}

CollisionDetectorPtr tryCreateCollisionDetector(const std::string& requestedKey)
{
  if (requestedKey.empty())
    return nullptr;

  auto key = common::toLower(requestedKey);

  auto* factory = CollisionDetector::getFactory();
  DART_ASSERT(factory);
  if (!factory->canCreate(key))
    return nullptr;

  auto detector = factory->create(key);
  if (!detector) {
    DART_WARN(
        "Failed to create collision detector '{}' even though the factory "
        "reported it was available.",
        key);
  }

  return detector;
}

CollisionDetectorPtr tryCreateCollisionDetector(CollisionDetectorType type)
{
  return tryCreateCollisionDetector(toCollisionDetectorKey(type));
}

const dynamics::BodyNode* getRootBodyNodeIfAny(
    const dynamics::Skeleton& skeleton)
{
  if (skeleton.getNumTrees() == 0u)
    return nullptr;

  return skeleton.getRootBodyNode();
}

dynamics::BodyNode* getRootBodyNodeIfAny(dynamics::Skeleton& skeleton)
{
  if (skeleton.getNumTrees() == 0u)
    return nullptr;

  return skeleton.getRootBodyNode();
}

void copySkeletonPositions(
    const dynamics::Skeleton& skeleton, Eigen::VectorXd& positions)
{
  const auto numDofs = static_cast<Eigen::Index>(skeleton.getNumDofs());
  positions.resize(numDofs);
  for (Eigen::Index i = 0; i < numDofs; ++i)
    positions[i] = skeleton.getPosition(static_cast<std::size_t>(i));
}

bool skeletonPositionsMatch(
    const dynamics::Skeleton& skeleton, const Eigen::VectorXd& positions)
{
  const auto numDofs = static_cast<Eigen::Index>(skeleton.getNumDofs());
  if (positions.size() != numDofs)
    return false;

  for (Eigen::Index i = 0; i < numDofs; ++i) {
    if (skeleton.getPosition(static_cast<std::size_t>(i)) != positions[i])
      return false;
  }

  return true;
}

// Resolves a collision detector for a World constructed from a WorldConfig.
//
// Returns nullptr when the requested detector is the default 'fcl' backend so
// the World keeps the constraint solver's existing default detector untouched.
// This keeps the default World construction path byte-for-byte equivalent to
// the pre-WorldConfig behavior (FCL with PRIMITIVE shapes), which downstream
// consumers such as gz-physics rely on. Only an explicitly non-default request
// triggers a detector swap.
CollisionDetectorPtr resolveCollisionDetector(const WorldConfig& config)
{
  const auto requestedType = config.collisionDetector;

  // Default request: leave the constraint solver's default detector in place.
  if (requestedType == CollisionDetectorType::Fcl)
    return nullptr;

  const auto requestedKey = toCollisionDetectorKey(requestedType);
  if (auto detector = tryCreateCollisionDetector(requestedType))
    return detector;

  DART_WARN(
      "WorldConfig requested collision detector '{}', but it is not "
      "available. Keeping the world's default collision detector.",
      requestedKey);
  return nullptr;
}

void findShallowSupportedFreeRoots(
    const std::vector<dynamics::SkeletonPtr>& skeletons,
    const collision::CollisionResult& contacts,
    const Eigen::Vector3d& gravity,
    std::vector<char>& supported)
{
  supported.clear();
  supported.resize(skeletons.size(), false);

  const double gravityNorm = gravity.norm();
  if (contacts.getNumContacts() == 0u || gravityNorm <= 0.0)
    return;

  const Eigen::Vector3d up = -gravity / gravityNorm;
  constexpr double kSupportContactPenetrationTolerance = 1e-4;
  constexpr double kSupportNormalMinVerticalComponent = 0.5;

  const auto findSkeletonIndex
      = [&](const dynamics::Skeleton* skeleton) -> std::size_t {
    for (std::size_t i = 0; i < skeletons.size(); ++i) {
      if (skeletons[i].get() == skeleton)
        return i;
    }
    return skeletons.size();
  };

  auto markIfSupportedRoot = [&](const auto& bodyNode,
                                 const auto& supportBodyNode,
                                 const Eigen::Vector3d& normal,
                                 double penetrationDepth) {
    // Mirror the constraint solver's contact gating: a contact with a
    // non-finite or negative penetration depth (e.g. a Bullet proximity hit
    // kept by allowNegativePenetrationDepthContacts) never creates a contact
    // constraint, so it cannot inject the Baumgarte correction this
    // suppression compensates for.
    if (!bodyNode || !supportBodyNode || !std::isfinite(penetrationDepth)
        || penetrationDepth < 0.0
        || penetrationDepth > kSupportContactPenetrationTolerance) {
      return;
    }

    const auto* skeleton = bodyNode->getSkeletonRawPtr();
    if (skeleton == nullptr || !skeleton->isMobile())
      return;

    if (bodyNode != getRootBodyNodeIfAny(*skeleton))
      return;

    const auto* supportSkeleton = supportBodyNode->getSkeletonRawPtr();
    const bool supportRestingAndUnperturbed
        = supportSkeleton != nullptr && supportSkeleton->isResting()
          && !supportSkeleton->isImpulseApplied();
    const bool supportInactive = supportSkeleton == nullptr
                                 || !supportSkeleton->isMobile()
                                 || supportRestingAndUnperturbed;
    if (!supportInactive)
      return;

    const double normalNorm = normal.norm();
    if (!normal.allFinite() || normalNorm <= 0.0)
      return;

    const double verticalComponent = normal.dot(up) / normalNorm;
    if (verticalComponent < kSupportNormalMinVerticalComponent)
      return;

    // Same relative-height guard as the resting support check: only a contact
    // that supports the root from below qualifies. A shallow contact with the
    // underside of a ceiling or overhang must not clamp legitimate small
    // lateral or tilt motion.
    const double bodyHeightAboveSupport
        = (bodyNode->getTransform().translation()
           - supportBodyNode->getTransform().translation())
              .dot(up);
    if (bodyHeightAboveSupport < -kSupportContactPenetrationTolerance)
      return;

    const auto index = findSkeletonIndex(skeleton);
    if (index < supported.size())
      supported[index] = true;
  };

  for (std::size_t i = 0; i < contacts.getNumContacts(); ++i) {
    const auto& contact = contacts.getContact(i);
    const auto bodyNode1 = contact.getBodyNodePtr1();
    const auto bodyNode2 = contact.getBodyNodePtr2();
    markIfSupportedRoot(
        bodyNode1, bodyNode2, contact.normal, contact.penetrationDepth);
    markIfSupportedRoot(
        bodyNode2, bodyNode1, -contact.normal, contact.penetrationDepth);
  }
}

bool hasFiniteNonzeroVelocity(const Eigen::Vector3d& velocity)
{
  return velocity.allFinite() && velocity.squaredNorm() > 0.0;
}

void restoreVelocityActuatorCommands(
    dynamics::FreeJoint& joint, const Eigen::VectorXd& commands)
{
  for (std::size_t i = 0; i < joint.getNumDofs(); ++i) {
    if (joint.getActuatorType(i) != dynamics::Joint::VELOCITY)
      continue;

    const auto index = static_cast<Eigen::Index>(i);
    if (index < commands.size() && joint.getCommand(i) != commands[index])
      joint.setCommand(i, commands[index]);
  }
}

common::MemoryAllocator& resolveWorldMemoryBaseAllocator(
    const WorldConfig& config)
{
  return config.baseAllocator ? *config.baseAllocator
                              : common::MemoryAllocator::GetDefault();
}

common::MemoryManager::Options makeWorldMemoryManagerOptions(
    const WorldConfig& config)
{
  common::MemoryManager::Options options;
  options.freeListInitialAllocation = config.freeListInitialAllocation;
  options.freeListGrowthPolicy = config.freeListGrowthPolicy;
  options.frameAllocatorInitialCapacity = config.frameScratchInitialCapacity;
  return options;
}

} // namespace

//==============================================================================
static bool hasNonzeroGeneralizedVelocity(const dynamics::Skeleton& skel)
{
  for (std::size_t i = 0; i < skel.getNumDofs(); ++i) {
    const auto* dof = skel.getDof(i);
    if (dof && dof->getVelocity() != 0.0)
      return true;
  }

  return false;
}

//==============================================================================
void World::invalidateSimulationMode()
{
  mSimulationMode = false;
  mSimulationModeStructuralVersion = 0;
}

//==============================================================================
void World::refreshSkeletonDofIndices()
{
  mIndices.clear();
  mIndices.reserve(mSkeletons.size() + 1u);
  mIndices.push_back(0);

  for (const auto& skeleton : mSkeletons) {
    const auto numDofs = skeleton ? skeleton->getNumDofs() : 0u;
    mIndices.push_back(mIndices.back() + static_cast<int>(numDofs));
  }
}

//==============================================================================
void World::reserveMemoryManagerForSimulationShape()
{
  DART_ASSERT(mMemoryManager != nullptr);
  if (!mMemoryManager)
    return;

  const std::size_t numSkeletons = mSkeletons.size();
  const std::size_t numSimpleFrames = mSimpleFrames.size();
  const std::size_t numDofs
      = mIndices.empty() ? 0u : static_cast<std::size_t>(mIndices.back());
  const std::size_t numLastContacts
      = mConstraintSolver
            ? mConstraintSolver->getLastCollisionResult().getNumContacts()
            : 0u;
  const std::size_t contactCapacity
      = std::max(numLastContacts, numSkeletons * 4u);

  const std::size_t freeListReservation = std::max(
      mMemoryManagerFreeListInitialAllocation,
      4096u + numSkeletons * 4096u + numSimpleFrames * 512u + numDofs * 512u
          + contactCapacity * 1024u);
  if (void* memory = mMemoryManager->allocateUsingFree(freeListReservation)) {
    mMemoryManager->deallocateUsingFree(memory, freeListReservation);
  }

  auto& frameAllocator = mMemoryManager->getFrameAllocator();
  const std::size_t frameReservation = std::max(
      mMemoryManagerFrameScratchInitialCapacity,
      4096u + numSkeletons * 1024u + numSimpleFrames * 256u + numDofs * 256u
          + contactCapacity * 512u);
  if (frameAllocator.usableCapacity() < frameReservation)
    (void)frameAllocator.allocate(frameReservation);
  frameAllocator.reset();

  mPreSolveFreeRootVelocityScratch.reserve(numSkeletons);
  mShallowSupportedFreeRootScratch.reserve(numSkeletons);
  mDisturbedThisStepScratch.reserve(numSkeletons);
  mDeepInitialContactSkeletonScratch.reserve(numSkeletons);
  mSupportedInitialContactSkeletonScratch.reserve(numSkeletons);
}

//==============================================================================
void World::enterSimulationMode()
{
  if (isInSimulationMode())
    return;

  refreshSkeletonDofIndices();
  syncShallowSupportFreeRootVelocityStates();
  reserveMemoryManagerForSimulationShape();
  if (mConstraintSolver)
    mConstraintSolver->prepareForSimulation();
  reserveMemoryManagerForSimulationShape();
  mSimulationModeStructuralVersion
      = dynamics::Skeleton::getGlobalStructuralVersion();
  mSimulationMode = true;
}

//==============================================================================
bool World::isInSimulationMode() const
{
  return mSimulationMode
         && mSimulationModeStructuralVersion
                == dynamics::Skeleton::getGlobalStructuralVersion();
}

//==============================================================================
common::MemoryManager& World::getMemoryManager()
{
  DART_ASSERT(mMemoryManager != nullptr);
  return *mMemoryManager;
}

//==============================================================================
const common::MemoryManager& World::getMemoryManager() const
{
  DART_ASSERT(mMemoryManager != nullptr);
  return *mMemoryManager;
}

//==============================================================================
void World::syncShallowSupportFreeRootVelocityStates()
{
  mShallowSupportFreeRootVelocityStates.resize(mSkeletons.size());

  for (std::size_t i = 0; i < mSkeletons.size(); ++i) {
    auto& state = mShallowSupportFreeRootVelocityStates[i];
    const auto* skeleton = mSkeletons[i].get();
    if (state.mSkeleton == skeleton)
      continue;

    state = ShallowSupportFreeRootVelocityState{};
    state.mSkeleton = skeleton;
  }
}

//==============================================================================
const std::vector<World::FreeRootVelocitySnapshot>&
World::snapshotFreeRootVelocities()
{
  syncShallowSupportFreeRootVelocityStates();

  mPreSolveFreeRootVelocityScratch.clear();
  mPreSolveFreeRootVelocityScratch.resize(mSkeletons.size());

  for (std::size_t i = 0; i < mSkeletons.size(); ++i) {
    const auto& skeleton = mSkeletons[i];
    if (!skeleton || !skeleton->isMobile())
      continue;

    const auto* rootBody = getRootBodyNodeIfAny(*skeleton);
    if (rootBody == nullptr)
      continue;

    const auto* freeJoint
        = dynamic_cast<const dynamics::FreeJoint*>(rootBody->getParentJoint());
    if (freeJoint == nullptr)
      continue;

    const auto& state = mShallowSupportFreeRootVelocityStates[i];
    auto& snapshot = mPreSolveFreeRootVelocityScratch[i];
    snapshot.mSkeleton = skeleton.get();
    snapshot.mValid = true;
    snapshot.mVelocityEditedSinceLastStep
        = skeleton->getVelocityVersion() != state.mObservedVelocityVersion;
    snapshot.mExternallyDisturbed = skeleton->hasExternalDisturbance();
    snapshot.mLinear = rootBody->getLinearVelocity();
    snapshot.mAngular = rootBody->getAngularVelocity();
  }

  return mPreSolveFreeRootVelocityScratch;
}

//==============================================================================
void World::clearUnsupportedShallowSupportFreeRootVelocityStates(
    const std::vector<char>& shallowSupportedFreeRoots,
    const std::vector<FreeRootVelocitySnapshot>& preSolveVelocities)
{
  syncShallowSupportFreeRootVelocityStates();

  const bool canStoreUnsupportedVelocities = mGravity.squaredNorm() > 0.0;
  Eigen::Vector3d up = Eigen::Vector3d::Zero();
  if (canStoreUnsupportedVelocities)
    up = -mGravity.normalized();

  for (std::size_t i = 0; i < mShallowSupportFreeRootVelocityStates.size();
       ++i) {
    if (i < shallowSupportedFreeRoots.size() && shallowSupportedFreeRoots[i])
      continue;

    auto& state = mShallowSupportFreeRootVelocityStates[i];
    state.mPreserveLateralVelocity = false;
    state.mPreserveTiltVelocity = false;
    state.mLateralVelocity.setZero();
    state.mTiltVelocity.setZero();
    state.mHasUnsupportedLateralVelocity = false;
    state.mHasUnsupportedTiltVelocity = false;
    state.mHasResetLateralVelocity = false;
    state.mHasResetTiltVelocity = false;
    state.mUnsupportedLateralVelocity.setZero();
    state.mUnsupportedTiltVelocity.setZero();
    state.mResetLateralVelocity.setZero();
    state.mResetTiltVelocity.setZero();

    if (!canStoreUnsupportedVelocities || i >= preSolveVelocities.size())
      continue;

    const auto& snapshot = preSolveVelocities[i];
    if (!snapshot.mValid || snapshot.mSkeleton != state.mSkeleton)
      continue;

    const Eigen::Vector3d verticalVelocity = up * snapshot.mLinear.dot(up);
    const Eigen::Vector3d lateralVelocity = snapshot.mLinear - verticalVelocity;
    if (hasFiniteNonzeroVelocity(lateralVelocity)) {
      state.mHasUnsupportedLateralVelocity = true;
      state.mUnsupportedLateralVelocity = lateralVelocity;
    }

    const Eigen::Vector3d yawVelocity = up * snapshot.mAngular.dot(up);
    const Eigen::Vector3d tiltVelocity = snapshot.mAngular - yawVelocity;
    if (hasFiniteNonzeroVelocity(tiltVelocity)) {
      state.mHasUnsupportedTiltVelocity = true;
      state.mUnsupportedTiltVelocity = tiltVelocity;
    }
  }
}

//==============================================================================
void World::updateShallowSupportFreeRootVelocityVersions()
{
  syncShallowSupportFreeRootVelocityStates();

  for (std::size_t i = 0; i < mSkeletons.size(); ++i) {
    const auto& skeleton = mSkeletons[i];
    if (skeleton)
      mShallowSupportFreeRootVelocityStates[i].mObservedVelocityVersion
          = skeleton->getVelocityVersion();
  }
}

//==============================================================================
void World::captureResetShallowSupportFreeRootVelocityTargets()
{
  syncShallowSupportFreeRootVelocityStates();

  if (mGravity.squaredNorm() == 0.0)
    return;

  const Eigen::Vector3d up = -mGravity.normalized();
  for (std::size_t i = 0; i < mSkeletons.size(); ++i) {
    const auto& skeleton = mSkeletons[i];
    if (!skeleton || !skeleton->isMobile())
      continue;

    const auto* rootBody = getRootBodyNodeIfAny(*skeleton);
    if (rootBody == nullptr)
      continue;

    const auto* freeJoint
        = dynamic_cast<const dynamics::FreeJoint*>(rootBody->getParentJoint());
    if (freeJoint == nullptr)
      continue;

    auto& state = mShallowSupportFreeRootVelocityStates[i];
    const Eigen::Vector3d linearVelocity = rootBody->getLinearVelocity();
    const Eigen::Vector3d verticalVelocity = up * linearVelocity.dot(up);
    const Eigen::Vector3d lateralVelocity = linearVelocity - verticalVelocity;
    if (hasFiniteNonzeroVelocity(lateralVelocity)) {
      state.mHasResetLateralVelocity = true;
      state.mResetLateralVelocity = lateralVelocity;
    }

    const Eigen::Vector3d angularVelocity = rootBody->getAngularVelocity();
    const Eigen::Vector3d yawVelocity = up * angularVelocity.dot(up);
    const Eigen::Vector3d tiltVelocity = angularVelocity - yawVelocity;
    if (hasFiniteNonzeroVelocity(tiltVelocity)) {
      state.mHasResetTiltVelocity = true;
      state.mResetTiltVelocity = tiltVelocity;
    }
  }
}

// DART 6 keeps Baumgarte contact correction in the velocity solve by default.
// On shallow support contacts, small contact-manifold asymmetry can leak that
// vertical correction into lateral free-root velocity or roll/pitch drift. Keep
// the observable upward correction, yaw, and explicitly seeded pre-solve
// lateral/tilt motion, but remove only tiny lateral/tilt deltas introduced by
// support contacts.
//==============================================================================
void World::suppressShallowSupportedFreeRootDrift(
    const dynamics::SkeletonPtr& skeleton,
    const Eigen::Vector3d& gravity,
    const FreeRootVelocitySnapshot& preSolveVelocity,
    ShallowSupportFreeRootVelocityState& state)
{
  state.mSkeleton = skeleton.get();

  if (!skeleton || !skeleton->isMobile() || gravity.squaredNorm() <= 0.0)
    return;

  auto* rootBody = getRootBodyNodeIfAny(*skeleton);
  if (rootBody == nullptr)
    return;

  auto* freeJoint
      = dynamic_cast<dynamics::FreeJoint*>(rootBody->getParentJoint());
  if (freeJoint == nullptr)
    return;

  if (!preSolveVelocity.mValid
      || preSolveVelocity.mSkeleton != skeleton.get()) {
    state.mPreserveLateralVelocity = false;
    state.mPreserveTiltVelocity = false;
    state.mLateralVelocity.setZero();
    state.mTiltVelocity.setZero();
    return;
  }

  const Eigen::Vector3d up = -gravity.normalized();
  constexpr double kRootLinearDriftSpeedCap = 2e-4;
  constexpr double kRootLinearDriftFinalQuietRatio = 0.5;
  // Use the configured final-quiet threshold when deactivation is enabled, so
  // callers can tighten this gate by tuning DeactivationOptions. The legacy
  // always-awake path has no threshold, so it keeps the bounded platform-jitter
  // cap that absorbs FreeBSD VM-backed cross-axis Baumgarte leakage.
  double rootLinearDriftSpeed = kRootLinearDriftSpeedCap;
  if (mDeactivationOptions.mEnabled) {
    const double finalQuietLinearSpeed = std::max(
        0.0,
        kFinalSleepLinearRatio * mDeactivationOptions.mLinearSpeedThreshold);
    rootLinearDriftSpeed = std::min(
        kRootLinearDriftSpeedCap,
        kRootLinearDriftFinalQuietRatio * finalQuietLinearSpeed);
  }
  constexpr double kRootAngularDriftSpeed = 5e-4;
  const bool preSolveClearsStoredTarget
      = preSolveVelocity.mVelocityEditedSinceLastStep
        || preSolveVelocity.mExternallyDisturbed;
  Eigen::VectorXd velocityActuatorCommands;
  bool restoreCommands = false;

  auto captureVelocityActuatorCommands = [&]() {
    if (restoreCommands)
      return;

    velocityActuatorCommands = freeJoint->getCommands();
    restoreCommands = true;
  };

  Eigen::Vector3d linearVelocity = rootBody->getLinearVelocity();
  const Eigen::Vector3d verticalVelocity = up * linearVelocity.dot(up);
  const Eigen::Vector3d lateralVelocity = linearVelocity - verticalVelocity;
  const Eigen::Vector3d preSolveVerticalVelocity
      = up * preSolveVelocity.mLinear.dot(up);
  const Eigen::Vector3d preSolveLateralVelocity
      = preSolveVelocity.mLinear - preSolveVerticalVelocity;
  Eigen::Vector3d targetLateralVelocity = Eigen::Vector3d::Zero();
  bool targetPreservesLateralVelocity = false;
  // Unedited shallow-contact residuals inside the same platform drift band are
  // contact leakage, not a new intentional baseline.
  const bool preservePreSolveLateralVelocity
      = hasFiniteNonzeroVelocity(preSolveLateralVelocity)
        && (preSolveClearsStoredTarget
            || preSolveLateralVelocity.norm() > rootLinearDriftSpeed);
  if (preservePreSolveLateralVelocity) {
    targetLateralVelocity = preSolveLateralVelocity;
    targetPreservesLateralVelocity = true;
  } else if (
      !preSolveClearsStoredTarget && state.mPreserveLateralVelocity
      && state.mLateralVelocity.allFinite()) {
    targetLateralVelocity = state.mLateralVelocity;
    targetPreservesLateralVelocity
        = hasFiniteNonzeroVelocity(targetLateralVelocity);
  } else if (
      !preSolveClearsStoredTarget && state.mHasUnsupportedLateralVelocity
      && state.mUnsupportedLateralVelocity.allFinite()) {
    targetLateralVelocity = state.mUnsupportedLateralVelocity;
    targetPreservesLateralVelocity
        = hasFiniteNonzeroVelocity(targetLateralVelocity);
  } else if (
      !preSolveClearsStoredTarget && state.mHasResetLateralVelocity
      && state.mResetLateralVelocity.allFinite()) {
    targetLateralVelocity = state.mResetLateralVelocity;
    targetPreservesLateralVelocity
        = hasFiniteNonzeroVelocity(targetLateralVelocity);
  }

  const bool clampedLateralVelocity
      = targetLateralVelocity.allFinite()
        && (lateralVelocity - targetLateralVelocity).norm()
               <= rootLinearDriftSpeed;
  if (clampedLateralVelocity) {
    captureVelocityActuatorCommands();
    freeJoint->setLinearVelocity(
        verticalVelocity + targetLateralVelocity,
        dynamics::Frame::World(),
        dynamics::Frame::World());
  }

  state.mPreserveLateralVelocity
      = clampedLateralVelocity && targetPreservesLateralVelocity;
  state.mLateralVelocity = state.mPreserveLateralVelocity
                               ? targetLateralVelocity
                               : Eigen::Vector3d::Zero();

  Eigen::Vector3d angularVelocity = rootBody->getAngularVelocity();
  const Eigen::Vector3d yawVelocity = up * angularVelocity.dot(up);
  const Eigen::Vector3d tiltVelocity = angularVelocity - yawVelocity;
  const Eigen::Vector3d preSolveYawVelocity
      = up * preSolveVelocity.mAngular.dot(up);
  const Eigen::Vector3d preSolveTiltVelocity
      = preSolveVelocity.mAngular - preSolveYawVelocity;
  Eigen::Vector3d targetTiltVelocity = Eigen::Vector3d::Zero();
  bool targetPreservesTiltVelocity = false;
  if (preSolveClearsStoredTarget) {
    if (hasFiniteNonzeroVelocity(preSolveTiltVelocity)) {
      targetTiltVelocity = preSolveTiltVelocity;
      targetPreservesTiltVelocity = true;
    }
  } else if (
      !preSolveClearsStoredTarget && state.mPreserveTiltVelocity
      && state.mTiltVelocity.allFinite()) {
    targetTiltVelocity = state.mTiltVelocity;
    targetPreservesTiltVelocity = hasFiniteNonzeroVelocity(targetTiltVelocity);
  } else if (
      !preSolveClearsStoredTarget && state.mHasUnsupportedTiltVelocity
      && state.mUnsupportedTiltVelocity.allFinite()) {
    targetTiltVelocity = state.mUnsupportedTiltVelocity;
    targetPreservesTiltVelocity = hasFiniteNonzeroVelocity(targetTiltVelocity);
  } else if (
      !preSolveClearsStoredTarget && state.mHasResetTiltVelocity
      && state.mResetTiltVelocity.allFinite()) {
    targetTiltVelocity = state.mResetTiltVelocity;
    targetPreservesTiltVelocity = hasFiniteNonzeroVelocity(targetTiltVelocity);
  }

  const bool clampedTiltVelocity
      = targetTiltVelocity.allFinite()
        && (tiltVelocity - targetTiltVelocity).norm() <= kRootAngularDriftSpeed;
  if (clampedTiltVelocity) {
    captureVelocityActuatorCommands();
    freeJoint->setAngularVelocity(
        yawVelocity + targetTiltVelocity,
        dynamics::Frame::World(),
        dynamics::Frame::World());
  }

  state.mPreserveTiltVelocity
      = clampedTiltVelocity && targetPreservesTiltVelocity;
  state.mTiltVelocity = state.mPreserveTiltVelocity ? targetTiltVelocity
                                                    : Eigen::Vector3d::Zero();

  if (restoreCommands)
    restoreVelocityActuatorCommands(*freeJoint, velocityActuatorCommands);

  state.mHasUnsupportedLateralVelocity = false;
  state.mHasUnsupportedTiltVelocity = false;
  state.mHasResetLateralVelocity = false;
  state.mHasResetTiltVelocity = false;
  state.mUnsupportedLateralVelocity.setZero();
  state.mUnsupportedTiltVelocity.setZero();
  state.mResetLateralVelocity.setZero();
  state.mResetTiltVelocity.setZero();
}

//==============================================================================
class SimulationThreadPool
{
public:
  SimulationThreadPool() = default;

  ~SimulationThreadPool()
  {
    setWorkerCount(0u);
  }

  void setWorkerCount(std::size_t workerCount)
  {
    if (workerCount == mWorkers.size())
      return;

    stopWorkers();
    if (workerCount == 0u)
      return;

    {
      std::lock_guard<std::mutex> lock(mMutex);
      mStop = false;
    }

    mWorkers.reserve(workerCount);
    for (std::size_t i = 0; i < workerCount; ++i) {
      mWorkers.emplace_back([this] { workerLoop(); });
    }
  }

  template <typename Func>
  void parallelFor(std::size_t count, std::size_t numThreads, Func&& func)
  {
    if (count == 0u)
      return;

    const std::size_t totalParticipants = std::min<std::size_t>(
        std::min<std::size_t>(numThreads, count), mWorkers.size() + 1u);
    if (totalParticipants <= 1u) {
      for (std::size_t i = 0; i < count; ++i)
        func(i);
      return;
    }

    const std::size_t chunkSize
        = (count + totalParticipants - 1u) / totalParticipants;
    const std::size_t workerCount = totalParticipants - 1u;
    using Function = typename std::remove_reference<Func>::type;

    {
      std::lock_guard<std::mutex> lock(mMutex);
      DART_ASSERT(!mTaskActive);
      mTaskActive = true;
      mTaskCallable = static_cast<void*>(std::addressof(func));
      mTaskInvoker = [](void* callable, std::size_t begin, std::size_t end) {
        auto& task = *static_cast<Function*>(callable);
        for (std::size_t i = begin; i < end; ++i)
          task(i);
      };
      mTaskCount = count;
      mTaskChunkSize = chunkSize;
      mWorkerLimit = totalParticipants;
      mNextWorkerIndex = 1u;
      mActiveWorkerCount = workerCount;
      ++mTaskGeneration;
    }

    mTaskCv.notify_all();

    const std::size_t mainEnd = std::min<std::size_t>(count, chunkSize);
    for (std::size_t i = 0; i < mainEnd; ++i)
      func(i);

    {
      std::unique_lock<std::mutex> lock(mMutex);
      mDoneCv.wait(lock, [this] { return mActiveWorkerCount == 0u; });
      mTaskActive = false;
      mTaskCallable = nullptr;
      mTaskInvoker = nullptr;
      mWorkerLimit = 1u;
    }
  }

private:
  using TaskInvoker = void (*)(void*, std::size_t, std::size_t);

  void stopWorkers()
  {
    {
      std::lock_guard<std::mutex> lock(mMutex);
      mStop = true;
      ++mTaskGeneration;
    }

    mTaskCv.notify_all();

    for (auto& worker : mWorkers) {
      if (worker.joinable())
        worker.join();
    }
    mWorkers.clear();

    std::lock_guard<std::mutex> lock(mMutex);
    mStop = false;
    mTaskActive = false;
    mTaskCallable = nullptr;
    mTaskInvoker = nullptr;
    mActiveWorkerCount = 0u;
    mWorkerLimit = 1u;
    mNextWorkerIndex = 1u;
  }

  void workerLoop()
  {
    std::size_t observedGeneration = 0u;

    while (true) {
      void* callable = nullptr;
      TaskInvoker invoker = nullptr;
      std::size_t begin = 0u;
      std::size_t end = 0u;

      {
        std::unique_lock<std::mutex> lock(mMutex);
        mTaskCv.wait(lock, [&] {
          return mStop || observedGeneration != mTaskGeneration;
        });

        if (mStop)
          return;

        observedGeneration = mTaskGeneration;
        if (!mTaskActive || mNextWorkerIndex >= mWorkerLimit)
          continue;

        const std::size_t workerIndex = mNextWorkerIndex++;
        begin = workerIndex * mTaskChunkSize;
        end = std::min<std::size_t>(mTaskCount, begin + mTaskChunkSize);
        callable = mTaskCallable;
        invoker = mTaskInvoker;
      }

      if (begin < end)
        invoker(callable, begin, end);

      {
        std::lock_guard<std::mutex> lock(mMutex);
        DART_ASSERT(mActiveWorkerCount > 0u);
        --mActiveWorkerCount;
        if (mActiveWorkerCount == 0u)
          mDoneCv.notify_one();
      }
    }
  }

  std::vector<std::thread> mWorkers;
  std::mutex mMutex;
  std::condition_variable mTaskCv;
  std::condition_variable mDoneCv;
  bool mStop = false;
  bool mTaskActive = false;
  std::size_t mTaskGeneration = 0u;
  std::size_t mTaskCount = 0u;
  std::size_t mTaskChunkSize = 0u;
  std::size_t mWorkerLimit = 1u;
  std::size_t mNextWorkerIndex = 1u;
  std::size_t mActiveWorkerCount = 0u;
  void* mTaskCallable = nullptr;
  TaskInvoker mTaskInvoker = nullptr;
};

//==============================================================================
template <typename Func>
void parallelForIndexRange(
    SimulationThreadPool* threadPool,
    std::size_t count,
    std::size_t numThreads,
    Func&& func)
{
  if (count == 0u)
    return;

  if (threadPool == nullptr || numThreads <= 1u || count < 128u) {
    for (std::size_t i = 0; i < count; ++i)
      func(i);
    return;
  }

  threadPool->parallelFor(count, numThreads, std::forward<Func>(func));
}

//==============================================================================
std::shared_ptr<World> World::create(const std::string& name)
{
  return std::make_shared<World>(name);
}

//==============================================================================
std::shared_ptr<World> World::create(const WorldConfig& config)
{
  return std::make_shared<World>(config);
}

//==============================================================================
World::World(const std::string& _name) : World(WorldConfig(_name)) {}

//==============================================================================
World::World(const WorldConfig& config)
  : mName(config.name),
    mNameMgrForSkeletons("World::Skeleton | " + config.name, "skeleton"),
    mNameMgrForSimpleFrames("World::SimpleFrame | " + config.name, "frame"),
    mGravity(0.0, 0.0, -9.81),
    mTimeStep(0.001),
    mTime(0.0),
    mFrame(0),
    mMemoryManager(std::make_unique<common::MemoryManager>(
        resolveWorldMemoryBaseAllocator(config),
        makeWorldMemoryManagerOptions(config))),
    mMemoryManagerFreeListInitialAllocation(config.freeListInitialAllocation),
    mMemoryManagerFrameScratchInitialCapacity(
        config.frameScratchInitialCapacity),
    mRecording(new Recording(mSkeletons)),
    onNameChanged(mNameChangedSignal)
{
  mIndices.push_back(0);

  auto solver = std::make_unique<constraint::BoxedLcpConstraintSolver>();
  setConstraintSolver(std::move(solver));

  // Only swap the collision detector when a non-default backend is requested.
  // The default 'fcl' request resolves to nullptr so the constraint solver's
  // default detector (FCL with PRIMITIVE shapes) is preserved unchanged.
  if (auto detector = resolveCollisionDetector(config))
    setCollisionDetector(detector);
}

//==============================================================================
World::~World()
{
  delete mRecording;

  for (common::Connection& connection : mNameConnectionsForSkeletons)
    connection.disconnect();

  for (common::Connection& connection : mNameConnectionsForSimpleFrames)
    connection.disconnect();
}

//==============================================================================
WorldPtr World::clone() const
{
  WorldPtr worldClone = World::create(mName);

  worldClone->setGravity(mGravity);
  worldClone->setTimeStep(mTimeStep);
  worldClone->setDeactivationOptions(mDeactivationOptions);
  worldClone->setNumSimulationThreads(mNumSimulationThreads);

  auto cd = getConstraintSolver()->getCollisionDetector();
  if (cd) {
    worldClone->setCollisionDetector(cd->cloneWithoutCollisionObjects());
  }

  // Clone and add each Skeleton
  for (std::size_t i = 0; i < mSkeletons.size(); ++i) {
    worldClone->addSkeleton(mSkeletons[i]->cloneSkeleton());
  }

  // Clone and add each SimpleFrame
  for (std::size_t i = 0; i < mSimpleFrames.size(); ++i) {
    worldClone->addSimpleFrame(
        mSimpleFrames[i]->clone(mSimpleFrames[i]->getParentFrame()));
  }

  // For each newly cloned SimpleFrame, try to make its parent Frame be one of
  // the new clones if there is a match. This is meant to minimize any possible
  // interdependencies between the kinematics of different worlds.
  for (std::size_t i = 0; i < worldClone->getNumSimpleFrames(); ++i) {
    dynamics::Frame* current_parent
        = worldClone->getSimpleFrame(i)->getParentFrame();

    dynamics::SimpleFramePtr parent_candidate
        = worldClone->getSimpleFrame(current_parent->getName());

    if (parent_candidate)
      worldClone->getSimpleFrame(i)->setParentFrame(parent_candidate.get());
  }

  return worldClone;
}

//==============================================================================
void World::setTimeStep(double _timeStep)
{
  if (!std::isfinite(_timeStep) || _timeStep <= 0.0) {
    dtwarn << "[World] Attempting to set invalid timestep (" << _timeStep
           << "). Ignoring this "
           << "request because it can lead to undefined behavior.\n";
    return;
  }

  mTimeStep = _timeStep;
  DART_ASSERT(mConstraintSolver);
  mConstraintSolver->setTimeStep(_timeStep);
  for (auto& skel : mSkeletons)
    skel->setTimeStep(_timeStep);
  invalidateAllRestingKinematicSnapshot();
}

//==============================================================================
double World::getTimeStep() const
{
  return mTimeStep;
}

//==============================================================================
void World::setNumSimulationThreads(std::size_t numThreads)
{
  if (numThreads == 0u) {
    numThreads = std::thread::hardware_concurrency();
    if (numThreads == 0u)
      numThreads = 1u;
  }

  mNumSimulationThreads = std::max<std::size_t>(1u, numThreads);
  if (mConstraintSolver)
    mConstraintSolver->setNumSimulationThreads(mNumSimulationThreads);

  if (mNumSimulationThreads <= 1u) {
    mSimulationThreadPool.reset();
    return;
  }

  if (!mSimulationThreadPool)
    mSimulationThreadPool = std::make_unique<SimulationThreadPool>();
  mSimulationThreadPool->setWorkerCount(mNumSimulationThreads - 1u);
}

//==============================================================================
std::size_t World::getNumSimulationThreads() const
{
  return mNumSimulationThreads;
}

//==============================================================================
void World::reset()
{
  mTime = 0.0;
  mFrame = 0;
  mRecording->clear();
  mConstraintSolver->clearLastCollisionResult();
  invalidateAllRestingKinematicSnapshot();
  invalidateLastStepRestingWorldState();
  clearUnsupportedShallowSupportFreeRootVelocityStates({});
  updateShallowSupportFreeRootVelocityVersions();
  captureResetShallowSupportFreeRootVelocityTargets();

  for (auto& skel : mSkeletons) {
    skel->clearConstraintImpulses();
    skel->setImpulseApplied(false);
  }
}

//==============================================================================
void World::step(bool _resetCommand)
{
  DART_PROFILE_FRAME;

  if (!isInSimulationMode())
    enterSimulationMode();
  getMemoryManager().getFrameAllocator().reset();

  const bool deactivationEnabled = mDeactivationOptions.mEnabled;

  // Mirror the enable flag onto the solver so its island rest-detection / LCP
  // skipping runs only when the feature is on (otherwise it is a strict no-op).
  mConstraintSolver->setDeactivationActive(deactivationEnabled);
  if (deactivationEnabled)
    wakeRestingSkeletonsIfStepStateChanged();

  if (!deactivationEnabled) {
    {
      DART_PROFILE_SCOPED_N("World::step - Integrate velocity");
      parallelForIndexRange(
          mSimulationThreadPool.get(),
          mSkeletons.size(),
          mNumSimulationThreads,
          [&](std::size_t i) {
            auto& skel = mSkeletons[i];
            if (!skel->isMobile())
              return;

            skel->computeForwardDynamics();
            skel->integrateVelocities(mTimeStep);
          });
    }

    const auto& preSolveFreeRootVelocities = snapshotFreeRootVelocities();

    {
      DART_PROFILE_SCOPED_N("World::step - Solve constraints");
      mConstraintSolver->solve();
    }

    findShallowSupportedFreeRoots(
        mSkeletons,
        mConstraintSolver->getLastCollisionResult(),
        mGravity,
        mShallowSupportedFreeRootScratch);
    clearUnsupportedShallowSupportFreeRootVelocityStates(
        mShallowSupportedFreeRootScratch, preSolveFreeRootVelocities);

    {
      DART_PROFILE_SCOPED_N("World::step - Integrate positions");
      parallelForIndexRange(
          mSimulationThreadPool.get(),
          mSkeletons.size(),
          mNumSimulationThreads,
          [&](std::size_t i) {
            auto& skel = mSkeletons[i];
            if (!skel->isMobile())
              return;

            if (skel->isImpulseApplied()) {
              skel->computeImpulseForwardDynamics();
              skel->setImpulseApplied(false);
            }

            if (i < mShallowSupportedFreeRootScratch.size()
                && mShallowSupportedFreeRootScratch[i]) {
              suppressShallowSupportedFreeRootDrift(
                  skel,
                  mGravity,
                  preSolveFreeRootVelocities[i],
                  mShallowSupportFreeRootVelocityStates[i]);
            }

            if (skel->isPositionImpulseApplied()) {
              skel->integratePositions(
                  mTimeStep, skel->getPositionVelocityChanges());
              skel->setPositionImpulseApplied(false);
              skel->clearPositionVelocityChanges();
            } else {
              skel->integratePositions(mTimeStep);
            }

            if (_resetCommand && skel->checkExternalDisturbanceAndReset(true)) {
              skel->clearInternalForces();
              skel->clearExternalForces();
              skel->resetCommands();
            }
          });
    }

    mTime += mTimeStep;
    mFrame++;
    updateShallowSupportFreeRootVelocityVersions();
    invalidateAllRestingKinematicSnapshot();
    invalidateLastStepRestingWorldState();
    return;
  }

  const bool lastStepHadNoContacts
      = mConstraintSolver->getLastCollisionResult().getNumContacts() == 0;
  bool allRestingFastPathReady = false;
  if (deactivationEnabled && lastStepHadNoContacts) {
    bool allRestingSnapshotStale = false;
    allRestingFastPathReady
        = isAllRestingFastPathReady(_resetCommand, &allRestingSnapshotStale);
    if (allRestingSnapshotStale)
      wakeRestingSkeletonsForWorldChange();
  }

  if (deactivationEnabled && lastStepHadNoContacts && allRestingFastPathReady) {
    DART_PROFILE_SCOPED_N("World::step - All-resting fast path");
    mTime += mTimeStep;
    mFrame++;
    updateShallowSupportFreeRootVelocityVersions();
    if (!mLastStepRestingWorldStateValid)
      updateLastStepRestingWorldState();
    return;
  }

  // Tracks, per skeleton, whether it was disturbed (woken or kept awake) this
  // step so the post-step rest-detection pass can treat it as non-quiet even
  // after _resetCommand clears the actuation vectors below. Disturbance
  // tracking is only needed while contact islands exist; unconstrained active
  // scenes cannot sleep.
  auto& disturbedThisStep = mDisturbedThisStepScratch;
  disturbedThisStep.clear();
  const bool trackDisturbances
      = deactivationEnabled
        && mConstraintSolver->getLastCollisionResult().getNumContacts() > 0;
  {
    DART_PROFILE_SCOPED_N("World::step - Prepare deactivation");
    if (trackDisturbances)
      disturbedThisStep.assign(mSkeletons.size(), false);
  }

  // Integrate velocity for unconstrained skeletons
  {
    DART_PROFILE_SCOPED_N("World::step - Integrate velocity");
    parallelForIndexRange(
        mSimulationThreadPool.get(),
        mSkeletons.size(),
        mNumSimulationThreads,
        [&](std::size_t i) {
          auto& skel = mSkeletons[i];
          if (!skel->isMobile())
            return;

          const bool externallyDisturbed
              = deactivationEnabled && (trackDisturbances || skel->isResting())
                && skel->hasExternalDisturbance();
          if (externallyDisturbed && !disturbedThisStep.empty())
            disturbedThisStep[i] = 1;

          // A resting skeleton skips forward dynamics (including gravity)
          // unless a user-applied force/command is pending this step, in which
          // case it is woken so the actuation is integrated rather than
          // silently dropped. Gravity alone is not a disturbance, so a body
          // resting on its support stays asleep.
          if (deactivationEnabled && skel->isResting()) {
            if (externallyDisturbed || hasNonzeroGeneralizedVelocity(*skel)) {
              skel->setResting(false);
              skel->setSleepCandidate(false);
              if (!disturbedThisStep.empty())
                disturbedThisStep[i] = 1;
            } else {
              if (_resetCommand) {
                skel->clearInternalForces();
                skel->clearExternalForces();
                skel->resetCommands();
              }
              return;
            }
          }

          skel->computeForwardDynamics();
          skel->integrateVelocities(mTimeStep);

          // A sleep candidate can briefly lose its support contact due to
          // discrete collision jitter. Preserve that candidacy only while the
          // integrated velocity is still inside the wake band; if the body is
          // genuinely moving again, clear the stale candidate before the
          // solver has a chance to freeze its next contact island.
          if (skel->isSleepCandidate()) {
            const double linWake = mDeactivationOptions.mWakeThresholdScale
                                   * mDeactivationOptions.mLinearSpeedThreshold;
            const double angWake
                = mDeactivationOptions.mWakeThresholdScale
                  * mDeactivationOptions.mAngularSpeedThreshold;
            const double linSpeed = skel->computeMaxBodyLinearSpeed();
            const double angSpeed = skel->computeMaxBodyAngularSpeed();
            if (externallyDisturbed || linSpeed > linWake
                || angSpeed > angWake) {
              skel->setSleepCandidate(false);
              if (!disturbedThisStep.empty())
                disturbedThisStep[i] = 1;
            }
          }
        });
  }

  const auto& preSolveFreeRootVelocities = snapshotFreeRootVelocities();

  // Detect activated constraints and compute constraint impulses
  {
    DART_PROFILE_SCOPED_N("World::step - Solve constraints");
    mConstraintSolver->solve();
  }

  findShallowSupportedFreeRoots(
      mSkeletons,
      mConstraintSolver->getLastCollisionResult(),
      mGravity,
      mShallowSupportedFreeRootScratch);
  clearUnsupportedShallowSupportFreeRootVelocityStates(
      mShallowSupportedFreeRootScratch, preSolveFreeRootVelocities);

  {
    DART_PROFILE_SCOPED_N("World::step - Integrate positions");
    parallelForIndexRange(
        mSimulationThreadPool.get(),
        mSkeletons.size(),
        mNumSimulationThreads,
        [&](std::size_t i) {
          auto& skel = mSkeletons[i];
          if (!skel->isMobile())
            return;

          bool preservingFinalSleepSolve = false;

          // A resting skeleton skips both the impulse-based forward dynamics
          // and the position integration. The only exception is when an impulse
          // was applied to it this step: that means a moving body contacted
          // this island (the solver united them so the group was not
          // all-resting and was solved), delivering the wake-up impulse to the
          // previously-resting bodies. Such a skeleton must wake and be
          // processed normally so the impulse is applied and the position is
          // advanced. A newly sleep-eligible contact island also receives one
          // final solved impulse before freezing so
          // transmitted-wrench/body-force queries keep the last solved contact
          // forces; that path processes the impulse but preserves the resting
          // state.
          if (deactivationEnabled && skel->isResting()) {
            if (skel->isImpulseApplied()) {
              if (skel->isSleepCandidate()) {
                preservingFinalSleepSolve = true;
              } else {
                skel->setResting(false);
                skel->setSleepCandidate(false);
                if (!disturbedThisStep.empty())
                  disturbedThisStep[i] = 1;
              }
            } else {
              return;
            }
          }

          if (skel->isImpulseApplied()) {
            skel->computeImpulseForwardDynamics();
            skel->setImpulseApplied(false);
          }

          if (i < mShallowSupportedFreeRootScratch.size()
              && mShallowSupportedFreeRootScratch[i]) {
            suppressShallowSupportedFreeRootDrift(
                skel,
                mGravity,
                preSolveFreeRootVelocities[i],
                mShallowSupportFreeRootVelocityStates[i]);
          }

          if (!preservingFinalSleepSolve) {
            if (skel->isPositionImpulseApplied()) {
              skel->integratePositions(
                  mTimeStep, skel->getPositionVelocityChanges());
              skel->setPositionImpulseApplied(false);
              skel->clearPositionVelocityChanges();
            } else {
              skel->integratePositions(mTimeStep);
            }
          }

          if (preservingFinalSleepSolve && skel->getNumDofs() > 0) {
            skel->setVelocities(
                Eigen::VectorXd::Zero(static_cast<int>(skel->getNumDofs())));
            skel->computeForwardKinematics(false, true, false);
          }

          if (_resetCommand) {
            skel->clearInternalForces();
            skel->clearExternalForces();
            skel->resetCommands();
          }
        });
  }

  // Rest-detection pass: decide which mobile skeletons are quiet enough to
  // sleep, and wake any that have started moving again. This is a deterministic
  // function of post-step cached speeds, dwell time, and the configured
  // thresholds, so it does not depend on container or iteration order.
  if (deactivationEnabled
      && mConstraintSolver->getLastCollisionResult().getNumContacts() > 0)
    updateRestStates(disturbedThisStep);

  mTime += mTimeStep;
  mFrame++;
  updateShallowSupportFreeRootVelocityVersions();

  if (deactivationEnabled
      && mConstraintSolver->getLastCollisionResult().getNumContacts() == 0) {
    updateAllRestingKinematicSnapshot(_resetCommand);
  } else {
    invalidateAllRestingKinematicSnapshot();
  }

  if (deactivationEnabled)
    updateLastStepRestingWorldState();
  else
    invalidateLastStepRestingWorldState();
}

//==============================================================================
void World::updateRestStates(const std::vector<char>& disturbedThisStep)
{
  DART_PROFILE_SCOPED_N("World::step - Rest detection");

  const double linSleep = mDeactivationOptions.mLinearSpeedThreshold;
  const double angSleep = mDeactivationOptions.mAngularSpeedThreshold;
  const double scale = mDeactivationOptions.mWakeThresholdScale;
  const double linWake = scale * linSleep;
  const double angWake = scale * angSleep;

  // EMA weight for the speed smoothing. Resting-contact solves emit small
  // per-step velocity jitter; smoothing the speed lets a genuinely settled
  // island fall below the sleep band and accumulate dwell, while a body in real
  // sustained motion keeps a high smoothed speed and never qualifies.
  const double alpha = 0.2;

  // This pass updates per-skeleton sleep candidacy. The constraint solver
  // handles active contact islands; the final contact-result pass below handles
  // quiet candidate bodies whose static support contact produced no active
  // constraint on this step.
  constexpr double kSleepContactPenetrationTolerance = 1e-5;
  // Final-quiet gate for sleep candidacy, expressed as fixed fractions of the
  // configured thresholds. At the default thresholds (0.01 m/s, 0.05 rad/s)
  // these evaluate to the previous hardcoded 1e-3 m/s and 1e-2 rad/s, so
  // default behavior is unchanged. Deriving the gate from the configured
  // thresholds keeps them effective: raising the thresholds for scenes with a
  // higher contact-solver jitter floor (e.g. dense mixed-shape piles) widens
  // the gate proportionally instead of being silently overridden by a hidden
  // stricter constant.
  const double finalSleepLinearSpeed = kFinalSleepLinearRatio * linSleep;
  const double finalSleepAngularSpeed = kFinalSleepAngularRatio * angSleep;
  constexpr double kSupportNormalMinVerticalComponent = 0.5;
  const auto& contacts = mConstraintSolver->getLastCollisionResult();
  const double gravityNorm = mGravity.norm();
  Eigen::Vector3d up = Eigen::Vector3d::Zero();
  if (gravityNorm > 0.0)
    up = -mGravity / gravityNorm;

  auto isShallowSupportContact = [&](const auto& bodyNode,
                                     const auto& supportBodyNode,
                                     const Eigen::Vector3d& normal) {
    if (!bodyNode || !supportBodyNode || gravityNorm <= 0.0)
      return false;

    const auto* skeleton = bodyNode->getSkeletonRawPtr();
    if (skeleton == nullptr || !skeleton->isMobile())
      return false;

    const auto* supportSkeleton = supportBodyNode->getSkeletonRawPtr();
    const bool supportInactive = supportSkeleton == nullptr
                                 || !supportSkeleton->isMobile()
                                 || supportSkeleton->isResting();
    if (!supportInactive)
      return false;

    const double normalNorm = normal.norm();
    if (normalNorm <= 0.0)
      return false;

    const double verticalComponent = std::abs(normal.dot(up)) / normalNorm;
    if (verticalComponent < kSupportNormalMinVerticalComponent)
      return false;

    const double bodyHeightAboveSupport
        = (bodyNode->getTransform().translation()
           - supportBodyNode->getTransform().translation())
              .dot(up);
    return bodyHeightAboveSupport >= -kSleepContactPenetrationTolerance;
  };

  auto& deepInitialContactSkeletons = mDeepInitialContactSkeletonScratch;
  auto& supportedInitialContactSkeletons
      = mSupportedInitialContactSkeletonScratch;
  deepInitialContactSkeletons.clear();
  supportedInitialContactSkeletons.clear();
  const auto containsSkeleton
      = [](const std::vector<const dynamics::Skeleton*>& skeletons,
           const dynamics::Skeleton* skeleton) {
          return std::find(skeletons.begin(), skeletons.end(), skeleton)
                 != skeletons.end();
        };
  if (mFrame == 0) {
    for (std::size_t i = 0; i < contacts.getNumContacts(); ++i) {
      const auto& contact = contacts.getContact(i);
      auto markMobileSkeleton = [&](auto bodyNode, auto& skeletons) {
        if (!bodyNode)
          return;
        const auto* skeleton = bodyNode->getSkeletonRawPtr();
        if (skeleton != nullptr && skeleton->isMobile()) {
          if (!containsSkeleton(skeletons, skeleton))
            skeletons.push_back(skeleton);
        }
      };

      const auto bodyNode1 = contact.getBodyNodePtr1();
      const auto bodyNode2 = contact.getBodyNodePtr2();
      if (contact.penetrationDepth > kSleepContactPenetrationTolerance) {
        markMobileSkeleton(bodyNode1, deepInitialContactSkeletons);
        markMobileSkeleton(bodyNode2, deepInitialContactSkeletons);
        continue;
      }

      if (isShallowSupportContact(bodyNode1, bodyNode2, contact.normal))
        markMobileSkeleton(bodyNode1, supportedInitialContactSkeletons);
      if (isShallowSupportContact(bodyNode2, bodyNode1, contact.normal))
        markMobileSkeleton(bodyNode2, supportedInitialContactSkeletons);
    }
  }

  for (std::size_t i = 0; i < mSkeletons.size(); ++i) {
    auto& skel = mSkeletons[i];
    if (!skel->isMobile())
      continue;

    const bool disturbed
        = (i < disturbedThisStep.size() && disturbedThisStep[i])
          || skel->hasExternalDisturbance();

    const bool islanded = skel->getIslandIndex() >= 0;
    if (!islanded)
      skel->setResting(false);

    if (islanded && skel->isResting() && skel->isSleepCandidate()
        && !disturbed) {
      skel->setSmoothedLinearSpeed(0.0);
      skel->setSmoothedAngularSpeed(0.0);
      continue;
    }

    const double linInstant = skel->computeMaxBodyLinearSpeed();
    const double angInstant = skel->computeMaxBodyAngularSpeed();
    const double linSpeed
        = (1.0 - alpha) * skel->getSmoothedLinearSpeed() + alpha * linInstant;
    const double angSpeed
        = (1.0 - alpha) * skel->getSmoothedAngularSpeed() + alpha * angInstant;
    skel->setSmoothedLinearSpeed(linSpeed);
    skel->setSmoothedAngularSpeed(angSpeed);

    if (skel->isSleepCandidate()) {
      // Stay a candidate unless the body clearly started moving again (beyond
      // the wider wake band) or was disturbed/impulsed this step. The wake band
      // is larger than the sleep band (hysteresis) to avoid candidacy
      // thrashing.
      if (disturbed || linSpeed > linWake || angSpeed > angWake) {
        skel->setSleepCandidate(false);
      }
    } else {
      const bool canAccumulateDwell = islanded || skel->isSleepCandidate()
                                      || skel->getRestDwellTime() > 0.0;
      const bool quiet = canAccumulateDwell && (linSpeed < linSleep)
                         && (angSpeed < angSleep) && !disturbed;
      if (quiet) {
        const bool finalQuiet = linSpeed < finalSleepLinearSpeed
                                && angSpeed < finalSleepAngularSpeed;
        double dwell = skel->getRestDwellTime() + mTimeStep;
        const bool deepInitialContact
            = containsSkeleton(deepInitialContactSkeletons, skel.get());
        const bool supportedInitialContact
            = containsSkeleton(supportedInitialContactSkeletons, skel.get());
        // The first-frame shortcut credits the full dwell only to bodies that
        // are essentially stationary at load time (pre-settled imported
        // scenes). It deliberately keeps these near-zero fixed bounds instead
        // of the threshold-scaled candidacy gate above, so raising the
        // thresholds cannot skip the configured dwell for a supported body
        // with real initial motion.
        constexpr double kInitialRestMaxLinearSpeed = 1e-3;
        constexpr double kInitialRestMaxAngularSpeed = 1e-2;
        const bool initialRestQuiet = linSpeed < kInitialRestMaxLinearSpeed
                                      && angSpeed < kInitialRestMaxAngularSpeed;
        if (mFrame == 0 && islanded && initialRestQuiet && !deepInitialContact
            && supportedInitialContact) {
          dwell = std::max(dwell, mDeactivationOptions.mTimeUntilSleep);
        }
        skel->setRestDwellTime(dwell);
        if (dwell >= mDeactivationOptions.mTimeUntilSleep && finalQuiet) {
          skel->setSleepCandidate(true);
        }
      } else {
        skel->setRestDwellTime(0.0);
      }
    }
  }

  // Collision backends can report a persistent, shallow static support contact
  // without producing an active contact constraint every frame. If a body has
  // already accumulated the full quiet dwell and is touching only an inactive
  // support, let it sleep instead of requiring a non-zero LCP island on that
  // exact frame. Moving or disturbed candidates are cleared before this point
  // by the wake-band checks above and in the pre-solve velocity pass.
  for (std::size_t i = 0; i < contacts.getNumContacts(); ++i) {
    const auto& contact = contacts.getContact(i);
    if (contact.penetrationDepth > kSleepContactPenetrationTolerance)
      continue;

    auto tryRestOnInactiveSupport = [&](const auto& bodyNode,
                                        const auto& supportBodyNode) {
      if (!bodyNode)
        return;

      auto* skel
          = const_cast<dynamics::Skeleton*>(bodyNode->getSkeletonRawPtr());
      if (!skel || !skel->isMobile() || skel->isResting()
          || !skel->isSleepCandidate() || skel->getIslandIndex() >= 0) {
        return;
      }

      const auto* supportSkel
          = supportBodyNode ? supportBodyNode->getSkeletonRawPtr() : nullptr;
      const bool supportInactive = supportSkel == nullptr
                                   || !supportSkel->isMobile()
                                   || supportSkel->isResting();
      if (!supportInactive)
        return;

      if (!isShallowSupportContact(bodyNode, supportBodyNode, contact.normal))
        return;

      if (skel->getSmoothedLinearSpeed() > linWake
          || skel->getSmoothedAngularSpeed() > angWake) {
        skel->setSleepCandidate(false);
        return;
      }

      skel->setIslandIndex(0);
      skel->setResting(true);
    };

    const auto bodyNode1 = contact.getBodyNodePtr1();
    const auto bodyNode2 = contact.getBodyNodePtr2();
    tryRestOnInactiveSupport(bodyNode1, bodyNode2);
    tryRestOnInactiveSupport(bodyNode2, bodyNode1);
  }

  constexpr double kZeroSpeedForContactMissSleep = 1e-10;
  for (auto& skel : mSkeletons) {
    if (!skel->isMobile() || skel->isResting() || !skel->isSleepCandidate()
        || skel->getIslandIndex() >= 0) {
      continue;
    }

    if (skel->getRestDwellTime() < mDeactivationOptions.mTimeUntilSleep)
      continue;

    if (skel->computeMaxBodyLinearSpeed() > kZeroSpeedForContactMissSleep
        || skel->computeMaxBodyAngularSpeed() > kZeroSpeedForContactMissSleep) {
      continue;
    }

    skel->setIslandIndex(0);
    skel->setResting(true);
  }
}

//==============================================================================
bool World::isAllRestingFastPathReady(bool _resetCommand, bool* snapshotStale)
{
  DART_PROFILE_SCOPED_N("all-resting readiness check");

  if (snapshotStale)
    *snapshotStale = false;

  auto markSnapshotStale = [&]() {
    if (snapshotStale)
      *snapshotStale = true;
  };

  const double linWake = mDeactivationOptions.mWakeThresholdScale
                         * mDeactivationOptions.mLinearSpeedThreshold;
  const double angWake = mDeactivationOptions.mWakeThresholdScale
                         * mDeactivationOptions.mAngularSpeedThreshold;

  auto restingSkeletonNeedsWake = [&](const dynamics::SkeletonPtr& skel) {
    if (!skel->isResting() || !skel->isSleepCandidate()
        || skel->getIslandIndex() < 0
        || skel->checkExternalDisturbanceAndReset(_resetCommand)) {
      return true;
    }

    if (skel->computeMaxBodyLinearSpeed() > linWake
        || skel->computeMaxBodyAngularSpeed() > angWake) {
      markSnapshotStale();
      return true;
    }

    if (hasNonzeroGeneralizedVelocity(*skel)) {
      markSnapshotStale();
      return true;
    }

    return false;
  };

  if (!mAllRestingKinematicSnapshotValid) {
    for (const auto& skel : mSkeletons) {
      if (skel->isMobile() && skel->isResting()) {
        markSnapshotStale();
        break;
      }
    }
    return false;
  }

  if (mAllRestingKinematicSnapshot.size() != mSkeletons.size()) {
    markSnapshotStale();
    return false;
  }

  const auto& collisionOption = mConstraintSolver->getCollisionOption();
  const auto* collisionFilter = collisionOption.collisionFilter.get();
  if (!isCollisionFilterSnapshotTrackable(collisionFilter)) {
    invalidateAllRestingKinematicSnapshot();
    return false;
  }

  const auto collisionFilterRevision
      = getCollisionFilterSnapshotRevision(collisionFilter);
  const bool collisionFilterUnchanged
      = mAllRestingSnapshotCollisionFilter == collisionFilter
        && mAllRestingSnapshotCollisionFilterRevision
               == collisionFilterRevision;
  const bool collisionOptionScalarsUnchanged
      = mAllRestingSnapshotCollisionEnableContact
            == collisionOption.enableContact
        && mAllRestingSnapshotCollisionMaxNumContacts
               == collisionOption.maxNumContacts
        && mAllRestingSnapshotCollisionMaxNumContactsPerPair
               == collisionOption.maxNumContactsPerPair
        && mAllRestingSnapshotCollisionAllowNegativePenetrationDepthContacts
               == collisionOption.allowNegativePenetrationDepthContacts;
  const auto collisionDetector = mConstraintSolver->getCollisionDetector();
  const auto collisionGroup = mConstraintSolver->getCollisionGroup();
  const bool collisionDetectorUnchanged
      = mAllRestingSnapshotCollisionDetector == collisionDetector.get()
        && mAllRestingSnapshotCollisionGroup == collisionGroup.get()
        && mAllRestingSnapshotCollisionGroupVersion
               == collisionGroup->getContentVersion();

  if (mAllRestingSnapshotReady && mAllRestingSnapshotHasMobileSkeleton
      && mAllRestingSnapshotStructuralVersion
             == dynamics::Skeleton::getGlobalStructuralVersion()
      && mAllRestingSnapshotKinematicVersion
             == dynamics::Skeleton::getGlobalKinematicVersion()
      && mAllRestingSnapshotExternalDisturbanceVersion
             == dynamics::Skeleton::getGlobalExternalDisturbanceVersion()
      && mAllRestingSnapshotDeactivationStateVersion
             == dynamics::Skeleton::getGlobalDeactivationStateVersion()
      && mAllRestingSnapshotVelocityVersion
             == dynamics::Skeleton::getGlobalVelocityVersion()
      && collisionDetectorUnchanged && collisionFilterUnchanged
      && collisionOptionScalarsUnchanged) {
    if (_resetCommand && !mAllRestingSnapshotResetCommand) {
      for (const auto& skel : mSkeletons) {
        if (!skel->isMobile())
          continue;

        if (skel->checkExternalDisturbanceAndReset(true)) {
          markSnapshotStale();
          return false;
        }
      }
      mAllRestingSnapshotResetCommand = true;
    }

    return true;
  }

  if (!collisionDetectorUnchanged || !collisionFilterUnchanged
      || !collisionOptionScalarsUnchanged) {
    markSnapshotStale();
    return false;
  }

  if (mAllRestingSnapshotDeactivationStateVersion
      != dynamics::Skeleton::getGlobalDeactivationStateVersion()) {
    markSnapshotStale();
    return false;
  }

  bool hasMobileSkeleton = false;
  for (std::size_t i = 0; i < mSkeletons.size(); ++i) {
    auto& skel = mSkeletons[i];
    auto& snapshot = mAllRestingKinematicSnapshot[i];
    if (snapshot.mSkeleton != skel.get()
        || snapshot.mNumBodyNodes != skel->getNumBodyNodes()) {
      markSnapshotStale();
      return false;
    }

    if (snapshot.mStructuralVersion != skel->getVersion()
        || snapshot.mKinematicVersion != skel->getKinematicVersion()) {
      bool kinematicStateUnchanged
          = skeletonPositionsMatch(*skel, snapshot.mPositions)
            && snapshot.mBodyTransforms.size() == skel->getNumBodyNodes();
      if (kinematicStateUnchanged) {
        for (std::size_t j = 0; j < skel->getNumBodyNodes(); ++j) {
          const auto* bodyNode = skel->getBodyNode(j);
          if (bodyNode == nullptr
              || !bodyNode->getTransform().matrix().isApprox(
                  snapshot.mBodyTransforms[j].matrix(), 0.0)) {
            kinematicStateUnchanged = false;
            break;
          }
        }
      }

      if (!kinematicStateUnchanged) {
        markSnapshotStale();
        return false;
      }

      snapshot.mStructuralVersion = skel->getVersion();
      snapshot.mKinematicVersion = skel->getKinematicVersion();
      copySkeletonPositions(*skel, snapshot.mPositions);
    }

    if (!skel->isMobile())
      continue;

    hasMobileSkeleton = true;
    if (restingSkeletonNeedsWake(skel)) {
      return false;
    }
  }

  if (hasMobileSkeleton) {
    mAllRestingSnapshotReady = true;
    mAllRestingSnapshotResetCommand = _resetCommand;
    updateAllRestingSnapshotGlobalVersions();
  } else {
    mAllRestingSnapshotReady = false;
    mAllRestingSnapshotResetCommand = true;
  }

  return hasMobileSkeleton;
}

//==============================================================================
void World::updateAllRestingKinematicSnapshot(bool _resetCommand)
{
  const auto& collisionOption = mConstraintSolver->getCollisionOption();
  if (!isCollisionFilterSnapshotTrackable(
          collisionOption.collisionFilter.get())) {
    invalidateAllRestingKinematicSnapshot();
    return;
  }

  mAllRestingKinematicSnapshot.resize(mSkeletons.size());
  mAllRestingSnapshotHasMobileSkeleton = false;
  mAllRestingSnapshotReady = false;
  mAllRestingSnapshotResetCommand = _resetCommand;

  for (std::size_t skelIndex = 0; skelIndex < mSkeletons.size(); ++skelIndex) {
    const auto& skel = mSkeletons[skelIndex];
    auto& snapshot = mAllRestingKinematicSnapshot[skelIndex];
    snapshot.mSkeleton = skel.get();
    snapshot.mStructuralVersion = skel->getVersion();
    snapshot.mKinematicVersion = skel->getKinematicVersion();
    snapshot.mNumBodyNodes = skel->getNumBodyNodes();
    copySkeletonPositions(*skel, snapshot.mPositions);
    snapshot.mBodyTransforms.clear();
    snapshot.mBodyTransforms.reserve(snapshot.mNumBodyNodes);
    for (std::size_t i = 0; i < snapshot.mNumBodyNodes; ++i)
      snapshot.mBodyTransforms.push_back(skel->getBodyNode(i)->getTransform());
    mAllRestingSnapshotHasMobileSkeleton
        = mAllRestingSnapshotHasMobileSkeleton || skel->isMobile();
  }

  updateAllRestingSnapshotGlobalVersions();
  mAllRestingKinematicSnapshotValid = true;
}

//==============================================================================
void World::updateAllRestingSnapshotGlobalVersions()
{
  mAllRestingSnapshotStructuralVersion
      = dynamics::Skeleton::getGlobalStructuralVersion();
  mAllRestingSnapshotKinematicVersion
      = dynamics::Skeleton::getGlobalKinematicVersion();
  mAllRestingSnapshotExternalDisturbanceVersion
      = dynamics::Skeleton::getGlobalExternalDisturbanceVersion();
  mAllRestingSnapshotDeactivationStateVersion
      = dynamics::Skeleton::getGlobalDeactivationStateVersion();
  mAllRestingSnapshotVelocityVersion
      = dynamics::Skeleton::getGlobalVelocityVersion();
  const auto collisionDetector = mConstraintSolver->getCollisionDetector();
  mAllRestingSnapshotCollisionDetector = collisionDetector.get();
  const auto collisionGroup = mConstraintSolver->getCollisionGroup();
  mAllRestingSnapshotCollisionGroup = collisionGroup.get();
  mAllRestingSnapshotCollisionGroupVersion
      = collisionGroup->getContentVersion();
  const auto& collisionOption = mConstraintSolver->getCollisionOption();
  mAllRestingSnapshotCollisionEnableContact = collisionOption.enableContact;
  mAllRestingSnapshotCollisionMaxNumContacts = collisionOption.maxNumContacts;
  mAllRestingSnapshotCollisionMaxNumContactsPerPair
      = collisionOption.maxNumContactsPerPair;
  mAllRestingSnapshotCollisionAllowNegativePenetrationDepthContacts
      = collisionOption.allowNegativePenetrationDepthContacts;
  mAllRestingSnapshotCollisionFilter = collisionOption.collisionFilter.get();
  mAllRestingSnapshotCollisionFilterRevision
      = getCollisionFilterSnapshotRevision(mAllRestingSnapshotCollisionFilter);
}

//==============================================================================
bool World::isCollisionFilterSnapshotTrackable(
    const collision::CollisionFilter* filter) const
{
  if (filter == nullptr)
    return true;

  return typeid(*filter) == typeid(collision::BodyNodeCollisionFilter);
}

//==============================================================================
std::size_t World::getCollisionFilterSnapshotRevision(
    const collision::CollisionFilter* filter) const
{
  const auto* bodyNodeFilter
      = dynamic_cast<const collision::BodyNodeCollisionFilter*>(filter);
  if (bodyNodeFilter == nullptr)
    return 0u;

  return bodyNodeFilter->getBodyNodePairBlackListRevision();
}

//==============================================================================
bool World::hasRestingMobileSkeleton() const
{
  for (const auto& skel : mSkeletons) {
    if (skel->isMobile() && skel->isResting())
      return true;
  }

  return false;
}

//==============================================================================
void World::wakeRestingSkeletonsIfStepStateChanged()
{
  if (!mLastStepRestingWorldStateValid || !hasRestingMobileSkeleton())
    return;

  const auto collisionDetector = mConstraintSolver->getCollisionDetector();
  const auto collisionGroup = mConstraintSolver->getCollisionGroup();
  const auto& collisionOption = mConstraintSolver->getCollisionOption();
  const auto* collisionFilter = collisionOption.collisionFilter.get();
  const bool collisionFilterTrackable
      = isCollisionFilterSnapshotTrackable(collisionFilter);
  // Before the no-contact kinematic snapshot exists, this guard is the only
  // place that can notice support pose edits made between the sleep transition
  // and the next step. Once the full snapshot exists, let its exact
  // pose/transform validation distinguish real kinematic edits from visual-only
  // version changes.
  bool skeletonStateUnchanged = true;
  if (!mAllRestingKinematicSnapshotValid) {
    skeletonStateUnchanged
        = mLastStepRestingWorldSkeletonStates.size() == mSkeletons.size();
    if (skeletonStateUnchanged) {
      for (std::size_t i = 0; i < mSkeletons.size(); ++i) {
        const auto& snapshot = mLastStepRestingWorldSkeletonStates[i];
        const auto& skel = mSkeletons[i];
        if (snapshot.mSkeleton != skel.get()
            || snapshot.mStructuralVersion != skel->getVersion()
            || snapshot.mKinematicVersion != skel->getKinematicVersion()
            || snapshot.mNumBodyNodes != skel->getNumBodyNodes()) {
          skeletonStateUnchanged = false;
          break;
        }
      }
    }
  }

  const bool worldStateUnchanged
      = mLastStepRestingWorldStateCollisionFilterTrackable
        && collisionFilterTrackable && skeletonStateUnchanged
        && mLastStepRestingWorldStateDeactivationStateVersion
               == dynamics::Skeleton::getGlobalDeactivationStateVersion()
        && mLastStepRestingWorldStateCollisionDetector
               == collisionDetector.get()
        && mLastStepRestingWorldStateCollisionGroup == collisionGroup.get()
        && mLastStepRestingWorldStateCollisionGroupVersion
               == collisionGroup->getContentVersion()
        && mLastStepRestingWorldStateCollisionEnableContact
               == collisionOption.enableContact
        && mLastStepRestingWorldStateCollisionMaxNumContacts
               == collisionOption.maxNumContacts
        && mLastStepRestingWorldStateCollisionMaxNumContactsPerPair
               == collisionOption.maxNumContactsPerPair
        && mLastStepRestingWorldStateCollisionAllowNegativePenetrationDepthContacts
               == collisionOption.allowNegativePenetrationDepthContacts
        && mLastStepRestingWorldStateCollisionFilter == collisionFilter
        && mLastStepRestingWorldStateCollisionFilterRevision
               == getCollisionFilterSnapshotRevision(collisionFilter);

  if (!worldStateUnchanged)
    wakeRestingSkeletonsForWorldChange();
}

//==============================================================================
void World::updateLastStepRestingWorldState()
{
  if (!hasRestingMobileSkeleton()) {
    invalidateLastStepRestingWorldState();
    return;
  }

  const auto collisionDetector = mConstraintSolver->getCollisionDetector();
  mLastStepRestingWorldStateCollisionDetector = collisionDetector.get();
  const auto collisionGroup = mConstraintSolver->getCollisionGroup();
  mLastStepRestingWorldStateCollisionGroup = collisionGroup.get();
  mLastStepRestingWorldStateCollisionGroupVersion
      = collisionGroup->getContentVersion();
  const auto& collisionOption = mConstraintSolver->getCollisionOption();
  mLastStepRestingWorldStateCollisionEnableContact
      = collisionOption.enableContact;
  mLastStepRestingWorldStateCollisionMaxNumContacts
      = collisionOption.maxNumContacts;
  mLastStepRestingWorldStateCollisionMaxNumContactsPerPair
      = collisionOption.maxNumContactsPerPair;
  mLastStepRestingWorldStateCollisionAllowNegativePenetrationDepthContacts
      = collisionOption.allowNegativePenetrationDepthContacts;
  mLastStepRestingWorldStateCollisionFilter
      = collisionOption.collisionFilter.get();
  mLastStepRestingWorldStateCollisionFilterTrackable
      = isCollisionFilterSnapshotTrackable(
          mLastStepRestingWorldStateCollisionFilter);
  mLastStepRestingWorldStateCollisionFilterRevision
      = mLastStepRestingWorldStateCollisionFilterTrackable
            ? getCollisionFilterSnapshotRevision(
                mLastStepRestingWorldStateCollisionFilter)
            : 0u;
  mLastStepRestingWorldSkeletonStates.clear();
  mLastStepRestingWorldSkeletonStates.reserve(mSkeletons.size());
  for (const auto& skel : mSkeletons) {
    mLastStepRestingWorldSkeletonStates.push_back(RestingWorldSkeletonState{
        skel.get(),
        skel->getVersion(),
        skel->getKinematicVersion(),
        skel->getNumBodyNodes()});
  }
  mLastStepRestingWorldStateDeactivationStateVersion
      = dynamics::Skeleton::getGlobalDeactivationStateVersion();
  mLastStepRestingWorldStateValid = true;
}

//==============================================================================
void World::invalidateLastStepRestingWorldState()
{
  mLastStepRestingWorldStateValid = false;
  mLastStepRestingWorldStateCollisionFilterTrackable = false;
  mLastStepRestingWorldSkeletonStates.clear();
  mLastStepRestingWorldStateDeactivationStateVersion = 0u;
  mLastStepRestingWorldStateCollisionDetector = nullptr;
  mLastStepRestingWorldStateCollisionGroup = nullptr;
  mLastStepRestingWorldStateCollisionGroupVersion = 0u;
  mLastStepRestingWorldStateCollisionEnableContact = false;
  mLastStepRestingWorldStateCollisionMaxNumContacts = 0u;
  mLastStepRestingWorldStateCollisionMaxNumContactsPerPair = 0u;
  mLastStepRestingWorldStateCollisionAllowNegativePenetrationDepthContacts
      = false;
  mLastStepRestingWorldStateCollisionFilter = nullptr;
  mLastStepRestingWorldStateCollisionFilterRevision = 0u;
}

//==============================================================================
void World::invalidateAllRestingKinematicSnapshot()
{
  mAllRestingKinematicSnapshotValid = false;
  mAllRestingSnapshotHasMobileSkeleton = false;
  mAllRestingSnapshotReady = false;
  mAllRestingSnapshotResetCommand = true;
  mAllRestingSnapshotCollisionDetector = nullptr;
  mAllRestingSnapshotCollisionGroup = nullptr;
  mAllRestingSnapshotCollisionGroupVersion = 0u;
  mAllRestingSnapshotCollisionEnableContact = false;
  mAllRestingSnapshotCollisionMaxNumContacts = 0u;
  mAllRestingSnapshotCollisionMaxNumContactsPerPair = 0u;
  mAllRestingSnapshotCollisionAllowNegativePenetrationDepthContacts = false;
  mAllRestingSnapshotCollisionFilter = nullptr;
  mAllRestingSnapshotCollisionFilterRevision = 0u;
  mAllRestingSnapshotVelocityVersion = 0u;
}

//==============================================================================
void World::wakeRestingSkeletonsForWorldChange()
{
  DART_PROFILE_SCOPED_N("World::step - Wake resting after world change");

  for (auto& skel : mSkeletons) {
    if (!skel->isMobile() || !skel->isResting())
      continue;

    skel->setResting(false);
    skel->setSleepCandidate(false);
    skel->setIslandIndex(-1);
    skel->setRestDwellTime(0.0);
  }

  invalidateAllRestingKinematicSnapshot();
  invalidateLastStepRestingWorldState();
}

//==============================================================================
void World::setTime(double _time)
{
  mTime = _time;
}

//==============================================================================
double World::getTime() const
{
  return mTime;
}

//==============================================================================
int World::getSimFrames() const
{
  return mFrame;
}

//==============================================================================
const std::string& World::setName(const std::string& _newName)
{
  if (_newName == mName)
    return mName;

  const std::string oldName = mName;
  mName = _newName;

  mNameChangedSignal.raise(oldName, mName);

  mNameMgrForSkeletons.setManagerName("World::Skeleton | " + mName);
  mNameMgrForSimpleFrames.setManagerName("World::SimpleFrame | " + mName);

  return mName;
}

//==============================================================================
const std::string& World::getName() const
{
  return mName;
}

//==============================================================================
void World::setGravity(const Eigen::Vector3d& _gravity)
{
  mGravity = _gravity;
  for (std::vector<dynamics::SkeletonPtr>::iterator it = mSkeletons.begin();
       it != mSkeletons.end();
       ++it) {
    (*it)->setGravity(_gravity);
  }
  invalidateAllRestingKinematicSnapshot();
  wakeRestingSkeletonsForWorldChange();
}

//==============================================================================
void World::setGravity(double x, double y, double z)
{
  setGravity(Eigen::Vector3d(x, y, z));
}

//==============================================================================
const Eigen::Vector3d& World::getGravity() const
{
  return mGravity;
}

//==============================================================================
dynamics::SkeletonPtr World::getSkeleton(std::size_t _index) const
{
  if (_index < mSkeletons.size())
    return mSkeletons[_index];

  return nullptr;
}

//==============================================================================
dynamics::SkeletonPtr World::getSkeleton(const std::string& _name) const
{
  return mNameMgrForSkeletons.getObject(_name);
}

//==============================================================================
std::size_t World::getNumSkeletons() const
{
  return mSkeletons.size();
}

//==============================================================================
std::string World::addSkeleton(const dynamics::SkeletonPtr& _skeleton)
{
  if (nullptr == _skeleton) {
    dtwarn << "[World::addSkeleton] Attempting to add a nullptr Skeleton to "
           << "the world!\n";
    return "";
  }

  // If mSkeletons already has _skeleton, then we do nothing.
  if (find(mSkeletons.begin(), mSkeletons.end(), _skeleton)
      != mSkeletons.end()) {
    dtwarn << "[World::addSkeleton] Skeleton named [" << _skeleton->getName()
           << "] is already in the world." << std::endl;
    return _skeleton->getName();
  }

  mSkeletons.push_back(_skeleton);
  mMapForSkeletons[_skeleton] = _skeleton;

  mNameConnectionsForSkeletons.push_back(_skeleton->onNameChanged.connect(
      [=](dynamics::ConstMetaSkeletonPtr skel,
          const std::string&,
          const std::string&) { this->handleSkeletonNameChange(skel); }));

  _skeleton->setName(
      mNameMgrForSkeletons.issueNewNameAndAdd(_skeleton->getName(), _skeleton));

  _skeleton->setTimeStep(mTimeStep);
  _skeleton->setGravity(mGravity);

  mIndices.push_back(mIndices.back() + _skeleton->getNumDofs());
  mConstraintSolver->addSkeleton(_skeleton);
  syncShallowSupportFreeRootVelocityStates();
  invalidateAllRestingKinematicSnapshot();
  wakeRestingSkeletonsForWorldChange();
  invalidateSimulationMode();

  // Update recording
  mRecording->updateNumGenCoords(mSkeletons);

  return _skeleton->getName();
}

//==============================================================================
void World::removeSkeleton(const dynamics::SkeletonPtr& _skeleton)
{
  DART_ASSERT(
      _skeleton != nullptr
      && "Attempted to remove nullptr Skeleton from world");

  if (nullptr == _skeleton) {
    dtwarn << "[World::removeSkeleton] Attempting to remove a nullptr Skeleton "
           << "from the world!\n";
    return;
  }

  // Find index of _skeleton in mSkeleton.
  std::size_t index = 0;
  for (; index < mSkeletons.size(); ++index) {
    if (mSkeletons[index] == _skeleton)
      break;
  }

  // If i is equal to the number of skeletons, then _skeleton is not in
  // mSkeleton. We do nothing.
  if (index == mSkeletons.size()) {
    dtwarn << "[World::removeSkeleton] Skeleton [" << _skeleton->getName()
           << "] is not in the world.\n";
    return;
  }

  // Update mIndices.
  for (std::size_t i = index + 1; i < mSkeletons.size() - 1; ++i)
    mIndices[i] = mIndices[i + 1] - _skeleton->getNumDofs();
  mIndices.pop_back();

  // Remove _skeleton from constraint handler.
  mConstraintSolver->removeSkeleton(_skeleton);
  invalidateAllRestingKinematicSnapshot();
  wakeRestingSkeletonsForWorldChange();

  // Remove _skeleton from mSkeletons
  mSkeletons.erase(
      remove(mSkeletons.begin(), mSkeletons.end(), _skeleton),
      mSkeletons.end());
  syncShallowSupportFreeRootVelocityStates();
  invalidateSimulationMode();

  // Disconnect the name change monitor
  mNameConnectionsForSkeletons[index].disconnect();
  mNameConnectionsForSkeletons.erase(
      mNameConnectionsForSkeletons.begin() + index);

  // Update recording
  mRecording->updateNumGenCoords(mSkeletons);

  // Remove from NameManager
  mNameMgrForSkeletons.removeName(_skeleton->getName());

  // Remove from the pointer map
  mMapForSkeletons.erase(_skeleton);
}

//==============================================================================
std::set<dynamics::SkeletonPtr> World::removeAllSkeletons()
{
  std::set<dynamics::SkeletonPtr> ptrs;
  for (std::vector<dynamics::SkeletonPtr>::iterator it = mSkeletons.begin(),
                                                    end = mSkeletons.end();
       it != end;
       ++it)
    ptrs.insert(*it);

  while (getNumSkeletons() > 0)
    removeSkeleton(getSkeleton(0));

  return ptrs;
}

//==============================================================================
bool World::hasSkeleton(const dynamics::ConstSkeletonPtr& skeleton) const
{
  return std::find(mSkeletons.begin(), mSkeletons.end(), skeleton)
         != mSkeletons.end();
}

//==============================================================================
bool World::hasSkeleton(const std::string& skeletonName) const
{
  return mNameMgrForSkeletons.hasName(skeletonName);
}

//==============================================================================
int World::getIndex(int _index) const
{
  if (_index < 0 || static_cast<std::size_t>(_index) >= mIndices.size()) {
    dterr << "[World::getIndex] Index [" << _index << "] is out of range. "
          << "Valid range is [0, " << mIndices.size() << ").\n";
    DART_ASSERT(false);
    return -1;
  }
  return mIndices[_index];
}

//==============================================================================
dynamics::SimpleFramePtr World::getSimpleFrame(std::size_t _index) const
{
  if (_index < mSimpleFrames.size())
    return mSimpleFrames[_index];

  return nullptr;
}

//==============================================================================
dynamics::SimpleFramePtr World::getSimpleFrame(const std::string& _name) const
{
  return mNameMgrForSimpleFrames.getObject(_name);
}

//==============================================================================
std::size_t World::getNumSimpleFrames() const
{
  return mSimpleFrames.size();
}

//==============================================================================
std::string World::addSimpleFrame(const dynamics::SimpleFramePtr& _frame)
{
  DART_ASSERT(
      _frame != nullptr && "Attempted to add nullptr SimpleFrame to world");

  if (nullptr == _frame) {
    dtwarn << "[World::addFrame] Attempting to add a nullptr SimpleFrame to "
              "the world!\n";
    return "";
  }

  if (find(mSimpleFrames.begin(), mSimpleFrames.end(), _frame)
      != mSimpleFrames.end()) {
    dtwarn << "[World::addFrame] SimpleFrame named [" << _frame->getName()
           << "] is already in the world.\n";
    return _frame->getName();
  }

  mSimpleFrames.push_back(_frame);
  mSimpleFrameToShared[_frame.get()] = _frame;

  mNameConnectionsForSimpleFrames.push_back(_frame->onNameChanged.connect(
      [=](const dynamics::Entity* _entity,
          const std::string&,
          const std::string&) { this->handleSimpleFrameNameChange(_entity); }));

  _frame->setName(
      mNameMgrForSimpleFrames.issueNewNameAndAdd(_frame->getName(), _frame));

  invalidateSimulationMode();

  return _frame->getName();
}

//==============================================================================
void World::removeSimpleFrame(const dynamics::SimpleFramePtr& _frame)
{
  DART_ASSERT(
      _frame != nullptr
      && "Attempted to remove nullptr SimpleFrame from world");

  std::vector<dynamics::SimpleFramePtr>::iterator it
      = find(mSimpleFrames.begin(), mSimpleFrames.end(), _frame);

  if (it == mSimpleFrames.end()) {
    dtwarn << "[World::removeFrame] Frame named [" << _frame->getName()
           << "] is not in the world.\n";
    return;
  }

  std::size_t index = it - mSimpleFrames.begin();

  // Remove the frame
  mSimpleFrames.erase(mSimpleFrames.begin() + index);

  // Disconnect the name change monitor
  mNameConnectionsForSimpleFrames[index].disconnect();
  mNameConnectionsForSimpleFrames.erase(
      mNameConnectionsForSimpleFrames.begin() + index);

  // Remove from NameManager
  mNameMgrForSimpleFrames.removeName(_frame->getName());

  // Remove from the pointer map
  mSimpleFrameToShared.erase(_frame.get());

  invalidateSimulationMode();
}

//==============================================================================
std::set<dynamics::SimpleFramePtr> World::removeAllSimpleFrames()
{
  std::set<dynamics::SimpleFramePtr> ptrs;
  for (std::vector<dynamics::SimpleFramePtr>::iterator it
       = mSimpleFrames.begin(),
       end = mSimpleFrames.end();
       it != end;
       ++it)
    ptrs.insert(*it);

  while (getNumSimpleFrames() > 0)
    removeSimpleFrame(getSimpleFrame(0));

  return ptrs;
}

//==============================================================================
bool World::checkCollision(bool checkAllCollisions)
{
  collision::CollisionOption option;

  if (checkAllCollisions)
    option.maxNumContacts = 1e+3;
  else
    option.maxNumContacts = 1u;

  return checkCollision(option);
}

//==============================================================================
bool World::checkCollision(
    const collision::CollisionOption& option,
    collision::CollisionResult* result)
{
  return mConstraintSolver->getCollisionGroup()->collide(option, result);
}

//==============================================================================
const collision::CollisionResult& World::getLastCollisionResult() const
{
  return mConstraintSolver->getLastCollisionResult();
}

//==============================================================================
void World::setCollisionDetector(
    const collision::CollisionDetectorPtr& collisionDetector)
{
  if (!collisionDetector) {
    DART_WARN(
        "Attempted to assign a null collision detector to world '{}'.", mName);
    return;
  }

  mConstraintSolver->setCollisionDetector(collisionDetector);
}

//==============================================================================
void World::setCollisionDetector(CollisionDetectorType collisionDetector)
{
  auto detector = tryCreateCollisionDetector(collisionDetector);
  if (!detector) {
    auto current = mConstraintSolver->getCollisionDetector();
    DART_WARN(
        "Collision detector '{}' is not available for world '{}'. Keeping the "
        "current detector '{}'.",
        toCollisionDetectorKey(collisionDetector),
        mName,
        current ? current->getType() : "unknown");
    return;
  }

  setCollisionDetector(detector);
}

//==============================================================================
collision::CollisionDetectorPtr World::getCollisionDetector()
{
  return mConstraintSolver->getCollisionDetector();
}

//==============================================================================
collision::ConstCollisionDetectorPtr World::getCollisionDetector() const
{
  return mConstraintSolver->getCollisionDetector();
}

//==============================================================================
void World::setConstraintSolver(constraint::UniqueConstraintSolverPtr solver)
{
  if (!solver) {
    dtwarn << "[World::setConstraintSolver] nullptr for constraint solver is "
           << "not allowed. Doing nothing.";
    return;
  }

  if (mConstraintSolver)
    solver->setFromOtherConstraintSolver(*mConstraintSolver);

  mConstraintSolver = std::move(solver);
  mConstraintSolver->setTimeStep(mTimeStep);
  mConstraintSolver->setNumSimulationThreads(mNumSimulationThreads);
  invalidateAllRestingKinematicSnapshot();
  wakeRestingSkeletonsForWorldChange();
  invalidateSimulationMode();
}

//==============================================================================
constraint::ConstraintSolver* World::getConstraintSolver()
{
  return mConstraintSolver.get();
}

//==============================================================================
const constraint::ConstraintSolver* World::getConstraintSolver() const
{
  return mConstraintSolver.get();
}

//==============================================================================
void World::setDeactivationOptions(const DeactivationOptions& options)
{
  mDeactivationOptions = options;
  invalidateAllRestingKinematicSnapshot();
  wakeRestingSkeletonsForWorldChange();

  // If the feature is being disabled, clear any resting/candidacy state so
  // subsequent steps process every skeleton normally and behavior is identical
  // to the feature being absent.
  if (!mDeactivationOptions.mEnabled) {
    for (auto& skel : mSkeletons) {
      skel->setResting(false);
      skel->setSleepCandidate(false);
      skel->setIslandIndex(-1);
    }
  }
}

//==============================================================================
const DeactivationOptions& World::getDeactivationOptions() const
{
  return mDeactivationOptions;
}

//==============================================================================
void World::bake()
{
  const auto collisionResult = getConstraintSolver()->getLastCollisionResult();
  const auto nContacts = static_cast<int>(collisionResult.getNumContacts());
  const auto nSkeletons = getNumSkeletons();

  Eigen::VectorXd state(getIndex(nSkeletons) + 6 * nContacts);
  for (auto i = 0u; i < getNumSkeletons(); ++i) {
    state.segment(getIndex(i), getSkeleton(i)->getNumDofs())
        = getSkeleton(i)->getPositions();
  }

  for (auto i = 0; i < nContacts; ++i) {
    auto begin = getIndex(nSkeletons) + i * 6;
    state.segment(begin, 3) = collisionResult.getContact(i).point;
    state.segment(begin + 3, 3) = collisionResult.getContact(i).force;
  }

  mRecording->addState(state);
}

//==============================================================================
Recording* World::getRecording()
{
  return mRecording;
}

//==============================================================================
void World::handleSkeletonNameChange(
    const dynamics::ConstMetaSkeletonPtr& _skeleton)
{
  if (nullptr == _skeleton) {
    dterr << "[World::handleSkeletonNameChange] Received a name change "
          << "callback for a nullptr Skeleton. This is most likely a bug. "
          << "Please report this!\n";
    DART_ASSERT(false);
    return;
  }

  // Get the new name of the Skeleton
  const std::string& newName = _skeleton->getName();

  // Find the shared version of the Skeleton
  std::map<dynamics::ConstMetaSkeletonPtr, dynamics::SkeletonPtr>::iterator it
      = mMapForSkeletons.find(_skeleton);
  if (it == mMapForSkeletons.end()) {
    dterr << "[World::handleSkeletonNameChange] Could not find Skeleton named ["
          << _skeleton->getName() << "] in the shared_ptr map of World ["
          << getName() << "]. This is most likely a bug. Please report this!\n";
    DART_ASSERT(false);
    return;
  }
  dynamics::SkeletonPtr sharedSkel = it->second;

  // Inform the NameManager of the change
  std::string issuedName
      = mNameMgrForSkeletons.changeObjectName(sharedSkel, newName);

  // If the name issued by the NameManger does not match, reset the name of the
  // Skeleton to match the newly issued name.
  if ((!issuedName.empty()) && (newName != issuedName)) {
    sharedSkel->setName(issuedName);
  } else if (issuedName.empty()) {
    dterr << "[World::handleSkeletonNameChange] Skeleton named ["
          << sharedSkel->getName() << "] (" << sharedSkel << ") does not exist "
          << "in the NameManager of World [" << getName() << "]. This is most "
          << "likely a bug. Please report this!\n";
    DART_ASSERT(false);
    return;
  }
}

//==============================================================================
void World::handleSimpleFrameNameChange(const dynamics::Entity* _entity)
{
  // Check that this is actually a SimpleFrame
  const dynamics::SimpleFrame* frame
      = dynamic_cast<const dynamics::SimpleFrame*>(_entity);

  if (nullptr == frame) {
    dterr << "[World::handleFrameNameChange] Received a callback for a nullptr "
          << "entity. This is most likely a bug. Please report this!\n";
    DART_ASSERT(false);
    return;
  }

  // Get the new name of the Frame
  const std::string& newName = frame->getName();

  // Find the shared version of the Frame
  std::map<const dynamics::SimpleFrame*, dynamics::SimpleFramePtr>::iterator it
      = mSimpleFrameToShared.find(frame);
  if (it == mSimpleFrameToShared.end()) {
    dterr << "[World::handleFrameNameChange] Could not find SimpleFrame named ["
          << frame->getName() << "] in the shared_ptr map of World ["
          << getName() << "]. This is most likely a bug. Please report this!\n";
    DART_ASSERT(false);
    return;
  }
  dynamics::SimpleFramePtr sharedFrame = it->second;

  std::string issuedName
      = mNameMgrForSimpleFrames.changeObjectName(sharedFrame, newName);

  if ((!issuedName.empty()) && (newName != issuedName)) {
    sharedFrame->setName(issuedName);
  } else if (issuedName.empty()) {
    dterr << "[World::handleFrameNameChange] SimpleFrame named ["
          << frame->getName() << "] (" << frame << ") does not exist in the "
          << "NameManager of World [" << getName() << "]. This is most likely "
          << "a bug. Please report this!\n";
    DART_ASSERT(false);
    return;
  }
}

} // namespace simulation
} // namespace dart
