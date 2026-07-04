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

#include "dart/constraint/constraint_solver.hpp"

#include "dart/collision/collision_detector.hpp"
#include "dart/collision/collision_filter.hpp"
#include "dart/collision/collision_group.hpp"
#include "dart/collision/collision_object.hpp"
#include "dart/collision/contact.hpp"
#include "dart/collision/dart/dart_collision_detector.hpp"
#include "dart/common/frame_allocator.hpp"
#include "dart/common/logging.hpp"
#include "dart/common/macros.hpp"
#include "dart/common/profile.hpp"
#include "dart/constraint/constrained_group.hpp"
#include "dart/constraint/contact_constraint.hpp"
#include "dart/constraint/contact_surface.hpp"
#include "dart/constraint/coupler_constraint.hpp"
#include "dart/constraint/joint_constraint.hpp"
#include "dart/constraint/joint_coulomb_friction_constraint.hpp"
#include "dart/constraint/mimic_motor_constraint.hpp"
#include "dart/constraint/soft_contact_constraint.hpp"
#include "dart/dynamics/body_node.hpp"
#include "dart/dynamics/joint.hpp"
#include "dart/dynamics/skeleton.hpp"
#include "dart/dynamics/soft_body_node.hpp"
#include "dart/math/lcp/lcp_utils.hpp"
#include "dart/math/lcp/pivoting/dantzig_solver.hpp"
#include "dart/math/lcp/projection/pgs_solver.hpp"

#include <fmt/ostream.h>

#include <algorithm>
#include <array>
#include <functional>
#include <iterator>
#include <memory>
#include <memory_resource>
#include <ranges>
#include <utility>
#include <vector>

#include <cmath>
#include <cstddef>
#include <cstdint>

namespace dart {
namespace constraint {

using namespace dynamics;

//==============================================================================
namespace {

collision::CollisionDetectorPtr createDefaultCollisionDetector()
{
  auto* factory = collision::CollisionDetector::getFactory();
  if (factory->canCreate("dart")) {
    return factory->create("dart");
  }
  if (factory->canCreate("experimental")) {
    return factory->create("experimental");
  }
  if (factory->canCreate("fcl")) {
    return factory->create("fcl");
  }
  return collision::DartCollisionDetector::create();
}

constexpr std::size_t kContactScratchBufferBytes = 64 * 1024;

struct ContactPairKey
{
  collision::CollisionObject* first = nullptr;
  collision::CollisionObject* second = nullptr;
};

ContactPairKey makeContactPairKey(
    collision::CollisionObject* first, collision::CollisionObject* second)
{
  if (std::less<collision::CollisionObject*>{}(second, first)) {
    std::swap(first, second);
  }
  return {first, second};
}

bool sameContactPair(ContactPairKey lhs, ContactPairKey rhs)
{
  return lhs.first == rhs.first && lhs.second == rhs.second;
}

struct ContactPairCount
{
  ContactPairKey key;
  std::size_t count = 0;
};

class ContactPairCounter
{
public:
  ContactPairCounter(
      std::size_t maxContacts, std::pmr::memory_resource& memoryResource)
    : mEntries(&memoryResource)
  {
    if (maxContacts == 0) {
      return;
    }

    std::size_t capacity = 1;
    while (capacity < maxContacts * 2) {
      capacity <<= 1;
    }

    mEntries.resize(capacity);
    mMask = capacity - 1;
  }

  void increment(
      collision::CollisionObject* first, collision::CollisionObject* second)
  {
    ++findOrInsert(makeContactPairKey(first, second)).count;
  }

  std::size_t count(
      collision::CollisionObject* first,
      collision::CollisionObject* second) const
  {
    if (mEntries.empty()) {
      return 1;
    }

    const auto key = makeContactPairKey(first, second);
    std::size_t index = hash(key) & mMask;
    while (true) {
      const auto& entry = mEntries[index];
      if (entry.count == 0) {
        return 1;
      }
      if (sameContactPair(entry.key, key)) {
        return entry.count;
      }
      index = (index + 1) & mMask;
    }
  }

private:
  ContactPairCount& findOrInsert(ContactPairKey key)
  {
    std::size_t index = hash(key) & mMask;
    while (true) {
      auto& entry = mEntries[index];
      if (entry.count == 0) {
        entry.key = key;
        return entry;
      }
      if (sameContactPair(entry.key, key)) {
        return entry;
      }
      index = (index + 1) & mMask;
    }
  }

  static std::size_t hash(ContactPairKey key)
  {
    auto first = reinterpret_cast<std::uintptr_t>(key.first) >> 4;
    auto second = reinterpret_cast<std::uintptr_t>(key.second) >> 4;
    return static_cast<std::size_t>(
        first ^ (second + 0x9e3779b97f4a7c15ULL + (first << 6) + (first >> 2)));
  }

  std::pmr::vector<ContactPairCount> mEntries;
  std::size_t mMask = 0;
};

} // namespace

ConstraintSolver::ConstraintSolver()
  : mCollisionDetector(createDefaultCollisionDetector()),
    mCollisionGroup(
        mCollisionDetector
            ? mCollisionDetector->createCollisionGroupAsSharedPtr()
            : nullptr),
    mCollisionOption(
        collision::CollisionOption(
            true,
            1000u,
            std::make_shared<collision::BodyNodeCollisionFilter>())),
    mTimeStep(0.001),
    mContactSurfaceHandler(std::make_shared<DefaultContactSurfaceHandler>()),
    mLcpSolver(std::make_shared<math::DantzigSolver>()),
    mSecondaryLcpSolver(std::make_shared<math::PgsSolver>()),
    mOwnedFrameAllocator(std::make_unique<common::FrameAllocator>()),
    mFrameAllocator(mOwnedFrameAllocator.get())
{
}

//==============================================================================
ConstraintSolver::ConstraintSolver(math::LcpSolverPtr primary)
  : ConstraintSolver()
{
  if (primary) {
    setLcpSolver(std::move(primary));
  }
}

//==============================================================================
ConstraintSolver::ConstraintSolver(
    math::LcpSolverPtr primary, math::LcpSolverPtr secondary)
  : ConstraintSolver(std::move(primary))
{
  setSecondaryLcpSolver(std::move(secondary));
}

//==============================================================================
ConstraintSolver::~ConstraintSolver()
{
  mActiveConstraints.clear();
  mContactConstraints.clear();
  mSoftContactConstraints.clear();
  mJointConstraints.clear();
  mMimicMotorConstraints.clear();
  mCouplerConstraints.clear();
  mJointCoulombFrictionConstraints.clear();
  mConstrainedGroups.clear();
  mPositionConstraints.clear();
}

//==============================================================================
void ConstraintSolver::addSkeleton(const SkeletonPtr& skeleton)
{
  DART_ASSERT(
      skeleton
      && "Null pointer skeleton is now allowed to add to ConstraintSover.");

  if (hasSkeleton(skeleton)) {
    DART_WARN(
        "Attempting to add skeleton '{}', which already exists in the "
        "ConstraintSolver.",
        skeleton->getName());

    return;
  }

  mCollisionGroup->subscribeTo(skeleton);
  mSkeletons.push_back(skeleton);
  mConstrainedGroups.reserve(mSkeletons.size());
}

//==============================================================================
void ConstraintSolver::addSkeletons(std::span<const SkeletonPtr> skeletons)
{
  for (const auto& skeleton : skeletons) {
    addSkeleton(skeleton);
  }
}

//==============================================================================
std::span<const SkeletonPtr> ConstraintSolver::getSkeletons() const
{
  return std::span<const SkeletonPtr>(mSkeletons);
}

//==============================================================================
void ConstraintSolver::removeSkeleton(const SkeletonPtr& skeleton)
{
  DART_ASSERT(
      skeleton
      && "Null pointer skeleton is now allowed to add to ConstraintSover.");

  DART_WARN_IF(
      !hasSkeleton(skeleton),
      "Attempting to remove skeleton '{}', which doesn't exist in the "
      "ConstraintSolver.",
      skeleton->getName());

  mCollisionGroup->unsubscribeFrom(skeleton.get());
  std::erase(mSkeletons, skeleton);
  mConstrainedGroups.reserve(mSkeletons.size());
}

//==============================================================================
void ConstraintSolver::removeSkeletons(std::span<const SkeletonPtr> skeletons)
{
  for (const auto& skeleton : skeletons) {
    removeSkeleton(skeleton);
  }
}

//==============================================================================
void ConstraintSolver::removeAllSkeletons()
{
  mCollisionGroup->removeAllShapeFrames();
  mSkeletons.clear();
}

//==============================================================================
void ConstraintSolver::addConstraint(const ConstraintBasePtr& constraint)
{
  DART_ASSERT(constraint);

  if (containConstraint(constraint)) {
    DART_WARN(
        "Constraint solver already contains constraint that you are trying to "
        "add.");
    return;
  }

  mManualConstraints.push_back(constraint);
}

//==============================================================================
void ConstraintSolver::removeConstraint(const ConstraintBasePtr& constraint)
{
  DART_ASSERT(constraint);

  if (!containConstraint(constraint)) {
    DART_WARN(
        "Constraint solver deos not contain constraint that you are trying to "
        "remove.");
    return;
  }

  std::erase(mManualConstraints, constraint);
}

//==============================================================================
void ConstraintSolver::removeAllConstraints()
{
  mManualConstraints.clear();
}

//==============================================================================
std::size_t ConstraintSolver::getNumConstraints() const
{
  return mManualConstraints.size();
}

//==============================================================================
ConstraintBasePtr ConstraintSolver::getConstraint(std::size_t index)
{
  return mManualConstraints[index];
}

//==============================================================================
ConstConstraintBasePtr ConstraintSolver::getConstraint(std::size_t index) const
{
  return mManualConstraints[index];
}

//==============================================================================
void ConstraintSolver::clearLastCollisionResult()
{
  mCollisionResult.clear();
}

//==============================================================================
void ConstraintSolver::setTimeStep(double _timeStep)
{
  DART_ASSERT(_timeStep > 0.0 && "Time step should be positive value.");
  mTimeStep = _timeStep;
}

//==============================================================================
double ConstraintSolver::getTimeStep() const
{
  return mTimeStep;
}

//==============================================================================
void ConstraintSolver::setFrameAllocator(common::FrameAllocator* alloc)
{
  mFrameAllocator = alloc ? alloc : mOwnedFrameAllocator.get();
}

void ConstraintSolver::setCollisionDetector(
    const std::shared_ptr<collision::CollisionDetector>& collisionDetector)
{
  if (!collisionDetector) {
    DART_WARN(
        "Attempting to assign nullptr as the new collision detector to the "
        "constraint solver, which is not allowed. Ignoring.");
    return;
  }

  if (mCollisionDetector == collisionDetector) {
    return;
  }

  mCollisionDetector = collisionDetector;

  mCollisionGroup = mCollisionDetector->createCollisionGroupAsSharedPtr();

  for (const auto& skeleton : mSkeletons) {
    mCollisionGroup->addShapeFramesOf(skeleton.get());
  }
}

//==============================================================================
collision::CollisionDetectorPtr ConstraintSolver::getCollisionDetector()
{
  return mCollisionDetector;
}

//==============================================================================
collision::ConstCollisionDetectorPtr ConstraintSolver::getCollisionDetector()
    const
{
  return mCollisionDetector;
}

//==============================================================================
collision::CollisionGroupPtr ConstraintSolver::getCollisionGroup()
{
  return mCollisionGroup;
}

//==============================================================================
collision::ConstCollisionGroupPtr ConstraintSolver::getCollisionGroup() const
{
  return mCollisionGroup;
}

//==============================================================================
collision::CollisionOption& ConstraintSolver::getCollisionOption()
{
  return mCollisionOption;
}

//==============================================================================
const collision::CollisionOption& ConstraintSolver::getCollisionOption() const
{
  return mCollisionOption;
}

//==============================================================================
collision::CollisionResult& ConstraintSolver::getLastCollisionResult()
{
  return mCollisionResult;
}

//==============================================================================
const collision::CollisionResult& ConstraintSolver::getLastCollisionResult()
    const
{
  return mCollisionResult;
}

//==============================================================================
void ConstraintSolver::solve()
{
  DART_PROFILE_SCOPED_N("ConstraintSolver::solve");

  for (auto& skeleton : mSkeletons) {
    skeleton->clearConstraintImpulses();
    skeleton->clearPositionConstraintImpulses();
    skeleton->clearPositionVelocityChanges();
    skeleton->setPositionImpulseApplied(false);
  }

  // Update constraints and collect active constraints
  updateConstraints();

  // Build constrained groups
  buildConstrainedGroups();

  // Solve constrained groups
  solveConstrainedGroups();

  if (mSplitImpulseEnabled) {
    solvePositionConstrainedGroups();
  }
}

//==============================================================================
void ConstraintSolver::setFromOtherConstraintSolver(
    const ConstraintSolver& other)
{
  removeAllSkeletons();
  mManualConstraints.clear();

  addSkeletons(other.getSkeletons());
  mManualConstraints = other.mManualConstraints;

  mContactSurfaceHandler = other.mContactSurfaceHandler;
  if (!mLcpSolverSetExplicitly) {
    mLcpSolver = other.mLcpSolver;
  }
  if (!mSecondaryLcpSolverSetExplicitly) {
    mSecondaryLcpSolver = other.mSecondaryLcpSolver;
  }
  mSplitImpulseEnabled = other.mSplitImpulseEnabled;
}

//==============================================================================
bool ConstraintSolver::hasSkeleton(const ConstSkeletonPtr& skeleton) const
{
#if _WIN32
  DART_ASSERT(
      skeleton != nullptr && "Not allowed to insert null pointer skeleton.");
#else
  DART_ASSERT(
      skeleton != nullptr, "Not allowed to insert null pointer skeleton.");
#endif

  return std::ranges::any_of(mSkeletons, [&](const SkeletonPtr& itrSkel) {
    return itrSkel == skeleton;
  });
}

//==============================================================================
bool ConstraintSolver::checkAndAddSkeleton(const SkeletonPtr& skeleton)
{
  if (!hasSkeleton(skeleton)) {
    mSkeletons.push_back(skeleton);
    return true;
  } else {
    DART_WARN(
        "Skeleton [{}] is already in ConstraintSolver.", skeleton->getName());
    return false;
  }
}

//==============================================================================
bool ConstraintSolver::containConstraint(
    const ConstConstraintBasePtr& constraint) const
{
  return std::ranges::find(mManualConstraints, constraint)
         != mManualConstraints.end();
}

//==============================================================================
bool ConstraintSolver::checkAndAddConstraint(
    const ConstraintBasePtr& constraint)
{
  if (!containConstraint(constraint)) {
    mManualConstraints.push_back(constraint);
    return true;
  } else {
    DART_WARN("Constraint is already in ConstraintSolver.");
    return false;
  }
}

//==============================================================================
void ConstraintSolver::updateConstraints()
{
  DART_PROFILE_SCOPED;

  // Clear previous active constraint list
  mActiveConstraints.clear();
  mConstrainedGroups.clear();
  mPositionConstraints.clear();

  // Destroy previous automatic constraints
  mContactConstraints.clear();
  mSoftContactConstraints.clear();
  mJointConstraints.clear();
  mMimicMotorConstraints.clear();
  mCouplerConstraints.clear();
  mJointCoulombFrictionConstraints.clear();

  // Reset the per-step arena after releasing all per-step constraints
  mFrameAllocator->reset();

  //----------------------------------------------------------------------------
  // Update manual constraints
  //----------------------------------------------------------------------------
  for (auto& manualConstraint : mManualConstraints) {
    manualConstraint->update();

    if (manualConstraint->isActive()) {
      mActiveConstraints.push_back(manualConstraint);
    }
  }

  //----------------------------------------------------------------------------
  // Update automatic constraints: contact constraints
  //----------------------------------------------------------------------------
  mCollisionResult.clear();

  mCollisionGroup->collide(mCollisionOption, &mCollisionResult);

  alignas(std::max_align_t) std::array<std::byte, kContactScratchBufferBytes>
      contactScratchBuffer{};
  std::pmr::monotonic_buffer_resource contactScratchResource(
      contactScratchBuffer.data(),
      contactScratchBuffer.size(),
      std::pmr::new_delete_resource());
  ContactPairCounter contactPairCounter(
      mCollisionResult.getNumContacts(), contactScratchResource);
  std::pmr::vector<collision::Contact*> contacts{&contactScratchResource};
  contacts.reserve(mCollisionResult.getNumContacts());

  // Create new contact constraints
  for (auto i = 0u; i < mCollisionResult.getNumContacts(); ++i) {
    auto& contact = mCollisionResult.getContact(i);

    // Skip contacts with non-finite geometry. A collision shape with an invalid
    // (infinite or NaN) dimension, a malformed mesh, or a third-party collision
    // backend can report a contact whose point, normal, or penetration depth is
    // not finite. Such a contact would otherwise inject NaN/Inf into the
    // contact constraint Jacobians, corrupting the LCP solve in release builds
    // and tripping an assertion in ContactConstraint in debug builds. See
    // gz-physics issue #1010.
    if (!contact.point.allFinite() || !contact.normal.allFinite()
        || !std::isfinite(contact.penetrationDepth)) {
      DART_WARN(
          "[ConstraintSolver] Ignoring contact with non-finite geometry "
          "(point, normal, or penetration depth). This usually indicates a "
          "malformed collision mesh or a collision backend that produced an "
          "invalid contact.");
      continue;
    }

    if (collision::Contact::isZeroNormal(contact.normal)) {
      // Skip this contact. This is because we assume that a contact with
      // zero-length normal is invalid.
      continue;
    }

    const auto bodyNode1 = contact.getBodyNodePtr1();
    const auto bodyNode2 = contact.getBodyNodePtr2();
    if (bodyNode1 == nullptr || bodyNode2 == nullptr) {
      DART_WARN(
          "[ConstraintSolver] Ignoring contact with a null collision object or "
          "missing BodyNode.");
      continue;
    }

    // If penetration depth is negative, then the collision isn't really
    // happening and the contact point should be ignored.
    // TODO(MXG): Investigate ways to leverage the proximity information of a
    //            negative penetration to improve collision handling.
    if (contact.penetrationDepth < 0.0) {
      continue;
    }

    if (isSoftContact(contact)) {
      mSoftContactConstraints.push_back(
          std::allocate_shared<SoftContactConstraint>(
              common::FrameStlAllocator<SoftContactConstraint>(
                  *mFrameAllocator),
              contact,
              mTimeStep));
    } else {
      contactPairCounter.increment(
          contact.collisionObject1, contact.collisionObject2);

      contacts.push_back(&contact);
    }
  }

  // Add the new contact constraints to dynamic constraint list
  for (auto* contact : contacts) {
    const auto numContacts = contactPairCounter.count(
        contact->collisionObject1, contact->collisionObject2);

    auto contactConstraint = mContactSurfaceHandler->createConstraint(
        *contact, numContacts, mTimeStep);
    if (contactConstraint == nullptr) {
      continue;
    }

    mContactConstraints.push_back(contactConstraint);

    contactConstraint->update();

    if (contactConstraint->isActive()) {
      mActiveConstraints.push_back(contactConstraint);
    }
  }

  // Add the new soft contact constraints to dynamic constraint list
  for (const auto& softContactConstraint : mSoftContactConstraints) {
    softContactConstraint->update();

    if (softContactConstraint->isActive()) {
      mActiveConstraints.push_back(softContactConstraint);
    }
  }

  //----------------------------------------------------------------------------
  // Update automatic constraints: joint constraints
  //----------------------------------------------------------------------------
  // Create new joint constraints
  for (const auto& skel : mSkeletons) {
    const std::size_t numJoints = skel->getNumJoints();
    for (std::size_t i = 0; i < numJoints; i++) {
      dynamics::Joint* joint = skel->getJoint(i);

      if (joint->isKinematic()) {
        continue;
      }

      const std::size_t dof = joint->getNumDofs();
      const bool hasCoulombFriction = std::ranges::any_of(
          std::views::iota(std::size_t{0}, dof),
          [&](std::size_t j) { return joint->getCoulombFriction(j) != 0.0; });
      if (hasCoulombFriction) {
        mJointCoulombFrictionConstraints.push_back(
            std::allocate_shared<JointCoulombFrictionConstraint>(
                common::FrameStlAllocator<JointCoulombFrictionConstraint>(
                    *mFrameAllocator),
                joint));
      }

      if (joint->areLimitsEnforced()
          || joint->hasActuatorType(dynamics::Joint::SERVO)) {
        mJointConstraints.push_back(
            std::allocate_shared<JointConstraint>(
                common::FrameStlAllocator<JointConstraint>(*mFrameAllocator),
                joint));
      }

      if (joint->hasActuatorType(dynamics::Joint::MIMIC)) {
        const auto mimicProps = joint->getMimicDofProperties();
        const auto dofCount = joint->getNumDofs();
        const dynamics::MimicDofProperties defaultProp{};
        bool hasValidMimicDof = false;
        bool useCouplerConstraint = false;
        bool allMimicWithReference = true;
        for (std::size_t dofIndex = 0; dofIndex < dofCount; ++dofIndex) {
          const auto& mimicProp = dofIndex < mimicProps.size()
                                      ? mimicProps[dofIndex]
                                      : defaultProp;
          if (joint->getActuatorType(dofIndex) == dynamics::Joint::MIMIC) {
            if (mimicProp.mReferenceJoint != nullptr) {
              hasValidMimicDof = true;
              if (mimicProp.mConstraintType
                  == dynamics::MimicConstraintType::Coupler) {
                useCouplerConstraint = true;
              }
            } else {
              allMimicWithReference = false;
              DART_WARN(
                  "Joint '{}' DoF {} is set to MIMIC without a reference; "
                  "mimic constraint will be skipped.",
                  joint->getName(),
                  dofIndex);
            }
          } else {
            allMimicWithReference = false;
          }
        }

        if (hasValidMimicDof) {
          if (useCouplerConstraint && allMimicWithReference) {
            mCouplerConstraints.push_back(
                std::allocate_shared<CouplerConstraint>(
                    common::FrameStlAllocator<CouplerConstraint>(
                        *mFrameAllocator),
                    joint,
                    joint->getMimicDofProperties()));
          } else {
            mMimicMotorConstraints.push_back(
                std::allocate_shared<MimicMotorConstraint>(
                    common::FrameStlAllocator<MimicMotorConstraint>(
                        *mFrameAllocator),
                    joint,
                    joint->getMimicDofProperties()));
          }
        }
      }
    }
  }

  // Add active joint limit
  for (auto& jointLimitConstraint : mJointConstraints) {
    jointLimitConstraint->update();

    if (jointLimitConstraint->isActive()) {
      mActiveConstraints.push_back(jointLimitConstraint);
    }
  }

  for (auto& mimicMotorConstraint : mMimicMotorConstraints) {
    mimicMotorConstraint->update();

    if (mimicMotorConstraint->isActive()) {
      mActiveConstraints.push_back(mimicMotorConstraint);
    }
  }

  for (auto& couplerConstraint : mCouplerConstraints) {
    couplerConstraint->update();

    if (couplerConstraint->isActive()) {
      mActiveConstraints.push_back(couplerConstraint);
    }
  }

  for (auto& jointFrictionConstraint : mJointCoulombFrictionConstraints) {
    jointFrictionConstraint->update();

    if (jointFrictionConstraint->isActive()) {
      mActiveConstraints.push_back(jointFrictionConstraint);
    }
  }
}

//==============================================================================
void ConstraintSolver::buildConstrainedGroups()
{
  DART_PROFILE_SCOPED;

  // Clear constrained groups
  mConstrainedGroups.clear();

  // Exit if there is no active constraint
  if (mActiveConstraints.empty()) {
    return;
  }

  //----------------------------------------------------------------------------
  // Unite skeletons according to constraints's relationships
  //----------------------------------------------------------------------------
  for (const auto& activeConstraint : mActiveConstraints) {
    activeConstraint->uniteSkeletons();
  }

  //----------------------------------------------------------------------------
  // Build constraint groups
  //----------------------------------------------------------------------------
  for (const auto& activeConstraint : mActiveConstraints) {
    const auto& skel = activeConstraint->getRootSkeleton();
    const bool found = std::ranges::any_of(
        mConstrainedGroups,
        [&](const auto& group) { return group.mRootSkeleton == skel; });

    if (found) {
      continue;
    }

    ConstrainedGroup newConstGroup;
    newConstGroup.mRootSkeleton = skel;
    skel->mUnionIndex = mConstrainedGroups.size();
    mConstrainedGroups.push_back(newConstGroup);
  }

  // Add active constraints to constrained groups
  for (const auto& activeConstraint : mActiveConstraints) {
    const auto& skel = activeConstraint->getRootSkeleton();
    mConstrainedGroups[skel->mUnionIndex].addConstraint(activeConstraint);
  }

  //----------------------------------------------------------------------------
  // Reset union since we don't need union information anymore.
  //----------------------------------------------------------------------------
  for (auto& skeleton : mSkeletons) {
    skeleton->resetUnion();
  }
}

//==============================================================================
void ConstraintSolver::solveConstrainedGroups()
{
  DART_PROFILE_SCOPED;

  for (auto& constraintGroup : mConstrainedGroups) {
    solveConstrainedGroup(constraintGroup);
  }
}

//==============================================================================
void ConstraintSolver::solvePositionConstrainedGroups()
{
  DART_PROFILE_SCOPED;

  // Preserve velocity-impulse flags across the position pass.
  mImpulseAppliedStates.clear();
  mImpulseAppliedStates.reserve(mSkeletons.size());
  std::ranges::copy(
      mSkeletons | std::views::transform([](const auto& skeleton) {
        return skeleton->isImpulseApplied();
      }),
      std::back_inserter(mImpulseAppliedStates));

  for (auto& constraintGroup : mConstrainedGroups) {
    mPositionConstraints.clear();
    mPositionConstraints.reserve(constraintGroup.getNumConstraints());
    for (const auto i : std::views::iota(
             std::size_t{0}, constraintGroup.getNumConstraints())) {
      const ConstraintBasePtr& constraint = constraintGroup.getConstraint(i);
      if (std::dynamic_pointer_cast<ContactConstraint>(constraint)) {
        mPositionConstraints.push_back(constraint);
      }
    }

    solveConstrainedGroupInternal(
        mPositionConstraints, ConstraintPhase::Position);
  }

  for (auto& skeleton : mSkeletons) {
    if (skeleton->isPositionImpulseApplied()) {
      skeleton->computePositionVelocityChanges();
    }
  }

  for (const auto i : std::views::iota(std::size_t{0}, mSkeletons.size())) {
    mSkeletons[i]->setImpulseApplied(mImpulseAppliedStates[i]);
  }
}

//==============================================================================
bool ConstraintSolver::isSoftContact(const collision::Contact& contact) const
{
  const auto bodyNode1 = contact.getBodyNodePtr1();
  const auto bodyNode2 = contact.getBodyNodePtr2();
  DART_ASSERT(bodyNode1);
  DART_ASSERT(bodyNode2);

  const auto bodyNode1IsSoft
      = dynamic_cast<const dynamics::SoftBodyNode*>(bodyNode1.get()) != nullptr;

  const auto bodyNode2IsSoft
      = dynamic_cast<const dynamics::SoftBodyNode*>(bodyNode2.get()) != nullptr;

  return bodyNode1IsSoft || bodyNode2IsSoft;
}

//==============================================================================
ContactSurfaceHandlerPtr ConstraintSolver::getLastContactSurfaceHandler() const
{
  return mContactSurfaceHandler;
}

//==============================================================================
void ConstraintSolver::addContactSurfaceHandler(
    ContactSurfaceHandlerPtr handler)
{
  // sanity check, do not add the same handler twice
  if (handler == mContactSurfaceHandler) {
    DART_ERROR(
        "Adding the same contact surface handler for the second time, "
        "ignoring.");
    return;
  }
  handler->setParent(mContactSurfaceHandler);
  mContactSurfaceHandler = std::move(handler);
}

//==============================================================================
bool ConstraintSolver::removeContactSurfaceHandler(
    const ContactSurfaceHandlerPtr& handler)
{
  bool found = false;
  ContactSurfaceHandlerPtr current = mContactSurfaceHandler;
  ContactSurfaceHandlerPtr previous = nullptr;
  while (current != nullptr) {
    if (current == handler) {
      if (previous != nullptr) {
        previous->mParent = current->mParent;
      } else {
        mContactSurfaceHandler = current->mParent;
      }
      found = true;
      break;
    }
    previous = current;
    current = current->mParent;
  }

  DART_ERROR_IF(
      mContactSurfaceHandler == nullptr,
      "No contact surface handler remained. This is an error. Add at least "
      "DefaultContactSurfaceHandler.");

  return found;
}

//==============================================================================
void ConstraintSolver::setLcpSolver(math::LcpSolverPtr lcpSolver)
{
  if (!lcpSolver) {
    DART_WARN("nullptr for LCP solver is not allowed.");
    return;
  }

  DART_WARN_IF(
      lcpSolver == mSecondaryLcpSolver,
      "Attempting to set a primary LCP solver that is the same as the "
      "secondary LCP solver, which is discouraged. Ignoring this request.");

  mLcpSolver = std::move(lcpSolver);
  mLcpSolverSetExplicitly = true;
}

//==============================================================================
math::LcpSolverPtr ConstraintSolver::getLcpSolver() const
{
  return mLcpSolver;
}

//==============================================================================
void ConstraintSolver::setSecondaryLcpSolver(math::LcpSolverPtr lcpSolver)
{
  DART_WARN_IF(
      lcpSolver == mLcpSolver,
      "Attempting to set the secondary LCP solver that is identical to the "
      "primary LCP solver, which is redundant. Please use different solvers or "
      "set the secondary LCP solver to nullptr.");

  mSecondaryLcpSolver = std::move(lcpSolver);
  mSecondaryLcpSolverSetExplicitly = true;
}

//==============================================================================
math::LcpSolverPtr ConstraintSolver::getSecondaryLcpSolver() const
{
  return mSecondaryLcpSolver;
}

//==============================================================================
void ConstraintSolver::setSplitImpulseEnabled(bool enabled)
{
  mSplitImpulseEnabled = enabled;
}

//==============================================================================
bool ConstraintSolver::isSplitImpulseEnabled() const
{
  return mSplitImpulseEnabled;
}

//==============================================================================
void ConstraintSolver::solveConstrainedGroupInternal(
    std::span<const ConstraintBasePtr> constraints, ConstraintPhase phase)
{
  DART_PROFILE_SCOPED;

  const std::size_t numConstraints = constraints.size();
  std::size_t n = 0;
  for (const auto& constraint : constraints) {
    n += constraint->getDimension();
  }

  if (0u == n) {
    return;
  }

  const int nSkip = math::padding(n);
#if defined(NDEBUG)
  mA.resize(n, nSkip);
#else
  mA.setZero(n, nSkip);
#endif
  mX.resize(n);
  mX.setZero();
  mB.resize(n);
  mW.setZero(n);
  mLo.resize(n);
  mHi.resize(n);
  mFIndex.setConstant(n, -1);

  mOffset.resize(numConstraints);
  mOffset[0] = 0;
  for (std::size_t i = 1; i < numConstraints; ++i) {
    const ConstraintBasePtr& constraint = constraints[i - 1];
    DART_ASSERT(constraint->getDimension() > 0);
    mOffset[i] = mOffset[i - 1] + constraint->getDimension();
  }

  {
    DART_PROFILE_SCOPED_N("Construct LCP");
    ConstraintInfo constInfo;
    constInfo.invTimeStep = 1.0 / mTimeStep;
    constInfo.phase = phase;
    constInfo.useSplitImpulse
        = (phase == ConstraintPhase::Velocity && mSplitImpulseEnabled);
    for (std::size_t i = 0; i < numConstraints; ++i) {
      const ConstraintBasePtr& constraint = constraints[i];

      constInfo.x = mX.data() + mOffset[i];
      constInfo.lo = mLo.data() + mOffset[i];
      constInfo.hi = mHi.data() + mOffset[i];
      constInfo.b = mB.data() + mOffset[i];
      constInfo.findex = mFIndex.data() + mOffset[i];
      constInfo.w = mW.data() + mOffset[i];

      {
        DART_PROFILE_SCOPED_N("Fill lo, hi, b, w");
        constraint->getInformation(&constInfo);
      }

      {
        DART_PROFILE_SCOPED_N("Fill A");
        constraint->excite();
        for (std::size_t j = 0; j < constraint->getDimension(); ++j) {
          if (mFIndex[mOffset[i] + j] >= 0) {
            mFIndex[mOffset[i] + j] += mOffset[i];
          }

          {
            DART_PROFILE_SCOPED_N("Unit impulse test");
            constraint->applyUnitImpulse(j);
          }

          {
            DART_PROFILE_SCOPED_N("Fill upper triangle of A");
            int index = nSkip * (mOffset[i] + j) + mOffset[i];
            constraint->getVelocityChange(mA.data() + index, true);
            for (std::size_t k = i + 1; k < numConstraints; ++k) {
              index = nSkip * (mOffset[i] + j) + mOffset[k];
              constraints[k]->getVelocityChange(mA.data() + index, false);
            }
          }
        }
      }

      {
        DART_PROFILE_SCOPED_N("Unexcite");
        constraint->unexcite();
      }
    }

    {
      DART_PROFILE_SCOPED_N("Fill lower triangle of A");
      mA.leftCols(n).triangularView<Eigen::Lower>()
          = mA.leftCols(n).triangularView<Eigen::Upper>().transpose();
    }
  }

#if !defined(NDEBUG)
  if (!isSymmetric(n, mA.data())) {
    DART_WARN(
        "[ConstraintSolver::solveConstrainedGroup] LCP matrix is not "
        "symmetric. "
        "Continuing to avoid assertion failure.");
  }
#endif

  if (mSecondaryLcpSolver) {
    mABackup = mA;
    mXBackup = mX;
    mBBackup = mB;
    mLoBackup = mLo;
    mHiBackup = mHi;
    mFIndexBackup = mFIndex;
  }
  DART_ASSERT(mLcpSolver);
  math::LcpOptions primaryOptions = mLcpSolver->getDefaultOptions();
  if (mSecondaryLcpSolver) {
    primaryOptions.earlyTermination = true;
  }
  // Preserve legacy behavior to avoid unnecessary fallback on strict checks.
  primaryOptions.validateSolution = false;
  Eigen::MatrixXd Ablock = mA.leftCols(static_cast<Eigen::Index>(n)).eval();
  math::LcpProblem problem(Ablock, mB, mLo, mHi, mFIndex);
  math::LcpResult primaryResult
      = mLcpSolver->solve(problem, mX, primaryOptions);
  constexpr double kImpulseClamp = 1e12;
  auto isUsable = [&](const Eigen::VectorXd& x) {
    return x.allFinite() && x.array().abs().maxCoeff() < kImpulseClamp;
  };

  bool success = primaryResult.succeeded() && isUsable(mX);

  bool fallbackSuccess = false;
  bool fallbackRan = false;
  bool fallbackUsable = false;
  if (!success && mSecondaryLcpSolver) {
    DART_PROFILE_SCOPED_N("Secondary LCP");
    math::LcpOptions fallbackOptions = mSecondaryLcpSolver->getDefaultOptions();
    fallbackOptions.earlyTermination = false;
    fallbackOptions.validateSolution = false;
    Eigen::MatrixXd Abackup
        = mABackup.leftCols(static_cast<Eigen::Index>(n)).eval();
    math::LcpProblem fallbackProblem(
        Abackup, mBBackup, mLoBackup, mHiBackup, mFIndexBackup);
    math::LcpResult fallbackResult = mSecondaryLcpSolver->solve(
        fallbackProblem, mXBackup, fallbackOptions);
    fallbackSuccess = fallbackResult.succeeded();
    fallbackUsable = mXBackup.allFinite()
                     && mXBackup.array().abs().maxCoeff() < kImpulseClamp;
    mX = mXBackup;
    fallbackRan = true;
    if (!fallbackSuccess && fallbackUsable) {
      const double absTol = (fallbackOptions.absoluteTolerance > 0.0)
                                ? fallbackOptions.absoluteTolerance
                                : 1e-6;
      const double residualThreshold = 10.0 * absTol;
      const bool residualFinite = std::isfinite(fallbackResult.residual);
      const bool warn
          = !residualFinite || fallbackResult.residual > residualThreshold;
      if (warn) {
        DART_WARN(
            "[ConstraintSolver] Secondary LCP fallback status={}, "
            "iters={}/{}, res={:.3g} (thr={:.3g}), comp={:.3g}. "
            "Using best-effort solution.",
            math::toString(fallbackResult.status),
            fallbackResult.iterations,
            fallbackOptions.maxIterations,
            fallbackResult.residual,
            residualThreshold,
            fallbackResult.complementarity);
      } else {
        DART_DEBUG(
            "[ConstraintSolver] Secondary LCP fallback status={}, "
            "iters={}/{}, res={:.3g}, comp={:.3g}. Using best-effort solution.",
            math::toString(fallbackResult.status),
            fallbackResult.iterations,
            fallbackOptions.maxIterations,
            fallbackResult.residual,
            fallbackResult.complementarity);
      }
    }
  }

  const bool solutionFinite = mX.allFinite();
  if (!success && fallbackRan && !solutionFinite) {
    mX = mX.array().isFinite().select(mX, 0.0);
  }

  const bool finalSuccess = success || (fallbackRan && fallbackUsable);

#if !defined(NDEBUG)
  const double maxImpulse = mX.size() > 0 ? mX.cwiseAbs().maxCoeff() : 0.0;
  if (maxImpulse > 1e6) {
    DART_WARN(
        "[ConstraintSolver] Large impulse magnitude detected (max = {}).",
        maxImpulse);
  }
#endif

  if (!finalSuccess) {
    if (!isUsable(mX)) {
      DART_ERROR(
          "[ConstraintSolver] The solution of LCP includes non-finite or "
          "unbounded values: {}. We're setting it zero for safety. Consider "
          "using a more robust secondary solver. If this happens even with a "
          "secondary solver, please report this as a bug.",
          mX.transpose());
    } else {
      DART_ERROR(
          "[ConstraintSolver] Primary LCP solver failed to find a solution. "
          "The constraint impulses are set to zero for safety. Consider "
          "configuring a secondary solver (e.g., PGS) to provide a fallback "
          "when Dantzig fails.");
    }

    mX.setZero();
  }

  if (phase == ConstraintPhase::Position) {
    DART_PROFILE_SCOPED_N("Apply position impulses");
    for (std::size_t i = 0; i < numConstraints; ++i) {
      auto* contact = dynamic_cast<ContactConstraint*>(constraints[i].get());
      DART_ASSERT(contact);
      if (contact) {
        contact->applyPositionImpulse(mX.data() + mOffset[i]);
      }
    }
  } else {
    DART_PROFILE_SCOPED_N("Apply constraint impulses");
    for (std::size_t i = 0; i < numConstraints; ++i) {
      const ConstraintBasePtr& constraint = constraints[i];
      constraint->applyImpulse(mX.data() + mOffset[i]);
      constraint->excite();
    }
  }
}

//==============================================================================
void ConstraintSolver::solveConstrainedGroup(ConstrainedGroup& group)
{
  solveConstrainedGroupInternal(group.mConstraints, ConstraintPhase::Velocity);
}

//==============================================================================
bool ConstraintSolver::isSymmetric(std::size_t n, double* A)
{
  std::size_t nSkip = math::padding(n);
  for (std::size_t i = 0; i < n; ++i) {
    for (std::size_t j = 0; j < n; ++j) {
      if (std::abs(A[nSkip * i + j] - A[nSkip * j + i]) > 1e-6) {
        std::cout << "A: " << std::endl;
        for (std::size_t k = 0; k < n; ++k) {
          for (std::size_t l = 0; l < nSkip; ++l) {
            std::cout << std::setprecision(4) << A[k * nSkip + l] << " ";
          }
          std::cout << std::endl;
        }

        std::cout << "A(" << i << ", " << j << "): " << A[nSkip * i + j]
                  << std::endl;
        std::cout << "A(" << j << ", " << i << "): " << A[nSkip * j + i]
                  << std::endl;
        return false;
      }
    }
  }

  return true;
}

//==============================================================================
bool ConstraintSolver::isSymmetric(
    std::size_t n, double* A, std::size_t begin, std::size_t end)
{
  std::size_t nSkip = math::padding(n);
  for (std::size_t i = begin; i <= end; ++i) {
    for (std::size_t j = begin; j <= end; ++j) {
      if (std::abs(A[nSkip * i + j] - A[nSkip * j + i]) > 1e-6) {
        std::cout << "A: " << std::endl;
        for (std::size_t k = 0; k < n; ++k) {
          for (std::size_t l = 0; l < nSkip; ++l) {
            std::cout << std::setprecision(4) << A[k * nSkip + l] << " ";
          }
          std::cout << std::endl;
        }

        std::cout << "A(" << i << ", " << j << "): " << A[nSkip * i + j]
                  << std::endl;
        std::cout << "A(" << j << ", " << i << "): " << A[nSkip * j + i]
                  << std::endl;
        return false;
      }
    }
  }

  return true;
}

//==============================================================================
void ConstraintSolver::print(
    std::size_t n,
    double* A,
    double* x,
    double* /*lo*/,
    double* /*hi*/,
    double* b,
    double* w,
    int* findex)
{
  std::size_t nSkip = math::padding(n);
  std::cout << "A: " << std::endl;
  for (std::size_t i = 0; i < n; ++i) {
    for (std::size_t j = 0; j < nSkip; ++j) {
      std::cout << std::setprecision(4) << A[i * nSkip + j] << " ";
    }
    std::cout << std::endl;
  }

  std::cout << "b: ";
  for (std::size_t i = 0; i < n; ++i) {
    std::cout << std::setprecision(4) << b[i] << " ";
  }
  std::cout << std::endl;

  std::cout << "w: ";
  for (std::size_t i = 0; i < n; ++i) {
    std::cout << w[i] << " ";
  }
  std::cout << std::endl;

  std::cout << "x: ";
  for (std::size_t i = 0; i < n; ++i) {
    std::cout << x[i] << " ";
  }
  std::cout << std::endl;

  std::cout << "frictionIndex: ";
  for (std::size_t i = 0; i < n; ++i) {
    std::cout << findex[i] << " ";
  }
  std::cout << std::endl;

  auto Ax = std::make_unique_for_overwrite<double[]>(n);
  for (std::size_t i = 0; i < n; ++i) {
    Ax[i] = 0.0;
  }
  for (std::size_t i = 0; i < n; ++i) {
    for (std::size_t j = 0; j < n; ++j) {
      Ax[i] += A[i * nSkip + j] * x[j];
    }
  }

  std::cout << "Ax   : ";
  for (std::size_t i = 0; i < n; ++i) {
    std::cout << Ax[i] << " ";
  }
  std::cout << std::endl;

  std::cout << "b + w: ";
  for (std::size_t i = 0; i < n; ++i) {
    std::cout << b[i] + w[i] << " ";
  }
  std::cout << std::endl;
}

} // namespace constraint
} // namespace dart
