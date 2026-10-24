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
#include "dart/collision/CollisionObject.hpp"
#include "dart/collision/CollisionResult.hpp"
#include "dart/collision/dart/DARTCollisionDetector.hpp"
#include "dart/collision/dart/DARTCollisionGroup.hpp"
#include "dart/collision/dart/detail/NativeShapeConversion.hpp"
#include "dart/collision/dart/shapes/Shape.hpp"
#include "dart/collision/detail/CollisionFilterSnapshotTracker.hpp"
#include "dart/collision/fcl/FCLCollisionDetector.hpp"
#include "dart/collision/fcl/FCLCollisionGroup.hpp"
#include "dart/common/Console.hpp"
#include "dart/common/Logging.hpp"
#include "dart/common/Macros.hpp"
#include "dart/common/String.hpp"
#include "dart/common/detail/DeterministicExecutor.h"
#include "dart/common/detail/ModelMutation.hpp"
#include "dart/constraint/BoxedLcpConstraintSolver.hpp"
#include "dart/constraint/ConstrainedGroup.hpp"
#include "dart/constraint/ConstraintSolver.hpp"
#include "dart/constraint/ContactSurface.hpp"
#include "dart/constraint/DantzigBoxedLcpSolver.hpp"
#include "dart/constraint/FbfFrictionSolver.hpp"
#include "dart/constraint/NsgsFrictionSolver.hpp"
#include "dart/constraint/PgsBoxedLcpSolver.hpp"
#include "dart/constraint/detail/SkeletonMembershipVersion.h"
#include "dart/dynamics/BallJoint.hpp"
#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/BoxShape.hpp"
#include "dart/dynamics/CapsuleShape.hpp"
#include "dart/dynamics/ConeShape.hpp"
#include "dart/dynamics/ConvexMeshShape.hpp"
#include "dart/dynamics/CylinderShape.hpp"
#include "dart/dynamics/DegreeOfFreedom.hpp"
#include "dart/dynamics/EllipsoidShape.hpp"
#include "dart/dynamics/EndEffector.hpp"
#include "dart/dynamics/EulerJoint.hpp"
#include "dart/dynamics/FreeJoint.hpp"
#include "dart/dynamics/Marker.hpp"
#include "dart/dynamics/MeshShape.hpp"
#include "dart/dynamics/PlanarJoint.hpp"
#include "dart/dynamics/PlaneShape.hpp"
#include "dart/dynamics/PrismaticJoint.hpp"
#include "dart/dynamics/PyramidShape.hpp"
#include "dart/dynamics/RevoluteJoint.hpp"
#include "dart/dynamics/ScrewJoint.hpp"
#include "dart/dynamics/ShapeFrame.hpp"
#include "dart/dynamics/ShapeNode.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/dynamics/SoftBodyNode.hpp"
#include "dart/dynamics/SoftMeshShape.hpp"
#include "dart/dynamics/SphereShape.hpp"
#include "dart/dynamics/TranslationalJoint.hpp"
#include "dart/dynamics/TranslationalJoint2D.hpp"
#include "dart/dynamics/UniversalJoint.hpp"
#include "dart/dynamics/WeldJoint.hpp"

#if HAVE_BULLET
  #include "dart/collision/bullet/BulletCollisionDetector.hpp"
  #include "dart/collision/bullet/BulletCollisionGroup.hpp"
#endif
#if HAVE_ODE
  #include "dart/collision/ode/OdeCollisionDetector.hpp"
  #include "dart/collision/ode/OdeCollisionGroup.hpp"
#endif

#include <algorithm>
#include <atomic>
#include <exception>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <new>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <typeinfo>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <cctype>
#include <cmath>
#include <cstdlib>

namespace dart {
namespace collision {
namespace detail {
struct CompositeCollisionFilterAccessor
{
  static const auto& filters(const CompositeCollisionFilter& filter)
  {
    return filter.mFilters;
  }
};

struct CollisionGroupOwnershipAccessor
{
  static void update(CollisionGroup& group, const simulation::World* requester)
  {
    group.updateImpl(requester);
  }
  static bool hasPendingUpdates(const CollisionGroup& group)
  {
    return group.hasPendingSourceUpdates();
  }

  static bool hasSources(const CollisionGroup& group)
  {
    return !group.mSkeletonSources.empty() || !group.mBodyNodeSources.empty();
  }

  template <class Visit>
  static void forEachSource(const CollisionGroup& group, Visit visit)
  {
    for (const auto& entry : group.mSkeletonSources)
      visit(entry.second.mSource);
  }

  template <class Visit>
  static void forEachBodySource(const CollisionGroup& group, Visit visit)
  {
    for (const auto& entry : group.mBodyNodeSources)
      visit(entry.second.mSource);
  }

  template <class Visit>
  static void forEachSkeleton(const CollisionGroup& group, Visit visit)
  {
    for (const auto& entry : group.mSkeletonSources) {
      const auto source = entry.second.mSource.lock();
      if (!source)
        continue;
      if (const auto* skeleton
          = dynamic_cast<const dynamics::Skeleton*>(source.get()))
        visit(skeleton);
      for (std::size_t i = 0u; i < source->getNumBodyNodes(); ++i)
        visit(source->getBodyNode(i)->getSkeleton().get());
    }
    for (const auto& entry : group.mBodyNodeSources) {
      const auto source = entry.second.mSource.lock();
      if (source)
        visit(source->getSkeleton().get());
    }
  }
};
} // namespace detail
} // namespace collision
namespace simulation {

namespace {

using dart::collision::CollisionDetector;
using dart::collision::CollisionDetectorPtr;

constraint::BoxedLcpSolverPtr cloneBoxedLcpSolver(
    const constraint::ConstBoxedLcpSolverPtr& solver, std::string_view role)
{
  const auto* backend = solver.get();
  if (backend == nullptr)
    return nullptr;

  // Match exact types so custom subclasses are not sliced into built-ins.
  if (typeid(*backend) == typeid(constraint::DantzigBoxedLcpSolver))
    return std::make_shared<constraint::DantzigBoxedLcpSolver>();

  if (typeid(*backend) == typeid(constraint::PgsBoxedLcpSolver)) {
    auto clone = std::make_shared<constraint::PgsBoxedLcpSolver>();
    clone->setOption(static_cast<const constraint::PgsBoxedLcpSolver*>(backend)
                         ->getOption());
    return clone;
  }

  if (typeid(*backend) == typeid(constraint::NsgsFrictionSolver))
    return std::make_shared<constraint::NsgsFrictionSolver>(
        static_cast<const constraint::NsgsFrictionSolver*>(backend)
            ->getOptions());

  if (typeid(*backend) == typeid(constraint::FbfFrictionSolver))
    return std::make_shared<constraint::FbfFrictionSolver>(
        static_cast<const constraint::FbfFrictionSolver*>(backend)
            ->getOptions());

  dtwarn << "[World::clone] Cannot clone " << role << " boxed LCP solver type '"
         << solver->getType() << "'. Keeping the default solver.\n";
  return nullptr;
}

constexpr double kFinalSleepLinearRatio = 0.1;
constexpr double kFinalSleepAngularRatio = 0.2;
constexpr std::size_t kDenseContactJitterMinIslandSize = 3;
constexpr double kDefaultSleepContactPenetrationTolerance = 1e-5;
constexpr double kAdaptivePlaneSleepContactPenetrationTolerance = 0.002;

// Largest angle (rad) between a support normal and gravity that still counts
// as a level support (about 0.06 degrees). Gravity drives motion along a
// steeper support, so the first-frame dwell credit does not apply there.
// Analytic shape pairs report exact normals on level supports; a noisier normal
// only forgoes the shortcut.
constexpr double kMaxLevelSupportNormalTilt = 1e-3;

bool isTiltedSupportNormal(
    const Eigen::Vector3d& normal, const Eigen::Vector3d& up)
{
  return normal.cross(up).norm() > kMaxLevelSupportNormalTilt * normal.norm();
}

// Fraction of |g| dt, the speed that one step of gravity adds, below which the
// first-frame dwell credit counts a body as at rest (see
// computeInitialRestSpeeds()).
constexpr double kInitialRestSpeedRatio = 1e-2;

// Largest growth of any of those speeds of a body between the first and the
// second solve that the first-frame dwell credit still counts as at rest, also
// as a fraction of |g| dt. The credit is only for bodies that start at rest,
// and such a body gains its acceleration times dt when it accelerates steadily,
// so none that accelerates steadily along its support, downward or in turning
// faster than 1e-6 |g| gets it, whatever the time step. The first solves leave
// settled bodies well below that, while a 4 m cube whose center of mass is
// 0.1 mm past an edge already tips at 3e-5 |g|.
constexpr double kInitialRestMaxSpeedGrowthRatio = 1e-6;

// The speeds of a body that the first-frame dwell credit checks: those of its
// origin along the support and downward, and its turning, as the tip speed of a
// 1 m lever arm. Upward speed is ignored: it is the Baumgarte velocity that
// corrects an initial penetration, which grows as the time step shrinks. A
// non-finite velocity counts as infinitely fast.
Eigen::Vector3d computeInitialRestSpeeds(
    const dynamics::BodyNode& bodyNode, const Eigen::Vector3d& up)
{
  const Eigen::Vector3d velocity = bodyNode.getLinearVelocity();
  const double upwardSpeed = velocity.dot(up);
  const Eigen::Vector3d speeds(
      (velocity - upwardSpeed * up).norm(),
      std::max(-upwardSpeed, 0.0),
      bodyNode.getAngularVelocity().norm());
  if (!speeds.allFinite())
    return Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  return speeds;
}

std::size_t countBodyNodes(const std::vector<dynamics::SkeletonPtr>& skeletons)
{
  std::size_t count = 0u;
  for (const auto& skeleton : skeletons)
    count += skeleton ? skeleton->getNumBodyNodes() : 0u;
  return count;
}

// Whether a joint with degrees of freedom couples bodies of the skeleton to
// each other, or joins one of them to the world other than through a free
// joint, as gz-physics builds an SDF joint whose parent is the world.
bool hasJointCoupledBodies(const dynamics::Skeleton& skeleton)
{
  std::size_t rootJointDofs = 0u;
  for (std::size_t i = 0; i < skeleton.getNumTrees(); ++i) {
    const auto* rootJoint = skeleton.getRootJoint(i);
    if (rootJoint->getNumDofs() > 0u
        && dynamic_cast<const dynamics::FreeJoint*>(rootJoint) == nullptr) {
      return true;
    }
    rootJointDofs += rootJoint->getNumDofs();
  }
  return skeleton.getNumDofs() > rootJointDofs;
}

bool contactTouchesPlaneShape(const collision::Contact& contact)
{
  auto isPlaneShapeObject = [](const collision::CollisionObject* object) {
    const auto shape = object ? object->getShape() : nullptr;
    return shape != nullptr
           && shape->getType() == dynamics::PlaneShape::getStaticType();
  };

  return isPlaneShapeObject(contact.collisionObject1)
         || isPlaneShapeObject(contact.collisionObject2);
}

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

// A ContactSurfaceHandler other than the solver's built-in default can derive
// contact parameters from state World cannot observe, e.g. a conveyor's
// surface velocity (gz-sim TrackController). A frozen island would never
// consult it again, so islands may sleep only while the built-in default
// handler is the whole chain.
bool usesBuiltInContactSurfaceHandler(
    const constraint::ConstraintSolver& solver)
{
  const auto chain = solver.getLastContactSurfaceHandler();
  auto* handler = chain.get();
  return handler != nullptr
         && typeid(*handler) == typeid(constraint::DefaultContactSurfaceHandler)
         && handler->getParent() == nullptr;
}

// Mobile skeletons that are frozen, or that the next solve may freeze.
bool hasRestingOrCandidateMobileSkeleton(
    const std::vector<dynamics::SkeletonPtr>& skeletons)
{
  for (const auto& skel : skeletons) {
    if (skel->isMobile() && (skel->isResting() || skel->isSleepCandidate()))
      return true;
  }
  return false;
}

// Mobile skeletons with quiet dwell that a between-step change must restart.
bool hasDwellingMobileSkeleton(
    const std::vector<dynamics::SkeletonPtr>& skeletons)
{
  for (const auto& skel : skeletons) {
    if (skel->isMobile() && skel->getRestDwellTime() > 0.0)
      return true;
  }
  return false;
}

// An unordered pair of collision objects, in address order.
struct PairKey
{
  const collision::CollisionObject* first;
  const collision::CollisionObject* second;
};

bool operator<(const PairKey& a, const PairKey& b)
{
  const std::less<const collision::CollisionObject*> less;
  return less(a.first, b.first)
         || (a.first == b.first && less(a.second, b.second));
}

bool operator==(const PairKey& a, const PairKey& b)
{
  return a.first == b.first && a.second == b.second;
}

PairKey makePairKey(const collision::Contact& contact)
{
  const collision::CollisionObject* a = contact.collisionObject1;
  const collision::CollisionObject* b = contact.collisionObject2;
  return std::less<const collision::CollisionObject*>()(b, a) ? PairKey{b, a}
                                                              : PairKey{a, b};
}

bool containsPair(const std::vector<PairKey>& sortedPairs, const PairKey& pair)
{
  return std::binary_search(sortedPairs.begin(), sortedPairs.end(), pair);
}

const dynamics::Skeleton* getSkeletonOf(
    const collision::CollisionObject* object)
{
  const auto* bodyNode = object ? object->getBodyNode() : nullptr;
  return bodyNode ? bodyNode->getSkeletonRawPtr() : nullptr;
}

// A contact the solver no longer solves: neither body moves, and at least one
// of them is a resting mobile skeleton.
bool isRestingPair(const collision::Contact& contact)
{
  const auto* skeleton1 = getSkeletonOf(contact.collisionObject1);
  const auto* skeleton2 = getSkeletonOf(contact.collisionObject2);
  if (skeleton1 == nullptr || skeleton2 == nullptr)
    return false;

  const bool resting1 = skeleton1->isMobile() && skeleton1->isResting();
  const bool resting2 = skeleton2->isMobile() && skeleton2->isResting();
  return (resting1 || resting2) && (resting1 || !skeleton1->isMobile())
         && (resting2 || !skeleton2->isMobile());
}

struct FilterDecision
{
  const collision::CollisionObject* object1;
  const collision::CollisionObject* object2;
  bool ignored;
};

// Forwards to a collision filter and records each of its decisions.
class CollisionFilterRecorder final : public collision::CollisionFilter
{
public:
  bool ignoresCollision(
      const collision::CollisionObject* object1,
      const collision::CollisionObject* object2) const override
  {
    // Bullet's dispatcher keeps the last query filter between collide()
    // calls. Outside World's solve the recorder is disarmed and filters
    // nothing.
    if (mTarget == nullptr)
      return false;

    const bool ignored = mTarget->ignoresCollision(object1, object2);
    // CollisionFilter makes no promise about the calling thread.
    std::lock_guard<std::mutex> lock(mMutex);
    mDecisions.push_back({object1, object2, ignored});
    return ignored;
  }

  const collision::CollisionFilter* mTarget = nullptr;
  mutable std::mutex mMutex;
  // Grow only at a new high-water mark and keep capacity across steps, like
  // contact constraints, collision results, and the pooled manifold cache.
  mutable std::vector<FilterDecision> mDecisions;
};

// Disarms the recorder when the solve returns or throws.
struct RecorderDisarm
{
  CollisionFilterRecorder& recorder;

  ~RecorderDisarm()
  {
    recorder.mTarget = nullptr;
  }
};

// DART_CUSTOM_FILTER_SLEEPING=0|false|off|no (any letter case) turns decision
// replay off for the World objects created afterwards (#3056): custom
// collision filters that World cannot track then keep its bodies awake. A
// temporary escape hatch for 6.20.x, read like
// DART_SOFT_FACE_INTERIOR_CONTACTS.
bool readCustomFilterSleepingSwitch()
{
  const char* value = std::getenv("DART_CUSTOM_FILTER_SLEEPING");
  if (value == nullptr)
    return true;

  std::string lower(value);
  std::transform(
      lower.begin(), lower.end(), lower.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
      });
  const bool off
      = lower == "0" || lower == "false" || lower == "off" || lower == "no";
  const bool on = lower.empty() || lower == "1" || lower == "true"
                  || lower == "on" || lower == "yes";
  if (off) {
    static std::atomic<bool> reported{false};
    if (!reported.exchange(true)) {
      dtmsg << "[World] DART_CUSTOM_FILTER_SLEEPING=" << value
            << ": custom-filter decision replay is disabled.\n";
    }
  } else if (!on) {
    static std::atomic<bool> reported{false};
    if (!reported.exchange(true)) {
      dtwarn << "[World] Ignoring DART_CUSTOM_FILTER_SLEEPING=" << value
             << "; set it to 0, false, off or no to disable custom-filter "
                "decision replay.\n";
    }
  }
  return !off;
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
// Collision-side state of automatic deactivation (#3056). It only holds data;
// the functions below and World implement the logic.
class WorldDeactivationState
{
public:
  explicit WorldDeactivationState(World* world, bool customFilterSleeping)
    : mWorld(world), mCustomFilterSleeping(customFilterSleeping)
  {
  }

  World* mWorld;
  // Registry index bookkeeping, guarded by the registry mutex (ModelOwners
  // below). Model keys take part in compile() ownership conflicts; World keys
  // only route solver-level hooks.
  std::unordered_map<const void*, std::size_t> mModelKeys;
  std::unordered_map<const void*, std::size_t> mWorldKeys;
  std::unordered_map<const dynamics::Frame*, std::vector<const void*>>
      mSimpleFrameChains;
  std::unordered_map<const dynamics::Frame*, std::vector<const void*>>
      mGroupFrameChains;
  std::unordered_map<
      const void*,
      std::vector<std::pair<const void*, const void*>>>
      mGroupSourceChains;
  std::vector<const void*> mSolverSkeletonKeys;
  std::vector<const void*> mSolverKeys;
  std::vector<const void*> mHandlerKeys;
  // Hooks on other threads read these under the registry mutex; the World's
  // own thread writes them. mStepping is set while step() or compile() runs.
  std::atomic<bool> mCompiled{false};
  std::atomic<std::size_t> mEditDepth{0u};
  std::atomic<bool> mTopologyDirty{false};
  std::atomic<std::size_t> mTopologyRevision{0u};
  std::size_t mCompiledTopologyRevision = 0u;
  std::atomic<std::size_t> mParameterRevision{0u};
  std::atomic<bool> mStepping{false};
  // Set by compile() and cleared at destruction or a failed rebuild. Only
  // indexed Worlds have registry entries, model-object hooks and revisions.
  bool mIndexed = false;
  // Pending work retains this token, so leaving and rejoining never reuses it.
  std::shared_ptr<const char> mRegistryIdentity;
  // Worlds stepping on this thread form a chain through this pointer.
  WorldDeactivationState* mOuterSteppingWorld = nullptr;
  // Set lazily before callbacks, so an unchanged resting step avoids TLS.
  WorldDeactivationState** mSteppingSlot = nullptr;
  // A recompile that failed at World::edit() scope exit, reported by the next
  // step() or compile().
  std::exception_ptr mPendingRecompileError;
  collision::CollisionOption mCompiledCollisionOption;
  const collision::CollisionGroup* mCompiledCollisionGroup = nullptr;
  bool mCompiledGroupAutomaticUpdate = true;
  std::vector<const dynamics::Skeleton*> mCompiledSolverSkeletons;
  std::vector<const constraint::ConstraintBase*> mCompiledConstraints;
  std::vector<const constraint::ContactSurfaceHandler*>
      mCompiledContactHandlers;
  std::size_t mCompiledFilterRevision = 0u;
  std::atomic<bool> mRawGroupMutation{false};
  struct CompiledCompositeFilter
  {
    const collision::CompositeCollisionFilter* mFilter;
    std::vector<const collision::CollisionFilter*> mChildren;
  };
  std::vector<CompiledCompositeFilter> mCompiledCompositeFilters;
  std::vector<std::pair<const collision::CollisionFilter*, std::size_t>>
      mCompiledFilterRevisions;
  struct CompiledSubscription
  {
    std::weak_ptr<const dynamics::MetaSkeleton> mSource;
    std::vector<const dynamics::BodyNode*> mBodies;
  };
  std::vector<CompiledSubscription> mCompiledSubscriptions;
  std::vector<dynamics::WeakConstBodyNodePtr> mCompiledBodySubscriptions;
  struct CompiledFrameAncestors
  {
    const dynamics::Frame* mFrame;
    std::vector<const dynamics::Frame*> mAncestors;
  };
  std::vector<CompiledFrameAncestors> mCompiledFrameAncestors;
  struct CompiledFrameChildren
  {
    const dynamics::Frame* mFrame;
    std::vector<const dynamics::Frame*> mChildren;
  };
  std::vector<CompiledFrameChildren> mCompiledFrameChildren;

  // Whether World replays the decisions of collision filters it cannot track
  // by revision (DART_CUSTOM_FILTER_SLEEPING, read at construction).
  const bool mCustomFilterSleeping;

  std::size_t mSolverSkeletonOwnershipCheckCount = 0u;
  std::size_t mSolverSkeletonCount = 0u;
  std::weak_ptr<const collision::CollisionGroup> mSolverSkeletonGroup;
  std::size_t mSolverSkeletonGroupVersion = 0u;
  bool mSolverSkeletonOwnershipValid = false;
  bool mOwnsSolverSkeletons = false;

  // The decisions such a filter made for every pair the last solve's contact
  // query asked about, and what they refer to. Bullet's dispatcher may keep a
  // reference to the recorder after the solve.
  std::shared_ptr<CollisionFilterRecorder> mRecorder;
  const collision::CollisionFilter* mRecordedFilter = nullptr;
  std::weak_ptr<const collision::CollisionGroup> mRecordedGroup;
  std::size_t mRecordedGroupVersion = 0u;
  bool mRecordValid = false;
  // Whether this step's start replayed the record successfully.
  bool mReplayVerified = false;
  // Current step's custom-filter eligibility, refreshed by its wake check.
  bool mStepCustomBodyFilter = false;

  struct PreparedSkeleton
  {
    std::weak_ptr<const dynamics::Skeleton> mSkeleton;
    std::size_t mVersion;
  };
  std::weak_ptr<const collision::CollisionGroup> mPreparedCollisionGroup;
  bool mPreparedGroupAutomaticUpdate = true;
  std::vector<PreparedSkeleton> mPreparedSkeletons;
  std::vector<std::pair<
      const dynamics::Skeleton*,
      std::weak_ptr<const dynamics::Skeleton>>>
      mPreparedSolverSkeletons;
  // Non-ShapeNode frames can change Skeleton ancestors without stamping the
  // global structural version. Group identity/content guards their lifetime:
  // destroying a frame removes it from its group and bumps the content version
  // at once, which isInSimulationMode() checks before reading these pointers.
  std::vector<std::pair<const dynamics::ShapeFrame*, const dynamics::Skeleton*>>
      mPreparedFrameAncestors;
  std::size_t mPreparedValidatedStructuralVersion = 0u;
  std::size_t mPreparedValidatedSolverMembershipVersion = 0u;
  bool mPreparedSolverUsesMembershipVersion = false;

  std::vector<char> mIslandJointDwellReady;

  // Frames since construction, reset() or restoreState(). Rest detection
  // credits bodies that start a run at rest on the first two of these frames.
  int mFramesSinceStart = 0;

  // Contacts of resting pairs, as solved when the set of resting skeletons
  // last changed; the solver no longer computes or solves them. Their
  // CollisionObjects are only used while mRetainedGroup and
  // mRetainedGroupVersion still match the solver's collision group, which
  // proves the objects are alive.
  std::vector<collision::Contact> mRetainedContacts;
  std::vector<PairKey> mRetainedPairs;    // sorted and unique
  std::vector<char> mRestingAtLastUpdate; // resting flag of each skeleton
  std::weak_ptr<const collision::CollisionGroup> mRetainedGroup;
  std::size_t mRetainedGroupVersion = 0u;

  // What World::getLastCollisionResult() reports while contacts are retained:
  // the solver's contacts of the other pairs, then the retained ones. Built on
  // the first read after a full step.
  std::mutex mViewMutex;
  bool mViewDirty = false;
  collision::CollisionResult mView;
};

namespace {

// Ownership registry of indexed Worlds: those with a valid compile index. Lazy
// Worlds never appear here and never take mMutex. ModelOwners::mMutex guards
// every structure here and the per-World index bookkeeping in
// WorldDeactivationState. Mutation hooks resolve ownership from this index
// alone: while holding the mutex they never read another World's containers,
// call user-overridable virtuals, lock weak pointers or release a model object.
struct ModelOwners
{
  std::mutex mMutex;
  std::vector<WorldDeactivationState*> mWorlds;
  // Model object -> indexed Worlds that reference it, each listed once.
  std::unordered_map<const void*, std::vector<WorldDeactivationState*>> mOwners;
  // Lookup scratch, reserved for every indexed World when it joins, so owner
  // deduplication never allocates under mMutex.
  std::vector<WorldDeactivationState*> mScratch;
  bool mScratchInUse = false;
};

ModelOwners& modelOwners()
{
  // Worlds may be held by globals initialized before main. Their destruction
  // can follow function-local static objects, so this bounded registry lives
  // until process exit without running a destructor before those Worlds.
  static auto* owners = new ModelOwners;
  return *owners;
}

// Lock-free gates. Every hook returns before any registry work while no World
// is indexed; the check hooks also run while a World is inside step().
std::atomic<std::size_t> gIndexedWorlds{0u};
std::atomic<std::size_t> gGuardedWorlds{0u};
// Registry lock acquisitions, read by tests to prove lazy paths take none.
std::atomic<std::size_t> gRegistryLocks{0u};
// Innermost World stepping on this thread. Its own callbacks may not change
// its model's topology, whether or not it is indexed. A step engages it before
// running user extension points: collision filters, detectors and contact
// handlers, constraint and solver hooks, and Joint and BodyNode dynamics
// overrides. Overrides of introspection getters (versions, counts, frame
// casts) are not extension points and may run before it.
thread_local WorldDeactivationState* tlsSteppingWorld = nullptr;

void engageSameThreadMarker(WorldDeactivationState& state)
{
  if (state.mSteppingSlot)
    return;
  state.mSteppingSlot = &tlsSteppingWorld;
  state.mOuterSteppingWorld = *state.mSteppingSlot;
  *state.mSteppingSlot = &state;
  gGuardedWorlds.fetch_add(1u, std::memory_order_release);
}

// Ends a step() or compile() that set mStepping: releases the same-thread
// marker if a callback stage engaged it, then the flag, also on exceptions.
struct BusyScope
{
  WorldDeactivationState& mState;
  ~BusyScope()
  {
    if (mState.mSteppingSlot) {
      // Every callback stage engages first, so markers unwind in LIFO order.
      DART_ASSERT(*mState.mSteppingSlot == &mState);
      gGuardedWorlds.fetch_sub(1u, std::memory_order_release);
      *mState.mSteppingSlot = mState.mOuterSteppingWorld;
      mState.mOuterSteppingWorld = nullptr;
      mState.mSteppingSlot = nullptr;
    }
    mState.mStepping.store(false, std::memory_order_release);
  }
};

inline bool noIndexedWorlds()
{
  return gIndexedWorlds.load(std::memory_order_acquire) == 0u;
}

inline bool noGuardedWorlds()
{
  return gGuardedWorlds.load(std::memory_order_acquire) == 0u;
}

// Slow paths stay out of line so a gated hook costs a load and a branch.
#if defined(_MSC_VER)
  #define DART_REGISTRY_NOINLINE __declspec(noinline)
#else
  #define DART_REGISTRY_NOINLINE __attribute__((noinline))
#endif

std::unique_lock<std::mutex> lockRegistry()
{
  gRegistryLocks.fetch_add(1u, std::memory_order_relaxed);
  return std::unique_lock<std::mutex>(modelOwners().mMutex);
}

// Keys identifying an object's ownership: the object, its frame ancestors and
// the Skeletons of the BodyNodes among them. Collected from the caller's own
// object before the registry mutex is taken.
class ModelKeys
{
public:
  void add(const void* key)
  {
    if (!key)
      return;
    if (mSize < kInline)
      mInline[mSize++] = key;
    else
      mOverflow.push_back(key);
  }

  template <class Visit>
  void each(Visit visit) const
  {
    for (std::size_t i = 0u; i < mSize; ++i)
      visit(mInline[i]);
    for (const void* key : mOverflow)
      visit(key);
  }

  bool empty() const
  {
    return mSize == 0u;
  }

  std::vector<const void*> toVector() const
  {
    std::vector<const void*> keys;
    keys.reserve(mSize + mOverflow.size());
    each([&](const void* key) { keys.push_back(key); });
    return keys;
  }

private:
  static constexpr std::size_t kInline = 32u;
  const void* mInline[kInline];
  std::size_t mSize = 0u;
  std::vector<const void*> mOverflow;
};

// Member chains also record BodyNode ancestors so a moved body can find the
// members below it; lookups only need those bodies' Skeletons. dynamic_cast
// through DART's virtual hierarchies is expensive, so bodies are walked
// through getParentBodyNode() and typed entry points avoid cross-casts.
void collectFrameKeys(
    const dynamics::Frame* frame, ModelKeys& keys, bool bodyKeys);

void collectBodyKeys(
    const dynamics::BodyNode* body, ModelKeys& keys, bool bodyKeys)
{
  const dynamics::Skeleton* lastSkeleton = nullptr;
  for (const auto* current = body; current;) {
    // Frame keys are always the Frame subobject: with virtual inheritance a
    // BodyNode* or ShapeFrame* to the same object has a different address.
    if (bodyKeys)
      keys.add(static_cast<const dynamics::Frame*>(current));
    const auto* skeleton = current->getSkeletonRawPtr();
    if (skeleton != lastSkeleton) {
      keys.add(skeleton);
      lastSkeleton = skeleton;
    }
    const auto* parent = current->getParentBodyNode();
    const auto* parentFrame = current->getParentFrame();
    if (parent && parentFrame == parent) {
      current = parent;
      continue;
    }
    // Attached to something other than its parent body: generic walk.
    collectFrameKeys(parentFrame, keys, bodyKeys);
    return;
  }
}

void collectFrameKeys(
    const dynamics::Frame* frame, ModelKeys& keys, bool bodyKeys)
{
  for (const auto* current = frame; current && !current->isWorld();
       current = current->getParentFrame()) {
    if (const auto* body = dynamic_cast<const dynamics::BodyNode*>(current)) {
      collectBodyKeys(body, keys, bodyKeys);
      return;
    }
    keys.add(current);
  }
}

void collectModelKeys(const dynamics::Skeleton* skeleton, ModelKeys& keys)
{
  keys.add(skeleton);
}

void collectModelKeys(const dynamics::BodyNode* body, ModelKeys& keys)
{
  collectBodyKeys(body, keys, false);
}

void collectModelKeys(const dynamics::ShapeFrame* frame, ModelKeys& keys)
{
  if (!frame)
    return;
  keys.add(static_cast<const dynamics::Frame*>(frame));
  if (const auto* node = frame->asShapeNode()) {
    if (const auto* body = node->getBodyNode()) {
      collectBodyKeys(body, keys, false);
      return;
    }
  }
  collectFrameKeys(frame->getParentFrame(), keys, false);
}

void collectModelKeys(const dynamics::Joint* joint, ModelKeys& keys)
{
  if (joint)
    collectBodyKeys(joint->getChildBodyNode(), keys, false);
}

void collectModelKeys(const common::Composite* object, ModelKeys& keys)
{
  if (!object)
    return;
  if (const auto* frame = dynamic_cast<const dynamics::Frame*>(object)) {
    collectFrameKeys(frame, keys, false);
    return;
  }
  if (const auto* skeleton = dynamic_cast<const dynamics::Skeleton*>(object)) {
    keys.add(skeleton);
    return;
  }
  if (const auto* node = dynamic_cast<const dynamics::Node*>(object)) {
    collectBodyKeys(node->getBodyNode(), keys, false);
    return;
  }
  if (const auto* joint = dynamic_cast<const dynamics::Joint*>(object))
    collectBodyKeys(joint->getChildBodyNode(), keys, false);
}

// Requires the registry mutex. Deduplicates into the registry scratch, whose
// capacity covers every indexed World, so a lookup never allocates.
class OwnerSet
{
public:
  OwnerSet() : mOwners(modelOwners().mScratch)
  {
    DART_ASSERT(!modelOwners().mScratchInUse);
    modelOwners().mScratchInUse = true;
    mOwners.clear();
  }

  ~OwnerSet()
  {
    modelOwners().mScratchInUse = false;
  }

  bool insert(WorldDeactivationState* owner)
  {
    if (std::find(mOwners.begin(), mOwners.end(), owner) != mOwners.end())
      return false;
    DART_ASSERT(mOwners.size() < mOwners.capacity());
    mOwners.push_back(owner);
    return true;
  }

  template <class Visit>
  void each(Visit visit) const
  {
    for (auto* owner : mOwners)
      visit(*owner);
  }

private:
  std::vector<WorldDeactivationState*>& mOwners;
};

// Requires the registry mutex. Visitors may update the index, so the owners
// are gathered first.
template <class Visit>
void forEachOwner(const void* key, Visit visit)
{
  const auto& owners = modelOwners().mOwners;
  const auto it = owners.find(key);
  if (it == owners.end())
    return;
  OwnerSet seen;
  for (auto* owner : it->second)
    seen.insert(owner);
  seen.each(visit);
}

// Requires the registry mutex.
template <class Visit>
void forEachOwner(const ModelKeys& keys, Visit visit)
{
  const auto& owners = modelOwners().mOwners;
  if (owners.empty())
    return;
  OwnerSet seen;
  keys.each([&](const void* key) {
    const auto it = owners.find(key);
    if (it == owners.end())
      return;
    for (auto* owner : it->second)
      seen.insert(owner);
  });
  seen.each(visit);
}

using KeyCounts = std::unordered_map<const void*, std::size_t>;

// Test hook: the n-th index allocation from now throws std::bad_alloc.
std::size_t gFailIndexAllocationIn = 0u;
// One-shot test hook between the two rechain locks.
std::function<void()> gRechainUnlockedHook;

// Called before each step of index building that may allocate.
void indexAllocation()
{
  if (gFailIndexAllocationIn != 0u && --gFailIndexAllocationIn == 0u)
    throw std::bad_alloc();
}

// Requires the registry mutex. A failed allocation leaves the index as it was.
void addOwner(KeyCounts& counts, WorldDeactivationState& state, const void* key)
{
  if (!key)
    return;
  indexAllocation();
  auto& count = counts[key];
  if (count == 0u) {
    auto& owners = modelOwners().mOwners;
    try {
      owners[key].push_back(&state);
    } catch (...) {
      const auto found = owners.find(key);
      if (found != owners.end() && found->second.empty())
        owners.erase(found);
      counts.erase(key);
      throw;
    }
  }
  ++count;
}

// Requires the registry mutex. Purged keys may already be gone.
void removeOwner(
    KeyCounts& counts, WorldDeactivationState& state, const void* key)
{
  const auto it = counts.find(key);
  if (it == counts.end())
    return;
  if (--it->second)
    return;
  counts.erase(it);
  auto& owners = modelOwners().mOwners;
  const auto found = owners.find(key);
  if (found == owners.end())
    return;
  auto& list = found->second;
  list.erase(std::remove(list.begin(), list.end(), &state), list.end());
  if (list.empty())
    owners.erase(found);
}

using FrameChains
    = std::unordered_map<const dynamics::Frame*, std::vector<const void*>>;

// Requires the registry mutex. A failed allocation leaves the chain as it was.
void registerChain(
    FrameChains& chains,
    WorldDeactivationState& state,
    const dynamics::Frame* member,
    const ModelKeys& keys)
{
  auto chain = keys.toVector();
  std::size_t added = 0u;
  try {
    for (; added < chain.size(); ++added)
      addOwner(state.mModelKeys, state, chain[added]);
    auto& registered = chains[member];
    for (const void* key : registered)
      removeOwner(state.mModelKeys, state, key);
    registered = std::move(chain);
  } catch (...) {
    while (added != 0u)
      removeOwner(state.mModelKeys, state, chain[--added]);
    throw;
  }
}

// Requires the registry mutex.
void unregisterChain(
    FrameChains& chains,
    WorldDeactivationState& state,
    const dynamics::Frame* member)
{
  const auto it = chains.find(member);
  if (it == chains.end())
    return;
  for (const void* key : it->second)
    removeOwner(state.mModelKeys, state, key);
  chains.erase(it);
}

// Requires the registry mutex.
void unregisterAllChains(FrameChains& chains, WorldDeactivationState& state)
{
  for (const auto& entry : chains) {
    for (const void* key : entry.second)
      removeOwner(state.mModelKeys, state, key);
  }
  chains.clear();
}

// Subscription source -> (body, Skeleton) pairs it contributes.
using SourceChains = std::unordered_map<
    const void*,
    std::vector<std::pair<const void*, const void*>>>;

// Requires the registry mutex.
void registerSource(
    WorldDeactivationState& state,
    const void* source,
    const std::vector<std::pair<const void*, const void*>>& entries)
{
  auto chain = entries;
  std::size_t added = 0u;
  try {
    for (; added < chain.size(); ++added)
      addOwner(state.mModelKeys, state, chain[added].second);
    auto& registered = state.mGroupSourceChains[source];
    for (const auto& entry : registered)
      removeOwner(state.mModelKeys, state, entry.second);
    registered = std::move(chain);
  } catch (...) {
    while (added != 0u)
      removeOwner(state.mModelKeys, state, chain[--added].second);
    throw;
  }
}

// Requires the registry mutex.
void unregisterSource(WorldDeactivationState& state, const void* source)
{
  const auto it = state.mGroupSourceChains.find(source);
  if (it == state.mGroupSourceChains.end())
    return;
  for (const auto& entry : it->second)
    removeOwner(state.mModelKeys, state, entry.second);
  state.mGroupSourceChains.erase(it);
}

// Requires the registry mutex.
void unregisterAllSources(WorldDeactivationState& state)
{
  for (const auto& source : state.mGroupSourceChains) {
    for (const auto& entry : source.second)
      removeOwner(state.mModelKeys, state, entry.second);
  }
  state.mGroupSourceChains.clear();
}

std::vector<std::pair<const void*, const void*>> sourceEntries(
    const dynamics::MetaSkeleton* source)
{
  std::vector<std::pair<const void*, const void*>> entries;
  if (!source)
    return entries;
  if (const auto* skeleton = dynamic_cast<const dynamics::Skeleton*>(source))
    entries.emplace_back(nullptr, skeleton);
  for (std::size_t i = 0u; i < source->getNumBodyNodes(); ++i) {
    const auto* body = source->getBodyNode(i);
    entries.emplace_back(body, body->getSkeleton().get());
  }
  return entries;
}

// Objects the World's solver currently refers to, apart from its contact
// handlers. They only route World-level hooks.
std::vector<const void*> collectSolverKeys(const World& world)
{
  std::vector<const void*> keys;
  const auto* solver = world.getConstraintSolver();
  keys.push_back(solver);
  keys.push_back(solver->getCollisionDetector().get());
  keys.push_back(solver->getCollisionGroup().get());
  if (const auto* boxed
      = dynamic_cast<const constraint::BoxedLcpConstraintSolver*>(solver)) {
    keys.push_back(boxed->getBoxedLcpSolver().get());
    keys.push_back(boxed->getSecondaryBoxedLcpSolver().get());
  }
  for (std::size_t i = 0u; i < solver->getNumConstraints(); ++i)
    keys.push_back(solver->getConstraint(i).get());
  return keys;
}

// The solver's contact handler chain. Handlers are model keys and take part in
// compile() conflicts. They are collected only while some World is compiled,
// by the hooks of operations on the chain (adding, removing, copying or
// reparenting handlers); constraint and skeleton changes never read it. Like
// the step's contact handling, these reads are unsynchronized: reparenting a
// handler that a World uses while another thread operates on that World is a
// data race, as any concurrent mutation of one World's model is.
std::vector<const void*> collectHandlerKeys(const World& world)
{
  std::vector<const void*> keys;
  const auto* solver = world.getConstraintSolver();
  for (auto handler = solver->getLastContactSurfaceHandler(); handler;
       handler = handler->getParent()) {
    if (std::find(keys.begin(), keys.end(), handler.get()) != keys.end())
      break;
    keys.push_back(handler.get());
  }
  return keys;
}

// Requires the registry mutex; reads only the owner's own solver, not its
// handler chain.
void refreshSolverKeys(WorldDeactivationState& state)
{
  auto worldKeys = collectSolverKeys(*state.mWorld);
  for (const void* key : state.mSolverKeys)
    removeOwner(state.mWorldKeys, state, key);
  for (const void* key : worldKeys)
    addOwner(state.mWorldKeys, state, key);
  state.mSolverKeys = std::move(worldKeys);
}

// Requires the registry mutex; walks the owner's own handler chain.
void refreshHandlerKeys(WorldDeactivationState& state)
{
  auto handlerKeys = collectHandlerKeys(*state.mWorld);
  for (const void* key : state.mHandlerKeys)
    removeOwner(state.mModelKeys, state, key);
  for (const void* key : handlerKeys)
    addOwner(state.mModelKeys, state, key);
  state.mHandlerKeys = std::move(handlerKeys);
}

// Everything a World owns, collected from its own containers without the
// registry mutex so no user-overridable virtual runs under it.
struct IndexSnapshot
{
  std::vector<std::pair<const dynamics::Frame*, ModelKeys>> mSimpleFrames;
  std::vector<std::pair<const dynamics::Frame*, ModelKeys>> mGroupFrames;
  std::vector<
      std::pair<const void*, std::vector<std::pair<const void*, const void*>>>>
      mSources;
  std::vector<const void*> mSkeletons;
  std::vector<const void*> mSolverSkeletons;
};

IndexSnapshot collectIndex(const World& world)
{
  IndexSnapshot snapshot;
  for (std::size_t i = 0u; i < world.getNumSkeletons(); ++i)
    snapshot.mSkeletons.push_back(world.getSkeleton(i).get());
  snapshot.mSimpleFrames.reserve(world.getNumSimpleFrames());
  for (std::size_t i = 0u; i < world.getNumSimpleFrames(); ++i) {
    snapshot.mSimpleFrames.emplace_back(
        world.getSimpleFrame(i).get(), ModelKeys());
    collectFrameKeys(
        snapshot.mSimpleFrames.back().first,
        snapshot.mSimpleFrames.back().second,
        true);
  }
  const auto* solver = world.getConstraintSolver();
  const auto group = solver->getCollisionGroup();
  snapshot.mGroupFrames.reserve(group->getNumShapeFrames());
  for (std::size_t i = 0u; i < group->getNumShapeFrames(); ++i) {
    snapshot.mGroupFrames.emplace_back(group->getShapeFrame(i), ModelKeys());
    collectFrameKeys(
        snapshot.mGroupFrames.back().first,
        snapshot.mGroupFrames.back().second,
        true);
  }
  collision::detail::CollisionGroupOwnershipAccessor::forEachSource(
      *group, [&](const auto& weakSource) {
        if (const auto source = weakSource.lock()) {
          snapshot.mSources.emplace_back(
              source.get(), sourceEntries(source.get()));
        }
      });
  collision::detail::CollisionGroupOwnershipAccessor::forEachBodySource(
      *group, [&](const auto& weakSource) {
        if (const auto body = weakSource.lock()) {
          snapshot.mSources.emplace_back(
              body.get(),
              std::vector<std::pair<const void*, const void*>>{
                  {body.get(), body->getSkeleton().get()}});
        }
      });
  for (const auto& skeleton : solver->getSkeletons())
    snapshot.mSolverSkeletons.push_back(skeleton.get());
  return snapshot;
}

// Requires the registry mutex. Drops every entry of the World but keeps it
// indexed.
void unregisterWorld(WorldDeactivationState& state);

// Requires the registry mutex.
void applyIndex(WorldDeactivationState& state, const IndexSnapshot& snapshot)
{
  unregisterWorld(state);
  for (const void* key : snapshot.mSkeletons)
    addOwner(state.mModelKeys, state, key);
  for (const auto& frame : snapshot.mSimpleFrames)
    registerChain(state.mSimpleFrameChains, state, frame.first, frame.second);
  for (const auto& frame : snapshot.mGroupFrames)
    registerChain(state.mGroupFrameChains, state, frame.first, frame.second);
  for (const auto& source : snapshot.mSources)
    registerSource(state, source.first, source.second);
  for (const void* key : snapshot.mSolverSkeletons) {
    state.mSolverSkeletonKeys.push_back(key);
    addOwner(state.mModelKeys, state, key);
  }
  refreshSolverKeys(state);
  refreshHandlerKeys(state);
}

// Requires the registry mutex. Drops the World from the registry; the counters
// were published when it joined.
void leaveRegistry(WorldDeactivationState& state) noexcept
{
  auto& worlds = modelOwners().mWorlds;
  worlds.erase(std::remove(worlds.begin(), worlds.end(), &state), worlds.end());
  state.mIndexed = false;
  state.mRegistryIdentity.reset();
  gIndexedWorlds.fetch_sub(1u, std::memory_order_release);
  gGuardedWorlds.fetch_sub(1u, std::memory_order_release);
}

// Rebuilds the World's index from its own containers, joining the registry on
// the first call. This is where index memory is allocated. A failure leaves
// no entry behind: the World becomes lazy until a successful compile().
void reindexWorld(WorldDeactivationState& state)
{
  const IndexSnapshot snapshot = collectIndex(*state.mWorld);
  const auto lock = lockRegistry();
  auto& owners = modelOwners();
  const bool joining = !state.mIndexed;
  if (joining) {
    // Reserve before publishing anything: each step below either completes
    // or leaves the registry as it was.
    indexAllocation();
    auto identity = std::make_shared<const char>();
    owners.mScratch.reserve(owners.mWorlds.size() + 1u);
    owners.mWorlds.push_back(&state);
    state.mRegistryIdentity = std::move(identity);
    state.mIndexed = true;
    gIndexedWorlds.fetch_add(1u, std::memory_order_release);
    gGuardedWorlds.fetch_add(1u, std::memory_order_release);
  }
  try {
    applyIndex(state, snapshot);
  } catch (...) {
    unregisterWorld(state);
    leaveRegistry(state);
    state.mCompiled.store(false, std::memory_order_relaxed);
    throw;
  }
}

// Leaves the registry; a no-op for lazy Worlds.
void unindexWorld(WorldDeactivationState& state)
{
  if (!state.mIndexed)
    return;
  const auto lock = lockRegistry();
  unregisterWorld(state);
  leaveRegistry(state);
}

// Requires the registry mutex.
void unregisterWorld(WorldDeactivationState& state)
{
  auto& owners = modelOwners().mOwners;
  const auto drop = [&](const KeyCounts& counts) {
    for (const auto& entry : counts) {
      const auto found = owners.find(entry.first);
      if (found == owners.end())
        continue;
      auto& list = found->second;
      list.erase(std::remove(list.begin(), list.end(), &state), list.end());
      if (list.empty())
        owners.erase(found);
    }
  };
  drop(state.mModelKeys);
  drop(state.mWorldKeys);
  state.mModelKeys.clear();
  state.mWorldKeys.clear();
  state.mSimpleFrameChains.clear();
  state.mGroupFrameChains.clear();
  state.mGroupSourceChains.clear();
  state.mSolverSkeletonKeys.clear();
  state.mSolverKeys.clear();
  state.mHandlerKeys.clear();
}

// A frame moved: members below it record new ancestors. Members are found and
// swapped under the mutex; their ancestry is walked between the two locks.
void rechainMembersUnder(const dynamics::Frame* frame)
{
  struct Pending
  {
    std::shared_ptr<const char> mIdentity;
    bool mGroup;
    const dynamics::Frame* mMember;
    ModelKeys mKeys;
  };
  std::vector<Pending> pending;
  {
    const auto lock = lockRegistry();
    const auto& owners = modelOwners().mOwners;
    const auto it = owners.find(frame);
    if (it == owners.end())
      return;
    for (auto* state : it->second) {
      for (auto* chains :
           {&state->mSimpleFrameChains, &state->mGroupFrameChains}) {
        for (const auto& entry : *chains) {
          if (std::find(entry.second.begin(), entry.second.end(), frame)
              != entry.second.end())
            pending.push_back(
                {state->mRegistryIdentity,
                 chains == &state->mGroupFrameChains,
                 entry.first,
                 ModelKeys()});
        }
      }
    }
  }
  if (pending.empty())
    return;
  for (auto& entry : pending)
    collectFrameKeys(entry.mMember, entry.mKeys, true);
  if (gRechainUnlockedHook)
    std::exchange(gRechainUnlockedHook, {})();
  const auto lock = lockRegistry();
  const auto& worlds = modelOwners().mWorlds;
  for (auto& entry : pending) {
    const auto found
        = std::find_if(worlds.begin(), worlds.end(), [&](const auto* state) {
            return state->mRegistryIdentity == entry.mIdentity;
          });
    if (found == worlds.end())
      continue;
    auto& state = **found;
    auto& chains
        = entry.mGroup ? state.mGroupFrameChains : state.mSimpleFrameChains;
    if (chains.find(entry.mMember) == chains.end())
      continue;
    registerChain(chains, state, entry.mMember, entry.mKeys);
  }
}

// A Skeleton that only weak references (group subscriptions) still name is
// being destroyed: forget its address everywhere so a later object reusing it
// is not mistaken for an owned model.
void purgeModelKey(const void* key)
{
  const auto lock = lockRegistry();
  auto& owners = modelOwners().mOwners;
  const auto it = owners.find(key);
  if (it == owners.end())
    return;
  const auto states = it->second;
  owners.erase(it);
  for (auto* state : states) {
    state->mModelKeys.erase(key);
    state->mWorldKeys.erase(key);
    for (auto* chains :
         {&state->mSimpleFrameChains, &state->mGroupFrameChains}) {
      for (auto& entry : *chains) {
        entry.second.erase(
            std::remove(entry.second.begin(), entry.second.end(), key),
            entry.second.end());
      }
    }
    for (auto& source : state->mGroupSourceChains) {
      auto& entries = source.second;
      entries.erase(
          std::remove_if(
              entries.begin(),
              entries.end(),
              [key](const auto& entry) { return entry.second == key; }),
          entries.end());
    }
    state->mSolverSkeletonKeys.erase(
        std::remove(
            state->mSolverSkeletonKeys.begin(),
            state->mSolverSkeletonKeys.end(),
            key),
        state->mSolverSkeletonKeys.end());
  }
}

// Why a World rejects a model mutation. Decided under the registry mutex and
// thrown after releasing it, so the check paths allocate nothing while it is
// held.
enum class Rejection
{
  None,
  Busy,
  Frozen,
  Shared
};

Rejection rejection(const WorldDeactivationState& state)
{
  if (state.mStepping.load(std::memory_order_acquire))
    return Rejection::Busy;
  if (state.mCompiled.load(std::memory_order_relaxed)
      && state.mEditDepth.load(std::memory_order_relaxed) == 0u)
    return Rejection::Frozen;
  return Rejection::None;
}

void throwRejection(Rejection reason)
{
  switch (reason) {
    case Rejection::None:
      return;
    case Rejection::Busy:
      throw std::runtime_error("WorldBusyError: model mutation during step");
    case Rejection::Frozen:
      throw std::runtime_error(
          "ModelFrozenError: use World::edit() for topology changes");
    case Rejection::Shared:
      throw std::runtime_error(
          "ModelFrozenError: model belongs to another World");
  }
}

void checkModelMutation(const WorldDeactivationState& state)
{
  throwRejection(rejection(state));
}

// Same-thread guard of a lazy World inside step(): reads only that World's own
// containers, which no other thread may touch while it steps. Members are
// matched through the same ancestry as the compiled index records: a frame,
// its frame ancestors and the Skeletons of the BodyNodes among them.
template <class Keys>
bool steppingWorldOwnsModel(
    const WorldDeactivationState& state, const Keys& keys)
{
  bool owned = false;
  const auto match = [&](const void* candidate) {
    keys.each([&](const void* key) { owned = owned || key == candidate; });
  };
  const auto matchChain = [&](const dynamics::Frame* member) {
    ModelKeys chain;
    collectFrameKeys(member, chain, true);
    chain.each(match);
  };
  const World& world = *state.mWorld;
  for (std::size_t i = 0u; !owned && i < world.getNumSkeletons(); ++i)
    match(world.getSkeleton(i).get());
  for (std::size_t i = 0u; !owned && i < world.getNumSimpleFrames(); ++i)
    matchChain(world.getSimpleFrame(i).get());
  const auto* solver = world.getConstraintSolver();
  for (const auto& skeleton : solver->getSkeletons()) {
    if (owned)
      break;
    match(skeleton.get());
  }
  const auto group = solver->getCollisionGroup();
  for (std::size_t i = 0u; !owned && i < group->getNumShapeFrames(); ++i)
    matchChain(group->getShapeFrame(i));
  collision::detail::CollisionGroupOwnershipAccessor::forEachSource(
      *group, [&](const auto& weakSource) {
        const auto source = weakSource.lock();
        if (!source || owned)
          return;
        match(source.get());
        for (std::size_t i = 0u; !owned && i < source->getNumBodyNodes(); ++i)
          match(source->getBodyNode(i)->getSkeleton().get());
      });
  collision::detail::CollisionGroupOwnershipAccessor::forEachBodySource(
      *group, [&](const auto& weakSource) {
        if (owned)
          return;
        if (const auto body = weakSource.lock())
          match(body->getSkeleton().get());
      });
  return owned;
}

template <class Keys>
bool steppingWorldOwnsSolverObject(
    const WorldDeactivationState& state, const Keys& keys)
{
  bool owned = false;
  const auto match = [&](const void* candidate) {
    keys.each([&](const void* key) { owned = owned || key == candidate; });
  };
  const auto* solver = state.mWorld->getConstraintSolver();
  match(solver);
  match(solver->getCollisionDetector().get());
  match(solver->getCollisionGroup().get());
  if (const auto* boxed
      = dynamic_cast<const constraint::BoxedLcpConstraintSolver*>(solver)) {
    match(boxed->getBoxedLcpSolver().get());
    match(boxed->getSecondaryBoxedLcpSolver().get());
  }
  for (std::size_t i = 0u; !owned && i < solver->getNumConstraints(); ++i)
    match(solver->getConstraint(i).get());
  if (owned)
    return true;
  // The same walk as collectHandlerKeys(): a repeated handler ends the chain.
  ModelKeys seen;
  for (auto handler = solver->getLastContactSurfaceHandler(); handler;
       handler = handler->getParent()) {
    match(handler.get());
    if (owned)
      return true;
    bool repeated = false;
    seen.each(
        [&](const void* key) { repeated = repeated || key == handler.get(); });
    if (repeated)
      break;
    seen.add(handler.get());
  }
  return false;
}

// Rejection by the lazy Worlds stepping on this thread. Indexed ones answer
// through the registry instead.
template <class Owns, class Keys>
Rejection lazyStepRejection(Owns owns, const Keys& keys)
{
  for (const auto* state = tlsSteppingWorld; state;
       state = state->mOuterSteppingWorld) {
    if (state->mIndexed)
      continue;
    if (owns(*state, keys))
      return Rejection::Busy;
  }
  return Rejection::None;
}

struct SingleKey
{
  const void* mKey;
  template <class Visit>
  void each(Visit visit) const
  {
    visit(mKey);
  }
};

Rejection lazyStepRejection(const ModelKeys& keys)
{
  return lazyStepRejection(steppingWorldOwnsModel<ModelKeys>, keys);
}

Rejection lazyStepRejection(const void* solverObject)
{
  return lazyStepRejection(
      steppingWorldOwnsSolverObject<SingleKey>, SingleKey{solverObject});
}

// Records the first rejection among visited owners.
struct RejectionCollector
{
  Rejection mReason = Rejection::None;
  void operator()(const WorldDeactivationState& owner)
  {
    if (mReason == Rejection::None)
      mReason = rejection(owner);
  }
};

void notifyModelTopology(WorldDeactivationState& state)
{
  if (state.mEditDepth.load(std::memory_order_relaxed)) {
    if (!state.mTopologyDirty.exchange(true, std::memory_order_relaxed))
      state.mTopologyRevision.fetch_add(1u, std::memory_order_relaxed);
  } else {
    state.mTopologyRevision.fetch_add(1u, std::memory_order_relaxed);
  }
}

//==============================================================================
// Custom body filters can sleep through revision tracking or decision replay.
bool isCustomBodyNodeFilter(
    const WorldDeactivationState& state,
    const collision::CollisionFilter* filter)
{
  return filter != nullptr
         && typeid(*filter) != typeid(collision::BodyNodeCollisionFilter)
         && dynamic_cast<const collision::BodyNodeCollisionFilter*>(filter)
                != nullptr
         && (state.mCustomFilterSleeping
             || dynamic_cast<
                    const collision::detail::CollisionFilterSnapshotTracker*>(
                    filter)
                    != nullptr);
}

bool isReplayed(
    const WorldDeactivationState& state,
    const collision::CollisionFilter* filter)
{
  return state.mCustomFilterSleeping && isCustomBodyNodeFilter(state, filter)
         && dynamic_cast<
                const collision::detail::CollisionFilterSnapshotTracker*>(
                filter)
                == nullptr;
}

//==============================================================================
// Custom-filter sleepers report no residual acceleration from their last
// solve. Joint resets leave acceleration-actuator commands alone.
void clearRestingAccelerations(
    dynamics::Skeleton& skeleton, bool updateVelocity = false)
{
  for (std::size_t joint = 0; joint < skeleton.getNumJoints(); ++joint)
    skeleton.getJoint(joint)->resetAccelerations();
  skeleton.computeForwardKinematics(false, updateVelocity, true);
}

//==============================================================================
// Invalidates the previous record and arms the recorder for `filter`.
CollisionFilterRecorder& armRecorder(
    WorldDeactivationState& state, const collision::CollisionFilter* filter)
{
  state.mRecordValid = false;
  if (!state.mRecorder)
    state.mRecorder = std::make_shared<CollisionFilterRecorder>();
  state.mRecorder->mDecisions.clear();
  state.mRecorder->mTarget = filter;
  return *state.mRecorder;
}

//==============================================================================
// Keeps the record of the solve that just ran only if it holds a decision for
// every pair the query could have reported: the query did not stop at its
// contact limit, contacts were enabled, and the collision group and filter
// stayed the same.
void finishRecord(
    WorldDeactivationState& state,
    const constraint::ConstraintSolver& solver,
    const std::shared_ptr<collision::CollisionFilter>& filter,
    const collision::ConstCollisionGroupPtr& group,
    std::size_t groupVersion,
    bool queryComplete,
    bool enableContact,
    bool restingAtSolve)
{
  const bool recordValid
      = queryComplete && enableContact && solver.getCollisionGroup() == group
        && group->getContentVersion() == groupVersion
        && solver.getCollisionOption().collisionFilter == filter;
  if (!recordValid)
    return;

  // While a body rests, BodyNodeCollisionFilter skips resting pairs during the
  // solver's query only; keep the filter's own decisions, which replay asks
  // for outside the query.
  if (restingAtSolve) {
    for (auto& decision : state.mRecorder->mDecisions) {
      decision.ignored
          = filter->ignoresCollision(decision.object1, decision.object2);
    }
  }
  state.mRecordedFilter = filter.get();
  state.mRecordedGroup = group;
  state.mRecordedGroupVersion = groupVersion;
  state.mRecordValid = true;
}

//==============================================================================
// Whether the solver's collision filter makes every recorded decision again.
// Resting bodies and their supports do not move, and other edits are caught by
// World's version and snapshot checks, so the pairs the last query asked about
// are all the pairs that can produce contacts.
bool replayRecord(
    WorldDeactivationState& state, const constraint::ConstraintSolver& solver)
{
  const auto* filter = solver.getCollisionOption().collisionFilter.get();
  if (!state.mRecordValid || filter != state.mRecordedFilter)
    return false;

  // The recorded objects are alive while the group keeps its content version.
  const auto group = state.mRecordedGroup.lock();
  if (group == nullptr || group != solver.getCollisionGroup()
      || group->getContentVersion() != state.mRecordedGroupVersion) {
    return false;
  }

  for (const auto& decision : state.mRecorder->mDecisions) {
    engageSameThreadMarker(state);
    if (filter->ignoresCollision(decision.object1, decision.object2)
        != decision.ignored) {
      return false;
    }
  }
  return true;
}

//==============================================================================
void clearRetainedContacts(WorldDeactivationState& state)
{
  state.mRetainedContacts.clear();
  state.mRetainedPairs.clear();
  state.mRestingAtLastUpdate.clear();
  state.mRetainedGroup.reset();
  state.mRetainedGroupVersion = 0u;
  state.mViewDirty = false;
  state.mView.clear();
}

//==============================================================================
// Whether there are retained contacts and every object they name is still in
// the solver's collision group. Removing an object from a group either bumps
// its content version right away or destroys the group.
bool isRetentionAlive(
    const WorldDeactivationState& state,
    const constraint::ConstraintSolver& solver)
{
  if (state.mRetainedContacts.empty())
    return false;

  const auto group = state.mRetainedGroup.lock();
  return group != nullptr && group == solver.getCollisionGroup()
         && group->getContentVersion() == state.mRetainedGroupVersion;
}

//==============================================================================
void sortRetainedPairs(WorldDeactivationState& state)
{
  auto& pairs = state.mRetainedPairs;
  pairs.clear();
  for (const auto& contact : state.mRetainedContacts)
    pairs.push_back(makePairKey(contact));
  std::sort(pairs.begin(), pairs.end());
  pairs.erase(std::unique(pairs.begin(), pairs.end()), pairs.end());
}

//==============================================================================
// Runs at the end of every full step with deactivation on. Custom body filters
// retain contacts whenever the set of resting skeletons changes: drop pairs
// that are no longer resting and retain this step's contacts of the pairs that
// froze in it, with the forces of their final solve. A resting pair missing
// from that solve's result (a one-frame contact miss) waits for the next
// change. `keep` is false when nothing rests, when the collision group changed
// during the step (the solver result may name freed objects), or when the
// collision filter cannot preserve sleep (its bodies wake at the next step
// start).
void updateRetainedContacts(
    WorldDeactivationState& state,
    const constraint::ConstraintSolver& solver,
    const std::vector<dynamics::SkeletonPtr>& skeletons,
    bool keep)
{
  if (!keep) {
    if (!state.mRetainedContacts.empty() || !state.mRestingAtLastUpdate.empty())
      clearRetainedContacts(state);
    return;
  }

  const auto group = solver.getCollisionGroup();
  bool restingSetChanged
      = state.mRestingAtLastUpdate.size() != skeletons.size();
  if (!state.mRetainedContacts.empty() && !isRetentionAlive(state, solver)) {
    // The retained objects may be gone; drop them without touching them.
    state.mRetainedContacts.clear();
    restingSetChanged = true;
  }
  for (std::size_t i = 0; i < skeletons.size() && !restingSetChanged; ++i) {
    restingSetChanged
        = (state.mRestingAtLastUpdate[i] != 0) != skeletons[i]->isResting();
  }

  if (restingSetChanged) {
    auto& retained = state.mRetainedContacts;
    retained.erase(
        std::remove_if(
            retained.begin(),
            retained.end(),
            [](const collision::Contact& contact) {
              return !isRestingPair(contact);
            }),
        retained.end());
    sortRetainedPairs(state);

    const std::size_t numRetained = retained.size();
    for (const auto& contact : solver.getLastCollisionResult().getContacts()) {
      if (isRestingPair(contact)
          && !containsPair(state.mRetainedPairs, makePairKey(contact))) {
        retained.push_back(contact);
        // The detector's own data, such as the dart detector's manifold
        // cache, does not outlive the step.
        retained.back().userData = nullptr;
      }
    }
    if (retained.size() != numRetained)
      sortRetainedPairs(state);

    state.mRestingAtLastUpdate.resize(skeletons.size());
    for (std::size_t i = 0; i < skeletons.size(); ++i)
      state.mRestingAtLastUpdate[i] = skeletons[i]->isResting() ? 1 : 0;
    state.mRetainedGroup = group;
    state.mRetainedGroupVersion = group->getContentVersion();
  }

  state.mViewDirty = !state.mRetainedContacts.empty();
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
  mSimulationModeCollisionGroup = nullptr;
  mSimulationModeCollisionGroupVersion = 0;
  mSimulationModeConstraintSolverThreads = 0;
  mSimulationModeCollisionEnableContact = false;
  mSimulationModeCollisionMaxNumContacts = 0;
  mSimulationModeCollisionMaxNumContactsPerPair = 0;
  mSimulationModeCollisionAllowNegativePenetrationDepthContacts = false;
  mSimulationModeCollisionFilter = nullptr;
  mSimulationModeCollisionFilterTrackable = true;
  mSimulationModeCollisionFilterRevision = 0;
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
void World::reserveSimulationScratch()
{
  const std::size_t numSkeletons = mSkeletons.size();

  mDisturbedThisStepScratch.reserve(numSkeletons);
  mUnsettledInitialContactSkeletonScratch.reserve(numSkeletons);
  mSupportedInitialContactSkeletonScratch.reserve(numSkeletons);
  mIslandInitialEquilibriumScratch.reserve(numSkeletons);
  mInitiallyMovingSkeletonScratch.reserve(numSkeletons);
  mInitialRestSpeedLimits.reserve(countBodyNodes(mSkeletons));
  mIslandHasMobileSkeletonScratch.reserve(numSkeletons);
  mIslandAllFinalSleepCandidateReadyScratch.reserve(numSkeletons);
  mIslandAllBelowWakeScratch.reserve(numSkeletons);
  mIslandMobileSkeletonCountScratch.reserve(numSkeletons);
  mIslandDwellWakeReadyCountScratch.reserve(numSkeletons);

  // A direct topology edit can leave the old snapshot marked valid until the
  // next step checks it. Detect shape mismatches now so the bake can prepare
  // the replacement storage without disturbing valid same-topology snapshots.
  if (mAllRestingKinematicSnapshotValid) {
    bool snapshotShapeMatches
        = mAllRestingKinematicSnapshot.size() == numSkeletons;
    for (std::size_t i = 0; snapshotShapeMatches && i < numSkeletons; ++i) {
      const auto& snapshot = mAllRestingKinematicSnapshot[i];
      const auto& skeleton = mSkeletons[i];
      snapshotShapeMatches
          = snapshot.mSkeleton == skeleton.get()
            && snapshot.mPositions.size()
                   == static_cast<Eigen::Index>(skeleton->getNumDofs())
            && snapshot.mNumBodyNodes == skeleton->getNumBodyNodes();
    }
    if (!snapshotShapeMatches)
      invalidateAllRestingKinematicSnapshot();
  }

  // Prepare snapshot storage before the first all-resting step. A valid
  // snapshot retains its values so changes can still be detected when waking.
  if (!mAllRestingKinematicSnapshotValid) {
    mAllRestingKinematicSnapshot.resize(numSkeletons);
    for (std::size_t i = 0; i < numSkeletons; ++i) {
      auto& snapshot = mAllRestingKinematicSnapshot[i];
      const auto& skeleton = mSkeletons[i];
      snapshot.mPositions.resize(
          static_cast<Eigen::Index>(skeleton ? skeleton->getNumDofs() : 0u));
      snapshot.mBodyTransforms.reserve(
          skeleton ? skeleton->getNumBodyNodes() : 0u);
    }
  }
  mLastStepRestingWorldSkeletonStates.reserve(numSkeletons);

  if (mDeactivationOptions.mEnabled
      && isCustomBodyNodeFilter(
          *mDeactivationState,
          mConstraintSolver->getCollisionOption().collisionFilter.get())) {
    const auto contactCapacity
        = mConstraintSolver->getLastCollisionResult().getContacts().capacity();
    mDeactivationState->mRetainedContacts.reserve(contactCapacity);
    mDeactivationState->mRetainedPairs.reserve(contactCapacity);
    mDeactivationState->mRestingAtLastUpdate.reserve(numSkeletons);
    mDeactivationState->mIslandJointDwellReady.reserve(numSkeletons);
  }
}

//==============================================================================
void World::enterSimulationMode()
{
  if (isInSimulationMode())
    return;

  // The first-frame dwell credit's speed limits belong to the skeletons and
  // bodies that the first solve saw, so a change between the first two steps
  // drops a pending confirmation.
  mInitialRestSpeedLimits.clear();
  refreshSkeletonDofIndices();
  reserveSimulationScratch();
  if (mConstraintSolver) {
    const auto filter = mConstraintSolver->getCollisionOption().collisionFilter;
    if (mDeactivationOptions.mEnabled
        && isReplayed(*mDeactivationState, filter.get())) {
      // Warm the exact preparation queries without replacing a valid record.
      auto recorder = std::make_shared<CollisionFilterRecorder>();
      recorder->mTarget = filter.get();
      const RecorderDisarm disarm{*recorder};
      mConstraintSolver->prepareForSimulationWithQueryFilter(recorder);
      if (!mDeactivationState->mRecorder) {
        mDeactivationState->mRecorder = recorder;
        recorder->mDecisions.clear();
      } else {
        mDeactivationState->mRecorder->mDecisions.reserve(
            recorder->mDecisions.capacity());
      }
    } else {
      mConstraintSolver->prepareForSimulation();
    }
  }
  reserveSimulationScratch();
  mSimulationModeStructuralVersion
      = dynamics::Skeleton::getGlobalStructuralVersion();
  auto& state = *mDeactivationState;
  state.mPreparedSkeletons.clear();
  state.mPreparedSolverSkeletons.clear();
  state.mPreparedFrameAncestors.clear();
  state.mPreparedCollisionGroup.reset();
  state.mPreparedGroupAutomaticUpdate = true;
  const auto* solver = mConstraintSolver.get();
  state.mPreparedSolverUsesMembershipVersion
      = solver
        && typeid(*solver) == typeid(constraint::BoxedLcpConstraintSolver);
  const auto rememberSkeleton = [&](const dynamics::Skeleton* skeleton) {
    if (skeleton)
      state.mPreparedSkeletons.push_back(
          {skeleton->getPtr(), skeleton->getVersion()});
  };
  for (const auto& skeleton : mSkeletons)
    rememberSkeleton(skeleton.get());
  if (mConstraintSolver) {
    for (const auto& skeleton : mConstraintSolver->getSkeletons()) {
      state.mPreparedSolverSkeletons.emplace_back(skeleton.get(), skeleton);
      rememberSkeleton(skeleton.get());
    }
    const auto collisionGroup = mConstraintSolver->getCollisionGroup();
    state.mPreparedCollisionGroup = collisionGroup;
    state.mPreparedGroupAutomaticUpdate
        = collisionGroup && collisionGroup->getAutomaticUpdate();
    for (std::size_t i = 0u;
         collisionGroup && i < collisionGroup->getNumShapeFrames();
         ++i) {
      const auto* frame = collisionGroup->getShapeFrame(i);
      const auto* skeleton = dynamics::Skeleton::getSkeletonOf(*frame);
      rememberSkeleton(skeleton);
      if (!frame->asShapeNode())
        state.mPreparedFrameAncestors.emplace_back(frame, skeleton);
    }
    mSimulationModeCollisionGroup = collisionGroup.get();
    mSimulationModeCollisionGroupVersion
        = collisionGroup ? collisionGroup->getContentVersion() : 0u;
    mSimulationModeConstraintSolverThreads
        = mConstraintSolver->getNumSimulationThreads();
    const auto& collisionOption = mConstraintSolver->getCollisionOption();
    mSimulationModeCollisionEnableContact = collisionOption.enableContact;
    mSimulationModeCollisionMaxNumContacts = collisionOption.maxNumContacts;
    mSimulationModeCollisionMaxNumContactsPerPair
        = collisionOption.maxNumContactsPerPair;
    mSimulationModeCollisionAllowNegativePenetrationDepthContacts
        = collisionOption.allowNegativePenetrationDepthContacts;
    mSimulationModeCollisionFilter = collisionOption.collisionFilter.get();
    mSimulationModeCollisionFilterTrackable
        = isCollisionFilterSnapshotTrackable(mSimulationModeCollisionFilter);
    mSimulationModeCollisionFilterRevision
        = mSimulationModeCollisionFilterTrackable
              ? getCollisionFilterSnapshotRevision(
                  mSimulationModeCollisionFilter)
              : 0u;
  } else {
    mSimulationModeCollisionGroup = nullptr;
    mSimulationModeCollisionGroupVersion = 0;
    mSimulationModeConstraintSolverThreads = 0;
    mSimulationModeCollisionEnableContact = false;
    mSimulationModeCollisionMaxNumContacts = 0;
    mSimulationModeCollisionMaxNumContactsPerPair = 0;
    mSimulationModeCollisionAllowNegativePenetrationDepthContacts = false;
    mSimulationModeCollisionFilter = nullptr;
    mSimulationModeCollisionFilterTrackable = true;
    mSimulationModeCollisionFilterRevision = 0;
  }
  state.mPreparedValidatedStructuralVersion = mSimulationModeStructuralVersion;
  state.mPreparedValidatedSolverMembershipVersion
      = constraint::detail::gSkeletonMembershipVersion.load(
          std::memory_order_relaxed);
  mSimulationMode = true;
}

//==============================================================================
void World::validateCompiledModel() const
{
  const auto& state = *mDeactivationState;
  if (!state.mCompiled || state.mEditDepth)
    return;

  const auto group = mConstraintSolver->getCollisionGroup();
  const auto& options = mConstraintSolver->getCollisionOption();
  const auto& saved = state.mCompiledCollisionOption;
  bool changed
      = state.mTopologyRevision != state.mCompiledTopologyRevision
        || group.get() != state.mCompiledCollisionGroup
        || state.mRawGroupMutation
        || group->getAutomaticUpdate() != state.mCompiledGroupAutomaticUpdate
        || options.enableContact != saved.enableContact
        || options.maxNumContacts != saved.maxNumContacts
        || options.maxNumContactsPerPair != saved.maxNumContactsPerPair
        || options.allowNegativePenetrationDepthContacts
               != saved.allowNegativePenetrationDepthContacts
        || options.collisionFilter != saved.collisionFilter;
  changed = changed
            || getCollisionFilterSnapshotRevision(options.collisionFilter.get())
                   != state.mCompiledFilterRevision;
  for (const auto& snapshot : state.mCompiledCompositeFilters) {
    if (changed)
      break;
    const auto& children
        = collision::detail::CompositeCollisionFilterAccessor::filters(
            *snapshot.mFilter);
    changed = children.size() != snapshot.mChildren.size();
    for (const auto* child : snapshot.mChildren)
      changed = changed || children.find(child) == children.end();
  }
  for (const auto& snapshot : state.mCompiledFilterRevisions) {
    if (changed)
      break;
    changed
        = getCollisionFilterSnapshotRevision(snapshot.first) != snapshot.second;
  }
  for (const auto& subscription : state.mCompiledSubscriptions) {
    if (changed)
      break;
    const auto source = subscription.mSource.lock();
    // A raw pointer keeps typeid's operand free of side effects.
    const auto* meta = source.get();
    if (meta && typeid(*meta) != typeid(dynamics::Skeleton)
        && state.mStepping.load(std::memory_order_acquire))
      engageSameThreadMarker(*mDeactivationState);
    changed
        = !source || source->getNumBodyNodes() != subscription.mBodies.size();
    for (std::size_t i = 0u; !changed && i < subscription.mBodies.size(); ++i)
      changed = source->getBodyNode(i) != subscription.mBodies[i];
  }
  for (const auto& subscription : state.mCompiledBodySubscriptions) {
    if (changed)
      break;
    changed = !subscription.lock();
  }
  for (const auto& snapshot : state.mCompiledFrameAncestors) {
    if (changed)
      break;
    const auto* parent = snapshot.mFrame->getParentFrame();
    for (const auto* ancestor : snapshot.mAncestors) {
      if (parent != ancestor) {
        changed = true;
        break;
      }
      parent = parent->getParentFrame();
    }
    changed = changed || (parent && !parent->isWorld());
  }
  // Parents precede children. A removed/replaced child is detected before an
  // old child pointer can be dereferenced, including borrowed descendants.
  for (const auto& snapshot : state.mCompiledFrameChildren) {
    if (changed)
      break;
    // The legacy const overload allocates a copied set; this overload only
    // reads the existing child container.
    const auto& children
        = const_cast<dynamics::Frame*>(snapshot.mFrame)->getChildFrames();
    changed = children.size() != snapshot.mChildren.size()
              || !std::equal(
                  children.begin(), children.end(), snapshot.mChildren.begin());
  }
  const auto& skeletons = mConstraintSolver->getSkeletons();
  changed
      = changed || skeletons.size() != state.mCompiledSolverSkeletons.size();
  for (std::size_t i = 0u; !changed && i < skeletons.size(); ++i)
    changed = skeletons[i].get() != state.mCompiledSolverSkeletons[i];
  changed = changed
            || mConstraintSolver->getNumConstraints()
                   != state.mCompiledConstraints.size();
  for (std::size_t i = 0u; !changed && i < state.mCompiledConstraints.size();
       ++i)
    changed = mConstraintSolver->getConstraint(i).get()
              != state.mCompiledConstraints[i];
  auto handler = mConstraintSolver->getLastContactSurfaceHandler();
  for (const auto* savedHandler : state.mCompiledContactHandlers) {
    if (changed)
      break;
    changed = handler.get() != savedHandler;
    if (handler)
      handler = handler->getParent();
  }
  changed = changed || handler != nullptr;
  if (changed)
    throw std::runtime_error(
        "ModelFrozenError: raw model configuration changed; use World::edit()");
}

//==============================================================================
void World::compile()
{
  auto& state = *mDeactivationState;
  // Compilation runs user extension points (collision filters and detectors,
  // solver and dynamics overrides), so it is busy as a step is: its callbacks
  // may not compile, edit or step this World or change its model's topology.
  if (state.mStepping.exchange(true, std::memory_order_acq_rel))
    throw std::runtime_error("WorldBusyError: compile during step or compile");
  const BusyScope busy{state};
  engageSameThreadMarker(state);
  if (state.mEditDepth)
    throw std::runtime_error("Cannot compile inside a World::edit() scope");
  if (state.mPendingRecompileError) {
    // Report the failed scope-exit recompile once; the next call compiles.
    const auto error = std::exchange(state.mPendingRecompileError, nullptr);
    std::rethrow_exception(error);
  }
  validateCompiledModel();
  const auto group = mConstraintSolver->getCollisionGroup();
  collision::detail::CollisionGroupOwnershipAccessor::update(*group, this);
  validateCompiledModel();
  std::vector<const constraint::ContactSurfaceHandler*> contactHandlers;
  for (auto handler = mConstraintSolver->getLastContactSurfaceHandler();
       handler;
       handler = handler->getParent()) {
    if (std::find(contactHandlers.begin(), contactHandlers.end(), handler.get())
        != contactHandlers.end())
      throw std::runtime_error(
          "Cyclic contact handler chain at World::compile()");
    contactHandlers.push_back(handler.get());
  }

  // The first compile joins the registry; a failed one leaves it again.
  const bool wasIndexed = state.mIndexed;
  reindexWorld(state);
  struct IndexRollback
  {
    WorldDeactivationState& mState;
    bool mArmed;
    ~IndexRollback()
    {
      if (mArmed)
        unindexWorld(mState);
    }
  } rollback{state, !wasIndexed};

  // A strict World cannot freeze a model another indexed World still edits.
  {
    Rejection reason = Rejection::None;
    {
      const auto lock = lockRegistry();
      const auto& owners = modelOwners().mOwners;
      for (const auto& entry : state.mModelKeys) {
        const auto it = owners.find(entry.first);
        if (it != owners.end() && it->second.size() > 1u) {
          reason = Rejection::Shared;
          break;
        }
      }
    }
    throwRejection(reason);
  }

  if (mConstraintSolver->getCollisionDetector()->getType() == "dart") {
    for (std::size_t i = 0u; i < group->getNumShapeFrames(); ++i) {
      const auto& shape = group->getShapeFrame(i)->getShape();
      if (shape && shape->getType() != dynamics::SoftMeshShape::getStaticType()
          && !collision::detail::NativeShapeConversion::create(*shape)) {
        throw std::runtime_error(
            "Unsupported collision shape at World::compile(): "
            + shape->getType());
      }
    }
  }

  enterSimulationMode();
  // The snapshots are rebuilt in place: if that throws, the World stays
  // uncompiled, as after a failed scope-exit recompile.
  const bool wasCompiled
      = state.mCompiled.exchange(false, std::memory_order_relaxed);
  state.mCompiledCollisionOption = mConstraintSolver->getCollisionOption();
  state.mCompiledFilterRevision = getCollisionFilterSnapshotRevision(
      state.mCompiledCollisionOption.collisionFilter.get());
  state.mCompiledCompositeFilters.clear();
  state.mCompiledFilterRevisions.clear();
  const auto rememberFilter =
      [&](const auto& self, const collision::CollisionFilter* filter) -> void {
    if (!filter
        || std::any_of(
            state.mCompiledFilterRevisions.begin(),
            state.mCompiledFilterRevisions.end(),
            [filter](const auto& saved) { return saved.first == filter; }))
      return;
    state.mCompiledFilterRevisions.emplace_back(
        filter, getCollisionFilterSnapshotRevision(filter));
    if (const auto* composite
        = dynamic_cast<const collision::CompositeCollisionFilter*>(filter)) {
      const auto& children
          = collision::detail::CompositeCollisionFilterAccessor::filters(
              *composite);
      state.mCompiledCompositeFilters.push_back(
          {composite, {children.begin(), children.end()}});
      for (const auto* child : children)
        self(self, child);
    }
  };
  rememberFilter(
      rememberFilter, state.mCompiledCollisionOption.collisionFilter.get());
  state.mCompiledSubscriptions.clear();
  collision::detail::CollisionGroupOwnershipAccessor::forEachSource(
      *group, [&](const auto& weakSource) {
        const auto source = weakSource.lock();
        if (!source)
          return;
        WorldDeactivationState::CompiledSubscription snapshot{source, {}};
        snapshot.mBodies.reserve(source->getNumBodyNodes());
        for (std::size_t i = 0u; i < source->getNumBodyNodes(); ++i)
          snapshot.mBodies.push_back(source->getBodyNode(i));
        state.mCompiledSubscriptions.push_back(std::move(snapshot));
      });
  state.mCompiledBodySubscriptions.clear();
  collision::detail::CollisionGroupOwnershipAccessor::forEachBodySource(
      *group, [&](const auto& source) {
        state.mCompiledBodySubscriptions.push_back(source);
      });
  state.mCompiledFrameAncestors.clear();
  const auto rememberAncestors = [&](const dynamics::Frame* frame) {
    WorldDeactivationState::CompiledFrameAncestors snapshot{frame, {}};
    for (const auto* parent = frame->getParentFrame();
         parent && !parent->isWorld();
         parent = parent->getParentFrame())
      snapshot.mAncestors.push_back(parent);
    state.mCompiledFrameAncestors.push_back(std::move(snapshot));
  };
  for (std::size_t i = 0u; i < group->getNumShapeFrames(); ++i)
    rememberAncestors(group->getShapeFrame(i));
  for (const auto& frame : mSimpleFrames)
    rememberAncestors(frame.get());
  std::vector<const dynamics::Skeleton*> rememberedSkeletons;
  const auto rememberSkeletonFrames = [&](const dynamics::Skeleton* skeleton) {
    if (!skeleton
        || std::find(
               rememberedSkeletons.begin(), rememberedSkeletons.end(), skeleton)
               != rememberedSkeletons.end())
      return;
    rememberedSkeletons.push_back(skeleton);
    for (std::size_t i = 0u; i < skeleton->getNumTrees(); ++i)
      rememberAncestors(skeleton->getRootBodyNode(i));
  };
  for (const auto& skeleton : mSkeletons)
    rememberSkeletonFrames(skeleton.get());
  for (const auto& skeleton : mConstraintSolver->getSkeletons())
    rememberSkeletonFrames(skeleton.get());
  collision::detail::CollisionGroupOwnershipAccessor::forEachSkeleton(
      *group, rememberSkeletonFrames);
  state.mCompiledFrameChildren.clear();
  const auto rememberChildren
      = [&](const auto& self, const dynamics::Frame* frame) -> void {
    if (std::any_of(
            state.mCompiledFrameChildren.begin(),
            state.mCompiledFrameChildren.end(),
            [frame](const auto& saved) { return saved.mFrame == frame; }))
      return;
    const auto& children = frame->getChildFrames();
    state.mCompiledFrameChildren.push_back(
        {frame, {children.begin(), children.end()}});
    for (const auto* child : children)
      self(self, child);
  };
  for (const auto& snapshot : state.mCompiledFrameAncestors) {
    const auto* root = snapshot.mAncestors.empty() ? snapshot.mFrame
                                                   : snapshot.mAncestors.back();
    rememberChildren(rememberChildren, root);
  }
  state.mCompiledCollisionGroup = group.get();
  state.mRawGroupMutation = false;
  state.mCompiledGroupAutomaticUpdate = group->getAutomaticUpdate();
  state.mCompiledSolverSkeletons.clear();
  for (const auto& skeleton : mConstraintSolver->getSkeletons())
    state.mCompiledSolverSkeletons.push_back(skeleton.get());
  state.mCompiledConstraints.clear();
  for (std::size_t i = 0u; i < mConstraintSolver->getNumConstraints(); ++i)
    state.mCompiledConstraints.push_back(
        mConstraintSolver->getConstraint(i).get());
  state.mCompiledContactHandlers = std::move(contactHandlers);
  if (!wasCompiled && !state.mTopologyDirty)
    ++state.mTopologyRevision;
  state.mTopologyDirty = false;
  state.mCompiledTopologyRevision = state.mTopologyRevision;
  state.mCompiled = true;
  rollback.mArmed = false;
}

//==============================================================================
bool World::isCompiled() const
{
  return mDeactivationState->mCompiled && mDeactivationState->mEditDepth == 0u;
}

//==============================================================================
World::EditScope::EditScope(World& world)
  : mWorld(world), mUncaughtExceptions(std::uncaught_exceptions())
{
  auto& state = *mWorld.mDeactivationState;
  if (state.mStepping.load(std::memory_order_acquire))
    throw std::runtime_error("WorldBusyError: edit during step or compile");
  // Lazy Worlds stay lazy. Compiled ones, and ones whose last recompile
  // failed, recompile at scope exit.
  mRecompile = state.mCompiled.load(std::memory_order_relaxed)
               || state.mPendingRecompileError != nullptr;
  state.mEditDepth.fetch_add(1u, std::memory_order_relaxed);
}

//==============================================================================
World::EditScope::~EditScope()
{
  auto& state = *mWorld.mDeactivationState;
  if (state.mEditDepth.fetch_sub(1u, std::memory_order_relaxed) != 1u)
    return;
  if (!mRecompile) {
    mWorld.invalidateSimulationMode();
    return;
  }
  state.mCompiled.store(false, std::memory_order_relaxed);
  mWorld.invalidateSimulationMode();
  // A failed edit remains editable; do not replace an exception while
  // unwinding.
  if (std::uncaught_exceptions() != mUncaughtExceptions)
    return;
  // This edit may have repaired the failure: compile afresh instead of
  // rethrowing it.
  state.mPendingRecompileError = nullptr;
  try {
    mWorld.compile();
  } catch (...) {
    // Scope exit never throws; the next step() or compile() reports this.
    state.mPendingRecompileError = std::current_exception();
  }
}

//==============================================================================
World::EditScope World::edit()
{
  return EditScope(*this);
}

//==============================================================================
std::size_t World::getTopologyRevision() const
{
  return mDeactivationState->mTopologyRevision;
}

//==============================================================================
std::size_t World::getParameterRevision() const
{
  return mDeactivationState->mParameterRevision;
}

//==============================================================================
bool World::isInSimulationMode() const
{
  if (!mSimulationMode || !mConstraintSolver)
    return false;

  const auto& collisionOption = mConstraintSolver->getCollisionOption();
  const auto* collisionFilter = collisionOption.collisionFilter.get();
  const bool collisionFilterTrackable
      = isCollisionFilterSnapshotTrackable(collisionFilter);
  const bool collisionFilterUnchanged
      = mSimulationModeCollisionFilter == collisionFilter
        && mSimulationModeCollisionFilterTrackable == collisionFilterTrackable
        && (!collisionFilterTrackable
            || mSimulationModeCollisionFilterRevision
                   == getCollisionFilterSnapshotRevision(collisionFilter));
  const auto collisionGroup = mConstraintSolver->getCollisionGroup();
  auto& state = *mDeactivationState;
  if (state.mPreparedCollisionGroup.owner_before(collisionGroup)
      || collisionGroup.owner_before(state.mPreparedCollisionGroup)
      || collisionGroup.get() != mSimulationModeCollisionGroup
      || mSimulationModeCollisionGroupVersion
             != (collisionGroup ? collisionGroup->getContentVersion() : 0u)
      || (collisionGroup
          && collisionGroup->getAutomaticUpdate()
                 != state.mPreparedGroupAutomaticUpdate)) {
    return false;
  }

  const auto solverMembershipVersion
      = constraint::detail::gSkeletonMembershipVersion.load(
          std::memory_order_relaxed);
  if (!state.mPreparedSolverUsesMembershipVersion
      || state.mPreparedValidatedSolverMembershipVersion
             != solverMembershipVersion) {
    const auto& solverSkeletons = mConstraintSolver->getSkeletons();
    if (solverSkeletons.size() != state.mPreparedSolverSkeletons.size())
      return false;
    for (std::size_t i = 0u; i < solverSkeletons.size(); ++i) {
      const auto& prepared = state.mPreparedSolverSkeletons[i];
      if (solverSkeletons[i].get() != prepared.first
          || prepared.second.owner_before(solverSkeletons[i])
          || solverSkeletons[i].owner_before(prepared.second)) {
        return false;
      }
    }
    state.mPreparedValidatedSolverMembershipVersion = solverMembershipVersion;
  }
  for (const auto& ancestor : state.mPreparedFrameAncestors) {
    if (dynamics::Skeleton::getSkeletonOf(*ancestor.first) != ancestor.second)
      return false;
  }

  const auto structuralVersion
      = dynamics::Skeleton::getGlobalStructuralVersion();
  if (state.mPreparedValidatedStructuralVersion != structuralVersion) {
    // Synchronize subscriptions before trusting their contents, including an
    // external subscription that had no collision shapes at preparation.
    if (collisionGroup && collisionGroup->getAutomaticUpdate()) {
      if (state.mStepping.load(std::memory_order_acquire))
        engageSameThreadMarker(state);
      collisionGroup->update();
      if (collisionGroup->getContentVersion()
          != mSimulationModeCollisionGroupVersion) {
        return false;
      }
    }
    for (const auto& snapshot : state.mPreparedSkeletons) {
      const auto skeleton = snapshot.mSkeleton.lock();
      if (!skeleton || skeleton->getVersion() != snapshot.mVersion)
        return false;
    }
    // Only unrelated edits catch up this hint; a stale dependency stays stale
    // until enterSimulationMode() captures a new preparation snapshot.
    state.mPreparedValidatedStructuralVersion = structuralVersion;
  }

  return mSimulationMode
         && mConstraintSolver->getNumSimulationThreads()
                == mSimulationModeConstraintSolverThreads
         && mSimulationModeCollisionEnableContact
                == collisionOption.enableContact
         && mSimulationModeCollisionMaxNumContacts
                == collisionOption.maxNumContacts
         && mSimulationModeCollisionMaxNumContactsPerPair
                == collisionOption.maxNumContactsPerPair
         && mSimulationModeCollisionAllowNegativePenetrationDepthContacts
                == collisionOption.allowNegativePenetrationDepthContacts
         && collisionFilterUnchanged;
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
class SimulationThreadPool : public common::detail::DeterministicExecutor
{
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

  // Lazy Worlds reject callback mutations through the calling thread's chain.
  auto* steppingWorld = common::detail::getWorldStepContext();
  threadPool->parallelFor(count, numThreads, [&](std::size_t i) {
    const common::detail::ScopedWorldStepContext context{steppingWorld};
    func(i);
  });
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
    mRecording(new Recording(mSkeletons)),
    onNameChanged(mNameChangedSignal),
    mDeactivationState(std::make_unique<WorldDeactivationState>(
        this, readCustomFilterSleepingSwitch()))
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
  unindexWorld(*mDeactivationState);
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

  if (const auto* solver
      = dynamic_cast<const constraint::BoxedLcpConstraintSolver*>(
          getConstraintSolver())) {
    // World::create() always installs a BoxedLcpConstraintSolver.
    auto* cloneSolver = static_cast<constraint::BoxedLcpConstraintSolver*>(
        worldClone->getConstraintSolver());
    if (auto primary
        = cloneBoxedLcpSolver(solver->getBoxedLcpSolver(), "primary"))
      cloneSolver->setBoxedLcpSolver(std::move(primary));

    const auto secondary = solver->getSecondaryBoxedLcpSolver();
    if (auto backend = cloneBoxedLcpSolver(secondary, "secondary"))
      cloneSolver->setSecondaryBoxedLcpSolver(std::move(backend));
    else if (!secondary)
      cloneSolver->setSecondaryBoxedLcpSolver(nullptr);

    cloneSolver->setSplitImpulseEnabled(solver->isSplitImpulseEnabled());
    cloneSolver->setMatrixFreeContactSolverOptions(
        solver->getMatrixFreeContactSolverOptions());
    cloneSolver->setTimeStep(solver->getTimeStep());
    cloneSolver->setNumSimulationThreads(solver->getNumSimulationThreads());

    const auto& options = solver->getCollisionOption();
    auto& cloneOptions = cloneSolver->getCollisionOption();
    cloneOptions.enableContact = options.enableContact;
    cloneOptions.maxNumContacts = options.maxNumContacts;
    cloneOptions.maxNumContactsPerPair = options.maxNumContactsPerPair;
    cloneOptions.allowNegativePenetrationDepthContacts
        = options.allowNegativePenetrationDepthContacts;
    // Filters, handler chains and manual constraints can reference the original
    // bodies or user state and have no clone/remapping API. Keep fresh
    // defaults, as for collision results, contact caches and solver scratch.
  }

  // Clone and add each Skeleton
  for (std::size_t i = 0; i < mSkeletons.size(); ++i) {
    worldClone->addSkeleton(mSkeletons[i]->cloneSkeleton());
  }

  // A compiled World's frames are frozen, so its SimpleFrame clones start at
  // the World frame with their sources' poses, then move to the clones of
  // parents this World holds: its SimpleFrames and its Skeletons' frames.
  // Any other parent throws, inside edit() too: sharing it would put the
  // clone's frame in this World's model at the next recompile, so dropping
  // the clone would change that model.
  if (mDeactivationState->mCompiled.load(std::memory_order_relaxed)) {
    std::unordered_map<const dynamics::Frame*, dynamics::Frame*> clones;
    for (std::size_t i = 0; i < mSkeletons.size(); ++i) {
      const dynamics::Skeleton& source = *mSkeletons[i];
      const auto skeleton = worldClone->getSkeleton(i);
      // cloneSkeleton() keeps the indices of BodyNodes and, per node type, of
      // their Nodes, so each BodyNode's Node lists pair up by position.
      for (std::size_t j = 0; j < source.getNumBodyNodes(); ++j) {
        auto* sourceBody = source.getBodyNode(j);
        auto* cloneBody = skeleton->getBodyNode(j);
        clones[sourceBody] = cloneBody;
        const auto sourceNodes = sourceBody->getNodes();
        const auto cloneNodes = cloneBody->getNodes();
        if (sourceNodes.size() != cloneNodes.size()) {
          throw std::runtime_error(
              "Cannot clone a compiled World: the clone of BodyNode '"
              + sourceBody->getName() + "' has a different set of Nodes");
        }
        for (std::size_t k = 0; k < sourceNodes.size(); ++k) {
          const auto* sourceFrame
              = dynamic_cast<const dynamics::Frame*>(sourceNodes[k]);
          auto* cloneFrame = dynamic_cast<dynamics::Frame*>(cloneNodes[k]);
          if (sourceFrame && cloneFrame)
            clones[sourceFrame] = cloneFrame;
        }
      }
    }
    // Check every parent before cloning: clone() reads the source's World
    // pose, which walks the whole ancestor chain and crashes on a detached one.
    // A detached frame has no World pose to clone from, and an external parent
    // (held by neither this World nor its Skeletons) is not supported.
    for (const auto& frame : mSimpleFrames)
      clones[frame.get()] = nullptr;
    for (const auto& frame : mSimpleFrames) {
      const auto* parent = frame->getParentFrame();
      if (parent == nullptr) {
        throw std::runtime_error(
            "Cannot clone a compiled World: SimpleFrame '" + frame->getName()
            + "' has no parent frame");
      }
      if (!parent->isWorld() && clones.find(parent) == clones.end()) {
        throw std::runtime_error(
            "Cannot clone a compiled World: the parent '" + parent->getName()
            + "' of SimpleFrame '" + frame->getName()
            + "' is not a frame this World holds");
      }
    }
    for (const auto& frame : mSimpleFrames) {
      auto clone = frame->clone(dynamics::Frame::World());
      clones[frame.get()] = clone.get();
      worldClone->addSimpleFrame(clone);
    }
    for (std::size_t i = 0; i < mSimpleFrames.size(); ++i) {
      auto* parent = mSimpleFrames[i]->getParentFrame();
      const auto it = clones.find(parent);
      const auto& clone = worldClone->getSimpleFrame(i);
      clone->setParentFrame(
          parent->isWorld() ? dynamics::Frame::World() : it->second);
      clone->setRelativeTransform(mSimpleFrames[i]->getRelativeTransform());
      clone->setRelativeSpatialVelocity(
          mSimpleFrames[i]->getRelativeSpatialVelocity());
      clone->setRelativeSpatialAcceleration(
          mSimpleFrames[i]->getRelativeSpatialAcceleration());
    }
    return worldClone;
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

  if (_timeStep != mTimeStep
      && isCustomBodyNodeFilter(
          *mDeactivationState,
          mConstraintSolver->getCollisionOption().collisionFilter.get())) {
    wakeRestingSkeletonsForWorldChange();
  }
  if (_timeStep != mTimeStep)
    ++mDeactivationState->mParameterRevision;
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

  const auto previousNumSimulationThreads = mNumSimulationThreads;
  if (std::max<std::size_t>(1u, numThreads) != previousNumSimulationThreads)
    checkModelMutation(*mDeactivationState);
  if (mConstraintSolver) {
    const auto* detector
        = dynamic_cast<const collision::DARTCollisionDetector*>(
            mConstraintSolver->getCollisionDetector().get());
    if (detector
        && detector->getNumCollisionThreads()
               != std::max<std::size_t>(1u, numThreads))
      collision::detail::checkWorldDetectorTopologyMutation(detector);
  }
  mNumSimulationThreads = std::max<std::size_t>(1u, numThreads);
  if (mConstraintSolver)
    mConstraintSolver->setNumSimulationThreads(mNumSimulationThreads);
  if (mNumSimulationThreads != previousNumSimulationThreads) {
    invalidateSimulationMode();
    notifyModelTopology(*mDeactivationState);
  }

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
  mDeactivationState->mFramesSinceStart = 0;
  mInitialRestSpeedLimits.clear();
  mRecording->clear();
  mConstraintSolver->clearLastCollisionResult();
  // Also clears sleep candidacy and dwell: the next step start cannot compare
  // against the state before reset() (#3056).
  wakeRestingSkeletonsForWorldChange();

  for (auto& skel : mSkeletons) {
    skel->clearConstraintImpulses();
    skel->setImpulseApplied(false);
  }
}

//==============================================================================
// What one step() call passes between its stages. Nothing in it carries over
// to the next step.
struct World::StepContext
{
  bool resetCommand = true;
  bool deactivationEnabled = false;
  bool customFilterSleeping = false;
  // Copies that keep the solve's collision group and filter alive until
  // step() returns.
  collision::CollisionGroupPtr collisionGroup;
  std::shared_ptr<collision::CollisionFilter> collisionFilter;
  bool collisionGroupUnchanged = false;
};

//==============================================================================
void World::step(bool _resetCommand)
{
  auto& state = *mDeactivationState;
  if (state.mStepping.exchange(true, std::memory_order_acq_rel))
    throw std::runtime_error("WorldBusyError: reentrant World::step()");
  // Callback stages engage the marker lazily.
  const BusyScope busy{state};
  if (state.mEditDepth)
    throw std::runtime_error("Cannot step inside a World::edit() scope");
  if (state.mPendingRecompileError)
    std::rethrow_exception(state.mPendingRecompileError);
  if (state.mCompiled)
    validateCompiledModel();
  const auto group = mConstraintSolver->getCollisionGroup();
  if (group && group->getAutomaticUpdate()
      && collision::detail::CollisionGroupOwnershipAccessor::hasPendingUpdates(
          *group)) {
    engageSameThreadMarker(state);
    collision::detail::CollisionGroupOwnershipAccessor::update(*group, this);
  }
  StepContext context;
  context.resetCommand = _resetCommand;
  beginStep(context);
  if (!context.deactivationEnabled) {
    engageSameThreadMarker(state);
    stepWithoutDeactivation(context);
  } else if (!tryAllRestingFastPath(context)) {
    engageSameThreadMarker(state);
    advanceVelocities(context);
    solveConstraints(context);
    advancePositions(context);
    finishDeactivationStep(context);
  }
}

//==============================================================================
inline void World::beginStep(StepContext& context)
{
  if (!isInSimulationMode()) {
    engageSameThreadMarker(*mDeactivationState);
    enterSimulationMode();
  }
  getMemoryManager().getFrameAllocator().reset();

  const bool deactivationEnabled = mDeactivationOptions.mEnabled;
  context.deactivationEnabled = deactivationEnabled;

  // Mirror the enable flag onto the solver so its island rest-detection / LCP
  // skipping runs only when the feature is on (otherwise it is a strict no-op).
  mConstraintSolver->setDeactivationActive(deactivationEnabled);
  if (deactivationEnabled)
    wakeRestingSkeletonsIfStepStateChanged();
}

//==============================================================================
inline void World::stepWithoutDeactivation(const StepContext& context)
{
  const bool _resetCommand = context.resetCommand;

  {
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

  {
    mConstraintSolver->solve();
  }

  {
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
  ++mDeactivationState->mFramesSinceStart;
  invalidateAllRestingKinematicSnapshot();
  invalidateLastStepRestingWorldState();
}

//==============================================================================
inline bool World::tryAllRestingFastPath(StepContext& context)
{
  const bool _resetCommand = context.resetCommand;

  const bool customFilterSleeping = mDeactivationState->mStepCustomBodyFilter;
  context.customFilterSleeping = customFilterSleeping;
  const bool lastStepHadNoContacts
      = mConstraintSolver->getLastCollisionResult().getNumContacts() == 0;
  bool allRestingFastPathReady = false;
  if (lastStepHadNoContacts) {
    bool allRestingSnapshotStale = false;
    allRestingFastPathReady
        = isAllRestingFastPathReady(_resetCommand, &allRestingSnapshotStale);
    if (allRestingSnapshotStale)
      wakeRestingSkeletonsForWorldChange();
  }

  if (lastStepHadNoContacts && allRestingFastPathReady) {
    if (customFilterSleeping) {
      for (const auto& skel : mSkeletons) {
        if (skel->isMobile() && skel->isResting()) {
          engageSameThreadMarker(*mDeactivationState);
          clearRestingAccelerations(*skel);
        }
      }
    }
    mTime += mTimeStep;
    mFrame++;
    ++mDeactivationState->mFramesSinceStart;
    if (!mLastStepRestingWorldStateValid)
      updateLastStepRestingWorldState();
    return true;
  }

  return false;
}

//==============================================================================
inline void World::advanceVelocities(const StepContext& context)
{
  const bool _resetCommand = context.resetCommand;

  // Tracks, per skeleton, whether it was disturbed (woken or kept awake) this
  // step so the post-step rest-detection pass can treat it as non-quiet even
  // after _resetCommand clears the actuation vectors below. Disturbance
  // tracking is only needed while contact islands exist; unconstrained active
  // scenes cannot sleep.
  auto& disturbedThisStep = mDisturbedThisStepScratch;
  disturbedThisStep.clear();
  const bool trackDisturbances
      = mConstraintSolver->getLastCollisionResult().getNumContacts() > 0;
  {
    if (trackDisturbances)
      disturbedThisStep.assign(mSkeletons.size(), false);
  }

  // The first-frame dwell credit is only for bodies that start at rest (see
  // updateRestStates()), so note which skeletons start the first step moving.
  if (mDeactivationState->mFramesSinceStart == 0) {
    auto& initiallyMoving = mInitiallyMovingSkeletonScratch;
    initiallyMoving.assign(mSkeletons.size(), 0);
    for (std::size_t i = 0; i < mSkeletons.size(); ++i)
      initiallyMoving[i] = hasNonzeroGeneralizedVelocity(*mSkeletons[i]);
  }

  // Integrate velocity for unconstrained skeletons
  {
    parallelForIndexRange(
        mSimulationThreadPool.get(),
        mSkeletons.size(),
        mNumSimulationThreads,
        [&](std::size_t i) {
          auto& skel = mSkeletons[i];
          if (!skel->isMobile())
            return;

          const bool externallyDisturbed
              = (trackDisturbances || skel->isResting())
                && skel->hasExternalDisturbance();
          if (externallyDisturbed && !disturbedThisStep.empty())
            disturbedThisStep[i] = 1;

          // A resting skeleton skips forward dynamics (including gravity)
          // unless a user-applied force/command is pending this step, in which
          // case it is woken so the actuation is integrated rather than
          // silently dropped. Gravity alone is not a disturbance, so a body
          // resting on its support stays asleep.
          if (skel->isResting()) {
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
}

//==============================================================================
DART_FORCEINLINE void World::solveConstraints(StepContext& context)
{
  // Detect activated constraints and compute constraint impulses. The group
  // stamp also proves that contacts remain alive through this solve.
  // The context holds the group and filter until step() returns, as the
  // locals did before this stage existed.
  context.collisionGroup = mConstraintSolver->getCollisionGroup();
  const auto& collisionGroup = context.collisionGroup;
  const std::size_t collisionGroupVersion = collisionGroup->getContentVersion();
  context.collisionFilter
      = mConstraintSolver->getCollisionOption().collisionFilter;
  const auto& collisionFilter = context.collisionFilter;
  auto& deactivationState = *mDeactivationState;
  if (isReplayed(deactivationState, collisionFilter.get())
      && usesBuiltInContactSurfaceHandler(*mConstraintSolver)
      && (deactivationState.mFramesSinceStart < 2
          || mDeactivationOptions.mTimeUntilSleep <= mTimeStep
          || hasRestingOrCandidateMobileSkeleton(mSkeletons)
          || hasDwellingMobileSkeleton(mSkeletons))) {
    // Record before a body can freeze; moving bodies with no quiet dwell
    // need no replay record.
    const bool enableContact
        = mConstraintSolver->getCollisionOption().enableContact;
    const bool restingAtSolve = hasRestingMobileSkeleton();
    const RecorderDisarm disarm{
        armRecorder(deactivationState, collisionFilter.get())};
    const bool queryComplete
        = mConstraintSolver->solveWithQueryFilter(deactivationState.mRecorder);
    finishRecord(
        deactivationState,
        *mConstraintSolver,
        collisionFilter,
        collisionGroup,
        collisionGroupVersion,
        queryComplete,
        enableContact,
        restingAtSolve);
  } else {
    deactivationState.mRecordValid = false;
    mConstraintSolver->solve();
  }
  const bool collisionGroupUnchanged
      = mConstraintSolver->getCollisionGroup() == collisionGroup
        && collisionGroup->getContentVersion() == collisionGroupVersion;

  context.collisionGroupUnchanged = collisionGroupUnchanged;
}

//==============================================================================
inline void World::advancePositions(const StepContext& context)
{
  const bool _resetCommand = context.resetCommand;
  const bool customFilterSleeping = context.customFilterSleeping;
  auto& disturbedThisStep = mDisturbedThisStepScratch;

  {
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
          if (skel->isResting()) {
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
              if (customFilterSleeping)
                clearRestingAccelerations(*skel);
              return;
            }
          }

          if (skel->isImpulseApplied()) {
            skel->computeImpulseForwardDynamics();
            skel->setImpulseApplied(false);
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
            // Same per-DOF setter path as setVelocities(), without a vector.
            for (std::size_t dof = 0; dof < skel->getNumDofs(); ++dof)
              skel->setVelocity(dof, 0.0);
            if (customFilterSleeping)
              clearRestingAccelerations(*skel, true);
            else
              skel->computeForwardKinematics(false, true, false);
          }

          if (_resetCommand) {
            skel->clearInternalForces();
            skel->clearExternalForces();
            skel->resetCommands();
          }
        });
  }
}

//==============================================================================
inline void World::finishDeactivationStep(const StepContext& context)
{
  const bool _resetCommand = context.resetCommand;
  const bool customFilterSleeping = context.customFilterSleeping;
  const bool collisionGroupUnchanged = context.collisionGroupUnchanged;
  auto& disturbedThisStep = mDisturbedThisStepScratch;
  auto& deactivationState = *mDeactivationState;

  // Rest-detection pass: decide which mobile skeletons are quiet enough to
  // sleep, and wake any that have started moving again. This is a deterministic
  // function of post-step cached speeds, dwell time, and the configured
  // thresholds, so it does not depend on container or iteration order.
  if (mConstraintSolver->getLastCollisionResult().getNumContacts() > 0)
    updateRestStates(disturbedThisStep);

  mTime += mTimeStep;
  mFrame++;
  ++mDeactivationState->mFramesSinceStart;

  // A snapshot can only serve a next step on which a mobile body can freeze.
  if (mConstraintSolver->getLastCollisionResult().getNumContacts() == 0
      && hasRestingOrCandidateMobileSkeleton(mSkeletons)) {
    updateAllRestingKinematicSnapshot(_resetCommand);
  } else {
    invalidateAllRestingKinematicSnapshot();
  }

  updateLastStepRestingWorldState();

  updateRetainedContacts(
      deactivationState,
      *mConstraintSolver,
      mSkeletons,
      customFilterSleeping && collisionGroupUnchanged
          && hasRestingMobileSkeleton());
}

//==============================================================================
void World::updateRestStates(const std::vector<char>& disturbedThisStep)
{
  const bool customFilterSleeping = isCustomBodyNodeFilter(
      *mDeactivationState,
      mConstraintSolver->getCollisionOption().collisionFilter.get());
  // Joint reactions relax per contact solve even when motion is negligible;
  // a larger time step must not freeze them after fewer solves.
  // shortcut: tested decay only; use impulse convergence for slower islands.
  constexpr double kJointCoupledQuietSolves = 2000.0;
  const auto requiredDwell = [&](const dynamics::Skeleton& skeleton) {
    return customFilterSleeping && hasJointCoupledBodies(skeleton)
               ? std::max(
                   mDeactivationOptions.mTimeUntilSleep,
                   kJointCoupledQuietSolves * mTimeStep)
               : mDeactivationOptions.mTimeUntilSleep;
  };
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
  const double sleepContactPenetrationTolerance = constraint::ConstraintSolver::
      getAutomaticSleepingContactPenetrationTolerance();
  const bool useAdaptivePlaneSleepContactPenetrationTolerance
      = !constraint::ConstraintSolver::
            isAutomaticSleepingContactPenetrationToleranceUserConfigured()
        && sleepContactPenetrationTolerance
               == kDefaultSleepContactPenetrationTolerance;
  auto getSleepContactPenetrationTolerance
      = [&](const collision::Contact& contact) {
          return useAdaptivePlaneSleepContactPenetrationTolerance
                         && contactTouchesPlaneShape(contact)
                     ? kAdaptivePlaneSleepContactPenetrationTolerance
                     : sleepContactPenetrationTolerance;
        };
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
  // While a custom handler is installed, every quiet dwell stays 0: no body
  // becomes a candidate (every rest path needs candidacy), and removing the
  // handler restarts the full sleep delay (#3056).
  const bool canSleep = usesBuiltInContactSurfaceHandler(*mConstraintSolver)
                        && (!customFilterSleeping || ownsSolverSkeletons());
  constexpr double kSupportNormalMinVerticalComponent = 0.5;
  const auto& contacts = mConstraintSolver->getLastCollisionResult();
  const double gravityNorm = mGravity.norm();
  Eigen::Vector3d up = Eigen::Vector3d::Zero();
  if (gravityNorm > 0.0)
    up = -mGravity / gravityNorm;

  auto isShallowSupportContact = [&](const auto& bodyNode,
                                     const auto& supportBodyNode,
                                     const Eigen::Vector3d& normal,
                                     double contactSleepPenetrationTolerance) {
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
    return bodyHeightAboveSupport >= -contactSleepPenetrationTolerance;
  };

  // The first-frame dwell credit (see islandInInitialEquilibrium below) is
  // checked on the first solve and confirmed on the second.
  const int framesSinceStart = mDeactivationState->mFramesSinceStart;
  const bool confirmingInitialRest
      = framesSinceStart == 1
        && mInitialRestSpeedLimits.size() == countBodyNodes(mSkeletons);
  const bool checkingInitialRest
      = framesSinceStart == 0 || confirmingInitialRest;

  // Skeletons whose contacts on those frames cannot certify a settled body: a
  // deep contact, or a support tilted enough for gravity to drive motion along
  // it. A contact is deep beyond the strict tolerance even on a PlaneShape:
  // the adaptive one lets a settled body rest up to 5 mm into a plane, but a
  // body that starts that deep has yet to be pushed out.
  auto& unsettledInitialContactSkeletons
      = mUnsettledInitialContactSkeletonScratch;
  auto& supportedInitialContactSkeletons
      = mSupportedInitialContactSkeletonScratch;
  unsettledInitialContactSkeletons.clear();
  supportedInitialContactSkeletons.clear();
  const auto containsSkeleton
      = [](const std::vector<const dynamics::Skeleton*>& skeletons,
           const dynamics::Skeleton* skeleton) {
          return std::find(skeletons.begin(), skeletons.end(), skeleton)
                 != skeletons.end();
        };
  auto markMobileSkeleton = [&](auto bodyNode, auto& skeletons) {
    if (!bodyNode)
      return;
    const auto* skeleton = bodyNode->getSkeletonRawPtr();
    if (skeleton != nullptr && skeleton->isMobile()) {
      if (!containsSkeleton(skeletons, skeleton))
        skeletons.push_back(skeleton);
    }
  };
  auto clearMobileSleepCandidate = [](auto bodyNode) {
    if (!bodyNode)
      return;
    auto* skeleton
        = const_cast<dynamics::Skeleton*>(bodyNode->getSkeletonRawPtr());
    if (skeleton != nullptr && skeleton->isMobile()
        && skeleton->getIslandIndex() < 0) {
      skeleton->setSleepCandidate(false);
    }
  };
  if (checkingInitialRest) {
    for (std::size_t i = 0; i < contacts.getNumContacts(); ++i) {
      const auto& contact = contacts.getContact(i);
      const double contactSleepContactPenetrationTolerance
          = getSleepContactPenetrationTolerance(contact);
      const auto bodyNode1 = contact.getBodyNodePtr1();
      const auto bodyNode2 = contact.getBodyNodePtr2();
      if (contact.penetrationDepth > sleepContactPenetrationTolerance) {
        markMobileSkeleton(bodyNode1, unsettledInitialContactSkeletons);
        markMobileSkeleton(bodyNode2, unsettledInitialContactSkeletons);
        continue;
      }

      auto& initialSupportSkeletons = isTiltedSupportNormal(contact.normal, up)
                                          ? unsettledInitialContactSkeletons
                                          : supportedInitialContactSkeletons;
      if (isShallowSupportContact(
              bodyNode1,
              bodyNode2,
              contact.normal,
              contactSleepContactPenetrationTolerance)) {
        markMobileSkeleton(bodyNode1, initialSupportSkeletons);
      }
      if (isShallowSupportContact(
              bodyNode2,
              bodyNode1,
              contact.normal,
              contactSleepContactPenetrationTolerance)) {
        markMobileSkeleton(bodyNode2, initialSupportSkeletons);
      }
    }
  }

  std::size_t islandCount = 0;
  std::size_t islandedMobileSkeletonCount = 0;
  for (const auto& skel : mSkeletons) {
    if (!skel->isMobile())
      continue;

    const int island = skel->getIslandIndex();
    if (island >= 0) {
      ++islandedMobileSkeletonCount;
      islandCount = std::max(
          islandCount, static_cast<std::size_t>(island) + std::size_t{1});
    }
  }

  // The first-frame shortcut credits the full dwell only to bodies that are
  // essentially stationary at load time (pre-settled imported scenes). The
  // smoothed speeds are only alpha times the first solved speed, and one step
  // of gravity adds just |g| * dt, so a body that the first solve left
  // rolling, sliding, tipping, or dropping still looks quiet. The dense-island
  // rule below also makes a whole island sleep once one member has the full
  // dwell. So credit an island only if the first solve left every mobile
  // member in equilibrium (slower than kInitialRestSpeedRatio of the speed one
  // step of gravity adds; see computeInitialRestSpeeds()) and none started on
  // a deep or tilted contact.
  //
  // A large body whose center of mass is just past an edge passes that check:
  // a step of tipping adds less than 1% of |g| dt while the overhang is below
  // about 0.4% of its size. Its speed doubles on the next step, while that of a
  // settled body does not grow. So the credit waits for the second solve: the
  // island must still pass the check, and no speed of a member's body may have
  // grown by more than kInitialRestMaxSpeedGrowthRatio of |g| dt since the
  // first solve. That growth is the body's acceleration times dt only if the
  // body starts at rest: one that starts moving can slow down while gravity
  // accelerates it, as a body seeded to slide uphill does. So every member must
  // also start the first step at rest. Each body is compared with itself,
  // because a body that the first solve set moving at a steady speed, such as
  // one pushed out of a wall, would hide the growth of another body of its
  // skeleton behind the skeleton's largest speeds.
  //
  // Nor may a member have bodies coupled by a joint with degrees of freedom,
  // to each other or to the world. Its contacts can hold an internal force,
  // such as friction that a joint reaction balances, which the first solves
  // have only started to relax although nothing moves (3.6 N on a 1 kg flap
  // hinged to a base, decaying over about a second). A frozen island keeps
  // reporting the forces of the solve that froze it through
  // BodyNode::getBodyForce(), which gives the joint transmitted wrench, and
  // through its contacts, so such an island takes the normal dwell.
  auto& islandInInitialEquilibrium = mIslandInitialEquilibriumScratch;
  if (checkingInitialRest) {
    const double initialRestMaxSpeed
        = kInitialRestSpeedRatio * gravityNorm * mTimeStep;
    islandInInitialEquilibrium.assign(islandCount, 1);
    std::size_t numPrecedingBodyNodes = 0u;
    for (std::size_t i = 0; i < mSkeletons.size(); ++i) {
      const auto& skel = mSkeletons[i];
      const std::size_t firstBodyNode = numPrecedingBodyNodes;
      numPrecedingBodyNodes += skel->getNumBodyNodes();
      const int island = skel->isMobile() ? skel->getIslandIndex() : -1;
      if (island < 0)
        continue;

      const bool startedMoving = framesSinceStart == 0
                                 && i < mInitiallyMovingSkeletonScratch.size()
                                 && mInitiallyMovingSkeletonScratch[i];
      bool inEquilibrium
          = !startedMoving
            && !containsSkeleton(unsettledInitialContactSkeletons, skel.get())
            && !hasJointCoupledBodies(*skel);
      for (std::size_t j = 0; inEquilibrium && j < skel->getNumBodyNodes();
           ++j) {
        const Eigen::Vector3d speeds
            = computeInitialRestSpeeds(*skel->getBodyNode(j), up);
        inEquilibrium
            = (speeds.array() < initialRestMaxSpeed).all()
              && (!confirmingInitialRest
                  || (speeds.array()
                      <= mInitialRestSpeedLimits[firstBodyNode + j].array())
                         .all());
      }
      if (!inEquilibrium)
        islandInInitialEquilibrium[static_cast<std::size_t>(island)] = 0;
    }
  }

  // On the first solve, record the speeds that the second may leave on each
  // body of a member of an island in equilibrium; the bodies of a skeleton
  // outside such an island get limits that no speed meets.
  if (framesSinceStart == 0 && canSleep) {
    const double initialRestMaxSpeedGrowth
        = kInitialRestMaxSpeedGrowthRatio * gravityNorm * mTimeStep;
    mInitialRestSpeedLimits.clear();
    for (const auto& skel : mSkeletons) {
      const int island = skel->isMobile() ? skel->getIslandIndex() : -1;
      const bool inEquilibrium
          = island >= 0
            && islandInInitialEquilibrium[static_cast<std::size_t>(island)];
      for (std::size_t j = 0; j < skel->getNumBodyNodes(); ++j) {
        Eigen::Vector3d limits = Eigen::Vector3d::Constant(-1.0);
        if (inEquilibrium) {
          limits = computeInitialRestSpeeds(*skel->getBodyNode(j), up).array()
                   + initialRestMaxSpeedGrowth;
        }
        mInitialRestSpeedLimits.push_back(limits);
      }
    }
  }

  auto& islandMobileSkeletonCount = mIslandMobileSkeletonCountScratch;
  bool hasDenseContactIsland = false;
  if (islandCount > 0u
      && islandedMobileSkeletonCount >= kDenseContactJitterMinIslandSize
      && islandedMobileSkeletonCount != islandCount) {
    islandMobileSkeletonCount.assign(islandCount, 0u);
    for (const auto& skel : mSkeletons) {
      if (!skel->isMobile())
        continue;

      const int island = skel->getIslandIndex();
      if (island < 0)
        continue;

      const auto islandIndex = static_cast<std::size_t>(island);
      if (islandIndex < islandMobileSkeletonCount.size()) {
        ++islandMobileSkeletonCount[islandIndex];
        if (islandMobileSkeletonCount[islandIndex]
            >= kDenseContactJitterMinIslandSize) {
          hasDenseContactIsland = true;
        }
      }
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
      const bool needsJointSolveDwell
          = customFilterSleeping && hasJointCoupledBodies(*skel);
      const bool canAccumulateDwell
          = islanded
            || (!needsJointSolveDwell
                && (skel->isSleepCandidate()
                    || skel->getRestDwellTime() > 0.0));
      const bool quiet = canSleep && canAccumulateDwell && (linSpeed < linSleep)
                         && (angSpeed < angSleep) && !disturbed;
      if (quiet) {
        const bool finalQuiet = linSpeed < finalSleepLinearSpeed
                                && angSpeed < finalSleepAngularSpeed;
        double dwell = skel->getRestDwellTime() + mTimeStep;
        const bool supportedInitialContact
            = containsSkeleton(supportedInitialContactSkeletons, skel.get());
        // The first-frame credit (see islandInInitialEquilibrium) deliberately
        // keeps these near-zero fixed bounds instead of the threshold-scaled
        // candidacy gate above, so raising the thresholds cannot skip the
        // configured dwell for a supported body with real initial motion.
        constexpr double kInitialRestMaxLinearSpeed = 1e-3;
        constexpr double kInitialRestMaxAngularSpeed = 1e-2;
        const bool initialRestQuiet = linSpeed < kInitialRestMaxLinearSpeed
                                      && angSpeed < kInitialRestMaxAngularSpeed;
        if (confirmingInitialRest && islanded && initialRestQuiet
            && supportedInitialContact
            && islandInInitialEquilibrium[static_cast<std::size_t>(
                skel->getIslandIndex())]) {
          dwell = std::max(dwell, mDeactivationOptions.mTimeUntilSleep);
        }
        skel->setRestDwellTime(dwell);
        bool denseContactIsland = false;
        if (hasDenseContactIsland && islanded) {
          const auto islandIndex
              = static_cast<std::size_t>(skel->getIslandIndex());
          denseContactIsland = islandIndex < islandMobileSkeletonCount.size()
                               && islandMobileSkeletonCount[islandIndex]
                                      >= kDenseContactJitterMinIslandSize;
        }
        if (!denseContactIsland && dwell >= requiredDwell(*skel)
            && finalQuiet) {
          skel->setSleepCandidate(true);
        }
      } else {
        skel->setRestDwellTime(0.0);
      }
    }
  }

  if (hasDenseContactIsland) {
    auto& islandHasMobileSkeleton = mIslandHasMobileSkeletonScratch;
    auto& islandAllFinalSleepCandidateReady
        = mIslandAllFinalSleepCandidateReadyScratch;
    auto& islandAllBelowWake = mIslandAllBelowWakeScratch;
    auto& islandDwellWakeReadyCount = mIslandDwellWakeReadyCountScratch;
    islandHasMobileSkeleton.assign(islandCount, 0);
    islandAllFinalSleepCandidateReady.assign(islandCount, 1);
    islandAllBelowWake.assign(islandCount, 1);
    islandDwellWakeReadyCount.assign(islandCount, 0u);
    auto& islandJointDwellReady = mDeactivationState->mIslandJointDwellReady;
    if (customFilterSleeping)
      islandJointDwellReady.assign(islandCount, 1);

    for (std::size_t i = 0; i < mSkeletons.size(); ++i) {
      const auto& skel = mSkeletons[i];
      if (!skel->isMobile())
        continue;

      const int island = skel->getIslandIndex();
      if (island < 0)
        continue;

      const auto islandIndex = static_cast<std::size_t>(island);
      if (islandIndex >= islandCount)
        continue;
      if (islandMobileSkeletonCount[islandIndex]
          < kDenseContactJitterMinIslandSize) {
        continue;
      }

      const bool disturbed
          = (i < disturbedThisStep.size() && disturbedThisStep[i])
            || skel->hasExternalDisturbance();
      const bool belowWake = !disturbed
                             && skel->getSmoothedLinearSpeed() < linWake
                             && skel->getSmoothedAngularSpeed() < angWake;
      const bool finalReady
          = !disturbed
            && (skel->isSleepCandidate()
                || (skel->getRestDwellTime() >= requiredDwell(*skel)
                    && skel->getSmoothedLinearSpeed() < finalSleepLinearSpeed
                    && skel->getSmoothedAngularSpeed()
                           < finalSleepAngularSpeed));
      if (customFilterSleeping && hasJointCoupledBodies(*skel)
          && skel->getRestDwellTime() < requiredDwell(*skel)) {
        islandJointDwellReady[islandIndex] = 0;
      }
      islandHasMobileSkeleton[islandIndex] = 1;
      islandAllFinalSleepCandidateReady[islandIndex]
          = islandAllFinalSleepCandidateReady[islandIndex] && finalReady;
      islandAllBelowWake[islandIndex]
          = islandAllBelowWake[islandIndex] && belowWake;
      if (belowWake && skel->getRestDwellTime() >= requiredDwell(*skel)) {
        ++islandDwellWakeReadyCount[islandIndex];
      }
    }

    for (const auto& skel : mSkeletons) {
      if (!skel->isMobile())
        continue;

      const int island = skel->getIslandIndex();
      if (island < 0)
        continue;

      const auto islandIndex = static_cast<std::size_t>(island);
      if (islandIndex >= islandCount || !islandHasMobileSkeleton[islandIndex])
        continue;
      if (islandMobileSkeletonCount[islandIndex]
          < kDenseContactJitterMinIslandSize) {
        continue;
      }

      const bool finalReady = islandAllFinalSleepCandidateReady[islandIndex];
      const bool denseContactJitterReady
          = islandMobileSkeletonCount[islandIndex]
                >= kDenseContactJitterMinIslandSize
            && islandAllBelowWake[islandIndex]
            && islandDwellWakeReadyCount[islandIndex] > 0u;
      if ((finalReady || denseContactJitterReady)
          && (!customFilterSleeping || islandJointDwellReady[islandIndex])) {
        skel->setSleepCandidate(true);
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
    const double contactSleepContactPenetrationTolerance
        = getSleepContactPenetrationTolerance(contact);
    const auto bodyNode1 = contact.getBodyNodePtr1();
    const auto bodyNode2 = contact.getBodyNodePtr2();
    if (contact.penetrationDepth > contactSleepContactPenetrationTolerance) {
      clearMobileSleepCandidate(bodyNode1);
      clearMobileSleepCandidate(bodyNode2);
      continue;
    }

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

      if (!isShallowSupportContact(
              bodyNode,
              supportBodyNode,
              contact.normal,
              contactSleepContactPenetrationTolerance)) {
        return;
      }

      if (skel->getSmoothedLinearSpeed() > linWake
          || skel->getSmoothedAngularSpeed() > angWake) {
        skel->setSleepCandidate(false);
        return;
      }

      skel->setIslandIndex(0);
      skel->setResting(true);
      if (customFilterSleeping)
        clearRestingAccelerations(*skel);
    };

    tryRestOnInactiveSupport(bodyNode1, bodyNode2);
    tryRestOnInactiveSupport(bodyNode2, bodyNode1);
  }

  constexpr double kZeroSpeedForContactMissSleep = 1e-10;
  for (auto& skel : mSkeletons) {
    if (!skel->isMobile() || skel->isResting() || !skel->isSleepCandidate()
        || skel->getIslandIndex() >= 0) {
      continue;
    }

    if (skel->getRestDwellTime() < requiredDwell(*skel))
      continue;

    if (skel->computeMaxBodyLinearSpeed() > kZeroSpeedForContactMissSleep
        || skel->computeMaxBodyAngularSpeed() > kZeroSpeedForContactMissSleep) {
      continue;
    }

    skel->setIslandIndex(0);
    skel->setResting(true);
    if (customFilterSleeping)
      clearRestingAccelerations(*skel);
  }
}

//==============================================================================
bool World::isAllRestingFastPathReady(bool _resetCommand, bool* snapshotStale)
{
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

  // Only a solve evaluates manual constraints, so the fast path is refused
  // while any exist; one added between resting bodies then reaches the
  // solver, whose island pass wakes the bodies. The pose validation below
  // still runs, so an edit on such a step still wakes the world. Worlds with
  // a permanently inactive manual constraint therefore lose the fast path;
  // snapshotting the constraint list would restore it.
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
  const bool nativeFilter
      = collisionFilter == nullptr
        || typeid(*collisionFilter)
               == typeid(collision::BodyNodeCollisionFilter);
  if (!nativeFilter && !isCollisionFilterSnapshotTrackable(collisionFilter)
      && !(
          isReplayed(*mDeactivationState, collisionFilter)
          && mDeactivationState->mReplayVerified)) {
    invalidateAllRestingKinematicSnapshot();
    return false;
  }

  const auto collisionFilterRevision
      = nativeFilter
            ? (collisionFilter
                   ? static_cast<const collision::BodyNodeCollisionFilter*>(
                         collisionFilter)
                         ->getBodyNodePairBlackListRevision()
                   : 0u)
            : getCollisionFilterSnapshotRevision(collisionFilter);
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

  // Another World's changes move only the global version. If none of this
  // World's skeletons changed, catch the record up so that such a change
  // alone does not force the per-skeleton validation below.
  const bool deactivationStateChanged
      = dynamics::Skeleton::getGlobalDeactivationStateVersion()
            != mAllRestingSnapshotDeactivationStateVersion
        && hasDeactivationStateChangedSince(
            *collisionGroup, mAllRestingSnapshotDeactivationStateVersion);
  if (!deactivationStateChanged) {
    mAllRestingSnapshotDeactivationStateVersion
        = dynamics::Skeleton::getGlobalDeactivationStateVersion();
  }

  if (mAllRestingSnapshotReady && mAllRestingSnapshotHasMobileSkeleton
      && mAllRestingSnapshotStructuralVersion
             == dynamics::Skeleton::getGlobalStructuralVersion()
      && mAllRestingSnapshotKinematicVersion
             == dynamics::Skeleton::getGlobalKinematicVersion()
      && mAllRestingSnapshotExternalDisturbanceVersion
             == dynamics::Skeleton::getGlobalExternalDisturbanceVersion()
      && !deactivationStateChanged
      && mAllRestingSnapshotVelocityVersion
             == dynamics::Skeleton::getGlobalVelocityVersion()
      && collisionDetectorUnchanged && collisionFilterUnchanged
      && collisionOptionScalarsUnchanged) {
    if (_resetCommand && !mAllRestingSnapshotResetCommand) {
      // Resetting forces and commands reads and resets custom Joint overrides.
      engageSameThreadMarker(*mDeactivationState);
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

    return mConstraintSolver->getNumConstraints() == 0u;
  }

  if (!collisionDetectorUnchanged || !collisionFilterUnchanged
      || !collisionOptionScalarsUnchanged) {
    markSnapshotStale();
    return false;
  }

  if (deactivationStateChanged) {
    markSnapshotStale();
    return false;
  }

  // Slow pose validation can evaluate custom Joint kinematics.
  engageSameThreadMarker(*mDeactivationState);
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

    // A skeleton without bodies cannot disturb a contact island.
    if (!skel->isMobile() || skel->getNumBodyNodes() == 0u)
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

  return hasMobileSkeleton && mConstraintSolver->getNumConstraints() == 0u;
}

//==============================================================================
void World::updateAllRestingKinematicSnapshot(bool _resetCommand)
{
  const auto* collisionFilter
      = mConstraintSolver->getCollisionOption().collisionFilter.get();
  if (!isCollisionFilterSnapshotTrackable(collisionFilter)
      && !isReplayed(*mDeactivationState, collisionFilter)) {
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

  return typeid(*filter) == typeid(collision::BodyNodeCollisionFilter)
         || dynamic_cast<
                const collision::detail::CollisionFilterSnapshotTracker*>(
                filter)
                != nullptr;
}

//==============================================================================
std::size_t World::getCollisionFilterSnapshotRevision(
    const collision::CollisionFilter* filter) const
{
  if (filter && typeid(*filter) == typeid(collision::BodyNodeCollisionFilter)) {
    return static_cast<const collision::BodyNodeCollisionFilter*>(filter)
        ->getBodyNodePairBlackListRevision();
  }

  const auto* snapshotTracker
      = dynamic_cast<const collision::detail::CollisionFilterSnapshotTracker*>(
          filter);
  if (snapshotTracker != nullptr) {
    if (mDeactivationState->mStepping.load(std::memory_order_acquire))
      engageSameThreadMarker(*mDeactivationState);
    return snapshotTracker->getCollisionFilterSnapshotRevision();
  }

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
bool World::hasDeactivationStateChangedSince(
    const collision::CollisionGroup& collisionGroup,
    std::size_t globalVersion) const
{
  // Each change stamps its skeleton with the global version it produced, so a
  // change made after globalVersion was read left a stamp in (globalVersion,
  // current]. Comparing offsets from globalVersion survives a counter wrap.
  const std::size_t window
      = dynamics::Skeleton::getGlobalDeactivationStateVersion() - globalVersion;
  if (window == 0u)
    return false;

  const bool customFilterSleeping = isCustomBodyNodeFilter(
      *mDeactivationState,
      mConstraintSolver->getCollisionOption().collisionFilter.get());
  const auto changed = [&](const dynamics::Skeleton* skel) {
    if (skel == nullptr
        || skel->mDeactivationStateVersion - globalVersion - 1u >= window) {
      return false;
    }
    // Active material writes cannot invalidate another island's sleep. A ready
    // cache must still notice a static support becoming mobile.
    return !customFilterSleeping || mAllRestingSnapshotReady
           || !skel->isMobile() || skel->isResting() || skel->isSleepCandidate()
           || skel->getRestDwellTime() > 0.0;
  };
  for (const auto& skel : mSkeletons) {
    if (changed(skel.get()))
      return true;
  }

  // A support may be registered only with the constraint solver, or only in
  // its collision group.
  for (const auto& skel : mConstraintSolver->getSkeletons()) {
    if (changed(skel.get()))
      return true;
  }

  for (std::size_t i = 0; i < collisionGroup.getNumShapeFrames(); ++i) {
    if (changed(dynamics::Skeleton::getSkeletonOf(
            *collisionGroup.getShapeFrame(i)))) {
      return true;
    }
  }

  return false;
}

//==============================================================================
void World::wakeRestingSkeletonsIfStepStateChanged()
{
  auto& deactivationState = *mDeactivationState;
  deactivationState.mReplayVerified = false;
  const auto& collisionOption = mConstraintSolver->getCollisionOption();
  const auto* collisionFilter = collisionOption.collisionFilter.get();
  // Classify once per step. The default filter needs only the same exact-type
  // check as snapshot tracking; World::step reuses the custom-filter decision.
  const bool nativeFilter
      = collisionFilter == nullptr
        || typeid(*collisionFilter)
               == typeid(collision::BodyNodeCollisionFilter);
  bool collisionFilterTrackable = nativeFilter;
  bool collisionFilterReplayed = false;
  deactivationState.mStepCustomBodyFilter = false;
  if (!collisionFilterTrackable) {
    const auto* tracker = dynamic_cast<
        const collision::detail::CollisionFilterSnapshotTracker*>(
        collisionFilter);
    collisionFilterTrackable = tracker != nullptr;
    deactivationState.mStepCustomBodyFilter
        = dynamic_cast<const collision::BodyNodeCollisionFilter*>(
              collisionFilter)
              != nullptr
          && (deactivationState.mCustomFilterSleeping || tracker != nullptr);
    collisionFilterReplayed
        = deactivationState.mStepCustomBodyFilter && tracker == nullptr;
  }
  // A sleep candidate is checked like a resting body: the next solve may
  // freeze it on evidence gathered before the change.
  const bool hasRestingOrCandidate
      = hasRestingOrCandidateMobileSkeleton(mSkeletons);
  // No sleep evidence exists to invalidate in a moving, contact-free world.
  if (!hasRestingOrCandidate && !hasDwellingMobileSkeleton(mSkeletons)
      && mLastStepRestingWorldSkeletonStates.empty()
      && mInitialRestSpeedLimits.empty()) {
    return;
  }
  const bool restingOrCandidate
      = mLastStepRestingWorldStateValid && hasRestingOrCandidate;
  const auto collisionDetector = mConstraintSolver->getCollisionDetector();
  const auto collisionGroup = mConstraintSolver->getCollisionGroup();
  // Before the no-contact kinematic snapshot exists, this guard is the only
  // place that can notice support pose edits made between the sleep transition
  // and the next step. Once the full snapshot exists, let its exact
  // pose/transform validation distinguish real kinematic edits from visual-only
  // version changes. Skeletons are recorded only while something can use them
  // (see updateLastStepRestingWorldState()).
  bool skeletonStateUnchanged = true;
  if (!mLastStepRestingWorldSkeletonStates.empty()
      && (!restingOrCandidate || !mAllRestingKinematicSnapshotValid)) {
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

  // Another World's changes move only the global version. If none of this
  // World's skeletons changed, catch the record up so that the next steps
  // compare in O(1) again.
  const bool deactivationStateChanged
      = dynamics::Skeleton::getGlobalDeactivationStateVersion()
            != mLastStepRestingWorldStateDeactivationStateVersion
        && hasDeactivationStateChangedSince(
            *collisionGroup,
            mLastStepRestingWorldStateDeactivationStateVersion);
  if (!deactivationStateChanged) {
    mLastStepRestingWorldStateDeactivationStateVersion
        = dynamics::Skeleton::getGlobalDeactivationStateVersion();
  }

  bool recordedStateUnchanged
      = skeletonStateUnchanged && !deactivationStateChanged
        && (!deactivationState.mStepCustomBodyFilter || ownsSolverSkeletons())
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
               == (nativeFilter
                       ? (collisionFilter
                              ? static_cast<
                                    const collision::BodyNodeCollisionFilter*>(
                                    collisionFilter)
                                    ->getBodyNodePairBlackListRevision()
                              : 0u)
                       : getCollisionFilterSnapshotRevision(collisionFilter));

  if (recordedStateUnchanged && collisionFilterReplayed
      && !mLastStepRestingWorldSkeletonStates.empty()
      && (restingOrCandidate || deactivationState.mRecordValid)) {
    deactivationState.mReplayVerified
        = replayRecord(deactivationState, *mConstraintSolver);
    recordedStateUnchanged = deactivationState.mReplayVerified;
  }

  if (!restingOrCandidate) {
    // Dwell rule: a between-step change to this World (a relaxed joint limit,
    // a spring edit, a moved support, a collision filter edit) restarts the
    // quiet dwell of awake non-candidates. Otherwise dwell gathered under the
    // old dynamics lets the next rest pass grant candidacy after one smoothed
    // step of the new motion, and the following solve freezes the body
    // (#3056). Candidacy set by hand between steps is kept. Another World's
    // sleep transitions and deactivation-state edits stamp none of the
    // skeletons checked here, so they do not restart it.
    if (!recordedStateUnchanged) {
      mInitialRestSpeedLimits.clear();
      for (auto& skel : mSkeletons) {
        if (skel->isMobile() && !skel->isResting() && !skel->isSleepCandidate())
          skel->setRestDwellTime(0.0);
      }
    }
    return;
  }

  // Adding a custom contact surface handler wakes resting bodies and clears
  // candidacy; see usesBuiltInContactSurfaceHandler().
  const bool worldStateUnchanged
      = recordedStateUnchanged
        && usesBuiltInContactSurfaceHandler(*mConstraintSolver)
        && mLastStepRestingWorldStateCollisionFilterTrackable
        && (collisionFilterTrackable || collisionFilterReplayed);

  if (!worldStateUnchanged)
    wakeRestingSkeletonsForWorldChange();
}

//==============================================================================
void World::updateLastStepRestingWorldState()
{
  // The next step start compares the world with this record: in full while a
  // mobile skeleton rests or is a sleep candidate, and for the dwell rule
  // otherwise.
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
            mLastStepRestingWorldStateCollisionFilter)
        || isReplayed(
            *mDeactivationState, mLastStepRestingWorldStateCollisionFilter);
  mLastStepRestingWorldStateCollisionFilterRevision
      = mLastStepRestingWorldStateCollisionFilterTrackable
            ? getCollisionFilterSnapshotRevision(
                mLastStepRestingWorldStateCollisionFilter)
            : 0u;
  mLastStepRestingWorldSkeletonStates.clear();
  const bool restingOrCandidate
      = hasRestingOrCandidateMobileSkeleton(mSkeletons);
  // Skeletons matter only while a body rests, is a candidate, or has quiet
  // dwell that a between-step edit must restart. Entering simulation mode
  // reserves the buffer (reserveSimulationScratch()), so recording allocates
  // nothing.
  if (restingOrCandidate || hasDwellingMobileSkeleton(mSkeletons)) {
    for (const auto& skel : mSkeletons) {
      mLastStepRestingWorldSkeletonStates.push_back(RestingWorldSkeletonState{
          skel.get(),
          skel->getVersion(),
          skel->getKinematicVersion(),
          skel->getNumBodyNodes()});
    }
  }
  mLastStepRestingWorldStateDeactivationStateVersion
      = dynamics::Skeleton::getGlobalDeactivationStateVersion();
  mLastStepRestingWorldStateValid = restingOrCandidate;
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
void World::restartFirstFrameRestCredit()
{
  mDeactivationState->mFramesSinceStart = 0;
}

//==============================================================================
void World::wakeRestingSkeletonsForWorldChange()
{
  mInitialRestSpeedLimits.clear();
  for (auto& skel : mSkeletons) {
    if (!skel->isMobile())
      continue;

    if (skel->isResting()) {
      skel->setResting(false);
      skel->setIslandIndex(-1);
    }
    // Clears candidacy and restarts the quiet dwell (setSleepCandidate(false)
    // zeroes it), so nothing freezes on evidence gathered before the change
    // (#3056). Bumps the version only for a real candidate.
    skel->setSleepCandidate(false);
  }

  invalidateAllRestingKinematicSnapshot();
  invalidateLastStepRestingWorldState();
  clearRetainedContacts(*mDeactivationState);
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
  // A host may re-apply an unchanged gravity before every step (gz-sim 10
  // does); that must not wake resting bodies (#3056). A skeleton whose gravity
  // differs, for example after a direct Skeleton::setGravity(), is still
  // synced, which wakes them as before.
  bool changed = mGravity != _gravity;
  mGravity = _gravity;
  for (auto& skel : mSkeletons) {
    if (skel->getGravity() != _gravity) {
      skel->setGravity(_gravity);
      changed = true;
    }
  }

  if (!changed)
    return;

  ++mDeactivationState->mParameterRevision;
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

  checkModelMutation(*mDeactivationState);
  common::detail::checkTopologyMutation(_skeleton.get());
  if (!mDeactivationState->mIndexed) {
    mSkeletons.push_back(_skeleton);
  } else {
    auto& state = *mDeactivationState;
    Rejection reason = Rejection::None;
    {
      const auto lock = lockRegistry();
      if (state.mCompiled || state.mEditDepth) {
        const auto& owners = modelOwners().mOwners;
        const auto it = owners.find(_skeleton.get());
        if (it != owners.end()
            && std::any_of(
                it->second.begin(), it->second.end(), [&](const auto* owner) {
                  return owner != &state;
                }))
          reason = Rejection::Shared;
      }
      if (reason == Rejection::None) {
        // Index first, so a failed allocation leaves the World unchanged.
        addOwner(state.mModelKeys, state, _skeleton.get());
        try {
          mSkeletons.push_back(_skeleton);
        } catch (...) {
          removeOwner(state.mModelKeys, state, _skeleton.get());
          throw;
        }
      }
    }
    throwRejection(reason);
  }
  mDeactivationState->mSolverSkeletonOwnershipValid = false;
  mMapForSkeletons[_skeleton] = _skeleton;

  mNameConnectionsForSkeletons.push_back(_skeleton->onNameChanged.connect(
      [this](
          dynamics::ConstMetaSkeletonPtr skel,
          const std::string&,
          const std::string&) { this->handleSkeletonNameChange(skel); }));

  _skeleton->setName(
      mNameMgrForSkeletons.issueNewNameAndAdd(_skeleton->getName(), _skeleton));

  _skeleton->setTimeStep(mTimeStep);
  _skeleton->setGravity(mGravity);

  refreshSkeletonDofIndices();
  mConstraintSolver->addSkeleton(_skeleton);
  invalidateAllRestingKinematicSnapshot();
  wakeRestingSkeletonsForWorldChange();
  invalidateSimulationMode();

  // Update recording
  mRecording->updateNumGenCoords(mSkeletons);

  notifyModelTopology(*mDeactivationState);
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

  // Remove _skeleton from constraint handler.
  checkModelMutation(*mDeactivationState);
  mConstraintSolver->removeSkeleton(_skeleton);
  invalidateAllRestingKinematicSnapshot();
  wakeRestingSkeletonsForWorldChange();

  // Remove _skeleton from mSkeletons
  if (!mDeactivationState->mIndexed) {
    mSkeletons.erase(
        remove(mSkeletons.begin(), mSkeletons.end(), _skeleton),
        mSkeletons.end());
  } else {
    const auto lock = lockRegistry();
    mSkeletons.erase(
        remove(mSkeletons.begin(), mSkeletons.end(), _skeleton),
        mSkeletons.end());
    removeOwner(
        mDeactivationState->mModelKeys, *mDeactivationState, _skeleton.get());
  }
  mDeactivationState->mSolverSkeletonOwnershipValid = false;
  refreshSkeletonDofIndices();
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
  notifyModelTopology(*mDeactivationState);
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
  if (_index < 0 || static_cast<std::size_t>(_index) > mSkeletons.size()) {
    dterr << "[World::getIndex] Index [" << _index << "] is out of range. "
          << "Valid range is [0, " << mSkeletons.size() << "].\n";
    DART_ASSERT(false);
    return -1;
  }
  // Sum the current DOF counts instead of reading mIndices so the result
  // tracks joint/body topology changes made after a skeleton was added.
  // This is O(n) in the number of skeletons; if it ever shows up in a
  // profile, cache it keyed on Skeleton::getGlobalStructuralVersion().
  int dofIndex = 0;
  for (int i = 0; i < _index; ++i)
    dofIndex += static_cast<int>(mSkeletons[i]->getNumDofs());
  return dofIndex;
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

  checkModelMutation(*mDeactivationState);
  common::detail::checkTopologyMutation(_frame.get());
  if (!mDeactivationState->mIndexed) {
    mSimpleFrames.push_back(_frame);
  } else {
    ModelKeys keys;
    collectFrameKeys(_frame.get(), keys, true);
    auto& state = *mDeactivationState;
    const auto lock = lockRegistry();
    // Index first, so a failed allocation leaves the World unchanged.
    registerChain(state.mSimpleFrameChains, state, _frame.get(), keys);
    try {
      mSimpleFrames.push_back(_frame);
    } catch (...) {
      unregisterChain(state.mSimpleFrameChains, state, _frame.get());
      throw;
    }
  }
  mSimpleFrameToShared[_frame.get()] = _frame;

  mNameConnectionsForSimpleFrames.push_back(_frame->onNameChanged.connect(
      [this](
          const dynamics::Entity* _entity,
          const std::string&,
          const std::string&) { this->handleSimpleFrameNameChange(_entity); }));

  _frame->setName(
      mNameMgrForSimpleFrames.issueNewNameAndAdd(_frame->getName(), _frame));

  invalidateSimulationMode();

  notifyModelTopology(*mDeactivationState);
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
  checkModelMutation(*mDeactivationState);
  if (!mDeactivationState->mIndexed) {
    mSimpleFrames.erase(mSimpleFrames.begin() + index);
  } else {
    const auto lock = lockRegistry();
    mSimpleFrames.erase(mSimpleFrames.begin() + index);
    unregisterChain(
        mDeactivationState->mSimpleFrameChains,
        *mDeactivationState,
        _frame.get());
  }

  // Disconnect the name change monitor
  mNameConnectionsForSimpleFrames[index].disconnect();
  mNameConnectionsForSimpleFrames.erase(
      mNameConnectionsForSimpleFrames.begin() + index);

  // Remove from NameManager
  mNameMgrForSimpleFrames.removeName(_frame->getName());

  // Remove from the pointer map
  mSimpleFrameToShared.erase(_frame.get());

  invalidateSimulationMode();
  notifyModelTopology(*mDeactivationState);
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
  auto& state = *mDeactivationState;
  const auto& solved = mConstraintSolver->getLastCollisionResult();
  if (!mDeactivationOptions.mEnabled
      || !isCustomBodyNodeFilter(
          state, mConstraintSolver->getCollisionOption().collisionFilter.get())
      || !isRetentionAlive(state, *mConstraintSolver)) {
    return solved;
  }

  std::lock_guard<std::mutex> lock(state.mViewMutex);
  if (state.mViewDirty) {
    auto& view = state.mView;
    view.clear();
    for (const auto& contact : solved.getContacts()) {
      if (!containsPair(state.mRetainedPairs, makePairKey(contact)))
        view.addContact(contact);
    }
    for (const auto& contact : state.mRetainedContacts)
      view.addContact(contact);
    state.mViewDirty = false;
  }
  return state.mView;
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

  const auto* previousCollisionGroup
      = mConstraintSolver->getCollisionGroup().get();
  if (mConstraintSolver->getCollisionDetector() != collisionDetector)
    checkModelMutation(*mDeactivationState);
  mConstraintSolver->setCollisionDetector(collisionDetector);
  if (mConstraintSolver->getCollisionGroup().get() != previousCollisionGroup) {
    invalidateSimulationMode();
    notifyModelTopology(*mDeactivationState);
  }
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
bool World::ownsSolverSkeletons()
{
  const auto& solverSkeletons = mConstraintSolver->getSkeletons();
  if (solverSkeletons.size() != mSkeletons.size())
    return false;

  auto& state = *mDeactivationState;
  const auto group = mConstraintSolver->getCollisionGroup();
  const auto version = group->getContentVersion();
  // Shapeless skeletons cannot be contact supports: a swap that changes neither
  // count nor group content cannot change the support-motion hazard. World list
  // edits invalidate this cache at insertion/removal.
  if (!state.mSolverSkeletonOwnershipValid
      || state.mSolverSkeletonCount != solverSkeletons.size()
      || state.mSolverSkeletonGroup.lock() != group
      || state.mSolverSkeletonGroupVersion != version) {
    ++state.mSolverSkeletonOwnershipCheckCount;
    // World snapshots cover only its own skeletons, regardless of solver order.
    state.mOwnsSolverSkeletons
        = solverSkeletons == mSkeletons
          || std::all_of(
              solverSkeletons.begin(),
              solverSkeletons.end(),
              [&](const auto& skel) {
                return mMapForSkeletons.find(skel) != mMapForSkeletons.end();
              });
    // Direct group additions can introduce supports outside World snapshots.
    for (std::size_t i = 0;
         state.mOwnsSolverSkeletons && i < group->getNumShapeFrames();
         ++i) {
      const auto* shapeNode = group->getShapeFrame(i)->asShapeNode();
      state.mOwnsSolverSkeletons
          = shapeNode
            && mMapForSkeletons.find(shapeNode->getSkeleton())
                   != mMapForSkeletons.end();
    }
    state.mSolverSkeletonCount = solverSkeletons.size();
    state.mSolverSkeletonGroup = group;
    state.mSolverSkeletonGroupVersion = version;
    state.mSolverSkeletonOwnershipValid = true;
  }
  return state.mOwnsSolverSkeletons;
}

//==============================================================================
std::size_t World::getSolverSkeletonOwnershipCheckCount() const
{
  return mDeactivationState->mSolverSkeletonOwnershipCheckCount;
}

//==============================================================================
void World::setConstraintSolver(constraint::UniqueConstraintSolverPtr solver)
{
  if (!solver) {
    dtwarn << "[World::setConstraintSolver] nullptr for constraint solver is "
           << "not allowed. Doing nothing.";
    return;
  }

  checkModelMutation(*mDeactivationState);
  const auto* incoming = dynamic_cast<const collision::DARTCollisionDetector*>(
      solver->getCollisionDetector().get());
  if (incoming && incoming->getNumCollisionThreads() != mNumSimulationThreads)
    collision::detail::checkWorldDetectorTopologyMutation(incoming);
  if (mConstraintSolver)
    solver->setFromOtherConstraintSolver(*mConstraintSolver);

  solver->setTimeStep(mTimeStep);
  solver->setNumSimulationThreads(mNumSimulationThreads);
  mConstraintSolver = std::move(solver);
  if (mDeactivationState->mIndexed)
    reindexWorld(*mDeactivationState);
  mDeactivationState->mSolverSkeletonOwnershipValid = false;
  invalidateAllRestingKinematicSnapshot();
  wakeRestingSkeletonsForWorldChange();
  invalidateSimulationMode();
  notifyModelTopology(*mDeactivationState);
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
  // Re-applying the current options changes nothing, so it keeps resting
  // bodies asleep and the sleep delay that awake bodies have accumulated.
  const DeactivationOptions& current = mDeactivationOptions;
  if (options.mEnabled == current.mEnabled
      && options.mLinearSpeedThreshold == current.mLinearSpeedThreshold
      && options.mAngularSpeedThreshold == current.mAngularSpeedThreshold
      && options.mTimeUntilSleep == current.mTimeUntilSleep
      && options.mWakeThresholdScale == current.mWakeThresholdScale) {
    return;
  }

  mDeactivationOptions = options;
  ++mDeactivationState->mParameterRevision;
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
  // A skeleton already in the world may have gained or lost joints since the
  // recording's layout was last refreshed by addSkeleton/removeSkeleton.
  mRecording->updateNumGenCoords(mSkeletons);

  const auto& collisionResult = getLastCollisionResult();
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

namespace dart {
namespace common {
namespace detail {

simulation::WorldDeactivationState* getWorldStepContext()
{
  return simulation::tlsSteppingWorld;
}

ScopedWorldStepContext::ScopedWorldStepContext(
    simulation::WorldDeactivationState* context)
  : mPrevious(simulation::tlsSteppingWorld)
{
  simulation::tlsSteppingWorld = context;
}

ScopedWorldStepContext::~ScopedWorldStepContext()
{
  simulation::tlsSteppingWorld = mPrevious;
}

namespace {

template <class Object>
DART_REGISTRY_NOINLINE void checkTopologySlow(const Object* object)
{
  simulation::ModelKeys keys;
  simulation::collectModelKeys(object, keys);
  if (keys.empty())
    return;
  simulation::throwRejection(simulation::lazyStepRejection(keys));
  if (simulation::noIndexedWorlds())
    return;
  simulation::RejectionCollector rejection;
  {
    const auto lock = simulation::lockRegistry();
    simulation::forEachOwner(keys, std::ref(rejection));
  }
  simulation::throwRejection(rejection.mReason);
}

template <class Object>
DART_REGISTRY_NOINLINE void notifyTopologySlow(const Object* object)
{
  simulation::ModelKeys keys;
  simulation::collectModelKeys(object, keys);
  if (keys.empty())
    return;
  const auto lock = simulation::lockRegistry();
  simulation::forEachOwner(keys, [](simulation::WorldDeactivationState& owner) {
    simulation::notifyModelTopology(owner);
  });
}

template <class Object>
DART_REGISTRY_NOINLINE void notifyParameterSlow(const Object* object)
{
  simulation::ModelKeys keys;
  simulation::collectModelKeys(object, keys);
  if (keys.empty())
    return;
  const auto lock = simulation::lockRegistry();
  simulation::forEachOwner(keys, [](simulation::WorldDeactivationState& owner) {
    owner.mParameterRevision.fetch_add(1u, std::memory_order_relaxed);
  });
}

// Requires the registry mutex.
void stampParameter(simulation::WorldDeactivationState& owner)
{
  owner.mParameterRevision.fetch_add(1u, std::memory_order_relaxed);
}

template <class Object>
inline void checkTopology(const Object* object)
{
  if (!simulation::noGuardedWorlds())
    checkTopologySlow(object);
}

template <class Object>
inline void notifyTopology(const Object* object)
{
  if (!simulation::noIndexedWorlds())
    notifyTopologySlow(object);
}

template <class Object>
inline void notifyParameter(const Object* object)
{
  if (!simulation::noIndexedWorlds())
    notifyParameterSlow(object);
}

DART_REGISTRY_NOINLINE void checkFrameTopologySlow(const dynamics::Frame* frame)
{
  simulation::ModelKeys keys;
  simulation::collectFrameKeys(frame, keys, false);
  simulation::throwRejection(simulation::lazyStepRejection(keys));
  if (simulation::noIndexedWorlds())
    return;
  simulation::RejectionCollector rejection;
  {
    const auto lock = simulation::lockRegistry();
    simulation::forEachOwner(keys, std::ref(rejection));
  }
  simulation::throwRejection(rejection.mReason);
}

DART_REGISTRY_NOINLINE void notifyFrameTopologySlow(
    const dynamics::Frame* frame)
{
  // ShapeFrame's Subject observer runs after its virtual Frame base. During
  // that teardown, its group entry must not be converted through old bases.
  if (frame->isShapeFrame() && !dynamic_cast<const Composite*>(frame))
    return;
  simulation::ModelKeys keys;
  simulation::collectFrameKeys(frame, keys, false);
  const auto lock = simulation::lockRegistry();
  simulation::forEachOwner(keys, [](simulation::WorldDeactivationState& owner) {
    simulation::notifyModelTopology(owner);
  });
}

} // namespace

bool hasTrackedModels()
{
  return !simulation::noIndexedWorlds();
}

bool hasGuardedModels()
{
  return !simulation::noGuardedWorlds();
}

void checkTopologyMutation(const Composite* object)
{
  checkTopology(object);
}

void checkTopologyMutation(const dynamics::BodyNode* object)
{
  checkTopology(object);
}

void checkTopologyMutation(const dynamics::ShapeFrame* object)
{
  checkTopology(object);
}

void checkTopologyMutation(const dynamics::Skeleton* object)
{
  checkTopology(object);
}

void checkTopologyMutation(const dynamics::Joint* object)
{
  checkTopology(object);
}

void notifyTopologyMutation(const Composite* object)
{
  notifyTopology(object);
}

void notifyTopologyMutation(const dynamics::BodyNode* object)
{
  notifyTopology(object);
}

void notifyTopologyMutation(const dynamics::ShapeFrame* object)
{
  notifyTopology(object);
}

void notifyTopologyMutation(const dynamics::Skeleton* object)
{
  notifyTopology(object);
}

void notifyTopologyMutation(const dynamics::Joint* object)
{
  notifyTopology(object);
}

void notifyParameterMutation(const Composite* object)
{
  notifyParameter(object);
}

void notifyParameterMutation(const dynamics::BodyNode* object)
{
  notifyParameter(object);
}

void notifyParameterMutation(const dynamics::ShapeFrame* object)
{
  notifyParameter(object);
}

void notifyParameterMutation(const dynamics::Skeleton* object)
{
  notifyParameter(object);
}

void notifyParameterMutation(const dynamics::Joint* object)
{
  notifyParameter(object);
}

void checkFrameTopologyMutation(const dynamics::Frame* frame)
{
  if (simulation::noGuardedWorlds() || !frame || frame->isWorld())
    return;
  checkFrameTopologySlow(frame);
}

void notifyFrameTopologyMutation(const dynamics::Frame* frame)
{
  if (simulation::noIndexedWorlds() || !frame || frame->isWorld())
    return;
  notifyFrameTopologySlow(frame);
}

void notifyFrameParentChanged(const dynamics::Frame* frame)
{
  if (simulation::noIndexedWorlds() || !frame || frame->isWorld())
    return;
  simulation::rechainMembersUnder(frame);
}

void notifyModelDestruction(const dynamics::Skeleton* skeleton)
{
  if (simulation::noIndexedWorlds() || !skeleton)
    return;
  simulation::purgeModelKey(skeleton);
}

} // namespace detail
} // namespace common
namespace collision {
namespace detail {

void checkWorldGroupMutation(const CollisionGroup* group)
{
  checkWorldGroupUpdate(group, nullptr);
}

void checkWorldGroupUpdate(
    const CollisionGroup* group, const simulation::World* requester)
{
  if (simulation::noGuardedWorlds())
    return;
  for (const auto* state = simulation::tlsSteppingWorld; state;
       state = state->mOuterSteppingWorld) {
    if (!state->mIndexed && state->mWorld != requester
        && simulation::steppingWorldOwnsSolverObject(
            *state, simulation::SingleKey{group}))
      simulation::throwRejection(simulation::Rejection::Busy);
  }
  if (simulation::noIndexedWorlds())
    return;
  // Only a stepping owner rejects: an idle compiled World reports raw group
  // edits at its next step() or compile().
  bool busy = false;
  {
    const auto lock = simulation::lockRegistry();
    simulation::forEachOwner(
        group, [&](const simulation::WorldDeactivationState& owner) {
          busy = busy
                 || (owner.mWorld != requester
                     && owner.mStepping.load(std::memory_order_acquire));
        });
  }
  if (busy)
    simulation::throwRejection(simulation::Rejection::Busy);
}

void notifyWorldGroupMutation(const CollisionGroup* group)
{
  if (simulation::noIndexedWorlds())
    return;
  const auto lock = simulation::lockRegistry();
  simulation::forEachOwner(
      group, [](simulation::WorldDeactivationState& owner) {
        if (owner.mCompiled.load(std::memory_order_relaxed)
            && owner.mEditDepth.load(std::memory_order_relaxed) == 0u)
          owner.mRawGroupMutation.store(true, std::memory_order_relaxed);
        simulation::notifyModelTopology(owner);
      });
}

void notifyWorldGroupShapeFrameAdded(
    const CollisionGroup* group, const dynamics::ShapeFrame* frame)
{
  if (simulation::noIndexedWorlds() || !frame)
    return;
  simulation::ModelKeys keys;
  simulation::collectFrameKeys(frame, keys, true);
  const auto lock = simulation::lockRegistry();
  simulation::forEachOwner(
      group, [&](simulation::WorldDeactivationState& owner) {
        simulation::registerChain(owner.mGroupFrameChains, owner, frame, keys);
      });
}

void notifyWorldGroupShapeFrameRemoved(
    const CollisionGroup* group, const dynamics::ShapeFrame* frame)
{
  if (simulation::noIndexedWorlds())
    return;
  const auto lock = simulation::lockRegistry();
  simulation::forEachOwner(
      group, [&](simulation::WorldDeactivationState& owner) {
        simulation::unregisterChain(owner.mGroupFrameChains, owner, frame);
      });
}

void notifyWorldGroupSubscribed(
    const CollisionGroup* group, const dynamics::MetaSkeleton* source)
{
  if (simulation::noIndexedWorlds())
    return;
  const auto entries = simulation::sourceEntries(source);
  const auto lock = simulation::lockRegistry();
  simulation::forEachOwner(
      group, [&](simulation::WorldDeactivationState& owner) {
        simulation::registerSource(owner, source, entries);
      });
}

void notifyWorldGroupSubscribed(
    const CollisionGroup* group, const dynamics::BodyNode* source)
{
  if (simulation::noIndexedWorlds() || !source)
    return;
  const std::vector<std::pair<const void*, const void*>> entries{
      {source, source->getSkeleton().get()}};
  const auto lock = simulation::lockRegistry();
  simulation::forEachOwner(
      group, [&](simulation::WorldDeactivationState& owner) {
        simulation::registerSource(owner, source, entries);
      });
}

void notifyWorldGroupUnsubscribed(
    const CollisionGroup* group, const void* source)
{
  if (simulation::noIndexedWorlds())
    return;
  const auto lock = simulation::lockRegistry();
  simulation::forEachOwner(
      group, [&](simulation::WorldDeactivationState& owner) {
        simulation::unregisterSource(owner, source);
      });
}

void notifyWorldGroupSourceBodyAdded(
    const CollisionGroup* group,
    const void* source,
    const dynamics::BodyNode* body)
{
  if (simulation::noIndexedWorlds() || !body)
    return;
  const void* skeleton = body->getSkeleton().get();
  const auto lock = simulation::lockRegistry();
  simulation::OwnerSet targets;
  const auto owners = simulation::modelOwners().mOwners.find(group);
  if (owners == simulation::modelOwners().mOwners.end())
    return;
  for (auto* owner : owners->second) {
    const auto sourceIt = owner->mGroupSourceChains.find(source);
    if (sourceIt == owner->mGroupSourceChains.end())
      continue;
    const auto& entries = sourceIt->second;
    if (std::none_of(entries.begin(), entries.end(), [body](const auto& entry) {
          return entry.first == body;
        }))
      targets.insert(owner);
  }
  std::size_t added = 0u;
  try {
    targets.each([&](simulation::WorldDeactivationState& owner) {
      auto& entries = owner.mGroupSourceChains.find(source)->second;
      simulation::addOwner(owner.mModelKeys, owner, skeleton);
      try {
        simulation::indexAllocation();
        entries.emplace_back(body, skeleton);
      } catch (...) {
        simulation::removeOwner(owner.mModelKeys, owner, skeleton);
        throw;
      }
      ++added;
    });
  } catch (...) {
    targets.each([&](simulation::WorldDeactivationState& owner) {
      if (added == 0u)
        return;
      --added;
      simulation::removeOwner(owner.mModelKeys, owner, skeleton);
      owner.mGroupSourceChains.find(source)->second.pop_back();
    });
    throw;
  }
}

void notifyWorldGroupSourceBodyRemoved(
    const CollisionGroup* group, const void* source, const void* body)
{
  if (simulation::noIndexedWorlds())
    return;
  const auto lock = simulation::lockRegistry();
  simulation::forEachOwner(
      group, [&](simulation::WorldDeactivationState& owner) {
        const auto it = owner.mGroupSourceChains.find(source);
        if (it == owner.mGroupSourceChains.end())
          return;
        auto& entries = it->second;
        const auto entry = std::find_if(
            entries.begin(), entries.end(), [body](const auto& candidate) {
              return candidate.first == body;
            });
        if (entry == entries.end())
          return;
        simulation::removeOwner(owner.mModelKeys, owner, entry->second);
        entries.erase(entry);
      });
}

void notifyWorldGroupCleared(const CollisionGroup* group)
{
  if (simulation::noIndexedWorlds())
    return;
  const auto lock = simulation::lockRegistry();
  simulation::forEachOwner(
      group, [](simulation::WorldDeactivationState& owner) {
        simulation::unregisterAllChains(owner.mGroupFrameChains, owner);
        simulation::unregisterAllSources(owner);
      });
}

void checkWorldDetectorTopologyMutation(const CollisionDetector* detector)
{
  if (simulation::noGuardedWorlds())
    return;
  simulation::throwRejection(simulation::lazyStepRejection(detector));
  if (simulation::noIndexedWorlds())
    return;
  simulation::RejectionCollector rejection;
  {
    const auto lock = simulation::lockRegistry();
    simulation::forEachOwner(detector, std::ref(rejection));
  }
  simulation::throwRejection(rejection.mReason);
}

void notifyWorldDetectorTopologyMutation(const CollisionDetector* detector)
{
  if (simulation::noIndexedWorlds())
    return;
  const auto lock = simulation::lockRegistry();
  simulation::forEachOwner(
      detector, [](simulation::WorldDeactivationState& owner) {
        simulation::notifyModelTopology(owner);
      });
}

void notifyWorldDetectorParameterMutation(const CollisionDetector* detector)
{
  if (simulation::noIndexedWorlds())
    return;
  const auto lock = simulation::lockRegistry();
  simulation::forEachOwner(detector, common::detail::stampParameter);
}

} // namespace detail
} // namespace collision
namespace constraint {
namespace detail {

void notifyWorldCollisionGroupReplaced(const ConstraintSolver* solver)
{
  if (simulation::noIndexedWorlds())
    return;
  const auto lock = simulation::lockRegistry();
  simulation::forEachOwner(
      solver, [](simulation::WorldDeactivationState& owner) {
        simulation::unregisterAllChains(owner.mGroupFrameChains, owner);
        simulation::unregisterAllSources(owner);
        simulation::refreshSolverKeys(owner);
      });
}

void notifyWorldSolverSkeletonAdded(
    const ConstraintSolver* solver, const dynamics::Skeleton* skeleton)
{
  if (simulation::noIndexedWorlds() || !skeleton)
    return;
  const auto lock = simulation::lockRegistry();
  simulation::forEachOwner(
      solver, [&](simulation::WorldDeactivationState& owner) {
        owner.mSolverSkeletonKeys.push_back(skeleton);
        simulation::addOwner(owner.mModelKeys, owner, skeleton);
      });
}

void notifyWorldSolverSkeletonRemoved(
    const ConstraintSolver* solver, const dynamics::Skeleton* skeleton)
{
  if (simulation::noIndexedWorlds())
    return;
  const auto lock = simulation::lockRegistry();
  simulation::forEachOwner(
      solver, [&](simulation::WorldDeactivationState& owner) {
        auto& keys = owner.mSolverSkeletonKeys;
        const auto it = std::find(keys.begin(), keys.end(), skeleton);
        if (it == keys.end())
          return;
        keys.erase(it);
        simulation::removeOwner(owner.mModelKeys, owner, skeleton);
      });
}

void checkWorldTopologyMutation(const ConstraintSolver* solver)
{
  if (simulation::noGuardedWorlds())
    return;
  simulation::throwRejection(simulation::lazyStepRejection(solver));
  if (simulation::noIndexedWorlds())
    return;
  simulation::RejectionCollector rejection;
  {
    const auto lock = simulation::lockRegistry();
    simulation::forEachOwner(solver, std::ref(rejection));
  }
  simulation::throwRejection(rejection.mReason);
}

void notifyWorldTopologyMutation(const ConstraintSolver* solver)
{
  if (simulation::noIndexedWorlds())
    return;
  const auto lock = simulation::lockRegistry();
  simulation::forEachOwner(
      solver, [](simulation::WorldDeactivationState& owner) {
        simulation::refreshSolverKeys(owner);
        simulation::notifyModelTopology(owner);
      });
}

void notifyWorldContactHandlersChanged(const ConstraintSolver* solver)
{
  if (simulation::noIndexedWorlds())
    return;
  const auto lock = simulation::lockRegistry();
  simulation::forEachOwner(
      solver, [](simulation::WorldDeactivationState& owner) {
        simulation::refreshSolverKeys(owner);
        simulation::refreshHandlerKeys(owner);
        simulation::notifyModelTopology(owner);
      });
}

void notifyWorldParameterMutation(const ConstraintSolver* solver)
{
  if (simulation::noIndexedWorlds())
    return;
  const auto lock = simulation::lockRegistry();
  simulation::forEachOwner(solver, common::detail::stampParameter);
}

void notifyBoxedSolverParameterMutation(const BoxedLcpSolver* solver)
{
  if (simulation::noIndexedWorlds())
    return;
  const auto lock = simulation::lockRegistry();
  simulation::forEachOwner(solver, common::detail::stampParameter);
}

void notifyConstraintParameterMutation(const ConstraintBase* constraint)
{
  if (simulation::noIndexedWorlds())
    return;
  const auto lock = simulation::lockRegistry();
  simulation::forEachOwner(constraint, common::detail::stampParameter);
}

void notifyGlobalConstraintParameterMutation()
{
  if (simulation::noIndexedWorlds())
    return;
  const auto lock = simulation::lockRegistry();
  for (auto* owner : simulation::modelOwners().mWorlds)
    owner->mParameterRevision.fetch_add(1u, std::memory_order_relaxed);
}

bool contactSurfaceChecksActive()
{
  return !simulation::noIndexedWorlds()
         || simulation::tlsSteppingWorld != nullptr;
}

void checkContactSurfaceMutation(const ContactSurfaceHandler* handler)
{
  if (simulation::noGuardedWorlds())
    return;
  simulation::throwRejection(simulation::lazyStepRejection(handler));
  if (simulation::noIndexedWorlds())
    return;
  simulation::RejectionCollector rejection;
  {
    const auto lock = simulation::lockRegistry();
    simulation::forEachOwner(handler, std::ref(rejection));
  }
  simulation::throwRejection(rejection.mReason);
}

void notifyContactSurfaceMutation(const ContactSurfaceHandler* handler)
{
  if (simulation::noIndexedWorlds())
    return;
  // The handler's new chain comes from the caller's own object before locking.
  // Owners splice it below the changed handler; none of their solvers is read.
  std::vector<const void*> parents;
  for (auto parent = const_cast<ContactSurfaceHandler*>(handler)->getParent();
       parent;
       parent = parent->getParent()) {
    if (parent.get() == handler
        || std::find(parents.begin(), parents.end(), parent.get())
               != parents.end())
      break;
    parents.push_back(parent.get());
  }
  const auto lock = simulation::lockRegistry();
  simulation::forEachOwner(
      handler, [&](simulation::WorldDeactivationState& owner) {
        auto& keys = owner.mHandlerKeys;
        const auto it = std::find(keys.begin(), keys.end(), handler);
        if (it == keys.end())
          return;
        std::vector<const void*> chain(keys.begin(), std::next(it));
        for (const void* parent : parents) {
          if (std::find(chain.begin(), chain.end(), parent) == chain.end())
            chain.push_back(parent);
        }
        for (const void* key : keys)
          simulation::removeOwner(owner.mModelKeys, owner, key);
        for (const void* key : chain)
          simulation::addOwner(owner.mModelKeys, owner, key);
        keys = std::move(chain);
        simulation::notifyModelTopology(owner);
      });
}

} // namespace detail
} // namespace constraint

namespace simulation {
namespace detail {

std::size_t getIndexedWorldCount()
{
  return gIndexedWorlds.load(std::memory_order_acquire);
}

void failIndexAllocation(std::size_t nth)
{
  gFailIndexAllocationIn = nth;
}

void setRechainUnlockedHook(std::function<void()> hook)
{
  gRechainUnlockedHook = std::move(hook);
}

std::size_t getRegistryLockCount()
{
  return gRegistryLocks.load(std::memory_order_relaxed);
}

} // namespace detail
} // namespace simulation
} // namespace dart

#undef DART_REGISTRY_NOINLINE
