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

#include "dart/constraint/ConstraintSolver.hpp"

#include "dart/collision/CollisionFilter.hpp"
#include "dart/collision/CollisionGroup.hpp"
#include "dart/collision/CollisionObject.hpp"
#include "dart/collision/Contact.hpp"
#include "dart/collision/dart/DARTCollisionDetector.hpp"
#include "dart/collision/fcl/FCLCollisionDetector.hpp"
#include "dart/common/Console.hpp"
#include "dart/common/Macros.hpp"
#include "dart/common/Profile.hpp"
#include "dart/constraint/BoxedLcpConstraintSolver.hpp"
#include "dart/constraint/ConstrainedGroup.hpp"
#include "dart/constraint/ContactConstraint.hpp"
#include "dart/constraint/ContactSurface.hpp"
#include "dart/constraint/CouplerConstraint.hpp"
#include "dart/constraint/DantzigBoxedLcpSolver.hpp"
#include "dart/constraint/DynamicJointConstraint.hpp"
#include "dart/constraint/JointConstraint.hpp"
#include "dart/constraint/JointCoulombFrictionConstraint.hpp"
#include "dart/constraint/LCPSolver.hpp"
#include "dart/constraint/MimicMotorConstraint.hpp"
#include "dart/constraint/PgsBoxedLcpSolver.hpp"
#include "dart/constraint/SoftContactConstraint.hpp"
#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/Joint.hpp"
#include "dart/dynamics/PlaneShape.hpp"
#include "dart/dynamics/PointMass.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/dynamics/SoftBodyNode.hpp"

#include <algorithm>
#include <array>
#include <condition_variable>
#include <limits>
#include <mutex>
#include <thread>
#include <type_traits>
#include <typeinfo>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include <cmath>
#include <cstdint>

namespace dart {
namespace constraint {

using namespace dynamics;

constexpr double kDefaultSleepContactPenetrationTolerance = 1e-5;
constexpr double kDenseContactIslandSleepContactPenetrationTolerance = 0.005;
constexpr double kSmallContactIslandMaxErrorReductionVelocity = 1e-3;
constexpr std::size_t kDenseContactIslandMinMobileSkeletons = 3u;
double gSleepContactPenetrationTolerance
    = kDefaultSleepContactPenetrationTolerance;
bool gSleepContactPenetrationToleranceUserConfigured = false;

bool contactTouchesPlaneShape(const collision::Contact& collisionContact)
{
  auto isPlaneShapeObject = [](const collision::CollisionObject* object) {
    const auto shape = object ? object->getShape() : nullptr;
    return shape != nullptr && shape->getType() == PlaneShape::getStaticType();
  };

  return isPlaneShapeObject(collisionContact.collisionObject1)
         || isPlaneShapeObject(collisionContact.collisionObject2);
}

struct CollidingStateSnapshot
{
  std::vector<std::pair<dynamics::BodyNode*, bool>> bodyNodes;
  std::vector<std::pair<dynamics::PointMass*, bool>> pointMasses;
};

CollidingStateSnapshot snapshotCollidingState(
    const std::vector<dynamics::SkeletonPtr>& skeletons)
{
  CollidingStateSnapshot snapshot;

  std::size_t numBodyNodes = 0u;
  std::size_t numPointMasses = 0u;
  for (const auto& skeleton : skeletons) {
    if (!skeleton)
      continue;

    numBodyNodes += skeleton->getNumBodyNodes();
    for (auto* bodyNode : skeleton->getBodyNodes()) {
      if (auto* softBodyNode = bodyNode->asSoftBodyNode())
        numPointMasses += softBodyNode->getPointMasses().size();
    }
  }

  snapshot.bodyNodes.reserve(numBodyNodes);
  snapshot.pointMasses.reserve(numPointMasses);

  for (const auto& skeleton : skeletons) {
    if (!skeleton)
      continue;

    for (auto* bodyNode : skeleton->getBodyNodes()) {
      DART_SUPPRESS_DEPRECATED_BEGIN
      snapshot.bodyNodes.emplace_back(bodyNode, bodyNode->isColliding());
      DART_SUPPRESS_DEPRECATED_END

      if (auto* softBodyNode = bodyNode->asSoftBodyNode()) {
        for (auto* pointMass : softBodyNode->getPointMasses())
          snapshot.pointMasses.emplace_back(
              pointMass, pointMass->isColliding());
      }
    }
  }

  return snapshot;
}

void restoreCollidingState(const CollidingStateSnapshot& snapshot)
{
  for (const auto& [bodyNode, wasColliding] : snapshot.bodyNodes) {
    DART_SUPPRESS_DEPRECATED_BEGIN
    bodyNode->setColliding(wasColliding);
    DART_SUPPRESS_DEPRECATED_END
  }

  for (const auto& [pointMass, wasColliding] : snapshot.pointMasses)
    pointMass->setColliding(wasColliding);
}

//==============================================================================
template <typename ExactT, typename DynamicT>
bool isExactDynamicType(const DynamicT* object)
{
  if (object == nullptr)
    return false;

#if defined(__clang__)
  #pragma clang diagnostic push
  #pragma clang diagnostic ignored "-Wpotentially-evaluated-expression"
#endif
  const bool isExact = typeid(*object) == typeid(ExactT);
#if defined(__clang__)
  #pragma clang diagnostic pop
#endif

  return isExact;
}

//==============================================================================
bool isExactDefaultContactSurfaceHandler(
    const ContactSurfaceHandlerPtr& handler)
{
  return isExactDynamicType<DefaultContactSurfaceHandler>(handler.get());
}

//==============================================================================
bool isRandomizedPgsSolver(const ConstBoxedLcpSolverPtr& solver)
{
  const auto pgs = std::dynamic_pointer_cast<const PgsBoxedLcpSolver>(solver);
  return pgs != nullptr && pgs->getOption().mRandomizeConstraintOrder;
}

//==============================================================================
template <typename Solver>
bool isExactBoxedSolverType(const ConstBoxedLcpSolverPtr& solver)
{
  return isExactDynamicType<Solver>(solver.get());
}

//==============================================================================
bool isParallelSafeBuiltInBoxedSolver(const ConstBoxedLcpSolverPtr& solver)
{
  if (isExactBoxedSolverType<DantzigBoxedLcpSolver>(solver))
    return true;

  if (!isExactBoxedSolverType<PgsBoxedLcpSolver>(solver))
    return false;

  return !isRandomizedPgsSolver(solver);
}

//==============================================================================
bool canUseParallelBuiltInBoxedSolvers(const ConstraintSolver& solver)
{
  const auto* boxedSolver
      = dynamic_cast<const BoxedLcpConstraintSolver*>(&solver);
  if (boxedSolver == nullptr)
    return false;

  if (!isParallelSafeBuiltInBoxedSolver(boxedSolver->getBoxedLcpSolver()))
    return false;

  const auto secondarySolver = boxedSolver->getSecondaryBoxedLcpSolver();
  if (secondarySolver == nullptr)
    return true;

  return isParallelSafeBuiltInBoxedSolver(secondarySolver);
}

//==============================================================================
void configureDARTCollisionThreads(
    const collision::CollisionDetectorPtr& collisionDetector,
    std::size_t numThreads)
{
  auto* dartCollisionDetector = dynamic_cast<collision::DARTCollisionDetector*>(
      collisionDetector.get());
  if (dartCollisionDetector != nullptr)
    dartCollisionDetector->setNumCollisionThreads(numThreads);
}

//==============================================================================
class ConstraintThreadPool
{
public:
  ConstraintThreadPool() = default;

  ~ConstraintThreadPool()
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
    for (std::size_t i = 0; i < workerCount; ++i)
      mWorkers.emplace_back([this] { workerLoop(); });
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
ConstraintSolver::ConstraintSolver(double timeStep)
  : mCollisionDetector(collision::FCLCollisionDetector::create()),
    mCollisionGroup(mCollisionDetector->createCollisionGroupAsSharedPtr()),
    mCollisionOption(collision::CollisionOption(
        true, 1000u, std::make_shared<collision::BodyNodeCollisionFilter>())),
    mTimeStep(timeStep),
    mContactSurfaceHandler(std::make_shared<DefaultContactSurfaceHandler>())
{
  DART_ASSERT(timeStep > 0.0);

  auto cd = std::static_pointer_cast<collision::FCLCollisionDetector>(
      mCollisionDetector);

  cd->setPrimitiveShapeType(collision::FCLCollisionDetector::PRIMITIVE);
}

//==============================================================================
ConstraintSolver::ConstraintSolver()
  : mCollisionDetector(collision::FCLCollisionDetector::create()),
    mCollisionGroup(mCollisionDetector->createCollisionGroupAsSharedPtr()),
    mCollisionOption(collision::CollisionOption(
        true, 1000u, std::make_shared<collision::BodyNodeCollisionFilter>())),
    mTimeStep(0.001),
    mContactSurfaceHandler(std::make_shared<DefaultContactSurfaceHandler>())
{
  auto cd = std::static_pointer_cast<collision::FCLCollisionDetector>(
      mCollisionDetector);

  cd->setPrimitiveShapeType(collision::FCLCollisionDetector::PRIMITIVE);
}

//==============================================================================
ConstraintSolver::~ConstraintSolver() = default;

//==============================================================================
void ConstraintSolver::addSkeleton(const SkeletonPtr& skeleton)
{
  DART_ASSERT(
      skeleton
      && "Null pointer skeleton is now allowed to add to ConstraintSover.");

  if (hasSkeleton(skeleton)) {
    dtwarn << "[ConstraintSolver::addSkeleton] Attempting to add "
           << "skeleton '" << skeleton->getName()
           << "', which already exists in the ConstraintSolver.\n";

    return;
  }

  mCollisionGroup->subscribeTo(skeleton);
  mSkeletons.push_back(skeleton);
  mConstrainedGroups.reserve(mSkeletons.size());
}

//==============================================================================
void ConstraintSolver::addSkeletons(const std::vector<SkeletonPtr>& skeletons)
{
  for (const auto& skeleton : skeletons)
    addSkeleton(skeleton);
}

//==============================================================================
const std::vector<SkeletonPtr>& ConstraintSolver::getSkeletons() const
{
  return mSkeletons;
}

//==============================================================================
void ConstraintSolver::removeSkeleton(const SkeletonPtr& skeleton)
{
  DART_ASSERT(
      skeleton
      && "Null pointer skeleton is now allowed to add to ConstraintSover.");

  if (!hasSkeleton(skeleton)) {
    dtwarn << "[ConstraintSolver::removeSkeleton] Attempting to remove "
           << "skeleton '" << skeleton->getName()
           << "', which doesn't exist in the ConstraintSolver.\n";
  }

  mCollisionGroup->unsubscribeFrom(skeleton.get());
  mSkeletons.erase(
      remove(mSkeletons.begin(), mSkeletons.end(), skeleton), mSkeletons.end());
  mConstrainedGroups.reserve(mSkeletons.size());
}

//==============================================================================
void ConstraintSolver::removeSkeletons(
    const std::vector<SkeletonPtr>& skeletons)
{
  for (const auto& skeleton : skeletons)
    removeSkeleton(skeleton);
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
    dtwarn << "Constraint solver already contains constraint that you are "
           << "trying to add." << std::endl;
    return;
  }

  mManualConstraints.push_back(constraint);
}

//==============================================================================
void ConstraintSolver::removeConstraint(const ConstraintBasePtr& constraint)
{
  DART_ASSERT(constraint);

  if (!containConstraint(constraint)) {
    dtwarn << "Constraint solver deos not contain constraint that you are "
           << "trying to remove." << std::endl;
    return;
  }

  mManualConstraints.erase(
      remove(mManualConstraints.begin(), mManualConstraints.end(), constraint),
      mManualConstraints.end());
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
std::vector<ConstraintBasePtr> ConstraintSolver::getConstraints()
{
  // Return a copy of constraint list not to expose the implementation detail
  // that the constraint pointers are held in a vector, in case we want to
  // change this implementation in the future.
  return mManualConstraints;
}

//==============================================================================
std::vector<ConstConstraintBasePtr> ConstraintSolver::getConstraints() const
{
  std::vector<ConstConstraintBasePtr> constraints;
  constraints.reserve(mManualConstraints.size());
  for (auto& constraint : mManualConstraints)
    constraints.push_back(constraint);

  return constraints;
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
void ConstraintSolver::setDeactivationActive(bool _active)
{
  mDeactivationActive = _active;
}

//==============================================================================
void ConstraintSolver::setAutomaticSleepingEnabled(bool _enabled)
{
  setDeactivationActive(_enabled);
}

//==============================================================================
void ConstraintSolver::setAutomaticSleepingContactPenetrationTolerance(
    double tolerance)
{
  if (tolerance < 0.0) {
    dtwarn << "[ConstraintSolver] Automatic sleeping contact penetration "
           << "tolerance[" << tolerance << "] is lower than 0.0. It is set "
           << "to 0.0." << std::endl;
    tolerance = 0.0;
  }

  gSleepContactPenetrationTolerance = tolerance;
  gSleepContactPenetrationToleranceUserConfigured = true;
}

//==============================================================================
void ConstraintSolver::resetAutomaticSleepingContactPenetrationTolerance()
{
  gSleepContactPenetrationTolerance = kDefaultSleepContactPenetrationTolerance;
  gSleepContactPenetrationToleranceUserConfigured = false;
}

//==============================================================================
double ConstraintSolver::getAutomaticSleepingContactPenetrationTolerance()
{
  return gSleepContactPenetrationTolerance;
}

//==============================================================================
void ConstraintSolver::setNumSimulationThreads(std::size_t numThreads)
{
  if (numThreads == 0u) {
    numThreads = std::thread::hardware_concurrency();
    if (numThreads == 0u)
      numThreads = 1u;
  }

  mNumSimulationThreads = std::max<std::size_t>(1u, numThreads);
  configureDARTCollisionThreads(mCollisionDetector, mNumSimulationThreads);
  if (mNumSimulationThreads <= 1u) {
    mConstraintThreadPool.reset();
    return;
  }

  if (!mConstraintThreadPool)
    mConstraintThreadPool = std::make_unique<ConstraintThreadPool>();
  mConstraintThreadPool->setWorkerCount(mNumSimulationThreads - 1u);
}

//==============================================================================
std::size_t ConstraintSolver::getNumSimulationThreads() const
{
  return mNumSimulationThreads;
}

//==============================================================================
void ConstraintSolver::setCollisionDetector(
    collision::CollisionDetector* collisionDetector)
{
  setCollisionDetector(
      std::unique_ptr<collision::CollisionDetector>(collisionDetector));
}

//==============================================================================
void ConstraintSolver::setCollisionDetector(
    const std::shared_ptr<collision::CollisionDetector>& collisionDetector)
{
  if (!collisionDetector) {
    dtwarn << "[ConstraintSolver::setCollisionDetector] Attempting to assign "
           << "nullptr as the new collision detector to the constraint solver, "
           << "which is not allowed. Ignoring.\n";
    return;
  }

  if (mCollisionDetector == collisionDetector)
    return;

  mCollisionDetector = collisionDetector;
  configureDARTCollisionThreads(mCollisionDetector, mNumSimulationThreads);

  mCollisionGroup = mCollisionDetector->createCollisionGroupAsSharedPtr();

  for (const auto& skeleton : mSkeletons)
    mCollisionGroup->addShapeFramesOf(skeleton.get());
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
void ConstraintSolver::setLCPSolver(std::unique_ptr<LCPSolver> /*lcpSolver*/)
{
  dtwarn << "[ConstraintSolver::setLCPSolver] This function is deprecated in "
         << "DART 6.7. Please use "
         << "BoxedLcpConstraintSolver::setBoxedLcpSolver() instead. "
         << "Doing nothing.";
}

//==============================================================================
LCPSolver* ConstraintSolver::getLCPSolver() const
{
  dtwarn << "[ConstraintSolver::getLCPSolver] This function is deprecated in "
         << "DART 6.7. Please use "
         << "BoxedLcpConstraintSolver::getBoxedLcpSolver() instead. "
         << "Returning nullptr.";

  return nullptr;
}

//==============================================================================
void ConstraintSolver::solve()
{
  const bool profileRecording
      = dart::common::profile::isProfileRecordingEnabled();
  DART_PROFILE_SCOPED_IF_N(profileRecording, "ConstraintSolver::solve");

  const bool splitImpulse = mSplitImpulseEnabled;

  {
    DART_PROFILE_SCOPED_IF_N(
        profileRecording, "ConstraintSolver::clearSkeletonConstraintState");

    auto clearSkeletonAt = [&](std::size_t i) {
      auto& skeleton = mSkeletons[i];
      skeleton->clearConstraintImpulses();
      if (splitImpulse) {
        skeleton->clearPositionConstraintImpulses();
        skeleton->clearPositionVelocityChanges();
        skeleton->setPositionImpulseApplied(false);
      }
      DART_SUPPRESS_DEPRECATED_BEGIN
      skeleton->clearCollidingBodies();
      DART_SUPPRESS_DEPRECATED_END
    };

    if (mConstraintThreadPool != nullptr && mNumSimulationThreads > 1u
        && mSkeletons.size() >= 128u) {
      mConstraintThreadPool->parallelFor(
          mSkeletons.size(), mNumSimulationThreads, clearSkeletonAt);
    } else {
      for (std::size_t i = 0; i < mSkeletons.size(); ++i)
        clearSkeletonAt(i);
    }
  }

  // Update constraints and collect active constraints
  {
    DART_PROFILE_SCOPED_IF_N(
        profileRecording, "ConstraintSolver::updateConstraints");
    updateConstraints();
  }

  // Build constrained groups
  {
    DART_PROFILE_SCOPED_IF_N(
        profileRecording, "ConstraintSolver::buildConstrainedGroups");
    buildConstrainedGroups();
  }

  // Solve constrained groups
  {
    DART_PROFILE_SCOPED_IF_N(
        profileRecording, "ConstraintSolver::solveConstrainedGroups");
    solveConstrainedGroups();
  }

  if (splitImpulse) {
    DART_PROFILE_SCOPED_IF_N(
        profileRecording, "ConstraintSolver::solvePositionConstrainedGroups");
    solvePositionConstrainedGroups();
  }
}

//==============================================================================
void ConstraintSolver::prepareForSimulation()
{
  const auto collidingState = snapshotCollidingState(mSkeletons);
  const auto lastCollisionContacts = mCollisionResult.getContacts();
  const std::size_t collisionGroupContentVersion
      = mCollisionGroup ? mCollisionGroup->getContentVersion() : 0u;
  constexpr int kPreparationPasses = 2;
  for (int pass = 0; pass < kPreparationPasses; ++pass) {
    updateConstraints(false);
    buildConstrainedGroups();
    reserveConstrainedGroupsScratch();
  }
  mCollisionResult.clear();
  if (!mCollisionGroup
      || mCollisionGroup->getContentVersion() == collisionGroupContentVersion) {
    for (const auto& contact : lastCollisionContacts)
      mCollisionResult.addContact(contact);
  }
  restoreCollidingState(collidingState);
}

//==============================================================================
void ConstraintSolver::setFromOtherConstraintSolver(
    const ConstraintSolver& other)
{
  removeAllSkeletons();
  mManualConstraints.clear();
  mAutomaticJointConstraintJoints.clear();
  mAutomaticJointConstraintRevision = static_cast<std::size_t>(-1);
  mAutomaticJointConstraintSkeletonVersion = static_cast<std::size_t>(-1);

  addSkeletons(other.getSkeletons());
  mManualConstraints = other.mManualConstraints;

  mContactSurfaceHandler = other.mContactSurfaceHandler;
  mSplitImpulseEnabled = other.mSplitImpulseEnabled;
  setNumSimulationThreads(other.getNumSimulationThreads());
}

//==============================================================================
bool ConstraintSolver::canJointCreateAutomaticConstraint(
    const dynamics::Joint* joint) const
{
  if (joint == nullptr || joint->isKinematic())
    return false;

  return joint->hasCoulombFriction() || joint->areLimitsEnforced()
         || joint->hasActuatorType(dynamics::Joint::SERVO)
         || (joint->hasActuatorType(dynamics::Joint::MIMIC)
             && joint->getMimicJoint());
}

//==============================================================================
void ConstraintSolver::updateAutomaticJointConstraintCache()
{
  std::size_t skeletonVersion = mSkeletons.size();
  auto mix = [](std::size_t& value, std::size_t input) {
    value ^= input + 0x9e3779b97f4a7c15ULL + (value << 6) + (value >> 2);
  };

  for (const auto& skel : mSkeletons) {
    mix(skeletonVersion, reinterpret_cast<std::uintptr_t>(skel.get()));
    mix(skeletonVersion, skel ? skel->getVersion() : 0u);
  }

  const std::size_t jointRevision
      = dynamics::Joint::getAutomaticConstraintRevision();
  if (mAutomaticJointConstraintRevision == jointRevision
      && mAutomaticJointConstraintSkeletonVersion == skeletonVersion) {
    return;
  }

  mAutomaticJointConstraintRevision = jointRevision;
  mAutomaticJointConstraintSkeletonVersion = skeletonVersion;
  mAutomaticJointConstraintJoints.clear();

  for (const auto& skel : mSkeletons) {
    const std::size_t numJoints = skel->getNumJoints();
    for (std::size_t i = 0; i < numJoints; ++i) {
      dynamics::Joint* joint = skel->getJoint(i);
      if (canJointCreateAutomaticConstraint(joint))
        mAutomaticJointConstraintJoints.push_back(joint);
    }
  }
}

//==============================================================================
bool ConstraintSolver::containSkeleton(const ConstSkeletonPtr& skeleton) const
{
  return hasSkeleton(skeleton);
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

  for (const auto& itrSkel : mSkeletons) {
    if (itrSkel == skeleton)
      return true;
  }

  return false;
}

//==============================================================================
bool ConstraintSolver::checkAndAddSkeleton(const SkeletonPtr& skeleton)
{
  if (!hasSkeleton(skeleton)) {
    mSkeletons.push_back(skeleton);
    return true;
  } else {
    dtwarn << "Skeleton [" << skeleton->getName()
           << "] is already in ConstraintSolver." << std::endl;
    return false;
  }
}

//==============================================================================
bool ConstraintSolver::containConstraint(
    const ConstConstraintBasePtr& constraint) const
{
  return std::find(
             mManualConstraints.begin(), mManualConstraints.end(), constraint)
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
    dtwarn << "Constraint is already in ConstraintSolver." << std::endl;
    return false;
  }
}

//==============================================================================
void ConstraintSolver::updateConstraints(bool updateManualConstraints)
{
  // Clear previous active constraint list
  mActiveConstraints.clear();
  mActiveConstraintsAllSingleReactiveContacts = true;
  mActiveConstraintsHaveCustomContactConstraint = false;
  mActiveSingleReactiveContactsNeedSharedDependencyScan = false;

  //----------------------------------------------------------------------------
  // Update manual constraints
  //----------------------------------------------------------------------------
  if (updateManualConstraints) {
    for (auto& manualConstraint : mManualConstraints) {
      manualConstraint->update();

      if (manualConstraint->isActive()) {
        mActiveConstraintsAllSingleReactiveContacts = false;
        mActiveConstraints.push_back(manualConstraint);
      }
    }
  }

  //----------------------------------------------------------------------------
  // Update automatic constraints: contact constraints
  //----------------------------------------------------------------------------
  mCollisionResult.clear();

  auto* restingContactFilter
      = dynamic_cast<collision::BodyNodeCollisionFilter*>(
          mCollisionOption.collisionFilter.get());
  if (restingContactFilter != nullptr) {
    bool hasAwakeMobileSkeleton = false;
    if (mDeactivationActive) {
      for (const auto& skeleton : mSkeletons) {
        if (skeleton->isMobile() && !skeleton->isResting()) {
          hasAwakeMobileSkeleton = true;
          break;
        }
      }
    }
    restingContactFilter->setSolverRestingContactFilterActive(
        mDeactivationActive, hasAwakeMobileSkeleton);
  }

  {
    mCollisionGroup->collide(mCollisionOption, &mCollisionResult);
  }

  if (restingContactFilter != nullptr)
    restingContactFilter->setSolverRestingContactFilterActive(false, false);

  const bool useBuiltInDefaultContactActiveState
      = isExactDefaultContactSurfaceHandler(mContactSurfaceHandler);
  const auto* builtInDefaultContactHandler
      = useBuiltInDefaultContactActiveState
            ? static_cast<const DefaultContactSurfaceHandler*>(
                mContactSurfaceHandler.get())
            : nullptr;
  const bool useBuiltInDefaultSurfaceParamsCache
      = builtInDefaultContactHandler != nullptr
        && builtInDefaultContactHandler->mParent == nullptr;

  if (useBuiltInDefaultContactActiveState) {
    mReusableContactConstraints.clear();
    mReusableContactConstraints.swap(mContactConstraints);
  } else {
    mContactConstraints.clear();
    if (mReusableContactConstraints.capacity() < mContactConstraints.capacity())
      mReusableContactConstraints.reserve(mContactConstraints.capacity());
    mReusableContactConstraints.clear();
  }

  // Move previous soft contact constraints into a reuse pool so steady-state
  // soft contact solving does not allocate one shared object per contact.
  mReusableSoftContactConstraints.clear();
  mReusableSoftContactConstraints.swap(mSoftContactConstraints);

  // Create a mapping of contact pairs to the number of contacts between them.
  // The scratch table uses open addressing over retained vectors so the
  // per-step contact-pair count remains order-independent without allocating
  // one unordered_map node per contact pair.
  struct ContactPair
  {
    collision::CollisionObject* first;
    collision::CollisionObject* second;
  };
  const auto makeContactPair = [](collision::CollisionObject* object1,
                                  collision::CollisionObject* object2) {
    ContactPair pair{object1, object2};
    if (reinterpret_cast<std::uintptr_t>(pair.first)
        > reinterpret_cast<std::uintptr_t>(pair.second)) {
      std::swap(pair.first, pair.second);
    }
    return pair;
  };
  struct ContactPairHash
  {
    std::size_t operator()(const ContactPair& pair) const
    {
      const auto a = reinterpret_cast<std::uintptr_t>(pair.first);
      const auto b = reinterpret_cast<std::uintptr_t>(pair.second);
      const std::size_t h1 = std::hash<std::uintptr_t>()(a);
      const std::size_t h2 = std::hash<std::uintptr_t>()(b);
      return h1 ^ (h2 + 0x9e3779b97f4a7c15ULL + (h1 << 6) + (h1 >> 2));
    }
  };
  struct ContactPairCount
  {
    ContactPair pair;
    const dynamics::BodyNode* bodyNode1;
    const dynamics::BodyNode* bodyNode2;
    const dynamics::ShapeNode* shapeNode1;
    const dynamics::ShapeNode* shapeNode2;
    bool defaultSurfaceParamsChecked;
    bool canUseDefaultSurfaceParams;
    bool skipRelVelocityBody1;
    bool skipRelVelocityBody2;
    std::size_t count;
    std::size_t firstCandidateIndex;
    std::size_t lastCandidateIndex;
    ContactSurfaceParams surfaceParams;
    bool surfaceParamsInitialized;
  };
  struct ContactCandidate
  {
    collision::Contact* contact;
    std::size_t contactPairIndex;
    std::size_t nextCandidateIndex;
  };

  constexpr std::size_t invalidContactPairIndex
      = (std::numeric_limits<std::size_t>::max)();
  static thread_local std::vector<ContactPairCount> contactPairCounts;
  contactPairCounts.clear();
  static thread_local std::vector<std::size_t> contactPairBuckets;

  static thread_local std::vector<ContactCandidate> contactCandidates;
  contactCandidates.clear();

  {
    contactPairCounts.reserve(mCollisionResult.getNumContacts());
    contactCandidates.reserve(mCollisionResult.getNumContacts());
    const ContactPairHash contactPairHash;
    bool contactPairBucketsInitialized = false;
    const auto initializeContactPairBuckets = [&]() {
      if (contactPairBucketsInitialized)
        return;

      std::size_t bucketCount = 2u;
      while (bucketCount < mCollisionResult.getNumContacts() * 2u)
        bucketCount <<= 1u;
      if (contactPairBuckets.size() < bucketCount)
        contactPairBuckets.resize(bucketCount, invalidContactPairIndex);
      std::fill(
          contactPairBuckets.begin(),
          contactPairBuckets.end(),
          invalidContactPairIndex);
      contactPairBucketsInitialized = true;
    };

    std::size_t lastContactPairIndex = invalidContactPairIndex;
    ContactPair lastContactPair{nullptr, nullptr};

    const auto findOrCreateContactPairIndex
        = [&](const ContactPair& pair,
              const dynamics::BodyNode* bodyNode1,
              const dynamics::BodyNode* bodyNode2,
              const dynamics::ShapeNode* shapeNode1,
              const dynamics::ShapeNode* shapeNode2) -> std::size_t {
      initializeContactPairBuckets();

      const std::size_t bucketMask = contactPairBuckets.size() - 1u;
      std::size_t bucket = contactPairHash(pair) & bucketMask;
      while (true) {
        const std::size_t pairIndex = contactPairBuckets[bucket];
        if (pairIndex == invalidContactPairIndex) {
          const std::size_t newPairIndex = contactPairCounts.size();
          contactPairCounts.push_back(
              {pair,
               bodyNode1,
               bodyNode2,
               shapeNode1,
               shapeNode2,
               false,
               false,
               false,
               false,
               0u,
               invalidContactPairIndex,
               invalidContactPairIndex,
               ContactSurfaceParams(),
               false});
          contactPairBuckets[bucket] = newPairIndex;
          return newPairIndex;
        }

        const auto& existingPair = contactPairCounts[pairIndex].pair;
        if (existingPair.first == pair.first
            && existingPair.second == pair.second) {
          return pairIndex;
        }

        bucket = (bucket + 1u) & bucketMask;
      }
    };
    const auto findContactPairIndex
        = [&](collision::CollisionObject* object1,
              collision::CollisionObject* object2,
              dynamics::BodyNode* bodyNode1,
              dynamics::BodyNode* bodyNode2,
              const dynamics::ShapeNode* shapeNode1,
              const dynamics::ShapeNode* shapeNode2) -> std::size_t {
      const auto pair = makeContactPair(object1, object2);
      if (lastContactPairIndex != invalidContactPairIndex
          && lastContactPair.first == pair.first
          && lastContactPair.second == pair.second) {
        return lastContactPairIndex;
      }

      const bool objectsWereSwapped = pair.first != object1;
      const auto* pairBodyNode1 = objectsWereSwapped ? bodyNode2 : bodyNode1;
      const auto* pairBodyNode2 = objectsWereSwapped ? bodyNode1 : bodyNode2;
      const auto* pairShapeNode1 = objectsWereSwapped ? shapeNode2 : shapeNode1;
      const auto* pairShapeNode2 = objectsWereSwapped ? shapeNode1 : shapeNode2;
      const auto contactPairIndex = findOrCreateContactPairIndex(
          pair, pairBodyNode1, pairBodyNode2, pairShapeNode1, pairShapeNode2);
      lastContactPair = pair;
      lastContactPairIndex = contactPairIndex;
      return contactPairIndex;
    };

    for (auto i = 0u; i < mCollisionResult.getNumContacts(); ++i) {
      auto& contact = mCollisionResult.getContact(i);

      // Skip contacts with non-finite geometry. A collision shape with an
      // invalid (infinite or NaN) dimension, a malformed mesh, or a third-party
      // collision backend can report a contact whose point, normal, or
      // penetration depth is not finite. Such a contact would otherwise inject
      // NaN/Inf into the contact constraint Jacobians, corrupting the LCP solve
      // in release builds and tripping an assertion in ContactConstraint in
      // debug builds. See gz-physics issue #1010.
      if (!contact.point.allFinite() || !contact.normal.allFinite()
          || !std::isfinite(contact.penetrationDepth)) {
        dtwarn
            << "[ConstraintSolver] Ignoring contact with non-finite geometry "
            << "(point, normal, or penetration depth). This usually indicates "
            << "a malformed collision mesh or a collision backend that "
               "produced an invalid contact.\n";
        continue;
      }

      if (collision::Contact::isZeroNormal(contact.normal)) {
        // Skip this contact. This is because we assume that a contact with
        // zero-length normal is invalid.
        continue;
      }

      // Set colliding bodies
      if (contact.collisionObject1 == nullptr
          || contact.collisionObject2 == nullptr) {
        dtwarn << "[ConstraintSolver] Ignoring contact with a null collision "
               << "object.\n";
        continue;
      }

      const auto* shapeNode1 = contact.collisionObject1->getShapeNode();
      const auto* shapeNode2 = contact.collisionObject2->getShapeNode();
      auto* bodyNode1 = contact.collisionObject1->getBodyNode();
      auto* bodyNode2 = contact.collisionObject2->getBodyNode();
      if (shapeNode1 == nullptr || shapeNode2 == nullptr || bodyNode1 == nullptr
          || bodyNode2 == nullptr) {
        dtwarn << "[ConstraintSolver] Ignoring contact with a missing "
               << "ShapeNode or BodyNode.\n";
        continue;
      }

      DART_SUPPRESS_DEPRECATED_BEGIN
      bodyNode1->setColliding(true);
      bodyNode2->setColliding(true);
      DART_SUPPRESS_DEPRECATED_END

      // If penetration depth is negative, then the collision isn't really
      // happening and the contact point should be ignored.
      // TODO(MXG): Investigate ways to leverage the proximity information of a
      //            negative penetration to improve collision handling.
      if (contact.penetrationDepth < 0.0)
        continue;

      if (isSoftContact(bodyNode1, bodyNode2)) {
        SoftContactConstraintPtr softContactConstraint;
        while (!mReusableSoftContactConstraints.empty()) {
          softContactConstraint
              = std::move(mReusableSoftContactConstraints.back());
          mReusableSoftContactConstraints.pop_back();
          if (softContactConstraint != nullptr)
            break;
        }

        if (softContactConstraint != nullptr) {
          softContactConstraint->reset(contact, mTimeStep);
        } else {
          softContactConstraint
              = std::make_shared<SoftContactConstraint>(contact, mTimeStep);
        }

        mSoftContactConstraints.push_back(std::move(softContactConstraint));
      } else {
        const std::size_t contactPairIndex = findContactPairIndex(
            contact.collisionObject1,
            contact.collisionObject2,
            bodyNode1,
            bodyNode2,
            shapeNode1,
            shapeNode2);
        auto& contactPairCount = contactPairCounts[contactPairIndex];
        ++contactPairCount.count;

        const std::size_t candidateIndex = contactCandidates.size();
        contactCandidates.push_back(
            {&contact, contactPairIndex, invalidContactPairIndex});
        if (contactPairCount.firstCandidateIndex == invalidContactPairIndex) {
          contactPairCount.firstCandidateIndex = candidateIndex;
        } else {
          contactCandidates[contactPairCount.lastCandidateIndex]
              .nextCandidateIndex
              = candidateIndex;
        }
        contactPairCount.lastCandidateIndex = candidateIndex;
      }
    }
  }

  // Add the new contact constraints to dynamic constraint list
  {
    std::size_t reusableContactConstraintIndex = 0u;
    const auto createDefaultContactConstraint =
        [&](collision::Contact& contact,
            const ContactSurfaceParams& surfaceParams) -> ContactConstraintPtr {
      while (reusableContactConstraintIndex
             < mReusableContactConstraints.size()) {
        auto contactConstraint = std::move(
            mReusableContactConstraints[reusableContactConstraintIndex++]);
        if (contactConstraint == nullptr)
          continue;

        if (!isExactDynamicType<ContactConstraint>(contactConstraint.get()))
          continue;

        contactConstraint->reset(contact, mTimeStep, surfaceParams);
        return contactConstraint;
      }

      return builtInDefaultContactHandler
          ->DefaultContactSurfaceHandler::createConstraint(
              contact, mTimeStep, surfaceParams);
    };

    struct DefaultSurfaceCacheEntry
    {
      const dynamics::ShapeNode* shapeNode{nullptr};
      std::size_t shapeNodeVersion{std::numeric_limits<std::size_t>::max()};
      std::size_t updateEpoch{0u};
      bool hasDefaultProperties{false};
    };
    static thread_local std::array<DefaultSurfaceCacheEntry, 8192u>
        defaultSurfaceCache{};
    static thread_local std::size_t defaultSurfaceCacheEpoch = 0u;
    ++defaultSurfaceCacheEpoch;
    // LCOV_EXCL_START: requires wrapping size_t solver-update epochs.
    if (defaultSurfaceCacheEpoch == 0u) {
      std::fill(
          defaultSurfaceCache.begin(),
          defaultSurfaceCache.end(),
          DefaultSurfaceCacheEntry{});
      ++defaultSurfaceCacheEpoch;
    }
    // LCOV_EXCL_STOP

    const auto queryDefaultContactSurfacePropertiesUncached
        = [&](const dynamics::ShapeNode* shapeNode) {
            if (shapeNode == nullptr)
              return false;

            const auto* dynamicAspect = shapeNode->getDynamicsAspect();
            return dynamicAspect != nullptr
                   && dynamicAspect->getRestitutionCoeff()
                          == DART_DEFAULT_RESTITUTION_COEFF
                   && dynamicAspect->getPrimaryFrictionCoeff()
                          == DART_DEFAULT_FRICTION_COEFF
                   && dynamicAspect->getSecondaryFrictionCoeff()
                          == DART_DEFAULT_FRICTION_COEFF
                   && dynamicAspect->getPrimarySlipCompliance() == -1.0
                   && dynamicAspect->getSecondarySlipCompliance() == -1.0
                   && dynamicAspect->getFirstFrictionDirectionFrame() == nullptr
                   && dynamicAspect->getFirstFrictionDirection().squaredNorm()
                          < DART_CONTACT_CONSTRAINT_EPSILON_SQUARED;
          };

    const auto queryDefaultContactSurfaceProperties
        = [&](const dynamics::ShapeNode* shapeNode) {
            if (shapeNode == nullptr)
              return false;

            const auto shapeNodeVersion = shapeNode->getVersion();
            auto shapeNodeKey = static_cast<std::uint64_t>(
                reinterpret_cast<std::uintptr_t>(shapeNode) >> 4u);
            shapeNodeKey ^= shapeNodeKey >> 33u;
            shapeNodeKey *= 0xff51afd7ed558ccdULL;
            shapeNodeKey ^= shapeNodeKey >> 33u;
            auto& entry = defaultSurfaceCache
                [static_cast<std::size_t>(shapeNodeKey)
                 % defaultSurfaceCache.size()];
            if (entry.updateEpoch == defaultSurfaceCacheEpoch
                && entry.shapeNode == shapeNode
                && entry.shapeNodeVersion == shapeNodeVersion) {
              return entry.hasDefaultProperties;
            }

            const auto* dynamicAspect = shapeNode->getDynamicsAspect();
            const bool hasDefaultProperties
                = dynamicAspect != nullptr
                  && dynamicAspect->getRestitutionCoeff()
                         == DART_DEFAULT_RESTITUTION_COEFF
                  && dynamicAspect->getPrimaryFrictionCoeff()
                         == DART_DEFAULT_FRICTION_COEFF
                  && dynamicAspect->getSecondaryFrictionCoeff()
                         == DART_DEFAULT_FRICTION_COEFF
                  && dynamicAspect->getPrimarySlipCompliance() == -1.0
                  && dynamicAspect->getSecondarySlipCompliance() == -1.0
                  && dynamicAspect->getFirstFrictionDirectionFrame() == nullptr
                  && dynamicAspect->getFirstFrictionDirection().squaredNorm()
                         < DART_CONTACT_CONSTRAINT_EPSILON_SQUARED;

            entry
                = {shapeNode,
                   shapeNodeVersion,
                   defaultSurfaceCacheEpoch,
                   hasDefaultProperties};
            return hasDefaultProperties;
          };
    const dynamics::ShapeNode* lastContactShapeNode1 = nullptr;
    const dynamics::ShapeNode* lastContactShapeNode2 = nullptr;
    bool lastContactShapeNode1HasDefaultProperties = false;
    bool lastContactShapeNode2HasDefaultProperties = false;
    const auto hasDefaultContactSurfaceProperties
        = [&](const dynamics::ShapeNode* shapeNode, bool firstShapeNode) {
            auto& lastShapeNode = firstShapeNode ? lastContactShapeNode1
                                                 : lastContactShapeNode2;
            auto& lastHasDefaultProperties
                = firstShapeNode ? lastContactShapeNode1HasDefaultProperties
                                 : lastContactShapeNode2HasDefaultProperties;
            if (shapeNode == lastShapeNode)
              return lastHasDefaultProperties;

            const bool hasDefaultProperties
                = queryDefaultContactSurfaceProperties(shapeNode);
            lastShapeNode = shapeNode;
            lastHasDefaultProperties = hasDefaultProperties;
            return hasDefaultProperties;
          };
    const auto canUseDefaultSurfaceParamsForContactPair
        = [&](const ContactPairCount& contactPairCount) {
            return useBuiltInDefaultSurfaceParamsCache
                   && hasDefaultContactSurfaceProperties(
                       contactPairCount.shapeNode1, true)
                   && hasDefaultContactSurfaceProperties(
                       contactPairCount.shapeNode2, false);
          };
    const auto ensureDefaultSurfaceParamsChecked
        = [&](ContactPairCount& contactPairCount) {
            if (!contactPairCount.defaultSurfaceParamsChecked) {
              contactPairCount.canUseDefaultSurfaceParams
                  = canUseDefaultSurfaceParamsForContactPair(contactPairCount);
              contactPairCount.defaultSurfaceParamsChecked = true;
            }
            return contactPairCount.canUseDefaultSurfaceParams;
          };

    const auto initializeDefaultSurfaceParams =
        [&](ContactPairCount& contactPairCount, collision::Contact& contact) {
          if (contactPairCount.surfaceParamsInitialized)
            return;

          contactPairCount.surfaceParams
              = ensureDefaultSurfaceParamsChecked(contactPairCount)
                    ? ContactSurfaceParams()
                    : builtInDefaultContactHandler
                          ->DefaultContactSurfaceHandler::createParams(
                              contact, contactPairCount.count);
          const auto contactCount = static_cast<double>(contactPairCount.count);
          contactPairCount.surfaceParams.mPrimarySlipCompliance *= contactCount;
          contactPairCount.surfaceParams.mSecondarySlipCompliance
              *= contactCount;
          contactPairCount.surfaceParamsInitialized = true;
        };
    const auto initializeDefaultSurfaceParamsUncached =
        [&](ContactPairCount& contactPairCount) {
          if (contactPairCount.surfaceParamsInitialized)
            return;

          const bool canUseDefaultSurfaceParams
              = contactPairCount.defaultSurfaceParamsChecked
                && contactPairCount.canUseDefaultSurfaceParams;
          DART_ASSERT(canUseDefaultSurfaceParams);
          DART_UNUSED(canUseDefaultSurfaceParams);
          contactPairCount.surfaceParams = ContactSurfaceParams();

          const auto contactCount = static_cast<double>(contactPairCount.count);
          contactPairCount.surfaceParams.mPrimarySlipCompliance *= contactCount;
          contactPairCount.surfaceParams.mSecondarySlipCompliance
              *= contactCount;
          contactPairCount.surfaceParamsInitialized = true;
        };

    const auto activateContactConstraint =
        [&](const ContactConstraintPtr& contactConstraint) {
          if (contactConstraint == nullptr)
            return;

          if (useBuiltInDefaultContactActiveState) {
            contactConstraint->mActive = contactConstraint->mIsReactiveA
                                         || contactConstraint->mIsReactiveB;
          } else {
            contactConstraint->update();
          }

          const bool isActive = useBuiltInDefaultContactActiveState
                                    ? contactConstraint->mActive
                                    : contactConstraint->isActive();
          if (isActive) {
            if (!isExactDynamicType<ContactConstraint>(contactConstraint.get()))
              mActiveConstraintsHaveCustomContactConstraint = true;

            if (contactConstraint->getSingleReactiveSkeleton() == nullptr) {
              mActiveConstraintsAllSingleReactiveContacts = false;
            } else {
              const bool nonReactiveSideIsSkipped
                  = (contactConstraint->mIsReactiveA
                     && contactConstraint->mSkipRelVelocityB)
                    || (contactConstraint->mIsReactiveB
                        && contactConstraint->mSkipRelVelocityA);
              const bool nonReactiveSideIsFixed
                  = (contactConstraint->mIsReactiveA
                     && contactConstraint->mSkeletonB != nullptr
                     && !contactConstraint->mSkeletonB->isMobile())
                    || (contactConstraint->mIsReactiveB
                        && contactConstraint->mSkeletonA != nullptr
                        && !contactConstraint->mSkeletonA->isMobile());
              if (!nonReactiveSideIsSkipped && !nonReactiveSideIsFixed)
                mActiveSingleReactiveContactsNeedSharedDependencyScan = true;
            }
            mActiveConstraints.push_back(contactConstraint);
          }
        };

    constexpr std::size_t kParallelDefaultSurfacePrepassMinPairs = 1024u;

    bool parallelDefaultContactBuildNeedsSurfaceParamsPrepass = false;
    const auto canBuildDefaultContactsByPairInParallel = [&]() {
      parallelDefaultContactBuildNeedsSurfaceParamsPrepass = false;
      if (!useBuiltInDefaultSurfaceParamsCache
          || mConstraintThreadPool == nullptr || mNumSimulationThreads <= 1u
          || contactCandidates.size() < 512u || contactPairCounts.size() < 64u
          || mReusableContactConstraints.size() < contactCandidates.size()) {
        return false;
      }

      for (std::size_t i = 0u; i < contactCandidates.size(); ++i) {
        if (mReusableContactConstraints[i] == nullptr)
          return false;

        if (!isExactDynamicType<ContactConstraint>(
                mReusableContactConstraints[i].get())) {
          return false;
        }
      }

      static thread_local std::vector<const dynamics::BodyNode*>
          sharedBodyBuckets;
      static thread_local std::vector<const dynamics::BodyNode*>
          skipRelVelocityBodyBuckets;
      bool needsSurfaceParamsPrepass = false;

      const auto resetBodyBuckets =
          [](std::vector<const dynamics::BodyNode*>& buckets,
             std::size_t minimumBucketCount) {
            std::size_t bucketCount = 2u;
            while (bucketCount < minimumBucketCount)
              bucketCount <<= 1u;
            if (buckets.size() < bucketCount)
              buckets.resize(bucketCount, nullptr);
            std::fill(buckets.begin(), buckets.begin() + bucketCount, nullptr);
            return bucketCount - 1u;
          };
      const auto findBodyInBuckets
          = [](const std::vector<const dynamics::BodyNode*>& buckets,
               const std::size_t bucketMask,
               const dynamics::BodyNode* bodyNode) {
              const auto bodyNodeKey
                  = reinterpret_cast<std::uintptr_t>(bodyNode) >> 4u;
              std::size_t bucket
                  = std::hash<std::uintptr_t>()(bodyNodeKey) & bucketMask;
              while (true) {
                const auto* existingBodyNode = buckets[bucket];
                if (existingBodyNode == nullptr)
                  return false;

                if (existingBodyNode == bodyNode)
                  return true;

                bucket = (bucket + 1u) & bucketMask;
              }
            };
      const auto recordBodyIfUnique
          = [&](std::vector<const dynamics::BodyNode*>& buckets,
                const std::size_t bucketMask,
                const dynamics::BodyNode* bodyNode) {
              if (findBodyInBuckets(buckets, bucketMask, bodyNode))
                return false;

              const auto bodyNodeKey
                  = reinterpret_cast<std::uintptr_t>(bodyNode) >> 4u;
              std::size_t bucket
                  = std::hash<std::uintptr_t>()(bodyNodeKey) & bucketMask;
              while (buckets[bucket] != nullptr)
                bucket = (bucket + 1u) & bucketMask;
              buckets[bucket] = bodyNode;
              return true;
            };
      const std::size_t sharedBodyBucketMask
          = resetBodyBuckets(sharedBodyBuckets, contactPairCounts.size() * 4u);
      const std::size_t skipRelVelocityBodyBucketMask = resetBodyBuckets(
          skipRelVelocityBodyBuckets, contactPairCounts.size() * 4u);
      const auto recordSharedBodyIfUnique
          = [&](const dynamics::BodyNode* bodyNode) {
              return recordBodyIfUnique(
                  sharedBodyBuckets, sharedBodyBucketMask, bodyNode);
            };

      struct ParallelBodyCheckResult
      {
        bool canBuild;
        bool skipRelVelocity;
      };
      const auto checkBodyForParallelDefaultContact
          = [&](const dynamics::BodyNode* bodyNode) -> ParallelBodyCheckResult {
        if (bodyNode == nullptr)
          return {true, false};

        if (findBodyInBuckets(
                skipRelVelocityBodyBuckets,
                skipRelVelocityBodyBucketMask,
                bodyNode)) {
          return {true, true};
        }

        const auto* skeleton = bodyNode->getSkeletonRawPtr();
        const bool fixedZeroVelocitySupport
            = skeleton != nullptr && !skeleton->isMobile()
              && !bodyNode->isReactive()
              && bodyNode->getSpatialVelocity().squaredNorm()
                     < DART_CONTACT_CONSTRAINT_EPSILON_SQUARED;
        if (fixedZeroVelocitySupport) {
          recordBodyIfUnique(
              skipRelVelocityBodyBuckets,
              skipRelVelocityBodyBucketMask,
              bodyNode);
          return {true, true};
        }

        if (bodyNode->getNumDependentGenCoords() == 0u)
          return {true, false};

        return {recordSharedBodyIfUnique(bodyNode), false};
      };

      if (contactPairCounts.size() < kParallelDefaultSurfacePrepassMinPairs) {
        {
          for (auto& contactPairCount : contactPairCounts) {
            const auto body1Check = checkBodyForParallelDefaultContact(
                contactPairCount.bodyNode1);
            const auto body2Check = checkBodyForParallelDefaultContact(
                contactPairCount.bodyNode2);
            if (!body1Check.canBuild || !body2Check.canBuild)
              return false;

            contactPairCount.skipRelVelocityBody1 = body1Check.skipRelVelocity;
            contactPairCount.skipRelVelocityBody2 = body2Check.skipRelVelocity;
          }
        }

        {
          for (auto& contactPairCount : contactPairCounts) {
            if (!ensureDefaultSurfaceParamsChecked(contactPairCount))
              needsSurfaceParamsPrepass = true;
          }
        }
      } else {
        {
          for (auto& contactPairCount : contactPairCounts) {
            const auto body1Check = checkBodyForParallelDefaultContact(
                contactPairCount.bodyNode1);
            const auto body2Check = checkBodyForParallelDefaultContact(
                contactPairCount.bodyNode2);
            if (!body1Check.canBuild || !body2Check.canBuild)
              return false;

            contactPairCount.skipRelVelocityBody1 = body1Check.skipRelVelocity;
            contactPairCount.skipRelVelocityBody2 = body2Check.skipRelVelocity;
          }
        }

        {
          static thread_local std::vector<char> surfaceParamsPrepassScratch;
          surfaceParamsPrepassScratch.resize(contactPairCounts.size());

          auto* contactPairCountsForSurfaceScan = &contactPairCounts;
          auto* surfaceParamsPrepassScratchForScan
              = &surfaceParamsPrepassScratch;
          auto scanDefaultSurfaceParams = [&](std::size_t pairIndex) {
            auto& contactPairCount
                = (*contactPairCountsForSurfaceScan)[pairIndex];
            const bool canUseDefaultSurfaceParams
                = useBuiltInDefaultSurfaceParamsCache
                  && queryDefaultContactSurfacePropertiesUncached(
                      contactPairCount.shapeNode1)
                  && queryDefaultContactSurfacePropertiesUncached(
                      contactPairCount.shapeNode2);
            contactPairCount.canUseDefaultSurfaceParams
                = canUseDefaultSurfaceParams;
            contactPairCount.defaultSurfaceParamsChecked = true;
            (*surfaceParamsPrepassScratchForScan)[pairIndex]
                = canUseDefaultSurfaceParams ? 0 : 1;
          };

          mConstraintThreadPool->parallelFor(
              contactPairCountsForSurfaceScan->size(),
              mNumSimulationThreads,
              scanDefaultSurfaceParams);

          for (const char needsPrepass : surfaceParamsPrepassScratch) {
            if (needsPrepass) {
              needsSurfaceParamsPrepass = true;
              break;
            }
          }
        }
      }

      parallelDefaultContactBuildNeedsSurfaceParamsPrepass
          = needsSurfaceParamsPrepass;
      return true;
    };

    const bool useParallelDefaultContactBuild
        = canBuildDefaultContactsByPairInParallel();
    if (useBuiltInDefaultSurfaceParamsCache
        && (!useParallelDefaultContactBuild
            || parallelDefaultContactBuildNeedsSurfaceParamsPrepass)) {
      for (auto& contactPairCount : contactPairCounts) {
        if (contactPairCount.firstCandidateIndex == invalidContactPairIndex)
          continue;

        initializeDefaultSurfaceParams(
            contactPairCount,
            *contactCandidates[contactPairCount.firstCandidateIndex].contact);
      }
    }

    if (useParallelDefaultContactBuild) {
      mContactConstraints.resize(contactCandidates.size());
      auto* contactPairCountsForParallel = &contactPairCounts;
      auto* contactCandidatesForParallel = &contactCandidates;
      auto buildDefaultContactConstraintsForPair = [&](std::size_t pairIndex) {
        auto& contactPairCount = (*contactPairCountsForParallel)[pairIndex];
        if (contactPairCount.firstCandidateIndex == invalidContactPairIndex)
          return;

        initializeDefaultSurfaceParamsUncached(contactPairCount);

        for (auto i = contactPairCount.firstCandidateIndex;
             i != invalidContactPairIndex;
             i = (*contactCandidatesForParallel)[i].nextCandidateIndex) {
          const auto& candidate = (*contactCandidatesForParallel)[i];

          ContactConstraintPtr contactConstraint;
          if (i < mReusableContactConstraints.size()) {
            auto reusableContactConstraint
                = std::move(mReusableContactConstraints[i]);
            if (reusableContactConstraint != nullptr
                && isExactDynamicType<ContactConstraint>(
                    reusableContactConstraint.get())) {
              contactConstraint = std::move(reusableContactConstraint);
            }
          }

          if (contactConstraint != nullptr) {
            const bool contactUsesPairOrder
                = candidate.contact->collisionObject1
                  == contactPairCount.pair.first;
            DART_ASSERT(
                contactUsesPairOrder
                || candidate.contact->collisionObject1
                       == contactPairCount.pair.second);
            DART_ASSERT(
                candidate.contact->collisionObject2
                == (contactUsesPairOrder ? contactPairCount.pair.second
                                         : contactPairCount.pair.first));
            const bool skipRelVelocityA
                = contactUsesPairOrder ? contactPairCount.skipRelVelocityBody1
                                       : contactPairCount.skipRelVelocityBody2;
            const bool skipRelVelocityB
                = contactUsesPairOrder ? contactPairCount.skipRelVelocityBody2
                                       : contactPairCount.skipRelVelocityBody1;
            contactConstraint->reset(
                *candidate.contact,
                mTimeStep,
                contactPairCount.surfaceParams,
                skipRelVelocityA,
                skipRelVelocityB);
          } else {
            contactConstraint
                = builtInDefaultContactHandler
                      ->DefaultContactSurfaceHandler::createConstraint(
                          *candidate.contact,
                          mTimeStep,
                          contactPairCount.surfaceParams);
          }

          mContactConstraints[i] = std::move(contactConstraint);
        }
      };

      {
        mConstraintThreadPool->parallelFor(
            contactPairCountsForParallel->size(),
            mNumSimulationThreads,
            buildDefaultContactConstraintsForPair);
      }

      for (const auto& contactConstraint : mContactConstraints)
        activateContactConstraint(contactConstraint);
    } else {
      for (const auto& candidate : contactCandidates) {
        ContactConstraintPtr contactConstraint;
        if (useBuiltInDefaultSurfaceParamsCache) {
          auto& contactPairCount
              = contactPairCounts[candidate.contactPairIndex];
          initializeDefaultSurfaceParams(contactPairCount, *candidate.contact);

          contactConstraint = createDefaultContactConstraint(
              *candidate.contact, contactPairCount.surfaceParams);
        } else {
          auto& contactPairCount
              = contactPairCounts[candidate.contactPairIndex];
          const auto numContactsOnPair = contactPairCount.count;
          contactConstraint
              = builtInDefaultContactHandler != nullptr
                    ? builtInDefaultContactHandler
                          ->DefaultContactSurfaceHandler ::createConstraint(
                              *candidate.contact, numContactsOnPair, mTimeStep)
                    : mContactSurfaceHandler->createConstraint(
                        *candidate.contact, numContactsOnPair, mTimeStep);
        }
        if (contactConstraint == nullptr)
          continue;

        mContactConstraints.push_back(contactConstraint);
        activateContactConstraint(contactConstraint);
      }
    }

    mReusableContactConstraints.clear();
  }

  // Add the new soft contact constraints to dynamic constraint list
  {
    for (const auto& softContactConstraint : mSoftContactConstraints) {
      softContactConstraint->update();

      if (softContactConstraint->isActive()) {
        mActiveConstraintsAllSingleReactiveContacts = false;
        mActiveConstraints.push_back(softContactConstraint);
      }
    }
    mReusableSoftContactConstraints.clear();
  }

  //----------------------------------------------------------------------------
  // Update automatic constraints: joint constraints
  //----------------------------------------------------------------------------
  // Destroy previous joint constraints
  mJointConstraints.clear();
  mMimicMotorConstraints.clear();
  mCouplerConstraints.clear();
  mJointCoulombFrictionConstraints.clear();

  {
    updateAutomaticJointConstraintCache();
    for (auto* joint : mAutomaticJointConstraintJoints) {
      if (joint->hasCoulombFriction()) {
        mJointCoulombFrictionConstraints.push_back(
            std::make_shared<JointCoulombFrictionConstraint>(joint));
      }

      if (joint->areLimitsEnforced()
          || joint->hasActuatorType(dynamics::Joint::SERVO)) {
        mJointConstraints.push_back(std::make_shared<JointConstraint>(joint));
      }

      if (joint->hasActuatorType(dynamics::Joint::MIMIC)) {
        auto mimicProps = joint->getMimicDofProperties();
        const auto dofCount = joint->getNumDofs();
        bool hasValidMimicDof = false;
        for (std::size_t dofIndex = 0;
             dofIndex < dofCount && dofIndex < mimicProps.size();
             ++dofIndex) {
          if (joint->getActuatorType(dofIndex) == dynamics::Joint::MIMIC) {
            if (mimicProps[dofIndex].mReferenceJoint != nullptr) {
              hasValidMimicDof = true;
            } else {
              DART_WARN(
                  "Joint '{}' DoF {} is set to MIMIC without a reference; "
                  "mimic constraint will be skipped.",
                  joint->getName(),
                  dofIndex);
            }
          }
        }

        if (hasValidMimicDof) {
          if (joint->isUsingCouplerConstraint()) {
            mCouplerConstraints.push_back(std::make_shared<CouplerConstraint>(
                joint, joint->getMimicDofProperties()));
          } else {
            mMimicMotorConstraints.push_back(
                std::make_shared<MimicMotorConstraint>(
                    joint, joint->getMimicDofProperties()));
          }
        }
      }
    }
  }

  // Add active joint limit
  {
    for (auto& jointLimitConstraint : mJointConstraints) {
      jointLimitConstraint->update();

      if (jointLimitConstraint->isActive()) {
        mActiveConstraintsAllSingleReactiveContacts = false;
        mActiveConstraints.push_back(jointLimitConstraint);
      }
    }

    for (auto& mimicMotorConstraint : mMimicMotorConstraints) {
      mimicMotorConstraint->update();

      if (mimicMotorConstraint->isActive()) {
        mActiveConstraintsAllSingleReactiveContacts = false;
        mActiveConstraints.push_back(mimicMotorConstraint);
      }
    }

    for (auto& couplerConstraint : mCouplerConstraints) {
      couplerConstraint->update();

      if (couplerConstraint->isActive()) {
        mActiveConstraintsAllSingleReactiveContacts = false;
        mActiveConstraints.push_back(couplerConstraint);
      }
    }

    for (auto& jointFrictionConstraint : mJointCoulombFrictionConstraints) {
      jointFrictionConstraint->update();

      if (jointFrictionConstraint->isActive()) {
        mActiveConstraintsAllSingleReactiveContacts = false;
        mActiveConstraints.push_back(jointFrictionConstraint);
      }
    }
  }
}

//==============================================================================
void ConstraintSolver::buildConstrainedGroups()
{
  // Clear constrained groups while retaining per-group constraint-vector
  // capacity for the steady-state case where thousands of small contact groups
  // are rebuilt every step.
  for (auto& group : mConstrainedGroups) {
    group.removeAllConstraints();
    group.mRootSkeleton.reset();
    group.mAllSingleReactiveContacts = false;
    group.mAllExactContactConstraints = false;
    group.mSingleReactiveContactsShareBody = true;
    group.mSingleReactiveBodyNode = nullptr;
    group.mSingleReactiveSkeleton = nullptr;
  }
  mGroupResting.clear();
  mGroupAllSleepCandidates.clear();
  mGroupPreserveSleepCandidates.clear();

  // Exit if there is no active constraint
  if (mActiveConstraints.empty()) {
    mConstrainedGroups.clear();

    // With no active constraints, no island can be frozen. Clear any stale
    // freeze flags so a body that just lost all of its contacts resumes
    // simulating (e.g. a stack member that was kicked away). Preserve the
    // sleep-candidate dwell state here: World::updateRestStates() and the
    // pre-solve wake-band check clear stale candidates only when the body is
    // actually disturbed or moving again, which makes sleeping robust to
    // one-frame contact misses from collision jitter.
    if (mDeactivationActive && mHadDeactivationGroups) {
      if (mCollisionResult.getNumContacts() > 0) {
        static thread_local std::unordered_set<const dynamics::Skeleton*>
            contactedSkeletons;
        contactedSkeletons.clear();
        contactedSkeletons.reserve(mCollisionResult.getNumContacts());

        for (std::size_t i = 0; i < mCollisionResult.getNumContacts(); ++i) {
          const auto& contact = mCollisionResult.getContact(i);
          const auto bodyNode1 = contact.getBodyNodePtr1();
          const auto bodyNode2 = contact.getBodyNodePtr2();

          if (bodyNode1) {
            const auto skeleton = bodyNode1->getSkeleton();
            if (skeleton && skeleton->isMobile())
              contactedSkeletons.insert(skeleton.get());
          }

          if (bodyNode2) {
            const auto skeleton = bodyNode2->getSkeleton();
            if (skeleton && skeleton->isMobile())
              contactedSkeletons.insert(skeleton.get());
          }
        }

        for (auto& skeleton : mSkeletons) {
          if (contactedSkeletons.find(skeleton.get())
              != contactedSkeletons.end()) {
            continue;
          }

          if (skeleton->isResting() || skeleton->getIslandIndex() >= 0) {
            skeleton->setResting(false);
            skeleton->setIslandIndex(-1);
          }
        }

        recordConstrainedGroupProfileCounters();
        return;
      }

      bool preservedRestingIsland = false;
      for (auto& skeleton : mSkeletons) {
        const bool canRemainResting = skeleton->isResting()
                                      && skeleton->isSleepCandidate()
                                      && skeleton->getIslandIndex() >= 0
                                      && !skeleton->hasExternalDisturbance();

        if (canRemainResting) {
          preservedRestingIsland = true;
          continue;
        }

        if (skeleton->isResting() || skeleton->getIslandIndex() >= 0) {
          skeleton->setResting(false);
          skeleton->setIslandIndex(-1);
        }
      }

      if (preservedRestingIsland) {
        recordConstrainedGroupProfileCounters();
        return;
      }

      for (auto& skeleton : mSkeletons) {
        if (skeleton->isResting() || skeleton->getIslandIndex() >= 0) {
          skeleton->setResting(false);
          skeleton->setIslandIndex(-1);
        }
      }
    }
    mHadDeactivationGroups = false;
    recordConstrainedGroupProfileCounters();
    return;
  }

  //----------------------------------------------------------------------------
  // Unite skeletons according to constraints's relationships
  //----------------------------------------------------------------------------
  const bool allConstraintsHaveSingleReactiveSkeleton
      = mActiveConstraintsAllSingleReactiveContacts;
  if (!allConstraintsHaveSingleReactiveSkeleton) {
    for (const auto& activeConstraint : mActiveConstraints) {
      if (const auto* contact
          = dynamic_cast<const ContactConstraint*>(activeConstraint.get())) {
        if (contact->getSingleReactiveSkeleton() != nullptr)
          continue;
      }

      activeConstraint->uniteSkeletons();
    }
  }

  //----------------------------------------------------------------------------
  // Build constraint groups
  //----------------------------------------------------------------------------
  const std::size_t invalidUnionIndex = static_cast<std::size_t>(-1);
  std::size_t numConstrainedGroups = 0u;
  {
    if (allConstraintsHaveSingleReactiveSkeleton) {
      for (auto& activeConstraint : mActiveConstraints) {
        const auto* contact
            = static_cast<const ContactConstraint*>(activeConstraint.get());
        const bool exactContactConstraint
            = !mActiveConstraintsHaveCustomContactConstraint
              || isExactDynamicType<ContactConstraint>(contact);
        auto* skel = contact->getSingleReactiveSkeleton();
        auto* bodyNode = contact->getSingleReactiveBodyNode();
        DART_ASSERT(skel != nullptr);
        DART_ASSERT(bodyNode != nullptr);

        auto groupIndex = skel->mUnionIndex;

        if (groupIndex == invalidUnionIndex) {
          groupIndex = numConstrainedGroups;
          if (numConstrainedGroups == mConstrainedGroups.size())
            mConstrainedGroups.emplace_back();

          auto& group = mConstrainedGroups[groupIndex];
          group.mRootSkeleton = skel->getPtr();
          group.mAllSingleReactiveContacts = true;
          group.mAllExactContactConstraints = exactContactConstraint;
          group.mSingleReactiveContactsShareBody = true;
          group.mSingleReactiveBodyNode = bodyNode;
          group.mSingleReactiveSkeleton = skel;
          skel->mUnionIndex = groupIndex;
          ++numConstrainedGroups;
        } else {
          auto& group = mConstrainedGroups[groupIndex];
          if (group.mSingleReactiveContactsShareBody
              && group.mSingleReactiveBodyNode != bodyNode) {
            group.mSingleReactiveContactsShareBody = false;
            group.mSingleReactiveBodyNode = nullptr;
          }
          group.mAllExactContactConstraints
              = group.mAllExactContactConstraints && exactContactConstraint;
        }

        DART_ASSERT(activeConstraint->isActive());
        mConstrainedGroups[groupIndex].mConstraints.push_back(
            std::move(activeConstraint));
      }
    } else {
      for (auto& activeConstraint : mActiveConstraints) {
        const auto& skel = activeConstraint->getRootSkeleton();
        auto groupIndex = skel->mUnionIndex;

        if (groupIndex == invalidUnionIndex) {
          groupIndex = numConstrainedGroups;
          if (numConstrainedGroups == mConstrainedGroups.size())
            mConstrainedGroups.emplace_back();

          mConstrainedGroups[groupIndex].mRootSkeleton = skel;
          skel->mUnionIndex = groupIndex;
          ++numConstrainedGroups;
        }

        DART_ASSERT(activeConstraint->isActive());
        mConstrainedGroups[groupIndex].mConstraints.push_back(
            std::move(activeConstraint));
      }
    }

    mConstrainedGroups.resize(numConstrainedGroups);
  }

  const bool needsGroupMobileSkeletonCounts
      = mDeactivationActive
        || (!ContactConstraint::mMaxErrorReductionVelocityUserConfigured
            && ContactConstraint::mMaxErrorReductionVelocity
                   > kSmallContactIslandMaxErrorReductionVelocity);
  if (needsGroupMobileSkeletonCounts) {
    mGroupMobileSkeletonCountScratch.assign(mConstrainedGroups.size(), 0u);
    for (const auto& skeleton : mSkeletons) {
      if (!skeleton->isMobile())
        continue;

      const auto root = ConstraintBase::getRootSkeleton(skeleton);
      const auto groupIndex = root->mUnionIndex;
      if (groupIndex != invalidUnionIndex
          && groupIndex < mGroupMobileSkeletonCountScratch.size()) {
        ++mGroupMobileSkeletonCountScratch[groupIndex];
      }
    }
  }

  if (!ContactConstraint::mMaxErrorReductionVelocityUserConfigured
      && ContactConstraint::mMaxErrorReductionVelocity
             > kSmallContactIslandMaxErrorReductionVelocity) {
    for (std::size_t i = 0; i < mConstrainedGroups.size(); ++i) {
      bool hasMobileMobileContact = false;
      for (const auto& constraint : mConstrainedGroups[i].mConstraints) {
        const auto* contact
            = dynamic_cast<ContactConstraint*>(constraint.get());
        if (!contact)
          continue;

        const auto bodyNode1 = contact->getContact().getBodyNodePtr1();
        const auto bodyNode2 = contact->getContact().getBodyNodePtr2();
        const auto* skeleton1
            = bodyNode1 ? bodyNode1->getSkeletonRawPtr() : nullptr;
        const auto* skeleton2
            = bodyNode2 ? bodyNode2->getSkeletonRawPtr() : nullptr;
        if (skeleton1 != nullptr && skeleton2 != nullptr
            && skeleton1 != skeleton2 && skeleton1->isMobile()
            && skeleton2->isMobile()) {
          hasMobileMobileContact = true;
          break;
        }
      }

      const auto mobileSkeletonCount = mGroupMobileSkeletonCountScratch[i];
      const bool denseMobileIsland
          = mobileSkeletonCount >= kDenseContactIslandMinMobileSkeletons;
      const double effectiveMaxErrorReductionVelocity
          = denseMobileIsland && hasMobileMobileContact
                ? ContactConstraint::mMaxErrorReductionVelocity
                : kSmallContactIslandMaxErrorReductionVelocity;
      for (const auto& constraint : mConstrainedGroups[i].mConstraints) {
        auto* contact = dynamic_cast<ContactConstraint*>(constraint.get());
        if (contact) {
          contact->mEffectiveMaxErrorReductionVelocity
              = effectiveMaxErrorReductionVelocity;
        }
      }
    }
  }

  //----------------------------------------------------------------------------
  // Determine which islands are fully at rest, and set the island-atomic
  // freeze flag on every skeleton.
  //
  // An island may be frozen (its LCP skipped, and its members' gravity /
  // integration skipped in World::step) ONLY when EVERY skeleton in it is a
  // sleep candidate. This is computed here, while the union-find information is
  // still valid (before resetUnion below). Because a moving (awake) body that
  // contacts a resting island is united into the same group, that group will
  // contain a non-candidate member, so it is not frozen and IS solved -
  // delivering the wake-up impulse to the previously-resting bodies.
  //
  // Setting mIsResting island-atomically (rather than per skeleton) is what
  // keeps the freeze consistent: a body is never frozen in World::step while a
  // constraint-coupled neighbour is still moving, so gravity is never skipped
  // for a body whose island LCP still runs.
  //----------------------------------------------------------------------------
  if (mDeactivationActive) {
    mHadDeactivationGroups = !mConstrainedGroups.empty();
    mGroupResting.assign(mConstrainedGroups.size(), true);
    mGroupAllSleepCandidates.assign(mConstrainedGroups.size(), true);
    mGroupPreserveSleepCandidates.assign(mConstrainedGroups.size(), true);

    // Only rigid contact islands whose penetration correction has essentially
    // converged are eligible for automatic sleeping. Other constraints (joint
    // limits, motors, explicit dynamic joint constraints, soft contacts, etc.)
    // expose solver forces as part of their observable behavior; keeping those
    // islands awake preserves existing query semantics when sleeping is enabled
    // by default.
    const double sleepContactPenetrationTolerance
        = getAutomaticSleepingContactPenetrationTolerance();
    const bool useDenseIslandSleepContactPenetrationTolerance
        = !gSleepContactPenetrationToleranceUserConfigured
          && sleepContactPenetrationTolerance
                 == kDefaultSleepContactPenetrationTolerance;
    {
      for (std::size_t i = 0; i < mConstrainedGroups.size(); ++i) {
        const auto& group = mConstrainedGroups[i];
        const bool denseContactIsland
            = i < mGroupMobileSkeletonCountScratch.size()
              && mGroupMobileSkeletonCountScratch[i]
                     >= kDenseContactIslandMinMobileSkeletons;
        const double groupSleepContactPenetrationTolerance
            = useDenseIslandSleepContactPenetrationTolerance
                      && denseContactIsland
                  ? kDenseContactIslandSleepContactPenetrationTolerance
                  : sleepContactPenetrationTolerance;
        for (std::size_t j = 0; j < group.mConstraints.size(); ++j) {
          const auto* constraint = group.mConstraints[j].get();
          const auto* contact
              = dynamic_cast<const ContactConstraint*>(constraint);
          if (!contact) {
            mGroupResting[i] = false;
            mGroupPreserveSleepCandidates[i] = false;
            break;
          }

          const bool usePlaneShapeSleepContactPenetrationTolerance
              = useDenseIslandSleepContactPenetrationTolerance
                && contactTouchesPlaneShape(contact->getContact());
          const double contactSleepContactPenetrationTolerance
              = usePlaneShapeSleepContactPenetrationTolerance
                    ? kDenseContactIslandSleepContactPenetrationTolerance
                    : groupSleepContactPenetrationTolerance;
          const double preserveSleepCandidatePenetrationTolerance
              = contactSleepContactPenetrationTolerance;

          if (contact->getContact().penetrationDepth
              > contactSleepContactPenetrationTolerance) {
            mGroupResting[i] = false;
            if (contact->getContact().penetrationDepth
                > preserveSleepCandidatePenetrationTolerance) {
              mGroupPreserveSleepCandidates[i] = false;
              break;
            }
          }
        }
      }
    }

    {
      bool hasUngroupedAwakeMobileSkeleton = false;
      for (const auto& skeleton : mSkeletons) {
        if (!skeleton->isMobile())
          continue;

        const auto root = ConstraintBase::getRootSkeleton(skeleton);
        const auto groupIndex = root->mUnionIndex;
        const bool grouped = groupIndex != invalidUnionIndex
                             && groupIndex < mGroupResting.size();
        if (!grouped && !skeleton->isResting()
            && !skeleton->isSleepCandidate()) {
          hasUngroupedAwakeMobileSkeleton = true;
          break;
        }
      }

      // Pass 1: an island can freeze only if every member is a sleep candidate.
      // Keep that kinematic condition separate from constraint/contact
      // eligibility. A contact whose penetration exceeds the automatic
      // sleeping tolerance clears candidacy so a later freeze cannot preserve
      // unconverged contact state.
      for (const auto& skeleton : mSkeletons) {
        const auto root = ConstraintBase::getRootSkeleton(skeleton);
        const auto groupIndex = root->mUnionIndex;
        if (groupIndex == invalidUnionIndex
            || groupIndex >= mGroupResting.size()) {
          continue; // not in any active-constraint island
        }

        const bool sleepCandidate = skeleton->isSleepCandidate();
        mGroupAllSleepCandidates[groupIndex]
            = mGroupAllSleepCandidates[groupIndex] && sleepCandidate;
        mGroupResting[groupIndex] = mGroupResting[groupIndex] && sleepCandidate;
      }

      // Pass 2: stamp the island index and keep only already-frozen islands
      // frozen. A newly eligible island must run one final contact solve before
      // it freezes so observable force caches (e.g. transmitted wrench queries)
      // keep the last solved constraint forces instead of an unconstrained
      // forward-dynamics value.
      for (const auto& skeleton : mSkeletons) {
        const auto root = ConstraintBase::getRootSkeleton(skeleton);
        const auto groupIndex = root->mUnionIndex;
        const bool grouped = groupIndex != invalidUnionIndex
                             && groupIndex < mGroupResting.size();
        const bool groupAllSleepCandidates
            = grouped && groupIndex < mGroupAllSleepCandidates.size()
              && mGroupAllSleepCandidates[groupIndex];
        const bool groupCanRest
            = grouped && mGroupResting[groupIndex]
              && (!hasUngroupedAwakeMobileSkeleton || skeleton->isResting());
        const bool groupCanPreserveSleepCandidate
            = groupAllSleepCandidates && !hasUngroupedAwakeMobileSkeleton
              && groupIndex < mGroupPreserveSleepCandidates.size()
              && mGroupPreserveSleepCandidates[groupIndex];
        if (grouped && !groupCanRest && !groupCanPreserveSleepCandidate
            && skeleton->isSleepCandidate()) {
          skeleton->setSleepCandidate(false);
        }
        skeleton->setResting(groupCanRest && skeleton->isResting());
        skeleton->setIslandIndex(grouped ? static_cast<int>(groupIndex) : -1);
      }
    }
  }

  //----------------------------------------------------------------------------
  // Reset union since we don't need union information anymore.
  //----------------------------------------------------------------------------
  {
    for (auto& skeleton : mSkeletons)
      skeleton->resetUnion();
  }

  recordConstrainedGroupProfileCounters();
}

//==============================================================================
void ConstraintSolver::recordConstrainedGroupProfileCounters() const
{
#if DART_BUILD_PROFILE
  if (!dart::common::profile::isProfileRecordingEnabled())
    return;

  std::size_t totalConstraints = 0u;
  std::size_t totalContactConstraints = 0u;
  std::size_t totalBodies = 0u;
  std::size_t totalRows = 0u;
  std::size_t maxConstraints = 0u;
  std::size_t maxBodies = 0u;
  std::size_t maxRows = 0u;
  std::size_t rowBuckets[6] = {};
  std::size_t bodyBuckets[6] = {};
  std::size_t contactBuckets[6] = {};

  static thread_local std::unordered_set<const dynamics::BodyNode*> groupBodies;

  const auto bucketIndex = [](std::size_t value) {
    if (value == 0u)
      return 0u;
    if (value <= 3u)
      return 1u;
    if (value <= 12u)
      return 2u;
    if (value <= 48u)
      return 3u;
    if (value <= 192u)
      return 4u;
    return 5u;
  };

  const auto addBody = [](std::unordered_set<const dynamics::BodyNode*>& bodies,
                          const dynamics::BodyNode* body) {
    if (body != nullptr)
      bodies.insert(body);
  };

  for (const auto& group : mConstrainedGroups) {
    groupBodies.clear();

    std::size_t contactConstraints = 0u;
    for (const auto& constraint : group.mConstraints) {
      if (constraint == nullptr)
        continue;

      if (const auto* contact
          = dynamic_cast<const ContactConstraint*>(constraint.get())) {
        ++contactConstraints;
        addBody(groupBodies, contact->mBodyNodeA);
        addBody(groupBodies, contact->mBodyNodeB);
        continue;
      }

      if (const auto* softContact
          = dynamic_cast<const SoftContactConstraint*>(constraint.get())) {
        ++contactConstraints;
        addBody(groupBodies, softContact->mBodyNode1);
        addBody(groupBodies, softContact->mBodyNode2);
        continue;
      }

      if (const auto* dynamicJoint
          = dynamic_cast<const DynamicJointConstraint*>(constraint.get())) {
        addBody(groupBodies, dynamicJoint->getBodyNode1());
        addBody(groupBodies, dynamicJoint->getBodyNode2());
        continue;
      }

      if (const auto* joint
          = dynamic_cast<const JointConstraint*>(constraint.get())) {
        addBody(groupBodies, joint->mBodyNode);
        continue;
      }

      if (const auto* mimicMotor
          = dynamic_cast<const MimicMotorConstraint*>(constraint.get())) {
        addBody(groupBodies, mimicMotor->mBodyNode);
        continue;
      }

      if (const auto* coupler
          = dynamic_cast<const CouplerConstraint*>(constraint.get())) {
        addBody(groupBodies, coupler->mBodyNode);
        for (const auto& mimicProp : coupler->mMimicProps) {
          if (mimicProp.mReferenceJoint != nullptr) {
            addBody(groupBodies, mimicProp.mReferenceJoint->getChildBodyNode());
          }
        }
        continue;
      }

      if (const auto* jointFriction
          = dynamic_cast<const JointCoulombFrictionConstraint*>(
              constraint.get())) {
        addBody(groupBodies, jointFriction->mBodyNode);
      }
    }

    const std::size_t constraints = group.mConstraints.size();
    const std::size_t bodies = groupBodies.size();
    const std::size_t rows = group.getTotalDimension();

    totalConstraints += constraints;
    totalContactConstraints += contactConstraints;
    totalBodies += bodies;
    totalRows += rows;
    maxConstraints = std::max(maxConstraints, constraints);
    maxBodies = std::max(maxBodies, bodies);
    maxRows = std::max(maxRows, rows);
    ++rowBuckets[bucketIndex(rows)];
    ++bodyBuckets[bucketIndex(bodies)];
    ++contactBuckets[bucketIndex(contactConstraints)];
  }

  DART_PROFILE_COUNTER_N(
      "ConstraintSolver island groups", mConstrainedGroups.size());
  DART_PROFILE_COUNTER_N(
      "ConstraintSolver island constraints", totalConstraints);
  DART_PROFILE_COUNTER_N(
      "ConstraintSolver island contact constraints", totalContactConstraints);
  DART_PROFILE_COUNTER_N("ConstraintSolver island bodies", totalBodies);
  DART_PROFILE_COUNTER_N("ConstraintSolver island LCP rows", totalRows);
  DART_PROFILE_COUNTER_N(
      "ConstraintSolver island max constraints", maxConstraints);
  DART_PROFILE_COUNTER_N("ConstraintSolver island max bodies", maxBodies);
  DART_PROFILE_COUNTER_N("ConstraintSolver island max LCP rows", maxRows);
  DART_PROFILE_COUNTER_N("ConstraintSolver island rows 0", rowBuckets[0]);
  DART_PROFILE_COUNTER_N("ConstraintSolver island rows 1-3", rowBuckets[1]);
  DART_PROFILE_COUNTER_N("ConstraintSolver island rows 4-12", rowBuckets[2]);
  DART_PROFILE_COUNTER_N("ConstraintSolver island rows 13-48", rowBuckets[3]);
  DART_PROFILE_COUNTER_N("ConstraintSolver island rows 49-192", rowBuckets[4]);
  DART_PROFILE_COUNTER_N("ConstraintSolver island rows 193+", rowBuckets[5]);
  DART_PROFILE_COUNTER_N("ConstraintSolver island bodies 0", bodyBuckets[0]);
  DART_PROFILE_COUNTER_N("ConstraintSolver island bodies 1-3", bodyBuckets[1]);
  DART_PROFILE_COUNTER_N("ConstraintSolver island bodies 4-12", bodyBuckets[2]);
  DART_PROFILE_COUNTER_N(
      "ConstraintSolver island bodies 13-48", bodyBuckets[3]);
  DART_PROFILE_COUNTER_N(
      "ConstraintSolver island bodies 49-192", bodyBuckets[4]);
  DART_PROFILE_COUNTER_N("ConstraintSolver island bodies 193+", bodyBuckets[5]);
  DART_PROFILE_COUNTER_N(
      "ConstraintSolver island contacts 0", contactBuckets[0]);
  DART_PROFILE_COUNTER_N(
      "ConstraintSolver island contacts 1-3", contactBuckets[1]);
  DART_PROFILE_COUNTER_N(
      "ConstraintSolver island contacts 4-12", contactBuckets[2]);
  DART_PROFILE_COUNTER_N(
      "ConstraintSolver island contacts 13-48", contactBuckets[3]);
  DART_PROFILE_COUNTER_N(
      "ConstraintSolver island contacts 49-192", contactBuckets[4]);
  DART_PROFILE_COUNTER_N(
      "ConstraintSolver island contacts 193+", contactBuckets[5]);
#endif
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
void ConstraintSolver::solvePositionConstrainedGroup(
    ConstrainedGroup& /*group*/)
{
  // Base implementation does nothing. Concrete solvers that support split
  // impulse override this.
}

//==============================================================================
void ConstraintSolver::solvePositionConstrainedGroups()
{
  // Preserve velocity-impulse flags across the position pass.
  std::vector<bool> impulseAppliedStates;
  impulseAppliedStates.reserve(mSkeletons.size());
  for (const auto& skeleton : mSkeletons) {
    impulseAppliedStates.push_back(skeleton->isImpulseApplied());
  }

  for (auto& constraintGroup : mConstrainedGroups) {
    solvePositionConstrainedGroup(constraintGroup);
  }

  for (auto& skeleton : mSkeletons) {
    if (skeleton->isPositionImpulseApplied()) {
      skeleton->computePositionVelocityChanges();
    }
  }

  for (std::size_t i = 0; i < mSkeletons.size(); ++i) {
    mSkeletons[i]->setImpulseApplied(impulseAppliedStates[i]);
  }
}

//==============================================================================
bool ConstraintSolver::canSolveConstrainedGroupsInParallel() const
{
  auto hasCustomContactConstraint = [&]() {
    for (const auto& group : mConstrainedGroups) {
      for (std::size_t i = 0; i < group.getNumConstraints(); ++i) {
        const auto* constraint = group.getConstraint(i).get();
        const auto* contact
            = dynamic_cast<const ContactConstraint*>(constraint);
        if (contact != nullptr
            && !isExactDynamicType<ContactConstraint>(contact))
          return true;
      }
    }

    return false;
  };

  auto hasSharedNonReactiveDependency = [&]() {
    std::unordered_map<const dynamics::BodyNode*, std::size_t> bodyToGroup;
    std::unordered_map<const dynamics::Skeleton*, std::size_t>
        touchedSkeletonToGroup;
    std::unordered_map<const dynamics::Skeleton*, std::size_t>
        nonReactiveSkeletonToGroup;
    constexpr std::size_t kExemptFixedContactSupportGroup
        = std::numeric_limits<std::size_t>::max();

    auto touchesSharedDependency = [&](const dynamics::BodyNode* body,
                                       std::size_t groupIndex) {
      if (body == nullptr)
        return false;

      const auto skeleton = body->getSkeleton().get();
      const bool reactive = body->isReactive();
      const bool sharedFixedContactSupport
          = !reactive && skeleton != nullptr && !skeleton->isMobile()
            && groupIndex < mConstrainedGroups.size()
            && mConstrainedGroups[groupIndex].mAllSingleReactiveContacts;
      if (sharedFixedContactSupport) {
        const auto recordExemptSupport
            = [&](auto& dependencyToGroup, const auto* dependency) {
                const auto result = dependencyToGroup.emplace(
                    dependency, kExemptFixedContactSupportGroup);
                if (result.second)
                  return false;

                const auto recordedGroup = result.first->second;
                return recordedGroup != kExemptFixedContactSupportGroup
                       && recordedGroup != groupIndex;
              };

        if (recordExemptSupport(bodyToGroup, body))
          return true;

        if (skeleton != nullptr
            && (recordExemptSupport(touchedSkeletonToGroup, skeleton)
                || recordExemptSupport(nonReactiveSkeletonToGroup, skeleton))) {
          return true;
        }

        return false;
      }

      if (skeleton != nullptr) {
        const auto nonReactiveSkeleton
            = nonReactiveSkeletonToGroup.find(skeleton);
        if (nonReactiveSkeleton != nonReactiveSkeletonToGroup.end()
            && nonReactiveSkeleton->second != groupIndex) {
          return true;
        }

        const auto touchedSkeleton
            = touchedSkeletonToGroup.emplace(skeleton, groupIndex);
        if (!reactive && !touchedSkeleton.second
            && touchedSkeleton.first->second != groupIndex) {
          return true;
        }
      }

      if (reactive)
        return false;

      const auto bodyResult = bodyToGroup.emplace(body, groupIndex);
      if (!bodyResult.second && bodyResult.first->second != groupIndex)
        return true;

      if (skeleton == nullptr)
        return false;

      const auto skeletonResult
          = nonReactiveSkeletonToGroup.emplace(skeleton, groupIndex);
      return !skeletonResult.second
             && skeletonResult.first->second != groupIndex;
    };

    for (std::size_t groupIndex = 0; groupIndex < mConstrainedGroups.size();
         ++groupIndex) {
      const auto& group = mConstrainedGroups[groupIndex];
      for (std::size_t i = 0; i < group.getNumConstraints(); ++i) {
        const auto* constraint = group.getConstraint(i).get();

        if (const auto* contact
            = dynamic_cast<const ContactConstraint*>(constraint)) {
          if (touchesSharedDependency(contact->mBodyNodeA, groupIndex)
              || touchesSharedDependency(contact->mBodyNodeB, groupIndex)) {
            return true;
          }
        }

        if (const auto* softContact
            = dynamic_cast<const SoftContactConstraint*>(constraint)) {
          if (touchesSharedDependency(softContact->mBodyNode1, groupIndex)
              || touchesSharedDependency(softContact->mBodyNode2, groupIndex)) {
            return true;
          }
        }

        if (const auto* dynamicJoint
            = dynamic_cast<const DynamicJointConstraint*>(constraint)) {
          if (touchesSharedDependency(dynamicJoint->getBodyNode1(), groupIndex)
              || touchesSharedDependency(
                  dynamicJoint->getBodyNode2(), groupIndex)) {
            return true;
          }
        }

        if (const auto* joint
            = dynamic_cast<const JointConstraint*>(constraint)) {
          if (touchesSharedDependency(joint->mBodyNode, groupIndex))
            return true;
        }

        if (const auto* mimicMotor
            = dynamic_cast<const MimicMotorConstraint*>(constraint)) {
          if (touchesSharedDependency(mimicMotor->mBodyNode, groupIndex))
            return true;
        }

        if (const auto* jointFriction
            = dynamic_cast<const JointCoulombFrictionConstraint*>(constraint)) {
          if (touchesSharedDependency(jointFriction->mBodyNode, groupIndex))
            return true;
        }
      }
    }

    return false;
  };

  auto canSolveGroupsInParallel = [&]() {
    const bool allGroupsAreSingleReactiveContacts = std::all_of(
        mConstrainedGroups.begin(),
        mConstrainedGroups.end(),
        [](const ConstrainedGroup& group) {
          return group.mAllSingleReactiveContacts;
        });
    const bool canSkipCustomContactScan
        = mActiveConstraintsAllSingleReactiveContacts
          && allGroupsAreSingleReactiveContacts
          && !mActiveConstraintsHaveCustomContactConstraint;
    const bool canSkipSharedDependencyScan
        = mActiveConstraintsAllSingleReactiveContacts
          && allGroupsAreSingleReactiveContacts
          && !mActiveSingleReactiveContactsNeedSharedDependencyScan;

    return mConstraintThreadPool != nullptr && mNumSimulationThreads > 1u
           && mConstrainedGroups.size() >= 128u && mManualConstraints.empty()
           && canUseParallelBuiltInBoxedSolvers(*this)
           && (canSkipCustomContactScan || !hasCustomContactConstraint())
           && (canSkipSharedDependencyScan
               || !hasSharedNonReactiveDependency());
  };

  return canSolveGroupsInParallel();
}

//==============================================================================
void ConstraintSolver::solveConstrainedGroups()
{
  auto solveGroupAt = [&](std::size_t i) {
    solveConstrainedGroup(mConstrainedGroups[i]);
  };

  auto solveGroupsSerial = [&](const auto& solve) {
    for (std::size_t i = 0; i < mConstrainedGroups.size(); ++i)
      solve(i);
  };

  if (!mDeactivationActive) {
    if (canSolveConstrainedGroupsInParallel()) {
      mConstraintThreadPool->parallelFor(
          mConstrainedGroups.size(), mNumSimulationThreads, solveGroupAt);
    } else {
      solveGroupsSerial(solveGroupAt);
    }

    return;
  }

  mGroupAlreadyRestingScratch.assign(mConstrainedGroups.size(), 1);
  mGroupSolvedToRestScratch.assign(mConstrainedGroups.size(), 0);

  for (const auto& skeleton : mSkeletons) {
    const int island = skeleton->getIslandIndex();
    if (island < 0)
      continue;

    const auto groupIndex = static_cast<std::size_t>(island);
    if (groupIndex >= mGroupAlreadyRestingScratch.size())
      continue;

    if (!skeleton->isResting())
      mGroupAlreadyRestingScratch[groupIndex] = 0;
  }

  auto solveDeactivationGroupAt = [&](std::size_t i) {
    const bool groupCanRest = i < mGroupResting.size() && mGroupResting[i];

    // Skip only islands that were already frozen at the start of this solve. A
    // newly eligible island still needs one final solve to refresh the contact
    // impulse and body-force caches before it is frozen for subsequent steps.
    if (groupCanRest && mGroupAlreadyRestingScratch[i])
      return;

    solveConstrainedGroup(mConstrainedGroups[i]);

    if (groupCanRest)
      mGroupSolvedToRestScratch[i] = 1;
  };

  if (canSolveConstrainedGroupsInParallel()) {
    mConstraintThreadPool->parallelFor(
        mConstrainedGroups.size(),
        mNumSimulationThreads,
        solveDeactivationGroupAt);
  } else {
    solveGroupsSerial(solveDeactivationGroupAt);
  }

  for (const auto& skeleton : mSkeletons) {
    const int island = skeleton->getIslandIndex();
    if (island < 0)
      continue;

    const auto groupIndex = static_cast<std::size_t>(island);
    if (groupIndex < mGroupSolvedToRestScratch.size()
        && mGroupSolvedToRestScratch[groupIndex]) {
      skeleton->setResting(true);
    }
  }
}

//==============================================================================
void ConstraintSolver::reserveConstrainedGroupsScratch()
{
  const auto groupCount = mConstrainedGroups.size();
  mGroupResting.reserve(groupCount);
  mGroupAllSleepCandidates.reserve(groupCount);
  mGroupPreserveSleepCandidates.reserve(groupCount);
  mGroupMobileSkeletonCountScratch.reserve(groupCount);
  mGroupAlreadyRestingScratch.reserve(groupCount);
  mGroupSolvedToRestScratch.reserve(groupCount);

  for (const auto& group : mConstrainedGroups) {
    reserveConstrainedGroupScratch(group);
  }

  if (!canSolveConstrainedGroupsInParallel()) {
    return;
  }

  const std::size_t participantCount
      = std::min<std::size_t>(mNumSimulationThreads, mConstrainedGroups.size());
  if (participantCount <= 1u)
    return;

  auto reserveAllGroupScratchForThread = [&](std::size_t) {
    for (const auto& group : mConstrainedGroups)
      reserveConstrainedGroupScratch(group);
  };

  mConstraintThreadPool->parallelFor(
      participantCount, participantCount, reserveAllGroupScratchForThread);
}

//==============================================================================
void ConstraintSolver::reserveConstrainedGroupScratch(
    const ConstrainedGroup& /*group*/)
{
  // Base solvers do not own extra per-group scratch.
}

//==============================================================================
bool ConstraintSolver::isSoftContact(
    const dynamics::BodyNode* bodyNode1,
    const dynamics::BodyNode* bodyNode2) const
{
  DART_ASSERT(bodyNode1);
  DART_ASSERT(bodyNode2);

  const auto bodyNode1IsSoft = bodyNode1->asSoftBodyNode() != nullptr;
  const auto bodyNode2IsSoft = bodyNode2->asSoftBodyNode() != nullptr;

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
    dterr << "Adding the same contact surface handler for the second time, "
          << "ignoring.\n";
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
      if (previous != nullptr)
        previous->mParent = current->mParent;
      else
        mContactSurfaceHandler = current->mParent;
      found = true;
      break;
    }
    previous = current;
    current = current->mParent;
  }

  if (mContactSurfaceHandler == nullptr)
    dterr << "No contact surface handler remained. This is an error. Add at "
          << "least DefaultContactSurfaceHandler." << std::endl;

  return found;
}

} // namespace constraint
} // namespace dart
