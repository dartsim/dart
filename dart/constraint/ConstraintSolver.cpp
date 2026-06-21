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
#include "dart/dynamics/Skeleton.hpp"
#include "dart/dynamics/SoftBodyNode.hpp"

#include <algorithm>
#include <condition_variable>
#include <limits>
#include <mutex>
#include <thread>
#include <type_traits>
#include <typeinfo>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include <cstdint>

namespace dart {
namespace constraint {

using namespace dynamics;

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

  cd->setPrimitiveShapeType(collision::FCLCollisionDetector::MESH);
  // TODO(JS): Consider using FCL's primitive shapes once FCL addresses
  // incorrect contact point computation.
  // (see: https://github.com/flexible-collision-library/fcl/issues/106)
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

  cd->setPrimitiveShapeType(collision::FCLCollisionDetector::MESH);
  // TODO(JS): Consider using FCL's primitive shapes once FCL addresses
  // incorrect contact point computation.
  // (see: https://github.com/flexible-collision-library/fcl/issues/106)
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
void ConstraintSolver::setNumSimulationThreads(std::size_t numThreads)
{
  if (numThreads == 0u) {
    numThreads = std::thread::hardware_concurrency();
    if (numThreads == 0u)
      numThreads = 1u;
  }

  mNumSimulationThreads = std::max<std::size_t>(1u, numThreads);
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
  DART_PROFILE_SCOPED_N("ConstraintSolver::solve");

  {
    DART_PROFILE_SCOPED_N("ConstraintSolver::clear per-step state");
    auto clearSkeletonAt = [&](std::size_t i) {
      auto& skeleton = mSkeletons[i];
      skeleton->clearConstraintImpulses();
      DART_SUPPRESS_DEPRECATED_BEGIN
      skeleton->clearCollidingBodies();
      DART_SUPPRESS_DEPRECATED_END
    };

    if (mConstraintThreadPool != nullptr && mNumSimulationThreads > 1u
        && mSkeletons.size() >= 128u) {
      DART_PROFILE_SCOPED_N("parallel clear per-step state");
      mConstraintThreadPool->parallelFor(
          mSkeletons.size(), mNumSimulationThreads, clearSkeletonAt);
    } else {
      for (std::size_t i = 0; i < mSkeletons.size(); ++i)
        clearSkeletonAt(i);
    }
  }

  // Update constraints and collect active constraints
  updateConstraints();

  // Build constrained groups
  buildConstrainedGroups();

  // Solve constrained groups
  solveConstrainedGroups();
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
  setNumSimulationThreads(other.getNumSimulationThreads());
}

//==============================================================================
bool ConstraintSolver::canJointCreateAutomaticConstraint(
    const dynamics::Joint* joint) const
{
  if (joint == nullptr || joint->isKinematic())
    return false;

  return joint->hasCoulombFriction() || joint->areLimitsEnforced()
         || joint->getActuatorType() == dynamics::Joint::SERVO
         || (joint->getActuatorType() == dynamics::Joint::MIMIC
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
void ConstraintSolver::updateConstraints()
{
  DART_PROFILE_SCOPED;

  // Clear previous active constraint list
  mActiveConstraints.clear();
  mActiveConstraintsAllSingleReactiveContacts = true;

  //----------------------------------------------------------------------------
  // Update manual constraints
  //----------------------------------------------------------------------------
  {
    DART_PROFILE_SCOPED_N("update manual constraints");
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
    DART_PROFILE_SCOPED_N("collide");
    mCollisionGroup->collide(mCollisionOption, &mCollisionResult);
  }

  if (restingContactFilter != nullptr)
    restingContactFilter->setSolverRestingContactFilterActive(false, false);

  // Destroy previous contact constraints
  mContactConstraints.clear();

  // Destroy previous soft contact constraints
  mSoftContactConstraints.clear();

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

  // Create a mapping of contact pairs to the number of contacts between them.
  // The scratch table uses open addressing over retained vectors so the
  // per-step contact-pair count remains order-independent without allocating
  // one unordered_map node per contact pair.
  struct ContactPair
  {
    collision::CollisionObject* first;
    collision::CollisionObject* second;
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
    std::size_t count;
    ContactSurfaceParams surfaceParams;
    bool surfaceParamsInitialized;
  };
  struct ContactCandidate
  {
    collision::Contact* contact;
    std::size_t contactPairIndex;
  };

  constexpr std::size_t invalidContactPairIndex
      = (std::numeric_limits<std::size_t>::max)();
  static thread_local std::vector<ContactPairCount> contactPairCounts;
  contactPairCounts.clear();
  static thread_local std::vector<std::size_t> contactPairBuckets;

  static thread_local std::vector<ContactCandidate> contactCandidates;
  contactCandidates.clear();

  {
    DART_PROFILE_SCOPED_N("collect contact candidates");
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

    const auto findOrCreateContactPairIndex
        = [&](collision::CollisionObject* object1,
              collision::CollisionObject* object2) -> std::size_t {
      initializeContactPairBuckets();
      ContactPair pair{object1, object2};
      if (reinterpret_cast<std::uintptr_t>(pair.first)
          > reinterpret_cast<std::uintptr_t>(pair.second)) {
        std::swap(pair.first, pair.second);
      }

      const std::size_t bucketMask = contactPairBuckets.size() - 1u;
      std::size_t bucket = contactPairHash(pair) & bucketMask;
      while (true) {
        const std::size_t pairIndex = contactPairBuckets[bucket];
        if (pairIndex == invalidContactPairIndex) {
          const std::size_t newPairIndex = contactPairCounts.size();
          contactPairCounts.push_back(
              {pair, 0u, ContactSurfaceParams(), false});
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

    for (auto i = 0u; i < mCollisionResult.getNumContacts(); ++i) {
      auto& contact = mCollisionResult.getContact(i);

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
        mSoftContactConstraints.push_back(
            std::make_shared<SoftContactConstraint>(contact, mTimeStep));
      } else {
        const std::size_t contactPairIndex = findOrCreateContactPairIndex(
            contact.collisionObject1, contact.collisionObject2);
        ++contactPairCounts[contactPairIndex].count;

        contactCandidates.push_back({&contact, contactPairIndex});
      }
    }
  }

  // Add the new contact constraints to dynamic constraint list
  {
    DART_PROFILE_SCOPED_N("build contact constraints");
    for (const auto& candidate : contactCandidates) {
      ContactConstraintPtr contactConstraint;
      if (useBuiltInDefaultSurfaceParamsCache) {
        auto& contactPairCount = contactPairCounts[candidate.contactPairIndex];
        const auto numContactsOnPair = contactPairCount.count;
        if (!contactPairCount.surfaceParamsInitialized) {
          contactPairCount.surfaceParams
              = builtInDefaultContactHandler
                    ->DefaultContactSurfaceHandler::createParams(
                        *candidate.contact, numContactsOnPair);
          const auto contactCount = static_cast<double>(numContactsOnPair);
          contactPairCount.surfaceParams.mPrimarySlipCompliance *= contactCount;
          contactPairCount.surfaceParams.mSecondarySlipCompliance
              *= contactCount;
          contactPairCount.surfaceParamsInitialized = true;
        }

        contactConstraint
            = builtInDefaultContactHandler
                  ->DefaultContactSurfaceHandler::createConstraint(
                      *candidate.contact,
                      mTimeStep,
                      contactPairCount.surfaceParams);
      } else {
        auto& contactPairCount = contactPairCounts[candidate.contactPairIndex];
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
        if (contactConstraint->getSingleReactiveSkeleton() == nullptr)
          mActiveConstraintsAllSingleReactiveContacts = false;
        mActiveConstraints.push_back(contactConstraint);
      }
    }
  }

  // Add the new soft contact constraints to dynamic constraint list
  {
    DART_PROFILE_SCOPED_N("update soft contact constraints");
    for (const auto& softContactConstraint : mSoftContactConstraints) {
      softContactConstraint->update();

      if (softContactConstraint->isActive()) {
        mActiveConstraintsAllSingleReactiveContacts = false;
        mActiveConstraints.push_back(softContactConstraint);
      }
    }
  }

  //----------------------------------------------------------------------------
  // Update automatic constraints: joint constraints
  //----------------------------------------------------------------------------
  // Destroy previous joint constraints
  mJointConstraints.clear();
  mMimicMotorConstraints.clear();
  mJointCoulombFrictionConstraints.clear();

  {
    DART_PROFILE_SCOPED_N("scan joint constraints");
    updateAutomaticJointConstraintCache();
    for (auto* joint : mAutomaticJointConstraintJoints) {
      if (joint->hasCoulombFriction()) {
        mJointCoulombFrictionConstraints.push_back(
            std::make_shared<JointCoulombFrictionConstraint>(joint));
      }

      if (joint->areLimitsEnforced()
          || joint->getActuatorType() == dynamics::Joint::SERVO) {
        mJointConstraints.push_back(std::make_shared<JointConstraint>(joint));
      }

      if (joint->getActuatorType() == dynamics::Joint::MIMIC
          && joint->getMimicJoint()) {
        mMimicMotorConstraints.push_back(std::make_shared<MimicMotorConstraint>(
            joint, joint->getMimicDofProperties()));
      }
    }
  }

  // Add active joint limit
  {
    DART_PROFILE_SCOPED_N("update joint constraints");
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
  DART_PROFILE_SCOPED;

  // Clear constrained groups while retaining per-group constraint-vector
  // capacity for the steady-state case where thousands of small contact groups
  // are rebuilt every step.
  for (auto& group : mConstrainedGroups) {
    group.removeAllConstraints();
    group.mRootSkeleton.reset();
    group.mAllSingleReactiveContacts = false;
    group.mSingleReactiveContactsShareBody = true;
    group.mSingleReactiveBodyNode = nullptr;
    group.mSingleReactiveSkeleton = nullptr;
  }
  mGroupResting.clear();

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

      if (preservedRestingIsland)
        return;

      for (auto& skeleton : mSkeletons) {
        if (skeleton->isResting() || skeleton->getIslandIndex() >= 0) {
          skeleton->setResting(false);
          skeleton->setIslandIndex(-1);
        }
      }
    }
    mHadDeactivationGroups = false;
    return;
  }

  //----------------------------------------------------------------------------
  // Unite skeletons according to constraints's relationships
  //----------------------------------------------------------------------------
  const bool allConstraintsHaveSingleReactiveSkeleton
      = mActiveConstraintsAllSingleReactiveContacts;
  if (!allConstraintsHaveSingleReactiveSkeleton) {
    DART_PROFILE_SCOPED_N("unite active constraints");
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
    DART_PROFILE_SCOPED_N("build constrained group map");
    if (allConstraintsHaveSingleReactiveSkeleton) {
      for (auto& activeConstraint : mActiveConstraints) {
        const auto* contact
            = static_cast<const ContactConstraint*>(activeConstraint.get());
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

    // Only rigid contact islands whose penetration correction has essentially
    // converged are eligible for automatic sleeping. Other constraints (joint
    // limits, motors, explicit dynamic joint constraints, soft contacts, etc.)
    // expose solver forces as part of their observable behavior; keeping those
    // islands awake preserves existing query semantics when sleeping is enabled
    // by default.
    constexpr double kSleepContactPenetrationTolerance = 1e-5;
    {
      DART_PROFILE_SCOPED_N("classify resting groups");
      for (std::size_t i = 0; i < mConstrainedGroups.size(); ++i) {
        const auto& group = mConstrainedGroups[i];
        for (std::size_t j = 0; j < group.mConstraints.size(); ++j) {
          const auto* constraint = group.mConstraints[j].get();
          const auto* contact
              = dynamic_cast<const ContactConstraint*>(constraint);
          if (!contact) {
            mGroupResting[i] = false;
            break;
          }

          if (contact->getContact().penetrationDepth
              > kSleepContactPenetrationTolerance) {
            mGroupResting[i] = false;
            break;
          }
        }
      }
    }

    {
      DART_PROFILE_SCOPED_N("stamp resting islands");
      // Pass 1: an island rests only if every member is a sleep candidate.
      for (const auto& skeleton : mSkeletons) {
        const auto root = ConstraintBase::getRootSkeleton(skeleton);
        const auto groupIndex = root->mUnionIndex;
        if (groupIndex == invalidUnionIndex
            || groupIndex >= mGroupResting.size()) {
          continue; // not in any active-constraint island
        }
        mGroupResting[groupIndex]
            = mGroupResting[groupIndex] && skeleton->isSleepCandidate();
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
        const bool groupCanRest = grouped && mGroupResting[groupIndex];
        if (grouped && !groupCanRest && skeleton->isSleepCandidate())
          skeleton->setSleepCandidate(false);
        skeleton->setResting(groupCanRest && skeleton->isResting());
        skeleton->setIslandIndex(grouped ? static_cast<int>(groupIndex) : -1);
      }
    }
  }

  //----------------------------------------------------------------------------
  // Reset union since we don't need union information anymore.
  //----------------------------------------------------------------------------
  {
    DART_PROFILE_SCOPED_N("reset constraint unions");
    for (auto& skeleton : mSkeletons)
      skeleton->resetUnion();
  }
}

//==============================================================================
void ConstraintSolver::solveConstrainedGroups()
{
  DART_PROFILE_SCOPED;

  auto solveGroupAt = [&](std::size_t i) {
    solveConstrainedGroup(mConstrainedGroups[i]);
  };

  auto solveGroupsSerial = [&](const auto& solve) {
    for (std::size_t i = 0; i < mConstrainedGroups.size(); ++i)
      solve(i);
  };

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

    auto touchesSharedDependency
        = [&](const dynamics::BodyNode* body, std::size_t groupIndex) {
            if (body == nullptr)
              return false;

            const auto skeleton = body->getSkeleton().get();
            const bool reactive = body->isReactive();
            const bool sharedFixedContactSupport
                = !reactive && skeleton != nullptr && !skeleton->isMobile()
                  && groupIndex < mConstrainedGroups.size()
                  && mConstrainedGroups[groupIndex].mAllSingleReactiveContacts;
            if (sharedFixedContactSupport) {
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
    return mConstraintThreadPool != nullptr && mNumSimulationThreads > 1u
           && mConstrainedGroups.size() >= 128u && mManualConstraints.empty()
           && canUseParallelBuiltInBoxedSolvers(*this)
           && !hasCustomContactConstraint()
           && !hasSharedNonReactiveDependency();
  };

  if (!mDeactivationActive) {
    if (canSolveGroupsInParallel()) {
      DART_PROFILE_SCOPED_N("parallel solve groups");
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

  if (canSolveGroupsInParallel()) {
    DART_PROFILE_SCOPED_N("parallel solve deactivation groups");
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
