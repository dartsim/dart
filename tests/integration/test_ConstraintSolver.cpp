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

#include "AllocationCounting.hpp"
#include "TestHelpers.hpp"
#include "dart/collision/CollisionDetector.hpp"
#include "dart/collision/CollisionObject.hpp"
#include "dart/collision/Contact.hpp"
#include "dart/collision/dart/DARTCollisionDetector.hpp"
#include "dart/collision/native/NativeCollisionDetector.hpp"
#include "dart/collision/native/NativeCollisionObject.hpp"
#include "dart/collision/native/PersistentManifoldCache.hpp"
#include "dart/common/Profile.hpp"
#include "dart/constraint/BallJointConstraint.hpp"
#include "dart/constraint/BoxedLcpConstraintSolver.hpp"
#include "dart/constraint/ConstrainedGroup.hpp"
#include "dart/constraint/ConstraintSolver.hpp"
#include "dart/constraint/ContactConstraint.hpp"
#include "dart/constraint/ContactSurface.hpp"
#include "dart/constraint/DantzigBoxedLcpSolver.hpp"
#include "dart/constraint/ExactCoulombFbfConstraintSolver.hpp"
#include "dart/constraint/JointCoulombFrictionConstraint.hpp"
#include "dart/constraint/PgsBoxedLcpSolver.hpp"
#include "dart/constraint/SoftContactConstraint.hpp"
#include "dart/constraint/detail/ExactCoulombConstraintAdapter.hpp"
#include "dart/constraint/detail/ExactCoulombContactRowOperator.hpp"
#include "dart/dynamics/BoxShape.hpp"
#include "dart/dynamics/FreeJoint.hpp"
#include "dart/dynamics/Joint.hpp"
#include "dart/dynamics/PlaneShape.hpp"
#include "dart/dynamics/ShapeFrame.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/dynamics/SoftBodyNode.hpp"
#include "dart/simulation/DeactivationOptions.hpp"
#include "dart/simulation/World.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <limits>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <type_traits>
#include <typeinfo>
#include <vector>

#include <cmath>

using namespace dart;

namespace {

class FakeConstraint final : public constraint::ConstraintBase
{
public:
  explicit FakeConstraint(std::size_t dimension)
  {
    mDim = dimension;
  }

  void update() override {}

  void getInformation(constraint::ConstraintInfo*) override {}

  void applyUnitImpulse(std::size_t) override {}

  void getVelocityChange(double*, bool) override {}

  void excite() override {}

  void unexcite() override {}

  void applyImpulse(double*) override {}

  bool isActive() const override
  {
    return true;
  }

  dynamics::SkeletonPtr getRootSkeleton() const override
  {
    return nullptr;
  }
};

class CountingManualConstraint final : public constraint::ConstraintBase
{
public:
  CountingManualConstraint()
  {
    mDim = 1u;
  }

  void update() override
  {
    ++mNumUpdates;
  }

  void getInformation(constraint::ConstraintInfo*) override {}

  void applyUnitImpulse(std::size_t) override {}

  void getVelocityChange(double*, bool) override {}

  void excite() override {}

  void unexcite() override {}

  void applyImpulse(double*) override {}

  bool isActive() const override
  {
    return false;
  }

  dynamics::SkeletonPtr getRootSkeleton() const override
  {
    return nullptr;
  }

  std::size_t getNumUpdates() const
  {
    return mNumUpdates;
  }

private:
  std::size_t mNumUpdates{0u};
};

class DiagonalConstraint final : public constraint::ConstraintBase
{
public:
  explicit DiagonalConstraint(std::size_t dimension) : mActiveImpulse(dimension)
  {
    mDim = dimension;
  }

  void update() override {}

  void getInformation(constraint::ConstraintInfo* info) override
  {
    for (std::size_t i = 0u; i < mDim; ++i) {
      info->x[i] = 0.0;
      info->lo[i] = -1.0;
      info->hi[i] = 1.0;
      info->b[i] = 0.0;
      info->w[i] = 0.0;
      info->findex[i] = -1;
    }
  }

  void applyUnitImpulse(std::size_t index) override
  {
    mActiveImpulse = index;
  }

  void getVelocityChange(double* vel, bool) override
  {
    for (std::size_t i = 0u; i < mDim; ++i)
      vel[i] = i == mActiveImpulse ? 1.0 : 0.0;
  }

  void excite() override
  {
    mActiveImpulse = mDim;
  }

  void unexcite() override
  {
    mActiveImpulse = mDim;
  }

  void applyImpulse(double*) override {}

  bool isActive() const override
  {
    return true;
  }

  dynamics::SkeletonPtr getRootSkeleton() const override
  {
    return nullptr;
  }

private:
  std::size_t mActiveImpulse;
};

class DerivedDantzigBoxedLcpSolver final
  : public constraint::DantzigBoxedLcpSolver
{
};

class DerivedPgsBoxedLcpSolver final : public constraint::PgsBoxedLcpSolver
{
};

class CountingDantzigBoxedLcpSolver final
  : public constraint::DantzigBoxedLcpSolver
{
public:
  bool solve(
      int n,
      double* A,
      double* x,
      double* b,
      int nub,
      double* lo,
      double* hi,
      int* findex,
      bool earlyTermination) override
  {
    ++mNumSolves;
    return DantzigBoxedLcpSolver::solve(
        n, A, x, b, nub, lo, hi, findex, earlyTermination);
  }

  std::size_t getNumSolves() const
  {
    return mNumSolves;
  }

private:
  std::size_t mNumSolves{0u};
};

class CustomContactConstraint final : public constraint::ContactConstraint
{
public:
  using ContactConstraint::ContactConstraint;

  ~CustomContactConstraint() override
  {
    ++mNumDestroyed;
  }

  inline static std::atomic<std::size_t> mNumDestroyed{0u};
};

class ExposedContactConstraint final : public constraint::ContactConstraint
{
public:
  using ContactConstraint::applyImpulse;
  using ContactConstraint::ContactConstraint;
  using ContactConstraint::getInformation;
};

class ExposedNativeCollisionObject final
  : public collision::NativeCollisionObject
{
public:
  ExposedNativeCollisionObject(
      collision::CollisionDetector* detector,
      const dynamics::ShapeFrame* shapeFrame)
    : NativeCollisionObject(detector, shapeFrame)
  {
  }
};

class CustomContactSurfaceHandler final
  : public constraint::DefaultContactSurfaceHandler
{
public:
  constraint::ContactConstraintPtr createConstraint(
      collision::Contact& contact,
      const size_t numContactsOnCollisionObject,
      const double timeStep) const override
  {
    ++mNumCreateConstraintCalls;

    auto params = createParams(contact, numContactsOnCollisionObject);
    const auto contactCount = static_cast<double>(numContactsOnCollisionObject);
    params.mPrimarySlipCompliance *= contactCount;
    params.mSecondarySlipCompliance *= contactCount;

    return std::make_shared<CustomContactConstraint>(contact, timeStep, params);
  }

  mutable std::size_t mNumCreateConstraintCalls{0u};
};

class FakeCollisionObject final : public collision::CollisionObject
{
public:
  FakeCollisionObject(
      collision::CollisionDetector* detector,
      const dynamics::ShapeFrame* shapeFrame)
    : collision::CollisionObject(detector, shapeFrame)
  {
  }

protected:
  void updateEngineData() override {}
};

class FakeCollisionDetector final : public collision::CollisionDetector
{
public:
  std::shared_ptr<collision::CollisionDetector> cloneWithoutCollisionObjects()
      const override
  {
    return std::make_shared<FakeCollisionDetector>();
  }

  const std::string& getType() const override
  {
    static const std::string type = "FakeCollisionDetector";
    return type;
  }

  std::unique_ptr<collision::CollisionGroup> createCollisionGroup() override
  {
    return nullptr;
  }

  bool collide(
      collision::CollisionGroup*,
      const collision::CollisionOption& = collision::CollisionOption(),
      collision::CollisionResult* = nullptr) override
  {
    return false;
  }

  bool collide(
      collision::CollisionGroup*,
      collision::CollisionGroup*,
      const collision::CollisionOption& = collision::CollisionOption(),
      collision::CollisionResult* = nullptr) override
  {
    return false;
  }

  double distance(
      collision::CollisionGroup*,
      const collision::DistanceOption& = collision::DistanceOption(),
      collision::DistanceResult* = nullptr) override
  {
    return 0.0;
  }

  double distance(
      collision::CollisionGroup*,
      collision::CollisionGroup*,
      const collision::DistanceOption& = collision::DistanceOption(),
      collision::DistanceResult* = nullptr) override
  {
    return 0.0;
  }

protected:
  std::unique_ptr<collision::CollisionObject> createCollisionObject(
      const dynamics::ShapeFrame*) override
  {
    return nullptr;
  }

  void refreshCollisionObject(collision::CollisionObject*) override {}
};

class ExposedThreadedConstraintSolver final
  : public constraint::BoxedLcpConstraintSolver
{
public:
  using BoxedLcpConstraintSolver::BoxedLcpConstraintSolver;

  void addFakeConstrainedGroups(std::size_t numGroups, std::size_t dimension)
  {
    for (std::size_t i = 0; i < numGroups; ++i) {
      constraint::ConstrainedGroup group;
      group.addConstraint(std::make_shared<FakeConstraint>(dimension));
      mConstrainedGroups.push_back(group);
    }
  }

  void addConstrainedGroup(
      const std::vector<constraint::ConstraintBasePtr>& constraints)
  {
    constraint::ConstrainedGroup group;
    for (const auto& constraint : constraints)
      group.addConstraint(constraint);
    mConstrainedGroups.push_back(group);
  }

  void setGroupRestingForTest(std::size_t groupIndex, bool resting)
  {
    if (mGroupResting.size() < mConstrainedGroups.size())
      mGroupResting.assign(mConstrainedGroups.size(), false);

    ASSERT_LT(groupIndex, mGroupResting.size());
    mGroupResting[groupIndex] = resting;
  }

  void addSkeletonForTest(const dynamics::SkeletonPtr& skeleton)
  {
    mSkeletons.push_back(skeleton);
  }

  void addActiveConstraintForTest(
      const constraint::ConstraintBasePtr& constraint)
  {
    mActiveConstraints.push_back(constraint);
    if (mActiveConstraints.size() == 1u)
      mActiveConstraintsAllSingleReactiveContacts = true;

    const auto* contact
        = dynamic_cast<const constraint::ContactConstraint*>(constraint.get());
    if (contact == nullptr) {
      mActiveConstraintsAllSingleReactiveContacts = false;
    } else if (typeid(*contact) != typeid(constraint::ContactConstraint)) {
      mActiveConstraintsHaveCustomContactConstraint = true;
    }
  }

  void buildGroupsForTest()
  {
    buildConstrainedGroups();
  }

  void solveGroupsForTest()
  {
    solveConstrainedGroups();
  }

  void reserveScratchForCurrentGroupsForTest()
  {
    reserveConstrainedGroupsScratch();
  }

  int getNumSolvedGroups() const
  {
    return mNumSolvedGroups.load(std::memory_order_relaxed);
  }

  int getMaxConcurrentSolves() const
  {
    return mMaxConcurrentSolves.load(std::memory_order_relaxed);
  }

  void recordReserveThreadsForTest()
  {
    {
      std::lock_guard<std::mutex> lock(mReserveThreadMutex);
      mReserveThreadIds.clear();
    }
    mNumReserveCalls.store(0, std::memory_order_relaxed);
    mRecordReserveThreads.store(true, std::memory_order_relaxed);
  }

  int getNumReserveCalls() const
  {
    return mNumReserveCalls.load(std::memory_order_relaxed);
  }

  std::size_t getNumReserveThreads() const
  {
    std::lock_guard<std::mutex> lock(mReserveThreadMutex);
    return mReserveThreadIds.size();
  }

protected:
  void reserveConstrainedGroupScratch(
      const constraint::ConstrainedGroup& group) override
  {
    BoxedLcpConstraintSolver::reserveConstrainedGroupScratch(group);
    if (!mRecordReserveThreads.load(std::memory_order_relaxed))
      return;

    mNumReserveCalls.fetch_add(1, std::memory_order_relaxed);
    std::lock_guard<std::mutex> lock(mReserveThreadMutex);
    mReserveThreadIds.insert(std::this_thread::get_id());
  }

  void solveConstrainedGroup(constraint::ConstrainedGroup&) override
  {
    const int concurrent
        = mConcurrentSolves.fetch_add(1, std::memory_order_relaxed) + 1;
    int observed = mMaxConcurrentSolves.load(std::memory_order_relaxed);
    while (concurrent > observed
           && !mMaxConcurrentSolves.compare_exchange_weak(
               observed, concurrent, std::memory_order_relaxed)) {
      // Keep trying with the updated observed value.
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    mNumSolvedGroups.fetch_add(1, std::memory_order_relaxed);
    mConcurrentSolves.fetch_sub(1, std::memory_order_relaxed);
  }

private:
  std::atomic<int> mConcurrentSolves{0};
  std::atomic<int> mMaxConcurrentSolves{0};
  std::atomic<int> mNumSolvedGroups{0};
  std::atomic<bool> mRecordReserveThreads{false};
  std::atomic<int> mNumReserveCalls{0};
  mutable std::mutex mReserveThreadMutex;
  std::set<std::thread::id> mReserveThreadIds;
};

class ExposedExactCoulombParallelEligibilitySolver final
  : public constraint::ExactCoulombFbfConstraintSolver
{
public:
  using ExactCoulombFbfConstraintSolver::solveConstrainedGroup;

  void addFakeConstrainedGroups(std::size_t numGroups, std::size_t dimension)
  {
    for (std::size_t i = 0; i < numGroups; ++i) {
      constraint::ConstrainedGroup group;
      group.addConstraint(std::make_shared<FakeConstraint>(dimension));
      mConstrainedGroups.push_back(group);
    }
  }

  bool canSolveConstrainedGroupsInParallelForTest() const
  {
    return canSolveConstrainedGroupsInParallel();
  }

  template <typename Work>
  std::size_t parallelForConstraintWorkForTest(std::size_t count, Work& work)
  {
    return parallelForConstraintWork(
        count, std::addressof(work), [](void* context, std::size_t index) {
          (*static_cast<Work*>(context))(index);
        });
  }
};

class ExposedBoxedLcpConstraintSolver final
  : public constraint::BoxedLcpConstraintSolver
{
public:
  using BoxedLcpConstraintSolver::BoxedLcpConstraintSolver;

  constraint::ConstrainedGroup makeGroupForTest(
      const std::vector<constraint::ConstraintBasePtr>& constraints)
  {
    constraint::ConstrainedGroup group;
    for (const auto& constraint : constraints)
      group.addConstraint(constraint);
    return group;
  }

  void reserveGroupScratchForTest(const constraint::ConstrainedGroup& group)
  {
    reserveConstrainedGroupScratch(group);
  }

  void solveGroupForTest(constraint::ConstrainedGroup& group)
  {
    solveConstrainedGroup(group);
  }
};

dynamics::BodyNode* createFreeBody(
    const std::string& name,
    bool mobile,
    std::vector<dynamics::SkeletonPtr>& skeletons)
{
  auto skeleton = dynamics::Skeleton::create(name);
  auto body
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>().second;
  skeleton->setMobile(mobile);
  skeletons.push_back(skeleton);
  return body;
}

dynamics::SoftBodyNode* createSoftBody(
    const std::string& name,
    bool mobile,
    std::vector<dynamics::SkeletonPtr>& skeletons)
{
  auto skeleton = dynamics::Skeleton::create(name);
  auto body = skeleton
                  ->createJointAndBodyNodePair<
                      dynamics::FreeJoint,
                      dynamics::SoftBodyNode>()
                  .second;
  skeleton->setMobile(mobile);
  skeletons.push_back(skeleton);
  return body;
}

dynamics::SkeletonPtr createSolverTestBox(
    const std::string& name,
    const Eigen::Vector3d& size,
    const Eigen::Vector3d& position,
    bool mobile)
{
  auto skeleton = dynamics::Skeleton::create(name);
  dynamics::GenericJoint<math::SE3Space>::Properties jointProperties(
      name + "_joint");
  dynamics::BodyNode::Properties bodyProperties(
      dynamics::BodyNode::AspectProperties(name + "_body"));
  bodyProperties.mInertia.setMass(1.0);

  auto pair = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>(
      nullptr, jointProperties, bodyProperties);
  auto* joint = pair.first;
  auto* body = pair.second;

  auto shape = std::make_shared<dynamics::BoxShape>(size);
  body->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(shape);

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = position;
  joint->setPositions(dynamics::FreeJoint::convertToPositions(transform));
  skeleton->setMobile(mobile);
  return skeleton;
}

dynamics::SkeletonPtr createSolverTestPlane(const std::string& name)
{
  auto skeleton = dynamics::Skeleton::create(name);
  auto body
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>().second;
  body->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(
      std::make_shared<dynamics::PlaneShape>(Eigen::Vector3d::UnitZ(), 0.0));
  skeleton->setMobile(false);
  return skeleton;
}

std::pair<dynamics::BodyNode*, dynamics::BodyNode*> createMixedReactiveSkeleton(
    const std::string& name, std::vector<dynamics::SkeletonPtr>& skeletons)
{
  auto skeleton = dynamics::Skeleton::create(name);
  auto rootPair = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>();
  rootPair.first->setActuatorType(dynamics::Joint::VELOCITY);
  auto childPair
      = rootPair.second->createChildJointAndBodyNodePair<dynamics::BallJoint>();
  skeletons.push_back(skeleton);
  return {rootPair.second, childPair.second};
}

collision::Contact createContact(
    collision::CollisionObject* object1, collision::CollisionObject* object2)
{
  collision::Contact contact;
  contact.collisionObject1 = object1;
  contact.collisionObject2 = object2;
  contact.point = Eigen::Vector3d::Zero();
  contact.normal = Eigen::Vector3d::UnitZ();
  return contact;
}

Eigen::Vector3d makeContactTangentDirection(
    const Eigen::Vector3d& normal, const Eigen::Vector3d& seed)
{
  Eigen::Vector3d n = normal;
  if (n.squaredNorm() < DART_CONTACT_CONSTRAINT_EPSILON_SQUARED)
    n = Eigen::Vector3d::UnitZ();
  else
    n.normalize();

  Eigen::Vector3d tangent = seed - n * seed.dot(n);
  if (tangent.squaredNorm() < DART_CONTACT_CONSTRAINT_EPSILON_SQUARED)
    tangent = Eigen::Vector3d::UnitX() - n * n.x();
  if (tangent.squaredNorm() < DART_CONTACT_CONSTRAINT_EPSILON_SQUARED)
    tangent = Eigen::Vector3d::UnitY() - n * n.y();
  if (tangent.squaredNorm() < DART_CONTACT_CONSTRAINT_EPSILON_SQUARED)
    tangent = Eigen::Vector3d::UnitZ() - n * n.z();

  tangent.normalize();
  return tangent;
}

template <typename ConstraintT>
std::shared_ptr<ConstraintT> createContactConstraint(
    collision::Contact& contact)
{
  auto constraint = std::make_shared<ConstraintT>(
      contact, 0.001, constraint::ContactSurfaceParams{});
  constraint::ConstraintBase& base = *constraint;
  base.update();
  return constraint;
}

std::shared_ptr<constraint::SoftContactConstraint> createSoftContactConstraint(
    collision::Contact& contact)
{
  auto constraint
      = std::make_shared<constraint::SoftContactConstraint>(contact, 0.001);
  constraint::ConstraintBase& base = *constraint;
  base.update();
  return constraint;
}

void addPaddingGroups(ExposedThreadedConstraintSolver& solver)
{
  solver.addFakeConstrainedGroups(128, 100);
}

bool solvesGroupsInParallel(ExposedThreadedConstraintSolver& solver)
{
  solver.setNumSimulationThreads(4);
  solver.solveGroupsForTest();
  EXPECT_GT(solver.getNumSolvedGroups(), 0);
  return solver.getMaxConcurrentSolves() > 1;
}

} // namespace

//==============================================================================
std::shared_ptr<World> createWorld()
{
  return simulation::World::create();
}

//==============================================================================
std::shared_ptr<World> createSingleFreeBodyContactWorld(bool legacyAssembly)
{
  auto world = createWorld();
  world->setTimeStep(0.001);

  simulation::DeactivationOptions deactivation;
  deactivation.mEnabled = false;
  world->setDeactivationOptions(deactivation);

  auto* solver = world->getConstraintSolver();
  solver->setCollisionDetector(collision::DARTCollisionDetector::create());
  solver->setNumSimulationThreads(1u);

  if (legacyAssembly) {
    auto defaultHandler = solver->getLastContactSurfaceHandler();
    auto customHandler = std::make_shared<CustomContactSurfaceHandler>();
    solver->addContactSurfaceHandler(customHandler);
    solver->removeContactSurfaceHandler(defaultHandler);
  }

  world->addSkeleton(createSolverTestPlane("ground"));
  world->addSkeleton(createSolverTestBox(
      "box", Eigen::Vector3d::Ones(), Eigen::Vector3d(0.0, 0.0, 0.49), true));

  return world;
}

//==============================================================================
void limitWorldToSingleContact(const std::shared_ptr<World>& world)
{
  auto& option = world->getConstraintSolver()->getCollisionOption();
  option.maxNumContacts = 1u;
  option.maxNumContactsPerPair = 1u;
}

//==============================================================================
std::shared_ptr<World> createManySingleFreeBodyContactWorld(
    std::size_t numBoxes,
    std::size_t numThreads,
    bool useNonDefaultSurfaceParams = false)
{
  auto world = createWorld();
  world->setTimeStep(0.001);

  simulation::DeactivationOptions deactivation;
  deactivation.mEnabled = false;
  world->setDeactivationOptions(deactivation);
  world->setNumSimulationThreads(numThreads);

  auto* solver = world->getConstraintSolver();
  solver->setCollisionDetector(collision::DARTCollisionDetector::create());
  solver->getCollisionOption().maxNumContacts = numBoxes * 4u;
  solver->getCollisionOption().maxNumContactsPerPair = 4u;

  world->addSkeleton(createSolverTestPlane("ground"));
  constexpr std::size_t kColumns = 16u;
  for (std::size_t i = 0u; i < numBoxes; ++i) {
    const auto row = i / kColumns;
    const auto column = i % kColumns;
    const Eigen::Vector3d position(
        static_cast<double>(column) * 2.0,
        static_cast<double>(row) * 2.0,
        0.49);
    auto box = createSolverTestBox(
        "box_" + std::to_string(i), Eigen::Vector3d::Ones(), position, true);
    if (useNonDefaultSurfaceParams) {
      auto* body = box->getBodyNode(0);
      auto* dynamics = body->getShapeNode(0)->getDynamicsAspect();
      dynamics->setPrimaryFrictionCoeff(0.75);
      dynamics->setSecondaryFrictionCoeff(0.5);
      dynamics->setPrimarySlipCompliance(0.005);
      dynamics->setSecondarySlipCompliance(0.01);
      dynamics->setFirstFrictionDirection(Eigen::Vector3d::UnitX());

      auto* joint = static_cast<dynamics::FreeJoint*>(body->getParentJoint());
      joint->setLinearVelocity(Eigen::Vector3d(0.2, 0.0, 0.0));
    }
    world->addSkeleton(box);
  }

  return world;
}

//==============================================================================
std::shared_ptr<World> createExactCoulombParallelContactStackWorld(
    std::size_t solverThreads,
    std::size_t numBoxes = 40u,
    bool assembleDenseContactRowSnapshot = false,
    bool useContactRowDelassusOperator = true)
{
  auto world = createWorld();
  world->setTimeStep(0.001);

  simulation::DeactivationOptions deactivation;
  deactivation.mEnabled = false;
  world->setDeactivationOptions(deactivation);
  world->setNumSimulationThreads(1u);

  constraint::ExactCoulombFbfConstraintSolverOptions options;
  options.fallbackToBoxedLcp = false;
  options.assembleDenseContactRowSnapshot = assembleDenseContactRowSnapshot;
  options.useContactRowDelassusOperator = useContactRowDelassusOperator;
  options.useMatrixFreeDelassusSeed = true;
  options.enableWarmStart = false;
  options.enableProjectedGradientRetry = false;
  options.enableDenseResidualPolish = false;
  options.maxOuterIterations = 0;
  options.tolerance = 1e12;

  auto solverOwned
      = std::make_unique<constraint::ExactCoulombFbfConstraintSolver>(options);
  auto* solver = solverOwned.get();
  world->setConstraintSolver(std::move(solverOwned));
  solver->setSplitImpulseEnabled(false);
  solver->setCollisionDetector(collision::DARTCollisionDetector::create());
  solver->getCollisionOption().maxNumContacts = numBoxes * 4u;
  solver->getCollisionOption().maxNumContactsPerPair = 4u;
  solver->setNumSimulationThreads(solverThreads);

  // Keep collision and all non-solver World work serial. Only the exact
  // contact-row W*x kernel receives `solverThreads` in this fixture.
  auto dartDetector
      = std::dynamic_pointer_cast<collision::DARTCollisionDetector>(
          solver->getCollisionDetector());
  EXPECT_NE(nullptr, dartDetector);
  if (dartDetector != nullptr)
    dartDetector->setNumCollisionThreads(1u);

  world->addSkeleton(createSolverTestPlane("ground"));
  for (std::size_t i = 0u; i < numBoxes; ++i) {
    const Eigen::Vector3d position(
        0.0, 0.0, 0.49 + 0.98 * static_cast<double>(i));
    world->addSkeleton(createSolverTestBox(
        "stack_box_" + std::to_string(i),
        Eigen::Vector3d::Ones(),
        position,
        true));
  }

  return world;
}

//==============================================================================
void setManySingleFreeBodyContactSurfaceParams(
    const std::shared_ptr<World>& world, std::size_t numBoxes)
{
  for (std::size_t i = 0u; i < numBoxes; ++i) {
    const auto name = "box_" + std::to_string(i);
    auto box = world->getSkeleton(name);
    ASSERT_NE(nullptr, box) << name;

    auto* body = box->getBodyNode(0);
    ASSERT_NE(nullptr, body) << name;
    auto* shapeNode = body->getShapeNode(0);
    ASSERT_NE(nullptr, shapeNode) << name;
    auto* dynamics = shapeNode->getDynamicsAspect();
    ASSERT_NE(nullptr, dynamics) << name;

    dynamics->setPrimaryFrictionCoeff(0.75);
    dynamics->setSecondaryFrictionCoeff(0.5);
    dynamics->setPrimarySlipCompliance(0.005);
    dynamics->setSecondarySlipCompliance(0.01);
    dynamics->setFirstFrictionDirection(Eigen::Vector3d::UnitX());

    auto* joint = static_cast<dynamics::FreeJoint*>(body->getParentJoint());
    ASSERT_NE(nullptr, joint) << name;
    joint->setLinearVelocity(Eigen::Vector3d(0.2, 0.0, 0.0));
  }
}

//==============================================================================
void expectManySingleFreeBodyContactWorldsMatch(
    const std::shared_ptr<World>& expectedWorld,
    const std::shared_ptr<World>& actualWorld,
    std::size_t numBoxes)
{
  const auto& expectedContacts
      = expectedWorld->getConstraintSolver()->getLastCollisionResult();
  const auto& actualContacts
      = actualWorld->getConstraintSolver()->getLastCollisionResult();
  EXPECT_GE(expectedContacts.getNumContacts(), numBoxes * 3u);
  EXPECT_EQ(expectedContacts.getNumContacts(), actualContacts.getNumContacts());

  for (std::size_t i = 0u; i < numBoxes; ++i) {
    const auto name = "box_" + std::to_string(i);
    const auto expectedBox = expectedWorld->getSkeleton(name);
    const auto actualBox = actualWorld->getSkeleton(name);
    ASSERT_NE(nullptr, expectedBox) << name;
    ASSERT_NE(nullptr, actualBox) << name;

    EXPECT_TRUE(
        expectedBox->getPositions().isApprox(actualBox->getPositions(), 1e-12))
        << name;
    EXPECT_TRUE(expectedBox->getVelocities().isApprox(
        actualBox->getVelocities(), 1e-12))
        << name;

    const auto* expectedBody = expectedBox->getBodyNode(0);
    const auto* actualBody = actualBox->getBodyNode(0);
    ASSERT_NE(nullptr, expectedBody) << name;
    ASSERT_NE(nullptr, actualBody) << name;
    EXPECT_TRUE(expectedBody->getWorldTransform().matrix().isApprox(
        actualBody->getWorldTransform().matrix(), 1e-12))
        << name;
    EXPECT_TRUE(expectedBody->getSpatialVelocity().isApprox(
        actualBody->getSpatialVelocity(), 1e-12))
        << name;
  }
}

//==============================================================================
TEST(ConstraintSolver, DirectSingleFreeBodyContactsMatchLegacyAssembly)
{
  auto directWorld = createSingleFreeBodyContactWorld(false);
  auto legacyWorld = createSingleFreeBodyContactWorld(true);

  for (std::size_t i = 0u; i < 300u; ++i) {
    directWorld->step();
    legacyWorld->step();
  }

  EXPECT_GT(
      directWorld->getConstraintSolver()
          ->getLastCollisionResult()
          .getNumContacts(),
      0u);
  EXPECT_GT(
      legacyWorld->getConstraintSolver()
          ->getLastCollisionResult()
          .getNumContacts(),
      0u);

  const auto directBox = directWorld->getSkeleton("box");
  const auto legacyBox = legacyWorld->getSkeleton("box");
  ASSERT_NE(nullptr, directBox);
  ASSERT_NE(nullptr, legacyBox);

  EXPECT_TRUE(
      directBox->getPositions().isApprox(legacyBox->getPositions(), 1e-12));
  EXPECT_TRUE(
      directBox->getVelocities().isApprox(legacyBox->getVelocities(), 1e-12));

  const auto* directBody = directBox->getBodyNode(0);
  const auto* legacyBody = legacyBox->getBodyNode(0);
  ASSERT_NE(nullptr, directBody);
  ASSERT_NE(nullptr, legacyBody);
  EXPECT_TRUE(directBody->getWorldTransform().matrix().isApprox(
      legacyBody->getWorldTransform().matrix(), 1e-12));
  EXPECT_TRUE(directBody->getSpatialVelocity().isApprox(
      legacyBody->getSpatialVelocity(), 1e-12));
}

//==============================================================================
TEST(ConstraintSolver, ExactCoulombFbfWorldSmokeComparesDefaultSingleContact)
{
  auto defaultWorld = createSingleFreeBodyContactWorld(false);
  auto exactWorld = createSingleFreeBodyContactWorld(false);
  limitWorldToSingleContact(defaultWorld);
  limitWorldToSingleContact(exactWorld);

  constraint::ExactCoulombFbfConstraintSolverOptions options;
  options.maxOuterIterations = 100;
  options.tolerance = 1e-8;
  options.innerMaxSweeps = 20;
  options.innerLocalIterations = 10;

  auto solver
      = std::make_unique<constraint::ExactCoulombFbfConstraintSolver>(options);
  auto* exactSolver = solver.get();
  exactWorld->setConstraintSolver(std::move(solver));
  limitWorldToSingleContact(exactWorld);

  defaultWorld->step();
  exactWorld->step();

  const auto defaultContactCount = defaultWorld->getConstraintSolver()
                                       ->getLastCollisionResult()
                                       .getNumContacts();
  const auto exactContactCount = exactWorld->getConstraintSolver()
                                     ->getLastCollisionResult()
                                     .getNumContacts();
  ASSERT_EQ(defaultContactCount, 1u);
  ASSERT_EQ(exactContactCount, defaultContactCount);

  EXPECT_EQ(
      exactSolver->getLastExactCoulombStatus(),
      constraint::ExactCoulombFbfConstraintSolverStatus::Success);
  EXPECT_EQ(
      exactSolver->getLastExactCoulombBuildStatus(),
      constraint::detail::ExactCoulombConstraintBuildStatus::Success);
  EXPECT_EQ(
      exactSolver->getLastExactCoulombFbfStatus(),
      math::detail::ExactCoulombFbfStatus::Success);
  EXPECT_EQ(exactSolver->getNumExactCoulombSolves(), 1u);
  EXPECT_EQ(exactSolver->getNumBoxedLcpFallbacks(), 0u);
  EXPECT_TRUE(std::isfinite(exactSolver->getLastExactCoulombResidual()));
  EXPECT_LE(exactSolver->getLastExactCoulombResidual(), options.tolerance);

  const auto defaultBox = defaultWorld->getSkeleton("box");
  const auto exactBox = exactWorld->getSkeleton("box");
  ASSERT_NE(nullptr, defaultBox);
  ASSERT_NE(nullptr, exactBox);

  EXPECT_TRUE(exactBox->getPositions().allFinite());
  EXPECT_TRUE(exactBox->getVelocities().allFinite());
  const double positionError
      = (defaultBox->getPositions() - exactBox->getPositions()).norm();
  const double velocityError
      = (defaultBox->getVelocities() - exactBox->getVelocities()).norm();
  EXPECT_LE(positionError, 1e-5)
      << "default=" << defaultBox->getPositions().transpose()
      << " exact=" << exactBox->getPositions().transpose();
  EXPECT_LE(velocityError, 1e-2)
      << "default=" << defaultBox->getVelocities().transpose()
      << " exact=" << exactBox->getVelocities().transpose();

  const auto* defaultBody = defaultBox->getBodyNode(0);
  const auto* exactBody = exactBox->getBodyNode(0);
  ASSERT_NE(nullptr, defaultBody);
  ASSERT_NE(nullptr, exactBody);
  EXPECT_TRUE(exactBody->getWorldTransform().matrix().allFinite());
  EXPECT_TRUE(exactBody->getSpatialVelocity().allFinite());
  EXPECT_TRUE(defaultBody->getWorldTransform().matrix().isApprox(
      exactBody->getWorldTransform().matrix(), 1e-5));
  const double spatialVelocityError
      = (defaultBody->getSpatialVelocity() - exactBody->getSpatialVelocity())
            .norm();
  EXPECT_LE(spatialVelocityError, 1e-2)
      << "default=" << defaultBody->getSpatialVelocity().transpose()
      << " exact=" << exactBody->getSpatialVelocity().transpose();
}

//==============================================================================
TEST(ConstraintSolver, ExactCoulombFbfWorldSmokeCanUseMatrixFreeDelassus)
{
  auto exactWorld = createSingleFreeBodyContactWorld(false);
  limitWorldToSingleContact(exactWorld);

  constraint::ExactCoulombFbfConstraintSolverOptions options;
  options.maxOuterIterations = 100;
  options.tolerance = 1e-8;
  options.innerMaxSweeps = 20;
  options.innerLocalIterations = 10;
  options.useMatrixFreeDelassusOperator = true;
  // This smoke verifies the legacy impulse-test product route, so keep the
  // scratch-backed contact-row operator out of the way.
  options.useContactRowDelassusOperator = false;

  auto solver
      = std::make_unique<constraint::ExactCoulombFbfConstraintSolver>(options);
  auto* exactSolver = solver.get();
  exactWorld->setConstraintSolver(std::move(solver));
  limitWorldToSingleContact(exactWorld);

  exactWorld->step();

  const auto exactContactCount = exactWorld->getConstraintSolver()
                                     ->getLastCollisionResult()
                                     .getNumContacts();
  ASSERT_EQ(exactContactCount, 1u);

  EXPECT_EQ(
      exactSolver->getLastExactCoulombStatus(),
      constraint::ExactCoulombFbfConstraintSolverStatus::Success);
  EXPECT_EQ(
      exactSolver->getLastExactCoulombBuildStatus(),
      constraint::detail::ExactCoulombConstraintBuildStatus::Success);
  EXPECT_EQ(
      exactSolver->getLastExactCoulombFbfStatus(),
      math::detail::ExactCoulombFbfStatus::Success);
  EXPECT_TRUE(exactSolver->getLastExactCoulombMatrixFreeDelassusOperatorUsed());
  EXPECT_EQ(exactSolver->getNumExactCoulombSolves(), 1u);
  EXPECT_EQ(exactSolver->getNumBoxedLcpFallbacks(), 0u);
  EXPECT_TRUE(std::isfinite(exactSolver->getLastExactCoulombResidual()));
  EXPECT_LE(exactSolver->getLastExactCoulombResidual(), options.tolerance);

  const auto exactBox = exactWorld->getSkeleton("box");
  ASSERT_NE(nullptr, exactBox);
  EXPECT_TRUE(exactBox->getPositions().allFinite());
  EXPECT_TRUE(exactBox->getVelocities().allFinite());
}

//==============================================================================
TEST(ConstraintSolver, ExactCoulombFbfWorldSmokeUsesContactRowOperator)
{
  auto rowOperatorWorld = createSingleFreeBodyContactWorld(false);
  auto noSnapshotWorld = createSingleFreeBodyContactWorld(false);
  auto impulseTestWorld = createSingleFreeBodyContactWorld(false);
  limitWorldToSingleContact(rowOperatorWorld);
  limitWorldToSingleContact(noSnapshotWorld);
  limitWorldToSingleContact(impulseTestWorld);

  constraint::ExactCoulombFbfConstraintSolverOptions options;
  options.maxOuterIterations = 100;
  options.tolerance = 1e-8;
  options.innerMaxSweeps = 20;
  options.innerLocalIterations = 10;
  ASSERT_TRUE(options.useContactRowDelassusOperator);

  auto rowSolverOwned
      = std::make_unique<constraint::ExactCoulombFbfConstraintSolver>(options);
  auto* rowSolver = rowSolverOwned.get();
  rowOperatorWorld->setConstraintSolver(std::move(rowSolverOwned));
  limitWorldToSingleContact(rowOperatorWorld);

  auto noSnapshotOptions = options;
  noSnapshotOptions.assembleDenseContactRowSnapshot = false;
  noSnapshotOptions.useMatrixFreeDelassusSeed = true;
  noSnapshotOptions.enableDenseResidualPolish = false;
  auto noSnapshotSolverOwned
      = std::make_unique<constraint::ExactCoulombFbfConstraintSolver>(
          noSnapshotOptions);
  auto* noSnapshotSolver = noSnapshotSolverOwned.get();
  noSnapshotWorld->setConstraintSolver(std::move(noSnapshotSolverOwned));
  limitWorldToSingleContact(noSnapshotWorld);

  auto impulseOptions = options;
  impulseOptions.useContactRowDelassusOperator = false;
  auto impulseSolverOwned
      = std::make_unique<constraint::ExactCoulombFbfConstraintSolver>(
          impulseOptions);
  auto* impulseSolver = impulseSolverOwned.get();
  impulseTestWorld->setConstraintSolver(std::move(impulseSolverOwned));
  limitWorldToSingleContact(impulseTestWorld);

  rowOperatorWorld->step();
  noSnapshotWorld->step();
  impulseTestWorld->step();

  EXPECT_EQ(
      rowSolver->getLastExactCoulombStatus(),
      constraint::ExactCoulombFbfConstraintSolverStatus::Success);
  EXPECT_TRUE(rowSolver->getLastExactCoulombContactRowOperatorUsed());
  EXPECT_TRUE(rowSolver->getLastExactCoulombDenseContactRowSnapshotAssembled());
  EXPECT_FALSE(rowSolver->getLastExactCoulombMatrixFreeDelassusOperatorUsed());
  EXPECT_EQ(rowSolver->getNumBoxedLcpFallbacks(), 0u);
  EXPECT_LE(rowSolver->getLastExactCoulombResidual(), options.tolerance);

  EXPECT_EQ(
      noSnapshotSolver->getLastExactCoulombStatus(),
      constraint::ExactCoulombFbfConstraintSolverStatus::Success);
  EXPECT_TRUE(noSnapshotSolver->getLastExactCoulombContactRowOperatorUsed());
  EXPECT_FALSE(
      noSnapshotSolver->getLastExactCoulombDenseContactRowSnapshotAssembled());
  EXPECT_TRUE(
      noSnapshotSolver->getLastExactCoulombMatrixFreeDelassusSeedUsed());
  EXPECT_EQ(noSnapshotSolver->getNumBoxedLcpFallbacks(), 0u);
  EXPECT_LE(
      noSnapshotSolver->getLastExactCoulombResidual(),
      noSnapshotOptions.tolerance);

  EXPECT_EQ(
      impulseSolver->getLastExactCoulombStatus(),
      constraint::ExactCoulombFbfConstraintSolverStatus::Success);
  EXPECT_FALSE(impulseSolver->getLastExactCoulombContactRowOperatorUsed());

  const auto rowBox = rowOperatorWorld->getSkeleton("box");
  const auto noSnapshotBox = noSnapshotWorld->getSkeleton("box");
  const auto impulseBox = impulseTestWorld->getSkeleton("box");
  ASSERT_NE(nullptr, rowBox);
  ASSERT_NE(nullptr, noSnapshotBox);
  ASSERT_NE(nullptr, impulseBox);
  EXPECT_TRUE(rowBox->getPositions().allFinite());
  EXPECT_TRUE(rowBox->getVelocities().allFinite());
  EXPECT_TRUE(noSnapshotBox->getPositions().allFinite());
  EXPECT_TRUE(noSnapshotBox->getVelocities().allFinite());
  // The row-assembled snapshot equals the impulse-test snapshot to solver
  // precision, so the one-step states must agree far below contact scales.
  EXPECT_LE((rowBox->getPositions() - impulseBox->getPositions()).norm(), 1e-9)
      << "row=" << rowBox->getPositions().transpose()
      << " impulse=" << impulseBox->getPositions().transpose();
  EXPECT_LE(
      (rowBox->getVelocities() - impulseBox->getVelocities()).norm(), 1e-7)
      << "row=" << rowBox->getVelocities().transpose()
      << " impulse=" << impulseBox->getVelocities().transpose();
  EXPECT_LE(
      (noSnapshotBox->getPositions() - impulseBox->getPositions()).norm(), 1e-9)
      << "no_snapshot=" << noSnapshotBox->getPositions().transpose()
      << " impulse=" << impulseBox->getPositions().transpose();
  EXPECT_LE(
      (noSnapshotBox->getVelocities() - impulseBox->getVelocities()).norm(),
      1e-7)
      << "no_snapshot=" << noSnapshotBox->getVelocities().transpose()
      << " impulse=" << impulseBox->getVelocities().transpose();
}

//==============================================================================
TEST(
    ConstraintSolver,
    ExactCoulombParallelContactRowMatchesSerialWorldStateBitExactly)
{
  auto defaultWorld = createWorld();
  ASSERT_NE(nullptr, defaultWorld->getConstraintSolver());
  EXPECT_EQ(
      typeid(constraint::BoxedLcpConstraintSolver),
      typeid(*defaultWorld->getConstraintSolver()));

  auto serialWorld = createExactCoulombParallelContactStackWorld(1u);
  auto parallelWorld = createExactCoulombParallelContactStackWorld(4u);
  auto* serialSolver
      = dynamic_cast<constraint::ExactCoulombFbfConstraintSolver*>(
          serialWorld->getConstraintSolver());
  auto* parallelSolver
      = dynamic_cast<constraint::ExactCoulombFbfConstraintSolver*>(
          parallelWorld->getConstraintSolver());
  ASSERT_NE(nullptr, serialSolver);
  ASSERT_NE(nullptr, parallelSolver);

  serialWorld->step();
  parallelWorld->step();

  const auto& serialContacts = serialSolver->getLastCollisionResult();
  const auto& parallelContacts = parallelSolver->getLastCollisionResult();
  ASSERT_GE(serialContacts.getNumContacts(), 128u);
  ASSERT_EQ(serialContacts.getNumContacts(), parallelContacts.getNumContacts());
  ASSERT_EQ(
      constraint::ExactCoulombFbfConstraintSolverStatus::Success,
      serialSolver->getLastExactCoulombStatus());
  EXPECT_EQ(
      serialSolver->getLastExactCoulombStatus(),
      parallelSolver->getLastExactCoulombStatus());
  EXPECT_EQ(
      serialSolver->getLastExactCoulombFbfStatus(),
      parallelSolver->getLastExactCoulombFbfStatus());
  EXPECT_DOUBLE_EQ(
      serialSolver->getLastExactCoulombResidual(),
      parallelSolver->getLastExactCoulombResidual());
  EXPECT_DOUBLE_EQ(
      serialSolver->getLastExactCoulombStepSize(),
      parallelSolver->getLastExactCoulombStepSize());
  EXPECT_EQ(
      serialSolver->getLastExactCoulombIterations(),
      parallelSolver->getLastExactCoulombIterations());

  EXPECT_GT(serialSolver->getLastExactCoulombContactRowDelassusProducts(), 0u);
  EXPECT_EQ(
      serialSolver->getLastExactCoulombContactRowDelassusProducts(),
      parallelSolver->getLastExactCoulombContactRowDelassusProducts());
  EXPECT_EQ(
      serialSolver->getLastExactCoulombContactRowDelassusProducts(),
      serialSolver->getNumExactCoulombContactRowDelassusProducts());
  EXPECT_EQ(
      parallelSolver->getLastExactCoulombContactRowDelassusProducts(),
      parallelSolver->getNumExactCoulombContactRowDelassusProducts());
  EXPECT_EQ(
      serialSolver->getLastExactCoulombParallelContactRowDelassusProducts(),
      0u);
  EXPECT_EQ(serialSolver->getLastExactCoulombMaxContactRowParticipants(), 1u);
  EXPECT_GT(
      parallelSolver->getLastExactCoulombParallelContactRowDelassusProducts(),
      0u);
  EXPECT_GT(parallelSolver->getLastExactCoulombMaxContactRowParticipants(), 1u);
  EXPECT_EQ(
      parallelSolver->getLastExactCoulombParallelContactRowDelassusProducts(),
      parallelSolver->getNumExactCoulombParallelContactRowDelassusProducts());
  EXPECT_EQ(
      parallelSolver->getLastExactCoulombMaxContactRowParticipants(),
      parallelSolver->getMaxExactCoulombContactRowParticipants());

  EXPECT_TRUE(
      serialSolver->getLastExactCoulombContactRowLogicalCpuIds().empty());
  EXPECT_TRUE(serialSolver->getExactCoulombContactRowLogicalCpuIds().empty());
  EXPECT_TRUE(serialSolver->getLastExactCoulombMaxPhaseContactRowLogicalCpuIds()
                  .empty());
  EXPECT_TRUE(
      serialSolver->getMaxExactCoulombPhaseContactRowLogicalCpuIds().empty());
  const auto lastLogicalCpuIds
      = parallelSolver->getLastExactCoulombContactRowLogicalCpuIds();
  const auto logicalCpuIds
      = parallelSolver->getExactCoulombContactRowLogicalCpuIds();
  const auto lastMaxPhaseLogicalCpuIds
      = parallelSolver->getLastExactCoulombMaxPhaseContactRowLogicalCpuIds();
  const auto maxPhaseLogicalCpuIds
      = parallelSolver->getMaxExactCoulombPhaseContactRowLogicalCpuIds();
#if defined(__linux__)
  ASSERT_FALSE(lastLogicalCpuIds.empty());
  EXPECT_TRUE(
      std::is_sorted(lastLogicalCpuIds.begin(), lastLogicalCpuIds.end()));
  EXPECT_EQ(
      std::adjacent_find(lastLogicalCpuIds.begin(), lastLogicalCpuIds.end()),
      lastLogicalCpuIds.end());
  for (const int logicalCpu : lastLogicalCpuIds)
    EXPECT_GE(logicalCpu, 0);

  // One CPU observation is made per participating thread in each of the two
  // row-product phases. This bound remains valid when a thread migrates across
  // phases; the IDs intentionally do not claim simultaneous execution.
  EXPECT_LE(
      lastLogicalCpuIds.size(),
      2u * parallelSolver->getLastExactCoulombContactRowDelassusProducts()
          * parallelSolver->getLastExactCoulombMaxContactRowParticipants());
  EXPECT_TRUE(std::includes(
      logicalCpuIds.begin(),
      logicalCpuIds.end(),
      lastLogicalCpuIds.begin(),
      lastLogicalCpuIds.end()));
  EXPECT_TRUE(std::is_sorted(logicalCpuIds.begin(), logicalCpuIds.end()));
  EXPECT_EQ(
      std::adjacent_find(logicalCpuIds.begin(), logicalCpuIds.end()),
      logicalCpuIds.end());
  for (const int logicalCpu : logicalCpuIds)
    EXPECT_GE(logicalCpu, 0);

  ASSERT_FALSE(lastMaxPhaseLogicalCpuIds.empty());
  EXPECT_TRUE(std::is_sorted(
      lastMaxPhaseLogicalCpuIds.begin(), lastMaxPhaseLogicalCpuIds.end()));
  EXPECT_EQ(
      std::adjacent_find(
          lastMaxPhaseLogicalCpuIds.begin(), lastMaxPhaseLogicalCpuIds.end()),
      lastMaxPhaseLogicalCpuIds.end());
  for (const int logicalCpu : lastMaxPhaseLogicalCpuIds)
    EXPECT_GE(logicalCpu, 0);
  EXPECT_LE(
      lastMaxPhaseLogicalCpuIds.size(),
      parallelSolver->getLastExactCoulombMaxContactRowParticipants());
  EXPECT_TRUE(std::includes(
      lastLogicalCpuIds.begin(),
      lastLogicalCpuIds.end(),
      lastMaxPhaseLogicalCpuIds.begin(),
      lastMaxPhaseLogicalCpuIds.end()));
  EXPECT_EQ(lastMaxPhaseLogicalCpuIds, maxPhaseLogicalCpuIds);
#else
  EXPECT_TRUE(lastLogicalCpuIds.empty());
  EXPECT_TRUE(logicalCpuIds.empty());
  EXPECT_TRUE(lastMaxPhaseLogicalCpuIds.empty());
  EXPECT_TRUE(maxPhaseLogicalCpuIds.empty());
#endif

  ASSERT_EQ(serialWorld->getNumSkeletons(), parallelWorld->getNumSkeletons());
  for (std::size_t i = 0u; i < serialWorld->getNumSkeletons(); ++i) {
    const auto serialSkeleton = serialWorld->getSkeleton(i);
    const auto parallelSkeleton = parallelWorld->getSkeleton(i);
    ASSERT_NE(nullptr, serialSkeleton);
    ASSERT_NE(nullptr, parallelSkeleton);
    EXPECT_TRUE((serialSkeleton->getPositions().array()
                 == parallelSkeleton->getPositions().array())
                    .all())
        << serialSkeleton->getName();
    EXPECT_TRUE((serialSkeleton->getVelocities().array()
                 == parallelSkeleton->getVelocities().array())
                    .all())
        << serialSkeleton->getName();

    ASSERT_EQ(
        serialSkeleton->getNumBodyNodes(), parallelSkeleton->getNumBodyNodes());
    for (std::size_t body = 0u; body < serialSkeleton->getNumBodyNodes();
         ++body) {
      const auto* serialBody = serialSkeleton->getBodyNode(body);
      const auto* parallelBody = parallelSkeleton->getBodyNode(body);
      ASSERT_NE(nullptr, serialBody);
      ASSERT_NE(nullptr, parallelBody);
      EXPECT_TRUE((serialBody->getWorldTransform().matrix().array()
                   == parallelBody->getWorldTransform().matrix().array())
                      .all())
          << serialSkeleton->getName();
      EXPECT_TRUE((serialBody->getSpatialVelocity().array()
                   == parallelBody->getSpatialVelocity().array())
                      .all())
          << serialSkeleton->getName();
    }
  }

  // The last-attempt set resets when the exact row products execute serially,
  // while the lifetime-to-date union remains available to a trace spanning
  // multiple World steps.
  parallelSolver->setNumSimulationThreads(1u);
  parallelWorld->step();
  EXPECT_TRUE(
      parallelSolver->getLastExactCoulombContactRowLogicalCpuIds().empty());
  EXPECT_TRUE(
      parallelSolver->getLastExactCoulombMaxPhaseContactRowLogicalCpuIds()
          .empty());
  EXPECT_EQ(
      logicalCpuIds, parallelSolver->getExactCoulombContactRowLogicalCpuIds());
  EXPECT_EQ(
      maxPhaseLogicalCpuIds,
      parallelSolver->getMaxExactCoulombPhaseContactRowLogicalCpuIds());
}

//==============================================================================
TEST(
    ConstraintSolver,
    ExactCoulombParallelContactRowEligibilityKeepsOtherRoutesSerial)
{
  auto smallWorld = createExactCoulombParallelContactStackWorld(4u, 20u);
  auto denseWorld
      = createExactCoulombParallelContactStackWorld(4u, 40u, true, true);
  auto impulseWorld
      = createExactCoulombParallelContactStackWorld(4u, 40u, false, false);

  smallWorld->step();
  denseWorld->step();
  impulseWorld->step();

  auto* smallSolver
      = dynamic_cast<constraint::ExactCoulombFbfConstraintSolver*>(
          smallWorld->getConstraintSolver());
  auto* denseSolver
      = dynamic_cast<constraint::ExactCoulombFbfConstraintSolver*>(
          denseWorld->getConstraintSolver());
  auto* impulseSolver
      = dynamic_cast<constraint::ExactCoulombFbfConstraintSolver*>(
          impulseWorld->getConstraintSolver());
  ASSERT_NE(nullptr, smallSolver);
  ASSERT_NE(nullptr, denseSolver);
  ASSERT_NE(nullptr, impulseSolver);

  ASSERT_LT(smallSolver->getLastCollisionResult().getNumContacts(), 128u);
  ASSERT_GE(denseSolver->getLastCollisionResult().getNumContacts(), 128u);
  ASSERT_GE(impulseSolver->getLastCollisionResult().getNumContacts(), 128u);
  EXPECT_TRUE(smallSolver->getLastExactCoulombContactRowOperatorUsed());
  EXPECT_TRUE(denseSolver->getLastExactCoulombContactRowOperatorUsed());
  EXPECT_FALSE(impulseSolver->getLastExactCoulombContactRowOperatorUsed());
  EXPECT_FALSE(
      smallSolver->getLastExactCoulombDenseContactRowSnapshotAssembled());
  EXPECT_TRUE(
      denseSolver->getLastExactCoulombDenseContactRowSnapshotAssembled());

  for (const auto* solver : {smallSolver, denseSolver}) {
    EXPECT_GT(solver->getLastExactCoulombContactRowDelassusProducts(), 0u);
    EXPECT_EQ(
        solver->getLastExactCoulombParallelContactRowDelassusProducts(), 0u);
    EXPECT_EQ(solver->getLastExactCoulombMaxContactRowParticipants(), 1u);
    EXPECT_TRUE(solver->getLastExactCoulombContactRowLogicalCpuIds().empty());
    EXPECT_TRUE(solver->getExactCoulombContactRowLogicalCpuIds().empty());
    EXPECT_TRUE(
        solver->getLastExactCoulombMaxPhaseContactRowLogicalCpuIds().empty());
    EXPECT_TRUE(
        solver->getMaxExactCoulombPhaseContactRowLogicalCpuIds().empty());
  }
  EXPECT_EQ(impulseSolver->getLastExactCoulombContactRowDelassusProducts(), 0u);
  EXPECT_EQ(
      impulseSolver->getLastExactCoulombParallelContactRowDelassusProducts(),
      0u);
  EXPECT_EQ(impulseSolver->getLastExactCoulombMaxContactRowParticipants(), 0u);
  EXPECT_TRUE(
      impulseSolver->getLastExactCoulombContactRowLogicalCpuIds().empty());
  EXPECT_TRUE(impulseSolver->getExactCoulombContactRowLogicalCpuIds().empty());
  EXPECT_TRUE(
      impulseSolver->getLastExactCoulombMaxPhaseContactRowLogicalCpuIds()
          .empty());
  EXPECT_TRUE(
      impulseSolver->getMaxExactCoulombPhaseContactRowLogicalCpuIds().empty());
}

//==============================================================================
TEST(ConstraintSolver, SplitImpulsePreservesVelocityPhaseContactResponse)
{
  // Regression guard for the split-impulse position pass: its LCP assembly
  // runs the same unit-impulse tests as the velocity pass, and those clear
  // each touched skeleton's accumulated constraint impulses. Without the
  // save/restore in ConstraintSolver::solvePositionConstrainedGroups the
  // velocity-phase contact response is silently discarded before World::step
  // integrates it, so a resting box free-falls through the ground while its
  // contact persists.
  auto world = createSingleFreeBodyContactWorld(false);
  world->getConstraintSolver()->setSplitImpulseEnabled(true);

  for (std::size_t step = 0u; step < 120u; ++step) {
    world->step();
  }

  const auto box = world->getSkeleton("box");
  ASSERT_NE(nullptr, box);
  const auto* body = box->getBodyNode(0);
  ASSERT_NE(nullptr, body);
  const double height = body->getWorldTransform().translation().z();
  // The box starts with its bottom face 0.01 below the plane (z = 0.49) and
  // must remain resting near z = 0.5; free fall would put it far below.
  EXPECT_GT(height, 0.45) << "box fell through the ground: z=" << height;
  EXPECT_LT(height, 0.55);
  EXPECT_LT(box->getVelocities().norm(), 0.05);
}

//==============================================================================
TEST(ConstraintSolver, ExactCoulombFbfWorldSmokeManifoldWarmStartAcrossSteps)
{
  auto world = createSingleFreeBodyContactWorld(false);
  limitWorldToSingleContact(world);

  constraint::ExactCoulombFbfConstraintSolverOptions options;
  options.maxOuterIterations = 200;
  options.tolerance = 1e-8;
  options.innerMaxSweeps = 20;
  options.innerLocalIterations = 10;
  ASSERT_TRUE(options.enableWarmStart);

  auto solverOwned
      = std::make_unique<constraint::ExactCoulombFbfConstraintSolver>(options);
  auto* solver = solverOwned.get();
  world->setConstraintSolver(std::move(solverOwned));
  limitWorldToSingleContact(world);

  world->step();
  EXPECT_EQ(
      solver->getLastExactCoulombStatus(),
      constraint::ExactCoulombFbfConstraintSolverStatus::Success);
  // Contact constraints are recreated per step, so the first step is cold.
  EXPECT_FALSE(solver->getLastExactCoulombWarmStartUsed());

  world->step();
  EXPECT_EQ(
      solver->getLastExactCoulombStatus(),
      constraint::ExactCoulombFbfConstraintSolverStatus::Success);
  // The persistent box/ground contact must warm start through manifold
  // matching even though the constraint pointers changed.
  EXPECT_TRUE(solver->getLastExactCoulombWarmStartUsed());
  EXPECT_GE(solver->getLastExactCoulombWarmStartMatchedContacts(), 1u);
  EXPECT_EQ(solver->getNumBoxedLcpFallbacks(), 0u);
  EXPECT_LE(solver->getLastExactCoulombResidual(), options.tolerance);

  const auto box = world->getSkeleton("box");
  ASSERT_NE(nullptr, box);
  EXPECT_TRUE(box->getPositions().allFinite());
  EXPECT_TRUE(box->getVelocities().allFinite());
}

//==============================================================================
TEST(
    ConstraintSolver,
    ExactCoulombFbfPooledContactPointerStillRequiresManifoldIdentity)
{
  std::vector<dynamics::SkeletonPtr> skeletons;
  auto* fixedBody = createFreeBody("fixed", false, skeletons);
  auto* dynamicBody = createFreeBody("dynamic", true, skeletons);

  auto shape = std::make_shared<dynamics::BoxShape>(Eigen::Vector3d::Ones());
  auto* fixedShape = fixedBody->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(shape);
  auto* dynamicShape = dynamicBody->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(shape);

  FakeCollisionDetector detector;
  FakeCollisionObject fixedObject(&detector, fixedShape);
  FakeCollisionObject dynamicObject(&detector, dynamicShape);
  auto contact = createContact(&dynamicObject, &fixedObject);
  contact.penetrationDepth = 0.0;
  auto constraint
      = createContactConstraint<constraint::ContactConstraint>(contact);
  constraint::ConstrainedGroup group;
  group.addConstraint(constraint);

  constraint::ExactCoulombFbfConstraintSolverOptions options;
  options.maxOuterIterations = 0;
  options.tolerance = 0.0;
  options.seedNormalImpulseFromDiagonal = false;
  options.enableProjectedGradientRetry = false;
  options.enableDenseResidualPolish = false;
  options.fallbackToBoxedLcp = false;
  ExposedExactCoulombParallelEligibilitySolver solver;
  solver.setExactCoulombOptions(options);
  solver.setTimeStep(0.001);

  solver.solveConstrainedGroup(group);
  ASSERT_EQ(
      solver.getLastExactCoulombStatus(),
      constraint::ExactCoulombFbfConstraintSolverStatus::Success);
  EXPECT_FALSE(solver.getLastExactCoulombWarmStartUsed());
  EXPECT_FALSE(solver.getLastExactCoulombPersistentStepSizeUsed());

  // An unchanged physical frame is a valid manifold match even when the
  // ContactConstraint pointer is reused.
  solver.solveConstrainedGroup(group);
  ASSERT_EQ(
      solver.getLastExactCoulombStatus(),
      constraint::ExactCoulombFbfConstraintSolverStatus::Success);
  EXPECT_TRUE(solver.getLastExactCoulombWarmStartUsed());
  EXPECT_TRUE(solver.getLastExactCoulombPersistentStepSizeUsed());
  EXPECT_EQ(solver.getLastExactCoulombWarmStartMatchedContacts(), 1u);

  // ConstraintSolver pools ContactConstraint objects and resets their contact
  // data. Mutating the referenced frame models that reuse while preserving the
  // exact same pointer sequence. A contact outside the manifold tolerance must
  // not inherit either the prior reaction or its persisted gamma.
  contact.point += Eigen::Vector3d(1.0, 0.0, 0.0);
  solver.solveConstrainedGroup(group);
  EXPECT_EQ(
      solver.getLastExactCoulombStatus(),
      constraint::ExactCoulombFbfConstraintSolverStatus::Success);
  EXPECT_FALSE(solver.getLastExactCoulombWarmStartUsed());
  EXPECT_FALSE(solver.getLastExactCoulombPersistentStepSizeUsed());
  EXPECT_EQ(solver.getLastExactCoulombWarmStartMatchedContacts(), 0u);
}

//==============================================================================
TEST(
    ConstraintSolver,
    ExactCoulombFbfPooledContactReorderMatchesReactionByManifold)
{
  std::vector<dynamics::SkeletonPtr> skeletons;
  auto* fixedBody = createFreeBody("fixed", false, skeletons);
  auto* dynamicBody = createFreeBody("dynamic", true, skeletons);

  auto shape = std::make_shared<dynamics::BoxShape>(Eigen::Vector3d::Ones());
  auto* fixedShape = fixedBody->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(shape);
  auto* dynamicShape = dynamicBody->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(shape);

  FakeCollisionDetector detector;
  FakeCollisionObject fixedObject(&detector, fixedShape);
  FakeCollisionObject dynamicObject(&detector, dynamicShape);
  auto contactA = createContact(&dynamicObject, &fixedObject);
  auto contactB = createContact(&dynamicObject, &fixedObject);
  contactA.point = Eigen::Vector3d(-0.2, 0.0, 0.0);
  contactB.point = Eigen::Vector3d(0.2, 0.0, 0.0);
  contactA.penetrationDepth = 0.0;
  contactB.penetrationDepth = 0.01;
  auto constraintA
      = createContactConstraint<constraint::ContactConstraint>(contactA);
  auto constraintB
      = createContactConstraint<constraint::ContactConstraint>(contactB);
  constraint::ConstrainedGroup group;
  group.addConstraint(constraintA);
  group.addConstraint(constraintB);

  constraint::ExactCoulombFbfConstraintSolverOptions options;
  options.maxOuterIterations = 0;
  options.tolerance = 0.0;
  options.acceptOuterMaxIterations = true;
  options.enableProjectedGradientRetry = false;
  options.enableDenseResidualPolish = false;
  options.fallbackToBoxedLcp = false;
  ExposedExactCoulombParallelEligibilitySolver solver;
  solver.setExactCoulombOptions(options);
  solver.setTimeStep(0.001);

  solver.solveConstrainedGroup(group);
  ASSERT_EQ(
      solver.getLastExactCoulombStatus(),
      constraint::ExactCoulombFbfConstraintSolverStatus::Success);
  const Eigen::Vector3d impulseAtA = 0.001 * contactA.force;
  const Eigen::Vector3d impulseAtB = 0.001 * contactB.force;
  ASSERT_GT((impulseAtA - impulseAtB).norm(), 1e-8);

  // Keep the pointer order fixed while exchanging the world-space frames they
  // reference. A pointer fast path would leave the two reactions in their old
  // slots; manifold matching must follow the points instead.
  std::swap(contactA.point, contactB.point);
  solver.solveConstrainedGroup(group);
  ASSERT_EQ(
      solver.getLastExactCoulombStatus(),
      constraint::ExactCoulombFbfConstraintSolverStatus::MaxIterationsAccepted);
  EXPECT_TRUE(solver.getLastExactCoulombWarmStartUsed());
  EXPECT_TRUE(solver.getLastExactCoulombPersistentStepSizeUsed());
  EXPECT_EQ(solver.getLastExactCoulombWarmStartMatchedContacts(), 2u);
  EXPECT_TRUE((0.001 * contactA.force).isApprox(impulseAtB, 1e-10));
  EXPECT_TRUE((0.001 * contactB.force).isApprox(impulseAtA, 1e-10));
}

//==============================================================================
TEST(ConstraintSolver, ExactCoulombFbfManifoldWarmStartMatchesMultipleContacts)
{
  // A two-box stack forms one constrained group with two persistent
  // contacts (ground/box and box/box), so the second step must warm start
  // both contacts through body-pair / nearest-point matching.
  auto world = createSingleFreeBodyContactWorld(false);
  world->addSkeleton(createSolverTestBox(
      "box_two",
      Eigen::Vector3d::Ones(),
      Eigen::Vector3d(0.0, 0.0, 1.47),
      true));

  constraint::ExactCoulombFbfConstraintSolverOptions options;
  options.maxOuterIterations = 2000;
  options.tolerance = 1e-8;
  options.innerMaxSweeps = 40;
  options.innerLocalIterations = 16;

  auto solverOwned
      = std::make_unique<constraint::ExactCoulombFbfConstraintSolver>(options);
  auto* solver = solverOwned.get();
  world->setConstraintSolver(std::move(solverOwned));

  world->step();
  ASSERT_EQ(
      solver->getLastExactCoulombStatus(),
      constraint::ExactCoulombFbfConstraintSolverStatus::Success);
  const auto firstStepContacts
      = world->getConstraintSolver()->getLastCollisionResult().getNumContacts();
  ASSERT_GE(firstStepContacts, 2u);

  world->step();
  EXPECT_EQ(
      solver->getLastExactCoulombStatus(),
      constraint::ExactCoulombFbfConstraintSolverStatus::Success);
  EXPECT_TRUE(solver->getLastExactCoulombWarmStartUsed());
  EXPECT_GE(solver->getLastExactCoulombWarmStartMatchedContacts(), 2u);
  EXPECT_EQ(solver->getNumBoxedLcpFallbacks(), 0u);
  EXPECT_LE(solver->getLastExactCoulombResidual(), options.tolerance);
}

//==============================================================================
TEST(ConstraintSolver, ExactCoulombFailureInvalidatesOnlyItsManifoldGammaToken)
{
  std::vector<dynamics::SkeletonPtr> skeletons;
  auto* fixedBodyA = createFreeBody("fixed_a", false, skeletons);
  auto* dynamicBodyA = createFreeBody("dynamic_a", true, skeletons);
  auto* fixedBodyB = createFreeBody("fixed_b", false, skeletons);
  auto* dynamicBodyB = createFreeBody("dynamic_b", true, skeletons);

  auto shape = std::make_shared<dynamics::BoxShape>(Eigen::Vector3d::Ones());
  auto* fixedShapeA = fixedBodyA->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(shape);
  auto* dynamicShapeA = dynamicBodyA->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(shape);
  auto* fixedShapeB = fixedBodyB->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(shape);
  auto* dynamicShapeB = dynamicBodyB->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(shape);

  FakeCollisionDetector detector;
  FakeCollisionObject fixedObjectA(&detector, fixedShapeA);
  FakeCollisionObject dynamicObjectA(&detector, dynamicShapeA);
  FakeCollisionObject fixedObjectB(&detector, fixedShapeB);
  FakeCollisionObject dynamicObjectB(&detector, dynamicShapeB);

  const auto makeGroup = [](collision::Contact& contact) {
    constraint::ConstrainedGroup group;
    group.addConstraint(
        createContactConstraint<constraint::ContactConstraint>(contact));
    return group;
  };

  constraint::ExactCoulombFbfConstraintSolverOptions options;
  options.maxOuterIterations = 0;
  options.tolerance = 0.0;
  options.seedNormalImpulseFromDiagonal = false;
  options.enableProjectedGradientRetry = false;
  options.enableDenseResidualPolish = false;
  options.fallbackToBoxedLcp = false;
  ExposedExactCoulombParallelEligibilitySolver solver;
  solver.setExactCoulombOptions(options);
  solver.setTimeStep(0.001);

  auto firstContactA = createContact(&dynamicObjectA, &fixedObjectA);
  firstContactA.penetrationDepth = 0.0;
  auto firstGroupA = makeGroup(firstContactA);
  solver.solveConstrainedGroup(firstGroupA);
  ASSERT_EQ(
      solver.getLastExactCoulombStatus(),
      constraint::ExactCoulombFbfConstraintSolverStatus::Success);

  auto firstContactB = createContact(&dynamicObjectB, &fixedObjectB);
  firstContactB.penetrationDepth = 0.0;
  auto firstGroupB = makeGroup(firstContactB);
  solver.solveConstrainedGroup(firstGroupB);
  ASSERT_EQ(
      solver.getLastExactCoulombStatus(),
      constraint::ExactCoulombFbfConstraintSolverStatus::Success);

  // Recreate island A's constraint at the same manifold point, but make the
  // contact require a nonzero reaction. With a zero outer budget this fails
  // after matching A's prior reaction and gamma records.
  auto failingContactA = createContact(&dynamicObjectA, &fixedObjectA);
  failingContactA.penetrationDepth = 0.01;
  auto failingGroupA = makeGroup(failingContactA);
  solver.solveConstrainedGroup(failingGroupA);
  ASSERT_EQ(
      solver.getLastExactCoulombStatus(),
      constraint::ExactCoulombFbfConstraintSolverStatus::FbfFailed);
  EXPECT_TRUE(solver.getLastExactCoulombWarmStartUsed());
  EXPECT_TRUE(solver.getLastExactCoulombPersistentStepSizeUsed());

  // Island B's independent persisted gamma survives A's failure.
  auto secondContactB = createContact(&dynamicObjectB, &fixedObjectB);
  secondContactB.penetrationDepth = 0.0;
  auto secondGroupB = makeGroup(secondContactB);
  solver.solveConstrainedGroup(secondGroupB);
  ASSERT_EQ(
      solver.getLastExactCoulombStatus(),
      constraint::ExactCoulombFbfConstraintSolverStatus::Success);
  EXPECT_TRUE(solver.getLastExactCoulombWarmStartUsed());
  EXPECT_TRUE(solver.getLastExactCoulombPersistentStepSizeUsed());

  // A's world-space reaction record is retained as a warm start, but its gamma
  // token is gone after the failed solve.
  auto recoveryContactA = createContact(&dynamicObjectA, &fixedObjectA);
  recoveryContactA.penetrationDepth = 0.0;
  auto recoveryGroupA = makeGroup(recoveryContactA);
  solver.solveConstrainedGroup(recoveryGroupA);
  EXPECT_EQ(
      solver.getLastExactCoulombStatus(),
      constraint::ExactCoulombFbfConstraintSolverStatus::Success);
  EXPECT_TRUE(solver.getLastExactCoulombWarmStartUsed());
  EXPECT_FALSE(solver.getLastExactCoulombPersistentStepSizeUsed());
}

//==============================================================================
TEST(
    ConstraintSolver,
    ExactCoulombFbfManifoldWarmStartSurvivesContactCountChange)
{
  // Shrinking the contact cap between steps leaves stale cached records; the
  // warm start must degrade to a partial (or skipped) seed and the solve must
  // still succeed - never fail or corrupt results.
  auto world = createSingleFreeBodyContactWorld(false);
  world->addSkeleton(createSolverTestBox(
      "box_two",
      Eigen::Vector3d::Ones(),
      Eigen::Vector3d(2.5, 0.0, 0.49),
      true));

  constraint::ExactCoulombFbfConstraintSolverOptions options;
  options.maxOuterIterations = 2000;
  options.tolerance = 1e-8;
  options.innerMaxSweeps = 40;
  options.innerLocalIterations = 16;

  auto solverOwned
      = std::make_unique<constraint::ExactCoulombFbfConstraintSolver>(options);
  auto* solver = solverOwned.get();
  world->setConstraintSolver(std::move(solverOwned));

  world->step();
  ASSERT_EQ(
      solver->getLastExactCoulombStatus(),
      constraint::ExactCoulombFbfConstraintSolverStatus::Success);
  ASSERT_GE(
      world->getConstraintSolver()->getLastCollisionResult().getNumContacts(),
      2u);

  limitWorldToSingleContact(world);
  world->step();
  ASSERT_EQ(
      world->getConstraintSolver()->getLastCollisionResult().getNumContacts(),
      1u);
  EXPECT_EQ(
      solver->getLastExactCoulombStatus(),
      constraint::ExactCoulombFbfConstraintSolverStatus::Success);
  EXPECT_EQ(solver->getNumBoxedLcpFallbacks(), 0u);
  EXPECT_LE(solver->getLastExactCoulombResidual(), options.tolerance);
  if (solver->getLastExactCoulombWarmStartUsed()) {
    EXPECT_GE(solver->getLastExactCoulombWarmStartMatchedContacts(), 1u);
    EXPECT_LE(solver->getLastExactCoulombWarmStartMatchedContacts(), 1u);
  }

  const auto box = world->getSkeleton("box");
  ASSERT_NE(nullptr, box);
  EXPECT_TRUE(box->getPositions().allFinite());
  EXPECT_TRUE(box->getVelocities().allFinite());
}

//==============================================================================
TEST(ConstraintSolver, ThreadedDefaultContactRebuildMatchesSerial)
{
  constexpr std::size_t kNumBoxes = 192u;
  auto serialWorld = createManySingleFreeBodyContactWorld(kNumBoxes, 1u);
  auto threadedWorld = createManySingleFreeBodyContactWorld(kNumBoxes, 4u);

  for (std::size_t i = 0u; i < 20u; ++i) {
    serialWorld->step();
    threadedWorld->step();
  }

  const auto& serialContacts
      = serialWorld->getConstraintSolver()->getLastCollisionResult();
  const auto& threadedContacts
      = threadedWorld->getConstraintSolver()->getLastCollisionResult();
  EXPECT_GE(serialContacts.getNumContacts(), kNumBoxes * 3u);
  EXPECT_EQ(serialContacts.getNumContacts(), threadedContacts.getNumContacts());

  for (std::size_t i = 0u; i < kNumBoxes; ++i) {
    const auto name = "box_" + std::to_string(i);
    const auto serialBox = serialWorld->getSkeleton(name);
    const auto threadedBox = threadedWorld->getSkeleton(name);
    ASSERT_NE(nullptr, serialBox);
    ASSERT_NE(nullptr, threadedBox);

    EXPECT_TRUE(
        serialBox->getPositions().isApprox(threadedBox->getPositions(), 1e-12))
        << name;
    EXPECT_TRUE(serialBox->getVelocities().isApprox(
        threadedBox->getVelocities(), 1e-12))
        << name;

    const auto* serialBody = serialBox->getBodyNode(0);
    const auto* threadedBody = threadedBox->getBodyNode(0);
    ASSERT_NE(nullptr, serialBody);
    ASSERT_NE(nullptr, threadedBody);
    EXPECT_TRUE(serialBody->getWorldTransform().matrix().isApprox(
        threadedBody->getWorldTransform().matrix(), 1e-12))
        << name;
    EXPECT_TRUE(serialBody->getSpatialVelocity().isApprox(
        threadedBody->getSpatialVelocity(), 1e-12))
        << name;
  }
}

//==============================================================================
TEST(ConstraintSolver, ThreadedDefaultContactRebuildMatchesSerialSurfaceParams)
{
  constexpr std::size_t kNumBoxes = 192u;
  auto serialWorld = createManySingleFreeBodyContactWorld(kNumBoxes, 1u, true);
  auto threadedWorld
      = createManySingleFreeBodyContactWorld(kNumBoxes, 4u, true);

  for (std::size_t i = 0u; i < 20u; ++i) {
    serialWorld->step();
    threadedWorld->step();
  }

  const auto& serialContacts
      = serialWorld->getConstraintSolver()->getLastCollisionResult();
  const auto& threadedContacts
      = threadedWorld->getConstraintSolver()->getLastCollisionResult();
  EXPECT_GE(serialContacts.getNumContacts(), kNumBoxes * 3u);
  EXPECT_EQ(serialContacts.getNumContacts(), threadedContacts.getNumContacts());

  for (std::size_t i = 0u; i < kNumBoxes; ++i) {
    const auto name = "box_" + std::to_string(i);
    const auto serialBox = serialWorld->getSkeleton(name);
    const auto threadedBox = threadedWorld->getSkeleton(name);
    ASSERT_NE(nullptr, serialBox);
    ASSERT_NE(nullptr, threadedBox);

    EXPECT_TRUE(
        serialBox->getPositions().isApprox(threadedBox->getPositions(), 1e-12))
        << name;
    EXPECT_TRUE(serialBox->getVelocities().isApprox(
        threadedBox->getVelocities(), 1e-12))
        << name;
  }
}

//==============================================================================
TEST(ConstraintSolver, ThreadedSurfacePrepassMatchesSerialForLargeBatches)
{
  constexpr std::size_t kNumBoxes = 1040u;

  const auto runCase = [](bool useNonDefaultSurfaceParams) {
    auto serialWorld = createManySingleFreeBodyContactWorld(
        kNumBoxes, 1u, useNonDefaultSurfaceParams);
    auto threadedWorld = createManySingleFreeBodyContactWorld(
        kNumBoxes, 4u, useNonDefaultSurfaceParams);

    for (std::size_t i = 0u; i < 6u; ++i) {
      serialWorld->step();
      threadedWorld->step();
    }

    ASSERT_NO_FATAL_FAILURE(expectManySingleFreeBodyContactWorldsMatch(
        serialWorld, threadedWorld, kNumBoxes));
  };

  runCase(false);
  runCase(true);
}

//==============================================================================
TEST(ConstraintSolver, DefaultSurfaceCacheInvalidatesAfterDynamicsUpdate)
{
  constexpr std::size_t kNumBoxes = 192u;
  auto cachedWorld = createManySingleFreeBodyContactWorld(kNumBoxes, 4u);
  auto referenceWorld = createManySingleFreeBodyContactWorld(kNumBoxes, 4u);

  for (std::size_t i = 0u; i < 5u; ++i) {
    cachedWorld->step();
    referenceWorld->step();
  }

  ASSERT_NO_FATAL_FAILURE(
      setManySingleFreeBodyContactSurfaceParams(cachedWorld, kNumBoxes));
  ASSERT_NO_FATAL_FAILURE(
      setManySingleFreeBodyContactSurfaceParams(referenceWorld, kNumBoxes));

  for (std::size_t i = 0u; i < 40u; ++i)
    cachedWorld->step();

  std::thread referenceThread([&]() {
    for (std::size_t i = 0u; i < 40u; ++i)
      referenceWorld->step();
  });
  referenceThread.join();

  ASSERT_NO_FATAL_FAILURE(expectManySingleFreeBodyContactWorldsMatch(
      referenceWorld, cachedWorld, kNumBoxes));
}

//==============================================================================
TEST(ConstraintSolver, CustomContactSurfaceHandlerKeepsConstructingConstraints)
{
  auto world = createWorld();
  world->setTimeStep(0.001);

  simulation::DeactivationOptions deactivation;
  deactivation.mEnabled = false;
  world->setDeactivationOptions(deactivation);

  auto* solver = world->getConstraintSolver();
  solver->setCollisionDetector(collision::DARTCollisionDetector::create());
  solver->setNumSimulationThreads(1u);

  auto defaultHandler = solver->getLastContactSurfaceHandler();
  auto customHandler = std::make_shared<CustomContactSurfaceHandler>();
  solver->addContactSurfaceHandler(customHandler);
  solver->removeContactSurfaceHandler(defaultHandler);

  world->addSkeleton(createSolverTestPlane("ground"));
  world->addSkeleton(createSolverTestBox(
      "box", Eigen::Vector3d::Ones(), Eigen::Vector3d(0.0, 0.0, 0.49), true));

  world->step();
  const auto firstStepCalls = customHandler->mNumCreateConstraintCalls;
  ASSERT_GT(firstStepCalls, 0u);

  world->step();
  EXPECT_GT(customHandler->mNumCreateConstraintCalls, firstStepCalls);
}

//==============================================================================
TEST(ConstraintSolver, RemovedCustomContactSurfaceHandlerDoesNotReuseConstraint)
{
  CustomContactConstraint::mNumDestroyed.store(0u);

  auto world = createWorld();
  world->setTimeStep(0.001);

  simulation::DeactivationOptions deactivation;
  deactivation.mEnabled = false;
  world->setDeactivationOptions(deactivation);

  auto* solver = world->getConstraintSolver();
  solver->setCollisionDetector(collision::DARTCollisionDetector::create());
  solver->setNumSimulationThreads(1u);

  auto defaultHandler = solver->getLastContactSurfaceHandler();
  auto customHandler = std::make_shared<CustomContactSurfaceHandler>();
  solver->addContactSurfaceHandler(customHandler);

  world->addSkeleton(createSolverTestPlane("ground"));
  world->addSkeleton(createSolverTestBox(
      "box", Eigen::Vector3d::Ones(), Eigen::Vector3d(0.0, 0.0, 0.49), true));

  world->step();
  const auto firstStepCalls = customHandler->mNumCreateConstraintCalls;
  ASSERT_GT(firstStepCalls, 0u);

  ASSERT_TRUE(solver->removeContactSurfaceHandler(customHandler));
  EXPECT_EQ(defaultHandler, solver->getLastContactSurfaceHandler());

  world->step();
  EXPECT_EQ(firstStepCalls, customHandler->mNumCreateConstraintCalls);
  EXPECT_EQ(firstStepCalls, CustomContactConstraint::mNumDestroyed.load());
}

//==============================================================================
TEST(ConstraintSolver, DirectSimulationThreadSettingSolvesGroupsInParallel)
{
  ExposedThreadedConstraintSolver solver;
  solver.setNumSimulationThreads(4);
  solver.addFakeConstrainedGroups(130, 100);

  solver.solveGroupsForTest();

  EXPECT_EQ(130, solver.getNumSolvedGroups());
  EXPECT_GT(solver.getMaxConcurrentSolves(), 1);
}

//==============================================================================
TEST(ConstraintSolver, PrepareForSimulationDoesNotUpdateManualConstraints)
{
  constraint::BoxedLcpConstraintSolver solver;
  auto manualConstraint = std::make_shared<CountingManualConstraint>();
  solver.addConstraint(manualConstraint);

  solver.prepareForSimulation();
  EXPECT_EQ(0u, manualConstraint->getNumUpdates());

  solver.solve();
  EXPECT_EQ(1u, manualConstraint->getNumUpdates());
}

//==============================================================================
TEST(ConstraintSolver, ParallelPreparationWarmsScratchOnWorkerThreads)
{
  ExposedThreadedConstraintSolver solver;
  solver.setNumSimulationThreads(4);
  solver.addFakeConstrainedGroups(130, 100);
  solver.recordReserveThreadsForTest();

  solver.reserveScratchForCurrentGroupsForTest();

  EXPECT_EQ(0, solver.getNumSolvedGroups());
  EXPECT_GT(solver.getNumReserveCalls(), 130);
  EXPECT_GT(solver.getNumReserveThreads(), 1u);

  solver.recordReserveThreadsForTest();
  solver.solveGroupsForTest();

  EXPECT_EQ(130, solver.getNumSolvedGroups());
  EXPECT_GT(solver.getMaxConcurrentSolves(), 1);
  EXPECT_EQ(0, solver.getNumReserveCalls());
}

//==============================================================================
TEST(ConstraintSolver, BoxedLcpScratchRetainsLargestPreparedGroup)
{
  ExposedBoxedLcpConstraintSolver solver;
  const std::vector<constraint::ConstraintBasePtr> largeGroup{
      std::make_shared<DiagonalConstraint>(128u)};
  const std::vector<constraint::ConstraintBasePtr> smallGroup{
      std::make_shared<DiagonalConstraint>(32u)};
  auto largeConstrainedGroup = solver.makeGroupForTest(largeGroup);
  auto smallConstrainedGroup = solver.makeGroupForTest(smallGroup);

  solver.reserveGroupScratchForTest(largeConstrainedGroup);
  solver.reserveGroupScratchForTest(smallConstrainedGroup);

  dart::test::ScopedHeapAllocationCounter counter;
  solver.solveGroupForTest(smallConstrainedGroup);
  solver.solveGroupForTest(largeConstrainedGroup);
  counter.stop();

  EXPECT_EQ(counter.allocationCount(), 0u);
  EXPECT_EQ(counter.allocationBytes(), 0u);
}

//==============================================================================
TEST(ConstraintSolver, MatrixFreeContactScratchRetainsPreparedGroup)
{
  std::vector<dynamics::SkeletonPtr> skeletons;
  auto* fixedBody = createFreeBody("fixed", false, skeletons);

  auto shape = std::make_shared<dynamics::BoxShape>(Eigen::Vector3d::Ones());
  auto* fixedShapeNode = fixedBody->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(shape);

  FakeCollisionDetector detector;
  FakeCollisionObject fixedObject(&detector, fixedShapeNode);

  constexpr std::size_t kNumContacts = 96u;
  std::vector<std::unique_ptr<FakeCollisionObject>> dynamicObjects;
  std::vector<collision::Contact> contacts;
  std::vector<constraint::ConstraintBasePtr> constraints;
  dynamicObjects.reserve(kNumContacts);
  contacts.reserve(kNumContacts);
  constraints.reserve(kNumContacts);

  for (std::size_t i = 0u; i < kNumContacts; ++i) {
    auto* dynamicBody
        = createFreeBody("dynamic_" + std::to_string(i), true, skeletons);
    auto* dynamicShapeNode = dynamicBody->createShapeNodeWith<
        dynamics::CollisionAspect,
        dynamics::DynamicsAspect>(shape);
    dynamicObjects.push_back(
        std::make_unique<FakeCollisionObject>(&detector, dynamicShapeNode));
    contacts.push_back(
        createContact(dynamicObjects.back().get(), &fixedObject));
    constraints.push_back(
        createContactConstraint<constraint::ContactConstraint>(
            contacts.back()));
  }

  ExposedBoxedLcpConstraintSolver solver;
  auto options = solver.getMatrixFreeContactSolverOptions();
  options.mEnabled = true;
  options.mMinRows = 1u;
  options.mMaxIterations = 5;
  solver.setMatrixFreeContactSolverOptions(options);

  auto constrainedGroup = solver.makeGroupForTest(constraints);
  solver.reserveGroupScratchForTest(constrainedGroup);

  dart::test::ScopedHeapAllocationCounter counter;
  solver.solveGroupForTest(constrainedGroup);
  counter.stop();

  EXPECT_EQ(counter.allocationCount(), 0u);
  EXPECT_EQ(counter.allocationBytes(), 0u);
}

//==============================================================================
TEST(ConstraintSolver, MatrixFreeContactSolverSeedsCachedImpulseResidual)
{
  std::vector<dynamics::SkeletonPtr> skeletons;
  auto* fixedBody = createFreeBody("fixed", false, skeletons);
  auto* dynamicBody = createFreeBody("dynamic", true, skeletons);

  auto shape = std::make_shared<dynamics::BoxShape>(Eigen::Vector3d::Ones());
  auto* fixedShapeNode = fixedBody->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(shape);
  auto* dynamicShapeNode = dynamicBody->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(shape);

  auto detector = collision::NativeCollisionDetector::create();
  auto collisionGroup
      = detector->createCollisionGroup(dynamicShapeNode, fixedShapeNode);

  collision::CollisionResult result;
  ASSERT_TRUE(
      collisionGroup->collide(collision::CollisionOption(true, 10u), &result));
  ASSERT_GT(result.getNumContacts(), 0u);

  auto contact = result.getContact(0);
  ASSERT_NE(nullptr, contact.userData);
  auto* cachedContact
      = static_cast<collision::native::CachedContact*>(contact.userData);
  cachedContact->cachedNormalImpulse = 100.0;
  cachedContact->cachedFrictionImpulse1 = 0.0;
  cachedContact->cachedFrictionImpulse2 = 0.0;
  cachedContact->hasCachedFrictionBasis = false;

  std::vector<constraint::ConstraintBasePtr> constraints{
      createContactConstraint<constraint::ContactConstraint>(contact)};

  ExposedBoxedLcpConstraintSolver solver;
  auto options = solver.getMatrixFreeContactSolverOptions();
  options.mEnabled = true;
  options.mMinRows = 1u;
  options.mMaxIterations = 1;
  options.mSor = 1.0;
  solver.setMatrixFreeContactSolverOptions(options);

  auto constrainedGroup = solver.makeGroupForTest(constraints);
  solver.solveGroupForTest(constrainedGroup);

  EXPECT_TRUE(std::isfinite(cachedContact->cachedNormalImpulse));
  EXPECT_LT(cachedContact->cachedNormalImpulse, 10.0);
}

//==============================================================================
TEST(ConstraintSolver, MatrixFreeContactSolverFallsBackWhenNotConverged)
{
  std::vector<dynamics::SkeletonPtr> skeletons;
  auto* fixedBody = createFreeBody("fixed", false, skeletons);
  auto* dynamicBody = createFreeBody("dynamic", true, skeletons);

  auto shape = std::make_shared<dynamics::BoxShape>(Eigen::Vector3d::Ones());
  auto* fixedShapeNode = fixedBody->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(shape);
  auto* dynamicShapeNode = dynamicBody->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(shape);

  auto detector = collision::NativeCollisionDetector::create();
  auto collisionGroup
      = detector->createCollisionGroup(dynamicShapeNode, fixedShapeNode);

  collision::CollisionResult result;
  ASSERT_TRUE(
      collisionGroup->collide(collision::CollisionOption(true, 10u), &result));
  ASSERT_GT(result.getNumContacts(), 0u);

  auto contact = result.getContact(0);
  std::vector<constraint::ConstraintBasePtr> constraints{
      createContactConstraint<constraint::ContactConstraint>(contact)};

  auto primarySolver = std::make_shared<CountingDantzigBoxedLcpSolver>();
  ExposedBoxedLcpConstraintSolver solver(primarySolver, nullptr);
  auto options = solver.getMatrixFreeContactSolverOptions();
  options.mEnabled = true;
  options.mMinRows = 1u;
  options.mMaxIterations = 1;
  options.mSor = 1.0;
  options.mDeltaTolerance = 0.0;
  options.mRelativeDeltaTolerance = 0.0;
  solver.setMatrixFreeContactSolverOptions(options);

  auto constrainedGroup = solver.makeGroupForTest(constraints);
  solver.solveGroupForTest(constrainedGroup);

  EXPECT_EQ(1u, primarySolver->getNumSolves());
}

//==============================================================================
TEST(ConstraintSolver, MatrixFreeContactSolverRejectsMixedFreeJointActuators)
{
  std::vector<dynamics::SkeletonPtr> skeletons;
  auto* fixedBody = createFreeBody("fixed", false, skeletons);
  auto* dynamicBody = createFreeBody("dynamic", true, skeletons);
  auto* dynamicJoint = dynamicBody->getParentJoint();
  ASSERT_NE(nullptr, dynamicJoint);
  dynamicJoint->setActuatorType(0u, dynamics::Joint::MIMIC);
  ASSERT_EQ(dynamics::Joint::MIMIC, dynamicJoint->getActuatorType(0u));
  ASSERT_EQ(dynamics::Joint::FORCE, dynamicJoint->getActuatorType());

  auto shape = std::make_shared<dynamics::BoxShape>(Eigen::Vector3d::Ones());
  auto* fixedShapeNode = fixedBody->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(shape);
  auto* dynamicShapeNode = dynamicBody->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(shape);

  auto detector = collision::NativeCollisionDetector::create();
  auto collisionGroup
      = detector->createCollisionGroup(dynamicShapeNode, fixedShapeNode);

  collision::CollisionResult result;
  ASSERT_TRUE(
      collisionGroup->collide(collision::CollisionOption(true, 10u), &result));
  ASSERT_GT(result.getNumContacts(), 0u);

  auto contact = result.getContact(0);
  std::vector<constraint::ConstraintBasePtr> constraints{
      createContactConstraint<constraint::ContactConstraint>(contact)};

  auto primarySolver = std::make_shared<CountingDantzigBoxedLcpSolver>();
  ExposedBoxedLcpConstraintSolver solver(primarySolver, nullptr);
  auto options = solver.getMatrixFreeContactSolverOptions();
  options.mEnabled = true;
  options.mMinRows = 1u;
  options.mMaxIterations = 30;
  solver.setMatrixFreeContactSolverOptions(options);

  auto constrainedGroup = solver.makeGroupForTest(constraints);
  solver.solveGroupForTest(constrainedGroup);

  EXPECT_EQ(1u, primarySolver->getNumSolves());
}

//==============================================================================
TEST(ConstraintSolver, ManualConstraintsForceSerialParallelGroupSolves)
{
  ExposedThreadedConstraintSolver solver;
  solver.setNumSimulationThreads(4);
  solver.addConstraint(std::make_shared<FakeConstraint>(1));
  solver.addFakeConstrainedGroups(130, 100);

  solver.solveGroupsForTest();

  EXPECT_EQ(130, solver.getNumSolvedGroups());
  EXPECT_EQ(1, solver.getMaxConcurrentSolves());
}

//==============================================================================
TEST(ConstraintSolver, DeactivationActiveAwakeGroupsSolveInParallel)
{
  ExposedThreadedConstraintSolver solver;
  solver.setDeactivationActive(true);
  solver.setNumSimulationThreads(4);
  solver.addFakeConstrainedGroups(130, 100);

  const auto candidate = dynamics::Skeleton::create("candidate");
  candidate->setSleepCandidate(true);
  candidate->setResting(false);
  candidate->setIslandIndex(0);
  solver.addSkeletonForTest(candidate);
  solver.setGroupRestingForTest(0, true);

  solver.solveGroupsForTest();

  EXPECT_EQ(130, solver.getNumSolvedGroups());
  EXPECT_GT(solver.getMaxConcurrentSolves(), 1);
  EXPECT_TRUE(candidate->isResting());
}

//==============================================================================
TEST(ConstraintSolver, DeactivationActiveSkipsAlreadyRestingGroupsInParallel)
{
  ExposedThreadedConstraintSolver solver;
  solver.setDeactivationActive(true);
  solver.setNumSimulationThreads(4);
  solver.addFakeConstrainedGroups(130, 100);

  const auto resting = dynamics::Skeleton::create("resting");
  resting->setSleepCandidate(true);
  resting->setResting(true);
  resting->setIslandIndex(0);
  solver.addSkeletonForTest(resting);
  solver.setGroupRestingForTest(0, true);

  solver.solveGroupsForTest();

  EXPECT_EQ(129, solver.getNumSolvedGroups());
  EXPECT_GT(solver.getMaxConcurrentSolves(), 1);
  EXPECT_TRUE(resting->isResting());
}

//==============================================================================
TEST(ConstraintSolver, SharedFixedContactSupportCanSolveGroupsInParallel)
{
  std::vector<dynamics::SkeletonPtr> skeletons;
  auto* fixedBody = createFreeBody("fixed", false, skeletons);

  auto shape = std::make_shared<dynamics::BoxShape>(Eigen::Vector3d::Ones());
  auto* fixedShapeNode = fixedBody->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(shape);

  FakeCollisionDetector detector;
  FakeCollisionObject fixedObject(&detector, fixedShapeNode);

  ExposedThreadedConstraintSolver solver;
  solver.setDeactivationActive(true);
  solver.setNumSimulationThreads(4);

  std::vector<std::unique_ptr<FakeCollisionObject>> dynamicObjects;
  std::vector<collision::Contact> contacts;
  dynamicObjects.reserve(130u);
  contacts.reserve(130u);

  for (std::size_t i = 0; i < 130u; ++i) {
    auto* dynamicBody
        = createFreeBody("dynamic_" + std::to_string(i), true, skeletons);
    auto* dynamicShapeNode = dynamicBody->createShapeNodeWith<
        dynamics::CollisionAspect,
        dynamics::DynamicsAspect>(shape);
    dynamicObjects.push_back(
        std::make_unique<FakeCollisionObject>(&detector, dynamicShapeNode));
    contacts.push_back(
        createContact(dynamicObjects.back().get(), &fixedObject));
    solver.addActiveConstraintForTest(
        createContactConstraint<constraint::ContactConstraint>(
            contacts.back()));
  }

  for (const auto& skeleton : skeletons)
    solver.addSkeletonForTest(skeleton);

  solver.buildGroupsForTest();
  solver.solveGroupsForTest();

  EXPECT_EQ(130, solver.getNumSolvedGroups());
  EXPECT_GT(solver.getMaxConcurrentSolves(), 1);
}

//==============================================================================
TEST(ConstraintSolver, SharedFixedCustomContactSupportForcesSerialAfterBuild)
{
  std::vector<dynamics::SkeletonPtr> skeletons;
  auto* fixedBody = createFreeBody("fixed", false, skeletons);

  auto shape = std::make_shared<dynamics::BoxShape>(Eigen::Vector3d::Ones());
  auto* fixedShapeNode = fixedBody->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(shape);

  FakeCollisionDetector detector;
  FakeCollisionObject fixedObject(&detector, fixedShapeNode);

  ExposedThreadedConstraintSolver solver;
  solver.setDeactivationActive(true);
  solver.setNumSimulationThreads(4);

  std::vector<std::unique_ptr<FakeCollisionObject>> dynamicObjects;
  std::vector<collision::Contact> contacts;
  dynamicObjects.reserve(130u);
  contacts.reserve(130u);

  for (std::size_t i = 0; i < 130u; ++i) {
    auto* dynamicBody
        = createFreeBody("dynamic_" + std::to_string(i), true, skeletons);
    auto* dynamicShapeNode = dynamicBody->createShapeNodeWith<
        dynamics::CollisionAspect,
        dynamics::DynamicsAspect>(shape);
    dynamicObjects.push_back(
        std::make_unique<FakeCollisionObject>(&detector, dynamicShapeNode));
    contacts.push_back(
        createContact(dynamicObjects.back().get(), &fixedObject));
    solver.addActiveConstraintForTest(
        createContactConstraint<CustomContactConstraint>(contacts.back()));
  }

  for (const auto& skeleton : skeletons)
    solver.addSkeletonForTest(skeleton);

  solver.buildGroupsForTest();
  solver.solveGroupsForTest();

  EXPECT_EQ(130, solver.getNumSolvedGroups());
  EXPECT_EQ(1, solver.getMaxConcurrentSolves());
}

//==============================================================================
TEST(ConstraintSolver, MovingFixedContactSupportContributesRelVelocity)
{
  std::vector<dynamics::SkeletonPtr> skeletons;
  auto* fixedBody = createFreeBody("fixed", false, skeletons);
  auto* fixedJoint
      = static_cast<dynamics::FreeJoint*>(fixedBody->getParentJoint());
  fixedJoint->setLinearVelocity(Eigen::Vector3d(0.0, 0.0, 0.25));

  auto* dynamicBody = createFreeBody("dynamic", true, skeletons);

  auto shape = std::make_shared<dynamics::BoxShape>(Eigen::Vector3d::Ones());
  auto* fixedShapeNode = fixedBody->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(shape);
  auto* dynamicShapeNode = dynamicBody->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(shape);

  FakeCollisionDetector detector;
  FakeCollisionObject fixedObject(&detector, fixedShapeNode);
  FakeCollisionObject dynamicObject(&detector, dynamicShapeNode);

  auto contact = createContact(&dynamicObject, &fixedObject);
  ExposedContactConstraint constraint(
      contact, 0.001, constraint::ContactSurfaceParams{});

  double x[3] = {0.0, 0.0, 0.0};
  double lo[3] = {0.0, 0.0, 0.0};
  double hi[3] = {0.0, 0.0, 0.0};
  double b[3] = {0.0, 0.0, 0.0};
  double w[3] = {0.0, 0.0, 0.0};
  int findex[3] = {-1, -1, -1};
  constraint::ConstraintInfo info;
  info.x = x;
  info.lo = lo;
  info.hi = hi;
  info.b = b;
  info.w = w;
  info.findex = findex;
  info.invTimeStep = 1000.0;
  constraint.getInformation(&info);

  EXPECT_NEAR(0.25, b[0], 1e-12);
}

//==============================================================================
TEST(ConstraintSolver, ContactConstraintCachesSolvedImpulse)
{
  std::vector<dynamics::SkeletonPtr> skeletons;
  auto* fixedBody = createFreeBody("fixed", false, skeletons);
  auto* dynamicBody = createFreeBody("dynamic", true, skeletons);

  auto shape = std::make_shared<dynamics::BoxShape>(Eigen::Vector3d::Ones());
  auto* fixedShapeNode = fixedBody->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(shape);
  auto* dynamicShapeNode = dynamicBody->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(shape);

  auto detector = collision::NativeCollisionDetector::create();
  auto group = detector->createCollisionGroup(dynamicShapeNode, fixedShapeNode);

  collision::CollisionResult result;
  ASSERT_TRUE(group->collide(collision::CollisionOption(true, 10u), &result));
  ASSERT_GT(result.getNumContacts(), 0u);

  auto contact = result.getContact(0);
  ASSERT_NE(nullptr, contact.userData);
  auto* cachedContact
      = static_cast<collision::native::CachedContact*>(contact.userData);
  cachedContact->cachedNormalImpulse = 1.25;
  cachedContact->cachedFrictionImpulse1 = -0.5;
  cachedContact->cachedFrictionImpulse2 = 0.75;

  contact.userData = cachedContact;

  ExposedContactConstraint constraint(
      contact, 0.001, constraint::ContactSurfaceParams{});

  double x[3] = {1.0, 2.0, 3.0};
  double lo[3] = {0.0, 0.0, 0.0};
  double hi[3] = {0.0, 0.0, 0.0};
  double b[3] = {0.0, 0.0, 0.0};
  double w[3] = {0.0, 0.0, 0.0};
  int findex[3] = {-1, -1, -1};
  constraint::ConstraintInfo info;
  info.x = x;
  info.lo = lo;
  info.hi = hi;
  info.b = b;
  info.w = w;
  info.findex = findex;
  info.invTimeStep = 1000.0;
  constraint.getInformation(&info);

  EXPECT_NEAR(1.25, x[0], 1e-12);
  EXPECT_NEAR(0.0, x[1], 1e-12);
  EXPECT_NEAR(0.0, x[2], 1e-12);
  EXPECT_NEAR(0.0, cachedContact->cachedFrictionImpulse1, 1e-12);
  EXPECT_NEAR(0.0, cachedContact->cachedFrictionImpulse2, 1e-12);
  EXPECT_FALSE(cachedContact->hasCachedFrictionBasis);

  double lambda[3] = {2.5, -0.25, 0.125};
  constraint.applyImpulse(lambda);

  EXPECT_NEAR(2.5, cachedContact->cachedNormalImpulse, 1e-12);
  EXPECT_NEAR(-0.25, cachedContact->cachedFrictionImpulse1, 1e-12);
  EXPECT_NEAR(0.125, cachedContact->cachedFrictionImpulse2, 1e-12);
  EXPECT_TRUE(cachedContact->hasCachedFrictionBasis);
  EXPECT_FALSE(cachedContact->cachedFrictionBasis1.isZero());
  EXPECT_FALSE(cachedContact->cachedFrictionBasis2.isZero());

  ExposedContactConstraint warmConstraint(
      contact, 0.001, constraint::ContactSurfaceParams{});
  double warmX[3] = {0.0, 0.0, 0.0};
  double warmLo[3] = {0.0, 0.0, 0.0};
  double warmHi[3] = {0.0, 0.0, 0.0};
  double warmB[3] = {0.0, 0.0, 0.0};
  double warmW[3] = {0.0, 0.0, 0.0};
  int warmFindex[3] = {-1, -1, -1};
  constraint::ConstraintInfo warmInfo;
  warmInfo.x = warmX;
  warmInfo.lo = warmLo;
  warmInfo.hi = warmHi;
  warmInfo.b = warmB;
  warmInfo.w = warmW;
  warmInfo.findex = warmFindex;
  warmInfo.invTimeStep = 1000.0;
  warmConstraint.getInformation(&warmInfo);

  EXPECT_NEAR(2.5, warmX[0], 1e-12);
  EXPECT_NEAR(-0.25, warmX[1], 1e-12);
  EXPECT_NEAR(0.125, warmX[2], 1e-12);
}

//==============================================================================
TEST(ConstraintSolver, ContactConstraintClearsFrictionForChangedFrictionBasis)
{
  std::vector<dynamics::SkeletonPtr> skeletons;
  auto* fixedBody = createFreeBody("fixed", false, skeletons);
  auto* dynamicBody = createFreeBody("dynamic", true, skeletons);

  auto shape = std::make_shared<dynamics::BoxShape>(Eigen::Vector3d::Ones());
  auto* fixedShapeNode = fixedBody->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(shape);
  auto* dynamicShapeNode = dynamicBody->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(shape);

  auto detector = collision::NativeCollisionDetector::create();
  auto group = detector->createCollisionGroup(dynamicShapeNode, fixedShapeNode);

  collision::CollisionResult result;
  ASSERT_TRUE(group->collide(collision::CollisionOption(true, 10u), &result));
  ASSERT_GT(result.getNumContacts(), 0u);

  auto contact = result.getContact(0);
  ASSERT_NE(nullptr, contact.userData);
  auto* cachedContact
      = static_cast<collision::native::CachedContact*>(contact.userData);

  constraint::ContactSurfaceParams firstParams;
  firstParams.mFirstFrictionalDirection
      = makeContactTangentDirection(contact.normal, Eigen::Vector3d::UnitX());
  Eigen::Vector3d n = contact.normal;
  ASSERT_GT(n.squaredNorm(), DART_CONTACT_CONSTRAINT_EPSILON_SQUARED);
  n.normalize();
  constraint::ContactSurfaceParams secondParams;
  secondParams.mFirstFrictionalDirection
      = n.cross(firstParams.mFirstFrictionalDirection).normalized();

  ExposedContactConstraint solved(contact, 0.001, firstParams);
  double lambda[3] = {2.5, -0.25, 0.125};
  solved.applyImpulse(lambda);
  ASSERT_TRUE(cachedContact->hasCachedFrictionBasis);

  ExposedContactConstraint changed(contact, 0.001, secondParams);
  double x[3] = {1.0, 2.0, 3.0};
  double lo[3] = {0.0, 0.0, 0.0};
  double hi[3] = {0.0, 0.0, 0.0};
  double b[3] = {0.0, 0.0, 0.0};
  double w[3] = {0.0, 0.0, 0.0};
  int findex[3] = {-1, -1, -1};
  constraint::ConstraintInfo info;
  info.x = x;
  info.lo = lo;
  info.hi = hi;
  info.b = b;
  info.w = w;
  info.findex = findex;
  info.invTimeStep = 1000.0;
  changed.getInformation(&info);

  EXPECT_NEAR(2.5, x[0], 1e-12);
  EXPECT_NEAR(0.0, x[1], 1e-12);
  EXPECT_NEAR(0.0, x[2], 1e-12);
  EXPECT_NEAR(0.0, cachedContact->cachedFrictionImpulse1, 1e-12);
  EXPECT_NEAR(0.0, cachedContact->cachedFrictionImpulse2, 1e-12);
  EXPECT_FALSE(cachedContact->hasCachedFrictionBasis);
}

//==============================================================================
TEST(ConstraintSolver, ContactConstraintDoesNotSeedFrictionInPositionPhase)
{
  std::vector<dynamics::SkeletonPtr> skeletons;
  auto* fixedBody = createFreeBody("fixed", false, skeletons);
  auto* dynamicBody = createFreeBody("dynamic", true, skeletons);

  auto shape = std::make_shared<dynamics::BoxShape>(Eigen::Vector3d::Ones());
  auto* fixedShapeNode = fixedBody->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(shape);
  auto* dynamicShapeNode = dynamicBody->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(shape);

  auto detector = collision::NativeCollisionDetector::create();
  auto group = detector->createCollisionGroup(dynamicShapeNode, fixedShapeNode);

  collision::CollisionResult result;
  ASSERT_TRUE(group->collide(collision::CollisionOption(true, 10u), &result));
  ASSERT_GT(result.getNumContacts(), 0u);

  auto contact = result.getContact(0);
  ASSERT_NE(nullptr, contact.userData);
  auto* cachedContact
      = static_cast<collision::native::CachedContact*>(contact.userData);
  contact.userData = cachedContact;

  ExposedContactConstraint velocityConstraint(
      contact, 0.001, constraint::ContactSurfaceParams{});
  double lambda[3] = {1.25, -0.5, 0.75};
  velocityConstraint.applyImpulse(lambda);
  ASSERT_TRUE(cachedContact->hasCachedFrictionBasis);

  ExposedContactConstraint positionConstraint(
      contact, 0.001, constraint::ContactSurfaceParams{});

  double x[3] = {1.0, 2.0, 3.0};
  double lo[3] = {0.0, 0.0, 0.0};
  double hi[3] = {0.0, 0.0, 0.0};
  double b[3] = {0.0, 0.0, 0.0};
  double w[3] = {0.0, 0.0, 0.0};
  int findex[3] = {-1, -1, -1};
  constraint::ConstraintInfo info;
  info.x = x;
  info.lo = lo;
  info.hi = hi;
  info.b = b;
  info.w = w;
  info.findex = findex;
  info.invTimeStep = 1000.0;
  info.phase = constraint::ConstraintPhase::Position;
  positionConstraint.getInformation(&info);

  EXPECT_NEAR(1.25, x[0], 1e-12);
  EXPECT_NEAR(0.0, x[1], 1e-12);
  EXPECT_NEAR(0.0, x[2], 1e-12);
  EXPECT_NEAR(-0.5, cachedContact->cachedFrictionImpulse1, 1e-12);
  EXPECT_NEAR(0.75, cachedContact->cachedFrictionImpulse2, 1e-12);
  EXPECT_TRUE(cachedContact->hasCachedFrictionBasis);
}

//==============================================================================
TEST(ConstraintSolver, ContactConstraintIgnoresForeignUserData)
{
  std::vector<dynamics::SkeletonPtr> skeletons;
  auto* fixedBody = createFreeBody("fixed", false, skeletons);
  auto* dynamicBody = createFreeBody("dynamic", true, skeletons);

  auto shape = std::make_shared<dynamics::BoxShape>(Eigen::Vector3d::Ones());
  auto* fixedShapeNode = fixedBody->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(shape);
  auto* dynamicShapeNode = dynamicBody->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(shape);

  FakeCollisionDetector detector;
  FakeCollisionObject fixedObject(&detector, fixedShapeNode);
  FakeCollisionObject dynamicObject(&detector, dynamicShapeNode);

  struct ForeignPayload
  {
    double value1;
    double value2;
    double value3;
  };
  ForeignPayload payload{10.0, 20.0, 30.0};

  auto contact = createContact(&dynamicObject, &fixedObject);
  contact.userData = &payload;

  ExposedContactConstraint constraint(
      contact, 0.001, constraint::ContactSurfaceParams{});

  double x[3] = {1.0, 2.0, 3.0};
  double lo[3] = {0.0, 0.0, 0.0};
  double hi[3] = {0.0, 0.0, 0.0};
  double b[3] = {0.0, 0.0, 0.0};
  double w[3] = {0.0, 0.0, 0.0};
  int findex[3] = {-1, -1, -1};
  constraint::ConstraintInfo info;
  info.x = x;
  info.lo = lo;
  info.hi = hi;
  info.b = b;
  info.w = w;
  info.findex = findex;
  info.invTimeStep = 1000.0;
  constraint.getInformation(&info);

  EXPECT_NEAR(0.0, x[0], 1e-12);
  EXPECT_NEAR(0.0, x[1], 1e-12);
  EXPECT_NEAR(0.0, x[2], 1e-12);

  double lambda[3] = {2.5, -0.25, 0.125};
  constraint.applyImpulse(lambda);

  EXPECT_NEAR(10.0, payload.value1, 1e-12);
  EXPECT_NEAR(20.0, payload.value2, 1e-12);
  EXPECT_NEAR(30.0, payload.value3, 1e-12);
}

//==============================================================================
TEST(ConstraintSolver, ContactConstraintIgnoresForeignNativeUserData)
{
  std::vector<dynamics::SkeletonPtr> skeletons;
  auto* fixedBody = createFreeBody("fixed", false, skeletons);
  auto* dynamicBody = createFreeBody("dynamic", true, skeletons);

  auto shape = std::make_shared<dynamics::BoxShape>(Eigen::Vector3d::Ones());
  auto* fixedShapeNode = fixedBody->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(shape);
  auto* dynamicShapeNode = dynamicBody->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(shape);

  auto detector = collision::NativeCollisionDetector::create();
  ExposedNativeCollisionObject fixedObject(detector.get(), fixedShapeNode);
  ExposedNativeCollisionObject dynamicObject(detector.get(), dynamicShapeNode);

  struct ForeignPayload
  {
    double value1;
    double value2;
    double value3;
  };
  ForeignPayload payload{10.0, 20.0, 30.0};

  auto contact = createContact(&dynamicObject, &fixedObject);
  contact.userData = &payload;

  ExposedContactConstraint constraint(
      contact, 0.001, constraint::ContactSurfaceParams{});

  double x[3] = {1.0, 2.0, 3.0};
  double lo[3] = {0.0, 0.0, 0.0};
  double hi[3] = {0.0, 0.0, 0.0};
  double b[3] = {0.0, 0.0, 0.0};
  double w[3] = {0.0, 0.0, 0.0};
  int findex[3] = {-1, -1, -1};
  constraint::ConstraintInfo info;
  info.x = x;
  info.lo = lo;
  info.hi = hi;
  info.b = b;
  info.w = w;
  info.findex = findex;
  info.invTimeStep = 1000.0;
  constraint.getInformation(&info);

  EXPECT_NEAR(0.0, x[0], 1e-12);
  EXPECT_NEAR(0.0, x[1], 1e-12);
  EXPECT_NEAR(0.0, x[2], 1e-12);

  double lambda[3] = {2.5, -0.25, 0.125};
  constraint.applyImpulse(lambda);

  EXPECT_NEAR(10.0, payload.value1, 1e-12);
  EXPECT_NEAR(20.0, payload.value2, 1e-12);
  EXPECT_NEAR(30.0, payload.value3, 1e-12);
}

//==============================================================================
TEST(ConstraintSolver, ExactCoulombAdapterBuildsFromRealContactConstraint)
{
  std::vector<dynamics::SkeletonPtr> skeletons;
  auto* fixedBody = createFreeBody("fixed", false, skeletons);
  auto* fixedJoint
      = static_cast<dynamics::FreeJoint*>(fixedBody->getParentJoint());
  fixedJoint->setLinearVelocity(Eigen::Vector3d(0.0, 0.0, 0.25));

  auto* dynamicBody = createFreeBody("dynamic", true, skeletons);

  auto shape = std::make_shared<dynamics::BoxShape>(Eigen::Vector3d::Ones());
  auto* fixedShapeNode = fixedBody->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(shape);
  auto* dynamicShapeNode = dynamicBody->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(shape);

  FakeCollisionDetector detector;
  FakeCollisionObject fixedObject(&detector, fixedShapeNode);
  FakeCollisionObject dynamicObject(&detector, dynamicShapeNode);

  auto contact = createContact(&dynamicObject, &fixedObject);
  auto constraint
      = createContactConstraint<constraint::ContactConstraint>(contact);

  constraint::detail::ExactCoulombConstraintBuildOptions options;
  options.invTimeStep = 1000.0;

  auto result = constraint::detail::buildExactCoulombConstraintProblem(
      {constraint.get()}, options);

  ASSERT_EQ(
      result.status,
      constraint::detail::ExactCoulombConstraintBuildStatus::Success);
  ASSERT_EQ(result.contactProblem.getDimension(), 3);
  EXPECT_NEAR(result.contactProblem.coefficients[0], 1.0, 1e-12);
  EXPECT_NEAR(result.contactProblem.freeVelocity[0], -0.25, 1e-12);
  EXPECT_NEAR(result.contactProblem.freeVelocity[1], 0.0, 1e-12);
  EXPECT_NEAR(result.contactProblem.freeVelocity[2], 0.0, 1e-12);
  EXPECT_TRUE(result.delassus.allFinite());
  EXPECT_TRUE(result.delassus.isApprox(result.delassus.transpose(), 1e-12));
  EXPECT_GT(result.delassus(0, 0), 0.0);
}

//==============================================================================
TEST(ConstraintSolver, ExactCoulombContactRowOperatorMatchesImpulseTests)
{
  std::vector<dynamics::SkeletonPtr> skeletons;
  auto* fixedBody = createFreeBody("fixed", false, skeletons);
  auto* dynamicBodyA = createFreeBody("dynamicA", true, skeletons);
  auto* dynamicBodyB = createFreeBody("dynamicB", true, skeletons);
  auto* dynamicBodyC = createFreeBody("dynamicC", true, skeletons);

  auto shape = std::make_shared<dynamics::BoxShape>(Eigen::Vector3d::Ones());
  auto* fixedShapeNode = fixedBody->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(shape);
  auto* dynamicShapeNodeA = dynamicBodyA->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(shape);
  auto* dynamicShapeNodeB = dynamicBodyB->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(shape);
  auto* dynamicShapeNodeC = dynamicBodyC->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(shape);

  FakeCollisionDetector detector;
  FakeCollisionObject fixedObject(&detector, fixedShapeNode);
  FakeCollisionObject dynamicObjectA(&detector, dynamicShapeNodeA);
  FakeCollisionObject dynamicObjectB(&detector, dynamicShapeNodeB);
  FakeCollisionObject dynamicObjectC(&detector, dynamicShapeNodeC);

  // Mix fixed-dynamic and dynamic-dynamic contacts so the row operator has
  // to reproduce shared-body Delassus coupling, not only diagonal blocks.
  auto contactAFixed = createContact(&dynamicObjectA, &fixedObject);
  auto contactBFixed = createContact(&dynamicObjectB, &fixedObject);
  auto contactAB = createContact(&dynamicObjectA, &dynamicObjectB);
  auto contactAFixedOffset = createContact(&dynamicObjectA, &fixedObject);
  contactAFixedOffset.point = Eigen::Vector3d(0.2, -0.3, 0.1);
  contactAFixedOffset.normal = Eigen::Vector3d(0.1, -0.2, 1.0).normalized();
  auto contactABOffset = createContact(&dynamicObjectA, &dynamicObjectB);
  contactABOffset.point = Eigen::Vector3d(-0.15, 0.25, -0.05);
  contactABOffset.normal = Eigen::Vector3d(-0.2, 0.15, 1.0).normalized();
  auto contactCFixed = createContact(&dynamicObjectC, &fixedObject);

  auto constraintAFixed
      = createContactConstraint<constraint::ContactConstraint>(contactAFixed);
  auto constraintBFixed
      = createContactConstraint<constraint::ContactConstraint>(contactBFixed);
  auto constraintAB
      = createContactConstraint<constraint::ContactConstraint>(contactAB);
  auto constraintAFixedOffset
      = createContactConstraint<constraint::ContactConstraint>(
          contactAFixedOffset);
  auto constraintABOffset
      = createContactConstraint<constraint::ContactConstraint>(contactABOffset);
  auto constraintCFixed
      = createContactConstraint<constraint::ContactConstraint>(contactCFixed);

  const std::vector<constraint::ConstraintBase*> constraints{
      constraintAFixed.get(),
      constraintBFixed.get(),
      constraintAB.get(),
      constraintAFixedOffset.get(),
      constraintABOffset.get(),
      constraintCFixed.get()};

  constraint::detail::ExactCoulombConstraintBuildOptions buildOptions;
  buildOptions.invTimeStep = 1000.0;
  auto problem = constraint::detail::buildExactCoulombConstraintProblem(
      constraints, buildOptions);
  ASSERT_EQ(
      problem.status,
      constraint::detail::ExactCoulombConstraintBuildStatus::Success);
  const Eigen::Index dimension = problem.contactProblem.getDimension();
  ASSERT_EQ(dimension, 18);
  ASSERT_TRUE(problem.delassus.allFinite());

  constraint::detail::ExactCoulombContactRowOperator rowOperator;
  ASSERT_TRUE(rowOperator.build(constraints));
  ASSERT_EQ(rowOperator.getDimension(), dimension);

  const auto& schedule = rowOperator.getColoredBlockGaussSeidelSchedule();
  ASSERT_EQ(schedule.manifolds.size(), 4u);
  ASSERT_EQ(schedule.colors.size(), 3u);
  EXPECT_TRUE(schedule.hasUsableParallelism());
  EXPECT_EQ(
      schedule.manifolds[0].canonicalBodyPair, (std::array<int, 2>{-1, 0}));
  EXPECT_EQ(
      schedule.manifolds[1].canonicalBodyPair, (std::array<int, 2>{-1, 1}));
  EXPECT_EQ(
      schedule.manifolds[2].canonicalBodyPair, (std::array<int, 2>{0, 1}));
  EXPECT_EQ(
      schedule.manifolds[3].canonicalBodyPair, (std::array<int, 2>{-1, 2}));
  EXPECT_EQ(schedule.manifolds[0].contacts, (std::vector<Eigen::Index>{0, 3}));
  EXPECT_EQ(schedule.manifolds[1].contacts, (std::vector<Eigen::Index>{1}));
  EXPECT_EQ(schedule.manifolds[2].contacts, (std::vector<Eigen::Index>{2, 4}));
  EXPECT_EQ(schedule.manifolds[3].contacts, (std::vector<Eigen::Index>{5}));
  EXPECT_EQ(
      schedule.manifolds[0].writeContacts,
      (std::vector<Eigen::Index>{0, 2, 3, 4}));
  EXPECT_EQ(
      schedule.manifolds[1].writeContacts,
      (std::vector<Eigen::Index>{1, 2, 4}));
  EXPECT_EQ(
      schedule.manifolds[2].writeContacts,
      (std::vector<Eigen::Index>{0, 1, 2, 3, 4}));
  EXPECT_EQ(
      schedule.manifolds[3].writeContacts, (std::vector<Eigen::Index>{5}));
  EXPECT_EQ(schedule.colors[0], (std::vector<std::size_t>{0u, 3u}));
  EXPECT_EQ(schedule.colors[1], (std::vector<std::size_t>{1u}));
  EXPECT_EQ(schedule.colors[2], (std::vector<std::size_t>{2u}));

  for (const auto& color : schedule.colors) {
    for (std::size_t first = 0u; first < color.size(); ++first) {
      for (std::size_t second = 0u; second < first; ++second) {
        const auto& firstWrites
            = schedule.manifolds[color[first]].writeContacts;
        const auto& secondWrites
            = schedule.manifolds[color[second]].writeContacts;
        std::vector<Eigen::Index> intersection;
        std::set_intersection(
            firstWrites.begin(),
            firstWrites.end(),
            secondWrites.begin(),
            secondWrites.end(),
            std::back_inserter(intersection));
        EXPECT_TRUE(intersection.empty());
      }
    }
  }

  constraint::detail::ExactCoulombContactRowOperator rebuiltRowOperator;
  ASSERT_TRUE(rebuiltRowOperator.build(constraints));
  const auto& rebuiltSchedule
      = rebuiltRowOperator.getColoredBlockGaussSeidelSchedule();
  ASSERT_EQ(rebuiltSchedule.manifolds.size(), schedule.manifolds.size());
  EXPECT_EQ(rebuiltSchedule.colors, schedule.colors);
  for (std::size_t manifold = 0u; manifold < schedule.manifolds.size();
       ++manifold) {
    EXPECT_EQ(
        rebuiltSchedule.manifolds[manifold].canonicalBodyPair,
        schedule.manifolds[manifold].canonicalBodyPair);
    EXPECT_EQ(
        rebuiltSchedule.manifolds[manifold].contacts,
        schedule.manifolds[manifold].contacts);
    EXPECT_EQ(
        rebuiltSchedule.manifolds[manifold].writeContacts,
        schedule.manifolds[manifold].writeContacts);
  }
  const std::vector<constraint::ConstraintBase*> emptyConstraints;
  EXPECT_FALSE(rebuiltRowOperator.build(emptyConstraints));
  EXPECT_FALSE(rebuiltRowOperator.isBuilt());
  EXPECT_EQ(rebuiltRowOperator.getDimension(), 0);
  EXPECT_TRUE(rebuiltRowOperator.getColoredBlockGaussSeidelSchedule()
                  .manifolds.empty());
  EXPECT_TRUE(
      rebuiltRowOperator.getColoredBlockGaussSeidelSchedule().colors.empty());

  Eigen::MatrixXd rowDelassus;
  rowOperator.assembleDense(rowDelassus);
  ASSERT_EQ(rowDelassus.rows(), dimension);
  EXPECT_TRUE(rowDelassus.allFinite());
  EXPECT_TRUE(rowDelassus.isApprox(problem.delassus, 1e-9))
      << "row-assembled Delassus deviates from the impulse-test snapshot:\n"
      << (rowDelassus - problem.delassus);

  Eigen::VectorXd input(dimension);
  for (Eigen::Index row = 0; row < dimension; ++row)
    input[row] = std::sin(0.37 * static_cast<double>(row + 1));

  ExposedExactCoulombParallelEligibilitySolver parallelSolver;
  parallelSolver.setNumSimulationThreads(4u);

  // Five indices split across four requested participants form three
  // nonempty ceiling-sized chunks. Prove the bridge reports the threads that
  // actually execute callbacks, not the configured thread limit.
  std::array<int, 5> callCounts{};
  std::mutex fiveWayMutex;
  std::set<std::thread::id> fiveWayThreads;
  auto fiveWayWork = [&](std::size_t index) {
    std::lock_guard<std::mutex> lock(fiveWayMutex);
    ++callCounts[index];
    fiveWayThreads.insert(std::this_thread::get_id());
  };
  EXPECT_EQ(
      parallelSolver.parallelForConstraintWorkForTest(5u, fiveWayWork), 3u);
  EXPECT_EQ(fiveWayThreads.size(), 3u);
  for (int callCount : callCounts)
    EXPECT_EQ(callCount, 1);

  std::size_t maxParticipants = 0u;
  auto parallelFor = [&](std::size_t count, auto& work) -> std::size_t {
    // Keep evidence local to this one scatter or gather phase. This makes the
    // reported count directly comparable to distinct executing threads.
    std::mutex participantMutex;
    std::set<std::thread::id> participantThreads;
    auto recordedWork = [&](std::size_t index) {
      {
        std::lock_guard<std::mutex> lock(participantMutex);
        participantThreads.insert(std::this_thread::get_id());
      }
      work(index);
    };
    const std::size_t participants
        = parallelSolver.parallelForConstraintWorkForTest(count, recordedWork);
    EXPECT_EQ(participants, participantThreads.size());
    maxParticipants = std::max(maxParticipants, participants);
    return participants;
  };

  for (int sample = 0; sample < 3; ++sample) {
    Eigen::VectorXd rowProduct(dimension);
    rowOperator.apply(input, rowProduct);

    // Lock the pre-colored scalar row arithmetic: scatter rows in input order,
    // then gather rows in output order. Coloring is metadata-only until the
    // explicit inner-BGS opt-in is selected.
    std::vector<Eigen::Vector6d> referenceVelocityChanges(
        rowOperator.bodies.size(), Eigen::Vector6d::Zero());
    for (std::size_t rowIndex = 0u; rowIndex < rowOperator.rows.size();
         ++rowIndex) {
      const double impulse = input[static_cast<Eigen::Index>(rowIndex)];
      if (impulse == 0.0)
        continue;
      const auto& row = rowOperator.rows[rowIndex];
      for (int side = 0; side < 2; ++side) {
        const int bodyIndex = row.bodyIndices[side];
        if (bodyIndex < 0)
          continue;
        referenceVelocityChanges[static_cast<std::size_t>(bodyIndex)].noalias()
            += row.unitVelocityChanges[side] * impulse;
      }
    }
    Eigen::VectorXd scalarReference(dimension);
    for (std::size_t rowIndex = 0u; rowIndex < rowOperator.rows.size();
         ++rowIndex) {
      const auto& row = rowOperator.rows[rowIndex];
      double value = 0.0;
      for (int side = 0; side < 2; ++side) {
        const int bodyIndex = row.bodyIndices[side];
        if (bodyIndex < 0)
          continue;
        value += row.jacobians[side].dot(
            referenceVelocityChanges[static_cast<std::size_t>(bodyIndex)]);
      }
      scalarReference[static_cast<Eigen::Index>(rowIndex)] = value;
    }
    EXPECT_TRUE((rowProduct.array() == scalarReference.array()).all())
        << "contact-row arithmetic changed from the scalar reference for "
           "sample "
        << sample;

    Eigen::VectorXd parallelRowProduct(dimension);
    rowOperator.applyParallel(input, parallelRowProduct, parallelFor);
    EXPECT_TRUE((parallelRowProduct.array() == rowProduct.array()).all())
        << "parallel row product is not bit-identical for sample " << sample;

    const Eigen::VectorXd denseProduct = problem.delassus * input;
    EXPECT_TRUE(rowProduct.isApprox(denseProduct, 1e-9))
        << "row product deviates for sample " << sample << ": "
        << (rowProduct - denseProduct).transpose();
    input = input.reverse().eval() * -0.625;
  }
  EXPECT_GT(maxParticipants, 1u);

  for (Eigen::Index contact = 0; contact < dimension / 3; ++contact) {
    const Eigen::Matrix3d denseBlock
        = problem.delassus.block<3, 3>(3 * contact, 3 * contact);
    EXPECT_TRUE(rowOperator.diagonalBlock(contact).isApprox(denseBlock, 1e-9));

    const Eigen::Vector3d delta
        = static_cast<double>(contact + 1) * Eigen::Vector3d(0.5, -0.25, 0.125);
    Eigen::VectorXd accumulator
        = Eigen::VectorXd::LinSpaced(dimension, -0.125, 0.375);
    const Eigen::VectorXd initialAccumulator = accumulator;
    rowOperator.accumulateBlockColumns(contact, delta, accumulator);
    const Eigen::VectorXd denseColumns
        = initialAccumulator
          + problem.delassus.middleCols<3>(3 * contact) * delta;
    EXPECT_TRUE(accumulator.isApprox(denseColumns, 1e-9))
        << "block columns deviate for contact " << contact << ": "
        << (accumulator - denseColumns).transpose();
  }
}

//==============================================================================
TEST(
    ConstraintSolver,
    ExactCoulombColoredBlockGaussSeidelIsDeterministicAcrossThreadCounts)
{
  struct SolveRecord
  {
    constraint::ExactCoulombFbfConstraintSolverStatus status;
    math::detail::ExactCoulombFbfStatus fbfStatus;
    math::detail::CoulombConeResidual residual;
    double stepSize = std::numeric_limits<double>::quiet_NaN();
    int iterations = 0;
    std::vector<Eigen::Vector3d> reactions;
    bool contactRowOperatorUsed = false;
    bool coloredUsed = false;
    std::size_t coloredSolves = 0u;
    std::size_t coloredDispatches = 0u;
    std::size_t coloredParticipants = 0u;
    std::size_t coloredManifolds = 0u;
    std::size_t coloredColors = 0u;
    std::size_t coloredMaxManifoldsPerColor = 0u;
    std::vector<int> coloredLogicalCpuIds;
    std::vector<int> coloredMaxPhaseLogicalCpuIds;
  };

  const auto runSolve = [](std::size_t requestedThreads, bool coloredEnabled) {
    std::vector<dynamics::SkeletonPtr> skeletons;
    auto* fixedBody = createFreeBody("colored_fixed", false, skeletons);
    auto shape = std::make_shared<dynamics::BoxShape>(Eigen::Vector3d::Ones());
    auto* fixedShapeNode = fixedBody->createShapeNodeWith<
        dynamics::CollisionAspect,
        dynamics::DynamicsAspect>(shape);

    FakeCollisionDetector detector;
    FakeCollisionObject fixedObject(&detector, fixedShapeNode);
    std::vector<std::unique_ptr<FakeCollisionObject>> dynamicObjects;
    dynamicObjects.reserve(4u);
    for (std::size_t body = 0u; body < 4u; ++body) {
      auto* dynamicBody = createFreeBody(
          "colored_dynamic_" + std::to_string(body), true, skeletons);
      auto* joint
          = static_cast<dynamics::FreeJoint*>(dynamicBody->getParentJoint());
      joint->setLinearVelocity(Eigen::Vector3d(
          0.025 * static_cast<double>(body + 1u),
          -0.01 * static_cast<double>(body),
          -0.2 - 0.01 * static_cast<double>(body)));
      auto* dynamicShapeNode = dynamicBody->createShapeNodeWith<
          dynamics::CollisionAspect,
          dynamics::DynamicsAspect>(shape);
      dynamicObjects.push_back(
          std::make_unique<FakeCollisionObject>(&detector, dynamicShapeNode));
    }

    std::array<collision::Contact, 12> contacts;
    constexpr std::array<std::size_t, 4> kFixedManifoldBodyOrder{
        {0u, 2u, 1u, 3u}};
    for (std::size_t manifold = 0u; manifold < 4u; ++manifold) {
      const std::size_t body = kFixedManifoldBodyOrder[manifold];
      for (std::size_t point = 0u; point < 2u; ++point) {
        const std::size_t contact = 2u * manifold + point;
        contacts[contact]
            = createContact(dynamicObjects[body].get(), &fixedObject);
        contacts[contact].point = Eigen::Vector3d(
            point == 0u ? -0.2 : 0.2, 0.1 * static_cast<double>(body), 0.0);
      }
    }
    for (std::size_t pair = 0u; pair < 2u; ++pair) {
      const std::size_t firstBody = 2u * pair;
      const std::size_t secondBody = firstBody + 1u;
      for (std::size_t point = 0u; point < 2u; ++point) {
        const std::size_t contact = 8u + 2u * pair + point;
        contacts[contact] = createContact(
            dynamicObjects[firstBody].get(), dynamicObjects[secondBody].get());
        contacts[contact].point = Eigen::Vector3d(
            point == 0u ? -0.1 : 0.1, 0.2 * static_cast<double>(pair), 0.15);
        contacts[contact].normal = Eigen::Vector3d::UnitX();
      }
    }

    constraint::ConstrainedGroup group;
    std::vector<std::shared_ptr<constraint::ContactConstraint>> constraints;
    constraints.reserve(contacts.size());
    for (auto& contact : contacts) {
      auto contactConstraint
          = createContactConstraint<constraint::ContactConstraint>(contact);
      group.addConstraint(contactConstraint);
      constraints.push_back(std::move(contactConstraint));
    }

    constraint::ExactCoulombFbfConstraintSolverOptions options;
    options.fallbackToBoxedLcp = false;
    options.assembleDenseContactRowSnapshot = false;
    options.enableWarmStart = false;
    options.seedNormalImpulseFromDiagonal = false;
    options.enableProjectedGradientRetry = false;
    options.enableDenseResidualPolish = false;
    options.maxOuterIterations = 2;
    options.acceptOuterMaxIterations = true;
    options.tolerance = 0.0;
    options.initialStepSize = 0.1;
    options.capInitialStepSizeAtSafeBound = false;
    options.enableAdaptiveStepSize = false;
    options.innerMaxSweeps = 3;
    options.runFixedInnerSweeps = true;

    ExposedExactCoulombParallelEligibilitySolver solver;
    solver.setExactCoulombOptions(options);
    solver.setTimeStep(0.001);
    solver.setNumSimulationThreads(requestedThreads);
    solver.setExactCoulombColoredBlockGaussSeidelEnabled(coloredEnabled);
    solver.solveConstrainedGroup(group);

    SolveRecord record;
    record.status = solver.getLastExactCoulombStatus();
    record.fbfStatus = solver.getLastExactCoulombFbfStatus();
    record.residual = solver.getLastExactCoulombResidualDetails();
    record.stepSize = solver.getLastExactCoulombStepSize();
    record.iterations = solver.getLastExactCoulombIterations();
    record.contactRowOperatorUsed
        = solver.getLastExactCoulombContactRowOperatorUsed();
    record.coloredUsed
        = solver.getLastExactCoulombColoredBlockGaussSeidelUsed();
    record.coloredSolves
        = solver.getLastExactCoulombColoredBlockGaussSeidelSolves();
    record.coloredDispatches
        = solver.getLastExactCoulombColoredBlockGaussSeidelDispatches();
    record.coloredParticipants
        = solver.getLastExactCoulombColoredBlockGaussSeidelParticipants();
    record.coloredManifolds
        = solver.getLastExactCoulombColoredBlockGaussSeidelManifolds();
    record.coloredColors
        = solver.getLastExactCoulombColoredBlockGaussSeidelColors();
    record.coloredMaxManifoldsPerColor
        = solver
              .getLastExactCoulombColoredBlockGaussSeidelMaxManifoldsPerColor();
    record.coloredLogicalCpuIds
        = solver.getLastExactCoulombColoredBlockGaussSeidelLogicalCpuIds();
    record.coloredMaxPhaseLogicalCpuIds
        = solver
              .getLastExactCoulombColoredBlockGaussSeidelMaxPhaseLogicalCpuIds();
    record.reactions.reserve(contacts.size());
    for (const auto& contact : contacts)
      record.reactions.push_back(0.001 * contact.force);
    return record;
  };

  const SolveRecord legacy = runSolve(1u, false);
  const SolveRecord colored1 = runSolve(1u, true);
  const SolveRecord colored2 = runSolve(2u, true);
  const SolveRecord colored4 = runSolve(4u, true);
  const SolveRecord repeated4 = runSolve(4u, true);

  ASSERT_TRUE(legacy.contactRowOperatorUsed);
  EXPECT_FALSE(legacy.coloredUsed);
  EXPECT_EQ(legacy.coloredSolves, 0u);
  EXPECT_EQ(legacy.coloredDispatches, 0u);
  EXPECT_EQ(legacy.coloredParticipants, 0u);
  EXPECT_EQ(legacy.coloredManifolds, 0u);
  EXPECT_EQ(legacy.coloredColors, 0u);
  EXPECT_EQ(legacy.coloredMaxManifoldsPerColor, 0u);
  EXPECT_TRUE(legacy.coloredLogicalCpuIds.empty());
  EXPECT_TRUE(legacy.coloredMaxPhaseLogicalCpuIds.empty());
  for (const auto* record : {&colored1, &colored2, &colored4, &repeated4}) {
    ASSERT_TRUE(record->contactRowOperatorUsed);
    EXPECT_TRUE(record->coloredUsed);
    EXPECT_GT(record->coloredSolves, 0u);
    EXPECT_EQ(record->coloredManifolds, 6u);
    EXPECT_EQ(record->coloredColors, 3u);
    EXPECT_EQ(record->coloredMaxManifoldsPerColor, 2u);
  }
  EXPECT_EQ(colored1.coloredDispatches, 0u);
  EXPECT_EQ(colored2.coloredDispatches, 1u);
  EXPECT_EQ(colored4.coloredDispatches, 1u);
  EXPECT_EQ(repeated4.coloredDispatches, 1u);
  EXPECT_LT(colored2.coloredDispatches, colored2.coloredSolves);
  EXPECT_LT(colored4.coloredDispatches, colored4.coloredSolves);
  EXPECT_EQ(colored1.coloredParticipants, 1u);
  EXPECT_EQ(colored2.coloredParticipants, 2u);
  EXPECT_EQ(colored4.coloredParticipants, 4u);
  EXPECT_EQ(repeated4.coloredParticipants, 4u);
  EXPECT_TRUE(colored1.coloredLogicalCpuIds.empty());
  EXPECT_TRUE(colored1.coloredMaxPhaseLogicalCpuIds.empty());
  for (const auto* record : {&colored2, &colored4, &repeated4}) {
    EXPECT_TRUE(std::is_sorted(
        record->coloredLogicalCpuIds.begin(),
        record->coloredLogicalCpuIds.end()));
    EXPECT_TRUE(std::is_sorted(
        record->coloredMaxPhaseLogicalCpuIds.begin(),
        record->coloredMaxPhaseLogicalCpuIds.end()));
    EXPECT_LE(
        record->coloredMaxPhaseLogicalCpuIds.size(),
        record->coloredParticipants);
  }

  const auto expectBitIdentical = [](const SolveRecord& expected,
                                     const SolveRecord& actual) {
    EXPECT_EQ(expected.status, actual.status);
    EXPECT_EQ(expected.fbfStatus, actual.fbfStatus);
    EXPECT_DOUBLE_EQ(expected.residual.value, actual.residual.value);
    EXPECT_DOUBLE_EQ(
        expected.residual.primalFeasibility, actual.residual.primalFeasibility);
    EXPECT_DOUBLE_EQ(
        expected.residual.dualFeasibility, actual.residual.dualFeasibility);
    EXPECT_DOUBLE_EQ(
        expected.residual.complementarity, actual.residual.complementarity);
    EXPECT_DOUBLE_EQ(expected.stepSize, actual.stepSize);
    EXPECT_EQ(expected.iterations, actual.iterations);
    ASSERT_EQ(expected.reactions.size(), actual.reactions.size());
    for (std::size_t contact = 0u; contact < expected.reactions.size();
         ++contact) {
      EXPECT_TRUE((expected.reactions[contact].array()
                   == actual.reactions[contact].array())
                      .all())
          << "contact " << contact;
    }
  };

  // The shared-body manifolds force three colors. They are stored
  // contiguously in eventual color order, so the explicit one-thread colored
  // traversal also matches the disabled legacy order.
  expectBitIdentical(legacy, colored1);
  expectBitIdentical(colored1, colored2);
  expectBitIdentical(colored1, colored4);
  expectBitIdentical(colored4, repeated4);
}

//==============================================================================
TEST(ConstraintSolver, SharedFixedContactSupportWithMixedGroupForcesSerial)
{
  std::vector<dynamics::SkeletonPtr> skeletons;
  auto* fixedBody = createFreeBody("fixed", false, skeletons);

  auto shape = std::make_shared<dynamics::BoxShape>(Eigen::Vector3d::Ones());
  auto* fixedShapeNode = fixedBody->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(shape);

  FakeCollisionDetector detector;
  FakeCollisionObject fixedObject(&detector, fixedShapeNode);

  ExposedThreadedConstraintSolver solver;
  solver.setDeactivationActive(true);
  solver.setNumSimulationThreads(4);

  std::vector<std::unique_ptr<FakeCollisionObject>> dynamicObjects;
  std::vector<collision::Contact> contacts;
  dynamicObjects.reserve(129u);
  contacts.reserve(129u);

  for (std::size_t i = 0; i < 129u; ++i) {
    auto* dynamicBody
        = createFreeBody("dynamic_" + std::to_string(i), true, skeletons);
    auto* dynamicShapeNode = dynamicBody->createShapeNodeWith<
        dynamics::CollisionAspect,
        dynamics::DynamicsAspect>(shape);
    dynamicObjects.push_back(
        std::make_unique<FakeCollisionObject>(&detector, dynamicShapeNode));
    contacts.push_back(
        createContact(dynamicObjects.back().get(), &fixedObject));
    solver.addActiveConstraintForTest(
        createContactConstraint<constraint::ContactConstraint>(
            contacts.back()));
  }

  for (const auto& skeleton : skeletons)
    solver.addSkeletonForTest(skeleton);

  solver.buildGroupsForTest();

  auto* softBody = createSoftBody("soft", true, skeletons);
  auto* softShapeNode = softBody->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(shape);
  FakeCollisionObject softObject(&detector, softShapeNode);
  auto softContact = createContact(&softObject, &fixedObject);
  solver.addConstrainedGroup({
      std::make_shared<FakeConstraint>(100),
      createSoftContactConstraint(softContact),
  });
  solver.addSkeletonForTest(skeletons.back());

  solver.solveGroupsForTest();

  EXPECT_EQ(130, solver.getNumSolvedGroups());
  EXPECT_EQ(1, solver.getMaxConcurrentSolves());
}

//==============================================================================
TEST(ConstraintSolver, ManualConstraintsForceSerialDeactivationGroupSolves)
{
  ExposedThreadedConstraintSolver solver;
  solver.setDeactivationActive(true);
  solver.setNumSimulationThreads(4);
  solver.addConstraint(std::make_shared<FakeConstraint>(1));
  solver.addFakeConstrainedGroups(130, 100);

  solver.solveGroupsForTest();

  EXPECT_EQ(130, solver.getNumSolvedGroups());
  EXPECT_EQ(1, solver.getMaxConcurrentSolves());
}

//==============================================================================
TEST(ConstraintSolver, ParallelGroupSolveRequiresExactBuiltInSolvers)
{
  auto solvesInParallel = [](ExposedThreadedConstraintSolver& solver) {
    solver.addFakeConstrainedGroups(130, 100);
    return solvesGroupsInParallel(solver);
  };

  ExposedThreadedConstraintSolver defaultSolver;
  EXPECT_TRUE(solvesInParallel(defaultSolver));

  ExposedThreadedConstraintSolver noSecondarySolver(
      std::make_shared<constraint::DantzigBoxedLcpSolver>(), nullptr);
  EXPECT_TRUE(solvesInParallel(noSecondarySolver));

  ExposedExactCoulombParallelEligibilitySolver exactCoulombSolver;
  exactCoulombSolver.setNumSimulationThreads(4);
  exactCoulombSolver.addFakeConstrainedGroups(130, 100);
  EXPECT_FALSE(exactCoulombSolver.canSolveConstrainedGroupsInParallelForTest());

  ExposedThreadedConstraintSolver pgsPrimarySolver(
      std::make_shared<constraint::PgsBoxedLcpSolver>(), nullptr);
  EXPECT_TRUE(solvesInParallel(pgsPrimarySolver));

  auto randomizedPrimaryPgs = std::make_shared<constraint::PgsBoxedLcpSolver>();
  auto primaryOption = randomizedPrimaryPgs->getOption();
  primaryOption.mRandomizeConstraintOrder = true;
  randomizedPrimaryPgs->setOption(primaryOption);

  ExposedThreadedConstraintSolver randomizedPrimarySolver(
      randomizedPrimaryPgs, nullptr);
  EXPECT_FALSE(solvesInParallel(randomizedPrimarySolver));

  ExposedThreadedConstraintSolver derivedPrimarySolver(
      std::make_shared<DerivedDantzigBoxedLcpSolver>(),
      std::make_shared<constraint::PgsBoxedLcpSolver>());
  EXPECT_FALSE(solvesInParallel(derivedPrimarySolver));

  ExposedThreadedConstraintSolver derivedSecondarySolver(
      std::make_shared<constraint::DantzigBoxedLcpSolver>(),
      std::make_shared<DerivedPgsBoxedLcpSolver>());
  EXPECT_FALSE(solvesInParallel(derivedSecondarySolver));

  auto randomizedPgs = std::make_shared<constraint::PgsBoxedLcpSolver>();
  auto option = randomizedPgs->getOption();
  option.mRandomizeConstraintOrder = true;
  randomizedPgs->setOption(option);

  ExposedThreadedConstraintSolver randomizedSecondarySolver(
      std::make_shared<constraint::DantzigBoxedLcpSolver>(), randomizedPgs);
  EXPECT_FALSE(solvesInParallel(randomizedSecondarySolver));
}

//==============================================================================
TEST(ConstraintSolver, CustomContactConstraintsForceSerialParallelGroupSolves)
{
  std::vector<dynamics::SkeletonPtr> skeletons;
  auto* fixedBody1 = createFreeBody("fixed1", false, skeletons);
  auto* fixedBody2 = createFreeBody("fixed2", false, skeletons);
  auto* dynamicBody1 = createFreeBody("dynamic1", true, skeletons);
  auto* dynamicBody2 = createFreeBody("dynamic2", true, skeletons);

  auto shape = std::make_shared<dynamics::BoxShape>(Eigen::Vector3d::Ones());
  auto* fixedShapeNode1 = fixedBody1->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(shape);
  auto* fixedShapeNode2 = fixedBody2->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(shape);
  auto* dynamicShapeNode1 = dynamicBody1->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(shape);
  auto* dynamicShapeNode2 = dynamicBody2->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(shape);

  FakeCollisionDetector detector;
  FakeCollisionObject fixedObject1(&detector, fixedShapeNode1);
  FakeCollisionObject fixedObject2(&detector, fixedShapeNode2);
  FakeCollisionObject dynamicObject1(&detector, dynamicShapeNode1);
  FakeCollisionObject dynamicObject2(&detector, dynamicShapeNode2);

  auto contact1 = createContact(&dynamicObject1, &fixedObject1);
  auto contact2 = createContact(&dynamicObject2, &fixedObject2);

  ExposedThreadedConstraintSolver solver;
  solver.setNumSimulationThreads(4);
  solver.addConstrainedGroup({
      std::make_shared<FakeConstraint>(100),
      createContactConstraint<CustomContactConstraint>(contact1),
  });
  solver.addConstrainedGroup({
      std::make_shared<FakeConstraint>(100),
      createContactConstraint<CustomContactConstraint>(contact2),
  });
  addPaddingGroups(solver);

  solver.solveGroupsForTest();

  EXPECT_EQ(130, solver.getNumSolvedGroups());
  EXPECT_EQ(1, solver.getMaxConcurrentSolves());
}

//==============================================================================
TEST(ConstraintSolver, DistinctNonReactiveBodiesCanSolveGroupsInParallel)
{
  std::vector<dynamics::SkeletonPtr> skeletons;
  auto* fixedBody1 = createFreeBody("fixed1", false, skeletons);
  auto* fixedBody2 = createFreeBody("fixed2", false, skeletons);
  auto* dynamicBody1 = createFreeBody("dynamic1", true, skeletons);
  auto* dynamicBody2 = createFreeBody("dynamic2", true, skeletons);

  ExposedThreadedConstraintSolver solver;
  solver.setNumSimulationThreads(4);
  solver.addConstrainedGroup({
      std::make_shared<FakeConstraint>(100),
      std::make_shared<constraint::BallJointConstraint>(
          dynamicBody1, fixedBody1, Eigen::Vector3d::Zero()),
  });
  solver.addConstrainedGroup({
      std::make_shared<FakeConstraint>(100),
      std::make_shared<constraint::BallJointConstraint>(
          dynamicBody2, fixedBody2, Eigen::Vector3d::Zero()),
  });
  addPaddingGroups(solver);

  solver.solveGroupsForTest();

  EXPECT_EQ(130, solver.getNumSolvedGroups());
  EXPECT_GT(solver.getMaxConcurrentSolves(), 1);
}

//==============================================================================
TEST(ConstraintSolver, SharedNonReactiveBodiesForceSerialParallelGroupSolves)
{
  std::vector<dynamics::SkeletonPtr> skeletons;
  auto* fixedBody = createFreeBody("fixed", false, skeletons);
  auto* dynamicBody1 = createFreeBody("dynamic1", true, skeletons);
  auto* dynamicBody2 = createFreeBody("dynamic2", true, skeletons);

  ExposedThreadedConstraintSolver solver;
  solver.setNumSimulationThreads(4);
  solver.addConstrainedGroup({
      std::make_shared<FakeConstraint>(100),
      std::make_shared<constraint::BallJointConstraint>(
          dynamicBody1, fixedBody, Eigen::Vector3d::Zero()),
  });
  solver.addConstrainedGroup({
      std::make_shared<FakeConstraint>(100),
      std::make_shared<constraint::BallJointConstraint>(
          dynamicBody2, fixedBody, Eigen::Vector3d::Zero()),
  });
  addPaddingGroups(solver);

  solver.solveGroupsForTest();

  EXPECT_EQ(130, solver.getNumSolvedGroups());
  EXPECT_EQ(1, solver.getMaxConcurrentSolves());
}

//==============================================================================
TEST(ConstraintSolver, SharedNonReactiveSkeletonForcesSerialParallelGroupSolves)
{
  std::vector<dynamics::SkeletonPtr> skeletons;
  const auto mixedBodies = createMixedReactiveSkeleton("mixed", skeletons);
  auto* dynamicBody1 = createFreeBody("dynamic1", true, skeletons);
  auto* dynamicBody2 = createFreeBody("dynamic2", true, skeletons);

  ASSERT_FALSE(mixedBodies.first->isReactive());
  ASSERT_TRUE(mixedBodies.second->isReactive());

  ExposedThreadedConstraintSolver solver;
  solver.setNumSimulationThreads(4);
  solver.addConstrainedGroup({
      std::make_shared<FakeConstraint>(100),
      std::make_shared<constraint::BallJointConstraint>(
          dynamicBody1, mixedBodies.second, Eigen::Vector3d::Zero()),
  });
  solver.addConstrainedGroup({
      std::make_shared<FakeConstraint>(100),
      std::make_shared<constraint::BallJointConstraint>(
          dynamicBody2, mixedBodies.first, Eigen::Vector3d::Zero()),
  });
  addPaddingGroups(solver);

  solver.solveGroupsForTest();

  EXPECT_EQ(130, solver.getNumSolvedGroups());
  EXPECT_EQ(1, solver.getMaxConcurrentSolves());
}

//==============================================================================
TEST(ConstraintSolver, SharedNonReactiveSoftContactsForceSerialGroupSolves)
{
  std::vector<dynamics::SkeletonPtr> skeletons;
  auto* softBody = createSoftBody("soft", false, skeletons);
  auto* dynamicBody1 = createFreeBody("dynamic1", true, skeletons);
  auto* dynamicBody2 = createFreeBody("dynamic2", true, skeletons);

  ASSERT_FALSE(softBody->isReactive());
  ASSERT_TRUE(dynamicBody1->isReactive());
  ASSERT_TRUE(dynamicBody2->isReactive());

  auto shape = std::make_shared<dynamics::BoxShape>(Eigen::Vector3d::Ones());
  auto* softShapeNode = softBody->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(shape);
  auto* dynamicShapeNode1 = dynamicBody1->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(shape);
  auto* dynamicShapeNode2 = dynamicBody2->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(shape);

  FakeCollisionDetector detector;
  FakeCollisionObject softObject(&detector, softShapeNode);
  FakeCollisionObject dynamicObject1(&detector, dynamicShapeNode1);
  FakeCollisionObject dynamicObject2(&detector, dynamicShapeNode2);

  auto contact1 = createContact(&dynamicObject1, &softObject);
  auto contact2 = createContact(&dynamicObject2, &softObject);

  ExposedThreadedConstraintSolver solver;
  solver.setNumSimulationThreads(4);
  solver.addConstrainedGroup({
      std::make_shared<FakeConstraint>(100),
      createSoftContactConstraint(contact1),
  });
  solver.addConstrainedGroup({
      std::make_shared<FakeConstraint>(100),
      createSoftContactConstraint(contact2),
  });
  addPaddingGroups(solver);

  solver.solveGroupsForTest();

  EXPECT_EQ(130, solver.getNumSolvedGroups());
  EXPECT_EQ(1, solver.getMaxConcurrentSolves());
}

//==============================================================================
TEST(ConstraintSolver, FixedSkeletonJointConstraintsForceSerialGroupSolves)
{
  std::vector<dynamics::SkeletonPtr> skeletons;
  auto fixedSkeleton = dynamics::Skeleton::create("fixed");
  auto fixedPair
      = fixedSkeleton->createJointAndBodyNodePair<dynamics::FreeJoint>();
  fixedSkeleton->setMobile(false);
  skeletons.push_back(fixedSkeleton);

  auto* fixedJoint = fixedPair.first;
  auto* fixedBody = fixedPair.second;
  fixedJoint->setCoulombFriction(0, 1.0);
  fixedJoint->setVelocity(0, 1.0);

  auto jointFriction
      = std::make_shared<constraint::JointCoulombFrictionConstraint>(
          fixedJoint);
  constraint::ConstraintBase& jointFrictionBase = *jointFriction;
  jointFrictionBase.update();
  ASSERT_TRUE(jointFrictionBase.isActive());

  auto* dynamicBody = createFreeBody("dynamic", true, skeletons);

  ExposedThreadedConstraintSolver solver;
  solver.setNumSimulationThreads(4);
  solver.addConstrainedGroup({
      std::make_shared<FakeConstraint>(100),
      jointFriction,
  });
  solver.addConstrainedGroup({
      std::make_shared<FakeConstraint>(100),
      std::make_shared<constraint::BallJointConstraint>(
          dynamicBody, fixedBody, Eigen::Vector3d::Zero()),
  });
  addPaddingGroups(solver);

  solver.solveGroupsForTest();

  EXPECT_EQ(130, solver.getNumSolvedGroups());
  EXPECT_EQ(1, solver.getMaxConcurrentSolves());
}

//==============================================================================
TEST(ConstraintSolver, DefaultConstactSurfaceHandler)
{
  auto world = createWorld();
  auto solver = world->getConstraintSolver();
  ASSERT_NE(nullptr, solver->getLastContactSurfaceHandler());
}

//==============================================================================
TEST(ConstraintSolver, AutomaticSleepingAliasIsSourceCompatible)
{
  auto world = createWorld();
  auto* solver = world->getConstraintSolver();
  solver->setAutomaticSleepingEnabled(false);
  solver->setAutomaticSleepingEnabled(true);
}

//==============================================================================
TEST(ConstraintSolver, CustomConstactSurfaceHandler)
{
  class CustomHandler : public constraint::ContactSurfaceHandler
  {
  public:
    constraint::ContactSurfaceParams createParams(
        const collision::Contact& contact,
        const size_t numContactsOnCollisionObject) const override
    {
      auto params = ContactSurfaceHandler::createParams(
          contact, numContactsOnCollisionObject);
      params.mFirstFrictionalDirection = Eigen::Vector3d::UnitY();
      params.mContactSurfaceMotionVelocity = Eigen::Vector3d::UnitY();
      return params;
    }
  };

  auto world = createWorld();

  auto solver = world->getConstraintSolver();
  auto defaultHandler = solver->getLastContactSurfaceHandler();
  EXPECT_EQ(nullptr, defaultHandler->getParent());

  auto customHandler = std::make_shared<CustomHandler>();
  solver->addContactSurfaceHandler(customHandler);

  ASSERT_NE(nullptr, solver->getLastContactSurfaceHandler());
  EXPECT_EQ(nullptr, defaultHandler->getParent());
  EXPECT_EQ(defaultHandler, customHandler->getParent());

  // try to remove nonexisting handler
  EXPECT_FALSE(
      solver->removeContactSurfaceHandler(std::make_shared<CustomHandler>()));

  EXPECT_TRUE(solver->removeContactSurfaceHandler(defaultHandler));
  EXPECT_EQ(nullptr, customHandler->getParent());
  EXPECT_EQ(customHandler, solver->getLastContactSurfaceHandler());

  // removing last handler should not be done, but we test it anyways
  // a printed error message is expected
  EXPECT_TRUE(solver->removeContactSurfaceHandler(customHandler));
  EXPECT_EQ(nullptr, customHandler->getParent());
  EXPECT_EQ(nullptr, solver->getLastContactSurfaceHandler());

  solver->addContactSurfaceHandler(defaultHandler);
  ASSERT_NE(nullptr, solver->getLastContactSurfaceHandler());
  EXPECT_EQ(defaultHandler, solver->getLastContactSurfaceHandler());

  solver->addContactSurfaceHandler(customHandler);
  ASSERT_NE(nullptr, solver->getLastContactSurfaceHandler());
  EXPECT_EQ(customHandler, solver->getLastContactSurfaceHandler());
  EXPECT_EQ(nullptr, defaultHandler->getParent());
  EXPECT_EQ(defaultHandler, customHandler->getParent());

  auto customHandler2 = std::make_shared<CustomHandler>();
  auto customHandler3 = std::make_shared<CustomHandler>();
  solver->addContactSurfaceHandler(customHandler2);
  solver->addContactSurfaceHandler(customHandler3);
  ASSERT_NE(nullptr, solver->getLastContactSurfaceHandler());
  EXPECT_EQ(customHandler3, solver->getLastContactSurfaceHandler());
  EXPECT_EQ(nullptr, defaultHandler->getParent());
  EXPECT_EQ(defaultHandler, customHandler->getParent());
  EXPECT_EQ(customHandler, customHandler2->getParent());
  EXPECT_EQ(customHandler2, customHandler3->getParent());

  EXPECT_TRUE(solver->removeContactSurfaceHandler(customHandler));
  ASSERT_NE(nullptr, solver->getLastContactSurfaceHandler());
  EXPECT_EQ(customHandler3, solver->getLastContactSurfaceHandler());
  EXPECT_EQ(nullptr, defaultHandler->getParent());
  EXPECT_EQ(defaultHandler, customHandler->getParent());
  EXPECT_EQ(defaultHandler, customHandler2->getParent());
  EXPECT_EQ(customHandler2, customHandler3->getParent());

  EXPECT_TRUE(solver->removeContactSurfaceHandler(customHandler3));
  ASSERT_NE(nullptr, solver->getLastContactSurfaceHandler());
  EXPECT_EQ(customHandler2, solver->getLastContactSurfaceHandler());
  EXPECT_EQ(nullptr, defaultHandler->getParent());
  EXPECT_EQ(defaultHandler, customHandler->getParent());
  EXPECT_EQ(defaultHandler, customHandler2->getParent());
  EXPECT_EQ(customHandler2, customHandler3->getParent());

  // after we break the chain at handler 2, default handler is no longer
  // reachable
  customHandler2->setParent(nullptr);
  EXPECT_FALSE(solver->removeContactSurfaceHandler(defaultHandler));
}

//==============================================================================
TEST(ConstraintSolver, ConstactSurfaceHandlerIsCalled)
{
  class ValueHandler : public constraint::ContactSurfaceHandler
  {
  public:
    ValueHandler(int value) : mValue(value)
    {
      // Do nothing
    }

    constraint::ContactSurfaceParams createParams(
        const collision::Contact& contact,
        const size_t numContactsOnCollisionObject) const override
    {
      auto params = ContactSurfaceHandler::createParams(
          contact, numContactsOnCollisionObject);
      mCalled = true;
      params.mPrimaryFrictionCoeff = mValue;

      return params;
    }

    mutable bool mCalled{false};
    int mValue{0};
  };

  auto world = createWorld();

  auto solver = world->getConstraintSolver();
  auto defaultHandler = solver->getLastContactSurfaceHandler();
  EXPECT_EQ(nullptr, defaultHandler->getParent());

  auto customHandler = std::make_shared<ValueHandler>(1);
  solver->addContactSurfaceHandler(customHandler);
  solver->removeContactSurfaceHandler(defaultHandler);

  customHandler->mCalled = false;
  auto params = solver->getLastContactSurfaceHandler()->createParams({}, 0);
  EXPECT_TRUE(customHandler->mCalled);
  EXPECT_EQ(1, params.mPrimaryFrictionCoeff);

  auto customHandler2 = std::make_shared<ValueHandler>(2);
  solver->addContactSurfaceHandler(customHandler2);

  customHandler->mCalled = customHandler2->mCalled = false;
  params = solver->getLastContactSurfaceHandler()->createParams({}, 0);
  EXPECT_TRUE(customHandler->mCalled);
  EXPECT_TRUE(customHandler2->mCalled);
  EXPECT_EQ(2, params.mPrimaryFrictionCoeff);

  // Try once more adding the same handler instance; this should be ignored.
  // If it were added, the createParams() call could get into an infinite loop
  // calling the last handler as its parent, so rather check for it.
  solver->addContactSurfaceHandler(customHandler2);

  customHandler->mCalled = customHandler2->mCalled = false;
  params = solver->getLastContactSurfaceHandler()->createParams({}, 0);
  EXPECT_TRUE(customHandler->mCalled);
  EXPECT_TRUE(customHandler2->mCalled);
  EXPECT_EQ(2, params.mPrimaryFrictionCoeff);
}

//==============================================================================
TEST(ConstraintSolver, ConstactSurfaceHandlerIgnoreParent)
{
  class IgnoreParentHandler : public constraint::ContactSurfaceHandler
  {
  public:
    IgnoreParentHandler(int value) : mValue(value)
    {
      // Do nothing
    }

    constraint::ContactSurfaceParams createParams(
        const collision::Contact& /*contact*/,
        const size_t /*numContactsOnCollisionObject*/) const override
    {
      auto params = constraint::ContactSurfaceParams{};
      mCalled = true;
      params.mPrimaryFrictionCoeff = mValue;

      return params;
    }

    mutable bool mCalled{false};
    int mValue{0};
  };

  auto world = createWorld();

  auto solver = world->getConstraintSolver();
  auto defaultHandler = solver->getLastContactSurfaceHandler();
  EXPECT_EQ(nullptr, defaultHandler->getParent());

  auto customHandler = std::make_shared<IgnoreParentHandler>(1);
  solver->addContactSurfaceHandler(customHandler);
  solver->removeContactSurfaceHandler(defaultHandler);

  customHandler->mCalled = false;
  auto params = solver->getLastContactSurfaceHandler()->createParams({}, 0);
  EXPECT_TRUE(customHandler->mCalled);
  EXPECT_EQ(1, params.mPrimaryFrictionCoeff);

  auto customHandler2 = std::make_shared<IgnoreParentHandler>(2);
  solver->addContactSurfaceHandler(customHandler2);

  customHandler->mCalled = customHandler2->mCalled = false;
  params = solver->getLastContactSurfaceHandler()->createParams({}, 0);
  EXPECT_FALSE(customHandler->mCalled);
  EXPECT_TRUE(customHandler2->mCalled);
  EXPECT_EQ(2, params.mPrimaryFrictionCoeff);
}

//==============================================================================
// Split impulse must be opt-in: disabled by default so the existing Baumgarte
// (velocity-phase) penetration correction is preserved unchanged.
TEST(ConstraintSolver, SplitImpulseDisabledByDefault)
{
  constraint::BoxedLcpConstraintSolver solver;
  EXPECT_FALSE(solver.isSplitImpulseEnabled());
}

//==============================================================================
TEST(ConstraintSolver, SplitImpulseEnabledRoundTrips)
{
  constraint::BoxedLcpConstraintSolver solver;
  solver.setSplitImpulseEnabled(true);
  EXPECT_TRUE(solver.isSplitImpulseEnabled());
  solver.setSplitImpulseEnabled(false);
  EXPECT_FALSE(solver.isSplitImpulseEnabled());
}

//==============================================================================
// setFromOtherConstraintSolver must copy the split impulse flag so cloned
// worlds preserve the configured contact-solve behavior.
TEST(ConstraintSolver, SplitImpulseFlagIsCopiedFromOtherSolver)
{
  constraint::BoxedLcpConstraintSolver source;
  source.setSplitImpulseEnabled(true);

  constraint::BoxedLcpConstraintSolver target;
  ASSERT_FALSE(target.isSplitImpulseEnabled());
  target.setFromOtherConstraintSolver(source);
  EXPECT_TRUE(target.isSplitImpulseEnabled());

  constraint::BoxedLcpConstraintSolver sourceOff;
  sourceOff.setSplitImpulseEnabled(false);
  target.setFromOtherConstraintSolver(sourceOff);
  EXPECT_FALSE(target.isSplitImpulseEnabled());
}

//==============================================================================
TEST(ConstraintSolver, ExactColoredOptionsAreDefaultOffAndCopiedExplicitly)
{
  constraint::ExactCoulombFbfConstraintSolver source;
  EXPECT_FALSE(source.getExactCoulombColoredBlockGaussSeidelEnabled());
  EXPECT_FALSE(
      source
          .getExactCoulombColoredBlockGaussSeidelParticipantAffinityEnabled());

  auto options = source.getExactCoulombOptions();
  options.maxOuterIterations = 17;
  source.setExactCoulombOptions(options);
  source.setExactCoulombColoredBlockGaussSeidelEnabled(true);
  source.setExactCoulombColoredBlockGaussSeidelParticipantAffinityEnabled(true);

  constraint::ExactCoulombFbfConstraintSolver target;
  target.setFromOtherConstraintSolver(source);
  EXPECT_EQ(target.getExactCoulombOptions().maxOuterIterations, 17);
  EXPECT_TRUE(target.getExactCoulombColoredBlockGaussSeidelEnabled());
  EXPECT_TRUE(
      target
          .getExactCoulombColoredBlockGaussSeidelParticipantAffinityEnabled());

  constraint::BoxedLcpConstraintSolver ordinary;
  target.setFromOtherConstraintSolver(ordinary);
  EXPECT_FALSE(target.getExactCoulombColoredBlockGaussSeidelEnabled());
  EXPECT_FALSE(
      target
          .getExactCoulombColoredBlockGaussSeidelParticipantAffinityEnabled());
}

//==============================================================================
TEST(ConstraintSolver, MatrixFreeContactSolverOptionsDisabledByDefault)
{
  constraint::BoxedLcpConstraintSolver solver;
  const auto& options = solver.getMatrixFreeContactSolverOptions();

  EXPECT_FALSE(options.mEnabled);
  EXPECT_EQ(193u, options.mMinRows);
  EXPECT_EQ(30, options.mMaxIterations);
  EXPECT_DOUBLE_EQ(0.9, options.mSor);
  EXPECT_DOUBLE_EQ(1e-6, options.mDeltaTolerance);
  EXPECT_DOUBLE_EQ(1e-3, options.mRelativeDeltaTolerance);
  EXPECT_DOUBLE_EQ(1e-9, options.mEpsilonForDivision);
}

//==============================================================================
TEST(ConstraintSolver, MatrixFreeContactSolverOptionsSanitizeAndRoundTrip)
{
  constraint::BoxedLcpConstraintSolver solver;
  auto options = solver.getMatrixFreeContactSolverOptions();
  options.mEnabled = true;
  options.mMinRows = 7u;
  options.mMaxIterations = -4;
  options.mSor = std::numeric_limits<double>::quiet_NaN();
  options.mDeltaTolerance = -1.0;
  options.mRelativeDeltaTolerance = -2.0;
  options.mEpsilonForDivision = 0.0;

  solver.setMatrixFreeContactSolverOptions(options);
  const auto& stored = solver.getMatrixFreeContactSolverOptions();

  EXPECT_TRUE(stored.mEnabled);
  EXPECT_EQ(7u, stored.mMinRows);
  EXPECT_EQ(1, stored.mMaxIterations);
  EXPECT_DOUBLE_EQ(1.0, stored.mSor);
  EXPECT_DOUBLE_EQ(0.0, stored.mDeltaTolerance);
  EXPECT_DOUBLE_EQ(0.0, stored.mRelativeDeltaTolerance);
  EXPECT_DOUBLE_EQ(1e-9, stored.mEpsilonForDivision);
}

//==============================================================================
TEST(ConstraintSolver, MatrixFreeContactSolverOptionsCopiedFromOtherSolver)
{
  constraint::BoxedLcpConstraintSolver source;
  auto options = source.getMatrixFreeContactSolverOptions();
  options.mEnabled = true;
  options.mMinRows = 11u;
  options.mMaxIterations = 12;
  options.mSor = 1.1;
  options.mDeltaTolerance = 1e-5;
  options.mRelativeDeltaTolerance = 2e-3;
  options.mEpsilonForDivision = 1e-8;
  source.setMatrixFreeContactSolverOptions(options);

  constraint::BoxedLcpConstraintSolver target;
  ASSERT_FALSE(target.getMatrixFreeContactSolverOptions().mEnabled);
  target.setFromOtherConstraintSolver(source);

  const auto& copied = target.getMatrixFreeContactSolverOptions();
  EXPECT_TRUE(copied.mEnabled);
  EXPECT_EQ(options.mMinRows, copied.mMinRows);
  EXPECT_EQ(options.mMaxIterations, copied.mMaxIterations);
  EXPECT_DOUBLE_EQ(options.mSor, copied.mSor);
  EXPECT_DOUBLE_EQ(options.mDeltaTolerance, copied.mDeltaTolerance);
  EXPECT_DOUBLE_EQ(
      options.mRelativeDeltaTolerance, copied.mRelativeDeltaTolerance);
  EXPECT_DOUBLE_EQ(options.mEpsilonForDivision, copied.mEpsilonForDivision);
}

//==============================================================================
TEST(ConstraintSolver, MatrixFreeContactSolverOptInKeepsContactWorldFinite)
{
  constexpr std::size_t kNumBoxes = 48u;
  auto world = createManySingleFreeBodyContactWorld(kNumBoxes, 1u);
  auto* boxedSolver = dynamic_cast<constraint::BoxedLcpConstraintSolver*>(
      world->getConstraintSolver());
  ASSERT_NE(nullptr, boxedSolver);

  auto options = boxedSolver->getMatrixFreeContactSolverOptions();
  options.mEnabled = true;
  options.mMinRows = 1u;
  options.mMaxIterations = 15;
  boxedSolver->setMatrixFreeContactSolverOptions(options);

  const bool previousRecording = common::profile::setProfileRecordingEnabled(
      common::profile::isTextProfilingEnabled());
  if (common::profile::isTextProfilingEnabled())
    common::profile::resetProfile();

  for (std::size_t step = 0u; step < 10u; ++step)
    world->step();

  const auto profileSummary = common::profile::getProfileSummaryText();
  common::profile::setProfileRecordingEnabled(previousRecording);
  common::profile::resetProfile();

  const auto& contacts = world->getConstraintSolver()->getLastCollisionResult();
  EXPECT_GE(contacts.getNumContacts(), kNumBoxes * 3u);

  for (std::size_t i = 0u; i < kNumBoxes; ++i) {
    const auto skeleton = world->getSkeleton("box_" + std::to_string(i));
    ASSERT_NE(nullptr, skeleton) << i;
    EXPECT_TRUE(skeleton->getPositions().allFinite()) << i;
    EXPECT_TRUE(skeleton->getVelocities().allFinite()) << i;
  }

  if (common::profile::isTextProfilingEnabled()) {
    EXPECT_NE(
        std::string::npos,
        profileSummary.find(
            "BoxedLcpConstraintSolver::matrixFreeContactSolve"));
    EXPECT_NE(
        std::string::npos,
        profileSummary.find(
            "BoxedLcpConstraintSolver::matrixFreeContactIterations"));
  }
}

//==============================================================================
TEST(ConstraintSolver, MatrixFreeContactSolverOptInSupportsTwoReactiveBodies)
{
  auto world = createWorld();
  world->setTimeStep(0.001);

  simulation::DeactivationOptions deactivation;
  deactivation.mEnabled = false;
  world->setDeactivationOptions(deactivation);

  auto* solver = world->getConstraintSolver();
  solver->setCollisionDetector(collision::DARTCollisionDetector::create());
  solver->getCollisionOption().maxNumContacts = 16u;
  solver->getCollisionOption().maxNumContactsPerPair = 4u;

  auto* boxedSolver
      = dynamic_cast<constraint::BoxedLcpConstraintSolver*>(solver);
  ASSERT_NE(nullptr, boxedSolver);

  auto options = boxedSolver->getMatrixFreeContactSolverOptions();
  options.mEnabled = true;
  options.mMinRows = 1u;
  options.mMaxIterations = 20;
  boxedSolver->setMatrixFreeContactSolverOptions(options);

  world->addSkeleton(createSolverTestBox(
      "box_a", Eigen::Vector3d::Ones(), Eigen::Vector3d::Zero(), true));
  world->addSkeleton(createSolverTestBox(
      "box_b", Eigen::Vector3d::Ones(), Eigen::Vector3d(0.0, 0.0, 0.9), true));

  const bool previousRecording = common::profile::setProfileRecordingEnabled(
      common::profile::isTextProfilingEnabled());
  if (common::profile::isTextProfilingEnabled())
    common::profile::resetProfile();

  for (std::size_t step = 0u; step < 5u; ++step)
    world->step();

  const auto profileSummary = common::profile::getProfileSummaryText();
  common::profile::setProfileRecordingEnabled(previousRecording);
  common::profile::resetProfile();

  const auto& contacts = solver->getLastCollisionResult();
  EXPECT_GT(contacts.getNumContacts(), 0u);

  for (const auto& name : {"box_a", "box_b"}) {
    const auto skeleton = world->getSkeleton(name);
    ASSERT_NE(nullptr, skeleton) << name;
    EXPECT_TRUE(skeleton->getPositions().allFinite()) << name;
    EXPECT_TRUE(skeleton->getVelocities().allFinite()) << name;
  }

  if (common::profile::isTextProfilingEnabled()) {
    EXPECT_NE(
        std::string::npos,
        profileSummary.find(
            "BoxedLcpConstraintSolver::matrixFreeContactSolve"));
  }
}
