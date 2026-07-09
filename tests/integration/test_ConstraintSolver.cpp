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
#include "dart/constraint/JointCoulombFrictionConstraint.hpp"
#include "dart/constraint/PgsBoxedLcpSolver.hpp"
#include "dart/constraint/SoftContactConstraint.hpp"
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

#include <atomic>
#include <chrono>
#include <limits>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <typeinfo>
#include <vector>

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

class ExposedBoxedLcpConstraintSolver final
  : public constraint::BoxedLcpConstraintSolver
{
public:
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
