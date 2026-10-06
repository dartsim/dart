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
#include "dart/collision/CollisionGroup.hpp"
#include "dart/collision/CollisionObject.hpp"
#include "dart/collision/Contact.hpp"
#include "dart/collision/dart/DARTCollisionDetector.hpp"
#include "dart/collision/dart/DARTCollisionObject.hpp"
#include "dart/collision/dart/PersistentManifoldCache.hpp"
#include "dart/collision/fcl/FCLCollisionDetector.hpp"
#include "dart/common/Profile.hpp"
#include "dart/config.hpp"
#include "dart/constraint/BallJointConstraint.hpp"
#include "dart/constraint/BoxedLcpConstraintSolver.hpp"
#include "dart/constraint/ConstrainedGroup.hpp"
#include "dart/constraint/ConstraintSolver.hpp"
#include "dart/constraint/ContactConstraint.hpp"
#include "dart/constraint/ContactSurface.hpp"
#include "dart/constraint/DantzigBoxedLcpSolver.hpp"
#include "dart/constraint/DynamicJointConstraint.hpp"
#include "dart/constraint/JointConstraint.hpp"
#include "dart/constraint/JointCoulombFrictionConstraint.hpp"
#include "dart/constraint/JointLimitConstraint.hpp"
#include "dart/constraint/PgsBoxedLcpSolver.hpp"
#include "dart/constraint/ServoMotorConstraint.hpp"
#include "dart/constraint/SoftContactConstraint.hpp"
#include "dart/dynamics/BoxShape.hpp"
#include "dart/dynamics/CylinderShape.hpp"
#include "dart/dynamics/FreeJoint.hpp"
#include "dart/dynamics/Joint.hpp"
#include "dart/dynamics/PlaneShape.hpp"
#include "dart/dynamics/ShapeFrame.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/dynamics/SoftBodyNode.hpp"
#include "dart/dynamics/SphereShape.hpp"
#include "dart/simulation/DeactivationOptions.hpp"
#include "dart/simulation/World.hpp"

#if HAVE_BULLET
  #include "dart/collision/bullet/BulletCollisionDetector.hpp"
#endif
#if HAVE_ODE
  #include "dart/collision/ode/OdeCollisionDetector.hpp"
#endif

#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <new>
#include <set>
#include <string>
#include <string_view>
#include <thread>
#include <tuple>
#include <type_traits>
#include <typeinfo>
#include <vector>

using namespace dart;

namespace {

DART_SUPPRESS_DEPRECATED_BEGIN
class ConstructorProbeConstraintSolver final
  : public constraint::ConstraintSolver
{
public:
  ConstructorProbeConstraintSolver() = default;

  explicit ConstructorProbeConstraintSolver(double timeStep)
    : ConstraintSolver(timeStep)
  {
    // Do nothing
  }

private:
  void solveConstrainedGroup(constraint::ConstrainedGroup&) override
  {
    // Do nothing
  }
};
DART_SUPPRESS_DEPRECATED_END

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

class ExposedDARTCollisionObject final : public collision::DARTCollisionObject
{
public:
  ExposedDARTCollisionObject(
      collision::CollisionDetector* detector,
      const dynamics::ShapeFrame* shapeFrame)
    : DARTCollisionObject(detector, shapeFrame)
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

class RejectingContactSurfaceHandler final
  : public constraint::ContactSurfaceHandler
{
public:
  constraint::ContactConstraintPtr createConstraint(
      collision::Contact&, const size_t, const double) const override
  {
    ++mNumCreateConstraintCalls;
    return nullptr;
  }

  mutable std::size_t mNumCreateConstraintCalls{0u};
};

// Mirrors gz-physics' contact-properties handler: it keeps the previous handler
// as its parent and runs a user callback each time it creates the surface
// parameters of a contact.
class CountingContactSurfaceHandler final
  : public constraint::ContactSurfaceHandler
{
public:
  constraint::ContactSurfaceParams createParams(
      const collision::Contact& contact,
      const size_t numContactsOnCollisionObject) const override
  {
    ++mNumCreateParamsCalls;
    return ContactSurfaceHandler::createParams(
        contact, numContactsOnCollisionObject);
  }

  mutable std::size_t mNumCreateParamsCalls{0u};
};

// Like gz-physics' contact-properties handler when a callback asks for a
// maximum error reduction velocity: it sets the global limit after creating
// each contact constraint.
class ErrorReductionVelocityContactSurfaceHandler final
  : public constraint::ContactSurfaceHandler
{
public:
  constraint::ContactConstraintPtr createConstraint(
      collision::Contact& contact,
      const size_t numContactsOnCollisionObject,
      const double timeStep) const override
  {
    auto constraint = ContactSurfaceHandler::createConstraint(
        contact, numContactsOnCollisionObject, timeStep);
    if (mMaxErrorReductionVelocity >= 0.0) {
      constraint::ContactConstraint::setMaxErrorReductionVelocity(
          mMaxErrorReductionVelocity);
    }
    return constraint;
  }

  double mMaxErrorReductionVelocity{-1.0};
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

  void setPreviousDeactivationGroupsForTest(bool value)
  {
    mHadDeactivationGroups = value;
  }

  void setCollisionResultForTest(const collision::Contact& contact)
  {
    mCollisionResult.clear();
    mCollisionResult.addContact(contact);
  }

  void addCollisionContactForTest(const collision::Contact& contact)
  {
    mCollisionResult.addContact(contact);
  }

  bool clearInactiveConstrainedGroupsForTest()
  {
    return clearInactiveConstrainedGroups();
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
TEST(ConstraintSolver, ConstructorsInstallFCLCollisionDetector)
{
  const ConstructorProbeConstraintSolver defaultSolver;
  EXPECT_DOUBLE_EQ(0.001, defaultSolver.getTimeStep());
  const auto defaultDetector
      = std::dynamic_pointer_cast<const collision::FCLCollisionDetector>(
          defaultSolver.getCollisionDetector());
  ASSERT_NE(nullptr, defaultDetector);
  EXPECT_EQ(
      collision::FCLCollisionDetector::PRIMITIVE,
      defaultDetector->getPrimitiveShapeType());

  DART_SUPPRESS_DEPRECATED_BEGIN
  const ConstructorProbeConstraintSolver explicitSolver(0.002);
  DART_SUPPRESS_DEPRECATED_END
  EXPECT_DOUBLE_EQ(0.002, explicitSolver.getTimeStep());
  const auto explicitDetector
      = std::dynamic_pointer_cast<const collision::FCLCollisionDetector>(
          explicitSolver.getCollisionDetector());
  ASSERT_NE(nullptr, explicitDetector);
  EXPECT_EQ(
      collision::FCLCollisionDetector::PRIMITIVE,
      explicitDetector->getPrimitiveShapeType());
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
  // The explicitly selected dart detector emits one centroid contact per flat
  // box-vs-plane pair, not a per-corner manifold.
  EXPECT_GE(expectedContacts.getNumContacts(), numBoxes);
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
  EXPECT_GE(serialContacts.getNumContacts(), kNumBoxes);
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
  EXPECT_GE(serialContacts.getNumContacts(), kNumBoxes);
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
TEST(ConstraintSolver, ContactSurfaceHandlerMayRejectContacts)
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
  auto rejectingHandler = std::make_shared<RejectingContactSurfaceHandler>();
  solver->addContactSurfaceHandler(rejectingHandler);
  solver->removeContactSurfaceHandler(defaultHandler);

  world->addSkeleton(createSolverTestPlane("ground"));
  world->addSkeleton(createSolverTestBox(
      "box", Eigen::Vector3d::Ones(), Eigen::Vector3d(0.0, 0.0, 0.49), true));

  ASSERT_NO_FATAL_FAILURE(world->step());
  EXPECT_GT(rejectingHandler->mNumCreateConstraintCalls, 0u);
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
TEST(ConstraintSolver, SimulationPreparationDoesNotRunContactSurfaceHandlers)
{
  auto world = createWorld();
  world->setTimeStep(0.001);

  simulation::DeactivationOptions deactivation;
  deactivation.mEnabled = false;
  world->setDeactivationOptions(deactivation);

  auto* solver = world->getConstraintSolver();
  solver->setCollisionDetector(collision::DARTCollisionDetector::create());
  solver->setNumSimulationThreads(1u);

  auto handler = std::make_shared<CountingContactSurfaceHandler>();
  solver->addContactSurfaceHandler(handler);
  ASSERT_NE(nullptr, handler->getParent());

  world->addSkeleton(createSolverTestPlane("ground"));
  world->addSkeleton(createSolverTestBox(
      "box", Eigen::Vector3d::Ones(), Eigen::Vector3d(0.0, 0.0, 0.49), true));

  // The first step enters simulation mode. Its preparation must not call the
  // handler, so the step calls it exactly once per contact.
  ASSERT_FALSE(world->isInSimulationMode());
  world->step();
  ASSERT_GT(world->getLastCollisionResult().getNumContacts(), 0u);
  EXPECT_EQ(
      world->getLastCollisionResult().getNumContacts(),
      handler->mNumCreateParamsCalls);

  // Adding a skeleton makes the next step prepare again.
  world->addSkeleton(createSolverTestBox(
      "box2", Eigen::Vector3d::Ones(), Eigen::Vector3d(3.0, 0.0, 0.49), true));
  ASSERT_FALSE(world->isInSimulationMode());
  handler->mNumCreateParamsCalls = 0u;
  world->step();
  EXPECT_EQ(
      world->getLastCollisionResult().getNumContacts(),
      handler->mNumCreateParamsCalls);

  // An explicit preparation calls no handler and keeps the handler chain.
  world->addSkeleton(createSolverTestBox(
      "box3", Eigen::Vector3d::Ones(), Eigen::Vector3d(-3.0, 0.0, 0.49), true));
  handler->mNumCreateParamsCalls = 0u;
  world->enterSimulationMode();
  EXPECT_EQ(0u, handler->mNumCreateParamsCalls);
  EXPECT_EQ(handler, solver->getLastContactSurfaceHandler());
}

//==============================================================================
TEST(ConstraintSolver, ContactHandlerErrorReductionVelocityAppliesToWholeStep)
{
  const auto createBoxWorld
      = [](const std::shared_ptr<ErrorReductionVelocityContactSurfaceHandler>&
               handler) {
          auto world = createWorld();
          world->setTimeStep(0.001);

          simulation::DeactivationOptions deactivation;
          deactivation.mEnabled = false;
          world->setDeactivationOptions(deactivation);

          auto* solver = world->getConstraintSolver();
          solver->setCollisionDetector(
              collision::DARTCollisionDetector::create());
          solver->setNumSimulationThreads(1u);
          solver->addContactSurfaceHandler(handler);

          world->addSkeleton(createSolverTestPlane("ground"));
          world->addSkeleton(createSolverTestBox(
              "box",
              Eigen::Vector3d::Ones(),
              Eigen::Vector3d(0.0, 0.0, 0.49),
              true));
          return world;
        };

  constraint::ContactConstraint::resetMaxErrorReductionVelocity();
  auto settingHandler
      = std::make_shared<ErrorReductionVelocityContactSurfaceHandler>();
  auto settingWorld = createBoxWorld(settingHandler);
  auto presetWorld = createBoxWorld(
      std::make_shared<ErrorReductionVelocityContactSurfaceHandler>());
  settingWorld->step();
  presetWorld->step();

  // Correcting the 1 cm penetration needs far more than this limit.
  constexpr double kMaxErrorReductionVelocity = 1e-4;

  // One world's handler sets the limit while the step creates the contact
  // constraints; the other world has it set before the step.
  settingHandler->mMaxErrorReductionVelocity = kMaxErrorReductionVelocity;
  settingWorld->step();
  constraint::ContactConstraint::setMaxErrorReductionVelocity(
      kMaxErrorReductionVelocity);
  presetWorld->step();
  constraint::ContactConstraint::resetMaxErrorReductionVelocity();

  const Eigen::VectorXd presetVelocities
      = presetWorld->getSkeleton("box")->getVelocities();
  const Eigen::VectorXd settingVelocities
      = settingWorld->getSkeleton("box")->getVelocities();
  EXPECT_TRUE(presetVelocities == settingVelocities)
      << "set before the step: " << presetVelocities.transpose()
      << "\nset by the handler: " << settingVelocities.transpose();
}

//==============================================================================
namespace {

// Rejects the contacts between two bodies, like a user handler that lets one
// body pass through another, and builds stock constraints for the others.
class PairRejectingContactSurfaceHandler final
  : public constraint::ContactSurfaceHandler
{
public:
  PairRejectingContactSurfaceHandler(
      const dynamics::BodyNode* bodyNode1, const dynamics::BodyNode* bodyNode2)
    : mBodyNode1(bodyNode1), mBodyNode2(bodyNode2)
  {
    // Do nothing
  }

  constraint::ContactConstraintPtr createConstraint(
      collision::Contact& contact,
      const size_t numContactsOnCollisionObject,
      const double timeStep) const override
  {
    const auto* bodyNode1 = contact.collisionObject1->getBodyNode();
    const auto* bodyNode2 = contact.collisionObject2->getBodyNode();
    if ((bodyNode1 == mBodyNode1 && bodyNode2 == mBodyNode2)
        || (bodyNode1 == mBodyNode2 && bodyNode2 == mBodyNode1)) {
      return nullptr;
    }

    return ContactSurfaceHandler::createConstraint(
        contact, numContactsOnCollisionObject, timeStep);
  }

private:
  const dynamics::BodyNode* mBodyNode1;
  const dynamics::BodyNode* mBodyNode2;
};

// One detector of each built-in kind, to clone for each world.
std::vector<collision::CollisionDetectorPtr> createCollisionDetectorPrototypes()
{
  std::vector<collision::CollisionDetectorPtr> detectors{
      collision::DARTCollisionDetector::create(),
      collision::FCLCollisionDetector::create()};
#if HAVE_BULLET
  detectors.push_back(collision::BulletCollisionDetector::create());
#endif
#if HAVE_ODE
  detectors.push_back(collision::OdeCollisionDetector::create());
#endif
  return detectors;
}

// A box ground whose top face is at z = 0, like the one gz-physics builds.
dynamics::SkeletonPtr createSleepTestGround()
{
  return createSolverTestBox(
      "ground",
      Eigen::Vector3d(20.0, 20.0, 1.0),
      Eigen::Vector3d(0.0, 0.0, -0.5),
      false);
}

// A frictionless sphere sliding along +x on the ground.
dynamics::SkeletonPtr createSlidingPuck(
    const std::string& name, double x, double speed)
{
  constexpr double radius = 0.2;
  auto puck = dynamics::Skeleton::create(name);
  dynamics::BodyNode::Properties bodyProperties(
      dynamics::BodyNode::AspectProperties(name + "_body"));
  bodyProperties.mInertia.setMass(1.0);
  auto pair = puck->createJointAndBodyNodePair<dynamics::FreeJoint>(
      nullptr, dynamics::FreeJoint::Properties(), bodyProperties);
  auto* shapeNode = pair.second->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(
      std::make_shared<dynamics::SphereShape>(radius));
  shapeNode->getDynamicsAspect()->setFrictionCoeff(0.0);

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(x, 0.0, radius - 5e-4);
  pair.first->setPositions(dynamics::FreeJoint::convertToPositions(transform));
  Eigen::Vector6d velocity = Eigen::Vector6d::Zero();
  velocity[3] = speed;
  pair.first->setVelocities(velocity);
  return puck;
}

// Makes the world enter simulation mode again on its next step without
// changing anything the step-start sleep check compares: either a skeleton of
// another World changes (the structural version is process-wide), or the
// thread count changes.
void invalidateSimulationModeFromOutside(
    simulation::World& world,
    simulation::World& otherWorld,
    bool changeOtherWorld)
{
  if (changeOtherWorld) {
    auto* body = otherWorld.getSkeleton(0)->getBodyNode(0);
    body->setName(body->getName() + "_renamed");
  } else {
    world.setNumSimulationThreads(world.getNumSimulationThreads() + 1u);
  }
}

bool hasContactBetween(
    const collision::CollisionResult& result,
    const dynamics::BodyNode* bodyNode1,
    const dynamics::BodyNode* bodyNode2)
{
  for (const auto& contact : result.getContacts()) {
    const auto* contactBodyNode1 = contact.collisionObject1->getBodyNode();
    const auto* contactBodyNode2 = contact.collisionObject2->getBodyNode();
    if ((contactBodyNode1 == bodyNode1 && contactBodyNode2 == bodyNode2)
        || (contactBodyNode1 == bodyNode2 && contactBodyNode2 == bodyNode1)) {
      return true;
    }
  }
  return false;
}

// The resting flag, sleep candidacy, island index and quiet dwell of every
// skeleton.
std::vector<std::tuple<bool, bool, int, double>> getSleepStates(
    const simulation::World& world)
{
  std::vector<std::tuple<bool, bool, int, double>> states;
  for (std::size_t i = 0u; i < world.getNumSkeletons(); ++i) {
    const auto skeleton = world.getSkeleton(i);
    states.emplace_back(
        skeleton->isResting(),
        skeleton->isSleepCandidate(),
        skeleton->getIslandIndex(),
        skeleton->getRestDwellTime());
  }
  return states;
}

} // namespace

//==============================================================================
// A frictionless puck slides through a box, and the handler rejects their
// contacts. Preparing for simulation builds constraints for those contacts
// through a stateless handler; it must not use them to change any sleep state,
// such as waking the box.
TEST(
    ConstraintSolver,
    SimulationPreparationKeepsSleepStateThroughRejectedContacts)
{
  for (const auto& detector : createCollisionDetectorPrototypes()) {
    for (const bool changeOtherWorld : {true, false}) {
      SCOPED_TRACE(
          detector->getType()
          + (changeOtherWorld ? ", other World changed"
                              : ", thread count changed"));

      auto otherWorld = createWorld();
      otherWorld->addSkeleton(createSolverTestBox(
          "other_box",
          Eigen::Vector3d::Ones(),
          Eigen::Vector3d(0.0, 0.0, 0.5),
          true));

      auto world = createWorld();
      world->setTimeStep(0.001);
      auto* solver = world->getConstraintSolver();
      solver->setCollisionDetector(detector->cloneWithoutCollisionObjects());
      world->addSkeleton(createSleepTestGround());
      auto box = createSolverTestBox(
          "box",
          Eigen::Vector3d::Constant(0.5),
          Eigen::Vector3d(0.0, 0.0, 0.2495),
          true);
      world->addSkeleton(box);
      auto puck = createSlidingPuck("puck", -0.6, 0.1);
      world->addSkeleton(puck);
      solver->addContactSurfaceHandler(
          std::make_shared<PairRejectingContactSurfaceHandler>(
              puck->getBodyNode(0), box->getBodyNode(0)));

      // The puck is inside the box at 2.5 s. Whether the box sleeps beside a
      // rejecting handler is for the step to decide; preparation must keep
      // whatever the step decided.
      for (int i = 0; i < 2500; ++i)
        world->step();
      ASSERT_TRUE(hasContactBetween(
          world->getLastCollisionResult(),
          puck->getBodyNode(0),
          box->getBodyNode(0)));
      const bool boxResting = box->isResting();

      invalidateSimulationModeFromOutside(
          *world, *otherWorld, changeOtherWorld);
      ASSERT_FALSE(world->isInSimulationMode());
      const auto sleepStates = getSleepStates(*world);
      world->enterSimulationMode();
      EXPECT_EQ(sleepStates, getSleepStates(*world));

      const Eigen::VectorXd boxPositions = box->getPositions();
      for (int i = 0; i < 10; ++i)
        world->step();
      EXPECT_EQ(boxResting, box->isResting());
      if (boxResting) {
        EXPECT_EQ(boxPositions, box->getPositions());
      }
    }
  }
}

//==============================================================================
// Boxes A and B rest, and a frictionless puck slides into A. The step on which
// the puck reaches A wakes A, and only A. Entering simulation mode right
// before that step must not change any sleep state, so that B stays asleep:
// with the default contact surface handler, preparation would otherwise wake A
// early, and that change of sleep state makes World wake every resting body.
TEST(ConstraintSolver, SimulationPreparationLeavesSleepStateUntouched)
{
  const auto createScene = [](const collision::CollisionDetectorPtr& detector) {
    auto world = createWorld();
    world->setTimeStep(0.001);
    world->getConstraintSolver()->setCollisionDetector(
        detector->cloneWithoutCollisionObjects());
    world->addSkeleton(createSleepTestGround());
    world->addSkeleton(createSolverTestBox(
        "box_a",
        Eigen::Vector3d::Constant(0.5),
        Eigen::Vector3d(0.0, 0.0, 0.2495),
        true));
    world->addSkeleton(createSolverTestBox(
        "box_b",
        Eigen::Vector3d::Constant(0.5),
        Eigen::Vector3d(5.0, 0.0, 0.2495),
        true));
    world->addSkeleton(createSlidingPuck("puck", -2.5, 1.0));
    return world;
  };

  for (const auto& detector : createCollisionDetectorPrototypes()) {
    SCOPED_TRACE(detector->getType());

    // The step on which the puck wakes A when nothing re-enters. This world
    // finishes before the next one steps, so its sleep-state changes cannot
    // wake anything there.
    int wakeStep = 0;
    {
      auto world = createScene(detector);
      const auto boxA = world->getSkeleton("box_a");
      bool rested = false;
      for (int i = 1; i <= 5000 && wakeStep == 0; ++i) {
        world->step();
        rested = rested || boxA->isResting();
        if (rested && !boxA->isResting())
          wakeStep = i;
      }
    }
    ASSERT_GT(wakeStep, 0);

    for (const bool changeOtherWorld : {true, false}) {
      SCOPED_TRACE(
          changeOtherWorld ? "other World changed" : "thread count changed");

      auto otherWorld = createWorld();
      otherWorld->addSkeleton(createSolverTestBox(
          "other_box",
          Eigen::Vector3d::Ones(),
          Eigen::Vector3d(0.0, 0.0, 0.5),
          true));
      auto world = createScene(detector);
      const auto boxA = world->getSkeleton("box_a");
      const auto boxB = world->getSkeleton("box_b");
      for (int i = 1; i < wakeStep; ++i)
        world->step();
      ASSERT_TRUE(boxA->isResting());
      ASSERT_TRUE(boxB->isResting());

      invalidateSimulationModeFromOutside(
          *world, *otherWorld, changeOtherWorld);
      ASSERT_FALSE(world->isInSimulationMode());
      const auto sleepStates = getSleepStates(*world);
      world->enterSimulationMode();
      EXPECT_EQ(sleepStates, getSleepStates(*world));

      const Eigen::VectorXd boxBPositions = boxB->getPositions();
      world->step();
      EXPECT_FALSE(boxA->isResting());
      EXPECT_TRUE(boxB->isResting());
      EXPECT_EQ(boxBPositions, boxB->getPositions());
    }
  }
}

//==============================================================================
// Island indices are stamped only while automatic sleeping is enabled.
// Entering simulation mode after sleeping was disabled must not stamp them
// with the previous step's setting.
TEST(
    ConstraintSolver,
    SimulationPreparationStampsNoIslandWhileSleepingIsDisabled)
{
  auto world = createWorld();
  world->setTimeStep(0.001);
  world->addSkeleton(createSleepTestGround());
  auto box = createSolverTestBox(
      "box",
      Eigen::Vector3d::Constant(0.5),
      Eigen::Vector3d(0.0, 0.0, 0.2495),
      true);
  world->addSkeleton(box);
  world->step();
  ASSERT_GE(box->getIslandIndex(), 0);

  simulation::DeactivationOptions deactivation;
  deactivation.mEnabled = false;
  world->setDeactivationOptions(deactivation);
  ASSERT_EQ(-1, box->getIslandIndex());

  world->setNumSimulationThreads(world->getNumSimulationThreads() + 1u);
  world->step();
  EXPECT_EQ(-1, box->getIslandIndex());
}

//==============================================================================
// A body that leaves every contact gets island index -1 on its next step, also
// when the World enters simulation mode again right before that step:
// preparation must not make the step skip clearing the previous islands.
TEST(ConstraintSolver, SimulationPreparationLetsLiftedBodyLeaveItsIsland)
{
  auto world = createWorld();
  world->setTimeStep(0.001);
  world->addSkeleton(createSleepTestGround());
  auto box = createSolverTestBox(
      "box",
      Eigen::Vector3d::Constant(0.5),
      Eigen::Vector3d(0.0, 0.0, 0.2495),
      true);
  world->addSkeleton(box);
  world->step();
  ASSERT_GE(box->getIslandIndex(), 0);

  box->getJoint(0)->setPosition(5, 2.0); // Lift the box to z = 2.
  world->setNumSimulationThreads(world->getNumSimulationThreads() + 1u);
  world->step();
  EXPECT_EQ(-1, box->getIslandIndex());
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
namespace {

// Steps a world so that its last collision result holds contacts of the
// "changed" box, applies a change that frees the collision objects those
// contacts point to, and steps again.
template <typename Change>
void expectStepAfterChangeReportsOnlyLiveContacts(
    std::string_view name, const Change& change)
{
  SCOPED_TRACE(name);

  auto world = createWorld();
  world->getConstraintSolver()->setCollisionDetector(
      collision::DARTCollisionDetector::create());
  world->addSkeleton(createSolverTestPlane("ground"));
  world->addSkeleton(createSolverTestBox(
      "kept",
      Eigen::Vector3d::Constant(0.2),
      Eigen::Vector3d(0.0, 0.0, 0.1),
      true));
  auto changed = createSolverTestBox(
      "changed",
      Eigen::Vector3d::Constant(0.2),
      Eigen::Vector3d(1.0, 0.0, 0.1),
      true);
  world->addSkeleton(changed);

  world->step();
  // Match frames directly: inCollision() would materialize the result's lookup
  // caches and change how the next step records contacts.
  const dynamics::ShapeFrame* changedShape
      = changed->getBodyNode(0)->getShapeNode(0);
  std::size_t numChangedContacts = 0u;
  for (const auto& contact : world->getLastCollisionResult().getContacts()) {
    if (contact.getShapeFrame1() == changedShape
        || contact.getShapeFrame2() == changedShape) {
      ++numChangedContacts;
    }
  }
  ASSERT_GT(numChangedContacts, 0u);

  change(*world, changed);
  world->step();

  const auto group = world->getConstraintSolver()->getCollisionGroup();
  const auto& result = world->getLastCollisionResult();
  EXPECT_GT(result.getNumContacts(), 0u);
  for (const auto& contact : result.getContacts()) {
    EXPECT_TRUE(group->hasShapeFrame(contact.getShapeFrame1()));
    EXPECT_TRUE(group->hasShapeFrame(contact.getShapeFrame2()));
  }
}

} // namespace

//==============================================================================
// The first step after each of these changes used to rebuild the previous
// step's contacts with CollisionResult::addContact() while preparing the
// simulation, which read the collision objects the change had just freed
// (reported by ASAN and valgrind).
TEST(ConstraintSolver, StepAfterStructuralChangeReportsOnlyLiveContacts)
{
  expectStepAfterChangeReportsOnlyLiveContacts(
      "removeSkeleton", [](World& world, SkeletonPtr& skeleton) {
        world.removeSkeleton(skeleton);
        skeleton.reset();
      });

  expectStepAfterChangeReportsOnlyLiveContacts(
      "removeAllSkeletons", [](World& world, SkeletonPtr& skeleton) {
        world.removeAllSkeletons();
        skeleton.reset();
        world.addSkeleton(createSolverTestPlane("new_ground"));
        world.addSkeleton(createSolverTestBox(
            "new_box",
            Eigen::Vector3d::Constant(0.2),
            Eigen::Vector3d(0.0, 0.0, 0.1),
            true));
      });

  expectStepAfterChangeReportsOnlyLiveContacts(
      "BodyNode::remove", [](World&, SkeletonPtr& skeleton) {
        skeleton->getBodyNode(0)->remove();
      });

  expectStepAfterChangeReportsOnlyLiveContacts(
      "ShapeNode::remove", [](World&, SkeletonPtr& skeleton) {
        skeleton->getBodyNode(0)->getShapeNode(0)->remove();
      });

  // Moving a body out of the world frees its collision objects as soon as
  // anything updates the collision group before the next step.
  const auto outside = dynamics::Skeleton::create("outside");
  expectStepAfterChangeReportsOnlyLiveContacts(
      "BodyNode::moveTo", [&](World& world, SkeletonPtr& skeleton) {
        skeleton->getBodyNode(0)->moveTo(outside, nullptr);
        world.checkCollision();
      });

  expectStepAfterChangeReportsOnlyLiveContacts(
      "setCollisionDetector", [](World& world, SkeletonPtr&) {
        world.getConstraintSolver()->setCollisionDetector(
            collision::DARTCollisionDetector::create());
      });
}

//==============================================================================
TEST(ConstraintSolver, AddingSkeletonClearsExistingConstraintState)
{
  std::vector<dynamics::SkeletonPtr> skeletons;
  auto* body = createFreeBody("body", true, skeletons);
  body->setConstraintImpulse(Eigen::Vector6d::Ones());
  DART_SUPPRESS_DEPRECATED_BEGIN
  body->setColliding(true);
  DART_SUPPRESS_DEPRECATED_END

  ExposedThreadedConstraintSolver solver;
  solver.addSkeleton(skeletons[0]);

  EXPECT_TRUE(body->getConstraintImpulse().isZero());
  DART_SUPPRESS_DEPRECATED_BEGIN
  EXPECT_FALSE(body->isColliding());
  DART_SUPPRESS_DEPRECATED_END
}

//==============================================================================
TEST(ConstraintSolver, PreviousActiveConstraintsClearConstraintImpulses)
{
  std::vector<dynamics::SkeletonPtr> skeletons;
  auto* body = createFreeBody("body", true, skeletons);

  ExposedThreadedConstraintSolver solver;
  solver.addSkeletonForTest(skeletons[0]);
  solver.addActiveConstraintForTest(std::make_shared<FakeConstraint>(1u));
  body->setConstraintImpulse(Eigen::Vector6d::Ones());

  solver.solve();

  EXPECT_TRUE(body->getConstraintImpulse().isZero());
}

//==============================================================================
TEST(ConstraintSolver, PreparationPreservesPendingConstraintImpulseClear)
{
  std::vector<dynamics::SkeletonPtr> skeletons;
  auto* body = createFreeBody("body", true, skeletons);

  ExposedThreadedConstraintSolver solver;
  solver.addSkeletonForTest(skeletons[0]);
  solver.addActiveConstraintForTest(std::make_shared<FakeConstraint>(1u));
  body->setConstraintImpulse(Eigen::Vector6d::Ones());

  solver.prepareForSimulation();
  solver.solve();

  EXPECT_TRUE(body->getConstraintImpulse().isZero());
}

//==============================================================================
TEST(ConstraintSolver, PreviousCollisionResultClearsCollidingState)
{
  std::vector<dynamics::SkeletonPtr> skeletons;
  auto* body = createFreeBody("body", true, skeletons);
  auto shape = std::make_shared<dynamics::BoxShape>(Eigen::Vector3d::Ones());
  auto* shapeNode = body->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(shape);
  FakeCollisionDetector detector;
  FakeCollisionObject object(&detector, shapeNode);

  ExposedThreadedConstraintSolver solver;
  solver.addSkeletonForTest(skeletons[0]);
  solver.setCollisionResultForTest(createContact(&object, &object));
  DART_SUPPRESS_DEPRECATED_BEGIN
  body->setColliding(true);
  DART_SUPPRESS_DEPRECATED_END

  solver.solve();
  DART_SUPPRESS_DEPRECATED_BEGIN
  EXPECT_FALSE(body->isColliding());
  DART_SUPPRESS_DEPRECATED_END
}

//==============================================================================
TEST(ConstraintSolver, ClearingCollisionResultClearsCollidingState)
{
  std::vector<dynamics::SkeletonPtr> skeletons;
  auto* body = createFreeBody("body", true, skeletons);
  auto shape = std::make_shared<dynamics::BoxShape>(Eigen::Vector3d::Ones());
  auto* shapeNode = body->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(shape);
  FakeCollisionDetector detector;
  FakeCollisionObject object(&detector, shapeNode);

  ExposedThreadedConstraintSolver solver;
  solver.addSkeletonForTest(skeletons[0]);
  solver.setCollisionResultForTest(createContact(&object, &object));
  DART_SUPPRESS_DEPRECATED_BEGIN
  body->setColliding(true);
  DART_SUPPRESS_DEPRECATED_END

  solver.clearLastCollisionResult();

  EXPECT_EQ(solver.getLastCollisionResult().getNumContacts(), 0u);
  DART_SUPPRESS_DEPRECATED_BEGIN
  EXPECT_FALSE(body->isColliding());
  DART_SUPPRESS_DEPRECATED_END
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

  auto detector = collision::DARTCollisionDetector::create();
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

  auto detector = collision::DARTCollisionDetector::create();
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

  auto detector = collision::DARTCollisionDetector::create();
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
TEST(ConstraintSolver, ContactedRestingIslandSurvivesEmptyActiveSet)
{
  std::vector<dynamics::SkeletonPtr> skeletons;
  auto* fixedBody = createFreeBody("fixed", false, skeletons);
  auto* contactedBody = createFreeBody("contacted", true, skeletons);
  createFreeBody("stale", true, skeletons);

  auto shape = std::make_shared<dynamics::BoxShape>(Eigen::Vector3d::Ones());
  auto* fixedShapeNode = fixedBody->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(shape);
  auto* contactedShapeNode = contactedBody->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(shape);

  FakeCollisionDetector detector;
  FakeCollisionObject fixedObject(&detector, fixedShapeNode);
  FakeCollisionObject contactedObject(&detector, contactedShapeNode);
  const auto contact = createContact(&contactedObject, &fixedObject);

  const auto& contacted = skeletons[1];
  contacted->setResting(true);
  contacted->setIslandIndex(0);
  const auto& stale = skeletons[2];
  stale->setResting(true);
  stale->setIslandIndex(1);

  ExposedThreadedConstraintSolver solver;
  solver.setDeactivationActive(true);
  solver.setPreviousDeactivationGroupsForTest(true);
  solver.setCollisionResultForTest(contact);
  solver.addCollisionContactForTest(
      createContact(&fixedObject, &contactedObject));
  for (const auto& skeleton : skeletons)
    solver.addSkeletonForTest(skeleton);

  EXPECT_TRUE(solver.clearInactiveConstrainedGroupsForTest());
  EXPECT_TRUE(contacted->isResting());
  EXPECT_EQ(contacted->getIslandIndex(), 0);
  EXPECT_FALSE(stale->isResting());
  EXPECT_EQ(stale->getIslandIndex(), -1);
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

  auto detector = collision::DARTCollisionDetector::create();
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

  auto detector = collision::DARTCollisionDetector::create();
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

  auto detector = collision::DARTCollisionDetector::create();
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

  auto detector = collision::DARTCollisionDetector::create();
  ExposedDARTCollisionObject fixedObject(detector.get(), fixedShapeNode);
  ExposedDARTCollisionObject dynamicObject(detector.get(), dynamicShapeNode);

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
  EXPECT_GE(contacts.getNumContacts(), kNumBoxes);

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

//==============================================================================
namespace {

// Sets an out-of-range value through a class-wide constraint parameter setter,
// expects the getter to report the bound named by the setter's warning, then
// restores the previous value so nothing leaks into other tests in this binary.
void expectSetterClampsToBound(
    void (*set)(double), double (*get)(), double invalid, double bound)
{
  const double previous = get();
  set(invalid);
  EXPECT_DOUBLE_EQ(bound, get()) << "argument " << invalid;
  set(previous);
  EXPECT_DOUBLE_EQ(previous, get());
}

template <typename Constraint>
void expectParameterSettersClampToBounds(const char* name)
{
  SCOPED_TRACE(name);
  expectSetterClampsToBound(
      &Constraint::setErrorAllowance,
      &Constraint::getErrorAllowance,
      -0.25,
      0.0);
  expectSetterClampsToBound(
      &Constraint::setErrorReductionParameter,
      &Constraint::getErrorReductionParameter,
      -0.25,
      0.0);
  expectSetterClampsToBound(
      &Constraint::setErrorReductionParameter,
      &Constraint::getErrorReductionParameter,
      1.25,
      1.0);
  expectSetterClampsToBound(
      &Constraint::setMaxErrorReductionVelocity,
      &Constraint::getMaxErrorReductionVelocity,
      -0.25,
      0.0);
  expectSetterClampsToBound(
      &Constraint::setConstraintForceMixing,
      &Constraint::getConstraintForceMixing,
      0.0,
      1e-9);
}

} // namespace

//==============================================================================
// Regression test for https://github.com/dartsim/dart/issues/3501: the setters
// warned that an invalid argument "is set to" the bound but stored the invalid
// argument anyway.
TEST(ConstraintSolver, ParameterSettersClampInvalidValues)
{
  expectParameterSettersClampToBounds<constraint::DynamicJointConstraint>(
      "DynamicJointConstraint");
  expectParameterSettersClampToBounds<constraint::JointConstraint>(
      "JointConstraint");
  expectParameterSettersClampToBounds<constraint::JointLimitConstraint>(
      "JointLimitConstraint");
  expectParameterSettersClampToBounds<constraint::SoftContactConstraint>(
      "SoftContactConstraint");

  {
    // ContactConstraint::setMaxErrorReductionVelocity already stores the bound
    // and also switches off the adaptive max-ERV policy, so it is not probed.
    SCOPED_TRACE("ContactConstraint");
    using constraint::ContactConstraint;
    expectSetterClampsToBound(
        &ContactConstraint::setErrorAllowance,
        &ContactConstraint::getErrorAllowance,
        -0.25,
        0.0);
    expectSetterClampsToBound(
        &ContactConstraint::setErrorReductionParameter,
        &ContactConstraint::getErrorReductionParameter,
        -0.25,
        0.0);
    expectSetterClampsToBound(
        &ContactConstraint::setErrorReductionParameter,
        &ContactConstraint::getErrorReductionParameter,
        1.25,
        1.0);
    expectSetterClampsToBound(
        &ContactConstraint::setConstraintForceMixing,
        &ContactConstraint::getConstraintForceMixing,
        0.0,
        1e-9);
  }

  expectSetterClampsToBound(
      &constraint::JointCoulombFrictionConstraint::setConstraintForceMixing,
      &constraint::JointCoulombFrictionConstraint::getConstraintForceMixing,
      0.0,
      1e-9);
  expectSetterClampsToBound(
      &constraint::ServoMotorConstraint::setConstraintForceMixing,
      &constraint::ServoMotorConstraint::getConstraintForceMixing,
      0.0,
      1e-9);
}

//==============================================================================
// #3056: ConstraintSolver treats CollisionOption::maxNumContacts as a contact
// budget that the colliding pairs share.
namespace {

constexpr std::size_t kCapTestBoxes = 8u;

// A box ground and kCapTestBoxes unit boxes 1 cm deep in it. Box i is tilted
// about x by tiltStep * (i + 1), which gives every pair distinct depths.
std::shared_ptr<World> createCapTestWorld(double tiltStep)
{
  auto world = createWorld();
  simulation::DeactivationOptions deactivation;
  deactivation.mEnabled = false;
  world->setDeactivationOptions(deactivation);
  // Box ground: box-box manifolds carry per-point depths.
  world->addSkeleton(createSolverTestBox(
      "ground",
      Eigen::Vector3d(40.0, 4.0, 1.0),
      Eigen::Vector3d(7.0, 0.0, -0.5),
      false));
  for (std::size_t i = 0u; i < kCapTestBoxes; ++i) {
    auto box = createSolverTestBox(
        "box_" + std::to_string(i),
        Eigen::Vector3d::Ones(),
        Eigen::Vector3d::Zero(),
        true);
    Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
    tf.linear()
        = Eigen::AngleAxisd(tiltStep * (i + 1.0), Eigen::Vector3d::UnitX())
              .toRotationMatrix();
    tf.translation() = Eigen::Vector3d(2.0 * i, 0.0, 0.49);
    box->getJoint(0)->setPositions(dynamics::FreeJoint::convertToPositions(tf));
    world->addSkeleton(box);
  }
  return world;
}

using CapTestPair = std::pair<const void*, const void*>;

CapTestPair shapeFramePair(const collision::Contact& contact)
{
  const void* a = contact.collisionObject1->getShapeFrame();
  const void* b = contact.collisionObject2->getShapeFrame();
  return std::less<const void*>()(b, a) ? CapTestPair(b, a) : CapTestPair(a, b);
}

std::map<CapTestPair, std::vector<collision::Contact>> contactsByPair(
    const collision::CollisionResult& result)
{
  std::map<CapTestPair, std::vector<collision::Contact>> pairs;
  for (const auto& contact : result.getContacts())
    pairs[shapeFramePair(contact)].push_back(contact);
  return pairs;
}

// A dart detector that rewrites its result after the parent's collide(), like
// gz-physics' GzOdeCollisionDetector::LimitCollisionPairMaxContacts does. The
// filter also gets the option the detector was called with.
class PostFilteringDetector : public collision::DARTCollisionDetector
{
public:
  using Filter = std::function<void(
      const collision::CollisionOption&, collision::CollisionResult&)>;

  static std::shared_ptr<PostFilteringDetector> create(Filter filter)
  {
    return std::shared_ptr<PostFilteringDetector>(
        new PostFilteringDetector(std::move(filter)));
  }

  using collision::DARTCollisionDetector::collide;

  bool collide(
      collision::CollisionGroup* group,
      const collision::CollisionOption& option,
      collision::CollisionResult* result) override
  {
    const bool collided
        = collision::DARTCollisionDetector::collide(group, option, result);
    if (result != nullptr)
      mFilter(option, *result);
    return collided;
  }

private:
  explicit PostFilteringDetector(Filter filter) : mFilter(std::move(filter))
  {
    // Do nothing
  }

  Filter mFilter;
};

void keepFirstContactPerPair(
    const collision::CollisionOption& /*option*/,
    collision::CollisionResult& result)
{
  const auto all = result.getContacts();
  result.clear();
  std::set<CapTestPair> seen;
  for (const auto& contact : all) {
    if (seen.insert(shapeFramePair(contact)).second)
      result.addContact(contact);
  }
}

// Records the option of every collide() call.
template <typename Detector>
class OptionRecordingDetector : public Detector
{
public:
  static std::shared_ptr<OptionRecordingDetector> create()
  {
    return std::shared_ptr<OptionRecordingDetector>(
        new OptionRecordingDetector());
  }

  using Detector::collide;

  bool collide(
      collision::CollisionGroup* group,
      const collision::CollisionOption& option,
      collision::CollisionResult* result) override
  {
    mOptions.push_back(option);
    return Detector::collide(group, option, result);
  }

  std::vector<collision::CollisionOption> mOptions;
};

dynamics::SkeletonPtr createCapTestBody(
    const std::string& name,
    const dynamics::ShapePtr& shape,
    const Eigen::Vector3d& position,
    const Eigen::Matrix3d& rotation = Eigen::Matrix3d::Identity())
{
  auto skeleton = dynamics::Skeleton::create(name);
  auto* body
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>().second;
  body->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(shape);
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.linear() = rotation;
  tf.translation() = position;
  skeleton->getJoint(0)->setPositions(
      dynamics::FreeJoint::convertToPositions(tf));
  return skeleton;
}

// Boxes, a sphere, and a cylinder a few millimeters deep in a box ground; one
// box slides.
std::shared_ptr<World> createMixedCapTestWorld()
{
  auto world = createWorld();
  simulation::DeactivationOptions deactivation;
  deactivation.mEnabled = false;
  world->setDeactivationOptions(deactivation);
  world->addSkeleton(createSolverTestBox(
      "ground",
      Eigen::Vector3d(10.0, 4.0, 1.0),
      Eigen::Vector3d(2.0, 0.0, -0.5),
      false));

  auto slidingBox = createCapTestBody(
      "sliding_box",
      std::make_shared<dynamics::BoxShape>(Eigen::Vector3d::Constant(0.5)),
      Eigen::Vector3d(0.0, 0.0, 0.245));
  static_cast<dynamics::FreeJoint*>(slidingBox->getJoint(0))
      ->setLinearVelocity(Eigen::Vector3d(0.5, 0.0, 0.0));
  world->addSkeleton(slidingBox);

  constexpr double kTilt = 0.1;
  world->addSkeleton(createCapTestBody(
      "tilted_box",
      std::make_shared<dynamics::BoxShape>(Eigen::Vector3d::Constant(0.5)),
      Eigen::Vector3d(
          1.0, 0.0, 0.25 * (std::cos(kTilt) + std::sin(kTilt)) - 0.003),
      Eigen::AngleAxisd(kTilt, Eigen::Vector3d::UnitX()).toRotationMatrix()));
  world->addSkeleton(createCapTestBody(
      "sphere",
      std::make_shared<dynamics::SphereShape>(0.2),
      Eigen::Vector3d(2.0, 0.0, 0.198)));
  world->addSkeleton(createCapTestBody(
      "cylinder",
      std::make_shared<dynamics::CylinderShape>(0.2, 0.4),
      Eigen::Vector3d(3.0, 0.0, 0.198)));
  return world;
}

std::string capTestObjectName(const collision::CollisionObject* object)
{
  return object->getBodyNode()->getSkeleton()->getName();
}

void expectSameContacts(
    const collision::CollisionResult& actual,
    const collision::CollisionResult& expected)
{
  ASSERT_EQ(actual.getNumContacts(), expected.getNumContacts());
  for (std::size_t i = 0u; i < actual.getNumContacts(); ++i) {
    const auto& a = actual.getContact(i);
    const auto& e = expected.getContact(i);
    EXPECT_EQ(
        capTestObjectName(a.collisionObject1),
        capTestObjectName(e.collisionObject1))
        << "contact " << i;
    EXPECT_EQ(
        capTestObjectName(a.collisionObject2),
        capTestObjectName(e.collisionObject2))
        << "contact " << i;
    EXPECT_TRUE(a.point == e.point) << "contact " << i;
    EXPECT_TRUE(a.normal == e.normal) << "contact " << i;
    EXPECT_EQ(a.penetrationDepth, e.penetrationDepth) << "contact " << i;
  }
}

} // namespace

//==============================================================================
// #3056: when the contact demand exceeds CollisionOption::maxNumContacts, every
// colliding pair keeps its deepest contact, the remaining budget is shared
// round-robin, and a pair's second contact is the one farthest from its first
// (instead of starving the pairs late in broadphase order).
TEST(ConstraintSolver, ContactCapOverflowSharesBudgetAcrossPairs)
{
  auto world = createCapTestWorld(0.002);
  auto* solver = world->getConstraintSolver();
  solver->setCollisionDetector(collision::DARTCollisionDetector::create());

  // Full demand from an independent detector (same geometry, no cap).
  auto referenceGroup = collision::DARTCollisionDetector::create()
                            ->createCollisionGroupAsSharedPtr();
  for (std::size_t i = 0u; i < world->getNumSkeletons(); ++i)
    referenceGroup->addShapeFramesOf(world->getSkeleton(i).get());
  collision::CollisionResult reference;
  referenceGroup->collide(
      collision::CollisionOption(true, 100000u), &reference);
  auto demand = contactsByPair(reference);
  ASSERT_EQ(demand.size(), kCapTestBoxes);
  ASSERT_GT(reference.getNumContacts(), kCapTestBoxes + 3u);
  for (const auto& [pair, contacts] : demand) {
    ASSERT_GE(contacts.size(), 2u);
    const auto depth = [](const auto& x, const auto& y) {
      return x.penetrationDepth < y.penetrationDepth;
    };
    ASSERT_NE(
        std::max_element(contacts.begin(), contacts.end(), depth)
            ->penetrationDepth,
        std::min_element(contacts.begin(), contacts.end(), depth)
            ->penetrationDepth)
        << "need distinct depths within a pair";
  }

  solver->getCollisionOption().maxNumContacts = kCapTestBoxes + 3u;
  world->step();

  const auto& result = solver->getLastCollisionResult();
  EXPECT_EQ(result.getNumContacts(), kCapTestBoxes + 3u);
  const auto kept = contactsByPair(result);
  EXPECT_EQ(kept.size(), kCapTestBoxes) << "a colliding pair was starved";
  std::size_t pairsWithTwo = 0u;
  for (const auto& [pair, contacts] : kept) {
    ASSERT_EQ(demand.count(pair), 1u);
    const auto& all = demand[pair];
    ASSERT_LE(contacts.size(), 2u);
    const auto deeper = [](const auto& x, const auto& y) {
      return x.penetrationDepth < y.penetrationDepth;
    };
    const auto& deepest
        = *std::max_element(contacts.begin(), contacts.end(), deeper);
    EXPECT_DOUBLE_EQ(
        deepest.penetrationDepth,
        std::max_element(all.begin(), all.end(), deeper)->penetrationDepth);
    if (contacts.size() == 2u) {
      ++pairsWithTwo;
      double farthest = 0.0;
      for (const auto& contact : all)
        farthest = std::max(farthest, (contact.point - deepest.point).norm());
      EXPECT_NEAR(
          (contacts[0].point - contacts[1].point).norm(), farthest, 1e-9)
          << "second contact is not the farthest from the deepest";
    }
  }
  EXPECT_EQ(pairsWithTwo, 3u);

  // Below the cap nothing is trimmed.
  solver->getCollisionOption().maxNumContacts = 1000u;
  world->step();
  EXPECT_EQ(
      solver->getLastCollisionResult().getNumContacts(),
      reference.getNumContacts());
}

//==============================================================================
// #3056: within a pair the trim keeps contacts the solver can use before ones
// it skips (non-finite or negative depth), keeps the deepest ones once the
// spread no longer separates them, and keeps the detector's order.
TEST(ConstraintSolver, ContactCapOverflowKeepsDeepestSolvableContacts)
{
  constexpr std::size_t kStackedContacts = 40u;
  constexpr std::size_t kCap = 45u;

  auto world = createCapTestWorld(0.0);
  const auto* firstBox = world->getSkeleton("box_0")->getBodyNode(0);
  // Replaces the first box's contacts with kStackedContacts contacts at one
  // point with increasing depths, after a NaN-depth and a negative-depth one.
  auto detector = PostFilteringDetector::create(
      [firstBox](
          const collision::CollisionOption& /*option*/,
          collision::CollisionResult& result) {
        const auto all = result.getContacts();
        result.clear();
        bool replaced = false;
        for (const auto& contact : all) {
          if (contact.collisionObject1->getBodyNode() != firstBox
              && contact.collisionObject2->getBodyNode() != firstBox) {
            result.addContact(contact);
            continue;
          }
          if (replaced)
            continue;
          replaced = true;
          auto stacked = contact;
          stacked.penetrationDepth = std::numeric_limits<double>::quiet_NaN();
          result.addContact(stacked);
          stacked.penetrationDepth = -0.01;
          result.addContact(stacked);
          for (std::size_t k = 1u; k <= kStackedContacts; ++k) {
            stacked.penetrationDepth = 1e-3 * static_cast<double>(k);
            result.addContact(stacked);
          }
        }
      });
  auto* solver = world->getConstraintSolver();
  solver->setCollisionDetector(detector);
  solver->getCollisionOption().maxNumContacts = kCap;

  world->step();

  const auto& result = solver->getLastCollisionResult();
  EXPECT_EQ(result.getNumContacts(), kCap);
  const auto kept = contactsByPair(result);
  EXPECT_EQ(kept.size(), kCapTestBoxes) << "a colliding pair was starved";
  std::vector<double> stackedDepths;
  for (const auto& contact : result.getContacts()) {
    if (contact.collisionObject1->getBodyNode() == firstBox
        || contact.collisionObject2->getBodyNode() == firstBox) {
      stackedDepths.push_back(contact.penetrationDepth);
    }
  }
  // The other boxes keep their few contacts and the stacked pair the rest of
  // the budget: more than the 16 farthest-point picks, all of them from the
  // deepest solvable contacts, in detector order.
  ASSERT_GT(stackedDepths.size(), 16u);
  ASSERT_LE(stackedDepths.size(), kStackedContacts);
  const std::size_t firstKept = kStackedContacts - stackedDepths.size() + 1u;
  for (std::size_t k = 0u; k < stackedDepths.size(); ++k) {
    EXPECT_DOUBLE_EQ(
        stackedDepths[k], 1e-3 * static_cast<double>(firstKept + k))
        << "kept contact " << k;
  }
}

//==============================================================================
// #3056: contacts the solver skips get only the budget the solvable ones leave,
// so a pair the detector reports first with only skipped contacts (proximity
// contacts of allowNegativePenetrationDepthContacts, or non-finite ones) cannot
// starve a pair the solver needs.
TEST(ConstraintSolver, ContactCapOverflowGivesSolvableContactsTheBudgetFirst)
{
  auto world = createCapTestWorld(0.0);
  // Gives every contact of the first pair in detector order a negative depth.
  auto detector = PostFilteringDetector::create(
      [](const collision::CollisionOption& /*option*/,
         collision::CollisionResult& result) {
        if (result.getNumContacts() == 0u)
          return;
        const auto first = shapeFramePair(result.getContact(0));
        for (std::size_t i = 0u; i < result.getNumContacts(); ++i) {
          if (shapeFramePair(result.getContact(i)) == first)
            result.getContact(i).penetrationDepth = -0.01;
        }
      });
  auto* solver = world->getConstraintSolver();
  solver->setCollisionDetector(detector);

  world->step();
  const auto demand = contactsByPair(solver->getLastCollisionResult());
  ASSERT_EQ(demand.size(), kCapTestBoxes);
  std::size_t numSkipped = 0u;
  for (const auto& contact : solver->getLastCollisionResult().getContacts())
    numSkipped += contact.penetrationDepth < 0.0 ? 1u : 0u;
  const std::size_t numSolvable
      = solver->getLastCollisionResult().getNumContacts() - numSkipped;
  ASSERT_GE(numSkipped, 2u);

  // Fewer slots than pairs: every pair with a solvable contact keeps one.
  solver->getCollisionOption().maxNumContacts = kCapTestBoxes - 1u;
  world->step();
  const auto& result = solver->getLastCollisionResult();
  EXPECT_EQ(result.getNumContacts(), kCapTestBoxes - 1u);
  EXPECT_EQ(contactsByPair(result).size(), kCapTestBoxes - 1u)
      << "a pair with solvable contacts was starved";
  for (const auto& contact : result.getContacts())
    EXPECT_GE(contact.penetrationDepth, 0.0);

  // Room for every solvable contact and one more: the spare slot goes to a
  // skipped contact, which the result still reports.
  solver->getCollisionOption().maxNumContacts = numSolvable + 1u;
  world->step();
  std::size_t numKeptSkipped = 0u;
  for (const auto& contact : solver->getLastCollisionResult().getContacts())
    numKeptSkipped += contact.penetrationDepth < 0.0 ? 1u : 0u;
  EXPECT_EQ(
      solver->getLastCollisionResult().getNumContacts(), numSolvable + 1u);
  EXPECT_EQ(numKeptSkipped, 1u);
}

//==============================================================================
// #3056: a contact between bodies that cannot react, such as a velocity-driven
// box on the static ground, never becomes an active constraint, so it gets
// only the budget the reactive pairs leave.
TEST(ConstraintSolver, ContactCapOverflowGivesReactivePairsTheBudgetFirst)
{
  auto world = createCapTestWorld(0.0);
  auto* driven = world->getSkeleton("box_0")->getJoint(0);
  driven->setActuatorType(dynamics::Joint::VELOCITY);
  const auto* drivenBody = world->getSkeleton("box_0")->getBodyNode(0);
  auto* solver = world->getConstraintSolver();
  solver->setCollisionDetector(collision::DARTCollisionDetector::create());
  world->step();
  ASSERT_EQ(
      contactsByPair(solver->getLastCollisionResult()).size(), kCapTestBoxes);

  solver->getCollisionOption().maxNumContacts = kCapTestBoxes - 1u;
  world->step();
  const auto& result = solver->getLastCollisionResult();
  EXPECT_EQ(result.getNumContacts(), kCapTestBoxes - 1u);
  EXPECT_EQ(contactsByPair(result).size(), kCapTestBoxes - 1u)
      << "a pair with a reactive body was starved";
  for (const auto& contact : result.getContacts()) {
    EXPECT_NE(contact.collisionObject1->getBodyNode(), drivenBody);
    EXPECT_NE(contact.collisionObject2->getBodyNode(), drivenBody);
  }
}

//==============================================================================
// #3056: a detector that drops contacts per pair after its parent's collide()
// (gz-physics does) must still see every pair; a capped parent collide would
// stop in broadphase order and the post-filter would hide the saturation.
TEST(ConstraintSolver, ContactCapAppliesAfterDetectorPostFilter)
{
  auto world = createCapTestWorld(0.0);
  auto* solver = world->getConstraintSolver();
  solver->setCollisionDetector(
      PostFilteringDetector::create(&keepFirstContactPerPair));
  solver->getCollisionOption().maxNumContacts = kCapTestBoxes + 3u;

  world->step();
  EXPECT_EQ(
      contactsByPair(solver->getLastCollisionResult()).size(), kCapTestBoxes)
      << "a colliding pair was starved";

  // A supported box stays near its rest height of 0.5 m; a starved one falls
  // about 0.2 m in these 0.2 s.
  for (int i = 0; i < 200; ++i)
    world->step();
  for (std::size_t i = 1u; i <= kCapTestBoxes; ++i) {
    EXPECT_GT(
        world->getSkeleton(i)->getBodyNode(0)->getTransform().translation().z(),
        0.45)
        << "box " << i << " fell through the ground";
  }
}

//==============================================================================
// #3056: detection stops at its bound (8 * cap here) the way it used to stop at
// the cap, so a scene whose raw demand reaches the bound is incomplete: the
// pairs found before the bound share the budget and each keeps a contact, and
// the pairs found later get none (the documented limit).
TEST(ConstraintSolver, ContactCapAtDetectionBoundSharesDetectedPairs)
{
  constexpr std::size_t kCap = 20u;
  constexpr std::size_t kBound = 8u * kCap;
  constexpr std::size_t kContactsPerPair = 30u;
  // The pairs that get contacts before detection stops at the bound.
  constexpr std::size_t kDetectedPairs
      = (kBound + kContactsPerPair - 1u) / kContactsPerPair;
  static_assert(kDetectedPairs < kCapTestBoxes, "pairs must follow the bound");
  static_assert(kDetectedPairs <= kCap, "every detected pair fits the budget");

  auto world = createCapTestWorld(0.0);
  std::size_t requested = 0u;
  std::size_t detected = 0u;
  std::vector<CapTestPair> detectedPairs;
  // Reports kContactsPerPair contacts per colliding pair and, like the
  // built-in detectors, stops once it has found option.maxNumContacts.
  auto detector = PostFilteringDetector::create(
      [&](const collision::CollisionOption& option,
          collision::CollisionResult& result) {
        requested = option.maxNumContacts;
        const auto all = result.getContacts();
        result.clear();
        detectedPairs.clear();
        for (const auto& contact : all) {
          const auto pair = shapeFramePair(contact);
          if (std::find(detectedPairs.begin(), detectedPairs.end(), pair)
              != detectedPairs.end()) {
            continue;
          }
          if (result.getNumContacts() >= option.maxNumContacts)
            break;
          detectedPairs.push_back(pair);
          for (std::size_t k = 0u;
               k < kContactsPerPair
               && result.getNumContacts() < option.maxNumContacts;
               ++k) {
            auto copy = contact;
            copy.point.x() += 0.005 * static_cast<double>(k);
            result.addContact(copy);
          }
        }
        detected = result.getNumContacts();
      });
  auto* solver = world->getConstraintSolver();
  solver->setCollisionDetector(detector);
  auto& option = solver->getCollisionOption();
  option.maxNumContacts = kCap;

  world->step();

  EXPECT_EQ(requested, kBound);
  EXPECT_EQ(detected, kBound);
  ASSERT_EQ(detectedPairs.size(), kDetectedPairs);
  const auto& result = solver->getLastCollisionResult();
  EXPECT_EQ(result.getNumContacts(), kCap);
  const auto kept = contactsByPair(result);
  EXPECT_EQ(kept.size(), kDetectedPairs)
      << "a pair found before the bound was starved, or one after it was kept";
  for (const auto& pair : detectedPairs) {
    ASSERT_EQ(kept.count(pair), 1u) << "a pair found before the bound starved";
    EXPECT_GE(kept.at(pair).size(), kCap / kDetectedPairs);
  }
  EXPECT_EQ(option.maxNumContacts, kCap);
  EXPECT_EQ(option.maxNumContactsPerPair, 0u);
}

//==============================================================================
// #3056: ContactSurfaceHandler::createParams() receives each pair's number of
// kept contacts, which DefaultContactSurfaceHandler multiplies slip compliance
// by, so a trimmed pair's count drops with its contacts.
TEST(ConstraintSolver, ContactCapCountsKeptContactsForSurfaceHandlers)
{
  class CountRecordingHandler : public constraint::ContactSurfaceHandler
  {
  public:
    constraint::ContactSurfaceParams createParams(
        const collision::Contact& contact,
        const size_t numContactsOnCollisionObject) const override
    {
      mCounts[shapeFramePair(contact)].push_back(numContactsOnCollisionObject);
      return ContactSurfaceHandler::createParams(
          contact, numContactsOnCollisionObject);
    }

    mutable std::map<CapTestPair, std::vector<std::size_t>> mCounts;
  };

  auto world = createCapTestWorld(0.002);
  auto* solver = world->getConstraintSolver();
  solver->setCollisionDetector(collision::DARTCollisionDetector::create());
  auto handler = std::make_shared<CountRecordingHandler>();
  solver->addContactSurfaceHandler(handler);
  solver->getCollisionOption().maxNumContacts = kCapTestBoxes + 3u;

  world->step();

  const auto kept = contactsByPair(solver->getLastCollisionResult());
  ASSERT_EQ(kept.size(), kCapTestBoxes) << "a colliding pair was starved";
  ASSERT_EQ(handler->mCounts.size(), kCapTestBoxes);
  for (const auto& [pair, contacts] : kept) {
    for (const auto count : handler->mCounts[pair])
      EXPECT_EQ(count, contacts.size());
  }
}

//==============================================================================
// #3056: the solver's query reaches the detector with every field the built-in
// detectors read keeping its effect below the budget: the user's flags and
// filter; the per-pair request (FCL asks max(100, cap) per pair when
// maxNumContactsPerPair is 0, ODE, Bullet, and the dart detector use
// getEffectiveMaxNumContactsPerPair() and need at least the cap, and the dart
// detector keeps a full manifold for SIZE_MAX); and maxNumContacts raised to
// the detection bound. Binary checks and unlimited budgets pass unchanged.
TEST(ConstraintSolver, ContactCapDetectionOptionKeepsDetectorSemantics)
{
  constexpr auto kUnlimited = std::numeric_limits<std::size_t>::max();
  const auto fclPerPairRequest = [](const collision::CollisionOption& option) {
    return option.maxNumContactsPerPair > 0u
               ? option.getEffectiveMaxNumContactsPerPair()
               : std::max<std::size_t>(100u, option.maxNumContacts);
  };

  struct Case
  {
    std::size_t cap;
    std::size_t perPair;
    bool dartDetector;
    std::size_t expectedBound;
    std::size_t expectedPerPair;
  };
  const Case cases[] = {
      {60u, 0u, false, 480u, 100u},
      {11u, 0u, false, 100u, 100u}, // the bound never cuts FCL's 100
      {11u, 0u, true, 100u, 100u},
      {60u, 4u, false, 480u, 4u},
      {60u, 200u, false, 480u, 60u},
      {60u, kUnlimited, true, 480u, kUnlimited},
      {60u, kUnlimited, false, 480u, 60u},
      {1u, 0u, true, 1u, 0u},
      {0u, 0u, false, 0u, 0u},
      {kUnlimited, 0u, true, kUnlimited, 0u},
  };
  for (const auto& c : cases) {
    SCOPED_TRACE(
        "cap " + std::to_string(c.cap) + " perPair " + std::to_string(c.perPair)
        + (c.dartDetector ? " dart" : " fcl"));
    auto world = createCapTestWorld(0.0);
    auto* solver = world->getConstraintSolver();
    const std::vector<collision::CollisionOption>* received = nullptr;
    if (c.dartDetector) {
      auto detector
          = OptionRecordingDetector<collision::DARTCollisionDetector>::create();
      received = &detector->mOptions;
      solver->setCollisionDetector(detector);
    } else {
      auto detector
          = OptionRecordingDetector<collision::FCLCollisionDetector>::create();
      received = &detector->mOptions;
      solver->setCollisionDetector(detector);
    }
    auto& option = solver->getCollisionOption();
    option.maxNumContacts = c.cap;
    option.maxNumContactsPerPair = c.perPair;
    option.allowNegativePenetrationDepthContacts = true;

    world->step();

    ASSERT_FALSE(received->empty());
    for (const auto& seen : *received) {
      EXPECT_EQ(seen.maxNumContacts, c.expectedBound);
      EXPECT_EQ(seen.maxNumContactsPerPair, c.expectedPerPair);
      EXPECT_EQ(seen.enableContact, option.enableContact);
      EXPECT_EQ(
          seen.allowNegativePenetrationDepthContacts,
          option.allowNegativePenetrationDepthContacts);
      EXPECT_EQ(seen.collisionFilter, option.collisionFilter);
      if (!c.dartDetector) {
        EXPECT_EQ(fclPerPairRequest(seen), fclPerPairRequest(option));
      }
      EXPECT_GE(
          seen.getEffectiveMaxNumContactsPerPair(),
          option.getEffectiveMaxNumContactsPerPair());
      // The dart detector's solver-facing manifold target.
      EXPECT_EQ(
          std::min<std::size_t>(seen.getEffectiveMaxNumContactsPerPair(), 3u),
          std::min<std::size_t>(
              option.getEffectiveMaxNumContactsPerPair(), 3u));
    }
    EXPECT_EQ(option.maxNumContacts, c.cap);
    EXPECT_EQ(option.maxNumContactsPerPair, c.perPair);
  }
}

//==============================================================================
// #3056: within the budget the solver's contacts are bit-identical to the
// legacy capped query for every built-in detector. Each detector runs at cap 99
// (below FCL's legacy per-pair request of 100 and ODE's per-pair maximum of
// 250; ODE's trimesh cylinder alone reports dozens of contacts) and then at a
// cap just above that run's largest demand. There the legacy query comes
// closest to stopping, and its requests differ most from the solver's: ODE asks
// for cap instead of 100 contacts per pair, and a small cap gets the detection
// bound's floor of 100 instead of 8 * cap. A twin world answers the legacy
// query on the same state every step, so ODE's contact history evolves the
// same way in both.
TEST(ConstraintSolver, ContactCapWithinBudgetMatchesLegacyQuery)
{
  const std::vector<
      std::pair<std::string, std::function<collision::CollisionDetectorPtr()>>>
      detectors
      = { {"fcl",
           [] {
             return collision::FCLCollisionDetector::create();
           }},
          {"fcl_mesh",
           [] {
             auto detector = collision::FCLCollisionDetector::create();
             detector->setPrimitiveShapeType(
                 collision::FCLCollisionDetector::MESH);
             return detector;
           }},
          {"dart",
           [] {
             return collision::DARTCollisionDetector::create();
           }},
#if HAVE_BULLET
          {"bullet",
           [] {
             return collision::BulletCollisionDetector::create();
           }},
#endif
#if HAVE_ODE
          {"ode",
           [] {
             return collision::OdeCollisionDetector::create();
           }},
#endif
        };

  // Steps a world at `cap`, compares every step with the legacy query, and
  // records the largest legacy demand.
  const auto expectLegacyContacts =
      [](const std::function<collision::CollisionDetectorPtr()>& createDetector,
         std::size_t cap,
         std::size_t& maxDemand) {
        SCOPED_TRACE("cap " + std::to_string(cap));
        auto world = createMixedCapTestWorld();
        auto twin = createMixedCapTestWorld();
        for (auto* w : {world.get(), twin.get()}) {
          w->getConstraintSolver()->setCollisionDetector(createDetector());
          w->getConstraintSolver()->getCollisionOption().maxNumContacts = cap;
          w->enterSimulationMode();
        }

        auto* twinSolver = twin->getConstraintSolver();
        maxDemand = 0u;
        for (int step = 0; step < 30; ++step) {
          SCOPED_TRACE("step " + std::to_string(step));
          for (std::size_t i = 0u; i < world->getNumSkeletons(); ++i) {
            twin->getSkeleton(i)->setPositions(
                world->getSkeleton(i)->getPositions());
            twin->getSkeleton(i)->setVelocities(
                world->getSkeleton(i)->getVelocities());
          }
          collision::CollisionResult legacy;
          twinSolver->getCollisionGroup()->collide(
              twinSolver->getCollisionOption(), &legacy);
          ASSERT_LT(legacy.getNumContacts(), cap);
          maxDemand = std::max(maxDemand, legacy.getNumContacts());

          world->step();
          expectSameContacts(
              world->getConstraintSolver()->getLastCollisionResult(), legacy);
        }
      };

  for (const auto& [name, createDetector] : detectors) {
    SCOPED_TRACE(name);
    std::size_t maxDemand = 0u;
    expectLegacyContacts(createDetector, 99u, maxDemand);
    ASSERT_FALSE(HasFatalFailure());
    expectLegacyContacts(createDetector, maxDemand + 1u, maxDemand);
  }
}

//==============================================================================
// #3056: the trim keeps its scratch per thread, so worlds stepped concurrently
// on different threads trim independently and end in the same state as a world
// stepped alone.
TEST(ConstraintSolver, ContactCapTrimsConcurrentWorldsIndependently)
{
  constexpr std::size_t kWorlds = 4u;
  constexpr std::size_t kCap = kCapTestBoxes + 3u;
  const auto createSaturatedWorld = [] {
    auto world = createCapTestWorld(0.002);
    auto* solver = world->getConstraintSolver();
    solver->setCollisionDetector(collision::DARTCollisionDetector::create());
    solver->getCollisionOption().maxNumContacts = kCap;
    return world;
  };
  const auto stepAndGetState = [](World& world) {
    for (int i = 0; i < 100; ++i)
      world.step();
    std::vector<double> state;
    for (std::size_t i = 0u; i < world.getNumSkeletons(); ++i) {
      const Eigen::VectorXd q = world.getSkeleton(i)->getPositions();
      const Eigen::VectorXd v = world.getSkeleton(i)->getVelocities();
      state.insert(state.end(), q.data(), q.data() + q.size());
      state.insert(state.end(), v.data(), v.data() + v.size());
    }
    return state;
  };

  auto reference = createSaturatedWorld();
  const auto expected = stepAndGetState(*reference);
  ASSERT_EQ(reference->getLastCollisionResult().getNumContacts(), kCap)
      << "the scene no longer exceeds its contact budget";

  std::vector<std::shared_ptr<World>> worlds;
  for (std::size_t i = 0u; i < kWorlds; ++i)
    worlds.push_back(createSaturatedWorld());
  std::vector<std::vector<double>> states(kWorlds);
  std::vector<std::thread> threads;
  for (std::size_t i = 0u; i < kWorlds; ++i)
    threads.emplace_back([&, i] { states[i] = stepAndGetState(*worlds[i]); });
  for (auto& thread : threads)
    thread.join();

  for (std::size_t i = 0u; i < kWorlds; ++i)
    EXPECT_TRUE(states[i] == expected) << "world " << i;
}
