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

#include "TestHelpers.hpp"
#include "dart/constraint/BallJointConstraint.hpp"
#include "dart/constraint/BoxedLcpConstraintSolver.hpp"
#include "dart/constraint/ConstrainedGroup.hpp"
#include "dart/constraint/ConstraintSolver.hpp"
#include "dart/constraint/ContactSurface.hpp"
#include "dart/constraint/DantzigBoxedLcpSolver.hpp"
#include "dart/constraint/PgsBoxedLcpSolver.hpp"
#include "dart/constraint/detail/IslandSolveExecutor.hpp"
#include "dart/dynamics/FreeJoint.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/simulation/World.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
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

class DerivedDantzigBoxedLcpSolver final
  : public constraint::DantzigBoxedLcpSolver
{
};

class DerivedPgsBoxedLcpSolver final : public constraint::PgsBoxedLcpSolver
{
};

class ExposedBoxedLcpConstraintSolver final
  : public constraint::BoxedLcpConstraintSolver
{
public:
  using BoxedLcpConstraintSolver::BoxedLcpConstraintSolver;

  bool isParallelSolveSafeForTest() const
  {
    return isConstrainedGroupSolveThreadSafe();
  }
};

class ExposedThreadedConstraintSolver final
  : public constraint::ConstraintSolver
{
public:
  bool hasOwnedIslandSolveExecutor() const
  {
    return mOwnedIslandSolveExecutor != nullptr;
  }

  bool hasInjectedIslandSolveExecutor() const
  {
    return mIslandSolveExecutor != nullptr;
  }

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

  void solveGroupsForTest()
  {
    solveConstrainedGroups();
  }

  int getNumSolvedGroups() const
  {
    return mNumSolvedGroups.load(std::memory_order_relaxed);
  }

  int getMaxConcurrentSolves() const
  {
    return mMaxConcurrentSolves.load(std::memory_order_relaxed);
  }

protected:
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

    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    mNumSolvedGroups.fetch_add(1, std::memory_order_relaxed);
    mConcurrentSolves.fetch_sub(1, std::memory_order_relaxed);
  }

  bool isConstrainedGroupSolveThreadSafe() const override
  {
    return true;
  }

private:
  std::atomic<int> mConcurrentSolves{0};
  std::atomic<int> mMaxConcurrentSolves{0};
  std::atomic<int> mNumSolvedGroups{0};
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

} // namespace

//==============================================================================
std::shared_ptr<World> createWorld()
{
  return simulation::World::create();
}

//==============================================================================
TEST(ConstraintSolver, DirectThreadSettingCreatesOwnedExecutor)
{
  ExposedThreadedConstraintSolver solver;

  EXPECT_EQ(1u, solver.getNumThreads());
  EXPECT_FALSE(solver.hasOwnedIslandSolveExecutor());
  EXPECT_FALSE(solver.hasInjectedIslandSolveExecutor());

  solver.setNumThreads(0);
  EXPECT_EQ(1u, solver.getNumThreads());
  EXPECT_FALSE(solver.hasOwnedIslandSolveExecutor());

  solver.setNumThreads(4);
  EXPECT_EQ(4u, solver.getNumThreads());
  EXPECT_TRUE(solver.hasOwnedIslandSolveExecutor());

  constraint::detail::IslandSolveExecutor externalExecutor(2);
  solver.setIslandSolveExecutor(&externalExecutor);
  EXPECT_TRUE(solver.hasInjectedIslandSolveExecutor());
  EXPECT_FALSE(solver.hasOwnedIslandSolveExecutor());

  solver.setNumThreads(3);
  EXPECT_EQ(3u, solver.getNumThreads());
  EXPECT_TRUE(solver.hasInjectedIslandSolveExecutor());
  EXPECT_FALSE(solver.hasOwnedIslandSolveExecutor());

  solver.setIslandSolveExecutor(nullptr);
  EXPECT_FALSE(solver.hasInjectedIslandSolveExecutor());
  EXPECT_TRUE(solver.hasOwnedIslandSolveExecutor());

  solver.setNumThreads(1);
  EXPECT_EQ(1u, solver.getNumThreads());
  EXPECT_FALSE(solver.hasOwnedIslandSolveExecutor());
}

//==============================================================================
TEST(ConstraintSolver, DirectThreadSettingSolvesGroupsInParallel)
{
  ExposedThreadedConstraintSolver solver;
  solver.setNumThreads(4);
  solver.addFakeConstrainedGroups(8, 100);

  solver.solveGroupsForTest();

  EXPECT_EQ(8, solver.getNumSolvedGroups());
  EXPECT_GT(solver.getMaxConcurrentSolves(), 1);
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
  solver.setNumThreads(4);
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

  solver.solveGroupsForTest();

  EXPECT_EQ(2, solver.getNumSolvedGroups());
  EXPECT_GT(solver.getMaxConcurrentSolves(), 1);
}

//==============================================================================
TEST(ConstraintSolver, SharedNonReactiveBodiesForceSerialGroupSolves)
{
  std::vector<dynamics::SkeletonPtr> skeletons;
  auto* fixedBody = createFreeBody("fixed", false, skeletons);
  auto* dynamicBody1 = createFreeBody("dynamic1", true, skeletons);
  auto* dynamicBody2 = createFreeBody("dynamic2", true, skeletons);

  ExposedThreadedConstraintSolver solver;
  solver.setNumThreads(4);
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

  solver.solveGroupsForTest();

  EXPECT_EQ(2, solver.getNumSolvedGroups());
  EXPECT_EQ(1, solver.getMaxConcurrentSolves());
}

//==============================================================================
TEST(ConstraintSolver, ParallelSolveRequiresExactBuiltInSolvers)
{
  ExposedBoxedLcpConstraintSolver defaultSolver;
  EXPECT_TRUE(defaultSolver.isParallelSolveSafeForTest());

  ExposedBoxedLcpConstraintSolver noSecondarySolver(
      std::make_shared<constraint::DantzigBoxedLcpSolver>(), nullptr);
  EXPECT_TRUE(noSecondarySolver.isParallelSolveSafeForTest());

  ExposedBoxedLcpConstraintSolver derivedPrimarySolver(
      std::make_shared<DerivedDantzigBoxedLcpSolver>(),
      std::make_shared<constraint::PgsBoxedLcpSolver>());
  EXPECT_FALSE(derivedPrimarySolver.isParallelSolveSafeForTest());

  ExposedBoxedLcpConstraintSolver derivedSecondarySolver(
      std::make_shared<constraint::DantzigBoxedLcpSolver>(),
      std::make_shared<DerivedPgsBoxedLcpSolver>());
  EXPECT_FALSE(derivedSecondarySolver.isParallelSolveSafeForTest());

  auto randomizedPgs = std::make_shared<constraint::PgsBoxedLcpSolver>();
  auto option = randomizedPgs->getOption();
  option.mRandomizeConstraintOrder = true;
  randomizedPgs->setOption(option);

  ExposedBoxedLcpConstraintSolver randomizedSecondarySolver(
      std::make_shared<constraint::DantzigBoxedLcpSolver>(), randomizedPgs);
  EXPECT_FALSE(randomizedSecondarySolver.isParallelSolveSafeForTest());
}

//==============================================================================
TEST(ConstraintSolver, IslandSolveExecutorRunsAllItems)
{
  constraint::detail::IslandSolveExecutor executor(4);
  EXPECT_EQ(4u, executor.getNumThreads());

  std::vector<std::atomic<int>> counts(128);
  executor.run(counts.size(), [&](std::size_t i) {
    counts[i].fetch_add(1, std::memory_order_relaxed);
  });

  for (const auto& count : counts)
    EXPECT_EQ(1, count.load(std::memory_order_relaxed));
}

//==============================================================================
TEST(ConstraintSolver, IslandSolveExecutorSingleThreadRunsInline)
{
  constraint::detail::IslandSolveExecutor executor(0);
  EXPECT_EQ(1u, executor.getNumThreads());

  bool ranZeroCountBody = false;
  executor.run(0, [&](std::size_t) { ranZeroCountBody = true; });
  EXPECT_FALSE(ranZeroCountBody);

  std::vector<int> values(8, 0);
  executor.run(values.size(), [&](std::size_t i) {
    values[i] = static_cast<int>(i + 1);
  });

  for (std::size_t i = 0; i < values.size(); ++i)
    EXPECT_EQ(static_cast<int>(i + 1), values[i]);
}

//==============================================================================
TEST(ConstraintSolver, IslandSolveExecutorRethrowsAndCanRunAgain)
{
  constraint::detail::IslandSolveExecutor executor(3);

  EXPECT_THROW(
      executor.run(
          16,
          [](std::size_t i) {
            if (i == 7)
              throw std::runtime_error("expected test exception");
          }),
      std::runtime_error);

  std::atomic<int> count{0};
  executor.run(
      16, [&](std::size_t) { count.fetch_add(1, std::memory_order_relaxed); });

  EXPECT_EQ(16, count.load(std::memory_order_relaxed));
}

//==============================================================================
TEST(ConstraintSolver, DefaultConstactSurfaceHandler)
{
  auto world = createWorld();
  auto solver = world->getConstraintSolver();
  ASSERT_NE(nullptr, solver->getLastContactSurfaceHandler());
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
