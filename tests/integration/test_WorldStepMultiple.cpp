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
#include "dart/collision/CollisionGroup.hpp"
#include "dart/collision/CollisionObject.hpp"
#include "dart/collision/CollisionResult.hpp"
#include "dart/collision/dart/DARTCollisionDetector.hpp"
#include "dart/common/Aspect.hpp"
#include "dart/constraint/BoxedLcpConstraintSolver.hpp"
#include "dart/constraint/ContactSurface.hpp"
#include "dart/dynamics/BoxShape.hpp"
#include "dart/dynamics/FreeJoint.hpp"
#include "dart/dynamics/Group.hpp"
#include "dart/dynamics/PointMass.hpp"
#include "dart/dynamics/ShapeNode.hpp"
#include "dart/dynamics/SoftBodyNode.hpp"
#include "dart/dynamics/WeldJoint.hpp"
#include "dart/simulation/World.hpp"

#include <gtest/gtest.h>

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <future>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <type_traits>

#include <cstring>

using namespace dart;
using namespace dart::dynamics;
using namespace dart::simulation;

static_assert(
    std::is_same<decltype(&World::step), void (World::*)(bool)>::value,
    "Legacy step member-pointer resolution must remain unchanged");
static_assert(
    !std::is_move_constructible<World::EditScope>::value,
    "EditScope retains its existing nonmovable surface");

namespace dart::simulation {
struct WorldTestAccess
{
  static void setFrame(World& world, int frame)
  {
    world.mFrame = frame;
  }
};
} // namespace dart::simulation

// Private registry probes defined in World.cpp for tests only.
namespace dart::simulation::detail {
std::size_t getIndexedWorldCount();
std::size_t getRegistryLockCount();
} // namespace dart::simulation::detail

namespace {

WorldPtr makeCountWorld(
    bool soft = false,
    bool sleeping = false,
    common::MemoryAllocator* allocator = nullptr)
{
  WorldConfig config;
  if (allocator)
    config.baseAllocator = allocator;
  auto world = std::make_shared<World>(config);
  world->setCollisionDetector(collision::DARTCollisionDetector::create());
  auto options = world->getDeactivationOptions();
  options.mEnabled = sleeping;
  options.mTimeUntilSleep = 0.01;
  world->setDeactivationOptions(options);
  auto floor = Skeleton::create("floor");
  auto* ground = floor->createJointAndBodyNodePair<WeldJoint>().second;
  ground->createShapeNodeWith<CollisionAspect, DynamicsAspect>(
      std::make_shared<BoxShape>(Eigen::Vector3d(10, 10, 0.1)));
  auto pose = Eigen::Isometry3d::Identity();
  pose.translation().z() = -0.05;
  ground->getParentJoint()->setTransformFromParentBodyNode(pose);
  floor->setMobile(false);
  world->addSkeleton(floor);
  auto body = Skeleton::create("body");
  BodyNode* node;
  if (soft) {
    const auto properties = SoftBodyNodeHelper::makeBoxProperties(
        Eigen::Vector3d::Constant(0.2),
        Eigen::Isometry3d::Identity(),
        Eigen::Vector3i(3, 3, 3),
        1.0,
        500.0,
        10.0,
        1.0);
    node = body->createJointAndBodyNodePair<FreeJoint, SoftBodyNode>(
                   nullptr,
                   FreeJoint::Properties(),
                   SoftBodyNode::Properties(BodyNode::Properties(), properties))
               .second;
  } else {
    node = body->createJointAndBodyNodePair<FreeJoint>().second;
    node->createShapeNodeWith<CollisionAspect, DynamicsAspect>(
        std::make_shared<BoxShape>(Eigen::Vector3d::Constant(0.2)));
  }
  body->setPosition(5, 0.0999);
  world->addSkeleton(body);
  return world;
}

template <typename A, typename B>
void expectBits(const Eigen::MatrixBase<A>& a, const Eigen::MatrixBase<B>& b)
{
  ASSERT_EQ(a.size(), b.size());
  const auto lhs = a.eval();
  const auto rhs = b.eval();
  if (lhs.size() > 0) {
    EXPECT_EQ(
        0, std::memcmp(lhs.data(), rhs.data(), lhs.size() * sizeof(double)));
  }
}

void expectCountWorldsEqual(const World& a, const World& b)
{
  const auto lhsTime = a.getTime();
  const auto rhsTime = b.getTime();
  EXPECT_EQ(0, std::memcmp(&lhsTime, &rhsTime, sizeof(double)));
  EXPECT_EQ(a.getSimFrames(), b.getSimFrames());
  ASSERT_EQ(a.getNumSkeletons(), b.getNumSkeletons());
  for (std::size_t s = 0; s < a.getNumSkeletons(); ++s) {
    const auto lhs = a.getSkeleton(s);
    const auto rhs = b.getSkeleton(s);
    expectBits(lhs->getPositions(), rhs->getPositions());
    expectBits(lhs->getVelocities(), rhs->getVelocities());
    expectBits(lhs->getAccelerations(), rhs->getAccelerations());
    expectBits(lhs->getCommands(), rhs->getCommands());
    expectBits(lhs->getForces(), rhs->getForces());
    expectBits(lhs->getConstraintForces(), rhs->getConstraintForces());
    EXPECT_EQ(lhs->isImpulseApplied(), rhs->isImpulseApplied());
    EXPECT_EQ(lhs->isResting(), rhs->isResting());
    EXPECT_EQ(lhs->isSleepCandidate(), rhs->isSleepCandidate());
    for (std::size_t i = 0; i < lhs->getNumBodyNodes(); ++i) {
      const auto* x = lhs->getBodyNode(i);
      const auto* y = rhs->getBodyNode(i);
      expectBits(x->getConstraintImpulse(), y->getConstraintImpulse());
      expectBits(x->getExternalForceLocal(), y->getExternalForceLocal());
      if (const auto* xs = dynamic_cast<const SoftBodyNode*>(x)) {
        const auto* ys = dynamic_cast<const SoftBodyNode*>(y);
        ASSERT_NE(ys, nullptr);
        ASSERT_EQ(xs->getNumPointMasses(), ys->getNumPointMasses());
        for (std::size_t p = 0; p < xs->getNumPointMasses(); ++p) {
          const auto* xp = xs->getPointMass(p);
          const auto* yp = ys->getPointMass(p);
          const auto& xstate = xp->getState();
          const auto& ystate = yp->getState();
          expectBits(xstate.mPositions, ystate.mPositions);
          expectBits(xstate.mVelocities, ystate.mVelocities);
          expectBits(xstate.mAccelerations, ystate.mAccelerations);
          expectBits(xstate.mForces, ystate.mForces);
          expectBits(xp->getConstraintImpulses(), yp->getConstraintImpulses());
        }
      }
    }
  }
  const auto& x = a.getLastCollisionResult();
  const auto& y = b.getLastCollisionResult();
  ASSERT_EQ(x.getNumContacts(), y.getNumContacts());
  for (std::size_t i = 0; i < x.getNumContacts(); ++i) {
    const auto& xc = x.getContact(i);
    const auto& yc = y.getContact(i);
    expectBits(xc.point, yc.point);
    expectBits(xc.normal, yc.normal);
    expectBits(xc.force, yc.force);
    EXPECT_EQ(
        0,
        std::memcmp(
            &xc.penetrationDepth, &yc.penetrationDepth, sizeof(double)));
    EXPECT_EQ(
        xc.getBodyNodePtr1()->getSkeleton()->getName(),
        yc.getBodyNodePtr1()->getSkeleton()->getName());
    EXPECT_EQ(
        xc.getBodyNodePtr2()->getSkeleton()->getName(),
        yc.getBodyNodePtr2()->getSkeleton()->getName());
  }
}

class CountStepHandler final : public constraint::ContactSurfaceHandler
{
public:
  std::function<void()> callback;
  constraint::ContactSurfaceParams createParams(
      const collision::Contact& contact, std::size_t count) const override
  {
    if (callback)
      callback();
    return ContactSurfaceHandler::createParams(contact, count);
  }
};

class CountPreparationSolver final : public constraint::BoxedLcpConstraintSolver
{
public:
  std::function<void()> callback;

protected:
  void reserveConstrainedGroupScratch(
      const constraint::ConstrainedGroup& group) override
  {
    BoxedLcpConstraintSolver::reserveConstrainedGroupScratch(group);
    if (callback)
      callback();
  }
};

class CountOperationGate
{
public:
  void arrive()
  {
    std::unique_lock<std::mutex> lock(mMutex);
    mEntered = true;
    mCondition.notify_all();
    if (!mCondition.wait_for(
            lock, std::chrono::seconds(10), [&] { return mOpen; }))
      throw std::runtime_error("Lifecycle test gate timed out");
  }
  bool wait()
  {
    std::unique_lock<std::mutex> lock(mMutex);
    return mCondition.wait_for(
        lock, std::chrono::seconds(5), [&] { return mEntered; });
  }
  void open()
  {
    const std::lock_guard<std::mutex> lock(mMutex);
    mOpen = true;
    mCondition.notify_all();
  }

private:
  std::mutex mMutex;
  std::condition_variable mCondition;
  bool mEntered = false;
  bool mOpen = false;
};

void expectCountEntriesRejected(const WorldPtr& world)
{
  const auto frame = world->getSimFrames();
  const auto time = world->getTime();
  const auto q = world->getSkeleton(1)->getPositions();
  const auto v = world->getSkeleton(1)->getVelocities();
  try {
    world->step();
    ADD_FAILURE() << "Busy World accepted a step";
  } catch (const std::runtime_error& error) {
    EXPECT_EQ(std::string(error.what()).find("WorldBusyError: "), 0u);
  }
  EXPECT_THROW(world->stepMultiple(0), std::runtime_error);
  EXPECT_THROW(world->stepMultiple(2), std::runtime_error);
  EXPECT_THROW(world->compile(), std::runtime_error);
  WorldState output;
  EXPECT_THROW(world->captureState(), std::runtime_error);
  EXPECT_THROW(world->captureState(output), std::runtime_error);
  EXPECT_THROW(world->restoreState(output), std::runtime_error);
  EXPECT_EQ(world->getSimFrames(), frame);
  EXPECT_DOUBLE_EQ(world->getTime(), time);
  expectBits(q, world->getSkeleton(1)->getPositions());
  expectBits(v, world->getSkeleton(1)->getVelocities());
}

template <typename Mutation>
void expectCountBusy(const Mutation& mutation)
{
  try {
    mutation();
    ADD_FAILURE() << "Busy World accepted model mutation";
  } catch (const std::runtime_error& error) {
    EXPECT_EQ(std::string(error.what()).find("WorldBusyError: "), 0u);
  }
}

void expectCountModelMutationsRejected(const WorldPtr& world)
{
  expectCountBusy(
      [&] { world->addSimpleFrame(std::make_shared<SimpleFrame>()); });
  expectCountBusy([&] { world->addSkeleton(Skeleton::create()); });
  expectCountBusy([&] { world->removeSkeleton(world->getSkeleton(1)); });
}

} // namespace

TEST(WorldStepMultiple, RigidAndSoftSequencesMatchLegacyBitwise)
{
  for (const bool soft : {false, true})
    for (const bool sleeping : {false, true})
      for (const bool compiled : {false, true})
        for (const bool reset : {false, true})
          for (const std::size_t count : {0u, 1u, 9u}) {
            SCOPED_TRACE(soft);
            SCOPED_TRACE(sleeping);
            SCOPED_TRACE(compiled);
            SCOPED_TRACE(reset);
            SCOPED_TRACE(count);
            auto actual = makeCountWorld(soft, sleeping);
            auto expected = makeCountWorld(soft, sleeping);
            for (const auto& world : {actual, expected}) {
              world->getSkeleton(1)->setCommands(
                  Eigen::VectorXd::Constant(6, 0.02));
              world->getSkeleton(1)->getBodyNode(0)->addExtForce(
                  Eigen::Vector3d(0.1, 0.2, 0.3));
              if (compiled)
                world->compile();
            }
            actual->stepMultiple(count, reset);
            for (std::size_t i = 0; i < count; ++i)
              expected->step(reset);
            expectCountWorldsEqual(*actual, *expected);
            for (int continuation = 0; continuation < 3; ++continuation) {
              actual->step(reset);
              expected->step(reset);
              expectCountWorldsEqual(*actual, *expected);
            }
          }
}

TEST(
    WorldStepMultiple,
    ZeroDoesNotPrepareOrChangeContactsAndOverflowRejectsAtomically)
{
  auto world = makeCountWorld();
  auto reference = makeCountWorld();
  const auto topology = world->getTopologyRevision();
  const auto parameters = world->getParameterRevision();
  world->stepMultiple(0);
  EXPECT_FALSE(world->isInSimulationMode());
  EXPECT_FALSE(world->isCompiled());
  EXPECT_EQ(world->getTime(), 0);
  EXPECT_EQ(world->getSimFrames(), 0);
  EXPECT_EQ(world->getLastCollisionResult().getNumContacts(), 0);
  EXPECT_EQ(world->getTopologyRevision(), topology);
  EXPECT_EQ(world->getParameterRevision(), parameters);
  WorldTestAccess::setFrame(*world, std::numeric_limits<int>::max() - 1);
  WorldTestAccess::setFrame(*reference, std::numeric_limits<int>::max() - 1);
  EXPECT_THROW(world->stepMultiple(2), std::overflow_error);
  EXPECT_THROW(
      world->stepMultiple(std::numeric_limits<std::size_t>::max()),
      std::overflow_error);
  EXPECT_FALSE(world->isInSimulationMode());
  EXPECT_EQ(world->getSimFrames(), std::numeric_limits<int>::max() - 1);
  expectCountWorldsEqual(*world, *reference);
  world->stepMultiple(1);
  reference->step();
  EXPECT_EQ(world->getSimFrames(), std::numeric_limits<int>::max());
  EXPECT_GT(world->getLastCollisionResult().getNumContacts(), 0);
  world->stepMultiple(0);
  expectCountWorldsEqual(*world, *reference);
  EXPECT_THROW(world->stepMultiple(1), std::overflow_error);
  expectCountWorldsEqual(*world, *reference);
  EXPECT_NO_THROW(world->compile());
}

TEST(WorldStepMultiple, LegacyIntegralCallsRetainBooleanResetSemantics)
{
  auto actual = makeCountWorld();
  auto expected = makeCountWorld();
  for (const auto& world : {actual, expected})
    world->getSkeleton(1)->setCommands(Eigen::VectorXd::Constant(6, 0.02));
  actual->step(0);
  expected->step(false);
  expectCountWorldsEqual(*actual, *expected);
  actual->step(1);
  expected->step(true);
  expectCountWorldsEqual(*actual, *expected);
}

TEST(WorldStepMultiple, SleepingSequenceAndWakeContinuationMatchLegacy)
{
  for (const bool compiled : {false, true}) {
    SCOPED_TRACE(compiled);
    auto actual = makeCountWorld(false, true);
    auto expected = makeCountWorld(false, true);
    for (int i = 0; i < 3000 && !expected->getSkeleton(1)->isResting(); ++i) {
      actual->step();
      expected->step();
    }
    ASSERT_TRUE(actual->getSkeleton(1)->isResting());
    ASSERT_TRUE(expected->getSkeleton(1)->isResting());
    if (compiled) {
      actual->compile();
      expected->compile();
    }
    actual->stepMultiple(7);
    for (int i = 0; i < 7; ++i)
      expected->step();
    expectCountWorldsEqual(*actual, *expected);
    for (const auto& world : {actual, expected})
      world->getSkeleton(1)->getBodyNode(0)->addExtForce(
          Eigen::Vector3d(1, 0, 0));
    actual->stepMultiple(4, false);
    for (int i = 0; i < 4; ++i)
      expected->step(false);
    expectCountWorldsEqual(*actual, *expected);
  }
}

TEST(WorldStepMultiple, EverySubstepRetainsOwnershipAndIndependentWorldCanStep)
{
  for (const bool compiled : {false, true})
    for (const std::size_t count : {1u, 3u}) {
      SCOPED_TRACE(compiled);
      SCOPED_TRACE(count);
      auto world = makeCountWorld();
      auto unrelated = makeCountWorld();
      auto handler = std::make_shared<CountStepHandler>();
      world->getConstraintSolver()->addContactSurfaceHandler(handler);
      if (compiled)
        world->compile();
      std::array<CountOperationGate, 3> gates;
      int lastFrame = -1;
      handler->callback = [&] {
        const auto frame = world->getSimFrames();
        if (frame == lastFrame)
          return;
        lastFrame = frame;
        expectCountEntriesRejected(world);
        EXPECT_THROW({ auto edit = world->edit(); }, std::runtime_error);
        gates.at(frame).arrive();
      };
      auto stepping = std::async(std::launch::async, [&] {
        if (count == 1u)
          world->step();
        else
          world->stepMultiple(count);
      });
      for (std::size_t i = 0; i < count; ++i) {
        auto& gate = gates[i];
        const bool entered = gate.wait();
        EXPECT_TRUE(entered);
        if (entered) {
          expectCountEntriesRejected(world);
          EXPECT_THROW({ auto edit = world->edit(); }, std::runtime_error);
          EXPECT_NO_THROW(unrelated->step());
        }
        gate.open();
      }
      EXPECT_NO_THROW(stepping.get());
      handler->callback = {};
      EXPECT_EQ(world->getSimFrames(), count);
      EXPECT_EQ(unrelated->getSimFrames(), count);
      EXPECT_NO_THROW(world->compile());
      EXPECT_NO_THROW(world->step());
    }
}

TEST(WorldStepMultiple, CompileAndOuterEditCloseKeepExclusiveOwnership)
{
  for (const bool editing : {false, true}) {
    SCOPED_TRACE(editing);
    auto world = makeCountWorld();
    auto solver = std::make_unique<CountPreparationSolver>();
    auto* probe = solver.get();
    world->setConstraintSolver(std::move(solver));
    world->setCollisionDetector(collision::DARTCollisionDetector::create());
    if (editing)
      world->compile();
    CountOperationGate editGate;
    CountOperationGate compileGate;
    bool first = true;
    probe->callback = [&] {
      if (!first)
        return;
      first = false;
      expectCountEntriesRejected(world);
      EXPECT_THROW({ auto edit = world->edit(); }, std::runtime_error);
      EXPECT_THROW(
          world->getSkeleton(1)->createJointAndBodyNodePair<FreeJoint>(),
          std::runtime_error);
      EXPECT_THROW(
          world->getConstraintSolver()
              ->getCollisionGroup()
              ->removeAllShapeFrames(),
          std::runtime_error);
      compileGate.arrive();
    };
    auto operation = std::async(std::launch::async, [&] {
      if (editing) {
        auto outer = world->edit();
        world->addSkeleton(Skeleton::create("edited"));
        {
          auto inner = world->edit();
          EXPECT_FALSE(world->isCompiled());
          expectCountEntriesRejected(world);
        }
        editGate.arrive();
      } else {
        world->compile();
      }
    });
    if (editing) {
      const bool entered = editGate.wait();
      EXPECT_TRUE(entered);
      if (entered) {
        expectCountEntriesRejected(world);
        EXPECT_THROW({ auto edit = world->edit(); }, std::runtime_error);
      }
      editGate.open();
    }
    const bool entered = compileGate.wait();
    EXPECT_TRUE(entered);
    if (entered) {
      expectCountEntriesRejected(world);
      EXPECT_THROW({ auto edit = world->edit(); }, std::runtime_error);
    }
    if (entered) {
      EXPECT_THROW(
          world->getSkeleton(1)->createJointAndBodyNodePair<FreeJoint>(),
          std::runtime_error);
      EXPECT_THROW(
          world->getConstraintSolver()
              ->getCollisionGroup()
              ->removeAllShapeFrames(),
          std::runtime_error);
    }
    compileGate.open();
    EXPECT_NO_THROW(operation.get());
    probe->callback = {};
    EXPECT_TRUE(world->isCompiled());
    EXPECT_NO_THROW(world->stepMultiple(2));
  }
}

TEST(WorldStepMultiple, OuterEditRecompileSynchronizesBorrowedGroup)
{
  auto world = makeCountWorld();
  auto view = Group::create();
  auto group = world->getConstraintSolver()->getCollisionGroup();
  group->subscribeTo(view);
  world->compile();
  {
    auto edit = world->edit();
    view->addBodyNode(world->getSkeleton(1)->getBodyNode(0));
  }
  EXPECT_TRUE(world->isCompiled());
  EXPECT_NO_THROW(world->stepMultiple(2));
  EXPECT_EQ(world->getSimFrames(), 2);
  EXPECT_NO_THROW(world->compile());
}

TEST(WorldStepMultiple, CompetingIdleStepAndCompileHaveOneWinner)
{
  auto world = makeCountWorld();
  auto solver = std::make_unique<CountPreparationSolver>();
  auto* probe = solver.get();
  world->setConstraintSolver(std::move(solver));
  world->setCollisionDetector(collision::DARTCollisionDetector::create());
  CountOperationGate start;
  CountOperationGate winner;
  std::atomic<bool> first{true};
  probe->callback = [&] {
    if (first.exchange(false))
      winner.arrive();
  };
  std::promise<void> rejected;
  auto rejection = rejected.get_future();
  auto run = [&](bool compile) {
    start.arrive();
    try {
      if (compile)
        world->compile();
      else
        world->step();
      return true;
    } catch (const std::runtime_error&) {
      rejected.set_value();
      return false;
    }
  };
  auto a = std::async(std::launch::async, [&] { return run(false); });
  auto b = std::async(std::launch::async, [&] { return run(true); });
  EXPECT_TRUE(start.wait());
  start.open();
  const bool entered = winner.wait();
  EXPECT_TRUE(entered);
  EXPECT_EQ(
      rejection.wait_for(std::chrono::seconds(5)), std::future_status::ready);
  winner.open();
  const bool stepWon = a.get();
  const bool compileWon = b.get();
  EXPECT_NE(stepWon, compileWon);
  EXPECT_EQ(world->getSimFrames(), stepWon ? 1 : 0);
  probe->callback = {};
  EXPECT_NO_THROW(world->compile());
  EXPECT_NO_THROW(world->stepMultiple(2));
}

TEST(WorldStepMultiple, ExceptionsStopAtFirstFailureAndReleaseEveryOperation)
{
  auto actual = makeCountWorld();
  auto expected = makeCountWorld();
  for (const auto& world : {actual, expected}) {
    auto handler = std::make_shared<CountStepHandler>();
    handler->callback = [world = world.get()] {
      if (world->getSimFrames() == 1)
        throw std::runtime_error("intentional substep failure");
    };
    world->getConstraintSolver()->addContactSurfaceHandler(handler);
  }
  EXPECT_THROW(actual->stepMultiple(4, false), std::runtime_error);
  EXPECT_NO_THROW(expected->step(false));
  EXPECT_THROW(expected->step(false), std::runtime_error);
  EXPECT_EQ(actual->getSimFrames(), 1);
  expectCountWorldsEqual(*actual, *expected);
  for (const auto& world : {actual, expected})
    std::static_pointer_cast<CountStepHandler>(
        world->getConstraintSolver()->getLastContactSurfaceHandler())
        ->callback
        = {};
  actual->stepMultiple(2);
  expected->step();
  expected->step();
  expectCountWorldsEqual(*actual, *expected);
  auto world = makeCountWorld();
  auto solver = std::make_unique<CountPreparationSolver>();
  auto* probe = solver.get();
  world->setConstraintSolver(std::move(solver));
  probe->callback = [] {
    throw std::runtime_error("intentional compile failure");
  };
  EXPECT_THROW(world->compile(), std::runtime_error);
  EXPECT_NO_THROW({ auto edit = world->edit(); });
  EXPECT_THROW(
      {
        auto edit = world->edit();
        throw std::runtime_error("edit body");
      },
      std::runtime_error);
  probe->callback = {};
  EXPECT_NO_THROW(world->compile());
  EXPECT_NO_THROW({ auto edit = world->edit(); });
  EXPECT_NO_THROW(world->stepMultiple(2));
}

TEST(WorldStepMultiple, PreparedSingleAndCountedStepsAllocateNoBytes)
{
  for (const bool compiled : {false, true})
    for (const bool soft : {false, true}) {
      SCOPED_TRACE(compiled);
      SCOPED_TRACE(soft);
      test::CountingMemoryAllocator allocator;
      auto world = makeCountWorld(soft, false, &allocator);
      if (compiled)
        world->compile();
      world->stepMultiple(10);
      allocator.reset();
      allocator.setCountingEnabled(true);
      test::ScopedHeapAllocationCounter heap;
      test::ScopedRawHeapAllocationCounter raw;
      world->step();
      world->stepMultiple(5);
      world->stepMultiple(0);
      raw.stop();
      heap.stop();
      allocator.setCountingEnabled(false);
      const auto base = allocator.snapshot();
      EXPECT_EQ(heap.allocationCount(), 0);
      EXPECT_EQ(heap.allocationBytes(), 0);
      EXPECT_EQ(base.allocationCount, 0);
      EXPECT_EQ(base.allocationBytes, 0);
      if (!raw.skipped()) {
        EXPECT_EQ(raw.allocationCount(), 0);
        EXPECT_EQ(raw.allocationBytes(), 0);
      }
    }
}

TEST(WorldStepMultiple, StateEntriesRespectWholeCountedReservation)
{
  auto world = makeCountWorld();
  const auto state = world->captureState();
  auto output = state;
  CountOperationGate gate;
  auto handler = std::make_shared<CountStepHandler>();
  handler->callback = [&] {
    EXPECT_THROW(world->captureState(output), std::runtime_error);
    EXPECT_THROW(world->restoreState(state), std::runtime_error);
    gate.arrive();
  };
  world->getConstraintSolver()->addContactSurfaceHandler(handler);
  const auto revision = world->getParameterRevision();
  auto operation
      = std::async(std::launch::async, [&] { world->stepMultiple(3u); });
  const bool entered = gate.wait();
  EXPECT_TRUE(entered);
  if (entered) {
    EXPECT_THROW(world->captureState(), std::runtime_error);
    EXPECT_THROW(world->captureState(output), std::runtime_error);
    EXPECT_THROW(world->restoreState(state), std::runtime_error);
    expectBits(output.positions, state.positions);
    expectBits(output.velocities, state.velocities);
    EXPECT_EQ(world->getParameterRevision(), revision);
  }
  gate.open();
  EXPECT_NO_THROW(operation.get());
  EXPECT_EQ(world->getSimFrames(), 3);
  handler->callback = [] {
    throw std::runtime_error("counted callback failure");
  };
  EXPECT_THROW(world->stepMultiple(2u), std::runtime_error);
  handler->callback = {};
  EXPECT_NO_THROW(world->captureState(output));
  EXPECT_NO_THROW(world->restoreState(state));
  EXPECT_NO_THROW(world->compile());
  EXPECT_NO_THROW(world->stepMultiple(2u));
}

TEST(WorldStepMultiple, CaptureContenderCannotAcquireBetweenSubsteps)
{
  constexpr int count = 10000;
  for (int round = 0; round < 3; ++round) {
    SCOPED_TRACE(round);
    auto world = makeCountWorld();
    auto output = world->captureState();
    CountOperationGate gate;
    auto handler = std::make_shared<CountStepHandler>();
    handler->callback = [&] {
      if (world->getSimFrames() == 0)
        gate.arrive();
    };
    world->getConstraintSolver()->addContactSurfaceHandler(handler);
    auto stepping
        = std::async(std::launch::async, [&] { world->stepMultiple(count); });
    const bool entered = gate.wait();
    EXPECT_TRUE(entered);
    std::atomic<bool> stop{false};
    std::promise<void> attempted;
    auto firstAttempt = attempted.get_future();
    auto contender = std::async(std::launch::async, [&] {
      bool first = true;
      std::size_t captures = 0;
      while (!stop.load()) {
        try {
          world->captureState(output);
          ++captures;
        } catch (const std::runtime_error& error) {
          EXPECT_EQ(std::string(error.what()).find("WorldBusyError: "), 0u);
        }
        if (first) {
          first = false;
          attempted.set_value();
        }
      }
      return captures;
    });
    EXPECT_EQ(
        firstAttempt.wait_for(std::chrono::seconds(5)),
        std::future_status::ready);
    gate.open();
    EXPECT_NO_THROW(stepping.get());
    stop.store(true);
    // Captures after the whole count exits are allowed.
    EXPECT_NO_THROW(contender.get());
    EXPECT_EQ(world->getSimFrames(), count);
  }
}

TEST(WorldStepMultiple, LazyLifecycleEntriesSkipTheRegistry)
{
  if (dart::simulation::detail::getIndexedWorldCount() != 0u)
    GTEST_SKIP() << "an earlier test keeps a compiled World alive";
  auto world = makeCountWorld();
  auto state = world->captureState();
  const auto locks = dart::simulation::detail::getRegistryLockCount();
  {
    auto outer = world->edit();
    auto inner = world->edit();
  }
  EXPECT_EQ(dart::simulation::detail::getRegistryLockCount(), locks);
  world->stepMultiple(3);
  EXPECT_EQ(dart::simulation::detail::getRegistryLockCount(), locks);
  world->captureState(state);
  EXPECT_EQ(dart::simulation::detail::getRegistryLockCount(), locks);
  world->restoreState(state);
  EXPECT_EQ(dart::simulation::detail::getRegistryLockCount(), locks);
  EXPECT_FALSE(world->isCompiled());
}

TEST(WorldStepMultiple, LazyAndCompiledEditsHoldReservationUntilOuterExit)
{
  for (const bool compiled : {false, true}) {
    SCOPED_TRACE(compiled);
    auto world = makeCountWorld();
    auto unrelated = makeCountWorld();
    if (compiled)
      world->compile();
    CountOperationGate gate;
    auto editing = std::async(std::launch::async, [&] {
      auto outer = world->edit();
      {
        auto inner = world->edit();
        expectCountEntriesRejected(world);
      }
      expectCountEntriesRejected(world);
      gate.arrive();
    });
    const bool entered = gate.wait();
    EXPECT_TRUE(entered);
    if (entered) {
      expectCountEntriesRejected(world);
      EXPECT_THROW({ auto edit = world->edit(); }, std::runtime_error);
      EXPECT_NO_THROW(unrelated->stepMultiple(2));
    }
    gate.open();
    EXPECT_NO_THROW(editing.get());
    EXPECT_EQ(world->isCompiled(), compiled);
    EXPECT_EQ(unrelated->getSimFrames(), 2);
    EXPECT_NO_THROW(world->step());
  }
}

TEST(WorldStepMultiple, FailedOuterRecompileKeepsReservationAndP9ErrorRecovery)
{
  auto world = makeCountWorld();
  auto solver = std::make_unique<CountPreparationSolver>();
  auto* probe = solver.get();
  world->setConstraintSolver(std::move(solver));
  world->setCollisionDetector(collision::DARTCollisionDetector::create());
  world->compile();
  CountOperationGate gate;
  probe->callback = [&] {
    expectCountEntriesRejected(world);
    EXPECT_THROW({ auto edit = world->edit(); }, std::runtime_error);
    gate.arrive();
    throw std::runtime_error("intentional outer recompile failure");
  };
  auto editing = std::async(std::launch::async, [&] {
    auto outer = world->edit();
    world->addSkeleton(Skeleton::create("edited"));
  });
  const bool entered = gate.wait();
  EXPECT_TRUE(entered);
  if (entered) {
    expectCountEntriesRejected(world);
    EXPECT_THROW({ auto edit = world->edit(); }, std::runtime_error);
  }
  gate.open();
  EXPECT_NO_THROW(editing.get());
  EXPECT_FALSE(world->isCompiled());
  EXPECT_NO_THROW(world->stepMultiple(0));
  EXPECT_THROW(world->step(), std::runtime_error);
  EXPECT_THROW(world->stepMultiple(2), std::runtime_error);
  EXPECT_EQ(world->getSimFrames(), 0);
  probe->callback = {};
  EXPECT_THROW(world->compile(), std::runtime_error);
  EXPECT_NO_THROW(world->compile());
  EXPECT_TRUE(world->isCompiled());
  EXPECT_NO_THROW(world->stepMultiple(2));
}

namespace {

class CountStateAspect final : public common::Aspect
{
public:
  std::function<void()> callback;
  std::unique_ptr<common::Aspect> cloneAspect() const override
  {
    return std::make_unique<CountStateAspect>();
  }
  const State* getAspectState() const override
  {
    if (callback)
      callback();
    return nullptr;
  }
};

} // namespace

TEST(WorldStepMultiple, CaptureAndRestoreHoldExclusiveReservation)
{
  for (const bool compiled : {false, true})
    for (const bool restoring : {false, true}) {
      SCOPED_TRACE(restoring);
      SCOPED_TRACE(compiled);
      auto world = makeCountWorld();
      auto unrelated = makeCountWorld();
      const auto state = world->captureState();
      auto output = state;
      auto* aspect = world->getSkeleton(1)->createAspect<CountStateAspect>();
      if (compiled)
        world->compile();
      CountOperationGate gate;
      aspect->callback = [&] {
        expectCountEntriesRejected(world);
        expectCountModelMutationsRejected(world);
        EXPECT_THROW({ auto edit = world->edit(); }, std::runtime_error);
        gate.arrive();
      };
      auto operation = std::async(std::launch::async, [&] {
        if (restoring)
          world->restoreState(state);
        else
          world->captureState(output);
      });
      const bool entered = gate.wait();
      EXPECT_TRUE(entered);
      if (entered) {
        expectCountEntriesRejected(world);
        expectCountModelMutationsRejected(world);
        if (compiled) {
          expectCountBusy([&] {
            world->getConstraintSolver()
                ->getCollisionGroup()
                ->removeAllShapeFrames();
          });
        }
        EXPECT_THROW({ auto edit = world->edit(); }, std::runtime_error);
        EXPECT_NO_THROW(unrelated->step());
      }
      gate.open();
      EXPECT_NO_THROW(operation.get());
      aspect->callback = {};
      EXPECT_NO_THROW(world->stepMultiple(2));
      EXPECT_NO_THROW(world->compile());
    }
}

TEST(WorldStepMultiple, NestedWorldMarkersUnwindAfterExceptions)
{
  for (const bool compiling : {false, true}) {
    SCOPED_TRACE(compiling);
    auto outer = makeCountWorld();
    auto inner = makeCountWorld();
    auto* aspect = outer->getSkeleton(1)->createAspect<CountStateAspect>();
    auto solver = std::make_unique<CountPreparationSolver>();
    auto* probe = solver.get();
    inner->setConstraintSolver(std::move(solver));
    inner->setCollisionDetector(collision::DARTCollisionDetector::create());
    probe->callback = [] {
      throw std::runtime_error("nested World callback failure");
    };
    bool called = false;
    aspect->callback = [&] {
      called = true;
      if (compiling)
        EXPECT_THROW(inner->compile(), std::runtime_error);
      else
        EXPECT_THROW(inner->stepMultiple(2), std::runtime_error);
      // The inner operation must restore the outer World's active marker.
      EXPECT_THROW(
          outer->getSkeleton(1)->createJointAndBodyNodePair<FreeJoint>(),
          std::runtime_error);
      EXPECT_THROW({ auto edit = outer->edit(); }, std::runtime_error);
    };
    EXPECT_NO_THROW(outer->captureState());
    EXPECT_TRUE(called);
    probe->callback = {};
    aspect->callback = {};
    EXPECT_NO_THROW(inner->compile());
    EXPECT_NO_THROW(outer->compile());
    EXPECT_NO_THROW(inner->step());
    EXPECT_NO_THROW(outer->step());
  }
}
