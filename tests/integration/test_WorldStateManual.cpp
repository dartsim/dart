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

#include "AllocationCounting.hpp"
#include "dart/collision/CollisionResult.hpp"
#include "dart/constraint/BallJointConstraint.hpp"
#include "dart/constraint/BoxedLcpConstraintSolver.hpp"
#include "dart/constraint/ContactConstraint.hpp"
#include "dart/constraint/ContactSurface.hpp"
#include "dart/constraint/CouplerConstraint.hpp"
#include "dart/constraint/CylindricalJointConstraint.hpp"
#include "dart/constraint/DantzigBoxedLcpSolver.hpp"
#include "dart/constraint/JointConstraint.hpp"
#include "dart/constraint/JointCoulombFrictionConstraint.hpp"
#include "dart/constraint/JointLimitConstraint.hpp"
#include "dart/constraint/MimicMotorConstraint.hpp"
#include "dart/constraint/RevoluteJointConstraint.hpp"
#include "dart/constraint/ServoMotorConstraint.hpp"
#include "dart/constraint/SoftContactConstraint.hpp"
#include "dart/constraint/WeldJointConstraint.hpp"
#include "dart/dynamics/BoxShape.hpp"
#include "dart/dynamics/FreeJoint.hpp"
#include "dart/dynamics/RevoluteJoint.hpp"
#include "dart/dynamics/ShapeNode.hpp"
#include "dart/dynamics/WeldJoint.hpp"
#include "dart/simulation/World.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <limits>
#include <stdexcept>

#include <cmath>

using namespace dart;
using namespace dart::dynamics;
using namespace dart::simulation;

namespace {

enum class ManualKind
{
  Ball,
  Revolute,
  Cylindrical,
  Weld
};

constexpr ManualKind kinds[]
    = {ManualKind::Ball,
       ManualKind::Revolute,
       ManualKind::Cylindrical,
       ManualKind::Weld};

WorldPtr makeWorld(const std::string& name)
{
  auto world = World::create(name);
  world->setGravity(Eigen::Vector3d(1, -3, -9));
  for (int s = 0; s < 2; ++s) {
    auto skeleton = Skeleton::create(name + std::to_string(s));
    skeleton->createJointAndBodyNodePair<FreeJoint>();
    if (s == 0)
      skeleton->createJointAndBodyNodePair<FreeJoint>();
    world->addSkeleton(skeleton);
  }
  return world;
}

std::shared_ptr<constraint::DynamicJointConstraint> makeConstraint(
    ManualKind kind, BodyNode* a, BodyNode* b, bool alternate = false)
{
  const Eigen::Vector3d anchor = alternate ? Eigen::Vector3d(0.2, -0.1, 0.3)
                                           : Eigen::Vector3d(0.1, 0.2, -0.1);
  const Eigen::Vector3d axis
      = alternate ? Eigen::Vector3d::UnitX() : Eigen::Vector3d::UnitZ();
  switch (kind) {
    case ManualKind::Ball:
      return b ? std::make_shared<constraint::BallJointConstraint>(a, b, anchor)
               : std::make_shared<constraint::BallJointConstraint>(a, anchor);
    case ManualKind::Revolute:
      return b ? std::make_shared<constraint::RevoluteJointConstraint>(
                 a, b, anchor, axis, axis)
               : std::make_shared<constraint::RevoluteJointConstraint>(
                   a, anchor, axis);
    case ManualKind::Cylindrical:
      return b ? std::make_shared<constraint::CylindricalJointConstraint>(
                 a, b, anchor, axis, axis)
               : std::make_shared<constraint::CylindricalJointConstraint>(
                   a, anchor, axis);
    case ManualKind::Weld: {
      auto weld = b ? std::make_shared<constraint::WeldJointConstraint>(a, b)
                    : std::make_shared<constraint::WeldJointConstraint>(a);
      if (alternate) {
        Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
        transform.translation() = anchor;
        transform.linear() = Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitY())
                                 .toRotationMatrix();
        weld->setRelativeTransform(transform);
      }
      return weld;
    }
  }
  throw std::logic_error("unhandled manual constraint kind");
}

void addConstraint(
    const WorldPtr& world,
    ManualKind kind,
    bool bodyPair,
    bool alternate = false)
{
  world->getConstraintSolver()->addConstraint(makeConstraint(
      kind,
      world->getSkeleton(0)->getBodyNode(0),
      bodyPair ? world->getSkeleton(1)->getBodyNode(0) : nullptr,
      alternate));
}

void expectEqual(const WorldState& a, const WorldState& b, double tolerance = 0)
{
  EXPECT_TRUE(a.positions.isApprox(b.positions, tolerance));
  EXPECT_TRUE(a.velocities.isApprox(b.velocities, tolerance));
  EXPECT_TRUE(a.accelerations.isApprox(b.accelerations, tolerance));
  EXPECT_TRUE(a.forces.isApprox(b.forces, tolerance));
  EXPECT_TRUE(a.commands.isApprox(b.commands, tolerance));
  EXPECT_TRUE(a.bodyExternalForces.isApprox(b.bodyExternalForces, tolerance));
  EXPECT_EQ(a.time, b.time);
}

void expectAtomicMismatch(const WorldPtr& target, const WorldState& source)
{
  target->setTime(3.25);
  target->getSkeleton(0)->setVelocity(0, 0.42);
  const auto before = target->captureState();
  EXPECT_THROW(target->restoreState(source), std::invalid_argument);
  expectEqual(before, target->captureState());
  auto output = source;
  EXPECT_THROW(target->captureState(output), std::invalid_argument);
  expectEqual(source, output);
}

void expectUnsupported(
    const WorldPtr& world, const constraint::ConstraintBasePtr& unsupported)
{
  const auto before = world->captureState();
  auto output = before;
  world->getConstraintSolver()->addConstraint(unsupported);
  EXPECT_THROW(world->captureState(), std::invalid_argument);
  EXPECT_THROW(world->captureState(output), std::invalid_argument);
  expectEqual(before, output);
  EXPECT_THROW(world->restoreState(before), std::invalid_argument);
  world->getConstraintSolver()->removeConstraint(unsupported);
  expectEqual(before, world->captureState());
}

class RecordingSolver final : public constraint::BoxedLcpSolver
{
public:
  const std::string& getType() const override
  {
    static const std::string type = "WorldStateRecordingSolver";
    return type;
  }

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
    ++calls;
    incomingMax = 0;
    for (int i = 0; i < n; ++i)
      incomingMax = std::max(incomingMax, std::abs(x[i]));
    const bool solved
        = delegate.solve(n, A, x, b, nub, lo, hi, findex, earlyTermination);
    outgoingMax = 0;
    for (int i = 0; i < n; ++i)
      outgoingMax = std::max(outgoingMax, std::abs(x[i]));
    return solved;
  }

  int calls = 0;
  double incomingMax = 0;
  double outgoingMax = 0;

private:
  constraint::DantzigBoxedLcpSolver delegate;
};

class CustomBall final : public constraint::BallJointConstraint
{
public:
  using BallJointConstraint::BallJointConstraint;
};

WorldPtr makeSleepingWorld()
{
  auto world = World::create();
  auto floor = Skeleton::create("floor");
  auto* ground = floor->createJointAndBodyNodePair<WeldJoint>().second;
  ground->createShapeNodeWith<CollisionAspect, DynamicsAspect>(
      std::make_shared<BoxShape>(Eigen::Vector3d(10, 10, 0.1)));
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation().z() = -0.05;
  ground->getParentJoint()->setTransformFromParentBodyNode(transform);
  floor->setMobile(false);
  world->addSkeleton(floor);
  auto box = Skeleton::create("box");
  auto* body = box->createJointAndBodyNodePair<FreeJoint>().second;
  body->createShapeNodeWith<CollisionAspect, DynamicsAspect>(
      std::make_shared<BoxShape>(Eigen::Vector3d::Constant(0.1)));
  box->setPosition(5, 0.05 - 1e-6);
  world->addSkeleton(box);
  return world;
}

} // namespace

TEST(WorldStateManual, IndependentGraphsAndRenamedTargetsRoundTrip)
{
  for (const auto kind : kinds) {
    for (const bool bodyPair : {false, true}) {
      SCOPED_TRACE(static_cast<int>(kind));
      SCOPED_TRACE(bodyPair);
      auto source = makeWorld("source");
      auto target = makeWorld("independent");
      addConstraint(source, kind, bodyPair);
      addConstraint(target, kind, bodyPair);
      source->getSkeleton(0)->setPosition(0, 0.3);
      source->getSkeleton(1)->setVelocity(5, 0.4);
      source->setTime(8.5);
      const auto state = source->captureState();
      target->getSkeleton(0)->getBodyNode(0)->setName("renamed body");
      target->getSkeleton(1)->getJoint(0)->setName("renamed joint");
      target->getSkeleton(0)->getJoint(0)->setDofName(0, "renamed dof", true);
      target->restoreState(state);
      expectEqual(state, target->captureState());
      target->setName("renamed world");
      target->getSkeleton(0)->setName("renamed model");
      target->restoreState(state);
      expectEqual(state, target->captureState());
    }
  }
}

TEST(WorldStateManual, CloneRequiresManualConstraintsBeforeRestore)
{
  for (const bool bodyPair : {false, true}) {
    SCOPED_TRACE(bodyPair);
    auto source = makeWorld("source");
    source->getSkeleton(0)->setPosition(0, 0.3);
    source->getSkeleton(1)->setVelocity(5, 0.4);
    source->setTime(8.5);
    addConstraint(source, ManualKind::Weld, bodyPair);
    const auto state = source->captureState();
    auto clone = source->clone();
    ASSERT_EQ(clone->getConstraintSolver()->getNumConstraints(), 0);
    expectAtomicMismatch(clone, state);

    addConstraint(clone, ManualKind::Weld, bodyPair);
    ASSERT_NO_THROW(clone->restoreState(state));
    expectEqual(state, clone->captureState());
    source->step(false);
    clone->step(false);
    expectEqual(source->captureState(), clone->captureState(), 1e-13);
  }
}

TEST(WorldStateManual, MembershipTypeOrderAndEndpointMismatchAreAtomic)
{
  auto source = makeWorld("source");
  addConstraint(source, ManualKind::Ball, true);
  const auto state = source->captureState();
  auto missing = makeWorld("missing");
  expectAtomicMismatch(missing, state);
  auto extra = makeWorld("extra");
  addConstraint(extra, ManualKind::Ball, true);
  addConstraint(extra, ManualKind::Weld, false);
  expectAtomicMismatch(extra, state);
  auto otherType = makeWorld("type");
  addConstraint(otherType, ManualKind::Revolute, true);
  expectAtomicMismatch(otherType, state);
  auto worldEndpoint = makeWorld("world endpoint");
  addConstraint(worldEndpoint, ManualKind::Ball, false);
  expectAtomicMismatch(worldEndpoint, state);
  for (const bool reverse : {false, true}) {
    auto endpoint = makeWorld("endpoint");
    auto* a = endpoint->getSkeleton(0)->getBodyNode(reverse ? 0 : 1);
    auto* b = endpoint->getSkeleton(1)->getBodyNode(0);
    endpoint->getConstraintSolver()->addConstraint(
        makeConstraint(ManualKind::Ball, reverse ? b : a, reverse ? a : b));
    expectAtomicMismatch(endpoint, state);
  }
  source->getConstraintSolver()->removeAllConstraints();
  auto reordered = makeWorld("order");
  for (int index : {0, 1}) {
    source->getConstraintSolver()->addConstraint(makeConstraint(
        ManualKind::Ball, source->getSkeleton(0)->getBodyNode(index), nullptr));
    reordered->getConstraintSolver()->addConstraint(makeConstraint(
        ManualKind::Ball,
        reordered->getSkeleton(0)->getBodyNode(1 - index),
        nullptr));
  }
  expectAtomicMismatch(reordered, source->captureState());
}

TEST(WorldStateManual, RestoreClearsWarmStartsAndRetainsTargetParameters)
{
  for (const auto kind : kinds) {
    for (const bool bodyPair : {false, true}) {
      SCOPED_TRACE(static_cast<int>(kind));
      SCOPED_TRACE(bodyPair);
      auto source = makeWorld("source");
      auto target = makeWorld("stale target");
      auto fresh = makeWorld("fresh target");
      addConstraint(source, kind, bodyPair);
      addConstraint(target, kind, bodyPair, true);
      addConstraint(fresh, kind, bodyPair, true);
      auto state = source->captureState();
      state.positions.setLinSpaced(0.01, 0.18);
      state.velocities.setLinSpaced(-0.3, 0.4);
      auto recording = std::make_shared<RecordingSolver>();
      auto* solver = dynamic_cast<constraint::BoxedLcpConstraintSolver*>(
          target->getConstraintSolver());
      ASSERT_NE(solver, nullptr);
      solver->setBoxedLcpSolver(recording);
      solver->setSecondaryBoxedLcpSolver(nullptr);
      target->getSkeleton(0)->setVelocity(0, 0.8);
      target->step(false);
      ASSERT_GT(recording->outgoingMax, 1e-9);
      target->step(false);
      ASSERT_GT(recording->incomingMax, 1e-9)
          << "the oracle must observe a previously retained impulse";
      const double retainedWarmStart = recording->outgoingMax;
      ASSERT_GT(retainedWarmStart, 1e-9);
      auto invalid = state;
      invalid.commands[0] = std::numeric_limits<double>::quiet_NaN();
      const auto beforeRejected = target->captureState();
      EXPECT_THROW(target->restoreState(invalid), std::invalid_argument);
      expectEqual(beforeRejected, target->captureState());
      target->step(false);
      EXPECT_DOUBLE_EQ(recording->incomingMax, retainedWarmStart)
          << "rejected restore must retain the target warm start";
      Eigen::Isometry3d weldParameter = Eigen::Isometry3d::Identity();
      if (kind == ManualKind::Weld) {
        weldParameter
            = std::static_pointer_cast<constraint::WeldJointConstraint>(
                  target->getConstraintSolver()->getConstraint(0))
                  ->getRelativeTransform();
      }
      target->restoreState(state);
      fresh->restoreState(state);
      source->restoreState(state);
      expectEqual(state, target->captureState());
      if (kind == ManualKind::Weld) {
        EXPECT_TRUE(weldParameter.matrix().isApprox(
            std::static_pointer_cast<constraint::WeldJointConstraint>(
                target->getConstraintSolver()->getConstraint(0))
                ->getRelativeTransform()
                .matrix(),
            0));
      }
      const int calls = recording->calls;
      target->step(false);
      fresh->step(false);
      source->step(false);
      ASSERT_EQ(recording->calls, calls + 1);
      EXPECT_EQ(recording->incomingMax, 0);
      const auto actual = target->captureState();
      EXPECT_TRUE(actual.positions.allFinite());
      EXPECT_TRUE(actual.velocities.allFinite());
      expectEqual(actual, fresh->captureState(), 1e-13);
      EXPECT_FALSE(
          actual.velocities.isApprox(source->captureState().velocities, 1e-8))
          << "different target anchors/axes must still affect the solve";
    }
  }
}

TEST(WorldStateManual, UnsupportedFamiliesAndUnownedEndpointsRejectAtomically)
{
  auto world = makeWorld("supported topology");
  auto* body = world->getSkeleton(0)->getBodyNode(0);
  expectUnsupported(
      world, std::make_shared<CustomBall>(body, Eigen::Vector3d::Zero()));
  auto foreign = makeWorld("unowned");
  for (const bool firstForeign : {false, true}) {
    expectUnsupported(
        world,
        makeConstraint(
            ManualKind::Ball,
            firstForeign ? foreign->getSkeleton(0)->getBodyNode(0) : body,
            firstForeign ? nullptr : foreign->getSkeleton(1)->getBodyNode(0)));
  }
  auto scalar = Skeleton::create("scalar");
  auto* joint = scalar->createJointAndBodyNodePair<RevoluteJoint>().first;
  auto* reference = scalar->createJointAndBodyNodePair<RevoluteJoint>().first;
  world->addSkeleton(scalar);
  const std::vector<MimicDofProperties> mimic{{reference, 0, 1.0, 0.0}};
  const constraint::ConstraintBasePtr unsupported[]
      = {std::make_shared<constraint::JointConstraint>(joint),
         std::make_shared<constraint::JointLimitConstraint>(joint),
         std::make_shared<constraint::ServoMotorConstraint>(joint),
         std::make_shared<constraint::JointCoulombFrictionConstraint>(joint),
         std::make_shared<constraint::MimicMotorConstraint>(joint, mimic),
         std::make_shared<constraint::CouplerConstraint>(joint, mimic)};
  for (const auto& item : unsupported) {
    SCOPED_TRACE(item->getType());
    expectUnsupported(world, item);
  }
}

TEST(WorldStateManual, ContactConstraintsRemainEphemeralAndUnsupported)
{
  auto world = makeSleepingWorld();
  world->step();
  auto& contacts = world->getConstraintSolver()->getLastCollisionResult();
  ASSERT_GT(contacts.getNumContacts(), 0);
  auto contact = contacts.getContact(0);
  expectUnsupported(
      world,
      std::make_shared<constraint::ContactConstraint>(
          contact, world->getTimeStep(), constraint::ContactSurfaceParams()));
  expectUnsupported(
      world,
      std::make_shared<constraint::SoftContactConstraint>(
          contact, world->getTimeStep()));
}

TEST(
    WorldStateManual, RestoreWakesSleepingBoxWithManualWeldAndInvalidatesCaches)
{
  auto target = makeSleepingWorld();
  auto fresh = makeSleepingWorld();
  for (const auto& world : {target, fresh}) {
    world->getConstraintSolver()->addConstraint(
        std::make_shared<constraint::WeldJointConstraint>(
            world->getSkeleton(0)->getBodyNode(0)));
  }
  auto options = target->getDeactivationOptions();
  options.mEnabled = true;
  options.mTimeUntilSleep = 0.01;
  target->setDeactivationOptions(options);
  auto box = target->getSkeleton(1);
  for (int step = 0; step < 3000 && !box->isResting(); ++step)
    target->step();
  ASSERT_TRUE(box->isResting());
  target->step();
  target->step();
  ASSERT_TRUE(box->isResting());
  auto state = target->captureState();
  state.positions << 0.1, -0.2, 0.3, 0.7, 0.2, 1.0;
  state.velocities << 0.3, 0.2, 0.1, 0.4, 0.5, 0.6;
  static_cast<void>(box->getBodyNode(0)->getWorldTransform());
  static_cast<void>(box->getBodyNode(0)->getSpatialVelocity());
  static_cast<void>(box->getMassMatrix());
  static_cast<void>(box->getCoriolisAndGravityForces());
  target->restoreState(state);
  fresh->restoreState(state);
  EXPECT_FALSE(box->isResting());
  EXPECT_TRUE(box->getBodyNode(0)->getWorldTransform().matrix().isApprox(
      fresh->getSkeleton(1)->getBodyNode(0)->getWorldTransform().matrix(),
      1e-14));
  EXPECT_TRUE(box->getMassMatrix().isApprox(
      fresh->getSkeleton(1)->getMassMatrix(), 1e-14));
  target->step(false);
  fresh->step(false);
  expectEqual(target->captureState(), fresh->captureState(), 1e-13);
}

TEST(WorldStateManual, ReusableCaptureWithManualConstraintsDoesNotAllocate)
{
  auto world = makeWorld("allocations");
  for (const auto kind : kinds) {
    addConstraint(world, kind, false);
    addConstraint(world, kind, true);
  }
  auto output = world->captureState();
  test::ScopedHeapAllocationCounter heap;
  test::ScopedRawHeapAllocationCounter raw;
  for (int i = 0; i < 10; ++i)
    world->captureState(output);
  heap.stop();
  raw.stop();
  EXPECT_EQ(heap.allocationCount(), 0);
  if (!raw.skipped()) {
    EXPECT_EQ(raw.allocationCount(), 0);
  }
}
