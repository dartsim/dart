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
#include "dart/collision/CollisionObject.hpp"
#include "dart/collision/dart/DARTCollisionDetector.hpp"
#include "dart/common/Aspect.hpp"
#include "dart/config.hpp"
#include "dart/constraint/BallJointConstraint.hpp"
#include "dart/constraint/BoxedLcpConstraintSolver.hpp"
#include "dart/constraint/ConstraintSolver.hpp"
#include "dart/constraint/FbfFrictionSolver.hpp"
#include "dart/constraint/NsgsFrictionSolver.hpp"
#include "dart/constraint/PgsBoxedLcpSolver.hpp"
#include "dart/dynamics/BallJoint.hpp"
#include "dart/dynamics/BoxShape.hpp"
#include "dart/dynamics/CapsuleShape.hpp"
#include "dart/dynamics/EndEffector.hpp"
#include "dart/dynamics/EulerJoint.hpp"
#include "dart/dynamics/FreeJoint.hpp"
#include "dart/dynamics/Marker.hpp"
#include "dart/dynamics/PlanarJoint.hpp"
#include "dart/dynamics/PrismaticJoint.hpp"
#include "dart/dynamics/RevoluteJoint.hpp"
#include "dart/dynamics/ScrewJoint.hpp"
#include "dart/dynamics/ShapeNode.hpp"
#include "dart/dynamics/SoftBodyNode.hpp"
#include "dart/dynamics/TranslationalJoint.hpp"
#include "dart/dynamics/TranslationalJoint2D.hpp"
#include "dart/dynamics/UniversalJoint.hpp"
#include "dart/dynamics/WeldJoint.hpp"
#include "dart/simulation/World.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>

using namespace dart;
using namespace dart::dynamics;
using namespace dart::simulation;

namespace {

WorldPtr makeWorld(bool prismatic = false, bool separateRoots = false)
{
  auto world = World::create();
  world->setGravity(Eigen::Vector3d::Zero());
  auto skeleton = Skeleton::create();
  auto* root = skeleton->createJointAndBodyNodePair<RevoluteJoint>().second;
  root->createShapeNodeWith<CollisionAspect, VisualAspect, DynamicsAspect>(
      std::make_shared<BoxShape>(Eigen::Vector3d::Constant(0.1)));
  if (prismatic)
    skeleton->createJointAndBodyNodePair<PrismaticJoint>(
        separateRoots ? nullptr : root);
  else
    skeleton->createJointAndBodyNodePair<RevoluteJoint>(
        separateRoots ? nullptr : root);
  world->addSkeleton(skeleton);
  return world;
}

WorldPtr makeSleepingWorld()
{
  auto world = World::create();
  auto floor = Skeleton::create("floor");
  auto* ground = floor->createJointAndBodyNodePair<WeldJoint>().second;
  ground->createShapeNodeWith<CollisionAspect, DynamicsAspect>(
      std::make_shared<BoxShape>(Eigen::Vector3d(10, 10, 0.1)));
  WeldJoint::Properties floorJoint;
  floorJoint.mT_ParentBodyToJoint.translation().z() = -0.05;
  ground->getParentJoint()->setTransformFromParentBodyNode(
      floorJoint.mT_ParentBodyToJoint);
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

enum class SnapshotSolver
{
  Nsgs,
  Pgs,
  Fbf
};

WorldPtr makeContactWorld(
    SnapshotSolver backend = SnapshotSolver::Nsgs,
    bool splitImpulse = false,
    const std::string& detector = "dart")
{
  auto world = makeSleepingWorld();
  auto* solver = static_cast<constraint::BoxedLcpConstraintSolver*>(
      world->getConstraintSolver());
  solver->setCollisionDetector(
      collision::CollisionDetector::getFactory()->create(detector));
  solver->setSplitImpulseEnabled(splitImpulse);
  constraint::NsgsFrictionSolver::Options options;
  options.maxSweeps = 2;
  solver->setBoxedLcpSolver(
      std::make_shared<constraint::NsgsFrictionSolver>(options));
  solver->setSecondaryBoxedLcpSolver(nullptr);
  if (backend == SnapshotSolver::Fbf) {
    constraint::FbfFrictionSolver::Options fbfOptions;
    fbfOptions.maxOuterIterations = 2;
    solver->setBoxedLcpSolver(
        std::make_shared<constraint::FbfFrictionSolver>(fbfOptions));
  }
  if (backend == SnapshotSolver::Pgs) {
    auto pgs = std::make_shared<constraint::PgsBoxedLcpSolver>();
    // Accept the capped iterate so cold and warm seeds remain observable.
    pgs->setOption(constraint::PgsBoxedLcpSolver::Option(2, 1e-6, 1e6));
    solver->setBoxedLcpSolver(pgs);
    solver->setSecondaryBoxedLcpSolver(nullptr);
    auto limited = Skeleton::create("limited");
    auto* joint = limited->createJointAndBodyNodePair<PrismaticJoint>().first;
    joint->setPositionLowerLimit(0, 0.0);
    joint->setPosition(0, -1e-6);
    joint->setLimitEnforcement(true);
    world->addSkeleton(limited);
  }
  world->getSkeleton(1)->setVelocity(3, 0.02);
  if (detector == "ode") {
    auto box = world->getSkeleton(1);
    box->getBodyNode(0)->getShapeNode(0)->setShape(
        std::make_shared<CapsuleShape>(0.03, 0.04));
    box->setPosition(1, math::constantsd::pi() / 2.0);
    box->setPosition(5, 0.03 - 1e-6);
  }
  return world;
}

void expectFreshTrajectory(
    SnapshotSolver backend = SnapshotSolver::Nsgs,
    bool splitImpulse = false,
    const std::string& detector = "dart")
{
  auto a = makeContactWorld(backend, splitImpulse, detector);
  for (int i = 0; i < 40; ++i)
    a->step();
  ASSERT_GT(a->getLastCollisionResult().getNumContacts(), 0u);
  if (backend == SnapshotSolver::Pgs) {
    ASSERT_GT(a->getSkeleton(2)->getJoint(0)->getConstraintImpulse(0), 0.0);
  }
  const auto state = a->captureState();
  for (int i = 0; i < 30; ++i)
    a->step();
  auto b = makeContactWorld(backend, splitImpulse, detector);
  auto* group = a->getConstraintSolver()->getCollisionGroup().get();
  const auto contentVersion = group->getContentVersion();
  a->restoreState(state);
  b->restoreState(state);
  EXPECT_EQ(a->getConstraintSolver()->getCollisionGroup().get(), group);
  EXPECT_EQ(group->getContentVersion(), contentVersion);
  EXPECT_EQ(a->getLastCollisionResult().getNumContacts(), 0u);
  for (int i = 0; i < 8; ++i) {
    SCOPED_TRACE(i);
    a->step();
    b->step();
    const auto sa = a->captureState();
    const auto sb = b->captureState();
    ASSERT_TRUE((sa.positions.array() == sb.positions.array()).all())
        << "position delta: " << (sa.positions - sb.positions).transpose();
    ASSERT_TRUE((sa.velocities.array() == sb.velocities.array()).all())
        << "velocity delta: " << (sa.velocities - sb.velocities).transpose();
    const auto& ca = a->getLastCollisionResult();
    const auto& cb = b->getLastCollisionResult();
    ASSERT_EQ(ca.getNumContacts(), cb.getNumContacts());
    EXPECT_EQ(ca.isCollision(), cb.isCollision());
    for (std::size_t c = 0; c < ca.getNumContacts(); ++c) {
      const auto& x = ca.getContact(c);
      const auto& y = cb.getContact(c);
      EXPECT_TRUE((x.point.array() == y.point.array()).all());
      EXPECT_TRUE((x.normal.array() == y.normal.array()).all());
      EXPECT_TRUE((x.force.array() == y.force.array()).all());
      EXPECT_EQ(x.penetrationDepth, y.penetrationDepth);
      EXPECT_EQ(
          x.collisionObject1->getBodyNode()->getSkeleton()->getName(),
          y.collisionObject1->getBodyNode()->getSkeleton()->getName());
      EXPECT_EQ(
          x.collisionObject2->getBodyNode()->getSkeleton()->getName(),
          y.collisionObject2->getBodyNode()->getSkeleton()->getName());
      EXPECT_EQ(
          x.collisionObject1->getBodyNode()->getName(),
          y.collisionObject1->getBodyNode()->getName());
      EXPECT_EQ(
          x.collisionObject2->getBodyNode()->getName(),
          y.collisionObject2->getBodyNode()->getName());
    }
  }
}

template <typename JointType = WeldJoint>
WorldPtr makeSoftWorld()
{
  auto world = World::create();
  world->setGravity(Eigen::Vector3d::Zero());
  auto skeleton = Skeleton::create();
  SoftBodyNode::Properties properties(
      BodyNode::Properties(),
      SoftBodyNodeHelper::makeSinglePointMassProperties(1.0, 0.0, 0.0, 0.0));
  skeleton->createJointAndBodyNodePair<JointType, SoftBodyNode>(
      nullptr, typename JointType::Properties(), properties);
  world->addSkeleton(skeleton);
  return world;
}

void expectEqual(const WorldState& a, const WorldState& b)
{
  EXPECT_TRUE(a.positions.isApprox(b.positions, 0));
  EXPECT_TRUE(a.velocities.isApprox(b.velocities, 0));
  EXPECT_TRUE(a.accelerations.isApprox(b.accelerations, 0));
  EXPECT_TRUE(a.forces.isApprox(b.forces, 0));
  EXPECT_TRUE(a.commands.isApprox(b.commands, 0));
  EXPECT_TRUE(a.bodyExternalForces.isApprox(b.bodyExternalForces, 0));
  EXPECT_EQ(a.time, b.time);
  ASSERT_EQ(a.softBodies.size(), b.softBodies.size());
  for (std::size_t s = 0; s < a.softBodies.size(); ++s) {
    ASSERT_EQ(a.softBodies[s].points.size(), b.softBodies[s].points.size());
    for (std::size_t p = 0; p < a.softBodies[s].points.size(); ++p) {
      EXPECT_TRUE(a.softBodies[s].points[p] == b.softBodies[s].points[p]);
      EXPECT_TRUE(a.softBodies[s].externalForces[p].isApprox(
          b.softBodies[s].externalForces[p], 0));
    }
  }
}

class ExternalCommandJoint : public RevoluteJoint
{
public:
  explicit ExternalCommandJoint(const Properties& properties)
    : RevoluteJoint(properties)
  {
  }
  double getCommand(std::size_t) const override
  {
    return 99.0;
  }
};

struct ExtensionState
{
  double value = 42;
};
class StatefulAspect : public common::Aspect
{
public:
  std::unique_ptr<common::Aspect> cloneAspect() const override
  {
    return std::make_unique<StatefulAspect>();
  }
  const State* getAspectState() const override
  {
    return &mState;
  }

private:
  common::Aspect::MakeState<ExtensionState> mState;
};

} // namespace

TEST(WorldState, OriginCloneIndependentWorldAndCosmeticNames)
{
  auto source = makeWorld();
  auto skeleton = source->getSkeleton(0);
  skeleton->setPosition(0, 0.4);
  skeleton->setVelocity(1, -0.3);
  skeleton->setAcceleration(0, 0.6);
  skeleton->setForce(0, 0.8);
  skeleton->setCommand(1, 0.9);
  skeleton->getBodyNode(1)->addExtForce(
      Eigen::Vector3d(1, 2, 3), Eigen::Vector3d(0.2, 0, 0), true, true);
  source->setTime(12.5);
  const auto state = source->captureState();
  auto clone = source->clone();
  auto independent = makeWorld();
  independent->setName("cosmetic world");
  independent->getSkeleton(0)->setName("cosmetic model");
  independent->getSkeleton(0)->getBodyNode(0)->setName("cosmetic body");
  independent->getSkeleton(0)->getJoint(0)->setName("cosmetic joint");
  independent->getSkeleton(0)->getJoint(0)->setDofName(0, "cosmetic dof", true);
  for (const auto& target : {source, clone, independent}) {
    target->getSkeleton(0)->setPositions(Eigen::Vector2d(-0.5, 0.7));
    static_cast<void>(
        target->getSkeleton(0)->getBodyNode(1)->getWorldTransform());
    target->restoreState(state);
    expectEqual(state, target->captureState());
    EXPECT_TRUE(skeleton->getBodyNode(1)->getWorldTransform().matrix().isApprox(
        target->getSkeleton(0)->getBodyNode(1)->getWorldTransform().matrix(),
        1e-14));
  }
  skeleton->setPosition(0, 3.0);
  EXPECT_EQ(state.positions[0], 0.4) << "snapshot owns its values";
}

TEST(WorldState, ParametersAndActuatorLimitsRemainTargetOwned)
{
  auto source = makeWorld();
  auto state = source->captureState();
  state.positions << 0.3, 0.4;
  state.velocities << 0.5, 0.6;
  state.accelerations << 0.7, 0.8;
  state.forces << 9.0, 10.0;
  state.commands << 11.0, 12.0;
  auto target = makeWorld();
  const auto detector = collision::DARTCollisionDetector::create();
  target->getConstraintSolver()->setCollisionDetector(detector);
  target->setTimeStep(0.002);
  target->setGravity(Eigen::Vector3d(1, 2, 3));
  auto* joint = target->getSkeleton(0)->getJoint(0);
  joint->setActuatorType(Joint::VELOCITY);
  joint->setVelocityUpperLimit(0, 0.1);
  joint->setForceUpperLimit(0, 0.2);
  target->getSkeleton(0)->getBodyNode(0)->setMass(2.0);
  target->restoreState(state);
  auto expected = state;
  expected.commands[0] = 0.1;
  expectEqual(expected, target->captureState());
  EXPECT_EQ(joint->getActuatorType(), Joint::VELOCITY);
  EXPECT_EQ(joint->getVelocityUpperLimit(0), 0.1);
  EXPECT_EQ(target->getSkeleton(0)->getBodyNode(0)->getMass(), 2.0);
  EXPECT_EQ(target->getTimeStep(), 0.002);
  EXPECT_EQ(target->getConstraintSolver()->getCollisionDetector(), detector);
  EXPECT_TRUE(target->getGravity().isApprox(Eigen::Vector3d(1, 2, 3), 0));
}

TEST(WorldState, RestoredCommandsFollowPerDofActuatorLimits)
{
  auto world = World::create();
  auto skeleton = Skeleton::create();
  auto* joint = skeleton->createJointAndBodyNodePair<FreeJoint>().first;
  world->addSkeleton(skeleton);
  auto state = world->captureState();
  state.commands << 10, -10, 10, -10, 10, -10;
  const Joint::ActuatorType types[]
      = {Joint::FORCE, Joint::SERVO, Joint::VELOCITY, Joint::ACCELERATION};
  for (std::size_t d = 0; d < 6; ++d) {
    joint->setForceLowerLimit(d, -0.2 * (d + 1));
    joint->setForceUpperLimit(d, 0.2 * (d + 1));
    joint->setVelocityLowerLimit(d, -0.3 * (d + 1));
    joint->setVelocityUpperLimit(d, 0.3 * (d + 1));
    joint->setAccelerationLowerLimit(d, -0.4 * (d + 1));
    joint->setAccelerationUpperLimit(d, 0.4 * (d + 1));
  }
  for (const auto type : types) {
    SCOPED_TRACE(static_cast<int>(type));
    joint->setActuatorType(type);
    world->restoreState(state);
    auto expected = state;
    const double limit = type == Joint::FORCE          ? 0.2
                         : type == Joint::ACCELERATION ? 0.4
                                                       : 0.3;
    for (Eigen::Index d = 0; d < 6; ++d)
      expected.commands[d] = (d % 2 == 0 ? 1 : -1) * limit * (d + 1);
    expectEqual(expected, world->captureState());
  }
  joint->setActuatorType(Joint::FORCE);
  joint->setActuatorType(5, Joint::MIMIC);
  world->restoreState(state);
  auto expected = state;
  for (Eigen::Index d = 0; d < 6; ++d)
    expected.commands[d] = (d % 2 == 0 ? 1 : -1) * 0.2 * (d + 1);
  expected.commands[5] = 0;
  expectEqual(expected, world->captureState());
}

TEST(WorldState, RestoredCommandsMatchSetCommandForEveryActuatorType)
{
  // Restore applies setCommand()'s per-actuator rule through its own helper,
  // so both must give the same command for every type and value.
  auto world = World::create();
  auto skeleton = Skeleton::create();
  auto* joint = skeleton->createJointAndBodyNodePair<RevoluteJoint>().first;
  world->addSkeleton(skeleton);
  joint->setForceLowerLimit(0, -0.2);
  joint->setForceUpperLimit(0, 0.25);
  joint->setVelocityLowerLimit(0, -0.3);
  joint->setVelocityUpperLimit(0, 0.35);
  joint->setAccelerationLowerLimit(0, -0.4);
  joint->setAccelerationUpperLimit(0, 0.45);
  const Joint::ActuatorType types[]
      = {Joint::FORCE,
         Joint::PASSIVE,
         Joint::SERVO,
         Joint::MIMIC,
         Joint::ACCELERATION,
         Joint::VELOCITY,
         Joint::LOCKED};
  for (const auto type : types) {
    joint->setActuatorType(type);
    for (const double command : {-10.0, -0.1, 0.0, 0.1, 10.0}) {
      SCOPED_TRACE(
          std::to_string(static_cast<int>(type)) + " "
          + std::to_string(command));
      joint->resetCommands();
      joint->setCommand(0, command);
      const double expected = joint->getCommand(0);
      auto state = world->captureState();
      state.commands[0] = command;
      joint->resetCommands();
      world->restoreState(state);
      EXPECT_EQ(joint->getCommand(0), expected);
    }
  }
}

TEST(WorldState, ForceCommandsRespectLimitsOnNextStep)
{
  auto source = makeWorld();
  source->getSkeleton(0)->setCommand(0, 11.0);
  auto target = makeWorld();
  auto* joint = target->getSkeleton(0)->getJoint(0);
  joint->setForceUpperLimit(0, 0.2);
  target->restoreState(source->captureState());
  EXPECT_EQ(joint->getCommand(0), 0.2);
  target->step(false);
  EXPECT_EQ(joint->getCommand(0), 0.2);
  EXPECT_EQ(joint->getForce(0), 0.2);
  EXPECT_TRUE(target->getSkeleton(0)->getVelocities().allFinite());
}

TEST(WorldState, PassiveTargetDiscardsRestoredCommands)
{
  auto source = makeWorld();
  source->getSkeleton(0)->setCommand(0, 11.0);
  const auto state = source->captureState();
  for (const auto type : {Joint::PASSIVE, Joint::MIMIC, Joint::LOCKED}) {
    auto target = makeWorld();
    auto* joint = target->getSkeleton(0)->getJoint(0);
    joint->setActuatorType(type);
    target->restoreState(state);
    EXPECT_EQ(joint->getCommand(0), 0.0);
    EXPECT_EQ(joint->getActuatorType(), type);
  }
}

TEST(WorldState, EndEffectorSupportAndMarkerRemainTargetOwned)
{
  auto source = makeWorld();
  auto* sourceBody = source->getSkeleton(0)->getBodyNode(0);
  auto* sourceSupport = sourceBody->createEndEffector()->createSupport();
  sourceSupport->setActive(true);
  sourceBody->createMarker("source marker", Eigen::Vector3d(1, 2, 3));
  source->getSkeleton(0)->setPosition(0, 0.4);
  const auto state = source->captureState();
  auto target = makeWorld();
  auto* targetBody = target->getSkeleton(0)->getBodyNode(0);
  auto* targetSupport = targetBody->createEndEffector()->createSupport();
  auto* marker
      = targetBody->createMarker("target marker", Eigen::Vector3d::Zero());
  for (const bool active : {false, true}) {
    targetSupport->setActive(active);
    sourceSupport->setActive(!active);
    target->restoreState(source->captureState());
    EXPECT_EQ(targetSupport->isActive(), active);
    EXPECT_TRUE(marker->getLocalPosition().isZero(0));
    expectEqual(state, target->captureState());
  }
}

TEST(WorldState, ImmobileSkeletonRestoresPositionsWithoutWaking)
{
  auto source = makeWorld();
  source->getSkeleton(0)->setPositions(Eigen::Vector2d(0.3, -0.4));
  const auto state = source->captureState();
  auto target = makeWorld();
  auto skeleton = target->getSkeleton(0);
  skeleton->setMobile(false);
  skeleton->setResting(true);
  static_cast<void>(skeleton->getBodyNode(1)->getWorldTransform());
  target->restoreState(state);
  expectEqual(state, target->captureState());
  EXPECT_FALSE(skeleton->isMobile());
  EXPECT_TRUE(skeleton->isResting());
  EXPECT_TRUE(skeleton->getBodyNode(1)->getWorldTransform().matrix().isApprox(
      source->getSkeleton(0)->getBodyNode(1)->getWorldTransform().matrix(),
      1e-14));
}

TEST(WorldState, TopologyAndMalformedChannelsRejectBeforeEffects)
{
  auto source = makeWorld();
  const auto state = source->captureState();
  for (auto target : {makeWorld(true), makeWorld(false, true)}) {
    target->setTime(8.0);
    const auto before = target->captureState();
    EXPECT_THROW(target->restoreState(state), std::invalid_argument);
    expectEqual(before, target->captureState());
  }
  auto extra = makeWorld();
  auto weld = Skeleton::create();
  weld->createJointAndBodyNodePair<WeldJoint>();
  extra->addSkeleton(weld);
  const auto beforeExtra = extra->captureState();
  EXPECT_THROW(extra->restoreState(state), std::invalid_argument);
  expectEqual(beforeExtra, extra->captureState());
  for (int kind = 0; kind < 5; ++kind) {
    auto bad = state;
    if (kind == 0)
      bad.commands.conservativeResize(1);
    if (kind == 1)
      bad.forces[1] = std::numeric_limits<double>::quiet_NaN();
    if (kind == 2)
      bad.time = std::numeric_limits<double>::infinity();
    if (kind == 3)
      bad.bodyExternalForces(0, 1) = std::numeric_limits<double>::infinity();
    if (kind == 4)
      bad.commands[0] = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(source->restoreState(bad), std::invalid_argument);
    expectEqual(state, source->captureState());
  }
  EXPECT_THROW(source->restoreState(WorldState()), std::invalid_argument);
}

TEST(WorldState, CaptureRejectsInvalidOutputWithoutChangingIt)
{
  auto world = makeWorld();
  auto output = world->captureState();
  output.positions.conservativeResize(1);
  output.time = 5;
  world->setTime(10);
  EXPECT_THROW(world->captureState(output), std::invalid_argument);
  EXPECT_EQ(output.positions.size(), 1);
  EXPECT_EQ(output.time, 5);
  auto incompatible = makeWorld(true);
  auto other = incompatible->captureState();
  const auto before = other;
  EXPECT_THROW(world->captureState(other), std::invalid_argument);
  expectEqual(before, other);
  auto finite = world->captureState();
  const auto beforeNonFinite = finite;
  world->setTime(std::numeric_limits<double>::infinity());
  EXPECT_THROW(world->captureState(finite), std::invalid_argument);
  expectEqual(beforeNonFinite, finite);
}

TEST(WorldState, CustomJointAndStatefulAspectsRejectCaptureAndRestore)
{
  auto custom = World::create();
  auto skeleton = Skeleton::create();
  skeleton->createJointAndBodyNodePair<ExternalCommandJoint>();
  custom->addSkeleton(skeleton);
  EXPECT_THROW(custom->captureState(), std::invalid_argument);
  auto world = makeWorld();
  const auto state = world->captureState();
  world->getSkeleton(0)->getBodyNode(0)->createAspect<StatefulAspect>();
  EXPECT_THROW(world->captureState(), std::invalid_argument);
  EXPECT_THROW(world->restoreState(state), std::invalid_argument);
  EXPECT_EQ(world->getTime(), state.time);
  EXPECT_TRUE(
      world->getSkeleton(0)->getPositions().isApprox(state.positions, 0));
}

TEST(WorldState, ManualConstraintMembershipChangesRejectBeforeEffects)
{
  auto world = makeWorld();
  const auto state = world->captureState();
  world->getConstraintSolver()->addConstraint(
      std::make_shared<constraint::BallJointConstraint>(
          world->getSkeleton(0)->getBodyNode(0), Eigen::Vector3d::Zero()));
  EXPECT_NO_THROW(world->captureState());
  EXPECT_THROW(world->restoreState(state), std::invalid_argument);
  EXPECT_EQ(world->getTime(), state.time);
  EXPECT_TRUE(
      world->getSkeleton(0)->getPositions().isApprox(state.positions, 0));
}

TEST(WorldState, AllBuiltInJointSpacesRoundTrip)
{
  auto world = World::create();
  auto skeleton = Skeleton::create();
  skeleton->createJointAndBodyNodePair<BallJoint>();
  skeleton->createJointAndBodyNodePair<EulerJoint>();
  skeleton->createJointAndBodyNodePair<FreeJoint>();
  skeleton->createJointAndBodyNodePair<PlanarJoint>();
  skeleton->createJointAndBodyNodePair<PrismaticJoint>();
  skeleton->createJointAndBodyNodePair<RevoluteJoint>();
  skeleton->createJointAndBodyNodePair<ScrewJoint>();
  skeleton->createJointAndBodyNodePair<TranslationalJoint>();
  skeleton->createJointAndBodyNodePair<TranslationalJoint2D>();
  skeleton->createJointAndBodyNodePair<UniversalJoint>();
  skeleton->createJointAndBodyNodePair<WeldJoint>();
  world->addSkeleton(skeleton);
  auto state = world->captureState();
  for (Eigen::Index d = 0; d < state.positions.size(); ++d) {
    state.positions[d] = 0.01 * (d + 1);
    state.velocities[d] = 0.02 * (d + 1);
    state.accelerations[d] = 0.03 * (d + 1);
    state.forces[d] = 0.04 * (d + 1);
    state.commands[d] = 0.05 * (d + 1);
  }
  world->restoreState(state);
  expectEqual(state, world->captureState());
}

TEST(WorldState, RestoredSleepDecisionsMatchFreshWorld)
{
  // Rest detection smooths body speeds across steps and credits bodies that
  // start a run at rest, so a restored World must restart both like a new
  // World restoring the same snapshot.
  const auto makeWorld = [] {
    auto world = makeSleepingWorld();
    world->getConstraintSolver()->setCollisionDetector(
        collision::DARTCollisionDetector::create());
    auto options = world->getDeactivationOptions();
    options.mEnabled = true;
    options.mTimeUntilSleep = 0.01;
    world->setDeactivationOptions(options);
    return world;
  };
  auto a = makeWorld();
  for (int i = 0; i < 20; ++i)
    a->step();
  const auto state = a->captureState();
  a->getSkeleton(1)->setVelocity(3, 2.0);
  for (int i = 0; i < 5; ++i)
    a->step();
  ASSERT_GT(a->getSkeleton(1)->getSmoothedLinearSpeed(), 0.0);
  auto b = makeWorld();
  a->restoreState(state);
  b->restoreState(state);
  bool rested = false;
  for (int i = 0; i < 3000 && !rested; ++i) {
    SCOPED_TRACE(i);
    a->step();
    b->step();
    ASSERT_EQ(a->getSkeleton(1)->isResting(), b->getSkeleton(1)->isResting());
    const auto sa = a->captureState();
    const auto sb = b->captureState();
    ASSERT_TRUE((sa.positions.array() == sb.positions.array()).all());
    ASSERT_TRUE((sa.velocities.array() == sb.velocities.array()).all());
    rested = b->getSkeleton(1)->isResting();
  }
  EXPECT_TRUE(rested);
}

TEST(WorldState, RestoreInvalidatesWarmedCachesAndSleepingFastPath)
{
  auto world = makeSleepingWorld();
  auto control = makeSleepingWorld();
  auto options = world->getDeactivationOptions();
  options.mEnabled = true;
  options.mTimeUntilSleep = 0.01;
  world->setDeactivationOptions(options);
  auto skeleton = world->getSkeleton(1);
  for (int step = 0; step < 3000 && !skeleton->isResting(); ++step)
    world->step();
  ASSERT_TRUE(skeleton->isResting());
  world->step();
  world->step();
  ASSERT_TRUE(skeleton->isResting());
  auto state = world->captureState();
  state.positions << 0.1, -0.2, 0.3, 0.7, 0.2, 1.0;
  state.velocities << 0.3, 0.2, 0.1, 0.4, 0.5, 0.6;
  state.commands.setConstant(0.4);
  static_cast<void>(skeleton->getBodyNode(0)->getWorldTransform());
  static_cast<void>(skeleton->getBodyNode(0)->getSpatialVelocity());
  static_cast<void>(skeleton->getMassMatrix());
  static_cast<void>(skeleton->getCoriolisAndGravityForces());
  world->restoreState(state);
  control->restoreState(state);
  EXPECT_FALSE(skeleton->isResting());
  EXPECT_TRUE(skeleton->getBodyNode(0)->getWorldTransform().matrix().isApprox(
      control->getSkeleton(1)->getBodyNode(0)->getWorldTransform().matrix(),
      1e-14));
  EXPECT_TRUE(skeleton->getMassMatrix().isApprox(
      control->getSkeleton(1)->getMassMatrix(), 1e-14));
  world->step(false);
  control->step(false);
  expectEqual(world->captureState(), control->captureState());
}

TEST(WorldState, SoftPointInputsSurviveFrozenActivationAndNextStep)
{
  auto target = makeSoftWorld();
  auto control = makeSoftWorld();
  auto* soft = target->getSkeleton(0)->getSoftBodyNode(0);
  soft->setAdaptiveContactActivationEnabled(true);
  soft->setAdaptiveContactActivationLingerSteps(0);
  target->step();
  target->step();
  ASSERT_EQ(soft->getNumActivePointMasses(), 0u);
  auto state = control->captureState();
  state.softBodies[0].points[0].mPositions = Eigen::Vector3d(0.2, 0.3, 0.4);
  state.softBodies[0].points[0].mVelocities = Eigen::Vector3d(0.5, 0.6, 0.7);
  state.softBodies[0].points[0].mAccelerations = Eigen::Vector3d(0.8, 0.9, 1.0);
  state.softBodies[0].points[0].mForces = Eigen::Vector3d(1.1, 1.2, 1.3);
  state.softBodies[0].externalForces[0] = Eigen::Vector3d(2, 3, 4);
  static_cast<void>(soft->getPointMass(0)->getWorldPosition());
  target->restoreState(state);
  control->restoreState(state);
  expectEqual(state, target->captureState());
  auto clone = target->clone();
  clone->restoreState(state);
  expectEqual(state, clone->captureState());
  EXPECT_EQ(soft->getNumActivePointMasses(), 1u);
  target->step(false);
  control->step(false);
  expectEqual(target->captureState(), control->captureState());
  EXPECT_GT(soft->getPointMass(0)->getVelocities().x(), 0.5);
  auto bad = state;
  bad.softBodies[0].externalForces.clear();
  const auto before = target->captureState();
  EXPECT_THROW(target->restoreState(bad), std::invalid_argument);
  expectEqual(before, target->captureState());
  bad = state;
  bad.softBodies[0].points[0].mPositions.x()
      = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(target->restoreState(bad), std::invalid_argument);
  expectEqual(before, target->captureState());
}

TEST(WorldState, SoftFreeJointRoundTripMatchesNextStep)
{
  auto control = makeSoftWorld<FreeJoint>();
  auto controlSkeleton = control->getSkeleton(0);
  controlSkeleton->setPositions(
      (Eigen::Vector6d() << 0.1, -0.2, 0.3, 0.4, -0.5, 0.6).finished());
  controlSkeleton->setVelocities(Eigen::Vector6d::Constant(0.2));
  controlSkeleton->setCommands(Eigen::Vector6d::Constant(0.3));
  auto* controlSoft = controlSkeleton->getSoftBodyNode(0);
  auto* point = controlSoft->getPointMass(0);
  point->setPositions(Eigen::Vector3d(0.2, 0.3, 0.4));
  point->setVelocities(Eigen::Vector3d(0.5, 0.6, 0.7));
  point->setAccelerations(Eigen::Vector3d(0.8, 0.9, 1.0));
  point->setForces(Eigen::Vector3d(1.1, 1.2, 1.3));
  point->addExtForce(Eigen::Vector3d(2, 3, 4), true);
  controlSoft->addExtForce(
      Eigen::Vector3d(5, 6, 7), Eigen::Vector3d::Zero(), true);
  const auto state = control->captureState();
  auto target = makeSoftWorld<FreeJoint>();
  auto targetSkeleton = target->getSkeleton(0);
  targetSkeleton->setPositions(Eigen::Vector6d::Constant(-0.1));
  auto* targetSoft = targetSkeleton->getSoftBodyNode(0);
  static_cast<void>(targetSoft->getPointMass(0)->getWorldPosition());
  static_cast<void>(targetSkeleton->getMassMatrix());
  static_cast<void>(targetSkeleton->getExternalForces());
  target->restoreState(state);
  expectEqual(state, target->captureState());
  EXPECT_TRUE(targetSkeleton->getExternalForces().isApprox(
      controlSkeleton->getExternalForces(), 1e-14));
  target->step(false);
  control->step(false);
  expectEqual(control->captureState(), target->captureState());
}

TEST(WorldState, SoftConnectivityRejectsEqualSizedState)
{
  auto source = makeSoftWorld();
  auto target = makeSoftWorld();
  auto* a = source->getSkeleton(0)->getSoftBodyNode(0);
  auto* b = target->getSkeleton(0)->getSoftBodyNode(0);
  a->addPointMass(PointMass::Properties(Eigen::Vector3d(1, 0, 0), 1.0));
  b->addPointMass(PointMass::Properties(Eigen::Vector3d(1, 0, 0), 1.0));
  b->connectPointMasses(0, 1);
  const auto before = target->captureState();
  EXPECT_THROW(
      target->restoreState(source->captureState()), std::invalid_argument);
  expectEqual(before, target->captureState());
}

TEST(WorldState, ReusableGatherDoesNotAllocate)
{
  auto world = makeWorld();
  world->addSkeleton(makeSoftWorld()->getSkeleton(0));
  auto output = world->captureState();
  test::ScopedHeapAllocationCounter heap;
  test::ScopedRawHeapAllocationCounter raw;
  for (int i = 0; i < 10; ++i)
    world->captureState(output);
  heap.stop();
  raw.stop();
  EXPECT_EQ(heap.allocationCount(), 0u);
  if (!raw.skipped()) {
    EXPECT_EQ(raw.allocationCount(), 0u);
  }
}

TEST(WorldState, RestoreClearsPersistentContactHistory)
{
  expectFreshTrajectory();
}

TEST(WorldState, RestoreClearsIterativeContactHistoryWithJointLimits)
{
  expectFreshTrajectory(SnapshotSolver::Pgs);
}

TEST(WorldState, RestoreClearsSplitImpulseHistory)
{
  expectFreshTrajectory(SnapshotSolver::Nsgs, true);
}

TEST(WorldState, RestoreClearsFbfContactHistory)
{
  expectFreshTrajectory(SnapshotSolver::Fbf);
}

TEST(WorldState, RestoreClearsFclContactHistory)
{
  expectFreshTrajectory(SnapshotSolver::Nsgs, false, "fcl");
}

#if HAVE_BULLET
TEST(WorldState, RestoreClearsBulletContactHistory)
{
  expectFreshTrajectory(SnapshotSolver::Nsgs, false, "bullet");
}
#endif

#if HAVE_ODE
TEST(WorldState, RestoreClearsOdeContactHistory)
{
  expectFreshTrajectory(SnapshotSolver::Nsgs, false, "ode");
}
#endif

TEST(WorldState, RestoreDoesNotAllocate)
{
  const auto measure = [](const WorldPtr& world) {
    const auto state = world->captureState();
    test::ScopedHeapAllocationCounter heap;
    test::ScopedRawHeapAllocationCounter raw;
    for (int i = 0; i < 10; ++i)
      world->restoreState(state);
    heap.stop();
    raw.stop();
    EXPECT_EQ(heap.allocationCount(), 0u);
    if (!raw.skipped()) {
      EXPECT_EQ(raw.allocationCount(), 0u);
    }
  };
  for (const auto* detector : {
         "dart", "fcl",
#if HAVE_BULLET
             "bullet",
#endif
#if HAVE_ODE
             "ode",
#endif
       }) {
    SCOPED_TRACE(detector);
    auto world = makeContactWorld(SnapshotSolver::Nsgs, false, detector);
    for (int i = 0; i < 40; ++i)
      world->step();
    measure(world);
  }
  measure(makeSoftWorld());
}

TEST(WorldState, InvalidRestorePreservesContactHistory)
{
  auto world = makeContactWorld();
  auto control = makeContactWorld();
  for (int i = 0; i < 40; ++i) {
    world->step();
    control->step();
  }
  const auto before = world->captureState();
  const auto contacts = world->getLastCollisionResult().getNumContacts();
  ASSERT_GT(contacts, 0u);
  auto invalid = before;
  invalid.positions[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(world->restoreState(invalid), std::invalid_argument);
  expectEqual(before, world->captureState());
  EXPECT_EQ(world->getLastCollisionResult().getNumContacts(), contacts);
  world->step();
  control->step();
  expectEqual(world->captureState(), control->captureState());
}
