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

// Integration tests for automatic island deactivation ("sleeping"). A solver
// island that has come to rest is frozen: its forward dynamics, its constraint
// solve, and its position integration are all skipped together each step until
// it is disturbed. These tests exercise the correctness invariants: atomic
// freeze, island-level granularity, wake-on-contact, wake-on-force, and the
// no-op-when-disabled guarantee.

#include "dart/simulation/DeactivationOptions.hpp"

#include <dart/dart.hpp>

#include <TestHelpers.hpp>
#include <gtest/gtest.h>

#include <chrono>
#include <iostream>

#include <cmath>

using namespace dart;
using namespace dart::dynamics;
using namespace dart::simulation;

namespace {

class ExposedConstraintSolver final : public constraint::ConstraintSolver
{
public:
  void solveGroupsForTest()
  {
    solveConstrainedGroups();
  }

protected:
  void solveConstrainedGroup(constraint::ConstrainedGroup&) override {}
};

//==============================================================================
// Builds a thin static floor (immobile) centered so its top surface sits at
// z = 0.
SkeletonPtr createFloor()
{
  auto floor = Skeleton::create("floor");
  auto body = floor->createJointAndBodyNodePair<WeldJoint>(nullptr).second;

  const double width = 10.0;
  const double thickness = 0.1;
  auto shape
      = std::make_shared<BoxShape>(Eigen::Vector3d(width, width, thickness));
  body->createShapeNodeWith<CollisionAspect, DynamicsAspect>(shape);

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(0.0, 0.0, -thickness / 2.0);
  body->getParentJoint()->setTransformFromParentBodyNode(tf);

  // A static floor is immobile, so it is never a candidate for sleeping and
  // never adds itself to a solver island as an awake member.
  floor->setMobile(false);
  return floor;
}

//==============================================================================
// Builds a free-floating unit-density box of the given half-extent-ish size,
// placed at the given center.
SkeletonPtr createFreeBox(
    const std::string& name,
    const Eigen::Vector3d& size,
    const Eigen::Vector3d& position)
{
  auto skel = Skeleton::create(name);

  GenericJoint<SE3Space>::Properties jointProps(std::string(name + "_joint"));
  BodyNode::Properties bodyProps(
      BodyNode::AspectProperties(std::string(name + "_body")));
  bodyProps.mInertia.setMass(1.0);

  auto pair = skel->createJointAndBodyNodePair<FreeJoint>(
      nullptr, jointProps, bodyProps);
  auto* body = pair.second;

  auto shape = std::make_shared<BoxShape>(size);
  body->createShapeNodeWith<CollisionAspect, DynamicsAspect>(shape);

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = position;
  skel->getJoint(0)->setPositions(FreeJoint::convertToPositions(tf));

  return skel;
}

//==============================================================================
// Builds a one-link articulated body with a revolute root joint and a box link
// whose bottom face rests on the floor. Unlike a single free body, this exposes
// the solved contact load through BodyNode::getBodyForce(), matching the joint
// transmitted-wrench path used by downstream adapters.
SkeletonPtr createPinnedContactArm(const std::string& name)
{
  auto skel = Skeleton::create(name);

  RevoluteJoint::Properties jointProps;
  jointProps.mName = name + "_joint";
  jointProps.mAxis = Eigen::Vector3d::UnitY();
  jointProps.mT_ParentBodyToJoint.translation()
      = Eigen::Vector3d(0, 0, 0.999999);
  jointProps.mT_ChildBodyToJoint.translation() = Eigen::Vector3d(0, 0, 0.5);

  BodyNode::Properties bodyProps(
      BodyNode::AspectProperties(std::string(name + "_body")));
  bodyProps.mInertia.setMass(1.0);

  auto* body = skel->createJointAndBodyNodePair<RevoluteJoint>(
                       nullptr, jointProps, bodyProps)
                   .second;

  auto shape = std::make_shared<BoxShape>(Eigen::Vector3d(0.2, 0.2, 1.0));
  body->createShapeNodeWith<CollisionAspect, DynamicsAspect>(shape);

  return skel;
}

//==============================================================================
struct ServoWheelCart
{
  SkeletonPtr skel;
  BodyNode* chassis{nullptr};
  RevoluteJoint* leftWheel{nullptr};
  RevoluteJoint* rightWheel{nullptr};
};

//==============================================================================
ServoWheelCart createServoWheelCart(const std::string& name)
{
  ServoWheelCart cart;
  cart.skel = Skeleton::create(name);

  FreeJoint::Properties rootProps;
  rootProps.mName = name + "_root";
  BodyNode::Properties chassisProps(
      BodyNode::AspectProperties(std::string(name + "_chassis")));
  chassisProps.mInertia.setMass(3.0);

  auto rootPair = cart.skel->createJointAndBodyNodePair<FreeJoint>(
      nullptr, rootProps, chassisProps);
  auto* root = rootPair.first;
  cart.chassis = rootPair.second;

  auto chassisShape
      = std::make_shared<BoxShape>(Eigen::Vector3d(1.0, 0.5, 0.2));
  cart.chassis->createShapeNodeWith<CollisionAspect, DynamicsAspect>(
      chassisShape);

  Eigen::Isometry3d rootTf = Eigen::Isometry3d::Identity();
  rootTf.translation() = Eigen::Vector3d(0.0, 0.0, 0.475);
  root->setPositions(FreeJoint::convertToPositions(rootTf));

  auto addWheel = [&](const std::string& wheelName, double y) {
    RevoluteJoint::Properties jointProps;
    jointProps.mName = name + "_" + wheelName + "_joint";
    jointProps.mAxis = Eigen::Vector3d::UnitY();
    jointProps.mT_ParentBodyToJoint.translation()
        = Eigen::Vector3d(0.25, y, -0.175);

    BodyNode::Properties wheelProps(
        BodyNode::AspectProperties(std::string(name + "_" + wheelName)));
    wheelProps.mInertia.setMass(1.0);

    auto pair = cart.chassis->createChildJointAndBodyNodePair<RevoluteJoint>(
        jointProps, wheelProps);
    auto* joint = pair.first;
    auto* wheel = pair.second;

    auto wheelShape = std::make_shared<SphereShape>(0.3);
    wheel->createShapeNodeWith<CollisionAspect, DynamicsAspect>(wheelShape);

    joint->setForceLowerLimit(0, -1000.0);
    joint->setForceUpperLimit(0, 1000.0);
    joint->setVelocityLowerLimit(0, -100.0);
    joint->setVelocityUpperLimit(0, 100.0);

    return joint;
  };

  cart.leftWheel = addWheel("left_wheel", 0.35);
  cart.rightWheel = addWheel("right_wheel", -0.35);

  return cart;
}

//==============================================================================
// Steps the world until the predicate is satisfied or maxSteps is exceeded.
// Returns the number of steps taken (== maxSteps if never satisfied).
template <typename Predicate>
std::size_t stepUntil(World* world, std::size_t maxSteps, Predicate pred)
{
  for (std::size_t i = 0; i < maxSteps; ++i) {
    if (pred())
      return i;
    world->step();
  }
  return maxSteps;
}

constexpr double kBoxSize = 0.2;
constexpr double kHalf = kBoxSize / 2.0;

// Creates a world with automatic sleeping enabled. The library default is
// ON, but the helper keeps tests explicit and resilient to per-test overrides.
WorldPtr makeSleepingEnabledWorld()
{
  auto world = World::create();
  auto opts = world->getDeactivationOptions();
  opts.mEnabled = true;
  world->setDeactivationOptions(opts);
  return world;
}

} // namespace

//==============================================================================
// A box dropped onto static ground must come to rest, become flagged resting
// within a bounded number of steps, and then remain frozen (position constant)
// for many further steps.
TEST(IslandDeactivation, SettlesThenSleeps)
{
  EXPECT_TRUE(World::create()->getDeactivationOptions().mEnabled)
      << "automatic deactivation must default to ON";
  auto world = makeSleepingEnabledWorld();

  world->addSkeleton(createFloor());
  // Start just above the floor so it settles quickly.
  auto box = createFreeBox(
      "box",
      Eigen::Vector3d::Constant(kBoxSize),
      Eigen::Vector3d(0, 0, kHalf + 0.02));
  world->addSkeleton(box);

  const std::size_t maxSteps = 5000;
  const std::size_t stepsToSleep
      = stepUntil(world.get(), maxSteps, [&]() { return box->isResting(); });

  EXPECT_LT(stepsToSleep, maxSteps) << "box never went to sleep";
  ASSERT_TRUE(box->isResting());

  // Once asleep, the box must stay frozen: record its position and confirm it
  // does not change over many further steps.
  const Eigen::VectorXd frozenPos = box->getPositions();
  for (std::size_t i = 0; i < 2000; ++i) {
    world->step();
    ASSERT_TRUE(box->isResting()) << "box spuriously woke at step " << i;
    EXPECT_TRUE(box->getPositions().isApprox(frozenPos, 1e-12))
        << "frozen box drifted at step " << i;
  }
}

//==============================================================================
// A skeleton that is put to sleep must preserve the last solved contact wrench
// in its BodyNode force cache. Downstream adapters expose this through joint
// transmitted-wrench APIs, so the transition into sleep must not overwrite the
// value with an unconstrained forward-dynamics pass.
TEST(IslandDeactivation, SleepingPreservesContactBodyForce)
{
  auto world = makeSleepingEnabledWorld();
  world->addSkeleton(createFloor());
  auto arm = createPinnedContactArm("arm");
  world->addSkeleton(arm);

  const std::size_t stepsToSleep
      = stepUntil(world.get(), 5000, [&]() { return arm->isResting(); });
  ASSERT_LT(stepsToSleep, 5000u) << "arm never went to sleep";
  ASSERT_TRUE(arm->isResting());

  const auto* body = arm->getBodyNode(0);
  const Eigen::Vector6d bodyForce = body->getBodyForce();

  EXPECT_GT(bodyForce.tail<3>().norm(), 1.0)
      << "sleep transition discarded the solved contact force";
}

//==============================================================================
// A contact island that is already resting but carries the final solved impulse
// must process that impulse without waking. This covers the path that preserves
// the contact-force cache for transmitted-wrench queries while freezing the
// island for subsequent steps.
TEST(IslandDeactivation, FinalSleepImpulseKeepsBodyFrozen)
{
  auto world = makeSleepingEnabledWorld();
  world->addSkeleton(createFloor());
  auto arm = createPinnedContactArm("arm");
  world->addSkeleton(arm);

  // Establish the contact island, then force the exact post-solver state used
  // for the final sleep solve: resting, sleep-candidate, and carrying an
  // impulse.
  world->step();
  arm->setSleepCandidate(true);
  arm->setResting(true);
  arm->setImpulseApplied(true);
  arm->setVelocities(
      Eigen::VectorXd::Ones(static_cast<int>(arm->getNumDofs())));

  world->step();

  EXPECT_TRUE(arm->isResting());
  EXPECT_TRUE(arm->isSleepCandidate());
  EXPECT_FALSE(arm->isImpulseApplied());
  EXPECT_TRUE(arm->getVelocities().isZero(1e-12));
}

//==============================================================================
// If a resting body receives an impulse while it is not still a sleep
// candidate, it must wake and process the impulse normally.
TEST(IslandDeactivation, NonCandidateImpulseWakesRestingBody)
{
  auto world = makeSleepingEnabledWorld();
  world->setGravity(Eigen::Vector3d::Zero());

  auto box = createFreeBox(
      "box", Eigen::Vector3d::Constant(kBoxSize), Eigen::Vector3d::Zero());
  world->addSkeleton(box);

  box->setResting(true);
  box->setSleepCandidate(false);
  box->setImpulseApplied(true);

  world->step();

  EXPECT_FALSE(box->isResting());
  EXPECT_FALSE(box->isSleepCandidate());
  EXPECT_FALSE(box->isImpulseApplied());
}

//==============================================================================
// The solver may see stale diagnostic island indices when
// solveConstrainedGroups is exercised directly without rebuilding groups. It
// should ignore them instead of indexing into an empty group vector.
TEST(IslandDeactivation, SolverIgnoresStaleIslandIndexWithoutGroups)
{
  ExposedConstraintSolver solver;
  auto box = createFreeBox(
      "box", Eigen::Vector3d::Constant(kBoxSize), Eigen::Vector3d::Zero());

  solver.addSkeleton(box);
  solver.setAutomaticSleepingEnabled(true);
  box->setIslandIndex(0);
  box->setResting(true);

  solver.solveGroupsForTest();

  EXPECT_TRUE(box->isResting());
}

//==============================================================================
// A sleeping box that is struck by a second moving body must wake (via the
// wake-on-contact impulse path) and move.
TEST(IslandDeactivation, WakeOnContact)
{
  auto world = makeSleepingEnabledWorld();
  world->addSkeleton(createFloor());

  auto sleeper = createFreeBox(
      "sleeper",
      Eigen::Vector3d::Constant(kBoxSize),
      Eigen::Vector3d(0, 0, kHalf + 0.02));
  world->addSkeleton(sleeper);

  // Let the sleeper settle and fall asleep first.
  const std::size_t settled
      = stepUntil(world.get(), 5000, [&]() { return sleeper->isResting(); });
  ASSERT_LT(settled, 5000u);
  ASSERT_TRUE(sleeper->isResting());

  const Eigen::Vector3d sleeperPosBefore
      = sleeper->getBodyNode(0)->getTransform().translation();

  // Introduce a projectile aimed at the sleeper along +x with a strong inbound
  // velocity. It is awake (just added), so the island that forms on contact is
  // not all-resting and gets solved, delivering an impulse to the sleeper.
  auto projectile = createFreeBox(
      "projectile",
      Eigen::Vector3d::Constant(kBoxSize),
      Eigen::Vector3d(-0.8, 0, kHalf + 0.02));
  world->addSkeleton(projectile);
  Eigen::Vector6d vel = Eigen::Vector6d::Zero();
  vel[3] = 5.0; // +x linear velocity
  projectile->getJoint(0)->setVelocities(vel);

  // Step until the sleeper wakes or we time out.
  const std::size_t woke
      = stepUntil(world.get(), 3000, [&]() { return !sleeper->isResting(); });
  EXPECT_LT(woke, 3000u) << "sleeper never woke on contact";
  EXPECT_FALSE(sleeper->isResting());

  // It must actually move as a result of the contact.
  for (std::size_t i = 0; i < 500; ++i)
    world->step();
  const Eigen::Vector3d sleeperPosAfter
      = sleeper->getBodyNode(0)->getTransform().translation();
  EXPECT_GT((sleeperPosAfter - sleeperPosBefore).norm(), 1e-3)
      << "sleeper woke but did not move";
}

//==============================================================================
// Applying an external force to a sleeping body must wake it (via the
// wake-on-force path in step loop 1) and the body must accelerate.
TEST(IslandDeactivation, WakeOnExternalForce)
{
  auto world = makeSleepingEnabledWorld();
  world->addSkeleton(createFloor());

  auto box = createFreeBox(
      "box",
      Eigen::Vector3d::Constant(kBoxSize),
      Eigen::Vector3d(0, 0, kHalf + 0.02));
  world->addSkeleton(box);

  const std::size_t settled
      = stepUntil(world.get(), 5000, [&]() { return box->isResting(); });
  ASSERT_LT(settled, 5000u);
  ASSERT_TRUE(box->isResting());

  auto* body = box->getBodyNode(0);

  // Apply a strong horizontal external force every step (so it persists even
  // though World::step clears external forces with _resetCommand==true). The
  // force is present at the top of each step, which is where loop 1 checks for
  // a disturbance.
  const Eigen::Vector3d forceWorld(50.0, 0.0, 0.0);
  body->setExtForce(forceWorld);
  world->step();
  EXPECT_FALSE(box->isResting()) << "external force did not wake the body";

  const double xBefore = body->getTransform().translation().x();
  for (std::size_t i = 0; i < 200; ++i) {
    body->setExtForce(forceWorld);
    world->step();
  }
  const double xAfter = body->getTransform().translation().x();
  EXPECT_GT(xAfter - xBefore, 1e-3) << "forced body did not accelerate";
}

//==============================================================================
// A body with a pending external disturbance must not accumulate quiet dwell
// and freeze, even if the disturbance is small enough that the body remains
// below the sleep thresholds. This covers the pre-resetCommand disturbance
// bookkeeping path for bodies that are not already resting.
TEST(IslandDeactivation, ExternalDisturbancePreventsSleep)
{
  auto world = makeSleepingEnabledWorld();
  auto opts = world->getDeactivationOptions();
  opts.mTimeUntilSleep = 0.05;
  world->setDeactivationOptions(opts);
  world->addSkeleton(createFloor());

  auto box = createFreeBox(
      "box",
      Eigen::Vector3d::Constant(kBoxSize),
      Eigen::Vector3d(0, 0, kHalf + 0.02));
  world->addSkeleton(box);
  auto* body = box->getBodyNode(0);

  // Greater than hasExternalDisturbance()'s tolerance but far below the force
  // needed to keep the body visibly moving on the floor.
  const Eigen::Vector3d tinyForce(1.0e-5, 0.0, 0.0);
  for (std::size_t i = 0; i < 3000; ++i) {
    body->setExtForce(tinyForce);
    world->step();
    ASSERT_FALSE(box->isResting()) << "actuated quiet body slept at step " << i;
  }
}

//==============================================================================
// A vertical contact stack must sleep atomically as a single island. No member
// sleeps while another member in the same contact island is not yet a sleep
// candidate.
TEST(IslandDeactivation, StackSleepsAsIsland)
{
  auto world = makeSleepingEnabledWorld();
  world->setGravity(Eigen::Vector3d::Zero());
  world->addSkeleton(createFloor());

  // Three stacked boxes with tiny deterministic overlaps, each below the
  // contact-correction gate for sleep eligibility. This avoids relying on
  // platform-specific settling jitter to create the contact island.
  constexpr double overlap = 1.0e-8;
  std::vector<SkeletonPtr> stack;
  for (int i = 0; i < 3; ++i) {
    const double z = kHalf - overlap / 2.0 + i * (kBoxSize - overlap);
    auto b = createFreeBox(
        "stack" + std::to_string(i),
        Eigen::Vector3d::Constant(kBoxSize),
        Eigen::Vector3d(0, 0, z));
    world->addSkeleton(b);
    stack.push_back(b);
  }

  // A partial candidate island must remain fully awake.
  stack[0]->setSleepCandidate(true);
  stack[1]->setSleepCandidate(true);
  stack[2]->setSleepCandidate(false);
  world->step();
  for (const auto& b : stack)
    EXPECT_FALSE(b->isResting())
        << b->getName() << " slept before the full island was ready";

  // Once all members are candidates, the solver stamps the resting flag on the
  // whole island together.
  for (const auto& b : stack)
    b->setSleepCandidate(true);
  world->step();
  for (const auto& b : stack)
    EXPECT_TRUE(b->isResting())
        << b->getName() << " did not sleep with the rest of the stack"
        << " (island=" << b->getIslandIndex() << ")";

  // Confirm the whole stack stays frozen together.
  std::vector<Eigen::VectorXd> frozen;
  for (const auto& b : stack)
    frozen.push_back(b->getPositions());
  for (std::size_t i = 0; i < 1000; ++i) {
    world->step();
    for (std::size_t j = 0; j < stack.size(); ++j) {
      ASSERT_TRUE(stack[j]->isResting());
      EXPECT_TRUE(stack[j]->getPositions().isApprox(frozen[j], 1e-10));
    }
  }
}

//==============================================================================
// A body kept in genuine slow motion (above the sleep thresholds) must NOT be
// put to sleep within the test horizon. This is the "always beneficial"
// correctness gate: we must never freeze a body that is actually moving.
TEST(IslandDeactivation, NoPrematureSleepOfMovingBody)
{
  auto world = makeSleepingEnabledWorld();
  world->setGravity(Eigen::Vector3d::Zero()); // isolate the motion we impose

  auto box = createFreeBox(
      "mover", Eigen::Vector3d::Constant(kBoxSize), Eigen::Vector3d(0, 0, 0));
  world->addSkeleton(box);

  const auto& opts = world->getDeactivationOptions();
  // Choose a speed comfortably above the linear sleep threshold but below the
  // wake band, representing steady slow translation.
  const double speed = opts.mLinearSpeedThreshold * 5.0;

  Eigen::Vector6d vel = Eigen::Vector6d::Zero();
  vel[3] = speed; // +x

  const double horizon = 5.0; // seconds
  const std::size_t steps
      = static_cast<std::size_t>(horizon / world->getTimeStep());
  for (std::size_t i = 0; i < steps; ++i) {
    // Re-impose the velocity each step to model sustained driven motion in the
    // absence of any forces or contacts.
    box->getJoint(0)->setVelocities(vel);
    world->step();
    ASSERT_FALSE(box->isResting())
        << "genuinely moving body was frozen at step " << i;
  }
}

//==============================================================================
// With sleeping disabled, the box must never report resting, and its trajectory
// must match a baseline where sleeping is enabled but the body never reaches
// sleep eligibility (i.e. identical physics). The baseline keeps sleeping
// unreachable by setting an unreachable dwell time, so both runs execute the
// same physics path with no skips.
TEST(IslandDeactivation, DisabledIsNoOp)
{
  auto makeWorld = []() {
    auto world = World::create();
    world->addSkeleton(createFloor());
    auto box = createFreeBox(
        "box",
        Eigen::Vector3d::Constant(kBoxSize),
        Eigen::Vector3d(0, 0, kHalf + 0.05));
    world->addSkeleton(box);
    return world;
  };

  // Run A: sleeping explicitly disabled.
  auto worldA = makeWorld();
  DeactivationOptions disabled;
  disabled.mEnabled = false;
  worldA->setDeactivationOptions(disabled);
  auto boxA = worldA->getSkeleton("box");

  // Run B (baseline): sleeping enabled but configured so the body can never
  // accumulate enough dwell time to sleep within the horizon. This produces the
  // never-sleeping reference trajectory the disabled run must match.
  auto worldB = makeWorld();
  DeactivationOptions neverSleeps;
  neverSleeps.mEnabled = true;
  neverSleeps.mTimeUntilSleep = 1.0e30;
  worldB->setDeactivationOptions(neverSleeps);
  auto boxB = worldB->getSkeleton("box");

  const std::size_t steps = 2000;
  for (std::size_t i = 0; i < steps; ++i) {
    worldA->step();
    worldB->step();

    // Disabled run must never flag resting.
    ASSERT_FALSE(boxA->isResting());
    // Baseline run also never sleeps within the horizon (unreachable dwell).
    ASSERT_FALSE(boxB->isResting());

    // Trajectories must be byte-for-byte identical: with no skeleton resting,
    // the new code paths take no action, so the physics is unchanged.
    ASSERT_EQ(boxA->getPositions(), boxB->getPositions())
        << "trajectory diverged at step " << i;
  }
}

//==============================================================================
// A settled body exposes the solver-island index it belongs to (used by the
// GUI example to color islands). It is -1 only when the feature is off or the
// body is in no island.
TEST(IslandDeactivation, IslandIndexExposed)
{
  auto world = makeSleepingEnabledWorld();
  world->addSkeleton(createFloor());
  auto box = createFreeBox(
      "box",
      Eigen::Vector3d::Constant(kBoxSize),
      Eigen::Vector3d(0, 0, kHalf + 0.02));
  world->addSkeleton(box);

  // After it lands on the floor the box forms a one-body contact island, so the
  // solver stamps a valid (non-negative) island index every step thereafter.
  for (std::size_t i = 0; i < 200; ++i)
    world->step();
  EXPECT_GE(box->getIslandIndex(), 0);
}

//==============================================================================
// hasExternalDisturbance() must wake a sleeping body when an internal
// generalized force is applied (covers the getForces() branch).
TEST(IslandDeactivation, WakeOnInternalForce)
{
  auto world = makeSleepingEnabledWorld();
  world->addSkeleton(createFloor());
  auto box = createFreeBox(
      "box",
      Eigen::Vector3d::Constant(kBoxSize),
      Eigen::Vector3d(0, 0, kHalf + 0.02));
  world->addSkeleton(box);
  ASSERT_LT(
      stepUntil(world.get(), 5000, [&]() { return box->isResting(); }), 5000u);

  box->setForces(Eigen::Vector6d::Constant(5.0));
  world->step();
  EXPECT_FALSE(box->isResting()) << "internal force should wake the body";
}

//==============================================================================
// hasExternalDisturbance() must wake a sleeping body when a command is set
// (covers the getCommands() branch).
TEST(IslandDeactivation, WakeOnCommand)
{
  auto world = makeSleepingEnabledWorld();
  world->addSkeleton(createFloor());
  auto box = createFreeBox(
      "box",
      Eigen::Vector3d::Constant(kBoxSize),
      Eigen::Vector3d(0, 0, kHalf + 0.02));
  world->addSkeleton(box);
  ASSERT_LT(
      stepUntil(world.get(), 5000, [&]() { return box->isResting(); }), 5000u);

  const Eigen::VectorXd positionsBefore = box->getPositions();
  box->setCommands(Eigen::Vector6d::Constant(1.0));
  world->step();
  EXPECT_FALSE(box->isResting()) << "a set command should wake the body";
  EXPECT_GT((box->getPositions() - positionsBefore).norm(), 0.0)
      << "a set command should advance the body in the waking step";
}

//==============================================================================
// A sleeping vehicle-like model with servo-driven wheel joints must advance on
// the first commanded step and match the same step from an otherwise identical
// awake baseline. gz-sim's DiffDrive system sends wheel velocity commands
// through gz-physics as DART SERVO commands, so sleeping must be a performance
// optimization only: it must not consume or delay the first command.
TEST(IslandDeactivation, ServoWheelCommandMatchesAwakeBaseline)
{
  auto sleepingWorld = makeSleepingEnabledWorld();
  sleepingWorld->addSkeleton(createFloor());
  auto sleepingCart = createServoWheelCart("sleeping_cart");
  sleepingWorld->addSkeleton(sleepingCart.skel);

  ASSERT_LT(
      stepUntil(
          sleepingWorld.get(),
          5000,
          [&]() { return sleepingCart.skel->isResting(); }),
      5000u);
  ASSERT_TRUE(sleepingCart.skel->isResting());

  auto awakeWorld = makeSleepingEnabledWorld();
  auto awakeOptions = awakeWorld->getDeactivationOptions();
  awakeOptions.mTimeUntilSleep = 1.0e30;
  awakeWorld->setDeactivationOptions(awakeOptions);
  awakeWorld->addSkeleton(createFloor());
  auto awakeCart = createServoWheelCart("awake_cart");
  awakeCart.skel->setPositions(sleepingCart.skel->getPositions());
  awakeCart.skel->setVelocities(sleepingCart.skel->getVelocities());
  awakeWorld->addSkeleton(awakeCart.skel);

  auto setWheelServoCommand = [](ServoWheelCart& cart, double command) {
    cart.leftWheel->setActuatorType(Joint::SERVO);
    cart.rightWheel->setActuatorType(Joint::SERVO);
    cart.leftWheel->setCommand(0, command);
    cart.rightWheel->setCommand(0, command);
  };

  const double xBefore = sleepingCart.chassis->getTransform().translation().x();
  constexpr double command = -1.0 / 300.0;
  setWheelServoCommand(sleepingCart, command);
  setWheelServoCommand(awakeCart, command);
  sleepingWorld->step();
  awakeWorld->step();

  EXPECT_FALSE(sleepingCart.skel->isResting())
      << "wheel velocity commands should wake the cart";
  EXPECT_GT(sleepingCart.chassis->getTransform().translation().x(), xBefore)
      << "wheel velocity commands should move the cart in the waking step";
  EXPECT_TRUE(sleepingCart.skel->getPositions().isApprox(
      awakeCart.skel->getPositions(), 1e-12))
      << "sleeping changed the first commanded-step positions";
  EXPECT_TRUE(sleepingCart.skel->getVelocities().isApprox(
      awakeCart.skel->getVelocities(), 1e-12))
      << "sleeping changed the first commanded-step velocities";
}

//==============================================================================
// A zero-DOF skeleton never reports a disturbance (covers the early return).
TEST(IslandDeactivation, DisturbanceFalseForZeroDof)
{
  auto floor = createFloor(); // WeldJoint to world => zero DOFs
  ASSERT_EQ(floor->getNumDofs(), 0u);
  EXPECT_FALSE(floor->hasExternalDisturbance());
}

//==============================================================================
// Disabling sleeping must clear any existing resting state so subsequent steps
// process every skeleton normally.
TEST(IslandDeactivation, DisablingClearsRestState)
{
  auto world = makeSleepingEnabledWorld();
  world->addSkeleton(createFloor());
  auto box = createFreeBox(
      "box",
      Eigen::Vector3d::Constant(kBoxSize),
      Eigen::Vector3d(0, 0, kHalf + 0.02));
  world->addSkeleton(box);
  ASSERT_LT(
      stepUntil(world.get(), 5000, [&]() { return box->isResting(); }), 5000u);
  ASSERT_TRUE(box->isResting());

  auto opts = world->getDeactivationOptions();
  opts.mEnabled = false;
  world->setDeactivationOptions(opts);
  EXPECT_FALSE(box->isResting()) << "disabling must clear the resting flag";
  EXPECT_EQ(box->getIslandIndex(), -1)
      << "disabling must clear the visualization island index";
}

//==============================================================================
// Performance evidence (run explicitly with --gtest_also_run_disabled_tests).
// Not a pass/fail correctness test - it prints wall-clock step times so the
// "always beneficial" claim can be measured: a large speedup at scale for
// resting scenes, and negligible overhead for an always-awake scene.
namespace {

double timeSteps(World* world, std::size_t numSteps)
{
  const auto t0 = std::chrono::steady_clock::now();
  for (std::size_t i = 0; i < numSteps; ++i)
    world->step();
  const auto t1 = std::chrono::steady_clock::now();
  return std::chrono::duration<double, std::milli>(t1 - t0).count();
}

// Grid of separated boxes on the floor: each box is its own independent solver
// island, all coming to rest. The setup lets them settle (and sleep, if
// sleeping is enabled) before the timed window.
WorldPtr makeRestingGrid(int n, bool sleepingEnabled)
{
  auto world = World::create();
  auto opts = world->getDeactivationOptions();
  opts.mEnabled = sleepingEnabled;
  world->setDeactivationOptions(opts);
  world->addSkeleton(createFloor());
  const int side
      = static_cast<int>(std::ceil(std::sqrt(static_cast<double>(n))));
  const double spacing = 4.0 * kBoxSize; // far enough apart to never touch
  int made = 0;
  for (int r = 0; r < side && made < n; ++r) {
    for (int c = 0; c < side && made < n; ++c, ++made) {
      const Eigen::Vector3d pos(
          (c - side / 2) * spacing, (r - side / 2) * spacing, kHalf + 0.01);
      world->addSkeleton(createFreeBox(
          "b" + std::to_string(made),
          Eigen::Vector3d::Constant(kBoxSize),
          pos));
    }
  }
  return world;
}

} // namespace

TEST(IslandDeactivation, DISABLED_BenchmarkRestingGrid)
{
  std::cout
      << "\n[bench] resting grid: step time over 1000 steps after settling\n";
  std::cout << "  N    off(ms)   on(ms)   speedup\n";
  for (int n : {16, 64, 144, 256}) {
    auto wOff = makeRestingGrid(n, false);
    auto wOn = makeRestingGrid(n, true);
    for (int i = 0; i < 2000; ++i) {
      wOff->step();
      wOn->step();
    } // settle
    const double off = timeSteps(wOff.get(), 1000);
    const double on = timeSteps(wOn.get(), 1000);
    std::cout << "  " << n << "   " << off << "   " << on << "   " << (off / on)
              << "x\n";
  }
}

TEST(IslandDeactivation, DISABLED_BenchmarkActiveOverhead)
{
  // Always-awake scene: boxes kept in continuous motion so nothing ever sleeps.
  // Measures the detection overhead of the feature in the worst case for it.
  std::cout << "\n[bench] awake scene (never sleeps): detection overhead\n";
  const int n = 144;
  auto build = [&](bool sleepingEnabled) {
    auto world = World::create();
    auto opts = world->getDeactivationOptions();
    opts.mEnabled = sleepingEnabled;
    world->setDeactivationOptions(opts);
    const int side = static_cast<int>(std::ceil(std::sqrt((double)n)));
    int made = 0;
    for (int r = 0; r < side && made < n; ++r)
      for (int c = 0; c < side && made < n; ++c, ++made) {
        auto b = createFreeBox(
            "b" + std::to_string(made),
            Eigen::Vector3d::Constant(kBoxSize),
            Eigen::Vector3d(c * 0.5, r * 0.5, 5.0));
        b->setVelocities(Eigen::Vector6d::Constant(0.5)); // keep moving
        world->addSkeleton(b);
      }
    world->setGravity(Eigen::Vector3d::Zero()); // free, no settling
    return world;
  };
  auto wOff = build(false);
  auto wOn = build(true);
  const double off = timeSteps(wOff.get(), 1000);
  const double on = timeSteps(wOn.get(), 1000);
  std::cout << "  N=" << n << "  off(ms)=" << off << "  on(ms)=" << on
            << "  overhead=" << (100.0 * (on - off) / off) << "%\n";
}

//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
