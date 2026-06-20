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
using namespace dart::collision;
using namespace dart::dynamics;
using namespace dart::simulation;

namespace {

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
// Builds an articulated mobile skeleton with two intentionally overlapping
// collision boxes. The solver resting-contact filter must still honor the
// skeleton's ordinary self-collision settings when an awake body exists.
SkeletonPtr createOverlappingTwoBodySkeleton(const std::string& name)
{
  auto skel = Skeleton::create(name);

  GenericJoint<SE3Space>::Properties rootJointProps(name + "_root_joint");
  BodyNode::Properties rootBodyProps(
      BodyNode::AspectProperties(name + "_root_body"));
  rootBodyProps.mInertia.setMass(1.0);

  auto rootPair = skel->createJointAndBodyNodePair<FreeJoint>(
      nullptr, rootJointProps, rootBodyProps);
  auto* rootBody = rootPair.second;
  rootBody->createShapeNodeWith<CollisionAspect, DynamicsAspect>(
      std::make_shared<BoxShape>(Eigen::Vector3d::Constant(0.2)));

  WeldJoint::Properties childJointProps;
  childJointProps.mName = name + "_child_joint";
  BodyNode::Properties childBodyProps(
      BodyNode::AspectProperties(name + "_child_body"));
  childBodyProps.mInertia.setMass(1.0);

  auto* childBody = skel->createJointAndBodyNodePair<WeldJoint>(
                            rootBody, childJointProps, childBodyProps)
                        .second;
  childBody->createShapeNodeWith<CollisionAspect, DynamicsAspect>(
      std::make_shared<BoxShape>(Eigen::Vector3d::Constant(0.2)));

  skel->disableSelfCollisionCheck();
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

class TogglePairCollisionFilter : public BodyNodeCollisionFilter
{
public:
  TogglePairCollisionFilter(
      const BodyNode* body1, const BodyNode* body2, bool ignorePair)
    : mBody1(body1), mBody2(body2), mIgnorePair(ignorePair)
  {
  }

  void setIgnorePair(bool ignorePair)
  {
    mIgnorePair = ignorePair;
  }

  void resetNumCalls()
  {
    mNumCalls = 0u;
  }

  std::size_t getNumCalls() const
  {
    return mNumCalls;
  }

  bool ignoresCollision(
      const CollisionObject* object1,
      const CollisionObject* object2) const override
  {
    ++mNumCalls;

    if (BodyNodeCollisionFilter::ignoresCollision(object1, object2))
      return true;

    const auto* body1 = object1->getBodyNode();
    const auto* body2 = object2->getBodyNode();
    const bool targetPair = (body1 == mBody1 && body2 == mBody2)
                            || (body1 == mBody2 && body2 == mBody1);
    return mIgnorePair && targetPair;
  }

private:
  const BodyNode* mBody1;
  const BodyNode* mBody2;
  bool mIgnorePair;
  mutable std::size_t mNumCalls = 0u;
};

// Creates a world with automatic deactivation enabled. Keep this explicit so
// tests stay robust if callers locally override the default options.
WorldPtr makeSleepWorld()
{
  auto world = World::create();
  auto opts = world->getDeactivationOptions();
  opts.mEnabled = true;
  world->setDeactivationOptions(opts);
  return world;
}

WorldPtr makeSettlingComparisonWorld(bool deactivationEnabled)
{
  auto world = World::create();
  auto opts = world->getDeactivationOptions();
  opts.mEnabled = deactivationEnabled;
  world->setDeactivationOptions(opts);

  world->addSkeleton(createFloor());
  world->addSkeleton(createFreeBox(
      "box",
      Eigen::Vector3d::Constant(kBoxSize),
      Eigen::Vector3d(0, 0, kHalf + 0.2)));

  return world;
}

void stepUntilRestingFastPathReady(
    World* world, const SkeletonPtr& sleeper, std::size_t maxSteps = 5000)
{
  const std::size_t settled
      = stepUntil(world, maxSteps, [&]() { return sleeper->isResting(); });
  ASSERT_LT(settled, maxSteps);
  ASSERT_TRUE(sleeper->isResting());

  for (std::size_t i = 0; i < 1000; ++i) {
    world->step();
    if (world->getLastCollisionResult().getNumContacts() == 0)
      break;
  }

  ASSERT_EQ(0u, world->getLastCollisionResult().getNumContacts());
  ASSERT_TRUE(sleeper->isResting());

  // Exercise the cached all-resting step once before mutating support geometry.
  world->step();
  ASSERT_EQ(0u, world->getLastCollisionResult().getNumContacts());
  ASSERT_TRUE(sleeper->isResting());
}

void expectSleeperFallsAfterSupportEdit(
    World* world, const SkeletonPtr& sleeper)
{
  const double zBefore
      = sleeper->getBodyNode(0)->getTransform().translation().z();

  world->step();
  EXPECT_FALSE(sleeper->isResting())
      << "support edit did not wake the sleeping dynamic body";

  for (std::size_t i = 0; i < 40; ++i)
    world->step();

  const double zAfter
      = sleeper->getBodyNode(0)->getTransform().translation().z();
  EXPECT_LT(zAfter, zBefore - 1e-4)
      << "unsupported body did not resume falling";
}

void expectSolverRunsAfterRestingConstraintEdit(
    World* world, const SkeletonPtr& sleeper)
{
  world->step();
  EXPECT_GT(world->getLastCollisionResult().getNumContacts(), 0u)
      << "automatic joint-constraint edit reused the all-resting fast path";
  EXPECT_FALSE(sleeper->isResting())
      << "automatic joint-constraint edit did not wake the sleeping body";
}

} // namespace

//==============================================================================
// Automatic deactivation is enabled by default so users get resting-scene
// acceleration by upgrading. The opt-out path is covered by DisabledIsNoOp.
TEST(IslandDeactivation, EnabledByDefault)
{
  const auto world = World::create();
  const auto& opts = world->getDeactivationOptions();
  EXPECT_TRUE(opts.mEnabled);
}

//==============================================================================
// Default-on sleeping must not trade physics fidelity for speed. After the same
// drop-and-settle scenario, the sleeping result should match the always-active
// solver path within contact-solver tolerance and land at the expected support
// height.
TEST(IslandDeactivation, DefaultEnabledSettlesCloseToAlwaysActivePath)
{
  auto enabledWorld = makeSettlingComparisonWorld(true);
  auto activeWorld = makeSettlingComparisonWorld(false);
  auto enabledBox = enabledWorld->getSkeleton("box");
  auto activeBox = activeWorld->getSkeleton("box");
  ASSERT_TRUE(enabledBox);
  ASSERT_TRUE(activeBox);

  constexpr std::size_t steps = 6000;
  for (std::size_t i = 0; i < steps; ++i) {
    enabledWorld->step();
    activeWorld->step();
  }

  ASSERT_TRUE(enabledBox->isResting());
  EXPECT_FALSE(activeBox->isResting());

  const auto enabledTransform = enabledBox->getBodyNode(0)->getTransform();
  const auto activeTransform = activeBox->getBodyNode(0)->getTransform();
  EXPECT_TRUE(enabledTransform.translation().isApprox(
      activeTransform.translation(), 1e-4));
  EXPECT_TRUE(
      enabledTransform.linear().isApprox(activeTransform.linear(), 1e-6));
  EXPECT_NEAR(enabledTransform.translation().z(), kHalf, 1e-5);
}

//==============================================================================
// A box dropped onto static ground must come to rest, become flagged resting
// within a bounded number of steps, and then remain frozen (position constant)
// for many further steps.
TEST(IslandDeactivation, SettlesThenSleeps)
{
  auto world = makeSleepWorld();

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
// The transition into the frozen state runs one final contact solve for
// observable contact-force caches, but it must not integrate that final impulse
// into the pose or leave stale non-zero body velocity caches behind.
TEST(IslandDeactivation, SleepTransitionFreezesLastSolvedPoseAndVelocity)
{
  auto world = makeSleepWorld();
  world->addSkeleton(createFloor());
  auto box = createFreeBox(
      "box",
      Eigen::Vector3d::Constant(kBoxSize),
      Eigen::Vector3d(0, 0, kHalf + 0.02));
  world->addSkeleton(box);

  auto* body = box->getBodyNode(0);
  Eigen::VectorXd lastAwakePos = box->getPositions();
  bool slept = false;
  for (std::size_t i = 0; i < 5000; ++i) {
    ASSERT_FALSE(box->isResting());
    lastAwakePos = box->getPositions();

    world->step();
    if (box->isResting()) {
      slept = true;
      break;
    }
  }

  ASSERT_TRUE(slept) << "box never went to sleep";
  EXPECT_TRUE(box->getPositions().isApprox(lastAwakePos, 1e-12))
      << "sleep transition integrated the final contact impulse";
  EXPECT_NEAR(body->getTransform().translation().z(), kHalf, 1e-5);
  EXPECT_NEAR(body->getLinearVelocity().norm(), 0.0, 1e-12);
  EXPECT_NEAR(body->getAngularVelocity().norm(), 0.0, 1e-12);
}

//==============================================================================
// A candidate island whose contacts have not physically converged must be kept
// awake. Otherwise default-on sleeping can leave downstream integrations with a
// residual velocity before the contact solver has actually settled the body.
TEST(IslandDeactivation, UnconvergedContactClearsSleepCandidate)
{
  auto world = makeSleepWorld();
  world->addSkeleton(createFloor());
  auto box = createFreeBox(
      "box",
      Eigen::Vector3d::Constant(kBoxSize),
      Eigen::Vector3d(0, 0, kHalf - 0.01));
  Eigen::Vector6d residualVelocity = Eigen::Vector6d::Zero();
  residualVelocity[3] = 0.005;
  box->getJoint(0)->setVelocities(residualVelocity);
  box->setSleepCandidate(true);
  box->setRestDwellTime(world->getDeactivationOptions().mTimeUntilSleep);
  world->addSkeleton(box);

  world->step();

  ASSERT_GT(world->getLastCollisionResult().getNumContacts(), 0u);
  EXPECT_FALSE(box->isSleepCandidate());
  EXPECT_FALSE(box->isResting());
}

//==============================================================================
// A quiet support contact must not freeze while another mobile body in the same
// world is still falling. Downstream worlds can attach/detach joints between
// otherwise independent models; sleeping one settled model before the other
// dynamic models finish settling changes the later contact response.
TEST(IslandDeactivation, SettledBodyWaitsForOtherMobileBodyToSettle)
{
  auto world = makeSleepWorld();
  world->addSkeleton(createFloor());
  auto settled = createFreeBox(
      "settled",
      Eigen::Vector3d::Constant(kBoxSize),
      Eigen::Vector3d(0, 0, kHalf + 0.02));
  auto falling = createFreeBox(
      "falling",
      Eigen::Vector3d::Constant(kBoxSize),
      Eigen::Vector3d(1.0, 0, kHalf + 3.0));
  world->addSkeleton(settled);
  world->addSkeleton(falling);

  for (std::size_t i = 0; i < 650; ++i) {
    world->step();
    ASSERT_FALSE(settled->isResting())
        << "settled body slept while another body was still active at step "
        << i;
    ASSERT_FALSE(falling->isResting());
  }

  EXPECT_GT(falling->getBodyNode(0)->getTransform().translation().z(), 1.0);

  const std::size_t stepsToSleep = stepUntil(world.get(), 7000, [&]() {
    return settled->isResting() && falling->isResting();
  });
  EXPECT_LT(stepsToSleep, 7000u)
      << "bodies did not sleep after the active body settled";
}

//==============================================================================
// Once every mobile body is resting and the previous collision pass produced no
// contacts, the all-resting fast path may skip physics work. It still must
// advance the observable world clock and frame counter exactly like a normal
// step.
TEST(IslandDeactivation, AllRestingFastPathAdvancesTimeAndFrame)
{
  auto world = makeSleepWorld();

  world->addSkeleton(createFloor());
  auto box = createFreeBox(
      "box",
      Eigen::Vector3d::Constant(kBoxSize),
      Eigen::Vector3d(0, 0, kHalf + 0.02));
  world->addSkeleton(box);

  const std::size_t stepsToSleep
      = stepUntil(world.get(), 5000, [&]() { return box->isResting(); });
  ASSERT_LT(stepsToSleep, 5000u) << "box never went to sleep";
  ASSERT_TRUE(box->isResting());

  for (std::size_t i = 0; i < 1000; ++i) {
    world->step();
    if (world->getLastCollisionResult().getNumContacts() == 0)
      break;
  }
  ASSERT_EQ(0u, world->getLastCollisionResult().getNumContacts());
  ASSERT_TRUE(box->isResting());

  const double timeBefore = world->getTime();
  const int frameBefore = world->getSimFrames();
  const Eigen::VectorXd frozenPos = box->getPositions();

  constexpr std::size_t steps = 100;
  for (std::size_t i = 0; i < steps; ++i) {
    world->step();
    ASSERT_TRUE(box->isResting()) << "box spuriously woke at step " << i;
    EXPECT_TRUE(box->getPositions().isApprox(frozenPos, 1e-12))
        << "frozen box drifted at step " << i;
  }

  EXPECT_NEAR(
      world->getTime() - timeBefore,
      static_cast<double>(steps) * world->getTimeStep(),
      1e-12);
  EXPECT_EQ(frameBefore + static_cast<int>(steps), world->getSimFrames());
}

//==============================================================================
// GUI-only visual updates must not invalidate the all-resting fast path. The
// sleep-state color overlay changes VisualAspect properties after a body has
// gone to sleep; this must not look like a pose edit or wake the simulation.
TEST(IslandDeactivation, VisualColorChangeDoesNotWakeRestingBody)
{
  auto world = makeSleepWorld();

  world->addSkeleton(createFloor());
  auto box = createFreeBox(
      "box",
      Eigen::Vector3d::Constant(kBoxSize),
      Eigen::Vector3d(0, 0, kHalf + 0.02));

  auto* body = box->getBodyNode(0);
  body->createShapeNodeWith<VisualAspect>(
      std::make_shared<BoxShape>(Eigen::Vector3d::Constant(kBoxSize)));
  world->addSkeleton(box);

  const std::size_t stepsToSleep
      = stepUntil(world.get(), 5000, [&]() { return box->isResting(); });
  ASSERT_LT(stepsToSleep, 5000u) << "box never went to sleep";
  ASSERT_TRUE(box->isResting());

  for (std::size_t i = 0; i < 1000; ++i) {
    world->step();
    if (world->getLastCollisionResult().getNumContacts() == 0)
      break;
  }
  ASSERT_EQ(0u, world->getLastCollisionResult().getNumContacts());
  ASSERT_TRUE(box->isResting());

  // Prime the ready snapshot, then mutate only the visual color.
  world->step();
  ASSERT_TRUE(box->isResting());
  ASSERT_EQ(0u, world->getLastCollisionResult().getNumContacts());

  box->setColor(Eigen::Vector4d(0.05, 0.35, 1.0, 1.0));
  world->step();

  EXPECT_TRUE(box->isResting())
      << "visual-only color change woke a sleeping body";
  EXPECT_EQ(0u, world->getLastCollisionResult().getNumContacts())
      << "visual-only color change invalidated the all-resting fast path";
}

//==============================================================================
// A skeleton that is put to sleep must preserve the last solved contact wrench
// in its BodyNode force cache. Downstream adapters expose this through joint
// transmitted-wrench APIs, so the transition into sleep must not overwrite the
// value with an unconstrained forward-dynamics pass.
TEST(IslandDeactivation, SleepingPreservesContactBodyForce)
{
  auto world = makeSleepWorld();
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
// A sleeping box that is struck by a second moving body must wake (via the
// wake-on-contact impulse path) and move.
TEST(IslandDeactivation, WakeOnContact)
{
  auto world = makeSleepWorld();
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
// Editing a sleeping body's velocity between all-resting fast-path steps must
// wake it so the requested motion is integrated.
TEST(IslandDeactivation, WakeOnVelocityEdit)
{
  auto world = makeSleepWorld();
  world->addSkeleton(createFloor());
  auto sleeper = createFreeBox(
      "sleeper",
      Eigen::Vector3d::Constant(kBoxSize),
      Eigen::Vector3d(0, 0, kHalf + 0.02));
  world->addSkeleton(sleeper);

  ASSERT_NO_FATAL_FAILURE(stepUntilRestingFastPathReady(world.get(), sleeper));

  const double xBefore
      = sleeper->getBodyNode(0)->getTransform().translation().x();
  Eigen::Vector6d vel = Eigen::Vector6d::Zero();
  vel[3] = 1.0;
  sleeper->getJoint(0)->setVelocities(vel);

  world->step();
  EXPECT_FALSE(sleeper->isResting())
      << "velocity edit was hidden by the all-resting fast path";
  const double xAfter
      = sleeper->getBodyNode(0)->getTransform().translation().x();
  EXPECT_GT(xAfter, xBefore + 1e-5) << "edited velocity was not integrated";
}

//==============================================================================
// When an awake body hits one member of a frozen mobile contact island, the
// resting-resting contacts inside that island must participate in the same
// solver pass so the wake is island-atomic.
TEST(IslandDeactivation, WakeOnContactPreservesRestingIsland)
{
  auto world = makeSleepWorld();
  world->setGravity(Eigen::Vector3d::Zero());

  auto projectile = createFreeBox(
      "projectile",
      Eigen::Vector3d::Constant(kBoxSize),
      Eigen::Vector3d(-0.8, 0, kHalf));
  world->addSkeleton(projectile);

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

  for (const auto& b : stack) {
    b->setSleepCandidate(true);
    b->setIslandIndex(0);
    b->setResting(true);
    ASSERT_TRUE(b->isResting()) << b->getName() << " did not sleep";
  }

  Eigen::Vector6d vel = Eigen::Vector6d::Zero();
  vel[3] = 2.0;
  projectile->getJoint(0)->setVelocities(vel);

  const std::size_t woke
      = stepUntil(world.get(), 1000, [&]() { return !stack[0]->isResting(); });
  ASSERT_LT(woke, 1000u) << "projectile never woke the contacted stack member";
  for (const auto& b : stack) {
    EXPECT_FALSE(b->isResting())
        << b->getName() << " stayed frozen after its island was hit";
  }
}

//==============================================================================
// Preserving resting contacts inside a frozen mobile island is a solver-local
// wakeup optimization; it must not bypass ordinary same-skeleton filtering.
TEST(IslandDeactivation, RestingIslandPreservesSelfCollisionFilter)
{
  auto world = makeSleepWorld();
  world->setGravity(Eigen::Vector3d::Zero());

  auto resting = createOverlappingTwoBodySkeleton("resting");
  world->addSkeleton(resting);

  auto awake = createFreeBox(
      "awake", Eigen::Vector3d::Constant(kBoxSize), Eigen::Vector3d(10, 0, 0));
  world->addSkeleton(awake);

  resting->setSleepCandidate(true);
  resting->setIslandIndex(0);
  resting->setResting(true);
  ASSERT_TRUE(resting->isResting());
  ASSERT_FALSE(resting->isEnabledSelfCollisionCheck());

  world->getConstraintSolver()->solve();
  EXPECT_EQ(
      0u,
      world->getConstraintSolver()->getLastCollisionResult().getNumContacts());
}

//==============================================================================
// A sleeping body must not stay hidden behind the all-resting fast path when an
// external API edits static geometry. gz-physics can move free groups between
// simulation steps, so DART must invalidate the resting cache and perform a
// real collision pass instead of filtering resting-vs-static pairs forever.
TEST(IslandDeactivation, WakeOnStaticPoseChange)
{
  auto world = makeSleepWorld();
  world->addSkeleton(createFloor());

  auto sleeper = createFreeBox(
      "sleeper",
      Eigen::Vector3d::Constant(kBoxSize),
      Eigen::Vector3d(0, 0, kHalf + 0.02));
  world->addSkeleton(sleeper);

  auto blocker = createFreeBox(
      "blocker",
      Eigen::Vector3d::Constant(kBoxSize),
      Eigen::Vector3d(2.0, 0, kHalf));
  blocker->setMobile(false);
  world->addSkeleton(blocker);

  ASSERT_NO_FATAL_FAILURE(stepUntilRestingFastPathReady(world.get(), sleeper));

  Eigen::Isometry3d moved = Eigen::Isometry3d::Identity();
  moved.translation() = Eigen::Vector3d(kBoxSize * 0.25, 0, kHalf);
  static_cast<FreeJoint*>(blocker->getJoint(0))->setTransform(moved);

  world->step();
  EXPECT_GT(world->getLastCollisionResult().getNumContacts(), 0u)
      << "static pose edit was hidden by the all-resting fast path";
  EXPECT_FALSE(sleeper->isResting())
      << "static pose edit did not wake the sleeping dynamic body";
}

//==============================================================================
// Removing a static support must wake sleeping bodies before the no-contact
// solver path can preserve stale resting flags.
TEST(IslandDeactivation, WakeOnSupportRemoved)
{
  auto world = makeSleepWorld();
  auto floor = createFloor();
  world->addSkeleton(floor);

  auto sleeper = createFreeBox(
      "sleeper",
      Eigen::Vector3d::Constant(kBoxSize),
      Eigen::Vector3d(0, 0, kHalf + 0.02));
  world->addSkeleton(sleeper);

  ASSERT_NO_FATAL_FAILURE(stepUntilRestingFastPathReady(world.get(), sleeper));

  world->removeSkeleton(floor);
  expectSleeperFallsAfterSupportEdit(world.get(), sleeper);
}

//==============================================================================
// Disabling collision on an immobile support changes the physical contact set
// even though the support skeleton itself is not mobile.
TEST(IslandDeactivation, WakeOnSupportCollidabilityDisabled)
{
  auto world = makeSleepWorld();
  auto floor = createFloor();
  world->addSkeleton(floor);

  auto sleeper = createFreeBox(
      "sleeper",
      Eigen::Vector3d::Constant(kBoxSize),
      Eigen::Vector3d(0, 0, kHalf + 0.02));
  world->addSkeleton(sleeper);

  ASSERT_NO_FATAL_FAILURE(stepUntilRestingFastPathReady(world.get(), sleeper));

  floor->getBodyNode(0)->setCollidable(false);
  expectSleeperFallsAfterSupportEdit(world.get(), sleeper);
}

//==============================================================================
// Solver collision-filter updates can remove support contacts without touching
// skeleton pose/version counters. The all-resting snapshot must include the
// filter revision so sleeping bodies wake instead of reusing the cached no-op
// step forever.
TEST(IslandDeactivation, WakeOnSolverCollisionFilterChange)
{
  auto world = makeSleepWorld();
  auto floor = createFloor();
  world->addSkeleton(floor);

  auto sleeper = createFreeBox(
      "sleeper",
      Eigen::Vector3d::Constant(kBoxSize),
      Eigen::Vector3d(0, 0, kHalf + 0.02));
  world->addSkeleton(sleeper);

  ASSERT_NO_FATAL_FAILURE(stepUntilRestingFastPathReady(world.get(), sleeper));

  auto* filter = dynamic_cast<BodyNodeCollisionFilter*>(
      world->getConstraintSolver()->getCollisionOption().collisionFilter.get());
  ASSERT_NE(filter, nullptr);
  filter->addBodyNodePairToBlackList(
      floor->getBodyNode(0), sleeper->getBodyNode(0));

  expectSleeperFallsAfterSupportEdit(world.get(), sleeper);
}

//==============================================================================
// Collision-shape edits under a static support must invalidate the all-resting
// snapshot even when the static skeleton pose itself does not change.
TEST(IslandDeactivation, WakeOnSupportShapeGeometryChange)
{
  auto world = makeSleepWorld();
  auto floor = createFloor();
  world->addSkeleton(floor);

  auto sleeper = createFreeBox(
      "sleeper",
      Eigen::Vector3d::Constant(kBoxSize),
      Eigen::Vector3d(0, 0, kHalf + 0.02));
  world->addSkeleton(sleeper);

  ASSERT_NO_FATAL_FAILURE(stepUntilRestingFastPathReady(world.get(), sleeper));

  auto* floorShapeNode = floor->getBodyNode(0)->getShapeNode(0);
  Eigen::Isometry3d moved = Eigen::Isometry3d::Identity();
  moved.translation() = Eigen::Vector3d(0.0, 0.0, -0.25);
  floorShapeNode->setRelativeTransform(moved);

  expectSleeperFallsAfterSupportEdit(world.get(), sleeper);

  ASSERT_NO_FATAL_FAILURE(
      stepUntilRestingFastPathReady(world.get(), sleeper, 8000));

  auto floorBox
      = std::dynamic_pointer_cast<BoxShape>(floorShapeNode->getShape());
  ASSERT_TRUE(floorBox);
  floorBox->setSize(Eigen::Vector3d(10.0, 10.0, 0.02));

  expectSleeperFallsAfterSupportEdit(world.get(), sleeper);
}

//==============================================================================
// If the all-resting snapshot is invalidated before a new snapshot is built,
// support edits must still wake resting bodies on the next step.
TEST(IslandDeactivation, WakeOnSupportShapeChangeAfterSnapshotInvalidation)
{
  auto world = makeSleepWorld();
  auto floor = createFloor();
  world->addSkeleton(floor);

  auto sleeper = createFreeBox(
      "sleeper",
      Eigen::Vector3d::Constant(kBoxSize),
      Eigen::Vector3d(0, 0, kHalf + 0.02));
  world->addSkeleton(sleeper);

  ASSERT_NO_FATAL_FAILURE(stepUntilRestingFastPathReady(world.get(), sleeper));

  world->setTimeStep(world->getTimeStep());
  ASSERT_TRUE(sleeper->isResting());

  auto* floorShapeNode = floor->getBodyNode(0)->getShapeNode(0);
  Eigen::Isometry3d moved = Eigen::Isometry3d::Identity();
  moved.translation() = Eigen::Vector3d(0.0, 0.0, -0.25);
  floorShapeNode->setRelativeTransform(moved);

  expectSleeperFallsAfterSupportEdit(world.get(), sleeper);
}

//==============================================================================
// Enabling automatic joint constraints can make a previously resting body need
// a solver pass even when its world pose has not otherwise changed.
TEST(IslandDeactivation, WakeOnAutomaticJointConstraintEnforcementChange)
{
  auto world = makeSleepWorld();
  world->addSkeleton(createFloor());

  auto sleeper = createFreeBox(
      "sleeper",
      Eigen::Vector3d::Constant(kBoxSize),
      Eigen::Vector3d(0, 0, kHalf + 0.02));
  world->addSkeleton(sleeper);

  ASSERT_NO_FATAL_FAILURE(stepUntilRestingFastPathReady(world.get(), sleeper));

  auto* joint = sleeper->getJoint(0);
  ASSERT_GT(joint->getNumDofs(), 0u);
  const double position = joint->getPosition(0);
  joint->setPositionLowerLimit(0, position + 0.05);
  joint->setPositionUpperLimit(0, position + 1.0);
  joint->setLimitEnforcement(true);

  expectSolverRunsAfterRestingConstraintEdit(world.get(), sleeper);
}

//==============================================================================
// Limit edits are automatic joint-constraint edits too: a joint that was
// already limit-enforced can become newly constrained without changing
// kinematics.
TEST(IslandDeactivation, WakeOnAutomaticJointLimitChange)
{
  auto world = makeSleepWorld();
  world->addSkeleton(createFloor());

  auto sleeper = createFreeBox(
      "sleeper",
      Eigen::Vector3d::Constant(kBoxSize),
      Eigen::Vector3d(0, 0, kHalf + 0.02));
  auto* joint = sleeper->getJoint(0);
  ASSERT_GT(joint->getNumDofs(), 0u);
  const double initialPosition = joint->getPosition(0);
  joint->setPositionLowerLimit(0, initialPosition - 1.0);
  joint->setPositionUpperLimit(0, initialPosition + 1.0);
  joint->setLimitEnforcement(true);
  world->addSkeleton(sleeper);

  ASSERT_NO_FATAL_FAILURE(stepUntilRestingFastPathReady(world.get(), sleeper));

  joint->setPositionLowerLimit(0, joint->getPosition(0) + 0.05);

  expectSolverRunsAfterRestingConstraintEdit(world.get(), sleeper);
}

//==============================================================================
// Joint-frame edits can change world geometry without changing generalized
// positions. The all-resting snapshot must invalidate on the kinematic version
// change and run a real collision pass.
TEST(IslandDeactivation, WakeOnSupportJointFrameChange)
{
  auto world = makeSleepWorld();
  auto floor = createFloor();
  world->addSkeleton(floor);

  auto sleeper = createFreeBox(
      "sleeper",
      Eigen::Vector3d::Constant(kBoxSize),
      Eigen::Vector3d(0, 0, kHalf + 0.02));
  world->addSkeleton(sleeper);

  ASSERT_NO_FATAL_FAILURE(stepUntilRestingFastPathReady(world.get(), sleeper));

  auto* joint = floor->getJoint(0);
  Eigen::Isometry3d moved = joint->getTransformFromParentBodyNode();
  moved.translation().z() -= 0.25;
  joint->setTransformFromParentBodyNode(moved);

  expectSleeperFallsAfterSupportEdit(world.get(), sleeper);
}

//==============================================================================
// The resting-contact filter is an internal solver optimization. Public
// collision queries that use BodyNodeCollisionFilter must still report contacts
// involving bodies that the solver has marked resting.
TEST(IslandDeactivation, PublicCollisionFilterReportsRestingContacts)
{
  auto world = makeSleepWorld();
  auto floor = createFloor();
  world->addSkeleton(floor);

  auto sleeper = createFreeBox(
      "sleeper",
      Eigen::Vector3d::Constant(kBoxSize),
      Eigen::Vector3d(0, 0, kHalf + 0.02));
  world->addSkeleton(sleeper);

  ASSERT_NO_FATAL_FAILURE(stepUntilRestingFastPathReady(world.get(), sleeper));

  auto detector = world->getConstraintSolver()->getCollisionDetector();
  auto group = detector->createCollisionGroup();
  group->addShapeFramesOf(floor.get());
  group->addShapeFramesOf(sleeper.get());

  CollisionOption option;
  option.collisionFilter = std::make_shared<BodyNodeCollisionFilter>();
  CollisionResult result;
  EXPECT_TRUE(group->collide(option, &result));
  EXPECT_GT(result.getNumContacts(), 0u);
}

//==============================================================================
// Stateful custom filters do not expose a revision, so the all-resting cache
// must opt out instead of reusing a snapshot after the filter's decision
// changes.
TEST(IslandDeactivation, CustomFilterDisablesAllRestingFastPath)
{
  auto world = makeSleepWorld();
  auto floor = createFloor();
  world->addSkeleton(floor);

  auto sleeper = createFreeBox(
      "sleeper",
      Eigen::Vector3d::Constant(kBoxSize),
      Eigen::Vector3d(0, 0, kHalf));
  world->addSkeleton(sleeper);

  auto filter = std::make_shared<TogglePairCollisionFilter>(
      floor->getBodyNode(0), sleeper->getBodyNode(0), true);
  world->getConstraintSolver()->getCollisionOption().collisionFilter = filter;

  sleeper->setSleepCandidate(true);
  sleeper->setIslandIndex(0);
  sleeper->setResting(true);

  world->step();
  ASSERT_EQ(0u, world->getLastCollisionResult().getNumContacts());
  world->step();
  ASSERT_EQ(0u, world->getLastCollisionResult().getNumContacts());

  filter->setIgnorePair(false);
  filter->resetNumCalls();
  world->step();
  EXPECT_GT(filter->getNumCalls(), 0u)
      << "custom BodyNodeCollisionFilter subclass was hidden by the "
         "all-resting fast path";
}

//==============================================================================
// Applying an external force to a sleeping body must wake it (via the
// wake-on-force path in step loop 1) and the body must accelerate.
TEST(IslandDeactivation, WakeOnExternalForce)
{
  auto world = makeSleepWorld();
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
// A generalized force written through the Skeleton/DOF API must also wake a
// sleeping body. This covers the all-resting fast path's force/command dirty
// cache, which is separate from BodyNode external-force dirty flags.
TEST(IslandDeactivation, WakeOnGeneralizedForce)
{
  auto world = makeSleepWorld();
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

  // Run one all-resting cached step first so the quiet force/command state is
  // cached, then mutate a generalized force. The next step must not reuse the
  // quiet cache.
  world->step();
  ASSERT_TRUE(box->isResting());

  box->setForce(3, 50.0);
  world->step();
  EXPECT_FALSE(box->isResting()) << "generalized force did not wake the body";
}

//==============================================================================
// A body with a pending external disturbance must not accumulate quiet dwell
// and freeze, even if the disturbance is small enough that the body remains
// below the sleep thresholds. This covers the pre-resetCommand disturbance
// bookkeeping path for bodies that are not already resting.
TEST(IslandDeactivation, ExternalDisturbancePreventsSleep)
{
  auto world = makeSleepWorld();
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
  auto world = makeSleepWorld();
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
// Resting is decided per independent contact island. A separate body in genuine
// motion must not prevent an otherwise settled floor contact from sleeping.
TEST(IslandDeactivation, IndependentQuietIslandSleepsWhileOtherBodyMoves)
{
  auto world = makeSleepWorld();
  auto opts = world->getDeactivationOptions();
  opts.mTimeUntilSleep = 0.05;
  world->setDeactivationOptions(opts);
  world->addSkeleton(createFloor());

  auto sleeper = createFreeBox(
      "sleeper",
      Eigen::Vector3d::Constant(kBoxSize),
      Eigen::Vector3d(0, 0, kHalf + 0.02));
  world->addSkeleton(sleeper);

  auto mover = createFreeBox(
      "mover",
      Eigen::Vector3d::Constant(kBoxSize),
      Eigen::Vector3d(3.0, 0, kHalf + 0.02));
  world->addSkeleton(mover);

  Eigen::Vector6d movingVelocity = Eigen::Vector6d::Zero();
  movingVelocity[3]
      = 2.0 * opts.mWakeThresholdScale * opts.mLinearSpeedThreshold;

  const std::size_t maxSteps = 5000;
  std::size_t steps = 0;
  for (; steps < maxSteps && !sleeper->isResting(); ++steps) {
    mover->getJoint(0)->setVelocities(movingVelocity);
    world->step();
    ASSERT_FALSE(mover->isResting())
        << "moving body slept before the quiet island at step " << steps;
  }

  EXPECT_LT(steps, maxSteps)
      << "quiet independent island did not sleep while another body moved";
  EXPECT_TRUE(sleeper->isResting());
}

//==============================================================================
// A body kept in genuine slow motion (above the sleep thresholds) must NOT be
// put to sleep within the test horizon. This is the "always beneficial"
// correctness gate: we must never freeze a body that is actually moving.
TEST(IslandDeactivation, NoPrematureSleepOfMovingBody)
{
  auto world = makeSleepWorld();
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
// With the feature disabled, the box must never report resting, and its
// trajectory must match the trajectory produced when the feature is enabled but
// the body simply never sleeps (i.e. identical physics). We compare a disabled
// run against a baseline run that disables sleeping by setting an unreachable
// dwell time, so both runs execute the exact same code path with no skips.
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

  // Run A: feature explicitly disabled.
  auto worldA = makeWorld();
  DeactivationOptions disabled;
  disabled.mEnabled = false;
  worldA->setDeactivationOptions(disabled);
  auto boxA = worldA->getSkeleton("box");

  // Run B (baseline): feature enabled but configured so the body can never
  // accumulate enough dwell time to sleep within the horizon. This produces the
  // "enabled-off" reference trajectory the disabled run must match.
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
  auto world = makeSleepWorld();
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
  auto world = makeSleepWorld();
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
  auto world = makeSleepWorld();
  world->addSkeleton(createFloor());
  auto box = createFreeBox(
      "box",
      Eigen::Vector3d::Constant(kBoxSize),
      Eigen::Vector3d(0, 0, kHalf + 0.02));
  world->addSkeleton(box);
  ASSERT_LT(
      stepUntil(world.get(), 5000, [&]() { return box->isResting(); }), 5000u);

  box->setCommands(Eigen::Vector6d::Constant(1.0));
  world->step();
  EXPECT_FALSE(box->isResting()) << "a set command should wake the body";
}

//==============================================================================
// A quiet command observed by step(false) is cached as non-disturbing, but a
// later step(true) still must honor the resetCommand contract and clear it.
TEST(IslandDeactivation, ResetCommandClearsCachedQuietCommand)
{
  auto world = makeSleepWorld();
  world->addSkeleton(createFloor());
  auto box = createFreeBox(
      "box",
      Eigen::Vector3d::Constant(kBoxSize),
      Eigen::Vector3d(0, 0, kHalf + 0.02));
  world->addSkeleton(box);
  ASSERT_NO_FATAL_FAILURE(stepUntilRestingFastPathReady(world.get(), box));

  const Eigen::Vector6d quietCommand = Eigen::Vector6d::Constant(1e-12);
  box->setCommands(quietCommand);
  world->step(false);
  EXPECT_TRUE(box->getCommands().isApprox(quietCommand, 0.0));
  ASSERT_TRUE(box->isResting());

  world->step(true);
  EXPECT_TRUE(box->getCommands().isZero(0.0))
      << "step(true) should clear cached quiet commands";
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
// Disabling deactivation must clear any existing resting state so subsequent
// steps process every skeleton normally.
TEST(IslandDeactivation, DisablingClearsRestState)
{
  auto world = makeSleepWorld();
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
// resting scenes, and negligible overhead for an always-active scene.
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
// island, all coming to rest. settleSteps lets them settle (and sleep, if
// enabled) before the timed window.
WorldPtr makeRestingGrid(int n, bool enabled)
{
  auto world = World::create();
  auto opts = world->getDeactivationOptions();
  opts.mEnabled = enabled;
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
  // Always-active scene: boxes kept in continuous motion so nothing ever
  // sleeps. Measures the detection overhead of the feature in the worst case
  // for it.
  std::cout << "\n[bench] active scene (never sleeps): detection overhead\n";
  const int n = 144;
  auto build = [&](bool enabled) {
    auto world = World::create();
    auto opts = world->getDeactivationOptions();
    opts.mEnabled = enabled;
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
