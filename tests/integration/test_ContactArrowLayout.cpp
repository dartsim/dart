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

// Gates for the demo contact-force arrow layout.
//
// The layout replaced a fixed 0.1 m-per-newton constant. That constant was
// tuned on a scene of 1 kg blocks; on anything heavier every arrow ran past the
// 5 m safety cap, so all arrows came out the same length -- a bundle of
// identical streaks whose heads sat meters away from the bodies they belonged
// to, destroying both the magnitude encoding and any sense that the arrows were
// attached to the contact at all.
//
// The gates below cover the three properties that failure violated: arrows stay
// anchored to their contact points, their length stays proportional to the
// scene rather than to an absolute constant, and their length still orders by
// force magnitude.

#include "examples/demos/ContactArrowLayout.hpp"

#include <dart/dart.hpp>

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include <cstddef>

namespace {

constexpr std::size_t kMaxArrows = 256;

/// Drives one update() with no contacts. update() is what re-derives the
/// scene-dependent references, so this is how a test advances the layout the
/// way ContactVisualizer does.
void pump(
    dart_demos::ContactArrowLayout& layout,
    const dart::simulation::WorldPtr& world)
{
  layout.update(*world, {}, kMaxArrows);
}

//==============================================================================
dart::collision::Contact makeContact(
    const Eigen::Vector3d& point, const Eigen::Vector3d& force)
{
  dart::collision::Contact contact;
  contact.point = point;
  contact.force = force;
  contact.normal = Eigen::Vector3d::UnitY();
  return contact;
}

//==============================================================================
/// A `mass` kg box of side `size`, dropped onto a large static ground box.
/// Returns the world; `boxOut` receives the falling box's skeleton.
dart::simulation::WorldPtr makeBoxOnGround(
    double mass, double size, dart::dynamics::SkeletonPtr& boxOut)
{
  auto world = dart::simulation::World::create("box_on_ground");
  world->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));
  world->setTimeStep(0.001);

  auto ground = dart::dynamics::Skeleton::create("ground");
  auto* groundBody
      = ground->createJointAndBodyNodePair<dart::dynamics::WeldJoint>().second;
  groundBody->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(
      std::make_shared<dart::dynamics::BoxShape>(
          Eigen::Vector3d(20.0, 0.2, 20.0)));
  Eigen::Isometry3d groundTf = Eigen::Isometry3d::Identity();
  groundTf.translation() = Eigen::Vector3d(0.0, -0.1, 0.0);
  groundBody->getParentJoint()->setTransformFromParentBodyNode(groundTf);
  world->addSkeleton(ground);

  auto box = dart::dynamics::Skeleton::create("box");
  auto* boxBody
      = box->createJointAndBodyNodePair<dart::dynamics::FreeJoint>().second;
  boxBody->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(
      std::make_shared<dart::dynamics::BoxShape>(
          Eigen::Vector3d::Constant(size)));
  dart::dynamics::Inertia inertia;
  inertia.setMass(mass);
  boxBody->setInertia(inertia);

  Eigen::Vector6d pose = Eigen::Vector6d::Zero();
  pose[4] = 0.5 * size + 0.01;
  box->getJoint(0)->setPositions(pose);
  world->addSkeleton(box);

  boxOut = box;
  return world;
}

} // namespace

//==============================================================================
// The reported symptom: arrow heads far from the bodies they belong to. Every
// arrow must start exactly at its contact point and end within one reference
// length of it, so an arrow can never appear detached from its contact.
TEST(ContactArrowLayoutTest, ArrowsStayAnchoredToTheirContactPoints)
{
  dart::dynamics::SkeletonPtr box;
  auto world = makeBoxOnGround(180.0, 0.3, box);

  dart_demos::ContactArrowLayout layout;
  layout.resetForWorld(*world);

  // Step long enough to cover the impact transient and the resting phase after
  // it, and slide the box so the contact points move between steps.
  bool sawContact = false;
  for (int step = 0; step < 400; ++step) {
    if (step == 100)
      box->getBodyNode(0)->addExtForce(Eigen::Vector3d(600.0, 0.0, 0.0));
    world->step();

    const auto& contacts = world->getLastCollisionResult().getContacts();
    const auto& arrows = layout.update(*world, contacts, kMaxArrows);
    if (arrows.empty())
      continue;
    sawContact = true;

    for (const auto& arrow : arrows) {
      // Anchored: the tail is a contact point of this very step.
      const bool matchesAContact = std::any_of(
          contacts.begin(),
          contacts.end(),
          [&arrow](const dart::collision::Contact& contact) {
            return contact.point.isApprox(arrow.tail);
          });
      EXPECT_TRUE(matchesAContact)
          << "arrow tail " << arrow.tail.transpose()
          << " is not a contact point of step " << step;

      // Bounded: the head cannot wander off toward the horizon.
      EXPECT_LE(
          (arrow.head - arrow.tail).norm(), layout.getReferenceLength() + 1e-12)
          << "arrow longer than the reference length at step " << step;
    }
  }
  EXPECT_TRUE(sawContact) << "the box never touched the ground";
}

//==============================================================================
// The cause: a fixed newtons-to-meters constant. The reference length must come
// from the scene, so the same heavy box that saturated the old constant now
// gets arrows sized to the box.
TEST(ContactArrowLayoutTest, ReferenceLengthTracksTheSceneNotTheForce)
{
  dart::dynamics::SkeletonPtr smallBox;
  auto smallWorld = makeBoxOnGround(180.0, 0.3, smallBox);
  dart_demos::ContactArrowLayout smallLayout;
  smallLayout.resetForWorld(*smallWorld);

  dart::dynamics::SkeletonPtr bigBox;
  auto bigWorld = makeBoxOnGround(180.0, 3.0, bigBox);
  dart_demos::ContactArrowLayout bigLayout;
  bigLayout.resetForWorld(*bigWorld);

  // Same mass, ten times the size: the arrows scale with the geometry.
  EXPECT_GT(
      bigLayout.getReferenceLength(), 5.0 * smallLayout.getReferenceLength());

  // And the small scene's arrows stay comparable to the box they annotate,
  // rather than to the 5 m cap the old fixed constant always hit.
  EXPECT_LT(smallLayout.getReferenceLength(), 1.0);

  // The old constant would have saturated here: a 180 kg box at rest pushes
  // roughly 1765 N through its contacts, and 0.1 m/N of that is 176 m, far
  // past the 5 m cap that clamped every arrow to the same length.
  const double restingForce = 180.0 * 9.81;
  EXPECT_GT(0.1 * restingForce, 5.0)
      << "this scene no longer reproduces the saturation being fixed";

  std::cout << "contact_arrow_scale  small_box_ref_len="
            << smallLayout.getReferenceLength()
            << " m  big_box_ref_len=" << bigLayout.getReferenceLength()
            << " m  old_fixed_scale_would_be=" << 0.1 * restingForce << " m\n";
}

//==============================================================================
// Length must still encode magnitude: that is what saturation destroyed.
TEST(ContactArrowLayoutTest, ArrowLengthOrdersByForceMagnitude)
{
  dart::dynamics::SkeletonPtr box;
  auto world = makeBoxOnGround(10.0, 0.5, box);
  dart_demos::ContactArrowLayout layout;
  layout.resetForWorld(*world);

  const Eigen::Vector3d up = Eigen::Vector3d::UnitY();
  const std::vector<dart::collision::Contact> contacts = {
      makeContact(Eigen::Vector3d(0.0, 0.0, 0.0), 100.0 * up),
      makeContact(Eigen::Vector3d(1.0, 0.0, 0.0), 50.0 * up),
      makeContact(Eigen::Vector3d(2.0, 0.0, 0.0), 25.0 * up),
  };

  const auto& arrows = layout.update(*world, contacts, kMaxArrows);
  ASSERT_EQ(arrows.size(), 3u);

  const double longest = (arrows[0].head - arrows[0].tail).norm();
  const double middle = (arrows[1].head - arrows[1].tail).norm();
  const double shortest = (arrows[2].head - arrows[2].tail).norm();

  // The peak contact draws at exactly the reference length, and the others in
  // proportion, so halving the force halves the arrow.
  EXPECT_NEAR(longest, layout.getReferenceLength(), 1e-12);
  EXPECT_NEAR(middle, 0.5 * longest, 1e-12);
  EXPECT_NEAR(shortest, 0.25 * longest, 1e-12);

  // Color is driven by the same normalized value, so the two always agree.
  EXPECT_NEAR(arrows[0].normalizedMagnitude, 1.0, 1e-12);
  EXPECT_NEAR(arrows[1].normalizedMagnitude, 0.5, 1e-12);
  EXPECT_NEAR(arrows[2].normalizedMagnitude, 0.25, 1e-12);
}

//==============================================================================
// A diverging solve must not poison the arrow mesh vertices, and resting
// near-zero contacts must not add degenerate zero-length arrows.
TEST(ContactArrowLayoutTest, DropsNonFiniteAndNegligibleContacts)
{
  dart::dynamics::SkeletonPtr box;
  auto world = makeBoxOnGround(10.0, 0.5, box);
  dart_demos::ContactArrowLayout layout;
  layout.resetForWorld(*world);

  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  const std::vector<dart::collision::Contact> contacts = {
      makeContact(Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 100.0, 0.0)),
      makeContact(Eigen::Vector3d(nan, 0.0, 0.0), Eigen::Vector3d::UnitY()),
      makeContact(Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, nan, 0.0)),
      makeContact(Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, inf, 0.0)),
      makeContact(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()),
  };

  const auto& arrows = layout.update(*world, contacts, kMaxArrows);
  ASSERT_EQ(arrows.size(), 1u) << "only the one finite, non-negligible contact "
                                  "should be laid out";
  EXPECT_TRUE(arrows[0].head.allFinite());
  EXPECT_TRUE(layout.getReferenceForce() > 0.0);
  EXPECT_TRUE(std::isfinite(layout.getReferenceForce()));
}

//==============================================================================
// After an impact spike the reference has to come back down, or every resting
// contact stays crushed to an invisible sliver for the rest of the run.
TEST(ContactArrowLayoutTest, ReferenceForceRecoversAfterASpike)
{
  dart::dynamics::SkeletonPtr box;
  auto world = makeBoxOnGround(10.0, 0.5, box);
  dart_demos::ContactArrowLayout layout;
  layout.resetForWorld(*world);

  const std::vector<dart::collision::Contact> spike = {
      makeContact(Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 5000.0, 0.0))};
  const std::vector<dart::collision::Contact> resting
      = {makeContact(Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 25.0, 0.0))};

  layout.update(*world, spike, kMaxArrows);
  const double duringSpike = layout.getReferenceForce();
  EXPECT_NEAR(duringSpike, 5000.0, 1e-9);

  // Immediately after the spike the resting contact is a sliver, as it should
  // be -- it really is tiny next to what just happened.
  layout.update(*world, resting, kMaxArrows);
  EXPECT_LT(layout.getArrows().front().normalizedMagnitude, 0.01);

  // One decay time later it is readable again.
  const int stepsPerDecayTime = static_cast<int>(
      dart_demos::ContactArrowLayout::kForceDecayTime / world->getTimeStep());
  for (int step = 0; step < 4 * stepsPerDecayTime; ++step)
    layout.update(*world, resting, kMaxArrows);

  EXPECT_LT(layout.getReferenceForce(), duringSpike);
  EXPECT_GT(layout.getArrows().front().normalizedMagnitude, 0.2)
      << "resting contacts never recovered from the spike";

  std::cout << "contact_arrow_recovery  spike_ref=" << duringSpike
            << " N  settled_ref=" << layout.getReferenceForce()
            << " N  settled_normalized="
            << layout.getArrows().front().normalizedMagnitude << "\n";
}

//==============================================================================
// A body that cannot collide cannot produce a contact, so it must not set the
// scale. The `sleeping` demo parks its projectile pool far off-scene with the
// bodies made noncollidable; counting those would stretch the reference length
// to the clamp and raise the force floor by their idle mass, recreating the
// detached-arrow symptom on the small boxes that are actually in play.
TEST(ContactArrowLayoutTest, IgnoresNoncollidableBodies)
{
  dart::dynamics::SkeletonPtr box;
  auto world = makeBoxOnGround(10.0, 0.3, box);

  dart_demos::ContactArrowLayout withoutPool;
  withoutPool.resetForWorld(*world);
  const double baselineLength = withoutPool.getReferenceLength();

  // Two heavy projectiles parked far away, exactly as the sleeping scene does.
  for (int i = 0; i < 2; ++i) {
    auto parked
        = dart::dynamics::Skeleton::create("parked" + std::to_string(i));
    auto* body = parked->createJointAndBodyNodePair<dart::dynamics::FreeJoint>()
                     .second;
    body->createShapeNodeWith<
        dart::dynamics::VisualAspect,
        dart::dynamics::CollisionAspect,
        dart::dynamics::DynamicsAspect>(
        std::make_shared<dart::dynamics::BoxShape>(
            Eigen::Vector3d::Constant(0.5)));
    dart::dynamics::Inertia inertia;
    inertia.setMass(468.0);
    body->setInertia(inertia);
    body->setCollidable(false);

    Eigen::Vector6d pose = Eigen::Vector6d::Zero();
    pose.tail<3>() = Eigen::Vector3d(60.0, 60.0, 10.0 + i);
    parked->getJoint(0)->setPositions(pose);
    world->addSkeleton(parked);
  }

  dart_demos::ContactArrowLayout withPool;
  withPool.resetForWorld(*world);

  EXPECT_DOUBLE_EQ(withPool.getReferenceLength(), baselineLength)
      << "parked noncollidable bodies changed the arrow scale";
  EXPECT_LT(withPool.getReferenceLength(), 1.0);

  // The force floor must not have moved either: their mass is not carried by
  // any contact.
  const std::vector<dart::collision::Contact> contacts
      = {makeContact(Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 30.0, 0.0))};
  const auto& arrows = withPool.update(*world, contacts, kMaxArrows);
  ASSERT_EQ(arrows.size(), 1u);
  EXPECT_NEAR(arrows[0].normalizedMagnitude, 1.0, 1e-12)
      << "the idle pool's mass raised the force floor";

  std::cout << "contact_arrow_noncollidable  baseline_ref_len="
            << baselineLength
            << " m  with_parked_pool=" << withPool.getReferenceLength()
            << " m  ref_force=" << withPool.getReferenceForce() << " N\n";
}

//==============================================================================
// The host lets the timestep change while a scene runs, so the decay has to
// follow it rather than whatever it was when the scene was installed.
TEST(ContactArrowLayoutTest, DecayFollowsTheLiveTimestep)
{
  dart::dynamics::SkeletonPtr box;
  auto world = makeBoxOnGround(10.0, 0.5, box);

  const std::vector<dart::collision::Contact> spike = {
      makeContact(Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 5000.0, 0.0))};
  const std::vector<dart::collision::Contact> resting
      = {makeContact(Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 25.0, 0.0))};

  // Same elapsed simulated time, ten times the timestep, so a tenth of the
  // steps: the reference must land in the same place.
  const auto referenceAfter = [&](double dt, int steps) {
    world->setTimeStep(dt);
    dart_demos::ContactArrowLayout layout;
    layout.resetForWorld(*world);
    layout.update(*world, spike, kMaxArrows);
    for (int s = 0; s < steps; ++s)
      layout.update(*world, resting, kMaxArrows);
    return layout.getReferenceForce();
  };

  const double fine = referenceAfter(0.001, 500);
  const double coarse = referenceAfter(0.01, 50);
  EXPECT_NEAR(fine, coarse, 0.02 * fine)
      << "the force reference decayed by step count rather than by time";

  std::cout << "contact_arrow_decay  dt=0.001 after 0.5 s -> " << fine
            << " N   dt=0.01 after 0.5 s -> " << coarse << " N\n";
}

//==============================================================================
// Scenes such as `add_delete_skels` and `rigid_shapes` spawn and remove bodies
// while running. A scale derived only at install time would stay stuck on
// whatever the world held at startup -- for `add_delete_skels` that is a bare
// welded ground, so every spawned cube would be annotated with fallback-sized
// arrows and a near-zero force floor.
TEST(ContactArrowLayoutTest, RefreshesWhenTheSceneGainsBodies)
{
  auto world = dart::simulation::World::create("empty_but_ground");
  world->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));
  world->setTimeStep(0.001);
  auto ground = dart::dynamics::Skeleton::create("ground");
  auto* groundBody
      = ground->createJointAndBodyNodePair<dart::dynamics::WeldJoint>().second;
  groundBody->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(
      std::make_shared<dart::dynamics::BoxShape>(
          Eigen::Vector3d(20.0, 0.2, 20.0)));
  world->addSkeleton(ground);

  dart_demos::ContactArrowLayout layout;
  layout.resetForWorld(*world);

  // Nothing mobile yet, so the fallback is all there is to go on.
  EXPECT_DOUBLE_EQ(
      layout.getReferenceLength(),
      dart_demos::ContactArrowLayout::kFallbackReferenceLength);

  // A no-op update must not disturb anything.
  pump(layout, world);
  EXPECT_DOUBLE_EQ(
      layout.getReferenceLength(),
      dart_demos::ContactArrowLayout::kFallbackReferenceLength);

  // Spawn a cube, the way add_delete_skels does on 'q'.
  auto cube = dart::dynamics::Skeleton::create("cube");
  auto* cubeBody
      = cube->createJointAndBodyNodePair<dart::dynamics::FreeJoint>().second;
  cubeBody->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(
      std::make_shared<dart::dynamics::BoxShape>(
          Eigen::Vector3d::Constant(1.2)));
  dart::dynamics::Inertia inertia;
  inertia.setMass(4.0);
  cubeBody->setInertia(inertia);
  world->addSkeleton(cube);

  // The peak the layout has been tracking survives the refresh.
  const std::vector<dart::collision::Contact> contacts = {
      makeContact(Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 900.0, 0.0))};
  layout.update(*world, contacts, kMaxArrows);
  const double trackedForce = layout.getReferenceForce();
  ASSERT_NEAR(trackedForce, 900.0, 1e-9);

  pump(layout, world);

  EXPECT_GT(layout.getReferenceLength(), 0.25)
      << "the spawned cube did not reach the arrow scale";
  // Within one step's decay of what it was: re-deriving the scale must not
  // throw the tracked peak away and drop back to the floor.
  EXPECT_GT(layout.getReferenceForce(), 0.99 * trackedForce)
      << "re-deriving the scale discarded the force reference being tracked";

  std::cout << "contact_arrow_refresh  ground_only_ref_len="
            << dart_demos::ContactArrowLayout::kFallbackReferenceLength
            << " m  after_cube_spawn=" << layout.getReferenceLength()
            << " m  tracked_force_kept=" << layout.getReferenceForce()
            << " N\n";
}

//==============================================================================
// The `sleeping` scene launches and reparks a preallocated projectile pool by
// toggling collidability, without adding or removing any skeleton, body or
// degree of freedom. A fingerprint built from counts alone would never notice.
TEST(ContactArrowLayoutTest, RefreshesWhenCollidabilityChanges)
{
  dart::dynamics::SkeletonPtr box;
  auto world = makeBoxOnGround(10.0, 0.3, box);

  auto projectile = dart::dynamics::Skeleton::create("projectile");
  auto* projectileBody
      = projectile->createJointAndBodyNodePair<dart::dynamics::FreeJoint>()
            .second;
  projectileBody->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(
      std::make_shared<dart::dynamics::BoxShape>(
          Eigen::Vector3d::Constant(0.4)));
  dart::dynamics::Inertia inertia;
  inertia.setMass(20.0);
  projectileBody->setInertia(inertia);
  projectileBody->setCollidable(false);

  Eigen::Vector6d parked = Eigen::Vector6d::Zero();
  parked.tail<3>() = Eigen::Vector3d(60.0, 60.0, 10.0);
  projectile->getJoint(0)->setPositions(parked);
  world->addSkeleton(projectile);

  dart_demos::ContactArrowLayout layout;
  layout.resetForWorld(*world);
  const double parkedLength = layout.getReferenceLength();

  // Launch it: same counts, different collidability and position.
  Eigen::Vector6d launched = Eigen::Vector6d::Zero();
  launched.tail<3>() = Eigen::Vector3d(1.5, 1.0, 0.0);
  projectile->getJoint(0)->setPositions(launched);
  projectileBody->setCollidable(true);

  pump(layout, world);
  const double activeLength = layout.getReferenceLength();
  EXPECT_GT(activeLength, parkedLength)
      << "an active projectile never reached the arrow scale";

  // Repark it and the scale comes back.
  projectile->getJoint(0)->setPositions(parked);
  projectileBody->setCollidable(false);
  pump(layout, world);
  EXPECT_DOUBLE_EQ(layout.getReferenceLength(), parkedLength)
      << "reparking the projectile left the arrow scale stretched";

  std::cout << "contact_arrow_collidability  parked=" << parkedLength
            << " m  launched=" << activeLength
            << " m  reparked=" << layout.getReferenceLength() << " m\n";
}

//==============================================================================
// A force whose components are each finite can still overflow to infinity under
// norm(). An infinite reference would normalize that contact as inf/inf, put a
// NaN in the arrow head, and stay infinite for every later step.
TEST(ContactArrowLayoutTest, RejectsForcesThatOverflowUnderNorm)
{
  dart::dynamics::SkeletonPtr box;
  auto world = makeBoxOnGround(10.0, 0.5, box);
  dart_demos::ContactArrowLayout layout;
  layout.resetForWorld(*world);

  const double huge = 1e200; // finite, but 3 * huge^2 overflows
  const std::vector<dart::collision::Contact> contacts = {
      makeContact(Eigen::Vector3d::Zero(), Eigen::Vector3d::Constant(huge)),
      makeContact(Eigen::Vector3d::UnitX(), Eigen::Vector3d(0.0, 40.0, 0.0)),
  };
  ASSERT_TRUE(Eigen::Vector3d::Constant(huge).allFinite())
      << "this contact no longer exercises the overflow path";
  ASSERT_FALSE(std::isfinite(Eigen::Vector3d::Constant(huge).norm()))
      << "this contact no longer overflows under norm()";

  const auto& arrows = layout.update(*world, contacts, kMaxArrows);
  ASSERT_EQ(arrows.size(), 1u) << "the overflowing contact should be dropped";
  EXPECT_TRUE(arrows[0].head.allFinite());
  EXPECT_TRUE(std::isfinite(layout.getReferenceForce()));

  // And the reference must not have been poisoned for later steps.
  const std::vector<dart::collision::Contact> normal
      = {makeContact(Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 40.0, 0.0))};
  const auto& later = layout.update(*world, normal, kMaxArrows);
  ASSERT_EQ(later.size(), 1u);
  EXPECT_TRUE(later[0].head.allFinite());
  EXPECT_NEAR(later[0].normalizedMagnitude, 1.0, 1e-12);
}

//==============================================================================
// Gravity is a live control: the host has a Gravity checkbox and some scenes
// have a gravity slider. The force floor is derived from the weight the
// contacts carry, so it has to follow.
TEST(ContactArrowLayoutTest, ForceFloorFollowsLiveGravity)
{
  dart::dynamics::SkeletonPtr box;
  auto world = makeBoxOnGround(100.0, 0.5, box);
  dart_demos::ContactArrowLayout layout;
  layout.resetForWorld(*world);

  // 100 kg at 9.81 puts the floor at 5% of ~981 N.
  const double weightedFloor = layout.getReferenceForce();
  EXPECT_NEAR(weightedFloor, 0.05 * 100.0 * 9.81, 1e-9);

  // A small contact is compressed against that floor, as it should be.
  const std::vector<dart::collision::Contact> small
      = {makeContact(Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 5.0, 0.0))};
  layout.update(*world, small, kMaxArrows);
  EXPECT_LT(layout.getArrows().front().normalizedMagnitude, 0.2);

  // Switch gravity off. There is no weight left to carry, so that same contact
  // must become readable instead of staying crushed by the old floor.
  world->setGravity(Eigen::Vector3d::Zero());
  for (int s = 0; s < 3000; ++s) // let the tracked peak decay away
    layout.update(*world, small, kMaxArrows);
  const double zeroGravityReference = layout.getReferenceForce();
  EXPECT_LT(zeroGravityReference, weightedFloor);
  EXPECT_NEAR(layout.getArrows().front().normalizedMagnitude, 1.0, 1e-9)
      << "the floor stayed at the old weight after gravity was switched off";

  // Switch it back on and the floor returns.
  world->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));
  pump(layout, world);
  EXPECT_NEAR(layout.getReferenceForce(), weightedFloor, 1e-9)
      << "the floor did not come back when gravity was restored";

  std::cout << "contact_arrow_gravity  with_gravity_floor=" << weightedFloor
            << " N  zero_gravity_ref=" << zeroGravityReference
            << " N  restored_floor=" << layout.getReferenceForce() << " N\n";
}

//==============================================================================
// `rigid_shapes` lets a user delete a shape and spawn a differently-sized
// replacement. If both happen before the next update, every count is identical,
// so a fingerprint built from topology alone would never re-derive the scale.
TEST(ContactArrowLayoutTest, RefreshesWhenASkeletonIsReplacedBySameTopology)
{
  dart::dynamics::SkeletonPtr box;
  auto world = makeBoxOnGround(10.0, 0.3, box);

  dart_demos::ContactArrowLayout layout;
  layout.resetForWorld(*world);
  const double smallLength = layout.getReferenceLength();

  // Same one body, same six degrees of freedom, ten times the size.
  world->removeSkeleton(box);
  auto replacement = dart::dynamics::Skeleton::create("box");
  auto* body
      = replacement->createJointAndBodyNodePair<dart::dynamics::FreeJoint>()
            .second;
  body->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(
      std::make_shared<dart::dynamics::BoxShape>(
          Eigen::Vector3d::Constant(3.0)));
  dart::dynamics::Inertia inertia;
  inertia.setMass(10.0);
  body->setInertia(inertia);
  world->addSkeleton(replacement);

  ASSERT_EQ(world->getNumSkeletons(), 2u);
  ASSERT_EQ(replacement->getNumBodyNodes(), box->getNumBodyNodes());
  ASSERT_EQ(replacement->getNumDofs(), box->getNumDofs());

  pump(layout, world);
  EXPECT_GT(layout.getReferenceLength(), 2.0 * smallLength)
      << "a same-topology replacement of a different size did not reach the "
         "arrow scale";

  std::cout << "contact_arrow_replacement  before=" << smallLength
            << " m  after=" << layout.getReferenceLength() << " m\n";
}
