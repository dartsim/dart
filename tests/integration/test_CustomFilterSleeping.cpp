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

#include <dart/simulation/World.hpp>

#include <dart/constraint/ConstraintSolver.hpp>

#include <dart/collision/CollisionFilter.hpp>
#include <dart/collision/CollisionResult.hpp>
#include <dart/collision/dart/DARTCollisionDetector.hpp>
#include <dart/collision/detail/CollisionFilterSnapshotTracker.hpp>
#include <dart/collision/fcl/FCLCollisionDetector.hpp>

#include <dart/dynamics/BallJoint.hpp>
#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/BoxShape.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/RevoluteJoint.hpp>
#include <dart/dynamics/ShapeNode.hpp>
#include <dart/dynamics/Skeleton.hpp>
#include <dart/dynamics/WeldJoint.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <string>

#include <cstddef>

using namespace dart;

namespace {

class CustomFilter : public collision::BodyNodeCollisionFilter
{
};

class TrackedCustomFilter
  : public CustomFilter,
    public collision::detail::CollisionFilterSnapshotTracker
{
public:
  std::size_t getCollisionFilterSnapshotRevision() const override
  {
    return 0;
  }
};

simulation::WorldPtr createWorld(double timeStep = 0.001, bool tracked = false)
{
  auto world = simulation::World::create();
  world->setTimeStep(timeStep);
  world->setCollisionDetector(collision::DARTCollisionDetector::create());
  if (tracked) {
    world->getConstraintSolver()->getCollisionOption().collisionFilter
        = std::make_shared<TrackedCustomFilter>();
  } else {
    world->getConstraintSolver()->getCollisionOption().collisionFilter
        = std::make_shared<CustomFilter>();
  }
  return world;
}

dynamics::SkeletonPtr createBox(
    const std::string& name,
    const Eigen::Vector3d& size,
    const Eigen::Vector3d& position,
    double mass = 1.0)
{
  auto skeleton = dynamics::Skeleton::create(name);
  auto* body
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>().second;
  dynamics::Inertia inertia;
  inertia.setMass(mass);
  inertia.setMoment(dynamics::BoxShape::computeInertia(size, mass));
  body->setInertia(inertia);
  body->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(std::make_shared<dynamics::BoxShape>(size));
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = position;
  skeleton->getRootJoint()->setPositions(
      dynamics::FreeJoint::convertToPositions(transform));
  return skeleton;
}

dynamics::SkeletonPtr createFloor()
{
  auto floor = dynamics::Skeleton::create("floor");
  auto pair = floor->createJointAndBodyNodePair<dynamics::WeldJoint>();
  pair.second->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(
      std::make_shared<dynamics::BoxShape>(Eigen::Vector3d(10.0, 10.0, 0.1)));
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation().z() = -0.05;
  pair.first->setTransformFromParentBodyNode(transform);
  floor->setMobile(false);
  return floor;
}

bool settle(
    simulation::World& world,
    const dynamics::Skeleton& skeleton,
    std::size_t maxSteps = 2000)
{
  for (std::size_t i = 0; i < maxSteps; ++i) {
    world.step();
    if (skeleton.isResting())
      return true;
  }
  return false;
}

struct JointScene
{
  simulation::WorldPtr world;
  dynamics::SkeletonPtr model;
  dynamics::BodyNode* flap;
};

enum class JointMount
{
  Hinge,
  WorldBall,
  DoorBall
};

JointScene createJointScene(
    double timeStep, bool deactivation, JointMount mount, bool tracked = false)
{
  JointScene scene;
  scene.world = createWorld(timeStep, tracked);
  scene.world->setCollisionDetector(collision::FCLCollisionDetector::create());
  auto options = scene.world->getDeactivationOptions();
  options.mEnabled = deactivation;
  scene.world->setDeactivationOptions(options);
  scene.world->addSkeleton(createFloor());

  const bool door = mount == JointMount::DoorBall;
  const Eigen::Vector3d size
      = door ? Eigen::Vector3d(0.05, 1.0, 2.0) : Eigen::Vector3d(0.5, 0.5, 0.1);
  const Eigen::Vector3d edge(0.25, 0.0, 0.0);
  const double mass = door ? 20.0 : 1.0;
  dynamics::BodyNode::Properties properties;
  properties.mInertia.setMass(mass);
  properties.mInertia.setMoment(dynamics::BoxShape::computeInertia(size, mass));
  if (mount != JointMount::Hinge) {
    scene.model = dynamics::Skeleton::create("model");
    dynamics::BallJoint::Properties joint;
    if (door) {
      joint.mT_ParentBodyToJoint.translation() = Eigen::Vector3d(0.0, 0.0, 1.0);
      joint.mT_ChildBodyToJoint.translation() = Eigen::Vector3d(0.0, -0.5, 0.0);
    } else {
      joint.mT_ParentBodyToJoint.translation()
          = edge + Eigen::Vector3d(0.0, 0.0, 0.05);
      joint.mT_ChildBodyToJoint.translation() = -edge;
    }
    scene.flap = scene.model
                     ->createJointAndBodyNodePair<dynamics::BallJoint>(
                         nullptr, joint, properties)
                     .second;
  } else {
    scene.model
        = createBox("model", size, Eigen::Vector3d(0.0, 0.0, 0.05), 5.0);
    dynamics::RevoluteJoint::Properties joint;
    joint.mAxis = Eigen::Vector3d::UnitY();
    joint.mT_ParentBodyToJoint.translation() = edge;
    joint.mT_ChildBodyToJoint.translation() = -edge;
    scene.flap = scene.model
                     ->createJointAndBodyNodePair<dynamics::RevoluteJoint>(
                         scene.model->getBodyNode(0), joint, properties)
                     .second;
  }
  scene.flap->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(std::make_shared<dynamics::BoxShape>(size));
  scene.world->addSkeleton(scene.model);
  return scene;
}

Eigen::Vector3d contactForce(const JointScene& scene)
{
  Eigen::Vector3d force = Eigen::Vector3d::Zero();
  const auto& result = scene.world->getLastCollisionResult();
  for (std::size_t i = 0; i < result.getNumContacts(); ++i) {
    const auto& contact = result.getContact(i);
    if (contact.getBodyNodePtr1().get() == scene.flap)
      force += contact.force;
    else if (contact.getBodyNodePtr2().get() == scene.flap)
      force -= contact.force;
  }
  return force;
}

} // namespace

TEST(CustomFilterSleeping, ActiveMaterialWritesKeepUnrelatedIslandAsleep)
{
  for (const bool tracked : {false, true}) {
    SCOPED_TRACE(tracked ? "tracked custom filter" : "custom filter");
    auto world = createWorld(0.001, tracked);
    world->addSkeleton(createFloor());
    auto sleeper = createBox(
        "sleeper",
        Eigen::Vector3d::Constant(0.2),
        Eigen::Vector3d(1.0, 0.0, 0.1));
    auto driver = createBox(
        "driver",
        Eigen::Vector3d::Constant(0.2),
        Eigen::Vector3d(-2.0, 0.0, 0.1));
    auto* material = driver->getBodyNode(0)
                         ->getShapeNode(0)
                         ->get<dynamics::DynamicsAspect>();
    material->setFrictionCoeff(0.0);
    driver->setVelocity(3, 0.5);
    world->addSkeleton(sleeper);
    world->addSkeleton(driver);

    for (std::size_t i = 0; i < 1000 && !sleeper->isResting(); ++i) {
      material->setPrimarySlipCompliance(0.01 + static_cast<double>(i) * 1e-6);
      world->step();
    }
    ASSERT_TRUE(sleeper->isResting());

    for (std::size_t i = 0; i < 100; ++i) {
      ASSERT_FALSE(driver->isResting());
      ASSERT_FALSE(driver->isSleepCandidate());
      ASSERT_DOUBLE_EQ(0.0, driver->getRestDwellTime());
      material->setPrimarySlipCompliance(0.01 + static_cast<double>(i) * 1e-6);
      world->step();
      EXPECT_TRUE(sleeper->isResting())
          << "unrelated island woke at step " << i;
    }
    EXPECT_GT(driver->getBodyNode(0)->getLinearVelocity().x(), 0.4);

    // The edited body's contacts must still wake the sleeper when it reaches
    // it.
    driver->setVelocity(3, 2.0);
    world->step();
    ASSERT_TRUE(sleeper->isResting());
    bool wokeOnContact = false;
    for (std::size_t i = 0; i < 2000 && !wokeOnContact; ++i) {
      material->setPrimarySlipCompliance(0.02 + static_cast<double>(i) * 1e-6);
      world->step();
      wokeOnContact = !sleeper->isResting();
    }
    EXPECT_TRUE(wokeOnContact);
    EXPECT_GT(sleeper->getBodyNode(0)->getLinearVelocity().x(), 0.1);
  }
}

TEST(CustomFilterSleeping, RestingAndStaticMaterialEditsStillWake)
{
  for (const bool tracked : {false, true}) {
    SCOPED_TRACE(tracked ? "tracked custom filter" : "custom filter");
    for (const bool editSupport : {false, true}) {
      SCOPED_TRACE(editSupport ? "static support" : "resting body");
      auto world = createWorld(0.001, tracked);
      auto floor = createFloor();
      auto sleeper = createBox(
          "sleeper",
          Eigen::Vector3d::Constant(0.2),
          Eigen::Vector3d(0.0, 0.0, 0.1));
      world->addSkeleton(floor);
      world->addSkeleton(sleeper);
      ASSERT_TRUE(settle(*world, *sleeper));
      world->step();
      auto target = editSupport ? floor : sleeper;
      target->getBodyNode(0)
          ->getShapeNode(0)
          ->get<dynamics::DynamicsAspect>()
          ->setFrictionCoeff(0.0);
      world->step();
      EXPECT_FALSE(sleeper->isResting());
      EXPECT_GT(
          world->getConstraintSolver()
              ->getLastCollisionResult()
              .getNumContacts(),
          0u);
    }
  }
}

// Joint reactions can relax over many contact solves even after motion is
// negligible. Sleeping must preserve a settled wrench at either time step.
TEST(CustomFilterSleeping, JointCoupledDwellCountsContactSolves)
{
  for (const bool tracked : {false, true}) {
    SCOPED_TRACE(tracked ? "tracked custom filter" : "custom filter");
    for (const double timeStep : {0.001, 0.002}) {
      for (const auto mount :
           {JointMount::Hinge, JointMount::WorldBall, JointMount::DoorBall}) {
        SCOPED_TRACE(timeStep);
        SCOPED_TRACE(static_cast<int>(mount));
        auto sleeping = createJointScene(timeStep, true, mount, tracked);
        auto awake = createJointScene(timeStep, false, mount, tracked);
        std::size_t sleepStep = 0;
        for (std::size_t i = 1; i <= 3000; ++i) {
          sleeping.world->step();
          awake.world->step();
          if (sleepStep == 0 && sleeping.model->isResting())
            sleepStep = i;
        }
        ASSERT_NE(0u, sleepStep);
        EXPECT_GE(sleepStep, 2000u);
        const double tolerance = 0.001 * sleeping.flap->getMass() * 9.81;
        EXPECT_LT(
            (sleeping.flap->getBodyForce() - awake.flap->getBodyForce())
                .cwiseAbs()
                .maxCoeff(),
            tolerance);
        EXPECT_LT(
            (contactForce(sleeping) - contactForce(awake))
                .cwiseAbs()
                .maxCoeff(),
            tolerance);
      }
    }
  }
}

TEST(CustomFilterSleeping, DenseIslandCannotShortenJointCoupledDwell)
{
  for (const bool tracked : {false, true}) {
    SCOPED_TRACE(tracked ? "tracked custom filter" : "custom filter");
    auto scene = createJointScene(0.002, true, JointMount::Hinge, tracked);
    auto left = createBox(
        "left",
        Eigen::Vector3d::Constant(0.2),
        Eigen::Vector3d(-0.1, 0.0, 0.2));
    auto right = createBox(
        "right",
        Eigen::Vector3d::Constant(0.2),
        Eigen::Vector3d(0.1, 0.0, 0.2));
    scene.world->addSkeleton(left);
    scene.world->addSkeleton(right);
    bool sharedIsland = false;
    std::size_t sleepStep = 0;
    for (std::size_t i = 1; i <= 3000; ++i) {
      scene.world->step();
      const int island = scene.model->getIslandIndex();
      sharedIsland = sharedIsland
                     || (island >= 0 && island == left->getIslandIndex()
                         && island == right->getIslandIndex());
      if (scene.model->isResting()) {
        sleepStep = i;
        break;
      }
    }
    ASSERT_TRUE(sharedIsland);
    ASSERT_NE(0u, sleepStep);
    EXPECT_GE(sleepStep, 2000u);
  }
}

TEST(CustomFilterSleeping, TimeStepChangeRestartsJointCoupledDwell)
{
  for (const bool tracked : {false, true}) {
    SCOPED_TRACE(tracked ? "tracked custom filter" : "custom filter");
    auto scene = createJointScene(0.002, true, JointMount::Hinge, tracked);
    for (std::size_t i = 0; i < 1000; ++i)
      scene.world->step();
    ASSERT_FALSE(scene.model->isResting());

    scene.world->setTimeStep(0.001);
    std::size_t sleepStep = 0;
    for (std::size_t i = 1; i <= 3000; ++i) {
      scene.world->setTimeStep(scene.world->getTimeStep());
      scene.world->step();
      if (scene.model->isResting()) {
        sleepStep = i;
        break;
      }
    }
    ASSERT_NE(0u, sleepStep);
    EXPECT_GE(sleepStep, 2000u);
  }
}

TEST(CustomFilterSleeping, JointDwellDoesNotAccrueOutsideSolvedContactIsland)
{
  for (const bool tracked : {false, true}) {
    SCOPED_TRACE(tracked ? "tracked custom filter" : "custom filter");
    auto scene = createJointScene(0.001, true, JointMount::WorldBall, tracked);
    auto* joint = scene.model->getRootJoint();
    auto transform = joint->getTransformFromParentBodyNode();
    transform.translation().z() += 1.0;
    joint->setTransformFromParentBodyNode(transform);
    scene.flap->setGravityMode(false);
    scene.world->addSkeleton(createBox(
        "supported",
        Eigen::Vector3d::Constant(0.2),
        Eigen::Vector3d(-2.0, 0.0, 0.1)));
    for (std::size_t i = 0; i < 5; ++i)
      scene.world->step();
    ASSERT_LT(scene.model->getIslandIndex(), 0);
    ASSERT_DOUBLE_EQ(0.0, scene.model->getRestDwellTime());

    // A temporarily missed support can leave quiet dwell from earlier solves.
    scene.model->setRestDwellTime(1.0);
    for (std::size_t i = 0; i < 100; ++i) {
      scene.world->step();
      ASSERT_GT(
          scene.world->getConstraintSolver()
              ->getLastCollisionResult()
              .getNumContacts(),
          0u);
      ASSERT_LT(scene.model->getIslandIndex(), 0);
      ASSERT_DOUBLE_EQ(0.0, scene.model->getRestDwellTime());
      ASSERT_FALSE(scene.model->isSleepCandidate());
      ASSERT_FALSE(scene.model->isResting());
    }
  }
}

TEST(CustomFilterSleeping, StaticSupportBecomingMobileInvalidatesReadyCache)
{
  for (const bool tracked : {false, true}) {
    SCOPED_TRACE(tracked ? "tracked custom filter" : "custom filter");
    auto world = createWorld(0.001, tracked);
    auto support = createBox(
        "support",
        Eigen::Vector3d(2.0, 2.0, 0.2),
        Eigen::Vector3d(0.0, 0.0, -0.1),
        10.0);
    support->setMobile(false);
    auto sleeper = createBox(
        "sleeper",
        Eigen::Vector3d::Constant(0.2),
        Eigen::Vector3d(0.0, 0.0, 0.1));
    world->addSkeleton(support);
    world->addSkeleton(sleeper);
    ASSERT_TRUE(settle(*world, *sleeper));
    for (std::size_t i = 0; i < 10; ++i)
      world->step();
    ASSERT_EQ(
        0u,
        world->getConstraintSolver()
            ->getLastCollisionResult()
            .getNumContacts());
    ASSERT_TRUE(sleeper->isResting());

    support->setMobile(true);
    world->step();
    EXPECT_LT(support->getBodyNode(0)->getLinearVelocity().z(), -0.005);
    EXPECT_FALSE(sleeper->isResting());
  }
}

TEST(CustomFilterSleeping, SolverOnlySupportKeepsBodiesAwake)
{
  for (const bool replaceSleepingSupport : {false, true}) {
    SCOPED_TRACE(
        replaceSleepingSupport ? "replace support after sleep"
                               : "solver-only support from the first step");
    auto world = createWorld();
    auto support = createFloor();
    auto box = createBox(
        "box", Eigen::Vector3d::Constant(0.2), Eigen::Vector3d(0.0, 0.0, 0.1));
    world->addSkeleton(box);
    if (replaceSleepingSupport) {
      auto ownedSupport = createFloor();
      world->addSkeleton(ownedSupport);
      ASSERT_TRUE(settle(*world, *box));
      world->getConstraintSolver()->removeSkeleton(ownedSupport);
    }
    world->getConstraintSolver()->addSkeleton(support);
    ASSERT_EQ(
        replaceSleepingSupport ? world->getNumSkeletons() : 2u,
        world->getConstraintSolver()->getSkeletons().size());

    std::size_t restingSteps = 0;
    for (std::size_t i = 0; i < 1000; ++i) {
      world->step();
      restingSteps += box->isResting() ? 1u : 0u;
    }
    EXPECT_EQ(0u, restingSteps);
    EXPECT_FALSE(box->isSleepCandidate());

    const double startZ = box->getBodyNode(0)->getTransform().translation().z();
    auto transform = support->getRootJoint()->getTransformFromParentBodyNode();
    transform.translation().x() += 10.0;
    support->getRootJoint()->setTransformFromParentBodyNode(transform);
    for (std::size_t i = 0; i < 40; ++i)
      world->step();
    EXPECT_LT(
        box->getBodyNode(0)->getTransform().translation().z(), startZ - 0.005);
    EXPECT_FALSE(box->isResting());
  }
}

TEST(CustomFilterSleeping, SolverOnlySupportPreservesDefaultFilterSleep)
{
  auto world = createWorld();
  world->getConstraintSolver()->getCollisionOption().collisionFilter
      = std::make_shared<collision::BodyNodeCollisionFilter>();
  world->getConstraintSolver()->addSkeleton(createFloor());
  auto box = createBox(
      "box", Eigen::Vector3d::Constant(0.2), Eigen::Vector3d(0.0, 0.0, 0.1));
  world->addSkeleton(box);
  EXPECT_TRUE(settle(*world, *box));
}
