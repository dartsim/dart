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
