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

#include "helpers/gtest_utils.hpp"

#include "dart/constraint/constraint_solver.hpp"
#include "dart/dynamics/box_shape.hpp"
#include "dart/dynamics/free_joint.hpp"
#include "dart/dynamics/inertia.hpp"
#include "dart/dynamics/skeleton.hpp"
#include "dart/dynamics/weld_joint.hpp"
#include "dart/simulation/world.hpp"

#include <gtest/gtest.h>

using namespace dart;
using namespace dart::dynamics;

namespace {

constexpr double kFloorHeight = 0.1;
constexpr double kFloorSize = 10.0;
constexpr double kBoxSize = 0.2;
constexpr double kPenetration = 0.01;
constexpr std::size_t kCorrectionSteps = 50;

SkeletonPtr createFloor()
{
  auto floor = Skeleton::create("floor");
  auto [joint, body] = floor->createJointAndBodyNodePair<WeldJoint>(nullptr);

  auto shape = std::make_shared<BoxShape>(
      Eigen::Vector3d(kFloorSize, kFloorSize, kFloorHeight));
  auto shapeNode = body->createShapeNodeWith<
      VisualAspect,
      CollisionAspect,
      DynamicsAspect>(shape);
  auto* dynamics = shapeNode->getDynamicsAspect();
  dynamics->setFrictionCoeff(0.0);
  dynamics->setRestitutionCoeff(0.0);

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation().z() = -kFloorHeight / 2.0;
  joint->setTransformFromParentBodyNode(tf);

  return floor;
}

SkeletonPtr createBox(double centerHeight)
{
  auto box = Skeleton::create("box");
  auto [joint, body] = box->createJointAndBodyNodePair<FreeJoint>(nullptr);

  auto shape = std::make_shared<BoxShape>(Eigen::Vector3d::Constant(kBoxSize));
  auto shapeNode = body->createShapeNodeWith<
      VisualAspect,
      CollisionAspect,
      DynamicsAspect>(shape);
  auto* dynamics = shapeNode->getDynamicsAspect();
  dynamics->setFrictionCoeff(0.0);
  dynamics->setRestitutionCoeff(0.0);

  const double mass = 1.0;
  Inertia inertia;
  inertia.setMass(mass);
  inertia.setMoment(shape->computeInertia(mass));
  body->setInertia(inertia);

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation().z() = centerHeight;
  FreeJoint::setTransformOf(joint, tf);

  return box;
}

} // namespace

TEST(Issue201, SplitImpulseKeepsRestingContactVelocityZero)
{
  auto world = simulation::World::create();
  world->setGravity(Eigen::Vector3d::Zero());
  world->setTimeStep(0.001);
  auto* solver = world->getConstraintSolver();
  ASSERT_NE(nullptr, solver);
  solver->setSplitImpulseEnabled(true);

  auto floor = createFloor();
  auto box = createBox(kBoxSize / 2.0 - kPenetration);
  box->setVelocities(Eigen::VectorXd::Zero(box->getNumDofs()));

  world->addSkeleton(floor);
  world->addSkeleton(box);

  const auto* body = box->getRootBodyNode();
  ASSERT_NE(body, nullptr);
  const double initialHeight = body->getTransform().translation().z();

  for (std::size_t i = 0; i < kCorrectionSteps; ++i) {
    world->step();
  }

  EXPECT_NEAR(body->getLinearVelocity().z(), 0.0, 1e-6);
  EXPECT_GT(body->getTransform().translation().z(), initialHeight + 1e-6);
}
