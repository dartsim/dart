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

#include "helpers/dynamics_helpers.hpp"

#include <dart/simulation/world.hpp>

#include <dart/constraint/constraint_solver.hpp>

#include <dart/collision/bullet/bullet_collision_detector.hpp>
#include <dart/collision/ode/ode_collision_detector.hpp>

#include <dart/dynamics/capsule_shape.hpp>
#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/inertia.hpp>
#include <dart/dynamics/skeleton.hpp>

#include <dart/math/constants.hpp>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include <functional>
#include <tuple>

using namespace dart::dynamics;
using namespace dart::simulation;

namespace {

constexpr double kCapsuleRadius = 0.2;
constexpr double kCapsuleHeight = 0.6;
constexpr double kTolerance = 0.05;
constexpr int kNumSteps = 50000;

using DetectorFactory = std::function<dart::collision::CollisionDetectorPtr()>;

//==============================================================================
void runCapsuleGroundContactTest(const DetectorFactory& factory)
{
  auto world = World::create();
  world->getConstraintSolver()->setCollisionDetector(factory());

  auto ground = createBox({10000, 1000, 0.1}, {0, 0, -0.05});
  ground->setMobile(false);
  world->addSkeleton(ground);

  auto skeleton = Skeleton::create("capsule");
  FreeJoint* joint;
  BodyNode* bodyNode;
  std::tie(joint, bodyNode) = skeleton->createJointAndBodyNodePair<FreeJoint>();

  Eigen::Isometry3d placement
      = Eigen::Translation3d(0, 0, 0.5)
        * Eigen::AngleAxisd(dart::math::half_pi, Eigen::Vector3d::UnitY());
  joint->setRelativeTransform(placement);

  auto capsuleShape
      = std::make_shared<CapsuleShape>(kCapsuleRadius, kCapsuleHeight);
  auto* capsuleNode
      = bodyNode->createShapeNodeWith<CollisionAspect, DynamicsAspect>(
          capsuleShape);

  dart::dynamics::Inertia inertia;
  inertia.setMass(1.0);
  inertia.setMoment(capsuleNode->getShape()->computeInertia(inertia.getMass()));
  bodyNode->setInertia(inertia);

  world->addSkeleton(skeleton);

  for (int i = 0; i < kNumSteps; ++i) {
    world->step();
    const auto position = bodyNode->getWorldTransform().translation();
    ASSERT_GT(position.z(), kCapsuleRadius - kTolerance);
  }
}

} // namespace

//==============================================================================
TEST(Issue1654, BulletCapsuleGroundContact)
{
  runCapsuleGroundContactTest(
      [] { return dart::collision::BulletCollisionDetector::create(); });
}

//==============================================================================
TEST(Issue1654, ODECapsuleGroundContact)
{
  runCapsuleGroundContactTest(
      [] { return dart::collision::OdeCollisionDetector::create(); });
}
