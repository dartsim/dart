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

#include <dart/all.hpp>

#include <gtest/gtest.h>

#include <limits>
#include <ranges>

using namespace dart;
using namespace dart::dynamics;

//==============================================================================
// Helper function to create a simple skeleton with one body
//==============================================================================
static SkeletonPtr createSimpleSkeleton()
{
  auto skel = Skeleton::create("test_skeleton");

  // Create a single body with a FreeJoint
  auto pair = skel->createJointAndBodyNodePair<FreeJoint>();
  auto* body = pair.second;
  body->setName("test_body");

  // Set some mass properties
  body->setMass(1.0);

  return skel;
}

//==============================================================================
// Test that NaN values in addExtForce are ignored (not applied)
// This prevents simulation crashes from bad input parameters
//==============================================================================
TEST(BodyNodeExternalForce, NaNForceIgnoredInAddExtForce)
{
  auto skel = createSimpleSkeleton();
  auto* body = skel->getBodyNode(0);

  const double nan = std::numeric_limits<double>::quiet_NaN();

  // Clear any existing forces
  body->clearExternalForces();
  Eigen::Vector6d initialForce = body->getExternalForceLocal();
  EXPECT_TRUE(initialForce.isZero());

  // Attempt to add a force with NaN - should be ignored
  body->addExtForce(Eigen::Vector3d(nan, 1.0, 2.0));
  EXPECT_TRUE(body->getExternalForceLocal().isZero())
      << "NaN force should be ignored";

  body->addExtForce(Eigen::Vector3d(1.0, nan, 2.0));
  EXPECT_TRUE(body->getExternalForceLocal().isZero())
      << "NaN force should be ignored";

  body->addExtForce(Eigen::Vector3d(1.0, 2.0, nan));
  EXPECT_TRUE(body->getExternalForceLocal().isZero())
      << "NaN force should be ignored";

  // Valid force should still work
  body->addExtForce(Eigen::Vector3d(1.0, 2.0, 3.0));
  EXPECT_FALSE(body->getExternalForceLocal().isZero())
      << "Valid force should be applied";
}

//==============================================================================
TEST(BodyNodeExternalForce, NaNOffsetIgnoredInAddExtForce)
{
  auto skel = createSimpleSkeleton();
  auto* body = skel->getBodyNode(0);

  const double nan = std::numeric_limits<double>::quiet_NaN();

  body->clearExternalForces();

  // Force with NaN offset - should be ignored
  body->addExtForce(
      Eigen::Vector3d(1.0, 2.0, 3.0), Eigen::Vector3d(nan, 0.0, 0.0));
  EXPECT_TRUE(body->getExternalForceLocal().isZero())
      << "Force with NaN offset should be ignored";

  // Valid force with valid offset should work
  body->addExtForce(
      Eigen::Vector3d(1.0, 2.0, 3.0), Eigen::Vector3d(0.0, 0.0, 0.0));
  EXPECT_FALSE(body->getExternalForceLocal().isZero())
      << "Valid force with valid offset should be applied";
}

//==============================================================================
TEST(BodyNodeExternalForce, NaNForceIgnoredInSetExtForce)
{
  auto skel = createSimpleSkeleton();
  auto* body = skel->getBodyNode(0);

  const double nan = std::numeric_limits<double>::quiet_NaN();

  // First set a valid force
  body->setExtForce(Eigen::Vector3d(1.0, 2.0, 3.0));
  Eigen::Vector6d forceAfterValidSet = body->getExternalForceLocal();
  EXPECT_FALSE(forceAfterValidSet.isZero());

  // Attempt to set NaN force - should be ignored, keeping previous value
  body->setExtForce(Eigen::Vector3d(nan, 1.0, 2.0));
  EXPECT_EQ(body->getExternalForceLocal(), forceAfterValidSet)
      << "NaN force in setExtForce should be ignored, keeping previous value";
}

//==============================================================================
TEST(BodyNodeExternalForce, NaNTorqueIgnoredInAddExtTorque)
{
  auto skel = createSimpleSkeleton();
  auto* body = skel->getBodyNode(0);

  const double nan = std::numeric_limits<double>::quiet_NaN();

  body->clearExternalForces();

  // Attempt to add a torque with NaN - should be ignored
  body->addExtTorque(Eigen::Vector3d(nan, 1.0, 2.0));
  EXPECT_TRUE(body->getExternalForceLocal().isZero())
      << "NaN torque should be ignored";

  body->addExtTorque(Eigen::Vector3d(1.0, nan, 2.0));
  EXPECT_TRUE(body->getExternalForceLocal().isZero())
      << "NaN torque should be ignored";

  // Valid torque should still work
  body->addExtTorque(Eigen::Vector3d(1.0, 2.0, 3.0));
  EXPECT_FALSE(body->getExternalForceLocal().isZero())
      << "Valid torque should be applied";
}

//==============================================================================
TEST(BodyNodeExternalForce, NaNTorqueIgnoredInSetExtTorque)
{
  auto skel = createSimpleSkeleton();
  auto* body = skel->getBodyNode(0);

  const double nan = std::numeric_limits<double>::quiet_NaN();

  // First set a valid torque
  body->setExtTorque(Eigen::Vector3d(1.0, 2.0, 3.0));
  Eigen::Vector6d torqueAfterValidSet = body->getExternalForceLocal();
  EXPECT_FALSE(torqueAfterValidSet.isZero());

  // Attempt to set NaN torque - should be ignored, keeping previous value
  body->setExtTorque(Eigen::Vector3d(1.0, nan, 2.0));
  EXPECT_EQ(body->getExternalForceLocal(), torqueAfterValidSet)
      << "NaN torque in setExtTorque should be ignored, keeping previous value";
}

//==============================================================================
// Test that infinity values are also handled appropriately
//==============================================================================
TEST(BodyNodeExternalForce, InfinityForceIgnored)
{
  auto skel = createSimpleSkeleton();
  auto* body = skel->getBodyNode(0);

  const double inf = std::numeric_limits<double>::infinity();

  body->clearExternalForces();

  // +Inf force should be ignored
  body->addExtForce(Eigen::Vector3d(inf, 1.0, 2.0));
  EXPECT_TRUE(body->getExternalForceLocal().isZero())
      << "+Inf force should be ignored";

  // -Inf force should be ignored
  body->addExtForce(Eigen::Vector3d(-inf, 1.0, 2.0));
  EXPECT_TRUE(body->getExternalForceLocal().isZero())
      << "-Inf force should be ignored";
}

//==============================================================================
// Integration test: Verify that simulation doesn't crash with NaN forces
// This is the core scenario from gz-physics#844
//==============================================================================
TEST(BodyNodeExternalForce, SimulationDoesNotCrashWithNaNForce)
{
  auto skel = createSimpleSkeleton();
  auto world = simulation::World::create();
  world->addSkeleton(skel);

  auto* body = skel->getBodyNode(0);
  const double nan = std::numeric_limits<double>::quiet_NaN();

  // Apply NaN force (simulating what gz-sim hydrodynamics plugin might do
  // with invalid parameters like <xDotU>NaN</xDotU>)
  body->addExtForce(Eigen::Vector3d(nan, 0.0, 0.0));

  // This should not crash - the NaN force should be ignored
  EXPECT_NO_THROW({
    for (int i = 0; i < 10; ++i) {
      world->step();
    }
  });

  // Positions should still be valid (no NaN propagation)
  Eigen::VectorXd positions = skel->getPositions();
  for (const auto i : std::views::iota(Eigen::Index{0}, positions.size())) {
    EXPECT_FALSE(std::isnan(positions[i]))
        << "Position " << i << " should not be NaN";
  }
}

//==============================================================================
TEST(BodyNodeExternalForce, ExternalSpringCanConnectSeparateSkeletons)
{
  auto skelA = createSimpleSkeleton();
  auto skelB = createSimpleSkeleton();
  skelA->setName("spring_body_a");
  skelB->setName("spring_body_b");

  auto* bodyA = skelA->getBodyNode(0);
  auto* bodyB = skelB->getBodyNode(0);
  auto* jointA = dynamic_cast<FreeJoint*>(bodyA->getParentJoint());
  auto* jointB = dynamic_cast<FreeJoint*>(bodyB->getParentJoint());
  ASSERT_NE(nullptr, jointA);
  ASSERT_NE(nullptr, jointB);
  bodyA->setMomentOfInertia(1.0, 1.0, 1.0);
  bodyB->setMomentOfInertia(1.0, 1.0, 1.0);

  Eigen::Isometry3d tfA = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tfB = Eigen::Isometry3d::Identity();
  tfB.linear()
      = Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  tfB.translation() = Eigen::Vector3d(2.0, 0.0, 0.0);
  jointA->setTransform(tfA);
  jointB->setTransform(tfB);

  auto world = simulation::World::create();
  world->setGravity(Eigen::Vector3d::Zero());
  world->setTimeStep(1e-3);
  world->addSkeleton(skelA);
  world->addSkeleton(skelB);

  const Eigen::Vector3d localA = Eigen::Vector3d::Zero();
  const Eigen::Vector3d localB = Eigen::Vector3d::Zero();
  const double restLength = 1.0;
  const double stiffness = 10.0;

  const Eigen::Vector3d pA = bodyA->getWorldTransform() * localA;
  const Eigen::Vector3d pB = bodyB->getWorldTransform() * localB;
  const Eigen::Vector3d displacement = pB - pA;
  const double length = displacement.norm();
  ASSERT_GT(length, 1e-12);

  const Eigen::Vector3d direction = displacement / length;
  const Eigen::Vector3d force = stiffness * (length - restLength) * direction;

  bodyA->addExtForce(force, localA, false, true);
  bodyB->addExtForce(-force, localB, false, true);

  const double rotationalStiffness = 2.0;
  const Eigen::Matrix3d restRelativeRotation = Eigen::Matrix3d::Identity();
  const Eigen::Matrix3d currentRelativeRotation
      = bodyA->getWorldTransform().linear().transpose()
        * bodyB->getWorldTransform().linear();
  const Eigen::Vector3d rotationErrorInA = math::logMap(
      restRelativeRotation.transpose() * currentRelativeRotation);
  const Eigen::Vector3d torqueOnA = bodyA->getWorldTransform().linear()
                                    * (rotationalStiffness * rotationErrorInA);
  bodyA->addExtTorque(torqueOnA, false);
  bodyB->addExtTorque(-torqueOnA, false);

  world->step();

  const Eigen::Vector3d velocityA = bodyA->getLinearVelocity(localA);
  const Eigen::Vector3d velocityB = bodyB->getLinearVelocity(localB);
  EXPECT_GT(velocityA.x(), 0.0);
  EXPECT_LT(velocityB.x(), 0.0);
  EXPECT_NEAR(velocityA.x(), -velocityB.x(), 1e-12);
  EXPECT_NEAR(velocityA.y(), 0.0, 1e-12);
  EXPECT_NEAR(velocityB.y(), 0.0, 1e-12);
  EXPECT_NEAR(velocityA.z(), 0.0, 1e-12);
  EXPECT_NEAR(velocityB.z(), 0.0, 1e-12);

  const Eigen::Vector3d angularVelocityA = bodyA->getAngularVelocity();
  const Eigen::Vector3d angularVelocityB = bodyB->getAngularVelocity();
  EXPECT_GT(angularVelocityA.z(), 0.0);
  EXPECT_LT(angularVelocityB.z(), 0.0);
  EXPECT_NEAR(angularVelocityA.z(), -angularVelocityB.z(), 1e-12);
  EXPECT_NEAR(angularVelocityA.x(), 0.0, 1e-12);
  EXPECT_NEAR(angularVelocityB.x(), 0.0, 1e-12);
  EXPECT_NEAR(angularVelocityA.y(), 0.0, 1e-12);
  EXPECT_NEAR(angularVelocityB.y(), 0.0, 1e-12);

  EXPECT_TRUE(bodyA->getExternalForceLocal().isZero(1e-12));
  EXPECT_TRUE(bodyB->getExternalForceLocal().isZero(1e-12));
}
