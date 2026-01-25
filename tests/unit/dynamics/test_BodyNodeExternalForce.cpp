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

#include <dart/dart.hpp>

#include <gtest/gtest.h>

#include <limits>

#include <cmath>

using namespace dart;
using namespace dart::dynamics;

//==============================================================================
// Helper function to create a simple skeleton for testing
SkeletonPtr createTestSkeleton()
{
  SkeletonPtr skel = Skeleton::create("TestSkeleton");

  // Create a free joint with a body
  FreeJoint::Properties jointProps;
  jointProps.mName = "FreeJoint";

  BodyNode::Properties bodyProps;
  bodyProps.mName = "TestBody";
  bodyProps.mInertia.setMass(1.0);

  skel->createJointAndBodyNodePair<FreeJoint>(nullptr, jointProps, bodyProps);

  return skel;
}

//==============================================================================
// Tests for addExtForce with NaN values
TEST(BodyNodeExternalForce, AddExtForceWithNaN)
{
  auto skel = createTestSkeleton();
  auto body = skel->getBodyNode(0);

  // Store original external force
  Eigen::Vector6d originalFext = body->getExternalForceLocal();

  // Create a force vector with NaN
  Eigen::Vector3d nanForce(1.0, std::numeric_limits<double>::quiet_NaN(), 3.0);
  Eigen::Vector3d offset = Eigen::Vector3d::Zero();

  // Apply NaN force - should be ignored
  body->addExtForce(nanForce, offset, true, true);

  // Verify the external force did not change
  EXPECT_TRUE(body->getExternalForceLocal().isApprox(originalFext));
}

//==============================================================================
// Tests for addExtForce with Inf values
TEST(BodyNodeExternalForce, AddExtForceWithInf)
{
  auto skel = createTestSkeleton();
  auto body = skel->getBodyNode(0);

  // Store original external force
  Eigen::Vector6d originalFext = body->getExternalForceLocal();

  // Create a force vector with Inf
  Eigen::Vector3d infForce(std::numeric_limits<double>::infinity(), 2.0, 3.0);
  Eigen::Vector3d offset = Eigen::Vector3d::Zero();

  // Apply Inf force - should be ignored
  body->addExtForce(infForce, offset, true, true);

  // Verify the external force did not change
  EXPECT_TRUE(body->getExternalForceLocal().isApprox(originalFext));
}

//==============================================================================
// Tests for addExtForce with NaN offset
TEST(BodyNodeExternalForce, AddExtForceWithNaNOffset)
{
  auto skel = createTestSkeleton();
  auto body = skel->getBodyNode(0);

  // Store original external force
  Eigen::Vector6d originalFext = body->getExternalForceLocal();

  // Create valid force but NaN offset
  Eigen::Vector3d force(1.0, 2.0, 3.0);
  Eigen::Vector3d nanOffset(0.0, std::numeric_limits<double>::quiet_NaN(), 0.0);

  // Apply force with NaN offset - should be ignored
  body->addExtForce(force, nanOffset, true, true);

  // Verify the external force did not change
  EXPECT_TRUE(body->getExternalForceLocal().isApprox(originalFext));
}

//==============================================================================
// Tests for setExtForce with NaN values
TEST(BodyNodeExternalForce, SetExtForceWithNaN)
{
  auto skel = createTestSkeleton();
  auto body = skel->getBodyNode(0);

  // Apply a valid force first
  Eigen::Vector3d validForce(1.0, 2.0, 3.0);
  Eigen::Vector3d offset = Eigen::Vector3d::Zero();
  body->setExtForce(validForce, offset, true, true);

  // Store current external force
  Eigen::Vector6d currentFext = body->getExternalForceLocal();

  // Try to set NaN force - should be ignored
  Eigen::Vector3d nanForce(std::numeric_limits<double>::quiet_NaN(), 2.0, 3.0);
  body->setExtForce(nanForce, offset, true, true);

  // Verify the external force did not change
  EXPECT_TRUE(body->getExternalForceLocal().isApprox(currentFext));
}

//==============================================================================
// Tests for addExtTorque with NaN values
TEST(BodyNodeExternalForce, AddExtTorqueWithNaN)
{
  auto skel = createTestSkeleton();
  auto body = skel->getBodyNode(0);

  // Store original external force (torque is part of Fext)
  Eigen::Vector6d originalFext = body->getExternalForceLocal();

  // Create a torque vector with NaN
  Eigen::Vector3d nanTorque(1.0, 2.0, std::numeric_limits<double>::quiet_NaN());

  // Apply NaN torque - should be ignored
  body->addExtTorque(nanTorque, true);

  // Verify the external force did not change
  EXPECT_TRUE(body->getExternalForceLocal().isApprox(originalFext));
}

//==============================================================================
// Tests for addExtTorque with Inf values
TEST(BodyNodeExternalForce, AddExtTorqueWithInf)
{
  auto skel = createTestSkeleton();
  auto body = skel->getBodyNode(0);

  // Store original external force
  Eigen::Vector6d originalFext = body->getExternalForceLocal();

  // Create a torque vector with negative infinity
  Eigen::Vector3d infTorque(-std::numeric_limits<double>::infinity(), 2.0, 3.0);

  // Apply Inf torque - should be ignored
  body->addExtTorque(infTorque, true);

  // Verify the external force did not change
  EXPECT_TRUE(body->getExternalForceLocal().isApprox(originalFext));
}

//==============================================================================
// Tests for setExtTorque with NaN values
TEST(BodyNodeExternalForce, SetExtTorqueWithNaN)
{
  auto skel = createTestSkeleton();
  auto body = skel->getBodyNode(0);

  // Apply a valid torque first
  Eigen::Vector3d validTorque(1.0, 2.0, 3.0);
  body->setExtTorque(validTorque, true);

  // Store current external force
  Eigen::Vector6d currentFext = body->getExternalForceLocal();

  // Try to set NaN torque - should be ignored
  Eigen::Vector3d nanTorque(1.0, std::numeric_limits<double>::quiet_NaN(), 3.0);
  body->setExtTorque(nanTorque, true);

  // Verify the external force did not change
  EXPECT_TRUE(body->getExternalForceLocal().isApprox(currentFext));
}

//==============================================================================
// Tests for valid force application (sanity check)
TEST(BodyNodeExternalForce, ValidForceApplication)
{
  auto skel = createTestSkeleton();
  auto body = skel->getBodyNode(0);

  // Clear any existing external forces
  body->clearExternalForces();

  // Apply a valid force
  Eigen::Vector3d force(10.0, 20.0, 30.0);
  Eigen::Vector3d offset = Eigen::Vector3d::Zero();

  body->addExtForce(force, offset, true, true);

  // Verify the force was applied (linear part should be non-zero)
  Eigen::Vector6d fext = body->getExternalForceLocal();
  EXPECT_FALSE(fext.tail<3>().isZero());
}

//==============================================================================
// Integration test: Simulate scenario from gz-physics#844
// NaN parameters from hydrodynamics plugin should not crash simulation
TEST(BodyNodeExternalForce, GzPhysics844Scenario)
{
  auto skel = createTestSkeleton();
  auto body = skel->getBodyNode(0);

  // Simulate the scenario where hydrodynamics plugin provides NaN parameters
  // like <xDotU>NaN</xDotU>

  // Clear forces
  body->clearExternalForces();

  // Try to apply NaN forces multiple times (simulating continuous plugin
  // updates)
  for (int i = 0; i < 10; ++i) {
    Eigen::Vector3d nanForce(
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN());

    // This should not crash or corrupt state
    body->addExtForce(nanForce);
    body->addExtTorque(nanForce, true);
  }

  // External forces should remain zero (all NaN forces were rejected)
  Eigen::Vector6d fext = body->getExternalForceLocal();
  EXPECT_TRUE(fext.isZero());

  // Skeleton should still be valid
  EXPECT_TRUE(skel != nullptr);
  EXPECT_EQ(skel->getNumBodyNodes(), 1u);
}
