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

#include <dart/dynamics/ball_joint.hpp>
#include <dart/dynamics/joint.hpp>
#include <dart/dynamics/skeleton.hpp>

#include <gtest/gtest.h>

#include <vector>

using namespace dart::dynamics;

//==============================================================================
// Per-DoF actuator overrides are stored as a sorted, compact vector of
// (index, type) pairs in JointProperties rather than a std::map, so the
// Properties struct stays trivially copyable across translation-unit and
// shared-library boundaries (regression for the heap-corruption seen when
// SDF-loaded worlds create and destroy joint properties).
TEST(Joints, PerDofActuatorOverridesStayOrderedAndCompact)
{
  auto skel = Skeleton::create("per_dof_actuator_order");
  auto [joint, body] = skel->createJointAndBodyNodePair<BallJoint>();
  (void)body;

  joint->setActuatorType(Joint::SERVO);
  joint->setActuatorType(2, Joint::MIMIC);
  joint->setActuatorType(1, Joint::MIMIC);

  const std::vector<Joint::ActuatorType> expectedWithOverrides{
      Joint::SERVO, Joint::MIMIC, Joint::MIMIC};
  EXPECT_EQ(joint->getActuatorTypes(), expectedWithOverrides);
  EXPECT_EQ(joint->getActuatorType(0), Joint::SERVO);
  EXPECT_EQ(joint->getActuatorType(1), Joint::MIMIC);
  EXPECT_EQ(joint->getActuatorType(2), Joint::MIMIC);

  // Overrides are kept sorted by DoF index regardless of insertion order.
  const auto& properties = joint->getJointProperties();
  ASSERT_EQ(properties.mActuatorTypes.size(), 2u);
  EXPECT_EQ(properties.mActuatorTypes[0].first, 1u);
  EXPECT_EQ(properties.mActuatorTypes[0].second, Joint::MIMIC);
  EXPECT_EQ(properties.mActuatorTypes[1].first, 2u);
  EXPECT_EQ(properties.mActuatorTypes[1].second, Joint::MIMIC);

  // Re-setting an existing override does not duplicate it.
  joint->setActuatorType(2, Joint::MIMIC);
  EXPECT_EQ(joint->getJointProperties().mActuatorTypes.size(), 2u);

  // Restoring a DoF to the joint-wide default erases its override and keeps
  // the remainder compact and sorted.
  joint->setActuatorType(1, Joint::SERVO);
  const std::vector<Joint::ActuatorType> expectedAfterErase{
      Joint::SERVO, Joint::SERVO, Joint::MIMIC};
  EXPECT_EQ(joint->getActuatorTypes(), expectedAfterErase);
  ASSERT_EQ(joint->getJointProperties().mActuatorTypes.size(), 1u);
  EXPECT_EQ(joint->getJointProperties().mActuatorTypes[0].first, 2u);

  joint->setActuatorType(2, Joint::SERVO);
  EXPECT_TRUE(joint->getJointProperties().mActuatorTypes.empty());
}

//==============================================================================
// Bulk actuator types round-trip through JointProperties, including copying a
// Properties struct between joints via setProperties() (the cross-boundary
// path that previously corrupted the heap with a std::map member).
TEST(Joints, BulkActuatorTypesRoundTripThroughJointProperties)
{
  auto skel = Skeleton::create("bulk_actuator_types");
  auto [joint, body] = skel->createJointAndBodyNodePair<BallJoint>();
  (void)body;

  const std::vector<Joint::ActuatorType> actuatorTypes{
      Joint::SERVO, Joint::MIMIC, Joint::SERVO};
  joint->setActuatorTypes(actuatorTypes);

  EXPECT_EQ(joint->getActuatorType(), Joint::SERVO);
  EXPECT_EQ(joint->getActuatorTypes(), actuatorTypes);
  ASSERT_EQ(joint->getJointProperties().mActuatorTypes.size(), 1u);
  EXPECT_EQ(joint->getJointProperties().mActuatorTypes[0].first, 1u);
  EXPECT_EQ(joint->getJointProperties().mActuatorTypes[0].second, Joint::MIMIC);

  auto [copiedJoint, copiedBody]
      = skel->createJointAndBodyNodePair<BallJoint>();
  (void)copiedBody;
  Joint* copiedBase = copiedJoint;
  copiedBase->setProperties(joint->getJointProperties());
  EXPECT_EQ(copiedJoint->getActuatorTypes(), actuatorTypes);

  copiedJoint->setActuatorTypes(actuatorTypes);
  EXPECT_EQ(copiedJoint->getActuatorTypes(), actuatorTypes);

  copiedJoint->setActuatorTypes(
      std::vector<Joint::ActuatorType>{
          Joint::VELOCITY, Joint::MIMIC, Joint::VELOCITY});
  EXPECT_EQ(copiedJoint->getActuatorTypes(), actuatorTypes);

  copiedJoint->setActuatorTypes(
      std::vector<Joint::ActuatorType>{
          Joint::SERVO, Joint::PASSIVE, Joint::SERVO});
  EXPECT_EQ(copiedJoint->getActuatorTypes(), actuatorTypes);
}
