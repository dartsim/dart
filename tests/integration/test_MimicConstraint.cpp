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

#include "dart/dynamics/Joint.hpp"
#include "dart/dynamics/MimicDofProperties.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/simulation/World.hpp"
#include "dart/utils/sdf/SdfParser.hpp"

#include <gtest/gtest.h>

using dart::dynamics::Joint;
using dart::dynamics::SkeletonPtr;
using dart::simulation::WorldPtr;

//==============================================================================
// The SDF parser should import a joint's <mimic> metadata and wire the follower
// DoF to the referenced joint through the existing per-DoF mimic runtime.
TEST(MimicConstraint, ParsesMimicMetadataFromSdf)
{
  const std::string worldUri = "dart://sample/sdf/test/mimic_joint_test.sdf";

  WorldPtr world = dart::utils::SdfParser::readWorld(worldUri);
  ASSERT_TRUE(world != nullptr);

  const SkeletonPtr mimicModel = world->getSkeleton("mimic_model");
  ASSERT_TRUE(mimicModel != nullptr);

  auto* referenceJoint = mimicModel->getJoint("reference_joint");
  auto* followerJoint = mimicModel->getJoint("follower_joint");
  ASSERT_NE(nullptr, referenceJoint);
  ASSERT_NE(nullptr, followerJoint);
  ASSERT_EQ(1u, followerJoint->getNumDofs());

  // The reference joint carries no <mimic>, so it stays a regular force joint.
  EXPECT_NE(referenceJoint->getActuatorType(), Joint::MIMIC);
  EXPECT_EQ(nullptr, referenceJoint->getMimicJoint(0));

  // The follower joint was switched to the MIMIC actuator by the parser.
  EXPECT_EQ(followerJoint->getActuatorType(), Joint::MIMIC);

  const auto props = followerJoint->getMimicDofProperties();
  ASSERT_FALSE(props.empty());
  EXPECT_EQ(props[0].mReferenceJoint, referenceJoint);
  EXPECT_EQ(props[0].mReferenceDofIndex, 0u);
  EXPECT_DOUBLE_EQ(props[0].mMultiplier, 2.0);
  EXPECT_DOUBLE_EQ(props[0].mOffset, 0.25);

  // The convenience accessors agree with the parsed properties.
  EXPECT_EQ(followerJoint->getMimicJoint(0), referenceJoint);
  EXPECT_DOUBLE_EQ(followerJoint->getMimicMultiplier(0), 2.0);
  EXPECT_DOUBLE_EQ(followerJoint->getMimicOffset(0), 0.25);
}

//==============================================================================
// A model without any <mimic> element must parse exactly as before: the parser
// addition is purely additive and never alters non-mimic joints (gz-physics /
// gz-sim SDF loads stay unaffected).
TEST(MimicConstraint, ModelWithoutMimicIsUnchanged)
{
  const std::string worldUri = "dart://sample/sdf/test/mimic_joint_test.sdf";

  WorldPtr world = dart::utils::SdfParser::readWorld(worldUri);
  ASSERT_TRUE(world != nullptr);

  const SkeletonPtr plainModel = world->getSkeleton("plain_model");
  ASSERT_TRUE(plainModel != nullptr);

  auto* plainJoint = plainModel->getJoint("plain_joint");
  ASSERT_NE(nullptr, plainJoint);

  // No <mimic> means no MIMIC actuator and no reference joint wiring.
  EXPECT_NE(plainJoint->getActuatorType(), Joint::MIMIC);
  EXPECT_EQ(nullptr, plainJoint->getMimicJoint(0));
}

//==============================================================================
// Loading an SDF that contains no <mimic> at all must succeed and leave every
// joint as a non-mimic joint, matching pre-backport behavior byte-for-byte.
TEST(MimicConstraint, ExistingNonMimicSdfParsesUnchanged)
{
  const std::string worldUri
      = "dart://sample/sdf/test/issue1193_revolute_test.sdf";

  WorldPtr world = dart::utils::SdfParser::readWorld(worldUri);
  ASSERT_TRUE(world != nullptr);

  const SkeletonPtr skeleton = world->getSkeleton(0);
  ASSERT_TRUE(skeleton != nullptr);

  for (std::size_t i = 0; i < skeleton->getNumJoints(); ++i) {
    auto* joint = skeleton->getJoint(i);
    ASSERT_NE(nullptr, joint);
    EXPECT_NE(joint->getActuatorType(), Joint::MIMIC);
  }
}
