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

#include "demo_models/ContactInverseDynamicsModel.hpp"

#include <dart/utils/SkelParser.hpp>

#include <dart/common/Macros.hpp>

#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>

namespace {

using dart_demos::ContactInverseDynamicsModel;

//==============================================================================
dart::simulation::WorldPtr squatWorld()
{
  auto world
      = dart::utils::SkelParser::readWorld("dart://sample/skel/fullbody1.skel");
  if (!world)
    throw std::runtime_error("failed to load fullbody1.skel");
  Eigen::Isometry3d rotation = Eigen::Isometry3d::Identity();
  rotation.linear() << 1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0;
  for (std::size_t i = 0; i < world->getNumSkeletons(); ++i) {
    const auto skeleton = world->getSkeleton(i);
    for (std::size_t j = 0; j < skeleton->getNumJoints(); ++j) {
      auto* joint = skeleton->getJoint(j);
      if (!joint->getParentBodyNode())
        joint->setTransformFromParentBodyNode(
            rotation * joint->getTransformFromParentBodyNode());
    }
  }
  world->setGravity(rotation.linear() * world->getGravity());
  return world;
}

//==============================================================================
TEST(
    ContactInverseDynamicsModelTest,
    ReplaysDeterministicallyAndRetainsHistories)
{
  ContactInverseDynamicsModel first(squatWorld());
  ContactInverseDynamicsModel second(squatWorld());
  ASSERT_EQ(first.keyframes.size(), 60u);
  ASSERT_EQ(first.contactFrames.size(), 4u);
  ASSERT_EQ(first.world->getNumSimpleFrames(), 4u);
  EXPECT_EQ(first.biped->getNumBodyNodes(), 20u);
  EXPECT_EQ(first.biped->getNumDofs(), 37u);
  EXPECT_TRUE(
      first.biped->getPositions().isApprox(first.initialPositions, 0.0));
  ASSERT_EQ(first.torqueNames.size(), 3u);
  EXPECT_EQ(first.torqueNames[0], "j_thigh_left_z");
  EXPECT_EQ(first.torqueNames[1], "j_shin_left");
  EXPECT_EQ(first.torqueNames[2], "j_heel_left_1");

  for (int frame = 0; frame < 241; ++frame) {
    first.updateFrame();
    second.updateFrame();
    EXPECT_TRUE(first.biped->getPositions().allFinite());
    EXPECT_TRUE(first.biped->getVelocities().allFinite());
    EXPECT_TRUE(first.biped->getAccelerations().allFinite());
    EXPECT_TRUE(first.biped->getPositions().isApprox(
        second.biped->getPositions(), 0.0));
    EXPECT_TRUE(first.biped->getVelocities().isApprox(
        second.biped->getVelocities(), 0.0));
    EXPECT_TRUE(first.biped->getAccelerations().isApprox(
        second.biped->getAccelerations(), 0.0));
    EXPECT_EQ(first.feasible, second.feasible);
    EXPECT_EQ(first.residualNorm, second.residualNorm);
    EXPECT_EQ(first.totalVerticalForce, second.totalVerticalForce);
    ASSERT_EQ(first.contactForces.size(), 4u);
    for (std::size_t i = 0; i < 4; ++i) {
      EXPECT_TRUE(first.contactForces[i].allFinite());
      EXPECT_TRUE(
          first.contactForces[i].isApprox(second.contactForces[i], 0.0));
    }
  }
  EXPECT_EQ(first.sampleIndex, 1u);
  EXPECT_EQ(first.plotSamples.size(), 240u);
  EXPECT_EQ(first.plotSamples, second.plotSamples);
  EXPECT_EQ(first.torquePlots, second.torquePlots);
  EXPECT_FLOAT_EQ(
      first.plotSamples[0], static_cast<float>(first.totalVerticalForce));
}

//==============================================================================
TEST(ContactInverseDynamicsModelTest, PausesRegeneratesAndScalesArrows)
{
  ContactInverseDynamicsModel model(squatWorld());
  model.setPlaybackSpeed(2.0f);
  model.updateFrame();
  EXPECT_DOUBLE_EQ(model.time, 1.0 / 30.0);
  model.playing = false;
  model.time = 1.5;
  model.updateFrame();
  const Eigen::VectorXd squat = model.biped->getPositions();
  EXPECT_DOUBLE_EQ(model.time, 1.5);
  EXPECT_DOUBLE_EQ(model.biped->getVelocities().norm(), 0.0);
  EXPECT_DOUBLE_EQ(model.biped->getAccelerations().norm(), 0.0);

  model.setSquatDepth(0.0f);
  model.requestRegeneration();
  EXPECT_TRUE(model.regenerateRequested);
  model.updateFrame();
  EXPECT_FALSE(model.regenerateRequested);
  EXPECT_EQ(model.keyframes.size(), 60u);
  EXPECT_GT((model.biped->getPositions() - squat).norm(), 0.01);

  const auto contacts = model.solver->getContacts();
  ASSERT_EQ(contacts.size(), 4u);
  model.setFrictionCoeff(0.3f);
  model.setForceScale(0.004f);
  model.updateFrame();
  for (std::size_t i = 0; i < contacts.size(); ++i) {
    EXPECT_DOUBLE_EQ(
        model.solver->getContacts()[i].frictionCoeff,
        static_cast<double>(0.3f));
    if (model.contactForces[i].norm() < 1e-8) {
      EXPECT_TRUE(model.contactFrames[i]->getVisualAspect()->isHidden());
      continue;
    }
    EXPECT_FALSE(model.contactFrames[i]->getVisualAspect()->isHidden());
    const Eigen::Vector3d tail
        = contacts[i].bodyNode->getWorldTransform() * contacts[i].localOffset;
    const Eigen::Vector3d head
        = tail + model.forceScale * model.contactForces[i];
    EXPECT_TRUE(model.arrows[i]->getTail().isApprox(tail));
    EXPECT_TRUE(model.arrows[i]->getHead().isApprox(head));
    DART_SUPPRESS_DEPRECATED_BEGIN
    const auto* mesh = model.arrows[i]->getMesh();
    DART_SUPPRESS_DEPRECATED_END
    ASSERT_GT(mesh->mNumMeshes, 0u);
    ASSERT_GT(mesh->mMeshes[0]->mNumVertices, 0u);
    const auto color = mesh->mMeshes[0]->mColors[0][0];
    EXPECT_FLOAT_EQ(color.r, model.feasible ? 0.0f : 1.0f);
    EXPECT_FLOAT_EQ(color.g, model.feasible ? 1.0f : 0.0f);
    EXPECT_FLOAT_EQ(color.b, 0.0f);
    EXPECT_FLOAT_EQ(color.a, 1.0f);
  }
}

//==============================================================================
TEST(ContactInverseDynamicsModelTest, RejectsInvalidWorldAndNonfiniteControls)
{
  EXPECT_THROW(ContactInverseDynamicsModel{nullptr}, std::invalid_argument);
  EXPECT_THROW(
      ContactInverseDynamicsModel{dart::simulation::World::create()},
      std::runtime_error);
  auto zeroGravity = dart::simulation::World::create();
  zeroGravity->setGravity(Eigen::Vector3d::Zero());
  EXPECT_THROW(ContactInverseDynamicsModel{zeroGravity}, std::invalid_argument);

  ContactInverseDynamicsModel model(squatWorld());
  const auto nan = std::numeric_limits<float>::quiet_NaN();
  EXPECT_THROW(model.setPlaybackSpeed(nan), std::invalid_argument);
  EXPECT_THROW(model.setSquatDepth(nan), std::invalid_argument);
  EXPECT_THROW(model.setFrictionCoeff(nan), std::invalid_argument);
  EXPECT_THROW(model.setForceScale(nan), std::invalid_argument);
  model.setPlaybackSpeed(9.0f);
  model.setSquatDepth(-1.0f);
  model.setFrictionCoeff(9.0f);
  model.setForceScale(-1.0f);
  EXPECT_FLOAT_EQ(model.playbackSpeed, 2.0f);
  EXPECT_FLOAT_EQ(model.squatDepth, 0.0f);
  EXPECT_FLOAT_EQ(model.frictionCoeff, 1.5f);
  EXPECT_FLOAT_EQ(model.forceScale, 0.0001f);
}

} // namespace
