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

#include "demo_models/BipedStandController.hpp"

#include <dart/utils/SkelParser.hpp>

#include <dart/dart.hpp>

#include <gtest/gtest.h>

#include <stdexcept>

namespace {

dart::simulation::WorldPtr bipedStandWorld()
{
  auto world
      = dart::utils::SkelParser::readWorld("dart://sample/skel/fullbody1.skel");
  if (!world)
    throw std::runtime_error("failed to load fullbody1.skel");
  world->setNumSimulationThreads(1);
  Eigen::Isometry3d rotation = Eigen::Isometry3d::Identity();
  rotation.linear() << 1, 0, 0, 0, 0, -1, 0, 1, 0;
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
  auto biped = world->getSkeleton("fullbody1");
  biped->getDof("j_pelvis_rot_y")->setPosition(-0.20);
  biped->getDof("j_thigh_left_z")->setPosition(0.15);
  biped->getDof("j_shin_left")->setPosition(-0.40);
  biped->getDof("j_heel_left_1")->setPosition(0.25);
  biped->getDof("j_thigh_right_z")->setPosition(0.15);
  biped->getDof("j_shin_right")->setPosition(-0.40);
  biped->getDof("j_heel_right_1")->setPosition(0.25);
  biped->getDof("j_abdomen_2")->setPosition(0.00);
  return world;
}

Eigen::VectorXd runBipedStand()
{
  auto world = bipedStandWorld();
  auto biped = world->getSkeleton("fullbody1");
  auto controller
      = dart_demos::makeBipedStandController(biped, world->getTimeStep());
  for (int i = 0; i < 200; ++i) {
    auto torques = dart_demos::computeTorques(biped, controller);
    EXPECT_TRUE(torques.allFinite());
    EXPECT_TRUE(torques.head(6).isZero(0));
    biped->setForces(torques);
    world->step();
  }
  Eigen::VectorXd state(biped->getNumDofs() * 2);
  state << biped->getPositions(), biped->getVelocities();
  return state;
}

} // namespace

TEST(BipedStandControllerTest, CapturesStandingPoseAndUnactuatedRootGains)
{
  auto world = bipedStandWorld();
  auto biped = world->getSkeleton("fullbody1");
  // Lean forward so this fixture exercises the active ankle range.
  biped->getDof("j_pelvis_rot_y")->setPosition(0.5);
  auto controller
      = dart_demos::makeBipedStandController(biped, world->getTimeStep());
  EXPECT_TRUE(controller.desiredDofs.isApprox(biped->getPositions(), 0));
  for (int i = 0; i < controller.desiredDofs.size(); ++i) {
    EXPECT_EQ(controller.kp(i, i), i < 6 ? 0 : 400);
    EXPECT_EQ(controller.kd(i, i), i < 6 ? 0 : 40);
  }
  auto first = dart_demos::computeTorques(biped, controller);
  const double offset = biped->getCOM()[0]
                        - (controller.leftHeel->getTransform()
                           * Eigen::Vector3d(0.05, 0, 0))[0];
  double ankleGain = 0;
  if (offset > 0 && offset < 0.1)
    ankleGain = 10;
  else if (offset > -0.2 && offset < -0.05)
    ankleGain = 100;
  ASSERT_GT(ankleGain, 0);
  EXPECT_EQ(controller.preOffset, offset);
  auto second = dart_demos::computeTorques(biped, controller);
  Eigen::VectorXd expected = Eigen::VectorXd::Zero(first.size());
  for (auto index :
       {controller.leftHeelDof,
        controller.leftToeDof,
        controller.rightHeelDof,
        controller.rightToeDof})
    expected[index] = ankleGain * offset;
  EXPECT_TRUE((second - first).isApprox(expected, 1e-12));
  EXPECT_TRUE(controller.desiredDofs.isApprox(biped->getPositions(), 0));
}

TEST(BipedStandControllerTest, TimestepChangesImplicitDamping)
{
  auto world = bipedStandWorld();
  auto biped = world->getSkeleton("fullbody1");
  auto initial = dart_demos::makeBipedStandController(biped, 0.001);
  auto changed = dart_demos::makeBipedStandController(biped, 0.01);
  auto first = dart_demos::computeTorques(biped, initial);
  auto second = dart_demos::computeTorques(biped, changed);
  EXPECT_GT((first - second).norm(), 0);
  EXPECT_TRUE(first.head(6).isZero(0));
  EXPECT_TRUE(second.head(6).isZero(0));
}

TEST(BipedStandControllerTest, RepeatsFiniteStandingDeterministically)
{
  auto first = runBipedStand();
  auto second = runBipedStand();
  EXPECT_TRUE(first.allFinite());
  EXPECT_TRUE(second.isApprox(first, 0));
}

TEST(BipedStandControllerTest, MissingFootDofsReportTheModelError)
{
  auto biped = dart::dynamics::Skeleton::create();
  EXPECT_THROW(
      dart_demos::makeBipedStandController(biped, 0.001), std::runtime_error);
}
