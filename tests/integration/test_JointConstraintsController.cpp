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

#include "demo_models/JointConstraintsController.hpp"

#include <dart/simulation/World.hpp>

#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/RevoluteJoint.hpp>

#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>

namespace {

// Independent unit-inertia hinges give a scalar SPD oracle for each DOF.
dart::dynamics::SkeletonPtr makeJointConstraintsHinges(int count = 28)
{
  auto skel = dart::dynamics::Skeleton::create("hinges");
  for (int i = 0; i < count; ++i) {
    auto pair
        = skel->createJointAndBodyNodePair<dart::dynamics::RevoluteJoint>();
    pair.first->setAxis(Eigen::Vector3d::UnitY());
    pair.second->setMomentOfInertia(1, 1, 1);
  }
  skel->setGravity(Eigen::Vector3d::Zero());
  return skel;
}

} // namespace

TEST(JointConstraintsController, OriginalGainsAndImplicitDamping)
{
  auto skel = makeJointConstraintsHinges();
  dart_demos::JointConstraintsController controller(skel, 0.001);
  skel->setPositions(Eigen::VectorXd::Constant(28, 0.3));
  skel->setVelocities(Eigen::VectorXd::Constant(28, 0.2));
  for (const double timestep : {0.001, 0.002}) {
    controller.timestep = timestep;
    const auto torques = controller.computeTorques(skel);
    EXPECT_TRUE(torques.head(6).isZero(0));
    for (int i = 6; i < 28; ++i) {
      const double kp = i < 22 ? 200 : 20;
      const double kd = i < 22 ? 100 : 10;
      const double expected
          = (-kp * (0.3 + 0.2 * timestep) - kd * 0.2) / (1 + kd * timestep);
      EXPECT_NEAR(torques[i], expected, 1e-12);
    }
  }
  EXPECT_TRUE(skel->getForces().isZero(0));
}

TEST(JointConstraintsController, AnkleStrategyRemembersPreviousOffset)
{
  auto skel = makeJointConstraintsHinges();
  skel->getBodyNode(0)->setName("h_heel_left");
  dart_demos::JointConstraintsController controller(skel, 0.001);
  const auto first = controller.computeTorques(skel);
  const auto second = controller.computeTorques(skel);
  for (int i = 0; i < 28; ++i) {
    if (i == 17 || i == 19) {
      EXPECT_NEAR(first[i], 6, 1e-12);
      EXPECT_NEAR(second[i], 1, 1e-12);
    } else if (i == 25 || i == 26) {
      EXPECT_NEAR(first[i], 5.5, 1e-12);
      EXPECT_NEAR(second[i], 0.5, 1e-12);
    } else {
      EXPECT_DOUBLE_EQ(first[i], 0);
      EXPECT_DOUBLE_EQ(second[i], 0);
    }
  }
}

TEST(JointConstraintsController, DeterministicWorldSteps)
{
  Eigen::VectorXd positions[2];
  for (int run = 0; run < 2; ++run) {
    auto skel = makeJointConstraintsHinges();
    auto world = dart::simulation::World::create();
    world->setGravity(Eigen::Vector3d::Zero());
    world->addSkeleton(skel);
    dart_demos::JointConstraintsController controller(
        skel, world->getTimeStep());
    skel->setPositions(Eigen::VectorXd::Constant(28, 0.3));
    for (int step = 0; step < 8; ++step) {
      skel->setForces(controller.computeTorques(skel));
      world->step();
    }
    positions[run] = skel->getPositions();
    EXPECT_TRUE(positions[run].allFinite());
    EXPECT_LT(positions[run][6], 0.3);
  }
  EXPECT_TRUE(positions[0].isApprox(positions[1], 0));
}

TEST(JointConstraintsController, RejectsInvalidInputsAndGuardsSmallLayouts)
{
  auto skel = makeJointConstraintsHinges(5);
  dart_demos::JointConstraintsController controller(skel, 0.001);
  EXPECT_TRUE(controller.computeTorques(skel).isZero(0));
  EXPECT_THROW(
      dart_demos::JointConstraintsController(nullptr, 0.001),
      std::invalid_argument);
  EXPECT_THROW(
      dart_demos::JointConstraintsController(
          makeJointConstraintsHinges(0), 0.001),
      std::invalid_argument);
  for (double dt :
       {0.0,
        -0.001,
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::quiet_NaN()}) {
    EXPECT_THROW(
        dart_demos::JointConstraintsController(skel, dt),
        std::invalid_argument);
    controller.timestep = dt;
    EXPECT_THROW(controller.computeTorques(skel), std::invalid_argument);
  }
  controller.timestep = 0.001;
  EXPECT_THROW(controller.computeTorques(nullptr), std::invalid_argument);
  EXPECT_THROW(
      controller.computeTorques(makeJointConstraintsHinges(6)),
      std::invalid_argument);
}
