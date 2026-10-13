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
 *     copyright notice, this list of conditions and the following disclaimer
 *     in the documentation and/or other materials provided with the
 *     distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "demo_models/ssik_ik_gui/ArmSkeleton.hpp"

#include <gtest/gtest.h>

#include <set>

namespace {
namespace arm_model = dart_demos::ssik_ik_gui;

std::vector<arm_model::JointSpec> armTestChain()
{
  std::vector<arm_model::JointSpec> chain(2);
  chain[0].tLeft.translation() = Eigen::Vector3d(0.4, 0.0, 0.1);
  chain[0].tRight.translation() = Eigen::Vector3d(0.2, 0.0, 0.0);
  chain[0].axis = Eigen::Vector3d(0.0, 0.0, 3.0);
  chain[1].tLeft.translation() = Eigen::Vector3d(0.3, 0.1, 0.05);
  chain[1].tLeft.linear()
      = Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitX()).toRotationMatrix();
  chain[1].tRight.translation() = Eigen::Vector3d(0.1, 0.02, 0.04);
  chain[1].tRight.linear()
      = Eigen::AngleAxisd(-0.2, Eigen::Vector3d::UnitY()).toRotationMatrix();
  chain[1].axis = Eigen::Vector3d(0.0, 2.0, 0.0);
  chain[1].prismatic = true;
  return chain;
}

void expectArmColor(
    const dart::dynamics::SkeletonPtr& skeleton,
    const Eigen::Vector4d& expected)
{
  for (std::size_t b = 0; b < skeleton->getNumBodyNodes(); ++b) {
    const auto* body = skeleton->getBodyNode(b);
    for (std::size_t s = 0; s < body->getNumShapeNodes(); ++s) {
      const auto* shape = body->getShapeNode(s);
      ASSERT_NE(shape->getVisualAspect(), nullptr);
      EXPECT_TRUE(shape->getVisualAspect()->getRGBA().isApprox(expected));
      EXPECT_FALSE(shape->hasCollisionAspect());
    }
  }
}
} // namespace

//==============================================================================
TEST(ArmSkeletonTest, PreservesCatalogOrderAndNineteenDistinctArms)
{
  const auto specs = arm_model::getArmSpecs();
  ASSERT_EQ(specs.size(), 19);
  EXPECT_EQ(specs.front().label, "Universal Robots UR5");
  EXPECT_EQ(specs.front().module, "ur5_ik");
  EXPECT_EQ(specs.back().label, "OpenArm right");
  EXPECT_EQ(specs.back().module, "openarm_right_ik");
  std::set<std::string> labels;
  std::set<std::string> modules;
  for (const auto& spec : specs) {
    EXPECT_FALSE(spec.label.empty());
    EXPECT_TRUE(labels.insert(spec.label).second);
    EXPECT_TRUE(modules.insert(spec.module).second);
  }
}

//==============================================================================
TEST(ArmSkeletonTest, MatchesProductOfExponentialsForMixedJoints)
{
  const auto world = dart::simulation::World::create();
  arm_model::ArmSkeleton arm;
  arm.setWorld(world);
  const auto chain = armTestChain();
  arm.build(chain);
  arm_model::SsikSolution solution;
  solution.q = Eigen::Vector2d(0.7, 0.25);
  arm.showSolutions({solution}, 0);

  ASSERT_EQ(world->getNumSkeletons(), 1);
  const auto skeleton = world->getSkeleton(0);
  EXPECT_EQ(skeleton->getName(), "ssik_arm_0");
  ASSERT_EQ(skeleton->getNumBodyNodes(), 3);
  ASSERT_EQ(skeleton->getNumDofs(), 2);
  EXPECT_EQ(skeleton->getBodyNode(0)->getName(), "base");
  EXPECT_EQ(skeleton->getBodyNode(1)->getName(), "b0");
  EXPECT_EQ(skeleton->getBodyNode(2)->getName(), "b1");
  EXPECT_EQ(skeleton->getJoint(0)->getType(), "WeldJoint");
  EXPECT_EQ(skeleton->getJoint(1)->getType(), "RevoluteJoint");
  EXPECT_EQ(skeleton->getJoint(2)->getType(), "PrismaticJoint");
  EXPECT_EQ(skeleton->getPositions(), solution.q);
  Eigen::Isometry3d rotation = Eigen::Isometry3d::Identity();
  rotation.linear() = Eigen::AngleAxisd(solution.q[0], Eigen::Vector3d::UnitZ())
                          .toRotationMatrix();
  Eigen::Isometry3d translation = Eigen::Isometry3d::Identity();
  translation.translation() = solution.q[1] * Eigen::Vector3d::UnitY();
  const Eigen::Isometry3d expected = chain[0].tLeft * rotation * chain[0].tRight
                                     * chain[1].tLeft * translation
                                     * chain[1].tRight;
  EXPECT_TRUE(skeleton->getBodyNode(2)->getWorldTransform().matrix().isApprox(
      expected.matrix(), 1e-14));
  expectArmColor(skeleton, Eigen::Vector4d(0.90, 0.20, 0.18, 1.0));
}

//==============================================================================
TEST(ArmSkeletonTest, CapsGhostsResetsRestPoseAndClearsOnlyOwnedSkeletons)
{
  const auto world = dart::simulation::World::create();
  const auto unrelated = dart::dynamics::Skeleton::create("unrelated");
  world->addSkeleton(unrelated);
  arm_model::ArmSkeleton arm;
  arm.setWorld(world);
  arm.build(armTestChain());
  std::vector<arm_model::SsikSolution> solutions(20);
  for (std::size_t i = 0; i < solutions.size(); ++i)
    solutions[i].q = Eigen::Vector2d(i * 0.03, i * 0.02);
  arm.showSolutions(solutions, 3);
  ASSERT_EQ(world->getNumSkeletons(), 17);
  for (std::size_t i = 0; i < 16; ++i) {
    const auto skeleton = world->getSkeleton(i + 1);
    EXPECT_EQ(skeleton->getPositions(), solutions[i].q);
    expectArmColor(
        skeleton,
        i == 3 ? Eigen::Vector4d(0.90, 0.20, 0.18, 1.0)
               : Eigen::Vector4d(0.85, 0.32, 0.30, 0.22));
  }
  arm.showSolutions({}, 0);
  ASSERT_EQ(world->getNumSkeletons(), 2);
  EXPECT_EQ(world->getSkeleton(1)->getPositions(), Eigen::VectorXd::Zero(2));
  expectArmColor(world->getSkeleton(1), Eigen::Vector4d(0.55, 0.58, 0.62, 0.5));
  arm.build({});
  arm.showSolutions({}, 0);
  ASSERT_EQ(world->getNumSkeletons(), 1);
  EXPECT_EQ(world->getSkeleton(0), unrelated);
}

//==============================================================================
TEST(ArmSkeletonTest, UsesOriginalFloatControlsAndEigenEulerBranch)
{
  std::array<float, 3> xyz{{0.123456789f, -0.234567891f, 0.345678912f}};
  std::array<float, 3> rpy{{0.0f, 0.0f, -30.0f}};
  const auto transform = arm_model::makeTargetTransform(xyz, rpy);
  EXPECT_EQ(transform.translation(), Eigen::Vector3d(xyz[0], xyz[1], xyz[2]));
  std::array<float, 3> readXyz;
  std::array<float, 3> readRpy;
  arm_model::targetControlsFromTransform(transform, readXyz, readRpy);
  EXPECT_EQ(readXyz, xyz);
  EXPECT_GE(readRpy[2], 0.0f);
  EXPECT_LE(readRpy[2], 180.0f);
  EXPECT_TRUE(arm_model::makeTargetTransform(readXyz, readRpy)
                  .matrix()
                  .isApprox(transform.matrix(), 1e-7));
}
