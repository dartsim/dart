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

#include "TestHelpers.hpp"
#include "dart/collision/native/NativeCollisionDetector.hpp"
#include "dart/dart.hpp"
#include "dart/utils/mjcf/detail/MujocoModel.hpp"
#include "dart/utils/mjcf/detail/Types.hpp"
#include "dart/utils/mjcf/detail/Utils.hpp"
#include "dart/utils/utils.hpp"

#include <gtest/gtest.h>

#include <iostream>
#include <string>
#include <vector>

using namespace dart;
using namespace utils::MjcfParser::detail;

//==============================================================================
common::ResourceRetrieverPtr createRetriever()
{
  auto newRetriever = std::make_shared<utils::CompositeResourceRetriever>();
  newRetriever->addSchemaRetriever(
      "file", std::make_shared<common::LocalResourceRetriever>());
  newRetriever->addSchemaRetriever(
      "dart", utils::DartResourceRetriever::create());
  return newRetriever;
}

//==============================================================================
TEST(MjcfParserTest, ParseDetailMujocoAnt)
{
  const auto uri = "dart://sample/mjcf/openai/ant.xml";

  auto mujoco = utils::MjcfParser::detail::MujocoModel();
  auto errors = mujoco.read(uri, createRetriever());
  ASSERT_TRUE(errors.empty());

  EXPECT_EQ(mujoco.getModel(), "ant");

  const auto& compiler = mujoco.getCompiler();
  EXPECT_EQ(compiler.getAngle(), Angle::DEGREE);
  EXPECT_EQ(compiler.getCoordinate(), Coordinate::LOCAL);

  const auto& option = mujoco.getOption();
  EXPECT_EQ(option.getIntegrator(), Integrator::RK4);
  EXPECT_DOUBLE_EQ(option.getTimestep(), 0.01);

  const auto& worldbody = mujoco.getWorldbody();

  ASSERT_EQ(worldbody.getNumGeoms(), 1);

  ASSERT_EQ(worldbody.getNumRootBodies(), 1);
  const auto& rootBody0 = worldbody.getRootBody(0);
  EXPECT_EQ(rootBody0.getName(), "torso");
  EXPECT_TRUE(equals(
      rootBody0.getRelativeTransform().translation().eval(),
      Eigen::Vector3d(0, 0, 0.75)));

  ASSERT_EQ(rootBody0.getNumJoints(), 1);
  const auto& rootJoint0 = rootBody0.getJoint(0);
  EXPECT_EQ(rootJoint0.getType(), JointType::FREE);
  EXPECT_EQ(rootJoint0.getName(), "root");
}

//==============================================================================
TEST(MjcfParserTest, DefaultSettings)
{
  // For the details, see http://mujoco.org/book/modeling.html#CDefault

  const auto uri = "dart://sample/mjcf/test/default.xml";
  const auto options = utils::MjcfParser::Options();
  auto world = utils::MjcfParser::readWorld(uri);
  ASSERT_NE(world, nullptr);

  ASSERT_EQ(world->getNumSkeletons(), 2);

  auto boxSkel = world->getSkeleton(options.mGeomSkeletonNamePrefix);
  ASSERT_NE(boxSkel, nullptr);
  auto boxShapeNode = boxSkel->getShapeNode(0);
  ASSERT_NE(boxShapeNode, nullptr);
  auto boxVisualAspect = boxShapeNode->getVisualAspect();
  ASSERT_NE(boxVisualAspect, nullptr);
  EXPECT_TRUE(boxVisualAspect->getRGBA().isApprox(Eigen::Vector4d(1, 0, 0, 1)));

  auto bodySkel = world->getSkeleton("body0");
  ASSERT_NE(bodySkel, nullptr);

  auto ellipsoidShapeNode = bodySkel->getShapeNode(0);
  ASSERT_NE(ellipsoidShapeNode, nullptr);
  auto ellipsoidVisualAspect = ellipsoidShapeNode->getVisualAspect();
  ASSERT_NE(ellipsoidVisualAspect, nullptr);
  ellipsoidVisualAspect->getRGBA().isApprox(Eigen::Vector4d(0, 1, 0, 1));

  auto sphereShapeNode = bodySkel->getShapeNode(1);
  ASSERT_NE(sphereShapeNode, nullptr);
  auto sphereVisualAspect = sphereShapeNode->getVisualAspect();
  ASSERT_NE(sphereVisualAspect, nullptr);
  sphereVisualAspect->getRGBA().isApprox(Eigen::Vector4d(0, 0, 1, 1));

  auto cylinderShapeNode = bodySkel->getShapeNode(2);
  ASSERT_NE(cylinderShapeNode, nullptr);
  auto cylinderVisualAspect = cylinderShapeNode->getVisualAspect();
  ASSERT_NE(cylinderVisualAspect, nullptr);
  cylinderVisualAspect->getRGBA().isApprox(Eigen::Vector4d(1, 0, 0, 1));
}

//==============================================================================
TEST(MjcfParserTest, IncludeDefaultSettings)
{
  // For the details, see http://mujoco.org/book/modeling.html#CDefault

  const auto uri = "dart://sample/mjcf/test/include_main.xml";
  const auto options = utils::MjcfParser::Options();
  auto world = utils::MjcfParser::readWorld(uri);
  ASSERT_NE(world, nullptr);

  ASSERT_EQ(world->getNumSkeletons(), 2);

  auto boxSkel = world->getSkeleton(options.mGeomSkeletonNamePrefix);
  ASSERT_NE(boxSkel, nullptr);
  auto boxShapeNode = boxSkel->getShapeNode(0);
  ASSERT_NE(boxShapeNode, nullptr);
  auto boxVisualAspect = boxShapeNode->getVisualAspect();
  ASSERT_NE(boxVisualAspect, nullptr);
  EXPECT_TRUE(boxVisualAspect->getRGBA().isApprox(Eigen::Vector4d(1, 0, 0, 1)));

  auto bodySkel = world->getSkeleton("body0");
  ASSERT_NE(bodySkel, nullptr);

  auto ellipsoidShapeNode = bodySkel->getShapeNode(0);
  ASSERT_NE(ellipsoidShapeNode, nullptr);
  auto ellipsoidVisualAspect = ellipsoidShapeNode->getVisualAspect();
  ASSERT_NE(ellipsoidVisualAspect, nullptr);
  ellipsoidVisualAspect->getRGBA().isApprox(Eigen::Vector4d(0, 1, 0, 1));

  auto sphereShapeNode = bodySkel->getShapeNode(1);
  ASSERT_NE(sphereShapeNode, nullptr);
  auto sphereVisualAspect = sphereShapeNode->getVisualAspect();
  ASSERT_NE(sphereVisualAspect, nullptr);
  sphereVisualAspect->getRGBA().isApprox(Eigen::Vector4d(0, 0, 1, 1));

  auto cylinderShapeNode = bodySkel->getShapeNode(2);
  ASSERT_NE(cylinderShapeNode, nullptr);
  auto cylinderVisualAspect = cylinderShapeNode->getVisualAspect();
  ASSERT_NE(cylinderVisualAspect, nullptr);
  cylinderVisualAspect->getRGBA().isApprox(Eigen::Vector4d(1, 0, 0, 1));
}

//==============================================================================
TEST(MjcfParserTest, Ant)
{
  const auto uri = "dart://sample/mjcf/openai/ant.xml";
  const auto options = utils::MjcfParser::Options();
  auto world = utils::MjcfParser::readWorld(uri);
  ASSERT_NE(world, nullptr);

  // The number of skeletons are the sum of the number of root <geom>s and root
  // <body>s in <worldbody>. In this case, there are one root <geom> and one
  // root <body> so it's 2.
  ASSERT_EQ(world->getNumSkeletons(), 2);

  auto antSkel = world->getSkeleton("torso");
  ASSERT_NE(antSkel, nullptr);
  // The number of bodies are 13: one for the torso and three bodies in each
  // four legs.
  ASSERT_EQ(antSkel->getNumBodyNodes(), 13);
  // The ant model is free floating.
  EXPECT_EQ(
      antSkel->getRootJoint()->getType(), dynamics::FreeJoint::getStaticType());

  auto floorSkele
      = world->getSkeleton(options.mGeomSkeletonNamePrefix + "floor");
  ASSERT_NE(floorSkele, nullptr);
}

//==============================================================================
TEST(MjcfParserTest, InvertedPendulum)
{
  const auto uri = "dart://sample/mjcf/openai/inverted_pendulum.xml";
  const auto options = utils::MjcfParser::Options();
  auto world = utils::MjcfParser::readWorld(uri);
  ASSERT_NE(world, nullptr);

  // The number of skeletons are the sum of the number of root <geom>s and root
  // <body>s in <worldbody>.
  ASSERT_EQ(world->getNumSkeletons(), 2);

  auto invertedPendulumSkel = world->getSkeleton("cart");
  ASSERT_NE(invertedPendulumSkel, nullptr);
  ASSERT_EQ(invertedPendulumSkel->getNumBodyNodes(), 2);
  EXPECT_EQ(
      invertedPendulumSkel->getRootJoint()->getType(),
      dynamics::PrismaticJoint::getStaticType());

  auto railSkel = world->getSkeleton(options.mGeomSkeletonNamePrefix + "rail");
  ASSERT_NE(railSkel, nullptr);
}

//==============================================================================
TEST(MjcfParserTest, InvertedDoublePendulum)
{
  const auto uri = "dart://sample/mjcf/openai/inverted_double_pendulum.xml";
  const auto options = utils::MjcfParser::Options();
  auto world = utils::MjcfParser::readWorld(uri);
  ASSERT_NE(world, nullptr);

  // The number of skeletons are the sum of the number of root <geom>s and root
  // <body>s in <worldbody>.
  ASSERT_EQ(world->getNumSkeletons(), 3);

  auto invertedPendulumSkel = world->getSkeleton("cart");
  ASSERT_NE(invertedPendulumSkel, nullptr);
  ASSERT_EQ(invertedPendulumSkel->getNumBodyNodes(), 3);
  EXPECT_EQ(
      invertedPendulumSkel->getRootJoint()->getType(),
      dynamics::PrismaticJoint::getStaticType());

  auto floorSkele
      = world->getSkeleton(options.mGeomSkeletonNamePrefix + "floor");
  ASSERT_NE(floorSkele, nullptr);

  auto railSkel = world->getSkeleton(options.mGeomSkeletonNamePrefix + "rail");
  ASSERT_NE(railSkel, nullptr);
}

//==============================================================================
TEST(MjcfParserTest, Reacher)
{
  const auto uri = "dart://sample/mjcf/openai/reacher.xml";
  const auto options = utils::MjcfParser::Options();
  auto world = utils::MjcfParser::readWorld(uri);
  ASSERT_NE(world, nullptr);

  // The number of skeletons are the sum of the number of root <geom>s and root
  // <body>s in <worldbody>.
  ASSERT_EQ(world->getNumSkeletons(), 8);

  auto reacherSkel = world->getSkeleton("body0");
  ASSERT_NE(reacherSkel, nullptr);
  ASSERT_EQ(reacherSkel->getNumBodyNodes(), 3);
  EXPECT_EQ(
      reacherSkel->getRootJoint()->getType(),
      dynamics::RevoluteJoint::getStaticType());

  auto targetSkel = world->getSkeleton("target");
  ASSERT_NE(targetSkel, nullptr);

  EXPECT_TRUE(world->hasSkeleton(options.mGeomSkeletonNamePrefix + "ground"));
  EXPECT_TRUE(world->hasSkeleton(options.mGeomSkeletonNamePrefix + "sideS"));
  EXPECT_TRUE(world->hasSkeleton(options.mGeomSkeletonNamePrefix + "sideE"));
  EXPECT_TRUE(world->hasSkeleton(options.mGeomSkeletonNamePrefix + "sideN"));
  EXPECT_TRUE(world->hasSkeleton(options.mGeomSkeletonNamePrefix + "sideW"));
  EXPECT_TRUE(world->hasSkeleton(options.mGeomSkeletonNamePrefix + "root"));
}

//==============================================================================
TEST(MjcfParserTest, Striker)
{
  const auto uri = "dart://sample/mjcf/openai/striker.xml";
  const auto options = utils::MjcfParser::Options();
  auto world = utils::MjcfParser::readWorld(uri);
  ASSERT_NE(world, nullptr);

  // The number of skeletons are the sum of the number of root <geom>s and root
  // <body>s in <worldbody>.
  ASSERT_EQ(world->getNumSkeletons(), 4);

  auto strikerSkel = world->getSkeleton("r_shoulder_pan_link");
  ASSERT_NE(strikerSkel, nullptr);
  ASSERT_EQ(strikerSkel->getNumBodyNodes(), 10);
  EXPECT_EQ(
      strikerSkel->getRootJoint()->getType(),
      dynamics::RevoluteJoint::getStaticType());

  auto objectSkel = world->getSkeleton("object");
  ASSERT_NE(objectSkel, nullptr);

  auto goalSkel = world->getSkeleton("goal");
  ASSERT_NE(goalSkel, nullptr);

  EXPECT_TRUE(world->hasSkeleton(options.mGeomSkeletonNamePrefix + "table"));
}

//==============================================================================
TEST(MjcfParserTest, Thrower)
{
  const auto uri = "dart://sample/mjcf/openai/thrower.xml";
  const auto options = utils::MjcfParser::Options();
  auto world = utils::MjcfParser::readWorld(uri);
  ASSERT_NE(world, nullptr);

  // The number of skeletons are the sum of the number of root <geom>s and root
  // <body>s in <worldbody>.
  ASSERT_EQ(world->getNumSkeletons(), 4);

  auto throwerSkel = world->getSkeleton("r_shoulder_pan_link");
  ASSERT_NE(throwerSkel, nullptr);
  ASSERT_EQ(throwerSkel->getNumBodyNodes(), 16);
  EXPECT_EQ(
      throwerSkel->getRootJoint()->getType(),
      dynamics::RevoluteJoint::getStaticType());

  auto goalSkel = world->getSkeleton("goal");
  ASSERT_NE(goalSkel, nullptr);

  auto ballSkel = world->getSkeleton("ball");
  ASSERT_NE(ballSkel, nullptr);

  EXPECT_TRUE(world->hasSkeleton(options.mGeomSkeletonNamePrefix + ""));
}

//==============================================================================
TEST(MjcfParserTest, RoboticsFetch)
{
  const common::Uri uri
      = "dart://sample/mjcf/openai/robotics/fetch/pick_and_place.xml";
  auto world = utils::MjcfParser::readWorld(uri);
  ASSERT_NE(world, nullptr);

  ASSERT_EQ(world->getNumSkeletons(), 6);
}

//==============================================================================
// Bodies with more <joint>s than DART has a predefined multi-DOF joint for
// (i.e. anything other than 2 or 3 slide joints) are represented as a chain
// of single-DOF joints connected by intermediate massless BodyNodes. Those
// BodyNodes are named "<ownerBodyName>__mjcf_dof<i>" and carry no shapes.
std::size_t countStackedJointDummyBodies(const dynamics::SkeletonPtr& skel)
{
  std::size_t count = 0u;
  for (auto i = 0u; i < skel->getNumBodyNodes(); ++i) {
    if (skel->getBodyNode(i)->getName().find("__mjcf_dof")
        != std::string::npos) {
      ++count;
    }
  }
  return count;
}

//==============================================================================
TEST(MjcfParserTest, HumanoidStackedHingeJoints)
{
  // The humanoid model has several bodies with 2 or 3 stacked hinge joints
  // (lwaist: 2, right_thigh/left_thigh: 3, right_upper_arm/left_upper_arm: 2)
  // that are not any of the predefined multi-DOF joint compositions.
  const auto uri = "dart://sample/mjcf/openai/humanoid.xml";
  auto world = utils::MjcfParser::readWorld(uri);
  ASSERT_NE(world, nullptr);

  ASSERT_EQ(world->getNumSkeletons(), 2);

  auto humanoidSkel = world->getSkeleton("torso");
  ASSERT_NE(humanoidSkel, nullptr);

  // 6 DOFs from the free root joint + 17 DOFs from the hinge joints.
  EXPECT_EQ(humanoidSkel->getNumDofs(), 23u);

  // 13 real bodies plus 7 massless intermediate bodies inserted to represent
  // the joint stacks: lwaist (2 joints -> 1 dummy), right_thigh and
  // left_thigh (3 joints -> 2 dummies each), right_upper_arm and
  // left_upper_arm (2 joints -> 1 dummy each).
  EXPECT_EQ(humanoidSkel->getNumBodyNodes(), 20u);
  EXPECT_EQ(countStackedJointDummyBodies(humanoidSkel), 7u);

  const std::vector<std::string> realBodyNames
      = {"torso",
         "lwaist",
         "pelvis",
         "right_thigh",
         "right_shin",
         "right_foot",
         "left_thigh",
         "left_shin",
         "left_foot",
         "right_upper_arm",
         "right_lower_arm",
         "left_upper_arm",
         "left_lower_arm"};
  for (const auto& name : realBodyNames) {
    EXPECT_NE(humanoidSkel->getBodyNode(name), nullptr) << name;
  }

  EXPECT_EQ(
      humanoidSkel->getRootJoint()->getType(),
      dynamics::FreeJoint::getStaticType());

  world->getConstraintSolver()->setCollisionDetector(
      collision::NativeCollisionDetector::create());

  for (auto i = 0; i < 100; ++i) {
    world->step();
  }

  EXPECT_TRUE(humanoidSkel->getPositions().allFinite());
  EXPECT_TRUE(humanoidSkel->getVelocities().allFinite());
}

//==============================================================================
TEST(MjcfParserTest, HumanoidStandupStackedHingeJoints)
{
  // Same body/joint topology as humanoid.xml, just different geometry.
  const auto uri = "dart://sample/mjcf/openai/humanoidstandup.xml";
  auto world = utils::MjcfParser::readWorld(uri);
  ASSERT_NE(world, nullptr);

  ASSERT_EQ(world->getNumSkeletons(), 2);

  auto humanoidSkel = world->getSkeleton("torso");
  ASSERT_NE(humanoidSkel, nullptr);

  EXPECT_EQ(humanoidSkel->getNumDofs(), 23u);
  EXPECT_EQ(humanoidSkel->getNumBodyNodes(), 20u);
  EXPECT_EQ(countStackedJointDummyBodies(humanoidSkel), 7u);

  for (auto i = 0; i < 10; ++i) {
    world->step();
  }

  EXPECT_TRUE(humanoidSkel->getPositions().allFinite());
  EXPECT_TRUE(humanoidSkel->getVelocities().allFinite());
}

//==============================================================================
TEST(MjcfParserTest, Walker2dMixedJointStack)
{
  // torso has a 2-slide + 1-hinge stack (rootx, rootz, rooty), which is not
  // the predefined 3-slide TranslationalJoint composition.
  const auto uri = "dart://sample/mjcf/openai/walker2d.xml";
  auto world = utils::MjcfParser::readWorld(uri);
  ASSERT_NE(world, nullptr);

  ASSERT_EQ(world->getNumSkeletons(), 2);

  auto skel = world->getSkeleton("torso");
  ASSERT_NE(skel, nullptr);

  // 7 real bodies (torso, thigh, leg, foot, thigh_left, leg_left, foot_left)
  // + 2 dummies for the torso's 3-joint stack.
  EXPECT_EQ(skel->getNumBodyNodes(), 9u);
  EXPECT_EQ(countStackedJointDummyBodies(skel), 2u);

  // 3 DOFs from torso's stack + 1 DOF from each of the 6 single-hinge bodies.
  EXPECT_EQ(skel->getNumDofs(), 9u);

  for (auto i = 0; i < 10; ++i) {
    world->step();
  }

  EXPECT_TRUE(skel->getPositions().allFinite());
  EXPECT_TRUE(skel->getVelocities().allFinite());
}

//==============================================================================
TEST(MjcfParserTest, HopperMixedJointStack)
{
  const auto uri = "dart://sample/mjcf/openai/hopper.xml";
  auto world = utils::MjcfParser::readWorld(uri);
  ASSERT_NE(world, nullptr);

  ASSERT_EQ(world->getNumSkeletons(), 2);

  auto skel = world->getSkeleton("torso");
  ASSERT_NE(skel, nullptr);

  // 4 real bodies (torso, thigh, leg, foot) + 2 dummies for torso's stack.
  EXPECT_EQ(skel->getNumBodyNodes(), 6u);
  EXPECT_EQ(countStackedJointDummyBodies(skel), 2u);
  EXPECT_EQ(skel->getNumDofs(), 6u);

  for (auto i = 0; i < 10; ++i) {
    world->step();
  }

  EXPECT_TRUE(skel->getPositions().allFinite());
  EXPECT_TRUE(skel->getVelocities().allFinite());
}

//==============================================================================
TEST(MjcfParserTest, HalfCheetahMixedJointStack)
{
  const auto uri = "dart://sample/mjcf/openai/half_cheetah.xml";
  auto world = utils::MjcfParser::readWorld(uri);
  ASSERT_NE(world, nullptr);

  ASSERT_EQ(world->getNumSkeletons(), 2);

  auto skel = world->getSkeleton("torso");
  ASSERT_NE(skel, nullptr);

  // 7 real bodies (torso, bthigh, bshin, bfoot, fthigh, fshin, ffoot) + 2
  // dummies for torso's stack.
  EXPECT_EQ(skel->getNumBodyNodes(), 9u);
  EXPECT_EQ(countStackedJointDummyBodies(skel), 2u);
  EXPECT_EQ(skel->getNumDofs(), 9u);

  for (auto i = 0; i < 10; ++i) {
    world->step();
  }

  EXPECT_TRUE(skel->getPositions().allFinite());
  EXPECT_TRUE(skel->getVelocities().allFinite());
}

//==============================================================================
TEST(MjcfParserTest, SwimmerMixedJointStack)
{
  const auto uri = "dart://sample/mjcf/openai/swimmer.xml";
  auto world = utils::MjcfParser::readWorld(uri);
  ASSERT_NE(world, nullptr);

  ASSERT_EQ(world->getNumSkeletons(), 2);

  auto skel = world->getSkeleton("torso");
  ASSERT_NE(skel, nullptr);

  // 3 real bodies (torso, mid, back) + 2 dummies for torso's stack.
  EXPECT_EQ(skel->getNumBodyNodes(), 5u);
  EXPECT_EQ(countStackedJointDummyBodies(skel), 2u);
  EXPECT_EQ(skel->getNumDofs(), 5u);

  for (auto i = 0; i < 10; ++i) {
    world->step();
  }

  EXPECT_TRUE(skel->getPositions().allFinite());
  EXPECT_TRUE(skel->getVelocities().allFinite());
}

//==============================================================================
TEST(MjcfParserTest, UnnamedRootBodyWithUnnamedJointStack)
{
  // Regression: a root <body> without a name whose joints are also unnamed
  // used to crash in the stacked-joint chain path (nullptr parent BodyNode in
  // the unnamed-joint name fallback), and unnamed root bodies must keep
  // inheriting a generated BodyNode name instead of a synthetic
  // "__mjcf_dof" link name or the World's generic default.
  const auto uri = "dart://sample/mjcf/test/unnamed_root_joint_stack.xml";

  auto world = utils::MjcfParser::readWorld(uri);
  ASSERT_NE(world, nullptr);

  dynamics::SkeletonPtr stacked = nullptr;
  for (std::size_t i = 0u; i < world->getNumSkeletons(); ++i) {
    auto skel = world->getSkeleton(i);
    if (skel->getNumDofs() == 2u) {
      stacked = skel;
      break;
    }
  }
  ASSERT_NE(stacked, nullptr);

  // Two chained revolute joints: one synthetic intermediate link plus the
  // real body.
  EXPECT_EQ(stacked->getNumBodyNodes(), 2u);
  EXPECT_NE(stacked->getRootJoint(), nullptr);

  // The skeleton name must come from the real (non-synthetic) BodyNode.
  EXPECT_EQ(stacked->getName().find("__mjcf_dof"), std::string::npos);
  EXPECT_FALSE(stacked->getName().empty());

  for (auto i = 0; i < 10; ++i) {
    world->step();
  }

  EXPECT_TRUE(stacked->getPositions().allFinite());
  EXPECT_TRUE(stacked->getVelocities().allFinite());
}
