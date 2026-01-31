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

#include "helpers/gtest_utils.hpp"

#include "dart/utils/All.hpp"
#include "dart/utils/mjcf/detail/mujoco_model.hpp"
#include "dart/utils/mjcf/detail/types.hpp"
#include "dart/utils/mjcf/detail/utils.hpp"

#include <dart/all.hpp>

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

using namespace dart;
using namespace dart::test;
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
TEST(MjcfParserTest, LoadsDefaultWorld)
{
  const auto uri = "dart://sample/mjcf/test/default.xml";
  auto world = utils::MjcfParser::readWorld(uri);
  ASSERT_NE(world, nullptr);
  EXPECT_GE(world->getNumSkeletons(), 1);
}

//==============================================================================
TEST(MjcfParserTest, ParsesSitesAndOptions)
{
  const std::string xml = R"(
<?xml version="1.0" ?>
<mujoco model="site_option">
  <default>
    <geom type="sphere" size="0.1" />
  </default>
  <option timestep="0.01" apirate="30" impratio="2"
          gravity="0 0 -9.81" wind="1 2 3" magnetic="0.1 0.2 0.3"
          density="1000" viscosity="0.5" integrator="RK4"
          collision="dynamic" cone="pyramidal" jacobian="sparse"
          solver="Newton" iterations="20" tolerance="1e-6"
          mpr_iterations="3" mpr_tolerance="1e-5" />
  <worldbody>
    <body name="root">
      <geom type="sphere" size="0.1" />
      <site name="capsule_site" type="capsule" size="0.2"
            fromto="0 0 0 0 0 2" />
      <site name="box_site" type="box" size="0.1 0.2 0.3"
            pos="1 2 3" euler="0 90 0" />
    </body>
  </worldbody>
</mujoco>
)";

  const auto tempPath
      = std::filesystem::temp_directory_path() / "dart_mjcf_sites.xml";
  std::ofstream output(tempPath.string(), std::ios::binary);
  output << xml;
  output.close();

  auto mujoco = utils::MjcfParser::detail::MujocoModel();
  auto errors = mujoco.read(common::Uri(tempPath.string()), createRetriever());
  std::error_code ec;
  std::filesystem::remove(tempPath, ec);
  ASSERT_TRUE(errors.empty());

  const auto& option = mujoco.getOption();
  EXPECT_DOUBLE_EQ(option.getTimestep(), 0.01);
  EXPECT_EQ(option.getIntegrator(), Integrator::RK4);
  EXPECT_EQ(option.getCollision(), CollisionType::DYNAMIC);
  EXPECT_EQ(option.getCone(), ConeType::PYRAMIDAL);
  EXPECT_EQ(option.getJacobian(), JacobianType::SPARSE);
  EXPECT_EQ(option.getSolver(), SolverType::NEWTON);
  EXPECT_EQ(option.getIterations(), 20);
  EXPECT_DOUBLE_EQ(option.getTolerance(), 1e-6);
  EXPECT_EQ(option.getMprIterations(), 3);
  EXPECT_DOUBLE_EQ(option.getMprTolerance(), 1e-5);
  EXPECT_TRUE(option.getGravity().isApprox(Eigen::Vector3d(0, 0, -9.81)));
  EXPECT_TRUE(option.getWind().isApprox(Eigen::Vector3d(1, 2, 3)));
  EXPECT_TRUE(option.getMagnetic().isApprox(Eigen::Vector3d(0.1, 0.2, 0.3)));
  EXPECT_DOUBLE_EQ(option.getDensity(), 1000);
  EXPECT_DOUBLE_EQ(option.getViscosity(), 0.5);

  const auto& worldbody = mujoco.getWorldbody();
  ASSERT_EQ(worldbody.getNumRootBodies(), 1u);
  const auto& body = worldbody.getRootBody(0);
  ASSERT_EQ(body.getNumSites(), 2u);

  const auto& capsuleSite = body.getSite(0);
  EXPECT_EQ(capsuleSite.getType(), GeomType::CAPSULE);
  EXPECT_NEAR(capsuleSite.getCapsuleRadius(), 0.2, 1e-12);
  EXPECT_NEAR(capsuleSite.getCapsuleHalfLength(), 1.0, 1e-12);
  EXPECT_TRUE(capsuleSite.getRelativeTransform().translation().isApprox(
      Eigen::Vector3d(0, 0, 1)));

  const auto& boxSite = body.getSite(1);
  EXPECT_EQ(boxSite.getType(), GeomType::BOX);
  EXPECT_TRUE(boxSite.getRelativeTransform().translation().isApprox(
      Eigen::Vector3d(1, 2, 3)));
}

//==============================================================================
TEST(MjcfParserTest, ReportsInvalidOptionAndSiteType)
{
  const std::string xml = R"(
<?xml version="1.0" ?>
<mujoco model="bad_option_site">
  <default>
    <geom type="sphere" size="0.1" />
  </default>
  <option integrator="bad_integrator" />
  <worldbody>
    <body name="root">
      <geom type="sphere" size="0.1" />
      <site name="bad_site" type="bad_type" size="0.1" />
    </body>
  </worldbody>
</mujoco>
)";

  const auto tempPath
      = std::filesystem::temp_directory_path() / "dart_mjcf_bad_option.xml";
  std::ofstream output(tempPath.string(), std::ios::binary);
  output << xml;
  output.close();

  auto mujoco = utils::MjcfParser::detail::MujocoModel();
  auto errors = mujoco.read(common::Uri(tempPath.string()), createRetriever());
  std::error_code ec;
  std::filesystem::remove(tempPath, ec);

  ASSERT_FALSE(errors.empty());
  bool hasInvalid = false;
  for (const auto& error : errors) {
    if (error.getCode() == ErrorCode::ATTRIBUTE_INVALID) {
      hasInvalid = true;
      break;
    }
  }
  EXPECT_TRUE(hasInvalid);
}

//==============================================================================
TEST(MjcfParserTest, ReportsInvalidSiteSize)
{
  const std::string xml = R"(
<?xml version="1.0" ?>
<mujoco model="bad_site_size">
  <default>
    <geom type="sphere" size="0.1" />
  </default>
  <worldbody>
    <body name="root">
      <geom type="sphere" size="0.1" />
      <site name="bad_size" type="box" size="0.1 0.2 0.3 0.4" />
    </body>
  </worldbody>
</mujoco>
)";

  const auto tempPath
      = std::filesystem::temp_directory_path() / "dart_mjcf_bad_size.xml";
  std::ofstream output(tempPath.string(), std::ios::binary);
  output << xml;
  output.close();

  auto mujoco = utils::MjcfParser::detail::MujocoModel();
  auto errors = mujoco.read(common::Uri(tempPath.string()), createRetriever());
  std::error_code ec;
  std::filesystem::remove(tempPath, ec);

  ASSERT_FALSE(errors.empty());
  bool hasInvalid = false;
  for (const auto& error : errors) {
    if (error.getCode() == ErrorCode::ATTRIBUTE_INVALID) {
      hasInvalid = true;
      break;
    }
  }
  EXPECT_TRUE(hasInvalid);
}
