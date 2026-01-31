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

//==============================================================================
TEST(MjcfParserTest, CompilerAttributesParsed)
{
  const std::string xml = R"(
<?xml version="1.0" ?>
<mujoco model="compiler_attrs">
  <compiler boundmass="1.2" boundinertia="0.3" settotalmass="5.5"
            balanceinertia="true" strippath="true" coordinate="global"
            angle="radian" fitaabb="true" eulerseq="zyx"
            meshdir="meshes" texturedir="textures" discardvisual="true"
            convexhull="false" userthread="true" fusestatic="true"
            inertiafromgeom="auto" inertiagrouprange="1 5" />
  <default>
    <geom type="sphere" size="0.1" />
  </default>
  <worldbody>
    <body name="root">
      <geom type="sphere" size="0.1" />
    </body>
  </worldbody>
</mujoco>
 )";

  const auto tempPath
      = std::filesystem::temp_directory_path() / "dart_mjcf_compiler_attrs.xml";
  std::ofstream output(tempPath.string(), std::ios::binary);
  output << xml;
  output.close();

  auto mujoco = utils::MjcfParser::detail::MujocoModel();
  auto errors = mujoco.read(common::Uri(tempPath.string()), createRetriever());
  std::error_code ec;
  std::filesystem::remove(tempPath, ec);

  ASSERT_TRUE(errors.empty());

  const auto& compiler = mujoco.getCompiler();
  EXPECT_DOUBLE_EQ(compiler.getBoundMass(), 1.2);
  EXPECT_DOUBLE_EQ(compiler.getBoundInertia(), 0.3);
  EXPECT_DOUBLE_EQ(compiler.getSetTotalMass(), 5.5);
  EXPECT_TRUE(compiler.getBalanceInertia());
  EXPECT_TRUE(compiler.getStripPath());
  EXPECT_EQ(compiler.getCoordinate(), Coordinate::GLOBAL);
  EXPECT_EQ(compiler.getAngle(), Angle::RADIAN);
  EXPECT_TRUE(compiler.getFitAabb());
  EXPECT_EQ(compiler.getEulerSeq(), "zyx");
  EXPECT_EQ(compiler.getMeshDir(), "meshes");
  EXPECT_EQ(compiler.getTextureDir(), "textures");
  EXPECT_TRUE(compiler.getDiscardVisual());
  EXPECT_FALSE(compiler.getConvexHull());
  EXPECT_TRUE(compiler.getUserThread());
  EXPECT_TRUE(compiler.getFuseStatic());
  EXPECT_EQ(compiler.getInertiaFromGeom(), InertiaFromGeom::IFG_AUTO);
  EXPECT_EQ(compiler.getInertiaGroupRange(), Eigen::Vector2i(1, 5));
}

//==============================================================================
TEST(MjcfParserTest, CompilerReportsInvalidAngle)
{
  const std::string xml = R"(
<?xml version="1.0" ?>
<mujoco model="compiler_invalid">
  <compiler angle="invalid" />
  <default>
    <geom type="sphere" size="0.1" />
  </default>
  <worldbody>
    <body name="root">
      <geom type="sphere" size="0.1" />
    </body>
  </worldbody>
</mujoco>
 )";

  const auto tempPath
      = std::filesystem::temp_directory_path() / "dart_mjcf_bad_compiler.xml";
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
TEST(MjcfParserTest, CompilerReportsInvalidCoordinate)
{
  const std::string xml = R"(
<?xml version="1.0" ?>
<mujoco model="compiler_invalid_coord">
  <compiler coordinate="invalid" />
  <default>
    <geom type="sphere" size="0.1" />
  </default>
  <worldbody>
    <body name="root">
      <geom type="sphere" size="0.1" />
    </body>
  </worldbody>
</mujoco>
 )";

  const auto tempPath = std::filesystem::temp_directory_path()
                        / "dart_mjcf_bad_compiler_coord.xml";
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
TEST(MjcfParserTest, GeomAttributesDefaultClassOverrides)
{
  const std::string xml = R"(
<?xml version="1.0" ?>
<mujoco model="geom_defaults">
  <default>
    <geom type="sphere" size="0.1" />
    <default class="blue">
      <geom type="box" size="0.2 0.3 0.4" rgba="0 0 1 1"
            friction="0.2 0.3 0.4" solmix="0.9" margin="0.01" gap="0.02"
            contype="2" conaffinity="3" condim="4" group="2" priority="5"
            density="500" />
    </default>
  </default>
  <worldbody>
    <body name="root">
      <geom class="blue" name="box_geom" pos="1 2 3" euler="0 0 90" />
    </body>
  </worldbody>
</mujoco>
 )";

  const auto tempPath
      = std::filesystem::temp_directory_path() / "dart_mjcf_geom_defaults.xml";
  std::ofstream output(tempPath.string(), std::ios::binary);
  output << xml;
  output.close();

  auto mujoco = utils::MjcfParser::detail::MujocoModel();
  auto errors = mujoco.read(common::Uri(tempPath.string()), createRetriever());
  std::error_code ec;
  std::filesystem::remove(tempPath, ec);

  ASSERT_TRUE(errors.empty());

  const auto& worldbody = mujoco.getWorldbody();
  ASSERT_EQ(worldbody.getNumRootBodies(), 1u);
  const auto& body = worldbody.getRootBody(0);
  ASSERT_EQ(body.getNumGeoms(), 1u);
  const auto& geom = body.getGeom(0);
  EXPECT_EQ(geom.getType(), GeomType::BOX);
  EXPECT_TRUE(geom.getBoxHalfSize().isApprox(Eigen::Vector3d(0.2, 0.3, 0.4)));
  EXPECT_TRUE(geom.getRGBA().isApprox(Eigen::Vector4d(0, 0, 1, 1)));
  EXPECT_TRUE(geom.getFriction().isApprox(Eigen::Vector3d(0.2, 0.3, 0.4)));
  EXPECT_DOUBLE_EQ(geom.getSolMix(), 0.9);
  EXPECT_DOUBLE_EQ(geom.getMargine(), 0.01);
  EXPECT_DOUBLE_EQ(geom.getGap(), 0.02);
  EXPECT_EQ(geom.getConType(), 2);
  EXPECT_EQ(geom.getConAffinity(), 3);
  EXPECT_EQ(geom.getConDim(), 4);
  EXPECT_EQ(geom.getGroup(), 2);
  EXPECT_EQ(geom.getPriority(), 5);
  EXPECT_DOUBLE_EQ(geom.getDensity(), 500.0);
  EXPECT_TRUE(geom.getRelativeTransform().translation().isApprox(
      Eigen::Vector3d(1, 2, 3)));
}

//==============================================================================
TEST(MjcfParserTest, GeomFromToAndMeshErrors)
{
  const std::string xml = R"(
<?xml version="1.0" ?>
<mujoco model="geom_fromto">
  <default>
    <geom type="sphere" size="0.1" />
  </default>
  <worldbody>
    <body name="root">
      <geom type="sphere" size="0.1" />
      <geom name="capsule" type="capsule" size="0.2"
            fromto="0 0 0 0 0 2" />
      <geom name="box" type="box" size="0.1 0.2 0.3"
            fromto="0 0 0 0 0 4" />
      <geom name="badmesh" type="mesh" />
      <geom name="meshfit" type="box" mesh="dummy_mesh" size="0.1 0.1 0.1" />
    </body>
  </worldbody>
</mujoco>
 )";

  const auto tempPath
      = std::filesystem::temp_directory_path() / "dart_mjcf_geom_fromto.xml";
  std::ofstream output(tempPath.string(), std::ios::binary);
  output << xml;
  output.close();

  auto mujoco = utils::MjcfParser::detail::MujocoModel();
  auto errors = mujoco.read(common::Uri(tempPath.string()), createRetriever());
  std::error_code ec;
  std::filesystem::remove(tempPath, ec);

  ASSERT_FALSE(errors.empty());
  bool hasMissing = false;
  bool hasUndefined = false;
  for (const auto& error : errors) {
    if (error.getCode() == ErrorCode::ATTRIBUTE_MISSING) {
      hasMissing = true;
    }
    if (error.getCode() == ErrorCode::UNDEFINED_ERROR) {
      hasUndefined = true;
    }
  }
  EXPECT_TRUE(hasMissing);
  EXPECT_TRUE(hasUndefined);

  const auto& body = mujoco.getWorldbody().getRootBody(0);
  ASSERT_GE(body.getNumGeoms(), 4u);

  const auto findGeom = [&body](const std::string& name) {
    for (std::size_t i = 0; i < body.getNumGeoms(); ++i) {
      const auto& geom = body.getGeom(i);
      if (geom.getName() == name) {
        return &geom;
      }
    }
    return static_cast<const Geom*>(nullptr);
  };

  const auto* capsule = findGeom("capsule");
  ASSERT_NE(capsule, nullptr);
  EXPECT_EQ(capsule->getType(), GeomType::CAPSULE);
  EXPECT_NEAR(capsule->getCapsuleRadius(), 0.2, 1e-12);
  EXPECT_NEAR(capsule->getCapsuleHalfLength(), 1.0, 1e-12);
  EXPECT_TRUE(capsule->getRelativeTransform().translation().isApprox(
      Eigen::Vector3d(0, 0, 1)));

  const auto* box = findGeom("box");
  ASSERT_NE(box, nullptr);
  EXPECT_EQ(box->getType(), GeomType::BOX);
  EXPECT_NEAR(box->getBoxHalfSize().z(), 2.0, 1e-12);

  const auto* meshfit = findGeom("meshfit");
  ASSERT_NE(meshfit, nullptr);
  EXPECT_EQ(meshfit->getType(), GeomType::BOX);
  EXPECT_TRUE(
      meshfit->getBoxHalfSize().isApprox(Eigen::Vector3d::Constant(0.1)));
}

//==============================================================================
TEST(MjcfParserTest, SiteEllipsoidAndCylinderFromTo)
{
  const std::string xml = R"(
<?xml version="1.0" ?>
<mujoco model="site_types">
  <default>
    <geom type="sphere" size="0.1" />
  </default>
  <worldbody>
    <body name="root">
      <geom type="sphere" size="0.1" />
      <site name="ellipsoid_site" type="ellipsoid" size="0.2 0.3 0.4"
            pos="1 2 3" />
      <site name="cylinder_site" type="cylinder" size="0.1"
            fromto="0 0 0 0 0 2" />
    </body>
  </worldbody>
</mujoco>
 )";

  const auto tempPath
      = std::filesystem::temp_directory_path() / "dart_mjcf_sites_more.xml";
  std::ofstream output(tempPath.string(), std::ios::binary);
  output << xml;
  output.close();

  auto mujoco = utils::MjcfParser::detail::MujocoModel();
  auto errors = mujoco.read(common::Uri(tempPath.string()), createRetriever());
  std::error_code ec;
  std::filesystem::remove(tempPath, ec);
  ASSERT_TRUE(errors.empty());

  const auto& body = mujoco.getWorldbody().getRootBody(0);
  ASSERT_EQ(body.getNumSites(), 2u);
  const auto& ellipsoid = body.getSite(0);
  EXPECT_EQ(ellipsoid.getType(), GeomType::ELLIPSOID);
  EXPECT_TRUE(ellipsoid.getRelativeTransform().translation().isApprox(
      Eigen::Vector3d(1, 2, 3)));

  const auto& cylinder = body.getSite(1);
  EXPECT_EQ(cylinder.getType(), GeomType::CYLINDER);
  EXPECT_NEAR(cylinder.getCylinderRadius(), 0.1, 1e-12);
  EXPECT_NEAR(cylinder.getCylinderHalfLength(), 1.0, 1e-12);
}

//==============================================================================
TEST(MjcfParserTest, BodyUserAttributeMissingSize)
{
  const std::string xml = R"(
<?xml version="1.0" ?>
<mujoco model="body_user_invalid">
  <default>
    <geom type="sphere" size="0.1" />
  </default>
  <worldbody>
    <body name="root" user="1 2">
      <geom type="sphere" size="0.1" />
    </body>
  </worldbody>
</mujoco>
 )";

  const auto tempPath
      = std::filesystem::temp_directory_path() / "dart_mjcf_body_user.xml";
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
TEST(MjcfParserTest, JointAttributesParsed)
{
  const std::string xml = R"(
<?xml version="1.0" ?>
<mujoco model="joint_attrs">
  <default>
    <geom type="sphere" size="0.1" />
  </default>
  <worldbody>
    <body name="root">
      <geom type="sphere" size="0.1" />
      <joint name="hinge_joint" type="hinge" pos="0.1 0.2 0.3"
             axis="0 1 0" range="-0.5 0.5" damping="0.2" springref="0.1" />
    </body>
  </worldbody>
</mujoco>
 )";

  const auto tempPath
      = std::filesystem::temp_directory_path() / "dart_mjcf_joint_attrs.xml";
  std::ofstream output(tempPath.string(), std::ios::binary);
  output << xml;
  output.close();

  auto mujoco = utils::MjcfParser::detail::MujocoModel();
  auto errors = mujoco.read(common::Uri(tempPath.string()), createRetriever());
  std::error_code ec;
  std::filesystem::remove(tempPath, ec);

  ASSERT_TRUE(errors.empty());
  const auto& body = mujoco.getWorldbody().getRootBody(0);
  ASSERT_EQ(body.getNumJoints(), 1u);
  const auto& joint = body.getJoint(0);
  EXPECT_EQ(joint.getType(), JointType::HINGE);
  EXPECT_TRUE(joint.getPos().isApprox(Eigen::Vector3d(0.1, 0.2, 0.3)));
  EXPECT_TRUE(joint.getAxis().isApprox(Eigen::Vector3d(0, 1, 0)));
  EXPECT_TRUE(joint.getRange().isApprox(Eigen::Vector2d(-0.5, 0.5)));
  EXPECT_DOUBLE_EQ(joint.getDamping(), 0.2);
  EXPECT_DOUBLE_EQ(joint.getSpringRef(), 0.1);
}

//==============================================================================
TEST(MjcfParserTest, TranslationalJointCreation)
{
  const std::string xml = R"(
<?xml version="1.0" ?>
<mujoco model="translational_joint">
  <default>
    <geom type="sphere" size="0.1" />
  </default>
  <worldbody>
    <body name="root">
      <geom type="sphere" size="0.1" />
      <joint name="slide_x" type="slide" axis="1 0 0" range="-1 1" />
      <joint name="slide_y" type="slide" axis="0 1 0" range="-1 1" />
      <body name="child">
        <geom type="sphere" size="0.1" />
      </body>
    </body>
  </worldbody>
</mujoco>
 )";

  const auto tempPath
      = std::filesystem::temp_directory_path() / "dart_mjcf_translational.xml";
  std::ofstream output(tempPath.string(), std::ios::binary);
  output << xml;
  output.close();

  auto world = utils::MjcfParser::readWorld(common::Uri(tempPath.string()));
  std::error_code ec;
  std::filesystem::remove(tempPath, ec);
  ASSERT_NE(world, nullptr);

  auto skel = world->getSkeleton("root");
  ASSERT_NE(skel, nullptr);
  auto body = skel->getBodyNode("root");
  ASSERT_NE(body, nullptr);
  EXPECT_EQ(
      body->getParentJoint()->getType(),
      dynamics::TranslationalJoint2D::getStaticType());
}

//==============================================================================
TEST(MjcfParserTest, JointTypeMappingToDart)
{
  const std::string xml = R"(
<?xml version="1.0" ?>
<mujoco model="joint_mapping">
  <default>
    <geom type="sphere" size="0.1" />
  </default>
  <worldbody>
    <body name="root">
      <geom type="sphere" size="0.1" />
      <joint name="root_free" type="free" />
      <body name="child_slide" pos="0 0 1">
        <geom type="sphere" size="0.1" />
        <joint name="slide_joint" type="slide" axis="1 0 0" range="-1 2" />
      </body>
      <body name="child_hinge" pos="0 0 2">
        <geom type="sphere" size="0.1" />
        <joint name="hinge_joint" type="hinge" axis="0 1 0" range="-0.5 0.5" />
      </body>
      <body name="child_ball" pos="0 0 3">
        <geom type="sphere" size="0.1" />
        <joint name="ball_joint" type="ball" damping="0.3" />
      </body>
    </body>
  </worldbody>
</mujoco>
 )";

  const auto tempPath
      = std::filesystem::temp_directory_path() / "dart_mjcf_joint_mapping.xml";
  std::ofstream output(tempPath.string(), std::ios::binary);
  output << xml;
  output.close();

  auto world = utils::MjcfParser::readWorld(common::Uri(tempPath.string()));
  std::error_code ec;
  std::filesystem::remove(tempPath, ec);
  ASSERT_NE(world, nullptr);

  auto skel = world->getSkeleton("root");
  ASSERT_NE(skel, nullptr);
  EXPECT_EQ(
      skel->getRootJoint()->getType(), dynamics::FreeJoint::getStaticType());

  const auto* slideBody = skel->getBodyNode("child_slide");
  ASSERT_NE(slideBody, nullptr);
  EXPECT_EQ(
      slideBody->getParentJoint()->getType(),
      dynamics::PrismaticJoint::getStaticType());

  const auto* hingeBody = skel->getBodyNode("child_hinge");
  ASSERT_NE(hingeBody, nullptr);
  EXPECT_EQ(
      hingeBody->getParentJoint()->getType(),
      dynamics::RevoluteJoint::getStaticType());

  const auto* ballBody = skel->getBodyNode("child_ball");
  ASSERT_NE(ballBody, nullptr);
  EXPECT_EQ(
      ballBody->getParentJoint()->getType(),
      dynamics::BallJoint::getStaticType());
}

//==============================================================================
TEST(MjcfParserTest, SizeAttributesParsed)
{
  const std::string xml = R"(
<?xml version="1.0" ?>
<mujoco model="size_attrs">
  <size mjmax="1024" nconmax="64" nstack="2048" nuserdata="4" nkey="3"
        nuser_body="2" nuser_jnt="5" nuser_geom="6" nuser_site="7"
        nuser_cam="8" nuser_tendon="9" nuser_actuator="10" nuser_sensor="11" />
  <default>
    <geom type="sphere" size="0.1" />
  </default>
  <worldbody>
    <body name="root">
      <geom type="sphere" size="0.1" />
    </body>
  </worldbody>
</mujoco>
  )";

  const auto tempPath
      = std::filesystem::temp_directory_path() / "dart_mjcf_size_attrs.xml";
  std::ofstream output(tempPath.string(), std::ios::binary);
  output << xml;
  output.close();

  auto mujoco = utils::MjcfParser::detail::MujocoModel();
  auto errors = mujoco.read(common::Uri(tempPath.string()), createRetriever());
  std::error_code ec;
  std::filesystem::remove(tempPath, ec);

  ASSERT_TRUE(errors.empty());
  const auto& size = mujoco.getSize();
  EXPECT_EQ(size.getMJMax(), 1024);
  EXPECT_EQ(size.getNConMax(), 64);
  EXPECT_EQ(size.getNStack(), 2048);
  EXPECT_EQ(size.getNUserData(), 4);
  EXPECT_EQ(size.getNKey(), 3);
  EXPECT_EQ(size.getNUserBody(), 2);
  EXPECT_EQ(size.getNUserJnt(), 5);
  EXPECT_EQ(size.getNUserGeom(), 6);
  EXPECT_EQ(size.getNUserSite(), 7);
  EXPECT_EQ(size.getNUserCam(), 8);
  EXPECT_EQ(size.getNUserTendon(), 9);
  EXPECT_EQ(size.getNUserActuator(), 10);
  EXPECT_EQ(size.getNUserSensor(), 11);
}

//==============================================================================
TEST(MjcfParserTest, OptionSecondaryAttributesParsed)
{
  const std::string xml = R"(
<?xml version="1.0" ?>
<mujoco model="option_secondary">
  <default>
    <geom type="sphere" size="0.1" />
  </default>
  <option timestep="0.005" apirate="120" impratio="1.5"
          noslip_iterations="5" noslip_tolerance="0.0002"
          mpr_iterations="6" mpr_tolerance="1e-4" />
  <worldbody>
    <body name="root">
      <geom type="sphere" size="0.1" />
    </body>
  </worldbody>
</mujoco>
  )";

  const auto tempPath = std::filesystem::temp_directory_path()
                        / "dart_mjcf_option_secondary.xml";
  std::ofstream output(tempPath.string(), std::ios::binary);
  output << xml;
  output.close();

  auto mujoco = utils::MjcfParser::detail::MujocoModel();
  auto errors = mujoco.read(common::Uri(tempPath.string()), createRetriever());
  std::error_code ec;
  std::filesystem::remove(tempPath, ec);

  ASSERT_TRUE(errors.empty());
  const auto& option = mujoco.getOption();
  EXPECT_DOUBLE_EQ(option.getTimestep(), 0.005);
  EXPECT_GT(option.getApiRate(), 0.0);
  EXPECT_GT(option.getImpRatio(), 0.0);
  EXPECT_GE(option.getNoSlipIterations(), 0);
  EXPECT_GE(option.getNoSlipTolerance(), 0.0);
  EXPECT_GE(option.getMprIterations(), 0);
  EXPECT_GE(option.getMprTolerance(), 0.0);
}

//==============================================================================
TEST(MjcfParserTest, SiteOrientationConflictReportsError)
{
  const std::string xml = R"(
<?xml version="1.0" ?>
<mujoco model="site_orientation_conflict">
  <default>
    <geom type="sphere" size="0.1" />
  </default>
  <worldbody>
    <body name="root">
      <geom type="sphere" size="0.1" />
      <site name="bad_site" type="box" size="0.1 0.2 0.3"
            quat="1 0 0 0" euler="0 0 90" />
    </body>
  </worldbody>
</mujoco>
  )";

  const auto tempPath = std::filesystem::temp_directory_path()
                        / "dart_mjcf_site_orientation_conflict.xml";
  std::ofstream output(tempPath.string(), std::ios::binary);
  output << xml;
  output.close();

  auto mujoco = utils::MjcfParser::detail::MujocoModel();
  auto errors = mujoco.read(common::Uri(tempPath.string()), createRetriever());
  std::error_code ec;
  std::filesystem::remove(tempPath, ec);

  ASSERT_FALSE(errors.empty());
  bool hasConflict = false;
  for (const auto& error : errors) {
    if (error.getCode() == ErrorCode::ATTRIBUTE_CONFLICT) {
      hasConflict = true;
      break;
    }
  }
  EXPECT_TRUE(hasConflict);
}

//==============================================================================
TEST(MjcfParserTest, SiteAttributesAndOrientations)
{
  const std::string xml = R"(
<?xml version="1.0" ?>
<mujoco model="site_attrs">
  <default>
    <geom type="sphere" size="0.1" />
  </default>
  <worldbody>
    <body name="root">
      <geom type="sphere" size="0.1" />
      <site name="cylinder_site" type="cylinder" size="0.2"
            fromto="0 0 0 0 0 2" rgba="0.1 0.2 0.3 0.4" group="2" />
      <site name="capsule_site" type="capsule" size="0.1"
            fromto="0 0 0 0 0 1" />
      <site name="ellipsoid_site" type="ellipsoid" size="0.2 0.3 0.4"
            pos="1 2 3" axisangle="1 0 0 1.57079632679" />
      <site name="box_site" type="box" size="0.1 0.2 0.3"
            pos="-1 -2 -3" quat="0.7071068 0 0.7071068 0" />
      <site name="euler_site" type="box" size="0.1 0.1 0.1"
            pos="0 1 0" euler="0 90 0" class="custom" />
    </body>
  </worldbody>
</mujoco>
 )";

  const auto tempPath
      = std::filesystem::temp_directory_path() / "dart_mjcf_site_attrs.xml";
  std::ofstream output(tempPath.string(), std::ios::binary);
  output << xml;
  output.close();

  auto mujoco = utils::MjcfParser::detail::MujocoModel();
  auto errors = mujoco.read(common::Uri(tempPath.string()), createRetriever());
  std::error_code ec;
  std::filesystem::remove(tempPath, ec);
  ASSERT_TRUE(errors.empty());

  const auto& body = mujoco.getWorldbody().getRootBody(0);
  ASSERT_EQ(body.getNumSites(), 5u);

  const auto& cylinder = body.getSite(0);
  EXPECT_EQ(cylinder.getType(), GeomType::CYLINDER);
  EXPECT_NEAR(cylinder.getCylinderRadius(), 0.2, 1e-12);
  EXPECT_NEAR(cylinder.getCylinderHalfLength(), 1.0, 1e-12);
  EXPECT_EQ(cylinder.getGroup(), 2);
  EXPECT_TRUE(cylinder.getRGBA().isApprox(Eigen::Vector4d(0.1, 0.2, 0.3, 0.4)));

  const auto& capsule = body.getSite(1);
  EXPECT_EQ(capsule.getType(), GeomType::CAPSULE);
  EXPECT_NEAR(capsule.getCapsuleRadius(), 0.1, 1e-12);
  EXPECT_NEAR(capsule.getCapsuleHalfLength(), 0.5, 1e-12);

  const auto& ellipsoid = body.getSite(2);
  EXPECT_EQ(ellipsoid.getType(), GeomType::ELLIPSOID);
  EXPECT_TRUE(ellipsoid.getRelativeTransform().translation().isApprox(
      Eigen::Vector3d(1, 2, 3)));

  const auto& box = body.getSite(3);
  EXPECT_EQ(box.getType(), GeomType::BOX);
  EXPECT_TRUE(box.getRelativeTransform().translation().isApprox(
      Eigen::Vector3d(-1, -2, -3)));

  const auto& eulerSite = body.getSite(4);
  EXPECT_EQ(eulerSite.getType(), GeomType::BOX);
  EXPECT_TRUE(eulerSite.getRelativeTransform().translation().isApprox(
      Eigen::Vector3d(0, 1, 0)));
}

//==============================================================================
TEST(MjcfParserTest, EqualitySensorContactActuatorTendonParsing)
{
  const std::string xml = R"(
<?xml version="1.0" ?>
<mujoco model="constraints">
  <default>
    <geom type="sphere" size="0.1" />
  </default>
  <worldbody>
    <body name="body1">
      <geom type="sphere" size="0.1" />
      <site name="site1" type="sphere" size="0.05" />
    </body>
    <body name="body2" pos="0 0 1">
      <geom type="sphere" size="0.1" />
      <joint name="hinge" type="hinge" axis="0 0 1" />
      <site name="site2" type="sphere" size="0.05" />
    </body>
  </worldbody>
  <equality>
    <weld body1="body1" body2="body2" />
  </equality>
  <contact>
    <exclude body1="body1" body2="body2" />
  </contact>
  <sensor>
    <touch name="touch1" site="site1" />
  </sensor>
  <tendon>
    <spatial name="tendon1">
      <site site="site1" />
      <site site="site2" />
    </spatial>
  </tendon>
  <actuator>
    <motor joint="hinge" />
  </actuator>
</mujoco>
 )";

  const auto tempPath
      = std::filesystem::temp_directory_path() / "dart_mjcf_constraints.xml";
  std::ofstream output(tempPath.string(), std::ios::binary);
  output << xml;
  output.close();

  const auto world
      = utils::MjcfParser::readWorld(common::Uri(tempPath.string()));
  std::error_code ec;
  std::filesystem::remove(tempPath, ec);
  ASSERT_NE(world, nullptr);
  EXPECT_GT(world->getNumSkeletons(), 0u);
  EXPECT_GT(world->getConstraintSolver()->getNumConstraints(), 0u);
}
