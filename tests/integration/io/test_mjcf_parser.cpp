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

#include "dart/utils/All.hpp"
#include "dart/utils/mjcf/detail/mujoco_model.hpp"
#include "dart/utils/mjcf/detail/types.hpp"
#include "dart/utils/mjcf/detail/utils.hpp"

#include <dart/all.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <string>

#include <cstring>

using namespace dart;
using namespace utils::MjcfParser::detail;

namespace {

class MemoryResource final : public common::Resource
{
public:
  explicit MemoryResource(std::string data) : mData(std::move(data)), mOffset(0)
  {
  }

  std::size_t getSize() override
  {
    return mData.size();
  }

  std::size_t tell() override
  {
    return mOffset;
  }

  bool seek(ptrdiff_t offset, SeekType origin) override
  {
    std::size_t base = 0;
    if (origin == SEEKTYPE_CUR) {
      base = mOffset;
    } else if (origin == SEEKTYPE_END) {
      base = mData.size();
    }

    const auto next = static_cast<long long>(base) + offset;
    if (next < 0 || next > static_cast<long long>(mData.size())) {
      return false;
    }

    mOffset = static_cast<std::size_t>(next);
    return true;
  }

  std::size_t read(void* buffer, std::size_t size, std::size_t count) override
  {
    const std::size_t bytes = size * count;
    if (bytes == 0) {
      return 0;
    }

    const std::size_t available = mData.size() - mOffset;
    const std::size_t toCopy = std::min(bytes, available);
    std::memcpy(buffer, mData.data() + mOffset, toCopy);
    mOffset += toCopy;
    return toCopy / size;
  }

private:
  std::string mData;
  std::size_t mOffset;
};

class MemoryResourceRetriever final : public common::ResourceRetriever
{
public:
  void add(const std::string& uri, std::string data)
  {
    mFiles.emplace(uri, std::move(data));
  }

  bool exists(const common::Uri& uri) override
  {
    return mFiles.count(uri.toString()) > 0;
  }

  common::ResourcePtr retrieve(const common::Uri& uri) override
  {
    auto it = mFiles.find(uri.toString());
    if (it == mFiles.end()) {
      return nullptr;
    }
    return std::make_shared<MemoryResource>(it->second);
  }

private:
  std::map<std::string, std::string> mFiles;
};

common::Uri addMemoryFile(
    MemoryResourceRetriever& retriever, std::string_view uri, std::string data)
{
  retriever.add(std::string(uri), std::move(data));
  return common::Uri(std::string(uri));
}

const std::string kMjcfPadding(4096, 'x');

} // namespace

//==============================================================================
common::ResourceRetrieverPtr createRetriever()
{
  return nullptr;
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
  EXPECT_EQ(
      compiler.getAngle(), dart::utils::MjcfParser::detail::Angle::DEGREE);
  EXPECT_EQ(
      compiler.getCoordinate(),
      dart::utils::MjcfParser::detail::Coordinate::LOCAL);

  const auto& option = mujoco.getOption();
  EXPECT_EQ(
      option.getIntegrator(), dart::utils::MjcfParser::detail::Integrator::RK4);
  EXPECT_DOUBLE_EQ(option.getTimestep(), 0.01);

  const auto& worldbody = mujoco.getWorldbody();

  ASSERT_EQ(worldbody.getNumGeoms(), 1);

  ASSERT_EQ(worldbody.getNumRootBodies(), 1);
  const auto& rootBody0 = worldbody.getRootBody(0);
  EXPECT_EQ(rootBody0.getName(), "torso");
  EXPECT_TRUE(rootBody0.getRelativeTransform().translation().isApprox(
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

  const auto bodyNodes
      = dart::utils::MjcfParser::detail::getBodyNodes(*world, "body0");
  EXPECT_EQ(bodyNodes.size(), 1u);

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
  EXPECT_EQ(reacherSkel->getRootJoint()->getType(), "RevoluteJoint");

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
  EXPECT_EQ(strikerSkel->getRootJoint()->getType(), "RevoluteJoint");

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
  EXPECT_EQ(throwerSkel->getRootJoint()->getType(), "RevoluteJoint");

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
  <default class="main">
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
  EXPECT_EQ(
      option.getIntegrator(), dart::utils::MjcfParser::detail::Integrator::RK4);
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
  <default class="main">
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
  <default class="main">
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
  EXPECT_EQ(
      compiler.getCoordinate(),
      dart::utils::MjcfParser::detail::Coordinate::GLOBAL);
  EXPECT_EQ(
      compiler.getAngle(), dart::utils::MjcfParser::detail::Angle::RADIAN);
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
  <default class="main">
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
  <default class="main">
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
  <default class="main">
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
  <default class="main">
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
  <default class="main">
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
  <default class="main">
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
  <default class="main">
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
  EXPECT_EQ(hingeBody->getParentJoint()->getType(), "RevoluteJoint");

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

//==============================================================================
TEST(MjcfParserTest, SiteBoxFromToSetsSize)
{
  const std::string xml = R"(
<?xml version="1.0" ?>
<mujoco model="site_box_fromto">
  <default>
    <geom type="sphere" size="0.1" />
  </default>
  <worldbody>
    <body name="root">
      <geom type="sphere" size="0.1" />
      <site name="box_site" type="box" size="0.1 0.2 0.3"
            fromto="0 0 0 0 0 2" />
    </body>
  </worldbody>
</mujoco>
  )";

  const auto tempPath
      = std::filesystem::temp_directory_path() / "dart_mjcf_site_box.xml";
  std::ofstream output(tempPath.string(), std::ios::binary);
  output << xml;
  output.close();

  auto mujoco = utils::MjcfParser::detail::MujocoModel();
  auto errors = mujoco.read(common::Uri(tempPath.string()), createRetriever());
  std::error_code ec;
  std::filesystem::remove(tempPath, ec);
  ASSERT_TRUE(errors.empty());

  const auto& body = mujoco.getWorldbody().getRootBody(0);
  ASSERT_EQ(body.getNumSites(), 1u);
  const auto& site = body.getSite(0);
  EXPECT_EQ(site.getType(), GeomType::BOX);
  EXPECT_NEAR(site.getBoxHalfSize().z(), 1.0, 1e-12);
}

//==============================================================================
TEST(MjcfParserTest, InertialDiagInertiaParsing)
{
  const std::string xml = R"(
<?xml version="1.0" ?>
<mujoco model="inertial_diag">
  <default>
    <geom type="sphere" size="0.1" />
  </default>
  <worldbody>
    <body name="root">
      <geom type="sphere" size="0.1" />
      <inertial pos="1 2 3" mass="2.5" diaginertia="0.1 0.2 0.3"
                quat="1 0 0 0" />
    </body>
  </worldbody>
</mujoco>
  )";

  const auto tempPath
      = std::filesystem::temp_directory_path() / "dart_mjcf_inertial_diag.xml";
  std::ofstream output(tempPath.string(), std::ios::binary);
  output << xml;
  output.close();

  auto mujoco = utils::MjcfParser::detail::MujocoModel();
  auto errors = mujoco.read(common::Uri(tempPath.string()), createRetriever());
  std::error_code ec;
  std::filesystem::remove(tempPath, ec);
  ASSERT_TRUE(errors.empty());

  const auto& body = mujoco.getWorldbody().getRootBody(0);
  const auto& inertial = body.getInertial();
  EXPECT_DOUBLE_EQ(inertial.getMass(), 2.5);
  EXPECT_TRUE(
      inertial.getDiagInertia().isApprox(Eigen::Vector3d(0.1, 0.2, 0.3)));
  EXPECT_TRUE(inertial.getRelativeTransform().translation().isApprox(
      Eigen::Vector3d(1.0, 2.0, 3.0)));
}

//==============================================================================
TEST(MjcfParserTest, InertialFullInertiaParsing)
{
  const std::string xml = R"(
<?xml version="1.0" ?>
<mujoco model="inertial_full">
  <default>
    <geom type="sphere" size="0.1" />
  </default>
  <worldbody>
    <body name="root">
      <geom type="sphere" size="0.1" />
      <inertial pos="0 0 0" mass="1.0" fullinertia="1 2 3 0.1 0.2 0.3"
                axisangle="1 0 0 1.57079632679" />
    </body>
  </worldbody>
</mujoco>
  )";

  const auto tempPath
      = std::filesystem::temp_directory_path() / "dart_mjcf_inertial_full.xml";
  std::ofstream output(tempPath.string(), std::ios::binary);
  output << xml;
  output.close();

  auto mujoco = utils::MjcfParser::detail::MujocoModel();
  auto errors = mujoco.read(common::Uri(tempPath.string()), createRetriever());
  std::error_code ec;
  std::filesystem::remove(tempPath, ec);
  ASSERT_TRUE(errors.empty());

  const auto& body = mujoco.getWorldbody().getRootBody(0);
  const auto& inertial = body.getInertial();
  EXPECT_DOUBLE_EQ(inertial.getMass(), 1.0);
  EXPECT_TRUE(
      inertial.getDiagInertia().isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));
  EXPECT_TRUE(
      inertial.getOffDiagInertia().isApprox(Eigen::Vector3d(0.1, 0.2, 0.3)));
}

//==============================================================================
TEST(MjcfParserTest, ComplexMjcfCoversDetails)
{
  const auto tempDir = std::filesystem::temp_directory_path();
  const auto meshPath = tempDir / "dart_mjcf_unit_mesh.stl";
  const auto xmlPath = tempDir / "dart_mjcf_complex.xml";

  {
    std::ofstream meshOutput(meshPath.string(), std::ios::binary);
    meshOutput << "solid unit\n"
                  "facet normal 0 0 1\n"
                  "  outer loop\n"
                  "    vertex 0 0 0\n"
                  "    vertex 1 0 0\n"
                  "    vertex 0 1 0\n"
                  "  endloop\n"
                  "endfacet\n"
                  "endsolid unit\n";
  }

  const std::string meshFileName = meshPath.filename().string();
  std::string meshDirStr = tempDir.string();
  std::replace(meshDirStr.begin(), meshDirStr.end(), '\\', '/');
  std::string xml = R"(
<?xml version="1.0" ?>
<mujoco model="complex_mjcf">
  <compiler angle="degree" coordinate="global" eulerseq="zyx" meshdir="${MESH_DIR}" />
  <option gravity="0 0 -9.81" timestep="0.002" integrator="Euler" />
  <size nuser_body="2" />
  <default>
    <geom type="sphere" size="0.1" />
    <joint damping="0.1" springref="0.2" range="-1 1" />
    <default class="classA">
      <geom type="box" size="0.2 0.3 0.4" rgba="0.2 0.3 0.4 1"
            friction="0.1 0.2 0.3" contype="1" conaffinity="2" condim="3"
            group="4" priority="7" density="200" margin="0.01" gap="0.02" />
      <joint type="hinge" axis="0 1 0" damping="0.4" springref="0.5" />
    </default>
  </default>
  <asset>
    <mesh name="test_mesh" file="${MESH_FILE}" scale="1 2 3" />
  </asset>
  <worldbody>
    <geom name="ground_plane" type="plane" size="1 1 0.1" rgba="0.1 0.2 0.3 1" />
    <body name="root" pos="0 0 0" user="1 2" zaxis="0 0 1">
      <geom type="sphere" size="0.1" />
      <geom name="box_fromto" type="box" size="0.1 0.2 0.3" fromto="0 0 0 0 0 1" />
      <site name="body_site" type="box" size="0.1 0.2 0.3" fromto="0 0 0 0 0 2" />
      <joint name="root_free" type="free" />
      <body name="child1" pos="0 0 1" quat="1 0 0 0">
        <geom class="classA" name="child_geom" pos="0.1 0.2 0.3" euler="0 0 45" />
        <joint name="hinge_joint" type="hinge" axis="0 1 0" range="-0.5 0.5" damping="0.2"
               springref="0.1" />
      </body>
      <body name="child2" pos="0 0 2" xyaxes="1 0 0 0 1 0">
        <geom name="mesh_geom" type="mesh" mesh="test_mesh" />
        <inertial pos="0 0 0" mass="1.0" diaginertia="0.1 0.2 0.3" />
        <joint name="slide_x" type="slide" axis="1 0 0" range="-1 1" />
        <joint name="slide_y" type="slide" axis="0 1 0" range="-1 1" />
        <joint name="slide_z" type="slide" axis="0 0 1" range="-1 1" />
      </body>
      <body name="child3" pos="0 0 3">
        <geom name="capsule_geom" type="capsule" size="0.1" fromto="0 0 0 0 0 2" mass="1.5" />
        <joint name="ball_joint" type="ball" damping="0.3" />
      </body>
      <body name="mocap_body" pos="1 0 0" mocap="true">
        <geom type="sphere" size="0.1" />
      </body>
    </body>
  </worldbody>
  <equality>
    <weld name="weld1" body1="root" body2="child1" active="false"
          solimp="0.95 0.9 0.003 0.6 1" solref="0.01 2"
          relpose="0 0 0 1 0 0 0" />
  </equality>
</mujoco>
)";

  const std::string meshDirToken = "${MESH_DIR}";
  const auto meshDirTokenPos = xml.find(meshDirToken);
  ASSERT_NE(meshDirTokenPos, std::string::npos);
  xml.replace(meshDirTokenPos, meshDirToken.size(), meshDirStr);

  const std::string meshToken = "${MESH_FILE}";
  const auto meshTokenPos = xml.find(meshToken);
  ASSERT_NE(meshTokenPos, std::string::npos);
  xml.replace(meshTokenPos, meshToken.size(), meshFileName);

  std::ofstream output(xmlPath.string(), std::ios::binary);
  output << xml;
  output.close();

  const auto options = utils::MjcfParser::Options();
  auto world = utils::MjcfParser::readWorld(
      common::Uri::createFromPath(xmlPath.string()), options);

  std::error_code ec;
  std::filesystem::remove(xmlPath, ec);
  std::filesystem::remove(meshPath, ec);

  ASSERT_NE(world, nullptr);
  EXPECT_TRUE(world->hasSkeleton("root"));
  EXPECT_TRUE(
      world->hasSkeleton(options.mGeomSkeletonNamePrefix + "ground_plane"));

  auto skel = world->getSkeleton("root");
  ASSERT_NE(skel, nullptr);
  EXPECT_EQ(
      skel->getRootJoint()->getType(), dynamics::FreeJoint::getStaticType());

  const auto* child2 = skel->getBodyNode("child2");
  ASSERT_NE(child2, nullptr);
  EXPECT_EQ(child2->getParentJoint()->getType(), "TranslationalJoint");

  const auto* child3 = skel->getBodyNode("child3");
  ASSERT_NE(child3, nullptr);
  EXPECT_EQ(
      child3->getParentJoint()->getType(),
      dynamics::BallJoint::getStaticType());

  const auto* mocapBody = skel->getBodyNode("mocap_body");
  ASSERT_NE(mocapBody, nullptr);
  auto* mocapShapeNode = mocapBody->getShapeNode(0);
  ASSERT_NE(mocapShapeNode, nullptr);
  EXPECT_EQ(mocapShapeNode->getCollisionAspect(), nullptr);
  EXPECT_EQ(mocapShapeNode->getDynamicsAspect(), nullptr);

  auto* meshShapeNode = child2->getShapeNode(0);
  ASSERT_NE(meshShapeNode, nullptr);
  EXPECT_NE(meshShapeNode->getShape(), nullptr);

  auto groundSkel
      = world->getSkeleton(options.mGeomSkeletonNamePrefix + "ground_plane");
  ASSERT_NE(groundSkel, nullptr);
  EXPECT_FALSE(groundSkel->isMobile());

  EXPECT_GT(world->getConstraintSolver()->getNumConstraints(), 0u);
}

//==============================================================================
TEST(MjcfParserTest, CustomModelParsesBodiesGeomsAndActuators)
{
  const auto tempDir = std::filesystem::temp_directory_path();
  const auto meshPath = tempDir / "dart_mjcf_custom_mesh.stl";
  const auto xmlPath = tempDir / "dart_mjcf_custom.xml";

  {
    std::ofstream meshOutput(meshPath.string(), std::ios::binary);
    meshOutput << "solid unit\n"
                  "facet normal 0 0 1\n"
                  "  outer loop\n"
                  "    vertex 0 0 0\n"
                  "    vertex 1 0 0\n"
                  "    vertex 0 1 0\n"
                  "  endloop\n"
                  "endfacet\n"
                  "endsolid unit\n";
  }

  const std::string meshFileName = meshPath.filename().string();
  std::string meshDirStr = tempDir.string();
  std::replace(meshDirStr.begin(), meshDirStr.end(), '\\', '/');
  std::string xml = R"(
<?xml version="1.0" ?>
<mujoco model="custom_parser">
  <compiler angle="radian" coordinate="global" eulerseq="xyz" meshdir="${MESH_DIR}" />
  <option timestep="0.003" gravity="0 0 -9.81" integrator="Euler" />
  <size nuserdata="2" nkey="1" />
  <default>
    <geom type="sphere" size="0.1" />
  </default>
  <asset>
    <mesh name="unit_mesh" file="${MESH_FILE}" scale="1 1 1" />
  </asset>
  <worldbody>
    <body name="root" pos="0 0 0">
      <geom name="root_box" type="box" size="0.2 0.2 0.2" />
      <joint name="root_hinge" type="hinge" axis="0 0 1" range="-1 1" />
      <site name="root_site" type="box" size="0.1 0.1 0.1" pos="0 0 0.2" />
      <body name="child" pos="0 0 1">
        <geom name="child_capsule" type="capsule" size="0.05" fromto="0 0 0 0 0 1" />
        <joint name="child_slide" type="slide" axis="1 0 0" range="-0.5 0.5" />
        <body name="grand" pos="0 0 1">
          <geom name="grand_mesh" type="mesh" mesh="unit_mesh" />
          <inertial pos="0 0 0" mass="1.0" diaginertia="0.1 0.1 0.1" />
          <joint name="grand_ball" type="ball" />
        </body>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor joint="root_hinge" gear="2" />
    <position joint="child_slide" kp="10" />
    <velocity joint="root_hinge" kv="0.1" />
  </actuator>
</mujoco>
  )";

  const std::string meshDirToken = "${MESH_DIR}";
  const auto meshDirTokenPos = xml.find(meshDirToken);
  ASSERT_NE(meshDirTokenPos, std::string::npos);
  xml.replace(meshDirTokenPos, meshDirToken.size(), meshDirStr);

  const std::string meshToken = "${MESH_FILE}";
  const auto meshTokenPos = xml.find(meshToken);
  ASSERT_NE(meshTokenPos, std::string::npos);
  xml.replace(meshTokenPos, meshToken.size(), meshFileName);

  std::ofstream output(xmlPath.string(), std::ios::binary);
  output << xml;
  output.close();

  const auto world = utils::MjcfParser::readWorld(
      common::Uri::createFromPath(xmlPath.string()));
  std::error_code ec;
  std::filesystem::remove(xmlPath, ec);
  std::filesystem::remove(meshPath, ec);

  ASSERT_NE(world, nullptr);
  const auto skel = world->getSkeleton("root");
  ASSERT_NE(skel, nullptr);
  auto* rootBody = skel->getBodyNode("root");
  ASSERT_NE(rootBody, nullptr);
  EXPECT_TRUE(
      rootBody->getTransform().translation().isApprox(Eigen::Vector3d::Zero()));
  EXPECT_NE(skel->getJoint("root_hinge"), nullptr);
  EXPECT_NE(skel->getJoint("child_slide"), nullptr);
  EXPECT_NE(skel->getJoint("grand_ball"), nullptr);
}

//==============================================================================
TEST(MjcfParserTest, InlineWorldCoversDefaultsWorldbodyAndActuators)
{
  const auto tempDir = std::filesystem::temp_directory_path();
  const auto meshPath = tempDir / "dart_mjcf_coverage_mesh.obj";
  const auto xmlPath = tempDir / "dart_mjcf_coverage.xml";

  {
    std::ofstream meshOutput(meshPath.string(), std::ios::binary);
    meshOutput << "o Mesh\n"
                  "v 0 0 0\n"
                  "v 1 0 0\n"
                  "v 0 1 0\n"
                  "f 1 2 3\n";
  }

  const std::string xml = R"(
<?xml version="1.0" ?>
<mujoco model="coverage">
  <compiler meshdir="." />
  <default>
    <geom type="sphere" size="0.1" />
    <joint type="hinge" damping="0.2" limited="true" range="-0.5 0.5" />
    <site type="box" size="0.05 0.05 0.05" />
    <motor ctrllimited="true" ctrlrange="-1 1" />
  </default>
  <option timestep="0.01" apirate="60" impratio="2"
          gravity="0 0 -9.81" wind="0.1 0.2 0.3" magnetic="0 0 0"
          density="1000" viscosity="0.2" integrator="Euler"
          collision="all" cone="elliptic" jacobian="dense"
          solver="PGS" iterations="10" tolerance="1e-4" />
  <asset>
    <mesh name="coverage_mesh" file="dart_mjcf_coverage_mesh.obj" />
  </asset>
  <worldbody>
    <light name="light0" pos="0 0 1" />
    <camera name="cam0" pos="0 0 2" />
    <geom name="ground" type="plane" size="1 1 0.1" />
    <body name="root">
      <geom name="root_box" type="box" size="0.1 0.2 0.3" />
      <site name="sphere_site" type="sphere" size="0.15" />
      <body name="capsule_body">
        <geom name="capsule_geom" type="capsule" size="0.05" fromto="0 0 0 0 0 0.6" />
        <site name="box_site" type="box" size="0.1 0.1 0.1" pos="0 0 0.2" />
        <body name="cylinder_body">
          <geom name="cylinder_geom" type="cylinder" size="0.05" fromto="0 0 0 0 0 0.4" />
          <body name="mesh_body">
            <geom name="mesh_geom" type="mesh" mesh="coverage_mesh" />
            <inertial pos="0 0 0" mass="1" diaginertia="0.1 0.1 0.1" />
            <site name="capsule_site" type="capsule" size="0.04" fromto="0 0 0 0 0 0.4" />
          </body>
        </body>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor name="motor0" joint="root_joint" />
    <position name="position0" joint="root_joint" />
    <velocity name="velocity0" joint="root_joint" />
  </actuator>
</mujoco>
  )";

  std::ofstream output(xmlPath.string(), std::ios::binary);
  output << xml;
  output.close();

  auto mujoco = utils::MjcfParser::detail::MujocoModel();
  auto errors = mujoco.read(common::Uri(xmlPath.string()), createRetriever());
  std::error_code ec;
  std::filesystem::remove(xmlPath, ec);
  std::filesystem::remove(meshPath, ec);

  ASSERT_TRUE(errors.empty());
  const auto& worldbody = mujoco.getWorldbody();
  ASSERT_EQ(worldbody.getNumRootBodies(), 1u);
  const auto& rootBody = worldbody.getRootBody(0);
  ASSERT_GE(rootBody.getNumGeoms(), 1u);
  ASSERT_EQ(rootBody.getNumSites(), 1u);
  const auto& sphereSite = rootBody.getSite(0);
  EXPECT_EQ(sphereSite.getType(), GeomType::SPHERE);
}

//==============================================================================
TEST(MjcfParserTest, DefaultsNestedClassInheritance)
{
  const std::string xml = R"(
<?xml version="1.0" ?>
<mujoco model="nested_defaults">
  <default>
    <geom type="sphere" size="0.1" />
    <default class="outer">
      <geom type="box" size="0.2 0.3 0.4" rgba="0.1 0.2 0.3 0.4"
            friction="0.2 0.3 0.4" />
      <default class="inner">
        <geom size="0.5 0.6 0.7" rgba="0.9 0.8 0.7 0.6" />
      </default>
    </default>
  </default>
  <worldbody>
    <body name="root">
      <geom class="inner" />
    </body>
  </worldbody>
</mujoco>
  )";

  auto retriever = std::make_shared<MemoryResourceRetriever>();
  const auto uri
      = addMemoryFile(*retriever, "memory://mjcf_nested_defaults.xml", xml);

  auto mujoco = utils::MjcfParser::detail::MujocoModel();
  auto errors = mujoco.read(uri, retriever);
  ASSERT_TRUE(errors.empty());

  const auto& body = mujoco.getWorldbody().getRootBody(0);
  ASSERT_EQ(body.getNumGeoms(), 1u);
  const auto& geom = body.getGeom(0);
  EXPECT_EQ(geom.getType(), GeomType::BOX);
  EXPECT_TRUE(geom.getBoxHalfSize().isApprox(Eigen::Vector3d(0.5, 0.6, 0.7)));
  EXPECT_TRUE(geom.getRGBA().isApprox(Eigen::Vector4d(0.9, 0.8, 0.7, 0.6)));
  EXPECT_TRUE(geom.getFriction().isApprox(Eigen::Vector3d(0.2, 0.3, 0.4)));
}

//==============================================================================
TEST(MjcfParserTest, DefaultNestedMissingClassReportsError)
{
  const std::string xml = R"(
<?xml version="1.0" ?>
<mujoco model="missing_default_class">
  <default>
    <geom type="sphere" size="0.1" />
    <default>
      <geom type="box" size="0.2 0.3 0.4" />
    </default>
  </default>
  <worldbody>
    <body name="root">
      <geom type="sphere" size="0.1" />
    </body>
  </worldbody>
</mujoco>
  )";

  auto retriever = std::make_shared<MemoryResourceRetriever>();
  const auto uri = addMemoryFile(
      *retriever, "memory://mjcf_missing_default_class.xml", xml);

  auto mujoco = utils::MjcfParser::detail::MujocoModel();
  auto errors = mujoco.read(uri, retriever);
  ASSERT_FALSE(errors.empty());
  bool hasMissing = false;
  for (const auto& error : errors) {
    if (error.getCode() == ErrorCode::ATTRIBUTE_MISSING) {
      hasMissing = true;
      break;
    }
  }
  EXPECT_TRUE(hasMissing);
}

//==============================================================================
TEST(MjcfParserTest, SiteShapeNodesCoverPrimitiveTypes)
{
  const std::string xml = R"(
<?xml version="1.0" ?>
<mujoco model="site_shapes">
  <default>
    <geom type="sphere" size="0.1" />
  </default>
  <worldbody>
    <body name="root">
      <geom type="sphere" size="0.1" />
      <site name="sphere_site" type="sphere" size="0.2" />
      <site name="capsule_site" type="capsule" size="0.1"
            fromto="0 0 0 0 0 1" />
      <site name="cylinder_site" type="cylinder" size="0.15"
            fromto="0 0 0 0 0 2" />
      <site name="ellipsoid_site" type="ellipsoid" size="0.2 0.3 0.4"
            fromto="0 0 0 0 0 1" />
      <site name="box_site" type="box" size="0.1 0.2 0.3"
            fromto="0 0 0 0 0 2" />
    </body>
  </worldbody>
</mujoco>
  )";

  auto retriever = std::make_shared<MemoryResourceRetriever>();
  const auto uri
      = addMemoryFile(*retriever, "memory://mjcf_site_shapes.xml", xml);

  const auto options = utils::MjcfParser::Options(retriever);
  auto world = utils::MjcfParser::readWorld(uri, options);
  ASSERT_NE(world, nullptr);

  auto skel = world->getSkeleton("root");
  ASSERT_NE(skel, nullptr);
  auto* body = skel->getBodyNode("root");
  ASSERT_NE(body, nullptr);

  const auto findShapeNode = [body](const std::string& name) {
    dynamics::ShapeNode* found = nullptr;
    body->eachShapeNodeWith<dynamics::VisualAspect>(
        [&found, &name](dynamics::ShapeNode* shapeNode) -> bool {
          if (shapeNode && shapeNode->getName() == name) {
            found = shapeNode;
            return false;
          }
          return true;
        });
    return found;
  };

  const auto* sphereNode = findShapeNode("site:sphere_site");
  const auto* capsuleNode = findShapeNode("site:capsule_site");
  const auto* cylinderNode = findShapeNode("site:cylinder_site");
  const auto* ellipsoidNode = findShapeNode("site:ellipsoid_site");
  const auto* boxNode = findShapeNode("site:box_site");
  ASSERT_NE(sphereNode, nullptr);
  ASSERT_NE(capsuleNode, nullptr);
  ASSERT_NE(cylinderNode, nullptr);
  ASSERT_NE(ellipsoidNode, nullptr);
  ASSERT_NE(boxNode, nullptr);

  EXPECT_NE(
      dynamic_cast<const dynamics::SphereShape*>(sphereNode->getShape().get()),
      nullptr);
  EXPECT_NE(
      dynamic_cast<const dynamics::CapsuleShape*>(
          capsuleNode->getShape().get()),
      nullptr);
  EXPECT_NE(
      dynamic_cast<const dynamics::CylinderShape*>(
          cylinderNode->getShape().get()),
      nullptr);
  EXPECT_NE(
      dynamic_cast<const dynamics::EllipsoidShape*>(
          ellipsoidNode->getShape().get()),
      nullptr);
  EXPECT_NE(
      dynamic_cast<const dynamics::BoxShape*>(boxNode->getShape().get()),
      nullptr);
}

//==============================================================================
TEST(MjcfParserTest, GeomClassMissingDefaultReportsError)
{
  const std::string xml = R"(
<?xml version="1.0" ?>
<mujoco model="geom_missing_default">
  <default>
    <geom type="sphere" size="0.1" />
  </default>
  <worldbody>
    <body name="root">
      <geom class="missing_class" />
    </body>
  </worldbody>
</mujoco>
  )";

  auto retriever = std::make_shared<MemoryResourceRetriever>();
  const auto uri
      = addMemoryFile(*retriever, "memory://mjcf_missing_geom_class.xml", xml);

  auto mujoco = utils::MjcfParser::detail::MujocoModel();
  auto errors = mujoco.read(uri, retriever);
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
TEST(MjcfParserTest, JointLimitedAttribute)
{
  // limited="true" with autolimits=false
  {
    const std::string xml = std::string(R"(
 <?xml version="1.0" ?>
 <mujoco model="joint_limited_true">
   <compiler autolimits="false"/>
   <default>
     <geom type="sphere" size="0.1" />
   </default>
   <!-- )") + kMjcfPadding + R"( -->
   <worldbody>
     <body name="root">
       <geom type="sphere" size="0.1" />
       <joint type="hinge" limited="true" range="-1 1" />
     </body>
   </worldbody>
 </mujoco>
)";
    const auto tempPath
        = std::filesystem::temp_directory_path() / "dart_mjcf_limited_true.xml";
    std::ofstream output(tempPath.string(), std::ios::binary);
    output << xml;
    output.close();

    auto world = utils::MjcfParser::readWorld(common::Uri(tempPath.string()));
    std::error_code ec;
    std::filesystem::remove(tempPath, ec);
    ASSERT_NE(world, nullptr);
    auto* joint = world->getSkeleton(0)->getJoint(0);
    EXPECT_TRUE(joint->areLimitsEnforced());
    EXPECT_DOUBLE_EQ(joint->getPositionLowerLimit(0), -1.0);
    EXPECT_DOUBLE_EQ(joint->getPositionUpperLimit(0), 1.0);
  }

  // limited="false" explicitly
  {
    const std::string xml = std::string(R"(
 <?xml version="1.0" ?>
 <mujoco model="joint_limited_false">
   <compiler autolimits="false"/>
   <default>
     <geom type="sphere" size="0.1" />
   </default>
   <!-- )") + kMjcfPadding + R"( -->
   <worldbody>
     <body name="root">
       <geom type="sphere" size="0.1" />
       <joint type="hinge" limited="false" range="-1 1" />
     </body>
   </worldbody>
 </mujoco>
)";
    const auto tempPath = std::filesystem::temp_directory_path()
                          / "dart_mjcf_limited_false.xml";
    std::ofstream output(tempPath.string(), std::ios::binary);
    output << xml;
    output.close();

    auto world = utils::MjcfParser::readWorld(common::Uri(tempPath.string()));
    std::error_code ec;
    std::filesystem::remove(tempPath, ec);
    ASSERT_NE(world, nullptr);
    auto* joint = world->getSkeleton(0)->getJoint(0);
    EXPECT_FALSE(joint->areLimitsEnforced());
  }

  // limited absent with autolimits=false
  {
    const std::string xml = std::string(R"(
 <?xml version="1.0" ?>
 <mujoco model="joint_limited_absent">
   <compiler autolimits="false"/>
   <default>
     <geom type="sphere" size="0.1" />
   </default>
   <!-- )") + kMjcfPadding + R"( -->
   <worldbody>
     <body name="root">
       <geom type="sphere" size="0.1" />
       <joint type="hinge" range="-1 1" />
     </body>
   </worldbody>
 </mujoco>
)";
    const auto tempPath = std::filesystem::temp_directory_path()
                          / "dart_mjcf_limited_absent.xml";
    std::ofstream output(tempPath.string(), std::ios::binary);
    output << xml;
    output.close();

    auto world = utils::MjcfParser::readWorld(common::Uri(tempPath.string()));
    std::error_code ec;
    std::filesystem::remove(tempPath, ec);
    ASSERT_NE(world, nullptr);
    auto* joint = world->getSkeleton(0)->getJoint(0);
    EXPECT_FALSE(joint->areLimitsEnforced());
  }
}

//==============================================================================
TEST(MjcfParserTest, AutoLimitsDefault)
{
  // autolimits=true (default) + range present
  {
    const std::string xml = std::string(R"(
 <?xml version="1.0" ?>
 <mujoco model="autolimits_default_range">
   <default>
     <geom type="sphere" size="0.1" />
   </default>
   <!-- )") + kMjcfPadding + R"( -->
   <worldbody>
     <body name="root">
       <geom type="sphere" size="0.1" />
       <joint type="hinge" range="-1.57 1.57" />
     </body>
   </worldbody>
 </mujoco>
)";
    const auto tempPath = std::filesystem::temp_directory_path()
                          / "dart_mjcf_autolimits_default_range.xml";
    std::ofstream output(tempPath.string(), std::ios::binary);
    output << xml;
    output.close();

    auto world = utils::MjcfParser::readWorld(common::Uri(tempPath.string()));
    std::error_code ec;
    std::filesystem::remove(tempPath, ec);
    ASSERT_NE(world, nullptr);
    auto* joint = world->getSkeleton(0)->getJoint(0);
    EXPECT_TRUE(joint->areLimitsEnforced());
  }

  // autolimits=true + no range
  {
    const std::string xml = std::string(R"(
 <?xml version="1.0" ?>
 <mujoco model="autolimits_default_norange">
   <default>
     <geom type="sphere" size="0.1" />
   </default>
   <!-- )") + kMjcfPadding + R"( -->
   <worldbody>
     <body name="root">
       <geom type="sphere" size="0.1" />
       <joint type="hinge" />
     </body>
   </worldbody>
 </mujoco>
)";
    const auto tempPath = std::filesystem::temp_directory_path()
                          / "dart_mjcf_autolimits_default_norange.xml";
    std::ofstream output(tempPath.string(), std::ios::binary);
    output << xml;
    output.close();

    auto world = utils::MjcfParser::readWorld(common::Uri(tempPath.string()));
    std::error_code ec;
    std::filesystem::remove(tempPath, ec);
    ASSERT_NE(world, nullptr);
    auto* joint = world->getSkeleton(0)->getJoint(0);
    EXPECT_FALSE(joint->areLimitsEnforced());
  }

  // autolimits=true + limited="false" explicitly + range
  {
    const std::string xml = std::string(R"(
 <?xml version="1.0" ?>
 <mujoco model="autolimits_default_limited_false">
   <default>
     <geom type="sphere" size="0.1" />
   </default>
   <!-- )") + kMjcfPadding + R"( -->
   <worldbody>
     <body name="root">
       <geom type="sphere" size="0.1" />
       <joint type="hinge" limited="false" range="-1 1" />
     </body>
   </worldbody>
 </mujoco>
)";
    const auto tempPath = std::filesystem::temp_directory_path()
                          / "dart_mjcf_autolimits_default_limited_false.xml";
    std::ofstream output(tempPath.string(), std::ios::binary);
    output << xml;
    output.close();

    auto world = utils::MjcfParser::readWorld(common::Uri(tempPath.string()));
    std::error_code ec;
    std::filesystem::remove(tempPath, ec);
    ASSERT_NE(world, nullptr);
    auto* joint = world->getSkeleton(0)->getJoint(0);
    EXPECT_FALSE(joint->areLimitsEnforced());
  }

  // autolimits=false (legacy) + range present but limited absent
  {
    const std::string xml = std::string(R"(
 <?xml version="1.0" ?>
 <mujoco model="autolimits_legacy_range">
   <compiler autolimits="false"/>
   <default>
     <geom type="sphere" size="0.1" />
   </default>
   <!-- )") + kMjcfPadding + R"( -->
   <worldbody>
     <body name="root">
       <geom type="sphere" size="0.1" />
       <joint type="hinge" range="-1 1" />
     </body>
   </worldbody>
 </mujoco>
)";
    const auto tempPath = std::filesystem::temp_directory_path()
                          / "dart_mjcf_autolimits_legacy_range.xml";
    std::ofstream output(tempPath.string(), std::ios::binary);
    output << xml;
    output.close();

    auto world = utils::MjcfParser::readWorld(common::Uri(tempPath.string()));
    std::error_code ec;
    std::filesystem::remove(tempPath, ec);
    ASSERT_NE(world, nullptr);
    auto* joint = world->getSkeleton(0)->getJoint(0);
    EXPECT_FALSE(joint->areLimitsEnforced());
  }
}

//==============================================================================
TEST(MjcfParserTest, FreeJoint)
{
  const std::string xml = R"(
 <?xml version="1.0" ?>
 <mujoco model="freejoint">
   <default>
     <geom type="sphere" size="0.1" />
   </default>
   <worldbody>
     <body name="root">
       <freejoint name="root" />
       <geom type="sphere" size="0.1" />
     </body>
   </worldbody>
 </mujoco>
)";
  const auto tempPath
      = std::filesystem::temp_directory_path() / "dart_mjcf_freejoint.xml";
  std::ofstream output(tempPath.string(), std::ios::binary);
  output << xml;
  output.close();

  auto world = utils::MjcfParser::readWorld(common::Uri(tempPath.string()));
  std::error_code ec;
  std::filesystem::remove(tempPath, ec);
  ASSERT_NE(world, nullptr);
  auto* skel = world->getSkeleton(0).get();
  ASSERT_NE(skel, nullptr);
  EXPECT_EQ(skel->getNumJoints(), 1u);
  auto* joint = skel->getJoint(0);
  EXPECT_NE(dynamic_cast<dynamics::FreeJoint*>(joint), nullptr);
}

//==============================================================================
TEST(MjcfParserTest, JointStiffnessAndFriction)
{
  const std::string xml = std::string(R"(
 <?xml version="1.0" ?>
 <mujoco model="joint_stiffness_friction">
   <compiler autolimits="false"/>
   <default>
     <geom type="sphere" size="0.1" />
   </default>
   <!-- )") + kMjcfPadding + R"( -->
   <worldbody>
     <body name="root">
       <geom type="sphere" size="0.1" />
       <joint type="hinge" stiffness="100" frictionloss="0.5" springref="0.3" damping="2.0" />
     </body>
   </worldbody>
 </mujoco>
)";
  const auto tempPath = std::filesystem::temp_directory_path()
                        / "dart_mjcf_joint_stiffness_friction.xml";
  std::ofstream output(tempPath.string(), std::ios::binary);
  output << xml;
  output.close();

  auto world = utils::MjcfParser::readWorld(common::Uri(tempPath.string()));
  std::error_code ec;
  std::filesystem::remove(tempPath, ec);
  ASSERT_NE(world, nullptr);
  auto* joint = world->getSkeleton(0)->getJoint(0);
  EXPECT_DOUBLE_EQ(joint->getSpringStiffness(0), 100.0);
  EXPECT_DOUBLE_EQ(joint->getRestPosition(0), 0.3);
  EXPECT_DOUBLE_EQ(joint->getDampingCoefficient(0), 2.0);
  EXPECT_DOUBLE_EQ(joint->getCoulombFriction(0), 0.5);
}

//==============================================================================
TEST(MjcfParserTest, OptionTimestepGravity)
{
  const std::string xml = std::string(R"(
 <?xml version="1.0" ?>
 <mujoco model="option_timestep_gravity">
   <option timestep="0.005" gravity="0 0 -10"/>
   <default>
     <geom type="sphere" size="0.1" />
   </default>
   <!-- )") + kMjcfPadding + R"( -->
   <worldbody>
     <body name="root">
       <geom type="sphere" size="0.1" />
       <joint type="hinge" />
     </body>
   </worldbody>
 </mujoco>
)";
  const auto tempPath = std::filesystem::temp_directory_path()
                        / "dart_mjcf_option_timestep_gravity.xml";
  std::ofstream output(tempPath.string(), std::ios::binary);
  output << xml;
  output.close();

  auto world = utils::MjcfParser::readWorld(common::Uri(tempPath.string()));
  std::error_code ec;
  std::filesystem::remove(tempPath, ec);
  ASSERT_NE(world, nullptr);
  EXPECT_DOUBLE_EQ(world->getTimeStep(), 0.005);
  EXPECT_TRUE(world->getGravity().isApprox(Eigen::Vector3d(0, 0, -10)));
}

//==============================================================================
TEST(MjcfParserTest, ActuatorMotorSetsForceType)
{
  const std::string xml = R"(
<?xml version="1.0" ?>
<mujoco model="actuator_motor">
  <default>
    <geom type="sphere" size="0.1" />
  </default>
  <worldbody>
    <body name="root">
      <geom type="sphere" size="0.1" />
      <joint name="hinge1" type="hinge" axis="0 0 1" />
    </body>
  </worldbody>
  <actuator>
    <motor joint="hinge1" gear="2" forcelimited="true" forcerange="-10 10" />
  </actuator>
</mujoco>
)";
  const auto tempPath
      = std::filesystem::temp_directory_path() / "dart_mjcf_actuator_motor.xml";
  std::ofstream output(tempPath.string(), std::ios::binary);
  output << xml;
  output.close();

  auto world = utils::MjcfParser::readWorld(common::Uri(tempPath.string()));
  std::error_code ec;
  std::filesystem::remove(tempPath, ec);
  ASSERT_NE(world, nullptr);

  auto* joint = world->getSkeleton(0)->getJoint("hinge1");
  ASSERT_NE(joint, nullptr);
  EXPECT_EQ(joint->getActuatorType(), dynamics::Joint::FORCE);
  // Force limits scaled by gear=2
  EXPECT_DOUBLE_EQ(joint->getForceLowerLimit(0), -20.0);
  EXPECT_DOUBLE_EQ(joint->getForceUpperLimit(0), 20.0);
}

//==============================================================================
TEST(MjcfParserTest, ActuatorAutoLimitedResolvesFromRange)
{
  const std::string xml = R"(
<?xml version="1.0" ?>
<mujoco model="actuator_auto_limited">
  <default>
    <geom type="sphere" size="0.1" />
  </default>
  <worldbody>
    <body name="root">
      <geom type="sphere" size="0.1" />
      <joint name="hinge1" type="hinge" axis="0 0 1" />
    </body>
    <body name="root2" pos="1 0 0">
      <geom type="sphere" size="0.1" />
      <joint name="hinge2" type="hinge" axis="0 0 1" />
    </body>
  </worldbody>
  <actuator>
    <motor joint="hinge1" forcelimited="auto" forcerange="-5 5" />
    <motor joint="hinge2" ctrllimited="auto" />
  </actuator>
</mujoco>
)";
  const auto tempPath = std::filesystem::temp_directory_path()
                        / "dart_mjcf_actuator_auto_limited.xml";
  std::ofstream output(tempPath.string(), std::ios::binary);
  output << xml;
  output.close();

  auto world = utils::MjcfParser::readWorld(common::Uri(tempPath.string()));
  std::error_code ec;
  std::filesystem::remove(tempPath, ec);
  ASSERT_NE(world, nullptr);

  auto* joint1 = world->getSkeleton(0)->getJoint("hinge1");
  ASSERT_NE(joint1, nullptr);
  EXPECT_DOUBLE_EQ(joint1->getForceLowerLimit(0), -5.0);
  EXPECT_DOUBLE_EQ(joint1->getForceUpperLimit(0), 5.0);

  auto* joint2 = world->getSkeleton(1)->getJoint("hinge2");
  ASSERT_NE(joint2, nullptr);
  EXPECT_EQ(joint2->getActuatorType(), dynamics::Joint::FORCE);
}

//==============================================================================
TEST(MjcfParserTest, ActuatorPositionSetsServoType)
{
  const std::string xml = R"(
<?xml version="1.0" ?>
<mujoco model="actuator_position">
  <default>
    <geom type="sphere" size="0.1" />
  </default>
  <worldbody>
    <body name="root">
      <geom type="sphere" size="0.1" />
      <joint name="slide1" type="slide" axis="1 0 0" />
    </body>
  </worldbody>
  <actuator>
    <position joint="slide1" />
  </actuator>
</mujoco>
)";
  const auto tempPath = std::filesystem::temp_directory_path()
                        / "dart_mjcf_actuator_position.xml";
  std::ofstream output(tempPath.string(), std::ios::binary);
  output << xml;
  output.close();

  auto world = utils::MjcfParser::readWorld(common::Uri(tempPath.string()));
  std::error_code ec;
  std::filesystem::remove(tempPath, ec);
  ASSERT_NE(world, nullptr);

  auto* joint = world->getSkeleton(0)->getJoint("slide1");
  ASSERT_NE(joint, nullptr);
  EXPECT_EQ(joint->getActuatorType(), dynamics::Joint::SERVO);
}

//==============================================================================
TEST(MjcfParserTest, ContactExcludeDisablesCollision)
{
  const std::string xml = R"(
<?xml version="1.0" ?>
<mujoco model="contact_exclude">
  <default>
    <geom type="sphere" size="0.1" />
  </default>
  <worldbody>
    <body name="body1">
      <geom type="sphere" size="0.1" />
    </body>
    <body name="body2" pos="0 0 1">
      <geom type="sphere" size="0.1" />
    </body>
  </worldbody>
  <contact>
    <exclude body1="body1" body2="body2" />
  </contact>
</mujoco>
)";
  const auto tempPath = std::filesystem::temp_directory_path()
                        / "dart_mjcf_contact_exclude.xml";
  std::ofstream output(tempPath.string(), std::ios::binary);
  output << xml;
  output.close();

  auto world = utils::MjcfParser::readWorld(common::Uri(tempPath.string()));
  std::error_code ec;
  std::filesystem::remove(tempPath, ec);
  ASSERT_NE(world, nullptr);

  // Verify that a collision filter was set
  const auto& collisionOption
      = world->getConstraintSolver()->getCollisionOption();
  EXPECT_NE(collisionOption.collisionFilter, nullptr);
}

//==============================================================================
TEST(MjcfParserTest, AssetTextureAndMaterialParsing)
{
  const std::string xml = R"(
<?xml version="1.0" ?>
<mujoco model="asset_texture_material">
  <default>
    <geom type="sphere" size="0.1" />
  </default>
  <asset>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.8 0.8 0.8"
             rgb2="0.2 0.2 0.2" width="512" height="512" />
    <material name="grid_mat" texture="grid" rgba="1 1 1 1"
              emission="0.1" specular="0.8" shininess="0.9" reflectance="0.2" />
  </asset>
  <worldbody>
    <body name="root">
      <geom type="sphere" size="0.1" />
    </body>
  </worldbody>
</mujoco>
)";
  const auto tempPath
      = std::filesystem::temp_directory_path() / "dart_mjcf_asset_tex_mat.xml";
  std::ofstream output(tempPath.string(), std::ios::binary);
  output << xml;
  output.close();

  auto mujoco = utils::MjcfParser::detail::MujocoModel();
  auto errors = mujoco.read(common::Uri(tempPath.string()), createRetriever());
  std::error_code ec;
  std::filesystem::remove(tempPath, ec);
  ASSERT_TRUE(errors.empty());

  const auto& asset = mujoco.getAsset();
  ASSERT_EQ(asset.getNumTextures(), 1u);
  EXPECT_EQ(asset.getTexture(0).getName(), "grid");
  EXPECT_EQ(asset.getTexture(0).getType(), "2d");
  EXPECT_EQ(asset.getTexture(0).getBuiltin(), "checker");

  const auto* texByName = asset.getTexture("grid");
  ASSERT_NE(texByName, nullptr);
  EXPECT_EQ(texByName->getName(), "grid");

  ASSERT_EQ(asset.getNumMaterials(), 1u);
  EXPECT_EQ(asset.getMaterial(0).getName(), "grid_mat");
  EXPECT_EQ(asset.getMaterial(0).getTexture(), "grid");
  EXPECT_DOUBLE_EQ(asset.getMaterial(0).getEmission(), 0.1);
  EXPECT_DOUBLE_EQ(asset.getMaterial(0).getSpecular(), 0.8);
  EXPECT_DOUBLE_EQ(asset.getMaterial(0).getShininess(), 0.9);
  EXPECT_DOUBLE_EQ(asset.getMaterial(0).getReflectance(), 0.2);

  const auto* matByName = asset.getMaterial("grid_mat");
  ASSERT_NE(matByName, nullptr);
  EXPECT_EQ(matByName->getName(), "grid_mat");
}

//==============================================================================
TEST(MjcfParserTest, ActuatorDefaultAttributes)
{
  const std::string xml = R"(
<?xml version="1.0" ?>
<mujoco model="actuator_defaults">
  <default>
    <geom type="sphere" size="0.1" />
    <motor ctrllimited="true" ctrlrange="-1 1" forcelimited="true"
           forcerange="-50 50" gear="3" />
  </default>
  <worldbody>
    <body name="root">
      <geom type="sphere" size="0.1" />
      <joint name="hinge1" type="hinge" axis="0 0 1" />
    </body>
  </worldbody>
  <actuator>
    <motor joint="hinge1" name="motor1" />
  </actuator>
</mujoco>
 )";
  const auto tempPath = std::filesystem::temp_directory_path()
                        / "dart_mjcf_actuator_defaults.xml";
  std::ofstream output(tempPath.string(), std::ios::binary);
  output << xml;
  output.close();

  auto world = utils::MjcfParser::readWorld(common::Uri(tempPath.string()));
  std::error_code ec;
  std::filesystem::remove(tempPath, ec);
  ASSERT_NE(world, nullptr);

  auto* joint = world->getSkeleton(0)->getJoint("hinge1");
  ASSERT_NE(joint, nullptr);
  EXPECT_TRUE(std::isinf(joint->getForceLowerLimit(0)));
  EXPECT_TRUE(std::isinf(joint->getForceUpperLimit(0)));
  EXPECT_LT(joint->getForceLowerLimit(0), 0.0);
  EXPECT_GT(joint->getForceUpperLimit(0), 0.0);
}

//==============================================================================
TEST(MjcfParserTest, ActuatorVelocityAndGeneralTypes)
{
  const std::string xml = R"(
<?xml version="1.0" ?>
<mujoco model="actuator_vel_general">
  <default><geom type="sphere" size="0.1" /></default>
  <worldbody>
    <body name="root">
      <geom type="sphere" size="0.1" />
      <joint name="j1" type="hinge" axis="0 0 1" />
    </body>
    <body name="root2" pos="1 0 0">
      <geom type="sphere" size="0.1" />
      <joint name="j2" type="hinge" axis="0 0 1" />
    </body>
  </worldbody>
  <actuator>
    <velocity joint="j1" />
    <general joint="j2" gainprm="100 0 0" biasprm="0 -100 -10" />
  </actuator>
</mujoco>
 )";
  const auto tempPath = std::filesystem::temp_directory_path()
                        / "dart_mjcf_actuator_vel_general.xml";
  std::ofstream output(tempPath.string(), std::ios::binary);
  output << xml;
  output.close();

  auto world = utils::MjcfParser::readWorld(common::Uri(tempPath.string()));
  std::error_code ec;
  std::filesystem::remove(tempPath, ec);
  ASSERT_NE(world, nullptr);

  auto* joint1 = world->getSkeleton(0)->getJoint("j1");
  ASSERT_NE(joint1, nullptr);
  EXPECT_EQ(joint1->getActuatorType(), dynamics::Joint::VELOCITY);

  auto* joint2 = world->getSkeleton(1)->getJoint("j2");
  ASSERT_NE(joint2, nullptr);
}

//==============================================================================
TEST(MjcfParserTest, ContactPairParsing)
{
  const std::string xml = R"(
<?xml version="1.0" ?>
<mujoco model="contact_pair">
  <default><geom type="sphere" size="0.1" /></default>
  <worldbody>
    <body name="body1">
      <geom name="geom1" type="sphere" size="0.1" />
    </body>
    <body name="body2" pos="0 0 1">
      <geom name="geom2" type="sphere" size="0.1" />
    </body>
  </worldbody>
  <contact>
    <pair geom1="geom1" geom2="geom2" condim="4" margin="0.001" gap="0.01" />
  </contact>
</mujoco>
 )";
  const auto tempPath
      = std::filesystem::temp_directory_path() / "dart_mjcf_contact_pair.xml";
  std::ofstream output(tempPath.string(), std::ios::binary);
  output << xml;
  output.close();

  auto mujoco = utils::MjcfParser::detail::MujocoModel();
  auto errors = mujoco.read(common::Uri(tempPath.string()), createRetriever());
  std::error_code ec;
  std::filesystem::remove(tempPath, ec);
  ASSERT_TRUE(errors.empty());

  const auto& contact = mujoco.getContact();
  ASSERT_EQ(contact.getNumPairs(), 1u);
  const auto& pair = contact.getPair(0);
  EXPECT_EQ(pair.mGeom1, "geom1");
  EXPECT_EQ(pair.mGeom2, "geom2");
  EXPECT_EQ(pair.mCondim, 4);
  EXPECT_DOUBLE_EQ(pair.mMargin, 0.001);
  EXPECT_DOUBLE_EQ(pair.mGap, 0.01);
}

//==============================================================================
TEST(MjcfParserTest, BodyCameraExtendedAttributes)
{
  const std::string xml = R"(
<?xml version="1.0" ?>
<mujoco model="camera_extended">
  <default><geom type="sphere" size="0.1" /></default>
  <worldbody>
    <body name="root">
      <geom type="sphere" size="0.1" />
      <camera name="cam_targeted" pos="1 0 0" target="root"
              quat="1 0 0 0" />
    </body>
  </worldbody>
</mujoco>
 )";
  const auto tempPath = std::filesystem::temp_directory_path()
                        / "dart_mjcf_camera_extended.xml";
  std::ofstream output(tempPath.string(), std::ios::binary);
  output << xml;
  output.close();

  auto mujoco = utils::MjcfParser::detail::MujocoModel();
  auto errors = mujoco.read(common::Uri(tempPath.string()), createRetriever());
  std::error_code ec;
  std::filesystem::remove(tempPath, ec);
  ASSERT_TRUE(errors.empty());

  const auto& body = mujoco.getWorldbody().getRootBody(0);
  ASSERT_EQ(body.getNumCameras(), 1u);
  const auto& camera = body.getCamera(0);
  EXPECT_EQ(camera.getTarget(), "root");
}

//==============================================================================
TEST(MjcfParserTest, BodyLightExtendedAttributes)
{
  const std::string xml = R"(
<?xml version="1.0" ?>
<mujoco model="light_extended">
  <default><geom type="sphere" size="0.1" /></default>
  <worldbody>
    <body name="root">
      <geom type="sphere" size="0.1" />
      <light name="inactive_light" active="false" directional="false"
             pos="0 0 3" dir="0 0 -1" />
    </body>
  </worldbody>
</mujoco>
 )";
  const auto tempPath
      = std::filesystem::temp_directory_path() / "dart_mjcf_light_extended.xml";
  std::ofstream output(tempPath.string(), std::ios::binary);
  output << xml;
  output.close();

  auto mujoco = utils::MjcfParser::detail::MujocoModel();
  auto errors = mujoco.read(common::Uri(tempPath.string()), createRetriever());
  std::error_code ec;
  std::filesystem::remove(tempPath, ec);
  ASSERT_TRUE(errors.empty());

  const auto& body = mujoco.getWorldbody().getRootBody(0);
  ASSERT_EQ(body.getNumLights(), 1u);
  const auto& light = body.getLight(0);
  EXPECT_FALSE(light.getActive());
  EXPECT_FALSE(light.getDirectional());
}

//==============================================================================
TEST(MjcfParserTest, AssetMaterialRgba)
{
  const std::string xml = R"(
<?xml version="1.0" ?>
<mujoco model="material_rgba">
  <default><geom type="sphere" size="0.1" /></default>
  <asset>
    <material name="red_mat" rgba="1 0 0 0.5" />
  </asset>
  <worldbody>
    <body name="root">
      <geom type="sphere" size="0.1" />
    </body>
  </worldbody>
</mujoco>
 )";
  const auto tempPath
      = std::filesystem::temp_directory_path() / "dart_mjcf_material_rgba.xml";
  std::ofstream output(tempPath.string(), std::ios::binary);
  output << xml;
  output.close();

  auto mujoco = utils::MjcfParser::detail::MujocoModel();
  auto errors = mujoco.read(common::Uri(tempPath.string()), createRetriever());
  std::error_code ec;
  std::filesystem::remove(tempPath, ec);
  ASSERT_TRUE(errors.empty());

  const auto& asset = mujoco.getAsset();
  ASSERT_EQ(asset.getNumMaterials(), 1u);
  const auto* material = asset.getMaterial("red_mat");
  ASSERT_NE(material, nullptr);
  EXPECT_TRUE(material->getRgba().isApprox(Eigen::Vector4d(1, 0, 0, 0.5)));
}

//==============================================================================
TEST(MjcfParserTest, UnknownElementWarns)
{
  // An unknown element <foo> in <body> should not cause parse errors
  // (warnUnknownElements logs but does not add to Errors vector)
  const std::string xml = R"(
<?xml version="1.0" ?>
<mujoco model="unknown_element">
  <default>
    <geom type="sphere" size="0.1" />
  </default>
  <worldbody>
    <body name="root">
      <geom type="sphere" size="0.1" />
      <foo bar="baz" />
    </body>
  </worldbody>
</mujoco>
)";
  const auto tempPath
      = std::filesystem::temp_directory_path() / "dart_mjcf_unknown_elem.xml";
  std::ofstream output(tempPath.string(), std::ios::binary);
  output << xml;
  output.close();

  auto mujoco = utils::MjcfParser::detail::MujocoModel();
  auto errors = mujoco.read(common::Uri(tempPath.string()), createRetriever());
  std::error_code ec;
  std::filesystem::remove(tempPath, ec);
  // Unknown elements produce warnings, not errors
  EXPECT_TRUE(errors.empty());
}

//==============================================================================
TEST(MjcfParserTest, JointArmatureParsed)
{
  const std::string xml = R"(
<?xml version="1.0" ?>
<mujoco model="joint_armature">
  <default>
    <geom type="sphere" size="0.1" />
  </default>
  <worldbody>
    <body name="root">
      <geom type="sphere" size="0.1" />
      <joint name="hinge1" type="hinge" armature="0.5"
             stiffness="10" frictionloss="0.3" />
    </body>
  </worldbody>
</mujoco>
)";
  const auto tempPath
      = std::filesystem::temp_directory_path() / "dart_mjcf_joint_armature.xml";
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
  EXPECT_DOUBLE_EQ(joint.getArmature(), 0.5);
  EXPECT_DOUBLE_EQ(joint.getStiffness(), 10.0);
  EXPECT_DOUBLE_EQ(joint.getFrictionLoss(), 0.3);
}

//==============================================================================
TEST(MjcfParserTest, MultipleFreeJointsError)
{
  // A body with both <freejoint> and <joint type="free"> should produce error
  const std::string xml = R"(
<?xml version="1.0" ?>
<mujoco model="multi_free">
  <default>
    <geom type="sphere" size="0.1" />
  </default>
  <worldbody>
    <body name="root">
      <geom type="sphere" size="0.1" />
      <freejoint name="fj" />
      <joint name="jf" type="free" />
    </body>
  </worldbody>
</mujoco>
)";
  const auto tempPath
      = std::filesystem::temp_directory_path() / "dart_mjcf_multi_free.xml";
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
    if (error.getCode() == ErrorCode::ELEMENT_INVALID) {
      hasInvalid = true;
      break;
    }
  }
  EXPECT_TRUE(hasInvalid);
}

//==============================================================================
TEST(MjcfParserTest, AutoLimitsZeroRangeSetsUnlimited)
{
  // autolimits=true (default) + no range attr + no default range override
  // → range stays (0,0) → mLimited = false
  const std::string xml = R"(
<?xml version="1.0" ?>
<mujoco model="autolimits_zero_range">
  <default>
    <geom type="sphere" size="0.1" />
    <joint type="hinge" />
  </default>
  <worldbody>
    <body name="root">
      <geom type="sphere" size="0.1" />
      <joint name="hinge1" type="hinge" />
    </body>
  </worldbody>
</mujoco>
)";
  const auto tempPath = std::filesystem::temp_directory_path()
                        / "dart_mjcf_autolimits_zero.xml";
  std::ofstream output(tempPath.string(), std::ios::binary);
  output << xml;
  output.close();

  auto world = utils::MjcfParser::readWorld(common::Uri(tempPath.string()));
  std::error_code ec;
  std::filesystem::remove(tempPath, ec);
  ASSERT_NE(world, nullptr);
  auto* joint = world->getSkeleton(0)->getJoint("hinge1");
  ASSERT_NE(joint, nullptr);
  EXPECT_FALSE(joint->areLimitsEnforced());
}

//==============================================================================
TEST(MjcfParserTest, RootBodyUnnamedJointGetsDefaultName)
{
  // A root body with an unnamed joint should get "root_Joint" as name
  // (null parent → falls through to "root_Joint" path in
  // createJointCommonProperties)
  const std::string xml = std::string(R"(
 <?xml version="1.0" ?>
 <mujoco model="root_unnamed_joint">
   <default>
     <geom type="sphere" size="0.1" />
   </default>
   <!-- )") + kMjcfPadding + R"( -->
   <worldbody>
     <body name="root">
       <geom type="sphere" size="0.1" />
       <joint type="hinge" range="-1 1" />
     </body>
   </worldbody>
 </mujoco>
)";
  const auto tempPath
      = std::filesystem::temp_directory_path() / "dart_mjcf_root_unnamed.xml";
  std::ofstream output(tempPath.string(), std::ios::binary);
  output << xml;
  output.close();

  auto world = utils::MjcfParser::readWorld(common::Uri(tempPath.string()));
  std::error_code ec;
  std::filesystem::remove(tempPath, ec);
  ASSERT_NE(world, nullptr);

  auto* skel = world->getSkeleton(0).get();
  ASSERT_NE(skel, nullptr);
  // The unnamed joint on a root body (no parent BodyNode) gets "root_Joint"
  auto* joint = skel->getJoint("root_Joint");
  EXPECT_NE(joint, nullptr);
}

//==============================================================================
TEST(MjcfParserTest, ActuatorNegativeGearPreservesForceOrdering)
{
  // A negative gear should not flip force limits (lower > upper)
  const std::string xml = R"(
<?xml version="1.0" ?>
<mujoco model="actuator_neg_gear">
  <default>
    <geom type="sphere" size="0.1" />
  </default>
  <worldbody>
    <body name="root">
      <geom type="sphere" size="0.1" />
      <joint name="hinge1" type="hinge" axis="0 0 1" />
    </body>
  </worldbody>
  <actuator>
    <motor joint="hinge1" gear="-3" forcelimited="true" forcerange="-10 10" />
  </actuator>
</mujoco>
)";
  const auto tempPath
      = std::filesystem::temp_directory_path() / "dart_mjcf_neg_gear.xml";
  std::ofstream output(tempPath.string(), std::ios::binary);
  output << xml;
  output.close();

  auto world = utils::MjcfParser::readWorld(common::Uri(tempPath.string()));
  std::error_code ec;
  std::filesystem::remove(tempPath, ec);
  ASSERT_NE(world, nullptr);

  auto* joint = world->getSkeleton(0)->getJoint("hinge1");
  ASSERT_NE(joint, nullptr);
  // gear=-3, forcerange=[-10,10]: scaled = [30, -30]
  // After ordering: lower=-30, upper=30
  EXPECT_DOUBLE_EQ(joint->getForceLowerLimit(0), -30.0);
  EXPECT_DOUBLE_EQ(joint->getForceUpperLimit(0), 30.0);
  EXPECT_LE(joint->getForceLowerLimit(0), joint->getForceUpperLimit(0));
}

//==============================================================================
TEST(MjcfParserTest, JointLimitedAutoValue)
{
  // limited="auto" should behave the same as absent (std::nullopt)
  const std::string xml = R"(
<?xml version="1.0" ?>
<mujoco model="limited_auto">
  <default>
    <geom type="sphere" size="0.1" />
  </default>
  <worldbody>
    <body name="root">
      <geom type="sphere" size="0.1" />
      <joint name="hinge1" type="hinge" limited="auto" range="-1 1" />
    </body>
  </worldbody>
</mujoco>
)";
  const auto tempPath
      = std::filesystem::temp_directory_path() / "dart_mjcf_limited_auto.xml";
  std::ofstream output(tempPath.string(), std::ios::binary);
  output << xml;
  output.close();

  // With autolimits=true (default), limited="auto" → nullopt → inferred from
  // range
  auto world = utils::MjcfParser::readWorld(common::Uri(tempPath.string()));
  std::error_code ec;
  std::filesystem::remove(tempPath, ec);
  ASSERT_NE(world, nullptr);
  auto* joint = world->getSkeleton(0)->getJoint("hinge1");
  ASSERT_NE(joint, nullptr);
  // range=(-1,1) non-zero + autolimits → limited=true
  EXPECT_TRUE(joint->areLimitsEnforced());
}

//==============================================================================
TEST(MjcfParserTest, BodyCameraAndLightParsing)
{
  const std::string xml = R"(
<?xml version="1.0" ?>
<mujoco model="body_camera_light">
  <default>
    <geom type="sphere" size="0.1" />
  </default>
  <worldbody>
    <body name="root">
      <geom type="sphere" size="0.1" />
      <camera name="cam1" fovy="60" pos="0 -1 1" mode="trackcom" />
      <light name="light1" pos="0 0 2" dir="0 0 -1"
             diffuse="0.9 0.9 0.9" specular="0.1 0.1 0.1"
             directional="true" />
    </body>
  </worldbody>
</mujoco>
)";
  const auto tempPath
      = std::filesystem::temp_directory_path() / "dart_mjcf_camera_light.xml";
  std::ofstream output(tempPath.string(), std::ios::binary);
  output << xml;
  output.close();

  auto mujoco = utils::MjcfParser::detail::MujocoModel();
  auto errors = mujoco.read(common::Uri(tempPath.string()), createRetriever());
  std::error_code ec;
  std::filesystem::remove(tempPath, ec);
  ASSERT_TRUE(errors.empty());

  const auto& body = mujoco.getWorldbody().getRootBody(0);

  ASSERT_EQ(body.getNumCameras(), 1u);
  const auto& cam = body.getCamera(0);
  EXPECT_EQ(cam.getName(), "cam1");
  EXPECT_DOUBLE_EQ(cam.getFovy(), 60.0);
  EXPECT_TRUE(cam.getPos().isApprox(Eigen::Vector3d(0, -1, 1)));
  EXPECT_EQ(cam.getMode(), "trackcom");

  ASSERT_EQ(body.getNumLights(), 1u);
  const auto& light = body.getLight(0);
  EXPECT_EQ(light.getName(), "light1");
  EXPECT_TRUE(light.getPos().isApprox(Eigen::Vector3d(0, 0, 2)));
  EXPECT_TRUE(light.getDir().isApprox(Eigen::Vector3d(0, 0, -1)));
  EXPECT_TRUE(light.getDiffuse().isApprox(Eigen::Vector3d(0.9, 0.9, 0.9)));
  EXPECT_TRUE(light.getSpecular().isApprox(Eigen::Vector3d(0.1, 0.1, 0.1)));
  EXPECT_TRUE(light.getDirectional());
}
