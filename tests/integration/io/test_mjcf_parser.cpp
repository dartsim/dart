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
#define private public
#include "dart/utils/mjcf/detail/actuator.hpp"
#include "dart/utils/mjcf/detail/asset.hpp"
#include "dart/utils/mjcf/detail/camera.hpp"
#include "dart/utils/mjcf/detail/contact.hpp"
#include "dart/utils/mjcf/detail/default.hpp"
#include "dart/utils/mjcf/detail/equality.hpp"
#include "dart/utils/mjcf/detail/joint.hpp"
#include "dart/utils/mjcf/detail/light.hpp"
#include "dart/utils/mjcf/detail/material.hpp"
#include "dart/utils/mjcf/detail/mesh.hpp"
#include "dart/utils/mjcf/detail/mujoco_model.hpp"
#include "dart/utils/mjcf/detail/size.hpp"
#include "dart/utils/mjcf/detail/texture.hpp"
#include "dart/utils/mjcf/detail/weld.hpp"
#undef private
#include "dart/utils/mjcf/detail/body_attributes.hpp"
#include "dart/utils/mjcf/detail/joint_attributes.hpp"
#include "dart/utils/mjcf/detail/mesh_attributes.hpp"
#include "dart/utils/mjcf/detail/types.hpp"
#include "dart/utils/mjcf/detail/utils.hpp"
#include "dart/utils/mjcf/detail/weld_attributes.hpp"

#include <dart/all.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <limits>
#include <map>
#include <optional>
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
    ptrdiff_t base = 0;
    if (origin == SEEKTYPE_CUR) {
      base = static_cast<ptrdiff_t>(mOffset);
    } else if (origin == SEEKTYPE_END) {
      base = std::ssize(mData);
    }

    const auto next = base + offset;
    if (next < 0 || next > std::ssize(mData)) {
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
    return mFiles.contains(uri.toString());
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

bool hasErrorCode(const Errors& errors, ErrorCode code)
{
  return std::ranges::any_of(
      errors, [code](const Error& error) { return error.getCode() == code; });
}

void expectSingleErrorCode(const Errors& errors, ErrorCode code)
{
  ASSERT_EQ(errors.size(), 1u);
  EXPECT_EQ(errors.front().getCode(), code);
}

tinyxml2::XMLElement* parseRootElement(
    tinyxml2::XMLDocument& document, const char* xml)
{
  if (document.Parse(xml) != tinyxml2::XML_SUCCESS) {
    return nullptr;
  }

  return document.RootElement();
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
TEST(MjcfParserTest, IncludeErrorsReportMalformedFilesAndRoots)
{
  auto retriever = std::make_shared<MemoryResourceRetriever>();

  retriever->add("memory://mjcf_bad_xml_child.xml", "<mujoco>");

  const auto malformedXmlUri = addMemoryFile(
      *retriever,
      "memory://mjcf_malformed_xml_include.xml",
      R"(
<?xml version="1.0" ?>
<mujoco model="malformed_xml_include">
  <default>
    <geom type="sphere" size="0.1" />
  </default>
  <include file="memory://mjcf_bad_xml_child.xml" />
  <worldbody />
</mujoco>
)");

  auto malformedXmlMujoco = utils::MjcfParser::detail::MujocoModel();
  const auto malformedXmlErrors
      = malformedXmlMujoco.read(malformedXmlUri, retriever);
  ASSERT_FALSE(malformedXmlErrors.empty());
  EXPECT_TRUE(hasErrorCode(malformedXmlErrors, ErrorCode::FILE_READ));
  EXPECT_TRUE(hasErrorCode(malformedXmlErrors, ErrorCode::ELEMENT_MISSING));
  EXPECT_EQ(
      utils::MjcfParser::readWorld(
          malformedXmlUri, utils::MjcfParser::Options(retriever)),
      nullptr);

  retriever->add(
      "memory://mjcf_not_mujoco_child.xml",
      R"(
<?xml version="1.0" ?>
<not_mujoco />
)");

  const auto malformedUri = addMemoryFile(
      *retriever,
      "memory://mjcf_malformed_include.xml",
      R"(
<?xml version="1.0" ?>
<mujoco model="malformed_include">
  <default>
    <geom type="sphere" size="0.1" />
  </default>
  <include file="memory://mjcf_not_mujoco_child.xml" />
  <worldbody />
</mujoco>
)");

  auto malformedMujoco = utils::MjcfParser::detail::MujocoModel();
  const auto malformedErrors = malformedMujoco.read(malformedUri, retriever);
  ASSERT_FALSE(malformedErrors.empty());
  EXPECT_TRUE(hasErrorCode(malformedErrors, ErrorCode::ELEMENT_MISSING));

  retriever->add("memory://mjcf_malformed_root.xml", "<mujoco>");

  auto malformedRootMujoco = utils::MjcfParser::detail::MujocoModel();
  const auto malformedRootErrors = malformedRootMujoco.read(
      common::Uri("memory://mjcf_malformed_root.xml"), retriever);
  expectSingleErrorCode(malformedRootErrors, ErrorCode::FILE_READ);

  retriever->add(
      "memory://mjcf_not_mujoco_root.xml",
      R"(
<?xml version="1.0" ?>
<not_mujoco />
)");

  auto notMujocoRoot = utils::MjcfParser::detail::MujocoModel();
  const auto notMujocoRootErrors = notMujocoRoot.read(
      common::Uri("memory://mjcf_not_mujoco_root.xml"), retriever);
  expectSingleErrorCode(notMujocoRootErrors, ErrorCode::ELEMENT_MISSING);
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
TEST(MjcfParserTest, ReportsInvalidOptionAndSiteAttributes)
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
      <site name="bad_size_site" type="box" size="0.1 0.2 0.3 0.4" />
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
  EXPECT_TRUE(hasErrorCode(errors, ErrorCode::ATTRIBUTE_INVALID));
}

//==============================================================================
TEST(MjcfParserTest, OptionInvalidEnumAttributesReportErrors)
{
  struct InvalidOptionCase
  {
    const char* mUri;
    const char* mAttributes;
  };

  const InvalidOptionCase cases[] = {
      {"memory://mjcf_bad_collision_option.xml", R"(collision="unsupported")"},
      {"memory://mjcf_bad_cone_option.xml", R"(cone="unsupported")"},
      {"memory://mjcf_bad_jacobian_option.xml", R"(jacobian="unsupported")"},
      {"memory://mjcf_bad_solver_option.xml", R"(solver="unsupported")"},
  };

  for (const auto& optionCase : cases) {
    SCOPED_TRACE(optionCase.mUri);

    auto retriever = std::make_shared<MemoryResourceRetriever>();
    const std::string xml = std::string(R"(
<?xml version="1.0" ?>
<mujoco model="bad_option">
  <default>
    <geom type="sphere" size="0.1" />
  </default>
  <option )") + optionCase.mAttributes
                            + R"( />
  <worldbody>
    <body name="root">
      <geom type="sphere" size="0.1" />
    </body>
  </worldbody>
</mujoco>
)";
    const auto uri = addMemoryFile(*retriever, optionCase.mUri, xml);

    auto mujoco = utils::MjcfParser::detail::MujocoModel();
    const auto errors = mujoco.read(uri, retriever);
    ASSERT_FALSE(errors.empty());
    EXPECT_TRUE(hasErrorCode(errors, ErrorCode::ATTRIBUTE_INVALID));
  }
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
TEST(MjcfParserTest, CompilerInertiaFromGeomVariantsAndErrors)
{
  struct InertiaFromGeomCase
  {
    const char* mUri;
    const char* mValue;
    bool mValid;
    InertiaFromGeom mExpectedValue;
  };

  const InertiaFromGeomCase cases[] = {
      {"memory://mjcf_compiler_inertiafromgeom_false.xml",
       "false",
       true,
       InertiaFromGeom::IFG_FALSE},
      {"memory://mjcf_compiler_inertiafromgeom_true.xml",
       "true",
       true,
       InertiaFromGeom::IFG_TRUE},
      {"memory://mjcf_compiler_inertiafromgeom_auto.xml",
       "auto",
       true,
       InertiaFromGeom::IFG_AUTO},
      {"memory://mjcf_compiler_inertiafromgeom_invalid.xml",
       "unsupported",
       false,
       InertiaFromGeom::IFG_AUTO},
  };

  for (const auto& inertiaCase : cases) {
    SCOPED_TRACE(inertiaCase.mUri);

    const std::string xml = std::string(R"(
<?xml version="1.0" ?>
<mujoco model="compiler_inertiafromgeom">
  <compiler inertiafromgeom=")")
                            + inertiaCase.mValue + R"(" />
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

    auto retriever = std::make_shared<MemoryResourceRetriever>();
    const auto uri = addMemoryFile(*retriever, inertiaCase.mUri, xml);

    auto mujoco = utils::MjcfParser::detail::MujocoModel();
    const auto errors = mujoco.read(uri, retriever);
    if (inertiaCase.mValid) {
      ASSERT_TRUE(errors.empty());
      EXPECT_EQ(
          mujoco.getCompiler().getInertiaFromGeom(),
          inertiaCase.mExpectedValue);
    } else {
      ASSERT_FALSE(errors.empty());
      EXPECT_TRUE(hasErrorCode(errors, ErrorCode::ATTRIBUTE_INVALID));
    }
  }
}

//==============================================================================
TEST(MjcfParserTest, CompileRotationNonFiniteInputsResetToIdentity)
{
  const Compiler compiler;
  const auto nan = std::numeric_limits<double>::quiet_NaN();
  const Eigen::Quaterniond validQuat = Eigen::Quaterniond::Identity();
  const std::optional<Eigen::Vector4d> noAxisAngle;
  const std::optional<Eigen::Vector3d> noEuler;
  const std::optional<Eigen::Vector6d> noXYAxes;
  const std::optional<Eigen::Vector3d> noZAxis;

  const auto expectIdentity = [](const Eigen::Matrix3d& rotation) {
    EXPECT_TRUE(rotation.isApprox(Eigen::Matrix3d::Identity()));
  };

  expectIdentity(compileRotation(
      validQuat,
      std::optional<Eigen::Vector4d>{Eigen::Vector4d(nan, 0, 0, 90)},
      noEuler,
      noXYAxes,
      noZAxis,
      compiler));
  expectIdentity(compileRotation(
      validQuat,
      noAxisAngle,
      std::optional<Eigen::Vector3d>{Eigen::Vector3d(nan, 0, 0)},
      noXYAxes,
      noZAxis,
      compiler));
  expectIdentity(compileRotation(
      validQuat,
      noAxisAngle,
      noEuler,
      std::optional<Eigen::Vector6d>{Eigen::Vector6d::Zero()},
      noZAxis,
      compiler));
  expectIdentity(compileRotation(
      validQuat,
      noAxisAngle,
      noEuler,
      noXYAxes,
      std::optional<Eigen::Vector3d>{Eigen::Vector3d(nan, 0, 0)},
      compiler));
  expectIdentity(compileRotation(
      Eigen::Quaterniond(nan, 0, 0, 0),
      noAxisAngle,
      noEuler,
      noXYAxes,
      noZAxis,
      compiler));
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
  EXPECT_TRUE(hasErrorCode(errors, ErrorCode::ATTRIBUTE_INVALID));
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
  EXPECT_TRUE(hasErrorCode(errors, ErrorCode::ATTRIBUTE_INVALID));
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
      <geom name="sphere_fromto" type="sphere" size="0.3"
            fromto="0 0 0 0 0 5" pos="1 2 3" />
      <geom name="capsule" type="capsule" size="0.2"
            fromto="0 0 0 0 0 2" />
      <geom name="cylinder_plain" type="cylinder" size="0.4 0.8"
            pos="2 3 4" />
      <geom name="ellipsoid_plain" type="ellipsoid" size="0.2 0.3 0.4"
            pos="3 4 5" />
      <geom name="box" type="box" size="0.1 0.2 0.3"
            fromto="0 0 0 0 0 4" />
      <geom name="xyaxes_box" type="box" size="0.2 0.3 0.4"
            pos="4 5 6" xyaxes="0 1 0 -1 0 0" />
      <geom name="zaxis_box" type="box" size="0.2 0.3 0.4"
            pos="5 6 7" zaxis="0 1 0" />
      <geom name="hfield_named" type="hfield" hfield="terrain"
            size="1 2 0.1" />
      <geom name="badhfield" type="hfield" size="1 1 0.1" />
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
  EXPECT_TRUE(hasErrorCode(errors, ErrorCode::ATTRIBUTE_MISSING));
  EXPECT_TRUE(hasErrorCode(errors, ErrorCode::UNDEFINED_ERROR));

  const auto& body = mujoco.getWorldbody().getRootBody(0);
  ASSERT_GE(body.getNumGeoms(), 11u);

  const auto findGeom = [&body](const std::string& name) {
    for (std::size_t i = 0; i < body.getNumGeoms(); ++i) {
      const auto& geom = body.getGeom(i);
      if (geom.getName() == name) {
        return &geom;
      }
    }
    return static_cast<const Geom*>(nullptr);
  };

  const auto* sphereFromTo = findGeom("sphere_fromto");
  ASSERT_NE(sphereFromTo, nullptr);
  EXPECT_EQ(sphereFromTo->getType(), GeomType::SPHERE);
  EXPECT_TRUE(sphereFromTo->getSize().isApprox(Eigen::Vector3d(0.3, 0, 0)));
  EXPECT_NEAR(sphereFromTo->getSphereRadius(), 0.3, 1e-12);
  EXPECT_TRUE(sphereFromTo->getRelativeTransform().translation().isApprox(
      Eigen::Vector3d(1, 2, 3)));

  const auto* capsule = findGeom("capsule");
  ASSERT_NE(capsule, nullptr);
  EXPECT_EQ(capsule->getType(), GeomType::CAPSULE);
  EXPECT_NEAR(capsule->getCapsuleRadius(), 0.2, 1e-12);
  EXPECT_NEAR(capsule->getCapsuleHalfLength(), 1.0, 1e-12);
  EXPECT_NEAR(capsule->getCapsuleLength(), 2.0, 1e-12);
  EXPECT_TRUE(capsule->getRelativeTransform().translation().isApprox(
      Eigen::Vector3d(0, 0, 1)));

  const auto* cylinderPlain = findGeom("cylinder_plain");
  ASSERT_NE(cylinderPlain, nullptr);
  EXPECT_EQ(cylinderPlain->getType(), GeomType::CYLINDER);
  EXPECT_NEAR(cylinderPlain->getCylinderRadius(), 0.4, 1e-12);
  EXPECT_NEAR(cylinderPlain->getCylinderHalfLength(), 0.8, 1e-12);
  EXPECT_NEAR(cylinderPlain->getCylinderLength(), 1.6, 1e-12);
  EXPECT_TRUE(cylinderPlain->getRelativeTransform().translation().isApprox(
      Eigen::Vector3d(2, 3, 4)));

  const auto* ellipsoidPlain = findGeom("ellipsoid_plain");
  ASSERT_NE(ellipsoidPlain, nullptr);
  EXPECT_EQ(ellipsoidPlain->getType(), GeomType::ELLIPSOID);
  EXPECT_TRUE(ellipsoidPlain->getEllipsoidRadii().isApprox(
      Eigen::Vector3d(0.2, 0.3, 0.4)));
  EXPECT_TRUE(ellipsoidPlain->getEllipsoidDiameters().isApprox(
      Eigen::Vector3d(0.4, 0.6, 0.8)));
  EXPECT_GT(ellipsoidPlain->getVolume(), 0.0);
  EXPECT_TRUE(ellipsoidPlain->getRelativeTransform().translation().isApprox(
      Eigen::Vector3d(3, 4, 5)));

  const auto* box = findGeom("box");
  ASSERT_NE(box, nullptr);
  EXPECT_EQ(box->getType(), GeomType::BOX);
  EXPECT_NEAR(box->getBoxHalfSize().z(), 2.0, 1e-12);
  EXPECT_TRUE(box->getBoxSize().isApprox(Eigen::Vector3d(0.2, 0.4, 4.0)));

  const auto* xyaxesBox = findGeom("xyaxes_box");
  ASSERT_NE(xyaxesBox, nullptr);
  EXPECT_EQ(xyaxesBox->getType(), GeomType::BOX);
  EXPECT_TRUE(xyaxesBox->getRelativeTransform().translation().isApprox(
      Eigen::Vector3d(4, 5, 6)));
  EXPECT_TRUE(xyaxesBox->getRelativeTransform().linear().col(0).isApprox(
      Eigen::Vector3d::UnitY()));
  EXPECT_TRUE(xyaxesBox->getRelativeTransform().linear().col(1).isApprox(
      -Eigen::Vector3d::UnitX()));

  const auto* zaxisBox = findGeom("zaxis_box");
  ASSERT_NE(zaxisBox, nullptr);
  EXPECT_EQ(zaxisBox->getType(), GeomType::BOX);
  EXPECT_TRUE(zaxisBox->getRelativeTransform().translation().isApprox(
      Eigen::Vector3d(5, 6, 7)));
  EXPECT_TRUE(zaxisBox->getRelativeTransform().linear().col(2).isApprox(
      Eigen::Vector3d::UnitY()));

  const auto* hfieldNamed = findGeom("hfield_named");
  ASSERT_NE(hfieldNamed, nullptr);
  EXPECT_EQ(hfieldNamed->getType(), GeomType::HFIELD);
  EXPECT_EQ(hfieldNamed->getHField(), "terrain");
  EXPECT_TRUE(hfieldNamed->getSize().isApprox(Eigen::Vector3d(1, 2, 0.1)));

  const auto* badHField = findGeom("badhfield");
  ASSERT_NE(badHField, nullptr);
  EXPECT_EQ(badHField->getType(), GeomType::HFIELD);
  EXPECT_TRUE(badHField->getHField().empty());
  EXPECT_DOUBLE_EQ(badHField->getVolume(), 1.0);

  const auto* badMesh = findGeom("badmesh");
  ASSERT_NE(badMesh, nullptr);
  EXPECT_EQ(badMesh->getType(), GeomType::MESH);
  EXPECT_TRUE(badMesh->getMesh().empty());
  EXPECT_DOUBLE_EQ(badMesh->getVolume(), 1.0);

  const auto* meshfit = findGeom("meshfit");
  ASSERT_NE(meshfit, nullptr);
  EXPECT_EQ(meshfit->getType(), GeomType::BOX);
  EXPECT_EQ(meshfit->getMesh(), "dummy_mesh");
  EXPECT_TRUE(
      meshfit->getBoxHalfSize().isApprox(Eigen::Vector3d::Constant(0.1)));
  EXPECT_DOUBLE_EQ(meshfit->getMass(), 1.0);
  EXPECT_DOUBLE_EQ(meshfit->getVolume(), 1.0);
  EXPECT_DOUBLE_EQ(meshfit->getDensity(), 1000.0);
  EXPECT_TRUE(meshfit->getInertia().allFinite());

  auto mutableGeom = *box;
  Eigen::Isometry3d relative = Eigen::Isometry3d::Identity();
  relative.translation() = Eigen::Vector3d(4, 5, 6);
  mutableGeom.setRelativeTransform(relative);
  EXPECT_TRUE(mutableGeom.getRelativeTransform().isApprox(relative));

  Eigen::Isometry3d world = Eigen::Isometry3d::Identity();
  world.translation() = Eigen::Vector3d(-4, -5, -6);
  mutableGeom.setWorldTransform(world);
  EXPECT_TRUE(mutableGeom.getWorldTransform().isApprox(world));
}

//==============================================================================
TEST(MjcfParserTest, InvalidGeomAttributesReportErrors)
{
  struct InvalidGeomCase
  {
    const char* mUri;
    const char* mGeomAttributes;
  };

  const InvalidGeomCase cases[] = {
      {"memory://mjcf_bad_geom_type.xml", R"(type="unsupported")"},
      {"memory://mjcf_bad_geom_size.xml", R"(size="0.1 0.2 0.3 0.4")"},
      {"memory://mjcf_bad_geom_friction.xml", R"(friction="0.1 0.2 0.3 0.4")"},
  };

  for (const auto& geomCase : cases) {
    SCOPED_TRACE(geomCase.mUri);

    auto retriever = std::make_shared<MemoryResourceRetriever>();
    const std::string xml = std::string(R"(
<?xml version="1.0" ?>
<mujoco model="bad_geom">
  <default>
    <geom type="sphere" size="0.1" />
  </default>
  <worldbody>
    <body name="root">
      <geom )") + geomCase.mGeomAttributes
                            + R"( />
    </body>
  </worldbody>
</mujoco>
)";
    const auto uri = addMemoryFile(*retriever, geomCase.mUri, xml);

    auto mujoco = utils::MjcfParser::detail::MujocoModel();
    const auto errors = mujoco.read(uri, retriever);

    ASSERT_FALSE(errors.empty());
    EXPECT_TRUE(hasErrorCode(errors, ErrorCode::ATTRIBUTE_INVALID));
  }
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
  EXPECT_TRUE(hasErrorCode(errors, ErrorCode::ATTRIBUTE_INVALID));
}

//==============================================================================
TEST(MjcfParserTest, JointAttributesParsed)
{
  const std::string xml = R"(
<?xml version="1.0" ?>
<mujoco model="joint_attrs">
  <default class="main">
    <geom type="sphere" size="0.1" />
    <default class="stiff_slide">
      <joint type="slide" axis="1 0 0" range="-2 2" limited="true"
             damping="0.6" springref="0.7" stiffness="3" armature="0.4"
             frictionloss="0.5" />
    </default>
  </default>
  <worldbody>
    <body name="root">
      <geom type="sphere" size="0.1" />
      <joint name="hinge_joint" type="hinge" pos="0.1 0.2 0.3"
             axis="0 1 0" range="-0.5 0.5" damping="0.2" springref="0.1" />
      <joint class="stiff_slide" name="class_joint" />
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
  ASSERT_EQ(body.getNumJoints(), 2u);
  const auto& joint = body.getJoint(0);
  EXPECT_EQ(joint.getType(), JointType::HINGE);
  EXPECT_TRUE(joint.getPos().isApprox(Eigen::Vector3d(0.1, 0.2, 0.3)));
  EXPECT_TRUE(joint.getAxis().isApprox(Eigen::Vector3d(0, 1, 0)));
  EXPECT_TRUE(joint.getRange().isApprox(Eigen::Vector2d(-0.5, 0.5)));
  EXPECT_DOUBLE_EQ(joint.getDamping(), 0.2);
  EXPECT_DOUBLE_EQ(joint.getSpringRef(), 0.1);

  const auto& classJoint = body.getJoint(1);
  EXPECT_EQ(classJoint.getName(), "class_joint");
  EXPECT_EQ(classJoint.getType(), JointType::SLIDE);
  EXPECT_TRUE(classJoint.getAxis().isApprox(Eigen::Vector3d::UnitX()));
  EXPECT_TRUE(classJoint.getRange().isApprox(Eigen::Vector2d(-2, 2)));
  ASSERT_TRUE(classJoint.getLimited().has_value());
  EXPECT_TRUE(*classJoint.getLimited());
  EXPECT_TRUE(classJoint.isLimited());
  EXPECT_DOUBLE_EQ(classJoint.getDamping(), 0.6);
  EXPECT_DOUBLE_EQ(classJoint.getSpringRef(), 0.7);
  EXPECT_DOUBLE_EQ(classJoint.getStiffness(), 3.0);
  EXPECT_DOUBLE_EQ(classJoint.getArmature(), 0.4);
  EXPECT_DOUBLE_EQ(classJoint.getFrictionLoss(), 0.5);
}

//==============================================================================
TEST(MjcfParserTest, InvalidJointAttributesReportErrors)
{
  struct InvalidJointCase
  {
    const char* mUri;
    const char* mJointAttributes;
  };

  const InvalidJointCase cases[] = {
      {"memory://mjcf_bad_joint_type.xml", R"(type="unsupported")"},
      {"memory://mjcf_bad_joint_limited.xml",
       R"(type="hinge" limited="maybe")"},
      {"memory://mjcf_bad_joint_class.xml", R"(class="missing_class")"},
  };

  for (const auto& jointCase : cases) {
    SCOPED_TRACE(jointCase.mUri);

    auto retriever = std::make_shared<MemoryResourceRetriever>();
    const std::string xml = std::string(R"(
<?xml version="1.0" ?>
<mujoco model="bad_joint">
  <default>
    <geom type="sphere" size="0.1" />
  </default>
  <worldbody>
    <body name="root">
      <geom type="sphere" size="0.1" />
      <joint )") + jointCase.mJointAttributes
                            + R"( />
    </body>
  </worldbody>
</mujoco>
)";
    const auto uri = addMemoryFile(*retriever, jointCase.mUri, xml);

    auto mujoco = utils::MjcfParser::detail::MujocoModel();
    const auto errors = mujoco.read(uri, retriever);

    ASSERT_FALSE(errors.empty());
    EXPECT_TRUE(hasErrorCode(errors, ErrorCode::ATTRIBUTE_INVALID));
  }
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
  EXPECT_EQ(option.getNoSlipIterations(), 5);
  EXPECT_DOUBLE_EQ(option.getNoSlipTolerance(), 0.0002);
  EXPECT_EQ(option.getMprIterations(), 6);
  EXPECT_DOUBLE_EQ(option.getMprTolerance(), 1e-4);
}

//==============================================================================
TEST(MjcfParserTest, OptionEnumVariantsParsed)
{
  struct OptionCase
  {
    const char* mUri;
    const char* mAttributes;
    Integrator mIntegrator;
    CollisionType mCollision;
    ConeType mCone;
    JacobianType mJacobian;
    SolverType mSolver;
  };

  const OptionCase cases[] = {
      {"memory://mjcf_option_enum_first.xml",
       R"(integrator="Euler" collision="all" cone="elliptic"
          jacobian="dense" solver="PGS")",
       Integrator::EULER,
       CollisionType::ALL,
       ConeType::ELLIPTIC,
       JacobianType::DENSE,
       SolverType::PGS},
      {"memory://mjcf_option_enum_second.xml",
       R"(integrator="RK4" collision="predefined" cone="pyramidal"
          jacobian="auto" solver="CG")",
       Integrator::RK4,
       CollisionType::PREDEFINED,
       ConeType::PYRAMIDAL,
       JacobianType::AUTO,
       SolverType::CG},
  };

  for (const auto& optionCase : cases) {
    SCOPED_TRACE(optionCase.mUri);

    auto retriever = std::make_shared<MemoryResourceRetriever>();
    const std::string xml = std::string(R"(
<?xml version="1.0" ?>
<mujoco model="option_enum">
  <default>
    <geom type="sphere" size="0.1" />
  </default>
  <option )") + optionCase.mAttributes
                            + R"( />
  <worldbody>
    <body name="root">
      <geom type="sphere" size="0.1" />
    </body>
  </worldbody>
</mujoco>
)";
    const auto uri = addMemoryFile(*retriever, optionCase.mUri, xml);

    auto mujoco = utils::MjcfParser::detail::MujocoModel();
    const auto errors = mujoco.read(uri, retriever);
    ASSERT_TRUE(errors.empty());

    const auto& option = mujoco.getOption();
    EXPECT_EQ(option.getIntegrator(), optionCase.mIntegrator);
    EXPECT_EQ(option.getCollision(), optionCase.mCollision);
    EXPECT_EQ(option.getCone(), optionCase.mCone);
    EXPECT_EQ(option.getJacobian(), optionCase.mJacobian);
    EXPECT_EQ(option.getSolver(), optionCase.mSolver);
  }
}

//==============================================================================
TEST(MjcfParserTest, SiteOrientationConflictReportsError)
{
  struct OrientationConflictCase
  {
    const char* mUri;
    const char* mAttributes;
  };

  const OrientationConflictCase cases[] = {
      {"memory://mjcf_site_quat_euler_conflict.xml",
       R"(quat="1 0 0 0" euler="0 0 90")"},
      {"memory://mjcf_site_quat_axisangle_conflict.xml",
       R"(quat="1 0 0 0" axisangle="1 0 0 45")"},
      {"memory://mjcf_site_quat_xyaxes_conflict.xml",
       R"(quat="1 0 0 0" xyaxes="1 0 0 0 1 0")"},
      {"memory://mjcf_site_quat_zaxis_conflict.xml",
       R"(quat="1 0 0 0" zaxis="0 0 1")"},
  };

  for (const auto& conflictCase : cases) {
    SCOPED_TRACE(conflictCase.mUri);

    auto retriever = std::make_shared<MemoryResourceRetriever>();
    const std::string xml = std::string(R"(
<?xml version="1.0" ?>
<mujoco model="site_orientation_conflict">
  <default>
    <geom type="sphere" size="0.1" />
  </default>
  <worldbody>
    <body name="root">
      <geom type="sphere" size="0.1" />
      <site name="bad_site" type="box" size="0.1 0.2 0.3"
            )") + conflictCase.mAttributes
                            + R"( />
    </body>
  </worldbody>
</mujoco>
)";
    const auto uri = addMemoryFile(*retriever, conflictCase.mUri, xml);

    auto mujoco = utils::MjcfParser::detail::MujocoModel();
    const auto errors = mujoco.read(uri, retriever);

    ASSERT_FALSE(errors.empty());
    EXPECT_TRUE(hasErrorCode(errors, ErrorCode::ATTRIBUTE_CONFLICT));
  }
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
      <site name="cylinder_plain_site" type="cylinder" size="0.25 0.75"
            pos="2 0 0" />
      <site name="capsule_plain_site" type="capsule" size="0.12 0.7"
            pos="3 0 0" />
      <site name="xyaxes_site" type="box" size="0.2 0.3 0.4"
            pos="0 2 0" xyaxes="0 1 0 -1 0 0" />
      <site name="zaxis_site" type="box" size="0.2 0.3 0.4"
            pos="0 3 0" zaxis="0 1 0" />
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
  ASSERT_EQ(body.getNumSites(), 9u);

  const auto& cylinder = body.getSite(0);
  EXPECT_EQ(cylinder.getType(), GeomType::CYLINDER);
  EXPECT_NEAR(cylinder.getCylinderRadius(), 0.2, 1e-12);
  EXPECT_NEAR(cylinder.getCylinderHalfLength(), 1.0, 1e-12);
  EXPECT_NEAR(cylinder.getCylinderLength(), 2.0, 1e-12);
  EXPECT_EQ(cylinder.getGroup(), 2);
  EXPECT_TRUE(cylinder.getRGBA().isApprox(Eigen::Vector4d(0.1, 0.2, 0.3, 0.4)));

  const auto& capsule = body.getSite(1);
  EXPECT_EQ(capsule.getType(), GeomType::CAPSULE);
  EXPECT_NEAR(capsule.getCapsuleRadius(), 0.1, 1e-12);
  EXPECT_NEAR(capsule.getCapsuleHalfLength(), 0.5, 1e-12);
  EXPECT_NEAR(capsule.getCapsuleLength(), 1.0, 1e-12);

  const auto& ellipsoid = body.getSite(2);
  EXPECT_EQ(ellipsoid.getType(), GeomType::ELLIPSOID);
  EXPECT_TRUE(
      ellipsoid.getEllipsoidRadii().isApprox(Eigen::Vector3d(0.2, 0.3, 0.4)));
  EXPECT_TRUE(ellipsoid.getEllipsoidDiameters().isApprox(
      Eigen::Vector3d(0.4, 0.6, 0.8)));
  EXPECT_TRUE(ellipsoid.getRelativeTransform().translation().isApprox(
      Eigen::Vector3d(1, 2, 3)));

  const auto& box = body.getSite(3);
  EXPECT_EQ(box.getType(), GeomType::BOX);
  EXPECT_TRUE(box.getBoxSize().isApprox(Eigen::Vector3d(0.2, 0.4, 0.6)));
  EXPECT_TRUE(box.getRelativeTransform().translation().isApprox(
      Eigen::Vector3d(-1, -2, -3)));

  const auto& eulerSite = body.getSite(4);
  EXPECT_EQ(eulerSite.getType(), GeomType::BOX);
  EXPECT_TRUE(eulerSite.getSize().isApprox(Eigen::Vector3d(0.1, 0.1, 0.1)));
  EXPECT_TRUE(eulerSite.getRelativeTransform().translation().isApprox(
      Eigen::Vector3d(0, 1, 0)));

  const auto& cylinderPlain = body.getSite(5);
  EXPECT_EQ(cylinderPlain.getType(), GeomType::CYLINDER);
  EXPECT_NEAR(cylinderPlain.getCylinderRadius(), 0.25, 1e-12);
  EXPECT_NEAR(cylinderPlain.getCylinderHalfLength(), 0.75, 1e-12);
  EXPECT_NEAR(cylinderPlain.getCylinderLength(), 1.5, 1e-12);
  EXPECT_TRUE(cylinderPlain.getRelativeTransform().translation().isApprox(
      Eigen::Vector3d(2, 0, 0)));

  const auto& capsulePlain = body.getSite(6);
  EXPECT_EQ(capsulePlain.getType(), GeomType::CAPSULE);
  EXPECT_NEAR(capsulePlain.getCapsuleRadius(), 0.12, 1e-12);
  EXPECT_NEAR(capsulePlain.getCapsuleHalfLength(), 0.7, 1e-12);
  EXPECT_NEAR(capsulePlain.getCapsuleLength(), 1.4, 1e-12);
  EXPECT_TRUE(capsulePlain.getRelativeTransform().translation().isApprox(
      Eigen::Vector3d(3, 0, 0)));

  const auto& xyaxesSite = body.getSite(7);
  EXPECT_EQ(xyaxesSite.getType(), GeomType::BOX);
  EXPECT_TRUE(xyaxesSite.getRelativeTransform().translation().isApprox(
      Eigen::Vector3d(0, 2, 0)));
  EXPECT_TRUE(xyaxesSite.getRelativeTransform().linear().col(0).isApprox(
      Eigen::Vector3d(0, 1, 0)));
  EXPECT_TRUE(xyaxesSite.getRelativeTransform().linear().col(1).isApprox(
      Eigen::Vector3d(-1, 0, 0)));

  const auto& zaxisSite = body.getSite(8);
  EXPECT_EQ(zaxisSite.getType(), GeomType::BOX);
  EXPECT_TRUE(zaxisSite.getRelativeTransform().translation().isApprox(
      Eigen::Vector3d(0, 3, 0)));
  EXPECT_TRUE(zaxisSite.getRelativeTransform().linear().col(2).isApprox(
      Eigen::Vector3d(0, 1, 0)));

  auto mutableSite = box;
  Eigen::Isometry3d relative = Eigen::Isometry3d::Identity();
  relative.translation() = Eigen::Vector3d(4, 5, 6);
  mutableSite.setRelativeTransform(relative);
  EXPECT_TRUE(mutableSite.getRelativeTransform().isApprox(relative));

  Eigen::Isometry3d world = Eigen::Isometry3d::Identity();
  world.translation() = Eigen::Vector3d(-4, -5, -6);
  mutableSite.setWorldTransform(world);
  EXPECT_TRUE(mutableSite.getWorldTransform().isApprox(world));
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
    <weld body1="missing_body" body2="body2" />
    <weld body1="body1" body2="missing_body" />
    <weld body1="body1" body2="body1" />
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
  EXPECT_EQ(world->getConstraintSolver()->getNumConstraints(), 1u);
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
TEST(MjcfParserTest, InertialOrientationVariantsAndMutators)
{
  const std::string xml = R"(
<?xml version="1.0" ?>
<mujoco model="inertial_orientations">
  <default>
    <geom type="sphere" size="0.1" />
  </default>
  <worldbody>
    <body name="euler_body">
      <geom type="sphere" size="0.1" />
      <inertial pos="1 0 0" mass="1.0" diaginertia="1 2 3"
                euler="0 0 1.57079632679" />
    </body>
    <body name="xyaxes_body">
      <geom type="sphere" size="0.1" />
      <inertial pos="0 2 0" mass="2.0" diaginertia="2 3 4"
                xyaxes="0 1 0 -1 0 0" />
    </body>
    <body name="zaxis_body">
      <geom type="sphere" size="0.1" />
      <inertial pos="0 0 3" mass="3.0" diaginertia="3 4 5"
                zaxis="0 1 0" />
    </body>
  </worldbody>
</mujoco>
  )";

  const auto tempPath = std::filesystem::temp_directory_path()
                        / "dart_mjcf_inertial_orientations.xml";
  std::ofstream output(tempPath.string(), std::ios::binary);
  output << xml;
  output.close();

  auto mujoco = utils::MjcfParser::detail::MujocoModel();
  auto errors = mujoco.read(common::Uri(tempPath.string()), createRetriever());
  std::error_code ec;
  std::filesystem::remove(tempPath, ec);
  ASSERT_TRUE(errors.empty());

  const auto& worldbody = mujoco.getWorldbody();
  ASSERT_EQ(worldbody.getNumRootBodies(), 3u);

  const auto& eulerInertial = worldbody.getRootBody(0).getInertial();
  EXPECT_DOUBLE_EQ(eulerInertial.getMass(), 1.0);
  EXPECT_TRUE(eulerInertial.getRelativeTransform().translation().isApprox(
      Eigen::Vector3d(1, 0, 0)));
  EXPECT_TRUE(eulerInertial.getRelativeTransform().linear().allFinite());

  const auto& xyaxesInertial = worldbody.getRootBody(1).getInertial();
  EXPECT_DOUBLE_EQ(xyaxesInertial.getMass(), 2.0);
  EXPECT_TRUE(xyaxesInertial.getRelativeTransform().translation().isApprox(
      Eigen::Vector3d(0, 2, 0)));
  EXPECT_TRUE(xyaxesInertial.getRelativeTransform().linear().allFinite());

  const auto& zaxisInertial = worldbody.getRootBody(2).getInertial();
  EXPECT_DOUBLE_EQ(zaxisInertial.getMass(), 3.0);
  EXPECT_TRUE(zaxisInertial.getRelativeTransform().translation().isApprox(
      Eigen::Vector3d(0, 0, 3)));
  EXPECT_TRUE(zaxisInertial.getRelativeTransform().linear().allFinite());

  auto mutableInertial = eulerInertial;
  mutableInertial.setMass(4.0);
  mutableInertial.setDiagInertia(Eigen::Vector3d(4, 5, 6));
  mutableInertial.setOffDiagInertia(Eigen::Vector3d(0.1, 0.2, 0.3));

  Eigen::Isometry3d relative = Eigen::Isometry3d::Identity();
  relative.translation() = Eigen::Vector3d(4, 5, 6);
  mutableInertial.setRelativeTransform(relative);

  Eigen::Isometry3d world = Eigen::Isometry3d::Identity();
  world.translation() = Eigen::Vector3d(-4, -5, -6);
  mutableInertial.setWorldTransform(world);

  EXPECT_DOUBLE_EQ(mutableInertial.getMass(), 4.0);
  EXPECT_TRUE(
      mutableInertial.getDiagInertia().isApprox(Eigen::Vector3d(4, 5, 6)));
  EXPECT_TRUE(mutableInertial.getOffDiagInertia().isApprox(
      Eigen::Vector3d(0.1, 0.2, 0.3)));
  EXPECT_TRUE(mutableInertial.getRelativeTransform().isApprox(relative));
  EXPECT_TRUE(mutableInertial.getWorldTransform().isApprox(world));
}

//==============================================================================
TEST(MjcfParserTest, InertialRequiredAttributesReportErrors)
{
  struct InvalidInertialCase
  {
    const char* mUri;
    const char* mAttributes;
    ErrorCode mExpectedError;
  };

  const InvalidInertialCase cases[] = {
      {"memory://mjcf_inertial_missing_pos.xml",
       R"(mass="1" diaginertia="1 1 1")",
       ErrorCode::ATTRIBUTE_MISSING},
      {"memory://mjcf_inertial_missing_mass.xml",
       R"(pos="0 0 0" diaginertia="1 1 1")",
       ErrorCode::ATTRIBUTE_MISSING},
      {"memory://mjcf_inertial_missing_inertia.xml",
       R"(pos="0 0 0" mass="1")",
       ErrorCode::ATTRIBUTE_MISSING},
      {"memory://mjcf_inertial_conflicting_inertia.xml",
       R"(pos="0 0 0" mass="1" diaginertia="1 1 1"
          fullinertia="1 1 1 0 0 0")",
       ErrorCode::ATTRIBUTE_CONFLICT},
  };

  for (const auto& inertialCase : cases) {
    SCOPED_TRACE(inertialCase.mUri);

    auto retriever = std::make_shared<MemoryResourceRetriever>();
    const std::string xml = std::string(R"(
<?xml version="1.0" ?>
<mujoco model="bad_inertial">
  <default>
    <geom type="sphere" size="0.1" />
  </default>
  <worldbody>
    <body name="root">
      <geom type="sphere" size="0.1" />
      <inertial )") + inertialCase.mAttributes
                            + R"( />
    </body>
  </worldbody>
</mujoco>
)";
    const auto uri = addMemoryFile(*retriever, inertialCase.mUri, xml);

    auto mujoco = utils::MjcfParser::detail::MujocoModel();
    const auto errors = mujoco.read(uri, retriever);

    ASSERT_FALSE(errors.empty());
    EXPECT_TRUE(hasErrorCode(errors, inertialCase.mExpectedError));
  }
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
  std::ranges::replace(meshDirStr, '\\', '/');
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
      <body name="inertial_only">
        <inertial pos="0.25 0.5 0.75" mass="1.0" diaginertia="0.2 0.3 0.4" />
        <geom type="sphere" size="0.05" />
      </body>
      <body name="invalid_inertial_only">
        <inertial pos="nan 0 0" mass="1.0" diaginertia="0.2 0.3 0.4" />
        <geom type="sphere" size="0.05" />
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

  const auto* inertialOnlyBody = skel->getBodyNode("inertial_only");
  ASSERT_NE(inertialOnlyBody, nullptr);
  EXPECT_TRUE(inertialOnlyBody->getTransform().translation().isApprox(
      Eigen::Vector3d(0.25, 0.5, 0.75)));

  const auto* invalidInertialOnlyBody
      = skel->getBodyNode("invalid_inertial_only");
  ASSERT_NE(invalidInertialOnlyBody, nullptr);
  EXPECT_TRUE(invalidInertialOnlyBody->getTransform().isApprox(
      Eigen::Isometry3d::Identity()));

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
  std::ranges::replace(meshDirStr, '\\', '/');
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
    <site name="world_marker" type="box" size="0.05 0.06 0.07"
          pos="0.1 0.2 0.3" rgba="0.2 0.4 0.6 0.8" />
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

  const auto options = utils::MjcfParser::Options();
  const auto world = utils::MjcfParser::readWorld(
      common::Uri::createFromPath(xmlPath.string()), options);
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

  const auto worldMarkerSkel
      = world->getSkeleton(options.mSiteSkeletonNamePrefix + "world_marker");
  ASSERT_NE(worldMarkerSkel, nullptr);
  EXPECT_FALSE(worldMarkerSkel->isMobile());
  auto* worldMarkerShapeNode = worldMarkerSkel->getShapeNode(0);
  ASSERT_NE(worldMarkerShapeNode, nullptr);
  auto* worldMarkerVisualAspect = worldMarkerShapeNode->getVisualAspect();
  ASSERT_NE(worldMarkerVisualAspect, nullptr);
  EXPECT_TRUE(worldMarkerVisualAspect->getRGBA().isApprox(
      Eigen::Vector4d(0.2, 0.4, 0.6, 0.8)));
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
    <default class="world_defaults">
      <geom type="box" size="0.2 0.3 0.4" />
    </default>
  </default>
  <option timestep="0.01" apirate="60" impratio="2"
          gravity="0 0 -9.81" wind="0.1 0.2 0.3" magnetic="0 0 0"
          density="1000" viscosity="0.2" integrator="Euler"
          collision="all" cone="elliptic" jacobian="dense"
          solver="PGS" iterations="10" tolerance="1e-4" />
  <asset>
    <mesh name="coverage_mesh" file="dart_mjcf_coverage_mesh.obj" />
  </asset>
  <worldbody childclass="world_defaults">
    <light name="light0" pos="0 0 1" />
    <camera name="cam0" pos="0 0 2" />
    <geom name="ground" />
    <site name="world_site" type="box" size="0.05 0.06 0.07"
          pos="0.1 0.2 0.3" rgba="0.2 0.4 0.6 0.8" group="3" />
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
  ASSERT_EQ(worldbody.getNumGeoms(), 1u);
  const auto& ground = worldbody.getGeom(0);
  EXPECT_EQ(ground.getName(), "ground");
  EXPECT_EQ(ground.getType(), GeomType::BOX);
  EXPECT_TRUE(ground.getBoxHalfSize().isApprox(Eigen::Vector3d(0.2, 0.3, 0.4)));
  ASSERT_EQ(worldbody.getNumSites(), 1u);
  const auto& worldSite = worldbody.getSite(0);
  EXPECT_EQ(worldSite.getName(), "world_site");
  EXPECT_EQ(worldSite.getType(), GeomType::BOX);
  EXPECT_EQ(worldSite.getGroup(), 3);
  EXPECT_TRUE(
      worldSite.getBoxHalfSize().isApprox(Eigen::Vector3d(0.05, 0.06, 0.07)));
  EXPECT_TRUE(
      worldSite.getRGBA().isApprox(Eigen::Vector4d(0.2, 0.4, 0.6, 0.8)));
  EXPECT_TRUE(worldSite.getRelativeTransform().translation().isApprox(
      Eigen::Vector3d(0.1, 0.2, 0.3)));
  EXPECT_TRUE(worldSite.getWorldTransform().translation().isApprox(
      Eigen::Vector3d(0.1, 0.2, 0.3)));
  ASSERT_EQ(worldbody.getNumRootBodies(), 1u);
  const auto& rootBody = worldbody.getRootBody(0);
  EXPECT_EQ(rootBody.getName(), "root");
  EXPECT_FALSE(rootBody.getMocap());
  ASSERT_GE(rootBody.getNumGeoms(), 1u);
  ASSERT_EQ(rootBody.getNumSites(), 1u);
  const auto& sphereSite = rootBody.getSite(0);
  EXPECT_EQ(sphereSite.getType(), GeomType::SPHERE);
  ASSERT_EQ(rootBody.getNumChildBodies(), 1u);

  const auto& capsuleBody = rootBody.getChildBody(0);
  EXPECT_EQ(capsuleBody.getName(), "capsule_body");
  ASSERT_EQ(capsuleBody.getNumChildBodies(), 1u);
  const auto& cylinderBody = capsuleBody.getChildBody(0);
  EXPECT_EQ(cylinderBody.getName(), "cylinder_body");

  auto mutableBody = capsuleBody;
  Eigen::Isometry3d relative = Eigen::Isometry3d::Identity();
  relative.translation() = Eigen::Vector3d(1, 2, 3);
  mutableBody.setRelativeTransform(relative);
  EXPECT_TRUE(mutableBody.getRelativeTransform().isApprox(relative));

  Eigen::Isometry3d world = Eigen::Isometry3d::Identity();
  world.translation() = Eigen::Vector3d(4, 5, 6);
  mutableBody.setWorldTransform(world);
  EXPECT_TRUE(mutableBody.getWorldTransform().isApprox(world));

  Eigen::Isometry3d invalid = Eigen::Isometry3d::Identity();
  invalid.translation().x() = std::numeric_limits<double>::quiet_NaN();
  mutableBody.setRelativeTransform(invalid);
  EXPECT_TRUE(mutableBody.getRelativeTransform().isApprox(
      Eigen::Isometry3d::Identity()));
  mutableBody.setWorldTransform(invalid);
  EXPECT_TRUE(
      mutableBody.getWorldTransform().isApprox(Eigen::Isometry3d::Identity()));
}

//==============================================================================
TEST(MjcfParserTest, DefaultsNestedClassInheritance)
{
  const std::string xml = R"(
<?xml version="1.0" ?>
<mujoco model="nested_defaults">
  <default>
    <geom type="sphere" size="0.1" />
    <equality>
      <weld body1="root" body2="middle" active="false"
            solimp="0.91 0.82 0.003 0.4 1.2" solref="0.03 4"
            relpose="0.1 0.2 0.3 1 0 0 0" />
    </equality>
    <default class="outer">
      <geom type="box" size="0.2 0.3 0.4" rgba="0.1 0.2 0.3 0.4"
            friction="0.2 0.3 0.4" />
      <equality>
        <weld body1="root" body2="outer_child" solref="0.04 5" />
      </equality>
      <default class="inner">
        <geom size="0.5 0.6 0.7" rgba="0.9 0.8 0.7 0.6" />
        <equality>
          <weld body1="root" active="true"
                solimp="0.8 0.7 0.002 0.3 1.1"
                relpose="0.4 0.5 0.6 0.7071067812 0 0.7071067812 0" />
        </equality>
      </default>
    </default>
  </default>
  <worldbody>
    <body name="root">
      <geom class="inner" />
      <body name="middle">
        <geom type="sphere" size="0.1" />
      </body>
      <body name="outer_child">
        <geom type="sphere" size="0.1" />
      </body>
    </body>
  </worldbody>
  <equality>
    <weld name="root_default_weld" body1="root" />
    <weld name="inner_weld" class="inner" body1="root" />
  </equality>
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

  const auto& equality = mujoco.getEquality();
  ASSERT_EQ(equality.getNumWelds(), 2u);

  Eigen::Matrix<double, 5, 1> expectedRootSolImp;
  expectedRootSolImp << 0.91, 0.82, 0.003, 0.4, 1.2;

  const auto& rootWeld = equality.getWeld(0);
  EXPECT_EQ(rootWeld.getName(), "root_default_weld");
  EXPECT_FALSE(rootWeld.getActive());
  EXPECT_EQ(rootWeld.getBody1(), "root");
  EXPECT_EQ(rootWeld.getBody2(), "middle");
  EXPECT_TRUE(rootWeld.getSolRef().isApprox(Eigen::Vector2d(0.03, 4.0)));
  EXPECT_TRUE(rootWeld.getSolImp().isApprox(expectedRootSolImp));
  ASSERT_TRUE(rootWeld.getRelativeTransform());
  EXPECT_TRUE(rootWeld.getRelativeTransform()->translation().isApprox(
      Eigen::Vector3d(0.1, 0.2, 0.3)));

  Eigen::Matrix<double, 5, 1> expectedInnerSolImp;
  expectedInnerSolImp << 0.8, 0.7, 0.002, 0.3, 1.1;

  const auto& innerWeld = equality.getWeld(1);
  EXPECT_EQ(innerWeld.getName(), "inner_weld");
  EXPECT_TRUE(innerWeld.getActive());
  EXPECT_EQ(innerWeld.getBody1(), "root");
  EXPECT_EQ(innerWeld.getBody2(), "outer_child");
  EXPECT_TRUE(innerWeld.getSolRef().isApprox(Eigen::Vector2d(0.04, 5.0)));
  EXPECT_TRUE(innerWeld.getSolImp().isApprox(expectedInnerSolImp));
  ASSERT_TRUE(innerWeld.getRelativeTransform());
  EXPECT_TRUE(innerWeld.getRelativeTransform()->translation().isApprox(
      Eigen::Vector3d(0.4, 0.5, 0.6)));
}

//==============================================================================
TEST(MjcfParserTest, EqualityWeldInvalidAttributesReportErrors)
{
  struct InvalidWeldCase
  {
    const char* mUri;
    const char* mWeldElement;
    ErrorCode mExpectedError;
  };

  const InvalidWeldCase cases[] = {
      {"memory://mjcf_weld_invalid_solimp.xml",
       R"(<weld body1="root" body2="child" solimp="1 2 3 4 5 6" />)",
       ErrorCode::ATTRIBUTE_INVALID},
      {"memory://mjcf_weld_missing_body1.xml",
       R"(<weld body2="child" />)",
       ErrorCode::ATTRIBUTE_MISSING},
  };

  for (const auto& weldCase : cases) {
    SCOPED_TRACE(weldCase.mUri);

    const std::string xml = std::string(R"(
<?xml version="1.0" ?>
<mujoco model="invalid_weld">
  <default>
    <geom type="sphere" size="0.1" />
  </default>
  <worldbody>
    <body name="root">
      <geom type="sphere" size="0.1" />
      <body name="child">
        <geom type="sphere" size="0.1" />
      </body>
    </body>
  </worldbody>
  <equality>
    )") + weldCase.mWeldElement
                            + R"(
  </equality>
</mujoco>
)";

    auto retriever = std::make_shared<MemoryResourceRetriever>();
    const auto uri = addMemoryFile(*retriever, weldCase.mUri, xml);

    auto mujoco = utils::MjcfParser::detail::MujocoModel();
    const auto errors = mujoco.read(uri, retriever);
    ASSERT_FALSE(errors.empty());
    EXPECT_TRUE(hasErrorCode(errors, weldCase.mExpectedError));
  }
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
  EXPECT_TRUE(hasErrorCode(errors, ErrorCode::ATTRIBUTE_MISSING));
}

//==============================================================================
TEST(MjcfParserTest, BodyMissingChildClassReportsError)
{
  const std::string xml = R"(
<?xml version="1.0" ?>
<mujoco model="body_missing_childclass">
  <default>
    <geom type="sphere" size="0.1" />
  </default>
  <worldbody>
    <body name="root" childclass="missing">
      <geom type="sphere" size="0.1" />
    </body>
  </worldbody>
</mujoco>
  )";

  auto retriever = std::make_shared<MemoryResourceRetriever>();
  const auto uri
      = addMemoryFile(*retriever, "memory://mjcf_missing_childclass.xml", xml);

  auto mujoco = utils::MjcfParser::detail::MujocoModel();
  const auto errors = mujoco.read(uri, retriever);
  ASSERT_FALSE(errors.empty());
  EXPECT_TRUE(hasErrorCode(errors, ErrorCode::ATTRIBUTE_INVALID));
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
  EXPECT_TRUE(hasErrorCode(errors, ErrorCode::ATTRIBUTE_INVALID));
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
TEST(MjcfParserTest, ActuatorOmittedLimitedDefaultsToAuto)
{
  const std::string xml = R"(
<?xml version="1.0" ?>
<mujoco model="actuator_omitted_limited">
  <default>
    <geom type="sphere" size="0.1" />
  </default>
  <worldbody>
    <body name="b1">
      <geom type="sphere" size="0.1" />
      <joint name="j1" type="hinge" axis="0 0 1" />
    </body>
    <body name="b2" pos="1 0 0">
      <geom type="sphere" size="0.1" />
      <joint name="j2" type="hinge" axis="0 0 1" />
    </body>
    <body name="b3" pos="2 0 0">
      <geom type="sphere" size="0.1" />
      <joint name="j3" type="hinge" axis="0 0 1" />
    </body>
  </worldbody>
  <actuator>
    <motor joint="j1" forcerange="-8 8" />
    <motor joint="j2" ctrlrange="-3 3" />
    <motor joint="j3" />
  </actuator>
</mujoco>
)";
  const auto tempPath = std::filesystem::temp_directory_path()
                        / "dart_mjcf_actuator_omitted_limited.xml";
  std::ofstream output(tempPath.string(), std::ios::binary);
  output << xml;
  output.close();

  auto world = utils::MjcfParser::readWorld(common::Uri(tempPath.string()));
  std::error_code ec;
  std::filesystem::remove(tempPath, ec);
  ASSERT_NE(world, nullptr);

  // j1: forcerange set without forcelimited => auto => limits applied
  auto* joint1 = world->getSkeleton(0)->getJoint("j1");
  ASSERT_NE(joint1, nullptr);
  EXPECT_DOUBLE_EQ(joint1->getForceLowerLimit(0), -8.0);
  EXPECT_DOUBLE_EQ(joint1->getForceUpperLimit(0), 8.0);

  // j2: ctrlrange set without ctrllimited => auto => limits applied
  auto* joint2 = world->getSkeleton(1)->getJoint("j2");
  ASSERT_NE(joint2, nullptr);
  EXPECT_TRUE(joint2->getActuatorType() == dynamics::Joint::FORCE);

  // j3: no range and no attribute => auto resolves to false (no limits)
  auto* joint3 = world->getSkeleton(2)->getJoint("j3");
  ASSERT_NE(joint3, nullptr);
  EXPECT_DOUBLE_EQ(
      joint3->getForceLowerLimit(0), -std::numeric_limits<double>::infinity());
  EXPECT_DOUBLE_EQ(
      joint3->getForceUpperLimit(0), std::numeric_limits<double>::infinity());
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
    <exclude body1="body1" body2="missing_body" />
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
    <mesh name="missing_mesh" file="missing.stl" scale="1 2 3" />
    <texture name="grid" type="2d" builtin="checker" rgb1="0.8 0.8 0.8"
             rgb2="0.2 0.2 0.2" width="512" height="512"
             file="grid.png" />
    <material name="grid_mat" texture="grid" rgba="1 1 1 1"
              emission="0.1" specular="0.8" shininess="0.9"
              reflectance="0.2" texrepeat="2 3" texuniform="true" />
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
  ASSERT_EQ(asset.getNumMeshes(), 1u);
  const auto& mesh = asset.getMesh(0);
  EXPECT_EQ(mesh.getName(), "missing_mesh");
  EXPECT_EQ(mesh.getFile(), "missing.stl");
  EXPECT_TRUE(mesh.getScale().isApprox(Eigen::Vector3d(1, 2, 3)));
  EXPECT_EQ(mesh.getMeshShape(), nullptr);

  const auto* meshByName = asset.getMesh("missing_mesh");
  ASSERT_NE(meshByName, nullptr);
  EXPECT_EQ(meshByName->getName(), "missing_mesh");
  EXPECT_EQ(asset.getMesh("unknown_mesh"), nullptr);

  ASSERT_EQ(asset.getNumTextures(), 1u);
  EXPECT_EQ(asset.getTexture(0).getName(), "grid");
  EXPECT_EQ(asset.getTexture(0).getType(), "2d");
  EXPECT_EQ(asset.getTexture(0).getFile(), "grid.png");
  EXPECT_EQ(asset.getTexture(0).getBuiltin(), "checker");

  const auto* texByName = asset.getTexture("grid");
  ASSERT_NE(texByName, nullptr);
  EXPECT_EQ(texByName->getName(), "grid");
  EXPECT_EQ(asset.getTexture("missing_texture"), nullptr);

  ASSERT_EQ(asset.getNumMaterials(), 1u);
  EXPECT_EQ(asset.getMaterial(0).getName(), "grid_mat");
  EXPECT_EQ(asset.getMaterial(0).getTexture(), "grid");
  EXPECT_TRUE(
      asset.getMaterial(0).getRgba().isApprox(Eigen::Vector4d(1, 1, 1, 1)));
  EXPECT_DOUBLE_EQ(asset.getMaterial(0).getEmission(), 0.1);
  EXPECT_DOUBLE_EQ(asset.getMaterial(0).getSpecular(), 0.8);
  EXPECT_DOUBLE_EQ(asset.getMaterial(0).getShininess(), 0.9);
  EXPECT_DOUBLE_EQ(asset.getMaterial(0).getReflectance(), 0.2);

  const auto* matByName = asset.getMaterial("grid_mat");
  ASSERT_NE(matByName, nullptr);
  EXPECT_EQ(matByName->getName(), "grid_mat");
  EXPECT_EQ(asset.getMaterial("missing_material"), nullptr);
}

//==============================================================================
TEST(MjcfParserTest, MeshAttributesReaderHandlesValidAndInvalidElements)
{
  tinyxml2::XMLDocument validDocument;
  ASSERT_EQ(
      validDocument.Parse(
          R"(<mesh name="raw_mesh" file="raw.stl" scale="2 3 4" />)"),
      tinyxml2::XML_SUCCESS);
  auto* meshElement = validDocument.FirstChildElement("mesh");
  ASSERT_NE(meshElement, nullptr);

  MeshAttributes attributes;
  auto errors = appendMeshAttributes(attributes, meshElement);
  EXPECT_TRUE(errors.empty());
  ASSERT_TRUE(attributes.mName);
  EXPECT_EQ(*attributes.mName, "raw_mesh");
  ASSERT_TRUE(attributes.mFile);
  EXPECT_EQ(*attributes.mFile, "raw.stl");
  EXPECT_TRUE(attributes.mScale.isApprox(Eigen::Vector3d(2, 3, 4)));

  tinyxml2::XMLDocument invalidDocument;
  ASSERT_EQ(
      invalidDocument.Parse(R"(<geom name="not_mesh" />)"),
      tinyxml2::XML_SUCCESS);
  auto* invalidElement = invalidDocument.FirstChildElement("geom");
  ASSERT_NE(invalidElement, nullptr);

  errors = appendMeshAttributes(attributes, invalidElement);
  ASSERT_EQ(errors.size(), 1u);
  EXPECT_TRUE(errors.front());
  EXPECT_EQ(errors.front().getCode(), ErrorCode::INCORRECT_ELEMENT_TYPE);
  EXPECT_NE(errors.front().getMessage().find("<mesh>"), std::string::npos);

  EXPECT_FALSE(Error());
}

//==============================================================================
TEST(MjcfParserTest, RawDetailReadersHandleInvalidElementTypes)
{
  tinyxml2::XMLDocument document;
  auto* element = parseRootElement(document, R"(<geom name="wrong_type" />)");
  ASSERT_NE(element, nullptr);

  Default defaultsTemplate;
  auto errors = defaultsTemplate.read(element, nullptr);
  expectSingleErrorCode(errors, ErrorCode::INCORRECT_ELEMENT_TYPE);

  Defaults defaults;
  errors = defaults.read(element, nullptr);
  expectSingleErrorCode(errors, ErrorCode::INCORRECT_ELEMENT_TYPE);

  Texture texture;
  errors = texture.read(element);
  expectSingleErrorCode(errors, ErrorCode::INCORRECT_ELEMENT_TYPE);

  Material material;
  errors = material.read(element);
  expectSingleErrorCode(errors, ErrorCode::INCORRECT_ELEMENT_TYPE);

  Camera camera;
  errors = camera.read(element);
  expectSingleErrorCode(errors, ErrorCode::INCORRECT_ELEMENT_TYPE);

  Light light;
  errors = light.read(element);
  expectSingleErrorCode(errors, ErrorCode::INCORRECT_ELEMENT_TYPE);

  Mesh mesh;
  errors = mesh.read(element);
  expectSingleErrorCode(errors, ErrorCode::INCORRECT_ELEMENT_TYPE);

  Asset asset;
  errors = asset.read(element);
  expectSingleErrorCode(errors, ErrorCode::INCORRECT_ELEMENT_TYPE);

  Contact contact;
  errors = contact.read(element);
  expectSingleErrorCode(errors, ErrorCode::INCORRECT_ELEMENT_TYPE);

  Size size;
  errors = size.read(element);
  expectSingleErrorCode(errors, ErrorCode::INCORRECT_ELEMENT_TYPE);

  Equality equality;
  errors = equality.read(element, defaults);
  expectSingleErrorCode(errors, ErrorCode::INCORRECT_ELEMENT_TYPE);

  BodyAttributes bodyAttributes;
  errors = appendBodyAttributes(bodyAttributes, element, std::nullopt);
  expectSingleErrorCode(errors, ErrorCode::INCORRECT_ELEMENT_TYPE);

  JointAttributes jointAttributes;
  errors = appendJointAttributes(jointAttributes, element);
  expectSingleErrorCode(errors, ErrorCode::INCORRECT_ELEMENT_TYPE);

  WeldAttributes weldAttributes;
  errors = appendWeldAttributes(weldAttributes, element);
  expectSingleErrorCode(errors, ErrorCode::INCORRECT_ELEMENT_TYPE);

  tinyxml2::XMLDocument defaultDocument;
  auto* defaultElement
      = parseRootElement(defaultDocument, R"(<default class="root" />)");
  ASSERT_NE(defaultElement, nullptr);

  Defaults validDefaults;
  errors = validDefaults.read(defaultElement, nullptr);
  ASSERT_TRUE(errors.empty());
  const Default* rootDefault = validDefaults.getRootDefault();
  ASSERT_NE(rootDefault, nullptr);

  Body body;
  errors = body.read(element, std::nullopt, validDefaults, rootDefault);
  expectSingleErrorCode(errors, ErrorCode::INCORRECT_ELEMENT_TYPE);

  Site site;
  errors = site.read(element);
  expectSingleErrorCode(errors, ErrorCode::INCORRECT_ELEMENT_TYPE);

  Joint joint;
  errors
      = joint.read(element, validDefaults, rootDefault->getJointAttributes());
  expectSingleErrorCode(errors, ErrorCode::INCORRECT_ELEMENT_TYPE);

  Weld weld;
  errors = weld.read(element, validDefaults);
  expectSingleErrorCode(errors, ErrorCode::INCORRECT_ELEMENT_TYPE);

  Actuator actuator;
  errors = actuator.read(element, validDefaults);
  expectSingleErrorCode(errors, ErrorCode::INCORRECT_ELEMENT_TYPE);

  Worldbody worldbody;
  errors = worldbody.read(
      element,
      std::nullopt,
      validDefaults,
      rootDefault,
      common::Uri("memory://raw_invalid.xml"),
      createRetriever());
  expectSingleErrorCode(errors, ErrorCode::INCORRECT_ELEMENT_TYPE);

  MujocoModel mujoco;
  errors = mujoco.read(
      element, common::Uri("memory://raw_invalid.xml"), createRetriever());
  expectSingleErrorCode(errors, ErrorCode::INCORRECT_ELEMENT_TYPE);
}

//==============================================================================
TEST(MjcfParserTest, RawDetailReadersHandleDefaultsAndAssets)
{
  tinyxml2::XMLDocument defaultDocument;
  auto* defaultElement = parseRootElement(
      defaultDocument,
      R"(<default class="root">
           <mesh name="template_mesh" file="template.stl" scale="1 2 3" />
           <default class="scaled_child">
             <mesh scale="4 5 6" />
           </default>
         </default>)");
  ASSERT_NE(defaultElement, nullptr);

  Defaults defaults;
  auto errors = defaults.read(defaultElement, nullptr);
  ASSERT_TRUE(errors.empty());
  EXPECT_TRUE(defaults.hasDefault("root"));
  EXPECT_TRUE(defaults.hasDefault("scaled_child"));
  EXPECT_EQ(defaults.getDefault("missing"), nullptr);

  const auto* rootDefault = defaults.getRootDefault();
  ASSERT_NE(rootDefault, nullptr);
  const auto& rootMeshAttributes = rootDefault->getMeshAttributes();
  ASSERT_TRUE(rootMeshAttributes.mName);
  EXPECT_EQ(*rootMeshAttributes.mName, "template_mesh");
  ASSERT_TRUE(rootMeshAttributes.mFile);
  EXPECT_EQ(*rootMeshAttributes.mFile, "template.stl");
  EXPECT_TRUE(rootMeshAttributes.mScale.isApprox(Eigen::Vector3d(1, 2, 3)));

  const auto* childDefault = defaults.getDefault("scaled_child");
  ASSERT_NE(childDefault, nullptr);
  const auto& childMeshAttributes = childDefault->getMeshAttributes();
  ASSERT_TRUE(childMeshAttributes.mName);
  EXPECT_EQ(*childMeshAttributes.mName, "template_mesh");
  ASSERT_TRUE(childMeshAttributes.mFile);
  EXPECT_EQ(*childMeshAttributes.mFile, "template.stl");
  EXPECT_TRUE(childMeshAttributes.mScale.isApprox(Eigen::Vector3d(4, 5, 6)));

  tinyxml2::XMLDocument invalidDefaultDocument;
  auto* invalidDefaultElement = parseRootElement(
      invalidDefaultDocument,
      R"(<default class="invalid_geom">
           <geom type="unknown_geom_type" />
         </default>)");
  ASSERT_NE(invalidDefaultElement, nullptr);

  errors = defaults.read(invalidDefaultElement, nullptr);
  ASSERT_FALSE(errors.empty());
  EXPECT_TRUE(hasErrorCode(errors, ErrorCode::ATTRIBUTE_INVALID));

  tinyxml2::XMLDocument textureDocument;
  auto* textureElement = parseRootElement(
      textureDocument,
      R"(<texture name="grid" type="cube" file="grid.png"
                  builtin="checker" rgb1="0.1 0.2 0.3"
                  rgb2="0.4 0.5 0.6" width="128" height="256" />)");
  ASSERT_NE(textureElement, nullptr);

  Texture texture;
  errors = texture.read(textureElement);
  ASSERT_TRUE(errors.empty());
  EXPECT_EQ(texture.getName(), "grid");
  EXPECT_EQ(texture.getType(), "cube");
  EXPECT_EQ(texture.getFile(), "grid.png");
  EXPECT_EQ(texture.getBuiltin(), "checker");
  EXPECT_TRUE(texture.mRgb1.isApprox(Eigen::Vector3d(0.1, 0.2, 0.3)));
  EXPECT_TRUE(texture.mRgb2.isApprox(Eigen::Vector3d(0.4, 0.5, 0.6)));
  EXPECT_EQ(texture.mWidth, 128);
  EXPECT_EQ(texture.mHeight, 256);

  tinyxml2::XMLDocument materialDocument;
  auto* materialElement = parseRootElement(
      materialDocument,
      R"(<material name="mat" texture="grid" texrepeat="2 3"
                   texuniform="false" rgba="0.2 0.3 0.4 0.5"
                   emission="0.1" specular="0.2" shininess="0.3"
                   reflectance="0.4" />)");
  ASSERT_NE(materialElement, nullptr);

  Material material;
  errors = material.read(materialElement);
  ASSERT_TRUE(errors.empty());
  EXPECT_EQ(material.getName(), "mat");
  EXPECT_EQ(material.getTexture(), "grid");
  EXPECT_TRUE(material.mTexRepeat.isApprox(Eigen::Vector2d(2, 3)));
  EXPECT_FALSE(material.mTexUniform);
  EXPECT_TRUE(material.getRgba().isApprox(Eigen::Vector4d(0.2, 0.3, 0.4, 0.5)));
  EXPECT_DOUBLE_EQ(material.getEmission(), 0.1);
  EXPECT_DOUBLE_EQ(material.getSpecular(), 0.2);
  EXPECT_DOUBLE_EQ(material.getShininess(), 0.3);
  EXPECT_DOUBLE_EQ(material.getReflectance(), 0.4);
}

//==============================================================================
TEST(MjcfParserTest, RawDetailReadersHandleLifecycleHelpers)
{
  Compiler compiler;
  compiler.setBaseUri(common::Uri("memory://raw_lifecycle/model.xml"));
  compiler.setResourceRetriever(createRetriever());
  compiler.mMeshDir = "meshes";

  tinyxml2::XMLDocument assetDocument;
  auto* assetElement = parseRootElement(
      assetDocument,
      R"(<asset extra="ignored">
           <mesh name="raw_mesh" class="unused_class"
                 file="mesh.stl" scale="2 3 4" />
           <texture name="raw_texture" file="texture.png" />
           <material name="raw_material" texture="raw_texture" />
           <unknown />
         </asset>)");
  ASSERT_NE(assetElement, nullptr);

  warnUnknownElements(assetElement, {"mesh", "texture", "material"});
  warnUnknownAttributes(assetElement, {"known"});

  Asset asset;
  auto errors = asset.read(assetElement);
  ASSERT_TRUE(errors.empty());

  errors = asset.preprocess(compiler);
  ASSERT_TRUE(errors.empty());
  ASSERT_EQ(asset.getNumMeshes(), 1u);
  EXPECT_EQ(asset.getMesh(0).getName(), "raw_mesh");
  EXPECT_EQ(asset.getMesh(0).getFile(), "mesh.stl");
  EXPECT_TRUE(asset.getMesh(0).getScale().isApprox(Eigen::Vector3d(2, 3, 4)));

  errors = asset.compile(compiler);
  ASSERT_TRUE(errors.empty());
  ASSERT_NE(asset.getMesh("raw_mesh"), nullptr);
  ASSERT_NE(asset.getTexture("raw_texture"), nullptr);
  ASSERT_NE(asset.getMaterial("raw_material"), nullptr);

  errors = asset.postprocess(compiler);
  ASSERT_TRUE(errors.empty());

  tinyxml2::XMLDocument lightDocument;
  auto* lightElement = parseRootElement(
      lightDocument, R"(<light directional="false" castshadow="false" />)");
  ASSERT_NE(lightElement, nullptr);

  Light light;
  errors = light.read(lightElement);
  ASSERT_TRUE(errors.empty());
  EXPECT_FALSE(light.getDirectional());
  EXPECT_FALSE(light.mCastshadow);

  tinyxml2::XMLDocument boxSiteDocument;
  auto* boxSiteElement = parseRootElement(
      boxSiteDocument,
      R"(<site name="box_site" type="box" size="1 2"
               fromto="0 0 0 0 0 4" />)");
  ASSERT_NE(boxSiteElement, nullptr);

  Site boxSite;
  errors = boxSite.read(boxSiteElement);
  ASSERT_TRUE(errors.empty());
  errors = boxSite.preprocess(compiler);
  ASSERT_TRUE(errors.empty());
  errors = boxSite.postprocess(nullptr, compiler);
  ASSERT_TRUE(errors.empty());
  EXPECT_EQ(boxSite.getName(), "box_site");
  EXPECT_TRUE(boxSite.getBoxHalfSize().isApprox(Eigen::Vector3d(1, 2, 2)));
  EXPECT_TRUE(boxSite.getWorldTransform().translation().isApprox(
      Eigen::Vector3d(0, 0, 2)));

  tinyxml2::XMLDocument sphereSiteDocument;
  auto* sphereSiteElement = parseRootElement(
      sphereSiteDocument,
      R"(<site name="sphere_site" type="sphere" size="0.25" pos="1 2 3"
               fromto="0 0 0 0 0 4" />)");
  ASSERT_NE(sphereSiteElement, nullptr);

  Site sphereSite;
  errors = sphereSite.read(sphereSiteElement);
  ASSERT_TRUE(errors.empty());
  errors = sphereSite.preprocess(compiler);
  ASSERT_TRUE(errors.empty());
  EXPECT_EQ(sphereSite.getName(), "sphere_site");
  EXPECT_DOUBLE_EQ(sphereSite.getSphereRadius(), 0.25);
  EXPECT_TRUE(sphereSite.getRelativeTransform().translation().isApprox(
      Eigen::Vector3d(1, 2, 3)));
  EXPECT_DOUBLE_EQ(sphereSite.computeVolume(), 1.0);
  EXPECT_TRUE(
      sphereSite.computeInertia().isApprox(Eigen::Matrix3d::Identity()));

  Compiler zyxCompiler;
  zyxCompiler.mEulerSeq = "zyx";
  const Eigen::Vector3d invalidEuler{
      std::numeric_limits<double>::infinity(), 0.0, 0.0};
  EXPECT_TRUE(compileRotation(
                  Eigen::Quaterniond::Identity(),
                  std::nullopt,
                  invalidEuler,
                  std::nullopt,
                  std::nullopt,
                  zyxCompiler)
                  .isIdentity());

  Compiler invalidSeqCompiler;
  invalidSeqCompiler.mEulerSeq = "unsupported";
  EXPECT_TRUE(compileRotation(
                  Eigen::Quaterniond::Identity(),
                  std::nullopt,
                  Eigen::Vector3d::Zero(),
                  std::nullopt,
                  std::nullopt,
                  invalidSeqCompiler)
                  .isIdentity());
}

//==============================================================================
TEST(MjcfParserTest, RawBodyAttributesHandleUserDataSizing)
{
  tinyxml2::XMLDocument sizeDocument;
  auto* sizeElement = parseRootElement(
      sizeDocument,
      R"(<size mjmax="10" nconmax="20" nstack="30" nuserdata="40"
               nkey="2" nuser_body="2" nuser_jnt="3" nuser_geom="4"
               nuser_site="5" nuser_cam="6" nuser_tendon="7"
               nuser_actuator="8" nuser_sensor="9" />)");
  ASSERT_NE(sizeElement, nullptr);

  Size size;
  auto errors = size.read(sizeElement);
  ASSERT_TRUE(errors.empty());
  EXPECT_EQ(size.getMJMax(), 10);
  EXPECT_EQ(size.getNConMax(), 20);
  EXPECT_EQ(size.getNStack(), 30);
  EXPECT_EQ(size.getNUserData(), 40);
  EXPECT_EQ(size.getNKey(), 2);
  EXPECT_EQ(size.getNUserBody(), 2);
  EXPECT_EQ(size.getNUserJnt(), 3);
  EXPECT_EQ(size.getNUserGeom(), 4);
  EXPECT_EQ(size.getNUserSite(), 5);
  EXPECT_EQ(size.getNUserCam(), 6);
  EXPECT_EQ(size.getNUserTendon(), 7);
  EXPECT_EQ(size.getNUserActuator(), 8);
  EXPECT_EQ(size.getNUserSensor(), 9);

  tinyxml2::XMLDocument bodyDocument;
  auto* bodyElement = parseRootElement(
      bodyDocument,
      R"(<body name="raw_body" mocap="true" pos="1 2 3"
                quat="1 0 0 0" user="4 5" />)");
  ASSERT_NE(bodyElement, nullptr);

  BodyAttributes bodyAttributes;
  errors = appendBodyAttributes(bodyAttributes, bodyElement, size);
  ASSERT_TRUE(errors.empty());
  ASSERT_TRUE(bodyAttributes.mName);
  EXPECT_EQ(*bodyAttributes.mName, "raw_body");
  EXPECT_TRUE(bodyAttributes.mMocap);
  ASSERT_TRUE(bodyAttributes.mPos);
  EXPECT_TRUE(bodyAttributes.mPos->isApprox(Eigen::Vector3d(1, 2, 3)));
  EXPECT_DOUBLE_EQ(bodyAttributes.mQuat.w(), 1.0);
  EXPECT_TRUE(bodyAttributes.mUser.isApprox(Eigen::Vector2d(4, 5)));

  tinyxml2::XMLDocument missingSizeDocument;
  auto* missingSizeElement
      = parseRootElement(missingSizeDocument, R"(<body user="1 2" />)");
  ASSERT_NE(missingSizeElement, nullptr);

  BodyAttributes missingSizeAttributes;
  errors = appendBodyAttributes(missingSizeAttributes, missingSizeElement, {});
  ASSERT_EQ(errors.size(), 1u);
  EXPECT_EQ(errors.front().getCode(), ErrorCode::ATTRIBUTE_INVALID);

  tinyxml2::XMLDocument noUserDocument;
  auto* noUserElement = parseRootElement(noUserDocument, R"(<body />)");
  ASSERT_NE(noUserElement, nullptr);

  BodyAttributes noUserAttributes;
  errors = appendBodyAttributes(noUserAttributes, noUserElement, {});
  ASSERT_TRUE(errors.empty());
  EXPECT_EQ(noUserAttributes.mUser.size(), 0);
}

//==============================================================================
TEST(MjcfParserTest, AssetMaterialInvalidBooleanReportsError)
{
  const std::string xml = R"(
<?xml version="1.0" ?>
<mujoco model="asset_bad_material">
  <default>
    <geom type="sphere" size="0.1" />
  </default>
  <asset>
    <material name="bad_mat" texuniform="maybe" />
  </asset>
  <worldbody>
    <body name="root">
      <geom type="sphere" size="0.1" />
    </body>
  </worldbody>
</mujoco>
)";
  const auto tempPath
      = std::filesystem::temp_directory_path() / "dart_mjcf_asset_bad_mat.xml";
  std::ofstream output(tempPath.string(), std::ios::binary);
  output << xml;
  output.close();

  auto mujoco = utils::MjcfParser::detail::MujocoModel();
  const auto errors
      = mujoco.read(common::Uri(tempPath.string()), createRetriever());
  std::error_code ec;
  std::filesystem::remove(tempPath, ec);

  ASSERT_FALSE(errors.empty());
  EXPECT_TRUE(hasErrorCode(errors, ErrorCode::ATTRIBUTE_INVALID));
  EXPECT_EQ(mujoco.getAsset().getNumMaterials(), 0u);
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
  EXPECT_DOUBLE_EQ(joint->getForceLowerLimit(0), -150.0);
  EXPECT_DOUBLE_EQ(joint->getForceUpperLimit(0), 150.0);
}

//==============================================================================
TEST(MjcfParserTest, ActuatorDefaultsApplyPerActuatorType)
{
  const std::string xml = R"(
<?xml version="1.0" ?>
<mujoco model="actuator_defaults_by_type">
  <default>
    <geom type="sphere" size="0.1" />
    <motor ctrllimited="true" ctrlrange="-1 1"
           forcelimited="true" forcerange="-10 10" gear="2" />
    <position ctrllimited="true" ctrlrange="-2 2"
              forcelimited="true" forcerange="-20 20" gear="3" />
    <velocity ctrlrange="-3 3" forcerange="-30 30" gear="4" />
    <general ctrlrange="-4 4" forcerange="-40 40" gear="5"
             gainprm="1 2 3" biasprm="4 5 6" />
  </default>
  <worldbody>
    <body name="root">
      <geom type="sphere" size="0.1" />
      <joint name="j_motor" type="hinge" axis="0 0 1" />
      <joint name="j_position" type="hinge" axis="0 1 0" />
      <joint name="j_velocity" type="hinge" axis="1 0 0" />
      <joint name="j_general" type="hinge" axis="1 1 0" />
    </body>
  </worldbody>
  <actuator>
    <motor joint="j_motor" />
    <position joint="j_position" />
    <velocity joint="j_velocity" />
    <general joint="j_general" />
  </actuator>
</mujoco>
)";

  auto retriever = std::make_shared<MemoryResourceRetriever>();
  const auto uri = addMemoryFile(
      *retriever, "memory://mjcf_actuator_defaults_by_type.xml", xml);

  auto mujoco = utils::MjcfParser::detail::MujocoModel();
  const auto errors = mujoco.read(uri, retriever);
  ASSERT_TRUE(errors.empty());

  const auto& actuator = mujoco.getActuator();
  ASSERT_EQ(actuator.getNumEntries(), 4u);

  const auto& motor = actuator.getEntry(0);
  EXPECT_EQ(motor.mType, ActuatorType::MOTOR);
  EXPECT_EQ(motor.mJoint, "j_motor");
  EXPECT_TRUE(motor.mCtrlLimited);
  EXPECT_TRUE(motor.mForceLimited);
  EXPECT_TRUE(motor.mCtrlRange.isApprox(Eigen::Vector2d(-1, 1)));
  EXPECT_TRUE(motor.mForceRange.isApprox(Eigen::Vector2d(-10, 10)));
  EXPECT_TRUE(motor.mGear.isApprox(Eigen::Vector6d(2, 0, 0, 0, 0, 0)));

  const auto& position = actuator.getEntry(1);
  EXPECT_EQ(position.mType, ActuatorType::POSITION);
  EXPECT_EQ(position.mJoint, "j_position");
  EXPECT_TRUE(position.mCtrlLimited);
  EXPECT_TRUE(position.mForceLimited);
  EXPECT_TRUE(position.mCtrlRange.isApprox(Eigen::Vector2d(-2, 2)));
  EXPECT_TRUE(position.mForceRange.isApprox(Eigen::Vector2d(-20, 20)));
  EXPECT_TRUE(position.mGear.isApprox(Eigen::Vector6d(3, 0, 0, 0, 0, 0)));

  const auto& velocity = actuator.getEntry(2);
  EXPECT_EQ(velocity.mType, ActuatorType::VELOCITY);
  EXPECT_EQ(velocity.mJoint, "j_velocity");
  EXPECT_TRUE(velocity.mCtrlLimited);
  EXPECT_TRUE(velocity.mForceLimited);
  EXPECT_TRUE(velocity.mCtrlRange.isApprox(Eigen::Vector2d(-3, 3)));
  EXPECT_TRUE(velocity.mForceRange.isApprox(Eigen::Vector2d(-30, 30)));
  EXPECT_TRUE(velocity.mGear.isApprox(Eigen::Vector6d(4, 0, 0, 0, 0, 0)));

  const auto& general = actuator.getEntry(3);
  EXPECT_EQ(general.mType, ActuatorType::GENERAL);
  EXPECT_EQ(general.mJoint, "j_general");
  EXPECT_TRUE(general.mCtrlLimited);
  EXPECT_TRUE(general.mForceLimited);
  EXPECT_TRUE(general.mCtrlRange.isApprox(Eigen::Vector2d(-4, 4)));
  EXPECT_TRUE(general.mForceRange.isApprox(Eigen::Vector2d(-40, 40)));
  EXPECT_TRUE(general.mGear.isApprox(Eigen::Vector6d(5, 0, 0, 0, 0, 0)));
  EXPECT_TRUE(general.mGainPrm.isApprox(Eigen::Vector3d(1, 2, 3)));
  EXPECT_TRUE(general.mBiasPrm.isApprox(Eigen::Vector3d(4, 5, 6)));
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
    <pair geom1="geom1" geom2="geom2" condim="4"
          friction="1 0.5 0.1" margin="0.001" gap="0.01" />
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
  ASSERT_EQ(pair.mFriction.size(), 3);
  EXPECT_TRUE(pair.mFriction.isApprox(Eigen::Vector3d(1, 0.5, 0.1)));
  EXPECT_DOUBLE_EQ(pair.mMargin, 0.001);
  EXPECT_DOUBLE_EQ(pair.mGap, 0.01);
}

//==============================================================================
TEST(MjcfParserTest, ContactMissingRequiredAttributesReportErrors)
{
  const std::string xml = R"(
<?xml version="1.0" ?>
<mujoco model="contact_missing_attrs">
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
    <pair geom1="geom1" />
    <pair geom2="geom2" />
    <exclude body1="body1" />
    <exclude body2="body2" />
  </contact>
</mujoco>
 )";
  const auto tempPath = std::filesystem::temp_directory_path()
                        / "dart_mjcf_contact_missing_attrs.xml";
  std::ofstream output(tempPath.string(), std::ios::binary);
  output << xml;
  output.close();

  auto mujoco = utils::MjcfParser::detail::MujocoModel();
  const auto errors
      = mujoco.read(common::Uri(tempPath.string()), createRetriever());
  std::error_code ec;
  std::filesystem::remove(tempPath, ec);

  ASSERT_FALSE(errors.empty());
  EXPECT_TRUE(hasErrorCode(errors, ErrorCode::ATTRIBUTE_MISSING));

  const auto& contact = mujoco.getContact();
  EXPECT_EQ(contact.getNumPairs(), 2u);
  EXPECT_EQ(contact.getNumExcludes(), 2u);
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
              mode="targetbody" fovy="60" quat="1 0 0 0" />
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
  EXPECT_EQ(camera.getName(), "cam_targeted");
  EXPECT_DOUBLE_EQ(camera.getFovy(), 60.0);
  EXPECT_TRUE(camera.getPos().isApprox(Eigen::Vector3d(1, 0, 0)));
  EXPECT_EQ(camera.getMode(), "targetbody");
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
      <light name="directional_light" active="true" directional="true"
             castshadow="true" pos="1 2 3" dir="0 1 0"
             diffuse="0.1 0.2 0.3" specular="0.4 0.5 0.6" />
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
  ASSERT_EQ(body.getNumLights(), 2u);

  const auto& inactiveLight = body.getLight(0);
  EXPECT_EQ(inactiveLight.getName(), "inactive_light");
  EXPECT_FALSE(inactiveLight.getActive());
  EXPECT_FALSE(inactiveLight.getDirectional());
  EXPECT_TRUE(inactiveLight.getPos().isApprox(Eigen::Vector3d(0, 0, 3)));
  EXPECT_TRUE(inactiveLight.getDir().isApprox(Eigen::Vector3d(0, 0, -1)));

  const auto& directionalLight = body.getLight(1);
  EXPECT_EQ(directionalLight.getName(), "directional_light");
  EXPECT_TRUE(directionalLight.getActive());
  EXPECT_TRUE(directionalLight.getDirectional());
  EXPECT_TRUE(directionalLight.getPos().isApprox(Eigen::Vector3d(1, 2, 3)));
  EXPECT_TRUE(directionalLight.getDir().isApprox(Eigen::Vector3d(0, 1, 0)));
  EXPECT_TRUE(
      directionalLight.getDiffuse().isApprox(Eigen::Vector3d(0.1, 0.2, 0.3)));
  EXPECT_TRUE(
      directionalLight.getSpecular().isApprox(Eigen::Vector3d(0.4, 0.5, 0.6)));
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
  EXPECT_TRUE(hasErrorCode(errors, ErrorCode::ELEMENT_INVALID));
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
TEST(MjcfParserTest, UnnamedJointsGetContextualDefaultNames)
{
  const std::string xml = std::string(R"(
 <?xml version="1.0" ?>
 <mujoco model="root_unnamed_joint">
   <default>
     <geom type="sphere" size="0.1" />
   </default>
   <!-- )") + kMjcfPadding + R"( -->
   <worldbody>
     <body name="parent">
       <geom type="sphere" size="0.1" />
       <joint type="hinge" range="-1 1" />
       <body name="child" pos="0 0 1">
         <geom type="sphere" size="0.05" />
         <joint type="hinge" axis="0 1 0" range="-1 1" />
       </body>
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
  EXPECT_NE(skel->getJoint("root_Joint"), nullptr);

  auto* childBody = skel->getBodyNode("child");
  ASSERT_NE(childBody, nullptr);
  auto* childJoint = childBody->getParentJoint();
  ASSERT_NE(childJoint, nullptr);
  EXPECT_EQ(childJoint->getName(), "parent_Joint");
  EXPECT_EQ(skel->getJoint("parent_Joint"), childJoint);
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
TEST(MjcfParserTest, ActuatorOverwriteAndMissingTargetKeepValidJointState)
{
  const std::string xml = R"(
<?xml version="1.0" ?>
<mujoco model="actuator_overwrite">
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
    <motor joint="hinge1" forcelimited="true" forcerange="-10 10" />
    <velocity joint="hinge1" />
    <motor name="missing_target" joint="missing_joint" />
  </actuator>
</mujoco>
)";
  const auto tempPath = std::filesystem::temp_directory_path()
                        / "dart_mjcf_actuator_overwrite.xml";
  std::ofstream output(tempPath.string(), std::ios::binary);
  output << xml;
  output.close();

  auto world = utils::MjcfParser::readWorld(common::Uri(tempPath.string()));
  std::error_code ec;
  std::filesystem::remove(tempPath, ec);
  ASSERT_NE(world, nullptr);

  auto* joint = world->getSkeleton(0)->getJoint("hinge1");
  ASSERT_NE(joint, nullptr);
  EXPECT_EQ(joint->getActuatorType(), dynamics::Joint::VELOCITY);
  EXPECT_DOUBLE_EQ(
      joint->getForceLowerLimit(0), -std::numeric_limits<double>::infinity());
  EXPECT_DOUBLE_EQ(
      joint->getForceUpperLimit(0), std::numeric_limits<double>::infinity());
}

//==============================================================================
TEST(MjcfParserTest, ActuatorVectorGearScalesForceByFirstElement)
{
  const std::string xml = R"(
<?xml version="1.0" ?>
<mujoco model="actuator_vector_gear">
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
    <motor joint="hinge1" gear="5 0 0 0 0 0"
           forcelimited="true" forcerange="-10 10" />
  </actuator>
</mujoco>
)";
  const auto tempPath
      = std::filesystem::temp_directory_path() / "dart_mjcf_vector_gear.xml";
  std::ofstream output(tempPath.string(), std::ios::binary);
  output << xml;
  output.close();

  auto world = utils::MjcfParser::readWorld(common::Uri(tempPath.string()));
  std::error_code ec;
  std::filesystem::remove(tempPath, ec);
  ASSERT_NE(world, nullptr);

  auto* joint = world->getSkeleton(0)->getJoint("hinge1");
  ASSERT_NE(joint, nullptr);
  EXPECT_DOUBLE_EQ(joint->getForceLowerLimit(0), -50.0);
  EXPECT_DOUBLE_EQ(joint->getForceUpperLimit(0), 50.0);
}

//==============================================================================
TEST(MjcfParserTest, ActuatorPerDofGearScaling)
{
  const std::string xml = R"(
<?xml version="1.0" ?>
<mujoco model="actuator_per_dof_gear">
  <default>
    <geom type="sphere" size="0.1" />
  </default>
  <worldbody>
    <body name="root">
      <geom type="sphere" size="0.1" />
      <joint name="ball1" type="ball" />
    </body>
  </worldbody>
  <actuator>
    <motor joint="ball1" gear="2 3 4"
           forcelimited="true" forcerange="-10 10" />
  </actuator>
</mujoco>
)";
  const auto tempPath
      = std::filesystem::temp_directory_path() / "dart_mjcf_per_dof_gear.xml";
  std::ofstream output(tempPath.string(), std::ios::binary);
  output << xml;
  output.close();

  auto world = utils::MjcfParser::readWorld(common::Uri(tempPath.string()));
  std::error_code ec;
  std::filesystem::remove(tempPath, ec);
  ASSERT_NE(world, nullptr);

  auto* joint = world->getSkeleton(0)->getJoint("ball1");
  ASSERT_NE(joint, nullptr);
  ASSERT_EQ(joint->getNumDofs(), 3u);
  EXPECT_DOUBLE_EQ(joint->getForceLowerLimit(0), -20.0);
  EXPECT_DOUBLE_EQ(joint->getForceUpperLimit(0), 20.0);
  EXPECT_DOUBLE_EQ(joint->getForceLowerLimit(1), -30.0);
  EXPECT_DOUBLE_EQ(joint->getForceUpperLimit(1), 30.0);
  EXPECT_DOUBLE_EQ(joint->getForceLowerLimit(2), -40.0);
  EXPECT_DOUBLE_EQ(joint->getForceUpperLimit(2), 40.0);
}

//==============================================================================
TEST(MjcfParserTest, DefaultActuatorAttributesInherited)
{
  const std::string xml = R"(
<?xml version="1.0" ?>
<mujoco model="default_actuator">
  <default>
    <motor ctrllimited="true" ctrlrange="-1 1"
           forcelimited="true" forcerange="-50 50" gear="3" />
    <geom type="sphere" size="0.1" />
    <default class="strong">
      <motor forcelimited="true" forcerange="-2 2" gear="4" />
    </default>
  </default>
  <worldbody>
    <body name="b1">
      <geom type="sphere" size="0.1" />
      <joint name="hinge1" type="hinge" axis="0 0 1" />
      <body name="b2">
        <geom type="sphere" size="0.1" />
        <joint name="hinge2" type="hinge" axis="0 1 0" />
      </body>
      <body name="b3">
        <geom type="sphere" size="0.1" />
        <joint name="hinge3" type="hinge" axis="1 0 0" />
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor joint="hinge1" />
    <motor joint="hinge2" gear="5" forcelimited="false" />
    <motor class="strong" joint="hinge3" />
  </actuator>
</mujoco>
)";
  const auto tempPath = std::filesystem::temp_directory_path()
                        / "dart_mjcf_default_actuator.xml";
  std::ofstream output(tempPath.string(), std::ios::binary);
  output << xml;
  output.close();

  auto world = utils::MjcfParser::readWorld(common::Uri(tempPath.string()));
  std::error_code ec;
  std::filesystem::remove(tempPath, ec);
  ASSERT_NE(world, nullptr);

  auto* j1 = world->getSkeleton(0)->getJoint("hinge1");
  ASSERT_NE(j1, nullptr);
  EXPECT_EQ(j1->getActuatorType(), dynamics::Joint::FORCE);
  EXPECT_DOUBLE_EQ(j1->getForceLowerLimit(0), -150.0);
  EXPECT_DOUBLE_EQ(j1->getForceUpperLimit(0), 150.0);

  auto* j2 = world->getSkeleton(0)->getJoint("hinge2");
  ASSERT_NE(j2, nullptr);
  EXPECT_EQ(j2->getActuatorType(), dynamics::Joint::FORCE);
  EXPECT_DOUBLE_EQ(
      j2->getForceLowerLimit(0), -std::numeric_limits<double>::infinity());
  EXPECT_DOUBLE_EQ(
      j2->getForceUpperLimit(0), std::numeric_limits<double>::infinity());

  auto* j3 = world->getSkeleton(0)->getJoint("hinge3");
  ASSERT_NE(j3, nullptr);
  EXPECT_EQ(j3->getActuatorType(), dynamics::Joint::FORCE);
  EXPECT_DOUBLE_EQ(j3->getForceLowerLimit(0), -8.0);
  EXPECT_DOUBLE_EQ(j3->getForceUpperLimit(0), 8.0);
}

//==============================================================================
TEST(MjcfParserTest, ActuatorInvalidAttributesReportErrors)
{
  struct InvalidActuatorCase
  {
    const char* mUri;
    const char* mActuatorElement;
    ErrorCode mExpectedError;
  };

  const InvalidActuatorCase cases[] = {
      {"memory://mjcf_actuator_invalid_ctrllimited.xml",
       R"(<motor joint="hinge1" ctrllimited="sometimes" />)",
       ErrorCode::ATTRIBUTE_INVALID},
      {"memory://mjcf_actuator_invalid_forcelimited.xml",
       R"(<motor joint="hinge1" forcelimited="sometimes" />)",
       ErrorCode::ATTRIBUTE_INVALID},
      {"memory://mjcf_actuator_missing_joint.xml",
       R"(<motor />)",
       ErrorCode::ATTRIBUTE_MISSING},
  };

  for (const auto& actuatorCase : cases) {
    SCOPED_TRACE(actuatorCase.mUri);

    const std::string xml = std::string(R"(
<?xml version="1.0" ?>
<mujoco model="invalid_actuator">
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
    )") + actuatorCase.mActuatorElement
                            + R"(
  </actuator>
</mujoco>
)";

    auto retriever = std::make_shared<MemoryResourceRetriever>();
    const auto uri = addMemoryFile(*retriever, actuatorCase.mUri, xml);

    auto mujoco = utils::MjcfParser::detail::MujocoModel();
    const auto errors = mujoco.read(uri, retriever);
    ASSERT_FALSE(errors.empty());
    EXPECT_TRUE(hasErrorCode(errors, actuatorCase.mExpectedError));
  }
}

//==============================================================================
TEST(MjcfParserTest, ActuatorUnknownClassEmitsError)
{
  const std::string xml = R"(
<?xml version="1.0" ?>
<mujoco model="unknown_class">
  <default>
    <geom type="sphere" size="0.1" />
  </default>
  <worldbody>
    <body name="b1">
      <geom type="sphere" size="0.1" />
      <joint name="hinge1" type="hinge" axis="0 0 1" />
    </body>
  </worldbody>
  <actuator>
    <motor class="nonexistent" joint="hinge1" />
  </actuator>
</mujoco>
)";
  const auto tempPath = std::filesystem::temp_directory_path()
                        / "dart_mjcf_unknown_actuator_class.xml";
  std::ofstream output(tempPath.string(), std::ios::binary);
  output << xml;
  output.close();

  auto mujoco = utils::MjcfParser::detail::MujocoModel();
  auto errors = mujoco.read(common::Uri(tempPath.string()), createRetriever());
  std::error_code ec;
  std::filesystem::remove(tempPath, ec);

  EXPECT_TRUE(hasErrorCode(errors, ErrorCode::ATTRIBUTE_INVALID));
}

//==============================================================================
TEST(MjcfParserTest, DefaultCtrlLimitedPreservedOverAutoInfer)
{
  const std::string xml = R"(
<?xml version="1.0" ?>
<mujoco model="ctrl_default_preserved">
  <default>
    <motor ctrllimited="false" ctrlrange="-1 1"
           forcelimited="false" forcerange="-50 50" gear="1" />
    <geom type="sphere" size="0.1" />
  </default>
  <worldbody>
    <body name="b1">
      <geom type="sphere" size="0.1" />
      <joint name="hinge1" type="hinge" axis="0 0 1" />
    </body>
  </worldbody>
  <actuator>
    <motor joint="hinge1" />
  </actuator>
</mujoco>
)";
  const auto tempPath = std::filesystem::temp_directory_path()
                        / "dart_mjcf_ctrl_default_preserved.xml";
  std::ofstream output(tempPath.string(), std::ios::binary);
  output << xml;
  output.close();

  auto world = utils::MjcfParser::readWorld(common::Uri(tempPath.string()));
  std::error_code ec;
  std::filesystem::remove(tempPath, ec);
  ASSERT_NE(world, nullptr);

  auto* j1 = world->getSkeleton(0)->getJoint("hinge1");
  ASSERT_NE(j1, nullptr);
  EXPECT_DOUBLE_EQ(
      j1->getForceLowerLimit(0), -std::numeric_limits<double>::infinity());
  EXPECT_DOUBLE_EQ(
      j1->getForceUpperLimit(0), std::numeric_limits<double>::infinity());
}

//==============================================================================
TEST(MjcfParserTest, JointLimitedAutoValue)
{
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
