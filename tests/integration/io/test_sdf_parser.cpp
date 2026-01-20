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

#include "dart/common/resource.hpp"
#include "dart/common/resource_retriever.hpp"
#include "dart/common/uri.hpp"
#include "dart/config.hpp"
#include "dart/dynamics/free_joint.hpp"
#include "dart/dynamics/mesh_shape.hpp"
#include "dart/dynamics/planar_joint.hpp"
#include "dart/dynamics/prismatic_joint.hpp"
#include "dart/dynamics/revolute_joint.hpp"
#include "dart/dynamics/screw_joint.hpp"
#include "dart/dynamics/skeleton.hpp"
#include "dart/dynamics/soft_body_node.hpp"
#include "dart/dynamics/universal_joint.hpp"
#include "dart/simulation/world.hpp"
#include "dart/utils/sdf/sdf_parser.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <filesystem>
#include <iostream>
#include <map>
#include <sstream>
#include <string>

#include <cstring>

#if DART_HAVE_spdlog
  #include <spdlog/sinks/ostream_sink.h>
  #include <spdlog/spdlog.h>
#endif

using namespace dart;
using namespace dart::dynamics;
using namespace dart::test;
using namespace math;
using namespace simulation;
using namespace utils;

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
    if (origin == SEEKTYPE_CUR)
      base = mOffset;
    else if (origin == SEEKTYPE_END)
      base = mData.size();

    const auto next = static_cast<long long>(base) + offset;
    if (next < 0 || next > static_cast<long long>(mData.size()))
      return false;

    mOffset = static_cast<std::size_t>(next);
    return true;
  }

  std::size_t read(void* buffer, std::size_t size, std::size_t count) override
  {
    const std::size_t bytes = size * count;
    if (bytes == 0)
      return 0;

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
    if (it == mFiles.end())
      return nullptr;
    return std::make_shared<MemoryResource>(it->second);
  }

private:
  std::map<std::string, std::string> mFiles;
};

class LogCapture
{
public:
  LogCapture()
  {
#if DART_HAVE_spdlog
    mStream = std::make_shared<std::ostringstream>();
    mSink = std::make_shared<spdlog::sinks::ostream_sink_mt>(*mStream);
    mPreviousLogger = spdlog::default_logger();
    mLogger = std::make_shared<spdlog::logger>("sdf-parser-log-capture", mSink);
    mLogger->set_level(spdlog::level::trace);
    mLogger->flush_on(spdlog::level::trace);
    spdlog::set_default_logger(mLogger);
    // Capture third-party warnings that log directly to stdout/stderr.
    mOldCout = std::cout.rdbuf(mStream->rdbuf());
    mOldCerr = std::cerr.rdbuf(mStream->rdbuf());
#else
    mOldCout = std::cout.rdbuf(mStream.rdbuf());
    mOldCerr = std::cerr.rdbuf(mStream.rdbuf());
#endif
  }

  ~LogCapture()
  {
#if DART_HAVE_spdlog
    if (mLogger)
      mLogger->flush();
    spdlog::set_default_logger(mPreviousLogger);
    if (mLogger)
      spdlog::drop(mLogger->name());
    if (mOldCout)
      std::cout.rdbuf(mOldCout);
    if (mOldCerr)
      std::cerr.rdbuf(mOldCerr);
#else
    std::cout.rdbuf(mOldCout);
    std::cerr.rdbuf(mOldCerr);
#endif
  }

  std::string contents()
  {
#if DART_HAVE_spdlog
    if (mLogger)
      mLogger->flush();
    if (mStream)
      return mStream->str();
    return {};
#else
    return mStream.str();
#endif
  }

private:
#if DART_HAVE_spdlog
  std::shared_ptr<std::ostringstream> mStream;
  std::shared_ptr<spdlog::sinks::ostream_sink_mt> mSink;
  std::shared_ptr<spdlog::logger> mPreviousLogger;
  std::shared_ptr<spdlog::logger> mLogger;
  std::streambuf* mOldCout{nullptr};
  std::streambuf* mOldCerr{nullptr};
#else
  std::ostringstream mStream;
  std::streambuf* mOldCout;
  std::streambuf* mOldCerr;
#endif
};

} // namespace

//==============================================================================
TEST(SdfParser, SDFSingleBodyWithoutJoint)
{
  // Regression test for #444
  WorldPtr world = SdfParser::readWorld(
      "dart://sample/sdf/test/single_bodynode_skeleton.world");
  EXPECT_TRUE(world != nullptr);

  SkeletonPtr skel = world->getSkeleton(0);
  EXPECT_TRUE(skel != nullptr);
  EXPECT_EQ(skel->getNumBodyNodes(), 1u);
  EXPECT_EQ(skel->getNumJoints(), 1u);

  BodyNodePtr bodyNode = skel->getBodyNode(0);
  EXPECT_TRUE(bodyNode != nullptr);
  EXPECT_EQ(bodyNode->getNumShapeNodesWith<VisualAspect>(), 1u);
  EXPECT_EQ(bodyNode->getNumShapeNodesWith<CollisionAspect>(), 1u);

  JointPtr joint = skel->getJoint(0);
  EXPECT_TRUE(joint != nullptr);
  EXPECT_EQ(joint->getType(), FreeJoint::getStaticType());
}

//==============================================================================
TEST(SdfParser, ParsesHighVersionWorlds)
{
  WorldPtr world
      = SdfParser::readWorld("dart://sample/sdf/test/high_version.world");
  ASSERT_TRUE(world != nullptr);
  ASSERT_GT(world->getNumSkeletons(), 0u);

  SkeletonPtr skeleton = world->getSkeleton(0);
  ASSERT_TRUE(skeleton != nullptr);
  EXPECT_EQ(skeleton->getNumBodyNodes(), 1u);
}

//==============================================================================
TEST(SdfParser, SDFJointProperties)
{
  WorldPtr world = SdfParser::readWorld(
      "dart://sample/sdf/test/test_skeleton_joint.world");
  EXPECT_TRUE(world != nullptr);

  SkeletonPtr skel = world->getSkeleton(0);
  EXPECT_TRUE(skel != nullptr);
  EXPECT_EQ(skel->getNumBodyNodes(), 5u);
  EXPECT_EQ(skel->getNumJoints(), 5u);

  const double epsilon = 1e-7;

  auto testProperties = [epsilon](const Joint* joint, const size_t idx) {
    EXPECT_TRUE(joint->areLimitsEnforced()) << joint->getName();
    EXPECT_NEAR(joint->getPositionLowerLimit(idx), 0, epsilon);
    EXPECT_NEAR(joint->getPositionUpperLimit(idx), 3, epsilon);
    EXPECT_NEAR(joint->getDampingCoefficient(idx), 0, epsilon);
    EXPECT_NEAR(joint->getCoulombFriction(idx), 1, epsilon);
    EXPECT_NEAR(joint->getRestPosition(idx), 2, epsilon);
    EXPECT_NEAR(joint->getSpringStiffness(idx), 3, epsilon);
  };

  skel->eachJoint([&](Joint* joint) {
    if (joint->getType() == PrismaticJoint::getStaticType()
        || joint->getType() == RevoluteJoint::getStaticType()
        || joint->getType() == ScrewJoint::getStaticType()) {
      testProperties(joint, 0);
    } else if (joint->getType() == UniversalJoint::getStaticType()) {
      testProperties(joint, 0);
      testProperties(joint, 1);
    } else if (joint->getType() == FreeJoint::getStaticType()) {
      EXPECT_FALSE(joint->areLimitsEnforced());
    }
  });
}

//==============================================================================
TEST(SdfParser, ResolvesMeshesRelativeToIncludedModels)
{
  WorldPtr world = SdfParser::readWorld(
      "dart://sample/sdf/test/include_relative_mesh/"
      "include_relative_mesh.world");
  ASSERT_TRUE(world != nullptr);
  ASSERT_EQ(world->getNumSkeletons(), 1u);

  SkeletonPtr skeleton = world->getSkeleton(0);
  ASSERT_TRUE(skeleton != nullptr);

  bool foundMesh = false;
  skeleton->eachBodyNode([&](dynamics::BodyNode* body) {
    const auto numShapeNodes = body->getNumShapeNodes();
    for (auto i = 0u; i < numShapeNodes; ++i) {
      const auto* shapeNode = body->getShapeNode(i);
      const auto shape = shapeNode->getShape();
      const auto* mesh = dynamic_cast<const dynamics::MeshShape*>(shape.get());
      if (!mesh)
        continue;

      foundMesh = true;
      const std::string meshPath = mesh->getMeshPath();
      EXPECT_FALSE(meshPath.empty());
      EXPECT_NE(meshPath.find("relative_box.obj"), std::string::npos);
      EXPECT_TRUE(std::filesystem::exists(meshPath))
          << "Mesh path [" << meshPath << "] should exist.";

      const std::string meshUri = mesh->getMeshUri();
      EXPECT_NE(meshUri.find("meshes/relative_box.obj"), std::string::npos);
    }
  });

  EXPECT_TRUE(foundMesh);
}

//==============================================================================
TEST(SdfParser, ResolvesRelativeIncludesFromRetriever)
{
  auto retriever = std::make_shared<MemoryResourceRetriever>();
  const std::string worldUri
      = "memory://pkg/worlds/include_relative_include.world";
  const std::string modelUri = "memory://pkg/models/box/model.sdf";
  const std::string meshUri = "memory://pkg/models/box/meshes/box.obj";

  const std::string worldSdf = R"(
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="default">
    <include>
      <uri>../models/box/model.sdf</uri>
    </include>
  </world>
</sdf>
)";

  const std::string modelSdf = R"(
<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="box">
    <static>true</static>
    <link name="link">
      <inertial>
        <mass>1.0</mass>
      </inertial>
      <visual name="visual">
        <geometry>
          <mesh>
            <uri>meshes/box.obj</uri>
          </mesh>
        </geometry>
      </visual>
      <collision name="collision">
        <geometry>
          <mesh>
            <uri>meshes/box.obj</uri>
          </mesh>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>
)";

  const std::string meshObj = R"(
o Box
v -0.5 -0.5 -0.5
v 0.5 -0.5 -0.5
v 0.5 0.5 -0.5
v -0.5 0.5 -0.5
v -0.5 -0.5 0.5
v 0.5 -0.5 0.5
v 0.5 0.5 0.5
v -0.5 0.5 0.5
f 1 2 3
f 1 3 4
f 5 6 7
f 5 7 8
f 1 5 6
f 1 6 2
f 2 6 7
f 2 7 3
f 3 7 8
f 3 8 4
f 4 8 5
f 4 5 1
)";

  retriever->add(worldUri, worldSdf);
  retriever->add(modelUri, modelSdf);
  retriever->add(meshUri, meshObj);

  utils::SdfParser::Options options(retriever);
  auto world = utils::SdfParser::readWorld(common::Uri(worldUri), options);
  ASSERT_TRUE(world != nullptr);
  ASSERT_EQ(world->getNumSkeletons(), 1u);

  const auto skeleton = world->getSkeleton(0);
  ASSERT_TRUE(skeleton != nullptr);
  EXPECT_EQ("box", skeleton->getName());

  bool foundMesh = false;
  skeleton->eachBodyNode([&](dynamics::BodyNode* body) {
    const auto numShapeNodes = body->getNumShapeNodes();
    for (auto i = 0u; i < numShapeNodes; ++i) {
      const auto* shapeNode = body->getShapeNode(i);
      const auto shape = shapeNode->getShape();
      const auto* mesh = dynamic_cast<const dynamics::MeshShape*>(shape.get());
      if (!mesh)
        continue;

      foundMesh = true;
      const std::string meshUriStr = mesh->getMeshUri();
      EXPECT_NE(meshUriStr.find("meshes/box.obj"), std::string::npos);
    }
  });

  EXPECT_TRUE(foundMesh);
}

//==============================================================================
TEST(SdfParser, ParsingSDFFiles)
{
  const auto numSteps = 10u;

  // Create a list of sdf files to test with where the sdf files contains World
  std::vector<common::Uri> worldFiles;
  worldFiles.push_back("dart://sample/sdf/benchmark.world");
  worldFiles.push_back("dart://sample/sdf/double_pendulum.world");
  worldFiles.push_back("dart://sample/sdf/double_pendulum_with_base.world");
  worldFiles.push_back("dart://sample/sdf/empty.world");
  worldFiles.push_back("dart://sample/sdf/ground.world");
  worldFiles.push_back("dart://sample/sdf/test/single_bodynode_skeleton.world");

  std::vector<WorldPtr> worlds;
  for (const auto& worldFile : worldFiles)
    worlds.push_back(SdfParser::readWorld(worldFile));

  for (auto world : worlds) {
    EXPECT_TRUE(nullptr != world);

    for (auto i = 0u; i < numSteps; ++i)
      world->step();
  }

  // Create another list of sdf files to test with where the sdf files contains
  // Skeleton
  std::vector<common::Uri> skeletonFiles;
  skeletonFiles.push_back("dart://sample/sdf/atlas/atlas_v3_no_head.sdf");
  skeletonFiles.push_back(
      "dart://sample/sdf/atlas/atlas_v3_no_head_soft_feet.sdf");

  auto world = std::make_shared<World>();
  std::vector<SkeletonPtr> skeletons;
  for (const auto& skeletonFile : skeletonFiles)
    skeletons.push_back(SdfParser::readSkeleton(skeletonFile));

  for (auto skeleton : skeletons) {
    EXPECT_TRUE(nullptr != skeleton);

    world->addSkeleton(skeleton);
    for (auto i = 0u; i < numSteps; ++i)
      world->step();

    world->removeAllSkeletons();
  }
}

//==============================================================================
TEST(SdfParser, ReadMaterial)
{
  const common::Uri sdfUri("dart://sample/sdf/quad.sdf");
  SkeletonPtr skeleton = SdfParser::readSkeleton(sdfUri);
  EXPECT_TRUE(nullptr != skeleton);
  auto bodyNode = skeleton->getBodyNode(0);

  bodyNode->eachShapeNodeWith<dart::dynamics::VisualAspect>(
      [](dart::dynamics::ShapeNode* shapeNode) {
        Eigen::Vector4d color = shapeNode->getVisualAspect()->getRGBA();
        Eigen::Vector4d expected_color(0.5, 0.6, 0.8, 1.0);
        double diff = (color - expected_color).norm();
        EXPECT_LT(diff, 1e-4);
      });
}

//==============================================================================
TEST(SdfParser, ReadsMeshesFromCustomRetriever)
{
  auto retriever = std::make_shared<MemoryResourceRetriever>();

  const std::string meshUri = "mem://meshes/unit_box.obj";
  const std::string meshData = R"(o Box
v 0 0 0
v 1 0 0
v 0 1 0
v 0 0 1
f 1 2 3
f 1 3 4
)";

  const std::string worldUri = "mem://worlds/mesh.world";
  const std::string worldData = R"(
<sdf version='1.10'>
  <world name='default'>
    <model name='mesh_model'>
      <static>true</static>
      <link name='mesh_link'>
        <visual name='mesh_visual'>
          <geometry>
            <mesh>
              <uri>)" + meshUri + R"(</uri>
            </mesh>
          </geometry>
        </visual>
      </link>
    </model>
  </world>
</sdf>)";

  retriever->add(worldUri, worldData);
  retriever->add(meshUri, meshData);

  SdfParser::Options options;
  options.mResourceRetriever = retriever;

  WorldPtr world = SdfParser::readWorld(common::Uri(worldUri), options);
  ASSERT_TRUE(world);
  ASSERT_GT(world->getNumSkeletons(), 0u);

  SkeletonPtr skeleton = world->getSkeleton(0);
  ASSERT_TRUE(skeleton);

  auto* body = skeleton->getBodyNode(0);
  ASSERT_TRUE(body);

  bool foundMesh = false;
  body->eachShapeNodeWith<dynamics::VisualAspect>(
      [&](dynamics::ShapeNode* node) {
        if (std::dynamic_pointer_cast<dynamics::MeshShape>(node->getShape()))
          foundMesh = true;
      });
  EXPECT_TRUE(foundMesh);
}

//==============================================================================
TEST(SdfParser, WarnsOnMissingInertialBlock)
{
  auto retriever = std::make_shared<MemoryResourceRetriever>();
  const common::Uri modelUri("memory://pkg/models/missing_inertial/model.sdf");
  const std::string modelUriString = modelUri.toString();

  const std::string modelSdf = R"(
<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="no_inertial">
    <link name="link_without_inertial">
      <visual name="v">
        <geometry>
          <box><size>0.1 0.1 0.1</size></box>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>
)";

  retriever->add(modelUriString, modelSdf);

  LogCapture capture;
  SdfParser::Options options;
  options.mResourceRetriever = retriever;
  auto skeleton = SdfParser::readSkeleton(modelUri, options);

  ASSERT_TRUE(skeleton);
  ASSERT_EQ(skeleton->getNumBodyNodes(), 1u);
  const auto* body = skeleton->getBodyNode(0);
  EXPECT_DOUBLE_EQ(body->getMass(), 1.0);

  const auto logs = capture.contents();
  if (!logs.empty()) {
    EXPECT_NE(logs.find("missing <inertial>"), std::string::npos)
        << "Expected warning about missing <inertial> block in logs: " << logs;
  }
}

//==============================================================================
TEST(SdfParser, WarnsOnTinyMassAndDefaultsInertia)
{
  auto retriever = std::make_shared<MemoryResourceRetriever>();
  const common::Uri modelUri("memory://pkg/models/tiny_mass/model.sdf");
  const std::string modelUriString = modelUri.toString();
  const double tinyMass = 1e-14;
  const double clampedMass = 1e-9; // matches parser clamp

  const std::string modelSdf = std::string(R"(
<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="tiny_mass_model">
    <link name="link_with_mass_only">
      <inertial>
        <mass>)") + std::to_string(tinyMass)
                               + R"(</mass>
      </inertial>
      <collision name="c">
        <geometry>
          <box><size>0.1 0.1 0.1</size></box>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>
)";

  retriever->add(modelUriString, modelSdf);

  LogCapture capture;
  SdfParser::Options options;
  options.mResourceRetriever = retriever;
  auto skeleton = SdfParser::readSkeleton(modelUri, options);

  ASSERT_TRUE(skeleton);
  ASSERT_EQ(skeleton->getNumBodyNodes(), 1u);
  const auto* body = skeleton->getBodyNode(0);
  const auto inertia = body->getInertia();
  EXPECT_DOUBLE_EQ(inertia.getMass(), clampedMass);
  const Eigen::Matrix3d expectedMoment
      = Eigen::Matrix3d::Identity() * clampedMass;
  EXPECT_TRUE(inertia.getMoment().isApprox(expectedMoment));

  const auto logs = capture.contents();
  const bool warningsCaptured = !logs.empty();
  if (warningsCaptured) {
    const bool hasSmallMassWarning
        = logs.find("very small mass") != std::string::npos
          || logs.find("non-positive mass") != std::string::npos;
    EXPECT_TRUE(hasSmallMassWarning)
        << "Expected warning about tiny mass clamping in logs: " << logs;
    std::string logsLower = logs;
    std::transform(
        logsLower.begin(), logsLower.end(), logsLower.begin(), ::tolower);
    EXPECT_NE(logsLower.find("clamping to"), std::string::npos)
        << "Expected warning about tiny mass clamping in logs: " << logs;
    EXPECT_NE(logs.find("defines <mass> but no <inertia>"), std::string::npos)
        << "Expected warning about missing inertia tensor in logs: " << logs;
  }
}
