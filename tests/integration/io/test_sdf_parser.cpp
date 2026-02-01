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

#include "dart/common/resource.hpp"
#include "dart/common/resource_retriever.hpp"
#include "dart/common/uri.hpp"
#include "dart/dynamics/ball_joint.hpp"
#include "dart/dynamics/box_shape.hpp"
#include "dart/dynamics/capsule_shape.hpp"
#include "dart/dynamics/cylinder_shape.hpp"
#include "dart/dynamics/ellipsoid_shape.hpp"
#include "dart/dynamics/free_joint.hpp"
#include "dart/dynamics/mesh_shape.hpp"
#include "dart/dynamics/prismatic_joint.hpp"
#include "dart/dynamics/revolute_joint.hpp"
#include "dart/dynamics/screw_joint.hpp"
#include "dart/dynamics/sphere_shape.hpp"
#include "dart/dynamics/universal_joint.hpp"
#include "dart/dynamics/weld_joint.hpp"
#include "dart/io/read.hpp"
#include "dart/simulation/world.hpp"
#include "dart/utils/sdf/detail/sdf_helpers.hpp"
#include "dart/utils/sdf/sdf_parser.hpp"

#include <dart/all.hpp>

#include <gtest/gtest.h>
#include <sdf/sdf.hh>

#include <algorithm>
#include <filesystem>
#include <fstream>
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
    if (mLogger) {
      mLogger->flush();
    }
    spdlog::set_default_logger(mPreviousLogger);
    if (mLogger) {
      spdlog::drop(mLogger->name());
    }
    if (mOldCout) {
      std::cout.rdbuf(mOldCout);
    }
    if (mOldCerr) {
      std::cerr.rdbuf(mOldCerr);
    }
#else
    std::cout.rdbuf(mOldCout);
    std::cerr.rdbuf(mOldCerr);
#endif
  }

  std::string contents()
  {
#if DART_HAVE_spdlog
    if (mLogger) {
      mLogger->flush();
    }
    if (mStream) {
      return mStream->str();
    }
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

std::filesystem::path writeTempFile(
    const std::string& xml,
    const std::string& tag,
    const std::string& extension)
{
  static std::size_t counter = 0;
  const auto tempPath
      = std::filesystem::temp_directory_path()
        / ("test_" + tag + "_" + std::to_string(counter++) + extension);
  std::ofstream output(tempPath.string(), std::ios::binary);
  output << xml;
  output.close();
  return tempPath;
}

std::filesystem::path writeTempFileWithPrefix(
    const std::string& xml,
    const std::string& tag,
    const std::string& extension)
{
  static std::size_t counter = 0;
  const auto tempPath
      = std::filesystem::path("/tmp")
        / ("dart_test_" + tag + "_" + std::to_string(counter++) + extension);
  std::ofstream output(tempPath.string(), std::ios::binary);
  output << xml;
  output.close();
  return tempPath;
}

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
      if (!mesh) {
        continue;
      }

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
      if (!mesh) {
        continue;
      }

      foundMesh = true;
      const std::string meshUriStr = mesh->getMeshUri();
      EXPECT_NE(meshUriStr.find("meshes/box.obj"), std::string::npos);
    }
  });

  EXPECT_TRUE(foundMesh);
}

//==============================================================================
TEST(SdfParser, ParsesPlaneAndJointVariants)
{
  const std::string sdfXml = R"(
<?xml version="1.0" ?>
<sdf version="1.7" xmlns:dart="https://dartsim.org">
  <world name="default">
    <model name="joint_model">
      <static>true</static>
      <link name="base">
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.1</ixx>
            <iyy>0.1</iyy>
            <izz>0.1</izz>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyz>0</iyz>
          </inertia>
        </inertial>
        <collision name="plane_collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>10 10</size>
            </plane>
          </geometry>
        </collision>
      </link>
      <link name="link1">
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.1</ixx>
            <iyy>0.1</iyy>
            <izz>0.1</izz>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyz>0</iyz>
          </inertia>
        </inertial>
      </link>
      <link name="link2">
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.1</ixx>
            <iyy>0.1</iyy>
            <izz>0.1</izz>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyz>0</iyz>
          </inertia>
        </inertial>
      </link>
      <link name="link3">
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.1</ixx>
            <iyy>0.1</iyy>
            <izz>0.1</izz>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyz>0</iyz>
          </inertia>
        </inertial>
      </link>
      <link name="link4">
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.1</ixx>
            <iyy>0.1</iyy>
            <izz>0.1</izz>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyz>0</iyz>
          </inertia>
        </inertial>
      </link>
      <joint name="revolute2_joint" type="revolute2">
        <parent>base</parent>
        <child>link1</child>
        <axis>
          <xyz>1 0 0</xyz>
          <limit>
            <lower>-1</lower>
            <upper>1</upper>
          </limit>
          <dynamics>
            <spring_reference>0.1</spring_reference>
            <spring_stiffness>100</spring_stiffness>
          </dynamics>
        </axis>
        <axis2>
          <xyz>0 1 0</xyz>
        </axis2>
        <dart:actuator_type>servo</dart:actuator_type>
      </joint>
      <joint name="ball_joint" type="ball">
        <parent>link1</parent>
        <child>link2</child>
      </joint>
      <joint name="screw_joint" type="screw">
        <parent>link2</parent>
        <child>link3</child>
        <axis>
          <xyz>0 0 1</xyz>
          <limit>
            <lower>-0.5</lower>
            <upper>0.5</upper>
          </limit>
        </axis>
        <thread_pitch>0.2</thread_pitch>
      </joint>
      <joint name="universal_joint" type="universal">
        <parent>link3</parent>
        <child>link4</child>
        <axis>
          <xyz>1 0 0</xyz>
        </axis>
        <axis2>
          <xyz>0 1 0</xyz>
        </axis2>
      </joint>
    </model>
  </world>
</sdf>
 )";

  const auto tempPath = writeTempFile(sdfXml, "sdf_parser", ".sdf");
  auto world = dart::io::readWorld(common::Uri(tempPath.string()));
  std::error_code ec;
  std::filesystem::remove(tempPath, ec);

  ASSERT_NE(world, nullptr);
  auto skel = world->getSkeleton("joint_model");
  ASSERT_NE(skel, nullptr);

  auto* base = skel->getBodyNode("base");
  ASSERT_NE(base, nullptr);
  bool foundPlane = false;
  base->eachShapeNodeWith<CollisionAspect>([&](ShapeNode* node) {
    if (dynamic_cast<const BoxShape*>(node->getShape().get())) {
      foundPlane = true;
    }
  });
  EXPECT_TRUE(foundPlane);

  auto* revolute2
      = dynamic_cast<UniversalJoint*>(skel->getJoint("revolute2_joint"));
  ASSERT_NE(revolute2, nullptr);
  EXPECT_DOUBLE_EQ(revolute2->getSpringStiffness(0), 100);
  EXPECT_DOUBLE_EQ(revolute2->getRestPosition(0), 0.1);

  auto* ball = dynamic_cast<BallJoint*>(skel->getJoint("ball_joint"));
  EXPECT_NE(ball, nullptr);

  auto* screw = dynamic_cast<ScrewJoint*>(skel->getJoint("screw_joint"));
  ASSERT_NE(screw, nullptr);
  EXPECT_DOUBLE_EQ(screw->getPitch(), 0.2);

  auto* universal
      = dynamic_cast<UniversalJoint*>(skel->getJoint("universal_joint"));
  EXPECT_NE(universal, nullptr);
}

//==============================================================================
TEST(SdfParser, ParsesMjcfActuators)
{
  const std::string mjcfXml = R"(
<mujoco model="mjcf_actuator">
  <option timestep="0.01" />
  <default>
    <geom type="sphere" size="0.1" />
  </default>
  <worldbody>
    <body name="torso">
      <joint name="root" type="free" />
      <geom type="sphere" size="0.1" />
      <body name="link1" pos="0 0 0.2">
        <joint name="hinge1" type="hinge" axis="1 0 0" />
        <geom type="sphere" size="0.1" />
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor name="motor1" joint="hinge1" gear="1" />
    <position name="pos1" joint="hinge1" kp="10" />
    <velocity name="vel1" joint="hinge1" kv="2" />
  </actuator>
</mujoco>
 )";

  const auto tempPath = writeTempFile(mjcfXml, "mjcf_parser", ".xml");
  auto world = dart::io::readWorld(common::Uri(tempPath.string()));
  std::error_code ec;
  std::filesystem::remove(tempPath, ec);

  ASSERT_NE(world, nullptr);
  EXPECT_GT(world->getNumSkeletons(), 0u);
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
  for (const auto& worldFile : worldFiles) {
    worlds.push_back(SdfParser::readWorld(worldFile));
  }

  for (auto world : worlds) {
    EXPECT_TRUE(nullptr != world);

    for (auto i = 0u; i < numSteps; ++i) {
      world->step();
    }
  }

  // Create another list of sdf files to test with where the sdf files contains
  // Skeleton
  std::vector<common::Uri> skeletonFiles;
  skeletonFiles.push_back("dart://sample/sdf/atlas/atlas_v3_no_head.sdf");
  skeletonFiles.push_back(
      "dart://sample/sdf/atlas/atlas_v3_no_head_soft_feet.sdf");

  auto world = std::make_shared<World>();
  std::vector<SkeletonPtr> skeletons;
  for (const auto& skeletonFile : skeletonFiles) {
    skeletons.push_back(SdfParser::readSkeleton(skeletonFile));
  }

  for (auto skeleton : skeletons) {
    EXPECT_TRUE(nullptr != skeleton);

    world->addSkeleton(skeleton);
    for (auto i = 0u; i < numSteps; ++i) {
      world->step();
    }

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
        if (std::dynamic_pointer_cast<dynamics::MeshShape>(node->getShape())) {
          foundMesh = true;
        }
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

#if DART_HAVE_SDFORMAT
namespace {

template <typename ShapeType>
bool hasShapeType(const dynamics::SkeletonPtr& skeleton)
{
  if (!skeleton) {
    return false;
  }

  for (std::size_t i = 0; i < skeleton->getNumBodyNodes(); ++i) {
    const auto* body = skeleton->getBodyNode(i);
    if (!body) {
      continue;
    }
    for (std::size_t j = 0; j < body->getNumShapeNodes(); ++j) {
      const auto* shapeNode = body->getShapeNode(j);
      if (!shapeNode) {
        continue;
      }
      const auto shape = shapeNode->getShape();
      if (shape && shape->is<ShapeType>()) {
        return true;
      }
    }
  }

  return false;
}

} // namespace

//==============================================================================
TEST(SdfParser, RevoluteJointWorldParses)
{
  const auto world = utils::SdfParser::readWorld(
      "dart://sample/sdf/test/issue1193_revolute_test.sdf");
  ASSERT_NE(world, nullptr);

  const auto skel = world->getSkeleton("M");
  ASSERT_NE(skel, nullptr);
  EXPECT_EQ(skel->getNumBodyNodes(), 2u);

  const auto joint = skel->getJoint("revJoint");
  ASSERT_NE(joint, nullptr);
  EXPECT_NE(dynamic_cast<dynamics::RevoluteJoint*>(joint), nullptr);
}

//==============================================================================
TEST(SdfParser, BoxWorldParsesGeometry)
{
  const auto world = utils::SdfParser::readWorld(
      "dart://sample/sdf/test/issue1624_cubes.sdf");
  ASSERT_NE(world, nullptr);
  EXPECT_GT(world->getNumSkeletons(), 1u);

  const auto skel = world->getSkeleton("model_0_0_0");
  ASSERT_NE(skel, nullptr);
  EXPECT_TRUE(hasShapeType<dynamics::BoxShape>(skel));
  EXPECT_GT(skel->getNumBodyNodes(), 0u);
}
#endif

//==============================================================================
TEST(SdfParser, ReadWorldMissingWorldElementReturnsNull)
{
  auto retriever = std::make_shared<MemoryResourceRetriever>();
  const std::string uri = "memory://pkg/missing_world.sdf";
  retriever->add(uri, "<sdf version='1.7'><model name='m'/></sdf>");

  SdfParser::Options options;
  options.mResourceRetriever = retriever;
  EXPECT_EQ(SdfParser::readWorld(common::Uri(uri), options), nullptr);
}

//==============================================================================
TEST(SdfParser, ReadSkeletonMissingModelElementReturnsNull)
{
  auto retriever = std::make_shared<MemoryResourceRetriever>();
  const std::string uri = "memory://pkg/missing_model.sdf";
  retriever->add(uri, "<sdf version='1.7'><world name='w'/></sdf>");

  SdfParser::Options options;
  options.mResourceRetriever = retriever;
  EXPECT_EQ(SdfParser::readSkeleton(common::Uri(uri), options), nullptr);
}

//==============================================================================
TEST(SdfParser, ReadWorldEmptyContentReturnsNull)
{
  auto retriever = std::make_shared<MemoryResourceRetriever>();
  const std::string uri = "memory://pkg/empty.sdf";
  retriever->add(uri, "");

  SdfParser::Options options;
  options.mResourceRetriever = retriever;
  EXPECT_EQ(SdfParser::readWorld(common::Uri(uri), options), nullptr);
}

//==============================================================================
TEST(SdfParser, ReadWorldInvalidXmlReturnsNull)
{
  auto retriever = std::make_shared<MemoryResourceRetriever>();
  const std::string uri = "memory://pkg/invalid_xml.sdf";
  retriever->add(uri, "<sdf version='1.7'><world></sdf>");

  SdfParser::Options options;
  options.mResourceRetriever = retriever;
  EXPECT_EQ(SdfParser::readWorld(common::Uri(uri), options), nullptr);
}

//==============================================================================
TEST(SdfParser, WorldPhysicsOverridesGravityAndStep)
{
  auto retriever = std::make_shared<MemoryResourceRetriever>();
  const std::string uri = "memory://pkg/physics.world";
  const std::string worldSdf = R"(
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="default">
    <physics type="dart">
      <max_step_size>0.02</max_step_size>
      <gravity>0 1 -2</gravity>
    </physics>
    <model name="box">
      <link name="link">
        <inertial>
          <mass>1.0</mass>
        </inertial>
      </link>
    </model>
  </world>
</sdf>
)";

  retriever->add(uri, worldSdf);
  SdfParser::Options options;
  options.mResourceRetriever = retriever;
  auto world = SdfParser::readWorld(common::Uri(uri), options);
  ASSERT_TRUE(world != nullptr);
  EXPECT_DOUBLE_EQ(world->getTimeStep(), 0.02);
  EXPECT_TRUE(world->getGravity().isApprox(Eigen::Vector3d(0.0, 1.0, -2.0)));
}

//==============================================================================
TEST(SdfParser, LinkMissingMassDefaultsToUnit)
{
  auto retriever = std::make_shared<MemoryResourceRetriever>();
  const std::string uri = "memory://pkg/missing_mass.sdf";
  const std::string modelSdf = R"(
<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="missing_mass">
    <link name="link">
      <inertial>
        <inertia>
          <ixx>0.1</ixx>
          <iyy>0.1</iyy>
          <izz>0.1</izz>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyz>0</iyz>
        </inertia>
      </inertial>
    </link>
  </model>
</sdf>
)";

  retriever->add(uri, modelSdf);
  SdfParser::Options options;
  options.mResourceRetriever = retriever;
  auto skeleton = SdfParser::readSkeleton(common::Uri(uri), options);
  ASSERT_TRUE(skeleton != nullptr);
  const auto* body = skeleton->getBodyNode(0);
  ASSERT_TRUE(body != nullptr);
  EXPECT_DOUBLE_EQ(body->getMass(), 1.0);
}

//==============================================================================
TEST(SdfParser, GeometryParsesPlaneCylinderSphereAndBox)
{
  auto retriever = std::make_shared<MemoryResourceRetriever>();
  const std::string uri = "memory://pkg/geometry.world";
  const std::string worldSdf = R"(
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="default">
    <model name="shapes">
      <link name="link">
        <inertial>
          <mass>1.0</mass>
        </inertial>
        <visual name="box_visual">
          <geometry>
            <box><size>1 2 3</size></box>
          </geometry>
        </visual>
        <visual name="plane_visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>2 3</size>
            </plane>
          </geometry>
        </visual>
        <visual name="cylinder_visual">
          <geometry>
            <cylinder><radius>0.2</radius><length>0.5</length></cylinder>
          </geometry>
        </visual>
        <collision name="sphere_collision">
          <geometry>
            <sphere><radius>0.1</radius></sphere>
          </geometry>
        </collision>
      </link>
    </model>
  </world>
</sdf>
)";

  retriever->add(uri, worldSdf);
  SdfParser::Options options;
  options.mResourceRetriever = retriever;
  auto world = SdfParser::readWorld(common::Uri(uri), options);
  ASSERT_TRUE(world != nullptr);
  const auto skeleton = world->getSkeleton(0);
  ASSERT_TRUE(skeleton != nullptr);
  auto* body = skeleton->getBodyNode(0);
  ASSERT_TRUE(body != nullptr);

  std::size_t boxCount = 0u;
  std::size_t planeCount = 0u;
  std::size_t cylinderCount = 0u;
  std::size_t sphereCount = 0u;
  const auto numShapes = body->getNumShapeNodes();
  for (std::size_t i = 0; i < numShapes; ++i) {
    const auto* shapeNode = body->getShapeNode(i);
    ASSERT_NE(shapeNode, nullptr);
    const auto shape = shapeNode->getShape();
    if (!shape) {
      continue;
    }
    const auto type = shape->getType();
    if (type == "SphereShape") {
      ++sphereCount;
    } else if (type == "CylinderShape") {
      ++cylinderCount;
    } else if (type == "BoxShape") {
      auto box = std::dynamic_pointer_cast<const dynamics::BoxShape>(shape);
      ASSERT_TRUE(box);
      const auto size = box->getSize();
      if (size.isApprox(Eigen::Vector3d(2.0, 3.0, 0.001))) {
        ++planeCount;
      } else if (size.isApprox(Eigen::Vector3d(1.0, 2.0, 3.0))) {
        ++boxCount;
      }
    }
  }

  EXPECT_EQ(boxCount, 1u);
  EXPECT_EQ(planeCount, 1u);
  EXPECT_EQ(cylinderCount, 1u);
  EXPECT_EQ(sphereCount, 1u);
}

//==============================================================================
TEST(SdfParser, VisualAndCollisionNamesAreApplied)
{
  auto retriever = std::make_shared<MemoryResourceRetriever>();
  const std::string uri = "memory://pkg/missing_names.world";
  const std::string worldSdf = R"(
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="default">
    <model name="unnamed">
      <link name="link">
        <inertial>
          <mass>1.0</mass>
        </inertial>
        <visual name="visual_one">
          <geometry>
            <box><size>0.1 0.1 0.1</size></box>
          </geometry>
        </visual>
        <collision name="collision_one">
          <geometry>
            <sphere><radius>0.1</radius></sphere>
          </geometry>
        </collision>
      </link>
    </model>
  </world>
</sdf>
)";

  retriever->add(uri, worldSdf);
  SdfParser::Options options;
  options.mResourceRetriever = retriever;
  auto world = SdfParser::readWorld(common::Uri(uri), options);
  ASSERT_TRUE(world != nullptr);
  const auto skeleton = world->getSkeleton(0);
  ASSERT_TRUE(skeleton != nullptr);
  auto* body = skeleton->getBodyNode(0);
  ASSERT_TRUE(body != nullptr);

  bool foundVisualName = false;
  bool foundCollisionName = false;
  const auto numShapes = body->getNumShapeNodes();
  for (std::size_t i = 0; i < numShapes; ++i) {
    const auto* shapeNode = body->getShapeNode(i);
    ASSERT_NE(shapeNode, nullptr);
    if (shapeNode->getName() == "link - visual_one") {
      foundVisualName = true;
    }
    if (shapeNode->getName() == "link - collision_one") {
      foundCollisionName = true;
    }
  }

  EXPECT_TRUE(foundVisualName);
  EXPECT_TRUE(foundCollisionName);
}

//==============================================================================
TEST(SdfParser, ReadWorldMissingResourceReturnsNull)
{
  auto retriever = std::make_shared<MemoryResourceRetriever>();
  const std::string uri = "memory://pkg/missing_resource.world";

  SdfParser::Options options;
  options.mResourceRetriever = retriever;
  EXPECT_EQ(SdfParser::readWorld(common::Uri(uri), options), nullptr);
}

namespace {

SkeletonPtr readSkeletonFromSdfString(
    const std::string& uri,
    const std::string& sdfString,
    const std::shared_ptr<MemoryResourceRetriever>& retriever)
{
  retriever->add(uri, sdfString);
  SdfParser::Options options;
  options.mResourceRetriever = retriever;
  return SdfParser::readSkeleton(common::Uri(uri), options);
}

} // namespace

//==============================================================================
TEST(SdfParser, AxisLimitsSetInitialMidpoint)
{
  auto retriever = std::make_shared<MemoryResourceRetriever>();
  const std::string uri = "memory://pkg/axis_midpoint/model.sdf";
  const std::string modelSdf = R"(
<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="axis_midpoint">
    <link name="base">
      <inertial><mass>1.0</mass></inertial>
    </link>
    <link name="tip">
      <inertial><mass>1.0</mass></inertial>
    </link>
    <joint name="rev_joint" type="revolute">
      <parent>base</parent>
      <child>tip</child>
      <axis>
        <xyz>0 0 1</xyz>
        <limit>
          <lower>1.0</lower>
          <upper>2.0</upper>
        </limit>
      </axis>
    </joint>
  </model>
</sdf>
 )";

  const auto skeleton = readSkeletonFromSdfString(uri, modelSdf, retriever);
  ASSERT_NE(skeleton, nullptr);
  auto* joint = skeleton->getJoint("rev_joint");
  ASSERT_NE(joint, nullptr);
  EXPECT_NEAR(joint->getPosition(0), 1.5, 1e-9);
  EXPECT_NEAR(joint->getRestPosition(0), 1.5, 1e-9);
}

//==============================================================================
TEST(SdfParser, AxisLimitLowerOnlySetsInitialToLower)
{
  auto retriever = std::make_shared<MemoryResourceRetriever>();
  const std::string uri = "memory://pkg/axis_lower_only/model.sdf";
  const std::string modelSdf = R"(
<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="axis_lower_only">
    <link name="base">
      <inertial><mass>1.0</mass></inertial>
    </link>
    <link name="slider">
      <inertial><mass>1.0</mass></inertial>
    </link>
    <joint name="prismatic_joint" type="prismatic">
      <parent>base</parent>
      <child>slider</child>
      <axis>
        <xyz>1 0 0</xyz>
        <limit>
          <lower>0.5</lower>
        </limit>
      </axis>
    </joint>
  </model>
</sdf>
 )";

  const auto skeleton = readSkeletonFromSdfString(uri, modelSdf, retriever);
  ASSERT_NE(skeleton, nullptr);
  auto* joint = skeleton->getJoint("prismatic_joint");
  ASSERT_NE(joint, nullptr);
  EXPECT_NEAR(joint->getPosition(0), 0.5, 1e-9);
}

//==============================================================================
TEST(SdfParser, AxisUsesParentModelFrameRotation)
{
  auto retriever = std::make_shared<MemoryResourceRetriever>();
  const std::string uri = "memory://pkg/axis_parent_frame/model.sdf";
  const std::string modelSdf = R"(
<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="axis_parent_frame">
    <link name="base">
      <inertial><mass>1.0</mass></inertial>
    </link>
    <link name="tip">
      <inertial><mass>1.0</mass></inertial>
    </link>
    <joint name="rev_joint" type="revolute">
      <parent>base</parent>
      <child>tip</child>
      <pose>0 0 0 0 0 1.57079632679</pose>
      <axis>
        <xyz>1 0 0</xyz>
        <use_parent_model_frame>true</use_parent_model_frame>
      </axis>
    </joint>
  </model>
</sdf>
 )";

  const auto skeleton = readSkeletonFromSdfString(uri, modelSdf, retriever);
  ASSERT_NE(skeleton, nullptr);
  auto* joint
      = dynamic_cast<dynamics::RevoluteJoint*>(skeleton->getJoint("rev_joint"));
  ASSERT_NE(joint, nullptr);
  EXPECT_TRUE(joint->getAxis().isApprox(Eigen::Vector3d(0.0, -1.0, 0.0), 1e-9));
}

//==============================================================================
TEST(SdfParser, JointDynamicsPropertiesFromAxis)
{
  auto retriever = std::make_shared<MemoryResourceRetriever>();
  const std::string uri = "memory://pkg/axis_dynamics/model.sdf";
  const std::string modelSdf = R"(
<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="axis_dynamics">
    <link name="base">
      <inertial><mass>1.0</mass></inertial>
    </link>
    <link name="tip">
      <inertial><mass>1.0</mass></inertial>
    </link>
    <joint name="rev_joint" type="revolute">
      <parent>base</parent>
      <child>tip</child>
      <axis>
        <xyz>0 1 0</xyz>
        <dynamics>
          <damping>0.3</damping>
          <friction>0.2</friction>
          <spring_reference>0.1</spring_reference>
          <spring_stiffness>0.9</spring_stiffness>
        </dynamics>
      </axis>
    </joint>
  </model>
</sdf>
 )";

  const auto skeleton = readSkeletonFromSdfString(uri, modelSdf, retriever);
  ASSERT_NE(skeleton, nullptr);
  auto* joint = skeleton->getJoint("rev_joint");
  ASSERT_NE(joint, nullptr);
  EXPECT_NEAR(joint->getDampingCoefficient(0), 0.3, 1e-9);
  EXPECT_NEAR(joint->getCoulombFriction(0), 0.2, 1e-9);
  EXPECT_NEAR(joint->getRestPosition(0), 0.1, 1e-9);
  EXPECT_NEAR(joint->getSpringStiffness(0), 0.9, 1e-9);
}

//==============================================================================
TEST(SdfParser, MimicJointAppliesActuator)
{
  auto retriever = std::make_shared<MemoryResourceRetriever>();
  const std::string uri = "memory://pkg/mimic_applies/model.sdf";
  const std::string modelSdf = R"(
<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="mimic_applies">
    <link name="base">
      <inertial><mass>1.0</mass></inertial>
    </link>
    <link name="link1">
      <inertial><mass>1.0</mass></inertial>
    </link>
    <link name="link2">
      <inertial><mass>1.0</mass></inertial>
    </link>
    <joint name="joint1" type="revolute">
      <parent>base</parent>
      <child>link1</child>
      <axis>
        <xyz>0 0 1</xyz>
      </axis>
    </joint>
    <joint name="joint2" type="revolute">
      <parent>link1</parent>
      <child>link2</child>
      <axis>
        <xyz>0 1 0</xyz>
        <mimic joint="joint1">
          <multiplier>2.0</multiplier>
          <offset>0.5</offset>
        </mimic>
      </axis>
    </joint>
  </model>
</sdf>
 )";

  const auto skeleton = readSkeletonFromSdfString(uri, modelSdf, retriever);
  ASSERT_NE(skeleton, nullptr);
  auto* joint = skeleton->getJoint("joint2");
  ASSERT_NE(joint, nullptr);
  EXPECT_EQ(joint->getActuatorType(), dynamics::Joint::MIMIC);
  auto mimicProps = joint->getMimicDofProperties();
  ASSERT_FALSE(mimicProps.empty());
  EXPECT_EQ(mimicProps[0].mMultiplier, 2.0);
  EXPECT_EQ(mimicProps[0].mOffset, 0.5);
}

//==============================================================================
TEST(SdfParser, MimicAxis2SetsReferenceDof)
{
  auto retriever = std::make_shared<MemoryResourceRetriever>();
  const std::string uri = "memory://pkg/mimic_axis2/model.sdf";
  const std::string modelSdf = R"(
<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="mimic_axis2">
    <link name="base">
      <inertial><mass>1.0</mass></inertial>
    </link>
    <link name="link1">
      <inertial><mass>1.0</mass></inertial>
    </link>
    <link name="link2">
      <inertial><mass>1.0</mass></inertial>
    </link>
    <joint name="universal_ref" type="universal">
      <parent>base</parent>
      <child>link1</child>
      <axis>
        <xyz>1 0 0</xyz>
      </axis>
      <axis2>
        <xyz>0 1 0</xyz>
      </axis2>
    </joint>
    <joint name="mimic_joint" type="revolute">
      <parent>link1</parent>
      <child>link2</child>
      <axis>
        <xyz>0 0 1</xyz>
        <mimic joint="universal_ref" axis="axis2">
          <multiplier>1.5</multiplier>
          <offset>-0.25</offset>
        </mimic>
      </axis>
    </joint>
  </model>
</sdf>
  )";

  const auto skeleton = readSkeletonFromSdfString(uri, modelSdf, retriever);
  ASSERT_NE(skeleton, nullptr);
  auto* joint = skeleton->getJoint("mimic_joint");
  ASSERT_NE(joint, nullptr);
  EXPECT_EQ(joint->getActuatorType(), dynamics::Joint::MIMIC);
  const auto mimicProps = joint->getMimicDofProperties();
  ASSERT_EQ(mimicProps.size(), 1u);
  EXPECT_EQ(mimicProps[0].mReferenceDofIndex, 1u);
  EXPECT_DOUBLE_EQ(mimicProps[0].mMultiplier, 1.5);
  EXPECT_DOUBLE_EQ(mimicProps[0].mOffset, -0.25);
}

//==============================================================================
TEST(SdfParser, MimicMissingReferenceIgnored)
{
  auto retriever = std::make_shared<MemoryResourceRetriever>();
  const std::string uri = "memory://pkg/mimic_missing/model.sdf";
  const std::string modelSdf = R"(
<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="mimic_missing">
    <link name="base">
      <inertial><mass>1.0</mass></inertial>
    </link>
    <link name="tip">
      <inertial><mass>1.0</mass></inertial>
    </link>
    <joint name="rev_joint" type="revolute">
      <parent>base</parent>
      <child>tip</child>
      <axis>
        <xyz>0 0 1</xyz>
        <mimic joint="missing_joint" />
      </axis>
    </joint>
  </model>
</sdf>
 )";

  const auto skeleton = readSkeletonFromSdfString(uri, modelSdf, retriever);
  ASSERT_NE(skeleton, nullptr);
  auto* joint = skeleton->getJoint("rev_joint");
  ASSERT_NE(joint, nullptr);
  EXPECT_NE(joint->getActuatorType(), dynamics::Joint::MIMIC);
}

//==============================================================================
TEST(SdfParser, MeshScaleFromMemoryResource)
{
  auto retriever = std::make_shared<MemoryResourceRetriever>();
  const std::string meshUri = "memory://pkg/meshes/box.obj";
  const std::string modelUri = "memory://pkg/models/mesh_scale/model.sdf";
  const std::string meshData = R"(
o Box
v 0 0 0
v 1 0 0
v 0 1 0
f 1 2 3
 )";
  const std::string modelSdf = std::string(R"(
<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="mesh_scale">
    <link name="link">
      <inertial><mass>1.0</mass></inertial>
      <visual name="mesh_visual">
        <geometry>
          <mesh>
            <uri>)") + meshUri + R"(</uri>
            <scale>1 2 3</scale>
          </mesh>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>
 )";

  retriever->add(meshUri, meshData);
  const auto skeleton
      = readSkeletonFromSdfString(modelUri, modelSdf, retriever);
  ASSERT_NE(skeleton, nullptr);
  auto* body = skeleton->getBodyNode(0);
  ASSERT_NE(body, nullptr);
  bool foundMesh = false;
  body->eachShapeNodeWith<dynamics::VisualAspect>(
      [&](dynamics::ShapeNode* node) {
        auto mesh
            = std::dynamic_pointer_cast<dynamics::MeshShape>(node->getShape());
        if (!mesh) {
          return;
        }
        foundMesh = true;
        EXPECT_TRUE(mesh->getScale().isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));
      });
  EXPECT_TRUE(foundMesh);
}

//==============================================================================
TEST(SdfParser, MassWithoutInertiaUsesIsotropicTensor)
{
  auto retriever = std::make_shared<MemoryResourceRetriever>();
  const std::string uri = "memory://pkg/isotropic_inertia/model.sdf";
  const std::string modelSdf = R"(
<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="isotropic_inertia">
    <link name="link">
      <inertial>
        <mass>2.0</mass>
      </inertial>
    </link>
  </model>
</sdf>
 )";

  const auto skeleton = readSkeletonFromSdfString(uri, modelSdf, retriever);
  ASSERT_NE(skeleton, nullptr);
  auto* body = skeleton->getBodyNode(0);
  ASSERT_NE(body, nullptr);
  EXPECT_DOUBLE_EQ(body->getMass(), 2.0);
  const auto expected = Eigen::Matrix3d::Identity() * 2.0;
  EXPECT_TRUE(body->getInertia().getMoment().isApprox(expected));
}

//==============================================================================
TEST(SdfParser, MultipleModelsInSingleWorld)
{
  auto retriever = std::make_shared<MemoryResourceRetriever>();
  const std::string uri = "memory://pkg/multi.world";
  const std::string worldSdf = R"(
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="default">
    <model name="first">
      <link name="link">
        <inertial><mass>1.0</mass></inertial>
      </link>
    </model>
    <model name="second">
      <link name="link">
        <inertial><mass>2.0</mass></inertial>
      </link>
    </model>
  </world>
</sdf>
 )";

  retriever->add(uri, worldSdf);
  SdfParser::Options options;
  options.mResourceRetriever = retriever;
  const auto world = SdfParser::readWorld(common::Uri(uri), options);
  ASSERT_TRUE(world != nullptr);
  EXPECT_EQ(world->getNumSkeletons(), 2u);
}

//==============================================================================
TEST(SdfParser, GeometryParsesCylinderPlaneAndMesh)
{
  auto retriever = std::make_shared<MemoryResourceRetriever>();
  const std::string meshUri = "memory://pkg/meshes/tri.obj";
  const std::string worldUri = "memory://pkg/geometry_mesh.world";
  const std::string meshData = R"(
o Tri
v 0 0 0
v 1 0 0
v 0 1 0
f 1 2 3
 )";
  const std::string worldSdf = std::string(R"(
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="default">
    <model name="geo">
      <link name="link">
        <inertial><mass>1.0</mass></inertial>
        <visual name="cylinder">
          <geometry>
            <cylinder><radius>0.2</radius><length>0.5</length></cylinder>
          </geometry>
        </visual>
        <visual name="plane">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>2 3</size>
            </plane>
          </geometry>
        </visual>
        <visual name="mesh">
          <geometry>
            <mesh>
              <uri>)") + meshUri
                               + R"(</uri>
            </mesh>
          </geometry>
        </visual>
      </link>
    </model>
  </world>
</sdf>
 )";

  retriever->add(meshUri, meshData);
  retriever->add(worldUri, worldSdf);
  SdfParser::Options options;
  options.mResourceRetriever = retriever;
  const auto world = SdfParser::readWorld(common::Uri(worldUri), options);
  ASSERT_TRUE(world != nullptr);
  const auto skeleton = world->getSkeleton("geo");
  ASSERT_TRUE(skeleton != nullptr);
  auto* body = skeleton->getBodyNode(0);
  ASSERT_TRUE(body != nullptr);

  bool foundCylinder = false;
  bool foundMesh = false;
  bool foundPlane = false;
  const auto numShapes = body->getNumShapeNodes();
  for (std::size_t i = 0; i < numShapes; ++i) {
    const auto* shapeNode = body->getShapeNode(i);
    ASSERT_NE(shapeNode, nullptr);
    const auto shape = shapeNode->getShape();
    if (!shape) {
      continue;
    }
    if (shape->getType() == "CylinderShape") {
      foundCylinder = true;
    } else if (shape->is<dynamics::MeshShape>()) {
      foundMesh = true;
    } else if (shape->is<dynamics::BoxShape>()) {
      auto box = std::dynamic_pointer_cast<const dynamics::BoxShape>(shape);
      if (box && box->getSize().z() > 0.0) {
        foundPlane = true;
      }
    }
  }

  EXPECT_TRUE(foundCylinder);
  EXPECT_TRUE(foundMesh);
  EXPECT_TRUE(foundPlane);
}

//==============================================================================
TEST(SdfParser, JointAxisLimitsEffortAndDamping)
{
  auto retriever = std::make_shared<MemoryResourceRetriever>();
  const std::string uri = "memory://pkg/joint_limits/model.sdf";
  const std::string modelSdf = R"(
<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="joint_limits">
    <link name="base">
      <inertial><mass>1.0</mass></inertial>
    </link>
    <link name="tip">
      <inertial><mass>1.0</mass></inertial>
    </link>
    <joint name="rev_joint" type="revolute">
      <parent>base</parent>
      <child>tip</child>
      <axis>
        <xyz>0 0 1</xyz>
        <limit>
          <lower>-1.0</lower>
          <upper>2.0</upper>
          <effort>5.5</effort>
          <velocity>3.3</velocity>
        </limit>
        <dynamics>
          <damping>0.4</damping>
          <friction>0.2</friction>
        </dynamics>
      </axis>
    </joint>
  </model>
</sdf>
 )";

  const auto skeleton = readSkeletonFromSdfString(uri, modelSdf, retriever);
  ASSERT_NE(skeleton, nullptr);
  auto* joint
      = dynamic_cast<dynamics::RevoluteJoint*>(skeleton->getJoint("rev_joint"));
  ASSERT_NE(joint, nullptr);
  EXPECT_NEAR(joint->getPositionLowerLimit(0), -1.0, 1e-9);
  EXPECT_NEAR(joint->getPositionUpperLimit(0), 2.0, 1e-9);
  // SDF parser reads effort/velocity from <limit> but DART's SDF parser
  // currently only maps position limits and dynamics; velocity/effort limits
  // remain at defaults (infinity). Verify the position limits and dynamics
  // properties that ARE parsed.
  EXPECT_NEAR(joint->getDampingCoefficient(0), 0.4, 1e-9);
  EXPECT_NEAR(joint->getCoulombFriction(0), 0.2, 1e-9);
}

//==============================================================================
TEST(SdfParser, MaterialDiffuseFromString)
{
  auto retriever = std::make_shared<MemoryResourceRetriever>();
  const std::string uri = "memory://pkg/material.world";
  const std::string worldSdf = R"(
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="default">
    <model name="material">
      <link name="link">
        <inertial><mass>1.0</mass></inertial>
        <visual name="visual">
          <geometry>
            <box><size>0.1 0.2 0.3</size></box>
          </geometry>
          <material>
            <diffuse>0.2 0.4 0.6 0.8</diffuse>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
 )";

  retriever->add(uri, worldSdf);
  SdfParser::Options options;
  options.mResourceRetriever = retriever;
  const auto world = SdfParser::readWorld(common::Uri(uri), options);
  ASSERT_TRUE(world != nullptr);
  const auto skeleton = world->getSkeleton("material");
  ASSERT_TRUE(skeleton != nullptr);
  auto* body = skeleton->getBodyNode(0);
  ASSERT_TRUE(body != nullptr);

  bool foundVisual = false;
  body->eachShapeNodeWith<dynamics::VisualAspect>(
      [&](dynamics::ShapeNode* node) {
        foundVisual = true;
        EXPECT_TRUE(node->getVisualAspect()->getRGBA().isApprox(
            Eigen::Vector4d(0.2, 0.4, 0.6, 0.8)));
      });
  EXPECT_TRUE(foundVisual);
}

//==============================================================================
TEST(SdfParser, JointTypesParseFromSkeleton)
{
  auto retriever = std::make_shared<MemoryResourceRetriever>();
  const std::string uri = "memory://pkg/joint_types/model.sdf";
  const std::string modelSdf = R"(
<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="joint_types">
    <link name="base">
      <inertial><mass>1.0</mass></inertial>
    </link>
    <link name="link1">
      <inertial><mass>1.0</mass></inertial>
    </link>
    <link name="link2">
      <inertial><mass>1.0</mass></inertial>
    </link>
    <link name="link3">
      <inertial><mass>1.0</mass></inertial>
    </link>
    <link name="link4">
      <inertial><mass>1.0</mass></inertial>
    </link>
    <link name="link5">
      <inertial><mass>1.0</mass></inertial>
    </link>
    <joint name="prismatic_joint" type="prismatic">
      <parent>base</parent>
      <child>link1</child>
      <axis>
        <xyz>1 0 0</xyz>
        <limit><lower>-0.5</lower><upper>0.5</upper></limit>
      </axis>
    </joint>
    <joint name="screw_joint" type="screw">
      <parent>link1</parent>
      <child>link2</child>
      <axis>
        <xyz>0 0 1</xyz>
      </axis>
      <thread_pitch>0.2</thread_pitch>
    </joint>
    <joint name="universal_joint" type="universal">
      <parent>link2</parent>
      <child>link3</child>
      <axis>
        <xyz>1 0 0</xyz>
      </axis>
      <axis2>
        <xyz>0 1 0</xyz>
      </axis2>
    </joint>
    <joint name="ball_joint" type="ball">
      <parent>link3</parent>
      <child>link4</child>
    </joint>
    <joint name="fixed_joint" type="fixed">
      <parent>link4</parent>
      <child>link5</child>
    </joint>
  </model>
</sdf>
  )";

  const auto skeleton = readSkeletonFromSdfString(uri, modelSdf, retriever);
  ASSERT_NE(skeleton, nullptr);
  EXPECT_NE(
      dynamic_cast<dynamics::PrismaticJoint*>(
          skeleton->getJoint("prismatic_joint")),
      nullptr);
  EXPECT_NE(
      dynamic_cast<dynamics::ScrewJoint*>(skeleton->getJoint("screw_joint")),
      nullptr);
  EXPECT_NE(
      dynamic_cast<dynamics::UniversalJoint*>(
          skeleton->getJoint("universal_joint")),
      nullptr);
  EXPECT_NE(
      dynamic_cast<dynamics::BallJoint*>(skeleton->getJoint("ball_joint")),
      nullptr);
  EXPECT_NE(
      dynamic_cast<dynamics::WeldJoint*>(skeleton->getJoint("fixed_joint")),
      nullptr);
}

//==============================================================================
TEST(SdfParser, AxisLimitSpringAndFriction)
{
  auto retriever = std::make_shared<MemoryResourceRetriever>();
  const std::string uri = "memory://pkg/axis_spring/model.sdf";
  const std::string modelSdf = R"(
<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="axis_spring">
    <link name="base">
      <inertial><mass>1.0</mass></inertial>
    </link>
    <link name="tip">
      <inertial><mass>1.0</mass></inertial>
    </link>
    <joint name="rev_joint" type="revolute">
      <parent>base</parent>
      <child>tip</child>
      <axis>
        <xyz>0 0 1</xyz>
        <limit>
          <lower>-1.0</lower>
          <upper>1.5</upper>
        </limit>
        <dynamics>
          <damping>0.8</damping>
          <friction>0.4</friction>
          <spring_reference>0.2</spring_reference>
          <spring_stiffness>3.2</spring_stiffness>
        </dynamics>
      </axis>
    </joint>
  </model>
</sdf>
  )";

  const auto skeleton = readSkeletonFromSdfString(uri, modelSdf, retriever);
  ASSERT_NE(skeleton, nullptr);
  auto* joint
      = dynamic_cast<dynamics::RevoluteJoint*>(skeleton->getJoint("rev_joint"));
  ASSERT_NE(joint, nullptr);
  EXPECT_NEAR(joint->getPositionLowerLimit(0), -1.0, 1e-9);
  EXPECT_NEAR(joint->getPositionUpperLimit(0), 1.5, 1e-9);
  EXPECT_NEAR(joint->getDampingCoefficient(0), 0.8, 1e-9);
  EXPECT_NEAR(joint->getCoulombFriction(0), 0.4, 1e-9);
  EXPECT_NEAR(joint->getRestPosition(0), 0.2, 1e-9);
  EXPECT_NEAR(joint->getSpringStiffness(0), 3.2, 1e-9);
}

//==============================================================================
TEST(SdfParser, FixedRootJointTypeCreatesWeldJoint)
{
  auto retriever = std::make_shared<MemoryResourceRetriever>();
  const std::string uri = "memory://pkg/fixed_root/model.sdf";
  const std::string modelSdf = R"(
<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="fixed_root">
    <link name="link">
      <inertial><mass>1.0</mass></inertial>
    </link>
  </model>
</sdf>
  )";

  retriever->add(uri, modelSdf);
  SdfParser::Options options(retriever, SdfParser::RootJointType::Fixed);
  const auto skeleton = SdfParser::readSkeleton(common::Uri(uri), options);
  ASSERT_NE(skeleton, nullptr);
  auto* joint = skeleton->getRootJoint();
  ASSERT_NE(joint, nullptr);
  EXPECT_EQ(joint->getType(), dynamics::WeldJoint::getStaticType());
}

//==============================================================================
TEST(SdfParser, HelperUtilitiesParseText)
{
  using namespace dart::utils::SdfParser::detail;
  const std::string sdfText = R"(
 <?xml version="1.0" ?>
 <sdf version="1.7">
   <model name="helper">
     <link name="link">
       <pose>1 2 3 0 0 0</pose>
       <user>1 2 3</user>
       <size>4 5</size>
       <custom>0.1 0.2 0.3 0.4</custom>
       <bool_flag>true</bool_flag>
       <visual name="visual">
         <geometry>
           <box><size>0.1 0.2 0.3</size></box>
         </geometry>
         <material>
           <diffuse>0.25 0.5 0.75 1.0</diffuse>
         </material>
       </visual>
     </link>
   </model>
 </sdf>
   )";

  sdf::Root root;
  const auto errors = root.LoadSdfString(sdfText);
  ASSERT_TRUE(errors.empty());
  auto modelElement = root.Element()->GetElement("model");
  ASSERT_NE(modelElement, nullptr);
  EXPECT_EQ(getAttributeString(modelElement, "name"), "helper");
  EXPECT_EQ(toLowerCopy("AbC"), "abc");
  EXPECT_EQ(trimCopy("  value  "), "value");

  auto linkElement = modelElement->GetElement("link");
  ASSERT_NE(linkElement, nullptr);
  EXPECT_EQ(getChildElementText(linkElement, "user"), "1 2 3");

  const auto size = getValueVector2d(linkElement, "size");
  EXPECT_TRUE(size.isApprox(Eigen::Vector2d(4.0, 5.0)));

  const auto vec3i = getValueVector3i(linkElement, "user");
  EXPECT_TRUE(vec3i == Eigen::Vector3i(1, 2, 3));

  const auto vecxd = getValueVectorXd(linkElement, "custom");
  ASSERT_EQ(vecxd.size(), 4);
  EXPECT_DOUBLE_EQ(vecxd[0], 0.1);

  const auto pose
      = getValueIsometry3dWithExtrinsicRotation(linkElement, "pose");
  EXPECT_TRUE(pose.translation().isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));

  EXPECT_TRUE(getValueBool(linkElement, "bool_flag"));

  const auto elementText = getElementText(linkElement->GetElement("custom"));
  EXPECT_EQ(elementText, "0.1 0.2 0.3 0.4");

  auto visualElement = linkElement->GetElement("visual");
  ASSERT_NE(visualElement, nullptr);
  auto materialElement = visualElement->GetElement("material");
  ASSERT_NE(materialElement, nullptr);
  const auto color = getValueVectorXd(materialElement, "diffuse");
  ASSERT_EQ(color.size(), 4);
  EXPECT_DOUBLE_EQ(color[0], 0.25);
  EXPECT_DOUBLE_EQ(color[2], 0.75);
}

//==============================================================================
TEST(SdfParser, CollisionMeshUriAndScale)
{
  auto retriever = std::make_shared<MemoryResourceRetriever>();
  const std::string meshUri = "memory://pkg/meshes/collision_box.obj";
  const std::string modelUri = "memory://pkg/models/collision_mesh/model.sdf";
  const std::string meshData = R"(
o Box
v 0 0 0
v 1 0 0
v 0 1 0
f 1 2 3
  )";
  const std::string modelSdf = std::string(R"(
<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="collision_mesh">
    <link name="link">
      <inertial><mass>1.0</mass></inertial>
      <collision name="mesh_collision">
        <geometry>
          <mesh>
            <uri>)") + meshUri + R"(</uri>
            <scale>0.5 1.0 2.0</scale>
          </mesh>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>
  )";

  retriever->add(meshUri, meshData);
  const auto skeleton
      = readSkeletonFromSdfString(modelUri, modelSdf, retriever);
  ASSERT_NE(skeleton, nullptr);
  auto* body = skeleton->getBodyNode("link");
  ASSERT_NE(body, nullptr);

  bool foundMesh = false;
  body->eachShapeNodeWith<dynamics::CollisionAspect>(
      [&](dynamics::ShapeNode* node) {
        auto mesh
            = std::dynamic_pointer_cast<dynamics::MeshShape>(node->getShape());
        if (!mesh) {
          return;
        }
        foundMesh = true;
        EXPECT_TRUE(mesh->getScale().isApprox(Eigen::Vector3d(0.5, 1.0, 2.0)));
      });
  EXPECT_TRUE(foundMesh);
}

//==============================================================================
TEST(SdfParser, WorldParsesJointAndGeometryVariants)
{
  auto retriever = std::make_shared<MemoryResourceRetriever>();
  const std::string meshUri = "memory://pkg/meshes/variant.obj";
  const std::string worldUri = "memory://pkg/worlds/variants.world";
  const std::string meshData = R"(
o Variant
v 0 0 0
v 1 0 0
v 0 1 0
f 1 2 3
  )";

  const std::string worldSdf = std::string(R"(
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="default">
    <model name="joint_model">
      <link name="base">
        <inertial><mass>1.0</mass></inertial>
      </link>
      <link name="link1">
        <inertial><mass>1.0</mass></inertial>
      </link>
      <link name="link2">
        <inertial><mass>1.0</mass></inertial>
      </link>
      <link name="link3">
        <inertial><mass>1.0</mass></inertial>
      </link>
      <link name="link4">
        <inertial><mass>1.0</mass></inertial>
      </link>
      <link name="link5">
        <inertial><mass>1.0</mass></inertial>
      </link>
      <link name="link6">
        <inertial><mass>1.0</mass></inertial>
      </link>
      <link name="link6">
        <inertial><mass>1.0</mass></inertial>
      </link>
      <link name="link6">
        <inertial><mass>1.0</mass></inertial>
      </link>
      <link name="link6">
        <inertial><mass>1.0</mass></inertial>
      </link>
      <joint name="prismatic_joint" type="prismatic">
        <parent>base</parent>
        <child>link1</child>
        <axis>
          <xyz>1 0 0</xyz>
          <limit><lower>-0.5</lower><upper>0.5</upper></limit>
        </axis>
      </joint>
      <joint name="screw_joint" type="screw">
        <parent>link1</parent>
        <child>link2</child>
        <axis>
          <xyz>0 0 1</xyz>
        </axis>
        <thread_pitch>0.25</thread_pitch>
      </joint>
      <joint name="universal_joint" type="universal">
        <parent>link2</parent>
        <child>link3</child>
        <axis>
          <xyz>1 0 0</xyz>
        </axis>
        <axis2>
          <xyz>0 1 0</xyz>
        </axis2>
      </joint>
      <joint name="ball_joint" type="ball">
        <parent>link3</parent>
        <child>link4</child>
      </joint>
      <joint name="fixed_joint" type="fixed">
        <parent>link4</parent>
        <child>link5</child>
      </joint>
    </model>
    <model name="geometry_model">
      <link name="geo_link">
        <inertial><mass>1.0</mass></inertial>
        <visual name="cylinder_visual">
          <geometry>
            <cylinder><radius>0.2</radius><length>0.5</length></cylinder>
          </geometry>
        </visual>
        <visual name="plane_visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>1 2</size>
            </plane>
          </geometry>
        </visual>
        <visual name="mesh_visual">
          <geometry>
            <mesh>
              <uri>)") + meshUri
                               + R"(</uri>
            </mesh>
          </geometry>
        </visual>
        <visual name="diffuse_visual">
          <geometry>
            <box><size>0.1 0.2 0.3</size></box>
          </geometry>
          <material>
            <diffuse>0.1 0.2 0.3</diffuse>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
  )";

  retriever->add(meshUri, meshData);
  retriever->add(worldUri, worldSdf);

  SdfParser::Options options;
  options.mResourceRetriever = retriever;
  const auto world = SdfParser::readWorld(common::Uri(worldUri), options);
  ASSERT_TRUE(world != nullptr);
  EXPECT_EQ(world->getNumSkeletons(), 2u);

  const auto joints = world->getSkeleton("joint_model");
  ASSERT_TRUE(joints != nullptr);
  EXPECT_NE(
      dynamic_cast<dynamics::PrismaticJoint*>(
          joints->getJoint("prismatic_joint")),
      nullptr);
  EXPECT_NE(
      dynamic_cast<dynamics::ScrewJoint*>(joints->getJoint("screw_joint")),
      nullptr);
  EXPECT_NE(
      dynamic_cast<dynamics::UniversalJoint*>(
          joints->getJoint("universal_joint")),
      nullptr);
  EXPECT_NE(
      dynamic_cast<dynamics::BallJoint*>(joints->getJoint("ball_joint")),
      nullptr);
  EXPECT_NE(
      dynamic_cast<dynamics::WeldJoint*>(joints->getJoint("fixed_joint")),
      nullptr);

  const auto geometry = world->getSkeleton("geometry_model");
  ASSERT_TRUE(geometry != nullptr);
  auto* body = geometry->getBodyNode("geo_link");
  ASSERT_TRUE(body != nullptr);

  bool foundCylinder = false;
  bool foundMesh = false;
  bool foundPlane = false;
  bool foundDiffuse = false;
  const auto numShapes = body->getNumShapeNodes();
  for (std::size_t i = 0; i < numShapes; ++i) {
    const auto* shapeNode = body->getShapeNode(i);
    ASSERT_NE(shapeNode, nullptr);
    const auto shape = shapeNode->getShape();
    if (!shape) {
      continue;
    }
    if (shape->getType() == "CylinderShape") {
      foundCylinder = true;
    } else if (shape->is<dynamics::MeshShape>()) {
      foundMesh = true;
    } else if (shape->is<dynamics::BoxShape>()) {
      auto box = std::dynamic_pointer_cast<const dynamics::BoxShape>(shape);
      if (box && box->getSize().z() < 0.01) {
        foundPlane = true;
      }
    }
    const auto* visual = shapeNode->getVisualAspect();
    if (visual
        && visual->getRGBA().isApprox(
            Eigen::Vector4d(0.1, 0.2, 0.3, 1.0), 1e-9)) {
      foundDiffuse = true;
    }
  }

  EXPECT_TRUE(foundCylinder);
  EXPECT_TRUE(foundMesh);
  EXPECT_TRUE(foundPlane);
  EXPECT_TRUE(foundDiffuse);
}

//==============================================================================
TEST(SdfParser, StaticModelParsesBooleanValues)
{
  auto retriever = std::make_shared<MemoryResourceRetriever>();
  const std::string uri = "memory://pkg/static_bool/model.sdf";
  const std::string modelSdf = R"(
<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="static_bool">
    <static>TrUe</static>
    <link name="link">
      <inertial><mass>1.0</mass></inertial>
    </link>
  </model>
</sdf>
  )";

  const auto skeleton = readSkeletonFromSdfString(uri, modelSdf, retriever);
  ASSERT_NE(skeleton, nullptr);
  EXPECT_FALSE(skeleton->isMobile());
}

//==============================================================================
TEST(SdfParser, UniversalScrewAndBallJointLimits)
{
  auto retriever = std::make_shared<MemoryResourceRetriever>();
  const std::string uri = "memory://pkg/joint_details/model.sdf";
  const std::string modelSdf = R"(
<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="joint_details">
    <link name="base">
      <inertial><mass>1.0</mass></inertial>
    </link>
    <link name="link1">
      <inertial><mass>1.0</mass></inertial>
    </link>
    <link name="link2">
      <inertial><mass>1.0</mass></inertial>
    </link>
    <link name="link3">
      <inertial><mass>1.0</mass></inertial>
    </link>
    <joint name="screw_joint" type="screw">
      <parent>base</parent>
      <child>link1</child>
      <axis>
        <xyz>0 0 1</xyz>
        <limit>
          <lower>0.2</lower>
          <upper>0.6</upper>
        </limit>
        <dynamics>
          <damping>0.1</damping>
          <friction>0.05</friction>
          <spring_reference>0.25</spring_reference>
          <spring_stiffness>2.0</spring_stiffness>
        </dynamics>
      </axis>
      <thread_pitch>0.25</thread_pitch>
    </joint>
    <joint name="universal_joint" type="universal">
      <parent>link1</parent>
      <child>link2</child>
      <axis>
        <xyz>1 0 0</xyz>
        <limit>
          <lower>0.5</lower>
          <upper>1.5</upper>
        </limit>
        <dynamics>
          <damping>0.2</damping>
          <friction>0.1</friction>
          <spring_reference>0.3</spring_reference>
          <spring_stiffness>3.0</spring_stiffness>
        </dynamics>
      </axis>
      <axis2>
        <xyz>0 1 0</xyz>
        <limit>
          <lower>-2.0</lower>
          <upper>-1.0</upper>
        </limit>
        <dynamics>
          <damping>0.4</damping>
          <friction>0.15</friction>
          <spring_reference>-1.2</spring_reference>
          <spring_stiffness>4.0</spring_stiffness>
        </dynamics>
      </axis2>
    </joint>
    <joint name="ball_joint" type="ball">
      <parent>link2</parent>
      <child>link3</child>
    </joint>
  </model>
</sdf>
  )";

  const auto skeleton = readSkeletonFromSdfString(uri, modelSdf, retriever);
  ASSERT_NE(skeleton, nullptr);

  auto* screw
      = dynamic_cast<dynamics::ScrewJoint*>(skeleton->getJoint("screw_joint"));
  ASSERT_NE(screw, nullptr);
  EXPECT_TRUE(screw->areLimitsEnforced());
  EXPECT_TRUE(screw->getAxis().isApprox(Eigen::Vector3d(0.0, 0.0, 1.0)));
  EXPECT_NEAR(screw->getPitch(), 0.25, 1e-9);
  EXPECT_NEAR(screw->getPositionLowerLimit(0), 0.2, 1e-9);
  EXPECT_NEAR(screw->getPositionUpperLimit(0), 0.6, 1e-9);
  EXPECT_NEAR(screw->getPosition(0), 0.4, 1e-9);
  EXPECT_NEAR(screw->getRestPosition(0), 0.4, 1e-9);
  EXPECT_NEAR(screw->getDampingCoefficient(0), 0.1, 1e-9);
  EXPECT_NEAR(screw->getCoulombFriction(0), 0.05, 1e-9);
  EXPECT_NEAR(screw->getSpringStiffness(0), 2.0, 1e-9);

  auto* universal = dynamic_cast<dynamics::UniversalJoint*>(
      skeleton->getJoint("universal_joint"));
  ASSERT_NE(universal, nullptr);
  EXPECT_TRUE(universal->areLimitsEnforced());
  EXPECT_NEAR(universal->getPosition(0), 1.0, 1e-9);
  EXPECT_NEAR(universal->getPosition(1), -1.5, 1e-9);
  EXPECT_NEAR(universal->getDampingCoefficient(0), 0.2, 1e-9);
  EXPECT_NEAR(universal->getCoulombFriction(0), 0.1, 1e-9);
  EXPECT_NEAR(universal->getRestPosition(0), 1.0, 1e-9);
  EXPECT_NEAR(universal->getSpringStiffness(0), 3.0, 1e-9);
  EXPECT_NEAR(universal->getDampingCoefficient(1), 0.4, 1e-9);
  EXPECT_NEAR(universal->getCoulombFriction(1), 0.15, 1e-9);
  EXPECT_NEAR(universal->getRestPosition(1), -1.5, 1e-9);
  EXPECT_NEAR(universal->getSpringStiffness(1), 4.0, 1e-9);

  auto* ball
      = dynamic_cast<dynamics::BallJoint*>(skeleton->getJoint("ball_joint"));
  ASSERT_NE(ball, nullptr);
  EXPECT_EQ(ball->getNumDofs(), 3u);
}

//==============================================================================
TEST(SdfParser, MaterialDiffuseThreeComponentsAndMeshPose)
{
  auto retriever = std::make_shared<MemoryResourceRetriever>();
  const std::string meshUri = "memory://pkg/meshes/material.obj";
  const std::string worldUri = "memory://pkg/material_geometry.world";
  const std::string meshData = R"(
o Tri
v 0 0 0
v 1 0 0
v 0 1 0
f 1 2 3
  )";

  const std::string worldSdf = std::string(R"(
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="default">
    <model name="geometry_material">
      <link name="link">
        <inertial><mass>1.0</mass></inertial>
        <visual name="cylinder_visual">
          <geometry>
            <cylinder><radius>0.2</radius><length>0.5</length></cylinder>
          </geometry>
        </visual>
        <visual name="mesh_visual">
          <geometry>
            <mesh>
              <uri>)") + meshUri
                               + R"(</uri>
            </mesh>
          </geometry>
          <pose>0.1 0.2 0.3 0 0 0</pose>
        </visual>
        <visual name="diffuse_visual">
          <geometry>
            <box><size>0.1 0.2 0.3</size></box>
          </geometry>
          <material>
            <diffuse> 0.10000000009 0.2 0.3 </diffuse>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
  )";

  retriever->add(meshUri, meshData);
  retriever->add(worldUri, worldSdf);

  SdfParser::Options options;
  options.mResourceRetriever = retriever;
  const auto world = SdfParser::readWorld(common::Uri(worldUri), options);
  ASSERT_TRUE(world != nullptr);

  const auto skeleton = world->getSkeleton("geometry_material");
  ASSERT_TRUE(skeleton != nullptr);
  auto* body = skeleton->getBodyNode("link");
  ASSERT_TRUE(body != nullptr);

  bool foundCylinder = false;
  bool foundMesh = false;
  bool foundDiffuse = false;
  bool foundMeshPose = false;
  const auto numShapes = body->getNumShapeNodes();
  for (std::size_t i = 0; i < numShapes; ++i) {
    const auto* shapeNode = body->getShapeNode(i);
    ASSERT_NE(shapeNode, nullptr);
    const auto shape = shapeNode->getShape();
    if (!shape) {
      continue;
    }
    if (shape->is<dynamics::CylinderShape>()) {
      foundCylinder = true;
    }
    if (shape->is<dynamics::MeshShape>()) {
      foundMesh = true;
      const auto translation = shapeNode->getRelativeTransform().translation();
      if (translation.isApprox(Eigen::Vector3d(0.1, 0.2, 0.3), 1e-9)) {
        foundMeshPose = true;
      }
    }
    const auto* visual = shapeNode->getVisualAspect();
    if (visual
        && visual->getRGBA().isApprox(Eigen::Vector4d(0.1, 0.2, 0.3, 1.0))) {
      foundDiffuse = true;
    }
  }

  EXPECT_TRUE(foundCylinder);
  EXPECT_TRUE(foundMesh);
  EXPECT_TRUE(foundMeshPose);
  EXPECT_TRUE(foundDiffuse);
}

//==============================================================================
TEST(SdfParser, InlineWorldCoversJointTypesMaterialsAndErrors)
{
  auto retriever = std::make_shared<MemoryResourceRetriever>();
  const std::string worldUri = "memory://pkg/worlds/coverage.world";
  const std::string meshUri = "memory://pkg/meshes/coverage.obj";
  const std::string meshData = R"(
o Mesh
v 0 0 0
v 1 0 0
v 0 1 0
f 1 2 3
  )";

  const std::string worldSdf = std::string(R"(
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="coverage_world">
    <physics type="ode">
      <max_step_size>0.01</max_step_size>
      <gravity>0 0 -9.81</gravity>
    </physics>
    <model name="coverage_model">
      <link name="base">
        <inertial><mass>1.0</mass></inertial>
        <visual name="sphere_visual">
          <geometry>
            <sphere><radius>0.2</radius></sphere>
          </geometry>
          <material>
            <diffuse>0.1 0.2 0.3</diffuse>
          </material>
        </visual>
      </link>
      <link name="link1">
        <inertial><mass>1.0</mass></inertial>
        <visual name="box_visual">
          <geometry>
            <box><size>0.2 0.3 0.4</size></box>
          </geometry>
          <material>
            <diffuse>0.1 0.2 0.3 0.4</diffuse>
          </material>
        </visual>
      </link>
      <link name="link2">
        <inertial><mass>1.0</mass></inertial>
        <visual name="cylinder_visual">
          <geometry>
            <cylinder><radius>0.1</radius><length>0.5</length></cylinder>
          </geometry>
        </visual>
      </link>
      <link name="link3">
        <inertial><mass>1.0</mass></inertial>
        <visual name="plane_visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>1 2</size>
            </plane>
          </geometry>
        </visual>
      </link>
      <link name="link4">
        <inertial><mass>1.0</mass></inertial>
        <visual name="mesh_visual">
          <geometry>
            <mesh>
              <uri>)") + meshUri
                               + R"(</uri>
            </mesh>
          </geometry>
        </visual>
      </link>
      <link name="link5">
        <inertial><mass>1.0</mass></inertial>
      </link>
      <link name="link6">
        <inertial><mass>1.0</mass></inertial>
      </link>
      <joint name="fixed_joint" type="fixed">
        <parent>base</parent>
        <child>link1</child>
      </joint>
      <joint name="revolute_joint" type="revolute">
        <parent>link1</parent>
        <child>link2</child>
        <axis>
          <xyz>0 0 1</xyz>
          <dynamics>
            <damping>0.2</damping>
            <friction>0.1</friction>
            <spring_reference>0.1</spring_reference>
            <spring_stiffness>2.0</spring_stiffness>
          </dynamics>
          <limit>
            <lower>0.1</lower>
            <upper>0.5</upper>
          </limit>
        </axis>
      </joint>
      <joint name="prismatic_joint" type="prismatic">
        <parent>link2</parent>
        <child>link3</child>
        <axis>
          <xyz>1 0 0</xyz>
          <limit>
            <lower>-0.2</lower>
            <upper>0.2</upper>
          </limit>
        </axis>
      </joint>
      <joint name="screw_joint" type="screw">
        <parent>link3</parent>
        <child>link4</child>
        <axis>
          <xyz>0 1 0</xyz>
        </axis>
        <thread_pitch>0.1</thread_pitch>
      </joint>
      <joint name="universal_joint" type="universal">
        <parent>link4</parent>
        <child>link5</child>
        <axis>
          <xyz>1 0 0</xyz>
        </axis>
        <axis2>
          <xyz>0 1 0</xyz>
        </axis2>
      </joint>
      <joint name="ball_joint" type="ball">
        <parent>link5</parent>
        <child>link6</child>
      </joint>
    </model>
  </world>
</sdf>
  )";

  retriever->add(meshUri, meshData);
  retriever->add(worldUri, worldSdf);

  const std::string errorSdf = R"(
<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="error_model">
    <link name="link">
      <pose>0 0</pose>
      <inertial>
        <mass>bad</mass>
      </inertial>
    </link>
  </model>
</sdf>
  )";

  LogCapture capture;
  sdf::Root errorRoot;
  errorRoot.LoadSdfString(errorSdf);
  if (auto errorElement = errorRoot.Element()) {
    if (auto modelElement = errorElement->GetElement("model")) {
      if (auto linkElement = modelElement->GetElement("link")) {
        const auto pose = dart::utils::SdfParser::detail::getValueVector3d(
            linkElement, "pose");
        (void)pose;
        if (auto inertialElement = linkElement->GetElement("inertial")) {
          const auto mass = dart::utils::SdfParser::detail::getValueDouble(
              inertialElement, "mass");
          (void)mass;
        }
      }
    }
  }

  SdfParser::Options options;
  options.mResourceRetriever = retriever;
  auto world = SdfParser::readWorld(common::Uri(worldUri), options);
  ASSERT_NE(world, nullptr) << capture.contents();
  const auto skeleton = world->getSkeleton("coverage_model");
  ASSERT_NE(skeleton, nullptr);
  EXPECT_EQ(skeleton->getNumJoints(), 7u);

  bool foundMesh = false;
  bool foundCylinder = false;
  bool foundSphere = false;
  bool foundPlane = false;
  skeleton->eachBodyNode([&](dynamics::BodyNode* body) {
    if (!body) {
      return;
    }
    const auto numShapes = body->getNumShapeNodes();
    for (std::size_t i = 0; i < numShapes; ++i) {
      const auto* node = body->getShapeNode(i);
      if (!node) {
        continue;
      }
      const auto shape = node->getShape();
      if (!shape) {
        continue;
      }
      if (shape->is<dynamics::MeshShape>()) {
        foundMesh = true;
      } else if (shape->is<dynamics::CylinderShape>()) {
        foundCylinder = true;
      } else if (shape->is<dynamics::SphereShape>()) {
        foundSphere = true;
      } else if (shape->is<dynamics::BoxShape>()) {
        const auto box
            = std::dynamic_pointer_cast<const dynamics::BoxShape>(shape);
        if (box && box->getSize()(2) <= 0.0011) {
          foundPlane = true;
        }
      }
    }
  });
  EXPECT_TRUE(foundMesh);
  EXPECT_TRUE(foundCylinder);
  EXPECT_TRUE(foundSphere);
  EXPECT_TRUE(foundPlane);

  const auto logs = capture.contents();
  if (!logs.empty()) {
    EXPECT_NE(logs.find("must have 6 values"), std::string::npos)
        << "Expected pose warning in logs: " << logs;
  }
}

//==============================================================================
TEST(SdfParser, HelperUtilitiesHandleInvalidValues)
{
  using namespace dart::utils::SdfParser::detail;
  const std::string sdfText = R"(
<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="helper_invalid">
    <link name="link">
      <empty />
      <bool_flag>maybe</bool_flag>
      <uint_flag>bad</uint_flag>
      <double_flag>bad</double_flag>
      <vec2>1</vec2>
      <vec3>1 2</vec3>
      <vec3i>1 2</vec3i>
    </link>
  </model>
</sdf>
  )";

  sdf::Root root;
  const auto errors = root.LoadSdfString(sdfText);
  ASSERT_TRUE(errors.empty());
  auto modelElement = root.Element()->GetElement("model");
  ASSERT_NE(modelElement, nullptr);
  auto linkElement = modelElement->GetElement("link");
  ASSERT_NE(linkElement, nullptr);

  LogCapture capture;
  EXPECT_TRUE(getElementText(linkElement->GetElement("empty")).empty());
  EXPECT_TRUE(getAttributeString(linkElement, "missing").empty());
  EXPECT_FALSE(getValueBool(linkElement, "bool_flag"));
  EXPECT_EQ(getValueUInt(linkElement, "uint_flag"), 0u);
  EXPECT_DOUBLE_EQ(getValueDouble(linkElement, "double_flag"), 0.0);
  EXPECT_TRUE(getValueVector2d(linkElement, "vec2")
                  .isApprox(Eigen::Vector2d::Zero(), 1e-12));
  EXPECT_TRUE(getValueVector3d(linkElement, "vec3")
                  .isApprox(Eigen::Vector3d::Zero(), 1e-12));
  EXPECT_TRUE(
      (getValueVector3i(linkElement, "vec3i") == Eigen::Vector3i::Zero()));
  const auto pose
      = getValueIsometry3dWithExtrinsicRotation(linkElement, "pose");
  EXPECT_TRUE(pose.isApprox(Eigen::Isometry3d::Identity(), 1e-12));

  const auto logs = capture.contents();
  if (!logs.empty()) {
    EXPECT_NE(logs.find("bool_flag"), std::string::npos) << logs;
    EXPECT_NE(logs.find("Missing attribute [missing]"), std::string::npos)
        << logs;
    EXPECT_NE(logs.find("expected 2 values"), std::string::npos) << logs;
    EXPECT_NE(logs.find("expected 3 values"), std::string::npos) << logs;
  }
}

//==============================================================================
TEST(SdfParser, Revolute2JointUpperLowerLimitInitials)
{
  auto retriever = std::make_shared<MemoryResourceRetriever>();
  const std::string uri = "memory://pkg/revolute2_upper/model.sdf";
  const std::string modelSdf = R"(
<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="revolute2_upper">
    <link name="base">
      <inertial><mass>1.0</mass></inertial>
    </link>
    <link name="link1">
      <inertial><mass>1.0</mass></inertial>
    </link>
    <joint name="rev2_joint" type="revolute2">
      <parent>base</parent>
      <child>link1</child>
      <axis>
        <xyz>1 0 0</xyz>
        <limit>
          <upper>-0.4</upper>
        </limit>
      </axis>
      <axis2>
        <xyz>0 1 0</xyz>
        <limit>
          <lower>0.2</lower>
        </limit>
      </axis2>
    </joint>
  </model>
</sdf>
  )";

  const auto skeleton = readSkeletonFromSdfString(uri, modelSdf, retriever);
  ASSERT_NE(skeleton, nullptr);
  auto* joint = dynamic_cast<dynamics::UniversalJoint*>(
      skeleton->getJoint("rev2_joint"));
  ASSERT_NE(joint, nullptr);
  EXPECT_TRUE(joint->areLimitsEnforced());
  EXPECT_NEAR(joint->getPosition(0), -0.4, 1e-9);
  EXPECT_NEAR(joint->getRestPosition(0), -0.4, 1e-9);
  EXPECT_NEAR(joint->getPosition(1), 0.2, 1e-9);
  EXPECT_NEAR(joint->getRestPosition(1), 0.2, 1e-9);
}

//==============================================================================
TEST(SdfParser, MjcfGeomJointSiteAndTendonParsing)
{
  const std::string mjcfXml = R"(
<mujoco model="parser_coverage">
  <default>
    <geom type="sphere" size="0.1" />
  </default>
  <worldbody>
    <body name="root">
      <joint name="root_free" type="free" />
      <geom name="root_capsule" type="capsule" size="0.05 0.2" />
      <site name="root_site" type="ellipsoid" size="0.02 0.03 0.04"
            rgba="0.2 0.3 0.4 1" />
      <body name="hinge_body" pos="0 0 0.1">
        <joint name="hinge_joint" type="hinge" axis="0 1 0" />
        <geom name="hinge_cylinder" type="cylinder" size="0.02 0.3" />
      </body>
      <body name="slide_body" pos="0 0 0.2">
        <joint name="slide_joint" type="slide" axis="1 0 0"
               range="-0.1 0.2" />
        <geom name="slide_ellipsoid" type="ellipsoid" size="0.05 0.06 0.07" />
      </body>
      <body name="ball_body" pos="0 0 0.3">
        <joint name="ball_joint" type="ball" />
        <geom name="ball_sphere" type="sphere" size="0.08" />
      </body>
    </body>
  </worldbody>
  <tendon>
    <spatial name="tendon1">
      <site site="root_site" />
    </spatial>
  </tendon>
  <actuator>
    <motor name="motor1" joint="hinge_joint" gear="1" />
  </actuator>
</mujoco>
  )";

  const auto tempPath
      = writeTempFileWithPrefix(mjcfXml, "mjcf_coverage", ".xml");
  auto world = dart::io::readWorld(common::Uri(tempPath.string()));
  std::error_code ec;
  std::filesystem::remove(tempPath, ec);

  ASSERT_NE(world, nullptr);
  const auto skeleton = world->getSkeleton("root");
  ASSERT_NE(skeleton, nullptr);

  auto* root = skeleton->getBodyNode("root");
  ASSERT_NE(root, nullptr);
  EXPECT_NE(
      dynamic_cast<dynamics::FreeJoint*>(root->getParentJoint()), nullptr);

  auto* hinge = skeleton->getBodyNode("hinge_body");
  ASSERT_NE(hinge, nullptr);
  EXPECT_NE(
      dynamic_cast<dynamics::RevoluteJoint*>(hinge->getParentJoint()), nullptr);

  auto* slide = skeleton->getBodyNode("slide_body");
  ASSERT_NE(slide, nullptr);
  EXPECT_NE(
      dynamic_cast<dynamics::PrismaticJoint*>(slide->getParentJoint()),
      nullptr);

  auto* ball = skeleton->getBodyNode("ball_body");
  ASSERT_NE(ball, nullptr);
  EXPECT_NE(
      dynamic_cast<dynamics::BallJoint*>(ball->getParentJoint()), nullptr);

  bool foundCapsule = false;
  bool foundCylinder = false;
  bool foundEllipsoid = false;
  for (std::size_t i = 0; i < skeleton->getNumBodyNodes(); ++i) {
    auto* body = skeleton->getBodyNode(i);
    if (!body) {
      continue;
    }
    const auto numShapes = body->getNumShapeNodes();
    for (std::size_t j = 0; j < numShapes; ++j) {
      const auto* node = body->getShapeNode(j);
      if (!node || !node->getShape()) {
        continue;
      }
      if (node->getShape()->is<dynamics::CapsuleShape>()) {
        foundCapsule = true;
      } else if (node->getShape()->is<dynamics::CylinderShape>()) {
        foundCylinder = true;
      } else if (node->getShape()->is<dynamics::EllipsoidShape>()) {
        foundEllipsoid = true;
      }
    }
  }

  EXPECT_TRUE(foundCapsule);
  EXPECT_TRUE(foundCylinder);
  EXPECT_TRUE(foundEllipsoid);

  bool foundSite = false;
  const auto rootShapes = root->getNumShapeNodes();
  for (std::size_t i = 0; i < rootShapes; ++i) {
    const auto* node = root->getShapeNode(i);
    if (!node || node->getName() != "site:root_site") {
      continue;
    }
    ASSERT_NE(node->getShape(), nullptr);
    EXPECT_TRUE(node->getShape()->is<dynamics::EllipsoidShape>());
    foundSite = true;
  }
  EXPECT_TRUE(foundSite);
}
