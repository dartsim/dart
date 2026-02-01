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

#include "dart/dynamics/box_shape.hpp"
#include "dart/dynamics/cylinder_shape.hpp"
#include "dart/dynamics/free_joint.hpp"
#include "dart/dynamics/mesh_shape.hpp"
#include "dart/dynamics/planar_joint.hpp"
#include "dart/dynamics/prismatic_joint.hpp"
#include "dart/dynamics/sphere_shape.hpp"
#include "dart/dynamics/weld_joint.hpp"
#include "dart/simulation/world.hpp"
#include "dart/utils/urdf/urdf_parser.hpp"

#include <dart/all.hpp>

#if DART_IO_HAS_URDF
  #include "dart/config.hpp"
  #include "dart/dynamics/revolute_joint.hpp"
  #include "dart/io/read.hpp"
#endif

#include <Eigen/Core>
#include <Eigen/Dense>
#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <map>
#include <string>

#include <cstring>

using namespace dart;
using namespace dart::test;
using dart::common::Uri;
using dart::utils::UrdfParser;

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

std::filesystem::path makeTempDir(const std::string& tag)
{
  const auto stamp
      = std::chrono::steady_clock::now().time_since_epoch().count();
  const auto dir = std::filesystem::temp_directory_path()
                   / (tag + "-" + std::to_string(stamp));
  std::filesystem::create_directories(dir);
  return dir;
}

common::Uri addMemoryFile(
    MemoryResourceRetriever& retriever, std::string_view uri, std::string data)
{
  retriever.add(std::string(uri), std::move(data));
  return common::Uri(std::string(uri));
}

#if DART_IO_HAS_URDF
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
#endif

} // namespace

//==============================================================================
TEST(UrdfParser, parseSkeleton_NonExistantPathReturnsNull)
{
  UrdfParser parser;
  EXPECT_EQ(
      nullptr,
      parser.parseSkeleton("dart://sample/skel/test/does_not_exist.urdf"));
}

//==============================================================================
TEST(UrdfParser, parseSkeleton_InvalidUrdfReturnsNull)
{
  UrdfParser parser;
  EXPECT_EQ(
      nullptr, parser.parseSkeleton("dart://sample/urdf/test/invalid.urdf)"));
}

//==============================================================================
TEST(UrdfParser, parseSkeleton_MissingMeshReturnsNull)
{
  UrdfParser parser;
  EXPECT_EQ(
      nullptr,
      parser.parseSkeleton("dart://sample/urdf/test/missing_mesh.urdf"));
}

//==============================================================================
TEST(UrdfParser, parseSkeleton_InvalidMeshReturnsNull)
{
  UrdfParser parser;
  EXPECT_EQ(
      nullptr,
      parser.parseSkeleton("dart://sample/urdf/test/invalid_mesh.urdf"));
}

//==============================================================================
TEST(UrdfParser, parseSkeleton_MissingPackageReturnsNull)
{
  UrdfParser parser;
  EXPECT_EQ(
      nullptr,
      parser.parseSkeleton("dart://sample/urdf/test/missing_package.urdf"));
}

//==============================================================================
TEST(UrdfParser, parseSkeleton_LoadsPrimitiveGeometry)
{
  UrdfParser parser;
  EXPECT_TRUE(
      nullptr
      != parser.parseSkeleton(
          "dart://sample/urdf/test/primitive_geometry.urdf"));
}

//==============================================================================
TEST(UrdfParser, parseWorld)
{
  UrdfParser parser;
  EXPECT_TRUE(
      nullptr != parser.parseWorld("dart://sample/urdf/test/testWorld.urdf"));
}

//==============================================================================
TEST(UrdfParser, parseWorld_ResolvesIncludedModel)
{
  UrdfParser parser;

  const auto tempDir = makeTempDir("dart-urdf-world");
  const auto modelPath = tempDir / "model.urdf";

  std::ofstream modelFile(modelPath);
  ASSERT_TRUE(modelFile.is_open());
  modelFile << "<robot name=\"model\"><link name=\"link\"/></robot>";
  modelFile.close();

  const std::string worldXml = R"(
<world name="world">
  <include filename="model.urdf" model_name="model1"/>
  <entity name="entity1" model="model1"/>
</world>
)";

  const Uri baseUri = Uri::createFromPath((tempDir / "world.urdf").string());
  EXPECT_TRUE(parser.parseWorldString(worldXml, baseUri) != nullptr);

  std::filesystem::remove_all(tempDir);
}

//==============================================================================
TEST(UrdfParser, parseWorld_MissingIncludeReturnsNull)
{
  UrdfParser parser;

  const auto tempDir = makeTempDir("dart-urdf-world-missing");

  const std::string worldXml = R"(
<world name="world">
  <include filename="model.urdf" model_name="model1"/>
  <entity name="entity1" model="missing"/>
</world>
)";

  const Uri baseUri = Uri::createFromPath((tempDir / "world.urdf").string());
  EXPECT_EQ(parser.parseWorldString(worldXml, baseUri), nullptr);

  std::filesystem::remove_all(tempDir);
}

//==============================================================================
TEST(UrdfParser, parseJointProperties)
{
  // clang-format off
  std::string urdfStr = R"(
    <robot name="testRobot">
      <link name="link_0">
        <visual>
          <origin rpy="0 0 0" xyz="0 0 -0.087" />
          <geometry>
            <box size="1 1 1" />
          </geometry>
        </visual>
        <visual>
          <origin rpy="0 0 0" xyz="0 0 0" />
          <geometry>
            <box size="1 1 1" />
          </geometry>
        </visual>
      </link>
      <joint name="0_to_1" type="revolute">
        <parent link="link_0" />
        <child link="link_1" />
        <limit effort="2.5" lower="-3.14159265359"
               upper="3.14159265359" velocity="3.00545697193" />
        <axis xyz="0 0 1" />
        <dynamics damping="1.2" friction="2.3" />
      </joint>
      <link name="link_1">
        <visual>
          <origin rpy="0 0 0" xyz="0 0 -0.087" />
          <geometry>
            <box size="1 1 1" />
          </geometry>
        </visual>
        <visual>
          <origin rpy="0 0 0" xyz="0 0 0" />
          <geometry>
            <box size="1 1 1" />
          </geometry>
        </visual>
      </link>
      <joint name="1_to_2" type="continuous">
        <parent link="link_1" />
        <child link="link_2" />
        <limit effort="2.5" velocity="3.00545697193" />
        <axis xyz="0 0 1" />
        <dynamics damping="1.2" friction="2.3" />
      </joint>
      <link name="link_2">
        <visual>
          <origin rpy="0 0 0" xyz="0 0 -0.087" />
          <geometry>
            <box size="1 1 1" />
          </geometry>
        </visual>
        <visual>
          <origin rpy="0 0 0" xyz="0 0 0" />
          <geometry>
            <box size="1 1 1" />
          </geometry>
        </visual>
      </link>
    </robot>
  )";
  // clang-format on

  UrdfParser parser;
  auto robot = parser.parseSkeletonString(urdfStr, "");
  ASSERT_TRUE(nullptr != robot);

  auto joint1 = robot->getJoint(1);
  ASSERT_TRUE(nullptr != joint1);
  EXPECT_NEAR(joint1->getDampingCoefficient(0), 1.2, 1e-12);
  EXPECT_NEAR(joint1->getCoulombFriction(0), 2.3, 1e-12);

  auto joint2 = robot->getJoint(2);
  EXPECT_DOUBLE_EQ(joint2->getPositionLowerLimit(0), -dart::math::inf);
  EXPECT_DOUBLE_EQ(joint2->getPositionUpperLimit(0), dart::math::inf);
  EXPECT_TRUE(joint2->isCyclic(0));
}

#if DART_IO_HAS_URDF
//==============================================================================
TEST(UrdfParser, ParsesMimicJoint)
{
  const std::string urdfXml = R"(
<robot name="mimic_robot">
  <link name="base" />
  <link name="link1" />
  <link name="link2" />
  <joint name="joint1" type="revolute">
    <parent link="base" />
    <child link="link1" />
    <axis xyz="0 0 1" />
    <limit lower="-1" upper="1" effort="1" velocity="1" />
  </joint>
  <joint name="joint2" type="revolute">
    <parent link="link1" />
    <child link="link2" />
    <axis xyz="0 1 0" />
    <limit lower="-1" upper="1" effort="1" velocity="1" />
    <mimic joint="joint1" multiplier="2" offset="0.5" />
  </joint>
</robot>
 )";

  const auto tempPath = writeTempFile(urdfXml, "urdf_mimic", ".urdf");
  auto skel = dart::io::readSkeleton(common::Uri(tempPath.string()));
  std::error_code ec;
  std::filesystem::remove(tempPath, ec);

  ASSERT_NE(skel, nullptr);
  auto* joint1 = skel->getJoint("joint1");
  auto* joint2 = skel->getJoint("joint2");
  ASSERT_NE(joint1, nullptr);
  ASSERT_NE(joint2, nullptr);
  EXPECT_EQ(joint2->getActuatorType(), dynamics::Joint::MIMIC);
  EXPECT_EQ(joint2->getMimicJoint(), joint1);
}

//==============================================================================
TEST(UrdfParser, ParsesPlanarAndFloatingJoints)
{
  const std::string urdfXml = R"(
<robot name="planar_floating_robot">
  <link name="base" />
  <link name="link1" />
  <link name="link2" />
  <joint name="planar_joint" type="planar">
    <parent link="base" />
    <child link="link1" />
    <origin xyz="0 0 0" rpy="0 0 0" />
    <axis xyz="0 0 1" />
  </joint>
  <joint name="floating_joint" type="floating">
    <parent link="link1" />
    <child link="link2" />
    <origin xyz="0 0 0" rpy="0 0 0" />
  </joint>
</robot>
 )";

  const auto tempPath = writeTempFile(urdfXml, "urdf_joint", ".urdf");
  auto skel = dart::io::readSkeleton(common::Uri(tempPath.string()));
  std::error_code ec;
  std::filesystem::remove(tempPath, ec);

  ASSERT_NE(skel, nullptr);
  auto* planar
      = dynamic_cast<dynamics::PlanarJoint*>(skel->getJoint("planar_joint"));
  auto* floating
      = dynamic_cast<dynamics::FreeJoint*>(skel->getJoint("floating_joint"));
  ASSERT_NE(planar, nullptr);
  ASSERT_NE(floating, nullptr);
  EXPECT_EQ(planar->getPlaneType(), dynamics::PlanarJoint::PlaneType::XY);
}
#endif

//==============================================================================
TEST(UrdfParser, parsePlanarJointLimitsAndAxis)
{
  // clang-format off
  const std::string urdfStr = R"(
    <robot name="planar_example">
      <link name="base"/>
      <link name="tip"/>
      <joint name="planar_joint" type="planar">
        <parent link="base"/>
        <child link="tip"/>
        <limit lower="0.1" upper="1.1" velocity="2.5" effort="3.5"/>
        <axis xyz="0 0 1"/>
        <dynamics damping="0.3" friction="0.7"/>
      </joint>
    </robot>
  )";
  // clang-format on

  UrdfParser parser;
  auto robot = parser.parseSkeletonString(urdfStr, "");
  ASSERT_TRUE(robot);

  auto* joint
      = dynamic_cast<dynamics::PlanarJoint*>(robot->getJoint("planar_joint"));
  ASSERT_TRUE(joint);

  EXPECT_EQ(joint->getPlaneType(), dynamics::PlanarJoint::PlaneType::XY);
  EXPECT_TRUE(
      joint->getTranslationalAxis1().isApprox(Eigen::Vector3d::UnitX()));
  EXPECT_TRUE(
      joint->getTranslationalAxis2().isApprox(Eigen::Vector3d::UnitY()));
  EXPECT_TRUE(joint->getRotationalAxis().isApprox(Eigen::Vector3d::UnitZ()));

  const Eigen::VectorXd lower
      = Eigen::VectorXd::Constant(joint->getNumDofs(), 0.1);
  const Eigen::VectorXd upper
      = Eigen::VectorXd::Constant(joint->getNumDofs(), 1.1);
  const Eigen::VectorXd velocityUpper
      = Eigen::VectorXd::Constant(joint->getNumDofs(), 2.5);
  const Eigen::VectorXd effortUpper
      = Eigen::VectorXd::Constant(joint->getNumDofs(), 3.5);

  EXPECT_TRUE(joint->getPositionLowerLimits().isApprox(lower));
  EXPECT_TRUE(joint->getPositionUpperLimits().isApprox(upper));
  EXPECT_TRUE(joint->getVelocityLowerLimits().isApprox(-velocityUpper));
  EXPECT_TRUE(joint->getVelocityUpperLimits().isApprox(velocityUpper));
  EXPECT_TRUE(joint->getForceLowerLimits().isApprox(-effortUpper));
  EXPECT_TRUE(joint->getForceUpperLimits().isApprox(effortUpper));
  EXPECT_TRUE(joint->getRestPositions().isApprox(
      Eigen::VectorXd::Constant(joint->getNumDofs(), 0.6)));
  EXPECT_TRUE(joint->getDampingCoefficients().isConstant(0.3));
  EXPECT_TRUE(joint->getFrictions().isConstant(0.7));
}

//==============================================================================
TEST(UrdfParser, parsePlanarJointArbitraryAxis)
{
  // clang-format off
  const std::string urdfStr = R"(
    <robot name="planar_example">
      <link name="base"/>
      <link name="tip"/>
      <joint name="planar_joint" type="planar">
        <parent link="base"/>
        <child link="tip"/>
        <axis xyz="1 1 1"/>
      </joint>
    </robot>
  )";
  // clang-format on

  UrdfParser parser;
  auto robot = parser.parseSkeletonString(urdfStr, "");
  ASSERT_TRUE(robot);

  auto* joint
      = dynamic_cast<dynamics::PlanarJoint*>(robot->getJoint("planar_joint"));
  ASSERT_TRUE(joint);

  const Eigen::Vector3d axis = Eigen::Vector3d::Ones().normalized();
  EXPECT_TRUE(joint->getRotationalAxis().isApprox(axis));
  EXPECT_NEAR(joint->getTranslationalAxis1().dot(axis), 0.0, 1e-12);
  EXPECT_NEAR(joint->getTranslationalAxis2().dot(axis), 0.0, 1e-12);
  EXPECT_NEAR(
      joint->getTranslationalAxis1().dot(joint->getTranslationalAxis2()),
      0.0,
      1e-12);
  EXPECT_NEAR(joint->getTranslationalAxis1().norm(), 1.0, 1e-12);
  EXPECT_NEAR(joint->getTranslationalAxis2().norm(), 1.0, 1e-12);
}

//==============================================================================
TEST(UrdfParser, parseFloatingJointLimits)
{
  // clang-format off
  const std::string urdfStr = R"(
    <robot name="floating_example">
      <link name="base"/>
      <link name="child"/>
      <joint name="floating_joint" type="floating">
        <parent link="base"/>
        <child link="child"/>
        <limit lower="0.1" upper="0.2" velocity="1.1" effort="2.2"/>
        <dynamics damping="0.4" friction="0.5"/>
      </joint>
    </robot>
  )";
  // clang-format on

  UrdfParser parser;
  auto robot = parser.parseSkeletonString(urdfStr, "");
  ASSERT_TRUE(robot);

  auto* joint
      = dynamic_cast<dynamics::FreeJoint*>(robot->getJoint("floating_joint"));
  ASSERT_TRUE(joint);

  const Eigen::VectorXd lower
      = Eigen::VectorXd::Constant(joint->getNumDofs(), 0.1);
  const Eigen::VectorXd upper
      = Eigen::VectorXd::Constant(joint->getNumDofs(), 0.2);
  const Eigen::VectorXd velocityUpper
      = Eigen::VectorXd::Constant(joint->getNumDofs(), 1.1);
  const Eigen::VectorXd effortUpper
      = Eigen::VectorXd::Constant(joint->getNumDofs(), 2.2);

  EXPECT_TRUE(joint->getPositionLowerLimits().isApprox(lower));
  EXPECT_TRUE(joint->getPositionUpperLimits().isApprox(upper));
  EXPECT_TRUE(joint->getVelocityLowerLimits().isApprox(-velocityUpper));
  EXPECT_TRUE(joint->getVelocityUpperLimits().isApprox(velocityUpper));
  EXPECT_TRUE(joint->getForceLowerLimits().isApprox(-effortUpper));
  EXPECT_TRUE(joint->getForceUpperLimits().isApprox(effortUpper));
  EXPECT_TRUE(joint->getRestPositions().isConstant(0.15));
  EXPECT_TRUE(joint->getDampingCoefficients().isConstant(0.4));
  EXPECT_TRUE(joint->getFrictions().isConstant(0.5));
}

//==============================================================================
TEST(UrdfParser, parseUrdfWithoutWorldLink)
{
  // clang-format off
  const std::string urdfStr = R"(
    <robot name="testRobot">
      <link name="link_0" />
      <joint name="0_to_1" type="revolute">
        <parent link="link_0" />
        <child link="link_1" />
        <limit effort="2.5" lower="-3.14159265359"
               upper="3.14159265359" velocity="3.00545697193" />
        <axis xyz="0 0 1" />
      </joint>
      <link name="link_1" />
    </robot>
  )";
  // clang-format on

  UrdfParser parser;
  UrdfParser::Options options;

  // Default
  auto robot1 = parser.parseSkeletonString(urdfStr, "");
  ASSERT_TRUE(nullptr != robot1);
  EXPECT_EQ(
      robot1->getRootJoint()->getType(), dynamics::FreeJoint::getStaticType());
  EXPECT_EQ(robot1->getRootJoint()->getName(), "rootJoint");

  // Floating
  options.mDefaultRootJointType = UrdfParser::RootJointType::Floating;
  parser.setOptions(options);
  auto robot2 = parser.parseSkeletonString(urdfStr, "");
  ASSERT_TRUE(nullptr != robot2);
  EXPECT_EQ(
      robot2->getRootJoint()->getType(), dynamics::FreeJoint::getStaticType());
  EXPECT_EQ(robot2->getRootJoint()->getName(), "rootJoint");

  // Fixed
  options.mDefaultRootJointType = UrdfParser::RootJointType::Fixed;
  parser.setOptions(options);
  auto robot3 = parser.parseSkeletonString(urdfStr, "");
  ASSERT_TRUE(nullptr != robot3);
  EXPECT_EQ(
      robot3->getRootJoint()->getType(), dynamics::WeldJoint::getStaticType());
  EXPECT_EQ(robot3->getRootJoint()->getName(), "rootJoint");
}

//==============================================================================
TEST(UrdfParser, mimicJoint)
{
  // clang-format off
  std::string urdfStr = R"(
    <robot name="testRobot">
      <link name="link_0">
        <visual>
          <origin rpy="0 0 0" xyz="0 0 -0.087" />
          <geometry>
            <box size="1 1 1" />
          </geometry>
        </visual>
        <visual>
          <origin rpy="0 0 0" xyz="0 0 0" />
          <geometry>
            <box size="1 1 1" />
          </geometry>
        </visual>
      </link>
      <joint name="0_to_1" type="revolute">
        <parent link="link_0" />
        <child link="link_1" />
        <limit effort="2.5" lower="-3.14159265359"
               upper="3.14159265359" velocity="3.00545697193" />
        <axis xyz="0 0 1" />
        <dynamics damping="1.2" friction="2.3" />
      </joint>
      <link name="link_1">
        <visual>
          <origin rpy="0 0 0" xyz="0 0 -0.087" />
          <geometry>
            <box size="1 1 1" />
          </geometry>
        </visual>
        <visual>
          <origin rpy="0 0 0" xyz="0 0 0" />
          <geometry>
            <box size="1 1 1" />
          </geometry>
        </visual>
      </link>
      <joint name="1_to_2" type="continuous">
        <parent link="link_1" />
        <child link="link_2" />
        <limit effort="2.5" velocity="3.00545697193" />
        <axis xyz="0 0 1" />
        <dynamics damping="1.2" friction="2.3" />
        <mimic joint="0_to_1" multiplier="2." offset="0.1" />
      </joint>
      <link name="link_2">
        <visual>
          <origin rpy="0 0 0" xyz="0 0 -0.087" />
          <geometry>
            <box size="1 1 1" />
          </geometry>
        </visual>
        <visual>
          <origin rpy="0 0 0" xyz="0 0 0" />
          <geometry>
            <box size="1 1 1" />
          </geometry>
        </visual>
      </link>
    </robot>
  )";
  // clang-format on

  UrdfParser parser;
  auto robot = parser.parseSkeletonString(urdfStr, "");
  ASSERT_TRUE(nullptr != robot);

  auto joint1 = robot->getJoint(1);
  ASSERT_TRUE(nullptr != joint1);
  EXPECT_NEAR(joint1->getDampingCoefficient(0), 1.2, 1e-12);
  EXPECT_NEAR(joint1->getCoulombFriction(0), 2.3, 1e-12);

  auto joint2 = robot->getJoint(2);
  EXPECT_DOUBLE_EQ(joint2->getPositionLowerLimit(0), -dart::math::inf);
  EXPECT_DOUBLE_EQ(joint2->getPositionUpperLimit(0), dart::math::inf);
  EXPECT_TRUE(joint2->isCyclic(0));

  EXPECT_TRUE(joint2->getActuatorType() == dart::dynamics::Joint::MIMIC);
  EXPECT_TRUE(nullptr != joint2->getMimicJoint());
  EXPECT_DOUBLE_EQ(joint2->getMimicMultiplier(), 2.);
  EXPECT_DOUBLE_EQ(joint2->getMimicOffset(), 0.1);
}

//==============================================================================
TEST(UrdfParser, transmissionsCreateCoupledMimicJoints)
{
  // clang-format off
  const std::string urdfStr = R"(
    <robot name="transmission_robot">
      <link name="base"/>
      <joint name="j1" type="continuous">
        <parent link="base"/>
        <child link="l1"/>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <axis xyz="0 0 1"/>
      </joint>
      <link name="l1"/>
      <joint name="j2" type="continuous">
        <parent link="l1"/>
        <child link="l2"/>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <axis xyz="0 0 1"/>
      </joint>
      <link name="l2"/>

      <transmission name="tx1">
        <type>transmission_interface/SimpleTransmission</type>
        <actuator name="motor">
          <mechanicalReduction>2.0</mechanicalReduction>
        </actuator>
        <joint name="j1">
          <hardwareInterface>EffortJointInterface</hardwareInterface>
        </joint>
      </transmission>
      <transmission name="tx2">
        <type>transmission_interface/SimpleTransmission</type>
        <actuator name="motor">
          <mechanicalReduction>4.0</mechanicalReduction>
        </actuator>
        <joint name="j2">
          <hardwareInterface>EffortJointInterface</hardwareInterface>
        </joint>
      </transmission>
    </robot>
  )";
  // clang-format on

  UrdfParser loader;
  auto robot = loader.parseSkeletonString(urdfStr, "");
  ASSERT_TRUE(robot);

  auto* refJoint = robot->getJoint("j1");
  auto* follower = robot->getJoint("j2");
  ASSERT_NE(refJoint, nullptr);
  ASSERT_NE(follower, nullptr);

  EXPECT_NE(refJoint->getActuatorType(), dart::dynamics::Joint::MIMIC);
  EXPECT_EQ(follower->getActuatorType(), dart::dynamics::Joint::MIMIC);
  EXPECT_EQ(follower->getMimicJoint(), refJoint);
  EXPECT_TRUE(follower->isUsingCouplerConstraint());
  EXPECT_DOUBLE_EQ(follower->getMimicMultiplier(), 0.5);
  EXPECT_DOUBLE_EQ(follower->getMimicOffset(), 0.0);
}

//==============================================================================
TEST(UrdfParser, transmissionDynamicsRespectsGearRatio)
{
  // clang-format off
  const std::string urdfStr = R"(
    <robot name="transmission_robot_dynamic">
      <link name="base">
        <inertial>
          <mass value="1.0"/>
          <origin xyz="0 0 0" rpy="0 0 0"/>
          <inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/>
        </inertial>
      </link>
      <joint name="j1" type="continuous">
        <parent link="base"/>
        <child link="l1"/>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <axis xyz="0 0 1"/>
      </joint>
      <link name="l1">
        <inertial>
          <mass value="1.0"/>
          <origin xyz="0 0 0" rpy="0 0 0"/>
          <inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/>
        </inertial>
      </link>
      <joint name="j2" type="continuous">
        <parent link="l1"/>
        <child link="l2"/>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <axis xyz="0 0 1"/>
      </joint>
      <link name="l2">
        <inertial>
          <mass value="1.0"/>
          <origin xyz="0 0 0" rpy="0 0 0"/>
          <inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/>
        </inertial>
      </link>

      <transmission name="tx1">
        <type>transmission_interface/SimpleTransmission</type>
        <actuator name="motor">
          <mechanicalReduction>2.0</mechanicalReduction>
        </actuator>
        <joint name="j1">
          <hardwareInterface>EffortJointInterface</hardwareInterface>
        </joint>
      </transmission>
      <transmission name="tx2">
        <type>transmission_interface/SimpleTransmission</type>
        <actuator name="motor">
          <mechanicalReduction>4.0</mechanicalReduction>
        </actuator>
        <joint name="j2">
          <hardwareInterface>EffortJointInterface</hardwareInterface>
        </joint>
      </transmission>
    </robot>
  )";
  // clang-format on

  dart::utils::UrdfParser::Options options;
  options.mDefaultRootJointType = dart::utils::UrdfParser::RootJointType::Fixed;
  UrdfParser loader(options);
  auto robot = loader.parseSkeletonString(urdfStr, "");
  ASSERT_TRUE(robot);

  auto world = dart::simulation::World::create();
  world->setGravity(Eigen::Vector3d::Zero());
  world->setTimeStep(0.001);
  world->addSkeleton(robot);

  auto* j1 = robot->getJoint("j1");
  auto* j2 = robot->getJoint("j2");
  ASSERT_NE(j1, nullptr);
  ASSERT_NE(j2, nullptr);

  ASSERT_TRUE(j2->isUsingCouplerConstraint());
  ASSERT_EQ(j2->getActuatorType(), dart::dynamics::Joint::MIMIC);
  const double multiplier = j2->getMimicMultiplier();
  ASSERT_DOUBLE_EQ(multiplier, 0.5);

  const double tol = 5e-3;
  const double torque = 1.0;
  for (int i = 0; i < 200; ++i) {
    j1->setForce(0, torque);
    j2->setForce(0, 0.0);
    world->step();

    EXPECT_NEAR(j2->getPosition(0), multiplier * j1->getPosition(0), tol);
    EXPECT_NEAR(j2->getVelocity(0), multiplier * j1->getVelocity(0), tol);
  }
}

//==============================================================================
TEST(UrdfParser, transmissionsSkipInvalidEntries)
{
  // clang-format off
  const std::string urdfStr = R"(
    <robot name="transmission_robot_invalid">
      <link name="base"/>
      <joint name="j1" type="continuous">
        <parent link="base"/>
        <child link="l1"/>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <axis xyz="0 0 1"/>
      </joint>
      <link name="l1"/>
      <joint name="j2" type="continuous">
        <parent link="l1"/>
        <child link="l2"/>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <axis xyz="0 0 1"/>
      </joint>
      <link name="l2"/>

      <!-- Unsupported type should be ignored -->
      <transmission name="tx_bad_type">
        <type>transmission_interface/FourBarLinkageTransmission</type>
        <actuator name="motor">
          <mechanicalReduction>2.0</mechanicalReduction>
        </actuator>
        <joint name="j1">
          <hardwareInterface>EffortJointInterface</hardwareInterface>
        </joint>
      </transmission>

      <!-- Zero ratio should be ignored -->
      <transmission name="tx_zero_ratio">
        <type>transmission_interface/SimpleTransmission</type>
        <actuator name="motor">
          <mechanicalReduction>0.0</mechanicalReduction>
        </actuator>
        <joint name="j2">
          <hardwareInterface>EffortJointInterface</hardwareInterface>
        </joint>
      </transmission>
    </robot>
  )";
  // clang-format on

  UrdfParser loader;
  auto robot = loader.parseSkeletonString(urdfStr, "");
  ASSERT_TRUE(robot);

  auto* j1 = robot->getJoint("j1");
  auto* j2 = robot->getJoint("j2");
  ASSERT_NE(j1, nullptr);
  ASSERT_NE(j2, nullptr);

  // Neither joint should have been converted to a mimic actuator because the
  // only transmissions were invalid.
  EXPECT_NE(j1->getActuatorType(), dart::dynamics::Joint::MIMIC);
  EXPECT_NE(j2->getActuatorType(), dart::dynamics::Joint::MIMIC);
}

//==============================================================================
TEST(UrdfParser, badMimicJoint)
{
  // clang-format off
  std::string urdfStr = R"(
    <robot name="testRobot">
      <link name="link_0">
        <visual>
          <origin rpy="0 0 0" xyz="0 0 -0.087" />
          <geometry>
            <box size="1 1 1" />
          </geometry>
        </visual>
        <visual>
          <origin rpy="0 0 0" xyz="0 0 0" />
          <geometry>
            <box size="1 1 1" />
          </geometry>
        </visual>
      </link>
      <joint name="0_to_1" type="revolute">
        <parent link="link_0" />
        <child link="link_1" />
        <limit effort="2.5" lower="-3.14159265359"
               upper="3.14159265359" velocity="3.00545697193" />
        <axis xyz="0 0 1" />
        <dynamics damping="1.2" friction="2.3" />
      </joint>
      <link name="link_1">
        <visual>
          <origin rpy="0 0 0" xyz="0 0 -0.087" />
          <geometry>
            <box size="1 1 1" />
          </geometry>
        </visual>
        <visual>
          <origin rpy="0 0 0" xyz="0 0 0" />
          <geometry>
            <box size="1 1 1" />
          </geometry>
        </visual>
      </link>
      <joint name="1_to_2" type="continuous">
        <parent link="link_1" />
        <child link="link_2" />
        <limit effort="2.5" velocity="3.00545697193" />
        <axis xyz="0 0 1" />
        <dynamics damping="1.2" friction="2.3" />
        <mimic joint="mjoint" multiplier="2." offset="0.1" />
      </joint>
      <link name="link_2">
        <visual>
          <origin rpy="0 0 0" xyz="0 0 -0.087" />
          <geometry>
            <box size="1 1 1" />
          </geometry>
        </visual>
        <visual>
          <origin rpy="0 0 0" xyz="0 0 0" />
          <geometry>
            <box size="1 1 1" />
          </geometry>
        </visual>
      </link>
    </robot>
  )";
  // clang-format on

  UrdfParser parser;
  auto robot = parser.parseSkeletonString(urdfStr, "");
  ASSERT_TRUE(nullptr != robot);

  auto joint1 = robot->getJoint(1);
  ASSERT_TRUE(nullptr != joint1);
  EXPECT_NEAR(joint1->getDampingCoefficient(0), 1.2, 1e-12);
  EXPECT_NEAR(joint1->getCoulombFriction(0), 2.3, 1e-12);

  auto joint2 = robot->getJoint(2);
  EXPECT_DOUBLE_EQ(joint2->getPositionLowerLimit(0), -dart::math::inf);
  EXPECT_DOUBLE_EQ(joint2->getPositionUpperLimit(0), dart::math::inf);
  EXPECT_TRUE(joint2->isCyclic(0));

  EXPECT_TRUE(joint2->getActuatorType() != dart::dynamics::Joint::MIMIC);
  EXPECT_TRUE(nullptr == joint2->getMimicJoint());
}

//==============================================================================
TEST(UrdfParser, WorldShouldBeTreatedAsKeyword)
{
  // clang-format off
  const std::string urdfStr = R"(
    <robot name="testRobot">
      <link name="world" />
      <joint name="world_to_1" type="revolute">
        <parent link="world" />
        <child link="link_0" />
        <axis xyz="0 0 1" />
        <limit effort="2.5" lower="-3.14159265359"
               upper="3.14159265359" velocity="3.00545697193" />
        <axis xyz="0 0 1" />
        <dynamics damping="1.2" friction="2.3" />
      </joint>
      <link name="link_0" />
    </robot>
  )";
  // clang-format on

  UrdfParser parser;
  auto robot = parser.parseSkeletonString(urdfStr, "");
  ASSERT_TRUE(nullptr != robot);

  EXPECT_TRUE(robot->getNumBodyNodes() == 1u);
  EXPECT_TRUE(robot->getRootBodyNode()->getName() == "link_0");
}

//==============================================================================
TEST(UrdfParser, SingleLinkWithoutJoint)
{
  // clang-format off
  const std::string urdfStr = R"(
    <robot name="testRobot">
      <link name="link_0" />
    </robot>
  )";
  // clang-format on

  UrdfParser parser;
  auto robot = parser.parseSkeletonString(urdfStr, "");
  ASSERT_TRUE(nullptr != robot);

  EXPECT_TRUE(robot->getNumBodyNodes() == 1u);
  EXPECT_TRUE(robot->getRootBodyNode()->getName() == "link_0");
  EXPECT_EQ(
      robot->getRootJoint()->getType(),
      dart::dynamics::FreeJoint::getStaticType());
}

//==============================================================================
TEST(UrdfParser, MultiTreeRobot)
{
  // clang-format off
  const std::string urdfStr = R"(
    <robot name="testRobot">
      <link name="world" />
      <joint name="world_to_0" type="revolute">
        <parent link="world" />
        <child link="link_0" />
        <axis xyz="0 0 1" />
        <limit effort="2.5" lower="-3.14159265359"
               upper="3.14159265359" velocity="3.00545697193" />
        <axis xyz="0 0 1" />
        <dynamics damping="1.2" friction="2.3" />
      </joint>
      <link name="link_0" />
      <joint name="world_to_1" type="revolute">
        <parent link="world" />
        <child link="link_1" />
        <axis xyz="0 0 1" />
        <limit effort="2.5" lower="-3.14159265359"
               upper="3.14159265359" velocity="3.00545697193" />
        <axis xyz="0 0 1" />
        <dynamics damping="1.2" friction="2.3" />
      </joint>
      <link name="link_1" />
    </robot>
  )";
  // clang-format on

  UrdfParser parser;
  auto robot = parser.parseSkeletonString(urdfStr, "");
  ASSERT_TRUE(nullptr != robot);

  EXPECT_EQ(robot->getNumBodyNodes(), 2u);
  EXPECT_EQ(robot->getNumTrees(), 2u);
}

//==============================================================================
TEST(UrdfParser, KR5MeshColor)
{
  UrdfParser parser;
  auto robot
      = parser.parseSkeleton("dart://sample/urdf/KR5/KR5 sixx R650.urdf");
  ASSERT_TRUE(nullptr != robot);

  EXPECT_EQ(robot->getNumBodyNodes(), 7u);
  EXPECT_EQ(robot->getNumTrees(), 1u);

  robot->eachBodyNode([](dynamics::BodyNode* bodyNode) {
    bodyNode->eachShapeNodeWith<dynamics::VisualAspect>(
        [&](dynamics::ShapeNode* shapeNode) {
          auto shape = shapeNode->getShape();
          if (auto mesh = shape->as<dynamics::MeshShape>()) {
            EXPECT_EQ(mesh->getColorMode(), dynamics::MeshShape::SHAPE_COLOR);
          }
        });
  });
}

//==============================================================================
TEST(UrdfParser, parseVisualCollisionName)
{
  // clang-format off
  std::string urdfStr = R"(
    <robot name="testRobot">
      <link name="link_0">
        <visual name="first_visual">
          <origin rpy="0 0 0" xyz="0 0 -0.087" />
          <geometry>
            <box size="1 1 1" />
          </geometry>
        </visual>
        <visual name="second_visual">
          <origin rpy="0 0 0" xyz="0 0 0" />
          <geometry>
            <box size="1 1 1" />
          </geometry>
        </visual>
      </link>
      <joint name="0_to_1" type="revolute">
        <parent link="link_0" />
        <child link="link_1" />
        <limit effort="2.5" lower="-3.14159265359"
               upper="3.14159265359" velocity="3.00545697193" />
        <axis xyz="0 0 1" />
        <dynamics damping="1.2" friction="2.3" />
      </joint>
      <link name="link_1">
        <visual>
          <origin rpy="0 0 0" xyz="0 0 -0.087" />
          <geometry>
            <box size="1 1 1" />
          </geometry>
        </visual>
        <visual>
          <origin rpy="0 0 0" xyz="0 0 0" />
          <geometry>
            <box size="1 1 1" />
          </geometry>
        </visual>
      </link>
    </robot>
  )";
  // clang-format on

  UrdfParser parser;
  auto robot = parser.parseSkeletonString(urdfStr, "");
  ASSERT_TRUE(nullptr != robot);

  auto body0 = robot->getBodyNode(0);
  ASSERT_TRUE(nullptr != body0);
  ASSERT_EQ(body0->getNumShapeNodes(), 2);
  EXPECT_EQ(body0->getShapeNode(0)->getName(), "first_visual");
  EXPECT_EQ(body0->getShapeNode(1)->getName(), "second_visual");

  auto body1 = robot->getBodyNode(1);
  ASSERT_TRUE(nullptr != body1);
  ASSERT_EQ(body1->getNumShapeNodes(), 2);
  // The ShapeNode naming pattern is inferred from
  // BodyNode::createShapeNodeWith(const ShapePtr& shape) sot this test could
  // fail if the naming pattern in the function is changed.
  EXPECT_EQ(
      body1->getShapeNode(0)->getName(), body1->getName() + "_ShapeNode_0");
  EXPECT_EQ(
      body1->getShapeNode(1)->getName(), body1->getName() + "_ShapeNode_1");
}

//==============================================================================
TEST(UrdfParser, Options)
{
  // clang-format off
  std::string urdfStr = R"(
    <robot name="testRobot">
      <link name="link_0">
      </link>
    </robot>
  )";
  // clang-format on

  UrdfParser parser;
  UrdfParser::Options options;

  // Default inertia
  auto robot = parser.parseSkeletonString(urdfStr, "");
  ASSERT_TRUE(nullptr != robot);
  auto link_0 = robot->getBodyNode("link_0");
  ASSERT_TRUE(link_0 != nullptr);
  EXPECT_TRUE(equals(
      link_0->getInertia().getSpatialTensor(),
      dynamics::Inertia().getSpatialTensor()));

  // Custom inertia
  options.mDefaultInertia.setMass(5);
  options.mDefaultInertia.setMoment(1, 2, 3, 4, 5, 6);
  parser.setOptions(options);
  robot = parser.parseSkeletonString(urdfStr, "");
  ASSERT_TRUE(nullptr != robot);
  link_0 = robot->getBodyNode("link_0");
  ASSERT_TRUE(link_0 != nullptr);
  EXPECT_TRUE(equals(
      link_0->getInertia().getSpatialTensor(),
      options.mDefaultInertia.getSpatialTensor()));
}

#if DART_IO_HAS_URDF
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
TEST(UrdfParser, WamMeshMaterialAndLimits)
{
  io::ReadOptions options;
  options.format = io::ModelFormat::Urdf;
  options.addPackageDirectory("herb_description", config::dataPath("urdf/wam"));

  const auto skeleton
      = io::readSkeleton("dart://sample/urdf/wam/wam.urdf", options);
  ASSERT_NE(skeleton, nullptr);
  EXPECT_TRUE(hasShapeType<dynamics::MeshShape>(skeleton));

  const auto joint = skeleton->getJoint("/j1");
  ASSERT_NE(joint, nullptr);
  const auto* revolute = dynamic_cast<dynamics::RevoluteJoint*>(joint);
  ASSERT_NE(revolute, nullptr);
  EXPECT_NEAR(revolute->getPositionLowerLimit(0), -2.6, 1e-6);
  EXPECT_NEAR(revolute->getPositionUpperLimit(0), 2.6, 1e-6);

  const Eigen::Vector4d expectedColor(
      0.792156862745098, 0.819607843137255, 0.933333333333333, 1.0);
  bool foundVisual = false;
  for (std::size_t i = 0; i < skeleton->getNumBodyNodes(); ++i) {
    auto* body = skeleton->getBodyNode(i);
    if (!body) {
      continue;
    }
    body->eachShapeNodeWith<dynamics::VisualAspect>(
        [&](const dynamics::ShapeNode* shapeNode) {
          if (shapeNode->getVisualAspect()->getRGBA().isApprox(expectedColor)) {
            foundVisual = true;
          }
        });
  }
  EXPECT_TRUE(foundVisual);
}

//==============================================================================
TEST(UrdfParser, DrchuboStructure)
{
  io::ReadOptions options;
  options.format = io::ModelFormat::Urdf;
  options.addPackageDirectory("drchubo", config::dataPath("urdf/drchubo"));

  const auto skeleton
      = io::readSkeleton("dart://sample/urdf/drchubo/drchubo.urdf", options);
  ASSERT_NE(skeleton, nullptr);
  EXPECT_GT(skeleton->getNumBodyNodes(), 20u);
  EXPECT_GT(skeleton->getNumDofs(), 20u);
  EXPECT_TRUE(hasShapeType<dynamics::MeshShape>(skeleton));
}
#endif

//==============================================================================
TEST(UrdfParser, ParseSkeletonStringEmptyReturnsNull)
{
  UrdfParser parser;
  EXPECT_EQ(parser.parseSkeletonString("", ""), nullptr);
}

//==============================================================================
TEST(UrdfParser, ParseWorldStringEmptyReturnsNull)
{
  UrdfParser parser;
  EXPECT_EQ(parser.parseWorldString("", ""), nullptr);
}

//==============================================================================
TEST(UrdfParser, ParseWorldStringMissingWorldElementReturnsNull)
{
  UrdfParser parser;
  const auto tempDir = makeTempDir("dart-urdf-world-missing-world");
  const Uri baseUri = Uri::createFromPath((tempDir / "world.urdf").string());
  const std::string worldXml = R"(<robot name="not_a_world"/>)";
  EXPECT_EQ(parser.parseWorldString(worldXml, baseUri), nullptr);
  std::filesystem::remove_all(tempDir);
}

//==============================================================================
TEST(UrdfParser, ParseWorldStringMissingNameReturnsNull)
{
  UrdfParser parser;
  const auto tempDir = makeTempDir("dart-urdf-world-missing-name");
  const Uri baseUri = Uri::createFromPath((tempDir / "world.urdf").string());
  const std::string worldXml = R"(<world></world>)";
  EXPECT_EQ(parser.parseWorldString(worldXml, baseUri), nullptr);
  std::filesystem::remove_all(tempDir);
}

//==============================================================================
TEST(UrdfParser, ParseWorldStringInvalidXmlReturnsNull)
{
  UrdfParser parser;
  const auto tempDir = makeTempDir("dart-urdf-world-invalid-xml");
  const Uri baseUri = Uri::createFromPath((tempDir / "world.urdf").string());
  const std::string worldXml = R"(<world name="world">)";
  EXPECT_EQ(parser.parseWorldString(worldXml, baseUri), nullptr);
  std::filesystem::remove_all(tempDir);
}

//==============================================================================
TEST(UrdfParser, MemoryResourceRetrieverMaterialMeshAndPackageUri)
{
  const std::string meshData
      = "solid unit\n"
        "facet normal 0 0 1\n"
        "  outer loop\n"
        "    vertex 0 0 0\n"
        "    vertex 1 0 0\n"
        "    vertex 0 1 0\n"
        "  endloop\n"
        "endfacet\n"
        "endsolid unit\n";

  const std::string urdfStr = R"(
    <robot name="mem_mesh">
      <material name="global_color">
        <color rgba="0.2 0.3 0.4 0.5" />
      </material>
      <link name="base">
        <visual name="mesh_visual">
          <geometry>
            <mesh filename="package://test_pkg/mesh.stl" />
          </geometry>
          <material name="global_color" />
        </visual>
      </link>
    </robot>
  )";

  auto retriever = std::make_shared<MemoryResourceRetriever>();
  const auto uri = addMemoryFile(*retriever, "memory://mem_mesh.urdf", urdfStr);
  retriever->add("package://test_pkg/mesh.stl", meshData);

  UrdfParser::Options options(retriever);
  UrdfParser parser(options);
  const auto robot = parser.parseSkeleton(uri);
  ASSERT_TRUE(robot);

  auto* body = robot->getBodyNode("base");
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
        EXPECT_GT(mesh->getScale()[0], 0.0);
      });
  EXPECT_TRUE(foundMesh);
}

//==============================================================================
TEST(UrdfParser, AddPackageDirectoryResolvesMesh)
{
  const auto tempDir = makeTempDir("dart-urdf-package");
  const auto meshPath = tempDir / "mesh.stl";
  const auto urdfPath = tempDir / "model.urdf";

  std::ofstream meshFile(meshPath.string(), std::ios::binary);
  ASSERT_TRUE(meshFile.is_open());
  meshFile << "solid unit\n"
              "facet normal 0 0 1\n"
              "  outer loop\n"
              "    vertex 0 0 0\n"
              "    vertex 1 0 0\n"
              "    vertex 0 1 0\n"
              "  endloop\n"
              "endfacet\n"
              "endsolid unit\n";
  meshFile.close();

  const std::string urdfStr = R"(
    <robot name="pkg_mesh">
      <link name="base">
        <visual name="mesh_visual">
          <geometry>
            <mesh filename="package://test_pkg/mesh.stl" />
          </geometry>
          <material name="mesh_color">
            <color rgba="0.1 0.2 0.3 0.4" />
          </material>
        </visual>
      </link>
    </robot>
  )";

  std::ofstream urdfFile(urdfPath.string(), std::ios::binary);
  ASSERT_TRUE(urdfFile.is_open());
  urdfFile << urdfStr;
  urdfFile.close();

  UrdfParser parser;
  parser.addPackageDirectory("test_pkg", tempDir.string());
  const auto robot
      = parser.parseSkeleton(Uri::createFromPath(urdfPath.string()));
  ASSERT_TRUE(robot);

  auto* body = robot->getBodyNode("base");
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
        EXPECT_EQ(
            mesh->getColorMode(), dynamics::MeshShape::ColorMode::SHAPE_COLOR);
      });
  EXPECT_TRUE(foundMesh);

  std::filesystem::remove_all(tempDir);
}

//==============================================================================
TEST(UrdfParser, MemoryResourceRetrieverJointLimitsAndTransmissions)
{
  const std::string urdfStr = R"(
    <robot name="mem_limits">
      <link name="base" />
      <link name="link1" />
      <link name="link2" />
      <joint name="joint1" type="revolute">
        <parent link="base" />
        <child link="link1" />
        <axis xyz="0 0 1" />
        <limit lower="1.0" upper="3.0" effort="5" velocity="6" />
      </joint>
      <joint name="joint2" type="revolute">
        <parent link="link1" />
        <child link="link2" />
        <axis xyz="0 1 0" />
        <limit lower="-2.0" upper="-1.0" effort="2" velocity="3" />
      </joint>
      <transmission name="tx1">
        <type>transmission_interface/SimpleTransmission</type>
        <joint name="joint1" />
        <actuator name="actuator">
          <mechanicalReduction>2</mechanicalReduction>
        </actuator>
      </transmission>
      <transmission name="tx2">
        <type>transmission_interface/SimpleTransmission</type>
        <joint name="joint2" />
        <actuator name="actuator">
          <mechanicalReduction>4</mechanicalReduction>
        </actuator>
      </transmission>
    </robot>
  )";

  auto retriever = std::make_shared<MemoryResourceRetriever>();
  const auto uri
      = addMemoryFile(*retriever, "memory://mem_limits.urdf", urdfStr);

  UrdfParser::Options options(retriever);
  UrdfParser parser(options);
  const auto robot = parser.parseSkeleton(uri);
  ASSERT_TRUE(robot);

  auto* joint1 = robot->getJoint("joint1");
  auto* joint2 = robot->getJoint("joint2");
  ASSERT_NE(joint1, nullptr);
  ASSERT_NE(joint2, nullptr);

  EXPECT_DOUBLE_EQ(joint1->getPositionLowerLimit(0), 1.0);
  EXPECT_DOUBLE_EQ(joint1->getPositionUpperLimit(0), 3.0);
  EXPECT_DOUBLE_EQ(joint1->getRestPosition(0), 2.0);
  EXPECT_NEAR(joint1->getPosition(0), 2.0, 1e-12);

  EXPECT_EQ(joint2->getActuatorType(), dart::dynamics::Joint::MIMIC);
}

//==============================================================================
TEST(UrdfParser, ParsePlanarJointAxisXMapsToYZ)
{
  const std::string urdfStr = R"(
    <robot name="planar_example">
      <link name="base"/>
      <link name="tip"/>
      <joint name="planar_joint" type="planar">
        <parent link="base"/>
        <child link="tip"/>
        <axis xyz="1 0 0"/>
      </joint>
    </robot>
  )";

  UrdfParser parser;
  auto robot = parser.parseSkeletonString(urdfStr, "");
  ASSERT_TRUE(robot);

  auto* joint
      = dynamic_cast<dynamics::PlanarJoint*>(robot->getJoint("planar_joint"));
  ASSERT_TRUE(joint);
  EXPECT_EQ(joint->getPlaneType(), dynamics::PlanarJoint::PlaneType::YZ);
  EXPECT_TRUE(joint->getRotationalAxis().isApprox(Eigen::Vector3d::UnitX()));
}

//==============================================================================
TEST(UrdfParser, ParseSkeletonStringPrismaticJointType)
{
  const std::string urdfStr = R"(
    <robot name="prismatic_example">
      <link name="base"/>
      <link name="slider"/>
      <joint name="slide_joint" type="prismatic">
        <parent link="base"/>
        <child link="slider"/>
        <axis xyz="0 1 0"/>
        <limit lower="-0.5" upper="0.5" effort="1.0" velocity="2.0"/>
      </joint>
    </robot>
  )";

  UrdfParser parser;
  auto robot = parser.parseSkeletonString(urdfStr, "");
  ASSERT_TRUE(robot);

  auto* joint = robot->getJoint("slide_joint");
  ASSERT_NE(joint, nullptr);
  EXPECT_EQ(joint->getType(), "PrismaticJoint");
}

//==============================================================================
TEST(UrdfParser, ParseSkeletonStringFixedJointType)
{
  const std::string urdfStr = R"(
    <robot name="fixed_example">
      <link name="base"/>
      <link name="fixed_link"/>
      <joint name="fixed_joint" type="fixed">
        <parent link="base"/>
        <child link="fixed_link"/>
      </joint>
    </robot>
  )";

  UrdfParser parser;
  auto robot = parser.parseSkeletonString(urdfStr, "");
  ASSERT_TRUE(robot);

  auto* joint = robot->getJoint("fixed_joint");
  ASSERT_NE(joint, nullptr);
  EXPECT_EQ(joint->getType(), "WeldJoint");
}

//==============================================================================
TEST(UrdfParser, ParseWorldStringSetsFixedRootTransform)
{
  UrdfParser::Options options;
  options.mDefaultRootJointType = UrdfParser::RootJointType::Fixed;
  UrdfParser parser(options);

  const auto tempDir = makeTempDir("dart-urdf-world-fixed");
  const auto modelPath = tempDir / "model.urdf";

  std::ofstream modelFile(modelPath);
  ASSERT_TRUE(modelFile.is_open());
  modelFile << "<robot name=\"model\"><link name=\"base\"/></robot>";
  modelFile.close();

  const std::string worldXml = R"(
<world name="world">
  <include filename="model.urdf" model_name="model1"/>
  <entity name="entity1" model="model1">
    <origin xyz="1 2 3" rpy="0 0 1.57079632679"/>
  </entity>
</world>
)";

  const Uri baseUri = Uri::createFromPath((tempDir / "world.urdf").string());
  auto world = parser.parseWorldString(worldXml, baseUri);
  ASSERT_TRUE(world != nullptr);

  const auto skeleton = world->getSkeleton("entity1");
  ASSERT_TRUE(skeleton != nullptr);
  auto* joint = skeleton->getRootJoint();
  ASSERT_NE(joint, nullptr);
  EXPECT_EQ(joint->getType(), "WeldJoint");
  EXPECT_TRUE(joint->getTransformFromParentBodyNode().translation().isApprox(
      Eigen::Vector3d(1.0, 2.0, 3.0)));

  std::filesystem::remove_all(tempDir);
}

//==============================================================================
TEST(UrdfParser, ParseSkeletonStringInvalidXmlReturnsNull)
{
  UrdfParser parser;
  const std::string urdfStr = "<robot name=\"bad\"><link name=\"a\"></robot";
  EXPECT_EQ(parser.parseSkeletonString(urdfStr, ""), nullptr);
}

//==============================================================================
TEST(UrdfParser, ParseSkeletonStringMissingChildLinkReturnsNull)
{
  UrdfParser parser;
  const std::string urdfStr = R"(
    <robot name="missing_child">
      <link name="base" />
      <joint name="base_to_missing" type="revolute">
        <parent link="base" />
        <child link="missing" />
        <axis xyz="0 0 1" />
        <limit effort="1.0" lower="-1.0" upper="1.0" velocity="1.0" />
      </joint>
    </robot>
  )";
  EXPECT_EQ(parser.parseSkeletonString(urdfStr, ""), nullptr);
}

//==============================================================================
TEST(UrdfParser, ParseSkeletonStringJointTypes)
{
  const std::string urdfStr = R"(
    <robot name="joint_types">
      <link name="base" />
      <link name="slide" />
      <link name="plane" />
      <link name="fixed_link" />
      <link name="floating_link" />
      <joint name="prismatic_joint" type="prismatic">
        <parent link="base" />
        <child link="slide" />
        <axis xyz="1 0 0" />
        <limit effort="1" lower="-1" upper="1" velocity="2" />
      </joint>
      <joint name="planar_joint" type="planar">
        <parent link="slide" />
        <child link="plane" />
        <axis xyz="0 0 1" />
      </joint>
      <joint name="fixed_joint" type="fixed">
        <parent link="plane" />
        <child link="fixed_link" />
      </joint>
      <joint name="floating_joint" type="floating">
        <parent link="fixed_link" />
        <child link="floating_link" />
      </joint>
    </robot>
  )";

  UrdfParser parser;
  const auto robot = parser.parseSkeletonString(urdfStr, "");
  ASSERT_TRUE(robot);
  EXPECT_EQ(robot->getJoint("prismatic_joint")->getType(), "PrismaticJoint");
  EXPECT_EQ(robot->getJoint("planar_joint")->getType(), "PlanarJoint");
  EXPECT_EQ(robot->getJoint("fixed_joint")->getType(), "WeldJoint");
  EXPECT_EQ(robot->getJoint("floating_joint")->getType(), "FreeJoint");
}

//==============================================================================
TEST(UrdfParser, MaterialAndGeometryParsing)
{
  const std::string urdfStr = R"(
    <robot name="materials">
      <link name="link">
        <visual>
          <geometry>
            <box size="0.1 0.2 0.3" />
          </geometry>
          <material name="mat">
            <color rgba="0.1 0.2 0.3 0.4" />
          </material>
        </visual>
        <collision>
          <geometry>
            <cylinder radius="0.2" length="0.4" />
          </geometry>
        </collision>
      </link>
    </robot>
  )";

  UrdfParser parser;
  const auto robot = parser.parseSkeletonString(urdfStr, "");
  ASSERT_TRUE(robot);
  auto* body = robot->getBodyNode("link");
  ASSERT_NE(body, nullptr);

  EXPECT_GT(body->getNumShapeNodes(), 0u);

  bool foundCollision = false;
  body->eachShapeNodeWith<dynamics::CollisionAspect>(
      [&](dynamics::ShapeNode* shapeNode) {
        foundCollision = true;
        EXPECT_EQ(shapeNode->getShape()->getType(), "CylinderShape");
      });

  EXPECT_TRUE(foundCollision);
}

//==============================================================================
TEST(UrdfParser, MeshScaleFromString)
{
  const auto tempDir = makeTempDir("dart-urdf-mesh-scale");
  const auto meshPath = tempDir / "mesh.obj";
  std::ofstream meshFile(meshPath);
  ASSERT_TRUE(meshFile.is_open());
  meshFile << "o Mesh\n"
           << "v 0 0 0\n"
           << "v 1 0 0\n"
           << "v 0 1 0\n"
           << "f 1 2 3\n";
  meshFile.close();

  const std::string urdfStr = R"(
    <robot name="mesh_scale">
      <link name="link">
        <visual>
          <geometry>
            <mesh filename="mesh.obj" scale="1 2 3" />
          </geometry>
        </visual>
      </link>
    </robot>
  )";

  UrdfParser parser;
  const Uri baseUri = Uri::createFromPath((tempDir / "model.urdf").string());
  const auto robot = parser.parseSkeletonString(urdfStr, baseUri);
  ASSERT_TRUE(robot);
  auto* body = robot->getBodyNode("link");
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

  std::filesystem::remove_all(tempDir);
}

//==============================================================================
TEST(UrdfParser, ParseSkeletonStringMissingRobotRootReturnsNull)
{
  UrdfParser parser;
  const std::string urdfStr = R"(<not_robot />)";
  EXPECT_EQ(parser.parseSkeletonString(urdfStr, ""), nullptr);
}

//==============================================================================
TEST(UrdfParser, CollisionGeometryTypesParsing)
{
  const std::string urdfStr = R"(
    <robot name="collision_geom">
      <link name="base">
        <collision>
          <geometry>
            <box size="0.1 0.2 0.3" />
          </geometry>
        </collision>
        <collision>
          <geometry>
            <sphere radius="0.2" />
          </geometry>
        </collision>
        <collision>
          <geometry>
            <cylinder radius="0.1" length="0.4" />
          </geometry>
        </collision>
      </link>
    </robot>
  )";

  UrdfParser parser;
  const auto robot = parser.parseSkeletonString(urdfStr, "");
  ASSERT_TRUE(robot);
  auto* body = robot->getBodyNode("base");
  ASSERT_NE(body, nullptr);

  std::size_t boxCount = 0u;
  std::size_t sphereCount = 0u;
  std::size_t cylinderCount = 0u;
  body->eachShapeNodeWith<dynamics::CollisionAspect>(
      [&](dynamics::ShapeNode* node) {
        const auto type = node->getShape()->getType();
        if (type == "BoxShape") {
          ++boxCount;
        } else if (type == "SphereShape") {
          ++sphereCount;
        } else if (type == "CylinderShape") {
          ++cylinderCount;
        }
      });
  EXPECT_EQ(boxCount, 1u);
  EXPECT_EQ(sphereCount, 1u);
  EXPECT_EQ(cylinderCount, 1u);
}

//==============================================================================
TEST(UrdfParser, ParsesJointTypesGeometryAndInertia)
{
  const auto tempDir = makeTempDir("dart-urdf-variants");
  const auto meshPath = tempDir / "mesh.obj";
  std::ofstream meshFile(meshPath);
  ASSERT_TRUE(meshFile.is_open());
  meshFile << "o Mesh\n"
           << "v 0 0 0\n"
           << "v 1 0 0\n"
           << "v 0 1 0\n"
           << "f 1 2 3\n";
  meshFile.close();

  const std::string urdfStr = R"(
    <robot name="variants">
      <link name="base">
        <inertial>
          <mass value="3.0"/>
          <origin xyz="0 0 0" rpy="0 0 0"/>
          <inertia ixx="0.3" iyy="0.3" izz="0.3" ixy="0" ixz="0" iyz="0"/>
        </inertial>
        <visual>
          <geometry>
            <cylinder radius="0.2" length="0.5"/>
          </geometry>
        </visual>
        <visual>
          <geometry>
            <mesh filename="mesh.obj" scale="1 2 3"/>
          </geometry>
        </visual>
        <collision>
          <geometry>
            <sphere radius="0.1"/>
          </geometry>
        </collision>
      </link>
      <link name="link1"/>
      <joint name="continuous_joint" type="continuous">
        <parent link="base"/>
        <child link="link1"/>
        <axis xyz="0 0 1"/>
      </joint>
      <link name="link2"/>
      <joint name="fixed_joint" type="fixed">
        <parent link="link1"/>
        <child link="link2"/>
      </joint>
      <link name="link3"/>
      <joint name="floating_joint" type="floating">
        <parent link="link2"/>
        <child link="link3"/>
      </joint>
    </robot>
  )";

  UrdfParser parser;
  const Uri baseUri = Uri::createFromPath((tempDir / "model.urdf").string());
  const auto robot = parser.parseSkeletonString(urdfStr, baseUri);
  ASSERT_TRUE(robot);

  auto* continuous = robot->getJoint("continuous_joint");
  auto* fixed = robot->getJoint("fixed_joint");
  auto* floating = robot->getJoint("floating_joint");
  ASSERT_NE(continuous, nullptr);
  ASSERT_NE(fixed, nullptr);
  ASSERT_NE(floating, nullptr);
  EXPECT_EQ(continuous->getType(), "RevoluteJoint");
  EXPECT_EQ(fixed->getType(), "WeldJoint");
  EXPECT_EQ(floating->getType(), "FreeJoint");

  auto* base = robot->getBodyNode("base");
  ASSERT_NE(base, nullptr);
  EXPECT_DOUBLE_EQ(base->getMass(), 3.0);

  bool foundCylinder = false;
  bool foundSphere = false;
  bool foundMesh = false;
  base->eachShapeNode([&](const dynamics::ShapeNode* node) {
    const auto shape = node->getShape();
    if (!shape) {
      return;
    }
    if (shape->is<dynamics::CylinderShape>()) {
      foundCylinder = true;
    } else if (shape->is<dynamics::SphereShape>()) {
      foundSphere = true;
    } else if (shape->is<dynamics::MeshShape>()) {
      const auto mesh
          = std::dynamic_pointer_cast<const dynamics::MeshShape>(shape);
      ASSERT_TRUE(mesh != nullptr);
      EXPECT_TRUE(mesh->getScale().isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));
      foundMesh = true;
    }
  });

  EXPECT_TRUE(foundCylinder);
  EXPECT_TRUE(foundSphere);
  EXPECT_TRUE(foundMesh);

  std::filesystem::remove_all(tempDir);
}

//==============================================================================
TEST(UrdfParser, OptionsGetterAndPackageResolution)
{
  UrdfParser parser;
  UrdfParser::Options options;
  options.mDefaultRootJointType = UrdfParser::RootJointType::Fixed;
  parser.setOptions(options);
  EXPECT_EQ(
      parser.getOptions().mDefaultRootJointType,
      UrdfParser::RootJointType::Fixed);

  const auto tempDir = makeTempDir("dart-urdf-pkg");
  const auto meshDir = tempDir / "meshes";
  std::filesystem::create_directories(meshDir);
  const auto meshPath = meshDir / "mesh.obj";
  std::ofstream meshFile(meshPath);
  ASSERT_TRUE(meshFile.is_open());
  meshFile << "o Mesh\n"
           << "v 0 0 0\n"
           << "v 1 0 0\n"
           << "v 0 1 0\n"
           << "f 1 2 3\n";
  meshFile.close();

  parser.addPackageDirectory("test_pkg", tempDir.string());
  const std::string urdfStr = R"(
    <robot name="pkg_robot">
      <link name="base">
        <visual>
          <geometry>
            <mesh filename="package://test_pkg/meshes/mesh.obj" />
          </geometry>
        </visual>
      </link>
    </robot>
  )";

  const Uri baseUri = Uri::createFromPath((tempDir / "model.urdf").string());
  const auto robot = parser.parseSkeletonString(urdfStr, baseUri);
  ASSERT_TRUE(robot);
  auto* body = robot->getBodyNode("base");
  ASSERT_NE(body, nullptr);

  bool foundMesh = false;
  body->eachShapeNodeWith<dynamics::VisualAspect>(
      [&](dynamics::ShapeNode* node) {
        if (std::dynamic_pointer_cast<dynamics::MeshShape>(node->getShape())) {
          foundMesh = true;
        }
      });
  EXPECT_TRUE(foundMesh);

  std::filesystem::remove_all(tempDir);
}

//==============================================================================
TEST(UrdfParser, TransmissionsSkipMissingElements)
{
  const std::string urdfStr = R"(
    <robot name="transmission_missing_elements">
      <link name="base"/>
      <joint name="j1" type="continuous">
        <parent link="base"/>
        <child link="l1"/>
        <axis xyz="0 0 1"/>
      </joint>
      <link name="l1"/>
      <transmission name="tx_missing_actuator">
        <type>transmission_interface/SimpleTransmission</type>
        <joint name="j1" />
      </transmission>
      <transmission name="tx_missing_joint">
        <type>transmission_interface/SimpleTransmission</type>
        <actuator name="motor">
          <mechanicalReduction>2.0</mechanicalReduction>
        </actuator>
      </transmission>
    </robot>
  )";

  UrdfParser parser;
  const auto robot = parser.parseSkeletonString(urdfStr, "");
  ASSERT_TRUE(robot);
  auto* joint = robot->getJoint("j1");
  ASSERT_NE(joint, nullptr);
  EXPECT_NE(joint->getActuatorType(), dart::dynamics::Joint::MIMIC);
}

//==============================================================================
TEST(UrdfParser, TransmissionsSkipInvalidReductionAndMultiDof)
{
  const std::string urdfStr = R"(
    <robot name="transmission_variants">
      <link name="base"/>
      <link name="link1"/>
      <joint name="j1" type="continuous">
        <parent link="base"/>
        <child link="link1"/>
        <axis xyz="0 0 1"/>
      </joint>
      <link name="link2"/>
      <joint name="j2" type="floating">
        <parent link="link1"/>
        <child link="link2"/>
      </joint>

      <transmission name="tx_bad_ratio">
        <type>transmission_interface/SimpleTransmission</type>
        <actuator name="motor">
          <mechanicalReduction>bad</mechanicalReduction>
        </actuator>
        <joint name="j1">
          <hardwareInterface>EffortJointInterface</hardwareInterface>
        </joint>
      </transmission>
      <transmission name="tx_multi_dof">
        <type>transmission_interface/SimpleTransmission</type>
        <actuator name="motor">
          <mechanicalReduction>2.0</mechanicalReduction>
        </actuator>
        <joint name="j2">
          <hardwareInterface>EffortJointInterface</hardwareInterface>
        </joint>
      </transmission>
    </robot>
  )";

  UrdfParser parser;
  const auto robot = parser.parseSkeletonString(urdfStr, "");
  ASSERT_TRUE(robot);

  auto* j1 = robot->getJoint("j1");
  auto* j2 = robot->getJoint("j2");
  ASSERT_NE(j1, nullptr);
  ASSERT_NE(j2, nullptr);

  EXPECT_NE(j1->getActuatorType(), dart::dynamics::Joint::MIMIC);
  EXPECT_NE(j2->getActuatorType(), dart::dynamics::Joint::MIMIC);
}

//==============================================================================
TEST(UrdfParser, PrismaticLimitsSetInitialAndRestPositions)
{
  const std::string urdfStr = R"(
    <robot name="prismatic_limits">
      <link name="base" />
      <link name="slider" />
      <joint name="slide" type="prismatic">
        <parent link="base" />
        <child link="slider" />
        <axis xyz="0 1 0" />
        <limit lower="1.0" upper="3.0" velocity="2.0" effort="4.0" />
        <dynamics damping="0.5" friction="0.7" />
      </joint>
    </robot>
  )";

  UrdfParser parser;
  const auto robot = parser.parseSkeletonString(urdfStr, "");
  ASSERT_TRUE(robot);

  auto* joint
      = dynamic_cast<dynamics::PrismaticJoint*>(robot->getJoint("slide"));
  ASSERT_NE(joint, nullptr);
  EXPECT_DOUBLE_EQ(joint->getPositionLowerLimit(0), 1.0);
  EXPECT_DOUBLE_EQ(joint->getPositionUpperLimit(0), 3.0);
  EXPECT_DOUBLE_EQ(joint->getRestPosition(0), 2.0);
  EXPECT_DOUBLE_EQ(joint->getDampingCoefficient(0), 0.5);
  EXPECT_DOUBLE_EQ(joint->getCoulombFriction(0), 0.7);
  EXPECT_NEAR(joint->getPosition(0), 2.0, 1e-12);
}

//==============================================================================
TEST(UrdfParser, MeshMaterialUsesGlobalColorAndCollisionMeshScale)
{
  const auto tempDir = makeTempDir("dart-urdf-mesh-material");
  const auto visualMeshPath = tempDir / "visual.obj";
  const auto collisionMeshPath = tempDir / "collision.obj";

  std::ofstream visualMeshFile(visualMeshPath);
  ASSERT_TRUE(visualMeshFile.is_open());
  visualMeshFile << "o Mesh\n"
                 << "v 0 0 0\n"
                 << "v 1 0 0\n"
                 << "v 0 1 0\n"
                 << "f 1 2 3\n";
  visualMeshFile.close();

  std::ofstream collisionMeshFile(collisionMeshPath);
  ASSERT_TRUE(collisionMeshFile.is_open());
  collisionMeshFile << "o Mesh\n"
                    << "v 0 0 0\n"
                    << "v 1 0 0\n"
                    << "v 0 1 0\n"
                    << "f 1 2 3\n";
  collisionMeshFile.close();

  const std::string urdfStr = R"(
    <robot name="mesh_material">
      <material name="box_color">
        <color rgba="0.2 0.4 0.6 0.8" />
      </material>
      <material name="mesh_color">
        <color rgba="0.7 0.8 0.9 1.0" />
      </material>
      <link name="link">
        <visual>
          <geometry>
            <box size="0.1 0.2 0.3" />
          </geometry>
          <material name="box_color" />
        </visual>
        <visual>
          <geometry>
            <mesh filename="visual.obj" scale="1 2 3" />
          </geometry>
          <material name="mesh_color" />
        </visual>
        <collision>
          <geometry>
            <mesh filename="collision.obj" scale="0.5 0.5 0.5" />
          </geometry>
        </collision>
      </link>
    </robot>
  )";

  UrdfParser parser;
  const Uri baseUri = Uri::createFromPath((tempDir / "model.urdf").string());
  const auto robot = parser.parseSkeletonString(urdfStr, baseUri);
  ASSERT_TRUE(robot);

  auto* body = robot->getBodyNode("link");
  ASSERT_NE(body, nullptr);

  bool foundVisualMesh = false;
  bool foundBoxColor = false;
  body->eachShapeNodeWith<dynamics::VisualAspect>(
      [&](dynamics::ShapeNode* node) {
        auto mesh
            = std::dynamic_pointer_cast<dynamics::MeshShape>(node->getShape());
        if (mesh) {
          foundVisualMesh = true;
          EXPECT_TRUE(
              mesh->getScale().isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));
          EXPECT_EQ(mesh->getColorMode(), dynamics::MeshShape::SHAPE_COLOR);
          return;
        }
        if (node->getShape()->is<dynamics::BoxShape>()) {
          const auto* visual = node->getVisualAspect();
          ASSERT_NE(visual, nullptr);
          foundBoxColor = true;
        }
      });
  EXPECT_TRUE(foundVisualMesh);
  EXPECT_TRUE(foundBoxColor);

  bool foundCollisionMesh = false;
  body->eachShapeNodeWith<dynamics::CollisionAspect>(
      [&](dynamics::ShapeNode* node) {
        auto mesh
            = std::dynamic_pointer_cast<dynamics::MeshShape>(node->getShape());
        if (!mesh) {
          return;
        }
        foundCollisionMesh = true;
        EXPECT_TRUE(mesh->getScale().isApprox(Eigen::Vector3d(0.5, 0.5, 0.5)));
      });
  EXPECT_TRUE(foundCollisionMesh);

  std::filesystem::remove_all(tempDir);
}

//==============================================================================
TEST(UrdfParser, VisualWithoutGeometryIsIgnored)
{
  const std::string urdfStr = R"(
    <robot name="bad_visual">
      <link name="base">
        <visual>
          <origin xyz="0 0 0" rpy="0 0 0" />
        </visual>
      </link>
    </robot>
  )";

  UrdfParser parser;
  const auto robot = parser.parseSkeletonString(urdfStr, "");
  ASSERT_TRUE(robot);
  auto* body = robot->getBodyNode("base");
  ASSERT_NE(body, nullptr);
  EXPECT_EQ(body->getNumShapeNodes(), 0u);
}

//==============================================================================
TEST(UrdfParser, TransmissionsAndPackageMeshes)
{
  const auto tempDir = makeTempDir("dart-urdf-package");
  const auto meshDir = tempDir / "meshes";
  std::filesystem::create_directories(meshDir);
  const auto meshPath = meshDir / "box.obj";

  std::ofstream meshFile(meshPath.string());
  ASSERT_TRUE(meshFile.is_open());
  meshFile << "o Box\n"
           << "v 0 0 0\n"
           << "v 1 0 0\n"
           << "v 0 1 0\n"
           << "f 1 2 3\n";
  meshFile.close();

  const std::string urdfStr = R"(
    <robot name="tx_robot">
      <link name="base">
        <inertial>
          <mass value="2.0"/>
          <origin xyz="0 0 0" rpy="0 0 0"/>
          <inertia ixx="0.2" iyy="0.2" izz="0.2" ixy="0" ixz="0" iyz="0"/>
        </inertial>
        <visual>
          <geometry>
            <mesh filename="package://test_pkg/meshes/box.obj"/>
          </geometry>
        </visual>
      </link>
      <link name="link1"/>
      <link name="link2"/>
      <joint name="joint1" type="revolute">
        <parent link="base"/>
        <child link="link1"/>
        <axis xyz="0 0 1"/>
        <limit lower="1.0" upper="2.0" effort="5" velocity="6"/>
        <dynamics damping="0.1" friction="0.2"/>
      </joint>
      <joint name="joint2" type="revolute">
        <parent link="link1"/>
        <child link="link2"/>
        <axis xyz="0 1 0"/>
        <limit lower="-1.0" upper="1.0" effort="2" velocity="3"/>
        <dynamics damping="0.3" friction="0.4"/>
      </joint>
      <transmission name="tx1">
        <type>transmission_interface/SimpleTransmission</type>
        <joint name="joint1">
          <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
        </joint>
        <actuator name="actuator">
          <mechanicalReduction>2</mechanicalReduction>
        </actuator>
      </transmission>
      <transmission name="tx2">
        <type>transmission_interface/SimpleTransmission</type>
        <joint name="joint2"/>
        <actuator name="actuator">
          <mechanicalReduction>4</mechanicalReduction>
        </actuator>
      </transmission>
    </robot>
  )";

  UrdfParser parser;
  parser.addPackageDirectory("test_pkg", tempDir.string());
  const auto robot = parser.parseSkeletonString(urdfStr, "");
  ASSERT_TRUE(robot);

  auto* base = robot->getBodyNode("base");
  ASSERT_NE(base, nullptr);
  bool foundMesh = false;
  base->eachShapeNodeWith<dynamics::VisualAspect>(
      [&](dynamics::ShapeNode* node) {
        if (std::dynamic_pointer_cast<dynamics::MeshShape>(node->getShape())) {
          foundMesh = true;
        }
      });
  EXPECT_TRUE(foundMesh);

  auto* joint2 = robot->getJoint("joint2");
  ASSERT_NE(joint2, nullptr);
  EXPECT_EQ(joint2->getActuatorType(), dart::dynamics::Joint::MIMIC);

  std::filesystem::remove_all(tempDir);
}

#if DART_IO_HAS_URDF
//==============================================================================
TEST(UrdfParser, VisualMaterialGlobalOverridesLocalColor)
{
  const std::string urdfXml = R"(
<robot name="material_override">
  <material name="shared">
    <color rgba="0.1 0.2 0.3 0.4" />
  </material>
  <link name="link">
    <visual name="shared_visual">
      <geometry>
        <box size="0.1 0.1 0.1" />
      </geometry>
      <material name="shared">
        <color rgba="0.9 0.8 0.7 0.6" />
      </material>
    </visual>
    <visual name="local_visual">
      <geometry>
        <box size="0.2 0.2 0.2" />
      </geometry>
      <material name="local">
        <color rgba="0.4 0.5 0.6 0.7" />
      </material>
    </visual>
  </link>
</robot>
  )";

  const auto tempPath
      = writeTempFileWithPrefix(urdfXml, "urdf_material_override", ".urdf");
  auto skeleton = dart::io::readSkeleton(common::Uri(tempPath.string()));
  std::error_code ec;
  std::filesystem::remove(tempPath, ec);

  ASSERT_NE(skeleton, nullptr);
  auto* body = skeleton->getBodyNode("link");
  ASSERT_NE(body, nullptr);

  bool foundShared = false;
  bool foundLocal = false;
  body->eachShapeNodeWith<dynamics::VisualAspect>(
      [&](dynamics::ShapeNode* node) {
        const auto* visual = node->getVisualAspect();
        ASSERT_NE(visual, nullptr);
        if (node->getName() == "shared_visual") {
          foundShared = true;
          EXPECT_TRUE(
              visual->getRGBA().isApprox(Eigen::Vector4d(0.1, 0.2, 0.3, 0.4)));
        } else if (node->getName() == "local_visual") {
          foundLocal = true;
          EXPECT_TRUE(
              visual->getRGBA().isApprox(Eigen::Vector4d(0.4, 0.5, 0.6, 0.7)));
        }
      });
  EXPECT_TRUE(foundShared);
  EXPECT_TRUE(foundLocal);
}

//==============================================================================
TEST(UrdfParser, CollisionNameAndOriginParsed)
{
  const std::string urdfXml = R"(
<robot name="collision_details">
  <link name="base">
    <collision name="col1">
      <origin xyz="0.1 0.2 0.3" rpy="0 0 0" />
      <geometry>
        <box size="0.1 0.1 0.1" />
      </geometry>
    </collision>
  </link>
</robot>
  )";

  const auto tempPath
      = writeTempFileWithPrefix(urdfXml, "urdf_collision_details", ".urdf");
  auto skeleton = dart::io::readSkeleton(common::Uri(tempPath.string()));
  std::error_code ec;
  std::filesystem::remove(tempPath, ec);

  ASSERT_NE(skeleton, nullptr);
  auto* body = skeleton->getBodyNode("base");
  ASSERT_NE(body, nullptr);

  bool foundCollision = false;
  body->eachShapeNodeWith<dynamics::CollisionAspect>(
      [&](dynamics::ShapeNode* node) {
        if (node->getName() != "col1") {
          return;
        }
        foundCollision = true;
        const auto translation = node->getRelativeTransform().translation();
        EXPECT_TRUE(translation.isApprox(Eigen::Vector3d(0.1, 0.2, 0.3)));
      });
  EXPECT_TRUE(foundCollision);
}
#endif
