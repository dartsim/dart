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

#include <dart/config.hpp>

#include <dart/utils/skel_parser.hpp>

#include <dart/all.hpp>
#include <dart/io/read.hpp>

#include <gtest/gtest.h>
#include <tinyxml2.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <string_view>
#include <vector>

#include <cmath>

using namespace dart;
using namespace math;
using namespace dynamics;
using namespace utils;
using namespace dart::test;

namespace {

std::filesystem::path makeTempSkelPath(const std::string& tag)
{
  static std::size_t counter = 0;
  const auto filename
      = "dart_skel_" + tag + "_" + std::to_string(counter++) + ".skel";
  return std::filesystem::temp_directory_path() / filename;
}

bool writeSkelXml(const std::filesystem::path& path, std::string_view xml)
{
  tinyxml2::XMLDocument doc;
  const std::string xmlCopy(xml);
  if (doc.Parse(xmlCopy.c_str()) != tinyxml2::XML_SUCCESS) {
    return false;
  }
  return doc.SaveFile(path.string().c_str()) == tinyxml2::XML_SUCCESS;
}

dynamics::SkeletonPtr readSkeletonFromSkelXml(std::string_view xml)
{
  const auto path = makeTempSkelPath("inline");
  if (!writeSkelXml(path, xml)) {
    return nullptr;
  }
  auto skeleton = dart::io::readSkeleton(path.string());
  std::error_code ec;
  std::filesystem::remove(path, ec);
  return skeleton;
}

std::string inertialBodyXml(std::string_view name, std::string_view extra = {})
{
  return "    <body name=\"" + std::string(name) + R"(">
      <inertia>
        <mass>1.0</mass>
        <moment_of_inertia>
          <ixx>0.1</ixx> <iyy>0.1</iyy> <izz>0.1</izz>
          <ixy>0</ixy> <ixz>0</ixz> <iyz>0</iyz>
        </moment_of_inertia>
      </inertia>
)" + std::string(extra)
         + "    </body>\n";
}

} // namespace

//==============================================================================
TEST(SkelParser, DataStructure)
{
  bool valBool = true;
  int valInt = -3;
  unsigned int valUInt = 1;
  float valFloat = -3.140f;
  double valDouble = 1.4576640;
  char valChar = 'd';
  Eigen::Vector2d valVector2d = Eigen::Vector2d::Random();
  Eigen::Vector3d valVector3d = Eigen::Vector3d::Random();
  Eigen::Vector3i valVector3i = Eigen::Vector3i::Random();
  Eigen::Vector6d valVector6d = Eigen::Vector6d::Random();
  Eigen::VectorXd valVectorXd = Eigen::VectorXd::Random(10);
  Eigen::Isometry3d valIsometry3d = Eigen::Isometry3d::Identity();

  std::string strBool = toString(valBool);
  std::string strInt = toString(valInt);
  std::string strUInt = toString(valUInt);
  std::string strFloat = toString(valFloat);
  std::string strDouble = toString(valDouble);
  std::string strChar = toString(valChar);
  std::string strVector2d = toString(valVector2d);
  std::string strVector3d = toString(valVector3d);
  std::string strVector3i = toString(valVector3i);
  std::string strVector6d = toString(valVector6d);
  std::string strVectorXd = toString(valVectorXd);
  std::string strIsometry3d = toString(valIsometry3d);

  EXPECT_EQ(valBool, toBool(strBool));
  EXPECT_EQ(valInt, toInt(strInt));
  EXPECT_EQ(valUInt, toUInt(strUInt));
  EXPECT_EQ(valFloat, toFloat(strFloat));
  EXPECT_EQ(valDouble, toDouble(strDouble));
  EXPECT_EQ(valChar, toChar(strChar));
  EXPECT_VECTOR_DOUBLE_EQ(valVector2d, toVector2d(strVector2d));
  EXPECT_VECTOR_DOUBLE_EQ(valVector3d, toVector3d(strVector3d));
  EXPECT_VECTOR_DOUBLE_EQ(valVector3i, toVector3i(strVector3i));
  EXPECT_VECTOR_DOUBLE_EQ(valVector6d, toVector6d(strVector6d));
  EXPECT_VECTOR_DOUBLE_EQ(valVectorXd, toVectorXd(strVectorXd));
  EXPECT_TRANSFORM_DOUBLE_EQ(valIsometry3d, toIsometry3d(strIsometry3d));
}

//==============================================================================
TEST(SkelParser, ReadSkeletonMissingFileReturnsNull)
{
  const auto missingPath
      = std::filesystem::temp_directory_path() / "dart_missing_skeleton.skel";
  std::error_code ec;
  std::filesystem::remove(missingPath, ec);

  EXPECT_EQ(SkelParser::readSkeleton(missingPath.string()), nullptr);
}

//==============================================================================
TEST(SkelParser, ReadSkeletonMissingRootElements)
{
  auto writeTemp = [](std::string_view xml) {
    const auto path = std::filesystem::temp_directory_path()
                      / "dart_skel_missing_root.skel";
    std::ofstream output(path.string(), std::ios::binary);
    output << xml;
    output.close();
    return path;
  };

  const auto noSkelPath = writeTemp("<root></root>");
  EXPECT_EQ(SkelParser::readSkeleton(noSkelPath.string()), nullptr);
  std::error_code ec;
  std::filesystem::remove(noSkelPath, ec);

  const auto noSkeletonPath = writeTemp(R"(
<skel version="1.0">
  <world name="default"/>
</skel>
)");
  EXPECT_EQ(SkelParser::readSkeleton(noSkeletonPath.string()), nullptr);
  std::filesystem::remove(noSkeletonPath, ec);
}

//==============================================================================
TEST(SkelParser, HandlesInvalidTopologyAndDofMetadataFromXml)
{
  const std::string deprecatedPlaneShape = R"(      <visualization_shape>
        <geometry>
          <plane>
            <normal>0 0 1</normal>
            <point>0 0 2</point>
          </plane>
        </geometry>
      </visualization_shape>
)";
  const std::string missingMeshShape = R"(      <visualization_shape>
        <geometry>
          <mesh>
            <file_name>missing_mesh_for_parser_validation.obj</file_name>
            <scale>1 1 1</scale>
          </mesh>
        </geometry>
      </visualization_shape>
)";
  const std::string skelXml
      = std::string(R"(<?xml version="1.0" ?>
<skel version="1.0">
  <skeleton name="validation_skel">
)") + inertialBodyXml("base")
        + inertialBodyXml("base") + inertialBodyXml("invalid_actuator_child")
        + inertialBodyXml("orphan_parent_child")
        + inertialBodyXml("orphan_child_parent") + inertialBodyXml("dup_child")
        + inertialBodyXml("unsupported_joint_child")
        + inertialBodyXml("dof_invalid_child")
        + inertialBodyXml("deprecated_plane", deprecatedPlaneShape)
        + inertialBodyXml("missing_mesh", missingMeshShape)
        + R"(    <joint type="free" name="root_joint">
      <parent>world</parent>
      <child>base</child>
    </joint>
    <joint type="revolute" name="invalid_actuator_joint" actuator="torque">
      <parent>base</parent>
      <child>invalid_actuator_child</child>
      <axis>
        <xyz>0 0 1</xyz>
      </axis>
    </joint>
    <joint type="weld" name="missing_parent_joint">
      <parent>missing_parent</parent>
      <child>orphan_parent_child</child>
    </joint>
    <joint type="weld" name="missing_child_joint">
      <parent>base</parent>
      <child>missing_child</child>
    </joint>
    <joint type="weld" name="dup_first">
      <parent>base</parent>
      <child>dup_child</child>
    </joint>
    <joint type="weld" name="dup_second">
      <parent>base</parent>
      <child>dup_child</child>
    </joint>
    <joint type="hinge" name="unsupported_joint">
      <parent>base</parent>
      <child>unsupported_joint_child</child>
    </joint>
    <joint type="ball" name="dof_invalid_joint">
      <parent>dup_child</parent>
      <child>dof_invalid_child</child>
      <dof local_index="3">
        <position initial="0.4" />
      </dof>
      <dof>
        <position initial="0.5" />
      </dof>
      <dof local_index="bad">
        <position initial="0.6" />
      </dof>
    </joint>
    <joint type="weld" name="deprecated_plane_joint">
      <parent>dof_invalid_child</parent>
      <child>deprecated_plane</child>
    </joint>
    <joint type="weld" name="missing_mesh_joint">
      <parent>deprecated_plane</parent>
      <child>missing_mesh</child>
    </joint>
  </skeleton>
</skel>
)";

  const auto skel = readSkeletonFromSkelXml(skelXml);
  ASSERT_NE(skel, nullptr);
  EXPECT_EQ(skel->getNumBodyNodes(), 6u);
  EXPECT_EQ(skel->getNumJoints(), 6u);

  auto* invalidActuator = dynamic_cast<dynamics::RevoluteJoint*>(
      skel->getJoint("invalid_actuator_joint"));
  ASSERT_NE(invalidActuator, nullptr);
  EXPECT_TRUE(invalidActuator->getAxis().isApprox(Eigen::Vector3d::UnitZ()));

  EXPECT_EQ(skel->getJoint("missing_parent_joint"), nullptr);
  EXPECT_EQ(skel->getJoint("missing_child_joint"), nullptr);
  EXPECT_EQ(skel->getJoint("dup_second"), nullptr);
  EXPECT_EQ(skel->getJoint("unsupported_joint"), nullptr);
  EXPECT_EQ(skel->getBodyNode("orphan_parent_child"), nullptr);
  EXPECT_EQ(skel->getBodyNode("orphan_child_parent"), nullptr);
  EXPECT_EQ(skel->getBodyNode("unsupported_joint_child"), nullptr);

  auto* duplicateChild = skel->getBodyNode("dup_child");
  ASSERT_NE(duplicateChild, nullptr);
  ASSERT_NE(duplicateChild->getParentJoint(), nullptr);
  EXPECT_EQ(duplicateChild->getParentJoint()->getName(), "dup_first");

  auto* ball
      = dynamic_cast<dynamics::BallJoint*>(skel->getJoint("dof_invalid_joint"));
  ASSERT_NE(ball, nullptr);
  EXPECT_EQ(ball->getNumDofs(), 3u);
  EXPECT_TRUE(ball->getPositions().isZero(1e-12));

  auto* deprecatedPlane = skel->getBodyNode("deprecated_plane");
  ASSERT_NE(deprecatedPlane, nullptr);
  ASSERT_EQ(deprecatedPlane->getNumShapeNodes(), 1u);
  EXPECT_EQ(deprecatedPlane->getShapeNode(0)->getShape(), nullptr);

  auto* missingMesh = skel->getBodyNode("missing_mesh");
  ASSERT_NE(missingMesh, nullptr);
  ASSERT_EQ(missingMesh->getNumShapeNodes(), 1u);
  EXPECT_EQ(missingMesh->getShapeNode(0)->getShape(), nullptr);
}

//==============================================================================
TEST(SkelParser, ParsesTwoDimensionalJointPlaneVariantsFromXml)
{
  const std::string skelXml = std::string(R"(<?xml version="1.0" ?>
<skel version="1.0">
  <skeleton name="plane_skel">
)") + inertialBodyXml("base") + inertialBodyXml("link_yz")
                              + inertialBodyXml("link_zx")
                              + inertialBodyXml("link_bad")
                              + inertialBodyXml("link_missing")
                              + inertialBodyXml("planar_missing")
                              + R"(    <joint type="free" name="root_joint">
      <parent>world</parent>
      <child>base</child>
    </joint>
    <joint type="translational2d" name="trans2d_yz">
      <parent>base</parent>
      <child>link_yz</child>
      <plane type="yz" />
    </joint>
    <joint type="translational2d" name="trans2d_zx">
      <parent>link_yz</parent>
      <child>link_zx</child>
      <plane type="zx" />
    </joint>
    <joint type="translational2d" name="trans2d_bad">
      <parent>link_zx</parent>
      <child>link_bad</child>
      <plane type="bad" />
    </joint>
    <joint type="translational2d" name="trans2d_missing">
      <parent>link_bad</parent>
      <child>link_missing</child>
    </joint>
    <joint type="planar" name="planar_missing">
      <parent>link_missing</parent>
      <child>planar_missing</child>
    </joint>
  </skeleton>
</skel>
)";

  const auto skel = readSkeletonFromSkelXml(skelXml);
  ASSERT_NE(skel, nullptr);

  auto* trans2dYz = dynamic_cast<dynamics::TranslationalJoint2D*>(
      skel->getJoint("trans2d_yz"));
  ASSERT_NE(trans2dYz, nullptr);
  EXPECT_EQ(
      trans2dYz->getPlaneType(), dynamics::TranslationalJoint2D::PlaneType::YZ);

  auto* trans2dZx = dynamic_cast<dynamics::TranslationalJoint2D*>(
      skel->getJoint("trans2d_zx"));
  ASSERT_NE(trans2dZx, nullptr);
  EXPECT_EQ(
      trans2dZx->getPlaneType(), dynamics::TranslationalJoint2D::PlaneType::ZX);

  auto* trans2dBad = dynamic_cast<dynamics::TranslationalJoint2D*>(
      skel->getJoint("trans2d_bad"));
  ASSERT_NE(trans2dBad, nullptr);
  EXPECT_EQ(
      trans2dBad->getPlaneType(),
      dynamics::TranslationalJoint2D::PlaneType::XY);

  auto* trans2dMissing = dynamic_cast<dynamics::TranslationalJoint2D*>(
      skel->getJoint("trans2d_missing"));
  ASSERT_NE(trans2dMissing, nullptr);
  EXPECT_EQ(
      trans2dMissing->getPlaneType(),
      dynamics::TranslationalJoint2D::PlaneType::XY);

  auto* planarMissing
      = dynamic_cast<dynamics::PlanarJoint*>(skel->getJoint("planar_missing"));
  ASSERT_NE(planarMissing, nullptr);
  EXPECT_EQ(
      planarMissing->getPlaneType(), dynamics::PlanarJoint::PlaneType::XY);
}

//==============================================================================
TEST(SkelParser, ParsesJointTypesLimitsAndDofsFromXml)
{
  const std::string skelXml = R"(<?xml version="1.0" ?>
<skel version="1.0">
  <skeleton name="skel">
    <body name="base">
      <inertia>
        <mass>1.0</mass>
        <moment_of_inertia>
          <ixx>0.1</ixx> <iyy>0.1</iyy> <izz>0.1</izz>
          <ixy>0</ixy> <ixz>0</ixz> <iyz>0</iyz>
        </moment_of_inertia>
      </inertia>
    </body>
    <body name="link1">
      <inertia>
        <mass>1.0</mass>
        <moment_of_inertia>
          <ixx>0.1</ixx> <iyy>0.1</iyy> <izz>0.1</izz>
          <ixy>0</ixy> <ixz>0</ixz> <iyz>0</iyz>
        </moment_of_inertia>
      </inertia>
    </body>
    <body name="link2">
      <inertia>
        <mass>1.0</mass>
        <moment_of_inertia>
          <ixx>0.1</ixx> <iyy>0.1</iyy> <izz>0.1</izz>
          <ixy>0</ixy> <ixz>0</ixz> <iyz>0</iyz>
        </moment_of_inertia>
      </inertia>
    </body>
    <body name="link3">
      <inertia>
        <mass>1.0</mass>
        <moment_of_inertia>
          <ixx>0.1</ixx> <iyy>0.1</iyy> <izz>0.1</izz>
          <ixy>0</ixy> <ixz>0</ixz> <iyz>0</iyz>
        </moment_of_inertia>
      </inertia>
    </body>
    <body name="link4">
      <inertia>
        <mass>1.0</mass>
        <moment_of_inertia>
          <ixx>0.1</ixx> <iyy>0.1</iyy> <izz>0.1</izz>
          <ixy>0</ixy> <ixz>0</ixz> <iyz>0</iyz>
        </moment_of_inertia>
      </inertia>
    </body>
    <body name="link5">
      <inertia>
        <mass>1.0</mass>
        <moment_of_inertia>
          <ixx>0.1</ixx> <iyy>0.1</iyy> <izz>0.1</izz>
          <ixy>0</ixy> <ixz>0</ixz> <iyz>0</iyz>
        </moment_of_inertia>
      </inertia>
    </body>
    <body name="link6">
      <inertia>
        <mass>1.0</mass>
        <moment_of_inertia>
          <ixx>0.1</ixx> <iyy>0.1</iyy> <izz>0.1</izz>
          <ixy>0</ixy> <ixz>0</ixz> <iyz>0</iyz>
        </moment_of_inertia>
      </inertia>
    </body>
    <body name="link7">
      <inertia>
        <mass>1.0</mass>
        <moment_of_inertia>
          <ixx>0.1</ixx> <iyy>0.1</iyy> <izz>0.1</izz>
          <ixy>0</ixy> <ixz>0</ixz> <iyz>0</iyz>
        </moment_of_inertia>
      </inertia>
    </body>
    <joint type="free" name="root_joint">
      <parent>world</parent>
      <child>base</child>
      <init_pos>0.1 0.2 0.3 0.4 0.5 0.6</init_pos>
      <init_vel>-0.1 -0.2 -0.3 -0.4 -0.5 -0.6</init_vel>
    </joint>
    <joint type="prismatic" name="prismatic_joint">
      <parent>base</parent>
      <child>link1</child>
      <axis>
        <xyz>1 0 0</xyz>
        <limit>
          <lower>-0.2</lower>
          <upper>0.4</upper>
        </limit>
        <dynamics>
          <damping>0.1</damping>
          <friction>0.2</friction>
          <spring_rest_position>0.3</spring_rest_position>
          <spring_stiffness>4.0</spring_stiffness>
        </dynamics>
      </axis>
      <dof local_index="0">
        <position lower="-0.2" upper="0.4" initial="0.1" />
        <damping>0.15</damping>
        <spring_stiffness>2.5</spring_stiffness>
      </dof>
    </joint>
    <joint type="screw" name="screw_joint">
      <parent>link1</parent>
      <child>link2</child>
      <axis>
        <xyz>0 0 1</xyz>
        <pitch>0.25</pitch>
        <limit>
          <lower>-0.6</lower>
          <upper>0.8</upper>
        </limit>
        <dynamics>
          <damping>0.35</damping>
          <spring_stiffness>1.7</spring_stiffness>
        </dynamics>
      </axis>
      <init_pos>0.15</init_pos>
      <init_vel>-0.25</init_vel>
      <dof local_index="0" name="screw_dof">
        <position lower="-0.6" upper="0.8" initial="0.15" />
      </dof>
    </joint>
    <joint type="translational" name="translational_joint">
      <parent>link2</parent>
      <child>link3</child>
      <init_pos>0.1 0.2 0.3</init_pos>
      <init_vel>-0.4 -0.5 -0.6</init_vel>
      <dof local_index="1">
        <position lower="-1.0" upper="1.0" initial="0.2" />
      </dof>
    </joint>
    <joint type="universal" name="universal_joint">
      <parent>link3</parent>
      <child>link4</child>
      <axis>
        <xyz>1 0 0</xyz>
        <limit>
          <lower>-0.4</lower>
          <upper>0.6</upper>
        </limit>
        <dynamics>
          <damping>0.2</damping>
          <spring_stiffness>0.9</spring_stiffness>
        </dynamics>
      </axis>
      <axis2>
        <xyz>0 1 0</xyz>
        <limit>
          <lower>-0.5</lower>
          <upper>0.7</upper>
        </limit>
        <dynamics>
          <damping>0.3</damping>
          <spring_stiffness>1.1</spring_stiffness>
        </dynamics>
      </axis2>
      <init_pos>0.1 0.2</init_pos>
      <init_vel>0.3 0.4</init_vel>
      <dof local_index="0">
        <position lower="-0.4" upper="0.6" initial="0.1" />
      </dof>
      <dof local_index="1">
        <position lower="-0.5" upper="0.7" initial="0.2" />
      </dof>
    </joint>
    <joint type="ball" name="ball_joint">
      <parent>link4</parent>
      <child>link5</child>
      <init_pos>0.1 0.2 0.3</init_pos>
      <init_vel>-0.1 -0.2 -0.3</init_vel>
      <dof local_index="2">
        <position lower="-0.3" upper="0.3" initial="0.2" />
      </dof>
    </joint>
    <joint type="planar" name="planar_joint">
      <parent>link5</parent>
      <child>link6</child>
      <plane type="arbitrary">
        <translation_axis1>
          <xyz>1 0 0</xyz>
        </translation_axis1>
        <translation_axis2>
          <xyz>0 1 0</xyz>
        </translation_axis2>
      </plane>
      <init_pos>0.2 0.3 0.4</init_pos>
      <init_vel>-0.2 -0.3 -0.4</init_vel>
    </joint>
    <joint type="translational2d" name="translational2d_joint">
      <parent>link6</parent>
      <child>link7</child>
      <plane type="arbitrary">
        <translation_axis1>
          <xyz>1 0 0</xyz>
        </translation_axis1>
        <translation_axis2>
          <xyz>0 1 0</xyz>
        </translation_axis2>
      </plane>
      <axis>
        <limit>
          <lower>-0.7</lower>
          <upper>0.9</upper>
        </limit>
        <dynamics>
          <damping>0.45</damping>
          <spring_stiffness>1.9</spring_stiffness>
        </dynamics>
      </axis>
      <axis2>
        <limit>
          <lower>-0.8</lower>
          <upper>1.0</upper>
        </limit>
        <dynamics>
          <damping>0.55</damping>
          <spring_stiffness>2.1</spring_stiffness>
        </dynamics>
      </axis2>
      <init_pos>0.4 0.5</init_pos>
      <init_vel>-0.6 -0.7</init_vel>
      <dof local_index="1" name="translational2d_y">
        <position lower="-0.8" upper="1.0" initial="0.5" />
      </dof>
    </joint>
  </skeleton>
</skel>
  )";

  const auto skel = readSkeletonFromSkelXml(skelXml);
  ASSERT_NE(skel, nullptr);

  auto* prismatic = dynamic_cast<dynamics::PrismaticJoint*>(
      skel->getJoint("prismatic_joint"));
  ASSERT_NE(prismatic, nullptr);
  EXPECT_DOUBLE_EQ(prismatic->getPositionLowerLimit(0), -0.2);
  EXPECT_DOUBLE_EQ(prismatic->getPositionUpperLimit(0), 0.4);
  EXPECT_DOUBLE_EQ(prismatic->getDampingCoefficient(0), 0.15);
  EXPECT_DOUBLE_EQ(prismatic->getSpringStiffness(0), 2.5);

  auto* screw
      = dynamic_cast<dynamics::ScrewJoint*>(skel->getJoint("screw_joint"));
  ASSERT_NE(screw, nullptr);
  EXPECT_TRUE(screw->getAxis().isApprox(Eigen::Vector3d::UnitZ()));
  EXPECT_DOUBLE_EQ(screw->getPitch(), 0.25);
  EXPECT_DOUBLE_EQ(screw->getPositionLowerLimit(0), -0.6);
  EXPECT_DOUBLE_EQ(screw->getPositionUpperLimit(0), 0.8);
  EXPECT_DOUBLE_EQ(screw->getDampingCoefficient(0), 0.35);
  EXPECT_DOUBLE_EQ(screw->getSpringStiffness(0), 1.7);
  EXPECT_DOUBLE_EQ(screw->getPosition(0), 0.15);
  EXPECT_DOUBLE_EQ(screw->getVelocity(0), -0.25);

  auto* translational = dynamic_cast<dynamics::TranslationalJoint*>(
      skel->getJoint("translational_joint"));
  ASSERT_NE(translational, nullptr);
  EXPECT_TRUE(
      translational->getPositions().isApprox(Eigen::Vector3d(0.1, 0.2, 0.3)));
  EXPECT_TRUE(translational->getVelocities().isApprox(
      Eigen::Vector3d(-0.4, -0.5, -0.6)));

  auto* universal = dynamic_cast<dynamics::UniversalJoint*>(
      skel->getJoint("universal_joint"));
  ASSERT_NE(universal, nullptr);
  EXPECT_DOUBLE_EQ(universal->getPositionLowerLimit(0), -0.4);
  EXPECT_DOUBLE_EQ(universal->getPositionUpperLimit(0), 0.6);
  EXPECT_DOUBLE_EQ(universal->getDampingCoefficient(0), 0.2);
  EXPECT_DOUBLE_EQ(universal->getSpringStiffness(0), 0.9);
  EXPECT_DOUBLE_EQ(universal->getPositionLowerLimit(1), -0.5);
  EXPECT_DOUBLE_EQ(universal->getPositionUpperLimit(1), 0.7);
  EXPECT_DOUBLE_EQ(universal->getDampingCoefficient(1), 0.3);
  EXPECT_DOUBLE_EQ(universal->getSpringStiffness(1), 1.1);

  auto* ball = dynamic_cast<dynamics::BallJoint*>(skel->getJoint("ball_joint"));
  ASSERT_NE(ball, nullptr);
  EXPECT_EQ(ball->getNumDofs(), 3u);

  auto* planar
      = dynamic_cast<dynamics::PlanarJoint*>(skel->getJoint("planar_joint"));
  ASSERT_NE(planar, nullptr);
  EXPECT_EQ(
      planar->getPlaneType(), dynamics::PlanarJoint::PlaneType::ARBITRARY);
  EXPECT_TRUE(planar->getPositions().isApprox(Eigen::Vector3d(0.2, 0.3, 0.4)));
  EXPECT_TRUE(
      planar->getVelocities().isApprox(Eigen::Vector3d(-0.2, -0.3, -0.4)));

  auto* translational2d = dynamic_cast<dynamics::TranslationalJoint2D*>(
      skel->getJoint("translational2d_joint"));
  ASSERT_NE(translational2d, nullptr);
  EXPECT_EQ(
      translational2d->getPlaneType(),
      dynamics::TranslationalJoint2D::PlaneType::ARBITRARY);
  EXPECT_TRUE(translational2d->getTranslationalAxis1().isApprox(
      Eigen::Vector3d::UnitX()));
  EXPECT_TRUE(translational2d->getTranslationalAxis2().isApprox(
      Eigen::Vector3d::UnitY()));
  EXPECT_DOUBLE_EQ(translational2d->getPositionLowerLimit(0), -0.7);
  EXPECT_DOUBLE_EQ(translational2d->getPositionUpperLimit(0), 0.9);
  EXPECT_DOUBLE_EQ(translational2d->getDampingCoefficient(0), 0.45);
  EXPECT_DOUBLE_EQ(translational2d->getSpringStiffness(0), 1.9);
  EXPECT_DOUBLE_EQ(translational2d->getPositionLowerLimit(1), -0.8);
  EXPECT_DOUBLE_EQ(translational2d->getPositionUpperLimit(1), 1.0);
  EXPECT_DOUBLE_EQ(translational2d->getDampingCoefficient(1), 0.55);
  EXPECT_DOUBLE_EQ(translational2d->getSpringStiffness(1), 2.1);
  EXPECT_TRUE(
      translational2d->getPositions().isApprox(Eigen::Vector2d(0.4, 0.5)));
  EXPECT_TRUE(
      translational2d->getVelocities().isApprox(Eigen::Vector2d(-0.6, -0.7)));
}

//==============================================================================
TEST(SkelParser, ParsesRigidShapeVariantsFromXml)
{
  static std::size_t counter = 0;
  const auto tempDir = std::filesystem::temp_directory_path()
                       / ("dart_skel_shapes_" + std::to_string(counter++));
  std::filesystem::create_directories(tempDir);

  const auto meshPath = tempDir / "mesh.obj";
  std::ofstream meshFile(meshPath.string(), std::ios::binary);
  ASSERT_TRUE(meshFile.is_open());
  meshFile << "o Mesh\n"
           << "v 0 0 0\n"
           << "v 1 0 0\n"
           << "v 0 1 0\n"
           << "f 1 2 3\n";
  meshFile.close();

  // Use absolute path with forward slashes so Windows resolves the mesh file
  std::string meshPathStr = meshPath.string();
  std::ranges::replace(meshPathStr, '\\', '/');

  const auto skelPath = tempDir / "shapes.skel";
  const std::string skelXml = std::string(R"(<?xml version="1.0" ?>
<skel version="1.0">
  <skeleton name="skel">
    <body name="box_body">
      <inertia>
        <mass>1.0</mass>
        <moment_of_inertia>
          <ixx>0.1</ixx> <iyy>0.1</iyy> <izz>0.1</izz>
          <ixy>0</ixy> <ixz>0</ixz> <iyz>0</iyz>
        </moment_of_inertia>
      </inertia>
      <visualization_shape>
        <geometry>
          <box><size>0.2 0.3 0.4</size></box>
        </geometry>
      </visualization_shape>
    </body>
    <body name="ellipsoid_body">
      <inertia>
        <mass>1.0</mass>
        <moment_of_inertia>
          <ixx>0.1</ixx> <iyy>0.1</iyy> <izz>0.1</izz>
          <ixy>0</ixy> <ixz>0</ixz> <iyz>0</iyz>
        </moment_of_inertia>
      </inertia>
      <visualization_shape>
        <geometry>
          <ellipsoid><size>0.1 0.2 0.3</size></ellipsoid>
        </geometry>
      </visualization_shape>
    </body>
    <body name="cylinder_body">
      <inertia>
        <mass>1.0</mass>
        <moment_of_inertia>
          <ixx>0.1</ixx> <iyy>0.1</iyy> <izz>0.1</izz>
          <ixy>0</ixy> <ixz>0</ixz> <iyz>0</iyz>
        </moment_of_inertia>
      </inertia>
      <visualization_shape>
        <geometry>
          <cylinder><radius>0.2</radius><height>0.5</height></cylinder>
        </geometry>
      </visualization_shape>
    </body>
    <body name="capsule_body">
      <inertia>
        <mass>1.0</mass>
        <moment_of_inertia>
          <ixx>0.1</ixx> <iyy>0.1</iyy> <izz>0.1</izz>
          <ixy>0</ixy> <ixz>0</ixz> <iyz>0</iyz>
        </moment_of_inertia>
      </inertia>
      <visualization_shape>
        <geometry>
          <capsule><radius>0.15</radius><height>0.4</height></capsule>
        </geometry>
      </visualization_shape>
    </body>
    <body name="mesh_body">
      <inertia>
        <mass>1.0</mass>
        <moment_of_inertia>
          <ixx>0.1</ixx> <iyy>0.1</iyy> <izz>0.1</izz>
          <ixy>0</ixy> <ixz>0</ixz> <iyz>0</iyz>
        </moment_of_inertia>
      </inertia>
      <visualization_shape>
        <geometry>
          <mesh>
            <file_name>)") + meshPathStr
                              + R"(</file_name>
            <scale>2 3 4</scale>
          </mesh>
        </geometry>
      </visualization_shape>
    </body>
    <joint type="free" name="root_joint">
      <parent>world</parent>
      <child>box_body</child>
    </joint>
    <joint type="weld" name="ellipsoid_joint">
      <parent>box_body</parent>
      <child>ellipsoid_body</child>
    </joint>
    <joint type="weld" name="cylinder_joint">
      <parent>ellipsoid_body</parent>
      <child>cylinder_body</child>
    </joint>
    <joint type="weld" name="capsule_joint">
      <parent>cylinder_body</parent>
      <child>capsule_body</child>
    </joint>
    <joint type="weld" name="mesh_joint">
      <parent>capsule_body</parent>
      <child>mesh_body</child>
    </joint>
  </skeleton>
</skel>
  )";

  ASSERT_TRUE(writeSkelXml(skelPath, skelXml));
  const auto skel = dart::io::readSkeleton(skelPath.string());
  ASSERT_NE(skel, nullptr);

  auto* boxBody = skel->getBodyNode("box_body");
  ASSERT_NE(boxBody, nullptr);
  EXPECT_TRUE(boxBody->getShapeNode(0)->getShape()->is<dynamics::BoxShape>());

  auto* ellipsoidBody = skel->getBodyNode("ellipsoid_body");
  ASSERT_NE(ellipsoidBody, nullptr);
  EXPECT_TRUE(ellipsoidBody->getShapeNode(0)
                  ->getShape()
                  ->is<dynamics::EllipsoidShape>());

  auto* cylinderBody = skel->getBodyNode("cylinder_body");
  ASSERT_NE(cylinderBody, nullptr);
  EXPECT_TRUE(
      cylinderBody->getShapeNode(0)->getShape()->is<dynamics::CylinderShape>());

  auto* capsuleBody = skel->getBodyNode("capsule_body");
  ASSERT_NE(capsuleBody, nullptr);
  EXPECT_TRUE(
      capsuleBody->getShapeNode(0)->getShape()->is<dynamics::CapsuleShape>());

  auto* meshBody = skel->getBodyNode("mesh_body");
  ASSERT_NE(meshBody, nullptr);
  auto meshShape = std::dynamic_pointer_cast<dynamics::MeshShape>(
      meshBody->getShapeNode(0)->getShape());
  ASSERT_NE(meshShape, nullptr);
  EXPECT_TRUE(meshShape->getScale().isApprox(Eigen::Vector3d(2.0, 3.0, 4.0)));

  std::error_code ec;
  std::filesystem::remove_all(tempDir, ec);
}

//==============================================================================
TEST(SkelParser, ParsesSoftBodyVariantsFromXml)
{
  const std::string skelXml = R"(<?xml version="1.0" ?>
<skel version="1.0">
  <skeleton name="soft_skel">
    <body name="soft_sphere">
      <inertia>
        <mass>1.0</mass>
        <moment_of_inertia>
          <ixx>0.1</ixx> <iyy>0.1</iyy> <izz>0.1</izz>
          <ixy>0</ixy> <ixz>0</ixz> <iyz>0</iyz>
        </moment_of_inertia>
      </inertia>
      <soft_shape>
        <total_mass>1.0</total_mass>
        <geometry>
          <sphere>
            <radius>0.2</radius>
            <num_slices>6</num_slices>
            <num_stacks>4</num_stacks>
          </sphere>
        </geometry>
        <kv>1.0</kv>
        <ke>2.0</ke>
        <damp>0.3</damp>
      </soft_shape>
    </body>
    <body name="soft_box">
      <inertia>
        <mass>1.0</mass>
        <moment_of_inertia>
          <ixx>0.1</ixx> <iyy>0.1</iyy> <izz>0.1</izz>
          <ixy>0</ixy> <ixz>0</ixz> <iyz>0</iyz>
        </moment_of_inertia>
      </inertia>
      <soft_shape>
        <total_mass>1.1</total_mass>
        <geometry>
          <box>
            <size>0.3 0.2 0.1</size>
            <frags>2 3 4</frags>
          </box>
        </geometry>
        <kv>1.2</kv>
        <ke>2.2</ke>
        <damp>0.4</damp>
      </soft_shape>
    </body>
    <body name="soft_ellipsoid">
      <inertia>
        <mass>1.0</mass>
        <moment_of_inertia>
          <ixx>0.1</ixx> <iyy>0.1</iyy> <izz>0.1</izz>
          <ixy>0</ixy> <ixz>0</ixz> <iyz>0</iyz>
        </moment_of_inertia>
      </inertia>
      <soft_shape>
        <total_mass>1.2</total_mass>
        <geometry>
          <ellipsoid>
            <size>0.2 0.3 0.4</size>
            <num_slices>8</num_slices>
            <num_stacks>5</num_stacks>
          </ellipsoid>
        </geometry>
        <kv>1.3</kv>
        <ke>2.3</ke>
        <damp>0.5</damp>
      </soft_shape>
    </body>
    <body name="soft_cylinder">
      <inertia>
        <mass>1.0</mass>
        <moment_of_inertia>
          <ixx>0.1</ixx> <iyy>0.1</iyy> <izz>0.1</izz>
          <ixy>0</ixy> <ixz>0</ixz> <iyz>0</iyz>
        </moment_of_inertia>
      </inertia>
      <soft_shape>
        <total_mass>1.3</total_mass>
        <geometry>
          <cylinder>
            <radius>0.15</radius>
            <height>0.4</height>
            <num_slices>8</num_slices>
            <num_stacks>5</num_stacks>
            <num_rings>3</num_rings>
          </cylinder>
        </geometry>
        <kv>1.4</kv>
        <ke>2.4</ke>
        <damp>0.6</damp>
      </soft_shape>
    </body>
    <joint type="free" name="root_joint">
      <parent>world</parent>
      <child>soft_sphere</child>
    </joint>
    <joint type="weld" name="soft_box_joint">
      <parent>soft_sphere</parent>
      <child>soft_box</child>
    </joint>
    <joint type="weld" name="soft_ellipsoid_joint">
      <parent>soft_box</parent>
      <child>soft_ellipsoid</child>
    </joint>
    <joint type="weld" name="soft_cylinder_joint">
      <parent>soft_ellipsoid</parent>
      <child>soft_cylinder</child>
    </joint>
  </skeleton>
</skel>
  )";

  const auto skel = readSkeletonFromSkelXml(skelXml);
  ASSERT_NE(skel, nullptr);
  ASSERT_EQ(skel->getNumSoftBodyNodes(), 4u);

  for (std::size_t i = 0; i < skel->getNumSoftBodyNodes(); ++i) {
    auto* softBody = skel->getSoftBodyNode(i);
    ASSERT_NE(softBody, nullptr);
    EXPECT_GT(softBody->getNumPointMasses(), 0u);
  }
}
