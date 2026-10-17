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
#include "dart/common/LocalResourceRetriever.hpp"
#include "dart/common/Uri.hpp"
#include "dart/config.hpp"
#include "dart/constraint/ConstraintSolver.hpp"
#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/BoxShape.hpp"
#include "dart/dynamics/FreeJoint.hpp"
#include "dart/dynamics/Joint.hpp"
#include "dart/dynamics/MeshShape.hpp"
#include "dart/dynamics/PlanarJoint.hpp"
#include "dart/dynamics/PlaneShape.hpp"
#include "dart/dynamics/PointMass.hpp"
#include "dart/dynamics/PrismaticJoint.hpp"
#include "dart/dynamics/RevoluteJoint.hpp"
#include "dart/dynamics/ScrewJoint.hpp"
#include "dart/dynamics/ShapeFrame.hpp"
#include "dart/dynamics/ShapeNode.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/dynamics/SoftBodyNode.hpp"
#include "dart/dynamics/UniversalJoint.hpp"
#include "dart/simulation/World.hpp"
#include "dart/utils/sdf/SdfParser.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gtest/gtest.h>
#include <sdf/Root.hh>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <future>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <cstddef>
#if HAVE_BULLET
  #include "dart/collision/bullet/bullet.hpp"
#endif

using namespace dart;
using namespace math;
using namespace dynamics;
using namespace simulation;
using namespace utils;

namespace {

struct SdfPoseResourceRetriever : PresentResourceRetriever
{
  std::string xml;

  std::string readAll(const common::Uri&) override
  {
    return xml;
  }
};

//==============================================================================
SdfParser::Options sdfMemoryOptions(const std::string& xml)
{
  auto retriever = std::make_shared<SdfPoseResourceRetriever>();
  retriever->xml = xml;
  SdfParser::Options options;
  options.mResourceRetriever = retriever;
  return options;
}

//==============================================================================
SkeletonPtr readSdfUniversalAxes(
    const std::string& axis,
    const std::string& axis2,
    const std::string& version)
{
  const auto options = sdfMemoryOptions(
      "<sdf version='" + version + "'><model name='axes'>"
      "<pose>1 2 3 0.1 -0.2 0.3</pose><link name='parent'/>"
      "<link name='child'><pose>0.4 0.5 0.6 0.4 -0.3 0.2</pose></link>"
      "<joint name='joint' type='universal'><parent>parent</parent>"
      "<child>child</child><pose>0.1 0.2 0.3 -0.2 0.5 -0.4</pose>"
      + axis + axis2 + "</joint></model></sdf>");
  return SdfParser::readSkeleton("memory://axes.sdf", options);
}

//==============================================================================
SkeletonPtr readSdfPose(
    const std::string& element,
    const std::string& attributes,
    const std::string& values,
    const std::string& version = "1.9")
{
  const auto pose = "<pose " + attributes + ">" + values + "</pose>";
  auto retriever = std::make_shared<SdfPoseResourceRetriever>();
  retriever->xml = "<sdf version='" + version + "'><world name='pose_world'><frame name='world_origin'/><model name='pose_model'>"
                  + (element == "model" ? pose
                                        : "<pose>2 -1 3 0.1 -0.2 0.3</pose>")
                  + "<link name='parent'><pose>5 6 7 0.3 -0.2 0.1</pose></link>"
                    "<link name='child'>"
                  + (element == "link" ? pose
                                       : "<pose>0.4 0.5 0.6 0.2 0.1 -0.3</pose>")
                  + "<inertial>"
                  + (element == "inertial" ? pose : "")
                  + "<mass>1</mass><inertia><ixx>1</ixx><iyy>2</iyy>"
                    "<izz>3</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>"
                    "</inertia></inertial>";
  if (element == "visual" || element == "collision") {
    retriever->xml += "<" + element + " name='shape'>" + pose
                      + "<geometry><box><size>1 2 3</size></box></geometry></"
                      + element + ">";
  }
  retriever->xml += "</link><joint name='joint' type='fixed'>"
                    "<parent>parent</parent><child>child</child>"
                    + (element == "joint" ? pose : "")
                    + "</joint></model></world></sdf>";
  SdfParser::Options options;
  options.mResourceRetriever = retriever;
  const auto world = SdfParser::readWorld("memory://pose.sdf", options);
  return world ? world->getSkeleton(0) : nullptr;
}

//==============================================================================
Eigen::Isometry3d sdfPoseTransform(
    const SkeletonPtr& skeleton, const std::string& element)
{
  auto* child = skeleton->getBodyNode("child");
  if (element == "model")
    return skeleton->getBodyNode("parent")->getWorldTransform();
  if (element == "link")
    return child->getWorldTransform();
  if (element == "joint")
    return skeleton->getJoint("joint")->getTransformFromChildBodyNode();
  if (element == "visual" || element == "collision")
    return child->getShapeNode(0)->getRelativeTransform();
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = child->getLocalCOM();
  return transform;
}

//==============================================================================
void expectSdfPoseError(
    const std::string& element,
    const std::string& attributes,
    const std::string& values)
{
  EXPECT_EQ(readSdfPose(element, attributes, values), nullptr);
}

struct SdfFixtureResourceRetriever : common::ResourceRetriever
{
  std::vector<std::string> requests;
  common::LocalResourceRetriever local;

  bool exists(const common::Uri& uri) override
  {
    const auto path = fixturePath(uri);
    return !path.empty() && local.exists(common::Uri(path));
  }

  common::ResourcePtr retrieve(const common::Uri& uri) override
  {
    requests.push_back(uri.toString());
    const auto path = fixturePath(uri);
    return path.empty() ? nullptr : local.retrieve(common::Uri(path));
  }

  std::string getFilePath(const common::Uri&) override
  {
    return "";
  }

  static std::string fixturePath(const common::Uri& uri)
  {
    const std::string prefix = "memory://pkg/";
    const auto resource = uri.toString();
    if (resource.compare(0, prefix.size(), prefix) != 0)
      return "";
    return std::string(DART_DATA_PATH) + "/sdf/test/include_relative_mesh/"
           + resource.substr(prefix.size());
  }
};

struct TemporarySdfDirectory
{
  std::filesystem::path path
      = std::filesystem::temp_directory_path()
        / ("dart-sdf-regression-"
           + std::to_string(
               std::chrono::steady_clock::now().time_since_epoch().count()));

  TemporarySdfDirectory()
  {
    std::filesystem::create_directory(path);
  }

  ~TemporarySdfDirectory()
  {
    std::error_code error;
    std::filesystem::remove_all(path, error);
  }

  void write(const std::string& file, const std::string& contents) const
  {
    std::ofstream stream(path / file);
    stream << contents;
    ASSERT_TRUE(stream);
  }
};

constexpr const char* sdfPoseElements[]
    = {"model", "link", "joint", "visual", "collision", "inertial"};

} // namespace

//==============================================================================
TEST(SdfParser, QuaternionPosesMatchEuler)
{
  struct Rotation
  {
    const char* quaternion;
    const char* euler;
  };
  const Rotation rotations[] = {
      {"0 0 0 1", "0 0 0"},
      {"2 0 0 2", "1.570796326794896619 0 0"},
      {"0 2 0 2", "0 1.570796326794896619 0"},
      {"0 0 2 2", "0 0 1.570796326794896619"},
      {"0 0 1e-300 1e-300", "0 0 1.570796326794896619"},
      {"0 0 1e300 1e300", "0 0 1.570796326794896619"},
  };
  for (const auto* element : sdfPoseElements) {
    for (const auto& rotation : rotations) {
      SCOPED_TRACE(element);
      SCOPED_TRACE(rotation.quaternion);
      const auto quaternion = readSdfPose(
          element,
          "rotation_format='quat_xyzw'",
          std::string("1 2 3 ") + rotation.quaternion);
      const auto euler = readSdfPose(
          element,
          "rotation_format='euler_rpy'",
          std::string("1 2 3 ") + rotation.euler);
      ASSERT_NE(quaternion, nullptr);
      ASSERT_NE(euler, nullptr);
      EXPECT_TRUE(
          sdfPoseTransform(quaternion, element)
              .matrix()
              .isApprox(sdfPoseTransform(euler, element).matrix(), 1e-12));
    }
  }
}

//==============================================================================
TEST(SdfParser, EulerDegreesPosesMatchRadians)
{
  const std::string radians
      = "1 2 3 0.523598775598298873 -0.785398163397448309 "
        "1.047197551196597746";
  for (const auto* element : sdfPoseElements) {
    SCOPED_TRACE(element);
    const auto reference = readSdfPose(element, "", radians);
    ASSERT_NE(reference, nullptr);
    for (const auto* degrees : {"true", "1", "TRUE", " true "}) {
      SCOPED_TRACE(degrees);
      const auto skeleton = readSdfPose(
          element,
          std::string("rotation_format='euler_rpy' degrees='") + degrees + "'",
          "1 2 3 30 -45 60");
      ASSERT_NE(skeleton, nullptr);
      EXPECT_TRUE(
          sdfPoseTransform(skeleton, element)
              .matrix()
              .isApprox(sdfPoseTransform(reference, element).matrix(), 1e-12));
    }
    for (const auto* degrees : {"false", "0"}) {
      const auto skeleton = readSdfPose(
          element, std::string("degrees='") + degrees + "'", radians);
      ASSERT_NE(skeleton, nullptr);
      EXPECT_TRUE(
          sdfPoseTransform(skeleton, element)
              .matrix()
              .isApprox(sdfPoseTransform(reference, element).matrix(), 1e-12));
    }
  }
}

//==============================================================================
TEST(SdfParser, RotatedInertialPoseTransformsMomentAtCom)
{
  Eigen::Matrix3d expectedMoment;
  expectedMoment << 5, -0.2, -0.4, -0.2, 4, 0.3, -0.4, 0.3, 6;
  for (const auto* pose :
       {"<pose>1 2 3 0 0 1.570796326794896619</pose>",
        "<pose degrees='true'>1 2 3 0 0 90</pose>",
        "<pose rotation_format='quat_xyzw'>1 2 3 0 0 1 1</pose>"}) {
    SCOPED_TRACE(pose);
    auto retriever = std::make_shared<SdfPoseResourceRetriever>();
    retriever->xml
        = "<sdf version='1.9'><model name='inertia'><link name='body'>"
          "<pose>4 5 6 0.2 0.3 0.4</pose><inertial>"
          + std::string(pose)
          + "<mass>2</mass><inertia><ixx>4</ixx><iyy>5</iyy><izz>6</izz>"
            "<ixy>0.2</ixy><ixz>0.3</ixz><iyz>0.4</iyz></inertia>"
            "</inertial></link></model></sdf>";
    SdfParser::Options options;
    options.mResourceRetriever = retriever;
    const auto skeleton
        = SdfParser::readSkeleton("memory://inertia.sdf", options);
    ASSERT_NE(skeleton, nullptr);
    const auto* body = skeleton->getBodyNode("body");
    ASSERT_NE(body, nullptr);
    EXPECT_DOUBLE_EQ(body->getMass(), 2.0);
    EXPECT_TRUE(body->getLocalCOM().isApprox(Eigen::Vector3d(1, 2, 3), 1e-12));
    // The moment is about the COM; translation must not add a parallel-axis
    // term.
    EXPECT_TRUE(body->getInertia().getMoment().isApprox(expectedMoment, 1e-12));
  }
}

//==============================================================================
TEST(SdfParser, SoftShapePoseTransformsEveryRestingPosition)
{
  const char* geometries[]
      = {"<sphere><radius>0.5</radius><num_slices>4</num_slices>"
         "<num_stacks>3</num_stacks></sphere>",
         "<box><size>1 2 3</size><frags>2 2 2</frags></box>",
         "<ellipsoid><size>1 2 3</size><num_slices>4</num_slices>"
         "<num_stacks>3</num_stacks></ellipsoid>",
         "<cylinder><radius>0.5</radius><height>1.3</height>"
         "<num_slices>4</num_slices><num_stacks>2</num_stacks>"
         "<num_rings>2</num_rings></cylinder>"};
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(1, 2, 3);
  transform.linear()
      = Eigen::AngleAxisd(1.570796326794896619, Eigen::Vector3d::UnitZ())
            .toRotationMatrix();
  for (const auto* geometry : geometries) {
    SCOPED_TRACE(geometry);
    const auto readSoftShape = [geometry](const std::string& pose) {
      auto retriever = std::make_shared<SdfPoseResourceRetriever>();
      retriever->xml
          = "<sdf version='1.9'><model name='soft'><link name='body'>"
            "<inertial><mass>1</mass></inertial>"
            "<soft_shape><total_mass>1</total_mass>"
            + pose + "<geometry>" + geometry
            + "</geometry></soft_shape></link></model></sdf>";
      SdfParser::Options options;
      options.mResourceRetriever = retriever;
      return SdfParser::readSkeleton("memory://soft.sdf", options);
    };
    const auto reference = readSoftShape("");
    ASSERT_NE(reference, nullptr);
    const auto* referenceBody = reference->getSoftBodyNode("body");
    ASSERT_NE(referenceBody, nullptr);
    ASSERT_GT(referenceBody->getNumPointMasses(), 0u);
    for (const auto* pose :
         {"<pose>1 2 3 0 0 1.570796326794896619</pose>",
          "<pose degrees='true'>1 2 3 0 0 90</pose>",
          "<pose rotation_format='quat_xyzw'>1 2 3 0 0 1 1</pose>"}) {
      SCOPED_TRACE(pose);
      const auto skeleton = readSoftShape(pose);
      ASSERT_NE(skeleton, nullptr);
      const auto* body = skeleton->getSoftBodyNode("body");
      ASSERT_NE(body, nullptr);
      ASSERT_EQ(body->getNumPointMasses(), referenceBody->getNumPointMasses());
      for (std::size_t i = 0; i < body->getNumPointMasses(); ++i) {
        SCOPED_TRACE(i);
        EXPECT_TRUE(body->getPointMass(i)->getRestingPosition().isApprox(
            transform * referenceBody->getPointMass(i)->getRestingPosition(),
            1e-12));
      }
    }
  }
}

//==============================================================================
TEST(SdfParser, EmptyPosesUseIdentity)
{
  for (const auto* element : sdfPoseElements) {
    SCOPED_TRACE(element);
    const auto reference = readSdfPose(element, "", "0 0 0 0 0 0");
    ASSERT_NE(reference, nullptr);
    for (const auto* attributes :
         {"",
          "rotation_format='euler_rpy'",
          "degrees='true'",
          "rotation_format='quat_xyzw'"}) {
      SCOPED_TRACE(attributes);
      for (const auto* values : {"", " \t\n "}) {
        const auto skeleton = readSdfPose(element, attributes, values);
        ASSERT_NE(skeleton, nullptr);
        EXPECT_TRUE(
            sdfPoseTransform(skeleton, element)
                .matrix()
                .isApprox(
                    sdfPoseTransform(reference, element).matrix(), 1e-12));
      }
    }
  }
}

//==============================================================================
TEST(SdfParser, SupportedVersionsLoadPoseAttributes)
{
  for (const auto* version : {"1.4", "1.5", "1.6", "1.7", "1.8", "1.9"}) {
    SCOPED_TRACE(version);
    const auto radians
        = readSdfPose("visual", "", "1 2 3 0 0 1.570796326794896619", version);
    const auto degrees
        = readSdfPose("visual", "degrees='true'", "1 2 3 0 0 90", version);
    const auto quaternion = readSdfPose(
        "visual", "rotation_format='quat_xyzw'", "1 2 3 0 0 1 1", version);
    ASSERT_NE(radians, nullptr);
    ASSERT_NE(degrees, nullptr);
    ASSERT_NE(quaternion, nullptr);
    EXPECT_TRUE(
        sdfPoseTransform(degrees, "visual")
            .matrix()
            .isApprox(sdfPoseTransform(radians, "visual").matrix(), 1e-12));
    EXPECT_TRUE(
        sdfPoseTransform(quaternion, "visual")
            .matrix()
            .isApprox(sdfPoseTransform(radians, "visual").matrix(), 1e-12));
    auto retriever = std::make_shared<SdfPoseResourceRetriever>();
    retriever->xml = "<sdf version='" + std::string(version)
                     + "'><world name='test_world'><model name='model'>"
                       "<pose rotation_format='quat_xyzw'>1 2 3 0 0 1 1</pose>"
                       "<link name='body'/></model></world></sdf>";
    SdfParser::Options options;
    options.mResourceRetriever = retriever;
    EXPECT_NE(SdfParser::readWorld("memory://version.sdf", options), nullptr);
  }
}

//==============================================================================
TEST(SdfParser, UnqualifiedVersionsAreRejected)
{
  for (const auto* version :
       {"", "unknown", "2.0", "1.70", "1.7.1", "1.7junk"}) {
    SCOPED_TRACE(version);
    EXPECT_EQ(readSdfPose("model", "", "0 0 0 0 0 0", version), nullptr);
    auto retriever = std::make_shared<SdfPoseResourceRetriever>();
    retriever->xml = "<sdf version='" + std::string(version)
                     + "'><world name='test_world'/></sdf>";
    SdfParser::Options options;
    options.mResourceRetriever = retriever;
    EXPECT_EQ(SdfParser::readWorld("memory://version.sdf", options), nullptr);
  }
}

//==============================================================================
TEST(SdfParser, UniversalAxesRespectExpressedInFrames)
{
  const auto rotation = [](double roll, double pitch, double yaw) {
    return (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
            * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
            * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
        .toRotationMatrix();
  };
  const Eigen::Matrix3d modelRotation = rotation(0.1, -0.2, 0.3);
  const Eigen::Matrix3d worldJointRotation
      = modelRotation * rotation(0.4, -0.3, 0.2) * rotation(-0.2, 0.5, -0.4);
  const Eigen::Matrix3d modelToJointRotation
      = worldJointRotation.transpose() * modelRotation;
  struct AxisFrame
  {
    const char* attributes;
    const char* legacy;
    bool modelFrame;
  };
  const AxisFrame frames[]
      = {{"expressed_in='__model__'", "", true},
         {"expressed_in=''", "", false},
         {"", "", false}};
  for (const auto* version : {"1.7", "1.8", "1.9"}) {
    SCOPED_TRACE(version);
    for (const auto& frame : frames) {
      SCOPED_TRACE(frame.attributes);
      SCOPED_TRACE(frame.legacy);
      const auto skeleton = readSdfUniversalAxes(
          "<axis><xyz " + std::string(frame.attributes) + ">1 0 0</xyz>"
              + frame.legacy + "</axis>",
          "<axis2><xyz " + std::string(frame.attributes) + ">0 1 0</xyz>"
              + frame.legacy + "</axis2>",
          version);
      ASSERT_NE(skeleton, nullptr);
      const auto* joint
          = dynamic_cast<const UniversalJoint*>(skeleton->getJoint("joint"));
      ASSERT_NE(joint, nullptr);
      Eigen::Matrix3d expectedRotation = Eigen::Matrix3d::Identity();
      if (frame.modelFrame)
        expectedRotation = modelToJointRotation;
      EXPECT_TRUE(joint->getAxis1().isApprox(expectedRotation.col(0), 1e-12));
      EXPECT_TRUE(joint->getAxis2().isApprox(expectedRotation.col(1), 1e-12));
    }
  }
}

//==============================================================================
TEST(SdfParser, LegacyAxisFrameIsConvertedBySdformat)
{
  const Eigen::Matrix3d childRotation
      = (Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitZ())
         * Eigen::AngleAxisd(-0.3, Eigen::Vector3d::UnitY())
         * Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitX()))
            .toRotationMatrix();
  const Eigen::Matrix3d jointRotation
      = (Eigen::AngleAxisd(-0.4, Eigen::Vector3d::UnitZ())
         * Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitY())
         * Eigen::AngleAxisd(-0.2, Eigen::Vector3d::UnitX()))
            .toRotationMatrix();
  const Eigen::Matrix3d modelToJoint
      = (childRotation * jointRotation).transpose();
  for (const auto* version : {"1.5", "1.6"}) {
    const auto skeleton = readSdfUniversalAxes(
        "<axis><xyz>1 0 "
        "0</xyz><use_parent_model_frame>true</use_parent_model_frame></axis>",
        "<axis2><xyz>0 1 "
        "0</xyz><use_parent_model_frame>true</use_parent_model_frame></axis2>",
        version);
    ASSERT_NE(skeleton, nullptr);
    const auto* joint
        = dynamic_cast<const UniversalJoint*>(skeleton->getJoint("joint"));
    ASSERT_NE(joint, nullptr);
    EXPECT_TRUE(joint->getAxis1().isApprox(modelToJoint.col(0), 1e-12));
    EXPECT_TRUE(joint->getAxis2().isApprox(modelToJoint.col(1), 1e-12));
  }
}

//==============================================================================
TEST(SdfParser, UniversalAxesRejectUnknownFrames)
{
  for (const auto* axis : {"axis", "axis2"}) {
    const std::string invalid = "<" + std::string(axis)
                                + "><xyz expressed_in='missing'>1 0 0</xyz></"
                                + axis + ">";
    EXPECT_EQ(
        readSdfUniversalAxes(
            std::string(axis) == "axis" ? invalid
                                        : "<axis><xyz>1 0 0</xyz></axis>",
            std::string(axis) == "axis2" ? invalid
                                         : "<axis2><xyz>0 1 0</xyz></axis2>",
            "1.9"),
        nullptr);
  }
}

//==============================================================================
TEST(SdfParser, NamedFramesResolvePosesAxesAndAttachmentAliases)
{
  const auto options = sdfMemoryOptions(R"(
    <sdf version='1.9'><world name='test_world'>
      <frame name='origin'><pose>10 0 0 0 0 1.570796326794896619</pose></frame>
      <model name='framed'><pose relative_to='origin'>2 0 0 0 0 0</pose>
        <link name='base'/>
        <frame name='offset' attached_to='base'><pose>1 0 0 0 0 0</pose></frame>
        <link name='child'><pose relative_to='offset'>0 2 0 0 0 0</pose>
          <collision name='shape'><pose relative_to='offset'>0 3 0 0 0 0</pose>
            <geometry><box><size>1 1 1</size></box></geometry></collision>
        </link>
        <frame name='child_alias' attached_to='child'/>
        <joint name='joint' type='revolute'><parent>offset</parent>
          <child>child_alias</child><pose relative_to='offset'>0 0 0 0 0 0</pose>
          <axis><xyz expressed_in='offset'>1 0 0</xyz></axis>
        </joint>
      </model>
    </world></sdf>)");
  const auto world = SdfParser::readWorld("memory://frames.sdf", options);
  ASSERT_NE(world, nullptr);
  const auto skeleton = world->getSkeleton("framed");
  ASSERT_NE(skeleton, nullptr);
  ASSERT_EQ(skeleton->getNumBodyNodes(), 2u);
  const auto* base = skeleton->getBodyNode("base");
  const auto* child = skeleton->getBodyNode("child");
  ASSERT_NE(base, nullptr);
  ASSERT_NE(child, nullptr);
  EXPECT_EQ(child->getParentBodyNode(), base);
  EXPECT_TRUE(base->getWorldTransform().translation().isApprox(
      Eigen::Vector3d(10, 2, 0), 1e-12));
  EXPECT_TRUE(child->getWorldTransform().translation().isApprox(
      Eigen::Vector3d(8, 3, 0), 1e-12));
  ASSERT_EQ(child->getNumShapeNodes(), 1u);
  EXPECT_TRUE(
      child->getShapeNode(0)->getWorldTransform().translation().isApprox(
          Eigen::Vector3d(7, 3, 0), 1e-12));
  const auto* joint
      = dynamic_cast<const RevoluteJoint*>(skeleton->getJoint("joint"));
  ASSERT_NE(joint, nullptr);
  EXPECT_TRUE(joint->getAxis().isApprox(Eigen::Vector3d::UnitX(), 1e-12));
  EXPECT_TRUE(joint->getTransformFromChildBodyNode().translation().isApprox(
      Eigen::Vector3d(0, -2, 0), 1e-12));
}

//==============================================================================
TEST(SdfParser, PlacementFramePositionsTheNamedFrame)
{
  const auto options = sdfMemoryOptions(R"(
    <sdf version='1.9'><world name='test_world'><model name='placed' placement_frame='anchor'>
      <pose>4 5 6 0 0 0</pose><link name='body'/>
      <frame name='anchor' attached_to='body'><pose>1 2 3 0 0 0</pose></frame>
    </model></world></sdf>)");
  const auto world = SdfParser::readWorld("memory://placed.sdf", options);
  ASSERT_NE(world, nullptr);
  ASSERT_EQ(world->getNumSkeletons(), 1u);
  EXPECT_TRUE(world->getSkeleton(0)
                  ->getBodyNode(0)
                  ->getWorldTransform()
                  .translation()
                  .isApprox(Eigen::Vector3d(3, 3, 3), 1e-12));
}

//==============================================================================
TEST(SdfParser, RejectsNestedModelsForEverySupportedVersion)
{
  for (const auto* version : {"1.4", "1.5", "1.6", "1.7", "1.8", "1.9"}) {
    const auto options = sdfMemoryOptions("<sdf version='" + std::string(version)
        + "'><model name='outer'><link name='body'/><model name='nested'><link name='nested_body'/></model></model></sdf>");
    EXPECT_EQ(SdfParser::readSkeleton("memory://nested.sdf", options), nullptr);
  }
}

//==============================================================================
TEST(SdfParser, ModernJointsRejectUnknownLinksAndAcceptWorldParent)
{
  for (const auto* version : {"1.7", "1.8", "1.9"}) {
    SCOPED_TRACE(version);
    for (const auto* parent : {"world", "unknown_parent"}) {
      SCOPED_TRACE(parent);
      for (const auto* child : {"body", "unknown_child", "world"}) {
        SCOPED_TRACE(child);
        const std::string model
            = "<model name='links'><link name='body'/>"
              "<joint name='joint' type='fixed'><parent>"
              + std::string(parent) + "</parent><child>" + child
              + "</child></joint></model>";
        const auto options = sdfMemoryOptions(
            "<sdf version='" + std::string(version) + "'>" + model + "</sdf>");
        const auto worldOptions = sdfMemoryOptions(
            "<sdf version='" + std::string(version)
            + "'><world name='test_world'>" + model + "</world></sdf>");
        if (std::string(parent) == "world" && std::string(child) == "body") {
          const auto skeleton
              = SdfParser::readSkeleton("memory://links.sdf", options);
          ASSERT_NE(skeleton, nullptr);
          EXPECT_EQ(skeleton->getNumBodyNodes(), 1u);
          const auto world
              = SdfParser::readWorld("memory://links.sdf", worldOptions);
          ASSERT_NE(world, nullptr);
          EXPECT_EQ(world->getNumSkeletons(), 1u);
        } else {
          EXPECT_EQ(
              SdfParser::readSkeleton("memory://links.sdf", options), nullptr);
          EXPECT_EQ(
              SdfParser::readWorld("memory://links.sdf", worldOptions),
              nullptr);
        }
      }
    }
  }
}

//==============================================================================
TEST(SdfParser, PoseFramesWorldPreservesInertiaAndSoftPose)
{
  const auto world
      = SdfParser::readWorld("dart://sample/sdf/test/pose_frames.world");
  ASSERT_NE(world, nullptr);
  ASSERT_EQ(world->getNumSkeletons(), 2u);
  const auto rigid = world->getSkeleton("inertia_frame");
  ASSERT_NE(rigid, nullptr);
  EXPECT_EQ(rigid->getNumSoftBodyNodes(), 0u);
  const auto* body = rigid->getBodyNode("rotated_inertia");
  ASSERT_NE(body, nullptr);
  EXPECT_TRUE(body->getLocalCOM().isApprox(Eigen::Vector3d(0.15, 0, 0), 1e-12));
  Eigen::Matrix3d expectedMoment;
  expectedMoment << 0.1, -0.06, 0, -0.06, 0.1, 0, 0, 0, 0.18;
  EXPECT_TRUE(body->getInertia().getMoment().isApprox(expectedMoment, 1e-12));
  const auto soft = world->getSkeleton("soft_frame");
  ASSERT_NE(soft, nullptr);
  ASSERT_EQ(soft->getNumSoftBodyNodes(), 1u);
  const auto* softBody = soft->getSoftBodyNode("posed_soft_ellipsoid");
  ASSERT_NE(softBody, nullptr);
  ASSERT_EQ(softBody->getNumPointMasses(), 114u);
  Eigen::Vector3d meanRestPosition = Eigen::Vector3d::Zero();
  for (std::size_t i = 0; i < softBody->getNumPointMasses(); ++i)
    meanRestPosition += softBody->getPointMass(i)->getRestingPosition();
  meanRestPosition /= softBody->getNumPointMasses();
  EXPECT_TRUE(meanRestPosition.isApprox(Eigen::Vector3d(0.3, 0.3, 0.2), 1e-12));
}

//==============================================================================
TEST(SdfParser, MassOnlyInertiaKeepsLegacyMassTimesIdentityFallback)
{
  const auto readMoment = [](const std::string& inertial) {
    const auto options = sdfMemoryOptions(
        "<sdf version='1.9'><model name='mass_only'><link name='link'>"
        + inertial + "</link></model></sdf>");
    const auto skeleton
        = SdfParser::readSkeleton("memory://mass-only.sdf", options);
    EXPECT_NE(skeleton, nullptr);
    if (!skeleton)
      return Eigen::Matrix3d::Zero().eval();
    const auto* body = skeleton->getBodyNode("link");
    EXPECT_NE(body, nullptr);
    return body ? body->getInertia().getMoment() : Eigen::Matrix3d::Zero();
  };

  EXPECT_TRUE(readMoment("<inertial><mass>100</mass></inertial>")
                  .isApprox(100.0 * Eigen::Matrix3d::Identity(), 1e-12));
  EXPECT_TRUE(
      readMoment("<inertial><mass>100</mass><inertia><ixx>1</ixx><iyy>1</iyy>"
                 "<izz>1</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>"
                 "</inertial>")
          .isApprox(Eigen::Matrix3d::Identity(), 1e-12));
}

//==============================================================================
TEST(SdfParser, PoseRotationFormatsRequireMatchingCounts)
{
  const std::string values[]
      = {"1 2 3 0 0", "1 2 3 0 0 0", "1 2 3 0 0 0 1", "1 2 3 0 0 0 1 8"};
  for (const auto* element : sdfPoseElements) {
    for (const auto* format : {"", "euler_rpy", "quat_xyzw"}) {
      for (std::size_t i = 0; i < 4; ++i) {
        SCOPED_TRACE(element);
        SCOPED_TRACE(format);
        SCOPED_TRACE(i + 5);
        const auto attributes
            = std::string(format).empty()
                  ? ""
                  : std::string("rotation_format='") + format + "'";
        const auto validCount = std::string(format) == "quat_xyzw" ? 7u : 6u;
        if (i + 5 == validCount) {
          EXPECT_NE(readSdfPose(element, attributes, values[i]), nullptr);
        } else {
          expectSdfPoseError(element, attributes, values[i]);
        }
      }
    }
  }
}

//==============================================================================
TEST(SdfParser, InvalidPoseAttributesReturnNull)
{
  for (const auto* element : sdfPoseElements) {
    SCOPED_TRACE(element);
    expectSdfPoseError(element, "rotation_format='quaternion'", "1 2 3 0 0 0");
    expectSdfPoseError(element, "rotation_format='quat_xyzw'", "1 2 3 0 0 0 0");
    expectSdfPoseError(
        element, "rotation_format='quat_xyzw' degrees='true'", "1 2 3 0 0 0 1");
    expectSdfPoseError(element, "degrees='sometimes'", "1 2 3 0 0 0");
    for (const auto* values : {"", " \t\n "}) {
      expectSdfPoseError(element, "rotation_format='quaternion'", values);
      expectSdfPoseError(element, "degrees='sometimes'", values);
      expectSdfPoseError(
          element, "rotation_format='quat_xyzw' degrees='true'", values);
      expectSdfPoseError(element, "relative_to='unknown_frame'", values);
    }
  }
}

//==============================================================================
TEST(SdfParser, RelativeToAcceptsDefaultAndExistingFrames)
{
  const char* frames[]
      = {"world", "__model__", "child", "child", "child", "child"};
  const char* otherFrames[]
      = {"world_origin",
         "parent",
         "parent",
         "__model__",
         "__model__",
         "__model__"};
  for (std::size_t i = 0; i < 6; ++i) {
    const auto* element = sdfPoseElements[i];
    SCOPED_TRACE(element);
    const auto reference = readSdfPose(element, "", "1 2 3 0.2 -0.3 0.4");
    ASSERT_NE(reference, nullptr);
    for (const auto* frame : {"", frames[i]}) {
      SCOPED_TRACE(frame);
      const auto skeleton = readSdfPose(
          element,
          std::string("relative_to='") + frame + "'",
          "1 2 3 0.2 -0.3 0.4");
      ASSERT_NE(skeleton, nullptr);
      EXPECT_TRUE(
          sdfPoseTransform(skeleton, element)
              .matrix()
              .isApprox(sdfPoseTransform(reference, element).matrix(), 1e-12));
    }
    if (std::string(element) != "inertial") {
      EXPECT_NE(
          readSdfPose(
              element,
              std::string("relative_to='") + otherFrames[i] + "'",
              "1 2 3 0.2 -0.3 0.4"),
          nullptr);
    }
    for (const auto* frame : {"unknown_frame"}) {
      SCOPED_TRACE(frame);
      expectSdfPoseError(
          element,
          std::string("relative_to='") + frame + "'",
          "1 2 3 0.2 -0.3 0.4");
    }
  }
}

//==============================================================================
TEST(SdfParser, AllDataSdfFilesLoad)
{
  std::size_t files = 0;
  for (const auto& entry :
       std::filesystem::recursive_directory_iterator(DART_DATA_PATH)) {
    const auto extension = entry.path().extension();
    if (extension != ".sdf" && extension != ".world" && extension != ".model")
      continue;
    SCOPED_TRACE(entry.path().string());
    ++files;
    sdf::Root document;
    document.Load(entry.path().string());
    if (document.WorldCount() > 0)
      EXPECT_NE(SdfParser::readWorld(entry.path().string()), nullptr);
    else
      EXPECT_NE(SdfParser::readSkeleton(entry.path().string()), nullptr);
  }
  EXPECT_GT(files, 0u);
}

//==============================================================================
TEST(SdfParser, IncludedRelativeMeshesKeepTheirSourceUri)
{
  for (const bool customRetriever : {false, true}) {
    SCOPED_TRACE(customRetriever);
    auto retriever = std::make_shared<SdfFixtureResourceRetriever>();
    SdfParser::Options options;
    if (customRetriever)
      options.mResourceRetriever = retriever;
    const auto world = SdfParser::readWorld(
        customRetriever ? "memory://pkg/include_relative_mesh.world"
                        : "dart://sample/sdf/test/include_relative_mesh/"
                          "include_relative_mesh.world",
        options);
    ASSERT_NE(world, nullptr);
    ASSERT_EQ(world->getNumSkeletons(), 1u);
    const auto* body = world->getSkeleton(0)->getBodyNode("mesh_body");
    ASSERT_NE(body, nullptr);
    ASSERT_EQ(body->getNumShapeNodes(), 2u);
    for (std::size_t i = 0; i < body->getNumShapeNodes(); ++i) {
      const auto mesh = std::dynamic_pointer_cast<const MeshShape>(
          body->getShapeNode(i)->getShape());
      ASSERT_NE(mesh, nullptr);
      ASSERT_NE(mesh->getTriMesh(), nullptr);
      EXPECT_GT(mesh->getTriMesh()->getVertices().size(), 0u);
    }
    if (customRetriever) {
      EXPECT_NE(
          std::find(
              retriever->requests.begin(),
              retriever->requests.end(),
              "memory://pkg/included_model/meshes/relative_box.obj"),
          retriever->requests.end());
    }
  }
}

//==============================================================================
TEST(SdfParser, LocalIncludedFilesAreValidatedBeforeConversion)
{
  TemporarySdfDirectory directory;
  directory.write("root.world", R"(<sdf version='1.9'><world name='test_world'>
    <include><uri>included.sdf</uri></include></world></sdf>)");
  for (const auto* quaternion : {"0 0 1e300 1e300", "0 0 1e-300 1e-300"}) {
    directory.write(
        "included.sdf",
        "<sdf version='1.9'><model name='included'>"
        "<pose rotation_format='quat_xyzw'>1 2 3 "
            + std::string(quaternion)
            + "</pose><link name='body'/></model></sdf>");
    const auto world
        = SdfParser::readWorld((directory.path / "root.world").string());
    ASSERT_NE(world, nullptr);
    const auto transform
        = world->getSkeleton(0)->getBodyNode(0)->getWorldTransform();
    EXPECT_TRUE(
        transform.translation().isApprox(Eigen::Vector3d(1, 2, 3), 1e-12));
    EXPECT_TRUE(
        transform.linear().col(0).isApprox(Eigen::Vector3d::UnitY(), 1e-12));
  }
  for (const auto* badPose : {"0 0 0 0 0", "0 0 nan 0 0 0"}) {
    directory.write(
        "included.sdf",
        "<sdf version='1.9'><model name='included'><pose>"
            + std::string(badPose)
            + "</pose><link name='body'/></model></sdf>");
    EXPECT_EQ(
        SdfParser::readWorld((directory.path / "root.world").string()),
        nullptr);
  }
}

//==============================================================================
TEST(SdfParser, ConcurrentLoadsResolveIncludesIndependently)
{
  TemporarySdfDirectory first;
  TemporarySdfDirectory second;
  for (const auto* directory : {&first, &second}) {
    directory->write("root.world", R"(<sdf version='1.9'><world name='isolated'>
      <include><uri>included.sdf</uri></include></world></sdf>)");
    directory->write(
        "included.sdf",
        "<sdf version='1.9'><model name='included'><link name='body'><pose>"
            + std::string(directory == &first ? "1" : "2")
            + " 0 0 0 0 0</pose></link></model></sdf>");
  }
  auto loadFirst = std::async(std::launch::async, [&] {
    return SdfParser::readWorld((first.path / "root.world").string());
  });
  auto loadSecond = std::async(std::launch::async, [&] {
    return SdfParser::readWorld((second.path / "root.world").string());
  });
  const auto firstWorld = loadFirst.get();
  const auto secondWorld = loadSecond.get();
  ASSERT_NE(firstWorld, nullptr);
  ASSERT_NE(secondWorld, nullptr);
  EXPECT_DOUBLE_EQ(
      firstWorld->getSkeleton(0)
          ->getBodyNode(0)
          ->getWorldTransform()
          .translation()
          .x(),
      1);
  EXPECT_DOUBLE_EQ(
      secondWorld->getSkeleton(0)
          ->getBodyNode(0)
          ->getWorldTransform()
          .translation()
          .x(),
      2);
}

//==============================================================================
TEST(SdfParser, StagedDocumentsAreRemovedAfterSuccessOrFailure)
{
  struct ObservedRetriever : SdfPoseResourceRetriever
  {
    std::string marker
        = "cleanup_"
          + std::to_string(
              std::chrono::steady_clock::now().time_since_epoch().count());
    std::string child;
    std::vector<std::filesystem::path> observed;

    std::string readAll(const common::Uri& uri) override
    {
      if (uri.toString() == "memory://cleanup/scene.world")
        return "<sdf version='1.9'><world name='" + marker
               + "'><include><uri>child.sdf</uri></include></world></sdf>";
      // The include callback runs while the containing document is staged.
      for (const auto& entry : std::filesystem::directory_iterator(
               std::filesystem::temp_directory_path())) {
        if (entry.path().filename().string().compare(0, 9, "dart-sdf-") != 0)
          continue;
        std::error_code error;
        for (const auto& file :
             std::filesystem::directory_iterator(entry.path(), error)) {
          if (!file.is_regular_file(error))
            continue;
          std::ifstream stream(file.path());
          const std::string contents(
              (std::istreambuf_iterator<char>(stream)),
              std::istreambuf_iterator<char>());
          if (contents.find(marker) != std::string::npos)
            observed.push_back(entry.path());
        }
      }
      return child;
    }
  };
  auto retriever = std::make_shared<ObservedRetriever>();
  SdfParser::Options options;
  options.mResourceRetriever = retriever;
  for (const bool valid : {true, false}) {
    SCOPED_TRACE(valid);
    retriever->observed.clear();
    retriever->child
        = "<sdf version='1.9'><model name='included'><link name='body'><pose>"
          + std::string(valid ? "0" : "nan")
          + " 0 0 0 0 0</pose></link></model></sdf>";
    const auto world
        = SdfParser::readWorld("memory://cleanup/scene.world", options);
    EXPECT_EQ(world != nullptr, valid);
    ASSERT_FALSE(retriever->observed.empty());
    for (const auto& directory : retriever->observed)
      EXPECT_FALSE(std::filesystem::exists(directory));
  }
}

//==============================================================================
TEST(SdfParser, MissingAndCyclicIncludesFailTheCompleteLoad)
{
  TemporarySdfDirectory directory;
  directory.write(
      "missing.world", R"(<sdf version='1.9'><world name='test_world'>
    <model name='valid'><link name='body'/></model>
    <include><uri>missing.sdf</uri></include></world></sdf>)");
  EXPECT_EQ(
      SdfParser::readWorld((directory.path / "missing.world").string()),
      nullptr);
  directory.write("cycle.sdf", R"(<sdf version='1.9'><model name='cycle'>
    <link name='body'/><include><uri>cycle.sdf</uri></include></model></sdf>)");
  EXPECT_EQ(
      SdfParser::readSkeleton((directory.path / "cycle.sdf").string()),
      nullptr);
}

//==============================================================================
TEST(SdfParser, MergedIncludeFlattensIntoTheContainingSkeleton)
{
  TemporarySdfDirectory directory;
  directory.write("included.sdf", R"(<sdf version='1.9'><model name='component'>
    <link name='included_body'><pose>0 2 0 0 0 0</pose></link></model></sdf>)");
  directory.write("root.sdf", R"(<sdf version='1.9'><model name='assembled'>
    <link name='base'/><include merge='true'><uri>included.sdf</uri>
      <pose>1 0 0 0 0 0</pose></include></model></sdf>)");
  const auto skeleton
      = SdfParser::readSkeleton((directory.path / "root.sdf").string());
  ASSERT_NE(skeleton, nullptr);
  ASSERT_EQ(skeleton->getNumBodyNodes(), 2u);
  const auto* body = skeleton->getBodyNode("included_body");
  ASSERT_NE(body, nullptr);
  EXPECT_TRUE(body->getWorldTransform().translation().isApprox(
      Eigen::Vector3d(1, 2, 0), 1e-12));
}

//==============================================================================
TEST(SdfParser, CustomRetrieverDoesNotFallBackToAmbientFiles)
{
  struct RootOnlyRetriever : PresentResourceRetriever
  {
    std::string rootUri;
    std::string xml;

    bool exists(const common::Uri& uri) override
    {
      return uri.toString() == rootUri;
    }

    std::string getFilePath(const common::Uri&) override
    {
      return "";
    }

    std::string readAll(const common::Uri& uri) override
    {
      return uri.toString() == rootUri ? xml : "";
    }
  };

  TemporarySdfDirectory directory;
  directory.write(
      "included.sdf",
      "<sdf version='1.9'><model name='ambient'><link name='body'/></model>"
      "</sdf>");
  const auto includedUri
      = common::Uri::createFromPath((directory.path / "included.sdf").string());

  auto retriever = std::make_shared<RootOnlyRetriever>();
  retriever->rootUri = "memory://custom/scene.world";
  retriever->xml = "<sdf version='1.9'><world name='custom'><include><uri>"
                   + includedUri.toString() + "</uri></include></world></sdf>";
  SdfParser::Options options;
  options.mResourceRetriever = retriever;

  EXPECT_EQ(SdfParser::readWorld(retriever->rootUri, options), nullptr);
}

//==============================================================================
TEST(SdfParser, ReadWorldSelectsTheFirstWorldAndValidatesTheDocument)
{
  const auto contents = R"(<sdf version='1.9'>
    <world name='first'><gravity>1 2 -3</gravity><model name='first_body'><link name='body'/></model></world>
    <world name='second'><gravity>4 5 -6</gravity><model name='second_body'><link name='body'/></model></world>
  </sdf>)";
  const auto world
      = SdfParser::readWorld("memory://worlds.sdf", sdfMemoryOptions(contents));
  ASSERT_NE(world, nullptr);
  EXPECT_EQ(world->getName(), "first");
  EXPECT_TRUE(world->getGravity().isApprox(Eigen::Vector3d(1, 2, -3), 1e-12));
  ASSERT_EQ(world->getNumSkeletons(), 1u);
  EXPECT_EQ(world->getSkeleton(0)->getName(), "first_body");
  const auto invalid = R"(<sdf version='1.9'>
    <world name='first'><model name='first_body'><link name='body'/></model></world>
    <world name='second'><model name='bad'><link name='body'><pose>nan 0 0 0 0 0</pose></link></model></world>
  </sdf>)";
  EXPECT_EQ(
      SdfParser::readWorld("memory://worlds.sdf", sdfMemoryOptions(invalid)),
      nullptr);
}

//==============================================================================
TEST(SdfParser, WorldUsesAuthoredGravityAndTimestep)
{
  const auto options
      = sdfMemoryOptions(R"(<sdf version='1.9'><world name='test_world'>
    <gravity>1 2 -3</gravity><physics name='first' type='ignored'>
      <max_step_size>0.0025</max_step_size></physics>
    <model name='body'><link name='link'/></model></world></sdf>)");
  const auto world = SdfParser::readWorld("memory://world.sdf", options);
  ASSERT_NE(world, nullptr);
  EXPECT_TRUE(world->getGravity().isApprox(Eigen::Vector3d(1, 2, -3), 1e-12));
  EXPECT_DOUBLE_EQ(world->getTimeStep(), 0.0025);
}

//==============================================================================
TEST(SdfParser, ScrewPitchUsesCanonicalUnits)
{
  // Legacy thread_pitch is radians/metre with the opposite handedness;
  // screw_thread_pitch and DART pitch are metres/revolution.
  for (const auto* pitch :
       {"<thread_pitch>-25.132741228718345</thread_pitch>",
        "<screw_thread_pitch>0.25</screw_thread_pitch>"}) {
    const auto options = sdfMemoryOptions(
        "<sdf version='1.9'><model name='screw'>"
        "<link name='base'/><link name='body'/><joint name='screw' "
        "type='screw'>"
        "<parent>base</parent><child>body</child><axis><xyz>0 0 1</xyz></axis>"
        + std::string(pitch) + "</joint></model></sdf>");
    const auto skeleton
        = SdfParser::readSkeleton("memory://screw.sdf", options);
    ASSERT_NE(skeleton, nullptr);
    const auto* joint
        = dynamic_cast<const ScrewJoint*>(skeleton->getJoint("screw"));
    ASSERT_NE(joint, nullptr);
    EXPECT_NEAR(joint->getPitch(), 0.25, 1e-12);
  }
}

//==============================================================================
TEST(SdfParser, MimicMapsFollowerAndReferenceAxesIndependently)
{
  const auto model = R"(<sdf version='1.9'><model name='mimic'>
    <link name='base'/><link name='middle'/><link name='body'/>
    <joint name='reference' type='universal'><parent>base</parent><child>middle</child>
      <axis><xyz>1 0 0</xyz></axis><axis2><xyz>0 1 0</xyz></axis2></joint>
    <joint name='follower' type='universal'><parent>middle</parent><child>body</child>
      <axis><xyz>1 0 0</xyz><mimic joint='reference' axis='axis2'>
        <multiplier>2</multiplier><offset>0.5</offset><reference>0.75</reference>
      </mimic></axis>
      <axis2><xyz>0 1 0</xyz><mimic joint='reference' axis='axis'>
        <multiplier>3</multiplier><offset>0.25</offset><reference>-0.5</reference>
      </mimic></axis2></joint>
  </model></sdf>)";
  const auto skeleton
      = SdfParser::readSkeleton("memory://mimic.sdf", sdfMemoryOptions(model));
  ASSERT_NE(skeleton, nullptr);
  const auto* reference = skeleton->getJoint("reference");
  const auto* follower = skeleton->getJoint("follower");
  ASSERT_NE(reference, nullptr);
  ASSERT_NE(follower, nullptr);
  EXPECT_EQ(follower->getActuatorType(), Joint::MIMIC);
  const auto& properties = follower->getMimicDofProperties();
  ASSERT_EQ(properties.size(), 2u);
  EXPECT_EQ(properties[0].mReferenceJoint, reference);
  EXPECT_EQ(properties[0].mReferenceDofIndex, 1u);
  EXPECT_DOUBLE_EQ(properties[0].mMultiplier, 2.0);
  EXPECT_DOUBLE_EQ(properties[0].mOffset, -1.0);
  EXPECT_EQ(properties[1].mReferenceJoint, reference);
  EXPECT_EQ(properties[1].mReferenceDofIndex, 0u);
  EXPECT_DOUBLE_EQ(properties[1].mMultiplier, 3.0);
  EXPECT_DOUBLE_EQ(properties[1].mOffset, 1.75);
}

//==============================================================================
TEST(SdfParser, OrdinaryJointAxesPreserveConstructorNormalization)
{
  for (const auto* version : {"1.4", "1.9"}) {
    for (const double sign : {-1.0, 1.0}) {
      SCOPED_TRACE(version);
      SCOPED_TRACE(sign);
      const auto options = sdfMemoryOptions(
          "<sdf version='" + std::string(version)
          + "'><model name='axis'><link name='body'/>"
            "<joint name='joint' type='revolute'><parent>world</parent>"
            "<child>body</child><axis><xyz>0 0.5 "
          + (sign < 0.0 ? "-0.866025" : "0.866025")
          + "</xyz></axis></joint></model></sdf>");
      const auto skeleton
          = SdfParser::readSkeleton("memory://axis.sdf", options);
      ASSERT_NE(skeleton, nullptr);
      const auto* joint
          = dynamic_cast<const RevoluteJoint*>(skeleton->getJoint("joint"));
      ASSERT_NE(joint, nullptr);
      const Eigen::Vector3d expected
          = Eigen::Vector3d(0.0, 0.5, sign * 0.866025).normalized();
      for (int index = 0; index < 3; ++index)
        EXPECT_EQ(joint->getAxis()[index], expected[index]);
    }
  }
}

//==============================================================================
TEST(SdfParser, ExtremeJointAxesRemainFiniteAndNormalized)
{
  struct AxisCase
  {
    const char* text;
    Eigen::Vector3d direction;
  };
  const AxisCase axes[]
      = {{"1e300 2e300 3e300", {1, 2, 3}},
         {"1e-300 2e-300 3e-300", {1, 2, 3}},
         {"1e-8 1e-8 1e-8", {1, 1, 1}},
         {"6e-7 6e-7 6e-7", {1, 1, 1}},
         {"7.741001517595157e153 7.741001517595157e153 7.741001517595157e153",
          {1, 1, 1}}};
  for (const auto& axis : axes) {
    SCOPED_TRACE(axis.text);
    const auto options = sdfMemoryOptions(
        "<sdf version='1.9'><model name='axis'><link name='body'/>"
        "<joint name='joint' type='revolute'><parent>world</parent>"
        "<child>body</child><axis><xyz>"
        + std::string(axis.text) + "</xyz></axis></joint></model></sdf>");
    const auto skeleton = SdfParser::readSkeleton("memory://axis.sdf", options);
    ASSERT_NE(skeleton, nullptr);
    const auto* joint
        = dynamic_cast<const RevoluteJoint*>(skeleton->getJoint("joint"));
    ASSERT_NE(joint, nullptr);
    EXPECT_TRUE(joint->getAxis().allFinite());
    EXPECT_TRUE(joint->getAxis().isApprox(axis.direction.normalized(), 1e-15));
  }
}

//==============================================================================
TEST(SdfParser, ExtremeUniversalAxesResolveInAuthoredFrames)
{
  const auto skeleton = readSdfUniversalAxes(
      "<axis><xyz expressed_in='__model__'>1e300 1e300 1e300</xyz></axis>",
      "<axis2><xyz expressed_in='__model__'>1e-300 -1e-300 0</xyz></axis2>",
      "1.9");
  ASSERT_NE(skeleton, nullptr);
  const auto* joint
      = dynamic_cast<const UniversalJoint*>(skeleton->getJoint("joint"));
  ASSERT_NE(joint, nullptr);
  const Eigen::Matrix3d modelToJoint
      = (eulerZYXToMatrix(Eigen::Vector3d(0.2, -0.3, 0.4))
         * eulerZYXToMatrix(Eigen::Vector3d(-0.4, 0.5, -0.2)))
            .transpose();
  EXPECT_TRUE(joint->getAxis1().allFinite());
  EXPECT_TRUE(joint->getAxis2().allFinite());
  EXPECT_NEAR(joint->getAxis1().norm(), 1.0, 1e-15);
  EXPECT_NEAR(joint->getAxis2().norm(), 1.0, 1e-15);
  EXPECT_TRUE(joint->getAxis1().isApprox(
      modelToJoint * Eigen::Vector3d::Ones().normalized(), 1e-12));
  EXPECT_TRUE(joint->getAxis2().isApprox(
      modelToJoint * Eigen::Vector3d(1, -1, 0).normalized(), 1e-12));
}

//==============================================================================
TEST(SdfParser, JointLimitsAndDynamicsUseTypedValues)
{
  const auto options
      = sdfMemoryOptions(R"(<sdf version='1.9'><model name='limits'>
    <link name='body'/><joint name='joint' type='revolute'><parent>world</parent>
      <child>body</child><axis><xyz>0 0 2</xyz>
        <limit><lower>-inf</lower><upper>inf</upper><effort>4</effort><velocity>5</velocity></limit>
        <dynamics><damping>0.2</damping><friction>0.3</friction>
          <spring_reference>0.4</spring_reference><spring_stiffness>0.5</spring_stiffness></dynamics>
      </axis></joint></model></sdf>)");
  const auto skeleton = SdfParser::readSkeleton("memory://limits.sdf", options);
  ASSERT_NE(skeleton, nullptr);
  const auto* joint = skeleton->getJoint("joint");
  ASSERT_NE(joint, nullptr);
  EXPECT_EQ(
      joint->getPositionLowerLimit(0),
      -std::numeric_limits<double>::infinity());
  EXPECT_EQ(
      joint->getPositionUpperLimit(0), std::numeric_limits<double>::infinity());
  EXPECT_DOUBLE_EQ(joint->getForceLowerLimit(0), -4);
  EXPECT_DOUBLE_EQ(joint->getForceUpperLimit(0), 4);
  EXPECT_DOUBLE_EQ(joint->getVelocityLowerLimit(0), -5);
  EXPECT_DOUBLE_EQ(joint->getVelocityUpperLimit(0), 5);
  EXPECT_DOUBLE_EQ(joint->getDampingCoefficient(0), 0.2);
  EXPECT_DOUBLE_EQ(joint->getCoulombFriction(0), 0.3);
  EXPECT_DOUBLE_EQ(joint->getRestPosition(0), 0.4);
  EXPECT_DOUBLE_EQ(joint->getSpringStiffness(0), 0.5);
}

//==============================================================================
TEST(SdfParser, InitialPositionWithinLimitsPreservesAuthoredSpringReference)
{
  const auto options
      = sdfMemoryOptions(R"(<sdf version='1.9'><model name='spring'>
    <link name='body'/><joint name='joint' type='revolute'><parent>world</parent><child>body</child>
      <axis><xyz>0 0 1</xyz><limit><lower>1</lower><upper>3</upper></limit>
        <dynamics><spring_reference>0.4</spring_reference></dynamics></axis>
    </joint></model></sdf>)");
  const auto skeleton = SdfParser::readSkeleton("memory://spring.sdf", options);
  ASSERT_NE(skeleton, nullptr);
  const auto* joint = skeleton->getJoint("joint");
  ASSERT_NE(joint, nullptr);
  EXPECT_DOUBLE_EQ(joint->getPosition(0), 2);
  EXPECT_DOUBLE_EQ(joint->getRestPosition(0), 0.4);
}

//==============================================================================
TEST(SdfParser, InvalidGeometryAndWorldNumbersReturnNull)
{
  for (const auto* shape :
       {"<box><size>1 0 1</size></box>",
        "<sphere><radius>-1</radius></sphere>",
        "<cylinder><radius>1</radius><length>nan</length></cylinder>",
        "<plane><normal>0 0 0</normal><size>1 1</size></plane>",
        "<heightmap><uri>missing.png</uri></heightmap>"}) {
    const auto options = sdfMemoryOptions(
        "<sdf version='1.9'><model name='invalid'><link name='body'>"
        "<collision name='shape'><geometry>"
        + std::string(shape) + "</geometry></collision></link></model></sdf>");
    EXPECT_EQ(
        SdfParser::readSkeleton("memory://geometry.sdf", options), nullptr);
  }
  for (const auto* step : {"0", "-1", "nan", "inf", "1junk"}) {
    const auto options = sdfMemoryOptions(
        "<sdf version='1.9'><world name='invalid'>"
        "<physics name='physics' type='ignored'><max_step_size>"
        + std::string(step) + "</max_step_size></physics></world></sdf>");
    EXPECT_EQ(SdfParser::readWorld("memory://world.sdf", options), nullptr);
  }
}

//==============================================================================
TEST(SdfParser, InvalidJointParametersAndTopologiesReturnNull)
{
  for (const auto* axis :
       {"<xyz>0 0 0</xyz>",
        "<xyz>0 0 nan</xyz>",
        "<xyz>0 0 1</xyz><dynamics><damping>-1</damping></dynamics>",
        "<xyz>0 0 1</xyz><dynamics><friction>nan</friction></dynamics>",
        "<xyz>0 0 1</xyz><limit><lower>2</lower><upper>1</upper></limit>",
        "<xyz>0 0 1</xyz><limit><effort>nan</effort></limit>",
        "<xyz>0 0 1</xyz><mimic joint='missing'/>",
        "<xyz>0 0 1</xyz><mimic joint='joint' axis='axis2'/>",
        "<xyz>0 0 1</xyz><mimic joint='joint' axis='unknown'/>"}) {
    const auto options = sdfMemoryOptions(
        "<sdf version='1.9'><model name='invalid'><link name='body'/>"
        "<joint name='joint' "
        "type='revolute'><parent>world</parent><child>body</child><axis>"
        + std::string(axis) + "</axis></joint></model></sdf>");
    EXPECT_EQ(SdfParser::readSkeleton("memory://joint.sdf", options), nullptr);
  }
  for (const auto* joints :
       {"<joint name='self' "
        "type='fixed'><parent>a</parent><child>a</child></joint>",
        "<joint name='first' "
        "type='fixed'><parent>a</parent><child>b</child></joint><joint "
        "name='second' type='fixed'><parent>b</parent><child>a</child></joint>",
        "<joint name='first' "
        "type='fixed'><parent>world</parent><child>a</child></joint><joint "
        "name='second' "
        "type='fixed'><parent>b</parent><child>a</child></joint>"}) {
    const auto options = sdfMemoryOptions(
        "<sdf version='1.9'><model name='invalid'><link name='a'/><link "
        "name='b'/>"
        + std::string(joints) + "</model></sdf>");
    EXPECT_EQ(SdfParser::readSkeleton("memory://tree.sdf", options), nullptr);
  }
}

//==============================================================================
TEST(SdfParser, ExcessiveSoftShapeSubdivisionsReturnNull)
{
  const auto options
      = sdfMemoryOptions(R"(<sdf version='1.9'><model name='oversized'>
    <link name='body'><soft_shape><total_mass>1</total_mass><geometry><sphere>
      <radius>1</radius><num_slices>2147483647</num_slices><num_stacks>3</num_stacks>
    </sphere></geometry></soft_shape></link></model></sdf>)");
  EXPECT_EQ(
      SdfParser::readSkeleton("memory://oversized.sdf", options), nullptr);
}

//==============================================================================
TEST(SdfParser, ConsumedNumericValuesRejectSuffixesAndNonfiniteTokens)
{
  for (const auto* value : {"1garbage", "nan", "inf"}) {
    SCOPED_TRACE(value);
    for (const auto* field : {"mass", "ixx", "diffuse"}) {
      SCOPED_TRACE(field);
      const std::string contents = std::string(field) == "mass"
          ? "<inertial><mass>" + std::string(value) + "</mass></inertial>"
          : std::string(field) == "ixx"
              ? "<inertial><inertia><ixx>" + std::string(value)
                + "</ixx><iyy>2</iyy><izz>3</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>"
              : "<visual name='shape'><geometry><box><size>1 1 1</size></box></geometry><material><diffuse>"
                + std::string(value) + " 0.2 0.3 1</diffuse></material></visual>";
      const auto options = sdfMemoryOptions(
          "<sdf version='1.9'><model name='invalid'><link name='body'>"
          + contents + "</link></model></sdf>");
      EXPECT_EQ(
          SdfParser::readSkeleton("memory://numeric.sdf", options), nullptr);
    }
    for (const auto* field : {"damping", "upper", "screw_thread_pitch"}) {
      SCOPED_TRACE(field);
      const std::string contents
          = std::string(field) == "damping"
                ? "<axis><xyz>0 0 1</xyz><dynamics><damping>"
                      + std::string(value) + "</damping></dynamics></axis>"
            : std::string(field) == "upper"
                ? "<axis><xyz>0 0 1</xyz><limit><upper>" + std::string(value)
                      + "</upper></limit></axis>"
                : "<axis><xyz>0 0 1</xyz></axis><screw_thread_pitch>"
                      + std::string(value) + "</screw_thread_pitch>";
      // Infinite position limits are valid; the complete token must still
      // parse.
      if (std::string(field) == "upper" && std::string(value) == "inf")
        continue;
      const auto options = sdfMemoryOptions(
          "<sdf version='1.9'><model name='invalid'><link name='body'/>"
          "<joint name='joint' "
          "type='screw'><parent>world</parent><child>body</child>"
          + contents + "</joint></model></sdf>");
      EXPECT_EQ(
          SdfParser::readSkeleton("memory://numeric.sdf", options), nullptr);
    }
  }
}

//==============================================================================
TEST(SdfParser, WorldJointRejectsUnsupportedCrossModelTopology)
{
  const auto options
      = sdfMemoryOptions(R"(<sdf version='1.9'><world name='cross_model'>
    <model name='first'><link name='body'/></model>
    <model name='second'><link name='body'/></model>
    <joint name='cross_model' type='fixed'><parent>first::body</parent><child>second::body</child></joint>
  </world></sdf>)");
  EXPECT_EQ(SdfParser::readWorld("memory://world-joint.sdf", options), nullptr);
}

//==============================================================================
TEST(SdfParser, AuthoredRootJointNameRemainsTheMimicReference)
{
  const auto options
      = sdfMemoryOptions(R"(<sdf version='1.9'><model name='named_root'>
    <link name='base'/><link name='middle'/><link name='body'/>
    <joint name='root' type='revolute'><parent>base</parent><child>middle</child>
      <axis><xyz>0 0 1</xyz></axis></joint>
    <joint name='follower' type='revolute'><parent>middle</parent><child>body</child>
      <axis><xyz>0 0 1</xyz><mimic joint='root'/></axis></joint></model></sdf>)");
  const auto skeleton
      = SdfParser::readSkeleton("memory://named-root.sdf", options);
  ASSERT_NE(skeleton, nullptr);
  const auto* reference = skeleton->getJoint("root");
  ASSERT_NE(dynamic_cast<const RevoluteJoint*>(reference), nullptr);
  const auto* follower = skeleton->getJoint("follower");
  ASSERT_NE(follower, nullptr);
  EXPECT_EQ(follower->getMimicJoint(), reference);
}

//==============================================================================
TEST(SdfParser, MeshScalePreservesMirroring)
{
  TemporarySdfDirectory directory;
  const auto meshUri = common::Uri::createFromPath(std::string(DART_DATA_PATH)
      + "/sdf/test/include_relative_mesh/included_model/meshes/relative_box.obj").toString();
  directory.write("mirror.sdf", "<sdf version='1.9'><model name='mirrored'><link name='body'>"
      "<visual name='mesh'><geometry><mesh><uri>" + meshUri + "</uri>"
      "<scale>-1 1 1</scale></mesh></geometry></visual></link></model></sdf>");
  const auto skeleton
      = SdfParser::readSkeleton((directory.path / "mirror.sdf").string());
  ASSERT_NE(skeleton, nullptr);
  ASSERT_EQ(skeleton->getBodyNode(0)->getNumShapeNodes(), 1u);
  const auto mesh = std::dynamic_pointer_cast<const MeshShape>(
      skeleton->getBodyNode(0)->getShapeNode(0)->getShape());
  ASSERT_NE(mesh, nullptr);
  EXPECT_TRUE(mesh->getScale().isApprox(Eigen::Vector3d(-1, 1, 1), 1e-12));
  EXPECT_TRUE(mesh->getBoundingBox().computeFullExtents().isApprox(
      Eigen::Vector3d::Ones(), 1e-12));
  ASSERT_NE(mesh->getTriMesh(), nullptr);
  EXPECT_FALSE(mesh->getTriMesh()->getTriangles().empty());
}

//==============================================================================
TEST(SdfParser, MeshSubmeshSelectionReturnsNull)
{
  TemporarySdfDirectory directory;
  const auto meshUri = common::Uri::createFromPath(std::string(DART_DATA_PATH)
      + "/sdf/test/include_relative_mesh/included_model/meshes/relative_box.obj").toString();
  directory.write("submesh.sdf", "<sdf version='1.9'><model name='selected'><link name='body'>"
      "<visual name='mesh'><geometry><mesh><uri>" + meshUri + "</uri>"
      "<submesh><name>partial</name><center>false</center></submesh>"
      "</mesh></geometry></visual></link></model></sdf>");
  EXPECT_EQ(
      SdfParser::readSkeleton((directory.path / "submesh.sdf").string()),
      nullptr);
}

//==============================================================================
TEST(SdfParser, InvalidMimicReferencesFailTheCompleteWorld)
{
  for (const auto* reference :
       {"joint='fixed'",
        "joint='reference' axis='axis2'",
        "joint='follower'"}) {
    const std::string model
        = "<model name='invalid'><link name='base'/><link name='middle'/><link "
          "name='body'/>"
          "<joint name='fixed' "
          "type='fixed'><parent>world</parent><child>base</child></joint>"
          "<joint name='reference' "
          "type='revolute'><parent>base</parent><child>middle</"
          "child><axis><xyz>0 0 1</xyz></axis></joint>"
          "<joint name='follower' "
          "type='revolute'><parent>middle</parent><child>body</"
          "child><axis><xyz>0 0 1</xyz><mimic "
          + std::string(reference) + "/></axis></joint></model>";
    const auto options = sdfMemoryOptions(
        "<sdf version='1.9'><world name='atomic'><model name='valid'><link "
        "name='body'/></model>"
        + model + "</world></sdf>");
    EXPECT_EQ(SdfParser::readWorld("memory://mimic.world", options), nullptr);
  }
  const auto options
      = sdfMemoryOptions(R"(<sdf version='1.9'><model name='cycle'>
    <link name='base'/><link name='middle'/><link name='body'/>
    <joint name='first' type='revolute'><parent>base</parent><child>middle</child>
      <axis><xyz>0 0 1</xyz><mimic joint='second'/></axis></joint>
    <joint name='second' type='revolute'><parent>middle</parent><child>body</child>
      <axis><xyz>0 0 1</xyz><mimic joint='first'/></axis></joint></model></sdf>)");
  EXPECT_EQ(SdfParser::readSkeleton("memory://mimic.sdf", options), nullptr);
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

  for (auto& joint : skel->getJoints()) {
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
  }
}

//==============================================================================
TEST(SdfParser, ParsingSDFFiles)
{
  const auto numSteps = 10u;

  // Create a list of sdf files to test with where the sdf files contains World
  std::vector<std::string> worldFiles;
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
// Regression: an SDF <soft_shape> link must load as a SoftBodyNode. Before
// SdfParser forwarded the NodeType template argument to
// createJointAndBodyNodePair, every soft SDF body loaded as a rigid BodyNode
// (SoftBodyNode::Properties sliced away), so the soft-feet Atlas reported zero
// soft bodies.
TEST(SdfParser, SoftShapeLinkLoadsAsSoftBodyNode)
{
  auto softAtlas = SdfParser::readSkeleton(
      "dart://sample/sdf/atlas/atlas_v3_no_head_soft_feet.sdf");
  ASSERT_NE(nullptr, softAtlas);
  EXPECT_EQ(2u, softAtlas->getNumSoftBodyNodes());
  EXPECT_NE(nullptr, softAtlas->getSoftBodyNode("l_foot"));
  EXPECT_NE(nullptr, softAtlas->getSoftBodyNode("r_foot"));
  ASSERT_NE(nullptr, softAtlas->getBodyNode("l_foot"));
  EXPECT_NE(nullptr, softAtlas->getBodyNode("l_foot")->asSoftBodyNode());

  // Control: the rigid variant of the same model stays fully rigid.
  auto rigidAtlas
      = SdfParser::readSkeleton("dart://sample/sdf/atlas/atlas_v3_no_head.sdf");
  ASSERT_NE(nullptr, rigidAtlas);
  EXPECT_EQ(0u, rigidAtlas->getNumSoftBodyNodes());
}

//==============================================================================
TEST(SdfParser, PlaneShapeOption)
{
  const std::string sdfFilename = "dart://sample/sdf/test/plane_shape.sdf";

  SkeletonPtr legacySkeleton = SdfParser::readSkeleton(sdfFilename);
  ASSERT_TRUE(legacySkeleton != nullptr);
  ASSERT_EQ(legacySkeleton->getNumBodyNodes(), 1u);
  auto* legacyBody = legacySkeleton->getBodyNode(0);
  ASSERT_TRUE(legacyBody != nullptr);
  ASSERT_EQ(legacyBody->getNumShapeNodes(), 1u);
  EXPECT_TRUE(
      std::dynamic_pointer_cast<BoxShape>(
          legacyBody->getShapeNode(0)->getShape())
      != nullptr);

  SdfParser::Options options;
  options.mUsePlaneShapeForPlane = true;
  SkeletonPtr planeSkeleton = SdfParser::readSkeleton(sdfFilename, options);
  ASSERT_TRUE(planeSkeleton != nullptr);
  ASSERT_EQ(planeSkeleton->getNumBodyNodes(), 1u);
  auto* planeBody = planeSkeleton->getBodyNode(0);
  ASSERT_TRUE(planeBody != nullptr);
  ASSERT_EQ(planeBody->getNumShapeNodes(), 1u);

  const auto plane = std::dynamic_pointer_cast<PlaneShape>(
      planeBody->getShapeNode(0)->getShape());
  ASSERT_TRUE(plane != nullptr);
  EXPECT_TRUE(plane->getNormal().isApprox(Eigen::Vector3d::UnitZ()));
}

//==============================================================================
TEST(SdfParser, PlaneShapeBulletWorldSettles)
{
#if HAVE_BULLET
  SdfParser::Options options;
  options.mUsePlaneShapeForPlane = true;
  options.mCollisionDetector = collision::BulletCollisionDetector::create();

  WorldPtr world = SdfParser::readWorld(
      "dart://sample/sdf/test/plane_box_world.sdf", options);
  ASSERT_TRUE(world != nullptr);

  auto deactivation = world->getDeactivationOptions();
  deactivation.mEnabled = true;
  world->setDeactivationOptions(deactivation);
  world->getConstraintSolver()->getCollisionOption().maxNumContacts = 32u;

  SkeletonPtr ground = world->getSkeleton("ground");
  SkeletonPtr box = world->getSkeleton("box");
  ASSERT_TRUE(ground != nullptr);
  ASSERT_TRUE(box != nullptr);
  EXPECT_FALSE(ground->isMobile());
  EXPECT_TRUE(box->isMobile());

  ASSERT_EQ(ground->getNumBodyNodes(), 1u);
  auto* groundBody = ground->getBodyNode(0);
  ASSERT_TRUE(groundBody != nullptr);
  ASSERT_EQ(groundBody->getNumShapeNodes(), 1u);
  EXPECT_TRUE(
      std::dynamic_pointer_cast<PlaneShape>(
          groundBody->getShapeNode(0)->getShape())
      != nullptr);

  ASSERT_EQ(box->getNumBodyNodes(), 1u);
  auto* boxBody = box->getBodyNode(0);
  ASSERT_TRUE(boxBody != nullptr);

  bool sawContact = false;
  std::size_t steps = 0;
  for (; steps < 5000 && !box->isResting(); ++steps) {
    world->step();
    sawContact
        = sawContact || world->getLastCollisionResult().getNumContacts() > 0;
  }

  EXPECT_TRUE(sawContact) << "box never contacted the parsed PlaneShape";
  ASSERT_LT(steps, 5000u) << "box never went to sleep on the parsed plane";
  ASSERT_TRUE(box->isResting());

  const double z = boxBody->getTransform().translation().z();
  EXPECT_NEAR(z, 0.1, 2e-2);

  const Eigen::VectorXd frozenPositions = box->getPositions();
  for (std::size_t i = 0; i < 200; ++i) {
    world->step();
    ASSERT_TRUE(box->isResting()) << "box spuriously woke at step " << i;
    EXPECT_TRUE(box->getPositions().isApprox(frozenPositions, 1e-12))
        << "resting box drifted on parsed PlaneShape at step " << i;
  }
#else
  GTEST_SKIP() << "Bullet is unavailable";
#endif
}

//==============================================================================
TEST(SdfParser, ReadMaterial)
{
  std::string sdf_filename = "dart://sample/sdf/quad.sdf";
  SkeletonPtr skeleton = SdfParser::readSkeleton(sdf_filename);
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
