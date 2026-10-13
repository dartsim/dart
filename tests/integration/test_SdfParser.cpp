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
#include "dart/common/Uri.hpp"
#include "dart/config.hpp"
#include "dart/constraint/ConstraintSolver.hpp"
#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/BoxShape.hpp"
#include "dart/dynamics/FreeJoint.hpp"
#include "dart/dynamics/Joint.hpp"
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
#include <tinyxml2.h>

#include <filesystem>
#include <initializer_list>
#include <iostream>
#include <memory>
#include <stdexcept>
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
  retriever->xml = "<sdf version='" + version + "'><model name='pose_model'>"
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
                    + "</joint></model></sdf>";
  SdfParser::Options options;
  options.mResourceRetriever = retriever;
  return SdfParser::readSkeleton("memory://pose.sdf", options);
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
    const std::string& values,
    std::initializer_list<const char*> diagnostics)
{
  try {
    readSdfPose(element, attributes, values);
    FAIL() << "Expected a pose parse error";
  } catch (const std::invalid_argument& error) {
    const std::string message = error.what();
    for (const auto* diagnostic : diagnostics)
      EXPECT_NE(message.find(diagnostic), std::string::npos) << message;
  }
}

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
        = "<sdf version='1.6'><model name='inertia'><link name='body'>"
          "<inertial>"
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
          = "<sdf version='1.6'><model name='soft'><link name='body'>"
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
    const auto reference = readSdfPose(element, "", "0 0 0 0 0 0", "1.6");
    ASSERT_NE(reference, nullptr);
    for (const auto* attributes :
         {"",
          "rotation_format='euler_rpy'",
          "degrees='true'",
          "rotation_format='quat_xyzw'"}) {
      SCOPED_TRACE(attributes);
      for (const auto* values : {"", " \t\n "}) {
        const auto skeleton = readSdfPose(element, attributes, values, "1.6");
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
                     + "'><world name='world'><model name='model'>"
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
       {"", "unknown", "1.3", "1.10", "2.0", "1.70", "1.7.1", "1.7junk"}) {
    SCOPED_TRACE(version);
    EXPECT_EQ(readSdfPose("model", "", "0 0 0 0 0 0", version), nullptr);
    auto retriever = std::make_shared<SdfPoseResourceRetriever>();
    retriever->xml = "<sdf version='" + std::string(version)
                     + "'><world name='world'/></sdf>";
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
      = {{"", "<use_parent_model_frame>true</use_parent_model_frame>", true},
         {"expressed_in='__model__'", "", true},
         {"expressed_in=''", "", false},
         {"expressed_in='  '", "", false},
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
TEST(SdfParser, UniversalAxesRejectUnsupportedAndConflictingFrames)
{
  for (const auto* version : {"1.7", "1.8", "1.9"}) {
    SCOPED_TRACE(version);
    for (const auto* frame : {"other_link", "world"}) {
      SCOPED_TRACE(frame);
      const std::string xyz
          = "<xyz expressed_in='" + std::string(frame) + "'>1 0 0</xyz>";
      EXPECT_THROW(
          readSdfUniversalAxes(
              "<axis>" + xyz + "</axis>",
              "<axis2><xyz>0 1 0</xyz></axis2>",
              version),
          std::invalid_argument);
      EXPECT_THROW(
          readSdfUniversalAxes(
              "<axis><xyz>1 0 0</xyz></axis>",
              "<axis2>" + xyz + "</axis2>",
              version),
          std::invalid_argument);
    }
    for (const auto* legacy : {"true", "false"}) {
      SCOPED_TRACE(legacy);
      const std::string xyz
          = "<xyz expressed_in='__model__'>1 0 0</xyz>"
            "<use_parent_model_frame>"
            + std::string(legacy) + "</use_parent_model_frame>";
      EXPECT_THROW(
          readSdfUniversalAxes(
              "<axis>" + xyz + "</axis>",
              "<axis2><xyz>0 1 0</xyz></axis2>",
              version),
          std::invalid_argument);
      EXPECT_THROW(
          readSdfUniversalAxes(
              "<axis><xyz>1 0 0</xyz></axis>",
              "<axis2>" + xyz + "</axis2>",
              version),
          std::invalid_argument);
    }
  }
}

//==============================================================================
TEST(SdfParser, ModernModelsRejectUnsupportedFrameComposition)
{
  for (const auto* version : {"1.7", "1.8", "1.9"}) {
    SCOPED_TRACE(version);
    for (const auto* content :
         {"<frame name='named'/>",
          "<include><uri>memory://included.sdf</uri></include>",
          "<model name='nested'><link name='nested_body'/></model>"}) {
      SCOPED_TRACE(content);
      const std::string model = "<model name='model'><link name='body'/>"
                                + std::string(content) + "</model>";
      auto options = sdfMemoryOptions(
          "<sdf version='" + std::string(version) + "'>" + model + "</sdf>");
      EXPECT_THROW(
          SdfParser::readSkeleton("memory://frames.sdf", options),
          std::invalid_argument);
      options = sdfMemoryOptions(
          "<sdf version='" + std::string(version) + "'><world name='world'>"
          + model + "</world></sdf>");
      EXPECT_THROW(
          SdfParser::readWorld("memory://frames.sdf", options),
          std::invalid_argument);
    }
    for (const auto* frame : {"", "__model__", " __model__ ", "other_link"}) {
      SCOPED_TRACE(frame);
      const std::string model = "<model name='model' placement_frame='"
                                + std::string(frame)
                                + "'><link name='body'/></model>";
      const auto options = sdfMemoryOptions(
          "<sdf version='" + std::string(version) + "'>" + model + "</sdf>");
      const auto worldOptions = sdfMemoryOptions(
          "<sdf version='" + std::string(version) + "'><world name='world'>"
          + model + "</world></sdf>");
      if (std::string(frame) == "other_link") {
        EXPECT_THROW(
            SdfParser::readSkeleton("memory://frames.sdf", options),
            std::invalid_argument);
        EXPECT_THROW(
            SdfParser::readWorld("memory://frames.sdf", worldOptions),
            std::invalid_argument);
      } else {
        EXPECT_NE(
            SdfParser::readSkeleton("memory://frames.sdf", options), nullptr);
        EXPECT_NE(
            SdfParser::readWorld("memory://frames.sdf", worldOptions), nullptr);
      }
    }
  }
}

//==============================================================================
TEST(SdfParser, ModernWorldsRejectUnsupportedFrameComposition)
{
  for (const auto* version : {"1.7", "1.8", "1.9"}) {
    SCOPED_TRACE(version);
    for (const auto* content :
         {"<frame name='named'/>",
          "<include><uri>memory://included.sdf</uri></include>"}) {
      SCOPED_TRACE(content);
      const auto options = sdfMemoryOptions(
          "<sdf version='" + std::string(version) + "'><world name='world'>"
          + content + "<model name='model'><link name='body'/></model>"
                      "</world></sdf>");
      EXPECT_THROW(
          SdfParser::readWorld("memory://frames.sdf", options),
          std::invalid_argument);
    }
  }
}

//==============================================================================
TEST(SdfParser, LegacyFrameCompositionRemainsAccepted)
{
  const std::string model
      = "<model name='model' placement_frame='other_link'>"
        "<link name='body'/><frame name='named'/>"
        "<include><uri>memory://included.sdf</uri></include>"
        "<model name='nested'><link name='nested_body'/></model></model>";
  for (const auto* version : {"1.4", "1.5", "1.6"}) {
    SCOPED_TRACE(version);
    const auto options = sdfMemoryOptions(
        "<sdf version='" + std::string(version) + "'>" + model + "</sdf>");
    const auto skeleton
        = SdfParser::readSkeleton("memory://frames.sdf", options);
    ASSERT_NE(skeleton, nullptr);
    EXPECT_EQ(skeleton->getNumBodyNodes(), 1u);
    const auto worldOptions = sdfMemoryOptions(
        "<sdf version='" + std::string(version) + "'><world name='world'>"
        "<frame name='world_frame'/><include><uri>memory://included.sdf</uri>"
        "</include>" + model + "</world></sdf>");
    const auto world
        = SdfParser::readWorld("memory://frames.sdf", worldOptions);
    ASSERT_NE(world, nullptr);
    EXPECT_EQ(world->getNumSkeletons(), 1u);
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
            "<sdf version='" + std::string(version) + "'><world name='world'>"
            + model + "</world></sdf>");
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
          EXPECT_THROW(
              SdfParser::readSkeleton("memory://links.sdf", options),
              std::invalid_argument);
          EXPECT_THROW(
              SdfParser::readWorld("memory://links.sdf", worldOptions),
              std::invalid_argument);
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
          expectSdfPoseError(
              element, attributes, values[i], {"pose", "rotation_format"});
        }
      }
    }
  }
}

//==============================================================================
TEST(SdfParser, InvalidPoseAttributesThrow)
{
  for (const auto* element : sdfPoseElements) {
    SCOPED_TRACE(element);
    expectSdfPoseError(
        element,
        "rotation_format='quaternion'",
        "1 2 3 0 0 0",
        {"pose", "rotation_format", "quaternion"});
    expectSdfPoseError(
        element,
        "rotation_format='quat_xyzw'",
        "1 2 3 0 0 0 0",
        {"pose", "rotation_format", "zero"});
    expectSdfPoseError(
        element,
        "rotation_format='quat_xyzw' degrees='true'",
        "1 2 3 0 0 0 1",
        {"pose", "degrees", "quat_xyzw"});
    expectSdfPoseError(
        element, "degrees='sometimes'", "1 2 3 0 0 0", {"pose", "degrees"});
    for (const auto* values : {"", " \t\n "}) {
      expectSdfPoseError(
          element,
          "rotation_format='quaternion'",
          values,
          {"pose", "rotation_format", "quaternion"});
      expectSdfPoseError(
          element, "degrees='sometimes'", values, {"pose", "degrees"});
      expectSdfPoseError(
          element,
          "rotation_format='quat_xyzw' degrees='true'",
          values,
          {"pose", "degrees", "quat_xyzw"});
      expectSdfPoseError(
          element,
          "relative_to='unknown_frame'",
          values,
          {"pose", "relative_to", "unknown_frame"});
    }
  }
}

//==============================================================================
TEST(SdfParser, RelativeToAcceptsOnlyDefaultFrame)
{
  const char* frames[]
      = {"world", "__model__", "child", "child", "child", "child"};
  const char* otherFrames[] = {
      "child", "pose_model", "parent", "__model__", "__model__", "__model__"};
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
    for (const auto* frame : {"unknown_frame", otherFrames[i]}) {
      SCOPED_TRACE(frame);
      expectSdfPoseError(
          element,
          std::string("relative_to='") + frame + "'",
          "1 2 3 0.2 -0.3 0.4",
          {"pose",
           "relative_to",
           "does not support SDF frame semantics",
           frame});
    }
  }
}

//==============================================================================
TEST(SdfParser, AllDataSdfFilesLoadAndReportPoseAttributes)
{
  std::size_t files = 0;
  std::size_t attributes = 0;
  for (const auto& entry :
       std::filesystem::recursive_directory_iterator(DART_DATA_PATH)) {
    const auto extension = entry.path().extension();
    if (extension != ".sdf" && extension != ".world" && extension != ".model")
      continue;
    SCOPED_TRACE(entry.path().string());
    ++files;
    tinyxml2::XMLDocument document;
    ASSERT_EQ(
        document.LoadFile(entry.path().string().c_str()),
        tinyxml2::XML_SUCCESS);
    const auto* sdf = document.FirstChildElement("sdf");
    ASSERT_NE(sdf, nullptr);
    std::vector<const tinyxml2::XMLElement*> pending{sdf};
    while (!pending.empty()) {
      const auto* element = pending.back();
      pending.pop_back();
      if (std::string(element->Name()) == "pose") {
        for (const auto* attribute :
             {"rotation_format", "degrees", "relative_to"}) {
          if (const auto* value = element->Attribute(attribute)) {
            ++attributes;
            std::cout
                << entry.path().lexically_relative(DART_DATA_PATH).string()
                << ": <pose> " << attribute << "=\"" << value << "\"\n";
          }
        }
      }
      for (const auto* child = element->FirstChildElement(); child;
           child = child->NextSiblingElement())
        pending.push_back(child);
    }
    if (sdf->FirstChildElement("world")) {
      EXPECT_NE(SdfParser::readWorld(entry.path().string()), nullptr);
    } else {
      ASSERT_NE(sdf->FirstChildElement("model"), nullptr);
      EXPECT_NE(SdfParser::readSkeleton(entry.path().string()), nullptr);
    }
  }
  EXPECT_GT(files, 0u);
  std::cout << "Scanned " << files << " data SDF/world/model files; found "
            << attributes << " pose attributes.\n";
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
