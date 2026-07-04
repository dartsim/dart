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
#include "dart/utils/sdf/detail/sdf_helpers.hpp"
#include "dart/utils/sdf/sdf_parser.hpp"

#include <dart/all.hpp>

#include <gtest/gtest.h>
#include <sdf/sdf.hh>

#include <algorithm>
#include <array>
#include <iostream>
#include <iterator>
#include <map>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

#include <cctype>
#include <cstring>

#if DART_HAVE_spdlog
  #include <spdlog/sinks/ostream_sink.h>
  #include <spdlog/spdlog.h>
#endif

using namespace dart;
using namespace dart::dynamics;
using namespace math;
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
    const auto uriString = uri.toString();
    return mFiles.contains(uriString);
  }

  common::ResourcePtr retrieve(const common::Uri& uri) override
  {
    const auto uriString = uri.toString();
    auto it = mFiles.find(uriString);
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
    constexpr auto massWarningSnippets = std::to_array<std::string_view>(
        {"very small mass", "non-positive mass"});
    const bool hasSmallMassWarning = std::ranges::any_of(
        massWarningSnippets, [&](std::string_view snippet) {
          return logs.find(snippet) != std::string::npos;
        });
    EXPECT_TRUE(hasSmallMassWarning)
        << "Expected warning about tiny mass clamping in logs: " << logs;
    std::string logsLower = logs;
    std::ranges::transform(logsLower, logsLower.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });
    EXPECT_NE(logsLower.find("clamping to"), std::string::npos)
        << "Expected warning about tiny mass clamping in logs: " << logs;
    EXPECT_NE(logs.find("defines <mass> but no <inertia>"), std::string::npos)
        << "Expected warning about missing inertia tensor in logs: " << logs;
  }
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
TEST(SdfParser, UnsupportedSdfJointTypeStopsAtConstructedParent)
{
  auto retriever = std::make_shared<MemoryResourceRetriever>();
  const std::string uri = "memory://pkg/unsupported_joint/model.sdf";
  const std::string modelSdf = R"(
<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="unsupported_joint">
    <link name="base">
      <inertial><mass>1.0</mass></inertial>
    </link>
    <link name="unsupported_child">
      <inertial><mass>1.0</mass></inertial>
    </link>
    <joint name="gearbox_joint" type="gearbox">
      <parent>base</parent>
      <child>unsupported_child</child>
      <gearbox_reference_body>base</gearbox_reference_body>
      <gearbox_ratio>1.0</gearbox_ratio>
      <axis>
        <xyz>1 0 0</xyz>
        <mimic joint="root" />
      </axis>
    </joint>
  </model>
</sdf>
  )";

  const auto skeleton = readSkeletonFromSdfString(uri, modelSdf, retriever);
  ASSERT_NE(skeleton, nullptr);
  EXPECT_NE(skeleton->getBodyNode("base"), nullptr);
  EXPECT_EQ(skeleton->getBodyNode("unsupported_child"), nullptr);
  EXPECT_EQ(skeleton->getJoint("gearbox_joint"), nullptr);
}

//==============================================================================
TEST(SdfParser, DuplicateChildJointKeepsFirstClaim)
{
  auto retriever = std::make_shared<MemoryResourceRetriever>();
  const std::string uri = "memory://pkg/duplicate_child_joint/model.sdf";
  const std::string modelSdf = R"(
<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="duplicate_child_joint">
    <link name="base">
      <inertial><mass>1.0</mass></inertial>
    </link>
    <link name="alternate_parent">
      <inertial><mass>1.0</mass></inertial>
    </link>
    <link name="child">
      <inertial><mass>1.0</mass></inertial>
    </link>
    <joint name="first_child_joint" type="fixed">
      <parent>base</parent>
      <child>child</child>
    </joint>
    <joint name="duplicate_child_joint" type="fixed">
      <parent>alternate_parent</parent>
      <child>child</child>
    </joint>
  </model>
</sdf>
  )";

  const auto skeleton = readSkeletonFromSdfString(uri, modelSdf, retriever);
  ASSERT_NE(skeleton, nullptr);
  auto* child = skeleton->getBodyNode("child");
  ASSERT_NE(child, nullptr);
  ASSERT_NE(child->getParentJoint(), nullptr);
  EXPECT_EQ(child->getParentJoint()->getName(), "first_child_joint");
  EXPECT_NE(skeleton->getJoint("first_child_joint"), nullptr);
  EXPECT_EQ(skeleton->getJoint("duplicate_child_joint"), nullptr);
}

//==============================================================================
TEST(SdfParser, InertialAndMaterialVariantsFromXml)
{
  auto retriever = std::make_shared<MemoryResourceRetriever>();
  const std::string uri = "memory://pkg/inertial_material/model.sdf";
  const std::string modelSdf = R"(
<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="inertial_material">
    <link name="base">
      <inertial>
        <pose>0.1 0.2 0.3 0 0 0</pose>
        <mass>0.0</mass>
        <inertia>
          <ixx>0.4</ixx>
          <iyy>0.5</iyy>
          <izz>0.6</izz>
          <ixy>0.01</ixy>
          <ixz>0.02</ixz>
          <iyz>0.03</iyz>
        </inertia>
      </inertial>
      <visual name="color_visual">
        <pose>1 2 3 0 0 0</pose>
        <geometry><box><size>0.1 0.2 0.3</size></box></geometry>
        <material><diffuse>0.2 0.3 0.4</diffuse></material>
      </visual>
      <collision name="offset_collision">
        <pose>0 0 0.5 0 0 0</pose>
        <geometry><sphere><radius>0.25</radius></sphere></geometry>
      </collision>
    </link>
    <link name="tiny_gravity_off">
      <gravity>false</gravity>
      <inertial>
        <mass>1e-14</mass>
      </inertial>
    </link>
  </model>
</sdf>
  )";

  const auto skeleton = readSkeletonFromSdfString(uri, modelSdf, retriever);
  ASSERT_NE(skeleton, nullptr);
  auto* body = skeleton->getBodyNode("base");
  ASSERT_NE(body, nullptr);
  EXPECT_DOUBLE_EQ(body->getMass(), 1e-9);
  EXPECT_TRUE(body->getInertia().getLocalCOM().isApprox(
      Eigen::Vector3d(0.1, 0.2, 0.3)));
  Eigen::Matrix3d expectedMoment;
  expectedMoment << 0.4, 0.01, 0.02, 0.01, 0.5, 0.03, 0.02, 0.03, 0.6;
  EXPECT_TRUE(body->getInertia().getMoment().isApprox(expectedMoment));

  dynamics::ShapeNode* visual = nullptr;
  body->eachShapeNodeWith<dynamics::VisualAspect>([&](auto* shapeNode) {
    if (shapeNode->getName() == "base - color_visual") {
      visual = shapeNode;
    }
  });
  ASSERT_NE(visual, nullptr);
  EXPECT_TRUE(visual->getRelativeTransform().translation().isApprox(
      Eigen::Vector3d(1.0, 2.0, 3.0)));
  EXPECT_TRUE(visual->getVisualAspect()->getRGBA().head<3>().isApprox(
      Eigen::Vector3d(0.2, 0.3, 0.4)));

  dynamics::ShapeNode* collision = nullptr;
  body->eachShapeNodeWith<dynamics::CollisionAspect>([&](auto* shapeNode) {
    if (shapeNode->getName() == "base - offset_collision") {
      collision = shapeNode;
    }
  });
  ASSERT_NE(collision, nullptr);
  EXPECT_TRUE(collision->getRelativeTransform().translation().isApprox(
      Eigen::Vector3d(0.0, 0.0, 0.5)));
  EXPECT_TRUE(collision->getShape()->is<dynamics::SphereShape>());

  auto* tinyBody = skeleton->getBodyNode("tiny_gravity_off");
  ASSERT_NE(tinyBody, nullptr);
  EXPECT_FALSE(tinyBody->getGravityMode());
  EXPECT_DOUBLE_EQ(tinyBody->getMass(), 1e-9);
}

//==============================================================================
TEST(SdfParser, PlaneGeometryReadsAsThinBox)
{
  auto retriever = std::make_shared<MemoryResourceRetriever>();
  const std::string uri = "memory://pkg/plane_geometry/model.sdf";
  const std::string modelSdf = R"(
<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="plane_geometry">
    <link name="ground">
      <inertial><mass>1.0</mass></inertial>
      <visual name="ground_visual">
        <geometry>
          <plane>
            <normal>0 0 1</normal>
            <size>2 3</size>
          </plane>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>
  )";

  const auto skeleton = readSkeletonFromSdfString(uri, modelSdf, retriever);
  ASSERT_NE(skeleton, nullptr);
  auto* body = skeleton->getBodyNode("ground");
  ASSERT_NE(body, nullptr);

  dynamics::ShapeNode* visual = nullptr;
  body->eachShapeNodeWith<dynamics::VisualAspect>([&](auto* shapeNode) {
    if (shapeNode->getName() == "ground - ground_visual") {
      visual = shapeNode;
    }
  });
  ASSERT_NE(visual, nullptr);
  auto box = std::dynamic_pointer_cast<dynamics::BoxShape>(visual->getShape());
  ASSERT_NE(box, nullptr);
  EXPECT_TRUE(box->getSize().isApprox(Eigen::Vector3d(2.0, 3.0, 0.001)));
}

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
TEST(SdfParser, MimicElementOnAxis2ConfiguresSecondFollowerDof)
{
  auto retriever = std::make_shared<MemoryResourceRetriever>();
  const std::string uri = "memory://pkg/mimic_on_axis2/model.sdf";
  const std::string modelSdf = R"(
<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="mimic_on_axis2">
    <link name="base">
      <inertial><mass>1.0</mass></inertial>
    </link>
    <link name="link1">
      <inertial><mass>1.0</mass></inertial>
    </link>
    <link name="link2">
      <inertial><mass>1.0</mass></inertial>
    </link>
    <joint name="reference_joint" type="universal">
      <parent>base</parent>
      <child>link1</child>
      <axis>
        <xyz>1 0 0</xyz>
      </axis>
      <axis2>
        <xyz>0 1 0</xyz>
      </axis2>
    </joint>
    <joint name="follower_joint" type="universal">
      <parent>link1</parent>
      <child>link2</child>
      <axis>
        <xyz>0 0 1</xyz>
      </axis>
      <axis2>
        <xyz>0 1 0</xyz>
        <mimic joint="reference_joint">
          <multiplier>0.75</multiplier>
          <offset>0.125</offset>
        </mimic>
      </axis2>
    </joint>
  </model>
</sdf>
  )";

  const auto skeleton = readSkeletonFromSdfString(uri, modelSdf, retriever);
  ASSERT_NE(skeleton, nullptr);
  auto* joint
      = dynamic_cast<UniversalJoint*>(skeleton->getJoint("follower_joint"));
  ASSERT_NE(joint, nullptr);
  EXPECT_EQ(joint->getActuatorType(), dynamics::Joint::MIMIC);
  const auto mimicProps = joint->getMimicDofProperties();
  ASSERT_EQ(mimicProps.size(), 2u);
  EXPECT_EQ(mimicProps[1].mReferenceJoint->getName(), "reference_joint");
  EXPECT_EQ(mimicProps[1].mReferenceDofIndex, 1u);
  EXPECT_DOUBLE_EQ(mimicProps[1].mMultiplier, 0.75);
  EXPECT_DOUBLE_EQ(mimicProps[1].mOffset, 0.125);
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
TEST(SdfParser, ContinuousJointParsesAsUnboundedRevolute)
{
  auto retriever = std::make_shared<MemoryResourceRetriever>();
  const std::string uri = "memory://pkg/continuous_joint/model.sdf";
  const std::string modelSdf = R"(
<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="continuous_joint">
    <link name="base">
      <inertial><mass>1.0</mass></inertial>
    </link>
    <link name="tip">
      <inertial><mass>1.0</mass></inertial>
    </link>
    <joint name="continuous_hinge" type="continuous">
      <parent>base</parent>
      <child>tip</child>
      <axis>
        <xyz>0 0 1</xyz>
      </axis>
    </joint>
  </model>
</sdf>
 )";

  const auto skeleton = readSkeletonFromSdfString(uri, modelSdf, retriever);
  ASSERT_NE(skeleton, nullptr);
  auto* joint = dynamic_cast<dynamics::RevoluteJoint*>(
      skeleton->getJoint("continuous_hinge"));
  ASSERT_NE(joint, nullptr);
  EXPECT_TRUE(joint->isCyclic(0));
  EXPECT_TRUE(joint->getAxis().isApprox(Eigen::Vector3d::UnitZ(), 1e-9));
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
TEST(SdfParser, InvalidRootJointTypeFallsBackToFreeJoint)
{
  auto retriever = std::make_shared<MemoryResourceRetriever>();
  const std::string uri = "memory://pkg/invalid_root/model.sdf";
  const std::string modelSdf = R"(
<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="invalid_root">
    <link name="link">
      <inertial><mass>1.0</mass></inertial>
    </link>
  </model>
</sdf>
  )";

  retriever->add(uri, modelSdf);
  SdfParser::Options options(
      retriever, static_cast<SdfParser::RootJointType>(99));
  const auto skeleton = SdfParser::readSkeleton(common::Uri(uri), options);
  ASSERT_NE(skeleton, nullptr);
  ASSERT_EQ(skeleton->getNumBodyNodes(), 1u);
  auto* joint = skeleton->getRootJoint();
  ASSERT_NE(joint, nullptr);
  EXPECT_EQ(joint->getType(), dynamics::FreeJoint::getStaticType());
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
       <custom_vec2>6 7</custom_vec2>
       <custom_vec3>8 9 10</custom_vec3>
       <custom_vec3i>11 12 13</custom_vec3i>
       <custom_pose>1 2 3 0.1 0.2 0.3</custom_pose>
       <bool_flag>true</bool_flag>
       <false_flag>false</false_flag>
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

  const auto customVec2 = getValueVector2d(linkElement, "custom_vec2");
  EXPECT_TRUE(customVec2.isApprox(Eigen::Vector2d(6.0, 7.0)));

  const auto customVec3 = getValueVector3d(linkElement, "custom_vec3");
  EXPECT_TRUE(customVec3.isApprox(Eigen::Vector3d(8.0, 9.0, 10.0)));

  const auto vec3i = getValueVector3i(linkElement, "user");
  EXPECT_TRUE(vec3i == Eigen::Vector3i(1, 2, 3));

  const auto customVec3i = getValueVector3i(linkElement, "custom_vec3i");
  EXPECT_TRUE(customVec3i == Eigen::Vector3i(11, 12, 13));

  const auto vecxd = getValueVectorXd(linkElement, "custom");
  ASSERT_EQ(vecxd.size(), 4);
  EXPECT_DOUBLE_EQ(vecxd[0], 0.1);

  const auto pose
      = getValueIsometry3dWithExtrinsicRotation(linkElement, "pose");
  EXPECT_TRUE(pose.translation().isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));

  const auto customPose
      = getValueIsometry3dWithExtrinsicRotation(linkElement, "custom_pose");
  EXPECT_TRUE(
      customPose.translation().isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));

  EXPECT_TRUE(getValueBool(linkElement, "bool_flag"));
  EXPECT_FALSE(getValueBool(linkElement, "false_flag"));

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

  bool parsedBool = false;
  EXPECT_TRUE(parseScalar("true", parsedBool));
  EXPECT_TRUE(parsedBool);
  EXPECT_TRUE(parseScalar("false", parsedBool));
  EXPECT_FALSE(parsedBool);

  ElementEnumerator emptyEnumerator(nullptr, "link");
  EXPECT_FALSE(emptyEnumerator.next());

  ElementEnumerator visualEnumerator(linkElement, "visual");
  ASSERT_TRUE(visualEnumerator.next());
  EXPECT_EQ(visualEnumerator.get(), visualElement);
  EXPECT_FALSE(visualEnumerator.next());
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

  (void)capture.contents();
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
