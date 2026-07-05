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

#include "helpers/io_round_trip_helpers.hpp"

#include "dart/common/diagnostics.hpp"
#include "dart/common/local_resource_retriever.hpp"
#include "dart/common/resource.hpp"
#include "dart/common/resource_retriever.hpp"
#include "dart/utils/sdf/sdf_parser.hpp"
#include "dart/utils/sdf/sdf_writer.hpp"

#include <dart/config.hpp>

#include <dart/dynamics/arrow_shape.hpp>
#include <dart/dynamics/ball_joint.hpp>
#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/capsule_shape.hpp>
#include <dart/dynamics/cone_shape.hpp>
#include <dart/dynamics/convex_mesh_shape.hpp>
#include <dart/dynamics/cylinder_shape.hpp>
#include <dart/dynamics/ellipsoid_shape.hpp>
#include <dart/dynamics/euler_joint.hpp>
#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/heightmap_shape.hpp>
#include <dart/dynamics/line_segment_shape.hpp>
#include <dart/dynamics/mesh_shape.hpp>
#include <dart/dynamics/mimic_dof_properties.hpp>
#include <dart/dynamics/multi_sphere_convex_hull_shape.hpp>
#include <dart/dynamics/planar_joint.hpp>
#include <dart/dynamics/plane_shape.hpp>
#include <dart/dynamics/point_cloud_shape.hpp>
#include <dart/dynamics/prismatic_joint.hpp>
#include <dart/dynamics/pyramid_shape.hpp>
#include <dart/dynamics/revolute_joint.hpp>
#include <dart/dynamics/screw_joint.hpp>
#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/soft_body_node.hpp>
#include <dart/dynamics/sphere_shape.hpp>
#include <dart/dynamics/translational_joint.hpp>
#include <dart/dynamics/translational_joint2_d.hpp>
#include <dart/dynamics/universal_joint.hpp>
#include <dart/dynamics/voxel_grid_shape.hpp>
#include <dart/dynamics/weld_joint.hpp>

#include <gtest/gtest.h>
#include <sdf/Collision.hh>
#include <sdf/Geometry.hh>
#include <sdf/Joint.hh>
#include <sdf/JointAxis.hh>
#include <sdf/Link.hh>
#include <sdf/Material.hh>
#include <sdf/Mesh.hh>
#include <sdf/Model.hh>
#include <sdf/Pbr.hh>
#include <sdf/Root.hh>
#include <sdf/Surface.hh>
#include <sdf/Visual.hh>
#include <sdf/World.hh>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <cmath>
#include <cstring>

using namespace dart;

namespace {

class StringResource final : public common::Resource
{
public:
  explicit StringResource(std::string data) : mData(std::move(data)) {}

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
      base = static_cast<ptrdiff_t>(mData.size());
    }

    const ptrdiff_t next = base + offset;
    if (next < 0 || next > static_cast<ptrdiff_t>(mData.size())) {
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
    const std::size_t copied = std::min(bytes, available);
    std::memcpy(buffer, mData.data() + mOffset, copied);
    mOffset += copied;
    return copied / size;
  }

private:
  std::string mData;
  std::size_t mOffset{0};
};

class MapResourceRetriever final : public common::ResourceRetriever
{
public:
  void add(std::string uri, std::string data)
  {
    mResources.emplace(std::move(uri), std::move(data));
  }

  bool exists(const common::Uri& uri) override
  {
    return mResources.contains(uri.toString());
  }

  common::ResourcePtr retrieve(const common::Uri& uri) override
  {
    const auto it = mResources.find(uri.toString());
    if (it == mResources.end()) {
      return nullptr;
    }

    return std::make_shared<StringResource>(it->second);
  }

private:
  std::map<std::string, std::string> mResources;
};

constexpr std::string_view kTriangleObj = R"(
o Triangle
v 0 0 0
v 1 0 0
v 0 1 0
f 1 2 3
)";

void addTriangleObj(MapResourceRetriever& retriever, std::string_view uri)
{
  retriever.add(std::string(uri), std::string(kTriangleObj));
}

class MeshShapeConversionHarness final : public dynamics::MeshShape
{
public:
  using dynamics::MeshShape::convertAssimpMesh;
};

std::shared_ptr<math::TriMesh<double>> loadTriMesh(
    const common::Uri& uri, const common::ResourceRetrieverPtr& retriever)
{
  DART_SUPPRESS_DEPRECATED_BEGIN
  const aiScene* scene = dynamics::MeshShape::loadMesh(uri, retriever);
  DART_SUPPRESS_DEPRECATED_END
  if (!scene) {
    return nullptr;
  }

  auto triMesh = MeshShapeConversionHarness::convertAssimpMesh(scene);
  aiReleaseImport(const_cast<aiScene*>(scene));
  return triMesh;
}

std::filesystem::path writeTempSdf(std::string_view text, std::string_view name)
{
  const auto path = std::filesystem::temp_directory_path()
                    / (std::string(name) + "_dart_sdf_writer_test.sdf");
  std::ofstream output(path);
  output << text;
  return path;
}

std::string bodyNodeName(const dynamics::BodyNode* bodyNode)
{
  return bodyNode ? bodyNode->getName() : "world";
}

void expectDoubleRoundTrips(double actual, double expected, double tolerance)
{
  if (std::isfinite(actual) && std::isfinite(expected)) {
    EXPECT_NEAR(actual, expected, tolerance);
  } else {
    EXPECT_EQ(actual, expected);
  }
}

void expectBodyInertiaRoundTrips(
    const dynamics::BodyNode& bodyNode,
    const dynamics::BodyNode& expected,
    double tolerance)
{
  EXPECT_NEAR(bodyNode.getMass(), expected.getMass(), tolerance);
  EXPECT_VECTOR_NEAR(bodyNode.getLocalCOM(), expected.getLocalCOM(), tolerance);
  EXPECT_MATRIX_NEAR(
      bodyNode.getInertia().getMoment(),
      expected.getInertia().getMoment(),
      tolerance);
}

void expectJointRoundTrips(
    const dynamics::Joint& joint,
    const dynamics::Joint& expected,
    double tolerance)
{
  EXPECT_EQ(joint.getType(), expected.getType());
  EXPECT_EQ(joint.getNumDofs(), expected.getNumDofs());
  EXPECT_EQ(
      bodyNodeName(joint.getParentBodyNode()),
      bodyNodeName(expected.getParentBodyNode()));
  EXPECT_EQ(
      bodyNodeName(joint.getChildBodyNode()),
      bodyNodeName(expected.getChildBodyNode()));
  EXPECT_MATRIX_NEAR(
      joint.getTransformFromParentBodyNode().matrix(),
      expected.getTransformFromParentBodyNode().matrix(),
      tolerance);
  EXPECT_MATRIX_NEAR(
      joint.getTransformFromChildBodyNode().matrix(),
      expected.getTransformFromChildBodyNode().matrix(),
      tolerance);

  for (std::size_t i = 0; i < joint.getNumDofs(); ++i) {
    SCOPED_TRACE(i);
    expectDoubleRoundTrips(
        joint.getPositionLowerLimit(i),
        expected.getPositionLowerLimit(i),
        tolerance);
    expectDoubleRoundTrips(
        joint.getPositionUpperLimit(i),
        expected.getPositionUpperLimit(i),
        tolerance);
    expectDoubleRoundTrips(
        joint.getVelocityLowerLimit(i),
        expected.getVelocityLowerLimit(i),
        tolerance);
    expectDoubleRoundTrips(
        joint.getVelocityUpperLimit(i),
        expected.getVelocityUpperLimit(i),
        tolerance);
    expectDoubleRoundTrips(
        joint.getForceLowerLimit(i), expected.getForceLowerLimit(i), tolerance);
    expectDoubleRoundTrips(
        joint.getForceUpperLimit(i), expected.getForceUpperLimit(i), tolerance);
    expectDoubleRoundTrips(
        joint.getDampingCoefficient(i),
        expected.getDampingCoefficient(i),
        tolerance);
    expectDoubleRoundTrips(
        joint.getCoulombFriction(i), expected.getCoulombFriction(i), tolerance);
    expectDoubleRoundTrips(
        joint.getRestPosition(i), expected.getRestPosition(i), tolerance);
    expectDoubleRoundTrips(
        joint.getSpringStiffness(i), expected.getSpringStiffness(i), tolerance);
  }
}

template <class AspectT>
void expectShapeNodePoseRoundTrips(
    const dynamics::BodyNode& bodyNode,
    const dynamics::BodyNode& expected,
    std::size_t index,
    double tolerance)
{
  const auto* shapeNode = bodyNode.getShapeNodeWith<AspectT>(index);
  const auto* expectedShapeNode = expected.getShapeNodeWith<AspectT>(index);
  ASSERT_NE(shapeNode, nullptr);
  ASSERT_NE(expectedShapeNode, nullptr);
  EXPECT_MATRIX_NEAR(
      shapeNode->getRelativeTransform().matrix(),
      expectedShapeNode->getRelativeTransform().matrix(),
      tolerance);
}

template <class AspectT>
void expectBoxShapeRoundTrips(
    const dynamics::BodyNode& bodyNode,
    const dynamics::BodyNode& expected,
    std::size_t index,
    std::size_t expectedCount,
    double tolerance)
{
  const auto* expectedBox = test::requireShape<AspectT, dynamics::BoxShape>(
      expected, index, expectedCount);
  const auto* box = test::requireShape<AspectT, dynamics::BoxShape>(
      bodyNode, index, expectedCount);
  ASSERT_NE(expectedBox, nullptr);
  ASSERT_NE(box, nullptr);
  EXPECT_VECTOR_NEAR(box->getSize(), expectedBox->getSize(), tolerance);
  expectShapeNodePoseRoundTrips<AspectT>(bodyNode, expected, index, tolerance);
}

sdf::ElementPtr getSdfChildElement(
    const sdf::ElementPtr& parent, const std::string& name)
{
  return parent ? parent->FindElement(name) : nullptr;
}

double getSdfDoubleElement(
    const sdf::ElementPtr& parent, const std::string& name)
{
  sdf::Errors errors;
  const auto [value, found] = parent->Get<double>(errors, name, 0.0);
  EXPECT_TRUE(found);
  EXPECT_TRUE(errors.empty())
      << (errors.empty() ? "" : errors.front().Message());
  return value;
}

std::shared_ptr<math::TriMesh<double>> makeTriangleMesh()
{
  auto mesh = std::make_shared<math::TriMesh<double>>();
  mesh->addVertex(0.0, 0.0, 0.0);
  mesh->addVertex(1.0, 0.0, 0.0);
  mesh->addVertex(0.0, 1.0, 0.0);
  mesh->addTriangle(0, 1, 2);
  return mesh;
}

dynamics::SkeletonPtr makeRoundTripSkeleton()
{
  auto skeleton = dynamics::Skeleton::create("writer_roundtrip");

  dynamics::FreeJoint::Properties rootProperties;
  rootProperties.mName = "root";
  rootProperties.mT_ParentBodyToJoint.translation()
      = Eigen::Vector3d(0.25, -0.5, 1.0);

  dynamics::BodyNode::Properties baseProperties;
  baseProperties.mName = "base";
  baseProperties.mInertia.setMass(2.5);
  baseProperties.mInertia.setLocalCOM(Eigen::Vector3d(0.01, 0.02, 0.03));
  baseProperties.mInertia.setMoment(0.2, 0.3, 0.4, 0.01, 0.02, 0.03);

  auto [rootJoint, base]
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>(
          nullptr, rootProperties, baseProperties);
  (void)rootJoint;

  auto* baseShape = base->createShapeNodeWith<
      dynamics::VisualAspect,
      dynamics::CollisionAspect>(
      std::make_shared<dynamics::BoxShape>(Eigen::Vector3d(0.4, 0.5, 0.6)),
      "base_box");
  baseShape->setRelativeTranslation(Eigen::Vector3d(0.1, 0.0, 0.0));
  baseShape->getVisualAspect()->setRGBA(Eigen::Vector4d(0.2, 0.4, 0.6, 0.8));
  baseShape->getVisualAspect()->setMetallic(0.65);
  baseShape->getVisualAspect()->setRoughness(0.35);

  dynamics::RevoluteJoint::Properties hingeProperties;
  hingeProperties.mName = "hinge";
  hingeProperties.mAxis = Eigen::Vector3d::UnitZ();
  hingeProperties.mT_ParentBodyToJoint.translation()
      = Eigen::Vector3d(0.0, 0.0, 0.8);
  hingeProperties.mT_ChildBodyToJoint.translation()
      = Eigen::Vector3d(0.0, 0.0, -0.2);
  hingeProperties.mPositionLowerLimits[0] = -1.25;
  hingeProperties.mPositionUpperLimits[0] = 1.5;

  dynamics::BodyNode::Properties linkProperties;
  linkProperties.mName = "tip";
  linkProperties.mInertia.setMass(1.25);
  linkProperties.mInertia.setMoment(
      Eigen::Matrix3d::Identity() * linkProperties.mInertia.getMass());

  auto [hinge, tip]
      = skeleton->createJointAndBodyNodePair<dynamics::RevoluteJoint>(
          base, hingeProperties, linkProperties);
  hinge->setDampingCoefficient(0, 0.25);
  hinge->setCoulombFriction(0, 0.125);
  hinge->setRestPosition(0, -0.5);
  hinge->setSpringStiffness(0, 3.75);

  tip->createShapeNodeWith<dynamics::VisualAspect>(
      std::make_shared<dynamics::SphereShape>(0.15), "tip_sphere_visual");
  tip->createShapeNodeWith<dynamics::CollisionAspect>(
      std::make_shared<dynamics::CylinderShape>(0.1, 0.3),
      "tip_cylinder_collision");

  dynamics::PrismaticJoint::Properties sliderProperties;
  sliderProperties.mName = "slider_joint";
  sliderProperties.mAxis = Eigen::Vector3d::UnitX();
  sliderProperties.mT_ParentBodyToJoint.translation()
      = Eigen::Vector3d(0.0, 0.2, 0.0);
  sliderProperties.mPositionLowerLimits[0] = 0.1;
  sliderProperties.mPositionUpperLimits[0] = 0.9;

  dynamics::BodyNode::Properties sliderBodyProperties;
  sliderBodyProperties.mName = "slider";
  sliderBodyProperties.mInertia.setMass(0.75);
  sliderBodyProperties.mInertia.setMoment(Eigen::Matrix3d::Identity() * 0.75);

  auto [sliderJoint, slider]
      = skeleton->createJointAndBodyNodePair<dynamics::PrismaticJoint>(
          tip, sliderProperties, sliderBodyProperties);
  (void)sliderJoint;

  const common::Uri meshUri
      = common::Uri::createFromPath(dart::config::dataPath("obj/BoxSmall.obj"));
  slider->createShapeNodeWith<dynamics::VisualAspect>(
      std::make_shared<dynamics::MeshShape>(
          Eigen::Vector3d(0.5, 1.0, 2.0), makeTriangleMesh(), meshUri),
      "slider_mesh_visual");

  dynamics::WeldJoint::Properties weldProperties;
  weldProperties.mName = "fixed_tip";
  weldProperties.mT_ParentBodyToJoint.translation()
      = Eigen::Vector3d(0.0, 0.0, 0.25);

  dynamics::BodyNode::Properties fixedBodyProperties;
  fixedBodyProperties.mName = "fixed";
  fixedBodyProperties.mInertia.setMass(0.25);
  fixedBodyProperties.mInertia.setMoment(Eigen::Matrix3d::Identity() * 0.25);

  auto [fixedJoint, fixed]
      = skeleton->createJointAndBodyNodePair<dynamics::WeldJoint>(
          slider, weldProperties, fixedBodyProperties);
  (void)fixedJoint;
  fixed->setGravityMode(false);
  fixed->createShapeNodeWith<dynamics::CollisionAspect>(
      std::make_shared<dynamics::BoxShape>(Eigen::Vector3d(0.1, 0.1, 0.1)),
      "fixed_box_collision");

  dynamics::ScrewJoint::Properties screwProperties;
  screwProperties.mName = "screw_drive";
  screwProperties.mAxis = Eigen::Vector3d::UnitY();
  screwProperties.mPitch = 0.25;
  screwProperties.mT_ParentBodyToJoint.translation()
      = Eigen::Vector3d(0.2, 0.0, 0.0);
  screwProperties.mPositionLowerLimits[0] = -0.75;
  screwProperties.mPositionUpperLimits[0] = 0.75;

  dynamics::BodyNode::Properties screwBodyProperties;
  screwBodyProperties.mName = "screw_tip";
  screwBodyProperties.mInertia.setMass(0.3);
  screwBodyProperties.mInertia.setMoment(Eigen::Matrix3d::Identity() * 0.3);

  auto [screwJoint, screwTip]
      = skeleton->createJointAndBodyNodePair<dynamics::ScrewJoint>(
          fixed, screwProperties, screwBodyProperties);
  screwJoint->setDampingCoefficient(0, 0.2);
  screwJoint->setCoulombFriction(0, 0.05);
  screwJoint->setRestPosition(0, 0.1);
  screwJoint->setSpringStiffness(0, 1.5);
  screwTip->createShapeNodeWith<dynamics::CollisionAspect>(
      std::make_shared<dynamics::SphereShape>(0.08), "screw_tip_collision");

  dynamics::UniversalJoint::Properties universalProperties;
  universalProperties.mName = "universal_shoulder";
  universalProperties.mAxis[0] = Eigen::Vector3d::UnitX();
  universalProperties.mAxis[1] = Eigen::Vector3d::UnitZ();
  universalProperties.mT_ParentBodyToJoint.translation()
      = Eigen::Vector3d(0.0, 0.1, 0.0);
  universalProperties.mPositionLowerLimits[0] = -0.4;
  universalProperties.mPositionUpperLimits[0] = 0.6;
  universalProperties.mPositionLowerLimits[1] = -1.25;
  universalProperties.mPositionUpperLimits[1] = 1.5;

  dynamics::BodyNode::Properties universalBodyProperties;
  universalBodyProperties.mName = "universal_tip";
  universalBodyProperties.mInertia.setMass(0.2);
  universalBodyProperties.mInertia.setMoment(Eigen::Matrix3d::Identity() * 0.2);

  auto [universalJoint, universalTip]
      = skeleton->createJointAndBodyNodePair<dynamics::UniversalJoint>(
          screwTip, universalProperties, universalBodyProperties);
  universalJoint->setDampingCoefficient(0, 0.3);
  universalJoint->setCoulombFriction(0, 0.06);
  universalJoint->setRestPosition(0, -0.2);
  universalJoint->setSpringStiffness(0, 2.5);
  universalJoint->setDampingCoefficient(1, 0.45);
  universalJoint->setCoulombFriction(1, 0.08);
  universalJoint->setRestPosition(1, 0.35);
  universalJoint->setSpringStiffness(1, 4.5);
  universalTip->createShapeNodeWith<dynamics::CollisionAspect>(
      std::make_shared<dynamics::CylinderShape>(0.04, 0.18),
      "universal_tip_collision");
  universalTip->createShapeNodeWith<dynamics::VisualAspect>(
      std::make_shared<dynamics::EllipsoidShape>(
          Eigen::Vector3d(0.12, 0.16, 0.2)),
      "universal_tip_ellipsoid");

  dynamics::BallJoint::Properties ballProperties;
  ballProperties.mName = "ball_socket";
  ballProperties.mT_ParentBodyToJoint.translation()
      = Eigen::Vector3d(0.0, -0.1, 0.0);

  dynamics::BodyNode::Properties ballBodyProperties;
  ballBodyProperties.mName = "ball_tip";
  ballBodyProperties.mInertia.setMass(0.15);
  ballBodyProperties.mInertia.setMoment(Eigen::Matrix3d::Identity() * 0.15);

  auto [ballJoint, ballTip]
      = skeleton->createJointAndBodyNodePair<dynamics::BallJoint>(
          universalTip, ballProperties, ballBodyProperties);
  (void)ballJoint;
  ballTip->createShapeNodeWith<dynamics::CollisionAspect>(
      std::make_shared<dynamics::SphereShape>(0.05), "ball_tip_collision");

  return skeleton;
}

template <typename JointType>
void expectUnsupportedChildJointError(
    std::string_view skeletonName,
    std::string_view jointName,
    std::string_view expectedSnippet)
{
  auto skeleton = dynamics::Skeleton::create(std::string(skeletonName));
  auto [rootJoint, base]
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>();
  (void)rootJoint;
  base->setName("base");

  typename JointType::Properties jointProperties;
  jointProperties.mName = std::string(jointName);

  dynamics::BodyNode::Properties tipProperties;
  tipProperties.mName = "tip";

  auto [joint, tip] = skeleton->createJointAndBodyNodePair<JointType>(
      base, jointProperties, tipProperties);
  (void)joint;
  (void)tip;

  const auto result = utils::SdfParser::tryWriteSkeletonToString(*skeleton);
  ASSERT_TRUE(result.isErr());
  EXPECT_NE(
      result.error().message.find(std::string(expectedSnippet)),
      std::string::npos)
      << result.error().message;
}

template <typename JointType>
void expectUnsupportedRootJointError(
    std::string_view skeletonName,
    std::string_view jointName,
    std::string_view expectedSnippet)
{
  auto skeleton = dynamics::Skeleton::create(std::string(skeletonName));

  typename JointType::Properties jointProperties;
  jointProperties.mName = std::string(jointName);

  dynamics::BodyNode::Properties bodyProperties;
  bodyProperties.mName = "root_body";

  auto [joint, body] = skeleton->createJointAndBodyNodePair<JointType>(
      nullptr, jointProperties, bodyProperties);
  (void)joint;
  (void)body;

  const auto result = utils::SdfParser::tryWriteSkeletonToString(*skeleton);
  ASSERT_TRUE(result.isErr());
  EXPECT_NE(
      result.error().message.find(std::string(expectedSnippet)),
      std::string::npos)
      << result.error().message;
}

dynamics::RevoluteJoint* createLimitProbeJoint(
    dynamics::SkeletonPtr& skeleton, std::string_view skeletonName)
{
  skeleton = dynamics::Skeleton::create(std::string(skeletonName));
  auto [rootJoint, base]
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>();
  (void)rootJoint;
  base->setName("base");

  dynamics::RevoluteJoint::Properties hingeProperties;
  hingeProperties.mName = "limited_hinge";
  hingeProperties.mAxis = Eigen::Vector3d::UnitZ();

  dynamics::BodyNode::Properties tipProperties;
  tipProperties.mName = "tip";

  auto [hinge, tip]
      = skeleton->createJointAndBodyNodePair<dynamics::RevoluteJoint>(
          base, hingeProperties, tipProperties);
  (void)tip;
  return hinge;
}

} // namespace

//==============================================================================
TEST(SdfWriter, RoundTripsSupportedSkeletonSubset)
{
  const auto original = makeRoundTripSkeleton();

  const auto writeResult
      = utils::SdfParser::tryWriteSkeletonToString(*original);
  ASSERT_TRUE(writeResult.isOk()) << writeResult.error().message;
  EXPECT_NE(writeResult.value().find("<thread_pitch>"), std::string::npos);
  EXPECT_EQ(
      writeResult.value().find("<screw_thread_pitch>"), std::string::npos);

  sdf::Root sdfRoot;
  const auto sdfErrors = sdfRoot.LoadSdfString(writeResult.value());
  ASSERT_TRUE(sdfErrors.empty()) << sdfErrors.front().Message();
  const auto* sdfModel = sdfRoot.Model();
  ASSERT_NE(sdfModel, nullptr);
  EXPECT_EQ(sdfModel->Name(), "writer_roundtrip");

  const auto* sdfFixedLink = sdfModel->LinkByName("fixed");
  ASSERT_NE(sdfFixedLink, nullptr);
  EXPECT_FALSE(sdfFixedLink->EnableGravity());

  const auto* sdfHingeJoint = sdfModel->JointByName("hinge");
  ASSERT_NE(sdfHingeJoint, nullptr);
  EXPECT_EQ(sdfHingeJoint->Type(), sdf::JointType::REVOLUTE);

  const auto* sdfScrewJoint = sdfModel->JointByName("screw_drive");
  ASSERT_NE(sdfScrewJoint, nullptr);
  EXPECT_EQ(sdfScrewJoint->Type(), sdf::JointType::SCREW);
  EXPECT_NEAR(sdfScrewJoint->ScrewThreadPitch(), 0.25, 1e-12);

  const auto* sdfUniversalJoint = sdfModel->JointByName("universal_shoulder");
  ASSERT_NE(sdfUniversalJoint, nullptr);
  EXPECT_EQ(sdfUniversalJoint->Type(), sdf::JointType::UNIVERSAL);
  ASSERT_NE(sdfUniversalJoint->Axis(1), nullptr);

  const auto* sdfBallJoint = sdfModel->JointByName("ball_socket");
  ASSERT_NE(sdfBallJoint, nullptr);
  EXPECT_EQ(sdfBallJoint->Type(), sdf::JointType::BALL);

  ASSERT_NE(sdfHingeJoint->Axis(0), nullptr);
  EXPECT_NEAR(sdfHingeJoint->Axis(0)->Damping(), 0.25, 1e-12);
  EXPECT_NEAR(sdfHingeJoint->Axis(0)->Friction(), 0.125, 1e-12);
  EXPECT_NEAR(sdfHingeJoint->Axis(0)->SpringReference(), -0.5, 1e-12);
  EXPECT_NEAR(sdfHingeJoint->Axis(0)->SpringStiffness(), 3.75, 1e-12);

  const auto* sdfUniversalTip = sdfModel->LinkByName("universal_tip");
  ASSERT_NE(sdfUniversalTip, nullptr);
  const auto* sdfEllipsoidVisual
      = sdfUniversalTip->VisualByName("universal_tip_ellipsoid");
  ASSERT_NE(sdfEllipsoidVisual, nullptr);
  ASSERT_NE(sdfEllipsoidVisual->Geom(), nullptr);
  EXPECT_EQ(sdfEllipsoidVisual->Geom()->Type(), sdf::GeometryType::ELLIPSOID);

  const auto* sdfBase = sdfModel->LinkByName("base");
  ASSERT_NE(sdfBase, nullptr);
  const auto* sdfBaseVisual = sdfBase->VisualByName("base_box");
  ASSERT_NE(sdfBaseVisual, nullptr);
  const auto* sdfBaseMaterial = sdfBaseVisual->Material();
  ASSERT_NE(sdfBaseMaterial, nullptr);
  EXPECT_EQ(sdfBaseMaterial->Diffuse(), gz::math::Color(0.2, 0.4, 0.6, 0.8));
  ASSERT_NE(sdfBaseMaterial->PbrMaterial(), nullptr);
  const auto* workflow
      = sdfBaseMaterial->PbrMaterial()->Workflow(sdf::PbrWorkflowType::METAL);
  ASSERT_NE(workflow, nullptr);
  EXPECT_NEAR(workflow->Metalness(), 0.65, 1e-12);
  EXPECT_NEAR(workflow->Roughness(), 0.35, 1e-12);

  const auto path = writeTempSdf(writeResult.value(), "roundtrip");
  const auto reparsed = utils::SdfParser::readSkeleton(
      common::Uri::createFromPath(path.string()));
  std::filesystem::remove(path);

  ASSERT_NE(reparsed, nullptr);
  EXPECT_EQ(reparsed->getName(), original->getName());
  EXPECT_EQ(reparsed->getNumBodyNodes(), original->getNumBodyNodes());
  EXPECT_EQ(reparsed->getNumJoints(), original->getNumJoints());

  const auto* base = test::requireBodyNode(*reparsed, "base");
  const auto* tip = test::requireBodyNode(*reparsed, "tip");
  const auto* slider = test::requireBodyNode(*reparsed, "slider");
  const auto* fixed = test::requireBodyNode(*reparsed, "fixed");
  const auto* screwTip = test::requireBodyNode(*reparsed, "screw_tip");
  const auto* universalTip = test::requireBodyNode(*reparsed, "universal_tip");
  const auto* ballTip = test::requireBodyNode(*reparsed, "ball_tip");
  ASSERT_NE(base, nullptr);
  ASSERT_NE(tip, nullptr);
  ASSERT_NE(slider, nullptr);
  ASSERT_NE(fixed, nullptr);
  ASSERT_NE(screwTip, nullptr);
  ASSERT_NE(universalTip, nullptr);
  ASSERT_NE(ballTip, nullptr);

  const auto* root = test::requireJoint<dynamics::FreeJoint>(*reparsed, "root");
  ASSERT_NE(root, nullptr);
  test::expectJointTopology(*root, nullptr, base);
  EXPECT_VECTOR_NEAR(
      root->getTransformFromParentBodyNode().translation(),
      Eigen::Vector3d(0.25, -0.5, 1.0),
      1e-12);

  Eigen::Matrix3d baseMoment;
  baseMoment << 0.2, 0.01, 0.02, 0.01, 0.3, 0.03, 0.02, 0.03, 0.4;
  test::expectBodyInertia(
      *base, 2.5, Eigen::Vector3d(0.01, 0.02, 0.03), baseMoment, 1e-12);

  const auto* hinge
      = test::requireJoint<dynamics::RevoluteJoint>(*reparsed, "hinge");
  ASSERT_NE(hinge, nullptr);
  test::expectJointTopology(*hinge, base, tip);
  EXPECT_VECTOR_NEAR(
      hinge->getTransformFromParentBodyNode().translation(),
      Eigen::Vector3d(0.0, 0.0, 0.8),
      1e-12);
  EXPECT_VECTOR_NEAR(
      hinge->getTransformFromChildBodyNode().translation(),
      Eigen::Vector3d(0.0, 0.0, -0.2),
      1e-12);
  EXPECT_VECTOR_NEAR(hinge->getAxis(), Eigen::Vector3d::UnitZ(), 1e-12);
  test::expectDofPositionLimits(*hinge, 0, -1.25, 1.5, 1e-12);
  test::expectDofDynamics(*hinge, 0, 0.25, 0.125, -0.5, 3.75, 1e-12);

  const auto* sliderJoint
      = test::requireJoint<dynamics::PrismaticJoint>(*reparsed, "slider_joint");
  ASSERT_NE(sliderJoint, nullptr);
  test::expectJointTopology(*sliderJoint, tip, slider);
  EXPECT_VECTOR_NEAR(sliderJoint->getAxis(), Eigen::Vector3d::UnitX(), 1e-12);
  test::expectDofPositionLimits(*sliderJoint, 0, 0.1, 0.9, 1e-12);

  const auto* fixedJoint
      = test::requireJoint<dynamics::WeldJoint>(*reparsed, "fixed_tip");
  ASSERT_NE(fixedJoint, nullptr);
  test::expectJointTopology(*fixedJoint, slider, fixed);
  EXPECT_FALSE(fixed->getGravityMode());

  const auto* screwJoint
      = test::requireJoint<dynamics::ScrewJoint>(*reparsed, "screw_drive");
  ASSERT_NE(screwJoint, nullptr);
  test::expectJointTopology(*screwJoint, fixed, screwTip);
  EXPECT_VECTOR_NEAR(screwJoint->getAxis(), Eigen::Vector3d::UnitY(), 1e-12);
  EXPECT_NEAR(screwJoint->getPitch(), 0.25, 1e-12);
  test::expectDofPositionLimits(*screwJoint, 0, -0.75, 0.75, 1e-12);
  test::expectDofDynamics(*screwJoint, 0, 0.2, 0.05, 0.1, 1.5, 1e-12);

  const auto* universalJoint = test::requireJoint<dynamics::UniversalJoint>(
      *reparsed, "universal_shoulder");
  ASSERT_NE(universalJoint, nullptr);
  test::expectJointTopology(*universalJoint, screwTip, universalTip);
  EXPECT_VECTOR_NEAR(
      universalJoint->getAxis1(), Eigen::Vector3d::UnitX(), 1e-12);
  EXPECT_VECTOR_NEAR(
      universalJoint->getAxis2(), Eigen::Vector3d::UnitZ(), 1e-12);
  test::expectDofPositionLimits(*universalJoint, 0, -0.4, 0.6, 1e-12);
  test::expectDofDynamics(*universalJoint, 0, 0.3, 0.06, -0.2, 2.5, 1e-12);
  test::expectDofPositionLimits(*universalJoint, 1, -1.25, 1.5, 1e-12);
  test::expectDofDynamics(*universalJoint, 1, 0.45, 0.08, 0.35, 4.5, 1e-12);

  const auto* ballJoint
      = test::requireJoint<dynamics::BallJoint>(*reparsed, "ball_socket");
  ASSERT_NE(ballJoint, nullptr);
  test::expectJointTopology(*ballJoint, universalTip, ballTip);
  EXPECT_EQ(ballJoint->getNumDofs(), 3u);

  const auto* baseBox
      = test::requireShape<dynamics::VisualAspect, dynamics::BoxShape>(
          *base, 0, 1);
  ASSERT_NE(baseBox, nullptr);
  EXPECT_VECTOR_NEAR(baseBox->getSize(), Eigen::Vector3d(0.4, 0.5, 0.6), 1e-12);
  const auto* baseCollisionBox
      = test::requireShape<dynamics::CollisionAspect, dynamics::BoxShape>(
          *base, 0, 1);
  ASSERT_NE(baseCollisionBox, nullptr);
  EXPECT_VECTOR_NEAR(
      baseCollisionBox->getSize(), Eigen::Vector3d(0.4, 0.5, 0.6), 1e-12);
  EXPECT_VECTOR_NEAR(
      base->getShapeNodeWith<dynamics::VisualAspect>(0)
          ->getRelativeTransform()
          .translation(),
      Eigen::Vector3d(0.1, 0.0, 0.0),
      1e-12);
  EXPECT_VECTOR_NEAR(
      base->getShapeNodeWith<dynamics::VisualAspect>(0)
          ->getVisualAspect()
          ->getRGBA(),
      Eigen::Vector4d(0.2, 0.4, 0.6, 0.8),
      1e-6);
  EXPECT_NEAR(
      base->getShapeNodeWith<dynamics::VisualAspect>(0)
          ->getVisualAspect()
          ->getMetallic(),
      0.65,
      1e-12);
  EXPECT_NEAR(
      base->getShapeNodeWith<dynamics::VisualAspect>(0)
          ->getVisualAspect()
          ->getRoughness(),
      0.35,
      1e-12);

  const auto* tipSphere
      = test::requireShape<dynamics::VisualAspect, dynamics::SphereShape>(
          *tip, 0, 1);
  ASSERT_NE(tipSphere, nullptr);
  EXPECT_NEAR(tipSphere->getRadius(), 0.15, 1e-12);

  const auto* tipCylinder
      = test::requireShape<dynamics::CollisionAspect, dynamics::CylinderShape>(
          *tip, 0, 1);
  ASSERT_NE(tipCylinder, nullptr);
  EXPECT_NEAR(tipCylinder->getRadius(), 0.1, 1e-12);
  EXPECT_NEAR(tipCylinder->getHeight(), 0.3, 1e-12);

  const auto* sliderMesh
      = test::requireShape<dynamics::VisualAspect, dynamics::MeshShape>(
          *slider, 0, 1);
  ASSERT_NE(sliderMesh, nullptr);
  EXPECT_VECTOR_NEAR(
      sliderMesh->getScale(), Eigen::Vector3d(0.5, 1.0, 2.0), 1e-12);
  const common::Uri meshUri
      = common::Uri::createFromPath(dart::config::dataPath("obj/BoxSmall.obj"));
  EXPECT_EQ(sliderMesh->getMeshUri2().toString(), meshUri.toString());

  const auto* fixedBox
      = test::requireShape<dynamics::CollisionAspect, dynamics::BoxShape>(
          *fixed, 0, 1);
  ASSERT_NE(fixedBox, nullptr);
  EXPECT_VECTOR_NEAR(
      fixedBox->getSize(), Eigen::Vector3d(0.1, 0.1, 0.1), 1e-12);

  const auto* screwTipSphere
      = test::requireShape<dynamics::CollisionAspect, dynamics::SphereShape>(
          *screwTip, 0, 1);
  ASSERT_NE(screwTipSphere, nullptr);
  EXPECT_NEAR(screwTipSphere->getRadius(), 0.08, 1e-12);

  const auto* universalTipCylinder
      = test::requireShape<dynamics::CollisionAspect, dynamics::CylinderShape>(
          *universalTip, 0, 1);
  ASSERT_NE(universalTipCylinder, nullptr);
  EXPECT_NEAR(universalTipCylinder->getRadius(), 0.04, 1e-12);
  EXPECT_NEAR(universalTipCylinder->getHeight(), 0.18, 1e-12);
  const auto* universalTipEllipsoid
      = test::requireShape<dynamics::VisualAspect, dynamics::EllipsoidShape>(
          *universalTip, 0, 1);
  ASSERT_NE(universalTipEllipsoid, nullptr);
  EXPECT_VECTOR_NEAR(
      universalTipEllipsoid->getDiameters(),
      Eigen::Vector3d(0.12, 0.16, 0.2),
      1e-12);

  const auto* ballTipSphere
      = test::requireShape<dynamics::CollisionAspect, dynamics::SphereShape>(
          *ballTip, 0, 1);
  ASSERT_NE(ballTipSphere, nullptr);
  EXPECT_NEAR(ballTipSphere->getRadius(), 0.05, 1e-12);
}

//==============================================================================
TEST(SdfWriter, RoundTripsExistingSinglePendulumFixture)
{
  const auto original = utils::SdfParser::readSkeleton(
      common::Uri("dart://sample/sdf/test/single_pendulum.sdf"));
  ASSERT_NE(original, nullptr);
  ASSERT_EQ(original->getNumBodyNodes(), 1u);
  ASSERT_EQ(original->getNumJoints(), 1u);
  const auto* originalLink = test::requireBodyNode(*original, "link 1");
  ASSERT_NE(originalLink, nullptr);
  const auto* originalParentJoint = originalLink->getParentJoint();
  ASSERT_NE(originalParentJoint, nullptr);

  const auto writeResult
      = utils::SdfParser::tryWriteSkeletonToString(*original);
  ASSERT_TRUE(writeResult.isOk()) << writeResult.error().message;

  const auto path
      = writeTempSdf(writeResult.value(), "existing_single_pendulum");
  const auto reparsed = utils::SdfParser::readSkeleton(
      common::Uri::createFromPath(path.string()));
  std::filesystem::remove(path);

  ASSERT_NE(reparsed, nullptr);
  EXPECT_EQ(reparsed->getName(), original->getName());
  EXPECT_EQ(reparsed->getNumBodyNodes(), original->getNumBodyNodes());
  EXPECT_EQ(reparsed->getNumJoints(), original->getNumJoints());

  const auto* link = test::requireBodyNode(*reparsed, "link 1");
  ASSERT_NE(link, nullptr);
  const auto* parentJoint = link->getParentJoint();
  ASSERT_NE(parentJoint, nullptr);
  EXPECT_VECTOR_NEAR(
      parentJoint->getTransformFromChildBodyNode().translation(),
      originalParentJoint->getTransformFromChildBodyNode().translation(),
      1e-12);

  Eigen::Matrix3d expectedMoment = Eigen::Matrix3d::Zero();
  expectedMoment.diagonal() = Eigen::Vector3d(1.0, 2.0, 3.0);
  test::expectBodyInertia(
      *link, 5.0, Eigen::Vector3d::Zero(), expectedMoment, 1e-12);

  const auto* joint
      = test::requireJoint<dynamics::RevoluteJoint>(*reparsed, "joint 1");
  ASSERT_NE(joint, nullptr);
  test::expectJointTopology(*joint, nullptr, link);
  EXPECT_VECTOR_NEAR(joint->getAxis(), Eigen::Vector3d::UnitZ(), 1e-12);
  EXPECT_NEAR(joint->getDampingCoefficient(0), 10.0, 1e-12);

  const auto* visualBox
      = test::requireShape<dynamics::VisualAspect, dynamics::BoxShape>(
          *link, 0, 1);
  ASSERT_NE(visualBox, nullptr);
  EXPECT_VECTOR_NEAR(
      visualBox->getSize(), Eigen::Vector3d(0.1, 0.2, 0.3), 1e-12);

  const auto* collisionBox
      = test::requireShape<dynamics::CollisionAspect, dynamics::BoxShape>(
          *link, 0, 1);
  ASSERT_NE(collisionBox, nullptr);
  EXPECT_VECTOR_NEAR(
      collisionBox->getSize(), Eigen::Vector3d(0.1, 0.2, 0.3), 1e-12);
}

//==============================================================================
TEST(SdfWriter, RoundTripsConvertedSkelBoxFixtures)
{
  const std::vector<std::pair<std::string, std::string>> fixtures = {
      {"dart://sample/sdf/test/cube.sdf", "box"},
      {"dart://sample/sdf/test/shapes.sdf", "box"},
      {"dart://sample/sdf/test/test_shapes.sdf", "ground"},
  };

  for (const auto& [uri, bodyName] : fixtures) {
    SCOPED_TRACE(uri);
    const auto original = utils::SdfParser::readSkeleton(common::Uri(uri));
    ASSERT_NE(original, nullptr);
    ASSERT_EQ(original->getNumBodyNodes(), 1u);
    const auto* originalBody = test::requireBodyNode(*original, bodyName);
    ASSERT_NE(originalBody, nullptr);
    const auto* originalJoint = originalBody->getParentJoint();
    ASSERT_NE(originalJoint, nullptr);

    const auto writeResult
        = utils::SdfParser::tryWriteSkeletonToString(*original);
    ASSERT_TRUE(writeResult.isOk()) << writeResult.error().message;

    const auto path = writeTempSdf(writeResult.value(), bodyName);
    const auto reparsed = utils::SdfParser::readSkeleton(
        common::Uri::createFromPath(path.string()));
    std::filesystem::remove(path);

    ASSERT_NE(reparsed, nullptr);
    EXPECT_EQ(reparsed->getName(), original->getName());
    EXPECT_EQ(reparsed->isMobile(), original->isMobile());
    EXPECT_EQ(reparsed->getNumBodyNodes(), original->getNumBodyNodes());
    EXPECT_EQ(reparsed->getNumJoints(), original->getNumJoints());

    const auto* body = test::requireBodyNode(*reparsed, bodyName);
    ASSERT_NE(body, nullptr);
    const auto* joint = body->getParentJoint();
    ASSERT_NE(joint, nullptr);
    EXPECT_EQ(joint->getType(), originalJoint->getType());
    EXPECT_VECTOR_NEAR(
        joint->getTransformFromParentBodyNode().translation(),
        originalJoint->getTransformFromParentBodyNode().translation(),
        1e-12);
    EXPECT_VECTOR_NEAR(
        joint->getTransformFromChildBodyNode().translation(),
        originalJoint->getTransformFromChildBodyNode().translation(),
        1e-12);
    EXPECT_NEAR(body->getMass(), originalBody->getMass(), 1e-12);
    EXPECT_VECTOR_NEAR(body->getLocalCOM(), originalBody->getLocalCOM(), 1e-12);
    EXPECT_MATRIX_NEAR(
        body->getInertia().getMoment(),
        originalBody->getInertia().getMoment(),
        1e-12);

    const auto* originalVisualBox
        = test::requireShape<dynamics::VisualAspect, dynamics::BoxShape>(
            *originalBody, 0, 1);
    const auto* visualBox
        = test::requireShape<dynamics::VisualAspect, dynamics::BoxShape>(
            *body, 0, 1);
    ASSERT_NE(originalVisualBox, nullptr);
    ASSERT_NE(visualBox, nullptr);
    EXPECT_VECTOR_NEAR(
        visualBox->getSize(), originalVisualBox->getSize(), 1e-12);
    const auto* originalVisualShapeNode
        = originalBody->getShapeNodeWith<dynamics::VisualAspect>(0);
    const auto* visualShapeNode
        = body->getShapeNodeWith<dynamics::VisualAspect>(0);
    ASSERT_NE(originalVisualShapeNode, nullptr);
    ASSERT_NE(visualShapeNode, nullptr);
    EXPECT_MATRIX_NEAR(
        visualShapeNode->getRelativeTransform().matrix(),
        originalVisualShapeNode->getRelativeTransform().matrix(),
        1e-12);

    const auto* originalCollisionBox
        = test::requireShape<dynamics::CollisionAspect, dynamics::BoxShape>(
            *originalBody, 0, 1);
    const auto* collisionBox
        = test::requireShape<dynamics::CollisionAspect, dynamics::BoxShape>(
            *body, 0, 1);
    ASSERT_NE(originalCollisionBox, nullptr);
    ASSERT_NE(collisionBox, nullptr);
    EXPECT_VECTOR_NEAR(
        collisionBox->getSize(), originalCollisionBox->getSize(), 1e-12);
    const auto* originalCollisionShapeNode
        = originalBody->getShapeNodeWith<dynamics::CollisionAspect>(0);
    const auto* collisionShapeNode
        = body->getShapeNodeWith<dynamics::CollisionAspect>(0);
    ASSERT_NE(originalCollisionShapeNode, nullptr);
    ASSERT_NE(collisionShapeNode, nullptr);
    EXPECT_MATRIX_NEAR(
        collisionShapeNode->getRelativeTransform().matrix(),
        originalCollisionShapeNode->getRelativeTransform().matrix(),
        1e-12);
  }
}

//==============================================================================
TEST(SdfWriter, RoundTripsExistingTwoLinkRevoluteFixture)
{
  const auto original = utils::SdfParser::readSkeleton(
      common::Uri("dart://sample/sdf/test/two_link_revolute_model.sdf"));
  ASSERT_NE(original, nullptr);
  ASSERT_EQ(original->getNumBodyNodes(), 2u);
  ASSERT_EQ(original->getNumJoints(), 2u);
  const auto* originalBase = test::requireBodyNode(*original, "base");
  const auto* originalLink = test::requireBodyNode(*original, "link");
  ASSERT_NE(originalBase, nullptr);
  ASSERT_NE(originalLink, nullptr);
  const auto* originalRootJoint = originalBase->getParentJoint();
  ASSERT_NE(originalRootJoint, nullptr);
  const auto* originalHinge
      = test::requireJoint<dynamics::RevoluteJoint>(*original, "hinge");
  ASSERT_NE(originalHinge, nullptr);

  const auto writeResult
      = utils::SdfParser::tryWriteSkeletonToString(*original);
  ASSERT_TRUE(writeResult.isOk()) << writeResult.error().message;

  const auto path
      = writeTempSdf(writeResult.value(), "existing_two_link_revolute");
  const auto reparsed = utils::SdfParser::readSkeleton(
      common::Uri::createFromPath(path.string()));
  std::filesystem::remove(path);

  ASSERT_NE(reparsed, nullptr);
  EXPECT_EQ(reparsed->getName(), original->getName());
  EXPECT_EQ(reparsed->isMobile(), original->isMobile());
  EXPECT_EQ(reparsed->getNumBodyNodes(), original->getNumBodyNodes());
  EXPECT_EQ(reparsed->getNumJoints(), original->getNumJoints());

  const auto* base = test::requireBodyNode(*reparsed, "base");
  const auto* link = test::requireBodyNode(*reparsed, "link");
  ASSERT_NE(base, nullptr);
  ASSERT_NE(link, nullptr);
  expectBodyInertiaRoundTrips(*base, *originalBase, 1e-12);
  expectBodyInertiaRoundTrips(*link, *originalLink, 1e-12);

  const auto* rootJoint = base->getParentJoint();
  ASSERT_NE(rootJoint, nullptr);
  expectJointRoundTrips(*rootJoint, *originalRootJoint, 1e-12);

  const auto* hinge
      = test::requireJoint<dynamics::RevoluteJoint>(*reparsed, "hinge");
  ASSERT_NE(hinge, nullptr);
  expectJointRoundTrips(*hinge, *originalHinge, 1e-12);
  EXPECT_VECTOR_NEAR(hinge->getAxis(), originalHinge->getAxis(), 1e-12);

  const auto* originalBaseVisualBox
      = test::requireShape<dynamics::VisualAspect, dynamics::BoxShape>(
          *originalBase, 0, 1);
  const auto* baseVisualBox
      = test::requireShape<dynamics::VisualAspect, dynamics::BoxShape>(
          *base, 0, 1);
  ASSERT_NE(originalBaseVisualBox, nullptr);
  ASSERT_NE(baseVisualBox, nullptr);
  EXPECT_VECTOR_NEAR(
      baseVisualBox->getSize(), originalBaseVisualBox->getSize(), 1e-12);
  expectShapeNodePoseRoundTrips<dynamics::VisualAspect>(
      *base, *originalBase, 0, 1e-12);

  const auto* originalBaseCollisionBox
      = test::requireShape<dynamics::CollisionAspect, dynamics::BoxShape>(
          *originalBase, 0, 1);
  const auto* baseCollisionBox
      = test::requireShape<dynamics::CollisionAspect, dynamics::BoxShape>(
          *base, 0, 1);
  ASSERT_NE(originalBaseCollisionBox, nullptr);
  ASSERT_NE(baseCollisionBox, nullptr);
  EXPECT_VECTOR_NEAR(
      baseCollisionBox->getSize(), originalBaseCollisionBox->getSize(), 1e-12);
  expectShapeNodePoseRoundTrips<dynamics::CollisionAspect>(
      *base, *originalBase, 0, 1e-12);

  const auto* originalLinkVisualCylinder
      = test::requireShape<dynamics::VisualAspect, dynamics::CylinderShape>(
          *originalLink, 0, 1);
  const auto* linkVisualCylinder
      = test::requireShape<dynamics::VisualAspect, dynamics::CylinderShape>(
          *link, 0, 1);
  ASSERT_NE(originalLinkVisualCylinder, nullptr);
  ASSERT_NE(linkVisualCylinder, nullptr);
  EXPECT_NEAR(
      linkVisualCylinder->getRadius(),
      originalLinkVisualCylinder->getRadius(),
      1e-12);
  EXPECT_NEAR(
      linkVisualCylinder->getHeight(),
      originalLinkVisualCylinder->getHeight(),
      1e-12);
  expectShapeNodePoseRoundTrips<dynamics::VisualAspect>(
      *link, *originalLink, 0, 1e-12);

  const auto* originalLinkCollisionCylinder
      = test::requireShape<dynamics::CollisionAspect, dynamics::CylinderShape>(
          *originalLink, 0, 1);
  const auto* linkCollisionCylinder
      = test::requireShape<dynamics::CollisionAspect, dynamics::CylinderShape>(
          *link, 0, 1);
  ASSERT_NE(originalLinkCollisionCylinder, nullptr);
  ASSERT_NE(linkCollisionCylinder, nullptr);
  EXPECT_NEAR(
      linkCollisionCylinder->getRadius(),
      originalLinkCollisionCylinder->getRadius(),
      1e-12);
  EXPECT_NEAR(
      linkCollisionCylinder->getHeight(),
      originalLinkCollisionCylinder->getHeight(),
      1e-12);
  expectShapeNodePoseRoundTrips<dynamics::CollisionAspect>(
      *link, *originalLink, 0, 1e-12);
}

//==============================================================================
TEST(SdfWriter, RoundTripsExistingWorldRevoluteFixtures)
{
  const std::vector<std::pair<std::string, std::string>> fixtures = {
      {"dart://sample/sdf/test/issue1193_revolute_test.sdf",
       "issue1193_revolute"},
      {"dart://sample/sdf/test/issue1193_revolute_with_offset_test.sdf",
       "issue1193_revolute_with_offset"},
  };

  for (const auto& [uri, name] : fixtures) {
    SCOPED_TRACE(uri);
    const auto original = utils::SdfParser::readSkeleton(common::Uri(uri));
    ASSERT_NE(original, nullptr);
    ASSERT_EQ(original->getNumBodyNodes(), 2u);
    ASSERT_EQ(original->getNumJoints(), 2u);
    const auto* originalLink1 = test::requireBodyNode(*original, "link1");
    const auto* originalLink2 = test::requireBodyNode(*original, "link2");
    ASSERT_NE(originalLink1, nullptr);
    ASSERT_NE(originalLink2, nullptr);
    const auto* originalRootJoint = originalLink1->getParentJoint();
    ASSERT_NE(originalRootJoint, nullptr);
    const auto* originalRevolute
        = test::requireJoint<dynamics::RevoluteJoint>(*original, "revJoint");
    ASSERT_NE(originalRevolute, nullptr);

    const auto writeResult
        = utils::SdfParser::tryWriteSkeletonToString(*original);
    ASSERT_TRUE(writeResult.isOk()) << writeResult.error().message;

    const auto path = writeTempSdf(writeResult.value(), name);
    const auto reparsed = utils::SdfParser::readSkeleton(
        common::Uri::createFromPath(path.string()));
    std::filesystem::remove(path);

    ASSERT_NE(reparsed, nullptr);
    EXPECT_EQ(reparsed->getName(), original->getName());
    EXPECT_EQ(reparsed->isMobile(), original->isMobile());
    EXPECT_VECTOR_NEAR(reparsed->getGravity(), original->getGravity(), 1e-12);
    EXPECT_EQ(reparsed->getNumBodyNodes(), original->getNumBodyNodes());
    EXPECT_EQ(reparsed->getNumJoints(), original->getNumJoints());

    const auto* link1 = test::requireBodyNode(*reparsed, "link1");
    const auto* link2 = test::requireBodyNode(*reparsed, "link2");
    ASSERT_NE(link1, nullptr);
    ASSERT_NE(link2, nullptr);
    expectBodyInertiaRoundTrips(*link1, *originalLink1, 1e-12);
    expectBodyInertiaRoundTrips(*link2, *originalLink2, 1e-12);

    const auto* rootJoint = link1->getParentJoint();
    ASSERT_NE(rootJoint, nullptr);
    expectJointRoundTrips(*rootJoint, *originalRootJoint, 1e-12);

    const auto* revolute
        = test::requireJoint<dynamics::RevoluteJoint>(*reparsed, "revJoint");
    ASSERT_NE(revolute, nullptr);
    expectJointRoundTrips(*revolute, *originalRevolute, 1e-12);
    EXPECT_VECTOR_NEAR(revolute->getAxis(), originalRevolute->getAxis(), 1e-12);

    const auto* originalLink1Box
        = test::requireShape<dynamics::VisualAspect, dynamics::BoxShape>(
            *originalLink1, 0, 2);
    const auto* link1Box
        = test::requireShape<dynamics::VisualAspect, dynamics::BoxShape>(
            *link1, 0, 2);
    ASSERT_NE(originalLink1Box, nullptr);
    ASSERT_NE(link1Box, nullptr);
    EXPECT_VECTOR_NEAR(link1Box->getSize(), originalLink1Box->getSize(), 1e-12);
    expectShapeNodePoseRoundTrips<dynamics::VisualAspect>(
        *link1, *originalLink1, 0, 1e-12);

    const auto* originalLink1Sphere
        = test::requireShape<dynamics::VisualAspect, dynamics::SphereShape>(
            *originalLink1, 1, 2);
    const auto* link1Sphere
        = test::requireShape<dynamics::VisualAspect, dynamics::SphereShape>(
            *link1, 1, 2);
    ASSERT_NE(originalLink1Sphere, nullptr);
    ASSERT_NE(link1Sphere, nullptr);
    EXPECT_NEAR(
        link1Sphere->getRadius(), originalLink1Sphere->getRadius(), 1e-12);
    expectShapeNodePoseRoundTrips<dynamics::VisualAspect>(
        *link1, *originalLink1, 1, 1e-12);

    const auto* originalLink2Box
        = test::requireShape<dynamics::VisualAspect, dynamics::BoxShape>(
            *originalLink2, 0, 1);
    const auto* link2Box
        = test::requireShape<dynamics::VisualAspect, dynamics::BoxShape>(
            *link2, 0, 1);
    ASSERT_NE(originalLink2Box, nullptr);
    ASSERT_NE(link2Box, nullptr);
    EXPECT_VECTOR_NEAR(link2Box->getSize(), originalLink2Box->getSize(), 1e-12);
    expectShapeNodePoseRoundTrips<dynamics::VisualAspect>(
        *link2, *originalLink2, 0, 1e-12);

    EXPECT_EQ(
        link1->getNumShapeNodesWith<dynamics::CollisionAspect>(),
        originalLink1->getNumShapeNodesWith<dynamics::CollisionAspect>());
    EXPECT_EQ(
        link2->getNumShapeNodesWith<dynamics::CollisionAspect>(),
        originalLink2->getNumShapeNodesWith<dynamics::CollisionAspect>());
  }
}

//==============================================================================
TEST(SdfWriter, RoundTripsExistingForceTorqueWorldFixture)
{
  const auto original = utils::SdfParser::readSkeleton(
      common::Uri("dart://sample/sdf/test/force_torque_test.world"));
  ASSERT_NE(original, nullptr);
  ASSERT_EQ(original->getNumBodyNodes(), 2u);
  ASSERT_EQ(original->getNumJoints(), 2u);
  const auto* originalLink1 = test::requireBodyNode(*original, "link_1");
  const auto* originalLink2 = test::requireBodyNode(*original, "link_2");
  ASSERT_NE(originalLink1, nullptr);
  ASSERT_NE(originalLink2, nullptr);
  const auto* originalRoot
      = test::requireJoint<dynamics::RevoluteJoint>(*original, "joint_01");
  const auto* originalHinge
      = test::requireJoint<dynamics::RevoluteJoint>(*original, "joint_12");
  ASSERT_NE(originalRoot, nullptr);
  ASSERT_NE(originalHinge, nullptr);

  const auto writeResult
      = utils::SdfParser::tryWriteSkeletonToString(*original);
  ASSERT_TRUE(writeResult.isOk()) << writeResult.error().message;

  const auto path = writeTempSdf(writeResult.value(), "force_torque_world");
  const auto reparsed = utils::SdfParser::readSkeleton(
      common::Uri::createFromPath(path.string()));
  std::filesystem::remove(path);

  ASSERT_NE(reparsed, nullptr);
  EXPECT_EQ(reparsed->getName(), original->getName());
  EXPECT_EQ(reparsed->isMobile(), original->isMobile());
  EXPECT_VECTOR_NEAR(reparsed->getGravity(), original->getGravity(), 1e-12);
  EXPECT_EQ(reparsed->getNumBodyNodes(), original->getNumBodyNodes());
  EXPECT_EQ(reparsed->getNumJoints(), original->getNumJoints());

  const auto* link1 = test::requireBodyNode(*reparsed, "link_1");
  const auto* link2 = test::requireBodyNode(*reparsed, "link_2");
  ASSERT_NE(link1, nullptr);
  ASSERT_NE(link2, nullptr);
  expectBodyInertiaRoundTrips(*link1, *originalLink1, 1e-12);
  expectBodyInertiaRoundTrips(*link2, *originalLink2, 1e-12);

  const auto* root
      = test::requireJoint<dynamics::RevoluteJoint>(*reparsed, "joint_01");
  const auto* hinge
      = test::requireJoint<dynamics::RevoluteJoint>(*reparsed, "joint_12");
  ASSERT_NE(root, nullptr);
  ASSERT_NE(hinge, nullptr);
  expectJointRoundTrips(*root, *originalRoot, 1e-12);
  expectJointRoundTrips(*hinge, *originalHinge, 1e-12);
  EXPECT_VECTOR_NEAR(root->getAxis(), originalRoot->getAxis(), 1e-12);
  EXPECT_VECTOR_NEAR(hinge->getAxis(), originalHinge->getAxis(), 1e-12);

  const auto* originalLink1VisualSphere
      = test::requireShape<dynamics::VisualAspect, dynamics::SphereShape>(
          *originalLink1, 0, 1);
  const auto* link1VisualSphere
      = test::requireShape<dynamics::VisualAspect, dynamics::SphereShape>(
          *link1, 0, 1);
  ASSERT_NE(originalLink1VisualSphere, nullptr);
  ASSERT_NE(link1VisualSphere, nullptr);
  EXPECT_NEAR(
      link1VisualSphere->getRadius(),
      originalLink1VisualSphere->getRadius(),
      1e-12);
  expectShapeNodePoseRoundTrips<dynamics::VisualAspect>(
      *link1, *originalLink1, 0, 1e-12);

  const auto* originalLink1CollisionSphere
      = test::requireShape<dynamics::CollisionAspect, dynamics::SphereShape>(
          *originalLink1, 0, 1);
  const auto* link1CollisionSphere
      = test::requireShape<dynamics::CollisionAspect, dynamics::SphereShape>(
          *link1, 0, 1);
  ASSERT_NE(originalLink1CollisionSphere, nullptr);
  ASSERT_NE(link1CollisionSphere, nullptr);
  EXPECT_NEAR(
      link1CollisionSphere->getRadius(),
      originalLink1CollisionSphere->getRadius(),
      1e-12);
  expectShapeNodePoseRoundTrips<dynamics::CollisionAspect>(
      *link1, *originalLink1, 0, 1e-12);

  const auto* originalLink2VisualBox
      = test::requireShape<dynamics::VisualAspect, dynamics::BoxShape>(
          *originalLink2, 0, 1);
  const auto* link2VisualBox
      = test::requireShape<dynamics::VisualAspect, dynamics::BoxShape>(
          *link2, 0, 1);
  ASSERT_NE(originalLink2VisualBox, nullptr);
  ASSERT_NE(link2VisualBox, nullptr);
  EXPECT_VECTOR_NEAR(
      link2VisualBox->getSize(), originalLink2VisualBox->getSize(), 1e-12);
  expectShapeNodePoseRoundTrips<dynamics::VisualAspect>(
      *link2, *originalLink2, 0, 1e-12);

  const auto* originalLink2CollisionBox
      = test::requireShape<dynamics::CollisionAspect, dynamics::BoxShape>(
          *originalLink2, 0, 1);
  const auto* link2CollisionBox
      = test::requireShape<dynamics::CollisionAspect, dynamics::BoxShape>(
          *link2, 0, 1);
  ASSERT_NE(originalLink2CollisionBox, nullptr);
  ASSERT_NE(link2CollisionBox, nullptr);
  EXPECT_VECTOR_NEAR(
      link2CollisionBox->getSize(),
      originalLink2CollisionBox->getSize(),
      1e-12);
  expectShapeNodePoseRoundTrips<dynamics::CollisionAspect>(
      *link2, *originalLink2, 0, 1e-12);
}

//==============================================================================
TEST(SdfWriter, RoundTripsExistingForceTorqueChainWorldFixture)
{
  const auto original = utils::SdfParser::readSkeleton(
      common::Uri("dart://sample/sdf/test/force_torque_test2.world"));
  ASSERT_NE(original, nullptr);
  ASSERT_EQ(original->getNumBodyNodes(), 3u);
  ASSERT_EQ(original->getNumJoints(), 3u);

  const auto* originalLink1 = test::requireBodyNode(*original, "link1");
  const auto* originalLink2 = test::requireBodyNode(*original, "link2");
  const auto* originalLink3 = test::requireBodyNode(*original, "link3");
  ASSERT_NE(originalLink1, nullptr);
  ASSERT_NE(originalLink2, nullptr);
  ASSERT_NE(originalLink3, nullptr);

  const auto* originalRoot = originalLink1->getParentJoint();
  const auto* originalJoint1
      = test::requireJoint<dynamics::RevoluteJoint>(*original, "joint1");
  const auto* originalJoint2
      = test::requireJoint<dynamics::RevoluteJoint>(*original, "joint2");
  ASSERT_NE(originalRoot, nullptr);
  ASSERT_NE(originalJoint1, nullptr);
  ASSERT_NE(originalJoint2, nullptr);

  const auto writeResult
      = utils::SdfParser::tryWriteSkeletonToString(*original);
  ASSERT_TRUE(writeResult.isOk()) << writeResult.error().message;

  const auto path
      = writeTempSdf(writeResult.value(), "force_torque_chain_world");
  const auto reparsed = utils::SdfParser::readSkeleton(
      common::Uri::createFromPath(path.string()));
  std::filesystem::remove(path);

  ASSERT_NE(reparsed, nullptr);
  EXPECT_EQ(reparsed->getName(), original->getName());
  EXPECT_EQ(reparsed->isMobile(), original->isMobile());
  EXPECT_VECTOR_NEAR(reparsed->getGravity(), original->getGravity(), 1e-12);
  EXPECT_EQ(reparsed->getNumBodyNodes(), original->getNumBodyNodes());
  EXPECT_EQ(reparsed->getNumJoints(), original->getNumJoints());

  const auto* link1 = test::requireBodyNode(*reparsed, "link1");
  const auto* link2 = test::requireBodyNode(*reparsed, "link2");
  const auto* link3 = test::requireBodyNode(*reparsed, "link3");
  ASSERT_NE(link1, nullptr);
  ASSERT_NE(link2, nullptr);
  ASSERT_NE(link3, nullptr);
  expectBodyInertiaRoundTrips(*link1, *originalLink1, 1e-12);
  expectBodyInertiaRoundTrips(*link2, *originalLink2, 1e-12);
  expectBodyInertiaRoundTrips(*link3, *originalLink3, 1e-12);

  const auto* root = link1->getParentJoint();
  const auto* joint1
      = test::requireJoint<dynamics::RevoluteJoint>(*reparsed, "joint1");
  const auto* joint2
      = test::requireJoint<dynamics::RevoluteJoint>(*reparsed, "joint2");
  ASSERT_NE(root, nullptr);
  ASSERT_NE(joint1, nullptr);
  ASSERT_NE(joint2, nullptr);
  expectJointRoundTrips(*root, *originalRoot, 1e-12);
  expectJointRoundTrips(*joint1, *originalJoint1, 1e-12);
  expectJointRoundTrips(*joint2, *originalJoint2, 1e-12);
  EXPECT_VECTOR_NEAR(joint1->getAxis(), originalJoint1->getAxis(), 1e-12);
  EXPECT_VECTOR_NEAR(joint2->getAxis(), originalJoint2->getAxis(), 1e-12);

  for (const auto* body : {link1, link2, link3}) {
    ASSERT_NE(body, nullptr);
    const auto* originalBody = original->getBodyNode(body->getName());
    ASSERT_NE(originalBody, nullptr);
    SCOPED_TRACE(body->getName());
    expectBoxShapeRoundTrips<dynamics::VisualAspect>(
        *body, *originalBody, 0, 1, 1e-12);
    expectBoxShapeRoundTrips<dynamics::CollisionAspect>(
        *body, *originalBody, 0, 1, 1e-12);
  }
}

//==============================================================================
TEST(SdfWriter, RoundTripsJointAxisVelocityAndEffortLimits)
{
  dynamics::SkeletonPtr skeleton;
  auto* hinge = createLimitProbeJoint(skeleton, "axis_limit_writer");
  ASSERT_NE(hinge, nullptr);
  hinge->setVelocityLowerLimit(0, -3.3);
  hinge->setVelocityUpperLimit(0, 3.3);
  hinge->setForceLowerLimit(0, -5.5);
  hinge->setForceUpperLimit(0, 5.5);

  const auto writeResult
      = utils::SdfParser::tryWriteSkeletonToString(*skeleton);
  ASSERT_TRUE(writeResult.isOk()) << writeResult.error().message;

  sdf::Root sdfRoot;
  const auto sdfErrors = sdfRoot.LoadSdfString(writeResult.value());
  ASSERT_TRUE(sdfErrors.empty()) << sdfErrors.front().Message();
  ASSERT_NE(sdfRoot.Model(), nullptr);
  const auto* sdfJoint = sdfRoot.Model()->JointByName("limited_hinge");
  ASSERT_NE(sdfJoint, nullptr);
  ASSERT_NE(sdfJoint->Axis(0), nullptr);
  EXPECT_DOUBLE_EQ(sdfJoint->Axis(0)->MaxVelocity(), 3.3);
  EXPECT_DOUBLE_EQ(sdfJoint->Axis(0)->Effort(), 5.5);

  const auto path = writeTempSdf(writeResult.value(), "axis_limits");
  const auto reparsed = utils::SdfParser::readSkeleton(
      common::Uri::createFromPath(path.string()));
  std::filesystem::remove(path);

  ASSERT_NE(reparsed, nullptr);
  const auto* reparsedHinge
      = test::requireJoint<dynamics::RevoluteJoint>(*reparsed, "limited_hinge");
  ASSERT_NE(reparsedHinge, nullptr);
  EXPECT_DOUBLE_EQ(reparsedHinge->getVelocityLowerLimit(0), -3.3);
  EXPECT_DOUBLE_EQ(reparsedHinge->getVelocityUpperLimit(0), 3.3);
  EXPECT_DOUBLE_EQ(reparsedHinge->getForceLowerLimit(0), -5.5);
  EXPECT_DOUBLE_EQ(reparsedHinge->getForceUpperLimit(0), 5.5);
}

//==============================================================================
TEST(SdfWriter, WritesModernScrewThreadPitchForSdf110)
{
  const auto original = makeRoundTripSkeleton();

  utils::SdfParser::WriteOptions options;
  options.version = "1.10";
  const auto writeResult
      = utils::SdfParser::tryWriteSkeletonToString(*original, options);
  ASSERT_TRUE(writeResult.isOk()) << writeResult.error().message;
  EXPECT_NE(
      writeResult.value().find("<screw_thread_pitch>0.25</screw_thread_pitch>"),
      std::string::npos);
  EXPECT_EQ(writeResult.value().find("<thread_pitch>"), std::string::npos);

  const auto path = writeTempSdf(writeResult.value(), "screw_thread_pitch");
  const auto reparsed = utils::SdfParser::readSkeleton(
      common::Uri::createFromPath(path.string()));
  std::filesystem::remove(path);

  ASSERT_NE(reparsed, nullptr);
  const auto* screwJoint
      = test::requireJoint<dynamics::ScrewJoint>(*reparsed, "screw_drive");
  ASSERT_NE(screwJoint, nullptr);
  EXPECT_NEAR(screwJoint->getPitch(), 0.25, 1e-12);
}

//==============================================================================
TEST(SdfWriter, EmptySkeletonReturnsError)
{
  const auto skeleton = dynamics::Skeleton::create("empty");

  const auto result = utils::SdfParser::tryWriteSkeletonToString(*skeleton);
  ASSERT_TRUE(result.isErr());
  EXPECT_NE(result.error().message.find("empty Skeleton"), std::string::npos);
}

//==============================================================================
TEST(SdfWriter, EmptySdfVersionReturnsError)
{
  const auto skeleton = makeRoundTripSkeleton();

  utils::SdfParser::WriteOptions options;
  options.version.clear();
  const auto result
      = utils::SdfParser::tryWriteSkeletonToString(*skeleton, options);

  ASSERT_TRUE(result.isErr());
  EXPECT_NE(
      result.error().message.find("SDF version must not be empty"),
      std::string::npos);
}

//==============================================================================
TEST(SdfWriter, MalformedSdfVersionReturnsSdformatError)
{
  const auto skeleton = makeRoundTripSkeleton();

  utils::SdfParser::WriteOptions options;
  options.version = "not-a-version";
  const auto result
      = utils::SdfParser::tryWriteSkeletonToString(*skeleton, options);

  ASSERT_TRUE(result.isErr());
  EXPECT_NE(
      result.error().message.find(
          "sdformat rejected SDF version [not-a-version]"),
      std::string::npos);
}

//==============================================================================
TEST(SdfWriter, PreservesAbsoluteNonFileMeshUris)
{
  auto retriever = std::make_shared<MapResourceRetriever>();
  const std::string meshUri = "memory://pkg/meshes/writer_triangle.obj";
  addTriangleObj(*retriever, meshUri);

  auto skeleton = dynamics::Skeleton::create("mesh_uri_roundtrip");
  auto [joint, body]
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>();
  (void)joint;
  body->setName("mesh_body");
  body->createShapeNodeWith<dynamics::VisualAspect>(
      std::make_shared<dynamics::MeshShape>(
          Eigen::Vector3d(2.0, 3.0, 4.0),
          makeTriangleMesh(),
          common::Uri(meshUri),
          retriever),
      "memory_mesh_visual");

  const auto writeResult
      = utils::SdfParser::tryWriteSkeletonToString(*skeleton);
  ASSERT_TRUE(writeResult.isOk()) << writeResult.error().message;

  sdf::Root sdfRoot;
  const auto sdfErrors = sdfRoot.LoadSdfString(writeResult.value());
  ASSERT_TRUE(sdfErrors.empty()) << sdfErrors.front().Message();
  ASSERT_NE(sdfRoot.Model(), nullptr);
  const auto* sdfLink = sdfRoot.Model()->LinkByName("mesh_body");
  ASSERT_NE(sdfLink, nullptr);
  const auto* sdfVisual = sdfLink->VisualByName("memory_mesh_visual");
  ASSERT_NE(sdfVisual, nullptr);
  ASSERT_NE(sdfVisual->Geom(), nullptr);
  const auto* sdfMesh = sdfVisual->Geom()->MeshShape();
  ASSERT_NE(sdfMesh, nullptr);
  EXPECT_EQ(sdfMesh->Uri(), meshUri);

  const std::string modelUri = "memory://pkg/models/writer_roundtrip.sdf";
  retriever->add(modelUri, writeResult.value());

  const utils::SdfParser::Options options(retriever);
  const auto reparsed
      = utils::SdfParser::readSkeleton(common::Uri(modelUri), options);

  ASSERT_NE(reparsed, nullptr);
  const auto* reparsedBody = test::requireBodyNode(*reparsed, "mesh_body");
  ASSERT_NE(reparsedBody, nullptr);
  const auto* mesh
      = test::requireShape<dynamics::VisualAspect, dynamics::MeshShape>(
          *reparsedBody, 0, 1);
  ASSERT_NE(mesh, nullptr);
  EXPECT_EQ(mesh->getMeshUri2().toString(), meshUri);
  EXPECT_VECTOR_NEAR(mesh->getScale(), Eigen::Vector3d(2.0, 3.0, 4.0), 1e-12);
}

//==============================================================================
TEST(SdfWriter, RoundTripsMeshMaterialVariantsThroughUri)
{
  const common::Uri meshUri = common::Uri::createFromPath(
      dart::config::dataPath("gltf/pbr_multi_material.gltf"));
  auto retriever = std::make_shared<common::LocalResourceRetriever>();
  auto triMesh = loadTriMesh(meshUri, retriever);
  ASSERT_NE(triMesh, nullptr);

  auto skeleton = dynamics::Skeleton::create("mesh_material_roundtrip");
  auto [joint, body]
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>();
  (void)joint;
  body->setName("mesh_body");
  auto mesh = std::make_shared<dynamics::MeshShape>(
      Eigen::Vector3d::Ones(), std::move(triMesh), meshUri, retriever);
  ASSERT_GE(mesh->getMaterials().size(), 2u);
  ASSERT_GE(mesh->getSubMeshRanges().size(), 2u);
  body->createShapeNodeWith<dynamics::VisualAspect>(
      std::move(mesh), "multi_material_mesh_visual");

  const auto writeResult
      = utils::SdfParser::tryWriteSkeletonToString(*skeleton);
  ASSERT_TRUE(writeResult.isOk()) << writeResult.error().message;

  sdf::Root sdfRoot;
  const auto sdfErrors = sdfRoot.LoadSdfString(writeResult.value());
  ASSERT_TRUE(sdfErrors.empty()) << sdfErrors.front().Message();
  ASSERT_NE(sdfRoot.Model(), nullptr);
  const auto* sdfLink = sdfRoot.Model()->LinkByName("mesh_body");
  ASSERT_NE(sdfLink, nullptr);
  const auto* sdfVisual = sdfLink->VisualByName("multi_material_mesh_visual");
  ASSERT_NE(sdfVisual, nullptr);
  ASSERT_NE(sdfVisual->Geom(), nullptr);
  const auto* sdfMesh = sdfVisual->Geom()->MeshShape();
  ASSERT_NE(sdfMesh, nullptr);
  EXPECT_EQ(sdfMesh->Uri(), meshUri.toString());

  const auto path
      = writeTempSdf(writeResult.value(), "mesh_material_roundtrip");
  const auto reparsed = utils::SdfParser::readSkeleton(
      common::Uri::createFromPath(path.string()));
  std::filesystem::remove(path);

  ASSERT_NE(reparsed, nullptr);
  const auto* reparsedBody = test::requireBodyNode(*reparsed, "mesh_body");
  ASSERT_NE(reparsedBody, nullptr);
  const auto* reparsedMesh
      = test::requireShape<dynamics::VisualAspect, dynamics::MeshShape>(
          *reparsedBody, 0, 1);
  ASSERT_NE(reparsedMesh, nullptr);

  ASSERT_GE(reparsedMesh->getSubMeshRanges().size(), 2u);
  std::set<unsigned int> materialIndices;
  for (const auto& range : reparsedMesh->getSubMeshRanges()) {
    materialIndices.insert(range.materialIndex);
  }
  EXPECT_TRUE(materialIndices.contains(0u));
  EXPECT_TRUE(materialIndices.contains(1u));

  ASSERT_GE(reparsedMesh->getMaterials().size(), 2u);
  const auto* opaqueMaterial = reparsedMesh->getMaterial(0u);
  const auto* transparentMaterial = reparsedMesh->getMaterial(1u);
  ASSERT_NE(opaqueMaterial, nullptr);
  ASSERT_NE(transparentMaterial, nullptr);

  EXPECT_TRUE(opaqueMaterial->diffuse.isApprox(
      Eigen::Vector4f(0.9f, 0.18f, 0.12f, 1.0f), 1e-6f));
  EXPECT_FLOAT_EQ(opaqueMaterial->metallicFactor, 0.25f);
  EXPECT_FLOAT_EQ(opaqueMaterial->roughnessFactor, 0.45f);
  EXPECT_NE(
      opaqueMaterial->baseColorTexturePath.find("block.png"),
      std::string::npos);

  EXPECT_TRUE(transparentMaterial->diffuse.isApprox(
      Eigen::Vector4f(0.1f, 0.5f, 0.9f, 0.6f), 1e-6f));
  EXPECT_FLOAT_EQ(transparentMaterial->metallicFactor, 0.8f);
  EXPECT_FLOAT_EQ(transparentMaterial->roughnessFactor, 0.2f);
  EXPECT_NE(
      transparentMaterial->baseColorTexturePath.find("block_hidden.png"),
      std::string::npos);
}

//==============================================================================
TEST(SdfWriter, IncludeOptionsControlVisualAndCollisionEntries)
{
  auto skeleton = dynamics::Skeleton::create("writer_options");
  auto [joint, body]
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>();
  (void)joint;
  body->setName("body");
  body->createShapeNodeWith<dynamics::VisualAspect>(
      std::make_shared<dynamics::BoxShape>(Eigen::Vector3d::Ones()),
      "visible_box");
  body->createShapeNodeWith<dynamics::CollisionAspect>(
      std::make_shared<dynamics::SphereShape>(0.5), "collision_sphere");

  utils::SdfParser::WriteOptions options;
  options.includeVisuals = false;

  const auto noVisuals
      = utils::SdfParser::tryWriteSkeletonToString(*skeleton, options);
  ASSERT_TRUE(noVisuals.isOk()) << noVisuals.error().message;

  sdf::Root noVisualsRoot;
  const auto noVisualsErrors = noVisualsRoot.LoadSdfString(noVisuals.value());
  ASSERT_TRUE(noVisualsErrors.empty()) << noVisualsErrors.front().Message();
  ASSERT_NE(noVisualsRoot.Model(), nullptr);
  const auto* noVisualsLink = noVisualsRoot.Model()->LinkByName("body");
  ASSERT_NE(noVisualsLink, nullptr);
  EXPECT_EQ(noVisualsLink->VisualCount(), 0u);
  ASSERT_EQ(noVisualsLink->CollisionCount(), 1u);
  ASSERT_NE(noVisualsLink->CollisionByName("collision_sphere"), nullptr);

  options.includeVisuals = true;
  options.includeCollisions = false;

  const auto noCollisions
      = utils::SdfParser::tryWriteSkeletonToString(*skeleton, options);
  ASSERT_TRUE(noCollisions.isOk()) << noCollisions.error().message;

  sdf::Root noCollisionsRoot;
  const auto noCollisionsErrors
      = noCollisionsRoot.LoadSdfString(noCollisions.value());
  ASSERT_TRUE(noCollisionsErrors.empty())
      << noCollisionsErrors.front().Message();
  ASSERT_NE(noCollisionsRoot.Model(), nullptr);
  const auto* noCollisionsLink = noCollisionsRoot.Model()->LinkByName("body");
  ASSERT_NE(noCollisionsLink, nullptr);
  ASSERT_EQ(noCollisionsLink->VisualCount(), 1u);
  ASSERT_NE(noCollisionsLink->VisualByName("visible_box"), nullptr);
  EXPECT_EQ(noCollisionsLink->CollisionCount(), 0u);
}

//==============================================================================
TEST(SdfWriter, RoundTripsModelSelfCollision)
{
  auto skeleton = dynamics::Skeleton::create("self_collision_writer");
  skeleton->enableSelfCollisionCheck();

  auto [joint, body]
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>();
  (void)joint;
  body->setName("body");
  body->createShapeNodeWith<dynamics::CollisionAspect>(
      std::make_shared<dynamics::BoxShape>(Eigen::Vector3d::Ones()),
      "body_collision");

  const auto writeResult
      = utils::SdfParser::tryWriteSkeletonToString(*skeleton);
  ASSERT_TRUE(writeResult.isOk()) << writeResult.error().message;

  sdf::Root sdfRoot;
  const auto sdfErrors = sdfRoot.LoadSdfString(writeResult.value());
  ASSERT_TRUE(sdfErrors.empty()) << sdfErrors.front().Message();
  ASSERT_NE(sdfRoot.Model(), nullptr);
  EXPECT_TRUE(sdfRoot.Model()->SelfCollide());

  const auto path = writeTempSdf(writeResult.value(), "self_collision");
  const auto reparsed = utils::SdfParser::readSkeleton(
      common::Uri::createFromPath(path.string()));
  std::filesystem::remove(path);

  ASSERT_NE(reparsed, nullptr);
  EXPECT_TRUE(reparsed->getSelfCollisionCheck());
}

//==============================================================================
TEST(SdfWriter, RoundTripsModelStaticStateWithImplicitRoot)
{
  auto skeleton = dynamics::Skeleton::create("static_model_writer");
  skeleton->setMobile(false);

  dynamics::FreeJoint::Properties rootProperties;
  rootProperties.mName = "root";
  rootProperties.mT_ParentBodyToJoint.translation()
      = Eigen::Vector3d(0.2, -0.1, 0.4);

  dynamics::BodyNode::Properties bodyProperties;
  bodyProperties.mName = "body";
  bodyProperties.mInertia.setMass(1.0);
  bodyProperties.mInertia.setMoment(Eigen::Matrix3d::Identity());

  auto [rootJoint, body]
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>(
          nullptr, rootProperties, bodyProperties);
  (void)rootJoint;
  body->createShapeNodeWith<dynamics::CollisionAspect>(
      std::make_shared<dynamics::BoxShape>(Eigen::Vector3d(0.2, 0.3, 0.4)),
      "body_collision");

  const auto writeResult
      = utils::SdfParser::tryWriteSkeletonToString(*skeleton);
  ASSERT_TRUE(writeResult.isOk()) << writeResult.error().message;

  sdf::Root sdfRoot;
  const auto sdfErrors = sdfRoot.LoadSdfString(writeResult.value());
  ASSERT_TRUE(sdfErrors.empty()) << sdfErrors.front().Message();
  ASSERT_NE(sdfRoot.Model(), nullptr);
  EXPECT_TRUE(sdfRoot.Model()->Static());
  EXPECT_EQ(sdfRoot.Model()->LinkCount(), 1u);
  EXPECT_EQ(sdfRoot.Model()->JointCount(), 0u);

  const auto path = writeTempSdf(writeResult.value(), "static_model");
  const auto reparsed = utils::SdfParser::readSkeleton(
      common::Uri::createFromPath(path.string()));
  std::filesystem::remove(path);

  ASSERT_NE(reparsed, nullptr);
  EXPECT_FALSE(reparsed->isMobile());
  EXPECT_EQ(reparsed->getNumBodyNodes(), 1u);
  EXPECT_EQ(reparsed->getNumJoints(), 1u);

  const auto* reparsedBody = test::requireBodyNode(*reparsed, "body");
  ASSERT_NE(reparsedBody, nullptr);
  const auto* reparsedRoot = dynamic_cast<const dynamics::FreeJoint*>(
      reparsedBody->getParentJoint());
  ASSERT_NE(reparsedRoot, nullptr);
  test::expectJointTopology(*reparsedRoot, nullptr, reparsedBody);
  EXPECT_VECTOR_NEAR(
      reparsedRoot->getTransformFromParentBodyNode().translation(),
      Eigen::Vector3d(0.2, -0.1, 0.4),
      1e-12);

  const auto* reparsedBox
      = test::requireShape<dynamics::CollisionAspect, dynamics::BoxShape>(
          *reparsedBody, 0, 1);
  ASSERT_NE(reparsedBox, nullptr);
  EXPECT_VECTOR_NEAR(
      reparsedBox->getSize(), Eigen::Vector3d(0.2, 0.3, 0.4), 1e-12);
}

//==============================================================================
TEST(SdfWriter, RoundTripsNonDefaultSkeletonGravityThroughWorld)
{
  auto skeleton = dynamics::Skeleton::create("gravity_writer");
  skeleton->setGravity(Eigen::Vector3d(1.0, 2.0, -3.0));

  auto [joint, body]
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>();
  (void)joint;
  body->setName("body");
  body->createShapeNodeWith<dynamics::CollisionAspect>(
      std::make_shared<dynamics::BoxShape>(Eigen::Vector3d::Ones()),
      "body_collision");

  const auto writeResult
      = utils::SdfParser::tryWriteSkeletonToString(*skeleton);
  ASSERT_TRUE(writeResult.isOk()) << writeResult.error().message;

  sdf::Root sdfRoot;
  const auto sdfErrors = sdfRoot.LoadSdfString(writeResult.value());
  ASSERT_TRUE(sdfErrors.empty()) << sdfErrors.front().Message();
  ASSERT_EQ(sdfRoot.WorldCount(), 1u);
  const auto* sdfWorld = sdfRoot.WorldByIndex(0);
  ASSERT_NE(sdfWorld, nullptr);
  EXPECT_DOUBLE_EQ(sdfWorld->Gravity().X(), 1.0);
  EXPECT_DOUBLE_EQ(sdfWorld->Gravity().Y(), 2.0);
  EXPECT_DOUBLE_EQ(sdfWorld->Gravity().Z(), -3.0);
  ASSERT_EQ(sdfWorld->ModelCount(), 1u);
  ASSERT_NE(sdfWorld->ModelByIndex(0), nullptr);
  EXPECT_EQ(sdfWorld->ModelByIndex(0)->Name(), "gravity_writer");

  const auto path = writeTempSdf(writeResult.value(), "gravity_world");
  const auto reparsed = utils::SdfParser::readSkeleton(
      common::Uri::createFromPath(path.string()));
  std::filesystem::remove(path);

  ASSERT_NE(reparsed, nullptr);
  EXPECT_TRUE(reparsed->getGravity().isApprox(Eigen::Vector3d(1.0, 2.0, -3.0)));
}

//==============================================================================
TEST(SdfWriter, RoundTripsVisualShadowAndHiddenState)
{
  auto skeleton = dynamics::Skeleton::create("visual_metadata_writer");
  auto [joint, body]
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>();
  (void)joint;
  body->setName("body");

  auto* shapeNode = body->createShapeNodeWith<dynamics::VisualAspect>(
      std::make_shared<dynamics::BoxShape>(Eigen::Vector3d::Ones()),
      "quiet_visual");
  auto* visualAspect = shapeNode->getVisualAspect();
  ASSERT_NE(visualAspect, nullptr);
  visualAspect->setShadowed(false);
  visualAspect->hide();

  const auto writeResult
      = utils::SdfParser::tryWriteSkeletonToString(*skeleton);
  ASSERT_TRUE(writeResult.isOk()) << writeResult.error().message;

  sdf::Root sdfRoot;
  const auto sdfErrors = sdfRoot.LoadSdfString(writeResult.value());
  ASSERT_TRUE(sdfErrors.empty()) << sdfErrors.front().Message();
  ASSERT_NE(sdfRoot.Model(), nullptr);
  const auto* sdfLink = sdfRoot.Model()->LinkByName("body");
  ASSERT_NE(sdfLink, nullptr);
  const auto* sdfVisual = sdfLink->VisualByName("quiet_visual");
  ASSERT_NE(sdfVisual, nullptr);
  EXPECT_FALSE(sdfVisual->CastShadows());
  EXPECT_EQ(sdfVisual->VisibilityFlags(), 0u);

  const auto path = writeTempSdf(writeResult.value(), "visual_metadata");
  const auto reparsed = utils::SdfParser::readSkeleton(
      common::Uri::createFromPath(path.string()));
  std::filesystem::remove(path);

  ASSERT_NE(reparsed, nullptr);
  const auto* reparsedBody = test::requireBodyNode(*reparsed, "body");
  ASSERT_NE(reparsedBody, nullptr);
  const auto* reparsedShapeNode
      = reparsedBody->getShapeNodeWith<dynamics::VisualAspect>(0);
  ASSERT_NE(reparsedShapeNode, nullptr);
  const auto* reparsedVisual = reparsedShapeNode->getVisualAspect();
  ASSERT_NE(reparsedVisual, nullptr);
  EXPECT_FALSE(reparsedVisual->getShadowed());
  EXPECT_TRUE(reparsedVisual->isHidden());
}

//==============================================================================
TEST(SdfWriter, RoundTripsVisualTransparency)
{
  auto skeleton = dynamics::Skeleton::create("visual_transparency_writer");
  auto [joint, body]
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>();
  (void)joint;
  body->setName("body");

  auto* shapeNode = body->createShapeNodeWith<dynamics::VisualAspect>(
      std::make_shared<dynamics::BoxShape>(Eigen::Vector3d::Ones()),
      "transparent_visual");
  auto* visualAspect = shapeNode->getVisualAspect();
  ASSERT_NE(visualAspect, nullptr);
  visualAspect->setAlpha(0.75);

  const auto writeResult
      = utils::SdfParser::tryWriteSkeletonToString(*skeleton);
  ASSERT_TRUE(writeResult.isOk()) << writeResult.error().message;

  sdf::Root sdfRoot;
  const auto sdfErrors = sdfRoot.LoadSdfString(writeResult.value());
  ASSERT_TRUE(sdfErrors.empty()) << sdfErrors.front().Message();
  ASSERT_NE(sdfRoot.Model(), nullptr);
  const auto* sdfLink = sdfRoot.Model()->LinkByName("body");
  ASSERT_NE(sdfLink, nullptr);
  const auto* sdfVisual = sdfLink->VisualByName("transparent_visual");
  ASSERT_NE(sdfVisual, nullptr);
  EXPECT_NEAR(sdfVisual->Transparency(), 0.25f, 1e-6);

  const auto path = writeTempSdf(writeResult.value(), "visual_transparency");
  const auto reparsed = utils::SdfParser::readSkeleton(
      common::Uri::createFromPath(path.string()));
  std::filesystem::remove(path);

  ASSERT_NE(reparsed, nullptr);
  const auto* reparsedBody = test::requireBodyNode(*reparsed, "body");
  ASSERT_NE(reparsedBody, nullptr);
  const auto* reparsedShapeNode
      = reparsedBody->getShapeNodeWith<dynamics::VisualAspect>(0);
  ASSERT_NE(reparsedShapeNode, nullptr);
  const auto* reparsedVisual = reparsedShapeNode->getVisualAspect();
  ASSERT_NE(reparsedVisual, nullptr);
  EXPECT_TRUE(reparsedVisual->usesDefaultColor());
  EXPECT_NEAR(reparsedVisual->getAlpha(), 0.75, 1e-12);
}

//==============================================================================
TEST(SdfWriter, RoundTripsCollisionSurfaceOdeFriction)
{
  auto skeleton = dynamics::Skeleton::create("surface_writer");
  auto [joint, body]
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>();
  (void)joint;
  body->setName("body");

  auto* shapeNode = body->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(
      std::make_shared<dynamics::BoxShape>(Eigen::Vector3d::Ones()),
      "surface_box");
  auto* dynamicsAspect = shapeNode->getDynamicsAspect();
  ASSERT_NE(dynamicsAspect, nullptr);
  dynamicsAspect->setPrimaryFrictionCoeff(0.25);
  dynamicsAspect->setSecondaryFrictionCoeff(0.75);
  dynamicsAspect->setPrimarySlipCompliance(0.015);
  dynamicsAspect->setSecondarySlipCompliance(0.025);
  dynamicsAspect->setFirstFrictionDirection(Eigen::Vector3d::UnitY());
  dynamicsAspect->setFirstFrictionDirectionFrame(shapeNode);

  const auto writeResult
      = utils::SdfParser::tryWriteSkeletonToString(*skeleton);
  ASSERT_TRUE(writeResult.isOk()) << writeResult.error().message;

  sdf::Root sdfRoot;
  const auto sdfErrors = sdfRoot.LoadSdfString(writeResult.value());
  ASSERT_TRUE(sdfErrors.empty()) << sdfErrors.front().Message();
  ASSERT_NE(sdfRoot.Model(), nullptr);
  const auto* sdfLink = sdfRoot.Model()->LinkByName("body");
  ASSERT_NE(sdfLink, nullptr);
  const auto* sdfCollision = sdfLink->CollisionByName("surface_box");
  ASSERT_NE(sdfCollision, nullptr);
  ASSERT_NE(sdfCollision->Surface(), nullptr);
  ASSERT_NE(sdfCollision->Surface()->Friction(), nullptr);
  ASSERT_NE(sdfCollision->Surface()->Friction()->ODE(), nullptr);
  const auto* ode = sdfCollision->Surface()->Friction()->ODE();
  EXPECT_DOUBLE_EQ(ode->Mu(), 0.25);
  EXPECT_DOUBLE_EQ(ode->Mu2(), 0.75);
  EXPECT_TRUE(
      Eigen::Vector3d(ode->Fdir1().X(), ode->Fdir1().Y(), ode->Fdir1().Z())
          .isApprox(Eigen::Vector3d::UnitY()));
  EXPECT_DOUBLE_EQ(ode->Slip1(), 0.015);
  EXPECT_DOUBLE_EQ(ode->Slip2(), 0.025);

  const auto path = writeTempSdf(writeResult.value(), "surface_friction");
  const auto reparsed = utils::SdfParser::readSkeleton(
      common::Uri::createFromPath(path.string()));
  std::filesystem::remove(path);

  ASSERT_NE(reparsed, nullptr);
  const auto* reparsedBody = test::requireBodyNode(*reparsed, "body");
  ASSERT_NE(reparsedBody, nullptr);
  const auto* reparsedShapeNode
      = reparsedBody->getShapeNodeWith<dynamics::CollisionAspect>(0);
  ASSERT_NE(reparsedShapeNode, nullptr);
  const auto* reparsedDynamics = reparsedShapeNode->getDynamicsAspect();
  ASSERT_NE(reparsedDynamics, nullptr);
  EXPECT_DOUBLE_EQ(reparsedDynamics->getPrimaryFrictionCoeff(), 0.25);
  EXPECT_DOUBLE_EQ(reparsedDynamics->getSecondaryFrictionCoeff(), 0.75);
  EXPECT_TRUE(reparsedDynamics->getFirstFrictionDirection().isApprox(
      Eigen::Vector3d::UnitY()));
  EXPECT_EQ(reparsedDynamics->getFirstFrictionDirectionFrame(), nullptr);
  EXPECT_DOUBLE_EQ(reparsedDynamics->getPrimarySlipCompliance(), 0.015);
  EXPECT_DOUBLE_EQ(reparsedDynamics->getSecondarySlipCompliance(), 0.025);
}

//==============================================================================
TEST(SdfWriter, NonFiniteCollisionSurfaceFrictionDirectionReturnsError)
{
  auto skeleton = dynamics::Skeleton::create("bad_surface_friction_direction");
  auto [joint, body]
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>();
  (void)joint;
  body->setName("body");

  auto* shapeNode = body->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(
      std::make_shared<dynamics::BoxShape>(Eigen::Vector3d::Ones()),
      "bad_surface");
  auto* dynamicsAspect = shapeNode->getDynamicsAspect();
  ASSERT_NE(dynamicsAspect, nullptr);
  dynamicsAspect->setFirstFrictionDirection(
      Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0));
  dynamicsAspect->setFirstFrictionDirectionFrame(shapeNode);

  const auto result = utils::SdfParser::tryWriteSkeletonToString(*skeleton);
  ASSERT_TRUE(result.isErr());
  EXPECT_NE(
      result.error().message.find("non-finite friction direction"),
      std::string::npos);
}

//==============================================================================
TEST(SdfWriter, RoundTripsCollisionSurfaceContactBitmask)
{
  auto skeleton = dynamics::Skeleton::create("collision_bitmask_writer");
  auto [joint, body]
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>();
  (void)joint;
  body->setName("body");

  auto* shapeNode = body->createShapeNodeWith<dynamics::CollisionAspect>(
      std::make_shared<dynamics::SphereShape>(0.2), "disabled_surface");
  auto* collisionAspect = shapeNode->getCollisionAspect();
  ASSERT_NE(collisionAspect, nullptr);
  collisionAspect->setCollidable(false);

  const auto writeResult
      = utils::SdfParser::tryWriteSkeletonToString(*skeleton);
  ASSERT_TRUE(writeResult.isOk()) << writeResult.error().message;

  sdf::Root sdfRoot;
  const auto sdfErrors = sdfRoot.LoadSdfString(writeResult.value());
  ASSERT_TRUE(sdfErrors.empty()) << sdfErrors.front().Message();
  ASSERT_NE(sdfRoot.Model(), nullptr);
  const auto* sdfLink = sdfRoot.Model()->LinkByName("body");
  ASSERT_NE(sdfLink, nullptr);
  const auto* sdfCollision = sdfLink->CollisionByName("disabled_surface");
  ASSERT_NE(sdfCollision, nullptr);
  ASSERT_NE(sdfCollision->Surface(), nullptr);
  ASSERT_NE(sdfCollision->Surface()->Contact(), nullptr);
  EXPECT_EQ(sdfCollision->Surface()->Contact()->CollideBitmask(), 0u);

  const auto path = writeTempSdf(writeResult.value(), "collision_bitmask");
  const auto reparsed = utils::SdfParser::readSkeleton(
      common::Uri::createFromPath(path.string()));
  std::filesystem::remove(path);

  ASSERT_NE(reparsed, nullptr);
  const auto* reparsedBody = test::requireBodyNode(*reparsed, "body");
  ASSERT_NE(reparsedBody, nullptr);
  const auto* reparsedShapeNode
      = reparsedBody->getShapeNodeWith<dynamics::CollisionAspect>(0);
  ASSERT_NE(reparsedShapeNode, nullptr);
  const auto* reparsedCollision = reparsedShapeNode->getCollisionAspect();
  ASSERT_NE(reparsedCollision, nullptr);
  EXPECT_FALSE(reparsedCollision->isCollidable());
}

//==============================================================================
TEST(SdfWriter, RoundTripsBodyLevelCollidableAsCollisionBitmask)
{
  auto skeleton = dynamics::Skeleton::create("body_collision_bitmask_writer");
  auto [joint, body]
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>();
  (void)joint;
  body->setName("body");
  body->setCollidable(false);

  body->createShapeNodeWith<dynamics::CollisionAspect>(
      std::make_shared<dynamics::SphereShape>(0.2), "body_disabled_surface");

  const auto writeResult
      = utils::SdfParser::tryWriteSkeletonToString(*skeleton);
  ASSERT_TRUE(writeResult.isOk()) << writeResult.error().message;

  sdf::Root sdfRoot;
  const auto sdfErrors = sdfRoot.LoadSdfString(writeResult.value());
  ASSERT_TRUE(sdfErrors.empty()) << sdfErrors.front().Message();
  ASSERT_NE(sdfRoot.Model(), nullptr);
  const auto* sdfLink = sdfRoot.Model()->LinkByName("body");
  ASSERT_NE(sdfLink, nullptr);
  const auto* sdfCollision = sdfLink->CollisionByName("body_disabled_surface");
  ASSERT_NE(sdfCollision, nullptr);
  ASSERT_NE(sdfCollision->Surface(), nullptr);
  ASSERT_NE(sdfCollision->Surface()->Contact(), nullptr);
  EXPECT_EQ(sdfCollision->Surface()->Contact()->CollideBitmask(), 0u);

  const auto path = writeTempSdf(writeResult.value(), "body_collision_bitmask");
  const auto reparsed = utils::SdfParser::readSkeleton(
      common::Uri::createFromPath(path.string()));
  std::filesystem::remove(path);

  ASSERT_NE(reparsed, nullptr);
  const auto* reparsedBody = test::requireBodyNode(*reparsed, "body");
  ASSERT_NE(reparsedBody, nullptr);
  const auto* reparsedShapeNode
      = reparsedBody->getShapeNodeWith<dynamics::CollisionAspect>(0);
  ASSERT_NE(reparsedShapeNode, nullptr);
  const auto* reparsedCollision = reparsedShapeNode->getCollisionAspect();
  ASSERT_NE(reparsedCollision, nullptr);
  EXPECT_FALSE(reparsedCollision->isCollidable());
}

//==============================================================================
TEST(SdfWriter, RoundTripsCollisionSurfaceBounceRestitution)
{
  auto skeleton = dynamics::Skeleton::create("collision_bounce_writer");
  auto [joint, body]
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>();
  (void)joint;
  body->setName("body");

  auto* shapeNode = body->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(
      std::make_shared<dynamics::SphereShape>(0.2), "bouncy_surface");
  auto* dynamicsAspect = shapeNode->getDynamicsAspect();
  ASSERT_NE(dynamicsAspect, nullptr);
  dynamicsAspect->setRestitutionCoeff(0.42);

  const auto writeResult
      = utils::SdfParser::tryWriteSkeletonToString(*skeleton);
  ASSERT_TRUE(writeResult.isOk()) << writeResult.error().message;

  sdf::Root sdfRoot;
  const auto sdfErrors = sdfRoot.LoadSdfString(writeResult.value());
  ASSERT_TRUE(sdfErrors.empty()) << sdfErrors.front().Message();
  ASSERT_NE(sdfRoot.Model(), nullptr);
  const auto* sdfLink = sdfRoot.Model()->LinkByName("body");
  ASSERT_NE(sdfLink, nullptr);
  const auto* sdfCollision = sdfLink->CollisionByName("bouncy_surface");
  ASSERT_NE(sdfCollision, nullptr);
  ASSERT_NE(sdfCollision->Surface(), nullptr);
  const auto surfaceElement = sdfCollision->Surface()->Element();
  ASSERT_NE(surfaceElement, nullptr);
  const auto bounceElement = getSdfChildElement(surfaceElement, "bounce");
  ASSERT_NE(bounceElement, nullptr);
  EXPECT_DOUBLE_EQ(
      getSdfDoubleElement(bounceElement, "restitution_coefficient"), 0.42);
  EXPECT_DOUBLE_EQ(getSdfDoubleElement(bounceElement, "threshold"), 0.0);

  const auto path = writeTempSdf(writeResult.value(), "collision_bounce");
  const auto reparsed = utils::SdfParser::readSkeleton(
      common::Uri::createFromPath(path.string()));
  std::filesystem::remove(path);

  ASSERT_NE(reparsed, nullptr);
  const auto* reparsedBody = test::requireBodyNode(*reparsed, "body");
  ASSERT_NE(reparsedBody, nullptr);
  const auto* reparsedShapeNode
      = reparsedBody->getShapeNodeWith<dynamics::CollisionAspect>(0);
  ASSERT_NE(reparsedShapeNode, nullptr);
  const auto* reparsedDynamics = reparsedShapeNode->getDynamicsAspect();
  ASSERT_NE(reparsedDynamics, nullptr);
  EXPECT_DOUBLE_EQ(reparsedDynamics->getRestitutionCoeff(), 0.42);
}

//==============================================================================
TEST(SdfWriter, RoundTripsContinuousRevoluteJoint)
{
  auto skeleton = dynamics::Skeleton::create("continuous_writer");
  auto [rootJoint, base]
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>();
  (void)rootJoint;
  base->setName("base");

  dynamics::RevoluteJoint::Properties hingeProperties;
  hingeProperties.mName = "continuous_hinge";
  hingeProperties.mAxis = Eigen::Vector3d::UnitZ();
  hingeProperties.mPositionLowerLimits[0]
      = -std::numeric_limits<double>::infinity();
  hingeProperties.mPositionUpperLimits[0]
      = std::numeric_limits<double>::infinity();

  dynamics::BodyNode::Properties tipProperties;
  tipProperties.mName = "tip";
  auto [hinge, tip]
      = skeleton->createJointAndBodyNodePair<dynamics::RevoluteJoint>(
          base, hingeProperties, tipProperties);
  (void)hinge;
  (void)tip;

  const auto writeResult
      = utils::SdfParser::tryWriteSkeletonToString(*skeleton);
  ASSERT_TRUE(writeResult.isOk()) << writeResult.error().message;

  sdf::Root sdfRoot;
  const auto sdfErrors = sdfRoot.LoadSdfString(writeResult.value());
  ASSERT_TRUE(sdfErrors.empty()) << sdfErrors.front().Message();
  ASSERT_NE(sdfRoot.Model(), nullptr);
  const auto* sdfJoint = sdfRoot.Model()->JointByName("continuous_hinge");
  ASSERT_NE(sdfJoint, nullptr);
  EXPECT_EQ(sdfJoint->Type(), sdf::JointType::CONTINUOUS);

  const auto path = writeTempSdf(writeResult.value(), "continuous");
  const auto reparsed = utils::SdfParser::readSkeleton(
      common::Uri::createFromPath(path.string()));
  std::filesystem::remove(path);

  ASSERT_NE(reparsed, nullptr);
  const auto* reparsedHinge = test::requireJoint<dynamics::RevoluteJoint>(
      *reparsed, "continuous_hinge");
  ASSERT_NE(reparsedHinge, nullptr);
  EXPECT_TRUE(reparsedHinge->isCyclic(0));
  EXPECT_VECTOR_NEAR(reparsedHinge->getAxis(), Eigen::Vector3d::UnitZ(), 1e-12);
}

//==============================================================================
TEST(SdfWriter, RoundTripsMimicMetadataWithSdf111)
{
  auto skeleton = dynamics::Skeleton::create("mimic_writer");
  auto [rootJoint, base]
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>();
  (void)rootJoint;
  base->setName("base");

  dynamics::UniversalJoint::Properties referenceProperties;
  referenceProperties.mName = "reference_joint";
  referenceProperties.mAxis[0] = Eigen::Vector3d::UnitX();
  referenceProperties.mAxis[1] = Eigen::Vector3d::UnitY();

  dynamics::BodyNode::Properties referenceBodyProperties;
  referenceBodyProperties.mName = "reference_link";

  auto [referenceJoint, referenceLink]
      = skeleton->createJointAndBodyNodePair<dynamics::UniversalJoint>(
          base, referenceProperties, referenceBodyProperties);
  (void)referenceLink;

  dynamics::UniversalJoint::Properties followerProperties;
  followerProperties.mName = "follower_joint";
  followerProperties.mAxis[0] = Eigen::Vector3d::UnitZ();
  followerProperties.mAxis[1] = Eigen::Vector3d::UnitY();

  dynamics::BodyNode::Properties followerBodyProperties;
  followerBodyProperties.mName = "follower_link";

  auto [followerJoint, followerLink]
      = skeleton->createJointAndBodyNodePair<dynamics::UniversalJoint>(
          referenceLink, followerProperties, followerBodyProperties);
  (void)followerLink;

  std::vector<dynamics::MimicDofProperties> mimicProps(
      followerJoint->getNumDofs());
  mimicProps[1].mReferenceJoint = referenceJoint;
  mimicProps[1].mReferenceDofIndex = 1u;
  mimicProps[1].mMultiplier = 0.75;
  mimicProps[1].mOffset = 0.125;
  followerJoint->setMimicJointDofs(mimicProps);
  followerJoint->setActuatorType(dynamics::Joint::MIMIC);

  utils::SdfParser::WriteOptions options;
  options.version = "1.11";
  const auto writeResult
      = utils::SdfParser::tryWriteSkeletonToString(*skeleton, options);
  ASSERT_TRUE(writeResult.isOk()) << writeResult.error().message;

  sdf::Root sdfRoot;
  const auto sdfErrors = sdfRoot.LoadSdfString(writeResult.value());
  ASSERT_TRUE(sdfErrors.empty()) << sdfErrors.front().Message();
  ASSERT_NE(sdfRoot.Model(), nullptr);
  const auto* sdfJoint = sdfRoot.Model()->JointByName("follower_joint");
  ASSERT_NE(sdfJoint, nullptr);
  ASSERT_NE(sdfJoint->Axis(1), nullptr);
  const auto mimic = sdfJoint->Axis(1)->Mimic();
  ASSERT_TRUE(mimic.has_value());
  EXPECT_EQ(mimic->Joint(), "reference_joint");
  EXPECT_EQ(mimic->Axis(), "axis2");
  EXPECT_NEAR(mimic->Multiplier(), 0.75, 1e-12);
  EXPECT_NEAR(mimic->Offset(), 0.125, 1e-12);
  EXPECT_NEAR(mimic->Reference(), 0.0, 1e-12);

  const auto path = writeTempSdf(writeResult.value(), "mimic");
  const auto reparsed = utils::SdfParser::readSkeleton(
      common::Uri::createFromPath(path.string()));
  std::filesystem::remove(path);

  ASSERT_NE(reparsed, nullptr);
  const auto* follower = test::requireJoint<dynamics::UniversalJoint>(
      *reparsed, "follower_joint");
  ASSERT_NE(follower, nullptr);
  EXPECT_EQ(follower->getActuatorType(), dynamics::Joint::MIMIC);
  const auto reparsedMimicProps = follower->getMimicDofProperties();
  ASSERT_EQ(reparsedMimicProps.size(), 2u);
  ASSERT_NE(reparsedMimicProps[1].mReferenceJoint, nullptr);
  EXPECT_EQ(
      reparsedMimicProps[1].mReferenceJoint->getName(), "reference_joint");
  EXPECT_EQ(reparsedMimicProps[1].mReferenceDofIndex, 1u);
  EXPECT_DOUBLE_EQ(reparsedMimicProps[1].mMultiplier, 0.75);
  EXPECT_DOUBLE_EQ(reparsedMimicProps[1].mOffset, 0.125);
}

//==============================================================================
TEST(SdfWriter, MimicMetadataRequiresSdf111)
{
  auto skeleton = dynamics::Skeleton::create("mimic_version");
  auto [rootJoint, base]
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>();
  (void)rootJoint;
  base->setName("base");

  dynamics::RevoluteJoint::Properties referenceProperties;
  referenceProperties.mName = "reference_joint";
  referenceProperties.mAxis = Eigen::Vector3d::UnitZ();
  dynamics::BodyNode::Properties referenceBodyProperties;
  referenceBodyProperties.mName = "reference_link";
  auto [referenceJoint, referenceLink]
      = skeleton->createJointAndBodyNodePair<dynamics::RevoluteJoint>(
          base, referenceProperties, referenceBodyProperties);

  dynamics::RevoluteJoint::Properties followerProperties;
  followerProperties.mName = "follower_joint";
  followerProperties.mAxis = Eigen::Vector3d::UnitY();
  dynamics::BodyNode::Properties followerBodyProperties;
  followerBodyProperties.mName = "follower_link";
  auto [followerJoint, followerLink]
      = skeleton->createJointAndBodyNodePair<dynamics::RevoluteJoint>(
          referenceLink, followerProperties, followerBodyProperties);
  (void)followerLink;

  followerJoint->setMimicJoint(referenceJoint, 2.0, 0.5);
  followerJoint->setActuatorType(dynamics::Joint::MIMIC);

  const auto result = utils::SdfParser::tryWriteSkeletonToString(*skeleton);
  ASSERT_TRUE(result.isErr());
  EXPECT_NE(
      result.error().message.find("mimic requires SDF 1.11"),
      std::string::npos);
}

//==============================================================================
TEST(SdfWriter, CouplerMimicReturnsError)
{
  auto skeleton = dynamics::Skeleton::create("mimic_coupler");
  auto [rootJoint, base]
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>();
  (void)rootJoint;
  base->setName("base");

  dynamics::RevoluteJoint::Properties referenceProperties;
  referenceProperties.mName = "reference_joint";
  referenceProperties.mAxis = Eigen::Vector3d::UnitZ();
  dynamics::BodyNode::Properties referenceBodyProperties;
  referenceBodyProperties.mName = "reference_link";
  auto [referenceJoint, referenceLink]
      = skeleton->createJointAndBodyNodePair<dynamics::RevoluteJoint>(
          base, referenceProperties, referenceBodyProperties);

  dynamics::RevoluteJoint::Properties followerProperties;
  followerProperties.mName = "follower_joint";
  followerProperties.mAxis = Eigen::Vector3d::UnitY();
  dynamics::BodyNode::Properties followerBodyProperties;
  followerBodyProperties.mName = "follower_link";
  auto [followerJoint, followerLink]
      = skeleton->createJointAndBodyNodePair<dynamics::RevoluteJoint>(
          referenceLink, followerProperties, followerBodyProperties);
  (void)followerLink;

  followerJoint->setMimicJoint(referenceJoint, 2.0, 0.5);
  followerJoint->setActuatorType(dynamics::Joint::MIMIC);
  followerJoint->setUseCouplerConstraint(true);

  utils::SdfParser::WriteOptions options;
  options.version = "1.11";
  const auto result
      = utils::SdfParser::tryWriteSkeletonToString(*skeleton, options);
  ASSERT_TRUE(result.isErr());
  EXPECT_NE(
      result.error().message.find("coupler constraint"), std::string::npos);
}

//==============================================================================
TEST(SdfWriter, RootWeldRoundTripsWithFixedRootOption)
{
  auto skeleton = dynamics::Skeleton::create("root_weld_writer");
  skeleton->setMobile(false);

  dynamics::WeldJoint::Properties rootProperties;
  rootProperties.mName = "root";
  rootProperties.mT_ParentBodyToJoint.translation()
      = Eigen::Vector3d(0.2, 0.3, 0.4);

  dynamics::BodyNode::Properties bodyProperties;
  bodyProperties.mName = "fixed_root";
  bodyProperties.mInertia.setMass(1.0);
  bodyProperties.mInertia.setMoment(Eigen::Matrix3d::Identity());

  auto [rootJoint, body]
      = skeleton->createJointAndBodyNodePair<dynamics::WeldJoint>(
          nullptr, rootProperties, bodyProperties);
  (void)rootJoint;
  body->createShapeNodeWith<dynamics::CollisionAspect>(
      std::make_shared<dynamics::SphereShape>(0.2), "fixed_root_sphere");

  const auto writeResult
      = utils::SdfParser::tryWriteSkeletonToString(*skeleton);
  ASSERT_TRUE(writeResult.isOk()) << writeResult.error().message;

  sdf::Root sdfRoot;
  const auto sdfErrors = sdfRoot.LoadSdfString(writeResult.value());
  ASSERT_TRUE(sdfErrors.empty()) << sdfErrors.front().Message();
  ASSERT_NE(sdfRoot.Model(), nullptr);
  EXPECT_TRUE(sdfRoot.Model()->Static());

  const auto path = writeTempSdf(writeResult.value(), "root_weld");
  const utils::SdfParser::Options options(
      nullptr, utils::SdfParser::RootJointType::Fixed);
  const auto reparsed = utils::SdfParser::readSkeleton(
      common::Uri::createFromPath(path.string()), options);
  std::filesystem::remove(path);

  ASSERT_NE(reparsed, nullptr);
  EXPECT_FALSE(reparsed->isMobile());
  ASSERT_EQ(reparsed->getNumBodyNodes(), 1u);
  ASSERT_EQ(reparsed->getNumJoints(), 1u);
  const auto* root = test::requireJoint<dynamics::WeldJoint>(*reparsed, "root");
  ASSERT_NE(root, nullptr);
  const auto* fixedRoot = test::requireBodyNode(*reparsed, "fixed_root");
  ASSERT_NE(fixedRoot, nullptr);
  test::expectJointTopology(*root, nullptr, fixedRoot);
}

//==============================================================================
TEST(SdfWriter, RootRevoluteJointRoundTripsAsParentWorld)
{
  auto skeleton = dynamics::Skeleton::create("root_revolute_writer");

  dynamics::RevoluteJoint::Properties rootProperties;
  rootProperties.mName = "world_hinge";
  rootProperties.mAxis = Eigen::Vector3d::UnitY();
  rootProperties.mT_ParentBodyToJoint.translation()
      = Eigen::Vector3d(0.3, -0.2, 0.5);
  rootProperties.mT_ChildBodyToJoint.translation()
      = Eigen::Vector3d(0.0, 0.1, 0.0);
  rootProperties.mPositionLowerLimits[0] = -0.25;
  rootProperties.mPositionUpperLimits[0] = 0.75;

  dynamics::BodyNode::Properties bodyProperties;
  bodyProperties.mName = "pendulum";
  bodyProperties.mInertia.setMass(1.0);
  bodyProperties.mInertia.setMoment(Eigen::Matrix3d::Identity());

  auto [rootJoint, body]
      = skeleton->createJointAndBodyNodePair<dynamics::RevoluteJoint>(
          nullptr, rootProperties, bodyProperties);
  rootJoint->setDampingCoefficient(0, 0.2);
  rootJoint->setCoulombFriction(0, 0.05);
  rootJoint->setRestPosition(0, 0.1);
  rootJoint->setSpringStiffness(0, 1.5);
  body->createShapeNodeWith<dynamics::CollisionAspect>(
      std::make_shared<dynamics::SphereShape>(0.2), "pendulum_collision");

  const auto writeResult
      = utils::SdfParser::tryWriteSkeletonToString(*skeleton);
  ASSERT_TRUE(writeResult.isOk()) << writeResult.error().message;

  sdf::Root sdfRoot;
  const auto sdfErrors = sdfRoot.LoadSdfString(writeResult.value());
  ASSERT_TRUE(sdfErrors.empty()) << sdfErrors.front().Message();
  ASSERT_NE(sdfRoot.Model(), nullptr);
  const auto* sdfJoint = sdfRoot.Model()->JointByName("world_hinge");
  ASSERT_NE(sdfJoint, nullptr);
  EXPECT_EQ(sdfJoint->Type(), sdf::JointType::REVOLUTE);
  EXPECT_EQ(sdfJoint->ParentName(), "world");
  EXPECT_EQ(sdfJoint->ChildName(), "pendulum");

  const auto path = writeTempSdf(writeResult.value(), "root_revolute");
  const auto reparsed = utils::SdfParser::readSkeleton(
      common::Uri::createFromPath(path.string()));
  std::filesystem::remove(path);

  ASSERT_NE(reparsed, nullptr);
  ASSERT_EQ(reparsed->getNumBodyNodes(), 1u);
  ASSERT_EQ(reparsed->getNumJoints(), 1u);
  const auto* reparsedBody = test::requireBodyNode(*reparsed, "pendulum");
  ASSERT_NE(reparsedBody, nullptr);
  const auto* reparsedJoint
      = test::requireJoint<dynamics::RevoluteJoint>(*reparsed, "world_hinge");
  ASSERT_NE(reparsedJoint, nullptr);
  test::expectJointTopology(*reparsedJoint, nullptr, reparsedBody);
  EXPECT_VECTOR_NEAR(reparsedJoint->getAxis(), Eigen::Vector3d::UnitY(), 1e-12);
  EXPECT_VECTOR_NEAR(
      reparsedJoint->getTransformFromParentBodyNode().translation(),
      Eigen::Vector3d(0.3, -0.2, 0.5),
      1e-12);
  EXPECT_VECTOR_NEAR(
      reparsedJoint->getTransformFromChildBodyNode().translation(),
      Eigen::Vector3d(0.0, 0.1, 0.0),
      1e-12);
  test::expectDofPositionLimits(*reparsedJoint, 0, -0.25, 0.75, 1e-12);
  test::expectDofDynamics(*reparsedJoint, 0, 0.2, 0.05, 0.1, 1.5, 1e-12);
}

//==============================================================================
TEST(SdfWriter, RootContinuousRevoluteJointRoundTripsAsParentWorld)
{
  auto skeleton = dynamics::Skeleton::create("root_continuous_writer");

  dynamics::RevoluteJoint::Properties rootProperties;
  rootProperties.mName = "world_continuous_hinge";
  rootProperties.mAxis = Eigen::Vector3d::UnitY();
  rootProperties.mT_ParentBodyToJoint.translation()
      = Eigen::Vector3d(0.3, -0.1, 0.2);
  rootProperties.mT_ChildBodyToJoint.translation()
      = Eigen::Vector3d(-0.02, 0.04, 0.06);
  rootProperties.mPositionLowerLimits[0]
      = -std::numeric_limits<double>::infinity();
  rootProperties.mPositionUpperLimits[0]
      = std::numeric_limits<double>::infinity();

  dynamics::BodyNode::Properties bodyProperties;
  bodyProperties.mName = "continuous_body";
  bodyProperties.mInertia.setMass(1.0);
  bodyProperties.mInertia.setMoment(Eigen::Matrix3d::Identity());

  auto [rootJoint, body]
      = skeleton->createJointAndBodyNodePair<dynamics::RevoluteJoint>(
          nullptr, rootProperties, bodyProperties);
  rootJoint->setDampingCoefficient(0, 0.25);
  rootJoint->setCoulombFriction(0, 0.07);
  rootJoint->setRestPosition(0, -0.15);
  rootJoint->setSpringStiffness(0, 0.6);
  body->createShapeNodeWith<dynamics::CollisionAspect>(
      std::make_shared<dynamics::SphereShape>(0.2),
      "continuous_body_collision");

  const auto writeResult
      = utils::SdfParser::tryWriteSkeletonToString(*skeleton);
  ASSERT_TRUE(writeResult.isOk()) << writeResult.error().message;

  sdf::Root sdfRoot;
  const auto sdfErrors = sdfRoot.LoadSdfString(writeResult.value());
  ASSERT_TRUE(sdfErrors.empty()) << sdfErrors.front().Message();
  ASSERT_NE(sdfRoot.Model(), nullptr);
  const auto* sdfJoint = sdfRoot.Model()->JointByName("world_continuous_hinge");
  ASSERT_NE(sdfJoint, nullptr);
  EXPECT_EQ(sdfJoint->Type(), sdf::JointType::CONTINUOUS);
  EXPECT_EQ(sdfJoint->ParentName(), "world");
  EXPECT_EQ(sdfJoint->ChildName(), "continuous_body");

  const auto path = writeTempSdf(writeResult.value(), "root_continuous");
  const auto reparsed = utils::SdfParser::readSkeleton(
      common::Uri::createFromPath(path.string()));
  std::filesystem::remove(path);

  ASSERT_NE(reparsed, nullptr);
  ASSERT_EQ(reparsed->getNumBodyNodes(), 1u);
  ASSERT_EQ(reparsed->getNumJoints(), 1u);
  const auto* reparsedBody
      = test::requireBodyNode(*reparsed, "continuous_body");
  ASSERT_NE(reparsedBody, nullptr);
  const auto* reparsedJoint = test::requireJoint<dynamics::RevoluteJoint>(
      *reparsed, "world_continuous_hinge");
  ASSERT_NE(reparsedJoint, nullptr);
  test::expectJointTopology(*reparsedJoint, nullptr, reparsedBody);
  EXPECT_TRUE(reparsedJoint->isCyclic(0));
  EXPECT_VECTOR_NEAR(reparsedJoint->getAxis(), Eigen::Vector3d::UnitY(), 1e-12);
  EXPECT_VECTOR_NEAR(
      reparsedJoint->getTransformFromParentBodyNode().translation(),
      Eigen::Vector3d(0.3, -0.1, 0.2),
      1e-12);
  EXPECT_VECTOR_NEAR(
      reparsedJoint->getTransformFromChildBodyNode().translation(),
      Eigen::Vector3d(-0.02, 0.04, 0.06),
      1e-12);
  test::expectDofDynamics(*reparsedJoint, 0, 0.25, 0.07, -0.15, 0.6, 1e-12);
}

//==============================================================================
TEST(SdfWriter, RootPrismaticJointRoundTripsAsParentWorld)
{
  auto skeleton = dynamics::Skeleton::create("root_prismatic_writer");

  dynamics::PrismaticJoint::Properties rootProperties;
  rootProperties.mName = "world_slider";
  rootProperties.mAxis = Eigen::Vector3d::UnitX();
  rootProperties.mT_ParentBodyToJoint.translation()
      = Eigen::Vector3d(-0.1, 0.2, 0.3);
  rootProperties.mT_ChildBodyToJoint.translation()
      = Eigen::Vector3d(0.0, -0.05, 0.1);
  rootProperties.mPositionLowerLimits[0] = -0.4;
  rootProperties.mPositionUpperLimits[0] = 0.6;

  dynamics::BodyNode::Properties bodyProperties;
  bodyProperties.mName = "slider";
  bodyProperties.mInertia.setMass(1.0);
  bodyProperties.mInertia.setMoment(Eigen::Matrix3d::Identity());

  auto [rootJoint, body]
      = skeleton->createJointAndBodyNodePair<dynamics::PrismaticJoint>(
          nullptr, rootProperties, bodyProperties);
  rootJoint->setDampingCoefficient(0, 0.15);
  rootJoint->setCoulombFriction(0, 0.03);
  rootJoint->setRestPosition(0, -0.2);
  rootJoint->setSpringStiffness(0, 0.9);
  body->createShapeNodeWith<dynamics::CollisionAspect>(
      std::make_shared<dynamics::BoxShape>(Eigen::Vector3d(0.2, 0.3, 0.4)),
      "slider_collision");

  const auto writeResult
      = utils::SdfParser::tryWriteSkeletonToString(*skeleton);
  ASSERT_TRUE(writeResult.isOk()) << writeResult.error().message;

  sdf::Root sdfRoot;
  const auto sdfErrors = sdfRoot.LoadSdfString(writeResult.value());
  ASSERT_TRUE(sdfErrors.empty()) << sdfErrors.front().Message();
  ASSERT_NE(sdfRoot.Model(), nullptr);
  const auto* sdfJoint = sdfRoot.Model()->JointByName("world_slider");
  ASSERT_NE(sdfJoint, nullptr);
  EXPECT_EQ(sdfJoint->Type(), sdf::JointType::PRISMATIC);
  EXPECT_EQ(sdfJoint->ParentName(), "world");
  EXPECT_EQ(sdfJoint->ChildName(), "slider");

  const auto path = writeTempSdf(writeResult.value(), "root_prismatic");
  const auto reparsed = utils::SdfParser::readSkeleton(
      common::Uri::createFromPath(path.string()));
  std::filesystem::remove(path);

  ASSERT_NE(reparsed, nullptr);
  ASSERT_EQ(reparsed->getNumBodyNodes(), 1u);
  ASSERT_EQ(reparsed->getNumJoints(), 1u);
  const auto* reparsedBody = test::requireBodyNode(*reparsed, "slider");
  ASSERT_NE(reparsedBody, nullptr);
  const auto* reparsedJoint
      = test::requireJoint<dynamics::PrismaticJoint>(*reparsed, "world_slider");
  ASSERT_NE(reparsedJoint, nullptr);
  test::expectJointTopology(*reparsedJoint, nullptr, reparsedBody);
  EXPECT_VECTOR_NEAR(reparsedJoint->getAxis(), Eigen::Vector3d::UnitX(), 1e-12);
  EXPECT_VECTOR_NEAR(
      reparsedJoint->getTransformFromParentBodyNode().translation(),
      Eigen::Vector3d(-0.1, 0.2, 0.3),
      1e-12);
  EXPECT_VECTOR_NEAR(
      reparsedJoint->getTransformFromChildBodyNode().translation(),
      Eigen::Vector3d(0.0, -0.05, 0.1),
      1e-12);
  test::expectDofPositionLimits(*reparsedJoint, 0, -0.4, 0.6, 1e-12);
  test::expectDofDynamics(*reparsedJoint, 0, 0.15, 0.03, -0.2, 0.9, 1e-12);
}

//==============================================================================
TEST(SdfWriter, RootScrewJointRoundTripsAsParentWorld)
{
  auto skeleton = dynamics::Skeleton::create("root_screw_writer");

  dynamics::ScrewJoint::Properties rootProperties;
  rootProperties.mName = "world_screw";
  rootProperties.mAxis = Eigen::Vector3d::UnitZ();
  rootProperties.mPitch = 0.35;
  rootProperties.mT_ParentBodyToJoint.translation()
      = Eigen::Vector3d(0.15, -0.25, 0.35);
  rootProperties.mT_ChildBodyToJoint.translation()
      = Eigen::Vector3d(0.0, 0.0, -0.08);
  rootProperties.mPositionLowerLimits[0] = -0.5;
  rootProperties.mPositionUpperLimits[0] = 0.5;

  dynamics::BodyNode::Properties bodyProperties;
  bodyProperties.mName = "screw_body";
  bodyProperties.mInertia.setMass(1.0);
  bodyProperties.mInertia.setMoment(Eigen::Matrix3d::Identity());

  auto [rootJoint, body]
      = skeleton->createJointAndBodyNodePair<dynamics::ScrewJoint>(
          nullptr, rootProperties, bodyProperties);
  rootJoint->setDampingCoefficient(0, 0.12);
  rootJoint->setCoulombFriction(0, 0.02);
  rootJoint->setRestPosition(0, 0.05);
  rootJoint->setSpringStiffness(0, 0.7);
  body->createShapeNodeWith<dynamics::CollisionAspect>(
      std::make_shared<dynamics::CylinderShape>(0.1, 0.25),
      "screw_body_collision");

  const auto writeResult
      = utils::SdfParser::tryWriteSkeletonToString(*skeleton);
  ASSERT_TRUE(writeResult.isOk()) << writeResult.error().message;

  sdf::Root sdfRoot;
  const auto sdfErrors = sdfRoot.LoadSdfString(writeResult.value());
  ASSERT_TRUE(sdfErrors.empty()) << sdfErrors.front().Message();
  ASSERT_NE(sdfRoot.Model(), nullptr);
  const auto* sdfJoint = sdfRoot.Model()->JointByName("world_screw");
  ASSERT_NE(sdfJoint, nullptr);
  EXPECT_EQ(sdfJoint->Type(), sdf::JointType::SCREW);
  EXPECT_EQ(sdfJoint->ParentName(), "world");
  EXPECT_EQ(sdfJoint->ChildName(), "screw_body");
  EXPECT_NEAR(sdfJoint->ScrewThreadPitch(), 0.35, 1e-12);

  const auto path = writeTempSdf(writeResult.value(), "root_screw");
  const auto reparsed = utils::SdfParser::readSkeleton(
      common::Uri::createFromPath(path.string()));
  std::filesystem::remove(path);

  ASSERT_NE(reparsed, nullptr);
  ASSERT_EQ(reparsed->getNumBodyNodes(), 1u);
  ASSERT_EQ(reparsed->getNumJoints(), 1u);
  const auto* reparsedBody = test::requireBodyNode(*reparsed, "screw_body");
  ASSERT_NE(reparsedBody, nullptr);
  const auto* reparsedJoint
      = test::requireJoint<dynamics::ScrewJoint>(*reparsed, "world_screw");
  ASSERT_NE(reparsedJoint, nullptr);
  test::expectJointTopology(*reparsedJoint, nullptr, reparsedBody);
  EXPECT_VECTOR_NEAR(reparsedJoint->getAxis(), Eigen::Vector3d::UnitZ(), 1e-12);
  EXPECT_NEAR(reparsedJoint->getPitch(), 0.35, 1e-12);
  EXPECT_VECTOR_NEAR(
      reparsedJoint->getTransformFromParentBodyNode().translation(),
      Eigen::Vector3d(0.15, -0.25, 0.35),
      1e-12);
  EXPECT_VECTOR_NEAR(
      reparsedJoint->getTransformFromChildBodyNode().translation(),
      Eigen::Vector3d(0.0, 0.0, -0.08),
      1e-12);
  test::expectDofPositionLimits(*reparsedJoint, 0, -0.5, 0.5, 1e-12);
  test::expectDofDynamics(*reparsedJoint, 0, 0.12, 0.02, 0.05, 0.7, 1e-12);
}

//==============================================================================
TEST(SdfWriter, RootUniversalJointRoundTripsAsParentWorld)
{
  auto skeleton = dynamics::Skeleton::create("root_universal_writer");

  dynamics::UniversalJoint::Properties rootProperties;
  rootProperties.mName = "world_universal";
  rootProperties.mAxis[0] = Eigen::Vector3d::UnitX();
  rootProperties.mAxis[1] = Eigen::Vector3d::UnitY();
  rootProperties.mT_ParentBodyToJoint.translation()
      = Eigen::Vector3d(-0.25, 0.15, 0.45);
  rootProperties.mT_ChildBodyToJoint.translation()
      = Eigen::Vector3d(0.05, 0.0, -0.05);
  rootProperties.mPositionLowerLimits[0] = -0.6;
  rootProperties.mPositionUpperLimits[0] = 0.7;
  rootProperties.mPositionLowerLimits[1] = -0.8;
  rootProperties.mPositionUpperLimits[1] = 0.9;

  dynamics::BodyNode::Properties bodyProperties;
  bodyProperties.mName = "universal_body";
  bodyProperties.mInertia.setMass(1.0);
  bodyProperties.mInertia.setMoment(Eigen::Matrix3d::Identity());

  auto [rootJoint, body]
      = skeleton->createJointAndBodyNodePair<dynamics::UniversalJoint>(
          nullptr, rootProperties, bodyProperties);
  rootJoint->setDampingCoefficient(0, 0.21);
  rootJoint->setCoulombFriction(0, 0.04);
  rootJoint->setRestPosition(0, -0.3);
  rootJoint->setSpringStiffness(0, 1.1);
  rootJoint->setDampingCoefficient(1, 0.31);
  rootJoint->setCoulombFriction(1, 0.06);
  rootJoint->setRestPosition(1, 0.4);
  rootJoint->setSpringStiffness(1, 1.3);
  body->createShapeNodeWith<dynamics::CollisionAspect>(
      std::make_shared<dynamics::SphereShape>(0.2), "universal_body_collision");

  const auto writeResult
      = utils::SdfParser::tryWriteSkeletonToString(*skeleton);
  ASSERT_TRUE(writeResult.isOk()) << writeResult.error().message;

  sdf::Root sdfRoot;
  const auto sdfErrors = sdfRoot.LoadSdfString(writeResult.value());
  ASSERT_TRUE(sdfErrors.empty()) << sdfErrors.front().Message();
  ASSERT_NE(sdfRoot.Model(), nullptr);
  const auto* sdfJoint = sdfRoot.Model()->JointByName("world_universal");
  ASSERT_NE(sdfJoint, nullptr);
  EXPECT_EQ(sdfJoint->Type(), sdf::JointType::UNIVERSAL);
  EXPECT_EQ(sdfJoint->ParentName(), "world");
  EXPECT_EQ(sdfJoint->ChildName(), "universal_body");

  const auto path = writeTempSdf(writeResult.value(), "root_universal");
  const auto reparsed = utils::SdfParser::readSkeleton(
      common::Uri::createFromPath(path.string()));
  std::filesystem::remove(path);

  ASSERT_NE(reparsed, nullptr);
  ASSERT_EQ(reparsed->getNumBodyNodes(), 1u);
  ASSERT_EQ(reparsed->getNumJoints(), 1u);
  const auto* reparsedBody = test::requireBodyNode(*reparsed, "universal_body");
  ASSERT_NE(reparsedBody, nullptr);
  const auto* reparsedJoint = test::requireJoint<dynamics::UniversalJoint>(
      *reparsed, "world_universal");
  ASSERT_NE(reparsedJoint, nullptr);
  test::expectJointTopology(*reparsedJoint, nullptr, reparsedBody);
  EXPECT_VECTOR_NEAR(
      reparsedJoint->getAxis1(), Eigen::Vector3d::UnitX(), 1e-12);
  EXPECT_VECTOR_NEAR(
      reparsedJoint->getAxis2(), Eigen::Vector3d::UnitY(), 1e-12);
  EXPECT_VECTOR_NEAR(
      reparsedJoint->getTransformFromParentBodyNode().translation(),
      Eigen::Vector3d(-0.25, 0.15, 0.45),
      1e-12);
  EXPECT_VECTOR_NEAR(
      reparsedJoint->getTransformFromChildBodyNode().translation(),
      Eigen::Vector3d(0.05, 0.0, -0.05),
      1e-12);
  test::expectDofPositionLimits(*reparsedJoint, 0, -0.6, 0.7, 1e-12);
  test::expectDofDynamics(*reparsedJoint, 0, 0.21, 0.04, -0.3, 1.1, 1e-12);
  test::expectDofPositionLimits(*reparsedJoint, 1, -0.8, 0.9, 1e-12);
  test::expectDofDynamics(*reparsedJoint, 1, 0.31, 0.06, 0.4, 1.3, 1e-12);
}

//==============================================================================
TEST(SdfWriter, RootBallJointRoundTripsAsParentWorld)
{
  auto skeleton = dynamics::Skeleton::create("root_ball_writer");

  dynamics::BallJoint::Properties rootProperties;
  rootProperties.mName = "world_ball";
  rootProperties.mT_ParentBodyToJoint.translation()
      = Eigen::Vector3d(0.2, 0.1, -0.3);
  rootProperties.mT_ChildBodyToJoint.translation()
      = Eigen::Vector3d(-0.05, 0.1, 0.0);

  dynamics::BodyNode::Properties bodyProperties;
  bodyProperties.mName = "ball_body";
  bodyProperties.mInertia.setMass(1.0);
  bodyProperties.mInertia.setMoment(Eigen::Matrix3d::Identity());

  auto [rootJoint, body]
      = skeleton->createJointAndBodyNodePair<dynamics::BallJoint>(
          nullptr, rootProperties, bodyProperties);
  (void)rootJoint;
  body->createShapeNodeWith<dynamics::CollisionAspect>(
      std::make_shared<dynamics::SphereShape>(0.2), "ball_body_collision");

  const auto writeResult
      = utils::SdfParser::tryWriteSkeletonToString(*skeleton);
  ASSERT_TRUE(writeResult.isOk()) << writeResult.error().message;

  sdf::Root sdfRoot;
  const auto sdfErrors = sdfRoot.LoadSdfString(writeResult.value());
  ASSERT_TRUE(sdfErrors.empty()) << sdfErrors.front().Message();
  ASSERT_NE(sdfRoot.Model(), nullptr);
  const auto* sdfJoint = sdfRoot.Model()->JointByName("world_ball");
  ASSERT_NE(sdfJoint, nullptr);
  EXPECT_EQ(sdfJoint->Type(), sdf::JointType::BALL);
  EXPECT_EQ(sdfJoint->ParentName(), "world");
  EXPECT_EQ(sdfJoint->ChildName(), "ball_body");

  const auto path = writeTempSdf(writeResult.value(), "root_ball");
  const auto reparsed = utils::SdfParser::readSkeleton(
      common::Uri::createFromPath(path.string()));
  std::filesystem::remove(path);

  ASSERT_NE(reparsed, nullptr);
  ASSERT_EQ(reparsed->getNumBodyNodes(), 1u);
  ASSERT_EQ(reparsed->getNumJoints(), 1u);
  const auto* reparsedBody = test::requireBodyNode(*reparsed, "ball_body");
  ASSERT_NE(reparsedBody, nullptr);
  const auto* reparsedJoint
      = test::requireJoint<dynamics::BallJoint>(*reparsed, "world_ball");
  ASSERT_NE(reparsedJoint, nullptr);
  test::expectJointTopology(*reparsedJoint, nullptr, reparsedBody);
  EXPECT_EQ(reparsedJoint->getNumDofs(), 3u);
  EXPECT_VECTOR_NEAR(
      reparsedJoint->getTransformFromParentBodyNode().translation(),
      Eigen::Vector3d(0.2, 0.1, -0.3),
      1e-12);
  EXPECT_VECTOR_NEAR(
      reparsedJoint->getTransformFromChildBodyNode().translation(),
      Eigen::Vector3d(-0.05, 0.1, 0.0),
      1e-12);
}

//==============================================================================
TEST(SdfWriter, RoundTripsMultipleRootFreeJointTrees)
{
  auto skeleton = dynamics::Skeleton::create("multi_root_writer");

  dynamics::FreeJoint::Properties alphaRootProperties;
  alphaRootProperties.mName = "alpha_root";
  alphaRootProperties.mT_ParentBodyToJoint.translation()
      = Eigen::Vector3d(0.25, 0.5, 0.75);

  dynamics::BodyNode::Properties alphaProperties;
  alphaProperties.mName = "alpha";
  alphaProperties.mInertia.setMass(1.0);
  alphaProperties.mInertia.setMoment(Eigen::Matrix3d::Identity());

  auto [alphaRoot, alpha]
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>(
          nullptr, alphaRootProperties, alphaProperties);
  (void)alphaRoot;
  alpha->createShapeNodeWith<dynamics::VisualAspect>(
      std::make_shared<dynamics::BoxShape>(Eigen::Vector3d(0.2, 0.3, 0.4)),
      "alpha_box");

  dynamics::FreeJoint::Properties betaRootProperties;
  betaRootProperties.mName = "beta_root";
  betaRootProperties.mT_ParentBodyToJoint.translation()
      = Eigen::Vector3d(-0.5, 0.25, 1.25);

  dynamics::BodyNode::Properties betaProperties;
  betaProperties.mName = "beta";
  betaProperties.mInertia.setMass(0.75);
  betaProperties.mInertia.setMoment(Eigen::Matrix3d::Identity() * 0.75);

  auto [betaRoot, beta]
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>(
          nullptr, betaRootProperties, betaProperties);
  (void)betaRoot;
  beta->createShapeNodeWith<dynamics::CollisionAspect>(
      std::make_shared<dynamics::SphereShape>(0.35), "beta_sphere");

  const auto writeResult
      = utils::SdfParser::tryWriteSkeletonToString(*skeleton);
  ASSERT_TRUE(writeResult.isOk()) << writeResult.error().message;

  sdf::Root sdfRoot;
  const auto sdfErrors = sdfRoot.LoadSdfString(writeResult.value());
  ASSERT_TRUE(sdfErrors.empty()) << sdfErrors.front().Message();
  ASSERT_NE(sdfRoot.Model(), nullptr);
  EXPECT_EQ(sdfRoot.Model()->LinkCount(), 2u);
  EXPECT_EQ(sdfRoot.Model()->JointCount(), 0u);
  EXPECT_NE(sdfRoot.Model()->LinkByName("alpha"), nullptr);
  EXPECT_NE(sdfRoot.Model()->LinkByName("beta"), nullptr);

  const auto path = writeTempSdf(writeResult.value(), "multi_root");
  const auto reparsed = utils::SdfParser::readSkeleton(
      common::Uri::createFromPath(path.string()));
  std::filesystem::remove(path);

  ASSERT_NE(reparsed, nullptr);
  EXPECT_EQ(reparsed->getNumBodyNodes(), 2u);
  EXPECT_EQ(reparsed->getNumJoints(), 2u);

  const auto* alphaBody = test::requireBodyNode(*reparsed, "alpha");
  const auto* betaBody = test::requireBodyNode(*reparsed, "beta");
  ASSERT_NE(alphaBody, nullptr);
  ASSERT_NE(betaBody, nullptr);

  const auto* alphaJoint
      = dynamic_cast<const dynamics::FreeJoint*>(alphaBody->getParentJoint());
  ASSERT_NE(alphaJoint, nullptr);
  test::expectJointTopology(*alphaJoint, nullptr, alphaBody);
  EXPECT_VECTOR_NEAR(
      alphaJoint->getTransformFromParentBodyNode().translation(),
      Eigen::Vector3d(0.25, 0.5, 0.75),
      1e-12);

  const auto* betaJoint
      = dynamic_cast<const dynamics::FreeJoint*>(betaBody->getParentJoint());
  ASSERT_NE(betaJoint, nullptr);
  test::expectJointTopology(*betaJoint, nullptr, betaBody);
  EXPECT_VECTOR_NEAR(
      betaJoint->getTransformFromParentBodyNode().translation(),
      Eigen::Vector3d(-0.5, 0.25, 1.25),
      1e-12);

  const auto* alphaBox
      = test::requireShape<dynamics::VisualAspect, dynamics::BoxShape>(
          *alphaBody, 0, 1);
  ASSERT_NE(alphaBox, nullptr);
  EXPECT_VECTOR_NEAR(
      alphaBox->getSize(), Eigen::Vector3d(0.2, 0.3, 0.4), 1e-12);

  const auto* betaSphere
      = test::requireShape<dynamics::CollisionAspect, dynamics::SphereShape>(
          *betaBody, 0, 1);
  ASSERT_NE(betaSphere, nullptr);
  EXPECT_DOUBLE_EQ(betaSphere->getRadius(), 0.35);
}

//==============================================================================
TEST(SdfWriter, RoundTripsMixedImplicitAndParentWorldRoots)
{
  auto skeleton = dynamics::Skeleton::create("mixed_root_writer");

  dynamics::FreeJoint::Properties baseRootProperties;
  baseRootProperties.mName = "base_root";
  baseRootProperties.mT_ParentBodyToJoint.translation()
      = Eigen::Vector3d(0.4, -0.2, 0.6);

  dynamics::BodyNode::Properties baseProperties;
  baseProperties.mName = "base";
  baseProperties.mInertia.setMass(1.25);
  baseProperties.mInertia.setMoment(Eigen::Matrix3d::Identity() * 1.25);

  auto [baseRoot, base]
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>(
          nullptr, baseRootProperties, baseProperties);
  (void)baseRoot;
  base->createShapeNodeWith<dynamics::VisualAspect>(
      std::make_shared<dynamics::BoxShape>(Eigen::Vector3d(0.3, 0.2, 0.1)),
      "base_box");

  dynamics::RevoluteJoint::Properties hingeProperties;
  hingeProperties.mName = "world_hinge";
  hingeProperties.mAxis = Eigen::Vector3d::UnitY();
  hingeProperties.mT_ParentBodyToJoint.translation()
      = Eigen::Vector3d(-0.3, 0.5, 0.25);
  hingeProperties.mT_ChildBodyToJoint.translation()
      = Eigen::Vector3d(0.0, -0.1, 0.0);
  hingeProperties.mPositionLowerLimits[0] = -0.2;
  hingeProperties.mPositionUpperLimits[0] = 0.8;

  dynamics::BodyNode::Properties pendulumProperties;
  pendulumProperties.mName = "pendulum";
  pendulumProperties.mInertia.setMass(0.8);
  pendulumProperties.mInertia.setMoment(Eigen::Matrix3d::Identity() * 0.8);

  auto [hinge, pendulum]
      = skeleton->createJointAndBodyNodePair<dynamics::RevoluteJoint>(
          nullptr, hingeProperties, pendulumProperties);
  hinge->setDampingCoefficient(0, 0.14);
  hinge->setCoulombFriction(0, 0.03);
  hinge->setRestPosition(0, 0.05);
  hinge->setSpringStiffness(0, 0.9);
  pendulum->createShapeNodeWith<dynamics::CollisionAspect>(
      std::make_shared<dynamics::SphereShape>(0.25), "pendulum_sphere");

  const auto writeResult
      = utils::SdfParser::tryWriteSkeletonToString(*skeleton);
  ASSERT_TRUE(writeResult.isOk()) << writeResult.error().message;

  sdf::Root sdfRoot;
  const auto sdfErrors = sdfRoot.LoadSdfString(writeResult.value());
  ASSERT_TRUE(sdfErrors.empty()) << sdfErrors.front().Message();
  ASSERT_NE(sdfRoot.Model(), nullptr);
  EXPECT_EQ(sdfRoot.Model()->LinkCount(), 2u);
  EXPECT_EQ(sdfRoot.Model()->JointCount(), 1u);
  EXPECT_NE(sdfRoot.Model()->LinkByName("base"), nullptr);
  EXPECT_NE(sdfRoot.Model()->LinkByName("pendulum"), nullptr);
  const auto* sdfJoint = sdfRoot.Model()->JointByName("world_hinge");
  ASSERT_NE(sdfJoint, nullptr);
  EXPECT_EQ(sdfJoint->Type(), sdf::JointType::REVOLUTE);
  EXPECT_EQ(sdfJoint->ParentName(), "world");
  EXPECT_EQ(sdfJoint->ChildName(), "pendulum");

  const auto path = writeTempSdf(writeResult.value(), "mixed_root");
  const auto reparsed = utils::SdfParser::readSkeleton(
      common::Uri::createFromPath(path.string()));
  std::filesystem::remove(path);

  ASSERT_NE(reparsed, nullptr);
  EXPECT_EQ(reparsed->getNumBodyNodes(), 2u);
  EXPECT_EQ(reparsed->getNumJoints(), 2u);

  const auto* baseBody = test::requireBodyNode(*reparsed, "base");
  const auto* pendulumBody = test::requireBodyNode(*reparsed, "pendulum");
  ASSERT_NE(baseBody, nullptr);
  ASSERT_NE(pendulumBody, nullptr);

  const auto* reparsedBaseRoot
      = dynamic_cast<const dynamics::FreeJoint*>(baseBody->getParentJoint());
  ASSERT_NE(reparsedBaseRoot, nullptr);
  test::expectJointTopology(*reparsedBaseRoot, nullptr, baseBody);
  EXPECT_VECTOR_NEAR(
      reparsedBaseRoot->getTransformFromParentBodyNode().translation(),
      Eigen::Vector3d(0.4, -0.2, 0.6),
      1e-12);

  const auto* reparsedHinge
      = test::requireJoint<dynamics::RevoluteJoint>(*reparsed, "world_hinge");
  ASSERT_NE(reparsedHinge, nullptr);
  test::expectJointTopology(*reparsedHinge, nullptr, pendulumBody);
  EXPECT_VECTOR_NEAR(reparsedHinge->getAxis(), Eigen::Vector3d::UnitY(), 1e-12);
  EXPECT_VECTOR_NEAR(
      reparsedHinge->getTransformFromParentBodyNode().translation(),
      Eigen::Vector3d(-0.3, 0.5, 0.25),
      1e-12);
  EXPECT_VECTOR_NEAR(
      reparsedHinge->getTransformFromChildBodyNode().translation(),
      Eigen::Vector3d(0.0, -0.1, 0.0),
      1e-12);
  test::expectDofPositionLimits(*reparsedHinge, 0, -0.2, 0.8, 1e-12);
  test::expectDofDynamics(*reparsedHinge, 0, 0.14, 0.03, 0.05, 0.9, 1e-12);

  const auto* baseBox
      = test::requireShape<dynamics::VisualAspect, dynamics::BoxShape>(
          *baseBody, 0, 1);
  ASSERT_NE(baseBox, nullptr);
  EXPECT_VECTOR_NEAR(baseBox->getSize(), Eigen::Vector3d(0.3, 0.2, 0.1), 1e-12);

  const auto* pendulumSphere
      = test::requireShape<dynamics::CollisionAspect, dynamics::SphereShape>(
          *pendulumBody, 0, 1);
  ASSERT_NE(pendulumSphere, nullptr);
  EXPECT_DOUBLE_EQ(pendulumSphere->getRadius(), 0.25);
}

//==============================================================================
TEST(SdfWriter, ChildFreeJointReturnsExplicitUnsupportedError)
{
  expectUnsupportedChildJointError<dynamics::FreeJoint>(
      "unsupported_child_free_joint",
      "floating_child",
      "DART FreeJoint [floating_child] as an SDF child joint");
}

//==============================================================================
TEST(SdfWriter, RootEulerJointReturnsExplicitUnsupportedError)
{
  expectUnsupportedRootJointError<dynamics::EulerJoint>(
      "unsupported_root_euler_joint",
      "euler_root",
      "root DART EulerJoint [euler_root] as SDF because SDF has no "
      "Euler-axis-order joint primitive");
}

//==============================================================================
TEST(SdfWriter, EulerJointReturnsExplicitUnsupportedError)
{
  expectUnsupportedChildJointError<dynamics::EulerJoint>(
      "unsupported_euler_joint",
      "euler_child",
      "SDF has no Euler-axis-order joint primitive");
}

//==============================================================================
TEST(SdfWriter, PlanarJointReturnsExplicitUnsupportedError)
{
  expectUnsupportedChildJointError<dynamics::PlanarJoint>(
      "unsupported_planar_joint",
      "planar_child",
      "SDF has no planar joint primitive");
}

//==============================================================================
TEST(SdfWriter, TranslationalJoint2DReturnsExplicitUnsupportedError)
{
  expectUnsupportedChildJointError<dynamics::TranslationalJoint2D>(
      "unsupported_translational_2d_joint",
      "slide_2d_child",
      "SDF has no two-axis translational joint primitive");
}

//==============================================================================
TEST(SdfWriter, TranslationalJointReturnsExplicitUnsupportedError)
{
  expectUnsupportedChildJointError<dynamics::TranslationalJoint>(
      "unsupported_translational_joint",
      "slide_3d_child",
      "SDF has no three-axis translational joint primitive");
}

//==============================================================================
TEST(SdfWriter, RoundTripsCapsuleAndConeGeometry)
{
  auto skeleton = dynamics::Skeleton::create("capsule_cone_writer");
  auto [joint, body]
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>();
  (void)joint;
  body->setName("body");
  body->createShapeNodeWith<dynamics::VisualAspect>(
      std::make_shared<dynamics::CapsuleShape>(0.2, 0.7), "capsule_visual");
  body->createShapeNodeWith<dynamics::CollisionAspect>(
      std::make_shared<dynamics::ConeShape>(0.3, 0.9), "cone_collision");

  utils::SdfParser::WriteOptions options;
  options.version = "1.12";
  const auto writeResult
      = utils::SdfParser::tryWriteSkeletonToString(*skeleton, options);
  ASSERT_TRUE(writeResult.isOk()) << writeResult.error().message;

  sdf::Root sdfRoot;
  const auto sdfErrors = sdfRoot.LoadSdfString(writeResult.value());
  ASSERT_TRUE(sdfErrors.empty()) << sdfErrors.front().Message();
  ASSERT_NE(sdfRoot.Model(), nullptr);
  const auto* sdfLink = sdfRoot.Model()->LinkByName("body");
  ASSERT_NE(sdfLink, nullptr);
  const auto* sdfCapsuleVisual = sdfLink->VisualByName("capsule_visual");
  ASSERT_NE(sdfCapsuleVisual, nullptr);
  ASSERT_NE(sdfCapsuleVisual->Geom(), nullptr);
  EXPECT_EQ(sdfCapsuleVisual->Geom()->Type(), sdf::GeometryType::CAPSULE);
  const auto* sdfConeCollision = sdfLink->CollisionByName("cone_collision");
  ASSERT_NE(sdfConeCollision, nullptr);
  ASSERT_NE(sdfConeCollision->Geom(), nullptr);
  EXPECT_EQ(sdfConeCollision->Geom()->Type(), sdf::GeometryType::CONE);

  const auto path = writeTempSdf(writeResult.value(), "capsule_cone");
  const auto reparsed = utils::SdfParser::readSkeleton(
      common::Uri::createFromPath(path.string()));
  std::filesystem::remove(path);

  ASSERT_NE(reparsed, nullptr);
  const auto* reparsedBody = test::requireBodyNode(*reparsed, "body");
  ASSERT_NE(reparsedBody, nullptr);
  const auto* capsule
      = test::requireShape<dynamics::VisualAspect, dynamics::CapsuleShape>(
          *reparsedBody, 0, 1);
  ASSERT_NE(capsule, nullptr);
  EXPECT_DOUBLE_EQ(capsule->getRadius(), 0.2);
  EXPECT_DOUBLE_EQ(capsule->getHeight(), 0.7);

  const auto* cone
      = test::requireShape<dynamics::CollisionAspect, dynamics::ConeShape>(
          *reparsedBody, 0, 1);
  ASSERT_NE(cone, nullptr);
  EXPECT_DOUBLE_EQ(cone->getRadius(), 0.3);
  EXPECT_DOUBLE_EQ(cone->getHeight(), 0.9);
}

//==============================================================================
TEST(SdfWriter, PyramidShapeReturnsGeneratedMeshError)
{
  auto skeleton = dynamics::Skeleton::create("pyramid_shape");
  auto [joint, body]
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>();
  (void)joint;
  body->setName("body");
  body->createShapeNodeWith<dynamics::VisualAspect>(
      std::make_shared<dynamics::PyramidShape>(0.5, 0.75, 1.0), "pyramid");

  const auto result = utils::SdfParser::tryWriteSkeletonToString(*skeleton);
  ASSERT_TRUE(result.isErr());
  EXPECT_NE(
      result.error().message.find("DART PyramidShape"), std::string::npos);
  EXPECT_NE(
      result.error().message.find("generated mesh resources"),
      std::string::npos);
}

//==============================================================================
TEST(SdfWriter, ArrowShapeReturnsGeneratedMeshError)
{
  auto skeleton = dynamics::Skeleton::create("arrow_shape");
  auto [joint, body]
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>();
  (void)joint;
  body->setName("body");
  body->createShapeNodeWith<dynamics::VisualAspect>(
      std::make_shared<dynamics::ArrowShape>(
          Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitZ()),
      "arrow");

  const auto result = utils::SdfParser::tryWriteSkeletonToString(*skeleton);
  ASSERT_TRUE(result.isErr());
  EXPECT_NE(result.error().message.find("DART ArrowShape"), std::string::npos);
  EXPECT_NE(
      result.error().message.find("generated mesh resources"),
      std::string::npos);
}

//==============================================================================
TEST(SdfWriter, PointCloudShapeReturnsExplicitUnsupportedError)
{
  auto skeleton = dynamics::Skeleton::create("point_cloud_shape");
  auto [joint, body]
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>();
  (void)joint;
  body->setName("body");
  auto pointCloud = std::make_shared<dynamics::PointCloudShape>();
  pointCloud->addPoint(Eigen::Vector3d::Zero());
  pointCloud->addPoint(Eigen::Vector3d::UnitX());
  body->createShapeNodeWith<dynamics::VisualAspect>(
      std::move(pointCloud), "point_cloud");

  const auto result = utils::SdfParser::tryWriteSkeletonToString(*skeleton);
  ASSERT_TRUE(result.isErr());
  EXPECT_NE(
      result.error().message.find("DART PointCloudShape"), std::string::npos);
  EXPECT_NE(
      result.error().message.find("no point-cloud geometry primitive"),
      std::string::npos);
}

//==============================================================================
TEST(SdfWriter, LineSegmentShapeReturnsExplicitUnsupportedError)
{
  auto skeleton = dynamics::Skeleton::create("line_segment_shape");
  auto [joint, body]
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>();
  (void)joint;
  body->setName("body");
  body->createShapeNodeWith<dynamics::VisualAspect>(
      std::make_shared<dynamics::LineSegmentShape>(
          Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX(), 0.01F),
      "line_segment");

  const auto result = utils::SdfParser::tryWriteSkeletonToString(*skeleton);
  ASSERT_TRUE(result.isErr());
  EXPECT_NE(
      result.error().message.find("DART LineSegmentShape"), std::string::npos);
  EXPECT_NE(
      result.error().message.find("no line-segment geometry primitive"),
      std::string::npos);
}

//==============================================================================
TEST(SdfWriter, VoxelGridShapeReturnsExplicitUnsupportedError)
{
  auto skeleton = dynamics::Skeleton::create("voxel_grid_shape");
  auto [joint, body]
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>();
  (void)joint;
  body->setName("body");
  body->createShapeNodeWith<dynamics::VisualAspect>(
      std::make_shared<dynamics::VoxelGridShape>(0.05), "voxel_grid");

  const auto result = utils::SdfParser::tryWriteSkeletonToString(*skeleton);
  ASSERT_TRUE(result.isErr());
  EXPECT_NE(
      result.error().message.find("DART VoxelGridShape"), std::string::npos);
  EXPECT_NE(
      result.error().message.find("no occupancy-grid geometry primitive"),
      std::string::npos);
}

//==============================================================================
TEST(SdfWriter, MultiSphereConvexHullShapeReturnsGeneratedMeshError)
{
  auto skeleton = dynamics::Skeleton::create("multi_sphere_convex_hull_shape");
  auto [joint, body]
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>();
  (void)joint;
  body->setName("body");
  const dynamics::MultiSphereConvexHullShape::Spheres spheres
      = {{0.1, Eigen::Vector3d::Zero()},
         {0.1, Eigen::Vector3d::UnitX()},
         {0.1, Eigen::Vector3d::UnitY()}};
  body->createShapeNodeWith<dynamics::CollisionAspect>(
      std::make_shared<dynamics::MultiSphereConvexHullShape>(spheres),
      "multi_sphere");

  const auto result = utils::SdfParser::tryWriteSkeletonToString(*skeleton);
  ASSERT_TRUE(result.isErr());
  EXPECT_NE(
      result.error().message.find("DART MultiSphereConvexHullShape"),
      std::string::npos);
  EXPECT_NE(
      result.error().message.find("generated mesh resources"),
      std::string::npos);
}

//==============================================================================
TEST(SdfWriter, SoftBodyNodeReturnsExplicitUnsupportedError)
{
  auto skeleton = dynamics::Skeleton::create("soft_body_writer");

  dynamics::SoftBodyNode::UniqueProperties softProperties;
  softProperties.addPointMass(
      dynamics::PointMass::Properties(Eigen::Vector3d::Zero(), 0.1));
  softProperties.addPointMass(
      dynamics::PointMass::Properties(Eigen::Vector3d(0.1, 0.0, 0.0), 0.1));
  softProperties.connectPointMasses(0, 1);

  dynamics::BodyNode::Properties bodyProperties;
  bodyProperties.mName = "soft_body";
  bodyProperties.mInertia.setMass(0.2);
  dynamics::SoftBodyNode::Properties properties(bodyProperties, softProperties);
  auto [joint, body] = skeleton->createJointAndBodyNodePair<
      dynamics::FreeJoint,
      dynamics::SoftBodyNode>(
      nullptr, dynamics::FreeJoint::Properties(), properties);
  (void)joint;
  ASSERT_NE(body, nullptr);

  const auto result = utils::SdfParser::tryWriteSkeletonToString(*skeleton);
  ASSERT_TRUE(result.isErr());
  EXPECT_NE(
      result.error().message.find("DART SoftBodyNode"), std::string::npos);
}

//==============================================================================
TEST(SdfWriter, PlaneShapeReturnsExplicitUnsupportedError)
{
  auto skeleton = dynamics::Skeleton::create("plane_shape");
  auto [joint, body]
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>();
  (void)joint;
  body->setName("body");
  body->createShapeNodeWith<dynamics::VisualAspect>(
      std::make_shared<dynamics::PlaneShape>(Eigen::Vector3d::UnitZ(), 0.0),
      "plane");

  const auto result = utils::SdfParser::tryWriteSkeletonToString(*skeleton);
  ASSERT_TRUE(result.isErr());
  EXPECT_NE(
      result.error().message.find("finite SDF plane size"), std::string::npos);
}

//==============================================================================
TEST(SdfWriter, HeightmapShapeReturnsSourceUriPolicyError)
{
  auto skeleton = dynamics::Skeleton::create("heightmap_shape");
  auto [joint, body]
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>();
  (void)joint;
  body->setName("body");

  auto heightmap = std::make_shared<dynamics::HeightmapShaped>();
  dynamics::HeightmapShaped::HeightField heights(2, 2);
  heights << 0.0, 0.1, 0.2, 0.3;
  heightmap->setHeightField(heights);
  body->createShapeNodeWith<dynamics::VisualAspect>(
      std::move(heightmap), "heightmap");

  const auto result = utils::SdfParser::tryWriteSkeletonToString(*skeleton);
  ASSERT_TRUE(result.isErr());
  EXPECT_NE(
      result.error().message.find("SDF heightmap geometry"), std::string::npos);
  EXPECT_NE(
      result.error().message.find("source heightmap URI"), std::string::npos);
}

//==============================================================================
TEST(SdfWriter, ConvexMeshShapeReturnsGeneratedMeshError)
{
  auto skeleton = dynamics::Skeleton::create("convex_mesh_shape");
  auto [joint, body]
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>();
  (void)joint;
  body->setName("body");
  body->createShapeNodeWith<dynamics::VisualAspect>(
      std::make_shared<dynamics::ConvexMeshShape>(makeTriangleMesh()),
      "convex_mesh");

  const auto result = utils::SdfParser::tryWriteSkeletonToString(*skeleton);
  ASSERT_TRUE(result.isErr());
  EXPECT_NE(
      result.error().message.find("target SDF URI for generated mesh"),
      std::string::npos);
}

//==============================================================================
TEST(SdfWriter, MeshWithoutUriReturnsError)
{
  auto skeleton = dynamics::Skeleton::create("missing_mesh_uri");
  auto [joint, body]
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>();
  (void)joint;
  body->setName("body");
  body->createShapeNodeWith<dynamics::VisualAspect>(
      std::make_shared<dynamics::MeshShape>(
          Eigen::Vector3d::Ones(), makeTriangleMesh()),
      "mesh_without_uri");

  const auto result = utils::SdfParser::tryWriteSkeletonToString(*skeleton);
  ASSERT_TRUE(result.isErr());
  EXPECT_NE(
      result.error().message.find("without a mesh URI"), std::string::npos);
}

//==============================================================================
TEST(SdfWriter, RelativeMeshUriReturnsError)
{
  auto retriever = std::make_shared<MapResourceRetriever>();
  addTriangleObj(*retriever, "meshes/triangle.obj");
  addTriangleObj(*retriever, "file://meshes/triangle.obj");

  auto skeleton = dynamics::Skeleton::create("relative_mesh_uri");
  auto [joint, body]
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>();
  (void)joint;
  body->setName("body");
  body->createShapeNodeWith<dynamics::VisualAspect>(
      std::make_shared<dynamics::MeshShape>(
          Eigen::Vector3d::Ones(),
          makeTriangleMesh(),
          common::Uri::createFromString("meshes/triangle.obj"),
          retriever),
      "relative_mesh_uri");

  const auto result = utils::SdfParser::tryWriteSkeletonToString(*skeleton);
  ASSERT_TRUE(result.isErr());
  EXPECT_NE(
      result.error().message.find("relative mesh URI"), std::string::npos);
}

//==============================================================================
TEST(SdfWriter, RelativeFileMeshUriReturnsError)
{
  auto retriever = std::make_shared<MapResourceRetriever>();
  addTriangleObj(*retriever, "file:meshes/triangle.obj");

  auto skeleton = dynamics::Skeleton::create("relative_file_mesh_uri");
  auto [joint, body]
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>();
  (void)joint;
  body->setName("body");
  body->createShapeNodeWith<dynamics::VisualAspect>(
      std::make_shared<dynamics::MeshShape>(
          Eigen::Vector3d::Ones(),
          makeTriangleMesh(),
          common::Uri::createFromString("file:meshes/triangle.obj"),
          retriever),
      "relative_file_mesh_uri");

  const auto result = utils::SdfParser::tryWriteSkeletonToString(*skeleton);
  ASSERT_TRUE(result.isErr());
  EXPECT_NE(
      result.error().message.find("relative or host-qualified file URI"),
      std::string::npos);
}

//==============================================================================
TEST(SdfWriter, HostQualifiedFileMeshUriReturnsError)
{
  auto retriever = std::make_shared<MapResourceRetriever>();
  addTriangleObj(*retriever, "file://meshes/triangle.obj");

  auto skeleton = dynamics::Skeleton::create("host_qualified_file_mesh_uri");
  auto [joint, body]
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>();
  (void)joint;
  body->setName("body");
  body->createShapeNodeWith<dynamics::VisualAspect>(
      std::make_shared<dynamics::MeshShape>(
          Eigen::Vector3d::Ones(),
          makeTriangleMesh(),
          common::Uri("meshes/triangle.obj"),
          retriever),
      "host_qualified_file_mesh_uri");

  const auto result = utils::SdfParser::tryWriteSkeletonToString(*skeleton);
  ASSERT_TRUE(result.isErr());
  EXPECT_NE(
      result.error().message.find("relative or host-qualified file URI"),
      std::string::npos);
}

//==============================================================================
TEST(SdfWriter, NonFiniteMeshScaleReturnsError)
{
  auto skeleton = dynamics::Skeleton::create("non_finite_mesh_scale");
  auto [joint, body]
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>();
  (void)joint;
  body->setName("body");

  const common::Uri meshUri
      = common::Uri::createFromPath(dart::config::dataPath("obj/BoxSmall.obj"));
  body->createShapeNodeWith<dynamics::VisualAspect>(
      std::make_shared<dynamics::MeshShape>(
          Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(), 1.0, 1.0),
          makeTriangleMesh(),
          meshUri),
      "non_finite_mesh_scale");

  const auto result = utils::SdfParser::tryWriteSkeletonToString(*skeleton);
  ASSERT_TRUE(result.isErr());
  EXPECT_NE(result.error().message.find("non-finite scale"), std::string::npos);
}

//==============================================================================
TEST(SdfWriter, MeshColorModeReturnsError)
{
  auto skeleton = dynamics::Skeleton::create("mesh_color_mode");
  auto [joint, body]
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>();
  (void)joint;
  body->setName("body");

  const common::Uri meshUri
      = common::Uri::createFromPath(dart::config::dataPath("obj/BoxSmall.obj"));
  auto mesh = std::make_shared<dynamics::MeshShape>(
      Eigen::Vector3d::Ones(), makeTriangleMesh(), meshUri);
  mesh->setColorMode(dynamics::MeshShape::COLOR_INDEX);
  body->createShapeNodeWith<dynamics::VisualAspect>(
      std::move(mesh), "mesh_color_mode");

  const auto result = utils::SdfParser::tryWriteSkeletonToString(*skeleton);
  ASSERT_TRUE(result.isErr());
  EXPECT_NE(result.error().message.find("mesh color mode"), std::string::npos);
}

//==============================================================================
TEST(SdfWriter, MeshAlphaModeReturnsError)
{
  auto skeleton = dynamics::Skeleton::create("mesh_alpha_mode");
  auto [joint, body]
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>();
  (void)joint;
  body->setName("body");

  const common::Uri meshUri
      = common::Uri::createFromPath(dart::config::dataPath("obj/BoxSmall.obj"));
  auto mesh = std::make_shared<dynamics::MeshShape>(
      Eigen::Vector3d::Ones(), makeTriangleMesh(), meshUri);
  mesh->setAlphaMode(dynamics::MeshShape::SHAPE_ALPHA);
  body->createShapeNodeWith<dynamics::VisualAspect>(
      std::move(mesh), "mesh_alpha_mode");

  const auto result = utils::SdfParser::tryWriteSkeletonToString(*skeleton);
  ASSERT_TRUE(result.isErr());
  EXPECT_NE(result.error().message.find("mesh alpha mode"), std::string::npos);
}

//==============================================================================
TEST(SdfWriter, InvalidCollisionSurfaceFrictionReturnsError)
{
  auto skeleton = dynamics::Skeleton::create("bad_surface_friction");
  auto [joint, body]
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>();
  (void)joint;
  body->setName("body");
  auto* shapeNode = body->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(
      std::make_shared<dynamics::BoxShape>(Eigen::Vector3d::Ones()),
      "bad_surface");
  shapeNode->getDynamicsAspect()->setPrimaryFrictionCoeff(-0.1);

  const auto result = utils::SdfParser::tryWriteSkeletonToString(*skeleton);
  ASSERT_TRUE(result.isErr());
  EXPECT_NE(
      result.error().message.find("negative friction"), std::string::npos);
}

//==============================================================================
TEST(SdfWriter, InvalidCollisionSurfaceRestitutionReturnsError)
{
  auto skeleton = dynamics::Skeleton::create("bad_surface_restitution");
  auto [joint, body]
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>();
  (void)joint;
  body->setName("body");
  auto* shapeNode = body->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(
      std::make_shared<dynamics::BoxShape>(Eigen::Vector3d::Ones()),
      "bad_surface");
  shapeNode->getDynamicsAspect()->setRestitutionCoeff(1.1);

  const auto result = utils::SdfParser::tryWriteSkeletonToString(*skeleton);
  ASSERT_TRUE(result.isErr());
  EXPECT_NE(
      result.error().message.find("out-of-range restitution"),
      std::string::npos);
}

//==============================================================================
TEST(SdfWriter, NonFiniteCollisionSurfaceSlipReturnsError)
{
  auto skeleton = dynamics::Skeleton::create("bad_surface_slip");
  auto [joint, body]
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>();
  (void)joint;
  body->setName("body");
  auto* shapeNode = body->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(
      std::make_shared<dynamics::BoxShape>(Eigen::Vector3d::Ones()),
      "bad_surface");
  shapeNode->getDynamicsAspect()->setPrimarySlipCompliance(
      std::numeric_limits<double>::quiet_NaN());

  const auto result = utils::SdfParser::tryWriteSkeletonToString(*skeleton);
  ASSERT_TRUE(result.isErr());
  EXPECT_NE(
      result.error().message.find("non-finite slip compliance"),
      std::string::npos);
}

//==============================================================================
TEST(SdfWriter, NonCollisionFrameFrictionDirectionReturnsError)
{
  auto skeleton = dynamics::Skeleton::create("bad_surface_friction_frame");
  auto [joint, body]
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>();
  (void)joint;
  body->setName("body");
  auto* shapeNode = body->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(
      std::make_shared<dynamics::BoxShape>(Eigen::Vector3d::Ones()),
      "bad_surface");
  auto* dynamicsAspect = shapeNode->getDynamicsAspect();
  ASSERT_NE(dynamicsAspect, nullptr);
  dynamicsAspect->setFirstFrictionDirection(Eigen::Vector3d::UnitX());
  dynamicsAspect->setFirstFrictionDirectionFrame(dynamics::Frame::World());

  const auto result = utils::SdfParser::tryWriteSkeletonToString(*skeleton);
  ASSERT_TRUE(result.isErr());
  EXPECT_NE(
      result.error().message.find("non-collision frame"), std::string::npos);
}

//==============================================================================
TEST(SdfWriter, NonFiniteMaterialColorReturnsError)
{
  auto skeleton = dynamics::Skeleton::create("non_finite_material");
  auto [joint, body]
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>();
  (void)joint;
  body->setName("body");

  auto* visual = body->createShapeNodeWith<dynamics::VisualAspect>(
      std::make_shared<dynamics::BoxShape>(Eigen::Vector3d::Ones()),
      "bad_material");
  visual->getVisualAspect()->setRGBA(
      Eigen::Vector4d(0.1, std::numeric_limits<double>::quiet_NaN(), 0.3, 1.0));

  const auto result = utils::SdfParser::tryWriteSkeletonToString(*skeleton);
  ASSERT_TRUE(result.isErr());
  EXPECT_NE(
      result.error().message.find("non-finite material color"),
      std::string::npos);
}

//==============================================================================
TEST(SdfWriter, InvalidVisualTransparencyReturnsError)
{
  auto skeleton = dynamics::Skeleton::create("bad_visual_transparency");
  auto [joint, body]
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>();
  (void)joint;
  body->setName("body");

  auto* visual = body->createShapeNodeWith<dynamics::VisualAspect>(
      std::make_shared<dynamics::BoxShape>(Eigen::Vector3d::Ones()),
      "bad_transparency");
  visual->getVisualAspect()->setAlpha(-0.1);

  const auto result = utils::SdfParser::tryWriteSkeletonToString(*skeleton);
  ASSERT_TRUE(result.isErr());
  EXPECT_NE(
      result.error().message.find("visual transparency"), std::string::npos);
}

//==============================================================================
TEST(SdfWriter, InvalidPbrMaterialFactorReturnsError)
{
  auto skeleton = dynamics::Skeleton::create("bad_pbr");
  dynamics::FreeJoint::Properties rootProperties;
  rootProperties.mName = "root";
  dynamics::BodyNode::Properties bodyProperties;
  bodyProperties.mName = "body";
  bodyProperties.mInertia.setMass(1.0);
  bodyProperties.mInertia.setMoment(Eigen::Matrix3d::Identity());
  auto [rootJoint, body]
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>(
          nullptr, rootProperties, bodyProperties);
  (void)rootJoint;
  auto* visual = body->createShapeNodeWith<dynamics::VisualAspect>(
      std::make_shared<dynamics::BoxShape>(Eigen::Vector3d::Ones()),
      "bad_pbr_visual");
  visual->getVisualAspect()->setMetallic(
      std::numeric_limits<double>::quiet_NaN());

  const auto writeResult
      = utils::SdfParser::tryWriteSkeletonToString(*skeleton);
  ASSERT_TRUE(writeResult.isErr());
  EXPECT_NE(writeResult.error().message.find("PBR factor"), std::string::npos);
}

//==============================================================================
TEST(SdfWriter, UnsupportedVisualReflectanceReturnsError)
{
  auto skeleton = dynamics::Skeleton::create("unsupported_reflectance");
  auto [joint, body]
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>();
  (void)joint;
  body->setName("body");

  auto* visual = body->createShapeNodeWith<dynamics::VisualAspect>(
      std::make_shared<dynamics::BoxShape>(Eigen::Vector3d::Ones()),
      "reflective_visual");
  visual->getVisualAspect()->setReflectance(0.4);

  const auto writeResult
      = utils::SdfParser::tryWriteSkeletonToString(*skeleton);
  ASSERT_TRUE(writeResult.isErr());
  EXPECT_NE(writeResult.error().message.find("reflectance"), std::string::npos);
}

//==============================================================================
TEST(SdfWriter, NonFiniteSkeletonGravityReturnsError)
{
  auto skeleton = dynamics::Skeleton::create("non_finite_skeleton_gravity");
  auto [rootJoint, body]
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>();
  (void)rootJoint;
  body->setName("body");
  skeleton->setGravity(
      Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(), 0.0, -9.81));

  const auto result = utils::SdfParser::tryWriteSkeletonToString(*skeleton);
  ASSERT_TRUE(result.isErr());
  EXPECT_NE(
      result.error().message.find("non-finite skeleton gravity"),
      std::string::npos);
}

//==============================================================================
TEST(SdfWriter, NonFiniteShapePoseReturnsError)
{
  auto skeleton = dynamics::Skeleton::create("non_finite_shape_pose");
  auto [joint, body]
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>();
  (void)joint;
  body->setName("body");

  auto* visual = body->createShapeNodeWith<dynamics::VisualAspect>(
      std::make_shared<dynamics::BoxShape>(Eigen::Vector3d::Ones()),
      "bad_pose_visual");
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation().x() = std::numeric_limits<double>::quiet_NaN();
  visual->setRelativeTransform(pose);

  const auto result = utils::SdfParser::tryWriteSkeletonToString(*skeleton);
  ASSERT_TRUE(result.isErr());
  EXPECT_NE(result.error().message.find("non-finite pose"), std::string::npos);
}

//==============================================================================
TEST(SdfWriter, NonFiniteInertialDataReturnsError)
{
  auto skeleton = dynamics::Skeleton::create("non_finite_inertial_data");
  auto [joint, body]
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>();
  (void)joint;
  body->setName("body");
  body->setLocalCOM(
      Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0));

  const auto result = utils::SdfParser::tryWriteSkeletonToString(*skeleton);
  ASSERT_TRUE(result.isErr());
  EXPECT_NE(
      result.error().message.find("non-finite inertial data"),
      std::string::npos);
}

//==============================================================================
TEST(SdfWriter, NonPositiveMassReturnsError)
{
  auto skeleton = dynamics::Skeleton::create("non_positive_mass");
  auto [joint, body]
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>();
  (void)joint;
  body->setName("body");
  body->setMass(0.0);

  const auto result = utils::SdfParser::tryWriteSkeletonToString(*skeleton);
  ASSERT_TRUE(result.isErr());
  EXPECT_NE(result.error().message.find("SDF link [body]"), std::string::npos);
  EXPECT_NE(
      result.error().message.find("non-finite or non-positive mass"),
      std::string::npos);
}

//==============================================================================
TEST(SdfWriter, NonFiniteJointAxisReturnsError)
{
  auto skeleton = dynamics::Skeleton::create("non_finite_joint_axis");
  auto [rootJoint, base]
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>();
  (void)rootJoint;
  base->setName("base");

  dynamics::RevoluteJoint::Properties hingeProperties;
  hingeProperties.mName = "hinge";
  hingeProperties.mAxis = Eigen::Vector3d::UnitZ();

  dynamics::BodyNode::Properties tipProperties;
  tipProperties.mName = "tip";

  auto [hinge, tip]
      = skeleton->createJointAndBodyNodePair<dynamics::RevoluteJoint>(
          base, hingeProperties, tipProperties);
  (void)tip;
  hinge->setAxis(
      Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(), 0.0, 1.0));

  const auto result = utils::SdfParser::tryWriteSkeletonToString(*skeleton);
  ASSERT_TRUE(result.isErr());
  EXPECT_NE(result.error().message.find("non-finite axis"), std::string::npos);
}

//==============================================================================
TEST(SdfWriter, AsymmetricJointVelocityLimitReturnsError)
{
  dynamics::SkeletonPtr skeleton;
  auto* hinge = createLimitProbeJoint(skeleton, "asymmetric_velocity_limit");
  ASSERT_NE(hinge, nullptr);
  hinge->setVelocityLowerLimit(0, -1.0);
  hinge->setVelocityUpperLimit(0, 2.0);

  const auto result = utils::SdfParser::tryWriteSkeletonToString(*skeleton);
  ASSERT_TRUE(result.isErr());
  EXPECT_NE(
      result.error().message.find("asymmetric velocity limits"),
      std::string::npos)
      << result.error().message;
}

//==============================================================================
TEST(SdfWriter, AsymmetricJointForceLimitReturnsError)
{
  dynamics::SkeletonPtr skeleton;
  auto* hinge = createLimitProbeJoint(skeleton, "asymmetric_force_limit");
  ASSERT_NE(hinge, nullptr);
  hinge->setForceLowerLimit(0, -1.0);
  hinge->setForceUpperLimit(0, 2.0);

  const auto result = utils::SdfParser::tryWriteSkeletonToString(*skeleton);
  ASSERT_TRUE(result.isErr());
  EXPECT_NE(
      result.error().message.find("asymmetric force/effort limits"),
      std::string::npos)
      << result.error().message;
}

//==============================================================================
TEST(SdfWriter, NaNJointVelocityLimitReturnsError)
{
  dynamics::SkeletonPtr skeleton;
  auto* hinge = createLimitProbeJoint(skeleton, "nan_velocity_limit");
  ASSERT_NE(hinge, nullptr);
  hinge->setVelocityLowerLimit(0, std::numeric_limits<double>::quiet_NaN());

  const auto result = utils::SdfParser::tryWriteSkeletonToString(*skeleton);
  ASSERT_TRUE(result.isErr());
  EXPECT_NE(
      result.error().message.find("NaN velocity limits"), std::string::npos)
      << result.error().message;
}

//==============================================================================
TEST(SdfWriter, NaNJointForceLimitReturnsError)
{
  dynamics::SkeletonPtr skeleton;
  auto* hinge = createLimitProbeJoint(skeleton, "nan_force_limit");
  ASSERT_NE(hinge, nullptr);
  hinge->setForceUpperLimit(0, std::numeric_limits<double>::quiet_NaN());

  const auto result = utils::SdfParser::tryWriteSkeletonToString(*skeleton);
  ASSERT_TRUE(result.isErr());
  EXPECT_NE(
      result.error().message.find("NaN force/effort limits"), std::string::npos)
      << result.error().message;
}

//==============================================================================
TEST(SdfWriter, NonFiniteJointDynamicsReturnsError)
{
  auto skeleton = dynamics::Skeleton::create("non_finite_joint_dynamics");
  auto [rootJoint, base]
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>();
  (void)rootJoint;
  base->setName("base");

  dynamics::RevoluteJoint::Properties hingeProperties;
  hingeProperties.mName = "hinge";
  hingeProperties.mAxis = Eigen::Vector3d::UnitZ();

  dynamics::BodyNode::Properties tipProperties;
  tipProperties.mName = "tip";

  auto [hinge, tip]
      = skeleton->createJointAndBodyNodePair<dynamics::RevoluteJoint>(
          base, hingeProperties, tipProperties);
  (void)tip;
  hinge->setRestPosition(0, std::numeric_limits<double>::quiet_NaN());

  const auto result = utils::SdfParser::tryWriteSkeletonToString(*skeleton);
  ASSERT_TRUE(result.isErr());
  EXPECT_NE(
      result.error().message.find("non-finite dynamics"), std::string::npos);
}

//==============================================================================
TEST(SdfWriter, NaNJointLimitReturnsError)
{
  auto skeleton = dynamics::Skeleton::create("nan_joint_limit");
  auto [rootJoint, base]
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>();
  (void)rootJoint;
  base->setName("base");

  dynamics::RevoluteJoint::Properties hingeProperties;
  hingeProperties.mName = "hinge";
  hingeProperties.mAxis = Eigen::Vector3d::UnitZ();

  dynamics::BodyNode::Properties tipProperties;
  tipProperties.mName = "tip";

  auto [hinge, tip]
      = skeleton->createJointAndBodyNodePair<dynamics::RevoluteJoint>(
          base, hingeProperties, tipProperties);
  (void)tip;
  hinge->setPositionLowerLimit(0, std::numeric_limits<double>::quiet_NaN());

  const auto result = utils::SdfParser::tryWriteSkeletonToString(*skeleton);
  ASSERT_TRUE(result.isErr());
  EXPECT_NE(
      result.error().message.find("NaN position limits"), std::string::npos);
}

//==============================================================================
TEST(SdfWriter, NonFiniteScrewJointPitchReturnsError)
{
  auto skeleton = dynamics::Skeleton::create("non_finite_screw_pitch");
  auto [rootJoint, base]
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>();
  (void)rootJoint;
  base->setName("base");

  dynamics::ScrewJoint::Properties screwProperties;
  screwProperties.mName = "screw";
  screwProperties.mAxis = Eigen::Vector3d::UnitZ();

  dynamics::BodyNode::Properties tipProperties;
  tipProperties.mName = "tip";

  auto [screw, tip]
      = skeleton->createJointAndBodyNodePair<dynamics::ScrewJoint>(
          base, screwProperties, tipProperties);
  (void)tip;
  screw->setPitch(std::numeric_limits<double>::quiet_NaN());

  const auto result = utils::SdfParser::tryWriteSkeletonToString(*skeleton);
  ASSERT_TRUE(result.isErr());
  EXPECT_NE(result.error().message.find("non-finite pitch"), std::string::npos);
}

//==============================================================================
TEST(SdfWriter, BallJointDynamicsReturnError)
{
  auto skeleton = dynamics::Skeleton::create("ball_joint_dynamics");
  auto [rootJoint, base]
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>();
  (void)rootJoint;
  base->setName("base");

  dynamics::BallJoint::Properties ballProperties;
  ballProperties.mName = "ball";

  dynamics::BodyNode::Properties tipProperties;
  tipProperties.mName = "tip";

  auto [ball, tip] = skeleton->createJointAndBodyNodePair<dynamics::BallJoint>(
      base, ballProperties, tipProperties);
  (void)tip;
  ball->setDampingCoefficient(0, 0.1);

  const auto result = utils::SdfParser::tryWriteSkeletonToString(*skeleton);
  ASSERT_TRUE(result.isErr());
  EXPECT_NE(
      result.error().message.find("ball joint [ball] with dynamics"),
      std::string::npos);
}

//==============================================================================
TEST(SdfWriter, NaNBallJointLimitReturnsError)
{
  auto skeleton = dynamics::Skeleton::create("nan_ball_joint_limit");
  auto [rootJoint, base]
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>();
  (void)rootJoint;
  base->setName("base");

  dynamics::BallJoint::Properties ballProperties;
  ballProperties.mName = "ball";

  dynamics::BodyNode::Properties tipProperties;
  tipProperties.mName = "tip";

  auto [ball, tip] = skeleton->createJointAndBodyNodePair<dynamics::BallJoint>(
      base, ballProperties, tipProperties);
  (void)tip;
  ball->setPositionLowerLimit(0, std::numeric_limits<double>::quiet_NaN());

  const auto result = utils::SdfParser::tryWriteSkeletonToString(*skeleton);
  ASSERT_TRUE(result.isErr());
  EXPECT_NE(
      result.error().message.find("NaN position limits"), std::string::npos);
}
