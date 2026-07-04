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

#include "dart/common/resource.hpp"
#include "dart/common/resource_retriever.hpp"
#include "dart/utils/sdf/sdf_parser.hpp"
#include "dart/utils/sdf/sdf_writer.hpp"

#include <dart/config.hpp>

#include <dart/dynamics/ball_joint.hpp>
#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/cylinder_shape.hpp>
#include <dart/dynamics/ellipsoid_shape.hpp>
#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/mesh_shape.hpp>
#include <dart/dynamics/prismatic_joint.hpp>
#include <dart/dynamics/revolute_joint.hpp>
#include <dart/dynamics/screw_joint.hpp>
#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/sphere_shape.hpp>
#include <dart/dynamics/universal_joint.hpp>
#include <dart/dynamics/weld_joint.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <utility>

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

std::filesystem::path writeTempSdf(std::string_view text, std::string_view name)
{
  const auto path = std::filesystem::temp_directory_path()
                    / (std::string(name) + "_dart_sdf_writer_test.sdf");
  std::ofstream output(path);
  output << text;
  return path;
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

} // namespace

//==============================================================================
TEST(SdfWriter, RoundTripsSupportedSkeletonSubset)
{
  const auto original = makeRoundTripSkeleton();

  const auto writeResult
      = utils::SdfParser::tryWriteSkeletonToString(*original);
  ASSERT_TRUE(writeResult.isOk()) << writeResult.error().message;
  EXPECT_NE(
      writeResult.value().find("<model name=\"writer_roundtrip\">"),
      std::string::npos);
  EXPECT_NE(
      writeResult.value().find("<gravity>false</gravity>"), std::string::npos);
  EXPECT_NE(
      writeResult.value().find("<joint name=\"screw_drive\" type=\"screw\">"),
      std::string::npos);
  EXPECT_NE(
      writeResult.value().find("<thread_pitch>0.25</thread_pitch>"),
      std::string::npos);
  EXPECT_NE(
      writeResult.value().find(
          "<joint name=\"universal_shoulder\" type=\"universal\">"),
      std::string::npos);
  EXPECT_NE(writeResult.value().find("<axis2>"), std::string::npos);
  EXPECT_NE(
      writeResult.value().find("<joint name=\"ball_socket\" type=\"ball\">"),
      std::string::npos);
  EXPECT_NE(
      writeResult.value().find("<damping>0.25</damping>"), std::string::npos);
  EXPECT_NE(
      writeResult.value().find("<friction>0.125</friction>"),
      std::string::npos);
  EXPECT_NE(
      writeResult.value().find("<spring_reference>-0.5</spring_reference>"),
      std::string::npos);
  EXPECT_NE(
      writeResult.value().find("<spring_stiffness>3.75</spring_stiffness>"),
      std::string::npos);
  EXPECT_NE(
      writeResult.value().find(
          "<diffuse>0.20000000000000001 "
          "0.40000000000000002 "
          "0.59999999999999998 "
          "0.80000000000000004</diffuse>"),
      std::string::npos);

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

  const auto* ballTipSphere
      = test::requireShape<dynamics::CollisionAspect, dynamics::SphereShape>(
          *ballTip, 0, 1);
  ASSERT_NE(ballTipSphere, nullptr);
  EXPECT_NEAR(ballTipSphere->getRadius(), 0.05, 1e-12);
}

//==============================================================================
TEST(SdfWriter, PreservesAbsoluteNonFileMeshUris)
{
  auto retriever = std::make_shared<MapResourceRetriever>();
  const std::string meshUri = "memory://pkg/meshes/writer_triangle.obj";
  retriever->add(
      meshUri,
      R"(
o Triangle
v 0 0 0
v 1 0 0
v 0 1 0
f 1 2 3
)");

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
  EXPECT_NE(
      writeResult.value().find("<uri>" + meshUri + "</uri>"),
      std::string::npos);

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
  EXPECT_NE(
      writeResult.value().find("<static>true</static>"), std::string::npos);

  const auto path = writeTempSdf(writeResult.value(), "root_weld");
  const utils::SdfParser::Options options(
      nullptr, utils::SdfParser::RootJointType::Fixed);
  const auto reparsed = utils::SdfParser::readSkeleton(
      common::Uri::createFromPath(path.string()), options);
  std::filesystem::remove(path);

  ASSERT_NE(reparsed, nullptr);
  ASSERT_EQ(reparsed->getNumBodyNodes(), 1u);
  ASSERT_EQ(reparsed->getNumJoints(), 1u);
  const auto* root = test::requireJoint<dynamics::WeldJoint>(*reparsed, "root");
  ASSERT_NE(root, nullptr);
  const auto* fixedRoot = test::requireBodyNode(*reparsed, "fixed_root");
  ASSERT_NE(fixedRoot, nullptr);
  test::expectJointTopology(*root, nullptr, fixedRoot);
}

//==============================================================================
TEST(SdfWriter, UnsupportedShapeReturnsError)
{
  auto skeleton = dynamics::Skeleton::create("unsupported_shape");
  auto [joint, body]
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>();
  (void)joint;
  body->setName("body");
  body->createShapeNodeWith<dynamics::VisualAspect>(
      std::make_shared<dynamics::EllipsoidShape>(Eigen::Vector3d::Ones()),
      "ellipsoid");

  const auto result = utils::SdfParser::tryWriteSkeletonToString(*skeleton);
  ASSERT_TRUE(result.isErr());
  EXPECT_NE(
      result.error().message.find("Unsupported shape type"), std::string::npos);
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
