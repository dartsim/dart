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

#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>

using namespace dart;

namespace {

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

void expectVectorNear(
    const Eigen::Vector3d& actual,
    const Eigen::Vector3d& expected,
    double tolerance)
{
  EXPECT_NEAR(actual.x(), expected.x(), tolerance);
  EXPECT_NEAR(actual.y(), expected.y(), tolerance);
  EXPECT_NEAR(actual.z(), expected.z(), tolerance);
}

void expectVectorNear(
    const Eigen::Vector4d& actual,
    const Eigen::Vector4d& expected,
    double tolerance)
{
  EXPECT_NEAR(actual.x(), expected.x(), tolerance);
  EXPECT_NEAR(actual.y(), expected.y(), tolerance);
  EXPECT_NEAR(actual.z(), expected.z(), tolerance);
  EXPECT_NEAR(actual.w(), expected.w(), tolerance);
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

  const auto* base = reparsed->getBodyNode("base");
  const auto* tip = reparsed->getBodyNode("tip");
  const auto* slider = reparsed->getBodyNode("slider");
  const auto* fixed = reparsed->getBodyNode("fixed");
  const auto* screwTip = reparsed->getBodyNode("screw_tip");
  const auto* universalTip = reparsed->getBodyNode("universal_tip");
  const auto* ballTip = reparsed->getBodyNode("ball_tip");
  ASSERT_NE(base, nullptr);
  ASSERT_NE(tip, nullptr);
  ASSERT_NE(slider, nullptr);
  ASSERT_NE(fixed, nullptr);
  ASSERT_NE(screwTip, nullptr);
  ASSERT_NE(universalTip, nullptr);
  ASSERT_NE(ballTip, nullptr);

  const auto* root
      = dynamic_cast<const dynamics::FreeJoint*>(reparsed->getJoint("root"));
  ASSERT_NE(root, nullptr);
  EXPECT_EQ(root->getParentBodyNode(), nullptr);
  EXPECT_EQ(root->getChildBodyNode(), base);
  expectVectorNear(
      root->getTransformFromParentBodyNode().translation(),
      Eigen::Vector3d(0.25, -0.5, 1.0),
      1e-12);

  EXPECT_NEAR(base->getMass(), 2.5, 1e-12);
  expectVectorNear(
      base->getLocalCOM(), Eigen::Vector3d(0.01, 0.02, 0.03), 1e-12);

  const auto baseMoment = base->getInertia().getMoment();
  EXPECT_NEAR(baseMoment(0, 0), 0.2, 1e-12);
  EXPECT_NEAR(baseMoment(1, 1), 0.3, 1e-12);
  EXPECT_NEAR(baseMoment(2, 2), 0.4, 1e-12);
  EXPECT_NEAR(baseMoment(0, 1), 0.01, 1e-12);
  EXPECT_NEAR(baseMoment(0, 2), 0.02, 1e-12);
  EXPECT_NEAR(baseMoment(1, 2), 0.03, 1e-12);

  const auto* hinge = dynamic_cast<const dynamics::RevoluteJoint*>(
      reparsed->getJoint("hinge"));
  ASSERT_NE(hinge, nullptr);
  EXPECT_EQ(hinge->getParentBodyNode(), base);
  EXPECT_EQ(hinge->getChildBodyNode(), tip);
  expectVectorNear(
      hinge->getTransformFromParentBodyNode().translation(),
      Eigen::Vector3d(0.0, 0.0, 0.8),
      1e-12);
  expectVectorNear(
      hinge->getTransformFromChildBodyNode().translation(),
      Eigen::Vector3d(0.0, 0.0, -0.2),
      1e-12);
  expectVectorNear(hinge->getAxis(), Eigen::Vector3d::UnitZ(), 1e-12);
  EXPECT_NEAR(hinge->getPositionLowerLimit(0), -1.25, 1e-12);
  EXPECT_NEAR(hinge->getPositionUpperLimit(0), 1.5, 1e-12);
  EXPECT_NEAR(hinge->getDampingCoefficient(0), 0.25, 1e-12);
  EXPECT_NEAR(hinge->getCoulombFriction(0), 0.125, 1e-12);
  EXPECT_NEAR(hinge->getRestPosition(0), -0.5, 1e-12);
  EXPECT_NEAR(hinge->getSpringStiffness(0), 3.75, 1e-12);

  const auto* sliderJoint = dynamic_cast<const dynamics::PrismaticJoint*>(
      reparsed->getJoint("slider_joint"));
  ASSERT_NE(sliderJoint, nullptr);
  EXPECT_EQ(sliderJoint->getParentBodyNode(), tip);
  EXPECT_EQ(sliderJoint->getChildBodyNode(), slider);
  expectVectorNear(sliderJoint->getAxis(), Eigen::Vector3d::UnitX(), 1e-12);
  EXPECT_NEAR(sliderJoint->getPositionLowerLimit(0), 0.1, 1e-12);
  EXPECT_NEAR(sliderJoint->getPositionUpperLimit(0), 0.9, 1e-12);

  const auto* fixedJoint = dynamic_cast<const dynamics::WeldJoint*>(
      reparsed->getJoint("fixed_tip"));
  ASSERT_NE(fixedJoint, nullptr);
  EXPECT_EQ(fixedJoint->getParentBodyNode(), slider);
  EXPECT_EQ(fixedJoint->getChildBodyNode(), fixed);
  EXPECT_FALSE(fixed->getGravityMode());

  const auto* screwJoint = dynamic_cast<const dynamics::ScrewJoint*>(
      reparsed->getJoint("screw_drive"));
  ASSERT_NE(screwJoint, nullptr);
  EXPECT_EQ(screwJoint->getParentBodyNode(), fixed);
  EXPECT_EQ(screwJoint->getChildBodyNode(), screwTip);
  expectVectorNear(screwJoint->getAxis(), Eigen::Vector3d::UnitY(), 1e-12);
  EXPECT_NEAR(screwJoint->getPitch(), 0.25, 1e-12);
  EXPECT_NEAR(screwJoint->getPositionLowerLimit(0), -0.75, 1e-12);
  EXPECT_NEAR(screwJoint->getPositionUpperLimit(0), 0.75, 1e-12);
  EXPECT_NEAR(screwJoint->getDampingCoefficient(0), 0.2, 1e-12);
  EXPECT_NEAR(screwJoint->getCoulombFriction(0), 0.05, 1e-12);
  EXPECT_NEAR(screwJoint->getRestPosition(0), 0.1, 1e-12);
  EXPECT_NEAR(screwJoint->getSpringStiffness(0), 1.5, 1e-12);

  const auto* universalJoint = dynamic_cast<const dynamics::UniversalJoint*>(
      reparsed->getJoint("universal_shoulder"));
  ASSERT_NE(universalJoint, nullptr);
  EXPECT_EQ(universalJoint->getParentBodyNode(), screwTip);
  EXPECT_EQ(universalJoint->getChildBodyNode(), universalTip);
  expectVectorNear(universalJoint->getAxis1(), Eigen::Vector3d::UnitX(), 1e-12);
  expectVectorNear(universalJoint->getAxis2(), Eigen::Vector3d::UnitZ(), 1e-12);
  EXPECT_NEAR(universalJoint->getPositionLowerLimit(0), -0.4, 1e-12);
  EXPECT_NEAR(universalJoint->getPositionUpperLimit(0), 0.6, 1e-12);
  EXPECT_NEAR(universalJoint->getDampingCoefficient(0), 0.3, 1e-12);
  EXPECT_NEAR(universalJoint->getCoulombFriction(0), 0.06, 1e-12);
  EXPECT_NEAR(universalJoint->getRestPosition(0), -0.2, 1e-12);
  EXPECT_NEAR(universalJoint->getSpringStiffness(0), 2.5, 1e-12);
  EXPECT_NEAR(universalJoint->getPositionLowerLimit(1), -1.25, 1e-12);
  EXPECT_NEAR(universalJoint->getPositionUpperLimit(1), 1.5, 1e-12);
  EXPECT_NEAR(universalJoint->getDampingCoefficient(1), 0.45, 1e-12);
  EXPECT_NEAR(universalJoint->getCoulombFriction(1), 0.08, 1e-12);
  EXPECT_NEAR(universalJoint->getRestPosition(1), 0.35, 1e-12);
  EXPECT_NEAR(universalJoint->getSpringStiffness(1), 4.5, 1e-12);

  const auto* ballJoint = dynamic_cast<const dynamics::BallJoint*>(
      reparsed->getJoint("ball_socket"));
  ASSERT_NE(ballJoint, nullptr);
  EXPECT_EQ(ballJoint->getParentBodyNode(), universalTip);
  EXPECT_EQ(ballJoint->getChildBodyNode(), ballTip);
  EXPECT_EQ(ballJoint->getNumDofs(), 3u);

  ASSERT_EQ(base->getNumShapeNodesWith<dynamics::VisualAspect>(), 1u);
  ASSERT_EQ(base->getNumShapeNodesWith<dynamics::CollisionAspect>(), 1u);
  const auto* baseBox = dynamic_cast<const dynamics::BoxShape*>(
      base->getShapeNodeWith<dynamics::VisualAspect>(0)->getShape().get());
  ASSERT_NE(baseBox, nullptr);
  expectVectorNear(baseBox->getSize(), Eigen::Vector3d(0.4, 0.5, 0.6), 1e-12);
  expectVectorNear(
      base->getShapeNodeWith<dynamics::VisualAspect>(0)
          ->getRelativeTransform()
          .translation(),
      Eigen::Vector3d(0.1, 0.0, 0.0),
      1e-12);
  expectVectorNear(
      base->getShapeNodeWith<dynamics::VisualAspect>(0)
          ->getVisualAspect()
          ->getRGBA(),
      Eigen::Vector4d(0.2, 0.4, 0.6, 0.8),
      1e-12);

  ASSERT_EQ(tip->getNumShapeNodesWith<dynamics::VisualAspect>(), 1u);
  ASSERT_EQ(tip->getNumShapeNodesWith<dynamics::CollisionAspect>(), 1u);
  const auto* tipSphere = dynamic_cast<const dynamics::SphereShape*>(
      tip->getShapeNodeWith<dynamics::VisualAspect>(0)->getShape().get());
  ASSERT_NE(tipSphere, nullptr);
  EXPECT_NEAR(tipSphere->getRadius(), 0.15, 1e-12);

  const auto* tipCylinder = dynamic_cast<const dynamics::CylinderShape*>(
      tip->getShapeNodeWith<dynamics::CollisionAspect>(0)->getShape().get());
  ASSERT_NE(tipCylinder, nullptr);
  EXPECT_NEAR(tipCylinder->getRadius(), 0.1, 1e-12);
  EXPECT_NEAR(tipCylinder->getHeight(), 0.3, 1e-12);

  ASSERT_EQ(slider->getNumShapeNodesWith<dynamics::VisualAspect>(), 1u);
  const auto* sliderMesh = dynamic_cast<const dynamics::MeshShape*>(
      slider->getShapeNodeWith<dynamics::VisualAspect>(0)->getShape().get());
  ASSERT_NE(sliderMesh, nullptr);
  expectVectorNear(
      sliderMesh->getScale(), Eigen::Vector3d(0.5, 1.0, 2.0), 1e-12);
  const common::Uri meshUri
      = common::Uri::createFromPath(dart::config::dataPath("obj/BoxSmall.obj"));
  EXPECT_EQ(sliderMesh->getMeshUri2().toString(), meshUri.toString());

  ASSERT_EQ(fixed->getNumShapeNodesWith<dynamics::CollisionAspect>(), 1u);
  const auto* fixedBox = dynamic_cast<const dynamics::BoxShape*>(
      fixed->getShapeNodeWith<dynamics::CollisionAspect>(0)->getShape().get());
  ASSERT_NE(fixedBox, nullptr);
  expectVectorNear(fixedBox->getSize(), Eigen::Vector3d(0.1, 0.1, 0.1), 1e-12);

  ASSERT_EQ(screwTip->getNumShapeNodesWith<dynamics::CollisionAspect>(), 1u);
  const auto* screwTipSphere = dynamic_cast<const dynamics::SphereShape*>(
      screwTip->getShapeNodeWith<dynamics::CollisionAspect>(0)
          ->getShape()
          .get());
  ASSERT_NE(screwTipSphere, nullptr);
  EXPECT_NEAR(screwTipSphere->getRadius(), 0.08, 1e-12);

  ASSERT_EQ(
      universalTip->getNumShapeNodesWith<dynamics::CollisionAspect>(), 1u);
  const auto* universalTipCylinder
      = dynamic_cast<const dynamics::CylinderShape*>(
          universalTip->getShapeNodeWith<dynamics::CollisionAspect>(0)
              ->getShape()
              .get());
  ASSERT_NE(universalTipCylinder, nullptr);
  EXPECT_NEAR(universalTipCylinder->getRadius(), 0.04, 1e-12);
  EXPECT_NEAR(universalTipCylinder->getHeight(), 0.18, 1e-12);

  ASSERT_EQ(ballTip->getNumShapeNodesWith<dynamics::CollisionAspect>(), 1u);
  const auto* ballTipSphere = dynamic_cast<const dynamics::SphereShape*>(
      ballTip->getShapeNodeWith<dynamics::CollisionAspect>(0)
          ->getShape()
          .get());
  ASSERT_NE(ballTipSphere, nullptr);
  EXPECT_NEAR(ballTipSphere->getRadius(), 0.05, 1e-12);
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
  const auto* root
      = dynamic_cast<const dynamics::WeldJoint*>(reparsed->getJoint(0));
  ASSERT_NE(root, nullptr);
  EXPECT_EQ(root->getParentBodyNode(), nullptr);
  ASSERT_NE(root->getChildBodyNode(), nullptr);
  EXPECT_EQ(root->getChildBodyNode()->getName(), "fixed_root");
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
