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

#include "dart/config.hpp"
#include "dart/utils/urdf/urdf_parser.hpp"

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/cylinder_shape.hpp>
#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/mesh_shape.hpp>
#include <dart/dynamics/planar_joint.hpp>
#include <dart/dynamics/point_mass.hpp>
#include <dart/dynamics/prismatic_joint.hpp>
#include <dart/dynamics/revolute_joint.hpp>
#include <dart/dynamics/shape_node.hpp>
#include <dart/dynamics/soft_body_node.hpp>
#include <dart/dynamics/sphere_shape.hpp>
#include <dart/dynamics/weld_joint.hpp>

#include <dart/math/tri_mesh.hpp>

#include <dart/common/uri.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include <limits>
#include <memory>
#include <string>
#include <string_view>
#include <utility>

using namespace dart;

namespace {

constexpr double kTolerance = 1e-9;
constexpr double kColorTolerance = 1e-7;

std::shared_ptr<math::TriMesh<double>> makeTriangleMesh()
{
  auto mesh = std::make_shared<math::TriMesh<double>>();
  mesh->addVertex(0.0, 0.0, 0.0);
  mesh->addVertex(1.0, 0.0, 0.0);
  mesh->addVertex(0.0, 1.0, 0.0);
  mesh->addTriangle(0, 1, 2);
  return mesh;
}

dynamics::BodyNode::Properties makeBodyProperties(
    std::string name,
    double mass,
    const Eigen::Vector3d& com,
    const Eigen::Vector3d& moment)
{
  dynamics::BodyNode::Properties properties;
  properties.mName = std::move(name);
  properties.mInertia.setMass(mass);
  properties.mInertia.setLocalCOM(com);
  properties.mInertia.setMoment(
      moment.x(), moment.y(), moment.z(), 0.0, 0.0, 0.0);
  return properties;
}

void setFiniteJointMetadata(
    dynamics::Joint* joint,
    double lower,
    double upper,
    double velocity,
    double effort,
    double damping,
    double friction)
{
  ASSERT_NE(joint, nullptr);
  joint->setPositionLowerLimit(0, lower);
  joint->setPositionUpperLimit(0, upper);
  joint->setVelocityLowerLimit(0, -velocity);
  joint->setVelocityUpperLimit(0, velocity);
  joint->setForceLowerLimit(0, -effort);
  joint->setForceUpperLimit(0, effort);
  joint->setDampingCoefficient(0, damping);
  joint->setCoulombFriction(0, friction);
}

void expectContains(std::string_view text, std::string_view needle)
{
  EXPECT_NE(text.find(needle), std::string_view::npos)
      << "Expected [" << text << "] to contain [" << needle << "]";
}

dynamics::SkeletonPtr makeRoundTripSkeleton()
{
  auto skeleton = dynamics::Skeleton::create("urdf_writer_roundtrip");

  dynamics::FreeJoint::Properties rootProperties;
  rootProperties.mName = "root_joint";

  auto [rootJoint, base]
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>(
          nullptr,
          rootProperties,
          makeBodyProperties(
              "base",
              2.0,
              Eigen::Vector3d(0.01, 0.02, 0.03),
              Eigen::Vector3d(0.2, 0.3, 0.4)));
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

  auto [hinge, tip]
      = skeleton->createJointAndBodyNodePair<dynamics::RevoluteJoint>(
          base,
          hingeProperties,
          makeBodyProperties(
              "tip",
              1.25,
              Eigen::Vector3d(-0.01, 0.02, 0.0),
              Eigen::Vector3d(0.1, 0.15, 0.2)));
  setFiniteJointMetadata(hinge, -1.25, 1.5, 3.0, 4.0, 0.25, 0.125);
  tip->createShapeNodeWith<dynamics::VisualAspect>(
      std::make_shared<dynamics::SphereShape>(0.15), "tip_sphere_visual");
  tip->createShapeNodeWith<dynamics::CollisionAspect>(
      std::make_shared<dynamics::CylinderShape>(0.1, 0.3),
      "tip_cylinder_collision");

  dynamics::PrismaticJoint::Properties sliderProperties;
  sliderProperties.mName = "slider";
  sliderProperties.mAxis = Eigen::Vector3d::UnitX();
  sliderProperties.mT_ParentBodyToJoint.translation()
      = Eigen::Vector3d(0.0, 0.2, 0.0);

  auto [slider, sliderBody]
      = skeleton->createJointAndBodyNodePair<dynamics::PrismaticJoint>(
          tip,
          sliderProperties,
          makeBodyProperties(
              "slider_body",
              0.75,
              Eigen::Vector3d(0.0, 0.0, 0.02),
              Eigen::Vector3d(0.06, 0.07, 0.08)));
  setFiniteJointMetadata(slider, 0.1, 0.9, 2.0, 5.0, 0.2, 0.05);

  const common::Uri meshUri
      = common::Uri::createFromPath(config::dataPath("obj/BoxSmall.obj"));
  sliderBody->createShapeNodeWith<dynamics::VisualAspect>(
      std::make_shared<dynamics::MeshShape>(
          Eigen::Vector3d(0.5, 1.0, 2.0), makeTriangleMesh(), meshUri),
      "slider_mesh_visual");

  dynamics::WeldJoint::Properties weldProperties;
  weldProperties.mName = "fixed_tip";
  weldProperties.mT_ParentBodyToJoint.translation()
      = Eigen::Vector3d(0.0, 0.0, 0.25);

  auto [fixedJoint, fixed]
      = skeleton->createJointAndBodyNodePair<dynamics::WeldJoint>(
          sliderBody,
          weldProperties,
          makeBodyProperties(
              "fixed",
              0.25,
              Eigen::Vector3d::Zero(),
              Eigen::Vector3d(0.02, 0.03, 0.04)));
  (void)fixedJoint;
  fixed->createShapeNodeWith<dynamics::CollisionAspect>(
      std::make_shared<dynamics::BoxShape>(Eigen::Vector3d(0.1, 0.1, 0.1)),
      "fixed_box_collision");

  return skeleton;
}

template <class ShapeT, class AspectT>
const ShapeT* getFirstShape(const dynamics::BodyNode* body)
{
  if (!body || body->getNumShapeNodesWith<AspectT>() == 0) {
    return nullptr;
  }

  const auto* shapeNode = body->getShapeNodeWith<AspectT>(0);
  if (!shapeNode) {
    return nullptr;
  }

  return dynamic_cast<const ShapeT*>(shapeNode->getShape().get());
}

void expectBodyInertiaRoundTrips(
    const dynamics::BodyNode& expected, const dynamics::BodyNode& actual)
{
  EXPECT_NEAR(expected.getMass(), actual.getMass(), kTolerance);
  EXPECT_TRUE(
      expected.getLocalCOM().isApprox(actual.getLocalCOM(), kTolerance));
  EXPECT_TRUE(expected.getInertia().getMoment().isApprox(
      actual.getInertia().getMoment(), kTolerance));
}

} // namespace

//==============================================================================
TEST(UrdfWriter, RoundTripsSupportedTreeSubset)
{
  const auto skeleton = makeRoundTripSkeleton();

  const auto writeResult
      = utils::UrdfParser::tryWriteSkeletonToString(*skeleton);
  ASSERT_TRUE(writeResult.isOk()) << writeResult.error().message;
  expectContains(writeResult.value(), "<robot");
  expectContains(writeResult.value(), "urdf_writer_roundtrip");

  utils::UrdfParser parser;
  const auto reparsed = parser.parseSkeletonString(writeResult.value(), "");
  ASSERT_NE(reparsed, nullptr);

  EXPECT_EQ(reparsed->getName(), skeleton->getName());
  EXPECT_EQ(reparsed->getNumBodyNodes(), skeleton->getNumBodyNodes());
  EXPECT_EQ(reparsed->getNumJoints(), skeleton->getNumJoints());

  for (std::size_t i = 0; i < skeleton->getNumBodyNodes(); ++i) {
    const auto* expected = skeleton->getBodyNode(i);
    ASSERT_NE(expected, nullptr);
    const auto* actual = reparsed->getBodyNode(expected->getName());
    ASSERT_NE(actual, nullptr);
    expectBodyInertiaRoundTrips(*expected, *actual);
  }

  const auto* base = reparsed->getBodyNode("base");
  ASSERT_NE(base, nullptr);
  const auto* baseBox
      = getFirstShape<dynamics::BoxShape, dynamics::VisualAspect>(base);
  ASSERT_NE(baseBox, nullptr);
  EXPECT_TRUE(baseBox->getSize().isApprox(Eigen::Vector3d(0.4, 0.5, 0.6)));
  ASSERT_EQ(base->getNumShapeNodesWith<dynamics::VisualAspect>(), 1u);
  const auto* baseVisual
      = base->getShapeNodeWith<dynamics::VisualAspect>(0)->getVisualAspect();
  ASSERT_NE(baseVisual, nullptr);
  EXPECT_TRUE(baseVisual->getRGBA().isApprox(
      Eigen::Vector4d(0.2, 0.4, 0.6, 0.8), kColorTolerance))
      << "actual: " << baseVisual->getRGBA().transpose();

  const auto* hinge = dynamic_cast<const dynamics::RevoluteJoint*>(
      reparsed->getJoint("hinge"));
  ASSERT_NE(hinge, nullptr);
  EXPECT_EQ(hinge->getParentBodyNode()->getName(), "base");
  EXPECT_EQ(hinge->getChildBodyNode()->getName(), "tip");
  EXPECT_TRUE(hinge->getAxis().isApprox(Eigen::Vector3d::UnitZ(), kTolerance));
  EXPECT_TRUE(hinge->getTransformFromParentBodyNode().translation().isApprox(
      Eigen::Vector3d(0.0, 0.0, 0.8), kTolerance));
  EXPECT_NEAR(hinge->getPositionLowerLimit(0), -1.25, kTolerance);
  EXPECT_NEAR(hinge->getPositionUpperLimit(0), 1.5, kTolerance);
  EXPECT_NEAR(hinge->getVelocityUpperLimit(0), 3.0, kTolerance);
  EXPECT_NEAR(hinge->getForceUpperLimit(0), 4.0, kTolerance);
  EXPECT_NEAR(hinge->getDampingCoefficient(0), 0.25, kTolerance);
  EXPECT_NEAR(hinge->getCoulombFriction(0), 0.125, kTolerance);

  const auto* tip = reparsed->getBodyNode("tip");
  ASSERT_NE(tip, nullptr);
  const auto* tipSphere
      = getFirstShape<dynamics::SphereShape, dynamics::VisualAspect>(tip);
  EXPECT_NE(tipSphere, nullptr);
  const auto* tipCylinder
      = getFirstShape<dynamics::CylinderShape, dynamics::CollisionAspect>(tip);
  ASSERT_NE(tipCylinder, nullptr);
  EXPECT_NEAR(tipCylinder->getRadius(), 0.1, kTolerance);
  EXPECT_NEAR(tipCylinder->getHeight(), 0.3, kTolerance);

  const auto* slider = dynamic_cast<const dynamics::PrismaticJoint*>(
      reparsed->getJoint("slider"));
  ASSERT_NE(slider, nullptr);
  EXPECT_EQ(slider->getParentBodyNode()->getName(), "tip");
  EXPECT_EQ(slider->getChildBodyNode()->getName(), "slider_body");
  EXPECT_TRUE(slider->getAxis().isApprox(Eigen::Vector3d::UnitX(), kTolerance));
  EXPECT_NEAR(slider->getPositionLowerLimit(0), 0.1, kTolerance);
  EXPECT_NEAR(slider->getPositionUpperLimit(0), 0.9, kTolerance);
  EXPECT_NEAR(slider->getVelocityUpperLimit(0), 2.0, kTolerance);
  EXPECT_NEAR(slider->getForceUpperLimit(0), 5.0, kTolerance);

  const auto* sliderBody = reparsed->getBodyNode("slider_body");
  ASSERT_NE(sliderBody, nullptr);
  const auto* mesh
      = getFirstShape<dynamics::MeshShape, dynamics::VisualAspect>(sliderBody);
  ASSERT_NE(mesh, nullptr);
  EXPECT_TRUE(mesh->getScale().isApprox(Eigen::Vector3d(0.5, 1.0, 2.0)));

  const auto* fixed = dynamic_cast<const dynamics::WeldJoint*>(
      reparsed->getJoint("fixed_tip"));
  ASSERT_NE(fixed, nullptr);
  EXPECT_EQ(fixed->getParentBodyNode()->getName(), "slider_body");
  EXPECT_EQ(fixed->getChildBodyNode()->getName(), "fixed");
}

//==============================================================================
TEST(UrdfWriter, RoundTripsMimicMetadata)
{
  auto skeleton = dynamics::Skeleton::create("urdf_mimic_writer");

  auto [rootJoint, base]
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>(
          nullptr,
          dynamics::FreeJoint::Properties(),
          makeBodyProperties(
              "base", 1.0, Eigen::Vector3d::Zero(), Eigen::Vector3d::Ones()));
  (void)rootJoint;

  dynamics::RevoluteJoint::Properties referenceProperties;
  referenceProperties.mName = "reference_joint";
  referenceProperties.mAxis = Eigen::Vector3d::UnitZ();
  auto [referenceJoint, referenceBody]
      = skeleton->createJointAndBodyNodePair<dynamics::RevoluteJoint>(
          base,
          referenceProperties,
          makeBodyProperties(
              "reference_link",
              1.0,
              Eigen::Vector3d::Zero(),
              Eigen::Vector3d::Ones()));
  setFiniteJointMetadata(referenceJoint, -1.0, 1.0, 2.0, 3.0, 0.0, 0.0);

  dynamics::PrismaticJoint::Properties followerProperties;
  followerProperties.mName = "follower_joint";
  followerProperties.mAxis = Eigen::Vector3d::UnitX();
  auto [followerJoint, followerBody]
      = skeleton->createJointAndBodyNodePair<dynamics::PrismaticJoint>(
          referenceBody,
          followerProperties,
          makeBodyProperties(
              "follower_link",
              1.0,
              Eigen::Vector3d::Zero(),
              Eigen::Vector3d::Ones()));
  (void)followerBody;
  setFiniteJointMetadata(followerJoint, -0.5, 0.5, 4.0, 5.0, 0.0, 0.0);
  followerJoint->setMimicJoint(referenceJoint, 2.0, -0.25);
  followerJoint->setActuatorType(dynamics::Joint::MIMIC);

  const auto writeResult
      = utils::UrdfParser::tryWriteSkeletonToString(*skeleton);
  ASSERT_TRUE(writeResult.isOk()) << writeResult.error().message;
  expectContains(writeResult.value(), "<mimic");
  expectContains(writeResult.value(), "joint=\"reference_joint\"");

  utils::UrdfParser parser;
  const auto reparsed = parser.parseSkeletonString(writeResult.value(), "");
  ASSERT_NE(reparsed, nullptr);

  const auto* reparsedReference = reparsed->getJoint("reference_joint");
  ASSERT_NE(reparsedReference, nullptr);
  const auto* reparsedFollower = reparsed->getJoint("follower_joint");
  ASSERT_NE(reparsedFollower, nullptr);
  EXPECT_EQ(reparsedFollower->getActuatorType(), dynamics::Joint::MIMIC);
  EXPECT_EQ(reparsedFollower->getMimicJoint(), reparsedReference);
  EXPECT_DOUBLE_EQ(reparsedFollower->getMimicMultiplier(), 2.0);
  EXPECT_DOUBLE_EQ(reparsedFollower->getMimicOffset(), -0.25);
}

//==============================================================================
TEST(UrdfWriter, RoundTripsContinuousJointDynamics)
{
  auto skeleton = dynamics::Skeleton::create("urdf_continuous_writer");

  auto [rootJoint, base]
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>(
          nullptr,
          dynamics::FreeJoint::Properties(),
          makeBodyProperties(
              "base", 1.0, Eigen::Vector3d::Zero(), Eigen::Vector3d::Ones()));
  (void)rootJoint;

  dynamics::RevoluteJoint::Properties hingeProperties;
  hingeProperties.mName = "continuous_hinge";
  hingeProperties.mAxis = Eigen::Vector3d::UnitY();
  auto [hinge, tip]
      = skeleton->createJointAndBodyNodePair<dynamics::RevoluteJoint>(
          base,
          hingeProperties,
          makeBodyProperties(
              "tip", 1.0, Eigen::Vector3d::Zero(), Eigen::Vector3d::Ones()));
  (void)tip;

  const double inf = std::numeric_limits<double>::infinity();
  hinge->setPositionLowerLimit(0, -inf);
  hinge->setPositionUpperLimit(0, inf);
  hinge->setDampingCoefficient(0, 0.35);
  hinge->setCoulombFriction(0, 0.15);

  const auto writeResult
      = utils::UrdfParser::tryWriteSkeletonToString(*skeleton);
  ASSERT_TRUE(writeResult.isOk()) << writeResult.error().message;
  expectContains(writeResult.value(), "type=\"continuous\"");
  expectContains(writeResult.value(), "<dynamics");
  EXPECT_EQ(writeResult.value().find("<limit"), std::string::npos);

  utils::UrdfParser parser;
  const auto reparsed = parser.parseSkeletonString(writeResult.value(), "");
  ASSERT_NE(reparsed, nullptr);

  const auto* reparsedHinge = dynamic_cast<const dynamics::RevoluteJoint*>(
      reparsed->getJoint("continuous_hinge"));
  ASSERT_NE(reparsedHinge, nullptr);
  EXPECT_TRUE(reparsedHinge->isCyclic(0));
  EXPECT_TRUE(
      reparsedHinge->getAxis().isApprox(Eigen::Vector3d::UnitY(), kTolerance));
  EXPECT_NEAR(reparsedHinge->getDampingCoefficient(0), 0.35, kTolerance);
  EXPECT_NEAR(reparsedHinge->getCoulombFriction(0), 0.15, kTolerance);
}

//==============================================================================
TEST(UrdfWriter, IncludeOptionsControlVisualsAndCollisions)
{
  const auto skeleton = makeRoundTripSkeleton();

  utils::UrdfParser::WriteOptions options;
  options.includeVisuals = false;
  const auto noVisuals
      = utils::UrdfParser::tryWriteSkeletonToString(*skeleton, options);
  ASSERT_TRUE(noVisuals.isOk()) << noVisuals.error().message;
  expectContains(noVisuals.value(), "<collision");
  EXPECT_EQ(noVisuals.value().find("<visual"), std::string::npos);

  options.includeVisuals = true;
  options.includeCollisions = false;
  const auto noCollisions
      = utils::UrdfParser::tryWriteSkeletonToString(*skeleton, options);
  ASSERT_TRUE(noCollisions.isOk()) << noCollisions.error().message;
  expectContains(noCollisions.value(), "<visual");
  EXPECT_EQ(noCollisions.value().find("<collision"), std::string::npos);
}

//==============================================================================
TEST(UrdfWriter, MultipleRootTreesReturnError)
{
  auto skeleton = dynamics::Skeleton::create("two_roots");
  skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>(
      nullptr,
      dynamics::FreeJoint::Properties(),
      makeBodyProperties(
          "root_a", 1.0, Eigen::Vector3d::Zero(), Eigen::Vector3d::Ones()));
  skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>(
      nullptr,
      dynamics::FreeJoint::Properties(),
      makeBodyProperties(
          "root_b", 1.0, Eigen::Vector3d::Zero(), Eigen::Vector3d::Ones()));

  const auto result = utils::UrdfParser::tryWriteSkeletonToString(*skeleton);
  ASSERT_TRUE(result.isErr());
  expectContains(result.error().message, "multiple root trees");
}

//==============================================================================
TEST(UrdfWriter, UnsupportedPlanarJointReturnsError)
{
  auto skeleton = dynamics::Skeleton::create("unsupported_planar");
  auto [rootJoint, base]
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>(
          nullptr,
          dynamics::FreeJoint::Properties(),
          makeBodyProperties(
              "base", 1.0, Eigen::Vector3d::Zero(), Eigen::Vector3d::Ones()));
  (void)rootJoint;
  skeleton->createJointAndBodyNodePair<dynamics::PlanarJoint>(
      base,
      dynamics::PlanarJoint::Properties(),
      makeBodyProperties(
          "tip", 1.0, Eigen::Vector3d::Zero(), Eigen::Vector3d::Ones()));

  const auto result = utils::UrdfParser::tryWriteSkeletonToString(*skeleton);
  ASSERT_TRUE(result.isErr());
  expectContains(result.error().message, "PlanarJoint");
  expectContains(result.error().message, "URDF");
}

//==============================================================================
TEST(UrdfWriter, SoftBodyNodeReturnsExplicitUnsupportedError)
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
  dynamics::SoftBodyNode::Properties bodyNodeProperties(
      bodyProperties, softProperties);

  auto [joint, body] = skeleton->createJointAndBodyNodePair<
      dynamics::FreeJoint,
      dynamics::SoftBodyNode>(
      nullptr, dynamics::FreeJoint::Properties(), bodyNodeProperties);
  (void)joint;
  ASSERT_NE(body, nullptr);

  const auto result = utils::UrdfParser::tryWriteSkeletonToString(*skeleton);
  ASSERT_TRUE(result.isErr());
  expectContains(result.error().message, "DART SoftBodyNode");
  expectContains(result.error().message, "soft mesh topology");
}

//==============================================================================
TEST(UrdfWriter, NonIdentityChildJointFrameReturnsError)
{
  auto skeleton = dynamics::Skeleton::create("child_frame");
  auto [rootJoint, base]
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>(
          nullptr,
          dynamics::FreeJoint::Properties(),
          makeBodyProperties(
              "base", 1.0, Eigen::Vector3d::Zero(), Eigen::Vector3d::Ones()));
  (void)rootJoint;

  dynamics::RevoluteJoint::Properties properties;
  properties.mName = "hinge";
  properties.mAxis = Eigen::Vector3d::UnitZ();
  properties.mT_ChildBodyToJoint.translation() = Eigen::Vector3d(0.1, 0.0, 0.0);
  auto [hinge, tip]
      = skeleton->createJointAndBodyNodePair<dynamics::RevoluteJoint>(
          base,
          properties,
          makeBodyProperties(
              "tip", 1.0, Eigen::Vector3d::Zero(), Eigen::Vector3d::Ones()));
  (void)tip;
  setFiniteJointMetadata(hinge, -1.0, 1.0, 2.0, 3.0, 0.0, 0.0);

  const auto result = utils::UrdfParser::tryWriteSkeletonToString(*skeleton);
  ASSERT_TRUE(result.isErr());
  expectContains(result.error().message, "child link frame");
}

//==============================================================================
TEST(UrdfWriter, UnboundedPrismaticLimitsReturnError)
{
  auto skeleton = dynamics::Skeleton::create("unbounded_prismatic");
  auto [rootJoint, base]
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>(
          nullptr,
          dynamics::FreeJoint::Properties(),
          makeBodyProperties(
              "base", 1.0, Eigen::Vector3d::Zero(), Eigen::Vector3d::Ones()));
  (void)rootJoint;

  dynamics::PrismaticJoint::Properties properties;
  properties.mName = "slider";
  properties.mAxis = Eigen::Vector3d::UnitX();
  auto [slider, tip]
      = skeleton->createJointAndBodyNodePair<dynamics::PrismaticJoint>(
          base,
          properties,
          makeBodyProperties(
              "tip", 1.0, Eigen::Vector3d::Zero(), Eigen::Vector3d::Ones()));
  (void)tip;
  slider->setVelocityLowerLimit(0, -1.0);
  slider->setVelocityUpperLimit(0, 1.0);
  slider->setForceLowerLimit(0, -1.0);
  slider->setForceUpperLimit(0, 1.0);

  const auto result = utils::UrdfParser::tryWriteSkeletonToString(*skeleton);
  ASSERT_TRUE(result.isErr());
  expectContains(result.error().message, "unbounded URDF position limits");
}

//==============================================================================
TEST(UrdfWriter, MimicWithoutReferenceReturnsError)
{
  auto skeleton = dynamics::Skeleton::create("mimic_without_reference");
  auto [rootJoint, base]
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>(
          nullptr,
          dynamics::FreeJoint::Properties(),
          makeBodyProperties(
              "base", 1.0, Eigen::Vector3d::Zero(), Eigen::Vector3d::Ones()));
  (void)rootJoint;

  dynamics::RevoluteJoint::Properties properties;
  properties.mName = "follower_joint";
  properties.mAxis = Eigen::Vector3d::UnitZ();
  auto [follower, tip]
      = skeleton->createJointAndBodyNodePair<dynamics::RevoluteJoint>(
          base,
          properties,
          makeBodyProperties(
              "tip", 1.0, Eigen::Vector3d::Zero(), Eigen::Vector3d::Ones()));
  (void)tip;
  setFiniteJointMetadata(follower, -1.0, 1.0, 2.0, 3.0, 0.0, 0.0);
  follower->setActuatorType(dynamics::Joint::MIMIC);

  const auto result = utils::UrdfParser::tryWriteSkeletonToString(*skeleton);
  ASSERT_TRUE(result.isErr());
  expectContains(result.error().message, "no reference joint");
}

//==============================================================================
TEST(UrdfWriter, CouplerMimicReturnsError)
{
  auto skeleton = dynamics::Skeleton::create("coupler_mimic");
  auto [rootJoint, base]
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>(
          nullptr,
          dynamics::FreeJoint::Properties(),
          makeBodyProperties(
              "base", 1.0, Eigen::Vector3d::Zero(), Eigen::Vector3d::Ones()));
  (void)rootJoint;

  dynamics::RevoluteJoint::Properties referenceProperties;
  referenceProperties.mName = "reference_joint";
  referenceProperties.mAxis = Eigen::Vector3d::UnitZ();
  auto [referenceJoint, referenceBody]
      = skeleton->createJointAndBodyNodePair<dynamics::RevoluteJoint>(
          base,
          referenceProperties,
          makeBodyProperties(
              "reference_link",
              1.0,
              Eigen::Vector3d::Zero(),
              Eigen::Vector3d::Ones()));
  setFiniteJointMetadata(referenceJoint, -1.0, 1.0, 2.0, 3.0, 0.0, 0.0);

  dynamics::RevoluteJoint::Properties followerProperties;
  followerProperties.mName = "follower_joint";
  followerProperties.mAxis = Eigen::Vector3d::UnitY();
  auto [followerJoint, followerBody]
      = skeleton->createJointAndBodyNodePair<dynamics::RevoluteJoint>(
          referenceBody,
          followerProperties,
          makeBodyProperties(
              "follower_link",
              1.0,
              Eigen::Vector3d::Zero(),
              Eigen::Vector3d::Ones()));
  (void)followerBody;
  setFiniteJointMetadata(followerJoint, -1.0, 1.0, 2.0, 3.0, 0.0, 0.0);
  followerJoint->setMimicJoint(referenceJoint, 2.0, 0.5);
  followerJoint->setActuatorType(dynamics::Joint::MIMIC);
  followerJoint->setUseCouplerConstraint(true);

  const auto result = utils::UrdfParser::tryWriteSkeletonToString(*skeleton);
  ASSERT_TRUE(result.isErr());
  expectContains(result.error().message, "coupler mimic constraint");
}
