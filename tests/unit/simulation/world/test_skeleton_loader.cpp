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
 *     copyright notice and this list of conditions in the documentation
 *     and/or other materials provided with the distribution.
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

#include <dart/simulation/common/exceptions.hpp>
#include <dart/simulation/io/skeleton_loader.hpp>
#include <dart/simulation/multibody/joint.hpp>
#include <dart/simulation/multibody/link.hpp>
#include <dart/simulation/multibody/multibody.hpp>
#include <dart/simulation/world.hpp>

#include <dart/dynamics/ball_joint.hpp>
#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/capsule_shape.hpp>
#include <dart/dynamics/cone_shape.hpp>
#include <dart/dynamics/cylinder_shape.hpp>
#include <dart/dynamics/euler_joint.hpp>
#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/inertia.hpp>
#include <dart/dynamics/mesh_shape.hpp>
#include <dart/dynamics/planar_joint.hpp>
#include <dart/dynamics/prismatic_joint.hpp>
#include <dart/dynamics/revolute_joint.hpp>
#include <dart/dynamics/screw_joint.hpp>
#include <dart/dynamics/shape_frame.hpp>
#include <dart/dynamics/shape_node.hpp>
#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/sphere_shape.hpp>
#include <dart/dynamics/universal_joint.hpp>

#include <dart/math/tri_mesh.hpp>

#include <dart/common/uri.hpp>

#include <dart/io/read.hpp>

#include <gtest/gtest.h>

#include <initializer_list>
#include <memory>
#include <numbers>
#include <optional>
#include <string>
#include <vector>

#include <cmath>

namespace {

namespace dynamics = dart::dynamics;
namespace sx = dart::simulation;

void setBodyInertia(
    dynamics::BodyNode* body,
    double mass,
    const Eigen::Vector3d& centerOfMass,
    const Eigen::Matrix3d& moment)
{
  dynamics::Inertia inertia;
  inertia.setMass(mass);
  inertia.setLocalCOM(centerOfMass);
  inertia.setMoment(moment);
  body->setInertia(inertia);
}

void configureSingleDofJoint(
    dynamics::Joint* joint, double position, double velocity, double force)
{
  joint->setPosition(0, position);
  joint->setVelocity(0, velocity);
  joint->setForce(0, force);
  joint->setCommand(0, force);
  joint->setPositionLowerLimit(0, -0.75);
  joint->setPositionUpperLimit(0, 0.8);
  joint->setVelocityLowerLimit(0, -1.25);
  joint->setVelocityUpperLimit(0, 1.5);
  joint->setForceLowerLimit(0, -2.0);
  joint->setForceUpperLimit(0, 2.5);
  joint->setSpringStiffness(0, 0.125);
  joint->setRestPosition(0, -0.05);
  joint->setDampingCoefficient(0, 0.25);
  joint->setCoulombFriction(0, 0.0625);
}

Eigen::VectorXd makeVector(std::initializer_list<double> values)
{
  Eigen::VectorXd out(static_cast<Eigen::Index>(values.size()));
  Eigen::Index index = 0;
  for (const double value : values) {
    out[index++] = value;
  }
  return out;
}

void configureJointVectors(
    dynamics::Joint* joint,
    const Eigen::VectorXd& position,
    const Eigen::VectorXd& velocity,
    const Eigen::VectorXd& force)
{
  ASSERT_EQ(joint->getNumDofs(), static_cast<std::size_t>(position.size()));
  ASSERT_EQ(joint->getNumDofs(), static_cast<std::size_t>(velocity.size()));
  ASSERT_EQ(joint->getNumDofs(), static_cast<std::size_t>(force.size()));

  joint->setPositions(position);
  joint->setVelocities(velocity);
  joint->setForces(force);
  joint->setCommands(force);
}

void expectBoxCollisionShape(
    const sx::Link& link, const Eigen::Vector3d& halfExtents)
{
  ASSERT_TRUE(link.hasCollisionShape());
  const std::optional<sx::CollisionShape> shape = link.getCollisionShape();
  ASSERT_TRUE(shape.has_value());
  EXPECT_EQ(shape->type, sx::CollisionShapeType::Box);
  EXPECT_TRUE(shape->halfExtents.isApprox(halfExtents));
}

void expectSphereCollisionShape(const sx::Link& link, double radius)
{
  ASSERT_TRUE(link.hasCollisionShape());
  const std::optional<sx::CollisionShape> shape = link.getCollisionShape();
  ASSERT_TRUE(shape.has_value());
  EXPECT_EQ(shape->type, sx::CollisionShapeType::Sphere);
  EXPECT_DOUBLE_EQ(shape->radius, radius);
}

void expectCapsuleCollisionShape(
    const sx::Link& link, double radius, double halfHeight)
{
  ASSERT_TRUE(link.hasCollisionShape());
  const std::optional<sx::CollisionShape> shape = link.getCollisionShape();
  ASSERT_TRUE(shape.has_value());
  EXPECT_EQ(shape->type, sx::CollisionShapeType::Capsule);
  EXPECT_DOUBLE_EQ(shape->radius, radius);
  EXPECT_TRUE(
      shape->halfExtents.isApprox(Eigen::Vector3d(radius, radius, halfHeight)));
}

void expectCylinderCollisionShape(
    const sx::Link& link, double radius, double halfHeight)
{
  ASSERT_TRUE(link.hasCollisionShape());
  const std::optional<sx::CollisionShape> shape = link.getCollisionShape();
  ASSERT_TRUE(shape.has_value());
  EXPECT_EQ(shape->type, sx::CollisionShapeType::Cylinder);
  EXPECT_DOUBLE_EQ(shape->radius, radius);
  EXPECT_TRUE(
      shape->halfExtents.isApprox(Eigen::Vector3d(radius, radius, halfHeight)));
}

void expectMeshCollisionShape(const sx::Link& link)
{
  ASSERT_TRUE(link.hasCollisionShape());
  const std::optional<sx::CollisionShape> shape = link.getCollisionShape();
  ASSERT_TRUE(shape.has_value());
  EXPECT_EQ(shape->type, sx::CollisionShapeType::Mesh);

  ASSERT_EQ(shape->vertices.size(), 4u);
  EXPECT_TRUE(shape->vertices[0].isApprox(Eigen::Vector3d(0.0, 0.0, 0.0)));
  EXPECT_TRUE(shape->vertices[1].isApprox(Eigen::Vector3d(2.0, 0.0, 0.0)));
  EXPECT_TRUE(shape->vertices[2].isApprox(Eigen::Vector3d(0.0, 3.0, 0.0)));
  EXPECT_TRUE(shape->vertices[3].isApprox(Eigen::Vector3d(0.0, 0.0, 4.0)));

  ASSERT_EQ(shape->triangles.size(), 2u);
  EXPECT_TRUE(shape->triangles[0].isApprox(Eigen::Vector3i(0, 1, 2)));
  EXPECT_TRUE(shape->triangles[1].isApprox(Eigen::Vector3i(0, 2, 3)));
}

dynamics::SkeletonPtr createSupportedTreeSkeleton()
{
  auto skeleton = dynamics::Skeleton::create("loader_tree");

  dynamics::RevoluteJoint::Properties rootProperties;
  rootProperties.mAxis = Eigen::Vector3d::UnitY();
  rootProperties.mT_ChildBodyToJoint.translation()
      = Eigen::Vector3d(-1.25, 0.0, 0.0);
  auto [rootJoint, rootBody]
      = skeleton->createJointAndBodyNodePair<dynamics::RevoluteJoint>(
          nullptr,
          rootProperties,
          dynamics::BodyNode::AspectProperties("root"));
  rootJoint->setName("shoulder");
  configureSingleDofJoint(rootJoint, 0.2, -0.3, 0.4);
  setBodyInertia(
      rootBody,
      2.0,
      Eigen::Vector3d(0.1, -0.2, 0.3),
      Eigen::Vector3d(2.0, 3.0, 4.0).asDiagonal());
  rootBody->createShapeNodeWith<dynamics::CollisionAspect>(
      std::make_shared<dynamics::BoxShape>(Eigen::Vector3d(0.2, 0.4, 0.6)));

  dynamics::PrismaticJoint::Properties sliderProperties;
  sliderProperties.mAxis = Eigen::Vector3d::UnitZ();
  sliderProperties.mT_ParentBodyToJoint.translation()
      = Eigen::Vector3d(0.25, 0.0, 0.0);
  auto [sliderJoint, sliderBody]
      = skeleton->createJointAndBodyNodePair<dynamics::PrismaticJoint>(
          rootBody,
          sliderProperties,
          dynamics::BodyNode::AspectProperties("slider"));
  sliderJoint->setName("lift");
  configureSingleDofJoint(sliderJoint, -0.1, 0.5, 0.0);
  sliderJoint->setActuatorType(dynamics::Joint::VELOCITY);
  sliderJoint->setCommand(0, 0.75);
  setBodyInertia(
      sliderBody,
      1.5,
      Eigen::Vector3d(-0.05, 0.0, 0.2),
      Eigen::Vector3d(1.0, 1.25, 1.5).asDiagonal());
  sliderBody->createShapeNodeWith<dynamics::CollisionAspect>(
      std::make_shared<dynamics::CapsuleShape>(0.12, 0.8));

  return skeleton;
}

dynamics::SkeletonPtr createHigherDofJointSkeleton()
{
  auto skeleton = dynamics::Skeleton::create("joint_family_tree");

  auto [ballJoint, ballBody]
      = skeleton->createJointAndBodyNodePair<dynamics::BallJoint>(
          nullptr,
          dynamics::BallJoint::Properties(),
          dynamics::BodyNode::AspectProperties("ball_link"));
  ballJoint->setName("ball");
  configureJointVectors(
      ballJoint,
      makeVector({0.1, -0.2, 0.3}),
      makeVector({-0.4, 0.5, -0.6}),
      makeVector({0.7, -0.8, 0.9}));

  auto [screwJoint, screwBody]
      = skeleton->createJointAndBodyNodePair<dynamics::ScrewJoint>(
          ballBody,
          dynamics::ScrewJoint::Properties(),
          dynamics::BodyNode::AspectProperties("screw_link"));
  screwJoint->setName("screw");
  screwJoint->setAxis(Eigen::Vector3d::UnitY());
  screwJoint->setPitch(2.0 * std::numbers::pi * 0.25);
  configureJointVectors(
      screwJoint, makeVector({0.15}), makeVector({-0.25}), makeVector({0.35}));

  auto [universalJoint, universalBody]
      = skeleton->createJointAndBodyNodePair<dynamics::UniversalJoint>(
          screwBody,
          dynamics::UniversalJoint::Properties(),
          dynamics::BodyNode::AspectProperties("universal_link"));
  universalJoint->setName("universal");
  universalJoint->setAxis1(Eigen::Vector3d::UnitZ());
  universalJoint->setAxis2(Eigen::Vector3d::UnitY());
  configureJointVectors(
      universalJoint,
      makeVector({0.2, -0.3}),
      makeVector({0.4, -0.5}),
      makeVector({0.6, -0.7}));

  auto [planarJoint, planarBody]
      = skeleton->createJointAndBodyNodePair<dynamics::PlanarJoint>(
          universalBody,
          dynamics::PlanarJoint::Properties(),
          dynamics::BodyNode::AspectProperties("planar_link"));
  planarJoint->setName("planar");
  planarJoint->setArbitraryPlane(
      Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY());
  configureJointVectors(
      planarJoint,
      makeVector({0.1, 0.2, 0.3}),
      makeVector({-0.1, -0.2, -0.3}),
      makeVector({0.4, 0.5, 0.6}));

  auto [freeJoint, freeBody]
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>(
          planarBody,
          dynamics::FreeJoint::Properties(),
          dynamics::BodyNode::AspectProperties("free_link"));
  freeJoint->setName("free");
  configureJointVectors(
      freeJoint,
      makeVector({0.01, 0.02, 0.03, 1.0, 2.0, 3.0}),
      makeVector({0.04, 0.05, 0.06, -1.0, -2.0, -3.0}),
      makeVector({0.07, 0.08, 0.09, 4.0, 5.0, 6.0}));

  return skeleton;
}

dynamics::SkeletonPtr createPendulumSkeleton()
{
  auto skeleton = dynamics::Skeleton::create("loaded_pendulum");

  dynamics::RevoluteJoint::Properties jointProperties;
  jointProperties.mAxis = Eigen::Vector3d::UnitY();
  jointProperties.mT_ChildBodyToJoint.translation()
      = Eigen::Vector3d(-1.0, 0.0, 0.0);
  auto [joint, body]
      = skeleton->createJointAndBodyNodePair<dynamics::RevoluteJoint>(
          nullptr,
          jointProperties,
          dynamics::BodyNode::AspectProperties("bob"));
  joint->setName("hinge");
  joint->setPosition(0, 0.3);
  joint->setVelocity(0, -0.1);

  setBodyInertia(
      body,
      1.25,
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d(1.0, 1.1, 1.2).asDiagonal());
  body->createShapeNodeWith<dynamics::CollisionAspect>(
      std::make_shared<dynamics::SphereShape>(0.35));

  return skeleton;
}

dynamics::SkeletonPtr createParallelUniversalAxisSkeleton()
{
  auto skeleton = dynamics::Skeleton::create("parallel_universal_axis");

  auto [joint, body]
      = skeleton->createJointAndBodyNodePair<dynamics::UniversalJoint>(
          nullptr,
          dynamics::UniversalJoint::Properties(),
          dynamics::BodyNode::AspectProperties("bad_universal_body"));
  (void)body;
  joint->setName("bad_universal");
  joint->setAxis1(Eigen::Vector3d::UnitZ());
  joint->setAxis2(Eigen::Vector3d::UnitZ());

  return skeleton;
}

dynamics::SkeletonPtr createZeroMassSkeleton()
{
  auto skeleton = dynamics::Skeleton::create("zero_mass_loader");

  auto [joint, body]
      = skeleton->createJointAndBodyNodePair<dynamics::RevoluteJoint>(
          nullptr,
          dynamics::RevoluteJoint::Properties(),
          dynamics::BodyNode::AspectProperties("zero_mass_body"));
  (void)joint;
  setBodyInertia(
      body, 0.0, Eigen::Vector3d::Zero(), Eigen::Vector3d::Ones().asDiagonal());

  return skeleton;
}

void seedWorldLinkAndJointCounters(sx::World& world)
{
  sx::Multibody multibody = world.addMultibody("counter_seed");
  sx::Link root = multibody.addLink("seed_root");
  (void)multibody.addLink("", root, sx::JointSpec{.name = ""});
}

dynamics::SkeletonPtr createTargetLinkCounterConflictSkeleton()
{
  auto skeleton = dynamics::Skeleton::create("link_counter_conflict");

  auto [rootJoint, rootBody]
      = skeleton->createJointAndBodyNodePair<dynamics::RevoluteJoint>(
          nullptr,
          dynamics::RevoluteJoint::Properties(),
          dynamics::BodyNode::AspectProperties("root"));
  rootJoint->setName("root_joint");
  const_cast<std::string&>(rootBody->getName()).clear();

  auto [childJoint, childBody]
      = skeleton->createJointAndBodyNodePair<dynamics::RevoluteJoint>(
          rootBody,
          dynamics::RevoluteJoint::Properties(),
          dynamics::BodyNode::AspectProperties("link_002"));
  (void)childBody;
  childJoint->setName("child_joint");

  return skeleton;
}

dynamics::SkeletonPtr createTargetJointCounterConflictSkeleton()
{
  auto skeleton = dynamics::Skeleton::create("joint_counter_conflict");

  auto [rootJoint, rootBody]
      = skeleton->createJointAndBodyNodePair<dynamics::RevoluteJoint>(
          nullptr,
          dynamics::RevoluteJoint::Properties(),
          dynamics::BodyNode::AspectProperties("root"));
  const_cast<std::string&>(rootJoint->getName()).clear();

  auto [childJoint, childBody]
      = skeleton->createJointAndBodyNodePair<dynamics::RevoluteJoint>(
          rootBody,
          dynamics::RevoluteJoint::Properties(),
          dynamics::BodyNode::AspectProperties("child"));
  (void)childBody;
  childJoint->setName("joint_002");

  return skeleton;
}

} // namespace

TEST(SkeletonLoader, TranslatesSupportedTreeProperties)
{
  sx::World world;
  const auto skeleton = createSupportedTreeSkeleton();

  const sx::Multibody multibody = sx::io::addSkeleton(world, *skeleton);

  EXPECT_EQ(multibody.getName(), "loader_tree");
  EXPECT_EQ(multibody.getLinkCount(), 3u);
  EXPECT_EQ(multibody.getJointCount(), 2u);
  EXPECT_EQ(multibody.getDOFCount(), 2u);
  EXPECT_TRUE(multibody.getLink("root_anchor_root").has_value());

  auto root = multibody.getLink("root");
  ASSERT_TRUE(root.has_value());
  EXPECT_DOUBLE_EQ(root->getMass(), 2.0);
  EXPECT_TRUE(
      root->getCenterOfMass().isApprox(Eigen::Vector3d(0.1, -0.2, 0.3)));
  EXPECT_TRUE(root->getInertia().isApprox(
      Eigen::Vector3d(2.0, 3.0, 4.0).asDiagonal().toDenseMatrix()));
  expectBoxCollisionShape(*root, Eigen::Vector3d(0.1, 0.2, 0.3));

  sx::Joint shoulder = root->getParentJoint();
  EXPECT_EQ(shoulder.getName(), "shoulder");
  EXPECT_EQ(shoulder.getType(), sx::JointType::Revolute);
  EXPECT_TRUE(shoulder.getAxis().isApprox(Eigen::Vector3d::UnitY()));
  EXPECT_DOUBLE_EQ(shoulder.getPosition()[0], 0.2);
  EXPECT_DOUBLE_EQ(shoulder.getVelocity()[0], -0.3);
  EXPECT_DOUBLE_EQ(shoulder.getForce()[0], 0.4);
  EXPECT_DOUBLE_EQ(shoulder.getPositionLowerLimits()[0], -0.75);
  EXPECT_DOUBLE_EQ(shoulder.getPositionUpperLimits()[0], 0.8);
  EXPECT_DOUBLE_EQ(shoulder.getVelocityLowerLimits()[0], -1.25);
  EXPECT_DOUBLE_EQ(shoulder.getVelocityUpperLimits()[0], 1.5);
  EXPECT_DOUBLE_EQ(shoulder.getEffortLowerLimits()[0], -2.0);
  EXPECT_DOUBLE_EQ(shoulder.getEffortUpperLimits()[0], 2.5);
  EXPECT_DOUBLE_EQ(shoulder.getSpringStiffness()[0], 0.125);
  EXPECT_DOUBLE_EQ(shoulder.getRestPosition()[0], -0.05);
  EXPECT_DOUBLE_EQ(shoulder.getDampingCoefficient()[0], 0.25);
  EXPECT_DOUBLE_EQ(shoulder.getCoulombFriction()[0], 0.0625);

  auto slider = multibody.getLink("slider");
  ASSERT_TRUE(slider.has_value());
  EXPECT_DOUBLE_EQ(slider->getMass(), 1.5);
  EXPECT_TRUE(
      slider->getCenterOfMass().isApprox(Eigen::Vector3d(-0.05, 0.0, 0.2)));
  EXPECT_TRUE(slider->getInertia().isApprox(
      Eigen::Vector3d(1.0, 1.25, 1.5).asDiagonal().toDenseMatrix()));
  expectCapsuleCollisionShape(*slider, 0.12, 0.4);

  sx::Joint lift = slider->getParentJoint();
  EXPECT_EQ(lift.getName(), "lift");
  EXPECT_EQ(lift.getType(), sx::JointType::Prismatic);
  EXPECT_EQ(lift.getActuatorType(), sx::ActuatorType::Velocity);
  EXPECT_TRUE(lift.getAxis().isApprox(Eigen::Vector3d::UnitZ()));
  EXPECT_DOUBLE_EQ(lift.getPosition()[0], -0.1);
  EXPECT_DOUBLE_EQ(lift.getVelocity()[0], 0.5);
  EXPECT_DOUBLE_EQ(lift.getCommandVelocity()[0], 0.75);

  world.enterSimulationMode();
  const auto* classicRoot = skeleton->getBodyNode("root");
  const auto* classicSlider = skeleton->getBodyNode("slider");
  ASSERT_NE(classicRoot, nullptr);
  ASSERT_NE(classicSlider, nullptr);
  EXPECT_TRUE(root->getWorldTransform().matrix().isApprox(
      classicRoot->getTransform().matrix(), 1e-12));
  EXPECT_TRUE(slider->getWorldTransform().matrix().isApprox(
      classicSlider->getTransform().matrix(), 1e-12));
}

TEST(SkeletonLoader, TranslatesForceActuatorCommandsAsEffort)
{
  auto skeleton = dynamics::Skeleton::create("force_command");
  auto [joint, body]
      = skeleton->createJointAndBodyNodePair<dynamics::RevoluteJoint>(
          nullptr,
          dynamics::RevoluteJoint::Properties(),
          dynamics::BodyNode::AspectProperties("force_link"));
  (void)body;
  joint->setName("force_hinge");
  joint->setForce(0, -0.25);
  joint->setCommand(0, 1.25);

  sx::World world;
  const sx::Multibody multibody = sx::io::addSkeleton(world, *skeleton);

  auto loadedJoint = multibody.getJoint("force_hinge");
  ASSERT_TRUE(loadedJoint.has_value());
  EXPECT_EQ(loadedJoint->getActuatorType(), sx::ActuatorType::Force);
  EXPECT_DOUBLE_EQ(loadedJoint->getForce()[0], 1.25);
}

TEST(SkeletonLoader, TranslatesServoActuatorCommandsAsVelocity)
{
  auto skeleton = dynamics::Skeleton::create("servo_command");
  auto [joint, body]
      = skeleton->createJointAndBodyNodePair<dynamics::RevoluteJoint>(
          nullptr,
          dynamics::RevoluteJoint::Properties(),
          dynamics::BodyNode::AspectProperties("servo_link"));
  (void)body;
  joint->setName("servo_hinge");
  joint->setActuatorType(dynamics::Joint::SERVO);
  joint->setVelocity(0, -0.25);
  joint->setCommand(0, 1.25);

  sx::World world;
  const sx::Multibody multibody = sx::io::addSkeleton(world, *skeleton);

  auto loadedJoint = multibody.getJoint("servo_hinge");
  ASSERT_TRUE(loadedJoint.has_value());
  EXPECT_EQ(loadedJoint->getActuatorType(), sx::ActuatorType::Servo);
  ASSERT_EQ(loadedJoint->getCommandVelocity().size(), 1);
  EXPECT_DOUBLE_EQ(loadedJoint->getVelocity()[0], -0.25);
  EXPECT_DOUBLE_EQ(loadedJoint->getCommandVelocity()[0], 1.25);
}

TEST(SkeletonLoader, TranslatesAccelerationActuatorCommandsAsAcceleration)
{
  auto skeleton = dynamics::Skeleton::create("acceleration_command");
  auto [joint, body]
      = skeleton->createJointAndBodyNodePair<dynamics::RevoluteJoint>(
          nullptr,
          dynamics::RevoluteJoint::Properties(),
          dynamics::BodyNode::AspectProperties("acceleration_link"));
  (void)body;
  joint->setName("acceleration_hinge");
  joint->setActuatorType(dynamics::Joint::ACCELERATION);
  joint->setVelocity(0, -0.25);
  joint->setCommand(0, 1.25);

  sx::World world;
  const sx::Multibody multibody = sx::io::addSkeleton(world, *skeleton);

  auto loadedJoint = multibody.getJoint("acceleration_hinge");
  ASSERT_TRUE(loadedJoint.has_value());
  EXPECT_EQ(loadedJoint->getActuatorType(), sx::ActuatorType::Acceleration);
  ASSERT_EQ(loadedJoint->getCommandAcceleration().size(), 1);
  EXPECT_DOUBLE_EQ(loadedJoint->getVelocity()[0], -0.25);
  EXPECT_DOUBLE_EQ(loadedJoint->getCommandAcceleration()[0], 1.25);
}

TEST(SkeletonLoader, RejectsMimicActuatorBeforeCreatingMultibody)
{
  auto skeleton = dynamics::Skeleton::create("mimic_command");
  auto [joint, body]
      = skeleton->createJointAndBodyNodePair<dynamics::RevoluteJoint>(
          nullptr,
          dynamics::RevoluteJoint::Properties(),
          dynamics::BodyNode::AspectProperties("mimic_link"));
  (void)body;
  joint->setName("mimic_hinge");
  joint->setActuatorType(dynamics::Joint::MIMIC);

  sx::World world;
  EXPECT_THROW(
      sx::io::addSkeleton(world, *skeleton), sx::InvalidArgumentException);
  EXPECT_EQ(world.getMultibodyCount(), 0u);
}

TEST(SkeletonLoader, LoadedPendulumAdvancesOneStep)
{
  const auto skeleton = createPendulumSkeleton();
  constexpr double kTimeStep = 0.001;
  const Eigen::Vector3d gravity(0.0, 0.0, -9.81);

  sx::World experimentalWorld;
  experimentalWorld.setGravity(gravity);
  experimentalWorld.setTimeStep(kTimeStep);
  const sx::Multibody multibody
      = sx::io::addSkeleton(experimentalWorld, *skeleton);

  experimentalWorld.enterSimulationMode();
  experimentalWorld.step();

  auto hinge = multibody.getJoint("hinge");
  ASSERT_TRUE(hinge.has_value());
  EXPECT_TRUE(std::isfinite(hinge->getPosition()[0]));
  EXPECT_TRUE(std::isfinite(hinge->getVelocity()[0]));
  EXPECT_TRUE(std::isfinite(hinge->getAcceleration()[0]));

  auto bob = multibody.getLink("bob");
  ASSERT_TRUE(bob.has_value());
  EXPECT_TRUE(bob->getWorldTransform().translation().allFinite());
  expectSphereCollisionShape(*bob, 0.35);
  EXPECT_NEAR(experimentalWorld.getTime(), kTimeStep, 1e-12);
  EXPECT_EQ(experimentalWorld.getFrame(), 1u);
}

TEST(SkeletonLoader, LoadsSkeletonFromUri)
{
#if DART_HAS_SDFORMAT
  sx::World world;
  sx::io::SkeletonLoadOptions options;
  options.rootAnchorPrefix = "uri_anchor_";

  const sx::Multibody multibody = sx::io::addSkeleton(
      world,
      dart::common::Uri("dart://sample/sdf/test/single_pendulum.sdf"),
      options);

  EXPECT_EQ(multibody.getName(), "single_pendulum");
  EXPECT_EQ(multibody.getLinkCount(), 2u);
  EXPECT_EQ(multibody.getJointCount(), 1u);
  EXPECT_EQ(multibody.getDOFCount(), 1u);
  EXPECT_TRUE(multibody.getLink("uri_anchor_link 1").has_value());

  const auto link = multibody.getLink("link 1");
  ASSERT_TRUE(link.has_value());
  const sx::Joint joint = link->getParentJoint();
  EXPECT_EQ(joint.getName(), "joint 1");
  EXPECT_EQ(joint.getType(), sx::JointType::Revolute);
  EXPECT_TRUE(joint.getAxis().isApprox(Eigen::Vector3d::UnitZ()));
  expectBoxCollisionShape(*link, Eigen::Vector3d(0.05, 0.1, 0.15));
#else
  GTEST_SKIP() << "SDF support is disabled";
#endif
}

TEST(SkeletonLoader, TranslatesMultipleSkeletonsIntoWorld)
{
  sx::World world;
  const std::vector<sx::Multibody> multibodies = {
      sx::io::addSkeleton(world, *createSupportedTreeSkeleton()),
      sx::io::addSkeleton(world, *createPendulumSkeleton()),
  };

  ASSERT_EQ(multibodies.size(), 2u);
  EXPECT_EQ(world.getMultibodyCount(), 2u);
  EXPECT_EQ(multibodies[0].getName(), "loader_tree");
  EXPECT_EQ(multibodies[1].getName(), "loaded_pendulum");
  EXPECT_TRUE(world.getMultibody("loader_tree").has_value());
  EXPECT_TRUE(world.getMultibody("loaded_pendulum").has_value());

  auto bob = multibodies[1].getLink("bob");
  ASSERT_TRUE(bob.has_value());
  expectSphereCollisionShape(*bob, 0.35);
}

TEST(SkeletonLoader, RejectsNameConflictBeforeCreatingMultibody)
{
  sx::World world;
  world.addMultibody("loaded_pendulum");

  EXPECT_THROW(
      sx::io::addSkeleton(world, *createPendulumSkeleton()),
      sx::InvalidArgumentException);
  EXPECT_EQ(world.getMultibodyCount(), 1u);
}

TEST(SkeletonLoader, GeneratesNextAvailableMultibodyName)
{
  auto unnamedSkeleton = createSupportedTreeSkeleton();
  const_cast<std::string&>(unnamedSkeleton->getName()).clear();

  sx::World world;
  world.addMultibody("multibody_001");

  const sx::Multibody multibody = sx::io::addSkeleton(world, *unnamedSkeleton);

  EXPECT_EQ(multibody.getName(), "multibody_002");
  EXPECT_EQ(world.getMultibodyCount(), 2u);
}

TEST(SkeletonLoader, RejectsTargetCounterConflictsBeforeCreatingMultibody)
{
  {
    sx::World world;
    seedWorldLinkAndJointCounters(world);

    EXPECT_THROW(
        sx::io::addSkeleton(world, *createTargetLinkCounterConflictSkeleton()),
        sx::InvalidArgumentException);
    EXPECT_EQ(world.getMultibodyCount(), 1u);
    EXPECT_FALSE(world.getMultibody("link_counter_conflict").has_value());
  }

  {
    sx::World world;
    seedWorldLinkAndJointCounters(world);

    EXPECT_THROW(
        sx::io::addSkeleton(world, *createTargetJointCounterConflictSkeleton()),
        sx::InvalidArgumentException);
    EXPECT_EQ(world.getMultibodyCount(), 1u);
    EXPECT_FALSE(world.getMultibody("joint_counter_conflict").has_value());
  }
}

TEST(SkeletonLoader, RejectsJointAxisConflictBeforeCreatingMultibody)
{
  sx::World world;
  EXPECT_THROW(
      sx::io::addSkeleton(world, *createParallelUniversalAxisSkeleton()),
      sx::InvalidArgumentException);
  EXPECT_EQ(world.getMultibodyCount(), 0u);
}

TEST(SkeletonLoader, RejectsInvalidBodyInertiaBeforeCreatingMultibody)
{
  sx::World world;
  EXPECT_THROW(
      sx::io::addSkeleton(world, *createZeroMassSkeleton()),
      sx::InvalidArgumentException);
  EXPECT_EQ(world.getMultibodyCount(), 0u);
}

TEST(SkeletonLoader, LoadsUriWithReadOptions)
{
#if DART_HAS_SDFORMAT
  const dart::common::Uri uri("dart://sample/sdf/test/single_pendulum.sdf");

  dart::io::ReadOptions readOptions;
  readOptions.format = dart::io::ModelFormat::Sdf;

  sx::io::SkeletonLoadOptions loadOptions;
  loadOptions.rootAnchorPrefix = "read_options_anchor_";

  sx::World skeletonWorld;
  const sx::Multibody multibody
      = sx::io::addSkeleton(skeletonWorld, uri, readOptions, loadOptions);
  EXPECT_EQ(multibody.getName(), "single_pendulum");
  EXPECT_TRUE(multibody.getLink("read_options_anchor_link 1").has_value());

  dart::io::ReadOptions wrongFormat;
  wrongFormat.format = dart::io::ModelFormat::Urdf;

  sx::World rejectedWorld;
  EXPECT_THROW(
      sx::io::addSkeleton(rejectedWorld, uri, wrongFormat),
      sx::InvalidArgumentException);
#else
  GTEST_SKIP() << "SDF support is disabled";
#endif
}

TEST(SkeletonLoader, TranslatesMeshCollisionShape)
{
  auto skeleton = dynamics::Skeleton::create("mesh_collision_shape");
  auto [joint, body]
      = skeleton->createJointAndBodyNodePair<dynamics::RevoluteJoint>(
          nullptr,
          dynamics::RevoluteJoint::Properties(),
          dynamics::BodyNode::AspectProperties("mesh_body"));
  (void)joint;

  auto mesh = std::make_shared<dart::math::TriMesh<double>>();
  mesh->reserveVertices(4);
  mesh->addVertex(0.0, 0.0, 0.0);
  mesh->addVertex(1.0, 0.0, 0.0);
  mesh->addVertex(0.0, 1.0, 0.0);
  mesh->addVertex(0.0, 0.0, 1.0);
  mesh->reserveTriangles(2);
  mesh->addTriangle(0, 1, 2);
  mesh->addTriangle(0, 2, 3);

  body->createShapeNodeWith<dynamics::CollisionAspect>(
      std::make_shared<dynamics::MeshShape>(
          Eigen::Vector3d(2.0, 3.0, 4.0), mesh));

  sx::World world;
  const sx::Multibody multibody = sx::io::addSkeleton(world, *skeleton);

  auto loaded = multibody.getLink("mesh_body");
  ASSERT_TRUE(loaded.has_value());
  expectMeshCollisionShape(*loaded);
}

TEST(SkeletonLoader, TranslatesCylinderCollisionShape)
{
  auto skeleton = dynamics::Skeleton::create("cylinder_collision_shape");
  auto [joint, body]
      = skeleton->createJointAndBodyNodePair<dynamics::RevoluteJoint>(
          nullptr,
          dynamics::RevoluteJoint::Properties(),
          dynamics::BodyNode::AspectProperties("cylinder_body"));
  (void)joint;

  body->createShapeNodeWith<dynamics::CollisionAspect>(
      std::make_shared<dynamics::CylinderShape>(0.25, 1.5));

  sx::World world;
  const sx::Multibody multibody = sx::io::addSkeleton(world, *skeleton);

  auto loaded = multibody.getLink("cylinder_body");
  ASSERT_TRUE(loaded.has_value());
  expectCylinderCollisionShape(*loaded, 0.25, 0.75);
}

TEST(SkeletonLoader, RejectsUnreadableUri)
{
  sx::World world;
  EXPECT_THROW(
      sx::io::addSkeleton(
          world,
          dart::common::Uri("dart://sample/sdf/test/does_not_exist.sdf")),
      sx::InvalidArgumentException);
}

TEST(SkeletonLoader, RejectsUnsupportedEulerJoint)
{
  auto skeleton = dynamics::Skeleton::create("euler_body");
  [[maybe_unused]] auto pair
      = skeleton->createJointAndBodyNodePair<dynamics::EulerJoint>(
          nullptr,
          dynamics::EulerJoint::Properties(),
          dynamics::BodyNode::AspectProperties("body"));

  sx::World world;
  EXPECT_THROW(
      sx::io::addSkeleton(world, *skeleton), sx::InvalidArgumentException);
  EXPECT_EQ(world.getMultibodyCount(), 0u);
}

TEST(SkeletonLoader, RejectsMultipleCollidableCollisionShapes)
{
  auto skeleton = dynamics::Skeleton::create("multiple_collision_shapes");
  auto [joint, body]
      = skeleton->createJointAndBodyNodePair<dynamics::RevoluteJoint>(
          nullptr,
          dynamics::RevoluteJoint::Properties(),
          dynamics::BodyNode::AspectProperties("body"));
  (void)joint;

  body->createShapeNodeWith<dynamics::CollisionAspect>(
      std::make_shared<dynamics::BoxShape>(Eigen::Vector3d::Ones()));
  body->createShapeNodeWith<dynamics::CollisionAspect>(
      std::make_shared<dynamics::SphereShape>(0.25));

  sx::World world;
  EXPECT_THROW(
      sx::io::addSkeleton(world, *skeleton), sx::InvalidArgumentException);
  EXPECT_EQ(world.getMultibodyCount(), 0u);
}

TEST(SkeletonLoader, RejectsOffsetCollisionShape)
{
  auto skeleton = dynamics::Skeleton::create("offset_collision_shape");
  auto [joint, body]
      = skeleton->createJointAndBodyNodePair<dynamics::RevoluteJoint>(
          nullptr,
          dynamics::RevoluteJoint::Properties(),
          dynamics::BodyNode::AspectProperties("body"));
  (void)joint;

  auto* shapeNode = body->createShapeNodeWith<dynamics::CollisionAspect>(
      std::make_shared<dynamics::BoxShape>(Eigen::Vector3d::Ones()));
  shapeNode->setRelativeTranslation(Eigen::Vector3d(0.1, 0.0, 0.0));

  sx::World world;
  EXPECT_THROW(
      sx::io::addSkeleton(world, *skeleton), sx::InvalidArgumentException);
  EXPECT_EQ(world.getMultibodyCount(), 0u);
}

TEST(SkeletonLoader, RejectsUnsupportedCollisionShape)
{
  auto skeleton = dynamics::Skeleton::create("unsupported_collision_shape");
  auto [joint, body]
      = skeleton->createJointAndBodyNodePair<dynamics::RevoluteJoint>(
          nullptr,
          dynamics::RevoluteJoint::Properties(),
          dynamics::BodyNode::AspectProperties("body"));
  (void)joint;

  body->createShapeNodeWith<dynamics::CollisionAspect>(
      std::make_shared<dynamics::ConeShape>(0.25, 0.5));

  sx::World world;
  EXPECT_THROW(
      sx::io::addSkeleton(world, *skeleton), sx::InvalidArgumentException);
  EXPECT_EQ(world.getMultibodyCount(), 0u);
}

TEST(SkeletonLoader, RejectsNonExpMapCoordinateCharts)
{
  {
    auto skeleton = dynamics::Skeleton::create("ball_euler_body");
    auto [joint, body]
        = skeleton->createJointAndBodyNodePair<dynamics::BallJoint>(
            nullptr,
            dynamics::BallJoint::Properties(),
            dynamics::BodyNode::AspectProperties("body"));
    (void)body;
    joint->setCoordinateChart(dynamics::BallJoint::CoordinateChart::EULER_XYZ);

    sx::World world;
    EXPECT_THROW(
        sx::io::addSkeleton(world, *skeleton), sx::InvalidArgumentException);
    EXPECT_EQ(world.getMultibodyCount(), 0u);
  }

  {
    auto skeleton = dynamics::Skeleton::create("free_euler_body");
    auto [joint, body]
        = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>(
            nullptr,
            dynamics::FreeJoint::Properties(),
            dynamics::BodyNode::AspectProperties("body"));
    (void)body;
    joint->setCoordinateChart(dynamics::FreeJoint::CoordinateChart::EULER_ZYX);

    sx::World world;
    EXPECT_THROW(
        sx::io::addSkeleton(world, *skeleton), sx::InvalidArgumentException);
    EXPECT_EQ(world.getMultibodyCount(), 0u);
  }
}

TEST(SkeletonLoader, TranslatesHigherDofJointFamilies)
{
  sx::World world;
  const auto skeleton = createHigherDofJointSkeleton();

  const sx::Multibody multibody = sx::io::addSkeleton(world, *skeleton);

  EXPECT_EQ(multibody.getName(), "joint_family_tree");
  EXPECT_EQ(multibody.getLinkCount(), 6u);
  EXPECT_EQ(multibody.getJointCount(), 5u);
  EXPECT_EQ(multibody.getDOFCount(), 15u);

  auto ballLink = multibody.getLink("ball_link");
  ASSERT_TRUE(ballLink.has_value());
  sx::Joint ball = ballLink->getParentJoint();
  EXPECT_EQ(ball.getName(), "ball");
  EXPECT_EQ(ball.getType(), sx::JointType::Spherical);
  EXPECT_TRUE(ball.getPosition().isApprox(makeVector({0.1, -0.2, 0.3})));
  EXPECT_TRUE(ball.getVelocity().isApprox(makeVector({-0.4, 0.5, -0.6})));
  EXPECT_TRUE(ball.getForce().isApprox(makeVector({0.7, -0.8, 0.9})));

  auto screwLink = multibody.getLink("screw_link");
  ASSERT_TRUE(screwLink.has_value());
  sx::Joint screw = screwLink->getParentJoint();
  EXPECT_EQ(screw.getName(), "screw");
  EXPECT_EQ(screw.getType(), sx::JointType::Screw);
  EXPECT_TRUE(screw.getAxis().isApprox(Eigen::Vector3d::UnitY()));
  EXPECT_DOUBLE_EQ(screw.getPitch(), 0.25);
  EXPECT_TRUE(screw.getPosition().isApprox(makeVector({0.15})));

  auto universalLink = multibody.getLink("universal_link");
  ASSERT_TRUE(universalLink.has_value());
  sx::Joint universal = universalLink->getParentJoint();
  EXPECT_EQ(universal.getName(), "universal");
  EXPECT_EQ(universal.getType(), sx::JointType::Universal);
  EXPECT_TRUE(universal.getAxis().isApprox(Eigen::Vector3d::UnitZ()));
  EXPECT_TRUE(universal.getAxis2().isApprox(Eigen::Vector3d::UnitY()));
  EXPECT_TRUE(universal.getPosition().isApprox(makeVector({0.2, -0.3})));

  auto planarLink = multibody.getLink("planar_link");
  ASSERT_TRUE(planarLink.has_value());
  sx::Joint planar = planarLink->getParentJoint();
  EXPECT_EQ(planar.getName(), "planar");
  EXPECT_EQ(planar.getType(), sx::JointType::Planar);
  EXPECT_TRUE(planar.getAxis().isApprox(Eigen::Vector3d::UnitZ()));
  EXPECT_TRUE(planar.getAxis2().isApprox(Eigen::Vector3d::UnitX()));
  EXPECT_TRUE(planar.getPosition().isApprox(makeVector({0.1, 0.2, 0.3})));

  auto freeLink = multibody.getLink("free_link");
  ASSERT_TRUE(freeLink.has_value());
  sx::Joint free = freeLink->getParentJoint();
  EXPECT_EQ(free.getName(), "free");
  EXPECT_EQ(free.getType(), sx::JointType::Floating);
  EXPECT_TRUE(free.getPosition().isApprox(
      makeVector({1.0, 2.0, 3.0, 0.01, 0.02, 0.03})));
  EXPECT_TRUE(free.getVelocity().isApprox(
      makeVector({-1.0, -2.0, -3.0, 0.04, 0.05, 0.06})));
  EXPECT_TRUE(
      free.getForce().isApprox(makeVector({4.0, 5.0, 6.0, 0.07, 0.08, 0.09})));
}
