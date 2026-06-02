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

#include <dart/simulation/experimental/body/deformable_body.hpp>
#include <dart/simulation/experimental/body/rigid_body.hpp>
#include <dart/simulation/experimental/common/constants.hpp>
#include <dart/simulation/experimental/comps/frame_types.hpp>
#include <dart/simulation/experimental/comps/joint.hpp>
#include <dart/simulation/experimental/comps/link.hpp>
#include <dart/simulation/experimental/comps/multibody.hpp>
#include <dart/simulation/experimental/comps/name.hpp>
#include <dart/simulation/experimental/constraint/loop_closure_spec.hpp>
#include <dart/simulation/experimental/frame/fixed_frame.hpp>
#include <dart/simulation/experimental/frame/frame.hpp>
#include <dart/simulation/experimental/frame/free_frame.hpp>
#include <dart/simulation/experimental/io/binary_io.hpp>
#include <dart/simulation/experimental/io/serializer.hpp>
#include <dart/simulation/experimental/multibody/joint.hpp>
#include <dart/simulation/experimental/multibody/link.hpp>
#include <dart/simulation/experimental/multibody/multibody.hpp>
#include <dart/simulation/experimental/world.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <iterator>
#include <ranges>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

#include <cstdint>
#include <cstring>

//==============================================================================
// Serialization Tests - Comprehensive Coverage
//==============================================================================

namespace {

void writeLegacyJointV1(
    std::ostream& output,
    const dart::simulation::experimental::comps::Joint& joint,
    const dart::simulation::experimental::io::EntityMap& entityMap)
{
  namespace io = dart::simulation::experimental::io;

  io::writePOD(output, joint.type);
  io::writeString(output, joint.name);
  io::writeVectorXd(output, joint.position);
  io::writeVectorXd(output, joint.velocity);
  io::writeVectorXd(output, joint.acceleration);
  io::writeVectorXd(output, joint.torque);
  io::writeVectorXd(output, joint.limits.lower);
  io::writeVectorXd(output, joint.limits.upper);
  io::writeVectorXd(output, joint.limits.velocityUpper);
  io::writeVectorXd(output, joint.limits.effortUpper);
  io::writeVector3d(output, joint.axis);
  io::writeVector3d(output, joint.axis2);
  io::writePOD(output, joint.pitch);

  const auto mappedParent
      = joint.parentLink != entt::null
            ? static_cast<std::uint32_t>(entityMap.at(joint.parentLink))
            : static_cast<std::uint32_t>(entt::null);
  const auto mappedChild
      = joint.childLink != entt::null
            ? static_cast<std::uint32_t>(entityMap.at(joint.childLink))
            : static_cast<std::uint32_t>(entt::null);
  io::writePOD(output, mappedParent);
  io::writePOD(output, mappedChild);
}

void writeMassPropertiesV8(
    std::ostream& output,
    const dart::simulation::experimental::comps::MassProperties& mass)
{
  namespace io = dart::simulation::experimental::io;

  io::writePOD(output, mass.mass);
  for (int i = 0; i < 3; ++i) {
    for (int j = i; j < 3; ++j) {
      io::writePOD(output, mass.inertia(i, j));
    }
  }
  io::writeVector3d(output, mass.localCenterOfMass);
}

void writeLegacyLinkV8(
    std::ostream& output,
    const dart::simulation::experimental::comps::Link& link,
    const dart::simulation::experimental::io::EntityMap& entityMap,
    bool includeExternalForce)
{
  namespace io = dart::simulation::experimental::io;

  io::writeString(output, link.name);
  writeMassPropertiesV8(output, link.mass);
  io::writeIsometry3d(output, link.transformFromParentJoint);

  const auto mappedParent
      = link.parentJoint != entt::null
            ? static_cast<std::uint32_t>(entityMap.at(link.parentJoint))
            : static_cast<std::uint32_t>(entt::null);
  io::writePOD(output, mappedParent);

  io::writePOD(output, link.childJoints.size());
  for (const auto childJoint : link.childJoints) {
    const auto mappedChild
        = childJoint != entt::null
              ? static_cast<std::uint32_t>(entityMap.at(childJoint))
              : static_cast<std::uint32_t>(entt::null);
    io::writePOD(output, mappedChild);
  }

  io::writeIsometry3d(output, link.worldTransform);
  if (includeExternalForce) {
    for (Eigen::Index i = 0; i < link.externalForce.size(); ++i) {
      io::writePOD(output, link.externalForce[i]);
    }
  }
}

void saveLegacyWorldWithCurrentEntities(
    std::ostream& output,
    const dart::simulation::experimental::World& world,
    std::uint32_t legacyVersion)
{
  namespace comps = dart::simulation::experimental::comps;
  namespace io = dart::simulation::experimental::io;

  constexpr std::uint32_t magicNumber = 0x44525437;
  io::writePOD(output, magicNumber);
  io::writePOD(output, legacyVersion);

  const auto& registry = world.getRegistry();
  std::vector<entt::entity> entities;
  auto nameView = registry.view<comps::Name>();
  for (auto entity : nameView) {
    entities.push_back(entity);
  }
  std::ranges::sort(entities, [](entt::entity lhs, entt::entity rhs) {
    return static_cast<std::uint32_t>(lhs) < static_cast<std::uint32_t>(rhs);
  });

  io::writePOD(output, entities.size());

  io::EntityMap entityMap;
  std::uint32_t sequential = 0;
  for (auto entity : entities) {
    entityMap[entity] = static_cast<entt::entity>(sequential++);
  }

  const auto& serializers = io::SerializerRegistry::instance().getSerializers();
  for (auto entity : entities) {
    io::writePOD(output, static_cast<std::uint32_t>(entityMap.at(entity)));

    std::vector<std::string> componentTypes;
    for (const auto& [typeName, serializer] : serializers) {
      if (serializer->hasComponent(entity, registry)) {
        componentTypes.push_back(typeName);
      }
    }
    std::ranges::sort(componentTypes);

    io::writePOD(output, componentTypes.size());
    for (const auto& typeName : componentTypes) {
      io::writeString(output, typeName);
      if (legacyVersion == 1u && typeName == comps::Joint::getTypeName()) {
        writeLegacyJointV1(
            output, registry.get<comps::Joint>(entity), entityMap);
      } else if (typeName == comps::Link::getTypeName()) {
        writeLegacyLinkV8(
            output,
            registry.get<comps::Link>(entity),
            entityMap,
            legacyVersion >= 5u);
      } else {
        serializers.at(typeName)->save(output, entity, registry, entityMap);
      }
    }
  }
}

void saveLegacyV1WorldWithCurrentEntities(
    std::ostream& output, const dart::simulation::experimental::World& world)
{
  saveLegacyWorldWithCurrentEntities(output, world, /*legacyVersion=*/1u);
}

void saveLegacyV8WorldWithCurrentEntities(
    std::ostream& output, const dart::simulation::experimental::World& world)
{
  saveLegacyWorldWithCurrentEntities(output, world, /*legacyVersion=*/8u);
}

} // namespace

// Test save/load empty world
TEST(Serialization, EmptyWorld)
{
  dart::simulation::experimental::World world1;

  // Save to stream
  std::stringstream ss;
  world1.saveBinary(ss);
  // Load into new world
  dart::simulation::experimental::World world2;
  world2.loadBinary(ss);

  // Verify empty
  EXPECT_EQ(world2.getMultibodyCount(), 0);
  EXPECT_EQ(world2.getRigidBodyCount(), 0);
  EXPECT_FALSE(world2.isSimulationMode());
}

// Test deformable custom serializers are restored after registry reset.
TEST(Serialization, DeformableSerializersAreRegisteredAfterRegistryClear)
{
  namespace sx = dart::simulation::experimental;

  sx::io::SerializerRegistry::instance().clear();

  sx::World world1;
  sx::DeformableBodyOptions options;
  options.positions
      = {Eigen::Vector3d::Zero(),
         Eigen::Vector3d::UnitX(),
         Eigen::Vector3d::UnitY(),
         Eigen::Vector3d::UnitZ()};
  options.tetrahedra = {sx::DeformableTetrahedron{0, 1, 2, 3}};
  options.material.density = 12.0;
  [[maybe_unused]] auto body
      = world1.addDeformableBody("serializer_mesh", options);

  std::stringstream stream;
  ASSERT_NO_THROW(world1.saveBinary(stream));

  sx::World world2;
  ASSERT_NO_THROW(world2.loadBinary(stream));
  auto restored = world2.getDeformableBody("serializer_mesh");
  ASSERT_TRUE(restored.has_value());
  EXPECT_EQ(restored->getTetrahedronCount(), 1u);
  EXPECT_EQ(restored->getSurfaceTriangleCount(), 4u);
  EXPECT_DOUBLE_EQ(restored->getMass(0), 0.5);
}

// Test save/load world with single multibody (no links)
TEST(Serialization, SingleMultibodyNoLinks)
{
  dart::simulation::experimental::World world1;
  [[maybe_unused]] auto mb1 = world1.addMultibody("robot1");

  std::stringstream ss;
  world1.saveBinary(ss);

  dart::simulation::experimental::World world2;
  world2.loadBinary(ss);

  EXPECT_EQ(world2.getMultibodyCount(), 1);
  auto mb_restored = world2.getMultibody("robot1");
  ASSERT_TRUE(mb_restored.has_value());
  EXPECT_EQ(mb_restored->getName(), "robot1");
  EXPECT_EQ(mb_restored->getLinkCount(), 0);
  EXPECT_EQ(mb_restored->getJointCount(), 0);
}

// Test save/load world with single link (root, no parent joint)
TEST(Serialization, SingleRootLink)
{
  dart::simulation::experimental::World world1;
  auto mb = world1.addMultibody("robot");
  [[maybe_unused]] auto base = mb.addLink("base");

  std::stringstream ss;
  world1.saveBinary(ss);

  dart::simulation::experimental::World world2;
  world2.loadBinary(ss);

  auto mb_restored = world2.getMultibody("robot");
  ASSERT_TRUE(mb_restored.has_value());
  EXPECT_EQ(mb_restored->getName(), "robot"); // Verify name preserved
  EXPECT_EQ(mb_restored->getLinkCount(), 1);
  EXPECT_EQ(mb_restored->getJointCount(), 0);
}

// Test save/load simple 2-link chain with full property verification
TEST(Serialization, TwoLinkChain)
{
  dart::simulation::experimental::World world1;
  auto mb = world1.addMultibody("robot");
  auto base = mb.addLink("base");
  [[maybe_unused]] auto link1 = mb.addLink(
      "link1",
      {.parentLink = base,
       .jointName = "joint1",
       .jointType = dart::simulation::experimental::JointType::Revolute,
       .axis = {0, 0, 1}});

  std::stringstream ss;
  world1.saveBinary(ss);

  dart::simulation::experimental::World world2;
  world2.loadBinary(ss);

  // Verify Multibody restored
  auto mb_restored = world2.getMultibody("robot");
  ASSERT_TRUE(mb_restored.has_value());
  EXPECT_EQ(mb_restored->getName(), "robot");
  EXPECT_EQ(mb_restored->getLinkCount(), 2);
  EXPECT_EQ(mb_restored->getJointCount(), 1);

  // Verify links are accessible by name
  auto base_restored = mb_restored->getLink("base");
  ASSERT_TRUE(base_restored.has_value());
  EXPECT_EQ(base_restored->getName(), "base");

  auto link1_restored = mb_restored->getLink("link1");
  ASSERT_TRUE(link1_restored.has_value());
  EXPECT_EQ(link1_restored->getName(), "link1");

  // Verify joint properties
  auto joint1_restored = mb_restored->getJoint("joint1");
  ASSERT_TRUE(joint1_restored.has_value());
  EXPECT_EQ(joint1_restored->getName(), "joint1");
  EXPECT_EQ(
      joint1_restored->getType(),
      dart::simulation::experimental::JointType::Revolute);

  // Verify joint axis
  auto axis = joint1_restored->getAxis();
  EXPECT_DOUBLE_EQ(axis[0], 0.0);
  EXPECT_DOUBLE_EQ(axis[1], 0.0);
  EXPECT_DOUBLE_EQ(axis[2], 1.0);
}

// Test save/load preserves per-coordinate joint position, velocity, and effort
// limits.
TEST(Serialization, PreservesJointLimits)
{
  namespace sx = dart::simulation::experimental;

  sx::World world1;
  auto mb = world1.addMultibody("robot");
  auto base = mb.addLink("base");
  auto link = mb.addLink(
      "link",
      base,
      sx::JointSpec{.name = "joint", .type = sx::JointType::Revolute});

  auto joint = link.getParentJoint();
  joint.setPositionLimits(
      Eigen::VectorXd::Constant(1, -0.5), Eigen::VectorXd::Constant(1, 0.5));
  joint.setVelocityLimits(
      Eigen::VectorXd::Constant(1, -1.5), Eigen::VectorXd::Constant(1, 2.5));
  joint.setEffortLimits(
      Eigen::VectorXd::Constant(1, -3.0), Eigen::VectorXd::Constant(1, 4.0));

  std::stringstream ss;
  world1.saveBinary(ss);

  sx::World world2;
  world2.loadBinary(ss);

  auto mb_restored = world2.getMultibody("robot");
  ASSERT_TRUE(mb_restored.has_value());
  auto joint_restored = mb_restored->getJoint("joint");
  ASSERT_TRUE(joint_restored.has_value());

  EXPECT_DOUBLE_EQ(joint_restored->getPositionLowerLimits()[0], -0.5);
  EXPECT_DOUBLE_EQ(joint_restored->getPositionUpperLimits()[0], 0.5);
  EXPECT_DOUBLE_EQ(joint_restored->getVelocityLowerLimits()[0], -1.5);
  EXPECT_DOUBLE_EQ(joint_restored->getVelocityUpperLimits()[0], 2.5);
  EXPECT_DOUBLE_EQ(joint_restored->getEffortLowerLimits()[0], -3.0);
  EXPECT_DOUBLE_EQ(joint_restored->getEffortUpperLimits()[0], 4.0);
}

// Test that v1 joint records migrate the legacy single-sided velocity/effort
// limit vectors and default fields added in later binary formats.
TEST(Serialization, LoadsLegacyV1JointRecord)
{
  namespace sx = dart::simulation::experimental;
  namespace comps = dart::simulation::experimental::comps;

  sx::World world1;
  auto mb = world1.addMultibody("robot");
  auto base = mb.addLink("base");
  auto link = mb.addLink(
      "link",
      base,
      sx::JointSpec{
          .name = "joint",
          .type = sx::JointType::Revolute,
          .axis = Eigen::Vector3d::UnitX()});

  auto joint = link.getParentJoint();
  joint.setPosition(Eigen::VectorXd::Constant(1, 0.25));
  joint.setVelocity(Eigen::VectorXd::Constant(1, -0.75));
  world1.getRegistry().get<comps::Joint>(joint.getEntity()).acceleration
      = Eigen::VectorXd::Constant(1, 1.25);
  joint.setForce(Eigen::VectorXd::Constant(1, 2.25));
  joint.setPositionLimits(
      Eigen::VectorXd::Constant(1, -0.5), Eigen::VectorXd::Constant(1, 0.5));
  joint.setVelocityLimits(
      Eigen::VectorXd::Constant(1, -1.5), Eigen::VectorXd::Constant(1, 1.5));
  joint.setEffortLimits(
      Eigen::VectorXd::Constant(1, -3.0), Eigen::VectorXd::Constant(1, 3.0));

  std::stringstream legacy;
  saveLegacyV1WorldWithCurrentEntities(legacy, world1);

  sx::World world2;
  world2.loadBinary(legacy);

  auto mbRestored = world2.getMultibody("robot");
  ASSERT_TRUE(mbRestored.has_value());
  auto jointRestored = mbRestored->getJoint("joint");
  ASSERT_TRUE(jointRestored.has_value());
  EXPECT_EQ(jointRestored->getType(), sx::JointType::Revolute);
  EXPECT_TRUE(jointRestored->getAxis().isApprox(Eigen::Vector3d::UnitX()));
  EXPECT_DOUBLE_EQ(jointRestored->getPosition()[0], 0.25);
  EXPECT_DOUBLE_EQ(jointRestored->getVelocity()[0], -0.75);
  EXPECT_DOUBLE_EQ(jointRestored->getAcceleration()[0], 1.25);
  EXPECT_DOUBLE_EQ(jointRestored->getForce()[0], 2.25);
  EXPECT_DOUBLE_EQ(jointRestored->getPositionLowerLimits()[0], -0.5);
  EXPECT_DOUBLE_EQ(jointRestored->getPositionUpperLimits()[0], 0.5);
  EXPECT_DOUBLE_EQ(jointRestored->getVelocityLowerLimits()[0], -1.5);
  EXPECT_DOUBLE_EQ(jointRestored->getVelocityUpperLimits()[0], 1.5);
  EXPECT_DOUBLE_EQ(jointRestored->getEffortLowerLimits()[0], -3.0);
  EXPECT_DOUBLE_EQ(jointRestored->getEffortUpperLimits()[0], 3.0);

  const auto& jointComp
      = world2.getRegistry().get<comps::Joint>(jointRestored->getEntity());
  EXPECT_EQ(jointComp.actuatorType, comps::ActuatorType::Force);
  EXPECT_TRUE(jointComp.springStiffness.isZero());
  EXPECT_TRUE(jointComp.dampingCoefficient.isZero());
  EXPECT_TRUE(jointComp.restPosition.isZero());
  EXPECT_TRUE(jointComp.armature.isZero());
  EXPECT_TRUE(jointComp.coulombFriction.isZero());
  EXPECT_TRUE(jointComp.commandVelocity.isZero());
}

TEST(Serialization, LoadsLegacyV8LinkRecord)
{
  namespace sx = dart::simulation::experimental;
  namespace comps = dart::simulation::experimental::comps;

  sx::World world1;
  auto mb = world1.addMultibody("robot");
  auto base = mb.addLink("base");
  auto link = mb.addLink(
      "link",
      base,
      sx::JointSpec{.name = "joint", .type = sx::JointType::Revolute});

  Eigen::Isometry3d legacyJointToLink = Eigen::Isometry3d::Identity();
  legacyJointToLink.translate(Eigen::Vector3d(0.25, -0.5, 0.75));
  legacyJointToLink.rotate(Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitZ()));

  Eigen::Isometry3d unsavedParentToJoint = Eigen::Isometry3d::Identity();
  unsavedParentToJoint.translate(Eigen::Vector3d(1.0, 2.0, 3.0));

  Eigen::Isometry3d worldTransform = Eigen::Isometry3d::Identity();
  worldTransform.translate(Eigen::Vector3d(-1.0, 0.5, 2.0));

  Eigen::Matrix<double, 6, 1> externalForce;
  externalForce << 1.0, -2.0, 3.0, -4.0, 5.0, -6.0;

  auto& linkComp = world1.getRegistry().get<comps::Link>(link.getEntity());
  linkComp.transformFromParentToJoint = unsavedParentToJoint;
  linkComp.transformFromParentJoint = legacyJointToLink;
  linkComp.worldTransform = worldTransform;
  linkComp.externalForce = externalForce;

  std::stringstream legacy;
  saveLegacyV8WorldWithCurrentEntities(legacy, world1);

  sx::World world2;
  world2.loadBinary(legacy);

  auto mbRestored = world2.getMultibody("robot");
  ASSERT_TRUE(mbRestored.has_value());
  auto linkRestored = mbRestored->getLink("link");
  ASSERT_TRUE(linkRestored.has_value());

  const auto& restoredLinkComp
      = world2.getRegistry().get<comps::Link>(linkRestored->getEntity());
  EXPECT_TRUE(restoredLinkComp.transformFromParentToJoint.isApprox(
      Eigen::Isometry3d::Identity()));
  EXPECT_TRUE(
      restoredLinkComp.transformFromParentJoint.isApprox(legacyJointToLink));
  EXPECT_TRUE(restoredLinkComp.worldTransform.isApprox(worldTransform));
  EXPECT_TRUE(restoredLinkComp.externalForce.isApprox(externalForce));

  auto jointRestored = mbRestored->getJoint("joint");
  ASSERT_TRUE(jointRestored.has_value());
  EXPECT_EQ(restoredLinkComp.parentJoint, jointRestored->getEntity());

  auto baseRestored = mbRestored->getLink("base");
  ASSERT_TRUE(baseRestored.has_value());
  const auto& restoredBaseComp
      = world2.getRegistry().get<comps::Link>(baseRestored->getEntity());
  ASSERT_EQ(restoredBaseComp.childJoints.size(), 1u);
  EXPECT_EQ(restoredBaseComp.childJoints.front(), jointRestored->getEntity());
}

// Test save/load preserves names
TEST(Serialization, PreservesNames)
{
  dart::simulation::experimental::World world1;
  auto mb = world1.addMultibody("test_robot");
  [[maybe_unused]] auto base = mb.addLink("base_link");
  [[maybe_unused]] auto link = mb.addLink(
      "arm_link", {.parentLink = base, .jointName = "shoulder_joint"});

  std::stringstream ss;
  world1.saveBinary(ss);

  dart::simulation::experimental::World world2;
  world2.loadBinary(ss);

  auto mb_restored = world2.getMultibody("test_robot");
  ASSERT_TRUE(mb_restored.has_value());
  EXPECT_EQ(mb_restored->getName(), "test_robot");
}

// Test save/load with rigid bodies
TEST(Serialization, WithRigidBodies)
{
  dart::simulation::experimental::World world1;
  [[maybe_unused]] auto rb1 = world1.addRigidBody("box1");
  [[maybe_unused]] auto rb2 = world1.addRigidBody("box2");

  std::stringstream ss;
  world1.saveBinary(ss);

  dart::simulation::experimental::World world2;
  world2.loadBinary(ss);

  EXPECT_EQ(world2.getRigidBodyCount(), 2);
  EXPECT_TRUE(world2.hasRigidBody("box1"));
  EXPECT_TRUE(world2.hasRigidBody("box2"));
}

TEST(Serialization, PreservesRigidBodyCollisionComponents)
{
  namespace sx = dart::simulation::experimental;

  sx::World world1;
  auto ground = world1.addRigidBody("ground");
  ground.setStatic(true);
  ground.setRestitution(0.75);
  ground.setFriction(0.25);
  sx::CollisionShape groundCollisionShape
      = sx::CollisionShape::makeBox(Eigen::Vector3d(1.0, 2.0, 0.5));
  groundCollisionShape.localTransform.translation()
      = Eigen::Vector3d(0.25, -0.5, 0.75);
  ground.setCollisionShape(groundCollisionShape);
  ground.setDeformableGroundBarrier(true);
  ground.setDeformableSurfaceCcdObstacle(true);

  auto ball = world1.addRigidBody("ball");
  ball.setCollisionShape(sx::CollisionShape::makeSphere(0.3));
  sx::CollisionShape ballCompoundShape
      = sx::CollisionShape::makeBox(Eigen::Vector3d(0.1, 0.2, 0.3));
  ballCompoundShape.localTransform.translation()
      = Eigen::Vector3d(0.5, 0.0, 0.0);
  ball.addCollisionShape(ballCompoundShape);

  auto capsule = world1.addRigidBody("capsule");
  sx::CollisionShape capsuleCollisionShape
      = sx::CollisionShape::makeCapsule(0.2, 0.4);
  capsuleCollisionShape.localTransform.translation()
      = Eigen::Vector3d(-0.25, 0.5, -0.75);
  capsule.setCollisionShape(capsuleCollisionShape);

  auto cylinder = world1.addRigidBody("cylinder");
  sx::CollisionShape cylinderCollisionShape
      = sx::CollisionShape::makeCylinder(0.25, 0.45);
  cylinderCollisionShape.localTransform.translation()
      = Eigen::Vector3d(0.4, -0.2, 0.1);
  cylinder.setCollisionShape(cylinderCollisionShape);

  auto plane = world1.addRigidBody("plane");
  sx::CollisionShape planeCollisionShape
      = sx::CollisionShape::makePlane(Eigen::Vector3d::UnitZ(), 0.15);
  planeCollisionShape.localTransform.translation()
      = Eigen::Vector3d(-0.3, 0.2, 0.05);
  plane.setCollisionShape(planeCollisionShape);

  auto mesh = world1.addRigidBody("mesh");
  sx::CollisionShape meshCollisionShape = sx::CollisionShape::makeMesh(
      {Eigen::Vector3d(-1.0, -1.0, 0.0),
       Eigen::Vector3d(1.0, -1.0, 0.0),
       Eigen::Vector3d(-1.0, 1.0, 0.0),
       Eigen::Vector3d(1.0, 1.0, 0.0)},
      {Eigen::Vector3i(0, 1, 2), Eigen::Vector3i(1, 3, 2)});
  meshCollisionShape.localTransform.translation()
      = Eigen::Vector3d(0.1, 0.2, 0.3);
  mesh.setCollisionShape(meshCollisionShape);

  std::stringstream ss;
  world1.saveBinary(ss);

  sx::World world2;
  world2.loadBinary(ss);

  auto groundRestored = world2.getRigidBody("ground");
  ASSERT_TRUE(groundRestored.has_value());
  EXPECT_TRUE(groundRestored->isStatic());
  EXPECT_DOUBLE_EQ(groundRestored->getRestitution(), 0.75);
  EXPECT_DOUBLE_EQ(groundRestored->getFriction(), 0.25);
  EXPECT_TRUE(groundRestored->isDeformableGroundBarrier());
  EXPECT_TRUE(groundRestored->isDeformableSurfaceCcdObstacle());

  auto groundShape = groundRestored->getCollisionShape();
  ASSERT_TRUE(groundShape.has_value());
  EXPECT_EQ(groundShape->type, sx::CollisionShapeType::Box);
  EXPECT_TRUE(
      groundShape->halfExtents.isApprox(Eigen::Vector3d(1.0, 2.0, 0.5)));
  EXPECT_TRUE(groundShape->localTransform.isApprox(
      groundCollisionShape.localTransform, 1e-12));

  auto ballRestored = world2.getRigidBody("ball");
  ASSERT_TRUE(ballRestored.has_value());
  EXPECT_FALSE(ballRestored->isStatic());

  auto ballShape = ballRestored->getCollisionShape();
  ASSERT_TRUE(ballShape.has_value());
  EXPECT_EQ(ballShape->type, sx::CollisionShapeType::Sphere);
  EXPECT_DOUBLE_EQ(ballShape->radius, 0.3);
  const auto ballShapes = ballRestored->getCollisionShapes();
  ASSERT_EQ(ballShapes.size(), 2u);
  EXPECT_EQ(ballShapes[0].type, sx::CollisionShapeType::Sphere);
  EXPECT_EQ(ballShapes[1].type, sx::CollisionShapeType::Box);
  EXPECT_TRUE(
      ballShapes[1].halfExtents.isApprox(
          Eigen::Vector3d(0.1, 0.2, 0.3), 1e-12));
  EXPECT_TRUE(
      ballShapes[1].localTransform.isApprox(
          ballCompoundShape.localTransform, 1e-12));

  auto capsuleRestored = world2.getRigidBody("capsule");
  ASSERT_TRUE(capsuleRestored.has_value());
  auto capsuleShape = capsuleRestored->getCollisionShape();
  ASSERT_TRUE(capsuleShape.has_value());
  EXPECT_EQ(capsuleShape->type, sx::CollisionShapeType::Capsule);
  EXPECT_DOUBLE_EQ(capsuleShape->radius, 0.2);
  EXPECT_TRUE(
      capsuleShape->halfExtents.isApprox(Eigen::Vector3d(0.2, 0.2, 0.4)));
  EXPECT_TRUE(capsuleShape->localTransform.isApprox(
      capsuleCollisionShape.localTransform, 1e-12));

  auto cylinderRestored = world2.getRigidBody("cylinder");
  ASSERT_TRUE(cylinderRestored.has_value());
  auto cylinderShape = cylinderRestored->getCollisionShape();
  ASSERT_TRUE(cylinderShape.has_value());
  EXPECT_EQ(cylinderShape->type, sx::CollisionShapeType::Cylinder);
  EXPECT_DOUBLE_EQ(cylinderShape->radius, 0.25);
  EXPECT_TRUE(
      cylinderShape->halfExtents.isApprox(Eigen::Vector3d(0.25, 0.25, 0.45)));
  EXPECT_TRUE(cylinderShape->localTransform.isApprox(
      cylinderCollisionShape.localTransform, 1e-12));

  auto planeRestored = world2.getRigidBody("plane");
  ASSERT_TRUE(planeRestored.has_value());
  auto planeShape = planeRestored->getCollisionShape();
  ASSERT_TRUE(planeShape.has_value());
  EXPECT_EQ(planeShape->type, sx::CollisionShapeType::Plane);
  EXPECT_TRUE(planeShape->normal.isApprox(Eigen::Vector3d::UnitZ(), 1e-12));
  EXPECT_DOUBLE_EQ(planeShape->offset, 0.15);
  EXPECT_TRUE(planeShape->localTransform.isApprox(
      planeCollisionShape.localTransform, 1e-12));

  auto meshRestored = world2.getRigidBody("mesh");
  ASSERT_TRUE(meshRestored.has_value());
  auto meshShape = meshRestored->getCollisionShape();
  ASSERT_TRUE(meshShape.has_value());
  EXPECT_EQ(meshShape->type, sx::CollisionShapeType::Mesh);
  ASSERT_EQ(meshShape->vertices.size(), meshCollisionShape.vertices.size());
  ASSERT_EQ(meshShape->triangles.size(), meshCollisionShape.triangles.size());
  EXPECT_TRUE(
      meshShape->vertices[0].isApprox(meshCollisionShape.vertices[0], 1e-12));
  EXPECT_EQ(meshShape->triangles[1], meshCollisionShape.triangles[1]);
  EXPECT_TRUE(meshShape->localTransform.isApprox(
      meshCollisionShape.localTransform, 1e-12));
}

TEST(Serialization, WithLoopClosures)
{
  namespace sx = dart::simulation::experimental;

  sx::World world1;
  auto robot = world1.addMultibody("robot");
  auto base = robot.addLink("base");
  auto link = robot.addLink("link", {.parentLink = base, .jointName = "joint"});
  auto ground = world1.addRigidBody("ground");

  Eigen::Isometry3d offsetA = Eigen::Isometry3d::Identity();
  offsetA.translate(Eigen::Vector3d(0.25, 0.0, 0.0));
  Eigen::Isometry3d offsetB = Eigen::Isometry3d::Identity();
  offsetB.translate(Eigen::Vector3d(-0.25, 0.0, 0.0));

  auto closure1 = world1.addLoopClosure(
      "closure",
      {.frameA = link,
       .frameB = ground,
       .family = sx::LoopClosureFamily::Point,
       .offsetA = offsetA,
       .offsetB = offsetB});
  closure1.setRuntimePolicy(
      {.enabled = false,
       .kinematics = sx::ClosureKinematicsPolicy::Project,
       .dynamics = sx::ClosureDynamicsPolicy::Solve});

  std::stringstream ss;
  world1.saveBinary(ss);

  sx::World world2;
  world2.loadBinary(ss);

  EXPECT_EQ(world2.getLoopClosureCount(), 1u);
  auto closure = world2.getLoopClosure("closure");
  ASSERT_TRUE(closure.has_value());
  EXPECT_EQ(closure->getFamily(), sx::LoopClosureFamily::Point);
  EXPECT_TRUE(closure->getOffsetA().isApprox(offsetA));
  EXPECT_TRUE(closure->getOffsetB().isApprox(offsetB));
  const auto runtimePolicy = closure->getRuntimePolicy();
  EXPECT_FALSE(runtimePolicy.enabled);
  EXPECT_EQ(runtimePolicy.kinematics, sx::ClosureKinematicsPolicy::Project);
  EXPECT_EQ(runtimePolicy.dynamics, sx::ClosureDynamicsPolicy::Solve);

  auto restoredRobot = world2.getMultibody("robot");
  ASSERT_TRUE(restoredRobot.has_value());
  auto restoredLink = restoredRobot->getLink("link");
  ASSERT_TRUE(restoredLink.has_value());
  auto restoredGround = world2.getRigidBody("ground");
  ASSERT_TRUE(restoredGround.has_value());
  EXPECT_TRUE(closure->getFrameA().isSameInstanceAs(*restoredLink));
  EXPECT_TRUE(closure->getFrameB().isSameInstanceAs(*restoredGround));

  auto restoredBase = restoredRobot->getLink("base");
  ASSERT_TRUE(restoredBase.has_value());
  auto nextClosure = world2.addLoopClosure(
      "", {.frameA = *restoredBase, .frameB = *restoredGround});
  EXPECT_EQ(nextClosure.getName(), "loop_closure_002");
}

// Test save/load preserves simulation mode
TEST(Serialization, PreservesSimulationMode)
{
  dart::simulation::experimental::World world1;
  world1.addMultibody("robot");
  world1.enterSimulationMode();
  ASSERT_TRUE(world1.isSimulationMode());

  std::stringstream ss;
  world1.saveBinary(ss);

  dart::simulation::experimental::World world2;
  world2.loadBinary(ss);

  EXPECT_TRUE(world2.isSimulationMode());
}

// Test save/load preserves world timing metadata
TEST(Serialization, PreservesWorldTimingMetadata)
{
  dart::simulation::experimental::World world1;
  world1.setTimeStep(0.125);
  world1.setTime(2.0);
  world1.setGravity(Eigen::Vector3d(0.0, -1.5, -3.0));
  world1.enterSimulationMode();
  world1.step();

  std::stringstream ss;
  world1.saveBinary(ss);

  dart::simulation::experimental::World world2;
  world2.loadBinary(ss);

  EXPECT_TRUE(world2.isSimulationMode());
  EXPECT_DOUBLE_EQ(world2.getTimeStep(), 0.125);
  EXPECT_DOUBLE_EQ(world2.getTime(), 2.125);
  EXPECT_EQ(world2.getFrame(), 1u);
  EXPECT_TRUE(world2.getGravity().isApprox(Eigen::Vector3d(0.0, -1.5, -3.0)));
}

// Test that legacy world records without versioned gravity metadata leave
// trailing stream bytes untouched instead of consuming them as gravity.
TEST(Serialization, LegacyWorldMetadataDoesNotConsumeTrailingBytesAsGravity)
{
  dart::simulation::experimental::World world1;
  world1.setTimeStep(0.125);
  world1.setTime(2.0);
  world1.setGravity(Eigen::Vector3d(0.0, -1.5, -3.0));

  std::stringstream saved;
  world1.saveBinary(saved);

  std::string legacyRecord = saved.str();
  ASSERT_GE(
      legacyRecord.size(),
      2 * sizeof(std::uint32_t) + 3 * sizeof(double) + sizeof(std::uint8_t));

  const std::uint32_t legacyVersion = 1;
  std::memcpy(
      legacyRecord.data() + sizeof(std::uint32_t),
      &legacyVersion,
      sizeof(legacyVersion));
  // Format v3 appends a one-byte differentiable flag after the deformable-body
  // counter. Drop it so the remaining record matches the v2 tail layout this
  // legacy-compatibility check was written against (deformable counter, then
  // the gravity block as the last three doubles).
  legacyRecord.pop_back();
  legacyRecord.resize(legacyRecord.size() - 3 * sizeof(double));

  constexpr std::string_view trailerBytes = "0123456789abcdefghijklmn";
  static_assert(trailerBytes.size() == 3 * sizeof(double));
  const std::string trailer(trailerBytes);
  legacyRecord += trailer;

  std::stringstream input(legacyRecord);
  dart::simulation::experimental::World world2;
  world2.loadBinary(input);

  EXPECT_DOUBLE_EQ(world2.getTimeStep(), 0.125);
  EXPECT_DOUBLE_EQ(world2.getTime(), 2.0);
  EXPECT_TRUE(world2.getGravity().isApprox(Eigen::Vector3d(0.0, 0.0, -9.81)));

  const std::string remaining{
      std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
  EXPECT_EQ(remaining, trailer);
}

// Test save/load preserves auto-generation counters
TEST(Serialization, PreservesCounters)
{
  dart::simulation::experimental::World world1;
  [[maybe_unused]] auto mb1 = world1.addMultibody("");
  [[maybe_unused]] auto mb2 = world1.addMultibody("");

  std::stringstream ss;
  world1.saveBinary(ss);

  dart::simulation::experimental::World world2;
  world2.loadBinary(ss);

  auto mb3 = world2.addMultibody("");
  EXPECT_EQ(mb3.getName(), "multibody_003");
}

// Test multiple save/load cycles
TEST(Serialization, MultipleCycles)
{
  dart::simulation::experimental::World world1;
  auto mb = world1.addMultibody("robot");
  mb.addLink("base");

  for (int i = 0; i < 3; ++i) {
    std::stringstream ss;
    world1.saveBinary(ss);

    dart::simulation::experimental::World world2;
    world2.loadBinary(ss);

    std::stringstream ss2;
    world2.saveBinary(ss2);
    world1.loadBinary(ss2);

    EXPECT_EQ(world1.getMultibodyCount(), 1);
    auto mb_restored = world1.getMultibody("robot");
    ASSERT_TRUE(mb_restored.has_value());
    EXPECT_EQ(mb_restored->getLinkCount(), 1);
  }
}

// Test load clears existing state
TEST(Serialization, LoadClearsExisting)
{
  dart::simulation::experimental::World world1;
  world1.addMultibody("robot1");

  std::stringstream ss;
  world1.saveBinary(ss);

  dart::simulation::experimental::World world2;
  world2.addMultibody("robot2");
  world2.addMultibody("robot3");
  EXPECT_EQ(world2.getMultibodyCount(), 2);

  world2.loadBinary(ss);
  EXPECT_EQ(world2.getMultibodyCount(), 1);
  EXPECT_TRUE(world2.getMultibody("robot1").has_value());
  EXPECT_FALSE(world2.getMultibody("robot2").has_value());
  EXPECT_FALSE(world2.getMultibody("robot3").has_value());
}

// Test saves and restores joint type (REVOLUTE)
TEST(Serialization, JointTypeRevolute)
{
  dart::simulation::experimental::World world1;
  auto mb = world1.addMultibody("robot");
  [[maybe_unused]] auto base = mb.addLink("base");
  [[maybe_unused]] auto link1 = mb.addLink(
      "link1",
      {.parentLink = base,
       .jointName = "j1",
       .jointType = dart::simulation::experimental::JointType::Revolute,
       .axis = {0, 0, 1}});

  std::stringstream ss;
  world1.saveBinary(ss);

  dart::simulation::experimental::World world2;
  world2.loadBinary(ss);

  [[maybe_unused]] auto mb2 = world2.getMultibody("robot");
  ASSERT_TRUE(mb2.has_value());
  EXPECT_EQ(mb2->getJointCount(), 1);
}

// Test saves and restores joint type (PRISMATIC)
TEST(Serialization, JointTypePrismatic)
{
  dart::simulation::experimental::World world1;
  auto mb = world1.addMultibody("robot");
  [[maybe_unused]] auto base = mb.addLink("base");
  [[maybe_unused]] auto link1 = mb.addLink(
      "link1",
      {.parentLink = base,
       .jointName = "j1",
       .jointType = dart::simulation::experimental::JointType::Prismatic,
       .axis = {1, 0, 0}});

  std::stringstream ss;
  world1.saveBinary(ss);

  dart::simulation::experimental::World world2;
  world2.loadBinary(ss);

  [[maybe_unused]] auto mb2 = world2.getMultibody("robot");
  ASSERT_TRUE(mb2.has_value());
  EXPECT_EQ(mb2->getJointCount(), 1);
}

// Test complex hierarchy (6-DOF manipulator)
TEST(Serialization, ComplexHierarchy)
{
  dart::simulation::experimental::World world1;
  auto robot = world1.addMultibody("ur5");
  [[maybe_unused]] auto base = robot.addLink("base");
  [[maybe_unused]] auto link1 = robot.addLink(
      "shoulder_link",
      {.parentLink = base,
       .jointName = "shoulder_pan",
       .jointType = dart::simulation::experimental::JointType::Revolute,
       .axis = {0, 0, 1}});
  auto link2 = robot.addLink(
      "upper_arm_link",
      {.parentLink = link1,
       .jointName = "shoulder_lift",
       .jointType = dart::simulation::experimental::JointType::Revolute,
       .axis = {0, 1, 0}});
  auto link3 = robot.addLink(
      "forearm_link",
      {.parentLink = link2,
       .jointName = "elbow",
       .jointType = dart::simulation::experimental::JointType::Revolute,
       .axis = {0, 1, 0}});
  auto link4 = robot.addLink(
      "wrist_1_link",
      {.parentLink = link3,
       .jointName = "wrist_1",
       .jointType = dart::simulation::experimental::JointType::Revolute,
       .axis = {0, 1, 0}});
  auto link5 = robot.addLink(
      "wrist_2_link",
      {.parentLink = link4,
       .jointName = "wrist_2",
       .jointType = dart::simulation::experimental::JointType::Revolute,
       .axis = {0, 0, 1}});
  [[maybe_unused]] auto link6 = robot.addLink(
      "wrist_3_link",
      {.parentLink = link5,
       .jointName = "wrist_3",
       .jointType = dart::simulation::experimental::JointType::Revolute,
       .axis = {0, 1, 0}});

  std::stringstream ss;
  world1.saveBinary(ss);

  dart::simulation::experimental::World world2;
  world2.loadBinary(ss);

  auto robot_restored = world2.getMultibody("ur5");
  ASSERT_TRUE(robot_restored.has_value());
  EXPECT_EQ(robot_restored->getLinkCount(), 7);
  EXPECT_EQ(robot_restored->getJointCount(), 6);

  // Verify all link names are preserved
  EXPECT_TRUE(robot_restored->getLink("base").has_value());
  EXPECT_TRUE(robot_restored->getLink("shoulder_link").has_value());
  EXPECT_TRUE(robot_restored->getLink("upper_arm_link").has_value());
  EXPECT_TRUE(robot_restored->getLink("forearm_link").has_value());
  EXPECT_TRUE(robot_restored->getLink("wrist_1_link").has_value());
  EXPECT_TRUE(robot_restored->getLink("wrist_2_link").has_value());
  EXPECT_TRUE(robot_restored->getLink("wrist_3_link").has_value());

  // Verify all joints and their properties
  auto shoulder_pan = robot_restored->getJoint("shoulder_pan");
  ASSERT_TRUE(shoulder_pan.has_value());
  EXPECT_EQ(
      shoulder_pan->getType(),
      dart::simulation::experimental::JointType::Revolute);
  auto axis1 = shoulder_pan->getAxis();
  EXPECT_DOUBLE_EQ(axis1[0], 0.0);
  EXPECT_DOUBLE_EQ(axis1[1], 0.0);
  EXPECT_DOUBLE_EQ(axis1[2], 1.0);

  auto shoulder_lift = robot_restored->getJoint("shoulder_lift");
  ASSERT_TRUE(shoulder_lift.has_value());
  EXPECT_EQ(
      shoulder_lift->getType(),
      dart::simulation::experimental::JointType::Revolute);
  auto axis2 = shoulder_lift->getAxis();
  EXPECT_DOUBLE_EQ(axis2[0], 0.0);
  EXPECT_DOUBLE_EQ(axis2[1], 1.0);
  EXPECT_DOUBLE_EQ(axis2[2], 0.0);

  auto elbow = robot_restored->getJoint("elbow");
  ASSERT_TRUE(elbow.has_value());
  EXPECT_EQ(
      elbow->getType(), dart::simulation::experimental::JointType::Revolute);
  auto axis3 = elbow->getAxis();
  EXPECT_DOUBLE_EQ(axis3[0], 0.0);
  EXPECT_DOUBLE_EQ(axis3[1], 1.0);
  EXPECT_DOUBLE_EQ(axis3[2], 0.0);

  auto wrist_1 = robot_restored->getJoint("wrist_1");
  ASSERT_TRUE(wrist_1.has_value());
  EXPECT_EQ(
      wrist_1->getType(), dart::simulation::experimental::JointType::Revolute);

  auto wrist_2 = robot_restored->getJoint("wrist_2");
  ASSERT_TRUE(wrist_2.has_value());
  EXPECT_EQ(
      wrist_2->getType(), dart::simulation::experimental::JointType::Revolute);
  auto axis5 = wrist_2->getAxis();
  EXPECT_DOUBLE_EQ(axis5[0], 0.0);
  EXPECT_DOUBLE_EQ(axis5[1], 0.0);
  EXPECT_DOUBLE_EQ(axis5[2], 1.0);

  auto wrist_3 = robot_restored->getJoint("wrist_3");
  ASSERT_TRUE(wrist_3.has_value());
  EXPECT_EQ(
      wrist_3->getType(), dart::simulation::experimental::JointType::Revolute);
  auto axis6 = wrist_3->getAxis();
  EXPECT_DOUBLE_EQ(axis6[0], 0.0);
  EXPECT_DOUBLE_EQ(axis6[1], 1.0);
  EXPECT_DOUBLE_EQ(axis6[2], 0.0);
}

// Test multiple multibodies
TEST(Serialization, MultipleMultibodies)
{
  dart::simulation::experimental::World world1;
  [[maybe_unused]] auto mb1 = world1.addMultibody("robot1");
  [[maybe_unused]] auto mb2 = world1.addMultibody("robot2");
  auto mb3 = world1.addMultibody("robot3");

  mb1.addLink("base1");
  mb2.addLink("base2");
  mb3.addLink("base3");

  std::stringstream ss;
  world1.saveBinary(ss);

  dart::simulation::experimental::World world2;
  world2.loadBinary(ss);

  EXPECT_EQ(world2.getMultibodyCount(), 3);
  EXPECT_TRUE(world2.getMultibody("robot1").has_value());
  EXPECT_TRUE(world2.getMultibody("robot2").has_value());
  EXPECT_TRUE(world2.getMultibody("robot3").has_value());
}

// Test mixed multibodies and rigid bodies
TEST(Serialization, MixedContent)
{
  dart::simulation::experimental::World world1;

  auto mb = world1.addMultibody("robot");
  mb.addLink("base");

  [[maybe_unused]] auto rb1 = world1.addRigidBody("box1");
  [[maybe_unused]] auto rb2 = world1.addRigidBody("box2");

  std::stringstream ss;
  world1.saveBinary(ss);

  dart::simulation::experimental::World world2;
  world2.loadBinary(ss);

  EXPECT_EQ(world2.getMultibodyCount(), 1);
  EXPECT_EQ(world2.getRigidBodyCount(), 2);
  EXPECT_TRUE(world2.getMultibody("robot").has_value());
  EXPECT_TRUE(world2.hasRigidBody("box1"));
  EXPECT_TRUE(world2.hasRigidBody("box2"));
}

// Test auto-generated names are preserved
TEST(Serialization, EmptyNames)
{
  dart::simulation::experimental::World world1;
  auto mb = world1.addMultibody("");
  auto base = mb.addLink("");
  dart::simulation::experimental::LinkOptions opts{
      .parentLink = base, .jointName = ""};
  [[maybe_unused]] auto child = mb.addLink("", opts);

  std::stringstream ss;
  world1.saveBinary(ss);

  dart::simulation::experimental::World world2;
  world2.loadBinary(ss);

  auto restoredMb = world2.getMultibody("multibody_001");
  ASSERT_TRUE(restoredMb.has_value());
  EXPECT_EQ(restoredMb->getName(), "multibody_001");

  auto restoredBase = restoredMb->getLink("link_001");
  ASSERT_TRUE(restoredBase.has_value());
  EXPECT_EQ(restoredBase->getName(), "link_001");

  auto restoredChild = restoredMb->getLink("link_002");
  ASSERT_TRUE(restoredChild.has_value());
  EXPECT_EQ(restoredChild->getName(), "link_002");

  auto restoredJoint = restoredMb->getJoint("joint_001");
  ASSERT_TRUE(restoredJoint.has_value());
  EXPECT_EQ(restoredJoint->getName(), "joint_001");
}

// Test large hierarchy (stress test)
TEST(Serialization, LargeHierarchy)
{
  dart::simulation::experimental::World world1;
  auto robot = world1.addMultibody("large_robot");

  // Create chain of 20 links
  auto prev = robot.addLink("base");
  for (int i = 1; i < 20; ++i) {
    prev = robot.addLink(
        "link_" + std::to_string(i),
        {.parentLink = prev, .jointName = "joint_" + std::to_string(i)});
  }

  std::stringstream ss;
  world1.saveBinary(ss);

  dart::simulation::experimental::World world2;
  world2.loadBinary(ss);

  auto robot2 = world2.getMultibody("large_robot");
  ASSERT_TRUE(robot2.has_value());
  EXPECT_EQ(robot2->getLinkCount(), 20);
  EXPECT_EQ(robot2->getJointCount(), 19);
}

//==============================================================================
// Properties/State/Cache Serialization Tests
//==============================================================================

// Test cache components are NOT serialized
TEST(Serialization, CacheNotSerialized)
{
  dart::simulation::experimental::World world;

  // Create a FreeFrame and set a transform
  auto frame = world.addFreeFrame("test");
  Eigen::Isometry3d T1 = Eigen::Isometry3d::Identity();
  T1.translate(Eigen::Vector3d(10, 20, 30));
  frame.setLocalTransform(T1);

  // Trigger cache computation to populate worldTransform
  auto cachedTransform = frame.getTransform();
  EXPECT_TRUE(cachedTransform.isApprox(T1));

  // Verify cache is clean
  {
    auto& registry = world.getRegistry();
    auto entity = frame.getEntity();
    ASSERT_TRUE(registry.valid(entity)) << "Entity should be valid";
    ASSERT_TRUE(
        registry.all_of<dart::simulation::experimental::comps::FrameCache>(
            entity))
        << "Entity should have FrameCache component";

    const auto& cache
        = registry.get<dart::simulation::experimental::comps::FrameCache>(
            entity);
    EXPECT_FALSE(cache.needTransformUpdate)
        << "Cache should be clean after getTransform()";
  }

  // Serialize
  std::stringstream ss;
  world.saveBinary(ss);

  // Deserialize into new world
  dart::simulation::experimental::World world2;
  world2.loadBinary(ss);

  // Check the registry directly for the restored FreeFrame
  auto& registry2 = world2.getRegistry();
  auto view
      = registry2.view<dart::simulation::experimental::comps::FreeFrameTag>();

  ASSERT_FALSE(view.empty()) << "Should have restored FreeFrame";

  auto entity2 = *view.begin();

  ASSERT_TRUE(registry2.valid(entity2)) << "Restored entity should be valid";
  ASSERT_TRUE(
      registry2.all_of<dart::simulation::experimental::comps::FrameCache>(
          entity2))
      << "Restored entity should have FrameCache component";

  const auto& cache2
      = registry2.get<dart::simulation::experimental::comps::FrameCache>(
          entity2);

  // CRITICAL: Cache should be dirty after load (not serialized)
  EXPECT_TRUE(cache2.needTransformUpdate)
      << "Cache should be dirty after load - proves it was not serialized";

  // Verify state WAS serialized (parent should be entt::null)
  ASSERT_TRUE(
      registry2.all_of<dart::simulation::experimental::comps::FrameState>(
          entity2))
      << "Restored entity should have FrameState component";

  const auto& state2
      = registry2.get<dart::simulation::experimental::comps::FrameState>(
          entity2);
  EXPECT_FALSE(registry2.valid(state2.parentFrame))
      << "State should be serialized correctly (parent should be null)";

  // Verify properties WAS serialized (local transform should match)
  ASSERT_TRUE(
      registry2
          .all_of<dart::simulation::experimental::comps::FreeFrameProperties>(
              entity2))
      << "Restored entity should have FreeFrameProperties component";

  const auto& props2
      = registry2
            .get<dart::simulation::experimental::comps::FreeFrameProperties>(
                entity2);
  EXPECT_TRUE(props2.localTransform.isApprox(T1))
      << "Properties should be serialized correctly";
}

// Test state components ARE serialized
// TODO(serialization): Re-enable parent relationship testing once name-based
// entity reference system is implemented. Currently disabled because entity IDs
// change during deserialization, making entity references invalid.
TEST(Serialization, StateSerializedCorrectly)
{
  dart::simulation::experimental::World world;

  // Create parent and child frames
  auto parent = world.addFreeFrame("parent");
  auto child = world.addFreeFrame("child", parent);

  // Set transforms
  Eigen::Isometry3d T_parent = Eigen::Isometry3d::Identity();
  T_parent.translate(Eigen::Vector3d(5, 0, 0));
  parent.setLocalTransform(T_parent);

  Eigen::Isometry3d T_child = Eigen::Isometry3d::Identity();
  T_child.translate(Eigen::Vector3d(0, 10, 0));
  child.setLocalTransform(T_child);

  // Serialize
  std::stringstream ss;
  world.saveBinary(ss);

  // Deserialize
  dart::simulation::experimental::World world2;
  world2.loadBinary(ss);

  // Check that we have 2 FreeFrames
  auto& registry2 = world2.getRegistry();
  auto view
      = registry2.view<dart::simulation::experimental::comps::FreeFrameTag>();
  EXPECT_EQ(std::ranges::distance(view), 2)
      << "Should have restored 2 FreeFrames";

  // Verify that FrameState components exist
  for (auto entity : view) {
    // All FreeFrames should have FrameState component
    EXPECT_TRUE(
        registry2.all_of<dart::simulation::experimental::comps::FrameState>(
            entity))
        << "All FreeFrames should have FrameState after deserialization";
  }

  // Verify parent relationships are preserved after serialization
  // Count how many have valid parent references
  int framesWithParent = 0;
  int framesWithoutParent = 0;

  for (auto entity : view) {
    const auto& state
        = registry2.get<dart::simulation::experimental::comps::FrameState>(
            entity);
    if (registry2.valid(state.parentFrame)) {
      framesWithParent++;
    } else {
      framesWithoutParent++;
    }
  }

  // We should have 1 frame with parent (child) and 1 without (parent attached
  // to world)
  EXPECT_EQ(framesWithParent, 1)
      << "Should have 1 frame with parent relationship";
  EXPECT_EQ(framesWithoutParent, 1)
      << "Should have 1 frame without parent (attached to world)";
}

// Test properties ARE serialized
TEST(Serialization, PropertiesSerializedCorrectly)
{
  dart::simulation::experimental::World world;

  // Create FixedFrame with specific offset
  auto parent = world.addFreeFrame("parent");
  Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
  offset.translate(Eigen::Vector3d(3.14, 2.71, 1.41));
  offset.rotate(
      Eigen::AngleAxisd(
          dart::simulation::experimental::pi / 4, Eigen::Vector3d(1, 0, 0)));

  auto fixed = world.addFixedFrame("fixed", parent, offset);

  // Serialize
  std::stringstream ss;
  world.saveBinary(ss);

  // Deserialize
  dart::simulation::experimental::World world2;
  world2.loadBinary(ss);

  // Find the FixedFrame
  auto& registry2 = world2.getRegistry();
  auto view
      = registry2.view<dart::simulation::experimental::comps::FixedFrameTag>();
  EXPECT_FALSE(view.empty()) << "Should have restored FixedFrame";

  auto entity = *view.begin();
  const auto& props
      = registry2
            .get<dart::simulation::experimental::comps::FixedFrameProperties>(
                entity);

  // Properties should match exactly
  EXPECT_TRUE(props.localTransform.isApprox(offset, 1e-10))
      << "Properties (localTransform) should be serialized correctly";
}

// Test round-trip consistency (save twice, should be identical)
TEST(Serialization, RoundTripConsistency)
{
  dart::simulation::experimental::World world;

  auto parent = world.addFreeFrame("parent");
  Eigen::Isometry3d T_parent = Eigen::Isometry3d::Identity();
  T_parent.translate(Eigen::Vector3d(1, 2, 3));
  parent.setLocalTransform(T_parent);

  auto child = world.addFreeFrame("child", parent);
  Eigen::Isometry3d T_child = Eigen::Isometry3d::Identity();
  T_child.translate(Eigen::Vector3d(4, 5, 6));
  child.setLocalTransform(T_child);

  [[maybe_unused]] auto parentTransform = parent.getTransform();

  std::stringstream ss1;
  world.saveBinary(ss1);

  std::stringstream ss2;
  world.saveBinary(ss2);

  EXPECT_EQ(ss1.str(), ss2.str())
      << "Repeated saves should produce identical binary output";

  dart::simulation::experimental::World world2;
  world2.loadBinary(ss1);

  auto& registry2 = world2.getRegistry();
  auto view = registry2.view<
      dart::simulation::experimental::comps::Name,
      dart::simulation::experimental::comps::FreeFrameProperties,
      dart::simulation::experimental::comps::FrameState>();

  std::size_t frameCount = 0;
  for (auto entity : view) {
    const auto& name
        = view.get<dart::simulation::experimental::comps::Name>(entity).name;
    const auto& props
        = view.get<dart::simulation::experimental::comps::FreeFrameProperties>(
            entity);
    const auto& state
        = view.get<dart::simulation::experimental::comps::FrameState>(entity);

    if (name == "parent") {
      EXPECT_TRUE(props.localTransform.isApprox(T_parent));
      EXPECT_TRUE(state.parentFrame == entt::null);
      ++frameCount;
    } else if (name == "child") {
      EXPECT_TRUE(props.localTransform.isApprox(T_child));
      EXPECT_TRUE(state.parentFrame != entt::null);
      ++frameCount;
    }
  }

  EXPECT_EQ(frameCount, 2u);
}

//==============================================================================
// Cloning Tests (via serialization)
//==============================================================================

TEST(Serialization, CloneDeepCopy)
{
  dart::simulation::experimental::World world;

  auto parent = world.addFreeFrame("parent");
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translate(Eigen::Vector3d(1, 2, 3));
  parent.setLocalTransform(T);

  auto child = world.addFreeFrame("child", parent);
  Eigen::Isometry3d T_child = Eigen::Isometry3d::Identity();
  T_child.translate(Eigen::Vector3d(4, 5, 6));
  child.setLocalTransform(T_child);

  std::stringstream ss;
  world.saveBinary(ss);

  dart::simulation::experimental::World clone;
  clone.loadBinary(ss);

  auto& cloneReg = clone.getRegistry();
  auto cloneView = cloneReg.view<
      dart::simulation::experimental::comps::Name,
      dart::simulation::experimental::comps::FreeFrameProperties>();
  EXPECT_EQ(std::ranges::distance(cloneView), 2);

  for (auto entity : cloneView) {
    auto& props
        = cloneView
              .get<dart::simulation::experimental::comps::FreeFrameProperties>(
                  entity);
    const auto& name
        = cloneView.get<dart::simulation::experimental::comps::Name>(entity)
              .name;
    if (name == "parent") {
      EXPECT_TRUE(props.localTransform.isApprox(T));
    } else if (name == "child") {
      EXPECT_TRUE(props.localTransform.isApprox(T_child));
      props.localTransform.translation() = Eigen::Vector3d(100, 200, 300);
    }
  }

  const auto& originalReg = world.getRegistry();
  auto originalView = originalReg.view<
      dart::simulation::experimental::comps::Name,
      dart::simulation::experimental::comps::FreeFrameProperties>();
  for (auto entity : originalView) {
    const auto& props
        = originalView
              .get<dart::simulation::experimental::comps::FreeFrameProperties>(
                  entity);
    const auto& name
        = originalView.get<dart::simulation::experimental::comps::Name>(entity)
              .name;
    if (name == "child") {
      EXPECT_TRUE(props.localTransform.isApprox(T_child))
          << "Modifying clone should not affect original world";
    }
  }
}

TEST(Serialization, CloneResetCounters)
{
  dart::simulation::experimental::World world;
  auto frame1 = world.addFreeFrame(); // free_frame_001
  auto frame2 = world.addFreeFrame(); // free_frame_002
  (void)frame1;
  (void)frame2;
  [[maybe_unused]] auto mb1 = world.addMultibody(""); // multibody_001

  std::stringstream ss;
  world.saveBinary(ss);

  dart::simulation::experimental::World clone;
  clone.loadBinary(ss);

  auto nextFrame = clone.addFreeFrame();
  auto& cloneReg = clone.getRegistry();
  const auto& nextFrameName
      = cloneReg
            .get<dart::simulation::experimental::comps::Name>(
                nextFrame.getEntity())
            .name;
  EXPECT_EQ(nextFrameName, "free_frame_003");

  auto nextMb = clone.addMultibody("");
  EXPECT_EQ(nextMb.getName(), "multibody_002");
}

// ---------------------------------------------------------------------------
// Checkpoint / replay parity (PLAN-080 serialization/replay gate).
//
// The cases above prove that world *state* round-trips through
// saveBinary/loadBinary. These cases prove the stronger replay guarantee that
// checkpoint/restore and deterministic replay depend on: a world saved
// mid-trajectory and reloaded reproduces the original trajectory when stepped.
// ---------------------------------------------------------------------------

namespace {

// Builds a fixed-base two-link revolute chain that evolves deterministically:
// nonzero initial joint velocities integrate the joint positions every step
// (independent of link inertia), and gravity acts in the joint plane. The
// tests guard that the state actually changes so the parity check is not
// trivially satisfied by a static scene.
void buildReplayChain(dart::simulation::experimental::World& world)
{
  namespace sx = dart::simulation::experimental;
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));

  auto mb = world.addMultibody("chain");
  auto base = mb.addLink("base");
  auto link1 = mb.addLink(
      "link1",
      base,
      sx::JointSpec{
          .name = "joint1",
          .type = sx::JointType::Revolute,
          .axis = Eigen::Vector3d(0, 1, 0)});
  auto link2 = mb.addLink(
      "link2",
      link1,
      sx::JointSpec{
          .name = "joint2",
          .type = sx::JointType::Revolute,
          .axis = Eigen::Vector3d(0, 1, 0)});

  link1.getParentJoint().setPosition(Eigen::VectorXd::Constant(1, 0.3));
  link2.getParentJoint().setPosition(Eigen::VectorXd::Constant(1, -0.2));
  link1.getParentJoint().setVelocity(Eigen::VectorXd::Constant(1, 0.7));
  link2.getParentJoint().setVelocity(Eigen::VectorXd::Constant(1, -1.1));
}

// Gathers the chain's generalized state (joint positions and velocities) via
// the public handle API. getStateVector() is the differentiable rigid-body
// reduction (3 * dynamic rigid bodies) and is empty for a pure-multibody
// scene, so compare joint state instead.
Eigen::VectorXd chainState(dart::simulation::experimental::World& world)
{
  auto mb = world.getMultibody("chain").value();
  const auto j1 = mb.getJoint("joint1").value();
  const auto j2 = mb.getJoint("joint2").value();
  Eigen::VectorXd s(4);
  s << j1.getPosition()[0], j1.getVelocity()[0], j2.getPosition()[0],
      j2.getVelocity()[0];
  return s;
}

// A single dynamic rigid body that free-falls under gravity. getStateVector()
// is populated for dynamic rigid bodies (the differentiable [q; q̇] reduction),
// so the rigid-body integration path can be checked with the state vector
// directly, complementing the multibody joint-state path above.
void buildFallingBody(dart::simulation::experimental::World& world)
{
  namespace sx = dart::simulation::experimental;
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  sx::RigidBodyOptions options;
  options.mass = 2.0;
  options.position = Eigen::Vector3d(0.0, 0.0, 5.0);
  options.linearVelocity = Eigen::Vector3d(0.3, -0.2, 0.0);
  world.addRigidBody("ball", options);
}

// Deterministic CPU integration of identical state should agree to round-off;
// this tight tolerance guards against benign platform summation-order
// differences without hiding a real divergence.
constexpr double kReplayTol = 1e-10;

} // namespace

// A world checkpointed mid-trajectory and reloaded must continue identically to
// the still-running original instance.
TEST(Serialization, CheckpointReloadContinuesIdentically)
{
  namespace sx = dart::simulation::experimental;

  sx::World worldA;
  buildReplayChain(worldA);
  worldA.enterSimulationMode();

  const Eigen::VectorXd initial = chainState(worldA);
  worldA.step(25); // advance to a mid-trajectory checkpoint

  const Eigen::VectorXd checkpoint = chainState(worldA);
  EXPECT_GT((checkpoint - initial).norm(), 1e-6)
      << "Scene must evolve, otherwise the parity check is trivial";

  // Reload the checkpoint into a fresh world.
  std::stringstream ss;
  worldA.saveBinary(ss);
  sx::World worldB;
  worldB.loadBinary(ss);

  EXPECT_LT((chainState(worldB) - checkpoint).norm(), kReplayTol)
      << "Reloaded state must match the checkpoint";

  // Continue both worlds and require identical trajectories.
  worldA.step(25);
  worldB.step(25);

  EXPECT_LT((chainState(worldB) - chainState(worldA)).norm(), kReplayTol)
      << "Reloaded world must continue identically to the original instance";
  EXPECT_DOUBLE_EQ(worldB.getTime(), worldA.getTime());
}

// A checkpoint-and-reload mid-run must match an uninterrupted reference run:
// saving and loading does not perturb the trajectory.
TEST(Serialization, CheckpointReloadMatchesUninterruptedRun)
{
  namespace sx = dart::simulation::experimental;

  // Reference: run K + M steps without ever serializing.
  sx::World reference;
  buildReplayChain(reference);
  reference.enterSimulationMode();
  reference.step(40);
  const Eigen::VectorXd referenceState = chainState(reference);

  // Checkpointed: run K, serialize, reload into a fresh world, run M.
  sx::World original;
  buildReplayChain(original);
  original.enterSimulationMode();
  original.step(15);

  std::stringstream ss;
  original.saveBinary(ss);
  sx::World resumed;
  resumed.loadBinary(ss);
  resumed.step(25);

  EXPECT_LT((chainState(resumed) - referenceState).norm(), kReplayTol)
      << "Checkpoint+reload mid-run must match the uninterrupted reference run";
  EXPECT_DOUBLE_EQ(resumed.getTime(), reference.getTime());
}

// Same replay guarantee for the rigid-body integration path, checked through
// the world state vector [q; q̇] which is populated for dynamic rigid bodies.
TEST(Serialization, RigidBodyCheckpointReloadContinuesIdentically)
{
  namespace sx = dart::simulation::experimental;

  sx::World worldA;
  buildFallingBody(worldA);
  worldA.enterSimulationMode();

  const Eigen::VectorXd initial = worldA.getStateVector();
  ASSERT_EQ(initial.size(), 6) // [pos(3); linVel(3)] for one dynamic body
      << "One dynamic rigid body should expose a size-6 state vector";
  worldA.step(25); // advance to a mid-trajectory checkpoint

  const Eigen::VectorXd checkpoint = worldA.getStateVector();
  EXPECT_GT((checkpoint - initial).norm(), 1e-6)
      << "Falling body must evolve under gravity";

  std::stringstream ss;
  worldA.saveBinary(ss);
  sx::World worldB;
  worldB.loadBinary(ss);

  ASSERT_EQ(worldB.getStateVector().size(), checkpoint.size());
  EXPECT_LT((worldB.getStateVector() - checkpoint).norm(), kReplayTol)
      << "Reloaded rigid-body state must match the checkpoint";

  worldA.step(25);
  worldB.step(25);

  EXPECT_LT(
      (worldB.getStateVector() - worldA.getStateVector()).norm(), kReplayTol)
      << "Reloaded world must continue identically for the rigid-body path";
  EXPECT_DOUBLE_EQ(worldB.getTime(), worldA.getTime());
}
