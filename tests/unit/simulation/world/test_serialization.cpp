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

#include <dart/simulation/body/contact.hpp>
#include <dart/simulation/body/deformable_body.hpp>
#include <dart/simulation/body/rigid_body.hpp>
#include <dart/simulation/common/constants.hpp>
#include <dart/simulation/comps/contact_material.hpp>
#include <dart/simulation/comps/dynamics.hpp>
#include <dart/simulation/comps/frame_types.hpp>
#include <dart/simulation/comps/joint.hpp>
#include <dart/simulation/comps/link.hpp>
#include <dart/simulation/comps/multibody.hpp>
#include <dart/simulation/comps/name.hpp>
#include <dart/simulation/compute/variational_integration.hpp>
#include <dart/simulation/constraint/loop_closure_spec.hpp>
#include <dart/simulation/detail/entity_conversion.hpp>
#include <dart/simulation/detail/rigid_avbd/rigid_world_contact.hpp>
#include <dart/simulation/detail/world_registry_access.hpp>
#include <dart/simulation/frame/fixed_frame.hpp>
#include <dart/simulation/frame/frame.hpp>
#include <dart/simulation/frame/free_frame.hpp>
#include <dart/simulation/io/binary_io.hpp>
#include <dart/simulation/io/serializer.hpp>
#include <dart/simulation/multibody/joint.hpp>
#include <dart/simulation/multibody/link.hpp>
#include <dart/simulation/multibody/multibody.hpp>
#include <dart/simulation/world.hpp>
#include <dart/simulation/world_options.hpp>

#include <dart/common/memory_allocator.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <iterator>
#include <map>
#include <new>
#include <ranges>
#include <set>
#include <sstream>
#include <string>
#include <string_view>
#include <typeinfo>
#include <vector>

#include <cstdint>
#include <cstring>

//==============================================================================
// Serialization Tests - Comprehensive Coverage
//==============================================================================

namespace {

class CountingMemoryAllocator final : public dart::common::MemoryAllocator
{
public:
  [[nodiscard]] std::string_view getType() const override
  {
    return "CountingMemoryAllocator";
  }

  [[nodiscard]] void* allocate(std::size_t bytes) noexcept override
  {
    if (bytes == 0) {
      return nullptr;
    }

    ++allocationCount;
    return ::operator new(bytes, std::nothrow);
  }

  [[nodiscard]] void* allocate(
      std::size_t bytes, std::size_t alignment) noexcept override
  {
    if (bytes == 0 || alignment == 0 || (alignment & (alignment - 1)) != 0) {
      return nullptr;
    }

    ++allocationCount;
    ++alignedAllocationCount;
    if (alignment <= __STDCPP_DEFAULT_NEW_ALIGNMENT__) {
      return ::operator new(bytes, std::nothrow);
    }

    return ::operator new(bytes, std::align_val_t(alignment), std::nothrow);
  }

  void deallocate(void* pointer, std::size_t /*bytes*/) override
  {
    ++deallocationCount;
    ::operator delete(pointer);
  }

  void deallocate(
      void* pointer, std::size_t /*bytes*/, std::size_t alignment) override
  {
    ++deallocationCount;
    ++alignedDeallocationCount;
    if (alignment <= __STDCPP_DEFAULT_NEW_ALIGNMENT__) {
      ::operator delete(pointer);
      return;
    }

    ::operator delete(pointer, std::align_val_t(alignment));
  }

  std::size_t allocationCount{0};
  std::size_t alignedAllocationCount{0};
  std::size_t deallocationCount{0};
  std::size_t alignedDeallocationCount{0};
};

struct VersionedLoadFallbackComponent
{
  std::uint32_t value{0};
};

class LegacyLoadFallbackSerializer final
  : public dart::simulation::io::ComponentSerializer
{
public:
  [[nodiscard]] std::string_view getTypeName() const override
  {
    return "test::VersionedLoadFallbackComponent";
  }

  void save(
      std::ostream&,
      entt::entity,
      const dart::simulation::detail::WorldRegistry&,
      const dart::simulation::io::EntityMap&) const override
  {
  }

  void load(
      std::istream&,
      entt::entity entity,
      dart::simulation::detail::WorldRegistry& registry) const override
  {
    registry.emplace<VersionedLoadFallbackComponent>(entity, 123u);
  }

  [[nodiscard]] bool hasComponent(
      entt::entity entity,
      const dart::simulation::detail::WorldRegistry& registry) const override
  {
    return registry.all_of<VersionedLoadFallbackComponent>(entity);
  }
};

using RegistryStorageCapacities = std::map<entt::id_type, std::size_t>;

RegistryStorageCapacities registryStorageCapacities(
    const dart::simulation::detail::WorldRegistry& registry)
{
  RegistryStorageCapacities capacities;
  const auto* entityStorage = registry.storage<entt::entity>();
  EXPECT_NE(entityStorage, nullptr);
  capacities.emplace(
      entt::type_hash<entt::entity>::value(),
      entityStorage == nullptr ? 0u : entityStorage->capacity());
  for (auto&& [id, storage] : registry.storage()) {
    capacities.emplace(id, storage.capacity());
  }
  return capacities;
}

void expectRegistryStorageCapacitiesUnchanged(
    const RegistryStorageCapacities& expected,
    const dart::simulation::detail::WorldRegistry& registry)
{
  const auto actual = registryStorageCapacities(registry);
  EXPECT_EQ(actual.size(), expected.size());

  for (const auto& [id, capacity] : actual) {
    if (!expected.contains(id)) {
      ADD_FAILURE() << "unexpected storage id " << id << " capacity "
                    << capacity;
    }
  }

  for (const auto& [id, capacity] : expected) {
    const auto it = actual.find(id);
    ASSERT_NE(it, actual.end()) << "missing storage id " << id;
    EXPECT_EQ(it->second, capacity) << "storage id " << id;
  }
}

bool hasContactBetween(
    const std::vector<dart::simulation::Contact>& contacts,
    std::string_view first,
    std::string_view second)
{
  for (const auto& contact : contacts) {
    const auto nameA = contact.bodyA.getName();
    const auto nameB = contact.bodyB.getName();
    if ((nameA == first && nameB == second)
        || (nameA == second && nameB == first)) {
      return true;
    }
  }
  return false;
}

template <typename AddJoint>
void expectBrokenRigidBodyJointRoundTrips(
    std::string_view jointName,
    dart::simulation::JointType expectedType,
    std::size_t expectedDofCount,
    bool hasExpectedAxis,
    const Eigen::Vector3d& expectedAxis,
    AddJoint addJoint)
{
  namespace sx = dart::simulation;

  sx::World world1;
  world1.setGravity(Eigen::Vector3d::Zero());

  sx::RigidBodyOptions parentOptions;
  parentOptions.isStatic = true;
  auto parent = world1.addRigidBody("parent", parentOptions);
  parent.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  sx::RigidBodyOptions childOptions;
  childOptions.position = Eigen::Vector3d(0.4, 0.0, 0.0);
  auto child = world1.addRigidBody("child", childOptions);
  child.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  auto joint = addJoint(world1, jointName, parent, child);
  joint.setBreakForce(12.5);

  auto& jointComp
      = dart::simulation::detail::registryOf(world1).get<sx::comps::JointState>(
          dart::simulation::detail::toRegistryEntity(joint.getEntity()));
  jointComp.broken = true;
  world1.enterSimulationMode();

  std::stringstream ss;
  world1.saveBinary(ss);

  sx::World world2;
  world2.loadBinary(ss);

  EXPECT_TRUE(world2.isSimulationMode());
  EXPECT_EQ(world2.getRigidBodyJointCount(), 1u);
  auto restoredJoint = world2.getRigidBodyJoint(jointName);
  ASSERT_TRUE(restoredJoint.has_value());
  EXPECT_EQ(restoredJoint->getType(), expectedType);
  EXPECT_EQ(restoredJoint->getDOFCount(), expectedDofCount);
  if (hasExpectedAxis) {
    EXPECT_TRUE(restoredJoint->getAxis().isApprox(expectedAxis));
  }
  EXPECT_DOUBLE_EQ(restoredJoint->getBreakForce(), 12.5);
  EXPECT_TRUE(restoredJoint->isBroken());
  EXPECT_EQ(restoredJoint->getParentRigidBody().getName(), "parent");
  EXPECT_EQ(restoredJoint->getChildRigidBody().getName(), "child");
  EXPECT_TRUE(hasContactBetween(world2.collide(), "parent", "child"));

  restoredJoint->resetBreakage();
  EXPECT_FALSE(restoredJoint->isBroken());
  EXPECT_FALSE(hasContactBetween(world2.collide(), "parent", "child"));
}

template <typename AddJoint>
void expectBrokenArticulatedPointJointRoundTrips(
    std::string_view jointName,
    dart::simulation::JointType expectedType,
    std::size_t expectedDofCount,
    bool hasExpectedAxis,
    const Eigen::Vector3d& expectedAxis,
    bool hasParentLink,
    AddJoint addJoint)
{
  namespace sx = dart::simulation;

  sx::World world1;
  world1.setGravity(Eigen::Vector3d::Zero());
  world1.setMultibodyOptions({sx::MultibodyIntegrationFamily::Variational});

  auto robot = world1.addMultibody("robot");
  auto base = robot.addLink("base");
  sx::JointSpec spec;
  spec.name = "floating";
  spec.type = sx::JointType::Floating;
  spec.transformFromParent.translation() = Eigen::Vector3d(0.4, 0.2, 0.1);
  auto child = robot.addLink("child", base, spec);

  const Eigen::Vector3d childAnchor(-0.1, 0.05, 0.02);
  const Eigen::Vector3d parentOrWorldAnchor
      = child.getWorldTransform() * childAnchor;
  auto joint = addJoint(
      world1, jointName, base, child, parentOrWorldAnchor, childAnchor);
  joint.setBreakForce(12.5);
  if (expectedDofCount == 1u) {
    joint.setActuatorType(sx::ActuatorType::Velocity);
    joint.setCommandVelocity(Eigen::VectorXd::Constant(1, 0.25));
    joint.setEffortLimits(
        Eigen::VectorXd::Constant(1, -3.0), Eigen::VectorXd::Constant(1, 4.0));
  }

  world1.enterSimulationMode();
  auto& jointComp
      = dart::simulation::detail::registryOf(world1).get<sx::comps::JointState>(
          dart::simulation::detail::toRegistryEntity(joint.getEntity()));
  jointComp.broken = true;

  std::stringstream ss;
  world1.saveBinary(ss);

  sx::World world2;
  world2.loadBinary(ss);

  EXPECT_TRUE(world2.isSimulationMode());
  EXPECT_EQ(
      world2.getMultibodyOptions().integrationFamily,
      sx::MultibodyIntegrationFamily::Variational);
  EXPECT_EQ(world2.getArticulatedJointCount(), 1u);
  auto restoredJoint = world2.getArticulatedJoint(jointName);
  ASSERT_TRUE(restoredJoint.has_value());
  EXPECT_EQ(restoredJoint->getType(), expectedType);
  EXPECT_EQ(restoredJoint->getDOFCount(), expectedDofCount);
  if (hasExpectedAxis) {
    EXPECT_TRUE(restoredJoint->getAxis().isApprox(expectedAxis));
  }
  EXPECT_DOUBLE_EQ(restoredJoint->getBreakForce(), 12.5);
  EXPECT_TRUE(restoredJoint->isBroken());
  EXPECT_EQ(restoredJoint->getChildLink().getName(), "child");
  if (hasParentLink) {
    EXPECT_EQ(restoredJoint->getParentLink().getName(), "base");
  } else {
    EXPECT_THROW(
        {
          auto parent = restoredJoint->getParentLink();
          (void)parent;
        },
        sx::InvalidArgumentException);
  }
  if (expectedDofCount == 1u) {
    EXPECT_EQ(restoredJoint->getActuatorType(), sx::ActuatorType::Velocity);
    EXPECT_TRUE(restoredJoint->getCommandVelocity().isApprox(
        Eigen::VectorXd::Constant(1, 0.25)));
    EXPECT_TRUE(restoredJoint->getEffortLowerLimits().isApprox(
        Eigen::VectorXd::Constant(1, -3.0)));
    EXPECT_TRUE(restoredJoint->getEffortUpperLimits().isApprox(
        Eigen::VectorXd::Constant(1, 4.0)));
  }

  restoredJoint->resetBreakage();
  EXPECT_FALSE(restoredJoint->isBroken());
}

void saveLegacyWorldWithCurrentEntities(
    std::ostream& output,
    const dart::simulation::World& world,
    std::uint32_t legacyVersion)
{
  namespace comps = dart::simulation::comps;
  namespace io = dart::simulation::io;

  constexpr std::uint32_t magicNumber = 0x44525437;
  io::writePOD(output, magicNumber);
  io::writePOD(output, legacyVersion);

  const auto& registry = dart::simulation::detail::registryOf(world);
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
      serializers.at(typeName)->save(output, entity, registry, entityMap);
    }
  }
}

void writeLegacyV15EmptyWorldWithSolverOptions(std::ostream& output)
{
  namespace io = dart::simulation::io;

  constexpr std::uint32_t magicNumber = 0x44525437;
  io::writePOD(output, magicNumber);
  io::writePOD(output, 15u);

  const std::size_t entityCount = 0u;
  io::writePOD(output, entityCount);

  const std::uint8_t simulationFlag = 0u;
  io::writePOD(output, simulationFlag);
  const std::size_t counter = 0u;
  io::writePOD(output, counter); // free frames
  io::writePOD(output, counter); // fixed frames
  io::writePOD(output, counter); // multibodies
  io::writePOD(output, counter); // rigid bodies
  io::writePOD(output, counter); // links
  io::writePOD(output, counter); // joints

  io::writePOD(output, 0.125); // time step
  io::writePOD(output, 2.5);   // time
  io::writePOD(output, counter);

  io::writePOD(output, 0.0);
  io::writePOD(output, -1.5);
  io::writePOD(output, -3.0);

  io::writePOD(output, counter); // deformable-body counter

  const std::uint8_t differentiableFlag = 1u;
  io::writePOD(output, differentiableFlag);

  const std::uint8_t rigidBodySolver = 1u;
  const std::uint8_t contactSolverMethod = 1u;
  const std::uint8_t contactGradientMode = 2u;
  const std::uint8_t multibodyIntegrationMethod = 1u;
  io::writePOD(output, rigidBodySolver);
  io::writePOD(output, contactSolverMethod);
  io::writePOD(output, contactGradientMode);
  io::writePOD(output, multibodyIntegrationMethod);
}

} // namespace

// Test save/load empty world
TEST(Serialization, EmptyWorld)
{
  dart::simulation::World world1;

  // Save to stream
  std::stringstream ss;
  world1.saveBinary(ss);
  // Load into new world
  dart::simulation::World world2;
  world2.loadBinary(ss);

  // Verify empty
  EXPECT_EQ(world2.getMultibodyCount(), 0);
  EXPECT_EQ(world2.getRigidBodyCount(), 0);
  EXPECT_FALSE(world2.isSimulationMode());
}

TEST(Serialization, IgnoredCollisionPairsRoundTrip)
{
  namespace sx = dart::simulation;

  sx::World world1;
  auto bodyA = world1.addRigidBody("rigid_a");
  bodyA.setCollisionShape(sx::CollisionShape::makeSphere(0.5));
  sx::RigidBodyOptions bodyBOptions;
  bodyBOptions.position = Eigen::Vector3d(0.4, 0.0, 0.0);
  auto bodyB = world1.addRigidBody("rigid_b", bodyBOptions);
  bodyB.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  ASSERT_FALSE(world1.collide().empty());
  world1.setCollisionPairIgnored(bodyA, bodyB);
  ASSERT_TRUE(world1.collide().empty());

  std::stringstream ss;
  world1.saveBinary(ss);

  sx::World world2;
  world2.loadBinary(ss);

  auto restoredA = world2.getRigidBody("rigid_a");
  auto restoredB = world2.getRigidBody("rigid_b");
  ASSERT_TRUE(restoredA.has_value());
  ASSERT_TRUE(restoredB.has_value());
  EXPECT_EQ(world2.getIgnoredCollisionPairCount(), 1u);
  EXPECT_TRUE(world2.isCollisionPairIgnored(*restoredB, *restoredA));
  EXPECT_TRUE(world2.collide().empty());

  world2.setCollisionPairIgnored(*restoredA, *restoredB, false);
  EXPECT_FALSE(world2.collide().empty());
}

TEST(Serialization, ComponentSerializerVersionedLoadFallsBackToLegacyLoad)
{
  namespace sx = dart::simulation;

  sx::World world;
  auto& registry = sx::detail::registryOf(world);
  const auto entity = registry.create();
  std::stringstream input;

  LegacyLoadFallbackSerializer serializer;
  const sx::io::ComponentSerializer& baseSerializer = serializer;
  baseSerializer.load(input, entity, registry, 99u);

  ASSERT_TRUE(registry.all_of<VersionedLoadFallbackComponent>(entity));
  EXPECT_EQ(registry.get<VersionedLoadFallbackComponent>(entity).value, 123u);
}

// Test deformable custom serializers are restored after registry reset.
TEST(Serialization, DeformableSerializersAreRegisteredAfterRegistryClear)
{
  namespace sx = dart::simulation;

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
  dart::simulation::World world1;
  [[maybe_unused]] auto mb1 = world1.addMultibody("robot1");

  std::stringstream ss;
  world1.saveBinary(ss);

  dart::simulation::World world2;
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
  dart::simulation::World world1;
  auto mb = world1.addMultibody("robot");
  [[maybe_unused]] auto base = mb.addLink("base");

  std::stringstream ss;
  world1.saveBinary(ss);

  dart::simulation::World world2;
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
  dart::simulation::World world1;
  auto mb = world1.addMultibody("robot");
  auto base = mb.addLink("base");
  [[maybe_unused]] auto link1 = mb.addLink(
      "link1",
      {.parentLink = base,
       .jointName = "joint1",
       .jointType = dart::simulation::JointType::Revolute,
       .axis = {0, 0, 1}});

  std::stringstream ss;
  world1.saveBinary(ss);

  dart::simulation::World world2;
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
  EXPECT_EQ(joint1_restored->getType(), dart::simulation::JointType::Revolute);

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
  namespace sx = dart::simulation;

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

// Test save/load preserves names
TEST(Serialization, PreservesNames)
{
  dart::simulation::World world1;
  auto mb = world1.addMultibody("test_robot");
  [[maybe_unused]] auto base = mb.addLink("base_link");
  [[maybe_unused]] auto link = mb.addLink(
      "arm_link", {.parentLink = base, .jointName = "shoulder_joint"});

  std::stringstream ss;
  world1.saveBinary(ss);

  dart::simulation::World world2;
  world2.loadBinary(ss);

  auto mb_restored = world2.getMultibody("test_robot");
  ASSERT_TRUE(mb_restored.has_value());
  EXPECT_EQ(mb_restored->getName(), "test_robot");
}

// Test save/load with rigid bodies
TEST(Serialization, WithRigidBodies)
{
  dart::simulation::World world1;
  [[maybe_unused]] auto rb1 = world1.addRigidBody("box1");
  [[maybe_unused]] auto rb2 = world1.addRigidBody("box2");

  std::stringstream ss;
  world1.saveBinary(ss);

  dart::simulation::World world2;
  world2.loadBinary(ss);

  EXPECT_EQ(world2.getRigidBodyCount(), 2);
  EXPECT_TRUE(world2.hasRigidBody("box1"));
  EXPECT_TRUE(world2.hasRigidBody("box2"));
}

TEST(Serialization, PreservesRigidBodyCollisionComponents)
{
  namespace sx = dart::simulation;

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
  namespace sx = dart::simulation;

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
  dart::simulation::World world1;
  world1.addMultibody("robot");
  world1.enterSimulationMode();
  ASSERT_TRUE(world1.isSimulationMode());

  std::stringstream ss;
  world1.saveBinary(ss);

  dart::simulation::World world2;
  world2.loadBinary(ss);

  EXPECT_TRUE(world2.isSimulationMode());
}

// Test save/load preserves world timing metadata
TEST(Serialization, PreservesWorldTimingMetadata)
{
  dart::simulation::World world1;
  world1.setTimeStep(0.125);
  world1.setTime(2.0);
  world1.setGravity(Eigen::Vector3d(0.0, -1.5, -3.0));
  world1.enterSimulationMode();
  world1.step();

  std::stringstream ss;
  world1.saveBinary(ss);

  dart::simulation::World world2;
  world2.loadBinary(ss);

  EXPECT_TRUE(world2.isSimulationMode());
  EXPECT_DOUBLE_EQ(world2.getTimeStep(), 0.125);
  EXPECT_DOUBLE_EQ(world2.getTime(), 2.125);
  EXPECT_EQ(world2.getFrame(), 1u);
  EXPECT_TRUE(world2.getGravity().isApprox(Eigen::Vector3d(0.0, -1.5, -3.0)));
}

// Test save/load preserves World-level solver-family and policy metadata.
TEST(Serialization, PreservesWorldSolverOptions)
{
  namespace sx = dart::simulation;

  sx::WorldOptions options;
  options.rigidBodySolver = sx::RigidBodySolver::Ipc;
  options.multibodyOptions.integrationFamily
      = sx::MultibodyIntegrationFamily::Variational;
  options.multibodyOptions.variationalMaxIterations = 200;
  options.multibodyOptions.variationalTolerance = 1e-9;
  options.differentiable = true;
  options.contactSolverMethod = sx::ContactSolverMethod::BoxedLcp;
  options.contactGradientMode = sx::ContactGradientMode::PreContactSurrogate;

  sx::World world1(options);

  std::stringstream ss;
  world1.saveBinary(ss);

  sx::World world2;
  world2.loadBinary(ss);

  EXPECT_EQ(world2.getRigidBodySolver(), sx::RigidBodySolver::Ipc);
  EXPECT_EQ(
      world2.getMultibodyOptions().integrationFamily,
      sx::MultibodyIntegrationFamily::Variational);
  EXPECT_EQ(world2.getMultibodyOptions().variationalMaxIterations, 200u);
  EXPECT_DOUBLE_EQ(world2.getMultibodyOptions().variationalTolerance, 1e-9);
  EXPECT_TRUE(world2.isDifferentiable());
  EXPECT_EQ(world2.getContactSolverMethod(), sx::ContactSolverMethod::BoxedLcp);
  EXPECT_EQ(
      world2.getContactGradientMode(),
      sx::ContactGradientMode::PreContactSurrogate);
}

TEST(Serialization, PreservesComplementarityAwareContactGradientMode)
{
  namespace sx = dart::simulation;

  sx::WorldOptions options;
  options.contactGradientMode = sx::ContactGradientMode::ComplementarityAware;
  sx::World world1(options);

  std::stringstream ss;
  world1.saveBinary(ss);

  sx::World world2;
  world2.loadBinary(ss);

  EXPECT_EQ(
      world2.getContactGradientMode(),
      sx::ContactGradientMode::ComplementarityAware);
}

TEST(Serialization, LegacyV15WorldSolverOptionsLoadBeforeIgnoredPairs)
{
  namespace sx = dart::simulation;

  std::stringstream legacy;
  writeLegacyV15EmptyWorldWithSolverOptions(legacy);

  sx::World world;
  world.loadBinary(legacy);

  EXPECT_FALSE(world.isSimulationMode());
  EXPECT_DOUBLE_EQ(world.getTimeStep(), 0.125);
  EXPECT_DOUBLE_EQ(world.getTime(), 2.5);
  EXPECT_TRUE(world.getGravity().isApprox(Eigen::Vector3d(0.0, -1.5, -3.0)));
  EXPECT_TRUE(world.isDifferentiable());
  EXPECT_EQ(world.getRigidBodySolver(), sx::RigidBodySolver::Ipc);
  EXPECT_EQ(
      world.getMultibodyOptions().integrationFamily,
      sx::MultibodyIntegrationFamily::Variational);
  EXPECT_EQ(world.getMultibodyOptions().variationalMaxIterations, 100u);
  EXPECT_DOUBLE_EQ(world.getMultibodyOptions().variationalTolerance, 1e-10);
  EXPECT_EQ(world.getContactSolverMethod(), sx::ContactSolverMethod::BoxedLcp);
  EXPECT_EQ(
      world.getContactGradientMode(),
      sx::ContactGradientMode::PreContactSurrogate);
  EXPECT_EQ(world.getIgnoredCollisionPairCount(), 0u);
}

TEST(Serialization, RejectsInvalidWorldSolverOptionTail)
{
  namespace sx = dart::simulation;

  sx::World world;
  std::stringstream ss;
  world.saveBinary(ss);
  const auto validRecord = ss.str();
  ASSERT_GE(validRecord.size(), 4u);

  const auto expectInvalidByte = [&](std::size_t offsetFromEnd) {
    auto corruptRecord = validRecord;
    corruptRecord[corruptRecord.size() - offsetFromEnd] = static_cast<char>(99);

    std::stringstream input(corruptRecord);
    sx::World loaded;
    EXPECT_THROW(loaded.loadBinary(input), sx::InvalidArgumentException);
  };

  const std::size_t solverSuffixBytes
      = sizeof(std::size_t) + sizeof(std::size_t) + sizeof(double);
  ASSERT_GE(validRecord.size(), solverSuffixBytes + 4u);
  expectInvalidByte(solverSuffixBytes + 4u); // rigid-body solver
  expectInvalidByte(solverSuffixBytes + 3u); // contact solver method
  expectInvalidByte(solverSuffixBytes + 2u); // contact gradient mode
  expectInvalidByte(solverSuffixBytes + 1u); // multibody integration method

  const auto expectInvalidTailField
      = [&](std::size_t offsetFromStart, const void* value, std::size_t size) {
          auto corruptRecord = validRecord;
          std::memcpy(corruptRecord.data() + offsetFromStart, value, size);

          std::stringstream input(corruptRecord);
          sx::World loaded;
          EXPECT_THROW(loaded.loadBinary(input), sx::InvalidArgumentException);
        };

  const std::size_t toleranceOffset = validRecord.size() - sizeof(double);
  const std::size_t iterationOffset = toleranceOffset - sizeof(std::size_t);
  const std::size_t invalidIterations = 0u;
  expectInvalidTailField(
      iterationOffset, &invalidIterations, sizeof(invalidIterations));
  const double invalidTolerance = 0.0;
  expectInvalidTailField(
      toleranceOffset, &invalidTolerance, sizeof(invalidTolerance));
}

// Test loadBinary resets solver-family and policy metadata when reading records
// that predate the versioned World-level solver-option tail.
TEST(Serialization, LegacyLoadResetsMissingWorldSolverOptionsToDefaults)
{
  namespace sx = dart::simulation;

  sx::World legacySource;
  std::stringstream legacy;
  saveLegacyWorldWithCurrentEntities(
      legacy, legacySource, /*legacyVersion=*/14u);

  sx::WorldOptions options;
  options.rigidBodySolver = sx::RigidBodySolver::Ipc;
  options.multibodyOptions.integrationFamily
      = sx::MultibodyIntegrationFamily::Variational;
  options.multibodyOptions.variationalMaxIterations = 200;
  options.multibodyOptions.variationalTolerance = 1e-9;
  options.differentiable = true;
  options.contactSolverMethod = sx::ContactSolverMethod::BoxedLcp;
  options.contactGradientMode = sx::ContactGradientMode::PreContactSurrogate;
  sx::World loaded(options);

  loaded.loadBinary(legacy);

  EXPECT_EQ(
      loaded.getRigidBodySolver(), sx::RigidBodySolver::SequentialImpulse);
  EXPECT_EQ(
      loaded.getMultibodyOptions().integrationFamily,
      sx::MultibodyIntegrationFamily::SemiImplicit);
  EXPECT_EQ(loaded.getMultibodyOptions().variationalMaxIterations, 100u);
  EXPECT_DOUBLE_EQ(loaded.getMultibodyOptions().variationalTolerance, 1e-10);
  EXPECT_FALSE(loaded.isDifferentiable());
  EXPECT_EQ(
      loaded.getContactSolverMethod(),
      sx::ContactSolverMethod::SequentialImpulse);
  EXPECT_EQ(loaded.getContactGradientMode(), sx::ContactGradientMode::Analytic);
}

// Test that legacy world records without versioned gravity metadata leave
// trailing stream bytes untouched instead of consuming them as gravity.
TEST(Serialization, LegacyWorldMetadataDoesNotConsumeTrailingBytesAsGravity)
{
  dart::simulation::World world1;
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
  // Format v6+ appends World option metadata after the deformable-body counter.
  // Drop the current modern option tail, then erase only the v2+ gravity block
  // so this legacy-compatibility check still leaves the v1 deformable counter
  // in place before the synthetic trailer.
  constexpr std::size_t modernWorldOptionTailBytes
      = sizeof(std::uint8_t) + 4u * sizeof(std::uint8_t) + sizeof(std::size_t)
        + sizeof(std::size_t) + sizeof(double);
  constexpr std::size_t legacyDeformableCounterBytes = sizeof(std::size_t);
  ASSERT_GE(
      legacyRecord.size(),
      modernWorldOptionTailBytes + legacyDeformableCounterBytes
          + 3u * sizeof(double));
  legacyRecord.resize(legacyRecord.size() - modernWorldOptionTailBytes);
  const std::size_t gravityOffset = legacyRecord.size()
                                    - legacyDeformableCounterBytes
                                    - 3u * sizeof(double);
  legacyRecord.erase(gravityOffset, 3u * sizeof(double));

  constexpr std::string_view trailerBytes = "0123456789abcdefghijklmn";
  static_assert(trailerBytes.size() == 3 * sizeof(double));
  const std::string trailer(trailerBytes);
  legacyRecord += trailer;

  std::stringstream input(legacyRecord);
  dart::simulation::World world2;
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
  dart::simulation::World world1;
  [[maybe_unused]] auto mb1 = world1.addMultibody("");
  [[maybe_unused]] auto mb2 = world1.addMultibody("");

  std::stringstream ss;
  world1.saveBinary(ss);

  dart::simulation::World world2;
  world2.loadBinary(ss);

  auto mb3 = world2.addMultibody("");
  EXPECT_EQ(mb3.getName(), "multibody_003");
}

TEST(Serialization, PreservesJointCounterAcrossArticulatedGeneratedNames)
{
  namespace sx = dart::simulation;

  sx::World world1;
  world1.setMultibodyOptions({sx::MultibodyIntegrationFamily::Variational});

  auto robot = world1.addMultibody("robot");
  auto base = robot.addLink("base");
  sx::JointSpec spec;
  spec.name = "tree";
  spec.type = sx::JointType::Floating;
  spec.transformFromParent.translation() = Eigen::Vector3d::UnitX();
  auto child = robot.addLink("child", base, spec);

  auto explicitJoint
      = world1.addArticulatedFixedJoint("joint_001", base, child);
  auto generatedJoint
      = world1.addArticulatedRevoluteJoint("", child, Eigen::Vector3d::UnitY());
  EXPECT_EQ(explicitJoint.getName(), "joint_001");
  EXPECT_EQ(generatedJoint.getName(), "joint_002");

  std::stringstream ss;
  world1.saveBinary(ss);

  sx::World world2;
  world2.loadBinary(ss);

  auto restoredRobot = world2.getMultibody("robot");
  ASSERT_TRUE(restoredRobot.has_value());
  auto restoredBase = restoredRobot->getLink("base");
  ASSERT_TRUE(restoredBase.has_value());
  auto restoredChild = restoredRobot->getLink("child");
  ASSERT_TRUE(restoredChild.has_value());

  EXPECT_TRUE(world2.hasArticulatedJoint("joint_001"));
  EXPECT_TRUE(world2.hasArticulatedJoint("joint_002"));
  EXPECT_EQ(world2.getArticulatedJointCount(), 2u);

  // The topology joint is also a restored Joint component, so the load-time
  // counter guard advances the next generated public facade past it.
  auto afterLoadGenerated = world2.addArticulatedPrismaticJoint(
      "", *restoredBase, *restoredChild, Eigen::Vector3d::UnitX());
  EXPECT_EQ(afterLoadGenerated.getName(), "joint_004");
  EXPECT_EQ(world2.getArticulatedJointCount(), 3u);
}

// Test multiple save/load cycles
TEST(Serialization, MultipleCycles)
{
  dart::simulation::World world1;
  auto mb = world1.addMultibody("robot");
  mb.addLink("base");

  for (int i = 0; i < 3; ++i) {
    std::stringstream ss;
    world1.saveBinary(ss);

    dart::simulation::World world2;
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
  dart::simulation::World world1;
  world1.addMultibody("robot1");

  std::stringstream ss;
  world1.saveBinary(ss);

  dart::simulation::World world2;
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
  dart::simulation::World world1;
  auto mb = world1.addMultibody("robot");
  [[maybe_unused]] auto base = mb.addLink("base");
  [[maybe_unused]] auto link1 = mb.addLink(
      "link1",
      {.parentLink = base,
       .jointName = "j1",
       .jointType = dart::simulation::JointType::Revolute,
       .axis = {0, 0, 1}});

  std::stringstream ss;
  world1.saveBinary(ss);

  dart::simulation::World world2;
  world2.loadBinary(ss);

  [[maybe_unused]] auto mb2 = world2.getMultibody("robot");
  ASSERT_TRUE(mb2.has_value());
  EXPECT_EQ(mb2->getJointCount(), 1);
}

// Test saves and restores joint type (PRISMATIC)
TEST(Serialization, JointTypePrismatic)
{
  dart::simulation::World world1;
  auto mb = world1.addMultibody("robot");
  [[maybe_unused]] auto base = mb.addLink("base");
  [[maybe_unused]] auto link1 = mb.addLink(
      "link1",
      {.parentLink = base,
       .jointName = "j1",
       .jointType = dart::simulation::JointType::Prismatic,
       .axis = {1, 0, 0}});

  std::stringstream ss;
  world1.saveBinary(ss);

  dart::simulation::World world2;
  world2.loadBinary(ss);

  [[maybe_unused]] auto mb2 = world2.getMultibody("robot");
  ASSERT_TRUE(mb2.has_value());
  EXPECT_EQ(mb2->getJointCount(), 1);
}

// Test complex hierarchy (6-DOF manipulator)
TEST(Serialization, ComplexHierarchy)
{
  dart::simulation::World world1;
  auto robot = world1.addMultibody("ur5");
  [[maybe_unused]] auto base = robot.addLink("base");
  [[maybe_unused]] auto link1 = robot.addLink(
      "shoulder_link",
      {.parentLink = base,
       .jointName = "shoulder_pan",
       .jointType = dart::simulation::JointType::Revolute,
       .axis = {0, 0, 1}});
  auto link2 = robot.addLink(
      "upper_arm_link",
      {.parentLink = link1,
       .jointName = "shoulder_lift",
       .jointType = dart::simulation::JointType::Revolute,
       .axis = {0, 1, 0}});
  auto link3 = robot.addLink(
      "forearm_link",
      {.parentLink = link2,
       .jointName = "elbow",
       .jointType = dart::simulation::JointType::Revolute,
       .axis = {0, 1, 0}});
  auto link4 = robot.addLink(
      "wrist_1_link",
      {.parentLink = link3,
       .jointName = "wrist_1",
       .jointType = dart::simulation::JointType::Revolute,
       .axis = {0, 1, 0}});
  auto link5 = robot.addLink(
      "wrist_2_link",
      {.parentLink = link4,
       .jointName = "wrist_2",
       .jointType = dart::simulation::JointType::Revolute,
       .axis = {0, 0, 1}});
  [[maybe_unused]] auto link6 = robot.addLink(
      "wrist_3_link",
      {.parentLink = link5,
       .jointName = "wrist_3",
       .jointType = dart::simulation::JointType::Revolute,
       .axis = {0, 1, 0}});

  std::stringstream ss;
  world1.saveBinary(ss);

  dart::simulation::World world2;
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
  EXPECT_EQ(shoulder_pan->getType(), dart::simulation::JointType::Revolute);
  auto axis1 = shoulder_pan->getAxis();
  EXPECT_DOUBLE_EQ(axis1[0], 0.0);
  EXPECT_DOUBLE_EQ(axis1[1], 0.0);
  EXPECT_DOUBLE_EQ(axis1[2], 1.0);

  auto shoulder_lift = robot_restored->getJoint("shoulder_lift");
  ASSERT_TRUE(shoulder_lift.has_value());
  EXPECT_EQ(shoulder_lift->getType(), dart::simulation::JointType::Revolute);
  auto axis2 = shoulder_lift->getAxis();
  EXPECT_DOUBLE_EQ(axis2[0], 0.0);
  EXPECT_DOUBLE_EQ(axis2[1], 1.0);
  EXPECT_DOUBLE_EQ(axis2[2], 0.0);

  auto elbow = robot_restored->getJoint("elbow");
  ASSERT_TRUE(elbow.has_value());
  EXPECT_EQ(elbow->getType(), dart::simulation::JointType::Revolute);
  auto axis3 = elbow->getAxis();
  EXPECT_DOUBLE_EQ(axis3[0], 0.0);
  EXPECT_DOUBLE_EQ(axis3[1], 1.0);
  EXPECT_DOUBLE_EQ(axis3[2], 0.0);

  auto wrist_1 = robot_restored->getJoint("wrist_1");
  ASSERT_TRUE(wrist_1.has_value());
  EXPECT_EQ(wrist_1->getType(), dart::simulation::JointType::Revolute);

  auto wrist_2 = robot_restored->getJoint("wrist_2");
  ASSERT_TRUE(wrist_2.has_value());
  EXPECT_EQ(wrist_2->getType(), dart::simulation::JointType::Revolute);
  auto axis5 = wrist_2->getAxis();
  EXPECT_DOUBLE_EQ(axis5[0], 0.0);
  EXPECT_DOUBLE_EQ(axis5[1], 0.0);
  EXPECT_DOUBLE_EQ(axis5[2], 1.0);

  auto wrist_3 = robot_restored->getJoint("wrist_3");
  ASSERT_TRUE(wrist_3.has_value());
  EXPECT_EQ(wrist_3->getType(), dart::simulation::JointType::Revolute);
  auto axis6 = wrist_3->getAxis();
  EXPECT_DOUBLE_EQ(axis6[0], 0.0);
  EXPECT_DOUBLE_EQ(axis6[1], 1.0);
  EXPECT_DOUBLE_EQ(axis6[2], 0.0);
}

// Test multiple multibodies
TEST(Serialization, MultipleMultibodies)
{
  dart::simulation::World world1;
  [[maybe_unused]] auto mb1 = world1.addMultibody("robot1");
  [[maybe_unused]] auto mb2 = world1.addMultibody("robot2");
  auto mb3 = world1.addMultibody("robot3");

  mb1.addLink("base1");
  mb2.addLink("base2");
  mb3.addLink("base3");

  std::stringstream ss;
  world1.saveBinary(ss);

  dart::simulation::World world2;
  world2.loadBinary(ss);

  EXPECT_EQ(world2.getMultibodyCount(), 3);
  EXPECT_TRUE(world2.getMultibody("robot1").has_value());
  EXPECT_TRUE(world2.getMultibody("robot2").has_value());
  EXPECT_TRUE(world2.getMultibody("robot3").has_value());
}

// Test mixed multibodies and rigid bodies
TEST(Serialization, MixedContent)
{
  dart::simulation::World world1;

  auto mb = world1.addMultibody("robot");
  mb.addLink("base");

  [[maybe_unused]] auto rb1 = world1.addRigidBody("box1");
  [[maybe_unused]] auto rb2 = world1.addRigidBody("box2");

  std::stringstream ss;
  world1.saveBinary(ss);

  dart::simulation::World world2;
  world2.loadBinary(ss);

  EXPECT_EQ(world2.getMultibodyCount(), 1);
  EXPECT_EQ(world2.getRigidBodyCount(), 2);
  EXPECT_TRUE(world2.getMultibody("robot").has_value());
  EXPECT_TRUE(world2.hasRigidBody("box1"));
  EXPECT_TRUE(world2.hasRigidBody("box2"));
}

TEST(Serialization, RigidBodyFixedJointBreakageRoundTrips)
{
  namespace sx = dart::simulation;

  expectBrokenRigidBodyJointRoundTrips(
      "fixed",
      sx::JointType::Fixed,
      0u,
      false,
      Eigen::Vector3d::Zero(),
      [](sx::World& world,
         std::string_view name,
         const sx::RigidBody& parent,
         const sx::RigidBody& child) {
        return world.addRigidBodyFixedJoint(name, parent, child);
      });
}

TEST(Serialization, RigidBodyRevoluteJointBreakageRoundTrips)
{
  namespace sx = dart::simulation;

  expectBrokenRigidBodyJointRoundTrips(
      "hinge",
      sx::JointType::Revolute,
      1u,
      true,
      Eigen::Vector3d::UnitY(),
      [](sx::World& world,
         std::string_view name,
         const sx::RigidBody& parent,
         const sx::RigidBody& child) {
        return world.addRigidBodyRevoluteJoint(
            name, parent, child, Eigen::Vector3d::UnitY());
      });
}

TEST(Serialization, RigidBodyPrismaticJointBreakageRoundTrips)
{
  namespace sx = dart::simulation;

  expectBrokenRigidBodyJointRoundTrips(
      "slider",
      sx::JointType::Prismatic,
      1u,
      true,
      Eigen::Vector3d::UnitY(),
      [](sx::World& world,
         std::string_view name,
         const sx::RigidBody& parent,
         const sx::RigidBody& child) {
        return world.addRigidBodyPrismaticJoint(
            name, parent, child, Eigen::Vector3d::UnitY());
      });
}

TEST(Serialization, RigidBodySphericalJointBreakageRoundTrips)
{
  namespace sx = dart::simulation;

  expectBrokenRigidBodyJointRoundTrips(
      "socket",
      sx::JointType::Spherical,
      3u,
      false,
      Eigen::Vector3d::Zero(),
      [](sx::World& world,
         std::string_view name,
         const sx::RigidBody& parent,
         const sx::RigidBody& child) {
        return world.addRigidBodySphericalJoint(name, parent, child);
      });
}

TEST(Serialization, ArticulatedPointJointBreakageRoundTrips)
{
  namespace sx = dart::simulation;

  const Eigen::Vector3d axis = Eigen::Vector3d::UnitY();

  expectBrokenArticulatedPointJointRoundTrips(
      "same_fixed",
      sx::JointType::Fixed,
      0u,
      false,
      Eigen::Vector3d::Zero(),
      true,
      [](sx::World& world,
         std::string_view name,
         const sx::Link& parent,
         const sx::Link& child,
         const Eigen::Vector3d& parentAnchor,
         const Eigen::Vector3d& childAnchor) {
        return world.addArticulatedFixedJoint(
            name, parent, child, parentAnchor, childAnchor);
      });
  expectBrokenArticulatedPointJointRoundTrips(
      "same_hinge",
      sx::JointType::Revolute,
      1u,
      true,
      axis,
      true,
      [axis](
          sx::World& world,
          std::string_view name,
          const sx::Link& parent,
          const sx::Link& child,
          const Eigen::Vector3d& parentAnchor,
          const Eigen::Vector3d& childAnchor) {
        return world.addArticulatedRevoluteJoint(
            name, parent, child, axis, parentAnchor, childAnchor);
      });
  expectBrokenArticulatedPointJointRoundTrips(
      "same_slider",
      sx::JointType::Prismatic,
      1u,
      true,
      axis,
      true,
      [axis](
          sx::World& world,
          std::string_view name,
          const sx::Link& parent,
          const sx::Link& child,
          const Eigen::Vector3d& parentAnchor,
          const Eigen::Vector3d& childAnchor) {
        return world.addArticulatedPrismaticJoint(
            name, parent, child, axis, parentAnchor, childAnchor);
      });
  expectBrokenArticulatedPointJointRoundTrips(
      "same_socket",
      sx::JointType::Spherical,
      3u,
      false,
      Eigen::Vector3d::Zero(),
      true,
      [](sx::World& world,
         std::string_view name,
         const sx::Link& parent,
         const sx::Link& child,
         const Eigen::Vector3d& parentAnchor,
         const Eigen::Vector3d& childAnchor) {
        return world.addArticulatedSphericalJoint(
            name, parent, child, parentAnchor, childAnchor);
      });

  expectBrokenArticulatedPointJointRoundTrips(
      "world_fixed",
      sx::JointType::Fixed,
      0u,
      false,
      Eigen::Vector3d::Zero(),
      false,
      [](sx::World& world,
         std::string_view name,
         const sx::Link&,
         const sx::Link& child,
         const Eigen::Vector3d& worldAnchor,
         const Eigen::Vector3d& childAnchor) {
        return world.addArticulatedFixedJoint(
            name, child, worldAnchor, childAnchor);
      });
  expectBrokenArticulatedPointJointRoundTrips(
      "world_hinge",
      sx::JointType::Revolute,
      1u,
      true,
      axis,
      false,
      [axis](
          sx::World& world,
          std::string_view name,
          const sx::Link&,
          const sx::Link& child,
          const Eigen::Vector3d& worldAnchor,
          const Eigen::Vector3d& childAnchor) {
        return world.addArticulatedRevoluteJoint(
            name, child, axis, worldAnchor, childAnchor);
      });
  expectBrokenArticulatedPointJointRoundTrips(
      "world_slider",
      sx::JointType::Prismatic,
      1u,
      true,
      axis,
      false,
      [axis](
          sx::World& world,
          std::string_view name,
          const sx::Link&,
          const sx::Link& child,
          const Eigen::Vector3d& worldAnchor,
          const Eigen::Vector3d& childAnchor) {
        return world.addArticulatedPrismaticJoint(
            name, child, axis, worldAnchor, childAnchor);
      });
  expectBrokenArticulatedPointJointRoundTrips(
      "world_socket",
      sx::JointType::Spherical,
      3u,
      false,
      Eigen::Vector3d::Zero(),
      false,
      [](sx::World& world,
         std::string_view name,
         const sx::Link&,
         const sx::Link& child,
         const Eigen::Vector3d& worldAnchor,
         const Eigen::Vector3d& childAnchor) {
        return world.addArticulatedSphericalJoint(
            name, child, worldAnchor, childAnchor);
      });
}

TEST(Serialization, RigidBodyJointAvbdStiffnessRoundTripsDesignMode)
{
  namespace sx = dart::simulation;

  sx::World world1;

  sx::RigidBodyOptions parentOptions;
  parentOptions.isStatic = true;
  auto parent = world1.addRigidBody("parent", parentOptions);

  sx::RigidBodyOptions childOptions;
  childOptions.position = Eigen::Vector3d::UnitX();
  auto child = world1.addRigidBody("child", childOptions);

  auto joint = world1.addRigidBodyFixedJoint("fixed", parent, child);
  joint.setAvbdStartStiffness(2.0);
  joint.setAvbdLinearStiffness(123.0);
  joint.setAvbdAngularStiffness(456.0);
  ASSERT_FALSE(world1.isSimulationMode());

  std::stringstream ss;
  world1.saveBinary(ss);

  sx::World world2;
  world2.loadBinary(ss);

  ASSERT_FALSE(world2.isSimulationMode());
  auto restoredJoint = world2.getRigidBodyJoint("fixed");
  ASSERT_TRUE(restoredJoint.has_value());
  EXPECT_DOUBLE_EQ(restoredJoint->getAvbdStartStiffness(), 2.0);
  EXPECT_DOUBLE_EQ(restoredJoint->getAvbdLinearStiffness(), 123.0);
  EXPECT_DOUBLE_EQ(restoredJoint->getAvbdAngularStiffness(), 456.0);

  restoredJoint->setAvbdLinearStiffness(789.0);
  restoredJoint->setAvbdAngularStiffness(987.0);
  EXPECT_DOUBLE_EQ(restoredJoint->getAvbdLinearStiffness(), 789.0);
  EXPECT_DOUBLE_EQ(restoredJoint->getAvbdAngularStiffness(), 987.0);

  world2.enterSimulationMode();
  restoredJoint = world2.getRigidBodyJoint("fixed");
  ASSERT_TRUE(restoredJoint.has_value());
  EXPECT_DOUBLE_EQ(restoredJoint->getAvbdStartStiffness(), 2.0);
  EXPECT_DOUBLE_EQ(restoredJoint->getAvbdLinearStiffness(), 789.0);
  EXPECT_DOUBLE_EQ(restoredJoint->getAvbdAngularStiffness(), 987.0);
}

TEST(Serialization, ArticulatedJointAvbdStiffnessRoundTripsDesignMode)
{
  namespace sx = dart::simulation;

  sx::World world1;
  world1.setMultibodyOptions({sx::MultibodyIntegrationFamily::Variational});

  auto robot = world1.addMultibody("robot");
  auto base = robot.addLink("base");

  const auto addFloatingLink = [&](std::string_view name) {
    sx::JointSpec spec;
    spec.name = std::string(name) + "_float";
    spec.type = sx::JointType::Floating;
    return robot.addLink(name, base, spec);
  };

  auto fixedChild = addFloatingLink("fixed_child");
  auto hingeChild = addFloatingLink("hinge_child");
  auto sliderChild = addFloatingLink("slider_child");
  auto socketChild = addFloatingLink("socket_child");
  auto worldFixedChild = addFloatingLink("world_fixed_child");
  auto worldHingeChild = addFloatingLink("world_hinge_child");
  auto worldSliderChild = addFloatingLink("world_slider_child");
  auto worldSocketChild = addFloatingLink("world_socket_child");

  struct ExpectedJoint
  {
    std::string_view name;
    sx::JointType type;
    std::size_t dofs;
    double startStiffness;
    double linearStiffness;
    double angularStiffness;
  };

  const std::vector<ExpectedJoint> expectedBeforeMutation{
      {"fixed", sx::JointType::Fixed, 0u, 3.0, 234.0, 567.0},
      {"hinge", sx::JointType::Revolute, 1u, 4.0, 345.0, 678.0},
      {"slider", sx::JointType::Prismatic, 1u, 5.0, 456.0, 789.0},
      {"socket", sx::JointType::Spherical, 3u, 6.0, 567.0, 890.0},
      {"world_fixed", sx::JointType::Fixed, 0u, 7.0, 678.0, 901.0},
      {"world_hinge", sx::JointType::Revolute, 1u, 8.0, 789.0, 1012.0},
      {"world_slider", sx::JointType::Prismatic, 1u, 9.0, 890.0, 1123.0},
      {"world_socket", sx::JointType::Spherical, 3u, 10.0, 901.0, 1234.0},
  };

  auto fixed = world1.addArticulatedFixedJoint("fixed", base, fixedChild);
  fixed.setAvbdStartStiffness(3.0);
  fixed.setAvbdLinearStiffness(234.0);
  fixed.setAvbdAngularStiffness(567.0);

  auto hinge = world1.addArticulatedRevoluteJoint(
      "hinge", base, hingeChild, Eigen::Vector3d::UnitY());
  hinge.setAvbdStartStiffness(4.0);
  hinge.setAvbdLinearStiffness(345.0);
  hinge.setAvbdAngularStiffness(678.0);

  auto slider = world1.addArticulatedPrismaticJoint(
      "slider", base, sliderChild, Eigen::Vector3d::UnitX());
  slider.setAvbdStartStiffness(5.0);
  slider.setAvbdLinearStiffness(456.0);
  slider.setAvbdAngularStiffness(789.0);

  auto socket
      = world1.addArticulatedSphericalJoint("socket", base, socketChild);
  socket.setAvbdStartStiffness(6.0);
  socket.setAvbdLinearStiffness(567.0);
  socket.setAvbdAngularStiffness(890.0);

  auto worldFixed
      = world1.addArticulatedFixedJoint("world_fixed", worldFixedChild);
  worldFixed.setAvbdStartStiffness(7.0);
  worldFixed.setAvbdLinearStiffness(678.0);
  worldFixed.setAvbdAngularStiffness(901.0);

  auto worldHinge = world1.addArticulatedRevoluteJoint(
      "world_hinge", worldHingeChild, Eigen::Vector3d::UnitY());
  worldHinge.setAvbdStartStiffness(8.0);
  worldHinge.setAvbdLinearStiffness(789.0);
  worldHinge.setAvbdAngularStiffness(1012.0);

  auto worldSlider = world1.addArticulatedPrismaticJoint(
      "world_slider", worldSliderChild, Eigen::Vector3d::UnitX());
  worldSlider.setAvbdStartStiffness(9.0);
  worldSlider.setAvbdLinearStiffness(890.0);
  worldSlider.setAvbdAngularStiffness(1123.0);

  auto worldSocket
      = world1.addArticulatedSphericalJoint("world_socket", worldSocketChild);
  worldSocket.setAvbdStartStiffness(10.0);
  worldSocket.setAvbdLinearStiffness(901.0);
  worldSocket.setAvbdAngularStiffness(1234.0);
  ASSERT_FALSE(world1.isSimulationMode());

  std::stringstream ss;
  world1.saveBinary(ss);

  sx::World world2;
  world2.loadBinary(ss);

  ASSERT_FALSE(world2.isSimulationMode());
  EXPECT_EQ(
      world2.getMultibodyOptions().integrationFamily,
      sx::MultibodyIntegrationFamily::Variational);
  for (const auto& expected : expectedBeforeMutation) {
    SCOPED_TRACE(expected.name);
    auto restoredJoint = world2.getArticulatedJoint(expected.name);
    ASSERT_TRUE(restoredJoint.has_value());
    EXPECT_EQ(restoredJoint->getType(), expected.type);
    EXPECT_EQ(restoredJoint->getDOFCount(), expected.dofs);
    EXPECT_DOUBLE_EQ(
        restoredJoint->getAvbdStartStiffness(), expected.startStiffness);
    EXPECT_DOUBLE_EQ(
        restoredJoint->getAvbdLinearStiffness(), expected.linearStiffness);
    EXPECT_DOUBLE_EQ(
        restoredJoint->getAvbdAngularStiffness(), expected.angularStiffness);
  }

  const std::vector<ExpectedJoint> expectedAfterMutation{
      {"fixed", sx::JointType::Fixed, 0u, 3.0, 432.0, 765.0},
      {"hinge", sx::JointType::Revolute, 1u, 4.0, 543.0, 876.0},
      {"slider", sx::JointType::Prismatic, 1u, 5.0, 654.0, 987.0},
      {"socket", sx::JointType::Spherical, 3u, 6.0, 765.0, 1098.0},
      {"world_fixed", sx::JointType::Fixed, 0u, 7.0, 876.0, 1209.0},
      {"world_hinge", sx::JointType::Revolute, 1u, 8.0, 987.0, 1320.0},
      {"world_slider", sx::JointType::Prismatic, 1u, 9.0, 1098.0, 1431.0},
      {"world_socket", sx::JointType::Spherical, 3u, 10.0, 1209.0, 1542.0},
  };
  for (const auto& expected : expectedAfterMutation) {
    SCOPED_TRACE(expected.name);
    auto restoredJoint = world2.getArticulatedJoint(expected.name);
    ASSERT_TRUE(restoredJoint.has_value());
    restoredJoint->setAvbdLinearStiffness(expected.linearStiffness);
    restoredJoint->setAvbdAngularStiffness(expected.angularStiffness);
    EXPECT_DOUBLE_EQ(
        restoredJoint->getAvbdLinearStiffness(), expected.linearStiffness);
    EXPECT_DOUBLE_EQ(
        restoredJoint->getAvbdAngularStiffness(), expected.angularStiffness);
  }

  world2.enterSimulationMode();
  for (const auto& expected : expectedAfterMutation) {
    SCOPED_TRACE(expected.name);
    auto restoredJoint = world2.getArticulatedJoint(expected.name);
    ASSERT_TRUE(restoredJoint.has_value());
    EXPECT_EQ(restoredJoint->getType(), expected.type);
    EXPECT_EQ(restoredJoint->getDOFCount(), expected.dofs);
    EXPECT_DOUBLE_EQ(
        restoredJoint->getAvbdStartStiffness(), expected.startStiffness);
    EXPECT_DOUBLE_EQ(
        restoredJoint->getAvbdLinearStiffness(), expected.linearStiffness);
    EXPECT_DOUBLE_EQ(
        restoredJoint->getAvbdAngularStiffness(), expected.angularStiffness);
  }
}

TEST(Serialization, AvbdPointJointConfigSerializerRoundTripsAllFields)
{
  namespace sx = dart::simulation;
  namespace dvbd = dart::simulation::detail::deformable_vbd;

  const auto* serializer = sx::io::SerializerRegistry::instance().getSerializer(
      "dart::simulation::detail::deformable_vbd::"
      "AvbdRigidWorldPointJointConfig");
  ASSERT_NE(serializer, nullptr);

  sx::detail::WorldRegistry registry1;
  const entt::entity entity1 = registry1.create();
  auto& config
      = registry1.emplace<dvbd::AvbdRigidWorldPointJointConfig>(entity1);
  config.enabled = false;
  config.localAnchorA = Eigen::Vector3d(1.0, -2.0, 3.0);
  config.localAnchorB = Eigen::Vector3d(-4.0, 5.0, -6.0);
  config.targetRelativeOrientation
      = Eigen::Quaterniond(Eigen::AngleAxisd(0.25, Eigen::Vector3d::UnitY()));
  config.linearAxes << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0;
  config.angularAxes << -1.0, -2.0, -3.0, -4.0, -5.0, -6.0, -7.0, -8.0, -9.0;
  config.linearAxisMask = 0b101u;
  config.angularAxisMask = 0b011u;
  config.startStiffness = 12.0;
  config.linearMaterialStiffness = 34.0;
  config.angularMaterialStiffness = 56.0;
  config.maxStiffness = 78.0;

  sx::io::EntityMap entityMap;
  entityMap.emplace(entity1, entity1);
  std::stringstream stream;
  serializer->save(stream, entity1, registry1, entityMap);

  sx::detail::WorldRegistry registry2;
  const entt::entity entity2 = registry2.create();
  serializer->load(stream, entity2, registry2);

  ASSERT_TRUE(registry2.all_of<dvbd::AvbdRigidWorldPointJointConfig>(entity2));
  const auto& restored
      = registry2.get<dvbd::AvbdRigidWorldPointJointConfig>(entity2);
  EXPECT_FALSE(restored.enabled);
  EXPECT_TRUE(restored.localAnchorA.isApprox(config.localAnchorA));
  EXPECT_TRUE(restored.localAnchorB.isApprox(config.localAnchorB));
  EXPECT_TRUE(restored.targetRelativeOrientation.coeffs().isApprox(
      config.targetRelativeOrientation.coeffs()));
  EXPECT_TRUE(restored.linearAxes.isApprox(config.linearAxes));
  EXPECT_TRUE(restored.angularAxes.isApprox(config.angularAxes));
  EXPECT_EQ(restored.linearAxisMask, config.linearAxisMask);
  EXPECT_EQ(restored.angularAxisMask, config.angularAxisMask);
  EXPECT_DOUBLE_EQ(restored.startStiffness, config.startStiffness);
  EXPECT_DOUBLE_EQ(
      restored.linearMaterialStiffness, config.linearMaterialStiffness);
  EXPECT_DOUBLE_EQ(
      restored.angularMaterialStiffness, config.angularMaterialStiffness);
  EXPECT_DOUBLE_EQ(restored.maxStiffness, config.maxStiffness);
}

TEST(Serialization, AvbdDistanceSpringConfigSerializerRoundTripsAllFields)
{
  namespace sx = dart::simulation;
  namespace dvbd = dart::simulation::detail::deformable_vbd;

  const auto* serializer = sx::io::SerializerRegistry::instance().getSerializer(
      "dart::simulation::detail::deformable_vbd::"
      "AvbdRigidWorldDistanceSpringConfig");
  ASSERT_NE(serializer, nullptr);

  sx::detail::WorldRegistry registry1;
  const entt::entity bodyA = registry1.create();
  const entt::entity bodyB = registry1.create();
  const entt::entity springEntity = registry1.create();
  auto& config = registry1.emplace<dvbd::AvbdRigidWorldDistanceSpringConfig>(
      springEntity);
  config.enabled = false;
  config.bodyA = bodyA;
  config.bodyB = bodyB;
  config.localAnchorA = Eigen::Vector3d(1.0, -2.0, 3.0);
  config.localAnchorB = Eigen::Vector3d(-4.0, 5.0, -6.0);
  config.restLength = 7.0;
  config.startStiffness = 8.0;
  config.materialStiffness = 9.0;
  config.maxStiffness = 10.0;

  sx::io::EntityMap entityMap;
  entityMap.emplace(bodyA, bodyA);
  entityMap.emplace(bodyB, bodyB);
  std::stringstream stream;
  serializer->save(stream, springEntity, registry1, entityMap);

  sx::detail::WorldRegistry registry2;
  const entt::entity restoredEntity = registry2.create();
  serializer->load(stream, restoredEntity, registry2);

  ASSERT_TRUE(registry2.all_of<dvbd::AvbdRigidWorldDistanceSpringConfig>(
      restoredEntity));
  const auto& restored
      = registry2.get<dvbd::AvbdRigidWorldDistanceSpringConfig>(restoredEntity);
  EXPECT_FALSE(restored.enabled);
  EXPECT_EQ(restored.bodyA, config.bodyA);
  EXPECT_EQ(restored.bodyB, config.bodyB);
  EXPECT_TRUE(restored.localAnchorA.isApprox(config.localAnchorA));
  EXPECT_TRUE(restored.localAnchorB.isApprox(config.localAnchorB));
  EXPECT_DOUBLE_EQ(restored.restLength, config.restLength);
  EXPECT_DOUBLE_EQ(restored.startStiffness, config.startStiffness);
  EXPECT_DOUBLE_EQ(restored.materialStiffness, config.materialStiffness);
  EXPECT_DOUBLE_EQ(restored.maxStiffness, config.maxStiffness);
}

TEST(Serialization, AvbdDistanceSpringConfigLoadAllEntitiesRemapsBodies)
{
  namespace sx = dart::simulation;
  namespace dvbd = dart::simulation::detail::deformable_vbd;

  sx::detail::WorldRegistry registry1;
  const entt::entity bodyA = registry1.create();
  registry1.emplace<sx::comps::Name>(bodyA, "body_a");
  const entt::entity bodyB = registry1.create();
  registry1.emplace<sx::comps::Name>(bodyB, "body_b");
  const entt::entity springEntity = registry1.create();
  registry1.emplace<sx::comps::Name>(springEntity, "spring");

  auto& config = registry1.emplace<dvbd::AvbdRigidWorldDistanceSpringConfig>(
      springEntity);
  config.bodyA = bodyA;
  config.bodyB = bodyB;
  config.localAnchorA = Eigen::Vector3d(0.1, 0.2, 0.3);
  config.localAnchorB = Eigen::Vector3d(-0.4, 0.5, -0.6);
  config.restLength = 1.25;
  config.startStiffness = 12.0;
  config.materialStiffness = 34.0;
  config.maxStiffness = 56.0;

  sx::io::EntityMap saveMap;
  std::stringstream stream;
  sx::io::SerializerRegistry::instance().saveAllEntities(
      stream, registry1, saveMap);

  sx::detail::WorldRegistry registry2;
  const entt::entity dummy = registry2.create();
  registry2.emplace<sx::comps::Name>(dummy, "dummy");

  sx::io::EntityMap loadMap;
  sx::io::SerializerRegistry::instance().loadAllEntities(
      stream, registry2, loadMap, sx::io::kBinaryFormatVersion);

  const auto findNamedEntity
      = [&registry2](std::string_view name) -> entt::entity {
    const auto view = registry2.view<sx::comps::Name>();
    for (const entt::entity entity : view) {
      if (view.get<sx::comps::Name>(entity).name == name) {
        return entity;
      }
    }
    return entt::null;
  };

  const entt::entity restoredBodyA = findNamedEntity("body_a");
  const entt::entity restoredBodyB = findNamedEntity("body_b");
  const entt::entity restoredSpring = findNamedEntity("spring");
  const entt::entity nullEntity = entt::null;
  ASSERT_NE(restoredBodyA, nullEntity);
  ASSERT_NE(restoredBodyB, nullEntity);
  ASSERT_NE(restoredSpring, nullEntity);
  EXPECT_NE(restoredBodyA, saveMap.at(bodyA));
  EXPECT_NE(restoredBodyB, saveMap.at(bodyB));

  ASSERT_TRUE(registry2.all_of<dvbd::AvbdRigidWorldDistanceSpringConfig>(
      restoredSpring));
  const auto& restored
      = registry2.get<dvbd::AvbdRigidWorldDistanceSpringConfig>(restoredSpring);
  EXPECT_EQ(restored.bodyA, restoredBodyA);
  EXPECT_EQ(restored.bodyB, restoredBodyB);
  EXPECT_TRUE(restored.localAnchorA.isApprox(config.localAnchorA));
  EXPECT_TRUE(restored.localAnchorB.isApprox(config.localAnchorB));
  EXPECT_DOUBLE_EQ(restored.restLength, config.restLength);
  EXPECT_DOUBLE_EQ(restored.startStiffness, config.startStiffness);
  EXPECT_DOUBLE_EQ(restored.materialStiffness, config.materialStiffness);
  EXPECT_DOUBLE_EQ(restored.maxStiffness, config.maxStiffness);
}

TEST(Serialization, AvbdDistanceSpringWorldBinaryRoundTrips)
{
  namespace sx = dart::simulation;
  namespace dvbd = dart::simulation::detail::deformable_vbd;

  sx::World world1;
  sx::RigidBodyOptions baseOptions;
  baseOptions.isStatic = true;
  auto base = world1.addRigidBody("base", baseOptions);

  sx::RigidBodyOptions linkOptions;
  linkOptions.mass = 2.0;
  linkOptions.position = Eigen::Vector3d(2.0, 0.0, 0.0);
  auto link = world1.addRigidBody("link", linkOptions);

  const Eigen::Vector3d anchorA(0.1, 0.2, 0.3);
  const Eigen::Vector3d anchorB(-0.4, 0.5, -0.6);
  world1.addRigidBodyDistanceSpring(
      "spring",
      base,
      link,
      /*restLength=*/1.25,
      /*stiffness=*/42.0,
      anchorA,
      anchorB);

  std::stringstream stream;
  world1.saveBinary(stream);

  sx::World world2;
  world2.loadBinary(stream);

  const auto restoredBase = world2.getRigidBody("base");
  const auto restoredLink = world2.getRigidBody("link");
  ASSERT_TRUE(restoredBase.has_value());
  ASSERT_TRUE(restoredLink.has_value());

  std::vector<dvbd::AvbdRigidWorldDistanceSpringInput> springs;
  dvbd::extractAvbdRigidWorldDistanceSpringInputsInto(
      sx::detail::registryOf(world2),
      springs,
      /*includeWorldAnchors=*/false);
  ASSERT_EQ(springs.size(), 1u);
  EXPECT_TRUE(springs[0].anchorsAreLocal);
  EXPECT_EQ(
      springs[0].bodyA,
      sx::detail::toRegistryEntity(restoredBase->getEntity()));
  EXPECT_EQ(
      springs[0].bodyB,
      sx::detail::toRegistryEntity(restoredLink->getEntity()));
  EXPECT_TRUE(springs[0].anchorA.isApprox(anchorA));
  EXPECT_TRUE(springs[0].anchorB.isApprox(anchorB));
  EXPECT_DOUBLE_EQ(springs[0].restLength, 1.25);
  EXPECT_DOUBLE_EQ(springs[0].startStiffness, 42.0);
  EXPECT_DOUBLE_EQ(springs[0].materialStiffness, 42.0);
  EXPECT_DOUBLE_EQ(springs[0].maxStiffness, 42.0);
}

// Test auto-generated names are preserved
TEST(Serialization, EmptyNames)
{
  dart::simulation::World world1;
  auto mb = world1.addMultibody("");
  auto base = mb.addLink("");
  dart::simulation::LinkOptions opts{.parentLink = base, .jointName = ""};
  [[maybe_unused]] auto child = mb.addLink("", opts);

  std::stringstream ss;
  world1.saveBinary(ss);

  dart::simulation::World world2;
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
  dart::simulation::World world1;
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

  dart::simulation::World world2;
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
  dart::simulation::World world;

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
    auto& registry = dart::simulation::detail::registryOf(world);
    auto entity = dart::simulation::detail::toRegistryEntity(frame.getEntity());
    ASSERT_TRUE(registry.valid(entity)) << "Entity should be valid";
    ASSERT_TRUE(registry.all_of<dart::simulation::comps::FrameCache>(entity))
        << "Entity should have FrameCache component";

    const auto& cache
        = registry.get<dart::simulation::comps::FrameCache>(entity);
    EXPECT_FALSE(cache.needTransformUpdate)
        << "Cache should be clean after getTransform()";
  }

  // Serialize
  std::stringstream ss;
  world.saveBinary(ss);

  // Deserialize into new world
  dart::simulation::World world2;
  world2.loadBinary(ss);

  // Check the registry directly for the restored FreeFrame
  auto& registry2 = dart::simulation::detail::registryOf(world2);
  auto view = registry2.view<dart::simulation::comps::FreeFrameTag>();

  ASSERT_FALSE(view.empty()) << "Should have restored FreeFrame";

  auto entity2 = *view.begin();

  ASSERT_TRUE(registry2.valid(entity2)) << "Restored entity should be valid";
  ASSERT_TRUE(registry2.all_of<dart::simulation::comps::FrameCache>(entity2))
      << "Restored entity should have FrameCache component";

  const auto& cache2
      = registry2.get<dart::simulation::comps::FrameCache>(entity2);

  // CRITICAL: Cache should be dirty after load (not serialized)
  EXPECT_TRUE(cache2.needTransformUpdate)
      << "Cache should be dirty after load - proves it was not serialized";

  // Verify state WAS serialized (parent should be entt::null)
  ASSERT_TRUE(registry2.all_of<dart::simulation::comps::FrameState>(entity2))
      << "Restored entity should have FrameState component";

  const auto& state2
      = registry2.get<dart::simulation::comps::FrameState>(entity2);
  EXPECT_FALSE(registry2.valid(state2.parentFrame))
      << "State should be serialized correctly (parent should be null)";

  // Verify properties WAS serialized (local transform should match)
  ASSERT_TRUE(
      registry2.all_of<dart::simulation::comps::FreeFrameProperties>(entity2))
      << "Restored entity should have FreeFrameProperties component";

  const auto& props2
      = registry2.get<dart::simulation::comps::FreeFrameProperties>(entity2);
  EXPECT_TRUE(props2.localTransform.isApprox(T1))
      << "Properties should be serialized correctly";
}

// Test state components ARE serialized
// TODO(serialization): Re-enable parent relationship testing once name-based
// entity reference system is implemented. Currently disabled because entity IDs
// change during deserialization, making entity references invalid.
TEST(Serialization, StateSerializedCorrectly)
{
  dart::simulation::World world;

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
  dart::simulation::World world2;
  world2.loadBinary(ss);

  // Check that we have 2 FreeFrames
  auto& registry2 = dart::simulation::detail::registryOf(world2);
  auto view = registry2.view<dart::simulation::comps::FreeFrameTag>();
  EXPECT_EQ(std::ranges::distance(view), 2)
      << "Should have restored 2 FreeFrames";

  // Verify that FrameState components exist
  for (auto entity : view) {
    // All FreeFrames should have FrameState component
    EXPECT_TRUE(registry2.all_of<dart::simulation::comps::FrameState>(entity))
        << "All FreeFrames should have FrameState after deserialization";
  }

  // Verify parent relationships are preserved after serialization
  // Count how many have valid parent references
  int framesWithParent = 0;
  int framesWithoutParent = 0;

  for (auto entity : view) {
    const auto& state
        = registry2.get<dart::simulation::comps::FrameState>(entity);
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
  dart::simulation::World world;

  // Create FixedFrame with specific offset
  auto parent = world.addFreeFrame("parent");
  Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
  offset.translate(Eigen::Vector3d(3.14, 2.71, 1.41));
  offset.rotate(
      Eigen::AngleAxisd(dart::simulation::pi / 4, Eigen::Vector3d(1, 0, 0)));

  auto fixed = world.addFixedFrame("fixed", parent, offset);

  // Serialize
  std::stringstream ss;
  world.saveBinary(ss);

  // Deserialize
  dart::simulation::World world2;
  world2.loadBinary(ss);

  // Find the FixedFrame
  auto& registry2 = dart::simulation::detail::registryOf(world2);
  auto view = registry2.view<dart::simulation::comps::FixedFrameTag>();
  EXPECT_FALSE(view.empty()) << "Should have restored FixedFrame";

  auto entity = *view.begin();
  const auto& props
      = registry2.get<dart::simulation::comps::FixedFrameProperties>(entity);

  // Properties should match exactly
  EXPECT_TRUE(props.localTransform.isApprox(offset, 1e-10))
      << "Properties (localTransform) should be serialized correctly";
}

// Test round-trip consistency (save twice, should be identical)
TEST(Serialization, RoundTripConsistency)
{
  dart::simulation::World world;

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

  dart::simulation::World world2;
  world2.loadBinary(ss1);

  auto& registry2 = dart::simulation::detail::registryOf(world2);
  auto view = registry2.view<
      dart::simulation::comps::Name,
      dart::simulation::comps::FreeFrameProperties,
      dart::simulation::comps::FrameState>();

  std::size_t frameCount = 0;
  for (auto entity : view) {
    const auto& name = view.get<dart::simulation::comps::Name>(entity).name;
    const auto& props
        = view.get<dart::simulation::comps::FreeFrameProperties>(entity);
    const auto& state = view.get<dart::simulation::comps::FrameState>(entity);

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
  dart::simulation::World world;

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

  dart::simulation::World clone;
  clone.loadBinary(ss);

  auto& cloneReg = dart::simulation::detail::registryOf(clone);
  auto cloneView = cloneReg.view<
      dart::simulation::comps::Name,
      dart::simulation::comps::FreeFrameProperties>();
  EXPECT_EQ(std::ranges::distance(cloneView), 2);

  for (auto entity : cloneView) {
    auto& props
        = cloneView.get<dart::simulation::comps::FreeFrameProperties>(entity);
    const auto& name
        = cloneView.get<dart::simulation::comps::Name>(entity).name;
    if (name == "parent") {
      EXPECT_TRUE(props.localTransform.isApprox(T));
    } else if (name == "child") {
      EXPECT_TRUE(props.localTransform.isApprox(T_child));
      props.localTransform.translation() = Eigen::Vector3d(100, 200, 300);
    }
  }

  const auto& originalReg = dart::simulation::detail::registryOf(world);
  auto originalView = originalReg.view<
      dart::simulation::comps::Name,
      dart::simulation::comps::FreeFrameProperties>();
  for (auto entity : originalView) {
    const auto& props
        = originalView.get<dart::simulation::comps::FreeFrameProperties>(
            entity);
    const auto& name
        = originalView.get<dart::simulation::comps::Name>(entity).name;
    if (name == "child") {
      EXPECT_TRUE(props.localTransform.isApprox(T_child))
          << "Modifying clone should not affect original world";
    }
  }
}

TEST(Serialization, CloneResetCounters)
{
  dart::simulation::World world;
  auto frame1 = world.addFreeFrame(); // free_frame_001
  auto frame2 = world.addFreeFrame(); // free_frame_002
  (void)frame1;
  (void)frame2;
  [[maybe_unused]] auto mb1 = world.addMultibody(""); // multibody_001

  std::stringstream ss;
  world.saveBinary(ss);

  dart::simulation::World clone;
  clone.loadBinary(ss);

  auto nextFrame = clone.addFreeFrame();
  auto& cloneReg = dart::simulation::detail::registryOf(clone);
  const auto& nextFrameName
      = cloneReg
            .get<dart::simulation::comps::Name>(
                dart::simulation::detail::toRegistryEntity(
                    nextFrame.getEntity()))
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
void buildReplayChain(dart::simulation::World& world)
{
  namespace sx = dart::simulation;
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
Eigen::VectorXd chainState(dart::simulation::World& world)
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
void buildFallingBody(dart::simulation::World& world)
{
  namespace sx = dart::simulation;
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
  namespace sx = dart::simulation;

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
  namespace sx = dart::simulation;

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

TEST(Serialization, SimulationModeReloadReservesRegistryStorageBeforeStep)
{
  namespace sx = dart::simulation;

  sx::World original;
  buildReplayChain(original);
  original.enterSimulationMode();
  original.step(15);

  std::stringstream ss;
  original.saveBinary(ss);

  CountingMemoryAllocator allocator;
  sx::WorldOptions options;
  options.baseAllocator = &allocator;
  sx::World resumed(options);
  resumed.loadBinary(ss);
  ASSERT_TRUE(resumed.isSimulationMode());

  const auto capacitiesAfterLoad
      = registryStorageCapacities(sx::detail::registryOf(resumed));
  const auto allocationsAfterLoad = allocator.allocationCount;
  const auto deallocationsAfterLoad = allocator.deallocationCount;
  const auto alignedAllocationsAfterLoad = allocator.alignedAllocationCount;
  const auto alignedDeallocationsAfterLoad = allocator.alignedDeallocationCount;

  resumed.step();

  expectRegistryStorageCapacitiesUnchanged(
      capacitiesAfterLoad, sx::detail::registryOf(resumed));
  EXPECT_EQ(allocator.allocationCount, allocationsAfterLoad);
  EXPECT_EQ(allocator.deallocationCount, deallocationsAfterLoad);
  EXPECT_EQ(allocator.alignedAllocationCount, alignedAllocationsAfterLoad);
  EXPECT_EQ(allocator.alignedDeallocationCount, alignedDeallocationsAfterLoad);
}

// Same replay guarantee for the rigid-body integration path, checked through
// the world state vector [q; q̇] which is populated for dynamic rigid bodies.
TEST(Serialization, RigidBodyCheckpointReloadContinuesIdentically)
{
  namespace sx = dart::simulation;

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

//==============================================================================
// Stable serialization identity (WP-091.23)
//
// On-disk component identity must be an explicit, stable, compiler-independent
// string ID, decoupled from the C++ type spelling. These tests are the
// acceptance gate: they prove the identity is NOT the compiler-mangled
// typeid(T).name(), that IDs are globally unique, and that a component
// round-trips keyed by its stable ID.
//==============================================================================

// Each component's getTypeName() returns the EXPLICIT stable ID, not the
// compiler-mangled typeid string. Renaming the C++ type must not change these
// literals. The expected literals below are hard-coded on purpose so that a
// future C++ rename that silently changes the on-disk identity fails here.
TEST(StableComponentIds, ComponentsReturnExplicitStableId)
{
  namespace comps = dart::simulation::comps;
  namespace compute = dart::simulation::compute;

  EXPECT_EQ(comps::Name::getTypeName(), "comps.Name");
  EXPECT_EQ(comps::Transform::getTypeName(), "comps.Transform");
  EXPECT_EQ(comps::Velocity::getTypeName(), "comps.Velocity");
  EXPECT_EQ(comps::MassProperties::getTypeName(), "comps.MassProperties");
  EXPECT_EQ(comps::Force::getTypeName(), "comps.Force");
  EXPECT_EQ(comps::ContactMaterial::getTypeName(), "comps.ContactMaterial");
  EXPECT_EQ(comps::JointModel::getTypeName(), "comps.JointModel");
  EXPECT_EQ(comps::JointState::getTypeName(), "comps.JointState");
  EXPECT_EQ(comps::JointActuation::getTypeName(), "comps.JointActuation");
  EXPECT_EQ(
      comps::AvbdJointStiffness::getTypeName(), "comps.AvbdJointStiffness");
  EXPECT_EQ(comps::LinkModel::getTypeName(), "comps.LinkModel");
  EXPECT_EQ(comps::LinkState::getTypeName(), "comps.LinkState");
  EXPECT_EQ(comps::LinkControl::getTypeName(), "comps.LinkControl");
  EXPECT_EQ(comps::FrameTag::getTypeName(), "comps.FrameTag");
  EXPECT_EQ(comps::FrameState::getTypeName(), "comps.FrameState");
  EXPECT_EQ(comps::FrameCache::getTypeName(), "comps.FrameCache");
  EXPECT_EQ(
      comps::MultibodyStructure::getTypeName(), "comps.MultibodyStructure");
  EXPECT_EQ(
      compute::MultibodyVariationalState::getTypeName(),
      "compute.MultibodyVariationalState");

  // The identity must not be the compiler-mangled typeid name. On Itanium ABI
  // (GCC/Clang) the mangled name for comps::Name is non-empty and differs from
  // the stable literal; this guards against a regression that re-introduces
  // typeid(T).name() into the macro.
  EXPECT_NE(
      std::string_view(comps::Name::getTypeName()),
      std::string_view(typeid(comps::Name).name()));
}

// Every registered serializer's identity (the SerializerRegistry key) is
// globally unique and never looks like a compiler-mangled typeid string. A
// duplicate key would silently corrupt serialization (two components sharing
// one on-disk identity), so this is the core uniqueness guard.
TEST(StableComponentIds, RegisteredIdsAreUniqueAndNotMangled)
{
  namespace io = dart::simulation::io;

  const auto& serializers = io::SerializerRegistry::instance().getSerializers();
  ASSERT_FALSE(serializers.empty());

  std::set<std::string> seen;
  for (const auto& [key, serializer] : serializers) {
    // The map key and the serializer's reported identity must agree.
    EXPECT_EQ(key, serializer->getTypeName());

    // Identity must be non-empty.
    EXPECT_FALSE(key.empty()) << "Empty component identity is not allowed";

    // Uniqueness: std::unordered_map keys are unique by construction, but
    // assert explicitly to document the invariant and to catch any future
    // change that iterates a non-map container.
    const bool inserted = seen.insert(key).second;
    EXPECT_TRUE(inserted) << "Duplicate component identity: " << key;

    // Identity must not be a typeid mangled name. Itanium-ABI mangled names for
    // class types begin with 'N' (nested name) or a digit (length-prefixed
    // unqualified name) and never contain '.', whereas every stable ID here is
    // dotted ("comps.X"/"compute.X"). The detail::deformable_vbd configs use a
    // "dart::simulation::...::Type" literal, which is likewise not a mangled
    // form. Require either a dotted ID or an explicit "::"-qualified literal.
    const bool dotted = key.find('.') != std::string::npos;
    const bool qualified = key.find("::") != std::string::npos;
    EXPECT_TRUE(dotted || qualified)
        << "Component identity looks mangled (not a stable string): " << key;
  }

  // Spot-check that the dotted stable IDs are present in the registry, proving
  // the macro-generated identities flow through to the registry key.
  EXPECT_TRUE(seen.contains("comps.Name"));
  EXPECT_TRUE(seen.contains("comps.JointModel"));
  EXPECT_TRUE(seen.contains("comps.JointState"));
  EXPECT_TRUE(seen.contains("comps.JointActuation"));
  EXPECT_TRUE(seen.contains("comps.LinkModel"));
  EXPECT_TRUE(seen.contains("comps.LinkState"));
  EXPECT_TRUE(seen.contains("comps.LinkControl"));
  EXPECT_TRUE(seen.contains("comps.AvbdJointStiffness"));
}

// Cross-component rename stability: a component round-trips through the
// registry keyed purely by its stable string ID. We look the serializer up by
// the literal "comps.ContactMaterial" (NOT by C++ type), which proves the
// on-disk identity is decoupled from the C++ type name -- a rename of the
// comps::ContactMaterial C++ type would leave this on-disk identity (and thus
// this lookup) unchanged.
TEST(StableComponentIds, RoundTripIsKeyedByStableIdNotCppType)
{
  namespace sx = dart::simulation;
  namespace comps = dart::simulation::comps;

  const auto* serializer = sx::io::SerializerRegistry::instance().getSerializer(
      "comps.ContactMaterial");
  ASSERT_NE(serializer, nullptr)
      << "ContactMaterial must be registered under its stable ID";
  EXPECT_EQ(serializer->getTypeName(), "comps.ContactMaterial");

  sx::detail::WorldRegistry registry1;
  const entt::entity entity1 = registry1.create();
  auto& material = registry1.emplace<comps::ContactMaterial>(entity1);
  material.friction = 0.4242;
  material.restitution = 0.1234;

  sx::io::EntityMap entityMap;
  entityMap.emplace(entity1, entity1);
  std::stringstream stream;
  serializer->save(stream, entity1, registry1, entityMap);

  sx::detail::WorldRegistry registry2;
  const entt::entity entity2 = registry2.create();
  serializer->load(stream, entity2, registry2);

  ASSERT_TRUE(registry2.all_of<comps::ContactMaterial>(entity2));
  const auto& restored = registry2.get<comps::ContactMaterial>(entity2);
  EXPECT_DOUBLE_EQ(restored.friction, material.friction);
  EXPECT_DOUBLE_EQ(restored.restitution, material.restitution);
}
