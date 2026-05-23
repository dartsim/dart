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

#include <dart/simulation/experimental/body/collision_shape.hpp>
#include <dart/simulation/experimental/body/contact.hpp>
#include <dart/simulation/experimental/body/rigid_body.hpp>
#include <dart/simulation/experimental/common/exceptions.hpp>
#include <dart/simulation/experimental/comps/dynamics.hpp>
#include <dart/simulation/experimental/comps/frame_types.hpp>
#include <dart/simulation/experimental/comps/joint.hpp>
#include <dart/simulation/experimental/compute/compute_executor.hpp>
#include <dart/simulation/experimental/compute/compute_graph.hpp>
#include <dart/simulation/experimental/compute/parallel_executor.hpp>
#include <dart/simulation/experimental/compute/sequential_executor.hpp>
#include <dart/simulation/experimental/compute/world_step_stage.hpp>
#include <dart/simulation/experimental/constraint/loop_closure_spec.hpp>
#include <dart/simulation/experimental/frame/fixed_frame.hpp>
#include <dart/simulation/experimental/frame/free_frame.hpp>
#include <dart/simulation/experimental/multi_body/multi_body.hpp>
#include <dart/simulation/experimental/version.hpp>
#include <dart/simulation/experimental/world.hpp>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include <limits>
#include <numbers>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace {

class CountingKinematicsStage final
  : public dart::simulation::experimental::compute::WorldStepStage
{
public:
  [[nodiscard]] std::string_view getName() const noexcept override
  {
    return "counting_kinematics";
  }

  void execute(
      dart::simulation::experimental::World& world,
      dart::simulation::experimental::compute::ComputeExecutor& executor)
      override
  {
    ++executionCount;
    defaultStage.execute(world, executor);
  }

  int executionCount{0};
  dart::simulation::experimental::compute::KinematicsStage defaultStage;
};

class RecordingWorldStage final
  : public dart::simulation::experimental::compute::WorldStepStage
{
public:
  RecordingWorldStage(
      std::string name,
      dart::simulation::experimental::compute::ComputeStageMetadata metadata,
      std::vector<std::string>& executionOrder)
    : m_name(std::move(name)),
      m_metadata(metadata),
      m_executionOrder(executionOrder)
  {
  }

  [[nodiscard]] std::string_view getName() const noexcept override
  {
    return m_name;
  }

  [[nodiscard]] dart::simulation::experimental::compute::ComputeStageMetadata
  getMetadata() const noexcept override
  {
    return m_metadata;
  }

  void execute(
      dart::simulation::experimental::World&,
      dart::simulation::experimental::compute::ComputeExecutor& executor)
      override
  {
    dart::simulation::experimental::compute::ComputeGraph graph;
    graph.addNode(
        m_name + "_node",
        [&]() { m_executionOrder.push_back(m_name); },
        m_metadata);
    executor.execute(graph);
  }

private:
  std::string m_name;
  dart::simulation::experimental::compute::ComputeStageMetadata m_metadata;
  std::vector<std::string>& m_executionOrder;
};

class RecordingExecutor final
  : public dart::simulation::experimental::compute::ComputeExecutor
{
public:
  void execute(
      const dart::simulation::experimental::compute::ComputeGraph& graph)
      override
  {
    record(graph);
    sequential.execute(graph);
  }

  [[nodiscard]]
  dart::simulation::experimental::compute::ComputeExecutionProfile
  executeProfiled(
      const dart::simulation::experimental::compute::ComputeGraph& graph)
      override
  {
    record(graph);
    return sequential.executeProfiled(graph);
  }

  [[nodiscard]] std::size_t getWorkerCount() const override
  {
    return sequential.getWorkerCount();
  }

  std::size_t nodeCount{0};
  std::size_t edgeCount{0};
  std::size_t executeCount{0};

private:
  void record(
      const dart::simulation::experimental::compute::ComputeGraph& graph)
  {
    ++executeCount;
    nodeCount = graph.getNodeCount();
    edgeCount = graph.getEdgeCount();
  }

  dart::simulation::experimental::compute::SequentialExecutor sequential;
};

} // namespace

// Test World construction
TEST(World, Construction)
{
  // Create a World instance
  dart::simulation::experimental::World world;
  (void)world;       // Suppress unused variable warning
  EXPECT_TRUE(true); // If we get here, construction succeeded
}

// Test version information
TEST(Version, VersionString)
{
  // Test that version() returns a valid string_view
  auto ver = dart::simulation::experimental::version();
  EXPECT_FALSE(ver.empty());
  EXPECT_GT(ver.size(), 0u);

  // Test individual version components
  EXPECT_GE(dart::simulation::experimental::versionMajor(), 7);
  EXPECT_GE(dart::simulation::experimental::versionMinor(), 0);
  EXPECT_GE(dart::simulation::experimental::versionPatch(), 0);
}

//==============================================================================
// Mode Control Tests (Design Mode vs Simulation Mode)
//==============================================================================

// Test default mode is design mode
TEST(World, DefaultModeIsDesign)
{
  dart::simulation::experimental::World world;
  EXPECT_FALSE(world.isSimulationMode());
}

// Test entering simulation mode
TEST(World, EnterSimulationMode)
{
  dart::simulation::experimental::World world;

  // Initially in design mode
  EXPECT_FALSE(world.isSimulationMode());

  // Enter simulation mode
  world.enterSimulationMode();

  // Now in simulation mode
  EXPECT_TRUE(world.isSimulationMode());
}

// Test cannot enter simulation mode twice
TEST(World, CannotEnterSimulationModeTwice)
{
  dart::simulation::experimental::World world;

  // First call succeeds
  world.enterSimulationMode();
  EXPECT_TRUE(world.isSimulationMode());

  // Second call should throw
  EXPECT_THROW(
      world.enterSimulationMode(),
      dart::simulation::experimental::InvalidArgumentException);
}

// Test baking with empty world
TEST(World, BakingEmptyWorld)
{
  dart::simulation::experimental::World world;

  // Baking should work even with empty world
  EXPECT_NO_THROW(world.enterSimulationMode());
  EXPECT_TRUE(world.isSimulationMode());
}

// Test baking with multibodies
TEST(World, BakingWithMultibodies)
{
  dart::simulation::experimental::World world;

  // Create several multibodies with joints and links
  auto robot1 = world.addMultiBody("robot1");
  auto base1 = robot1.addLink("base");
  (void)robot1.addLink("link1", {.parentLink = base1, .jointName = "joint1"});

  auto robot2 = world.addMultiBody("robot2");
  auto base2 = robot2.addLink("base");
  auto link2
      = robot2.addLink("link1", {.parentLink = base2, .jointName = "joint1"});
  (void)robot2.addLink(
      "link2",
      {.parentLink = link2,
       .jointName = "joint2",
       .jointType = dart::simulation::experimental::JointType::Prismatic});

  // Baking should succeed
  EXPECT_NO_THROW(world.enterSimulationMode());
  EXPECT_TRUE(world.isSimulationMode());

  // Counts should remain the same
  EXPECT_EQ(world.getMultiBodyCount(), 2u);
  EXPECT_EQ(robot1.getLinkCount(), 2u);
  EXPECT_EQ(robot1.getJointCount(), 1u);
  EXPECT_EQ(robot2.getLinkCount(), 3u);
  EXPECT_EQ(robot2.getJointCount(), 2u);
}

// Test multibody lookup by name returns first-class handles and exposes a
// symmetric presence query for world-owned names.
TEST(World, MultiBodyLookupByName)
{
  namespace sx = dart::simulation::experimental;

  sx::World world;
  auto robot = world.addMultiBody("robot");

  auto found = world.getMultiBody("robot");
  ASSERT_TRUE(found.has_value());
  EXPECT_TRUE(found->isValid());
  EXPECT_EQ(found->getEntity(), robot.getEntity());
  EXPECT_EQ(found->getWorld(), robot.getWorld());
  EXPECT_EQ(found->getName(), "robot");
  EXPECT_TRUE(world.hasMultiBody("robot"));
  EXPECT_FALSE(world.hasMultiBody("missing"));
  EXPECT_FALSE(world.getMultiBody("missing").has_value());

  EXPECT_THROW(world.addMultiBody("robot"), sx::InvalidArgumentException);
  EXPECT_EQ(world.getMultiBodyCount(), 1u);

  sx::World worldWithExplicitGeneratedName;
  [[maybe_unused]] auto explicitName
      = worldWithExplicitGeneratedName.addMultiBody("multibody_001");
  auto generated = worldWithExplicitGeneratedName.addMultiBody("");
  EXPECT_EQ(generated.getName(), "multibody_002");
  EXPECT_TRUE(worldWithExplicitGeneratedName.hasMultiBody("multibody_001"));
  EXPECT_TRUE(worldWithExplicitGeneratedName.hasMultiBody("multibody_002"));
  EXPECT_EQ(worldWithExplicitGeneratedName.getMultiBodyCount(), 2u);
}

TEST(World, ClearInvalidatesPublicHandlesAndResetsFacadeState)
{
  namespace sx = dart::simulation::experimental;

  sx::World world;
  world.setTimeStep(0.01);
  world.setTime(0.25);

  auto robot = world.addMultiBody("robot");
  auto base = robot.addLink("base");
  auto body = world.addRigidBody("body");
  auto closure = world.addLoopClosure(
      "closure",
      {.frameA = base, .frameB = body, .family = sx::LoopClosureFamily::Point});

  EXPECT_TRUE(robot.isValid());
  EXPECT_TRUE(base.isValid());
  EXPECT_TRUE(body.isValid());
  EXPECT_TRUE(closure.isValid());
  EXPECT_TRUE(world.hasMultiBody("robot"));
  EXPECT_TRUE(world.hasRigidBody("body"));
  EXPECT_TRUE(world.hasLoopClosure("closure"));

  world.enterSimulationMode();
  world.step();
  EXPECT_TRUE(world.isSimulationMode());
  EXPECT_GT(world.getTime(), 0.0);
  EXPECT_GT(world.getFrame(), 0u);

  world.clear();

  EXPECT_FALSE(robot.isValid());
  EXPECT_FALSE(base.isValid());
  EXPECT_FALSE(body.isValid());
  EXPECT_FALSE(closure.isValid());
  EXPECT_FALSE(world.isSimulationMode());
  EXPECT_DOUBLE_EQ(world.getTimeStep(), 0.001);
  EXPECT_DOUBLE_EQ(world.getTime(), 0.0);
  EXPECT_EQ(world.getFrame(), 0u);
  EXPECT_EQ(world.getMultiBodyCount(), 0u);
  EXPECT_EQ(world.getRigidBodyCount(), 0u);
  EXPECT_EQ(world.getLoopClosureCount(), 0u);
  EXPECT_FALSE(world.hasMultiBody("robot"));
  EXPECT_FALSE(world.hasRigidBody("body"));
  EXPECT_FALSE(world.hasLoopClosure("closure"));
  EXPECT_FALSE(world.getMultiBody("robot").has_value());
  EXPECT_FALSE(world.getRigidBody("body").has_value());
  EXPECT_FALSE(world.getLoopClosure("closure").has_value());
}

// Test rigid body lookup by name returns first-class handles.
TEST(World, RigidBodyLookupByName)
{
  namespace sx = dart::simulation::experimental;

  sx::World world;
  auto body = world.addRigidBody("body");

  auto found = world.getRigidBody("body");
  ASSERT_TRUE(found.has_value());
  EXPECT_TRUE(found->isValid());
  EXPECT_TRUE(found->isSameInstanceAs(body));
  EXPECT_EQ(found->getName(), "body");

  EXPECT_FALSE(world.getRigidBody("missing").has_value());

  EXPECT_THROW(world.addRigidBody("body"), sx::InvalidArgumentException);
  EXPECT_EQ(world.getRigidBodyCount(), 1u);

  sx::World worldWithExplicitGeneratedName;
  [[maybe_unused]] auto explicitName
      = worldWithExplicitGeneratedName.addRigidBody("rigid_body_001");
  auto generated = worldWithExplicitGeneratedName.addRigidBody("");
  EXPECT_EQ(generated.getName(), "rigid_body_002");
  EXPECT_EQ(worldWithExplicitGeneratedName.getRigidBodyCount(), 2u);
}

TEST(World, LoopClosureTopology)
{
  namespace sx = dart::simulation::experimental;

  sx::World world;
  auto robot = world.addMultiBody("four_bar");
  auto base = robot.addLink("base");
  auto coupler = robot.addLink(
      "coupler",
      {.parentLink = base,
       .jointName = "shoulder",
       .jointType = sx::JointType::Revolute});
  auto ground = world.addRigidBody("ground");

  Eigen::Isometry3d offsetA = Eigen::Isometry3d::Identity();
  offsetA.translate(Eigen::Vector3d(0.5, 0.0, 0.0));
  Eigen::Isometry3d offsetB = Eigen::Isometry3d::Identity();
  offsetB.translate(Eigen::Vector3d(-0.5, 0.0, 0.0));

  auto closure = world.addLoopClosure(
      "closing_bar",
      {.frameA = coupler,
       .frameB = ground,
       .family = sx::LoopClosureFamily::Rigid,
       .offsetA = offsetA,
       .offsetB = offsetB});

  EXPECT_TRUE(closure.isValid());
  EXPECT_EQ(closure.getName(), "closing_bar");
  EXPECT_EQ(closure.getFamily(), sx::LoopClosureFamily::Rigid);
  EXPECT_TRUE(closure.getFrameA().isSameInstanceAs(coupler));
  EXPECT_TRUE(closure.getFrameB().isSameInstanceAs(ground));
  EXPECT_TRUE(closure.getOffsetA().isApprox(offsetA));
  EXPECT_TRUE(closure.getOffsetB().isApprox(offsetB));

  auto runtimePolicy = closure.getRuntimePolicy();
  EXPECT_TRUE(runtimePolicy.enabled);
  EXPECT_EQ(
      runtimePolicy.kinematics, sx::ClosureKinematicsPolicy::ResidualOnly);
  EXPECT_EQ(runtimePolicy.dynamics, sx::ClosureDynamicsPolicy::ResidualOnly);

  closure.setRuntimePolicy(
      {.enabled = true,
       .kinematics = sx::ClosureKinematicsPolicy::Project,
       .dynamics = sx::ClosureDynamicsPolicy::Solve});
  runtimePolicy = closure.getRuntimePolicy();
  EXPECT_TRUE(runtimePolicy.enabled);
  EXPECT_EQ(runtimePolicy.kinematics, sx::ClosureKinematicsPolicy::Project);
  EXPECT_EQ(runtimePolicy.dynamics, sx::ClosureDynamicsPolicy::Solve);
  closure.setRuntimePolicy(
      {.enabled = true,
       .kinematics = sx::ClosureKinematicsPolicy::ResidualOnly,
       .dynamics = sx::ClosureDynamicsPolicy::ResidualOnly});

  EXPECT_EQ(world.getLoopClosureCount(), 1u);
  EXPECT_TRUE(world.hasLoopClosure("closing_bar"));

  auto found = world.getLoopClosure("closing_bar");
  ASSERT_TRUE(found.has_value());
  EXPECT_TRUE(found->isValid());
  EXPECT_TRUE(found->getFrameA().isSameInstanceAs(coupler));
  EXPECT_FALSE(world.getLoopClosure("missing").has_value());

  auto autoClosure = world.addLoopClosure(
      "",
      {.frameA = base,
       .frameB = ground,
       .family = sx::LoopClosureFamily::Point});
  EXPECT_EQ(autoClosure.getName(), "loop_closure_001");
  EXPECT_EQ(world.getLoopClosureCount(), 2u);

  world.enterSimulationMode();
  const auto residual = closure.computeResidual();
  EXPECT_TRUE(residual.enabled);
  EXPECT_TRUE(residual.active);
  EXPECT_EQ(residual.coordinates, sx::LoopClosureResidualCoordinates::World);
  EXPECT_FALSE(residual.forceAvailable);
  ASSERT_EQ(residual.value.size(), 6);
  EXPECT_TRUE(
      residual.value.head<3>().isApprox(Eigen::Vector3d(1.0, 0.0, 0.0)));
  EXPECT_TRUE(residual.value.tail<3>().isApprox(Eigen::Vector3d::Zero()));
  EXPECT_DOUBLE_EQ(residual.norm, 1.0);
}

TEST(World, LoopClosureResidualDiagnostics)
{
  namespace sx = dart::simulation::experimental;

  sx::World world;
  sx::RigidBodyOptions bodyOptions;
  auto a = world.addRigidBody("a", bodyOptions);
  bodyOptions.position = Eigen::Vector3d(0.0, 3.0, 4.0);
  auto b = world.addRigidBody("b", bodyOptions);

  auto point = world.addLoopClosure(
      "point",
      {.frameA = a, .frameB = b, .family = sx::LoopClosureFamily::Point});
  auto distance = world.addLoopClosure(
      "distance",
      {.frameA = a, .frameB = b, .family = sx::LoopClosureFamily::Distance});

  world.enterSimulationMode();

  const auto pointResidual = point.computeResidual();
  ASSERT_EQ(pointResidual.value.size(), 3);
  EXPECT_TRUE(pointResidual.value.isApprox(Eigen::Vector3d(0.0, -3.0, -4.0)));
  EXPECT_DOUBLE_EQ(pointResidual.norm, 5.0);
  EXPECT_TRUE(pointResidual.enabled);
  EXPECT_TRUE(pointResidual.active);
  EXPECT_FALSE(pointResidual.forceAvailable);

  const auto distanceResidual = distance.computeResidual();
  ASSERT_EQ(distanceResidual.value.size(), 1);
  EXPECT_DOUBLE_EQ(distanceResidual.value[0], 5.0);
  EXPECT_DOUBLE_EQ(distanceResidual.norm, 5.0);

  point.setRuntimePolicy({
      .enabled = false,
      .kinematics = sx::ClosureKinematicsPolicy::ResidualOnly,
      .dynamics = sx::ClosureDynamicsPolicy::ResidualOnly,
  });
  const auto disabledResidual = point.computeResidual();
  EXPECT_FALSE(disabledResidual.enabled);
  EXPECT_FALSE(disabledResidual.active);
  EXPECT_TRUE(disabledResidual.value.isApprox(pointResidual.value));
}

TEST(World, LoopClosureRejectsInvalidTopology)
{
  namespace sx = dart::simulation::experimental;

  sx::World world;
  auto robot = world.addMultiBody("robot");
  auto base = robot.addLink("base");
  auto link = robot.addLink("link", {.parentLink = base, .jointName = "joint"});

  sx::World otherWorld;
  auto otherBody = otherWorld.addRigidBody("other");

  EXPECT_THROW(
      world.addLoopClosure("same", {.frameA = base, .frameB = base}),
      sx::InvalidArgumentException);

  EXPECT_THROW(
      world.addLoopClosure(
          "cross_world", {.frameA = base, .frameB = otherBody}),
      sx::InvalidArgumentException);

  EXPECT_THROW(
      world.addLoopClosure(
          "bad_family",
          {.frameA = base,
           .frameB = link,
           .family = static_cast<sx::LoopClosureFamily>(999)}),
      sx::InvalidArgumentException);

  Eigen::Isometry3d invalidOffset = Eigen::Isometry3d::Identity();
  invalidOffset.linear()(0, 0) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(
      world.addLoopClosure(
          "bad_offset",
          {.frameA = base, .frameB = link, .offsetA = invalidOffset}),
      sx::InvalidArgumentException);

  world.addLoopClosure("valid", {.frameA = base, .frameB = link});
  EXPECT_THROW(
      world.addLoopClosure("valid", {.frameA = link, .frameB = base}),
      sx::InvalidArgumentException);

  world.enterSimulationMode();
  EXPECT_THROW(
      world.addLoopClosure("after_bake", {.frameA = base, .frameB = link}),
      sx::InvalidOperationException);
}

TEST(World, LoopClosureRejectsInvalidRuntimePolicy)
{
  namespace sx = dart::simulation::experimental;

  sx::World world;
  auto robot = world.addMultiBody("robot");
  auto base = robot.addLink("base");
  auto link = robot.addLink("link", {.parentLink = base, .jointName = "joint"});
  auto closure
      = world.addLoopClosure("closure", {.frameA = base, .frameB = link});

  EXPECT_THROW(
      closure.setRuntimePolicy(
          {.enabled = true,
           .kinematics = static_cast<sx::ClosureKinematicsPolicy>(999),
           .dynamics = sx::ClosureDynamicsPolicy::ResidualOnly}),
      sx::InvalidArgumentException);

  EXPECT_THROW(
      closure.setRuntimePolicy(
          {.enabled = true,
           .kinematics = sx::ClosureKinematicsPolicy::ResidualOnly,
           .dynamics = static_cast<sx::ClosureDynamicsPolicy>(999)}),
      sx::InvalidArgumentException);
}

TEST(World, LoopClosureRejectsUnsupportedActiveRuntimePolicy)
{
  namespace sx = dart::simulation::experimental;

  sx::World world;
  auto robot = world.addMultiBody("robot");
  auto base = robot.addLink("base");
  auto link = robot.addLink("link", {.parentLink = base, .jointName = "joint"});
  auto closure
      = world.addLoopClosure("closure", {.frameA = base, .frameB = link});

  closure.setRuntimePolicy(
      {.enabled = true,
       .kinematics = sx::ClosureKinematicsPolicy::Project,
       .dynamics = sx::ClosureDynamicsPolicy::ResidualOnly});
  EXPECT_THROW(world.enterSimulationMode(), sx::InvalidOperationException);

  closure.setRuntimePolicy(
      {.enabled = false,
       .kinematics = sx::ClosureKinematicsPolicy::Project,
       .dynamics = sx::ClosureDynamicsPolicy::Solve});
  EXPECT_NO_THROW(world.enterSimulationMode());
  EXPECT_NO_THROW(world.sync(sx::WorldSyncStage::Kinematics));
  EXPECT_NO_THROW(world.step());

  closure.setRuntimePolicy(
      {.enabled = true,
       .kinematics = sx::ClosureKinematicsPolicy::Project,
       .dynamics = sx::ClosureDynamicsPolicy::ResidualOnly});
  EXPECT_THROW(
      world.sync(sx::WorldSyncStage::Kinematics),
      sx::InvalidOperationException);

  closure.setRuntimePolicy(
      {.enabled = true,
       .kinematics = sx::ClosureKinematicsPolicy::ResidualOnly,
       .dynamics = sx::ClosureDynamicsPolicy::Solve});
  EXPECT_THROW(world.step(), sx::InvalidOperationException);
}

// Test that rigid bodies can be driven kinematically through the public
// transform API and that attached frames observe fresh transforms immediately.
TEST(World, RigidBodySetTransformRefreshesAttachedFrames)
{
  namespace sx = dart::simulation::experimental;

  sx::World world;
  auto body = world.addRigidBody("body");

  Eigen::Isometry3d sensorOffset = Eigen::Isometry3d::Identity();
  sensorOffset.translate(Eigen::Vector3d(0.0, 1.0, 0.0));
  auto sensor = world.addFixedFrame("sensor", body, sensorOffset);

  world.setGravity(Eigen::Vector3d::Zero());
  world.enterSimulationMode();

  Eigen::Isometry3d drivenPose = Eigen::Isometry3d::Identity();
  drivenPose.translate(Eigen::Vector3d(4.0, 0.0, 0.0));
  body.setTransform(drivenPose);

  EXPECT_TRUE(body.getTransform().isApprox(drivenPose));
  EXPECT_TRUE(sensor.getTransform().isApprox(drivenPose * sensorOffset));

  world.step();

  EXPECT_TRUE(body.getTransform().isApprox(drivenPose));
  EXPECT_TRUE(sensor.getTransform().isApprox(drivenPose * sensorOffset));
}

// Test that rigid bodies expose public velocity state and that stepping uses
// it without requiring direct ECS component access.
TEST(World, RigidBodyVelocityAccessorsDriveStep)
{
  namespace sx = dart::simulation::experimental;

  sx::World world;
  sx::RigidBodyOptions options;
  options.linearVelocity = Eigen::Vector3d(0.1, 0.2, 0.3);
  options.angularVelocity = Eigen::Vector3d(0.4, 0.5, 0.6);
  auto body = world.addRigidBody("body", options);

  EXPECT_TRUE(body.getLinearVelocity().isApprox(options.linearVelocity));
  EXPECT_TRUE(body.getAngularVelocity().isApprox(options.angularVelocity));

  const Eigen::Vector3d linearVelocity(2.0, 0.0, 0.0);
  const Eigen::Vector3d angularVelocity(0.0, 0.0, 1.0);
  body.setLinearVelocity(linearVelocity);
  body.setAngularVelocity(angularVelocity);

  EXPECT_TRUE(body.getLinearVelocity().isApprox(linearVelocity));
  EXPECT_TRUE(body.getAngularVelocity().isApprox(angularVelocity));

  EXPECT_THROW(
      body.setLinearVelocity(
          Eigen::Vector3d(std::numeric_limits<double>::infinity(), 0.0, 0.0)),
      sx::InvalidArgumentException);
  EXPECT_THROW(
      body.setAngularVelocity(
          Eigen::Vector3d(0.0, std::numeric_limits<double>::quiet_NaN(), 0.0)),
      sx::InvalidArgumentException);

  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.5);
  world.enterSimulationMode();
  world.step();

  const Eigen::Vector3d expectedTranslation(1.0, 0.0, 0.0);
  const Eigen::Quaterniond expectedOrientation(
      Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ()));

  EXPECT_TRUE(body.getLinearVelocity().isApprox(linearVelocity));
  EXPECT_TRUE(body.getAngularVelocity().isApprox(angularVelocity));
  EXPECT_TRUE(body.getTranslation().isApprox(expectedTranslation));
  EXPECT_TRUE(
      body.getRotation().isApprox(expectedOrientation.toRotationMatrix()));
}

// Test that simulation operations require simulation mode
TEST(World, UpdateKinematicsRequiresSimulationMode)
{
  namespace sx = dart::simulation::experimental;

  sx::World world;

  EXPECT_THROW(world.updateKinematics(), sx::InvalidArgumentException);
  EXPECT_THROW(
      world.sync(sx::WorldSyncStage::Kinematics), sx::InvalidArgumentException);

  world.enterSimulationMode();
  EXPECT_NO_THROW(world.updateKinematics());
  EXPECT_NO_THROW(world.sync(sx::WorldSyncStage::Kinematics));
  EXPECT_THROW(
      world.sync(static_cast<sx::WorldSyncStage>(999)),
      sx::InvalidArgumentException);
}

// Test that explicit sync refreshes a hierarchy through the compute graph
// without advancing simulation time.
TEST(World, SyncKinematicsRefreshesFrameHierarchy)
{
  namespace sx = dart::simulation::experimental;

  sx::World world;
  auto parent = world.addFreeFrame("parent");

  Eigen::Isometry3d parentTransform = Eigen::Isometry3d::Identity();
  parentTransform.translate(Eigen::Vector3d(1.0, 2.0, 3.0));
  parent.setLocalTransform(parentTransform);

  Eigen::Isometry3d childOffset = Eigen::Isometry3d::Identity();
  childOffset.translate(Eigen::Vector3d(0.0, 4.0, 0.0));
  auto child = world.addFixedFrame("child", parent, childOffset);

  world.enterSimulationMode();

  auto& registry = world.getRegistry();
  EXPECT_FALSE(registry.get<sx::comps::FrameCache>(parent.getEntity())
                   .needTransformUpdate);
  EXPECT_FALSE(registry.get<sx::comps::FrameCache>(child.getEntity())
                   .needTransformUpdate);
  EXPECT_TRUE(child.getTransform().isApprox(parentTransform * childOffset));

  Eigen::Isometry3d updatedParentTransform = Eigen::Isometry3d::Identity();
  updatedParentTransform.translate(Eigen::Vector3d(5.0, 6.0, 7.0));
  parent.setLocalTransform(updatedParentTransform);

  world.sync(sx::WorldSyncStage::Kinematics);

  EXPECT_FALSE(registry.get<sx::comps::FrameCache>(parent.getEntity())
                   .needTransformUpdate);
  EXPECT_FALSE(registry.get<sx::comps::FrameCache>(child.getEntity())
                   .needTransformUpdate);
  EXPECT_TRUE(
      child.getTransform().isApprox(updatedParentTransform * childOffset));
}

// Test that kinematics-only sync can use an injected executor
// without advancing the simulation clock.
TEST(World, SyncKinematicsAcceptsExecutorWithoutAdvancingClock)
{
  namespace sx = dart::simulation::experimental;

  sx::World world;
  auto parent = world.addFreeFrame("parent");

  Eigen::Isometry3d childOffset = Eigen::Isometry3d::Identity();
  childOffset.translate(Eigen::Vector3d(0.0, 4.0, 0.0));
  auto child = world.addFixedFrame("child", parent, childOffset);

  world.enterSimulationMode();

  Eigen::Isometry3d updatedParentTransform = Eigen::Isometry3d::Identity();
  updatedParentTransform.translate(Eigen::Vector3d(5.0, 6.0, 7.0));
  parent.setLocalTransform(updatedParentTransform);

  RecordingExecutor executor;
  world.sync(sx::WorldSyncStage::Kinematics, executor);

  EXPECT_EQ(executor.executeCount, 1u);
  EXPECT_EQ(executor.nodeCount, 2u);
  EXPECT_EQ(executor.edgeCount, 1u);
  EXPECT_DOUBLE_EQ(world.getTime(), 0.0);
  EXPECT_EQ(world.getFrame(), 0u);
  EXPECT_TRUE(
      child.getTransform().isApprox(updatedParentTransform * childOffset));
}

TEST(World, SyncKinematicsRefreshesJointDrivenLinkTransforms)
{
  namespace sx = dart::simulation::experimental;

  sx::World world;
  auto robot = world.addMultiBody("arm");
  auto base = robot.addLink("base");
  auto forearm = robot.addLink(
      "forearm",
      base,
      sx::JointSpec{
          .name = "elbow",
          .type = sx::JointType::Revolute,
          .axis = Eigen::Vector3d::UnitZ(),
      });
  auto slider = robot.addLink(
      "slider",
      base,
      sx::JointSpec{
          .name = "rail",
          .type = sx::JointType::Prismatic,
          .axis = Eigen::Vector3d::UnitX(),
      });

  Eigen::Isometry3d toolOffset = Eigen::Isometry3d::Identity();
  toolOffset.translate(Eigen::Vector3d(1.0, 0.0, 0.0));
  auto tool = world.addFixedFrame("tool", forearm, toolOffset);

  world.enterSimulationMode();

  auto elbow = forearm.getParentJoint();
  elbow.setPosition(Eigen::VectorXd::Constant(1, std::numbers::pi / 2.0));
  auto rail = slider.getParentJoint();
  rail.setPosition(Eigen::VectorXd::Constant(1, 2.0));

  world.sync(sx::WorldSyncStage::Kinematics);

  EXPECT_TRUE(
      tool.getTranslation().isApprox(Eigen::Vector3d(0.0, 1.0, 0.0), 1e-12));
  EXPECT_TRUE(slider.getTranslation().isApprox(Eigen::Vector3d(2.0, 0.0, 0.0)));
  EXPECT_DOUBLE_EQ(world.getTime(), 0.0);
  EXPECT_EQ(world.getFrame(), 0u);
}

TEST(World, LoopClosureResidualUsesSyncedJointTransforms)
{
  namespace sx = dart::simulation::experimental;

  sx::World world;
  auto robot = world.addMultiBody("arm");
  auto base = robot.addLink("base");
  auto forearm = robot.addLink(
      "forearm",
      base,
      sx::JointSpec{
          .name = "elbow",
          .type = sx::JointType::Revolute,
          .axis = Eigen::Vector3d::UnitZ(),
      });

  Eigen::Isometry3d toolOffset = Eigen::Isometry3d::Identity();
  toolOffset.translate(Eigen::Vector3d(1.0, 0.0, 0.0));
  auto tool = world.addFixedFrame("tool", forearm, toolOffset);
  auto ground = world.addRigidBody("ground");
  auto closure = world.addLoopClosure(
      "tool_to_ground",
      {.frameA = tool,
       .frameB = ground,
       .family = sx::LoopClosureFamily::Point});

  world.enterSimulationMode();
  forearm.getParentJoint().setPosition(
      Eigen::VectorXd::Constant(1, std::numbers::pi / 2.0));

  world.sync(sx::WorldSyncStage::Kinematics);

  const auto residual = closure.computeResidual();
  ASSERT_EQ(residual.value.size(), 3);
  EXPECT_TRUE(residual.value.isApprox(Eigen::Vector3d(0.0, 1.0, 0.0), 1e-12));
}

// Test that ordinary frame reads stay fresh after mutating an ancestor frame,
// without requiring users to know cache invalidation details.
TEST(World, FrameReadsRefreshDescendantsAfterParentLocalTransformChange)
{
  namespace sx = dart::simulation::experimental;

  sx::World world;
  auto parent = world.addFreeFrame("parent");

  Eigen::Isometry3d childOffset = Eigen::Isometry3d::Identity();
  childOffset.translate(Eigen::Vector3d(0.0, 2.0, 0.0));
  auto child = world.addFixedFrame("child", parent, childOffset);

  world.enterSimulationMode();
  EXPECT_TRUE(child.getTransform().isApprox(childOffset));

  Eigen::Isometry3d updatedParentTransform = Eigen::Isometry3d::Identity();
  updatedParentTransform.translate(Eigen::Vector3d(3.0, 0.0, 0.0));
  parent.setLocalTransform(updatedParentTransform);

  auto& registry = world.getRegistry();
  EXPECT_TRUE(registry.get<sx::comps::FrameCache>(parent.getEntity())
                  .needTransformUpdate);
  EXPECT_TRUE(registry.get<sx::comps::FrameCache>(child.getEntity())
                  .needTransformUpdate);
  EXPECT_TRUE(
      child.getTransform().isApprox(updatedParentTransform * childOffset));
}

// Test that the experimental step path enters simulation mode and refreshes
// kinematics using the default sequential graph executor.
TEST(World, StepRefreshesFrameHierarchy)
{
  namespace sx = dart::simulation::experimental;

  sx::World world;
  auto parent = world.addFreeFrame("parent");

  Eigen::Isometry3d parentTransform = Eigen::Isometry3d::Identity();
  parentTransform.translate(Eigen::Vector3d(1.0, 0.0, 0.0));
  parent.setLocalTransform(parentTransform);

  Eigen::Isometry3d childOffset = Eigen::Isometry3d::Identity();
  childOffset.translate(Eigen::Vector3d(0.0, 2.0, 0.0));
  auto child = world.addFixedFrame("child", parent, childOffset);

  Eigen::Isometry3d updatedParentTransform = Eigen::Isometry3d::Identity();
  updatedParentTransform.translate(Eigen::Vector3d(3.0, 0.0, 0.0));
  parent.setLocalTransform(updatedParentTransform);

  world.step();

  EXPECT_TRUE(world.isSimulationMode());

  auto& registry = world.getRegistry();
  EXPECT_FALSE(registry.get<sx::comps::FrameCache>(parent.getEntity())
                   .needTransformUpdate);
  EXPECT_FALSE(registry.get<sx::comps::FrameCache>(child.getEntity())
                   .needTransformUpdate);
  EXPECT_TRUE(
      child.getTransform().isApprox(updatedParentTransform * childOffset));
}

// Test that the experimental step path accepts alternate graph executors.
TEST(World, StepAcceptsParallelExecutor)
{
  namespace sx = dart::simulation::experimental;
  namespace compute = dart::simulation::experimental::compute;

  sx::World world;
  auto parent = world.addFreeFrame("parent");

  Eigen::Isometry3d childOffset = Eigen::Isometry3d::Identity();
  childOffset.translate(Eigen::Vector3d(0.0, 1.0, 0.0));
  auto child = world.addFixedFrame("child", parent, childOffset);

  world.enterSimulationMode();

  Eigen::Isometry3d updatedParentTransform = Eigen::Isometry3d::Identity();
  updatedParentTransform.translate(Eigen::Vector3d(2.0, 0.0, 0.0));
  parent.setLocalTransform(updatedParentTransform);

  compute::ParallelExecutor executor(1);
  world.step(executor);

  EXPECT_TRUE(
      child.getTransform().isApprox(updatedParentTransform * childOffset));
}

// Test that RigidBodyOptions seed the public rigid-body state used by the graph
// backed step path.
TEST(World, RigidBodyOptionsInitializePublicState)
{
  namespace sx = dart::simulation::experimental;

  sx::World world;
  sx::RigidBodyOptions options;
  options.mass = 2.0;
  options.inertia = Eigen::Vector3d(1.0, 2.0, 3.0).asDiagonal();
  options.position = Eigen::Vector3d(1.0, 2.0, 3.0);
  options.orientation
      = Eigen::Quaterniond(Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ()));
  options.linearVelocity = Eigen::Vector3d(4.0, 5.0, 6.0);
  options.angularVelocity = Eigen::Vector3d(0.1, 0.2, 0.3);

  auto body = world.addRigidBody("body", options);

  EXPECT_TRUE(body.getTranslation().isApprox(options.position));
  EXPECT_TRUE(body.getQuaternion().isApprox(options.orientation));
  EXPECT_TRUE(body.getLinearVelocity().isApprox(options.linearVelocity));
  EXPECT_TRUE(body.getAngularVelocity().isApprox(options.angularVelocity));
  EXPECT_DOUBLE_EQ(body.getMass(), options.mass);
  EXPECT_TRUE(body.getInertia().isApprox(options.inertia));

  const double updatedMass = 5.0;
  const Eigen::Matrix3d updatedInertia
      = Eigen::Vector3d(3.0, 4.0, 5.0).asDiagonal();
  body.setMass(updatedMass);
  body.setInertia(updatedInertia);

  EXPECT_DOUBLE_EQ(body.getMass(), updatedMass);
  EXPECT_TRUE(body.getInertia().isApprox(updatedInertia));

  EXPECT_TRUE(body.getForce().isApprox(Eigen::Vector3d::Zero()));
  body.setForce(Eigen::Vector3d(0.0, 2.0, 0.0));
  body.applyForce(Eigen::Vector3d(0.0, 3.0, 0.0));
  EXPECT_TRUE(body.getForce().isApprox(Eigen::Vector3d(0.0, 5.0, 0.0)));
  body.clearForce();
  EXPECT_TRUE(body.getForce().isApprox(Eigen::Vector3d::Zero()));

  EXPECT_TRUE(body.getTorque().isApprox(Eigen::Vector3d::Zero()));
  body.setTorque(Eigen::Vector3d(1.0, 0.0, 0.0));
  body.applyTorque(Eigen::Vector3d(2.0, 0.0, 0.0));
  EXPECT_TRUE(body.getTorque().isApprox(Eigen::Vector3d(3.0, 0.0, 0.0)));
  body.clearTorque();
  EXPECT_TRUE(body.getTorque().isApprox(Eigen::Vector3d::Zero()));

  world.enterSimulationMode();
  EXPECT_TRUE(body.getTransform().translation().isApprox(options.position));
  EXPECT_TRUE(
      body.getRotation().isApprox(options.orientation.toRotationMatrix()));
}

// Test that invalid rigid-body construction data is rejected before it reaches
// ECS dynamics components.
TEST(World, RigidBodyOptionsRejectInvalidValues)
{
  namespace sx = dart::simulation::experimental;

  const auto infinity = std::numeric_limits<double>::infinity();
  const auto quietNaN = std::numeric_limits<double>::quiet_NaN();

  auto expectInvalid = [](const sx::RigidBodyOptions& options) {
    sx::World world;
    EXPECT_THROW(
        (void)world.addRigidBody("body", options),
        sx::InvalidArgumentException);
  };

  sx::RigidBodyOptions options;

  options.mass = 0.0;
  expectInvalid(options);

  options = sx::RigidBodyOptions{};
  options.mass = infinity;
  expectInvalid(options);

  options = sx::RigidBodyOptions{};
  options.inertia = Eigen::Vector3d(1.0, -1.0, 1.0).asDiagonal();
  expectInvalid(options);

  options = sx::RigidBodyOptions{};
  options.inertia(0, 1) = 0.1;
  expectInvalid(options);

  options = sx::RigidBodyOptions{};
  options.position.x() = infinity;
  expectInvalid(options);

  options = sx::RigidBodyOptions{};
  options.orientation = Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0);
  expectInvalid(options);

  options = sx::RigidBodyOptions{};
  options.linearVelocity.x() = quietNaN;
  expectInvalid(options);

  options = sx::RigidBodyOptions{};
  options.angularVelocity.x() = infinity;
  expectInvalid(options);

  sx::World world;
  auto body = world.addRigidBody("body");
  EXPECT_THROW(body.setMass(0.0), sx::InvalidArgumentException);
  EXPECT_THROW(body.setMass(infinity), sx::InvalidArgumentException);

  EXPECT_THROW(
      body.setInertia(Eigen::Vector3d(1.0, -1.0, 1.0).asDiagonal()),
      sx::InvalidArgumentException);

  Eigen::Matrix3d asymmetricInertia = Eigen::Matrix3d::Identity();
  asymmetricInertia(0, 1) = 0.1;
  EXPECT_THROW(
      body.setInertia(asymmetricInertia), sx::InvalidArgumentException);

  EXPECT_THROW(
      body.setForce(Eigen::Vector3d(infinity, 0.0, 0.0)),
      sx::InvalidArgumentException);
  EXPECT_THROW(
      body.applyForce(Eigen::Vector3d(0.0, quietNaN, 0.0)),
      sx::InvalidArgumentException);
  EXPECT_THROW(
      body.setTorque(Eigen::Vector3d(0.0, infinity, 0.0)),
      sx::InvalidArgumentException);
  EXPECT_THROW(
      body.applyTorque(Eigen::Vector3d(0.0, 0.0, quietNaN)),
      sx::InvalidArgumentException);
}

// Test that World::step() runs the rigid-body integration graph before the
// kinematics cache refresh and then advances the simulation clock.
TEST(World, StepIntegratesRigidBodyStateAndAdvancesClock)
{
  namespace sx = dart::simulation::experimental;

  sx::World world;
  sx::RigidBodyOptions options;
  options.mass = 2.0;
  options.position = Eigen::Vector3d(1.0, 2.0, 3.0);
  options.linearVelocity = Eigen::Vector3d(2.0, 0.0, 0.0);

  auto body = world.addRigidBody("body", options);
  body.setForce(Eigen::Vector3d(0.0, 4.0, 0.0));

  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.5);
  world.enterSimulationMode();

  world.step();

  const auto expectedVelocity = Eigen::Vector3d(2.0, 1.0, 0.0);
  const auto expectedPosition = Eigen::Vector3d(2.0, 2.5, 3.0);

  EXPECT_TRUE(body.getLinearVelocity().isApprox(expectedVelocity));
  EXPECT_TRUE(body.getTranslation().isApprox(expectedPosition));
  EXPECT_DOUBLE_EQ(world.getTime(), 0.5);
  EXPECT_EQ(world.getFrame(), 1u);
}

// Test that World::step(count) provides the common repeated-step API.
TEST(World, StepCountAdvancesClockAndFrame)
{
  namespace sx = dart::simulation::experimental;

  sx::World world;
  world.setTimeStep(0.25);

  world.step(0);
  EXPECT_FALSE(world.isSimulationMode());

  world.step(4);

  EXPECT_TRUE(world.isSimulationMode());
  EXPECT_DOUBLE_EQ(world.getTime(), 1.0);
  EXPECT_EQ(world.getFrame(), 4u);
}

// Test that repeated stepping can use an injected executor and reuse the
// default pipeline state across steps.
TEST(World, StepCountAcceptsExecutor)
{
  namespace sx = dart::simulation::experimental;

  sx::World world;
  sx::RigidBodyOptions options;
  options.linearVelocity = Eigen::Vector3d(1.0, 0.0, 0.0);
  auto body = world.addRigidBody("body", options);

  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.25);

  RecordingExecutor executor;
  world.step(0, executor);
  EXPECT_FALSE(world.isSimulationMode());
  EXPECT_EQ(executor.executeCount, 0u);

  world.step(2, executor);

  EXPECT_TRUE(world.isSimulationMode());
  EXPECT_DOUBLE_EQ(world.getTime(), 0.5);
  EXPECT_EQ(world.getFrame(), 2u);
  EXPECT_TRUE(body.getTranslation().isApprox(Eigen::Vector3d(0.5, 0.0, 0.0)));
  EXPECT_GT(executor.executeCount, 0u);
}

// Test that torque integration uses the body-frame inertia tensor and updates
// angular velocity before advancing orientation.
TEST(World, StepIntegratesRigidBodyTorque)
{
  namespace sx = dart::simulation::experimental;

  sx::World world;
  sx::RigidBodyOptions options;
  constexpr double kHalfPi = 1.57079632679489661923;
  options.inertia = Eigen::Vector3d(2.0, 4.0, 8.0).asDiagonal();
  options.orientation = Eigen::Quaterniond(
      Eigen::AngleAxisd(kHalfPi, Eigen::Vector3d::UnitZ()));

  auto body = world.addRigidBody("body", options);
  body.setTorque(Eigen::Vector3d(8.0, 0.0, 0.0));

  world.setTimeStep(0.5);
  world.enterSimulationMode();

  world.step();

  const auto expectedAngularVelocity = Eigen::Vector3d(1.0, 0.0, 0.0);
  const Eigen::Quaterniond expectedOrientation
      = Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitX()) * options.orientation;

  EXPECT_TRUE(body.getAngularVelocity().isApprox(expectedAngularVelocity));
  EXPECT_TRUE(body.getQuaternion().isApprox(expectedOrientation));
  EXPECT_TRUE(
      body.getRotation().isApprox(expectedOrientation.toRotationMatrix()));
}

// Test that World gravity accelerates a free rigid body during stepping and is
// independent of body mass.
TEST(World, StepAppliesGravityToFreeBody)
{
  namespace sx = dart::simulation::experimental;

  sx::World world;
  EXPECT_TRUE(world.getGravity().isApprox(Eigen::Vector3d(0.0, 0.0, -9.81)));

  sx::RigidBodyOptions options;
  options.mass = 2.0;
  options.position = Eigen::Vector3d(0.0, 0.0, 10.0);
  auto body = world.addRigidBody("body", options);

  world.setTimeStep(0.1);
  world.enterSimulationMode();
  world.step();

  const Eigen::Vector3d gravity(0.0, 0.0, -9.81);
  const double dt = 0.1;
  const Eigen::Vector3d expectedVelocity = gravity * dt;
  const Eigen::Vector3d expectedPosition
      = Eigen::Vector3d(0.0, 0.0, 10.0) + expectedVelocity * dt;

  EXPECT_TRUE(body.getLinearVelocity().isApprox(expectedVelocity));
  EXPECT_TRUE(body.getTranslation().isApprox(expectedPosition));
}

// Test that World gravity is configurable, validated, and reset by clear().
TEST(World, GravityIsConfigurableAndValidated)
{
  namespace sx = dart::simulation::experimental;

  sx::World world;
  const Eigen::Vector3d customGravity(0.0, -3.0, 0.0);
  world.setGravity(customGravity);
  EXPECT_TRUE(world.getGravity().isApprox(customGravity));

  EXPECT_THROW(
      world.setGravity(
          Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0)),
      sx::InvalidArgumentException);
  EXPECT_THROW(
      world.setGravity(
          Eigen::Vector3d(0.0, std::numeric_limits<double>::infinity(), 0.0)),
      sx::InvalidArgumentException);

  world.clear();
  EXPECT_TRUE(world.getGravity().isApprox(Eigen::Vector3d(0.0, 0.0, -9.81)));
}

// Test that external force/torque accumulators are per-step applied loads that
// are consumed and then cleared by each step (legacy DART 6 convention).
TEST(World, StepClearsExternalForceAndTorque)
{
  namespace sx = dart::simulation::experimental;

  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());

  sx::RigidBodyOptions options;
  options.mass = 2.0;
  auto body = world.addRigidBody("body", options);
  body.setForce(Eigen::Vector3d(4.0, 0.0, 0.0));

  world.setTimeStep(0.5);
  world.enterSimulationMode();
  world.step();

  // Force consumed by the step (v += F/m * dt = 2 * 0.5 = 1) then cleared.
  const Eigen::Vector3d velocityAfterFirstStep(1.0, 0.0, 0.0);
  EXPECT_TRUE(body.getLinearVelocity().isApprox(velocityAfterFirstStep));
  EXPECT_TRUE(body.getForce().isApprox(Eigen::Vector3d::Zero()));
  EXPECT_TRUE(body.getTorque().isApprox(Eigen::Vector3d::Zero()));

  // A second step without re-applying the force must not accelerate further.
  world.step();
  EXPECT_TRUE(body.getLinearVelocity().isApprox(velocityAfterFirstStep));
}

// Test derived rigid-body dynamic quantities (momentum and energy).
TEST(World, RigidBodyDynamicQuantities)
{
  namespace sx = dart::simulation::experimental;

  sx::World world; // default gravity (0, 0, -9.81)

  sx::RigidBodyOptions options;
  options.mass = 2.0;
  options.inertia = Eigen::Vector3d(2.0, 4.0, 8.0).asDiagonal();
  options.position = Eigen::Vector3d(0.0, 0.0, 5.0);
  options.linearVelocity = Eigen::Vector3d(3.0, 0.0, 0.0);
  options.angularVelocity = Eigen::Vector3d(1.0, 0.0, 0.0);
  auto body = world.addRigidBody("body", options);

  EXPECT_TRUE(
      body.getLinearMomentum().isApprox(Eigen::Vector3d(6.0, 0.0, 0.0)));
  EXPECT_TRUE(
      body.getAngularMomentum().isApprox(Eigen::Vector3d(2.0, 0.0, 0.0)));
  // KE = 0.5 * m * |v|^2 + 0.5 * w . (I w) = 0.5*2*9 + 0.5*1*2 = 10.
  EXPECT_DOUBLE_EQ(body.getKineticEnergy(), 10.0);
  // PE = -m * gravity . position = -2 * (-9.81 * 5) = 98.1.
  EXPECT_NEAR(body.getPotentialEnergy(), 98.1, 1e-9);
}

// Test that articulated-body forward dynamics produces the analytical gravity
// acceleration for a horizontal single revolute pendulum.
TEST(World, MultiBodyRevolutePendulumGravityAcceleration)
{
  namespace sx = dart::simulation::experimental;

  sx::World world; // default gravity (0, 0, -9.81)

  auto robot = world.addMultiBody("pendulum");
  auto base = robot.addLink("base");

  // Child link: revolute about Y at the base origin; center of mass offset L
  // along +X so the link is horizontal at q = 0.
  const double length = 1.5;
  Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
  offset.translation() = Eigen::Vector3d(length, 0.0, 0.0);
  sx::JointSpec spec;
  spec.name = "hinge";
  spec.type = sx::JointType::Revolute;
  spec.axis = Eigen::Vector3d::UnitY();
  spec.transformFromParent = offset;
  auto bob = robot.addLink("bob", base, spec);

  const double mass = 2.0;
  const double inertiaYy = 0.2;
  bob.setMass(mass);
  bob.setInertia(Eigen::Vector3d(0.1, inertiaYy, 0.3).asDiagonal());

  world.setTimeStep(0.001);
  world.enterSimulationMode();
  world.step();

  // I_pivot = I_com + m L^2 (parallel axis); qddot = m g L / I_pivot.
  const double expected
      = 9.81 * mass * length / (inertiaYy + mass * length * length);
  auto joint = bob.getParentJoint();
  EXPECT_NEAR(joint.getAcceleration()[0], expected, 1e-9);
}

// Test that the public generalized mass matrix, gravity forces, and Coriolis
// forces match analytical values for a single revolute pendulum.
TEST(World, MultiBodyMassMatrixAndForcesSinglePendulum)
{
  namespace sx = dart::simulation::experimental;

  sx::World world; // default gravity (0, 0, -9.81)

  auto robot = world.addMultiBody("pendulum");
  auto base = robot.addLink("base");

  const double length = 1.5;
  Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
  offset.translation() = Eigen::Vector3d(length, 0.0, 0.0);
  sx::JointSpec spec;
  spec.name = "hinge";
  spec.type = sx::JointType::Revolute;
  spec.axis = Eigen::Vector3d::UnitY();
  spec.transformFromParent = offset;
  auto bob = robot.addLink("bob", base, spec);

  const double mass = 2.0;
  const double inertiaYy = 0.2;
  bob.setMass(mass);
  bob.setInertia(Eigen::Vector3d(0.1, inertiaYy, 0.3).asDiagonal());

  // A single revolute DOF has no Coriolis/centrifugal term (M is constant in
  // q), so a nonzero velocity must still leave the Coriolis force at zero.
  auto joint = bob.getParentJoint();
  joint.setVelocity(Eigen::VectorXd::Constant(1, 2.0));

  world.enterSimulationMode();

  // M = I_axis + m L^2 about the pivot (parallel-axis theorem), 1x1.
  const Eigen::MatrixXd massMatrix = robot.getMassMatrix();
  ASSERT_EQ(massMatrix.rows(), 1);
  ASSERT_EQ(massMatrix.cols(), 1);
  const double expectedMass = inertiaYy + mass * length * length;
  EXPECT_NEAR(massMatrix(0, 0), expectedMass, 1e-12);

  const Eigen::MatrixXd inverseMass = robot.getInverseMassMatrix();
  ASSERT_EQ(inverseMass.rows(), 1);
  ASSERT_EQ(inverseMass.cols(), 1);
  EXPECT_NEAR(inverseMass(0, 0), 1.0 / expectedMass, 1e-12);

  // Horizontal pendulum at q = 0: the gravity generalized force is -m g L, so
  // the unactuated joint accelerates by +m g L / M (the forward-dynamics sign).
  const Eigen::VectorXd gravityForces = robot.getGravityForces();
  ASSERT_EQ(gravityForces.size(), 1);
  EXPECT_NEAR(gravityForces[0], -mass * 9.81 * length, 1e-12);

  const Eigen::VectorXd coriolis = robot.getCoriolisForces();
  ASSERT_EQ(coriolis.size(), 1);
  EXPECT_NEAR(coriolis[0], 0.0, 1e-12);

  const Eigen::VectorXd combined = robot.getCoriolisAndGravityForces();
  ASSERT_EQ(combined.size(), 1);
  EXPECT_NEAR(combined[0], gravityForces[0], 1e-12);
}

// Test that the public mass matrix and bias forces satisfy the joint-space
// equation of motion M qddot + C + g = tau for a 2-DOF chain. This validates
// the decomposition for a multi-DOF system without hand-computing M.
TEST(World, MultiBodyEquationOfMotionConsistency)
{
  namespace sx = dart::simulation::experimental;

  sx::World world; // default gravity (0, 0, -9.81)

  auto robot = world.addMultiBody("double_pendulum");
  auto base = robot.addLink("base");

  const double l1 = 0.7;
  const double l2 = 0.6;
  Eigen::Isometry3d offset1 = Eigen::Isometry3d::Identity();
  offset1.translation() = Eigen::Vector3d(l1, 0.0, 0.0);
  Eigen::Isometry3d offset2 = Eigen::Isometry3d::Identity();
  offset2.translation() = Eigen::Vector3d(l2, 0.0, 0.0);

  sx::JointSpec spec1;
  spec1.name = "j1";
  spec1.type = sx::JointType::Revolute;
  spec1.axis = Eigen::Vector3d::UnitY();
  spec1.transformFromParent = offset1;
  auto link1 = robot.addLink("link1", base, spec1);
  link1.setMass(1.5);
  link1.setInertia(Eigen::Vector3d(0.05, 0.08, 0.05).asDiagonal());

  sx::JointSpec spec2;
  spec2.name = "j2";
  spec2.type = sx::JointType::Revolute;
  spec2.axis = Eigen::Vector3d::UnitY();
  spec2.transformFromParent = offset2;
  auto link2 = robot.addLink("link2", link1, spec2);
  link2.setMass(1.0);
  link2.setInertia(Eigen::Vector3d(0.04, 0.06, 0.04).asDiagonal());

  auto joint1 = link1.getParentJoint();
  auto joint2 = link2.getParentJoint();

  joint1.setPosition(Eigen::VectorXd::Constant(1, 0.3));
  joint1.setVelocity(Eigen::VectorXd::Constant(1, 1.1));
  joint1.setForce(Eigen::VectorXd::Constant(1, 2.0));
  joint2.setPosition(Eigen::VectorXd::Constant(1, -0.5));
  joint2.setVelocity(Eigen::VectorXd::Constant(1, 0.7));
  joint2.setForce(Eigen::VectorXd::Constant(1, -1.5));

  Eigen::VectorXd tau(2);
  tau << 2.0, -1.5;

  world.setTimeStep(1e-3);
  world.enterSimulationMode();

  // Read the dynamics terms at the current (pre-step) state.
  const Eigen::MatrixXd massMatrix = robot.getMassMatrix();
  const Eigen::VectorXd coriolis = robot.getCoriolisForces();
  const Eigen::VectorXd gravityForces = robot.getGravityForces();

  ASSERT_EQ(massMatrix.rows(), 2);
  ASSERT_EQ(massMatrix.cols(), 2);
  EXPECT_NEAR((massMatrix - massMatrix.transpose()).norm(), 0.0, 1e-12);

  // step() computes qddot from the same pre-step state, so the equation of
  // motion must hold exactly (up to solver round-off).
  world.step();
  Eigen::VectorXd qddot(2);
  qddot << joint1.getAcceleration()[0], joint2.getAcceleration()[0];

  const Eigen::VectorXd residual
      = massMatrix * qddot + coriolis + gravityForces - tau;
  EXPECT_NEAR(residual.norm(), 0.0, 1e-9);
}

// Test that the dynamics accessors return empty results for a multibody with no
// movable degrees of freedom.
TEST(World, MultiBodyDynamicsAccessorsNoDOF)
{
  namespace sx = dart::simulation::experimental;

  sx::World world;

  auto robot = world.addMultiBody("static_chain");
  auto base = robot.addLink("base");
  sx::JointSpec spec;
  spec.name = "weld";
  spec.type = sx::JointType::Fixed;
  (void)robot.addLink("welded", base, spec);

  world.enterSimulationMode();

  EXPECT_EQ(robot.getDOFCount(), 0u);
  EXPECT_EQ(robot.getMassMatrix().size(), 0);
  EXPECT_EQ(robot.getInverseMassMatrix().size(), 0);
  EXPECT_EQ(robot.getCoriolisForces().size(), 0);
  EXPECT_EQ(robot.getGravityForces().size(), 0);
  EXPECT_EQ(robot.getCoriolisAndGravityForces().size(), 0);
}

// Test that a prismatic joint aligned with gravity free-falls at g.
TEST(World, MultiBodyPrismaticFreeFall)
{
  namespace sx = dart::simulation::experimental;

  sx::World world; // default gravity (0, 0, -9.81)

  auto robot = world.addMultiBody("slider");
  auto base = robot.addLink("base");
  sx::JointSpec spec;
  spec.name = "rail";
  spec.type = sx::JointType::Prismatic;
  spec.axis = Eigen::Vector3d::UnitZ();
  auto carriage = robot.addLink("carriage", base, spec);
  carriage.setMass(3.0);

  const double dt = 0.01;
  world.setTimeStep(dt);
  world.enterSimulationMode();
  world.step();

  auto joint = carriage.getParentJoint();
  EXPECT_NEAR(joint.getAcceleration()[0], -9.81, 1e-9);
  EXPECT_NEAR(joint.getVelocity()[0], -9.81 * dt, 1e-12);
  EXPECT_NEAR(joint.getPosition()[0], -9.81 * dt * dt, 1e-12);
}

// Test that the pendulum integrator conserves mechanical energy over a swing
// (a sign or scale error in the dynamics would inject energy and diverge).
TEST(World, MultiBodyPendulumConservesEnergy)
{
  namespace sx = dart::simulation::experimental;

  sx::World world; // default gravity (0, 0, -9.81)
  const Eigen::Vector3d gravity(0.0, 0.0, -9.81);

  auto robot = world.addMultiBody("pendulum");
  auto base = robot.addLink("base");

  const double length = 1.0;
  Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
  offset.translation() = Eigen::Vector3d(length, 0.0, 0.0);
  sx::JointSpec spec;
  spec.name = "hinge";
  spec.type = sx::JointType::Revolute;
  spec.axis = Eigen::Vector3d::UnitY();
  spec.transformFromParent = offset;
  auto bob = robot.addLink("bob", base, spec);

  const double mass = 1.0;
  const Eigen::Matrix3d inertia
      = Eigen::Vector3d(0.05, 0.05, 0.05).asDiagonal();
  bob.setMass(mass);
  bob.setInertia(inertia);

  auto joint = bob.getParentJoint();

  // Released from horizontal at rest; total energy starts at zero.
  const auto energy = [&]() {
    const double qd = joint.getVelocity()[0];
    const Eigen::Isometry3d worldTransform = bob.getTransform();
    const Eigen::Vector3d com = worldTransform.translation();
    const Eigen::Vector3d omega = qd * Eigen::Vector3d::UnitY();
    const Eigen::Vector3d linear = omega.cross(com);
    const Eigen::Matrix3d rotation = worldTransform.linear();
    const Eigen::Matrix3d worldInertia
        = rotation * inertia * rotation.transpose();
    const double kinetic = 0.5 * mass * linear.squaredNorm()
                           + 0.5 * omega.dot(worldInertia * omega);
    const double potential = -mass * gravity.dot(com);
    return kinetic + potential;
  };

  world.setTimeStep(1e-4);
  world.enterSimulationMode();

  const double initialEnergy = energy();
  EXPECT_NEAR(initialEnergy, 0.0, 1e-12);

  // Roughly a quarter period: enough time to swing from horizontal toward the
  // bottom (stable equilibrium at q = pi/2).
  double maxAngle = 0.0;
  for (int i = 0; i < 8000; ++i) {
    world.step();
    EXPECT_NEAR(energy(), initialEnergy, 5e-2);
    maxAngle = std::max(maxAngle, std::abs(joint.getPosition()[0]));
  }

  // The pendulum should have swung down close to the bottom (q ~ pi/2).
  EXPECT_GT(maxAngle, 1.4);
}

// Test that joint spring stiffness and damping contribute passive generalized
// forces in the articulated-body dynamics.
TEST(World, MultiBodyJointSpringAndDamping)
{
  namespace sx = dart::simulation::experimental;

  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());

  auto robot = world.addMultiBody("slider");
  auto base = robot.addLink("base");
  sx::JointSpec spec;
  spec.name = "rail";
  spec.type = sx::JointType::Prismatic;
  spec.axis = Eigen::Vector3d::UnitZ();
  auto carriage = robot.addLink("carriage", base, spec);
  const double mass = 2.0;
  carriage.setMass(mass);

  auto joint = carriage.getParentJoint();
  const double stiffness = 10.0;
  const double damping = 3.0;
  joint.setSpringStiffness(Eigen::VectorXd::Constant(1, stiffness));
  joint.setDampingCoefficient(Eigen::VectorXd::Constant(1, damping));
  joint.setRestPosition(Eigen::VectorXd::Zero(1));

  EXPECT_THROW(
      joint.setSpringStiffness(Eigen::VectorXd::Constant(1, -1.0)),
      sx::InvalidArgumentException);
  EXPECT_THROW(
      joint.setDampingCoefficient(Eigen::VectorXd::Constant(1, -1.0)),
      sx::InvalidArgumentException);

  const double position = 0.5;
  const double velocity = 2.0;
  joint.setPosition(Eigen::VectorXd::Constant(1, position));
  joint.setVelocity(Eigen::VectorXd::Constant(1, velocity));

  world.setTimeStep(0.001);
  world.enterSimulationMode();
  world.step();

  // qddot = (-stiffness * position - damping * velocity) / mass.
  const double expected = (-stiffness * position - damping * velocity) / mass;
  EXPECT_NEAR(joint.getAcceleration()[0], expected, 1e-9);
}

// Test that a joint position limit acts as a hard stop in the dynamics.
TEST(World, MultiBodyJointPositionLimit)
{
  namespace sx = dart::simulation::experimental;

  sx::World world; // default gravity (0, 0, -9.81)

  auto robot = world.addMultiBody("pendulum");
  auto base = robot.addLink("base");
  Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
  offset.translation() = Eigen::Vector3d(1.0, 0.0, 0.0);
  sx::JointSpec spec;
  spec.name = "hinge";
  spec.type = sx::JointType::Revolute;
  spec.axis = Eigen::Vector3d::UnitY();
  spec.transformFromParent = offset;
  auto bob = robot.addLink("bob", base, spec);
  bob.setMass(1.0);
  bob.setInertia(Eigen::Vector3d(0.05, 0.05, 0.05).asDiagonal());

  auto joint = bob.getParentJoint();
  EXPECT_TRUE(std::isinf(joint.getPositionUpperLimits()[0]));
  EXPECT_TRUE(std::isinf(joint.getPositionLowerLimits()[0]));

  const double upperLimit = 0.5;
  joint.setPositionLimits(
      Eigen::VectorXd::Constant(1, -std::numeric_limits<double>::infinity()),
      Eigen::VectorXd::Constant(1, upperLimit));
  EXPECT_DOUBLE_EQ(joint.getPositionUpperLimits()[0], upperLimit);

  EXPECT_THROW(
      joint.setPositionLimits(
          Eigen::VectorXd::Constant(1, 1.0), Eigen::VectorXd::Constant(1, 0.0)),
      sx::InvalidArgumentException);
  EXPECT_THROW(
      joint.setPositionLimits(
          Eigen::VectorXd::Constant(2, 0.0), Eigen::VectorXd::Constant(2, 1.0)),
      sx::InvalidArgumentException);

  world.setTimeStep(0.005);
  world.enterSimulationMode();
  world.step(400);

  // Gravity drives the joint toward +pi/2, but it must stop at the upper limit.
  EXPECT_NEAR(joint.getPosition()[0], upperLimit, 1e-9);
  EXPECT_NEAR(joint.getVelocity()[0], 0.0, 1e-9);
}

// Test that joint effort limits clamp the commanded actuation force used by the
// articulated-body forward dynamics.
TEST(World, MultiBodyJointEffortLimit)
{
  namespace sx = dart::simulation::experimental;

  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());

  auto robot = world.addMultiBody("slider");
  auto base = robot.addLink("base");
  sx::JointSpec spec;
  spec.name = "rail";
  spec.type = sx::JointType::Prismatic;
  spec.axis = Eigen::Vector3d::UnitZ();
  auto carriage = robot.addLink("carriage", base, spec);
  const double mass = 2.0;
  carriage.setMass(mass);

  auto joint = carriage.getParentJoint();
  EXPECT_TRUE(std::isinf(joint.getEffortUpperLimits()[0]));
  EXPECT_TRUE(std::isinf(joint.getEffortLowerLimits()[0]));

  const double effortLimit = 10.0;
  joint.setEffortLimits(
      Eigen::VectorXd::Constant(1, -effortLimit),
      Eigen::VectorXd::Constant(1, effortLimit));
  joint.setForce(Eigen::VectorXd::Constant(1, 100.0)); // far above the limit

  EXPECT_THROW(
      joint.setEffortLimits(
          Eigen::VectorXd::Constant(1, 1.0), Eigen::VectorXd::Constant(1, 0.0)),
      sx::InvalidArgumentException);
  EXPECT_THROW(
      joint.setEffortLimits(
          Eigen::VectorXd::Constant(2, 0.0), Eigen::VectorXd::Constant(2, 1.0)),
      sx::InvalidArgumentException);

  world.setTimeStep(0.01);
  world.enterSimulationMode();
  world.step();

  // The applied effort is clamped to the limit, so qddot = effortLimit / mass.
  EXPECT_NEAR(joint.getAcceleration()[0], effortLimit / mass, 1e-12);
}

// Test that joint velocity limits clamp the generalized velocity each step.
TEST(World, MultiBodyJointVelocityLimit)
{
  namespace sx = dart::simulation::experimental;

  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());

  auto robot = world.addMultiBody("slider");
  auto base = robot.addLink("base");
  sx::JointSpec spec;
  spec.name = "rail";
  spec.type = sx::JointType::Prismatic;
  spec.axis = Eigen::Vector3d::UnitZ();
  auto carriage = robot.addLink("carriage", base, spec);
  carriage.setMass(2.0);

  auto joint = carriage.getParentJoint();
  EXPECT_TRUE(std::isinf(joint.getVelocityUpperLimits()[0]));
  EXPECT_TRUE(std::isinf(joint.getVelocityLowerLimits()[0]));

  const double velocityLimit = 0.1;
  joint.setVelocityLimits(
      Eigen::VectorXd::Constant(1, -velocityLimit),
      Eigen::VectorXd::Constant(1, velocityLimit));
  joint.setForce(Eigen::VectorXd::Constant(1, 10.0)); // accelerates the slider

  EXPECT_THROW(
      joint.setVelocityLimits(
          Eigen::VectorXd::Constant(1, 1.0), Eigen::VectorXd::Constant(1, 0.0)),
      sx::InvalidArgumentException);

  world.setTimeStep(0.01);
  world.enterSimulationMode();

  for (int i = 0; i < 200; ++i) {
    world.step();
    EXPECT_LE(joint.getVelocity()[0], velocityLimit + 1e-12);
  }

  // Under continued forcing the velocity saturates exactly at the limit.
  EXPECT_NEAR(joint.getVelocity()[0], velocityLimit, 1e-12);
}

// Test that World::collide() reports contacts between overlapping collision
// shapes and reports none once the shapes are separated.
TEST(World, CollisionQueryReportsContacts)
{
  namespace sx = dart::simulation::experimental;

  sx::World world;

  sx::RigidBodyOptions optionsA;
  optionsA.position = Eigen::Vector3d(0.0, 0.0, 0.0);
  auto bodyA = world.addRigidBody("a", optionsA);
  bodyA.setCollisionShape(sx::CollisionShape::makeSphere(1.0));

  sx::RigidBodyOptions optionsB;
  optionsB.position = Eigen::Vector3d(1.5, 0.0, 0.0);
  auto bodyB = world.addRigidBody("b", optionsB);
  bodyB.setCollisionShape(sx::CollisionShape::makeSphere(1.0));

  EXPECT_TRUE(bodyA.hasCollisionShape());
  ASSERT_TRUE(bodyA.getCollisionShape().has_value());
  EXPECT_EQ(bodyA.getCollisionShape()->type, sx::CollisionShapeType::Sphere);
  EXPECT_FALSE(world.addRigidBody("c").hasCollisionShape());

  // Spheres of radius 1 centered 1.5 apart overlap by 0.5.
  auto contacts = world.collide();
  ASSERT_FALSE(contacts.empty());
  for (const auto& contact : contacts) {
    EXPECT_GT(contact.depth, 0.0);
    EXPECT_NEAR(contact.depth, 0.5, 1e-6);
    EXPECT_NEAR(std::abs(contact.normal.x()), 1.0, 1e-6);
  }

  // Separate the bodies; the query should report no contacts.
  Eigen::Isometry3d farPose = Eigen::Isometry3d::Identity();
  farPose.translation() = Eigen::Vector3d(10.0, 0.0, 0.0);
  bodyB.setTransform(farPose);
  EXPECT_TRUE(world.collide().empty());
}

// Test that the contact stage resolves approaching velocities (fully inelastic)
// between overlapping rigid bodies and leaves separating bodies untouched.
TEST(World, RigidBodyContactResolvesApproachingVelocity)
{
  namespace sx = dart::simulation::experimental;

  // Two equal-mass spheres overlapping and approaching head-on along x.
  {
    sx::World world;
    world.setGravity(Eigen::Vector3d::Zero());

    sx::RigidBodyOptions optionsA;
    optionsA.position = Eigen::Vector3d(-0.45, 0.0, 0.0);
    optionsA.linearVelocity = Eigen::Vector3d(1.0, 0.0, 0.0);
    auto bodyA = world.addRigidBody("a", optionsA);
    bodyA.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

    sx::RigidBodyOptions optionsB;
    optionsB.position = Eigen::Vector3d(0.45, 0.0, 0.0);
    optionsB.linearVelocity = Eigen::Vector3d(-1.0, 0.0, 0.0);
    auto bodyB = world.addRigidBody("b", optionsB);
    bodyB.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

    world.setTimeStep(0.001);
    world.step();

    // Equal masses, head-on, fully inelastic: both come to rest. Linear
    // momentum (zero) is conserved.
    EXPECT_NEAR(bodyA.getLinearVelocity().x(), 0.0, 1e-9);
    EXPECT_NEAR(bodyB.getLinearVelocity().x(), 0.0, 1e-9);
  }

  // Overlapping but separating: the contact stage must not change velocities.
  {
    sx::World world;
    world.setGravity(Eigen::Vector3d::Zero());

    sx::RigidBodyOptions optionsA;
    optionsA.position = Eigen::Vector3d(-0.45, 0.0, 0.0);
    optionsA.linearVelocity = Eigen::Vector3d(-1.0, 0.0, 0.0);
    auto bodyA = world.addRigidBody("a", optionsA);
    bodyA.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

    sx::RigidBodyOptions optionsB;
    optionsB.position = Eigen::Vector3d(0.45, 0.0, 0.0);
    optionsB.linearVelocity = Eigen::Vector3d(1.0, 0.0, 0.0);
    auto bodyB = world.addRigidBody("b", optionsB);
    bodyB.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

    world.setTimeStep(0.001);
    world.step();

    EXPECT_NEAR(bodyA.getLinearVelocity().x(), -1.0, 1e-9);
    EXPECT_NEAR(bodyB.getLinearVelocity().x(), 1.0, 1e-9);
  }
}

// Test that a dynamic body dropped onto a static ground comes to rest on it
// (gravity + contact response + positional correction keep it from sinking).
TEST(World, RigidBodyRestsOnStaticGround)
{
  namespace sx = dart::simulation::experimental;

  sx::World world; // default gravity (0, 0, -9.81)

  // Static ground box with its top face at z = 0.
  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
  auto ground = world.addRigidBody("ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(5.0, 5.0, 0.5)));
  EXPECT_TRUE(ground.isStatic());

  // Dynamic sphere dropped from above.
  sx::RigidBodyOptions sphereOptions;
  sphereOptions.position = Eigen::Vector3d(0.0, 0.0, 2.0);
  auto sphere = world.addRigidBody("sphere", sphereOptions);
  sphere.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  world.setTimeStep(0.005);
  world.enterSimulationMode();
  world.step(1000);

  // The sphere (radius 0.5) should rest with its center near z = 0.5 and have
  // essentially stopped moving. The ground must not have moved.
  EXPECT_NEAR(sphere.getTranslation().z(), 0.5, 2e-2);
  EXPECT_LT(std::abs(sphere.getLinearVelocity().z()), 0.1);
  EXPECT_TRUE(
      ground.getTranslation().isApprox(Eigen::Vector3d(0.0, 0.0, -0.5)));
}

// Test that restitution makes contacts bounce: a perfectly elastic, equal-mass,
// head-on collision swaps the bodies' velocities.
TEST(World, RigidBodyContactRestitution)
{
  namespace sx = dart::simulation::experimental;

  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());

  sx::RigidBodyOptions optionsA;
  optionsA.position = Eigen::Vector3d(-0.45, 0.0, 0.0);
  optionsA.linearVelocity = Eigen::Vector3d(1.0, 0.0, 0.0);
  auto bodyA = world.addRigidBody("a", optionsA);
  bodyA.setCollisionShape(sx::CollisionShape::makeSphere(0.5));
  bodyA.setRestitution(1.0);

  sx::RigidBodyOptions optionsB;
  optionsB.position = Eigen::Vector3d(0.45, 0.0, 0.0);
  optionsB.linearVelocity = Eigen::Vector3d(-1.0, 0.0, 0.0);
  auto bodyB = world.addRigidBody("b", optionsB);
  bodyB.setCollisionShape(sx::CollisionShape::makeSphere(0.5));
  bodyB.setRestitution(1.0);

  EXPECT_DOUBLE_EQ(bodyA.getRestitution(), 1.0);
  EXPECT_DOUBLE_EQ(world.addRigidBody("default").getRestitution(), 0.0);
  EXPECT_DOUBLE_EQ(world.addRigidBody("default_friction").getFriction(), 1.0);

  world.setTimeStep(0.001);
  world.step();

  // Perfectly elastic, equal masses, head-on: the velocities swap.
  EXPECT_NEAR(bodyA.getLinearVelocity().x(), -1.0, 1e-9);
  EXPECT_NEAR(bodyB.getLinearVelocity().x(), 1.0, 1e-9);
}

// Test that Coulomb friction decelerates a body sliding on a static ground.
TEST(World, RigidBodyContactFrictionDeceleratesSlidingBody)
{
  namespace sx = dart::simulation::experimental;

  sx::World world; // default gravity (0, 0, -9.81)

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
  auto ground = world.addRigidBody("ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(5.0, 5.0, 0.5)));

  // A flat, low box (so friction torque does not tip it) sliding in +x.
  sx::RigidBodyOptions boxOptions;
  boxOptions.position = Eigen::Vector3d(0.0, 0.0, 0.1);
  boxOptions.linearVelocity = Eigen::Vector3d(2.0, 0.0, 0.0);
  auto slider = world.addRigidBody("slider", boxOptions);
  slider.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(0.5, 0.5, 0.1)));
  slider.setFriction(0.5);
  EXPECT_DOUBLE_EQ(slider.getFriction(), 0.5);

  world.setTimeStep(0.005);
  world.enterSimulationMode();
  world.step(400);

  // Friction should have braked the slide (started at 2 m/s) without reversing
  // it, and the body should have moved forward before stopping.
  const double velocityX = slider.getLinearVelocity().x();
  EXPECT_LT(velocityX, 0.5);
  EXPECT_GT(velocityX, -0.2);
  EXPECT_GT(slider.getTranslation().x(), 0.0);
}

// Test that rigid-body integration keeps the integrated world pose
// authoritative when a body has been reparented through the inherited frame
// API.
TEST(World, StepStoresReparentedRigidBodyPoseAsParentLocal)
{
  namespace sx = dart::simulation::experimental;

  sx::World world;
  auto parent = world.addFreeFrame("parent");

  Eigen::Isometry3d parentTransform = Eigen::Isometry3d::Identity();
  parentTransform.translate(Eigen::Vector3d(10.0, 0.0, 0.0));
  parent.setLocalTransform(parentTransform);

  sx::RigidBodyOptions options;
  options.position = Eigen::Vector3d(1.0, 2.0, 3.0);
  options.linearVelocity = Eigen::Vector3d(2.0, 0.0, 0.0);

  auto body = world.addRigidBody("body", options);
  body.setParentFrame(parent);

  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.5);
  world.enterSimulationMode();

  world.step();

  const auto expectedPosition = Eigen::Vector3d(2.0, 2.0, 3.0);
  Eigen::Isometry3d expectedWorldTransform = Eigen::Isometry3d::Identity();
  expectedWorldTransform.translation() = expectedPosition;
  Eigen::Isometry3d expectedLocalTransform
      = parentTransform.inverse() * expectedWorldTransform;

  EXPECT_TRUE(body.getTranslation().isApprox(expectedPosition));
  EXPECT_TRUE(body.getLocalTransform().isApprox(expectedLocalTransform));
  EXPECT_TRUE(body.getTransform().isApprox(expectedWorldTransform));
}

// Test that rigid-body frame ancestry is represented in the integration graph
// so a parallel executor cannot run a child integration while its parent rigid
// body is updating frame properties.
TEST(World, RigidBodyIntegrationStageOrdersRigidBodyFrameAncestry)
{
  namespace sx = dart::simulation::experimental;
  namespace compute = dart::simulation::experimental::compute;

  sx::World world;

  sx::RigidBodyOptions parentOptions;
  parentOptions.position = Eigen::Vector3d(10.0, 0.0, 0.0);
  parentOptions.linearVelocity = Eigen::Vector3d(2.0, 0.0, 0.0);
  auto parent = world.addRigidBody("parent", parentOptions);

  sx::RigidBodyOptions childOptions;
  childOptions.position = Eigen::Vector3d(20.0, 0.0, 0.0);
  childOptions.linearVelocity = Eigen::Vector3d(1.0, 0.0, 0.0);
  auto child = world.addRigidBody("child", childOptions);
  child.setParentFrame(parent);

  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.5);
  world.enterSimulationMode();

  RecordingExecutor executor;
  compute::RigidBodyIntegrationStage stage(1);
  stage.execute(world, executor);

  EXPECT_EQ(executor.nodeCount, 2u);
  EXPECT_EQ(executor.edgeCount, 1u);

  const auto expectedParentPosition = Eigen::Vector3d(11.0, 0.0, 0.0);
  const auto expectedChildPosition = Eigen::Vector3d(20.5, 0.0, 0.0);
  Eigen::Isometry3d expectedChildLocalTransform = Eigen::Isometry3d::Identity();
  expectedChildLocalTransform.translation()
      = expectedChildPosition - expectedParentPosition;

  EXPECT_TRUE(parent.getLocalTransform().translation().isApprox(
      expectedParentPosition));
  EXPECT_TRUE(child.getLocalTransform().isApprox(expectedChildLocalTransform));
}

// Test that the rigid-body integration stage produces the same state through
// sequential and parallel graph executors.
TEST(World, RigidBodyStepParallelMatchesSequential)
{
  namespace sx = dart::simulation::experimental;
  namespace compute = dart::simulation::experimental::compute;

  auto addBody = [](sx::World& world, int index) {
    sx::RigidBodyOptions options;
    options.mass = 1.0 + 0.25 * index;
    options.inertia = Eigen::Vector3d(1.0, 1.5, 2.0).asDiagonal();
    options.position = Eigen::Vector3d(0.1 * index, 0.2 * index, 0.3 * index);
    options.linearVelocity = Eigen::Vector3d(0.5, 0.25 * index, -0.125 * index);
    options.angularVelocity
        = Eigen::Vector3d(0.01 * index, 0.02 * index, 0.03 * index);

    auto body = world.addRigidBody("body_" + std::to_string(index), options);
    body.setForce(Eigen::Vector3d(0.1 * index, 0.2, -0.3));
    return body;
  };

  sx::World sequentialWorld;
  sx::World parallelWorld;
  std::vector<sx::RigidBody> sequentialBodies;
  std::vector<sx::RigidBody> parallelBodies;

  for (int i = 0; i < 8; ++i) {
    sequentialBodies.push_back(addBody(sequentialWorld, i));
    parallelBodies.push_back(addBody(parallelWorld, i));
  }

  sequentialWorld.setTimeStep(0.01);
  parallelWorld.setTimeStep(0.01);
  sequentialWorld.enterSimulationMode();
  parallelWorld.enterSimulationMode();

  compute::ParallelExecutor executor(2);
  for (int i = 0; i < 4; ++i) {
    sequentialWorld.step();
    parallelWorld.step(executor);
  }

  EXPECT_DOUBLE_EQ(sequentialWorld.getTime(), parallelWorld.getTime());
  EXPECT_EQ(sequentialWorld.getFrame(), parallelWorld.getFrame());

  constexpr double tolerance = 1e-10;
  for (std::size_t i = 0; i < sequentialBodies.size(); ++i) {
    const auto sequentialTranslation = sequentialBodies[i].getTranslation();
    const auto parallelTranslation = parallelBodies[i].getTranslation();
    const auto sequentialRotation = sequentialBodies[i].getRotation();
    const auto parallelRotation = parallelBodies[i].getRotation();
    const auto sequentialLinearVelocity
        = sequentialBodies[i].getLinearVelocity();
    const auto parallelLinearVelocity = parallelBodies[i].getLinearVelocity();
    const auto sequentialAngularVelocity
        = sequentialBodies[i].getAngularVelocity();
    const auto parallelAngularVelocity = parallelBodies[i].getAngularVelocity();

    EXPECT_TRUE(sequentialTranslation.isApprox(parallelTranslation, tolerance))
        << "position diff: "
        << (sequentialTranslation - parallelTranslation).norm();
    EXPECT_TRUE(sequentialRotation.isApprox(parallelRotation, tolerance))
        << "orientation diff: "
        << (sequentialRotation - parallelRotation).norm();
    EXPECT_TRUE(
        sequentialLinearVelocity.isApprox(parallelLinearVelocity, tolerance))
        << "linear velocity diff: "
        << (sequentialLinearVelocity - parallelLinearVelocity).norm();
    EXPECT_TRUE(
        sequentialAngularVelocity.isApprox(parallelAngularVelocity, tolerance))
        << "angular velocity diff: "
        << (sequentialAngularVelocity - parallelAngularVelocity).norm();
    EXPECT_TRUE(
        sequentialBodies[i].getTransform().isApprox(
            parallelBodies[i].getTransform(), tolerance));
  }
}

// Test that the experimental step path can swap the kinematics stage contract.
TEST(World, StepAcceptsCustomStage)
{
  namespace sx = dart::simulation::experimental;
  namespace compute = dart::simulation::experimental::compute;

  sx::World world;
  auto parent = world.addFreeFrame("parent");

  Eigen::Isometry3d childOffset = Eigen::Isometry3d::Identity();
  childOffset.translate(Eigen::Vector3d(0.0, 1.0, 0.0));
  auto child = world.addFixedFrame("child", parent, childOffset);

  world.enterSimulationMode();

  Eigen::Isometry3d updatedParentTransform = Eigen::Isometry3d::Identity();
  updatedParentTransform.translate(Eigen::Vector3d(2.0, 0.0, 0.0));
  parent.setLocalTransform(updatedParentTransform);

  compute::SequentialExecutor executor;
  CountingKinematicsStage stage;
  world.step(executor, stage);

  EXPECT_EQ(stage.executionCount, 1);
  EXPECT_TRUE(
      child.getTransform().isApprox(updatedParentTransform * childOffset));
}

// Test that the experimental world step path can compose multiple solver
// domains without expanding the default public World::step() surface.
TEST(World, StepAcceptsMultiDomainSolverPipeline)
{
  namespace sx = dart::simulation::experimental;
  namespace compute = dart::simulation::experimental::compute;

  sx::World world;
  world.setTimeStep(0.25);
  world.enterSimulationMode();

  std::vector<std::string> order;
  RecordingWorldStage articulated(
      "articulated",
      {compute::ComputeStageDomain::ArticulatedBody,
       compute::ComputeStageAcceleration::TaskParallel
           | compute::ComputeStageAcceleration::DataLocality},
      order);
  RecordingWorldStage deformable(
      "deformable",
      {compute::ComputeStageDomain::DeformableBody,
       compute::ComputeStageAcceleration::DataParallel
           | compute::ComputeStageAcceleration::Simd},
      order);
  RecordingWorldStage fluid(
      "fluid",
      {compute::ComputeStageDomain::Fluid,
       compute::ComputeStageAcceleration::DataParallel
           | compute::ComputeStageAcceleration::Gpu},
      order);
  RecordingWorldStage renderPrep(
      "render_prep",
      {compute::ComputeStageDomain::Rendering,
       compute::ComputeStageAcceleration::TaskParallel
           | compute::ComputeStageAcceleration::Gpu},
      order);

  compute::WorldStepPipeline pipeline;
  pipeline.addStage(articulated)
      .addStage(deformable)
      .addStage(fluid)
      .addStage(renderPrep);

  ASSERT_EQ(pipeline.getStageCount(), 4u);
  EXPECT_FALSE(pipeline.isEmpty());
  EXPECT_EQ(
      pipeline.getStage(3).getMetadata().domain,
      compute::ComputeStageDomain::Rendering);
  EXPECT_TRUE(
      compute::hasAcceleration(
          pipeline.getStage(3).getMetadata().acceleration,
          compute::ComputeStageAcceleration::Gpu));
  EXPECT_THROW({ (void)pipeline.getStage(4); }, sx::OutOfRangeException);

  compute::SequentialExecutor executor;
  world.step(executor, pipeline);

  const std::vector<std::string> expected{
      "articulated", "deformable", "fluid", "render_prep"};
  EXPECT_EQ(order, expected);
  EXPECT_DOUBLE_EQ(world.getTime(), 0.25);
  EXPECT_EQ(world.getFrame(), 1u);
}

// Test that repeated stepping can reuse a caller-owned executor and pipeline.
TEST(World, StepCountAcceptsMultiDomainSolverPipeline)
{
  namespace sx = dart::simulation::experimental;
  namespace compute = dart::simulation::experimental::compute;

  sx::World world;
  world.setTimeStep(0.1);

  std::vector<std::string> order;
  RecordingWorldStage kinematics(
      "kinematics",
      {compute::ComputeStageDomain::Kinematics,
       compute::toMask(compute::ComputeStageAcceleration::TaskParallel)},
      order);
  RecordingWorldStage sensor(
      "sensor",
      {compute::ComputeStageDomain::Sensor,
       compute::toMask(compute::ComputeStageAcceleration::TaskParallel)},
      order);

  compute::WorldStepPipeline pipeline;
  pipeline.addStage(kinematics).addStage(sensor);

  RecordingExecutor executor;
  world.step(0, executor, pipeline);
  EXPECT_FALSE(world.isSimulationMode());
  EXPECT_TRUE(order.empty());
  EXPECT_EQ(executor.executeCount, 0u);

  world.step(3, executor, pipeline);

  const std::vector<std::string> expected{
      "kinematics", "sensor", "kinematics", "sensor", "kinematics", "sensor"};
  EXPECT_EQ(order, expected);
  EXPECT_EQ(executor.executeCount, expected.size());
  EXPECT_DOUBLE_EQ(world.getTime(), 0.3);
  EXPECT_EQ(world.getFrame(), 3u);
}
