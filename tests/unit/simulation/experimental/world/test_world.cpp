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

#include <dart/simulation/experimental/body/rigid_body.hpp>
#include <dart/simulation/experimental/common/exceptions.hpp>
#include <dart/simulation/experimental/comps/dynamics.hpp>
#include <dart/simulation/experimental/comps/frame_types.hpp>
#include <dart/simulation/experimental/comps/joint.hpp>
#include <dart/simulation/experimental/compute/compute_executor.hpp>
#include <dart/simulation/experimental/compute/compute_graph.hpp>
#include <dart/simulation/experimental/compute/parallel_executor.hpp>
#include <dart/simulation/experimental/compute/rigid_body_state_batch.hpp>
#include <dart/simulation/experimental/compute/sequential_executor.hpp>
#include <dart/simulation/experimental/compute/world_batch.hpp>
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

// Test that parallel execution matches the sequential reference across a range
// of worker counts, hardening the deterministic-sync guarantee beyond a single
// worker configuration.
TEST(World, RigidBodyStepParallelMatchesSequentialAcrossWorkerCounts)
{
  namespace sx = dart::simulation::experimental;
  namespace compute = dart::simulation::experimental::compute;

  constexpr int bodyCount = 12;
  constexpr int stepCount = 5;
  constexpr double tolerance = 1e-10;

  auto buildWorld = [](sx::World& world, std::vector<sx::RigidBody>& bodies) {
    for (int i = 0; i < bodyCount; ++i) {
      sx::RigidBodyOptions options;
      options.mass = 1.0 + 0.1 * i;
      options.inertia = Eigen::Vector3d(1.0, 1.5, 2.0).asDiagonal();
      options.position = Eigen::Vector3d(0.1 * i, 0.2 * i, 0.3 * i);
      options.linearVelocity = Eigen::Vector3d(0.5, 0.25 * i, -0.1 * i);
      options.angularVelocity = Eigen::Vector3d(0.01 * i, 0.02 * i, 0.03 * i);
      auto body = world.addRigidBody("body_" + std::to_string(i), options);
      body.setForce(Eigen::Vector3d(0.1 * i, 0.2, -0.3));
      bodies.push_back(body);
    }
    world.setTimeStep(0.01);
    world.enterSimulationMode();
  };

  sx::World reference;
  std::vector<sx::RigidBody> referenceBodies;
  buildWorld(reference, referenceBodies);
  for (int s = 0; s < stepCount; ++s) {
    reference.step();
  }

  for (const std::size_t workers :
       {std::size_t{1}, std::size_t{2}, std::size_t{4}, std::size_t{8}}) {
    sx::World world;
    std::vector<sx::RigidBody> bodies;
    buildWorld(world, bodies);

    compute::ParallelExecutor executor(workers);
    for (int s = 0; s < stepCount; ++s) {
      world.step(executor);
    }

    for (int i = 0; i < bodyCount; ++i) {
      EXPECT_TRUE(
          referenceBodies[static_cast<std::size_t>(i)].getTransform().isApprox(
              bodies[static_cast<std::size_t>(i)].getTransform(), tolerance))
          << "workers=" << workers << " body=" << i;
      EXPECT_TRUE(
          referenceBodies[static_cast<std::size_t>(i)]
              .getLinearVelocity()
              .isApprox(
                  bodies[static_cast<std::size_t>(i)].getLinearVelocity(),
                  tolerance))
          << "workers=" << workers << " body=" << i;
      EXPECT_TRUE(
          referenceBodies[static_cast<std::size_t>(i)]
              .getAngularVelocity()
              .isApprox(
                  bodies[static_cast<std::size_t>(i)].getAngularVelocity(),
                  tolerance))
          << "workers=" << workers << " body=" << i;
    }
  }
}

// Test that the batched SoA integration stage matches the per-entity stage for
// free (non-frame-coupled) rigid bodies with no angular motion or torque, which
// is the regime where the two integrators share semantics.
TEST(World, BatchedRigidBodyIntegrationStageMatchesPerEntityForFreeBodies)
{
  namespace sx = dart::simulation::experimental;
  namespace compute = dart::simulation::experimental::compute;

  auto buildWorld = [](sx::World& world, std::vector<sx::RigidBody>& bodies) {
    for (int i = 0; i < 8; ++i) {
      sx::RigidBodyOptions options;
      options.mass = 1.0 + 0.3 * i;
      options.inertia = Eigen::Vector3d(1.0, 1.5, 2.0).asDiagonal();
      options.position = Eigen::Vector3d(0.2 * i, -0.1 * i, 0.05 * i);
      options.linearVelocity = Eigen::Vector3d(0.5, 0.25 * i, -0.1 * i);
      // Zero angular velocity: once a body spins, the per-entity (angle-axis)
      // and SoA (quaternion-product) orientation schemes diverge by design.
      options.angularVelocity = Eigen::Vector3d::Zero();
      auto body = world.addRigidBody("body_" + std::to_string(i), options);
      // Linear force only: a torque would exercise the per-entity inertia solve
      // that the SoA model does not yet carry.
      body.setForce(Eigen::Vector3d(0.1 * i, 0.2, -0.3));
      bodies.push_back(body);
    }
    world.setTimeStep(0.01);
    world.enterSimulationMode();
  };

  sx::World perEntityWorld;
  sx::World batchedWorld;
  std::vector<sx::RigidBody> perEntityBodies;
  std::vector<sx::RigidBody> batchedBodies;
  buildWorld(perEntityWorld, perEntityBodies);
  buildWorld(batchedWorld, batchedBodies);

  compute::SequentialExecutor executor;
  compute::RigidBodyIntegrationStage perEntityStage;
  compute::BatchedRigidBodyIntegrationStage batchedStage;

  constexpr double tolerance = 1e-10;
  for (int step = 0; step < 5; ++step) {
    perEntityStage.execute(perEntityWorld, executor);
    batchedStage.execute(batchedWorld, executor);

    for (std::size_t i = 0; i < perEntityBodies.size(); ++i) {
      EXPECT_TRUE(
          perEntityBodies[i].getTranslation().isApprox(
              batchedBodies[i].getTranslation(), tolerance))
          << "step " << step << " body " << i << " position";
      EXPECT_TRUE(
          perEntityBodies[i].getRotation().isApprox(
              batchedBodies[i].getRotation(), tolerance))
          << "step " << step << " body " << i << " orientation";
      EXPECT_TRUE(
          perEntityBodies[i].getLinearVelocity().isApprox(
              batchedBodies[i].getLinearVelocity(), tolerance))
          << "step " << step << " body " << i << " linear velocity";
      EXPECT_TRUE(
          perEntityBodies[i].getLocalTransform().isApprox(
              batchedBodies[i].getLocalTransform(), tolerance))
          << "step " << step << " body " << i << " local transform";
    }
  }
}

// Test that the batched stage defers to the per-entity stage (producing
// identical results) when a rigid body is parented to another rigid body, where
// local-transform bookkeeping needs parent-before-child ordering.
TEST(World, BatchedRigidBodyIntegrationStageDefersForFrameCoupledBodies)
{
  namespace sx = dart::simulation::experimental;
  namespace compute = dart::simulation::experimental::compute;

  auto buildWorld = [](sx::World& world) {
    sx::RigidBodyOptions parentOptions;
    parentOptions.position = Eigen::Vector3d(10.0, 0.0, 0.0);
    parentOptions.linearVelocity = Eigen::Vector3d(2.0, 0.0, 0.0);
    auto parent = world.addRigidBody("parent", parentOptions);

    sx::RigidBodyOptions childOptions;
    childOptions.position = Eigen::Vector3d(20.0, 0.0, 0.0);
    childOptions.linearVelocity = Eigen::Vector3d(1.0, 0.0, 0.0);
    auto child = world.addRigidBody("child", childOptions);
    child.setParentFrame(parent);

    world.setTimeStep(0.5);
    world.enterSimulationMode();
    return std::pair<sx::RigidBody, sx::RigidBody>(parent, child);
  };

  sx::World perEntityWorld;
  sx::World batchedWorld;
  auto [perEntityParent, perEntityChild] = buildWorld(perEntityWorld);
  auto [batchedParent, batchedChild] = buildWorld(batchedWorld);

  compute::SequentialExecutor executor;
  compute::RigidBodyIntegrationStage perEntityStage;
  compute::BatchedRigidBodyIntegrationStage batchedStage;
  perEntityStage.execute(perEntityWorld, executor);
  batchedStage.execute(batchedWorld, executor);

  EXPECT_TRUE(perEntityParent.getLocalTransform().isApprox(
      batchedParent.getLocalTransform()));
  EXPECT_TRUE(perEntityChild.getLocalTransform().isApprox(
      batchedChild.getLocalTransform()));
  EXPECT_TRUE(
      perEntityParent.getTransform().isApprox(batchedParent.getTransform()));
  EXPECT_TRUE(
      perEntityChild.getTransform().isApprox(batchedChild.getTransform()));
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

// Test that rigid-body state can be extracted to a structure-of-arrays batch
// and applied back, round-tripping exactly and rejecting size mismatches.
TEST(World, RigidBodyStateBatchRoundTrip)
{
  namespace sx = dart::simulation::experimental;
  namespace compute = dart::simulation::experimental::compute;

  auto build = [](sx::World& world) {
    std::vector<sx::RigidBody> bodies;
    for (int i = 0; i < 4; ++i) {
      sx::RigidBodyOptions options;
      options.mass = 1.0 + i;
      options.position = Eigen::Vector3d(0.1 * i, 0.2 * i, 0.3 * i);
      options.linearVelocity = Eigen::Vector3d(0.5 * i, -0.25 * i, 0.1 * i);
      options.angularVelocity = Eigen::Vector3d(0.01 * i, 0.02 * i, 0.03 * i);
      bodies.push_back(
          world.addRigidBody("body_" + std::to_string(i), options));
    }
    return bodies;
  };

  sx::World source;
  auto sourceBodies = build(source);

  const auto batch = compute::extractRigidBodyState(source);
  EXPECT_EQ(batch.worldCount, 1u);
  EXPECT_EQ(batch.bodyCount, 4u);
  EXPECT_EQ(batch.position.size(), 12u);
  EXPECT_EQ(batch.orientation.size(), 16u);
  EXPECT_EQ(batch.linearVelocity.size(), 12u);
  EXPECT_EQ(batch.angularVelocity.size(), 12u);

  // Apply the source snapshot to an identical world; the re-extracted state
  // must match field for field.
  sx::World target;
  auto targetBodies = build(target);
  compute::applyRigidBodyState(target, batch);

  const auto roundTrip = compute::extractRigidBodyState(target);
  ASSERT_EQ(roundTrip.bodyCount, batch.bodyCount);
  EXPECT_EQ(roundTrip.position, batch.position);
  EXPECT_EQ(roundTrip.orientation, batch.orientation);
  EXPECT_EQ(roundTrip.linearVelocity, batch.linearVelocity);
  EXPECT_EQ(roundTrip.angularVelocity, batch.angularVelocity);

  // Mutating the batch and applying it changes world state deterministically.
  auto mutated = batch;
  for (auto& value : mutated.position) {
    value += 1.0;
  }
  compute::applyRigidBodyState(source, mutated);
  const auto afterMutation = compute::extractRigidBodyState(source);
  for (std::size_t i = 0; i < afterMutation.position.size(); ++i) {
    EXPECT_DOUBLE_EQ(afterMutation.position[i], batch.position[i] + 1.0);
  }

  // A batch whose body count does not match the world is rejected.
  sx::World smaller;
  sx::RigidBodyOptions soleOptions;
  smaller.addRigidBody("only", soleOptions);
  EXPECT_THROW(
      compute::applyRigidBodyState(smaller, batch),
      sx::InvalidArgumentException);

  // A batch whose array sizes are inconsistent with bodyCount is rejected
  // before any out-of-bounds access.
  compute::RigidBodyStateBatch malformed;
  malformed.worldCount = 1;
  malformed.bodyCount = batch.bodyCount; // arrays intentionally left empty
  EXPECT_THROW(
      compute::applyRigidBodyState(target, malformed),
      sx::InvalidArgumentException);
}

// Test the homogeneous multi-world batch path: extract with a leading world
// dimension, world-major ordering, round-trip apply, and rejection of
// world-count and body-count mismatches.
TEST(World, RigidBodyStateBatchMultiWorld)
{
  namespace sx = dart::simulation::experimental;
  namespace compute = dart::simulation::experimental::compute;

  auto buildWorld = [](sx::World& world, double seed) {
    for (int i = 0; i < 2; ++i) {
      sx::RigidBodyOptions options;
      options.position = Eigen::Vector3d(seed + i, seed + 2 * i, seed + 3 * i);
      options.linearVelocity = Eigen::Vector3d(seed, -seed, 0.5 * seed);
      world.addRigidBody("body_" + std::to_string(i), options);
    }
  };

  sx::World w0;
  sx::World w1;
  sx::World w2;
  buildWorld(w0, 1.0);
  buildWorld(w1, 2.0);
  buildWorld(w2, 3.0);

  const std::vector<const sx::World*> worlds{&w0, &w1, &w2};
  const auto batch = compute::extractRigidBodyStateBatch(worlds);
  EXPECT_EQ(batch.worldCount, 3u);
  EXPECT_EQ(batch.bodyCount, 2u);
  EXPECT_EQ(batch.position.size(), 18u);    // 3 * worldCount * bodyCount
  EXPECT_EQ(batch.orientation.size(), 24u); // 4 * worldCount * bodyCount

  // World-major ordering: each world's slice equals its single-world extract.
  const auto single0 = compute::extractRigidBodyState(w0);
  const auto single2 = compute::extractRigidBodyState(w2);
  for (std::size_t k = 0; k < 6; ++k) {
    EXPECT_DOUBLE_EQ(batch.position[k], single0.position[k]);
    EXPECT_DOUBLE_EQ(batch.position[12 + k], single2.position[k]);
  }

  // Apply to fresh worlds (different initial state) and round-trip.
  sx::World t0;
  sx::World t1;
  sx::World t2;
  buildWorld(t0, 9.0);
  buildWorld(t1, 9.0);
  buildWorld(t2, 9.0);
  const std::vector<sx::World*> targets{&t0, &t1, &t2};
  compute::applyRigidBodyStateBatch(targets, batch);

  const std::vector<const sx::World*> constTargets{&t0, &t1, &t2};
  const auto roundTrip = compute::extractRigidBodyStateBatch(constTargets);
  EXPECT_EQ(roundTrip.position, batch.position);
  EXPECT_EQ(roundTrip.orientation, batch.orientation);
  EXPECT_EQ(roundTrip.linearVelocity, batch.linearVelocity);
  EXPECT_EQ(roundTrip.angularVelocity, batch.angularVelocity);

  // A world-count mismatch on apply is rejected.
  const std::vector<sx::World*> twoTargets{&t0, &t1};
  EXPECT_THROW(
      compute::applyRigidBodyStateBatch(twoTargets, batch),
      sx::InvalidArgumentException);

  // Heterogeneous body counts are rejected on extract.
  sx::World hetero;
  buildWorld(hetero, 5.0);
  sx::RigidBodyOptions extra;
  hetero.addRigidBody("extra", extra);
  const std::vector<const sx::World*> heteroWorlds{&w0, &hetero};
  EXPECT_THROW(
      { (void)compute::extractRigidBodyStateBatch(heteroWorlds); },
      sx::InvalidArgumentException);
}

// Test the CPU batch executor: N independent homogeneous worlds advanced in
// parallel match a single sequentially-stepped reference, and null worlds are
// rejected.
TEST(World, StepWorldsBatchedMatchesSequential)
{
  namespace sx = dart::simulation::experimental;
  namespace compute = dart::simulation::experimental::compute;

  auto build = [](sx::World& world) {
    for (int i = 0; i < 3; ++i) {
      sx::RigidBodyOptions options;
      options.mass = 1.0 + i;
      options.position = Eigen::Vector3d(1.0 + i, 0.0, 0.0);
      options.linearVelocity = Eigen::Vector3d(0.1 * (i + 1), 0.0, 0.0);
      auto body = world.addRigidBody("body_" + std::to_string(i), options);
      body.setForce(Eigen::Vector3d(0.2, -0.1, 0.05));
    }
    world.setTimeStep(0.01);
    world.enterSimulationMode();
  };

  sx::World reference;
  build(reference);
  reference.step(5);
  const auto refState = compute::extractRigidBodyState(reference);

  sx::World w0;
  sx::World w1;
  sx::World w2;
  sx::World w3;
  build(w0);
  build(w1);
  build(w2);
  build(w3);
  const std::vector<sx::World*> worlds{&w0, &w1, &w2, &w3};

  compute::ParallelExecutor executor(2);
  compute::stepWorldsBatched(worlds, 5, executor);

  for (auto* world : worlds) {
    const auto state = compute::extractRigidBodyState(*world);
    ASSERT_EQ(state.bodyCount, refState.bodyCount);
    EXPECT_EQ(state.position, refState.position);
    EXPECT_EQ(state.linearVelocity, refState.linearVelocity);
    EXPECT_EQ(state.orientation, refState.orientation);
    EXPECT_DOUBLE_EQ(world->getTime(), reference.getTime());
  }

  const std::vector<sx::World*> withNull{&w0, nullptr};
  EXPECT_THROW(
      compute::stepWorldsBatched(withNull, 1, executor),
      sx::InvalidArgumentException);
}

// Test the batched rollout: applying a shared initial state, advancing, and
// returning the final batched state matches a single-world reference.
TEST(World, RolloutWorldsBatchedMatchesReference)
{
  namespace sx = dart::simulation::experimental;
  namespace compute = dart::simulation::experimental::compute;

  auto build = [](sx::World& world) {
    for (int i = 0; i < 2; ++i) {
      sx::RigidBodyOptions options;
      options.mass = 1.0 + i;
      options.position = Eigen::Vector3d(0.5 + i, 0.0, 0.0);
      options.linearVelocity = Eigen::Vector3d(0.2 * (i + 1), 0.1, 0.0);
      auto body = world.addRigidBody("body_" + std::to_string(i), options);
      body.setForce(Eigen::Vector3d(0.15, -0.05, 0.1));
    }
    world.setTimeStep(0.01);
    world.enterSimulationMode();
  };

  sx::World w0;
  sx::World w1;
  build(w0);
  build(w1);
  const std::vector<sx::World*> worlds{&w0, &w1};

  // Capture the homogeneous initial state, then perturb the worlds so the
  // rollout's apply step must reset them.
  const std::vector<const sx::World*> constWorlds{&w0, &w1};
  const auto initial = compute::extractRigidBodyStateBatch(constWorlds);
  w0.step(3);
  w1.step(7);

  compute::ParallelExecutor executor(2);
  const auto result
      = compute::rolloutWorldsBatched(worlds, initial, 5, executor);

  // Reference: a single world built identically (so its state equals one slice
  // of the initial batch) and stepped the same number of times.
  sx::World reference;
  build(reference);
  reference.step(5);
  const auto refFinal = compute::extractRigidBodyState(reference);

  EXPECT_EQ(result.worldCount, 2u);
  ASSERT_EQ(result.bodyCount, refFinal.bodyCount);
  const auto stride = refFinal.position.size();
  for (std::size_t r = 0; r < 2; ++r) {
    for (std::size_t k = 0; k < stride; ++k) {
      EXPECT_DOUBLE_EQ(result.position[r * stride + k], refFinal.position[k]);
      EXPECT_DOUBLE_EQ(
          result.linearVelocity[r * stride + k], refFinal.linearVelocity[k]);
    }
  }
}

// Test the immutable Model batch: inverse masses are extracted in state order,
// and the model-based integrator overload matches the explicit-vector overload.
TEST(World, RigidBodyModelBatchIntegration)
{
  namespace sx = dart::simulation::experimental;
  namespace compute = dart::simulation::experimental::compute;

  sx::World world;
  for (int i = 0; i < 2; ++i) {
    sx::RigidBodyOptions options;
    options.mass = 2.0 * (i + 1); // body 0 mass 2, body 1 mass 4
    options.position = Eigen::Vector3d(i, 0.0, 0.0);
    world.addRigidBody("b" + std::to_string(i), options);
  }

  const auto state = compute::extractRigidBodyState(world);
  const auto model = compute::extractRigidBodyModelBatch(world);
  EXPECT_EQ(model.worldCount, 1u);
  EXPECT_EQ(model.bodyCount, 2u);
  ASSERT_EQ(model.inverseMass.size(), 2u);
  // EnTT view order is not insertion order, so check the set, not positions.
  EXPECT_TRUE(
      (model.inverseMass[0] == 0.5 && model.inverseMass[1] == 0.25)
      || (model.inverseMass[0] == 0.25 && model.inverseMass[1] == 0.5));

  const std::vector<double> force = {0.0, 0.0, 4.0, 0.0, 0.0, 8.0};
  const double dt = 0.5;

  auto viaModel = state;
  compute::integrateRigidBodyStateBatchLinear(viaModel, model, force, dt);

  auto viaVector = state;
  compute::integrateRigidBodyStateBatchLinear(
      viaVector, force, model.inverseMass, dt);

  EXPECT_EQ(viaModel.linearVelocity, viaVector.linearVelocity);
  EXPECT_EQ(viaModel.position, viaVector.position);
  // Per body, in extraction order: z-velocity == force.z * inverseMass * dt
  // (initial velocity is zero), independent of the view iteration order.
  for (std::size_t i = 0; i < model.bodyCount; ++i) {
    EXPECT_DOUBLE_EQ(
        viaModel.linearVelocity[3 * i + 2],
        force[3 * i + 2] * (model.inverseMass[i] * dt));
  }

  // A model whose body count does not match the state is rejected.
  compute::RigidBodyModelBatch wrong;
  wrong.worldCount = 1;
  wrong.bodyCount = 1;
  wrong.inverseMass = {1.0};
  EXPECT_THROW(
      compute::integrateRigidBodyStateBatchLinear(viaModel, wrong, force, dt),
      sx::InvalidArgumentException);
}
