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
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

// Tracks the public rigid-body fixed-joint facade and the AVBD contact-stage
// projection path it activates. The benchmark is a DART-internal baseline for
// regression tracking, not a solver-completeness or paper-parity claim.

#include <dart/simulation/experimental/body/rigid_body.hpp>
#include <dart/simulation/experimental/body/rigid_body_options.hpp>
#include <dart/simulation/experimental/detail/deformable_vbd/rigid_world_contact.hpp>
#include <dart/simulation/experimental/detail/entity_conversion.hpp>
#include <dart/simulation/experimental/detail/world_registry_access.hpp>
#include <dart/simulation/experimental/multibody/joint.hpp>
#include <dart/simulation/experimental/multibody/link.hpp>
#include <dart/simulation/experimental/multibody/multibody.hpp>
#include <dart/simulation/experimental/world.hpp>
#include <dart/simulation/experimental/world_options.hpp>

#include <Eigen/Core>
#include <benchmark/benchmark.h>

#include <memory>
#include <string>
#include <vector>

namespace sx = dart::simulation::experimental;
namespace vbd = dart::simulation::experimental::detail::deformable_vbd;

namespace {

std::unique_ptr<sx::World> makeRigidFixedJointWorld(std::size_t linkCount)
{
  sx::WorldOptions options;
  options.gravity = Eigen::Vector3d::Zero();
  options.timeStep = 0.005;
  auto world = std::make_unique<sx::World>(options);

  sx::RigidBodyOptions baseOptions;
  baseOptions.isStatic = true;
  auto parent = world->addRigidBody("base", baseOptions);

  std::vector<sx::RigidBody> links;
  links.reserve(linkCount);
  for (std::size_t i = 0; i < linkCount; ++i) {
    sx::RigidBodyOptions bodyOptions;
    bodyOptions.position = Eigen::Vector3d(
        static_cast<double>(i + 1), 0.1 * static_cast<double>(i % 3), 0.0);
    bodyOptions.linearVelocity = Eigen::Vector3d(0.15, 0.0, 0.0);
    bodyOptions.angularVelocity = Eigen::Vector3d(0.0, 0.0, 0.2);

    auto child = world->addRigidBody("link_" + std::to_string(i), bodyOptions);
    world->addRigidBodyFixedJoint("fixed_" + std::to_string(i), parent, child);
    parent = child;
    links.push_back(child);
  }

  benchmark::DoNotOptimize(links.data());
  return world;
}

std::unique_ptr<sx::World> makeRigidRevoluteMotorWorld(std::size_t motorCount)
{
  sx::WorldOptions options;
  options.gravity = Eigen::Vector3d::Zero();
  options.timeStep = 0.005;
  auto world = std::make_unique<sx::World>(options);

  sx::RigidBodyOptions baseOptions;
  baseOptions.isStatic = true;
  auto parent = world->addRigidBody("motor_base", baseOptions);

  std::vector<sx::RigidBody> links;
  std::vector<sx::Joint> joints;
  links.reserve(motorCount);
  joints.reserve(motorCount);
  for (std::size_t i = 0; i < motorCount; ++i) {
    sx::RigidBodyOptions bodyOptions;
    bodyOptions.mass = 1.0;
    bodyOptions.position = Eigen::Vector3d(
        0.75 * static_cast<double>(i + 1),
        0.08 * static_cast<double>(i % 2),
        0.0);

    auto child
        = world->addRigidBody("motor_link_" + std::to_string(i), bodyOptions);
    auto joint = world->addRigidBodyRevoluteJoint(
        "motor_hinge_" + std::to_string(i),
        parent,
        child,
        Eigen::Vector3d::UnitZ());
    joint.setActuatorType(sx::ActuatorType::Velocity);
    joint.setCommandVelocity(
        Eigen::VectorXd::Constant(1, 0.75 + 0.05 * static_cast<double>(i % 3)));
    joint.setEffortLimits(
        Eigen::VectorXd::Constant(1, -600.0),
        Eigen::VectorXd::Constant(1, 600.0));

    parent = child;
    links.push_back(child);
    joints.push_back(joint);
  }

  benchmark::DoNotOptimize(links.data());
  benchmark::DoNotOptimize(joints.data());
  return world;
}

std::vector<sx::Joint> makeRigidFixedJoints(
    sx::World& world, std::size_t jointCount)
{
  sx::RigidBodyOptions baseOptions;
  baseOptions.isStatic = true;
  auto parent = world.addRigidBody("endpoint_base", baseOptions);

  std::vector<sx::Joint> joints;
  joints.reserve(jointCount);
  for (std::size_t i = 0; i < jointCount; ++i) {
    sx::RigidBodyOptions bodyOptions;
    bodyOptions.position
        = Eigen::Vector3d(static_cast<double>(i + 1), 0.0, 0.0);
    auto child
        = world.addRigidBody("endpoint_link_" + std::to_string(i), bodyOptions);
    joints.push_back(world.addRigidBodyFixedJoint(
        "endpoint_fixed_" + std::to_string(i), parent, child));
    parent = child;
  }

  return joints;
}

std::vector<entt::entity> makeEndpointClassificationEntities(
    sx::World& world, std::size_t endpointCount)
{
  std::vector<entt::entity> endpoints;
  endpoints.reserve(endpointCount * 2u);

  for (std::size_t i = 0; i < endpointCount; ++i) {
    sx::RigidBodyOptions bodyOptions;
    bodyOptions.position = Eigen::Vector3d(static_cast<double>(i), -0.5, 0.0);
    auto body = world.addRigidBody(
        "classifier_rigid_" + std::to_string(i), bodyOptions);
    endpoints.push_back(sx::detail::toRegistryEntity(body.getEntity()));
  }

  auto robot = world.addMultibody("classifier_robot");
  auto parent = robot.addLink("classifier_root");
  endpoints.push_back(sx::detail::toRegistryEntity(parent.getEntity()));
  for (std::size_t i = 1; i < endpointCount; ++i) {
    sx::JointSpec joint;
    joint.name = "classifier_joint_" + std::to_string(i);
    joint.type = sx::JointType::Revolute;
    joint.axis = Eigen::Vector3d::UnitZ();
    joint.transformFromParent
        = Eigen::Isometry3d(Eigen::Translation3d(Eigen::Vector3d::UnitX()));
    parent
        = robot.addLink("classifier_link_" + std::to_string(i), parent, joint);
    endpoints.push_back(sx::detail::toRegistryEntity(parent.getEntity()));
  }

  return endpoints;
}

} // namespace

//==============================================================================
static void BM_AvbdRigidFixedJointCreate(benchmark::State& state)
{
  const auto linkCount = static_cast<std::size_t>(state.range(0));
  for (auto _ : state) {
    auto world = makeRigidFixedJointWorld(linkCount);
    benchmark::DoNotOptimize(world.get());
    benchmark::ClobberMemory();
  }
  state.counters["fixed_joints"] = static_cast<double>(linkCount);
}
BENCHMARK(BM_AvbdRigidFixedJointCreate)->Arg(1)->Arg(8)->Arg(32);

//==============================================================================
static void BM_AvbdRigidFixedJointEndpointAccess(benchmark::State& state)
{
  sx::World world;
  const auto jointCount = static_cast<std::size_t>(state.range(0));
  auto joints = makeRigidFixedJoints(world, jointCount);

  for (auto _ : state) {
    for (const sx::Joint& joint : joints) {
      auto parent = joint.getParentRigidBody();
      auto child = joint.getChildRigidBody();
      benchmark::DoNotOptimize(parent.isValid());
      benchmark::DoNotOptimize(child.isValid());
    }
  }
  state.counters["fixed_joints"] = static_cast<double>(jointCount);
}
BENCHMARK(BM_AvbdRigidFixedJointEndpointAccess)->Arg(1)->Arg(8)->Arg(32);

//==============================================================================
static void BM_AvbdRigidFixedJointWorldLookup(benchmark::State& state)
{
  sx::World world;
  const auto jointCount = static_cast<std::size_t>(state.range(0));
  (void)makeRigidFixedJoints(world, jointCount);

  std::vector<std::string> names;
  names.reserve(jointCount);
  for (std::size_t i = 0; i < jointCount; ++i) {
    names.push_back("endpoint_fixed_" + std::to_string(i));
  }

  for (auto _ : state) {
    for (const std::string& name : names) {
      auto joint = world.getRigidBodyFixedJoint(name);
      benchmark::DoNotOptimize(joint.has_value());
      if (joint.has_value()) {
        benchmark::DoNotOptimize(joint->getParentRigidBody().isValid());
        benchmark::DoNotOptimize(joint->getChildRigidBody().isValid());
      }
    }
    benchmark::DoNotOptimize(world.getRigidBodyFixedJointCount());
  }
  state.counters["fixed_joints"] = static_cast<double>(jointCount);
}
BENCHMARK(BM_AvbdRigidFixedJointWorldLookup)->Arg(1)->Arg(8)->Arg(32);

//==============================================================================
static void BM_AvbdRigidFixedJointWorldList(benchmark::State& state)
{
  sx::World world;
  const auto jointCount = static_cast<std::size_t>(state.range(0));
  (void)makeRigidFixedJoints(world, jointCount);

  for (auto _ : state) {
    const auto joints = world.getRigidBodyFixedJoints();
    benchmark::DoNotOptimize(joints.data());
    benchmark::DoNotOptimize(joints.size());
  }
  state.counters["fixed_joints"] = static_cast<double>(jointCount);
}
BENCHMARK(BM_AvbdRigidFixedJointWorldList)->Arg(1)->Arg(8)->Arg(32);

//==============================================================================
static void BM_AvbdRigidEndpointClassification(benchmark::State& state)
{
  sx::World world;
  const auto endpointCount = static_cast<std::size_t>(state.range(0));
  const std::vector<entt::entity> endpoints
      = makeEndpointClassificationEntities(world, endpointCount);
  const auto& registry = sx::detail::registryOf(world);

  for (auto _ : state) {
    std::size_t freeRigidEndpoints = 0;
    std::size_t multibodyEndpoints = 0;
    for (const entt::entity endpointEntity : endpoints) {
      const vbd::AvbdRigidWorldEndpoint endpoint
          = vbd::classifyAvbdRigidWorldEndpoint(registry, endpointEntity);
      if (endpoint.kind == vbd::AvbdRigidWorldEndpointKind::FreeRigidBody) {
        ++freeRigidEndpoints;
      } else if (
          endpoint.kind == vbd::AvbdRigidWorldEndpointKind::MultibodyLink) {
        ++multibodyEndpoints;
      }
    }
    benchmark::DoNotOptimize(freeRigidEndpoints);
    benchmark::DoNotOptimize(multibodyEndpoints);
  }

  state.counters["free_rigid_endpoints"] = static_cast<double>(endpointCount);
  state.counters["multibody_link_endpoints"]
      = static_cast<double>(endpointCount);
}
BENCHMARK(BM_AvbdRigidEndpointClassification)->Arg(1)->Arg(8)->Arg(32);

//==============================================================================
static void BM_AvbdRigidFixedJointStep(benchmark::State& state)
{
  const auto linkCount = static_cast<std::size_t>(state.range(0));
  auto world = makeRigidFixedJointWorld(linkCount);
  world->enterSimulationMode();

  for (auto _ : state) {
    world->step();
    benchmark::ClobberMemory();
  }
  state.counters["fixed_joints"] = static_cast<double>(linkCount);
}
BENCHMARK(BM_AvbdRigidFixedJointStep)->Arg(1)->Arg(8)->Arg(32);

//==============================================================================
static void BM_AvbdRigidRevoluteMotorStep(benchmark::State& state)
{
  const auto motorCount = static_cast<std::size_t>(state.range(0));
  auto world = makeRigidRevoluteMotorWorld(motorCount);
  world->enterSimulationMode();

  for (auto _ : state) {
    world->step();
    benchmark::ClobberMemory();
  }
  state.counters["motors"] = static_cast<double>(motorCount);
}
BENCHMARK(BM_AvbdRigidRevoluteMotorStep)->Arg(1)->Arg(8)->Arg(32);

BENCHMARK_MAIN();
