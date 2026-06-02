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
#include <dart/simulation/experimental/multibody/joint.hpp>
#include <dart/simulation/experimental/world.hpp>
#include <dart/simulation/experimental/world_options.hpp>

#include <Eigen/Core>
#include <benchmark/benchmark.h>

#include <memory>
#include <string>
#include <vector>

namespace sx = dart::simulation::experimental;

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

BENCHMARK_MAIN();
