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
 *     copyright notice, this list of conditions and the disclaimer in
 *     the documentation and/or other materials provided with the
 *     distribution.
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

// Private reduced GPU scene-parity packet. This intentionally exercises only
// DART-owned scene state extraction plus the private rigid-body state-batch
// CUDA rollout. It is not a GPU World::step or contact/constraint solver claim.

#include <dart/simulation/body/collision_shape.hpp>
#include <dart/simulation/body/rigid_body.hpp>
#include <dart/simulation/body/rigid_body_options.hpp>
#include <dart/simulation/compute/rigid_body_state_batch.hpp>
#include <dart/simulation/multibody/joint.hpp>
#include <dart/simulation/multibody/multibody.hpp>
#include <dart/simulation/world.hpp>

#include <Eigen/Core>
#include <benchmark/benchmark.h>
#include <dart/simulation/compute/cuda/rigid_body_state_batch_cuda.cuh>

#include <algorithm>
#include <array>
#include <string>
#include <string_view>
#include <vector>

#include <cmath>
#include <cstddef>

namespace sx = dart::simulation;
namespace compute = dart::simulation::compute;
namespace cuda = dart::simulation::compute::cuda;

namespace {

sx::Joint addFixedJoint(
    sx::World& world,
    std::string_view name,
    const sx::RigidBody& parent,
    const sx::RigidBody& child)
{
  sx::JointSpec spec;
  spec.name = std::string(name);
  spec.type = sx::JointType::Fixed;
  return world.addJoint(parent, child, spec);
}

constexpr std::array<double, 4> kBridgeBoardX{-0.45, -0.15, 0.15, 0.45};
constexpr std::size_t kSceneBodyCount = 7;
constexpr double kTimeStep = 0.005;

const Eigen::Vector3d kGravity(0.0, 0.0, -9.81);
const Eigen::Vector3d kBridgeBoardHalfExtents(0.10, 0.16, 0.025);
const Eigen::Vector3d kBridgePostHalfExtents(0.05, 0.20, 0.08);
const Eigen::Vector3d kBridgeTravelerHalfExtents(0.07, 0.07, 0.07);

void populateReducedHangingBridgeWorld(sx::World& world)
{
  world.setTimeStep(kTimeStep);
  world.setGravity(kGravity);

  sx::RigidBodyOptions leftPostOptions;
  leftPostOptions.isStatic = true;
  leftPostOptions.position = Eigen::Vector3d(-0.65, 0.0, 0.56);
  sx::RigidBody leftPost
      = world.addRigidBody("plan083_bridge_left_post", leftPostOptions);
  leftPost.setCollisionShape(
      sx::CollisionShape::makeBox(kBridgePostHalfExtents));

  sx::RigidBodyOptions rightPostOptions;
  rightPostOptions.isStatic = true;
  rightPostOptions.position = Eigen::Vector3d(0.65, 0.0, 0.56);
  sx::RigidBody rightPost
      = world.addRigidBody("plan083_bridge_right_post", rightPostOptions);
  rightPost.setCollisionShape(
      sx::CollisionShape::makeBox(kBridgePostHalfExtents));

  sx::RigidBody parent = leftPost;
  for (std::size_t index = 0; index < kBridgeBoardX.size(); ++index) {
    sx::RigidBodyOptions boardOptions;
    boardOptions.mass = 0.25;
    boardOptions.position = Eigen::Vector3d(kBridgeBoardX[index], 0.0, 0.50);
    boardOptions.linearVelocity
        = Eigen::Vector3d(0.02 * static_cast<double>(index), 0.0, -0.02);
    boardOptions.angularVelocity
        = Eigen::Vector3d(0.0, 0.04 + 0.01 * static_cast<double>(index), 0.0);
    sx::RigidBody board = world.addRigidBody(
        "plan083_bridge_board_" + std::to_string(index), boardOptions);
    board.setFriction(0.7);
    board.setCollisionShape(
        sx::CollisionShape::makeBox(kBridgeBoardHalfExtents));
    addFixedJoint(
        world,
        "plan083_bridge_point_connection_" + std::to_string(index),
        parent,
        board);
    parent = board;
  }

  sx::RigidBodyOptions travelerOptions;
  travelerOptions.mass = 0.12;
  travelerOptions.position = Eigen::Vector3d(-0.60, 0.0, 0.82);
  travelerOptions.linearVelocity = Eigen::Vector3d(0.35, 0.0, -0.05);
  travelerOptions.angularVelocity = Eigen::Vector3d(0.0, 0.25, 0.15);
  sx::RigidBody traveler
      = world.addRigidBody("plan083_bridge_traveler", travelerOptions);
  traveler.setFriction(0.4);
  traveler.setCollisionShape(
      sx::CollisionShape::makeBox(kBridgeTravelerHalfExtents));

  world.enterSimulationMode();
}

void replicateArray(
    const std::vector<double>& singleWorld,
    const std::size_t componentCount,
    const std::size_t bodyCount,
    const std::size_t worldCount,
    std::vector<double>& replicated)
{
  replicated.assign(componentCount * bodyCount * worldCount, 0.0);
  for (std::size_t world = 0; world < worldCount; ++world) {
    const std::size_t outputBase = world * componentCount * bodyCount;
    std::copy(
        singleWorld.begin(),
        singleWorld.end(),
        replicated.begin() + static_cast<std::ptrdiff_t>(outputBase));
  }
}

compute::RigidBodyStateBatch replicateState(
    const compute::RigidBodyStateBatch& singleWorld,
    const std::size_t worldCount)
{
  compute::RigidBodyStateBatch batch;
  batch.worldCount = worldCount;
  batch.bodyCount = singleWorld.bodyCount;
  replicateArray(
      singleWorld.position,
      3,
      singleWorld.bodyCount,
      worldCount,
      batch.position);
  replicateArray(
      singleWorld.orientation,
      4,
      singleWorld.bodyCount,
      worldCount,
      batch.orientation);
  replicateArray(
      singleWorld.linearVelocity,
      3,
      singleWorld.bodyCount,
      worldCount,
      batch.linearVelocity);
  replicateArray(
      singleWorld.angularVelocity,
      3,
      singleWorld.bodyCount,
      worldCount,
      batch.angularVelocity);
  return batch;
}

compute::RigidBodyModelBatch replicateModel(
    const compute::RigidBodyModelBatch& singleWorld,
    const std::size_t worldCount)
{
  compute::RigidBodyModelBatch batch;
  batch.worldCount = worldCount;
  batch.bodyCount = singleWorld.bodyCount;
  replicateArray(
      singleWorld.inverseMass,
      1,
      singleWorld.bodyCount,
      worldCount,
      batch.inverseMass);
  replicateArray(
      singleWorld.inertia, 9, singleWorld.bodyCount, worldCount, batch.inertia);
  return batch;
}

std::vector<double> makeGravityForce(const compute::RigidBodyModelBatch& model)
{
  std::vector<double> force(3 * model.worldCount * model.bodyCount, 0.0);
  for (std::size_t world = 0; world < model.worldCount; ++world) {
    for (std::size_t body = 0; body < model.bodyCount; ++body) {
      const std::size_t scalarIndex = world * model.bodyCount + body;
      const double inverseMass = model.inverseMass[scalarIndex];
      if (inverseMass <= 0.0 || !std::isfinite(inverseMass)) {
        continue;
      }
      const Eigen::Vector3d bodyForce = kGravity / inverseMass;
      const std::size_t vectorIndex = 3 * scalarIndex;
      force[vectorIndex + 0] = bodyForce.x();
      force[vectorIndex + 1] = bodyForce.y();
      force[vectorIndex + 2] = bodyForce.z();
    }
  }
  return force;
}

double maxAbsDiff(
    const std::vector<double>& lhs, const std::vector<double>& rhs)
{
  double result = 0.0;
  for (std::size_t index = 0; index < lhs.size(); ++index) {
    result = std::max(result, std::abs(lhs[index] - rhs[index]));
  }
  return result;
}

double maxStateAbsDiff(
    const compute::RigidBodyStateBatch& lhs,
    const compute::RigidBodyStateBatch& rhs)
{
  return std::max(
      {maxAbsDiff(lhs.position, rhs.position),
       maxAbsDiff(lhs.orientation, rhs.orientation),
       maxAbsDiff(lhs.linearVelocity, rhs.linearVelocity),
       maxAbsDiff(lhs.angularVelocity, rhs.angularVelocity)});
}

compute::RigidBodyStateBatch rolloutCpu(
    compute::RigidBodyStateBatch state,
    const compute::RigidBodyModelBatch& model,
    const std::vector<double>& force,
    const std::size_t stepCount)
{
  for (std::size_t step = 0; step < stepCount; ++step) {
    compute::integrateRigidBodyStateBatch(state, model, force, kTimeStep);
  }
  return state;
}

struct ReducedSceneParityFixture
{
  ReducedSceneParityFixture(
      const std::size_t requestedWorldCount, const std::size_t requestedSteps)
    : worldCount(requestedWorldCount), stepCount(requestedSteps)
  {
    sx::World seed;
    populateReducedHangingBridgeWorld(seed);
    const compute::RigidBodyStateBatch singleState
        = compute::extractRigidBodyState(seed);
    const compute::RigidBodyModelBatch singleModel
        = compute::extractRigidBodyModelBatch(seed);

    initial = replicateState(singleState, worldCount);
    model = replicateModel(singleModel, worldCount);
    force = makeGravityForce(model);
    expected = rolloutCpu(initial, model, force, stepCount);
  }

  std::size_t worldCount = 0;
  std::size_t stepCount = 0;
  compute::RigidBodyStateBatch initial;
  compute::RigidBodyModelBatch model;
  std::vector<double> force;
  compute::RigidBodyStateBatch expected;
};

void setCommonCounters(
    benchmark::State& state,
    const ReducedSceneParityFixture& fixture,
    const double maxError)
{
  state.counters["worlds"] = static_cast<double>(fixture.worldCount);
  state.counters["bodies"] = static_cast<double>(fixture.initial.bodyCount);
  state.counters["steps"] = static_cast<double>(fixture.stepCount);
  state.counters["scene_body_count"] = static_cast<double>(kSceneBodyCount);
  state.counters["max_result_abs_error"] = maxError;
  state.SetItemsProcessed(
      static_cast<int64_t>(
          state.iterations() * fixture.worldCount * fixture.initial.bodyCount
          * fixture.stepCount));
}

} // namespace

//==============================================================================
static void BM_Plan083SceneParityCpu(benchmark::State& state)
{
  const auto worldCount = static_cast<std::size_t>(state.range(0));
  const auto stepCount = static_cast<std::size_t>(state.range(1));
  ReducedSceneParityFixture fixture(worldCount, stepCount);
  double maxError = 0.0;

  for (auto _ : state) {
    compute::RigidBodyStateBatch current
        = rolloutCpu(fixture.initial, fixture.model, fixture.force, stepCount);
    maxError = maxStateAbsDiff(current, fixture.expected);
    benchmark::DoNotOptimize(current.position.data());
    benchmark::DoNotOptimize(maxError);
    benchmark::ClobberMemory();
  }

  setCommonCounters(state, fixture, maxError);
}

//==============================================================================
static void BM_Plan083SceneParityCuda(benchmark::State& state)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    state.SkipWithError("CUDA runtime has no available device");
    return;
  }

  const auto worldCount = static_cast<std::size_t>(state.range(0));
  const auto stepCount = static_cast<std::size_t>(state.range(1));
  ReducedSceneParityFixture fixture(worldCount, stepCount);
  double maxError = 0.0;

  for (auto _ : state) {
    compute::RigidBodyStateBatch current = fixture.initial;
    cuda::rolloutRigidBodyStateBatchCuda(
        current, fixture.model, fixture.force, kTimeStep, stepCount);
    maxError = maxStateAbsDiff(current, fixture.expected);
    benchmark::DoNotOptimize(current.position.data());
    benchmark::DoNotOptimize(maxError);
    benchmark::ClobberMemory();
  }

  setCommonCounters(state, fixture, maxError);
}

BENCHMARK(BM_Plan083SceneParityCpu)->Args({1024, 16})->Args({4096, 64});
BENCHMARK(BM_Plan083SceneParityCuda)->Args({1024, 16})->Args({4096, 64});
