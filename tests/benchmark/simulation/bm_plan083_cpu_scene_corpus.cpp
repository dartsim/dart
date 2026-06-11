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

// PLAN-083 CPU scene corpus packet seed. These benchmarks measure reduced
// runtime smoke scenes only; they are not paper-scale bridge, cone-twist,
// codimensional-rod, or mixed-domain performance claims.

#include <dart/simulation/body/collision_shape.hpp>
#include <dart/simulation/body/rigid_body.hpp>
#include <dart/simulation/body/rigid_body_options.hpp>
#include <dart/simulation/compute/sequential_executor.hpp>
#include <dart/simulation/compute/world_step_stage.hpp>
#include <dart/simulation/multibody/joint.hpp>
#include <dart/simulation/world.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <benchmark/benchmark.h>

#include <algorithm>
#include <array>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include <cmath>

namespace sx = dart::simulation;

namespace {

constexpr std::array<double, 4> kBridgeBoardX{-0.45, -0.15, 0.15, 0.45};
constexpr double kMaxReducedEqualityResidual = 1e-8;
constexpr double kPi = 3.141592653589793238462643383279502884;

const Eigen::Vector3d kBridgeBoardHalfExtents(0.10, 0.16, 0.025);
const Eigen::Vector3d kBridgePostHalfExtents(0.05, 0.20, 0.08);
const Eigen::Vector3d kBridgeTravelerHalfExtents(0.07, 0.07, 0.07);
const Eigen::Vector3d kNunchakuHandleHalfExtents(0.18, 0.035, 0.035);
const Eigen::Vector3d kWindmillHubHalfExtents(0.05, 0.05, 0.05);
const Eigen::Vector3d kWindmillBladeHalfExtents(0.045, 0.22, 0.025);
const Eigen::Vector3d kWindmillStrikerHalfExtents(0.09, 0.09, 0.09);
const Eigen::Vector3d kTerrainHalfExtents(0.70, 0.45, 0.025);
const Eigen::Vector3d kTerrainChassisHalfExtents(0.22, 0.12, 0.04);
constexpr double kTerrainWheelRadius = 0.06;
const Eigen::Vector3d kPrecessionGroundHalfExtents(0.55, 0.45, 0.025);
constexpr double kPrecessionWheelRadius = 0.16;
constexpr double kPrecessionWheelHalfHeight = 0.035;
const std::array<Eigen::Vector3d, 4> kTerrainWheelOffsets{{
    Eigen::Vector3d(-0.16, -0.14, -0.10),
    Eigen::Vector3d(-0.16, 0.14, -0.10),
    Eigen::Vector3d(0.16, -0.14, -0.10),
    Eigen::Vector3d(0.16, 0.14, -0.10),
}};

sx::CollisionShape makeOffsetBox(
    const Eigen::Vector3d& halfExtents, const Eigen::Vector3d& localOffset)
{
  auto shape = sx::CollisionShape::makeBox(halfExtents);
  shape.localTransform.translation() = localOffset;
  return shape;
}

struct BodySnapshot
{
  sx::RigidBody body;
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  Eigen::Vector3d linearVelocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d angularVelocity = Eigen::Vector3d::Zero();
};

void snapshotBody(
    std::vector<BodySnapshot>& snapshots, const sx::RigidBody& body)
{
  snapshots.push_back(
      {body,
       body.getTransform(),
       body.getLinearVelocity(),
       body.getAngularVelocity()});
}

struct HangingBridgeFixture
{
  HangingBridgeFixture()
  {
    world.setRigidBodySolver(sx::RigidBodySolver::Ipc);
    world.setTimeStep(0.005);
    world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));

    sx::RigidBodyOptions leftPostOptions;
    leftPostOptions.isStatic = true;
    leftPostOptions.position = Eigen::Vector3d(-0.65, 0.0, 0.56);
    leftPost.emplace(
        world.addRigidBody("plan083_bridge_left_post", leftPostOptions));
    leftPost->setCollisionShape(
        sx::CollisionShape::makeBox(kBridgePostHalfExtents));

    sx::RigidBodyOptions rightPostOptions;
    rightPostOptions.isStatic = true;
    rightPostOptions.position = Eigen::Vector3d(0.65, 0.0, 0.56);
    rightPost.emplace(
        world.addRigidBody("plan083_bridge_right_post", rightPostOptions));
    rightPost->setCollisionShape(
        sx::CollisionShape::makeBox(kBridgePostHalfExtents));

    sx::RigidBody parent = *leftPost;
    boards.reserve(kBridgeBoardX.size());
    for (std::size_t index = 0; index < kBridgeBoardX.size(); ++index) {
      sx::RigidBodyOptions boardOptions;
      boardOptions.mass = 0.25;
      boardOptions.position = Eigen::Vector3d(kBridgeBoardX[index], 0.0, 0.50);
      auto board = world.addRigidBody(
          "plan083_bridge_board_" + std::to_string(index), boardOptions);
      board.setFriction(0.7);
      board.setCollisionShape(
          sx::CollisionShape::makeBox(kBridgeBoardHalfExtents));
      world.addRigidBodyFixedJoint(
          "plan083_bridge_point_connection_" + std::to_string(index),
          parent,
          board);
      boards.push_back(board);
      parent = board;
    }

    sx::RigidBodyOptions travelerOptions;
    travelerOptions.mass = 0.12;
    travelerOptions.position = Eigen::Vector3d(-0.60, 0.0, 0.82);
    travelerOptions.linearVelocity = Eigen::Vector3d(0.35, 0.0, -0.05);
    traveler.emplace(
        world.addRigidBody("plan083_bridge_traveler", travelerOptions));
    traveler->setFriction(0.4);
    traveler->setCollisionShape(
        sx::CollisionShape::makeBox(kBridgeTravelerHalfExtents));

    snapshotBody(snapshots, *leftPost);
    snapshotBody(snapshots, *rightPost);
    for (const auto& board : boards) {
      snapshotBody(snapshots, board);
    }
    snapshotBody(snapshots, *traveler);

    world.enterSimulationMode();
  }

  void reset()
  {
    world.setTime(0.0);
    for (auto& snapshot : snapshots) {
      snapshot.body.setTransform(snapshot.transform);
      snapshot.body.setLinearVelocity(snapshot.linearVelocity);
      snapshot.body.setAngularVelocity(snapshot.angularVelocity);
    }
  }

  double maxBoardSag() const
  {
    double minBoardZ = std::numeric_limits<double>::infinity();
    for (const auto& board : boards) {
      minBoardZ = std::min(minBoardZ, board.getTranslation().z());
    }
    return 0.50 - minBoardZ;
  }

  sx::World world;
  std::optional<sx::RigidBody> leftPost;
  std::optional<sx::RigidBody> rightPost;
  std::vector<sx::RigidBody> boards;
  std::optional<sx::RigidBody> traveler;
  std::vector<BodySnapshot> snapshots;
};

struct NunchakuFixture
{
  NunchakuFixture()
  {
    world.setRigidBodySolver(sx::RigidBodySolver::Ipc);
    world.setTimeStep(0.005);
    world.setGravity(Eigen::Vector3d::Zero());

    sx::RigidBodyOptions anchorOptions;
    anchorOptions.isStatic = true;
    anchorOptions.position = Eigen::Vector3d(0.0, 0.0, 0.75);
    anchor.emplace(
        world.addRigidBody("plan083_nunchaku_anchor_handle", anchorOptions));
    anchor->setCollisionShape(makeOffsetBox(
        kNunchakuHandleHalfExtents,
        Eigen::Vector3d(-kNunchakuHandleHalfExtents.x(), 0.0, 0.0)));

    sx::RigidBodyOptions swingOptions;
    swingOptions.mass = 0.2;
    swingOptions.position = anchorOptions.position;
    swingOptions.angularVelocity = Eigen::Vector3d(0.0, 0.0, 1.5);
    swinging.emplace(
        world.addRigidBody("plan083_nunchaku_swing_handle", swingOptions));
    swinging->setCollisionShape(makeOffsetBox(
        kNunchakuHandleHalfExtents,
        Eigen::Vector3d(kNunchakuHandleHalfExtents.x(), 0.0, 0.0)));

    (void)world.addRigidBodyRevoluteJoint(
        "plan083_nunchaku_hinge", *anchor, *swinging, Eigen::Vector3d::UnitZ());

    snapshotBody(snapshots, *anchor);
    snapshotBody(snapshots, *swinging);

    world.enterSimulationMode();
  }

  void reset()
  {
    world.setTime(0.0);
    for (auto& snapshot : snapshots) {
      snapshot.body.setTransform(snapshot.transform);
      snapshot.body.setLinearVelocity(snapshot.linearVelocity);
      snapshot.body.setAngularVelocity(snapshot.angularVelocity);
    }
  }

  double swingingTipRadius() const
  {
    const Eigen::Vector3d localTip(
        2.0 * kNunchakuHandleHalfExtents.x(), 0.0, 0.0);
    return (swinging->getTransform() * localTip - anchor->getTranslation())
        .norm();
  }

  sx::World world;
  std::optional<sx::RigidBody> anchor;
  std::optional<sx::RigidBody> swinging;
  std::vector<BodySnapshot> snapshots;
};

struct WindmillFixture
{
  WindmillFixture()
  {
    world.setRigidBodySolver(sx::RigidBodySolver::Ipc);
    world.setTimeStep(0.005);
    world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));

    sx::RigidBodyOptions hubOptions;
    hubOptions.isStatic = true;
    hubOptions.position = Eigen::Vector3d(0.0, 0.0, 0.65);
    hub.emplace(world.addRigidBody("plan083_windmill_hub", hubOptions));
    hub->setCollisionShape(
        sx::CollisionShape::makeBox(kWindmillHubHalfExtents));

    sx::RigidBodyOptions bladeOptions;
    bladeOptions.mass = 0.25;
    bladeOptions.position = hubOptions.position;
    bladeOptions.angularVelocity = Eigen::Vector3d(0.0, 1.2, 0.0);
    blade.emplace(world.addRigidBody("plan083_windmill_blade", bladeOptions));
    blade->setFriction(0.6);
    blade->setCollisionShape(makeOffsetBox(
        kWindmillBladeHalfExtents, Eigen::Vector3d(0.0, 0.0, 0.18)));

    sx::RigidBodyOptions strikerOptions;
    strikerOptions.mass = 0.18;
    strikerOptions.position = Eigen::Vector3d(0.0, 0.0, 0.955);
    strikerOptions.linearVelocity = Eigen::Vector3d(0.0, 0.0, -0.25);
    striker.emplace(
        world.addRigidBody("plan083_windmill_falling_box", strikerOptions));
    striker->setFriction(0.6);
    striker->setCollisionShape(
        sx::CollisionShape::makeBox(kWindmillStrikerHalfExtents));

    (void)world.addRigidBodyRevoluteJoint(
        "plan083_windmill_hinge", *hub, *blade, Eigen::Vector3d::UnitY());

    snapshotBody(snapshots, *hub);
    snapshotBody(snapshots, *blade);
    snapshotBody(snapshots, *striker);

    world.enterSimulationMode();
  }

  void reset()
  {
    world.setTime(0.0);
    for (auto& snapshot : snapshots) {
      snapshot.body.setTransform(snapshot.transform);
      snapshot.body.setLinearVelocity(snapshot.linearVelocity);
      snapshot.body.setAngularVelocity(snapshot.angularVelocity);
    }
  }

  double bladeTipRadius() const
  {
    const Eigen::Vector3d localTip(0.0, 0.0, 0.36);
    return (blade->getTransform() * localTip - hub->getTranslation()).norm();
  }

  double strikerBladeClearance() const
  {
    const double bladeTop
        = (blade->getTransform() * Eigen::Vector3d(0.0, 0.0, 0.18)).z()
          + kWindmillBladeHalfExtents.z();
    const double strikerBottom
        = striker->getTranslation().z() - kWindmillStrikerHalfExtents.z();
    return strikerBottom - bladeTop;
  }

  sx::World world;
  std::optional<sx::RigidBody> hub;
  std::optional<sx::RigidBody> blade;
  std::optional<sx::RigidBody> striker;
  std::vector<BodySnapshot> snapshots;
};

struct TerrainVehicleFixture
{
  TerrainVehicleFixture()
  {
    world.setRigidBodySolver(sx::RigidBodySolver::Ipc);
    world.setTimeStep(0.005);
    world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));

    sx::RigidBodyOptions terrainOptions;
    terrainOptions.isStatic = true;
    terrainOptions.position = Eigen::Vector3d(0.0, 0.0, -0.025);
    terrain.emplace(
        world.addRigidBody("plan083_vehicle_terrain", terrainOptions));
    terrain->setFriction(0.8);
    terrain->setCollisionShape(
        sx::CollisionShape::makeBox(kTerrainHalfExtents));

    sx::RigidBodyOptions chassisOptions;
    chassisOptions.mass = 0.45;
    chassisOptions.position = Eigen::Vector3d(0.0, 0.0, 0.17);
    chassisOptions.linearVelocity = Eigen::Vector3d(0.12, 0.0, 0.0);
    chassis.emplace(
        world.addRigidBody("plan083_vehicle_chassis", chassisOptions));
    chassis->setFriction(0.7);
    chassis->setCollisionShape(
        sx::CollisionShape::makeBox(kTerrainChassisHalfExtents));

    wheels.reserve(kTerrainWheelOffsets.size());
    for (std::size_t index = 0; index < kTerrainWheelOffsets.size(); ++index) {
      sx::RigidBodyOptions wheelOptions;
      wheelOptions.mass = 0.08;
      wheelOptions.position
          = chassisOptions.position + kTerrainWheelOffsets[index];
      wheelOptions.angularVelocity = Eigen::Vector3d(0.0, 3.0, 0.0);
      auto wheel = world.addRigidBody(
          "plan083_vehicle_passive_wheel_" + std::to_string(index),
          wheelOptions);
      wheel.setFriction(0.9);
      wheel.setCollisionShape(
          sx::CollisionShape::makeSphere(kTerrainWheelRadius));
      (void)world.addRigidBodyRevoluteJoint(
          "plan083_vehicle_wheel_hinge_" + std::to_string(index),
          *chassis,
          wheel,
          Eigen::Vector3d::UnitY());
      wheels.push_back(wheel);
    }

    snapshotBody(snapshots, *terrain);
    snapshotBody(snapshots, *chassis);
    for (const auto& wheel : wheels) {
      snapshotBody(snapshots, wheel);
    }

    world.enterSimulationMode();
  }

  void reset()
  {
    world.setTime(0.0);
    for (auto& snapshot : snapshots) {
      snapshot.body.setTransform(snapshot.transform);
      snapshot.body.setLinearVelocity(snapshot.linearVelocity);
      snapshot.body.setAngularVelocity(snapshot.angularVelocity);
    }
  }

  double minWheelGroundClearance() const
  {
    double clearance = std::numeric_limits<double>::infinity();
    for (const auto& wheel : wheels) {
      clearance = std::min(
          clearance, wheel.getTranslation().z() - kTerrainWheelRadius);
    }
    return clearance;
  }

  sx::World world;
  std::optional<sx::RigidBody> terrain;
  std::optional<sx::RigidBody> chassis;
  std::vector<sx::RigidBody> wheels;
  std::vector<BodySnapshot> snapshots;
};

struct PrecessionFixture
{
  PrecessionFixture()
  {
    world.setRigidBodySolver(sx::RigidBodySolver::Ipc);
    world.setTimeStep(0.005);
    world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));

    sx::RigidBodyOptions groundOptions;
    groundOptions.isStatic = true;
    groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.025);
    ground.emplace(
        world.addRigidBody("plan083_precession_ground", groundOptions));
    ground->setFriction(0.8);
    ground->setCollisionShape(
        sx::CollisionShape::makeBox(kPrecessionGroundHalfExtents));

    sx::RigidBodyOptions wheelOptions;
    wheelOptions.mass = 0.22;
    wheelOptions.position = Eigen::Vector3d(-0.18, 0.0, kPrecessionWheelRadius);
    wheelOptions.orientation = Eigen::Quaterniond(
        Eigen::AngleAxisd(-kPi / 2.0, Eigen::Vector3d::UnitX()));
    wheelOptions.linearVelocity = Eigen::Vector3d(0.28, 0.0, 0.0);
    wheelOptions.angularVelocity = Eigen::Vector3d(0.0, 8.0, 1.2);
    wheel.emplace(world.addRigidBody("plan083_precession_wheel", wheelOptions));
    wheel->setFriction(0.9);
    wheel->setCollisionShape(
        sx::CollisionShape::makeSphere(kPrecessionWheelRadius));

    snapshotBody(snapshots, *ground);
    snapshotBody(snapshots, *wheel);

    world.enterSimulationMode();
  }

  void reset()
  {
    world.setTime(0.0);
    for (auto& snapshot : snapshots) {
      snapshot.body.setTransform(snapshot.transform);
      snapshot.body.setLinearVelocity(snapshot.linearVelocity);
      snapshot.body.setAngularVelocity(snapshot.angularVelocity);
    }
  }

  double wheelGroundClearance() const
  {
    return wheel->getTranslation().z() - kPrecessionWheelRadius;
  }

  sx::World world;
  std::optional<sx::RigidBody> ground;
  std::optional<sx::RigidBody> wheel;
  std::vector<BodySnapshot> snapshots;
};

} // namespace

//==============================================================================
static void BM_Plan083CpuScene_hanging_bridge_reduced_world_step(
    benchmark::State& state)
{
  HangingBridgeFixture fixture;
  sx::compute::SequentialExecutor executor;

  std::size_t failedSteps = 0;
  sx::compute::RigidIpcSolverStats lastStats;
  for (auto _ : state) {
    state.PauseTiming();
    fixture.reset();
    sx::compute::RigidIpcContactStage ipcStage(64);
    sx::compute::WorldStepPipeline pipeline;
    pipeline.addStage(ipcStage);
    state.ResumeTiming();

    fixture.world.step(executor, pipeline);
    lastStats = ipcStage.getLastStats();
    const bool residualOk
        = std::isfinite(lastStats.finalEqualityResidualNorm)
          && lastStats.finalEqualityResidualNorm <= kMaxReducedEqualityResidual;
    const bool satisfiedNoOp = residualOk && lastStats.solverIterations == 0u
                               && lastStats.activeArticulationConstraints >= 2u;
    if (!residualOk || (lastStats.failed && !satisfiedNoOp)) {
      ++failedSteps;
    }
    benchmark::DoNotOptimize(fixture.traveler->getTranslation().data());
    benchmark::ClobberMemory();
  }

  state.counters["row_unb_fig_02"] = 1.0;
  state.counters["paper_scale"] = 0.0;
  state.counters["body_count"] = static_cast<double>(lastStats.bodyCount);
  state.counters["dynamic_body_count"]
      = static_cast<double>(lastStats.dynamicBodyCount);
  state.counters["fixed_joint_count"]
      = static_cast<double>(fixture.boards.size());
  state.counters["active_articulation_constraints"]
      = static_cast<double>(lastStats.activeArticulationConstraints);
  state.counters["solver_iterations"]
      = static_cast<double>(lastStats.solverIterations);
  state.counters["failed_steps"] = static_cast<double>(failedSteps);
  state.counters["final_equality_residual_norm"]
      = lastStats.finalEqualityResidualNorm;
  state.counters["traveler_height_m"] = fixture.traveler->getTranslation().z();
  state.counters["max_board_sag_m"] = fixture.maxBoardSag();
}
BENCHMARK(BM_Plan083CpuScene_hanging_bridge_reduced_world_step)
    ->Unit(benchmark::kMillisecond);

//==============================================================================
static void BM_Plan083CpuScene_nunchaku_single_reduced_world_step(
    benchmark::State& state)
{
  NunchakuFixture fixture;
  sx::compute::SequentialExecutor executor;

  std::size_t failedSteps = 0;
  sx::compute::RigidIpcSolverStats lastStats;
  for (auto _ : state) {
    state.PauseTiming();
    fixture.reset();
    sx::compute::RigidIpcContactStage ipcStage(64);
    sx::compute::WorldStepPipeline pipeline;
    pipeline.addStage(ipcStage);
    state.ResumeTiming();

    fixture.world.step(executor, pipeline);
    lastStats = ipcStage.getLastStats();
    const bool residualOk
        = std::isfinite(lastStats.finalEqualityResidualNorm)
          && lastStats.finalEqualityResidualNorm <= kMaxReducedEqualityResidual;
    const bool satisfiedNoOp = residualOk && lastStats.solverIterations == 0u
                               && lastStats.activeArticulationConstraints >= 2u;
    if (!residualOk || (lastStats.failed && !satisfiedNoOp)) {
      ++failedSteps;
    }
    benchmark::DoNotOptimize(fixture.swinging->getTransform().data());
    benchmark::ClobberMemory();
  }

  state.counters["row_unb_fig_13"] = 1.0;
  state.counters["paper_scale"] = 0.0;
  state.counters["body_count"] = static_cast<double>(lastStats.bodyCount);
  state.counters["dynamic_body_count"]
      = static_cast<double>(lastStats.dynamicBodyCount);
  state.counters["revolute_joint_count"]
      = static_cast<double>(fixture.world.getRigidBodyJointCount());
  state.counters["active_articulation_constraints"]
      = static_cast<double>(lastStats.activeArticulationConstraints);
  state.counters["solver_iterations"]
      = static_cast<double>(lastStats.solverIterations);
  state.counters["failed_steps"] = static_cast<double>(failedSteps);
  state.counters["final_equality_residual_norm"]
      = lastStats.finalEqualityResidualNorm;
  state.counters["swinging_tip_radius_m"] = fixture.swingingTipRadius();
  state.counters["free_axis_angular_velocity_rad_s"]
      = fixture.swinging->getAngularVelocity().z();
}
BENCHMARK(BM_Plan083CpuScene_nunchaku_single_reduced_world_step)
    ->Unit(benchmark::kMillisecond);

//==============================================================================
static void BM_Plan083CpuScene_windmill_reduced_world_step(
    benchmark::State& state)
{
  WindmillFixture fixture;
  sx::compute::SequentialExecutor executor;

  std::size_t failedSteps = 0;
  sx::compute::RigidIpcSolverStats lastStats;
  for (auto _ : state) {
    state.PauseTiming();
    fixture.reset();
    sx::compute::RigidIpcContactStage ipcStage(64);
    sx::compute::WorldStepPipeline pipeline;
    pipeline.addStage(ipcStage);
    state.ResumeTiming();

    fixture.world.step(executor, pipeline);
    lastStats = ipcStage.getLastStats();
    const bool residualOk
        = std::isfinite(lastStats.finalEqualityResidualNorm)
          && lastStats.finalEqualityResidualNorm <= kMaxReducedEqualityResidual;
    const bool satisfiedNoOp = residualOk && lastStats.solverIterations == 0u
                               && lastStats.activeArticulationConstraints >= 2u;
    if (!residualOk || (lastStats.failed && !satisfiedNoOp)) {
      ++failedSteps;
    }
    benchmark::DoNotOptimize(fixture.blade->getTransform().data());
    benchmark::DoNotOptimize(fixture.striker->getTranslation().data());
    benchmark::ClobberMemory();
  }

  state.counters["row_unb_fig_20"] = 1.0;
  state.counters["paper_scale"] = 0.0;
  state.counters["body_count"] = static_cast<double>(lastStats.bodyCount);
  state.counters["dynamic_body_count"]
      = static_cast<double>(lastStats.dynamicBodyCount);
  state.counters["revolute_joint_count"]
      = static_cast<double>(fixture.world.getRigidBodyJointCount());
  state.counters["active_constraints"]
      = static_cast<double>(lastStats.activeConstraints);
  state.counters["active_friction_constraints"]
      = static_cast<double>(lastStats.activeFrictionConstraints);
  state.counters["active_articulation_constraints"]
      = static_cast<double>(lastStats.activeArticulationConstraints);
  state.counters["solver_iterations"]
      = static_cast<double>(lastStats.solverIterations);
  state.counters["failed_steps"] = static_cast<double>(failedSteps);
  state.counters["final_equality_residual_norm"]
      = lastStats.finalEqualityResidualNorm;
  state.counters["blade_tip_radius_m"] = fixture.bladeTipRadius();
  state.counters["striker_height_m"] = fixture.striker->getTranslation().z();
  state.counters["striker_blade_clearance_m"] = fixture.strikerBladeClearance();
}
BENCHMARK(BM_Plan083CpuScene_windmill_reduced_world_step)
    ->Unit(benchmark::kMillisecond);

//==============================================================================
static void BM_Plan083CpuScene_terrain_vehicle_reduced_world_step(
    benchmark::State& state)
{
  TerrainVehicleFixture fixture;
  sx::compute::SequentialExecutor executor;

  std::size_t failedSteps = 0;
  sx::compute::RigidIpcSolverStats lastStats;
  for (auto _ : state) {
    state.PauseTiming();
    fixture.reset();
    sx::compute::RigidIpcContactStage ipcStage(64);
    sx::compute::WorldStepPipeline pipeline;
    pipeline.addStage(ipcStage);
    state.ResumeTiming();

    fixture.world.step(executor, pipeline);
    lastStats = ipcStage.getLastStats();
    const bool residualOk
        = std::isfinite(lastStats.finalEqualityResidualNorm)
          && lastStats.finalEqualityResidualNorm <= kMaxReducedEqualityResidual;
    const bool satisfiedNoOp = residualOk && lastStats.solverIterations == 0u
                               && lastStats.activeArticulationConstraints >= 8u;
    if (!residualOk || (lastStats.failed && !satisfiedNoOp)) {
      ++failedSteps;
    }
    benchmark::DoNotOptimize(fixture.chassis->getTranslation().data());
    benchmark::DoNotOptimize(fixture.wheels.front().getTransform().data());
    benchmark::ClobberMemory();
  }

  state.counters["row_unb_fig_10"] = 1.0;
  state.counters["paper_scale"] = 0.0;
  state.counters["body_count"] = static_cast<double>(lastStats.bodyCount);
  state.counters["dynamic_body_count"]
      = static_cast<double>(lastStats.dynamicBodyCount);
  state.counters["wheel_count"] = static_cast<double>(fixture.wheels.size());
  state.counters["revolute_joint_count"]
      = static_cast<double>(fixture.world.getRigidBodyJointCount());
  state.counters["active_constraints"]
      = static_cast<double>(lastStats.activeConstraints);
  state.counters["active_friction_constraints"]
      = static_cast<double>(lastStats.activeFrictionConstraints);
  state.counters["active_articulation_constraints"]
      = static_cast<double>(lastStats.activeArticulationConstraints);
  state.counters["solver_iterations"]
      = static_cast<double>(lastStats.solverIterations);
  state.counters["failed_steps"] = static_cast<double>(failedSteps);
  state.counters["final_equality_residual_norm"]
      = lastStats.finalEqualityResidualNorm;
  state.counters["chassis_height_m"] = fixture.chassis->getTranslation().z();
  state.counters["min_wheel_ground_clearance_m"]
      = fixture.minWheelGroundClearance();
}
BENCHMARK(BM_Plan083CpuScene_terrain_vehicle_reduced_world_step)
    ->Unit(benchmark::kMillisecond);

//==============================================================================
static void BM_Plan083CpuScene_precession_reduced_world_step(
    benchmark::State& state)
{
  PrecessionFixture fixture;
  sx::compute::SequentialExecutor executor;

  std::size_t failedSteps = 0;
  sx::compute::RigidIpcSolverStats lastStats;
  for (auto _ : state) {
    state.PauseTiming();
    fixture.reset();
    sx::compute::RigidIpcContactStage ipcStage(64);
    sx::compute::WorldStepPipeline pipeline;
    pipeline.addStage(ipcStage);
    state.ResumeTiming();

    fixture.world.step(executor, pipeline);
    lastStats = ipcStage.getLastStats();
    const bool residualOk
        = std::isfinite(lastStats.finalEqualityResidualNorm)
          && lastStats.finalEqualityResidualNorm <= kMaxReducedEqualityResidual;
    const bool satisfiedNoOp = residualOk && lastStats.solverIterations == 0u
                               && lastStats.activeConstraints >= 1u;
    if (!residualOk || (lastStats.failed && !satisfiedNoOp)) {
      ++failedSteps;
    }
    benchmark::DoNotOptimize(fixture.wheel->getTransform().data());
    benchmark::ClobberMemory();
  }

  state.counters["row_unb_fig_23"] = 1.0;
  state.counters["paper_scale"] = 0.0;
  state.counters["body_count"] = static_cast<double>(lastStats.bodyCount);
  state.counters["dynamic_body_count"]
      = static_cast<double>(lastStats.dynamicBodyCount);
  state.counters["active_constraints"]
      = static_cast<double>(lastStats.activeConstraints);
  state.counters["active_friction_constraints"]
      = static_cast<double>(lastStats.activeFrictionConstraints);
  state.counters["solver_iterations"]
      = static_cast<double>(lastStats.solverIterations);
  state.counters["failed_steps"] = static_cast<double>(failedSteps);
  state.counters["final_equality_residual_norm"]
      = lastStats.finalEqualityResidualNorm;
  state.counters["wheel_height_m"] = fixture.wheel->getTranslation().z();
  state.counters["wheel_ground_clearance_m"] = fixture.wheelGroundClearance();
  state.counters["spin_rate_rad_s"]
      = fixture.wheel->getAngularVelocity().norm();
}
BENCHMARK(BM_Plan083CpuScene_precession_reduced_world_step)
    ->Unit(benchmark::kMillisecond);

BENCHMARK_MAIN();
