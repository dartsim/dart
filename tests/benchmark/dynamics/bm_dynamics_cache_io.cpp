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

/// @file bm_dynamics_cache_io.cpp
/// @brief Dynamics benchmarks using real robot models loaded via dart-io.
///
/// Separate binary from bm_dynamics_cache because this requires dart-io
/// linkage.
///
/// Run:  pixi run bm -- dynamics_cache_io

#include <dart/config.hpp>

#include <dart/simulation/world.hpp>

#include <dart/dynamics/degree_of_freedom.hpp>
#include <dart/dynamics/joint.hpp>
#include <dart/dynamics/skeleton.hpp>

#include <dart/math/random.hpp>

#include <dart/common/uri.hpp>

#include <dart/io/read.hpp>

#include <benchmark/benchmark.h>

using namespace dart;
using namespace dart::dynamics;
using namespace dart::simulation;

// ============================================================================
// Helpers (mirrored from bm_dynamics_cache.cpp)
// ============================================================================

inline void randomizeState(const SkeletonPtr& skel)
{
  for (std::size_t i = 0; i < skel->getNumDofs(); ++i) {
    auto* dof = skel->getDof(i);
    dof->setPosition(math::Random::uniform(-0.5, 0.5));
    dof->setVelocity(math::Random::uniform(-1.0, 1.0));
  }
}

inline void dirtyAllJoints(const SkeletonPtr& skel)
{
  for (std::size_t i = 0; i < skel->getNumJoints(); ++i) {
    skel->getJoint(i)->notifyPositionUpdated();
  }
}

static SkeletonPtr loadModel(
    const std::string& uri, const io::ReadOptions& options = io::ReadOptions())
{
  auto skel = dart::io::readSkeleton(common::Uri(uri), options);
  if (skel) {
    return skel;
  }
  auto world = dart::io::readWorld(common::Uri(uri), options);
  if (world && world->getNumSkeletons() > 0) {
    return world->getSkeleton(0);
  }
  return nullptr;
}

// Package directory options for URDFs that use package:// mesh URIs
static io::ReadOptions wamOptions()
{
  io::ReadOptions opts;
  opts.addPackageDirectory("herb_description", config::dataPath("urdf/wam"));
  return opts;
}

static io::ReadOptions drchuboOptions()
{
  io::ReadOptions opts;
  opts.addPackageDirectory("drchubo", config::dataPath("urdf/drchubo"));
  return opts;
}

// ============================================================================
// KR5 (6-DOF industrial arm)
// ============================================================================

static void BM_Robot_KR5_ForwardKinematics(benchmark::State& state)
{
  auto skel = loadModel("dart://sample/urdf/KR5/KR5 sixx R650.urdf");
  if (!skel) {
    state.SkipWithError("Failed to load KR5 model");
    return;
  }
  randomizeState(skel);

  for (auto _ : state) {
    state.PauseTiming();
    dirtyAllJoints(skel);
    randomizeState(skel);
    state.ResumeTiming();
    skel->computeForwardKinematics(true, true, true);
  }
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations()));
  state.counters["bodies"] = static_cast<double>(skel->getNumBodyNodes());
  state.counters["dofs"] = static_cast<double>(skel->getNumDofs());
}
BENCHMARK(BM_Robot_KR5_ForwardKinematics)->MinTime(0.1);

static void BM_Robot_KR5_ForwardDynamics(benchmark::State& state)
{
  auto skel = loadModel("dart://sample/urdf/KR5/KR5 sixx R650.urdf");
  if (!skel) {
    state.SkipWithError("Failed to load KR5 model");
    return;
  }
  randomizeState(skel);

  for (auto _ : state) {
    state.PauseTiming();
    dirtyAllJoints(skel);
    randomizeState(skel);
    state.ResumeTiming();
    skel->computeForwardDynamics();
  }
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations()));
  state.counters["bodies"] = static_cast<double>(skel->getNumBodyNodes());
  state.counters["dofs"] = static_cast<double>(skel->getNumDofs());
}
BENCHMARK(BM_Robot_KR5_ForwardDynamics)->MinTime(0.1);

static void BM_Robot_KR5_WorldStep(benchmark::State& state)
{
  auto skel = loadModel("dart://sample/urdf/KR5/KR5 sixx R650.urdf");
  if (!skel) {
    state.SkipWithError("Failed to load KR5 model");
    return;
  }
  randomizeState(skel);

  auto world = World::create();
  world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world->setTimeStep(0.001);
  world->addSkeleton(skel);

  for (auto _ : state) {
    world->step();
  }
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations()));
  state.counters["bodies"] = static_cast<double>(skel->getNumBodyNodes());
  state.counters["dofs"] = static_cast<double>(skel->getNumDofs());
}
BENCHMARK(BM_Robot_KR5_WorldStep)->MinTime(0.1);

// ============================================================================
// WAM (7-DOF arm) — requires package directory for mesh resolution
// ============================================================================

static void BM_Robot_WAM_ForwardKinematics(benchmark::State& state)
{
  auto skel = loadModel("dart://sample/urdf/wam/wam.urdf", wamOptions());
  if (!skel) {
    state.SkipWithError("Failed to load WAM model");
    return;
  }
  randomizeState(skel);

  for (auto _ : state) {
    state.PauseTiming();
    dirtyAllJoints(skel);
    randomizeState(skel);
    state.ResumeTiming();
    skel->computeForwardKinematics(true, true, true);
  }
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations()));
  state.counters["bodies"] = static_cast<double>(skel->getNumBodyNodes());
  state.counters["dofs"] = static_cast<double>(skel->getNumDofs());
}
BENCHMARK(BM_Robot_WAM_ForwardKinematics)->MinTime(0.1);

static void BM_Robot_WAM_ForwardDynamics(benchmark::State& state)
{
  auto skel = loadModel("dart://sample/urdf/wam/wam.urdf", wamOptions());
  if (!skel) {
    state.SkipWithError("Failed to load WAM model");
    return;
  }
  randomizeState(skel);

  for (auto _ : state) {
    state.PauseTiming();
    dirtyAllJoints(skel);
    randomizeState(skel);
    state.ResumeTiming();
    skel->computeForwardDynamics();
  }
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations()));
  state.counters["bodies"] = static_cast<double>(skel->getNumBodyNodes());
  state.counters["dofs"] = static_cast<double>(skel->getNumDofs());
}
BENCHMARK(BM_Robot_WAM_ForwardDynamics)->MinTime(0.1);

static void BM_Robot_WAM_WorldStep(benchmark::State& state)
{
  auto skel = loadModel("dart://sample/urdf/wam/wam.urdf", wamOptions());
  if (!skel) {
    state.SkipWithError("Failed to load WAM model");
    return;
  }
  randomizeState(skel);

  auto world = World::create();
  world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world->setTimeStep(0.001);
  world->addSkeleton(skel);

  for (auto _ : state) {
    world->step();
  }
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations()));
  state.counters["bodies"] = static_cast<double>(skel->getNumBodyNodes());
  state.counters["dofs"] = static_cast<double>(skel->getNumDofs());
}
BENCHMARK(BM_Robot_WAM_WorldStep)->MinTime(0.1);

// ============================================================================
// DRCHubo (~30 DOF humanoid) — requires package directory for mesh resolution
// ============================================================================

static void BM_Robot_DRCHubo_ForwardKinematics(benchmark::State& state)
{
  auto skel
      = loadModel("dart://sample/urdf/drchubo/drchubo.urdf", drchuboOptions());
  if (!skel) {
    state.SkipWithError("Failed to load DRCHubo model");
    return;
  }
  randomizeState(skel);

  for (auto _ : state) {
    state.PauseTiming();
    dirtyAllJoints(skel);
    randomizeState(skel);
    state.ResumeTiming();
    skel->computeForwardKinematics(true, true, true);
  }
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations()));
  state.counters["bodies"] = static_cast<double>(skel->getNumBodyNodes());
  state.counters["dofs"] = static_cast<double>(skel->getNumDofs());
}
BENCHMARK(BM_Robot_DRCHubo_ForwardKinematics)->MinTime(0.1);

static void BM_Robot_DRCHubo_ForwardDynamics(benchmark::State& state)
{
  auto skel
      = loadModel("dart://sample/urdf/drchubo/drchubo.urdf", drchuboOptions());
  if (!skel) {
    state.SkipWithError("Failed to load DRCHubo model");
    return;
  }
  randomizeState(skel);

  for (auto _ : state) {
    state.PauseTiming();
    dirtyAllJoints(skel);
    randomizeState(skel);
    state.ResumeTiming();
    skel->computeForwardDynamics();
  }
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations()));
  state.counters["bodies"] = static_cast<double>(skel->getNumBodyNodes());
  state.counters["dofs"] = static_cast<double>(skel->getNumDofs());
}
BENCHMARK(BM_Robot_DRCHubo_ForwardDynamics)->MinTime(0.1);

static void BM_Robot_DRCHubo_WorldStep(benchmark::State& state)
{
  auto skel
      = loadModel("dart://sample/urdf/drchubo/drchubo.urdf", drchuboOptions());
  if (!skel) {
    state.SkipWithError("Failed to load DRCHubo model");
    return;
  }
  randomizeState(skel);

  auto world = World::create();
  world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world->setTimeStep(0.001);
  world->addSkeleton(skel);

  for (auto _ : state) {
    world->step();
  }
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations()));
  state.counters["bodies"] = static_cast<double>(skel->getNumBodyNodes());
  state.counters["dofs"] = static_cast<double>(skel->getNumDofs());
}
BENCHMARK(BM_Robot_DRCHubo_WorldStep)->MinTime(0.1);

// ============================================================================
// Atlas (humanoid, loaded via URDF in sdf directory)
// ============================================================================

static void BM_Robot_Atlas_ForwardKinematics(benchmark::State& state)
{
  auto skel = loadModel("dart://sample/sdf/atlas/atlas_v3_no_head.urdf");
  if (!skel) {
    state.SkipWithError("Failed to load Atlas model");
    return;
  }
  randomizeState(skel);

  for (auto _ : state) {
    state.PauseTiming();
    dirtyAllJoints(skel);
    randomizeState(skel);
    state.ResumeTiming();
    skel->computeForwardKinematics(true, true, true);
  }
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations()));
  state.counters["bodies"] = static_cast<double>(skel->getNumBodyNodes());
  state.counters["dofs"] = static_cast<double>(skel->getNumDofs());
}
BENCHMARK(BM_Robot_Atlas_ForwardKinematics)->MinTime(0.1);

static void BM_Robot_Atlas_ForwardDynamics(benchmark::State& state)
{
  auto skel = loadModel("dart://sample/sdf/atlas/atlas_v3_no_head.urdf");
  if (!skel) {
    state.SkipWithError("Failed to load Atlas model");
    return;
  }
  randomizeState(skel);

  for (auto _ : state) {
    state.PauseTiming();
    dirtyAllJoints(skel);
    randomizeState(skel);
    state.ResumeTiming();
    skel->computeForwardDynamics();
  }
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations()));
  state.counters["bodies"] = static_cast<double>(skel->getNumBodyNodes());
  state.counters["dofs"] = static_cast<double>(skel->getNumDofs());
}
BENCHMARK(BM_Robot_Atlas_ForwardDynamics)->MinTime(0.1);

static void BM_Robot_Atlas_WorldStep(benchmark::State& state)
{
  auto skel = loadModel("dart://sample/sdf/atlas/atlas_v3_no_head.urdf");
  if (!skel) {
    state.SkipWithError("Failed to load Atlas model");
    return;
  }
  randomizeState(skel);

  auto world = World::create();
  world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world->setTimeStep(0.001);
  world->addSkeleton(skel);

  for (auto _ : state) {
    world->step();
  }
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations()));
  state.counters["bodies"] = static_cast<double>(skel->getNumBodyNodes());
  state.counters["dofs"] = static_cast<double>(skel->getNumDofs());
}
BENCHMARK(BM_Robot_Atlas_WorldStep)->MinTime(0.1);
