/*
 * Copyright (c) 2011-2026, The DART development contributors
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
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include "dart/collision/CollisionDetector.hpp"
#include "dart/constraint/ConstraintSolver.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/dynamics/SoftBodyNode.hpp"
#include "dart/simulation/World.hpp"
#include "dart/utils/SkelParser.hpp"

#if HAVE_BULLET
  #include "dart/collision/bullet/bullet.hpp"
#endif
#if HAVE_ODE
  #include "dart/collision/ode/ode.hpp"
#endif

#include <benchmark/benchmark.h>

#include <array>
#include <string>

#include <cstddef>
#include <cstdint>
#include <cstdlib>

namespace {

struct SoftBodyScene
{
  const char* name;
  const char* uri;
};

constexpr std::array<SoftBodyScene, 4> kScenes = {{
    {"adaptive_deformable",
     "dart://sample/skel/test/test_adaptive_deformable.skel"},
    {"soft_cubes", "dart://sample/skel/soft_cubes.skel"},
    {"soft_bodies", "dart://sample/skel/softBodies.skel"},
    {"soft_open_chain", "dart://sample/skel/soft_open_chain.skel"},
}};

struct SceneStats
{
  std::size_t softBodies = 0u;
  std::size_t pointMasses = 0u;
};

SceneStats collectSceneStats(const dart::simulation::WorldPtr& world)
{
  SceneStats stats;
  if (!world)
    return stats;

  for (std::size_t i = 0; i < world->getNumSkeletons(); ++i) {
    const auto skeleton = world->getSkeleton(i);
    if (!skeleton)
      continue;

    stats.softBodies += skeleton->getNumSoftBodyNodes();
    for (std::size_t j = 0; j < skeleton->getNumSoftBodyNodes(); ++j) {
      const auto* softBody = skeleton->getSoftBodyNode(j);
      if (softBody)
        stats.pointMasses += softBody->getNumPointMasses();
    }
  }

  return stats;
}

void keepOptionalCollisionBackendsLinked()
{
#if HAVE_BULLET
  benchmark::DoNotOptimize(
      &dart::collision::BulletCollisionDetector::getStaticType());
#endif
#if HAVE_ODE
  benchmark::DoNotOptimize(
      &dart::collision::OdeCollisionDetector::getStaticType());
#endif
}

void BM_SoftBodyStep(benchmark::State& state)
{
  keepOptionalCollisionBackendsLinked();

  const std::size_t sceneIndex = static_cast<std::size_t>(state.range(0));
  if (sceneIndex >= kScenes.size()) {
    state.SkipWithError("invalid soft-body scene index");
    return;
  }

  const auto& scene = kScenes[sceneIndex];
  const auto threads = static_cast<std::size_t>(state.range(1));
  const auto steps = static_cast<std::size_t>(state.range(2));
  SceneStats finalStats;
  std::string detectorName;
  double timeStep = 0.0;

  for (auto _ : state) {
    state.PauseTiming();
    auto world = dart::utils::SkelParser::readWorld(scene.uri);
    if (!world) {
      state.SkipWithError("failed to load soft-body scene");
      return;
    }

    if (const char* detectorEnv = std::getenv("COLLISION_DETECTOR");
        detectorEnv != nullptr && detectorEnv[0] != '\0') {
      auto detector = dart::collision::CollisionDetector::getFactory()->create(
          detectorEnv);
      if (!detector) {
        state.SkipWithError("failed to create collision detector");
        return;
      }
      detectorName = detector->getType();
      world->getConstraintSolver()->setCollisionDetector(std::move(detector));
    } else if (
        const auto detector
        = world->getConstraintSolver()->getCollisionDetector()) {
      detectorName = detector->getType();
    } else {
      detectorName = "none";
    }

    world->setNumSimulationThreads(threads);
    timeStep = world->getTimeStep();
    finalStats = collectSceneStats(world);
    // Keep one-time lazy collision/simulation preparation out of the measured
    // loop so rows compare steady-state soft-body stepping.
    world->step();
    state.ResumeTiming();

    for (std::size_t i = 0; i < steps; ++i)
      world->step();

    benchmark::DoNotOptimize(world->getTime());
  }

  state.SetItemsProcessed(static_cast<std::int64_t>(
      state.iterations() * steps * finalStats.pointMasses));
  state.SetLabel(
      std::string("scene=") + scene.name + " detector=" + detectorName
      + " threads=" + std::to_string(threads));
  state.counters["sim_s/s"] = benchmark::Counter(
      static_cast<double>(state.iterations()) * static_cast<double>(steps)
          * timeStep,
      benchmark::Counter::kIsRate);
  state.counters["soft_bodies"] = static_cast<double>(finalStats.softBodies);
  state.counters["point_masses"] = static_cast<double>(finalStats.pointMasses);
  state.counters["threads"] = static_cast<double>(threads);
}

BENCHMARK(BM_SoftBodyStep)
    ->Args({0, 1, 200})
    ->Args({0, 16, 200})
    ->Args({1, 1, 200})
    ->Args({1, 16, 200})
    ->Args({2, 1, 200})
    ->Args({2, 16, 200})
    ->Args({3, 1, 200})
    ->Args({3, 16, 200})
    ->Unit(benchmark::kMillisecond);

} // namespace
