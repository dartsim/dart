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

#include "ContactContainerScene.hpp"

#ifdef DART_CONTACT_CONTAINER_BENCHMARK_HAS_BULLET
  #include <dart/collision/bullet/BulletCollisionDetector.hpp>
#endif
#include <dart/collision/dart/DARTCollisionDetector.hpp>
#include <dart/collision/fcl/FCLCollisionDetector.hpp>
#include <dart/collision/ode/OdeCollisionDetector.hpp>

#include <benchmark/benchmark.h>

namespace {

enum Engine
{
  DART = 0,
  ODE = 1,
  FCL = 2,
  BULLET = 3,
};

dart::collision::CollisionDetectorPtr makeDetector(int engine)
{
  switch (engine) {
    case ODE:
      return dart::collision::OdeCollisionDetector::create();
    case FCL:
      return dart::collision::FCLCollisionDetector::create();
#ifdef DART_CONTACT_CONTAINER_BENCHMARK_HAS_BULLET
    case BULLET:
      return dart::collision::BulletCollisionDetector::create();
#endif
    case DART:
    default:
      return dart::collision::DARTCollisionDetector::create();
  }
}

const char* engineName(int engine)
{
  switch (engine) {
    case ODE:
      return "ode";
    case FCL:
      return "fcl";
    case BULLET:
      return "bullet";
    case DART:
    default:
      return "dart";
  }
}

void runContactContainerBenchmark(
    benchmark::State& state, bool deactivationEnabled, std::size_t steps)
{
  const auto objects = static_cast<std::size_t>(state.range(0));
  const int engine = static_cast<int>(state.range(1));
  const auto threads = static_cast<std::size_t>(state.range(2));

  std::size_t finalContacts = 0;
  std::size_t finalResting = 0;
  std::size_t finalMobile = 0;

  for (auto _ : state) {
    state.PauseTiming();
    dart::examples::contact_benchmark::ContactContainerOptions options;
    options.objects = objects;
    options.layers = 4;
    options.spacing
        = dart::examples::contact_benchmark::kDefaultContactContainerSpacing;
    options.jitter
        = dart::examples::contact_benchmark::kDefaultContactContainerJitter;
    options.dropHeight
        = dart::examples::contact_benchmark::kDefaultContactContainerDropHeight;
    options.seed
        = dart::examples::contact_benchmark::kDefaultContactContainerSeed;
    options.collisionDetector = makeDetector(engine);

    auto world = dart::examples::contact_benchmark::createContactContainerWorld(
        options);
    world->setNumSimulationThreads(threads);
    auto deactivation = world->getDeactivationOptions();
    deactivation.mEnabled = deactivationEnabled;
    world->setDeactivationOptions(deactivation);
    world->getConstraintSolver()->getCollisionOption().maxNumContacts = 20000;
    world->getConstraintSolver()->getCollisionOption().maxNumContactsPerPair
        = 4;
    state.ResumeTiming();

    for (std::size_t step = 0; step < steps; ++step)
      world->step();

    state.PauseTiming();
    finalContacts = world->getLastCollisionResult().getNumContacts();
    finalResting = 0;
    finalMobile = 0;
    for (std::size_t i = 0; i < world->getNumSkeletons(); ++i) {
      const auto skel = world->getSkeleton(i);
      if (!skel || !skel->isMobile())
        continue;
      ++finalMobile;
      if (skel->isResting())
        ++finalResting;
    }
    benchmark::DoNotOptimize(world->getTime());
    world.reset();
    state.ResumeTiming();
  }

  state.SetItemsProcessed(
      static_cast<std::int64_t>(state.iterations() * steps * objects));
  state.SetLabel(
      std::string("engine=") + engineName(engine)
      + " threads=" + std::to_string(threads)
      + " deactivation=" + (deactivationEnabled ? "on" : "off")
      + " steps=" + std::to_string(steps));
  state.counters["sim_s/s"] = benchmark::Counter(
      static_cast<double>(state.iterations()) * static_cast<double>(steps)
          * 0.001,
      benchmark::Counter::kIsRate);
  state.counters["contacts"] = static_cast<double>(finalContacts);
  state.counters["resting"] = static_cast<double>(finalResting);
  state.counters["mobile"] = static_cast<double>(finalMobile);
  state.counters["steps"] = static_cast<double>(steps);
}

void BM_ContactContainerActive(benchmark::State& state)
{
  runContactContainerBenchmark(state, false, 200);
}

void BM_ContactContainerLargeActive(benchmark::State& state)
{
  runContactContainerBenchmark(state, false, 20);
}

void BM_ContactContainerDeactivation(benchmark::State& state)
{
  runContactContainerBenchmark(state, true, 20);
}

BENCHMARK(BM_ContactContainerActive)
    ->Args({60, DART, 1})
    ->Args({60, DART, 16})
    ->Args({60, ODE, 1})
    ->Args({60, ODE, 16})
    ->Args({120, DART, 1})
    ->Args({120, DART, 16})
    ->Args({120, ODE, 1})
    ->Args({120, ODE, 16})
    ->Args({60, FCL, 1})
    ->Args({60, FCL, 16})
    ->Args({120, FCL, 1})
    ->Args({120, FCL, 16})
#ifdef DART_CONTACT_CONTAINER_BENCHMARK_HAS_BULLET
    ->Args({60, BULLET, 1})
    ->Args({60, BULLET, 16})
    ->Args({120, BULLET, 1})
    ->Args({120, BULLET, 16})
#endif
    ->Args({120, DART, 4})
    ->Args({120, ODE, 4})
    ->Args({120, FCL, 4})
#ifdef DART_CONTACT_CONTAINER_BENCHMARK_HAS_BULLET
    ->Args({120, BULLET, 4})
#endif
    ->Unit(benchmark::kMillisecond);

BENCHMARK(BM_ContactContainerLargeActive)
    ->Args({900, DART, 16})
    ->Args({900, ODE, 16})
    ->Iterations(1)
    ->Unit(benchmark::kMillisecond);

BENCHMARK(BM_ContactContainerDeactivation)
    ->Args({60, DART, 16})
    ->Args({60, ODE, 16})
    ->Iterations(1)
    ->Unit(benchmark::kMillisecond);

} // namespace
