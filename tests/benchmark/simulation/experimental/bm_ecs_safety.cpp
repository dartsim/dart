/*
 * Copyright (c) 2011-2025, The DART development contributors
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

#include <dart/simulation/experimental/common/ecs_utils.hpp>
#include <dart/simulation/experimental/comps/name.hpp>

#include <benchmark/benchmark.h>
#include <entt/entt.hpp>

// Benchmark direct registry.get() (unsafe but fast baseline)
static void BM_DirectGet(benchmark::State& state)
{
  entt::registry registry;
  auto entity = registry.create();
  registry.emplace<dart::simulation::experimental::comps::Name>(entity, "test");

  for (auto _ : state) {
    auto& comp
        = registry.get<dart::simulation::experimental::comps::Name>(entity);
    benchmark::DoNotOptimize(comp);
  }
}
BENCHMARK(BM_DirectGet);

// Benchmark safeGet() (safe with minimal overhead)
static void BM_SafeGet(benchmark::State& state)
{
  entt::registry registry;
  auto entity = registry.create();
  registry.emplace<dart::simulation::experimental::comps::Name>(entity, "test");

  for (auto _ : state) {
    auto& comp = dart::simulation::experimental::safeGet<
        dart::simulation::experimental::comps::Name>(registry, entity);
    benchmark::DoNotOptimize(comp);
  }
}
BENCHMARK(BM_SafeGet);

// Benchmark try_get() + manual check (what safeGet does internally)
static void BM_TryGetManual(benchmark::State& state)
{
  entt::registry registry;
  auto entity = registry.create();
  registry.emplace<dart::simulation::experimental::comps::Name>(entity, "test");

  for (auto _ : state) {
    auto* comp
        = registry.try_get<dart::simulation::experimental::comps::Name>(entity);
    if (comp == nullptr) {
      state.SkipWithError("Component not found");
      break;
    }
    benchmark::DoNotOptimize(*comp);
  }
}
BENCHMARK(BM_TryGetManual);

BENCHMARK_MAIN();
