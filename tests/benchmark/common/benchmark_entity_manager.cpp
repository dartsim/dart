/*
 * Copyright (c) 2011-2023, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#include <dart/common/common.hpp>

#include <benchmark/benchmark.h>

using namespace dart;
using namespace common;

// Define a benchmark fixture that sets up an EntityManager with a large number
// of entities
class EntityManagerFixture : public benchmark::Fixture
{
public:
  void SetUp(const benchmark::State& state)
  {
    num_entities = state.range(0);
    for (int i = 0; i < num_entities; ++i) {
      entities.push_back(em.create());
    }
  }

  void TearDown(const benchmark::State&)
  {
    for (const auto& entity : entities) {
      em.destroy(entity);
    }
  }

  // The number of entities to create in the test
  int num_entities = 0;

  // The entities created in the test
  std::vector<Entity> entities;

  // The entity manager used in the test
  EntityManagerT<> em;
};

// Measure the performance of creating many entities
BENCHMARK_DEFINE_F(EntityManagerFixture, CreateManyEntities)
(benchmark::State& state)
{
  for (auto _ : state) {
    benchmark::DoNotOptimize(em.create());
  }
}
BENCHMARK_REGISTER_F(EntityManagerFixture, CreateManyEntities)
    ->RangeMultiplier(10)
    ->Range(10, 1000000);

// Measure the performance of destroying many entities
BENCHMARK_DEFINE_F(EntityManagerFixture, DestroyManyEntities)
(benchmark::State& state)
{
  for (auto _ : state) {
    em.destroy(entities.back());
    entities.pop_back();
    if (entities.empty()) {
      for (int i = 0; i < num_entities; ++i) {
        entities.push_back(em.create());
      }
    }
  }
}
BENCHMARK_REGISTER_F(EntityManagerFixture, DestroyManyEntities)
    ->RangeMultiplier(10)
    ->Range(10, 1000000);

// Measure the performance of creating and destroying entities in a loop
BENCHMARK_DEFINE_F(EntityManagerFixture, CreateAndDestroyEntities)
(benchmark::State& state)
{
  while (state.KeepRunning()) {
    const auto entity = em.create();
    benchmark::DoNotOptimize(entity);
    em.destroy(entity);
  }
}
BENCHMARK_REGISTER_F(EntityManagerFixture, CreateAndDestroyEntities)
    ->RangeMultiplier(10)
    ->Range(10, 1000000);

// Measure the performance of creating and destroying entities in a loop, with
// reusing IDs
BENCHMARK_DEFINE_F(EntityManagerFixture, CreateAndDestroyEntitiesWithReuse)
(benchmark::State& state)
{
  while (state.KeepRunning()) {
    const auto entity = em.create();
    em.destroy(entity);
  }
}
BENCHMARK_REGISTER_F(EntityManagerFixture, CreateAndDestroyEntitiesWithReuse)
    ->RangeMultiplier(10)
    ->Range(10, 1000000);

// Measure the performance of checking entity validity
BENCHMARK_DEFINE_F(EntityManagerFixture, IsValidEntities)
(benchmark::State& state)
{
  for (auto _ : state) {
    benchmark::DoNotOptimize(em.isValid(entities.front()));
  }
}
BENCHMARK_REGISTER_F(EntityManagerFixture, IsValidEntities)
    ->RangeMultiplier(10)
    ->Range(10, 1000000);

// Measure the performance of checking entity equality
BENCHMARK_DEFINE_F(EntityManagerFixture, AreEqualEntities)
(benchmark::State& state)
{
  for (auto _ : state) {
    benchmark::DoNotOptimize(entities.front() == entities.back());
  }
}
BENCHMARK_REGISTER_F(EntityManagerFixture, AreEqualEntities)
    ->RangeMultiplier(10)
    ->Range(10, 1000000);

// Measure the performance of checking entity inequality
BENCHMARK_DEFINE_F(EntityManagerFixture, AreUnequalEntities)
(benchmark::State& state)
{
  for (auto _ : state) {
    benchmark::DoNotOptimize(entities.front() != entities.back());
  }
}
BENCHMARK_REGISTER_F(EntityManagerFixture, AreUnequalEntities)
    ->RangeMultiplier(10)
    ->Range(10, 1000000);
