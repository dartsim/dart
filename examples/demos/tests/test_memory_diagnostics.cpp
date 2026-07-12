/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the "BSD-style" License.
 */

#include "../memory_diagnostics.hpp"

#include <dart/simulation/body/rigid_body.hpp>
#include <dart/simulation/world.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <string>

namespace dart::examples::demos {
namespace {

TEST(MemoryDiagnostics, CollectsWorldAllocatorAndEcsScopes)
{
  simulation::World world;
  auto body = world.addRigidBody("diagnostic_body");
  (void)body;

  const DiagnosticSnapshot snapshot = collectMemoryDiagnostics(world, 42u);
  EXPECT_EQ(snapshot.engine, "DART 7 EnTT World");
  EXPECT_EQ(snapshot.generation, 42u);
  EXPECT_EQ(snapshot.frame, world.getFrame());
  EXPECT_EQ(snapshot.simulationTimeSeconds, world.getTime());

  const DiagnosticMetric* const rigidBodies
      = findMetric(snapshot, "world.rigid_bodies");
  ASSERT_NE(rigidBodies, nullptr);
  ASSERT_TRUE(rigidBodies->value);
  EXPECT_DOUBLE_EQ(*rigidBodies->value, 1.0);
  EXPECT_EQ(rigidBodies->quality, MetricQuality::Measured);

  const DiagnosticMetric* const freeBytes
      = findMetric(snapshot, "allocator.free.live_bytes");
  ASSERT_NE(freeBytes, nullptr);
  ASSERT_TRUE(freeBytes->value);
  EXPECT_EQ(freeBytes->unit, "bytes");
  EXPECT_EQ(freeBytes->quality, MetricQuality::Measured);

  const DiagnosticMetric* const entities
      = findMetric(snapshot, "ecs.aggregate.entities_live");
  const DiagnosticMetric* const components
      = findMetric(snapshot, "ecs.aggregate.components_live");
  const DiagnosticMetric* const componentCapacity
      = findMetric(snapshot, "ecs.aggregate.component_capacity");
  ASSERT_NE(entities, nullptr);
  ASSERT_NE(components, nullptr);
  ASSERT_NE(componentCapacity, nullptr);
  ASSERT_TRUE(entities->value);
  ASSERT_TRUE(components->value);
  ASSERT_TRUE(componentCapacity->value);
  EXPECT_GT(*entities->value, 0.0);
  EXPECT_GT(*components->value, 0.0);
  EXPECT_GE(*componentCapacity->value, *components->value);

  const auto proxy = std::find_if(
      snapshot.metrics.begin(),
      snapshot.metrics.end(),
      [](const DiagnosticMetric& metric) {
        return metric.key.starts_with("ecs.storage.")
               && metric.key.ends_with(".packed_contiguous");
      });
  ASSERT_NE(proxy, snapshot.metrics.end());
  EXPECT_EQ(proxy->quality, MetricQuality::Proxy);
  EXPECT_NE(proxy->limitation.find("cache"), std::string::npos);

  const auto scratchMap = std::find_if(
      snapshot.memoryMaps.begin(),
      snapshot.memoryMaps.end(),
      [](const MemoryMapRow& row) { return row.key == "scratch.arena"; });
  ASSERT_NE(scratchMap, snapshot.memoryMaps.end());
  EXPECT_EQ(
      scratchMap->activeUnits + scratchMap->holeUnits
          + scratchMap->reservedUnits,
      scratchMap->totalUnits);
  EXPECT_EQ(scratchMap->unit, "bytes");

  const auto storageMap = std::find_if(
      snapshot.memoryMaps.begin(),
      snapshot.memoryMaps.end(),
      [](const MemoryMapRow& row) {
        return row.key.starts_with("ecs.storage.")
               && row.key.ends_with(".capacity_map");
      });
  ASSERT_NE(storageMap, snapshot.memoryMaps.end());
  EXPECT_EQ(
      storageMap->activeUnits + storageMap->holeUnits
          + storageMap->reservedUnits,
      storageMap->totalUnits);
  EXPECT_EQ(storageMap->unit, "slots");
  EXPECT_NE(
      storageMap->limitation.find("do not reproduce packed-slot order"),
      std::string::npos);
}

TEST(MemoryDiagnostics, KeepsUnavailableInstrumentationDistinctFromZero)
{
  simulation::World world;
  const DiagnosticSnapshot snapshot = collectMemoryDiagnostics(world, 1u);

  const DiagnosticMetric* const resident
      = findMetric(snapshot, kProcessResidentBytesKey);
  const DiagnosticMetric* const poolBytes
      = findMetric(snapshot, "allocator.pool.live_bytes");
  ASSERT_NE(resident, nullptr);
  ASSERT_NE(poolBytes, nullptr);

#if defined(__linux__) || defined(_WIN32) || defined(__APPLE__)
  EXPECT_TRUE(resident->value);
#endif

  const auto& pool = world.getMemoryManager().getPoolAllocator();
  EXPECT_EQ(poolBytes->value.has_value(), pool.isDiagnosticsEnabled());
  if (!poolBytes->value) {
    EXPECT_NE(poolBytes->limitation.find("disabled"), std::string::npos);
  }

  const bool hasNoCacheClaim = std::any_of(
      snapshot.guidance.begin(),
      snapshot.guidance.end(),
      [](const std::string& guidance) {
        return guidance.find("do not measure cache misses")
               != std::string::npos;
      });
  EXPECT_TRUE(hasNoCacheClaim);
}

} // namespace
} // namespace dart::examples::demos
