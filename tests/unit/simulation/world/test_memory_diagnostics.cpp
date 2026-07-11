/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 * https://github.com/dartsim/dart/blob/main/LICENSE
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

#include <dart/simulation/detail/world_registry_access.hpp>
#include <dart/simulation/world.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <array>

#include <cstddef>
#include <cstdint>

namespace {

struct InPlaceDiagnosticComponent
{
  static constexpr bool in_place_delete = true;
  std::uint64_t value{0};
};

} // namespace

TEST(WorldMemoryDiagnostics, ReportsEcsStorageHolesAndRegions)
{
  namespace sx = dart::simulation;

  sx::World world;
  auto& registry = sx::detail::registryOf(world);
  std::array<entt::entity, 5> entities;
  for (std::size_t index = 0; index < entities.size(); ++index) {
    entities[index] = registry.create();
    registry.emplace<InPlaceDiagnosticComponent>(entities[index], index);
  }
  registry.erase<InPlaceDiagnosticComponent>(entities[1]);
  registry.erase<InPlaceDiagnosticComponent>(entities[3]);

  const auto storageId = static_cast<std::size_t>(
      entt::type_hash<InPlaceDiagnosticComponent>::value());
  const auto findStorage = [storageId](const auto& diagnostics) {
    return std::find_if(
        diagnostics.ecsDiagnostics.storages.begin(),
        diagnostics.ecsDiagnostics.storages.end(),
        [storageId](const auto& storage) {
          return storage.storageId == storageId;
        });
  };

  const auto summary = world.getMemoryDiagnostics();
  EXPECT_FALSE(summary.ecsDiagnostics.storageLayoutDetailsIncluded);
  const auto summaryStorage = findStorage(summary);
  ASSERT_NE(summaryStorage, summary.ecsDiagnostics.storages.end());
  EXPECT_EQ(summaryStorage->size, 5u);
  EXPECT_EQ(summaryStorage->holeCount, 0u);
  EXPECT_EQ(summaryStorage->livePackedRegionCount, 0u);
  EXPECT_FALSE(summaryStorage->packedContiguous);

  const auto diagnostics = world.getMemoryDiagnostics(
      sx::WorldMemoryDiagnosticsOptions{.includeStorageLayoutDetails = true});
  EXPECT_TRUE(diagnostics.ecsDiagnostics.storageLayoutDetailsIncluded);
  const auto storageIt = findStorage(diagnostics);

  ASSERT_NE(storageIt, diagnostics.ecsDiagnostics.storages.end());
  EXPECT_EQ(storageIt->diagnosticLabel, "Other/internal storage");
  EXPECT_EQ(storageIt->size, 3u);
  EXPECT_EQ(storageIt->packedSlotCount, 5u);
  EXPECT_EQ(storageIt->holeCount, 2u);
  EXPECT_EQ(storageIt->livePackedRegionCount, 3u);
  EXPECT_FALSE(storageIt->packedContiguous);
  EXPECT_EQ(
      storageIt->capacity,
      storageIt->packedSlotCount + storageIt->unusedCapacity);
  EXPECT_GE(storageIt->sparseExtent, entities.size());

  std::size_t liveComponentCount = 0;
  std::size_t packedComponentSlotCount = 0;
  for (const auto& storage : diagnostics.ecsDiagnostics.storages) {
    liveComponentCount += storage.size;
    packedComponentSlotCount += storage.packedSlotCount;
  }
  EXPECT_EQ(diagnostics.ecsDiagnostics.componentCount, liveComponentCount);
  EXPECT_EQ(packedComponentSlotCount, liveComponentCount + 2u);
}
