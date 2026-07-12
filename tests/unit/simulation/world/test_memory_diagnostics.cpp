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

#include <dart/simulation/body/rigid_body.hpp>
#include <dart/simulation/comps/all.hpp>
#include <dart/simulation/detail/world_registry_access.hpp>
#include <dart/simulation/world.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <string>
#include <string_view>

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
      sx::WorldMemoryDiagnosticsOptions{
          .includeStorageLayoutDetails = true,
          .includeMemoryLayoutDetails = true});
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

  bool foundExactTombstoneIndexRange = false;
  for (const auto& region : diagnostics.memoryRegions) {
    for (const auto& span : region.spans) {
      foundExactTombstoneIndexRange
          |= span.category == sx::WorldMemoryDataCategory::EntityIndex
             && span.logicalUse == sx::WorldMemoryLogicalUse::Tombstone
             && span.diagnosticLabel.find("Other/internal storage packed")
                    != std::string::npos;
    }
  }
  EXPECT_TRUE(foundExactTombstoneIndexRange);
}

TEST(WorldMemoryDiagnostics, PreservesKnownComponentPageSlotOrderAndSpareBytes)
{
  namespace sx = dart::simulation;

  constexpr std::size_t pageSlots
      = entt::component_traits<sx::comps::Transform>::page_size;
  static_assert(pageSlots > 0u);
  const std::size_t entityCount = pageSlots + 3u;

  sx::World world;
  auto& registry = sx::detail::registryOf(world);
  for (std::size_t index = 0; index < entityCount; ++index) {
    const auto entity = registry.create();
    registry.emplace<sx::comps::Transform>(entity);
  }
  const auto& constRegistry = registry;
  const auto* storage = constRegistry.storage<sx::comps::Transform>();
  ASSERT_NE(storage, nullptr);
  ASSERT_EQ(storage->capacity(), 2u * pageSlots);

  const auto diagnostics = world.getMemoryDiagnostics(
      sx::WorldMemoryDiagnosticsOptions{.includeMemoryLayoutDetails = true});
  std::size_t liveBytes = 0u;
  std::size_t spareBytes = 0u;
  bool foundSecondPageLiveRun = false;
  bool foundSecondPageSpareRun = false;
  for (const auto& region : diagnostics.memoryRegions) {
    for (const auto& span : region.spans) {
      if (span.category != sx::WorldMemoryDataCategory::SimulationState
          || span.diagnosticLabel.find("Transforms payload page")
                 == std::string::npos) {
        continue;
      }
      if (span.logicalUse == sx::WorldMemoryLogicalUse::Live) {
        liveBytes += span.sizeBytes;
        foundSecondPageLiveRun |= span.diagnosticLabel.find("page 1 live slots")
                                  != std::string::npos;
      } else if (span.logicalUse == sx::WorldMemoryLogicalUse::Spare) {
        spareBytes += span.sizeBytes;
        foundSecondPageSpareRun
            |= span.diagnosticLabel.find("page 1 spare slots")
               != std::string::npos;
      }
    }
  }

  EXPECT_EQ(liveBytes, entityCount * sizeof(sx::comps::Transform));
  EXPECT_EQ(
      spareBytes,
      (storage->capacity() - entityCount) * sizeof(sx::comps::Transform));
  EXPECT_TRUE(foundSecondPageLiveRun);
  EXPECT_TRUE(foundSecondPageSpareRun);
}

TEST(WorldMemoryDiagnostics, ReportsExactRelativeAllocatorRegionsAndTypedPages)
{
  namespace sx = dart::simulation;

  sx::World world;
  auto rigidBody = world.addRigidBody("layout_body");
  (void)rigidBody;
  auto& registry = sx::detail::registryOf(world);
  const auto geometryEntity = registry.create();
  registry.emplace<sx::comps::CollisionGeometry>(geometryEntity);

  const auto summary = world.getMemoryDiagnostics();
  EXPECT_FALSE(summary.memoryLayoutDetailsIncluded);
  EXPECT_TRUE(summary.memoryRegions.empty());

  const auto diagnostics = world.getMemoryDiagnostics(
      sx::WorldMemoryDiagnosticsOptions{
          .includeStorageLayoutDetails = true,
          .includeMemoryLayoutDetails = true});
  ASSERT_TRUE(diagnostics.memoryLayoutDetailsIncluded);
  ASSERT_FALSE(diagnostics.memoryRegions.empty());

  bool foundFreeList = false;
  bool foundFrameArena = false;
  bool foundTypedModel = false;
  bool foundTypedGeometry = false;
  bool foundTypedState = false;
  bool foundAllocatorBookkeeping = false;
  bool foundAllocatorInfrastructure = false;
  for (std::size_t regionIndex = 0;
       regionIndex < diagnostics.memoryRegions.size();
       ++regionIndex) {
    const auto& region = diagnostics.memoryRegions[regionIndex];
    EXPECT_EQ(region.addressOrder, regionIndex);
    EXPECT_EQ(
        region.evidence, sx::WorldMemoryEvidenceKind::ActualBackingRegion);
    EXPECT_GT(region.sizeBytes, 0u);
    foundFreeList |= region.kind == sx::WorldMemoryRegionKind::FreeListBacking;
    foundFrameArena |= region.kind == sx::WorldMemoryRegionKind::FrameArena;

    std::size_t expectedOffset = 0u;
    for (const auto& span : region.spans) {
      EXPECT_EQ(span.offsetBytes, expectedOffset);
      EXPECT_GT(span.sizeBytes, 0u);
      expectedOffset += span.sizeBytes;
      foundTypedModel
          |= span.category == sx::WorldMemoryDataCategory::SimulationModel
             && span.evidence
                    == sx::WorldMemoryEvidenceKind::TypedPayloadOverlay;
      foundTypedGeometry
          |= span.category == sx::WorldMemoryDataCategory::CollisionGeometry
             && span.evidence
                    == sx::WorldMemoryEvidenceKind::TypedPayloadOverlay;
      foundTypedState
          |= span.category == sx::WorldMemoryDataCategory::SimulationState
             && span.evidence
                    == sx::WorldMemoryEvidenceKind::TypedPayloadOverlay;
      foundAllocatorBookkeeping
          |= span.evidence == sx::WorldMemoryEvidenceKind::AllocatorBookkeeping;
      foundAllocatorInfrastructure
          |= span.state == sx::WorldMemorySpanState::Metadata
             && span.category
                    == sx::WorldMemoryDataCategory::AllocatorInfrastructure;
      if (span.state == sx::WorldMemorySpanState::Metadata) {
        EXPECT_EQ(
            span.category,
            sx::WorldMemoryDataCategory::AllocatorInfrastructure);
      } else if (span.state != sx::WorldMemorySpanState::Allocated) {
        EXPECT_EQ(span.category, sx::WorldMemoryDataCategory::None);
        EXPECT_EQ(span.logicalUse, sx::WorldMemoryLogicalUse::NotApplicable);
      }
    }
    EXPECT_EQ(expectedOffset, region.sizeBytes);
  }

  EXPECT_TRUE(foundFreeList);
  EXPECT_TRUE(foundFrameArena);
  EXPECT_TRUE(foundTypedModel);
  EXPECT_TRUE(foundTypedGeometry);
  EXPECT_TRUE(foundTypedState);
  EXPECT_TRUE(foundAllocatorBookkeeping);
  EXPECT_TRUE(foundAllocatorInfrastructure);
}

TEST(WorldMemoryDiagnostics, ClassifiesStableConfigStateAndScratchComponents)
{
  namespace sx = dart::simulation;

  sx::World world;
  auto& registry = sx::detail::registryOf(world);
  const auto entity = registry.create();
  registry.emplace<sx::comps::KinematicBodyTag>(entity);
  registry.emplace<sx::comps::KinematicBodyStepTrace>(entity);
  registry.emplace<sx::comps::RigidAvbdContactConfig>(entity);
  registry.emplace<sx::comps::DeformableVbdConfig>(entity);
  registry.emplace<sx::comps::DeformableSolverScratch>(
      entity, world.getMemoryManager().getFreeAllocator());

  const auto diagnostics = world.getMemoryDiagnostics(
      sx::WorldMemoryDiagnosticsOptions{.includeMemoryLayoutDetails = true});
  const auto hasTypedOverlay = [&diagnostics](
                                   sx::WorldMemoryDataCategory category,
                                   std::string_view label) {
    for (const auto& region : diagnostics.memoryRegions) {
      for (const auto& span : region.spans) {
        if (span.category == category
            && span.evidence == sx::WorldMemoryEvidenceKind::TypedPayloadOverlay
            && span.diagnosticLabel.find(label) != std::string::npos) {
          return true;
        }
      }
    }
    return false;
  };

  EXPECT_TRUE(hasTypedOverlay(
      sx::WorldMemoryDataCategory::SimulationControl,
      "Kinematic-body configuration"));
  EXPECT_TRUE(hasTypedOverlay(
      sx::WorldMemoryDataCategory::SimulationCache,
      "Kinematic-body step trace"));
  EXPECT_TRUE(hasTypedOverlay(
      sx::WorldMemoryDataCategory::ContactSolver,
      "Rigid AVBD contact configuration"));
  EXPECT_TRUE(hasTypedOverlay(
      sx::WorldMemoryDataCategory::ContactSolver,
      "Deformable VBD configuration"));
  EXPECT_TRUE(hasTypedOverlay(
      sx::WorldMemoryDataCategory::SimulationCache,
      "Deformable solver scratch"));
}
