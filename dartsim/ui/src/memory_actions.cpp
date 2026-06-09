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
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include <dart/simulation/world.hpp>

#include <dartsim_engine/object_manager.hpp>
#include <dartsim_ui/memory_actions.hpp>

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

namespace dartsim::ui {

namespace {

MemoryAllocatorStatus makeAllocatorStatus(
    std::string name,
    const dart::common::MemoryManager::AllocatorDebugDiagnostics& diagnostics)
{
  MemoryAllocatorStatus status;
  status.name = std::move(name);
  status.liveBytes = diagnostics.liveBytes;
  status.peakLiveBytes = diagnostics.peakLiveBytes;
  status.liveAllocationCount = diagnostics.liveAllocationCount;
  return status;
}

std::string byteCountLabel(std::size_t bytes)
{
  return std::to_string(bytes) + " bytes";
}

std::string makeFrameScratchLabel(
    const dart::simulation::WorldMemoryDiagnostics& diagnostics)
{
  return "Frame scratch: " + byteCountLabel(diagnostics.frameScratchUsedBytes)
         + " used / " + byteCountLabel(diagnostics.frameScratchCapacityBytes)
         + " capacity, " + byteCountLabel(diagnostics.frameScratchPeakUsedBytes)
         + " peak";
}

std::string makeAllocatorDebugLabel(
    const dart::common::MemoryManager::DebugDiagnostics& diagnostics)
{
  return diagnostics.enabled ? "Allocator debug counters enabled"
                             : "Allocator debug counters disabled";
}

std::string makeEcsSummaryLabel(
    const dart::simulation::WorldEcsDiagnostics& diagnostics)
{
  return "ECS: " + std::to_string(diagnostics.entityCount) + " entities / "
         + std::to_string(diagnostics.entityCapacity) + " entity capacity, "
         + std::to_string(diagnostics.componentCount) + " components / "
         + std::to_string(diagnostics.componentCapacity)
         + " component capacity";
}

} // namespace

MemoryStatus buildMemoryStatus(
    const SimEngine& engine, std::size_t storageLimit)
{
  const dart::simulation::WorldMemoryDiagnostics diagnostics
      = engine.objects().world().getMemoryDiagnostics();
  const dart::simulation::WorldEcsDiagnostics& ecs = diagnostics.ecsDiagnostics;
  const dart::common::MemoryManager::DebugDiagnostics& allocatorDiagnostics
      = diagnostics.allocatorDebugDiagnostics;

  MemoryStatus status;
  status.allocatorDebugEnabled = allocatorDiagnostics.enabled;
  status.frameScratchLabel = makeFrameScratchLabel(diagnostics);
  status.allocatorDebugLabel = makeAllocatorDebugLabel(allocatorDiagnostics);
  status.ecsSummaryLabel = makeEcsSummaryLabel(ecs);
  status.frameScratchCapacityBytes = diagnostics.frameScratchCapacityBytes;
  status.frameScratchUsedBytes = diagnostics.frameScratchUsedBytes;
  status.frameScratchPeakUsedBytes = diagnostics.frameScratchPeakUsedBytes;
  status.frameScratchOverflowCount = diagnostics.frameScratchOverflowCount;
  status.frameScratchOverflowBytes = diagnostics.frameScratchOverflowBytes;
  status.frameScratchResetCount = diagnostics.frameScratchResetCount;
  status.ecsEntityCount = ecs.entityCount;
  status.ecsEntityCapacity = ecs.entityCapacity;
  status.ecsStorageCount = ecs.storageCount;
  status.ecsComponentCount = ecs.componentCount;
  status.ecsComponentCapacity = ecs.componentCapacity;
  status.allocators.push_back(
      makeAllocatorStatus("free list", allocatorDiagnostics.freeAllocator));
  status.allocators.push_back(
      makeAllocatorStatus("pool", allocatorDiagnostics.poolAllocator));

  status.largestStorages.reserve(
      std::min(storageLimit, diagnostics.ecsDiagnostics.storages.size()));
  std::vector<dart::simulation::WorldEcsStorageDiagnostics> storages
      = diagnostics.ecsDiagnostics.storages;
  std::sort(
      storages.begin(), storages.end(), [](const auto& lhs, const auto& rhs) {
        if (lhs.capacity != rhs.capacity) {
          return lhs.capacity > rhs.capacity;
        }
        if (lhs.size != rhs.size) {
          return lhs.size > rhs.size;
        }
        return lhs.storageId < rhs.storageId;
      });
  const std::size_t count = std::min(storageLimit, storages.size());
  for (std::size_t i = 0; i < count; ++i) {
    status.largestStorages.push_back(
        MemoryEcsStorageStatus{
            storages[i].storageId,
            storages[i].size,
            storages[i].capacity,
        });
  }

  return status;
}

} // namespace dartsim::ui
