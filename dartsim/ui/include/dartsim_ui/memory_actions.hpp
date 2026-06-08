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

#pragma once

#include <dartsim_engine/sim_engine.hpp>

#include <string>
#include <vector>

#include <cstddef>

namespace dartsim::ui {

struct MemoryAllocatorStatus
{
  std::string name;
  std::size_t liveBytes = 0;
  std::size_t peakLiveBytes = 0;
  std::size_t liveAllocationCount = 0;
};

struct MemoryEcsStorageStatus
{
  std::size_t storageId = 0;
  std::size_t size = 0;
  std::size_t capacity = 0;
};

/// Snapshot consumed by the editor Memory panel.
struct MemoryStatus
{
  bool allocatorDebugEnabled = false;
  std::string frameScratchLabel;
  std::string allocatorDebugLabel;
  std::string ecsSummaryLabel;
  std::size_t frameScratchCapacityBytes = 0;
  std::size_t frameScratchUsedBytes = 0;
  std::size_t frameScratchPeakUsedBytes = 0;
  std::size_t frameScratchOverflowCount = 0;
  std::size_t frameScratchOverflowBytes = 0;
  std::size_t frameScratchResetCount = 0;
  std::size_t ecsEntityCount = 0;
  std::size_t ecsEntityCapacity = 0;
  std::size_t ecsStorageCount = 0;
  std::size_t ecsComponentCount = 0;
  std::size_t ecsComponentCapacity = 0;
  std::vector<MemoryAllocatorStatus> allocators;
  std::vector<MemoryEcsStorageStatus> largestStorages;
};

/// Build read-only panel data from the current DART 7 World diagnostics.
[[nodiscard]] MemoryStatus buildMemoryStatus(
    const SimEngine& engine, std::size_t storageLimit = 8);

} // namespace dartsim::ui
