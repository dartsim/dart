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

#pragma once

#include <dart/simulation/detail/world_registry_types.hpp>
#include <dart/simulation/memory_diagnostics.hpp>

#include <cstddef>

namespace dart::simulation::detail {

/// Owns cached World memory counters and the opt-in ECS snapshot scan.
///
/// Keeping this tracker in opaque `WorldStorage` prevents diagnostics state and
/// EnTT-specific collection logic from expanding the public `World` class.
class MemoryDiagnosticsTracker
{
public:
  /// Reset the frame allocator at a step boundary and record the new state.
  ///
  /// Post-reset usage and overflow are not cached: a reset releases the
  /// overflow allocations, and collect() reads both live, so storing zeros
  /// here would be overwritten before any caller could observe it.
  void resetFrameScratch(common::MemoryManager& memoryManager)
  {
    memoryManager.getFrameAllocator().reset();
    ++m_cached.frameScratchResetCount;
  }

  /// Sample the frame-scratch high-water mark after a simulation step.
  ///
  /// This is the only diagnostics work on the step path, because a peak cannot
  /// be reconstructed from a later allocator query the way capacity, usage, and
  /// overflow can.
  void recordFrameScratch(const common::MemoryManager& memoryManager);

  /// Collect a value snapshot, scanning ECS storage only for this explicit
  /// diagnostic query.
  [[nodiscard]] WorldMemoryDiagnostics collect(
      const common::MemoryManager& memoryManager,
      const WorldRegistry& registry,
      const WorldMemoryDiagnosticsOptions& options) const;

private:
  /// Hot-path state only, and only the counters that a later allocator query
  /// cannot recover: a running peak and a reset tally. Capacity, usage, and
  /// overflow are read live by collect(); rich ECS and region vectors are
  /// constructed by an explicit collect() call and never live in every World.
  struct CachedCounters
  {
    std::size_t frameScratchPeakUsedBytes = 0u;
    std::size_t frameScratchResetCount = 0u;
  };

  CachedCounters m_cached;
};

} // namespace dart::simulation::detail
