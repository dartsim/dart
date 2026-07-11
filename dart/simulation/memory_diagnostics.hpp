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

#include <dart/common/memory_manager.hpp>

#include <string>
#include <vector>

#include <cstddef>

namespace dart::simulation {

/// Controls the cost and detail of a World memory-diagnostics query.
struct WorldMemoryDiagnosticsOptions
{
  /// Scan every materialized ECS component slot to report holes and live
  /// packed regions.
  ///
  /// Off by default so summary consumers avoid the per-component packed-slot
  /// scan. Summary collection still walks materialized storages and scans the
  /// entity storage to count live entities.
  bool includeStorageLayoutDetails = false;
};

/// Per-component-storage ECS memory diagnostics. Storage IDs are internal
/// diagnostic tokens for grouping one snapshot; callers should not persist them
/// as stable public component identifiers.
struct WorldEcsStorageDiagnostics
{
  /// Internal diagnostic ID for this component storage.
  std::size_t storageId = 0;
  /// Best-effort human-readable role for interactive diagnostics.
  ///
  /// This label does not expose an EnTT type or form a stable component/API
  /// identifier. It may change as internal storage ownership evolves.
  std::string diagnosticLabel;
  /// Component count reported by this storage.
  ///
  /// A detailed query subtracts materialized holes and therefore reports live
  /// components. A summary query uses the type-erased storage size, which can
  /// include tombstones for an internal in-place-deletion policy.
  std::size_t size = 0;
  /// Current storage capacity before another component insertion may grow it.
  std::size_t capacity = 0;
  /// Number of materialized slots in the packed entity array, including holes.
  std::size_t packedSlotCount = 0;
  /// Addressable entity-index extent of the sparse index.
  ///
  /// This is an index-space extent, not a count of allocated sparse pages or
  /// bytes.
  std::size_t sparseExtent = 0;
  /// Reserved packed slots beyond the materialized packed entity array.
  ///
  /// Holes inside the materialized array are reported separately by
  /// `holeCount`.
  std::size_t unusedCapacity = 0;
  /// Materialized packed slots that do not currently contain a live component.
  ///
  /// These can be in-place deletion tombstones or other policy-specific holes.
  std::size_t holeCount = 0;
  /// Number of contiguous runs of live slots in the packed entity array.
  std::size_t livePackedRegionCount = 0;
  /// True when every materialized packed slot contains a live component.
  bool packedContiguous = true;
};

/// Aggregate ECS registry storage diagnostics for profiler/debugger surfaces.
struct WorldEcsDiagnostics
{
  /// Number of live entities in the World registry.
  std::size_t entityCount = 0;
  /// Current registry entity storage capacity.
  std::size_t entityCapacity = 0;
  /// Number of component storages currently materialized by the registry.
  std::size_t storageCount = 0;
  /// Sum of storage component counts.
  ///
  /// This is an exact live count when storage layout details are included; the
  /// summary path can include internal in-place-deletion tombstones.
  std::size_t componentCount = 0;
  /// Sum of component capacities across materialized component storages.
  std::size_t componentCapacity = 0;
  /// True when per-slot packed layout fields were collected by an explicit
  /// detailed query.
  ///
  /// When false, `holeCount`, `livePackedRegionCount`, and `packedContiguous`
  /// on storage rows are not observations and must not be interpreted.
  bool storageLayoutDetailsIncluded = false;
  /// Per-storage component-count/capacity counters for layout debugging and UI
  /// grouping.
  std::vector<WorldEcsStorageDiagnostics> storages;
};

/// Snapshot of the World's CPU memory hierarchy diagnostics.
///
/// This snapshot reports the World-owned frame allocator used for per-step
/// scratch, structured debug counters for direct free/pool allocations, and
/// ECS registry storage layout counters for memory debugger/profiler tools.
struct WorldMemoryDiagnostics
{
  /// Debug counters for the World-owned MemoryManager hierarchy.
  common::MemoryManager::DebugDiagnostics allocatorDebugDiagnostics;

  /// ECS registry/component storage diagnostics.
  WorldEcsDiagnostics ecsDiagnostics;

  /// Current usable frame-scratch arena capacity after alignment padding.
  std::size_t frameScratchCapacityBytes = 0;
  /// Bytes consumed in the current simulation frame, including overflow blocks.
  std::size_t frameScratchUsedBytes = 0;
  /// Maximum frame-scratch bytes observed since construction or clear().
  std::size_t frameScratchPeakUsedBytes = 0;
  /// Number of overflow allocations the frame allocator holds for the current
  /// frame. Nonzero means the current step exceeded the reserved arena.
  std::size_t frameScratchOverflowCount = 0;
  /// Bytes currently held by overflow allocations for the current frame.
  std::size_t frameScratchOverflowBytes = 0;
  /// Number of times the World reset frame scratch at step boundaries since
  /// construction or clear().
  std::size_t frameScratchResetCount = 0;
};

} // namespace dart::simulation
