/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the "BSD-style" License.
 */

#ifndef DART_EXAMPLES_DEMOS_MEMORY_MAP_MODEL_HPP_
#define DART_EXAMPLES_DEMOS_MEMORY_MAP_MODEL_HPP_

#include <string>
#include <vector>

#include <cstddef>

namespace dart::examples::demos {

/// Semantic state represented by one display cell in a logical memory map.
enum class MemoryMapCellKind
{
  Active,
  Hole,
  Reserved,
  Mixed,
};

/// One bounded display cell representing one or more logical units.
struct MemoryMapCell
{
  MemoryMapCellKind kind{MemoryMapCellKind::Reserved};
  std::size_t firstUnit{0};
  std::size_t unitCount{0};
  std::size_t activeUnits{0};
  std::size_t holeUnits{0};
  std::size_t reservedUnits{0};
  std::string tooltip;
};

/// A renderer-neutral row in a two-dimensional logical memory map.
struct MemoryMapRow
{
  std::string key;
  std::string label;
  std::string unit;
  std::size_t activeUnits{0};
  std::size_t holeUnits{0};
  std::size_t reservedUnits{0};
  std::size_t totalUnits{0};
  std::string scope;
  std::string source;
  std::string limitation;
  std::vector<MemoryMapCell> cells;
};

enum class MemoryAddressState
{
  Metadata,
  Allocated,
  Free,
  Reserved,
  Padding,
};

const char* memoryAddressStateLabel(MemoryAddressState state) noexcept;

/// Semantic owner used for hue, independent of allocator/logical state.
enum class MemoryAddressCategory
{
  None,
  AllocatorInfrastructure,
  SimulationModel,
  CollisionGeometry,
  SimulationState,
  SimulationControl,
  ContactSolver,
  SimulationCache,
  EntityIndex,
  SimulationMetadata,
  FrameScratch,
  Unknown,
};

const char* memoryAddressCategoryLabel(MemoryAddressCategory category) noexcept;

enum class MemoryAddressLogicalUse
{
  NotApplicable,
  Live,
  Tombstone,
  Spare,
};

const char* memoryAddressLogicalUseLabel(
    MemoryAddressLogicalUse logicalUse) noexcept;

enum class MemoryAddressEvidence
{
  AllocatorBookkeeping,
  TypedPayloadOverlay,
};

const char* memoryAddressEvidenceLabel(MemoryAddressEvidence evidence) noexcept;

/// Exact region-relative byte span before display binning.
struct MemoryAddressSpan
{
  std::size_t offsetBytes{0};
  std::size_t sizeBytes{0};
  MemoryAddressState state{MemoryAddressState::Reserved};
  MemoryAddressCategory category{MemoryAddressCategory::Unknown};
  MemoryAddressLogicalUse logicalUse{MemoryAddressLogicalUse::NotApplicable};
  MemoryAddressEvidence evidence{MemoryAddressEvidence::AllocatorBookkeeping};
  std::string label;
};

struct MemoryAddressCellSegment
{
  MemoryAddressState state{MemoryAddressState::Reserved};
  MemoryAddressCategory category{MemoryAddressCategory::Unknown};
  MemoryAddressLogicalUse logicalUse{MemoryAddressLogicalUse::NotApplicable};
  std::size_t sizeBytes{0};
};

/// One equal-scale display bin in a real backing region.
struct MemoryAddressCell
{
  std::size_t firstByte{0};
  std::size_t byteCount{0};
  /// Ordered byte-overlap segments; order is increasing relative address.
  std::vector<MemoryAddressCellSegment> segments;
  std::string tooltip;
};

/// Renderer-neutral display model for one actual contiguous backing region.
struct MemoryAddressMapRow
{
  std::string key;
  std::string label;
  std::size_t totalBytes{0};
  std::string scope;
  std::string source;
  std::string limitation;
  /// Exact source partition retained for interactive detail changes.
  std::vector<MemoryAddressSpan> spans;
  std::vector<MemoryAddressCell> cells;
};

/// Rebuilds equal-byte display cells from the retained exact span partition.
void rebinMemoryAddressMapRow(
    MemoryAddressMapRow& row, std::size_t maximumCells);

/// Bins exact, ordered byte spans without changing their address geometry.
MemoryAddressMapRow makeMemoryAddressMapRow(
    std::string key,
    std::string label,
    std::size_t totalBytes,
    std::vector<MemoryAddressSpan> spans,
    std::string scope,
    std::string source,
    std::string limitation,
    std::size_t maximumCells = 256);

/// Builds a bounded state-composition row.
///
/// Display cells are grouped in active, hole, reserved order. They communicate
/// composition, not physical addresses or the original order of logical units.
MemoryMapRow makeMemoryMapRow(
    std::string key,
    std::string label,
    std::string unit,
    std::size_t activeUnits,
    std::size_t holeUnits,
    std::size_t reservedUnits,
    std::string scope,
    std::string source,
    std::string limitation,
    std::size_t maximumCells = 64);

} // namespace dart::examples::demos

#endif // DART_EXAMPLES_DEMOS_MEMORY_MAP_MODEL_HPP_
