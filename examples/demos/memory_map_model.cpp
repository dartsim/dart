/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the "BSD-style" License.
 */

#include "memory_map_model.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

namespace dart::examples::demos {
namespace {

constexpr std::size_t kHardMaximumDisplayCells = 4096;

std::size_t checkedAdd(std::size_t lhs, std::size_t rhs)
{
  if (rhs > std::numeric_limits<std::size_t>::max() - lhs) {
    throw std::overflow_error("memory-map unit count overflow");
  }
  return lhs + rhs;
}

std::size_t cellBoundary(
    std::size_t cellIndex, std::size_t totalUnits, std::size_t cellCount)
{
  const std::size_t quotient = totalUnits / cellCount;
  const std::size_t remainder = totalUnits % cellCount;
  return quotient * cellIndex + (remainder * cellIndex) / cellCount;
}

std::size_t intervalOverlap(
    std::size_t lhsBegin,
    std::size_t lhsEnd,
    std::size_t rhsBegin,
    std::size_t rhsEnd)
{
  const std::size_t begin = std::max(lhsBegin, rhsBegin);
  const std::size_t end = std::min(lhsEnd, rhsEnd);
  return end > begin ? end - begin : 0u;
}

MemoryMapCellKind cellKind(
    std::size_t activeUnits, std::size_t holeUnits, std::size_t reservedUnits)
{
  const int nonemptyKinds = static_cast<int>(activeUnits > 0u)
                            + static_cast<int>(holeUnits > 0u)
                            + static_cast<int>(reservedUnits > 0u);
  if (nonemptyKinds > 1) {
    return MemoryMapCellKind::Mixed;
  }
  if (activeUnits > 0u) {
    return MemoryMapCellKind::Active;
  }
  if (holeUnits > 0u) {
    return MemoryMapCellKind::Hole;
  }
  return MemoryMapCellKind::Reserved;
}

std::string cellTooltip(const MemoryMapRow& row, const MemoryMapCell& cell)
{
  const std::size_t end = cell.firstUnit + cell.unitCount;
  return row.label + "\ngrouped display " + row.unit + " ["
         + std::to_string(cell.firstUnit) + ", " + std::to_string(end)
         + ")\nactive/used: " + std::to_string(cell.activeUnits) + " "
         + row.unit + "\nholes: " + std::to_string(cell.holeUnits) + " "
         + row.unit + "\nreserved/unused: " + std::to_string(cell.reservedUnits)
         + " " + row.unit + "\n\n" + "scope: " + row.scope
         + "\nsource: " + row.source + "\n" + row.limitation;
}

std::string addressCellTooltip(
    const MemoryAddressMapRow& row,
    const MemoryAddressCell& cell,
    const std::vector<MemoryAddressSpan>& spans,
    std::size_t firstSpan,
    std::size_t onePastLastSpan)
{
  const std::size_t cellEnd = cell.firstByte + cell.byteCount;
  std::string tooltip = row.label + "\nactual region-relative bytes ["
                        + std::to_string(cell.firstByte) + ", "
                        + std::to_string(cellEnd) + ")\ncomposition:";
  for (const MemoryAddressCellSegment& segment : cell.segments) {
    tooltip += "\n- ";
    tooltip += memoryAddressCategoryLabel(segment.category);
    tooltip += " / ";
    tooltip += memoryAddressStateLabel(segment.state);
    tooltip += " / ";
    tooltip += memoryAddressLogicalUseLabel(segment.logicalUse);
    tooltip += ": " + std::to_string(segment.sizeBytes) + " bytes";
  }
  tooltip += "\nexact contributing spans:";
  for (std::size_t spanIndex = firstSpan; spanIndex < onePastLastSpan;
       ++spanIndex) {
    const MemoryAddressSpan& span = spans[spanIndex];
    const std::size_t spanEnd = span.offsetBytes + span.sizeBytes;
    const std::size_t overlap
        = intervalOverlap(cell.firstByte, cellEnd, span.offsetBytes, spanEnd);
    if (overlap == 0u) {
      continue;
    }
    tooltip += "\n- " + span.label + ": " + std::to_string(overlap) + " bytes ("
               + memoryAddressEvidenceLabel(span.evidence) + ")";
  }
  return tooltip + "\n\nscope: " + row.scope + "\nsource: " + row.source + "\n"
         + row.limitation;
}

} // namespace

//==============================================================================
const char* memoryAddressStateLabel(MemoryAddressState state) noexcept
{
  switch (state) {
    case MemoryAddressState::Metadata:
      return "metadata";
    case MemoryAddressState::Allocated:
      return "allocated";
    case MemoryAddressState::Free:
      return "free";
    case MemoryAddressState::Reserved:
      return "reserved";
    case MemoryAddressState::Padding:
      return "padding";
  }
  return "unknown state";
}

//==============================================================================
const char* memoryAddressCategoryLabel(MemoryAddressCategory category) noexcept
{
  switch (category) {
    case MemoryAddressCategory::None:
      return "not applicable";
    case MemoryAddressCategory::AllocatorInfrastructure:
      return "allocator infrastructure";
    case MemoryAddressCategory::SimulationModel:
      return "simulation model";
    case MemoryAddressCategory::CollisionGeometry:
      return "collision / geometry";
    case MemoryAddressCategory::SimulationState:
      return "simulation state";
    case MemoryAddressCategory::SimulationControl:
      return "simulation control";
    case MemoryAddressCategory::ContactSolver:
      return "contact / solver";
    case MemoryAddressCategory::SimulationCache:
      return "cached derived data";
    case MemoryAddressCategory::EntityIndex:
      return "entity index";
    case MemoryAddressCategory::SimulationMetadata:
      return "simulation metadata";
    case MemoryAddressCategory::FrameScratch:
      return "frame scratch";
    case MemoryAddressCategory::Unknown:
      return "unknown live allocation";
  }
  return "unknown";
}

//==============================================================================
const char* memoryAddressLogicalUseLabel(
    MemoryAddressLogicalUse logicalUse) noexcept
{
  switch (logicalUse) {
    case MemoryAddressLogicalUse::NotApplicable:
      return "not applicable";
    case MemoryAddressLogicalUse::Live:
      return "live slot";
    case MemoryAddressLogicalUse::Tombstone:
      return "tombstone slot";
    case MemoryAddressLogicalUse::Spare:
      return "spare slot";
  }
  return "unknown logical use";
}

//==============================================================================
const char* memoryAddressEvidenceLabel(MemoryAddressEvidence evidence) noexcept
{
  switch (evidence) {
    case MemoryAddressEvidence::AllocatorBookkeeping:
      return "allocator bookkeeping";
    case MemoryAddressEvidence::TypedPayloadOverlay:
      return "typed payload overlay";
  }
  return "unknown evidence";
}

//==============================================================================
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
    std::size_t maximumCells)
{
  MemoryMapRow row;
  row.key = std::move(key);
  row.label = std::move(label);
  row.unit = std::move(unit);
  row.activeUnits = activeUnits;
  row.holeUnits = holeUnits;
  row.reservedUnits = reservedUnits;
  row.totalUnits
      = checkedAdd(checkedAdd(activeUnits, holeUnits), reservedUnits);
  row.scope = std::move(scope);
  row.source = std::move(source);
  row.limitation = std::move(limitation);

  if (row.totalUnits == 0u || maximumCells == 0u) {
    return row;
  }

  const std::size_t cellCount = std::min(
      row.totalUnits, std::min(maximumCells, kHardMaximumDisplayCells));
  row.cells.reserve(cellCount);
  const std::size_t activeEnd = activeUnits;
  const std::size_t holeEnd = checkedAdd(activeEnd, holeUnits);
  for (std::size_t index = 0; index < cellCount; ++index) {
    MemoryMapCell cell;
    cell.firstUnit = cellBoundary(index, row.totalUnits, cellCount);
    const std::size_t end = cellBoundary(index + 1u, row.totalUnits, cellCount);
    cell.unitCount = end - cell.firstUnit;
    cell.activeUnits = intervalOverlap(cell.firstUnit, end, 0u, activeEnd);
    cell.holeUnits = intervalOverlap(cell.firstUnit, end, activeEnd, holeEnd);
    cell.reservedUnits
        = intervalOverlap(cell.firstUnit, end, holeEnd, row.totalUnits);
    cell.kind = cellKind(cell.activeUnits, cell.holeUnits, cell.reservedUnits);
    cell.tooltip = cellTooltip(row, cell);
    row.cells.push_back(std::move(cell));
  }

  return row;
}

//==============================================================================
MemoryAddressMapRow makeMemoryAddressMapRow(
    std::string key,
    std::string label,
    std::size_t totalBytes,
    std::vector<MemoryAddressSpan> spans,
    std::string scope,
    std::string source,
    std::string limitation,
    std::size_t maximumCells)
{
  MemoryAddressMapRow row;
  row.key = std::move(key);
  row.label = std::move(label);
  row.totalBytes = totalBytes;
  row.scope = std::move(scope);
  row.source = std::move(source);
  row.limitation = std::move(limitation);

  std::size_t expectedOffset = 0u;
  for (const MemoryAddressSpan& span : spans) {
    if (span.sizeBytes == 0u || span.offsetBytes != expectedOffset
        || span.sizeBytes > totalBytes - expectedOffset) {
      throw std::invalid_argument(
          "address-map spans must exactly cover one ordered region");
    }
    expectedOffset += span.sizeBytes;
  }
  if (expectedOffset != totalBytes) {
    throw std::invalid_argument(
        "address-map spans must exactly cover one ordered region");
  }

  row.spans = std::move(spans);
  rebinMemoryAddressMapRow(row, maximumCells);
  return row;
}

//==============================================================================
void rebinMemoryAddressMapRow(
    MemoryAddressMapRow& row, std::size_t maximumCells)
{
  row.cells.clear();
  if (row.totalBytes == 0u || maximumCells == 0u) {
    return;
  }
  const std::size_t cellCount = std::min(
      row.totalBytes, std::min(maximumCells, kHardMaximumDisplayCells));
  row.cells.reserve(cellCount);
  std::size_t firstCandidateSpan = 0u;
  for (std::size_t index = 0; index < cellCount; ++index) {
    MemoryAddressCell cell;
    cell.firstByte = cellBoundary(index, row.totalBytes, cellCount);
    const std::size_t end = cellBoundary(index + 1u, row.totalBytes, cellCount);
    cell.byteCount = end - cell.firstByte;
    while (firstCandidateSpan < row.spans.size()) {
      const MemoryAddressSpan& span = row.spans[firstCandidateSpan];
      if (span.offsetBytes + span.sizeBytes > cell.firstByte) {
        break;
      }
      ++firstCandidateSpan;
    }
    std::size_t onePastLastSpan = firstCandidateSpan;
    while (onePastLastSpan < row.spans.size()) {
      const MemoryAddressSpan& span = row.spans[onePastLastSpan];
      if (span.offsetBytes >= end) {
        break;
      }
      ++onePastLastSpan;
      const std::size_t spanEnd = span.offsetBytes + span.sizeBytes;
      const std::size_t overlap
          = intervalOverlap(cell.firstByte, end, span.offsetBytes, spanEnd);
      if (overlap == 0u) {
        continue;
      }
      cell.segments.push_back(
          MemoryAddressCellSegment{
              .state = span.state,
              .category = span.category,
              .logicalUse = span.logicalUse,
              .sizeBytes = overlap});
    }
    cell.tooltip = addressCellTooltip(
        row, cell, row.spans, firstCandidateSpan, onePastLastSpan);
    row.cells.push_back(std::move(cell));
  }
}

} // namespace dart::examples::demos
