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

} // namespace

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

} // namespace dart::examples::demos
