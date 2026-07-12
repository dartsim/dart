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
