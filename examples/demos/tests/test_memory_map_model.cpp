/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the "BSD-style" License.
 */

#include "../memory_map_model.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <string>

namespace dart::examples::demos {
namespace {

MemoryMapRow makeTestRow(
    std::size_t active,
    std::size_t holes,
    std::size_t reserved,
    std::size_t maximumCells)
{
  return makeMemoryMapRow(
      "test.map",
      "Test map",
      "slots",
      active,
      holes,
      reserved,
      "test scope",
      "test source",
      "grouped composition, not address order",
      maximumCells);
}

TEST(MemoryMapModel, PreservesExactSmallComposition)
{
  const MemoryMapRow row = makeTestRow(3u, 2u, 3u, 8u);

  EXPECT_EQ(row.totalUnits, 8u);
  ASSERT_EQ(row.cells.size(), 8u);
  for (std::size_t index = 0; index < row.cells.size(); ++index) {
    const MemoryMapCell& cell = row.cells[index];
    EXPECT_EQ(cell.firstUnit, index);
    EXPECT_EQ(cell.unitCount, 1u);
  }
  EXPECT_EQ(row.cells[0].kind, MemoryMapCellKind::Active);
  EXPECT_EQ(row.cells[2].kind, MemoryMapCellKind::Active);
  EXPECT_EQ(row.cells[3].kind, MemoryMapCellKind::Hole);
  EXPECT_EQ(row.cells[4].kind, MemoryMapCellKind::Hole);
  EXPECT_EQ(row.cells[5].kind, MemoryMapCellKind::Reserved);
  EXPECT_EQ(row.cells[7].kind, MemoryMapCellKind::Reserved);
  EXPECT_NE(
      row.cells[0].tooltip.find("grouped display slots [0, 1)"),
      std::string::npos);
  EXPECT_EQ(row.cells[0].tooltip.find("logical slots"), std::string::npos);
  EXPECT_NE(row.cells[0].tooltip.find("not address order"), std::string::npos);
}

TEST(MemoryMapModel, BoundsLargeMapsWithoutLosingComposition)
{
  const MemoryMapRow row = makeTestRow(1000u, 111u, 889u, 64u);

  ASSERT_EQ(row.totalUnits, 2000u);
  ASSERT_EQ(row.cells.size(), 64u);
  const std::size_t active = std::accumulate(
      row.cells.begin(),
      row.cells.end(),
      0u,
      [](std::size_t sum, const MemoryMapCell& cell) {
        return sum + cell.activeUnits;
      });
  const std::size_t holes = std::accumulate(
      row.cells.begin(),
      row.cells.end(),
      0u,
      [](std::size_t sum, const MemoryMapCell& cell) {
        return sum + cell.holeUnits;
      });
  const std::size_t reserved = std::accumulate(
      row.cells.begin(),
      row.cells.end(),
      0u,
      [](std::size_t sum, const MemoryMapCell& cell) {
        return sum + cell.reservedUnits;
      });
  EXPECT_EQ(active, row.activeUnits);
  EXPECT_EQ(holes, row.holeUnits);
  EXPECT_EQ(reserved, row.reservedUnits);
  EXPECT_TRUE(
      std::any_of(
          row.cells.begin(), row.cells.end(), [](const MemoryMapCell& cell) {
            return cell.kind == MemoryMapCellKind::Mixed;
          }));
}

TEST(MemoryMapModel, HandlesEmptyAndDisabledMaps)
{
  const MemoryMapRow empty = makeTestRow(0u, 0u, 0u, 128u);
  EXPECT_EQ(empty.totalUnits, 0u);
  EXPECT_TRUE(empty.cells.empty());

  const MemoryMapRow disabled = makeTestRow(3u, 2u, 1u, 0u);
  EXPECT_EQ(disabled.totalUnits, 6u);
  EXPECT_TRUE(disabled.cells.empty());
}

TEST(MemoryMapModel, RejectsOverflowingComposition)
{
  EXPECT_THROW(
      makeTestRow(std::numeric_limits<std::size_t>::max(), 1u, 0u, 128u),
      std::overflow_error);
}

} // namespace
} // namespace dart::examples::demos
