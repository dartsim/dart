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
#include <utility>

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

TEST(MemoryAddressMapModel, PreservesRelativeByteOrderAndComposition)
{
  const MemoryAddressMapRow row = makeMemoryAddressMapRow(
      "region.0",
      "Backing region",
      256u,
      {
          MemoryAddressSpan{
              .offsetBytes = 0u,
              .sizeBytes = 32u,
              .state = MemoryAddressState::Metadata,
              .category = MemoryAddressCategory::None,
              .label = "header"},
          MemoryAddressSpan{
              .offsetBytes = 32u,
              .sizeBytes = 96u,
              .state = MemoryAddressState::Allocated,
              .category = MemoryAddressCategory::SimulationState,
              .logicalUse = MemoryAddressLogicalUse::Live,
              .evidence = MemoryAddressEvidence::TypedPayloadOverlay,
              .label = "state page"},
          MemoryAddressSpan{
              .offsetBytes = 128u,
              .sizeBytes = 128u,
              .state = MemoryAddressState::Free,
              .category = MemoryAddressCategory::None,
              .label = "free payload"},
      },
      "one region",
      "test allocator",
      "relative offsets",
      8u);

  ASSERT_EQ(row.cells.size(), 8u);
  for (std::size_t index = 0; index < row.cells.size(); ++index) {
    const auto& cell = row.cells[index];
    EXPECT_EQ(cell.firstByte, index * 32u);
    EXPECT_EQ(cell.byteCount, 32u);
    const std::size_t contributed = std::accumulate(
        cell.segments.begin(),
        cell.segments.end(),
        std::size_t{0},
        [](std::size_t sum, const auto& segment) {
          return sum + segment.sizeBytes;
        });
    EXPECT_EQ(contributed, cell.byteCount);
  }
  ASSERT_EQ(row.cells[0].segments.size(), 1u);
  EXPECT_EQ(row.cells[0].segments[0].state, MemoryAddressState::Metadata);
  ASSERT_EQ(row.cells[1].segments.size(), 1u);
  EXPECT_EQ(
      row.cells[1].segments[0].category,
      MemoryAddressCategory::SimulationState);
  EXPECT_EQ(row.cells[1].segments[0].logicalUse, MemoryAddressLogicalUse::Live);
  ASSERT_EQ(row.cells[4].segments.size(), 1u);
  EXPECT_EQ(row.cells[4].segments[0].state, MemoryAddressState::Free);
  EXPECT_NE(
      row.cells[1].tooltip.find("actual region-relative bytes [32, 64)"),
      std::string::npos);
  EXPECT_NE(
      row.cells[1].tooltip.find("typed payload overlay"), std::string::npos);
}

TEST(MemoryAddressMapModel, MarksBinsThatCrossExactSpanBoundaries)
{
  const MemoryAddressMapRow row = makeMemoryAddressMapRow(
      "region.mixed",
      "Mixed region",
      128u,
      {
          MemoryAddressSpan{
              .offsetBytes = 0u,
              .sizeBytes = 16u,
              .state = MemoryAddressState::Metadata,
              .category = MemoryAddressCategory::None,
              .label = "header"},
          MemoryAddressSpan{
              .offsetBytes = 16u,
              .sizeBytes = 112u,
              .state = MemoryAddressState::Allocated,
              .category = MemoryAddressCategory::Unknown,
              .label = "payload"},
      },
      "one region",
      "test allocator",
      "relative offsets",
      2u);

  ASSERT_EQ(row.cells.size(), 2u);
  ASSERT_EQ(row.cells[0].segments.size(), 2u);
  EXPECT_EQ(row.cells[0].segments[0].state, MemoryAddressState::Metadata);
  EXPECT_EQ(row.cells[0].segments[1].category, MemoryAddressCategory::Unknown);
  ASSERT_EQ(row.cells[1].segments.size(), 1u);
  EXPECT_EQ(row.cells[1].segments[0].category, MemoryAddressCategory::Unknown);
}

TEST(MemoryAddressMapModel, RejectsGapsAndOverlaps)
{
  const auto makeInvalid = [](std::vector<MemoryAddressSpan> spans) {
    return makeMemoryAddressMapRow(
        "invalid",
        "Invalid",
        64u,
        std::move(spans),
        "scope",
        "source",
        "limitation",
        8u);
  };

  EXPECT_THROW(
      makeInvalid({MemoryAddressSpan{
          .offsetBytes = 1u,
          .sizeBytes = 63u,
          .state = MemoryAddressState::Allocated,
          .category = MemoryAddressCategory::Unknown,
          .label = "gap"}}),
      std::invalid_argument);
  EXPECT_THROW(
      makeInvalid({
          MemoryAddressSpan{
              .offsetBytes = 0u,
              .sizeBytes = 40u,
              .state = MemoryAddressState::Allocated,
              .category = MemoryAddressCategory::Unknown,
              .label = "overlap a"},
          MemoryAddressSpan{
              .offsetBytes = 32u,
              .sizeBytes = 32u,
              .state = MemoryAddressState::Free,
              .category = MemoryAddressCategory::None,
              .label = "overlap b"},
      }),
      std::invalid_argument);
}

TEST(MemoryAddressMapModel, KeepsLogicalUseOrderedWithinOneColorBin)
{
  const MemoryAddressMapRow row = makeMemoryAddressMapRow(
      "logical.use",
      "Logical use",
      96u,
      {
          MemoryAddressSpan{
              .offsetBytes = 0u,
              .sizeBytes = 32u,
              .state = MemoryAddressState::Allocated,
              .category = MemoryAddressCategory::SimulationState,
              .logicalUse = MemoryAddressLogicalUse::Live,
              .evidence = MemoryAddressEvidence::TypedPayloadOverlay,
              .label = "live"},
          MemoryAddressSpan{
              .offsetBytes = 32u,
              .sizeBytes = 32u,
              .state = MemoryAddressState::Allocated,
              .category = MemoryAddressCategory::SimulationState,
              .logicalUse = MemoryAddressLogicalUse::Tombstone,
              .evidence = MemoryAddressEvidence::TypedPayloadOverlay,
              .label = "tombstone"},
          MemoryAddressSpan{
              .offsetBytes = 64u,
              .sizeBytes = 32u,
              .state = MemoryAddressState::Allocated,
              .category = MemoryAddressCategory::SimulationState,
              .logicalUse = MemoryAddressLogicalUse::Spare,
              .evidence = MemoryAddressEvidence::TypedPayloadOverlay,
              .label = "spare"},
      },
      "scope",
      "source",
      "limitation",
      1u);

  ASSERT_EQ(row.cells.size(), 1u);
  ASSERT_EQ(row.cells[0].segments.size(), 3u);
  EXPECT_EQ(row.cells[0].segments[0].logicalUse, MemoryAddressLogicalUse::Live);
  EXPECT_EQ(
      row.cells[0].segments[1].logicalUse, MemoryAddressLogicalUse::Tombstone);
  EXPECT_EQ(
      row.cells[0].segments[2].logicalUse, MemoryAddressLogicalUse::Spare);
  EXPECT_TRUE(
      std::all_of(
          row.cells[0].segments.begin(),
          row.cells[0].segments.end(),
          [](const auto& segment) {
            return segment.category == MemoryAddressCategory::SimulationState;
          }));
}

TEST(MemoryAddressMapModel, RebinnedFragmentedRegionKeepsEveryContributor)
{
  constexpr std::size_t kSpanCount = 4096u;
  std::vector<MemoryAddressSpan> spans;
  spans.reserve(kSpanCount);
  for (std::size_t index = 0; index < kSpanCount; ++index) {
    spans.push_back(
        MemoryAddressSpan{
            .offsetBytes = index,
            .sizeBytes = 1u,
            .state = index % 2u == 0u ? MemoryAddressState::Allocated
                                      : MemoryAddressState::Free,
            .category = index % 2u == 0u
                            ? MemoryAddressCategory::SimulationState
                            : MemoryAddressCategory::None,
            .label = "span " + std::to_string(index)});
  }

  const MemoryAddressMapRow row = makeMemoryAddressMapRow(
      "fragmented",
      "Fragmented",
      kSpanCount,
      std::move(spans),
      "scope",
      "source",
      "limitation",
      kSpanCount);

  ASSERT_EQ(row.cells.size(), kSpanCount);
  for (std::size_t index = 0; index < row.cells.size(); ++index) {
    const auto& cell = row.cells[index];
    ASSERT_EQ(cell.segments.size(), 1u) << index;
    EXPECT_EQ(cell.firstByte, index);
    EXPECT_EQ(cell.byteCount, 1u);
    EXPECT_NE(
        cell.tooltip.find("span " + std::to_string(index)), std::string::npos)
        << index;
  }
}

TEST(MemoryAddressMapModel, HandlesMaximumRegionSizeWithoutOverflow)
{
  const std::size_t maximum = std::numeric_limits<std::size_t>::max();
  const MemoryAddressMapRow row = makeMemoryAddressMapRow(
      "maximum",
      "Maximum",
      maximum,
      {MemoryAddressSpan{
          .offsetBytes = 0u,
          .sizeBytes = maximum,
          .state = MemoryAddressState::Allocated,
          .category = MemoryAddressCategory::Unknown,
          .label = "maximum"}},
      "scope",
      "source",
      "limitation",
      256u);
  ASSERT_EQ(row.cells.size(), 256u);
  EXPECT_EQ(row.cells.front().firstByte, 0u);
  EXPECT_EQ(row.cells.back().firstByte + row.cells.back().byteCount, maximum);
}

} // namespace
} // namespace dart::examples::demos
