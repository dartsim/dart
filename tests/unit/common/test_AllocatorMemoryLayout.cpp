/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the "BSD-style" License.
 */

#include <dart/common/FrameAllocator.hpp>
#include <dart/common/FreeListAllocator.hpp>
#include <dart/common/MemoryAllocator.hpp>
#include <dart/common/detail/AllocatorMemoryLayout.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <numeric>

namespace dart::common {
namespace {

using detail::AllocatorMemoryRegion;
using detail::AllocatorMemoryRegionKind;
using detail::AllocatorMemorySpanKind;

std::size_t coveredBytes(const AllocatorMemoryRegion& region)
{
  return std::accumulate(
      region.spans.begin(),
      region.spans.end(),
      std::size_t{0},
      [](std::size_t total, const detail::AllocatorMemorySpan& span) {
        return total + span.sizeBytes;
      });
}

std::size_t bytesWithKind(
    const AllocatorMemoryRegion& region, AllocatorMemorySpanKind kind)
{
  std::size_t total = 0;
  for (const auto& span : region.spans) {
    if (span.kind == kind) {
      total += span.sizeBytes;
    }
  }
  return total;
}

void expectExactPartition(const AllocatorMemoryRegion& region)
{
  std::size_t cursor = 0;
  for (const auto& span : region.spans) {
    EXPECT_EQ(span.offsetBytes, cursor);
    ASSERT_LE(span.sizeBytes, region.sizeBytes - cursor);
    cursor += span.sizeBytes;
  }
  EXPECT_EQ(cursor, region.sizeBytes);
}

TEST(AllocatorMemoryLayout, FreeListPreservesExactChunkAndPayloadOrder)
{
  FreeListAllocator allocator(
      MemoryAllocator::GetDefault(),
      256,
      FreeListAllocator::GrowthPolicy::FixedCapacity);

  auto layout = detail::captureAllocatorMemoryLayout(allocator);
  ASSERT_EQ(layout.regions.size(), 1u);
  const auto& initial = layout.regions.front();
  EXPECT_EQ(initial.kind, AllocatorMemoryRegionKind::FreeListBacking);
  EXPECT_EQ(coveredBytes(initial), initial.sizeBytes);
  expectExactPartition(initial);
  ASSERT_EQ(initial.spans.size(), 2u);
  EXPECT_EQ(initial.spans[0].offsetBytes, 0u);
  EXPECT_EQ(initial.spans[0].kind, AllocatorMemorySpanKind::AllocatorMetadata);
  EXPECT_EQ(initial.spans[1].kind, AllocatorMemorySpanKind::FreePayload);
  EXPECT_EQ(initial.spans[1].sizeBytes, 256u);

  void* first = allocator.allocate(32);
  void* second = allocator.allocate(48);
  ASSERT_NE(first, nullptr);
  ASSERT_NE(second, nullptr);

  layout = detail::captureAllocatorMemoryLayout(allocator);
  ASSERT_EQ(layout.regions.size(), 1u);
  const auto& active = layout.regions.front();
  EXPECT_EQ(coveredBytes(active), active.sizeBytes);
  expectExactPartition(active);
  EXPECT_EQ(
      bytesWithKind(active, AllocatorMemorySpanKind::AllocatedPayload), 80u);

  const auto containsPayload = [&active](const void* pointer) {
    const auto address = reinterpret_cast<std::uintptr_t>(pointer);
    return std::any_of(
        active.spans.begin(),
        active.spans.end(),
        [&active, address](const detail::AllocatorMemorySpan& span) {
          return span.kind == AllocatorMemorySpanKind::AllocatedPayload
                 && active.baseAddress + span.offsetBytes == address;
        });
  };
  EXPECT_TRUE(containsPayload(first));
  EXPECT_TRUE(containsPayload(second));

  allocator.deallocate(first, 32);
  allocator.deallocate(second, 48);
  layout = detail::captureAllocatorMemoryLayout(allocator);
  ASSERT_EQ(layout.regions.size(), 1u);
  EXPECT_EQ(
      bytesWithKind(
          layout.regions.front(), AllocatorMemorySpanKind::AllocatedPayload),
      0u);
  EXPECT_EQ(
      coveredBytes(layout.regions.front()), layout.regions.front().sizeBytes);
  expectExactPartition(layout.regions.front());
}

TEST(AllocatorMemoryLayout, FreeListGrowthCreatesSeparateBackingRegions)
{
  FreeListAllocator allocator(MemoryAllocator::GetDefault(), 64);
  void* first = allocator.allocate(64);
  void* second = allocator.allocate(16);
  ASSERT_NE(first, nullptr);
  ASSERT_NE(second, nullptr);

  const auto layout = detail::captureAllocatorMemoryLayout(allocator);
  ASSERT_EQ(layout.regions.size(), 2u);
  EXPECT_LT(layout.regions[0].baseAddress, layout.regions[1].baseAddress);
  for (const auto& region : layout.regions) {
    EXPECT_EQ(region.kind, AllocatorMemoryRegionKind::FreeListBacking);
    EXPECT_EQ(coveredBytes(region), region.sizeBytes);
    expectExactPartition(region);
  }

  allocator.deallocate(first, 64);
  allocator.deallocate(second, 16);
}

TEST(AllocatorMemoryLayout, FreeListTinyCapacityGrowthResetsRegionValidation)
{
  FreeListAllocator allocator(MemoryAllocator::GetDefault(), 1);
  void* first = allocator.allocate(1);
  void* second = allocator.allocate(2);
  ASSERT_NE(first, nullptr);
  ASSERT_NE(second, nullptr);
  // A tiny growth chunk cannot split, so the next pre-existing chunk begins a
  // new non-contiguous region without a cross-region mPrev link.
  allocator.deallocate(second, 2);

  const auto layout = detail::captureAllocatorMemoryLayout(allocator);
  ASSERT_EQ(layout.regions.size(), 2u);
  for (const auto& region : layout.regions) {
    EXPECT_EQ(region.kind, AllocatorMemoryRegionKind::FreeListBacking);
    EXPECT_EQ(coveredBytes(region), region.sizeBytes);
    expectExactPartition(region);
  }

  allocator.deallocate(first, 1);
}

TEST(AllocatorMemoryLayout, FrameReportsPrimaryAndOverflowBackingRegions)
{
  FrameAllocator allocator(MemoryAllocator::GetDefault(), 256);
  void* primaryPointer = allocator.allocate(96);
  void* overflowPointer = allocator.allocate(512);
  ASSERT_NE(primaryPointer, nullptr);
  ASSERT_NE(overflowPointer, nullptr);

  const auto layout = detail::captureAllocatorMemoryLayout(allocator);
  ASSERT_EQ(layout.regions.size(), 2u);

  const auto primary = std::find_if(
      layout.regions.begin(),
      layout.regions.end(),
      [](const AllocatorMemoryRegion& region) {
        return region.kind == AllocatorMemoryRegionKind::FramePrimary;
      });
  ASSERT_NE(primary, layout.regions.end());
  EXPECT_EQ(primary->sizeBytes, allocator.capacity());
  EXPECT_EQ(coveredBytes(*primary), primary->sizeBytes);
  expectExactPartition(*primary);
  EXPECT_EQ(
      bytesWithKind(*primary, AllocatorMemorySpanKind::FrameCursorConsumed),
      allocator.used());

  const auto overflow = std::find_if(
      layout.regions.begin(),
      layout.regions.end(),
      [](const AllocatorMemoryRegion& region) {
        return region.kind == AllocatorMemoryRegionKind::FrameOverflow;
      });
  ASSERT_NE(overflow, layout.regions.end());
  EXPECT_EQ(coveredBytes(*overflow), overflow->sizeBytes);
  expectExactPartition(*overflow);
  EXPECT_EQ(
      bytesWithKind(*overflow, AllocatorMemorySpanKind::FrameOverflowBacking),
      overflow->sizeBytes);
}

TEST(AllocatorMemoryLayout, FramePartitionsTinyAndNonPowerOfTwoCapacities)
{
  for (const std::size_t capacity :
       {std::size_t{1},
        std::size_t{31},
        std::size_t{63},
        std::size_t{65},
        std::size_t{127}}) {
    FrameAllocator allocator(MemoryAllocator::GetDefault(), capacity);
    const auto layout = detail::captureAllocatorMemoryLayout(allocator);
    ASSERT_EQ(layout.regions.size(), 1u) << capacity;
    const auto& region = layout.regions.front();
    EXPECT_EQ(region.kind, AllocatorMemoryRegionKind::FramePrimary) << capacity;
    EXPECT_EQ(region.sizeBytes, capacity) << capacity;
    expectExactPartition(region);

    if (allocator.usableCapacity() == 0u) {
      EXPECT_EQ(
          bytesWithKind(region, AllocatorMemorySpanKind::AlignmentPadding),
          capacity)
          << capacity;
    }
  }
}

} // namespace
} // namespace dart::common
