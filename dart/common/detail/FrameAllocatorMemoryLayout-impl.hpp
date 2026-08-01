/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the "BSD-style" License.
 */

#ifndef DART_COMMON_DETAIL_FRAMEALLOCATORMEMORYLAYOUT_IMPL_HPP_
#define DART_COMMON_DETAIL_FRAMEALLOCATORMEMORYLAYOUT_IMPL_HPP_

#include <algorithm>
#include <limits>
#include <utility>

namespace dart::common::detail {

namespace frame_allocator_memory_layout_detail {

inline void appendSpan(
    AllocatorMemoryRegion& region,
    std::size_t offsetBytes,
    std::size_t sizeBytes,
    AllocatorMemorySpanKind kind)
{
  if (sizeBytes != 0) {
    region.spans.push_back({offsetBytes, sizeBytes, kind});
  }
}

inline bool hasExactPartition(const AllocatorMemoryRegion& region)
{
  std::size_t cursor = 0;
  for (const auto& span : region.spans) {
    if (span.offsetBytes != cursor
        || span.sizeBytes > region.sizeBytes - cursor) {
      return false;
    }
    cursor += span.sizeBytes;
  }
  return cursor == region.sizeBytes;
}

} // namespace frame_allocator_memory_layout_detail

//==============================================================================
inline AllocatorMemoryLayout captureAllocatorMemoryLayout(
    const FrameAllocator& allocator)
{
  AllocatorMemoryLayout layout;

  if ((allocator.mBuffer == nullptr) != (allocator.mCapacity == 0)) {
    return {};
  }

  if (allocator.mBuffer != nullptr && allocator.mCapacity != 0) {
    AllocatorMemoryRegion region;
    region.baseAddress = reinterpret_cast<std::uintptr_t>(allocator.mBuffer);
    region.sizeBytes = allocator.mCapacity;
    region.kind = AllocatorMemoryRegionKind::FramePrimary;

    if (region.sizeBytes
        > std::numeric_limits<std::uintptr_t>::max() - region.baseAddress) {
      return {};
    }
    const std::uintptr_t rawEndAddress = region.baseAddress + region.sizeBytes;

    if (allocator.mBegin == nullptr || allocator.mEnd == nullptr
        || allocator.mCur == nullptr) {
      return {};
    }

    const auto beginAddress
        = reinterpret_cast<std::uintptr_t>(allocator.mBegin);
    const auto endAddress = reinterpret_cast<std::uintptr_t>(allocator.mEnd);
    const auto cursorAddress = reinterpret_cast<std::uintptr_t>(allocator.mCur);
    if (beginAddress < region.baseAddress || beginAddress > rawEndAddress
        || endAddress < beginAddress || endAddress > rawEndAddress
        || cursorAddress < beginAddress || cursorAddress > endAddress) {
      return {};
    }

    if (endAddress == beginAddress) {
      if (beginAddress != region.baseAddress
          || cursorAddress != region.baseAddress) {
        return {};
      }
      frame_allocator_memory_layout_detail::appendSpan(
          region,
          0,
          region.sizeBytes,
          AllocatorMemorySpanKind::AlignmentPadding);
    } else {
      const auto prefixBytes
          = static_cast<std::size_t>(beginAddress - region.baseAddress);
      const auto usableBytes
          = static_cast<std::size_t>(endAddress - beginAddress) & ~size_t{31};
      const auto usedBytes
          = static_cast<std::size_t>(cursorAddress - beginAddress);
      if (usedBytes > usableBytes || prefixBytes > region.sizeBytes
          || usableBytes > region.sizeBytes - prefixBytes) {
        return {};
      }

      frame_allocator_memory_layout_detail::appendSpan(
          region, 0, prefixBytes, AllocatorMemorySpanKind::AlignmentPadding);
      frame_allocator_memory_layout_detail::appendSpan(
          region,
          prefixBytes,
          usedBytes,
          AllocatorMemorySpanKind::FrameCursorConsumed);
      frame_allocator_memory_layout_detail::appendSpan(
          region,
          prefixBytes + usedBytes,
          usableBytes - usedBytes,
          AllocatorMemorySpanKind::FrameReservedAvailable);

      const auto classifiedBytes = prefixBytes + usableBytes;
      if (classifiedBytes < region.sizeBytes) {
        frame_allocator_memory_layout_detail::appendSpan(
            region,
            classifiedBytes,
            region.sizeBytes - classifiedBytes,
            AllocatorMemorySpanKind::AlignmentPadding);
      }
    }
    if (!frame_allocator_memory_layout_detail::hasExactPartition(region)) {
      return {};
    }
    layout.regions.push_back(std::move(region));
  }

  std::size_t overflowBytes = 0;
  for (const auto& overflow : allocator.mOverflowAllocations) {
    if (overflow.ptr == nullptr || overflow.allocatedSize == 0
        || overflow.allocatedSize
               > std::numeric_limits<std::uintptr_t>::max()
                     - reinterpret_cast<std::uintptr_t>(overflow.ptr)
        || overflow.allocatedSize
               > std::numeric_limits<std::size_t>::max() - overflowBytes) {
      return {};
    }
    AllocatorMemoryRegion region;
    region.baseAddress = reinterpret_cast<std::uintptr_t>(overflow.ptr);
    region.sizeBytes = overflow.allocatedSize;
    region.kind = AllocatorMemoryRegionKind::FrameOverflow;
    frame_allocator_memory_layout_detail::appendSpan(
        region,
        0,
        region.sizeBytes,
        AllocatorMemorySpanKind::FrameOverflowBacking);
    if (!frame_allocator_memory_layout_detail::hasExactPartition(region)) {
      return {};
    }
    overflowBytes += overflow.allocatedSize;
    layout.regions.push_back(std::move(region));
  }
  if (overflowBytes != allocator.mOverflowBytes) {
    return {};
  }

  std::sort(
      layout.regions.begin(),
      layout.regions.end(),
      [](const AllocatorMemoryRegion& lhs, const AllocatorMemoryRegion& rhs) {
        return lhs.baseAddress < rhs.baseAddress;
      });
  std::uintptr_t previousEnd = 0;
  for (const auto& region : layout.regions) {
    if (region.sizeBytes
            > std::numeric_limits<std::uintptr_t>::max() - region.baseAddress
        || (previousEnd != 0 && region.baseAddress < previousEnd)) {
      return {};
    }
    previousEnd = region.baseAddress + region.sizeBytes;
  }
  return layout;
}

} // namespace dart::common::detail

#endif // DART_COMMON_DETAIL_FRAMEALLOCATORMEMORYLAYOUT_IMPL_HPP_
