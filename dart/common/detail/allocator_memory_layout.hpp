/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the "BSD-style" License.
 */

#pragma once

#include <dart/common/frame_allocator.hpp>
#include <dart/common/free_list_allocator.hpp>

#include <algorithm>
#include <limits>
#include <vector>

#include <cstddef>
#include <cstdint>

namespace dart::common::detail {

/// Internal state of an exact byte range in an allocator backing allocation.
///
/// Addresses deliberately stay in this implementation-only snapshot. Public
/// diagnostics translate them to offsets relative to each backing region.
enum class AllocatorMemorySpanState
{
  Metadata,
  Allocated,
  Free,
  Reserved,
  Padding,
};

enum class AllocatorMemoryRegionKind
{
  FreeListBacking,
  FrameArena,
  FrameOverflow,
};

struct AllocatorMemorySpan
{
  std::uintptr_t address = 0;
  std::size_t sizeBytes = 0;
  AllocatorMemorySpanState state{AllocatorMemorySpanState::Reserved};
};

struct AllocatorMemoryRegion
{
  std::uintptr_t address = 0;
  std::size_t sizeBytes = 0;
  AllocatorMemoryRegionKind kind{AllocatorMemoryRegionKind::FreeListBacking};
  std::vector<AllocatorMemorySpan> spans;
};

/// Quiescent-point inspector for allocator-owned backing allocations.
///
/// The allocators are single-threaded. Calling this while another thread
/// mutates either allocator is unsupported for the same reason that concurrent
/// allocation is unsupported.
class AllocatorMemoryLayoutInspector
{
public:
  [[nodiscard]] static std::vector<AllocatorMemoryRegion> inspect(
      const FreeListAllocator& allocator)
  {
    std::vector<AllocatorMemoryRegion> regions;
    regions.reserve(allocator.mAllocatedBlocks.size());
    for (const auto& backing : allocator.mAllocatedBlocks) {
      AllocatorMemoryRegion region;
      region.address = reinterpret_cast<std::uintptr_t>(backing.pointer);
      region.sizeBytes = backing.size;
      region.kind = AllocatorMemoryRegionKind::FreeListBacking;

      if (region.sizeBytes
          > std::numeric_limits<std::uintptr_t>::max() - region.address) {
        continue;
      }

      const std::uintptr_t regionEnd = region.address + region.sizeBytes;
      auto* block = static_cast<const FreeListAllocator::MemoryBlockHeader*>(
          backing.pointer);
      std::uintptr_t expectedHeaderAddress = region.address;
      bool valid = true;
      constexpr std::size_t headerSize
          = sizeof(FreeListAllocator::MemoryBlockHeader);
      const std::size_t maximumHeaderCount = region.sizeBytes / headerSize + 1u;
      std::size_t headerCount = 0u;
      while (block != nullptr && valid) {
        if (++headerCount > maximumHeaderCount) {
          valid = false;
          break;
        }
        const std::uintptr_t headerAddress
            = reinterpret_cast<std::uintptr_t>(block);
        if (headerAddress != expectedHeaderAddress
            || headerAddress >= regionEnd) {
          valid = false;
          break;
        }

        if (headerSize > regionEnd - headerAddress) {
          valid = false;
          break;
        }
        const std::uintptr_t payloadAddress = headerAddress + headerSize;
        if (block->mSize > regionEnd - payloadAddress) {
          valid = false;
          break;
        }
        const std::uintptr_t payloadEnd = payloadAddress + block->mSize;

        region.spans.push_back(
            AllocatorMemorySpan{
                .address = headerAddress,
                .sizeBytes = headerSize,
                .state = AllocatorMemorySpanState::Metadata});
        appendIfNonempty(
            region,
            payloadAddress,
            block->mSize,
            block->mIsAllocated ? AllocatorMemorySpanState::Allocated
                                : AllocatorMemorySpanState::Free);

        if (!block->mIsNextContiguous) {
          valid = payloadEnd == regionEnd;
          break;
        }
        if (block->mNext == nullptr
            || reinterpret_cast<std::uintptr_t>(block->mNext) != payloadEnd) {
          valid = false;
          break;
        }
        expectedHeaderAddress = payloadEnd;
        block = block->mNext;
      }

      if (valid && isExactPartition(region)) {
        regions.push_back(std::move(region));
      }
    }

    sortByAddress(regions);
    return regions;
  }

  [[nodiscard]] static std::vector<AllocatorMemoryRegion> inspect(
      const FrameAllocator& allocator)
  {
    std::vector<AllocatorMemoryRegion> regions;
    regions.reserve(
        static_cast<std::size_t>(allocator.mBuffer != nullptr)
        + allocator.mOverflowAllocations.size());

    if (allocator.mBuffer != nullptr && allocator.mCapacity != 0u) {
      AllocatorMemoryRegion arena;
      arena.address = reinterpret_cast<std::uintptr_t>(allocator.mBuffer);
      arena.sizeBytes = allocator.mCapacity;
      arena.kind = AllocatorMemoryRegionKind::FrameArena;

      if (arena.sizeBytes
          > std::numeric_limits<std::uintptr_t>::max() - arena.address) {
        return regions;
      }

      const std::uintptr_t arenaEnd = arena.address + arena.sizeBytes;
      const bool hasUsableWindow = allocator.mBegin != nullptr
                                   && allocator.mEnd != nullptr
                                   && allocator.mBegin < allocator.mEnd;
      if (!hasUsableWindow) {
        arena.spans.push_back(
            AllocatorMemorySpan{
                .address = arena.address,
                .sizeBytes = arena.sizeBytes,
                .state = AllocatorMemorySpanState::Padding});
      } else {
        const std::uintptr_t begin
            = reinterpret_cast<std::uintptr_t>(allocator.mBegin);
        const std::uintptr_t current
            = reinterpret_cast<std::uintptr_t>(allocator.mCur);
        const std::uintptr_t rawEnd
            = reinterpret_cast<std::uintptr_t>(allocator.mEnd);
        const std::size_t usableCapacity = allocator.usableCapacity();
        if (begin < arena.address || current < begin || rawEnd < current
            || rawEnd > arenaEnd || usableCapacity > rawEnd - begin) {
          return regions;
        }
        const std::uintptr_t usableEnd = begin + usableCapacity;
        appendIfNonempty(
            arena,
            arena.address,
            begin - arena.address,
            AllocatorMemorySpanState::Padding);
        appendIfNonempty(
            arena, begin, current - begin, AllocatorMemorySpanState::Allocated);
        appendIfNonempty(
            arena,
            current,
            usableEnd - current,
            AllocatorMemorySpanState::Reserved);
        appendIfNonempty(
            arena,
            usableEnd,
            rawEnd - usableEnd,
            AllocatorMemorySpanState::Padding);
        appendIfNonempty(
            arena,
            rawEnd,
            arenaEnd - rawEnd,
            AllocatorMemorySpanState::Padding);
      }
      if (isExactPartition(arena)) {
        regions.push_back(std::move(arena));
      }
    }

    for (const auto& overflow : allocator.mOverflowAllocations) {
      AllocatorMemoryRegion region;
      region.address = reinterpret_cast<std::uintptr_t>(overflow.ptr);
      region.sizeBytes = overflow.allocatedSize;
      region.kind = AllocatorMemoryRegionKind::FrameOverflow;
      if (region.sizeBytes
          > std::numeric_limits<std::uintptr_t>::max() - region.address) {
        continue;
      }
      region.spans.push_back(
          AllocatorMemorySpan{
              .address = region.address,
              .sizeBytes = region.sizeBytes,
              .state = AllocatorMemorySpanState::Allocated});
      if (isExactPartition(region)) {
        regions.push_back(std::move(region));
      }
    }

    sortByAddress(regions);
    return regions;
  }

private:
  static void appendIfNonempty(
      AllocatorMemoryRegion& region,
      std::uintptr_t address,
      std::size_t sizeBytes,
      AllocatorMemorySpanState state)
  {
    if (sizeBytes == 0u) {
      return;
    }
    region.spans.push_back(
        AllocatorMemorySpan{
            .address = address, .sizeBytes = sizeBytes, .state = state});
  }

  static void sortByAddress(std::vector<AllocatorMemoryRegion>& regions)
  {
    std::sort(
        regions.begin(), regions.end(), [](const auto& lhs, const auto& rhs) {
          return lhs.address < rhs.address;
        });
  }

  static bool isExactPartition(const AllocatorMemoryRegion& region)
  {
    if (region.sizeBytes
        > std::numeric_limits<std::uintptr_t>::max() - region.address) {
      return false;
    }
    const std::uintptr_t regionEnd = region.address + region.sizeBytes;
    std::uintptr_t expectedAddress = region.address;
    for (const AllocatorMemorySpan& span : region.spans) {
      if (span.sizeBytes == 0u || span.address != expectedAddress
          || span.sizeBytes > regionEnd - expectedAddress) {
        return false;
      }
      expectedAddress += span.sizeBytes;
    }
    return expectedAddress == regionEnd;
  }
};

} // namespace dart::common::detail
