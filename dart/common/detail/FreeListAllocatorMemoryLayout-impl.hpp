/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the "BSD-style" License.
 */

#ifndef DART_COMMON_DETAIL_FREELISTALLOCATORMEMORYLAYOUT_IMPL_HPP_
#define DART_COMMON_DETAIL_FREELISTALLOCATORMEMORYLAYOUT_IMPL_HPP_

#include <algorithm>
#include <limits>
#include <mutex>
#include <unordered_set>
#include <utility>

namespace dart::common::detail {

//==============================================================================
inline AllocatorMemoryLayout captureAllocatorMemoryLayout(
    const FreeListAllocator& allocator)
{
  AllocatorMemoryLayout layout;
  std::lock_guard<std::mutex> lock(allocator.mMutex);

  using Header = FreeListAllocator::MemoryBlockHeader;
  constexpr std::size_t kHeaderSize = sizeof(Header);
  std::unordered_set<const Header*> visited;
  std::size_t reservedPayloadBytes = 0;
  std::size_t allocatedSpanBytes = 0;

  auto* block = allocator.mFirstMemoryBlock;
  Header* previousListBlock = nullptr;
  while (block != nullptr) {
    AllocatorMemoryRegion region;
    region.baseAddress = reinterpret_cast<std::uintptr_t>(block);
    region.kind = AllocatorMemoryRegionKind::FreeListBacking;
    std::uintptr_t expectedHeaderAddress = region.baseAddress;
    Header* previousContiguousBlock = nullptr;
    bool firstBlockInRegion = true;

    while (block != nullptr) {
      if (!visited.insert(block).second
          || reinterpret_cast<std::uintptr_t>(block) != expectedHeaderAddress
          || (firstBlockInRegion && previousListBlock == nullptr
              && block->mPrev != nullptr)
          || (firstBlockInRegion && previousListBlock != nullptr
              && block->mPrev != nullptr && block->mPrev != previousListBlock)
          || (!firstBlockInRegion && block->mPrev != previousContiguousBlock)
          || kHeaderSize > std::numeric_limits<std::uintptr_t>::max()
                               - expectedHeaderAddress) {
        return {};
      }

      const std::uintptr_t payloadAddress = expectedHeaderAddress + kHeaderSize;
      if (block->mSize
          > std::numeric_limits<std::uintptr_t>::max() - payloadAddress) {
        return {};
      }
      const std::uintptr_t blockEndAddress = payloadAddress + block->mSize;
      const std::uintptr_t headerOffsetValue
          = expectedHeaderAddress - region.baseAddress;
      const std::uintptr_t regionSizeValue
          = blockEndAddress - region.baseAddress;
      if (headerOffsetValue > std::numeric_limits<std::size_t>::max()
          || regionSizeValue > std::numeric_limits<std::size_t>::max()) {
        return {};
      }
      const auto headerOffset = static_cast<std::size_t>(headerOffsetValue);
      region.spans.push_back(
          {headerOffset,
           kHeaderSize,
           AllocatorMemorySpanKind::AllocatorMetadata});
      if (block->mSize != 0) {
        region.spans.push_back(
            {headerOffset + kHeaderSize,
             block->mSize,
             block->mIsAllocated ? AllocatorMemorySpanKind::AllocatedPayload
                                 : AllocatorMemorySpanKind::FreePayload});
        if (block->mIsAllocated) {
          if (block->mSize
              > std::numeric_limits<std::size_t>::max() - allocatedSpanBytes) {
            return {};
          }
          allocatedSpanBytes += block->mSize;
        }
      }

      const bool nextIsContiguous = block->mIsNextContiguous;
      auto* const next = block->mNext;
      region.sizeBytes = static_cast<std::size_t>(regionSizeValue);
      previousContiguousBlock = block;
      previousListBlock = block;
      firstBlockInRegion = false;
      block = next;
      if (!nextIsContiguous) {
        break;
      }
      if (next == nullptr) {
        return {};
      }
      expectedHeaderAddress = blockEndAddress;
    }

    if (region.sizeBytes < kHeaderSize
        || region.sizeBytes - kHeaderSize
               > std::numeric_limits<std::size_t>::max()
                     - reservedPayloadBytes) {
      return {};
    }
    reservedPayloadBytes += region.sizeBytes - kHeaderSize;
    layout.regions.push_back(std::move(region));
  }

  if (reservedPayloadBytes != allocator.mTotalAllocatedBlockSize
      || allocatedSpanBytes < allocator.mTotalAllocatedSize
      || (allocator.mFreeBlock != nullptr
          && (visited.count(allocator.mFreeBlock) == 0
              || allocator.mFreeBlock->mIsAllocated))) {
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

#endif // DART_COMMON_DETAIL_FREELISTALLOCATORMEMORYLAYOUT_IMPL_HPP_
