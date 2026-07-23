/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the "BSD-style" License.
 */

#ifndef DART_COMMON_DETAIL_ALLOCATORMEMORYLAYOUT_HPP_
#define DART_COMMON_DETAIL_ALLOCATORMEMORYLAYOUT_HPP_

#include <vector>

#include <cstddef>
#include <cstdint>

namespace dart::common {

class FrameAllocator;
class FreeListAllocator;

namespace detail {

/// The allocator-owned meaning of one exact byte range in a backing region.
enum class AllocatorMemorySpanKind
{
  AllocatorMetadata,
  AllocatedPayload,
  FreePayload,
  AlignmentPadding,
  FrameCursorConsumed,
  FrameReservedAvailable,
  FrameOverflowBacking,
};

enum class AllocatorMemoryRegionKind
{
  FreeListBacking,
  FramePrimary,
  FrameOverflow,
};

/// One address-ordered span, expressed relative to its containing region.
struct AllocatorMemorySpan
{
  std::size_t offsetBytes{0};
  std::size_t sizeBytes{0};
  AllocatorMemorySpanKind kind{AllocatorMemorySpanKind::FreePayload};
};

/// One actually contiguous allocation obtained from an allocator's base.
struct AllocatorMemoryRegion
{
  std::uintptr_t baseAddress{0};
  std::size_t sizeBytes{0};
  AllocatorMemoryRegionKind kind{AllocatorMemoryRegionKind::FreeListBacking};
  std::vector<AllocatorMemorySpan> spans;
};

/// Renderer-neutral snapshot of the backing allocations owned by one allocator.
struct AllocatorMemoryLayout
{
  std::vector<AllocatorMemoryRegion> regions;
};

/// Captures exact free-list backing chunks and their metadata/payload ranges.
///
/// The allocator mutex is held while the linked block structure is copied.
[[nodiscard]] inline AllocatorMemoryLayout captureAllocatorMemoryLayout(
    const FreeListAllocator& allocator);

/// Captures the primary frame arena and each live overflow backing allocation.
///
/// FrameAllocator is intentionally single-threaded. The caller must capture it
/// from the same externally synchronized context that performs allocations.
[[nodiscard]] inline AllocatorMemoryLayout captureAllocatorMemoryLayout(
    const FrameAllocator& allocator);

} // namespace detail
} // namespace dart::common

// Definitions stay header-only so demo/tool callers do not cross a shared-
// library export boundary on Windows. The allocator headers only carry friend
// declarations, so ordinary allocator users do not parse these inspectors.
#include <dart/common/FrameAllocator.hpp>
#include <dart/common/FreeListAllocator.hpp>
#include <dart/common/detail/FrameAllocatorMemoryLayout-impl.hpp>
#include <dart/common/detail/FreeListAllocatorMemoryLayout-impl.hpp>

#endif // DART_COMMON_DETAIL_ALLOCATORMEMORYLAYOUT_HPP_
