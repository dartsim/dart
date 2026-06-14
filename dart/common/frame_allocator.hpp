/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef DART_COMMON_FRAMEALLOCATOR_HPP_
#define DART_COMMON_FRAMEALLOCATOR_HPP_

#include <dart/common/memory_allocator.hpp>
#include <dart/common/memory_allocator_debugger.hpp>

#include <dart/export.hpp>

#include <algorithm>
#include <limits>
#include <memory>
#include <new>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>

#include <cstddef>
#include <cstdint>

namespace dart::common {

template <typename T>
class FrameStlAllocator;

class DART_API FrameAllocator : public MemoryAllocator
{
public:
  using Debug = MemoryAllocatorDebugger<FrameAllocator>;

  /// Constructor
  ///
  /// @param[in] baseAllocator: (optional) Base memory allocator.
  /// @param[in] initialCapacity: (optional) Bytes to initially allocate.
  explicit FrameAllocator(
      MemoryAllocator& baseAllocator = MemoryAllocator::GetDefault(),
      size_t initialCapacity = 65536);

  ~FrameAllocator() override;

  FrameAllocator(const FrameAllocator&) = delete;
  FrameAllocator& operator=(const FrameAllocator&) = delete;
  FrameAllocator(FrameAllocator&&) = delete;
  FrameAllocator& operator=(FrameAllocator&&) = delete;

  DART_STRING_TYPE(FrameAllocator);

  [[nodiscard]] const MemoryAllocator& getBaseAllocator() const;
  [[nodiscard]] MemoryAllocator& getBaseAllocator();

  // PERF: These fast paths MUST stay inline. Moving them out-of-line causes
  // 2-4x regression from function-call overhead (benchmarked). Do NOT move
  // to .cpp — see tests/benchmark/common/bm_allocators.cpp.

  [[nodiscard]] inline void* allocate(size_t bytes) noexcept override
  {
    return allocateDefaultAligned(bytes);
  }

  [[nodiscard]] inline void* allocate(
      size_t bytes, size_t alignment) noexcept override
  {
    return allocateAligned(bytes, alignment);
  }

  inline void deallocate(void* /*pointer*/, size_t /*bytes*/) override
  {
    // Arena semantics: memory is freed in bulk on reset()
  }

  inline void deallocate(
      void* /*pointer*/, size_t /*bytes*/, size_t /*alignment*/) override
  {
    // Arena semantics: memory is freed in bulk on reset()
  }

  void print(std::ostream& os = std::cout, int indent = 0) const override;

  [[nodiscard]] inline void* allocateAligned(
      size_t bytes, size_t alignment = 32) noexcept
  {
    if (bytes == 0 || alignment == 0 || (alignment & (alignment - 1)) != 0)
        [[unlikely]] {
      return nullptr;
    }

    if (!mCur) [[unlikely]] {
      return allocateAlignedSlow(bytes, alignment);
    }

    if (alignment == 64) {
      return allocateCacheAlignedFast(bytes, bytes > 2048);
    }
    if (alignment <= 32) {
      return allocateDefaultAligned(bytes);
    }

    const auto cur = reinterpret_cast<uintptr_t>(mCur);
    if (cur > std::numeric_limits<uintptr_t>::max() - (alignment - 1)) {
      return nullptr;
    }
    const auto aligned = (cur + alignment - 1) & ~(alignment - 1);
    if (aligned > std::numeric_limits<uintptr_t>::max() - bytes) {
      return nullptr;
    }
    const auto unpaddedNext = aligned + bytes;
    if (unpaddedNext > std::numeric_limits<uintptr_t>::max() - 31) {
      return nullptr;
    }
    // Round next to 32-byte boundary so that allocate()'s fast path
    // invariant (mCur is 32-aligned) is preserved after mixed calls.
    auto* next = reinterpret_cast<char*>((unpaddedNext + 31) & ~uintptr_t{31});

    if (next <= mEnd) [[likely]] {
      mCur = next;
      return reinterpret_cast<void*>(aligned);
    }

    return allocateAlignedSlow(bytes, alignment);
  }

  [[nodiscard]] inline void* allocateCacheAligned(size_t bytes) noexcept
  {
    return allocateCacheAlignedFast(bytes, false);
  }

  inline void reset() noexcept
  {
    if (mOverflowBytes == 0) [[likely]] {
      mCur = mBegin;
      mCacheLineColor = 2;
      return;
    }
    resetSlow();
  }

  template <typename T, typename... Args>
  [[nodiscard]] T* construct(Args&&... args)
  {
    const size_t alignment = std::max<size_t>(alignof(T), 32);
    void* memory = allocateAligned(sizeof(T), alignment);
    if (memory == nullptr) {
      return nullptr;
    }
    return std::construct_at(
        static_cast<T*>(memory), std::forward<Args>(args)...);
  }

  [[nodiscard]] size_t capacity() const noexcept;

  /// Returns the usable arena bytes after alignment padding.
  [[nodiscard]] size_t usableCapacity() const noexcept;

  [[nodiscard]] size_t used() const noexcept;

  [[nodiscard]] size_t overflowCount() const noexcept;

  [[nodiscard]] size_t overflowBytes() const noexcept;

private:
  template <typename T>
  friend class FrameStlAllocator;

  struct OverflowEntry
  {
    void* ptr;
    size_t allocatedSize;
  };

  [[nodiscard]] inline void* allocateDefaultAligned(size_t bytes) noexcept
  {
    if (bytes == 0 || !mCur) [[unlikely]] {
      return bytes == 0 ? nullptr : allocateAlignedSlow(bytes, 32);
    }

    const auto remaining = static_cast<size_t>(mEnd - mCur);
    if (bytes <= remaining) [[likely]] {
      const auto padded = (bytes + 31) & ~size_t{31};
      if (padded > remaining) [[unlikely]] {
        return allocateAlignedSlow(bytes, 32);
      }

      auto* result = mCur;
      mCur += padded;
      return result;
    }

    return allocateAlignedSlow(bytes, 32);
  }

  [[nodiscard]] inline void* allocateStlStorageCacheAligned(
      size_t bytes, bool colorStorage) noexcept
  {
    return allocateCacheAlignedFast(bytes, colorStorage);
  }

  [[nodiscard]] inline void* allocateCacheAlignedFast(
      size_t bytes, bool colorStorage) noexcept
  {
    if (bytes == 0 || !mCur) [[unlikely]] {
      return bytes == 0 ? nullptr : allocateAlignedSlow(bytes, 64);
    }
    if (bytes > std::numeric_limits<size_t>::max() - 31) [[unlikely]] {
      return allocateAlignedSlow(bytes, 64);
    }

    const auto remaining = static_cast<size_t>(mEnd - mCur);
    const auto addr = reinterpret_cast<uintptr_t>(mCur);
    const auto alignPadding = static_cast<size_t>((uintptr_t{0} - addr) & 63u);
    // EnTT component pages are commonly multiples of the 4 KiB L1 index
    // period. Four 256-byte colors spread starts across sets while bounding
    // arena padding to less than 1 KiB per large storage allocation.
    const auto colorPadding
        = colorStorage ? static_cast<size_t>(mCacheLineColor) * 256u : 0u;
    const auto padding = alignPadding + colorPadding;
    if (padding <= remaining) [[likely]] {
      const auto available = remaining - padding;
      const auto padded = (bytes + 31) & ~size_t{31};
      if (padded <= available) [[likely]] {
        auto* result = mCur + padding;
        mCur = result + padded;
        if (colorStorage) {
          mCacheLineColor
              = static_cast<std::uint8_t>((mCacheLineColor + 1u) & 3u);
        }
        return result;
      }
    }

    return allocateAlignedSlow(bytes, 64);
  }

  [[nodiscard]] void* allocateAlignedSlow(
      size_t bytes, size_t alignment) noexcept;
  void resetSlow() noexcept;

  MemoryAllocator& mBaseAllocator;

  char* mCur;
  char* mEnd;

  char* mBuffer;
  char* mBegin;
  size_t mCapacity;
  size_t mOverflowBytes;
  std::uint8_t mCacheLineColor;
  std::vector<OverflowEntry> mOverflowAllocations;
};

/// STL-compatible allocator adapter for FrameAllocator.
/// Allocated memory is freed only on FrameAllocator::reset(), not on
/// individual deallocate() calls.
template <typename T>
class FrameStlAllocator
{
public:
  using value_type = T;
  using is_always_equal = std::false_type;
  using propagate_on_container_copy_assignment = std::true_type;
  using propagate_on_container_move_assignment = std::true_type;
  using propagate_on_container_swap = std::true_type;

  explicit FrameStlAllocator(FrameAllocator& arena) noexcept : mArena(&arena) {}

  constexpr FrameStlAllocator(const FrameStlAllocator& other) noexcept
      = default;

  constexpr FrameStlAllocator& operator=(
      const FrameStlAllocator& other) noexcept = default;

  template <typename U>
  constexpr FrameStlAllocator(const FrameStlAllocator<U>& other) noexcept
    : mArena(other.mArena)
  {
  }

  [[nodiscard]] T* allocate(std::size_t n)
  {
    if (n > std::numeric_limits<std::size_t>::max() / sizeof(T)) [[unlikely]] {
      throw std::bad_alloc();
    }

    const std::size_t bytes = n * sizeof(T);
    void* p;
    // Cache-line alignment helps large value pages and over-aligned SIMD
    // payloads. Smaller scalar arrays stay compact on the default 32-byte
    // frame invariant to reduce cache/TLB pressure during one-shot bake work.
    if constexpr (alignof(T) > 32 || sizeof(T) >= 64) {
      p = mArena->allocateStlStorageCacheAligned(bytes, false);
    } else {
      p = mArena->allocateDefaultAligned(bytes);
    }
    if (!p) [[unlikely]] {
      throw std::bad_alloc();
    }
    return static_cast<T*>(p);
  }

  void deallocate(T*, std::size_t) noexcept
  {
    // Arena semantics: memory freed on reset()
  }

  template <typename U, typename... Args>
  void construct(U* pointer, Args&&... args)
  {
    std::construct_at(pointer, std::forward<Args>(args)...);
  }

  template <typename U>
  void destroy(U* pointer) noexcept
  {
    if constexpr (!std::is_trivially_destructible_v<U>) {
      std::destroy_at(pointer);
    }
  }

  template <typename U>
  bool operator==(const FrameStlAllocator<U>& other) const noexcept
  {
    return mArena == other.mArena;
  }

private:
  template <typename U>
  friend class FrameStlAllocator;
  FrameAllocator* mArena;
};

} // namespace dart::common

#endif // DART_COMMON_FRAMEALLOCATOR_HPP_
