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

#include <dart/common/MemoryAllocator.hpp>
#include <dart/common/MemoryAllocatorDebugger.hpp>

#include <algorithm>
#include <limits>
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

/// Bump-pointer arena allocator with bulk reset semantics.
///
/// FrameAllocator reserves one contiguous arena from a base allocator and
/// serves small allocations by moving a cursor through that arena. Individual
/// deallocations are intentionally no-ops; all arena allocations are made
/// available again by calling reset(). Requests that do not fit in the current
/// arena are allocated from the base allocator and tracked separately until the
/// next reset() or destruction.
class FrameAllocator : public MemoryAllocator
{
public:
  using Debug = MemoryAllocatorDebugger<FrameAllocator>;

  /// Constructor.
  ///
  /// \param[in] baseAllocator Base memory allocator.
  /// \param[in] initialCapacity Bytes to reserve for the frame arena.
  explicit FrameAllocator(
      MemoryAllocator& baseAllocator = MemoryAllocator::GetDefault(),
      size_t initialCapacity = 65536);

  /// Destructor.
  ~FrameAllocator() override;

  FrameAllocator(const FrameAllocator&) = delete;
  FrameAllocator& operator=(const FrameAllocator&) = delete;
  FrameAllocator(FrameAllocator&&) = delete;
  FrameAllocator& operator=(FrameAllocator&&) = delete;

  DART_STRING_TYPE(FrameAllocator);

  /// Returns the base allocator.
  [[nodiscard]] const MemoryAllocator& getBaseAllocator() const;

  /// Returns the base allocator.
  [[nodiscard]] MemoryAllocator& getBaseAllocator();

  /// Allocates \c bytes using the default 32-byte frame alignment.
  ///
  /// This fast path is kept inline to avoid function-call overhead in tight
  /// allocation loops.
  [[nodiscard]] inline void* allocate(size_t bytes) noexcept override
  {
    return allocateDefaultAligned(bytes);
  }

  /// Allocates \c bytes with at least \c alignment byte alignment.
  ///
  /// \param[in] bytes Number of bytes to allocate.
  /// \param[in] alignment Requested power-of-two alignment.
  /// \return Aligned storage on success; nullptr on failure.
  [[nodiscard]] inline void* allocate(size_t bytes, size_t alignment) noexcept
  {
    return allocateAligned(bytes, alignment);
  }

  /// Deallocates storage allocated by this allocator.
  ///
  /// This is intentionally a no-op because frame allocations are freed in bulk
  /// by reset().
  inline void deallocate(void* /*pointer*/, size_t /*bytes*/) override
  {
    // Arena semantics: memory is freed in bulk on reset().
  }

  /// Deallocates aligned storage allocated by this allocator.
  ///
  /// This is intentionally a no-op because frame allocations are freed in bulk
  /// by reset().
  inline void deallocate(
      void* /*pointer*/, size_t /*bytes*/, size_t /*alignment*/)
  {
    // Arena semantics: memory is freed in bulk on reset().
  }

  // Documentation inherited.
  void print(std::ostream& os = std::cout, int indent = 0) const override;

  /// Allocates \c bytes with at least \c alignment byte alignment.
  ///
  /// \param[in] bytes Number of bytes to allocate.
  /// \param[in] alignment Requested power-of-two alignment.
  /// \return Aligned storage on success; nullptr on failure.
  [[nodiscard]] inline void* allocateAligned(
      size_t bytes, size_t alignment = 32) noexcept
  {
    if (bytes == 0 || alignment == 0 || (alignment & (alignment - 1)) != 0) {
      return nullptr;
    }

    if (!mCur) {
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

    // Round the cursor to a 32-byte boundary so mixed aligned/default
    // allocations preserve the default fast-path invariant.
    auto* next = reinterpret_cast<char*>((unpaddedNext + 31) & ~uintptr_t{31});

    if (next <= mEnd) {
      mCur = next;
      return reinterpret_cast<void*>(aligned);
    }

    return allocateAlignedSlow(bytes, alignment);
  }

  /// Allocates cache-line-aligned storage.
  ///
  /// \param[in] bytes Number of bytes to allocate.
  /// \return 64-byte-aligned storage on success; nullptr on failure.
  [[nodiscard]] inline void* allocateCacheAligned(size_t bytes) noexcept
  {
    return allocateCacheAlignedFast(bytes, false);
  }

  /// Makes all arena allocations available for reuse.
  ///
  /// Overflow allocations are released to the base allocator. If overflow was
  /// needed during the frame, reset() may grow the arena to reduce future
  /// overflow traffic.
  inline void reset() noexcept
  {
    if (mOverflowBytes == 0) {
      mCur = mBegin;
      mCacheLineColor = 2;
      return;
    }
    resetSlow();
  }

  /// Allocates storage and constructs an object of type \c T.
  ///
  /// \tparam T Object type to construct.
  /// \tparam Args Constructor argument types.
  /// \param[in] args Constructor arguments.
  /// \return Constructed object on success; nullptr on failure.
  template <typename T, typename... Args>
  [[nodiscard]] T* construct(Args&&... args)
  {
    const size_t alignment = std::max<size_t>(alignof(T), 32);
    void* memory = allocateAligned(sizeof(T), alignment);
    if (memory == nullptr) {
      return nullptr;
    }
    return new (memory) T(std::forward<Args>(args)...);
  }

  /// Returns the total bytes reserved for the primary arena.
  [[nodiscard]] size_t capacity() const noexcept;

  /// Returns the usable primary-arena bytes after alignment padding.
  [[nodiscard]] size_t usableCapacity() const noexcept;

  /// Returns the bytes consumed in the primary arena since the last reset().
  [[nodiscard]] size_t used() const noexcept;

  /// Returns the number of live overflow allocations.
  [[nodiscard]] size_t overflowCount() const noexcept;

  /// Returns the bytes currently reserved for overflow allocations.
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
    if (bytes == 0 || !mCur) {
      return bytes == 0 ? nullptr : allocateAlignedSlow(bytes, 32);
    }

    const auto remaining = static_cast<size_t>(mEnd - mCur);
    if (bytes <= remaining) {
      const auto padded = (bytes + 31) & ~size_t{31};
      if (padded > remaining) {
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
    if (bytes == 0 || !mCur) {
      return bytes == 0 ? nullptr : allocateAlignedSlow(bytes, 64);
    }
    if (bytes > std::numeric_limits<size_t>::max() - 31) {
      return allocateAlignedSlow(bytes, 64);
    }

    const auto remaining = static_cast<size_t>(mEnd - mCur);
    const auto addr = reinterpret_cast<uintptr_t>(mCur);
    const auto alignPadding = static_cast<size_t>((uintptr_t{0} - addr) & 63u);
    const auto colorPadding
        = colorStorage ? static_cast<size_t>(mCacheLineColor) * 256u : 0u;
    const auto padding = alignPadding + colorPadding;
    if (padding <= remaining) {
      const auto available = remaining - padding;
      const auto padded = (bytes + 31) & ~size_t{31};
      if (padded <= available) {
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

  /// Handles allocations that cannot be served by the primary arena fast path.
  [[nodiscard]] void* allocateAlignedSlow(
      size_t bytes, size_t alignment) noexcept;

  /// Releases overflow allocations and grows the primary arena when possible.
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
///
/// Allocated memory is freed only by FrameAllocator::reset(), not by individual
/// deallocate() calls.
template <typename T>
class FrameStlAllocator
{
public:
  using value_type = T;
  using is_always_equal = std::false_type;
  using propagate_on_container_copy_assignment = std::true_type;
  using propagate_on_container_move_assignment = std::true_type;
  using propagate_on_container_swap = std::true_type;

  /// Constructor.
  ///
  /// \param[in] arena Frame allocator that will back this STL adapter.
  explicit FrameStlAllocator(FrameAllocator& arena) noexcept : mArena(&arena) {}

  /// Copy constructor.
  constexpr FrameStlAllocator(
      const FrameStlAllocator& other) noexcept = default;

  /// Copy assignment operator.
  constexpr FrameStlAllocator& operator=(
      const FrameStlAllocator& other) noexcept = default;

  /// Rebinding copy constructor.
  template <typename U>
  constexpr FrameStlAllocator(const FrameStlAllocator<U>& other) noexcept
    : mArena(other.mArena)
  {
    // Do nothing
  }

  /// Allocates storage for \c n objects.
  ///
  /// \param[in] n Number of objects to allocate.
  /// \return Storage for \c n objects.
  /// \throws std::bad_alloc on allocation failure.
  [[nodiscard]] T* allocate(std::size_t n)
  {
    if (n > std::numeric_limits<std::size_t>::max() / sizeof(T)) {
      throw std::bad_alloc();
    }

    const std::size_t bytes = n * sizeof(T);
    void* pointer;
    if (alignof(T) > 64) {
      pointer = mArena->allocateAligned(bytes, alignof(T));
    } else if (alignof(T) > 32 || sizeof(T) >= 64) {
      pointer = mArena->allocateStlStorageCacheAligned(bytes, false);
    } else {
      pointer = mArena->allocateDefaultAligned(bytes);
    }

    if (!pointer) {
      throw std::bad_alloc();
    }

    return static_cast<T*>(pointer);
  }

  /// Deallocates storage allocated by this adapter.
  ///
  /// This is intentionally a no-op because the backing arena is freed in bulk
  /// by FrameAllocator::reset().
  void deallocate(T*, std::size_t) noexcept
  {
    // Arena semantics: memory is freed in bulk on reset().
  }

  /// Constructs an object at existing storage.
  template <typename U, typename... Args>
  void construct(U* pointer, Args&&... args)
  {
    new (pointer) U(std::forward<Args>(args)...);
  }

  /// Destroys an object at existing storage.
  template <typename U>
  void destroy(U* pointer) noexcept
  {
    if (!std::is_trivially_destructible<U>::value) {
      pointer->~U();
    }
  }

  /// Returns true if two adapters use the same arena.
  template <typename U>
  bool operator==(const FrameStlAllocator<U>& other) const noexcept
  {
    return mArena == other.mArena;
  }

  /// Returns true if two adapters use different arenas.
  template <typename U>
  bool operator!=(const FrameStlAllocator<U>& other) const noexcept
  {
    return !(*this == other);
  }

private:
  template <typename U>
  friend class FrameStlAllocator;
  FrameAllocator* mArena;
};

} // namespace dart::common

#endif // DART_COMMON_FRAMEALLOCATOR_HPP_
