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
#include <new>
#include <stdexcept>
#include <utility>
#include <vector>

#include <cstddef>
#include <cstdint>

namespace dart::common {

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
    if (bytes == 0 || !mCur) [[unlikely]] {
      return bytes == 0 ? nullptr : allocateAlignedSlow(bytes, 32);
    }

    const auto padded = (bytes + 31) & ~size_t{31};
    auto* next = mCur + padded;

    if (next <= mEnd) [[likely]] {
      auto* result = mCur;
      mCur = next;
      return result;
    }

    return allocateAlignedSlow(bytes, 32);
  }

  inline void deallocate(void* /*pointer*/, size_t /*bytes*/) override
  {
    // Arena semantics: memory is freed in bulk on reset()
  }

  void print(std::ostream& os = std::cout, int indent = 0) const override;

  [[nodiscard]] inline void* allocateAligned(
      size_t bytes, size_t alignment = 32) noexcept
  {
    if (bytes == 0 || alignment == 0 || !mCur) [[unlikely]] {
      return (bytes == 0 || alignment == 0)
                 ? nullptr
                 : allocateAlignedSlow(bytes, alignment);
    }

    const auto cur = reinterpret_cast<uintptr_t>(mCur);
    const auto aligned = (cur + alignment - 1) & ~(alignment - 1);
    // Round next to 32-byte boundary so that allocate()'s fast path
    // invariant (mCur is 32-aligned) is preserved after mixed calls.
    auto* next
        = reinterpret_cast<char*>((aligned + bytes + 31) & ~uintptr_t{31});

    if (next <= mEnd) [[likely]] {
      mCur = next;
      return reinterpret_cast<void*>(aligned);
    }

    return allocateAlignedSlow(bytes, alignment);
  }

  inline void reset() noexcept
  {
    if (mOverflowAllocations.empty()) [[likely]] {
      const auto addr = reinterpret_cast<uintptr_t>(mBuffer);
      mCur = reinterpret_cast<char*>((addr + 31) & ~uintptr_t{31});
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
    return new (memory) T(std::forward<Args>(args)...);
  }

  [[nodiscard]] size_t capacity() const noexcept;

  [[nodiscard]] size_t used() const noexcept;

  [[nodiscard]] size_t overflowCount() const noexcept;

private:
  struct OverflowEntry
  {
    void* ptr;
    size_t allocatedSize;
  };

  [[nodiscard]] void* allocateAlignedSlow(
      size_t bytes, size_t alignment) noexcept;
  void resetSlow() noexcept;

  MemoryAllocator& mBaseAllocator;

  char* mCur;
  char* mEnd;

  char* mBuffer;
  size_t mCapacity;
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

  explicit FrameStlAllocator(FrameAllocator& arena) noexcept : mArena(&arena) {}

  template <typename U>
  FrameStlAllocator(const FrameStlAllocator<U>& other) noexcept
    : mArena(other.mArena)
  {
  }

  [[nodiscard]] T* allocate(std::size_t n)
  {
    void* p = mArena->allocateAligned(n * sizeof(T), alignof(T));
    if (!p) {
      throw std::bad_alloc();
    }
    return static_cast<T*>(p);
  }

  void deallocate(T*, std::size_t) noexcept
  {
    // Arena semantics: memory freed on reset()
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
