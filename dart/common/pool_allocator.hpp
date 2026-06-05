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

#ifndef DART_COMMON_POOLALLOCATOR_HPP_
#define DART_COMMON_POOLALLOCATOR_HPP_

#include <dart/common/memory_allocator.hpp>
#include <dart/common/memory_allocator_debugger.hpp>

#include <dart/export.hpp>

#include <array>
#include <limits>

namespace dart::common {

/// Memory allocator optimized for allocating many objects of the same or
/// similar sizes
class DART_API PoolAllocator : public MemoryAllocator
{
public:
  using Debug = MemoryAllocatorDebugger<PoolAllocator>;

  /// Constructor
  ///
  /// @param[in] baseAllocator: (optional) Base memory allocator.
  /// @param[in] initialAllocation: (optional) Bytes to initially allocate.
  explicit PoolAllocator(
      MemoryAllocator& baseAllocator = MemoryAllocator::GetDefault());

  /// Destructor
  ~PoolAllocator() override;

  DART_STRING_TYPE(PoolAllocator);

  /// Returns the base allocator
  [[nodiscard]] const MemoryAllocator& getBaseAllocator() const;

  /// Returns the base allocator
  [[nodiscard]] MemoryAllocator& getBaseAllocator();

  /// Returns the count of allocated memory blocks
  [[nodiscard]] int getNumAllocatedMemoryBlocks() const;

  /// Returns the number of user-requested bytes currently allocated from this
  /// allocator.
  [[nodiscard]] size_t getAllocatedSize() const;

  /// Returns the largest user-requested live byte total observed by this
  /// allocator.
  [[nodiscard]] size_t getPeakAllocatedSize() const;

  /// Returns the number of currently live allocations from this allocator.
  [[nodiscard]] size_t getAllocationCount() const;

  // PERF: These fast paths MUST stay inline. Moving them out-of-line causes
  // 2-5x regression from function-call overhead (benchmarked). Do NOT move
  // to .cpp — see tests/benchmark/common/bm_allocators.cpp.

  // Documentation inherited
  [[nodiscard]] inline void* allocate(size_t bytes) noexcept override
  {
    if (bytes == 0) [[unlikely]] {
      return nullptr;
    }

    if (bytes > MAX_UNIT_SIZE) [[unlikely]] {
      if (defaultUnitSize(bytes) == 0) {
        return nullptr;
      }
      void* pointer = mBaseAllocator.allocate(bytes);
      if (pointer != nullptr) {
        recordAllocation(bytes);
      }
      return pointer;
    }

    const int heapIndex = heapIndexForDefaultBytes(bytes);

    if (MemoryUnit* unit = mFreeMemoryUnits[heapIndex]) [[likely]] {
      mFreeMemoryUnits[heapIndex] = unit->mNext;
      recordAllocation(bytes);
      return unit;
    }

    void* pointer = allocateSlow(heapIndex);
    if (pointer != nullptr) {
      recordAllocation(bytes);
    }
    return pointer;
  }

  // Documentation inherited
  [[nodiscard]] inline void* allocate(
      size_t bytes, size_t alignment) noexcept override
  {
    const size_t unitSize = effectiveUnitSize(bytes, alignment);
    if (unitSize == 0) [[unlikely]] {
      return nullptr;
    }

    if (unitSize > MAX_UNIT_SIZE) [[unlikely]] {
      void* pointer = mBaseAllocator.allocate(bytes, alignment);
      if (pointer != nullptr) {
        recordAllocation(bytes);
      }
      return pointer;
    }

    const int heapIndex = heapIndexForUnitSize(unitSize);

    if (MemoryUnit* unit = mFreeMemoryUnits[heapIndex]) [[likely]] {
      mFreeMemoryUnits[heapIndex] = unit->mNext;
      recordAllocation(bytes);
      return unit;
    }

    void* pointer = allocateSlow(heapIndex);
    if (pointer != nullptr) {
      recordAllocation(bytes);
    }
    return pointer;
  }

  // Documentation inherited
  inline void deallocate(void* pointer, size_t bytes) override
  {
    if (pointer == nullptr || bytes == 0) [[unlikely]] {
      return;
    }

    if (bytes > MAX_UNIT_SIZE) [[unlikely]] {
      if (defaultUnitSize(bytes) == 0) {
        return;
      }
      mBaseAllocator.deallocate(pointer, bytes);
      recordDeallocation(bytes);
      return;
    }

    const int heapIndex = heapIndexForDefaultBytes(bytes);
    MemoryUnit* releasedUnit = static_cast<MemoryUnit*>(pointer);
    releasedUnit->mNext = mFreeMemoryUnits[heapIndex];
    mFreeMemoryUnits[heapIndex] = releasedUnit;
    recordDeallocation(bytes);
  }

  // Documentation inherited
  inline void deallocate(void* pointer, size_t bytes, size_t alignment) override
  {
    const size_t unitSize = effectiveUnitSize(bytes, alignment);
    if (pointer == nullptr || unitSize == 0) [[unlikely]] {
      return;
    }

    if (unitSize > MAX_UNIT_SIZE) [[unlikely]] {
      mBaseAllocator.deallocate(pointer, bytes, alignment);
      recordDeallocation(bytes);
      return;
    }

    const int heapIndex = heapIndexForUnitSize(unitSize);
    MemoryUnit* releasedUnit = static_cast<MemoryUnit*>(pointer);
    releasedUnit->mNext = mFreeMemoryUnits[heapIndex];
    mFreeMemoryUnits[heapIndex] = releasedUnit;
    recordDeallocation(bytes);
  }

  // Documentation inherited
  void print(std::ostream& os = std::cout, int indent = 0) const override;

private:
  struct MemoryUnit
  {
    MemoryUnit* mNext;
  };

  struct MemoryBlock
  {
    MemoryUnit* mMemoryUnits;
    size_t mAlignment;
  };

  [[nodiscard]] void* allocateSlow(int heapIndex) noexcept;

  void recordAllocation(size_t bytes) noexcept
  {
    mDiagnosticAllocatedSize += bytes;
    if (mDiagnosticPeakAllocatedSize < mDiagnosticAllocatedSize) {
      mDiagnosticPeakAllocatedSize = mDiagnosticAllocatedSize;
    }
    ++mDiagnosticAllocationCount;
  }

  void recordDeallocation(size_t bytes) noexcept
  {
    if (bytes <= mDiagnosticAllocatedSize) {
      mDiagnosticAllocatedSize -= bytes;
    } else {
      mDiagnosticAllocatedSize = 0;
    }

    if (mDiagnosticAllocationCount > 0) {
      --mDiagnosticAllocationCount;
    }
  }

  [[nodiscard]] static constexpr bool isPowerOfTwo(size_t value) noexcept
  {
    return value != 0 && (value & (value - 1)) == 0;
  }

  [[nodiscard]] static constexpr size_t roundUp(
      size_t bytes, size_t alignment) noexcept
  {
    return (bytes + alignment - 1) & ~(alignment - 1);
  }

  inline static constexpr size_t UNIT_GRANULARITY = 8;

  [[nodiscard]] static constexpr size_t effectiveUnitSize(
      size_t bytes, size_t alignment) noexcept
  {
    if (bytes == 0 || !isPowerOfTwo(alignment)) {
      return 0;
    }

    const size_t effectiveAlignment
        = alignment < alignof(MemoryUnit) ? alignof(MemoryUnit) : alignment;
    const size_t minimumSize
        = bytes < sizeof(MemoryUnit) ? sizeof(MemoryUnit) : bytes;
    if (minimumSize
        > std::numeric_limits<size_t>::max() - (effectiveAlignment - 1)) {
      return 0;
    }
    return roundUp(minimumSize, effectiveAlignment);
  }

  [[nodiscard]] static constexpr size_t defaultUnitSize(size_t bytes) noexcept
  {
    if (bytes == 0) {
      return 0;
    }

    const size_t minimumSize
        = bytes < sizeof(MemoryUnit) ? sizeof(MemoryUnit) : bytes;
    if (minimumSize
        > std::numeric_limits<size_t>::max() - (UNIT_GRANULARITY - 1)) {
      return 0;
    }
    return roundUp(minimumSize, UNIT_GRANULARITY);
  }

  [[nodiscard]] static constexpr int heapIndexForUnitSize(
      size_t unitSize) noexcept
  {
    return static_cast<int>(unitSize / UNIT_GRANULARITY - 1);
  }

  [[nodiscard]] static constexpr int heapIndexForDefaultBytes(
      size_t bytes) noexcept
  {
    return static_cast<int>((bytes - 1) / UNIT_GRANULARITY);
  }

  [[nodiscard]] static constexpr size_t blockAlignmentForUnitSize(
      size_t unitSize) noexcept
  {
    size_t alignment = alignof(MemoryUnit);
    while (unitSize % (alignment * 2) == 0) {
      alignment *= 2;
    }
    return alignment;
  }

  inline static constexpr int HEAP_COUNT = 128;

  inline static constexpr size_t MAX_UNIT_SIZE = 1024;

  inline static constexpr size_t BLOCK_SIZE = 16 * MAX_UNIT_SIZE;

  inline static constexpr std::array<size_t, HEAP_COUNT> mUnitSizes = [] {
    std::array<size_t, HEAP_COUNT> sizes{};
    for (size_t i = 0; i < sizes.size(); ++i) {
      sizes[i] = (i + 1) * UNIT_GRANULARITY;
    }
    return sizes;
  }();

  MemoryAllocator& mBaseAllocator;

  MemoryBlock* mMemoryBlocks;

  int mMemoryBlocksSize;

  int mCurrentMemoryBlockIndex;

  std::array<MemoryUnit*, HEAP_COUNT> mFreeMemoryUnits;

  size_t mDiagnosticAllocatedSize{0};

  size_t mDiagnosticPeakAllocatedSize{0};

  size_t mDiagnosticAllocationCount{0};
};

} // namespace dart::common

#endif // DART_COMMON_POOLALLOCATOR_HPP_
