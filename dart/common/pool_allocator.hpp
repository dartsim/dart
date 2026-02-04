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
      return mBaseAllocator.allocate(bytes);
    }

    const int heapIndex = mMapSizeToHeapIndex[bytes];

    if (MemoryUnit* unit = mFreeMemoryUnits[heapIndex]) [[likely]] {
      mFreeMemoryUnits[heapIndex] = unit->mNext;
      return unit;
    }

    return allocateSlow(heapIndex);
  }

  // Documentation inherited
  inline void deallocate(void* pointer, size_t bytes) override
  {
    if (pointer == nullptr || bytes == 0) [[unlikely]] {
      return;
    }

    if (bytes > MAX_UNIT_SIZE) [[unlikely]] {
      mBaseAllocator.deallocate(pointer, bytes);
      return;
    }

    const int heapIndex = mMapSizeToHeapIndex[bytes];
    MemoryUnit* releasedUnit = static_cast<MemoryUnit*>(pointer);
    releasedUnit->mNext = mFreeMemoryUnits[heapIndex];
    mFreeMemoryUnits[heapIndex] = releasedUnit;
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
  };

  [[nodiscard]] void* allocateSlow(int heapIndex) noexcept;

  inline static constexpr int HEAP_COUNT = 128;

  inline static constexpr size_t MAX_UNIT_SIZE = 1024;

  inline static constexpr size_t BLOCK_SIZE = 16 * MAX_UNIT_SIZE;

  inline static std::array<size_t, HEAP_COUNT> mUnitSizes;

  inline static std::array<int, MAX_UNIT_SIZE + 1> mMapSizeToHeapIndex;

  inline static bool mInitialized = false;

  MemoryAllocator& mBaseAllocator;

  MemoryBlock* mMemoryBlocks;

  int mMemoryBlocksSize;

  int mCurrentMemoryBlockIndex;

  std::array<MemoryUnit*, HEAP_COUNT> mFreeMemoryUnits;
};

} // namespace dart::common

#endif // DART_COMMON_POOLALLOCATOR_HPP_
