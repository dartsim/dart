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

#ifndef DART_COMMON_FIXEDPOOLALLOCATOR_HPP_
#define DART_COMMON_FIXEDPOOLALLOCATOR_HPP_

#include <dart/common/memory_allocator.hpp>
#include <dart/common/memory_allocator_debugger.hpp>

#include <dart/export.hpp>

#include <limits>

#include <cstddef>

namespace dart::common {

/// Memory allocator optimized for one fixed allocation size.
class DART_API FixedPoolAllocator : public MemoryAllocator
{
public:
  using Debug = MemoryAllocatorDebugger<FixedPoolAllocator>;

  /// Constructor
  ///
  /// @param[in] unitSize: Maximum byte size served by each fixed slot.
  /// @param[in] baseAllocator: Base memory allocator.
  /// @param[in] blockSize: Bytes reserved from the base allocator per block.
  explicit FixedPoolAllocator(
      size_t unitSize,
      MemoryAllocator& baseAllocator = MemoryAllocator::GetDefault(),
      size_t blockSize = 16384);

  /// Destructor
  ~FixedPoolAllocator() override;

  DART_STRING_TYPE(FixedPoolAllocator);

  /// Returns the base allocator.
  [[nodiscard]] const MemoryAllocator& getBaseAllocator() const;

  /// Returns the base allocator.
  [[nodiscard]] MemoryAllocator& getBaseAllocator();

  /// Returns the rounded fixed slot size.
  [[nodiscard]] size_t getUnitSize() const;

  /// Returns the count of allocated memory blocks.
  [[nodiscard]] int getNumAllocatedMemoryBlocks() const;

  /// Allocates one fixed-size slot.
  [[nodiscard]] inline void* allocate() noexcept
  {
    if (MemoryUnit* unit = mFreeMemoryUnits) [[likely]] {
      mFreeMemoryUnits = unit->mNext;
      return unit;
    }

    return allocateSlow();
  }

  /// Deallocates one fixed-size slot.
  inline void deallocate(void* pointer) noexcept
  {
    if (pointer == nullptr) [[unlikely]] {
      return;
    }

    MemoryUnit* releasedUnit = static_cast<MemoryUnit*>(pointer);
    releasedUnit->mNext = mFreeMemoryUnits;
    mFreeMemoryUnits = releasedUnit;
  }

  // Documentation inherited
  [[nodiscard]] inline void* allocate(size_t bytes) noexcept override
  {
    if (bytes == 0 || mUnitSize == 0) [[unlikely]] {
      return nullptr;
    }

    if (bytes > mUnitSize) [[unlikely]] {
      return mBaseAllocator.allocate(bytes);
    }

    return allocate();
  }

  // Documentation inherited
  [[nodiscard]] inline void* allocate(
      size_t bytes, size_t alignment) noexcept override
  {
    const size_t unitSize = effectiveUnitSize(bytes, alignment);
    if (unitSize == 0 || mUnitSize == 0) [[unlikely]] {
      return nullptr;
    }

    if (unitSize > mUnitSize || alignment > mBlockAlignment) [[unlikely]] {
      return mBaseAllocator.allocate(bytes, alignment);
    }

    return allocate();
  }

  // Documentation inherited
  inline void deallocate(void* pointer, size_t bytes) override
  {
    if (pointer == nullptr || bytes == 0 || mUnitSize == 0) [[unlikely]] {
      return;
    }

    if (bytes > mUnitSize) [[unlikely]] {
      mBaseAllocator.deallocate(pointer, bytes);
      return;
    }

    deallocate(pointer);
  }

  // Documentation inherited
  inline void deallocate(void* pointer, size_t bytes, size_t alignment) override
  {
    if (pointer == nullptr || bytes == 0 || mUnitSize == 0) [[unlikely]] {
      return;
    }

    const size_t unitSize = effectiveUnitSize(bytes, alignment);
    if (unitSize == 0) [[unlikely]] {
      return;
    }

    if (unitSize > mUnitSize || alignment > mBlockAlignment) [[unlikely]] {
      mBaseAllocator.deallocate(pointer, bytes, alignment);
      return;
    }

    deallocate(pointer);
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
    size_t mSize;
    size_t mAlignment;
  };

  [[nodiscard]] void* allocateSlow() noexcept;

  [[nodiscard]] static constexpr bool isPowerOfTwo(size_t value) noexcept
  {
    return value != 0 && (value & (value - 1)) == 0;
  }

  [[nodiscard]] static constexpr size_t roundUp(
      size_t bytes, size_t alignment) noexcept
  {
    return (bytes + alignment - 1) & ~(alignment - 1);
  }

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

  [[nodiscard]] static constexpr size_t blockAlignmentForUnitSize(
      size_t unitSize) noexcept
  {
    size_t alignment = alignof(MemoryUnit);
    while (alignment <= std::numeric_limits<size_t>::max() / 2
           && unitSize % (alignment * 2) == 0) {
      alignment *= 2;
    }
    return alignment;
  }

  MemoryAllocator& mBaseAllocator;

  size_t mUnitSize;

  size_t mBlockSize;

  size_t mBlockAlignment;

  MemoryBlock* mMemoryBlocks;

  int mMemoryBlocksSize;

  int mCurrentMemoryBlockIndex;

  MemoryUnit* mFreeMemoryUnits;
};

} // namespace dart::common

#endif // DART_COMMON_FIXEDPOOLALLOCATOR_HPP_
