/*
 * Copyright (c) 2011-2023, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#include <dart/common/Export.hpp>
#include <dart/common/allocator/Allocator.hpp>
#include <dart/common/allocator/AllocatorDebugger.hpp>

#include <array>
#include <mutex>

namespace dart::common {

/// Memory allocator optimized for allocating many objects of the same or
/// similar sizes
class DART_COMMON_API AllocatorPool : public Allocator
{
public:
  using Debug = AllocatorDebugger<AllocatorPool>;

  /// Constructor
  ///
  /// \param[in] baseAllocator: (optional) Base memory allocator.
  /// \param[in] initialAllocation: (optional) Bytes to initially allocate.
  explicit AllocatorPool(Allocator& baseAllocator = Allocator::GetDefault());

  /// Destructor
  ~AllocatorPool() override;

  DART_STRING_TYPE(AllocatorPool);

  /// Returns the base allocator
  [[nodiscard]] const Allocator& getBaseAllocator() const;

  /// Returns the base allocator
  [[nodiscard]] Allocator& getBaseAllocator();

  /// Returns the count of allocated memory blocks
  [[nodiscard]] int getNumAllocatedMemoryBlocks() const;

  // Documentation inherited
  [[nodiscard]] void* allocate(size_t bytes) noexcept override;

  // Documentation inherited
  void deallocate(void* pointer, size_t bytes) override;

  // Documentation inherited
  void print(std::ostream& os = std::cout, int indent = 0) const override;

private:
  struct MemoryUnit
  {
    /// Pointer to next memory block
    MemoryUnit* mNext;
  };

  struct MemoryBlock
  {
    /// Pointer to the first memory unit
    MemoryUnit* mMemoryUnits;
  };

  inline static constexpr int HEAP_COUNT = 128;

  inline static constexpr size_t MAX_UNIT_SIZE = 1024;

  inline static constexpr size_t BLOCK_SIZE = 16 * MAX_UNIT_SIZE;

  inline static std::array<size_t, HEAP_COUNT> mUnitSizes;

  inline static std::array<int, MAX_UNIT_SIZE + 1> mMapSizeToHeapIndex;

  inline static bool mInitialized = false;

  /// The base allocator to allocate memory chunk
  Allocator& mBaseAllocator;

  /// Mutex for for mNumAllocatedMemoryBlocks, mNumMemoryBlocks,
  /// mFreeMemoryUnits, and mAllocatedMemoryBlocks.
  mutable std::mutex mMutex;

  /// The array of memory blocks.
  ///
  /// This array is a placeholder of allocated memory blocks. Initially this
  /// contains nullptr as the elements.
  MemoryBlock* mMemoryBlocks;

  /// The size of mMemoryBlocks.
  ///
  /// This is simply the current size of mMemoryBlocks. The value doesn't mean
  /// the actual count of the allocated memory blocks.
  int mMemoryBlocksSize;

  /// The count of the allocated memory blocks in use.
  int mCurrentMemoryBlockIndex;

  /// List of free memory units.
  ///
  /// The size of mFreeMemoryUnits is fixed to HEAP_COUNT where each element is
  /// for a specific memory size. For example, the first element has the free
  /// memory unit for 8 bytes while the last element has the free memory unit
  /// for 1024 bytes.
  ///
  /// The size must be the same of mUnitSizes.
  std::array<MemoryUnit*, HEAP_COUNT> mFreeMemoryUnits;
};

} // namespace dart::common

#endif // DART_COMMON_POOLALLOCATOR_HPP_
