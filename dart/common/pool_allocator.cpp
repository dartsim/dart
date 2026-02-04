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

#include "dart/common/pool_allocator.hpp"

#include "dart/common/logging.hpp"
#include "dart/common/macros.hpp"

#include <cstring>

namespace dart::common {

//==============================================================================
PoolAllocator::PoolAllocator(MemoryAllocator& baseAllocator)
  : mBaseAllocator(baseAllocator)
{
  static_assert(
      8 <= sizeof(MemoryUnit),
      "sizeof(MemoryUnit) should be equal to or greater than 8.");

  if (!mInitialized) {
    for (auto i = 0u; i < HEAP_COUNT; ++i) {
      mUnitSizes[i] = (i + 1) * 8;
    }

    auto j = 0u;
    mMapSizeToHeapIndex[0] = -1;
    for (auto i = 1u; i <= MAX_UNIT_SIZE; ++i) {
      if (i <= mUnitSizes[j]) {
        mMapSizeToHeapIndex[i] = j;
      } else {
        mMapSizeToHeapIndex[i] = ++j;
      }
    }

    mInitialized = true;
  }

  mCurrentMemoryBlockIndex = 0;

  mMemoryBlocksSize = 64;
  mMemoryBlocks = mBaseAllocator.allocateAs<MemoryBlock>(mMemoryBlocksSize);
  const size_t allocatedSize = mMemoryBlocksSize * sizeof(MemoryBlock);
  std::memset(mMemoryBlocks, 0, allocatedSize);

  mFreeMemoryUnits.fill(nullptr);
}

//==============================================================================
PoolAllocator::~PoolAllocator()
{
  for (int i = 0; i < mCurrentMemoryBlockIndex; ++i) {
    mBaseAllocator.deallocate(mMemoryBlocks[i].mMemoryUnits, BLOCK_SIZE);
  }
  mBaseAllocator.deallocate(
      mMemoryBlocks, mMemoryBlocksSize * sizeof(MemoryBlock));
}

//==============================================================================
const MemoryAllocator& PoolAllocator::getBaseAllocator() const
{
  return mBaseAllocator;
}

//==============================================================================
MemoryAllocator& PoolAllocator::getBaseAllocator()
{
  return mBaseAllocator;
}

//==============================================================================
int PoolAllocator::getNumAllocatedMemoryBlocks() const
{
  return mCurrentMemoryBlockIndex;
}

//==============================================================================
void* PoolAllocator::allocateSlow(int heapIndex) noexcept
{
  if (mCurrentMemoryBlockIndex == mMemoryBlocksSize) {
    MemoryBlock* currentMemoryBlocks = mMemoryBlocks;
    mMemoryBlocksSize += 64;
    mMemoryBlocks = mBaseAllocator.allocateAs<MemoryBlock>(mMemoryBlocksSize);
    std::memcpy(
        mMemoryBlocks,
        currentMemoryBlocks,
        mCurrentMemoryBlockIndex * sizeof(MemoryBlock));
    std::memset(
        mMemoryBlocks + mCurrentMemoryBlockIndex, 0, 64 * sizeof(MemoryBlock));
  }

  MemoryBlock* newBlock = mMemoryBlocks + mCurrentMemoryBlockIndex;
  newBlock->mMemoryUnits
      = static_cast<MemoryUnit*>(mBaseAllocator.allocate(BLOCK_SIZE));
  const size_t unitSize = mUnitSizes[heapIndex];
  DART_ASSERT(unitSize > 0);
  const unsigned int unitCount = BLOCK_SIZE / unitSize;
  DART_ASSERT(unitCount > 0);
  auto* memoryUnitsBeginChar = reinterpret_cast<char*>(newBlock->mMemoryUnits);
  for (size_t i = 0u; i < unitCount - 1; ++i) {
    auto* unit
        = reinterpret_cast<MemoryUnit*>(memoryUnitsBeginChar + unitSize * i);
    auto* nextUnit = reinterpret_cast<MemoryUnit*>(
        memoryUnitsBeginChar + unitSize * (i + 1));
    unit->mNext = nextUnit;
  }

  auto* lastUnit = reinterpret_cast<MemoryUnit*>(
      memoryUnitsBeginChar + unitSize * (unitCount - 1));
  lastUnit->mNext = nullptr;

  mFreeMemoryUnits[heapIndex] = newBlock->mMemoryUnits->mNext;
  mCurrentMemoryBlockIndex++;

  return newBlock->mMemoryUnits;
}

//==============================================================================
void PoolAllocator::print(std::ostream& os, int indent) const
{
  if (indent == 0) {
    os << "[PoolAllocator]\n";
  }
  const std::string spaces(indent, ' ');
  if (indent != 0) {
    os << spaces << "type: " << getType() << "\n";
  }
  os << spaces << "allocated_memory_block_count: " << mMemoryBlocksSize << "\n";
  os << spaces << "current_memory_blocks_count: " << mCurrentMemoryBlockIndex
     << "\n";
  os << spaces << "base_allocator:\n";
  mBaseAllocator.print(os, indent + 2);
}

} // namespace dart::common
