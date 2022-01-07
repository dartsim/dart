/*
 * Copyright (c) 2011-2022, The DART development contributors
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

#include "dart/common/PoolAllocator.hpp"

#include <cstring>

#include "dart/common/Console.hpp"
#include "dart/common/Logging.hpp"
#include "dart/common/Macros.hpp"

namespace dart::common {

//==============================================================================
PoolAllocator::PoolAllocator(MemoryAllocator& baseAllocator)
  : mBaseAllocator(baseAllocator)
{
  static_assert(
      8 <= sizeof(MemoryUnit),
      "sizeof(MemoryUnit) should be equal to or greater than 8.");

  // Global setting
  if (!mInitialized)
  {
    // Fill mUnitSizes from 8 to 1024 with 8 interval
    for (auto i = 0u; i < HEAP_COUNT; ++i)
    {
      mUnitSizes[i] = (i + 1) * 8;
    }

    auto j = 0u;
    mMapSizeToHeapIndex[0] = -1;
    for (auto i = 1u; i <= MAX_UNIT_SIZE; ++i)
    {
      if (i <= mUnitSizes[j])
      {
        mMapSizeToHeapIndex[i] = j;
      }
      else
      {
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
  // Lock the mutex
  std::lock_guard<std::mutex> lock(mMutex);

  for (int i = 0; i < mCurrentMemoryBlockIndex; ++i)
  {
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
void* PoolAllocator::allocate(size_t bytes) noexcept
{
  // Cannot allocate zero bytes
  if (bytes == 0)
  {
    return nullptr;
  }

  // Use the default allocator to allocate memory that is greater than
  // MAX_UNIT_SIZE
  if (bytes > MAX_UNIT_SIZE)
  {
    DART_TRACE(
        "Cannot allocate memory of size > {} using PoolAllocator.",
        MAX_UNIT_SIZE);
    return mBaseAllocator.allocate(bytes);
  }

  // Lock the mutex
  std::lock_guard<std::mutex> lock(mMutex);

  const int heapIndex = mMapSizeToHeapIndex[bytes];

  if (MemoryUnit* unit = mFreeMemoryUnits[heapIndex])
  {
    mFreeMemoryUnits[heapIndex] = unit->mNext;
    return unit;
  }

  if (mCurrentMemoryBlockIndex == mMemoryBlocksSize)
  {
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
  void* memoryUnitsBegin = static_cast<void*>(newBlock->mMemoryUnits);
  char* memoryUnitsBeginChar = static_cast<char*>(memoryUnitsBegin);
  for (size_t i = 0u; i < unitCount - 1; ++i)
  {
    void* unitPointer = static_cast<void*>(memoryUnitsBeginChar + unitSize * i);
    void* nextUnitPointer
        = static_cast<void*>(memoryUnitsBeginChar + unitSize * (i + 1));
    MemoryUnit* unit = static_cast<MemoryUnit*>(unitPointer);
    MemoryUnit* nextUnit = static_cast<MemoryUnit*>(nextUnitPointer);
    unit->mNext = nextUnit;
  }

  void* lastUnitPointer
      = static_cast<void*>(memoryUnitsBeginChar + unitSize * (unitCount - 1));
  MemoryUnit* lastUnit = static_cast<MemoryUnit*>(lastUnitPointer);
  lastUnit->mNext = nullptr;

  mFreeMemoryUnits[heapIndex] = newBlock->mMemoryUnits->mNext;
  mCurrentMemoryBlockIndex++;

  return newBlock->mMemoryUnits;
}

//==============================================================================
void PoolAllocator::deallocate(void* pointer, size_t bytes)
{
  // Cannot deallocate nullptr or zero bytes
  if (pointer == nullptr || bytes == 0)
  {
    return;
  }

  if (bytes > MAX_UNIT_SIZE)
  {
    return mBaseAllocator.deallocate(pointer, bytes);
  }

  // Lock the mutex
  std::lock_guard<std::mutex> lock(mMutex);

  const int heapIndex = mMapSizeToHeapIndex[bytes];

  MemoryUnit* releasedUnit = static_cast<MemoryUnit*>(pointer);
  releasedUnit->mNext = mFreeMemoryUnits[heapIndex];
  mFreeMemoryUnits[heapIndex] = releasedUnit;
}

//==============================================================================
void PoolAllocator::print(std::ostream& os, int indent) const
{
  // Lock the mutex
  std::lock_guard<std::mutex> lock(mMutex);

  if (indent == 0)
  {
    os << "[PoolAllocator]\n";
  }
  const std::string spaces(indent, ' ');
  if (indent != 0)
  {
    os << spaces << "type: " << getType() << "\n";
  }
  os << spaces << "allocated_memory_block_count: " << mMemoryBlocksSize << "\n";
  os << spaces << "current_memory_blocks_count: " << mCurrentMemoryBlockIndex
     << "\n";
  os << spaces << "base_allocator:\n";
  mBaseAllocator.print(os, indent + 2);
}

} // namespace dart::common
