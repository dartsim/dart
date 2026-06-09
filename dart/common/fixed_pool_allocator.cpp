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

#include "dart/common/fixed_pool_allocator.hpp"

#include "dart/common/logging.hpp"
#include "dart/common/macros.hpp"

#include <algorithm>

#include <cstring>

namespace dart::common {

//==============================================================================
FixedPoolAllocator::FixedPoolAllocator(
    size_t unitSize, MemoryAllocator& baseAllocator, size_t blockSize)
  : mBaseAllocator(baseAllocator)
{
  static_assert(
      8 <= sizeof(MemoryUnit),
      "sizeof(MemoryUnit) should be equal to or greater than 8.");

  mUnitSize
      = cacheFriendlyUnitSize(effectiveUnitSize(unitSize, alignof(MemoryUnit)));
  mBlockSize = std::max(blockSize, mUnitSize);
  mBlockAlignment = mUnitSize == 0 ? alignof(MemoryUnit)
                                   : blockAlignmentForUnitSize(mUnitSize);
  mCurrentMemoryBlockIndex = 0;

  mMemoryBlocksSize = 64;
  mMemoryBlocks = mBaseAllocator.allocateAs<MemoryBlock>(mMemoryBlocksSize);
  if (mMemoryBlocks != nullptr) {
    const size_t allocatedSize = mMemoryBlocksSize * sizeof(MemoryBlock);
    std::memset(mMemoryBlocks, 0, allocatedSize);
  }

  mFreeMemoryUnits = nullptr;
}

//==============================================================================
FixedPoolAllocator::~FixedPoolAllocator()
{
  for (int i = 0; i < mCurrentMemoryBlockIndex; ++i) {
    mBaseAllocator.deallocate(
        mMemoryBlocks[i].mMemoryUnits,
        mMemoryBlocks[i].mSize,
        mMemoryBlocks[i].mAlignment);
  }
  if (mMemoryBlocks != nullptr) {
    mBaseAllocator.deallocate(
        mMemoryBlocks, mMemoryBlocksSize * sizeof(MemoryBlock));
  }
}

//==============================================================================
const MemoryAllocator& FixedPoolAllocator::getBaseAllocator() const
{
  return mBaseAllocator;
}

//==============================================================================
MemoryAllocator& FixedPoolAllocator::getBaseAllocator()
{
  return mBaseAllocator;
}

//==============================================================================
size_t FixedPoolAllocator::getUnitSize() const
{
  return mUnitSize;
}

//==============================================================================
int FixedPoolAllocator::getNumAllocatedMemoryBlocks() const
{
  return mCurrentMemoryBlockIndex;
}

//==============================================================================
void* FixedPoolAllocator::allocateSlow() noexcept
{
  if (mUnitSize == 0 || mMemoryBlocks == nullptr) {
    return nullptr;
  }

  if (mCurrentMemoryBlockIndex == mMemoryBlocksSize) {
    MemoryBlock* currentMemoryBlocks = mMemoryBlocks;
    const int currentMemoryBlocksSize = mMemoryBlocksSize;
    const int newMemoryBlocksSize = mMemoryBlocksSize + 64;
    MemoryBlock* newMemoryBlocks
        = mBaseAllocator.allocateAs<MemoryBlock>(newMemoryBlocksSize);
    if (newMemoryBlocks == nullptr) {
      return nullptr;
    }
    std::memcpy(
        newMemoryBlocks,
        currentMemoryBlocks,
        mCurrentMemoryBlockIndex * sizeof(MemoryBlock));
    std::memset(
        newMemoryBlocks + mCurrentMemoryBlockIndex,
        0,
        64 * sizeof(MemoryBlock));
    mBaseAllocator.deallocate(
        currentMemoryBlocks, currentMemoryBlocksSize * sizeof(MemoryBlock));
    mMemoryBlocks = newMemoryBlocks;
    mMemoryBlocksSize = newMemoryBlocksSize;
  }

  MemoryBlock* newBlock = mMemoryBlocks + mCurrentMemoryBlockIndex;
  newBlock->mMemoryUnits = static_cast<MemoryUnit*>(
      mBaseAllocator.allocate(mBlockSize, mBlockAlignment));
  if (newBlock->mMemoryUnits == nullptr) {
    return nullptr;
  }
  newBlock->mSize = mBlockSize;
  newBlock->mAlignment = mBlockAlignment;

  const size_t unitCount = mBlockSize / mUnitSize;
  DART_ASSERT(unitCount > 0);
  auto* memoryUnitsBeginChar = reinterpret_cast<char*>(newBlock->mMemoryUnits);
  for (size_t i = 0; i < unitCount - 1; ++i) {
    auto* unit
        = reinterpret_cast<MemoryUnit*>(memoryUnitsBeginChar + mUnitSize * i);
    auto* nextUnit = reinterpret_cast<MemoryUnit*>(
        memoryUnitsBeginChar + mUnitSize * (i + 1));
    unit->mNext = nextUnit;
  }

  auto* lastUnit = reinterpret_cast<MemoryUnit*>(
      memoryUnitsBeginChar + mUnitSize * (unitCount - 1));
  lastUnit->mNext = nullptr;

  mFreeMemoryUnits = newBlock->mMemoryUnits->mNext;
  mCurrentMemoryBlockIndex++;

  return newBlock->mMemoryUnits;
}

//==============================================================================
void FixedPoolAllocator::print(std::ostream& os, int indent) const
{
  if (indent == 0) {
    os << "[FixedPoolAllocator]\n";
  }
  const std::string spaces(indent, ' ');
  if (indent != 0) {
    os << spaces << "type: " << getType() << "\n";
  }
  os << spaces << "unit_size: " << mUnitSize << "\n";
  os << spaces << "block_size: " << mBlockSize << "\n";
  os << spaces << "allocated_memory_block_count: " << mMemoryBlocksSize << "\n";
  os << spaces << "current_memory_blocks_count: " << mCurrentMemoryBlockIndex
     << "\n";
  os << spaces << "base_allocator:\n";
  mBaseAllocator.print(os, indent + 2);
}

} // namespace dart::common
