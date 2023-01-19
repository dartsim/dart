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

#include "dart/common/allocator/AllocatorBuddy.hpp"

#include "dart/common/Macros.hpp"

namespace dart::common {

//==============================================================================
AllocatorBuddy::AllocatorBuddy(
    size_t block_size, size_t memory_size, Allocator& base_allocator) noexcept
  : m_base_allocator(base_allocator),
    mBlockSize(block_size),
    mMemorySize(memory_size)
{
  mMaxLevel = log2(memory_size / block_size);
  mFreeLists.resize(mMaxLevel + 1);
  mFreeLists[mMaxLevel].push_back(0);
}

//==============================================================================
AllocatorBuddy::~AllocatorBuddy()
{
  // Do nothing
}

//==============================================================================
void* AllocatorBuddy::allocate(size_t bytes) noexcept
{
  if (bytes == 0) {
    return nullptr;
  }

  size_t level = std::log2(bytes / mBlockSize);

  if (level > mMaxLevel)
    return nullptr;

  // Find the first free block at or above the required level
  size_t block = 0;
  for (size_t i = level; i <= mMaxLevel; i++) {
    if (!mFreeLists[i].empty()) {
      block = mFreeLists[i].back();
      mFreeLists[i].pop_back();
      break;
    }
  }

  // Split the block into smaller blocks
  size_t buddy = block ^ (1 << level);
  while (level < mMaxLevel) {
    level++;
    mFreeLists[level].push_back(buddy);
    buddy = block ^ (1 << level);
  }

  return static_cast<void*>(mMemory + block * mBlockSize);
}

//==============================================================================
void AllocatorBuddy::deallocate(void* pointer, size_t bytes)
{
  DART_UNUSED(bytes);
  size_t block = (static_cast<char*>(pointer) - mMemory) / mBlockSize;

  size_t level = 0;
  size_t buddy = block ^ (1 << level);
  while (buddy < (1ull << mMaxLevel) && !mFreeLists[level].empty()
         && std::find(mFreeLists[level].begin(), mFreeLists[level].end(), buddy)
                != mFreeLists[level].end()) {
    // Merge the block with its buddy
    mFreeLists[level].erase(
        std::remove(mFreeLists[level].begin(), mFreeLists[level].end(), buddy),
        mFreeLists[level].end());
    block = std::min(block, buddy);
    buddy = block ^ (1 << ++level);
  }

  mFreeLists[level].push_back(block);
}

//==============================================================================
void AllocatorBuddy::print(std::ostream& os, int indent) const
{
  if (indent == 0) {
    os << "[dart::common::AllocatorBuddy]\n";
  }
  const std::string spaces(indent, ' ');
  if (indent != 0) {
    os << spaces << "type: " << getType() << "\n";
  }
}

} // namespace dart::common
