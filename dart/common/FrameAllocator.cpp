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

#include "dart/common/FrameAllocator.hpp"

#include <algorithm>
#include <limits>

namespace dart::common {

//==============================================================================
FrameAllocator::FrameAllocator(
    MemoryAllocator& baseAllocator, size_t initialCapacity)
  : mBaseAllocator(baseAllocator),
    mCur(nullptr),
    mEnd(nullptr),
    mBuffer(static_cast<char*>(baseAllocator.allocate(initialCapacity))),
    mBegin(nullptr),
    mCapacity(mBuffer ? initialCapacity : 0),
    mOverflowBytes(0),
    mCacheLineColor(2)
{
  if (mBuffer) {
    const auto addr = reinterpret_cast<uintptr_t>(mBuffer);
    const auto aligned = (addr + 63) & ~uintptr_t{63};
    char* begin = reinterpret_cast<char*>(aligned);
    char* end = mBuffer + mCapacity;
    if (begin < end) {
      mBegin = begin;
      mCur = begin;
      mEnd = end;
    } else {
      // The buffer is too small to hold a 64-byte-aligned arena, so leave the
      // fast-path window empty (mCur == mEnd) and serve every request from the
      // overflow path. Without this, mBegin > mEnd makes remaining = mEnd -
      // mCur underflow to a huge size_t and the fast path returns
      // out-of-bounds storage.
      mBegin = mBuffer;
      mCur = mBuffer;
      mEnd = mBuffer;
    }
  }
}

//==============================================================================
FrameAllocator::~FrameAllocator()
{
  for (const auto& entry : mOverflowAllocations) {
    mBaseAllocator.deallocate(entry.ptr, entry.allocatedSize);
  }
  mOverflowAllocations.clear();

  if (mBuffer) {
    mBaseAllocator.deallocate(mBuffer, mCapacity);
  }
}

//==============================================================================
const MemoryAllocator& FrameAllocator::getBaseAllocator() const
{
  return mBaseAllocator;
}

//==============================================================================
MemoryAllocator& FrameAllocator::getBaseAllocator()
{
  return mBaseAllocator;
}

//==============================================================================
void FrameAllocator::print(std::ostream& os, int indent) const
{
  const std::string spaces(indent, ' ');
  os << spaces << "[FrameAllocator] capacity: " << mCapacity
     << " usable: " << usableCapacity() << " used: " << used()
     << " overflow: " << mOverflowAllocations.size()
     << " overflow bytes: " << mOverflowBytes << "\n";
}

//==============================================================================
void* FrameAllocator::allocateAlignedSlow(
    size_t bytes, size_t alignment) noexcept
{
  if (bytes == 0 || alignment == 0 || (alignment & (alignment - 1)) != 0) {
    return nullptr;
  }
  if (bytes > std::numeric_limits<size_t>::max() - alignment) {
    return nullptr;
  }

  const size_t totalSize = bytes + alignment;
  void* raw = mBaseAllocator.allocate(totalSize);
  if (!raw) {
    return nullptr;
  }

  const auto rawAddr = reinterpret_cast<uintptr_t>(raw);
  const auto overflowAligned = (rawAddr + alignment - 1) & ~(alignment - 1);
  void* pointer = reinterpret_cast<void*>(overflowAligned);

  try {
    mOverflowAllocations.push_back({raw, totalSize});
  } catch (...) {
    mBaseAllocator.deallocate(raw, totalSize);
    return nullptr;
  }

  mOverflowBytes += totalSize;
  return pointer;
}

//==============================================================================
void FrameAllocator::resetSlow() noexcept
{
  const size_t totalOverflow = mOverflowBytes;
  for (const auto& entry : mOverflowAllocations) {
    mBaseAllocator.deallocate(entry.ptr, entry.allocatedSize);
  }
  mOverflowAllocations.clear();
  mOverflowBytes = 0;

  const size_t usedBytes
      = (mBuffer && mCur) ? static_cast<size_t>(mCur - mBuffer) : 0;
  const size_t needed = usedBytes + totalOverflow;
  const size_t newCapacity = std::max(mCapacity * 2, needed + 256);
  void* newBuffer = mBaseAllocator.allocate(newCapacity);
  if (newBuffer) {
    if (mBuffer) {
      mBaseAllocator.deallocate(mBuffer, mCapacity);
    }
    mBuffer = static_cast<char*>(newBuffer);
    mCapacity = newCapacity;
    mEnd = mBuffer + mCapacity;
  }

  if (mBuffer) {
    const auto addr = reinterpret_cast<uintptr_t>(mBuffer);
    char* begin = reinterpret_cast<char*>((addr + 63) & ~uintptr_t{63});
    // Guard the same tiny-buffer case as the constructor: if a failed grow
    // left a buffer too small for a 64-byte-aligned arena, keep the fast-path
    // window empty (mCur == mEnd == mBuffer) instead of letting mBegin exceed
    // mEnd.
    mBegin = (begin < mEnd) ? begin : mBuffer;
    mEnd = (begin < mEnd) ? mEnd : mBuffer;
  } else {
    mBegin = nullptr;
    mEnd = nullptr;
  }
  mCur = mBegin;
  mCacheLineColor = 2;
}

//==============================================================================
size_t FrameAllocator::capacity() const noexcept
{
  return mCapacity;
}

//==============================================================================
size_t FrameAllocator::usableCapacity() const noexcept
{
  if (!mBegin || !mEnd) {
    return 0;
  }

  if (mBegin >= mEnd) {
    return 0;
  }

  const auto usableBytes = static_cast<size_t>(mEnd - mBegin);
  return usableBytes & ~size_t{31};
}

//==============================================================================
size_t FrameAllocator::used() const noexcept
{
  if (!mBegin || !mCur) {
    return 0;
  }

  return static_cast<size_t>(mCur - mBegin);
}

//==============================================================================
size_t FrameAllocator::overflowCount() const noexcept
{
  return mOverflowAllocations.size();
}

//==============================================================================
size_t FrameAllocator::overflowBytes() const noexcept
{
  return mOverflowBytes;
}

} // namespace dart::common
