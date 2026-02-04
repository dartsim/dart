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

#include "dart/common/frame_allocator.hpp"

#include <algorithm>

#include <cstdint>

namespace dart::common {

//==============================================================================
FrameAllocator::FrameAllocator(
    MemoryAllocator& baseAllocator, size_t initialCapacity)
  : mBaseAllocator(baseAllocator),
    mBuffer(static_cast<char*>(baseAllocator.allocate(initialCapacity))),
    mCapacity(mBuffer ? initialCapacity : 0),
    mOffset(0)
{
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
void* FrameAllocator::allocate(size_t bytes) noexcept
{
  return allocateAligned(bytes, 32);
}

//==============================================================================
void FrameAllocator::deallocate(void* /*pointer*/, size_t /*bytes*/)
{
  // Arena semantics: memory is freed in bulk on reset()
}

//==============================================================================
void FrameAllocator::print(std::ostream& os, int indent) const
{
  const std::string spaces(indent, ' ');
  os << spaces << "[FrameAllocator] capacity: " << mCapacity
     << " used: " << mOffset << " overflow: " << mOverflowAllocations.size()
     << "\n";
}

//==============================================================================
void* FrameAllocator::allocateAligned(size_t bytes, size_t alignment) noexcept
{
  if (bytes == 0 || alignment == 0) {
    return nullptr;
  }

  // Align based on the actual memory address, not just the offset, since the
  // buffer base may not be aligned to the requested alignment.
  const auto currentAddr = reinterpret_cast<uintptr_t>(mBuffer) + mOffset;
  const auto alignedAddr = (currentAddr + alignment - 1) & ~(alignment - 1);
  const auto alignedOffset
      = static_cast<size_t>(alignedAddr - reinterpret_cast<uintptr_t>(mBuffer));

  if (alignedOffset + bytes <= mCapacity) {
    mOffset = alignedOffset + bytes;
    return mBuffer + alignedOffset;
  }

  // Over-allocate to guarantee alignment within the raw block
  const size_t totalSize = bytes + alignment;
  void* raw = mBaseAllocator.allocate(totalSize);
  if (!raw) {
    return nullptr;
  }

  const auto rawAddr = reinterpret_cast<uintptr_t>(raw);
  const auto overflowAligned = (rawAddr + alignment - 1) & ~(alignment - 1);
  void* pointer = reinterpret_cast<void*>(overflowAligned);

  mOverflowAllocations.push_back({raw, totalSize});
  return pointer;
}

//==============================================================================
void FrameAllocator::reset() noexcept
{
  if (!mOverflowAllocations.empty()) {
    size_t totalOverflow = 0;
    for (const auto& entry : mOverflowAllocations) {
      mBaseAllocator.deallocate(entry.ptr, entry.allocatedSize);
      totalOverflow += entry.allocatedSize;
    }
    mOverflowAllocations.clear();

    const size_t needed = mOffset + totalOverflow;
    const size_t newCapacity = std::max(mCapacity * 2, needed + 256);
    void* newBuffer = mBaseAllocator.allocate(newCapacity);
    if (newBuffer) {
      if (mBuffer) {
        mBaseAllocator.deallocate(mBuffer, mCapacity);
      }
      mBuffer = static_cast<char*>(newBuffer);
      mCapacity = newCapacity;
    }
  }

  mOffset = 0;
}

//==============================================================================
size_t FrameAllocator::capacity() const noexcept
{
  return mCapacity;
}

//==============================================================================
size_t FrameAllocator::used() const noexcept
{
  return mOffset;
}

//==============================================================================
size_t FrameAllocator::overflowCount() const noexcept
{
  return mOverflowAllocations.size();
}

} // namespace dart::common
