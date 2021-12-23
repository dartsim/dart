/*
 * Copyright (c) 2011-2021, The DART development contributors
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

#include "dart/common/CAllocator.hpp"

#include "dart/common/Console.hpp"
#include "dart/common/Logging.hpp"

namespace dart::common {

//==============================================================================
CAllocator::CAllocator() noexcept
{
  // Do nothing
}

//==============================================================================
CAllocator::~CAllocator()
{
#ifndef NDEBUG
  std::lock_guard<std::mutex> lock(mMutex);
  if (!mMapPointerToSize.empty())
  {
    size_t totalSize = 0;
    for (auto it : mMapPointerToSize)
    {
      void* pointer = it.first;
      size_t size = it.second;
      totalSize += size;
      dterr << "Found memory leak of " << size << " bytes at " << pointer
            << "\n";
      // TODO(JS): Change to DART_FATAL once the issue of calling spdlog in
      // destructor is resolved.
    }
    dterr << "Found potential memory leak of total " << totalSize
          << " bytes!\n";
    // TODO(JS): Change to DART_FATAL once the issue of calling spdlog in
    // destructor is resolved.
  }
#endif
}

//==============================================================================
void* CAllocator::allocate(size_t size) noexcept
{
  if (size == 0)
  {
    return nullptr;
  }

  DART_TRACE("Allocated {} bytes.", size);
#ifndef NDEBUG
  std::lock_guard<std::mutex> lock(mMutex);
  auto newPtr = std::malloc(size);
  if (newPtr)
  {
    mSize += size;
    mPeak = std::max(mPeak, mSize);
    mMapPointerToSize[newPtr] = size;
  }
  return newPtr;
#else
  return std::malloc(size);
#endif
}

//==============================================================================
void CAllocator::deallocate(void* pointer, size_t size)
{
  (void)size;
#ifndef NDEBUG
  std::lock_guard<std::mutex> lock(mMutex);
  auto it = mMapPointerToSize.find(pointer);
  if (it != mMapPointerToSize.end())
  {
    auto allocatedSize = it->second;
    if (size != allocatedSize)
    {
      DART_FATAL(
          "Attempting to deallocate memory at {} of {} bytes that is different "
          "from the allocated size {}, which is a critical bug. Deallocating "
          "{} bytes.",
          pointer,
          size,
          allocatedSize,
          allocatedSize);
      size = allocatedSize;
    }
    mSize -= size;
    mMapPointerToSize.erase(it);
    DART_TRACE("Deallocated {} bytes.", size);
  }
  else
  {
    DART_FATAL(
        "Cannot deallocate memory {} that is not allocated by this allocator!",
        pointer);
    return;
  }
#else
  DART_TRACE("Deallocated.");
#endif
  std::free(pointer);
}

//==============================================================================
void CAllocator::print(std::ostream& os, int indent) const
{
  if (indent == 0)
  {
    os << "[CAllocator]\n";
  }
  const std::string spaces(indent, ' ');
  if (indent != 0)
  {
    os << spaces << "type: " << getType() << "\n";
  }
#ifndef NDEBUG
  std::lock_guard<std::mutex> lock(mMutex);
  os << spaces << "size_in_bytes: " << mSize << "\n";
  os << spaces << "peak: " << mPeak << "\n";
#endif
}

} // namespace dart::common
