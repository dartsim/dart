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

#ifndef DART_COMMON_DETAIL_MEMORYALLOCATORDEBUGGER_IMPL_HPP_
#define DART_COMMON_DETAIL_MEMORYALLOCATORDEBUGGER_IMPL_HPP_

#include "dart/common/Console.hpp"
#include "dart/common/Logging.hpp"
#include "dart/common/MemoryAllocatorDebugger.hpp"

namespace dart::common {

//==============================================================================
template <typename T>
template <typename... Args>
MemoryAllocatorDebugger<T>::MemoryAllocatorDebugger(Args&&... args)
  : mInternalAllocator(std::forward<Args>(args)...)
{
  // Do nothing
}

//==============================================================================
template <typename T>
MemoryAllocatorDebugger<T>::~MemoryAllocatorDebugger()
{
  // Lock the mutex
  std::lock_guard<std::mutex> lock(mMutex);

  if (!mMapPointerToSize.empty())
  {
    size_t totalSize = 0;
    for (auto it : mMapPointerToSize)
    {
      void* pointer = it.first;
      size_t size = it.second;
      totalSize += size;
      dtdbg << "Found potential memory leak at " << pointer << " (" << size
            << " bytes).\n";
      // TODO(JS): Change to DART_FATAL once the issue of calling spdlog in
      // destructor is resolved.
    }

    dtdbg << "Found potential memory leak of total " << totalSize
          << " bytes. The internal allocator will try to forcefully "
          << "deallocate it but it's is not guaranteed.\n";
    // TODO(JS): Change to DART_FATAL once the issue of calling spdlog in
    // destructor is resolved.
  }
}

//==============================================================================
template <typename T>
const std::string& MemoryAllocatorDebugger<T>::getStaticType()
{
  static const std::string type
      = "MemoryAllocatorDebugger<" + T::getStaticType() + ">";
  return type;
}

//==============================================================================
template <typename T>
const std::string& MemoryAllocatorDebugger<T>::getType() const
{
  return getStaticType();
}

//==============================================================================
template <typename T>
void* MemoryAllocatorDebugger<T>::allocate(size_t bytes) noexcept
{
  void* newPtr = mInternalAllocator.allocate(bytes);

  if (newPtr)
  {
    std::lock_guard<std::mutex> lock(mMutex);
    mSize += bytes;
    mPeak = std::max(mPeak, mSize);
    mMapPointerToSize[newPtr] = bytes;
  }

  return newPtr;
}

//==============================================================================
template <typename T>
void MemoryAllocatorDebugger<T>::deallocate(void* pointer, size_t bytes)
{
  std::lock_guard<std::mutex> lock(mMutex);

  auto it = mMapPointerToSize.find(pointer);
  if (it == mMapPointerToSize.end())
  {
    DART_DEBUG(
        "Cannot deallocate memory {} not allocated by this allocator.",
        pointer);
    return;
  }

  auto allocatedSize = it->second;
  if (bytes != allocatedSize)
  {
    DART_DEBUG(
        "Cannot deallocate memory at {} of {} bytes that is different from the "
        "allocated size {}, which is a critical bug.",
        pointer,
        bytes,
        allocatedSize);
    return;
  }

  mInternalAllocator.deallocate(pointer, bytes);
  mMapPointerToSize.erase(it);
  mSize -= bytes;
}

//==============================================================================
template <typename T>
bool MemoryAllocatorDebugger<T>::isEmpty() const
{
  std::lock_guard<std::mutex> lock(mMutex);
  return mMapPointerToSize.empty();
}

//==============================================================================
template <typename T>
bool MemoryAllocatorDebugger<T>::hasAllocated(void* pointer, size_t size) const
{
  std::lock_guard<std::mutex> lock(mMutex);

  const auto it = mMapPointerToSize.find(pointer);
  if (it == mMapPointerToSize.end())
  {
    return false;
  }

  const auto& allocatedSize = it->second;
  if (size != allocatedSize)
  {
    return false;
  }

  return true;
}

//==============================================================================
template <typename T>
const T& MemoryAllocatorDebugger<T>::getInternalAllocator() const
{
  return mInternalAllocator;
}

//==============================================================================
template <typename T>
T& MemoryAllocatorDebugger<T>::getInternalAllocator()
{
  return mInternalAllocator;
}

//==============================================================================
template <typename T>
void MemoryAllocatorDebugger<T>::print(std::ostream& os, int indent) const
{
  if (indent == 0)
  {
    os << "[" << getType() << "]\n";
  }
  const std::string spaces(indent, ' ');
  if (indent != 0)
  {
    os << spaces << "type: " << getType() << "\n";
  }
  std::lock_guard<std::mutex> lock(mMutex);
  os << spaces << "size_in_bytes: " << mSize << "\n";
  os << spaces << "peak: " << mPeak << "\n";
  os << spaces << "internal_allocator:\n";
  mInternalAllocator.print(os, indent + 2);
}

} // namespace dart::common

#endif // DART_COMMON_DETAIL_MEMORYALLOCATORDEBUGGER_IMPL_HPP_
