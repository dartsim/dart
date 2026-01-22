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

#include "dart/common/memory_manager.hpp"

#include "dart/common/macros.hpp"

namespace dart::common {

//==============================================================================
MemoryManager& MemoryManager::GetDefault()
{
  static MemoryManager defaultMemoryManager(MemoryAllocator::GetDefault());
  return defaultMemoryManager;
}

//==============================================================================
MemoryManager::MemoryManager(MemoryAllocator& baseAllocator)
  : mBaseAllocator(baseAllocator)
{
#if !defined(NDEBUG)
  mUseDebugAllocators = true;
#endif

  if (mUseDebugAllocators) {
    mFreeListAllocatorWithDebug
        = std::make_unique<FreeListAllocator::Debug>(mBaseAllocator);
    mPoolAllocatorWithDebug = std::make_unique<PoolAllocator::Debug>(
        mFreeListAllocatorWithDebug->getInternalAllocator());
  } else {
    mFreeListAllocator = std::make_unique<FreeListAllocator>(mBaseAllocator);
    mPoolAllocator = std::make_unique<PoolAllocator>(*mFreeListAllocator);
  }
}

//==============================================================================
MemoryManager::~MemoryManager()
{
  // Do nothing
}

//==============================================================================
MemoryAllocator& MemoryManager::getBaseAllocator()
{
  return mBaseAllocator;
}

//==============================================================================
FreeListAllocator& MemoryManager::getFreeListAllocator()
{
  if (mUseDebugAllocators) {
    DART_ASSERT(mFreeListAllocatorWithDebug != nullptr);
    return mFreeListAllocatorWithDebug->getInternalAllocator();
  }

  DART_ASSERT(mFreeListAllocator != nullptr);
  return *mFreeListAllocator;
}

//==============================================================================
PoolAllocator& MemoryManager::getPoolAllocator()
{
  if (mUseDebugAllocators) {
    DART_ASSERT(mPoolAllocatorWithDebug != nullptr);
    return mPoolAllocatorWithDebug->getInternalAllocator();
  }

  DART_ASSERT(mPoolAllocator != nullptr);
  return *mPoolAllocator;
}

//==============================================================================
void* MemoryManager::allocate(Type type, size_t bytes)
{
  switch (type) {
    case Type::Base:
      return mBaseAllocator.allocate(bytes);
    case Type::Free:
      if (mUseDebugAllocators) {
        DART_ASSERT(mFreeListAllocatorWithDebug != nullptr);
        return mFreeListAllocatorWithDebug->allocate(bytes);
      }

      DART_ASSERT(mFreeListAllocator != nullptr);
      return mFreeListAllocator->allocate(bytes);
    case Type::Pool:
      if (mUseDebugAllocators) {
        DART_ASSERT(mPoolAllocatorWithDebug != nullptr);
        return mPoolAllocatorWithDebug->allocate(bytes);
      }

      DART_ASSERT(mPoolAllocator != nullptr);
      return mPoolAllocator->allocate(bytes);
  }
  return nullptr;
}

//==============================================================================
void* MemoryManager::allocateUsingFree(size_t bytes)
{
  return allocate(Type::Free, bytes);
}

//==============================================================================
void* MemoryManager::allocateUsingPool(size_t bytes)
{
  return allocate(Type::Pool, bytes);
}

//==============================================================================
void MemoryManager::deallocate(Type type, void* pointer, size_t bytes)
{
  switch (type) {
    case Type::Base:
      mBaseAllocator.deallocate(pointer, bytes);
      break;
    case Type::Free:
      if (mUseDebugAllocators) {
        DART_ASSERT(mFreeListAllocatorWithDebug != nullptr);
        mFreeListAllocatorWithDebug->deallocate(pointer, bytes);
        break;
      }

      DART_ASSERT(mFreeListAllocator != nullptr);
      mFreeListAllocator->deallocate(pointer, bytes);
      break;
    case Type::Pool:
      if (mUseDebugAllocators) {
        DART_ASSERT(mPoolAllocatorWithDebug != nullptr);
        mPoolAllocatorWithDebug->deallocate(pointer, bytes);
        break;
      }

      DART_ASSERT(mPoolAllocator != nullptr);
      mPoolAllocator->deallocate(pointer, bytes);
      break;
  }
}

//==============================================================================
void MemoryManager::deallocateUsingFree(void* pointer, size_t bytes)
{
  deallocate(Type::Free, pointer, bytes);
}

//==============================================================================
void MemoryManager::deallocateUsingPool(void* pointer, size_t bytes)
{
  deallocate(Type::Pool, pointer, bytes);
}

//==============================================================================
bool MemoryManager::hasAllocated(void* pointer, size_t size) const noexcept
{
  if (!mUseDebugAllocators) {
    return false;
  }

  if (mFreeListAllocatorWithDebug != nullptr
      && mFreeListAllocatorWithDebug->hasAllocated(pointer, size)) {
    return true;
  }

  if (mPoolAllocatorWithDebug != nullptr
      && mPoolAllocatorWithDebug->hasAllocated(pointer, size)) {
    return true;
  }

  return false;
}

//==============================================================================
void MemoryManager::print(std::ostream& os, int indent) const
{
  if (indent == 0) {
    os << "[MemoryManager]\n";
  }
  const std::string spaces(indent, ' ');
  os << spaces << "free_allocator:\n";
  if (mUseDebugAllocators) {
    DART_ASSERT(mFreeListAllocatorWithDebug != nullptr);
    mFreeListAllocatorWithDebug->print(os, indent + 2);
  } else {
    DART_ASSERT(mFreeListAllocator != nullptr);
    mFreeListAllocator->print(os, indent + 2);
  }
  os << spaces << "pool_allocator:\n";
  if (mUseDebugAllocators) {
    DART_ASSERT(mPoolAllocatorWithDebug != nullptr);
    mPoolAllocatorWithDebug->print(os, indent + 2);
  } else {
    DART_ASSERT(mPoolAllocator != nullptr);
    mPoolAllocator->print(os, indent + 2);
  }
  os << spaces << "base_allocator:\n";
  mBaseAllocator.print(os, indent + 2);
}

//==============================================================================
std::ostream& operator<<(std::ostream& os, const MemoryManager& memoryManager)
{
  memoryManager.print(os);
  return os;
}

} // namespace dart::common
