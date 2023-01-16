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

#include "dart/simulation/MemoryManager.hpp"

#ifndef NDEBUG // debug
  #include "dart/common/Logging.hpp"
#endif

namespace dart::simulation {

//==============================================================================
MemoryManager& MemoryManager::GetDefault()
{
  static MemoryManager defaultMemoryManager(common::Allocator::GetDefault());
  return defaultMemoryManager;
}

//==============================================================================
MemoryManager::MemoryManager(common::Allocator& baseAllocator)
  : mBaseAllocator(baseAllocator),
    mAllocatorFreeList(mBaseAllocator),
#ifdef NDEBUG
    mAllocatorPool(mAllocatorFreeList)
#else
    mAllocatorPool(mAllocatorFreeList.getInternalAllocator())
#endif
{
  // Do nothing
}

//==============================================================================
MemoryManager::~MemoryManager()
{
  // Do nothing
}

//==============================================================================
common::Allocator& MemoryManager::getBaseAllocator()
{
  return mBaseAllocator;
}

//==============================================================================
common::AllocatorFreeList& MemoryManager::getAllocatorFreeList()
{
#ifdef NDEBUG
  return mAllocatorFreeList;
#else
  return mAllocatorFreeList.getInternalAllocator();
#endif
}

//==============================================================================
common::AllocatorPool& MemoryManager::getAllocatorPool()
{
#ifdef NDEBUG
  return mAllocatorPool;
#else
  return mAllocatorPool.getInternalAllocator();
#endif
}

//==============================================================================
void* MemoryManager::allocate(Type type, size_t bytes)
{
  switch (type) {
    case Type::Base:
      return mBaseAllocator.allocate(bytes);
    case Type::Free:
      return mAllocatorFreeList.allocate(bytes);
    case Type::Pool:
      return mAllocatorPool.allocate(bytes);
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
      mAllocatorFreeList.deallocate(pointer, bytes);
      break;
    case Type::Pool:
      mAllocatorPool.deallocate(pointer, bytes);
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

#ifndef NDEBUG
//==============================================================================
bool MemoryManager::hasAllocated(void* pointer, size_t size) const noexcept
{
  if (mAllocatorFreeList.hasAllocated(pointer, size))
    return true;

  if (mAllocatorPool.hasAllocated(pointer, size))
    return true;

  return false;
}
#endif

//==============================================================================
void MemoryManager::print(std::ostream& os, int indent) const
{
  if (indent == 0) {
    os << "[dart::simulation::MemoryManager]\n";
  }
  const std::string spaces(indent, ' ');
  os << spaces << "free_allocator:\n";
  mAllocatorFreeList.print(os, indent + 2);
  os << spaces << "pool_allocator:\n";
  mAllocatorPool.print(os, indent + 2);
  os << spaces << "base_allocator:\n";
  mBaseAllocator.print(os, indent + 2);
}

//==============================================================================
std::ostream& operator<<(std::ostream& os, const MemoryManager& memoryManager)
{
  memoryManager.print(os);
  return os;
}

} // namespace dart::simulation
