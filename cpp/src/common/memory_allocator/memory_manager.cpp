/*
 * Copyright (c) 2011-2021, The DART development contributors:
 * https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "dart/common/memory_allocator/memory_manager.hpp"

#include <algorithm>
#include <cstdlib>
#include <limits>

namespace dart::common {

//==============================================================================
MemoryManager& MemoryManager::GetDefault()
{
  static MemoryManager default_memory_manager(MemoryAllocator::GetDefault());
  return default_memory_manager;
}

//==============================================================================
MemoryManager::MemoryManager(MemoryAllocator& base_allocator)
  : m_base_allocator(base_allocator),
    m_free_list_allocator(m_base_allocator),
    m_pool_allocator(m_free_list_allocator),
    m_frame_allocator(m_free_list_allocator)
{
  // Do nothing
}

//==============================================================================
MemoryManager::~MemoryManager()
{
  // Do nothing
}

//==============================================================================
MemoryAllocator& MemoryManager::get_mutable_base_allocator()
{
  return m_base_allocator;
}

//==============================================================================
FreeListAllocator& MemoryManager::get_mutable_free_list_allocator()
{
  return m_free_list_allocator;
}

//==============================================================================
PoolAllocator& MemoryManager::get_mutable_pool_allocator()
{
  return m_pool_allocator;
}

//==============================================================================
FrameAllocator& MemoryManager::get_mutable_frame_allocator()
{
  return m_frame_allocator;
}

//==============================================================================
void* MemoryManager::allocate(Type type, size_t size)
{
  switch (type) {
    case Type::Base:
      return m_base_allocator.allocate(size);
    case Type::Free:
      return m_free_list_allocator.allocate(size);
    case Type::Pool:
      return m_pool_allocator.allocate(size);
    case Type::Frame:
      return m_frame_allocator.allocate(size);
  }

  return nullptr;
}

//==============================================================================
void MemoryManager::deallocate(Type type, void* pointer, size_t size)
{
  switch (type) {
    case Type::Base:
      m_base_allocator.deallocate(pointer, size);
      break;
    case Type::Free:
      m_free_list_allocator.deallocate(pointer, size);
      break;
    case Type::Pool:
      m_pool_allocator.deallocate(pointer, size);
      break;
    case Type::Frame:
      m_frame_allocator.deallocate(pointer, size);
      break;
  }
}

//==============================================================================
void MemoryManager::reset_frame_allocator()
{
  m_frame_allocator.reset();
}

//==============================================================================
void MemoryManager::print(std::ostream& os, int indent) const
{
  if (indent == 0) {
    os << "[MemoryManager]\n";
  }
  const std::string spaces(indent, ' ');
  os << spaces << "free_allocator:\n";
  m_free_list_allocator.print(os, indent + 2);
  os << spaces << "pool_allocator:\n";
  m_pool_allocator.print(os, indent + 2);
  os << spaces << "frame_allocator:\n";
  m_frame_allocator.print(os, indent + 2);
  os << spaces << "base_allocator:\n";
  m_base_allocator.print(os, indent + 2);
}

//==============================================================================
std::ostream& operator<<(std::ostream& os, const MemoryManager& memory_manager)
{
  memory_manager.print(os);
  return os;
}

} // namespace dart::common
