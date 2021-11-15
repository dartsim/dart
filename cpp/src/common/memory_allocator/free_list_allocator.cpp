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

#include "dart/common/memory_allocator/free_list_allocator.hpp"

#include <algorithm>
#include <cstdlib>

namespace dart::common {

////==============================================================================
// FreeListAllocator::FreeListAllocator(
//    MemoryAllocator& base_allocator, size_t initial_allocation)
//  : m_base_allocator(base_allocator),
//    m_allocated_size(0),
//    m_current_block(nullptr),
//    m_free(nullptr)
//{
//  DART_NOT_IMPLEMENTED;
//  allocate_new_block(std::min<size_t>(initial_allocation, 1024u));
//}

////==============================================================================
// void* FreeListAllocator::allocate(size_t size, size_t alignment)
//{
//  DART_NOT_IMPLEMENTED;

//  if (size == 0) {
//    return nullptr;
//  }

//  if (m_current_block == nullptr) {
//    return nullptr;
//  }

//  if (!is_valid_alignment(size, alignment)) {
//    return nullptr;
//  }

//  std::lock_guard<std::mutex> lock(m_mutex);

//  Header* current = m_current_block;
//  DART_ASSERT(m_current_block->previous == nullptr);

//  if (m_free) {
//    DART_ASSERT(!m_free->is_allocated);

//    if (size <= m_free->size) {
//      current = m_free;
//      m_free = nullptr;
//    }
//  }

//  while (current) {
//    if (current->is_allocated && size <= current->size) {
//      // split_memory_block(current, size);
//      break;
//    }
//    current = current->previous;
//  }

//  if (current == nullptr) {
//    if (!allocate_new_block((m_allocated_size + size) * 2)) {
//      return nullptr;
//    }

//    DART_ASSERT(m_free != nullptr);
//    DART_ASSERT(!m_free->is_allocated);

//    current = m_free;
//    DART_ASSERT(current->size >= size);

//    // split_memory_block(current, size);
//  }

//  current->is_allocated = true;

//  if (current->next != nullptr && !current->next->is_allocated) {
//    m_free = current->next;
//  }

//  return static_cast<void*>(
//      reinterpret_cast<unsigned char*>(current) + sizeof(Header));
//}

////==============================================================================
// void FreeListAllocator::deallocate(void* pointer, size_t)
//{
//  DART_NOT_IMPLEMENTED;
//  std::free(pointer);
//}

////==============================================================================
// bool FreeListAllocator::allocate_new_block(size_t size)
//{
//  void* new_block_void_ptr = m_base_allocator.allocate(size + sizeof(Header));
//  if (!new_block_void_ptr) {
//    return false;
//  }

//  Header* new_block
//      = new (new_block_void_ptr) Header(size, nullptr, m_current_block,
//      false);

//  if (m_current_block) {
//    m_current_block->previous = new_block;
//  }

//  m_current_block = new_block;
//  m_free = m_current_block;
//  m_allocated_size += size;

//  return true;
//}

} // namespace dart::common
