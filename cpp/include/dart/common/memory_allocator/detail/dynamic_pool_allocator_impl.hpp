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

#pragma once

#include <algorithm>
#include <cstdlib>

#include "dart/common/memory_allocator/dynamic_pool_allocator.hpp"

namespace dart::common {

// //==============================================================================
// template <typename T>
// PoolAllocator<T>::PoolAllocator(
//     size_t max_capacity, MemoryAllocator& base_allocator)
//   : m_max_capacity(max_capacity), m_base_allocator(base_allocator)
// {
//   static_assert(sizeof(T) < sizeof(void*), "Unsupported type.");
// }

// //==============================================================================
// template <typename T>
// PoolAllocator<T>::~PoolAllocator()
// {
//   DART_NOT_IMPLEMENTED;
// }

// //==============================================================================
// template <typename T>
// void* PoolAllocator<T>::allocate(size_t size, size_t alignment)
// {
//   if (size == 0) {
//     return nullptr;
//   }

//   if (alignment == 0) {
//     return std::malloc(size);
//   }

//   if (!is_valid_alignment(size, alignment)) {
//     return nullptr;
//   }

//   return common::aligned_alloc(alignment, size);
// }

// //==============================================================================
// template <typename T>
// void PoolAllocator<T>::deallocate(void* pointer, size_t size)
// {
//   DART_UNUSED(size);
//   std::free(pointer);
// }

// //==============================================================================
// template <typename T>
// bool PoolAllocator<T>::push_back_arena(size_t requested_size)
// {
//   DART_ASSERT(requested_size != 0);

//   // Update new arena size based on the max capacity and the current capacity
//   if (m_max_capacity > 0) {
//     DART_ASSERT(m_capacity <= m_max_capacity);
//     const size_t rest = m_max_capacity - m_capacity;
//     if (rest < requested_size) {
//       return false;
//     }
//   }

//   // Create new memory block
//   MemoryBlockType* new_block = m_base_allocator.construct<MemoryBlockType>(
//       requested_size, m_base_allocator);
//   if (!new_block) {
//     DART_DEBUG("Failed to allocate new memory block.");
//     return false;
//   } else if (!new_block->is_valid()) {
//     m_base_allocator.destroy(new_block);
//     DART_DEBUG("Failed to allocate new memory block.");
//     return false;
//   }

//   // Update pointer to the last memory block
//   if (m_block_back != nullptr) {
//     m_block_back->set_next_arena(new_block);
//   }
//   m_block_back = new_block;

//   // Update the pointer to the first block
//   if (m_block_front == nullptr) {
//     m_block_front = new_block;
//   }

//   // Add all the free object pointers of the new arena to the free object
//   stack
//   // TODO(JS): parallelize?
//   const void* front = new_block->get_front();
//   void* it = new_block->get_back();
//   //  for (; it != first_item; --it) {
//   //    m_free_object_stack.push(it);
//   //  }
//   //  m_free_object_stack.push(it);
//   DART_NOT_IMPLEMENTED;

//   m_num_free_objects += requested_size;
//   m_capacity += requested_size;

//   return true;
// }

// //==============================================================================
// template <typename T>
// void PoolAllocator<T>::clear()
// {
//   DART_NOT_IMPLEMENTED;
// }

} // namespace dart::common
