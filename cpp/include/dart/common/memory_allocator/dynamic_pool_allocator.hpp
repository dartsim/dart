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

namespace dart::common {

// /// Object pool implementation
// template <typename T>
// class PoolAllocator : public MemoryAllocator
// {
// public:
//   /// Constructor
//   PoolAllocator(
//       size_t max_capacity = 1024,
//       MemoryAllocator& base_allocator = get_default_allocator());

//   /// Destructor
//   ~PoolAllocator() override;

//   [[nodiscard]] T* allocate(size_t size, size_t alignment = 0) override;

//   void deallocate(T* pointer, size_t) override;

// private:
//   bool push_back_arena(size_t requested_size);

//   void clear();

//   using MemoryBlockType = detail::MemoryBlockLinkedList;
//   using MemoryBlockStack = detail::ObjectMemoryBlockStack<T>;

//   /// The maximum number of objects this pool can hold.
//   const size_t m_max_capacity;

//   detail::ObjectMemoryBlockStack<void*> m_free_object_stack;

//   /// Number of free items
//   size_t m_num_free_objects = 0;

//   MemoryAllocator& m_base_allocator;

//   std::mutex m_mutex;
// };

} // namespace dart::common

#include "dart/common/memory_allocator/detail/dynamic_pool_allocator_impl.hpp"
