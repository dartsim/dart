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

#include <mutex>

#include "dart/common/memory_allocator/default_allocator.hpp"
#include "dart/common/memory_allocator/detail/memory_block.hpp"
#include "dart/common/memory_allocator/memory_allocator.hpp"

namespace dart::common {

/// Object pool implementation
template <typename T>
class ObjectPool
{
public:
  /// Constructor
  explicit ObjectPool(
      size_t capacity = 1024,
      MemoryAllocator& base_allocator = get_default_allocator());

  /// Destructor
  virtual ~ObjectPool();

  /// Creates an object.
  template <typename... Args>
  [[nodiscard]] T* construct(Args&&... args) noexcept;

  /// Destroys an object created by this allocator.
  void destroy(T* object) noexcept;

  [[nodiscard]] size_t size() const;

  [[nodiscard]] size_t capacity() const;

  [[nodiscard]] const T* get_front() const;

#ifndef NDEBUG
  const auto& get_free_object_stack() const
  {
    return m_free_object_stack;
  }

  void print_free_object_stack() const
  {
    get_free_object_stack().print_list();
  }
#endif

private:
  [[nodiscard]] T* allocate();

  void deallocate(T* pointer);

  using MemoryBlockStack = detail::ObjectMemoryStack<T>;

  /// The maximum number of objects this pool can hold.
  const size_t m_capacity;

  MemoryAllocator& m_base_allocator;

  T* const m_front;

  MemoryBlockStack m_free_object_stack;

  std::mutex m_mutex;
};

// template <typename T>
// class DynamicObjectPool
//{
// public:
//  DynamicObjectPool(
//      size_t initial_capacity,
//      size_t max_capacity = 0,
//      MemoryAllocator& base_allocator = get_default_allocator())
//    : m_base_allocator(base_allocator), m_meta_pool(base_allocator)
//  {

//  }

//  MemoryAllocator& m_base_allocator;
//  ObjectPool<ObjectPool<T>> m_meta_pool;
//  std::vector<std::pair<size_t, ObjectPool<T>*>> m_pool_list;
//};

} // namespace dart::common

#include "dart/common/memory_allocator/detail/object_pool_impl.hpp"
