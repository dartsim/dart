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

#include "dart/common/memory_allocator/object_pool.hpp"

namespace dart::common {

//==============================================================================
template <typename T>
ObjectPool<T>::ObjectPool(size_t capacity, MemoryAllocator& base_allocator)
  : m_capacity(capacity),
    m_base_allocator(base_allocator),
    m_front(reinterpret_cast<T*>(m_base_allocator.allocate_aligned(
        sizeof(T) * capacity, std::max(sizeof(void*), sizeof(T)))))
{
  static_assert(sizeof(T) >= sizeof(void*), "Unsupported type.");

  DART_ASSERT(m_capacity > 0);

  if (!m_front) {
    return;
  }

  T* it = m_front + m_capacity - 1;
  for (; it != m_front; --it) {
    m_free_object_stack.push(it);
  }
  m_free_object_stack.push(it);
}

//==============================================================================
template <typename T>
ObjectPool<T>::~ObjectPool()
{
  std::lock_guard<std::mutex> lock(m_mutex);
  if (m_front) {
    m_base_allocator.deallocate_aligned(m_front);
  }
}

//==============================================================================
template <typename T>
template <typename... Args>
T* ObjectPool<T>::construct(Args&&... args) noexcept
{
  // Allocate new memory for a new object (without calling the constructor)
  T* object = allocate();
  if (!object) {
    return nullptr;
  }

  // Call constructor. Return nullptr if failed.
  try {
    new (object) T(std::forward<Args>(args)...);
  } catch (...) {
    deallocate(object);
    return nullptr;
  }

  return object;
}

//==============================================================================
template <typename T>
void ObjectPool<T>::destroy(T* object) noexcept
{
  if (!object) {
    return;
  }
  object->~T();
  deallocate(object);
}

//==============================================================================
template <typename T>
T* ObjectPool<T>::allocate()
{
  std::lock_guard<std::mutex> lock(m_mutex);

  if (m_free_object_stack.is_empty()) {
    return nullptr;
  }

  T* object = m_free_object_stack.pop();
  DART_ASSERT(!object || common::is_aligned<T>(object));
  return object;
}

//==============================================================================
template <typename T>
void ObjectPool<T>::deallocate(T* pointer)
{
  if (pointer == nullptr) {
    return;
  }

  std::lock_guard<std::mutex> lock(m_mutex);
  m_free_object_stack.push(std::move(pointer));
}

//==============================================================================
template <typename T>
size_t ObjectPool<T>::size() const
{
  return (m_capacity - m_free_object_stack.get_size());
}

//==============================================================================
template <typename T>
size_t ObjectPool<T>::capacity() const
{
  return m_capacity;
}

//==============================================================================
template <typename T>
const T* ObjectPool<T>::get_front() const
{
  return m_front;
}

} // namespace dart::common
