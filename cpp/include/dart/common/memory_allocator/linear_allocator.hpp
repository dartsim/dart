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
#include "dart/common/memory_allocator/memory_allocator.hpp"

namespace dart::common {

/// Memory allocator to keep a point at the first memory address of the memory
/// chunk and move it every time an allocation is done.
template <typename T = void>
class LinearAllocator : public MemoryAllocator<T>
{
public:
  /// Constructor
  LinearAllocator(
      size_t max_capacity,
      MemoryAllocator<T>& base_allocator = get_default_allocator<T>());

  /// Destructor
  ~LinearAllocator() override;

  /// @copydoc MemoryAllocator::allocate
  ///
  /// Complexity is O(1).
  [[nodiscard]] T* allocate(size_t size, size_t alignment = 0) override;

  /// This function does nothing. The allocated memory is released when this
  /// allocator is destructed.
  ///
  /// Complexity is O(1).
  void deallocate(T* pointer, size_t) override;

  /// Returns the maximum capacity of this allocator.
  [[nodiscard]] size_t get_max_capacity() const;

  /// Returns the size of allocated memory.
  ///
  /// The return is the same as get_size_in_bytes() if T is void.
  [[nodiscard]] size_t get_size() const;

  /// Returns the first address of this allocator uses.
  [[nodiscard]] const void* get_begin_address() const;

private:
  /// The maximum size of memory that this allocator can allocate
  const size_t m_max_capacity;

  /// The base allocator to allocate memory chunck
  MemoryAllocator<T>& m_base_allocator;

  /// The memory address of this allocator uses
  void* m_start_ptr = nullptr;

  /// The size of allocated memory or the offset from m_start_ptr.
  size_t m_offset = 0;

  /// Mutex for thread safety
  mutable std::mutex m_mutex;
};

//==============================================================================
template <typename T>
LinearAllocator<T>::LinearAllocator(
    size_t max_capacity, MemoryAllocator<T>& base_allocator)
  : m_max_capacity(max_capacity),
    m_base_allocator(base_allocator),
    m_start_ptr(base_allocator.allocate(max_capacity)),
    m_offset(0)
{
#ifndef NDEBUG
  if (max_capacity == 0) {
    DART_WARN(
        "Allocator with zero max capacity is not able to allocate any memory.");
  }
#endif
}

//==============================================================================
template <typename T>
LinearAllocator<T>::~LinearAllocator()
{
  if (m_start_ptr) {
    this->m_base_allocator.deallocate(m_start_ptr, m_max_capacity);
  }
}

//==============================================================================
template <typename T>
T* LinearAllocator<T>::allocate(size_t size, size_t alignment)
{
  if (size == 0) {
    return nullptr;
  }

  if (m_start_ptr == nullptr) {
    return nullptr;
  }

  if (!this->is_valid_alignment(size, alignment)) {
    return nullptr;
  }

  // Lock the mute
  std::lock_guard<std::mutex> lock(m_mutex);

  const size_t current_ptr = reinterpret_cast<size_t>(m_start_ptr) + m_offset;

  // Compute padding
  size_t padding = 0;
  if (alignment > 0 && m_offset % alignment != 0) {
    padding = get_padding(current_ptr, alignment);
  }

  // Check max capacity
  if (m_offset + padding + size > m_max_capacity) {
    DART_DEBUG(
        "Allocating {} with padding {} exceeds the max capacity {}. Returning "
        "nullptr.",
        size,
        padding,
        m_max_capacity);
    return nullptr;
  }

  // Update offset
  m_offset += padding + size;

  return reinterpret_cast<T*>(current_ptr + padding);
}

//==============================================================================
template <typename T>
void LinearAllocator<T>::deallocate(T* pointer, size_t size)
{
  // LinearAllocator doesn't allow to deallocate memory
  DART_UNUSED(pointer, size);
}

//==============================================================================
template <typename T>
size_t LinearAllocator<T>::get_max_capacity() const
{
  // No need to lock the mutex as m_max_capacity isn't changed once initialized
  return m_max_capacity;
}

//==============================================================================
template <typename T>
size_t LinearAllocator<T>::get_size() const
{
  std::lock_guard<std::mutex> lock(m_mutex);
  return m_offset;
}

//==============================================================================
template <typename T>
const void* LinearAllocator<T>::get_begin_address() const
{
  // No need to lock the mutex as m_head isn't changed once initialized
  return m_start_ptr;
}

} // namespace dart::common
