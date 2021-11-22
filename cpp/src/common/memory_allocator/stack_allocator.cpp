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

#include "dart/common/memory_allocator/stack_allocator.hpp"

#include <algorithm>
#include <cstdlib>
#include <limits>

namespace dart::common {

namespace {

//==============================================================================
std::size_t get_padding_with_header(
    std::size_t base_ptr, std::size_t alignment, std::size_t header_size)
{
  if (alignment == 0) {
    return header_size;
  }

  std::size_t padding = get_padding(base_ptr, alignment);
  std::size_t needed_space = header_size;

  if (padding < needed_space) {
    needed_space -= padding;

    if (needed_space % alignment == 0) {
      padding += alignment * (needed_space / alignment);
    } else {
      padding += alignment * (1 + (needed_space / alignment));
    }
  }

  return padding;
}

} // namespace

//==============================================================================
StackAllocator::StackAllocator(
    size_t max_capacity, MemoryAllocator& base_allocator)
  : m_max_capacity(max_capacity),
    m_base_allocator(base_allocator),
    m_start_ptr(base_allocator.allocate(max_capacity)),
    m_offset(0)
{
  if (max_capacity == 0) {
    DART_DEBUG(
        "Allocator with zero max capacity is not able to allocate any memory.");
  }
}

//==============================================================================
StackAllocator::~StackAllocator()
{
  if (m_start_ptr) {
    m_base_allocator.deallocate(m_start_ptr);
  }
}

//==============================================================================
void* StackAllocator::allocate(size_t size) noexcept
{
  if (size == 0) {
    return nullptr;
  }

  if (m_start_ptr == nullptr) {
    return nullptr;
  }

  // Lock the mutex
  std::lock_guard<std::mutex> lock(m_mutex);

  const size_t current_ptr = reinterpret_cast<size_t>(m_start_ptr) + m_offset;

  // Compute padding
  const size_t padding = sizeof(Header);

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

  const std::size_t next_address = current_ptr + padding;
  const std::size_t header_address = next_address - sizeof(Header);
  DART_ASSERT(
      padding <= static_cast<std::size_t>(std::numeric_limits<char>::max()));
  Header* header_ptr = reinterpret_cast<Header*>(header_address);
  header_ptr->padding = static_cast<char>(padding);

  m_offset += padding + size;

  return reinterpret_cast<void*>(next_address);
}

//==============================================================================
void* StackAllocator::allocate_aligned(size_t size, size_t alignment) noexcept
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

  // Lock the mutex
  std::lock_guard<std::mutex> lock(m_mutex);

  const size_t current_ptr = reinterpret_cast<size_t>(m_start_ptr) + m_offset;

  // Compute padding
  size_t padding = 0;
  if (alignment > 0 && m_offset % alignment != 0) {
    padding = get_padding_with_header(current_ptr, alignment, sizeof(Header));
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

  const std::size_t next_address = current_ptr + padding;
  const std::size_t header_address = next_address - sizeof(Header);
  DART_ASSERT(
      padding <= static_cast<std::size_t>(std::numeric_limits<char>::max()));
  Header* header_ptr = reinterpret_cast<Header*>(header_address);
  header_ptr->padding = static_cast<char>(padding);

  m_offset += padding + size;

  return reinterpret_cast<void*>(next_address);
}

//==============================================================================
void StackAllocator::deallocate(void* pointer)
{
  deallocate_aligned(pointer);
}

//==============================================================================
void StackAllocator::deallocate_aligned(void* pointer)
{
  if (pointer == nullptr) {
    return;
  }

  if (m_start_ptr == nullptr) {
    return;
  }

  const std::size_t current_address = reinterpret_cast<std::size_t>(pointer);
  DART_ASSERT(current_address > sizeof(Header));
  const std::size_t header_address = current_address - sizeof(Header);
  const Header* header_ptr = reinterpret_cast<Header*>(header_address);

  m_offset = current_address - static_cast<std::size_t>(header_ptr->padding)
             - reinterpret_cast<std::size_t>(m_start_ptr);
}

//==============================================================================
size_t StackAllocator::get_max_capacity() const
{
  // No need to lock the mutex as m_max_capacity isn't changed once initialized
  return m_max_capacity;
}

//==============================================================================
size_t StackAllocator::get_size() const
{
  std::lock_guard<std::mutex> lock(m_mutex);
  return m_offset;
}

//==============================================================================
const void* StackAllocator::get_begin_address() const
{
  // No need to lock the mutex as m_head isn't changed once initialized
  return m_start_ptr;
}

} // namespace dart::common
