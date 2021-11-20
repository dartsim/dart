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

#include "dart/common/memory_allocator/frame_allocator.hpp"

#include <algorithm>
#include <cstdlib>
#include <limits>

namespace dart::common {

//==============================================================================
FrameAllocator::FrameAllocator(MemoryAllocator& base_allocator)
  : m_base_allocator(base_allocator),
    m_total_size_bytes(m_init_single_frame_allocator_bytes),
    m_offset(0),
    m_frames_should_shrink(0),
    m_should_allocate_more(false)
{
  m_start_pointer = m_base_allocator.allocate(m_total_size_bytes);
}

//==============================================================================
FrameAllocator::~FrameAllocator()
{
  if (m_start_pointer) {
    m_base_allocator.deallocate(m_start_pointer, m_total_size_bytes);
  }
}

//==============================================================================
void* FrameAllocator::allocate(size_t size) noexcept
{
  // Lock the mutex
  std::lock_guard<std::mutex> lock(m_mutex);

  if (m_offset + size > m_total_size_bytes) {
    m_should_allocate_more = true;
    return m_base_allocator.allocate(size);
  }

  size_t next_available_memory
      = reinterpret_cast<size_t>(m_start_pointer) + m_offset;

  m_offset += size;

  return reinterpret_cast<void*>(next_available_memory);
}

//==============================================================================
void* FrameAllocator::allocate_aligned(size_t size, size_t alignment) noexcept
{
  // Lock the mutex
  std::lock_guard<std::mutex> lock(m_mutex);

  if (m_offset + size > m_total_size_bytes) {
    m_should_allocate_more = true;
    return m_base_allocator.allocate_aligned(size, alignment);
  }

  size_t next_available_memory
      = reinterpret_cast<size_t>(m_start_pointer) + m_offset;

  m_offset += size;

  return reinterpret_cast<void*>(next_available_memory);
}

//==============================================================================
void FrameAllocator::deallocate(void* pointer, size_t size)
{
  // Lock the mutex
  std::lock_guard<std::mutex> lock(m_mutex);

  size_t p = reinterpret_cast<size_t>(pointer);
  size_t s = reinterpret_cast<size_t>(m_start_pointer);

  if (p < s || p > s + m_total_size_bytes) {
    m_base_allocator.deallocate(pointer, size);
  }
}

//==============================================================================
void FrameAllocator::deallocate_aligned(void* pointer, size_t size)
{
  // Lock the mutex
  std::lock_guard<std::mutex> lock(m_mutex);

  size_t p = reinterpret_cast<size_t>(pointer);
  size_t s = reinterpret_cast<size_t>(m_start_pointer);

  if (p < s || p > s + m_total_size_bytes) {
    m_base_allocator.deallocate_aligned(pointer, size);
  }
}

//==============================================================================
void FrameAllocator::reset()
{
  // Lock the mutex
  std::lock_guard<std::mutex> lock(m_mutex);

  if (m_offset < m_total_size_bytes / 2) {
    m_frames_should_shrink++;

    if (m_frames_should_shrink > frames_until_shrink) {
      m_base_allocator.deallocate(m_start_pointer, m_total_size_bytes);
      m_total_size_bytes /= 2;
      m_total_size_bytes = std::max<size_t>(m_total_size_bytes, 1);
      m_start_pointer = m_base_allocator.allocate(m_total_size_bytes);
      m_frames_should_shrink = 0;
    }
  } else {
    m_frames_should_shrink = 0;
  }

  if (m_should_allocate_more) {
    m_base_allocator.deallocate(m_start_pointer, m_total_size_bytes);
    m_total_size_bytes *= 2;
    m_start_pointer = m_base_allocator.allocate(m_total_size_bytes);
    m_should_allocate_more = false;
    m_frames_should_shrink = 0;
  }

  m_offset = 0;
}

//==============================================================================
void FrameAllocator::print(std::ostream& os, int indent) const
{
  // Lock the mutex
  std::lock_guard<std::mutex> lock(m_mutex);

  if (indent == 0) {
    os << "[FrameAllocator]\n";
  }
  const std::string spaces(indent, ' ');
  if (indent != 0) {
    os << spaces << "type: " << get_type() << "\n";
  }
  os << spaces << "first_address: " << m_start_pointer << "\n";
  os << spaces << "size_in_bytes: " << m_offset << "\n";
  os << spaces << "base_allocator:\n";
  m_base_allocator.print(os, indent + 2);
}

} // namespace dart::common
