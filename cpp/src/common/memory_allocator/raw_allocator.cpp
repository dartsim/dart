/*
 * Copyright (c) 2011-2022, The DART development contributors:
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

#include "dart/common/memory_allocator/raw_allocator.hpp"

namespace dart::common {

//==============================================================================
RawAllocator::RawAllocator() noexcept
{
  // Do nothing
}

//==============================================================================
RawAllocator::~RawAllocator()
{
#ifndef NDEBUG
  std::lock_guard<std::mutex> lock(m_mutex);
  if (!m_map_pointer_to_size.empty()) {
    size_t total_size = 0;
    for (auto it : m_map_pointer_to_size) {
      void* pointer = it.first;
      size_t size = it.second;
      total_size += size;
      DART_FATAL("Found memory leak of {} bytes at {}!", size, pointer);
    }
    DART_FATAL("Found potential memory leak of total {} bytes!", total_size);
  }
#endif
}

//==============================================================================
void* RawAllocator::allocate(size_t size) noexcept
{
  return allocate_aligned(size, 0);
}

//==============================================================================
void* RawAllocator::allocate_aligned(size_t size, size_t alignment) noexcept
{
  if (size == 0) {
    return nullptr;
  }

  if (alignment == 0) {
    DART_TRACE("Allocated {} bytes.", size);
#ifndef NDEBUG
    std::lock_guard<std::mutex> lock(m_mutex);
    auto new_ptr = std::malloc(size);
    if (new_ptr) {
      m_size += size;
      m_peak = std::max(m_peak, m_size);
      m_map_pointer_to_size[new_ptr] = size;
    }
    return new_ptr;
#else
    return std::malloc(size);
#endif
  }

  if (!is_valid_alignment(size, alignment)) {
    return nullptr;
  }

  DART_TRACE("Allocated {} bytes.", size);
#ifndef NDEBUG
  std::lock_guard<std::mutex> lock(m_mutex);
  auto new_ptr = common::aligned_alloc(alignment, size);
  if (new_ptr) {
    m_size += size;
    m_peak = std::max(m_peak, m_size);
    m_map_pointer_to_size[new_ptr] = size;
  }
  return new_ptr;
#else
  return common::aligned_alloc(alignment, size);
#endif
}

//==============================================================================
void RawAllocator::deallocate(void* pointer, size_t size)
{
  DART_UNUSED(size);
#ifndef NDEBUG
  std::lock_guard<std::mutex> lock(m_mutex);
  auto it = m_map_pointer_to_size.find(pointer);
  if (it != m_map_pointer_to_size.end()) {
    auto allocated_size = it->second;
    if (size != allocated_size) {
      DART_FATAL(
          "Cannot deallocated memory {} because the deallocating size {} is "
          "different from the allocated size {}.",
          pointer,
          size,
          allocated_size);
      return;
    }
    m_size -= size;
    m_map_pointer_to_size.erase(it);
    DART_TRACE("Deallocated {} bytes.", size);
  } else {
    DART_FATAL(
        "Cannot deallocate memory {} that is not allocated by this allocator!",
        pointer);
    return;
  }
#else
  DART_TRACE("Deallocated.");
#endif
  std::free(pointer);
}

//==============================================================================
void RawAllocator::deallocate_aligned(void* pointer, size_t size)
{
  DART_UNUSED(size);
#ifndef NDEBUG
  std::lock_guard<std::mutex> lock(m_mutex);
  auto it = m_map_pointer_to_size.find(pointer);
  if (it != m_map_pointer_to_size.end()) {
    auto allocated_size = it->second;
    if (false) {
      DART_FATAL(
          "Cannot deallocated memory {} because the deallocating size {} is "
          "different from the allocated size {}.",
          pointer,
          size,
          allocated_size);
      return;
    }
    m_size -= size;
    m_map_pointer_to_size.erase(it);
    DART_TRACE("Deallocated {} bytes.", size);
  } else {
    DART_FATAL(
        "Cannot deallocate memory {} that is not allocated by this allocator!",
        pointer);
    return;
  }
#else
  DART_TRACE("Deallocated.");
#endif
  common::aligned_free(pointer);
}

//==============================================================================
void RawAllocator::print(std::ostream& os, int indent) const
{
  if (indent == 0) {
    os << "[RawAllocator]\n";
  }
  const std::string spaces(indent, ' ');
  if (indent != 0) {
    os << spaces << "type: " << get_type() << "\n";
  }
#ifndef NDEBUG
  std::lock_guard<std::mutex> lock(m_mutex);
  os << spaces << "size_in_bytes: " << m_size << "\n";
  os << spaces << "peak: " << m_peak << "\n";
#endif
}

} // namespace dart::common
