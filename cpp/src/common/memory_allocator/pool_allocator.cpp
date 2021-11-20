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

#include "dart/common/memory_allocator/pool_allocator.hpp"

#include <algorithm>
#include <cstdlib>

namespace dart::common {

//==============================================================================
constexpr int PoolAllocator::HEAP_COUNT;

constexpr size_t PoolAllocator::MAX_UNIT_SIZE;

constexpr size_t PoolAllocator::BLOCK_SIZE;

size_t PoolAllocator::m_unit_sizes[HEAP_COUNT];

int PoolAllocator::m_map_size_to_heap_index[MAX_UNIT_SIZE + 1];

bool PoolAllocator::m_initialized;

//==============================================================================
PoolAllocator::PoolAllocator(MemoryAllocator& base_allocator)
  : m_base_allocator(base_allocator)
{
  m_allocated_memory_block_count = 64;
  m_current_memory_blocks_count = 0;
  const size_t size_to_allocate
      = m_allocated_memory_block_count * sizeof(MemoryBlock);
  m_memory_blocks = m_base_allocator.allocate_as<MemoryBlock>(
      m_allocated_memory_block_count);
  std::memset(m_memory_blocks, 0, size_to_allocate);
  std::memset(m_free_memory_units, 0, sizeof(m_free_memory_units));

  static_assert(8 <= sizeof(MemoryUnit), "");

  // Global setting
  if (!m_initialized) {
    for (auto i = 0u; i < HEAP_COUNT; ++i) {
      m_unit_sizes[i] = (i + 1) * 8;
    }

    auto j = 0u;
    m_map_size_to_heap_index[0] = -1;
    for (auto i = 1u; i <= MAX_UNIT_SIZE; ++i) {
      if (i <= m_unit_sizes[j]) {
        m_map_size_to_heap_index[i] = j;
      } else {
        m_map_size_to_heap_index[i] = ++j;
      }
    }

    m_initialized = true;
  }
}

//==============================================================================
PoolAllocator::~PoolAllocator()
{
  // Lock the mutex
  std::lock_guard<std::mutex> lock(m_mutex);

#ifndef NDEBUG
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

  for (int i = 0; i < m_current_memory_blocks_count; ++i) {
    m_base_allocator.deallocate(m_memory_blocks[i].memory_units, BLOCK_SIZE);
  }
  m_base_allocator.deallocate(
      m_memory_blocks, m_allocated_memory_block_count * sizeof(MemoryBlock));
}

//==============================================================================
void* PoolAllocator::allocate(size_t size) noexcept
{
  if (size == 0) {
    return nullptr;
  }

  if (size > MAX_UNIT_SIZE) {
    DART_TRACE(
        "Cannot allocate memory of size > {} using PoolAllocator.",
        MAX_UNIT_SIZE);
    return m_base_allocator.allocate(size);
  }

  // Lock the mutex
  std::lock_guard<std::mutex> lock(m_mutex);

  const int heap_index = m_map_size_to_heap_index[size];

  if (MemoryUnit* unit = m_free_memory_units[heap_index]) {
    m_free_memory_units[heap_index] = unit->next;
#ifndef NDEBUG
    if (unit) {
      m_size += size;
      m_peak = std::max(m_peak, m_size);
      m_map_pointer_to_size[unit] = size;
    }
    return unit;
#else
    return unit;
#endif
  }

  if (m_current_memory_blocks_count == m_allocated_memory_block_count) {
    MemoryBlock* current_memory_blocks = m_memory_blocks;
    m_allocated_memory_block_count += 64;
    m_memory_blocks = m_base_allocator.allocate_as<MemoryBlock>(
        m_allocated_memory_block_count);
    std::memcpy(
        m_memory_blocks,
        current_memory_blocks,
        m_current_memory_blocks_count * sizeof(MemoryBlock));
    std::memset(
        m_memory_blocks + m_current_memory_blocks_count,
        0,
        64 * sizeof(MemoryBlock));
  }

  MemoryBlock* new_block = m_memory_blocks + m_current_memory_blocks_count;
  new_block->memory_units
      = static_cast<MemoryUnit*>(m_base_allocator.allocate(BLOCK_SIZE));
  const size_t unit_size = m_unit_sizes[heap_index];
  const unsigned int unit_count = BLOCK_SIZE / unit_size;
  void* memory_units_begin = static_cast<void*>(new_block->memory_units);
  char* memory_units_begin_char = static_cast<char*>(memory_units_begin);
  for (size_t i = 0u; i < unit_count; ++i) {
    void* unit_pointer
        = static_cast<void*>(memory_units_begin_char + unit_size * i);
    void* next_unit_pointer
        = static_cast<void*>(memory_units_begin_char + unit_size * (i + 1));
    MemoryUnit* unit = static_cast<MemoryUnit*>(unit_pointer);
    MemoryUnit* next_unit = static_cast<MemoryUnit*>(next_unit_pointer);
    unit->next = next_unit;
  }

  void* last_unit_pointer = static_cast<void*>(
      memory_units_begin_char + unit_size * (unit_count - 1));
  MemoryUnit* last_unit = static_cast<MemoryUnit*>(last_unit_pointer);
  last_unit->next = nullptr;

  m_free_memory_units[heap_index] = new_block->memory_units->next;
  m_current_memory_blocks_count++;

#ifndef NDEBUG
  if (new_block->memory_units) {
    m_size += size;
    m_peak = std::max(m_peak, m_size);
    m_map_pointer_to_size[new_block->memory_units] = size;
  }
  return new_block->memory_units;
#else
  return new_block->memory_units;
#endif
}

//==============================================================================
void* PoolAllocator::allocate_aligned(size_t size, size_t alignment) noexcept
{
  DART_UNUSED(alignment);

  if (size == 0) {
    return nullptr;
  }

  if (size > MAX_UNIT_SIZE) {
    DART_TRACE(
        "Cannot allocate memory of size > {} using PoolAllocator.",
        MAX_UNIT_SIZE);
    return m_base_allocator.allocate_aligned(size, alignment);
  }

  // Lock the mutex
  std::lock_guard<std::mutex> lock(m_mutex);

  const int heap_index = m_map_size_to_heap_index[size];

  if (MemoryUnit* unit = m_free_memory_units[heap_index]) {
    m_free_memory_units[heap_index] = unit->next;
#ifndef NDEBUG
    if (unit) {
      m_size += size;
      m_peak = std::max(m_peak, m_size);
      m_map_pointer_to_size[unit] = size;
    }
    return unit;
#else
    return unit;
#endif
  }

  if (m_current_memory_blocks_count == m_allocated_memory_block_count) {
    MemoryBlock* current_memory_blocks = m_memory_blocks;
    m_allocated_memory_block_count += 64;
    m_memory_blocks = m_base_allocator.allocate_as<MemoryBlock>(
        m_allocated_memory_block_count);
    std::memcpy(
        m_memory_blocks,
        current_memory_blocks,
        m_current_memory_blocks_count * sizeof(MemoryBlock));
    std::memset(
        m_memory_blocks + m_current_memory_blocks_count,
        0,
        64 * sizeof(MemoryBlock));
  }

  MemoryBlock* new_block = m_memory_blocks + m_current_memory_blocks_count;
  new_block->memory_units
      = static_cast<MemoryUnit*>(m_base_allocator.allocate(BLOCK_SIZE));
  const size_t unit_size = m_unit_sizes[heap_index];
  const unsigned int unit_count = BLOCK_SIZE / unit_size;
  void* memory_units_begin = static_cast<void*>(new_block->memory_units);
  char* memory_units_begin_char = static_cast<char*>(memory_units_begin);
  for (size_t i = 0u; i < unit_count; ++i) {
    void* unit_pointer
        = static_cast<void*>(memory_units_begin_char + unit_size * i);
    void* next_unit_pointer
        = static_cast<void*>(memory_units_begin_char + unit_size * (i + 1));
    MemoryUnit* unit = static_cast<MemoryUnit*>(unit_pointer);
    MemoryUnit* next_unit = static_cast<MemoryUnit*>(next_unit_pointer);
    unit->next = next_unit;
  }

  void* last_unit_pointer = static_cast<void*>(
      memory_units_begin_char + unit_size * (unit_count - 1));
  MemoryUnit* last_unit = static_cast<MemoryUnit*>(last_unit_pointer);
  last_unit->next = nullptr;

  m_free_memory_units[heap_index] = new_block->memory_units->next;
  m_current_memory_blocks_count++;

#ifndef NDEBUG
  if (new_block->memory_units) {
    m_size += size;
    m_peak = std::max(m_peak, m_size);
    m_map_pointer_to_size[new_block->memory_units] = size;
  }
  return new_block->memory_units;
#else
  return new_block->memory_units;
#endif
}

//==============================================================================
void PoolAllocator::deallocate(void* pointer, size_t size)
{
  if (size > MAX_UNIT_SIZE) {
    m_base_allocator.deallocate(pointer, size);
    return;
  }

  // Lock the mutex
  std::lock_guard<std::mutex> lock(m_mutex);

#ifndef NDEBUG
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
#endif

  const int heap_index = m_map_size_to_heap_index[size];

  MemoryUnit* released_unit = static_cast<MemoryUnit*>(pointer);
  released_unit->next = m_free_memory_units[heap_index];
  m_free_memory_units[heap_index] = released_unit;
}

//==============================================================================
void PoolAllocator::deallocate_aligned(void* pointer, size_t size)
{
  if (size > MAX_UNIT_SIZE) {
    m_base_allocator.deallocate_aligned(pointer, size);
    return;
  }

  // Lock the mutex
  std::lock_guard<std::mutex> lock(m_mutex);

  const int heap_index = m_map_size_to_heap_index[size];

  MemoryUnit* released_unit = static_cast<MemoryUnit*>(pointer);
  released_unit->next = m_free_memory_units[heap_index];
  m_free_memory_units[heap_index] = released_unit;
}

//==============================================================================
void PoolAllocator::print(std::ostream& os, int indent) const
{
  // Lock the mutex
  std::lock_guard<std::mutex> lock(m_mutex);

  if (indent == 0) {
    os << "[PoolAllocator]\n";
  }
  const std::string spaces(indent, ' ');
  if (indent != 0) {
    os << spaces << "type: " << get_type() << "\n";
  }
  os << spaces
     << "allocated_memory_block_count: " << m_allocated_memory_block_count
     << "\n";
  os << spaces
     << "current_memory_blocks_count: " << m_current_memory_blocks_count
     << "\n";
  os << spaces << "base_allocator:\n";
  m_base_allocator.print(os, indent + 2);
}

} // namespace dart::common
