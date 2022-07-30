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

#include "dart/common/memory_allocator/free_list_allocator.hpp"

#include <algorithm>
#include <cstdlib>

namespace dart::common {

//==============================================================================
FreeListAllocator::FreeListAllocator(
    MemoryAllocator& base_allocator, size_t initial_allocation)
  : m_base_allocator(base_allocator)
{
  allocate_memory_block(initial_allocation);
}

//==============================================================================
FreeListAllocator::~FreeListAllocator()
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

  MemoryBlockHeader* curr = m_block_head;
  while (curr) {
    DART_ASSERT(!curr->is_allocated); // TODO(JS): This means some of pointers
                                      // are not deallocated
    MemoryBlockHeader* next = curr->next;
    const auto size = curr->size;

    curr->~MemoryBlockHeader();
    m_base_allocator.deallocate(curr, size + sizeof(MemoryBlockHeader));

    curr = next;
  }
}

//==============================================================================
void* FreeListAllocator::allocate(size_t size) noexcept
{
  DART_UNUSED(size);

  if (size == 0) {
    return nullptr;
  }

  // Lock the mutex
  std::lock_guard<std::mutex> lock(m_mutex);

  MemoryBlockHeader* curr = m_block_head;
  DART_ASSERT(m_block_head->prev == nullptr);

  if (m_free_block) {
    DART_ASSERT(!m_free_block->is_allocated);
    if (size <= m_free_block->size) {
      curr = m_free_block;
      m_free_block = nullptr;
    }
  }

  while (curr) {
    if (!curr->is_allocated && size <= curr->size) {
      curr->split(size);
      break;
    }

    curr = curr->next;
  }

  if (curr == nullptr) {
    if (!allocate_memory_block((m_allocated_size + size) * 2)) {
      return nullptr;
    }

    DART_ASSERT(m_free_block != nullptr);
    DART_ASSERT(!m_free_block->is_allocated);

    curr = m_free_block;
    DART_ASSERT(curr->size >= size);

    curr->split(size);
  }

  curr->is_allocated = true;

  if (curr->next != nullptr && !curr->next->is_allocated) {
    m_free_block = curr->next;
  }

#ifndef NDEBUG
  auto out
      = static_cast<void*>(curr->as_char_ptr() + sizeof(MemoryBlockHeader));
  if (out) {
    m_size += size;
    m_peak = std::max(m_peak, m_size);
    m_map_pointer_to_size[out] = size;
  }
  return out;
#else
  return static_cast<void*>(curr->as_char_ptr() + sizeof(MemoryBlockHeader));
#endif
}

//==============================================================================
void* FreeListAllocator::allocate_aligned(
    size_t size, size_t alignment) noexcept
{
  DART_UNUSED(size, alignment);
  return nullptr;
}

//==============================================================================
void FreeListAllocator::deallocate(void* pointer, size_t size)
{
  DART_UNUSED(size, pointer);

  if (pointer == nullptr || size == 0) {
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

  unsigned char* block_addr
      = static_cast<unsigned char*>(pointer) - sizeof(MemoryBlockHeader);
  MemoryBlockHeader* block = reinterpret_cast<MemoryBlockHeader*>(block_addr);
  DART_ASSERT(block->is_allocated);
  block->is_allocated = false;

  MemoryBlockHeader* curr = block;

  if (block->prev != nullptr && !block->prev->is_allocated
      && block->prev->is_next_contiguous) {
    curr = block->prev;
    block->prev->merge(block);
  }

  if (curr->next != nullptr && !curr->next->is_allocated
      && curr->is_next_contiguous) {
    curr->merge(curr->next);
  }

  m_free_block = curr;
}

//==============================================================================
void FreeListAllocator::deallocate_aligned(void* pointer, size_t size)
{
  DART_UNUSED(size, pointer);
}

//==============================================================================
bool FreeListAllocator::allocate_memory_block(size_t size_to_allocate)
{
  void* memory
      = m_base_allocator.allocate(size_to_allocate + sizeof(MemoryBlockHeader));
  if (memory == nullptr) {
    return false;
  }

  m_block_head = m_base_allocator.construct_at<MemoryBlockHeader>(
      memory, size_to_allocate, nullptr, m_block_head, false);

  m_free_block = m_block_head;

  m_allocated_size += size_to_allocate;

  return true;
}

//==============================================================================
void FreeListAllocator::print(std::ostream& os, int indent) const
{
  // Lock the mutex
  std::lock_guard<std::mutex> lock(m_mutex);

  if (indent == 0) {
    os << "[FreeListAllocator]\n";
  }
  const std::string spaces(indent, ' ');
  if (indent != 0) {
    os << spaces << "type: " << get_type() << "\n";
  }
  os << spaces << "reserved_size: " << m_allocated_size << "\n";
  os << spaces << "memory_blocks:\n";
  auto curr = m_block_head;
  while (curr) {
    os << spaces << "- block_addr: " << curr << "\n";
    os << spaces << "  size: " << curr->size << "\n";
    os << spaces << "  prev: " << curr->prev << "\n";
    os << spaces << "  next: " << curr->next << "\n";
    os << spaces << "  is_allocated: " << curr->is_allocated << "\n";
    os << spaces << "  is_next_contiguous: " << curr->is_next_contiguous
       << "\n";
    curr = curr->next;
  }
  os << spaces << "free_block_addr: " << m_free_block << "\n";
  os << spaces << "header_size: " << sizeof(MemoryBlockHeader) << "\n";
  os << spaces << "base_allocator:\n";
  m_base_allocator.print(os, indent + 2);
}

//==============================================================================
FreeListAllocator::MemoryBlockHeader::MemoryBlockHeader(
    size_t size,
    MemoryBlockHeader* prev,
    MemoryBlockHeader* next,
    bool is_next_contiguous)
  : size(size),
    prev(prev),
    next(next),
    is_allocated(false),
    is_next_contiguous(is_next_contiguous)
{
  if (prev) {
    prev->next = this;
  }
}

//==============================================================================
size_t FreeListAllocator::MemoryBlockHeader::as_size_t() const
{
  return reinterpret_cast<size_t>(this);
}

//==============================================================================
unsigned char* FreeListAllocator::MemoryBlockHeader::as_char_ptr()
{
  return reinterpret_cast<unsigned char*>(this);
}

//==============================================================================
const unsigned char* FreeListAllocator::MemoryBlockHeader::as_char_ptr() const
{
  return reinterpret_cast<const unsigned char*>(this);
}

//==============================================================================
void FreeListAllocator::MemoryBlockHeader::split(size_t size_to_split)
{
  DART_ASSERT(size_to_split <= size);
  DART_ASSERT(!is_allocated);

  if (size_to_split + sizeof(MemoryBlockHeader) >= size) {
    // TODO(JS): Treat this as en error?
    return;
  }

  DART_ASSERT(size > size_to_split);

  unsigned char* new_block_addr
      = as_char_ptr() + sizeof(MemoryBlockHeader) + size_to_split;
  MemoryBlockHeader* new_block
      = new (static_cast<void*>(new_block_addr)) MemoryBlockHeader(
          size - sizeof(MemoryBlockHeader) - size_to_split,
          this,
          next,
          is_next_contiguous);
  next = new_block;
  if (new_block->next) {
    new_block->next->prev = new_block;
  }
  DART_ASSERT(next != this);
  is_next_contiguous = true;
  size = size_to_split;

  DART_ASSERT(is_valid());
  DART_ASSERT(new_block->is_valid());
}

//==============================================================================
void FreeListAllocator::MemoryBlockHeader::merge(MemoryBlockHeader* other)
{
  DART_ASSERT(other->prev == this);
  DART_ASSERT(other->next != this);
  DART_ASSERT(!other->is_allocated);
  DART_ASSERT(this->next == other);
  DART_ASSERT(!this->is_allocated);
  DART_ASSERT(this->is_next_contiguous);

  size += other->size + sizeof(MemoryBlockHeader);
  next = other->next;
  if (other->next) {
    other->next->prev = this;
  }
  is_next_contiguous = other->is_next_contiguous;

  other->~MemoryBlockHeader(); // TODO(JS): Need this?

  DART_ASSERT(is_valid());
}

//==============================================================================
#ifndef NDEBUG
bool FreeListAllocator::MemoryBlockHeader::is_valid() const
{
  if (prev != nullptr && prev->next != this) {
    return false;
  }

  if (next != nullptr && next->prev != this) {
    return false;
  }

  return true;
}
#endif

} // namespace dart::common
