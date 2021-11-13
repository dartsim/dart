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

#include "dart/common/memory_allocator/memory_allocator.hpp"

namespace dart::common {

/// Memory allocator to manage the memory as a stack.
///
/// This class keeps a pointer to the current memory address and it moves it
/// forward for every allocation. This class also can remove it backwards when a
/// deallocation operation is done.
class DART_COMMON_API StackAllocator : public MemoryAllocator
{
public:
  /// Constructor
  StackAllocator(
      size_t max_capacity,
      MemoryAllocator& base_allocator = get_default_allocator());

  /// Destructor
  ~StackAllocator() override;

  /// @copydoc MemoryAllocator::allocate
  ///
  /// Complexity is O(1).
  [[nodiscard]] void* allocate(size_t size, size_t alignment = 0) override;

  /// This function does nothing. The allocated memory is released when this
  /// allocator is destructed.
  ///
  /// Complexity is O(1).
  void deallocate(void* pointer, size_t) override;

  /// Returns the maximum capacity of this allocator.
  size_t get_max_capacity() const;

  /// Returns the size of allocated memory.
  ///
  /// The return is the same as get_size_in_bytes() if T is void.
  size_t get_size() const;

  /// Returns the first address of this allocator uses.
  const void* get_begin_address() const;

private:
  struct Header
  {
    char padding;
  };

  /// The maximum size of memory that this allocator can allocate
  const size_t m_max_capacity;

  /// The base allocator to allocate memory chunck
  MemoryAllocator& m_base_allocator;

  /// The memory address of this allocator uses
  void* m_start_ptr = nullptr;

  /// The size of allocated memory or the offset from m_start_ptr.
  size_t m_offset = 0;

  /// Mutex for thread safety
  mutable std::mutex m_mutex;
};

} // namespace dart::common
