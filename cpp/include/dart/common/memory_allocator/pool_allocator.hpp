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

#pragma once

#include <mutex>

#include "dart/common/memory_allocator/memory_allocator.hpp"

namespace dart::common {

class DART_COMMON_API PoolAllocator : public MemoryAllocator
{
public:
  using Base = MemoryAllocator;

  /// Constructor
  ///
  /// @param[in] base_allocator: Low level allocator to be used for allocating
  /// memory required by this memory allocator
  explicit PoolAllocator(
      MemoryAllocator& base_allocator = MemoryAllocator::GetDefault());

  /// Destructor
  ~PoolAllocator() override;

  DART_STRING_TYPE(PoolAllocator);

  // Documentation inherited
  [[nodiscard]] void* allocate(size_t size) noexcept override;

  // Documentation inherited
  [[nodiscard]] void* allocate_aligned(
      size_t size, size_t alignment) noexcept override;

  // Documentation inherited
  void deallocate(void* pointer, size_t size) override;

  // Documentation inherited
  void deallocate_aligned(void* pointer, size_t size) override;

  // Documentation inherited
  void print(std::ostream& os = std::cout, int indent = 0) const override;

private:
  struct MemoryUnit
  {
    MemoryUnit* next;
  };

  struct MemoryBlock
  {
    MemoryUnit* memory_units;
  };

  static constexpr int HEAP_COUNT = 128;

  static constexpr size_t MAX_UNIT_SIZE = 1024;

  static constexpr size_t BLOCK_SIZE = 16 * MAX_UNIT_SIZE;

  static size_t m_unit_sizes[HEAP_COUNT];

  static int m_map_size_to_heap_index[MAX_UNIT_SIZE + 1];

  static bool m_initialized;

  /// The base allocator to allocate memory chunk
  MemoryAllocator& m_base_allocator;

  int m_allocated_memory_block_count;

  int m_current_memory_blocks_count;

  /// Mutex for thread safety
  mutable std::mutex m_mutex;

  MemoryUnit* m_free_memory_units[HEAP_COUNT];

  MemoryBlock* m_memory_blocks;

#ifndef NDEBUG
private:
  size_t m_size = 0;
  size_t m_peak = 0;
  std::unordered_map<void*, size_t> m_map_pointer_to_size;
#endif
};

} // namespace dart::common
