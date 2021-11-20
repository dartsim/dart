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

#include "dart/common/export.hpp"
#include "dart/common/memory_allocator/memory_allocator.hpp"

namespace dart::common {

class DART_COMMON_API FreeListAllocator : public MemoryAllocator
{
public:
  FreeListAllocator(
      MemoryAllocator& base_allocator = MemoryAllocator::GetDefault(),
      size_t initial_allocation = 1048576 /* 1 MB */);

  ~FreeListAllocator() override;

  DART_STRING_TYPE(FreeListAllocator);

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
  struct MemoryBlockHeader
  {
    size_t size;

    MemoryBlockHeader* prev;

    MemoryBlockHeader* next;

    bool is_allocated;

    bool is_next_contiguous;

    MemoryBlockHeader(
        size_t size,
        MemoryBlockHeader* prev,
        MemoryBlockHeader* next,
        bool is_next_contiguous);

    size_t as_size_t() const;

    unsigned char* as_char_ptr();

    const unsigned char* as_char_ptr() const;

    void split(size_t size_to_split);

    void merge(MemoryBlockHeader* other);

#ifndef NDEBUG
    bool is_valid() const;
#endif
  };

  bool allocate_memory_block(size_t size_to_allocate);

  MemoryAllocator& m_base_allocator;

  mutable std::mutex m_mutex;

  MemoryBlockHeader* m_block_head{nullptr};

  MemoryBlockHeader* m_free_block{nullptr};

  size_t m_allocated_size{0};

#ifndef NDEBUG
private:
  size_t m_size = 0;
  size_t m_peak = 0;
  std::unordered_map<void*, size_t> m_map_pointer_to_size;
#endif
};

} // namespace dart::common
