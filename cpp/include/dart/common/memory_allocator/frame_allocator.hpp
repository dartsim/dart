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

class DART_COMMON_API FrameAllocator : public MemoryAllocator
{
public:
  using Base = MemoryAllocator;

  /// Constructor
  ///
  /// @param[in] base_allocator: Low level allocator to be used for allocating
  /// memory required by this memory allocator
  explicit FrameAllocator(
      MemoryAllocator& base_allocator = MemoryAllocator::GetDefault());

  /// Destructor
  ~FrameAllocator() override;

  DART_STRING_TYPE(FrameAllocator);

  // Documentation inherited
  [[nodiscard]] void* allocate(size_t size) noexcept override;

  // Documentation inherited
  [[nodiscard]] void* allocate_aligned(
      size_t size, size_t alignment) noexcept override;

  // Documentation inherited
  void deallocate(void* pointer, size_t size) override;

  // Documentation inherited
  void deallocate_aligned(void* pointer, size_t size) override;

  void reset();

  // Documentation inherited
  void print(std::ostream& os = std::cout, int indent = 0) const override;

private:
  /// The base allocator to allocate memory chunck
  MemoryAllocator& m_base_allocator;

  /// Mutex for thread safety
  mutable std::mutex m_mutex;

  size_t m_total_size_bytes;

  void* m_start_pointer;

  size_t m_offset;

  size_t m_frames_should_shrink;

  bool m_should_allocate_more;

  static constexpr int frames_until_shrink = 128;

  static constexpr size_t m_init_single_frame_allocator_bytes = 1048576; // 1 MB
};

} // namespace dart::common
