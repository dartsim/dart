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

#include <cstddef>

#include "dart/common/export.hpp"
#include "dart/common/memory.hpp"

namespace dart::common {

/// Base class for memory allocators.
///
/// \note The derived classes should be thread safe when DART_ENABLE_THREAD_SAFE
/// is defined to 1
template <typename T = void>
class MemoryAllocator
{
public:
  using value_type = T;

  /// Default constructor
  MemoryAllocator() noexcept = default;

  /// Destructor
  virtual ~MemoryAllocator() = default;

  /// Allocates @c size bytes of uninitialized storage.
  ///
  /// @param[in] size: Number of bytes to allocate.
  /// @param[in] alignment: Alignment size. Default is 0.
  /// @return On success, the pointer to the beginning of newly allocated
  /// memory.
  /// @return On failure, a null pointer
  [[nodiscard]] virtual T* allocate(size_t size, size_t alignment = 0) = 0;
  // TODO(JS): Make this constexpr once migrated to C++20

  /// Releases previously allocated memory.
  virtual void deallocate(T* pointer, size_t) = 0;
  // TODO(JS): Make this constexpr once migrated to C++20

  /// Creates an object.
  template <typename... Args>
  [[nodiscard]] T* construct(Args&&... args) noexcept;

  /// Creates an object to an aligned memory.
  template <typename... Args>
  [[nodiscard]] T* aligned_construct(size_t alignment, Args&&... args) noexcept;

  /// Destroys an object created by this allocator.
  void destroy(T* object) noexcept;

protected:
  bool is_valid_alignment(size_t size, size_t alignment) const;
};

} // namespace dart::common

#include "dart/common/memory_allocator/detail/memory_allocator_impl.hpp"
