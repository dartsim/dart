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

#include <cstddef>

#include "dart/common/export.hpp"
#include "dart/common/memory.hpp"

namespace dart::common {

/// Base class for std::allocator compatible allocators.
class DART_COMMON_API MemoryAllocator
{
public:
  static MemoryAllocator& GetDefault();

  /// Default constructor
  MemoryAllocator() noexcept = default;

  /// Destructor
  virtual ~MemoryAllocator() = default;

  /// Returns type string.
  virtual const std::string& get_type() const = 0;

  /// Allocates @c size bytes of uninitialized storage.
  ///
  /// @param[in] n: The byte size to allocate sotrage for.
  /// @param[in] alignment: Alignment size. Default is 0.
  /// @return On success, the pointer to the beginning of newly allocated
  /// memory.
  /// @return On failure, a null pointer
  [[nodiscard]] virtual void* allocate(size_t size) noexcept = 0;
  // TODO(JS): Make this constexpr once migrated to C++20

  /// Allocates @c size bytes of uninitialized storage whose alignment is
  /// specified by @c alignmnet.
  ///
  /// @param[in] n: The byte size to allocate sotrage for.
  /// @param[in] alignment: Alignment size. Default is 0.
  /// @return On success, the pointer to the beginning of newly allocated
  /// memory.
  /// @return On failure, a null pointer
  [[nodiscard]] virtual void* allocate_aligned(
      size_t size, size_t alignment) noexcept = 0;
  // TODO(JS): Make this constexpr once migrated to C++20

  template <typename T>
  [[nodiscard]] T* allocate_as(size_t n = 1) noexcept;

  /// Deallocates the storage referenced by the pointer @c p, which must be a
  /// pointer obtained by an earlier cal to allocate() or allocate_aligned().
  ///
  /// @param[in] pointer: Pointer obtained from allocate() or
  /// allocate_aligned().
  virtual void deallocate(void* pointer, size_t size) = 0;
  // TODO(JS): Make this constexpr once migrated to C++20

  /// Deallocates the storage referenced by the pointer @c p, which must be a
  /// pointer obtained by an earlier cal to allocate() or allocate_aligned().
  ///
  /// @param[in] pointer: Pointer obtained from allocate() or
  /// allocate_aligned().
  virtual void deallocate_aligned(void* pointer, size_t size) = 0;
  // TODO(JS): Make this constexpr once migrated to C++20

  /// Allocates uninitialized storage and constructs an object of type T to the
  /// allocated storage.
  ///
  /// @param[in] args...: The constructor arguments to use.
  template <typename T, typename... Args>
  [[nodiscard]] T* construct(Args&&... args) noexcept;

  /// Allocates uninitialized storage and constructs an object of type T to the
  /// allocated storage whose alignment is specified by @c alignmnet.
  ///
  /// @param[in] alignment: Alignment size. Default is 0.
  /// @param[in] args...: The constructor arguments to use.
  template <typename T, typename... Args>
  [[nodiscard]] T* construct_aligned(
      size_t alignment = 0, Args&&... args) noexcept;

  template <typename T, typename... Args>
  [[nodiscard]] T* construct_at(void* pointer, Args&&... args);

  template <typename T, typename... Args>
  [[nodiscard]] T* construct_at(T* pointer, Args&&... args);

  /// Calls the destructor of the object and deallocate the storage.
  template <typename T>
  void destroy(T* object) noexcept;

  /// Prints state of the memory allocator
  virtual void print(std::ostream& os = std::cout, int indent = 0) const;

  /// Prints state of the memory allocator
  DART_COMMON_API friend std::ostream& operator<<(
      std::ostream& os, const MemoryAllocator& allocator);

#ifndef NDEBUG
  // virtual size_t get_peak() const = 0;
  // virtual void print();
#endif

protected:
  /// Returns true if @c alignment is valid for @c size.
  bool is_valid_alignment(size_t size, size_t alignment) const;
};

} // namespace dart::common

#include "dart/common/memory_allocator/detail/memory_allocator_impl.hpp"
