/*
 * Copyright (c) 2011-2022, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef DART_COMMON_MEMORYALLOCATOR_HPP_
#define DART_COMMON_MEMORYALLOCATOR_HPP_

#include <cstddef>
#include <iostream>
#include <string>

#include "dart/common/Castable.hpp"

namespace dart::common {

/// Base class for std::allocator compatible allocators.
class MemoryAllocator : public Castable<MemoryAllocator>
{
public:
  /// Returns the default memory allocator
  static MemoryAllocator& GetDefault();

  /// Default constructor
  MemoryAllocator() noexcept = default;

  /// Destructor
  virtual ~MemoryAllocator() = default;

  /// Returns type string.
  [[nodiscard]] virtual const std::string& getType() const = 0;

  /// Allocates @c size bytes of uninitialized storage.
  ///
  /// @param[in] size: The byte size to allocate sotrage for.
  /// @return On success, the pointer to the beginning of newly allocated
  /// memory.
  /// @return On failure, a null pointer
  [[nodiscard]] virtual void* allocate(size_t size) noexcept = 0;
  // TODO(JS): Make this constexpr once migrated to C++20

  template <typename T>
  [[nodiscard]] T* allocateAs(size_t n = 1) noexcept;

  /// Deallocates the storage referenced by the pointer @c p, which must be a
  /// pointer obtained by an earlier cal to allocate().
  ///
  /// @param[in] pointer: Pointer obtained from allocate().
  virtual void deallocate(void* pointer, size_t size) = 0;
  // TODO(JS): Make this constexpr once migrated to C++20

  /// Allocates uninitialized storage and constructs an object of type T to the
  /// allocated storage.
  ///
  /// @param[in] args...: The constructor arguments to use.
  template <typename T, typename... Args>
  [[nodiscard]] T* construct(Args&&... args) noexcept;

  template <typename T, typename... Args>
  [[nodiscard]] T* constructAt(void* pointer, Args&&... args);

  template <typename T, typename... Args>
  [[nodiscard]] T* constructAt(T* pointer, Args&&... args);

  /// Calls the destructor of the object and deallocate the storage.
  template <typename T>
  void destroy(T* object) noexcept;

#ifndef NDEBUG
  /// Returns true if a pointer is allocated by this allocator.
  [[nodiscard]] virtual bool isAllocated(void* pointer, size_t size) const
      noexcept
      = 0;

  /// Returns true if there is no memory allocated by this allocator.
  [[nodiscard]] virtual bool isEmpty() const noexcept = 0;
#endif

  /// Prints state of the memory allocator
  virtual void print(std::ostream& os = std::cout, int indent = 0) const;

  /// Prints state of the memory allocator
  friend std::ostream& operator<<(
      std::ostream& os, const MemoryAllocator& allocator);
};

} // namespace dart::common

#include "dart/common/detail/MemoryAllocator-impl.hpp"

#endif // DART_COMMON_MEMORYALLOCATOR_HPP_
