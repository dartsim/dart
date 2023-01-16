/*
 * Copyright (c) 2011-2023, The DART development contributors
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

#pragma once

#include <dart/common/Castable.hpp>
#include <dart/common/Fwd.hpp>

#include <iostream>
#include <string>

#include <cstddef>

namespace dart::common {

/// Base class for std::allocator compatible allocators.
class DART_COMMON_API AlignedAllocator : public Castable<AlignedAllocator>
{
public:
  /// Returns the default memory allocator
  static AlignedAllocator& GetDefault();

  /// Default constructor
  AlignedAllocator() noexcept = default;

  /// Destructor
  virtual ~AlignedAllocator() = default;

  /// Returns type string.
  [[nodiscard]] virtual const std::string& getType() const = 0;

  /// Allocates \c size bytes of uninitialized storage.
  ///
  /// \param[in] bytes: The byte size to allocate storage for.
  /// \param[in] alignment: The alignment of the allocated memory.
  /// \return On success, the pointer to the beginning of newly allocated
  /// memory.
  /// \return On failure, a null pointer
  [[nodiscard]] virtual void* allocate(
      size_t bytes, size_t alignment) noexcept = 0;
  // TODO(JS): Make this constexpr once migrated to C++20

  /// Allocates object(s) without calling the constructor.
  ///
  /// This is identical to \c static_cast<T*>(allocate(n * sizeof(T))).
  ///
  /// \param[in] n: The number of objects to allocate.
  template <typename T>
  [[nodiscard]] T* allocateAs(size_t n = 1) noexcept;

  /// Deallocates the storage referenced by the pointer \c p, which must be a
  /// pointer obtained by an earlier cal to allocate().
  ///
  /// \param[in] pointer: Pointer obtained from allocate().
  /// \param[in] bytes: The bytes of the allocated memory.
  virtual void deallocate(void* pointer, size_t bytes) = 0;
  // TODO(JS): Make this constexpr once migrated to C++20

  /// Allocates uninitialized storage and constructs an object of type T to the
  /// allocated storage.
  ///
  /// \tparam T: The object type to construct.
  /// \tparam Args...: The argument types to pass to the object constructor.
  ///
  /// \param[in] args: The constructor arguments to use.
  template <typename T, typename... Args>
  [[nodiscard]] T* construct(Args&&... args) noexcept;

  template <typename T, typename... Args>
  [[nodiscard]] T* constructAt(void* pointer, Args&&... args);

  template <typename T, typename... Args>
  [[nodiscard]] T* constructAt(T* pointer, Args&&... args);

  /// Calls the destructor of the object and deallocate the storage.
  template <typename T>
  void destroy(T* object) noexcept;

  /// Prints state of the memory allocator
  virtual void print(std::ostream& os = std::cout, int indent = 0) const;

  /// Prints state of the memory allocator
  friend std::ostream& operator<<(
      std::ostream& os, const AlignedAllocator& allocator);
};

} // namespace dart::common

#include <dart/common/allocator/detail/AlignedAllocator-impl.hpp>
