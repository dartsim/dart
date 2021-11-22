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
#include "dart/common/memory_allocator/memory_allocator.hpp"

namespace dart::common {

/// std::allocator compatible allocator that wrapps various MemoryAllocator
template <typename T>
class StlAllocator : public std::allocator<T>
{
public:
  // Type aliases
  using Base = std::allocator<T>;
  using value_type = typename std::allocator_traits<Base>::value_type;
  using size_type = typename std::allocator_traits<Base>::size_type;
  using pointer = typename std::allocator_traits<Base>::pointer;
  using const_pointer = typename std::allocator_traits<Base>::const_pointer;

  template <typename U>
  struct rebind
  {
    using other = StlAllocator<U>;
  };

  /// Default constructor
  StlAllocator(
      MemoryAllocator& base_allocator = MemoryAllocator::GetDefault()) noexcept;

  /// Copy constructor
  StlAllocator(const StlAllocator& other) throw();

  /// Copy constructor
  template <class U>
  StlAllocator(const StlAllocator<U>& other) throw();

  /// Destructor
  ~StlAllocator() = default;

  const std::string& get_type() const;

  /// Allocates n * sizeof(T) bytes of uninitialized storage.
  ///
  /// @param[in] n: The number of objects to allocate sotrage for.
  /// @param[in] hint: Point to a nearby memory location.
  /// @return On success, the pointer to the beginning of newly allocated
  /// memory.
  /// @return On failure, a null pointer
  [[nodiscard]] pointer allocate(size_type n, const void* hint = 0);

  /// Deallocates the storage referenced by the pointer @c p, which must be a
  /// pointer obtained by an earlier cal to allocate() or allocate_aligned().
  ///
  /// @param[in] pointer: Pointer obtained from allocate() or
  /// allocate_aligned().
  /// @param[in] n: Number of objects earlier passed to allocate() or
  /// allocate_aligned().
  void deallocate(pointer pointer, size_type n);
  // TODO(JS): Make this constexpr once migrated to C++20

  // TODO(JS): Add size_type max_size() const noexcept;

  void print(std::ostream& os = std::cout, int indent = 0) const;

  template <typename U>
  friend std::ostream& operator<<(
      std::ostream& os, const StlAllocator<U>& allocator);

private:
  template <typename U>
  friend class StlAllocator;
  MemoryAllocator& m_base_allocator;
};

} // namespace dart::common

#include "dart/common/memory_allocator/detail/stl_allocator_impl.hpp"
