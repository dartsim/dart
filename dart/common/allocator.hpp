/*
 * Copyright (c) 2011-2021, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
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

#include <cstddef>
#include <memory>
#include <type_traits>

#include "dart/common/Memory.hpp"

namespace dart::common {

template <typename T>
constexpr T next_pow_2(T n) {
  unsigned int count = 0;

  // First n in the below condition
  // is for the case where n is 0
  if (n && !(n & (n - 1)))
    return n;

  while (n != 0) {
    n >>= 1;
    count += 1;
  }

  return 1 << count;
}

// enum class AlignmentType : std::size_t {
//  DEFAULT = sizeof(void*),
//  SSE = 16,
//  AVX = 32,
//};

class Allocator {
public:
  /// Constructor
  Allocator() = default;

  /// Destructor
  virtual ~Allocator() = default;

  /// Allocates memory of a given size in bytes and return a pointer to the
  /// allocated memory.
  virtual void* allocate(std::size_t size) = 0;

  /// Releases previously allocated memory.
  virtual void release(void* pointer, std::size_t size) = 0;
};

template <typename T, std::size_t Alignment = 8>
class AlignedAllocator;

template <std::size_t Alignment>
class AlignedAllocator<void, Alignment> {
public:
  using pointer = void*;
  using const_pointer = const void*;
  using value_type = void;

  constexpr std::size_t alignment() const {
    return Alignment;
  }

  template <class U>
  struct rebind {
    using other = AlignedAllocator<U, Alignment>;
  };
};

template <typename T, std::size_t Alignment>
class AlignedAllocator {
public:
  using value_type = T;
  using pointer = T*;
  using const_pointer = const T*;
  using reference = T&;
  using const_reference = const T&;
  using size_type = size_t;
  using difference_type = ptrdiff_t;

  using propagate_on_container_move_assignment = std::true_type;

  template <class U>
  struct rebind {
    using other = AlignedAllocator<U, Alignment>;
  };

public:
  AlignedAllocator() noexcept {
  }

  template <class U>
  AlignedAllocator(const AlignedAllocator<U, Alignment>& /*other*/) noexcept {
  }

  constexpr std::size_t alignment() const {
    return Alignment;
  }

  size_type max_size() const noexcept {
    return (size_type(~0) - size_type(Alignment)) / sizeof(T);
  }

  pointer address(reference x) const noexcept {
    return std::addressof(x);
  }

  const_pointer address(const_reference x) const noexcept {
    return std::addressof(x);
  }

  pointer allocate(size_type n, const_pointer = 0) {
    void* ptr = std::aligned_alloc(Alignment, n * sizeof(T));
    if (ptr == nullptr) {
      throw std::bad_alloc();
    }

    return reinterpret_cast<pointer>(ptr);
  }

  void deallocate(pointer p, size_type) noexcept {
    return std::free(p);
  }

  template <class U, class... Args>
  void construct_at(U* p, Args&&... args) {
    ::new (reinterpret_cast<void*>(p)) U(std::forward<Args>(args)...);
  }

  void destroy(pointer p) {
    p->~T();
  }
};

template <typename T, std::size_t TAlign, typename U, std::size_t UAlign>
inline constexpr bool operator==(
    const AlignedAllocator<T, TAlign>&,
    const AlignedAllocator<U, UAlign>&) noexcept {
  return TAlign == UAlign;
}

template <typename T, std::size_t TAlign, typename U, std::size_t UAlign>
inline constexpr bool operator!=(
    const AlignedAllocator<T, TAlign>&,
    const AlignedAllocator<U, UAlign>&) noexcept {
  return TAlign != UAlign;
}

} // namespace dart::common
