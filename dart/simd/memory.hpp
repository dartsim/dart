/*
 * Copyright (c) 2011, The DART development contributors
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

#include <new>
#include <vector>

#include <cstddef>
#include <cstdint>
#include <cstdlib>

#if defined(_MSC_VER)
  #include <malloc.h>
#endif

namespace dart::simd {

template <typename T, std::size_t Alignment = 32>
class AlignedAllocator
{
public:
  using value_type = T;
  using size_type = std::size_t;
  using difference_type = std::ptrdiff_t;
  using propagate_on_container_move_assignment = std::true_type;
  using is_always_equal = std::true_type;

  static constexpr std::size_t alignment = Alignment;

  constexpr AlignedAllocator() noexcept = default;

  template <typename U>
  constexpr AlignedAllocator(const AlignedAllocator<U, Alignment>&) noexcept
  {
  }

  [[nodiscard]] T* allocate(std::size_t n)
  {
    if (n == 0) {
      return nullptr;
    }

    if (n > static_cast<std::size_t>(-1) / sizeof(T)) {
      throw std::bad_array_new_length();
    }

    std::size_t bytes = n * sizeof(T);
    void* ptr = nullptr;

#if defined(_MSC_VER)
    ptr = _aligned_malloc(bytes, Alignment);
#else
    ptr = std::aligned_alloc(Alignment, bytes);
#endif

    if (ptr == nullptr) {
      throw std::bad_alloc();
    }

    return static_cast<T*>(ptr);
  }

  void deallocate(T* ptr, [[maybe_unused]] std::size_t n) noexcept
  {
    if (ptr == nullptr) {
      return;
    }

#if defined(_MSC_VER)
    _aligned_free(ptr);
#else
    std::free(ptr);
#endif
  }

  template <typename U>
  struct rebind
  {
    using other = AlignedAllocator<U, Alignment>;
  };
};

template <typename T1, std::size_t A1, typename T2, std::size_t A2>
[[nodiscard]] constexpr bool operator==(
    const AlignedAllocator<T1, A1>&, const AlignedAllocator<T2, A2>&) noexcept
{
  return A1 == A2;
}

template <typename T>
using aligned_vector = std::vector<T, AlignedAllocator<T, 32>>;

template <typename T>
[[nodiscard]] inline bool is_aligned(
    const T* ptr, std::size_t alignment) noexcept
{
  return (reinterpret_cast<std::uintptr_t>(ptr) % alignment) == 0;
}

template <std::size_t Alignment, typename T>
[[nodiscard]] inline T* assume_aligned(T* ptr) noexcept
{
#if defined(__GNUC__) || defined(__clang__)
  return static_cast<T*>(__builtin_assume_aligned(ptr, Alignment));
#elif defined(_MSC_VER)
  __assume((reinterpret_cast<std::uintptr_t>(ptr) & (Alignment - 1)) == 0);
  return ptr;
#else
  return ptr;
#endif
}

} // namespace dart::simd
