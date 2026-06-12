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

#ifndef DART_COMMON_STLALLOCATOR_HPP_
#define DART_COMMON_STLALLOCATOR_HPP_

#include <dart/common/memory_allocator.hpp>

#include <limits>
#include <memory>
#include <type_traits>

namespace dart::common {

/// Wrapper class for MemoryAllocator to be compatible with std::allocator
template <typename T>
class StlAllocator
{
public:
  // Type aliases
  using value_type = T;
  using size_type = std::size_t;
  using pointer = T*;
  using const_pointer = const T*;
  using is_always_equal = std::false_type;
  using propagate_on_container_copy_assignment = std::true_type;
  using propagate_on_container_move_assignment = std::true_type;
  using propagate_on_container_swap = std::true_type;

  template <typename U>
  struct rebind
  {
    using other = StlAllocator<U>;
  };

  /// Default constructor
  StlAllocator(
      MemoryAllocator& baseAllocator = MemoryAllocator::GetDefault()) noexcept;

  /// Copy constructor
  constexpr StlAllocator(const StlAllocator& other) noexcept = default;

  /// Copy assignment operator
  constexpr StlAllocator& operator=(const StlAllocator& other) noexcept
      = default;

  /// Copy constructor
  template <class U>
  constexpr StlAllocator(const StlAllocator<U>& other) noexcept;

  /// Destructor
  ~StlAllocator() = default;

  /// Allocates n * sizeof(T) bytes of uninitialized storage.
  ///
  /// @param[in] n: The number of objects to allocate storage for.
  /// @param[in] hint: Point to a nearby memory location.
  /// @return On success, the pointer to the beginning of newly allocated
  /// memory.
  /// @return On failure, a null pointer
  [[nodiscard]] pointer allocate(size_type n, const void* hint = 0);

  /// Deallocates the storage referenced by the pointer @c p, which must be a
  /// pointer obtained by an earlier cal to allocate().
  ///
  /// @param[in] pointer: Pointer obtained from allocate().
  /// @param[in] n: Number of objects earlier passed to allocate().
  void deallocate(pointer pointer, size_type n) noexcept;

  /// Constructs an object at previously allocated storage.
  template <typename U, typename... Args>
  void construct(U* pointer, Args&&... args);

  /// Destroys an object constructed by this allocator.
  template <typename U>
  void destroy(U* pointer) noexcept;

  /// Upper bound on elements that can be allocated.
  [[nodiscard]] constexpr size_type max_size() const noexcept
  {
    return std::numeric_limits<size_type>::max() / sizeof(T);
  }

  /// Alignment used for an allocation of @p bytes.
  [[nodiscard]] static constexpr size_type storageAlignmentFor(
      size_type bytes) noexcept;

  template <typename U>
  [[nodiscard]] bool operator==(const StlAllocator<U>& other) const noexcept;

  template <typename U>
  [[nodiscard]] bool operator!=(const StlAllocator<U>& other) const noexcept;

  /// Prints state of the memory allocator
  void print(std::ostream& os = std::cout, int indent = 0) const;

  /// Prints state of the memory allocator
  template <typename U>
  friend std::ostream& operator<<(
      std::ostream& os, const StlAllocator<U>& allocator);

private:
  template <typename U>
  friend class StlAllocator;
  MemoryAllocator* mBaseAllocator;
};

/// Stateless STL-compatible allocator backed by DART's default C heap.
///
/// Use this adapter when container allocator state is undesirable and the C
/// heap used by the process-wide default memory allocator is the intended
/// backing resource.
template <typename T>
class DefaultStlAllocator
{
public:
  using value_type = T;
  using size_type = std::size_t;
  using pointer = T*;
  using const_pointer = const T*;
  using is_always_equal = std::true_type;
  using propagate_on_container_copy_assignment = std::true_type;
  using propagate_on_container_move_assignment = std::true_type;
  using propagate_on_container_swap = std::true_type;

  template <typename U>
  struct rebind
  {
    using other = DefaultStlAllocator<U>;
  };

  constexpr DefaultStlAllocator() noexcept = default;

  constexpr DefaultStlAllocator(const DefaultStlAllocator& other) noexcept
      = default;

  constexpr DefaultStlAllocator& operator=(
      const DefaultStlAllocator& other) noexcept = default;

  template <typename U>
  constexpr DefaultStlAllocator(const DefaultStlAllocator<U>& other) noexcept;

  /// Allocates n * sizeof(T) bytes through DART's default C heap.
  [[nodiscard]] pointer allocate(size_type n, const void* hint = 0);

  /// Deallocates storage previously returned by allocate().
  void deallocate(pointer pointer, size_type n) noexcept;

  /// Constructs an object at previously allocated storage.
  template <typename U, typename... Args>
  void construct(U* pointer, Args&&... args);

  /// Destroys an object constructed by this allocator.
  template <typename U>
  void destroy(U* pointer) noexcept;

  /// Upper bound on elements that can be allocated.
  [[nodiscard]] constexpr size_type max_size() const noexcept
  {
    return std::numeric_limits<size_type>::max() / sizeof(T);
  }
};

template <typename T, typename U>
[[nodiscard]] constexpr bool operator==(
    const DefaultStlAllocator<T>&, const DefaultStlAllocator<U>&) noexcept
{
  return true;
}

template <typename T, typename U>
[[nodiscard]] constexpr bool operator!=(
    const DefaultStlAllocator<T>&, const DefaultStlAllocator<U>&) noexcept
{
  return false;
}

} // namespace dart::common

#include <dart/common/detail/stl_allocator-impl.hpp>

#endif // DART_COMMON_STLALLOCATOR_HPP_
