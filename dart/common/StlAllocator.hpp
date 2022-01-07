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

#ifndef DART_COMMON_STLALLOCATOR_HPP_
#define DART_COMMON_STLALLOCATOR_HPP_

#include <memory>

#include "dart/common/MemoryAllocator.hpp"

namespace dart::common {

/// Wrapper class for MemoryAllocator to be compatible with std::allocator
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
  explicit StlAllocator(
      MemoryAllocator& baseAllocator = MemoryAllocator::GetDefault()) noexcept;

  /// Copy constructor
  StlAllocator(const StlAllocator& other) throw();

  /// Copy constructor
  template <class U>
  StlAllocator(const StlAllocator<U>& other) throw();

  /// Destructor
  ~StlAllocator() = default;

  /// Allocates n * sizeof(T) bytes of uninitialized storage.
  ///
  /// \param[in] n: The number of objects to allocate sotrage for.
  /// \param[in] hint: Point to a nearby memory location.
  /// \return On success, the pointer to the beginning of newly allocated
  /// memory.
  /// \return On failure, a null pointer
  [[nodiscard]] pointer allocate(size_type n, const void* hint = 0);

  /// Deallocates the storage referenced by the pointer \c p, which must be a
  /// pointer obtained by an earlier cal to allocate().
  ///
  /// \param[in] pointer: Pointer obtained from allocate().
  /// \param[in] n: Number of objects earlier passed to allocate().
  void deallocate(pointer pointer, size_type n);
  // TODO(JS): Make this constexpr once migrated to C++20

  // TODO(JS): Add size_type max_size() const noexcept;

  /// Prints state of the memory allocator
  void print(std::ostream& os = std::cout, int indent = 0) const;

  /// Prints state of the memory allocator
  template <typename U>
  friend std::ostream& operator<<(
      std::ostream& os, const StlAllocator<U>& allocator);

private:
  template <typename U>
  friend class StlAllocator;
  MemoryAllocator& mBaseAllocator;
};

} // namespace dart::common

#include "dart/common/detail/StlAllocator-impl.hpp"

#endif // DART_COMMON_STLALLOCATOR_HPP_
