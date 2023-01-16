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

#include <dart/common/allocator/MemoryAllocatorAligned.hpp>

#include <memory>

namespace dart::common {

/// This class is a wrapper for a MemoryAllocator making it compatible with the
/// STL's std::allocator interface. By using this wrapper class, you can use
/// your custom memory allocator with STL container classes such as std::vector
/// and std::list.
///
/// Example Usage:
///
/// @code
/// MyAllocator<double> allocator;
/// std::vector<double, MyAllocator<double>> vec(allocator);
/// vec.push_back(1.2);
/// @endcode
///
/// Note that this class can be used with any custom memory allocator that
/// implements the required interface of the std::allocator, such as the
/// FreeListAllocator.
template <typename T>
class StlAllocatorAligned : public std::allocator<T>
{
public:
  using Base = std::allocator<T>;

  // The following typedefs are required by the C++ Standard Library to be a
  // valid allocator type (see 20.1.5 [allocator.requirements]/3) and are used
  // by the STL containers to determine the type of the elements they contain.
  // The typedefs are also used by the STL algorithms to determine the type of
  // the elements they operate on. The typedefs are not used by the allocator.
  // The typedefs are not required to be public, but they are public here to
  // make it easier to use the allocator with the STL containers.
  using value_type = typename std::allocator_traits<Base>::value_type;
  using size_type = typename std::allocator_traits<Base>::size_type;
  using pointer = typename std::allocator_traits<Base>::pointer;
  using const_pointer = typename std::allocator_traits<Base>::const_pointer;

  template <typename U>
  struct rebind
  {
    using other = StlAllocatorAligned<U>;
  };

  /// Default constructor
  explicit StlAllocatorAligned(
      MemoryAllocatorAligned& baseAllocator
      = MemoryAllocatorAligned::GetDefault()) noexcept;

  /// Copy constructor
  StlAllocatorAligned(const StlAllocatorAligned& other) throw();

  /// Copy constructor
  template <class U>
  StlAllocatorAligned(const StlAllocatorAligned<U>& other) throw();

  /// Destructor
  ~StlAllocatorAligned() = default;

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
      std::ostream& os, const StlAllocatorAligned<U>& allocator);

private:
  template <typename U>
  friend class StlAllocatorAligned;
  MemoryAllocatorAligned& mBaseAllocator;
};

} // namespace dart::common

#include <dart/common/allocator/detail/StlAllocatorAligned-impl.hpp>
