/*
 * Copyright (c) 2011-2019, The DART development contributors
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

#ifndef DART_COMMON_DETAIL_ALIGNEDALLOCATOR_HPP_
#define DART_COMMON_DETAIL_ALIGNEDALLOCATOR_HPP_

#include <memory>
#include <Eigen/Core>
#include "dart/config.hpp"

namespace dart {
namespace common {

#if EIGEN_VERSION_AT_LEAST(3,2,1) && EIGEN_VERSION_AT_MOST(3,2,8)

namespace detail {

/// Aligned allocator that is compatible with C++11
// Ref: https://bitbucket.org/eigen/eigen/commits/f5b7700
// TODO: Remove this and use Eigen::aligned_allocator once new version of Eigen
// is released with above commit.
template <class T>
class aligned_allocator_cpp11 : public std::allocator<T>
{
public:
  typedef std::size_t     size_type;
  typedef std::ptrdiff_t  difference_type;
  typedef T*              pointer;
  typedef const T*        const_pointer;
  typedef T&              reference;
  typedef const T&        const_reference;
  typedef T               value_type;

  template <class U>
  struct rebind
  {
    typedef aligned_allocator_cpp11<U> other;
  };

  aligned_allocator_cpp11()
    : std::allocator<T>() {}

  aligned_allocator_cpp11(const aligned_allocator_cpp11& other)
    : std::allocator<T>(other) {}

  template <class U>
  aligned_allocator_cpp11(const aligned_allocator_cpp11<U>& other)
    : std::allocator<T>(other) {}

  ~aligned_allocator_cpp11() {}

  pointer allocate(size_type num, const void* /*hint*/ = 0)
  {
    Eigen::internal::check_size_for_overflow<T>(num);
    return static_cast<pointer>( Eigen::internal::aligned_malloc(num * sizeof(T)) );
  }

  void deallocate(pointer p, size_type /*num*/)
  {
    Eigen::internal::aligned_free(p);
  }
};

} // namespace detail

#endif // EIGEN_VERSION_AT_LEAST(3,2,1) && EIGEN_VERSION_AT_MOST(3,2,8)

} // namespace common
} // namespace dart

#endif // DART_COMMON_DETAIL_ALIGNEDALLOCATOR_HPP_
