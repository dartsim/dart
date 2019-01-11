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

#ifndef DART_COMMON_DETAIL_MEMORY_IMPL_HPP_
#define DART_COMMON_DETAIL_MEMORY_IMPL_HPP_

#include <memory>
#include <Eigen/Core>
#include "dart/config.hpp"

#if EIGEN_VERSION_AT_LEAST(3,2,1) && EIGEN_VERSION_AT_MOST(3,2,8)
#include "dart/common/detail/AlignedAllocator.hpp"
#else
#include<Eigen/StdVector>
#endif

namespace dart {
namespace common {

//==============================================================================
template <typename _Tp, typename... _Args>
std::shared_ptr<_Tp> make_aligned_shared(_Args&&... __args)
{
  typedef typename std::remove_const<_Tp>::type _Tp_nc;

#if EIGEN_VERSION_AT_LEAST(3,2,1) && EIGEN_VERSION_AT_MOST(3,2,8)
  return std::allocate_shared<_Tp>(detail::aligned_allocator_cpp11<_Tp_nc>(),
                                   std::forward<_Args>(__args)...);
#else
  return std::allocate_shared<_Tp>(Eigen::aligned_allocator<_Tp_nc>(),
                                   std::forward<_Args>(__args)...);
#endif // EIGEN_VERSION_AT_LEAST(3,2,1) && EIGEN_VERSION_AT_MOST(3,2,8)
}

//==============================================================================
template<typename T, typename... Args>
std::unique_ptr<T> make_unique(Args&&... args)
{
  return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}
// TODO(JS): This is a stopgap solution as it was omitted from C++11 as "partly
// an oversight". This can be replaced by std::make_unique<T> of the standard
// library when we migrate to using C++14.

} // namespace common
} // namespace dart

#endif // DART_COMMON_DETAIL_MEMORY_IMPL_HPP_
