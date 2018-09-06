/*
 * Copyright (c) 2011-2018, The DART development contributors
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

#ifndef DART_MATH_DETAIL_TYPETRAITS_HPP_
#define DART_MATH_DETAIL_TYPETRAITS_HPP_

#include <type_traits>

namespace dart {
namespace math {
namespace detail {

/// Check whether \c T can be used for std::uniform_int_distribution<T>
/// Reference:
/// https://en.cppreference.com/w/cpp/numeric/random/uniform_int_distribution
template <typename T, typename Enable = void>
struct is_compatible_to_uniform_int_distribution : std::false_type
{
};

template <typename T>
struct
    is_compatible_to_uniform_int_distribution<T,
                                              typename std::
                                                  enable_if<std::is_same<
                                                                typename std::
                                                                    remove_cv<T>::
                                                                        type,
                                                                short>::value
                                                            || std::is_same<
                                                                   typename std::
                                                                       remove_cv<T>::
                                                                           type,
                                                                   int>::value
                                                            || std::is_same<
                                                                   typename std::
                                                                       remove_cv<T>::
                                                                           type,
                                                                   long>::value
                                                            || std::is_same<
                                                                   typename std::
                                                                       remove_cv<T>::
                                                                           type,
                                                                   long long>::
                                                                   value
                                                            || std::is_same<
                                                                   typename std::
                                                                       remove_cv<T>::
                                                                           type,
                                                                   unsigned short>::
                                                                   value
                                                            || std::is_same<
                                                                   typename std::
                                                                       remove_cv<T>::
                                                                           type,
                                                                   unsigned int>::
                                                                   value
                                                            || std::is_same<
                                                                   typename std::
                                                                       remove_cv<T>::
                                                                           type,
                                                                   unsigned long>::
                                                                   value
                                                            || std::is_same<
                                                                   typename std::
                                                                       remove_cv<T>::
                                                                           type,
                                                                   unsigned long long>::
                                                                   value>::type>
    : std::true_type
{
};

} // namespace detail
} // namespace math
} // namespace dart

#endif // DART_MATH_DETAIL_TYPETRAITS_HPP_
