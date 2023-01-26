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

#include <dart/math/Fwd.hpp>

// This macro is used to define constructors for concrete Lie group and tangent
// classes such as SO3, SE3, and SOTangent.
//
// It is intended to make it easy to create new Lie group types without having
// to manually implement all the required APIs in a consistent way. The macro
// defines several constructors, including copy and move constructors, and
// constructors that take raw parameters in the form of a matrix. The macro also
// includes Doxygen comments to improve the visibility of the defined APIs.
//
// However, it's worth noting that using macros can make the code less readable,
// so it's recommended to add Doxygen comments to the APIs defined in the macro.
//
// The macro name may change until it is finalized.
#define DART_DEFINE_CONSTRUCTORS_FOR_CONCRETE(CLASS_NAME)                      \
  /**                                                                          \
   * Copy constructor                                                          \
   * @param[in] other The other element to be copied                           \
   */                                                                          \
  CLASS_NAME(const CLASS_NAME& other) : Base(), m_params(other.m_params)       \
  {                                                                            \
    /* Do nothing */                                                           \
  }                                                                            \
                                                                               \
  /**                                                                          \
   * Move constructor                                                          \
   * @param[in] other The other element to be moved                            \
   */                                                                          \
  CLASS_NAME(CLASS_NAME&& other) : Base(), m_params(std::move(other.m_params)) \
  {                                                                            \
    /* Do nothing */                                                           \
  }                                                                            \
                                                                               \
  /**                                                                          \
   * Copy-constructs from raw parameters                                       \
   * @tparam OtherDerived The derived type of the base class                   \
   * @param[in] other The other element to be copied                           \
   */                                                                          \
  template <typename OtherDerived>                                             \
  CLASS_NAME(const CLASS_NAME##Base<OtherDerived>& other)                      \
    : Base(), m_params(other.params())                                         \
  {                                                                            \
    /* Do nothing */                                                           \
  }                                                                            \
                                                                               \
  /**                                                                          \
   * Move-constructs from raw parameters                                       \
   * @tparam OtherDerived The derived type of the base class                   \
   * @param[in] other The other element to be copied                           \
   */                                                                          \
  template <typename OtherDerived>                                             \
  CLASS_NAME(CLASS_NAME##Base<OtherDerived>&& other)                           \
    : Base(), m_params(std::move(other.params()))                              \
  {                                                                            \
    /* Do nothing */                                                           \
  }                                                                            \
                                                                               \
  /**                                                                          \
   * Copy-constructs from raw parameters                                       \
   * @tparam MatrixDerived The derived type of the matrix base                 \
   * @param[in] params The raw parameters to be copied                         \
   */                                                                          \
  template <typename MatrixDerived>                                            \
  CLASS_NAME(const ::Eigen::MatrixBase<MatrixDerived>& params)                 \
    : Base(), m_params(params)                                                 \
  {                                                                            \
    /* Do nothing */                                                           \
  }                                                                            \
                                                                               \
  /**                                                                          \
   * Move-constructs from raw parameters                                       \
   * @tparam MatrixDerived The derived type of the matrix base                 \
   * @param[in] params The raw parameters to be moved                          \
   */                                                                          \
  template <typename MatrixDerived>                                            \
  CLASS_NAME(::Eigen::MatrixBase<MatrixDerived>&& params)                      \
    : Base(), m_params(std::move(params))                                      \
  {                                                                            \
    /* Do nothing */                                                           \
  }                                                                            \
  static_assert(true, "")

namespace dart::math {

template <typename S>
constexpr S LieGroupTol()
{
  // TODO: This is a temporary solution. We need to find a better way to
  // determine the tolerance.
  if constexpr (std::is_same_v<S, float>) {
    return 1e-3;
  } else if constexpr (std::is_same_v<S, double>) {
    return 1e-6;
  } else if constexpr (std::is_same_v<S, long double>) {
    return 1e-9;
  }
}

} // namespace dart::math
