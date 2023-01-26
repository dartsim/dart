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

#include <dart/math/lie_group/SE3TangentBase.hpp>

namespace Eigen::internal {

// TODO(JS): Move to a dedicated header file
/// @brief Specialization of Eigen::internal::traits for SE3Tangent
template <typename S>
struct traits<::dart::math::SE3Tangent<S>>
{
  using Scalar = S;

  // LieGroup common
  static constexpr int Dim = 3;
  static constexpr int DoF = 6;
  static constexpr int ParamSize = DoF;
  using LieGroup = ::dart::math::SE3<S>;
  using Tangent = ::dart::math::SE3Tangent<S>;
  using Params = ::Eigen::Matrix<S, ParamSize, 1>;
};

} // namespace Eigen::internal

namespace dart::math {

/// @brief SE3Tangent is a specialization of LieGroupBase for SE3Tangent
/// @tparam S The scalar type
template <typename S>
class SE3Tangent : public SE3TangentBase<SE3Tangent<S>>
{
public:
  using Base = SE3TangentBase<SE3Tangent<S>>;
  using Scalar = typename Base::Scalar;

  // LieGroup types
  using LieGroup = typename Base::LieGroup;
  using Tangent = typename Base::Tangent;
  using Params = typename Base::Params;

  DART_DEFINE_CONSTRUCTORS_FOR_CONCRETE(SE3Tangent);

  template <typename MatrixDerivedA, typename MatrixDerivedB>
  SE3Tangent(
      const ::Eigen::MatrixBase<MatrixDerivedA>& angular,
      const ::Eigen::MatrixBase<MatrixDerivedB>& linear)
    : Base(), m_params((Params() << angular, linear).finished())
  {
    /* Do nothing */
  }

  /**
   * Move-constructs from raw parameters
   * @param[in] params The raw parameters to be moved
   */
  template <typename MatrixDerivedA, typename MatrixDerivedB>
  SE3Tangent(
      ::Eigen::MatrixBase<MatrixDerivedA>&& angular,
      ::Eigen::MatrixBase<MatrixDerivedB>&& linear)
    : Base(),
      m_params((Params() << std::move(angular), std::move(linear)).finished())
  {
    /* Do nothing */
  }

  operator Vector6<Scalar>() const
  {
    return m_params;
  }
  // TODO(JS): Check if this can be moved to upper level

  /// Returns the underlying params
  [[nodiscard]] const Params& params() const;

  /// Returns the underlying params
  [[nodiscard]] Params& params();

private:
  /// The underlying params
  Params m_params;
};

DART_TEMPLATE_CLASS_HEADER(MATH, SE3Tangent);

} // namespace dart::math

//==============================================================================
// Implementation
//==============================================================================

namespace dart::math {

//==============================================================================
template <typename S>
const typename SE3Tangent<S>::Params& SE3Tangent<S>::params() const
{
  return m_params;
}

//==============================================================================
template <typename S>
typename SE3Tangent<S>::Params& SE3Tangent<S>::params()
{
  return m_params;
}

} // namespace dart::math
