/*
 * Copyright (c) 2011-2022, The DART development contributors:
 * https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include "dart/math/lie_group/detail/r_tangent.hpp"
#include "dart/math/lie_group/detail/so3_tangent.hpp"
#include "dart/math/lie_group/detail/tangent_base.hpp"

namespace dart::math {

//==============================================================================
template <typename Derived>
class SE3TangentBase : public TangentBase<Derived>
{
public:
  using This = SE3TangentBase<Derived>;
  using Base = TangentBase<Derived>;

  DART_LIE_GROUP_USE_BASE_TYPES;

  using AngularBlock =
      typename DataType::template FixedSegmentReturnType<3>::Type;
  using LinearBlock =
      typename DataType::template FixedSegmentReturnType<3>::Type;
  using ConstAngularBlock =
      typename DataType::template ConstFixedSegmentReturnType<3>::Type;
  using ConstLinearBlock =
      typename DataType::template ConstFixedSegmentReturnType<3>::Type;

  using Quaternion = math::Quaternion<Scalar, Options>;
  using QuaternionMap = Eigen::Map<Quaternion>;
  using ConstQuaternionMap = Eigen::Map<const Quaternion>;

  template <typename OtherTangent>
  Tangent ad(const SE3TangentBase<OtherTangent>& other) const
  {
    //--------------------------------------------------------------------------
    // ad(s1, s2) = | [w1]    0 | | w2 |
    //              | [v1] [w1] | | v2 |
    //
    //            = |          [w1]w2 |
    //              | [v1]w2 + [w1]v2 |
    //--------------------------------------------------------------------------
    return Tangent(
        angular_coeffs().cross(other.angular_coeffs()),
        angular_coeffs().cross(other.linear_coeffs())
            + linear_coeffs().cross(other.angular_coeffs()));
  }

  const Eigen::Map<const SO3Tangent<Scalar>> angular() const
  {
    return Eigen::Map<const SO3Tangent<Scalar>>(angular_coeffs().data());
  }

  const Eigen::Map<const R3Tangent<Scalar>> linear() const
  {
    return Eigen::Map<const R3Tangent<Scalar>>(linear_coeffs().data());
  }

  const ConstAngularBlock angular_coeffs() const
  {
    return coeffs().template head<3>();
  }

  AngularBlock angular_coeffs()
  {
    return coeffs().template head<3>();
  }

  const ConstLinearBlock linear_coeffs() const
  {
    return coeffs().template tail<3>();
  }

  LinearBlock linear_coeffs()
  {
    return coeffs().template tail<3>();
  }

  using Base::coeffs;
};

} // namespace dart::math
