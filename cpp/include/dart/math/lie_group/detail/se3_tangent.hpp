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

#include "dart/math/lie_group/detail/se3_tangent_base.hpp"

namespace Eigen::internal {

//==============================================================================
template <typename Scalar_, int Options_>
struct traits<dart::math::SE3Tangent<Scalar_, Options_>>
{
  static constexpr int Options = Options_;
  static constexpr int SpaceDim = 3;
  static constexpr int GroupDim = 6;
  static constexpr int MatrixDim = 4;
  static constexpr int DataDim = 6;

  DART_LIE_GROUP_TRAITS_TYPES(SE3);

  using DataType = Eigen::Matrix<Scalar, DataDim, 1>;
};

} // namespace Eigen::internal

namespace dart::math {

//==============================================================================
template <typename Scalar_, int Options_>
class SE3Tangent : public SE3TangentBase<SE3Tangent<Scalar_, Options_>>
{
public:
  using Base = SE3TangentBase<SE3Tangent<Scalar_, Options_>>;
  using This = SE3Tangent<Scalar_, Options_>;

  DART_LIE_GROUP_USE_BASE_TYPES;

  using Quaternion = typename Base::Quaternion;
  using QuaternionMap = typename Base::QuaternionMap;
  using ConstQuaternionMap = typename Base::ConstQuaternionMap;

  /// Default constructor
  SE3Tangent() : m_data(DataType::Zero())
  {
    // Do nothing
  }

  DART_LIE_GROUP_CONSTRUCTORS(SE3Tangent);

#if EIGEN_VERSION_AT_LEAST(3, 4, 0)
  template <typename DerivedA, typename DerivedB>
  SE3Tangent(
      const Eigen::MatrixBase<DerivedA>& angular,
      const Eigen::MatrixBase<DerivedB>& linear)
    : m_data(
        angular[0], angular[1], angular[2], linear[0], linear[1], linear[2])
  {
    // Do nothing
  }
#else
  template <typename DerivedA, typename DerivedB>
  SE3Tangent(
      const Eigen::MatrixBase<DerivedA>& angular,
      const Eigen::MatrixBase<DerivedB>& linear)
  {
    m_data.template head<3>() = angular;
    m_data.template tail<3>() = linear;
  }
#endif

#if EIGEN_VERSION_AT_LEAST(3, 4, 0)
  template <typename DerivedA, typename DerivedB>
  SE3Tangent(
      Eigen::MatrixBase<DerivedA>&& angular,
      Eigen::MatrixBase<DerivedB>&& linear)
    : m_data(
        angular[0], angular[1], angular[2], linear[0], linear[1], linear[2])
  {
    // Do nothing
  }
#else
  template <typename DerivedA, typename DerivedB>
  SE3Tangent(
      Eigen::MatrixBase<DerivedA>&& angular,
      Eigen::MatrixBase<DerivedB>&& linear)
  {
    m_data.template head<3>() = std::move(angular);
    m_data.template tail<3>() = std::move(linear);
  }
#endif

  SE3Tangent& operator=(const SE3Tangent& o)
  {
    coeffs() = o.coeffs();
    return derived();
  }

  const DataType& coeffs() const
  {
    return m_data;
  }

  DataType& coeffs()
  {
    return m_data;
  }

  using Base::data;

protected:
  using Base::derived;

private:
  DataType m_data;
};

DART_TEMPLATE_CLASS_SCALAR(SE3Tangent)

} // namespace dart::math
