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

#include <dart/math/lie_group/SO3Base.hpp>

namespace Eigen::internal {

// TODO(JS): Move to a dedicated header file
/// @brief Specialization of Eigen::internal::traits for SO3
template <typename S, int Options_>
struct traits<::dart::math::SO3<S, Options_>>
{
  static constexpr int Options = Options_;
  static constexpr int DataDim = 4;

  using Scalar = S;
  using Data = ::Eigen::Quaternion<S, Options>;
  using PlainObject = ::dart::math::SO3<S, Options_>;
  using MatrixType = ::Eigen::Matrix<S, 3, 3>;
};

} // namespace Eigen::internal

namespace dart::math {

/// @brief SO3 is a specialization of LieGroupBase for SO3
/// @tparam S The scalar type
/// @tparam Options_ The options for the underlying Eigen::Matrix
template <typename S, int Options_>
class SO3 : public SO3Base<SO3<S, Options_>>
{
public:
  using Base = SO3Base<SO3<S, Options_>>;

  using Scalar = typename Base::Scalar;
  using Data = typename Base::Data;
  using PlainObject = typename Base::PlainObject;
  using MatrixType = typename Base::MatrixType;

  template <typename MatrixDrived>
  static Matrix3<S> Hat(const Eigen::MatrixBase<MatrixDrived>& xi);

  /// Returns the identity SO3
  [[nodiscard]] static PlainObject Identity();

  /// Returns a random SO3
  [[nodiscard]] static PlainObject Random();

  /// Default constructor that initializes the quaternion to identity
  SO3();

  /// Copy constructor
  /// @param[in] other The other SO3 to be copied
  SO3(const SO3& other);

  /// Move constructor
  /// @param[in] other The other SO3 to be moved
  SO3(SO3&& other);

  /// Constructs an SO3 from a quaternion
  template <typename QuaternionDrived>
  SO3(const ::Eigen::QuaternionBase<QuaternionDrived>& quat);

  /// Constructs an SO3 from a quaternion
  template <typename QuaternionDrived>
  SO3(::Eigen::QuaternionBase<QuaternionDrived>&& quat);

  /// Copy assignment operator
  /// @param[in] other The other SO3 to be copied
  /// @return Reference to this SO3
  SO3& operator=(const SO3& other);

  /// Move assignment operator
  /// @param[in] other The other SO3 to be moved
  /// @return Reference to this SO3
  SO3& operator=(SO3&& other);

  /// Returns the matrix representation of this SO3
  [[nodiscard]] MatrixType matrix() const;

  /// Returns the quaternion representation of this SO3
  [[nodiscard]] const Data& quaternion() const;

  /// Returns the quaternion representation of this SO3
  [[nodiscard]] Data& quaternion();

  /// Returns the pointer to data of the underlying quaternion coefficients
  [[nodiscard]] const Scalar* data() const;

  /// Returns the pointer to data of the underlying quaternion coefficients
  [[nodiscard]] Scalar* data();

private:
  /// The underlying quaternion coefficients
  Data m_data;
};

DART_TEMPLATE_CLASS_HEADER(MATH, SO3);

} // namespace dart::math

//==============================================================================
// Implementation
//==============================================================================

namespace dart::math {

//==============================================================================
template <typename S, int Options>
template <typename MatrixDrived>
Matrix3<S> SO3<S, Options>::Hat(const Eigen::MatrixBase<MatrixDrived>& xi)
{
  // clang-format off
  return Matrix3<S>{
    {      0, -xi[2], +xi[1]},
    { +xi[2],      0, -xi[0]},
    { -xi[1], +xi[0],      0}
  };
  // clang-format on
}

//==============================================================================
template <typename S, int Options>
typename SO3<S, Options>::PlainObject SO3<S, Options>::Identity()
{
  return SO3(::Eigen::Quaternion<S>::Identity());
}

//==============================================================================
template <typename S, int Options>
typename SO3<S, Options>::PlainObject SO3<S, Options>::Random()
{
  return SO3(::Eigen::Quaternion<S>::UnitRandom());
}

//==============================================================================
template <typename S, int Options>
template <typename QuaternionDrived>
SO3<S, Options>::SO3(const ::Eigen::QuaternionBase<QuaternionDrived>& quat)
  : m_data(quat)
{
  // Do nothing
}

//==============================================================================
template <typename S, int Options>
template <typename QuaternionDrived>
SO3<S, Options>::SO3(::Eigen::QuaternionBase<QuaternionDrived>&& quat)
  : m_data(std::move(quat))
{
  // Do nothing
}

//==============================================================================
template <typename S, int Options>
SO3<S, Options>::SO3() : m_data(::Eigen::Quaternion<S>::Identity().coeffs())
{
  // Do nothing
}

//==============================================================================
template <typename S, int Options>
SO3<S, Options>::SO3(const SO3& other) : m_data(other.m_data)
{
  // Do nothing
}

//==============================================================================
template <typename S, int Options>
SO3<S, Options>::SO3(SO3&& other) : m_data(std::move(other.m_data))
{
  // Do nothing
}

//==============================================================================
template <typename S, int Options>
SO3<S, Options>& SO3<S, Options>::operator=(const SO3& other)
{
  m_data = other.m_data;
  return *this;
}

//==============================================================================
template <typename S, int Options>
SO3<S, Options>& SO3<S, Options>::operator=(SO3&& other)
{
  m_data = std::move(other.m_data);
  return *this;
}

//==============================================================================
template <typename S, int Options>
typename SO3<S, Options>::MatrixType SO3<S, Options>::matrix() const
{
  return ::Eigen::Quaternion<S>(m_data).toRotationMatrix();
}

//==============================================================================
template <typename S, int Options>
const typename SO3<S, Options>::Data& SO3<S, Options>::quaternion() const
{
  return m_data;
}

//==============================================================================
template <typename S, int Options>
typename SO3<S, Options>::Data& SO3<S, Options>::quaternion()
{
  return m_data;
}

//==============================================================================
template <typename S, int Options>
const typename SO3<S, Options>::Scalar* SO3<S, Options>::data() const
{
  return m_data.coeffs().data();
}

//==============================================================================
template <typename S, int Options>
typename SO3<S, Options>::Scalar* SO3<S, Options>::data()
{
  return m_data.coeffs().data();
}

} // namespace dart::math
