/*
 * Copyright (c) 2011-2021, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
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

#include "dart/math/r.hpp"

namespace dart::math {

//==============================================================================
template <typename Scalar, int N, int Options>
R<Scalar, N, Options> R<Scalar, N, Options>::Zero()
{
  return R<Scalar, N, Options>(Eigen::Matrix<Scalar, N, 1, Options>::Zero());
}

//==============================================================================
template <typename Scalar, int N, int Options>
R<Scalar, N, Options> R<Scalar, N, Options>::Identity()
{
  return Zero();
}

//==============================================================================
template <typename Scalar, int N, int Options>
R<Scalar, N, Options> R<Scalar, N, Options>::Random()
{
  return R<Scalar, N, Options>(Eigen::Matrix<Scalar, N, 1, Options>::Random());
}

//==============================================================================
template <typename Scalar, int N, int Options>
R<Scalar, N, Options>::R() : m_data(Vector::Zero())
{
  // Do nothing
}

//==============================================================================
template <typename Scalar, int N, int Options>
template <typename OtherDerived>
R<Scalar, N, Options>::R(const RBase<OtherDerived>& other)
  : m_data(other.vector())
{
  // Do nothing
}

//==============================================================================
template <typename Scalar, int N, int Options>
template <typename OtherDerived>
R<Scalar, N, Options>::R(RBase<OtherDerived>&& other)
  : m_data(std::move(other.vector()))
{
  // Do nothing
}

//==============================================================================
template <typename Scalar, int N, int Options>
template <typename Derived>
R<Scalar, N, Options>::R(const math::MatrixBase<Derived>& vec) : m_data(vec)
{
  // Do nothing
}

//==============================================================================
template <typename Scalar, int N, int Options>
template <typename Derived>
R<Scalar, N, Options>::R(math::MatrixBase<Derived>&& vec)
  : m_data(std::move(vec))
{
  // Do nothing
}

//==============================================================================
template <typename Scalar, int N, int Options>
R<Scalar, N, Options>& R<Scalar, N, Options>::operator=(
    const R<Scalar, N, Options>& other)
{
  m_data = other.m_data;
  return *this;
}

//==============================================================================
template <typename Scalar, int N, int Options>
R<Scalar, N, Options>& R<Scalar, N, Options>::operator=(
    R<Scalar, N, Options>&& other)
{
  m_data = std::move(other.m_data);
  return *this;
}

//==============================================================================
template <typename Scalar, int N, int Options>
template <typename Derived>
R<Scalar, N, Options>& R<Scalar, N, Options>::operator=(
    const Eigen::MatrixBase<Derived>& matrix)
{
  m_data = matrix;
  return *this;
}

//==============================================================================
template <typename Scalar, int N, int Options>
template <typename Derived>
R<Scalar, N, Options>& R<Scalar, N, Options>::operator=(
    Eigen::MatrixBase<Derived>&& matrix)
{
  m_data = std::move(matrix);
}

//==============================================================================
template <typename Scalar, int N, int Options>
R<Scalar, N, Options> R<Scalar, N, Options>::operator-() const
{
  return R<Scalar, N, Options>(-m_data);
}

//==============================================================================
template <typename Scalar, int N, int Options>
R<Scalar, N, Options> R<Scalar, N, Options>::operator+(
    const R<Scalar, N, Options>& other) const
{
  return R<Scalar, N, Options>(m_data + other.m_data);
}

//==============================================================================
template <typename Scalar, int N, int Options>
R<Scalar, N, Options> R<Scalar, N, Options>::operator-(
    const R<Scalar, N, Options>& other) const
{
  return R<Scalar, N, Options>(m_data - other.m_data);
}

//==============================================================================
template <typename Scalar, int N, int Options>
void R<Scalar, N, Options>::set_identity()
{
  m_data.setZero();
}

//==============================================================================
template <typename Scalar, int N, int Options>
void R<Scalar, N, Options>::set_random()
{
  m_data.setRandom();
}

//==============================================================================
template <typename Scalar, int N, int Options>
constexpr int R<Scalar, N, Options>::dimension() const
{
  return GroupDim;
}

//==============================================================================
template <typename Scalar, int N, int Options>
typename R<Scalar, N, Options>::Vector& R<Scalar, N, Options>::vector()
{
  return m_data;
}

//==============================================================================
template <typename Scalar, int N, int Options>
const typename R<Scalar, N, Options>::Vector& R<Scalar, N, Options>::vector()
    const
{
  return m_data;
}

//==============================================================================
template <typename Scalar, int Options>
R<Scalar, Eigen::Dynamic, Options>::R() : m_data(Vector())
{
  // Do nothing
}

//==============================================================================
template <typename Scalar, int Options>
R<Scalar, Eigen::Dynamic, Options>::R(int dim) : m_data(Vector::Zero(dim))
{
  // Do nothing
}

//==============================================================================
template <typename Scalar, int Options>
R<Scalar, Eigen::Dynamic, Options>::R(const R& other) : m_data(other.m_data)
{
  // Do nothing
}

//==============================================================================
template <typename Scalar, int Options>
R<Scalar, Eigen::Dynamic, Options>::R(R&& other)
  : m_data(std::move(other.m_data))
{
  // Do nothing
}

//==============================================================================
template <typename Scalar, int Options>
template <typename Derived>
R<Scalar, Eigen::Dynamic, Options>::R(const math::MatrixBase<Derived>& vector)
  : m_data(vector)
{
  // Do nothing
}

//==============================================================================
template <typename Scalar, int Options>
template <typename Derived>
R<Scalar, Eigen::Dynamic, Options>::R(math::MatrixBase<Derived>&& vector)
  : m_data(std::move(vector))
{
  // Do nothing
}

//==============================================================================
template <typename Scalar, int Options>
template <typename Derived>
R<Scalar, Eigen::Dynamic, Options>&
R<Scalar, Eigen::Dynamic, Options>::operator=(
    const Eigen::MatrixBase<Derived>& matrix)
{
  m_data = matrix;
}

//==============================================================================
template <typename Scalar, int Options>
template <typename Derived>
R<Scalar, Eigen::Dynamic, Options>&
R<Scalar, Eigen::Dynamic, Options>::operator=(
    Eigen::MatrixBase<Derived>&& matrix)
{
  m_data = std::move(matrix);
}

//==============================================================================
template <typename Scalar, int Options>
const R<Scalar, Eigen::Dynamic, Options>&
R<Scalar, Eigen::Dynamic, Options>::operator+() const
{
  return *this;
}

//==============================================================================
template <typename Scalar, int Options>
R<Scalar, Eigen::Dynamic, Options>
R<Scalar, Eigen::Dynamic, Options>::operator-() const
{
  return R<Scalar, Eigen::Dynamic, Options>(m_data);
}

//==============================================================================
template <typename Scalar, int Options>
int R<Scalar, Eigen::Dynamic, Options>::dimension() const
{
  return m_data.size();
}

//==============================================================================
template <typename Scalar, int Options>
const typename R<Scalar, Eigen::Dynamic, Options>::Vector&
R<Scalar, Eigen::Dynamic, Options>::vector() const
{
  return m_data;
}

//==============================================================================
template <typename Scalar, int Options>
typename R<Scalar, Eigen::Dynamic, Options>::Vector&
R<Scalar, Eigen::Dynamic, Options>::mutable_vector()
{
  return m_data;
}

//==============================================================================
template <typename Scalar, int Dim, int Options>
RTangent<Scalar, Dim, Options>::RTangent() : m_data(TangentVector::Zero())
{
  // Do nothing
}

//==============================================================================
template <typename Scalar, int Dim, int Options>
RTangent<Scalar, Dim, Options>::RTangent(const RTangent& other)
  : m_data(other.m_data)
{
  // Do nothing
}

//==============================================================================
template <typename Scalar, int Dim, int Options>
RTangent<Scalar, Dim, Options>::RTangent(RTangent&& other)
  : m_data(std::move(other.m_data))
{
  // Do nothing
}

//==============================================================================
template <typename Scalar, int Dim, int Options>
template <typename OtherDerived>
RTangent<Scalar, Dim, Options>::RTangent(
    const RTangentBase<OtherDerived>& other)
  : m_data(other.vector())
{
  // Do nothing
}

//==============================================================================
template <typename Scalar, int Dim, int Options>
template <typename OtherDerived>
RTangent<Scalar, Dim, Options>::RTangent(RTangentBase<OtherDerived>&& other)
  : m_data(std::move(other.vector()))
{
  // Do nothing
}

//==============================================================================
template <typename Scalar, int Dim, int Options>
template <typename Derived>
RTangent<Scalar, Dim, Options>::RTangent(
    const Eigen::MatrixBase<Derived>& coeffs)
  : m_data(coeffs)
{
  // Do nothing
}

//==============================================================================
template <typename Scalar, int Dim, int Options>
template <typename Derived>
RTangent<Scalar, Dim, Options>::RTangent(Eigen::MatrixBase<Derived>&& coeffs)
  : m_data(std::move(coeffs))
{
  // Do nothing
}

//==============================================================================
template <typename Scalar, int Dim, int Options>
Scalar RTangent<Scalar, Dim, Options>::operator*(
    const RCotangent<Scalar, Dim>& cotan) const
{
  return m_data.dot(cotan.m_data);
}

//==============================================================================
template <typename Scalar, int Dim, int Options>
typename RTangent<Scalar, Dim, Options>::LieAlgebra
RTangent<Scalar, Dim, Options>::hat() const
{
  LieAlgebra out = LieAlgebra::Zero();
  out.template topRightCorner<Dim + 1, 1>() = m_data;
  return out;
}

//==============================================================================
template <typename Scalar, int Options>
RTangent<Scalar, Eigen::Dynamic, Options>::RTangent(int size)
  : m_data(TangentVector::Zero(size))
{
  // Do nothing
}

//==============================================================================
template <typename Scalar, int Options>
RTangent<Scalar, Eigen::Dynamic, Options>::RTangent(const RTangent& other)
  : m_data(other.m_data)
{
  // Do nothing
}

//==============================================================================
template <typename Scalar, int Options>
RTangent<Scalar, Eigen::Dynamic, Options>::RTangent(RTangent&& other)
  : m_data(std::move(other.m_data))
{
  // Do nothing
}

//==============================================================================
template <typename Scalar, int Options>
template <typename Derived>
RTangent<Scalar, Eigen::Dynamic, Options>::RTangent(
    const Eigen::MatrixBase<Derived>& coeffs)
  : m_data(coeffs)
{
  // Do nothing
}

//==============================================================================
template <typename Scalar, int Options>
template <typename Derived>
RTangent<Scalar, Eigen::Dynamic, Options>::RTangent(
    Eigen::MatrixBase<Derived>&& coeffs)
  : m_data(std::move(coeffs))
{
  // Do nothing
}

//==============================================================================
template <typename Scalar, int Options>
Scalar RTangent<Scalar, Eigen::Dynamic, Options>::operator*(
    const RCotangent<Scalar, Eigen::Dynamic>& cotan) const
{
  return m_data.dot(cotan.m_data);
}

//==============================================================================
template <typename Scalar, int Options>
typename RTangent<Scalar, Eigen::Dynamic, Options>::LieAlgebra
RTangent<Scalar, Eigen::Dynamic, Options>::hat() const
{
  LieAlgebra out = LieAlgebra::Zero(m_data.size() + 1, m_data.size() + 1);
  out.topRightCorner(m_data.size() + 1, 1) = m_data;
  return out;
}

} // namespace dart::math

namespace Eigen {

//==============================================================================
template <typename Scalar, int N, int Options>
Map<dart::math::R<Scalar, N, Options>, Options>::Map(Scalar* data)
  : m_data(data)
{
  // Do nothing
}

//==============================================================================
template <typename Scalar, int N, int Options>
Map<const dart::math::R<Scalar, N, Options>, Options>::Map(const Scalar* data)
  : m_data(data)
{
  // Do nothing
}

//==============================================================================
template <typename Scalar, int N, int Options>
Map<dart::math::RTangent<Scalar, N, Options>, Options>::Map(Scalar* data)
  : m_data(data)
{
  // Do nothing
}

//==============================================================================
template <typename Scalar, int N, int Options>
Map<const dart::math::RTangent<Scalar, N, Options>, Options>::Map(
    const Scalar* data)
  : m_data(data)
{
  // Do nothing
}

} // namespace Eigen
