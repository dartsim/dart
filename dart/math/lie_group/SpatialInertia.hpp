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
#include <dart/math/lie_group/SE3TangentBase.hpp>

namespace dart::math {

enum class SpatialInertiaType
{
  AT_COM,
};

template <typename S, SpatialInertiaType = SpatialInertiaType::AT_COM>
class SpatialInertia;

template <typename S, SpatialInertiaType>
class SpatialInertia
{
public:
  using Scalar = S;
};

template <typename S>
class SpatialInertia<S, SpatialInertiaType::AT_COM>
{
public:
  using Scalar = S;
  static constexpr SpatialInertiaType Type = SpatialInertiaType::AT_COM;

  /// Returns the identity spatial inertia.
  [[nodiscard]] static const SpatialInertia& Identity()
  {
    static SpatialInertia identity;
    return identity;
  }

  /// Default constructor. This creates a spatial inertia with one mass and
  /// identity inertia.
  SpatialInertia() : m_inertia(Matrix3<S>::Identity()), m_mass(1)
  {
    // Do nothing
  }

  /// Copy constructor
  SpatialInertia(const SpatialInertia<S, SpatialInertiaType::AT_COM>& other)
    : m_inertia(other.m_inertia), m_mass(other.m_mass)
  {
    // Do nothing
  }

  /// Move constructor
  SpatialInertia(SpatialInertia<S, SpatialInertiaType::AT_COM>&& other)
    : m_inertia(std::move(other.m_inertia)), m_mass(std::move(other.m_mass))
  {
    // Do nothing
  }

  /// Constructor
  ///
  /// @param[in] inertia Inertia matrix.
  /// @param[in] mass Mass.
  template <typename MatrixDerived>
  explicit SpatialInertia(
      const Eigen::MatrixBase<MatrixDerived>& inertia, S mass)
    : m_inertia(inertia), m_mass(mass)
  {
    // Do nothing
  }

  /// Move constructor
  ///
  /// @param[in] inertia Inertia matrix.
  /// @param[in] mass Mass.
  template <typename MatrixDerived>
  explicit SpatialInertia(Eigen::MatrixBase<MatrixDerived>&& inertia, S mass)
    : m_inertia(std::move(inertia)), m_mass(mass)
  {
    // Do nothing
  }

  /// Assignment operator
  SpatialInertia& operator=(
      const SpatialInertia<S, SpatialInertiaType::AT_COM>& other)
  {
    m_inertia = other.m_inertia;
    m_mass = other.m_mass;
    return *this;
  }

  /// Move assignment operator
  SpatialInertia& operator=(
      SpatialInertia<S, SpatialInertiaType::AT_COM>&& other)
  {
    m_inertia = std::move(other.m_inertia);
    m_mass = std::move(other.m_mass);
    return *this;
  }

  /// Returns the spatial momentum
  ///
  /// @param[in] V Spatial velocity.
  /// @return Spatial momentum.
  template <typename SE3TangentDerived>
  [[nodiscard]] SE3Tangent<S> operator*(
      const SE3TangentBase<SE3TangentDerived>& V) const
  {
    return SE3Tangent<S>(inertia() * V.angular(), mass() * V.linear());
  }

  /// Returns the sum of two spatial inertias.
  template <typename OtherS>
  SpatialInertia<S, SpatialInertiaType::AT_COM> operator+(
      const SpatialInertia<OtherS, SpatialInertiaType::AT_COM>& other) const
  {
    return SpatialInertia<S, SpatialInertiaType::AT_COM>(
        inertia() + other.inertia(), mass() + other.mass());
  }

  /// Returns whether this spatial inertia is the identity.
  [[nodiscard]] bool isIdentity() const
  {
    return (std::abs(mass() - 1) < LieGroupTol<S>()) && inertia().isIdentity();
  }

  /// Converts to a 6x6 matrix representation.
  [[nodiscard]] Matrix6<S> toMatrix() const
  {
    Matrix6<S> out = Matrix6<S>::Zero();
    out.template topLeftCorner<3, 3>() = m_inertia;
    out.template bottomRightCorner<3, 3>().noalias()
        = Matrix3<S>::Identity() * m_mass;
    return out;
  }

  /// Returns the moment of inertia matrix.
  [[nodiscard]] const Matrix3<S>& inertia() const
  {
    return m_inertia;
  }

  /// Returns the moment of inertia matrix.
  [[nodiscard]] Matrix3<S>& inertia()
  {
    return m_inertia;
  }

  /// Returns the mass.
  [[nodiscard]] const S mass() const
  {
    return m_mass;
  }

  /// Returns the mass.
  [[nodiscard]] S& mass()
  {
    return m_mass;
  }

private:
  Matrix3<S> m_inertia{Matrix3<S>::Identity()};
  S m_mass{1};
};

} // namespace dart::math

//==============================================================================
// Implementation
//==============================================================================

namespace dart::math {

//==============================================================================

} // namespace dart::math
