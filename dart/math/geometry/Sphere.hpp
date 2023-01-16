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

#include <dart/math/geometry/Geometry3.hpp>

namespace dart::math {

/// Sphere class
/// @tparam S Scalar type
template <typename S>
class Sphere : public Geometry3<S>
{
public:
  /// Scalar type
  using Scalar = S;

  /// Computes the surface area of the sphere
  ///
  /// The surface area is computed using the formula
  /// \f$ A = 4 \pi r^2 \f$
  ///
  /// @param radius Radius of the sphere
  /// @return Surface area of the sphere
  [[nodiscard]] static S ComputeSurfaceArea(S radius);

  /// Computes the volume of a sphere
  ///
  /// The volume is computed using the formula
  /// \f$ V = \frac{4}{3} \pi r^3 \f$
  ///
  /// @param radius Radius of the sphere
  /// @return Volume of the sphere
  [[nodiscard]] static S ComputeVolume(S radius);

  /// Computes the moment of inertia of a sphere with respect to its
  /// center of mass
  ///
  /// The inertia is computed using the parallel axis theorem
  ///
  /// @param radius Radius of the sphere
  /// @param mass Mass of the sphere
  /// @return Surface area of the sphere
  [[nodiscard]] static Matrix3<S> ComputeInertiaFromMass(S radius, S mass);

  /// Computes the moment of inertia of a sphere with respect to its
  /// center of mass
  ///
  /// The inertia is computed using the parallel axis theorem and the mass is
  /// computed using the density and the volume of the sphere
  ///
  /// @param radius Radius of the sphere
  /// @param density Density of the sphere
  /// @return Inertia of the sphere
  [[nodiscard]] static Matrix3<S> ComputeInertiaFromDensity(
      S radius, S density);

  /// Default constructor
  /// @param radius Radius of the sphere
  explicit Sphere(S radius = 0.5);

  /// Destructor
  ~Sphere() override = default;

  DART_STRING_TYPE_TEMPLATE_1(Sphere, S);

  /// Computes the volume of the sphere
  [[nodiscard]] S getVolume() const override;

  /// Sets the radius of the sphere
  /// @param radius Radius of the sphere
  void setRadius(S radius);

  /// Returns the radius of the sphere
  /// @return Radius of the sphere
  [[nodiscard]] S getRadius() const;

protected:
  /// Radius of the sphere
  S mRadius{0.0};
};

DART_TEMPLATE_CLASS_HEADER(MATH, Sphere);

} // namespace dart::math

//==============================================================================
// Implementation
//==============================================================================

#include <dart/math/Constants.hpp>

namespace dart::math {

//==============================================================================
template <typename S>
S Sphere<S>::ComputeSurfaceArea(S radius)
{
  return pi<S>() * S(4) * radius * radius;
}

//==============================================================================
template <typename S>
S Sphere<S>::ComputeVolume(S radius)
{
  return pi<S>() * S(4) / S(3) * radius * radius * radius;
}

//==============================================================================
template <typename S>
Matrix3<S> Sphere<S>::ComputeInertiaFromMass(S radius, S mass)
{
  Eigen::Matrix<S, 3, 3> inertia = Eigen::Matrix<S, 3, 3>::Zero();

  inertia(0, 0) = S(2) / S(5) * mass * radius * radius;
  inertia(1, 1) = inertia(0, 0);
  inertia(2, 2) = inertia(0, 0);

  return inertia;
}

//==============================================================================
template <typename S>
Matrix3<S> Sphere<S>::ComputeInertiaFromDensity(S radius, S density)
{
  const S mass = ComputeVolume(radius) * density;
  return ComputeInertiaFromMass(radius, mass);
}

//==============================================================================
template <typename S>
Sphere<S>::Sphere(S radius) : mRadius(radius)
{
  DART_ASSERT(radius > 0);
}

//==============================================================================
template <typename S>
S Sphere<S>::getVolume() const
{
  return ComputeVolume(mRadius);
}

//==============================================================================
template <typename S>
void Sphere<S>::setRadius(S radius)
{
  DART_ASSERT(radius > 0);
  mRadius = radius;
}

//==============================================================================
template <typename S>
S Sphere<S>::getRadius() const
{
  return mRadius;
}

} // namespace dart::math
