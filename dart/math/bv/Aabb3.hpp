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

namespace dart::math {

/// Axis-aligned bounding box
/// @tparam S The scalar type used for internal data.
template <typename S>
class Aabb3
{
public:
  using Scalar = S;

  /// Default constructor that initializes the bounding box.
  Aabb3();

  /// Constructor that initializes the bounding box with the given minimum and
  /// maximum world coordinates.
  /// @param[in] min The minimum world coordinates of the Aabb3.
  /// @param[in] max The maximum world coordinates of the Aabb3.
  explicit Aabb3(const Vector3<S>& min, const Vector3<S>& max);

  /// Copy constructor that creates a copy of the given Aabb3 object.
  Aabb3(const Aabb3& other) = default;

  /// Move constructor that takes ownership of the data from the given Aabb3
  /// object.
  Aabb3(Aabb3&& other) = default;

  /// Copy assignment operator that copies the data from the given Aabb3 object
  /// to the current object.
  Aabb3& operator=(const Aabb3& other) = default;

  /// Move assignment operator that takes ownership of the data from the given
  /// Aabb3 object.
  Aabb3& operator=(Aabb3&& other) = default;

  /// Returns the minimum world coordinates of the Aabb3.
  [[nodiscard]] const Vector3<S> getMin() const;

  /// Returns the maximum world coordinates of the Aabb3.
  [[nodiscard]] const Vector3<S> getMax() const;

  /// Merges the current bounding box with the given Aabb3 object.
  /// @param[in] other The Aabb3 object to be merged with the current bounding
  /// box.
  Aabb3& merge(const Aabb3& other);

  /// Merges the current bounding box with the given 3D point.
  /// @param[in] point The 3D point to be merged with the current bounding box.
  Aabb3& merge(const Vector3<S>& point);

  /// Determines if the current bounding box overlaps with the given Aabb3
  /// object.
  /// @return True if the two bounding boxes overlap, false otherwise.
  [[nodiscard]] bool overlaps(const Aabb3& other);

  /// Determines if the current bounding box completely contains the given Aabb3
  /// object.
  /// @param[in] other The Aabb3 object to test for containment within the
  /// current bounding box.
  /// @return True if the current bounding box completely contains the given
  /// Aabb3 object, false otherwise.
  [[nodiscard]] bool contains(const Aabb3& other);

private:
  /// Minimum world coordinates of the Aabb3.
  Vector3<S> m_min;

  /// Maximum world coordinates of the Aabb3.
  Vector3<S> m_max;
};

DART_TEMPLATE_CLASS_HEADER(MATH, Aabb3);

} // namespace dart::math

//==============================================================================
// Implementation
//==============================================================================

#include <dart/math/Constants.hpp>

namespace dart::math {

//==============================================================================
template <typename S>
Aabb3<S>::Aabb3()
  : m_min(Vector3<S>::Constant(inf<S>())),
    m_max(Vector3<S>::Constant(-inf<S>()))
{
  // Empty
}

//==============================================================================
template <typename S>
Aabb3<S>::Aabb3(const Vector3<S>& min, const Vector3<S>& max)
  : m_min(min), m_max(max)
{
  // Do nothing
}

//==============================================================================
template <typename S>
const Vector3<S> Aabb3<S>::getMin() const
{
  return m_min;
}

//==============================================================================
template <typename S>
const Vector3<S> Aabb3<S>::getMax() const
{
  return m_max;
}

//==============================================================================
template <typename S>
Aabb3<S>& Aabb3<S>::merge(const Aabb3& other)
{
  m_min = m_min.cwiseMin(other.m_min);
  m_max = m_max.cwiseMax(other.m_max);
  return *this;
}

//==============================================================================
template <typename S>
Aabb3<S>& Aabb3<S>::merge(const Vector3<S>& point)
{
  m_min = m_min.cwiseMin(point);
  m_max = m_max.cwiseMax(point);
  return *this;
}

//==============================================================================
template <typename S>
bool Aabb3<S>::overlaps(const Aabb3& other)
{
  if ((m_min.array() > other.m_max.array()).any())
    return false;

  if ((m_max.array() < other.m_min.array()).any())
    return false;

  return true;
}

//==============================================================================
template <typename S>
bool Aabb3<S>::contains(const Aabb3& other)
{
  if ((m_min.array() > other.m_min.array()).any())
    return false;

  if ((m_max.array() < other.m_max.array()).any())
    return false;

  return true;
}

} // namespace dart::math
