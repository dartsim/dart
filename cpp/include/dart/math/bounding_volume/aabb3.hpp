/*
 * Copyright (c) 2011-2021, The DART development contributors:
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

#include <vector>

#include "dart/common/limit.hpp"
#include "dart/math/constant.hpp"
#include "dart/math/geometry/line3.hpp"
#include "dart/math/geometry/ray3.hpp"

namespace dart::math {

template <typename Scalar_>
struct AabbRayIntersection3
{
  using Scalar = Scalar_;

  /// True if the box and ray intersects.
  bool isIntersecting = false;

  /// Distance to the first intersection point.
  Scalar tNear = common::max<Scalar>();

  /// Distance to the second (and the last) intersection point.
  Scalar tFar = common::max<Scalar>();
};

template <typename S_>
class Aabb3
{
public:
  using S = S_;

  static constexpr bool IS_ORIENTED = false;

  static Aabb3 FromPoints(const Vector3<S>& point1, const Vector3<S>& point2);

  /// Constructor
  Aabb3();

  /// Constructor
  Aabb3(const Vector3<S>& min, const Vector3<S>& max);

  /// Copy constructor
  Aabb3(const Aabb3& other);

  /// Move constructor
  Aabb3(Aabb3&& other);

  /// Destructor
  ~Aabb3() = default;

  void set_random();

  /// Resets this box to initial state (min=infinite, max=-infinite).
  void reset();

  void set_from_sphere(const Vector3<S>& center, S radius);

  /// Return the minimum coordinates of the Aabb3.
  const Vector3<S>& min() const;

  Vector3<S>& min();

  /// Set the minimum coordinates of the Aabb3.
  void set_min(const Vector3<S>& min);

  /// Return the maximum coordinates of the Aabb3.
  const Vector3<S>& max() const;

  Vector3<S>& get_mutable_max();

  /// Set the maximum coordinates of the Aabb3.
  void setMax(const Vector3<S>& max);

  /// Returns the mid-point of this box.
  Vector3<S> midPoint() const;

  /// Returns corner position. Index starts from x-first order.
  Vector3<S> corner(size_t idx) const;

  void setTransformed(const Aabb3& other, const Isometry3<S>& tf);

  void setTranslated(const Aabb3& other, const Vector3<S>& trans);

  void setRotated(const Aabb3& other, const Matrix3<S>& rot);

  void inflate(S delta);

  /// Expands this box by given delta to all direction.
  /// If the width of the box was x, expand(y) will result a box with
  /// x+y+y width.
  void expand(S delta);

  /// Return the center point.
  Vector3<S> get_center() const;

  Vector3<S> get_extent() const;

  S get_extent_x() const;

  S get_extent_y() const;

  S get_extentZ() const;

  /// Returns width of the box.
  S width() const;

  /// Returns height of the box.
  S height() const;

  /// Returns depth of the box.
  S depth() const;

  /// Returns length of the box in given axis.
  S length(size_t axis);

  /// Returns diagonal length of this box.
  S diagonal_length() const;

  S radius() const;

  /// Returns squared diagonal length of this box.
  S diagonal_length_squared() const;

  S size() const;

  S get_volume() const;

  S getSize() const;

  /// Return true if this Aabb3 overlaps with other Aabb3.
  bool overlaps(const Aabb3& other) const;

  /// Return true if this Aabb3 intersects with a ray.
  ///
  /// Implementation of RTCD 5.3.3.
  bool overlaps(const Line3<S>& line) const;

  /// Returns true if the input ray is intersecting with this box.
  bool overlaps(const Ray3<S>& ray) const;

  /// Return true if this Aabb3 contains other Aabb3 or they have the same
  /// size.
  bool contains(const Aabb3& other) const;

  /// Return true if this Aabb3 contains a point
  bool contains(const Vector3<S>& point) const;

  /// Returns intersection.isIntersecting = true if the input ray is
  /// intersecting with this box. If interesects, intersection.tNear is
  /// assigned with distant to the closest intersecting point, and
  /// intersection.tFar with furthest.
  AabbRayIntersection3<S> closest_intersection(const Ray3<S>& ray) const;

  bool equals(const Aabb3& other) const;

  bool is_approx(const Aabb3& other, S tol = S(1e-6)) const;

  void merge(const Vector3<S>& point);

  void merge(const Aabb3& point);

  void merge(const Aabb3& aabb1, const Aabb3& aabb2);

  /// Returns the clamped point.
  Vector3<S> clamp(const Vector3<S>& point) const;

  /// Returns true if the box is empty.
  bool isEmpty() const;

  static Aabb3 Random();

  Aabb3& operator=(const Aabb3& other);

  Aabb3& operator=(Aabb3&& other);

  bool operator==(const Aabb3& other) const;

  bool operator!=(const Aabb3& other) const;

  Aabb3& operator+=(const Aabb3& other);

  Aabb3 operator+(const Aabb3& other) const;

private:
  /// Minimum world coordinates of the Aabb3.
  Vector3<S> m_min;

  /// Maximum world coordinates of the Aabb3.
  Vector3<S> m_max;
};

using Aabb3d = Aabb3<double>;
using Aabb3f = Aabb3<float>;

using AabbRayIntersection3f = AabbRayIntersection3<float>;
using AabbRayIntersection3d = AabbRayIntersection3<double>;

// extern template class Aabb3<double>;

} // namespace dart::math

#include "dart/math/bounding_volume/detail/aabb3_impl.hpp"
