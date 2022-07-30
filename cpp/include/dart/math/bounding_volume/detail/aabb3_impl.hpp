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

#include "dart/math/bounding_volume/aabb3.hpp"

namespace dart::math {

//==============================================================================
template <typename S>
Aabb3<S> Aabb3<S>::From_points(
    const Vector3<S>& point1, const Vector3<S>& point2)
{
  return Aabb3<S>(point1.cwiseMin(point2), point1.cwiseMax(point2));
}

//==============================================================================
template <typename S>
Aabb3<S>::Aabb3()
  : m_min(Vector3<S>::Constant(common::inf<S>())),
    m_max(Vector3<S>::Constant(-common::inf<S>()))
{
  // Do nothing
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
Aabb3<S>::Aabb3(const Aabb3& other) : m_min(other.m_min), m_max(other.m_max)
{
  // Do nothing
}

//==============================================================================
template <typename S>
Aabb3<S>::Aabb3(Aabb3&& other)
  : m_min(std::move(other.m_min)), m_max(std::move(other.m_max))
{
  // Do nothing
}

//==============================================================================
template <typename S>
void Aabb3<S>::set_random()
{
  const Vector3<S> random1 = Vector3<S>::Random();
  const Vector3<S> random2 = Vector3<S>::Random();

  m_min = random1.cwiseMin(random2);
  m_max = random1.cwiseMax(random2);
}

//==============================================================================
template <typename S>
void Aabb3<S>::reset()
{
  m_min.setConstant(common::max<S>());
  m_max.setConstant(common::lowest<S>());
}

//==============================================================================
template <typename S>
void Aabb3<S>::set_from_sphere(const Vector3<S>& center, S radius)
{
  m_min.array() = center.array() - radius;
  m_max.array() = center.array() + radius;
}

//==============================================================================
template <typename S>
void Aabb3<S>::expand(S delta)
{
  inflate(delta);
}

//==============================================================================
template <typename S>
Vector3<S> Aabb3<S>::get_center() const
{
  return (m_min + m_max) * 0.5;
}

//==============================================================================
template <typename S>
Vector3<S> Aabb3<S>::get_extent() const
{
  return (m_max - m_min);
}

//==============================================================================
template <typename S>
S Aabb3<S>::get_size() const
{
  return get_extent().squaredNorm();
}

//==============================================================================
template <typename S>
S Aabb3<S>::get_extent_x() const
{
  return m_max[0] - m_min[0];
}

//==============================================================================
template <typename S>
S Aabb3<S>::get_extent_y() const
{
  return m_max[1] - m_min[1];
}

//==============================================================================
template <typename S>
S Aabb3<S>::get_extentZ() const
{
  return m_max[2] - m_min[2];
}

//==============================================================================
template <typename S>
S Aabb3<S>::width() const
{
  return get_extent_x();
}

//==============================================================================
template <typename S>
S Aabb3<S>::height() const
{
  return get_extent_y();
}

//==============================================================================
template <typename S>
S Aabb3<S>::depth() const
{
  return get_extentZ();
}

//==============================================================================
template <typename S>
S Aabb3<S>::length(size_t axis)
{
  return m_max[axis] - m_min[axis];
}

//==============================================================================
template <typename S>
S Aabb3<S>::diagonal_length() const
{
  return (m_max - m_min).norm();
}

//==============================================================================
template <typename S>
S Aabb3<S>::radius() const
{
  return 0.5 * diagonal_length();
}

//==============================================================================
template <typename S>
S Aabb3<S>::diagonal_length_squared() const
{
  return (m_max - m_min).squaredNorm();
}

//==============================================================================
template <typename S>
S Aabb3<S>::size() const
{
  // TODO(JS): This is used to compare size of BV. Find a better way
  return diagonal_length_squared();
}

//==============================================================================
template <typename S>
S Aabb3<S>::get_volume() const
{
  const auto extent = get_extent();
  return (extent[0] * extent[1] * extent[2]);
}

//==============================================================================
template <typename S>
const Vector3<S>& Aabb3<S>::min() const
{
  return m_min;
}

//==============================================================================
template <typename S>
Vector3<S>& Aabb3<S>::min()
{
  return m_min;
}

//==============================================================================
template <typename S>
void Aabb3<S>::set_min(const Vector3<S>& min)
{
  m_min = min;
}

//==============================================================================
template <typename S>
const Vector3<S>& Aabb3<S>::max() const
{
  return m_max;
}

//==============================================================================
template <typename S>
Vector3<S>& Aabb3<S>::get_mutable_max()
{
  return m_max;
}

//==============================================================================
template <typename S>
void Aabb3<S>::setMax(const Vector3<S>& max)
{
  m_max = max;
}

//==============================================================================
template <typename S>
Vector3<S> Aabb3<S>::midPoint() const
{
  return 0.5 * (m_max + m_min);
}

//==============================================================================
template <typename S>
Vector3<S> Aabb3<S>::corner(size_t index) const
{
  static const S h = 0.5;
  static const Vector3<S> offset[8]
      = {{-h, -h, -h},
         {+h, -h, -h},
         {-h, +h, -h},
         {+h, +h, -h},
         {-h, -h, +h},
         {+h, -h, +h},
         {-h, +h, +h},
         {+h, +h, +h}};

  return Vector3<S>(width(), height(), depth()).array() * offset[index].array()
         + midPoint().array();
}

//==============================================================================
template <typename S>
void Aabb3<S>::setTransformed(const Aabb3& other, const Isometry3<S>& tf)
{
  m_min = tf * other.m_min;
  m_max = tf * other.m_max;
}

//==============================================================================
template <typename S>
void Aabb3<S>::setTranslated(const Aabb3& other, const Vector3<S>& trans)
{
  m_min = trans + other.m_min;
  m_max = trans + other.m_max;
}

//==============================================================================
template <typename S>
void Aabb3<S>::setRotated(const Aabb3& other, const Matrix3<S>& rot)
{
  m_min = rot * other.m_min;
  m_max = rot * other.m_max;
}

//==============================================================================
template <typename S>
void Aabb3<S>::inflate(S delta)
{
  m_min.array() -= delta;
  m_max.array() += delta;
}

//==============================================================================
template <typename S>
bool Aabb3<S>::overlaps(const Aabb3& other) const
{
  if ((m_min.array() > other.m_max.array()).any())
    return false;

  if ((m_max.array() < other.m_min.array()).any())
    return false;

  return true;
}

//==============================================================================
template <typename S>
bool Aabb3<S>::contains(const Aabb3& other) const
{
  if ((m_min.array() > other.m_min.array()).any())
    return false;

  if ((m_max.array() < other.m_max.array()).any())
    return false;

  return true;
}

//==============================================================================
template <typename S>
bool Aabb3<S>::contains(const Vector3<S>& point) const
{
  if ((m_min.array() > point.array()).any())
    return false;

  if ((m_max.array() < point.array()).any())
    return false;

  return true;
}

//==============================================================================
template <typename S>
AabbRayIntersection3<S> Aabb3<S>::closest_intersection(const Ray3<S>& ray) const
{
  AabbRayIntersection3<S> intersection;

  S tMin = 0;
  S tMax = std::numeric_limits<S>::max();
  const Vector3<S>& rayInvDir = ray.direction.cwiseInverse();

  for (int i = 0; i < 3; ++i) {
    S tNear = (m_min[i] - ray.origin[i]) * rayInvDir[i];
    S tFar = (m_max[i] - ray.origin[i]) * rayInvDir[i];

    if (tNear > tFar)
      std::swap(tNear, tFar);
    tMin = tNear > tMin ? tNear : tMin;
    tMax = tFar < tMax ? tFar : tMax;

    if (tMin > tMax) {
      intersection.isIntersecting = false;
      return intersection;
    }
  }

  intersection.isIntersecting = true;

  if (contains(ray.origin)) {
    intersection.tNear = tMax;
    intersection.tFar = common::max<S>();
  } else {
    intersection.tNear = tMin;
    intersection.tFar = tMax;
  }

  return intersection;
}

//==============================================================================
template <typename S>
bool Aabb3<S>::equals(const Aabb3& other) const
{
  return (m_min == other.m_min) && (m_max == other.m_max);
}

//==============================================================================
template <typename S>
bool Aabb3<S>::is_approx(const Aabb3& other, S tol) const
{
  return m_min.isApprox(other.m_min, tol) && m_max.isApprox(other.m_max, tol);
}

//==============================================================================
template <typename S>
void Aabb3<S>::merge(const Vector3<S>& point)
{
  m_min = m_min.cwiseMin(point);
  m_max = m_max.cwiseMax(point);
}

//==============================================================================
template <typename S>
void Aabb3<S>::merge(const Aabb3& other)
{
  m_min = m_min.cwiseMin(other.m_min);
  m_max = m_max.cwiseMax(other.m_max);
}

//==============================================================================
template <typename S>
void Aabb3<S>::merge(const Aabb3& aabb1, const Aabb3& aabb2)
{
  m_min = aabb1.m_min.cwiseMin(aabb2.m_min);
  m_max = aabb1.m_max.cwiseMax(aabb2.m_max);
}

//==============================================================================
template <typename S>
bool Aabb3<S>::overlaps(const Line3<S>& line) const
{
  // TODO: consider the following algorithms as well:
  // - "An Efficient and Robust Ray-Box Intersection Algorithm" by Amy Williams,
  //   Steve Barrus, R. Keith Morley, and Peter Shirley
  // - "Fast Ray-Axis-Aligned Bounding Box Overlap Tests using Ray Slopes" by
  //   Martin Eisemann and Marcus Magnor
  // - "New simple Ray-box test from Andrew Kensler" http://psgraphics.blogspot.
  //   com/2016/02/new-simple-Ray3<S>-box-test-from-andrew.html

  const Vector3<S> c = 0.5 * (m_min + m_max);
  const Vector3<S> e = m_max - c;
  Vector3<S> m = 0.5 * (line.from + line.to);
  const Vector3<S> d = line.to - m;
  m -= c;

  // Try world coordinate axes as separating axes
  auto adx = std::abs(d[0]);
  if (std::abs(m[0]) > e[0] + adx)
    return false;

  auto ady = std::abs(d[1]);
  if (std::abs(m[1]) > e[1] + ady)
    return false;

  auto adz = std::abs(d[2]);
  if (std::abs(m[2]) > e[2] + adz)
    return false;

  // Add in an epsilon term to counteract arithmetic errors when segment is
  // (near) parallel to a coordinate axis (see text for detail)
  const S epsilon = S(1e-5);
  adx += epsilon;
  ady += epsilon;
  adz += epsilon;

  // Try cross products of segment direction vector with coordinate axes
  if (std::abs(m[1] * d[2] - m[2] * d[1]) > e[1] * adz + e[2] * ady)
    return false;

  if (std::abs(m[2] * d[0] - m[0] * d[2]) > e[0] * adz + e[2] * adx)
    return false;

  if (std::abs(m[0] * d[1] - m[1] * d[0]) > e[0] * ady + e[1] * adx)
    return false;

  // No separating axis found; segment must be overlapping Aabb3
  return true;
}

//==============================================================================
template <typename S>
bool Aabb3<S>::overlaps(const Ray3<S>& ray) const
{
  S tMin = 0;
  S tMax = common::max<S>();
  const Vector3<S>& rayInvDir = ray.direction.cwiseInverse();

  for (int i = 0; i < 3; ++i) {
    S tNear = (m_min[i] - ray.origin[i]) * rayInvDir[i];
    S tFar = (m_max[i] - ray.origin[i]) * rayInvDir[i];

    if (tNear > tFar) {
      std::swap(tNear, tFar);
    }
    tMin = tNear > tMin ? tNear : tMin;
    tMax = tFar < tMax ? tFar : tMax;

    if (tMin > tMax) {
      return false;
    }
  }

  return true;
}

//==============================================================================
template <typename S>
Vector3<S> Aabb3<S>::clamp(const Vector3<S>& point) const
{
  return m_min.cwiseMax(point.cwiseMin(m_max));
}

//==============================================================================
template <typename S>
bool Aabb3<S>::isEmpty() const
{
  return (
      m_min.x() >= m_max.x() || m_min.y() >= m_max.y()
      || m_min.z() >= m_max.z());
}

//==============================================================================
template <typename S>
Aabb3<S> Aabb3<S>::Random()
{
  Aabb3 Aabb3;
  Aabb3.set_random();
  return Aabb3;
}

//==============================================================================
template <typename S>
Aabb3<S>& Aabb3<S>::operator=(const Aabb3& other)
{
  m_min = other.m_min;
  m_max = other.m_max;
  return *this;
}

//==============================================================================
template <typename S>
Aabb3<S>& Aabb3<S>::operator=(Aabb3&& other)
{
  m_min = std::move(other.m_min);
  m_max = std::move(other.m_max);
  return *this;
}

//==============================================================================
template <typename S>
bool Aabb3<S>::operator==(const Aabb3& other) const
{
  return equals(other);
}

//==============================================================================
template <typename S>
bool Aabb3<S>::operator!=(const Aabb3& other) const
{
  return !(*this == other);
}

//==============================================================================
template <typename S>
Aabb3<S>& Aabb3<S>::operator+=(const Aabb3& other)
{
  m_min = m_min.cwiseMin(other.m_min);
  m_max = m_max.cwiseMax(other.m_max);

  return *this;
}

//==============================================================================
template <typename S>
Aabb3<S> Aabb3<S>::operator+(const Aabb3& other) const
{
  Aabb3 res(*this);
  return res += other;
}

} // namespace dart::math
