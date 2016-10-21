/*
 * Copyright (c) 2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
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

#include "dart/math/Aabb.hpp"

#include "dart/math/Constants.hpp"

namespace dart {
namespace math {

//==============================================================================
Aabb::Aabb()
  : mMin(Eigen::Vector3d::Constant(constantsd::inf())),
    mMax(Eigen::Vector3d::Constant(-constantsd::inf()))
{
  // Do nothing
}

//==============================================================================
Aabb::Aabb(const Eigen::Vector3d& min, const Eigen::Vector3d& max)
  : mMin(min), mMax(max)
{
  // Do nothing
}

//==============================================================================
void Aabb::setRandom()
{
  const Eigen::Vector3d random1 = Eigen::Vector3d::Random();
  const Eigen::Vector3d random2 = Eigen::Vector3d::Random();

  mMin = random1.cwiseMin(random2);
  mMax = random1.cwiseMax(random2);
}

//==============================================================================
Eigen::Vector3d Aabb::getCenter() const
{
  return (mMin + mMax) * 0.5;
}

//==============================================================================
Eigen::Vector3d Aabb::getExtent() const
{
  return (mMax - mMin);
}

//==============================================================================
double Aabb::getSize() const
{
  return getExtent().squaredNorm();
}

//==============================================================================
double Aabb::getVolume() const
{
  const auto extent = getExtent();
  return (extent[0] * extent[1] * extent[2]);
}

//==============================================================================
const Eigen::Vector3d& Aabb::getMin() const
{
  return mMin;
}

//==============================================================================
void Aabb::setMin(const Eigen::Vector3d& min)
{
  mMin = min;
}

//==============================================================================
const Eigen::Vector3d& Aabb::getMax() const
{
  return mMax;
}

//==============================================================================
void Aabb::setMax(const Eigen::Vector3d& max)
{
  mMax = max;
}

//==============================================================================
void Aabb::setTransformed(const Aabb& other, const Eigen::Isometry3d& tf)
{
  mMin = tf * other.mMin;
  mMax = tf * other.mMax;
}

//==============================================================================
void Aabb::setTranslated(const Aabb& other, const Eigen::Vector3d& trans)
{
  mMin = trans + other.mMin;
  mMax = trans + other.mMax;
}

//==============================================================================
void Aabb::setRotated(const Aabb& other, const Eigen::Matrix3d& rot)
{
  mMin = rot * other.mMin;
  mMax = rot * other.mMax;
}

//==============================================================================
void Aabb::inflate(double margin)
{
  const Eigen::Vector3d extension = Eigen::Vector3d::Constant(margin);

  mMin -= extension;
  mMax += extension;
}

//==============================================================================
bool Aabb::overlapsWith(const Aabb& other) const
{
  if ((mMin.array() > other.mMax.array()).any())
    return false;

  if ((mMax.array() < other.mMin.array()).any())
    return false;

  return true;
}

//==============================================================================
bool Aabb::contains(const Aabb& other) const
{
  if ((mMin.array() > other.mMin.array()).any())
    return false;

  if ((mMax.array() < other.mMax.array()).any())
    return false;

  return true;
}

//==============================================================================
bool Aabb::contains(const Eigen::Vector3d& point) const
{
  if ((mMin.array() > point.array()).any())
    return false;

  if ((mMax.array() < point.array()).any())
    return false;

  return true;
}

//==============================================================================
bool Aabb::equals(const Aabb& other) const
{
  return (mMin == other.mMin) && (mMax == other.mMax);
}

//==============================================================================
bool Aabb::almostEquals(const Aabb& other, double tol) const
{
  return mMin.isApprox(other.mMin, tol) && mMax.isApprox(other.mMax, tol);
}

//==============================================================================
void Aabb::mergeWith(const Eigen::Vector3d& point)
{
  mMin = mMin.cwiseMin(point);
  mMax = mMax.cwiseMax(point);
}

//==============================================================================
void Aabb::mergeWith(const Eigen::Vector3d& min, const Eigen::Vector3d& max)
{
  mMin = mMin.cwiseMin(min);
  mMax = mMax.cwiseMax(max);
}

//==============================================================================
void Aabb::mergeWith(const Aabb& other)
{
  mMin = mMin.cwiseMin(other.mMin);
  mMax = mMax.cwiseMax(other.mMax);
}

//==============================================================================
void Aabb::mergeWith(const Aabb& aabb1, const Aabb& aabb2)
{
  mMin = aabb1.mMin.cwiseMin(aabb2.mMin);
  mMax = aabb1.mMax.cwiseMax(aabb2.mMax);
}

//==============================================================================
bool Aabb::overlapsWith(const Ray& ray) const
{
  // TODO: consider following algorithms as well:
  // - "An Efficient and Robust Ray-Box Intersection Algorithm" by Amy Williams,
  //   Steve Barrus, R. Keith Morley, and Peter Shirley
  // - "Fast Ray-Axis-Aligned Bounding Box Overlap Tests using Ray Slopes" by
  //   Martin Eisemann and Marcus Magnor
  // - "New simple ray-box test from Andrew Kensler" http://psgraphics.blogspot.
  //   com/2016/02/new-simple-ray-box-test-from-andrew.html

  const Eigen::Vector3d point2 = ray.from + ray.fraction * ray.to;
  const Eigen::Vector3d e = mMax - mMin;
  const Eigen::Vector3d d = point2 - ray.from;
  const Eigen::Vector3d m = ray.from + point2 - mMin - mMax;

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
  const auto epsilon = double(1e-5);
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

  // No separating axis found; segment must be overlapping AABB
  return true;
}

//==============================================================================
Aabb Aabb::Random()
{
  Aabb aabb;
  aabb.setRandom();
  return aabb;
}

//==============================================================================
bool Aabb::operator==(const Aabb& other)
{
  return equals(other);
}

//==============================================================================
bool Aabb::operator!=(const Aabb& other)
{
  return !(*this == other);
}

//==============================================================================
Aabb& Aabb::operator+=(const Aabb& other)
{
  mMin = mMin.cwiseMin(other.mMin);
  mMax = mMax.cwiseMax(other.mMax);

  return *this;
}

//==============================================================================
Aabb Aabb::operator+(const Aabb& other) const
{
  Aabb res(*this);
  return res += other;
}

}  // namespace math
}  // namespace dart
