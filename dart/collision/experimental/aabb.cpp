/*
 * Copyright (c) 2011, The DART development contributors
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

#include <dart/collision/experimental/aabb.hpp>

#include <algorithm>

#include <cmath>

namespace dart::collision::experimental {

Aabb::Aabb() : min(Eigen::Vector3d::Zero()), max(Eigen::Vector3d::Zero()) {}

Aabb::Aabb(const Eigen::Vector3d& min, const Eigen::Vector3d& max)
  : min(min), max(max)
{
}

bool Aabb::overlaps(const Aabb& other) const
{
  return (min.x() <= other.max.x() && max.x() >= other.min.x())
         && (min.y() <= other.max.y() && max.y() >= other.min.y())
         && (min.z() <= other.max.z() && max.z() >= other.min.z());
}

bool Aabb::contains(const Eigen::Vector3d& point) const
{
  return (point.x() >= min.x() && point.x() <= max.x())
         && (point.y() >= min.y() && point.y() <= max.y())
         && (point.z() >= min.z() && point.z() <= max.z());
}

bool Aabb::contains(const Aabb& other) const
{
  return (other.min.x() >= min.x() && other.max.x() <= max.x())
         && (other.min.y() >= min.y() && other.max.y() <= max.y())
         && (other.min.z() >= min.z() && other.max.z() <= max.z());
}

Eigen::Vector3d Aabb::center() const
{
  return (min + max) * 0.5;
}

Eigen::Vector3d Aabb::halfExtents() const
{
  return (max - min) * 0.5;
}

Eigen::Vector3d Aabb::extents() const
{
  return max - min;
}

double Aabb::volume() const
{
  const Eigen::Vector3d e = extents();
  return e.x() * e.y() * e.z();
}

void Aabb::merge(const Aabb& other)
{
  min = min.cwiseMin(other.min);
  max = max.cwiseMax(other.max);
}

void Aabb::expand(double margin)
{
  const Eigen::Vector3d m(margin, margin, margin);
  min -= m;
  max += m;
}

Aabb Aabb::forSphere(double radius)
{
  const Eigen::Vector3d r(radius, radius, radius);
  return Aabb(-r, r);
}

Aabb Aabb::forBox(const Eigen::Vector3d& halfExtents)
{
  return Aabb(-halfExtents, halfExtents);
}

Aabb Aabb::forCapsule(double radius, double height)
{
  const double halfHeight = height * 0.5 + radius;
  return Aabb(
      Eigen::Vector3d(-radius, -radius, -halfHeight),
      Eigen::Vector3d(radius, radius, halfHeight));
}

Aabb Aabb::forCylinder(double radius, double height)
{
  const double halfHeight = height * 0.5;
  return Aabb(
      Eigen::Vector3d(-radius, -radius, -halfHeight),
      Eigen::Vector3d(radius, radius, halfHeight));
}

Aabb Aabb::transformed(const Aabb& local, const Eigen::Isometry3d& transform)
{
  const Eigen::Vector3d localCenter = local.center();
  const Eigen::Vector3d localHalfExtents = local.halfExtents();

  const Eigen::Vector3d worldCenter = transform * localCenter;

  const Eigen::Matrix3d absRotation = transform.rotation().cwiseAbs();
  const Eigen::Vector3d worldHalfExtents = absRotation * localHalfExtents;

  return Aabb(worldCenter - worldHalfExtents, worldCenter + worldHalfExtents);
}

} // namespace dart::collision::experimental
