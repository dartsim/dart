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

#ifndef DART_MATH_AABB_HPP_
#define DART_MATH_AABB_HPP_

#include <Eigen/Dense>

#include "dart/common/Deprecated.hpp"
#include "dart/math/Ray.hpp"

namespace dart {
namespace math {

/// Aabb encodes axis-aligned bounding box represented by minimum and maximum
/// coordinate values along each axis. The bounding volume region of space is
/// specified as the box of the two opposing corner points: min and max.
class Aabb final
{
public:
  /// Constructor
  Aabb();

  /// Constructor
  Aabb(const Eigen::Vector3d& min, const Eigen::Vector3d& max);

  /// Destructor
  ~Aabb() = default;

  void setRandom();

  /// Return the minimum coordinates of the Aabb.
  const Eigen::Vector3d& getMin() const;

  /// Set the minimum coordinates of the Aabb.
  void setMin(const Eigen::Vector3d& min);

  /// Return the maximum coordinates of the Aabb.
  const Eigen::Vector3d& getMax() const;

  /// Set the maximum coordinates of the Aabb.
  void setMax(const Eigen::Vector3d& max);

  void setTransformed(const Aabb& other, const Eigen::Isometry3d& tf);

  void setTranslated(const Aabb& other, const Eigen::Vector3d& trans);

  void setRotated(const Aabb& other, const Eigen::Matrix3d& rot);

  void inflate(double margin);

  /// Return the center point.
  Eigen::Vector3d getCenter() const;

  DART_DEPRECATED(6.2)
  Eigen::Vector3d computeCenter() const { return getCenter(); }

  DART_DEPRECATED(6.2)
  Eigen::Vector3d computeHalfExtents() const { return 0.5 * getExtent(); }

  DART_DEPRECATED(6.2)
  Eigen::Vector3d computeFullExtents() const { return getExtent(); }

  Eigen::Vector3d getExtent() const;

  double getVolume() const;

  double getSize() const;

  /// Return true if this Aabb overlaps with other Aabb.
  bool overlapsWith(const Aabb& other) const;

  /// Return true if this Aabb intersects with a ray.
  ///
  /// Implementation of RTCD 5.3.3.
  bool overlapsWith(const Ray& ray) const;

  /// Return true if this Aabb contains other Aabb or they have the same
  /// size.
  bool contains(const Aabb& other) const;

  /// Return true if this Aabb contains a point
  bool contains(const Eigen::Vector3d& point) const;

  bool equals(const Aabb& other) const;

  bool almostEquals(const Aabb& other, double tol = 1e-6) const;

  void mergeWith(const Eigen::Vector3d& point);

  void mergeWith(const Eigen::Vector3d& min, const Eigen::Vector3d& max);

  void mergeWith(const Aabb& other);

  void mergeWith(const Aabb& aabb1, const Aabb& aabb2);

  //void intersectWith(const Aabb& other);
  // TODO(JS): implement

  static Aabb Random();

  bool operator==(const Aabb& other);
  bool operator!=(const Aabb& other);

  Aabb& operator+=(const Aabb& other);

  Aabb operator+(const Aabb& other) const;

protected:
  /// Minimum world coordinates of the Aabb.
  Eigen::Vector3d mMin;

  /// Maximum world coordinates of the Aabb.
  Eigen::Vector3d mMax;
};

using BoundingBox = Aabb;
// TODO(JS): deprecate BoundingBox

}  // namespace math
}  // namespace dart

#endif  // DART_MATH_AABB_HPP_
