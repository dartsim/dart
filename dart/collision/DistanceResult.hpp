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

#ifndef DART_COLLISION_DISTANCE_RESULT_HPP_
#define DART_COLLISION_DISTANCE_RESULT_HPP_

#include <Eigen/Dense>

namespace dart {

namespace dynamics {
class ShapeFrame;
} // namesapce dynamics

namespace collision {

struct DistanceResult
{
  /// Minimum "singed" distance between two Shapes. If the two Shapes are in
  /// collision, the distance is negative.
  ///
  /// The minimum signed distance meaning can be varied depending on the
  /// DistanceOption::minimumDistance. Please see
  /// DistanceOption::minimumDistanceThreshold for further details.
  double minimumDistance;

  /// First ShapeFrame
  const dynamics::ShapeFrame* shapeFrame1;

  /// Second ShapeFrame
  const dynamics::ShapeFrame* shapeFrame2;

  /// The nearest point on shapeFrame1. The point is calculated only when
  /// DistanceOption::enableNearestPoints is true, which is false by default.
  Eigen::Vector3d nearestPoint1;

  /// The nearest point on shapeFrame2. The point is calculated only when
  /// DistanceOption::enableNearestPoints is true, which is false by default.
  Eigen::Vector3d nearestPoint2;

  /// Constructor
  DistanceResult();

  /// Clear (initialize) the result
  void clear();

  /// Return true if the result is valid
  bool found() const;
};

}  // namespace collision
}  // namespace dart

#endif  // DART_COLLISION_DISTANCE_RESULT_HPP_
