/*
 * Copyright (c) 2011-2019, The DART development contributors
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
  /// Minimum \b singed distance between the checked Shape pairs.
  ///
  /// If no shape pair was checked (the collision group was empty or all pairs
  /// were excluded by the filter) then this value will be remained the default
  /// value 0.0. You can check if a valid distance was found using
  /// DistanceResult::found() that returns true when it's found.
  ///
  /// The result minimum distance is clamped by
  /// DistanceOption::distanceLowerBound
  /// (min. distance >= distance lower bound). You can still get the unclamped
  /// distance from DistanceResult::unclampedMinDistance.
  ///
  /// If the minimum distance is negative value, that means the distance is
  /// the negative penetration depth measured from the shapes that are in
  /// collision.
  double minDistance;

  /// Unclamped minimum distance that is the first distance equal to or less
  /// than DistanceOption::distanceLowerBound.
  ///
  /// \warning This value is implementation defined, which means the value could
  /// be different depending on various factors (e.g., the collision detector
  /// and even the version of the collision detector), and we don't intend this
  /// value to be (reasonably) indentical over thoes factors. This valude is for
  /// debugging purpose or advanced use.
  double unclampedMinDistance;

  /// First ShapeFrame of the minDistance
  ///
  /// If no shape pair was checked then shapeFrame1 will be nullptr.
  const dynamics::ShapeFrame* shapeFrame1;

  /// Second ShapeFrame of the minDistance
  ///
  /// If no shape pair was checked then shapeFrame2 will be nullptr.
  const dynamics::ShapeFrame* shapeFrame2;

  /// The nearest point on DistanceResult::shapeFrame1 expressed in the world
  /// coordinates.
  ///
  /// The point is calculated only when DistanceOption::enableNearestPoints is
  /// true.
  ///
  /// The distance between the two nearest points is equal to
  /// unclampedMinDistance rather than minDistance.
  Eigen::Vector3d nearestPoint1;

  /// The nearest point on DistanceResult::shapeFrame2 expressed the world
  /// coordinates.
  ///
  /// The point is calculated only when DistanceOption::enableNearestPoints is
  /// true.
  ///
  /// The distance between the two nearest points is equal to
  /// unclampedMinDistance rather than minDistance.
  Eigen::Vector3d nearestPoint2;

  /// Constructor
  DistanceResult();

  /// Clear the result
  void clear();

  /// Get true if the result is valid (at least one shape pair was checked
  /// and the result values are good to use).
  bool found() const;

  /// Get true if the minimum distance was not clamped
  /// (minDistance == unclampedMinDistance).
  bool isMinDistanceClamped() const;
};

}  // namespace collision
}  // namespace dart

#endif  // DART_COLLISION_DISTANCE_RESULT_HPP_
