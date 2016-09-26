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

#ifndef DART_COLLISION_DISTANCE_OPTION_HPP_
#define DART_COLLISION_DISTANCE_OPTION_HPP_

#include <cstddef>
#include <memory>

namespace dart {
namespace collision {

class DistanceFilter;

struct DistanceOption
{
  /// Whether to calculate the nearest points.
  ///
  /// The default is false.
  /// \sa DistanceResult::nearestPoint1, DistanceResult::nearestPoint2
  bool enableNearestPoints;

  /// Stopping criteria for distance calculation in broadphase.
  ///
  /// This option is used for early termination of distance calculate that stops
  /// as soon as a distance is found that is equal to or less than the lower
  /// bound. If you want to check all the shape pairs without the early
  /// termination then set this value to -inf.
  ///
  /// The default value is 0.0.
  double distanceLowerBound;

  /// Distance filter for excluding ShapeFrame pairs from distance calculation
  /// in broadphase.
  ///
  /// If nullptr, every pairs of ShapeFrames in the CollisionGroup(s) are
  /// checked. The default is nullptr. \sa DistanceFilter
  std::shared_ptr<DistanceFilter> distanceFilter;

  /// Constructor
  DistanceOption(
      bool enableNearestPoints = false,
      double distanceLowerBound = 0.0,
      const std::shared_ptr<DistanceFilter>& distanceFilter = nullptr);
};

}  // namespace collision
}  // namespace dart

#endif  // DART_COLLISION_DISTANCE_OPTION_HPP_
