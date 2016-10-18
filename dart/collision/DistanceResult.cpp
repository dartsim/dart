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

#include "dart/collision/DistanceResult.hpp"

#define DART_DEFAULT_MIN_DISTANCE (0.0)
#define DART_DEFAULT_UNCLAMPED_MIN_DISTANCE (0.0)

namespace dart {
namespace collision {

//==============================================================================
DistanceResult::DistanceResult()
  : minDistance(DART_DEFAULT_MIN_DISTANCE),
    unclampedMinDistance(0.0),
    shapeFrame1(nullptr),
    shapeFrame2(nullptr),
    nearestPoint1(Eigen::Vector3d::Zero()),
    nearestPoint2(Eigen::Vector3d::Zero())
{
  // Do nothing
}

//==============================================================================
void DistanceResult::clear()
{
  minDistance = DART_DEFAULT_MIN_DISTANCE;
  unclampedMinDistance = DART_DEFAULT_UNCLAMPED_MIN_DISTANCE;

  nearestPoint1.setZero();
  nearestPoint2.setZero();

  shapeFrame1 = nullptr;
  shapeFrame2 = nullptr;
}

//==============================================================================
bool DistanceResult::found() const
{
  if (!shapeFrame1 || !shapeFrame2)
    return false;

  return true;
}

//==============================================================================
bool DistanceResult::isMinDistanceClamped() const
{
  return found() && (minDistance == unclampedMinDistance);
}

}  // namespace collision
}  // namespace dart
