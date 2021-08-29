/*
 * Copyright (c) 2011-2021, The DART development contributors
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

#ifndef DART_COLLISION_RAYCASTRESULT_HPP_
#define DART_COLLISION_RAYCASTRESULT_HPP_

#include <vector>

#include <Eigen/Dense>

namespace dart {
namespace collision {

class CollisionObject;

struct RayHit
{
  /// The collision object the ray hit
  const CollisionObject* mCollisionObject;

  /// The hit point in the world coordinates
  Eigen::Vector3d mPoint;

  /// The fraction from "from" point to "to" point
  double mFraction;

  /// The normal at the hit point in the world coordinates
  Eigen::Vector3d mNormal;

  /// Constructor
  RayHit();
};

struct RaycastResult
{
  /// Clear the result
  void clear();

  /// Returns true if there is a hit
  bool hasHit() const;

  std::vector<RayHit> mRayHits;

  /// Constructor
  RaycastResult();
};

} // namespace collision
} // namespace dart

#endif // DART_COLLISION_RAYCASTRESULT_HPP_
