/*
 * Copyright (c) 2017, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2017, Personal Robotics Lab, Carnegie Mellon University
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

#ifndef DART_COLLISION_BULLET_DETAIL_BULLETOVERLAPFILTERCALLBACK_HPP_
#define DART_COLLISION_BULLET_DETAIL_BULLETOVERLAPFILTERCALLBACK_HPP_

// Must be included before any Bullet headers.
#include "dart/config.hpp"

#include <btBulletCollisionCommon.h>

#include "dart/collision/CollisionOption.hpp"
#include "dart/collision/CollisionResult.hpp"

namespace dart {
namespace collision {
namespace detail {

struct BulletOverlapFilterCallback : public btOverlapFilterCallback
{
  // Constructor
  explicit BulletOverlapFilterCallback(
      const std::shared_ptr<CollisionFilter>& filter = nullptr);

  /// Returns true when pairs need collision
  bool needBroadphaseCollision(
      btBroadphaseProxy* proxy0, btBroadphaseProxy* proxy1) const override;

  /// True if at least one contact is found. This flag is used only when
  /// mResult is nullptr; otherwise the actual collision result is in mResult.
  bool foundCollision;

  /// Whether the collision iteration can stop
  mutable bool done;

  std::shared_ptr<CollisionFilter> filter;
};

}  // namespace detail
}  // namespace collision
}  // namespace dart

#endif  // DART_COLLISION_BULLET_DETAIL_BULLETOVERLAPFILTERCALLBACK_HPP_
