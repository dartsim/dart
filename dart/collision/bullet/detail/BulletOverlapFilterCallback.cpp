/*
 * Copyright (c) 2011-2017, The DART development contributors
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

#include "dart/collision/bullet/detail/BulletOverlapFilterCallback.hpp"

#include "dart/collision/CollisionFilter.hpp"
#include "dart/collision/bullet/BulletCollisionObject.hpp"

namespace dart {
namespace collision {
namespace detail {

//==============================================================================
BulletOverlapFilterCallback::BulletOverlapFilterCallback(
    const std::shared_ptr<CollisionFilter>& filter)
  : foundCollision(false),
    done(false),
    filter(filter)
{
  // Do nothing
}

//==============================================================================
bool BulletOverlapFilterCallback::needBroadphaseCollision(
    btBroadphaseProxy* proxy0, btBroadphaseProxy* proxy1) const
{
  if (done)
    return false;

  assert((proxy0 != nullptr && proxy1 != nullptr) &&
         "Bullet broadphase overlapping pair proxies are nullptr");

  const bool collide1
      = proxy0->m_collisionFilterGroup & proxy1->m_collisionFilterMask;
  const bool collide2
      = proxy1->m_collisionFilterGroup & proxy0->m_collisionFilterMask;

  bool collide = collide1 & collide2;

  if (collide && filter)
  {
    auto object0 = static_cast<btCollisionObject*>(proxy0->m_clientObject);
    auto object1 = static_cast<btCollisionObject*>(proxy1->m_clientObject);

    auto userPtr0 = object0->getUserPointer();
    auto userPtr1 = object1->getUserPointer();

    const auto collObj0 = static_cast<BulletCollisionObject*>(userPtr0);
    const auto collObj1 = static_cast<BulletCollisionObject*>(userPtr1);

    return !filter->ignoresCollision(collObj0, collObj1);
  }

  return collide;
}

}  // namespace detail
}  // namespace collision
}  // namespace dart
