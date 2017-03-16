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

#include "dart/collision/bullet/BulletCollisionDispatcher.hpp"

#include "dart/collision/bullet/BulletCollisionObject.hpp"

namespace dart {
namespace collision {

//==============================================================================
BulletCustomCollisionDispatcher::BulletCustomCollisionDispatcher(
    btCollisionConfiguration* collisionConfiguration)
  : btCollisionDispatcher(collisionConfiguration),
    mDone(false),
    mFilter(nullptr)
{
  // Do nothing
}

//==============================================================================
void BulletCustomCollisionDispatcher::setDone(bool done)
{
  mDone = done;
}

//==============================================================================
void BulletCustomCollisionDispatcher::setFilter(
    const std::shared_ptr<CollisionFilter>& filter)
{
  mFilter = filter;
}

//==============================================================================
std::shared_ptr<CollisionFilter>
BulletCustomCollisionDispatcher::getFilter() const
{
  return mFilter;
}

//==============================================================================
bool BulletCustomCollisionDispatcher::needsCollision(
    const btCollisionObject* body0, const btCollisionObject* body1)
{
  const auto userData0 = static_cast<BulletCollisionObject::UserData*>(
        body0->getUserPointer());
  const auto userData1 = static_cast<BulletCollisionObject::UserData*>(
        body1->getUserPointer());

  if (mFilter && !mFilter->needCollision(userData0->collisionObject,
                                         userData1->collisionObject))
  {
    return false;
  }

  return btCollisionDispatcher::needsCollision(body0, body1);
}

}  // namespace collision
}  // namespace dart
