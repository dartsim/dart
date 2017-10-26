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

#include "dart/collision/bullet/detail/BulletCollisionDispatcher.hpp"

#include "dart/collision/bullet/BulletCollisionObject.hpp"

namespace dart {
namespace collision {
namespace detail {

//==============================================================================
BulletCollisionDispatcher::BulletCollisionDispatcher(
    btCollisionConfiguration* config)
  : btCollisionDispatcher(config),
    mDone(false),
    mFilter(nullptr)
{
  // Do nothing
}

//==============================================================================
void BulletCollisionDispatcher::setDone(bool done)
{
  mDone = done;
}

//==============================================================================
void BulletCollisionDispatcher::setFilter(
    const std::shared_ptr<CollisionFilter>& filter)
{
  mFilter = filter;
}

//==============================================================================
auto BulletCollisionDispatcher::getFilter() const
-> std::shared_ptr<CollisionFilter>
{
  return mFilter;
}

//==============================================================================
bool BulletCollisionDispatcher::needsCollision(
    const btCollisionObject* body0, const btCollisionObject* body1)
{
  if (mDone)
    return false;

  const auto collObj0
      = static_cast<BulletCollisionObject*>(body0->getUserPointer());
  const auto collObj1
      = static_cast<BulletCollisionObject*>(body1->getUserPointer());

  if (mFilter && mFilter->ignoresCollision(collObj0, collObj1))
    return false;

  return btCollisionDispatcher::needsCollision(body0, body1);
}

}  // namespace detail
}  // namespace collision
}  // namespace dart
