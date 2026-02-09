/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
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

#include "dart/collision/collision_filter.hpp"

#include "dart/collision/collision_object.hpp"

namespace dart {
namespace collision {

//==============================================================================
CollisionFilter::~CollisionFilter()
{
  // Do nothing
}

//==============================================================================
void CompositeCollisionFilter::addCollisionFilter(const CollisionFilter* filter)
{
  // nullptr is not an allowed filter
  if (!filter) {
    return;
  }

  mFilters.insert(filter);
}

//==============================================================================
void CompositeCollisionFilter::removeCollisionFilter(
    const CollisionFilter* filter)
{
  mFilters.erase(filter);
}

//==============================================================================
void CompositeCollisionFilter::removeAllCollisionFilters()
{
  mFilters.clear();
}

//==============================================================================
bool CompositeCollisionFilter::ignoresCollision(
    const CollisionObject* object1, const CollisionObject* object2) const
{
  for (const auto* filter : mFilters) {
    if (filter->ignoresCollision(object1, object2)) {
      return true;
    }
  }

  return false;
}

} // namespace collision
} // namespace dart
