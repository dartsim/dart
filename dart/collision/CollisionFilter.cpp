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

#include "dart/collision/CollisionFilter.hpp"

#include "dart/dynamics/BodyNode.hpp"
#include "dart/collision/CollisionObject.hpp"

namespace dart {
namespace collision {

//==============================================================================
bool BodyNodeCollisionFilter::needCollision(
    const collision::CollisionObject* object1,
    const collision::CollisionObject* object2) const
{
  if (object1 == object2)
    return false;

  auto shapeNode1 = object1->getShapeFrame()->asShapeNode();
  auto shapeNode2 = object2->getShapeFrame()->asShapeNode();

  if (!shapeNode1 || !shapeNode2)
    return true;
  // We assume that non-ShapeNode is always being checked collision.

  auto bodyNode1 = shapeNode1->getBodyNodePtr();
  auto bodyNode2 = shapeNode2->getBodyNodePtr();

  if (!bodyNode1->isCollidable() || !bodyNode2->isCollidable())
    return false;

  if (bodyNode1->getSkeleton() == bodyNode2->getSkeleton())
  {
    auto skeleton = bodyNode1->getSkeleton();

    if (!skeleton->isEnabledSelfCollisionCheck())
      return false;

    if (!skeleton->isEnabledAdjacentBodyCheck())
    {
      if (areAdjacentBodies(bodyNode1, bodyNode2))
        return false;
    }
  }

  return true;
}

//==============================================================================
bool BodyNodeCollisionFilter::areAdjacentBodies(
    const dynamics::BodyNode* bodyNode1,
    const dynamics::BodyNode* bodyNode2) const
{
  if ((bodyNode1->getParentBodyNode() == bodyNode2)
      || (bodyNode2->getParentBodyNode() == bodyNode1))
  {
    assert(bodyNode1->getSkeleton() == bodyNode2->getSkeleton());
    return true;
  }

  return false;
}

}  // namespace collision
}  // namespace dart
