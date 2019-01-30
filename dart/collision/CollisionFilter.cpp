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

#include "dart/collision/CollisionFilter.hpp"

#include "dart/dynamics/BodyNode.hpp"
#include "dart/collision/CollisionObject.hpp"

namespace dart {
namespace collision {

//==============================================================================
bool CollisionFilter::needCollision(
    const CollisionObject* object1, const CollisionObject* object2) const
{
  return !ignoresCollision(object1, object2);
}

//==============================================================================
void CompositeCollisionFilter::addCollisionFilter(const CollisionFilter* filter)
{
  // nullptr is not an allowed filter
  if (!filter)
    return;

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
  for (const auto* filter : mFilters)
  {
    if (filter->ignoresCollision(object1, object2))
      return true;
  }

  return false;
}

//==============================================================================
void BodyNodeCollisionFilter::addBodyNodePairToBlackList(
    const dynamics::BodyNode* bodyNode1, const dynamics::BodyNode* bodyNode2)
{
  mBodyNodeBlackList.addPair(bodyNode1, bodyNode2);
}

//==============================================================================
void BodyNodeCollisionFilter::removeBodyNodePairFromBlackList(
    const dynamics::BodyNode* bodyNode1, const dynamics::BodyNode* bodyNode2)
{
  mBodyNodeBlackList.removePair(bodyNode1, bodyNode2);
}

//==============================================================================
void BodyNodeCollisionFilter::removeAllBodyNodePairsFromBlackList()
{
  mBodyNodeBlackList.removeAllPairs();
}

//==============================================================================
bool BodyNodeCollisionFilter::ignoresCollision(
    const collision::CollisionObject* object1,
    const collision::CollisionObject* object2) const
{
  if (object1 == object2)
    return true;

  auto shapeNode1 = object1->getShapeFrame()->asShapeNode();
  auto shapeNode2 = object2->getShapeFrame()->asShapeNode();

  // We don't filter out for non-ShapeNode because this class shouldn't have the
  // authority to make decisions about filtering any ShapeFrames that aren't
  // attached to a BodyNode. So here we just return false. In order to decide
  // whether the non-ShapeNode should be ignored, please use other collision
  // filters.
  if (!shapeNode1 || !shapeNode2)
    return false;

  auto bodyNode1 = shapeNode1->getBodyNodePtr();
  auto bodyNode2 = shapeNode2->getBodyNodePtr();

  if (bodyNode1 == bodyNode2)
    return true;

  if (!bodyNode1->isCollidable() || !bodyNode2->isCollidable())
    return true;

  const auto& skel1 = bodyNode1->getSkeleton();
  const auto& skel2 = bodyNode2->getSkeleton();

  if ( !skel1->isMobile() && !skel2->isMobile() )
    return true;

  if (skel1 == skel2)
  {
    if (!skel1->isEnabledSelfCollisionCheck())
      return true;

    if (!skel1->isEnabledAdjacentBodyCheck())
    {
      if (areAdjacentBodies(bodyNode1, bodyNode2))
        return true;
    }
  }

  if (mBodyNodeBlackList.contains(bodyNode1, bodyNode2))
    return true;

  return false;
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

} // namespace collision
} // namespace dart
