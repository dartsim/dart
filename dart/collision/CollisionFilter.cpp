/*
 * Copyright (c) 2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016-2017, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016-2017, Personal Robotics Lab, Carnegie Mellon University
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
bool CollisionFilter::needCollision(
    const CollisionObject* object1, const CollisionObject* object2) const
{
  return !ignoresCollision(object1, object2);
}

//==============================================================================
void CompositeCollisionFilter::addCollisionFilter(CollisionFilter* filter)
{
  if (!filter)
    return;

  const auto result = std::find(mFilters.begin(), mFilters.end(), filter);
  const bool found = (result != mFilters.end());
  if (found)
    return;

  mFilters.push_back(filter);
}

//==============================================================================
void CompositeCollisionFilter::removeCollisionFilter(
    const CollisionFilter* filter)
{
  mFilters.erase(
        std::remove(mFilters.begin(), mFilters.end(), filter), mFilters.end());
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
    const bool collisionIgnored = filter->ignoresCollision(object1, object2);
    if (collisionIgnored)
      return true;
  }

  return false;
}

//==============================================================================
void BodyNodeCollisionFilter::addBodyNodePairToBlackList(
    const dynamics::BodyNode* bodyNode1, const dynamics::BodyNode* bodyNode2)
{
  if (!bodyNode1 || !bodyNode2)
    return;

  const auto* bodyNodeLess = bodyNode1;
  const auto* bodyNodeGreater = bodyNode2;

  if (bodyNodeLess > bodyNodeGreater)
    std::swap(bodyNodeLess, bodyNodeGreater);

  // Call insert in case an entry for bodyNodeLess doesn't exist. If it doesn't
  // exist, it will be initialized with an empty set. If it does already exist,
  // we will just get an iterator to the existing entry.
  const auto it_less = mBlackList.insert(
        std::make_pair(bodyNodeLess,
                       std::set<const dynamics::BodyNode*>())).first;

  // Insert bodyNodeGreater into the set corresponding to bodyNodeLess. If the
  // pair already existed, this will do nothing.
  it_less->second.insert(bodyNodeGreater);
}

//==============================================================================
void BodyNodeCollisionFilter::removeBodyNodePairFromBlackList(
    const dynamics::BodyNode* bodyNode1, const dynamics::BodyNode* bodyNode2)
{
  if (!bodyNode1 || !bodyNode2)
    return;

  const auto* bodyNodeLess = bodyNode1;
  const auto* bodyNodeGreater = bodyNode2;

  if (bodyNodeLess > bodyNodeGreater)
    std::swap(bodyNodeLess, bodyNodeGreater);

  // Remove the pair only when it already exists
  const auto resultLeft = mBlackList.find(bodyNodeLess);
  const bool foundLeft = (resultLeft != mBlackList.end());
  if (foundLeft)
  {
    auto& key = resultLeft->second;

    const auto resultRight = key.find(bodyNodeGreater);
    const bool foundRight = (resultRight != key.end());
    if (foundRight)
    {
      key.erase(bodyNodeGreater);

      if (key.empty())
        mBlackList.erase(resultLeft);
    }
  }
}

//==============================================================================
void BodyNodeCollisionFilter::removeAllBodyNodePairsFromBlackList()
{
  mBlackList.clear();
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

  if (!shapeNode1 || !shapeNode2)
    return false;
  // We assume that non-ShapeNodes are always being checked for collisions.

  auto bodyNode1 = shapeNode1->getBodyNodePtr();
  auto bodyNode2 = shapeNode2->getBodyNodePtr();

  if (bodyNode1 == bodyNode2)
    return true;

  if (!bodyNode1->isCollidable() || !bodyNode2->isCollidable())
    return true;

  if (bodyNode1->getSkeleton() == bodyNode2->getSkeleton())
  {
    auto skeleton = bodyNode1->getSkeleton();

    if (!skeleton->isEnabledSelfCollisionCheck())
      return true;

    if (!skeleton->isEnabledAdjacentBodyCheck())
    {
      if (areAdjacentBodies(bodyNode1, bodyNode2))
        return true;
    }
  }

  if (existsBodyNodePairInBlacklist(bodyNode1, bodyNode2))
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

//==============================================================================
bool BodyNodeCollisionFilter::existsBodyNodePairInBlacklist(
    const dynamics::BodyNode* bodyNode1,
    const dynamics::BodyNode* bodyNode2) const
{
  const auto* bodyNodeLess = bodyNode1;
  const auto* bodyNodeGreater = bodyNode2;

  if (bodyNodeLess > bodyNodeGreater)
    std::swap(bodyNodeLess, bodyNodeGreater);

  const auto resultLeft = mBlackList.find(bodyNodeLess);
  const bool foundLeft = (resultLeft != mBlackList.end());
  if (foundLeft)
  {
    auto& key = resultLeft->second;

    const auto resultRight = key.find(bodyNodeGreater);
    const bool foundRight = (resultRight != key.end());
    if (foundRight)
      return true;
  }

  return false;
}

} // namespace collision
} // namespace dart
