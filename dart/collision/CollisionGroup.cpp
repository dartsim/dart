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

#include "dart/collision/CollisionGroup.hpp"

#include <cassert>

#include "dart/collision/CollisionObject.hpp"
#include "dart/collision/CollisionDetector.hpp"
#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/Skeleton.hpp"

namespace dart {
namespace collision {

//==============================================================================
CollisionGroup::CollisionGroup(const CollisionDetectorPtr& collisionDetector)
  : mCollisionDetector(collisionDetector)
{
  assert(mCollisionDetector);
}

//==============================================================================
CollisionDetectorPtr CollisionGroup::getCollisionDetector()
{
  return mCollisionDetector;
}

//==============================================================================
ConstCollisionDetectorPtr CollisionGroup::getCollisionDetector() const
{
  return mCollisionDetector;
}

//==============================================================================
void CollisionGroup::addShapeFrame(const dynamics::ShapeFrame* shapeFrame)
{
  if (!shapeFrame)
    return;

  if (hasShapeFrame(shapeFrame))
    return;

  auto collObj = mCollisionDetector->claimCollisionObject(shapeFrame);

  addCollisionObjectToEngine(collObj.get());

  mShapeFrameMap.push_back(std::make_pair(shapeFrame, collObj));
}

//==============================================================================
void CollisionGroup::addShapeFrames(
    const std::vector<const dynamics::ShapeFrame*>& shapeFrames)
{
  for (const auto& shapeFrame : shapeFrames)
    addShapeFrame(shapeFrame);
}

//==============================================================================
void CollisionGroup::addShapeFramesOf()
{
  // Do nothing
}

//==============================================================================
void CollisionGroup::removeShapeFrame(const dynamics::ShapeFrame* shapeFrame)
{
  if (!shapeFrame)
    return;

  const auto search
      = std::find_if(mShapeFrameMap.begin(), mShapeFrameMap.end(),
                     [&](const std::pair<const dynamics::ShapeFrame*,
                                         CollisionObjectPtr>& pair)
                     { return pair.first == shapeFrame; });

  if (mShapeFrameMap.end() == search)
    return;

  removeCollisionObjectFromEngine(search->second.get());

  mShapeFrameMap.erase(search);
}

//==============================================================================
void CollisionGroup::removeShapeFrames(
    const std::vector<const dynamics::ShapeFrame*>& shapeFrames)
{
  for (const auto& shapeFrame : shapeFrames)
    removeShapeFrame(shapeFrame);
}

//==============================================================================
void CollisionGroup::removeShapeFramesOf()
{
  // Do nothing
}

//==============================================================================
void CollisionGroup::removeAllShapeFrames()
{
  removeAllCollisionObjectsFromEngine();

  mShapeFrameMap.clear();
}

//==============================================================================
bool CollisionGroup::hasShapeFrame(
    const dynamics::ShapeFrame* shapeFrame) const
{
  return std::find_if(mShapeFrameMap.begin(), mShapeFrameMap.end(),
                      [&](const std::pair<const dynamics::ShapeFrame*,
                                          CollisionObjectPtr>& pair)
                      { return pair.first == shapeFrame; })
      != mShapeFrameMap.end();
}

//==============================================================================
std::size_t CollisionGroup::getNumShapeFrames() const
{
  return mShapeFrameMap.size();
}

//==============================================================================
const dynamics::ShapeFrame* CollisionGroup::getShapeFrame(
    std::size_t index) const
{
  assert(index < mShapeFrameMap.size());
  if(index < mShapeFrameMap.size())
    return mShapeFrameMap[index].first;

  return nullptr;
}

//==============================================================================
bool CollisionGroup::collide(
    const CollisionOption& option, CollisionResult* result)
{
  return mCollisionDetector->collide(this, option, result);
}

//==============================================================================
bool CollisionGroup::collide(
    CollisionGroup* otherGroup,
    const CollisionOption& option,
    CollisionResult* result)
{
  return mCollisionDetector->collide(this, otherGroup, option, result);
}

//==============================================================================
double CollisionGroup::distance(
    const DistanceOption& option, DistanceResult* result)
{
  return mCollisionDetector->distance(this, option, result);
}

//==============================================================================
double CollisionGroup::distance(
    CollisionGroup* otherGroup,
    const DistanceOption& option,
    DistanceResult* result)
{
  return mCollisionDetector->distance(this, otherGroup, option, result);
}

//==============================================================================
void CollisionGroup::updateEngineData()
{
  for (auto& pair : mShapeFrameMap)
    pair.second->updateEngineData();

  updateCollisionGroupEngineData();
}

}  // namespace collision
}  // namespace dart
