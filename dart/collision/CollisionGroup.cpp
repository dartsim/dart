/*
 * Copyright (c) 2016, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
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
CollisionGroup::~CollisionGroup()
{
  assert(mShapeFrames.empty());
  assert(mCollisionObjects.empty());
}

//==============================================================================
CollisionDetector* CollisionGroup::getCollisionDetector() const
{
  return mCollisionDetector.get();
}

//==============================================================================
void CollisionGroup::registerShapeFrame(const dynamics::ShapeFrame* shapeFrame)
{
  if (!shapeFrame)
    return;

  if (hasShapeFrame(shapeFrame))
    return;

  auto collObj = mCollisionDetector->claimCollisionObject(shapeFrame);

  notifyCollisionObjectAdded(collObj);

  mShapeFrames.push_back(shapeFrame);
  mCollisionObjects.push_back(collObj);
}

//==============================================================================
void CollisionGroup::registerShapeFrames(
    const std::vector<const dynamics::ShapeFrame*>& shapeFrames)
{
  for (const auto& shapeFrame : shapeFrames)
    registerShapeFrame(shapeFrame);
}

//==============================================================================
void CollisionGroup::registerShapeFrames()
{
  // Do nothing
}

//==============================================================================
void CollisionGroup::registerShapeFramesOf(
    const dynamics::ShapeFrame* shapeFrame)
{
  registerShapeFrame(shapeFrame);
}

//==============================================================================
void CollisionGroup::registerShapeFramesOf(
    const std::vector<const dynamics::ShapeFrame*>& shapeFrames)
{
  registerShapeFrames(shapeFrames);
}

//==============================================================================
void CollisionGroup::registerShapeFramesOf(const CollisionGroup* other)
{
  assert(other);

  if (other && this != other)
  {
    for (const auto& shapeFrame : other->mShapeFrames)
      registerShapeFrame(shapeFrame);
  }
}

//==============================================================================
void CollisionGroup::registerShapeFramesOf(const dynamics::BodyNode* bodyNode)
{
  assert(bodyNode);

  auto collisionShapeNodes
      = bodyNode->getShapeNodesWith<dynamics::CollisionAddon>();

  for (auto& shapeNode : collisionShapeNodes)
    registerShapeFrame(shapeNode);
}

//==============================================================================
void CollisionGroup::registerShapeFramesOf(const dynamics::Skeleton* skel)
{
  assert(skel);

  auto numBodyNodes = skel->getNumBodyNodes();
  for (auto i = 0u; i < numBodyNodes; ++i)
    registerShapeFramesOf(skel->getBodyNode(i));
}

//==============================================================================
void CollisionGroup::unregisterShapeFrame(
    const dynamics::ShapeFrame* shapeFrame)
{
  if (!shapeFrame)
    return;

  const auto search
      = std::find_if(mShapeFrames.begin(), mShapeFrames.end(),
                     [&](const dynamics::ShapeFrame* it)
                     { return it == shapeFrame; });

  if (mShapeFrames.end() == search)
    return;

  const size_t index = search - mShapeFrames.begin();

  notifyCollisionObjectRemoved(mCollisionObjects[index]);

  mCollisionDetector->reclaimCollisionObject(mCollisionObjects[index]);

  mShapeFrames.erase(search);
  mCollisionObjects.erase(mCollisionObjects.begin() + index);
}

//==============================================================================
void CollisionGroup::unregisterShapeFrames(
    const std::vector<const dynamics::ShapeFrame*>& shapeFrames)
{
  for (const auto& shapeFrame : shapeFrames)
    unregisterShapeFrame(shapeFrame);
}

//==============================================================================
void CollisionGroup::unregisterShapeFramesOf()
{
  // Do nothing
}

//==============================================================================
void CollisionGroup::unregisterShapeFramesOf(
    const dynamics::ShapeFrame* shapeFrame)
{
  unregisterShapeFrame(shapeFrame);
}

//==============================================================================
void CollisionGroup::unregisterShapeFramesOf(
    const std::vector<const dynamics::ShapeFrame*>& shapeFrames)
{
  unregisterShapeFrames(shapeFrames);
}

//==============================================================================
void CollisionGroup::unregisterShapeFramesOf(const CollisionGroup* other)
{
  assert(other);

  if (other)
  {
    if (this == other)
    {
      unregisterAllShapeFrames();
      return;
    }

    for (const auto& shapeFrame : other->mShapeFrames)
      unregisterShapeFrame(shapeFrame);
  }
}

//==============================================================================
void CollisionGroup::unregisterShapeFramesOf(
    const dynamics::BodyNode* bodyNode)
{
  assert(bodyNode);

  auto collisionShapeNodes
      = bodyNode->getShapeNodesWith<dynamics::CollisionAddon>();

  for (auto& shapeNode : collisionShapeNodes)
    unregisterShapeFrame(shapeNode);
}

//==============================================================================
void CollisionGroup::unregisterShapeFramesOf(const dynamics::Skeleton* skel)
{
  assert(skel);

  auto numBodyNodes = skel->getNumBodyNodes();
  for (auto i = 0u; i < numBodyNodes; ++i)
    unregisterShapeFramesOf(skel->getBodyNode(i));
}

//==============================================================================
void CollisionGroup::unregisterAllShapeFrames()
{
  notifyAllCollisionObjectsRemoved();

  for (const auto& object : mCollisionObjects)
    mCollisionDetector->reclaimCollisionObject(object);

  mShapeFrames.clear();
  mCollisionObjects.clear();
}

//==============================================================================
bool CollisionGroup::hasShapeFrame(
    const dynamics::ShapeFrame* shapeFrame) const
{
  return std::find_if(
        mShapeFrames.begin(), mShapeFrames.end(),
        [&](const dynamics::ShapeFrame* it) { return it == shapeFrame; })
      != mShapeFrames.end();
}

//==============================================================================
size_t CollisionGroup::getNumShapeFrames() const
{
  return mShapeFrames.size();
}

//==============================================================================
const dynamics::ShapeFrame* CollisionGroup::getShapeFrame(size_t index) const
{
  return common::getVectorObjectIfAvailable(index, mShapeFrames);
}

//==============================================================================
const std::vector<const dynamics::ShapeFrame*>
CollisionGroup::getShapeFrames() const
{
  return mShapeFrames;
}

//==============================================================================
void CollisionGroup::update()
{
  for (auto& object : mCollisionObjects)
    object->updateEngineData();

  updateEngineData();
}

//==============================================================================
bool CollisionGroup::detect(const Option& option, Result& result)
{
  return mCollisionDetector->detect(this, option, result);
}

//==============================================================================
bool CollisionGroup::detect(
    CollisionGroup* other, const Option& option, Result& result)
{
  return mCollisionDetector->detect(this, other, option, result);
}

//==============================================================================
const std::vector<CollisionObject*>& CollisionGroup::getCollisionObjects()
{
  return mCollisionObjects;
}

}  // namespace collision
}  // namespace dart
