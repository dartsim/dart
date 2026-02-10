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

#include "dart/collision/collision_group.hpp"

#include "dart/collision/collision_detector.hpp"
#include "dart/collision/collision_object.hpp"
#include "dart/common/macros.hpp"
#include "dart/dynamics/shape_frame.hpp"

#include <algorithm>

#include <cassert>
#include <cstdint>

namespace dart {
namespace collision {

//==============================================================================
CollisionGroup::ShapeFrameObserver::ShapeFrameObserver(CollisionGroup* group)
  : mGroup(group)
{
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
  addShapeFrameImpl(shapeFrame, nullptr);
}

//==============================================================================
void CollisionGroup::addShapeFrames(
    std::span<const dynamics::ShapeFrame* const> shapeFrames)
{
  for (const auto& shapeFrame : shapeFrames) {
    addShapeFrame(shapeFrame);
  }
}

//==============================================================================
void CollisionGroup::addShapeFramesOf()
{
  // Do nothing
}

//==============================================================================
void CollisionGroup::subscribeTo()
{
  // Do nothing
}

//==============================================================================
void CollisionGroup::removeShapeFrame(const dynamics::ShapeFrame* shapeFrame)
{
  if (!shapeFrame) {
    return;
  }

  const auto search = std::ranges::find_if(
      mObjectInfoList, [&](const std::unique_ptr<ObjectInfo>& info) {
        return info->mFrame == shapeFrame;
      });

  if (mObjectInfoList.end() == search) {
    return;
  }

  removeCollisionObjectFromEngine((*search)->mObject.get());

  // Since the user is explicitly telling us to remove this ShapeFrame, we can
  // no longer remain subscribed to any sources that were providing this
  // ShapeFrame. Otherwise, this ShapeFrame would just reappear instantly the
  // next time an update is performed.
  for (const void* source : (*search)->mSources) {
    if (nullptr == source) {
      continue;
    }

    eraseSkeletonSource(static_cast<const dynamics::MetaSkeleton*>(source));
    eraseBodyNodeSource(static_cast<const dynamics::BodyNode*>(source));
  }

  mObjectInfoList.erase(search);
  mObserver.removeShapeFrame(shapeFrame);
}

//==============================================================================
void CollisionGroup::removeShapeFrames(
    std::span<const dynamics::ShapeFrame* const> shapeFrames)
{
  for (const auto& shapeFrame : shapeFrames) {
    removeShapeFrame(shapeFrame);
  }
}

//==============================================================================
void CollisionGroup::removeShapeFramesOf()
{
  // Do nothing
}

//==============================================================================
void CollisionGroup::unsubscribeFrom()
{
  // Do nothing
}

//==============================================================================
bool CollisionGroup::isSubscribedTo()
{
  return true;
}

//==============================================================================
void CollisionGroup::removeAllShapeFrames()
{
  removeAllCollisionObjectsFromEngine();

  mObjectInfoList.clear();
  mObserver.removeAllShapeFrames();
}

//==============================================================================
bool CollisionGroup::hasShapeFrame(const dynamics::ShapeFrame* shapeFrame) const
{
  return std::ranges::any_of(
      mObjectInfoList, [&](const std::unique_ptr<ObjectInfo>& info) {
        return info->mFrame == shapeFrame;
      });
}

//==============================================================================
std::size_t CollisionGroup::getNumShapeFrames() const
{
  return mObjectInfoList.size();
}

//==============================================================================
const dynamics::ShapeFrame* CollisionGroup::getShapeFrame(
    std::size_t index) const
{
  DART_ASSERT(index < mObjectInfoList.size());
  if (index < mObjectInfoList.size()) {
    return mObjectInfoList[index]->mFrame;
  }

  return nullptr;
}

//==============================================================================
bool CollisionGroup::collide(
    const CollisionOption& option, CollisionResult* result)
{
  if (mUpdateAutomatically) {
    update();
  }

  return mCollisionDetector->collide(this, option, result);
}

//==============================================================================
bool CollisionGroup::collide(
    CollisionGroup* otherGroup,
    const CollisionOption& option,
    CollisionResult* result)
{
  if (mUpdateAutomatically) {
    update();
  }

  return mCollisionDetector->collide(this, otherGroup, option, result);
}

//==============================================================================
double CollisionGroup::distance(
    const DistanceOption& option, DistanceResult* result)
{
  if (mUpdateAutomatically) {
    update();
  }

  return mCollisionDetector->distance(this, option, result);
}

//==============================================================================
double CollisionGroup::distance(
    CollisionGroup* otherGroup,
    const DistanceOption& option,
    DistanceResult* result)
{
  if (mUpdateAutomatically) {
    update();
  }

  return mCollisionDetector->distance(this, otherGroup, option, result);
}

//==============================================================================
bool CollisionGroup::raycast(
    const Eigen::Vector3d& from,
    const Eigen::Vector3d& to,
    const RaycastOption& option,
    RaycastResult* result)
{
  if (mUpdateAutomatically) {
    update();
  }

  return mCollisionDetector->raycast(this, from, to, option, result);
}

//==============================================================================
void CollisionGroup::setAutomaticUpdate(const bool automatic)
{
  mUpdateAutomatically = automatic;
}

//==============================================================================
bool CollisionGroup::getAutomaticUpdate() const
{
  return mUpdateAutomatically;
}

//==============================================================================
void CollisionGroup::update()
{
  removeDeletedShapeFrames();
  updateSubscriptions();
}

//==============================================================================
void CollisionGroup::updateEngineData()
{
  for (const auto& info : mObjectInfoList) {
    info->mObject->updateEngineData();
  }

  updateCollisionGroupEngineData();
}

//==============================================================================
void CollisionGroup::ShapeFrameObserver::addShapeFrame(
    const dynamics::ShapeFrame* shapeFrame)
{
  addSubject(shapeFrame);
  mMap.insert(
      std::make_pair(
          static_cast<const common::Subject*>(shapeFrame), shapeFrame));
}

//==============================================================================
void CollisionGroup::ShapeFrameObserver::removeShapeFrame(
    const dynamics::ShapeFrame* shapeFrame)
{
  removeSubject(shapeFrame);
  mMap.erase(shapeFrame);
}

//==============================================================================
void CollisionGroup::ShapeFrameObserver::removeAllShapeFrames()
{
  removeAllSubjects();
}

//==============================================================================
void CollisionGroup::ShapeFrameObserver::handleDestructionNotification(
    const common::Subject* subject)
{
  auto it = mMap.find(subject);
  if (it == mMap.end()) {
    return;
  }

  const dynamics::ShapeFrame* frame = it->second;
  mMap.erase(it);

  if (mGroup) {
    mGroup->handleShapeFrameDestruction(frame);
  } else {
    mDeletedFrames.insert(frame);
  }
}

//==============================================================================
void CollisionGroup::removeShapeFrameInternal(
    const dynamics::ShapeFrame* shapeFrame, const void* source)
{
  if (!shapeFrame) {
    return;
  }

  const auto search = std::ranges::find_if(
      mObjectInfoList, [&](const std::unique_ptr<ObjectInfo>& info) {
        return info->mFrame == shapeFrame;
      });

  if (mObjectInfoList.end() == search) {
    return;
  }

  std::unordered_set<const void*>& objectSources = (*search)->mSources;
  objectSources.erase(source);

  if (objectSources.empty()) {
    removeCollisionObjectFromEngine((*search)->mObject.get());
    mObjectInfoList.erase(search);
    mObserver.removeShapeFrame(shapeFrame);
  }
}

//==============================================================================
void CollisionGroup::handleShapeFrameDestruction(
    const dynamics::ShapeFrame* shapeFrame)
{
  if (!shapeFrame) {
    return;
  }

  mObserver.mDeletedFrames.insert(shapeFrame);
  removeDeletedShapeFrames();
}

} // namespace collision
} // namespace dart
