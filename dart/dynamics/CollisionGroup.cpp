/*
 * Copyright (c) 2011-2021, The DART development contributors
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

#include "dart/dynamics/CollisionGroup.hpp"

#include <cassert>

#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/CollisionDetector.hpp"
#include "dart/dynamics/CollisionObject.hpp"
#include "dart/dynamics/Skeleton.hpp"

namespace dart {
namespace dynamics {

//==============================================================================
CollisionGroup::CollisionGroup(const CollisionDetectorPtr& collisionDetector)
  : mCollisionDetector(collisionDetector), mUpdateAutomatically(true)
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
  addShapeFrameImpl(shapeFrame, nullptr);
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
void CollisionGroup::subscribeTo()
{
  // Do nothing
}

//==============================================================================
void CollisionGroup::removeShapeFrame(const dynamics::ShapeFrame* shapeFrame)
{
  if (!shapeFrame)
    return;

  const auto search = std::find_if(
      mObjectInfoList.begin(),
      mObjectInfoList.end(),
      [&](const std::unique_ptr<ObjectInfo>& info) {
        return info->mFrame == shapeFrame;
      });

  if (mObjectInfoList.end() == search)
    return;

  removeCollisionObjectFromEngine((*search)->mObject.get());

  // Since the user is explicitly telling us to remove this ShapeFrame, we can
  // no longer remain subscribed to any sources that were providing this
  // ShapeFrame. Otherwise, this ShapeFrame would just reappear instantly the
  // next time an update is performed.
  for (const void* source : (*search)->mSources)
  {
    if (nullptr == source)
      continue;

    // We don't know which container this source is kept in, so try erasing it
    // from each.
    if (mSkeletonSources.erase(
            static_cast<const dynamics::MetaSkeleton*>(source))
        > 0)
      continue;

    mBodyNodeSources.erase(static_cast<const dynamics::BodyNode*>(source));
  }

  mObjectInfoList.erase(search);
  mObserver.removeShapeFrame(shapeFrame);
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

  mObjectInfoList.clear();
  mObserver.removeAllShapeFrames();
}

//==============================================================================
bool CollisionGroup::hasShapeFrame(const dynamics::ShapeFrame* shapeFrame) const
{
  return std::find_if(
             mObjectInfoList.begin(),
             mObjectInfoList.end(),
             [&](const std::unique_ptr<ObjectInfo>& info) {
               return info->mFrame == shapeFrame;
             })
         != mObjectInfoList.end();
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
  assert(index < mObjectInfoList.size());
  if (index < mObjectInfoList.size())
    return mObjectInfoList[index]->mFrame;

  return nullptr;
}

//==============================================================================
bool CollisionGroup::collide(
    const CollisionOption& option, CollisionResult* result)
{
  if (mUpdateAutomatically)
    update();

  return mCollisionDetector->collide(this, option, result);
}

//==============================================================================
bool CollisionGroup::collide(
    CollisionGroup* otherGroup,
    const CollisionOption& option,
    CollisionResult* result)
{
  if (mUpdateAutomatically)
    update();

  return mCollisionDetector->collide(this, otherGroup, option, result);
}

//==============================================================================
double CollisionGroup::distance(
    const DistanceOption& option, DistanceResult* result)
{
  if (mUpdateAutomatically)
    update();

  return mCollisionDetector->distance(this, option, result);
}

//==============================================================================
double CollisionGroup::distance(
    CollisionGroup* otherGroup,
    const DistanceOption& option,
    DistanceResult* result)
{
  if (mUpdateAutomatically)
    update();

  return mCollisionDetector->distance(this, otherGroup, option, result);
}

//==============================================================================
bool CollisionGroup::raycast(
    const Eigen::Vector3d& from,
    const Eigen::Vector3d& to,
    const RaycastOption& option,
    RaycastResult* result)
{
  if (mUpdateAutomatically)
    update();

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

  for (auto& entry : mSkeletonSources)
    updateSkeletonSource(entry);

  for (auto& entry : mBodyNodeSources)
    updateBodyNodeSource(entry);
}

//==============================================================================
void CollisionGroup::removeDeletedShapeFrames()
{
  for (auto shapeFrame : mObserver.mDeletedFrames)
  {
    const auto search = std::find_if(
        mObjectInfoList.begin(),
        mObjectInfoList.end(),
        [&](const std::unique_ptr<ObjectInfo>& info) {
          return info->mFrame == shapeFrame;
        });

    if (mObjectInfoList.end() == search)
      continue;

    // Clear out the ShapeFrame from any subscriber that it might have
    for (const void* source : (*search)->mSources)
    {
      if (nullptr == source)
        continue;

      auto skelSearch = mSkeletonSources.find(
          static_cast<const dynamics::MetaSkeleton*>(source));
      if (skelSearch != mSkeletonSources.end())
      {
        skelSearch->second.mObjects.erase(shapeFrame);
        for (auto& child : skelSearch->second.mChildren)
          child.second.mFrames.erase(shapeFrame);
      }

      auto bodySearch = mBodyNodeSources.find(
          static_cast<const dynamics::BodyNode*>(source));
      if (bodySearch != mBodyNodeSources.end())
        bodySearch->second.mObjects.erase(shapeFrame);
    }

    removeCollisionObjectFromEngine((*search)->mObject.get());
    mObjectInfoList.erase(search);
  }

  mObserver.mDeletedFrames.clear();
}

//==============================================================================
void CollisionGroup::updateEngineData()
{
  for (const auto& info : mObjectInfoList)
    info->mObject->updateEngineData();

  updateCollisionGroupEngineData();
}

//==============================================================================
void CollisionGroup::ShapeFrameObserver::addShapeFrame(
    const dynamics::ShapeFrame* shapeFrame)
{
  addSubject(shapeFrame);
  mMap.insert(std::make_pair(
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
  mDeletedFrames.insert(mMap[subject]);
  mMap.erase(subject);
}

//==============================================================================
auto CollisionGroup::addShapeFrameImpl(
    const dynamics::ShapeFrame* shapeFrame, const void* source) -> ObjectInfo*
{
  if (!shapeFrame)
    return nullptr;

  auto it = std::find_if(
      mObjectInfoList.begin(),
      mObjectInfoList.end(),
      [&](const std::unique_ptr<ObjectInfo>& info) {
        return info->mFrame == shapeFrame;
      });

  if (it == mObjectInfoList.end())
  {
    auto collObj = mCollisionDetector->claimCollisionObject(shapeFrame);

    addCollisionObjectToEngine(collObj.get());

    const dynamics::ConstShapePtr& shape = shapeFrame->getShape();

    // If shape is a nullptr, use improbable values for ID and version. ID is
    // not likely to ever be 0 because Shape IDs start at
    // PRIMITIVE_MAGIC_NUMBER (defined as 1000 in Shape.cpp). Version is not
    // likely to be 0 because the Shape class starts its version counter at 1,
    // and the version count should monotonically increase.
    mObjectInfoList.emplace_back(new ObjectInfo{shapeFrame,
                                                collObj,
                                                shape ? shape->getID() : 0,
                                                shape ? shape->getVersion() : 0,
                                                {}});
    mObserver.addShapeFrame(shapeFrame);

    it = --mObjectInfoList.end();
  }

  (*it)->mSources.insert(source);

  return it->get();
}

//==============================================================================
void CollisionGroup::removeShapeFrameInternal(
    const dynamics::ShapeFrame* shapeFrame, const void* source)
{
  if (!shapeFrame)
    return;

  const auto search = std::find_if(
      mObjectInfoList.begin(),
      mObjectInfoList.end(),
      [&](const std::unique_ptr<ObjectInfo>& info) {
        return info->mFrame == shapeFrame;
      });

  if (mObjectInfoList.end() == search)
    return;

  std::unordered_set<const void*>& objectSources = (*search)->mSources;
  objectSources.erase(source);

  if (objectSources.empty())
  {
    removeCollisionObjectFromEngine((*search)->mObject.get());
    mObjectInfoList.erase(search);
    mObserver.removeShapeFrame(shapeFrame);
  }
}

//==============================================================================
bool CollisionGroup::updateSkeletonSource(SkeletonSources::value_type& entry)
{
  SkeletonSource& source = entry.second;

  const dynamics::ConstMetaSkeletonPtr& meta = source.mSource.lock();
  if (!meta)
  {
    // This skeleton no longer exists, so we should remove all its contents from
    // the CollisionGroup.
    for (const auto& object : source.mObjects)
      removeShapeFrameInternal(object.second->mFrame, object.first);

    return true;
  }

  // Note: Right now we can only subscribe to Skeletons, not MetaSkeletons, so
  // we can safely do a static_cast here. Eventually this static_cast should not
  // be needed, if we can figure out a versioning system for MetaSkeleton.
  const dynamics::Skeleton* skeleton
      = dynamic_cast<const dynamics::Skeleton*>(meta.get());

  const std::size_t currentSkeletonVersion = skeleton->getVersion();
  // If the version hasn't changed, then there will be nothing to update.
  if (currentSkeletonVersion == source.mLastKnownVersion)
    return false;

  source.mLastKnownVersion = currentSkeletonVersion;
  bool updateNeeded = false;

  auto unusedChildren = source.mChildren;

  // Check each child to see if its version number has changed. If it has, then
  // check to see if any of its ShapeFrames need to be updated, added, or
  // removed from the CollisionGroup.
  for (std::size_t i = 0; i < skeleton->getNumBodyNodes(); ++i)
  {
    const dynamics::BodyNode* bn = skeleton->getBodyNode(i);
    const std::size_t currentVersion = bn->getVersion();

    unusedChildren.erase(bn);

    // Attempt an insertion, in case this BodyNode was not already accounted for
    auto insertion = source.mChildren.insert(
        std::make_pair(bn, SkeletonSource::ChildInfo(currentVersion)));

    const auto child = insertion.first;

    if (insertion.second)
    {
      // This child was newly introduced, so we need to add it to the
      // CollisionGroup.
      updateNeeded = true;

      const auto& collisionShapeNodes
          = bn->getShapeNodesWith<dynamics::CollisionAspect>();

      for (const auto& shapeNode : collisionShapeNodes)
      {
        source.mObjects.insert(
            {shapeNode, addShapeFrameImpl(shapeNode, meta.get())});
        child->second.mFrames.insert(shapeNode);
      }

      // Skip the rest of the for-loop, because it's only for updating old
      // children
      continue;
    }

    // If the version of this BodyNode has not changed, then move on to the next
    // one.
    if (child->second.mLastKnownVersion == currentVersion)
      continue;

    child->second.mLastKnownVersion = currentVersion;

    std::unordered_set<const dynamics::ShapeFrame*> unusedFrames
        = child->second.mFrames;

    const std::vector<const dynamics::ShapeNode*> nodes
        = child->first->getShapeNodesWith<dynamics::CollisionAspect>();

    for (const dynamics::ShapeNode* node : nodes)
    {
      unusedFrames.erase(node);

      auto frameInsertion
          = source.mObjects.insert(std::make_pair(node, nullptr));

      const auto& it = frameInsertion.first;
      if (frameInsertion.second)
      {
        // If the insertion occurred, then this is a new ShapeFrame, and we
        // need to create a collision object for it.
        updateNeeded = true;

        it->second = addShapeFrameImpl(node, meta.get());
        child->second.mFrames.insert(node);
      }
      else
      {
        // If the insertion did not occur, then this is an old ShapeFrame, and
        // we should check if it needs an update.
        updateNeeded |= updateShapeFrame(it->second);
      }
    }

    for (const dynamics::ShapeFrame* unused : unusedFrames)
    {
      updateNeeded = true;
      removeShapeFrameInternal(unused, meta.get());
      child->second.mFrames.erase(unused);
    }
  }

  // Remove from this group any BodyNodes that no longer belong to the skeleton
  for (const auto& unusedChild : unusedChildren)
  {
    for (const dynamics::ShapeFrame* unusedFrame : unusedChild.second.mFrames)
    {
      updateNeeded = true;
      removeShapeFrameInternal(unusedFrame, meta.get());
      source.mObjects.erase(unusedFrame);
    }

    source.mChildren.erase(unusedChild.first);
  }

  return updateNeeded;
}

//==============================================================================
bool CollisionGroup::updateBodyNodeSource(BodyNodeSources::value_type& entry)
{
  BodyNodeSource& source = entry.second;

  const dynamics::ConstBodyNodePtr bn = source.mSource.lock();
  if (!bn)
  {
    // This BodyNode no longer exists, so we should remove all i ts contents
    // from the CollisionGroup.
    for (const auto& object : source.mObjects)
      removeShapeFrameInternal(object.second->mFrame, object.first);

    return true;
  }

  const std::size_t currentBodyNodeVersion = bn->getVersion();

  // If the version hasn't changed, then tehre will be nothing to update.
  if (currentBodyNodeVersion == source.mLastKnownVersion)
    return false;

  source.mLastKnownVersion = currentBodyNodeVersion;
  bool updateNeeded = false;

  std::unordered_map<const dynamics::ShapeFrame*, ObjectInfo*> unusedFrames
      = source.mObjects;

  const std::vector<const dynamics::ShapeNode*> nodes
      = bn->getShapeNodesWith<dynamics::CollisionAspect>();

  for (const dynamics::ShapeNode* node : nodes)
  {
    unusedFrames.erase(node);

    auto frameInsertion = source.mObjects.insert(std::make_pair(node, nullptr));

    const auto& it = frameInsertion.first;
    if (frameInsertion.second)
    {
      updateNeeded = true;
      // If the insertion occurred, then this is a new ShapeFrame, and we need
      // to create a collision object for it.
      it->second = addShapeFrameImpl(node, bn.get());
    }
    else
    {
      // If the insertion did not occur, then this is an old ShapeFrame, and
      // we should check if it needs an update.
      updateNeeded |= updateShapeFrame(it->second);
    }
  }

  // Remove from this group and ShapeFrames that no longer belong to the
  // BodyNode
  for (const auto& unusedFrame : unusedFrames)
  {
    updateNeeded = true;
    removeShapeFrameInternal(unusedFrame.first, bn.get());
    source.mObjects.erase(unusedFrame.first);
  }

  return updateNeeded;
}

//==============================================================================
bool CollisionGroup::updateShapeFrame(ObjectInfo* object)
{
  const dynamics::ConstShapePtr& shape = object->mFrame->getShape();
  const std::size_t currentID = shape ? shape->getID() : 0;
  const std::size_t currentVersion = shape ? shape->getVersion() : 0;

  if (currentID != object->mLastKnownShapeID
      || currentVersion != object->mLastKnownVersion)
  {
    removeCollisionObjectFromEngine(object->mObject.get());
    mCollisionDetector->refreshCollisionObject(object->mObject.get());
    addCollisionObjectToEngine(object->mObject.get());

    object->mLastKnownShapeID = currentID;
    object->mLastKnownVersion = currentVersion;

    return true;
  }

  return false;
}

} // namespace collision
} // namespace dart
