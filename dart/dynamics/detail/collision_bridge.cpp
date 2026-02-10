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

#include "dart/collision/collision_detector.hpp"
#include "dart/collision/collision_filter.hpp"
#include "dart/collision/collision_group.hpp"
#include "dart/collision/collision_object.hpp"
#include "dart/collision/collision_result.hpp"
#include "dart/collision/contact.hpp"
#include "dart/collision/distance_filter.hpp"
#include "dart/common/logging.hpp"
#include "dart/common/macros.hpp"
#include "dart/dynamics/body_node.hpp"
#include "dart/dynamics/shape_frame.hpp"
#include "dart/dynamics/shape_node.hpp"
#include "dart/dynamics/skeleton.hpp"

#include <algorithm>

#include <cstdint>

namespace dart {
namespace collision {

//==============================================================================
struct CollisionGroup::Impl
{
  template <typename Source, typename Child = void>
  struct CollisionSourceData
  {
    Source mSource;

    std::size_t mLastKnownVersion;

    std::unordered_map<const dynamics::ShapeFrame*, ObjectInfo*> mObjects;

    struct ChildInfo
    {
      std::size_t mLastKnownVersion;
      std::unordered_set<const dynamics::ShapeFrame*> mFrames;

      explicit ChildInfo(const std::size_t version) : mLastKnownVersion(version)
      {
      }
    };

    std::unordered_map<const Child*, ChildInfo> mChildren;

    CollisionSourceData(const Source& source, std::size_t lastKnownVersion)
      : mSource(source), mLastKnownVersion(lastKnownVersion)
    {
    }
  };

  using SkeletonSource = CollisionSourceData<
      dynamics::WeakConstMetaSkeletonPtr,
      dynamics::BodyNode>;
  using SkeletonSources
      = std::unordered_map<const dynamics::MetaSkeleton*, SkeletonSource>;

  using BodyNodeSource = CollisionSourceData<dynamics::WeakConstBodyNodePtr>;
  using BodyNodeSources
      = std::unordered_map<const dynamics::BodyNode*, BodyNodeSource>;

  SkeletonSources mSkeletonSources;
  BodyNodeSources mBodyNodeSources;

  static void ensureImpl(CollisionGroup& group)
  {
    if (!group.mImpl) {
      group.mImpl = std::make_unique<CollisionGroup::Impl>();
    }
  }

  static std::size_t computeMetaSkeletonVersion(
      const dynamics::MetaSkeleton& metaSkeleton)
  {
    if (const auto* skeleton
        = dynamic_cast<const dynamics::Skeleton*>(&metaSkeleton)) {
      return skeleton->getVersion();
    }

    const std::size_t numBodies = metaSkeleton.getNumBodyNodes();
    std::size_t seed = numBodies;

    for (std::size_t i = 0u; i < numBodies; ++i) {
      const auto* bodyNode = metaSkeleton.getBodyNode(i);
      const auto bodyNodeHash = static_cast<std::size_t>(
          reinterpret_cast<std::uintptr_t>(bodyNode));
      const auto bodyNodeVersion = bodyNode ? bodyNode->getVersion() : 0u;

      seed ^= bodyNodeHash + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
      seed ^= bodyNodeVersion + 0x9e3779b97f4a7c15ULL + (seed << 6)
              + (seed >> 2);
    }

    return seed;
  }

  static bool updateShapeFrame(
      CollisionGroup& group, CollisionGroup::ObjectInfo* object)
  {
    const dynamics::ConstShapePtr& shape = object->mFrame->getShape();
    const std::size_t currentID = shape ? shape->getID() : 0;
    const std::size_t currentVersion = shape ? shape->getVersion() : 0;

    if (currentID != object->mLastKnownShapeID
        || currentVersion != object->mLastKnownVersion) {
      group.removeCollisionObjectFromEngine(object->mObject.get());
      group.mCollisionDetector->refreshCollisionObject(object->mObject.get());
      group.addCollisionObjectToEngine(object->mObject.get());

      object->mLastKnownShapeID = currentID;
      object->mLastKnownVersion = currentVersion;

      return true;
    }

    return false;
  }

  static bool doUpdateSkeletonSource(
      CollisionGroup& group, SkeletonSources::value_type& entry)
  {
    SkeletonSource& source = entry.second;

    const dynamics::ConstMetaSkeletonPtr meta = source.mSource.lock();
    if (!meta) {
      for (const auto& object : source.mObjects) {
        group.removeShapeFrameInternal(object.second->mFrame, entry.first);
      }

      return true;
    }

    const std::size_t currentSkeletonVersion
        = computeMetaSkeletonVersion(*meta);
    if (currentSkeletonVersion == source.mLastKnownVersion) {
      return false;
    }

    source.mLastKnownVersion = currentSkeletonVersion;
    bool updateNeeded = false;

    auto unusedChildren = source.mChildren;

    for (std::size_t i = 0; i < meta->getNumBodyNodes(); ++i) {
      const dynamics::BodyNode* bn = meta->getBodyNode(i);
      const std::size_t currentVersion = bn->getVersion();

      unusedChildren.erase(bn);

      auto insertion = source.mChildren.insert(
          std::make_pair(bn, SkeletonSource::ChildInfo(currentVersion)));

      const auto child = insertion.first;

      if (insertion.second) {
        updateNeeded = true;

        bn->eachShapeNodeWith<dynamics::CollisionAspect>(
            [&](const dynamics::ShapeNode* shapeNode) {
              source.mObjects.insert(
                  {shapeNode, group.addShapeFrameImpl(shapeNode, meta.get())});
              child->second.mFrames.insert(shapeNode);
            });

        continue;
      }

      if (child->second.mLastKnownVersion == currentVersion) {
        continue;
      }

      child->second.mLastKnownVersion = currentVersion;

      std::unordered_set<const dynamics::ShapeFrame*> unusedFrames
          = child->second.mFrames;

      child->first->eachShapeNodeWith<dynamics::CollisionAspect>(
          [&](const dynamics::ShapeNode* shapeNode) {
            unusedFrames.erase(shapeNode);

            auto frameInsertion
                = source.mObjects.insert(std::make_pair(shapeNode, nullptr));

            const auto& it = frameInsertion.first;
            if (frameInsertion.second) {
              updateNeeded = true;

              it->second = group.addShapeFrameImpl(shapeNode, meta.get());
              child->second.mFrames.insert(shapeNode);
            } else {
              updateNeeded |= updateShapeFrame(group, it->second);
            }
          });

      for (const dynamics::ShapeFrame* unused : unusedFrames) {
        updateNeeded = true;
        group.removeShapeFrameInternal(unused, meta.get());
        child->second.mFrames.erase(unused);
      }
    }

    for (const auto& unusedChild : unusedChildren) {
      for (const dynamics::ShapeFrame* unusedFrame :
           unusedChild.second.mFrames) {
        updateNeeded = true;
        group.removeShapeFrameInternal(unusedFrame, meta.get());
        source.mObjects.erase(unusedFrame);
      }

      source.mChildren.erase(unusedChild.first);
    }

    return updateNeeded;
  }

  static bool doUpdateBodyNodeSource(
      CollisionGroup& group, BodyNodeSources::value_type& entry)
  {
    BodyNodeSource& source = entry.second;

    const dynamics::ConstBodyNodePtr bn = source.mSource.lock();
    if (!bn) {
      for (const auto& object : source.mObjects) {
        group.removeShapeFrameInternal(object.second->mFrame, object.first);
      }

      return true;
    }

    const std::size_t currentBodyNodeVersion = bn->getVersion();

    if (currentBodyNodeVersion == source.mLastKnownVersion) {
      return false;
    }

    source.mLastKnownVersion = currentBodyNodeVersion;
    bool updateNeeded = false;

    std::unordered_map<const dynamics::ShapeFrame*, CollisionGroup::ObjectInfo*>
        unusedFrames = source.mObjects;

    bn->eachShapeNodeWith<dynamics::CollisionAspect>(
        [&](const dynamics::ShapeNode* shapeNode) {
          unusedFrames.erase(shapeNode);

          auto frameInsertion
              = source.mObjects.insert(std::make_pair(shapeNode, nullptr));

          const auto& it = frameInsertion.first;
          if (frameInsertion.second) {
            updateNeeded = true;
            it->second = group.addShapeFrameImpl(shapeNode, bn.get());
          } else {
            updateNeeded |= updateShapeFrame(group, it->second);
          }
        });

    for (const auto& unusedFrame : unusedFrames) {
      updateNeeded = true;
      group.removeShapeFrameInternal(unusedFrame.first, bn.get());
      source.mObjects.erase(unusedFrame.first);
    }

    return updateNeeded;
  }
};

//==============================================================================
CollisionGroup::CollisionGroup(const CollisionDetectorPtr& collisionDetector)
  : mCollisionDetector(collisionDetector),
    mUpdateAutomatically(true),
    mObserver(this)
{
  DART_ASSERT(mCollisionDetector);
}

//==============================================================================
CollisionGroup::CollisionGroup(CollisionGroup&&) noexcept = default;

//==============================================================================
CollisionGroup& CollisionGroup::operator=(CollisionGroup&&) noexcept = default;

//==============================================================================
CollisionGroup::~CollisionGroup() = default;

//==============================================================================
dynamics::ConstShapePtr CollisionObject::getShape() const
{
  return mShapeFrame->getShape();
}

//==============================================================================
const Eigen::Isometry3d& CollisionObject::getTransform() const
{
  return mShapeFrame->getWorldTransform();
}

//==============================================================================
CollisionObject::CollisionObject(
    CollisionDetector* collisionDetector,
    const dynamics::ShapeFrame* shapeFrame)
  : mCollisionDetector(collisionDetector), mShapeFrame(shapeFrame)
{
  DART_ASSERT(mCollisionDetector);
  DART_ASSERT(mShapeFrame);
}

namespace {

const dynamics::ShapeFrame* getShapeFrame(const CollisionObject* object)
{
  if (object == nullptr) {
    return nullptr;
  }

  return object->getShapeFrame();
}

const dynamics::ShapeNode* getShapeNode(const CollisionObject* object)
{
  const auto* shapeFrame = getShapeFrame(object);
  if (shapeFrame == nullptr) {
    return nullptr;
  }

  return shapeFrame->asShapeNode();
}

dynamics::ConstBodyNodePtr getBodyNode(const CollisionObject* object)
{
  const auto* shapeNode = getShapeNode(object);
  if (shapeNode == nullptr) {
    return nullptr;
  }

  return shapeNode->getBodyNodePtr();
}

} // namespace

//==============================================================================
const dynamics::ShapeFrame* Contact::getShapeFrame1() const
{
  return getShapeFrame(collisionObject1);
}

//==============================================================================
const dynamics::ShapeFrame* Contact::getShapeFrame2() const
{
  return getShapeFrame(collisionObject2);
}

//==============================================================================
const dynamics::ShapeNode* Contact::getShapeNode1() const
{
  return getShapeNode(collisionObject1);
}

//==============================================================================
const dynamics::ShapeNode* Contact::getShapeNode2() const
{
  return getShapeNode(collisionObject2);
}

//==============================================================================
dynamics::ConstBodyNodePtr Contact::getBodyNodePtr1() const
{
  return getBodyNode(collisionObject1);
}

//==============================================================================
dynamics::ConstBodyNodePtr Contact::getBodyNodePtr2() const
{
  return getBodyNode(collisionObject2);
}

//==============================================================================
const std::unordered_set<const dynamics::BodyNode*>&
CollisionResult::getCollidingBodyNodes() const
{
  return mCollidingBodyNodes;
}

//==============================================================================
bool CollisionResult::inCollision(const dynamics::BodyNode* bn) const
{
  return mCollidingBodyNodes.contains(bn);
}

//==============================================================================
void CollisionResult::addObject(CollisionObject* object)
{
  if (!object) {
    DART_ERROR(
        "Attempting to add a collision with a nullptr object to a "
        "CollisionResult instance. This is not allowed. Please report this as "
        "a bug!");
    DART_ASSERT(false);
    return;
  }

  const dynamics::ShapeFrame* frame = object->getShapeFrame();
  mCollidingShapeFrames.insert(frame);

  if (frame->isShapeNode()) {
    const dynamics::ShapeNode* node = frame->asShapeNode();
    mCollidingBodyNodes.insert(node->getBodyNodePtr());
  }
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
  if (object1 == object2) {
    return true;
  }

  const auto* shapeFrame1 = object1->getShapeFrame();
  const auto* shapeFrame2 = object2->getShapeFrame();
  if (!shapeFrame1 || !shapeFrame2) {
    return false;
  }

  auto shapeNode1 = shapeFrame1->asShapeNode();
  auto shapeNode2 = shapeFrame2->asShapeNode();

  if (!shapeNode1 || !shapeNode2) {
    return false;
  }

  auto bodyNode1 = shapeNode1->getBodyNodePtr();
  auto bodyNode2 = shapeNode2->getBodyNodePtr();
  if (!bodyNode1 || !bodyNode2) {
    return false;
  }

  if (bodyNode1 == bodyNode2) {
    return true;
  }

  if (!bodyNode1->isCollidable() || !bodyNode2->isCollidable()) {
    return true;
  }

  const auto& skel1 = bodyNode1->getSkeleton();
  const auto& skel2 = bodyNode2->getSkeleton();
  if (!skel1 || !skel2) {
    return false;
  }

  if (!skel1->isMobile() && !skel2->isMobile()) {
    return true;
  }

  if (skel1 == skel2) {
    if (!skel1->isEnabledSelfCollisionCheck()) {
      return true;
    }

    if (!skel1->isEnabledAdjacentBodyCheck()) {
      if (areAdjacentBodies(bodyNode1, bodyNode2)) {
        return true;
      }
    }
  }

  if (mBodyNodeBlackList.contains(bodyNode1, bodyNode2)) {
    return true;
  }

  return false;
}

//==============================================================================
bool BodyNodeCollisionFilter::areAdjacentBodies(
    const dynamics::BodyNode* bodyNode1,
    const dynamics::BodyNode* bodyNode2) const
{
  if ((bodyNode1->getParentBodyNode() == bodyNode2)
      || (bodyNode2->getParentBodyNode() == bodyNode1)) {
    DART_ASSERT(bodyNode1->getSkeleton() == bodyNode2->getSkeleton());
    return true;
  }

  return false;
}

//==============================================================================
bool BodyNodeDistanceFilter::needDistance(
    const collision::CollisionObject* object1,
    const collision::CollisionObject* object2) const
{
  if (object1 == object2) {
    return false;
  }

  auto shapeNode1 = object1->getShapeFrame()->asShapeNode();
  auto shapeNode2 = object2->getShapeFrame()->asShapeNode();

  if (!shapeNode1 || !shapeNode2) {
    return true;
  }

  auto bodyNode1 = shapeNode1->getBodyNodePtr();
  auto bodyNode2 = shapeNode2->getBodyNodePtr();

  if (!bodyNode1->isCollidable() || !bodyNode2->isCollidable()) {
    return false;
  }

  if (bodyNode1->getSkeleton() == bodyNode2->getSkeleton()) {
    auto skeleton = bodyNode1->getSkeleton();

    if (!skeleton->isEnabledSelfCollisionCheck()) {
      return false;
    }

    if (!skeleton->isEnabledAdjacentBodyCheck()) {
      if (areAdjacentBodies(bodyNode1, bodyNode2)) {
        return false;
      }
    }
  }

  return true;
}

//==============================================================================
bool BodyNodeDistanceFilter::areAdjacentBodies(
    const dynamics::BodyNode* bodyNode1,
    const dynamics::BodyNode* bodyNode2) const
{
  if ((bodyNode1->getParentBodyNode() == bodyNode2)
      || (bodyNode2->getParentBodyNode() == bodyNode1)) {
    DART_ASSERT(bodyNode1->getSkeleton() == bodyNode2->getSkeleton());
    return true;
  }

  return false;
}

//==============================================================================
void CollisionGroup::removeDeletedShapeFrames()
{
  for (auto shapeFrame : mObserver.mDeletedFrames) {
    const auto search = std::ranges::find_if(
        mObjectInfoList, [&](const std::unique_ptr<ObjectInfo>& info) {
          return info->mFrame == shapeFrame;
        });

    if (mObjectInfoList.end() == search) {
      continue;
    }

    for (const void* source : (*search)->mSources) {
      if (nullptr == source) {
        continue;
      }

      if (mImpl) {
        auto skelSearch = mImpl->mSkeletonSources.find(
            static_cast<const dynamics::MetaSkeleton*>(source));
        if (skelSearch != mImpl->mSkeletonSources.end()) {
          skelSearch->second.mObjects.erase(shapeFrame);
          for (auto& child : skelSearch->second.mChildren) {
            child.second.mFrames.erase(shapeFrame);
          }
        }

        auto bodySearch = mImpl->mBodyNodeSources.find(
            static_cast<const dynamics::BodyNode*>(source));
        if (bodySearch != mImpl->mBodyNodeSources.end()) {
          bodySearch->second.mObjects.erase(shapeFrame);
        }
      }
    }

    removeCollisionObjectFromEngine((*search)->mObject.get());
    mObjectInfoList.erase(search);
  }

  mObserver.mDeletedFrames.clear();
}

//==============================================================================
auto CollisionGroup::addShapeFrameImpl(
    const dynamics::ShapeFrame* shapeFrame, const void* source) -> ObjectInfo*
{
  if (!shapeFrame) {
    return nullptr;
  }

  auto it = std::ranges::find_if(
      mObjectInfoList, [&](const std::unique_ptr<ObjectInfo>& info) {
        return info->mFrame == shapeFrame;
      });

  if (it == mObjectInfoList.end()) {
    auto collObj = mCollisionDetector->claimCollisionObject(shapeFrame);

    addCollisionObjectToEngine(collObj.get());

    const dynamics::ConstShapePtr& shape = shapeFrame->getShape();

    mObjectInfoList.emplace_back(new ObjectInfo{
        shapeFrame,
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
void CollisionGroup::addShapeFramesOfBodyNode(
    const dynamics::BodyNode* bodyNode)
{
  DART_ASSERT(bodyNode);

  bodyNode->eachShapeNodeWith<dynamics::CollisionAspect>(
      [this](const dynamics::ShapeNode* shapeNode) {
        addShapeFrame(shapeNode);
      });
}

//==============================================================================
void CollisionGroup::addShapeFramesOfMetaSkeleton(
    const dynamics::MetaSkeleton* skel)
{
  DART_ASSERT(skel);

  auto numBodyNodes = skel->getNumBodyNodes();
  for (auto i = 0u; i < numBodyNodes; ++i) {
    addShapeFramesOfBodyNode(skel->getBodyNode(i));
  }
}

//==============================================================================
void CollisionGroup::addShapeFramesOfGroup(const CollisionGroup* otherGroup)
{
  DART_ASSERT(otherGroup);

  if (otherGroup && this != otherGroup) {
    for (const auto& info : otherGroup->mObjectInfoList) {
      addShapeFrame(info->mFrame);
    }
  }
}

//==============================================================================
void CollisionGroup::subscribeToBodyNode(
    const dynamics::ConstBodyNodePtr& bodyNode)
{
  Impl::ensureImpl(*this);

  const auto inserted = mImpl->mBodyNodeSources.insert(
      Impl::BodyNodeSources::value_type(
          bodyNode.get(),
          Impl::BodyNodeSource(bodyNode.get(), bodyNode->getVersion())));

  if (inserted.second) {
    const Impl::BodyNodeSources::iterator& entry = inserted.first;
    bodyNode->eachShapeNodeWith<dynamics::CollisionAspect>(
        [&](const dynamics::ShapeNode* shapeNode) {
          entry->second.mObjects.insert(
              {shapeNode, addShapeFrameImpl(shapeNode, bodyNode.get())});
        });
  }
}

//==============================================================================
void CollisionGroup::subscribeToMetaSkeleton(
    const dynamics::ConstMetaSkeletonPtr& metaSkeleton)
{
  Impl::ensureImpl(*this);

  const auto inserted = mImpl->mSkeletonSources.insert(
      Impl::SkeletonSources::value_type(
          metaSkeleton.get(),
          Impl::SkeletonSource(
              metaSkeleton, Impl::computeMetaSkeletonVersion(*metaSkeleton))));

  if (inserted.second) {
    Impl::SkeletonSource& entry = inserted.first->second;

    const std::size_t numBodies = metaSkeleton->getNumBodyNodes();
    for (std::size_t i = 0u; i < numBodies; ++i) {
      const dynamics::BodyNode* bn = metaSkeleton->getBodyNode(i);

      auto& childInfo
          = entry.mChildren
                .insert(
                    std::make_pair(
                        bn, Impl::SkeletonSource::ChildInfo(bn->getVersion())))
                .first->second;

      bn->eachShapeNodeWith<dynamics::CollisionAspect>(
          [&](const dynamics::ShapeNode* shapeNode) {
            entry.mObjects.insert(
                {shapeNode, addShapeFrameImpl(shapeNode, metaSkeleton.get())});
            childInfo.mFrames.insert(shapeNode);
          });
    }
  }
}

//==============================================================================
void CollisionGroup::removeShapeFramesOfBodyNode(
    const dynamics::BodyNode* bodyNode)
{
  DART_ASSERT(bodyNode);

  bodyNode->eachShapeNodeWith<dynamics::CollisionAspect>(
      [&](const dynamics::ShapeNode* shapeNode) {
        removeShapeFrame(shapeNode);
      });
}

//==============================================================================
void CollisionGroup::removeShapeFramesOfMetaSkeleton(
    const dynamics::MetaSkeleton* skel)
{
  DART_ASSERT(skel);

  auto numBodyNodes = skel->getNumBodyNodes();
  for (auto i = 0u; i < numBodyNodes; ++i) {
    removeShapeFramesOfBodyNode(skel->getBodyNode(i));
  }
}

//==============================================================================
void CollisionGroup::removeShapeFramesOfGroup(const CollisionGroup* otherGroup)
{
  DART_ASSERT(otherGroup);

  if (otherGroup) {
    if (this == otherGroup) {
      removeAllShapeFrames();
      return;
    }

    for (const auto& info : otherGroup->mObjectInfoList) {
      removeShapeFrame(info->mFrame);
    }
  }
}

//==============================================================================
void CollisionGroup::unsubscribeFromBodyNode(const dynamics::BodyNode* bodyNode)
{
  if (!mImpl) {
    return;
  }

  auto it = mImpl->mBodyNodeSources.find(bodyNode);
  if (it != mImpl->mBodyNodeSources.end()) {
    for (const auto& entry : it->second.mObjects) {
      removeShapeFrameInternal(entry.first, bodyNode);
    }

    mImpl->mBodyNodeSources.erase(it);
  }
}

//==============================================================================
void CollisionGroup::unsubscribeFromMetaSkeleton(
    const dynamics::MetaSkeleton* skeleton)
{
  if (!mImpl) {
    return;
  }

  auto it = mImpl->mSkeletonSources.find(skeleton);
  if (it != mImpl->mSkeletonSources.end()) {
    for (const auto& entry : it->second.mObjects) {
      removeShapeFrameInternal(entry.first, skeleton);
    }

    mImpl->mSkeletonSources.erase(it);
  }
}

//==============================================================================
bool CollisionGroup::isSubscribedToBodyNode(const dynamics::BodyNode* bodyNode)
{
  return mImpl && mImpl->mBodyNodeSources.contains(bodyNode);
}

//==============================================================================
bool CollisionGroup::isSubscribedToMetaSkeleton(
    const dynamics::MetaSkeleton* skeleton)
{
  return mImpl && mImpl->mSkeletonSources.contains(skeleton);
}

//==============================================================================
void CollisionGroup::updateSubscriptions()
{
  if (!mImpl) {
    return;
  }

  for (auto& entry : mImpl->mSkeletonSources) {
    Impl::doUpdateSkeletonSource(*this, entry);
  }

  for (auto& entry : mImpl->mBodyNodeSources) {
    Impl::doUpdateBodyNodeSource(*this, entry);
  }
}

//==============================================================================
void CollisionGroup::eraseSkeletonSource(const dynamics::MetaSkeleton* source)
{
  if (mImpl) {
    mImpl->mSkeletonSources.erase(source);
  }
}

//==============================================================================
void CollisionGroup::eraseBodyNodeSource(const dynamics::BodyNode* source)
{
  if (mImpl) {
    mImpl->mBodyNodeSources.erase(source);
  }
}

} // namespace collision
} // namespace dart
