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

#ifndef DART_COLLISION_DETAIL_COLLISIONGROUP_HPP_
#define DART_COLLISION_DETAIL_COLLISIONGROUP_HPP_

#include "dart/collision/CollisionGroup.hpp"

#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/Skeleton.hpp"

namespace dart {
namespace collision {

//==============================================================================
template <typename... Others>
void CollisionGroup::addShapeFramesOf(
    const dynamics::ShapeFrame* shapeFrame, const Others*... others)
{
  addShapeFrame(shapeFrame);

  addShapeFramesOf(others...);
}

//==============================================================================
template <typename... Others>
void CollisionGroup::addShapeFramesOf(
    const std::vector<const dynamics::ShapeFrame*>& shapeFrames,
    const Others*... others)
{
  addShapeFrames(shapeFrames);

  addShapeFramesOf(others...);
}

//==============================================================================
template <typename... Others>
void CollisionGroup::addShapeFramesOf(
    const CollisionGroup* otherGroup, const Others*... others)
{
  assert(otherGroup);

  if (otherGroup && this != otherGroup)
  {
    for (const auto& info : otherGroup->mObjectInfoList)
      addShapeFrame(info->mFrame);
  }

  addShapeFramesOf(others...);
}

//==============================================================================
template <typename... Others>
void CollisionGroup::addShapeFramesOf(
    const dynamics::BodyNode* bodyNode, const Others*... others)
{
  assert(bodyNode);

  auto collisionShapeNodes
      = bodyNode->getShapeNodesWith<dynamics::CollisionAspect>();

  for (auto& shapeNode : collisionShapeNodes)
    addShapeFrame(shapeNode);

  addShapeFramesOf(others...);
}

//==============================================================================
template <typename... Others>
void CollisionGroup::addShapeFramesOf(
    const dynamics::MetaSkeleton* skel, const Others*... others)
{
  assert(skel);

  auto numBodyNodes = skel->getNumBodyNodes();
  for (auto i = 0u; i < numBodyNodes; ++i)
    addShapeFramesOf(skel->getBodyNode(i));

  addShapeFramesOf(others...);
}

//==============================================================================
template <typename... Others>
void CollisionGroup::subscribeTo(
    const dynamics::ConstBodyNodePtr& bodyNode,
    const Others&... others)
{
  const auto inserted = mBodyNodeSources.insert(
        BodyNodeSources::value_type(
          bodyNode.get(),
          BodyNodeSource(bodyNode.get(), bodyNode->getVersion()))
        );

  if(inserted.second)
  {
    const BodyNodeSources::iterator& entry = inserted.first;

    const auto collisionShapeNodes =
        bodyNode->getShapeNodesWith<dynamics::CollisionAspect>();

    for (const auto& shapeNode : collisionShapeNodes)
    {
      entry->second.mObjects.insert(
        {shapeNode, addShapeFrameImpl(shapeNode, bodyNode.get())});
    }
  }

  subscribeTo(others...);
}

//==============================================================================
template <typename... Others>
void CollisionGroup::subscribeTo(
    const dynamics::ConstSkeletonPtr& skeleton,
    const Others&... others)
{
  const auto inserted = mSkeletonSources.insert(
      SkeletonSources::value_type(
        skeleton.get(),
        SkeletonSource(skeleton, skeleton->getVersion()))
      );

  if(inserted.second)
  {
    SkeletonSource& entry = inserted.first->second;

    const std::size_t numBodies = skeleton->getNumBodyNodes();
    for (std::size_t i = 0u ; i < numBodies; ++i)
    {
      const dynamics::BodyNode* bn = skeleton->getBodyNode(i);

      const auto& collisionShapeNodes =
          bn->getShapeNodesWith<dynamics::CollisionAspect>();

      auto& childInfo = entry.mChildren.insert(
            std::make_pair(bn, SkeletonSource::ChildInfo(bn->getVersion())))
          .first->second;

      for (const auto& shapeNode : collisionShapeNodes)
      {
        entry.mObjects.insert(
            {shapeNode, addShapeFrameImpl(shapeNode, skeleton.get())});
        childInfo.mFrames.insert(shapeNode);
      }
    }
  }

  subscribeTo(others...);
}

//==============================================================================
template <typename... Others>
void CollisionGroup::removeShapeFramesOf(
    const dynamics::ShapeFrame* shapeFrame, const Others*... others)
{
  removeShapeFrame(shapeFrame);

  removeShapeFramesOf(others...);
}

//==============================================================================
template <typename... Others>
void CollisionGroup::removeShapeFramesOf(
    const std::vector<const dynamics::ShapeFrame*>& shapeFrames,
    const Others*... others)
{
  removeShapeFrames(shapeFrames);

  removeShapeFramesOf(others...);
}

//==============================================================================
template <typename... Others>
void CollisionGroup::removeShapeFramesOf(
    const CollisionGroup* otherGroup, const Others*... others)
{
  assert(otherGroup);

  if (otherGroup)
  {
    if (this == otherGroup)
    {
      removeAllShapeFrames();
      return;
    }

    for (const auto& info : otherGroup->mObjectInfoList)
      removeShapeFrame(info->mFrame);
  }

  removeShapeFramesOf(others...);
}

//==============================================================================
template <typename... Others>
void CollisionGroup::removeShapeFramesOf(
    const dynamics::BodyNode* bodyNode, const Others*... others)
{
  assert(bodyNode);

  auto collisionShapeNodes
      = bodyNode->getShapeNodesWith<dynamics::CollisionAspect>();

  for (auto& shapeNode : collisionShapeNodes)
    removeShapeFrame(shapeNode);

  removeShapeFramesOf(others...);
}

//==============================================================================
template <typename... Others>
void CollisionGroup::removeShapeFramesOf(
    const dynamics::MetaSkeleton* skel, const Others*... others)
{
  assert(skel);

  auto numBodyNodes = skel->getNumBodyNodes();
  for (auto i = 0u; i < numBodyNodes; ++i)
    removeShapeFramesOf(skel->getBodyNode(i));

  removeShapeFramesOf(others...);
}

//==============================================================================
template <typename... Others>
void CollisionGroup::unsubscribeFrom(
    const dynamics::BodyNode* bodyNode,
    const Others*... others)
{
  auto it = mBodyNodeSources.find(bodyNode);
  if(it != mBodyNodeSources.end())
  {
    for(const auto& entry : it->second.mObjects)
      removeShapeFrameInternal(entry.first, bodyNode);

    mBodyNodeSources.erase(it);
  }

  unsubscribeFrom(others...);
}

//==============================================================================
template <typename... Others>
void CollisionGroup::unsubscribeFrom(
    const dynamics::Skeleton* skeleton,
    const Others*... others)
{
  auto it = mSkeletonSources.find(skeleton);
  if(it != mSkeletonSources.end())
  {
    for(const auto& entry : it->second.mObjects)
    {
      removeShapeFrameInternal(
            entry.first, static_cast<const dynamics::MetaSkeleton*>(skeleton));
    }

    mSkeletonSources.erase(it);
  }

  unsubscribeFrom(others...);
}

}  // namespace collision
}  // namespace dart

#endif  // DART_COLLISION_DETAIL_COLLISIONGROUP_HPP_
