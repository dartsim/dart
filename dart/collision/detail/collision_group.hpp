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

#ifndef DART_COLLISION_DETAIL_COLLISIONGROUP_HPP_
#define DART_COLLISION_DETAIL_COLLISIONGROUP_HPP_

#include <dart/collision/collision_group.hpp>

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
    std::span<const dynamics::ShapeFrame* const> shapeFrames,
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
  addShapeFramesOfGroup(otherGroup);
  addShapeFramesOf(others...);
}

//==============================================================================
template <typename... Others>
void CollisionGroup::addShapeFramesOf(
    const dynamics::BodyNode* bodyNode, const Others*... others)
{
  addShapeFramesOfBodyNode(bodyNode);
  addShapeFramesOf(others...);
}

//==============================================================================
template <typename... Others>
void CollisionGroup::addShapeFramesOf(
    const dynamics::MetaSkeleton* skeleton, const Others*... others)
{
  addShapeFramesOfMetaSkeleton(skeleton);
  addShapeFramesOf(others...);
}

//==============================================================================
template <typename... Others>
void CollisionGroup::subscribeTo(
    const dynamics::ConstBodyNodePtr& bodyNode, const Others&... others)
{
  subscribeToBodyNode(bodyNode);
  subscribeTo(others...);
}

//==============================================================================
template <typename... Others>
void CollisionGroup::subscribeTo(
    const dynamics::ConstMetaSkeletonPtr& metaSkeleton, const Others&... others)
{
  subscribeToMetaSkeleton(metaSkeleton);
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
    std::span<const dynamics::ShapeFrame* const> shapeFrames,
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
  removeShapeFramesOfGroup(otherGroup);
  removeShapeFramesOf(others...);
}

//==============================================================================
template <typename... Others>
void CollisionGroup::removeShapeFramesOf(
    const dynamics::BodyNode* bodyNode, const Others*... others)
{
  removeShapeFramesOfBodyNode(bodyNode);
  removeShapeFramesOf(others...);
}

//==============================================================================
template <typename... Others>
void CollisionGroup::removeShapeFramesOf(
    const dynamics::MetaSkeleton* skeleton, const Others*... others)
{
  removeShapeFramesOfMetaSkeleton(skeleton);
  removeShapeFramesOf(others...);
}

//==============================================================================
template <typename... Others>
void CollisionGroup::unsubscribeFrom(
    const dynamics::BodyNode* bodyNode, const Others*... others)
{
  unsubscribeFromBodyNode(bodyNode);
  unsubscribeFrom(others...);
}

//==============================================================================
template <typename... Others>
void CollisionGroup::unsubscribeFrom(
    const dynamics::MetaSkeleton* skeleton, const Others*... others)
{
  unsubscribeFromMetaSkeleton(skeleton);
  unsubscribeFrom(others...);
}

//==============================================================================
template <typename... Others>
bool CollisionGroup::isSubscribedTo(
    const dynamics::BodyNode* bodyNode, const Others*... others)
{
  return isSubscribedToBodyNode(bodyNode) && isSubscribedTo(others...);
}

//==============================================================================
template <typename... Others>
bool CollisionGroup::isSubscribedTo(
    const dynamics::MetaSkeleton* skeleton, const Others*... others)
{
  return isSubscribedToMetaSkeleton(skeleton) && isSubscribedTo(others...);
}

} // namespace collision
} // namespace dart

#endif // DART_COLLISION_DETAIL_COLLISIONGROUP_HPP_
