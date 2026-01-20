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

#include "dart/collision/contact.hpp"

#include "dart/collision/collision_object.hpp"
#include "dart/dynamics/body_node.hpp"
#include "dart/dynamics/shape_frame.hpp"
#include "dart/dynamics/shape_node.hpp"

namespace dart {
namespace collision {
namespace {

const dynamics::ShapeFrame* getShapeFrame(const CollisionObject* object)
{
  if (object == nullptr)
    return nullptr;

  return object->getShapeFrame();
}

const dynamics::ShapeNode* getShapeNode(const CollisionObject* object)
{
  const auto* shapeFrame = getShapeFrame(object);
  if (shapeFrame == nullptr)
    return nullptr;

  return shapeFrame->asShapeNode();
}

dynamics::ConstBodyNodePtr getBodyNode(const CollisionObject* object)
{
  const auto* shapeNode = getShapeNode(object);
  if (shapeNode == nullptr)
    return nullptr;

  return shapeNode->getBodyNodePtr();
}

} // namespace

//==============================================================================
Contact::Contact()
  : point(Eigen::Vector3d::Zero()),
    normal(Eigen::Vector3d::Zero()),
    force(Eigen::Vector3d::Zero()),
    collisionObject1(nullptr),
    collisionObject2(nullptr),
    penetrationDepth(0),
    triID1(0),
    triID2(0),
    userData(nullptr)
{
  // TODO(MXG): Consider using NaN instead of zero for uninitialized quantities
  // Do nothing
}

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
bool Contact::isZeroNormal(const Eigen::Vector3d& normal)
{
  if (!normal.allFinite())
    return true;

  if (normal.squaredNorm() < getNormalEpsilonSquared())
    return true;
  else
    return false;
}

//==============================================================================
bool Contact::isNonZeroNormal(const Eigen::Vector3d& normal)
{
  return !isZeroNormal(normal);
}

} // namespace collision
} // namespace dart
