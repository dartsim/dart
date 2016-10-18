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

#include "dart/collision/CollisionResult.hpp"

#include "dart/collision/CollisionObject.hpp"
#include "dart/dynamics/ShapeFrame.hpp"
#include "dart/dynamics/ShapeNode.hpp"
#include "dart/dynamics/BodyNode.hpp"

namespace dart {
namespace collision {

//==============================================================================
void CollisionResult::addContact(const Contact& contact)
{
  mContacts.push_back(contact);
  addObject(contact.collisionObject1);
  addObject(contact.collisionObject2);
}

//==============================================================================
std::size_t CollisionResult::getNumContacts() const
{
  return mContacts.size();
}

//==============================================================================
Contact& CollisionResult::getContact(std::size_t index)
{
  assert(index < mContacts.size());

  return mContacts[index];
}

//==============================================================================
const Contact& CollisionResult::getContact(std::size_t index) const
{
  assert(index < mContacts.size());

  return mContacts[index];
}

//==============================================================================
const std::vector<Contact>& CollisionResult::getContacts() const
{
  return mContacts;
}

//==============================================================================
const std::unordered_set<const dynamics::BodyNode*>&
CollisionResult::getCollidingBodyNodes() const
{
  return mCollidingBodyNodes;
}

//==============================================================================
const std::unordered_set<const dynamics::ShapeFrame*>&
CollisionResult::getCollidingShapeFrames() const
{
  return mCollidingShapeFrames;
}

//==============================================================================
bool CollisionResult::inCollision(const dynamics::BodyNode* bn) const
{
  return (mCollidingBodyNodes.find(bn) != mCollidingBodyNodes.end());
}

//==============================================================================
bool CollisionResult::inCollision(const dynamics::ShapeFrame* frame) const
{
  return (mCollidingShapeFrames.find(frame) != mCollidingShapeFrames.end());
}

//==============================================================================
bool CollisionResult::isCollision() const
{
  return !mContacts.empty();
}

//==============================================================================
CollisionResult::operator bool() const
{
  return isCollision();
}

//==============================================================================
void CollisionResult::clear()
{
  mContacts.clear();
  mCollidingShapeFrames.clear();
  mCollidingBodyNodes.clear();
}

//==============================================================================
void CollisionResult::addObject(CollisionObject* object)
{
  if(!object)
  {
    dterr << "[CollisionResult::addObject] Attempting to add a collision with "
          << "a nullptr object to a CollisionResult instance. This is not "
          << "allowed. Please report this as a bug!";
    assert(false);
    return;
  }

  const dynamics::ShapeFrame* frame = object->getShapeFrame();
  mCollidingShapeFrames.insert(frame);

  if(frame->isShapeNode())
  {
    const dynamics::ShapeNode* node = frame->asShapeNode();
    mCollidingBodyNodes.insert(node->getBodyNodePtr());
  }
}

}  // namespace collision
}  // namespace dart
