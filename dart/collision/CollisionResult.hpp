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

#ifndef DART_COLLISION_COLLISIONRESULT_HPP_
#define DART_COLLISION_COLLISIONRESULT_HPP_

#include <vector>
#include <unordered_set>
#include "dart/collision/Contact.hpp"

namespace dart {

namespace dynamics {

class BodyNode;
class ShapeFrame;

} // namespace dynamics

namespace collision {

class CollisionResult
{
public:

  /// Add one contact
  void addContact(const Contact& contact);

  /// Return number of contacts
  std::size_t getNumContacts() const;

  /// Return the index-th contact
  Contact& getContact(std::size_t index);

  /// Return (const) the index-th contact
  const Contact& getContact(std::size_t index) const;

  /// Return contacts
  const std::vector<Contact>& getContacts() const;

  /// Return the set of BodyNodes that are in collision
  const std::unordered_set<const dynamics::BodyNode*>&
  getCollidingBodyNodes() const;

  /// Return the set of ShapeFrames that are in collision
  const std::unordered_set<const dynamics::ShapeFrame*>&
  getCollidingShapeFrames() const;

  /// Returns true if the given BodyNode is in collision
  bool inCollision(const dynamics::BodyNode* bn) const;

  /// Returns true if the given ShapeFrame is in collision
  bool inCollision(const dynamics::ShapeFrame* frame) const;

  /// Return binary collision result
  bool isCollision() const;

  /// Implicitly converts this CollisionResult to the value of isCollision()
  operator bool() const;

  /// Clear all the contacts
  void clear();

protected:

  void addObject(CollisionObject* object);

  /// List of contact information for each contact
  std::vector<Contact> mContacts;

  /// Set of BodyNodes that are colliding
  std::unordered_set<const dynamics::BodyNode*> mCollidingBodyNodes;

  /// Set of ShapeFrames that are colliding
  std::unordered_set<const dynamics::ShapeFrame*> mCollidingShapeFrames;

};

}  // namespace collision
}  // namespace dart

#endif  // DART_COLLISION_COLLISIONRESULT_HPP_
