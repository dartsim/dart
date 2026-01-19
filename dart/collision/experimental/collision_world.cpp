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

#include <dart/collision/experimental/collision_world.hpp>

#include <dart/collision/experimental/narrow_phase/narrow_phase.hpp>

#include <algorithm>

namespace dart::collision::experimental {

CollisionWorld::CollisionWorld() = default;

void CollisionWorld::addObject(std::shared_ptr<CollisionObject> object)
{
  if (!object) {
    return;
  }
  objects_.push_back(object);
  broadPhase_.add(object->getId(), object->computeAabb());
}

void CollisionWorld::removeObject(std::shared_ptr<CollisionObject> object)
{
  if (!object) {
    return;
  }
  removeObject(object->getId());
}

void CollisionWorld::removeObject(std::size_t objectId)
{
  auto it = std::find_if(objects_.begin(), objects_.end(), [objectId](const auto& obj) {
    return obj->getId() == objectId;
  });

  if (it != objects_.end()) {
    broadPhase_.remove(objectId);
    objects_.erase(it);
  }
}

void CollisionWorld::updateObject(CollisionObject* object)
{
  if (object) {
    broadPhase_.update(object->getId(), object->computeAabb());
  }
}

std::size_t CollisionWorld::numObjects() const
{
  return objects_.size();
}

CollisionObject* CollisionWorld::getObject(std::size_t index)
{
  if (index >= objects_.size()) {
    return nullptr;
  }
  return objects_[index].get();
}

const CollisionObject* CollisionWorld::getObject(std::size_t index) const
{
  if (index >= objects_.size()) {
    return nullptr;
  }
  return objects_[index].get();
}

bool CollisionWorld::collide(const CollisionOption& option, CollisionResult& result)
{
  result.clear();
  bool hasCollision = false;

  std::unordered_map<std::size_t, CollisionObject*> idToObject;
  for (auto& obj : objects_) {
    idToObject[obj->getId()] = obj.get();
  }

  auto pairs = broadPhase_.queryPairs();

  for (const auto& pair : pairs) {
    auto it1 = idToObject.find(pair.first);
    auto it2 = idToObject.find(pair.second);

    if (it1 == idToObject.end() || it2 == idToObject.end()) {
      continue;
    }

    if (NarrowPhase::collide(*it1->second, *it2->second, option, result)) {
      hasCollision = true;
      if (option.enableContact == false
          || (option.maxNumContacts > 0 && result.numContacts() >= option.maxNumContacts)) {
        break;
      }
    }
  }

  return hasCollision;
}

bool CollisionWorld::collide(
    CollisionObject* obj1,
    CollisionObject* obj2,
    const CollisionOption& option,
    CollisionResult& result)
{
  if (!obj1 || !obj2) {
    return false;
  }
  return NarrowPhase::collide(*obj1, *obj2, option, result);
}

void CollisionWorld::clear()
{
  broadPhase_.clear();
  objects_.clear();
}

}
