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

#ifndef DART_COLLISION_ENGINE_H_
#define DART_COLLISION_ENGINE_H_

#include <vector>

#include "dart/collision/CollisionDetector.h"
#include "dart/collision/SmartPointer.h"
#include "dart/dynamics/SmartPointer.h"

namespace dart {
namespace collision {

struct Option
{
  /// Flag whether compute contact information such as point, normal, and
  /// penetration depth. If this flag is set to false, the Engine returns only
  /// simple information whether there is a collision of not.
  bool enableContact;

  /// Maximum number of contacts to detect
  size_t maxNumContacts;

  /// Constructor
  Option(bool enableContact = true,
         size_t maxNumContacts = 100);
};

struct Result
{
  /// List of contact information for each contact
  std::vector<Contact> contacts;
};

class Engine
{
public:

  using CollisionObjectPtrs = std::vector<CollisionObjectPtr>;

  /// Create a collision object
  template <class CollisionObjectType, typename... Args>
  std::unique_ptr<CollisionObject> createCollisionObject(
      const dynamics::ShapePtr& shape,
      const Args&... args);

  /// Create a collision group
  template <class CollisionObjectType, typename... Args>
  std::unique_ptr<CollisionGroup> createCollisionGroup(
      const CollisionObjectPtrs& collObjects,
      const Args&... args);

  /// Return collision detection engine type in std::string
  virtual const std::string& getType() const = 0;

  /// Create collision detection engine specific data for CollisionObject
  virtual std::unique_ptr<CollisionObjectEngineData> createCollisionObjectData(
      CollisionObject* parent,
      const dynamics::ShapePtr& shape) = 0;

  /// Create collision detection engine specific data for CollisionGroup
  virtual std::unique_ptr<CollisionGroupEngineData> createCollisionGroupData(
      const CollisionObjectPtrs& collObjects) = 0;

  /// Perform collision detection for object1-object2.
  virtual bool detect(CollisionObject* object1, CollisionObject* object2,
                      const Option& option, Result& result) = 0;

  /// Perform collision detection for object-group.
  virtual bool detect(CollisionObject* object, CollisionGroup* group,
                      const Option& option, Result& result) = 0;

  /// Identical with detect(object, group, option, result)
  bool detect(CollisionGroup* group, CollisionObject* object,
              const Option& option, Result& result);

  /// Perform collision detection for group.
  virtual bool detect(CollisionGroup* group,
                      const Option& option, Result& result) = 0;

  /// Perform collision detection for group1-group2.
  virtual bool detect(CollisionGroup* group1, CollisionGroup* group2,
                      const Option& option, Result& result) = 0;

};

//==============================================================================
template <class CollisionObjectType, typename... Args>
std::unique_ptr<CollisionObject> Engine::createCollisionObject(
    const dynamics::ShapePtr& shape, const Args&... args)
{
  return std::unique_ptr<CollisionObject>(
        new CollisionObjectType(this, shape, args...));
}

//==============================================================================
template <class CollisionGroupType, typename... Args>
std::unique_ptr<CollisionGroup> Engine::createCollisionGroup(
    const Engine::CollisionObjectPtrs& collObjects, const Args&... args)
{
  return std::unique_ptr<CollisionObject>(
        new CollisionGroupType(this, collObjects, args...));
}

}  // namespace collision
}  // namespace dart

#endif  // DART_COLLISION_ENGINE_H_
