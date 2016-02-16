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

#ifndef DART_COLLISION_COLLISIONGROUP_H_
#define DART_COLLISION_COLLISIONGROUP_H_

#include <vector>

#include "dart/collision/Engine.h"

namespace dart {
namespace collision {

class Engine;
class CollisionNode;
class CollisionGroupEngineData;

class CollisionGroup
{
public:

  using CollisionObjectPtr = std::shared_ptr<CollisionObject>;
  using CollisionObjects = std::vector<CollisionObjectPtr>;

  /// Constructor
  CollisionGroup(const EnginePtr engine,
                 const CollisionObjects& collObjects = CollisionObjects());

  /// Constructor
  CollisionGroup(const EnginePtr& engine,
                 const CollisionObjectPtr& collObject);

  /// Destructor
  virtual ~CollisionGroup();

  /// Return collision detection engine associated with this CollisionGroup
  Engine* getEngine() const;

  /// Add collision object to this CollisionGroup
  void addCollisionObject(const CollisionObjectPtr& object);

  /// Perform collision detection within this CollisionGroup.
  bool detect(const Option& option, Result& result);

  /// Perform collision detection with other CollisionObject.
  ///
  /// Return false if the engine type of the other CollisionObject is different
  /// from this CollisionObject engine.
  bool detect(CollisionObject* object, const Option& option, Result& result);

  /// Perform collision detection with other CollisionGroup.
  ///
  /// Return false if the engine type of the other CollisionGroup is different
  /// from this CollisionObject engine.
  bool detect(CollisionGroup* group, const Option& option, Result& result);

  /// Return the collision detection engine specific data of this
  /// CollisionObject
  CollisionGroupEngineData* getEngineData();

  /// Update engine data. This function should be called before the collision
  /// detection is performed by the engine in most cases.
  void updateEngineData();

protected:

  /// Collision engine
  EnginePtr mEngine;

  /// Collision objects
  CollisionObjects mCollisionObjects;

  /// Engine specific data
  std::shared_ptr<CollisionGroupEngineData> mEngineData;

};
// TODO(JS): make this class iterable for collision objects

using CollisionGroupPtr = std::shared_ptr<CollisionGroup>;

}  // namespace collision
}  // namespace dart

#endif  // DART_COLLISION_COLLISIONGROUP_H_
