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

#include "dart/collision/CollisionDetector.h"
#include "dart/collision/CollisionGroupData.h"

namespace dart {
namespace collision {

class CollisionDetector;
class CollisionNode;

/// Heterogeneous collision group
class CollisionGroup
{
public:

  using CollisionObjectPtr = std::shared_ptr<CollisionObject>;
  using CollisionObjectPtrs = std::vector<CollisionObjectPtr>;
  using ConstCollisionObjectPtrs = std::vector<ConstCollisionObjectPtr>;

  /// Constructor
  CollisionGroup(
      const CollisionDetectorPtr& collisionDetector,
      const CollisionObjectPtrs& collObjects = CollisionObjectPtrs());

  /// Copy constructor
  CollisionGroup(const CollisionGroup& other);

  /// Destructor
  virtual ~CollisionGroup();

  /// Assignment operator
  CollisionGroup& operator=(const CollisionGroup& other);

  /// Copy another CollisionGroup into this CollisionGroup
  void copy(const CollisionGroup& other);

  /// Change engine
  void changeDetector(const CollisionDetectorPtr& collisionDetector);

  /// Return collision detection engine associated with this CollisionGroup
  CollisionDetector* getCollisionDetector() const;

  /// Return true if this CollisionGroup contains given object
  bool hasCollisionObject(const CollisionObjectPtr& object) const;

  /// Add collision object to this CollisionGroup
  void addCollisionObject(const CollisionObjectPtr& object,
                          bool init = true);

  /// Add collision objects to this CollisionGroup
  void addCollisionObjects(const CollisionObjectPtrs& objects,
                           bool init = true);

  /// Remove collision object from this CollisionGroup
  void removeCollisionObject(const CollisionObjectPtr& object,
                             bool init = true);

  /// Remove collision objects from this CollisionGroup
  void removeCollisionObjects(const CollisionObjectPtrs& objects,
                              bool init = true);

  /// Remove all the collision object in this CollisionGroup
  void removeAllCollisionObjects(bool init = true);

  /// Return array of collision objects
  const CollisionObjectPtrs& getCollisionObjects();

  /// Return array of (const) collision objects
  const ConstCollisionObjectPtrs getCollisionObjects() const;

  /// Merge other CollisionGroup into this CollisionGroup
  void unionGroup(const CollisionGroupPtr& other);

  /// Merge other CollisionGroup into this CollisionGroup
  void subtractGroup(const CollisionGroupPtr& other);

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
  CollisionGroupData* getEngineData();

  /// Update engine data. This function should be called before the collision
  /// detection is performed by the engine in most cases.
  void updateEngineData();

protected:

  /// Collision detector
  CollisionDetectorPtr mCollisionDetector;

  /// Collision objects
  CollisionObjectPtrs mCollisionObjects;

  /// Engine specific data
  std::unique_ptr<CollisionGroupData> mEngineData;

};
// TODO(JS): make this class iterable for collision objects

}  // namespace collision
}  // namespace dart

#endif  // DART_COLLISION_COLLISIONGROUP_H_
