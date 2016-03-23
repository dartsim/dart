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
#include <map>

#include "dart/collision/CollisionDetector.h"
#include "dart/collision/CollisionGroup.h"
#include "dart/collision/CollisionFilter.h"

namespace dart {
namespace collision {

class CollisionGroup
{
public:

  /// Constructor
  CollisionGroup(const CollisionDetectorPtr& collisionDetector);

  /// Destructor
  virtual ~CollisionGroup();

  /// Return collision detection engine associated with this CollisionGroup
  CollisionDetector* getCollisionDetector() const;

  /// Add a ShapeFrame to this CollisionGroup
  void addShapeFrame(const dynamics::ShapeFrame* shapeFrame);

  /// Add ShapeFrames to this CollisionGroup
  void addShapeFrames(
      const std::vector<const dynamics::ShapeFrame*>& shapeFrames);

  void addShapeFrames(const dynamics::Skeleton* skeleton);

  /// Remove collision object from this CollisionGroup
  void removeShapeFrame(const dynamics::ShapeFrame* shapeFrame);

  /// Remove collision objects from this CollisionGroup
  void removeShapeFrames(
      const std::vector<const dynamics::ShapeFrame*>& shapeFrames);

  /// Remove all the collision object in this CollisionGroup
  void removeAllShapeFrames();

  /// Return true if this CollisionGroup contains shapeFrame
  bool hasShapeFrame(const dynamics::ShapeFrame* shapeFrame) const;

  /// Merge other CollisionGroup into this CollisionGroup
  void unionGroup(const CollisionGroupPtr& other);

  /// Merge other CollisionGroup into this CollisionGroup
  void subtractGroup(const CollisionGroupPtr& other);

  /// Update engine data. This function should be called before the collision
  /// detection is performed by the engine in most cases.
  void update();

  /// Perform collision detection within this CollisionGroup.
  bool detect(const Option& option, Result& result);

  /// Perform collision detection with other CollisionGroup.
  ///
  /// Return false if the engine type of the other CollisionGroup is different
  /// from this CollisionObject engine.
  bool detect(CollisionGroup* group, const Option& option, Result& result);

  const std::vector<CollisionObject*>& getCollisionObjects();

protected:

  /// Initialize the collision group data of the collision detection engine such
  /// as broadphase algorithm. This function will be called whenever ShapeFrame
  /// is either added to or removed from this CollisionGroup.
  virtual void initializeEngineData() = 0;

  virtual void addCollisionObjectToEngine(CollisionObject* object) = 0;

  virtual void addCollisionObjectsToEngine(
      const std::vector<CollisionObject*>& collObjects) = 0;

  virtual void removeCollisionObjectFromEngine(CollisionObject* object) = 0;

  virtual void removeAllCollisionObjectsFromEngine() = 0;

  /// Update the collision group of the collision detection engine such as
  /// broadphase algorithm. This function will be called ahead of every
  /// collision checking.
  virtual void updateEngineData() = 0;

  using CollisionObjectPtr = std::shared_ptr<CollisionObject>;
  using CollisionObjectPtrs = std::vector<CollisionObjectPtr>;
  using ConstCollisionObjectPtrs = std::vector<ConstCollisionObjectPtr>;

protected:

  /// Collision detector
  CollisionDetectorPtr mCollisionDetector;

  std::vector<const dynamics::ShapeFrame*> mShapeFrames;
  std::vector<CollisionObject*> mCollisionObjects;

};

}  // namespace collision
}  // namespace dart

#endif  // DART_COLLISION_COLLISIONGROUP_H_
