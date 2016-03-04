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

#ifndef DART_COLLISION_COLLISIONOBJECT_H_
#define DART_COLLISION_COLLISIONOBJECT_H_

#include <Eigen/Dense>

#include "dart/collision/CollisionDetector.h"
#include "dart/collision/SmartPointer.h"
#include "dart/collision/CollisionObjectData.h"

namespace dart {
namespace collision {

class CollisionObject
{
public:

  friend class CollisionGroup;

  virtual ~CollisionObject();

  /// Return collision detection engine associated with this CollisionObject
  CollisionDetector* getCollisionDetector() const;

  /// Return shape pointer that associated with this CollisionObject
  dynamics::ShapePtr getShape() const;
  // TODO(JS): Shape should be in common or math

  /// Return the transformation of this CollisionObject in world coordinates
  virtual const Eigen::Isometry3d getTransform() const = 0;

  /// Return true if this CollisionObject is identical to other
  virtual bool isEqual(const CollisionObject* other) = 0;

  /// Perform collision detection with other CollisionObject.
  ///
  /// Return false if the engine type of the other CollisionObject is different
  /// from this CollisionObject engine.
  bool detect(CollisionObject* other, const Option& option, Result& result);

  /// Perform collision detection with other CollisionGroup.
  ///
  /// Return false if the engine type of the other CollisionGroup is different
  /// from this CollisionObject engine.
  bool detect(CollisionGroup* group, const Option& option, Result& result);

  /// Return the collision detection engine specific data of this
  /// CollisionObject
  CollisionObjectData* getEngineData() const;

  /// Update engine data. This function should be called before the collision
  /// detection is performed by the engine in most cases.
  void updateEngineData();

protected:

  /// Contructor
  CollisionObject(const CollisionDetectorPtr& collisionDetector,
                  const dynamics::ShapePtr& shape);

  void addGroup(CollisionGroup* group);

  void removeGroup(CollisionGroup* group);

  bool hasGroup(CollisionGroup* group);

protected:

  /// Collision detector
  CollisionDetectorPtr mCollisionDetector;

  /// Shape
  dynamics::ShapePtr mShape;

  /// Collision detection engine specific data
  std::unique_ptr<CollisionObjectData> mEngineData;

  std::vector<CollisionGroup*> mGroups;

};

/// FreeCollisionOjbect is a basic collision object whose transformation can be
/// set freely.
class FreeCollisionObject : public CollisionObject
{
public:

  static std::shared_ptr<FreeCollisionObject> create(
      const CollisionDetectorPtr& collisionDetector,
      const dynamics::ShapePtr& shape,
      const Eigen::Isometry3d& tf = Eigen::Isometry3d::Identity());

  /// Constructor
  FreeCollisionObject(
      const CollisionDetectorPtr& collisionDetector,
      const dynamics::ShapePtr& shape,
      const Eigen::Isometry3d& tf = Eigen::Isometry3d::Identity());
  // TODO(JS): change to engine pointer

  /// Set world transformation of this FreeCollisionObject
  void setTransform(const Eigen::Isometry3d& tf);

  /// Set world rotation of this FreeCollisionObject
  void setRotation(const Eigen::Matrix3d& rotation);

  /// Set world translation of this FreeCollisionObject
  void setTranslation(const Eigen::Vector3d& translation);

  /// Return world transformation of this FreeCollisionObject
  const Eigen::Isometry3d getTransform() const override;

  // Documentation inherited
  bool isEqual(const CollisionObject* other) override;

protected:

  /// Transformation in world coordinates
  Eigen::Isometry3d mW;

};

}  // namespace collision
}  // namespace dart

#endif  // DART_COLLISION_COLLISIONOBJECT_H_
