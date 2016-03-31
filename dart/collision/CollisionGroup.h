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
#include "dart/collision/SmartPointer.h"
#include "dart/collision/Option.h"
#include "dart/collision/Result.h"
#include "dart/dynamics/SmartPointer.h"

namespace dart {
namespace collision {

class CollisionGroup
{
public:

  friend class FCLCollisionDetector;
  friend class BulletCollisionDetector;

  /// Constructor
  CollisionGroup(const CollisionDetectorPtr& collisionDetector);

  /// Destructor
  virtual ~CollisionGroup();

  /// Return collision detection engine associated with this CollisionGroup
  CollisionDetector* getCollisionDetector();

  /// Return (const) collision detection engine associated with this
  /// CollisionGroup
  const CollisionDetector* getCollisionDetector() const;

  /// Add a ShapeFrame to this CollisionGroup
  void addShapeFrame(const dynamics::ShapeFrame* shapeFrame);

  /// Add ShapeFrames to this CollisionGroup
  void addShapeFrames(
      const std::vector<const dynamics::ShapeFrame*>& shapeFrames);

  /// Add a ShapeFrame, and also add ShapeFrames of other various objects.
  ///
  /// The other various objects can be any of ShapeFrame,
  /// std::vector<ShapeFrame>, CollisionGroup, BodyNode, and Skeleton.
  ///
  /// Note that this function adds only the ShapeFrames of each object at the
  /// moment of this function is called. The aftwerward changes of the objects
  /// does not affect on this CollisionGroup.
  template <typename... Others>
  void addShapeFramesOf(const dynamics::ShapeFrame* shapeFrame,
                        const Others*... others);

  /// Add ShapeFrames, and also add ShapeFrames of other various objects.
  template <typename... Others>
  void addShapeFramesOf(
      const std::vector<const dynamics::ShapeFrame*>& shapeFrames,
      const Others*... others);

  /// Add ShapeFrames of other CollisionGroup, and also add another ShapeFrames
  /// of other various objects.
  template <typename... Others>
  void addShapeFramesOf(const CollisionGroup* otherGroup,
                        const Others*... others);

  /// Add ShapeFrames of BodyNode, and also add another ShapeFrames of other
  /// various objects.
  template <typename... Others>
  void addShapeFramesOf(const dynamics::BodyNode* bodyNode,
                        const Others*... others);

  /// Add ShapeFrames of Skeleton, and also add another ShapeFrames of other
  /// various objects.
  template <typename... Others>
  void addShapeFramesOf(const dynamics::Skeleton* skeleton,
                        const Others*... others);

  /// Do nothing. This function is for terminating the recursive variadic
  /// template function template<typename...> addShapeFramesOf().
  void addShapeFramesOf();

  /// Remove a ShapeFrame from this CollisionGroup
  void removeShapeFrame(const dynamics::ShapeFrame* shapeFrame);

  /// Remove ShapeFrames from this CollisionGroup
  void removeShapeFrames(
      const std::vector<const dynamics::ShapeFrame*>& shapeFrames);

  /// Remove a ShapeFrame, and also remove ShapeFrames of other various objects.
  ///
  /// The other various objects can be any of ShapeFrame,
  /// std::vector<ShapeFrame>, CollisionGroup, BodyNode, and Skeleton.
  ///
  /// Note that this function removes only the ShapeFrames of each object at the
  /// moment of this function is called. The aftwerward changes of the objects
  /// does not affect on this CollisionGroup.
  template <typename... Others>
  void removeShapeFramesOf(const dynamics::ShapeFrame* shapeFrame,
                           const Others*... others);

  /// Remove ShapeFrames, and also remove ShapeFrames of other various objects.
  template <typename... Others>
  void removeShapeFramesOf(
      const std::vector<const dynamics::ShapeFrame*>& shapeFrames,
      const Others*... others);

  /// Remove ShapeFrames of other CollisionGroup, and also remove another
  /// ShapeFrames of other various objects.
  template <typename... Others>
  void removeShapeFramesOf(const CollisionGroup* otherGroup,
                           const Others*... others);

  /// Remove ShapeFrames of BodyNode, and also remove another ShapeFrames of
  /// other various objects.
  template <typename... Others>
  void removeShapeFramesOf(const dynamics::BodyNode* bodyNode,
                           const Others*... others);

  /// Remove ShapeFrames of Skeleton, and also remove another ShapeFrames of
  /// other various objects.
  template <typename... Others>
  void removeShapeFramesOf(const dynamics::Skeleton* skeleton,
                           const Others*... others);

  /// Do nothing. This function is for terminating the recursive variadic
  /// template function template<typename...> removeShapeFramesOf().
  void removeShapeFramesOf();

  /// Unregister all the ShapeFrames in this CollisionGroup
  void unregisterAllShapeFrames();

  /// Return true if this CollisionGroup contains shapeFrame
  bool hasShapeFrame(const dynamics::ShapeFrame* shapeFrame) const;

  /// Return number of ShapeFrames added to this CollisionGroup
  size_t getNumShapeFrames() const;

  /// Return index-th ShapeFrames registered to this CollisionGroup. Return
  /// nullptr if the index is out of the range.
  const dynamics::ShapeFrame* getShapeFrame(size_t index) const;

  /// Return all the ShapeFrames registered to this CollisionGroup
  const std::vector<const dynamics::ShapeFrame*>& getShapeFrames() const;

  /// Perform collision detection within this CollisionGroup.
  bool detect(const Option& option, Result& result);

  /// Perform collision detection with other CollisionGroup.
  ///
  /// Return false if the engine type of the other CollisionGroup is different
  /// from this CollisionObject engine.
  bool detect(CollisionGroup* group, const Option& option, Result& result);

  /// Return all the CollisionObjects in this CollisionGroup
  const std::vector<CollisionObject*>& getCollisionObjects();

protected:

  /// Update engine data. This function should be called before the collision
  /// detection is performed by the engine in most cases.
  void updateEngineData();

  /// Initialize the collision detection engine data such as broadphase
  /// algorithm. This function will be called whenever ShapeFrame is either
  /// added to or removed from this CollisionGroup.
  virtual void initializeEngineData() = 0;

  /// Notify that a CollisionObject is added so that the collision detection
  /// engine do some relevant work
  virtual void notifyCollisionObjectAdded(CollisionObject* object) = 0;

  /// Notify that CollisionObjects are added so that the collision detection
  /// engine do some relevant work
  virtual void notifyCollisionObjectsAdded(
      const std::vector<CollisionObject*>& collObjects) = 0;

  /// Notify that a CollisionObject is removed so that the collision detection
  /// engine do some relevant work
  virtual void notifyCollisionObjectRemoved(CollisionObject* object) = 0;

  /// Notify that all the CollisionObjects are remove so that the collision
  /// detection engine do some relevant work
  virtual void notifyAllCollisionObjectsRemoved() = 0;

  /// Update the collision detection engine data such as broadphase algorithm.
  /// This function will be called ahead of every collision checking.
  virtual void updateCollisionGroupEngineData() = 0;

private:

  /// Collision detector
  CollisionDetectorPtr mCollisionDetector;

  /// ShapeFrames registered to this CollisionGroup
  std::vector<const dynamics::ShapeFrame*> mShapeFrames;

  /// CollisionObjects associated with the registered ShapeFrames
  std::vector<CollisionObject*> mCollisionObjects;

};

}  // namespace collision
}  // namespace dart

#include "dart/collision/detail/CollisionGroup.h"

#endif  // DART_COLLISION_COLLISIONGROUP_H_
