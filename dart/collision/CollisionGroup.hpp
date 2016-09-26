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

#ifndef DART_COLLISION_COLLISIONGROUP_HPP_
#define DART_COLLISION_COLLISIONGROUP_HPP_

#include <map>
#include <vector>
#include "dart/collision/SmartPointer.hpp"
#include "dart/collision/CollisionOption.hpp"
#include "dart/collision/CollisionResult.hpp"
#include "dart/collision/DistanceOption.hpp"
#include "dart/collision/DistanceResult.hpp"
#include "dart/dynamics/SmartPointer.hpp"

namespace dart {
namespace collision {

class CollisionGroup
{
public:

  /// Constructor
  CollisionGroup(const CollisionDetectorPtr& collisionDetector);
  // CollisionGroup also can be created from CollisionDetector::create()

  /// Destructor
  virtual ~CollisionGroup() = default;

  /// Return collision detection engine associated with this CollisionGroup
  CollisionDetectorPtr getCollisionDetector();

  /// Return (const) collision detection engine associated with this
  /// CollisionGroup
  ConstCollisionDetectorPtr getCollisionDetector() const;

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
  void addShapeFramesOf(const dynamics::MetaSkeleton* skeleton,
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

  /// Remove all the ShapeFrames in this CollisionGroup
  void removeAllShapeFrames();

  /// Return true if this CollisionGroup contains shapeFrame
  bool hasShapeFrame(const dynamics::ShapeFrame* shapeFrame) const;

  /// Return number of ShapeFrames added to this CollisionGroup
  std::size_t getNumShapeFrames() const;

  /// Get the ShapeFrame corresponding to the given index
  const dynamics::ShapeFrame* getShapeFrame(std::size_t index) const;

  /// Perform collision check within this CollisionGroup.
  bool collide(
      const CollisionOption& option = CollisionOption(false, 1u, nullptr),
      CollisionResult* result = nullptr);

  /// Perform collision check with other CollisionGroup.
  ///
  /// Return false if the engine type of the other CollisionGroup is different
  /// from this CollisionObject engine.
  bool collide(
      CollisionGroup* otherGroup,
      const CollisionOption& option = CollisionOption(false, 1u, nullptr),
      CollisionResult* result = nullptr);

  /// Get the minimum signed distance between the Shape pairs in this
  /// CollisionGroup.
  ///
  /// The detailed results are stored in the given DistanceResult if provided.
  ///
  /// The results can be different by DistanceOption. By default, non-negative
  /// minimum distance (distance >= 0) is returned for all the shape pairs
  /// without computing nearest points.
  double distance(
      const DistanceOption& option = DistanceOption(false, 0.0, nullptr),
      DistanceResult* result = nullptr);

  /// Get the minimum signed distance between the Shape pairs where a pair
  /// consist of two shapes from each groups (one from this CollisionGroup and
  /// one from otherGroup).
  ///
  /// Note that the distance between shapes within the same CollisionGroup
  /// are not accounted.
  ///
  /// The detailed results are stored in the given DistanceResult if provided.
  ///
  /// The results can be different by DistanceOption. By default, non-negative
  /// minimum distance (distance >= 0) is returned for all the shape pairs
  /// without computing nearest points.
  double distance(
      CollisionGroup* otherGroup,
      const DistanceOption& option = DistanceOption(false, 0.0, nullptr),
      DistanceResult* result = nullptr);

protected:

  /// Update engine data. This function should be called before the collision
  /// detection is performed by the engine in most cases.
  void updateEngineData();

  /// Initialize the collision detection engine data such as broadphase
  /// algorithm. This function will be called whenever ShapeFrame is either
  /// added to or removed from this CollisionGroup.
  virtual void initializeEngineData() = 0;

  /// Add CollisionObject to the collision detection engine
  virtual void addCollisionObjectToEngine(CollisionObject* object) = 0;

  /// Add CollisionObjects to the collision detection engine
  virtual void addCollisionObjectsToEngine(
      const std::vector<CollisionObject*>& collObjects) = 0;

  /// Remove CollisionObject from the collision detection engine
  virtual void removeCollisionObjectFromEngine(CollisionObject* object) = 0;

  /// Remove all the CollisionObjects from the collision detection engine
  virtual void removeAllCollisionObjectsFromEngine() = 0;

  /// Update the collision detection engine data such as broadphase algorithm.
  /// This function will be called ahead of every collision checking.
  virtual void updateCollisionGroupEngineData() = 0;

protected:

  /// Collision detector
  CollisionDetectorPtr mCollisionDetector;
  // CollisionGroup shares the ownership of CollisionDetector with other
  // CollisionGroups created from the same CollisionDetector so that the
  // CollisionDetector doesn't get destroyed as long as at least one
  // CollisionGroup is alive.

  /// ShapeFrames and CollisionOjbects added to this CollisionGroup
  std::vector<std::pair<const dynamics::ShapeFrame*,
                        CollisionObjectPtr>> mShapeFrameMap;
  // CollisionGroup also shares the ownership of CollisionObjects across other
  // CollisionGroups for the same reason with above.
  //
  // Dev note: This was supposed to be std::map rather than std::vector for
  // better search performance. The reason we use std::vector is to get
  // deterministic contact results regardless of the order of CollisionObjects
  // in this container for FCLCollisionDetector.
  //
  // fcl's collision result is dependent on the order of objects in the broad
  // phase classes. If we use std::map, the orders of element between the
  // original and copy are not guranteed to be the same as we copy std::map
  // (e.g., by world cloning).

};

}  // namespace collision
}  // namespace dart

#include "dart/collision/detail/CollisionGroup.hpp"

#endif  // DART_COLLISION_COLLISIONGROUP_HPP_
