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

#ifndef DART_COLLISION_COLLISIONDETECTOR_HPP_
#define DART_COLLISION_COLLISIONDETECTOR_HPP_

#include <vector>
#include <map>

#include <Eigen/Dense>

#include "dart/common/Factory.hpp"
#include "dart/collision/Contact.hpp"
#include "dart/collision/CollisionOption.hpp"
#include "dart/collision/CollisionResult.hpp"
#include "dart/collision/DistanceOption.hpp"
#include "dart/collision/DistanceResult.hpp"
#include "dart/collision/SmartPointer.hpp"
#include "dart/dynamics/SmartPointer.hpp"

namespace dart {
namespace collision {

class CollisionObject;

class CollisionDetector : public std::enable_shared_from_this<CollisionDetector>
{
public:

  friend class CollisionObject;
  friend class CollisionGroup;

  using Factory = common::Factory<
      std::string, CollisionDetector, std::shared_ptr<CollisionDetector>>;

  using SingletonFactory = common::Singleton<Factory>;

  template <typename Derived>
  using Registrar = common::FactoryRegistrar<
      std::string, CollisionDetector, Derived, std::shared_ptr<CollisionDetector>>;

  /// Returns the singleton factory.
  static Factory* getFactory();

  /// Destructor
  virtual ~CollisionDetector() = default;

  /// \brief Create a clone of this CollisionDetector. All the properties will
  /// be copied over, but not collision objects.
  virtual std::shared_ptr<CollisionDetector> cloneWithoutCollisionObjects() = 0;

  /// Return collision detection engine type as a std::string
  virtual const std::string& getType() const = 0;

  /// Create a collision group
  virtual std::unique_ptr<CollisionGroup> createCollisionGroup() = 0;

  /// Helper function that creates and returns CollisionGroup as a shared_ptr.
  ///
  /// Internally, this function creates a shared_ptr from unique_ptr returned
  /// from createCollisionGroup() so the performance would be slighly worse than
  /// using std::make_unique.
  std::shared_ptr<CollisionGroup> createCollisionGroupAsSharedPtr();

  /// Create a collision group from any objects that are supported by
  /// CollisionGroup::addShapeFramesOf().
  ///
  /// The objects can be any of ShapeFrame, std::vector<ShapeFrame>,
  /// CollisionGroup, BodyNode, and Skeleton.
  ///
  /// Note that this function adds only the ShapeFrames of each object at the
  /// moment that this function is called. Any later addition to or removal of
  /// the ShapeFrames that are attached to these objects will NOT be noticed.
  template <typename... Args>
  std::unique_ptr<CollisionGroup> createCollisionGroup(const Args&... args);

  /// Helper function that creates and returns CollisionGroup as shared_ptr.
  template <typename... Args>
  std::shared_ptr<CollisionGroup> createCollisionGroupAsSharedPtr(
      const Args&... args);

  /// Perform collision check for a single group. If nullptr is passed to
  /// result, then the this returns only simple information whether there is a
  /// collision of not.
  virtual bool collide(
      CollisionGroup* group,
      const CollisionOption& option = CollisionOption(false, 1u, nullptr),
      CollisionResult* result = nullptr) = 0;

  /// Perform collision check for two groups. If nullptr is passed to
  /// result, then the this returns only simple information whether there is a
  /// collision of not.
  virtual bool collide(
      CollisionGroup* group1,
      CollisionGroup* group2,
      const CollisionOption& option = CollisionOption(false, 1u, nullptr),
      CollisionResult* result = nullptr) = 0;

  /// Get the minimum signed distance between the Shape pairs in the given
  /// CollisionGroup.
  ///
  /// The detailed results are stored in the given DistanceResult if provided.
  ///
  /// The results can be different by DistanceOption. By default, non-negative
  /// minimum distance (distance >= 0) is returned for all the shape pairs
  /// without computing nearest points.
  virtual double distance(
      CollisionGroup* group,
      const DistanceOption& option = DistanceOption(false, 0.0, nullptr),
      DistanceResult* result = nullptr) = 0;

  /// Get the minimum signed distance between the Shape pairs where a pair
  /// consist of two shapes from each groups (one from group1 and one from
  /// group2).
  ///
  /// Note that the distance between shapes within the same CollisionGroup
  /// are not accounted.
  ///
  /// The detailed results are stored in the given DistanceResult if provided.
  ///
  /// The results can be different by DistanceOption. By default, non-negative
  /// minimum distance (distance >= 0) is returned for all the shape pairs
  /// without computing nearest points.
  virtual double distance(
      CollisionGroup* group1,
      CollisionGroup* group2,
      const DistanceOption& option = DistanceOption(false, 0.0, nullptr),
      DistanceResult* result = nullptr) = 0;

protected:

  class CollisionObjectManager;
  class ManagerForUnsharableCollisionObjects;
  class ManagerForSharableCollisionObjects;

  /// Constructor
  CollisionDetector() = default;

  /// Claim CollisionObject associated with shapeFrame. New CollisionObject
  /// will be created if it hasn't created yet for shapeFrame.
  std::shared_ptr<CollisionObject> claimCollisionObject(
      const dynamics::ShapeFrame* shapeFrame);

  /// Create CollisionObject
  virtual std::unique_ptr<CollisionObject> createCollisionObject(
      const dynamics::ShapeFrame* shapeFrame) = 0;

  /// Update the collision geometry of a ShapeFrame
  virtual void refreshCollisionObject(CollisionObject* object) = 0;

  /// Notify that a CollisionObject is destroying. Do nothing by default.
  virtual void notifyCollisionObjectDestroying(CollisionObject* object);

protected:

  std::unique_ptr<CollisionObjectManager> mCollisionObjectManager;

};

//==============================================================================
class CollisionDetector::CollisionObjectManager
{
public:

  /// Constructor
  CollisionObjectManager(CollisionDetector* cd);

  /// Claim CollisionObject associated with shapeFrame. New CollisionObject
  /// will be created if it hasn't created yet for shapeFrame.
  virtual std::shared_ptr<CollisionObject> claimCollisionObject(
      const dynamics::ShapeFrame* shapeFrame) = 0;

  /// Returns collision detector
  CollisionDetector* getCollisionDetector();

  /// Virtual destructor
  virtual ~CollisionObjectManager() = default;

protected:

  CollisionDetector* mCollisionDetector;

};

//==============================================================================
class CollisionDetector::ManagerForUnsharableCollisionObjects final :
    public CollisionDetector::CollisionObjectManager
{
public:

  /// Constructor
  ManagerForUnsharableCollisionObjects(CollisionDetector* cd);

  // Documentation inherited
  std::shared_ptr<CollisionObject> claimCollisionObject(
      const dynamics::ShapeFrame* shapeFrame);

private:

  /// This deleter is responsible for deleting CollisionObject and removing it
  /// from mCollisionObjectMap when it is not shared by any CollisionGroups.
  struct CollisionObjectDeleter final
  {
    ManagerForUnsharableCollisionObjects* mCollisionObjectManager;

    CollisionObjectDeleter(ManagerForUnsharableCollisionObjects* mgr);

    void operator()(CollisionObject* object) const;
  };

  const CollisionObjectDeleter mCollisionObjectDeleter;

};

//==============================================================================
class CollisionDetector::ManagerForSharableCollisionObjects final :
    public CollisionDetector::CollisionObjectManager
{
public:

  /// Constructor
  ManagerForSharableCollisionObjects(CollisionDetector* cd);

  /// Destructor
  virtual ~ManagerForSharableCollisionObjects();

  // Documentation inherited
  std::shared_ptr<CollisionObject> claimCollisionObject(
      const dynamics::ShapeFrame* shapeFrame);

private:

  /// This deleter is responsible for deleting CollisionObject and removing it
  /// from mCollisionObjectMap when it is not shared by any CollisionGroups.
  struct CollisionObjectDeleter final
  {
    ManagerForSharableCollisionObjects* mCollisionObjectManager;

    CollisionObjectDeleter(ManagerForSharableCollisionObjects* mgr);

    void operator()(CollisionObject* object) const;
  };

  const CollisionObjectDeleter mCollisionObjectDeleter;

  using CollisionObjectMap = std::map<const dynamics::ShapeFrame*,
                                      std::weak_ptr<CollisionObject>>;

  CollisionObjectMap mCollisionObjectMap;

};

}  // namespace collision
}  // namespace dart

#include "dart/collision/detail/CollisionDetector.hpp"

#endif  // DART_COLLISION_COLLISIONDETECTOR_HPP_
