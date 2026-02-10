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

#ifndef DART_COLLISION_COLLISIONGROUP_HPP_
#define DART_COLLISION_COLLISIONGROUP_HPP_

#include <dart/collision/collision_fwd.hpp>
#include <dart/collision/collision_option.hpp>
#include <dart/collision/collision_result.hpp>
#include <dart/collision/distance_option.hpp>
#include <dart/collision/distance_result.hpp>
#include <dart/collision/fwd.hpp>
#include <dart/collision/raycast_option.hpp>
#include <dart/collision/raycast_result.hpp>

#include <dart/common/observer.hpp>

#include <dart/export.hpp>

#include <span>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace dart {
namespace collision {

class DART_API CollisionGroup
{
public:
  /// Constructor
  CollisionGroup(const CollisionDetectorPtr& collisionDetector);
  // CollisionGroup also can be created from CollisionDetector::create()

  CollisionGroup(const CollisionGroup&) = delete;
  CollisionGroup& operator=(const CollisionGroup&) = delete;

  CollisionGroup(CollisionGroup&&) noexcept;
  CollisionGroup& operator=(CollisionGroup&&) noexcept;

  /// Destructor
  virtual ~CollisionGroup();

  /// Return collision detection engine associated with this CollisionGroup
  CollisionDetectorPtr getCollisionDetector();

  /// Return (const) collision detection engine associated with this
  /// CollisionGroup
  ConstCollisionDetectorPtr getCollisionDetector() const;

  /// Add a ShapeFrame to this CollisionGroup
  void addShapeFrame(const dynamics::ShapeFrame* shapeFrame);

  /// Add ShapeFrames to this CollisionGroup
  void addShapeFrames(std::span<const dynamics::ShapeFrame* const> shapeFrames);

  /// Add a ShapeFrame, and also add ShapeFrames of other various objects.
  ///
  /// The other various objects can be any of ShapeFrame,
  /// std::vector<ShapeFrame>, CollisionGroup, BodyNode, and Skeleton.
  ///
  /// Note that this function adds only the ShapeFrames of each object at the
  /// moment of this function is called. The aftwerward changes of the objects
  /// does not affect on this CollisionGroup.
  template <typename... Others>
  void addShapeFramesOf(
      const dynamics::ShapeFrame* shapeFrame, const Others*... others);

  /// Add ShapeFrames, and also add ShapeFrames of other various objects.
  template <typename... Others>
  void addShapeFramesOf(
      std::span<const dynamics::ShapeFrame* const> shapeFrames,
      const Others*... others);

  /// Add ShapeFrames of other CollisionGroup, and also add another ShapeFrames
  /// of other various objects.
  template <typename... Others>
  void addShapeFramesOf(
      const CollisionGroup* otherGroup, const Others*... others);

  /// Add ShapeFrames of BodyNode, and also add another ShapeFrames of other
  /// various objects.
  template <typename... Others>
  void addShapeFramesOf(
      const dynamics::BodyNode* bodyNode, const Others*... others);

  /// Add ShapeFrames of MetaSkeleton, and also add another ShapeFrames of other
  /// various objects.
  template <typename... Others>
  void addShapeFramesOf(
      const dynamics::MetaSkeleton* skeleton, const Others*... others);

  /// Do nothing. This function is for terminating the recursive variadic
  /// template function template<typename...> addShapeFramesOf().
  void addShapeFramesOf();

  /// Add ShapeFrames of bodyNode, and also subscribe to the BodyNode so that
  /// the results from this CollisionGroup automatically reflect any changes
  /// that are made to bodyNode.
  ///
  /// This does likewise for the objects in ...others.
  template <typename... Others>
  void subscribeTo(
      const dynamics::ConstBodyNodePtr& bodyNode, const Others&... others);

  /// Add ShapeFrames of metaSkeleton, and also subscribe so that the results
  /// from this CollisionGroup automatically reflect any changes that are made
  /// to metaSkeleton.
  ///
  /// This does likewise for the objects in ...others.
  template <typename... Others>
  void subscribeTo(
      const dynamics::ConstMetaSkeletonPtr& metaSkeleton,
      const Others&... others);

  /// Do nothing. This function is for terminating the recursive variadic
  /// template.
  void subscribeTo();

  void addShapeFramesOfBodyNode(const dynamics::BodyNode* bodyNode);
  void addShapeFramesOfMetaSkeleton(const dynamics::MetaSkeleton* skeleton);
  void addShapeFramesOfGroup(const CollisionGroup* otherGroup);

  void subscribeToBodyNode(const dynamics::ConstBodyNodePtr& bodyNode);
  void subscribeToMetaSkeleton(
      const dynamics::ConstMetaSkeletonPtr& metaSkeleton);

  /// Remove a ShapeFrame from this CollisionGroup. If this ShapeFrame was being
  /// provided by any subscriptions, then calling this function will unsubscribe
  /// from those subscriptions, because otherwise this ShapeFrame would simply
  /// be put back into the CollisionGroup the next time the group gets updated.
  void removeShapeFrame(const dynamics::ShapeFrame* shapeFrame);

  /// Remove ShapeFrames from this CollisionGroup
  void removeShapeFrames(
      std::span<const dynamics::ShapeFrame* const> shapeFrames);

  /// Remove a ShapeFrame, and also remove ShapeFrames of other various objects.
  ///
  /// The other various objects can be any of ShapeFrame,
  /// std::vector<ShapeFrame>, CollisionGroup, BodyNode, and Skeleton.
  ///
  /// Note that this function removes only the ShapeFrames of each object at the
  /// moment of this function is called. The aftwerward changes of the objects
  /// does not affect on this CollisionGroup.
  template <typename... Others>
  void removeShapeFramesOf(
      const dynamics::ShapeFrame* shapeFrame, const Others*... others);

  /// Remove ShapeFrames, and also remove ShapeFrames of other various objects.
  template <typename... Others>
  void removeShapeFramesOf(
      std::span<const dynamics::ShapeFrame* const> shapeFrames,
      const Others*... others);

  /// Remove ShapeFrames of other CollisionGroup, and also remove another
  /// ShapeFrames of other various objects.
  ///
  /// If this CollisionGroup is subscribed to otherGroup, then this function
  /// will also unsubscribe from it.
  template <typename... Others>
  void removeShapeFramesOf(
      const CollisionGroup* otherGroup, const Others*... others);

  /// Remove ShapeFrames of BodyNode, and also remove another ShapeFrames of
  /// other various objects.
  ///
  /// If this CollisionGroup is subscribed to bodyNode, then this function will
  /// also unsubscribe from it.
  template <typename... Others>
  void removeShapeFramesOf(
      const dynamics::BodyNode* bodyNode, const Others*... others);

  /// Remove ShapeFrames of MetaSkeleton, and also remove another ShapeFrames of
  /// other various objects.
  ///
  /// If this CollisionGroup is subscribed to skeleton, then this function will
  /// also unsubscribe from it.
  template <typename... Others>
  void removeShapeFramesOf(
      const dynamics::MetaSkeleton* skeleton, const Others*... others);

  /// Do nothing. This function is for terminating the recursive variadic
  /// template function template<typename...> removeShapeFramesOf().
  void removeShapeFramesOf();

  void removeShapeFramesOfBodyNode(const dynamics::BodyNode* bodyNode);
  void removeShapeFramesOfMetaSkeleton(const dynamics::MetaSkeleton* skeleton);
  void removeShapeFramesOfGroup(const CollisionGroup* otherGroup);

  /// Remove all the ShapeFrames in this CollisionGroup
  void removeAllShapeFrames();

  /// Unsubscribe from bodyNode. The ShapeFrames of the BodyNode will also be
  /// removed if no other source is requesting them for this group.
  template <typename... Others>
  void unsubscribeFrom(
      const dynamics::BodyNode* bodyNode, const Others*... others);

  /// Unsubscribe from skeleton. The ShapeFrames of the skeleton will also be
  /// removed if no other source is requesting them for this group.
  template <typename... Others>
  void unsubscribeFrom(
      const dynamics::MetaSkeleton* skeleton, const Others*... others);

  /// Do nothing. This function is for terminating the recursive variadic
  /// template.
  void unsubscribeFrom();

  void unsubscribeFromBodyNode(const dynamics::BodyNode* bodyNode);
  void unsubscribeFromMetaSkeleton(const dynamics::MetaSkeleton* skeleton);

  /// Check if this is subscribed to bodyNode and the other sources
  template <typename... Others>
  bool isSubscribedTo(
      const dynamics::BodyNode* bodyNode, const Others*... others);

  /// Check if this is subscribed to skeleton and the other sources
  template <typename... Others>
  bool isSubscribedTo(
      const dynamics::MetaSkeleton* skeleton, const Others*... others);

  /// Return true. This function is for terminating the recursive variadic
  /// template
  bool isSubscribedTo();

  bool isSubscribedToBodyNode(const dynamics::BodyNode* bodyNode);
  bool isSubscribedToMetaSkeleton(const dynamics::MetaSkeleton* skeleton);

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

  /// Performs raycast to this collision group.
  ///
  /// @param[in] from The start point of the ray in world coordinates.
  /// @param[in] to The end point of the ray in world coordinates.
  /// @param[in] option The raycast option.
  /// @param[in] result The raycast result.
  /// @return True if the ray hit an collision object.
  bool raycast(
      const Eigen::Vector3d& from,
      const Eigen::Vector3d& to,
      const RaycastOption& option = RaycastOption(),
      RaycastResult* result = nullptr);

  /// Set whether this CollisionGroup will automatically check for updates.
  void setAutomaticUpdate(bool automatic = true);

  /// Get whether this CollisionGroup is set to automatically check for updates.
  bool getAutomaticUpdate() const;

  /// Check whether this CollisionGroup's subscriptions or any of its objects
  /// need an update, and then update them if they do.
  ///
  /// If automatic updating is turned on, then this will be called each time
  /// collide() is.
  ///
  /// This can be called manually in order to control when collision objects get
  /// created or destroyed. Those procedures can be time-consuming, so it may be
  /// useful to control when it occurs.
  void update();

  /// Remove any ShapeFrames that have been deleted. This will be done
  /// automatically for ShapeFrames that belong to subscriptions.
  ///
  /// Returns true if one or more ShapeFrames were removed; otherwise returns
  /// false.
  void removeDeletedShapeFrames();

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
      std::span<CollisionObject* const> collObjects)
      = 0;

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

  /// Information on a collision object belonging to this group. This info
  /// enables us to keep track of when a ShapeFrame has changed and therefore
  /// when the object needs to be updated. It also remembers what sources
  /// instructed this object to belong to this group.
  struct ObjectInfo final
  {
    /// The ShapeFrame for this object
    const dynamics::ShapeFrame* mFrame;

    /// The CollisionObject that was generated by the CollisionDetector
    CollisionObjectPtr mObject;

    /// The ID of that last known shape that was held by the shape frame
    std::size_t mLastKnownShapeID;

    /// The last known version of the last known shape that was held by the
    /// shape frame
    std::size_t mLastKnownVersion;

    /// The set of all sources that indicate that this object should be in this
    /// group. In the current implementation, this may consist of:
    /// user (nullptr), Skeleton subscription, and/or BodyNode subscription.
    ///
    /// When all sources are cleared out (via unsubscribing), this object will
    /// be removed from this group.
    std::unordered_set<const void*> mSources;
  };

  using ObjectInfoList = std::vector<std::unique_ptr<ObjectInfo>>;

  /// Information about ShapeFrames and CollisionObjects that have been added to
  /// this CollisionGroup.
  ObjectInfoList mObjectInfoList;
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
  // original and copy are not guaranteed to be the same as we copy std::map
  // (e.g., by world cloning).

private:
  /// This class watches when ShapeFrames get deleted so that they can be safely
  /// removes from the CollisionGroup. We cannot have a weak_ptr to a ShapeFrame
  /// because some are managed by std::shared_ptr while others are managed by
  /// NodePtr.
  ///
  /// If we don't carefully track when each ShapeFrame gets destructed, then we
  /// can land in a situation where a ShapeFrame gets removed from a BodyNode,
  /// deallocated, and then a new ShapeFrame is allocated in the memory address
  /// of the old one. This can lead to invalid memory accesses if we neglect to
  /// correctly clean up our references to deleted ShapeFrames.
  class ShapeFrameObserver final : public common::Observer
  {
  public:
    explicit ShapeFrameObserver(CollisionGroup* group = nullptr);

    /// Add a shape frame to this observer
    void addShapeFrame(const dynamics::ShapeFrame* shapeFrame);

    /// Remove a shape frame from this observer
    void removeShapeFrame(const dynamics::ShapeFrame* shapeFrame);

    /// Remove all shape frames from this observer
    void removeAllShapeFrames();

    /// Whenever an observed shape frame gets deleted, its pointer will be added
    /// to this set. The next time the collision group updates, it will check
    /// this set and wipe away any references to the pointers in this set.
    std::unordered_set<const dynamics::ShapeFrame*> mDeletedFrames;

  protected:
    /// This will be called each time an observed shape frame is deleted.
    void handleDestructionNotification(const common::Subject* subject) override;

  private:
    CollisionGroup* mGroup;

    /// A map from a subject pointer to its corresponding ShapeFrame pointer.
    /// This needs to be stored because by the time a Subject is being
    /// destructed, it can no longer be cast back to its ShapeFrame.
    std::unordered_map<const common::Subject*, const dynamics::ShapeFrame*>
        mMap;
  };

  /// Implementation of addShapeFrame. The source argument tells us whether this
  /// ShapeFrame is being requested explicitly by the user or implicitly through
  /// a BodyNode, Skeleton, or other CollisionGroup.
  ObjectInfo* addShapeFrameImpl(
      const dynamics::ShapeFrame* shapeFrame, const void* source);

  /// Internal version of removeShapeFrame. This will only remove the ShapeFrame
  /// if it is unsubscribed from all sources.
  void removeShapeFrameInternal(
      const dynamics::ShapeFrame* shapeFrame, const void* source);

  friend class ShapeFrameObserver;
  friend struct Impl;
  void handleShapeFrameDestruction(const dynamics::ShapeFrame* shapeFrame);

  void updateSubscriptions();
  void eraseSkeletonSource(const dynamics::MetaSkeleton* source);
  void eraseBodyNodeSource(const dynamics::BodyNode* source);

  bool mUpdateAutomatically;

  struct Impl;
  std::unique_ptr<Impl> mImpl;

  ShapeFrameObserver mObserver;
};

} // namespace collision
} // namespace dart

#include <dart/collision/detail/collision_group.hpp>

#endif // DART_COLLISION_COLLISIONGROUP_HPP_
