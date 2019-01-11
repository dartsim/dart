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

#ifndef DART_DYNAMICS_FRAME_HPP_
#define DART_DYNAMICS_FRAME_HPP_

#include <set>

#include <Eigen/Geometry>

#include "dart/dynamics/Entity.hpp"
#include "dart/math/MathTypes.hpp"

namespace dart {
namespace dynamics {

/// The Frame class serves as the backbone of DART's kinematic tree structure.
///
/// Frame inherits Entity, so it exists within a reference Frame. This class
/// keeps track of both its local (relative) and global (world) transforms,
/// velocities, and accelerations. It also notifies every child Entity when
/// a transform, velocity, or acceleration has changed locally or globally.
///
/// Entity class is inherited by using virtual inheritence to solve the
/// so-called "diamond problem". Because of that, the Entity's constructor will
/// be called directly by the most derived class's constructor.
class Frame : public virtual Entity
{
public:
  friend class Entity;
  friend class WorldFrame;
  friend class ShapeFrame;

  Frame(const Frame&) = delete;

  /// Destructor
  virtual ~Frame();

  static Frame* World();

  //--------------------------------------------------------------------------
  // Transform
  //--------------------------------------------------------------------------

  /// Get the transform of this Frame with respect to its parent Frame
  virtual const Eigen::Isometry3d& getRelativeTransform() const = 0;

  /// Get the transform of this Frame with respect to the World Frame
  const Eigen::Isometry3d& getWorldTransform() const;

  /// Get the transform of this Frame with respect to some other Frame
  Eigen::Isometry3d getTransform(
      const Frame* _withRespectTo = Frame::World()) const;

  /// Get the transform of this Frame with respect to some other Frame. It can
  /// be expressed in the coordinates of any Frame.
  Eigen::Isometry3d getTransform(const Frame* withRespectTo,
                                 const Frame* inCoordinatesOf) const;

  //-------------------------------------------------------------------------
  // Velocity
  //-------------------------------------------------------------------------

  /// Get the spatial velocity of this Frame relative to its parent Frame, in
  /// its own coordinates.
  virtual const Eigen::Vector6d& getRelativeSpatialVelocity() const = 0;

  /// Get the total spatial velocity of this Frame in the coordinates of this
  /// Frame.
  const Eigen::Vector6d& getSpatialVelocity() const;

  /// Get the spatial velocity of this Frame relative to some other Frame. It
  /// can be expressed in the coordinates of any Frame.
  Eigen::Vector6d getSpatialVelocity(const Frame* _relativeTo,
                                     const Frame* _inCoordinatesOf) const;

  /// Get the spatial velocity of a fixed point in this Frame. The velocity is
  /// in coordinates of this Frame and is relative to the World Frame.
  Eigen::Vector6d getSpatialVelocity(const Eigen::Vector3d& _offset) const;

  /// Get the spatial velocity of a fixed point in this Frame.
  Eigen::Vector6d getSpatialVelocity(const Eigen::Vector3d& _offset,
                                     const Frame* _relativeTo,
                                     const Frame* _inCoordinatesOf) const;

  /// Get the linear portion of classical velocity of this Frame relative to
  /// some other Frame. It can be expressed in the coordinates of any Frame.
  Eigen::Vector3d getLinearVelocity(
      const Frame* _relativeTo = Frame::World(),
      const Frame* _inCoordinatesOf = Frame::World()) const;

  /// Get the linear velocity of a point that is fixed in this Frame. You can
  /// specify a relative Frame, and it can be expressed in the coordinates of
  /// any Frame.
  Eigen::Vector3d getLinearVelocity(
      const Eigen::Vector3d& _offset,
      const Frame* _relativeTo = Frame::World(),
      const Frame* _inCoordinatesOf = Frame::World()) const;

  /// Get the angular portion of classical velocity of this Frame relative to
  /// some other Frame. It can be expressed in the coordinates of any Frame.
  Eigen::Vector3d getAngularVelocity(
      const Frame* _relativeTo = Frame::World(),
      const Frame* _inCoordinatesOf = Frame::World()) const;

  //--------------------------------------------------------------------------
  // Acceleration
  //--------------------------------------------------------------------------

  /// Get the spatial acceleration of this Frame relative to its parent Frame,
  /// in the coordinates of this Frame.
  virtual const Eigen::Vector6d& getRelativeSpatialAcceleration() const = 0;

  /// The Featherstone ABI algorithm exploits a component of the spatial
  /// acceleration which we refer to as the partial acceleration, accessible
  /// by getPartialAcceleration(). We save operations during our forward
  /// kinematics by computing and storing the partial acceleration separately
  /// from the rest of the Frame's acceleration. getPrimaryRelativeAcceleration()
  /// will return the portion of the relative spatial acceleration that is not
  /// contained in the partial acceleration. To get the full spatial
  /// acceleration of this Frame relative to its parent Frame, use
  /// getRelativeSpatialAcceleration(). To get the full spatial acceleration
  /// of this Frame relative to the World Frame, use getSpatialAcceleration().
  virtual const Eigen::Vector6d& getPrimaryRelativeAcceleration() const = 0;

  /// The Featherstone ABI algorithm exploits a component of the spatial
  /// acceleration which we refer to as the partial acceleration. This function
  /// returns that component of acceleration.
  virtual const Eigen::Vector6d& getPartialAcceleration() const = 0;

  /// Get the total spatial acceleration of this Frame in the coordinates of
  /// this Frame.
  const Eigen::Vector6d& getSpatialAcceleration() const;

  /// Get the spatial acceleration of this Frame relative to some other Frame.
  /// It can be expressed in the coordinates of any Frame.
  Eigen::Vector6d getSpatialAcceleration(const Frame* _relativeTo,
                                         const Frame* _inCoordinatesOf) const;

  /// Get the spatial acceleration of a fixed point in this Frame. The
  /// acceleration is in coordinates of this Frame and is relative to the World
  /// Frame.
  Eigen::Vector6d getSpatialAcceleration(const Eigen::Vector3d& _offset) const;

  /// Get the spatial acceleration of a fixed point in this Frame
  Eigen::Vector6d getSpatialAcceleration(const Eigen::Vector3d& _offset,
                                         const Frame* _relativeTo,
                                         const Frame* _inCoordinatesOf) const;

  /// Get the linear portion of classical acceleration of this Frame relative to
  /// some other Frame. It can be expressed in the coordinates of any Frame.
  Eigen::Vector3d getLinearAcceleration(
      const Frame* _relativeTo=Frame::World(),
      const Frame* _inCoordinatesOf=Frame::World()) const;

  Eigen::Vector3d getLinearAcceleration(
      const Eigen::Vector3d& _offset,
      const Frame* _relativeTo=Frame::World(),
      const Frame* _inCoordinatesOf=Frame::World()) const;

  /// Get the angular portion of classical acceleration of this Frame relative
  /// to some other Frame. It can be expressed in the coordinates of any Frame.
  Eigen::Vector3d getAngularAcceleration(
      const Frame* _relativeTo=Frame::World(),
      const Frame* _inCoordinatesOf=Frame::World()) const;

  //--------------------------------------------------------------------------
  // Relationships
  //--------------------------------------------------------------------------

  /// Get a container with the Entities that are children of this Frame.
  /// std::set is used because Entities may be arbitrarily added and removed
  /// from a parent Frame, and each entry should be unique. std::set makes this
  /// procedure easier.
  const std::set<Entity*>& getChildEntities();

  /// Get a container with the Entities that are children of this Frame. Note
  /// that this is version is slightly less efficient than the non-const version
  /// because it needs to rebuild a set where each pointer is converted to be a
  /// const pointer.
  const std::set<const Entity*> getChildEntities() const;

  /// Get the number of Entities that are currently children of this Frame.
  std::size_t getNumChildEntities() const;

  /// Get a container with the Frames that are children of this Frame.
  /// std::set is used because Frames may be arbitrarily added and removed
  /// from a parent Frame, and each entry should be unique.
  const std::set<Frame*>& getChildFrames();

  /// Get a container with the Frames that are children of this Frame. Note
  /// that this version is less efficient than the non-const version because
  /// it needs to rebuild a set so that the entries are const.
  std::set<const Frame*> getChildFrames() const;

  /// Get the number of Frames that are currently children of this Frame.
  std::size_t getNumChildFrames() const;

  /// Returns true if this Frame is a ShapeFrame
  bool isShapeFrame() const;

  /// Convert 'this' into a ShapeFrame pointer if Frame is a ShapeFrame,
  /// otherwise return nullptr
  virtual ShapeFrame* asShapeFrame();

  /// Convert 'const this' into a ShapeFrame pointer if Frame is a ShapeFrame,
  /// otherwise return nullptr
  virtual const ShapeFrame* asShapeFrame() const;

  /// Returns true if this Frame is the World Frame
  bool isWorld() const;

  /// Notify the transformation updates of this Frame and all its children are
  /// needed
  virtual void dirtyTransform() override;

  /// Notify the velocity updates of this Frame and all its children are needed
  virtual void dirtyVelocity() override;

  /// Notify the acceleration updates of this Frame and all its children are
  /// needed
  virtual void dirtyAcceleration() override;

protected:

  /// Used when constructing a pure abstract class, because calling the Frame
  /// constructor is just a formality
  enum ConstructAbstractTag { ConstructAbstract };

  /// Constructor for typical usage
  explicit Frame(Frame* _refFrame);

  /// Default constructor, delegates to Frame(ConstructAbstract_t)
  Frame();

  /// Constructor for use by pure abstract classes
  explicit Frame(ConstructAbstractTag);

  // Documentation inherited
  virtual void changeParentFrame(Frame* _newParentFrame) override;

  /// Called during a parent Frame change to allow extensions of the Frame class
  /// to handle new children in customized ways. This function is a no op unless
  /// an inheriting class (such as BodyNode) overrides it.
  virtual void processNewEntity(Entity* _newChildEntity);

  /// Called when a child Entity is removed from its parent Frame. This allows
  /// special post-processing to be performed for extensions of the Frame class.
  virtual void processRemovedEntity(Entity* _oldChildEntity);

private:

  /// Used when constructing the World
  enum ConstructWorldTag { ConstructWorld };

  /// Constructor only to be used by the WorldFrame class
  explicit Frame(ConstructWorldTag);

protected:
  /// World transform of this Frame. This object is mutable to enable
  /// auto-updating to happen in the const member getWorldTransform() function
  ///
  /// Do not use directly! Use getWorldTransform() to access this quantity
  mutable Eigen::Isometry3d mWorldTransform;

  /// Total velocity of this Frame, in the coordinates of this Frame
  ///
  /// Do not use directly! Use getSpatialVelocity() to access this quantity
  mutable Eigen::Vector6d mVelocity;

  /// Total acceleration of this Frame, in the coordinates of this Frame
  ///
  /// Do not use directly! Use getSpatialAcceleration() to access this quantity
  mutable Eigen::Vector6d mAcceleration;

  /// Container of this Frame's child Frames.
  std::set<Frame*> mChildFrames;

  /// Container of this Frame's child Entities.
  std::set<Entity*> mChildEntities;

private:
  /// Contains whether or not this is the World Frame
  const bool mAmWorld;

  /// Contains whether or not this is a ShapeFrame
  bool mAmShapeFrame;

public:
  // To get byte-aligned Eigen vectors
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/// The WorldFrame class is a class that is used internally to create the
/// singleton World Frame. This class cannot be instantiated directly: you must
/// use the Frame::World() function to access it. Only one World Frame exists
/// in any application.
class WorldFrame : public Frame
{
public:
  friend class Frame;

  /// Always returns the Identity Transform
  const Eigen::Isometry3d& getRelativeTransform() const override final;

  /// Always returns a zero vector
  const Eigen::Vector6d& getRelativeSpatialVelocity() const override final;

  /// Always returns a zero vector
  const Eigen::Vector6d& getRelativeSpatialAcceleration() const override final;

  /// Always returns a zero vector
  const Eigen::Vector6d& getPrimaryRelativeAcceleration() const override final;

  /// Always returns a zero vector
  const Eigen::Vector6d& getPartialAcceleration() const override final;

  const std::string& setName(const std::string& name) override final;

  const std::string& getName() const override final;

private:
  /// This may only be constructed by the Frame class
  explicit WorldFrame();

private:
  /// This is set to Identity and never changes
  const Eigen::Isometry3d mRelativeTf;

  /// This is set to a Zero vector and never changes
  static const Eigen::Vector6d mZero;

public:
  // To get byte-aligned Eigen vectors
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_FRAME_HPP_
