/*
 * Copyright (c) 2014-2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2014-2016, Humanoid Lab, Georgia Tech Research Corporation
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

#ifndef DART_DYNAMICS_ENTITY_HPP_
#define DART_DYNAMICS_ENTITY_HPP_

#include <Eigen/Core>
#include <string>
#include <vector>

#include "dart/common/Subject.hpp"
#include "dart/common/Signal.hpp"
#include "dart/common/Composite.hpp"
#include "dart/dynamics/Shape.hpp"
#include "dart/dynamics/SmartPointer.hpp"

namespace dart {
namespace dynamics {

class Frame;

/// Entity class is a base class for any objects that exist in the kinematic
/// tree structure of DART.
///
/// Entities all share the following properties: they exist within a reference
/// frame, have a name, and are visualizable. However, different Entity types
/// may have different policies about how/if their reference frame or name
/// can be changed. Use the Detachable class to create an Entity whose reference
/// Frame can be changed arbitrarily.
class Entity : public virtual common::Subject
{
public:
  friend class Frame;

  using EntitySignal = common::Signal<void(const Entity*)>;
  using FrameChangedSignal
      = common::Signal<void(const Entity*,
                            const Frame* _oldFrame,
                            const Frame* _newFrame)>;
  using NameChangedSignal
      = common::Signal<void(const Entity*,
                            const std::string& _oldName,
                            const std::string& _newName)>;

  /// Constructor for typical usage
  explicit Entity(Frame* _refFrame, bool _quiet);

  /// Default constructor, delegates to Entity(ConstructAbstract_t)
  Entity();

  Entity(const Entity&) = delete;

  /// Destructor
  virtual ~Entity();

  /// Set name. Some implementations of Entity may make alterations to the name
  /// that gets passed in. The final name that this entity will use gets passed
  /// back in the return of this function.
  virtual const std::string& setName(const std::string& _name) = 0;

  /// Return the name of this Entity
  virtual const std::string& getName() const = 0;

  /// Get the parent (reference) frame of this Entity
  Frame* getParentFrame();

  /// Get the parent (reference) frame of this Entity
  const Frame* getParentFrame() const;

  /// True if and only if this Entity depends on (i.e. kinematically descends
  /// from) _someFrame. If _someFrame is nullptr, this returns true in order to
  /// accommodate BodyNodes which always have a nullptr BodyNode as the parent
  /// of a root BodyNode.
  bool descendsFrom(const Frame* _someFrame) const;

  /// True iff this Entity is also a Frame.
  bool isFrame() const;

  /// Returns true if this Entity is set to be quiet.
  ///
  /// A quiet entity is unknown to its parent Frame. It will not be tracked by
  /// its parent; it will not receive notifications from its parent, and it will
  /// not be rendered. The advantage to a quiet Entity is that it has less
  /// overhead when constructing and deconstructing, which makes it more
  /// suitable for temporary objects.
  bool isQuiet() const;

  /// Notify this Entity that its parent Frame's pose has changed
  virtual void notifyTransformUpdate();

  /// Returns true iff a transform update is needed for this Entity
  bool needsTransformUpdate() const;

  /// Notify this Entity that its parent Frame's velocity has changed
  virtual void notifyVelocityUpdate();

  /// Returns true iff a velocity update is needed for this Entity
  bool needsVelocityUpdate() const;

  /// Notify this Entity that its parent Frame's acceleration has changed
  virtual void notifyAccelerationUpdate();

  /// Returns true iff an acceleration update is needed for this Entity
  bool needsAccelerationUpdate() const;

protected:

  /// Used when constructing a Frame class, because the Frame constructor will
  /// take care of setting up the parameters you pass into it
  enum ConstructFrameTag { ConstructFrame };

  explicit Entity(ConstructFrameTag);

  /// Used when constructing a pure abstract class, because calling the Entity
  /// constructor is just a formality
  enum ConstructAbstractTag { ConstructAbstract };

  explicit Entity(ConstructAbstractTag);

  /// Used by derived classes to change their parent frames
  virtual void changeParentFrame(Frame* _newParentFrame);

protected:

  /// Parent frame of this Entity
  Frame* mParentFrame;

  /// Does this Entity need a Transform update
  mutable bool mNeedTransformUpdate;

  /// Does this Entity need a Velocity update
  mutable bool mNeedVelocityUpdate;

  /// Does this Entity need an Acceleration update
  mutable bool mNeedAccelerationUpdate;

  /// Frame changed signal
  FrameChangedSignal mFrameChangedSignal;

  /// Name changed signal
  NameChangedSignal mNameChangedSignal;

  /// Transform changed signal
  EntitySignal mTransformUpdatedSignal;

  /// Velocity changed signal
  EntitySignal mVelocityChangedSignal;

  /// Acceleration changed signal
  EntitySignal mAccelerationChangedSignal;

public:
  //----------------------------------------------------------------------------
  /// \{ \name Slot registers
  //----------------------------------------------------------------------------

  /// Slot register for frame changed signal
  common::SlotRegister<FrameChangedSignal> onFrameChanged;

  /// Slot register for name changed signal
  common::SlotRegister<NameChangedSignal> onNameChanged;

  /// Slot register for transform updated signal
  common::SlotRegister<EntitySignal> onTransformUpdated;

  /// Slot register for velocity updated signal
  common::SlotRegister<EntitySignal> onVelocityChanged;

  /// Slot register for acceleration updated signal
  common::SlotRegister<EntitySignal> onAccelerationChanged;

private:
  /// Whether or not this Entity is set to be quiet
  const bool mAmQuiet;

  /// Whether or not this Entity is a Frame
  bool mAmFrame;
};

/// The Detachable class is a special case of the Entity base class. Detachable
/// allows the Entity's reference Frame to be changed arbitrarily by the user.
class Detachable : public virtual Entity
{
public:
  /// Constructor
  explicit Detachable(Frame* _refFrame, bool _quiet);

  /// Allows the user to change the parent Frame of this Entity
  virtual void setParentFrame(Frame* _newParentFrame);

protected:
  /// Constructor for inheriting classes, so they do not need to fill in the
  /// arguments
  Detachable();

};

} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_ENTITY_HPP_
