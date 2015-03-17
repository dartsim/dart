/*
 * Copyright (c) 2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Michael X. Grey <mxgrey@gatech.edu>
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

#ifndef OSGDART_DRAGANDDROP_H
#define OSGDART_DRAGANDDROP_H

#include <functional>

#include <Eigen/Geometry>

#include "dart/common/sub_ptr.h"
#include "dart/dynamics/Entity.h"
#include "dart/dynamics/Shape.h"
#include "DefaultEventHandler.h"

namespace dart {
namespace dynamics {
class SimpleFrame;
} // namespace dynamics
} // namespace dart

namespace osgDart
{

class Viewer;
class InteractiveFrame;

/// DragAndDrop is a class that facilitates enabling various kinds of dart
/// Entities to be dragged and dropped in an osgDart environment
class DragAndDrop : public dart::common::Publisher,
                    public dart::common::Subscriber
{
public:

  DragAndDrop(Viewer* viewer, dart::dynamics::Entity* entity);

  virtual ~DragAndDrop();

  /// Get the Entity that this DragAndDrop is associated with
  dart::dynamics::Entity* getEntity() const;

  /// Called when mouse events are being handled
  virtual void update();

  /// If true is passed in, this object will only be drag and droppable if there
  /// are no other objects between its picked point and the camera.
  void setObstructable(bool _obstructable);

  /// If this is true, then this object will only be drag and droppable if there
  /// are no other objects between its picked point and the camera. Default is
  /// true.
  bool isObstructable() const;

  /// Called to specify how the Entity should be moved
  virtual void move() = 0;

  /// Called when a point gets picked, and is used to save the current state of
  /// the Entity
  virtual void saveState() = 0;

  /// Default method for getting the translation requested by the user
  virtual Eigen::Vector3d getConstrainedDx() const;

  /// Default method for getting the rotation requested by the user
  virtual Eigen::AngleAxisd getConstrainedRotation() const;

  /// Remove all constraints from the dragging and dropping.
  void unconstrain();

  /// Constrain translation to only occur along the given slope, or constrain
  /// rotation to only occur about the given slope. For rotation, this function
  /// is equivalent to constrainToPlane
  void constrainToLine(const Eigen::Vector3d& slope);

  /// Constrain translation to only occur within the plane defined by the given
  /// normal, or constrain rotation to only occur about the given normal. For
  /// rotation, this function is equivalent to constrainToLine
  void constrainToPlane(const Eigen::Vector3d& normal);

  /// Returns the mAmMoving flag
  bool isMoving() const;

protected:

  virtual void handleDestructionNotification(
      const dart::common::Publisher* subscription) override;

  Viewer* mViewer;

  dart::dynamics::Entity* mEntity;

  Eigen::Vector3d mPickedPosition;

  /// Reference vector for constraint (slope for line constraint, or normal for
  /// plane constraint)
  Eigen::Vector3d mVector;

  /// Point in space about which rotations should happen
  Eigen::Vector3d mPivot;

  ConstraintType mConstraintType;

  bool mAmObstructable;

  bool mAmMoving;

};

//==============================================================================
/// SimpleFrameDnD is a DragAndDrop implementation for SimpleFrame objects
class SimpleFrameDnD : public DragAndDrop
{
public:

  enum class RotationOption : int {

    HOLD_CTRL = 0, // Default setting
    ALWAYS_ON,
    ALWAYS_OFF

  };

  SimpleFrameDnD(Viewer* viewer, dart::dynamics::SimpleFrame* frame);

  ~SimpleFrameDnD();

  dart::dynamics::SimpleFrame* getSimpleFrame() const;

  virtual void move() override;

  virtual void saveState() override;

  void setRotationOption(RotationOption option);

protected:

  RotationOption mOption;

  dart::dynamics::SimpleFrame* mFrame;

  Eigen::AngleAxisd mSavedRotation;
};

//==============================================================================
/// SimpleFrameShapeDnD is a version of SimpleFrameDnD that allows a specific
/// Shape within the SimpleFrame to be dragged and dropped (although it will
/// carry the entire SimpleFrame with it)
class SimpleFrameShapeDnD : public SimpleFrameDnD
{
public:

  SimpleFrameShapeDnD(Viewer* viewer, dart::dynamics::SimpleFrame* frame,
                      dart::dynamics::Shape* shape);

  ~SimpleFrameShapeDnD();

  dart::dynamics::Shape* getShape() const;

  virtual void update() override;

protected:

  virtual void handleDestructionNotification(
      const dart::common::Publisher* subscription) override;

  dart::dynamics::Shape* mShape;
};

//==============================================================================
class InteractiveFrameDnD : public DragAndDrop
{
public:

  InteractiveFrameDnD(Viewer* viewer, osgDart::InteractiveFrame* frame);

  virtual ~InteractiveFrameDnD();

  InteractiveFrame* getFrame() const;

  virtual void update() override;

  virtual void move() override;

  virtual void saveState() override;

protected:

  std::vector<SimpleFrameShapeDnD*> mDnDs;

  InteractiveFrame* mInteractiveFrame;

};

} // namespace osgDart


#endif // OSGDART_DRAGANDDROP_H
