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

#include "osgDart/DragAndDrop.h"
#include "osgDart/DefaultEventHandler.h"
#include "osgDart/Viewer.h"
#include "osgDart/InteractiveFrame.h"

#include "dart/dynamics/SimpleFrame.h"


#include <iostream>

namespace osgDart {

DragAndDrop::DragAndDrop(Viewer* viewer, dart::dynamics::Entity* entity)
  : mViewer(viewer),
    mEntity(entity),
    mPickedPosition(Eigen::Vector3d::Zero()),
    mConstraintType(UNCONSTRAINED),
    mAmObstructable(true),
    mAmMoving(false)
{
  addSubscription(mEntity);
  addSubscription(mViewer);
}

//==============================================================================
DragAndDrop::~DragAndDrop()
{
  // Do nothing
}

//==============================================================================
dart::dynamics::Entity* DragAndDrop::getEntity() const
{
  return mEntity;
}

//==============================================================================
void DragAndDrop::update()
{
  if(nullptr == mEntity)
    return;

  osgDart::MouseButtonEvent event =
      mViewer->getDefaultEventHandler()->getButtonEvent(LEFT_MOUSE);

  if(mAmMoving)
  {
    if(osgDart::BUTTON_RELEASE == event)
      mAmMoving = false;

    move();
  }
  else // not moving
  {
    if(osgDart::BUTTON_PUSH == event)
    {
      const std::vector<osgDart::PickInfo>& picks =
          mViewer->getDefaultEventHandler()->getButtonPicks(
            osgDart::LEFT_MOUSE, osgDart::BUTTON_PUSH);

      for(const osgDart::PickInfo& pick : picks)
      {
        if(pick.entity == mEntity)
        {
          mAmMoving = true;
          mPickedPosition = pick.position;
          saveState();
          return;
        }

        // The picks are always ordered from closest to furthest. If the closest
        // pick is not our Entity, then something is blocking the way, so if we
        // are obstructable, then we should quit.
        if(mAmObstructable)
          return;
      }
    }
  }
}

//==============================================================================
void DragAndDrop::setObstructable(bool _obstructable)
{
  mAmObstructable = _obstructable;
}

//==============================================================================
bool DragAndDrop::isObstructable() const
{
  return mAmObstructable;
}

//==============================================================================
Eigen::Vector3d DragAndDrop::getConstrainedDx() const
{
  return mViewer->getDefaultEventHandler()->getDeltaCursor(
        mPickedPosition, mConstraintType, mVector);
}

//==============================================================================
Eigen::AngleAxisd DragAndDrop::getConstrainedRotation() const
{
  Eigen::Vector3d v1 = mPickedPosition - mPivot;
  Eigen::Vector3d v2;

  if(LINE_CONSTRAINT == mConstraintType || PLANE_CONSTRAINT == mConstraintType)
  {
    v2 = mViewer->getDefaultEventHandler()->getDeltaCursor(
          mPickedPosition, PLANE_CONSTRAINT, mVector)
         + mPickedPosition - mPivot;
  }
  else
  {
    v2 = mViewer->getDefaultEventHandler()->getDeltaCursor(mPickedPosition)
         + mPickedPosition - mPivot;
  }

  if(v1.norm() == 0 || v2.norm() == 0 || v1.cross(v2).norm() == 0)
    return Eigen::AngleAxisd(0, Eigen::Vector3d(1,0,0));

  v1.normalize();
  v2.normalize();

  Eigen::Vector3d axis = v1.cross(v2);
  axis.normalize();

  return Eigen::AngleAxisd(acos(v1.dot(v2)), axis);
}

//==============================================================================
void DragAndDrop::unconstrain()
{
  mConstraintType = UNCONSTRAINED;
}

//==============================================================================
void DragAndDrop::constrainToLine(const Eigen::Vector3d& slope)
{
  mConstraintType = LINE_CONSTRAINT;
  mVector = slope;
}

//==============================================================================
void DragAndDrop::constrainToPlane(const Eigen::Vector3d& normal)
{
  mConstraintType = PLANE_CONSTRAINT;
  mVector = normal;
}

//==============================================================================
void DragAndDrop::handleDestructionNotification(
    const dart::common::Publisher* subscription)
{
  if(mEntity == subscription)
    mViewer->disableDragAndDrop(this);

  if(mViewer == subscription)
    delete this;
}

//==============================================================================
SimpleFrameDnD::SimpleFrameDnD(Viewer* viewer,
                               dart::dynamics::SimpleFrame* frame)
  : DragAndDrop(viewer, frame),
    mOption(RotationOption::HOLD_CTRL),
    mFrame(frame)
{

}

//==============================================================================
SimpleFrameDnD::~SimpleFrameDnD()
{
  // Do nothing
}

//==============================================================================
dart::dynamics::SimpleFrame* SimpleFrameDnD::getSimpleFrame() const
{
  return mFrame;
}

//==============================================================================
void SimpleFrameDnD::move()
{
  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());

  bool ctrl_down = (mViewer->getDefaultEventHandler()->getModKeyMask()
                    & osgGA::GUIEventAdapter::MODKEY_CTRL);

  if(  ((RotationOption::HOLD_CTRL==mOption) && ctrl_down)
      || RotationOption::ALWAYS_ON==mOption )
  {
    // Rotate

    Eigen::AngleAxisd R = getConstrainedRotation();

    tf.translation() = mPivot;
    tf.linear() = (R * mSavedRotation).matrix();
  }
  else
  {
    // Translate

    Eigen::Vector3d dx = getConstrainedDx();

    tf.translation() = mPivot + dx;
    tf.rotate(mSavedRotation);
  }

  dart::dynamics::Frame* parent = mFrame->getParentFrame();
  if(parent->isWorld())
    mFrame->setRelativeTransform(tf);
  else
    mFrame->setRelativeTransform( parent->getWorldTransform().inverse()*tf );
}

//==============================================================================
void SimpleFrameDnD::saveState()
{
  mPivot = mFrame->getWorldTransform().translation();
  mSavedRotation = mFrame->getWorldTransform().rotation();
}

//==============================================================================
void SimpleFrameDnD::setRotationOption(RotationOption option)
{
  mOption = option;
}

//==============================================================================
SimpleFrameShapeDnD::SimpleFrameShapeDnD(
    Viewer* viewer,
    dart::dynamics::SimpleFrame* frame,
    dart::dynamics::Shape* shape)
  : SimpleFrameDnD(viewer, frame),
    mShape(shape)
{
  // Do nothing
}

//==============================================================================
SimpleFrameShapeDnD::~SimpleFrameShapeDnD()
{
  // Do nothing
}

//==============================================================================
dart::dynamics::Shape* SimpleFrameShapeDnD::getShape() const
{
  return mShape;
}

//==============================================================================
void SimpleFrameShapeDnD::update()
{
  // This is almost identical to the original DragAndDrop::update() except that
  // it also checks that the picked shape matches

  if(nullptr == mEntity || nullptr == mShape)
    return;

  osgDart::MouseButtonEvent event =
      mViewer->getDefaultEventHandler()->getButtonEvent(LEFT_MOUSE);

  if(mAmMoving)
  {
    if(osgDart::BUTTON_RELEASE == event)
      mAmMoving = false;

    move();
  }
  else
  {
    if(osgDart::BUTTON_PUSH == event)
    {
      const std::vector<osgDart::PickInfo>& picks =
          mViewer->getDefaultEventHandler()->getButtonPicks(
            osgDart::LEFT_MOUSE, osgDart::BUTTON_PUSH);

      for(const osgDart::PickInfo& pick : picks)
      {
        if(pick.entity == mEntity && pick.shape == mShape)
        {
          mAmMoving = true;
          mPickedPosition = pick.position;
          saveState();
          return;
        }

        if(mAmObstructable)
          return;
      }
    }
  }
}

//==============================================================================
void SimpleFrameShapeDnD::handleDestructionNotification(
    const dart::common::Publisher* subscription)
{
  DragAndDrop::handleDestructionNotification(subscription);

  if(mShape == subscription)
    mViewer->disableDragAndDrop(this);
}

//==============================================================================
InteractiveFrameDnD::InteractiveFrameDnD(Viewer* viewer,
                                         InteractiveFrame* frame)
  : DragAndDrop(viewer, frame),
    mInteractiveFrame(frame)
{
  const std::vector<dart::dynamics::Shape*> shapes =
      frame->getVisualizationShapes();
  mDnDs.push_back(new SimpleFrameShapeDnD(viewer, frame, shapes[0]));
  mDnDs.push_back(new SimpleFrameShapeDnD(viewer, frame, shapes[1]));
  mDnDs.push_back(new SimpleFrameShapeDnD(viewer, frame, shapes[2]));

  for(size_t i=0; i<3; ++i)
  {
    SimpleFrameShapeDnD* dnd = mDnDs[i];
    dnd->setRotationOption(SimpleFrameDnD::RotationOption::ALWAYS_ON);
  }
}

//==============================================================================
InteractiveFrameDnD::~InteractiveFrameDnD()
{
  for(size_t i=0; i<mDnDs.size(); ++i)
    delete mDnDs[i];
  mDnDs.clear();
}

//==============================================================================
InteractiveFrame* InteractiveFrameDnD::getFrame() const
{
  return mInteractiveFrame;
}

//==============================================================================
void InteractiveFrameDnD::update()
{
  for(size_t i=0; i<3; ++i)
  {
    SimpleFrameShapeDnD* dnd = mDnDs[i];
    Eigen::Matrix3d R = mInteractiveFrame->getWorldTransform().linear();
    dnd->constrainToLine(R.col(i));
  }

  for(size_t i=0; i<3; ++i)
  {
    SimpleFrameShapeDnD* dnd = mDnDs[i];
    dnd->update();
  }
}

//==============================================================================
void InteractiveFrameDnD::move()
{
  // Do nothing
}

//==============================================================================
void InteractiveFrameDnD::saveState()
{
  // Do nothing
}

} // namespace osgDart
