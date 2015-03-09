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

#include "dart/dynamics/SimpleFrame.h"

namespace osgDart {

DragAndDrop::DragAndDrop(Viewer* viewer, dart::dynamics::Entity* entity)
  : mViewer(viewer),
    mEntity(entity),
    mPickedPosition(Eigen::Vector3d::Zero()),
    mConstraintType(UNCONSTRAINED),
    mAmMoving(false)
{
  addSubscription(mEntity);
}

//==============================================================================
DragAndDrop::~DragAndDrop()
{
  // Do nothing
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
      }
    }
  }
}

//==============================================================================
Eigen::Vector3d DragAndDrop::getConstrainedDx() const
{
  return mViewer->getDefaultEventHandler()->getDeltaCursor(
        mPickedPosition, mConstraintType, mVector);
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
    const dart::common::Subscription* subscription)
{
  if(mEntity == subscription)
    mViewer->disableDragAndDrop(mEntity);
}

//==============================================================================
SimpleFrameDnD::SimpleFrameDnD(Viewer* viewer,
                               dart::dynamics::SimpleFrame* frame)
  : DragAndDrop(viewer, frame),
    mFrame(frame),
    mSavedPosition(Eigen::Vector3d::Zero())
{

}

//==============================================================================
SimpleFrameDnD::~SimpleFrameDnD()
{
  // Do nothing
}

//==============================================================================
void SimpleFrameDnD::move()
{
  Eigen::Vector3d dx = getConstrainedDx();

  Eigen::Isometry3d tf = mFrame->getWorldTransform();
  tf.translation() = mSavedPosition + dx;
  dart::dynamics::Frame* parent = mFrame->getParentFrame();
  if(parent->isWorld())
    mFrame->setRelativeTransform(tf);
  else
    mFrame->setRelativeTransform( parent->getWorldTransform().inverse()*tf );
}

//==============================================================================
void SimpleFrameDnD::saveState()
{
  mSavedPosition = mFrame->getWorldTransform().translation();
}

} // namespace osgDart
