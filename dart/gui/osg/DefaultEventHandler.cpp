/*
 * Copyright (c) 2015-2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2015-2016, Humanoid Lab, Georgia Tech Research Corporation
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

#include <osgGA/GUIEventAdapter>

#include "dart/gui/osg/DefaultEventHandler.hpp"
#include "dart/gui/osg/MouseEventHandler.hpp"
#include "dart/gui/osg/Viewer.hpp"
#include "dart/gui/osg/render/ShapeNode.hpp"
#include "dart/gui/osg/ShapeFrameNode.hpp"
#include "dart/gui/osg/Utils.hpp"

#include "dart/dynamics/Entity.hpp"
#include "dart/dynamics/ShapeFrame.hpp"


#include <iostream>

namespace dart {
namespace gui {
namespace osg {

DefaultEventHandler::DefaultEventHandler(Viewer* _viewer)
  : mViewer(_viewer),
    mLastCursorPosition(Eigen::Vector2d::Zero()),
    mLastModKeyMask(0)
{
  mViewer->addInstructionText("Spacebar:     Turn simulation on/off\n");
  mViewer->addInstructionText("Ctrl+H:       Turn headlights on/off\n");

  for(std::size_t i=0; i<NUM_MOUSE_BUTTONS; ++i)
    for(std::size_t j=0; j<BUTTON_NOTHING; ++j)
      mSuppressButtonPicks[i][j] = false;
  mSuppressMovePicks = false;

  clearButtonEvents();
}

//==============================================================================
DefaultEventHandler::~DefaultEventHandler()
{
  // Do nothing
}

//==============================================================================
MouseButtonEvent DefaultEventHandler::getButtonEvent(MouseButton button) const
{
  return mLastButtonEvent[button];
}

//==============================================================================
int DefaultEventHandler::getModKeyMask() const
{
  return mLastModKeyMask;
}

//==============================================================================
double DefaultEventHandler::getWindowCursorX() const
{
  return mLastCursorPosition[0];
}

//==============================================================================
double DefaultEventHandler::getWindowCursorY() const
{
  return mLastCursorPosition[1];
}

//==============================================================================
Eigen::Vector3d DefaultEventHandler::getDeltaCursor(
    const Eigen::Vector3d& _fromPosition,
    ConstraintType _constraint,
    const Eigen::Vector3d& _constraintVector) const
{
  ::osg::Vec3d eye, center, up;
  mViewer->getCamera()->getViewMatrixAsLookAt(eye, center, up);

  Eigen::Vector3d near, far;
  getNearAndFarPointUnderCursor(near, far);
  Eigen::Vector3d v1 = far-near;

  if(LINE_CONSTRAINT == _constraint)
  {
    const Eigen::Vector3d& b1 = near;
    const Eigen::Vector3d& v2 = _constraintVector;
    const Eigen::Vector3d& b2 = _fromPosition;

    double v1_v1 = v1.dot(v1);
    double v2_v2 = v2.dot(v2);
    double v2_v1 = v2.dot(v1);

    double denominator = v1_v1*v2_v2 - v2_v1*v2_v1;
    double s;
    if(fabs(denominator) < 1e-10)
      s = 0;
    else
      s = (v1_v1*(v2.dot(b1)-v2.dot(b2)) + v2_v1*(v1.dot(b2)-v1.dot(b1)))/denominator;

    return v2*s;
  }
  else if(PLANE_CONSTRAINT == _constraint)
  {
    const Eigen::Vector3d& n = _constraintVector;
    double s = n.dot(_fromPosition - near) / n.dot(v1);
    return near - _fromPosition + s*v1;
  }
  else
  {
    Eigen::Vector3d n = osgToEigVec3(center - eye);
    double s = n.dot(_fromPosition - near) / n.dot(v1);
    return near - _fromPosition + s*v1;
  }

  return Eigen::Vector3d::Zero();
}

//==============================================================================
void DefaultEventHandler::getNearAndFarPointUnderCursor(Eigen::Vector3d& near,
                                                        Eigen::Vector3d& far,
                                                        double distance) const
{
  ::osg::Camera* C = mViewer->getCamera();
  ::osg::Matrix VPW = C->getViewMatrix() * C->getProjectionMatrix()
      * C->getViewport()->computeWindowMatrix();
  ::osg::Matrix invVPW;
  invVPW.invert(VPW);

  double x = getWindowCursorX(), y = getWindowCursorY();
  ::osg::Vec3 osgNear = ::osg::Vec3(x,y,0.0) * invVPW;
  ::osg::Vec3 osgFar = ::osg::Vec3(x,y,distance) * invVPW;

  near = osgToEigVec3(osgNear);
  far = osgToEigVec3(osgFar);
}

//==============================================================================
const std::vector<PickInfo>& DefaultEventHandler::getButtonPicks(
    MouseButton button, MouseButtonEvent event) const
{
  if(BUTTON_NOTHING == event)
    return mMovePicks;

  return mButtonPicks[button][event];
}

//==============================================================================
const std::vector<PickInfo>& DefaultEventHandler::getMovePicks() const
{
  return mMovePicks;
}

//==============================================================================
void DefaultEventHandler::suppressButtonPicks(MouseButton button,
                                              MouseButtonEvent event)
{
  if(BUTTON_NOTHING == event)
    mSuppressMovePicks = true;
  else
    mSuppressButtonPicks[button][event] = true;
}

//==============================================================================
void DefaultEventHandler::suppressMovePicks()
{
  mSuppressMovePicks = true;
}

//==============================================================================
void DefaultEventHandler::activateButtonPicks(MouseButton button,
                                              MouseButtonEvent event)
{
  if(BUTTON_NOTHING == event)
    mSuppressMovePicks = false;
  else
    mSuppressButtonPicks[button][event] = false;
}

//==============================================================================
void DefaultEventHandler::activateMovePicks()
{
  mSuppressMovePicks = false;
}

//==============================================================================
void DefaultEventHandler::pick(std::vector<PickInfo>& infoVector,
                               const ::osgGA::GUIEventAdapter& ea)
{
  ::osgUtil::LineSegmentIntersector::Intersections hlist;

  infoVector.clear();
  if(mViewer->computeIntersections(ea, hlist))
  {
    infoVector.reserve(hlist.size());
    for(const ::osgUtil::LineSegmentIntersector::Intersection& intersect : hlist)
    {
      ::osg::Drawable* drawable = intersect.drawable;
      render::ShapeNode* shape =
          dynamic_cast<render::ShapeNode*>(drawable->getParent(0));
      if(shape)
      {
        PickInfo info;
        info.shape = shape->getShape();
        info.frame = shape->getParentShapeFrameNode()->getShapeFrame();
        info.normal = osgToEigVec3(intersect.getWorldIntersectNormal());
        info.position = osgToEigVec3(intersect.getWorldIntersectPoint());

        infoVector.push_back(info);
      }
    }
  }
}

//==============================================================================
void DefaultEventHandler::addMouseEventHandler(MouseEventHandler* handler)
{
  mMouseEventHandlers.insert(handler);
  handler->mEventHandler = this;
  handler->addSubject(this);
}

//==============================================================================
const std::set<MouseEventHandler*>&
DefaultEventHandler::getMouseEventHandlers() const
{
  return mMouseEventHandlers;
}

//==============================================================================
static bool wasActive(MouseButtonEvent event)
{
  return ( (event == BUTTON_PUSH) || (event == BUTTON_DRAG) );
}

//==============================================================================
static void assignEventToButtons(
    MouseButtonEvent (&mLastButtonEvent)[NUM_MOUSE_BUTTONS],
    const ::osgGA::GUIEventAdapter& ea)
{
  MouseButtonEvent event = BUTTON_NOTHING;
  if(ea.getEventType() == ::osgGA::GUIEventAdapter::PUSH)
    event = BUTTON_PUSH;
  else if(ea.getEventType() == ::osgGA::GUIEventAdapter::DRAG)
    event = BUTTON_DRAG;
  else if(ea.getEventType() == ::osgGA::GUIEventAdapter::RELEASE)
    event = BUTTON_RELEASE;

  if(BUTTON_RELEASE == event)
  {
    if( (ea.getButtonMask() & ::osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON) == 0
        && wasActive(mLastButtonEvent[LEFT_MOUSE]) )
      mLastButtonEvent[LEFT_MOUSE] = event;

    if( (ea.getButtonMask() & ::osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON) == 0
        && wasActive(mLastButtonEvent[RIGHT_MOUSE]) )
      mLastButtonEvent[RIGHT_MOUSE] = event;

    if( (ea.getButtonMask() & ::osgGA::GUIEventAdapter::MIDDLE_MOUSE_BUTTON) == 0
        && wasActive(mLastButtonEvent[MIDDLE_MOUSE]) )
      mLastButtonEvent[MIDDLE_MOUSE] = event;
  }
  else
  {
    if(ea.getButtonMask() & ::osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON)
      mLastButtonEvent[LEFT_MOUSE] = event;

    if(ea.getButtonMask() & ::osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON)
      mLastButtonEvent[RIGHT_MOUSE] = event;

    if(ea.getButtonMask() & ::osgGA::GUIEventAdapter::MIDDLE_MOUSE_BUTTON)
      mLastButtonEvent[MIDDLE_MOUSE] = event;
  }
}

//==============================================================================
bool DefaultEventHandler::handle(const ::osgGA::GUIEventAdapter& ea,
                                 ::osgGA::GUIActionAdapter&)
{
  mLastModKeyMask = ea.getModKeyMask();

  switch(ea.getEventType())
  {
    case ::osgGA::GUIEventAdapter::PUSH:
    case ::osgGA::GUIEventAdapter::DRAG:
    case ::osgGA::GUIEventAdapter::RELEASE:
    case ::osgGA::GUIEventAdapter::MOVE:
      mLastCursorPosition[0] = ea.getX();
      mLastCursorPosition[1] = ea.getY();

      break;

    default:
      break;
  }

  switch(ea.getEventType())
  {
    case ::osgGA::GUIEventAdapter::KEYDOWN:
    {
      switch(ea.getKey())
      {
        case 8: // ctrl+h
        {
          mViewer->switchHeadlights(!mViewer->checkHeadlights());
          return true;
          break;
        }

        case ' ':
        {
          if(mViewer->isAllowingSimulation())
          {
            mViewer->simulate(!mViewer->isSimulating());
            return true;
          }
          break;
        }
      }
    }

    case ::osgGA::GUIEventAdapter::MOVE:
    {
      if(!mSuppressMovePicks)
        pick(mMovePicks, ea);

      triggerMouseEventHandlers();
      break;
    }

    case ::osgGA::GUIEventAdapter::PUSH:
    case ::osgGA::GUIEventAdapter::DRAG:
    case ::osgGA::GUIEventAdapter::RELEASE:

      assignEventToButtons(mLastButtonEvent, ea);
      eventPick(ea);

      mViewer->updateDragAndDrops();

      triggerMouseEventHandlers();
      break;

    default:
      break;
  }

  return false;
}

//==============================================================================
void DefaultEventHandler::triggerMouseEventHandlers()
{
  for(MouseEventHandler* h : mMouseEventHandlers)
  {
    h->update();
  }
}

//==============================================================================
void DefaultEventHandler::eventPick(const ::osgGA::GUIEventAdapter& ea)
{
  MouseButtonEvent mbe;
  switch(ea.getEventType())
  {
    case ::osgGA::GUIEventAdapter::PUSH:
      mbe = BUTTON_PUSH;
      break;
    case ::osgGA::GUIEventAdapter::DRAG:
      mbe = BUTTON_DRAG;
      break;
    case ::osgGA::GUIEventAdapter::RELEASE:
      mbe = BUTTON_RELEASE;
      break;
    default:
      return;
  }

  if(   ( (ea.getButtonMask() & ::osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON)
           && !mSuppressButtonPicks[LEFT_MOUSE][mbe])
     || ( (ea.getButtonMask() & ::osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON)
           && !mSuppressButtonPicks[RIGHT_MOUSE][mbe])
     || ( (ea.getButtonMask() & ::osgGA::GUIEventAdapter::MIDDLE_MOUSE_BUTTON)
           && !mSuppressButtonPicks[MIDDLE_MOUSE][mbe]))
  {
    pick(mTempPicks, ea);

    if(ea.getButtonMask() & ::osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON)
      mButtonPicks[LEFT_MOUSE][mbe] = mTempPicks;

    if(ea.getButtonMask() & ::osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON)
      mButtonPicks[RIGHT_MOUSE][mbe] = mTempPicks;

    if(ea.getButtonMask() & ::osgGA::GUIEventAdapter::MIDDLE_MOUSE_BUTTON)
      mButtonPicks[MIDDLE_MOUSE][mbe] = mTempPicks;
  }
}

//==============================================================================
void DefaultEventHandler::clearButtonEvents()
{
  for(std::size_t i=0; i<NUM_MOUSE_BUTTONS; ++i)
    mLastButtonEvent[i] = BUTTON_NOTHING;
}

//==============================================================================
void DefaultEventHandler::handleDestructionNotification(
    const dart::common::Subject* _subject)
{
  MouseEventHandler* meh = const_cast<MouseEventHandler*>(
        dynamic_cast<const MouseEventHandler*>(_subject));
  std::set<MouseEventHandler*>::iterator it = mMouseEventHandlers.find(meh);
  if(it != mMouseEventHandlers.end())
    mMouseEventHandlers.erase(it);
}

} // namespace osg
} // namespace gui
} // namespace dart
