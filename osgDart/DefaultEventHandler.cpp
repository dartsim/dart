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

#include <osgGA/GUIEventAdapter>

#include "osgDart/DefaultEventHandler.h"
#include "osgDart/Viewer.h"
#include "osgDart/render/ShapeNode.h"
#include "osgDart/EntityNode.h"
#include "osgDart/utils.h"

#include "dart/dynamics/Entity.h"

namespace osgDart
{

DefaultEventHandler::DefaultEventHandler(Viewer* _viewer)
  : mViewer(_viewer)
{
  mViewer->addInstructionText("Spacebar:     Turn simulation on/off for any active worlds\n");
  mViewer->addInstructionText("Ctrl+H:       Turn headlights on/off\n");

  for(size_t i=0; i<NUM_MOUSE_BUTTONS; ++i)
    for(size_t j=0; j<NUM_MOUSE_BUTTON_EVENTS; ++j)
      mSuppressButtonPicks[i][j] = false;
  mSuppressMovePicks = false;
}

//==============================================================================
DefaultEventHandler::~DefaultEventHandler()
{
  // Do nothing
}

//==============================================================================
bool DefaultEventHandler::handle(const osgGA::GUIEventAdapter& ea,
                                 osgGA::GUIActionAdapter&)
{
  switch(ea.getEventType())
  {
    case osgGA::GUIEventAdapter::KEYDOWN:
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
          mViewer->simulate(!mViewer->isSimulating());
          return true;
          break;
        }
      }
    }

    case osgGA::GUIEventAdapter::MOVE:
    {
      if(!mSuppressMovePicks)
        pick(mMovePicks, ea);
      break;
    }

    case osgGA::GUIEventAdapter::PUSH:
    case osgGA::GUIEventAdapter::DRAG:
    case osgGA::GUIEventAdapter::RELEASE:
      eventPick(ea);
      break;

    default:
      break;
  }

  return false;
}

//==============================================================================
void DefaultEventHandler::pick(std::vector<PickInfo>& infoVector,
                               const osgGA::GUIEventAdapter& ea)
{
  osgUtil::LineSegmentIntersector::Intersections hlist;

  infoVector.clear();
  if(mViewer->computeIntersections(ea.getX(), ea.getY(), hlist))
  {
    infoVector.reserve(hlist.size());
    for(const osgUtil::LineSegmentIntersector::Intersection& intersect : hlist)
    {
      osg::Drawable* drawable = intersect.drawable;
      render::ShapeNode* shape =
          dynamic_cast<render::ShapeNode*>(drawable->getParent(0));
      if(shape)
      {
        PickInfo info;
        info.shape = shape->getShape();
        info.entity = shape->getParentEntityNode()->getEntity();
        info.normal = osgToEigVec3(intersect.getWorldIntersectNormal());
        info.position = osgToEigVec3(intersect.getWorldIntersectPoint());

        infoVector.push_back(info);
      }
    }
  }
}

//==============================================================================
const std::vector<PickInfo>& DefaultEventHandler::getButtonPicks(
    MouseButton button, MouseButtonEvent event) const
{
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
  mSuppressButtonPicks[button][event] = false;
}

//==============================================================================
void DefaultEventHandler::activateMovePicks()
{
  mSuppressMovePicks = false;
}

//==============================================================================
void DefaultEventHandler::eventPick(const osgGA::GUIEventAdapter& ea)
{
  MouseButtonEvent mbe;
  switch(ea.getEventType())
  {
    case osgGA::GUIEventAdapter::PUSH:
      mbe = BUTTON_PUSH;
      break;
    case osgGA::GUIEventAdapter::DRAG:
      mbe = BUTTON_DRAG;
      break;
    case osgGA::GUIEventAdapter::RELEASE:
      mbe = BUTTON_RELEASE;
      break;
    default:
      return;
  }

  if(   ( (ea.getButtonMask() & osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON)
           && !mSuppressButtonPicks[LEFT_MOUSE][mbe])
     || ( (ea.getButtonMask() & osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON)
           && !mSuppressButtonPicks[RIGHT_MOUSE][mbe])
     || ( (ea.getButtonMask() & osgGA::GUIEventAdapter::MIDDLE_MOUSE_BUTTON)
           && !mSuppressButtonPicks[MIDDLE_MOUSE][mbe]))
  {
    pick(mTempPicks, ea);

    if(ea.getButtonMask() & osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON)
      mButtonPicks[LEFT_MOUSE][mbe] = mTempPicks;

    if(ea.getButtonMask() & osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON)
      mButtonPicks[RIGHT_MOUSE][mbe] = mTempPicks;

    if(ea.getButtonMask() & osgGA::GUIEventAdapter::MIDDLE_MOUSE_BUTTON)
      mButtonPicks[MIDDLE_MOUSE][mbe] = mTempPicks;
  }
}

} // namespace osgDart
