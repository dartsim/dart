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

namespace osgDart
{

DefaultEventHandler::DefaultEventHandler(Viewer* _viewer)
  : mViewer(_viewer)
{
  mViewer->addInstructionText("Spacebar: Turn simulation on/off for any active worlds\n");
  mViewer->addInstructionText("Ctrl+H: Turn headlights on/off\n");
}

//==============================================================================
DefaultEventHandler::~DefaultEventHandler()
{
  // Do nothing
}

//==============================================================================
bool DefaultEventHandler::handle(const osgGA::GUIEventAdapter& ea,
                                 osgGA::GUIActionAdapter &)
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

    default:
      break;
  }

  return false;
}

} // namespace osgDart
