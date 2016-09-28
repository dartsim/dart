/*
 * Copyright (c) 2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Humanoid Lab, Georgia Tech Research Corporation
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

#include "AtlasSimbiconEventHandler.hpp"

//==============================================================================
AtlasSimbiconEventHandler::AtlasSimbiconEventHandler()
{
  // Set up the customized event handler
}

//==============================================================================
bool AtlasSimbiconEventHandler::handle(
    const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter&)
{
  if(ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN)
  {
    if(ea.getKey() == 'q')
    {
      std::cout << "Lowercase q pressed" << std::endl;
      return true;
    }
    else if(ea.getKey() == 'Q')
    {
      std::cout << "Capital Q pressed" << std::endl;
      return true;
    }
    else if(ea.getKey() == osgGA::GUIEventAdapter::KEY_Left)
    {
      std::cout << "Left arrow key pressed" << std::endl;
      return true;
    }
    else if(ea.getKey() == osgGA::GUIEventAdapter::KEY_Right)
    {
      std::cout << "Right arrow key pressed" << std::endl;
      return true;
    }
  }
  else if(ea.getEventType() == osgGA::GUIEventAdapter::KEYUP)
  {
    if(ea.getKey() == 'q')
    {
      std::cout << "Lowercase q released" << std::endl;
      return true;
    }
    else if(ea.getKey() == 'Q')
    {
      std::cout << "Capital Q released" << std::endl;
      return true;
    }
    else if(ea.getKey() == osgGA::GUIEventAdapter::KEY_Left)
    {
      std::cout << "Left arrow key released" << std::endl;
      return true;
    }
    else if(ea.getKey() == osgGA::GUIEventAdapter::KEY_Right)
    {
      std::cout << "Right arrow key released" << std::endl;
      return true;
    }
  }

  // The return value should be 'true' if the input has been fully handled
  // and should not be visible to any remaining event handlers. It should be
  // false if the input has not been fully handled and should be viewed by
  // any remaining event handlers.
  return false;
}
