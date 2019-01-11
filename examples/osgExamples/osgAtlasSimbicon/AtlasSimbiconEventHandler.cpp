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

#include "AtlasSimbiconEventHandler.hpp"

//==============================================================================
AtlasSimbiconEventHandler::AtlasSimbiconEventHandler(
    AtlasSimbiconWorldNode* node)
  : mNode(node)
{
  // Do nothing
}

//==============================================================================
bool AtlasSimbiconEventHandler::handle(
    const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter&)
{
  if(ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN)
  {
    if(ea.getKey() == 'r' || ea.getKey() == 'R')
    {
      mNode->reset();
      return true;
    }
    else if(ea.getKey() == 'a' || ea.getKey() == 'A')
    {
      mNode->pushForwardAtlas();
      return true;
    }
    else if(ea.getKey() == 's' || ea.getKey() == 'S')
    {
      mNode->pushBackwardAtlas();
      return true;
    }
    else if(ea.getKey() == 'd' || ea.getKey() == 'D')
    {
      mNode->pushLeftAtlas();
      return true;
    }
    else if(ea.getKey() == 'f' || ea.getKey() == 'F')
    {
      mNode->pushRightAtlas();
      return true;
    }
  }

  // The return value should be 'true' if the input has been fully handled
  // and should not be visible to any remaining event handlers. It should be
  // false if the input has not been fully handled and should be viewed by
  // any remaining event handlers.
  return false;
}
