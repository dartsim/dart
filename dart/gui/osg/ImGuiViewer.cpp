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

#include "dart/gui/osg/ImGuiViewer.hpp"

#include "dart/gui/osg/ImGuiWidget.hpp"
#include "dart/gui/osg/ImGuiHandler.hpp"

namespace dart {
namespace gui {
namespace osg {

//==============================================================================
ImGuiViewer::ImGuiViewer(const ::osg::Vec4& clearColor)
  : Viewer(clearColor),
    mImGuiHandler(new ImGuiHandler()),
    mAboutWidget(new AboutWidget())
{
  mImGuiHandler->setCameraCallbacks(getCamera());
  mImGuiHandler->addWidget(mAboutWidget, false);

  addEventHandler(mImGuiHandler);
}

//==============================================================================
ImGuiViewer::~ImGuiViewer()
{
  // Do nothing
}

//==============================================================================
ImGuiHandler* ImGuiViewer::getImGuiHandler()
{
  return mImGuiHandler;
}

//==============================================================================
const ImGuiHandler* ImGuiViewer::getImGuiHandler() const
{
  return mImGuiHandler;
}

//==============================================================================
void ImGuiViewer::showAbout()
{
  mAboutWidget->show();
}

//==============================================================================
void ImGuiViewer::hideAbout()
{
  mAboutWidget->hide();
}

} // namespace osg
} // namespace gui
} // namespace dart
