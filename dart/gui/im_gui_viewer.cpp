/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
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

#include "dart/gui/im_gui_viewer.hpp"

#include "dart/gui/im_gui_handler.hpp"
#include "dart/gui/im_gui_widget.hpp"

#include <algorithm>

#include <cmath>

namespace dart {
namespace gui {

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
  if (mImGuiHandler)
    mImGuiHandler->removeAllWidget();
}

//==============================================================================
ImGuiHandler* ImGuiViewer::getImGuiHandler()
{
  return mImGuiHandler.get();
}

//==============================================================================
const ImGuiHandler* ImGuiViewer::getImGuiHandler() const
{
  return mImGuiHandler.get();
}

//==============================================================================
void ImGuiViewer::setImGuiScale(float scale)
{
  applyImGuiScale(scale);
  if (std::isfinite(scale) && scale > 0.f)
    mImGuiScale = scale;
}

//==============================================================================
void ImGuiViewer::setUpViewInWindow(int x, int y, int width, int height)
{
  setUpViewInWindow(x, y, width, height, 0);
}

//==============================================================================
void ImGuiViewer::setUpViewInWindow(
    int x, int y, int width, int height, int screenNum)
{
  const float scale = mImGuiScale;
  int scaledWidth = width;
  int scaledHeight = height;

  if (std::isfinite(scale) && scale > 0.f && std::abs(scale - 1.f) > 1e-6f) {
    scaledWidth = std::max(1, static_cast<int>(std::lround(width * scale)));
    scaledHeight = std::max(1, static_cast<int>(std::lround(height * scale)));
  }

  Viewer::setUpViewInWindow(x, y, scaledWidth, scaledHeight, screenNum);
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

} // namespace gui
} // namespace dart
