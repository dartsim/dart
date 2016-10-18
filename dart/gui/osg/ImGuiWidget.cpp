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
 *   * This code incorporates portions of Open Dynamics Engine
 *     (Copyright (c) 2001-2004, Russell L. Smith. All rights
 *     reserved.) and portions of FCL (Copyright (c) 2011, Willow
 *     Garage, Inc. All rights reserved.), which were released under
 *     the same BSD license as below
 *
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

#include "dart/gui/osg/ImGuiWidget.hpp"

#include "dart/external/imgui/imgui.h"

namespace dart {
namespace gui {
namespace osg {

//==============================================================================
ImGuiWidget::ImGuiWidget() : mIsVisible(true)
{
  // Do nothing
}

//==============================================================================
ImGuiWidget::~ImGuiWidget()
{
  // Do nothing
}

//==============================================================================
void ImGuiWidget::setVisible(bool visible)
{
  mIsVisible = visible;
}

//==============================================================================
void ImGuiWidget::toggleVisible()
{
  mIsVisible ^= 1;
}

//==============================================================================
void ImGuiWidget::show()
{
  mIsVisible = true;
}

//==============================================================================
void ImGuiWidget::hide()
{
  mIsVisible = false;
}

//==============================================================================
bool ImGuiWidget::isVisible() const
{
  return mIsVisible;
}

//==============================================================================
void AboutWidget::render()
{
  ImGui::Begin("About DART", &mIsVisible, ImGuiWindowFlags_AlwaysAutoResize);
  //ImGui::Text("HIT %s", Version::asString().c_str());
  ImGui::Separator();
  ImGui::Text("Copyright (c) 2011-2016, Graphics Lab, Georgia Tech Research Corporation");
  ImGui::Text("Copyright (c) 2011-2016, Humanoid Lab, Georgia Tech Research Corporation");
  ImGui::Text("Copyright (c) 2016, Personal Robotics Lab, Carnegie Mellon University");
  ImGui::Text("DART is licensed under the BSD 2 Clause License.");
  ImGui::Separator();
  ImGui::Text("Application average %.3f ms/frame (%.1f FPS)",
              1000.0f / ImGui::GetIO().Framerate,
              ImGui::GetIO().Framerate);
  ImGui::End();
}

} // namespace gui
} // namespace gui
} // namespace dart
