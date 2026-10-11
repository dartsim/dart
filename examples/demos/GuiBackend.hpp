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

#ifndef DART_EXAMPLES_DEMOS_GUIBACKEND_HPP_
#define DART_EXAMPLES_DEMOS_GUIBACKEND_HPP_

#include "../GuiOptions.hpp"

#include <dart/gui/DebugOverlay.hpp>
#include <dart/gui/DefaultEventHandler.hpp>
#include <dart/gui/DragAndDrop.hpp>
#include <dart/gui/GUIEventHandler.hpp>
#include <dart/gui/GridVisual.hpp>
#include <dart/gui/ImGuiWidget.hpp>
#include <dart/gui/InteractiveFrame.hpp>
#include <dart/gui/MouseEventHandler.hpp>
#include <dart/gui/PerformanceStatsPanel.hpp>
#include <dart/gui/RealTimeWorldNode.hpp>
#include <dart/gui/SupportPolygonVisual.hpp>
#include <dart/gui/Viewer.hpp>
#include <dart/gui/WorldNode.hpp>

#include <dart/dart.hpp>
#include <dart/external/imgui/imgui.h>

#include <algorithm>
#include <memory>
#include <ostream>
#include <string>
#include <utility>

#include <cmath>
#include <cstdlib>
namespace demo_gui = dart::gui;
namespace demo_events = dart::gui;
using GuiViewer = dart::gui::Viewer;
using GuiScaleSource = GuiViewer;
using GuiVector3 = Eigen::Vector3d;
using GuiGroup = dart::gui::WorldNode;
template <class T>
using GuiPtr = std::shared_ptr<T>;

namespace dart_demos {

template <class T, class... Args>
GuiPtr<T> makeGui(Args&&... args)
{
  return std::make_shared<T>(std::forward<Args>(args)...);
}

template <class T>
void addGuiWorldNode(GuiViewer& viewer, const GuiPtr<T>& node)
{
  viewer.addWorldNode(node);
}

template <class T>
void removeGuiWorldNode(GuiViewer& viewer, const GuiPtr<T>& node)
{
  viewer.removeWorldNode(node);
}

template <class T>
void addGuiAttachment(GuiViewer& viewer, const GuiPtr<T>& attachment)
{
  viewer.addAttachment(attachment);
}

template <class T>
void removeGuiAttachment(GuiViewer& viewer, const GuiPtr<T>& attachment)
{
  viewer.removeAttachment(attachment);
}

template <class T>
void addGuiEventHandler(GuiViewer& viewer, const GuiPtr<T>& handler)
{
  viewer.addEventHandler(handler);
}

template <class T>
void removeGuiEventHandler(GuiViewer& viewer, const GuiPtr<T>& handler)
{
  viewer.removeEventHandler(handler);
}

inline double guiScale(const GuiViewer& viewer)
{
  return viewer.getGuiScale();
}

// ImGui layout uses logical window coordinates.
inline double guiLayoutScale(double scale)
{
  if (ImGui::GetCurrentContext()) {
    const float framebufferScale = ImGui::GetIO().DisplayFramebufferScale.y;
    if (framebufferScale > 0)
      return scale / framebufferScale;
  }
  return scale;
}

inline bool guiEventHandled(const demo_events::GUIEventAdapter& event)
{
  // Viewer stops dispatching as soon as a handler consumes an event.
  (void)event;
  return false;
}

inline void setGuiScale(GuiViewer& viewer, double scale)
{
  viewer.setGuiScale(scale);
}

inline void setGuiShadows(
    demo_gui::WorldNode& node, demo_gui::Viewer& viewer, bool enabled)
{
  (void)viewer;
  node.setShadowsEnabled(enabled);
}

using dart_examples::parseGuiScale;
using dart_examples::sanitizeGuiScale;

inline void applyGuiScale(double scale)
{
  const float sanitized
      = static_cast<float>(std::isfinite(scale) && scale > 0 ? scale : 1.0);
  ImGui::GetStyle().ScaleAllSizes(sanitized);
#if IMGUI_VERSION_NUM >= 19200
  ImGui::GetStyle().FontScaleMain = sanitized;
#else
  ImGui::GetIO().FontGlobalScale = sanitized;
#endif
}

} // namespace dart_demos
#endif
