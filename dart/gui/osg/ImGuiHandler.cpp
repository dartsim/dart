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

#include "dart/gui/osg/ImGuiHandler.hpp"

#include <algorithm>

#include <osg/Camera>
#include <osg/RenderInfo>

#include "dart/external/imgui/imgui.h"
#include "dart/external/imgui/imgui_impl_opengl2.h"

#include "dart/common/Console.hpp"
#include "dart/gui/osg/ImGuiWidget.hpp"

namespace dart {
namespace gui {
namespace osg {

//==============================================================================
struct ImGuiNewFrameCallback : public ::osg::Camera::DrawCallback
{
  ImGuiNewFrameCallback(ImGuiHandler* handler) : mHandler(handler)
  {
    // Do nothing
  }

  virtual void operator()(::osg::RenderInfo& renderInfo) const
  {
    mHandler->newFrame(renderInfo);
  }

private:
  ::osg::ref_ptr<ImGuiHandler> mHandler;
};

//==============================================================================
struct ImGuiDrawCallback : public ::osg::Camera::DrawCallback
{
  ImGuiDrawCallback(ImGuiHandler* handler) : mHandler(handler)
  {
    // Do nothing
  }

  virtual void operator()(::osg::RenderInfo& renderInfo) const
  {
    mHandler->render(renderInfo);
  }

private:
  ::osg::ref_ptr<ImGuiHandler> mHandler;
};

//==============================================================================
ImGuiHandler::ImGuiHandler()
  : mTime{0.0}, mMousePressed{false, false, false}, mMouseWheel{0.0f}
{
  ImGui::CreateContext();

  ImGui::StyleColorsDark();

  ImGui_ImplOpenGL2_Init();
}

//==============================================================================
ImGuiHandler::~ImGuiHandler()
{
  // Do nothing
}

//==============================================================================
void ImGuiHandler::setCameraCallbacks(::osg::Camera* camera)
{
  if (nullptr == camera)
    return;

  ImGuiDrawCallback* postDrawCallback = new ImGuiDrawCallback(this);
  camera->setPostDrawCallback(postDrawCallback);

  ImGuiNewFrameCallback* preDrawCallback = new ImGuiNewFrameCallback(this);
  camera->setPreDrawCallback(preDrawCallback);
}

//==============================================================================
bool ImGuiHandler::hasWidget(const std::shared_ptr<ImGuiWidget>& widget) const
{
  return std::find(mWidgets.begin(), mWidgets.end(), widget) != mWidgets.end();
}

//==============================================================================
void ImGuiHandler::addWidget(
    const std::shared_ptr<ImGuiWidget>& widget, bool visible)
{
  if (hasWidget(widget))
  {
    dtwarn
        << "[ImGuiHandler::addWidget] Attempting to add existing widget to the "
           "viewer. Ignoring this action.";
    return;
  }

  widget->setVisible(visible);
  mWidgets.push_back(widget);
}

//==============================================================================
void ImGuiHandler::removeWidget(const std::shared_ptr<ImGuiWidget>& widget)
{
  if (!hasWidget(widget))
  {
    dtwarn << "[ImGuiHandler::removeWidget] Attempting to remove not existing "
              "widget from the viewer. Ignoring this action.\n";
    return;
  }

  mWidgets.erase(
      std::remove(mWidgets.begin(), mWidgets.end(), widget), mWidgets.end());
}

//==============================================================================
void ImGuiHandler::removeAllWidget()
{
  mWidgets.clear();
}

//==============================================================================
bool ImGuiHandler::handle(
    const osgGA::GUIEventAdapter& eventAdapter,
    osgGA::GUIActionAdapter& /*actionAdapter*/,
    ::osg::Object* /*object*/,
    ::osg::NodeVisitor* /*nodeVisitor*/)
{
  const auto wantCapureMouse = ImGui::GetIO().WantCaptureMouse;
  const auto wantCapureKeyboard = ImGui::GetIO().WantCaptureKeyboard;

  switch (eventAdapter.getEventType())
  {
    case osgGA::GUIEventAdapter::KEYDOWN:
    {
      auto& io = ImGui::GetIO();
      const auto c = eventAdapter.getKey();

      if (c > 0 && c < 0x10000)
        io.AddInputCharacter(static_cast<ImWchar>(c));

      return wantCapureKeyboard;
    }
    case (osgGA::GUIEventAdapter::PUSH):
    {
      auto& io = ImGui::GetIO();

      io.MousePos
          = ImVec2(eventAdapter.getX(), io.DisplaySize.y - eventAdapter.getY());
      mMousePressed[0] = true;

      return wantCapureMouse;
    }
    case (osgGA::GUIEventAdapter::DRAG):
    case (osgGA::GUIEventAdapter::MOVE):
    {
      auto& io = ImGui::GetIO();

      io.MousePos
          = ImVec2(eventAdapter.getX(), io.DisplaySize.y - eventAdapter.getY());

      return wantCapureMouse;
    }
    case (osgGA::GUIEventAdapter::RELEASE):
    {
      mMousePressed[0] = false;

      return wantCapureMouse;
    }
    case (osgGA::GUIEventAdapter::SCROLL):
    {
      float increment = 0.1;

      switch (eventAdapter.getScrollingMotion())
      {
        case (osgGA::GUIEventAdapter::SCROLL_NONE):
          break;
        case (osgGA::GUIEventAdapter::SCROLL_LEFT):
          break;
        case (osgGA::GUIEventAdapter::SCROLL_RIGHT):
          break;
        case (osgGA::GUIEventAdapter::SCROLL_UP):
          mMouseWheel += increment;
          break;
        case (osgGA::GUIEventAdapter::SCROLL_DOWN):
          mMouseWheel -= increment;
          break;
        case (osgGA::GUIEventAdapter::SCROLL_2D):
          mMouseWheel = eventAdapter.getScrollingDeltaY();
          break;
      }

      return wantCapureMouse;
    }
    default:
    {
      return false;
    }
  }

  return false;
}

//==============================================================================
void ImGuiHandler::newFrame(::osg::RenderInfo& renderInfo)
{
  ImGui_ImplOpenGL2_NewFrame();

  auto& io = ImGui::GetIO();
  auto* viewport = renderInfo.getCurrentCamera()->getViewport();

  io.DisplaySize = ImVec2(viewport->width(), viewport->height());

  const auto currentTime
      = renderInfo.getView()->getFrameStamp()->getSimulationTime();

  io.DeltaTime
      = mTime > 0.0 ? static_cast<float>(currentTime - mTime) : 1.0f / 60.0f;
  io.DeltaTime = std::max(io.DeltaTime, std::numeric_limits<float>::min());
  mTime = currentTime;
  assert(mTime >= 0.0);

  for (auto i = 0u; i < mMousePressed.size(); ++i)
    io.MouseDown[i] = mMousePressed[i];

  io.MouseWheel = mMouseWheel;
  mMouseWheel = 0.0f;

  ImGui::NewFrame();
}

//==============================================================================
void ImGuiHandler::render(::osg::RenderInfo& /*renderInfo*/)
{
  for (const auto& widget : mWidgets)
  {
    if (widget->isVisible())
      widget->render();
  }

  ImGui::Render();

  auto* drawData = ImGui::GetDrawData();
  ImGui_ImplOpenGL2_RenderDrawData(drawData);
}

} // namespace osg
} // namespace gui
} // namespace dart
