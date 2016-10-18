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

#include "dart/gui/osg/ImGuiHandler.hpp"

#include <algorithm>

#include <osg/Camera>
#include <osg/RenderInfo>

#include "dart/external/imgui/imgui.h"

#include "dart/common/Console.hpp"
#include "dart/gui/osg/ImGuiWidget.hpp"

namespace dart {
namespace gui {
namespace osg {

//==============================================================================
void ImGui_RenderDrawLists(ImDrawData* draw_data)
{
  // Avoid rendering when minimized, scale coordinates for retina displays (screen coordinates != framebuffer coordinates)
  ImGuiIO& io = ImGui::GetIO();
  int fb_width = (int)(io.DisplaySize.x * io.DisplayFramebufferScale.x);
  int fb_height = (int)(io.DisplaySize.y * io.DisplayFramebufferScale.y);
  if (fb_width == 0 || fb_height == 0)
    return;
  draw_data->ScaleClipRects(io.DisplayFramebufferScale);

  // We are using the OpenGL fixed pipeline to make the example code simpler to read!
  // Setup render state: alpha-blending enabled, no face culling, no depth testing, scissor enabled, vertex/texcoord/color pointers.
  GLint last_texture; glGetIntegerv(GL_TEXTURE_BINDING_2D, &last_texture);
  GLint last_viewport[4]; glGetIntegerv(GL_VIEWPORT, last_viewport);
  glPushAttrib(GL_ENABLE_BIT | GL_COLOR_BUFFER_BIT | GL_TRANSFORM_BIT);
  glPushClientAttrib(GL_CLIENT_ALL_ATTRIB_BITS);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glDisable(GL_CULL_FACE);
  glDisable(GL_DEPTH_TEST);
  glEnable(GL_SCISSOR_TEST);
  glDisable(GL_LIGHTING);
  glEnableClientState(GL_VERTEX_ARRAY);
  glEnableClientState(GL_TEXTURE_COORD_ARRAY);
  glEnableClientState(GL_COLOR_ARRAY);
  glEnable(GL_TEXTURE_2D);
  //glUseProgram(0); // You may want this if using this code in an OpenGL 3+ context

  // Setup viewport, orthographic projection matrix
  glViewport(0, 0, (GLsizei)fb_width, (GLsizei)fb_height);
  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity();
  glOrtho(0.0f, io.DisplaySize.x, io.DisplaySize.y, 0.0f, -1.0f, +1.0f);
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();

  // Render command lists
#define OFFSETOF(TYPE, ELEMENT) ((size_t)&(((TYPE *)0)->ELEMENT))
  for (int n = 0; n < draw_data->CmdListsCount; n++)
  {
    const ImDrawList* cmd_list = draw_data->CmdLists[n];
    const unsigned char* vtx_buffer = (const unsigned char*)&cmd_list->VtxBuffer.front();
    const ImDrawIdx* idx_buffer = &cmd_list->IdxBuffer.front();
    glVertexPointer(2, GL_FLOAT, sizeof(ImDrawVert), (void*)(vtx_buffer + OFFSETOF(ImDrawVert, pos)));
    glTexCoordPointer(2, GL_FLOAT, sizeof(ImDrawVert), (void*)(vtx_buffer + OFFSETOF(ImDrawVert, uv)));
    glColorPointer(4, GL_UNSIGNED_BYTE, sizeof(ImDrawVert), (void*)(vtx_buffer + OFFSETOF(ImDrawVert, col)));

    for (int cmd_i = 0; cmd_i < cmd_list->CmdBuffer.size(); cmd_i++)
    {
      const ImDrawCmd* pcmd = &cmd_list->CmdBuffer[cmd_i];
      if (pcmd->UserCallback)
      {
        pcmd->UserCallback(cmd_list, pcmd);
      }
      else
      {
        glBindTexture(GL_TEXTURE_2D, (GLuint)(intptr_t)pcmd->TextureId);
        glScissor((int)pcmd->ClipRect.x, (int)(fb_height - pcmd->ClipRect.w), (int)(pcmd->ClipRect.z - pcmd->ClipRect.x), (int)(pcmd->ClipRect.w - pcmd->ClipRect.y));
        glDrawElements(GL_TRIANGLES, (GLsizei)pcmd->ElemCount, sizeof(ImDrawIdx) == 2 ? GL_UNSIGNED_SHORT : GL_UNSIGNED_INT, idx_buffer);
      }
      idx_buffer += pcmd->ElemCount;
    }
  }
#undef OFFSETOF

  // Restore modified state
  //glDisableClientState(GL_COLOR_ARRAY);
  //glDisableClientState(GL_TEXTURE_COORD_ARRAY);
  //glDisableClientState(GL_VERTEX_ARRAY);
  glBindTexture(GL_TEXTURE_2D, (GLuint)last_texture);
  glMatrixMode(GL_MODELVIEW);
  glPopMatrix();
  glMatrixMode(GL_PROJECTION);
  glPopMatrix();
  glPopAttrib();
  glPopClientAttrib();
  glViewport(last_viewport[0], last_viewport[1], (GLsizei)last_viewport[2], (GLsizei)last_viewport[3]);
}

//==============================================================================
struct ImGuiNewFrameCallback : public ::osg::Camera::DrawCallback
{
  ImGuiNewFrameCallback(ImGuiHandler& handler) : mHandler(handler)
  {
    // Do nothing
  }

  virtual void operator()(::osg::RenderInfo& renderInfo) const
  {
    mHandler.newFrame(renderInfo);
  }

private:
  ImGuiHandler& mHandler;
};

//==============================================================================
struct ImGuiDrawCallback : public ::osg::Camera::DrawCallback
{
  ImGuiDrawCallback(ImGuiHandler& handler) : mHandler(handler)
  {
    // Do nothing
  }

  virtual void operator()(::osg::RenderInfo& renderInfo) const
  {
    mHandler.render(renderInfo);
  }

private:
  ImGuiHandler& mHandler;
};

//==============================================================================
ImGuiHandler::ImGuiHandler()
  : mTime{0.0f},
    mMousePressed{false, false, false},
    mMouseWheel{0.0f},
    mFontTexture{0u}
{
  // Do nothing
}

//==============================================================================
void ImGuiHandler::init()
{
  ImGuiIO& io = ImGui::GetIO();

  // Build texture atlas
  unsigned char* pixels;
  int width, height;
  io.Fonts->GetTexDataAsAlpha8(&pixels, &width, &height);

  // Create OpenGL texture
  GLint last_texture;
  glGetIntegerv(GL_TEXTURE_BINDING_2D, &last_texture);
  glGenTextures(1, &mFontTexture);
  glBindTexture(GL_TEXTURE_2D, mFontTexture);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexImage2D(GL_TEXTURE_2D,
               0,
               GL_ALPHA,
               width,
               height,
               0,
               GL_ALPHA,
               GL_UNSIGNED_BYTE,
               pixels);

  // Store our identifier
  io.Fonts->TexID = (void*)(intptr_t)mFontTexture;

  // Cleanup (don't clear the input data if you want to append new fonts later)
  io.Fonts->ClearInputData();
  io.Fonts->ClearTexData();
  glBindTexture(GL_TEXTURE_2D, last_texture);

  io.RenderDrawListsFn = ImGui_RenderDrawLists;
}

//==============================================================================
void ImGuiHandler::setCameraCallbacks(::osg::Camera* camera)
{
  if (nullptr == camera)
    return;

  ImGuiDrawCallback* postDrawCallback = new ImGuiDrawCallback(*this);
  camera->setPostDrawCallback(postDrawCallback);

  ImGuiNewFrameCallback* preDrawCallback = new ImGuiNewFrameCallback(*this);
  camera->setPreDrawCallback(preDrawCallback);
}

//==============================================================================
bool ImGuiHandler::hasWidget(const std::shared_ptr<ImGuiWidget>& widget) const
{
  return std::find(mWidgets.begin(), mWidgets.end(), widget) != mWidgets.end();
}

//==============================================================================
void ImGuiHandler::addWidget(const std::shared_ptr<ImGuiWidget>& widget,
                             bool visible)
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
              "widget "
              "from the viewer. Ignoring this action.";
    return;
  }

  mWidgets.erase(std::remove(mWidgets.begin(), mWidgets.end(), widget),
                 mWidgets.end());
}

//==============================================================================
void ImGuiHandler::removeAllWidget()
{
  mWidgets.clear();
}

//==============================================================================
bool ImGuiHandler::handle(const osgGA::GUIEventAdapter& eventAdapter,
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

      io.MousePos =
          ImVec2(eventAdapter.getX(), io.DisplaySize.y - eventAdapter.getY());
      mMousePressed[0] = true;

      return wantCapureMouse;
    }
    case (osgGA::GUIEventAdapter::DRAG):
    case (osgGA::GUIEventAdapter::MOVE):
    {
      auto& io = ImGui::GetIO();

      io.MousePos =
          ImVec2(eventAdapter.getX(), io.DisplaySize.y - eventAdapter.getY());

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
  if (!mFontTexture)
    init();

  auto& io = ImGui::GetIO();
  auto* viewport = renderInfo.getCurrentCamera()->getViewport();

  io.DisplaySize = ImVec2(viewport->width(), viewport->height());

  const auto currentTime =
      renderInfo.getView()->getFrameStamp()->getSimulationTime();

  io.DeltaTime =
      mTime > 0.0 ? static_cast<float>(currentTime - mTime) : 1.0f / 60.0f;
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
}

} // namespace osg
} // namespace gui
} // namespace dart
