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

#include "dart/common/Console.hpp"
#include "dart/common/Macros.hpp"
#include "dart/gui/osg/ImGuiWidget.hpp"
#include "dart/gui/osg/IncludeImGui.hpp"

#include <osg/Camera>
#include <osg/GLExtensions>
#include <osg/RenderInfo>

#include <algorithm>
#include <vector>

namespace dart {
namespace gui {
namespace osg {

#ifdef GL_ACTIVE_TEXTURE
constexpr GLenum kGLActiveTexture = GL_ACTIVE_TEXTURE;
#else
constexpr GLenum kGLActiveTexture = 0x84E0;
#endif

#ifdef GL_CLIENT_ACTIVE_TEXTURE
constexpr GLenum kGLClientActiveTexture = GL_CLIENT_ACTIVE_TEXTURE;
#else
constexpr GLenum kGLClientActiveTexture = 0x84E1;
#endif

#if IMGUI_VERSION_NUM < 19150

// Special keys that are usually greater than 512 in osgGA
//
// Important Note: Dear ImGui expects the control Keys indices not to be greater
// thant 511. It actually uses an array of 512 elements. However, OSG has
// indices greater than that. So here I do a conversion for special keys between
// ImGui and OSG.
enum ConvertedKey : int
{
  ConvertedKey_None = -1,
  ConvertedKey_Tab = 257,
  ConvertedKey_Left,
  ConvertedKey_Right,
  ConvertedKey_Up,
  ConvertedKey_Down,
  ConvertedKey_PageUp,
  ConvertedKey_PageDown,
  ConvertedKey_Home,
  ConvertedKey_End,
  ConvertedKey_Delete,
  ConvertedKey_BackSpace,
  ConvertedKey_Enter,
  ConvertedKey_Escape,
  ConvertedKey_LeftControl,
  ConvertedKey_RightControl,
  ConvertedKey_LeftShift,
  ConvertedKey_RightShift,
  ConvertedKey_LeftAlt,
  ConvertedKey_RightAlt,
  ConvertedKey_LeftSuper,
  ConvertedKey_RightSuper,
};

#endif

//==============================================================================
// Check for a special key and return the converted code (range [257, 511]) if
// so. Otherwise returns -1
#if IMGUI_VERSION_NUM >= 19150
ImGuiKey convertFromOSGKey(int key)
#else
ConvertedKey convertFromOSGKey(int key)
#endif
{
  using KeySymbol = osgGA::GUIEventAdapter::KeySymbol;

  switch (key) {
#if IMGUI_VERSION_NUM >= 19150
    case KeySymbol::KEY_Tab:
      return ImGuiKey_Tab;
    case KeySymbol::KEY_Left:
      return ImGuiKey_LeftArrow;
    case KeySymbol::KEY_Right:
      return ImGuiKey_RightArrow;
    case KeySymbol::KEY_Up:
      return ImGuiKey_UpArrow;
    case KeySymbol::KEY_Down:
      return ImGuiKey_DownArrow;
    case KeySymbol::KEY_Page_Up:
      return ImGuiKey_PageUp;
    case KeySymbol::KEY_Page_Down:
      return ImGuiKey_PageDown;
    case KeySymbol::KEY_Home:
      return ImGuiKey_Home;
    case KeySymbol::KEY_End:
      return ImGuiKey_End;
    case KeySymbol::KEY_Delete:
      return ImGuiKey_Delete;
    case KeySymbol::KEY_BackSpace:
      return ImGuiKey_Backspace;
    case KeySymbol::KEY_Return:
      return ImGuiKey_Enter;
    case KeySymbol::KEY_Escape:
      return ImGuiKey_Escape;
    case KeySymbol::KEY_Control_L:
    case KeySymbol::KEY_Control_R:
      return ImGuiMod_Ctrl;
    case KeySymbol::KEY_Shift_L:
    case KeySymbol::KEY_Shift_R:
      return ImGuiMod_Shift;
    case KeySymbol::KEY_Alt_L:
    case KeySymbol::KEY_Alt_R:
      return ImGuiMod_Alt;
    case KeySymbol::KEY_Super_L:
    case KeySymbol::KEY_Super_R:
      return ImGuiMod_Super;
    case KeySymbol::KEY_A:
      return ImGuiKey_A;
    case KeySymbol::KEY_C:
      return ImGuiKey_C;
    case KeySymbol::KEY_V:
      return ImGuiKey_V;
    case KeySymbol::KEY_X:
      return ImGuiKey_X;
    case KeySymbol::KEY_Y:
      return ImGuiKey_Y;
    case KeySymbol::KEY_Z:
      return ImGuiKey_Z;
    default:
      return ImGuiKey_None;
#else
    case KeySymbol::KEY_Tab:
      return ConvertedKey_Tab;
    case KeySymbol::KEY_Left:
      return ConvertedKey_Left;
    case KeySymbol::KEY_Right:
      return ConvertedKey_Right;
    case KeySymbol::KEY_Up:
      return ConvertedKey_Up;
    case KeySymbol::KEY_Down:
      return ConvertedKey_Down;
    case KeySymbol::KEY_Page_Up:
      return ConvertedKey_PageUp;
    case KeySymbol::KEY_Page_Down:
      return ConvertedKey_PageDown;
    case KeySymbol::KEY_Home:
      return ConvertedKey_Home;
    case KeySymbol::KEY_End:
      return ConvertedKey_End;
    case KeySymbol::KEY_Delete:
      return ConvertedKey_Delete;
    case KeySymbol::KEY_BackSpace:
      return ConvertedKey_BackSpace;
    case KeySymbol::KEY_Return:
      return ConvertedKey_Enter;
    case KeySymbol::KEY_Escape:
      return ConvertedKey_Escape;
    case KeySymbol::KEY_Control_L:
      return ConvertedKey_LeftControl;
    case KeySymbol::KEY_Control_R:
      return ConvertedKey_RightControl;
    case KeySymbol::KEY_Shift_L:
      return ConvertedKey_LeftShift;
    case KeySymbol::KEY_Shift_R:
      return ConvertedKey_RightShift;
    case KeySymbol::KEY_Alt_L:
      return ConvertedKey_LeftAlt;
    case KeySymbol::KEY_Alt_R:
      return ConvertedKey_RightAlt;
    case KeySymbol::KEY_Super_L:
      return ConvertedKey_LeftSuper;
    case KeySymbol::KEY_Super_R:
      return ConvertedKey_RightSuper;
    default:
      return ConvertedKey_None;
#endif
  }
}

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
class ScopedOpenGL2RenderState
{
public:
  struct TextureUnitState
  {
    bool mTexture2DEnabled = false;
    bool mTextureGenSEnabled = false;
    bool mTextureGenTEnabled = false;
    bool mTextureGenREnabled = false;
    bool mTextureGenQEnabled = false;
  };

  explicit ScopedOpenGL2RenderState(::osg::RenderInfo& renderInfo)
  {
    mExtensions = ::osg::GLExtensions::Get(renderInfo.getContextID(), true);
    if (!mExtensions)
      return;

    if (mExtensions->glUseProgram) {
      mPreviousProgram = mExtensions->getCurrentProgram();
      if (mPreviousProgram != 0)
        mExtensions->glUseProgram(0);
    }

    if (mExtensions->glActiveTexture) {
      glGetIntegerv(kGLActiveTexture, &mPreviousActiveTexture);

      GLint maxTextureUnits = 1;
      glGetIntegerv(GL_MAX_TEXTURE_UNITS, &maxTextureUnits);
      maxTextureUnits = std::max(1, maxTextureUnits);
      mTextureUnits.resize(static_cast<std::size_t>(maxTextureUnits));

      for (GLint i = 0; i < maxTextureUnits; ++i) {
        const GLenum unit = static_cast<GLenum>(GL_TEXTURE0 + i);
        mExtensions->glActiveTexture(unit);

        auto& state = mTextureUnits[static_cast<std::size_t>(i)];
        state.mTexture2DEnabled = (glIsEnabled(GL_TEXTURE_2D) == GL_TRUE);
        state.mTextureGenSEnabled = (glIsEnabled(GL_TEXTURE_GEN_S) == GL_TRUE);
        state.mTextureGenTEnabled = (glIsEnabled(GL_TEXTURE_GEN_T) == GL_TRUE);
        state.mTextureGenREnabled = (glIsEnabled(GL_TEXTURE_GEN_R) == GL_TRUE);
        state.mTextureGenQEnabled = (glIsEnabled(GL_TEXTURE_GEN_Q) == GL_TRUE);

        if (i > 0)
          glDisable(GL_TEXTURE_2D);
        glDisable(GL_TEXTURE_GEN_S);
        glDisable(GL_TEXTURE_GEN_T);
        glDisable(GL_TEXTURE_GEN_R);
        glDisable(GL_TEXTURE_GEN_Q);
      }

      mExtensions->glActiveTexture(GL_TEXTURE0);
    }

    if (mExtensions->glClientActiveTexture) {
      glGetIntegerv(kGLClientActiveTexture, &mPreviousClientActiveTexture);
      mExtensions->glClientActiveTexture(GL_TEXTURE0);
    }
  }

  ~ScopedOpenGL2RenderState()
  {
    if (!mExtensions)
      return;

    if (mExtensions->glActiveTexture) {
      for (std::size_t i = 0; i < mTextureUnits.size(); ++i) {
        const GLenum unit = static_cast<GLenum>(GL_TEXTURE0 + i);
        mExtensions->glActiveTexture(unit);
        const auto& state = mTextureUnits[i];
        if (state.mTexture2DEnabled)
          glEnable(GL_TEXTURE_2D);
        else
          glDisable(GL_TEXTURE_2D);

        if (state.mTextureGenSEnabled)
          glEnable(GL_TEXTURE_GEN_S);
        else
          glDisable(GL_TEXTURE_GEN_S);
        if (state.mTextureGenTEnabled)
          glEnable(GL_TEXTURE_GEN_T);
        else
          glDisable(GL_TEXTURE_GEN_T);
        if (state.mTextureGenREnabled)
          glEnable(GL_TEXTURE_GEN_R);
        else
          glDisable(GL_TEXTURE_GEN_R);
        if (state.mTextureGenQEnabled)
          glEnable(GL_TEXTURE_GEN_Q);
        else
          glDisable(GL_TEXTURE_GEN_Q);
      }
      mExtensions->glActiveTexture(mPreviousActiveTexture);
    }

    if (mExtensions->glClientActiveTexture)
      mExtensions->glClientActiveTexture(mPreviousClientActiveTexture);

    if (mExtensions->glUseProgram && mPreviousProgram != 0)
      mExtensions->glUseProgram(mPreviousProgram);
  }

private:
  ::osg::GLExtensions* mExtensions = nullptr;
  GLuint mPreviousProgram = 0;
  GLint mPreviousActiveTexture = GL_TEXTURE0;
  GLint mPreviousClientActiveTexture = GL_TEXTURE0;
  std::vector<TextureUnitState> mTextureUnits;
};

//==============================================================================
ImGuiHandler::ImGuiHandler()
  : mTime{0.0},
    mMousePressed{false, false, false},
    mMouseDown{false, false, false},
    mMouseWheel{0.0f}
{
  ImGui::CreateContext();

  ImGui::StyleColorsDark();

  ImGui_ImplOpenGL2_Init();

#if IMGUI_VERSION_NUM < 19150
  // Keyboard mapping. ImGui will use those indices to peek into the
  // io.KeyDown[] array.
  ImGuiIO& io = ImGui::GetIO();
  io.KeyMap[ImGuiKey_Tab] = ConvertedKey_Tab;
  io.KeyMap[ImGuiKey_LeftArrow] = ConvertedKey_Left;
  io.KeyMap[ImGuiKey_RightArrow] = ConvertedKey_Right;
  io.KeyMap[ImGuiKey_UpArrow] = ConvertedKey_Up;
  io.KeyMap[ImGuiKey_DownArrow] = ConvertedKey_Down;
  io.KeyMap[ImGuiKey_PageUp] = ConvertedKey_PageUp;
  io.KeyMap[ImGuiKey_PageDown] = ConvertedKey_PageDown;
  io.KeyMap[ImGuiKey_Home] = ConvertedKey_Home;
  io.KeyMap[ImGuiKey_End] = ConvertedKey_End;
  io.KeyMap[ImGuiKey_Delete] = ConvertedKey_Delete;
  io.KeyMap[ImGuiKey_Backspace] = ConvertedKey_BackSpace;
  io.KeyMap[ImGuiKey_Enter] = ConvertedKey_Enter;
  io.KeyMap[ImGuiKey_Escape] = ConvertedKey_Escape;
  io.KeyMap[ImGuiKey_A] = osgGA::GUIEventAdapter::KeySymbol::KEY_A;
  io.KeyMap[ImGuiKey_C] = osgGA::GUIEventAdapter::KeySymbol::KEY_C;
  io.KeyMap[ImGuiKey_V] = osgGA::GUIEventAdapter::KeySymbol::KEY_V;
  io.KeyMap[ImGuiKey_X] = osgGA::GUIEventAdapter::KeySymbol::KEY_X;
  io.KeyMap[ImGuiKey_Y] = osgGA::GUIEventAdapter::KeySymbol::KEY_Y;
  io.KeyMap[ImGuiKey_Z] = osgGA::GUIEventAdapter::KeySymbol::KEY_Z;
#endif
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
  if (hasWidget(widget)) {
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
  if (!hasWidget(widget)) {
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
  ImGuiIO& io = ImGui::GetIO();
  const bool wantCaptureMouse = io.WantCaptureMouse;
  const bool wantCaptureKeyboard = io.WantCaptureKeyboard;

  switch (eventAdapter.getEventType()) {
    case osgGA::GUIEventAdapter::KEYDOWN: {
      const int key = eventAdapter.getUnmodifiedKey();

#if IMGUI_VERSION_NUM >= 19150
      const ImGuiKey specialKey = convertFromOSGKey(key);
      if (specialKey != ImGuiKey_None) {
        io.AddKeyEvent(specialKey, true);
      } else if (key != 0 && key < 0x10000) {
        const ImWchar c = static_cast<ImWchar>(eventAdapter.getKey());
        io.AddInputCharacter(c);
      }
#else
      const ConvertedKey specialKey = convertFromOSGKey(key);
      if (specialKey != ConvertedKey_None) {
        DART_ASSERT(specialKey < 512 && "ImGui KeysDown is an array of 512");
        DART_ASSERT(
            specialKey > 256
            && "ASCII stop at 127, but we use the range [257, 511]");

        io.KeysDown[specialKey] = true;

        io.KeyCtrl = io.KeysDown[ConvertedKey_LeftControl]
                     || io.KeysDown[ConvertedKey_RightControl];
        io.KeyShift = io.KeysDown[ConvertedKey_LeftShift]
                      || io.KeysDown[ConvertedKey_RightShift];
        io.KeyAlt = io.KeysDown[ConvertedKey_LeftAlt]
                    || io.KeysDown[ConvertedKey_RightAlt];
        io.KeySuper = io.KeysDown[ConvertedKey_LeftSuper]
                      || io.KeysDown[ConvertedKey_RightSuper];
      } else if (0 < key && key < 0x10000) {
        io.KeysDown[key] = true;
        io.AddInputCharacter(static_cast<ImWchar>(key));
      }
#endif

      return wantCaptureKeyboard;
    }
    case osgGA::GUIEventAdapter::KEYUP: {
      const int key = eventAdapter.getUnmodifiedKey();

#if IMGUI_VERSION_NUM >= 19150
      const ImGuiKey specialKey = convertFromOSGKey(key);
      if (specialKey != ImGuiKey_None) {
        io.AddKeyEvent(specialKey, false);
      }
#else
      const ConvertedKey specialKey = convertFromOSGKey(key);
      if (specialKey != ConvertedKey_None) {
        DART_ASSERT(specialKey < 512 && "ImGui KeysDown is an array of 512");
        DART_ASSERT(
            specialKey > 256
            && "ASCII stop at 127, but we use the range [257, 511]");

        io.KeysDown[specialKey] = false;

        io.KeyCtrl = io.KeysDown[ConvertedKey_LeftControl]
                     || io.KeysDown[ConvertedKey_RightControl];
        io.KeyShift = io.KeysDown[ConvertedKey_LeftShift]
                      || io.KeysDown[ConvertedKey_RightShift];
        io.KeyAlt = io.KeysDown[ConvertedKey_LeftAlt]
                    || io.KeysDown[ConvertedKey_RightAlt];
        io.KeySuper = io.KeysDown[ConvertedKey_LeftSuper]
                      || io.KeysDown[ConvertedKey_RightSuper];
      } else if (0 < key && key < 0x10000) {
        io.KeysDown[key] = false;
        io.AddInputCharacter(static_cast<ImWchar>(key));
      }
#endif

      return wantCaptureKeyboard;
    }
    case osgGA::GUIEventAdapter::PUSH: {
      io.MousePos
          = ImVec2(eventAdapter.getX(), io.DisplaySize.y - eventAdapter.getY());

      if (eventAdapter.getButtonMask()
          == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON) {
        mMousePressed[0] = true;
        mMouseDown[0] = true;
      } else if (
          eventAdapter.getButtonMask()
          == osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON) {
        mMousePressed[1] = true;
        mMouseDown[1] = true;
      } else if (
          eventAdapter.getButtonMask()
          == osgGA::GUIEventAdapter::MIDDLE_MOUSE_BUTTON) {
        mMousePressed[2] = true;
        mMouseDown[2] = true;
      } else {
        // Shouldn't be reached to here. Mark left button by default.
        mMousePressed[0] = true;
        mMouseDown[0] = true;
      }

      return wantCaptureMouse;
    }
    case osgGA::GUIEventAdapter::DRAG:
    case osgGA::GUIEventAdapter::MOVE: {
      io.MousePos
          = ImVec2(eventAdapter.getX(), io.DisplaySize.y - eventAdapter.getY());

      return wantCaptureMouse;
    }
    case osgGA::GUIEventAdapter::RELEASE: {
      // When a mouse button is released no button mask is set. So we mark all
      // the buttons are released.
      mMouseDown[0] = false;
      mMouseDown[1] = false;
      mMouseDown[2] = false;

      return wantCaptureMouse;
    }
    case osgGA::GUIEventAdapter::SCROLL: {
      constexpr float increment = 0.1f;

      switch (eventAdapter.getScrollingMotion()) {
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

      return wantCaptureMouse;
    }
    default: {
      return false;
    }
  }
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
  DART_ASSERT(mTime >= 0.0);

  for (auto i = 0u; i < mMousePressed.size(); ++i) {
    io.MouseDown[i] = mMouseDown[i] || mMousePressed[i];
    mMousePressed[i] = false;
  }

  io.MouseWheel = mMouseWheel;
  mMouseWheel = 0.0f;

  ImGui::NewFrame();
}

//==============================================================================
void ImGuiHandler::render(::osg::RenderInfo& renderInfo)
{
  for (const auto& widget : mWidgets) {
    if (widget->isVisible())
      widget->render();
  }

  ImGui::Render();

  auto* drawData = ImGui::GetDrawData();
#ifdef IMGUI_HAS_TEXTURES
  if (drawData->Textures) {
    for (ImTextureData* texture : *drawData->Textures) {
      if (texture->Status != ImTextureStatus_OK)
        ImGui_ImplOpenGL2_UpdateTexture(texture);
    }
  }
#endif

  // The legacy OpenGL2 backend renders through the fixed pipeline. OSG may
  // leave a GLSL program bound after drawing the 3D scene; if so, text glyphs
  // are drawn as solid quads because the active shader ignores the font atlas.
  //
  // OSG shadow/render passes may also leave a nonzero active texture unit with
  // other fixed-pipeline texture units or texture-coordinate generation
  // enabled. The backend assumes texture unit 0 with caller-provided UVs, so
  // force that state for the duration of the ImGui draw and restore it
  // afterwards.
  ScopedOpenGL2RenderState gl2State(renderInfo);
  ImGui_ImplOpenGL2_RenderDrawData(drawData);
}

} // namespace osg
} // namespace gui
} // namespace dart
