/*
 * Copyright (c) 2011-2018, The DART development contributors
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

/*
 * Copyright (C) 2016 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "dart/gui/filament/App.hpp"

#include <cassert>

#if !defined(WIN32)
#include <unistd.h>
#else
#include <SDL_syswm.h>
#include <utils/unwindows.h>
#endif

//#include <imgui.h>

//#include "Panic.hpp"
#include "Cube.hpp"
#include "Path.hpp"

#include <filament/Camera.h>
#include <filament/Material.h>
#include <filament/MaterialInstance.h>
#include <filament/RenderableManager.h>
#include <filament/Renderer.h>
#include <filament/Scene.h>
#include <filament/Skybox.h>
#include <filament/View.h>

#include "ImGuiHelper.hpp"

//#include "Cube.h"
//#include "NativeWindowHelper.h"

#include "dart/external/imgui/imgui.h"
#include "dart/gui/filament/ImGuiHelper.hpp"
#include "dart/gui/filament/Window.hpp"

namespace dart {
namespace gui {
namespace flmt {

static constexpr uint8_t AI_DEFAULT_MAT_PACKAGE[] = {
    #include "generated/material/aiDefaultMat.inc"
};

static constexpr uint8_t TRANSPARENT_COLOR_PACKAGE[] = {
    #include "generated/material/transparentColor.inc"
};

static constexpr uint8_t DEPTH_VISUALIZER_PACKAGE[] = {
    #include "generated/material/depthVisualizer.inc"
};

FilamentApp& FilamentApp::get()
{
  static FilamentApp filamentApp;
  return filamentApp;
}

FilamentApp::FilamentApp()
{
  initSDL();
}

FilamentApp::~FilamentApp()
{
  SDL_Quit();
}

void FilamentApp::run(
    const Config& config,
    SetupCallback setupCallback,
    CleanupCallback cleanupCallback,
    ImGuiCallback imguiCallback,
    PreRenderCallback preRender,
    PostRenderCallback postRender,
    size_t width,
    size_t height)
{
  std::unique_ptr<Window> window(
      new Window(this, config, config.title, width, height));

  mDepthMaterial = filament::Material::Builder()
          .package(
              (void*)DEPTH_VISUALIZER_PACKAGE,
              sizeof(DEPTH_VISUALIZER_PACKAGE))
          .build(*mEngine);

  mDepthMI = mDepthMaterial->createInstance();

  mDefaultMaterial
          = filament::Material::Builder()
          .package(
              (void*)AI_DEFAULT_MAT_PACKAGE,
              sizeof(AI_DEFAULT_MAT_PACKAGE))
          .build(*mEngine);

  mTransparentMaterial = filament::Material::Builder()
          .package(
              (void*)TRANSPARENT_COLOR_PACKAGE,
              sizeof(TRANSPARENT_COLOR_PACKAGE))
          .build(*mEngine);

  std::unique_ptr<Cube> cameraCube(
      new Cube(*mEngine, mTransparentMaterial, {1, 0, 0}));
  // we can't cull the light-frustum because it's not applied a rigid transform
  // and currently, filament assumes that for culling
  std::unique_ptr<Cube> lightmapCube(
      new Cube(*mEngine, mTransparentMaterial, {0, 1, 0}, false));
  mScene = mEngine->createScene();

  window->mMainView->getView()->setVisibleLayers(0x4, 0x4);

  window->mUiView->getView()->setClearTargets(false, false, false);
  window->mUiView->getView()->setRenderTarget(
      filament::View::TargetBufferFlags::DEPTH_AND_STENCIL);
  window->mUiView->getView()->setPostProcessingEnabled(false);
  window->mUiView->getView()->setShadowsEnabled(false);

  if (config.splitView)
  {
    auto& rcm = mEngine->getRenderableManager();

    rcm.setLayerMask(
        rcm.getInstance(cameraCube->getSolidRenderable()), 0x3, 0x2);
    rcm.setLayerMask(
        rcm.getInstance(cameraCube->getWireFrameRenderable()), 0x3, 0x2);
    cameraCube->mapFrustum(*mEngine, window->mMainCameraMan.getCamera());

    rcm.setLayerMask(
        rcm.getInstance(lightmapCube->getSolidRenderable()), 0x3, 0x2);
    rcm.setLayerMask(
        rcm.getInstance(lightmapCube->getWireFrameRenderable()), 0x3, 0x2);

    // Create the camera mesh
    window->mMainCameraMan.setCameraChangedCallback(
        [&cameraCube, &lightmapCube, &window, engine = mEngine ](
            filament::Camera const* camera) {
          cameraCube->mapFrustum(*engine, camera);
          lightmapCube->mapFrustum(
              *engine,
              window->mMainView->getView()->getDirectionalLightCamera());
        });

    mScene->addEntity(cameraCube->getWireFrameRenderable());
    mScene->addEntity(cameraCube->getSolidRenderable());

    mScene->addEntity(lightmapCube->getWireFrameRenderable());
    mScene->addEntity(lightmapCube->getSolidRenderable());

    window->mGodView->getView()->setVisibleLayers(0x6, 0x6);
    window->mOrthoView->getView()->setVisibleLayers(0x6, 0x6);

    // only preserve the color buffer for additional views; depth and stencil
    // can be discarded.
    window->mDepthView->getView()->setRenderTarget(
        filament::View::TargetBufferFlags::DEPTH_AND_STENCIL);
    window->mGodView->getView()->setRenderTarget(
        filament::View::TargetBufferFlags::DEPTH_AND_STENCIL);
    window->mOrthoView->getView()->setRenderTarget(
        filament::View::TargetBufferFlags::DEPTH_AND_STENCIL);

    window->mDepthView->getView()->setShadowsEnabled(false);
    window->mGodView->getView()->setShadowsEnabled(false);
    window->mOrthoView->getView()->setShadowsEnabled(false);
  }

  loadIBL(config);
  if (mIBL != nullptr)
  {
    mIBL->getSkybox()->setLayerMask(0x7, 0x4);
    mScene->setSkybox(mIBL->getSkybox());
    mScene->setIndirectLight(mIBL->getIndirectLight());
  }

  for (auto& view : window->mViews)
  {
    if (view.get() != window->mUiView)
    {
      view->getView()->setScene(mScene);
    }
  }

  setupCallback(mEngine, window->mMainView->getView(), mScene);

  if (imguiCallback)
  {
    mImGuiHelper = std::make_unique<filagui::ImGuiHelper>(
        mEngine,
        window->mUiView->getView(),
        getRootPath() + "assets/fonts/Roboto-Medium.ttf");
    ImGuiIO& io = ImGui::GetIO();
#ifdef WIN32
    SDL_SysWMinfo wmInfo;
    SDL_VERSION(&wmInfo.version);
    SDL_GetWindowWMInfo(window->getSDLWindow(), &wmInfo);
    io.ImeWindowHandle = wmInfo.info.win.window;
#endif
    io.KeyMap[ImGuiKey_Tab] = SDL_SCANCODE_TAB;
    io.KeyMap[ImGuiKey_LeftArrow] = SDL_SCANCODE_LEFT;
    io.KeyMap[ImGuiKey_RightArrow] = SDL_SCANCODE_RIGHT;
    io.KeyMap[ImGuiKey_UpArrow] = SDL_SCANCODE_UP;
    io.KeyMap[ImGuiKey_DownArrow] = SDL_SCANCODE_DOWN;
    io.KeyMap[ImGuiKey_PageUp] = SDL_SCANCODE_PAGEUP;
    io.KeyMap[ImGuiKey_PageDown] = SDL_SCANCODE_PAGEDOWN;
    io.KeyMap[ImGuiKey_Home] = SDL_SCANCODE_HOME;
    io.KeyMap[ImGuiKey_End] = SDL_SCANCODE_END;
    io.KeyMap[ImGuiKey_Insert] = SDL_SCANCODE_INSERT;
    io.KeyMap[ImGuiKey_Delete] = SDL_SCANCODE_DELETE;
    io.KeyMap[ImGuiKey_Backspace] = SDL_SCANCODE_BACKSPACE;
    io.KeyMap[ImGuiKey_Space] = SDL_SCANCODE_SPACE;
    io.KeyMap[ImGuiKey_Enter] = SDL_SCANCODE_RETURN;
    io.KeyMap[ImGuiKey_Escape] = SDL_SCANCODE_ESCAPE;
    io.KeyMap[ImGuiKey_A] = SDL_SCANCODE_A;
    io.KeyMap[ImGuiKey_C] = SDL_SCANCODE_C;
    io.KeyMap[ImGuiKey_V] = SDL_SCANCODE_V;
    io.KeyMap[ImGuiKey_X] = SDL_SCANCODE_X;
    io.KeyMap[ImGuiKey_Y] = SDL_SCANCODE_Y;
    io.KeyMap[ImGuiKey_Z] = SDL_SCANCODE_Z;
    io.SetClipboardTextFn
        = [](void*, const char* text) { SDL_SetClipboardText(text); };
    io.GetClipboardTextFn
        = [](void*) -> const char* { return SDL_GetClipboardText(); };
    io.ClipboardUserData = nullptr;
  }

  bool mousePressed[3] = {false};

  while (!mClosed)
  {

    if (!UTILS_HAS_THREADING)
    {
      mEngine->execute();
    }

    // Allow the app to animate the scene if desired.
    if (mAnimation)
    {
      double now
          = (double)SDL_GetPerformanceCounter() / SDL_GetPerformanceFrequency();
      mAnimation(mEngine, window->mMainView->getView(), now);
    }

    // Loop over fresh events twice: first stash them and let ImGui process
    // them, then allow
    // the app to process the stashed events. This is done because ImGui might
    // wish to block
    // certain events from the app (e.g., when dragging the mouse over an
    // obscuring window).
    constexpr int kMaxEvents = 16;
    SDL_Event events[kMaxEvents];
    int nevents = 0;
    while (nevents < kMaxEvents && SDL_PollEvent(&events[nevents]) != 0)
    {
      if (mImGuiHelper)
      {
        ImGuiIO& io = ImGui::GetIO();
        SDL_Event* event = &events[nevents];
        switch (event->type)
        {
          case SDL_MOUSEWHEEL:
          {
            if (event->wheel.x > 0)
              io.MouseWheelH += 1;
            if (event->wheel.x < 0)
              io.MouseWheelH -= 1;
            if (event->wheel.y > 0)
              io.MouseWheel += 1;
            if (event->wheel.y < 0)
              io.MouseWheel -= 1;
            break;
          }
          case SDL_MOUSEBUTTONDOWN:
          {
            if (event->button.button == SDL_BUTTON_LEFT)
              mousePressed[0] = true;
            if (event->button.button == SDL_BUTTON_RIGHT)
              mousePressed[1] = true;
            if (event->button.button == SDL_BUTTON_MIDDLE)
              mousePressed[2] = true;
            break;
          }
          case SDL_TEXTINPUT:
          {
            io.AddInputCharactersUTF8(event->text.text);
            break;
          }
          case SDL_KEYDOWN:
          case SDL_KEYUP:
          {
            int key = event->key.keysym.scancode;
            IM_ASSERT(key >= 0 && key < IM_ARRAYSIZE(io.KeysDown));
            io.KeysDown[key] = (event->type == SDL_KEYDOWN);
            io.KeyShift = ((SDL_GetModState() & KMOD_SHIFT) != 0);
            io.KeyAlt = ((SDL_GetModState() & KMOD_ALT) != 0);
            io.KeyCtrl = ((SDL_GetModState() & KMOD_CTRL) != 0);
            io.KeySuper = ((SDL_GetModState() & KMOD_GUI) != 0);
            break;
          }
        }
      }
      nevents++;
    }

    // Now, loop over the events a second time for app-side processing.
    for (int i = 0; i < nevents; i++)
    {
      const SDL_Event& event = events[i];
      ImGuiIO* io = mImGuiHelper ? &ImGui::GetIO() : nullptr;
      switch (event.type)
      {
        case SDL_QUIT:
          mClosed = true;
          break;
        case SDL_KEYDOWN:
          if (event.key.keysym.scancode == SDL_SCANCODE_ESCAPE)
          {
            mClosed = true;
          }
          break;
        case SDL_MOUSEWHEEL:
          if (!io || !io->WantCaptureMouse)
            window->mouseWheel(event.wheel.y);
          break;
        case SDL_MOUSEBUTTONDOWN:
          if (!io || !io->WantCaptureMouse)
            window->mouseDown(
                event.button.button, event.button.x, event.button.y);
          break;
        case SDL_MOUSEBUTTONUP:
          if (!io || !io->WantCaptureMouse)
            window->mouseUp(event.button.x, event.button.y);
          break;
        case SDL_MOUSEMOTION:
          if (!io || !io->WantCaptureMouse)
            window->mouseMoved(event.motion.x, event.motion.y);
          break;
        case SDL_WINDOWEVENT:
          switch (event.window.event)
          {
            case SDL_WINDOWEVENT_RESIZED:
              window->resize();
              break;
            default:
              break;
          }
          break;
        default:
          break;
      }
    }

    // Populate the UI scene, regardless of whether Filament wants to a skip
    // frame. We should
    // always let ImGui generate a command list; if it skips a frame it'll
    // destroy its widgets.
    if (mImGuiHelper)
    {

      // Inform ImGui of the current window size in case it was resized.
      int windowWidth, windowHeight;
      int displayWidth, displayHeight;
      SDL_GetWindowSize(window->mWindow, &windowWidth, &windowHeight);
      SDL_GL_GetDrawableSize(window->mWindow, &displayWidth, &displayHeight);
      mImGuiHelper->setDisplaySize(
          windowWidth,
          windowHeight,
          windowWidth > 0 ? ((float)displayWidth / windowWidth) : 0,
          displayHeight > 0 ? ((float)displayHeight / windowHeight) : 0);

      // Setup mouse inputs (we already got mouse wheel, keyboard keys &
      // characters
      // from our event handler)
      ImGuiIO& io = ImGui::GetIO();
      int mx, my;
      Uint32 buttons = SDL_GetMouseState(&mx, &my);
      io.MousePos = ImVec2(-FLT_MAX, -FLT_MAX);
      io.MouseDown[0]
          = mousePressed[0] || (buttons & SDL_BUTTON(SDL_BUTTON_LEFT)) != 0;
      io.MouseDown[1]
          = mousePressed[1] || (buttons & SDL_BUTTON(SDL_BUTTON_RIGHT)) != 0;
      io.MouseDown[2]
          = mousePressed[2] || (buttons & SDL_BUTTON(SDL_BUTTON_MIDDLE)) != 0;
      mousePressed[0] = mousePressed[1] = mousePressed[2] = false;

      // TODO: Update to a newer SDL and use SDL_CaptureMouse() to retrieve
      // mouse coordinates
      // outside of the client area; see the imgui SDL example.
      if ((SDL_GetWindowFlags(window->mWindow) & SDL_WINDOW_INPUT_FOCUS) != 0)
      {
        io.MousePos = ImVec2((float)mx, (float)my);
      }

      // Populate the UI Scene.
      static Uint64 frequency = SDL_GetPerformanceFrequency();
      Uint64 now = SDL_GetPerformanceCounter();
      float timeStep = mTime > 0 ? (float)((double)(now - mTime) / frequency)
                                 : (float)(1.0f / 60.0f);
      mTime = now;
      mImGuiHelper->render(timeStep, imguiCallback);
    }

    window->mMainCameraMan.updateCameraTransform();

    // TODO: we need better timing or use SDL_GL_SetSwapInterval
    SDL_Delay(16);

    filament::Renderer* renderer = window->getRenderer();

    if (preRender)
    {
      for (auto const& view : window->mViews)
      {
        if (view.get() != window->mUiView)
        {
          preRender(mEngine, view->getView(), mScene, renderer);
        }
      }
    }

    if (renderer->beginFrame(window->getSwapChain()))
    {
      for (auto const& view : window->mViews)
      {
        renderer->render(view->getView());
      }
      renderer->endFrame();
    }

    if (postRender)
    {
      for (auto const& view : window->mViews)
      {
        if (view.get() != window->mUiView)
        {
          postRender(mEngine, view->getView(), mScene, renderer);
        }
      }
    }
  }

  if (mImGuiHelper)
  {
    mImGuiHelper.reset();
  }

  cleanupCallback(mEngine, window->mMainView->getView(), mScene);

  cameraCube.reset();
  lightmapCube.reset();
  window.reset();

  mIBL.reset();
  mEngine->destroy(mDepthMI);
  mEngine->destroy(mDepthMaterial);
  mEngine->destroy(mDefaultMaterial);
  mEngine->destroy(mTransparentMaterial);
  mEngine->destroy(mScene);
  filament::Engine::destroy(&mEngine);
  mEngine = nullptr;
}

void FilamentApp::loadIBL(const Config& config)
{
  if (!config.iblDirectory.empty())
  {
    utils::Path iblPath(config.iblDirectory);

    if (!iblPath.exists())
    {
      std::cerr << "The specified IBL path does not exist: " << iblPath
                << std::endl;
      return;
    }

    if (!iblPath.isDirectory())
    {
      std::cerr << "The specified IBL path is not a directory: " << iblPath
                << std::endl;
      return;
    }

    mIBL = std::make_unique<IBL>(*mEngine);
    if (!mIBL->loadFromDirectory(iblPath))
    {
      std::cerr << "Could not load the specified IBL: " << iblPath << std::endl;
      mIBL.reset(nullptr);
      return;
    }
  }
}

void FilamentApp::initSDL()
{
  if (SDL_Init(SDL_INIT_EVENTS) != 0)
  {
    assert(false && "SDL_Init Failure");
  }
}

} // namespace flmt
} // namespace gui
} // namespace dart
