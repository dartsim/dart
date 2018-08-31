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

#include "dart/gui/filament/Window.hpp"

#include "stb_image.h"

#include <fstream>
#include <string>

#include <filament/Engine.h>
#include <filament/IndirectLight.h>
#include <filament/Material.h>
#include <filament/MaterialInstance.h>
#include <filament/Skybox.h>
#include <filament/Texture.h>

#include "dart/gui/filament/App.hpp"
#include "dart/gui/filament/NativeWindowHelper.h"

namespace dart {
namespace gui {
namespace flmt {

Window::Window(
    FilamentApp* filamentApp,
    const Config& config,
    std::string title,
    size_t w,
    size_t h)
  : mFilamentApp(filamentApp)
{
  const int x = SDL_WINDOWPOS_CENTERED;
  const int y = SDL_WINDOWPOS_CENTERED;
  const uint32_t windowFlags
      = SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI;
  mWindow = SDL_CreateWindow(title.c_str(), x, y, (int)w, (int)h, windowFlags);

  // Create the Engine after the window in case this happens to be a
  // single-threaded platform.
  // For single-threaded platforms, we need to ensure that Filament's OpenGL
  // context is current,
  // rather than the one created by SDL.
  mFilamentApp->setEngine(filament::Engine::create(config.backend));
//    mFilamentApp->mEngine = Engine::create(config.backend);

// HACK: We don't use SDL's 2D rendering functionality, but by invoking it we
// cause
// SDL to create a Metal backing layer, which allows us to run Vulkan apps via
// MoltenVK.
#if defined(FILAMENT_DRIVER_SUPPORTS_VULKAN) && defined(__APPLE__)
  constexpr int METAL_DRIVER = 2;
  SDL_CreateRenderer(mWindow, METAL_DRIVER, SDL_RENDERER_ACCELERATED);
#endif

  void* nativeWindow = ::getNativeWindow(mWindow);
  mSwapChain = mFilamentApp->mEngine->createSwapChain(nativeWindow);
  mRenderer = mFilamentApp->mEngine->createRenderer();

  // create cameras
  mCameras[0] = mMainCamera = mFilamentApp->mEngine->createCamera();
  mCameras[1] = mDebugCamera = mFilamentApp->mEngine->createCamera();
  mCameras[2] = mOrthoCamera = mFilamentApp->mEngine->createCamera();
  mCameras[3] = mUiCamera = mFilamentApp->mEngine->createCamera();

  // set exposure
  for (auto camera : mCameras)
  {
    camera->setExposure(16.0f, 1 / 125.0f, 100.0f);
  }

  // create views
  mViews.emplace_back(mMainView = new CView(*mRenderer, "Main View"));
  if (config.splitView)
  {
    mViews.emplace_back(mDepthView = new CView(*mRenderer, "Depth View"));
    mViews.emplace_back(mGodView = new GodView(*mRenderer, "God View"));
    mViews.emplace_back(mOrthoView = new CView(*mRenderer, "Ortho View"));
    mDepthView->getView()->setDepthPrepass(
        filament::View::DepthPrepass::DISABLED);
  }
  mViews.emplace_back(mUiView = new CView(*mRenderer, "UI View"));

  // set-up the camera manipulators
  math::double3 at(0, 0, -4);
  mMainCameraMan.setCamera(mMainCamera);
  mDebugCameraMan.setCamera(mDebugCamera);
  mMainView->setCamera(mMainCamera);
  mMainView->setCameraManipulator(&mMainCameraMan);
  mUiView->setCamera(mUiCamera);
  if (config.splitView)
  {
    // Depth view always uses the main camera
    mDepthView->setCamera(mMainCamera);

    // The god view uses the main camera for culling, but the debug camera for
    // viewing
    mGodView->setCamera(mMainCamera);
    mGodView->setGodCamera(mDebugCamera);

    // Ortho view obviously uses an ortho camera
    mOrthoView->setCamera(
        (filament::Camera*)mMainView->getView()->getDirectionalLightCamera());

    mDepthView->setCameraManipulator(&mMainCameraMan);
    mGodView->setCameraManipulator(&mDebugCameraMan);
    mOrthoView->setCameraManipulator(&mOrthoCameraMan);
  }

  // configure the cameras
  configureCamerasForWindow();

  mMainCameraMan.lookAt(at + math::double3{0, 0, 4}, at);
  mDebugCameraMan.lookAt(at + math::double3{0, 0, 4}, at);
  mOrthoCameraMan.lookAt(at + math::double3{0, 0, 4}, at);
}

Window::~Window()
{
  mViews.clear();
  for (auto& camera : mCameras)
  {
    mFilamentApp->mEngine->destroy(camera);
  }
  mFilamentApp->mEngine->destroy(mRenderer);
  mFilamentApp->mEngine->destroy(mSwapChain);
  SDL_DestroyWindow(mWindow);
}

void Window::mouseDown(int button, ssize_t x, ssize_t y)
{
  fixupMouseCoordinatesForHdpi(x, y);
  y = mHeight - y;
  for (auto const& view : mViews)
  {
    if (view->intersects(x, y))
    {
      mEventTarget = view.get();
      view->mouseDown(button, x, y);
      break;
    }
  }
}

void Window::mouseWheel(ssize_t x)
{
  if (mEventTarget)
  {
    mEventTarget->mouseWheel(x);
  }
  else
  {
    for (auto const& view : mViews)
    {
      if (view->intersects(mLastX, mLastY))
      {
        view->mouseWheel(x);
        break;
      }
    }
  }
}

void Window::mouseUp(ssize_t x, ssize_t y)
{
  fixupMouseCoordinatesForHdpi(x, y);
  if (mEventTarget)
  {
    y = mHeight - y;
    mEventTarget->mouseUp(x, y);
    mEventTarget = nullptr;
  }
}

void Window::mouseMoved(ssize_t x, ssize_t y)
{
  fixupMouseCoordinatesForHdpi(x, y);
  y = mHeight - y;
  if (mEventTarget)
  {
    mEventTarget->mouseMoved(x, y);
  }
  mLastX = x;
  mLastY = y;
}

void Window::fixupMouseCoordinatesForHdpi(ssize_t& x, ssize_t& y) const
{
  int dw, dh, ww, wh;
  SDL_GL_GetDrawableSize(mWindow, &dw, &dh);
  SDL_GetWindowSize(mWindow, &ww, &wh);
  x = x * dw / ww;
  y = y * dh / wh;
}

void Window::resize()
{
  mFilamentApp->mEngine->destroy(mSwapChain);
  mSwapChain
      = mFilamentApp->mEngine->createSwapChain(::getNativeWindow(mWindow));
  configureCamerasForWindow();
}

filament::Renderer*Window::getRenderer()
{
  return mRenderer;
}

filament::SwapChain*Window::getSwapChain()
{
  return mSwapChain;
}

SDL_Window*Window::getSDLWindow()
{
  return mWindow;
}

void Window::configureCamerasForWindow()
{

  // Determine the current size of the window in physical pixels.
  uint32_t w, h;
  SDL_GL_GetDrawableSize(mWindow, (int*)&w, (int*)&h);
  mWidth = (size_t)w;
  mHeight = (size_t)h;

  // Compute the "virtual pixels to physical pixels" scale factor that the
  // the platform uses for UI elements.
  int virtualWidth, virtualHeight;
  SDL_GetWindowSize(mWindow, &virtualWidth, &virtualHeight);
  float dpiScaleX = (float)w / virtualWidth;
  float dpiScaleY = (float)h / virtualHeight;

  const math::float3 at(0, 0, -4);
  const double ratio = double(h) / double(w);

  double near = 0.1;
  double far = 50;
  mMainCamera->setProjection(
      45.0, double(w) / h, near, far, filament::Camera::Fov::VERTICAL);
  mDebugCamera->setProjection(
      45.0, double(w) / h, 0.0625, 4096, filament::Camera::Fov::VERTICAL);
  mOrthoCamera->setProjection(
      filament::Camera::Projection::ORTHO,
      -3,
      3,
      -3 * ratio,
      3 * ratio,
      near,
      far);
  mOrthoCamera->lookAt(at + math::float3{4, 0, 0}, at);
  mUiCamera->setProjection(
      filament::Camera::Projection::ORTHO,
      0.0,
      w / dpiScaleX,
      h / dpiScaleY,
      0.0,
      0.0,
      1.0);

  // We're in split view when there are more views than just the Main and UI
  // views.
  if (mViews.size() > 2)
  {
    uint32_t vpw = w / 2;
    uint32_t vph = h / 2;
    mMainView->setViewport({0, 0, vpw, vph});
    mDepthView->setViewport({int32_t(vpw), 0, w - vpw, vph});
    mGodView->setViewport({int32_t(vpw), int32_t(vph), w - vpw, h - vph});
    mOrthoView->setViewport({0, int32_t(vph), vpw, h - vph});

    mMainView->getCameraManipulator()->updateCameraTransform();
    mDepthView->getCameraManipulator()->updateCameraTransform();
    mGodView->getCameraManipulator()->updateCameraTransform();
    mOrthoView->getCameraManipulator()->updateCameraTransform();
  }
  else
  {
    mMainView->setViewport({0, 0, w, h});
  }
  mUiView->setViewport({0, 0, w, h});
}

} // namespace flmt
} // namespace gui
} // namespace dart
