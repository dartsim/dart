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

#ifndef DART_GUI_FILAMENT_VIEWER_HPP_
#define DART_GUI_FILAMENT_VIEWER_HPP_

#include <filament/Engine.h>
#include <SDL2/SDL.h>
#include <filament/Renderer.h>
#include <filament/View.h>
#include <filament/Viewport.h>
#include <math/vec3.h>

#include "dart/gui/filament/CameraManipulator.hpp"
#include "dart/gui/filament/Config.hpp"
#include "dart/gui/filament/Path.hpp"
#include "dart/gui/filament/View.hpp"
#include "dart/gui/filament/ImGuiHelper.hpp"
#include "dart/gui/filament/WorldScene.hpp"
#include "dart/gui/filament/IBL.hpp"
#include "dart/gui/filament/Cube.hpp"

namespace dart {
namespace gui {
namespace flmt {

struct GroundPlane {
  filament::VertexBuffer* vb;
  filament::IndexBuffer* ib;
  filament::Material* mat;
  ::utils::Entity renderable;
};

class Viewer
{
public:
  using SetupCallback = std::function<void(
      filament::Engine*, filament::View*, filament::Scene*)>;
  using CleanupCallback = std::function<void(
      filament::Engine*, filament::View*, filament::Scene*)>;
  using PreRenderCallback = std::function<void(
      filament::Engine*,
      filament::View*,
      filament::Scene*,
      filament::Renderer*)>;
  using PostRenderCallback = std::function<void(
      filament::Engine*,
      filament::View*,
      filament::Scene*,
      filament::Renderer*)>;
  using ImGuiCallback = std::function<void(filament::Engine*, filament::View*)>;
  using AnimCallback
      = std::function<void(filament::Engine*, filament::View*, double now)>;

  Viewer(const Config& config,
         std::string title = "notitle",
         size_t w = 640,
         size_t h = 860,
         WorldScenePtr worldScene = nullptr);
  virtual ~Viewer();

  virtual void run();

  void mouseDown(int button, ssize_t x, ssize_t y);
  void mouseUp(ssize_t x, ssize_t y);
  void mouseMoved(ssize_t x, ssize_t y);
  void mouseWheel(ssize_t x);
  void resize();

  // Returns the path to the Filament root for loading assets. This is
  // determined from the
  // executable folder, which allows users to launch samples from any folder.
  static const utils::Path& getRootPath()
  {
    static const utils::Path root
        = utils::Path::getCurrentExecutable().getParent();
    return root;
  }

protected:
  WorldScenePtr mWorldScene;

private:
  ::utils::Entity mLight;
  GroundPlane mPlane;

  void setupWorldScene();
  void loadIBL(const Config& config);
  void configureCamerasForWindow();
  void fixupMouseCoordinatesForHdpi(ssize_t& x, ssize_t& y) const;

  filament::Engine* mEngine = nullptr;
  filament::Scene* mScene = nullptr;
  std::unique_ptr<IBL> mIBL;
  bool mClosed = false;
  uint64_t mTime = 0;

  filament::Material const* mDefaultMaterial = nullptr;
  filament::Material const* mTransparentMaterial = nullptr;
  filament::Material const* mDepthMaterial = nullptr;
  filament::MaterialInstance* mDepthMI = nullptr;
  std::unique_ptr<filagui::ImGuiHelper> mImGuiHelper;
  AnimCallback mAnimation;

  SetupCallback mSetupCallback;
  CleanupCallback mCleanupCallback;
  PreRenderCallback mPreRenderCallback;
  PostRenderCallback mPostRenderCallback;
  ImGuiCallback mImGuiCallback;

  std::unique_ptr<Cube> mCameraCube;
  std::unique_ptr<Cube> mLightmapCube;

  SDL_Window* mWindow = nullptr;
  filament::Renderer* mRenderer = nullptr;

  CameraManipulator mMainCameraMan;
  CameraManipulator mOrthoCameraMan;
  CameraManipulator mDebugCameraMan;
  filament::SwapChain* mSwapChain = nullptr;

  filament::Camera* mCameras[4] = {nullptr};
  filament::Camera* mUiCamera;
  filament::Camera* mMainCamera;
  filament::Camera* mDebugCamera;
  filament::Camera* mOrthoCamera;

  std::vector<std::unique_ptr<CView>> mViews;
  CView* mMainView;
  CView* mUiView;
  CView* mDepthView;
  GodView* mGodView;
  CView* mOrthoView;

  size_t mWidth = 0;
  size_t mHeight = 0;
  ssize_t mLastX = 0;
  ssize_t mLastY = 0;
  CView* mEventTarget = nullptr;
};

} // namespace flmt
} // namespace gui
} // namespace dart

#endif // DART_GUI_FILAMENT_VIEWER_HPP_
