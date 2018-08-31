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

#include "dart/gui/filament/Viewer.hpp"

#include "stb_image.h"

#include <fstream>
#include <string>

#include <filament/Camera.h>
#include <filament/Engine.h>
#include <filament/IndirectLight.h>
#include <filament/LightManager.h>
#include <filament/Material.h>
#include <filament/Material.h>
#include <filament/MaterialInstance.h>
#include <filament/MaterialInstance.h>
#include <filament/RenderableManager.h>
#include <filament/Renderer.h>
#include <filament/Scene.h>
#include <filament/Skybox.h>
#include <filament/Skybox.h>
#include <filament/Texture.h>
#include <filament/TransformManager.h>
#include <filament/View.h>
#include <math/TVecHelpers.h>
#include <math/norm.h>
#include <math/vec3.h>
#include <math/vec4.h>
#include <utils/EntityManager.h>

#include "dart/gui/filament/App.hpp"
#include "dart/gui/filament/Cube.hpp"
#include "dart/gui/filament/NativeWindowHelper.h"
#include "dart/gui/filament/Types.hpp"

namespace dart {
namespace gui {
namespace flmt {

namespace {

static constexpr bool ENABLE_SHADOWS = true;
static constexpr uint8_t GROUND_SHADOW_PACKAGE[] = {
#include "generated/material/groundShadow.inc"
};

static GroundPlane createGroundPlane(filament::Engine* engine)
{
  filament::Material* shadowMaterial
      = filament::Material::Builder()
            .package(
                (void*)GROUND_SHADOW_PACKAGE, sizeof(GROUND_SHADOW_PACKAGE))
            .build(*engine);

  const static uint32_t indices[]{0, 1, 2, 2, 3, 0};
  const static ::math::float3 vertices[]{
      {-10, 0, -10}, {-10, 0, 10}, {10, 0, 10}, {10, 0, -10},
  };
  ::math::short4 tbn = ::math::packSnorm16(
      normalize(
          positive(
              ::math::mat3f{::math::float3{1.0f, 0.0f, 0.0f},
                            ::math::float3{0.0f, 0.0f, 1.0f},
                            ::math::float3{0.0f, 1.0f, 0.0f}}
                  .toQuaternion()))
          .xyzw);
  const static ::math::short4 normals[]{tbn, tbn, tbn, tbn};
  filament::VertexBuffer* vertexBuffer
      = filament::VertexBuffer::Builder()
            .vertexCount(4)
            .bufferCount(2)
            .attribute(
                filament::VertexAttribute::POSITION,
                0,
                filament::VertexBuffer::AttributeType::FLOAT3)
            .attribute(
                filament::VertexAttribute::TANGENTS,
                1,
                filament::VertexBuffer::AttributeType::SHORT4)
            .normalized(filament::VertexAttribute::TANGENTS)
            .build(*engine);
  vertexBuffer->setBufferAt(
      *engine,
      0,
      filament::VertexBuffer::BufferDescriptor(
          vertices, vertexBuffer->getVertexCount() * sizeof(vertices[0])));
  vertexBuffer->setBufferAt(
      *engine,
      1,
      filament::VertexBuffer::BufferDescriptor(
          normals, vertexBuffer->getVertexCount() * sizeof(normals[0])));
  filament::IndexBuffer* indexBuffer
      = filament::IndexBuffer::Builder().indexCount(6).build(*engine);
  indexBuffer->setBuffer(
      *engine,
      filament::IndexBuffer::BufferDescriptor(
          indices, indexBuffer->getIndexCount() * sizeof(uint32_t)));

  auto& em = ::utils::EntityManager::get();
  ::utils::Entity renderable = em.create();
  filament::RenderableManager::Builder(1)
      .boundingBox({{0, 0, 0}, {10, 1e-4f, 10}})
      .material(0, shadowMaterial->getDefaultInstance())
      .geometry(
          0,
          filament::RenderableManager::PrimitiveType::TRIANGLES,
          vertexBuffer,
          indexBuffer,
          0,
          6)
      .culling(false)
      .receiveShadows(ENABLE_SHADOWS)
      .castShadows(false)
      .build(*engine, renderable);

  auto& tcm = engine->getTransformManager();
  tcm.setTransform(
      tcm.getInstance(renderable),
      ::math::mat4f::translate(::math::float3{0, -1, -4}));
  return {
      .vb = vertexBuffer,
      .ib = indexBuffer,
      .mat = shadowMaterial,
      .renderable = renderable,
  };
}
}

static constexpr uint8_t AI_DEFAULT_MAT_PACKAGE[] = {
#include "generated/material/aiDefaultMat.inc"
};

static constexpr uint8_t TRANSPARENT_COLOR_PACKAGE[] = {
#include "generated/material/transparentColor.inc"
};

static constexpr uint8_t DEPTH_VISUALIZER_PACKAGE[] = {
#include "generated/material/depthVisualizer.inc"
};

Viewer::Viewer(
    const Config& config,
    std::string title,
    size_t w,
    size_t h,
    WorldScenePtr worldScene)
  : mWorldScene(std::move(worldScene))
{
  if (SDL_Init(SDL_INIT_EVENTS) != 0)
    assert(false && "SDL_Init Failure");

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
  mEngine = filament::Engine::create(config.backend);

// HACK: We don't use SDL's 2D rendering functionality, but by invoking it we
// cause
// SDL to create a Metal backing layer, which allows us to run Vulkan apps via
// MoltenVK.
#if defined(FILAMENT_DRIVER_SUPPORTS_VULKAN) && defined(__APPLE__)
  constexpr int METAL_DRIVER = 2;
  SDL_CreateRenderer(mWindow, METAL_DRIVER, SDL_RENDERER_ACCELERATED);
#endif

  void* nativeWindow = ::getNativeWindow(mWindow);
  mSwapChain = mEngine->createSwapChain(nativeWindow);
  mRenderer = mEngine->createRenderer();

  // create cameras
  mCameras[0] = mMainCamera = mEngine->createCamera();
  mCameras[1] = mDebugCamera = mEngine->createCamera();
  mCameras[2] = mOrthoCamera = mEngine->createCamera();
  mCameras[3] = mUiCamera = mEngine->createCamera();

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
  ::math::double3 at(0, 0, -4);
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

  mMainCameraMan.lookAt(at + ::math::double3{0, 0, 4}, at);
  mDebugCameraMan.lookAt(at + ::math::double3{0, 0, 4}, at);
  mOrthoCameraMan.lookAt(at + ::math::double3{0, 0, 4}, at);

  mDepthMaterial = filament::Material::Builder()
                       .package(
                           (void*)DEPTH_VISUALIZER_PACKAGE,
                           sizeof(DEPTH_VISUALIZER_PACKAGE))
                       .build(*mEngine);

  mDepthMI = mDepthMaterial->createInstance();

  mDefaultMaterial
      = filament::Material::Builder()
            .package(
                (void*)AI_DEFAULT_MAT_PACKAGE, sizeof(AI_DEFAULT_MAT_PACKAGE))
            .build(*mEngine);

  mTransparentMaterial = filament::Material::Builder()
                             .package(
                                 (void*)TRANSPARENT_COLOR_PACKAGE,
                                 sizeof(TRANSPARENT_COLOR_PACKAGE))
                             .build(*mEngine);

  mCameraCube.reset(new Cube(*mEngine, mTransparentMaterial, {1, 0, 0}));
  // we can't cull the light-frustum because it's not applied a rigid transform
  // and currently, filament assumes that for culling
  mLightmapCube.reset(
      new Cube(*mEngine, mTransparentMaterial, {0, 1, 0}, false));
  mScene = mEngine->createScene();

  setupWorldScene();

  auto& em = ::utils::EntityManager::get();

  // Add light sources into the scene.
  mLight = em.create();
  filament::LightManager::Builder(filament::LightManager::Type::SUN)
      .color(
          filament::Color::toLinear<filament::ACCURATE>(
              filament::sRGBColor(0.98f, 0.92f, 0.89f)))
      .intensity(110000)
      .direction({0.7, -1, -0.8})
      .sunAngularRadius(1.9f)
      .castShadows(true)
      .build(*mEngine, mLight);
  mScene->addEntity(mLight);

  mPlane = createGroundPlane(mEngine);
  mScene->addEntity(mPlane.renderable);

  mMainView->getView()->setVisibleLayers(0x4, 0x4);

  mUiView->getView()->setClearTargets(false, false, false);
  mUiView->getView()->setRenderTarget(
      filament::View::TargetBufferFlags::DEPTH_AND_STENCIL);
  mUiView->getView()->setPostProcessingEnabled(false);
  mUiView->getView()->setShadowsEnabled(false);

  if (config.splitView)
  {
    auto& rcm = mEngine->getRenderableManager();

    rcm.setLayerMask(
        rcm.getInstance(mCameraCube->getSolidRenderable()), 0x3, 0x2);
    rcm.setLayerMask(
        rcm.getInstance(mCameraCube->getWireFrameRenderable()), 0x3, 0x2);
    mCameraCube->mapFrustum(*mEngine, mMainCameraMan.getCamera());

    rcm.setLayerMask(
        rcm.getInstance(mLightmapCube->getSolidRenderable()), 0x3, 0x2);
    rcm.setLayerMask(
        rcm.getInstance(mLightmapCube->getWireFrameRenderable()), 0x3, 0x2);

    // Create the camera mesh
    mMainCameraMan.setCameraChangedCallback([
      this,
      cameraCube = mCameraCube.get(),
      lightmapCube = mLightmapCube.get(),
      engine = mEngine
    ](filament::Camera const* camera) {
      cameraCube->mapFrustum(*engine, camera);
      lightmapCube->mapFrustum(
          *engine, mMainView->getView()->getDirectionalLightCamera());
    });

    mScene->addEntity(mCameraCube->getWireFrameRenderable());
    mScene->addEntity(mCameraCube->getSolidRenderable());

    mScene->addEntity(mLightmapCube->getWireFrameRenderable());
    mScene->addEntity(mLightmapCube->getSolidRenderable());

    mGodView->getView()->setVisibleLayers(0x6, 0x6);
    mOrthoView->getView()->setVisibleLayers(0x6, 0x6);

    // only preserve the color buffer for additional views; depth and stencil
    // can be discarded.
    mDepthView->getView()->setRenderTarget(
        filament::View::TargetBufferFlags::DEPTH_AND_STENCIL);
    mGodView->getView()->setRenderTarget(
        filament::View::TargetBufferFlags::DEPTH_AND_STENCIL);
    mOrthoView->getView()->setRenderTarget(
        filament::View::TargetBufferFlags::DEPTH_AND_STENCIL);

    mDepthView->getView()->setShadowsEnabled(false);
    mGodView->getView()->setShadowsEnabled(false);
    mOrthoView->getView()->setShadowsEnabled(false);
  }

  loadIBL(config);
  if (mIBL != nullptr)
  {
    mIBL->getSkybox()->setLayerMask(0x7, 0x4);
    mScene->setSkybox(mIBL->getSkybox());
    mScene->setIndirectLight(mIBL->getIndirectLight());
  }

  for (auto& view : mViews)
  {
    if (view.get() != mUiView)
    {
      view->getView()->setScene(mScene);
      view->getView()->setClearColor({0.5f,0.75f,1.0f,1.0f});
    }
  }

  if (mSetupCallback)
    mSetupCallback(mEngine, mMainView->getView(), mScene);

  if (mImGuiCallback)
  {
    mImGuiHelper = std::make_unique<filagui::ImGuiHelper>(
        mEngine,
        mUiView->getView(),
        getRootPath() + "assets/fonts/Roboto-Medium.ttf");
    ImGuiIO& io = ImGui::GetIO();
#ifdef WIN32
    SDL_SysWMinfo wmInfo;
    SDL_VERSION(&wmInfo.version);
    SDL_GetWindowWMInfo(getSDLWindow(), &wmInfo);
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
}

Viewer::~Viewer()
{
  filament::Fence::waitAndDestroy(mEngine->createFence());

  destroyWorldScene();

  if (mImGuiHelper)
  {
    mImGuiHelper.reset();
  }

  if (mCleanupCallback)
    mCleanupCallback(mEngine, mMainView->getView(), mScene);

  mCameraCube.reset();
  mLightmapCube.reset();

  mViews.clear();
  for (auto& camera : mCameras)
    mEngine->destroy(camera);
  mEngine->destroy(mRenderer);
  mEngine->destroy(mSwapChain);

  mIBL.reset();
  mEngine->destroy(mDepthMI);
  mEngine->destroy(mDepthMaterial);
  mEngine->destroy(mDefaultMaterial);
  mEngine->destroy(mTransparentMaterial);
  mEngine->destroy(mScene);

  filament::Engine::destroy(&mEngine);
  mEngine = nullptr;

  SDL_DestroyWindow(mWindow);

  SDL_Quit();
}

void Viewer::run()
{
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
      const double now
          = (double)SDL_GetPerformanceCounter() / SDL_GetPerformanceFrequency();
      mAnimation(mEngine, mMainView->getView(), now);
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
    for (int i = 0; i < nevents; ++i)
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
            mouseWheel(event.wheel.y);
          break;
        case SDL_MOUSEBUTTONDOWN:
          if (!io || !io->WantCaptureMouse)
            mouseDown(event.button.button, event.button.x, event.button.y);
          break;
        case SDL_MOUSEBUTTONUP:
          if (!io || !io->WantCaptureMouse)
            mouseUp(event.button.x, event.button.y);
          break;
        case SDL_MOUSEMOTION:
          if (!io || !io->WantCaptureMouse)
            mouseMoved(event.motion.x, event.motion.y);
          break;
        case SDL_WINDOWEVENT:
          switch (event.window.event)
          {
            case SDL_WINDOWEVENT_RESIZED:
              resize();
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
    // frame. We should always let ImGui generate a command list; if it skips a
    // frame it'll destroy its widgets.
    if (mImGuiHelper)
    {
      // Inform ImGui of the current window size in case it was resized.
      int windowWidth, windowHeight;
      int displayWidth, displayHeight;
      SDL_GetWindowSize(mWindow, &windowWidth, &windowHeight);
      SDL_GL_GetDrawableSize(mWindow, &displayWidth, &displayHeight);
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
      if ((SDL_GetWindowFlags(mWindow) & SDL_WINDOW_INPUT_FOCUS) != 0)
      {
        io.MousePos = ImVec2((float)mx, (float)my);
      }

      // Populate the UI Scene.
      static Uint64 frequency = SDL_GetPerformanceFrequency();
      Uint64 now = SDL_GetPerformanceCounter();
      float timeStep = mTime > 0 ? (float)((double)(now - mTime) / frequency)
                                 : (float)(1.0f / 60.0f);
      mTime = now;
      mImGuiHelper->render(timeStep, mImGuiCallback);
    }

    mMainCameraMan.updateCameraTransform();

    // TODO: we need better timing or use SDL_GL_SetSwapInterval
    SDL_Delay(16);

    if (mWorldScene)
      mWorldScene->refresh();

    if (mPreRenderCallback)
    {
      for (auto const& view : mViews)
      {
        if (view.get() != mUiView)
        {
          mPreRenderCallback(mEngine, view->getView(), mScene, mRenderer);
        }
      }
    }

    if (mRenderer->beginFrame(mSwapChain))
    {
      for (auto const& view : mViews)
      {
        mRenderer->render(view->getView());
      }
      mRenderer->endFrame();
    }

    if (mPostRenderCallback)
    {
      for (auto const& view : mViews)
      {
        if (view.get() != mUiView)
        {
          mPostRenderCallback(mEngine, view->getView(), mScene, mRenderer);
        }
      }
    }
  }
}

void Viewer::mouseDown(int button, ssize_t x, ssize_t y)
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

void Viewer::mouseUp(ssize_t x, ssize_t y)
{
  fixupMouseCoordinatesForHdpi(x, y);
  if (mEventTarget)
  {
    y = mHeight - y;
    mEventTarget->mouseUp(x, y);
    mEventTarget = nullptr;
  }
}

void Viewer::mouseMoved(ssize_t x, ssize_t y)
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

void Viewer::mouseWheel(ssize_t x)
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

void Viewer::resize()
{
  mEngine->destroy(mSwapChain);
  mSwapChain = mEngine->createSwapChain(::getNativeWindow(mWindow));
  configureCamerasForWindow();
}

void Viewer::setupWorldScene()
{
  assert(mEngine);
  if (!mWorldScene)
    return;

  mWorldScene->setScene(mEngine, mScene);
}

void Viewer::destroyWorldScene()
{
  // TODO(JS)
  mWorldScene->setScene(nullptr, nullptr);
}

void Viewer::loadIBL(const Config& config)
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

void Viewer::configureCamerasForWindow()
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

  const ::math::float3 at(0, 0, -4);
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
  mOrthoCamera->lookAt(at + ::math::float3{4, 0, 0}, at);
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

void Viewer::fixupMouseCoordinatesForHdpi(ssize_t& x, ssize_t& y) const
{
  int dw, dh, ww, wh;
  SDL_GL_GetDrawableSize(mWindow, &dw, &dh);
  SDL_GetWindowSize(mWindow, &ww, &wh);
  x = x * dw / ww;
  y = y * dh / wh;
}

} // namespace flmt
} // namespace gui
} // namespace dart
