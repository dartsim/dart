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

#include "render_context.hpp"

#include <dart/gui/experimental/viewer.hpp>

#include <filament/Engine.h>
#include <filament/Renderer.h>
#include <filament/View.h>
#include <utils/EntityManager.h>

#include <cstdint>

namespace dart::examples::filament_gui {

FilamentRenderContext createFilamentRenderContext(
    const dart::gui::experimental::RunOptions& options,
    void* nativeWindow)
{
  FilamentRenderContext context;
  context.engine = filament::Engine::create(filament::Engine::Backend::OPENGL);
  context.renderer = context.engine->createRenderer();
  if (options.headless) {
    filament::Renderer::DisplayInfo displayInfo;
    displayInfo.refreshRate = 0.0f;
    context.renderer->setDisplayInfo(displayInfo);
  }
  context.swapChain
      = options.headless
            ? context.engine->createSwapChain(
                  static_cast<std::uint32_t>(options.width),
                  static_cast<std::uint32_t>(options.height))
            : context.engine->createSwapChain(nativeWindow);
  context.view = context.engine->createView();
  context.scene = context.engine->createScene();
  context.cameraEntity = utils::EntityManager::get().create();
  context.camera = context.engine->createCamera(context.cameraEntity);
  context.view->setScene(context.scene);
  context.view->setCamera(context.camera);
  return context;
}

bool beginFilamentFrame(FilamentRenderContext& context)
{
  return context.renderer->beginFrame(context.swapChain);
}

bool shouldSkipRenderedWorkAfterFrameSkip(
    const FilamentRenderContext& context, bool headless)
{
  return !headless || !context.renderer->shouldRenderFrame();
}

void renderFilamentViews(
    FilamentRenderContext& context, filament::View* overlayView)
{
  context.renderer->render(context.view);
  if (overlayView != nullptr) {
    context.renderer->render(overlayView);
  }
}

void endFilamentFrame(FilamentRenderContext& context)
{
  context.renderer->endFrame();
}

void destroyFilamentRenderContext(FilamentRenderContext& context)
{
  context.engine->destroyCameraComponent(context.cameraEntity);
  utils::EntityManager::get().destroy(context.cameraEntity);
  context.engine->destroy(context.view);
  context.engine->destroy(context.scene);
  context.engine->destroy(context.renderer);
  context.engine->destroy(context.swapChain);
  filament::Engine::destroy(&context.engine);
  context = {};
}

} // namespace dart::examples::filament_gui
