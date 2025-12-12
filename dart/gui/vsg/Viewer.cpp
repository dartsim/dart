/*
 * Copyright (c) 2011-2025, The DART development contributors
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

#include "dart/gui/vsg/Viewer.hpp"

#include <stdexcept>
#include <utility>

namespace dart::gui::vsg {

Viewer::Viewer(
    std::shared_ptr<dart::simulation::World> world,
    ::vsg::ref_ptr<::vsg::WindowTraits> traits)
  : mWorld(std::move(world)), mScene(::vsg::Group::create()), mInitialized(false)
{
  mWorldNode = std::make_unique<WorldNode>(mWorld);
  mScene->addChild(mWorldNode->getGroup());
  if (mWorldNode) {
    // Populate the scene graph before compilation so VSG can create Vulkan
    // objects for the geometry during viewer->compile().
    mWorldNode->refresh();
  }
  initialize(std::move(traits));
}

Viewer::~Viewer() = default;

void Viewer::initialize(::vsg::ref_ptr<::vsg::WindowTraits> traits)
{
  if (mInitialized) {
    return;
  }

  ::vsg::ShaderCompiler shaderCompiler;
  if (!shaderCompiler.supported()) {
    throw std::runtime_error(
        "VulkanSceneGraph shader compilation is not supported by this VSG "
        "build. Build/install VSG with GLSLang enabled (VSG_SUPPORTS_ShaderCompiler=1) "
        "or use a precompiled-shader setup.");
  }

  if (!traits) {
    traits = ::vsg::WindowTraits::create();
    traits->width = 1280;
    traits->height = 720;
    traits->windowTitle = "DART VulkanSceneGraph Viewer";
  }

  mViewer = ::vsg::Viewer::create();
  auto window = ::vsg::Window::create(traits);
  if (!window) {
    throw std::runtime_error("Failed to create VulkanSceneGraph window.");
  }

  mViewer->addWindow(window);

  auto camera = createCamera(window);
  mViewer->addEventHandler(::vsg::CloseHandler::create(mViewer));
  mViewer->addEventHandler(::vsg::Trackball::create(camera));

  auto commandGraph = ::vsg::createCommandGraphForView(window, camera, mScene);
  mViewer->assignRecordAndSubmitTaskAndPresentation({commandGraph});

  try {
    mViewer->compile();
  } catch (const ::vsg::Exception& e) {
    throw std::runtime_error(e.message);
  }
  mInitialized = true;
}

::vsg::ref_ptr<::vsg::Camera> Viewer::createCamera(
    const ::vsg::ref_ptr<::vsg::Window>& window) const
{
  const double aspect = static_cast<double>(window->extent2D().width)
                        / static_cast<double>(window->extent2D().height);
  auto perspective = ::vsg::Perspective::create(45.0, aspect, 0.01, 1000.0);
  auto lookAt = ::vsg::LookAt::create(
      ::vsg::dvec3(0.0, -5.0, 2.5),
      ::vsg::dvec3(0.0, 0.0, 0.0),
      ::vsg::dvec3(0.0, 0.0, 1.0));
  return ::vsg::Camera::create(
      perspective, lookAt, ::vsg::ViewportState::create(window->extent2D()));
}

bool Viewer::step()
{
  if (!mViewer || !mViewer->advanceToNextFrame()) {
    return false;
  }

  if (mWorldNode) {
    mWorldNode->refresh();
  }

  mViewer->handleEvents();
  mViewer->update();
  mViewer->recordAndSubmit();
  mViewer->present();

  return true;
}

void Viewer::run()
{
  while (step()) {
    // keep stepping until the window is closed
  }
}

void Viewer::simulate(bool on)
{
  if (mWorldNode) {
    mWorldNode->simulate(on);
  }
}

bool Viewer::isSimulating() const
{
  return mWorldNode && mWorldNode->isSimulating();
}

void Viewer::setNumStepsPerCycle(std::size_t steps)
{
  if (mWorldNode) {
    mWorldNode->setNumStepsPerCycle(steps);
  }
}

::vsg::ref_ptr<::vsg::Viewer> Viewer::getViewer() const
{
  return mViewer;
}

::vsg::ref_ptr<::vsg::Group> Viewer::getScene() const
{
  return mScene;
}

} // namespace dart::gui::vsg
