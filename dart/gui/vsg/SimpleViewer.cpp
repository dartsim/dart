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

#include "dart/gui/vsg/SimpleViewer.hpp"

#include "dart/gui/vsg/Conversions.hpp"
#include "dart/gui/vsg/DebugDraw.hpp"

#include <vsg/app/CloseHandler.h>
#include <vsg/app/CommandGraph.h>
#include <vsg/app/RenderGraph.h>
#include <vsg/app/Trackball.h>
#include <vsg/app/View.h>

namespace dart::gui::vsg {

SimpleViewer::SimpleViewer(int width, int height, const std::string& title)
  : m_width(width), m_height(height), m_title(title)
{
  m_root = ::vsg::Group::create();
  m_sceneRoot = ::vsg::Group::create();
  m_root->addChild(m_sceneRoot);
  setupViewer();
}

SimpleViewer::~SimpleViewer() = default;

void SimpleViewer::setupViewer()
{
  auto windowTraits = ::vsg::WindowTraits::create();
  windowTraits->windowTitle = m_title;
  windowTraits->width = m_width;
  windowTraits->height = m_height;
  windowTraits->debugLayer = false;
  windowTraits->apiDumpLayer = false;

  m_viewer = ::vsg::Viewer::create();
  m_window = ::vsg::Window::create(windowTraits);
  if (!m_window) {
    throw std::runtime_error("Failed to create VSG window");
  }
  m_viewer->addWindow(m_window);

  m_viewer->addEventHandler(::vsg::CloseHandler::create(m_viewer));

  setupCamera();
}

void SimpleViewer::setupCamera()
{
  auto vp = m_window->extent2D();
  double aspectRatio
      = static_cast<double>(vp.width) / static_cast<double>(vp.height);

  m_lookAt = ::vsg::LookAt::create(
      ::vsg::dvec3(5.0, 5.0, 5.0),
      ::vsg::dvec3(0.0, 0.0, 0.0),
      ::vsg::dvec3(0.0, 0.0, 1.0));

  auto perspective
      = ::vsg::Perspective::create(60.0, aspectRatio, 0.01, 1000.0);

  auto viewportState = ::vsg::ViewportState::create(vp);
  m_camera = ::vsg::Camera::create(perspective, m_lookAt, viewportState);

  auto trackball = ::vsg::Trackball::create(m_camera);
  m_viewer->addEventHandler(trackball);
}

void SimpleViewer::compile()
{
  if (!m_needsCompile) {
    return;
  }

  auto commandGraph = ::vsg::CommandGraph::create(m_window);

  auto renderGraph = ::vsg::RenderGraph::create(m_window);
  renderGraph->setClearValues(
      VkClearColorValue{
          {static_cast<float>(m_backgroundColor.x()),
           static_cast<float>(m_backgroundColor.y()),
           static_cast<float>(m_backgroundColor.z()),
           static_cast<float>(m_backgroundColor.w())}},
      VkClearDepthStencilValue{1.0f, 0});

  auto view = ::vsg::View::create(m_camera, m_root);
  renderGraph->addChild(view);
  commandGraph->addChild(renderGraph);

  m_viewer->assignRecordAndSubmitTaskAndPresentation({commandGraph});
  m_viewer->compile();

  m_needsCompile = false;
}

void SimpleViewer::setScene(::vsg::ref_ptr<::vsg::Node> scene)
{
  m_sceneRoot->children.clear();
  if (scene) {
    m_sceneRoot->addChild(scene);
  }
  m_needsCompile = true;
}

::vsg::ref_ptr<::vsg::Group> SimpleViewer::getRoot() const
{
  return m_sceneRoot;
}

void SimpleViewer::addNode(::vsg::ref_ptr<::vsg::Node> node)
{
  if (node) {
    m_sceneRoot->addChild(node);
    m_needsCompile = true;
  }
}

void SimpleViewer::removeNode(::vsg::ref_ptr<::vsg::Node> node)
{
  if (!node) {
    return;
  }
  auto& children = m_sceneRoot->children;
  auto it = std::find(children.begin(), children.end(), node);
  if (it != children.end()) {
    children.erase(it);
    m_needsCompile = true;
  }
}

void SimpleViewer::clear()
{
  m_sceneRoot->children.clear();
  m_needsCompile = true;
}

void SimpleViewer::lookAt(
    const Eigen::Vector3d& eye,
    const Eigen::Vector3d& center,
    const Eigen::Vector3d& up)
{
  m_lookAt->eye = toVsg(eye);
  m_lookAt->center = toVsg(center);
  m_lookAt->up = toVsg(up);
}

void SimpleViewer::resetCamera()
{
  lookAt(
      Eigen::Vector3d(5.0, 5.0, 5.0),
      Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector3d::UnitZ());
}

void SimpleViewer::setBackgroundColor(const Eigen::Vector4d& color)
{
  m_backgroundColor = color;
  m_needsCompile = true;
}

bool SimpleViewer::frame()
{
  compile();

  if (m_viewer->advanceToNextFrame()) {
    m_viewer->handleEvents();
    m_viewer->update();
    m_viewer->recordAndSubmit();
    m_viewer->present();
    return true;
  }
  return false;
}

void SimpleViewer::run()
{
  compile();
  while (frame()) {
  }
}

bool SimpleViewer::shouldClose() const
{
  return !m_viewer->active();
}

void SimpleViewer::addGrid(double size, double spacing)
{
  addNode(createGrid(size, spacing));
}

void SimpleViewer::addAxes(double length)
{
  addNode(createAxes(length));
}

} // namespace dart::gui::vsg
