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

#include "dart/gui/vsg/ImGuiViewer.hpp"

#ifdef DART_HAS_VSGIMGUI

  #include "dart/gui/vsg/Conversions.hpp"
  #include "dart/gui/vsg/DebugDraw.hpp"

  #include <vsg/all.h>

namespace dart::gui::vsg {

class ImGuiViewer::ImGuiCommand
  : public ::vsg::Inherit<::vsg::Command, ImGuiCommand>
{
public:
  ImGuiCommand(ImGuiCallback* callback) : m_callback(callback) {}

  void record([[maybe_unused]] ::vsg::CommandBuffer& cb) const override
  {
    if (m_callback && *m_callback) {
      (*m_callback)();
    }
  }

private:
  ImGuiCallback* m_callback;
};

ImGuiViewer::ImGuiViewer(int width, int height, const std::string& title)
  : m_width(width), m_height(height), m_title(title)
{
  m_root = ::vsg::Group::create();
  m_sceneRoot = ::vsg::Group::create();
  m_root->addChild(m_sceneRoot);
  setupViewer();
}

ImGuiViewer::~ImGuiViewer() = default;

void ImGuiViewer::setupViewer()
{
  auto windowTraits = ::vsg::WindowTraits::create();
  windowTraits->windowTitle = m_title;
  windowTraits->width = m_width;
  windowTraits->height = m_height;
  windowTraits->debugLayer = false;
  windowTraits->apiDumpLayer = false;
  windowTraits->swapchainPreferences.imageUsage
      = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT;

  m_viewer = ::vsg::Viewer::create();
  m_window = ::vsg::Window::create(windowTraits);
  if (!m_window) {
    throw std::runtime_error("Failed to create VSG window");
  }
  m_viewer->addWindow(m_window);

  m_viewer->addEventHandler(vsgImGui::SendEventsToImGui::create());
  m_viewer->addEventHandler(::vsg::CloseHandler::create(m_viewer));

  setupCamera();
}

void ImGuiViewer::setupCamera()
{
  VkExtent2D extent = m_window->extent2D();
  double aspectRatio
      = static_cast<double>(extent.width) / static_cast<double>(extent.height);

  m_lookAt = ::vsg::LookAt::create(
      ::vsg::dvec3(5.0, 5.0, 5.0),
      ::vsg::dvec3(0.0, 0.0, 0.0),
      ::vsg::dvec3(0.0, 0.0, 1.0));

  auto perspective
      = ::vsg::Perspective::create(60.0, aspectRatio, 0.01, 1000.0);
  auto viewportState = ::vsg::ViewportState::create(extent);
  m_camera = ::vsg::Camera::create(perspective, m_lookAt, viewportState);

  auto trackball = ::vsg::Trackball::create(m_camera);
  m_viewer->addEventHandler(trackball);
}

void ImGuiViewer::compile()
{
  if (!m_needsCompile) {
    return;
  }

  auto commandGraph = ::vsg::CommandGraph::create(m_window);
  auto renderGraph = ::vsg::RenderGraph::create(m_window);
  commandGraph->addChild(renderGraph);

  VkClearColorValue clearColor{
      {static_cast<float>(m_backgroundColor.x()),
       static_cast<float>(m_backgroundColor.y()),
       static_cast<float>(m_backgroundColor.z()),
       static_cast<float>(m_backgroundColor.w())}};
  renderGraph->setClearValues(clearColor, VkClearDepthStencilValue{0.0f, 0});

  auto view = ::vsg::View::create(m_camera, m_root);
  view->addChild(::vsg::createHeadlight());
  renderGraph->addChild(view);

  m_imguiCommand = ImGuiCommand::create(&m_imguiCallback);
  auto renderImGui = vsgImGui::RenderImGui::create(m_window, m_imguiCommand);
  renderGraph->addChild(renderImGui);

  m_viewer->assignRecordAndSubmitTaskAndPresentation({commandGraph});
  m_viewer->compile();

  m_needsCompile = false;
}

void ImGuiViewer::setScene(::vsg::ref_ptr<::vsg::Node> scene)
{
  m_sceneRoot->children.clear();
  if (scene) {
    m_sceneRoot->addChild(scene);
  }
  m_needsCompile = true;
}

::vsg::ref_ptr<::vsg::Group> ImGuiViewer::getRoot() const
{
  return m_sceneRoot;
}

void ImGuiViewer::addNode(::vsg::ref_ptr<::vsg::Node> node)
{
  if (node) {
    m_sceneRoot->addChild(node);
    m_needsCompile = true;
  }
}

void ImGuiViewer::removeNode(::vsg::ref_ptr<::vsg::Node> node)
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

void ImGuiViewer::clear()
{
  m_sceneRoot->children.clear();
  m_needsCompile = true;
}

void ImGuiViewer::setImGuiCallback(ImGuiCallback callback)
{
  m_imguiCallback = std::move(callback);
}

void ImGuiViewer::lookAt(
    const Eigen::Vector3d& eye,
    const Eigen::Vector3d& center,
    const Eigen::Vector3d& up)
{
  m_lookAt->eye = toVsg(eye);
  m_lookAt->center = toVsg(center);
  m_lookAt->up = toVsg(up);
}

void ImGuiViewer::resetCamera()
{
  lookAt(
      Eigen::Vector3d(5.0, 5.0, 5.0),
      Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector3d::UnitZ());
}

void ImGuiViewer::setBackgroundColor(const Eigen::Vector4d& color)
{
  m_backgroundColor = color;
  m_needsCompile = true;
}

bool ImGuiViewer::frame()
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

void ImGuiViewer::run()
{
  compile();
  while (frame()) {
  }
}

bool ImGuiViewer::shouldClose() const
{
  return !m_viewer->active();
}

void ImGuiViewer::addGrid(double size, double spacing)
{
  addNode(createGrid(size, spacing));
}

void ImGuiViewer::addAxes(double length)
{
  addNode(createAxes(length));
}

::vsg::ref_ptr<::vsg::Window> ImGuiViewer::getWindow() const
{
  return m_window;
}

::vsg::ref_ptr<::vsg::Viewer> ImGuiViewer::getViewer() const
{
  return m_viewer;
}

::vsg::ref_ptr<::vsg::Camera> ImGuiViewer::getCamera() const
{
  return m_camera;
}

bool ImGuiViewer::computePickingRay(
    double screenX,
    double screenY,
    Eigen::Vector3d& rayOrigin,
    Eigen::Vector3d& rayDirection) const
{
  if (!m_camera || !m_window) {
    return false;
  }

  VkExtent2D extent = m_window->extent2D();
  if (extent.width == 0 || extent.height == 0) {
    return false;
  }

  double ndcX = (2.0 * screenX / extent.width) - 1.0;
  double ndcY = 1.0 - (2.0 * screenY / extent.height);

  auto projection = m_camera->projectionMatrix;
  auto view = m_camera->viewMatrix;
  auto projMat = projection->transform();
  auto viewMat = view->transform();

  ::vsg::dmat4 invView = ::vsg::inverse(viewMat);
  ::vsg::dmat4 invProj = ::vsg::inverse(projMat);

  ::vsg::dvec4 nearClip(ndcX, ndcY, 0.0, 1.0);
  ::vsg::dvec4 farClip(ndcX, ndcY, 1.0, 1.0);

  ::vsg::dvec4 nearView = invProj * nearClip;
  ::vsg::dvec4 farView = invProj * farClip;
  nearView /= nearView.w;
  farView /= farView.w;

  ::vsg::dvec4 nearWorld = invView * nearView;
  ::vsg::dvec4 farWorld = invView * farView;

  rayOrigin = Eigen::Vector3d(nearWorld.x, nearWorld.y, nearWorld.z);
  Eigen::Vector3d farPoint(farWorld.x, farWorld.y, farWorld.z);
  rayDirection = (farPoint - rayOrigin).normalized();

  return true;
}

void ImGuiViewer::getWindowSize(int& width, int& height) const
{
  if (m_window) {
    VkExtent2D extent = m_window->extent2D();
    width = static_cast<int>(extent.width);
    height = static_cast<int>(extent.height);
  } else {
    width = m_width;
    height = m_height;
  }
}

} // namespace dart::gui::vsg

#endif // DART_HAS_VSGIMGUI
