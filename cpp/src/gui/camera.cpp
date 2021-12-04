/*
 * Copyright (c) 2011-2021, The DART development contributors:
 * https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "dart/gui/camera.hpp"

#include <mutex>

#include "dart/common/logging.hpp"
#include "dart/common/macro.hpp"
#include "dart/gui/osg_include.hpp"
#include "dart/gui/scene.hpp"

namespace dart::gui {

//==============================================================================
struct ImageSizeCallback : osg::Camera::DrawCallback
{
  int width;
  int height;
  mutable bool dirty{true};
  mutable std::mutex mutex;

  ImageSizeCallback(int width, int height) : width(width), height(height)
  {
    // Do nothing
  }

  void operator()(osg::RenderInfo& render_info) const override
  {
    std::lock_guard<std::mutex> lock(mutex);

    if (!dirty) {
      return;
    }

    osg::Camera* osg_camera = render_info.getCurrentCamera();
    DART_ASSERT(osg_camera);

    if (width == 0 || height == 0) {
      osg_camera->detach(osg::Camera::COLOR_BUFFER);
      return;
    }

    osg::ref_ptr<osg::Viewport> osg_viewport
        = new osg::Viewport(0, 0, width, height);
    osg_camera->setViewport(osg_viewport);
    osg_camera->setProjectionMatrixAsPerspective(
        30, width / height, 0.01, 100.0);

    dirty = false;
  }

  void set_image_size(int width, int height)
  {
    std::lock_guard<std::mutex> lock(mutex);
    if (this->width == width && this->height == height) {
      return;
    }
    this->width = width;
    this->height = height;
    dirty = true;
  }
};

//==============================================================================
struct ScreenShot : public osg::Camera::DrawCallback
{
  bool capture{true};

  void operator()(osg::RenderInfo& info) const override
  {
    if (!capture) {
      return;
    }

    const auto frame_number
        = info.getState()->getFrameStamp()->getFrameNumber();
    (void)frame_number;
  }
};

//==============================================================================
struct Camera::Implementation
{
  Scene* scene{nullptr};
  int width = 800;
  int height = 600;
  int pixel_channel = GL_RGBA;
  int pixel_data_type = GL_UNSIGNED_BYTE;
  bool image_size_dirty = true;
  osg::ref_ptr<osgViewer::Viewer> osg_viewer{nullptr};
  osg::ref_ptr<osg::Camera> osg_camera{nullptr};
  osg::ref_ptr<osg::Image> osg_image{nullptr};
  osg::ref_ptr<ImageSizeCallback> image_size_callback{nullptr};
  uint32_t texture_id{0};

  Implementation()
  {
    // Do nothing
  }
};

//==============================================================================
Camera::Camera(Scene* scene) : m_impl(std::make_unique<Implementation>())
{
  // Set the parent scene
  m_impl->scene = scene;

  // Set OSG viewer

  osg::ref_ptr<osg::DisplaySettings> ds = new osg::DisplaySettings;

  m_impl->osg_viewer = new osgViewer::Viewer();
  m_impl->osg_viewer->setThreadingModel(
      osgViewer::ViewerBase::ThreadingModel::SingleThreaded);
  m_impl->osg_viewer->setSceneData(m_impl->scene->get_mutable_osg_root_node());

  // osg::DisplaySettings* ds = osg::DisplaySettings::instance().get();

  osg::GraphicsContext::ScreenIdentifier main_screen_id;
  main_screen_id.readDISPLAY();
  main_screen_id.setUndefinedScreenDetailsToDefaultScreen();
  std::cout << "\n[DEBUG] " << main_screen_id.displayName() << std::endl;
  std::cout << "[DEBUG] " << main_screen_id.hostName << std::endl;
  std::cout << "[DEBUG] " << main_screen_id.displayNum << std::endl;
  std::cout << "[DEBUG] " << main_screen_id.screenNum << std::endl;

  osg::GraphicsContext::WindowingSystemInterface* wsi
      = osg::GraphicsContext::getWindowingSystemInterface();
  if (!wsi) {
    DART_ERROR("No WindowSystemInterface available, cannot create windows.");
    return;
  }
  // wsi->getScreenResolution(main_screen_id, m_impl->width, m_impl->height);

  DART_INFO("{} screen(s) detected", wsi->getNumScreens(main_screen_id));

  // Create graphics context
  osg::ref_ptr<osg::GraphicsContext::Traits> osg_traits
      = new osg::GraphicsContext::Traits(ds);
  osg_traits->x = 0;
  osg_traits->y = 0;
  osg_traits->width = m_impl->width;
  osg_traits->height = m_impl->height;
  osg_traits->red = 8;
  osg_traits->green = 8;
  osg_traits->blue = 8;
  osg_traits->alpha = 8;
  osg_traits->depth = 24;
  osg_traits->windowDecoration = false;
  osg_traits->pbuffer = true;
  osg_traits->doubleBuffer = true;
  std::cout << "\n[DEBUG] " << osg_traits->displayName() << std::endl;
  std::cout << "[DEBUG] " << osg_traits->hostName << std::endl;
  std::cout << "[DEBUG] " << osg_traits->displayNum << std::endl;
  std::cout << "[DEBUG] " << osg_traits->screenNum << std::endl;
  osg_traits->readDISPLAY();
  std::cout << "\n[DEBUG] " << osg_traits->displayName() << std::endl;
  std::cout << "[DEBUG] " << osg_traits->hostName << std::endl;
  std::cout << "[DEBUG] " << osg_traits->displayNum << std::endl;
  std::cout << "[DEBUG] " << osg_traits->screenNum << std::endl;
  osg_traits->setUndefinedScreenDetailsToDefaultScreen();
  std::cout << "\n[DEBUG] " << osg_traits->displayName() << std::endl;
  std::cout << "[DEBUG] " << osg_traits->hostName << std::endl;
  std::cout << "[DEBUG] " << osg_traits->displayNum << std::endl;
  std::cout << "[DEBUG] " << osg_traits->screenNum << std::endl;
  osg::ref_ptr<osg::GraphicsContext> gc
      = osg::GraphicsContext::createGraphicsContext(osg_traits);
  if (!gc || !gc->valid()) {
    DART_ERROR("GraphicsWindow has not been created successfully.");
  }

  // Set OSG camera
  m_impl->osg_camera = m_impl->osg_viewer->getCamera();
  m_impl->osg_camera->setGraphicsContext(gc);
  //  m_impl->osg_camera->setGraphicsContext(
  //        new osgViewer::GraphicsWindowEmbedded(osg_traits));
  m_impl->osg_camera->setViewport(new osg::Viewport(
      osg_traits->x, osg_traits->y, osg_traits->width, osg_traits->height));
  m_impl->osg_camera->setRenderTargetImplementation(osg::Camera::PIXEL_BUFFER);
  m_impl->osg_camera->setClearColor(osg::Vec4(1, 1, 1, 1));
  m_impl->osg_camera->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  m_impl->osg_camera->getOrCreateStateSet()->setMode(
      GL_DEPTH_TEST, osg::StateAttribute::ON);
  GLenum buffer = osg_traits->doubleBuffer ? GL_BACK : GL_FRONT;
  m_impl->osg_camera->setDrawBuffer(buffer);
  m_impl->osg_camera->setReadBuffer(buffer);

  // Set OSG image
  m_impl->osg_image = new osg::Image();
  m_impl->osg_image->allocateImage(
      m_impl->width,
      m_impl->height,
      1,
      m_impl->pixel_channel,
      m_impl->pixel_data_type);
  m_impl->osg_camera->attach(osg::Camera::COLOR_BUFFER, m_impl->osg_image);

  // Image size callback
  m_impl->image_size_callback
      = new ImageSizeCallback(m_impl->width, m_impl->height);
  m_impl->osg_camera->setPreDrawCallback(m_impl->image_size_callback);

  m_impl->osg_viewer->setCamera(m_impl->osg_camera);
  m_impl->osg_viewer->realize();
}

//==============================================================================
Camera::~Camera()
{
  // Do nothing
}

//==============================================================================
void Camera::set_image_width(int width)
{
  if (m_impl->width == width) {
    return;
  }
  m_impl->width = width;
  m_impl->image_size_dirty = true;
}

//==============================================================================
void Camera::set_image_height(int height)
{
  if (m_impl->height == height) {
    return;
  }
  m_impl->height = height;
  m_impl->image_size_dirty = true;
}

//==============================================================================
void Camera::render()
{
  if (m_impl->image_size_dirty) {
    m_impl->image_size_callback->set_image_size(m_impl->width, m_impl->height);
    m_impl->image_size_dirty = false;
  }
  m_impl->osg_viewer->frame();
}

//==============================================================================
void Camera::render_to_texture()
{
  // render();

  if (m_impl->width == 0 || m_impl->height == 0) {
    return;
  }

  if (m_impl->texture_id == 0) {
    glGenTextures(1, &m_impl->texture_id);
    DART_ASSERT(m_impl->texture_id != 0);
  }

  glBindTexture(GL_TEXTURE_2D, m_impl->texture_id);

  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

  glTexImage2D(
      GL_TEXTURE_2D,
      0,
      m_impl->osg_image->getInternalTextureFormat(),
      m_impl->osg_image->s(),
      m_impl->osg_image->t(),
      0,
      m_impl->osg_image->getPixelFormat(),
      m_impl->osg_image->getDataType(),
      m_impl->osg_image->data());

  osgDB::writeImageFile(*m_impl->osg_image, "./saved_image.bmp");
}

//==============================================================================
void Camera::copy(Image& image)
{
  image.set_from_osg_image(*m_impl->osg_image);
}

} // namespace dart::gui
