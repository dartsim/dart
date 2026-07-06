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

#include "dart/gui/osg/OffscreenViewer.hpp"

#include "dart/common/Console.hpp"
#include "dart/gui/osg/Viewer.hpp"

#include <osg/Camera>
#include <osg/GraphicsContext>
#include <osg/Viewport>
#include <osgGA/EventQueue>

#include <cmath>

namespace dart {
namespace gui {
namespace osg {

namespace {

// Local constant to avoid the non-portable M_PI macro.
constexpr double kPi = 3.14159265358979323846;

} // namespace

//==============================================================================
bool setUpOffscreenViewer(Viewer& viewer, const OffscreenSetup& setup)
{
  ::osg::ref_ptr<::osg::GraphicsContext::Traits> traits
      = new ::osg::GraphicsContext::Traits;
  traits->readDISPLAY();
  traits->setUndefinedScreenDetailsToDefaultScreen();
  traits->x = 0;
  traits->y = 0;
  traits->width = setup.width;
  traits->height = setup.height;
  traits->red = traits->green = traits->blue = 8;
  traits->alpha = 8;
  traits->depth = 24;
  traits->windowDecoration = false;
  traits->pbuffer = true;
  traits->doubleBuffer = true;

  ::osg::ref_ptr<::osg::GraphicsContext> gc
      = ::osg::GraphicsContext::createGraphicsContext(traits.get());
  if (!gc) {
    dterr << "[OffscreenViewer] off-screen GL context creation failed: no "
             "usable DISPLAY. On a headless host run under Xvfb, e.g. "
             "xvfb-run -a -s '-screen 0 1280x1024x24' <cmd>.\n";
    return false;
  }

  auto* camera = viewer.getCamera();
  camera->setGraphicsContext(gc.get());
  camera->setViewport(new ::osg::Viewport(0, 0, setup.width, setup.height));
  camera->setProjectionMatrixAsPerspective(
      setup.fovYDeg,
      static_cast<double>(setup.width) / setup.height,
      setup.nearClip,
      setup.farClip);
  const GLenum buffer = traits->doubleBuffer ? GL_BACK : GL_FRONT;
  camera->setDrawBuffer(buffer);
  camera->setReadBuffer(buffer);

  // Single-threaded so the screen-capture (a camera final-draw callback)
  // completes synchronously inside frame(); otherwise the draw thread may still
  // be writing the file as we tear down and exit, corrupting it.
  viewer.setThreadingModel(osgViewer::ViewerBase::SingleThreaded);

  // Drive stepping and the view ourselves, so the screenshot is deterministic
  // and independent of the real-time clock or any manipulator.
  viewer.setCameraManipulator(nullptr);
  viewer.simulate(false);

  viewer.realize();
  if (!viewer.isRealized()) {
    dterr << "[OffscreenViewer] viewer failed to realize off-screen.\n";
    return false;
  }
  if (auto* queue = viewer.getEventQueue()) {
    queue->windowResize(0, 0, setup.width, setup.height);
    queue->setMouseInputRange(0.0f, 0.0f, setup.width, setup.height);
  }
  return true;
}

//==============================================================================
bool captureOffscreen(
    Viewer& viewer,
    const std::string& pngPath,
    const ::osg::Vec3& eye,
    const ::osg::Vec3& center,
    const ::osg::Vec3& up,
    const OffscreenSetup& setup,
    int warmupFrames)
{
  if (!setUpOffscreenViewer(viewer, setup))
    return false;

  auto* camera = viewer.getCamera();
  // Re-pin the view each warm-up frame: realize() may have reset it, and ImGui
  // and the world node need a few frames to build their first frame before the
  // capture.
  for (int i = 0; i < warmupFrames; ++i) {
    camera->setViewMatrixAsLookAt(eye, center, up);
    viewer.frame();
  }

  viewer.captureScreen(pngPath);
  viewer.frame(); // SaveScreen writes the PNG during this frame.
  return true;
}

//==============================================================================
OffscreenCamera defaultAgentCamera(
    const ::osg::BoundingSphere& bound,
    double fovYDeg,
    double azimuthDeg,
    double elevationDeg)
{
  const double radius = bound.radius() > 0.0 ? bound.radius() : 1.0;
  const double fovYRad = fovYDeg * kPi / 180.0;
  // Fit-to-frame: place the eye far enough that the bounding sphere fills the
  // vertical field of view.
  const double distance = radius / std::sin(fovYRad * 0.5);

  const double az = azimuthDeg * kPi / 180.0;
  const double el = elevationDeg * kPi / 180.0;
  // z-up spherical offset from the scene center.
  const ::osg::Vec3 offset(
      static_cast<float>(distance * std::cos(el) * std::cos(az)),
      static_cast<float>(distance * std::cos(el) * std::sin(az)),
      static_cast<float>(distance * std::sin(el)));

  OffscreenCamera cam;
  cam.center = bound.center();
  cam.eye = bound.center() + offset;
  cam.up = ::osg::Vec3(0.0f, 0.0f, 1.0f);
  return cam;
}

} // namespace osg
} // namespace gui
} // namespace dart
