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

// Off-screen GLX-pbuffer capture smoke for the dart-gui-osg helper
// (dart/gui/osg/OffscreenViewer). Dependency-free (no GoogleTest / Pillow): it
// renders a trivial one-body scene headless via setUpOffscreenViewer and via
// the captureOffscreen sugar, reads each PNG back with osgDB, and asserts a
// valid image with non-blank content (luminance variance above a small
// epsilon). Opt-in (DART_ENABLE_GUI_OSG_SMOKE_TESTS); returns the ctest skip
// code (77) when no DISPLAY is available.

#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/BoxShape.hpp"
#include "dart/dynamics/FreeJoint.hpp"
#include "dart/dynamics/ShapeNode.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/gui/osg/ImGuiViewer.hpp"
#include "dart/gui/osg/OffscreenViewer.hpp"
#include "dart/gui/osg/Viewer.hpp"
#include "dart/gui/osg/WorldNode.hpp"
#include "dart/simulation/World.hpp"

#include <osg/BoundingSphere>
#include <osg/Image>
#include <osgDB/ReadFile>

#include <filesystem>
#include <iostream>
#include <string>

#include <cstdlib>

namespace {

// Skip code understood by ctest's SKIP_RETURN_CODE property.
constexpr int kSkip = 77;

// Builds a trivial one-body world: a single colored box at the origin. Enough
// geometry (plus the viewer's default lighting) that any correct render is
// clearly non-blank.
dart::simulation::WorldPtr buildBoxWorld()
{
  auto world = dart::simulation::World::create();
  world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));

  auto skeleton = dart::dynamics::Skeleton::create("box");
  auto pair = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>(
      nullptr);
  auto* body = pair.second;
  auto shape = std::make_shared<dart::dynamics::BoxShape>(
      Eigen::Vector3d(1.0, 1.0, 1.0));
  auto shapeNode = body->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(shape);
  shapeNode->getVisualAspect()->setColor(Eigen::Vector3d(0.8, 0.2, 0.2));

  world->addSkeleton(skeleton);
  return world;
}

// Reads a PNG back and asserts it is a valid image of the expected size with
// non-blank content (luminance variance above epsilon). A blank frame has
// variance 0.
bool checkNonBlank(
    const std::string& label,
    const std::string& path,
    int expectedWidth,
    int expectedHeight)
{
  ::osg::ref_ptr<::osg::Image> image = osgDB::readImageFile(path);
  if (!image) {
    std::cerr << "[offscreen_capture_smoke] " << label
              << ": failed to read image back from " << path << "\n";
    return false;
  }
  if (image->s() != expectedWidth || image->t() != expectedHeight) {
    std::cerr << "[offscreen_capture_smoke] " << label << ": unexpected size "
              << image->s() << "x" << image->t() << " (expected "
              << expectedWidth << "x" << expectedHeight << ")\n";
    return false;
  }

  double sum = 0.0;
  double sumSq = 0.0;
  const long count = static_cast<long>(image->s()) * image->t();
  for (int t = 0; t < image->t(); ++t) {
    for (int s = 0; s < image->s(); ++s) {
      const ::osg::Vec4 c = image->getColor(s, t);
      const double lum = 0.299 * c.r() + 0.587 * c.g() + 0.114 * c.b();
      sum += lum;
      sumSq += lum * lum;
    }
  }
  const double mean = sum / count;
  const double variance = sumSq / count - mean * mean;

  constexpr double kMinVariance = 1e-4;
  std::cout << "[offscreen_capture_smoke] " << label << ": " << image->s()
            << "x" << image->t() << " lum mean=" << mean << " var=" << variance
            << " (min " << kMinVariance << ")\n";
  if (variance <= kMinVariance) {
    std::cerr << "[offscreen_capture_smoke] " << label
              << ": image is blank (variance below epsilon)\n";
    return false;
  }
  return true;
}

} // namespace

int main()
{
  const char* display = std::getenv("DISPLAY");
  if (display == nullptr || display[0] == '\0') {
    std::cout << "[offscreen_capture_smoke] SKIP: no DISPLAY set. Run under a "
                 "headless X server, e.g. xvfb-run -a -s '-screen 0 "
                 "1280x1024x24'.\n";
    return kSkip;
  }

  dart::gui::osg::OffscreenSetup setup;
  setup.width = 320;
  setup.height = 240;

  // Agent-friendly default framing from the scene bounding sphere (a 1m box at
  // the origin -> radius ~0.87).
  const dart::gui::osg::OffscreenCamera camera
      = dart::gui::osg::defaultAgentCamera(
          ::osg::BoundingSphere(::osg::Vec3(0.0f, 0.0f, 0.0f), 0.87f));

  const std::filesystem::path tmpDir = std::filesystem::temp_directory_path();
  const std::string pathSetup
      = (tmpDir / "dart_offscreen_smoke_setup.png").string();
  const std::string pathSugar
      = (tmpDir / "dart_offscreen_smoke_sugar.png").string();
  const std::string pathZeroWarmup
      = (tmpDir / "dart_offscreen_smoke_sugar_zero.png").string();
  const std::filesystem::path missingDir
      = tmpDir / "dart_offscreen_smoke_missing_dir";
  const std::string pathMissingDir = (missingDir / "capture.png").string();

  // Path 1: setUpOffscreenViewer + explicit frame loop + captureScreen.
  {
    auto world = buildBoxWorld();
    ::osg::ref_ptr<dart::gui::osg::ImGuiViewer> viewer
        = new dart::gui::osg::ImGuiViewer();
    ::osg::ref_ptr<dart::gui::osg::WorldNode> node
        = new dart::gui::osg::WorldNode(world);
    viewer->addWorldNode(node);

    if (!dart::gui::osg::setUpOffscreenViewer(*viewer, setup)) {
      std::cerr << "[offscreen_capture_smoke] setUpOffscreenViewer failed with "
                   "DISPLAY="
                << display << "\n";
      return 1;
    }

    auto* cam = viewer->getCamera();
    for (int i = 0; i < 10; ++i) {
      cam->setViewMatrixAsLookAt(camera.eye, camera.center, camera.up);
      viewer->frame();
    }
    viewer->captureScreen(pathSetup);
    viewer->frame(); // SaveScreen writes the PNG during this frame.

    if (!checkNonBlank(
            "setUpOffscreenViewer", pathSetup, setup.width, setup.height))
      return 1;
  }

  // Path 2: captureOffscreen one-shot sugar on a fresh viewer/world.
  {
    auto world = buildBoxWorld();
    ::osg::ref_ptr<dart::gui::osg::ImGuiViewer> viewer
        = new dart::gui::osg::ImGuiViewer();
    ::osg::ref_ptr<dart::gui::osg::WorldNode> node
        = new dart::gui::osg::WorldNode(world);
    viewer->addWorldNode(node);

    if (!dart::gui::osg::captureOffscreen(
            *viewer,
            pathSugar,
            camera.eye,
            camera.center,
            camera.up,
            setup,
            10)) {
      std::cerr << "[offscreen_capture_smoke] captureOffscreen failed with "
                   "DISPLAY="
                << display << "\n";
      return 1;
    }

    if (!checkNonBlank(
            "captureOffscreen", pathSugar, setup.width, setup.height))
      return 1;
  }

  // Path 3: captureOffscreen must still apply the requested view when callers
  // intentionally skip warm-up frames for already-initialized scenes.
  {
    auto world = buildBoxWorld();
    ::osg::ref_ptr<dart::gui::osg::ImGuiViewer> viewer
        = new dart::gui::osg::ImGuiViewer();
    ::osg::ref_ptr<dart::gui::osg::WorldNode> node
        = new dart::gui::osg::WorldNode(world);
    viewer->addWorldNode(node);

    if (!dart::gui::osg::captureOffscreen(
            *viewer,
            pathZeroWarmup,
            camera.eye,
            camera.center,
            camera.up,
            setup,
            0)) {
      std::cerr << "[offscreen_capture_smoke] captureOffscreen zero warm-up "
                   "failed with DISPLAY="
                << display << "\n";
      return 1;
    }

    if (!checkNonBlank(
            "captureOffscreen zero warm-up",
            pathZeroWarmup,
            setup.width,
            setup.height))
      return 1;
  }

  // Path 4: captureOffscreen reports write failures instead of returning true
  // when SaveScreen cannot create the requested file.
  {
    std::filesystem::remove_all(missingDir);

    auto world = buildBoxWorld();
    ::osg::ref_ptr<dart::gui::osg::ImGuiViewer> viewer
        = new dart::gui::osg::ImGuiViewer();
    ::osg::ref_ptr<dart::gui::osg::WorldNode> node
        = new dart::gui::osg::WorldNode(world);
    viewer->addWorldNode(node);

    if (dart::gui::osg::captureOffscreen(
            *viewer,
            pathMissingDir,
            camera.eye,
            camera.center,
            camera.up,
            setup,
            0)) {
      std::cerr
          << "[offscreen_capture_smoke] captureOffscreen unexpectedly "
             "succeeded for missing directory "
          << pathMissingDir << "\n";
      return 1;
    }
  }

  std::cout << "[offscreen_capture_smoke] PASS\n";
  return 0;
}
