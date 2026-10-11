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

// Display-free capture smoke through the public Filament Viewer API. Every
// successful path reads the full RGBA PNG and verifies dimensions and visible
// box content. Graphics or write failures fail the test; there is no skip arm.

#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/BoxShape.hpp"
#include "dart/dynamics/FreeJoint.hpp"
#include "dart/dynamics/ShapeNode.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/gui/Viewer.hpp"
#include "dart/gui/WorldNode.hpp"
#include "dart/simulation/World.hpp"

#include <png.h>

#include <chrono>
#include <filesystem>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
dart::simulation::WorldPtr buildBoxWorld()
{
  auto world = dart::simulation::World::create();
  world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  auto skeleton = dart::dynamics::Skeleton::create("box");
  auto pair = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>(
      nullptr);
  auto shape = std::make_shared<dart::dynamics::BoxShape>(
      Eigen::Vector3d(1.0, 1.0, 1.0));
  auto* shapeNode = pair.second->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(shape);
  shapeNode->getVisualAspect()->setColor(Eigen::Vector3d(0.8, 0.2, 0.2));
  world->addSkeleton(skeleton);
  return world;
}

void checkImage(const std::filesystem::path& path, int width, int height)
{
  if (!std::filesystem::is_regular_file(path)
      || std::filesystem::file_size(path) == 0)
    throw std::runtime_error("Capture did not produce a nonempty PNG");
  png_image image{};
  image.version = PNG_IMAGE_VERSION;
  if (!png_image_begin_read_from_file(&image, path.string().c_str())) {
    const std::string error = image.message;
    png_image_free(&image);
    throw std::runtime_error("Cannot read capture: " + error);
  }
  if (image.width != static_cast<png_uint_32>(width)
      || image.height != static_cast<png_uint_32>(height)) {
    png_image_free(&image);
    throw std::runtime_error("Capture has incorrect dimensions");
  }
  image.format = PNG_FORMAT_RGBA;
  std::vector<unsigned char> pixels(PNG_IMAGE_SIZE(image));
  if (!png_image_finish_read(&image, nullptr, pixels.data(), 0, nullptr)) {
    const std::string error = image.message;
    png_image_free(&image);
    throw std::runtime_error("Cannot decode capture: " + error);
  }
  png_image_free(&image);
  std::size_t red = 0;
  double sum = 0, squares = 0;
  for (std::size_t i = 0; i < pixels.size(); i += 4) {
    const double r = pixels[i] / 255.0;
    const double g = pixels[i + 1] / 255.0;
    const double b = pixels[i + 2] / 255.0;
    red += pixels[i] > 40 && pixels[i] > pixels[i + 1] * 1.5
           && pixels[i] > pixels[i + 2] * 1.5;
    const double luminance = 0.299 * r + 0.587 * g + 0.114 * b;
    sum += luminance;
    squares += luminance * luminance;
  }
  const double count = static_cast<double>(width) * height;
  const double mean = sum / count;
  const double variance = squares / count - mean * mean;
  if (variance <= 1e-4 || red < 50)
    throw std::runtime_error("Capture has no visible red box");
  std::cout << "[offscreen_capture_smoke] " << path.filename().string() << ": "
            << width << 'x' << height << " red=" << red
            << " variance=" << variance << '\n';
}

void prepare(dart::gui::Viewer& viewer)
{
  viewer.addWorldNode(std::make_shared<dart::gui::WorldNode>(buildBoxWorld()));
  viewer.simulate(false);
  viewer.allowSimulation(false);
  viewer.setCameraHome(
      Eigen::Vector3d(2.5, -3, 2),
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d::UnitZ());
  viewer.setBackgroundColor(Eigen::Vector4d(0.12, 0.14, 0.18, 1));
  viewer.setUpViewForCapture(320, 240);
}

void capture(dart::gui::Viewer& viewer, const std::filesystem::path& path)
{
  viewer.captureScreen(path.string());
  viewer.frame();
}

void rejectCapture(dart::gui::Viewer& viewer, const std::filesystem::path& path)
{
  try {
    capture(viewer, path);
  } catch (const std::runtime_error& error) {
    if (std::string(error.what()).find("Cannot write capture '") != 0)
      throw;
    return;
  }
  throw std::runtime_error("Capture unexpectedly accepted an invalid path");
}

struct TemporaryDirectory
{
  std::filesystem::path path;
  TemporaryDirectory()
    : path(
        std::filesystem::temp_directory_path()
        / ("dart-offscreen-smoke-"
           + std::to_string(
               std::chrono::steady_clock::now().time_since_epoch().count())))
  {
    if (!std::filesystem::create_directory(path))
      throw std::runtime_error("Cannot create isolated capture directory");
  }
  ~TemporaryDirectory()
  {
    std::error_code ignored;
    std::filesystem::remove_all(path, ignored);
  }
};
} // namespace

int main()
{
  try {
    TemporaryDirectory temporary;
    // Explicit frame loop followed by queued capture.
    {
      dart::gui::Viewer viewer;
      prepare(viewer);
      for (int i = 0; i < 10; ++i)
        viewer.frame();
      const auto path = temporary.path / "warmed.png";
      capture(viewer, path);
      checkImage(path, 320, 240);
    }
    // A fresh context must apply the requested view without warm-up frames.
    {
      dart::gui::Viewer viewer;
      prepare(viewer);
      const auto path = temporary.path / "immediate.png";
      capture(viewer, path);
      checkImage(path, 320, 240);
      // Replacing a capture context must retain the scene and requested view.
      viewer.setUpViewForCapture(320, 240);
      const auto recreated = temporary.path / "recreated.png";
      capture(viewer, recreated);
      checkImage(recreated, 320, 240);
    }
    // Missing parents and directory targets must fail without deleting them.
    {
      dart::gui::Viewer viewer;
      prepare(viewer);
      rejectCapture(viewer, temporary.path / "missing" / "capture.png");
      const auto directory = temporary.path / "directory.png";
      std::filesystem::create_directory(directory);
      rejectCapture(viewer, directory);
      if (!std::filesystem::is_directory(directory))
        throw std::runtime_error("Capture removed its directory target");
    }
    std::cout << "[offscreen_capture_smoke] PASS\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "[offscreen_capture_smoke] " << error.what() << '\n';
    return 1;
  }
}
