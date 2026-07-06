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

#include <dart/gui/offscreen.hpp>
#include <dart/gui/viewer.hpp>

#include <gtest/gtest.h>

#include <algorithm>

#include <cstdlib>
#if !defined(_WIN32)
  #include <dlfcn.h>
#endif
#include <vector>

namespace {

void forceSoftwareGl()
{
#if !defined(_WIN32)
  setenv("LIBGL_ALWAYS_SOFTWARE", "1", 0);
  setenv("MESA_LOADER_DRIVER_OVERRIDE", "llvmpipe", 0);
#endif
}

bool canOpenLinuxDisplay()
{
#if defined(__linux__)
  const char* display = std::getenv("DISPLAY");
  if (display == nullptr || display[0] == '\0') {
    return false;
  }

  void* library = dlopen("libX11.so.6", RTLD_LAZY | RTLD_LOCAL);
  if (library == nullptr) {
    return false;
  }

  using XOpenDisplayFn = void* (*)(const char*);
  using XCloseDisplayFn = int (*)(void*);
  auto* openDisplay
      = reinterpret_cast<XOpenDisplayFn>(dlsym(library, "XOpenDisplay"));
  auto* closeDisplay
      = reinterpret_cast<XCloseDisplayFn>(dlsym(library, "XCloseDisplay"));
  if (openDisplay == nullptr || closeDisplay == nullptr) {
    dlclose(library);
    return false;
  }

  void* handle = openDisplay(display);
  if (handle == nullptr) {
    dlclose(library);
    return false;
  }

  closeDisplay(handle);
  dlclose(library);
  return true;
#else
  return true;
#endif
}

dart::gui::RenderableDescriptor makeBoxDescriptor()
{
  dart::gui::RenderableDescriptor descriptor;
  descriptor.id = 0xA5'0001u;
  descriptor.shapeFrameName = "offscreen_box";
  descriptor.shapeNodeName = "offscreen_box";
  descriptor.geometry.kind = dart::gui::ShapeKind::Box;
  descriptor.geometry.size = Eigen::Vector3d(0.8, 0.8, 0.8);
  descriptor.geometry.localBoundsMin = Eigen::Vector3d(-0.4, -0.4, -0.4);
  descriptor.geometry.localBoundsMax = Eigen::Vector3d(0.4, 0.4, 0.4);
  descriptor.geometry.hasLocalBounds = true;
  descriptor.material.rgba = Eigen::Vector4d(0.85, 0.22, 0.14, 1.0);
  descriptor.worldTransform = Eigen::Isometry3d::Identity();
  descriptor.renderResourceVersion = 1u;
  return descriptor;
}

bool hasVisibleContrast(const dart::gui::RenderedImage& image)
{
  if (image.pixels.size()
      != static_cast<std::size_t>(image.width)
             * static_cast<std::size_t>(image.height)
             * static_cast<std::size_t>(image.channels)) {
    return false;
  }

  auto minValue = std::uint8_t{255};
  auto maxValue = std::uint8_t{0};
  for (std::size_t i = 0; i + 3 < image.pixels.size(); i += 4) {
    minValue = std::min(
        {minValue, image.pixels[i], image.pixels[i + 1], image.pixels[i + 2]});
    maxValue = std::max(
        {maxValue, image.pixels[i], image.pixels[i + 1], image.pixels[i + 2]});
  }
  return static_cast<int>(maxValue) - static_cast<int>(minValue) > 12;
}

} // namespace

TEST(OffscreenRenderer, RendersDescriptorSceneDeterministically)
{
  forceSoftwareGl();
  if (!canOpenLinuxDisplay()) {
    GTEST_SKIP()
        << "Filament OpenGL headless rendering requires a usable DISPLAY/Xvfb";
  }

  dart::gui::OffscreenRenderOptions options;
  options.width = 160;
  options.height = 120;
  options.renderBackend = "opengl";
  options.warmupFrames = 2;

  dart::gui::OffscreenRenderer renderer(options);
  dart::gui::OrbitCamera camera;
  camera.target = Eigen::Vector3d::Zero();
  camera.distance = 3.0;
  camera.yaw = -0.75;
  camera.pitch = 0.32;

  const std::vector<dart::gui::RenderableDescriptor> descriptors{
      makeBoxDescriptor()};
  const dart::gui::RenderedImage first = renderer.render(descriptors, camera);
  ASSERT_EQ(first.width, 160u);
  ASSERT_EQ(first.height, 120u);
  ASSERT_EQ(first.channels, 4u);
  EXPECT_TRUE(hasVisibleContrast(first));

  const dart::gui::RenderedImage second = renderer.render(descriptors, camera);
  ASSERT_EQ(second.width, first.width);
  ASSERT_EQ(second.height, first.height);
  ASSERT_EQ(second.channels, first.channels);
  EXPECT_EQ(second.pixels, first.pixels);
}
