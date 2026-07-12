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
#include <stdexcept>
#include <string>
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

void setEnvVar(const char* name, const char* value)
{
#if defined(_WIN32)
  _putenv_s(name, value);
#else
  setenv(name, value, 1);
#endif
}

void unsetEnvVar(const char* name)
{
#if defined(_WIN32)
  _putenv_s(name, "");
#else
  unsetenv(name);
#endif
}

class ScopedEnvVar
{
public:
  ScopedEnvVar(const char* name, const char* value)
    : mName(name)
  {
    if (const char* previous = std::getenv(name)) {
      mHadPrevious = true;
      mPrevious = previous;
    }
    setEnvVar(name, value);
  }

  ~ScopedEnvVar()
  {
    if (mHadPrevious) {
      setEnvVar(mName.c_str(), mPrevious.c_str());
    } else {
      unsetEnvVar(mName.c_str());
    }
  }

private:
  std::string mName;
  bool mHadPrevious = false;
  std::string mPrevious;
};

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

TEST(OffscreenRenderer, RejectsNoopBackendFromEnvironment)
{
  const ScopedEnvVar backendOverride("DART_FILAMENT_BACKEND", "noop");

  dart::gui::OffscreenRenderOptions options;
  options.width = 16;
  options.height = 16;
  options.renderBackend = "opengl";

  try {
    dart::gui::OffscreenRenderer renderer(options);
    FAIL() << "OffscreenRenderer accepted the noop Filament backend";
  } catch (const std::runtime_error& error) {
    EXPECT_NE(std::string(error.what()).find("noop"), std::string::npos)
        << error.what();
  }
}

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

TEST(OffscreenRenderer, DebugSceneOverlayIsAppliedAndCleared)
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

  dart::gui::DebugScene debug;
  dart::gui::DebugLineDescriptor line;
  line.from = Eigen::Vector3d(-1.0, 0.0, 0.6);
  line.to = Eigen::Vector3d(1.0, 0.0, 0.6);
  line.rgba = Eigen::Vector4d(1.0, 0.9, 0.1, 1.0);
  debug.lines.push_back(line);

  const dart::gui::RenderedImage plain = renderer.render(descriptors, camera);
  const dart::gui::RenderedImage withDebug
      = renderer.render(descriptors, camera, debug);
  const dart::gui::RenderedImage plainAgain
      = renderer.render(descriptors, camera);
  dart::gui::DebugScene empty;
  const dart::gui::RenderedImage withEmptyDebug
      = renderer.render(descriptors, camera, empty);

  // Same-renderer replays can drift by a few levels (temporal pipeline
  // state; the offscreen parity gate tolerates this too), so compare with a
  // small per-pixel budget: the overlay must move many pixels far beyond
  // that budget, and clearing it must fall back inside the budget.
  const auto countPixelsOver =
      [](const dart::gui::RenderedImage& a,
         const dart::gui::RenderedImage& b,
         int threshold) {
        std::size_t count = 0;
        const std::size_t size = std::min(a.pixels.size(), b.pixels.size());
        for (std::size_t i = 0; i < size; ++i) {
          const int delta = std::abs(
              static_cast<int>(a.pixels[i]) - static_cast<int>(b.pixels[i]));
          if (delta > threshold) {
            ++count;
          }
        }
        return count;
      };
  constexpr int kReplayTolerance = 4;
  // Same-renderer replays showed a ~6-byte noise floor at this tolerance on
  // llvmpipe; the overlay line changes hundreds of bytes. Keep an order of
  // magnitude between the leak budget and the required overlay signal.
  constexpr std::size_t kReplayNoiseBudgetBytes = 64;
  constexpr std::size_t kOverlayMinChangedBytes = 256;

  EXPECT_GE(
      countPixelsOver(withDebug, plain, kReplayTolerance),
      kOverlayMinChangedBytes);
  // The overlay must not leak into a subsequent plain render or survive an
  // empty DebugScene.
  EXPECT_LE(
      countPixelsOver(plainAgain, plain, kReplayTolerance),
      kReplayNoiseBudgetBytes);
  EXPECT_LE(
      countPixelsOver(withEmptyDebug, plain, kReplayTolerance),
      kReplayNoiseBudgetBytes);
}
