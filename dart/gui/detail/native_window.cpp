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

#include <dart/gui/detail/gui_scale.hpp>
#include <dart/gui/detail/native_window.hpp>
#include <dart/gui/viewer.hpp>

#include <GLFW/glfw3.h>

#if defined(__linux__)
  #define GLFW_EXPOSE_NATIVE_X11
  #include <GLFW/glfw3native.h>
#elif defined(_WIN32)
  #define GLFW_EXPOSE_NATIVE_WIN32
  #include <GLFW/glfw3native.h>
#elif defined(__APPLE__)
  #define GLFW_EXPOSE_NATIVE_COCOA
  #include <GLFW/glfw3native.h>
#endif

#include <algorithm>
#include <ostream>
#include <utility>

namespace dart::gui::detail {
namespace {

struct MonitorWorkArea
{
  int x = 0;
  int y = 0;
  int width = 0;
  int height = 0;
};

double resolveMonitorDpiScale(GLFWmonitor* monitor)
{
  if (const auto overrideScale = guiDpiScaleOverrideFromEnvironment()) {
    return *overrideScale;
  }

  if (monitor == nullptr) {
    return 1.0;
  }

#if GLFW_VERSION_MAJOR > 3                                                     \
    || (GLFW_VERSION_MAJOR == 3 && GLFW_VERSION_MINOR >= 3)
  float xScale = 1.0f;
  float yScale = 1.0f;
  glfwGetMonitorContentScale(monitor, &xScale, &yScale);
  return normalizeGuiDpiScale((std::max)(xScale, yScale));
#else
  return 1.0;
#endif
}

MonitorWorkArea resolveMonitorWorkArea(GLFWmonitor* monitor)
{
  MonitorWorkArea workArea;
  if (monitor == nullptr) {
    return workArea;
  }

#if GLFW_VERSION_MAJOR > 3                                                     \
    || (GLFW_VERSION_MAJOR == 3 && GLFW_VERSION_MINOR >= 3)
  glfwGetMonitorWorkarea(
      monitor, &workArea.x, &workArea.y, &workArea.width, &workArea.height);
  if (workArea.width <= 0 || workArea.height <= 0) {
    workArea.width = 0;
    workArea.height = 0;
  }
#else
  const GLFWvidmode* mode = glfwGetVideoMode(monitor);
  if (mode != nullptr) {
    workArea.width = mode->width;
    workArea.height = mode->height;
  }
#endif

  return workArea;
}

GLFWmonitor* resolveWindowMonitor(GLFWwindow* window)
{
  GLFWmonitor* primaryMonitor = glfwGetPrimaryMonitor();
  if (window == nullptr) {
    return primaryMonitor;
  }

  int windowX = 0;
  int windowY = 0;
  int windowWidth = 0;
  int windowHeight = 0;
  glfwGetWindowPos(window, &windowX, &windowY);
  glfwGetWindowSize(window, &windowWidth, &windowHeight);
  if (windowWidth <= 0 || windowHeight <= 0) {
    return primaryMonitor;
  }

  int monitorCount = 0;
  GLFWmonitor** monitors = glfwGetMonitors(&monitorCount);
  GLFWmonitor* bestMonitor = primaryMonitor;
  int bestOverlapArea = 0;
  for (int i = 0; i < monitorCount; ++i) {
    GLFWmonitor* monitor = monitors[i];
    const MonitorWorkArea workArea = resolveMonitorWorkArea(monitor);
    if (workArea.width <= 0 || workArea.height <= 0) {
      continue;
    }

    const int overlapWidth = (std::max)(0,
                                        (std::min)(windowX + windowWidth,
                                                   workArea.x + workArea.width)
                                            - (std::max)(windowX, workArea.x));
    const int overlapHeight
        = (std::max)(0,
                     (std::min)(windowY + windowHeight,
                                workArea.y + workArea.height)
                         - (std::max)(windowY, workArea.y));
    const int overlapArea = overlapWidth * overlapHeight;
    if (overlapArea > bestOverlapArea) {
      bestOverlapArea = overlapArea;
      bestMonitor = monitor;
    }
  }

  return bestMonitor;
}

} // namespace

ApplicationWindow::ApplicationWindow(GLFWwindow* window) : mWindow(window) {}

ApplicationWindow::~ApplicationWindow()
{
  reset();
}

ApplicationWindow::ApplicationWindow(ApplicationWindow&& other) noexcept
  : mWindow(std::exchange(other.mWindow, nullptr))
{
}

ApplicationWindow& ApplicationWindow::operator=(
    ApplicationWindow&& other) noexcept
{
  if (this != &other) {
    reset();
    mWindow = std::exchange(other.mWindow, nullptr);
  }
  return *this;
}

GLFWwindow* ApplicationWindow::get() const
{
  return mWindow;
}

void ApplicationWindow::reset(GLFWwindow* window)
{
  if (mWindow != nullptr) {
    glfwDestroyWindow(mWindow);
    glfwTerminate();
  }
  mWindow = window;
}

ApplicationWindow createApplicationWindow(
    const dart::gui::RunOptions& options,
    bool automaticWindowWidth,
    bool automaticWindowHeight,
    std::ostream& errors)
{
  if (options.headless) {
    return {};
  }

#if defined(__linux__)
  glfwInitHint(GLFW_PLATFORM, GLFW_PLATFORM_X11);
#endif
  if (!glfwInit()) {
    errors << "Failed to initialize GLFW\n";
    return {};
  }

  GLFWmonitor* primaryMonitor = glfwGetPrimaryMonitor();
  const MonitorWorkArea workArea = resolveMonitorWorkArea(primaryMonitor);
  const GuiScaleState guiScale = makeGuiScaleState(
      options.guiScale, resolveMonitorDpiScale(primaryMonitor));
  const GuiWindowSize windowSize = resolveAutomaticGuiWindowSize(
      options.width,
      options.height,
      guiScale.effectiveScale,
      workArea.width,
      workArea.height,
      automaticWindowWidth,
      automaticWindowHeight);

  glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
  GLFWwindow* window = glfwCreateWindow(
      windowSize.width,
      windowSize.height,
      options.windowTitle.c_str(),
      nullptr,
      nullptr);
  if (window == nullptr) {
    errors << "Failed to create GLFW window\n";
    glfwTerminate();
    return {};
  }

  return ApplicationWindow(window);
}

void* getNativeWindow(GLFWwindow* window)
{
#if defined(__linux__)
  return reinterpret_cast<void*>(glfwGetX11Window(window));
#elif defined(_WIN32)
  return glfwGetWin32Window(window);
#elif defined(__APPLE__)
  return glfwGetCocoaWindow(window);
#else
  (void)window;
  return nullptr;
#endif
}

double resolveWindowDpiScale(GLFWwindow* window)
{
  if (const auto overrideScale = guiDpiScaleOverrideFromEnvironment()) {
    return *overrideScale;
  }

  if (window == nullptr) {
    return 1.0;
  }

#if GLFW_VERSION_MAJOR > 3                                                     \
    || (GLFW_VERSION_MAJOR == 3 && GLFW_VERSION_MINOR >= 3)
  float xScale = 1.0f;
  float yScale = 1.0f;
  glfwGetWindowContentScale(window, &xScale, &yScale);
  return normalizeGuiDpiScale((std::max)(xScale, yScale));
#else
  return 1.0;
#endif
}

void resizeAutomaticApplicationWindow(
    GLFWwindow* window,
    const dart::gui::RunOptions& options,
    const GuiScaleState& guiScale,
    bool automaticWindowWidth,
    bool automaticWindowHeight)
{
  if (window == nullptr || (!automaticWindowWidth && !automaticWindowHeight)) {
    return;
  }

  const MonitorWorkArea workArea
      = resolveMonitorWorkArea(resolveWindowMonitor(window));
  const GuiWindowSize targetSize = resolveAutomaticGuiWindowSize(
      options.width,
      options.height,
      guiScale.effectiveScale,
      workArea.width,
      workArea.height,
      automaticWindowWidth,
      automaticWindowHeight);

  int windowWidth = targetSize.width;
  int windowHeight = targetSize.height;
  glfwGetWindowSize(window, &windowWidth, &windowHeight);
  if (windowWidth == targetSize.width && windowHeight == targetSize.height) {
    return;
  }

  glfwSetWindowSize(window, targetSize.width, targetSize.height);
}

bool shouldContinueApplicationLoop(bool headless, GLFWwindow* window)
{
  return headless || (window != nullptr && !glfwWindowShouldClose(window));
}

} // namespace dart::gui::detail
