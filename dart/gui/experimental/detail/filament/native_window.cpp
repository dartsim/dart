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

#include <dart/gui/experimental/detail/filament/native_window.hpp>
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

#include <ostream>
#include <utility>

namespace dart::gui::filament {

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
    const dart::gui::RunOptions& options, std::ostream& errors)
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

  glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
  GLFWwindow* window = glfwCreateWindow(
      options.width,
      options.height,
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

bool shouldContinueApplicationLoop(bool headless, GLFWwindow* window)
{
  return headless || (window != nullptr && !glfwWindowShouldClose(window));
}

} // namespace dart::gui::filament
