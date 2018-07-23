/*
 * Copyright (c) 2011-2018, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#include "dart/gui/glfw/Window.hpp"

#include <iostream>
#include <thread>

//#include <GLFW/glfw3.h>

#include "dart/common/Console.hpp"
#include "dart/common/Platform.hpp"
//#include "dart/gui/glfw/LoadGlfw.hpp"
#include "dart/gui/LoadOpengl.hpp"
#include "dart/math/Constants.hpp"

namespace dart {
namespace gui {
namespace glfw {

//==============================================================================
bool Window::mMainloopActive = false;
std::unordered_map<GLFWwindow*, Window*> Window::mViewerMap;
const float Window::SCROLL_SENSITIVITY = 0.08f;

//==============================================================================
Window::Window(const std::string& title, int width, int height, bool show)
  : mIsVisible(true), mClearColor(Eigen::Vector4f(1.0, 1.0, 1.0, 1.0))
{
  if (mViewerMap.empty())
    startupGlfw();

  glfwWindowHint(GLFW_VISIBLE, show);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);

  mGlfwWindow
      = glfwCreateWindow(width, height, title.c_str(), nullptr, nullptr);
  if (!mGlfwWindow)
  {
    dterr << "[Window::initWindow] Failed to create a GLFW window";
    exit(EXIT_FAILURE);
  }

  auto ok = mViewerMap.insert(std::make_pair(mGlfwWindow, this)).second;
  if (!ok)
  {
    dterr << "[Window::initWindow] Attempting to create the same GLFW window";
    exit(EXIT_FAILURE);
  }

  glfwMakeContextCurrent(mGlfwWindow);

  glewExperimental = GL_TRUE;
  const GLenum error = glewInit();
  if (error != GLEW_OK)
  {
    assert(false);
  }

  setCallbacks();

  // TODO(JS): Call initializations
}

//==============================================================================
Window::~Window()
{
  assert(mGlfwWindow);

  glfwDestroyWindow(mGlfwWindow);

  mViewerMap.erase(mGlfwWindow);
  if (mViewerMap.empty())
    shutdownGlfw();
}

//==============================================================================
Window* Window::findWindow(GLFWwindow* window)
{
  auto result = mViewerMap.find(window);

  if (result == mViewerMap.end())
  {
    dterr << "[Window::current] Failed to find an Window associated GLFWwindow."
             " Exiting.\n";
    exit(EXIT_FAILURE);
  }

  return result->second;
}

//==============================================================================
void Window::setScenePosition(
    const Eigen::Vector3f& position, float /*sceneRadius*/)
{
  // Set the position and radius of the scene
  mCenterScene = position;
  //  mCamera.setSceneRadius(sceneRadius);

  // Reset the camera position and zoom in order to view all the scene
  resetCameraToViewAll();
}

//==============================================================================
void Window::resetCameraToViewAll()
{
  // Move the camera to the origin of the scene
  //  mCamera.translateWorld(-mCamera.getTranslation());

  // Move the camera to the center of the scene
  //  mCamera.translateWorld(mCenterScene);

  // Set the zoom of the camera so that the scene center is
  // in negative view direction of the camera
  //  mCamera.setZoom(1.0);
}

//==============================================================================
bool Window::mapMouseCoordinatesToSphere(
    double xMouse, double yMouse, Eigen::Vector3f& spherePoint) const
{
  const auto pi = math::constantsf::pi();

  if ((xMouse >= 0) && (xMouse <= mWindowWidth) && (yMouse >= 0)
      && (yMouse <= mWindowHeight))
  {
    auto x = float(xMouse - 0.5f * mWindowWidth) / float(mWindowWidth);
    auto y = float(0.5f * mWindowHeight - yMouse) / float(mWindowHeight);
    auto sinx = std::sin(pi * x * 0.5f);
    auto siny = std::sin(pi * y * 0.5f);
    auto sinx2siny2 = sinx * sinx + siny * siny;

    // Compute the point on the sphere
    spherePoint[0] = sinx;
    spherePoint[1] = siny;
    spherePoint[2] = (sinx2siny2 < 1.0f) ? std::sqrt(1.0f - sinx2siny2) : 0.0f;

    return true;
  }

  return false;
}

//==============================================================================
void Window::zoom(float /*zoomDiff*/)
{
  // Zoom the camera
  //  mCamera.setZoom(zoomDiff);
}

//==============================================================================
void Window::translate(int /*xMouse*/, int /*yMouse*/)
{
  //  const auto dx = static_cast<float>(xMouse - mLastMouseX);
  //  const auto dy = static_cast<float>(yMouse - mLastMouseY);

  // Translate the camera
  //  mCamera.translateCamera(
  //      -dx / float(mCamera.getWidth()),
  //      -dy / float(mCamera.getHeight()),
  //      mCenterScene);
}

//==============================================================================
void Window::rotate(int xMouse, int yMouse)
{
  if (mIsLastPointOnSphereValid)
  {
    Eigen::Vector3f newPoint3D;
    bool isNewPointOK = mapMouseCoordinatesToSphere(xMouse, yMouse, newPoint3D);

    if (isNewPointOK)
    {
      Eigen::Vector3f axis = mLastPointOnSphere.cross(newPoint3D);
      float cosAngle = mLastPointOnSphere.dot(newPoint3D);

      float epsilon = std::numeric_limits<float>::epsilon();
      if (std::fabs(cosAngle) < 1.0f && axis.norm() > epsilon)
      {
        axis.normalize();
        //        float angle = 2.0f * std::acos(cosAngle);

        // Rotate the camera around the center of the scene
        //        mCamera.rotateAroundLocalPoint(axis, -angle, mCenterScene);
      }
    }
  }
}

//==============================================================================
void Window::runAllViewers(std::size_t refresh)
{
  if (mMainloopActive)
    throw std::runtime_error(
        "[Viewer::runMainLoop] Attempting to run the main "
        "loop twice. Ignoring this action.\n");

  mMainloopActive = true;

  std::thread refreshThread;
  if (refresh > 0)
  {
    refreshThread = std::thread([refresh]() {
      std::chrono::milliseconds time(refresh);
      while (mMainloopActive)
      {
        std::this_thread::sleep_for(time);
        glfwPostEmptyEvent();
      }
    });
  }

  try
  {
    while (mMainloopActive)
    {
      int numScreens = 0;

      for (auto windowPair : mViewerMap)
      {
        auto& window = windowPair.second;

        if (!window->isVisible())
        {
          continue;
        }
        else if (glfwWindowShouldClose(window->mGlfwWindow))
        {
          window->hide();
          continue;
        }

        window->render();
        numScreens++;
      }

      if (numScreens == 0)
      {
        // Give up if there was nothing to render
        mMainloopActive = false;
        break;
      }

      // Wait for mouse/keyboard or empty refresh events
      glfwWaitEvents();
    }

    // Process events once more
    glfwPollEvents();
  }
  catch (const std::exception& e)
  {
    dterr << "[Viewer::runAllViewers] Caught exception in main loop: "
          << e.what() << "\n";
    abort();
  }

  if (refresh > 0)
    refreshThread.join();
}

//==============================================================================
void Window::setTitle(const std::string& title)
{
  glfwSetWindowTitle(mGlfwWindow, title.c_str());
}

//==============================================================================
void Window::setSize(const Eigen::Vector2i& size)
{
  setSize(size[0], size[1]);
}

//==============================================================================
void Window::setSize(int width, int height)
{
  glfwSetWindowSize(mGlfwWindow, width, height);
}

//==============================================================================
Eigen::Vector2i Window::getSize() const
{
  Eigen::Vector2i size;

  glfwGetWindowSize(mGlfwWindow, &size[0], &size[1]);

  return size;
}

//==============================================================================
void Window::setVisible(bool visible)
{
  if (mIsVisible == visible)
    return;

  mIsVisible = visible;

  if (mIsVisible)
    glfwShowWindow(mGlfwWindow);
  else
    glfwHideWindow(mGlfwWindow);
}

//==============================================================================
void Window::show()
{
  setVisible(true);
}

//==============================================================================
void Window::hide()
{
  setVisible(false);
}

//==============================================================================
bool Window::isVisible() const
{
  return mIsVisible;
}

//==============================================================================
void Window::windowSizeCallback(int /*width*/, int /*height*/)
{
  // Do nothing
}

//==============================================================================
void Window::framebufferSizeCallback(int width, int height)
{
  mWindowWidth = width;
  mWindowHeight = height;

  mViewportX = 0;
  mViewportY = 0;

  mViewportWidth = mWindowWidth;
  mViewportHeight = mWindowHeight;

  glViewport(mViewportX, mViewportY, mViewportWidth, mViewportHeight);
}

//==============================================================================
void Window::cursorPosCallback(double xMouse, double yMouse)
{
  const int leftButtonState
      = glfwGetMouseButton(mGlfwWindow, GLFW_MOUSE_BUTTON_LEFT);
  const int rightButtonState
      = glfwGetMouseButton(mGlfwWindow, GLFW_MOUSE_BUTTON_RIGHT);
  const int middleButtonState
      = glfwGetMouseButton(mGlfwWindow, GLFW_MOUSE_BUTTON_MIDDLE);
  const int altKeyState = glfwGetKey(mGlfwWindow, GLFW_KEY_LEFT_ALT);

  // Zoom
  if (leftButtonState == GLFW_PRESS && altKeyState == GLFW_PRESS)
  {
    const float dy = static_cast<float>(yMouse - mLastMouseY);
    const float h = static_cast<float>(mWindowHeight);

    // Zoom the camera
    zoom(-dy / h);

    std::cout << "zoom" << std::endl;
  }
  // Translation
  else if (
      middleButtonState == GLFW_PRESS || rightButtonState == GLFW_PRESS
      || (leftButtonState == GLFW_PRESS && altKeyState == GLFW_PRESS))
  {
    std::cout << "translate" << std::endl;
    translate(static_cast<int>(xMouse), static_cast<int>(yMouse));
  }
  // Rotation
  else if (leftButtonState == GLFW_PRESS)
  {
    std::cout << "rotate" << std::endl;
    rotate(static_cast<int>(xMouse), static_cast<int>(yMouse));
  }

  // Remember the mouse position
  mLastMouseX = xMouse;
  mLastMouseY = yMouse;
  mIsLastPointOnSphereValid
      = mapMouseCoordinatesToSphere(xMouse, yMouse, mLastPointOnSphere);
}

//==============================================================================
void Window::mouseButtonCallback(int /*button*/, int action, int /*mods*/)
{
  // Get the mouse cursor position
  double x;
  double y;
  glfwGetCursorPos(mGlfwWindow, &x, &y);

  // TODO: For debug
  std::cout << "Mouse clicked at (" << x << ", " << y << ")" << std::endl;

  // If the mouse button is pressed
  if (action == GLFW_PRESS)
  {
    mLastMouseX = x;
    mLastMouseY = y;
    mIsLastPointOnSphereValid
        = mapMouseCoordinatesToSphere(x, y, mLastPointOnSphere);
  }
  else
  {
    // If the mouse button is released
    mIsLastPointOnSphereValid = false;
  }
}

//==============================================================================
void Window::keyboardCallback(
    int key, int /*scancode*/, int action, int /*mods*/)
{
  if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
    glfwSetWindowShouldClose(mGlfwWindow, true);
}

//==============================================================================
void Window::charCallback(unsigned int /*codepoint*/)
{
  // Do nothing
}

//==============================================================================
void Window::dropCallback(int /*count*/, const char** /*filenames*/)
{
  // Do nothing
}

//==============================================================================
void Window::scrollCallback(double /*xoffset*/, double yoffset)
{
  zoom(static_cast<float>(yoffset) * SCROLL_SENSITIVITY);
}

//==============================================================================
void Window::render()
{
  glfwMakeContextCurrent(mGlfwWindow);

  // Enabling color write (previously disabled for light POV z-buffer rendering)
  glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);

  // Rendering
  glClearColor(
      mClearColor.x(), mClearColor.y(), mClearColor.z(), mClearColor.w());
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

  renderScene();
  renderGui();

  glfwSwapBuffers(mGlfwWindow);
}

//==============================================================================
void Window::renderScene()
{
  // Do nothing
}

//==============================================================================
void Window::renderGui()
{
  // Do nothing
}

//==============================================================================
void Window::startupGlfw()
{
  assert(mViewerMap.empty());

  // dtmsg << "Starting GLFW " << glfwGetVersionString() << "\n";

  // Setup window
  glfwSetErrorCallback([](int error, const char* description) {
    dterr << "GLFW Error occured, Error ID: " << error
          << ", Description: " << description << "\n";
  });

  if (!glfwInit())
    throw std::runtime_error("Could not initialize GLFW.");

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#if DART_OS_MACOS
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif
}

//==============================================================================
void Window::shutdownGlfw()
{
  assert(mViewerMap.empty());

  // TODO(JS): Notify OpenGL context is being shutting down to
  // - scene

  glfwTerminate();
}

//==============================================================================
void Window::setCallbacks()
{
  assert(mGlfwWindow);
  assert(glfwGetCurrentContext() == mGlfwWindow);

  glfwSetWindowSizeCallback(
      mGlfwWindow, [](GLFWwindow* window, int width, int height) {
        auto* viewer = findWindow(window);
        if (viewer)
          viewer->windowSizeCallback(width, height);
      });

  glfwSetCursorPosCallback(
      mGlfwWindow, [](GLFWwindow* window, double xpos, double ypos) {
        auto* viewer = findWindow(window);
        if (viewer)
          viewer->cursorPosCallback(xpos, ypos);
      });

  glfwSetMouseButtonCallback(
      mGlfwWindow, [](GLFWwindow* window, int button, int action, int mods) {
        auto* viewer = findWindow(window);
        if (viewer)
          viewer->mouseButtonCallback(button, action, mods);
      });

  glfwSetKeyCallback(
      mGlfwWindow,
      [](GLFWwindow* window, int key, int scancode, int action, int mods) {
        auto* viewer = findWindow(window);
        if (viewer)
          viewer->keyboardCallback(key, scancode, action, mods);
      });

  glfwSetCharCallback(
      mGlfwWindow, [](GLFWwindow* window, unsigned int codepoint) {
        auto* viewer = findWindow(window);
        if (viewer)
          viewer->charCallback(codepoint);
      });

  glfwSetDropCallback(
      mGlfwWindow, [](GLFWwindow* window, int count, const char** filenames) {
        auto* viewer = findWindow(window);
        if (viewer)
          viewer->dropCallback(count, filenames);
      });

  glfwSetScrollCallback(
      mGlfwWindow, [](GLFWwindow* window, double xoffset, double yoffset) {
        auto* viewer = findWindow(window);
        if (viewer)
          viewer->scrollCallback(xoffset, yoffset);
      });

  glfwSetFramebufferSizeCallback(
      mGlfwWindow, [](GLFWwindow* window, int width, int height) {
        auto* viewer = findWindow(window);
        if (viewer)
          viewer->framebufferSizeCallback(width, height);
      });
}

//==============================================================================
void Window::initLights()
{
  static float ambient[] = {0.2f, 0.2f, 0.2f, 1.0f};
  static float diffuse[] = {0.6f, 0.6f, 0.6f, 1.0f};
  static float front_mat_shininess[] = {60.0f};
  static float front_mat_specular[] = {0.2f, 0.2f, 0.2f, 1.0f};
  static float front_mat_diffuse[] = {0.5f, 0.28f, 0.38f, 1.0f};
  static float lmodel_ambient[] = {0.2f, 0.2f, 0.2f, 1.0f};
  static float lmodel_twoside[] = {GL_FALSE};

  GLfloat position[] = {1.0f, 0.0f, 0.0f, 0.0f};
  GLfloat position1[] = {-1.0f, 0.0f, 0.0f, 0.0f};

  glEnable(GL_LIGHT0);
  glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
  glLightfv(GL_LIGHT0, GL_POSITION, position);

  glLightModelfv(GL_LIGHT_MODEL_AMBIENT, lmodel_ambient);
  glLightModelfv(GL_LIGHT_MODEL_TWO_SIDE, lmodel_twoside);

  glEnable(GL_LIGHT1);
  glLightfv(GL_LIGHT1, GL_DIFFUSE, diffuse);
  glLightfv(GL_LIGHT1, GL_POSITION, position1);
  glEnable(GL_LIGHTING);
  glEnable(GL_COLOR_MATERIAL);

  glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, front_mat_shininess);
  glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, front_mat_specular);
  glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, front_mat_diffuse);

  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LEQUAL);
  glDisable(GL_CULL_FACE);
  glEnable(GL_NORMALIZE);
}

} // namespace glfw
} // namespace gui
} // namespace dart
