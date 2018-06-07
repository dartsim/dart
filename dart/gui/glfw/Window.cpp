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

#include <thread>

#include <GLFW/glfw3.h>

#include "dart/common/Console.hpp"
#include "dart/gui/OpenGLRenderInterface.hpp"
#include "dart/gui/glfw/gl3w.h"

namespace dart {
namespace gui {
namespace glfw {

//==============================================================================
bool Viewer::mMainloopActive = false;
std::unordered_map<GLFWwindow*, Viewer*> Viewer::mViewerMap;

//==============================================================================
Viewer::Viewer(const std::string& title, int width, int height, bool show)
  : mIsVisible(true), mClearColor(Eigen::Vector4f(1, 1, 1, 1))
{
  mCapture = false;

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

  setCallbacks();
}

//==============================================================================
Viewer::~Viewer()
{
  assert(mGlfwWindow);

  glfwDestroyWindow(mGlfwWindow);

  mViewerMap.erase(mGlfwWindow);
  if (mViewerMap.empty())
    shutdownGlfw();
}

//==============================================================================
Viewer* Viewer::findViewer(GLFWwindow* window)
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
void Viewer::runAllViewers(std::size_t refresh)
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

        //        window->render();
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
          << e.what();
    abort();
  }

  if (refresh > 0)
    refreshThread.join();
}

//==============================================================================
void Viewer::setTitle(const std::string& title)
{
  glfwSetWindowTitle(mGlfwWindow, title.c_str());
}

//==============================================================================
void Viewer::setSize(const Eigen::Vector2i& size)
{
  setSize(size[0], size[1]);
}

//==============================================================================
void Viewer::setSize(int width, int height)
{
  glfwSetWindowSize(mGlfwWindow, width, height);
}

//==============================================================================
Eigen::Vector2i Viewer::getSize() const
{
  Eigen::Vector2i size;

  glfwGetWindowSize(mGlfwWindow, &size[0], &size[1]);

  return size;
}

//==============================================================================
void Viewer::setVisible(bool visible)
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
void Viewer::show()
{
  setVisible(true);
}

//==============================================================================
void Viewer::hide()
{
  setVisible(false);
}

//==============================================================================
bool Viewer::isVisible() const
{
  return mIsVisible;
}

//==============================================================================
void Viewer::windowSizeCallback(int width, int height)
{
  mWinWidth = width;
  mWinHeight = height;

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glViewport(0, 0, mWinWidth, mWinHeight);
  gluPerspective(
      mPersp,
      static_cast<double>(mWinWidth) / static_cast<double>(mWinHeight),
      0.1,
      10.0);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  mTrackBall.setCenter(Eigen::Vector2d(width * 0.5, height * 0.5));
  mTrackBall.setRadius(std::min(width, height) / 2.5);

  //  glfwPostRedisplay();
}

//==============================================================================
void Viewer::cursorPosCallback(double xpos, double ypos)
{
}

//==============================================================================
void Viewer::mouseButtonCallback(int button, int action, int mods)
{
}

//==============================================================================
void Viewer::keyboardCallback(
    int key, int /*scancode*/, int action, int /*mods*/)
{
  if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
    glfwSetWindowShouldClose(mGlfwWindow, true);
}

//==============================================================================
void Viewer::charCallback(unsigned int codepoint)
{
}

//==============================================================================
void Viewer::dropCallback(int count, const char** filenames)
{
}

//==============================================================================
void Viewer::scrollCallback(double xoffset, double yoffset)
{
}

//==============================================================================
void Viewer::render()
{
  glfwMakeContextCurrent(mGlfwWindow);

  // Rendering
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(
      mPersp,
      static_cast<double>(mWinWidth) / static_cast<double>(mWinHeight),
      0.1,
      10.0);
  gluLookAt(mEye[0], mEye[1], mEye[2], 0.0, 0.0, -1.0, mUp[0], mUp[1], mUp[2]);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  glClearColor(
      mClearColor.x(), mClearColor.y(), mClearColor.z(), mClearColor.w());
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

  mTrackBall.applyGLRotation();

  // Draw world origin indicator
  if (!mCapture)
  {
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_TEXTURE_2D);
    glDisable(GL_LIGHTING);
    glLineWidth(2.0);
    if (mRotate || mTranslate || mZooming)
    {
      glColor3f(1.0f, 0.0f, 0.0f);
      glBegin(GL_LINES);
      glVertex3f(-0.1f, 0.0f, -0.0f);
      glVertex3f(0.15f, 0.0f, -0.0f);
      glEnd();

      glColor3f(0.0f, 1.0f, 0.0f);
      glBegin(GL_LINES);
      glVertex3f(0.0f, -0.1f, 0.0f);
      glVertex3f(0.0f, 0.15f, 0.0f);
      glEnd();

      glColor3f(0.0f, 0.0f, 1.0f);
      glBegin(GL_LINES);
      glVertex3f(0.0f, 0.0f, -0.1f);
      glVertex3f(0.0f, 0.0f, 0.15f);
      glEnd();
    }
  }

  glScalef(mZoom, mZoom, mZoom);
  glTranslatef(mTrans[0] * 0.001, mTrans[1] * 0.001, mTrans[2] * 0.001);

  initLights();
  renderScene();

  // Draw trackball indicator
  if (mRotate && !mCapture)
    mTrackBall.draw(mWinWidth, mWinHeight);

  glfwSwapBuffers(mGlfwWindow);
}

//==============================================================================
void Viewer::renderScene()
{
}

//==============================================================================
void Viewer::startupGlfw()
{
  assert(mViewerMap.empty());

  dtmsg << "starting GLFW " << glfwGetVersionString();

  // Setup window
  glfwSetErrorCallback([](int error, const char* description) {
    dterr << "GLFW Error occured, Error ID: " << error
          << ", Description: " << description;
  });

  if (!glfwInit())
    throw std::runtime_error("Could not initialize GLFW.");

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#if __APPLE__
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

  gl3wInit();
}

//==============================================================================
void Viewer::shutdownGlfw()
{
  assert(mViewerMap.empty());

  glfwTerminate();
}

//==============================================================================
void Viewer::setCallbacks()
{
  assert(mGlfwWindow);

  glfwSetWindowSizeCallback(
      mGlfwWindow, [](GLFWwindow* window, int width, int height) {
        auto* viewer = findViewer(window);
        if (viewer)
          viewer->windowSizeCallback(width, height);
      });

  glfwSetCursorPosCallback(
      mGlfwWindow, [](GLFWwindow* window, double xpos, double ypos) {
        auto* viewer = findViewer(window);
        if (viewer)
          viewer->cursorPosCallback(xpos, ypos);
      });

  glfwSetMouseButtonCallback(
      mGlfwWindow, [](GLFWwindow* window, int button, int action, int mods) {
        auto* viewer = findViewer(window);
        if (viewer)
          viewer->mouseButtonCallback(button, action, mods);
      });

  glfwSetKeyCallback(
      mGlfwWindow,
      [](GLFWwindow* window, int key, int scancode, int action, int mods) {
        auto* viewer = findViewer(window);
        if (viewer)
          viewer->keyboardCallback(key, scancode, action, mods);
      });

  glfwSetCharCallback(
      mGlfwWindow, [](GLFWwindow* window, unsigned int codepoint) {
        auto* viewer = findViewer(window);
        if (viewer)
          viewer->charCallback(codepoint);
      });

  glfwSetDropCallback(
      mGlfwWindow, [](GLFWwindow* window, int count, const char** filenames) {
        auto* viewer = findViewer(window);
        if (viewer)
          viewer->dropCallback(count, filenames);
      });

  glfwSetScrollCallback(
      mGlfwWindow, [](GLFWwindow* window, double xoffset, double yoffset) {
        auto* viewer = findViewer(window);
        if (viewer)
          viewer->scrollCallback(xoffset, yoffset);
      });

  glfwSetFramebufferSizeCallback(
      mGlfwWindow, [](GLFWwindow* window, int width, int height) {
        auto* viewer = findViewer(window);
        if (viewer)
          viewer->windowSizeCallback(width, height);
      });
}

//==============================================================================
void Viewer::initLights()
{
  static float ambient[] = {0.2, 0.2, 0.2, 1.0};
  static float diffuse[] = {0.6, 0.6, 0.6, 1.0};
  static float front_mat_shininess[] = {60.0};
  static float front_mat_specular[] = {0.2, 0.2, 0.2, 1.0};
  static float front_mat_diffuse[] = {0.5, 0.28, 0.38, 1.0};
  static float lmodel_ambient[] = {0.2, 0.2, 0.2, 1.0};
  static float lmodel_twoside[] = {GL_FALSE};

  GLfloat position[] = {1.0, 0.0, 0.0, 0.0};
  GLfloat position1[] = {-1.0, 0.0, 0.0, 0.0};

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
