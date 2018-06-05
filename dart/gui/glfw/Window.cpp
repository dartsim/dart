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

#include <GLFW/glfw3.h>

#ifdef _WIN32
#include <direct.h>
#include <sys/stat.h>
#include <sys/types.h>
#else
#include <dirent.h>
#include <sys/stat.h>
#include <sys/types.h>
#endif
#include <cstdio>
#include <iostream>
#include <vector>
#include <thread>

#include "dart/common/Console.hpp"
#include "dart/gui/GLFuncs.hpp"
#include "dart/gui/OpenGLRenderInterface.hpp"
#include "dart/external/lodepng/lodepng.h"

namespace dart {
namespace gui {
namespace glfw {

//==============================================================================
bool Viewer::mMainloopActive = false;
std::unordered_map<GLFWwindow*, Viewer*> Viewer::mViewerMap;

//==============================================================================
Viewer::Viewer()
  : mIsVisible(true)
{
  mWinWidth = 0;
  mWinHeight = 0;
  mMouseX = 0;
  mMouseY = 0;
  mDisplayTimeout = 1000.0 / 30.0;
  mMouseDown = false;
  mMouseDrag = false;
  mCapture = false;
  mBackground[0] = 0.3;
  mBackground[1] = 0.3;
  mBackground[2] = 0.3;
  mBackground[3] = 1.0;
  mRI = nullptr;

  if (mViewerMap.empty())
    startupGlfw();
}

//==============================================================================
Viewer::~Viewer()
{
  mViewerMap.erase(mGlfwWindow);
  if (mViewerMap.empty())
    shutdownGlfw();
}

//==============================================================================
void Viewer::initWindow(int w, int h, const char* name)
{
  // TODO(JS): Improve
  glfwSetErrorCallback([](int error,const char* description){fprintf(stderr, "Error: %s\n", description);});

  mWinWidth = w;
  mWinHeight = h;

//  glutInitDisplayMode(
//      GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA | GLUT_MULTISAMPLE | GLUT_ACCUM);
//  glutInitWindowPosition(150, 100);

  mGlfwWindow = glfwCreateWindow(w, h, name, nullptr, nullptr);
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

//  glfwDisplayFunc(refresh);
//  glfwReshapeFunc(reshape);
  glfwSetKeyCallback(mGlfwWindow, onKeyEvent);
//  glfwSpecialFunc(specKeyEvent);
//  glfwMouseFunc(mouseClick);
//  glfwMotionFunc(mouseDrag);
//  glfwPassiveMotionFunc(mouseMove);

  mRI.reset(new gui::OpenGLRenderInterface());
  mRI->initialize();
// glutTimerFunc(mDisplayTimeout, refreshTimer, 0);
// glutTimerFunc(mDisplayTimeout, runTimer, 0);

#ifndef _WIN32
  glDisable(GL_MULTISAMPLE);
#endif
  // TODO: Disabled use of GL_MULTISAMPLE for Windows. Please see #411 for the
  // detail.

//  glutTimerFunc(mDisplayTimeout, refreshTimer, 0);
  // Note: We book the timer id 0 for the main rendering purpose.
}

////==============================================================================
//void Window::reshape(int w, int h)
//{
////  current(window)->mScreenshotTemp = std::vector<unsigned char>(w * h * 4);
////  current(window)->mScreenshotTemp2 = std::vector<unsigned char>(w * h * 4);
//  current(window)->resize(w, h);
//}

//==============================================================================
void Viewer::onKeyEvent(GLFWwindow* window, int key, int scancode, int action, int mods)
{
  current(window)->keyboard(window, key, scancode, action, mods);
}

////==============================================================================
//void Window::specKeyEvent(int key, int x, int y)
//{
//  current(window)->specKey(key, x, y);
//}

////==============================================================================
//void Window::mouseClick(int button, int state, int x, int y)
//{
//  current(window)->click(button, state, x, y);
//}

////==============================================================================
//void Window::mouseDrag(int x, int y)
//{
//  current(window)->drag(x, y);
//}

////==============================================================================
//void Window::mouseMove(int x, int y)
//{
//  current(window)->move(x, y);
//}

////==============================================================================
//void Window::refresh()
//{
//  current(window)->render();
//}

////==============================================================================
//void Window::refreshTimer(int val)
//{
//  current(window)->displayTimer(val);
//}

////==============================================================================
//void Window::runTimer(int val)
//{
//  current(window)->simTimer(val);
//}

//==============================================================================
bool Viewer::screenshot()
{
//  static int count = 0;
//  const char directory[8] = "frames";
//  const char fileBase[8] = "Capture";
//  char fileName[32];

//  // create frames directory if not exists
//  using Stat = struct stat;
//  Stat buff;

//#ifdef _WIN32
//#define __S_ISTYPE(mode, mask) (((mode)&_S_IFMT) == (mask))
//#define S_ISDIR(mode) __S_ISTYPE((mode), _S_IFDIR)
//  if (stat(directory, &buff) != 0)
//    _mkdir(directory);
//#else
//  if (stat(directory, &buff) != 0)
//    mkdir(directory, 0777);
//#endif

//  if (!S_ISDIR(buff.st_mode))
//  {
//    dtwarn << "[GlutWindow::screenshot] 'frames' is not a directory, "
//           << "cannot write a screenshot\n";
//    return false;
//  }

//// png
//#ifdef _WIN32
//  _snprintf(
//      fileName,
//      sizeof(fileName),
//      "%s%s%s%.4d.png",
//      directory,
//      "\\",
//      fileBase,
//      count++);
//#else
//  std::snprintf(
//      fileName,
//      sizeof(fileName),
//      "%s%s%s%.4d.png",
//      directory,
//      "/",
//      fileBase,
//      count++);
//#endif
//  int tw = glutGet(GLUT_WINDOW_WIDTH);
//  int th = glutGet(GLUT_WINDOW_HEIGHT);

//  glReadPixels(0, 0, tw, th, GL_RGBA, GL_UNSIGNED_BYTE, &mScreenshotTemp[0]);

//  // reverse temp2 temp1
//  for (int row = 0; row < th; row++)
//  {
//    memcpy(
//        &mScreenshotTemp2[row * tw * 4],
//        &mScreenshotTemp[(th - row - 1) * tw * 4],
//        tw * 4);
//  }

//  unsigned result = lodepng::encode(fileName, mScreenshotTemp2, tw, th);

//  // if there's an error, display it
//  if (result)
//  {
//    std::cout << "lodepng error " << result << ": "
//              << lodepng_error_text(result) << std::endl;
//    return false;
//  }
//  else
//  {
//    std::cout << "wrote screenshot " << fileName << "\n";
//    return true;
//  }
}

//==============================================================================
Viewer* Viewer::current(GLFWwindow* window)
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
    throw std::runtime_error("[Viewer::runMainLoop] Attempting to run the main "
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
void Viewer::startupGlfw()
{
  assert(mViewerMap.empty());

  dtmsg << "starting GLFW " << glfwGetVersionString();

  // Setup window
  glfwSetErrorCallback([](int error, const char* description) {
    std::cerr << "GLFW Error occured, Error ID: " << error
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
}

//==============================================================================
void Viewer::shutdownGlfw()
{
  assert(mViewerMap.empty());

  glfwTerminate();
}

//==============================================================================
void Viewer::keyboard(GLFWwindow* window, int key, int /*scancode*/, int action, int /*mods*/)
{
  if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
      glfwSetWindowShouldClose(window, GLFW_TRUE);
}

//==============================================================================
void Viewer::specKey(int /*key*/, int /*x*/, int /*y*/)
{
  // Do nothing
}

//==============================================================================
void Viewer::click(int /*button*/, int /*state*/, int /*x*/, int /*y*/)
{
  // Do nothing
}

//==============================================================================
void Viewer::drag(int /*x*/, int /*y*/)
{
  // Do nothing
}

//==============================================================================
void Viewer::move(int /*x*/, int /*y*/)
{
  // Do nothing
}

//==============================================================================
void Viewer::displayTimer(int val)
{
//  glutPostRedisplay();
//  glutTimerFunc(mDisplayTimeout, refreshTimer, val);
}

//==============================================================================
void Viewer::simTimer(int /*val*/)
{
  // Do nothing
}

} // namespace glfw
} // namespace gui
} // namespace dart
