/*
 * Copyright (c) 2011-2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2011-2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
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

#ifndef DART_GUI_GLFW_WINDOW_HPP_
#define DART_GUI_GLFW_WINDOW_HPP_

#include <vector>
#include <unordered_map>
#include <GLFW/glfw3.h>
#include "dart/gui/LoadOpengl.hpp"
#include "dart/gui/RenderInterface.hpp"

namespace dart {
namespace gui {
namespace glfw {

class Viewer
{
public:
  Viewer();
  virtual ~Viewer();

  /// \warning This function should be called once.
  virtual void initWindow(int w, int h, const char* name);

  // callback functions
//  static void reshape(int w, int h);
  static void onKeyEvent(GLFWwindow* window, int key, int scancode, int action, int mods);
//  static void specKeyEvent(int key, int x, int y);
//  static void mouseClick(int button, int state, int x, int y);
//  static void mouseDrag(int x, int y);
//  static void mouseMove(int x, int y);
//  static void refresh();
//  static void refreshTimer(int val);
//  static void runTimer(int val);

  static Viewer* current(GLFWwindow* window);
//  static std::vector<Window*> mWindows;
//  static std::vector<GLFWwindow*> mWinIDs;
  static bool mMainloopActive;
  static std::unordered_map<GLFWwindow*, Viewer*> mViewerMap;

  static void runAllViewers(std::size_t refresh = 50u);

  void setVisible(bool visible);

  void show();

  void hide();

  bool isVisible() const;

protected:
  void startupGlfw();
  void shutdownGlfw();
  // callback implementation
//  virtual void resize(int w, int h) = 0;
//  virtual void render() = 0;
  virtual void keyboard(GLFWwindow* window, int key, int scancode, int action, int mods);
  virtual void specKey(int key, int x, int y);
  virtual void click(int button, int state, int x, int y);
  virtual void drag(int x, int y);
  virtual void move(int x, int y);
  virtual void displayTimer(int val);
  virtual void simTimer(int val);

  virtual bool screenshot();

  int mWinWidth;
  int mWinHeight;
  int mMouseX;
  int mMouseY;
  double mDisplayTimeout;
  bool mMouseDown;
  bool mMouseDrag;
  bool mCapture;
  double mBackground[4];
  GLFWwindow* mGlfwWindow;
  std::unique_ptr<gui::RenderInterface> mRI;
  std::vector<unsigned char> mScreenshotTemp;
  std::vector<unsigned char> mScreenshotTemp2;

  bool mIsVisible;
};

} // namespace glfw
} // namespace gui
} // namespace dart

#endif // DART_GUI_GLFW_WINDOW_HPP_
