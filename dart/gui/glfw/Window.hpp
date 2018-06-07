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

#ifndef DART_GUI_GLFW_VIEWER_HPP_
#define DART_GUI_GLFW_VIEWER_HPP_

#include <unordered_map>
#include <Eigen/Dense>
#include <GLFW/glfw3.h>
#include "dart/gui/Trackball.hpp"

namespace dart {
namespace gui {
namespace glfw {

class Viewer
{
public:
  /// Constructor
  Viewer(
      const std::string& title = "Nonamed Window",
      int width = 640,
      int height = 360,
      bool show = true);

  /// Destructor
  virtual ~Viewer();

  static void runAllViewers(std::size_t refresh = 50u);

  //----------------------------------------------------------------------------
  /// \{ \name Properties
  //----------------------------------------------------------------------------

  /// Sets the title that will be displayed at the window bar.
  void setTitle(const std::string& title);

  /// Sets the size of this viewer.
  void setSize(const Eigen::Vector2i& size);

  /// Sets the size of this viewer.
  void setSize(int width, int height);

  /// Returns the size of this viewer.
  Eigen::Vector2i getSize() const;

  /// Sets the visibility.
  void setVisible(bool visible);

  /// Returns true if this Viewer is shown.
  bool isVisible() const;

  /// Shows this Viewer. Equivalent to setVisible(true).
  void show();

  /// Hides this Viewer. Equivalent to setVisible(false).
  void hide();

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name GLFW event handlers
  //----------------------------------------------------------------------------

  /// Windows resize callback function.
  virtual void windowSizeCallback(int width, int height);

  /// Mouse move callback function.
  virtual void cursorPosCallback(double xpos, double ypos);

  /// Mouse button callback function.
  virtual void mouseButtonCallback(int button, int action, int mods);

  /// Keyboard callback function.
  virtual void keyboardCallback(int key, int scancode, int action, int mods);

  /// Character callback function, which is called when a Unicode character is
  /// input.
  virtual void charCallback(unsigned int codepoint);

  /// File drop callback of the specified window, which is called when one or
  /// more dragged files are dropped on the window.
  virtual void dropCallback(int count, const char** filenames);

  /// Scroll callback of the specified window, which is called when a scrolling
  /// device is used, such as a mouse wheel or scrolling area of a touchpad.
  virtual void scrollCallback(double xoffset, double yoffset);

  /// \}

protected:
  void render();
  void renderScene();

  static Viewer* findViewer(GLFWwindow* window);

  GLFWwindow* mGlfwWindow;

  bool mIsVisible;

  Eigen::Vector4f mClearColor;

  Trackball mTrackBall;
  Eigen::Vector3d mTrans;
  Eigen::Vector3d mEye;
  Eigen::Vector3d mUp;
  float mZoom;
  float mPersp;

  bool mRotate;
  bool mTranslate;
  bool mZooming;

  int mWinWidth;
  int mWinHeight;
  bool mCapture;

private:
  static void startupGlfw();
  static void shutdownGlfw();
  void setCallbacks();
  void initLights();
  static std::unordered_map<GLFWwindow*, Viewer*> mViewerMap;
  static bool mMainloopActive;
};

} // namespace glfw
} // namespace gui
} // namespace dart

#endif // DART_GUI_GLFW_WINDOW_HPP_
