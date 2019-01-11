/*
 * Copyright (c) 2011-2019, The DART development contributors
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

#ifndef DART_GUI_GLUT_WINDOW_HPP_
#define DART_GUI_GLUT_WINDOW_HPP_

#include <vector>

#include "dart/gui/LoadOpengl.hpp"
#include "dart/gui/RenderInterface.hpp"

namespace dart {
namespace gui {
namespace glut {

/// \brief
class Window {
public:
  Window();
  virtual ~Window();

  /// \warning This function should be called once.
  virtual void initWindow(int _w, int _h, const char* _name);

  // callback functions
  static void reshape(int _w, int _h);
  static void keyEvent(unsigned char _key, int _x, int _y);
  static void specKeyEvent(int _key, int _x, int _y);
  static void mouseClick(int _button, int _state, int _x, int _y);
  static void mouseDrag(int _x, int _y);
  static void mouseMove(int _x, int _y);
  static void refresh();
  static void refreshTimer(int _val);
  static void runTimer(int _val);

  static Window* current();
  static std::vector<Window*> mWindows;
  static std::vector<int> mWinIDs;

protected:
  // callback implementation
  virtual void resize(int _w, int _h) = 0;
  virtual void render() = 0;
  virtual void keyboard(unsigned char _key, int _x, int _y);
  virtual void specKey(int _key, int _x, int _y);
  virtual void click(int _button, int _state, int _x, int _y);
  virtual void drag(int _x, int _y);
  virtual void move(int _x, int _y);
  virtual void displayTimer(int _val);
  virtual void simTimer(int _val);

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
  gui::RenderInterface* mRI;
  std::vector<unsigned char> mScreenshotTemp;
  std::vector<unsigned char> mScreenshotTemp2;
};

}  // namespace glut
}  // namespace gui
}  // namespace dart

#endif  // DART_GUI_GLUT_WINDOW_HPP_
