/*
 * Copyright (c) 2011-2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sumit Jain <sumit@cc.gatech.edu>
              Saul Reynolds-Haertle <saulrh@gatech.edu>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
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

#ifndef DART_GUI_GLUTWINDOW_H_
#define DART_GUI_GLUTWINDOW_H_

#include <vector>

#include "dart/renderer/LoadOpengl.h"
#include "dart/renderer/RenderInterface.h"
#include "dart/gui/lodepng.h"

namespace dart {
namespace gui {

/// \brief
class GlutWindow {
public:
  GlutWindow();
  virtual ~GlutWindow();

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

  static GlutWindow* current();
  static std::vector<GlutWindow*> mWindows;
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
  renderer::RenderInterface* mRI;
  std::vector<unsigned char> mScreenshotTemp;
  std::vector<unsigned char> mScreenshotTemp2;
};

}  // namespace gui
}  // namespace dart

#endif  // DART_GUI_GLUTWINDOW_H_
