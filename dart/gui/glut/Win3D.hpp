/*
 * Copyright (c) 2011-2025, The DART development contributors
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

#ifndef DART_GUI_GLUT_WIN3D_HPP_
#define DART_GUI_GLUT_WIN3D_HPP_

#include <dart/gui/Trackball.hpp>
#include <dart/gui/glut/Window.hpp>

#include <Eigen/Eigen>

namespace dart {
namespace gui {
namespace glut {

class Win3D : public glut::Window
{
public:
  Win3D();

  void initWindow(int _w, int _h, const char* _name) override;
  void resize(int _w, int _h) override;
  void render() override;

  void keyboard(unsigned char _key, int _x, int _y) override;
  void click(int _button, int _state, int _x, int _y) override;
  void drag(int _x, int _y) override;

  virtual void initGL();
  virtual void initLights();

  virtual void draw() = 0;

protected:
  Trackball mTrackBall;
  Eigen::Vector3d mTrans;
  Eigen::Vector3d mEye;
  Eigen::Vector3d mUp;
  float mZoom;
  float mPersp;

  bool mRotate;
  bool mTranslate;
  bool mZooming;
};

} // namespace glut
} // namespace gui
} // namespace dart

#endif // DART_GUI_GLUT_WIN3D_HPP_
