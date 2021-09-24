/*
 * Copyright (c) 2011-2021, The DART development contributors
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

#ifndef DART_GUI_GLUT_WIN2D_HPP_
#define DART_GUI_GLUT_WIN2D_HPP_

#include "dart/gui/glut/Window.hpp"

namespace dart {
namespace gui {
namespace glut {

/// \brief
class Win2D : public glut::Window
{
public:
  /// \brief
  Win2D();

  /// \brief
  void resize(int _w, int _h) override;

  /// \brief
  void render() override;

  /// \brief
  void keyboard(unsigned char _key, int _x, int _y) override;

  /// \brief
  void click(int _button, int _state, int _x, int _y) override;

  /// \brief
  void drag(int _x, int _y) override;

  /// \brief
  virtual void initGL();

  /// \brief
  virtual void draw() = 0;

protected:
  /// \brief
  bool mTranslate;

  /// \brief
  double mTransX;

  /// \brief
  double mTransY;
};

} // namespace glut
} // namespace gui
} // namespace dart

#endif // DART_GUI_GLUT_WIN2D_HPP_
