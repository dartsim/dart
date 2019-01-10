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

#ifndef DART_GUI_GLFUNCS_HPP_
#define DART_GUI_GLFUNCS_HPP_

#include <string>
#include <Eigen/Eigen>
#include "dart/common/Deprecated.hpp"

namespace dart {
namespace gui {

/// \deprecated Deprecated in 6.6. Please use
/// dart::gui::glut::drawStringOnScreen() instead in
/// dart/gui/glut/GLUTFuncs.hpp file.
DART_DEPRECATED(6.6)
void drawStringOnScreen(float _x, float _y, const std::string& _s,
                        bool _bigFont = true);

/// \brief
void drawArrow3D(const Eigen::Vector3d& _pt, const Eigen::Vector3d& _dir,
                 const double _length, const double _thickness,
                 const double _arrowThickness = -1);

/// \brief
void drawArrow2D(const Eigen::Vector2d& _pt, const Eigen::Vector2d& _vec,
                 double _thickness);

/// \brief
void drawProgressBar(int _currFrame, int _totalFrame);

// BOOL screenShot(FREE_IMAGE_FORMAT fif, int w, int h, char *fname,
//                bool _antialias);
// BOOL screenShot(FREE_IMAGE_FORMAT fif, int x, int y, int w, int h,
//                 char *fname, bool _antialias);
// bool screenShot(int w, int h, char *fname, bool _antialias = false);

// TODO(Unknown): freeimage

}  // namespace gui
}  // namespace dart

#endif  // DART_GUI_GLFUNCS_HPP_
