/*
 * Copyright (c) 2013-2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Can Erdogan
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

/**
 * @file MyWindow.hpp
 * @author Can Erdogan
 * @date Feb 02, 2013
 * @brief Simple example of a skeleton created from scratch.
 */

#ifndef APPS_HARDCODEDDESIGN_MYWINDOW_HPP_
#define APPS_HARDCODEDDESIGN_MYWINDOW_HPP_

#include <cstdio>
#include <cstdarg>

#include "dart/dart.hpp"
#include "dart/gui/gui.hpp"

class MyWindow : public dart::gui::Win3D {
public:
  /// \brief The constructor - set the position of the skeleton
  explicit MyWindow(dart::dynamics::SkeletonPtr _skel): Win3D(), skel(_skel) {
    mTrans[1] = 200.f;
    mZoom = 0.3;
  }

  /// \brief Draw the skeleton
  virtual void draw();

  /// \brief Move the joints with the {1,2,3} keys and '-' to change direction
  virtual void keyboard(unsigned char _key, int _x, int _y);

  /// \brief Hardcoded skeleton
  dart::dynamics::SkeletonPtr skel;
};

#endif  // APPS_HARDCODEDDESIGN_MYWINDOW_HPP_
