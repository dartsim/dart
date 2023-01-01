/*
 * Copyright (c) 2011-2022, The DART development contributors
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

#ifndef EXAMPLES_JOINTCONSTRAINTS_MYWINDOW_HPP_
  #define EXAMPLES_JOINTCONSTRAINTS_MYWINDOW_HPP_

  #include "Controller.hpp"

  #include <dart/gui/gui.hpp>

  #include <dart/dart.hpp>

  #include <Eigen/Dense>
  #include <stdarg.h>

class MyWindow : public dart::gui::glut::SimWindow
{
public:
  MyWindow() : SimWindow()
  {
    mForce = Eigen::Vector3d::Zero();
    mController = nullptr;
    mWeldJoint = nullptr;
    mImpulseDuration = 0;
    mHarnessOn = false;
  }
  virtual ~MyWindow() {}

  void timeStepping() override;
  void drawSkels() override;
  //  void displayTimer(int _val) override;
  //  void draw() override;
  void keyboard(unsigned char key, int x, int y) override;

  void setController(Controller* _controller)
  {
    mController = _controller;
  }

private:
  Eigen::Vector3d mForce;
  Controller* mController;
  dart::constraint::WeldJointConstraintPtr mWeldJoint;
  int mImpulseDuration;
  bool mHarnessOn;
};

#endif // EXAMPLES_JOINTCONSTRAINTS_MYWINDOW_HPP_
