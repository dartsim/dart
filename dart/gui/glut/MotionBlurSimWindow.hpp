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

//
//  MotionBlurSimWindow.hpp
//  dart
//
//  Created by Dong Xu on 1/22/17.
//
//

#ifndef DART_GUI_GLUT_MOTIONBLURSIMWINDOW_HPP_
#define DART_GUI_GLUT_MOTIONBLURSIMWINDOW_HPP_

#include <vector>
#include <Eigen/Dense>

#include "dart/gui/glut/SimWindow.hpp"

namespace dart {
namespace gui {
namespace glut {

class MotionBlurSimWindow : public glut::SimWindow
{
public:
  /// \brief
  MotionBlurSimWindow();

  /// \brief
  virtual ~MotionBlurSimWindow();
    
  // Set the Quality of Motion Blur
  // Default is 5 (record position of every frame)
  // int from 0 (No motion blur) - 5 (Highest)
  // The function takes value smaller than 0 as 0, larger than 5 as 5
  void setMotionBlurQuality(int _val);

  // Override the render function in dart/gui/Win3D.hpp
  // To draw the motion image
  // Render function is called once per GUI display time
  // but in MotionBlurSimWindow, draw function will run in motion blur frequency
  void render() override;

  // Override the display timer,
  // Move the part of "step" in world function to the render function
  void displayTimer(int _val) override;
    
protected:
  // Determines the frequency of the motion blur
  // Default is 1, which means motion blur effect has the highest quality
  // When set to m, motion blur record data every m frames
  int mMotionBlurFrequency;

}; // End of Class Definition

} // namespace glut
} // namespace gui
} // namespace dart

#endif // DART_GUI_GLUT_MOTIONBLURSIMWINDOW_HPP_
