/*
 * Copyright (c) 2011-2017, The DART development contributors
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

#ifndef DART_EXAMPLE_OSG_OSGMINITAUR_MINITAURWIDGET_HPP_
#define DART_EXAMPLE_OSG_OSGMINITAUR_MINITAURWIDGET_HPP_

#include "dart/gui/osg/ImGuiWidget.hpp"
#include "dart/gui/osg/ImGuiViewer.hpp"

class MinitaurWorldNode;

class MinitaurWidget : public dart::gui::osg::ImGuiWidget
{
public:

  /// Constructor
  MinitaurWidget(dart::gui::osg::ImGuiViewer* viewer,
                      MinitaurWorldNode* node);

  // Documentation inherited
  void render() override;\

protected:

  void setGravity(float gravity);

  dart::gui::osg::ImGuiViewer* mViewer;

  MinitaurWorldNode* mNode;

  float mGuiGravityAcc;

  float mGravityAcc;

  bool mGuiHeadlights;

  /// Control mode value for GUI
  int mGuiControlMode;

  /// Actual control mode
  ///   - 0: No control
  ///   - 1: Short-stride walking control
  ///   - 1: Normal-stride walking control
  int mControlMode;

};

#endif // DART_EXAMPLE_OSG_OSGMINITAUR_MINITAURWIDGET_HPP_
