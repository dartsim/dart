/*
 * Copyright (c) 2011-2018, The DART development contributors
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

#ifndef DART_GUI_GLFW_WORLDWINDOW_HPP_
#define DART_GUI_GLFW_WORLDWINDOW_HPP_

#include "dart/gui/LoadOpengl.hpp"
#include "dart/gui/RenderInterface.hpp"
#include "dart/gui/glfw/BoxEntity.hpp"
#include "dart/gui/glfw/Window.hpp"
#include "dart/simulation/World.hpp"
#include "dart/gui/Program.hpp"

namespace dart {
namespace gui {
namespace glfw {

class WorldWindow : public Window
{
public:
  /// Constructor
  WorldWindow(
      simulation::WorldPtr world = nullptr,
      const std::string& title = "Nonamed Window",
      int width = 640,
      int height = 360,
      bool show = true);

  /// Destructor
  ~WorldWindow() override;

protected:
  void renderScene() override;

  virtual void renderWorld();

  virtual void drawSkeleton(
      const dynamics::Skeleton* skeleton,
      const Eigen::Vector4d& color = Eigen::Vector4d::Constant(0.5),
      bool useDefaultColor = true) const;

  virtual void drawEntity(
      const dynamics::Entity* entity,
      const Eigen::Vector4d& color = Eigen::Vector4d::Constant(0.5),
      bool useDefaultColor = true) const;

  virtual void drawBodyNode(
      const dynamics::BodyNode* bodyNode,
      const Eigen::Vector4d& color = Eigen::Vector4d::Constant(0.5),
      bool useDefaultColor = true,
      bool recursive = false) const;

  virtual void drawShapeFrame(
      const dynamics::ShapeFrame* shapeFrame,
      const Eigen::Vector4d& color = Eigen::Vector4d::Constant(0.5),
      bool useDefaultColor = true) const;

  virtual void drawShape(
      const dynamics::Shape* shape,
      const Eigen::Vector4d& color = Eigen::Vector4d::Constant(0.5)) const;

  virtual void drawPointMasses(
      const std::vector<dynamics::PointMass*> pointMasses,
      const Eigen::Vector4d& color = Eigen::Vector4d::Constant(0.5),
      bool useDefaultColor = true) const;

  virtual void drawMarker(
      const dynamics::Marker* marker,
      const Eigen::Vector4d& color = Eigen::Vector4d::Constant(0.5),
      bool useDefaultColor = true) const;

  Trackball mTrackBall;
  Eigen::Vector3d mTrans;
  Eigen::Vector3d mEye;
  Eigen::Vector3d mUp;
  float mZoom;
  float mPersp;

  bool mRotate;
  bool mTranslate;
  bool mZooming;

  bool mCapture;

  std::shared_ptr<RenderInterface> mRI;

  simulation::WorldPtr mWorld;

  int mPlayFrame;

  bool mPlay;

  bool mSimulating;

  /// If true, render point masses of soft bodies
  bool mShowPointMasses;

  /// If true, render markers
  bool mShowMarkers;

  std::unique_ptr<Program> mProgram;

  BoxEntity mBox;
};

} // namespace glfw
} // namespace gui
} // namespace dart

#endif // DART_GUI_GLFW_WORLDWINDOW_HPP_
