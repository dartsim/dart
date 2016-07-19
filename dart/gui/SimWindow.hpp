/*
 * Copyright (c) 2013-2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2013-2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
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
 *   * This code incorporates portions of Open Dynamics Engine
 *     (Copyright (c) 2001-2004, Russell L. Smith. All rights
 *     reserved.) and portions of FCL (Copyright (c) 2011, Willow
 *     Garage, Inc. All rights reserved.), which were released under
 *     the same BSD license as below
 *
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

#ifndef DART_GUI_SIMWINDOW_HPP_
#define DART_GUI_SIMWINDOW_HPP_

#include <vector>

#include <Eigen/Dense>

#include "dart/common/Deprecated.hpp"
#include "dart/gui/Win3D.hpp"
#include "dart/simulation/World.hpp"

namespace dart {
namespace gui {

class GraphWindow;

/// \brief
class SimWindow : public Win3D {
public:
  /// \brief
  SimWindow();

  /// \brief
  virtual ~SimWindow();

  /// \brief
  virtual void timeStepping();

  virtual void drawWorld() const;

  virtual void drawSkeletons() const;

  DART_DEPRECATED(6.0)
  virtual void drawSkels();

  DART_DEPRECATED(6.0)
  virtual void drawEntities();

  /// \brief
  void displayTimer(int _val) override;

  /// \brief
  void draw() override;

  /// \brief
  void keyboard(unsigned char _key, int _x, int _y) override;

  /// \brief
  void setWorld(dart::simulation::WorldPtr _world);

  /// \brief Save world in 'tempWorld.txt'
  void saveWorld();

  /// \brief Plot _data in a 2D window
  void plot(Eigen::VectorXd& _data);
//  bool isSimulating() const { return mSimulating; }

//  void setSimulatingFlag(int _flag) { mSimulating = _flag; }

protected:

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

  /// \brief
  simulation::WorldPtr mWorld;

  /// \brief
  int mPlayFrame;

  /// \brief
  bool mPlay;

  /// \brief
  bool mSimulating;

  /// If true, render point masses of soft bodies
  bool mShowPointMasses;

  /// If true, render markers
  bool mShowMarkers;

  /// \brief Array of graph windows
  std::vector<GraphWindow*> mGraphWindows;
};

}  // namespace gui
}  // namespace dart

#endif  // DART_GUI_SIMWINDOW_HPP_
