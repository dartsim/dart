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

#ifndef EXAMPLES_RIGIDSHAPES_MYWINDOW_HPP_
#define EXAMPLES_RIGIDSHAPES_MYWINDOW_HPP_

#include <dart/dart.hpp>
#include <dart/gui/gui.hpp>

/// MyWindow
class MyWindow : public dart::gui::glut::SimWindow
{
public:
  /// Constructor
  MyWindow();

  /// Destructor
  virtual ~MyWindow();

  // Documentation inherited
  void timeStepping() override;

  // Documentation inherited
  void keyboard(unsigned char key, int x, int y) override;

  // Documentation inherited
  void drawWorld() const override;

  /// Spawn a box into the world
  void spawnBox(
      const Eigen::Isometry3d& _T,
      const Eigen::Vector3d& _size = Eigen::Vector3d(0.1, 0.1, 0.1),
      double _mass = 10);

  /// Spawn a ellipsoid into the world
  void spawnEllipsoid(
      const Eigen::Isometry3d& _T,
      const Eigen::Vector3d& _radii = Eigen::Vector3d(0.1, 0.1, 0.1),
      double _mass = 10);

  /// Spawn a cylinder into the world
  void spawnCylinder(
      const Eigen::Isometry3d& _T,
      double _radius = 0.05,
      double _height = 0.10,
      double _mass = 10);
};

#endif  // EXAMPLES_RIGIDSHAPES_MYWINDOW_HPP_
