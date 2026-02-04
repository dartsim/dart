/*
 * Copyright (c) 2011, The DART development contributors
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

#include <dart/gui/all.hpp>

#include <Eigen/Core>

#include <span>
#include <vector>

int main()
{
  // Provide the V-representation of a convex polyhedron. The visual will build
  // the convex hull automatically, so the vertices can be provided in any
  // order.
  std::vector<Eigen::Vector3d> vertices
      = {Eigen::Vector3d(-0.5, -0.5, 0.0),
         Eigen::Vector3d(0.5, -0.5, 0.2),
         Eigen::Vector3d(0.6, 0.5, 0.1),
         Eigen::Vector3d(-0.4, 0.6, 0.15),
         Eigen::Vector3d(-0.2, -0.2, 0.9),
         Eigen::Vector3d(0.35, -0.3, 0.8),
         Eigen::Vector3d(0.4, 0.35, 0.75),
         Eigen::Vector3d(-0.35, 0.4, 0.7)};

  dart::gui::Viewer viewer;

  // Add a grid for reference.
  auto grid = new dart::gui::GridVisual();
  grid->setOffset(Eigen::Vector3d::Zero());
  viewer.addAttachment(grid);

  // Attach the polyhedron visual and customize its colors.
  auto polyhedron = new dart::gui::PolyhedronVisual();
  polyhedron->setVertices(std::span<const Eigen::Vector3d>(vertices));
  polyhedron->setSurfaceColor(Eigen::Vector4d(0.1, 0.8, 0.6, 0.5));
  polyhedron->setWireframeColor(Eigen::Vector4d(0.05, 0.05, 0.05, 1.0));
  viewer.addAttachment(polyhedron);

  viewer.setUpViewInWindow(0, 0, 640, 480);
  viewer.getCameraManipulator()->setHomePosition(
      ::osg::Vec3(2.0, 2.0, 1.5),
      ::osg::Vec3(0.0, 0.0, 0.4),
      ::osg::Vec3(0.0, 0.0, 1.0));
  viewer.setCameraManipulator(viewer.getCameraManipulator());

  viewer.run();
}
