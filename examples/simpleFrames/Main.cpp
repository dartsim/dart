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

#include <dart/dart.hpp>
#include <dart/gui/gui.hpp>

using namespace dart::dynamics;

int main(int argc, char* argv[])
{
  dart::simulation::WorldPtr myWorld(new dart::simulation::World);

  Eigen::Isometry3d tf1(Eigen::Isometry3d::Identity());
  tf1.translate(Eigen::Vector3d(0.1,-0.1,0));

  Eigen::Isometry3d tf2(Eigen::Isometry3d::Identity());
  tf2.translate(Eigen::Vector3d(0,0.1,0));
  tf2.rotate(Eigen::AngleAxisd(45.0*M_PI/180.0, Eigen::Vector3d(1,0,0)));

  Eigen::Isometry3d tf3(Eigen::Isometry3d::Identity());
  tf3.translate(Eigen::Vector3d(0,0,0.1));
  tf3.rotate(Eigen::AngleAxisd(60*M_PI/180.0, Eigen::Vector3d(0,1,0)));

  SimpleFramePtr F1(new SimpleFrame(Frame::World(), "F1", tf1));
  F1->setShape(std::shared_ptr<Shape>(
                             new BoxShape(Eigen::Vector3d(0.05, 0.05, 0.02))));
  SimpleFrame F2(F1.get(), "F2", tf2);
  F2.setShape(std::shared_ptr<Shape>(
                             new BoxShape(Eigen::Vector3d(0.05, 0.05, 0.02))));
  SimpleFrame F3(&F2, "F3", tf3);
  F3.setShape(std::shared_ptr<Shape>(
                             new BoxShape(Eigen::Vector3d(0.05, 0.05, 0.02))));

  // Note: Adding a Frame to the world will also cause all Entities that descend
  // from that Frame to be rendered.
  myWorld->addSimpleFrame(F1);

  SimpleFramePtr A(new SimpleFrame(Frame::World(), "A"));
  A->setShape(std::shared_ptr<Shape>(
                          new EllipsoidShape(Eigen::Vector3d(0.02,0.02,0.02))));
  SimpleFrame A1(A.get(), "A1", F1->getTransform(A.get()));
  A1.setShape(std::shared_ptr<Shape>(
                          new EllipsoidShape(Eigen::Vector3d(0.01,0.01,0.01))));
  SimpleFrame A2(A.get(), "A2", F2.getTransform(A.get()));
  A2.setShape(std::shared_ptr<Shape>(
                          new EllipsoidShape(Eigen::Vector3d(0.01,0.01,0.01))));
  SimpleFrame A3(A.get(), "A3", F3.getTransform(A.get()));
  A3.setShape(std::shared_ptr<Shape>(
                          new EllipsoidShape(Eigen::Vector3d(0.01,0.01,0.01))));

  myWorld->addSimpleFrame(A);

  SimpleFramePtr arrow(new SimpleFrame(Frame::World(), "arrow"));
  arrow->setShape(
        std::shared_ptr<Shape>(
          new ArrowShape(Eigen::Vector3d(0.1,-0.1, 0.0),
                         Eigen::Vector3d(0.1, 0.0, 0.0),
                         ArrowShape::Properties(0.002, 1.8),
                         Eigen::Vector4d(1.0, 0.5, 0.5, 1.0))));
  myWorld->addSimpleFrame(arrow);

  // CAREFUL: For a Frame that gets added to the world to be
  // rendered correctly, it must be a child of the World Frame
  // TODO(MXG): Fix this issue ^

  dart::gui::glut::SimWindow window;
  window.setWorld(myWorld);

  glutInit(&argc, argv);
  window.initWindow(640, 480, "Simple Frames");
  glutMainLoop();
}
