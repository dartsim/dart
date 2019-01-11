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
#include <dart/gui/osg/osg.hpp>

using namespace dart::dynamics;

int main()
{
  dart::simulation::WorldPtr world(new dart::simulation::World);

  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());

  tf.translation() = Eigen::Vector3d(4,-4,0);
  dart::gui::osg::InteractiveFramePtr frame(new dart::gui::osg::InteractiveFrame(
          Frame::World(), "interactive frame", tf, 2.0));
  world->addSimpleFrame(frame);

  tf.translation() = Eigen::Vector3d(-4,4,0);
  SimpleFramePtr draggable(new SimpleFrame(frame.get(), "draggable", tf));
  draggable->setShape(std::make_shared<BoxShape>(Eigen::Vector3d(1,1,1)));
  world->addSimpleFrame(draggable);

  tf.translation() = Eigen::Vector3d(8.0, 0.0, 0.0);
  SimpleFramePtr x_marker(new SimpleFrame(Frame::World(), "X", tf));
  std::shared_ptr<BoxShape> x_shape(
        new BoxShape(Eigen::Vector3d(0.2, 0.2, 0.2)));
  x_marker->setShape(x_shape);
  x_marker->getVisualAspect(true)->setColor(Eigen::Vector3d(0.9, 0.0, 0.0));
  world->addSimpleFrame(x_marker);

  tf.translation() = Eigen::Vector3d(0.0, 8.0, 0.0);
  SimpleFramePtr y_marker(new SimpleFrame(Frame::World(), "Y", tf));
  std::shared_ptr<BoxShape> y_shape(
        new BoxShape(Eigen::Vector3d(0.2, 0.2, 0.2)));
  y_marker->setShape(y_shape);
  y_marker->getVisualAspect(true)->setColor(Eigen::Vector3d(0.0, 0.9, 0.0));
  world->addSimpleFrame(y_marker);

  tf.translation() = Eigen::Vector3d(0.0, 0.0, 8.0);
  SimpleFramePtr z_marker(new SimpleFrame(Frame::World(), "Z", tf));
  std::shared_ptr<BoxShape> z_shape(
        new BoxShape(Eigen::Vector3d(0.2, 0.2, 0.2)));
  z_marker->setShape(z_shape);
  z_marker->getVisualAspect(true)->setColor(Eigen::Vector3d(0.0, 0.0, 0.9));
  world->addSimpleFrame(z_marker);


  ::osg::ref_ptr<dart::gui::osg::WorldNode> node = new dart::gui::osg::WorldNode(world);

  dart::gui::osg::Viewer viewer;
  viewer.addWorldNode(node);
  viewer.enableDragAndDrop(frame.get());
  viewer.enableDragAndDrop(draggable.get());

  viewer.addInstructionText("\nCtrl + Left-click: Rotate the box\n");
  std::cout << viewer.getInstructions() << std::endl;

  viewer.setUpViewInWindow(0, 0, 640, 480);

  viewer.getCameraManipulator()->setHomePosition(::osg::Vec3(20.0, 17.0, 17.0),
                                                 ::osg::Vec3(0.0, 0.0, 0.0),
                                                 ::osg::Vec3(0, 0, 1));
  viewer.setCameraManipulator(viewer.getCameraManipulator());

  viewer.run();
}
