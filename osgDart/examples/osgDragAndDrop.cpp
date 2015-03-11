/*
 * Copyright (c) 2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Michael X. Grey <mxgrey@gatech.edu>
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

#include "osgDart/osgDart.h"

#include "dart/dart.h"

class ConstraintEventHandler : public osgGA::GUIEventHandler
{
public:

  ConstraintEventHandler(osgDart::DragAndDrop* dnd = nullptr)
    : mDnD(dnd)
  {
    clearConstraints();
    if(mDnD)
      mDnD->unconstrain();
  }

  void clearConstraints()
  {
    for(size_t i=0; i<3; ++i)
      mConstrained[i] = false;
  }

  virtual bool handle(const osgGA::GUIEventAdapter& ea,
                      osgGA::GUIActionAdapter&) override
  {
    if(nullptr == mDnD)
    {
      clearConstraints();
      return false;
    }

    bool handled = false;
    switch(ea.getEventType())
    {
      case osgGA::GUIEventAdapter::KEYDOWN:
      {
        switch(ea.getUnmodifiedKey())
        {
          case '1':
            mConstrained[0] = true;
            handled = true;
            break;
          case '2':
            mConstrained[1] = true;
            handled = true;
            break;
          case '3':
            mConstrained[2] = true;
            handled = true;
            break;
        }
        break;
      }

      case osgGA::GUIEventAdapter::KEYUP:
      {
        switch(ea.getUnmodifiedKey())
        {
          case '1':
            mConstrained[0] = false;
            handled = true;
            break;
          case '2':
            mConstrained[1] = false;
            handled = true;
            break;
          case '3':
            mConstrained[2] = false;
            handled = true;
            break;
        }
        break;
      }

      default:
        return false;
    }

    if(!handled)
      return handled;

    size_t constraintDofs = 0;
    for(size_t i=0; i<3; ++i)
      if(mConstrained[i])
        ++constraintDofs;

    if(constraintDofs==0 || constraintDofs==3)
    {
      mDnD->unconstrain();
    }
    else if(constraintDofs == 1)
    {
      Eigen::Vector3d v(Eigen::Vector3d::Zero());
      for(size_t i=0; i<3; ++i)
        if(mConstrained[i])
          v[i] = 1.0;

      mDnD->constrainToLine(v);
    }
    else if(constraintDofs == 2)
    {
      Eigen::Vector3d v(Eigen::Vector3d::Zero());
      for(size_t i=0; i<3; ++i)
        if(!mConstrained[i])
          v[i] = 1.0;

      mDnD->constrainToPlane(v);
    }

    return handled;
  }

  bool mConstrained[3];

  dart::sub_ptr<osgDart::DragAndDrop> mDnD;
};

using namespace dart::dynamics;

int main()
{
  dart::simulation::World* world = new dart::simulation::World;

  SimpleFrame draggable(Frame::World(), "draggable");
  draggable.addVisualizationShape(new BoxShape(Eigen::Vector3d(1,1,1)));
  world->addFrame(&draggable);

  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());

  tf.translation() = Eigen::Vector3d(8.0, 0.0, 0.0);
  SimpleFrame x_marker(Frame::World(), "X", tf);
  BoxShape* x_shape = new BoxShape(Eigen::Vector3d(0.2, 0.2, 0.2));
  x_shape->setColor(Eigen::Vector3d(0.9, 0.0, 0.0));
  x_marker.addVisualizationShape(x_shape);
  world->addFrame(&x_marker);

  tf.translation() = Eigen::Vector3d(0.0, 8.0, 0.0);
  SimpleFrame y_marker(Frame::World(), "Y", tf);
  BoxShape* y_shape = new BoxShape(Eigen::Vector3d(0.2, 0.2, 0.2));
  y_shape->setColor(Eigen::Vector3d(0.0, 0.9, 0.0));
  y_marker.addVisualizationShape(y_shape);
  world->addFrame(&y_marker);

  tf.translation() = Eigen::Vector3d(0.0, 0.0, 8.0);
  SimpleFrame z_marker(Frame::World(), "Z", tf);
  BoxShape* z_shape = new BoxShape(Eigen::Vector3d(0.2, 0.2, 0.2));
  z_shape->setColor(Eigen::Vector3d(0.0, 0.0, 0.9));
  z_marker.addVisualizationShape(z_shape);
  world->addFrame(&z_marker);


  osg::ref_ptr<osgDart::WorldNode> node = new osgDart::WorldNode(world);

  osgDart::Viewer viewer;
  viewer.addWorldNode(node);
  osgDart::DragAndDrop* dnd = viewer.enableDragAndDrop(&draggable);
  viewer.addEventHandler(new ConstraintEventHandler(dnd));

  std::cout << viewer.getInstructions() << std::endl;

  viewer.setUpViewInWindow(0, 0, 640, 480);

  viewer.getCameraManipulator()->setHomePosition(osg::Vec3(20.0, 17.0, 17.0),
                                                 osg::Vec3(0.0, 0.0, 0.0),
                                                 osg::Vec3(0, 0, 1));
  viewer.setCameraManipulator(viewer.getCameraManipulator());

  viewer.run();
}
