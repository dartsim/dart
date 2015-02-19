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

#include <osgViewer/Viewer>

#include "osgDart/WorldNode.h"

#include "dart/simulation/World.h"
#include "dart/dynamics/SimpleFrame.h"
#include "dart/dynamics/Shape.h"
#include "dart/dynamics/BoxShape.h"

using namespace dart::dynamics;

class CustomWorldNode : public osgDart::WorldNode
{
public:

  CustomWorldNode(dart::simulation::World* _world)
    : WorldNode(_world), F(nullptr), T(nullptr), S(nullptr), t(0.0) { }

  void customUpdate() override
  {
    t += 0.002;

    if(F)
    {
      Eigen::Isometry3d tf = F->getRelativeTransform();
      tf.rotate(Eigen::AngleAxisd(0.005, Eigen::Vector3d(0,0,1)));
      tf.translation() = Eigen::Vector3d(0,0,0.5)*sin(t);
      F->setRelativeTransform(tf);
    }

    if(T)
    {
      Eigen::Isometry3d tf = T->getLocalTransform();
      tf.rotate(Eigen::AngleAxisd(0.01, Eigen::Vector3d(1,0,0)));
      T->setLocalTransform(tf);
    }

    if(S)
    {
      Eigen::Vector3d scale(0.15,0.15,0.15);
      scale[0] *= 2*fabs(cos(5*t))+0.05;
      scale[1] *= 2*fabs(sin(5*t))+0.05;
      S->setSize(scale);
      S->setColor(scale);
    }
  }

  SimpleFrame* F; // Change the transform of this Frame over time
  Shape* T;       // Change the transform of this Shape over time
  BoxShape* S;    // Change the scaling and color of this Shape over time

  double t;

};

int main()
{
  dart::simulation::World* myWorld = new dart::simulation::World;

  SimpleFrame box1(Frame::World(), "box1");
  box1.addVisualizationShape(new BoxShape(Eigen::Vector3d(0.1,0.1,0.1)));
  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  tf.translate(Eigen::Vector3d(0,-0.5,0.5));
  box1.setRelativeTransform(tf);

  Entity box2(Frame::World(), "box2", false);
  BoxShape* shape2 = new BoxShape(Eigen::Vector3d(0.2, 0.2, 0.2));
  tf.translate(Eigen::Vector3d(0.5,0,0));
  shape2->setLocalTransform(tf);
  shape2->setColor(Eigen::Vector3d(1,0,0));
  shape2->setDataVariance(Shape::DYNAMIC_TRANSFORM);
  box2.addVisualizationShape(shape2);

  myWorld->addEntity(&box1);
  myWorld->addEntity(&box2);

  SimpleFrame F(Frame::World(), "F");
  F.addVisualizationShape(new BoxShape(Eigen::Vector3d(0.1,0.3,0.1)));

  Entity box3(&F, "box3", false);
  BoxShape* shape3 = new BoxShape(Eigen::Vector3d(0.05,0.05,0.05));
  shape3->setLocalTransform(tf);
  shape3->setColor(Eigen::Vector3d(0,1,0));
  box3.addVisualizationShape(shape3);

  Entity box4(&F, "box4", false);
  BoxShape* shape4 = new BoxShape(Eigen::Vector3d(0.15,0.15,0.15));
  shape4->setLocalTransform(tf.inverse());
  shape4->setDataVariance(Shape::DYNAMIC_SCALING | Shape::DYNAMIC_COLOR);
  box4.addVisualizationShape(shape4);

  myWorld->addFrame(&F);

  osg::ref_ptr<CustomWorldNode> node = new CustomWorldNode(myWorld);
  node->F = &F;
  node->T = shape2;
  node->S = shape4;

  osgViewer::Viewer viewer;
  viewer.getCamera()->setClearColor(osg::Vec4(0.9,0.9,0.9,1));
  viewer.setSceneData(node);

  viewer.setUpViewInWindow(0, 0, 640, 480);
  viewer.realize();

  viewer.getCamera()->getOrCreateStateSet()->setGlobalDefaults();

  viewer.run();
}
