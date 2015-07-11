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

using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::utils;
using namespace dart::math;

Eigen::Vector3d clamped_mag(const Eigen::Vector3d& v, double clamp=0.1)
{
  if(v.norm() > clamp)
    return v.normalized()*clamp;
  return v;
}

void clamp_components(Eigen::VectorXd& v, double clamp=0.1)
{
  clamp = fabs(clamp);
  for(int i=0; i<v.size(); ++i)
  {
    if(fabs(v[i]) > clamp)
      v[i] = v[i] > 0? clamp : -clamp;
  }
}

class TeleoperationWorld : public osgDart::WorldNode
{
public:

  TeleoperationWorld(WorldPtr _world, SkeletonPtr _robot)
    : osgDart::WorldNode(_world),
      mRobot(_robot)
  {
    // Do nothing
  }

  void customPreRefresh() override
  {
    for(size_t i=0; i < mRobot->getNumEndEffectors(); ++i)
    {
      const InverseKinematicsPtr& ik = mRobot->getEndEffector(i)->getIK();
      if(ik)
        ik->solve();
    }
  }

protected:

  SkeletonPtr mRobot;
};

int main()
{
  WorldPtr world(new World);

  SkeletonPtr ground = Skeleton::create("ground");
  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  tf.translation() = Eigen::Vector3d(0,0,-0.95);
  WeldJoint::Properties joint;
  joint.mT_ParentBodyToJoint = tf;
  ground->createJointAndBodyNodePair<WeldJoint>(nullptr, joint);
  ShapePtr groundShape = std::make_shared<BoxShape>(Eigen::Vector3d(10,10,0.01));
  ground->getBodyNode(0)->addVisualizationShape(groundShape);
  ground->getBodyNode(0)->addCollisionShape(groundShape);

  DartLoader urdf;
  SkeletonPtr atlas =
      urdf.parseSkeleton(DART_DATA_PATH"sdf/atlas/atlas_v3_no_head.urdf");

  world->addSkeleton(atlas);
  world->addSkeleton(ground);

  for(size_t i=0; i<atlas->getNumBodyNodes(); ++i)
    std::cout << atlas->getBodyNode(i)->getName() << std::endl;

  EndEffector* l_hand = atlas->getBodyNode("l_hand")->createEndEffector();
  l_hand->setName("l_hand");

  osgDart::InteractiveFramePtr l_target(new osgDart::InteractiveFrame(
                                          Frame::World(), "l_target"));
  l_target->setTransform(l_hand->getTransform());
  l_hand->getIK(true)->setTarget(l_target);
  world->addSimpleFrame(l_target);

  osg::ref_ptr<osgDart::WorldNode> node = new TeleoperationWorld(world, atlas);

  osgDart::Viewer viewer;
  viewer.addWorldNode(node);

  viewer.enableDragAndDrop(l_target.get());

  std::cout << viewer.getInstructions() << std::endl;

  viewer.setUpViewInWindow(0, 0, 640, 480);

  viewer.run();
}
