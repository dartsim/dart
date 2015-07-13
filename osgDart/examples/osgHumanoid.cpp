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

class RelaxedPose : public dart::optimizer::Function
{
public:

  static const double DefaultObjectiveWeight;

  RelaxedPose(const Eigen::VectorXd& _pose)
    : mObjectiveWeight(DefaultObjectiveWeight),
      mPose(_pose)
  {
    // Do nothing
  }

  double eval(const Eigen::VectorXd& _x) override
  {
    return mObjectiveWeight * 0.5 * (_x - mPose).dot(_x - mPose);
  }

  void evalGradient(const Eigen::VectorXd& _x,
                    Eigen::Map<Eigen::VectorXd> _grad) override
  {
    _grad = mObjectiveWeight * (_x - mPose);
  }

  double mObjectiveWeight;

protected:

  Eigen::VectorXd mPose;
};

const double RelaxedPose::DefaultObjectiveWeight = 2.0;

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
//    mRobot->getIK(true)->solve();
  }

protected:

  SkeletonPtr mRobot;
};

class InputHandler : public osgGA::GUIEventHandler
{
public:

  InputHandler(const SkeletonPtr& atlas)
    : mAtlas(atlas)
  {
    // Do nothing
  }

  virtual bool handle(const osgGA::GUIEventAdapter& ea,
                      osgGA::GUIActionAdapter&) override
  {
    if(nullptr == mAtlas)
    {
      return false;
    }

    if( osgGA::GUIEventAdapter::KEYDOWN == ea.getEventType() )
    {
      if( ea.getKey() == 'p' )
      {
        for(size_t i=0; i < mAtlas->getNumDofs(); ++i)
          std::cout << mAtlas->getDof(i)->getName() << ": "
                    << mAtlas->getDof(i)->getPosition() << std::endl;
        std::cout << "  -- -- -- -- -- " << std::endl;
        return true;
      }
    }

    return false;
  }

protected:

  SkeletonPtr mAtlas;

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

  for(size_t i=0; i<atlas->getNumDofs(); ++i)
  {
    DegreeOfFreedom* dof = atlas->getDof(i);
    std::cout << dof->getName() << " ("
              << dof->getPosition() << "): "
              << dof->getPositionLowerLimit()*180.0/M_PI << " -> "
              << dof->getPositionUpperLimit()*180.0/M_PI << std::endl;
  }

  atlas->getDof("r_leg_hpy")->setPosition(-45.0*M_PI/180.0);
  atlas->getDof("r_leg_kny")->setPosition( 90.0*M_PI/180.0);
  atlas->getDof("r_leg_aky")->setPosition(-45.0*M_PI/180.0);

//  atlas->getDof("r_arm_ely")->setPosition(90.0*M_PI/180.0);
//  atlas->getDof("l_arm_ely")->setPosition(90.0*M_PI/180.0);

  atlas->getDof("r_arm_elx")->setPosition(-90*M_PI/180.0);
  atlas->getDof("l_arm_elx")->setPosition( 90*M_PI/180.0);



  atlas->getDof("r_leg_kny")->setPositionLowerLimit( 10*M_PI/180.0);

  EndEffector* l_hand = atlas->getBodyNode("l_hand")->createEndEffector();
  l_hand->setName("l_hand");

  Eigen::VectorXd weights = Eigen::VectorXd::Ones(6);
  weights = 0.01*weights;

  osgDart::InteractiveFramePtr l_target(new osgDart::InteractiveFrame(
                                          Frame::World(), "l_target"));
  l_target->setTransform(l_hand->getTransform());
  l_hand->getIK(true)->setTarget(l_target);
  l_hand->getIK()->useWholeBody();
  l_hand->getIK()->getGradientMethod().setComponentWeights(weights);
  world->addSimpleFrame(l_target);

  EndEffector* r_hand = atlas->getBodyNode("r_hand")->createEndEffector();
  osgDart::InteractiveFramePtr r_target(new osgDart::InteractiveFrame(
                                          Frame::World(), "r_target"));
  r_target->setTransform(r_hand->getTransform());
  r_hand->getIK(true)->setTarget(r_target);
  r_hand->getIK()->useWholeBody();
  r_hand->getIK()->getGradientMethod().setComponentWeights(weights);
  world->addSimpleFrame(r_target);

//  BodyNode* r_foot = atlas->getBodyNode("r_foot");
//  InverseKinematicsPtr rik = r_foot->getIK(true);
//  rik->getErrorMethod().setLinearBounds(
//       -std::numeric_limits<double>::infinity()*Eigen::Vector3d(1.0, 1.0, 0.0),
//        std::numeric_limits<double>::infinity()*Eigen::Vector3d(1.0, 1.0, 0.0));

//  EndEffector* r_foot = atlas->getBodyNode("r_foot")->createEndEffector();
  BodyNode* r_foot = atlas->getBodyNode("r_foot");
  osgDart::InteractiveFramePtr rf_target(new osgDart::InteractiveFrame(
                                           Frame::World(), "rf_target"));
  rf_target->setTransform(r_foot->getTransform());
  r_foot->getIK(true)->setTarget(rf_target);
  r_foot->getIK()->useWholeBody();
  r_foot->getIK()->setHierarchyLevel(1);
  world->addSimpleFrame(rf_target);


  for(size_t i=3; i < 6; ++i)
  {
    atlas->getDof(i)->setPositionLowerLimit(-5);
    atlas->getDof(i)->setPositionUpperLimit( 5);
  }

  std::shared_ptr<dart::optimizer::GradientDescentSolver> solver =
      std::dynamic_pointer_cast<dart::optimizer::GradientDescentSolver>(
      atlas->getIK(true)->getSolver());
  solver->setNumMaxIterations(10);

  std::shared_ptr<RelaxedPose> objective(new RelaxedPose(atlas->getPositions()));
  atlas->getIK()->getProblem()->addSeed(atlas->getPositions());
  atlas->getIK()->setObjective(objective);

  osg::ref_ptr<osgDart::WorldNode> node = new TeleoperationWorld(world, atlas);

  osgDart::Viewer viewer;
  viewer.addWorldNode(node);

  viewer.addEventHandler(new InputHandler(atlas));

  viewer.enableDragAndDrop(l_target.get());
  viewer.enableDragAndDrop(r_target.get());

  viewer.enableDragAndDrop(rf_target.get());

  for(size_t i=0; i<atlas->getNumBodyNodes(); ++i)
//    viewer.enableDragAndDrop(atlas->getBodyNode(i), false);
    viewer.enableDragAndDrop(atlas->getBodyNode(i), true);

  std::cout << viewer.getInstructions() << std::endl;

  viewer.setUpViewInWindow(0, 0, 640, 480);

  viewer.run();
}
