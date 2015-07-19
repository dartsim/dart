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

const double display_elevation = 0.05;

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
      mRobot(_robot),
      iter(0)
  {
    // Do nothing
  }

  void customPreRefresh() override
  {
    bool solved = mRobot->getIK(true)->solve();

    if(!solved)
      ++iter;
    else
      iter = 0;

    if(iter == 1000)
    {
      std::cout << "Failing!" << std::endl;
    }
  }

protected:

  SkeletonPtr mRobot;
  size_t iter;

  Eigen::VectorXd grad;
};

void printVec3d(const osg::Vec3d& v)
{
  for(size_t i=0; i<3; ++i)
    std::cout << v[i] << "  ";
  std::cout << std::endl;
}

class InputHandler : public osgGA::GUIEventHandler
{
public:

  InputHandler(const SkeletonPtr& atlas, const WorldPtr& world)
    : mAtlas(atlas),
      mWorld(world)
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
                    << mAtlas->getDof(i)->getPosition()*180.0/M_PI << std::endl;
        std::cout << "  -- -- -- -- -- " << std::endl;
        return true;
      }
    }

    return false;
  }

protected:

  SkeletonPtr mAtlas;

  WorldPtr mWorld;
};

SkeletonPtr createGround()
{
  // Create a Skeleton to represent the ground
  SkeletonPtr ground = Skeleton::create("ground");
  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  double thickness = 0.01;
  tf.translation() = Eigen::Vector3d(0,0,-thickness/2.0);
  WeldJoint::Properties joint;
  joint.mT_ParentBodyToJoint = tf;
  ground->createJointAndBodyNodePair<WeldJoint>(nullptr, joint);
  ShapePtr groundShape =
      std::make_shared<BoxShape>(Eigen::Vector3d(10,10,thickness));
  groundShape->setColor(dart::Color::Blue(0.2));

  ground->getBodyNode(0)->addVisualizationShape(groundShape);
  ground->getBodyNode(0)->addCollisionShape(groundShape);

  return ground;
}

SkeletonPtr createAtlas()
{
  // Parse in the atlas model
  DartLoader urdf;
  SkeletonPtr atlas =
      urdf.parseSkeleton(DART_DATA_PATH"sdf/atlas/atlas_v3_no_head.urdf");

  // Add a box to the root node to make it easier to click and drag
  double scale = 0.25;
  ShapePtr boxShape =
      std::make_shared<BoxShape>(scale*Eigen::Vector3d(1.0, 1.0, 0.5));
  boxShape->setColor(dart::Color::Black());

  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  tf.translation() = Eigen::Vector3d(0.1*Eigen::Vector3d(0.0, 0.0, 1.0));
  boxShape->setLocalTransform(tf);

  atlas->getBodyNode(0)->addVisualizationShape(boxShape);

  return atlas;
}

void setupStartConfiguration(const SkeletonPtr& atlas)
{
  // Squat with the right leg
  atlas->getDof("r_leg_hpy")->setPosition(-45.0*M_PI/180.0);
  atlas->getDof("r_leg_kny")->setPosition( 90.0*M_PI/180.0);
  atlas->getDof("r_leg_aky")->setPosition(-45.0*M_PI/180.0);

  // Squat with the left left
  atlas->getDof("l_leg_hpy")->setPosition(-45.0*M_PI/180.0);
  atlas->getDof("l_leg_kny")->setPosition( 90.0*M_PI/180.0);
  atlas->getDof("l_leg_aky")->setPosition(-45.0*M_PI/180.0);

  // Get the right arm into a comfortable position
  atlas->getDof("r_arm_shx")->setPosition( 65.0*M_PI/180.0);
  atlas->getDof("r_arm_ely")->setPosition( 90.0*M_PI/180.0);
  atlas->getDof("r_arm_elx")->setPosition(-90.0*M_PI/180.0);
  atlas->getDof("r_arm_wry")->setPosition( 65.0*M_PI/180.0);

  // Get the left arm into a comfortable position
  atlas->getDof("l_arm_shx")->setPosition(-65.0*M_PI/180.0);
  atlas->getDof("l_arm_ely")->setPosition( 90.0*M_PI/180.0);
  atlas->getDof("l_arm_elx")->setPosition( 90.0*M_PI/180.0);
  atlas->getDof("l_arm_wry")->setPosition( 65.0*M_PI/180.0);

  // Prevent the knees from bending backwards
  atlas->getDof("r_leg_kny")->setPositionLowerLimit( 10*M_PI/180.0);
  atlas->getDof("l_leg_kny")->setPositionLowerLimit( 10*M_PI/180.0);

  // Give finite limits to the root translation
  for(size_t i=3; i < 6; ++i)
  {
    atlas->getDof(i)->setPositionLowerLimit(-5);
    atlas->getDof(i)->setPositionUpperLimit( 5);
  }
}

void setupEndEffectors(const SkeletonPtr& atlas)
{
  // Apply very small weights to the gradient of the root joint in order to
  // encourage the arms to use arm joints instead of only moving around the root
  // joint
  Eigen::VectorXd rootjoint_weights = Eigen::VectorXd::Ones(6);
  rootjoint_weights = 0.01*rootjoint_weights;

  // -- Set up the left hand --

  // Create a relative transform for the EndEffector frame. This is the
  // transform that the left hand will have relative to the BodyNode that it is
  // attached to
  Eigen::Isometry3d tf_hand(Eigen::Isometry3d::Identity());
  tf_hand.translation() = Eigen::Vector3d(0.0009, 0.1254, 0.012);
  tf_hand.rotate(Eigen::AngleAxisd(90.0*M_PI/180.0, Eigen::Vector3d::UnitZ()));

  // Create the left hand's end effector and set its relative transform
  EndEffector* l_hand = atlas->getBodyNode("l_hand")->createEndEffector("l_hand");
  l_hand->setDefaultRelativeTransform(tf_hand, true);

  // Create an interactive frame to use as the target for the left hand
  osgDart::InteractiveFramePtr l_target(new osgDart::InteractiveFrame(
                                          Frame::World(), "l_target"));

  // Set the target of the left hand to the interactive frame. We pass true into
  // the function to tell it that it should create the IK module if it does not
  // already exist. If we don't do that, then calling getIK() could return a
  // nullptr if the IK module was never created.
  l_hand->getIK(true)->setTarget(l_target);

  // Tell the left hand to use the whole body for its IK
  l_hand->getIK()->useWholeBody();

  // Set the weights for the gradient
  l_hand->getIK()->getGradientMethod().setComponentWeights(rootjoint_weights);

  // Start off being inactive
  l_hand->getIK()->setActive(false);

  // -- Set up the right hand --

  // The orientation of the right hand frame is different than the left, so we
  // need to adjust the signs of the relative transform
  tf_hand.translation()[0] = -tf_hand.translation()[0];
  tf_hand.translation()[1] = -tf_hand.translation()[1];
  tf_hand.linear() = tf_hand.linear().inverse();

  // Create the right hand's end effector and set its relative transform
  EndEffector* r_hand = atlas->getBodyNode("r_hand")->createEndEffector("r_hand");
  r_hand->setDefaultRelativeTransform(tf_hand, true);

  // Create an interactive frame to use as the target for the right hand
  osgDart::InteractiveFramePtr r_target(new osgDart::InteractiveFrame(
                                          Frame::World(), "r_target"));

  // Create the right hand's IK and set its  target
  r_hand->getIK(true)->setTarget(r_target);

  // Tell the right hand to use the whole body for its IK
  r_hand->getIK()->useWholeBody();

  // Set the weights for the gradient
  r_hand->getIK()->getGradientMethod().setComponentWeights(rootjoint_weights);

  // Start off being inactive
  r_hand->getIK()->setActive(false);


  // Define the support geometry for the feet. These points will be used to
  // compute the convex hull of the robot's support polygon
  dart::math::SupportGeometry support;
  const double sup_pos_x =  0.10;
  const double sup_neg_x = -0.03;
  const double sup_pos_y =  0.03;
  const double sup_neg_y = -0.03;
  support.push_back(Eigen::Vector3d(sup_neg_x, sup_neg_y, 0.0));
  support.push_back(Eigen::Vector3d(sup_pos_x, sup_neg_y, 0.0));
  support.push_back(Eigen::Vector3d(sup_pos_x, sup_pos_y, 0.0));
  support.push_back(Eigen::Vector3d(sup_neg_x, sup_pos_y, 0.0));

  // Create a relative transform that goes from the center of the feet to the
  // bottom of the feet
  Eigen::Isometry3d tf_foot(Eigen::Isometry3d::Identity());
  tf_foot.translation() = Eigen::Vector3d(0.0, 0.0, -0.08);

  // Define linear bounds for the feet that allow them to traverse across the
  // ground while also staying pinned to the ground
  Eigen::Vector3d linearBounds =
      Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  linearBounds[2] = 1e-8;

  // Define angular bounds for the feet that allow them to yaw while remaining
  // flat on the ground
  Eigen::Vector3d angularBounds =
      Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  angularBounds[0] = 1e-8;
  angularBounds[1] = 1e-8;

  // Create an end effector for the left foot and set its relative transform
  EndEffector* l_foot = atlas->getBodyNode("l_foot")->createEndEffector("l_foot");
  l_foot->setRelativeTransform(tf_foot);

  // Create the IK module for the left foot and set its hierarchy level to 1.
  // This will project its IK goals through the null space of any IK modules
  // that are on level 0. This means that it will try to accomplish its goals
  // while also accommodating the goals of other modules.
  l_foot->getIK(true)->setHierarchyLevel(1);

  // Use the bounds defined above
  l_foot->getIK()->getErrorMethod().setLinearBounds(
        -linearBounds, linearBounds);
  l_foot->getIK()->getErrorMethod().setAngularBounds(
        -angularBounds, angularBounds);

  // Pass in the support geometry for the foot
  l_foot->setSupportGeometry(support);

  // Turn on support mode so that it can be used as a foot
  l_foot->setSupportMode(true);

  // Create an end effector for the right foot and set its relative transform
  EndEffector* r_foot = atlas->getBodyNode("r_foot")->createEndEffector("r_foot");
  r_foot->setRelativeTransform(tf_foot);

  // Create the IK module for the right foot and set its hierarchy level to 1
  r_foot->getIK(true)->setHierarchyLevel(1);

  // Use the bounds defined above
  r_foot->getIK()->getErrorMethod().setLinearBounds(
        -linearBounds, linearBounds);
  r_foot->getIK()->getErrorMethod().setAngularBounds(
        -angularBounds, angularBounds);

  // Pass in the support geometry for the foot
  r_foot->setSupportGeometry(support);

  // Turn on support mode so that it can be used as a foot
  r_foot->setSupportMode(true);

  // Move atlas to the ground so that it starts out squatting with its feet on
  // the ground
  double heightChange = -r_foot->getIK()->getTarget()->getWorldTransform().translation()[2];
  atlas->getDof(5)->setPosition(heightChange);

  // Now that the feet are on the ground, we should set the feet's targets
  l_foot->getIK()->getTarget()->setTransform(l_foot->getTransform());
  r_foot->getIK()->getTarget()->setTransform(r_foot->getTransform());
}

void setupWholeBodySolver(const SkeletonPtr& atlas)
{
  // The default
  std::shared_ptr<dart::optimizer::GradientDescentSolver> solver =
      std::dynamic_pointer_cast<dart::optimizer::GradientDescentSolver>(
      atlas->getIK(true)->getSolver());
  solver->setNumMaxIterations(10);

  std::shared_ptr<RelaxedPose> objective(new RelaxedPose(atlas->getPositions()));
  atlas->getIK()->getProblem()->addSeed(atlas->getPositions());

  std::shared_ptr<dart::constraint::BalanceConstraint> balance =
      std::make_shared<dart::constraint::BalanceConstraint>(atlas->getIK());
  atlas->getIK()->getProblem()->addEqConstraint(balance);
  atlas->getIK()->setObjective(objective);

//  // Shift the center of mass towards the support polygon center while trying
//  // to keep the support polygon where it is
//  balance->setErrorMethod(dart::constraint::BalanceConstraint::FROM_CENTROID);
//  balance->setBalanceMethod(dart::constraint::BalanceConstraint::SHIFT_COM);

//  // Keep shifting the center of mass towards the center of the support
//  // polygon, even if it is already inside. This is useful for trying to
//  // optimize a stance
//  balance->setErrorMethod(dart::constraint::BalanceConstraint::OPTIMIZE_BALANCE);
//  balance->setBalanceMethod(dart::constraint::BalanceConstraint::SHIFT_COM);

//  // Try to leave the center of mass where it is while moving the support
//  // polygon to be under the current center of mass location
  balance->setErrorMethod(dart::constraint::BalanceConstraint::FROM_CENTROID);
  balance->setBalanceMethod(dart::constraint::BalanceConstraint::SHIFT_SUPPORT);

//  // Try to leave the center of mass where it is while moving the support
//  // point that is closest to the center of mass
//  balance->setErrorMethod(dart::constraint::BalanceConstraint::FROM_EDGE);
//  balance->setBalanceMethod(dart::constraint::BalanceConstraint::SHIFT_SUPPORT);

  // Note that using the FROM_EDGE error method is liable to leave the center of
  // mass visualization red even when the constraint was successfully solved.
  // This is because the constraint solver has a tiny bit of tolerance that
  // allows the Problem to be considered solved when the center of mass is
  // microscopically outside of the support polygon. This is an inherent risk of
  // using FROM_EDGE instead of FROM_CENTROID.

  // Feel free to experiment with the different balancing methods. You will find
  // that some work much better for user interaction than others.
}

int main()
{
  WorldPtr world(new World);

  SkeletonPtr atlas = createAtlas();
  world->addSkeleton(atlas);

  SkeletonPtr ground = createGround();
  world->addSkeleton(ground);

  setupStartConfiguration(atlas);

  setupEndEffectors(atlas);

  setupWholeBodySolver(atlas);

  osg::ref_ptr<osgDart::WorldNode> node = new TeleoperationWorld(world, atlas);

  osgDart::Viewer viewer;

  // Prevent this World from simulating
  viewer.allowSimulation(false);
  viewer.addWorldNode(node);

  // Add our custom input handler to the Viewer
  viewer.addEventHandler(new InputHandler(atlas, world));

  // Turn on drag-and-drop for the whole Skeleton
  for(size_t i=0; i<atlas->getNumBodyNodes(); ++i)
    viewer.enableDragAndDrop(atlas->getBodyNode(i), false, false);

  // Attach a support polygon visualizer
  viewer.addAttachment(new osgDart::SupportPolygonVisual(
                         atlas, display_elevation));

  // Print out instructions for the viewer
  std::cout << viewer.getInstructions() << std::endl;

  // Set up the window
  viewer.setUpViewInWindow(0, 0, 1280, 960);

  // Set up the default viewing position
  viewer.getCameraManipulator()->setHomePosition(osg::Vec3( 5.34,  3.00, 2.41),
                                                 osg::Vec3( 0.00,  0.00, 1.00),
                                                 osg::Vec3(-0.20, -0.08, 0.98));

  // Reset the camera manipulator so that it starts in the new viewing position
  viewer.setCameraManipulator(viewer.getCameraManipulator());

  // Run the Viewer
  viewer.run();
}
