/*
 * Copyright (c) 2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Karen Liu <karenliu@cc.gatech.edu>
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


const double default_speed_increment = 0.5;

const int default_ik_iterations = 4500;

const double default_force =  50.0; // N
const int default_countdown = 100;  // Number of timesteps for applying force

#include "dart/dart.h"

using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::gui;
using namespace dart::utils;
using namespace dart::math;

class Controller
{
public:
  /// Constructor
  Controller(const SkeletonPtr& biped)
    : mBiped(biped),
      mPreOffset(0.0),
      mSpeed(0.0)
  {
    int nDofs = mBiped->getNumDofs();
    
    mForces = Eigen::VectorXd::Zero(nDofs);
    
    mKp = Eigen::MatrixXd::Identity(nDofs, nDofs);
    mKd = Eigen::MatrixXd::Identity(nDofs, nDofs);
  
    for(size_t i = 0; i < 6; ++i)
    {
      mKp(i, i) = 0.0;
      mKd(i, i) = 0.0;
    }

    for(size_t i = 6; i < biped->getNumDofs(); ++i)
    {
      mKp(i, i) = 1000;
      mKd(i, i) = 50;
    }

    setTargetPositions(mBiped->getPositions());
  }
  
  /// Reset the desired dof position to the current position
  void setTargetPositions(const Eigen::VectorXd& pose)
  {
    mTargetPositions = pose;
  }

  /// Clear commanding forces
  void clearForces()
  {
    mForces.setZero();
  }
  
  /// Add commanding forces from PD controllers
  void addPDForces()
  {
    // Lesson 2
    Eigen::VectorXd q = mBiped->getPositions();
    Eigen::VectorXd dq = mBiped->getVelocities();

    Eigen::VectorXd p = -mKp*(q - mTargetPositions);
    Eigen::VectorXd d = -mKd * dq;

    mForces +=p+d;
    //mForces =p+d;
    mBiped->setForces(mForces);
  }

  /// Add commanind forces from Stable-PD controllers
  void addSPDForces()
  {
    // Lesson 3

    // make the biped robots not collapse, even it falls, it is still in the standing phase

  	
    Eigen::VectorXd q = mBiped->getPositions();
    Eigen::VectorXd dq =  mBiped->getVelocities();

    Eigen::MatrixXd invM = (mBiped->getMassMatrix() + mKd * mBiped->getTimeStep()).inverse();
    Eigen::VectorXd p = -mKp * (q + dq * mBiped->getTimeStep() - mTargetPositions);
    Eigen::VectorXd d = -mKd * dq;
    Eigen::VectorXd qddot = invM * (-mBiped->getCoriolisAndGravityForces() + p + d + mBiped->getConstraintForces());

    mForces += p + d -mKd * qddot * mBiped->getTimeStep();
    //mForces = p + d -mKd * qddot * mBiped->getTimeStep();
    mBiped->setForces(mForces);
	

  }   
  
  /// add commanding forces from ankle strategy
  void addAnkleStrategyForces()
  {
    // Lesson 4
    Eigen::Vector3d COM = mBiped->getCOM();
    Eigen::Vector3d offset(0.05, 0, 0);
    Eigen::Vector3d COP = mBiped->getBodyNode("h_heel_left")->getTransform() * offset;
    double diff = COM[0] - COP[0];

    Eigen::Vector3d dCOM = mBiped->getCOMLinearVelocity();
    Eigen::Vector3d dCOP = mBiped->getBodyNode("h_heel_left")->getLinearVelocity(offset);
    double dDiff = dCOM[0] - dCOP[0];

    //copy from *-finshed file
    int lHeelIndex = mBiped->getDof("j_heel_left_1")->getIndexInSkeleton();
    int rHeelIndex = mBiped->getDof("j_heel_right_1")->getIndexInSkeleton();
    int lToeIndex = mBiped->getDof("j_toe_left")->getIndexInSkeleton();
    int rToeIndex = mBiped->getDof("j_toe_right")->getIndexInSkeleton();
    if(diff < 0.1 && diff >= 0.0) {
      // Feedback rule for recovering forward push
      double k1 = 200.0;
      double k2 = 100.0;
      double kd = 10;
      mForces[lHeelIndex] += -k1 * diff - kd * dDiff;
      mForces[lToeIndex] += -k2 * diff - kd * dDiff;
      mForces[rHeelIndex] += -k1 * diff - kd * dDiff;
      mForces[rToeIndex] += -k2 * diff - kd * dDiff;
    }else if(diff > -0.2 && diff < -0.05) {
      // Feedback rule for recovering backward push
      double k1 = 2000.0;
      double k2 = 100.0;
      double kd = 100;
      mForces[lHeelIndex] += -k1 * diff - kd * dDiff;
      mForces[lToeIndex] += -k2 * diff - kd * dDiff;
      mForces[rHeelIndex] += -k1 * diff - kd * dDiff;
      mForces[rToeIndex] += -k2 * diff - kd * dDiff;
    }  
    //std::cout<<mForces;
    mBiped->setForces(mForces);

  }
  
  // Send velocity commands on wheel actuators
  void setWheelCommands()
  {
    // Lesson 6
    int wheelFirstIndex = mBiped->getDof("joint_front_left_1")->getIndexInSkeleton();
    for (size_t i = wheelFirstIndex; i< mBiped->getNumDofs(); ++i)
    {
      mKp(i,i) = 0.0;
      mKd(i,i) = 0.0;
    }

    int index1 = mBiped->getDof("joint_front_left_2")->getIndexInSkeleton();
    mBiped->setCommand(index1, mSpeed);

    int index2 = mBiped->getDof("joint_front_right_2")->getIndexInSkeleton();
    mBiped->setCommand(index2, mSpeed);

    int index3 = mBiped->getDof("joint_back_left")->getIndexInSkeleton();
    mBiped->setCommand(index3, mSpeed);

    int index4 = mBiped->getDof("joint_back_right")->getIndexInSkeleton();
    mBiped->setCommand(index4, mSpeed);
  }
  
  void changeWheelSpeed(double increment)
  {
    mSpeed += increment;
    std::cout << "wheel speed = " << mSpeed << std::endl;
  }
  
protected:
  /// The biped Skeleton that we will be controlling
  SkeletonPtr mBiped;
  
  /// Joint forces for the biped (output of the Controller)
  Eigen::VectorXd mForces;
  
  /// Control gains for the proportional error terms in the PD controller
  Eigen::MatrixXd mKp;

  /// Control gains for the derivative error terms in the PD controller
  Eigen::MatrixXd mKd;

  /// Target positions for the PD controllers
  Eigen::VectorXd mTargetPositions;
  
  /// For ankle strategy: Error in the previous timestep
  double mPreOffset;
  
  /// For velocity actuator: Current speed of the skateboard
  double mSpeed;
};

class MyWindow : public SimWindow
{
public:
  /// Constructor
  MyWindow(const WorldPtr& world)
  : mForceCountDown(0),
    mPositiveSign(true)
  {
    setWorld(world);
    
    mController = std::unique_ptr<Controller>
        (new Controller(mWorld->getSkeleton("biped")));
  }
  
  /// Handle keyboard input
  void keyboard(unsigned char key, int x, int y) override
  {
    switch(key)
    {
      case ',':
        mForceCountDown = default_countdown;
        mPositiveSign = false;
        break;
      case '.':
        mForceCountDown = default_countdown;
        mPositiveSign = true;
        break;
      case 'a':
        mController->changeWheelSpeed(default_speed_increment);
        break;
      case 's':
        mController->changeWheelSpeed(-default_speed_increment);
        break;
      default:
        SimWindow::keyboard(key, x, y);
    }
  }

  void timeStepping() override
  {
    mController->clearForces();
    
    // Lesson 3
    mController->addSPDForces();
    //mController->addPDForces();

    // Lesson 4
    mController->addAnkleStrategyForces();
    
    // Lesson 6
    mController->setWheelCommands();
    
    // Apply body forces based on user input, and color the body shape red
    if(mForceCountDown > 0)
    {
      BodyNode* bn = mWorld->getSkeleton("biped")->getBodyNode("h_abdomen");
      const ShapePtr& shape = bn->getVisualizationShape(0);
      shape->setColor(dart::Color::Red());
      
      if(mPositiveSign)
        bn->addExtForce(default_force * Eigen::Vector3d::UnitX(),
                        bn->getCOM(), false, false);
      else
        bn->addExtForce(-default_force*Eigen::Vector3d::UnitX(),
                        bn->getCOM(), false, false);
      
      --mForceCountDown;
    }
    
    // Step the simulation forward
    SimWindow::timeStepping();
  }

protected:
  std::unique_ptr<Controller> mController;
  
  /// Number of iterations before clearing a force entry
  int mForceCountDown;
  
  /// Whether a force should be applied in the positive or negative direction
  bool mPositiveSign;
  
};

// Load a biped model and enable joint limits and self-collision
SkeletonPtr loadBiped()
{
  // Lesson 1
  
  // Create the world with a skeleton
  WorldPtr world = SkelParser::readWorld(DART_DATA_PATH"skel/biped.skel");
  assert(world != nullptr);

  SkeletonPtr biped = world->getSkeleton("biped");

  // set Joint Position limit to make the biped behave more like real human
  for(size_t i=0; i < biped->getNumJoints();++i)
  {
    //biped->getJoint(i)->setPositionLimited(true);
    biped->getJoint(i)->setPositionLimitEnforced(true);
  }
  
  biped->enableSelfCollision();
  return biped;
}

// set biped to a particular cofiguration
// but cannot maintain this configuration because no controller put force on it
void setInitialPose(SkeletonPtr biped)
{
	
  biped->setPosition(biped->getDof("j_thigh_left_z")->getIndexInSkeleton(), 0.15);
  biped->setPosition(biped->getDof("j_thigh_right_z")->getIndexInSkeleton(), 0.15);
  biped->setPosition(biped->getDof("j_shin_left")->getIndexInSkeleton(), -0.4);
  biped->setPosition(biped->getDof("j_shin_right")->getIndexInSkeleton(), -0.4);
  biped->setPosition(biped->getDof("j_heel_left_1")->getIndexInSkeleton(), 0.25);
  biped->setPosition(biped->getDof("j_heel_right_1")->getIndexInSkeleton(), 0.25);
	
}


// Load a skateboard model and connect it to the biped model via an Euler joint
void modifyBipedWithSkateboard(SkeletonPtr biped)
{
  // Lesson 5
  WorldPtr world = SkelParser::readWorld(DART_DATA_PATH"skel/skateboard.skel");
  SkeletonPtr skateboard = world->getSkeleton(0);

  EulerJoint::Properties properties = EulerJoint::Properties();
  properties.mT_ChildBodyToJoint.translation() = Eigen::Vector3d(0, 0.1, 0);

  skateboard->getRootBodyNode()->moveTo<EulerJoint>(biped->getBodyNode("h_heel_left"), properties);


}

// Set the actuator type for four wheel joints to "VELOCITY"
void setVelocityAccuators(SkeletonPtr biped)
{
  // Lesson 6
  Joint* wheel1 = biped->getJoint("joint_front_left");
  wheel1->setActuatorType(Joint::VELOCITY);

  Joint* wheel2 = biped->getJoint("joint_front_right");
  wheel2->setActuatorType(Joint::VELOCITY);

  Joint* wheel3 = biped->getJoint("joint_back_left");
  wheel3->setActuatorType(Joint::VELOCITY);

  Joint* wheel4 = biped->getJoint("joint_back_right");
  wheel4->setActuatorType(Joint::VELOCITY);

}

// Solve for a balanced pose using IK
Eigen::VectorXd solveIK(SkeletonPtr biped)
{
  // Lesson 7
  /*
  Eigen::Vector3d localCOM = leftHeel->getCOM(leftHeel);
  LinearJacobian jacobian = biped->getCOMLinearJacobian() - biped->getLinearJacobian(leftHeel, localCOM);

  Eigen::Vector3d offset(0.0, -0.04, -0.03);
  gradient = biped->getLinearJacobian(leftHeel, offset).row(1);

  Eigen::VectorXd newPose = biped->getPositions();
  */

  //copy from *-finished file
      // Modify the intial pose to one-foot stance before IK 
        
        biped->setPosition(biped->getDof("j_shin_right")->
                           getIndexInSkeleton(), -1.4);
        biped->setPosition(biped->getDof("j_bicep_left_x")->
                           getIndexInSkeleton(), 0.8);
        biped->setPosition(biped->getDof("j_bicep_right_x")->
                           getIndexInSkeleton(), -0.8);
        

	//get positions for all generalized coordinates
        Eigen::VectorXd newPose = biped->getPositions();
        BodyNodePtr leftHeel = biped->getBodyNode("h_heel_left");
        BodyNodePtr leftToe = biped->getBodyNode("h_toe_left");
        double initialHeight = -0.8;

	for(size_t i = 0; i < default_ik_iterations; ++i)
        {
          //deviation between center of mass and center of pressure
	  Eigen::Vector3d deviation = biped->getCOM() - leftHeel->getCOM();
	  // center of pressure in the left heel reference frame
          Eigen::Vector3d localCOM = leftHeel->getCOM(leftHeel);
	  //Linear jacobian difference  c_dot - p_dot
	  //LinearJacobian is a 3 by dynamic matrix
          LinearJacobian jacobian = biped->getCOMLinearJacobian() -
              biped->getLinearJacobian(leftHeel, localCOM);
          
          // Sagittal deviation
          double error = deviation[0];
          Eigen::VectorXd gradient = jacobian.row(0);
          Eigen::VectorXd newDirection = -0.2 * error * gradient;
          
          // Lateral deviation
          error = deviation[2];
          gradient = jacobian.row(2);
          newDirection += -0.2 * error * gradient;
          
          // Position constraint on four (approximated) corners of the left foot
          Eigen::Vector3d offset(0.0, -0.04, -0.03);
	  //leftHeel->getTransform() compute the BodyNode w.r.t the world frame
          error = (leftHeel->getTransform() * offset)[1] - initialHeight;
          gradient = biped->getLinearJacobian(leftHeel, offset).row(1);
          newDirection += -0.2 * error * gradient;
          
          offset[2] = 0.03;
          error = (leftHeel->getTransform() * offset)[1] - initialHeight;
          gradient = biped->getLinearJacobian(leftHeel, offset).row(1);
          newDirection += -0.2 * error * gradient;
          
          offset[0] = 0.04;
          error = (leftToe->getTransform() * offset)[1] - initialHeight;
          gradient = biped->getLinearJacobian(leftToe, offset).row(1);
          newDirection += -0.2 * error * gradient;
          
          offset[2] = -0.03;
          error = (leftToe->getTransform() * offset)[1] - initialHeight;
          gradient = biped->getLinearJacobian(leftToe, offset).row(1);
          newDirection += -0.2 * error * gradient;
          
          newPose += newDirection;
          biped->setPositions(newPose);
	  //request updating forward kinematics
          biped->computeForwardKinematics(true, false, false);
        }

  return newPose;
}

SkeletonPtr createFloor()
{
  SkeletonPtr floor = Skeleton::create("floor");
  
  // Give the floor a body
  BodyNodePtr body =
      floor->createJointAndBodyNodePair<WeldJoint>(nullptr).second;
  
  // Give the body a shape
  double floor_width = 10.0;
  double floor_height = 0.01;
  std::shared_ptr<BoxShape> box(
      new BoxShape(Eigen::Vector3d(floor_width, floor_height, floor_width)));
  box->setColor(dart::Color::Black());
  
  body->addVisualizationShape(box);
  body->addCollisionShape(box);
  
  // Put the body into position
  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  tf.translation() = Eigen::Vector3d(0.0, -1.0, 0.0);
  body->getParentJoint()->setTransformFromParentBodyNode(tf);
  
  return floor;
}

int main(int argc, char* argv[])
{
  SkeletonPtr floor = createFloor();

  // Lesson 1
  SkeletonPtr biped = loadBiped();

  // Lesson 5
  modifyBipedWithSkateboard(biped);

  // Lesson 6
  setVelocityAccuators(biped);

  // Lesson 7
  Eigen::VectorXd balancedPose = solveIK(biped);
  biped->setPositions(balancedPose);
  
  WorldPtr world = std::make_shared<World>();
  world->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));

#ifdef HAVE_BULLET_COLLISION
  world->getConstraintSolver()->setCollisionDetector(
          new dart::collision::BulletCollisionDetector());
#endif
  
  world->addSkeleton(floor);
  world->addSkeleton(biped);
  
  // Create a window for rendering the world and handling user input
  MyWindow window(world);

  // Print instructions
  std::cout << "'.': forward push" << std::endl;
  std::cout << "',': backward push" << std::endl;
  std::cout << "'s': increase skateboard forward speed" << std::endl;
  std::cout << "'a': increase skateboard backward speed" << std::endl;
  std::cout << "space bar: simulation on/off" << std::endl;
  std::cout << "'p': replay simulation" << std::endl;
  std::cout << "'v': Turn contact force visualization on/off" << std::endl;
  std::cout << "'[' and ']': replay one frame backward and forward" << std::endl;
 
  // Initialize glut, initialize the window, and begin the glut event loop
  glutInit(&argc, argv);
  window.initWindow(640, 480, "Multi-Pendulum Tutorial");
  glutMainLoop();
}
