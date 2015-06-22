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

#include "dart/dart.h"

const double default_force =  50.0; // N
const int default_countdown = 100;  // Number of timesteps for applying force

using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::gui;
using namespace dart::utils;

class Controller
{
public:
  /// Constructor
  Controller(const SkeletonPtr& biped)
    : mBiped(biped)
  {
    int nDofs = mBiped->getNumDofs();
    mKp = Eigen::MatrixXd::Identity(nDofs, nDofs);
    mKd = Eigen::MatrixXd::Identity(nDofs, nDofs);
    
    for(size_t i = 0; i < 6; ++i)
    {
      mKp(i, i) = 0.0;
      mKd(i, i) = 0.0;
    }

    for(size_t i = 6; i < biped->getNumDofs(); ++i)
    {
      mKp(i, i) = 400;
      mKd(i, i) = 40;
    }

    resetTargetPositions();
  }
  
  /// Reset the desired dof position to the current position
  void resetTargetPositions()
  {
    mTargetPositions = mBiped->getPositions();
  }

  /// Compute commanding force using PD controllers ( Lesson 3 Answer)
  void setPDForces()
  {
    mForces.setZero();
    Eigen::VectorXd q = mBiped->getPositions();
    Eigen::VectorXd dq = mBiped->getVelocities();
    
    Eigen::VectorXd p = -mKp * (q - mTargetPositions);
    Eigen::VectorXd d = -mKd * dq;
    
    mForces = p + d;
    mBiped->setForces(mForces);
  }

  /// Compute commanind forces using Stable-PD controllers (Lesson 3 Answer)
  void setSPDForces()
  {
    mForces.setZero();
    Eigen::VectorXd q = mBiped->getPositions();
    Eigen::VectorXd dq = mBiped->getVelocities();

    Eigen::MatrixXd invM = (mBiped->getMassMatrix() + mKd * mBiped->getTimeStep()).inverse();
    Eigen::VectorXd p = -mKp * (q + dq * mBiped->getTimeStep() - mTargetPositions);
    Eigen::VectorXd d = -mKd * dq;
    Eigen::VectorXd qddot =
        invM * (-mBiped->getCoriolisAndGravityForces() + p + d + mBiped->getConstraintForces());
    
    mForces = p + d - mKd * qddot * mBiped->getTimeStep();
    mBiped->setForces(mForces);
  }
  
  /// Compute commanding forces using ankle strategy (Lesson 4 Answer)
  void setAnkleStrategyForces()
  {
    // Lesson 4
  }
  
  /// Compute commadning forces due to gravity compensation (Lesson 7 Answer)
  void setGravityCompensation()
  {
    // Lesson 7
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
    
    mController = std::unique_ptr<Controller>(new Controller(mWorld->getSkeleton("biped")));
  }
  
  /// Handle keyboard input
  void keyboard(unsigned char key, int x, int y) override
  {
    switch(key)
    {
      case 'q':
        mForceCountDown = default_countdown;
        mPositiveSign = false;
        break;
      case 'w':
        mForceCountDown = default_countdown;
        mPositiveSign = true;
        break;
      default:
        SimWindow::keyboard(key, x, y);
    }
  }

  void timeStepping() override
  {
    mController->setSPDForces();
    
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

// Lesson 1 Answer
void initializeBiped(SkeletonPtr biped)
{
  // Set joint limits on knees
  for(size_t i = 0; i < biped->getNumJoints(); ++i)
    biped->getJoint(i)->setPositionLimited(true);
  
  // Enable self collision check but ignore adjacent bodies
  biped->enableSelfCollision();
  
  // Set initial configuration
  biped->setPosition(biped->getDof("j_thigh_left_z")->getIndexInSkeleton(), 0.15);
  biped->setPosition(biped->getDof("j_thigh_right_z")->getIndexInSkeleton(), 0.15);
  biped->setPosition(biped->getDof("j_shin_left")->getIndexInSkeleton(), -0.4);
  biped->setPosition(biped->getDof("j_shin_right")->getIndexInSkeleton(), -0.4);
  biped->setPosition(biped->getDof("j_heel_left_1")->getIndexInSkeleton(), 0.25);
  biped->setPosition(biped->getDof("j_heel_right_1")->getIndexInSkeleton(), 0.25);
}

// Lesson 6 Answer
void setVelocityAccuators(SkeletonPtr biped)
{
  Joint* joint0 = biped->getJoint(0);
  joint0->setActuatorType(Joint::PASSIVE);
  for (size_t i = 1; i < biped->getNumBodyNodes(); ++i)
  {
    Joint* joint = biped->getJoint(i);
    joint->setActuatorType(Joint::VELOCITY);
  }
}


int main(int argc, char* argv[])
{
  // Create the world with a skeleton
  WorldPtr world
      = SkelParser::readWorld(
          DART_DATA_PATH"skel/biped.skel");
  assert(world != nullptr);

  initializeBiped(world->getSkeleton("biped"));
  
  // Create a window for rendering the world and handling user input
  MyWindow window(world);

  // Print instructions
  std::cout << "space bar: simulation on/off" << std::endl;
  std::cout << "'p': replay simulation" << std::endl;
  std::cout << "'v': Turn contact force visualization on/off" << std::endl;
  std::cout << "'[' and ']': replay one frame backward and forward" << std::endl;
 
  // Initialize glut, initialize the window, and begin the glut event loop
  glutInit(&argc, argv);
  window.initWindow(640, 480, "Multi-Pendulum Tutorial");
  glutMainLoop();
}
