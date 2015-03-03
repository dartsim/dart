/*
 * Copyright (c) 2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Michael X. Grey <mxgrey@gatech.edu>
 *            Jeongseok Lee <jslee02@gmail.com>
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
using namespace dart::math;

class OperationalSpaceControlWorld : public osgDart::WorldNode
{
public:

  OperationalSpaceControlWorld(dart::simulation::World* _world)
    : osgDart::WorldNode(_world)
  {
    // Extract relevant pointers
    mRobot = mWorld->getSkeleton(0);
    mEndEffector = mRobot->getBodyNode(mRobot->getNumBodyNodes()-1);

    // Setup gain matrices
    size_t dofs = 3;
    mKp.setZero(dofs,dofs);
    mKv.setZero(dofs,dofs);

    for(size_t i=0; i<dofs; ++i)
    {
      mKp(i,i) = 750.0;
      mKv(i,i) = 250.0;
    }

    // Set joint properties
    for(size_t i=0; i<mRobot->getNumJoints(); ++i)
    {
      mRobot->getJoint(i)->setPositionLimited(false);
      mRobot->getJoint(i)->setDampingCoefficient(0, 0.5);
    }

    // Create target Frame
    mTarget = new SimpleFrame(Frame::World(), "target",
                              mEndEffector->getWorldTransform());
    Shape* ball = new EllipsoidShape(Eigen::Vector3d(0.05,0.05,0.05));
    ball->setColor(Eigen::Vector3d(0.9,0,0));
    mTarget->addVisualizationShape(ball);
    mWorld->addFrame(mTarget);
  }

  // Triggered at the beginning of each simulation step
  void customPreStep() override
  {
    Eigen::Vector3d x    = mEndEffector->getWorldTransform().translation();
    Eigen::Vector3d dx   = mEndEffector->getLinearVelocity();
    Eigen::MatrixXd invM = mRobot->getInvMassMatrix();
    Eigen::VectorXd Cg   = mRobot->getCoriolisAndGravityForces();
    LinearJacobian Jv    = mEndEffector->getLinearJacobian();
    LinearJacobian dJv   = mEndEffector->getLinearJacobianDeriv();
    Eigen::VectorXd dq   = mRobot->getVelocities();

    Eigen::MatrixXd A = Jv*invM;
    Eigen::Vector3d b = dJv*dq;
    Eigen::Matrix3d M2 = Jv*invM*Jv.transpose();

    Eigen::Vector3d target = mTarget->getWorldTransform().translation();

    Eigen::Vector3d f = -mKp*(x - target) - mKv*dx;

    Eigen::Vector3d desired_ddx = b + M2*f;

    mForces = Cg;

    mForces += A.colPivHouseholderQr().solve(desired_ddx - b);

    mRobot->setForces(mForces);
  }

protected:

  // Triggered when this node gets added to the Viewer
  void setupViewer() override
  {
    if(mViewer)
      mViewer->enableDragAndDrop(mTarget);
  }

  Skeleton* mRobot;
  BodyNode* mEndEffector;
  SimpleFrame* mTarget;

  Eigen::Matrix3d mKp;
  Eigen::Matrix3d mKv;
  Eigen::VectorXd mForces;
};

int main()
{
  dart::utils::DartLoader loader;
  dart::simulation::World* world = new dart::simulation::World;

  dart::dynamics::Skeleton* robot =
      loader.parseSkeleton(DART_DATA_PATH"urdf/KR5/KR5 sixx R650.urdf");
  world->addSkeleton(robot);

  dart::dynamics::Skeleton* ground =
      loader.parseSkeleton(DART_DATA_PATH"urdf/KR5/ground.urdf");
  world->addSkeleton(ground);

  osg::ref_ptr<OperationalSpaceControlWorld> node =
      new OperationalSpaceControlWorld(world);
  node->setNumStepsPerCycle(10);

  osgDart::Viewer viewer;
  viewer.addWorldNode(node);
  viewer.simulate(true);

  viewer.addInstructionText("\nClick and drag the red ball to move the target of the operational space controller\n");

  std::cout << viewer.getInstructions() << std::endl;

  viewer.setUpViewInWindow(0, 0, 640, 480);

  viewer.run();
}
