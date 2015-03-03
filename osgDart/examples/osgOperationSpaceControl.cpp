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
    // Extract the relevant pointers
    mRobot = mWorld->getSkeleton(0);
    mEndEffector = mRobot->getBodyNode(mRobot->getNumBodyNodes()-1);

    // Setup gain matrices
    size_t dofs = mEndEffector->getNumDependentGenCoords();
    mKp.setZero();
    for(size_t i=0; i<3; ++i)
      mKp(i,i) = 50.0;

    mKd.setZero(dofs,dofs);
    for(size_t i=0; i<dofs; ++i)
      mKd(i,i) = 5.0;

    // Set joint properties
    for(size_t i=0; i<mRobot->getNumJoints(); ++i)
    {
      mRobot->getJoint(i)->setPositionLimited(false);
      mRobot->getJoint(i)->setDampingCoefficient(0, 0.5);
    }

    mOffset = Eigen::Vector3d(0.05,0,0);

    // Create target Frame
    Eigen::Isometry3d tf = mEndEffector->getWorldTransform();
    tf.pretranslate(mOffset);
    mTarget = new SimpleFrame(Frame::World(), "target", tf);
    Shape* ball = new EllipsoidShape(Eigen::Vector3d(0.05,0.05,0.05));
    ball->setColor(Eigen::Vector3d(0.9,0,0));
    mTarget->addVisualizationShape(ball);
    mWorld->addFrame(mTarget);

    mOffset = mEndEffector->getWorldTransform().rotation().transpose() * mOffset;
  }

  // Triggered at the beginning of each simulation step
  void customPreStep() override
  {
    Eigen::MatrixXd M = mRobot->getMassMatrix();

    LinearJacobian J = mEndEffector->getLinearJacobian(mOffset);
    Eigen::MatrixXd pinv_J = J.transpose()*(J*J.transpose()
                                +0.0025*Eigen::Matrix3d::Identity()).inverse();

    LinearJacobian dJ = mEndEffector->getLinearJacobianDeriv(mOffset);

    Eigen::MatrixXd pinv_dJ = dJ.transpose()*(dJ*dJ.transpose()
                                +0.0025*Eigen::Matrix3d::Identity()).inverse();


    Eigen::Vector3d e = mTarget->getWorldTransform().translation()
                        - mEndEffector->getWorldTransform()*mOffset;

    Eigen::Vector3d de = - mEndEffector->getLinearVelocity(mOffset);

    Eigen::VectorXd Cg = mRobot->getCoriolisAndGravityForces();

    mForces = M*(pinv_J*mKp*de + pinv_dJ*mKp*e) + Cg + mKd*pinv_J*mKp*e;

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

  Eigen::Vector3d mOffset;
  Eigen::Matrix3d mKp;
  Eigen::MatrixXd mKd;
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
