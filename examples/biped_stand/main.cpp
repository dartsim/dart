/*
 * Copyright (c) 2011-2022, The DART development contributors
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

using namespace dart;

//==============================================================================
class CustomWorldNode : public dart::gui::osg::RealTimeWorldNode
{
public:
  CustomWorldNode(
      dart::simulation::WorldPtr world,
      dart::dynamics::SkeletonPtr biped,
      ::osg::ref_ptr<osgShadow::ShadowTechnique> shadow = nullptr)
    : dart::gui::osg::RealTimeWorldNode(std::move(world), std::move(shadow)),
      mBiped(std::move(biped))
  {
    mLeftHeel = mBiped->getBodyNode("h_heel_left");

    mLeftFoot[0] = mBiped->getDof("j_heel_left_1")->getIndexInSkeleton();
    mLeftFoot[1] = mBiped->getDof("j_toe_left")->getIndexInSkeleton();

    mRightFoot[0] = mBiped->getDof("j_heel_right_1")->getIndexInSkeleton();
    mRightFoot[1] = mBiped->getDof("j_toe_right")->getIndexInSkeleton();

    mTimestep = mWorld->getTimeStep();
    mFrame = 0;
    const int nDof = static_cast<int>(mBiped->getNumDofs());
    mKp = Eigen::MatrixXd::Identity(nDof, nDof);
    mKd = Eigen::MatrixXd::Identity(nDof, nDof);

    mTorques.resize(nDof);
    mTorques.setZero();

    mDesiredDofs = mBiped->getPositions();

    // using SPD results in simple Kp coefficients
    for (int i = 0; i < 6; i++)
    {
      mKp(i, i) = 0.0;
      mKd(i, i) = 0.0;
    }
    for (int i = 6; i < nDof; i++)
      mKp(i, i) = 400.0;
    for (int i = 6; i < nDof; i++)
      mKd(i, i) = 40.0;

    mPreOffset = 0.0;

    mForce.setZero();
    mImpulseDuration = 0;
  }

  void customPreRefresh()
  {
    // Use this function to execute custom code before each time that the
    // window is rendered. This function can be deleted if it does not need
    // to be used.
  }

  void customPostRefresh()
  {
    // Use this function to execute custom code after each time that the
    // window is rendered. This function can be deleted if it does not need
    // to be used.
  }

  void customPreStep()
  {
    // Perturbation
    mBiped->getBodyNode("h_spine")->addExtForce(mForce);
    mImpulseDuration--;
    if (mImpulseDuration <= 0)
    {
      mImpulseDuration = 0;
      mForce.setZero();
    }

    const Eigen::VectorXd dof = mBiped->getPositions();
    const Eigen::VectorXd dofVel = mBiped->getVelocities();
    const Eigen::VectorXd constrForces = mBiped->getConstraintForces();

    // SPD tracking
    // std::size_t nDof = mSkel->getNumDofs();
    const Eigen::MatrixXd invM
        = (mBiped->getMassMatrix() + mKd * mTimestep).inverse();
    const Eigen::VectorXd p = -mKp * (dof + dofVel * mTimestep - mDesiredDofs);
    const Eigen::VectorXd d = -mKd * dofVel;
    const Eigen::VectorXd qddot
        = invM
          * (-mBiped->getCoriolisAndGravityForces() + p + d + constrForces);

    mTorques = p + d - mKd * qddot * mTimestep;

    // ankle strategy for sagital plane
    const Eigen::Vector3d com = mBiped->getCOM();
    const Eigen::Vector3d cop
        = mLeftHeel->getTransform() * Eigen::Vector3d(0.05, 0, 0);

    double offset = com[0] - cop[0];
    if (offset < 0.1 && offset > 0.0)
    {
      double k1 = 200.0;
      double k2 = 100.0;
      double kd = 10.0;
      mTorques[mLeftFoot[0]] += -k1 * offset + kd * (mPreOffset - offset);
      mTorques[mLeftFoot[1]] += -k2 * offset + kd * (mPreOffset - offset);
      mTorques[mRightFoot[0]] += -k1 * offset + kd * (mPreOffset - offset);
      mTorques[mRightFoot[1]] += -k2 * offset + kd * (mPreOffset - offset);
      mPreOffset = offset;
    }
    else if (offset > -0.2 && offset < -0.05)
    {
      double k1 = 2000.0;
      double k2 = 100.0;
      double kd = 100.0;
      mTorques[mLeftFoot[0]] += -k1 * offset + kd * (mPreOffset - offset);
      mTorques[mLeftFoot[1]] += -k2 * offset + kd * (mPreOffset - offset);
      mTorques[mRightFoot[0]] += -k1 * offset + kd * (mPreOffset - offset);
      mTorques[mRightFoot[1]] += -k2 * offset + kd * (mPreOffset - offset);
      mPreOffset = offset;
    }

    // Just to make sure no illegal torque is used
    for (int i = 0; i < 6; i++)
    {
      mTorques[i] = 0.0;
    }

    mBiped->setForces(mTorques);

    mFrame++;
  }

  void customPostStep()
  {
    // Use this function to execute custom code after each simulation time
    // step is performed. This function can be deleted if it does not need
    // to be used.
  }

  void perturbBiped(const Eigen::Vector3d& force, int frames = 100)
  {
    mForce = force;
    mImpulseDuration = frames;
  }

protected:
  dart::dynamics::SkeletonPtr mBiped;

  dart::dynamics::BodyNodePtr mLeftHeel;
  Eigen::VectorXd mTorques;
  Eigen::VectorXd mDesiredDofs;
  Eigen::MatrixXd mKp;
  Eigen::MatrixXd mKd;
  std::size_t mLeftFoot[2];
  std::size_t mRightFoot[2];
  int mFrame;
  double mTimestep;
  double mPreOffset;

  int mImpulseDuration;
  Eigen::Vector3d mForce;
};

//==============================================================================
class CustomEventHandler : public osgGA::GUIEventHandler
{
public:
  CustomEventHandler(CustomWorldNode* worldNode)
  {
    mWorldNode = worldNode;
  }

  bool handle(
      const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter&) override
  {
    if (ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN)
    {
      if (ea.getKey() == '1')
      {
        mWorldNode->perturbBiped(Eigen::Vector3d(50, 0, 0), 100);
        return true;
      }
      else if (ea.getKey() == '2')
      {
        mWorldNode->perturbBiped(Eigen::Vector3d(-50, 0, 0), 100);
        return true;
      }
      else if (ea.getKey() == '3')
      {
        mWorldNode->perturbBiped(Eigen::Vector3d(0, 0, 50), 100);
        return true;
      }
      else if (ea.getKey() == '4')
      {
        mWorldNode->perturbBiped(Eigen::Vector3d(0, 0, -50), 100);
        return true;
      }
    }

    // The return value should be 'true' if the input has been fully handled
    // and should not be visible to any remaining event handlers. It should be
    // false if the input has not been fully handled and should be viewed by
    // any remaining event handlers.
    return false;
  }

private:
  CustomWorldNode* mWorldNode;
};

//==============================================================================
int main()
{
  // Create a world and add the rigid body
  auto world
      = dart::io::SkelParser::readWorld("dart://sample/skel/fullbody1.skel");
  world->setGravity(Eigen::Vector3d(0, -9.81, 0));

  auto biped = world->getSkeleton("fullbody1");
  biped = world->getSkeleton("fullbody1");
  biped->getDof("j_pelvis_rot_y")->setPosition(-0.20);
  biped->getDof("j_thigh_left_z")->setPosition(0.15);
  biped->getDof("j_shin_left")->setPosition(-0.40);
  biped->getDof("j_heel_left_1")->setPosition(0.25);
  biped->getDof("j_thigh_right_z")->setPosition(0.15);
  biped->getDof("j_shin_right")->setPosition(-0.40);
  biped->getDof("j_heel_right_1")->setPosition(0.25);
  biped->getDof("j_abdomen_2")->setPosition(0.00);

  // Create a Viewer and set it up with the WorldNode
  auto viewer = gui::osg::Viewer();
  auto shadow = gui::osg::WorldNode::createDefaultShadowTechnique(&viewer);

  // Wrap a WorldNode around it
  ::osg::ref_ptr<CustomWorldNode> node
      = new CustomWorldNode(world, biped, shadow);
  viewer.addWorldNode(node);
  viewer.addEventHandler(new CustomEventHandler(node.get()));

  viewer.addInstructionText("Press space to start simulation.\n");
  std::cout << viewer.getInstructions() << std::endl;
  std::cout << "1: Push robot with +50 along x-axis N for 100 frames\n"
            << "2: Push robot with -50 along x-axis N for 100 frames\n"
            << "3: Push robot with +50 along z-axis N for 100 frames\n"
            << "4: Push robot with -50 along z-axis N for 100 frames\n"
            << std::endl;

  // Set up the window to be 640x480
  viewer.setUpViewInWindow(0, 0, 640, 480);

  // Adjust the viewpoint of the Viewer
  viewer.getCameraManipulator()->setHomePosition(
      ::osg::Vec3(3.0f, 1.5f, 3.0f),
      ::osg::Vec3(0.0f, 0.0f, 0.0f),
      ::osg::Vec3(0.0f, 1.0f, 0.0f));

  // We need to re-dirty the CameraManipulator by passing it into the viewer
  // again, so that the viewer knows to update its HomePosition setting
  viewer.setCameraManipulator(viewer.getCameraManipulator());

  // Begin running the application loop
  viewer.run();

  return 0;
}
