/*
 * Copyright (c) 2013-2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
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

#include <array>
#include <iostream>
#include <gtest/gtest.h>
#include "TestHelpers.h"

#include "dart/math/Geometry.h"
#include "dart/math/Helpers.h"
#include "dart/dynamics/BallJoint.h"
#include "dart/dynamics/FreeJoint.h"
#include "dart/dynamics/PrismaticJoint.h"
#include "dart/dynamics/RevoluteJoint.h"
#include "dart/dynamics/TranslationalJoint.h"
#include "dart/dynamics/UniversalJoint.h"
#include "dart/dynamics/WeldJoint.h"
#include "dart/dynamics/EulerJoint.h"
#include "dart/dynamics/ScrewJoint.h"
#include "dart/dynamics/PlanarJoint.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/simulation/World.h"
#include "dart/utils/Paths.h"
#include "dart/utils/SkelParser.h"

using namespace dart;
using namespace dart::math;
using namespace dart::dynamics;
using namespace dart::simulation;

#define JOINT_TOL 0.01

//==============================================================================
class JOINTS : public testing::Test
{
public:
  void kinematicsTest(Joint* _joint);
};

//==============================================================================
void JOINTS::kinematicsTest(Joint* _joint)
{
  assert(_joint);

  int numTests = 1;

  _joint->setTransformFromChildBodyNode(
        math::expMap(Eigen::Vector6d::Random()));
  _joint->setTransformFromParentBodyNode(
        math::expMap(Eigen::Vector6d::Random()));

  BodyNode* bodyNode = new BodyNode();
  bodyNode->setParentJoint(_joint);

  Skeleton skeleton;
  skeleton.addBodyNode(bodyNode);
  skeleton.init();

  int dof = _joint->getNumDofs();

  //--------------------------------------------------------------------------
  //
  //--------------------------------------------------------------------------
  VectorXd q = VectorXd::Zero(dof);
  VectorXd dq = VectorXd::Zero(dof);

  for (int idxTest = 0; idxTest < numTests; ++idxTest)
  {
    double q_delta = 0.000001;

    for (int i = 0; i < dof; ++i)
    {
      q(i) = random(-DART_PI*1.0, DART_PI*1.0);
      dq(i) = random(-DART_PI*1.0, DART_PI*1.0);
    }

    skeleton.setPositions(q);
    skeleton.setVelocities(dq);
    skeleton.computeForwardKinematics(true, true, false);

    if (_joint->getNumDofs() == 0)
      return;

    Eigen::Isometry3d T = _joint->getLocalTransform();
    Jacobian J = _joint->getLocalJacobian();
    Jacobian dJ = _joint->getLocalJacobianTimeDeriv();

    //--------------------------------------------------------------------------
    // Test T
    //--------------------------------------------------------------------------
    EXPECT_TRUE(math::verifyTransform(T));

    //--------------------------------------------------------------------------
    // Test analytic Jacobian and numerical Jacobian
    // J == numericalJ
    //--------------------------------------------------------------------------
    Jacobian numericJ = Jacobian::Zero(6,dof);
    for (int i = 0; i < dof; ++i)
    {
      // a
      Eigen::VectorXd q_a = q;
      _joint->setPositions(q_a);
      skeleton.computeForwardKinematics(true, false, false);
      Eigen::Isometry3d T_a = _joint->getLocalTransform();

      // b
      Eigen::VectorXd q_b = q;
      q_b(i) += q_delta;
      _joint->setPositions(q_b);
      skeleton.computeForwardKinematics(true, false, false);
      Eigen::Isometry3d T_b = _joint->getLocalTransform();

      //
      Eigen::Isometry3d Tinv_a = T_a.inverse();
      Eigen::Matrix4d Tinv_a_eigen = Tinv_a.matrix();

      // dTdq
      Eigen::Matrix4d T_a_eigen = T_a.matrix();
      Eigen::Matrix4d T_b_eigen = T_b.matrix();
      Eigen::Matrix4d dTdq_eigen = (T_b_eigen - T_a_eigen) / q_delta;
      //Matrix4d dTdq_eigen = (T_b_eigen * T_a_eigen.inverse()) / dt;

      // J(i)
      Eigen::Matrix4d Ji_4x4matrix_eigen = Tinv_a_eigen * dTdq_eigen;
      Eigen::Vector6d Ji;
      Ji[0] = Ji_4x4matrix_eigen(2,1);
      Ji[1] = Ji_4x4matrix_eigen(0,2);
      Ji[2] = Ji_4x4matrix_eigen(1,0);
      Ji[3] = Ji_4x4matrix_eigen(0,3);
      Ji[4] = Ji_4x4matrix_eigen(1,3);
      Ji[5] = Ji_4x4matrix_eigen(2,3);
      numericJ.col(i) = Ji;
    }

    for (int i = 0; i < dof; ++i)
    {
      for (int j = 0; j < 6; ++j)
        EXPECT_NEAR(J.col(i)(j), numericJ.col(i)(j), JOINT_TOL);
    }

    //--------------------------------------------------------------------------
    // Test first time derivative of analytic Jacobian and numerical Jacobian
    // dJ == numerical_dJ
    //--------------------------------------------------------------------------
    Jacobian numeric_dJ = Jacobian::Zero(6,dof);
    for (int i = 0; i < dof; ++i)
    {
      // a
      Eigen::VectorXd q_a = q;
      _joint->setPositions(q_a);
      skeleton.computeForwardKinematics(true, false, false);
      Jacobian J_a = _joint->getLocalJacobian();

      // b
      Eigen::VectorXd q_b = q;
      q_b(i) += q_delta;
      _joint->setPositions(q_b);
      skeleton.computeForwardKinematics(true, false, false);
      Jacobian J_b = _joint->getLocalJacobian();

      //
      Jacobian dJ_dq = (J_b - J_a) / q_delta;

      // J(i)
      numeric_dJ += dJ_dq * dq(i);
    }

    for (int i = 0; i < dof; ++i)
    {
      for (int j = 0; j < 6; ++j)
        EXPECT_NEAR(dJ.col(i)(j), numeric_dJ.col(i)(j), JOINT_TOL);
    }
  }

  // Forward kinematics test with high joint position
  double posMin = -1e+64;
  double posMax = +1e+64;

  for (int idxTest = 0; idxTest < numTests; ++idxTest)
  {
    for (int i = 0; i < dof; ++i)
      q(i) = random(posMin, posMax);

    skeleton.setPositions(q);
    skeleton.computeForwardKinematics(true, false, false);

    if (_joint->getNumDofs() == 0)
      return;

    Eigen::Isometry3d T = _joint->getLocalTransform();
    EXPECT_TRUE(math::verifyTransform(T));
  }
}

// 0-dof joint
TEST_F(JOINTS, WELD_JOINT)
{
  WeldJoint* weldJoint = new WeldJoint;

  kinematicsTest(weldJoint);
}

// 1-dof joint
TEST_F(JOINTS, REVOLUTE_JOINT)
{
  RevoluteJoint* revJoint = new RevoluteJoint;

  kinematicsTest(revJoint);
}

// 1-dof joint
TEST_F(JOINTS, PRISMATIC_JOINT)
{
  PrismaticJoint* priJoint = new PrismaticJoint;

  kinematicsTest(priJoint);
}

// 1-dof joint
TEST_F(JOINTS, SCREW_JOINT)
{
  ScrewJoint* screwJoint = new ScrewJoint;

  kinematicsTest(screwJoint);
}

// 2-dof joint
TEST_F(JOINTS, UNIVERSAL_JOINT)
{
  UniversalJoint* univJoint = new UniversalJoint;

  kinematicsTest(univJoint);
}

// 3-dof joint
TEST_F(JOINTS, BALL_JOINT)
{
  BallJoint* ballJoint = new BallJoint;

  kinematicsTest(ballJoint);
}

// 3-dof joint
TEST_F(JOINTS, EULER_JOINT)
{
  EulerJoint* eulerJoint1 = new EulerJoint;

  eulerJoint1->setAxisOrder(EulerJoint::AO_XYZ);
  kinematicsTest(eulerJoint1);

  EulerJoint* eulerJoint2 = new EulerJoint;

  eulerJoint2->setAxisOrder(EulerJoint::AO_ZYX);
  kinematicsTest(eulerJoint2);
}

// 3-dof joint
TEST_F(JOINTS, TRANSLATIONAL_JOINT)
{
  TranslationalJoint* translationalJoint = new TranslationalJoint;

  kinematicsTest(translationalJoint);
}

// 3-dof joint
TEST_F(JOINTS, PLANAR_JOINT)
{
  PlanarJoint* planarJoint = new PlanarJoint;

  kinematicsTest(planarJoint);
}

// 6-dof joint
TEST_F(JOINTS, FREE_JOINT)
{
  FreeJoint* freeJoint = new FreeJoint;

  kinematicsTest(freeJoint);
}

//==============================================================================
TEST_F(JOINTS, POSITION_LIMIT)
{
  double tol = 1e-3;

  simulation::World* myWorld
      = utils::SkelParser::readWorld(
          DART_DATA_PATH"/skel/test/joint_limit_test.skel");
  EXPECT_TRUE(myWorld != NULL);

  myWorld->setGravity(Eigen::Vector3d(0.0, 0.0, 0.0));

  dynamics::Skeleton* pendulum = myWorld->getSkeleton("double_pendulum");
  EXPECT_TRUE(pendulum != NULL);

  dynamics::Joint* joint0 = pendulum->getJoint("joint0");
  dynamics::Joint* joint1 = pendulum->getJoint("joint1");

  EXPECT_TRUE(joint0 != NULL);
  EXPECT_TRUE(joint1 != NULL);

  double limit0 = DART_PI / 6.0;
  double limit1 = DART_PI / 6.0;

  joint0->setPositionLimited(true);
  joint0->setPositionLowerLimit(0, -limit0);
  joint0->setPositionUpperLimit(0, limit0);

  joint1->setPositionLimited(true);
  joint1->setPositionLowerLimit(0, -limit1);
  joint1->setPositionUpperLimit(0, limit1);

  double simTime = 2.0;
  double timeStep = myWorld->getTimeStep();
  int nSteps = simTime / timeStep;

  // Two seconds with positive control forces
  for (int i = 0; i < nSteps; i++)
  {
    joint0->setForce(0, 0.1);
    joint1->setForce(0, 0.1);
    myWorld->step();

    double jointPos0 = joint0->getPosition(0);
    double jointPos1 = joint1->getPosition(0);

    EXPECT_GE(jointPos0, -limit0 - tol);
    EXPECT_GE(jointPos1, -limit1 - tol);

    EXPECT_LE(jointPos0, limit0 + tol);
    EXPECT_LE(jointPos1, limit1 + tol);
  }

  // Two more seconds with negative control forces
  for (int i = 0; i < nSteps; i++)
  {
    joint0->setForce(0, -0.1);
    joint1->setForce(0, -0.1);
    myWorld->step();

    double jointPos0 = joint0->getPosition(0);
    double jointPos1 = joint1->getPosition(0);

    EXPECT_GE(jointPos0, -limit0 - tol);
    EXPECT_GE(jointPos1, -limit1 - tol);

    EXPECT_LE(jointPos0, limit0 + tol);
    EXPECT_LE(jointPos1, limit1 + tol);
  }
}

//==============================================================================
void testJointCoulombFrictionForce(double _timeStep)
{
  double tol = 1e-9;

  simulation::World* myWorld
      = utils::SkelParser::readWorld(
          DART_DATA_PATH"/skel/test/joint_friction_test.skel");
  EXPECT_TRUE(myWorld != NULL);

  myWorld->setGravity(Eigen::Vector3d(0.0, 0.0, 0.0));
  myWorld->setTimeStep(_timeStep);

  dynamics::Skeleton* pendulum = myWorld->getSkeleton("double_pendulum");
  EXPECT_TRUE(pendulum != NULL);
  pendulum->disableSelfCollision();

  dynamics::Joint* joint0 = pendulum->getJoint("joint0");
  dynamics::Joint* joint1 = pendulum->getJoint("joint1");

  EXPECT_TRUE(joint0 != NULL);
  EXPECT_TRUE(joint1 != NULL);

  double frictionForce  = 5.0;

  joint0->setPositionLimited(false);
  joint1->setPositionLimited(false);

  joint0->setCoulombFriction(0, frictionForce);
  joint1->setCoulombFriction(0, frictionForce);

  EXPECT_EQ(joint0->getCoulombFriction(0), frictionForce);
  EXPECT_EQ(joint1->getCoulombFriction(0), frictionForce);

  double simTime = 2.0;
  double timeStep = myWorld->getTimeStep();
  int nSteps = simTime / timeStep;

  // Two seconds with lower control forces than the friction
  for (int i = 0; i < nSteps; i++)
  {
    joint0->setForce(0, +4.9);
    joint1->setForce(0, +4.9);
    myWorld->step();

    double jointVel0 = joint0->getVelocity(0);
    double jointVel1 = joint1->getVelocity(0);

    EXPECT_NEAR(jointVel0, 0.0, tol);
    EXPECT_NEAR(jointVel1, 0.0, tol);
  }

  // Another two seconds with lower control forces than the friction forces
  for (int i = 0; i < nSteps; i++)
  {
    joint0->setForce(0, -4.9);
    joint1->setForce(0, -4.9);
    myWorld->step();

    double jointVel0 = joint0->getVelocity(0);
    double jointVel1 = joint1->getVelocity(0);

    EXPECT_NEAR(jointVel0, 0.0, tol);
    EXPECT_NEAR(jointVel1, 0.0, tol);
  }

  // Another two seconds with higher control forces than the friction forces
  for (int i = 0; i < nSteps; i++)
  {
    joint0->setForce(0, 10.0);
    joint1->setForce(0, 10.0);
    myWorld->step();

    double jointVel0 = joint0->getVelocity(0);
    double jointVel1 = joint1->getVelocity(0);

    EXPECT_GE(std::fabs(jointVel0), 0.0);
    EXPECT_GE(std::fabs(jointVel1), 0.0);
  }

  // Spend 20 sec waiting the joints to stop
  for (int i = 0; i < nSteps * 10; i++)
  {
    myWorld->step();
  }
  double jointVel0 = joint0->getVelocity(0);
  double jointVel1 = joint1->getVelocity(0);

  EXPECT_NEAR(jointVel0, 0.0, tol);
  EXPECT_NEAR(jointVel1, 0.0, tol);

  // Another two seconds with lower control forces than the friction forces
  // and expect the joints to stop
  for (int i = 0; i < nSteps; i++)
  {
    joint0->setForce(0, 4.9);
    joint1->setForce(0, 4.9);
    myWorld->step();

    double jointVel0 = joint0->getVelocity(0);
    double jointVel1 = joint1->getVelocity(0);

    EXPECT_NEAR(jointVel0, 0.0, tol);
    EXPECT_NEAR(jointVel1, 0.0, tol);
  }
}

//==============================================================================
TEST_F(JOINTS, JOINT_COULOMB_FRICTION)
{
  std::array<double, 3> timeSteps;
  timeSteps[0] = 1e-2;
  timeSteps[1] = 1e-3;
  timeSteps[2] = 1e-4;

  for (auto timeStep : timeSteps)
    testJointCoulombFrictionForce(timeStep);
}

//==============================================================================
template<int N>
Eigen::Matrix<double,N,1> random_vec(double limit=100)
{
  Eigen::Matrix<double,N,1> v;
  for(size_t i=0; i<N; ++i)
    v[i] = math::random(-fabs(limit), fabs(limit));
  return v;
}

//==============================================================================
Eigen::Isometry3d random_transform(double translation_limit=100,
                                   double rotation_limit=2*M_PI)
{
  Eigen::Vector3d r = random_vec<3>(translation_limit);
  Eigen::Vector3d theta = random_vec<3>(rotation_limit);

  Eigen::Isometry3d tf;
  tf.setIdentity();
  tf.translate(r);

  if(theta.norm()>0)
    tf.rotate(Eigen::AngleAxisd(theta.norm(), theta.normalized()));

  return tf;
}

Eigen::Isometry3d predict_joint_transform(Joint* joint,
                                          const Eigen::Isometry3d& joint_tf)
{
  return joint->getTransformFromParentBodyNode() * joint_tf
          * joint->getTransformFromChildBodyNode().inverse();
}

Eigen::Isometry3d get_relative_transform(BodyNode* bn, BodyNode* relativeTo)
{
  return relativeTo->getTransform().inverse() * bn->getTransform();
}

//==============================================================================
TEST_F(JOINTS, CONVENIENCE_FUNCTIONS)
{
  // -- set up the root BodyNode
  BodyNode* root = new BodyNode("root");
  WeldJoint* rootjoint = new WeldJoint("base");
  root->setParentJoint(rootjoint);

  // -- set up the FreeJoint
  BodyNode* freejoint_bn = new BodyNode("freejoint_bn");
  FreeJoint* freejoint = new FreeJoint("freejoint");

  freejoint_bn->setParentJoint(freejoint);
  root->addChildBodyNode(freejoint_bn);

  freejoint->setTransformFromParentBodyNode(random_transform());
  freejoint->setTransformFromChildBodyNode(random_transform());

  // -- set up the EulerJoint
  BodyNode* eulerjoint_bn = new BodyNode("eulerjoint_bn");
  EulerJoint* eulerjoint = new EulerJoint("eulerjoint");

  eulerjoint_bn->setParentJoint(eulerjoint);
  root->addChildBodyNode(eulerjoint_bn);

  eulerjoint->setTransformFromParentBodyNode(random_transform());
  eulerjoint->setTransformFromChildBodyNode(random_transform());

  // -- set up the BallJoint
  BodyNode* balljoint_bn = new BodyNode("balljoint_bn");
  BallJoint* balljoint = new BallJoint("balljoint");

  balljoint_bn->setParentJoint(balljoint);
  root->addChildBodyNode(balljoint_bn);

  balljoint->setTransformFromParentBodyNode(random_transform());
  balljoint->setTransformFromChildBodyNode(random_transform());

  // -- set up Skeleton and compute forward kinematics
  Skeleton* skel = new Skeleton;
  skel->addBodyNode(root);
  skel->addBodyNode(freejoint_bn);
  skel->addBodyNode(eulerjoint_bn);
  skel->addBodyNode(balljoint_bn);
  skel->init();

  // Test a hundred times
  for(size_t n=0; n<100; ++n)
  {
    // -- convert transforms to positions and then positions back to transforms
    Eigen::Isometry3d desired_freejoint_tf = random_transform();
    freejoint->setPositions(FreeJoint::convertToPositions(desired_freejoint_tf));
    Eigen::Isometry3d actual_freejoint_tf = FreeJoint::convertToTransform(
          freejoint->getPositions());

    Eigen::Isometry3d desired_eulerjoint_tf = random_transform();
    desired_eulerjoint_tf.translation() = Eigen::Vector3d::Zero();
    eulerjoint->setPositions(
          eulerjoint->convertToPositions(desired_eulerjoint_tf.linear()));
    Eigen::Isometry3d actual_eulerjoint_tf = eulerjoint->convertToTransform(
          eulerjoint->getPositions());

    Eigen::Isometry3d desired_balljoint_tf = random_transform();
    desired_balljoint_tf.translation() = Eigen::Vector3d::Zero();
    balljoint->setPositions(
          BallJoint::convertToPositions(desired_balljoint_tf.linear()));
    Eigen::Isometry3d actual_balljoint_tf = BallJoint::convertToTransform(
          balljoint->getPositions());

    skel->computeForwardKinematics(true, false, false);

    // -- collect everything so we can cycle through the tests
    std::vector<Joint*> joints;
    std::vector<BodyNode*> bns;
    std::vector<Eigen::Isometry3d> desired_tfs;
    std::vector<Eigen::Isometry3d> actual_tfs;

    joints.push_back(freejoint);
    bns.push_back(freejoint_bn);
    desired_tfs.push_back(desired_freejoint_tf);
    actual_tfs.push_back(actual_freejoint_tf);

    joints.push_back(eulerjoint);
    bns.push_back(eulerjoint_bn);
    desired_tfs.push_back(desired_eulerjoint_tf);
    actual_tfs.push_back(actual_eulerjoint_tf);

    joints.push_back(balljoint);
    bns.push_back(balljoint_bn);
    desired_tfs.push_back(desired_balljoint_tf);
    actual_tfs.push_back(actual_balljoint_tf);

    for(size_t i=0; i<joints.size(); ++i)
    {
      Joint* joint = joints[i];
      BodyNode* bn = bns[i];
      Eigen::Isometry3d tf = desired_tfs[i];

      bool check_transform_conversion =
          equals(predict_joint_transform(joint, tf).matrix(),
                 get_relative_transform(bn, bn->getParentBodyNode()).matrix());
      EXPECT_TRUE(check_transform_conversion);

      if(!check_transform_conversion)
      {
        std::cout << "[" << joint->getName() << " Failed]\n";
        std::cout << "Predicted:\n" << predict_joint_transform(joint, tf).matrix()
                  << "\n\nActual:\n"
                  << get_relative_transform(bn, bn->getParentBodyNode()).matrix()
                  << "\n\n";
      }

      bool check_full_cycle = equals(desired_tfs[i].matrix(),
                                     actual_tfs[i].matrix());
      EXPECT_TRUE(check_full_cycle);

      if(!check_full_cycle)
      {
        std::cout << "[" << joint->getName() << " Failed]\n";
        std::cout << "Desired:\n" << desired_tfs[i].matrix()
                  << "\n\nActual:\n" << actual_tfs[i].matrix()
                  << "\n\n";
      }
    }
  }
}

//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

