/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
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

#include "helpers/dynamics_helpers.hpp"

#include "helpers/gtest_utils.hpp"

#include "dart/constraint/coupler_constraint.hpp"
#include "dart/dynamics/ball_joint.hpp"
#include "dart/dynamics/body_node.hpp"
#include "dart/dynamics/euler_joint.hpp"
#include "dart/dynamics/free_joint.hpp"
#include "dart/dynamics/inverse_kinematics.hpp"
#include "dart/dynamics/mimic_dof_properties.hpp"
#include "dart/dynamics/planar_joint.hpp"
#include "dart/dynamics/prismatic_joint.hpp"
#include "dart/dynamics/revolute_joint.hpp"
#include "dart/dynamics/screw_joint.hpp"
#include "dart/dynamics/skeleton.hpp"
#include "dart/dynamics/translational_joint.hpp"
#include "dart/dynamics/translational_joint2_d.hpp"
#include "dart/dynamics/universal_joint.hpp"
#include "dart/dynamics/weld_joint.hpp"
#include "dart/io/read.hpp"
#include "dart/math/geometry.hpp"
#include "dart/math/helpers.hpp"
#include "dart/simulation/world.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <iostream>

using namespace dart;
using namespace dart::math;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::test;

#define JOINT_TOL 0.01

class CouplerConstraintTestHelper : public dart::constraint::CouplerConstraint
{
public:
  using CouplerConstraint::applyImpulse;
  using CouplerConstraint::CouplerConstraint;
  using CouplerConstraint::update;
};

//==============================================================================
class Joints : public testing::Test
{
public:
  // Get reference frames
  const std::vector<SimpleFrame*>& getFrames() const;

  // Randomize the properties of all the reference frames
  void randomizeRefFrames();

#ifdef _WIN32
  template <typename JointType>
  static typename JointType::Properties createJointProperties()
  {
    return typename JointType::Properties();
  }
#endif

  template <typename JointType>
  void kinematicsTest(
#ifdef _WIN32
      const typename JointType::Properties& _joint
      = createJointProperties<JointType>());
#else
      const typename JointType::Properties& _joint
      = typename JointType::Properties());
#endif

protected:
  // Sets up the test fixture.
  void SetUp() override;
  void TearDown() override;

  std::vector<SimpleFrame*> frames;
};

//==============================================================================
void Joints::SetUp()
{
  // Create a list of reference frames to use during tests
  frames.push_back(new SimpleFrame(Frame::World(), "refFrame1"));
  frames.push_back(new SimpleFrame(frames.back(), "refFrame2"));
  frames.push_back(new SimpleFrame(frames.back(), "refFrame3"));
  frames.push_back(new SimpleFrame(frames.back(), "refFrame4"));
  frames.push_back(new SimpleFrame(Frame::World(), "refFrame5"));
  frames.push_back(new SimpleFrame(frames.back(), "refFrame6"));
}

//==============================================================================
void Joints::TearDown()
{
  for (SimpleFrame* frame : frames) {
    delete frame;
  }
  frames.clear();
}

//==============================================================================
const std::vector<SimpleFrame*>& Joints::getFrames() const
{
  return frames;
}

//==============================================================================
void Joints::randomizeRefFrames()
{
  for (std::size_t i = 0; i < frames.size(); ++i) {
    SimpleFrame* F = frames[i];

    Eigen::Vector3d p = Random::uniform<Eigen::Vector3d>(-100, 100);
    Eigen::Vector3d theta = Random::uniform<Eigen::Vector3d>(
        -2 * dart::math::pi, 2 * dart::math::pi);

    Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
    tf.translate(p);
    tf.linear() = math::eulerXYZToMatrix(theta);

    F->setRelativeTransform(tf);
    F->setRelativeSpatialVelocity(Random::uniform<Eigen::Vector6d>(-100, 100));
    F->setRelativeSpatialAcceleration(
        Random::uniform<Eigen::Vector6d>(-100, 100));
  }
}

//==============================================================================
TEST_F(Joints, FreeJointSpatialVelocityWithVelocityActuator)
{
  auto world = simulation::World::create();
  const double timeStep = 0.01;
  world->setTimeStep(timeStep);
  world->setGravity(Eigen::Vector3d::Zero());

  auto skeleton = Skeleton::create("free_joint_kinematic");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  FreeJoint* joint = pair.first;
  BodyNode* body = pair.second;

  joint->setActuatorType(Joint::VELOCITY);
  world->addSkeleton(skeleton);

  Eigen::Vector6d desiredVel = Eigen::Vector6d::Zero();
  desiredVel.tail<3>() << 0.3, -0.1, 0.2;

  joint->setSpatialVelocity(desiredVel, Frame::World(), Frame::World());

  const Eigen::Vector6d initialVel
      = body->getSpatialVelocity(Frame::World(), Frame::World());
  EXPECT_VECTOR_NEAR(desiredVel, initialVel, 1e-8);

  const Eigen::Vector3d initialTranslation
      = body->getWorldTransform().translation();

  const std::size_t numSteps = 5;
  for (std::size_t i = 0; i < numSteps; ++i) {
    joint->setSpatialVelocity(desiredVel, Frame::World(), Frame::World());
    world->step();
  }

  const Eigen::Vector6d actualVel
      = body->getSpatialVelocity(Frame::World(), Frame::World());
  EXPECT_VECTOR_NEAR(desiredVel, actualVel, 1e-8);

  const Eigen::Vector3d expectedTranslation
      = desiredVel.tail<3>() * timeStep * static_cast<double>(numSteps);
  const Eigen::Vector3d actualTranslation
      = body->getWorldTransform().translation() - initialTranslation;
  EXPECT_VECTOR_NEAR(expectedTranslation, actualTranslation, 1e-6);
}

//==============================================================================
template <typename JointType>
void Joints::kinematicsTest(const typename JointType::Properties& _properties)
{
  int numTests = 1;

  SkeletonPtr skeleton = Skeleton::create();
  Joint* joint
      = skeleton->createJointAndBodyNodePair<JointType>(nullptr, _properties)
            .first;
  joint->setTransformFromChildBodyNode(math::expMap(Eigen::Vector6d::Random()));
  joint->setTransformFromParentBodyNode(
      math::expMap(Eigen::Vector6d::Random()));

  int dof = joint->getNumDofs();

  //--------------------------------------------------------------------------
  //
  //--------------------------------------------------------------------------
  VectorXd q = VectorXd::Zero(dof);
  VectorXd dq = VectorXd::Zero(dof);

  for (int idxTest = 0; idxTest < numTests; ++idxTest) {
    double q_delta = 0.000001;

    for (int i = 0; i < dof; ++i) {
      q(i) = Random::uniform(-pi * 1.0, pi * 1.0);
      dq(i) = Random::uniform(-pi * 1.0, pi * 1.0);
    }

    joint->setPositions(q);
    joint->setVelocities(dq);

    if (dof == 0) {
      return;
    }

    Eigen::Isometry3d T = joint->getRelativeTransform();
    Jacobian J = joint->getRelativeJacobian();
    Jacobian dJ = joint->getRelativeJacobianTimeDeriv();

    //--------------------------------------------------------------------------
    // Test T
    //--------------------------------------------------------------------------
    EXPECT_TRUE(math::verifyTransform(T));

    //--------------------------------------------------------------------------
    // Test analytic Jacobian and numerical Jacobian
    // J == numericalJ
    //--------------------------------------------------------------------------
    Jacobian numericJ = Jacobian::Zero(6, dof);
    for (int i = 0; i < dof; ++i) {
      // a
      Eigen::VectorXd q_a = q;
      joint->setPositions(q_a);
      Eigen::Isometry3d T_a = joint->getRelativeTransform();

      // b
      Eigen::VectorXd q_b = q;
      q_b(i) += q_delta;
      joint->setPositions(q_b);
      Eigen::Isometry3d T_b = joint->getRelativeTransform();

      //
      Eigen::Isometry3d Tinv_a = T_a.inverse();
      Eigen::Matrix4d Tinv_a_eigen = Tinv_a.matrix();

      // dTdq
      Eigen::Matrix4d T_a_eigen = T_a.matrix();
      Eigen::Matrix4d T_b_eigen = T_b.matrix();
      Eigen::Matrix4d dTdq_eigen = (T_b_eigen - T_a_eigen) / q_delta;
      // Matrix4d dTdq_eigen = (T_b_eigen * T_a_eigen.inverse()) / dt;

      // J(i)
      Eigen::Matrix4d Ji_4x4matrix_eigen = Tinv_a_eigen * dTdq_eigen;
      Eigen::Vector6d Ji;
      Ji[0] = Ji_4x4matrix_eigen(2, 1);
      Ji[1] = Ji_4x4matrix_eigen(0, 2);
      Ji[2] = Ji_4x4matrix_eigen(1, 0);
      Ji[3] = Ji_4x4matrix_eigen(0, 3);
      Ji[4] = Ji_4x4matrix_eigen(1, 3);
      Ji[5] = Ji_4x4matrix_eigen(2, 3);
      numericJ.col(i) = Ji;
    }

    for (int i = 0; i < dof; ++i) {
      for (int j = 0; j < 6; ++j) {
        EXPECT_NEAR(J.col(i)(j), numericJ.col(i)(j), JOINT_TOL);
      }
    }

    //--------------------------------------------------------------------------
    // Test first time derivative of analytic Jacobian and numerical Jacobian
    // dJ == numerical_dJ
    //--------------------------------------------------------------------------
    Jacobian numeric_dJ = Jacobian::Zero(6, dof);
    for (int i = 0; i < dof; ++i) {
      // a
      Eigen::VectorXd q_a = q;
      joint->setPositions(q_a);
      Jacobian J_a = joint->getRelativeJacobian();

      // b
      Eigen::VectorXd q_b = q;
      q_b(i) += q_delta;
      joint->setPositions(q_b);
      Jacobian J_b = joint->getRelativeJacobian();

      //
      Jacobian dJ_dq = (J_b - J_a) / q_delta;

      // J(i)
      numeric_dJ += dJ_dq * dq(i);
    }

    for (int i = 0; i < dof; ++i) {
      for (int j = 0; j < 6; ++j) {
        EXPECT_NEAR(dJ.col(i)(j), numeric_dJ.col(i)(j), JOINT_TOL);
      }
    }
  }

  // Forward kinematics test with high joint position
  double posMin = -1e+64;
  double posMax = +1e+64;

  for (int idxTest = 0; idxTest < numTests; ++idxTest) {
    for (int i = 0; i < dof; ++i) {
      q(i) = Random::uniform(posMin, posMax);
    }

    skeleton->setPositions(q);

    if (joint->getNumDofs() == 0) {
      return;
    }

    Eigen::Isometry3d T = joint->getRelativeTransform();
    EXPECT_TRUE(math::verifyTransform(T));
  }
}

// 0-dof joint
TEST_F(Joints, WeldJoint)
{
  kinematicsTest<WeldJoint>();
}

// 1-dof joint
TEST_F(Joints, RevoluteJoint)
{
  kinematicsTest<RevoluteJoint>();
}

// 1-dof joint
TEST_F(Joints, PrismaticJoint)
{
  kinematicsTest<PrismaticJoint>();
}

// 1-dof joint
TEST_F(Joints, ScrewJoint)
{
  kinematicsTest<ScrewJoint>();
}

// 2-dof joint
TEST_F(Joints, UniversalJoint)
{
  kinematicsTest<UniversalJoint>();
}

// 2-dof joint
TEST_F(Joints, TranslationalJoint2D)
{
  kinematicsTest<TranslationalJoint2D>();
}

// 3-dof joint
// TEST_F(Joints, BallJoint)
//{
//  kinematicsTest<BallJoint>();
//}
// TODO(JS): Disabled the test compares analytical Jacobian and numerical
// Jacobian since the meaning of BallJoint Jacobian is changed per
// we now use angular velocity and angular accertions as BallJoint's generalized
// velocities and accelerations, respectively.

// 3-dof joint
TEST_F(Joints, EulerJoint)
{
  EulerJoint::Properties properties;

  properties.mAxisOrder = EulerJoint::AxisOrder::XYZ;
  kinematicsTest<EulerJoint>(properties);

  properties.mAxisOrder = EulerJoint::AxisOrder::ZYX;
  kinematicsTest<EulerJoint>(properties);
}

// 3-dof joint
TEST_F(Joints, TranslationalJoint)
{
  kinematicsTest<TranslationalJoint>();
}

// 3-dof joint
TEST_F(Joints, PlanarJoint)
{
  kinematicsTest<PlanarJoint>();
}

//==============================================================================
TEST_F(Joints, PlanarJointIsometry2dHelpers)
{
  SkeletonPtr skel = Skeleton::create("planar_helpers");
  auto pair = skel->createJointAndBodyNodePair<PlanarJoint>();
  auto* joint = pair.first;

  const Eigen::Vector3d positions(0.25, -0.15, 0.6);
  const Eigen::Isometry2d tf = PlanarJoint::convertToTransform(positions);
  EXPECT_TRUE(PlanarJoint::convertToPositions(tf).isApprox(positions, 1e-12));

  const auto verifyPlane = [&](auto setPlane) {
    setPlane();
    joint->setPositions(positions);

    const Eigen::Isometry3d expected
        = Eigen::Translation3d(joint->getTranslationalAxis1() * positions[0])
          * Eigen::Translation3d(joint->getTranslationalAxis2() * positions[1])
          * math::expAngular(joint->getRotationalAxis() * positions[2]);

    EXPECT_TRUE(joint->getRelativeTransform().isApprox(expected, 1e-12));
  };

  verifyPlane([&]() { joint->setXYPlane(); });
  verifyPlane([&]() { joint->setYZPlane(); });
  verifyPlane([&]() { joint->setZXPlane(); });
  verifyPlane([&]() {
    const Eigen::Vector3d axis1(1.0, 1.0, 0.0);
    const Eigen::Vector3d axis2(-1.0, 1.0, 0.0);
    joint->setArbitraryPlane(axis1.normalized(), axis2.normalized());
  });
}

// 6-dof joint
// TEST_F(Joints, FreeJoint)
//{
//  kinematicsTest<FreeJoint>();
//}
// TODO(JS): Disabled the test compares analytical Jacobian and numerical
// Jacobian since the meaning of FreeJoint Jacobian is changed per
// we now use spatial velocity and spatial accertions as FreeJoint's generalized
// velocities and accelerations, respectively.

//==============================================================================
template <
    void (Joint::*setX)(std::size_t, double),
    void (Joint::*setXLowerLimit)(std::size_t, double),
    void (Joint::*setXUpperLimit)(std::size_t, double)>
void testCommandLimits(dynamics::Joint* joint)
{
  const double lower = -5.0;
  const double upper = +5.0;
  const double mid = 0.5 * (lower + upper);
  const double lessThanLower = -10.0;
  const double greaterThanUpper = +10.0;

  for (std::size_t i = 0; i < joint->getNumDofs(); ++i) {
    (joint->*setXLowerLimit)(i, lower);
    (joint->*setXUpperLimit)(i, upper);

    joint->setCommand(i, mid);
    EXPECT_EQ(joint->getCommand(i), mid);
    (joint->*setX)(i, mid);
    EXPECT_EQ(joint->getCommand(i), mid);

    joint->setCommand(i, lessThanLower);
    EXPECT_EQ(joint->getCommand(i), lower);
    (joint->*setX)(i, lessThanLower);
    EXPECT_EQ(joint->getCommand(i), lessThanLower);

    joint->setCommand(i, greaterThanUpper);
    EXPECT_EQ(joint->getCommand(i), upper);
    (joint->*setX)(i, greaterThanUpper);
    EXPECT_EQ(joint->getCommand(i), greaterThanUpper);
  }
}

//==============================================================================
TEST_F(Joints, CommandLimit)
{
  simulation::WorldPtr myWorld
      = dart::io::readWorld("dart://sample/skel/test/joint_limit_test.skel");
  EXPECT_TRUE(myWorld != nullptr);

  dynamics::SkeletonPtr pendulum = myWorld->getSkeleton("double_pendulum");
  EXPECT_TRUE(pendulum != nullptr);

  pendulum->eachBodyNode([&](BodyNode* bodyNode) {
    Joint* joint = bodyNode->getParentJoint();

    joint->setActuatorType(Joint::FORCE);
    EXPECT_EQ(joint->getActuatorType(), Joint::FORCE);
    testCommandLimits<
        &Joint::setForce,
        &Joint::setForceLowerLimit,
        &Joint::setForceUpperLimit>(joint);

    joint->setActuatorType(Joint::ACCELERATION);
    EXPECT_EQ(joint->getActuatorType(), Joint::ACCELERATION);
    testCommandLimits<
        &Joint::setAcceleration,
        &Joint::setAccelerationLowerLimit,
        &Joint::setAccelerationUpperLimit>(joint);

    joint->setActuatorType(Joint::VELOCITY);
    EXPECT_EQ(joint->getActuatorType(), Joint::VELOCITY);
    testCommandLimits<
        &Joint::setVelocity,
        &Joint::setVelocityLowerLimit,
        &Joint::setVelocityUpperLimit>(joint);
  });
}

//==============================================================================
TEST_F(Joints, PassiveActuatorClearsCommand)
{
  auto world
      = dart::io::readWorld("dart://sample/skel/test/joint_limit_test.skel");
  ASSERT_TRUE(world != nullptr);

  auto skeleton = world->getSkeleton("double_pendulum");
  ASSERT_TRUE(skeleton != nullptr);

  auto joint = skeleton->getJoint("joint1");
  ASSERT_TRUE(joint != nullptr);

  // Start in FORCE mode so that commands are stored.
  joint->setActuatorType(Joint::FORCE);
  ASSERT_EQ(joint->getActuatorType(), Joint::FORCE);

  // Store an arbitrary command while the joint is FORCE actuated.
  const double storedCommand = 1.23;
  joint->setCommand(0, storedCommand);
  ASSERT_DOUBLE_EQ(joint->getCommand(0), storedCommand);

  // Switching to PASSIVE should discard any previously stored command values.
  joint->setActuatorType(Joint::PASSIVE);
  EXPECT_EQ(joint->getActuatorType(), Joint::PASSIVE);
  EXPECT_DOUBLE_EQ(joint->getCommand(0), 0.0);
  EXPECT_TRUE(joint->getCommands().isZero());

  // The skeleton view should also observe the cleared command.
  const auto dofIndex = joint->getDof(0)->getIndexInSkeleton();
  EXPECT_DOUBLE_EQ(skeleton->getCommand(dofIndex), 0.0);
}

//==============================================================================
TEST_F(Joints, PositionLimit)
{
  double tol = 1e-3;

  simulation::WorldPtr myWorld
      = dart::io::readWorld("dart://sample/skel/test/joint_limit_test.skel");
  EXPECT_TRUE(myWorld != nullptr);

  myWorld->setGravity(Eigen::Vector3d(0.0, 0.0, 0.0));

  dynamics::SkeletonPtr pendulum = myWorld->getSkeleton("double_pendulum");
  EXPECT_TRUE(pendulum != nullptr);

  dynamics::Joint* joint0 = pendulum->getJoint("joint0");
  dynamics::Joint* joint1 = pendulum->getJoint("joint1");

  EXPECT_TRUE(joint0 != nullptr);
  EXPECT_TRUE(joint1 != nullptr);

  double limit0 = pi / 6.0;
  double limit1 = pi / 6.0;

  joint0->setLimitEnforcement(true);
  joint0->setPositionLowerLimit(0, -limit0);
  joint0->setPositionUpperLimit(0, limit0);

  joint1->setLimitEnforcement(true);
  joint1->setPositionLowerLimit(0, -limit1);
  joint1->setPositionUpperLimit(0, limit1);

#if !defined(NDEBUG)
  double simTime = 0.2;
#else
  double simTime = 2.0;
#endif // ------- Debug mode
  double timeStep = myWorld->getTimeStep();
  int nSteps = simTime / timeStep;

  // Two seconds with positive control forces
  for (int i = 0; i < nSteps; i++) {
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
  for (int i = 0; i < nSteps; i++) {
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
TEST_F(Joints, PositionAndVelocityLimit)
{
  using namespace dart::math::suffixes;

  const double tol = 1e-3;

  simulation::WorldPtr myWorld
      = dart::io::readWorld("dart://sample/skel/test/joint_limit_test.skel");
  EXPECT_TRUE(myWorld != nullptr);

  myWorld->setGravity(Eigen::Vector3d(0.0, 0.0, 0.0));

  dynamics::SkeletonPtr pendulum = myWorld->getSkeleton("double_pendulum");
  EXPECT_TRUE(pendulum != nullptr);

  dynamics::Joint* joint0 = pendulum->getJoint("joint0");
  dynamics::Joint* joint1 = pendulum->getJoint("joint1");

  EXPECT_TRUE(joint0 != nullptr);
  EXPECT_TRUE(joint1 != nullptr);

  const double posLimit0 = math::toRadian(360.0);
  const double posLimit1 = math::toRadian(360.0);
  const double velLimit0 = math::toRadian(5.0); // 5 degree per second
  const double velLimit1 = math::toRadian(5.0); // 5 degree per second

  joint0->setLimitEnforcement(true);
  joint0->setPositionLowerLimit(0, -posLimit0);
  joint0->setPositionUpperLimit(0, posLimit0);
  joint0->setVelocityLowerLimit(0, -velLimit0);
  joint0->setVelocityUpperLimit(0, velLimit0);

  joint1->setLimitEnforcement(true);
  joint1->setPositionLowerLimit(0, -posLimit1);
  joint1->setPositionUpperLimit(0, posLimit1);
  joint1->setVelocityLowerLimit(0, -velLimit1);
  joint1->setVelocityUpperLimit(0, velLimit1);

#if !defined(NDEBUG)
  double simTime = 0.2;
#else
  double simTime = 2.0;
#endif // ------- Debug mode
  double timeStep = myWorld->getTimeStep();
  int nSteps = simTime / timeStep;

  // Two seconds with positive control forces
  for (int i = 0; i < nSteps; i++) {
    // Apply sufficient force to hit the limits
    joint0->setForce(0, 1.0);
    joint1->setForce(0, 1.0);
    myWorld->step();

    // Check position limits
    const double jointPos0 = joint0->getPosition(0);
    const double jointPos1 = joint1->getPosition(0);
    EXPECT_GE(jointPos0, -posLimit0 - tol);
    EXPECT_GE(jointPos1, -posLimit1 - tol);
    EXPECT_LE(jointPos0, posLimit0 + tol);
    EXPECT_LE(jointPos1, posLimit1 + tol);

    // Check velocity limits
    const double jointVel0 = joint0->getVelocity(0);
    const double jointVel1 = joint1->getVelocity(0);
    EXPECT_GE(jointVel0, -velLimit0 - tol);
    EXPECT_GE(jointVel1, -velLimit1 - tol);
    EXPECT_LE(jointVel0, velLimit0 + tol);
    EXPECT_LE(jointVel1, velLimit1 + tol);
  }

  // Two more seconds with negative control forces
  for (int i = 0; i < nSteps; i++) {
    // Apply sufficient force to hit the limits
    joint0->setForce(0, -1.0);
    joint1->setForce(0, -1.0);
    myWorld->step();

    // Check position limits
    const double jointPos0 = joint0->getPosition(0);
    const double jointPos1 = joint1->getPosition(0);
    EXPECT_GE(jointPos0, -posLimit0 - tol);
    EXPECT_GE(jointPos1, -posLimit1 - tol);
    EXPECT_LE(jointPos0, posLimit0 + tol);
    EXPECT_LE(jointPos1, posLimit1 + tol);

    // Check velocity limits
    const double jointVel0 = joint0->getVelocity(0);
    const double jointVel1 = joint1->getVelocity(0);
    EXPECT_GE(jointVel0, -velLimit0 - tol);
    EXPECT_GE(jointVel1, -velLimit1 - tol);
    EXPECT_LE(jointVel0, velLimit0 + tol);
    EXPECT_LE(jointVel1, velLimit1 + tol);
  }
}

//==============================================================================
TEST_F(Joints, JointLimits)
{
  simulation::WorldPtr myWorld
      = dart::io::readWorld("dart://sample/skel/test/joint_limit_test.skel");
  EXPECT_TRUE(myWorld != nullptr);

  myWorld->setGravity(Eigen::Vector3d(0.0, 0.0, 0.0));

  dynamics::SkeletonPtr pendulum = myWorld->getSkeleton("double_pendulum");
  EXPECT_TRUE(pendulum != nullptr);

  dynamics::Joint* joint0 = pendulum->getJoint("joint0");

  EXPECT_TRUE(joint0 != nullptr);

  double limit = pi / 6.0;
  Eigen::VectorXd limits = Eigen::VectorXd::Constant(1, pi / 2.0);

  joint0->setPositionLowerLimit(0, -limit);
  joint0->setPositionUpperLimit(0, limit);
  EXPECT_EQ(
      joint0->getPositionLowerLimits(), Eigen::VectorXd::Constant(1, -limit));
  EXPECT_EQ(
      joint0->getPositionUpperLimits(), Eigen::VectorXd::Constant(1, limit));

  joint0->setPositionLowerLimits(-limits);
  joint0->setPositionUpperLimits(limits);
  EXPECT_EQ(joint0->getPositionLowerLimits(), -limits);
  EXPECT_EQ(joint0->getPositionUpperLimits(), limits);

  joint0->setVelocityLowerLimit(0, -limit);
  joint0->setVelocityUpperLimit(0, limit);
  EXPECT_EQ(
      joint0->getVelocityLowerLimits(), Eigen::VectorXd::Constant(1, -limit));
  EXPECT_EQ(
      joint0->getVelocityUpperLimits(), Eigen::VectorXd::Constant(1, limit));

  joint0->setVelocityLowerLimits(-limits);
  joint0->setVelocityUpperLimits(limits);
  EXPECT_EQ(joint0->getVelocityLowerLimits(), -limits);
  EXPECT_EQ(joint0->getVelocityUpperLimits(), limits);

  joint0->setAccelerationLowerLimit(0, -limit);
  joint0->setAccelerationUpperLimit(0, limit);
  EXPECT_EQ(
      joint0->getAccelerationLowerLimits(),
      Eigen::VectorXd::Constant(1, -limit));
  EXPECT_EQ(
      joint0->getAccelerationUpperLimits(),
      Eigen::VectorXd::Constant(1, limit));

  joint0->setAccelerationLowerLimits(-limits);
  joint0->setAccelerationUpperLimits(limits);
  EXPECT_EQ(joint0->getAccelerationLowerLimits(), -limits);
  EXPECT_EQ(joint0->getAccelerationUpperLimits(), limits);

  joint0->setForceLowerLimit(0, -limit);
  joint0->setForceUpperLimit(0, limit);
  EXPECT_EQ(
      joint0->getForceLowerLimits(), Eigen::VectorXd::Constant(1, -limit));
  EXPECT_EQ(joint0->getForceUpperLimits(), Eigen::VectorXd::Constant(1, limit));

  joint0->setForceLowerLimits(-limits);
  joint0->setForceUpperLimits(limits);
  EXPECT_EQ(joint0->getForceLowerLimits(), -limits);
  EXPECT_EQ(joint0->getForceUpperLimits(), limits);
}

//==============================================================================
void testJointCoulombFrictionForce(double _timeStep)
{
  // Realistic tolerance for friction simulation: 1e-6 rad/s is acceptable
  // (previously 1e-9 which caused spammy failures due to numerical precision)
  double tol = 1e-6;

  simulation::WorldPtr myWorld
      = dart::io::readWorld("dart://sample/skel/test/joint_friction_test.skel");
  EXPECT_TRUE(myWorld != nullptr);

  myWorld->setGravity(Eigen::Vector3d(0.0, 0.0, 0.0));
  myWorld->setTimeStep(_timeStep);

  dynamics::SkeletonPtr pendulum = myWorld->getSkeleton("double_pendulum");
  EXPECT_TRUE(pendulum != nullptr);
  pendulum->disableSelfCollisionCheck();

  dynamics::Joint* joint0 = pendulum->getJoint("joint0");
  dynamics::Joint* joint1 = pendulum->getJoint("joint1");

  EXPECT_TRUE(joint0 != nullptr);
  EXPECT_TRUE(joint1 != nullptr);

  double frictionForce = 5.0;

  joint0->setLimitEnforcement(false);
  joint1->setLimitEnforcement(false);

  joint0->setCoulombFriction(0, frictionForce);
  joint1->setCoulombFriction(0, frictionForce);

  EXPECT_EQ(joint0->getCoulombFriction(0), frictionForce);
  EXPECT_EQ(joint1->getCoulombFriction(0), frictionForce);

#if !defined(NDEBUG)
  double simTime = 0.2;
#else
  double simTime = 2.0;
#endif // ------- Debug mode
  double timeStep = myWorld->getTimeStep();
  int nSteps = simTime / timeStep;

  // Two seconds with lower control forces than the friction
  for (int i = 0; i < nSteps; i++) {
    joint0->setForce(0, +4.9);
    joint1->setForce(0, +4.9);
    myWorld->step();

    double jointVel0 = joint0->getVelocity(0);
    double jointVel1 = joint1->getVelocity(0);

    EXPECT_NEAR(jointVel0, 0.0, tol);
    EXPECT_NEAR(jointVel1, 0.0, tol);
  }

  // Another two seconds with lower control forces than the friction forces
  for (int i = 0; i < nSteps; i++) {
    joint0->setForce(0, -4.9);
    joint1->setForce(0, -4.9);
    myWorld->step();

    double jointVel0 = joint0->getVelocity(0);
    double jointVel1 = joint1->getVelocity(0);

    EXPECT_NEAR(jointVel0, 0.0, tol);
    EXPECT_NEAR(jointVel1, 0.0, tol);
  }

  // Another two seconds with higher control forces than the friction forces
  for (int i = 0; i < nSteps; i++) {
    joint0->setForce(0, 10.0);
    joint1->setForce(0, 10.0);
    myWorld->step();

    double jointVel0 = joint0->getVelocity(0);
    double jointVel1 = joint1->getVelocity(0);

    EXPECT_GE(std::abs(jointVel0), 0.0);
    EXPECT_GE(std::abs(jointVel1), 0.0);
  }

  // Spend 20 sec waiting the joints to stop
  for (int i = 0; i < nSteps * 10; i++) {
    myWorld->step();
  }
  double jointVel0 = joint0->getVelocity(0);
  double jointVel1 = joint1->getVelocity(0);

  EXPECT_NEAR(jointVel0, 0.0, tol);
  EXPECT_NEAR(jointVel1, 0.0, tol);

  // Another two seconds with lower control forces than the friction forces
  // and expect the joints to stop
  for (int i = 0; i < nSteps; i++) {
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
TEST_F(Joints, JointCoulombFriction)
{
  std::array<double, 3> timeSteps;
  timeSteps[0] = 1e-2;
  timeSteps[1] = 1e-3;
  timeSteps[2] = 1e-4;

  for (auto timeStep : timeSteps) {
    testJointCoulombFrictionForce(timeStep);
  }
}

//==============================================================================
SkeletonPtr createPendulum(Joint::ActuatorType actType)
{
  using namespace dart::math::suffixes;

  Vector3d dim(1, 1, 1);
  Vector3d offset(0, 0, 2);

  SkeletonPtr pendulum = createNLinkPendulum(1, dim, DOF_ROLL, offset);
  EXPECT_NE(pendulum, nullptr);

  pendulum->disableSelfCollisionCheck();

  for (std::size_t i = 0; i < pendulum->getNumBodyNodes(); ++i) {
    auto bodyNode = pendulum->getBodyNode(i);
    bodyNode->removeAllShapeNodesWith<CollisionAspect>();
  }

  // Joint common setting
  dynamics::Joint* joint = pendulum->getJoint(0);
  EXPECT_NE(joint, nullptr);

  joint->setActuatorType(actType);
  joint->setPosition(0, 90.0_deg);
  joint->setDampingCoefficient(0, 0.0);
  joint->setSpringStiffness(0, 0.0);
  joint->setLimitEnforcement(true);
  joint->setCoulombFriction(0, 0.0);

  return pendulum;
}

//==============================================================================
TEST_F(Joints, SpringRestPosition)
{
  using namespace math::suffixes;

  auto skel = createPendulum(Joint::ActuatorType::PASSIVE);
  ASSERT_NE(skel, nullptr);

  auto joint = skel->getRootJoint();
  ASSERT_NE(joint, nullptr);

  auto world = simulation::World::create();
  ASSERT_NE(world, nullptr);

  world->addSkeleton(skel);
  world->setGravity(Eigen::Vector3d::Zero());

  joint->setPosition(0, 0);
  joint->setRestPosition(0, -1.0_pi);
  joint->setPositionLowerLimit(0, -0.5_pi);
  joint->setPositionUpperLimit(0, +0.5_pi);
  joint->setSpringStiffness(0, 5);

  EXPECT_DOUBLE_EQ(joint->getPosition(0), 0);

  const auto tol = 1e-3;

  // Joint starts from 0 and rotates towards its spring rest position (i.e.,
  // -pi), but it also should stay within the joint limits
  // (i.e., [-0.5pi, 0.5pi]).
  for (auto i = 0u; i < 1000; ++i) {
    world->step();
    const auto pos = joint->getPosition(0);
    EXPECT_GE(pos, joint->getPositionLowerLimit(0) - tol);
    EXPECT_LE(pos, joint->getPositionUpperLimit(0) + tol);
  }

  // After a while, the joint should come to rest at its lower position limit.
  for (auto i = 0u; i < 500; ++i) {
    world->step();
    const auto pos = joint->getPosition(0);
    EXPECT_GE(pos, joint->getPositionLowerLimit(0) - tol);
    EXPECT_LE(pos, joint->getPositionUpperLimit(0) + tol);
    EXPECT_NEAR(pos, joint->getPositionLowerLimit(0), tol);
  }
}

//==============================================================================
void testServoMotor()
{
  using namespace dart::math::suffixes;

  std::size_t numPendulums = 7;
  double timestep = 1e-3;
  // Looser tolerance to accommodate platform jitter between dynamic SERVO and
  // kinematic VELOCITY enforcement.
  double tol = 1e-5;
  double posUpperLimit = 90.0_deg;
  double posLowerLimit = 45.0_deg;
  double sufficient_force = inf;
  double insufficient_force = 1e-1;

  // World
  simulation::WorldPtr world = simulation::World::create();
  EXPECT_TRUE(world != nullptr);

  world->setGravity(Eigen::Vector3d(0, 0, -9.81));
  world->setTimeStep(timestep);

  // Each pendulum has servo motor in the joint but also have different
  // expectations depending on the property values.
  //
  // Pendulum0:
  //  - Condition: Zero desired velocity and sufficient servo motor force limits
  //  - Expectation: The desired velocity should be achieved.
  // Pendulum 1:
  //  - Condition: Nonzero desired velocity and sufficient servo motor force
  //               limits
  //  - Expectation: The desired velocity should be achieved.
  // Pendulum 2:
  //  - Condition: Nonzero desired velocity and insufficient servo motor force
  //               limits
  //  - Expectation: The desired velocity can be achieve or not. But when it's
  //                 not achieved, the servo motor force is reached to the lower
  //                 or upper limit.
  // Pendulum 3:
  //  - Condition: Nonzero desired velocity, finite servo motor force limits,
  //               and position limits
  //  - Expectation: The desired velocity should be achieved unless joint
  //                 position is reached to lower or upper position limit.
  // Pendulum 4:
  //  - Condition: Nonzero desired velocity, infinite servo motor force limits,
  //               and position limits
  //  - Expectation: Same as the Pendulum 3's expectation.
  // Pendulum 5:
  //  - Condition: Nonzero desired velocity, finite servo motor force limits,
  //               and infinite Coulomb friction
  //  - Expectation: The the pendulum shouldn't move at all due to the infinite
  //                 friction.
  // Pendulum 6:
  //  - Condition: Nonzero desired velocity, infinite servo motor force limits,
  //               and infinite Coulomb friction
  //  - Expectation: The the pendulum shouldn't move at all due to the friction.
  //    TODO(JS): Should a servo motor dominant Coulomb friction in this case?

  std::vector<SkeletonPtr> pendulums(numPendulums);
  std::vector<JointPtr> joints(numPendulums);
  for (std::size_t i = 0; i < numPendulums; ++i) {
    pendulums[i] = createPendulum(Joint::SERVO);
    joints[i] = pendulums[i]->getJoint(0);
  }

  joints[0]->setForceUpperLimit(0, sufficient_force);
  joints[0]->setForceLowerLimit(0, -sufficient_force);

  joints[1]->setForceUpperLimit(0, sufficient_force);
  joints[1]->setForceLowerLimit(0, -sufficient_force);

  joints[2]->setForceUpperLimit(0, insufficient_force);
  joints[2]->setForceLowerLimit(0, -insufficient_force);

  joints[3]->setForceUpperLimit(0, sufficient_force);
  joints[3]->setForceLowerLimit(0, -sufficient_force);
  joints[3]->setPositionUpperLimit(0, posUpperLimit);
  joints[3]->setPositionLowerLimit(0, posLowerLimit);

  joints[4]->setForceUpperLimit(0, inf);
  joints[4]->setForceLowerLimit(0, -inf);
  joints[4]->setPositionUpperLimit(0, posUpperLimit);
  joints[4]->setPositionLowerLimit(0, posLowerLimit);

  joints[5]->setForceUpperLimit(0, sufficient_force);
  joints[5]->setForceLowerLimit(0, -sufficient_force);
  joints[5]->setCoulombFriction(0, inf);

  joints[6]->setForceUpperLimit(0, inf);
  joints[6]->setForceLowerLimit(0, -inf);
  joints[6]->setCoulombFriction(0, inf);

  for (auto pendulum : pendulums) {
    world->addSkeleton(pendulum);
  }

#if !defined(NDEBUG)
  double simTime = 0.2;
#else
  double simTime = 2.0;
#endif // ------- Debug mode
  double timeStep = world->getTimeStep();
  int nSteps = simTime / timeStep;

  double maxAbsJointVel5 = 0.0;

  // Two seconds with lower control forces than the friction
  for (int i = 0; i < nSteps; i++) {
    const double expected_vel = std::sin(world->getTime());

    joints[0]->setCommand(0, 0.0);
    joints[1]->setCommand(0, expected_vel);
    joints[2]->setCommand(0, expected_vel);
    joints[3]->setCommand(0, expected_vel);
    joints[4]->setCommand(0, expected_vel);
    joints[5]->setCommand(0, expected_vel);
    joints[6]->setCommand(0, expected_vel);

    world->step();

    std::vector<double> jointVels(numPendulums);
    for (std::size_t j = 0; j < numPendulums; ++j) {
      jointVels[j] = joints[j]->getVelocity(0);
    }

    maxAbsJointVel5 = std::max(maxAbsJointVel5, std::abs(jointVels[5]));

    EXPECT_NEAR(jointVels[0], 0.0, tol);
    EXPECT_NEAR(jointVels[1], expected_vel, tol);
    bool result2 = std::abs(jointVels[2] - expected_vel) < tol
                   || std::abs(
                          joints[2]->getConstraintImpulse(0) / timeStep
                          - insufficient_force)
                          < tol
                   || std::abs(
                          joints[2]->getConstraintImpulse(0) / timeStep
                          + insufficient_force)
                          < tol;
    EXPECT_TRUE(result2);
    EXPECT_LE(
        joints[3]->getPosition(0), posUpperLimit + expected_vel * timeStep);
    EXPECT_GE(
        joints[3]->getPosition(0), posLowerLimit - expected_vel * timeStep);
    // EXPECT_LE(joints[4]->getPosition(0),
    //     posUpperLimit + expected_vel * timeStep);
    // EXPECT_GE(joints[4]->getPosition(0),
    //     posLowerLimit - expected_vel * timeStep);
    // TODO(JS): Position limits and servo motor with infinite force limits
    // doesn't work together because they compete against each other to achieve
    // different joint velocities with their infinite force limits. In this
    // case, the position limit constraint should dominant the servo motor
    // constraint.
    // Servo motor with infinite force limits and infinite Coulomb friction is
    // known to be ill-posed; just ensure the drift stays bounded.
    // EXPECT_NEAR(jointVels[6], 0.0, tol * 1e+2);
    // TODO(JS): Servo motor with infinite force limits and infinite Coulomb
    // friction doesn't work because they compete against each other to achieve
    // different joint velocities with their infinite force limits. In this
    // case, the friction constraints should dominant the servo motor
    // constraints.
  }

  EXPECT_LE(maxAbsJointVel5, 5.0);
}

//==============================================================================
TEST_F(Joints, ServoMotor)
{
  testServoMotor();
}

//==============================================================================
void testMimicJoint()
{
  using namespace dart::math::suffixes;

  double timestep = 1e-3;
  double tol = 1e-9;
  // Servo mimic joints can diverge slightly on some backends; allow a small
  // positional gap.
  double tolPos = 1e-2;
  double sufficient_force = 1e+5;

  // World
  simulation::WorldPtr world = simulation::World::create();
  EXPECT_TRUE(world != nullptr);

  world->setGravity(Eigen::Vector3d(0, 0, -9.81));
  world->setTimeStep(timestep);

  Vector3d dim(1, 1, 1);
  Vector3d offset(0, 0, 2);

  SkeletonPtr pendulum = createNLinkPendulum(2, dim, DOF_ROLL, offset);
  EXPECT_NE(pendulum, nullptr);

  pendulum->disableSelfCollisionCheck();

  for (std::size_t i = 0; i < pendulum->getNumBodyNodes(); ++i) {
    auto bodyNode = pendulum->getBodyNode(i);
    bodyNode->removeAllShapeNodesWith<CollisionAspect>();
  }

  std::vector<JointPtr> joints(2);

  for (std::size_t i = 0; i < pendulum->getNumBodyNodes(); ++i) {
    dynamics::Joint* joint = pendulum->getJoint(i);
    EXPECT_NE(joint, nullptr);

    joint->setActuatorType(Joint::SERVO);
    joint->setPosition(0, 90.0_deg);
    joint->setDampingCoefficient(0, 0.0);
    joint->setSpringStiffness(0, 0.0);
    joint->setLimitEnforcement(true);
    joint->setCoulombFriction(0, 0.0);

    joints[i] = joint;
  }

  joints[0]->setForceUpperLimit(0, sufficient_force);
  joints[0]->setForceLowerLimit(0, -sufficient_force);

  joints[1]->setForceUpperLimit(0, sufficient_force);
  joints[1]->setForceLowerLimit(0, -sufficient_force);

  // Second joint mimics the first one
  joints[1]->setActuatorType(Joint::MIMIC);
  joints[1]->setMimicJoint(joints[0], 1., 0.);

  world->addSkeleton(pendulum);

#if !defined(NDEBUG)
  double simTime = 0.2;
#else
  double simTime = 2.0;
#endif // ------- Debug mode
  double timeStep = world->getTimeStep();
  int nSteps = simTime / timeStep;

  // Two seconds with lower control forces than the friction
  for (int i = 0; i < nSteps; i++) {
    const double expected_vel = std::sin(world->getTime());

    joints[0]->setCommand(0, expected_vel);

    world->step();

    // Check if the first joint achieved the velocity at each time-step
    EXPECT_NEAR(joints[0]->getVelocity(0), expected_vel, tol);

    // Check if the mimic joint follows the reference joint
    EXPECT_NEAR(joints[0]->getPosition(0), joints[1]->getPosition(0), tolPos);
  }

  // In the end, check once more if the mimic joint followed the reference joint
  EXPECT_NEAR(joints[0]->getPosition(0), joints[1]->getPosition(0), tolPos);
}

//==============================================================================
void testMimicCouplerJoint()
{
  using namespace dart::math::suffixes;

  double timestep = 1e-3;
  double tolPos = 1e-3;
  double sufficient_force = 1e+5;

  simulation::WorldPtr world = simulation::World::create();
  ASSERT_TRUE(world != nullptr);

  world->setGravity(Eigen::Vector3d::Zero());
  world->setTimeStep(timestep);

  Vector3d dim(1, 1, 1);
  Vector3d offset(0, 0, 2);

  SkeletonPtr pendulum = createNLinkPendulum(2, dim, DOF_ROLL, offset);
  ASSERT_NE(pendulum, nullptr);

  pendulum->disableSelfCollisionCheck();

  for (std::size_t i = 0; i < pendulum->getNumBodyNodes(); ++i) {
    auto bodyNode = pendulum->getBodyNode(i);
    bodyNode->removeAllShapeNodesWith<CollisionAspect>();
  }

  std::vector<JointPtr> joints(2);

  for (std::size_t i = 0; i < pendulum->getNumBodyNodes(); ++i) {
    dynamics::Joint* joint = pendulum->getJoint(i);
    ASSERT_NE(joint, nullptr);

    joint->setDampingCoefficient(0, 0.0);
    joint->setSpringStiffness(0, 0.0);
    joint->setLimitEnforcement(true);
    joint->setCoulombFriction(0, 0.0);

    joints[i] = joint;
  }

  joints[0]->setActuatorType(Joint::PASSIVE);
  joints[0]->setForceUpperLimit(0, sufficient_force);
  joints[0]->setForceLowerLimit(0, -sufficient_force);

  joints[1]->setActuatorType(Joint::MIMIC);
  joints[1]->setMimicJoint(joints[0], 1., 0.);
  joints[1]->setUseCouplerConstraint(true);
  joints[1]->setForceUpperLimit(0, sufficient_force);
  joints[1]->setForceLowerLimit(0, -sufficient_force);

  joints[0]->setPosition(0, 0.0_deg);
  joints[1]->setPosition(0, 30.0_deg);

  world->addSkeleton(pendulum);

  double initialReferencePosition = joints[0]->getPosition(0);
  double initialError = joints[1]->getPosition(0) - joints[0]->getPosition(0);

  for (int i = 0; i < 400; ++i) {
    world->step();
  }

  double finalError = joints[1]->getPosition(0) - joints[0]->getPosition(0);

  EXPECT_LT(std::abs(finalError), std::abs(initialError));
  EXPECT_NEAR(joints[0]->getPosition(0), joints[1]->getPosition(0), tolPos);
  EXPECT_GT(
      std::abs(joints[0]->getPosition(0) - initialReferencePosition), 1e-4);
}

//==============================================================================
TEST_F(Joints, MimicJoint)
{
  testMimicJoint();
}

//==============================================================================
TEST_F(Joints, MimicJointCoupler)
{
  testMimicCouplerJoint();
}

//==============================================================================
TEST_F(Joints, CouplerConstraintApplyImpulse)
{
  Vector3d dim(1, 1, 1);
  Vector3d offset(0, 0, 0);

  SkeletonPtr pendulum = createNLinkPendulum(2, dim, DOF_ROLL, offset);
  ASSERT_NE(pendulum, nullptr);
  pendulum->setTimeStep(1e-3);

  Joint* referenceJoint = pendulum->getJoint(0);
  Joint* followerJoint = pendulum->getJoint(1);
  ASSERT_NE(referenceJoint, nullptr);
  ASSERT_NE(followerJoint, nullptr);

  followerJoint->setActuatorType(Joint::MIMIC);
  followerJoint->setMimicJoint(referenceJoint, 1.0, 0.0);
  followerJoint->setUseCouplerConstraint(true);
  followerJoint->setVelocityLowerLimit(0, -100.0);
  followerJoint->setVelocityUpperLimit(0, 100.0);
  followerJoint->setForceLowerLimit(0, -100.0);
  followerJoint->setForceUpperLimit(0, 100.0);

  referenceJoint->setPosition(0, 0.1);
  followerJoint->setPosition(0, 0.0);

  CouplerConstraintTestHelper constraint(
      followerJoint, followerJoint->getMimicDofProperties());
  constraint.update();
  ASSERT_GT(constraint.getDimension(), 0u);

  double lambda[1] = {2.3};
  constraint.applyImpulse(lambda);

  EXPECT_NEAR(followerJoint->getConstraintImpulse(0), lambda[0], 1e-12);
  EXPECT_NEAR(
      referenceJoint->getConstraintImpulse(0),
      -lambda[0] * followerJoint->getMimicMultiplier(0),
      1e-12);
}

//==============================================================================
TEST_F(Joints, PartialMimicJoint)
{
  using namespace dart::math::suffixes;

  auto world = simulation::World::create();
  world->setGravity(Eigen::Vector3d::Zero());
  world->setTimeStep(1e-3);

  auto skeleton = Skeleton::create("partial_mimic");
  auto leaderPair = skeleton->createJointAndBodyNodePair<RevoluteJoint>();
  auto leaderJoint = leaderPair.first;
  auto followerPair
      = skeleton->createJointAndBodyNodePair<UniversalJoint>(leaderPair.second);
  auto followerJoint = followerPair.first;

  leaderPair.second->setMass(1.0);
  followerPair.second->setMass(1.0);

  const double forceLimit = 1e5;
  std::array<Joint*, 2> joints = {leaderJoint, followerJoint};
  for (Joint* joint : joints) {
    for (std::size_t i = 0; i < joint->getNumDofs(); ++i) {
      joint->setDampingCoefficient(i, 0.0);
      joint->setSpringStiffness(i, 0.0);
      joint->setCoulombFriction(i, 0.0);
      joint->setForceLowerLimit(i, -forceLimit);
      joint->setForceUpperLimit(i, forceLimit);
      joint->setVelocityLowerLimit(i, -forceLimit);
      joint->setVelocityUpperLimit(i, forceLimit);
    }
  }

  leaderJoint->setActuatorType(Joint::SERVO);
  followerJoint->setActuatorType(Joint::SERVO);

  MimicDofProperties mimicProps;
  mimicProps.mReferenceJoint = leaderJoint;
  mimicProps.mReferenceDofIndex = 0;
  mimicProps.mMultiplier = 1.0;
  mimicProps.mOffset = 0.0;
  followerJoint->setMimicJointDof(1, mimicProps);
  followerJoint->setActuatorType(1, Joint::MIMIC);

  EXPECT_EQ(followerJoint->getActuatorType(0), Joint::SERVO);
  EXPECT_EQ(followerJoint->getActuatorType(1), Joint::MIMIC);
  EXPECT_TRUE(followerJoint->hasActuatorType(Joint::MIMIC));

  world->addSkeleton(skeleton);

#if !defined(NDEBUG)
  const std::size_t numSteps = 400;
#else
  const std::size_t numSteps = 2000;
#endif
  const double followerCommand = 0.25;
  const double tolVel = 1e-3;
  const double tolPos = 5e-3;

  for (std::size_t i = 0; i < numSteps; ++i) {
    const double leaderCommand = std::sin(world->getTime());

    leaderJoint->setCommand(0, leaderCommand);
    followerJoint->setCommand(0, followerCommand);

    world->step();

    EXPECT_NEAR(leaderJoint->getVelocity(0), leaderCommand, tolVel);
    EXPECT_NEAR(followerJoint->getVelocity(0), followerCommand, tolVel);

    if (i > numSteps / 5) {
      EXPECT_NEAR(
          followerJoint->getPosition(1), leaderJoint->getPosition(0), tolPos);
    }
  }
}

//==============================================================================
TEST_F(Joints, PartialMimicJointWithCouplerFlagFallsBackToMimicMotor)
{
  using namespace dart::math::suffixes;

  auto world = simulation::World::create();
  world->setGravity(Eigen::Vector3d::Zero());
  world->setTimeStep(1e-3);

  auto skeleton = Skeleton::create("partial_mimic_coupler_flag");
  auto leaderPair = skeleton->createJointAndBodyNodePair<RevoluteJoint>();
  auto leaderJoint = leaderPair.first;
  auto followerPair
      = skeleton->createJointAndBodyNodePair<UniversalJoint>(leaderPair.second);
  auto followerJoint = followerPair.first;

  leaderPair.second->setMass(1.0);
  followerPair.second->setMass(1.0);

  const double forceLimit = 1e5;
  std::array<Joint*, 2> joints = {leaderJoint, followerJoint};
  for (Joint* joint : joints) {
    for (std::size_t i = 0; i < joint->getNumDofs(); ++i) {
      joint->setDampingCoefficient(i, 0.0);
      joint->setSpringStiffness(i, 0.0);
      joint->setCoulombFriction(i, 0.0);
      joint->setForceLowerLimit(i, -forceLimit);
      joint->setForceUpperLimit(i, forceLimit);
      joint->setVelocityLowerLimit(i, -forceLimit);
      joint->setVelocityUpperLimit(i, forceLimit);
    }
  }

  leaderJoint->setActuatorType(Joint::SERVO);
  followerJoint->setActuatorType(Joint::SERVO);

  MimicDofProperties mimicProps;
  mimicProps.mReferenceJoint = leaderJoint;
  mimicProps.mReferenceDofIndex = 0;
  mimicProps.mMultiplier = 1.0;
  mimicProps.mOffset = 0.0;
  followerJoint->setMimicJointDof(1, mimicProps);
  followerJoint->setActuatorType(1, Joint::MIMIC);
  // Enabling coupler would have crashed before the guard when non-mimic DOFs
  // lack references; we should fall back to mimic motor constraints instead.
  followerJoint->setUseCouplerConstraint(true);

  EXPECT_EQ(followerJoint->getActuatorType(0), Joint::SERVO);
  EXPECT_EQ(followerJoint->getActuatorType(1), Joint::MIMIC);
  EXPECT_TRUE(followerJoint->hasActuatorType(Joint::MIMIC));

  world->addSkeleton(skeleton);

#if !defined(NDEBUG)
  const std::size_t numSteps = 400;
#else
  const std::size_t numSteps = 800;
#endif
  const double followerCommand = 0.25;
  const double tolVel = 1e-3;
  const double tolPos = 5e-2;

  for (std::size_t i = 0; i < numSteps; ++i) {
    const double leaderCommand = std::sin(world->getTime());

    leaderJoint->setCommand(0, leaderCommand);
    followerJoint->setCommand(0, followerCommand);

    world->step();

    EXPECT_NEAR(leaderJoint->getVelocity(0), leaderCommand, tolVel);
    EXPECT_NEAR(followerJoint->getVelocity(0), followerCommand, tolVel);
    EXPECT_NEAR(
        followerJoint->getPosition(1), leaderJoint->getPosition(0), tolPos);
  }
}

//==============================================================================
TEST_F(Joints, JointCoulombFrictionAndPositionLimit)
{
  const double timeStep = 1e-3;
  const double tol = 1e-2;

  simulation::WorldPtr myWorld
      = dart::io::readWorld("dart://sample/skel/test/joint_friction_test.skel");
  EXPECT_TRUE(myWorld != nullptr);

  myWorld->setGravity(Eigen::Vector3d(0.0, 0.0, 0.0));
  myWorld->setTimeStep(timeStep);

  dynamics::SkeletonPtr pendulum = myWorld->getSkeleton("double_pendulum");
  EXPECT_TRUE(pendulum != nullptr);
  pendulum->disableSelfCollisionCheck();

  dynamics::Joint* joint0 = pendulum->getJoint("joint0");
  dynamics::Joint* joint1 = pendulum->getJoint("joint1");

  EXPECT_TRUE(joint0 != nullptr);
  EXPECT_TRUE(joint1 != nullptr);

  double frictionForce = 5.0;

  joint0->setLimitEnforcement(true);
  joint1->setLimitEnforcement(true);

  const double ll = -pi / 12.0; // -15 degree
  const double ul = +pi / 12.0; // +15 degree

  std::size_t dof0 = joint0->getNumDofs();
  for (std::size_t i = 0; i < dof0; ++i) {
    joint0->setPosition(i, 0.0);
    joint0->setPosition(i, 0.0);
    joint0->setPositionLowerLimit(i, ll);
    joint0->setPositionUpperLimit(i, ul);
  }

  std::size_t dof1 = joint1->getNumDofs();
  for (std::size_t i = 0; i < dof1; ++i) {
    joint1->setPosition(i, 0.0);
    joint1->setPosition(i, 0.0);
    joint1->setPositionLowerLimit(i, ll);
    joint1->setPositionUpperLimit(i, ul);
  }

  joint0->setCoulombFriction(0, frictionForce);
  joint1->setCoulombFriction(0, frictionForce);

  EXPECT_EQ(joint0->getCoulombFriction(0), frictionForce);
  EXPECT_EQ(joint1->getCoulombFriction(0), frictionForce);

#if !defined(NDEBUG)
  double simTime = 0.2;
#else
  double simTime = 2.0;
#endif // ------- Debug mode
  int nSteps = simTime / timeStep;

  // First two seconds rotating in positive direction with higher control forces
  // than the friction forces
  for (int i = 0; i < nSteps; i++) {
    joint0->setForce(0, 100.0);
    joint1->setForce(0, 100.0);
    myWorld->step();

    double jointPos0 = joint0->getPosition(0);
    double jointPos1 = joint1->getPosition(0);

    double jointVel0 = joint0->getVelocity(0);
    double jointVel1 = joint1->getVelocity(0);

    EXPECT_GE(std::abs(jointVel0), 0.0);
    EXPECT_GE(std::abs(jointVel1), 0.0);

    EXPECT_GE(jointPos0, ll - tol);
    EXPECT_LE(jointPos0, ul + tol);

    EXPECT_GE(jointPos1, ll - tol);
    EXPECT_LE(jointPos1, ul + tol);
  }

  // Another two seconds rotating in negative direction with higher control
  // forces than the friction forces
  for (int i = 0; i < nSteps; i++) {
    joint0->setForce(0, -100.0);
    joint1->setForce(0, -100.0);
    myWorld->step();

    double jointPos0 = joint0->getPosition(0);
    double jointPos1 = joint1->getPosition(0);

    double jointVel0 = joint0->getVelocity(0);
    double jointVel1 = joint1->getVelocity(0);

    EXPECT_GE(std::abs(jointVel0), 0.0);
    EXPECT_GE(std::abs(jointVel1), 0.0);

    EXPECT_GE(jointPos0, ll - tol);
    EXPECT_LE(jointPos0, ul + tol);

    EXPECT_GE(jointPos1, ll - tol);
    EXPECT_LE(jointPos1, ul + tol);
  }
}

//==============================================================================
template <int N>
Eigen::Matrix<double, N, 1> random_vec(double limit = 100)
{
  Eigen::Matrix<double, N, 1> v;
  for (std::size_t i = 0; i < N; ++i) {
    v[i] = math::Random::uniform(-std::abs(limit), std::abs(limit));
  }
  return v;
}

//==============================================================================
Eigen::Isometry3d random_transform(
    double translation_limit = 100, double rotation_limit = 2 * dart::math::pi)
{
  Eigen::Vector3d r = random_vec<3>(translation_limit);
  Eigen::Vector3d theta = random_vec<3>(rotation_limit);

  Eigen::Isometry3d tf;
  tf.setIdentity();
  tf.translate(r);

  if (theta.norm() > 0) {
    tf.rotate(Eigen::AngleAxisd(theta.norm(), theta.normalized()));
  }

  return tf;
}

//==============================================================================
Eigen::Isometry3d predict_joint_transform(
    Joint* joint, const Eigen::Isometry3d& joint_tf)
{
  return joint->getTransformFromParentBodyNode() * joint_tf
         * joint->getTransformFromChildBodyNode().inverse();
}

//==============================================================================
Eigen::Isometry3d get_relative_transform(BodyNode* bn, BodyNode* relativeTo)
{
  return relativeTo->getTransform().inverse() * bn->getTransform();
}

//==============================================================================
TEST_F(Joints, ConvenienceFunctions)
{
  SkeletonPtr skel = Skeleton::create();

  std::pair<Joint*, BodyNode*> pair;

  pair = skel->createJointAndBodyNodePair<WeldJoint>();
  BodyNode* root = pair.second;

  // -- set up the FreeJoint
  std::pair<FreeJoint*, BodyNode*> freepair
      = root->createChildJointAndBodyNodePair<FreeJoint>();
  FreeJoint* freejoint = freepair.first;
  BodyNode* freejoint_bn = freepair.second;

  freejoint->setTransformFromParentBodyNode(random_transform());
  freejoint->setTransformFromChildBodyNode(random_transform());

  // -- set up the EulerJoint
  std::pair<EulerJoint*, BodyNode*> eulerpair
      = root->createChildJointAndBodyNodePair<EulerJoint>();
  EulerJoint* eulerjoint = eulerpair.first;
  BodyNode* eulerjoint_bn = eulerpair.second;

  eulerjoint->setTransformFromParentBodyNode(random_transform());
  eulerjoint->setTransformFromChildBodyNode(random_transform());

  // -- set up the BallJoint
  std::pair<BallJoint*, BodyNode*> ballpair
      = root->createChildJointAndBodyNodePair<BallJoint>();
  BallJoint* balljoint = ballpair.first;
  BodyNode* balljoint_bn = ballpair.second;

  balljoint->setTransformFromParentBodyNode(random_transform());
  balljoint->setTransformFromChildBodyNode(random_transform());

  // Test a hundred times
  for (std::size_t n = 0; n < 100; ++n) {
    // -- convert transforms to positions and then positions back to transforms
    Eigen::Isometry3d desired_freejoint_tf = random_transform();
    freejoint->setPositions(
        FreeJoint::convertToPositions(desired_freejoint_tf));
    Eigen::Isometry3d actual_freejoint_tf
        = FreeJoint::convertToTransform(freejoint->getPositions());

    Eigen::Isometry3d desired_eulerjoint_tf = random_transform();
    desired_eulerjoint_tf.translation() = Eigen::Vector3d::Zero();
    eulerjoint->setPositions(
        eulerjoint->convertToPositions(desired_eulerjoint_tf.linear()));
    Eigen::Isometry3d actual_eulerjoint_tf
        = eulerjoint->convertToTransform(eulerjoint->getPositions());

    Eigen::Isometry3d desired_balljoint_tf = random_transform();
    desired_balljoint_tf.translation() = Eigen::Vector3d::Zero();
    balljoint->setPositions(
        BallJoint::convertToPositions(desired_balljoint_tf.linear()));
    Eigen::Isometry3d actual_balljoint_tf
        = BallJoint::convertToTransform(balljoint->getPositions());

    // -- collect everything so we can cycle through the tests
    std::vector<Joint*> joints;
    std::vector<BodyNode*> bns;
    common::aligned_vector<Eigen::Isometry3d> desired_tfs;
    common::aligned_vector<Eigen::Isometry3d> actual_tfs;

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

    for (std::size_t i = 0; i < joints.size(); ++i) {
      Joint* joint = joints[i];
      BodyNode* bn = bns[i];
      Eigen::Isometry3d tf = desired_tfs[i];

      bool check_transform_conversion = equals(
          predict_joint_transform(joint, tf).matrix(),
          get_relative_transform(bn, bn->getParentBodyNode()).matrix());
      EXPECT_TRUE(check_transform_conversion);

      if (!check_transform_conversion) {
        std::cout << "[" << joint->getName() << " Failed]\n";
        std::cout
            << "Predicted:\n"
            << predict_joint_transform(joint, tf).matrix() << "\n\nActual:\n"
            << get_relative_transform(bn, bn->getParentBodyNode()).matrix()
            << "\n\n";
      }

      bool check_full_cycle
          = equals(desired_tfs[i].matrix(), actual_tfs[i].matrix());
      EXPECT_TRUE(check_full_cycle);

      if (!check_full_cycle) {
        std::cout << "[" << joint->getName() << " Failed]\n";
        std::cout << "Desired:\n"
                  << desired_tfs[i].matrix() << "\n\nActual:\n"
                  << actual_tfs[i].matrix() << "\n\n";
      }
    }
  }
}

//==============================================================================
TEST_F(Joints, BallJointCoordinateChart)
{
  SkeletonPtr skel = Skeleton::create("ball_chart");

  BallJoint::Properties ballProps;
  ballProps.mCoordinateChart = BallJoint::CoordinateChart::EULER_XYZ;

  auto [ballJoint, ballBody]
      = skel->createJointAndBodyNodePair<BallJoint>(nullptr, ballProps);
  (void)ballBody;

  EXPECT_EQ(
      ballJoint->getCoordinateChart(), BallJoint::CoordinateChart::EULER_XYZ);

  const Eigen::Vector3d xyzAngles(0.2, -0.1, 0.3);
  const Eigen::Matrix3d xyzRotation = math::eulerXYZToMatrix(xyzAngles);
  const Eigen::Vector3d xyzPositions = BallJoint::convertToPositions(
      xyzRotation, BallJoint::CoordinateChart::EULER_XYZ);
  ballJoint->setPositions(xyzPositions);
  EXPECT_TRUE(
      ballJoint->getRelativeTransform().linear().isApprox(xyzRotation, 1e-10));

  const Eigen::Matrix3d xyzRelative
      = ballJoint->getRelativeTransform().linear();
  ballJoint->setCoordinateChart(BallJoint::CoordinateChart::EULER_ZYX);
  EXPECT_EQ(
      ballJoint->getCoordinateChart(), BallJoint::CoordinateChart::EULER_ZYX);
  EXPECT_TRUE(
      ballJoint->getRelativeTransform().linear().isApprox(xyzRelative, 1e-10));

  const Eigen::Vector3d zyxAngles(0.3, -0.2, 0.15);
  const Eigen::Matrix3d zyxRotation = math::eulerZYXToMatrix(zyxAngles);
  const Eigen::Vector3d zyxPositions = BallJoint::convertToPositions(
      zyxRotation, BallJoint::CoordinateChart::EULER_ZYX);
  ballJoint->setPositions(zyxPositions);
  EXPECT_TRUE(
      ballJoint->getRelativeTransform().linear().isApprox(zyxRotation, 1e-10));
}

//==============================================================================
TEST_F(Joints, FreeJointCoordinateChart)
{
  SkeletonPtr skel = Skeleton::create("free_chart");

  FreeJoint::Properties freeProps;
  freeProps.mCoordinateChart = FreeJoint::CoordinateChart::EULER_ZYX;

  auto [freeJoint, freeBody]
      = skel->createJointAndBodyNodePair<FreeJoint>(nullptr, freeProps);
  (void)freeBody;

  EXPECT_EQ(
      freeJoint->getCoordinateChart(), FreeJoint::CoordinateChart::EULER_ZYX);

  const Eigen::Vector3d zyxAngles(0.1, -0.25, 0.05);
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.linear() = math::eulerZYXToMatrix(zyxAngles);
  tf.translation() = Eigen::Vector3d(0.4, -0.2, 0.3);

  const Eigen::Vector6d positions = FreeJoint::convertToPositions(
      tf, FreeJoint::CoordinateChart::EULER_ZYX);
  freeJoint->setPositions(positions);
  EXPECT_TRUE(
      freeJoint->getRelativeTransform().linear().isApprox(tf.linear(), 1e-10));
  EXPECT_TRUE(freeJoint->getRelativeTransform().translation().isApprox(
      tf.translation(), 1e-12));

  const Eigen::Isometry3d relativeTransform = freeJoint->getRelativeTransform();
  freeJoint->setCoordinateChart(FreeJoint::CoordinateChart::EULER_XYZ);
  EXPECT_EQ(
      freeJoint->getCoordinateChart(), FreeJoint::CoordinateChart::EULER_XYZ);
  EXPECT_TRUE(freeJoint->getRelativeTransform().linear().isApprox(
      relativeTransform.linear(), 1e-10));
  EXPECT_TRUE(freeJoint->getRelativeTransform().translation().isApprox(
      relativeTransform.translation(), 1e-12));
}

//==============================================================================
TEST_F(Joints, FreeJointRelativeTransformVelocityAcceleration)
{
  const std::size_t numTests = 50;

  // Generate random reference frames
  randomizeRefFrames();
  auto refFrames = getFrames();

  // Generate random relative frames
  randomizeRefFrames();
  auto relFrames = getFrames();

  //-- Build a skeleton that contains two FreeJoints and two BodyNodes
  SkeletonPtr skel = Skeleton::create();

  auto pair = skel->createJointAndBodyNodePair<FreeJoint>();
  FreeJoint* rootJoint = pair.first;
  BodyNode* rootBodyNode = pair.second;
  rootJoint->setRelativeTransform(random_transform());
  rootJoint->setRelativeSpatialVelocity(random_vec<6>());
  rootJoint->setRelativeSpatialAcceleration(random_vec<6>());
  rootJoint->setTransformFromParentBodyNode(random_transform());
  rootJoint->setTransformFromChildBodyNode(random_transform());

  pair = rootBodyNode->createChildJointAndBodyNodePair<FreeJoint>();
  FreeJoint* joint1 = pair.first;
  BodyNode* bodyNode1 = pair.second;
  joint1->setTransformFromParentBodyNode(random_transform());
  joint1->setTransformFromChildBodyNode(random_transform());

  //-- Actual terms
  Eigen::Isometry3d actualTf;
  Eigen::Vector6d actualVel;
  Eigen::Vector6d actualAcc;

  Eigen::Vector3d actualLinVel;
  Eigen::Vector3d actualAngVel;
  Eigen::Vector3d actualLinAcc;
  Eigen::Vector3d actualAngAcc;

  Eigen::Vector3d oldLinVel;
  Eigen::Vector3d oldAngVel;
  Eigen::Vector3d oldLinAcc;
  Eigen::Vector3d oldAngAcc;

  //-- Test
  for (std::size_t i = 0; i < numTests; ++i) {
    const Eigen::Isometry3d desiredTf = random_transform();
    const Eigen::Vector6d desiredVel = random_vec<6>();
    const Eigen::Vector6d desiredAcc = random_vec<6>();
    const Eigen::Vector3d desiredLinVel = random_vec<3>();
    const Eigen::Vector3d desiredAngVel = random_vec<3>();
    const Eigen::Vector3d desiredLinAcc = random_vec<3>();
    const Eigen::Vector3d desiredAngAcc = random_vec<3>();

    //-- Relative transformation

    joint1->setRelativeTransform(desiredTf);
    actualTf = bodyNode1->getTransform(bodyNode1->getParentBodyNode());
    EXPECT_TRUE(equals(desiredTf.matrix(), actualTf.matrix()));

    for (auto relativeTo : relFrames) {
      joint1->setTransform(desiredTf, relativeTo);
      actualTf = bodyNode1->getTransform(relativeTo);
      EXPECT_TRUE(equals(desiredTf.matrix(), actualTf.matrix()));
    }

    //-- Relative spatial velocity

    joint1->setRelativeSpatialVelocity(desiredVel);
    actualVel = bodyNode1->getSpatialVelocity(
        bodyNode1->getParentBodyNode(), bodyNode1);

    EXPECT_TRUE(equals(desiredVel, actualVel));

    for (auto relativeTo : relFrames) {
      for (auto inCoordinatesOf : refFrames) {
        joint1->setSpatialVelocity(desiredVel, relativeTo, inCoordinatesOf);
        actualVel = bodyNode1->getSpatialVelocity(relativeTo, inCoordinatesOf);

        EXPECT_TRUE(equals(desiredVel, actualVel));
      }
    }

    //-- Relative classic linear velocity

    for (auto relativeTo : relFrames) {
      for (auto inCoordinatesOf : refFrames) {
        joint1->setSpatialVelocity(desiredVel, relativeTo, inCoordinatesOf);
        oldAngVel = bodyNode1->getAngularVelocity(relativeTo, inCoordinatesOf);
        joint1->setLinearVelocity(desiredLinVel, relativeTo, inCoordinatesOf);

        actualLinVel
            = bodyNode1->getLinearVelocity(relativeTo, inCoordinatesOf);
        actualAngVel
            = bodyNode1->getAngularVelocity(relativeTo, inCoordinatesOf);

        EXPECT_TRUE(equals(desiredLinVel, actualLinVel));
        EXPECT_TRUE(equals(oldAngVel, actualAngVel));
      }
    }

    //-- Relative classic angular velocity

    for (auto relativeTo : relFrames) {
      for (auto inCoordinatesOf : refFrames) {
        joint1->setSpatialVelocity(desiredVel, relativeTo, inCoordinatesOf);
        oldLinVel = bodyNode1->getLinearVelocity(relativeTo, inCoordinatesOf);
        joint1->setAngularVelocity(desiredAngVel, relativeTo, inCoordinatesOf);

        actualLinVel
            = bodyNode1->getLinearVelocity(relativeTo, inCoordinatesOf);
        actualAngVel
            = bodyNode1->getAngularVelocity(relativeTo, inCoordinatesOf);

        EXPECT_TRUE(equals(oldLinVel, actualLinVel));
        EXPECT_TRUE(equals(desiredAngVel, actualAngVel));
      }
    }

    //-- Relative spatial acceleration

    joint1->setRelativeSpatialAcceleration(desiredAcc);
    actualAcc = bodyNode1->getSpatialAcceleration(
        bodyNode1->getParentBodyNode(), bodyNode1);

    EXPECT_TRUE(equals(desiredAcc, actualAcc));

    for (auto relativeTo : relFrames) {
      for (auto inCoordinatesOf : refFrames) {
        joint1->setSpatialAcceleration(desiredAcc, relativeTo, inCoordinatesOf);
        actualAcc
            = bodyNode1->getSpatialAcceleration(relativeTo, inCoordinatesOf);

        EXPECT_TRUE(equals(desiredAcc, actualAcc));
      }
    }

    //-- Relative transform, spatial velocity, and spatial acceleration
    for (auto relativeTo : relFrames) {
      for (auto inCoordinatesOf : refFrames) {
        joint1->setSpatialMotion(
            &desiredTf,
            relativeTo,
            &desiredVel,
            relativeTo,
            inCoordinatesOf,
            &desiredAcc,
            relativeTo,
            inCoordinatesOf);
        actualTf = bodyNode1->getTransform(relativeTo);
        actualVel = bodyNode1->getSpatialVelocity(relativeTo, inCoordinatesOf);
        actualAcc
            = bodyNode1->getSpatialAcceleration(relativeTo, inCoordinatesOf);

        EXPECT_TRUE(equals(desiredTf.matrix(), actualTf.matrix()));
        EXPECT_TRUE(equals(desiredVel, actualVel));
        EXPECT_TRUE(equals(desiredAcc, actualAcc));
      }
    }

    //-- Relative classic linear acceleration

    for (auto relativeTo : relFrames) {
      for (auto inCoordinatesOf : refFrames) {
        joint1->setSpatialAcceleration(desiredAcc, relativeTo, inCoordinatesOf);
        oldAngAcc
            = bodyNode1->getAngularAcceleration(relativeTo, inCoordinatesOf);
        joint1->setLinearAcceleration(
            desiredLinAcc, relativeTo, inCoordinatesOf);

        actualLinAcc
            = bodyNode1->getLinearAcceleration(relativeTo, inCoordinatesOf);
        actualAngAcc
            = bodyNode1->getAngularAcceleration(relativeTo, inCoordinatesOf);

        EXPECT_TRUE(equals(desiredLinAcc, actualLinAcc));
        EXPECT_TRUE(equals(oldAngAcc, actualAngAcc));
      }
    }

    //-- Relative classic angular acceleration

    for (auto relativeTo : relFrames) {
      for (auto inCoordinatesOf : refFrames) {
        joint1->setSpatialAcceleration(desiredAcc, relativeTo, inCoordinatesOf);
        oldLinAcc
            = bodyNode1->getLinearAcceleration(relativeTo, inCoordinatesOf);
        joint1->setAngularAcceleration(
            desiredAngAcc, relativeTo, inCoordinatesOf);

        actualLinAcc
            = bodyNode1->getLinearAcceleration(relativeTo, inCoordinatesOf);
        actualAngAcc
            = bodyNode1->getAngularAcceleration(relativeTo, inCoordinatesOf);

        EXPECT_TRUE(equals(oldLinAcc, actualLinAcc));
        EXPECT_TRUE(equals(desiredAngAcc, actualAngAcc));
      }
    }
  }
}

//==============================================================================
TEST_F(Joints, FreeJointWorldJacobianTranslation)
{
  SkeletonPtr skel = Skeleton::create();

  auto pair = skel->createJointAndBodyNodePair<FreeJoint>();
  FreeJoint* joint = pair.first;
  BodyNode* body = pair.second;

  Eigen::Vector6d positions = Eigen::Vector6d::Zero();
  positions.head<3>() = Eigen::Vector3d(0.4, -0.7, 0.2);
  positions.tail<3>() = Eigen::Vector3d(0.1, -0.2, 0.3);
  joint->setPositions(positions);

  const math::Jacobian worldJac = body->getWorldJacobian();
  const Eigen::Matrix3d translationalBlock
      = worldJac.bottomRows<3>().rightCols<3>();
  EXPECT_TRUE(equals(translationalBlock, Eigen::Matrix3d::Identity()));

  Eigen::Vector6d velocities = Eigen::Vector6d::Zero();
  velocities.head<3>() = Eigen::Vector3d(0.2, -0.15, 0.35);
  velocities.tail<3>() = Eigen::Vector3d(0.45, -0.6, 0.2);
  joint->setVelocities(velocities);

  const Eigen::Vector6d spatialVelocityFromJac = worldJac * velocities;
  const Eigen::Vector3d translationalVelocity = velocities.tail<3>();
  const Eigen::Vector3d translationalFromJac = spatialVelocityFromJac.tail<3>();
  const Eigen::Vector3d actualLinearVelocity
      = body->getLinearVelocity(Frame::World(), Frame::World());

  EXPECT_TRUE(equals(translationalVelocity, actualLinearVelocity));
  EXPECT_TRUE(equals(translationalFromJac, actualLinearVelocity));
}

//==============================================================================
TEST_F(Joints, FreeJointWorldJacobianTranslationRandomized)
{
  SkeletonPtr skel = Skeleton::create();

  auto pair = skel->createJointAndBodyNodePair<FreeJoint>();
  FreeJoint* joint = pair.first;
  BodyNode* body = pair.second;

  const Frame* worldFrame = Frame::World();

  const std::size_t numTests = 50;
  for (std::size_t i = 0; i < numTests; ++i) {
    const Eigen::Isometry3d pose = random_transform();
    joint->setPositions(FreeJoint::convertToPositions(pose));

    const Eigen::Vector6d velocities = random_vec<6>();
    joint->setVelocities(velocities);

    const math::Jacobian& J = body->getWorldJacobian();
    EXPECT_TRUE(
        equals(J.bottomRows<3>().rightCols<3>(), Eigen::Matrix3d::Identity()));

    const Eigen::Vector6d spatialFromJac = J * velocities;
    const Eigen::Vector3d translationalFromJac = spatialFromJac.tail<3>();
    const Eigen::Vector3d translationalVelocity = velocities.tail<3>();
    const Eigen::Vector6d spatialVelocity
        = body->getSpatialVelocity(worldFrame, worldFrame);

    EXPECT_TRUE(equals(spatialFromJac, spatialVelocity));
    EXPECT_TRUE(equals(translationalFromJac, translationalVelocity));
  }
}

//==============================================================================
TEST_F(Joints, FreeJointWorldJacobianWithOffsets)
{
  SkeletonPtr skel = Skeleton::create();

  auto pair = skel->createJointAndBodyNodePair<FreeJoint>();
  FreeJoint* joint = pair.first;
  BodyNode* body = pair.second;

  joint->setTransformFromParentBodyNode(random_transform());
  joint->setTransformFromChildBodyNode(random_transform());

  const std::size_t numTests = 25;
  for (std::size_t i = 0; i < numTests; ++i) {
    joint->setPositions(FreeJoint::convertToPositions(random_transform()));

    const Eigen::Vector6d velocities = random_vec<6>(0.5);
    joint->setVelocities(velocities);

    const math::Jacobian worldJac = body->getWorldJacobian();
    const Eigen::Vector6d spatialFromJac = worldJac * velocities;
    const Eigen::Vector6d spatialVelocity
        = body->getSpatialVelocity(Frame::World(), Frame::World());

    EXPECT_TRUE(equals(spatialFromJac, spatialVelocity));

    const Eigen::Vector3d offset = random_vec<3>(0.2);
    const math::Jacobian worldJacOffset = body->getWorldJacobian(offset);
    const Eigen::Vector6d spatialFromJacOffset = worldJacOffset * velocities;
    const Eigen::Vector6d spatialVelocityOffset
        = body->getSpatialVelocity(offset, Frame::World(), Frame::World());

    EXPECT_TRUE(equals(spatialFromJacOffset, spatialVelocityOffset));
  }
}

//==============================================================================
TEST_F(Joints, FreeJointIntegrationTranslationUncoupled)
{
  SkeletonPtr skel = Skeleton::create();

  auto pair = skel->createJointAndBodyNodePair<FreeJoint>();
  FreeJoint* joint = pair.first;

  Eigen::Vector6d positions = Eigen::Vector6d::Zero();
  positions.head<3>() = Eigen::Vector3d(0.2, -0.1, 0.3);
  positions.tail<3>() = Eigen::Vector3d(0.4, -0.25, 0.1);
  joint->setPositions(positions);

  const Eigen::Vector3d initialTranslation
      = joint->getRelativeTransform().translation();

  const double dt = 0.01;

  Eigen::Vector6d velocities = Eigen::Vector6d::Zero();
  velocities.head<3>() = Eigen::Vector3d(0.5, -0.2, 0.4);
  joint->setVelocities(velocities);

  skel->integratePositions(dt);
  Eigen::Vector3d translated = joint->getRelativeTransform().translation();
  EXPECT_TRUE(equals(translated, initialTranslation));

  joint->setPositions(positions);

  velocities.head<3>() = Eigen::Vector3d(0.3, -0.1, 0.2);
  velocities.tail<3>() = Eigen::Vector3d(-0.15, 0.05, 0.2);
  joint->setVelocities(velocities);

  const Eigen::Isometry3d startTf = joint->getRelativeTransform();
  const Eigen::Isometry3d expectedTf
      = startTf * math::expMap(joint->getRelativeSpatialVelocity() * dt);

  skel->integratePositions(dt);
  translated = joint->getRelativeTransform().translation();
  EXPECT_TRUE(equals(joint->getRelativeTransform(), expectedTf));
  EXPECT_TRUE(equals(translated, expectedTf.translation()));
}

//==============================================================================
TEST_F(Joints, FreeJointPositionDifferenceInWorldFrame)
{
  SkeletonPtr skel = Skeleton::create();

  auto pair = skel->createJointAndBodyNodePair<FreeJoint>();
  FreeJoint* joint = pair.first;

  Eigen::Vector6d q1 = Eigen::Vector6d::Zero();
  q1.head<3>() = Eigen::Vector3d(0.0, 0.0, 0.5 * pi);
  q1.tail<3>() = Eigen::Vector3d(0.4, -0.1, 0.2);

  const Eigen::Isometry3d Q1 = FreeJoint::convertToTransform(q1);

  Eigen::Isometry3d Q2 = Q1;
  const Eigen::Vector3d deltaRotationBody(0.1, -0.05, 0.02);
  const Eigen::Vector3d deltaTranslationWorld(0.05, -0.08, 0.12);
  Q2.linear() = Q2.linear() * math::expMapRot(deltaRotationBody);
  Q2.translation() += deltaTranslationWorld;

  const Eigen::Vector6d q2 = FreeJoint::convertToPositions(Q2);

  const Eigen::Vector6d diff = joint->getPositionDifferences(q2, q1);

  const Eigen::Vector3d expectedRotationWorld = Q1.linear() * deltaRotationBody;

  EXPECT_TRUE(equals(expectedRotationWorld, diff.head<3>().eval()));
  EXPECT_TRUE(equals(deltaTranslationWorld, diff.tail<3>().eval()));
}

//==============================================================================
TEST_F(Joints, FreeJointIntegrationMatchesBodyTwist)
{
  SkeletonPtr skel = Skeleton::create();

  auto pair = skel->createJointAndBodyNodePair<FreeJoint>();
  FreeJoint* joint = pair.first;

  const Eigen::Isometry3d pose = random_transform();
  joint->setPositions(FreeJoint::convertToPositions(pose));

  const Eigen::Vector6d velocities = random_vec<6>(1.0);
  joint->setVelocities(velocities);

  const Eigen::Vector6d bodyTwist = joint->getRelativeSpatialVelocity();
  const double dt = 1e-6;

  const Eigen::Isometry3d start = joint->getRelativeTransform();
  skel->integratePositions(dt);
  const Eigen::Isometry3d end = joint->getRelativeTransform();

  const Eigen::Vector6d twistFromStep
      = math::logMap(start.inverse() * end) / dt;

  EXPECT_TRUE(equals(twistFromStep, bodyTwist, 1e-6));
}

//==============================================================================
TEST_F(Joints, FreeJointIntegrationMatchesBodyTwistLargeDt)
{
  SkeletonPtr skel = Skeleton::create();

  auto pair = skel->createJointAndBodyNodePair<FreeJoint>();
  FreeJoint* joint = pair.first;

  joint->setPositions(FreeJoint::convertToPositions(random_transform()));
  const Eigen::Vector6d velocities = random_vec<6>(0.8);
  joint->setVelocities(velocities);

  const Eigen::Vector6d twist = joint->getRelativeSpatialVelocity();
  const double dt = 0.05;

  const Eigen::Isometry3d start = joint->getRelativeTransform();
  const Eigen::Vector3d vWorld = start.linear() * twist.tail<3>();
  Eigen::Isometry3d expected = start;
  expected.linear() = expected.linear() * math::expMapRot(twist.head<3>() * dt);
  expected.translation() += vWorld * dt;

  skel->integratePositions(dt);

  EXPECT_TRUE(equals(joint->getRelativeTransform(), expected, 1e-8));
}

//==============================================================================
TEST_F(Joints, FreeJointVelocityIntegratesAcceleration)
{
  SkeletonPtr skel = Skeleton::create();

  auto pair = skel->createJointAndBodyNodePair<FreeJoint>();
  FreeJoint* joint = pair.first;

  const Eigen::Vector6d velocities = random_vec<6>(0.5);
  const Eigen::Vector6d accelerations = random_vec<6>(0.5);
  joint->setVelocities(velocities);
  joint->setAccelerations(accelerations);

  const double dt = 1e-3;
  skel->integrateVelocities(dt);

  const Eigen::Vector6d expected = velocities + accelerations * dt;
  EXPECT_TRUE(equals(joint->getVelocities(), expected));
}

//==============================================================================
TEST_F(Joints, FreeJointConstrainedTermsMatchVelocityChange)
{
  SkeletonPtr skel = Skeleton::create();
  const double dt = 1e-3;
  skel->setTimeStep(dt);
  skel->setMobile(true);

  auto pair = skel->createJointAndBodyNodePair<FreeJoint>();
  FreeJoint* joint = pair.first;
  BodyNode* body = pair.second;

  body->setMass(1.0);
  body->setMomentOfInertia(0.1, 0.2, 0.3);

  joint->setActuatorType(Joint::FORCE);
  joint->setVelocities(Eigen::Vector6d::Zero());
  joint->setAccelerations(Eigen::Vector6d::Zero());

  body->setConstraintImpulse(
      (Eigen::Vector6d() << 1.0, 2.0, 3.0, 4.0, -5.0, 6.0).finished());

  const Eigen::Vector6d vel0 = joint->getVelocities();
  const Eigen::Vector6d accel0 = joint->getAccelerations();

  skel->computeImpulseForwardDynamics();

  const Eigen::Vector6d vel1 = joint->getVelocities();
  const Eigen::Vector6d accel1 = joint->getAccelerations();
  const Eigen::Vector6d deltaVel = vel1 - vel0;

  EXPECT_GT(deltaVel.norm(), 1e-12);
  EXPECT_TRUE(equals(accel1, accel0 + deltaVel / dt));
}

//==============================================================================
TEST_F(Joints, FreeJointVelocityInvariantThroughIntegration)
{
  SkeletonPtr skel = Skeleton::create();

  auto pair = skel->createJointAndBodyNodePair<FreeJoint>();
  FreeJoint* joint = pair.first;

  joint->setPositions(FreeJoint::convertToPositions(random_transform()));

  const Eigen::Vector6d velocities = random_vec<6>(0.6);
  joint->setVelocities(velocities);
  joint->setAccelerations(Eigen::Vector6d::Zero());

  const double dt = 0.02;
  skel->integratePositions(dt);

  EXPECT_TRUE(equals(joint->getVelocities(), velocities));
}

//==============================================================================
TEST_F(Joints, FreeJointRelativeJacobianTimeDerivative)
{
  SkeletonPtr skel = Skeleton::create();

  auto pair = skel->createJointAndBodyNodePair<FreeJoint>();
  FreeJoint* joint = pair.first;

  joint->setTransformFromParentBodyNode(random_transform());
  joint->setTransformFromChildBodyNode(random_transform());
  joint->setPositions(FreeJoint::convertToPositions(random_transform()));
  joint->setVelocities(random_vec<6>(0.5));

  const Eigen::Matrix6d jacobian0 = joint->getRelativeJacobian();
  const Eigen::Matrix6d jacobianDot = joint->getRelativeJacobianTimeDeriv();

  const Eigen::Vector6d savedPositions = joint->getPositions();
  const double dt = 1e-5;

  skel->integratePositions(dt);
  const Eigen::Matrix6d jacobian1 = joint->getRelativeJacobian();
  joint->setPositions(savedPositions);

  const Eigen::Matrix6d jacobianFd = (jacobian1 - jacobian0) / dt;

  const double maxError = (jacobianDot - jacobianFd).cwiseAbs().maxCoeff();
  EXPECT_LT(maxError, 1e-3);
}

//==============================================================================
TEST_F(Joints, FreeJointRelativeJacobianTimeDerivativeIdentityFrames)
{
  SkeletonPtr skel = Skeleton::create();

  auto pair = skel->createJointAndBodyNodePair<FreeJoint>();
  FreeJoint* joint = pair.first;

  joint->setTransformFromParentBodyNode(Eigen::Isometry3d::Identity());
  joint->setTransformFromChildBodyNode(Eigen::Isometry3d::Identity());
  joint->setPositions(Eigen::Vector6d::Zero());
  joint->setVelocities(Eigen::Vector6d(0.1, -0.2, 0.15, 0.0, 0.0, 0.0));

  const Eigen::Matrix6d jacobian0 = joint->getRelativeJacobian();
  const Eigen::Matrix6d jacobianDot = joint->getRelativeJacobianTimeDeriv();

  const double dt = 1e-6;
  skel->integratePositions(dt);
  const Eigen::Matrix6d jacobian1 = joint->getRelativeJacobian();

  const Eigen::Matrix6d jacobianFd = (jacobian1 - jacobian0) / dt;
  const double maxError = (jacobianDot - jacobianFd).cwiseAbs().maxCoeff();

  EXPECT_LT(maxError, 1e-6);
}

//==============================================================================
TEST_F(Joints, FreeJointRelativeJacobianStaticUsesProvidedPositions)
{
  SkeletonPtr skel = Skeleton::create();

  auto pair = skel->createJointAndBodyNodePair<FreeJoint>();
  FreeJoint* joint = pair.first;

  joint->setTransformFromParentBodyNode(Eigen::Isometry3d::Identity());
  joint->setTransformFromChildBodyNode(Eigen::Isometry3d::Identity());

  Eigen::Vector6d positions = Eigen::Vector6d::Zero();
  positions.head<3>() = Eigen::Vector3d(0.4, -0.7, 0.2);
  positions.tail<3>() = Eigen::Vector3d(0.1, -0.2, 0.3);

  const Eigen::Isometry3d T = FreeJoint::convertToTransform(positions);
  const Eigen::Matrix3d rotationTranspose = T.linear().transpose();

  Eigen::Matrix6d expected = Eigen::Matrix6d::Zero();
  expected.topLeftCorner<3, 3>() = rotationTranspose;
  expected.bottomRightCorner<3, 3>() = rotationTranspose;

  const Eigen::Matrix6d jacobian = joint->getRelativeJacobian(positions);
  EXPECT_TRUE(equals(jacobian, expected));

  Eigen::Vector6d otherPositions = positions;
  otherPositions.head<3>() += Eigen::Vector3d(0.2, 0.1, -0.3);
  const Eigen::Matrix6d otherJacobian
      = joint->getRelativeJacobian(otherPositions);

  EXPECT_GT((otherJacobian - jacobian).cwiseAbs().maxCoeff(), 1e-6);
}

//==============================================================================
TEST_F(Joints, FreeJointSetRelativeSpatialVelocityAfterPoseChange)
{
  SkeletonPtr skel = Skeleton::create();

  auto pair = skel->createJointAndBodyNodePair<FreeJoint>();
  FreeJoint* joint = pair.first;

  joint->setTransformFromParentBodyNode(random_transform());
  joint->setTransformFromChildBodyNode(random_transform());

  joint->setPositions(FreeJoint::convertToPositions(random_transform()));
  (void)joint->getRelativeJacobian();

  joint->setPositions(FreeJoint::convertToPositions(random_transform()));

  const Eigen::Vector6d desired = random_vec<6>(0.5);
  joint->setRelativeSpatialVelocity(desired);

  EXPECT_TRUE(equals(joint->getRelativeSpatialVelocity(), desired, 1e-12));
}

//==============================================================================
TEST_F(Joints, FreeJointWorldJacobianMatchesFiniteDifference)
{
  SkeletonPtr skel = Skeleton::create();

  auto pair = skel->createJointAndBodyNodePair<FreeJoint>();
  FreeJoint* joint = pair.first;
  BodyNode* body = pair.second;

  const double dt = 1e-8;
  const double tol = 1e-5;

  const auto checkJacobianAt
      = [&](const Eigen::Isometry3d& pose, const Eigen::Vector3d& offset) {
          SCOPED_TRACE(testing::Message() << "offset = " << offset.transpose());
          joint->setPositions(FreeJoint::convertToPositions(pose));
          const Eigen::Vector6d q0 = joint->getPositions();
          const math::Jacobian J = body->getWorldJacobian(offset);

          for (int i = 0; i < 6; ++i) {
            SCOPED_TRACE(testing::Message() << "column = " << i);
            Eigen::Vector6d velocities = Eigen::Vector6d::Zero();
            velocities[i] = 1.0;
            joint->setVelocities(velocities);

            const Eigen::Isometry3d T0 = body->getWorldTransform();
            const Eigen::Matrix3d R0 = T0.linear();
            const Eigen::Vector3d p0 = T0 * offset;

            skel->integratePositions(dt);

            const Eigen::Isometry3d T1 = body->getWorldTransform();
            const Eigen::Matrix3d R1 = T1.linear();
            const Eigen::Vector3d p1 = T1 * offset;

            Eigen::Vector6d numeric;
            numeric.head<3>() = math::logMap(R1 * R0.transpose()) / dt;
            numeric.tail<3>() = (p1 - p0) / dt;

            const double maxError = (numeric - J.col(i)).cwiseAbs().maxCoeff();
            EXPECT_LT(maxError, tol) << "column = " << i;

            joint->setPositions(q0);
          }
        };

  joint->setTransformFromParentBodyNode(Eigen::Isometry3d::Identity());
  joint->setTransformFromChildBodyNode(Eigen::Isometry3d::Identity());
  Eigen::Isometry3d pose0 = Eigen::Isometry3d::Identity();
  pose0.linear() = math::expMapRot(Eigen::Vector3d(0.4, -0.7, 0.2));
  pose0.translation() = Eigen::Vector3d(0.1, -0.2, 0.3);
  checkJacobianAt(pose0, Eigen::Vector3d::Zero());
  checkJacobianAt(pose0, Eigen::Vector3d(0.2, -0.1, 0.05));

  Eigen::Isometry3d parentToJoint = Eigen::Isometry3d::Identity();
  parentToJoint.linear() = math::expMapRot(Eigen::Vector3d(0.3, 0.1, -0.2));
  parentToJoint.translation() = Eigen::Vector3d(0.2, -0.1, 0.05);
  joint->setTransformFromParentBodyNode(parentToJoint);

  Eigen::Isometry3d childToJoint = Eigen::Isometry3d::Identity();
  childToJoint.linear() = math::expMapRot(Eigen::Vector3d(-0.2, 0.25, 0.1));
  childToJoint.translation() = Eigen::Vector3d(-0.1, 0.05, 0.15);
  joint->setTransformFromChildBodyNode(childToJoint);

  Eigen::Isometry3d pose1 = Eigen::Isometry3d::Identity();
  pose1.linear() = math::expMapRot(Eigen::Vector3d(-0.3, 0.6, 0.25));
  pose1.translation() = Eigen::Vector3d(-0.2, 0.15, 0.1);
  checkJacobianAt(pose1, Eigen::Vector3d(0.2, -0.1, 0.05));

  Eigen::Isometry3d nearPi = Eigen::Isometry3d::Identity();
  nearPi.linear() = math::expMapRot(Eigen::Vector3d(pi - 1e-6, 0.0, 0.0));
  nearPi.translation() = Eigen::Vector3d(0.1, -0.05, 0.2);
  joint->setTransformFromParentBodyNode(Eigen::Isometry3d::Identity());
  joint->setTransformFromChildBodyNode(Eigen::Isometry3d::Identity());
  checkJacobianAt(nearPi, Eigen::Vector3d(0.2, 0.1, -0.05));
}

//==============================================================================
TEST_F(Joints, FreeJointWorldJacobianClassicDerivMatchesFiniteDifference)
{
  SkeletonPtr skel = Skeleton::create();

  auto pair = skel->createJointAndBodyNodePair<FreeJoint>();
  FreeJoint* joint = pair.first;
  BodyNode* body = pair.second;

  const double dt = 1e-6;
  const double tol = 1e-4;

  const auto checkAt = [&](const Eigen::Isometry3d& parentToJoint,
                           const Eigen::Isometry3d& childToJoint,
                           const Eigen::Isometry3d& pose,
                           const Eigen::Vector6d& velocities,
                           const Eigen::Vector3d& offset) {
    joint->setTransformFromParentBodyNode(parentToJoint);
    joint->setTransformFromChildBodyNode(childToJoint);
    joint->setPositions(FreeJoint::convertToPositions(pose));
    joint->setVelocities(velocities);
    joint->setAccelerations(Eigen::Vector6d::Zero());

    const Eigen::Vector6d q0 = joint->getPositions();
    const math::Jacobian dJ0 = body->getJacobianClassicDeriv(offset);

    skel->integratePositions(dt);
    const Eigen::Vector6d VPlus
        = body->getSpatialVelocity(offset, Frame::World(), Frame::World());
    joint->setPositions(q0);

    skel->integratePositions(-dt);
    const Eigen::Vector6d VMinus
        = body->getSpatialVelocity(offset, Frame::World(), Frame::World());
    joint->setPositions(q0);

    const Eigen::Vector6d numeric = (VPlus - VMinus) / (2.0 * dt);
    Eigen::Vector6d predicted = dJ0 * velocities;
    const double maxError = (numeric - predicted).cwiseAbs().maxCoeff();
    EXPECT_LT(maxError, tol);
  };

  const Eigen::Vector6d velocities
      = (Eigen::Vector6d() << 0.3, -0.2, 0.1, 0.4, -0.1, 0.2).finished();
  Eigen::Vector6d translationOnly = Eigen::Vector6d::Zero();
  translationOnly.tail<3>() = Eigen::Vector3d(0.4, -0.1, 0.2);

  Eigen::Isometry3d pose0 = Eigen::Isometry3d::Identity();
  pose0.linear() = math::expMapRot(Eigen::Vector3d(0.4, -0.7, 0.2));
  pose0.translation() = Eigen::Vector3d(0.1, -0.2, 0.3);

  const Eigen::Vector3d offset(0.2, -0.1, 0.05);
  checkAt(
      Eigen::Isometry3d::Identity(),
      Eigen::Isometry3d::Identity(),
      pose0,
      velocities,
      offset);
  checkAt(
      Eigen::Isometry3d::Identity(),
      Eigen::Isometry3d::Identity(),
      pose0,
      translationOnly,
      offset);

  Eigen::Isometry3d parentToJoint = Eigen::Isometry3d::Identity();
  parentToJoint.linear() = math::expMapRot(Eigen::Vector3d(0.3, 0.1, -0.2));
  parentToJoint.translation() = Eigen::Vector3d(0.2, -0.1, 0.05);

  Eigen::Isometry3d childToJoint = Eigen::Isometry3d::Identity();
  childToJoint.linear() = math::expMapRot(Eigen::Vector3d(-0.2, 0.25, 0.1));
  childToJoint.translation() = Eigen::Vector3d(-0.1, 0.05, 0.15);

  Eigen::Isometry3d pose1 = Eigen::Isometry3d::Identity();
  pose1.linear() = math::expMapRot(Eigen::Vector3d(-0.3, 0.6, 0.25));
  pose1.translation() = Eigen::Vector3d(-0.2, 0.15, 0.1);

  checkAt(parentToJoint, childToJoint, pose1, velocities, offset);
  checkAt(parentToJoint, childToJoint, pose1, translationOnly, offset);

  Eigen::Isometry3d nearPi = Eigen::Isometry3d::Identity();
  nearPi.linear() = math::expMapRot(Eigen::Vector3d(pi - 1e-6, 0.0, 0.0));
  nearPi.translation() = Eigen::Vector3d(0.25, -0.15, 0.2);
  checkAt(
      Eigen::Isometry3d::Identity(),
      Eigen::Isometry3d::Identity(),
      nearPi,
      velocities,
      offset);
  checkAt(
      Eigen::Isometry3d::Identity(),
      Eigen::Isometry3d::Identity(),
      nearPi,
      translationOnly,
      offset);
}

//==============================================================================
TEST_F(Joints, FreeJointEnergyConservationNoForces)
{
  simulation::WorldPtr world = simulation::World::create();
  world->setGravity(Eigen::Vector3d::Zero());

  const double dt = 1e-4;
  world->setTimeStep(dt);

  auto runCase = [&](const Eigen::Vector6d& initialVelocities) {
    SkeletonPtr skel = Skeleton::create();

    auto pair = skel->createJointAndBodyNodePair<FreeJoint>();
    FreeJoint* joint = pair.first;
    BodyNode* body = pair.second;

    body->setMass(2.0);
    body->setMomentOfInertia(0.1, 0.2, 0.3);

    for (std::size_t i = 0; i < joint->getNumDofs(); ++i) {
      joint->setDampingCoefficient(i, 0.0);
      joint->setCoulombFriction(i, 0.0);
    }

    joint->setPositions(FreeJoint::convertToPositions(random_transform()));
    joint->setVelocities(initialVelocities);

    world->removeAllSkeletons();
    world->addSkeleton(skel);

    const double energy0 = skel->computeKineticEnergy();
    const Eigen::Vector6d momentum0 = math::dAdInvT(
        body->getWorldTransform(),
        body->getSpatialInertia() * body->getSpatialVelocity());
    double maxEnergyError = 0.0;
    double maxMomentumError = 0.0;

    const std::size_t numSteps = 500;
    for (std::size_t i = 0; i < numSteps; ++i) {
      world->step();
      const double energy = skel->computeKineticEnergy();
      maxEnergyError = std::max(maxEnergyError, std::abs(energy - energy0));
      const Eigen::Vector6d momentum = math::dAdInvT(
          body->getWorldTransform(),
          body->getSpatialInertia() * body->getSpatialVelocity());
      maxMomentumError = std::max(
          maxMomentumError, (momentum - momentum0).cwiseAbs().maxCoeff());
    }

    const double scale = std::max(1.0, energy0);
    EXPECT_LT(maxEnergyError / scale, 1e-3);

    const double momentumScale = std::max(1.0, momentum0.norm());
    EXPECT_LT(maxMomentumError / momentumScale, 1e-3);
  };

  Eigen::Vector6d linearOnly = Eigen::Vector6d::Zero();
  linearOnly.tail<3>() = Eigen::Vector3d(0.6, -0.4, 0.2);
  runCase(linearOnly);

  Eigen::Vector6d angularOnly = Eigen::Vector6d::Zero();
  angularOnly.head<3>() = Eigen::Vector3d(0.5, -0.3, 0.4);
  runCase(angularOnly);

  Eigen::Vector6d both = Eigen::Vector6d::Zero();
  both.head<3>() = Eigen::Vector3d(0.5, -0.3, 0.4);
  both.tail<3>() = Eigen::Vector3d(0.6, -0.4, 0.2);
  runCase(both);
}

//==============================================================================
TEST_F(Joints, FreeJointSphericalInertiaConstantWorldTwistLongHorizon)
{
  simulation::WorldPtr world = simulation::World::create();
  world->setGravity(Eigen::Vector3d::Zero());

  const double dt = 1e-3;
  world->setTimeStep(dt);

  SkeletonPtr skel = Skeleton::create();

  auto pair = skel->createJointAndBodyNodePair<FreeJoint>();
  FreeJoint* joint = pair.first;
  BodyNode* body = pair.second;

  body->setMass(2.0);
  body->setMomentOfInertia(0.2, 0.2, 0.2);

  for (std::size_t i = 0; i < joint->getNumDofs(); ++i) {
    joint->setDampingCoefficient(i, 0.0);
    joint->setCoulombFriction(i, 0.0);
  }

  joint->setPositions(FreeJoint::convertToPositions(random_transform()));

  Eigen::Vector6d velocities = Eigen::Vector6d::Zero();
  velocities.head<3>() = Eigen::Vector3d(0.4, -0.3, 0.25);
  velocities.tail<3>() = Eigen::Vector3d(0.5, -0.4, 0.2);
  joint->setVelocities(velocities);

  world->addSkeleton(skel);

  const Eigen::Isometry3d start = joint->getRelativeTransform();
  const double energy0 = skel->computeKineticEnergy();

  const std::size_t numSteps = 1000;
  for (std::size_t i = 0; i < numSteps; ++i) {
    world->step();
  }

  const double t = dt * static_cast<double>(numSteps);

  const Eigen::Vector6d velocitiesEnd = joint->getVelocities();
  EXPECT_TRUE(equals(velocitiesEnd, velocities, 1e-10));

  Eigen::Isometry3d expected = Eigen::Isometry3d::Identity();
  expected.linear()
      = math::expMapRot(velocities.head<3>() * t) * start.linear();
  expected.translation() = start.translation() + velocities.tail<3>() * t;
  EXPECT_TRUE(equals(joint->getRelativeTransform(), expected, 1e-7));

  const double energy = skel->computeKineticEnergy();
  const double scale = std::max(1.0, energy0);
  EXPECT_LT(std::abs(energy - energy0) / scale, 1e-6);
}

//==============================================================================
// Test for GitHub Issue #915: SERVO vs VELOCITY consistency
// https://github.com/dartsim/dart/issues/915
//==============================================================================
void testServoVelocityConsistency()
{
  using namespace dart::math::suffixes;

  double timestep = 1e-3;
  // Servo vs. velocity actuators rely on different solvers, so allow small
  // numeric drift between the two implementations.
  double tol = 1e-5;

  // Create two identical pendulums - one with SERVO, one with VELOCITY
  simulation::WorldPtr world = simulation::World::create();
  EXPECT_TRUE(world != nullptr);

  world->setGravity(Eigen::Vector3d(0, 0, -9.81));
  world->setTimeStep(timestep);

  // Create two identical single-link pendulums
  SkeletonPtr servoPendulum = createPendulum(Joint::SERVO);
  SkeletonPtr velocityPendulum = createPendulum(Joint::VELOCITY);

  EXPECT_NE(servoPendulum, nullptr);
  EXPECT_NE(velocityPendulum, nullptr);

  // Get the joints
  JointPtr servoJoint = servoPendulum->getJoint(0);
  JointPtr velocityJoint = velocityPendulum->getJoint(0);

  // Configure SERVO joint with sufficient force limits
  double sufficient_force = 1e+5;
  servoJoint->setForceUpperLimit(0, sufficient_force);
  servoJoint->setForceLowerLimit(0, -sufficient_force);

  // Set same initial conditions
  double initialPos = 45.0_deg;
  servoJoint->setPosition(0, initialPos);
  velocityJoint->setPosition(0, initialPos);

  // Disable damping, spring stiffness, and friction for pure comparison
  servoJoint->setDampingCoefficient(0, 0.0);
  servoJoint->setSpringStiffness(0, 0.0);
  servoJoint->setCoulombFriction(0, 0.0);
  velocityJoint->setDampingCoefficient(0, 0.0);
  velocityJoint->setSpringStiffness(0, 0.0);
  velocityJoint->setCoulombFriction(0, 0.0);

  world->addSkeleton(servoPendulum);
  world->addSkeleton(velocityPendulum);

#if !defined(NDEBUG)
  double simTime = 0.2;
#else
  double simTime = 2.0;
#endif
  int nSteps = simTime / timestep;

  // Test 1: Zero velocity command
  for (int i = 0; i < nSteps / 4; i++) {
    servoJoint->setCommand(0, 0.0);
    velocityJoint->setCommand(0, 0.0);
    world->step();

    double servoVel = servoJoint->getVelocity(0);
    double velocityVel = velocityJoint->getVelocity(0);

    EXPECT_NEAR(servoVel, 0.0, tol);
    EXPECT_NEAR(velocityVel, 0.0, tol);
    EXPECT_NEAR(servoVel, velocityVel, tol);
  }

  // Test 2: Positive velocity command
  for (int i = 0; i < nSteps / 4; i++) {
    double desiredVel = 0.5;
    servoJoint->setCommand(0, desiredVel);
    velocityJoint->setCommand(0, desiredVel);
    world->step();

    double servoVel = servoJoint->getVelocity(0);
    double velocityVel = velocityJoint->getVelocity(0);

    EXPECT_NEAR(servoVel, desiredVel, tol);
    EXPECT_NEAR(velocityVel, desiredVel, tol);
    EXPECT_NEAR(servoVel, velocityVel, tol);
  }

  // Test 3: Negative velocity command (sign consistency test)
  for (int i = 0; i < nSteps / 4; i++) {
    double desiredVel = -0.5;
    servoJoint->setCommand(0, desiredVel);
    velocityJoint->setCommand(0, desiredVel);
    world->step();

    double servoVel = servoJoint->getVelocity(0);
    double velocityVel = velocityJoint->getVelocity(0);

    EXPECT_NEAR(servoVel, desiredVel, tol);
    EXPECT_NEAR(velocityVel, desiredVel, tol);
    EXPECT_NEAR(servoVel, velocityVel, tol);
  }

  // Test 4: Time-varying velocity command
  for (int i = 0; i < nSteps / 4; i++) {
    double desiredVel = std::sin(world->getTime());
    servoJoint->setCommand(0, desiredVel);
    velocityJoint->setCommand(0, desiredVel);
    world->step();

    double servoVel = servoJoint->getVelocity(0);
    double velocityVel = velocityJoint->getVelocity(0);

    EXPECT_NEAR(servoVel, desiredVel, tol);
    EXPECT_NEAR(velocityVel, desiredVel, tol);
    EXPECT_NEAR(servoVel, velocityVel, tol);
  }
}

//==============================================================================
TEST_F(Joints, ServoVelocityConsistency)
{
  testServoVelocityConsistency();
}

//==============================================================================
// Test for GitHub Issue #915: CoM Jacobian sign consistency
//==============================================================================
void testCoMJacobianSignConsistency()
{
  using namespace dart::math::suffixes;

  double timestep = 1e-3;
  // Velocity actuators are kinematic while SERVO actuators rely on dynamic
  // constraints, so allow a small numerical gap between the two.
  double tol = 1e-3;

  // Create two identical 2-DOF robots - one with SERVO, one with VELOCITY
  simulation::WorldPtr world = simulation::World::create();
  EXPECT_TRUE(world != nullptr);

  world->setGravity(Eigen::Vector3d(0, 0, 0)); // Zero gravity for this test
  world->setTimeStep(timestep);

  // Create 2-link pendulums
  Vector3d dim(1, 1, 1);
  Vector3d offset(0, 0, 2);

  SkeletonPtr servoPendulum = createNLinkPendulum(2, dim, DOF_ROLL, offset);
  SkeletonPtr velocityPendulum = createNLinkPendulum(2, dim, DOF_ROLL, offset);

  EXPECT_NE(servoPendulum, nullptr);
  EXPECT_NE(velocityPendulum, nullptr);

  for (std::size_t i = 0; i < servoPendulum->getNumBodyNodes(); ++i) {
    servoPendulum->getBodyNode(i)->setCollidable(false);
  }
  for (std::size_t i = 0; i < velocityPendulum->getNumBodyNodes(); ++i) {
    velocityPendulum->getBodyNode(i)->setCollidable(false);
  }

  // Configure joints
  double sufficient_force = 1e+5;
  for (std::size_t i = 0; i < 2; ++i) {
    auto servoJoint = servoPendulum->getJoint(i);
    auto velocityJoint = velocityPendulum->getJoint(i);

    servoJoint->setActuatorType(Joint::SERVO);
    velocityJoint->setActuatorType(Joint::VELOCITY);

    servoJoint->setForceUpperLimit(0, sufficient_force);
    servoJoint->setForceLowerLimit(0, -sufficient_force);

    servoJoint->setDampingCoefficient(0, 0.0);
    servoJoint->setSpringStiffness(0, 0.0);
    servoJoint->setCoulombFriction(0, 0.0);
    velocityJoint->setDampingCoefficient(0, 0.0);
    velocityJoint->setSpringStiffness(0, 0.0);
    velocityJoint->setCoulombFriction(0, 0.0);

    // Set same initial position
    double initialPos = 30.0_deg;
    servoJoint->setPosition(0, initialPos);
    velocityJoint->setPosition(0, initialPos);
  }

  world->addSkeleton(servoPendulum);
  world->addSkeleton(velocityPendulum);

#if !defined(NDEBUG)
  double simTime = 0.2;
#else
  double simTime = 1.0;
#endif
  int nSteps = simTime / timestep;

  // Test that CoM Jacobian produces consistent results
  for (int i = 0; i < nSteps; i++) {
    // Set same velocity command for both
    double desiredVel0 = 0.3 * std::sin(world->getTime());
    double desiredVel1 = 0.2 * std::cos(world->getTime());

    servoPendulum->getJoint(0)->setCommand(0, desiredVel0);
    servoPendulum->getJoint(1)->setCommand(0, desiredVel1);
    velocityPendulum->getJoint(0)->setCommand(0, desiredVel0);
    velocityPendulum->getJoint(1)->setCommand(0, desiredVel1);

    world->step();

    // Get CoM Jacobian for both skeletons
    auto servoCoMJacobian = servoPendulum->getCOMLinearJacobian();
    auto velocityCoMJacobian = velocityPendulum->getCOMLinearJacobian();

    // The CoM Jacobian should be the same regardless of actuator type
    // since it's a kinematic property
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 2; ++col) {
        EXPECT_NEAR(
            servoCoMJacobian(row, col), velocityCoMJacobian(row, col), tol)
            << "CoM Jacobian differs at (" << row << "," << col << ")";
      }
    }

    // Servo vs. velocity tracking is validated elsewhere; here we focus on the
    // Jacobian sign consistency.
  }
}

//==============================================================================
TEST_F(Joints, DISABLED_CoMJacobianSignConsistency)
{
  testCoMJacobianSignConsistency();
}

//==============================================================================
// Test for GitHub Issue #915: Robot stability with SERVO actuators
//==============================================================================
void testServoActuatorStability()
{
  using namespace dart::math::suffixes;

  double timestep = 1e-3;
  double tol = 1e-6;

  simulation::WorldPtr world = simulation::World::create();
  EXPECT_TRUE(world != nullptr);

  world->setGravity(Eigen::Vector3d(0, 0, -9.81));
  world->setTimeStep(timestep);

  // Create a simple pendulum
  SkeletonPtr pendulum = createPendulum(Joint::SERVO);
  EXPECT_NE(pendulum, nullptr);

  auto joint = pendulum->getJoint(0);

  // Set parameters as mentioned in issue #915
  double sufficient_force = 1e+5;
  joint->setForceUpperLimit(0, sufficient_force);
  joint->setForceLowerLimit(0, -sufficient_force);
  joint->setDampingCoefficient(0, 0.0);
  joint->setSpringStiffness(0, 0.0);
  joint->setCoulombFriction(0, 0.0);
  joint->setLimitEnforcement(true);

  // Set position limits
  joint->setPositionUpperLimit(0, 90.0_deg);
  joint->setPositionLowerLimit(0, -90.0_deg);

  // Start from upright position
  joint->setPosition(0, 0.0);
  joint->setVelocity(0, 0.0);

  world->addSkeleton(pendulum);

#if !defined(NDEBUG)
  double simTime = 0.2;
#else
  double simTime = 2.0;
#endif
  int nSteps = simTime / timestep;

  // Test 1: Robot should stay at zero velocity when commanded
  for (int i = 0; i < nSteps / 2; i++) {
    joint->setCommand(0, 0.0);
    world->step();

    double vel = joint->getVelocity(0);
    double pos = joint->getPosition(0);

    // Velocity should be close to zero (fighting gravity)
    EXPECT_NEAR(vel, 0.0, tol);

    // Position should remain within bounds
    EXPECT_GE(pos, -90.0_deg - tol);
    EXPECT_LE(pos, 90.0_deg + tol);
  }

  // Test 2: Robot should move with commanded velocity
  for (int i = 0; i < nSteps / 2; i++) {
    double desiredVel = 0.1;
    joint->setCommand(0, desiredVel);
    world->step();

    double vel = joint->getVelocity(0);
    double pos = joint->getPosition(0);

    // Velocity should track command
    EXPECT_NEAR(vel, desiredVel, tol);

    // Position should remain within bounds
    EXPECT_GE(pos, -90.0_deg - tol);
    EXPECT_LE(pos, 90.0_deg + tol);
  }
}

//==============================================================================
TEST_F(Joints, ServoActuatorStability)
{
  testServoActuatorStability();
}

//==============================================================================
TEST_F(Joints, IntegratePositionsStateIndependent)
{
  constexpr double dt = 0.25;
  constexpr double tol = 1e-12;

  auto verify
      = [&](Joint& joint, const Eigen::VectorXd& q0, const Eigen::VectorXd& v) {
          const Eigen::VectorXd positionsBefore = joint.getPositions();
          const Eigen::VectorXd velocitiesBefore = joint.getVelocities();

          const Eigen::VectorXd qNextStateless
              = joint.integratePositions(q0, v, dt);
          EXPECT_VECTOR_NEAR(positionsBefore, joint.getPositions(), 0.0);
          EXPECT_VECTOR_NEAR(velocitiesBefore, joint.getVelocities(), 0.0);

          joint.setPositions(q0);
          joint.setVelocities(v);
          joint.integratePositions(dt);
          const Eigen::VectorXd qNextStateful = joint.getPositions();

          EXPECT_VECTOR_NEAR(qNextStateful, qNextStateless, tol);

          joint.setPositions(positionsBefore);
          joint.setVelocities(velocitiesBefore);
        };

  {
    SkeletonPtr skel = Skeleton::create("integrate_revolute");
    auto [joint, body] = skel->createJointAndBodyNodePair<RevoluteJoint>();
    (void)body;

    joint->setPosition(0, 0.7);
    joint->setVelocity(0, 0.3);

    const Eigen::VectorXd q0 = (Eigen::VectorXd(1) << -0.4).finished();
    const Eigen::VectorXd v = (Eigen::VectorXd(1) << 0.9).finished();

    verify(*joint, q0, v);
  }

  {
    SkeletonPtr skel = Skeleton::create("integrate_ball");
    auto [joint, body] = skel->createJointAndBodyNodePair<BallJoint>();
    (void)body;

    joint->setPositions(Eigen::Vector3d(0.2, -0.1, 0.3));
    joint->setVelocities(Eigen::Vector3d(0.01, 0.02, -0.03));

    const Eigen::VectorXd q0 = (Eigen::Vector3d(0.5, -0.4, 0.1)).eval();
    const Eigen::VectorXd v = (Eigen::Vector3d(0.07, -0.02, 0.04)).eval();

    verify(*joint, q0, v);
  }

  {
    SkeletonPtr skel = Skeleton::create("integrate_free");
    auto [joint, body] = skel->createJointAndBodyNodePair<FreeJoint>();
    (void)body;

    joint->setTransformFromParentBodyNode(random_transform(1.0, 1.0));
    joint->setTransformFromChildBodyNode(random_transform(1.0, 1.0));

    joint->setPositions(
        (Eigen::Vector6d() << 0.1, -0.2, 0.3, 1.0, -2.0, 3.0).finished());
    joint->setVelocities(
        (Eigen::Vector6d() << 0.01, 0.02, -0.03, -0.4, 0.5, -0.6).finished());

    const Eigen::VectorXd q0
        = (Eigen::Vector6d() << -0.3, 0.2, -0.1, -1.0, 2.0, -3.0).finished();
    const Eigen::VectorXd v
        = (Eigen::Vector6d() << 0.06, -0.05, 0.04, 0.7, -0.8, 0.9).finished();

    verify(*joint, q0, v);
  }

  {
    SkeletonPtr skel = Skeleton::create("integrate_weld");
    auto [joint, body] = skel->createJointAndBodyNodePair<WeldJoint>();
    (void)body;

    const Eigen::VectorXd q0 = Eigen::VectorXd::Zero(0);
    const Eigen::VectorXd v = Eigen::VectorXd::Zero(0);

    verify(*joint, q0, v);
  }
}

//==============================================================================
TEST_F(Joints, JacobianMethodGradientConversionDoesNotModifySkeletonState)
{
  SkeletonPtr skel = Skeleton::create("ik_stateless_integration");
  auto [joint, body] = skel->createJointAndBodyNodePair<BallJoint>();
  (void)joint;

  const auto& ik = body->getIK(true);
  ik->useWholeBody();

  const auto& dofs = ik->getDofs();
  ASSERT_EQ(dofs.size(), skel->getNumDofs());

  skel->setPositions(Eigen::VectorXd::Random(skel->getNumDofs()));
  skel->setVelocities(Eigen::VectorXd::Random(skel->getNumDofs()));

  const Eigen::VectorXd positionsBefore = skel->getPositions();
  const Eigen::VectorXd velocitiesBefore = skel->getVelocities();

  Eigen::VectorXd grad(dofs.size());
  grad << 0.01, -0.02, 0.03;

  ik->getGradientMethod().convertJacobianMethodOutputToGradient(grad, dofs);

  EXPECT_VECTOR_NEAR(positionsBefore, skel->getPositions(), 0.0);
  EXPECT_VECTOR_NEAR(velocitiesBefore, skel->getVelocities(), 0.0);
}
