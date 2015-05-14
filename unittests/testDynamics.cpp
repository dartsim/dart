/*
 * Copyright (c) 2011-2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sumit Jain <sumit@cc.gatech.edu>,
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

#include <iostream>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "TestHelpers.h"

#include "dart/common/Console.h"
#include "dart/math/Geometry.h"
#include "dart/math/Helpers.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/dynamics/SimpleFrame.h"
#include "dart/simulation/World.h"
#include "dart/utils/SkelParser.h"
#include "dart/utils/Paths.h"

using namespace Eigen;
using namespace dart;

//==============================================================================
class DynamicsTest : public ::testing::Test
{
public:
  // Get Skel file list to test.
  const std::vector<std::string>& getList() const;

  // Get reference frames
  const std::vector<SimpleFrame*>& getRefFrames() const;

  // Randomize the properties of all the reference frames
  void randomizeRefFrames();

  // Get mass matrix of _skel using Jacobians and inertias of each body
  // in _skel.
  MatrixXd getMassMatrix(dynamics::SkeletonPtr _skel);

  // Get augmented mass matrix of _skel using Jacobians and inertias of
  // each body in _skel.
  MatrixXd getAugMassMatrix(dynamics::SkeletonPtr _skel);

  // Compare velocities computed by recursive method, Jacobian, and finite
  // difference.
  void testJacobians(const std::string& _fileName);

  // Compare accelerations computed by recursive method, Jacobian, and finite
  // difference.
  void testFiniteDifference(const std::string& _fileName);

  // Compare dynamics terms in equations of motion such as mass matrix, mass
  // inverse matrix, Coriolis force vector, gravity force vector, and external
  // force vector.
  void compareEquationsOfMotion(const std::string& _fileName);

  // Test skeleton's COM and its related quantities.
  void testCenterOfMass(const std::string& _fileName);

  // Test if the com acceleration is equal to the gravity
  void testCenterOfMassFreeFall(const std::string& _fileName);

  //
  void testConstraintImpulse(const std::string& _fileName);

  // Test impulse based dynamics
  void testImpulseBasedDynamics(const std::string& _fileName);

protected:
  // Sets up the test fixture.
  virtual void SetUp();

  // Skel file list.
  std::vector<std::string> fileList;

  std::vector<SimpleFrame*> refFrames;
};

//==============================================================================
void DynamicsTest::SetUp()
{
  // Create a list of skel files to test with
  fileList.push_back(DART_DATA_PATH"skel/test/chainwhipa.skel");
  fileList.push_back(DART_DATA_PATH"skel/test/single_pendulum.skel");
  fileList.push_back(DART_DATA_PATH"skel/test/single_pendulum_euler_joint.skel");
  fileList.push_back(DART_DATA_PATH"skel/test/single_pendulum_ball_joint.skel");
  fileList.push_back(DART_DATA_PATH"skel/test/double_pendulum.skel");
  fileList.push_back(DART_DATA_PATH"skel/test/double_pendulum_euler_joint.skel");
  fileList.push_back(DART_DATA_PATH"skel/test/double_pendulum_ball_joint.skel");
  fileList.push_back(DART_DATA_PATH"skel/test/serial_chain_revolute_joint.skel");
  fileList.push_back(DART_DATA_PATH"skel/test/serial_chain_eulerxyz_joint.skel");
  fileList.push_back(DART_DATA_PATH"skel/test/serial_chain_ball_joint.skel");
  fileList.push_back(DART_DATA_PATH"skel/test/serial_chain_ball_joint_20.skel");
  fileList.push_back(DART_DATA_PATH"skel/test/serial_chain_ball_joint_40.skel");
  fileList.push_back(DART_DATA_PATH"skel/test/simple_tree_structure.skel");
  fileList.push_back(DART_DATA_PATH"skel/test/simple_tree_structure_euler_joint.skel");
  fileList.push_back(DART_DATA_PATH"skel/test/simple_tree_structure_ball_joint.skel");
  fileList.push_back(DART_DATA_PATH"skel/test/tree_structure.skel");
  fileList.push_back(DART_DATA_PATH"skel/test/tree_structure_euler_joint.skel");
  fileList.push_back(DART_DATA_PATH"skel/test/tree_structure_ball_joint.skel");
  fileList.push_back(DART_DATA_PATH"skel/fullbody1.skel");

  // Create a list of reference frames to use during tests
  refFrames.push_back(new SimpleFrame(Frame::World(), "refFrame1"));
  refFrames.push_back(new SimpleFrame(refFrames.back(), "refFrame2"));
  refFrames.push_back(new SimpleFrame(refFrames.back(), "refFrame3"));
  refFrames.push_back(new SimpleFrame(refFrames.back(), "refFrame4"));
  refFrames.push_back(new SimpleFrame(Frame::World(), "refFrame5"));
  refFrames.push_back(new SimpleFrame(refFrames.back(), "refFrame6"));
}

//==============================================================================
const std::vector<std::string>& DynamicsTest::getList() const
{
  return fileList;
}

//==============================================================================
const std::vector<SimpleFrame*>& DynamicsTest::getRefFrames() const
{
  return refFrames;
}

//==============================================================================
void DynamicsTest::randomizeRefFrames()
{
  for(size_t i=0; i<refFrames.size(); ++i)
  {
    SimpleFrame* F = refFrames[i];

    Eigen::Vector3d p = randomVector<3>(100);
    Eigen::Vector3d theta = randomVector<3>(2*M_PI);

    Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
    tf.translate(p);
    tf.linear() = math::eulerXYZToMatrix(theta);

    F->setRelativeTransform(tf);
    F->setRelativeSpatialVelocity(randomVector<6>(100));
    F->setRelativeSpatialAcceleration(randomVector<6>(100));
  }
}

//==============================================================================
MatrixXd DynamicsTest::getMassMatrix(dynamics::SkeletonPtr _skel)
{
  int skelDof = _skel->getNumDofs();

  MatrixXd skelM = MatrixXd::Zero(skelDof, skelDof);  // Mass matrix of skeleton
  MatrixXd M;  // Body mass
  MatrixXd I;  // Body inertia
  MatrixXd J;  // Body Jacobian

  for (size_t i = 0; i < _skel->getNumBodyNodes(); ++i)
  {
    dynamics::BodyNode* body = _skel->getBodyNode(i);

    int dof = body->getNumDependentGenCoords();
    I = body->getSpatialInertia();
    J = body->getJacobian();

    EXPECT_EQ(I.rows(), 6);
    EXPECT_EQ(I.cols(), 6);
    EXPECT_EQ(J.rows(), 6);
    EXPECT_EQ(J.cols(), dof);

    M = J.transpose() * I * J;  // (dof x dof) matrix

    for (int j = 0; j < dof; ++j)
    {
      int jIdx = body->getDependentGenCoordIndex(j);

      for (int k = 0; k < dof; ++k)
      {
        int kIdx = body->getDependentGenCoordIndex(k);

        skelM(jIdx, kIdx) += M(j, k);
      }
    }
  }

  return skelM;
}

//==============================================================================
MatrixXd DynamicsTest::getAugMassMatrix(dynamics::SkeletonPtr _skel)
{
  int    dof = _skel->getNumDofs();
  double dt  = _skel->getTimeStep();

  MatrixXd M = getMassMatrix(_skel);
  MatrixXd D = MatrixXd::Zero(dof, dof);
  MatrixXd K = MatrixXd::Zero(dof, dof);
  MatrixXd AugM;

  // Compute diagonal matrices of joint damping and joint stiffness
  for (size_t i = 0; i < _skel->getNumBodyNodes(); ++i)
  {
    dynamics::BodyNode* body  = _skel->getBodyNode(i);
    dynamics::Joint*    joint = body->getParentJoint();

    EXPECT_TRUE(body  != NULL);
    EXPECT_TRUE(joint != NULL);

    int dof = joint->getNumDofs();

    for (int j = 0; j < dof; ++j)
    {
      int idx = joint->getIndexInSkeleton(j);

      D(idx, idx) = joint->getDampingCoefficient(j);
      K(idx, idx) = joint->getSpringStiffness(j);
    }
  }

  AugM = M + (dt * D) + (dt * dt * K);

  return AugM;
}

template <typename T>
void printComparisonError(const std::string& _comparison,
                          const std::string& _name,
                          const std::string& _frame,
                          const T& fk,
                          const T& jac)
{
  std::cout << "Disagreement between FK and Jacobian results for "
            << _comparison << " of '" << _name
            << "' with a reference Frame of '" << _frame << "'\n"
            << "FK:  " << fk.transpose() << "\n"
            << "Jac: " << jac.transpose() << "\n";
}

//==============================================================================
void compareBodyNodeFkToJacobian(const BodyNode* bn,
                                 const Frame* refFrame,
                                 double tolerance)
{
  using math::Jacobian;
  using math::LinearJacobian;
  using math::AngularJacobian;

  ConstSkeletonPtr skel = bn->getSkeleton();

  VectorXd dq  = skel->getVelocities();
  VectorXd ddq = skel->getAccelerations();

  const std::vector<size_t>& coords = bn->getDependentGenCoordIndices();
  VectorXd dqSeg  = skel->getVelocitySegment(coords);
  VectorXd ddqSeg = skel->getAccelerationSegment(coords);

  //-- Spatial Jacobian tests --------------------------------------------------

  Vector6d SpatialVelFk = bn->getSpatialVelocity(Frame::World(), refFrame);
  Vector6d SpatialAccFk = bn->getSpatialAcceleration(Frame::World(), refFrame);

  Jacobian SpatialJacSeg      = bn->getJacobian(refFrame);
  Jacobian SpatialJacDerivSeg = bn->getJacobianSpatialDeriv(refFrame);

  Vector6d SpatialVelJacSeg = SpatialJacSeg * dqSeg;
  Vector6d SpatialAccJacSeg = SpatialJacSeg * ddqSeg
                              + SpatialJacDerivSeg * dqSeg;

  Jacobian SpatialJac      = skel->getJacobian(bn, refFrame);
  Jacobian SpatialJacDeriv = skel->getJacobianSpatialDeriv(bn, refFrame);

  Vector6d SpatialVelJac = SpatialJac * dq;
  Vector6d SpatialAccJac = SpatialJac * ddq + SpatialJacDeriv * dq;

  bool spatialVelSegEqual = equals(SpatialVelFk, SpatialVelJacSeg, tolerance);
  bool spatialVelEqual    = equals(SpatialVelFk, SpatialVelJac, tolerance);
  EXPECT_TRUE( spatialVelSegEqual );
  EXPECT_TRUE( spatialVelEqual );
  if(!spatialVelSegEqual)
    printComparisonError("spatial velocity (seg)", bn->getName(),
                         refFrame->getName(), SpatialVelFk, SpatialVelJacSeg);
  if(!spatialVelEqual)
    printComparisonError("spatial velocity", bn->getName(),
                         refFrame->getName(), SpatialVelFk, SpatialVelJac);

  bool spatialAccSegEqual = equals(SpatialAccFk, SpatialAccJacSeg, tolerance);
  bool spatialAccEqual    = equals(SpatialAccFk, SpatialAccJac, tolerance);
  EXPECT_TRUE( spatialAccSegEqual );
  EXPECT_TRUE( spatialAccEqual );
  if(!spatialAccSegEqual)
    printComparisonError("spatial acceleration (seg)", bn->getName(),
                         refFrame->getName(), SpatialAccFk, SpatialAccJacSeg);
  if(!spatialAccEqual)
    printComparisonError("spatial acceleration", bn->getName(),
                         refFrame->getName(), SpatialAccFk, SpatialAccJac);

  //-- Linear Jacobian tests ---------------------------------------------------

  Vector3d LinearVelFk = bn->getLinearVelocity(Frame::World(), refFrame);
  Vector3d LinearAccFk = bn->getLinearAcceleration(Frame::World(), refFrame);

  LinearJacobian LinearJacSeg      = bn->getLinearJacobian(refFrame);
  LinearJacobian LinearJacDerivSeg = bn->getLinearJacobianDeriv(refFrame);

  Vector3d LinearVelJacSeg = LinearJacSeg * dqSeg;
  Vector3d LinearAccJacSeg = LinearJacSeg * ddqSeg + LinearJacDerivSeg * dqSeg;

  LinearJacobian LinearJac      = skel->getLinearJacobian(bn, refFrame);
  LinearJacobian LinearJacDeriv = skel->getLinearJacobianDeriv(bn, refFrame);

  Vector3d LinearVelJac = LinearJac * dq;
  Vector3d LinearAccJac = LinearJac * ddq + LinearJacDeriv * dq;

  bool linearVelSegEqual = equals(LinearVelFk, LinearVelJacSeg, tolerance);
  bool linearVelEqual    = equals(LinearVelFk, LinearVelJac, tolerance);
  EXPECT_TRUE( linearVelSegEqual );
  EXPECT_TRUE( linearVelEqual );
  if(!linearVelSegEqual)
    printComparisonError("linear velocity (seg)", bn->getName(),
                         refFrame->getName(), LinearVelFk, LinearVelJacSeg);
  if(!linearVelEqual)
    printComparisonError("linear velocity", bn->getName(),
                         refFrame->getName(), LinearVelFk, LinearVelJac);

  bool linearAccSegEqual = equals(LinearAccFk, LinearAccJacSeg, tolerance);
  bool linearAccEqual    = equals(LinearAccFk, LinearAccJac, tolerance);
  EXPECT_TRUE( linearAccSegEqual );
  EXPECT_TRUE( linearAccEqual );
  if(!linearAccSegEqual)
    printComparisonError("linear acceleration (seg)", bn->getName(),
                         refFrame->getName(), LinearAccFk, LinearAccJacSeg);
  if(!linearAccEqual)
    printComparisonError("linear acceleration", bn->getName(),
                         refFrame->getName(), LinearAccFk, LinearAccJac);

  //-- Angular Jacobian tests

  Vector3d AngularVelFk = bn->getAngularVelocity(Frame::World(), refFrame);
  Vector3d AngularAccFk = bn->getAngularAcceleration(Frame::World(), refFrame);

  AngularJacobian AngularJacSeg      = bn->getAngularJacobian(refFrame);
  AngularJacobian AngularJacDerivSeg = bn->getAngularJacobianDeriv(refFrame);

  Vector3d AngularVelJacSeg = AngularJacSeg * dqSeg;
  Vector3d AngularAccJacSeg = AngularJacSeg * ddqSeg
                              + AngularJacDerivSeg * dqSeg;

  AngularJacobian AngularJac      = skel->getAngularJacobian(bn, refFrame);
  AngularJacobian AngularJacDeriv = skel->getAngularJacobianDeriv(bn, refFrame);

  Vector3d AngularVelJac = AngularJac * dq;
  Vector3d AngularAccJac = AngularJac * ddq + AngularJacDeriv * dq;

  bool angularVelSegEqual = equals(AngularVelFk, AngularVelJacSeg, tolerance);
  bool angularVelEqual    = equals(AngularVelFk, AngularVelJac, tolerance);
  EXPECT_TRUE( angularVelSegEqual );
  EXPECT_TRUE( angularVelEqual );
  if(!angularVelSegEqual)
    printComparisonError("angular velocity (seg)", bn->getName(),
                         refFrame->getName(), AngularVelFk, AngularVelJacSeg);
  if(!angularVelEqual)
    printComparisonError("angular velocity", bn->getName(),
                         refFrame->getName(), AngularVelFk, AngularVelJac);

  bool angularAccSegEqual = equals(AngularAccFk, AngularAccJacSeg, tolerance);
  bool angularAccEqual    = equals(AngularAccFk, AngularAccJac, tolerance);
  EXPECT_TRUE( angularAccSegEqual );
  EXPECT_TRUE( angularAccEqual );
  if(!angularAccSegEqual)
    printComparisonError("angular acceleration (seg)", bn->getName(),
                         refFrame->getName(), AngularAccFk, AngularAccJacSeg);
  if(!angularAccEqual)
    printComparisonError("angular acceleration", bn->getName(),
                         refFrame->getName(), AngularAccFk, AngularAccJac);
}

//==============================================================================
void compareBodyNodeFkToJacobian(const BodyNode* bn,
                                 const Frame* refFrame,
                                 const Eigen::Vector3d& offset,
                                 double tolerance)
{
  using math::Jacobian;
  using math::LinearJacobian;
  using math::AngularJacobian;

  ConstSkeletonPtr skel = bn->getSkeleton();

  VectorXd dq  = skel->getVelocities();
  VectorXd ddq = skel->getAccelerations();

  const std::vector<size_t>& coords = bn->getDependentGenCoordIndices();
  VectorXd dqSeg  = skel->getVelocitySegment(coords);
  VectorXd ddqSeg = skel->getAccelerationSegment(coords);

  //-- Spatial Jacobian tests --------------------------------------------------

  Vector6d SpatialVelFk = bn->getSpatialVelocity(
        offset, Frame::World(), refFrame);
  Vector6d SpatialAccFk = bn->getSpatialAcceleration(
        offset, Frame::World(), refFrame);

  Jacobian SpatialJacSeg      = bn->getJacobian(offset, refFrame);
  Jacobian SpatialJacDerivSeg = bn->getJacobianSpatialDeriv(offset, refFrame);

  Vector6d SpatialVelJacSeg = SpatialJacSeg * dqSeg;
  Vector6d SpatialAccJacSeg = SpatialJacSeg * ddqSeg
                              + SpatialJacDerivSeg * dqSeg;

  Jacobian SpatialJac
      = skel->getJacobian(bn, offset, refFrame);
  Jacobian SpatialJacDeriv
      = skel->getJacobianSpatialDeriv(bn, offset, refFrame);

  Vector6d SpatialVelJac = SpatialJac * dq;
  Vector6d SpatialAccJac = SpatialJac * ddq + SpatialJacDeriv * dq;

  bool spatialVelSegEqual = equals(SpatialVelFk, SpatialVelJacSeg, tolerance);
  bool spatialVelEqual    = equals(SpatialVelFk, SpatialVelJac, tolerance);
  EXPECT_TRUE( spatialVelSegEqual );
  EXPECT_TRUE( spatialVelEqual );
  if(!spatialVelSegEqual)
    printComparisonError("spatial velocity w/ offset (seg)", bn->getName(),
                         refFrame->getName(), SpatialVelFk, SpatialVelJacSeg);
  if(!spatialVelEqual)
    printComparisonError("spatial velocity w/ offset", bn->getName(),
                         refFrame->getName(), SpatialVelFk, SpatialVelJac);

  bool spatialAccSegEqual = equals(SpatialAccFk, SpatialAccJacSeg, tolerance);
  bool spatialAccEqual    = equals(SpatialAccFk, SpatialAccJac, tolerance);
  EXPECT_TRUE( spatialAccSegEqual );
  EXPECT_TRUE( spatialAccEqual );
  if(!spatialAccSegEqual)
    printComparisonError("spatial acceleration w/ offset (seg)", bn->getName(),
                         refFrame->getName(), SpatialAccFk, SpatialAccJacSeg);
  if(!spatialAccEqual)
    printComparisonError("spatial acceleration w/ offset", bn->getName(),
                         refFrame->getName(), SpatialAccFk, SpatialAccJac);

  //-- Linear Jacobian tests ---------------------------------------------------

  Vector3d LinearVelFk
      = bn->getLinearVelocity(offset, Frame::World(), refFrame);
  Vector3d LinearAccFk
      = bn->getLinearAcceleration(offset, Frame::World(), refFrame);

  LinearJacobian LinearJacSeg
      = bn->getLinearJacobian(offset, refFrame);
  LinearJacobian LinearJacDerivSeg
      = bn->getLinearJacobianDeriv(offset, refFrame);

  Vector3d LinearVelJacSeg = LinearJacSeg * dqSeg;
  Vector3d LinearAccJacSeg = LinearJacSeg * ddqSeg + LinearJacDerivSeg * dqSeg;

  LinearJacobian LinearJac
      = skel->getLinearJacobian(bn, offset, refFrame);
  LinearJacobian LinearJacDeriv
      = skel->getLinearJacobianDeriv(bn, offset, refFrame);

  Vector3d LinearVelJac = LinearJac * dq;
  Vector3d LinearAccJac = LinearJac * ddq + LinearJacDeriv * dq;

  bool linearVelSegEqual = equals(LinearVelFk, LinearVelJacSeg, tolerance);
  bool linearVelEqual    = equals(LinearVelFk, LinearVelJac, tolerance);
  EXPECT_TRUE( linearVelSegEqual );
  EXPECT_TRUE( linearVelEqual );
  if(!linearVelSegEqual)
    printComparisonError("linear velocity w/ offset (seg)", bn->getName(),
                         refFrame->getName(), LinearVelFk, LinearVelJacSeg);
  if(!linearVelEqual)
    printComparisonError("linear velocity w/ offset", bn->getName(),
                         refFrame->getName(), LinearVelFk, LinearVelJac);

  bool linearAccSegEqual = equals(LinearAccFk, LinearAccJacSeg, tolerance);
  bool linearAccEqual    = equals(LinearAccFk, LinearAccJac, tolerance);
  EXPECT_TRUE( linearAccSegEqual );
  EXPECT_TRUE( linearAccEqual );
  if(!linearAccSegEqual)
    printComparisonError("linear acceleration w/ offset (seg)", bn->getName(),
                         refFrame->getName(), LinearAccFk, LinearAccJacSeg);
  if(!linearAccEqual)
    printComparisonError("linear acceleration w/ offset", bn->getName(),
                         refFrame->getName(), LinearAccFk, LinearAccJac);

  //-- Angular Jacobian tests --------------------------------------------------

  Vector3d AngularVelFk = bn->getAngularVelocity(Frame::World(), refFrame);
  Vector3d AngularAccFk = bn->getAngularAcceleration(Frame::World(), refFrame);

  AngularJacobian AngularJacSeg      = bn->getAngularJacobian(refFrame);
  AngularJacobian AngularJacDerivSeg = bn->getAngularJacobianDeriv(refFrame);

  Vector3d AngularVelJacSeg = AngularJacSeg * dqSeg;
  Vector3d AngularAccJacSeg = AngularJacSeg * ddqSeg
                              + AngularJacDerivSeg * dqSeg;

  AngularJacobian AngularJac      = skel->getAngularJacobian(bn, refFrame);
  AngularJacobian AngularJacDeriv = skel->getAngularJacobianDeriv(bn, refFrame);

  Vector3d AngularVelJac = AngularJac * dq;
  Vector3d AngularAccJac = AngularJac * ddq + AngularJacDeriv * dq;

  bool angularVelSegEqual = equals(AngularVelFk, AngularVelJacSeg, tolerance);
  bool angularVelEqual    = equals(AngularVelFk, AngularVelJac, tolerance);
  EXPECT_TRUE( angularVelSegEqual );
  EXPECT_TRUE( angularVelEqual );
  if(!angularVelSegEqual)
    printComparisonError("angular velocity w/ offset (seg)", bn->getName(),
                         refFrame->getName(), AngularVelFk, AngularVelJacSeg);
  if(!angularVelEqual)
    printComparisonError("angular velocity w/ offset", bn->getName(),
                         refFrame->getName(), AngularVelFk, AngularVelJac);

  bool angularAccSegEqual = equals(AngularAccFk, AngularAccJacSeg, tolerance);
  bool angularAccEqual    = equals(AngularAccFk, AngularAccJac, tolerance);
  EXPECT_TRUE( angularAccSegEqual );
  if(!angularAccSegEqual)
    printComparisonError("angular acceleration w/ offset (seg)", bn->getName(),
                         refFrame->getName(), AngularAccFk, AngularAccJacSeg);
  EXPECT_TRUE( angularAccEqual );
  if(!angularAccEqual)
    printComparisonError("angular acceleration w/ offset", bn->getName(),
                         refFrame->getName(), AngularAccFk, AngularAccJac);
}

//==============================================================================
void DynamicsTest::testJacobians(const std::string& _fileName)
{
  using namespace std;
  using namespace Eigen;
  using namespace dart;
  using namespace math;
  using namespace dynamics;
  using namespace simulation;
  using namespace utils;

  //----------------------------- Settings -------------------------------------
  const double TOLERANCE = 1.0e-6;
#ifndef NDEBUG  // Debug mode
  int nTestItr = 2;
#else
  int nTestItr = 100;
#endif
  double qLB  = -0.5 * DART_PI;
  double qUB  =  0.5 * DART_PI;
  double dqLB = -0.5 * DART_PI;
  double dqUB =  0.5 * DART_PI;
  double ddqLB = -0.5 * DART_PI;
  double ddqUB =  0.5 * DART_PI;
  Vector3d gravity(0.0, -9.81, 0.0);

  // load skeleton
  WorldPtr world = SkelParser::readWorld(_fileName);
  assert(world != nullptr);
  world->setGravity(gravity);

  //------------------------------ Tests ---------------------------------------
  for (size_t i = 0; i < world->getNumSkeletons(); ++i)
  {
    SkeletonPtr skeleton = world->getSkeleton(i);
    assert(skeleton != nullptr);
    int dof = skeleton->getNumDofs();

    for (int j = 0; j < nTestItr; ++j)
    {
      // For the second half of the tests, scramble up the Skeleton
      if(j > ceil(nTestItr/2))
      {
        SkeletonPtr copy = skeleton->clone();
        size_t maxNode = skeleton->getNumBodyNodes()-1;
        BodyNode* bn1 = skeleton->getBodyNode(ceil(math::random(0, maxNode)));
        BodyNode* bn2 = skeleton->getBodyNode(ceil(math::random(0, maxNode)));

        if(bn1 != bn2)
        {
          BodyNode* child = bn1->descendsFrom(bn2)? bn1 : bn2;
          BodyNode* parent = child == bn1? bn2 : bn1;

          child->moveTo(parent);
        }

        EXPECT_TRUE(skeleton->getNumBodyNodes() == copy->getNumBodyNodes());
        EXPECT_TRUE(skeleton->getNumDofs() == copy->getNumDofs());
      }

      // Generate a random state
      VectorXd q   = VectorXd(dof);
      VectorXd dq  = VectorXd(dof);
      VectorXd ddq = VectorXd(dof);
      for (int k = 0; k < dof; ++k)
      {
        q[k]   = math::random(qLB,   qUB);
        dq[k]  = math::random(dqLB,  dqUB);
        ddq[k] = math::random(ddqLB, ddqUB);
      }
      skeleton->setPositions(q);
      skeleton->setVelocities(dq);
      skeleton->setAccelerations(ddq);

      randomizeRefFrames();

      // For each body node
      for (size_t k = 0; k < skeleton->getNumBodyNodes(); ++k)
      {
        const BodyNode* bn = skeleton->getBodyNode(k);

        // Compare results using the World reference Frame
        compareBodyNodeFkToJacobian(bn, Frame::World(), TOLERANCE);
        // Compare results using this BodyNode's own reference Frame
        compareBodyNodeFkToJacobian(bn, bn, TOLERANCE);

        // Compare results using the randomized reference Frames
        for(size_t r=0; r<refFrames.size(); ++r)
        {
          compareBodyNodeFkToJacobian(bn, refFrames[r], TOLERANCE);
        }

        compareBodyNodeFkToJacobian(
              bn, Frame::World(), bn->getLocalCOM(), TOLERANCE);
        compareBodyNodeFkToJacobian(bn, bn, bn->getLocalCOM(), TOLERANCE);

        for(size_t r=0; r<refFrames.size(); ++r)
        {
          compareBodyNodeFkToJacobian(
                bn, refFrames[r], bn->getLocalCOM(), TOLERANCE);
        }

        compareBodyNodeFkToJacobian(
              bn, Frame::World(), randomVector<3>(10), TOLERANCE);
        compareBodyNodeFkToJacobian(bn, bn, randomVector<3>(10), TOLERANCE);

        for(size_t r=0; r<refFrames.size(); ++r)
        {
          compareBodyNodeFkToJacobian(
                bn, refFrames[r], randomVector<3>(10), TOLERANCE);
        }
      }
    }
  }
}

//==============================================================================
void DynamicsTest::testFiniteDifference(const std::string& _fileName)
{
  using namespace std;
  using namespace Eigen;
  using namespace dart;
  using namespace math;
  using namespace dynamics;
  using namespace simulation;
  using namespace utils;

  //----------------------------- Settings -------------------------------------
  const double TOLERANCE = 1.0e-2;
#ifndef NDEBUG  // Debug mode
  int nRandomItr = 2;
#else
  int nRandomItr = 10;
#endif
  double qLB   = -0.5 * DART_PI;
  double qUB   =  0.5 * DART_PI;
  double dqLB  = -0.5 * DART_PI;
  double dqUB  =  0.5 * DART_PI;
  double ddqLB = -0.5 * DART_PI;
  double ddqUB =  0.5 * DART_PI;
  Vector3d gravity(0.0, -9.81, 0.0);
  double timeStep = 1.0e-6;

  // load skeleton
  WorldPtr world = SkelParser::readWorld(_fileName);
  assert(world != NULL);
  world->setGravity(gravity);
  world->setTimeStep(timeStep);

  //------------------------------ Tests ---------------------------------------
  for (size_t i = 0; i < world->getNumSkeletons(); ++i)
  {
    SkeletonPtr skeleton = world->getSkeleton(i);
    assert(skeleton != NULL);
    int dof = skeleton->getNumDofs();

    for (int j = 0; j < nRandomItr; ++j)
    {
      // Generate a random state and ddq
      VectorXd q   = VectorXd(dof);
      VectorXd dq  = VectorXd(dof);
      VectorXd ddq = VectorXd(dof);
      for (int k = 0; k < dof; ++k)
      {
        q[k]   = math::random(qLB,   qUB);
        dq[k]  = math::random(dqLB,  dqUB);
        ddq[k] = math::random(ddqLB, ddqUB);

//        q[k]   = 0.0;
//        dq[k]  = 0.0;
//        ddq[k] = 0.0;
      }

      VectorXd qNext  =  q +  dq * timeStep;
      VectorXd dqNext = dq + ddq * timeStep;

      // For each body node
      for (size_t k = 0; k < skeleton->getNumBodyNodes(); ++k)
      {
        BodyNode* bn = skeleton->getBodyNode(k);

        // Calculation of velocities and Jacobian at k-th time step
        skeleton->setPositions(q);
        skeleton->setVelocities(dq);
        skeleton->setAccelerations(ddq);

        Vector3d BodyLinVel1 = bn->getLinearVelocity(Frame::World(), bn);
        Vector3d BodyAngVel1 = bn->getAngularVelocity(Frame::World(), bn);
        Vector3d WorldLinVel1 = bn->getLinearVelocity();
        Vector3d WorldAngVel1 = bn->getAngularVelocity();
        // Isometry3d T1    = bn->getTransform();

        // Get accelerations and time derivatives of Jacobians at k-th time step
        Vector3d BodyLinAcc1 = bn->getSpatialAcceleration().tail<3>();
        Vector3d BodyAngAcc1 = bn->getSpatialAcceleration().head<3>();
        Vector3d WorldLinAcc1 = bn->getLinearAcceleration();
        Vector3d WorldAngAcc1 = bn->getAngularAcceleration();

        // Calculation of velocities and Jacobian at (k+1)-th time step
        skeleton->setPositions(qNext);
        skeleton->setVelocities(dqNext);
        skeleton->setAccelerations(ddq);

        Vector3d BodyLinVel2 = bn->getLinearVelocity(Frame::World(), bn);
        Vector3d BodyAngVel2 = bn->getAngularVelocity(Frame::World(), bn);
        Vector3d WorldLinVel2 = bn->getLinearVelocity();
        Vector3d WorldAngVel2 = bn->getAngularVelocity();
        // Isometry3d T2    = bn->getTransform();

        // Get accelerations and time derivatives of Jacobians at k-th time step
        Vector3d BodyLinAcc2 = bn->getSpatialAcceleration().tail<3>();
        Vector3d BodyAngAcc2 = bn->getSpatialAcceleration().head<3>();
        Vector3d WorldLinAcc2 = bn->getLinearAcceleration();
        Vector3d WorldAngAcc2 = bn->getAngularAcceleration();

        // Calculation of approximated accelerations
        Vector3d BodyLinAccApprox   = (BodyLinVel2  - BodyLinVel1)  / timeStep;
        Vector3d BodyAngAccApprox   = (BodyAngVel2  - BodyAngVel1)  / timeStep;
        Vector3d WorldLinAccApprox  = (WorldLinVel2 - WorldLinVel1) / timeStep;
        Vector3d WorldAngAccApprox  = (WorldAngVel2 - WorldAngVel1) / timeStep;

        // Comparing two velocities
        EXPECT_TRUE(equals(BodyLinAcc1,   BodyLinAccApprox,   TOLERANCE));
        EXPECT_TRUE(equals(BodyAngAcc1,   BodyAngAccApprox,   TOLERANCE));
        EXPECT_TRUE(equals(BodyLinAcc2,   BodyLinAccApprox,   TOLERANCE));
        EXPECT_TRUE(equals(BodyAngAcc2,   BodyAngAccApprox,   TOLERANCE));
        EXPECT_TRUE(equals(WorldLinAcc1,  WorldLinAccApprox,  TOLERANCE));
        EXPECT_TRUE(equals(WorldAngAcc1,  WorldAngAccApprox,  TOLERANCE));
        EXPECT_TRUE(equals(WorldLinAcc2,  WorldLinAccApprox,  TOLERANCE));
        EXPECT_TRUE(equals(WorldAngAcc2,  WorldAngAccApprox,  TOLERANCE));

        // Debugging code
        if (!equals(BodyLinAcc1, BodyLinAccApprox, TOLERANCE))
        {
          cout << "BodyLinAcc1     :" << BodyLinAcc1.transpose()      << endl;
          cout << "BodyLinAccApprox:" << BodyLinAccApprox.transpose() << endl;
        }
        if (!equals(BodyAngAcc1, BodyAngAccApprox, TOLERANCE))
        {
          cout << "BodyAngAcc1     :" << BodyAngAcc1.transpose()      << endl;
          cout << "BodyAngAccApprox:" << BodyAngAccApprox.transpose() << endl;
        }
        if (!equals(BodyLinAcc2, BodyLinAccApprox, TOLERANCE))
        {
          cout << "BodyLinAcc2     :" << BodyLinAcc2.transpose()      << endl;
          cout << "BodyLinAccApprox:" << BodyLinAccApprox.transpose() << endl;
        }
        if (!equals(BodyAngAcc2, BodyAngAccApprox, TOLERANCE))
        {
          cout << "BodyAngAcc2     :" << BodyAngAcc2.transpose()      << endl;
          cout << "BodyAngAccApprox:" << BodyAngAccApprox.transpose() << endl;
        }
        if (!equals(WorldLinAcc1, WorldLinAccApprox, TOLERANCE))
        {
          cout << "WorldLinAcc1     :" << WorldLinAcc1.transpose()      << endl;
          cout << "WorldLinAccApprox:" << WorldLinAccApprox.transpose() << endl;
        }
        if (!equals(WorldAngAcc1, WorldAngAccApprox, TOLERANCE))
        {
          cout << "WorldAngAcc1     :" << WorldAngAcc1.transpose()      << endl;
          cout << "WorldAngAccApprox:" << WorldAngAccApprox.transpose() << endl;
        }
        if (!equals(WorldLinAcc2, WorldLinAccApprox, TOLERANCE))
        {
          cout << "WorldLinAcc2     :" << WorldLinAcc2.transpose()      << endl;
          cout << "WorldLinAccApprox:" << WorldLinAccApprox.transpose() << endl;
        }
        if (!equals(WorldAngAcc2, WorldAngAccApprox, TOLERANCE))
        {
          cout << "WorldAngAcc2     :" << WorldAngAcc2.transpose()      << endl;
          cout << "WorldAngAccApprox:" << WorldAngAccApprox.transpose() << endl;
        }
      }
    }
  }
}

//==============================================================================
void DynamicsTest::compareEquationsOfMotion(const std::string& _fileName)
{
  using namespace std;
  using namespace Eigen;
  using namespace dart;
  using namespace math;
  using namespace dynamics;
  using namespace simulation;
  using namespace utils;

  //---------------------------- Settings --------------------------------------
  // Number of random state tests for each skeletons
#ifndef NDEBUG  // Debug mode
  size_t nRandomItr = 2;
#else
  size_t nRandomItr = 100;
#endif

  // Lower and upper bound of configuration for system
  double lb = -1.0 * DART_PI;
  double ub =  1.0 * DART_PI;

  // Lower and upper bound of joint damping and stiffness
  double lbD =  0.0;
  double ubD = 10.0;
  double lbK =  0.0;
  double ubK = 10.0;

  simulation::WorldPtr myWorld;

  //----------------------------- Tests ----------------------------------------
  // Check whether multiplication of mass matrix and its inverse is identity
  // matrix.
  myWorld = utils::SkelParser::readWorld(_fileName);
  EXPECT_TRUE(myWorld != nullptr);

  for (size_t i = 0; i < myWorld->getNumSkeletons(); ++i)
  {
    dynamics::SkeletonPtr skel = myWorld->getSkeleton(i);

    size_t dof = skel->getNumDofs();
//    int nBodyNodes = skel->getNumBodyNodes();

    if (dof == 0)
    {
      dtmsg << "Skeleton [" << skel->getName() << "] is skipped since it has "
            << "0 DOF." << endl;
      continue;
    }

    for (size_t j = 0; j < nRandomItr; ++j)
    {
      // Random joint stiffness and damping coefficient
      for (size_t k = 0; k < skel->getNumBodyNodes(); ++k)
      {
        BodyNode* body     = skel->getBodyNode(k);
        Joint*    joint    = body->getParentJoint();
        size_t    localDof = joint->getNumDofs();

        for (size_t l = 0; l < localDof; ++l)
        {
          joint->setDampingCoefficient(l, random(lbD,  ubD));
          joint->setSpringStiffness   (l, random(lbK,  ubK));

          double lbRP = joint->getPositionLowerLimit(l);
          double ubRP = joint->getPositionUpperLimit(l);
          if (lbRP < -DART_PI)
            lbRP = -DART_PI;
          if (ubRP > DART_PI)
            ubRP = DART_PI;
          joint->setRestPosition      (l, random(lbRP, ubRP));
        }
      }

      // Set random states
      VectorXd x = skel->getState();
      for (int k = 0; k < x.size(); ++k)
        x[k] = random(lb, ub);
      skel->setState(x);

      //------------------------ Mass Matrix Test ----------------------------
      // Get matrices
      MatrixXd M      = skel->getMassMatrix();
      MatrixXd M2     = getMassMatrix(skel);
      MatrixXd InvM   = skel->getInvMassMatrix();
      MatrixXd M_InvM = M * InvM;
      MatrixXd InvM_M = InvM * M;

      MatrixXd AugM         = skel->getAugMassMatrix();
      MatrixXd AugM2        = getAugMassMatrix(skel);
      MatrixXd InvAugM      = skel->getInvAugMassMatrix();
      MatrixXd AugM_InvAugM = AugM * InvAugM;
      MatrixXd InvAugM_AugM = InvAugM * AugM;

      MatrixXd I        = MatrixXd::Identity(dof, dof);

      bool failure = false;

      // Check if the number of generalized coordinates and dimension of mass
      // matrix are same.
      EXPECT_EQ(M.rows(), (int)dof);
      EXPECT_EQ(M.cols(), (int)dof);

      // Check mass matrix
      EXPECT_TRUE(equals(M, M2, 1e-6));
      if (!equals(M, M2, 1e-6))
      {
        cout << "M :" << endl << M  << endl << endl;
        cout << "M2:" << endl << M2 << endl << endl;
        failure = true;
      }

      // Check augmented mass matrix
      EXPECT_TRUE(equals(AugM, AugM2, 1e-6));
      if (!equals(AugM, AugM2, 1e-6))
      {
        cout << "AugM :" << endl << AugM  << endl << endl;
        cout << "AugM2:" << endl << AugM2 << endl << endl;
        failure = true;
      }

      // Check if both of (M * InvM) and (InvM * M) are identity.
      EXPECT_TRUE(equals(M_InvM, I, 1e-6));
      if (!equals(M_InvM, I, 1e-6))
      {
        cout << "InvM  :" << endl << InvM << endl << endl;
        failure = true;
      }
      EXPECT_TRUE(equals(InvM_M, I, 1e-6));
      if (!equals(InvM_M, I, 1e-6))
      {
        cout << "InvM_M:" << endl << InvM_M << endl << endl;
        failure = true;
      }

      // Check if both of (M * InvM) and (InvM * M) are identity.
      EXPECT_TRUE(equals(AugM_InvAugM, I, 1e-6));
      if (!equals(AugM_InvAugM, I, 1e-6))
      {
        cout << "AugM_InvAugM  :" << endl << AugM_InvAugM << endl << endl;
        failure = true;
      }
      EXPECT_TRUE(equals(InvAugM_AugM, I, 1e-6));
      if (!equals(InvAugM_AugM, I, 1e-6))
      {
        cout << "InvAugM_AugM:" << endl << InvAugM_AugM << endl << endl;
      }

      //------- Coriolis Force Vector and Combined Force Vector Tests --------
      // Get C1, Coriolis force vector using recursive method
      VectorXd C = skel->getCoriolisForces();
      VectorXd Cg = skel->getCoriolisAndGravityForces();

      // Get C2, Coriolis force vector using inverse dynamics algorithm
      Vector3d oldGravity = skel->getGravity();
      VectorXd oldTau     = skel->getForces();
      VectorXd oldDdq     = skel->getAccelerations();
      // TODO(JS): Save external forces of body nodes

      skel->clearInternalForces();
      skel->clearExternalForces();
      skel->setAccelerations(VectorXd::Zero(dof));

      EXPECT_TRUE(skel->getForces() == VectorXd::Zero(dof));
      EXPECT_TRUE(skel->getExternalForces() == VectorXd::Zero(dof));
      EXPECT_TRUE(skel->getAccelerations() == VectorXd::Zero(dof));

      skel->setGravity(Vector3d::Zero());
      EXPECT_TRUE(skel->getGravity() == Vector3d::Zero());
      skel->computeInverseDynamics(false, false);
      VectorXd C2 = skel->getForces();

      skel->setGravity(oldGravity);
      EXPECT_TRUE(skel->getGravity() == oldGravity);
      skel->computeInverseDynamics(false, false);
      VectorXd Cg2 = skel->getForces();

      EXPECT_TRUE(equals(C, C2, 1e-6));
      if (!equals(C, C2, 1e-6))
      {
        cout << "C :" << C.transpose()  << endl;
        cout << "C2:" << C2.transpose() << endl;
        failure = true;
      }

      EXPECT_TRUE(equals(Cg, Cg2, 1e-6));
      if (!equals(Cg, Cg2, 1e-6))
      {
        cout << "Cg :" << Cg.transpose()  << endl;
        cout << "Cg2:" << Cg2.transpose() << endl;
        failure = true;
      }

      skel->setForces(oldTau);
      skel->setAccelerations(oldDdq);
      // TODO(JS): Restore external forces of body nodes

      //------------------- Combined Force Vector Test -----------------------
      // TODO(JS): Not implemented yet.

      //---------------------- Damping Force Test ----------------------------
      // TODO(JS): Not implemented yet.

      //--------------------- External Force Test ----------------------------
      // TODO(JS): Not implemented yet.

      if(failure)
      {
        std::cout << "Failure occurred in the World of file: " << _fileName
                  << "\nWith Skeleton named: " << skel->getName() << "\n\n";
      }
    }
  }
}

//==============================================================================
void compareCOMJacobianToFk(const SkeletonPtr skel,
                            const Frame* refFrame,
                            double tolerance)
{
  VectorXd dq = skel->getVelocities();
  VectorXd ddq = skel->getAccelerations();

  Vector6d comSpatialVelFk =
      skel->getCOMSpatialVelocity(Frame::World(), refFrame);
  Vector6d comSpatialAccFk =
      skel->getCOMSpatialAcceleration(Frame::World(), refFrame);

  math::Jacobian comSpatialJac = skel->getCOMJacobian(refFrame);
  math::Jacobian comSpatialJacDeriv =
      skel->getCOMJacobianSpatialDeriv(refFrame);

  Vector6d comSpatialVelJac = comSpatialJac * dq;
  Vector6d comSpatialAccJac = comSpatialJac*ddq + comSpatialJacDeriv*dq;

  bool spatialVelEqual = equals(comSpatialVelFk, comSpatialVelJac, tolerance);
  EXPECT_TRUE( spatialVelEqual );
  if(!spatialVelEqual)
    printComparisonError("COM spatial velocity", skel->getName(),
                         refFrame->getName(), comSpatialVelFk, comSpatialVelJac);

  bool spatialAccEqual = equals(comSpatialAccFk, comSpatialAccJac, tolerance);
  EXPECT_TRUE( spatialAccEqual );
  if(!spatialAccEqual)
    printComparisonError("COM spatial acceleration", skel->getName(),
                         refFrame->getName(), comSpatialAccFk, comSpatialAccJac);

  Vector3d comLinearVelFk =
      skel->getCOMLinearVelocity(Frame::World(), refFrame);
  Vector3d comLinearAccFk =
      skel->getCOMLinearAcceleration(Frame::World(), refFrame);

  math::LinearJacobian comLinearJac = skel->getCOMLinearJacobian(refFrame);
  math::LinearJacobian comLinearJacDeriv =
      skel->getCOMLinearJacobianDeriv(refFrame);

  Vector3d comLinearVelJac = comLinearJac * dq;
  Vector3d comLinearAccJac = comLinearJac*ddq + comLinearJacDeriv*dq;

  bool linearVelEqual = equals(comLinearVelFk, comLinearVelJac);
  EXPECT_TRUE( linearVelEqual );
  if(!linearVelEqual)
    printComparisonError("COM linear velocity", skel->getName(),
                         refFrame->getName(), comLinearVelFk, comLinearVelJac);

  bool linearAccEqual = equals(comLinearAccFk, comLinearAccJac);
  EXPECT_TRUE( linearAccEqual );
  if(!linearAccEqual)
    printComparisonError("COM linear acceleration", skel->getName(),
                         refFrame->getName(), comLinearAccFk, comLinearAccJac);
}

//==============================================================================
void DynamicsTest::testCenterOfMass(const std::string& _fileName)
{
  using namespace std;
  using namespace Eigen;
  using namespace dart;
  using namespace math;
  using namespace dynamics;
  using namespace simulation;
  using namespace utils;

  //---------------------------- Settings --------------------------------------
  // Number of random state tests for each skeletons
#ifndef NDEBUG  // Debug mode
  size_t nRandomItr = 2;
#else
  size_t nRandomItr = 100;
#endif

  // Lower and upper bound of configuration for system
  double lb = -1.5 * DART_PI;
  double ub =  1.5 * DART_PI;

  // Lower and upper bound of joint damping and stiffness
  double lbD =  0.0;
  double ubD = 10.0;
  double lbK =  0.0;
  double ubK = 10.0;

  simulation::WorldPtr myWorld;

  //----------------------------- Tests ----------------------------------------
  // Check whether multiplication of mass matrix and its inverse is identity
  // matrix.
  myWorld = utils::SkelParser::readWorld(_fileName);
  EXPECT_TRUE(myWorld != nullptr);

  for (size_t i = 0; i < myWorld->getNumSkeletons(); ++i)
  {
    dynamics::SkeletonPtr skeleton = myWorld->getSkeleton(i);

    size_t dof = skeleton->getNumDofs();
    if (dof == 0)
    {
      dtmsg << "Skeleton [" << skeleton->getName() << "] is skipped since it "
            << "has 0 DOF." << endl;
      continue;
    }

    for (size_t j = 0; j < nRandomItr; ++j)
    {
      // For the second half of the tests, scramble up the Skeleton
      if(j > ceil(nRandomItr/2))
      {
        SkeletonPtr copy = skeleton->clone();
        size_t maxNode = skeleton->getNumBodyNodes()-1;
        BodyNode* bn1 = skeleton->getBodyNode(ceil(math::random(0, maxNode)));
        BodyNode* bn2 = skeleton->getBodyNode(ceil(math::random(0, maxNode)));

        if(bn1 != bn2)
        {
          BodyNode* child = bn1->descendsFrom(bn2)? bn1 : bn2;
          BodyNode* parent = child == bn1? bn2 : bn1;

          child->moveTo(parent);
        }

        EXPECT_TRUE(skeleton->getNumBodyNodes() == copy->getNumBodyNodes());
        EXPECT_TRUE(skeleton->getNumDofs() == copy->getNumDofs());
      }

      // Random joint stiffness and damping coefficient
      for (size_t k = 0; k < skeleton->getNumBodyNodes(); ++k)
      {
        BodyNode* body     = skeleton->getBodyNode(k);
        Joint*    joint    = body->getParentJoint();
        int       localDof = joint->getNumDofs();

        for (int l = 0; l < localDof; ++l)
        {
          joint->setDampingCoefficient(l, random(lbD,  ubD));
          joint->setSpringStiffness   (l, random(lbK,  ubK));

          double lbRP = joint->getPositionLowerLimit(l);
          double ubRP = joint->getPositionUpperLimit(l);
          if (lbRP < -DART_PI)
            lbRP = -DART_PI;
          if (ubRP > DART_PI)
            ubRP = DART_PI;
          joint->setRestPosition      (l, random(lbRP, ubRP));
        }
      }

      // Set random states
      VectorXd q   = VectorXd(dof);
      VectorXd dq  = VectorXd(dof);
      VectorXd ddq = VectorXd(dof);
      for (size_t k = 0; k < dof; ++k)
      {
        q[k]   = math::random(lb, ub);
        dq[k]  = math::random(lb, ub);
        ddq[k] = math::random(lb, ub);
      }
      skeleton->setPositions(q);
      skeleton->setVelocities(dq);
      skeleton->setAccelerations(ddq);

      randomizeRefFrames();

      compareCOMJacobianToFk(skeleton, Frame::World(), 1e-6);

      for(size_t r=0; r<refFrames.size(); ++r)
        compareCOMJacobianToFk(skeleton, refFrames[r], 1e-6);
    }
  }
}

//==============================================================================
void compareCOMAccelerationToGravity(SkeletonPtr skel,
                                     const Eigen::Vector3d& gravity,
                                     double tolerance)
{
  const size_t numFrames = 1e+2;
  skel->setGravity(gravity);

  for (size_t i = 0; i < numFrames; ++i)
  {
    skel->computeForwardDynamics();

    Vector3d comLinearAccFk = skel->getCOMLinearAcceleration();

    bool comLinearAccFkEqual = equals(gravity, comLinearAccFk, tolerance);
    EXPECT_TRUE(comLinearAccFkEqual);
    if (!comLinearAccFkEqual)
    {
      printComparisonError("COM linear acceleration", skel->getName(),
                           Frame::World()->getName(), gravity, comLinearAccFk);
    }

    VectorXd dq  = skel->getVelocities();
    VectorXd ddq = skel->getAccelerations();
    math::LinearJacobian comLinearJac      = skel->getCOMLinearJacobian();
    math::LinearJacobian comLinearJacDeriv = skel->getCOMLinearJacobianDeriv();
    Vector3d comLinearAccJac = comLinearJac * ddq + comLinearJacDeriv * dq;

    bool comLinearAccJacEqual = equals(gravity, comLinearAccJac,tolerance);
    EXPECT_TRUE(comLinearAccJacEqual);
    if (!comLinearAccJacEqual)
    {
      printComparisonError("COM linear acceleration", skel->getName(),
                           Frame::World()->getName(), gravity, comLinearAccJac);
    }
  }

}

//==============================================================================
void DynamicsTest::testCenterOfMassFreeFall(const std::string& _fileName)
{
  using namespace std;
  using namespace Eigen;
  using namespace dart;
  using namespace math;
  using namespace dynamics;
  using namespace simulation;
  using namespace utils;

  //---------------------------- Settings --------------------------------------
  // Number of random state tests for each skeletons
#ifndef NDEBUG // Debug mode
  size_t nRandomItr = 2;
#else
  size_t nRandomItr = 10;
#endif // ------- Debug mode

  // Lower and upper bound of configuration for system
  double lb = -1.5 * DART_PI;
  double ub =  1.5 * DART_PI;

  // Lower and upper bound of joint damping and stiffness
  double lbD =  0.0;
  double ubD = 10.0;
  double lbK =  0.0;
  double ubK = 10.0;

  simulation::WorldPtr myWorld;
  std::vector<Vector3d> gravities(4);
  gravities[0] = Vector3d::Zero();
  gravities[1] = Vector3d(-9.81, 0, 0);
  gravities[2] = Vector3d(0, -9.81, 0);
  gravities[3] = Vector3d(0, 0, -9.81);

  //----------------------------- Tests ----------------------------------------
  // Check whether multiplication of mass matrix and its inverse is identity
  // matrix.
  myWorld = utils::SkelParser::readWorld(_fileName);
  EXPECT_TRUE(myWorld != nullptr);

  for (size_t i = 0; i < myWorld->getNumSkeletons(); ++i)
  {
    auto skel          = myWorld->getSkeleton(i);
    auto rootJoint     = skel->getJoint(0);
    auto rootFreeJoint = dynamic_cast<dynamics::FreeJoint*>(rootJoint);

    auto dof = skel->getNumDofs();

    if (nullptr == rootFreeJoint || !skel->isMobile() || 0 == dof)
    {
#ifdef BUILD_TYPE_DEBUG
      dtmsg << "Skipping COM free fall test for Skeleton [" << skel->getName()
            << "] since the Skeleton doesn't have FreeJoint at the root body "
            << " or immobile." << endl;
#endif
      continue;
    }
    else
    {
      rootFreeJoint->setActuatorType(dynamics::Joint::PASSIVE);
    }

    // Make sure the damping and spring forces are zero for the root FreeJoint.
    for (size_t l = 0; l < rootJoint->getNumDofs(); ++l)
    {
      rootJoint->setDampingCoefficient(l, 0.0);
      rootJoint->setSpringStiffness(l, 0.0);
      rootJoint->setRestPosition(l, 0.0);
    }

    for (size_t j = 0; j < nRandomItr; ++j)
    {
      // Random joint stiffness and damping coefficient
      for (size_t k = 1; k < skel->getNumBodyNodes(); ++k)
      {
        auto body     = skel->getBodyNode(k);
        auto joint    = body->getParentJoint();
        auto localDof = joint->getNumDofs();

        for (size_t l = 0; l < localDof; ++l)
        {
          joint->setDampingCoefficient(l, random(lbD,  ubD));
          joint->setSpringStiffness(l, random(lbK,  ubK));

          double lbRP = joint->getPositionLowerLimit(l);
          double ubRP = joint->getPositionUpperLimit(l);
          if (lbRP < -DART_PI)
            lbRP = -DART_PI;
          if (ubRP > DART_PI)
            ubRP = DART_PI;
          joint->setRestPosition(l, random(lbRP, ubRP));
        }
      }

      // Set random states
      VectorXd q  = VectorXd(dof);
      VectorXd dq = VectorXd(dof);
      for (size_t k = 0; k < dof; ++k)
      {
        q[k]   = math::random(lb, ub);
        dq[k]  = math::random(lb, ub);
      }
      VectorXd ddq = VectorXd::Zero(dof);
      skel->setPositions(q);
      skel->setVelocities(dq);
      skel->setAccelerations(ddq);

      for (const auto& gravity : gravities)
        compareCOMAccelerationToGravity(skel, gravity, 1e-6);
    }
  }
}

//==============================================================================
void DynamicsTest::testConstraintImpulse(const std::string& _fileName)
{
  using namespace std;
  using namespace Eigen;
  using namespace dart;
  using namespace math;
  using namespace dynamics;
  using namespace simulation;
  using namespace utils;

  //---------------------------- Settings --------------------------------------
  // Number of random state tests for each skeletons
#ifndef NDEBUG  // Debug mode
  size_t nRandomItr = 1;
#else
  size_t nRandomItr = 1;
#endif

  // Lower and upper bound of configuration for system
//  double lb = -1.5 * DART_PI;
//  double ub =  1.5 * DART_PI;

  simulation::WorldPtr myWorld;

  //----------------------------- Tests ----------------------------------------
  // Check whether multiplication of mass matrix and its inverse is identity
  // matrix.
  myWorld = utils::SkelParser::readWorld(_fileName);
  EXPECT_TRUE(myWorld != nullptr);

  for (size_t i = 0; i < myWorld->getNumSkeletons(); ++i)
  {
    dynamics::SkeletonPtr skel = myWorld->getSkeleton(i);

    size_t dof            = skel->getNumDofs();
//    int nBodyNodes     = skel->getNumBodyNodes();

    if (dof == 0 || !skel->isMobile())
    {
      dtdbg << "Skeleton [" << skel->getName() << "] is skipped since it has "
            << "0 DOF or is immobile." << endl;
      continue;
    }

    for (size_t j = 0; j < nRandomItr; ++j)
    {
      // Set random configurations
      for (size_t k = 0; k < skel->getNumBodyNodes(); ++k)
      {
        BodyNode* body     = skel->getBodyNode(k);
        Joint*    joint    = body->getParentJoint();
        int       localDof = joint->getNumDofs();

        for (int l = 0; l < localDof; ++l)
        {
          double lbRP = joint->getPositionLowerLimit(l);
          double ubRP = joint->getPositionUpperLimit(l);
          if (lbRP < -DART_PI)
            lbRP = -DART_PI;
          if (ubRP > DART_PI)
            ubRP = DART_PI;
          joint->setPosition(l, random(lbRP, ubRP));
        }

        // Set constraint impulse on each body
        skel->clearConstraintImpulses();
        Eigen::Vector6d impulseOnBody = Eigen::Vector6d::Random();
        body->setConstraintImpulse(impulseOnBody);

        // Get constraint force vector
        Eigen::VectorXd constraintVector1 = skel->getConstraintForces();

        // Get constraint force vector by using Jacobian of skeleon
        Eigen::MatrixXd bodyJacobian = body->getJacobian();
        Eigen::VectorXd constraintVector2 = bodyJacobian.transpose()
                                            * impulseOnBody
                                            / skel->getTimeStep();

        size_t index = 0;
        for (size_t l = 0; l < dof; ++l)
        {
          if (constraintVector1(l) == 0.0)
            continue;

          EXPECT_NEAR(constraintVector1(l), constraintVector2(index), 1e-6);
          index++;
        }
        assert(static_cast<size_t>(bodyJacobian.cols()) == index);
      }
    }
  }
}

//==============================================================================
void DynamicsTest::testImpulseBasedDynamics(const std::string& _fileName)
{
  using namespace std;
  using namespace Eigen;
  using namespace dart;
  using namespace math;
  using namespace dynamics;
  using namespace simulation;
  using namespace utils;

  //---------------------------- Settings --------------------------------------
  // Number of random state tests for each skeletons
#ifndef NDEBUG  // Debug mode
  size_t nRandomItr = 1;
#else
  size_t nRandomItr = 100;
#endif

  // Lower and upper bound of configuration for system
  double lb = -1.5 * DART_PI;
  double ub =  1.5 * DART_PI;

  simulation::WorldPtr myWorld;

  //----------------------------- Tests ----------------------------------------
  // Check whether multiplication of mass matrix and its inverse is identity
  // matrix.
  myWorld = utils::SkelParser::readWorld(_fileName);
  EXPECT_TRUE(myWorld != nullptr);

  for (size_t i = 0; i < myWorld->getNumSkeletons(); ++i)
  {
    dynamics::SkeletonPtr skel = myWorld->getSkeleton(i);

    int dof            = skel->getNumDofs();
//    int nBodyNodes     = skel->getNumBodyNodes();

    if (dof == 0 || !skel->isMobile())
    {
      dtdbg << "Skeleton [" << skel->getName() << "] is skipped since it has "
            << "0 DOF or is immobile." << endl;
      continue;
    }

    for (size_t j = 0; j < nRandomItr; ++j)
    {
      // Set random configurations
      for (size_t k = 0; k < skel->getNumBodyNodes(); ++k)
      {
        BodyNode* body     = skel->getBodyNode(k);
        Joint*    joint    = body->getParentJoint();
        int       localDof = joint->getNumDofs();

        for (int l = 0; l < localDof; ++l)
        {
          double lbRP = joint->getPositionLowerLimit(l);
          double ubRP = joint->getPositionUpperLimit(l);
          if (lbRP < -DART_PI)
            lbRP = -DART_PI;
          if (ubRP > DART_PI)
            ubRP = DART_PI;
          joint->setPosition(l, random(lbRP, ubRP));
        }
      }
//      skel->computeForwardKinematics();
//      skel->setPositions(VectorXd::Zero(dof));

      // TODO(JS): Just clear what should be
      skel->clearExternalForces();
      skel->clearConstraintImpulses();

      // Set random impulses
      VectorXd impulses = VectorXd::Zero(dof);
      for (int k = 0; k < impulses.size(); ++k)
        impulses[k] = random(lb, ub);
      skel->setJointConstraintImpulses(impulses);

      // Compute impulse-based forward dynamics
      skel->computeImpulseForwardDynamics();

      // Compare resultant velocity change and invM * impulses
      VectorXd deltaVel1 = skel->getVelocityChanges();
      MatrixXd invM = skel->getInvMassMatrix();
      VectorXd deltaVel2 = invM * impulses;

      EXPECT_TRUE(equals(deltaVel1, deltaVel2, 1e-5));
      if (!equals(deltaVel1, deltaVel2, 1e-5))
      {
        cout << "deltaVel1: " << deltaVel1.transpose()  << endl;
        cout << "deltaVel2: " << deltaVel2.transpose() << endl;
        cout << "error: " << (deltaVel1-deltaVel2).norm() << endl;
      }
    }
  }
}

//==============================================================================
TEST_F(DynamicsTest, testJacobians)
{
  for (size_t i = 0; i < getList().size(); ++i)
  {
#ifndef NDEBUG
    dtdbg << getList()[i] << std::endl;
#endif
    testJacobians(getList()[i]);
  }
}

//==============================================================================
TEST_F(DynamicsTest, testFiniteDifference)
{
  for (size_t i = 0; i < getList().size(); ++i)
  {
#ifndef NDEBUG
    dtdbg << getList()[i] << std::endl;
#endif
    testFiniteDifference(getList()[i]);
  }
}

//==============================================================================
TEST_F(DynamicsTest, compareEquationsOfMotion)
{
  for (size_t i = 0; i < getList().size(); ++i)
  {
    ////////////////////////////////////////////////////////////////////////////
    // TODO(JS): Following skel files, which contain euler joints couldn't
    //           pass EQUATIONS_OF_MOTION, are disabled.
    std::string skelFileName = getList()[i];
    if (skelFileName == DART_DATA_PATH"skel/test/double_pendulum_euler_joint.skel"
        || skelFileName == DART_DATA_PATH"skel/test/chainwhipa.skel"
        || skelFileName == DART_DATA_PATH"skel/test/serial_chain_eulerxyz_joint.skel"
        || skelFileName == DART_DATA_PATH"skel/test/simple_tree_structure_euler_joint.skel"
        || skelFileName == DART_DATA_PATH"skel/test/tree_structure_euler_joint.skel"
        || skelFileName == DART_DATA_PATH"skel/fullbody1.skel")
    {
        continue;
    }
    ////////////////////////////////////////////////////////////////////////////

#ifndef NDEBUG
    dtdbg << getList()[i] << std::endl;
#endif
    compareEquationsOfMotion(getList()[i]);
  }
}

//==============================================================================
TEST_F(DynamicsTest, testCenterOfMass)
{
  for (size_t i = 0; i < getList().size(); ++i)
  {
#ifndef NDEBUG
    dtdbg << getList()[i] << std::endl;
#endif
    testCenterOfMass(getList()[i]);
  }
}

//==============================================================================
TEST_F(DynamicsTest, testCenterOfMassFreeFall)
{
  for (size_t i = 0; i < getList().size(); ++i)
  {
#ifndef NDEBUG
    dtdbg << getList()[i] << std::endl;
#endif
    testCenterOfMassFreeFall(getList()[i]);
  }
}

//==============================================================================
TEST_F(DynamicsTest, testConstraintImpulse)
{
  for (size_t i = 0; i < getList().size(); ++i)
  {
#ifndef NDEBUG
    dtdbg << getList()[i] << std::endl;
#endif
    testConstraintImpulse(getList()[i]);
  }
}

//==============================================================================
TEST_F(DynamicsTest, testImpulseBasedDynamics)
{
  for (size_t i = 0; i < getList().size(); ++i)
  {
#ifndef NDEBUG
    dtdbg << getList()[i] << std::endl;
#endif
    testImpulseBasedDynamics(getList()[i]);
  }
}

//==============================================================================
TEST_F(DynamicsTest, HybridDynamics)
{
  const double tol       = 1e-9;
  const double timeStep  = 1e-3;
#ifndef NDEBUG // Debug mode
  const size_t numFrames = 50;  // 0.05 secs
#else
  const size_t numFrames = 5e+3;  // 5 secs
#endif // ------- Debug mode

  // Load world and skeleton
  WorldPtr world = utils::SkelParser::readWorld(
                   DART_DATA_PATH"/skel/test/hybrid_dynamics_test.skel");
  world->setTimeStep(timeStep);
  EXPECT_TRUE(world != NULL);
  EXPECT_NEAR(world->getTimeStep(), timeStep, tol);

  SkeletonPtr skel = world->getSkeleton("skeleton 1");
  EXPECT_TRUE(skel != NULL);
  EXPECT_NEAR(skel->getTimeStep(), timeStep, tol);

  const size_t numDofs = skel->getNumDofs();

  // Zero initial states
  Eigen::VectorXd q0  = Eigen::VectorXd::Zero(numDofs);
  Eigen::VectorXd dq0 = Eigen::VectorXd::Zero(numDofs);

  // Initialize the skeleton with the zero initial states
  skel->setPositions(q0);
  skel->setVelocities(dq0);
  skel->computeForwardKinematics(true, true, true);
  EXPECT_TRUE(equals(skel->getPositions(), q0));
  EXPECT_TRUE(equals(skel->getVelocities(), dq0));

  // Make sure all the joint actuator types
  EXPECT_EQ(skel->getJoint(0)->getActuatorType(), Joint::FORCE);
  EXPECT_EQ(skel->getJoint(1)->getActuatorType(), Joint::ACCELERATION);
  EXPECT_EQ(skel->getJoint(2)->getActuatorType(), Joint::VELOCITY);
  EXPECT_EQ(skel->getJoint(3)->getActuatorType(), Joint::ACCELERATION);
  EXPECT_EQ(skel->getJoint(4)->getActuatorType(), Joint::VELOCITY);

  // Prepare command for each joint types per simulation steps
  Eigen::MatrixXd command = Eigen::MatrixXd::Zero(numFrames, numDofs);
  Eigen::VectorXd amp = Eigen::VectorXd::Zero(numDofs);
  for (size_t i = 0; i < numDofs; ++i)
    amp[i] = math::random(-1.5, 1.5);
  for (size_t i = 0; i < numFrames; ++i)
  {
    for (size_t j = 0; j < numDofs; ++j)
      command(i,j) = amp[j] * std::sin(i * timeStep);
  }

  // Record joint forces for joint[1~4]
  Eigen::MatrixXd forces  = Eigen::MatrixXd::Zero(numFrames, numDofs);
  for (size_t i = 0; i < numFrames; ++i)
  {
    skel->setCommands(command.row(i));

    world->step(false);

    forces.row(i) = skel->getForces();

    EXPECT_NEAR(command(i,0), skel->getForce(0),        tol);
    EXPECT_NEAR(command(i,1), skel->getAcceleration(1), tol);
    EXPECT_NEAR(command(i,2), skel->getVelocity(2),     tol);
    EXPECT_NEAR(command(i,3), skel->getAcceleration(3), tol);
    EXPECT_NEAR(command(i,4), skel->getVelocity(4),     tol);
  }

  // Restore the skeleton to the initial state
  skel->setPositions(q0);
  skel->setVelocities(dq0);
  skel->computeForwardKinematics(true, true, true);
  EXPECT_TRUE(equals(skel->getPositions(), q0));
  EXPECT_TRUE(equals(skel->getVelocities(), dq0));

  // Change all the actuator types to force
  skel->getJoint(0)->setActuatorType(Joint::FORCE);
  skel->getJoint(1)->setActuatorType(Joint::FORCE);
  skel->getJoint(2)->setActuatorType(Joint::FORCE);
  skel->getJoint(3)->setActuatorType(Joint::FORCE);
  skel->getJoint(4)->setActuatorType(Joint::FORCE);
  EXPECT_EQ(skel->getJoint(0)->getActuatorType(), Joint::FORCE);
  EXPECT_EQ(skel->getJoint(1)->getActuatorType(), Joint::FORCE);
  EXPECT_EQ(skel->getJoint(2)->getActuatorType(), Joint::FORCE);
  EXPECT_EQ(skel->getJoint(3)->getActuatorType(), Joint::FORCE);
  EXPECT_EQ(skel->getJoint(4)->getActuatorType(), Joint::FORCE);

  // Test if the skeleton moves as the command with the joint forces
  Eigen::MatrixXd output = Eigen::MatrixXd::Zero(numFrames, numDofs);
  for (size_t i = 0; i < numFrames; ++i)
  {
    skel->setCommands(forces.row(i));

    world->step(false);

    output(i,0) = skel->getJoint(0)->getForce(0);
    output(i,1) = skel->getJoint(1)->getAcceleration(0);
    output(i,2) = skel->getJoint(2)->getVelocity(0);
    output(i,3) = skel->getJoint(3)->getAcceleration(0);
    output(i,4) = skel->getJoint(4)->getVelocity(0);

    EXPECT_NEAR(command(i,0), output(i,0), tol);
    EXPECT_NEAR(command(i,1), output(i,1), tol);
    EXPECT_NEAR(command(i,2), output(i,2), tol);
    EXPECT_NEAR(command(i,3), output(i,3), tol);
    EXPECT_NEAR(command(i,4), output(i,4), tol);
  }
}

//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
