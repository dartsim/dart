/*
 * Copyright (c) 2011-2019, The DART development contributors
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

#include <iostream>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "TestHelpers.hpp"

#include "dart/common/Console.hpp"
#include "dart/math/Geometry.hpp"
#include "dart/math/Helpers.hpp"
#include "dart/math/Random.hpp"
#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/dynamics/SimpleFrame.hpp"
#include "dart/simulation/World.hpp"
#include "dart/io/SkelParser.hpp"

using namespace Eigen;
using namespace dart;

//==============================================================================
class DynamicsTest : public ::testing::Test
{
public:
  // Get Skel file URI to test.
  const std::vector<common::Uri>& getList() const;

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
  void testJacobians(const common::Uri& uri);

  // Compare velocities and accelerations with actual vaules and approximates
  // using finite differece method.
  void testFiniteDifferenceGeneralizedCoordinates(const common::Uri& uri);

  // Compare spatial velocities computed by forward kinematics and finite
  // difference.
  void testFiniteDifferenceBodyNodeVelocity(const common::Uri& uri);

  // Compare accelerations computed by recursive method, Jacobian, and finite
  // difference.
  void testFiniteDifferenceBodyNodeAcceleration(const common::Uri& uri);

  // Test if the recursive forward kinematics algorithm computes
  // transformations, spatial velocities, and spatial accelerations correctly.
  void testForwardKinematics(const common::Uri& uri);

  // Compare dynamics terms in equations of motion such as mass matrix, mass
  // inverse matrix, Coriolis force vector, gravity force vector, and external
  // force vector.
  void compareEquationsOfMotion(const common::Uri& uri);

  // Test skeleton's COM and its related quantities.
  void testCenterOfMass(const common::Uri& uri);

  // Test if the com acceleration is equal to the gravity
  void testCenterOfMassFreeFall(const common::Uri& uri);

  //
  void testConstraintImpulse(const common::Uri& uri);

  // Test impulse based dynamics
  void testImpulseBasedDynamics(const common::Uri& uri);

protected:
  // Sets up the test fixture.
  void SetUp() override;

  // Skel file list.
  std::vector<common::Uri> fileList;

  std::vector<SimpleFrame*> refFrames;
};

//==============================================================================
void DynamicsTest::SetUp()
{
  // Create a list of skel files to test with
  fileList.push_back("dart://sample/skel/test/chainwhipa.skel");
  fileList.push_back("dart://sample/skel/test/single_pendulum.skel");
  fileList.push_back("dart://sample/skel/test/single_pendulum_euler_joint.skel");
  fileList.push_back("dart://sample/skel/test/single_pendulum_ball_joint.skel");
  fileList.push_back("dart://sample/skel/test/double_pendulum.skel");
  fileList.push_back("dart://sample/skel/test/double_pendulum_euler_joint.skel");
  fileList.push_back("dart://sample/skel/test/double_pendulum_ball_joint.skel");
  fileList.push_back("dart://sample/skel/test/serial_chain_revolute_joint.skel");
  fileList.push_back("dart://sample/skel/test/serial_chain_eulerxyz_joint.skel");
  fileList.push_back("dart://sample/skel/test/serial_chain_ball_joint.skel");
  fileList.push_back("dart://sample/skel/test/serial_chain_ball_joint_20.skel");
  fileList.push_back("dart://sample/skel/test/serial_chain_ball_joint_40.skel");
  fileList.push_back("dart://sample/skel/test/simple_tree_structure.skel");
  fileList.push_back("dart://sample/skel/test/simple_tree_structure_euler_joint.skel");
  fileList.push_back("dart://sample/skel/test/simple_tree_structure_ball_joint.skel");
  fileList.push_back("dart://sample/skel/test/tree_structure.skel");
  fileList.push_back("dart://sample/skel/test/tree_structure_euler_joint.skel");
  fileList.push_back("dart://sample/skel/test/tree_structure_ball_joint.skel");
  fileList.push_back("dart://sample/skel/fullbody1.skel");

  // Create a list of reference frames to use during tests
  refFrames.push_back(new SimpleFrame(Frame::World(), "refFrame1"));
  refFrames.push_back(new SimpleFrame(refFrames.back(), "refFrame2"));
  refFrames.push_back(new SimpleFrame(refFrames.back(), "refFrame3"));
  refFrames.push_back(new SimpleFrame(refFrames.back(), "refFrame4"));
  refFrames.push_back(new SimpleFrame(Frame::World(), "refFrame5"));
  refFrames.push_back(new SimpleFrame(refFrames.back(), "refFrame6"));
}

//==============================================================================
const std::vector<common::Uri>& DynamicsTest::getList() const
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
  for(std::size_t i=0; i<refFrames.size(); ++i)
  {
    SimpleFrame* F = refFrames[i];

    Eigen::Vector3d p = Random::uniform<Eigen::Vector3d>(-100, 100);
    Eigen::Vector3d theta = Random::uniform<Eigen::Vector3d>(-2*M_PI, 2*M_PI);

    Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
    tf.translate(p);
    tf.linear() = math::eulerXYZToMatrix(theta);

    F->setRelativeTransform(tf);
    F->setRelativeSpatialVelocity(Random::uniform<Eigen::Vector6d>(-100, 100));
    F->setRelativeSpatialAcceleration(Random::uniform<Eigen::Vector6d>(-100, 100));
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

  for (std::size_t i = 0; i < _skel->getNumBodyNodes(); ++i)
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
  for (std::size_t i = 0; i < _skel->getNumBodyNodes(); ++i)
  {
    dynamics::BodyNode* body  = _skel->getBodyNode(i);
    dynamics::Joint*    joint = body->getParentJoint();

    EXPECT_TRUE(body  != nullptr);
    EXPECT_TRUE(joint != nullptr);

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

  const std::vector<std::size_t>& coords = bn->getDependentGenCoordIndices();
  VectorXd dqSeg  = skel->getVelocities(coords);
  VectorXd ddqSeg = skel->getAccelerations(coords);

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

  const std::vector<std::size_t>& coords = bn->getDependentGenCoordIndices();
  VectorXd dqSeg  = skel->getVelocities(coords);
  VectorXd ddqSeg = skel->getAccelerations(coords);

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
template <typename T>
void printComparisonError(const std::string& _comparison,
                          const std::string& _nameBN,
                          const std::string& _nameBNRelativeTo,
                          const std::string& _frame,
                          const T& fk,
                          const T& jac)
{
  std::cout << "Disagreement between FK and relative Jacobian results for "
            << _comparison << " of '" << _nameBN << "' relative to '"
            << _nameBNRelativeTo
            << "' with a reference Frame of '" << _frame << "'\n"
            << "FK:  " << fk.transpose() << "\n"
            << "Jac: " << jac.transpose() << "\n";
}

//==============================================================================
void compareBodyNodeFkToJacobianRelative(const JacobianNode* bn,
                                         const JacobianNode* relativeTo,
                                         const Frame* refFrame,
                                         double tolerance)
{
  using math::Jacobian;
  using math::LinearJacobian;
  using math::AngularJacobian;

  assert(bn->getSkeleton() == relativeTo->getSkeleton());
  auto skel = bn->getSkeleton();

  VectorXd dq  = skel->getVelocities();
  VectorXd ddq = skel->getAccelerations();

  //-- Spatial Jacobian tests --------------------------------------------------

  Vector6d SpatialVelFk = bn->getSpatialVelocity(relativeTo, refFrame);
  Vector6d SpatialAccFk = bn->getSpatialAcceleration(relativeTo, refFrame);

  Jacobian SpatialJac
      = skel->getJacobian(bn, relativeTo, refFrame);
  Jacobian SpatialJacDeriv
      = skel->getJacobianSpatialDeriv(bn, relativeTo, refFrame);

  Vector6d SpatialVelJac = SpatialJac * dq;
  Vector6d SpatialAccJac = SpatialJac * ddq + SpatialJacDeriv * dq;

  bool spatialVelEqual = equals(SpatialVelFk, SpatialVelJac, tolerance);
  EXPECT_TRUE(spatialVelEqual);
  if (!spatialVelEqual)
  {
    printComparisonError("spatial velocity",
                         bn->getName(), relativeTo->getName(),
                         refFrame->getName(),
                         SpatialVelFk,  SpatialVelJac);
  }

  bool spatialAccEqual = equals(SpatialAccFk, SpatialAccJac, tolerance);
  EXPECT_TRUE(spatialAccEqual);
  if (!spatialAccEqual)
  {
    printComparisonError("spatial acceleration",
                         bn->getName(), relativeTo->getName(),
                         refFrame->getName(),
                         SpatialAccFk,  SpatialAccJac);
  }

  //-- Linear Jacobian tests --------------------------------------------------

  Vector3d LinearVelFk = bn->getLinearVelocity(relativeTo, refFrame);

  LinearJacobian LinearJac
      = skel->getLinearJacobian(bn, relativeTo, refFrame);

  Vector3d LinearVelJac = LinearJac * dq;

  bool linearVelEqual = equals(LinearVelFk, LinearVelJac, tolerance);
  EXPECT_TRUE(linearVelEqual);
  if (!linearVelEqual)
  {
    printComparisonError("linear velocity",
                         bn->getName(), relativeTo->getName(),
                         refFrame->getName(),
                         LinearVelFk,  LinearVelJac);
  }

  //-- Angular Jacobian tests --------------------------------------------------

  Vector3d AngularVelFk = bn->getAngularVelocity(relativeTo, refFrame);

  AngularJacobian AngularJac
      = skel->getAngularJacobian(bn, relativeTo, refFrame);

  Vector3d AngularVelJac = AngularJac * dq;

  bool angularVelEqual = equals(AngularVelFk, AngularVelJac, tolerance);
  EXPECT_TRUE(angularVelEqual);
  if (!angularVelEqual)
  {
    printComparisonError("angular velocity",
                         bn->getName(), relativeTo->getName(),
                         refFrame->getName(),
                         AngularVelFk,  AngularVelJac);
  }
}

//==============================================================================
void compareBodyNodeFkToJacobianRelative(const JacobianNode* bn,
                                         const Eigen::Vector3d& _offset,
                                         const JacobianNode* relativeTo,
                                         const Frame* refFrame,
                                         double tolerance)
{
  using math::Jacobian;
  using math::LinearJacobian;
  using math::AngularJacobian;

  assert(bn->getSkeleton() == relativeTo->getSkeleton());
  auto skel = bn->getSkeleton();

  VectorXd dq  = skel->getVelocities();
  VectorXd ddq = skel->getAccelerations();

  //-- Spatial Jacobian tests --------------------------------------------------

  Vector6d SpatialVelFk
      = bn->getSpatialVelocity(_offset, relativeTo, refFrame);
  Vector6d SpatialAccFk
      = bn->getSpatialAcceleration(_offset, relativeTo, refFrame);

  Jacobian SpatialJac
      = skel->getJacobian(bn, _offset, relativeTo, refFrame);
  Jacobian SpatialJacDeriv
      = skel->getJacobianSpatialDeriv(bn, _offset, relativeTo, refFrame);

  Vector6d SpatialVelJac = SpatialJac * dq;
  Vector6d SpatialAccJac = SpatialJac * ddq + SpatialJacDeriv * dq;

  bool spatialVelEqual = equals(SpatialVelFk, SpatialVelJac, tolerance);
  EXPECT_TRUE(spatialVelEqual);
  if (!spatialVelEqual)
  {
    printComparisonError("spatial velocity w/ offset",
                         bn->getName(), relativeTo->getName(),
                         refFrame->getName(),
                         SpatialVelFk,  SpatialVelJac);
  }

  bool spatialAccEqual = equals(SpatialAccFk, SpatialAccJac, tolerance);
  EXPECT_TRUE(spatialAccEqual);
  if (!spatialAccEqual)
  {
    printComparisonError("spatial acceleration w/ offset",
                         bn->getName(), relativeTo->getName(),
                         refFrame->getName(),
                         SpatialAccFk,  SpatialAccJac);
  }

  //-- Linear Jacobian tests --------------------------------------------------

  Vector3d LinearVelFk = bn->getLinearVelocity(_offset, relativeTo, refFrame);

  LinearJacobian LinearJac
      = skel->getLinearJacobian(bn, _offset, relativeTo, refFrame);

  Vector3d LinearVelJac = LinearJac * dq;

  bool linearVelEqual = equals(LinearVelFk, LinearVelJac, tolerance);
  EXPECT_TRUE(linearVelEqual);
  if (!linearVelEqual)
  {
    printComparisonError("linear velocity w/ offset",
                         bn->getName(), relativeTo->getName(),
                         refFrame->getName(),
                         LinearVelFk,  LinearVelJac);
  }
}

//==============================================================================
void DynamicsTest::testJacobians(const common::Uri& uri)
{
  using namespace std;
  using namespace Eigen;
  using namespace dart;
  using namespace math;
  using namespace dynamics;
  using namespace simulation;
  using namespace io;

  //----------------------------- Settings -------------------------------------
  const double TOLERANCE = 1.0e-6;
#ifndef NDEBUG  // Debug mode
  int nTestItr = 2;
#else
  int nTestItr = 5;
#endif
  double qLB  = -0.5 * constantsd::pi();
  double qUB  =  0.5 * constantsd::pi();
  double dqLB = -0.5 * constantsd::pi();
  double dqUB =  0.5 * constantsd::pi();
  double ddqLB = -0.5 * constantsd::pi();
  double ddqUB =  0.5 * constantsd::pi();
  Vector3d gravity(0.0, -9.81, 0.0);

  // load skeleton
  WorldPtr world = SkelParser::readWorld(uri);
  assert(world != nullptr);
  world->setGravity(gravity);

  //------------------------------ Tests ---------------------------------------
  for (std::size_t i = 0; i < world->getNumSkeletons(); ++i)
  {
    SkeletonPtr skeleton = world->getSkeleton(i);
    assert(skeleton != nullptr);
    int dof = skeleton->getNumDofs();

    for (int j = 0; j < nTestItr; ++j)
    {
      // For the second half of the tests, scramble up the Skeleton
      if(j > std::ceil(nTestItr/2))
      {
        SkeletonPtr copy = skeleton->cloneSkeleton();
        std::size_t maxNode = skeleton->getNumBodyNodes()-1;
        BodyNode* bn1 = skeleton->getBodyNode(math::Random::uniform<std::size_t>(0, maxNode));
        BodyNode* bn2 = skeleton->getBodyNode(math::Random::uniform<std::size_t>(0, maxNode));

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
        q[k]   = math::Random::uniform(qLB,   qUB);
        dq[k]  = math::Random::uniform(dqLB,  dqUB);
        ddq[k] = math::Random::uniform(ddqLB, ddqUB);
      }
      skeleton->setPositions(q);
      skeleton->setVelocities(dq);
      skeleton->setAccelerations(ddq);

      randomizeRefFrames();

      // For each body node
      for (std::size_t k = 0; k < skeleton->getNumBodyNodes(); ++k)
      {
        const BodyNode* bn = skeleton->getBodyNode(k);

        // Compare results using the World reference Frame
        compareBodyNodeFkToJacobian(bn, Frame::World(), TOLERANCE);
        compareBodyNodeFkToJacobian(
              bn, Frame::World(), bn->getLocalCOM(), TOLERANCE);
        compareBodyNodeFkToJacobian(
              bn, Frame::World(), Random::uniform<Eigen::Vector3d>(-10, 10), TOLERANCE);

        // Compare results using this BodyNode's own reference Frame
        compareBodyNodeFkToJacobian(bn, bn, TOLERANCE);
        compareBodyNodeFkToJacobian(bn, bn, bn->getLocalCOM(), TOLERANCE);
        compareBodyNodeFkToJacobian(bn, bn, Random::uniform<Eigen::Vector3d>(-10, 10), TOLERANCE);

        // Compare results using the randomized reference Frames
        for(std::size_t r=0; r<refFrames.size(); ++r)
        {
          compareBodyNodeFkToJacobian(bn, refFrames[r], TOLERANCE);
          compareBodyNodeFkToJacobian(
              bn, refFrames[r], bn->getLocalCOM(), TOLERANCE);
          compareBodyNodeFkToJacobian(
              bn, refFrames[r], Random::uniform<Eigen::Vector3d>(-10, 10), TOLERANCE);
        }

        // -- Relative Jacobian tests

        compareBodyNodeFkToJacobianRelative(bn, bn, Frame::World(), TOLERANCE);

#ifndef NDEBUG // Debug mode
        if (skeleton->getNumBodyNodes() == 0u)
          continue;

        for (std::size_t l = skeleton->getNumBodyNodes() - 1;
             l < skeleton->getNumBodyNodes(); ++l)
#else
        for (std::size_t l = 0; l < skeleton->getNumBodyNodes(); ++l)
#endif
        {
          const BodyNode* relativeTo = skeleton->getBodyNode(l);

          compareBodyNodeFkToJacobianRelative(
                bn, relativeTo, Frame::World(), TOLERANCE);
          compareBodyNodeFkToJacobianRelative(
                bn, bn->getLocalCOM(), relativeTo, Frame::World(), TOLERANCE);
          compareBodyNodeFkToJacobianRelative(
                bn, Random::uniform<Eigen::Vector3d>(-10, 10), relativeTo, Frame::World(), TOLERANCE);

          compareBodyNodeFkToJacobianRelative(
                bn, relativeTo, bn, TOLERANCE);
          compareBodyNodeFkToJacobianRelative(
                bn, bn->getLocalCOM(), relativeTo, bn, TOLERANCE);
          compareBodyNodeFkToJacobianRelative(
                bn, Random::uniform<Eigen::Vector3d>(-10, 10), relativeTo, bn, TOLERANCE);

          compareBodyNodeFkToJacobianRelative(
                bn, relativeTo, relativeTo, TOLERANCE);
          compareBodyNodeFkToJacobianRelative(
                bn, bn->getLocalCOM(), relativeTo, relativeTo, TOLERANCE);
          compareBodyNodeFkToJacobianRelative(
                bn, Random::uniform<Eigen::Vector3d>(-10, 10), relativeTo, relativeTo, TOLERANCE);

          for (std::size_t r = 0; r < refFrames.size(); ++r)
          {
            compareBodyNodeFkToJacobianRelative(
                  bn, relativeTo, refFrames[r], TOLERANCE);
            compareBodyNodeFkToJacobianRelative(
                  bn, bn->getLocalCOM(), relativeTo, refFrames[r], TOLERANCE);
            compareBodyNodeFkToJacobianRelative(
                  bn, Random::uniform<Eigen::Vector3d>(-10, 10), relativeTo, refFrames[r], TOLERANCE);
          }
        }
      }
    }
  }
}

//==============================================================================
void DynamicsTest::testFiniteDifferenceGeneralizedCoordinates(
    const common::Uri& uri)
{
  using namespace std;
  using namespace Eigen;
  using namespace dart;
  using namespace math;
  using namespace dynamics;
  using namespace simulation;
  using namespace io;

  //----------------------------- Settings -------------------------------------
#ifndef NDEBUG  // Debug mode
  int nRandomItr = 2;
#else
  int nRandomItr = 10;
#endif
  double qLB   = -0.5 * constantsd::pi();
  double qUB   =  0.5 * constantsd::pi();
  double dqLB  = -0.3 * constantsd::pi();
  double dqUB  =  0.3 * constantsd::pi();
  double ddqLB = -0.1 * constantsd::pi();
  double ddqUB =  0.1 * constantsd::pi();
  Vector3d gravity(0.0, -9.81, 0.0);
  double timeStep = 1e-3;
  double TOLERANCE = 5e-4;

  // load skeleton
  WorldPtr world = SkelParser::readWorld(uri);
  assert(world != nullptr);
  world->setGravity(gravity);
  world->setTimeStep(timeStep);

  //------------------------------ Tests ---------------------------------------
  for (std::size_t i = 0; i < world->getNumSkeletons(); ++i)
  {
    SkeletonPtr skeleton = world->getSkeleton(i);
    assert(skeleton != nullptr);
    int dof = skeleton->getNumDofs();

    for (int j = 0; j < nRandomItr; ++j)
    {
      // Generate a random state and ddq
      VectorXd q0   = VectorXd(dof);
      VectorXd dq0  = VectorXd(dof);
      VectorXd ddq0 = VectorXd(dof);
      for (int k = 0; k < dof; ++k)
      {
        q0[k]   = math::Random::uniform(qLB,   qUB);
        dq0[k]  = math::Random::uniform(dqLB,  dqUB);
        ddq0[k] = math::Random::uniform(ddqLB, ddqUB);
      }

      skeleton->setPositions(q0);
      skeleton->setVelocities(dq0);
      skeleton->setAccelerations(ddq0);

      skeleton->integratePositions(timeStep);
      VectorXd q1 = skeleton->getPositions();
      skeleton->integrateVelocities(timeStep);
      VectorXd dq1 = skeleton->getVelocities();

      skeleton->integratePositions(timeStep);
      VectorXd q2 = skeleton->getPositions();
      skeleton->integrateVelocities(timeStep);
      VectorXd dq2 = skeleton->getVelocities();

      VectorXd dq0FD = skeleton->getPositionDifferences(q1, q0) / timeStep;
      VectorXd dq1FD = skeleton->getPositionDifferences(q2, q1) / timeStep;
      VectorXd ddqFD1 = skeleton->getVelocityDifferences(dq1FD, dq0FD) / timeStep;
      VectorXd ddqFD2 = skeleton->getVelocityDifferences(dq2, dq1) / timeStep;

      EXPECT_TRUE(equals(dq0, dq0FD, TOLERANCE));
      EXPECT_TRUE(equals(dq1, dq1FD, TOLERANCE));
      EXPECT_TRUE(equals(ddq0, ddqFD1, TOLERANCE));
      EXPECT_TRUE(equals(ddq0, ddqFD2, TOLERANCE));

      if (!equals(dq0FD, dq0, TOLERANCE))
      {
        std::cout << "dq0  : " << dq0.transpose() << std::endl;
        std::cout << "dq0FD: " << dq0FD.transpose() << std::endl;
      }
      if (!equals(dq1, dq1FD, TOLERANCE))
      {
        std::cout << "dq1  : " << dq1.transpose() << std::endl;
        std::cout << "dq1FD: " << dq1FD.transpose() << std::endl;
      }
      if (!equals(ddq0, ddqFD1, TOLERANCE))
      {
        std::cout << "ddq0  : " << ddq0.transpose() << std::endl;
        std::cout << "ddqFD1: " << ddqFD1.transpose() << std::endl;
      }
      if (!equals(ddq0, ddqFD2, TOLERANCE))
      {
        std::cout << "ddq0  : " << ddq0.transpose() << std::endl;
        std::cout << "ddqFD2: " << ddqFD2.transpose() << std::endl;
      }
    }
  }
}

//==============================================================================
void DynamicsTest::testFiniteDifferenceBodyNodeVelocity(const common::Uri& uri)
{
  using namespace std;
  using namespace Eigen;
  using namespace dart;
  using namespace math;
  using namespace dynamics;
  using namespace simulation;
  using namespace io;

  //----------------------------- Settings -------------------------------------
#ifndef NDEBUG  // Debug mode
  int nRandomItr = 2;
  std::size_t numSteps = 1e+1;
#else
  int nRandomItr = 10;
  std::size_t numSteps = 1e+3;
#endif
  double qLB   = -0.5 * constantsd::pi();
  double qUB   =  0.5 * constantsd::pi();
  double dqLB  = -0.5 * constantsd::pi();
  double dqUB  =  0.5 * constantsd::pi();
  double ddqLB = -0.5 * constantsd::pi();
  double ddqUB =  0.5 * constantsd::pi();
  Vector3d gravity(0.0, -9.81, 0.0);
  double timeStep = 1.0e-6;
  const double tol = timeStep * 1e+2;

  // load skeleton
  WorldPtr world = SkelParser::readWorld(uri);
  assert(world != nullptr);
  world->setGravity(gravity);
  world->setTimeStep(timeStep);

  //------------------------------ Tests ---------------------------------------
  for (int i = 0; i < nRandomItr; ++i)
  {
    for (std::size_t j = 0; j < world->getNumSkeletons(); ++j)
    {
      SkeletonPtr skeleton = world->getSkeleton(j);
      EXPECT_NE(skeleton, nullptr);

      std::size_t dof       = skeleton->getNumDofs();
      std::size_t numBodies = skeleton->getNumBodyNodes();

      // Generate random states
      VectorXd   q = Random::uniform<Eigen::VectorXd>(dof,   qLB,   qUB);
      VectorXd  dq = Random::uniform<Eigen::VectorXd>(dof,  dqLB,  dqUB);
      VectorXd ddq = Random::uniform<Eigen::VectorXd>(dof, ddqLB, ddqUB);

      skeleton->setPositions(q);
      skeleton->setVelocities(dq);
      skeleton->setAccelerations(ddq);

      common::aligned_map<dynamics::BodyNodePtr, Eigen::Isometry3d> Tmap;
      for (auto k = 0u; k < numBodies; ++k)
      {
        auto body  = skeleton->getBodyNode(k);
        Tmap[body] = body->getTransform();
      }

      for (std::size_t k = 0; k < numSteps; ++k)
      {
        skeleton->integrateVelocities(skeleton->getTimeStep());
        skeleton->integratePositions(skeleton->getTimeStep());

        for (std::size_t l = 0; l < skeleton->getNumBodyNodes(); ++l)
        {
          BodyNodePtr body  = skeleton->getBodyNode(l);

          Isometry3d T1 = Tmap[body];
          Isometry3d T2 = body->getTransform();

          Vector6d V_diff = math::logMap(T1.inverse() * T2) / timeStep;
          Vector6d V_actual = body->getSpatialVelocity();

          bool checkSpatialVelocity = equals(V_diff, V_actual, tol);
          EXPECT_TRUE(checkSpatialVelocity);
          if (!checkSpatialVelocity)
          {
            std::cout << "[" << body->getName() << "]" << std::endl;
            std::cout << "V_diff  : "
                      << V_diff.transpose() << std::endl;
            std::cout << "V_actual  : "
                      << V_actual.transpose() << std::endl;
            std::cout << std::endl;
          }

          Tmap[body] = body->getTransform();
        }
      }
    }
  }
}

//==============================================================================
void DynamicsTest::testFiniteDifferenceBodyNodeAcceleration(
    const common::Uri& uri)
{
  using namespace std;
  using namespace Eigen;
  using namespace dart;
  using namespace math;
  using namespace dynamics;
  using namespace simulation;
  using namespace io;

  //----------------------------- Settings -------------------------------------
  const double TOLERANCE = 1.0e-2;
#ifndef NDEBUG  // Debug mode
  int nRandomItr = 2;
#else
  int nRandomItr = 10;
#endif
  double qLB   = -0.5 * constantsd::pi();
  double qUB   =  0.5 * constantsd::pi();
  double dqLB  = -0.5 * constantsd::pi();
  double dqUB  =  0.5 * constantsd::pi();
  double ddqLB = -0.5 * constantsd::pi();
  double ddqUB =  0.5 * constantsd::pi();
  Vector3d gravity(0.0, -9.81, 0.0);
  double timeStep = 1.0e-6;

  // load skeleton
  WorldPtr world = SkelParser::readWorld(uri);
  assert(world != nullptr);
  world->setGravity(gravity);
  world->setTimeStep(timeStep);

  //------------------------------ Tests ---------------------------------------
  for (std::size_t i = 0; i < world->getNumSkeletons(); ++i)
  {
    SkeletonPtr skeleton = world->getSkeleton(i);
    assert(skeleton != nullptr);
    int dof = skeleton->getNumDofs();

    for (int j = 0; j < nRandomItr; ++j)
    {
      // Generate a random state and ddq
      VectorXd q   = VectorXd(dof);
      VectorXd dq  = VectorXd(dof);
      VectorXd ddq = VectorXd(dof);
      for (int k = 0; k < dof; ++k)
      {
        q[k]   = math::Random::uniform(qLB,   qUB);
        dq[k]  = math::Random::uniform(dqLB,  dqUB);
        ddq[k] = math::Random::uniform(ddqLB, ddqUB);
      }

      // For each body node
      for (std::size_t k = 0; k < skeleton->getNumBodyNodes(); ++k)
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
        skeleton->integrateVelocities(skeleton->getTimeStep());
        skeleton->integratePositions(skeleton->getTimeStep());

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
void testForwardKinematicsSkeleton(const dynamics::SkeletonPtr& skel)
{
#ifndef NDEBUG  // Debug mode
  std::size_t nRandomItr = 1e+1;
  std::size_t numSteps = 1e+1;
#else
  std::size_t nRandomItr = 1e+2;
  std::size_t numSteps = 1e+2;
#endif
  double qLB   = -0.5 * constantsd::pi();
  double qUB   =  0.5 * constantsd::pi();
  double dqLB  = -0.3 * constantsd::pi();
  double dqUB  =  0.3 * constantsd::pi();
  double ddqLB = -0.1 * constantsd::pi();
  double ddqUB =  0.1 * constantsd::pi();
  double timeStep = 1e-6;

  EXPECT_NE(skel, nullptr);

  skel->setTimeStep(timeStep);

  auto dof       = skel->getNumDofs();
  auto numBodies = skel->getNumBodyNodes();

  Eigen::VectorXd q;
  Eigen::VectorXd dq;
  Eigen::VectorXd ddq;

  common::aligned_map<dynamics::BodyNodePtr, Eigen::Isometry3d>  Tmap;
  common::aligned_map<dynamics::BodyNodePtr, Eigen::Vector6d>    Vmap;
  common::aligned_map<dynamics::BodyNodePtr, Eigen::Vector6d>   dVmap;

  for (auto j = 0u; j < numBodies; ++j)
  {
    auto body  = skel->getBodyNode(j);

     Tmap[body] = Eigen::Isometry3d::Identity();
     Vmap[body] = Eigen::Vector6d::Zero();
    dVmap[body] = Eigen::Vector6d::Zero();
  }

  for (auto i = 0u; i < nRandomItr; ++i)
  {
    q   = Random::uniform<Eigen::VectorXd>(dof,   qLB,   qUB);
    dq  = Random::uniform<Eigen::VectorXd>(dof,  dqLB,  dqUB);
    ddq = Random::uniform<Eigen::VectorXd>(dof, ddqLB, ddqUB);

    skel->setPositions(q);
    skel->setVelocities(dq);
    skel->setAccelerations(ddq);

    for (auto j = 0u; j < numSteps; ++j)
    {
      for (auto k = 0u; k < numBodies; ++k)
      {
        auto body       = skel->getBodyNode(k);
        auto joint      = skel->getJoint(k);
        auto parentBody = body->getParentBodyNode();
        Eigen::MatrixXd S  = joint->getRelativeJacobian();
        Eigen::MatrixXd dS = joint->getRelativeJacobianTimeDeriv();
        Eigen::VectorXd jointQ   = joint->getPositions();
        Eigen::VectorXd jointDQ  = joint->getVelocities();
        Eigen::VectorXd jointDDQ = joint->getAccelerations();

        Eigen::Isometry3d relT  = body->getRelativeTransform();
        Eigen::Vector6d   relV  =  S * jointDQ;
        Eigen::Vector6d   relDV = dS * jointDQ + S * jointDDQ;

        if (parentBody)
        {
          Tmap[body] = Tmap[parentBody] * relT;
          Vmap[body] = math::AdInvT(relT,  Vmap[parentBody]) + relV;
          dVmap[body] = math::AdInvT(relT, dVmap[parentBody])
              + math::ad(Vmap[body], S * jointDQ)
              + relDV;
        }
        else
        {
          Tmap[body] = relT;
          Vmap[body] = relV;
          dVmap[body] = relDV;
        }

        bool checkT  = equals(body->getTransform().matrix(), Tmap[body].matrix());
        bool checkV  = equals(body->getSpatialVelocity(), Vmap[body]);
        bool checkDV = equals(body->getSpatialAcceleration(), dVmap[body]);

        EXPECT_TRUE(checkT);
        EXPECT_TRUE(checkV);
        EXPECT_TRUE(checkDV);

        if (!checkT)
        {
          std::cout << "[" << body->getName() << "]" << std::endl;
          std::cout << "actual T  : " << std::endl
                    << body->getTransform().matrix() << std::endl;
          std::cout << "expected T: " << std::endl
                    << Tmap[body].matrix() << std::endl;
          std::cout << std::endl;
        }

        if (!checkV)
        {
          std::cout << "[" << body->getName() << "]" << std::endl;
          std::cout << "actual V  : "
                    << body->getSpatialVelocity().transpose() << std::endl;
          std::cout << "expected V: "
                    << Vmap[body].transpose() << std::endl;
          std::cout << std::endl;
        }

        if (!checkDV)
        {
          std::cout << "[" << body->getName() << "]" << std::endl;
          std::cout << "actual DV  : "
                    << body->getSpatialAcceleration().transpose() << std::endl;
          std::cout << "expected DV: "
                    << dVmap[body].transpose() << std::endl;
          std::cout << std::endl;
        }
      }
    }
  }
}

//==============================================================================
void DynamicsTest::testForwardKinematics(const common::Uri& uri)
{
  auto world = io::SkelParser::readWorld(uri);
  EXPECT_TRUE(world != nullptr);

  auto numSkeletons = world->getNumSkeletons();
  for (auto i = 0u; i < numSkeletons; ++i)
  {
    auto skeleton  = world->getSkeleton(i);
    testForwardKinematicsSkeleton(skeleton);
  }
}

//==============================================================================
void DynamicsTest::compareEquationsOfMotion(const common::Uri& uri)
{
  using namespace std;
  using namespace Eigen;
  using namespace dart;
  using namespace math;
  using namespace dynamics;
  using namespace simulation;
  using namespace io;

  //---------------------------- Settings --------------------------------------
  // Number of random state tests for each skeletons
#ifndef NDEBUG  // Debug mode
  std::size_t nRandomItr = 2;
#else
  std::size_t nRandomItr = 100;
#endif

  // Lower and upper bound of configuration for system
  double lb = -1.0 * constantsd::pi();
  double ub =  1.0 * constantsd::pi();

  // Lower and upper bound of joint damping and stiffness
  double lbD =  0.0;
  double ubD = 10.0;
  double lbK =  0.0;
  double ubK = 10.0;

  simulation::WorldPtr myWorld;

  //----------------------------- Tests ----------------------------------------
  // Check whether multiplication of mass matrix and its inverse is identity
  // matrix.
  myWorld = io::SkelParser::readWorld(uri);
  EXPECT_TRUE(myWorld != nullptr);

  for (std::size_t i = 0; i < myWorld->getNumSkeletons(); ++i)
  {
    dynamics::SkeletonPtr skel = myWorld->getSkeleton(i);

    std::size_t dof = skel->getNumDofs();
//    int nBodyNodes = skel->getNumBodyNodes();

    if (dof == 0)
    {
      dtmsg << "Skeleton [" << skel->getName() << "] is skipped since it has "
            << "0 DOF." << endl;
      continue;
    }

    for (std::size_t j = 0; j < nRandomItr; ++j)
    {
      // Random joint stiffness and damping coefficient
      for (std::size_t k = 0; k < skel->getNumBodyNodes(); ++k)
      {
        BodyNode* body     = skel->getBodyNode(k);
        Joint*    joint    = body->getParentJoint();
        std::size_t    localDof = joint->getNumDofs();

        for (std::size_t l = 0; l < localDof; ++l)
        {
          joint->setDampingCoefficient(l, Random::uniform(lbD,  ubD));
          joint->setSpringStiffness   (l, Random::uniform(lbK,  ubK));

          double lbRP = joint->getPositionLowerLimit(l);
          double ubRP = joint->getPositionUpperLimit(l);
          if (lbRP < -constantsd::pi())
            lbRP = -constantsd::pi();
          if (ubRP > constantsd::pi())
            ubRP = constantsd::pi();
          joint->setRestPosition      (l, Random::uniform(lbRP, ubRP));
        }
      }

      // Set random states
      Skeleton::Configuration x = skel->getConfiguration(
            Skeleton::CONFIG_POSITIONS | Skeleton::CONFIG_VELOCITIES);
      for (auto k = 0u; k < skel->getNumDofs(); ++k)
      {
        x.mPositions[k] = Random::uniform(lb, ub);
        x.mVelocities[k] = Random::uniform(lb, ub);
      }
      skel->setConfiguration(x);

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
        std::cout << "Failure occurred in the World of file: " << uri.toString()
                  << "\nWith Skeleton named: " << skel->getName() << "\n\n";
      }
    }
  }
}

//==============================================================================
void compareCOMJacobianToFk(const SkeletonPtr& skel,
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
void DynamicsTest::testCenterOfMass(const common::Uri& uri)
{
  using namespace std;
  using namespace Eigen;
  using namespace dart;
  using namespace math;
  using namespace dynamics;
  using namespace simulation;
  using namespace io;

  //---------------------------- Settings --------------------------------------
  // Number of random state tests for each skeletons
#ifndef NDEBUG  // Debug mode
  std::size_t nRandomItr = 2;
#else
  std::size_t nRandomItr = 100;
#endif

  // Lower and upper bound of configuration for system
  double lb = -1.5 * constantsd::pi();
  double ub =  1.5 * constantsd::pi();

  // Lower and upper bound of joint damping and stiffness
  double lbD =  0.0;
  double ubD = 10.0;
  double lbK =  0.0;
  double ubK = 10.0;

  simulation::WorldPtr myWorld;

  //----------------------------- Tests ----------------------------------------
  // Check whether multiplication of mass matrix and its inverse is identity
  // matrix.
  myWorld = io::SkelParser::readWorld(uri);
  EXPECT_TRUE(myWorld != nullptr);

  for (std::size_t i = 0; i < myWorld->getNumSkeletons(); ++i)
  {
    dynamics::SkeletonPtr skeleton = myWorld->getSkeleton(i);

    std::size_t dof = skeleton->getNumDofs();
    if (dof == 0)
    {
      dtmsg << "Skeleton [" << skeleton->getName() << "] is skipped since it "
            << "has 0 DOF." << endl;
      continue;
    }

    for (std::size_t j = 0; j < nRandomItr; ++j)
    {
      // For the second half of the tests, scramble up the Skeleton
      if(j > ceil(nRandomItr/2))
      {
        SkeletonPtr copy = skeleton->cloneSkeleton();
        std::size_t maxNode = skeleton->getNumBodyNodes()-1;
        BodyNode* bn1 = skeleton->getBodyNode(math::Random::uniform<std::size_t>(0, maxNode));
        BodyNode* bn2 = skeleton->getBodyNode(math::Random::uniform<std::size_t>(0, maxNode));

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
      for (std::size_t k = 0; k < skeleton->getNumBodyNodes(); ++k)
      {
        BodyNode* body     = skeleton->getBodyNode(k);
        Joint*    joint    = body->getParentJoint();
        int       localDof = joint->getNumDofs();

        for (int l = 0; l < localDof; ++l)
        {
          joint->setDampingCoefficient(l, Random::uniform(lbD,  ubD));
          joint->setSpringStiffness   (l, Random::uniform(lbK,  ubK));

          double lbRP = joint->getPositionLowerLimit(l);
          double ubRP = joint->getPositionUpperLimit(l);
          if (lbRP < -constantsd::pi())
            lbRP = -constantsd::pi();
          if (ubRP > constantsd::pi())
            ubRP = constantsd::pi();
          joint->setRestPosition      (l, Random::uniform(lbRP, ubRP));
        }
      }

      // Set random states
      VectorXd q   = VectorXd(dof);
      VectorXd dq  = VectorXd(dof);
      VectorXd ddq = VectorXd(dof);
      for (std::size_t k = 0; k < dof; ++k)
      {
        q[k]   = math::Random::uniform(lb, ub);
        dq[k]  = math::Random::uniform(lb, ub);
        ddq[k] = math::Random::uniform(lb, ub);
      }
      skeleton->setPositions(q);
      skeleton->setVelocities(dq);
      skeleton->setAccelerations(ddq);

      randomizeRefFrames();

      compareCOMJacobianToFk(skeleton, Frame::World(), 1e-6);

      for(std::size_t r=0; r<refFrames.size(); ++r)
        compareCOMJacobianToFk(skeleton, refFrames[r], 1e-6);
    }
  }
}

//==============================================================================
void compareCOMAccelerationToGravity(SkeletonPtr skel,
                                     const Eigen::Vector3d& gravity,
                                     double tolerance)
{
  const std::size_t numFrames = 1e+2;
  skel->setGravity(gravity);

  for (std::size_t i = 0; i < numFrames; ++i)
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
void DynamicsTest::testCenterOfMassFreeFall(const common::Uri& uri)
{
  using namespace std;
  using namespace Eigen;
  using namespace dart;
  using namespace math;
  using namespace dynamics;
  using namespace simulation;
  using namespace io;

  //---------------------------- Settings --------------------------------------
  // Number of random state tests for each skeletons
#ifndef NDEBUG // Debug mode
  std::size_t nRandomItr = 2;
#else
  std::size_t nRandomItr = 10;
#endif // ------- Debug mode

  // Lower and upper bound of configuration for system
  double lb = -1.5 * constantsd::pi();
  double ub =  1.5 * constantsd::pi();

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
  myWorld = io::SkelParser::readWorld(uri);
  EXPECT_TRUE(myWorld != nullptr);

  for (std::size_t i = 0; i < myWorld->getNumSkeletons(); ++i)
  {
    auto skel          = myWorld->getSkeleton(i);
    auto rootJoint     = skel->getJoint(0);
    auto rootFreeJoint = dynamic_cast<dynamics::FreeJoint*>(rootJoint);

    auto dof = skel->getNumDofs();

    if (nullptr == rootFreeJoint || !skel->isMobile() || 0 == dof)
    {
#if BUILD_TYPE_DEBUG
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
    for (std::size_t l = 0; l < rootJoint->getNumDofs(); ++l)
    {
      rootJoint->setDampingCoefficient(l, 0.0);
      rootJoint->setSpringStiffness(l, 0.0);
      rootJoint->setRestPosition(l, 0.0);
    }

    for (std::size_t j = 0; j < nRandomItr; ++j)
    {
      // Random joint stiffness and damping coefficient
      for (std::size_t k = 1; k < skel->getNumBodyNodes(); ++k)
      {
        auto body     = skel->getBodyNode(k);
        auto joint    = body->getParentJoint();
        auto localDof = joint->getNumDofs();

        for (std::size_t l = 0; l < localDof; ++l)
        {
          joint->setDampingCoefficient(l, Random::uniform(lbD,  ubD));
          joint->setSpringStiffness(l, Random::uniform(lbK,  ubK));

          double lbRP = joint->getPositionLowerLimit(l);
          double ubRP = joint->getPositionUpperLimit(l);
          if (lbRP < -constantsd::pi())
            lbRP = -constantsd::pi();
          if (ubRP > constantsd::pi())
            ubRP = constantsd::pi();
          joint->setRestPosition(l, Random::uniform(lbRP, ubRP));
        }
      }

      // Set random states
      VectorXd q  = VectorXd(dof);
      VectorXd dq = VectorXd(dof);
      for (std::size_t k = 0; k < dof; ++k)
      {
        q[k]   = math::Random::uniform(lb, ub);
        dq[k]  = math::Random::uniform(lb, ub);
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
void DynamicsTest::testConstraintImpulse(const common::Uri& uri)
{
  using namespace std;
  using namespace Eigen;
  using namespace dart;
  using namespace math;
  using namespace dynamics;
  using namespace simulation;
  using namespace io;

  //---------------------------- Settings --------------------------------------
  // Number of random state tests for each skeletons
#ifndef NDEBUG  // Debug mode
  std::size_t nRandomItr = 1;
#else
  std::size_t nRandomItr = 1;
#endif

  // Lower and upper bound of configuration for system
//  double lb = -1.5 * constantsd::pi();
//  double ub =  1.5 * constantsd::pi();

  simulation::WorldPtr myWorld;

  //----------------------------- Tests ----------------------------------------
  // Check whether multiplication of mass matrix and its inverse is identity
  // matrix.
  myWorld = io::SkelParser::readWorld(uri);
  EXPECT_TRUE(myWorld != nullptr);

  for (std::size_t i = 0; i < myWorld->getNumSkeletons(); ++i)
  {
    dynamics::SkeletonPtr skel = myWorld->getSkeleton(i);

    std::size_t dof            = skel->getNumDofs();
//    int nBodyNodes     = skel->getNumBodyNodes();

    if (dof == 0 || !skel->isMobile())
    {
      dtdbg << "Skeleton [" << skel->getName() << "] is skipped since it has "
            << "0 DOF or is immobile." << endl;
      continue;
    }

    for (std::size_t j = 0; j < nRandomItr; ++j)
    {
      // Set random configurations
      for (std::size_t k = 0; k < skel->getNumBodyNodes(); ++k)
      {
        BodyNode* body     = skel->getBodyNode(k);
        Joint*    joint    = body->getParentJoint();
        int       localDof = joint->getNumDofs();

        for (int l = 0; l < localDof; ++l)
        {
          double lbRP = joint->getPositionLowerLimit(l);
          double ubRP = joint->getPositionUpperLimit(l);
          if (lbRP < -constantsd::pi())
            lbRP = -constantsd::pi();
          if (ubRP > constantsd::pi())
            ubRP = constantsd::pi();
          joint->setPosition(l, Random::uniform(lbRP, ubRP));
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

        std::size_t index = 0;
        for (std::size_t l = 0; l < dof; ++l)
        {
          if (constraintVector1(l) == 0.0)
            continue;

          EXPECT_NEAR(constraintVector1(l), constraintVector2(index), 1e-6);
          index++;
        }
        assert(static_cast<std::size_t>(bodyJacobian.cols()) == index);
      }
    }
  }
}

//==============================================================================
void DynamicsTest::testImpulseBasedDynamics(const common::Uri& uri)
{
  using namespace std;
  using namespace Eigen;
  using namespace dart;
  using namespace math;
  using namespace dynamics;
  using namespace simulation;
  using namespace io;

  //---------------------------- Settings --------------------------------------
  // Number of random state tests for each skeletons
#ifndef NDEBUG  // Debug mode
  std::size_t nRandomItr = 1;
#else
  std::size_t nRandomItr = 100;
#endif

  double TOLERANCE = 1e-3;

  // Lower and upper bound of configuration for system
  double lb = -1.5 * constantsd::pi();
  double ub =  1.5 * constantsd::pi();

  simulation::WorldPtr myWorld;

  //----------------------------- Tests ----------------------------------------
  // Check whether multiplication of mass matrix and its inverse is identity
  // matrix.
  myWorld = io::SkelParser::readWorld(uri);
  EXPECT_TRUE(myWorld != nullptr);

  for (std::size_t i = 0; i < myWorld->getNumSkeletons(); ++i)
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

    for (std::size_t j = 0; j < nRandomItr; ++j)
    {
      // Set random configurations
      for (std::size_t k = 0; k < skel->getNumBodyNodes(); ++k)
      {
        BodyNode* body     = skel->getBodyNode(k);
        Joint*    joint    = body->getParentJoint();
        int       localDof = joint->getNumDofs();

        for (int l = 0; l < localDof; ++l)
        {
          double lbRP = joint->getPositionLowerLimit(l);
          double ubRP = joint->getPositionUpperLimit(l);
          if (lbRP < -constantsd::pi())
            lbRP = -constantsd::pi();
          if (ubRP > constantsd::pi())
            ubRP = constantsd::pi();
          joint->setPosition(l, Random::uniform(lbRP, ubRP));
        }
      }
//      skel->setPositions(VectorXd::Zero(dof));

      // TODO(JS): Just clear what should be
      skel->clearExternalForces();
      skel->clearConstraintImpulses();

      // Set random impulses
      VectorXd impulses = VectorXd::Zero(dof);
      for (int k = 0; k < impulses.size(); ++k)
        impulses[k] = Random::uniform(lb, ub);
      skel->setJointConstraintImpulses(impulses);

      // Compute impulse-based forward dynamics
      skel->computeImpulseForwardDynamics();

      // Compare resultant velocity change and invM * impulses
      VectorXd deltaVel1 = skel->getVelocityChanges();
      MatrixXd invM = skel->getInvMassMatrix();
      VectorXd deltaVel2 = invM * impulses;

      EXPECT_TRUE(equals(deltaVel1, deltaVel2, TOLERANCE));
      if (!equals(deltaVel1, deltaVel2, TOLERANCE))
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
  for (std::size_t i = 0; i < getList().size(); ++i)
  {
#ifndef NDEBUG
    dtdbg << getList()[i].toString() << std::endl;
#endif
    testJacobians(getList()[i]);
  }
}

//==============================================================================
TEST_F(DynamicsTest, testFiniteDifference)
{
  for (std::size_t i = 0; i < getList().size(); ++i)
  {
#if BUILD_TYPE_DEBUG
    dtdbg << getList()[i].toString() << std::endl;
#endif
    testFiniteDifferenceGeneralizedCoordinates(getList()[i]);
    testFiniteDifferenceBodyNodeVelocity(getList()[i]);
    testFiniteDifferenceBodyNodeAcceleration(getList()[i]);
  }
}

//==============================================================================
TEST_F(DynamicsTest, testForwardKinematics)
{
  for (std::size_t i = 0; i < getList().size(); ++i)
  {
#ifndef NDEBUG
    dtdbg << getList()[i].toString() << std::endl;
#endif
    testForwardKinematics(getList()[i]);
  }
}

//==============================================================================
TEST_F(DynamicsTest, compareEquationsOfMotion)
{
  for (std::size_t i = 0; i < getList().size(); ++i)
  {
    ////////////////////////////////////////////////////////////////////////////
    // TODO(JS): Following skel files, which contain euler joints couldn't
    //           pass EQUATIONS_OF_MOTION, are disabled.
    const auto uri = getList()[i];
    if (uri.toString() == "dart://sample/skel/test/double_pendulum_euler_joint.skel"
        || uri.toString() == "dart://sample/skel/test/chainwhipa.skel"
        || uri.toString() == "dart://sample/skel/test/serial_chain_eulerxyz_joint.skel"
        || uri.toString() == "dart://sample/skel/test/simple_tree_structure_euler_joint.skel"
        || uri.toString() == "dart://sample/skel/test/tree_structure_euler_joint.skel"
        || uri.toString() == "dart://sample/skel/fullbody1.skel")
    {
        continue;
    }
    ////////////////////////////////////////////////////////////////////////////

#ifndef NDEBUG
    dtdbg << getList()[i].toString() << std::endl;
#endif
    compareEquationsOfMotion(getList()[i]);
  }
}

//==============================================================================
TEST_F(DynamicsTest, testCenterOfMass)
{
  for (std::size_t i = 0; i < getList().size(); ++i)
  {
#ifndef NDEBUG
    dtdbg << getList()[i].toString() << std::endl;
#endif
    testCenterOfMass(getList()[i]);
  }
}

//==============================================================================
TEST_F(DynamicsTest, testCenterOfMassFreeFall)
{
  for (std::size_t i = 0; i < getList().size(); ++i)
  {
#ifndef NDEBUG
    dtdbg << getList()[i].toString() << std::endl;
#endif
    testCenterOfMassFreeFall(getList()[i]);
  }
}

//==============================================================================
TEST_F(DynamicsTest, testConstraintImpulse)
{
  for (std::size_t i = 0; i < getList().size(); ++i)
  {
#ifndef NDEBUG
    dtdbg << getList()[i].toString() << std::endl;
#endif
    testConstraintImpulse(getList()[i]);
  }
}

//==============================================================================
TEST_F(DynamicsTest, testImpulseBasedDynamics)
{
  for (std::size_t i = 0; i < getList().size(); ++i)
  {
#ifndef NDEBUG
    dtdbg << getList()[i].toString() << std::endl;
#endif
    testImpulseBasedDynamics(getList()[i]);
  }
}

//==============================================================================
TEST_F(DynamicsTest, HybridDynamics)
{
  const double tol       = 1e-8;
  const double timeStep  = 1e-3;
#ifndef NDEBUG // Debug mode
  const std::size_t numFrames = 50;  // 0.05 secs
#else
  const std::size_t numFrames = 5e+3;  // 5 secs
#endif // ------- Debug mode

  // Load world and skeleton
  WorldPtr world = io::SkelParser::readWorld(
      "dart://sample/skel/test/hybrid_dynamics_test.skel");
  world->setTimeStep(timeStep);
  EXPECT_TRUE(world != nullptr);
  EXPECT_NEAR(world->getTimeStep(), timeStep, tol);

  SkeletonPtr skel = world->getSkeleton("skeleton 1");
  EXPECT_TRUE(skel != nullptr);
  EXPECT_NEAR(skel->getTimeStep(), timeStep, tol);

  const std::size_t numDofs = skel->getNumDofs();

  // Zero initial states
  Eigen::VectorXd q0  = Eigen::VectorXd::Zero(numDofs);
  Eigen::VectorXd dq0 = Eigen::VectorXd::Zero(numDofs);

  // Initialize the skeleton with the zero initial states
  skel->setPositions(q0);
  skel->setVelocities(dq0);
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
  for (std::size_t i = 0; i < numDofs; ++i)
    amp[i] = math::Random::uniform(-1.5, 1.5);
  for (std::size_t i = 0; i < numFrames; ++i)
  {
    for (std::size_t j = 0; j < numDofs; ++j)
      command(i,j) = amp[j] * std::sin(i * timeStep);
  }

  // Record joint forces for joint[1~4]
  Eigen::MatrixXd forces  = Eigen::MatrixXd::Zero(numFrames, numDofs);
  for (std::size_t i = 0; i < numFrames; ++i)
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
  for (std::size_t i = 0; i < numFrames; ++i)
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
