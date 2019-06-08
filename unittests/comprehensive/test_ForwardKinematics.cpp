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
#include <gtest/gtest.h>
#include "TestHelpers.hpp"

#include "dart/common/Uri.hpp"
#include "dart/utils/SkelParser.hpp"
#include "dart/utils/urdf/DartLoader.hpp"

using namespace dart;

//==============================================================================
class ForwardKinematicsTest : public ::testing::Test
{
public:
  // Get Skel file URI to test.
  const std::vector<common::Uri>& getList() const;

  // Get Skel file URI to test.
  const std::vector<common::Uri>& getListForDebug() const;

  // Get reference frames
  const std::vector<SimpleFrame*>& getRefFrames() const;

  // Randomize the properties of all the reference frames
  void randomizeRefFrames();

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

protected:
  // Sets up the test fixture.
  void SetUp() override;

  // Skel file list.
  std::vector<common::Uri> fileList;

  // Skel file list for debug mode
  std::vector<common::Uri> fileListForDebug;

  std::vector<SimpleFrame*> refFrames;
};

//==============================================================================
void ForwardKinematicsTest::SetUp()
{
  // Create a list of skel files to test with
  fileList.push_back("dart://sample/skel/test/chainwhipa.skel");
  fileList.push_back("dart://sample/skel/test/single_pendulum.skel");
  fileList.push_back(
      "dart://sample/skel/test/single_pendulum_euler_joint.skel");
  fileList.push_back("dart://sample/skel/test/single_pendulum_ball_joint.skel");
  fileList.push_back("dart://sample/skel/test/double_pendulum.skel");
  fileList.push_back(
      "dart://sample/skel/test/double_pendulum_euler_joint.skel");
  fileList.push_back("dart://sample/skel/test/double_pendulum_ball_joint.skel");
  fileList.push_back(
      "dart://sample/skel/test/serial_chain_revolute_joint.skel");
  fileList.push_back(
      "dart://sample/skel/test/serial_chain_eulerxyz_joint.skel");
  fileList.push_back("dart://sample/skel/test/serial_chain_ball_joint.skel");
  fileList.push_back("dart://sample/skel/test/serial_chain_ball_joint_20.skel");
  fileList.push_back("dart://sample/skel/test/serial_chain_ball_joint_40.skel");
  fileList.push_back("dart://sample/skel/test/simple_tree_structure.skel");
  fileList.push_back(
      "dart://sample/skel/test/simple_tree_structure_euler_joint.skel");
  fileList.push_back(
      "dart://sample/skel/test/simple_tree_structure_ball_joint.skel");
  fileList.push_back("dart://sample/skel/test/tree_structure.skel");
  fileList.push_back("dart://sample/skel/test/tree_structure_euler_joint.skel");
  fileList.push_back("dart://sample/skel/test/tree_structure_ball_joint.skel");
  fileList.push_back("dart://sample/skel/fullbody1.skel");

  fileListForDebug.push_back(
      "dart://sample/skel/test/serial_chain_revolute_joint.skel");
  fileListForDebug.push_back(
      "dart://sample/skel/test/serial_chain_eulerxyz_joint.skel");
  fileListForDebug.push_back(
      "dart://sample/skel/test/serial_chain_ball_joint.skel");
  fileListForDebug.push_back(
      "dart://sample/skel/test/simple_tree_structure.skel");
  fileListForDebug.push_back("dart://sample/skel/fullbody1.skel");

  // Create a list of reference frames to use during tests
  refFrames.push_back(new SimpleFrame(Frame::World(), "refFrame1"));
  refFrames.push_back(new SimpleFrame(refFrames.back(), "refFrame2"));
  refFrames.push_back(new SimpleFrame(refFrames.back(), "refFrame3"));
  refFrames.push_back(new SimpleFrame(refFrames.back(), "refFrame4"));
  refFrames.push_back(new SimpleFrame(Frame::World(), "refFrame5"));
  refFrames.push_back(new SimpleFrame(refFrames.back(), "refFrame6"));
}

//==============================================================================
const std::vector<common::Uri>& ForwardKinematicsTest::getList() const
{
  return fileList;
}

//==============================================================================
const std::vector<common::Uri>& ForwardKinematicsTest::getListForDebug() const
{
  return fileListForDebug;
}

//==============================================================================
const std::vector<SimpleFrame*>& ForwardKinematicsTest::getRefFrames() const
{
  return refFrames;
}

//==============================================================================
void ForwardKinematicsTest::randomizeRefFrames()
{
  for (std::size_t i = 0; i < refFrames.size(); ++i)
  {
    SimpleFrame* F = refFrames[i];

    Eigen::Vector3d p = Random::uniform<Eigen::Vector3d>(-100, 100);
    Eigen::Vector3d theta
        = Random::uniform<Eigen::Vector3d>(-2 * M_PI, 2 * M_PI);

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
template <typename T>
void printComparisonError(
    const std::string& comparison,
    const std::string& name,
    const std::string& frame,
    const T& fk,
    const T& jac)
{
  std::cout << "Disagreement between FK and Jacobian results for " << comparison
            << " of '" << name << "' with a reference Frame of '" << frame
            << "'\n"
            << "FK:  " << fk.transpose() << "\n"
            << "Jac: " << jac.transpose() << "\n";
}

//==============================================================================
template <typename T>
void printComparisonError(
    const std::string& comparison,
    const std::string& nameBN,
    const std::string& nameBNRelativeTo,
    const std::string& frame,
    const T& fk,
    const T& jac)
{
  std::cout << "Disagreement between FK and relative Jacobian results for "
            << comparison << " of '" << nameBN << "' relative to '"
            << nameBNRelativeTo << "' with a reference Frame of '" << frame
            << "'\n"
            << "FK:  " << fk.transpose() << "\n"
            << "Jac: " << jac.transpose() << "\n";
}

//==============================================================================
void compareBodyNodeFkToJacobian(
    const BodyNode* bn, const Frame* refFrame, double tolerance)
{
  using math::AngularJacobian;
  using math::Jacobian;
  using math::LinearJacobian;

  ConstSkeletonPtr skel = bn->getSkeleton();

  VectorXd dq = skel->getVelocities();
  VectorXd ddq = skel->getAccelerations();

  const std::vector<std::size_t>& coords = bn->getDependentGenCoordIndices();
  VectorXd dqSeg = skel->getVelocities(coords);
  VectorXd ddqSeg = skel->getAccelerations(coords);

  //-- Spatial Jacobian tests --------------------------------------------------

  Vector6d SpatialVelFk = bn->getSpatialVelocity(Frame::World(), refFrame);
  Vector6d SpatialAccFk = bn->getSpatialAcceleration(Frame::World(), refFrame);

  Jacobian SpatialJacSeg = bn->getJacobian(refFrame);
  Jacobian SpatialJacDerivSeg = bn->getJacobianSpatialDeriv(refFrame);

  Vector6d SpatialVelJacSeg = SpatialJacSeg * dqSeg;
  Vector6d SpatialAccJacSeg
      = SpatialJacSeg * ddqSeg + SpatialJacDerivSeg * dqSeg;

  Jacobian SpatialJac = skel->getJacobian(bn, refFrame);
  Jacobian SpatialJacDeriv = skel->getJacobianSpatialDeriv(bn, refFrame);

  Vector6d SpatialVelJac = SpatialJac * dq;
  Vector6d SpatialAccJac = SpatialJac * ddq + SpatialJacDeriv * dq;

  bool spatialVelSegEqual = equals(SpatialVelFk, SpatialVelJacSeg, tolerance);
  bool spatialVelEqual = equals(SpatialVelFk, SpatialVelJac, tolerance);
  EXPECT_TRUE(spatialVelSegEqual);
  EXPECT_TRUE(spatialVelEqual);
  if (!spatialVelSegEqual)
    printComparisonError(
        "spatial velocity (seg)",
        bn->getName(),
        refFrame->getName(),
        SpatialVelFk,
        SpatialVelJacSeg);
  if (!spatialVelEqual)
    printComparisonError(
        "spatial velocity",
        bn->getName(),
        refFrame->getName(),
        SpatialVelFk,
        SpatialVelJac);

  bool spatialAccSegEqual = equals(SpatialAccFk, SpatialAccJacSeg, tolerance);
  bool spatialAccEqual = equals(SpatialAccFk, SpatialAccJac, tolerance);
  EXPECT_TRUE(spatialAccSegEqual);
  EXPECT_TRUE(spatialAccEqual);
  if (!spatialAccSegEqual)
    printComparisonError(
        "spatial acceleration (seg)",
        bn->getName(),
        refFrame->getName(),
        SpatialAccFk,
        SpatialAccJacSeg);
  if (!spatialAccEqual)
    printComparisonError(
        "spatial acceleration",
        bn->getName(),
        refFrame->getName(),
        SpatialAccFk,
        SpatialAccJac);

  //-- Linear Jacobian tests ---------------------------------------------------

  Vector3d LinearVelFk = bn->getLinearVelocity(Frame::World(), refFrame);
  Vector3d LinearAccFk = bn->getLinearAcceleration(Frame::World(), refFrame);

  LinearJacobian LinearJacSeg = bn->getLinearJacobian(refFrame);
  LinearJacobian LinearJacDerivSeg = bn->getLinearJacobianDeriv(refFrame);

  Vector3d LinearVelJacSeg = LinearJacSeg * dqSeg;
  Vector3d LinearAccJacSeg = LinearJacSeg * ddqSeg + LinearJacDerivSeg * dqSeg;

  LinearJacobian LinearJac = skel->getLinearJacobian(bn, refFrame);
  LinearJacobian LinearJacDeriv = skel->getLinearJacobianDeriv(bn, refFrame);

  Vector3d LinearVelJac = LinearJac * dq;
  Vector3d LinearAccJac = LinearJac * ddq + LinearJacDeriv * dq;

  bool linearVelSegEqual = equals(LinearVelFk, LinearVelJacSeg, tolerance);
  bool linearVelEqual = equals(LinearVelFk, LinearVelJac, tolerance);
  EXPECT_TRUE(linearVelSegEqual);
  EXPECT_TRUE(linearVelEqual);
  if (!linearVelSegEqual)
    printComparisonError(
        "linear velocity (seg)",
        bn->getName(),
        refFrame->getName(),
        LinearVelFk,
        LinearVelJacSeg);
  if (!linearVelEqual)
    printComparisonError(
        "linear velocity",
        bn->getName(),
        refFrame->getName(),
        LinearVelFk,
        LinearVelJac);

  bool linearAccSegEqual = equals(LinearAccFk, LinearAccJacSeg, tolerance);
  bool linearAccEqual = equals(LinearAccFk, LinearAccJac, tolerance);
  EXPECT_TRUE(linearAccSegEqual);
  EXPECT_TRUE(linearAccEqual);
  if (!linearAccSegEqual)
    printComparisonError(
        "linear acceleration (seg)",
        bn->getName(),
        refFrame->getName(),
        LinearAccFk,
        LinearAccJacSeg);
  if (!linearAccEqual)
    printComparisonError(
        "linear acceleration",
        bn->getName(),
        refFrame->getName(),
        LinearAccFk,
        LinearAccJac);

  //-- Angular Jacobian tests

  Vector3d AngularVelFk = bn->getAngularVelocity(Frame::World(), refFrame);
  Vector3d AngularAccFk = bn->getAngularAcceleration(Frame::World(), refFrame);

  AngularJacobian AngularJacSeg = bn->getAngularJacobian(refFrame);
  AngularJacobian AngularJacDerivSeg = bn->getAngularJacobianDeriv(refFrame);

  Vector3d AngularVelJacSeg = AngularJacSeg * dqSeg;
  Vector3d AngularAccJacSeg
      = AngularJacSeg * ddqSeg + AngularJacDerivSeg * dqSeg;

  AngularJacobian AngularJac = skel->getAngularJacobian(bn, refFrame);
  AngularJacobian AngularJacDeriv = skel->getAngularJacobianDeriv(bn, refFrame);

  Vector3d AngularVelJac = AngularJac * dq;
  Vector3d AngularAccJac = AngularJac * ddq + AngularJacDeriv * dq;

  bool angularVelSegEqual = equals(AngularVelFk, AngularVelJacSeg, tolerance);
  bool angularVelEqual = equals(AngularVelFk, AngularVelJac, tolerance);
  EXPECT_TRUE(angularVelSegEqual);
  EXPECT_TRUE(angularVelEqual);
  if (!angularVelSegEqual)
    printComparisonError(
        "angular velocity (seg)",
        bn->getName(),
        refFrame->getName(),
        AngularVelFk,
        AngularVelJacSeg);
  if (!angularVelEqual)
    printComparisonError(
        "angular velocity",
        bn->getName(),
        refFrame->getName(),
        AngularVelFk,
        AngularVelJac);

  bool angularAccSegEqual = equals(AngularAccFk, AngularAccJacSeg, tolerance);
  bool angularAccEqual = equals(AngularAccFk, AngularAccJac, tolerance);
  EXPECT_TRUE(angularAccSegEqual);
  EXPECT_TRUE(angularAccEqual);
  if (!angularAccSegEqual)
    printComparisonError(
        "angular acceleration (seg)",
        bn->getName(),
        refFrame->getName(),
        AngularAccFk,
        AngularAccJacSeg);
  if (!angularAccEqual)
    printComparisonError(
        "angular acceleration",
        bn->getName(),
        refFrame->getName(),
        AngularAccFk,
        AngularAccJac);
}

//==============================================================================
void compareBodyNodeFkToJacobian(
    const BodyNode* bn,
    const Frame* refFrame,
    const Eigen::Vector3d& offset,
    double tolerance)
{
  using math::AngularJacobian;
  using math::Jacobian;
  using math::LinearJacobian;

  ConstSkeletonPtr skel = bn->getSkeleton();

  VectorXd dq = skel->getVelocities();
  VectorXd ddq = skel->getAccelerations();

  const std::vector<std::size_t>& coords = bn->getDependentGenCoordIndices();
  VectorXd dqSeg = skel->getVelocities(coords);
  VectorXd ddqSeg = skel->getAccelerations(coords);

  //-- Spatial Jacobian tests --------------------------------------------------

  Vector6d SpatialVelFk
      = bn->getSpatialVelocity(offset, Frame::World(), refFrame);
  Vector6d SpatialAccFk
      = bn->getSpatialAcceleration(offset, Frame::World(), refFrame);

  Jacobian SpatialJacSeg = bn->getJacobian(offset, refFrame);
  Jacobian SpatialJacDerivSeg = bn->getJacobianSpatialDeriv(offset, refFrame);

  Vector6d SpatialVelJacSeg = SpatialJacSeg * dqSeg;
  Vector6d SpatialAccJacSeg
      = SpatialJacSeg * ddqSeg + SpatialJacDerivSeg * dqSeg;

  Jacobian SpatialJac = skel->getJacobian(bn, offset, refFrame);
  Jacobian SpatialJacDeriv
      = skel->getJacobianSpatialDeriv(bn, offset, refFrame);

  Vector6d SpatialVelJac = SpatialJac * dq;
  Vector6d SpatialAccJac = SpatialJac * ddq + SpatialJacDeriv * dq;

  bool spatialVelSegEqual = equals(SpatialVelFk, SpatialVelJacSeg, tolerance);
  bool spatialVelEqual = equals(SpatialVelFk, SpatialVelJac, tolerance);
  EXPECT_TRUE(spatialVelSegEqual);
  EXPECT_TRUE(spatialVelEqual);
  if (!spatialVelSegEqual)
    printComparisonError(
        "spatial velocity w/ offset (seg)",
        bn->getName(),
        refFrame->getName(),
        SpatialVelFk,
        SpatialVelJacSeg);
  if (!spatialVelEqual)
    printComparisonError(
        "spatial velocity w/ offset",
        bn->getName(),
        refFrame->getName(),
        SpatialVelFk,
        SpatialVelJac);

  bool spatialAccSegEqual = equals(SpatialAccFk, SpatialAccJacSeg, tolerance);
  bool spatialAccEqual = equals(SpatialAccFk, SpatialAccJac, tolerance);
  EXPECT_TRUE(spatialAccSegEqual);
  EXPECT_TRUE(spatialAccEqual);
  if (!spatialAccSegEqual)
    printComparisonError(
        "spatial acceleration w/ offset (seg)",
        bn->getName(),
        refFrame->getName(),
        SpatialAccFk,
        SpatialAccJacSeg);
  if (!spatialAccEqual)
    printComparisonError(
        "spatial acceleration w/ offset",
        bn->getName(),
        refFrame->getName(),
        SpatialAccFk,
        SpatialAccJac);

  //-- Linear Jacobian tests ---------------------------------------------------

  Vector3d LinearVelFk
      = bn->getLinearVelocity(offset, Frame::World(), refFrame);
  Vector3d LinearAccFk
      = bn->getLinearAcceleration(offset, Frame::World(), refFrame);

  LinearJacobian LinearJacSeg = bn->getLinearJacobian(offset, refFrame);
  LinearJacobian LinearJacDerivSeg
      = bn->getLinearJacobianDeriv(offset, refFrame);

  Vector3d LinearVelJacSeg = LinearJacSeg * dqSeg;
  Vector3d LinearAccJacSeg = LinearJacSeg * ddqSeg + LinearJacDerivSeg * dqSeg;

  LinearJacobian LinearJac = skel->getLinearJacobian(bn, offset, refFrame);
  LinearJacobian LinearJacDeriv
      = skel->getLinearJacobianDeriv(bn, offset, refFrame);

  Vector3d LinearVelJac = LinearJac * dq;
  Vector3d LinearAccJac = LinearJac * ddq + LinearJacDeriv * dq;

  bool linearVelSegEqual = equals(LinearVelFk, LinearVelJacSeg, tolerance);
  bool linearVelEqual = equals(LinearVelFk, LinearVelJac, tolerance);
  EXPECT_TRUE(linearVelSegEqual);
  EXPECT_TRUE(linearVelEqual);
  if (!linearVelSegEqual)
    printComparisonError(
        "linear velocity w/ offset (seg)",
        bn->getName(),
        refFrame->getName(),
        LinearVelFk,
        LinearVelJacSeg);
  if (!linearVelEqual)
    printComparisonError(
        "linear velocity w/ offset",
        bn->getName(),
        refFrame->getName(),
        LinearVelFk,
        LinearVelJac);

  bool linearAccSegEqual = equals(LinearAccFk, LinearAccJacSeg, tolerance);
  bool linearAccEqual = equals(LinearAccFk, LinearAccJac, tolerance);
  EXPECT_TRUE(linearAccSegEqual);
  EXPECT_TRUE(linearAccEqual);
  if (!linearAccSegEqual)
    printComparisonError(
        "linear acceleration w/ offset (seg)",
        bn->getName(),
        refFrame->getName(),
        LinearAccFk,
        LinearAccJacSeg);
  if (!linearAccEqual)
    printComparisonError(
        "linear acceleration w/ offset",
        bn->getName(),
        refFrame->getName(),
        LinearAccFk,
        LinearAccJac);

  //-- Angular Jacobian tests --------------------------------------------------

  Vector3d AngularVelFk = bn->getAngularVelocity(Frame::World(), refFrame);
  Vector3d AngularAccFk = bn->getAngularAcceleration(Frame::World(), refFrame);

  AngularJacobian AngularJacSeg = bn->getAngularJacobian(refFrame);
  AngularJacobian AngularJacDerivSeg = bn->getAngularJacobianDeriv(refFrame);

  Vector3d AngularVelJacSeg = AngularJacSeg * dqSeg;
  Vector3d AngularAccJacSeg
      = AngularJacSeg * ddqSeg + AngularJacDerivSeg * dqSeg;

  AngularJacobian AngularJac = skel->getAngularJacobian(bn, refFrame);
  AngularJacobian AngularJacDeriv = skel->getAngularJacobianDeriv(bn, refFrame);

  Vector3d AngularVelJac = AngularJac * dq;
  Vector3d AngularAccJac = AngularJac * ddq + AngularJacDeriv * dq;

  bool angularVelSegEqual = equals(AngularVelFk, AngularVelJacSeg, tolerance);
  bool angularVelEqual = equals(AngularVelFk, AngularVelJac, tolerance);
  EXPECT_TRUE(angularVelSegEqual);
  EXPECT_TRUE(angularVelEqual);
  if (!angularVelSegEqual)
    printComparisonError(
        "angular velocity w/ offset (seg)",
        bn->getName(),
        refFrame->getName(),
        AngularVelFk,
        AngularVelJacSeg);
  if (!angularVelEqual)
    printComparisonError(
        "angular velocity w/ offset",
        bn->getName(),
        refFrame->getName(),
        AngularVelFk,
        AngularVelJac);

  bool angularAccSegEqual = equals(AngularAccFk, AngularAccJacSeg, tolerance);
  bool angularAccEqual = equals(AngularAccFk, AngularAccJac, tolerance);
  EXPECT_TRUE(angularAccSegEqual);
  if (!angularAccSegEqual)
    printComparisonError(
        "angular acceleration w/ offset (seg)",
        bn->getName(),
        refFrame->getName(),
        AngularAccFk,
        AngularAccJacSeg);
  EXPECT_TRUE(angularAccEqual);
  if (!angularAccEqual)
    printComparisonError(
        "angular acceleration w/ offset",
        bn->getName(),
        refFrame->getName(),
        AngularAccFk,
        AngularAccJac);
}

//==============================================================================
void compareBodyNodeFkToJacobianRelative(
    const JacobianNode* bn,
    const JacobianNode* relativeTo,
    const Frame* refFrame,
    double tolerance)
{
  using math::AngularJacobian;
  using math::Jacobian;
  using math::LinearJacobian;

  assert(bn->getSkeleton() == relativeTo->getSkeleton());
  auto skel = bn->getSkeleton();

  VectorXd dq = skel->getVelocities();
  VectorXd ddq = skel->getAccelerations();

  //-- Spatial Jacobian tests --------------------------------------------------

  Vector6d SpatialVelFk = bn->getSpatialVelocity(relativeTo, refFrame);
  Vector6d SpatialAccFk = bn->getSpatialAcceleration(relativeTo, refFrame);

  Jacobian SpatialJac = skel->getJacobian(bn, relativeTo, refFrame);
  Jacobian SpatialJacDeriv
      = skel->getJacobianSpatialDeriv(bn, relativeTo, refFrame);

  Vector6d SpatialVelJac = SpatialJac * dq;
  Vector6d SpatialAccJac = SpatialJac * ddq + SpatialJacDeriv * dq;

  bool spatialVelEqual = equals(SpatialVelFk, SpatialVelJac, tolerance);
  EXPECT_TRUE(spatialVelEqual);
  if (!spatialVelEqual)
  {
    printComparisonError(
        "spatial velocity",
        bn->getName(),
        relativeTo->getName(),
        refFrame->getName(),
        SpatialVelFk,
        SpatialVelJac);
  }

  bool spatialAccEqual = equals(SpatialAccFk, SpatialAccJac, tolerance);
  EXPECT_TRUE(spatialAccEqual);
  if (!spatialAccEqual)
  {
    printComparisonError(
        "spatial acceleration",
        bn->getName(),
        relativeTo->getName(),
        refFrame->getName(),
        SpatialAccFk,
        SpatialAccJac);
  }

  //-- Linear Jacobian tests --------------------------------------------------

  Vector3d LinearVelFk = bn->getLinearVelocity(relativeTo, refFrame);

  LinearJacobian LinearJac = skel->getLinearJacobian(bn, relativeTo, refFrame);

  Vector3d LinearVelJac = LinearJac * dq;

  bool linearVelEqual = equals(LinearVelFk, LinearVelJac, tolerance);
  EXPECT_TRUE(linearVelEqual);
  if (!linearVelEqual)
  {
    printComparisonError(
        "linear velocity",
        bn->getName(),
        relativeTo->getName(),
        refFrame->getName(),
        LinearVelFk,
        LinearVelJac);
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
    printComparisonError(
        "angular velocity",
        bn->getName(),
        relativeTo->getName(),
        refFrame->getName(),
        AngularVelFk,
        AngularVelJac);
  }
}

//==============================================================================
void compareBodyNodeFkToJacobianRelative(
    const JacobianNode* bn,
    const Eigen::Vector3d& _offset,
    const JacobianNode* relativeTo,
    const Frame* refFrame,
    double tolerance)
{
  using math::AngularJacobian;
  using math::Jacobian;
  using math::LinearJacobian;

  assert(bn->getSkeleton() == relativeTo->getSkeleton());
  auto skel = bn->getSkeleton();

  VectorXd dq = skel->getVelocities();
  VectorXd ddq = skel->getAccelerations();

  //-- Spatial Jacobian tests --------------------------------------------------

  Vector6d SpatialVelFk = bn->getSpatialVelocity(_offset, relativeTo, refFrame);
  Vector6d SpatialAccFk
      = bn->getSpatialAcceleration(_offset, relativeTo, refFrame);

  Jacobian SpatialJac = skel->getJacobian(bn, _offset, relativeTo, refFrame);
  Jacobian SpatialJacDeriv
      = skel->getJacobianSpatialDeriv(bn, _offset, relativeTo, refFrame);

  Vector6d SpatialVelJac = SpatialJac * dq;
  Vector6d SpatialAccJac = SpatialJac * ddq + SpatialJacDeriv * dq;

  bool spatialVelEqual = equals(SpatialVelFk, SpatialVelJac, tolerance);
  EXPECT_TRUE(spatialVelEqual);
  if (!spatialVelEqual)
  {
    printComparisonError(
        "spatial velocity w/ offset",
        bn->getName(),
        relativeTo->getName(),
        refFrame->getName(),
        SpatialVelFk,
        SpatialVelJac);
  }

  bool spatialAccEqual = equals(SpatialAccFk, SpatialAccJac, tolerance);
  EXPECT_TRUE(spatialAccEqual);
  if (!spatialAccEqual)
  {
    printComparisonError(
        "spatial acceleration w/ offset",
        bn->getName(),
        relativeTo->getName(),
        refFrame->getName(),
        SpatialAccFk,
        SpatialAccJac);
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
    printComparisonError(
        "linear velocity w/ offset",
        bn->getName(),
        relativeTo->getName(),
        refFrame->getName(),
        LinearVelFk,
        LinearVelJac);
  }
}

//==============================================================================
void ForwardKinematicsTest::testJacobians(const common::Uri& uri)
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
#ifndef NDEBUG // Debug mode
  int nTestItr = 1;
#else
  int nTestItr = 1;
#endif
  double qLB = -0.5 * constantsd::pi();
  double qUB = 0.5 * constantsd::pi();
  double dqLB = -0.5 * constantsd::pi();
  double dqUB = 0.5 * constantsd::pi();
  double ddqLB = -0.5 * constantsd::pi();
  double ddqUB = 0.5 * constantsd::pi();
  Vector3d gravity(0.0, -9.81, 0.0);

  // load skeleton
  simulation::WorldPtr world = utils::SkelParser::readWorld(uri);
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
      if (j > std::ceil(nTestItr / 2))
      {
        SkeletonPtr copy = skeleton->cloneSkeleton();
        std::size_t maxNode = skeleton->getNumBodyNodes() - 1;
        BodyNode* bn1 = skeleton->getBodyNode(
            math::Random::uniform<std::size_t>(0, maxNode));
        BodyNode* bn2 = skeleton->getBodyNode(
            math::Random::uniform<std::size_t>(0, maxNode));

        if (bn1 != bn2)
        {
          BodyNode* child = bn1->descendsFrom(bn2) ? bn1 : bn2;
          BodyNode* parent = child == bn1 ? bn2 : bn1;

          child->moveTo(parent);
        }

        EXPECT_TRUE(skeleton->getNumBodyNodes() == copy->getNumBodyNodes());
        EXPECT_TRUE(skeleton->getNumDofs() == copy->getNumDofs());
      }

      // Generate a random state
      VectorXd q = VectorXd(dof);
      VectorXd dq = VectorXd(dof);
      VectorXd ddq = VectorXd(dof);
      for (int k = 0; k < dof; ++k)
      {
        q[k] = math::Random::uniform(qLB, qUB);
        dq[k] = math::Random::uniform(dqLB, dqUB);
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
            bn,
            Frame::World(),
            Random::uniform<Eigen::Vector3d>(-10, 10),
            TOLERANCE);

        // Compare results using this BodyNode's own reference Frame
        compareBodyNodeFkToJacobian(bn, bn, TOLERANCE);
        compareBodyNodeFkToJacobian(bn, bn, bn->getLocalCOM(), TOLERANCE);
        compareBodyNodeFkToJacobian(
            bn, bn, Random::uniform<Eigen::Vector3d>(-10, 10), TOLERANCE);

        // Compare results using the randomized reference Frames
        for (std::size_t r = 0; r < refFrames.size(); ++r)
        {
          compareBodyNodeFkToJacobian(bn, refFrames[r], TOLERANCE);
          compareBodyNodeFkToJacobian(
              bn, refFrames[r], bn->getLocalCOM(), TOLERANCE);
          compareBodyNodeFkToJacobian(
              bn,
              refFrames[r],
              Random::uniform<Eigen::Vector3d>(-10, 10),
              TOLERANCE);
        }

        // -- Relative Jacobian tests

        compareBodyNodeFkToJacobianRelative(bn, bn, Frame::World(), TOLERANCE);

#ifndef NDEBUG // Debug mode
        if (skeleton->getNumBodyNodes() == 0u)
          continue;

        for (std::size_t l = skeleton->getNumBodyNodes() - 1;
             l < skeleton->getNumBodyNodes();
             ++l)
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
              bn,
              Random::uniform<Eigen::Vector3d>(-10, 10),
              relativeTo,
              Frame::World(),
              TOLERANCE);

          compareBodyNodeFkToJacobianRelative(bn, relativeTo, bn, TOLERANCE);
          compareBodyNodeFkToJacobianRelative(
              bn, bn->getLocalCOM(), relativeTo, bn, TOLERANCE);
          compareBodyNodeFkToJacobianRelative(
              bn,
              Random::uniform<Eigen::Vector3d>(-10, 10),
              relativeTo,
              bn,
              TOLERANCE);

          compareBodyNodeFkToJacobianRelative(
              bn, relativeTo, relativeTo, TOLERANCE);
          compareBodyNodeFkToJacobianRelative(
              bn, bn->getLocalCOM(), relativeTo, relativeTo, TOLERANCE);
          compareBodyNodeFkToJacobianRelative(
              bn,
              Random::uniform<Eigen::Vector3d>(-10, 10),
              relativeTo,
              relativeTo,
              TOLERANCE);

          for (std::size_t r = 0; r < refFrames.size(); ++r)
          {
            compareBodyNodeFkToJacobianRelative(
                bn, relativeTo, refFrames[r], TOLERANCE);
            compareBodyNodeFkToJacobianRelative(
                bn, bn->getLocalCOM(), relativeTo, refFrames[r], TOLERANCE);
            compareBodyNodeFkToJacobianRelative(
                bn,
                Random::uniform<Eigen::Vector3d>(-10, 10),
                relativeTo,
                refFrames[r],
                TOLERANCE);
          }
        }
      }
    }
  }
}


//==============================================================================
void ForwardKinematicsTest::testFiniteDifferenceGeneralizedCoordinates(
    const common::Uri& uri)
{
  using namespace std;
  using namespace Eigen;
  using namespace dart;
  using namespace math;
  using namespace dynamics;
  using namespace simulation;
  using namespace utils;

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
void ForwardKinematicsTest::testFiniteDifferenceBodyNodeVelocity(const common::Uri& uri)
{
  using namespace std;
  using namespace Eigen;
  using namespace dart;
  using namespace math;
  using namespace dynamics;
  using namespace simulation;
  using namespace utils;

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
void ForwardKinematicsTest::testFiniteDifferenceBodyNodeAcceleration(
    const common::Uri& uri)
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
  std::size_t nRandomItr = 5;
  std::size_t numSteps = 5;
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
void ForwardKinematicsTest::testForwardKinematics(const common::Uri& uri)
{
  auto world = utils::SkelParser::readWorld(uri);
  EXPECT_TRUE(world != nullptr);

  auto numSkeletons = world->getNumSkeletons();
  for (auto i = 0u; i < numSkeletons; ++i)
  {
    auto skeleton  = world->getSkeleton(i);
    testForwardKinematicsSkeleton(skeleton);
  }
}

//==============================================================================
TEST_F(ForwardKinematicsTest, testFiniteDifference)
{
#ifndef NDEBUG  // Debug mode
  const auto& list = getListForDebug();
#else
  const auto& list = getList();
#endif

  for (const auto& uri : list)
  {
    testFiniteDifferenceGeneralizedCoordinates(uri);
    testFiniteDifferenceBodyNodeVelocity(uri);
    testFiniteDifferenceBodyNodeAcceleration(uri);
  }
}

//==============================================================================
TEST_F(ForwardKinematicsTest, testForwardKinematics)
{
#ifndef NDEBUG  // Debug mode
  const auto& list = getListForDebug();
#else
  const auto& list = getList();
#endif

  for (const auto& uri : list)
  {
    testForwardKinematics(uri);
  }
}

//==============================================================================
TEST_F(ForwardKinematicsTest, YawRoll)
{
  // Checks forward kinematics for two DoF arm manipulators.
  // NOTE: The following is the reference frame description of the world
  //       frame. The x-axis is into the page, z-axis is to the top of the
  //       page and the y-axis is to the left. At the zero angle, the links
  //       are parallel to the z-axis and face the +x-axis.

  // Create the world
  const double l1 = 1.5, l2 = 1.0;
  SkeletonPtr robot = createTwoLinkRobot(
      Vector3d(0.3, 0.3, l1), DOF_YAW, Vector3d(0.3, 0.3, l2), DOF_ROLL);

  // Set the test cases with the joint values and the expected end-effector
  // positions
  const std::size_t numTests = 2;
  double temp = sqrt(0.5 * l2 * l2);
  Vector2d joints[numTests]
      = {Vector2d(constantsd::pi() / 4.0, constantsd::pi() / 2.0),
         Vector2d(-constantsd::pi() / 4.0, -constantsd::pi() / 4.0)};
  Vector3d expectedPos[numTests]
      = {Vector3d(temp, -temp, l1),
         Vector3d(temp / sqrt(2.0), temp / sqrt(2.0), l1 + temp)};

  // Check each case by setting the joint values and obtaining the end-effector
  // position
  for (std::size_t i = 0; i < numTests; i++)
  {
    robot->setPositions(Eigen::VectorXd(joints[i]));
    BodyNode* bn = robot->getBodyNode("ee");
    Vector3d actual = bn->getTransform().translation();
    bool equality = equals(actual, expectedPos[i], 1e-3);
    EXPECT_TRUE(equality);
    if (!equality)
    {
      std::cout << "Joint values: " << joints[i].transpose() << std::endl;
      std::cout << "Actual pos: " << actual.transpose() << std::endl;
      std::cout << "Expected pos: " << expectedPos[i].transpose() << std::endl;
    }
  }
}

//==============================================================================
// TODO: Use link lengths in expectations explicitly
TEST_F(ForwardKinematicsTest, TwoRolls)
{
  // Checks forward kinematics for two DoF arm manipulators.
  // NOTE: The following is the reference frame description of the world
  //       frame. The x-axis is into the page, z-axis is to the top of the
  //       page and the y-axis is to the left. At the zero angle, the links
  //       are parallel to the z-axis and face the +x-axis.

  // Create the world
  const double link1 = 1.5, link2 = 1.0;
  SkeletonPtr robot = createTwoLinkRobot(
      Vector3d(0.3, 0.3, link1), DOF_ROLL, Vector3d(0.3, 0.3, link2), DOF_ROLL);

  // Set the test cases with the joint values and the expected end-effector
  // positions
  const std::size_t numTests = 2;
  Vector2d joints[numTests]
      = {Vector2d(0.0, constantsd::pi() / 2.0),
         Vector2d(3 * constantsd::pi() / 4.0, -constantsd::pi() / 4.0)};
  Vector3d expectedPos[numTests]
      = {Vector3d(0.0, -1.0, 1.5), Vector3d(0.0, -2.06, -1.06)};

  // Check each case by setting the joint values and obtaining the end-effector
  // position
  for (std::size_t i = 0; i < numTests; i++)
  {
    robot->setPositions(joints[i]);
    Vector3d actual = robot->getBodyNode("ee")->getTransform().translation();
    bool equality = equals(actual, expectedPos[i], 1e-3);
    EXPECT_TRUE(equality);
    if (!equality)
    {
      std::cout << "Joint values: " << joints[i].transpose() << std::endl;
      std::cout << "Actual pos: " << actual.transpose() << std::endl;
      std::cout << "Expected pos: " << expectedPos[i].transpose() << std::endl;
    }
  }
}

//==============================================================================
Eigen::MatrixXd finiteDifferenceJacobian(
    const SkeletonPtr& skeleton,
    const Eigen::VectorXd& q,
    const std::vector<std::size_t>& active_indices,
    JacobianNode* node)
{
  Eigen::MatrixXd J(3, q.size());
  for (int i = 0; i < q.size(); ++i)
  {
    const double dq = 1e-4;

    Eigen::VectorXd q_up = q;
    Eigen::VectorXd q_down = q;

    q_up[i] += 0.5 * dq;
    q_down[i] -= 0.5 * dq;

    skeleton->setPositions(active_indices, q_up);
    Eigen::Vector3d x_up = node->getTransform().translation();

    skeleton->setPositions(active_indices, q_down);
    Eigen::Vector3d x_down = node->getTransform().translation();

    skeleton->setPositions(active_indices, q);
    J.col(i)
        = node->getWorldTransform().linear().transpose() * (x_up - x_down) / dq;
  }

  return J;
}

//==============================================================================
Eigen::MatrixXd standardJacobian(
    const SkeletonPtr& skeleton,
    const Eigen::VectorXd& q,
    const std::vector<std::size_t>& activeIndices,
    JacobianNode* node)
{
  skeleton->setPositions(activeIndices, q);

  Eigen::MatrixXd J = skeleton->getJacobian(node).bottomRows<3>();

  Eigen::MatrixXd reduced_J(3, q.size());
  for (auto i = 0u; i < static_cast<std::size_t>(q.size()); ++i)
    reduced_J.col(i) = J.col(static_cast<int>(activeIndices[i]));

  return reduced_J;
}

//==============================================================================
TEST_F(ForwardKinematicsTest, testJacobians)
{
#ifndef NDEBUG // Debug mode
  const auto& list = getListForDebug();
#else
  const auto& list = getList();
#endif

  for (const auto& uri : list)
  {
    testJacobians(uri);
  }
}
//==============================================================================
TEST_F(ForwardKinematicsTest, JacobianPartialChange)
{
  // This is a regression test for issue #499
  const double tolerance = 1e-8;

  dart::utils::DartLoader loader;
  SkeletonPtr skeleton1
      = loader.parseSkeleton("dart://sample/urdf/KR5/KR5 sixx R650.urdf");

  SkeletonPtr skeleton2 = skeleton1->cloneSkeleton();

  std::vector<std::size_t> active_indices;
  for (std::size_t i = 0; i < 3; ++i)
    active_indices.push_back(i);

  Eigen::VectorXd q
      = Eigen::VectorXd::Random(static_cast<int>(active_indices.size()));

  Eigen::MatrixXd fd_J = finiteDifferenceJacobian(
      skeleton1,
      q,
      active_indices,
      skeleton1->getBodyNode(skeleton1->getNumBodyNodes() - 1));

  Eigen::MatrixXd J = standardJacobian(
      skeleton2,
      q,
      active_indices,
      skeleton2->getBodyNode(skeleton2->getNumBodyNodes() - 1));

  EXPECT_TRUE((fd_J - J).norm() < tolerance);

  q = Eigen::VectorXd::Random(static_cast<int>(active_indices.size()));

  fd_J = finiteDifferenceJacobian(
      skeleton1,
      q,
      active_indices,
      skeleton1->getBodyNode(skeleton1->getNumBodyNodes() - 1));

  J = standardJacobian(
      skeleton2,
      q,
      active_indices,
      skeleton2->getBodyNode(skeleton2->getNumBodyNodes() - 1));

  EXPECT_TRUE((fd_J - J).norm() < tolerance);
}

//==============================================================================
TEST_F(ForwardKinematicsTest, JacobianEndEffectorChange)
{
  // This is a regression test for pull request #683
  const double tolerance = 1e-8;

  dart::utils::DartLoader loader;
  SkeletonPtr skeleton1
      = loader.parseSkeleton("dart://sample/urdf/KR5/KR5 sixx R650.urdf");

  BodyNode* last_bn1 = skeleton1->getBodyNode(skeleton1->getNumBodyNodes() - 1);
  EndEffector* ee1 = last_bn1->createEndEffector();

  SkeletonPtr skeleton2 = skeleton1->cloneSkeleton();
  BodyNode* last_bn2 = skeleton2->getBodyNode(skeleton2->getNumBodyNodes() - 1);
  EndEffector* ee2 = last_bn2->createEndEffector();

  std::vector<std::size_t> active_indices;
  for (std::size_t i = 0; i < 3; ++i)
    active_indices.push_back(i);

  Eigen::VectorXd q
      = Eigen::VectorXd::Random(static_cast<int>(active_indices.size()));

  Eigen::MatrixXd fd_J
      = finiteDifferenceJacobian(skeleton1, q, active_indices, ee1);

  Eigen::MatrixXd J = standardJacobian(skeleton2, q, active_indices, ee2);

  EXPECT_TRUE((fd_J - J).norm() < tolerance);

  q = Eigen::VectorXd::Random(static_cast<int>(active_indices.size()));
  fd_J = finiteDifferenceJacobian(skeleton1, q, active_indices, ee1);
  J = standardJacobian(skeleton2, q, active_indices, ee2);

  EXPECT_TRUE((fd_J - J).norm() < tolerance);
}
