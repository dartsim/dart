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

#include <dart/all.hpp>

#include <gtest/gtest.h>

#include <iostream>
#include <limits>
#include <string>

using namespace dart;
using namespace dart::dynamics;
using namespace dart::math;

// class ZeroDofJointTest : public GenericJoint<NullSpace>
//{
// public:
//  /// Constructor called by Skeleton class
//  ZeroDofJointTest(const Properties& properties = Properties())
//    : GenericJoint<NullSpace>(properties) {}

//  virtual ~ZeroDofJointTest() {}

//  /// Gets a string representing the joint type
//  std::string_view getType() const override { return getStaticType(); }

//  std::string_view getStaticType() const
//  {
//    static const std::string name = "TestWeldJoint";
//    return name;
//  }

//  // Documentation inherited
//  bool isCyclic(size_t index) const override { return false; }

//  const JacobianMatrix getRelativeJacobianStatic(
//      const Vector& positions) const override
//  { return JacobianMatrix(); }

// protected:
//  // Documentation inherited
//  Joint* clone() const override { return nullptr; }

//  // Documentation inherited
//  void updateDegreeOfFreedomNames() override {}

//  // Documentation inherited
//  void updateRelativeTransform() const override {}

//  // Documentation inherited
//  void updateRelativeJacobian(bool mandatory = true) const override {}

//  // Documentation inherited
//  void updateRelativeJacobianTimeDeriv() const override {}
//};

class SingleDofJointTest : public GenericJoint<R1Space>
{
public:
  /// Constructor called by Skeleton class
  SingleDofJointTest(const Properties& properties = Properties())
    : GenericJoint<R1Space>(properties)
  {
  }

  virtual ~SingleDofJointTest() {}

  /// Gets a string representing the joint type
  std::string_view getType() const override
  {
    return getStaticType();
  }

  std::string_view getStaticType() const
  {
    static constexpr std::string_view name = "TestSingleDofJoint";
    return name;
  }

  // Documentation inherited
  bool isCyclic(size_t /*index*/) const override
  {
    return false;
  }

  JacobianMatrix getRelativeJacobianStatic(
      const Vector& /*positions*/) const override
  {
    return JacobianMatrix();
  }

protected:
  // Documentation inherited
  Joint* clone() const override
  {
    return nullptr;
  }

  // Documentation inherited
  void updateDegreeOfFreedomNames() override {}

  // Documentation inherited
  void updateRelativeTransform() const override {}

  // Documentation inherited
  void updateRelativeJacobian(bool /*mandatory = true*/) const override {}

  // Documentation inherited
  void updateRelativeJacobianTimeDeriv() const override {}
};

class MultiDofJointTest : public GenericJoint<RealVectorSpace<6>>
{
public:
  /// Constructor called by Skeleton class
  MultiDofJointTest(const Properties& properties = Properties())
    : GenericJoint<RealVectorSpace<6>>(properties)
  {
  }

  virtual ~MultiDofJointTest() {}

  /// Gets a string representing the joint type
  std::string_view getType() const override
  {
    return getStaticType();
  }

  std::string_view getStaticType() const
  {
    static constexpr std::string_view name = "TestMultiDofJoint";
    return name;
  }

  // Documentation inherited
  bool isCyclic(size_t /*index*/) const override
  {
    return false;
  }

  JacobianMatrix getRelativeJacobianStatic(
      const Vector& /*positions*/) const override
  {
    return JacobianMatrix();
  }

protected:
  // Documentation inherited
  Joint* clone() const override
  {
    return nullptr;
  }

  // Documentation inherited
  void updateDegreeOfFreedomNames() override {}

  // Documentation inherited
  void updateRelativeTransform() const override {}

  // Documentation inherited
  void updateRelativeJacobian(bool /*mandatory = true*/) const override {}

  // Documentation inherited
  void updateRelativeJacobianTimeDeriv() const override {}
};

class SO3JointTest : public GenericJoint<SO3Space>
{
public:
  /// Constructor called by Skeleton class
  SO3JointTest(const Properties& properties = Properties())
    : GenericJoint<SO3Space>(properties)
  {
  }

  virtual ~SO3JointTest() {}

  /// Gets a string representing the joint type
  std::string_view getType() const override
  {
    return getStaticType();
  }

  std::string_view getStaticType() const
  {
    static constexpr std::string_view name = "TestMultiDofJoint";
    return name;
  }

  // Documentation inherited
  bool isCyclic(size_t /*index*/) const override
  {
    return false;
  }

  JacobianMatrix getRelativeJacobianStatic(
      const Vector& /*positions*/) const override
  {
    return JacobianMatrix();
  }

protected:
  // Documentation inherited
  Joint* clone() const override
  {
    return nullptr;
  }

  // Documentation inherited
  void updateDegreeOfFreedomNames() override {}

  // Documentation inherited
  void updateRelativeTransform() const override {}

  // Documentation inherited
  void updateRelativeJacobian(bool /*mandatory = true*/) const override {}

  // Documentation inherited
  void updateRelativeJacobianTimeDeriv() const override {}
};

//==============================================================================
TEST(GenericJoint, Basic)
{
  //  ZeroDofJointTest zeroDofJoint;
  SingleDofJointTest singleDofJoint;
  MultiDofJointTest genericJoint;
  SO3JointTest so3Joint;
}

#if GTEST_HAS_DEATH_TEST
//==============================================================================
TEST(GenericJoint, RejectsNonFiniteInputs)
{
  #ifdef NDEBUG
  GTEST_SKIP() << "Assertions are disabled in Release builds.";
  #endif

  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();

  EXPECT_DEATH(
      {
        SingleDofJointTest joint;
        joint.setPosition(0, nan);
      },
      "");

  EXPECT_DEATH(
      {
        SingleDofJointTest joint;
        joint.setVelocity(0, inf);
      },
      "");

  EXPECT_DEATH(
      {
        MultiDofJointTest joint;
        const auto ndofs = static_cast<Eigen::Index>(joint.getNumDofs());
        Eigen::VectorXd positions = Eigen::VectorXd::Zero(ndofs);
        positions[1] = inf;
        joint.setPositions(positions);
      },
      "");

  EXPECT_DEATH(
      {
        MultiDofJointTest joint;
        const auto ndofs = static_cast<Eigen::Index>(joint.getNumDofs());
        Eigen::VectorXd accelerations = Eigen::VectorXd::Zero(ndofs);
        accelerations[2] = nan;
        joint.setAccelerations(accelerations);
      },
      "");
}
#endif

//==============================================================================
// Test that negative physics parameters are clamped to zero with a warning
// (rather than crashing via assertion). This is important for robustness when
// loading models with invalid parameters (e.g., negative damping in SDF files).
//==============================================================================
TEST(GenericJoint, NegativeDampingClampedToZero)
{
  SingleDofJointTest joint;

  // Set negative damping - should be clamped to 0
  joint.setDampingCoefficient(0, -5.0);
  EXPECT_EQ(joint.getDampingCoefficient(0), 0.0);

  // Positive damping should work normally
  joint.setDampingCoefficient(0, 10.0);
  EXPECT_EQ(joint.getDampingCoefficient(0), 10.0);

  // Zero damping should work
  joint.setDampingCoefficient(0, 0.0);
  EXPECT_EQ(joint.getDampingCoefficient(0), 0.0);
}

//==============================================================================
TEST(GenericJoint, NegativeDampingCoefficientsVectorClampedToZero)
{
  MultiDofJointTest joint;
  const auto ndofs = static_cast<Eigen::Index>(joint.getNumDofs());

  // Set vector with some negative values - they should be clamped to 0
  Eigen::VectorXd dampings = Eigen::VectorXd::Zero(ndofs);
  dampings[0] = -1.0;
  dampings[1] = 5.0;
  dampings[2] = -10.0;
  dampings[3] = 0.0;
  dampings[4] = 3.0;
  dampings[5] = -0.5;

  joint.setDampingCoefficients(dampings);

  EXPECT_EQ(joint.getDampingCoefficient(0), 0.0); // Was -1.0
  EXPECT_EQ(joint.getDampingCoefficient(1), 5.0);
  EXPECT_EQ(joint.getDampingCoefficient(2), 0.0); // Was -10.0
  EXPECT_EQ(joint.getDampingCoefficient(3), 0.0);
  EXPECT_EQ(joint.getDampingCoefficient(4), 3.0);
  EXPECT_EQ(joint.getDampingCoefficient(5), 0.0); // Was -0.5
}

//==============================================================================
TEST(GenericJoint, NegativeFrictionClampedToZero)
{
  SingleDofJointTest joint;

  // Set negative friction - should be clamped to 0
  joint.setCoulombFriction(0, -2.0);
  EXPECT_EQ(joint.getCoulombFriction(0), 0.0);

  // Positive friction should work normally
  joint.setCoulombFriction(0, 1.5);
  EXPECT_EQ(joint.getCoulombFriction(0), 1.5);
}

//==============================================================================
TEST(GenericJoint, NegativeFrictionsVectorClampedToZero)
{
  MultiDofJointTest joint;
  const auto ndofs = static_cast<Eigen::Index>(joint.getNumDofs());

  Eigen::VectorXd frictions = Eigen::VectorXd::Zero(ndofs);
  frictions[0] = -0.5;
  frictions[1] = 1.0;
  frictions[2] = -3.0;

  joint.setFrictions(frictions);

  EXPECT_EQ(joint.getCoulombFriction(0), 0.0); // Was -0.5
  EXPECT_EQ(joint.getCoulombFriction(1), 1.0);
  EXPECT_EQ(joint.getCoulombFriction(2), 0.0); // Was -3.0
}

//==============================================================================
TEST(GenericJoint, NegativeSpringStiffnessClampedToZero)
{
  SingleDofJointTest joint;

  // Set negative spring stiffness - should be clamped to 0
  joint.setSpringStiffness(0, -100.0);
  EXPECT_EQ(joint.getSpringStiffness(0), 0.0);

  // Positive stiffness should work normally
  joint.setSpringStiffness(0, 50.0);
  EXPECT_EQ(joint.getSpringStiffness(0), 50.0);
}

//==============================================================================
// Test that NaN and Inf physics parameters are clamped to zero with a warning.
// This preserves the invariant from the original DART_ASSERT(d >= 0.0) which
// would reject NaN (since NaN >= 0.0 is false).
//==============================================================================
TEST(GenericJoint, NaNDampingClampedToZero)
{
  SingleDofJointTest joint;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();

  // NaN damping should be clamped to 0
  joint.setDampingCoefficient(0, nan);
  EXPECT_EQ(joint.getDampingCoefficient(0), 0.0);

  // +Inf damping should be clamped to 0
  joint.setDampingCoefficient(0, inf);
  EXPECT_EQ(joint.getDampingCoefficient(0), 0.0);

  // -Inf damping should be clamped to 0
  joint.setDampingCoefficient(0, -inf);
  EXPECT_EQ(joint.getDampingCoefficient(0), 0.0);

  // Valid value should still work
  joint.setDampingCoefficient(0, 5.0);
  EXPECT_EQ(joint.getDampingCoefficient(0), 5.0);
}

//==============================================================================
TEST(GenericJoint, NaNDampingVectorClampedToZero)
{
  MultiDofJointTest joint;
  const auto ndofs = static_cast<Eigen::Index>(joint.getNumDofs());
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();

  Eigen::VectorXd dampings = Eigen::VectorXd::Zero(ndofs);
  dampings[0] = nan;
  dampings[1] = 5.0;
  dampings[2] = inf;
  dampings[3] = -inf;
  dampings[4] = 3.0;
  dampings[5] = -1.0;

  joint.setDampingCoefficients(dampings);

  EXPECT_EQ(joint.getDampingCoefficient(0), 0.0); // Was NaN
  EXPECT_EQ(joint.getDampingCoefficient(1), 5.0);
  EXPECT_EQ(joint.getDampingCoefficient(2), 0.0); // Was +Inf
  EXPECT_EQ(joint.getDampingCoefficient(3), 0.0); // Was -Inf
  EXPECT_EQ(joint.getDampingCoefficient(4), 3.0);
  EXPECT_EQ(joint.getDampingCoefficient(5), 0.0); // Was -1.0
}

//==============================================================================
TEST(GenericJoint, NaNFrictionClampedToZero)
{
  SingleDofJointTest joint;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();

  joint.setCoulombFriction(0, nan);
  EXPECT_EQ(joint.getCoulombFriction(0), 0.0);

  joint.setCoulombFriction(0, inf);
  EXPECT_EQ(joint.getCoulombFriction(0), 0.0);

  joint.setCoulombFriction(0, -inf);
  EXPECT_EQ(joint.getCoulombFriction(0), 0.0);
}

//==============================================================================
TEST(GenericJoint, NaNSpringStiffnessClampedToZero)
{
  SingleDofJointTest joint;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();

  joint.setSpringStiffness(0, nan);
  EXPECT_EQ(joint.getSpringStiffness(0), 0.0);

  joint.setSpringStiffness(0, inf);
  EXPECT_EQ(joint.getSpringStiffness(0), 0.0);

  joint.setSpringStiffness(0, -inf);
  EXPECT_EQ(joint.getSpringStiffness(0), 0.0);
}

namespace {

template <typename JointType>
SkeletonPtr createSkeletonWithJoint(const std::string& name)
{
  auto skel = Skeleton::create(name);

  BodyNode::Properties bodyProps;
  bodyProps.mName = name + "_body";
  bodyProps.mInertia.setMass(1.0);

  typename JointType::Properties jointProps;
  jointProps.mName = name + "_joint";

  skel->createJointAndBodyNodePair<JointType>(nullptr, jointProps, bodyProps);

  return skel;
}

template <typename JointType>
JointType* getJoint(const SkeletonPtr& skel)
{
  return static_cast<JointType*>(skel->getJoint(0));
}

} // namespace

TEST(FreeJoint, SpatialSettersAndConversions)
{
  auto skel = createSkeletonWithJoint<FreeJoint>("free_joint_coverage");
  auto* joint = getJoint<FreeJoint>(skel);

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);

  Eigen::Vector6d positions = FreeJoint::convertToPositions(tf);
  Eigen::Isometry3d tfBack = FreeJoint::convertToTransform(positions);
  EXPECT_TRUE(tfBack.translation().isApprox(tf.translation(), 1e-10));

  joint->setRelativeTransform(tf);
  EXPECT_TRUE(joint->getRelativeTransform().translation().isApprox(
      tf.translation(), 1e-10));

  Eigen::Vector6d relVel = Eigen::Vector6d::Zero();
  relVel << 0.1, -0.2, 0.3, 0.4, -0.5, 0.6;
  joint->setRelativeSpatialVelocity(relVel);
  EXPECT_TRUE(joint->getRelativeSpatialVelocity().isApprox(relVel, 1e-10));

  Eigen::Vector6d relAcc = Eigen::Vector6d::Zero();
  relAcc << -0.3, 0.2, -0.1, 0.6, 0.2, -0.4;
  joint->setRelativeSpatialAcceleration(relAcc);
  EXPECT_TRUE(joint->getRelativeSpatialAcceleration().isApprox(relAcc, 1e-10));

  Eigen::Vector6d worldVel = Eigen::Vector6d::Zero();
  worldVel << 0.2, 0.1, -0.2, 0.0, 0.3, -0.1;
  joint->setSpatialVelocity(worldVel, Frame::World(), Frame::World());
  EXPECT_TRUE(joint->getVelocities().allFinite());

  Eigen::Vector6d worldAcc = Eigen::Vector6d::Zero();
  worldAcc << 0.1, -0.1, 0.2, -0.3, 0.4, -0.2;
  joint->setSpatialAcceleration(worldAcc, Frame::World(), Frame::World());
  EXPECT_TRUE(joint->getAccelerations().allFinite());
}

TEST(TemplatedJacobianNode, JacobianOffsetsAndFrames)
{
  auto skel = createSkeletonWithJoint<RevoluteJoint>("jacobian_node");
  auto* joint = getJoint<RevoluteJoint>(skel);
  auto* body = skel->getBodyNode(0);

  joint->setPosition(0, 0.2);
  joint->setVelocity(0, 0.1);

  const Eigen::Vector3d offset(0.1, -0.2, 0.3);

  const auto J_local = body->getJacobian(body);
  const auto J_world = body->getJacobian(Frame::World());
  const auto J_offset = body->getJacobian(offset);
  const auto J_offset_world = body->getJacobian(offset, Frame::World());
  const auto J_world_offset = body->getWorldJacobian(offset);

  EXPECT_EQ(J_local.rows(), 6);
  EXPECT_EQ(J_world.rows(), 6);
  EXPECT_EQ(J_offset.rows(), 6);
  EXPECT_EQ(J_offset_world.rows(), 6);
  EXPECT_EQ(J_world_offset.rows(), 6);

  EXPECT_EQ(J_local.cols(), 1);
  EXPECT_TRUE(J_world.allFinite());
  EXPECT_TRUE(J_offset.allFinite());
  EXPECT_TRUE(J_offset_world.allFinite());
  EXPECT_TRUE(J_world_offset.allFinite());

  const auto J_linear = body->getLinearJacobian(Frame::World());
  const auto J_angular = body->getAngularJacobian(Frame::World());
  const auto J_linear_offset = body->getLinearJacobian(offset, Frame::World());

  EXPECT_EQ(J_linear.rows(), 3);
  EXPECT_EQ(J_angular.rows(), 3);
  EXPECT_EQ(J_linear_offset.rows(), 3);
  EXPECT_TRUE(J_linear.allFinite());
  EXPECT_TRUE(J_angular.allFinite());
  EXPECT_TRUE(J_linear_offset.allFinite());

  const auto J_spatial = body->getJacobianSpatialDeriv(Frame::World());
  const auto J_classic = body->getJacobianClassicDeriv(offset, Frame::World());

  EXPECT_EQ(J_spatial.rows(), 6);
  EXPECT_EQ(J_classic.rows(), 6);
  EXPECT_TRUE(J_spatial.allFinite());
  EXPECT_TRUE(J_classic.allFinite());
}

//==============================================================================
TEST(GenericJoint, DofAccessorsAndNaming)
{
  auto skeleton = Skeleton::create("generic_dof_accessors");
  auto pair = skeleton->createJointAndBodyNodePair<SingleDofJointTest>();
  auto* joint = pair.first;

  EXPECT_EQ(joint->getNumDofs(), 1u);
  EXPECT_NE(joint->getDof(0), nullptr);

  const auto* constJoint = joint;
  EXPECT_NE(constJoint->getDof(0), nullptr);

  EXPECT_EQ(joint->getIndexInSkeleton(0), 0u);
  EXPECT_EQ(joint->getIndexInTree(0), 0u);

  EXPECT_FALSE(joint->isDofNamePreserved(0));
  joint->preserveDofName(0, true);
  EXPECT_TRUE(joint->isDofNamePreserved(0));
  joint->preserveDofName(0, false);
  EXPECT_FALSE(joint->isDofNamePreserved(0));

  const std::string name = "custom_dof";
  joint->setDofName(0, name, false);
  EXPECT_EQ(joint->getDofName(0), name);

  joint->preserveDofName(0, true);
  EXPECT_TRUE(joint->isDofNamePreserved(0));
}

//==============================================================================
TEST(GenericJoint, CommandAndStateAccessors)
{
  MultiDofJointTest joint;
  const auto ndofs = static_cast<Eigen::Index>(joint.getNumDofs());

  Eigen::VectorXd commands
      = Eigen::VectorXd::LinSpaced(ndofs, 0.1, 0.1 * ndofs);
  joint.setCommands(commands);
  EXPECT_TRUE(joint.getCommands().isApprox(commands));

  joint.setCommand(0, -0.2);
  EXPECT_NEAR(joint.getCommand(0), -0.2, 1e-12);

  joint.resetCommands();
  EXPECT_TRUE(joint.getCommands().isZero());

  Eigen::VectorXd positions = Eigen::VectorXd::Constant(ndofs, 0.3);
  joint.setPositions(positions);
  EXPECT_TRUE(joint.getPositions().isApprox(positions));
  joint.setPosition(0, -0.4);
  EXPECT_NEAR(joint.getPosition(0), -0.4, 1e-12);

  Eigen::VectorXd velocities = Eigen::VectorXd::Constant(ndofs, 0.1);
  joint.setVelocities(velocities);
  EXPECT_TRUE(joint.getVelocities().isApprox(velocities));
  joint.resetVelocities();
  EXPECT_TRUE(joint.getVelocities().isZero());

  Eigen::VectorXd accelerations = Eigen::VectorXd::Constant(ndofs, -0.05);
  joint.setAccelerations(accelerations);
  EXPECT_TRUE(joint.getAccelerations().isApprox(accelerations));
  joint.resetAccelerations();
  EXPECT_TRUE(joint.getAccelerations().isZero());

  Eigen::VectorXd forces = Eigen::VectorXd::LinSpaced(ndofs, 0.2, 0.2 * ndofs);
  joint.setForces(forces);
  EXPECT_TRUE(joint.getForces().isApprox(forces));
  joint.resetForces();
  EXPECT_TRUE(joint.getForces().isZero());

  joint.setInitialPosition(0, 0.7);
  EXPECT_NEAR(joint.getInitialPosition(0), 0.7, 1e-12);

  Eigen::VectorXd initialVel = Eigen::VectorXd::Constant(ndofs, 0.15);
  joint.setInitialVelocities(initialVel);
  EXPECT_TRUE(joint.getInitialVelocities().isApprox(initialVel));
}

//==============================================================================
TEST(GenericJoint, VelocityChangesAndImpulses)
{
  MultiDofJointTest joint;

  joint.setVelocityChange(0, 0.5);
  EXPECT_NEAR(joint.getVelocityChange(0), 0.5, 1e-12);
  joint.resetVelocityChanges();
  EXPECT_NEAR(joint.getVelocityChange(0), 0.0, 1e-12);

  joint.setConstraintImpulse(0, 1.2);
  EXPECT_NEAR(joint.getConstraintImpulse(0), 1.2, 1e-12);
  joint.resetConstraintImpulses();
  EXPECT_NEAR(joint.getConstraintImpulse(0), 0.0, 1e-12);
}

//==============================================================================
TEST(GenericJoint, IntegratePositionsAndDifferences)
{
  SingleDofJointTest joint;

  joint.setPosition(0, 0.0);
  joint.setVelocity(0, 1.0);
  joint.integratePositions(0.1);
  EXPECT_NEAR(joint.getPosition(0), 0.1, 1e-12);

  Eigen::VectorXd q0(1);
  q0 << 0.2;
  Eigen::VectorXd v(1);
  v << -0.5;
  Eigen::VectorXd result;
  joint.integratePositions(q0, v, 0.2, result);
  ASSERT_EQ(result.size(), 1);
  EXPECT_NEAR(result[0], 0.1, 1e-12);

  Eigen::VectorXd q1(1);
  q1 << 0.3;
  Eigen::VectorXd q2(1);
  q2 << -0.1;
  auto diff = joint.getPositionDifferences(q2, q1);
  EXPECT_NEAR(diff[0], -0.4, 1e-12);

  SingleDofJointTest::Vector q1Static;
  SingleDofJointTest::Vector q2Static;
  q1Static[0] = q1[0];
  q2Static[0] = q2[0];
  auto diffStatic = joint.getPositionDifferencesStatic(q2Static, q1Static);
  EXPECT_NEAR(diffStatic[0], -0.4, 1e-12);
}

//==============================================================================
TEST(GenericJoint, RestPositionsAndCoefficients)
{
  MultiDofJointTest joint;
  const auto ndofs = static_cast<Eigen::Index>(joint.getNumDofs());

  Eigen::VectorXd rest = Eigen::VectorXd::LinSpaced(ndofs, 0.0, 0.2 * ndofs);
  joint.setRestPositions(rest);
  EXPECT_TRUE(joint.getRestPositions().isApprox(rest));

  Eigen::VectorXd damping = Eigen::VectorXd::Constant(ndofs, 0.25);
  joint.setDampingCoefficients(damping);
  EXPECT_TRUE(joint.getDampingCoefficients().isApprox(damping));

  Eigen::VectorXd frictions = Eigen::VectorXd::Constant(ndofs, 0.15);
  joint.setFrictions(frictions);
  EXPECT_TRUE(joint.getFrictions().isApprox(frictions));
}

//==============================================================================
TEST(FreeJoint, CoordinateChartConversions)
{
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.linear() = (Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitX())
                 * Eigen::AngleAxisd(-0.2, Eigen::Vector3d::UnitY())
                 * Eigen::AngleAxisd(0.15, Eigen::Vector3d::UnitZ()))
                    .toRotationMatrix();
  tf.translation() = Eigen::Vector3d(0.5, -0.3, 0.8);

  auto posXYZ = FreeJoint::convertToPositions(
      tf, FreeJoint::CoordinateChart::EULER_XYZ);
  auto tfXYZ = FreeJoint::convertToTransform(
      posXYZ, FreeJoint::CoordinateChart::EULER_XYZ);
  EXPECT_TRUE(tfXYZ.translation().isApprox(tf.translation(), 1e-10));
  EXPECT_TRUE(tfXYZ.linear().isApprox(tf.linear(), 1e-10));

  auto posZYX = FreeJoint::convertToPositions(
      tf, FreeJoint::CoordinateChart::EULER_ZYX);
  auto tfZYX = FreeJoint::convertToTransform(
      posZYX, FreeJoint::CoordinateChart::EULER_ZYX);
  EXPECT_TRUE(tfZYX.translation().isApprox(tf.translation(), 1e-10));
  EXPECT_TRUE(tfZYX.linear().isApprox(tf.linear(), 1e-10));
}

//==============================================================================
TEST(FreeJoint, SetTransformOfSkeletonRoots)
{
  auto skeleton = Skeleton::create("multi_root_free");
  auto rootA = skeleton->createJointAndBodyNodePair<FreeJoint>();
  auto rootB = skeleton->createJointAndBodyNodePair<FreeJoint>();

  EXPECT_EQ(skeleton->getNumTrees(), 2u);

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(0.2, -0.1, 0.4);

  FreeJoint::setTransformOf(skeleton.get(), tf, Frame::World(), true);

  EXPECT_TRUE(rootA.second->getWorldTransform().translation().isApprox(
      tf.translation(), 1e-10));
  EXPECT_TRUE(rootB.second->getWorldTransform().translation().isApprox(
      tf.translation(), 1e-10));
}

//==============================================================================
TEST(FreeJoint, SpatialMotionBatchSetter)
{
  auto skeleton = Skeleton::create("spatial_motion");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  auto* joint = pair.first;

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(-0.2, 0.4, 0.1);
  Eigen::Vector6d spatialVel;
  spatialVel << 0.1, -0.2, 0.3, 0.4, -0.5, 0.2;
  Eigen::Vector6d spatialAcc;
  spatialAcc << -0.3, 0.2, 0.1, -0.4, 0.5, -0.2;

  joint->setSpatialMotion(
      &tf,
      Frame::World(),
      &spatialVel,
      Frame::World(),
      Frame::World(),
      &spatialAcc,
      Frame::World(),
      Frame::World());

  EXPECT_TRUE(joint->getRelativeTransform().translation().isApprox(
      tf.translation(), 1e-10));
  EXPECT_TRUE(joint->getVelocities().allFinite());
  EXPECT_TRUE(joint->getAccelerations().allFinite());
}

//==============================================================================
TEST(GenericJoint, SingleDofLimitAccessors)
{
  auto skel = createSkeletonWithJoint<RevoluteJoint>("limit_revolute");
  auto* joint = getJoint<RevoluteJoint>(skel);

  joint->setPositionLowerLimit(0, -1.2);
  joint->setPositionUpperLimit(0, 1.3);
  EXPECT_DOUBLE_EQ(joint->getPositionLowerLimit(0), -1.2);
  EXPECT_DOUBLE_EQ(joint->getPositionUpperLimit(0), 1.3);

  joint->setVelocityLowerLimit(0, -2.0);
  joint->setVelocityUpperLimit(0, 2.0);
  EXPECT_DOUBLE_EQ(joint->getVelocityLowerLimit(0), -2.0);
  EXPECT_DOUBLE_EQ(joint->getVelocityUpperLimit(0), 2.0);

  joint->setAccelerationLowerLimit(0, -3.0);
  joint->setAccelerationUpperLimit(0, 3.0);
  EXPECT_DOUBLE_EQ(joint->getAccelerationLowerLimit(0), -3.0);
  EXPECT_DOUBLE_EQ(joint->getAccelerationUpperLimit(0), 3.0);

  joint->setForceLowerLimit(0, -4.0);
  joint->setForceUpperLimit(0, 4.0);
  EXPECT_DOUBLE_EQ(joint->getForceLowerLimit(0), -4.0);
  EXPECT_DOUBLE_EQ(joint->getForceUpperLimit(0), 4.0);

  joint->setPosition(0, 0.25);
  EXPECT_NEAR(joint->getPosition(0), 0.25, 1e-12);
}

//==============================================================================
TEST(GenericJoint, SingleDofVectorLimitsAndInitials)
{
  auto skel = createSkeletonWithJoint<PrismaticJoint>("limit_prismatic");
  auto* joint = getJoint<PrismaticJoint>(skel);

  Eigen::VectorXd posLower(1);
  Eigen::VectorXd posUpper(1);
  posLower << -0.5;
  posUpper << 0.5;
  joint->setPositionLowerLimits(posLower);
  joint->setPositionUpperLimits(posUpper);
  EXPECT_TRUE(joint->getPositionLowerLimits().isApprox(posLower));
  EXPECT_TRUE(joint->getPositionUpperLimits().isApprox(posUpper));

  Eigen::VectorXd velLower(1);
  Eigen::VectorXd velUpper(1);
  velLower << -1.0;
  velUpper << 1.0;
  joint->setVelocityLowerLimits(velLower);
  joint->setVelocityUpperLimits(velUpper);
  EXPECT_TRUE(joint->getVelocityLowerLimits().isApprox(velLower));
  EXPECT_TRUE(joint->getVelocityUpperLimits().isApprox(velUpper));

  Eigen::VectorXd accLower(1);
  Eigen::VectorXd accUpper(1);
  accLower << -2.0;
  accUpper << 2.0;
  joint->setAccelerationLowerLimits(accLower);
  joint->setAccelerationUpperLimits(accUpper);
  EXPECT_TRUE(joint->getAccelerationLowerLimits().isApprox(accLower));
  EXPECT_TRUE(joint->getAccelerationUpperLimits().isApprox(accUpper));

  Eigen::VectorXd forceLower(1);
  Eigen::VectorXd forceUpper(1);
  forceLower << -3.5;
  forceUpper << 3.5;
  joint->setForceLowerLimits(forceLower);
  joint->setForceUpperLimits(forceUpper);
  EXPECT_TRUE(joint->getForceLowerLimits().isApprox(forceLower));
  EXPECT_TRUE(joint->getForceUpperLimits().isApprox(forceUpper));

  Eigen::VectorXd initialPos(1);
  Eigen::VectorXd initialVel(1);
  initialPos << 0.15;
  initialVel << -0.2;
  joint->setInitialPositions(initialPos);
  joint->setInitialVelocities(initialVel);
  EXPECT_TRUE(joint->getInitialPositions().isApprox(initialPos));
  EXPECT_TRUE(joint->getInitialVelocities().isApprox(initialVel));
}

//==============================================================================
TEST(GenericJoint, MultiDofVectorLimitsAndState)
{
  auto skel = createSkeletonWithJoint<FreeJoint>("limit_free");
  auto* joint = getJoint<FreeJoint>(skel);
  const auto ndofs = static_cast<Eigen::Index>(joint->getNumDofs());

  Eigen::VectorXd posLower = Eigen::VectorXd::Constant(ndofs, -0.4);
  Eigen::VectorXd posUpper = Eigen::VectorXd::Constant(ndofs, 0.6);
  joint->setPositionLowerLimits(posLower);
  joint->setPositionUpperLimits(posUpper);
  EXPECT_TRUE(joint->getPositionLowerLimits().isApprox(posLower));
  EXPECT_TRUE(joint->getPositionUpperLimits().isApprox(posUpper));
  EXPECT_TRUE(joint->hasPositionLimit(0));

  Eigen::VectorXd velLower = Eigen::VectorXd::Constant(ndofs, -1.2);
  Eigen::VectorXd velUpper = Eigen::VectorXd::Constant(ndofs, 1.2);
  joint->setVelocityLowerLimits(velLower);
  joint->setVelocityUpperLimits(velUpper);
  EXPECT_TRUE(joint->getVelocityLowerLimits().isApprox(velLower));
  EXPECT_TRUE(joint->getVelocityUpperLimits().isApprox(velUpper));

  Eigen::VectorXd accLower = Eigen::VectorXd::Constant(ndofs, -2.5);
  Eigen::VectorXd accUpper = Eigen::VectorXd::Constant(ndofs, 2.5);
  joint->setAccelerationLowerLimits(accLower);
  joint->setAccelerationUpperLimits(accUpper);
  EXPECT_TRUE(joint->getAccelerationLowerLimits().isApprox(accLower));
  EXPECT_TRUE(joint->getAccelerationUpperLimits().isApprox(accUpper));

  Eigen::VectorXd forceLower = Eigen::VectorXd::Constant(ndofs, -5.0);
  Eigen::VectorXd forceUpper = Eigen::VectorXd::Constant(ndofs, 5.0);
  joint->setForceLowerLimits(forceLower);
  joint->setForceUpperLimits(forceUpper);
  EXPECT_TRUE(joint->getForceLowerLimits().isApprox(forceLower));
  EXPECT_TRUE(joint->getForceUpperLimits().isApprox(forceUpper));

  Eigen::VectorXd positions = Eigen::VectorXd::LinSpaced(ndofs, -0.2, 0.3);
  Eigen::VectorXd velocities = Eigen::VectorXd::LinSpaced(ndofs, 0.1, 0.6);
  Eigen::VectorXd accelerations = Eigen::VectorXd::LinSpaced(ndofs, -0.4, 0.2);
  Eigen::VectorXd forces = Eigen::VectorXd::LinSpaced(ndofs, 0.2, 1.2);
  Eigen::VectorXd commands = Eigen::VectorXd::LinSpaced(ndofs, -0.8, 0.4);

  joint->setPositions(positions);
  joint->setVelocities(velocities);
  joint->setAccelerations(accelerations);
  joint->setForces(forces);
  joint->setCommands(commands);

  EXPECT_TRUE(joint->getPositions().isApprox(positions));
  EXPECT_TRUE(joint->getVelocities().isApprox(velocities));
  EXPECT_TRUE(joint->getAccelerations().isApprox(accelerations));
  EXPECT_TRUE(joint->getForces().isApprox(forces));
  EXPECT_TRUE(joint->getCommands().isApprox(commands));
}

//==============================================================================
TEST(GenericJoint, SpringDampingFrictionSingleDof)
{
  auto skel = createSkeletonWithJoint<PrismaticJoint>("spring_prismatic");
  auto* joint = getJoint<PrismaticJoint>(skel);

  joint->setSpringStiffness(0, 2.5);
  joint->setRestPosition(0, 0.1);
  joint->setDampingCoefficient(0, 1.1);
  joint->setCoulombFriction(0, 0.2);

  EXPECT_DOUBLE_EQ(joint->getSpringStiffness(0), 2.5);
  EXPECT_DOUBLE_EQ(joint->getRestPosition(0), 0.1);
  EXPECT_DOUBLE_EQ(joint->getDampingCoefficient(0), 1.1);
  EXPECT_DOUBLE_EQ(joint->getCoulombFriction(0), 0.2);

  Eigen::VectorXd rest(1);
  Eigen::VectorXd damping(1);
  Eigen::VectorXd friction(1);
  rest << -0.2;
  damping << 0.8;
  friction << 0.3;
  joint->setRestPositions(rest);
  joint->setDampingCoefficients(damping);
  joint->setFrictions(friction);
  EXPECT_TRUE(joint->getRestPositions().isApprox(rest));
  EXPECT_TRUE(joint->getDampingCoefficients().isApprox(damping));
  EXPECT_TRUE(joint->getFrictions().isApprox(friction));

  joint->setPosition(0, 0.5);
  joint->setSpringStiffness(0, 2.0);
  joint->setRestPosition(0, 0.1);
  const double expected = 0.5 * 2.0 * (0.4 * 0.4);
  EXPECT_NEAR(joint->computePotentialEnergy(), expected, 1e-12);
}

//==============================================================================
TEST(GenericJoint, CommandClippingForceActuator)
{
  auto skel = createSkeletonWithJoint<RevoluteJoint>("force_command");
  auto* joint = getJoint<RevoluteJoint>(skel);

  joint->setActuatorType(Joint::FORCE);
  joint->setForceLowerLimit(0, -1.0);
  joint->setForceUpperLimit(0, 1.0);

  joint->setCommand(0, 2.0);
  EXPECT_DOUBLE_EQ(joint->getCommand(0), 1.0);

  joint->setCommand(0, -2.0);
  EXPECT_DOUBLE_EQ(joint->getCommand(0), -1.0);

  joint->setCommand(0, 0.5);
  EXPECT_DOUBLE_EQ(joint->getCommand(0), 0.5);
}

//==============================================================================
TEST(GenericJoint, CommandClippingVelocityActuator)
{
  auto skel = createSkeletonWithJoint<RevoluteJoint>("velocity_command");
  auto* joint = getJoint<RevoluteJoint>(skel);

  joint->setActuatorType(Joint::VELOCITY);
  joint->setVelocityLowerLimit(0, -0.5);
  joint->setVelocityUpperLimit(0, 0.5);

  joint->setCommand(0, 1.5);
  EXPECT_DOUBLE_EQ(joint->getCommand(0), 0.5);

  joint->setVelocity(0, -0.2);
  EXPECT_DOUBLE_EQ(joint->getCommand(0), -0.2);
}

//==============================================================================
TEST(GenericJoint, CommandClippingAccelerationActuator)
{
  auto skel = createSkeletonWithJoint<RevoluteJoint>("accel_command");
  auto* joint = getJoint<RevoluteJoint>(skel);

  joint->setActuatorType(Joint::ACCELERATION);
  joint->setAccelerationLowerLimit(0, -0.3);
  joint->setAccelerationUpperLimit(0, 0.3);

  joint->setCommand(0, 1.0);
  EXPECT_DOUBLE_EQ(joint->getCommand(0), 0.3);

  joint->setAcceleration(0, -0.1);
  EXPECT_DOUBLE_EQ(joint->getCommand(0), -0.1);
}

//==============================================================================
TEST(GenericJoint, NonFiniteCommandIgnored)
{
  auto skel = createSkeletonWithJoint<RevoluteJoint>("nonfinite_command");
  auto* joint = getJoint<RevoluteJoint>(skel);

  joint->setActuatorType(Joint::FORCE);
  joint->setForceLowerLimit(0, -1.0);
  joint->setForceUpperLimit(0, 1.0);
  joint->setCommand(0, 0.4);

  const double saved = joint->getCommand(0);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();

  joint->setCommand(0, nan);
  EXPECT_DOUBLE_EQ(joint->getCommand(0), saved);

  joint->setCommand(0, inf);
  EXPECT_DOUBLE_EQ(joint->getCommand(0), saved);
}

//==============================================================================
TEST(GenericJoint, ScrewJointLimitClamping)
{
  auto skel = createSkeletonWithJoint<ScrewJoint>("screw_limit");
  auto* joint = getJoint<ScrewJoint>(skel);

  joint->setPositionLowerLimit(0, -0.2);
  joint->setPositionUpperLimit(0, 0.2);
  joint->setVelocityLowerLimit(0, -0.5);
  joint->setVelocityUpperLimit(0, 0.5);
  joint->setAccelerationLowerLimit(0, -1.0);
  joint->setAccelerationUpperLimit(0, 1.0);
  joint->setForceLowerLimit(0, -2.0);
  joint->setForceUpperLimit(0, 2.0);

  joint->setActuatorType(Joint::VELOCITY);
  joint->setCommand(0, 1.5);
  EXPECT_DOUBLE_EQ(joint->getCommand(0), 0.5);

  joint->setActuatorType(Joint::FORCE);
  joint->setCommand(0, -3.0);
  EXPECT_DOUBLE_EQ(joint->getCommand(0), -2.0);
}

//==============================================================================
TEST(GenericJoint, CommandHandlingPassiveLockedAndMimic)
{
  auto skel = createSkeletonWithJoint<RevoluteJoint>("passive_locked_mimic");
  auto* joint = getJoint<RevoluteJoint>(skel);

  joint->setActuatorType(Joint::PASSIVE);
  joint->setCommand(0, 0.75);
  EXPECT_DOUBLE_EQ(joint->getCommand(0), 0.0);

  joint->setActuatorType(Joint::LOCKED);
  joint->setCommand(0, -0.5);
  EXPECT_DOUBLE_EQ(joint->getCommand(0), 0.0);

  joint->setActuatorType(Joint::MIMIC);
  joint->setCommand(0, 0.9);
  EXPECT_DOUBLE_EQ(joint->getCommand(0), 0.0);
}

//==============================================================================
TEST(Joint, ActuatorTypeOverrides)
{
  auto skel = createSkeletonWithJoint<FreeJoint>("actuator_overrides");
  auto* joint = getJoint<FreeJoint>(skel);

  joint->setActuatorType(Joint::FORCE);
  joint->setActuatorType(2, Joint::MIMIC);
  EXPECT_EQ(joint->getActuatorType(2), Joint::MIMIC);
  EXPECT_TRUE(joint->hasActuatorType(Joint::MIMIC));

  auto types = joint->getActuatorTypes();
  ASSERT_EQ(types.size(), joint->getNumDofs());
  EXPECT_EQ(types[2], Joint::MIMIC);

  joint->setActuatorType(2, Joint::FORCE);
  EXPECT_EQ(joint->getActuatorType(2), Joint::FORCE);

  std::vector<Joint::ActuatorType> newTypes(joint->getNumDofs(), Joint::FORCE);
  newTypes[4] = Joint::MIMIC;
  joint->setActuatorTypes(newTypes);
  EXPECT_EQ(joint->getActuatorType(4), Joint::MIMIC);
}

//==============================================================================
TEST(Joint, TransformSetters)
{
  auto skel = createSkeletonWithJoint<RevoluteJoint>("transform_setters");
  auto* joint = getJoint<RevoluteJoint>(skel);

  Eigen::Isometry3d parentTf = Eigen::Isometry3d::Identity();
  parentTf.linear()
      = Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitX()).toRotationMatrix();
  parentTf.translation() = Eigen::Vector3d(0.1, -0.2, 0.3);
  joint->setTransformFromParentBodyNode(parentTf);
  EXPECT_TRUE(joint->getTransformFromParentBodyNode().isApprox(parentTf));

  Eigen::Isometry3d childTf = Eigen::Isometry3d::Identity();
  childTf.linear()
      = Eigen::AngleAxisd(-0.1, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  childTf.translation() = Eigen::Vector3d(-0.3, 0.2, -0.1);
  joint->setTransformFromChildBodyNode(childTf);
  EXPECT_TRUE(joint->getTransformFromChildBodyNode().isApprox(childTf));
}

//==============================================================================
TEST(Joint, CheckSanityLimits)
{
  auto skel = createSkeletonWithJoint<RevoluteJoint>("check_sanity");
  auto* joint = getJoint<RevoluteJoint>(skel);

  joint->setPositionLowerLimit(0, -0.5);
  joint->setPositionUpperLimit(0, 0.5);
  joint->setVelocityLowerLimit(0, -1.0);
  joint->setVelocityUpperLimit(0, 1.0);

  joint->setInitialPosition(0, 1.0);
  joint->setInitialVelocity(0, 2.0);
  EXPECT_FALSE(joint->checkSanity(false));

  joint->setInitialPosition(0, 0.0);
  joint->setInitialVelocity(0, 0.0);
  EXPECT_TRUE(joint->checkSanity(false));
  EXPECT_TRUE(joint->checkSanity(true));
}

//==============================================================================
TEST(ZeroDofJoint, AccessorsAndEmptyState)
{
  auto skeleton = Skeleton::create("zero_dof");
  auto pair = skeleton->createJointAndBodyNodePair<WeldJoint>();
  auto* joint = pair.first;
  auto* body = pair.second;

  EXPECT_EQ(joint->getNumDofs(), 0u);
  EXPECT_EQ(joint->getCommands().size(), 0);
  EXPECT_EQ(joint->getPositions().size(), 0);
  EXPECT_EQ(joint->getVelocities().size(), 0);
  EXPECT_EQ(joint->getAccelerations().size(), 0);
  EXPECT_EQ(joint->getForces().size(), 0);

  EXPECT_EQ(joint->getPositionLowerLimits().size(), 0);
  EXPECT_EQ(joint->getPositionUpperLimits().size(), 0);
  EXPECT_EQ(joint->getVelocityLowerLimits().size(), 0);
  EXPECT_EQ(joint->getVelocityUpperLimits().size(), 0);
  EXPECT_EQ(joint->getAccelerationLowerLimits().size(), 0);
  EXPECT_EQ(joint->getAccelerationUpperLimits().size(), 0);
  EXPECT_EQ(joint->getForceLowerLimits().size(), 0);
  EXPECT_EQ(joint->getForceUpperLimits().size(), 0);

  joint->resetPositions();
  joint->resetVelocities();
  joint->resetAccelerations();
  joint->resetCommands();
  joint->resetForces();

  Eigen::VectorXd q0(0);
  Eigen::VectorXd v(0);
  Eigen::VectorXd result;
  joint->integratePositions(q0, v, 0.1, result);
  EXPECT_EQ(result.size(), 0);

  EXPECT_NEAR(joint->computePotentialEnergy(), 0.0, 1e-12);
  EXPECT_TRUE(
      joint->getBodyConstraintWrench().isApprox(body->getBodyForce(), 1e-12));
}

//==============================================================================
TEST(GenericJoint, BallJointVectorLimitsAndInitials)
{
  auto skel = createSkeletonWithJoint<BallJoint>("ball_vector_limits");
  auto* joint = getJoint<BallJoint>(skel);
  const auto ndofs = static_cast<Eigen::Index>(joint->getNumDofs());

  Eigen::VectorXd posLower = Eigen::VectorXd::Constant(ndofs, -0.4);
  Eigen::VectorXd posUpper = Eigen::VectorXd::Constant(ndofs, 0.5);
  joint->setPositionLowerLimits(posLower);
  joint->setPositionUpperLimits(posUpper);
  EXPECT_TRUE(joint->getPositionLowerLimits().isApprox(posLower));
  EXPECT_TRUE(joint->getPositionUpperLimits().isApprox(posUpper));

  Eigen::VectorXd velLower = Eigen::VectorXd::Constant(ndofs, -1.1);
  Eigen::VectorXd velUpper = Eigen::VectorXd::Constant(ndofs, 1.2);
  joint->setVelocityLowerLimits(velLower);
  joint->setVelocityUpperLimits(velUpper);
  EXPECT_TRUE(joint->getVelocityLowerLimits().isApprox(velLower));
  EXPECT_TRUE(joint->getVelocityUpperLimits().isApprox(velUpper));

  Eigen::VectorXd accLower = Eigen::VectorXd::Constant(ndofs, -2.2);
  Eigen::VectorXd accUpper = Eigen::VectorXd::Constant(ndofs, 2.3);
  joint->setAccelerationLowerLimits(accLower);
  joint->setAccelerationUpperLimits(accUpper);
  EXPECT_TRUE(joint->getAccelerationLowerLimits().isApprox(accLower));
  EXPECT_TRUE(joint->getAccelerationUpperLimits().isApprox(accUpper));

  Eigen::VectorXd forceLower = Eigen::VectorXd::Constant(ndofs, -4.0);
  Eigen::VectorXd forceUpper = Eigen::VectorXd::Constant(ndofs, 4.0);
  joint->setForceLowerLimits(forceLower);
  joint->setForceUpperLimits(forceUpper);
  EXPECT_TRUE(joint->getForceLowerLimits().isApprox(forceLower));
  EXPECT_TRUE(joint->getForceUpperLimits().isApprox(forceUpper));

  Eigen::VectorXd initialPos = Eigen::VectorXd::LinSpaced(ndofs, -0.1, 0.2);
  Eigen::VectorXd initialVel = Eigen::VectorXd::LinSpaced(ndofs, 0.05, 0.15);
  joint->setInitialPositions(initialPos);
  joint->setInitialVelocities(initialVel);
  EXPECT_TRUE(joint->getInitialPositions().isApprox(initialPos));
  EXPECT_TRUE(joint->getInitialVelocities().isApprox(initialVel));

  Eigen::VectorXd rest = Eigen::VectorXd::LinSpaced(ndofs, -0.2, 0.2);
  Eigen::VectorXd damping = Eigen::VectorXd::Constant(ndofs, 0.4);
  Eigen::VectorXd frictions = Eigen::VectorXd::Constant(ndofs, 0.3);
  joint->setRestPositions(rest);
  joint->setDampingCoefficients(damping);
  joint->setFrictions(frictions);
  EXPECT_TRUE(joint->getRestPositions().isApprox(rest));
  EXPECT_TRUE(joint->getDampingCoefficients().isApprox(damping));
  EXPECT_TRUE(joint->getFrictions().isApprox(frictions));
}

//==============================================================================
TEST(GenericJoint, EulerJointVectorCommandsAndStates)
{
  auto skel = createSkeletonWithJoint<EulerJoint>("euler_vector_states");
  auto* joint = getJoint<EulerJoint>(skel);
  const auto ndofs = static_cast<Eigen::Index>(joint->getNumDofs());

  Eigen::VectorXd positions = Eigen::VectorXd::LinSpaced(ndofs, -0.2, 0.4);
  Eigen::VectorXd velocities = Eigen::VectorXd::LinSpaced(ndofs, 0.1, 0.3);
  Eigen::VectorXd accelerations = Eigen::VectorXd::LinSpaced(ndofs, -0.3, 0.2);
  Eigen::VectorXd commands = Eigen::VectorXd::LinSpaced(ndofs, -0.5, 0.5);

  Eigen::VectorXd forceLower = Eigen::VectorXd::Constant(ndofs, -2.0);
  Eigen::VectorXd forceUpper = Eigen::VectorXd::Constant(ndofs, 2.0);
  joint->setForceLowerLimits(forceLower);
  joint->setForceUpperLimits(forceUpper);

  joint->setPositions(positions);
  joint->setVelocities(velocities);
  joint->setAccelerations(accelerations);
  joint->setCommands(commands);

  EXPECT_TRUE(joint->getPositions().isApprox(positions));
  EXPECT_TRUE(joint->getVelocities().isApprox(velocities));
  EXPECT_TRUE(joint->getAccelerations().isApprox(accelerations));
  EXPECT_TRUE(joint->getCommands().isApprox(commands));
}

//==============================================================================
TEST(GenericJoint, PlanarJointVectorLimitsAndResets)
{
  auto skel = createSkeletonWithJoint<PlanarJoint>("planar_vector_limits");
  auto* joint = getJoint<PlanarJoint>(skel);
  const auto ndofs = static_cast<Eigen::Index>(joint->getNumDofs());

  Eigen::VectorXd posLower = Eigen::VectorXd::Constant(ndofs, -0.7);
  Eigen::VectorXd posUpper = Eigen::VectorXd::Constant(ndofs, 0.9);
  joint->setPositionLowerLimits(posLower);
  joint->setPositionUpperLimits(posUpper);
  EXPECT_TRUE(joint->getPositionLowerLimits().isApprox(posLower));
  EXPECT_TRUE(joint->getPositionUpperLimits().isApprox(posUpper));

  Eigen::VectorXd initialPos = Eigen::VectorXd::LinSpaced(ndofs, -0.2, 0.2);
  Eigen::VectorXd initialVel = Eigen::VectorXd::LinSpaced(ndofs, 0.1, 0.2);
  joint->setInitialPositions(initialPos);
  joint->setInitialVelocities(initialVel);
  joint->resetPositions();
  joint->resetVelocities();
  EXPECT_TRUE(joint->getPositions().isApprox(initialPos));
  EXPECT_TRUE(joint->getVelocities().isApprox(initialVel));
}

//==============================================================================
TEST(GenericJoint, UniversalJointVectorSetters)
{
  auto skel = createSkeletonWithJoint<UniversalJoint>("universal_vector");
  auto* joint = getJoint<UniversalJoint>(skel);
  const auto ndofs = static_cast<Eigen::Index>(joint->getNumDofs());

  Eigen::VectorXd rest = Eigen::VectorXd::Constant(ndofs, 0.05);
  Eigen::VectorXd damping = Eigen::VectorXd::Constant(ndofs, 0.2);
  Eigen::VectorXd frictions = Eigen::VectorXd::Constant(ndofs, 0.1);
  joint->setRestPositions(rest);
  joint->setDampingCoefficients(damping);
  joint->setFrictions(frictions);
  EXPECT_TRUE(joint->getRestPositions().isApprox(rest));
  EXPECT_TRUE(joint->getDampingCoefficients().isApprox(damping));
  EXPECT_TRUE(joint->getFrictions().isApprox(frictions));

  Eigen::VectorXd accLower = Eigen::VectorXd::Constant(ndofs, -1.0);
  Eigen::VectorXd accUpper = Eigen::VectorXd::Constant(ndofs, 1.0);
  joint->setAccelerationLowerLimits(accLower);
  joint->setAccelerationUpperLimits(accUpper);
  EXPECT_TRUE(joint->getAccelerationLowerLimits().isApprox(accLower));
  EXPECT_TRUE(joint->getAccelerationUpperLimits().isApprox(accUpper));
}

//==============================================================================
TEST(GenericJoint, BallJointSpatialStateUpdates)
{
  auto skel = createSkeletonWithJoint<BallJoint>("ball_spatial_state");
  auto* joint = getJoint<BallJoint>(skel);
  auto* body = skel->getBodyNode(0);

  Eigen::Vector3d velocities(0.1, -0.2, 0.3);
  Eigen::Vector3d accelerations(-0.05, 0.2, -0.1);
  joint->setVelocities(velocities);
  joint->setAccelerations(accelerations);

  EXPECT_TRUE(joint->getRelativeSpatialVelocity().allFinite());
  EXPECT_TRUE(joint->getRelativeSpatialAcceleration().allFinite());
  EXPECT_TRUE(joint->getRelativePrimaryAcceleration().allFinite());

  const auto wrench = joint->getBodyConstraintWrench();
  EXPECT_TRUE(wrench.allFinite());
  EXPECT_TRUE(wrench.isApprox(body->getBodyForce(), 1e-12));
}

//==============================================================================
TEST(GenericJoint, WorldStepContactAndIntegration)
{
  auto world = simulation::World::create();
  world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));

  auto ground = Skeleton::create("generic_ground");
  auto groundPair = ground->createJointAndBodyNodePair<WeldJoint>();
  auto groundShape = std::make_shared<BoxShape>(Eigen::Vector3d(5.0, 5.0, 0.1));
  groundPair.second->createShapeNodeWith<VisualAspect, CollisionAspect>(
      groundShape);
  Eigen::Isometry3d groundTf = Eigen::Isometry3d::Identity();
  groundTf.translation() = Eigen::Vector3d(0.0, 0.0, -0.05);
  groundPair.first->setTransformFromParentBodyNode(groundTf);

  auto dynamic = Skeleton::create("generic_dynamic");
  auto rootPair = dynamic->createJointAndBodyNodePair<FreeJoint>();
  rootPair.second->setMass(1.0);
  auto box = std::make_shared<BoxShape>(Eigen::Vector3d(0.2, 0.2, 0.2));
  rootPair.second->createShapeNodeWith<VisualAspect, CollisionAspect>(box);
  Eigen::Isometry3d startTf = Eigen::Isometry3d::Identity();
  startTf.translation() = Eigen::Vector3d(0.0, 0.0, 0.0);
  rootPair.first->setPositions(FreeJoint::convertToPositions(startTf));

  auto ballPair = rootPair.second->createChildJointAndBodyNodePair<BallJoint>();
  ballPair.second->setMass(0.2);
  auto universalPair
      = ballPair.second->createChildJointAndBodyNodePair<UniversalJoint>();
  universalPair.first->setAxis1(Eigen::Vector3d::UnitX());
  universalPair.first->setAxis2(Eigen::Vector3d::UnitY());
  universalPair.second->setMass(0.2);
  auto eulerPair
      = universalPair.second->createChildJointAndBodyNodePair<EulerJoint>();
  eulerPair.second->setMass(0.2);
  auto planarPair
      = eulerPair.second->createChildJointAndBodyNodePair<PlanarJoint>();
  planarPair.second->setMass(0.2);
  auto screwPair
      = planarPair.second->createChildJointAndBodyNodePair<ScrewJoint>();
  screwPair.first->setAxis(Eigen::Vector3d::UnitZ());
  screwPair.first->setPitch(0.1);
  screwPair.second->setMass(0.2);
  auto transPair
      = screwPair.second->createChildJointAndBodyNodePair<TranslationalJoint>();
  transPair.second->setMass(0.2);

  ballPair.first->setVelocities(Eigen::Vector3d(0.1, -0.2, 0.05));
  ballPair.first->setAccelerations(Eigen::Vector3d(0.2, 0.1, -0.1));
  ballPair.first->setVelocityChange(0, 0.1);
  ballPair.first->setConstraintImpulse(0, 0.2);

  dynamic->setVelocities(
      Eigen::VectorXd::Constant(dynamic->getNumDofs(), 0.05));
  dynamic->setAccelerations(
      Eigen::VectorXd::Constant(dynamic->getNumDofs(), -0.02));
  dynamic->integrateVelocities(0.01);
  dynamic->integratePositions(0.01);

  rootPair.first->getRelativeJacobianTimeDeriv();

  world->addSkeleton(ground);
  world->addSkeleton(dynamic);

  for (int i = 0; i < 5; ++i) {
    world->step();
  }

  EXPECT_TRUE(dynamic->getPositions().allFinite());
}

//==============================================================================
TEST(GenericJoints, KinematicActuatorConstraintPaths)
{
  auto world = simulation::World::create();
  world->setTimeStep(0.001);
  world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));

  // Create a skeleton with a velocity-actuated revolute joint
  auto skel = dynamics::Skeleton::create("kinematic");
  auto pair = skel->createJointAndBodyNodePair<dynamics::RevoluteJoint>();
  auto* joint = pair.first;
  auto* body = pair.second;
  joint->setActuatorType(dynamics::Joint::VELOCITY);
  body->setMass(1.0);
  body->createShapeNodeWith<dynamics::VisualAspect, dynamics::CollisionAspect>(
      std::make_shared<dynamics::BoxShape>(Eigen::Vector3d(0.2, 0.2, 0.2)));

  // Create a ground box
  auto ground = dynamics::Skeleton::create("ground");
  auto gPair = ground->createJointAndBodyNodePair<dynamics::WeldJoint>();
  auto* gBody = gPair.second;
  gBody->setMass(1.0);
  gBody->createShapeNodeWith<dynamics::VisualAspect, dynamics::CollisionAspect>(
      std::make_shared<dynamics::BoxShape>(Eigen::Vector3d(10.0, 10.0, 0.1)));
  Eigen::Isometry3d gTf = Eigen::Isometry3d::Identity();
  gTf.translation() = Eigen::Vector3d(0.0, 0.0, -0.15);
  gPair.first->setTransformFromParentBodyNode(gTf);

  world->addSkeleton(skel);
  world->addSkeleton(ground);

  for (int i = 0; i < 10; ++i) {
    world->step();
  }
  EXPECT_TRUE(joint->getPositions().allFinite());
  EXPECT_TRUE(joint->getVelocities().allFinite());

  // Also test LOCKED actuator type
  joint->setActuatorType(dynamics::Joint::LOCKED);
  for (int i = 0; i < 5; ++i) {
    world->step();
  }
  EXPECT_TRUE(joint->getPositions().allFinite());
}

//==============================================================================
TEST(GenericJoints, BodyConstraintWrench)
{
  auto world = simulation::World::create();
  world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));

  auto skel = dynamics::Skeleton::create("wrench_test");
  auto pair = skel->createJointAndBodyNodePair<dynamics::RevoluteJoint>();
  auto* body = pair.second;
  body->setMass(1.0);
  body->createShapeNodeWith<dynamics::VisualAspect, dynamics::CollisionAspect>(
      std::make_shared<dynamics::BoxShape>(Eigen::Vector3d(0.2, 0.2, 0.2)));

  world->addSkeleton(skel);
  world->step();

  Eigen::Vector6d wrench = pair.first->getBodyConstraintWrench();
  EXPECT_TRUE(wrench.allFinite());
}

#if GTEST_HAS_DEATH_TEST
//==============================================================================
TEST(GenericJoint, OutOfRangeDofAccessorsDeath)
{
  #ifdef NDEBUG
  GTEST_SKIP() << "Assertions are disabled in Release builds.";
  #endif

  SingleDofJointTest joint;

  EXPECT_DEATH({ joint.getDof(1); }, "");
  EXPECT_DEATH({ joint.getDofName(1); }, "");
  EXPECT_DEATH({ joint.setDofName(1, "bad", false); }, "");
  EXPECT_DEATH({ joint.preserveDofName(1, true); }, "");
  EXPECT_DEATH({ joint.isDofNamePreserved(1); }, "");
  EXPECT_DEATH({ joint.getIndexInSkeleton(1); }, "");
  EXPECT_DEATH({ joint.getIndexInTree(1); }, "");
}

//==============================================================================
TEST(GenericJoint, OutOfRangeStateAccessorsDeath)
{
  #ifdef NDEBUG
  GTEST_SKIP() << "Assertions are disabled in Release builds.";
  #endif

  SingleDofJointTest joint;

  EXPECT_DEATH({ joint.setPosition(1, 0.2); }, "");
  EXPECT_DEATH({ joint.getPosition(1); }, "");
  EXPECT_DEATH({ joint.setVelocity(1, 0.3); }, "");
  EXPECT_DEATH({ joint.getVelocity(1); }, "");
  EXPECT_DEATH({ joint.setAcceleration(1, -0.1); }, "");
  EXPECT_DEATH({ joint.getAcceleration(1); }, "");
  EXPECT_DEATH({ joint.setForce(1, 0.4); }, "");
  EXPECT_DEATH({ joint.getForce(1); }, "");
  EXPECT_DEATH({ joint.setCommand(1, 0.5); }, "");
  EXPECT_DEATH({ joint.getCommand(1); }, "");
}

//==============================================================================
TEST(GenericJoint, OutOfRangeLimitAccessorsDeath)
{
  #ifdef NDEBUG
  GTEST_SKIP() << "Assertions are disabled in Release builds.";
  #endif

  SingleDofJointTest joint;

  EXPECT_DEATH({ joint.setPositionLowerLimit(1, -0.5); }, "");
  EXPECT_DEATH({ joint.getPositionLowerLimit(1); }, "");
  EXPECT_DEATH({ joint.setPositionUpperLimit(1, 0.5); }, "");
  EXPECT_DEATH({ joint.getPositionUpperLimit(1); }, "");
  EXPECT_DEATH({ joint.setVelocityLowerLimit(1, -0.5); }, "");
  EXPECT_DEATH({ joint.getVelocityLowerLimit(1); }, "");
  EXPECT_DEATH({ joint.setVelocityUpperLimit(1, 0.5); }, "");
  EXPECT_DEATH({ joint.getVelocityUpperLimit(1); }, "");
  EXPECT_DEATH({ joint.setAccelerationLowerLimit(1, -0.5); }, "");
  EXPECT_DEATH({ joint.getAccelerationLowerLimit(1); }, "");
  EXPECT_DEATH({ joint.setAccelerationUpperLimit(1, 0.5); }, "");
  EXPECT_DEATH({ joint.getAccelerationUpperLimit(1); }, "");
  EXPECT_DEATH({ joint.setForceLowerLimit(1, -0.5); }, "");
  EXPECT_DEATH({ joint.getForceLowerLimit(1); }, "");
  EXPECT_DEATH({ joint.setForceUpperLimit(1, 0.5); }, "");
  EXPECT_DEATH({ joint.getForceUpperLimit(1); }, "");
}

//==============================================================================
TEST(GenericJoint, CommandSizeMismatchDeath)
{
  #ifdef NDEBUG
  GTEST_SKIP() << "Assertions are disabled in Release builds.";
  #endif

  auto skel = createSkeletonWithJoint<UniversalJoint>("command_mismatch");
  auto* joint = getJoint<UniversalJoint>(skel);

  Eigen::VectorXd bad = Eigen::VectorXd::Zero(3);
  EXPECT_DEATH({ joint->setCommands(bad); }, "");
}

//==============================================================================
TEST(GenericJoint, StateVectorSizeMismatchDeath)
{
  #ifdef NDEBUG
  GTEST_SKIP() << "Assertions are disabled in Release builds.";
  #endif

  auto skel = createSkeletonWithJoint<EulerJoint>("state_mismatch");
  auto* joint = getJoint<EulerJoint>(skel);

  Eigen::VectorXd bad = Eigen::VectorXd::Zero(2);
  EXPECT_DEATH({ joint->setPositions(bad); }, "");
  EXPECT_DEATH({ joint->setVelocities(bad); }, "");
  EXPECT_DEATH({ joint->setAccelerations(bad); }, "");
  EXPECT_DEATH({ joint->setForces(bad); }, "");
}

//==============================================================================
TEST(GenericJoint, LimitVectorSizeMismatchDeath)
{
  #ifdef NDEBUG
  GTEST_SKIP() << "Assertions are disabled in Release builds.";
  #endif

  auto skel = createSkeletonWithJoint<BallJoint>("limit_mismatch");
  auto* joint = getJoint<BallJoint>(skel);

  Eigen::VectorXd bad = Eigen::VectorXd::Zero(2);
  EXPECT_DEATH({ joint->setPositionLowerLimits(bad); }, "");
  EXPECT_DEATH({ joint->setPositionUpperLimits(bad); }, "");
  EXPECT_DEATH({ joint->setVelocityLowerLimits(bad); }, "");
  EXPECT_DEATH({ joint->setVelocityUpperLimits(bad); }, "");
  EXPECT_DEATH({ joint->setAccelerationLowerLimits(bad); }, "");
  EXPECT_DEATH({ joint->setAccelerationUpperLimits(bad); }, "");
  EXPECT_DEATH({ joint->setForceLowerLimits(bad); }, "");
  EXPECT_DEATH({ joint->setForceUpperLimits(bad); }, "");
}

//==============================================================================
TEST(GenericJoint, InitialVectorSizeMismatchDeath)
{
  #ifdef NDEBUG
  GTEST_SKIP() << "Assertions are disabled in Release builds.";
  #endif

  auto skel = createSkeletonWithJoint<PlanarJoint>("initial_mismatch");
  auto* joint = getJoint<PlanarJoint>(skel);

  Eigen::VectorXd bad = Eigen::VectorXd::Zero(2);
  EXPECT_DEATH({ joint->setInitialPositions(bad); }, "");
  EXPECT_DEATH({ joint->setInitialVelocities(bad); }, "");
  EXPECT_DEATH({ joint->setRestPositions(bad); }, "");
  EXPECT_DEATH({ joint->setDampingCoefficients(bad); }, "");
  EXPECT_DEATH({ joint->setFrictions(bad); }, "");
}
#endif
