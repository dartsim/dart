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
