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

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/euler_joint.hpp>
#include <dart/dynamics/skeleton.hpp>

#include <gtest/gtest.h>

#include <cmath>

using namespace dart;
using namespace dart::dynamics;

namespace {

SkeletonPtr createSkeletonWithEulerJoint(
    const std::string& name, EulerJoint::AxisOrder order)
{
  auto skel = Skeleton::create(name);

  EulerJoint::Properties props;
  props.mAxisOrder = order;

  skel->createJointAndBodyNodePair<EulerJoint>(
      nullptr, props, BodyNode::AspectProperties("body"));

  return skel;
}

} // namespace

TEST(EulerJointTest, DefaultAxisOrder)
{
  auto skel = Skeleton::create("default_euler");
  auto pair = skel->createJointAndBodyNodePair<EulerJoint>(
      nullptr, EulerJoint::Properties(), BodyNode::AspectProperties("body"));

  auto* joint = pair.first;
  EXPECT_EQ(joint->getAxisOrder(), EulerJoint::AxisOrder::XYZ);
}

TEST(EulerJointTest, SetAxisOrderXYZ)
{
  auto skel
      = createSkeletonWithEulerJoint("xyz_skel", EulerJoint::AxisOrder::ZYX);
  auto* joint = static_cast<EulerJoint*>(skel->getJoint(0));

  joint->setAxisOrder(EulerJoint::AxisOrder::XYZ);
  EXPECT_EQ(joint->getAxisOrder(), EulerJoint::AxisOrder::XYZ);
}

TEST(EulerJointTest, SetAxisOrderZYX)
{
  auto skel
      = createSkeletonWithEulerJoint("zyx_skel", EulerJoint::AxisOrder::XYZ);
  auto* joint = static_cast<EulerJoint*>(skel->getJoint(0));

  joint->setAxisOrder(EulerJoint::AxisOrder::ZYX);
  EXPECT_EQ(joint->getAxisOrder(), EulerJoint::AxisOrder::ZYX);
}

TEST(EulerJointTest, ConvertToTransformXYZ)
{
  Eigen::Vector3d angles(0.1, 0.2, 0.3);
  Eigen::Isometry3d tf
      = EulerJoint::convertToTransform(angles, EulerJoint::AxisOrder::XYZ);

  EXPECT_TRUE(tf.matrix().allFinite());
  EXPECT_NEAR(tf.translation().norm(), 0.0, 1e-10);
}

TEST(EulerJointTest, ConvertToTransformZYX)
{
  Eigen::Vector3d angles(0.1, 0.2, 0.3);
  Eigen::Isometry3d tf
      = EulerJoint::convertToTransform(angles, EulerJoint::AxisOrder::ZYX);

  EXPECT_TRUE(tf.matrix().allFinite());
  EXPECT_NEAR(tf.translation().norm(), 0.0, 1e-10);
}

TEST(EulerJointTest, ConvertToRotationXYZ)
{
  Eigen::Vector3d angles(0.5, 0.3, 0.1);
  Eigen::Matrix3d rot
      = EulerJoint::convertToRotation(angles, EulerJoint::AxisOrder::XYZ);

  EXPECT_TRUE(rot.allFinite());
  EXPECT_NEAR(rot.determinant(), 1.0, 1e-10);
}

TEST(EulerJointTest, ConvertToRotationZYX)
{
  Eigen::Vector3d angles(0.5, 0.3, 0.1);
  Eigen::Matrix3d rot
      = EulerJoint::convertToRotation(angles, EulerJoint::AxisOrder::ZYX);

  EXPECT_TRUE(rot.allFinite());
  EXPECT_NEAR(rot.determinant(), 1.0, 1e-10);
}

TEST(EulerJointTest, GetRelativeJacobianStatic)
{
  auto skel
      = createSkeletonWithEulerJoint("jac_skel", EulerJoint::AxisOrder::XYZ);
  auto* joint = static_cast<EulerJoint*>(skel->getJoint(0));

  Eigen::Vector3d positions(0.1, 0.2, 0.3);
  Eigen::Matrix<double, 6, 3> jacobian
      = joint->getRelativeJacobianStatic(positions);

  EXPECT_TRUE(jacobian.allFinite());
  EXPECT_EQ(jacobian.rows(), 6);
  EXPECT_EQ(jacobian.cols(), 3);
}

TEST(EulerJointTest, IsCyclic)
{
  auto skel
      = createSkeletonWithEulerJoint("cyclic_skel", EulerJoint::AxisOrder::XYZ);
  auto* joint = static_cast<EulerJoint*>(skel->getJoint(0));

  EXPECT_TRUE(joint->isCyclic(0));
  EXPECT_TRUE(joint->isCyclic(1));
  EXPECT_TRUE(joint->isCyclic(2));
}

TEST(EulerJointTest, GetType)
{
  auto skel
      = createSkeletonWithEulerJoint("type_skel", EulerJoint::AxisOrder::XYZ);
  auto* joint = static_cast<EulerJoint*>(skel->getJoint(0));

  EXPECT_EQ(joint->getType(), EulerJoint::getStaticType());
  EXPECT_EQ(joint->getType(), "EulerJoint");
}

TEST(EulerJointTest, CopyProperties)
{
  auto skel1
      = createSkeletonWithEulerJoint("copy_src", EulerJoint::AxisOrder::ZYX);
  auto* joint1 = static_cast<EulerJoint*>(skel1->getJoint(0));

  auto skel2
      = createSkeletonWithEulerJoint("copy_dst", EulerJoint::AxisOrder::XYZ);
  auto* joint2 = static_cast<EulerJoint*>(skel2->getJoint(0));

  joint2->copy(joint1);

  EXPECT_EQ(joint2->getAxisOrder(), EulerJoint::AxisOrder::ZYX);
}

TEST(EulerJointTest, NumDofs)
{
  auto skel
      = createSkeletonWithEulerJoint("dof_skel", EulerJoint::AxisOrder::XYZ);
  auto* joint = static_cast<EulerJoint*>(skel->getJoint(0));

  EXPECT_EQ(joint->getNumDofs(), 3u);
}

TEST(EulerJointTest, SetPositions)
{
  auto skel
      = createSkeletonWithEulerJoint("pos_skel", EulerJoint::AxisOrder::XYZ);
  auto* joint = static_cast<EulerJoint*>(skel->getJoint(0));

  Eigen::Vector3d positions(0.5, 1.0, 1.5);
  joint->setPositions(positions);

  Eigen::Vector3d retrieved = joint->getPositions();
  EXPECT_TRUE(retrieved.isApprox(positions));
}
