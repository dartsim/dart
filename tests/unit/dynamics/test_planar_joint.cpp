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
#include <dart/dynamics/planar_joint.hpp>
#include <dart/dynamics/skeleton.hpp>

#include <gtest/gtest.h>

#include <cmath>

using namespace dart;
using namespace dart::dynamics;

namespace {

SkeletonPtr createSkeletonWithPlanarJoint(const std::string& name)
{
  auto skel = Skeleton::create(name);

  skel->createJointAndBodyNodePair<PlanarJoint>(
      nullptr, PlanarJoint::Properties(), BodyNode::AspectProperties("body"));

  return skel;
}

} // namespace

TEST(PlanarJointTest, DefaultPlaneType)
{
  auto skel = createSkeletonWithPlanarJoint("default_planar");
  auto* joint = static_cast<PlanarJoint*>(skel->getJoint(0));

  EXPECT_EQ(joint->getPlaneType(), PlanarJoint::PlaneType::XY);
}

TEST(PlanarJointTest, SetXYPlane)
{
  auto skel = createSkeletonWithPlanarJoint("xy_skel");
  auto* joint = static_cast<PlanarJoint*>(skel->getJoint(0));

  joint->setXYPlane();
  EXPECT_EQ(joint->getPlaneType(), PlanarJoint::PlaneType::XY);

  Eigen::Vector3d rotAxis = joint->getRotationalAxis();
  EXPECT_TRUE(rotAxis.isApprox(Eigen::Vector3d::UnitZ()));
}

TEST(PlanarJointTest, SetYZPlane)
{
  auto skel = createSkeletonWithPlanarJoint("yz_skel");
  auto* joint = static_cast<PlanarJoint*>(skel->getJoint(0));

  joint->setYZPlane();
  EXPECT_EQ(joint->getPlaneType(), PlanarJoint::PlaneType::YZ);

  Eigen::Vector3d rotAxis = joint->getRotationalAxis();
  EXPECT_TRUE(rotAxis.isApprox(Eigen::Vector3d::UnitX()));
}

TEST(PlanarJointTest, SetZXPlane)
{
  auto skel = createSkeletonWithPlanarJoint("zx_skel");
  auto* joint = static_cast<PlanarJoint*>(skel->getJoint(0));

  joint->setZXPlane();
  EXPECT_EQ(joint->getPlaneType(), PlanarJoint::PlaneType::ZX);

  Eigen::Vector3d rotAxis = joint->getRotationalAxis();
  EXPECT_TRUE(rotAxis.isApprox(Eigen::Vector3d::UnitY()));
}

TEST(PlanarJointTest, SetArbitraryPlane)
{
  auto skel = createSkeletonWithPlanarJoint("arb_skel");
  auto* joint = static_cast<PlanarJoint*>(skel->getJoint(0));

  Eigen::Vector3d axis1 = Eigen::Vector3d::UnitX();
  Eigen::Vector3d axis2 = Eigen::Vector3d::UnitY();

  joint->setArbitraryPlane(axis1, axis2);
  EXPECT_EQ(joint->getPlaneType(), PlanarJoint::PlaneType::ARBITRARY);

  EXPECT_TRUE(joint->getTranslationalAxis1().isApprox(axis1));
  EXPECT_TRUE(joint->getTranslationalAxis2().isApprox(axis2));
}

TEST(PlanarJointTest, GetTranslationalAxes)
{
  auto skel = createSkeletonWithPlanarJoint("axes_skel");
  auto* joint = static_cast<PlanarJoint*>(skel->getJoint(0));

  joint->setXYPlane();

  Eigen::Vector3d axis1 = joint->getTranslationalAxis1();
  Eigen::Vector3d axis2 = joint->getTranslationalAxis2();

  EXPECT_TRUE(axis1.isApprox(Eigen::Vector3d::UnitX()));
  EXPECT_TRUE(axis2.isApprox(Eigen::Vector3d::UnitY()));
}

TEST(PlanarJointTest, ConvertToPositions)
{
  Eigen::Isometry2d tf = Eigen::Isometry2d::Identity();
  tf.translate(Eigen::Vector2d(1.0, 2.0));
  tf.rotate(0.5);

  Eigen::Vector3d positions = PlanarJoint::convertToPositions(tf);

  EXPECT_NEAR(positions[0], 1.0, 1e-10);
  EXPECT_NEAR(positions[1], 2.0, 1e-10);
  EXPECT_NEAR(positions[2], 0.5, 1e-10);
}

TEST(PlanarJointTest, ConvertToTransform)
{
  Eigen::Vector3d positions(1.5, 2.5, 0.3);
  Eigen::Isometry2d tf = PlanarJoint::convertToTransform(positions);

  EXPECT_NEAR(tf.translation()[0], 1.5, 1e-10);
  EXPECT_NEAR(tf.translation()[1], 2.5, 1e-10);
}

TEST(PlanarJointTest, IsCyclic)
{
  auto skel = createSkeletonWithPlanarJoint("cyclic_skel");
  auto* joint = static_cast<PlanarJoint*>(skel->getJoint(0));

  EXPECT_FALSE(joint->isCyclic(0));
  EXPECT_FALSE(joint->isCyclic(1));
  EXPECT_TRUE(joint->isCyclic(2));
}

TEST(PlanarJointTest, GetType)
{
  auto skel = createSkeletonWithPlanarJoint("type_skel");
  auto* joint = static_cast<PlanarJoint*>(skel->getJoint(0));

  EXPECT_EQ(joint->getType(), PlanarJoint::getStaticType());
  EXPECT_EQ(joint->getType(), "PlanarJoint");
}

TEST(PlanarJointTest, CopyProperties)
{
  auto skel1 = createSkeletonWithPlanarJoint("copy_src");
  auto* joint1 = static_cast<PlanarJoint*>(skel1->getJoint(0));
  joint1->setYZPlane();

  auto skel2 = createSkeletonWithPlanarJoint("copy_dst");
  auto* joint2 = static_cast<PlanarJoint*>(skel2->getJoint(0));

  joint2->copy(joint1);

  EXPECT_EQ(joint2->getPlaneType(), PlanarJoint::PlaneType::YZ);
}

TEST(PlanarJointTest, NumDofs)
{
  auto skel = createSkeletonWithPlanarJoint("dof_skel");
  auto* joint = static_cast<PlanarJoint*>(skel->getJoint(0));

  EXPECT_EQ(joint->getNumDofs(), 3u);
}

TEST(PlanarJointTest, SetPositions)
{
  auto skel = createSkeletonWithPlanarJoint("pos_skel");
  auto* joint = static_cast<PlanarJoint*>(skel->getJoint(0));

  Eigen::Vector3d positions(1.0, 2.0, 0.5);
  joint->setPositions(positions);

  Eigen::Vector3d retrieved = joint->getPositions();
  EXPECT_TRUE(retrieved.isApprox(positions));
}

TEST(PlanarJointTest, GetRelativeJacobianStatic)
{
  auto skel = createSkeletonWithPlanarJoint("jac_skel");
  auto* joint = static_cast<PlanarJoint*>(skel->getJoint(0));

  Eigen::Vector3d positions(0.1, 0.2, 0.3);
  Eigen::Matrix<double, 6, 3> jacobian
      = joint->getRelativeJacobianStatic(positions);

  EXPECT_TRUE(jacobian.allFinite());
  EXPECT_EQ(jacobian.rows(), 6);
  EXPECT_EQ(jacobian.cols(), 3);
}
