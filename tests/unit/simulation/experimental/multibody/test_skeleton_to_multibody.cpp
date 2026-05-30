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

#include <dart/simulation/experimental/common/exceptions.hpp>
#include <dart/simulation/experimental/io/skeleton_to_multibody.hpp>
#include <dart/simulation/experimental/multibody/multibody.hpp>
#include <dart/simulation/experimental/world.hpp>

#include <dart/dynamics/ball_joint.hpp>
#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/prismatic_joint.hpp>
#include <dart/dynamics/revolute_joint.hpp>
#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/weld_joint.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gtest/gtest.h>

namespace sx = dart::simulation::experimental;
namespace dd = dart::dynamics;

namespace {

const Eigen::Vector3d kGravity(0.0, 0.0, -9.81);

double maxAbsDiff(const Eigen::MatrixXd& a, const Eigen::MatrixXd& b)
{
  EXPECT_EQ(a.rows(), b.rows());
  EXPECT_EQ(a.cols(), b.cols());
  return (a - b).cwiseAbs().maxCoeff();
}

Eigen::Isometry3d translation(double x, double y, double z)
{
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.translation() = Eigen::Vector3d(x, y, z);
  return t;
}

/// Build a planar double pendulum as a legacy revolute serial chain. The first
/// joint sits at the world origin; the second joint is offset one link-length
/// down the first link. Each link's center of mass is offset from its body
/// origin, exercising the loader's frame re-expression.
dd::SkeletonPtr makeDoublePendulum()
{
  auto skeleton = dd::Skeleton::create("double_pendulum");

  auto [joint1, body1]
      = skeleton->createJointAndBodyNodePair<dd::RevoluteJoint>(nullptr);
  joint1->setName("joint1");
  joint1->setAxis(Eigen::Vector3d::UnitY());
  body1->setName("body1");
  body1->setMass(2.0);
  body1->setMomentOfInertia(0.05, 0.05, 0.02, 0.0, 0.0, 0.0);
  body1->setLocalCOM(Eigen::Vector3d(0.0, 0.0, -0.5));

  auto [joint2, body2]
      = skeleton->createJointAndBodyNodePair<dd::RevoluteJoint>(body1);
  joint2->setName("joint2");
  joint2->setAxis(Eigen::Vector3d::UnitY());
  joint2->setTransformFromParentBodyNode(translation(0.0, 0.0, -1.0));
  body2->setName("body2");
  body2->setMass(1.0);
  body2->setMomentOfInertia(0.03, 0.03, 0.01, 0.0, 0.0, 0.0);
  body2->setLocalCOM(Eigen::Vector3d(0.0, 0.0, -0.5));

  skeleton->setGravity(kGravity);
  return skeleton;
}

} // namespace

//==============================================================================
TEST(SkeletonToMultibody, DoublePendulumStructure)
{
  auto skeleton = makeDoublePendulum();

  sx::World world;
  world.setGravity(kGravity);
  sx::Multibody multibody
      = sx::io::buildMultibodyFromSkeleton(world, *skeleton);

  // One extra link beyond the skeleton's bodies: the synthetic world base.
  EXPECT_EQ(multibody.getLinkCount(), skeleton->getNumBodyNodes() + 1);
  EXPECT_EQ(multibody.getJointCount(), skeleton->getNumBodyNodes());
  EXPECT_EQ(multibody.getDOFCount(), skeleton->getNumDofs());
  EXPECT_EQ(multibody.getDOFCount(), 2u);

  EXPECT_TRUE(multibody.getLink("base").has_value());
  EXPECT_TRUE(multibody.getLink("body1").has_value());
  EXPECT_TRUE(multibody.getLink("body2").has_value());
  EXPECT_TRUE(multibody.getJoint("joint1").has_value());
  EXPECT_TRUE(multibody.getJoint("joint2").has_value());
}

//==============================================================================
// DART 6 parity: the converted multibody reproduces the legacy skeleton's
// joint-space mass matrix, gravity forces, and Coriolis forces at a shared,
// non-trivial configuration and velocity.
TEST(SkeletonToMultibody, DoublePendulumMatchesLegacyDynamics)
{
  auto skeleton = makeDoublePendulum();

  Eigen::VectorXd q(2);
  q << 0.3, -0.5;
  Eigen::VectorXd dq(2);
  dq << 0.7, 0.2;
  skeleton->setPositions(q);
  skeleton->setVelocities(dq);

  const Eigen::MatrixXd legacyMass = skeleton->getMassMatrix();
  const Eigen::VectorXd legacyGravity = skeleton->getGravityForces();
  const Eigen::VectorXd legacyCoriolis = skeleton->getCoriolisForces();

  sx::World world;
  world.setGravity(kGravity);
  sx::Multibody multibody
      = sx::io::buildMultibodyFromSkeleton(world, *skeleton);

  EXPECT_LE(maxAbsDiff(multibody.getMassMatrix(), legacyMass), 1e-9);
  EXPECT_LE(maxAbsDiff(multibody.getGravityForces(), legacyGravity), 1e-9);
  EXPECT_LE(maxAbsDiff(multibody.getCoriolisForces(), legacyCoriolis), 1e-9);
}

//==============================================================================
// A prismatic joint with an offset interior joint reproduces the legacy mass
// matrix and gravity forces.
TEST(SkeletonToMultibody, PrismaticChainMatchesLegacyDynamics)
{
  auto skeleton = dd::Skeleton::create("prismatic_chain");

  auto [joint1, body1]
      = skeleton->createJointAndBodyNodePair<dd::PrismaticJoint>(nullptr);
  joint1->setName("slider");
  joint1->setAxis(Eigen::Vector3d::UnitZ());
  body1->setName("body1");
  body1->setMass(1.5);
  body1->setMomentOfInertia(0.02, 0.02, 0.02, 0.0, 0.0, 0.0);

  auto [joint2, body2]
      = skeleton->createJointAndBodyNodePair<dd::RevoluteJoint>(body1);
  joint2->setName("hinge");
  joint2->setAxis(Eigen::Vector3d::UnitX());
  joint2->setTransformFromParentBodyNode(translation(0.0, 0.0, -0.8));
  body2->setName("body2");
  body2->setMass(0.75);
  body2->setMomentOfInertia(0.01, 0.01, 0.01, 0.0, 0.0, 0.0);
  body2->setLocalCOM(Eigen::Vector3d(0.0, 0.0, -0.3));

  skeleton->setGravity(kGravity);

  Eigen::VectorXd q(2);
  q << 0.2, 0.4;
  skeleton->setPositions(q);

  const Eigen::MatrixXd legacyMass = skeleton->getMassMatrix();
  const Eigen::VectorXd legacyGravity = skeleton->getGravityForces();

  sx::World world;
  world.setGravity(kGravity);
  sx::Multibody multibody
      = sx::io::buildMultibodyFromSkeleton(world, *skeleton);

  EXPECT_EQ(multibody.getDOFCount(), 2u);
  EXPECT_LE(maxAbsDiff(multibody.getMassMatrix(), legacyMass), 1e-9);
  EXPECT_LE(maxAbsDiff(multibody.getGravityForces(), legacyGravity), 1e-9);
}

//==============================================================================
// A welded (fixed) base maps to a fixed joint beneath the synthetic world base
// and the actuated child still reproduces the legacy dynamics. The weld offset
// is non-trivial, exercising the fixed-joint placement path.
TEST(SkeletonToMultibody, FixedBaseChainMatchesLegacyDynamics)
{
  auto skeleton = dd::Skeleton::create("fixed_base_arm");

  Eigen::Isometry3d weldOffset = Eigen::Isometry3d::Identity();
  weldOffset.translation() = Eigen::Vector3d(0.1, 0.0, 0.2);
  weldOffset.linear()
      = Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitX()).toRotationMatrix();

  auto [base, baseBody]
      = skeleton->createJointAndBodyNodePair<dd::WeldJoint>(nullptr);
  base->setName("weld");
  base->setTransformFromParentBodyNode(weldOffset);
  baseBody->setName("base_link");
  baseBody->setMass(3.0);
  baseBody->setMomentOfInertia(0.1, 0.1, 0.1, 0.0, 0.0, 0.0);

  auto [hinge, armBody]
      = skeleton->createJointAndBodyNodePair<dd::RevoluteJoint>(baseBody);
  hinge->setName("hinge");
  hinge->setAxis(Eigen::Vector3d::UnitY());
  hinge->setTransformFromParentBodyNode(translation(0.0, 0.0, -0.4));
  armBody->setName("arm");
  armBody->setMass(1.25);
  armBody->setMomentOfInertia(0.02, 0.02, 0.01, 0.0, 0.0, 0.0);
  armBody->setLocalCOM(Eigen::Vector3d(0.0, 0.0, -0.25));

  skeleton->setGravity(kGravity);
  Eigen::VectorXd q(1);
  q << 0.35;
  skeleton->setPositions(q);

  const Eigen::MatrixXd legacyMass = skeleton->getMassMatrix();
  const Eigen::VectorXd legacyGravity = skeleton->getGravityForces();

  sx::World world;
  world.setGravity(kGravity);
  sx::Multibody multibody
      = sx::io::buildMultibodyFromSkeleton(world, *skeleton);

  EXPECT_EQ(multibody.getDOFCount(), 1u);
  EXPECT_EQ(multibody.getLinkCount(), skeleton->getNumBodyNodes() + 1);
  EXPECT_LE(maxAbsDiff(multibody.getMassMatrix(), legacyMass), 1e-9);
  EXPECT_LE(maxAbsDiff(multibody.getGravityForces(), legacyGravity), 1e-9);
}

//==============================================================================
// Exercises a non-identity frame re-expression. The root joint's parent-side
// transform is a pure rotation, so the motion axis is rotated by M.linear();
// the interior joint's transform also rotates, so the first link's inertia and
// center of mass are re-expressed by a rotation R != I. A non-diagonal inertia
// and an offset center of mass ensure a transpose/sign error cannot hide.
TEST(SkeletonToMultibody, RotatedFramesMatchLegacyDynamics)
{
  auto skeleton = dd::Skeleton::create("rotated");

  Eigen::Isometry3d rootOffset = Eigen::Isometry3d::Identity();
  rootOffset.linear()
      = Eigen::AngleAxisd(0.4, Eigen::Vector3d(1.0, 1.0, 0.0).normalized())
            .toRotationMatrix();

  auto [joint1, body1]
      = skeleton->createJointAndBodyNodePair<dd::RevoluteJoint>(nullptr);
  joint1->setName("joint1");
  joint1->setAxis(Eigen::Vector3d(0.0, 1.0, 1.0).normalized());
  joint1->setTransformFromParentBodyNode(rootOffset);
  body1->setName("body1");
  body1->setMass(2.0);
  // Non-diagonal, non-isotropic (diagonally dominant, hence SPD) inertia.
  body1->setMomentOfInertia(0.08, 0.06, 0.05, 0.01, 0.005, 0.002);
  body1->setLocalCOM(Eigen::Vector3d(0.1, -0.05, -0.3));

  Eigen::Isometry3d interiorOffset = Eigen::Isometry3d::Identity();
  interiorOffset.linear()
      = Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitY()).toRotationMatrix();
  interiorOffset.translation() = Eigen::Vector3d(0.2, 0.0, -0.6);

  // A non-identity child-side transform (C) exercises the C^-1 term of the
  // placement (the child body frame does not coincide with the joint frame).
  Eigen::Isometry3d childTransform = Eigen::Isometry3d::Identity();
  childTransform.linear()
      = Eigen::AngleAxisd(-0.2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  childTransform.translation() = Eigen::Vector3d(0.0, 0.1, -0.05);

  auto [joint2, body2]
      = skeleton->createJointAndBodyNodePair<dd::RevoluteJoint>(body1);
  joint2->setName("joint2");
  joint2->setAxis(Eigen::Vector3d::UnitZ());
  joint2->setTransformFromParentBodyNode(interiorOffset);
  joint2->setTransformFromChildBodyNode(childTransform);
  body2->setName("body2");
  body2->setMass(1.0);
  body2->setMomentOfInertia(0.03, 0.03, 0.02, 0.0, 0.0, 0.0);
  body2->setLocalCOM(Eigen::Vector3d(0.0, 0.0, -0.4));

  skeleton->setGravity(kGravity);
  Eigen::VectorXd q(2);
  q << 0.3, -0.2;
  Eigen::VectorXd dq(2);
  dq << 0.5, -0.4;
  skeleton->setPositions(q);
  skeleton->setVelocities(dq);

  const Eigen::MatrixXd legacyMass = skeleton->getMassMatrix();
  const Eigen::VectorXd legacyGravity = skeleton->getGravityForces();
  const Eigen::VectorXd legacyCoriolis = skeleton->getCoriolisForces();

  sx::World world;
  world.setGravity(kGravity);
  sx::Multibody multibody
      = sx::io::buildMultibodyFromSkeleton(world, *skeleton);

  EXPECT_LE(maxAbsDiff(multibody.getMassMatrix(), legacyMass), 1e-9);
  EXPECT_LE(maxAbsDiff(multibody.getGravityForces(), legacyGravity), 1e-9);
  EXPECT_LE(maxAbsDiff(multibody.getCoriolisForces(), legacyCoriolis), 1e-9);
}

//==============================================================================
// A valid first body followed by an unsupported (ball) joint must reject the
// whole skeleton and leave the World with no partial multibody: the plan pass
// validates everything before any World state is created.
TEST(SkeletonToMultibody, RejectsUnsupportedJointType)
{
  auto skeleton = dd::Skeleton::create("mixed");

  auto [hinge, link]
      = skeleton->createJointAndBodyNodePair<dd::RevoluteJoint>(nullptr);
  hinge->setName("hinge");
  hinge->setAxis(Eigen::Vector3d::UnitY());
  link->setName("link");
  link->setMass(1.0);
  link->setMomentOfInertia(0.01, 0.01, 0.01, 0.0, 0.0, 0.0);

  auto [ball, ballBody]
      = skeleton->createJointAndBodyNodePair<dd::BallJoint>(link);
  ball->setName("ball");
  ballBody->setName("ball_link");
  ballBody->setMass(1.0);
  ballBody->setMomentOfInertia(0.01, 0.01, 0.01, 0.0, 0.0, 0.0);

  sx::World world;
  EXPECT_THROW(
      sx::io::buildMultibodyFromSkeleton(world, *skeleton),
      sx::InvalidOperationException);
  EXPECT_EQ(world.getMultibodyCount(), 0u);
}

//==============================================================================
TEST(SkeletonToMultibody, RejectsOffsetRootJoint)
{
  auto skeleton = dd::Skeleton::create("offset_root");
  auto [joint, body]
      = skeleton->createJointAndBodyNodePair<dd::RevoluteJoint>(nullptr);
  joint->setName("hinge");
  joint->setAxis(Eigen::Vector3d::UnitY());
  // A non-zero parent-side offset on a moving root joint cannot be anchored at
  // the world base origin in this slice.
  joint->setTransformFromParentBodyNode(translation(0.5, 0.0, 0.0));
  body->setName("body");
  body->setMass(1.0);
  body->setMomentOfInertia(0.01, 0.01, 0.01, 0.0, 0.0, 0.0);

  sx::World world;
  EXPECT_THROW(
      sx::io::buildMultibodyFromSkeleton(world, *skeleton),
      sx::InvalidOperationException);
  EXPECT_EQ(world.getMultibodyCount(), 0u);
}
