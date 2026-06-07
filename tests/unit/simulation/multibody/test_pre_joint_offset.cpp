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

#include <dart/simulation/multibody/joint.hpp>
#include <dart/simulation/multibody/link.hpp>
#include <dart/simulation/multibody/multibody.hpp>
#include <dart/simulation/world.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gtest/gtest.h>

namespace sx = dart::simulation;

namespace {

Eigen::Isometry3d translation(double x, double y, double z)
{
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.translation() = Eigen::Vector3d(x, y, z);
  return t;
}

// Build a fixed-base single revolute pendulum (axis Y) with the given pre-joint
// and post-joint offsets and a COM-offset link.
sx::Link addPendulum(
    sx::Multibody& multibody,
    const sx::Link& base,
    const std::string& name,
    const Eigen::Isometry3d& transformToParent,
    const Eigen::Isometry3d& transformFromParent)
{
  sx::JointSpec spec;
  spec.name = name + "_joint";
  spec.type = sx::JointType::Revolute;
  spec.axis = Eigen::Vector3d::UnitY();
  spec.transformToParent = transformToParent;
  spec.transformFromParent = transformFromParent;
  sx::Link link = multibody.addLink(name, base, spec);
  Eigen::Matrix3d inertia = Eigen::Matrix3d::Zero();
  inertia.diagonal() = Eigen::Vector3d(0.05, 0.04, 0.02);
  link.setMass(2.0);
  link.setInertia(inertia);
  link.setCenterOfMass(Eigen::Vector3d(0.0, 0.0, -0.5));
  return link;
}

} // namespace

//==============================================================================
// Forward kinematics applies the pre-joint offset:
// world transform = transformToParent * jointMotion(q) * transformFromParent.
TEST(PreJointOffset, ForwardKinematicsAppliesPreOffset)
{
  sx::World world;
  sx::Multibody multibody = world.addMultibody("arm");
  sx::Link base = multibody.addLink("base");
  base.setMass(1.0);
  base.setInertia(Eigen::Matrix3d::Identity());

  const Eigen::Isometry3d toParent = translation(0.3, 0.0, 0.2);
  const Eigen::Isometry3d fromParent = translation(0.0, 0.0, -0.4);
  sx::Link link = addPendulum(multibody, base, "link", toParent, fromParent);

  const double q = 0.6;
  world.enterSimulationMode();
  link.getParentJoint().setPosition(Eigen::VectorXd::Constant(1, q));
  world.updateKinematics();

  Eigen::Isometry3d expected = Eigen::Isometry3d::Identity();
  expected.linear()
      = Eigen::AngleAxisd(q, Eigen::Vector3d::UnitY()).toRotationMatrix();
  const Eigen::Isometry3d worldTransform = toParent * expected * fromParent;

  EXPECT_LE(
      (link.getWorldTransform().matrix() - worldTransform.matrix())
          .cwiseAbs()
          .maxCoeff(),
      1e-9);
}

//==============================================================================
// A pure-translation pre-joint offset rigidly relocates a fixed-base pendulum:
// its joint-space mass matrix and gravity forces are unchanged, while the link
// world transform shifts by exactly the offset.
TEST(PreJointOffset, FixedBaseTranslationLeavesDynamicsInvariant)
{
  const Eigen::Vector3d gravity(0.0, 0.0, -9.81);
  const Eigen::Isometry3d fromParent = translation(0.0, 0.0, -0.4);
  const Eigen::Isometry3d offset = translation(0.5, -0.2, 0.3);
  const double q = 0.5;

  sx::World plain;
  plain.setGravity(gravity);
  sx::Multibody plainArm = plain.addMultibody("plain");
  sx::Link plainBase = plainArm.addLink("base");
  plainBase.setMass(1.0);
  plainBase.setInertia(Eigen::Matrix3d::Identity());
  sx::Link plainLink = addPendulum(
      plainArm, plainBase, "link", Eigen::Isometry3d::Identity(), fromParent);
  plain.enterSimulationMode();
  plainLink.getParentJoint().setPosition(Eigen::VectorXd::Constant(1, q));
  plain.updateKinematics();

  sx::World shifted;
  shifted.setGravity(gravity);
  sx::Multibody shiftedArm = shifted.addMultibody("shifted");
  sx::Link shiftedBase = shiftedArm.addLink("base");
  shiftedBase.setMass(1.0);
  shiftedBase.setInertia(Eigen::Matrix3d::Identity());
  sx::Link shiftedLink
      = addPendulum(shiftedArm, shiftedBase, "link", offset, fromParent);
  shifted.enterSimulationMode();
  shiftedLink.getParentJoint().setPosition(Eigen::VectorXd::Constant(1, q));
  shifted.updateKinematics();

  // The translation is a rigid relocation off the fixed base, so the
  // joint-space dynamics are identical.
  EXPECT_LE(
      (plainArm.getMassMatrix() - shiftedArm.getMassMatrix())
          .cwiseAbs()
          .maxCoeff(),
      1e-9);
  EXPECT_LE(
      (plainArm.getGravityForces() - shiftedArm.getGravityForces())
          .cwiseAbs()
          .maxCoeff(),
      1e-9);

  // The link world transform is shifted by exactly the offset translation.
  const Eigen::Vector3d delta = shiftedLink.getWorldTransform().translation()
                                - plainLink.getWorldTransform().translation();
  EXPECT_LE((delta - offset.translation()).cwiseAbs().maxCoeff(), 1e-9);
}
