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

// Executor-parity test (PLAN-110 D2): the differentiable opt-in flag must not
// change the forward result. Two identical worlds, one differentiable and one
// not, must produce bitwise-identical [q; q̇] over N steps. Uses only the
// always-compiled WorldOptions core, so it builds and runs in both the
// DART_BUILD_DIFF=ON and OFF configurations.

#include <dart/simulation/multibody/multibody.hpp>
#include <dart/simulation/world.hpp>
#include <dart/simulation/world_options.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include <memory>
#include <sstream>
#include <vector>

namespace sim = dart::simulation;

namespace {

//==============================================================================
// Build a two-link revolute pendulum with the requested differentiable flag and
// a fixed initial configuration / effort.
std::unique_ptr<sim::World> buildPendulum(bool differentiable)
{
  sim::WorldOptions options;
  options.timeStep = 1e-3;
  options.gravity = Eigen::Vector3d(0.0, 0.0, -9.81);
  options.differentiable = differentiable;
  auto world = std::make_unique<sim::World>(options);

  auto pendulum = world->addMultibody("pendulum");
  auto base = pendulum.addLink("base");

  Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
  offset.translation() = Eigen::Vector3d(1.0, 0.0, 0.0);

  auto link1 = pendulum.addLink(
      "link1",
      base,
      sim::JointSpec{
          .name = "hinge1",
          .type = sim::JointType::Revolute,
          .axis = Eigen::Vector3d::UnitY(),
          .transformFromParent = offset});
  link1.setMass(1.5);
  link1.setInertia(0.04 * Eigen::Matrix3d::Identity());

  auto link2 = pendulum.addLink(
      "link2",
      link1,
      sim::JointSpec{
          .name = "hinge2",
          .type = sim::JointType::Revolute,
          .axis = Eigen::Vector3d::UnitY(),
          .transformFromParent = offset});
  link2.setMass(1.0);
  link2.setInertia(0.03 * Eigen::Matrix3d::Identity());

  auto h1 = pendulum.getJoint("hinge1");
  auto h2 = pendulum.getJoint("hinge2");
  h1->setPosition(Eigen::VectorXd::Constant(1, 0.4));
  h1->setVelocity(Eigen::VectorXd::Constant(1, 0.9));
  h1->setForce(Eigen::VectorXd::Constant(1, 0.7));
  h2->setPosition(Eigen::VectorXd::Constant(1, -0.6));
  h2->setVelocity(Eigen::VectorXd::Constant(1, -0.5));
  h2->setForce(Eigen::VectorXd::Constant(1, -0.3));

  return world;
}

//==============================================================================
Eigen::VectorXd readState(const sim::Multibody& mb)
{
  std::vector<double> q;
  std::vector<double> qdot;
  for (const auto& joint : mb.getJoints()) {
    const Eigen::VectorXd p = joint.getPosition();
    const Eigen::VectorXd v = joint.getVelocity();
    for (Eigen::Index i = 0; i < p.size(); ++i) {
      q.push_back(p[i]);
    }
    for (Eigen::Index i = 0; i < v.size(); ++i) {
      qdot.push_back(v[i]);
    }
  }
  Eigen::VectorXd state(q.size() + qdot.size());
  for (std::size_t i = 0; i < q.size(); ++i) {
    state[static_cast<Eigen::Index>(i)] = q[i];
  }
  for (std::size_t i = 0; i < qdot.size(); ++i) {
    state[static_cast<Eigen::Index>(q.size() + i)] = qdot[i];
  }
  return state;
}

} // namespace

//==============================================================================
TEST(DiffZeroCostParity, FlagReflectsConstruction)
{
  auto on = buildPendulum(true);
  auto off = buildPendulum(false);
  EXPECT_TRUE(on->isDifferentiable());
  EXPECT_FALSE(off->isDifferentiable());

  sim::World defaultWorld;
  EXPECT_FALSE(defaultWorld.isDifferentiable());
}

//==============================================================================
TEST(DiffZeroCostParity, SerializationRoundTripsDifferentiableFlag)
{
  auto differentiable = buildPendulum(true);
  std::stringstream onStream;
  differentiable->saveBinary(onStream);

  sim::World loadedOn;
  loadedOn.loadBinary(onStream);
  EXPECT_TRUE(loadedOn.isDifferentiable());

  auto plain = buildPendulum(false);
  std::stringstream offStream;
  plain->saveBinary(offStream);

  sim::World loadedOff;
  loadedOff.loadBinary(offStream);
  EXPECT_FALSE(loadedOff.isDifferentiable());
}

//==============================================================================
TEST(DiffZeroCostParity, ForwardResultIsBitwiseIdentical)
{
  auto differentiable = buildPendulum(true);
  auto plain = buildPendulum(false);

  constexpr std::size_t steps = 50;
  for (std::size_t i = 0; i < steps; ++i) {
    differentiable->step();
    plain->step();
  }

  auto diffMb = differentiable->getMultibody("pendulum");
  auto plainMb = plain->getMultibody("pendulum");
  const Eigen::VectorXd diffState = readState(*diffMb);
  const Eigen::VectorXd plainState = readState(*plainMb);

  ASSERT_EQ(diffState.size(), plainState.size());
  for (Eigen::Index k = 0; k < diffState.size(); ++k) {
    // Bitwise equality: the differentiable flag must not perturb the forward
    // result by even a single ULP.
    EXPECT_EQ(diffState[k], plainState[k]) << "state component " << k;
  }
}
