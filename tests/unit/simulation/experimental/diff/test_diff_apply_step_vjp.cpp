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

// PUBLIC-facade correctness gate for the reverse-mode step VJP
// (PLAN-110 WS3 first slice). This test drives the PUBLIC API: it builds a
// BoxedLcp differentiable World, steps once, then asserts that
// World::applyStepVjp(g) equals the explicit transposed-Jacobian products
//   state   = stateJacobianᵀ   · g,
//   control = controlJacobianᵀ · g
// for a random upstream gradient g, using the Jacobians returned by
// World::getStepDerivatives(). It also checks the size-mismatch guard.
// Registered only when DART_BUILD_DIFF is ON.

#include <dart/simulation/experimental/body/collision_shape.hpp>
#include <dart/simulation/experimental/body/contact.hpp>
#include <dart/simulation/experimental/body/rigid_body.hpp>
#include <dart/simulation/experimental/body/rigid_body_options.hpp>
#include <dart/simulation/experimental/common/exceptions.hpp>
#include <dart/simulation/experimental/diff/step_derivatives.hpp>
#include <dart/simulation/experimental/diff/step_gradient.hpp>
#include <dart/simulation/experimental/world.hpp>
#include <dart/simulation/experimental/world_options.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include <memory>
#include <random>

namespace sx = dart::simulation::experimental;

namespace {

constexpr double kSphereRadius = 0.5;
constexpr double kSphereMass = 2.0;
constexpr double kTimeStep = 1e-3;

//==============================================================================
// Build a frictionless sphere-on-static-box-ground scene with the boxed-LCP
// contact solver and the differentiable opt-in (mirrors the public contact
// Jacobian test scene).
std::unique_ptr<sx::World> buildScene()
{
  sx::WorldOptions options;
  options.timeStep = kTimeStep;
  options.gravity = Eigen::Vector3d(0.0, 0.0, -9.81);
  options.differentiable = true;
  options.contactSolverMethod = sx::ContactSolverMethod::BoxedLcp;
  auto world = std::make_unique<sx::World>(options);

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
  auto ground = world->addRigidBody("ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(5.0, 5.0, 0.5)));
  ground.setFriction(0.0);

  sx::RigidBodyOptions sphereOptions;
  sphereOptions.mass = kSphereMass;
  sphereOptions.position = Eigen::Vector3d(0.0, 0.0, kSphereRadius - 5e-5);
  auto sphere = world->addRigidBody("sphere", sphereOptions);
  sphere.setCollisionShape(sx::CollisionShape::makeSphere(kSphereRadius));
  sphere.setFriction(0.0);

  auto handle = world->getRigidBody("sphere");
  handle->setLinearVelocity(Eigen::Vector3d(0.0, 0.0, -0.2));

  return world;
}

} // namespace

//==============================================================================
// applyStepVjp(g) must equal the explicit transposed-Jacobian products of the
// cached step derivatives for a random upstream gradient g.
TEST(DiffApplyStepVjp, MatchesTransposedJacobianProducts)
{
  auto world = buildScene();
  ASSERT_FALSE(world->collide().empty())
      << "expected an active sphere/ground contact";

  world->step();

  const sx::StepDerivatives derivatives = world->getStepDerivatives();
  const Eigen::Index stateSize = derivatives.stateJacobian.rows();
  ASSERT_EQ(stateSize, 6);
  ASSERT_EQ(derivatives.controlJacobian.rows(), stateSize);

  // Deterministic pseudo-random upstream gradient of size 2*ndof.
  std::mt19937 rng(12345);
  std::uniform_real_distribution<double> dist(-1.0, 1.0);
  Eigen::VectorXd g(stateSize);
  for (Eigen::Index i = 0; i < stateSize; ++i) {
    g[i] = dist(rng);
  }

  const sx::StepGradient gradient = world->applyStepVjp(g);

  const Eigen::VectorXd expectedState
      = derivatives.stateJacobian.transpose() * g;
  const Eigen::VectorXd expectedControl
      = derivatives.controlJacobian.transpose() * g;

  ASSERT_EQ(gradient.state.size(), expectedState.size());
  ASSERT_EQ(gradient.control.size(), expectedControl.size());

  EXPECT_LT((gradient.state - expectedState).cwiseAbs().maxCoeff(), 1e-12);
  EXPECT_LT((gradient.control - expectedControl).cwiseAbs().maxCoeff(), 1e-12);

  // The control gradient has the control dimension (ndof = stateSize / 2).
  EXPECT_EQ(gradient.control.size(), stateSize / 2);
}

//==============================================================================
// A wrong-sized upstream gradient is rejected rather than silently producing a
// shape-broken product.
TEST(DiffApplyStepVjp, RejectsWrongSizedGradient)
{
  auto world = buildScene();
  world->step();

  const sx::StepDerivatives derivatives = world->getStepDerivatives();
  Eigen::VectorXd wrong(derivatives.stateJacobian.rows() + 1);
  wrong.setZero();

  EXPECT_THROW(
      static_cast<void>(world->applyStepVjp(wrong)),
      sx::InvalidArgumentException);
}

//==============================================================================
// applyStepVjp inherits getStepDerivatives()'s differentiable gating: a
// non-differentiable World must throw rather than return a zero gradient.
TEST(DiffApplyStepVjp, NonDifferentiableWorldThrows)
{
  sx::WorldOptions options;
  options.timeStep = kTimeStep;
  options.differentiable = false;
  sx::World world(options);

  Eigen::VectorXd g(6);
  g.setZero();
  EXPECT_THROW(
      static_cast<void>(world.applyStepVjp(g)), sx::InvalidOperationException);
}
