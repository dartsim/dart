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

// PUBLIC-facade correctness gate for the contact-aware step Jacobian
// (PLAN-110 WS2 consolidation). Unlike test_diff_contact_jacobian.cpp, which
// calls the detail entry point directly, this test drives the PUBLIC API: it
// builds a BoxedLcp differentiable World, steps once, then reads
// World::getStepDerivatives() and compares it against a finite-difference of
// the full forward step. It asserts:
//   - an in-scope frictionless clamping contact: the public Jacobian carries
//     the contact-aware gradient (matches FD-of-step),
//   - a no-contact scene: the public Jacobian reduces to the contact-free
//     (free-fall) Jacobian (matches FD-of-step and the closed form),
//   - an in-scope multi-point box contact (multiple clamping normal rows whose
//     corner lever arms excite the angular DOFs): the public Jacobian matches
//     FD-of-step,
//   - an out-of-scope multibody/articulated-link contact: the public step()
//     throws rather than returning a wrong (contact-omitting) matrix.
// Registered only when DART_BUILD_DIFF is ON.

#include <dart/simulation/experimental/body/collision_shape.hpp>
#include <dart/simulation/experimental/body/contact.hpp>
#include <dart/simulation/experimental/body/rigid_body.hpp>
#include <dart/simulation/experimental/body/rigid_body_options.hpp>
#include <dart/simulation/experimental/common/exceptions.hpp>
#include <dart/simulation/experimental/diff/step_derivatives.hpp>
#include <dart/simulation/experimental/multibody/link.hpp>
#include <dart/simulation/experimental/multibody/multibody.hpp>
#include <dart/simulation/experimental/world.hpp>
#include <dart/simulation/experimental/world_options.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include <array>
#include <limits>
#include <memory>

#include <cmath>

namespace sx = dart::simulation::experimental;

namespace {

constexpr double kSphereRadius = 0.5;
constexpr double kSphereMass = 2.0;
constexpr double kTimeStep = 1e-3;

//==============================================================================
// Translational rigid-body state: x = [position; linear velocity], control =
// applied force. Angular state is held fixed (the scene keeps the contact lever
// arm parallel to the normal, so the angular DOFs decouple).
struct BodyState
{
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d force = Eigen::Vector3d::Zero();
};

//==============================================================================
// Build a frictionless sphere-on-static-box-ground scene with the boxed-LCP
// contact solver and the differentiable opt-in. The ground top face is at z =
// 0.
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
  sphereOptions.position = Eigen::Vector3d(0.0, 0.0, kSphereRadius);
  auto sphere = world->addRigidBody("sphere", sphereOptions);
  sphere.setCollisionShape(sx::CollisionShape::makeSphere(kSphereRadius));
  sphere.setFriction(0.0);

  return world;
}

//==============================================================================
void applyState(sx::World& world, const BodyState& state)
{
  auto sphere = world.getRigidBody("sphere");
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = state.position;
  sphere->setTransform(pose);
  sphere->setLinearVelocity(state.velocity);
  sphere->setForce(state.force);
}

//==============================================================================
Eigen::VectorXd readState(sx::World& world)
{
  auto sphere = world.getRigidBody("sphere");
  Eigen::VectorXd x(6);
  x.head<3>() = sphere->getTranslation();
  x.tail<3>() = sphere->getLinearVelocity();
  return x;
}

//==============================================================================
// Multi-contact box scene: a unit box resting flat on the static box ground.
// The face-face manifold has four corner contacts (several clamping normal
// rows) whose lever arms are not parallel to the normal, so the contact-aware
// public Jacobian must carry the multi-contact + angular-row gradient.
constexpr double kBoxHalfExtent = 0.5;
constexpr double kBoxRestZ = kBoxHalfExtent - 5e-5;

std::unique_ptr<sx::World> buildBoxScene()
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

  sx::RigidBodyOptions boxOptions;
  boxOptions.mass = 1.0;
  boxOptions.position = Eigen::Vector3d(0.0, 0.0, kBoxRestZ);
  auto box = world->addRigidBody("box", boxOptions);
  box.setCollisionShape(
      sx::CollisionShape::makeBox(
          Eigen::Vector3d(kBoxHalfExtent, kBoxHalfExtent, kBoxHalfExtent)));
  box.setFriction(0.0);

  return world;
}

void applyBoxState(sx::World& world, const BodyState& state)
{
  auto box = world.getRigidBody("box");
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = state.position;
  box->setTransform(pose);
  box->setLinearVelocity(state.velocity);
  box->setForce(state.force);
}

Eigen::VectorXd readBoxState(sx::World& world)
{
  auto box = world.getRigidBody("box");
  Eigen::VectorXd x(6);
  x.head<3>() = box->getTranslation();
  x.tail<3>() = box->getLinearVelocity();
  return x;
}

Eigen::MatrixXd finiteDifferenceBoxStateJacobian(
    const BodyState& nominal, double h)
{
  Eigen::MatrixXd jacobian(6, 6);
  for (Eigen::Index k = 0; k < 6; ++k) {
    BodyState plus = nominal;
    BodyState minus = nominal;
    if (k < 3) {
      plus.position[k] += h;
      minus.position[k] -= h;
    } else {
      plus.velocity[k - 3] += h;
      minus.velocity[k - 3] -= h;
    }

    auto plusWorld = buildBoxScene();
    applyBoxState(*plusWorld, plus);
    plusWorld->step();
    const Eigen::VectorXd nextPlus = readBoxState(*plusWorld);

    auto minusWorld = buildBoxScene();
    applyBoxState(*minusWorld, minus);
    minusWorld->step();
    const Eigen::VectorXd nextMinus = readBoxState(*minusWorld);

    jacobian.col(k) = (nextPlus - nextMinus) / (2.0 * h);
  }
  return jacobian;
}

//==============================================================================
// FD-of-step Jacobian: perturb each of the six state components by h, run one
// full forward step from a freshly built world, central-difference [q'; q̇'].
Eigen::MatrixXd finiteDifferenceStateJacobian(
    const BodyState& nominal, double h)
{
  Eigen::MatrixXd jacobian(6, 6);
  for (Eigen::Index k = 0; k < 6; ++k) {
    BodyState plus = nominal;
    BodyState minus = nominal;
    if (k < 3) {
      plus.position[k] += h;
      minus.position[k] -= h;
    } else {
      plus.velocity[k - 3] += h;
      minus.velocity[k - 3] -= h;
    }

    auto plusWorld = buildScene();
    applyState(*plusWorld, plus);
    plusWorld->step();
    const Eigen::VectorXd nextPlus = readState(*plusWorld);

    auto minusWorld = buildScene();
    applyState(*minusWorld, minus);
    minusWorld->step();
    const Eigen::VectorXd nextMinus = readState(*minusWorld);

    jacobian.col(k) = (nextPlus - nextMinus) / (2.0 * h);
  }
  return jacobian;
}

//==============================================================================
Eigen::MatrixXd finiteDifferenceControlJacobian(
    const BodyState& nominal, double h)
{
  Eigen::MatrixXd jacobian(6, 3);
  for (Eigen::Index k = 0; k < 3; ++k) {
    BodyState plus = nominal;
    BodyState minus = nominal;
    plus.force[k] += h;
    minus.force[k] -= h;

    auto plusWorld = buildScene();
    applyState(*plusWorld, plus);
    plusWorld->step();
    const Eigen::VectorXd nextPlus = readState(*plusWorld);

    auto minusWorld = buildScene();
    applyState(*minusWorld, minus);
    minusWorld->step();
    const Eigen::VectorXd nextMinus = readState(*minusWorld);

    jacobian.col(k) = (nextPlus - nextMinus) / (2.0 * h);
  }
  return jacobian;
}

//==============================================================================
double relativeError(const Eigen::MatrixXd& a, const Eigen::MatrixXd& b)
{
  const double denom = std::max(
      1.0, std::max(a.cwiseAbs().maxCoeff(), b.cwiseAbs().maxCoeff()));
  return (a - b).cwiseAbs().maxCoeff() / denom;
}

} // namespace

//==============================================================================
// In-scope clamping contact, read through the PUBLIC API: build, step once,
// then World::getStepDerivatives() returns the contact-aware Jacobian. It must
// match FD-of-step within rel < 1e-4.
TEST(DiffPublicContactJacobian, ClampingContactMatchesFiniteDifference)
{
  // Place the sphere with ~5e-5 penetration: deep enough that the contact is
  // present under every FD perturbation (max h = 1e-5), shallow enough that the
  // penetration stays below the 1e-4 positional-correction allowance.
  BodyState config;
  config.position = Eigen::Vector3d(0.0, 0.0, kSphereRadius - 5e-5);
  config.velocity = Eigen::Vector3d(0.0, 0.0, -0.2); // pressing into the ground
  config.force = Eigen::Vector3d::Zero();

  auto world = buildScene();
  applyState(*world, config);

  // The contact must be firmly clamping at this pre-step state.
  const auto contacts = world->collide();
  ASSERT_FALSE(contacts.empty()) << "expected an active sphere/ground contact";

  // Drive the PUBLIC step, then read the PUBLIC derivatives.
  world->step();
  const sx::StepDerivatives analytic = world->getStepDerivatives();
  ASSERT_EQ(analytic.stateJacobian.rows(), 6);
  ASSERT_EQ(analytic.stateJacobian.cols(), 6);
  ASSERT_EQ(analytic.controlJacobian.rows(), 6);
  ASSERT_EQ(analytic.controlJacobian.cols(), 3);

  const std::array<double, 3> sweep{1e-5, 1e-6, 1e-7};

  double bestStateError = std::numeric_limits<double>::infinity();
  for (const double h : sweep) {
    const Eigen::MatrixXd fd = finiteDifferenceStateJacobian(config, h);
    bestStateError
        = std::min(bestStateError, relativeError(analytic.stateJacobian, fd));
  }
  EXPECT_LT(bestStateError, 1e-4)
      << "best FD-of-step state relative error: " << bestStateError;

  double bestControlError = std::numeric_limits<double>::infinity();
  for (const double h : sweep) {
    const Eigen::MatrixXd fd = finiteDifferenceControlJacobian(config, h);
    bestControlError = std::min(
        bestControlError, relativeError(analytic.controlJacobian, fd));
  }
  EXPECT_LT(bestControlError, 1e-4)
      << "best FD-of-step control relative error: " << bestControlError;

  GTEST_LOG_(INFO) << "public clamping-contact FD-of-step rel error: state="
                   << bestStateError << " control=" << bestControlError;

  // Physical sanity: the clamped normal velocity is insensitive to the incoming
  // vertical velocity (∂vz'/∂vz ≈ 0), unlike the free-falling horizontal axes.
  EXPECT_NEAR(analytic.stateJacobian(5, 5), 0.0, 1e-6);
  EXPECT_NEAR(analytic.stateJacobian(3, 3), 1.0, 1e-6);
}

//==============================================================================
// No-contact scene through the PUBLIC API: with the sphere clear of the ground
// the public Jacobian reduces to the contact-free (free-fall) Jacobian and
// matches both the closed form and FD-of-step.
TEST(DiffPublicContactJacobian, NoContactMatchesContactFreeFiniteDifference)
{
  BodyState config;
  config.position = Eigen::Vector3d(0.0, 0.0, 1.0); // well above the ground
  config.velocity = Eigen::Vector3d(0.1, -0.2, 0.3);
  config.force = Eigen::Vector3d(0.5, 0.0, 0.0);

  auto world = buildScene();
  applyState(*world, config);

  ASSERT_TRUE(world->collide().empty()) << "expected no contact when separated";

  world->step();
  const sx::StepDerivatives analytic = world->getStepDerivatives();
  ASSERT_EQ(analytic.stateJacobian.rows(), 6);
  ASSERT_EQ(analytic.controlJacobian.cols(), 3);

  // Closed-form contact-free (free-fall) Jacobian.
  const double dt = kTimeStep;
  Eigen::MatrixXd expectedState = Eigen::MatrixXd::Zero(6, 6);
  expectedState.topLeftCorner<3, 3>() = Eigen::Matrix3d::Identity();
  expectedState.topRightCorner<3, 3>() = dt * Eigen::Matrix3d::Identity();
  expectedState.bottomRightCorner<3, 3>() = Eigen::Matrix3d::Identity();

  Eigen::MatrixXd expectedControl = Eigen::MatrixXd::Zero(6, 3);
  expectedControl.topRows<3>()
      = (dt * dt / kSphereMass) * Eigen::Matrix3d::Identity();
  expectedControl.bottomRows<3>()
      = (dt / kSphereMass) * Eigen::Matrix3d::Identity();

  const double stateClosedFormError
      = relativeError(analytic.stateJacobian, expectedState);
  const double controlClosedFormError
      = relativeError(analytic.controlJacobian, expectedControl);
  EXPECT_LT(stateClosedFormError, 1e-9);
  EXPECT_LT(controlClosedFormError, 1e-9);

  // Independent FD-of-step cross-check (plain free fall here).
  const Eigen::MatrixXd fdState = finiteDifferenceStateJacobian(config, 1e-6);
  const double stateFdError = relativeError(analytic.stateJacobian, fdState);
  EXPECT_LT(stateFdError, 1e-4);

  GTEST_LOG_(INFO) << "public no-contact rel error: state(closed-form)="
                   << stateClosedFormError
                   << " control(closed-form)=" << controlClosedFormError
                   << " state(FD-of-step)=" << stateFdError;
}

//==============================================================================
// In-scope multi-point box contact, read through the PUBLIC API: a box resting
// flat on the ground generates several corner contact points (multiple clamping
// normal rows) whose lever arms excite the angular DOFs. This was previously an
// out-of-scope throw case; the contact gradient now handles it. After a public
// step the public Jacobian must carry the multi-contact + angular-row gradient
// and match FD-of-step within rel < 1e-4.
TEST(DiffPublicContactJacobian, MultiContactBoxMatchesFiniteDifference)
{
  BodyState config;
  config.position = Eigen::Vector3d(0.0, 0.0, kBoxRestZ);
  config.velocity = Eigen::Vector3d(0.0, 0.0, -0.2); // pressing into the ground
  config.force = Eigen::Vector3d::Zero();

  auto world = buildBoxScene();
  applyBoxState(*world, config);

  // The flat box/ground manifold must present multiple contact points.
  const auto contacts = world->collide();
  ASSERT_GT(contacts.size(), 1u)
      << "expected a multi-point box/ground face contact";

  world->step();
  const sx::StepDerivatives analytic = world->getStepDerivatives();
  ASSERT_EQ(analytic.stateJacobian.rows(), 6);
  ASSERT_EQ(analytic.stateJacobian.cols(), 6);

  const std::array<double, 3> sweep{1e-5, 1e-6, 1e-7};

  double bestStateError = std::numeric_limits<double>::infinity();
  for (const double h : sweep) {
    const Eigen::MatrixXd fd = finiteDifferenceBoxStateJacobian(config, h);
    bestStateError
        = std::min(bestStateError, relativeError(analytic.stateJacobian, fd));
  }
  EXPECT_LT(bestStateError, 1e-4)
      << "best FD-of-step state relative error: " << bestStateError;

  GTEST_LOG_(INFO) << "public multi-contact box FD-of-step rel error: state="
                   << bestStateError;

  // Physical sanity: the multi-point flat clamp removes the downward normal
  // velocity (∂vz'/∂vz ≈ 0) while the in-plane axes free-fall (≈ 1).
  EXPECT_NEAR(analytic.stateJacobian(5, 5), 0.0, 1e-6);
  EXPECT_NEAR(analytic.stateJacobian(3, 3), 1.0, 1e-6);
}

//==============================================================================
// Out-of-scope multibody/articulated-link contact through the PUBLIC API: a
// multibody link with a collision shape overlapping the static rigid ground
// produces a contact that touches a non-rigid-body link. The rigid-body contact
// assembly does not handle articulated-link contacts, so the public step() must
// THROW rather than silently returning a wrong (contact-omitting) Jacobian.
TEST(DiffPublicContactJacobian, MultibodyLinkContactThrowsUnsupported)
{
  sx::WorldOptions options;
  options.timeStep = kTimeStep;
  options.gravity = Eigen::Vector3d(0.0, 0.0, -9.81);
  options.differentiable = true;
  options.contactSolverMethod = sx::ContactSolverMethod::BoxedLcp;
  sx::World world(options);

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
  auto ground = world.addRigidBody("ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(5.0, 5.0, 0.5)));
  ground.setFriction(0.0);

  // A multibody root link with a collision box at the origin, overlapping the
  // ground top face: the contact touches a non-rigid-body link.
  auto robot = world.addMultibody("robot");
  auto base = robot.addLink("base");
  base.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(0.5, 0.5, 0.5)));

  const auto contacts = world.collide();
  ASSERT_FALSE(contacts.empty())
      << "expected an active multibody-link/ground contact";

  EXPECT_THROW(world.step(), sx::NotImplementedException);
}
