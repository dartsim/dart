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

// Opt-in contact-gradient MODE refinements (PLAN-110 WS5). Three modes select
// how the BACKWARD pass produces its contact gradient; the FORWARD step is
// identical for every mode. Coverage:
//   * Mode-selector reflection through WorldOptions / World.
//   * 5a ELASTIC/RESTITUTION: the analytic gradient (ANALYTIC mode) is correct
//     for a STABLE clamping contact with restitution > 0, gated FD-of-step.
//   * 5b COMPLEMENTARITY_AWARE: a heuristic (NOT the true derivative) that
//   turns
//     a saddle (clamped) normal gradient into a non-zero search direction. No
//     FD gate -- asserts the mode changes the gradient as intended.
//   * 5c PRE_CONTACT_SURROGATE: a backward-only surrogate (NOT the true
//     derivative) that yields a non-zero gradient toward contact for an
//     approaching body that ANALYTIC leaves identically zero. No FD gate.
// Registered only when DART_BUILD_DIFF is ON.

#include <dart/simulation/experimental/body/collision_shape.hpp>
#include <dart/simulation/experimental/body/contact.hpp>
#include <dart/simulation/experimental/body/rigid_body.hpp>
#include <dart/simulation/experimental/body/rigid_body_options.hpp>
#include <dart/simulation/experimental/detail/contact_jacobians.hpp>
#include <dart/simulation/experimental/detail/world_registry_access.hpp>
#include <dart/simulation/experimental/diff/step_derivatives.hpp>
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
constexpr double kRestitution = 0.6;

//==============================================================================
struct BodyState
{
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d force = Eigen::Vector3d::Zero();
};

//==============================================================================
// Frictionless sphere-on-static-ground, boxed-LCP, differentiable, with a
// configurable restitution and gradient mode. The ground top face is at z = 0.
std::unique_ptr<sx::World> buildScene(
    double restitution = 0.0,
    sx::ContactGradientMode mode = sx::ContactGradientMode::Analytic)
{
  sx::WorldOptions options;
  options.timeStep = kTimeStep;
  options.gravity = Eigen::Vector3d(0.0, 0.0, -9.81);
  options.differentiable = true;
  options.contactSolverMethod = sx::ContactSolverMethod::BoxedLcp;
  options.contactGradientMode = mode;
  auto world = std::make_unique<sx::World>(options);

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
  auto ground = world->addRigidBody("ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(5.0, 5.0, 0.5)));
  ground.setFriction(0.0);
  ground.setRestitution(restitution);

  sx::RigidBodyOptions sphereOptions;
  sphereOptions.mass = kSphereMass;
  sphereOptions.position = Eigen::Vector3d(0.0, 0.0, kSphereRadius);
  auto sphere = world->addRigidBody("sphere", sphereOptions);
  sphere.setCollisionShape(sx::CollisionShape::makeSphere(kSphereRadius));
  sphere.setFriction(0.0);
  sphere.setRestitution(restitution);

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
double relativeError(const Eigen::MatrixXd& a, const Eigen::MatrixXd& b)
{
  const double denom = std::max(
      1.0, std::max(a.cwiseAbs().maxCoeff(), b.cwiseAbs().maxCoeff()));
  return (a - b).cwiseAbs().maxCoeff() / denom;
}

//==============================================================================
// FD-of-step state Jacobian: perturb each of the six state components by h, run
// one full forward step from a freshly built world (with restitution), central-
// difference [q'; q̇']. The forward step is restitution-mode-independent, so the
// scene is always built with ANALYTIC.
Eigen::MatrixXd finiteDifferenceStateJacobian(
    const BodyState& nominal, double h, double restitution)
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

    auto plusWorld = buildScene(restitution);
    applyState(*plusWorld, plus);
    plusWorld->step();
    const Eigen::VectorXd nextPlus = readState(*plusWorld);

    auto minusWorld = buildScene(restitution);
    applyState(*minusWorld, minus);
    minusWorld->step();
    const Eigen::VectorXd nextMinus = readState(*minusWorld);

    jacobian.col(k) = (nextPlus - nextMinus) / (2.0 * h);
  }
  return jacobian;
}

//==============================================================================
// Analytic contact Jacobian at the world's current (pre-step) state, in the
// given mode, via the detail entry point.
sx::StepDerivatives analyticDerivatives(
    sx::World& world, sx::ContactGradientMode mode)
{
  const auto contacts = world.collide();
  return sx::detail::contactStepDerivatives(
      dart::simulation::experimental::detail::registryOf(world),
      contacts,
      world.getGravity(),
      world.getTimeStep(),
      mode);
}

} // namespace

//==============================================================================
// Mode selector reflects through WorldOptions and World.
TEST(DiffContactGradientModes, ModeSelectorReflectsThroughWorld)
{
  // Default is ANALYTIC.
  auto defaultWorld = buildScene();
  EXPECT_EQ(
      defaultWorld->getContactGradientMode(),
      sx::ContactGradientMode::Analytic);

  for (const auto mode :
       {sx::ContactGradientMode::Analytic,
        sx::ContactGradientMode::ComplementarityAware,
        sx::ContactGradientMode::PreContactSurrogate}) {
    auto world = buildScene(0.0, mode);
    EXPECT_EQ(world->getContactGradientMode(), mode);
  }

  // The runtime setter overrides the construction-time mode (the mode only
  // affects the backward pass, so it is safe to change at any time).
  auto world = buildScene();
  world->setContactGradientMode(sx::ContactGradientMode::ComplementarityAware);
  EXPECT_EQ(
      world->getContactGradientMode(),
      sx::ContactGradientMode::ComplementarityAware);
  world->setContactGradientMode(sx::ContactGradientMode::PreContactSurrogate);
  EXPECT_EQ(
      world->getContactGradientMode(),
      sx::ContactGradientMode::PreContactSurrogate);
}

//==============================================================================
// 5a ELASTIC/RESTITUTION (FD-gated, ANALYTIC mode). A sphere presses into a
// restituting ground with a sustained downward velocity well past the
// restitution make/break threshold, so the contact is firmly clamping and the
// restitution bias (-e * approach) is smooth under the FD perturbations. The
// post-step normal velocity bounces (depends on e), so the gradient carries the
// restitution term. The analytic ∂x'/∂x must match FD-of-step.
TEST(DiffContactGradientModes, RestitutionClampingMatchesFiniteDifference)
{
  // ~5e-5 penetration (below the 1e-4 positional-correction allowance) and a
  // downward approach velocity (-0.2 m/s) far more negative than the 1e-3
  // restitution threshold, so the e-bias is active and smooth throughout the FD
  // sweep (max h = 1e-5).
  BodyState config;
  config.position = Eigen::Vector3d(0.0, 0.0, kSphereRadius - 5e-5);
  config.velocity = Eigen::Vector3d(0.0, 0.0, -0.2);
  config.force = Eigen::Vector3d::Zero();

  auto world = buildScene(kRestitution);
  applyState(*world, config);

  const auto contacts = world->collide();
  ASSERT_FALSE(contacts.empty()) << "expected an active sphere/ground contact";

  const sx::StepDerivatives analytic
      = analyticDerivatives(*world, sx::ContactGradientMode::Analytic);
  ASSERT_EQ(analytic.stateJacobian.rows(), 6);
  ASSERT_EQ(analytic.stateJacobian.cols(), 6);

  const std::array<double, 3> sweep{1e-5, 1e-6, 1e-7};
  double bestStateError = std::numeric_limits<double>::infinity();
  for (const double h : sweep) {
    const Eigen::MatrixXd fd
        = finiteDifferenceStateJacobian(config, h, kRestitution);
    bestStateError
        = std::min(bestStateError, relativeError(analytic.stateJacobian, fd));
  }
  EXPECT_LT(bestStateError, 1e-4)
      << "best FD-of-step state relative error (restitution): "
      << bestStateError;

  GTEST_LOG_(INFO) << "restitution clamping FD-of-step state rel error: "
                   << bestStateError;

  // Restitution signature: with e > 0 the post-step vertical velocity REVERSES
  // and scales with the incoming vertical velocity, so ∂vz'/∂vz ≈ -e (not 0 as
  // in the inelastic clamp). This is the term that distinguishes the
  // restitution gradient from the inelastic one.
  EXPECT_NEAR(analytic.stateJacobian(5, 5), -kRestitution, 1e-3)
      << "expected ∂vz'/∂vz ≈ -e for a clamping restituting contact";
}

//==============================================================================
// 5a guard: with restitution at zero the gradient reproduces the inelastic
// clamp (∂vz'/∂vz ≈ 0), confirming the restitution term is the only difference.
TEST(DiffContactGradientModes, InelasticClampHasZeroNormalVelocitySensitivity)
{
  BodyState config;
  config.position = Eigen::Vector3d(0.0, 0.0, kSphereRadius - 5e-5);
  config.velocity = Eigen::Vector3d(0.0, 0.0, -0.2);
  config.force = Eigen::Vector3d::Zero();

  auto world = buildScene(0.0);
  applyState(*world, config);
  ASSERT_FALSE(world->collide().empty());

  const sx::StepDerivatives analytic
      = analyticDerivatives(*world, sx::ContactGradientMode::Analytic);
  EXPECT_NEAR(analytic.stateJacobian(5, 5), 0.0, 1e-6);
}

//==============================================================================
// 5b COMPLEMENTARITY_AWARE (heuristic, NON-true-gradient, NO FD gate). At a
// firmly clamping inelastic contact the ANALYTIC gradient of the post-step
// normal velocity w.r.t. the incoming normal velocity is a saddle: ∂vz'/∂vz ≈
// 0, so an optimizer pulling the body off the ground sees no descent direction.
// COMPLEMENTARITY_AWARE reclassifies the clamping row as separating for the
// backward pass, restoring the free-fall sensitivity (∂vz'/∂vz ≈ 1). Assert the
// mode changes the gradient as intended; this is a heuristic, not the true
// derivative (the FD-of-step gradient remains ≈ 0, which is what ANALYTIC
// reports).
TEST(DiffContactGradientModes, ComplementarityAwareUnsaddlesClampingGradient)
{
  BodyState config;
  config.position = Eigen::Vector3d(0.0, 0.0, kSphereRadius - 5e-5);
  config.velocity = Eigen::Vector3d(0.0, 0.0, -0.2);
  config.force = Eigen::Vector3d::Zero();

  auto world = buildScene(0.0);
  applyState(*world, config);
  ASSERT_FALSE(world->collide().empty())
      << "expected an active clamping contact";

  const sx::StepDerivatives analytic
      = analyticDerivatives(*world, sx::ContactGradientMode::Analytic);
  const sx::StepDerivatives aware = analyticDerivatives(
      *world, sx::ContactGradientMode::ComplementarityAware);

  // ANALYTIC: the clamped normal velocity is insensitive to its input (saddle).
  EXPECT_NEAR(analytic.stateJacobian(5, 5), 0.0, 1e-6)
      << "ANALYTIC ∂vz'/∂vz should be ≈ 0 at a clamping contact";

  // COMPLEMENTARITY_AWARE: the reclassified-as-separating row restores a
  // non-zero search direction (the free-fall ∂vz'/∂vz ≈ 1).
  EXPECT_GT(std::abs(aware.stateJacobian(5, 5)), 0.5)
      << "COMPLEMENTARITY_AWARE should yield a non-zero ∂vz'/∂vz";
  EXPECT_NEAR(aware.stateJacobian(5, 5), 1.0, 1e-6);

  // The two modes must differ where it matters.
  EXPECT_GT(
      std::abs(aware.stateJacobian(5, 5) - analytic.stateJacobian(5, 5)), 0.5)
      << "the gradient mode must change the clamped-direction gradient";

  // Tangential (free) axes are unchanged by the mode: both free-fall there.
  EXPECT_NEAR(analytic.stateJacobian(3, 3), aware.stateJacobian(3, 3), 1e-9);
}

//==============================================================================
// 5c PRE_CONTACT_SURROGATE (backward-only surrogate, NON-true-gradient, NO FD
// gate). A sphere approaching the ground but not yet touching has NO active
// contact, so ANALYTIC produces exactly the contact-free (free-fall) Jacobian:
// its contact gradient block (the difference from free-fall) is identically
// zero. PRE_CONTACT_SURROGATE synthesizes a distance/velocity-based surrogate
// so the gradient is non-zero before contact and points along the approach
// (toward contact) direction. Assert the surrogate block is non-zero and the
// ANALYTIC one is zero; this is a surrogate, not the true (zero) derivative.
TEST(DiffContactGradientModes, PreContactSurrogateAddsTowardContactGradient)
{
  // Sphere well above the ground, moving DOWN (approaching, not touching).
  BodyState config;
  config.position = Eigen::Vector3d(0.0, 0.0, 1.0);
  config.velocity = Eigen::Vector3d(0.0, 0.0, -0.5); // approaching the ground
  config.force = Eigen::Vector3d::Zero();

  auto world = buildScene(0.0);
  applyState(*world, config);
  ASSERT_TRUE(world->collide().empty())
      << "expected NO contact (approaching, not touching)";

  const sx::StepDerivatives analytic
      = analyticDerivatives(*world, sx::ContactGradientMode::Analytic);
  const sx::StepDerivatives surrogate = analyticDerivatives(
      *world, sx::ContactGradientMode::PreContactSurrogate);

  // The contact-free (free-fall) reference Jacobian for q' = q + Δt q̇',
  // q̇' = q̇ + Δt(F/m + g): ∂q'/∂q = I, ∂q'/∂q̇ = Δt I, ∂q̇'/∂q̇ = I.
  const double dt = kTimeStep;
  Eigen::MatrixXd freeFall = Eigen::MatrixXd::Zero(6, 6);
  freeFall.topLeftCorner<3, 3>() = Eigen::Matrix3d::Identity();
  freeFall.topRightCorner<3, 3>() = dt * Eigen::Matrix3d::Identity();
  freeFall.bottomRightCorner<3, 3>() = Eigen::Matrix3d::Identity();

  // ANALYTIC: identically the free-fall block (zero contact gradient).
  EXPECT_LT(relativeError(analytic.stateJacobian, freeFall), 1e-9)
      << "ANALYTIC should give the pure free-fall (zero-contact) gradient";

  // PRE_CONTACT_SURROGATE: differs from free-fall by a non-zero surrogate
  // block.
  const double surrogateMagnitude
      = (surrogate.stateJacobian - freeFall).cwiseAbs().maxCoeff();
  EXPECT_GT(surrogateMagnitude, 1e-3)
      << "PRE_CONTACT_SURROGATE should add a non-zero pre-contact gradient";

  GTEST_LOG_(INFO) << "pre-contact surrogate block magnitude: "
                   << surrogateMagnitude;

  // The approach is purely along -z, so the surrogate arrests the vertical
  // velocity sensitivity: ∂vz'/∂vz drops from 1 (free fall) toward 0 (the clamp
  // it anticipates), while the in-plane axes stay at the free-fall value.
  EXPECT_LT(surrogate.stateJacobian(5, 5), 0.5)
      << "surrogate should reduce ∂vz'/∂vz toward the anticipated clamp";
  EXPECT_NEAR(surrogate.stateJacobian(3, 3), 1.0, 1e-9)
      << "in-plane (non-approach) axes must keep the free-fall sensitivity";
}

//==============================================================================
// 5c guard: ANALYTIC and the surrogate must agree exactly when a body is in a
// firmly clamping contact (the surrogate only fires for bodies NOT in contact).
TEST(DiffContactGradientModes, PreContactSurrogateInertWhenInContact)
{
  BodyState config;
  config.position = Eigen::Vector3d(0.0, 0.0, kSphereRadius - 5e-5);
  config.velocity = Eigen::Vector3d(0.0, 0.0, -0.2);
  config.force = Eigen::Vector3d::Zero();

  auto world = buildScene(0.0);
  applyState(*world, config);
  ASSERT_FALSE(world->collide().empty());

  const sx::StepDerivatives analytic
      = analyticDerivatives(*world, sx::ContactGradientMode::Analytic);
  const sx::StepDerivatives surrogate = analyticDerivatives(
      *world, sx::ContactGradientMode::PreContactSurrogate);

  EXPECT_LT(
      relativeError(analytic.stateJacobian, surrogate.stateJacobian), 1e-9)
      << "surrogate must be inert for a body already in clamping contact";
}
