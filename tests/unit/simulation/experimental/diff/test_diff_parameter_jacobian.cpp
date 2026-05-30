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

// PUBLIC-facade correctness gate for the differentiable PHYSICAL PARAMETER
// Jacobian (PLAN-110 WS4: MASS, INERTIA, FRICTION). It drives the PUBLIC API:
// build a BoxedLcp differentiable World, register a body's parameter via
// World::addDifferentiableParameter, step once, then read
// StepDerivatives::parameterJacobian and compare against a finite-difference of
// the full forward step with respect to the actual parameter (perturb the
// parameter, run one step, central difference). It asserts:
//   - MASS free-fall (no contact): the analytic parameter Jacobian matches both
//     the closed form (∂v'/∂m = -Δt F / m², ∂q'/∂m = Δt times that) and
//     FD-of-step,
//   - MASS clamping contact: the parameter Jacobian matches FD-of-step,
//   - INERTIA off-COM (tilted box) contact: the three diagonal-moment columns
//     match FD-of-step (inertia couples into the translational state through
//     the rotational contact response),
//   - FRICTION sliding sphere contact: the single column matches FD-of-step
//     (mu decelerates the tangential velocity in the sliding regime),
//   - a NO-registration step leaves parameterJacobian empty while still
//     producing the state/control Jacobians,
//   - CENTER_OF_MASS is rejected at registration (unsupported: the rigid-body
//     step fixes the COM at the body origin, so its gradient is identically
//     zero).
// Registered only when DART_BUILD_DIFF is ON.

#include <dart/simulation/experimental/body/collision_shape.hpp>
#include <dart/simulation/experimental/body/contact.hpp>
#include <dart/simulation/experimental/body/rigid_body.hpp>
#include <dart/simulation/experimental/body/rigid_body_options.hpp>
#include <dart/simulation/experimental/common/exceptions.hpp>
#include <dart/simulation/experimental/diff/physical_parameter.hpp>
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

//==============================================================================
struct BodyState
{
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d force = Eigen::Vector3d::Zero();
  double mass = kSphereMass;
  // Body-frame inertia diagonal (used by the tilted-box INERTIA scene; ignored
  // by the sphere scenes, which keep the default inertia).
  Eigen::Vector3d inertia = Eigen::Vector3d::Ones();
  // World-frame angular velocity (used by the tilted-box INERTIA scene to break
  // the corner-contact symmetry so all three diagonal moments couple into the
  // translational output; zero for the sphere scenes).
  Eigen::Vector3d angularVelocity = Eigen::Vector3d::Zero();
};

//==============================================================================
// Frictionless sphere-on-static-box-ground scene, BoxedLcp + differentiable.
// The ground top face is at z = 0.
std::unique_ptr<sx::World> buildScene(double mass)
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
  sphereOptions.mass = mass;
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
  sphere->setMass(state.mass);
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
// FD-of-step parameter Jacobian: perturb the actual mass by h, run one full
// forward step from a freshly built world, central-difference [q'; q̇'].
Eigen::VectorXd finiteDifferenceMassJacobian(const BodyState& nominal, double h)
{
  BodyState plus = nominal;
  BodyState minus = nominal;
  plus.mass += h;
  minus.mass -= h;

  auto plusWorld = buildScene(plus.mass);
  applyState(*plusWorld, plus);
  plusWorld->step();
  const Eigen::VectorXd nextPlus = readState(*plusWorld);

  auto minusWorld = buildScene(minus.mass);
  applyState(*minusWorld, minus);
  minusWorld->step();
  const Eigen::VectorXd nextMinus = readState(*minusWorld);

  return (nextPlus - nextMinus) / (2.0 * h);
}

//==============================================================================
double relativeError(const Eigen::VectorXd& a, const Eigen::VectorXd& b)
{
  const double denom = std::max(
      1.0, std::max(a.cwiseAbs().maxCoeff(), b.cwiseAbs().maxCoeff()));
  return (a - b).cwiseAbs().maxCoeff() / denom;
}

//==============================================================================
// FRICTION scene: a sphere sliding on a frictional static box ground. The lever
// arm is along the (vertical) normal, so the angular DOFs decouple, but the
// tangential friction impulse decelerates the in-plane velocity, so the
// post-step linear velocity depends on the combined Coulomb coefficient mu. A
// high incoming horizontal speed keeps the contact firmly in the SLIDING regime
// (cone edge), well away from the stick transition, so the active set is stable
// under FD perturbations of mu.
constexpr double kSlidingFriction = 0.3;

std::unique_ptr<sx::World> buildFrictionScene()
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
  ground.setFriction(kSlidingFriction);

  sx::RigidBodyOptions sphereOptions;
  sphereOptions.mass = kSphereMass;
  sphereOptions.position = Eigen::Vector3d(0.0, 0.0, kSphereRadius);
  auto sphere = world->addRigidBody("sphere", sphereOptions);
  sphere.setCollisionShape(sx::CollisionShape::makeSphere(kSphereRadius));
  sphere.setFriction(kSlidingFriction);

  return world;
}

//==============================================================================
// FD-of-step FRICTION Jacobian: perturb the sphere's actual friction
// coefficient by h, run one full forward step from a freshly built world,
// central-difference [q'; q̇']. Both bodies share kSlidingFriction; the combined
// mu = sqrt(muA*muB) = muA when muB is fixed, so perturbing the sphere's
// coefficient by h moves the per-pair mu by ~h/2 — the analytic column
// differentiates the SAME registry field, so the comparison is consistent.
Eigen::VectorXd finiteDifferenceFrictionJacobian(
    const BodyState& nominal, double frictionNominal, double h)
{
  const auto run = [&](double friction) {
    auto world = buildFrictionScene();
    auto sphere = world->getRigidBody("sphere");
    sphere->setFriction(friction);
    applyState(*world, nominal);
    world->step();
    return readState(*world);
  };
  const Eigen::VectorXd nextPlus = run(frictionNominal + h);
  const Eigen::VectorXd nextMinus = run(frictionNominal - h);
  return (nextPlus - nextMinus) / (2.0 * h);
}

//==============================================================================
// INERTIA scene: a non-cubic box lying flush on a static ground rotated by a
// GENERIC 3D orientation (a yaw + tilt combination, with the box rotated to
// match). The four corner contacts track the body, and a sticking friction
// (high mu, slow normal-only approach) plus a 3D angular velocity break the
// corner symmetry so that all three body-frame diagonal moments (Ixx, Iyy, Izz)
// couple into the post-step translational velocity. With a pure y-tilt and no
// friction the corner symmetry makes most of the inertia gradient cancel; the
// generic orientation, the sticking tangential rows, and the non-cubic extents
// remove every cancellation, so the FD comparison is non-vacuous for each axis.
// The configuration is stable (flush face, fixed orientation, sub-allowance
// penetration, firmly sticking), so the active set does not flip under FD
// perturbations of the inertia diagonal.
constexpr double kBoxMass = 1.0;
constexpr double kBoxHalfZ = 0.5; // half-extent along the box's contacting face
constexpr double kBoxTilt = 0.15;
constexpr double kBoxStickFriction
    = 5.0; // high enough to keep the contact stuck
// Anisotropic body-frame inertia so each diagonal moment has a distinct,
// nonzero gradient (a sphere-like isotropic tensor would make the three columns
// degenerate). Diagonal-dominant, so perturbing one diagonal entry keeps it
// SPD.
const Eigen::Vector3d kBoxInertiaDiag(0.20, 0.35, 0.50);
// Non-cubic half-extents; the half-z matches kBoxHalfZ so the rest height
// holds.
const Eigen::Vector3d kBoxHalfExtents(0.60, 0.45, 0.50);
// World-frame angular velocity that excites a rotational contact response about
// every axis (so each diagonal moment matters).
const Eigen::Vector3d kBoxAngularVelocity(0.7, -0.5, 0.9);

Eigen::Matrix3d boxRotation()
{
  return (Eigen::AngleAxisd(0.6, Eigen::Vector3d::UnitZ())
          * Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitX())
          * Eigen::AngleAxisd(kBoxTilt, Eigen::Vector3d::UnitY()))
      .toRotationMatrix();
}

Eigen::Vector3d boxNormal()
{
  return boxRotation() * Eigen::Vector3d::UnitZ();
}

// Box center for a flush rest with ~5e-5 penetration along the tilted normal.
Eigen::Vector3d tiltedBoxRestPosition()
{
  return boxRotation() * Eigen::Vector3d(0.0, 0.0, kBoxHalfZ)
         - 5e-5 * boxNormal();
}

std::unique_ptr<sx::World> buildTiltedBoxScene()
{
  sx::WorldOptions options;
  options.timeStep = kTimeStep;
  options.gravity = Eigen::Vector3d(0.0, 0.0, -9.81);
  options.differentiable = true;
  options.contactSolverMethod = sx::ContactSolverMethod::BoxedLcp;
  auto world = std::make_unique<sx::World>(options);

  const Eigen::Matrix3d rotation = boxRotation();

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  auto ground = world->addRigidBody("ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(5.0, 5.0, 0.5)));
  ground.setFriction(kBoxStickFriction);
  auto groundHandle = world->getRigidBody("ground");
  Eigen::Isometry3d groundPose = Eigen::Isometry3d::Identity();
  groundPose.linear() = rotation;
  groundPose.translation() = rotation * Eigen::Vector3d(0.0, 0.0, -0.5);
  groundHandle->setTransform(groundPose);

  sx::RigidBodyOptions boxOptions;
  boxOptions.mass = kBoxMass;
  boxOptions.inertia = Eigen::Matrix3d(kBoxInertiaDiag.asDiagonal());
  auto box = world->addRigidBody("box", boxOptions);
  box.setCollisionShape(sx::CollisionShape::makeBox(kBoxHalfExtents));
  box.setFriction(kBoxStickFriction);

  return world;
}

//==============================================================================
// Hold the box rotated flush with the slope at the given [pos; linvel],
// applying the inertia diagonal and the symmetry-breaking angular velocity.
void applyTiltedBoxState(sx::World& world, const BodyState& state)
{
  auto box = world.getRigidBody("box");
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() = boxRotation();
  pose.translation() = state.position;
  box->setTransform(pose);
  box->setLinearVelocity(state.velocity);
  box->setAngularVelocity(state.angularVelocity);
  box->setForce(state.force);
  box->setInertia(Eigen::Matrix3d(state.inertia.asDiagonal()));
}

Eigen::VectorXd readBoxState(sx::World& world)
{
  auto box = world.getRigidBody("box");
  Eigen::VectorXd x(6);
  x.head<3>() = box->getTranslation();
  x.tail<3>() = box->getLinearVelocity();
  return x;
}

//==============================================================================
// FD-of-step INERTIA Jacobian for diagonal moment `axis`: perturb that
// body-frame inertia diagonal entry by h, run one full forward step from a
// freshly built tilted-box world, central-difference [q'; q̇'].
Eigen::VectorXd finiteDifferenceInertiaJacobian(
    const BodyState& nominal, Eigen::Index axis, double h)
{
  BodyState plus = nominal;
  BodyState minus = nominal;
  plus.inertia[axis] += h;
  minus.inertia[axis] -= h;

  auto plusWorld = buildTiltedBoxScene();
  applyTiltedBoxState(*plusWorld, plus);
  plusWorld->step();
  const Eigen::VectorXd nextPlus = readBoxState(*plusWorld);

  auto minusWorld = buildTiltedBoxScene();
  applyTiltedBoxState(*minusWorld, minus);
  minusWorld->step();
  const Eigen::VectorXd nextMinus = readBoxState(*minusWorld);

  return (nextPlus - nextMinus) / (2.0 * h);
}

} // namespace

//==============================================================================
// Free fall (no contact): the analytic mass Jacobian matches the closed form
// and FD-of-step. With v' = v + Δt(F/m + g) and q' = q + Δt v', only the F/m
// term depends on m, so ∂v'/∂m = -Δt F / m² and ∂q'/∂m = Δt ∂v'/∂m.
TEST(DiffParameterJacobian, FreeFallMassMatchesClosedFormAndFiniteDifference)
{
  BodyState config;
  config.position = Eigen::Vector3d(0.0, 0.0, 1.0); // well above the ground
  config.velocity = Eigen::Vector3d(0.1, -0.2, 0.3);
  config.force = Eigen::Vector3d(0.5, -0.3, 0.7); // nonzero so ∂/∂m is nonzero
  config.mass = kSphereMass;

  auto world = buildScene(config.mass);
  applyState(*world, config);
  auto sphere = world->getRigidBody("sphere");
  world->addDifferentiableParameter(*sphere, sx::PhysicalParameter::MASS);
  ASSERT_EQ(world->getNumDifferentiableParameters(), 1u);

  ASSERT_TRUE(world->collide().empty()) << "expected no contact when separated";

  world->step();
  const sx::StepDerivatives analytic = world->getStepDerivatives();
  ASSERT_EQ(analytic.parameterJacobian.rows(), 6);
  ASSERT_EQ(analytic.parameterJacobian.cols(), 1);

  // Closed-form ∂x'/∂m.
  const double dt = kTimeStep;
  const double m = config.mass;
  Eigen::VectorXd expected(6);
  const Eigen::Vector3d dVel = -dt * config.force / (m * m);
  expected.tail<3>() = dVel;
  expected.head<3>() = dt * dVel;

  const Eigen::VectorXd analyticCol = analytic.parameterJacobian.col(0);
  const double closedFormError = relativeError(analyticCol, expected);
  EXPECT_LT(closedFormError, 1e-4)
      << "closed-form mass Jacobian rel error: " << closedFormError;

  const std::array<double, 3> sweep{1e-5, 1e-6, 1e-7};
  double bestFdError = std::numeric_limits<double>::infinity();
  for (const double h : sweep) {
    const Eigen::VectorXd fd = finiteDifferenceMassJacobian(config, h);
    bestFdError = std::min(bestFdError, relativeError(analyticCol, fd));
  }
  EXPECT_LT(bestFdError, 1e-4)
      << "best FD-of-step mass Jacobian rel error: " << bestFdError;

  GTEST_LOG_(INFO) << "free-fall mass Jacobian rel error: closed-form="
                   << closedFormError << " FD-of-step=" << bestFdError;
}

//==============================================================================
// Clamping contact: the analytic mass Jacobian matches FD-of-step. The sphere
// presses into the ground, so the normal impulse (and thus the post-step
// velocity) depends on the mass.
TEST(DiffParameterJacobian, ClampingContactMassMatchesFiniteDifference)
{
  BodyState config;
  config.position = Eigen::Vector3d(0.0, 0.0, kSphereRadius - 5e-5);
  config.velocity = Eigen::Vector3d(0.3, 0.0, -0.2); // sliding + pressing
  config.force = Eigen::Vector3d(0.4, 0.0, 0.0);     // horizontal push
  config.mass = kSphereMass;

  auto world = buildScene(config.mass);
  applyState(*world, config);
  auto sphere = world->getRigidBody("sphere");
  world->addDifferentiableParameter(
      sx::PhysicalParameterSelector(*sphere, sx::PhysicalParameter::MASS));

  const auto contacts = world->collide();
  ASSERT_FALSE(contacts.empty()) << "expected an active sphere/ground contact";

  world->step();
  const sx::StepDerivatives analytic = world->getStepDerivatives();
  ASSERT_EQ(analytic.parameterJacobian.rows(), 6);
  ASSERT_EQ(analytic.parameterJacobian.cols(), 1);
  // State/control Jacobians still present alongside the parameter Jacobian.
  ASSERT_EQ(analytic.stateJacobian.rows(), 6);
  ASSERT_EQ(analytic.controlJacobian.cols(), 3);

  const Eigen::VectorXd analyticCol = analytic.parameterJacobian.col(0);

  const std::array<double, 3> sweep{1e-5, 1e-6, 1e-7};
  double bestFdError = std::numeric_limits<double>::infinity();
  for (const double h : sweep) {
    const Eigen::VectorXd fd = finiteDifferenceMassJacobian(config, h);
    bestFdError = std::min(bestFdError, relativeError(analyticCol, fd));
  }
  EXPECT_LT(bestFdError, 1e-4)
      << "best FD-of-step mass Jacobian rel error: " << bestFdError;

  GTEST_LOG_(INFO) << "clamping-contact mass Jacobian FD-of-step rel error: "
                   << bestFdError;
}

//==============================================================================
// INERTIA (off-COM tilted box): the three diagonal-moment columns match
// FD-of-step. The box rests flush on a tilted slope, so the corner contacts'
// lever arms are not parallel to the normal and the rotational contact response
// couples the body-frame inertia into the post-step translational velocity.
TEST(DiffParameterJacobian, TiltedBoxInertiaMatchesFiniteDifference)
{
  BodyState config;
  config.position = tiltedBoxRestPosition();
  // Slow normal-only approach keeps the contact firmly STICKING (so the
  // tangential rows enter the clamping set and carry the in-plane inertia
  // coupling) without flipping the active set; the 3D angular velocity breaks
  // the corner symmetry so all three diagonal moments couple.
  config.velocity = Eigen::Vector3d(0.0, 0.0, -0.05);
  config.angularVelocity = kBoxAngularVelocity;
  config.inertia = kBoxInertiaDiag;

  auto world = buildTiltedBoxScene();
  applyTiltedBoxState(*world, config);
  auto box = world->getRigidBody("box");
  world->addDifferentiableParameter(*box, sx::PhysicalParameter::INERTIA);
  ASSERT_EQ(world->getNumDifferentiableParameters(), 1u);

  const auto contacts = world->collide();
  ASSERT_FALSE(contacts.empty())
      << "expected active box/ground corner contacts";

  world->step();
  const sx::StepDerivatives analytic = world->getStepDerivatives();
  ASSERT_EQ(analytic.parameterJacobian.rows(), 6);
  // INERTIA spans three columns (Ixx, Iyy, Izz).
  ASSERT_EQ(analytic.parameterJacobian.cols(), 3);

  const std::array<double, 3> sweep{1e-5, 1e-6, 1e-7};
  for (Eigen::Index axis = 0; axis < 3; ++axis) {
    const Eigen::VectorXd analyticCol = analytic.parameterJacobian.col(axis);
    // Each diagonal moment must have a non-trivial gradient, otherwise the FD
    // comparison would be a vacuous zero-vs-zero match.
    EXPECT_GT(analyticCol.cwiseAbs().maxCoeff(), 1e-3)
        << "expected a nonzero inertia gradient for axis " << axis;

    double bestFdError = std::numeric_limits<double>::infinity();
    for (const double h : sweep) {
      const Eigen::VectorXd fd
          = finiteDifferenceInertiaJacobian(config, axis, h);
      bestFdError = std::min(bestFdError, relativeError(analyticCol, fd));
    }
    EXPECT_LT(bestFdError, 1e-4) << "tilted-box inertia axis " << axis
                                 << " FD-of-step rel error: " << bestFdError;
    GTEST_LOG_(INFO) << "tilted-box inertia axis " << axis
                     << " FD-of-step rel error: " << bestFdError;
  }
}

//==============================================================================
// FRICTION (sliding sphere): the single column matches FD-of-step. The sphere
// slides on the frictional ground with a high horizontal speed, so the contact
// is firmly in the sliding (cone-edge) regime and the tangential friction
// impulse — hence the post-step horizontal velocity — depends on mu.
TEST(DiffParameterJacobian, SlidingFrictionMatchesFiniteDifference)
{
  BodyState config;
  // Pressing into the ground (clamping normal) and sliding fast horizontally.
  config.position = Eigen::Vector3d(0.0, 0.0, kSphereRadius - 5e-5);
  config.velocity = Eigen::Vector3d(2.0, 0.0, -0.2);

  auto world = buildFrictionScene();
  applyState(*world, config);
  auto sphere = world->getRigidBody("sphere");
  world->addDifferentiableParameter(*sphere, sx::PhysicalParameter::FRICTION);
  ASSERT_EQ(world->getNumDifferentiableParameters(), 1u);

  const auto contacts = world->collide();
  ASSERT_FALSE(contacts.empty()) << "expected an active sphere/ground contact";

  world->step();
  const sx::StepDerivatives analytic = world->getStepDerivatives();
  ASSERT_EQ(analytic.parameterJacobian.rows(), 6);
  ASSERT_EQ(analytic.parameterJacobian.cols(), 1);

  const Eigen::VectorXd analyticCol = analytic.parameterJacobian.col(0);
  // The friction gradient must be non-trivial (the sliding contact really does
  // depend on mu), otherwise the comparison would be vacuous.
  EXPECT_GT(analyticCol.cwiseAbs().maxCoeff(), 1e-6)
      << "expected a nonzero friction gradient in the sliding regime";

  const std::array<double, 3> sweep{1e-5, 1e-6, 1e-7};
  double bestFdError = std::numeric_limits<double>::infinity();
  for (const double h : sweep) {
    const Eigen::VectorXd fd
        = finiteDifferenceFrictionJacobian(config, kSlidingFriction, h);
    bestFdError = std::min(bestFdError, relativeError(analyticCol, fd));
  }
  EXPECT_LT(bestFdError, 1e-4)
      << "sliding friction FD-of-step rel error: " << bestFdError;
  GTEST_LOG_(INFO) << "sliding friction FD-of-step rel error: " << bestFdError;
}

//==============================================================================
// No registration: parameterJacobian stays empty, while the state/control
// Jacobians are produced as usual.
TEST(DiffParameterJacobian, NoRegistrationLeavesParameterJacobianEmpty)
{
  BodyState config;
  config.position = Eigen::Vector3d(0.0, 0.0, kSphereRadius - 5e-5);
  config.velocity = Eigen::Vector3d(0.0, 0.0, -0.2);
  config.mass = kSphereMass;

  auto world = buildScene(config.mass);
  applyState(*world, config);
  ASSERT_EQ(world->getNumDifferentiableParameters(), 0u);

  world->step();
  const sx::StepDerivatives analytic = world->getStepDerivatives();
  EXPECT_EQ(analytic.parameterJacobian.size(), 0);
  EXPECT_EQ(analytic.stateJacobian.rows(), 6);
  EXPECT_EQ(analytic.controlJacobian.cols(), 3);
}

//==============================================================================
// CENTER_OF_MASS is unsupported and rejected at registration with a clear error
// (the rigid-body step fixes the COM at the body origin). INERTIA and FRICTION
// are supported and register without throwing.
TEST(DiffParameterJacobian, CenterOfMassThrowsWhileInertiaAndFrictionRegister)
{
  auto world = buildScene(kSphereMass);
  auto sphere = world->getRigidBody("sphere");
  EXPECT_THROW(
      world->addDifferentiableParameter(
          *sphere, sx::PhysicalParameter::CENTER_OF_MASS),
      sx::NotImplementedException);
  EXPECT_EQ(world->getNumDifferentiableParameters(), 0u);

  EXPECT_NO_THROW(world->addDifferentiableParameter(
      *sphere, sx::PhysicalParameter::INERTIA));
  EXPECT_NO_THROW(world->addDifferentiableParameter(
      *sphere, sx::PhysicalParameter::FRICTION));
  EXPECT_EQ(world->getNumDifferentiableParameters(), 2u);
}

//==============================================================================
// A non-differentiable World rejects registration.
TEST(DiffParameterJacobian, NonDifferentiableWorldThrows)
{
  sx::WorldOptions options;
  options.timeStep = kTimeStep;
  options.differentiable = false;
  sx::World world(options);
  auto sphere = world.addRigidBody("sphere");
  EXPECT_THROW(
      world.addDifferentiableParameter(sphere, sx::PhysicalParameter::MASS),
      sx::InvalidOperationException);
}
