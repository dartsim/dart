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

// FD-vs-analytic correctness gate for the contact single-step Jacobian
// (PLAN-110 WS2): the analytic boxed-LCP contact gradient (A_CC⁻¹ implicit
// differentiation, FD-of-terms for the smooth geometric inputs). The analytic
// Jacobian is compared against a finite-difference-of-STEP that re-runs the
// full forward boxed-LCP step (collision requery + pivoting LCP solve), so the
// two paths are independent. Registered only when DART_BUILD_DIFF is ON.

#include <dart/simulation/body/collision_shape.hpp>
#include <dart/simulation/body/contact.hpp>
#include <dart/simulation/body/rigid_body.hpp>
#include <dart/simulation/body/rigid_body_options.hpp>
#include <dart/simulation/detail/contact_jacobians.hpp>
#include <dart/simulation/detail/world_registry_access.hpp>
#include <dart/simulation/diff/step_derivatives.hpp>
#include <dart/simulation/world.hpp>
#include <dart/simulation/world_options.hpp>

#include <dart/common/memory_allocator.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include <array>
#include <limits>
#include <memory>
#include <string_view>

#include <cmath>

namespace sx = dart::simulation;

namespace {

constexpr double kSphereRadius = 0.5;
constexpr double kSphereMass = 2.0;
constexpr double kTimeStep = 1e-3;

class CountingMemoryAllocator final : public dart::common::MemoryAllocator
{
public:
  [[nodiscard]] std::string_view getType() const override
  {
    return "CountingMemoryAllocator";
  }

  [[nodiscard]] void* allocate(size_t bytes) noexcept override
  {
    ++allocationCount;
    return dart::common::MemoryAllocator::GetDefault().allocate(bytes);
  }

  [[nodiscard]] void* allocate(size_t bytes, size_t alignment) noexcept override
  {
    ++alignedAllocationCount;
    return dart::common::MemoryAllocator::GetDefault().allocate(
        bytes, alignment);
  }

  void deallocate(void* pointer, size_t bytes) override
  {
    ++deallocationCount;
    dart::common::MemoryAllocator::GetDefault().deallocate(pointer, bytes);
  }

  void deallocate(void* pointer, size_t bytes, size_t alignment) override
  {
    ++alignedDeallocationCount;
    dart::common::MemoryAllocator::GetDefault().deallocate(
        pointer, bytes, alignment);
  }

  std::size_t allocationCount = 0;
  std::size_t deallocationCount = 0;
  std::size_t alignedAllocationCount = 0;
  std::size_t alignedDeallocationCount = 0;
};

//==============================================================================
// Translational rigid-body state: x = [position; linear velocity], control =
// applied force. Angular state is held fixed (the test scene keeps the contact
// lever arm parallel to the normal, so the angular DOFs decouple).
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
// Build a sphere-on-static-box-ground scene with Coulomb friction. The sphere
// contact lever arm is along the normal, so the angular DOFs decouple and the
// translational reduction stays exact.
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
// Multi-contact scene: a unit box resting flat on the static box ground. The
// box/ground manifold has four corner contact points, so the LCP carries
// several clamping normal rows. The corner lever arms are NOT parallel to the
// (vertical) normal, so the angular rows of J are exercised; by symmetry the
// net normal coupling cancels, but the multi-contact A_CC must still classify
// and invert the >1-row clamping block correctly.
constexpr double kBoxMass = 1.0;
constexpr double kBoxHalfExtent = 0.5;
// Box bottom at z = boxZ - kBoxHalfExtent; ground top at z = 0. Rest height
// gives a ~5e-5 penetration, below the 1e-4 positional-correction allowance.
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
  boxOptions.mass = kBoxMass;
  boxOptions.position = Eigen::Vector3d(0.0, 0.0, kBoxRestZ);
  auto box = world->addRigidBody("box", boxOptions);
  box.setCollisionShape(
      sx::CollisionShape::makeBox(
          Eigen::Vector3d(kBoxHalfExtent, kBoxHalfExtent, kBoxHalfExtent)));
  box.setFriction(0.0);

  return world;
}

//==============================================================================
// Rotational scene: a unit box lying flush on a static ground that is tilted by
// `kTiltAngle` about the world y axis (the box is rotated by the same angle so
// its bottom face is parallel to the slope). The four corner contacts track the
// body (their lever arms are fixed in the body frame), but the tilted normal
// makes the corner arms NON-parallel to the normal with a NET rotational
// coupling that does NOT cancel: the post-step linear velocity picks up an
// in-plane component from the normal-velocity perturbation (the off-diagonal
// ∂vx'/∂vz term), the hallmark of an off-COM contact exciting rotation. The
// configuration is stable (flush face, fixed orientation, sub-allowance
// penetration), so the active set does not flip under the FD perturbations.
constexpr double kTiltAngle = 0.15;

Eigen::Matrix3d tiltRotation()
{
  return Eigen::AngleAxisd(kTiltAngle, Eigen::Vector3d::UnitY())
      .toRotationMatrix();
}

Eigen::Vector3d tiltNormal()
{
  return tiltRotation() * Eigen::Vector3d::UnitZ();
}

// Box center for a flush rest with ~5e-5 penetration along the tilted normal.
Eigen::Vector3d tiltedBoxRestPosition()
{
  return tiltRotation() * Eigen::Vector3d(0.0, 0.0, kBoxHalfExtent)
         - 5e-5 * tiltNormal();
}

std::unique_ptr<sx::World> buildTiltedBoxScene()
{
  sx::WorldOptions options;
  options.timeStep = kTimeStep;
  options.gravity = Eigen::Vector3d(0.0, 0.0, -9.81);
  options.differentiable = true;
  options.contactSolverMethod = sx::ContactSolverMethod::BoxedLcp;
  auto world = std::make_unique<sx::World>(options);

  const Eigen::Matrix3d rotation = tiltRotation();

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  auto ground = world->addRigidBody("ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(5.0, 5.0, 0.5)));
  ground.setFriction(0.0);
  auto groundHandle = world->getRigidBody("ground");
  Eigen::Isometry3d groundPose = Eigen::Isometry3d::Identity();
  groundPose.linear() = rotation;
  groundPose.translation() = rotation * Eigen::Vector3d(0.0, 0.0, -0.5);
  groundHandle->setTransform(groundPose);

  sx::RigidBodyOptions boxOptions;
  boxOptions.mass = kBoxMass;
  auto box = world->addRigidBody("box", boxOptions);
  box.setCollisionShape(
      sx::CollisionShape::makeBox(
          Eigen::Vector3d(kBoxHalfExtent, kBoxHalfExtent, kBoxHalfExtent)));
  box.setFriction(0.0);
  // The box orientation is set per-sample by applyTiltedBoxState().

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
void applyBoxState(sx::World& world, const BodyState& state)
{
  auto box = world.getRigidBody("box");
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = state.position;
  box->setTransform(pose);
  box->setLinearVelocity(state.velocity);
  box->setForce(state.force);
}

//==============================================================================
// Like applyBoxState() but holds the box rotated flush with the tilted slope.
void applyTiltedBoxState(sx::World& world, const BodyState& state)
{
  auto box = world.getRigidBody("box");
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() = tiltRotation();
  pose.translation() = state.position;
  box->setTransform(pose);
  box->setLinearVelocity(state.velocity);
  box->setForce(state.force);
}

//==============================================================================
Eigen::VectorXd readBoxState(sx::World& world)
{
  auto box = world.getRigidBody("box");
  Eigen::VectorXd x(6);
  x.head<3>() = box->getTranslation();
  x.tail<3>() = box->getLinearVelocity();
  return x;
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

using SceneBuilder = std::unique_ptr<sx::World> (*)();

//==============================================================================
// FD-of-step Jacobian: perturb each of the six state components by h, run one
// full forward step from a freshly built world, central-difference [q'; q̇'].
Eigen::MatrixXd finiteDifferenceStateJacobian(
    const BodyState& nominal, double h, SceneBuilder build = buildScene)
{
  Eigen::VectorXd x0(6);
  x0.head<3>() = nominal.position;
  x0.tail<3>() = nominal.velocity;

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

    auto plusWorld = build();
    applyState(*plusWorld, plus);
    plusWorld->step();
    const Eigen::VectorXd nextPlus = readState(*plusWorld);

    auto minusWorld = build();
    applyState(*minusWorld, minus);
    minusWorld->step();
    const Eigen::VectorXd nextMinus = readState(*minusWorld);

    jacobian.col(k) = (nextPlus - nextMinus) / (2.0 * h);
  }
  return jacobian;
}

//==============================================================================
Eigen::MatrixXd finiteDifferenceControlJacobian(
    const BodyState& nominal, double h, SceneBuilder build = buildScene)
{
  Eigen::MatrixXd jacobian(6, 3);
  for (Eigen::Index k = 0; k < 3; ++k) {
    BodyState plus = nominal;
    BodyState minus = nominal;
    plus.force[k] += h;
    minus.force[k] -= h;

    auto plusWorld = build();
    applyState(*plusWorld, plus);
    plusWorld->step();
    const Eigen::VectorXd nextPlus = readState(*plusWorld);

    auto minusWorld = build();
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

//==============================================================================
// Compute the analytic contact Jacobian at the world's current (pre-step) state
// via the detail entry point. Uses the live active set from collide().
sx::StepDerivatives analyticDerivatives(sx::World& world)
{
  const auto contacts = world.collide();
  return sx::detail::contactStepDerivatives(
      dart::simulation::detail::registryOf(world),
      contacts,
      world.getGravity(),
      world.getTimeStep());
}

//==============================================================================
// Generic FD-of-step state Jacobian for the body named "box", parameterized by
// the scene builder and the state-applier (so the rotational scene can hold the
// box rotated flush with the slope while perturbing only [pos; linvel]).
using BoxSceneBuilder = std::unique_ptr<sx::World> (*)();
using BoxStateApplier = void (*)(sx::World&, const BodyState&);

Eigen::MatrixXd finiteDifferenceBoxStateJacobian(
    const BodyState& nominal,
    double h,
    BoxSceneBuilder build,
    BoxStateApplier apply)
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

    auto plusWorld = build();
    apply(*plusWorld, plus);
    plusWorld->step();
    const Eigen::VectorXd nextPlus = readBoxState(*plusWorld);

    auto minusWorld = build();
    apply(*minusWorld, minus);
    minusWorld->step();
    const Eigen::VectorXd nextMinus = readBoxState(*minusWorld);

    jacobian.col(k) = (nextPlus - nextMinus) / (2.0 * h);
  }
  return jacobian;
}

} // namespace

//==============================================================================
// Sustained, firmly clamping contact: the sphere rests on the ground with a
// small stable penetration (below the positional-correction allowance, so the
// projection term is inactive) and a downward free velocity that the normal
// impulse clamps. The analytic A_CC⁻¹ Jacobian must match FD-of-step.
TEST(DiffContactJacobian, ClampingContactMatchesFiniteDifference)
{
  // Place the sphere with ~5e-5 penetration: deep enough that the contact is
  // present under every FD perturbation (max h = 1e-5), shallow enough that the
  // penetration stays below the 1e-4 positional-correction allowance
  // throughout.
  BodyState config;
  config.position = Eigen::Vector3d(0.0, 0.0, kSphereRadius - 5e-5);
  config.velocity = Eigen::Vector3d(0.0, 0.0, -0.2); // pressing into the ground
  config.force = Eigen::Vector3d::Zero();

  auto world = buildScene();
  applyState(*world, config);

  // The contact must be firmly clamping at this state.
  const auto contacts = world->collide();
  ASSERT_FALSE(contacts.empty()) << "expected an active sphere/ground contact";

  const sx::StepDerivatives analytic = analyticDerivatives(*world);
  ASSERT_EQ(analytic.stateJacobian.rows(), 6);
  ASSERT_EQ(analytic.stateJacobian.cols(), 6);
  ASSERT_EQ(analytic.controlJacobian.rows(), 6);
  ASSERT_EQ(analytic.controlJacobian.cols(), 3);

  const std::array<double, 3> sweep{1e-5, 1e-6, 1e-7};

  double bestStateError = std::numeric_limits<double>::infinity();
  for (const double h : sweep) {
    const Eigen::MatrixXd fd = finiteDifferenceStateJacobian(config, h);
    const double error = relativeError(analytic.stateJacobian, fd);
    bestStateError = std::min(bestStateError, error);
  }
  EXPECT_LT(bestStateError, 1e-4)
      << "best FD-of-step state relative error: " << bestStateError;

  double bestControlError = std::numeric_limits<double>::infinity();
  for (const double h : sweep) {
    const Eigen::MatrixXd fd = finiteDifferenceControlJacobian(config, h);
    const double error = relativeError(analytic.controlJacobian, fd);
    bestControlError = std::min(bestControlError, error);
  }
  EXPECT_LT(bestControlError, 1e-4)
      << "best FD-of-step control relative error: " << bestControlError;

  // Physical sanity: with the contact clamping the downward normal velocity,
  // the post-step vertical velocity is insensitive to the incoming vertical
  // velocity (∂q̇'_z/∂q̇_z ≈ 0), unlike the horizontal axes which free-fall (≈
  // 1).
  EXPECT_NEAR(analytic.stateJacobian(5, 5), 0.0, 1e-6); // ∂vz'/∂vz
  EXPECT_NEAR(analytic.stateJacobian(3, 3), 1.0, 1e-6); // ∂vx'/∂vx
}

//==============================================================================
TEST(DiffContactJacobian, DetailOverloadUsesProvidedAllocator)
{
  BodyState config;
  config.position = Eigen::Vector3d(0.0, 0.0, kSphereRadius - 5e-5);
  config.velocity = Eigen::Vector3d(1.0, 0.0, -0.2);
  config.force = Eigen::Vector3d::Zero();

  auto world = buildFrictionScene();
  applyState(*world, config);

  const auto contacts = world->collide();
  ASSERT_FALSE(contacts.empty()) << "expected an active sphere/ground contact";

  CountingMemoryAllocator allocator;
  const sx::StepDerivatives analytic = sx::detail::contactStepDerivatives(
      dart::simulation::detail::registryOf(*world),
      contacts,
      world->getGravity(),
      world->getTimeStep(),
      sx::ContactGradientMode::Analytic,
      allocator);

  EXPECT_EQ(analytic.stateJacobian.rows(), 6);
  EXPECT_EQ(analytic.stateJacobian.cols(), 6);
  EXPECT_EQ(analytic.controlJacobian.rows(), 6);
  EXPECT_EQ(analytic.controlJacobian.cols(), 3);

  EXPECT_GT(allocator.allocationCount + allocator.alignedAllocationCount, 0u);
  EXPECT_EQ(allocator.deallocationCount, allocator.allocationCount);
  EXPECT_EQ(
      allocator.alignedDeallocationCount, allocator.alignedAllocationCount);
}

//==============================================================================
// Separating contact: the sphere is clear of the ground, so there is no active
// normal constraint and the contact Jacobian must reduce to the contact-free
// (free-fall) single-step Jacobian.
TEST(DiffContactJacobian, SeparatingContactMatchesContactFreeJacobian)
{
  BodyState config;
  config.position = Eigen::Vector3d(0.0, 0.0, 1.0); // well above the ground
  config.velocity = Eigen::Vector3d(0.1, -0.2, 0.3);
  config.force = Eigen::Vector3d(0.5, 0.0, 0.0);

  auto world = buildScene();
  applyState(*world, config);

  const auto contacts = world->collide();
  ASSERT_TRUE(contacts.empty()) << "expected no contact when separated";

  const sx::StepDerivatives analytic = analyticDerivatives(*world);

  // Closed-form contact-free (free-fall) Jacobian for q' = q + Δt q̇',
  // q̇' = q̇ + Δt (F/m + g):
  //   ∂q'/∂q = I,   ∂q'/∂q̇ = Δt I
  //   ∂q̇'/∂q = 0,  ∂q̇'/∂q̇ = I
  //   ∂q'/∂F = (Δt²/m) I,  ∂q̇'/∂F = (Δt/m) I
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

  EXPECT_LT(relativeError(analytic.stateJacobian, expectedState), 1e-9);
  EXPECT_LT(relativeError(analytic.controlJacobian, expectedControl), 1e-9);

  // Cross-check the separating analytic against FD-of-step (no contact -> the
  // forward step is plain free fall, so this is an additional independent
  // gate).
  const Eigen::MatrixXd fdState = finiteDifferenceStateJacobian(config, 1e-6);
  EXPECT_LT(relativeError(analytic.stateJacobian, fdState), 1e-4);
}

//==============================================================================
// Steadily sliding contact (Coulomb friction): the sphere presses into a
// frictional ground (firmly clamping normal) and slides along +x fast enough
// that friction is fully saturated (|f_t| = mu f_n). This is a STABLE regime,
// well past the cone edge: the friction row aligned with the slide is sliding
// (upper-bounded, mapped to its normal row), the orthogonal friction row sticks
// (interior), and the active set does not flip under the FD perturbations. The
// stick-slip transition itself is a known non-smooth boundary (a subgradient
// kink) and is deliberately out of scope. The analytic gradient -- normal
// A_CC⁻¹ implicit differentiation extended with the sliding-friction
// upper-bound mapping -- must match FD-of-step.
TEST(DiffContactJacobian, SlidingFrictionMatchesFiniteDifference)
{
  // ~5e-5 penetration (below the 1e-4 positional-correction allowance), a
  // downward velocity that the normal impulse clamps, and a strong +x slide so
  // friction saturates. The vertical normal force here is dominated by the
  // gravity/velocity term: mu f_n >> the tangential momentum a single step can
  // remove, so the contact slides steadily rather than sticking.
  BodyState config;
  config.position = Eigen::Vector3d(0.0, 0.0, kSphereRadius - 5e-5);
  config.velocity = Eigen::Vector3d(1.0, 0.0, -0.2);
  config.force = Eigen::Vector3d::Zero();

  auto world = buildFrictionScene();
  applyState(*world, config);

  const auto contacts = world->collide();
  ASSERT_FALSE(contacts.empty()) << "expected an active sphere/ground contact";

  const sx::StepDerivatives analytic = analyticDerivatives(*world);
  ASSERT_EQ(analytic.stateJacobian.rows(), 6);
  ASSERT_EQ(analytic.stateJacobian.cols(), 6);
  ASSERT_EQ(analytic.controlJacobian.rows(), 6);
  ASSERT_EQ(analytic.controlJacobian.cols(), 3);

  const std::array<double, 3> sweep{1e-5, 1e-6, 1e-7};

  double bestStateError = std::numeric_limits<double>::infinity();
  for (const double h : sweep) {
    const Eigen::MatrixXd fd
        = finiteDifferenceStateJacobian(config, h, buildFrictionScene);
    const double error = relativeError(analytic.stateJacobian, fd);
    bestStateError = std::min(bestStateError, error);
  }
  EXPECT_LT(bestStateError, 1e-4)
      << "best FD-of-step state relative error: " << bestStateError;

  double bestControlError = std::numeric_limits<double>::infinity();
  for (const double h : sweep) {
    const Eigen::MatrixXd fd
        = finiteDifferenceControlJacobian(config, h, buildFrictionScene);
    const double error = relativeError(analytic.controlJacobian, fd);
    bestControlError = std::min(bestControlError, error);
  }
  EXPECT_LT(bestControlError, 1e-4)
      << "best FD-of-step control relative error: " << bestControlError;

  // Physical sanity. The normal velocity is clamped (∂vz'/∂vz ≈ 0). The slide
  // direction (+x) is saturated friction f_t = mu f_n, which is independent of
  // the slide speed, so the post-step forward velocity is insensitive to small
  // changes in vx (∂vx'/∂vx ≈ 1). But the saturated friction is coupled to the
  // normal force, so the forward velocity DOES depend on the normal velocity
  // (∂vx'/∂vz != 0) -- the signature of sliding-friction coupling through the
  // upper-bound mapping.
  EXPECT_NEAR(analytic.stateJacobian(5, 5), 0.0, 1e-6); // ∂vz'/∂vz clamped
  EXPECT_NEAR(analytic.stateJacobian(3, 3), 1.0, 1e-6); // ∂vx'/∂vx (saturated)
  EXPECT_GT(std::abs(analytic.stateJacobian(3, 5)), 1e-3); // ∂vx'/∂vz coupling
}

//==============================================================================
// Multi-point clamping contact: a box rests flat on the ground, producing four
// corner contact points (several clamping normal rows). The active set is
// firmly clamping (sub-allowance penetration, downward approach), with no
// tied-set ambiguity in the net velocity update. The analytic multi-contact
// A_CC⁻¹ gradient must match FD-of-step.
TEST(DiffContactJacobian, MultiContactBoxMatchesFiniteDifference)
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

  const sx::StepDerivatives analytic = analyticDerivatives(*world);
  ASSERT_EQ(analytic.stateJacobian.rows(), 6);
  ASSERT_EQ(analytic.stateJacobian.cols(), 6);

  const std::array<double, 3> sweep{1e-5, 1e-6, 1e-7};

  double bestStateError = std::numeric_limits<double>::infinity();
  for (const double h : sweep) {
    const Eigen::MatrixXd fd = finiteDifferenceBoxStateJacobian(
        config, h, buildBoxScene, applyBoxState);
    bestStateError
        = std::min(bestStateError, relativeError(analytic.stateJacobian, fd));
  }
  EXPECT_LT(bestStateError, 1e-4)
      << "best FD-of-step state relative error: " << bestStateError;

  GTEST_LOG_(INFO) << "multi-contact box FD-of-step state rel error: "
                   << bestStateError;

  // Physical sanity: the multi-point flat clamp removes the downward normal
  // velocity (∂vz'/∂vz ≈ 0) while the in-plane axes free-fall (≈ 1).
  EXPECT_NEAR(analytic.stateJacobian(5, 5), 0.0, 1e-6); // ∂vz'/∂vz clamped
  EXPECT_NEAR(analytic.stateJacobian(3, 3), 1.0, 1e-6); // ∂vx'/∂vx free
}

//==============================================================================
// Rotational (off-COM) contact: a box lies flush on a ground tilted about y, so
// its four corner lever arms are NOT parallel to the (tilted) normal and excite
// the angular DOFs. The arms track the body, so the active set stays frozen
// under the FD perturbations; the tilted normal produces a NET rotational
// coupling that does not cancel. The analytic gradient -- full 6-DOF Delassus
// with the screw-axis angular rows of J, reduced to the translational output --
// must match FD-of-step.
TEST(DiffContactJacobian, RotationalContactMatchesFiniteDifference)
{
  BodyState config;
  config.position = tiltedBoxRestPosition();
  config.velocity = -0.2 * tiltNormal(); // pressing into the slope
  config.force = Eigen::Vector3d::Zero();

  auto world = buildTiltedBoxScene();
  applyTiltedBoxState(*world, config);

  const auto contacts = world->collide();
  ASSERT_GT(contacts.size(), 1u)
      << "expected a multi-point tilted box/ground face contact";

  const sx::StepDerivatives analytic = analyticDerivatives(*world);
  ASSERT_EQ(analytic.stateJacobian.rows(), 6);
  ASSERT_EQ(analytic.stateJacobian.cols(), 6);

  const std::array<double, 3> sweep{1e-5, 1e-6, 1e-7};

  double bestStateError = std::numeric_limits<double>::infinity();
  for (const double h : sweep) {
    const Eigen::MatrixXd fd = finiteDifferenceBoxStateJacobian(
        config, h, buildTiltedBoxScene, applyTiltedBoxState);
    bestStateError
        = std::min(bestStateError, relativeError(analytic.stateJacobian, fd));
  }
  EXPECT_LT(bestStateError, 1e-4)
      << "best FD-of-step state relative error: " << bestStateError;

  GTEST_LOG_(INFO) << "rotational (tilted box) FD-of-step state rel error: "
                   << bestStateError;

  // Physical signature of off-COM rotational coupling: a normal-velocity
  // perturbation produces an in-plane velocity response (the off-diagonal
  // ∂vx'/∂vz term is non-negligible), which only arises when the angular rows
  // of J are carried through the contact solve.
  EXPECT_GT(std::abs(analytic.stateJacobian(3, 5)), 1e-2)
      << "expected off-COM ∂vx'/∂vz coupling from the angular rows of J";
}
