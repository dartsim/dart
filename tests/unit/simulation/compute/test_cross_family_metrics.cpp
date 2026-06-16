/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the "BSD-style" License
 */

// PLAN-091 WP-091.24 -- cross-family metrics comparison (core acceptance).
//
// "The same scene runs under two rigid families and two multibody families
// producing comparable metric rows from one command." This test exercises that
// directly: a single reusable helper (`runSceneCollectMetrics`) builds a World
// with a given solver-family configuration, runs a shared scene builder, steps
// the World N times through the ordinary `World::step()` pipeline, and returns
// the solver-agnostic `compute::StepMetrics` snapshot. Driving the SAME scene
// through that one helper once per family produces one comparable metric row
// per family -- the rows are logged (RecordProperty) so the comparison reads as
// a table, and the assertions below gate physical consistency ACROSS families.
//
// The assertions deliberately do NOT demand bit-equality across families:
// different solver families legitimately differ numerically. Instead they check
//   (1) every family produces FINITE metrics (no NaN/Inf);
//   (2) each family individually honors the scene's conserved invariant within
//       a physically-justified, non-flaky tolerance (momentum for the rigid
//       collision, bounded energy drift for the pendulum);
//   (3) the families AGREE on the same scene's gross outcome within a sane
//       tolerance (final momenta / final energies close across families).
//
// This is an additive test (no library code changes), so the default-step
// goldens remain bit-identical.

#include <dart/simulation/body/collision_shape.hpp>
#include <dart/simulation/body/rigid_body.hpp>
#include <dart/simulation/body/rigid_body_options.hpp>
#include <dart/simulation/compute/world_step_profile.hpp>
#include <dart/simulation/multibody/multibody.hpp>
#include <dart/simulation/multibody/multibody_options.hpp>
#include <dart/simulation/world.hpp>
#include <dart/simulation/world_options.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include <functional>
#include <sstream>
#include <string>
#include <vector>

#include <cmath>

namespace {

namespace sx = dart::simulation;
namespace sxc = dart::simulation::compute;

//==============================================================================
// The reusable "one command per family" helper.
//
// `configureWorld` applies the family-specific solver/integration selection to
// a freshly-constructed World (before bodies exist, where the selectors are
// legal); `buildScene` adds the shared bodies; the helper then enters
// simulation mode, steps `steps` times through the ordinary pipeline, and
// returns the post-roll `StepMetrics`. The same `buildScene` is reused for
// every family so each family runs the identical scene -- only the configure
// step differs, which is exactly the cross-family comparison the packet wants.
//
// Test-scoped on purpose: it touches only the existing public facade
// (`World`, `WorldOptions`, `computeStepMetrics`) and adds no public surface,
// so it does not affect `check-api-boundaries`.
sxc::StepMetrics runSceneCollectMetrics(
    const std::function<void(sx::WorldOptions&)>& configureOptions,
    const std::function<void(sx::World&)>& configureWorld,
    const std::function<void(sx::World&)>& buildScene,
    int steps)
{
  sx::WorldOptions options;
  configureOptions(options);
  sx::World world(options);
  configureWorld(world);
  buildScene(world);
  world.enterSimulationMode();
  for (int k = 0; k < steps; ++k) {
    world.step();
  }
  return world.computeStepMetrics();
}

// One labeled metric row, so the per-family output reads as a comparable table.
struct FamilyRow
{
  std::string family;
  sxc::StepMetrics metrics;
};

std::string formatRow(const FamilyRow& row)
{
  std::ostringstream oss;
  oss << row.family << ": KE=" << row.metrics.kineticEnergy
      << " PE=" << row.metrics.potentialEnergy
      << " E=" << row.metrics.totalEnergy << " p=["
      << row.metrics.linearMomentum.x() << ", "
      << row.metrics.linearMomentum.y() << ", "
      << row.metrics.linearMomentum.z() << "] L=["
      << row.metrics.angularMomentum.x() << ", "
      << row.metrics.angularMomentum.y() << ", "
      << row.metrics.angularMomentum.z() << "]";
  return oss.str();
}

bool isFinite(const sxc::StepMetrics& m)
{
  return std::isfinite(m.kineticEnergy) && std::isfinite(m.potentialEnergy)
         && std::isfinite(m.totalEnergy) && m.linearMomentum.allFinite()
         && m.angularMomentum.allFinite();
}

//==============================================================================
// RIGID-FAMILY COMPARISON.
//
// Scene: two free spheres on a 1-D head-on course under ZERO gravity, with no
// static/kinematic bodies. The only forces are the internal contact impulses
// between the two bodies, so by Newton's third law TOTAL linear momentum is
// conserved through the collision regardless of which contact family resolves
// it -- a family-independent physical invariant, which makes it the right gate
// for a cross-family comparison. (Initial total momentum:
// m_a v_a + m_b v_b = 1*(1.5) + 2*(-0.5) = +0.5 along x.)
//
// Families: ContactSolverMethod::SequentialImpulse (the default Gauss-Seidel
// normal+friction solve) vs ContactSolverMethod::BoxedLcp (the pivoting boxed-
// LCP normal solve). Both run this sphere/sphere collision through the ordinary
// rigid-body contact stage and the same `World::step()`, so the scene genuinely
// executes under each. RigidBodySolver::Ipc is intentionally NOT run here: it
// is experimental and handles only free mesh-like rigid bodies through the
// internal rigid-IPC stage (see World::setRigidBodySolver docs / the PLAN-083
// bridge fixtures), so a sphere/sphere head-on scene would not exercise its
// contact path -- running it would risk a silent no-op, which this comparison
// is specifically built to avoid.
void buildRigidHeadOnCollision(sx::World& world)
{
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(1e-3);

  sx::RigidBodyOptions a;
  a.mass = 1.0;
  a.inertia = 0.1 * Eigen::Matrix3d::Identity();
  a.position = Eigen::Vector3d(-1.2, 0.0, 0.0);
  a.linearVelocity = Eigen::Vector3d(1.5, 0.0, 0.0);
  auto bodyA = world.addRigidBody("rigid_xfam_a", a);
  bodyA.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  sx::RigidBodyOptions b;
  b.mass = 2.0;
  b.inertia = 0.1 * Eigen::Matrix3d::Identity();
  b.position = Eigen::Vector3d(1.2, 0.0, 0.0);
  b.linearVelocity = Eigen::Vector3d(-0.5, 0.0, 0.0);
  auto bodyB = world.addRigidBody("rigid_xfam_b", b);
  bodyB.setCollisionShape(sx::CollisionShape::makeSphere(0.5));
}

TEST(CrossFamilyMetrics, RigidContactFamiliesAgreeOnConservedMomentum)
{
  // 1.4 m gap closing at 2 m/s (~0.7 s) plus settle margin: 1500 steps at
  // 1 ms guarantees the contact has fired and resolved for both families.
  constexpr int kSteps = 1500;

  // Initial momentum is +0.5 x by construction; conserved through the contact.
  const Eigen::Vector3d kExpectedMomentum(0.5, 0.0, 0.0);

  const auto noWorldConfig = [](sx::World&) {
  };

  std::vector<FamilyRow> rows;
  rows.push_back(
      {"rigid:sequential-impulse",
       runSceneCollectMetrics(
           [](sx::WorldOptions& o) {
             o.contactSolverMethod = sx::ContactSolverMethod::SequentialImpulse;
           },
           noWorldConfig,
           buildRigidHeadOnCollision,
           kSteps)});
  rows.push_back(
      {"rigid:boxed-lcp",
       runSceneCollectMetrics(
           [](sx::WorldOptions& o) {
             o.contactSolverMethod = sx::ContactSolverMethod::BoxedLcp;
           },
           noWorldConfig,
           buildRigidHeadOnCollision,
           kSteps)});

  std::ostringstream table;
  for (const auto& row : rows) {
    table << formatRow(row) << "\n";
  }
  RecordProperty("rigid_family_rows", table.str());

  // Initial total kinetic energy of the scene (for the dissipation check):
  // 0.5*1*1.5^2 + 0.5*2*0.5^2 = 1.125 + 0.25 = 1.375 J.
  constexpr double kInitialKineticEnergy = 1.375;

  for (const auto& row : rows) {
    // (1) Finite metrics: a broken family that NaNs out is caught immediately.
    ASSERT_TRUE(isFinite(row.metrics)) << row.family;

    // (2) Per-family conserved invariant: total linear momentum equals the
    // initial +0.5 x. Tolerance 1e-7 (absolute, on a 0.5-magnitude quantity =
    // ~2e-7 relative): the contact solve is an iterative impulse exchange, so
    // momentum is conserved to solver-residual round-off, not to machine
    // epsilon. 1e-7 sits comfortably above that residual for both families yet
    // is ~6 orders below the 0.5 signal, so a family that leaks momentum (a
    // real third-law bug) trips it.
    EXPECT_LT((row.metrics.linearMomentum - kExpectedMomentum).norm(), 1e-7)
        << row.family;

    // (3) The collision actually fired: a perfectly-inelastic restitution-0
    // head-on collision retains only the common-velocity kinetic energy
    // P^2/(2(m_a+m_b)) = 0.5^2/(2*3) ~= 0.0417 J, far below the 1.375 J input.
    // Requiring the final KE below half the input proves each family resolved a
    // real contact (not a no-op fly-through, which would keep all 1.375 J).
    EXPECT_LT(row.metrics.kineticEnergy, 0.5 * kInitialKineticEnergy)
        << row.family;
    // A passive contact can only remove kinetic energy, never inject it.
    EXPECT_LE(row.metrics.kineticEnergy, kInitialKineticEnergy + 1e-9)
        << row.family;
  }

  // Cross-family agreement on the SAME scene's gross outcome: both families
  // conserve the same momentum, so their final momenta agree to ~1e-7. (Each is
  // within 1e-7 of the exact +0.5 x above, so their difference is bounded by
  // 2e-7; 5e-7 leaves slack without being loose enough to miss a divergence.)
  EXPECT_LT(
      (rows[0].metrics.linearMomentum - rows[1].metrics.linearMomentum).norm(),
      5e-7);
  // Both families dissipate to the same near-inelastic end state, so their
  // final kinetic energies agree within 0.05 J (~3.6% of the 1.375 J input).
  // This is a gross-outcome agreement, not a numerical-identity claim: the two
  // contact formulations reach slightly different micro-states, so the band is
  // generous on purpose -- it catches a family that fails to dissipate (which
  // would differ by ~1 J) without flaking on the legitimate small spread.
  EXPECT_LT(
      std::abs(rows[0].metrics.kineticEnergy - rows[1].metrics.kineticEnergy),
      0.05);
}

//==============================================================================
// MULTIBODY-FAMILY COMPARISON.
//
// Scene: a 1-link pendulum (a single revolute joint, bob offset 1 m along x,
// released from rest at angle 1.0 rad) under gravity. This is a passive,
// frictionless articulated system, so its TOTAL mechanical energy is conserved;
// the only thing that varies across integration families is how well each
// preserves it. The setup mirrors the pendulum pattern in
// test_variational_integration.cpp.
//
// Families: multibody integrationFamily "semi-implicit" (default articulated-
// body forward dynamics) vs "variational integrator" (the discrete-mechanics
// integrator). Both run the SAME pendulum through the ordinary `World::step()`
// pipeline -- the variational integrator is reachable as a step-path
// substitution via setMultibodyOptions (see VariationalIntegration.
// SelectableThroughWorldStep), so this genuinely exercises both families.
void buildPendulum(sx::World& world)
{
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world.setTimeStep(1e-3);

  auto robot = world.addMultibody("xfam_pendulum");
  auto base = robot.addLink("base");
  Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
  offset.translation() = Eigen::Vector3d(1.0, 0.0, 0.0);
  sx::JointSpec spec;
  spec.name = "hinge";
  spec.type = sx::JointType::Revolute;
  spec.axis = Eigen::Vector3d::UnitY();
  spec.transformFromParent = offset;
  auto bob = robot.addLink("bob", base, spec);
  bob.setMass(1.0);
  bob.setInertia(Eigen::Vector3d(0.05, 0.05, 0.05).asDiagonal());
}

// Builds + enters a pendulum World, releases it from the raised rest pose, and
// returns the per-family metrics at release (energy0) and after `steps`. The
// release-from-rest must happen AFTER enterSimulationMode (the joint pose is
// set on the live structure), so this scene needs its own runner rather than
// the generic helper -- but it is the same shape: configure family, build,
// step, read metrics.
struct PendulumRollout
{
  sxc::StepMetrics atRelease;
  sxc::StepMetrics afterRoll;
};

PendulumRollout runPendulum(
    sx::MultibodyIntegrationFamily integrationFamily, int steps)
{
  sx::World world;
  buildPendulum(world);
  world.setMultibodyOptions({.integrationFamily = integrationFamily});
  world.enterSimulationMode();

  // Release from a raised position at rest (zero kinetic energy at t=0, so the
  // total energy is purely the gravitational potential of the raised bob).
  auto robot = world.getMultibody("xfam_pendulum").value();
  for (auto joint : robot.getJoints()) {
    if (joint.getDOFCount() > 0) {
      joint.setPosition(Eigen::VectorXd::Constant(1, 1.0));
    }
  }
  world.updateKinematics();

  PendulumRollout out;
  out.atRelease = world.computeStepMetrics();
  for (int k = 0; k < steps; ++k) {
    world.step();
  }
  out.afterRoll = world.computeStepMetrics();
  return out;
}

TEST(CrossFamilyMetrics, MultibodyIntegrationFamiliesAgreeOnConservedEnergy)
{
  // 20000 steps at 1 ms = 20 s of pendulum swing -- many full periods, enough
  // for any secular (non-bounded) energy drift to manifest. Matches the horizon
  // the VariationalIntegration.SelectableThroughWorldStep gate already uses.
  constexpr int kSteps = 20000;

  std::vector<std::pair<std::string, PendulumRollout>> runs
      = {{"multibody:semi-implicit",
          runPendulum(sx::MultibodyIntegrationFamily::SemiImplicit, kSteps)},
         {"multibody:variational integrator",
          runPendulum(sx::MultibodyIntegrationFamily::Variational, kSteps)}};

  std::ostringstream table;
  for (const auto& [family, run] : runs) {
    table << family << " @release: " << formatRow({family, run.atRelease})
          << "\n"
          << family << " @" << kSteps << ": "
          << formatRow({family, run.afterRoll}) << "\n";
  }
  RecordProperty("multibody_family_rows", table.str());

  for (const auto& [family, run] : runs) {
    // (1) Finite metrics for both the release snapshot and the rolled state.
    ASSERT_TRUE(isFinite(run.atRelease)) << family;
    ASSERT_TRUE(isFinite(run.afterRoll)) << family;

    // Sanity: the raised pendulum has non-trivial mechanical energy to
    // conserve. The multibody kinetic/potential split is taken from the
    // variational integrator's own forward-kinematics terms (PLAN-091 WP-091.24
    // fix), so it is physical: released from rest, the kinetic energy is ~0 and
    // the potential carries the total. (An earlier version derived the split
    // from the stored comps::LinkState transform cache, whose frame gauge
    // differed, giving a non-physical negative kinetic energy at rest.)
    ASSERT_GT(std::abs(run.atRelease.totalEnergy), 1e-6) << family;
    EXPECT_NEAR(run.atRelease.kineticEnergy, 0.0, 1e-6)
        << family << " released-from-rest multibody kinetic energy must be ~0";
    EXPECT_GE(run.afterRoll.kineticEnergy, -1e-9)
        << family << " physical kinetic energy must be non-negative";

    // (2) Per-family conserved invariant: total mechanical energy is preserved
    // across the 20 s swing. Tolerance 1% relative drift. Both families are
    // designed to bound energy error -- the variational integrator is
    // symplectic (bounded oscillation, no secular drift) and semi-implicit
    // Euler at 1 ms on this light pendulum stays well inside 1% -- so 1%
    // generously covers both without flaking, yet a family that pumps or bleeds
    // energy (a real integrator bug) would blow far past it.
    const double drift
        = std::abs(run.afterRoll.totalEnergy - run.atRelease.totalEnergy)
          / std::abs(run.atRelease.totalEnergy);
    EXPECT_LT(drift, 1e-2) << family << " relative energy drift " << drift;
  }

  // (3) Cross-family agreement: both integrators model the SAME pendulum, so
  // their conserved total energies (set by the identical release pose) agree to
  // floating-point round-off at release...
  EXPECT_NEAR(
      runs[0].second.atRelease.totalEnergy,
      runs[1].second.atRelease.totalEnergy,
      1e-9);
  // ...and stay within 2% of each other after 20 s. The bound is the sum of the
  // two families' individual 1% conservation budgets (each may drift up to 1%
  // in opposite directions), so 2% is the physically-correct cross-family band:
  // tight enough that a family which diverges (e.g. one losing 10% of its
  // energy) is caught, loose enough that the legitimate symplectic-vs-Euler
  // spread never flakes.
  const double e0 = runs[0].second.afterRoll.totalEnergy;
  const double e1 = runs[1].second.afterRoll.totalEnergy;
  EXPECT_LT(
      std::abs(e0 - e1) / std::abs(runs[0].second.atRelease.totalEnergy), 2e-2)
      << "cross-family final energy spread";
}

} // namespace
