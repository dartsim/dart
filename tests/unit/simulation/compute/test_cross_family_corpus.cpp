/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the "BSD-style" License
 */

// PLAN-091 WP-091.24 -- cross-family metrics CORPUS harness (additive slice).
//
// Where `test_cross_family_metrics.cpp` hand-codes two cross-family comparisons
// (a rigid head-on collision and a 1-link pendulum), this file generalizes that
// pattern into a small, registered SCENE CORPUS driven by ONE harness:
//
//   * A registry of named scene cases (`kCorpus`). Each case declares a scene
//     id, a scene-builder callable, the family AXIS it exercises, the families
//     to run on that axis, the number of steps, and the per-case physical
//     invariant used to derive the assertion. The two solver axes are modeled
//     EXPLICITLY (`FamilyAxis::RigidContactMethod` vs
//     `FamilyAxis::MultibodyIntegration`) because they are different axes and
//     are configured through different facade entry points.
//
//   * One harness test (`CrossFamilyCorpus.AllScenesRunUnderEveryFamily`) that
//     iterates the registry, runs EACH declared family of EACH case from a
//     single loop, collects the solver-agnostic `compute::StepMetrics` row,
//     logs it (RecordProperty) tagged `<scene>:<family>`, and asserts
//       (1) finite metrics,
//       (2) the case's declared per-family physical invariant within a robust,
//           conservation-budget-derived tolerance,
//       (3) cross-family agreement on the case's gross outcome within a
//           physically-justified band,
//       (4) a per-(scene,family) "actually ran" signal (a moved/settled body or
//           a non-trivial energy change), so no family can silently no-op a
//           scene and still pass.
//
// This is "the same scene under multiple families producing comparable rows
// from one command." It is purely additive test code: it touches only the
// existing public facade (`World`, `WorldOptions`, `setMultibodyOptions`,
// `computeStepMetrics`) and adds no public surface, so `check-api-boundaries`
// and the default-step goldens are unaffected.
//
// Tolerances are physically derived and commented per case, mirroring the
// rationale style of the sibling test; none demand bit-equality across families
// (different solver families legitimately differ numerically).

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
// Registry vocabulary.
//==============================================================================

// The two solver axes a case can exercise. They are genuinely different axes:
// the rigid-contact axis is selected via `WorldOptions.contactSolverMethod`
// before bodies exist; the multibody-integration axis is selected via
// `World::setMultibodyOptions({.integrationFamily = ...})`. Modeling the axis
// explicitly lets the harness configure each family correctly and lets a case
// declare which axis it lives on.
enum class FamilyAxis
{
  RigidContactMethod,
  MultibodyIntegration,
};

// The per-case physical expectation that derives the assertion. Each case picks
// the invariant that is family-independent for its scene, so it is the right
// gate for a cross-family comparison.
enum class Invariant
{
  // Total linear momentum is conserved (no external forces; only internal
  // contact impulses, which obey Newton's third law).
  ConservesLinearMomentum,
  // Total mechanical energy is conserved (passive, frictionless articulated
  // system; the only cross-family variation is integrator energy behavior).
  ConservesTotalEnergy,
  // The scene reaches a static resting state under gravity on a fixed support;
  // the cross-family gate is the final rest pose, not a conserved quantity
  // (gravity + the static support do external work, so momentum/energy are not
  // conserved here -- the contact must arrest the fall identically).
  SettlesOnStaticSupport,
};

// A "this family actually advanced the scene" signal. Avoids a family silently
// no-opping a scene yet passing a conservation check (a frozen scene trivially
// conserves everything). What the signal MEANS depends on the scene; each case
// supplies a callable that reads it from the live, post-roll World.
using RanSignalFn = std::function<double(sx::World&)>;

// A scene-builder: add bodies/multibody to a freshly-configured World (which
// already has the family-axis selection applied). Mirrors the sibling test's
// builder shape exactly.
using SceneBuilderFn = std::function<void(sx::World&)>;

// Optional post-enterSimulationMode setup (e.g. release a pendulum from a
// raised pose, which must happen on the live structure). Empty for scenes that
// are fully specified at build time.
using PostEnterFn = std::function<void(sx::World&)>;

struct SceneCase
{
  std::string id;
  FamilyAxis axis;
  std::vector<std::string> families;
  int steps = 0;
  Invariant invariant = Invariant::ConservesLinearMomentum;
  SceneBuilderFn build;
  PostEnterFn postEnter; // may be nullptr/empty
  RanSignalFn ranSignal; // reads the "advanced" signal from the live World
  // Minimum magnitude of |ranSignal| change (from its at-release value) that
  // proves the family advanced the scene rather than no-opping it.
  double minRanDelta = 0.0;
  // Reference value the ranSignal is compared against. For settle cases this is
  // the expected rest value of the signal; for "moved/changed" cases it is the
  // at-release value captured before stepping.
  double ranReference = 0.0;
};

//==============================================================================
// Scene builders. These are the canonical corpus versions; they intentionally
// mirror the proven patterns from test_cross_family_metrics.cpp (rigid head-on,
// 1-link pendulum) and test_boxed_lcp_contact.cpp (sphere-on-ground drop) /
// test_variational_integration.cpp (2-link chain) without depending on those
// translation units.
//==============================================================================

// CASE 1 (rigid-contact axis): two free spheres on a 1-D head-on course under
// ZERO gravity. The only forces are internal contact impulses, so total linear
// momentum is conserved through the collision regardless of contact family.
// Initial total momentum m_a v_a + m_b v_b = 1*(1.5) + 2*(-0.5) = +0.5 x.
void buildRigidHeadOn(sx::World& world)
{
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(1e-3);

  sx::RigidBodyOptions a;
  a.mass = 1.0;
  a.inertia = 0.1 * Eigen::Matrix3d::Identity();
  a.position = Eigen::Vector3d(-1.2, 0.0, 0.0);
  a.linearVelocity = Eigen::Vector3d(1.5, 0.0, 0.0);
  auto bodyA = world.addRigidBody("headon_a", a);
  bodyA.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  sx::RigidBodyOptions b;
  b.mass = 2.0;
  b.inertia = 0.1 * Eigen::Matrix3d::Identity();
  b.position = Eigen::Vector3d(1.2, 0.0, 0.0);
  b.linearVelocity = Eigen::Vector3d(-0.5, 0.0, 0.0);
  auto bodyB = world.addRigidBody("headon_b", b);
  bodyB.setCollisionShape(sx::CollisionShape::makeSphere(0.5));
}

// CASE 2 (rigid-contact axis): a sphere (radius 0.5) dropped from z = 1.0 onto
// a static box ground whose top face is at z = 0. The contact must arrest the
// fall so the sphere settles with its center near z = 0.5 (its radius). This is
// the SAME drop fixture the BoxedLcp contact tests use, so it genuinely runs
// under both contact families. (Frictionless / restitution-0, so it comes to
// rest rather than bouncing -- a clean, family-independent rest pose to
// compare.)
void buildSphereDrop(sx::World& world)
{
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world.setTimeStep(5e-3);

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
  auto ground = world.addRigidBody("drop_ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(5.0, 5.0, 0.5)));
  ground.setFriction(0.0);

  sx::RigidBodyOptions sphereOptions;
  sphereOptions.position = Eigen::Vector3d(0.0, 0.0, 1.0);
  auto sphere = world.addRigidBody("drop_sphere", sphereOptions);
  sphere.setCollisionShape(sx::CollisionShape::makeSphere(0.5));
  sphere.setFriction(0.0);
}

// CASE 3 (multibody-integration axis): a 1-link pendulum released from rest at
// 1.0 rad under gravity. Passive + frictionless => total mechanical energy is
// conserved; the only cross-family variation is each integrator's energy
// behavior. Mirrors the sibling test's pendulum.
void buildPendulum1(sx::World& world)
{
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world.setTimeStep(1e-3);

  auto robot = world.addMultibody("pend1");
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

// CASE 4 (multibody-integration axis): a 2-link chain (double pendulum)
// released from rest under gravity. A second articulated scene on the SAME
// integration axis -- richer (chaotic, coupled) than the 1-link case, so it is
// a stronger conservation gate. Both joints start at rest, so total energy is
// purely the gravitational potential of the raised links and must be conserved.
// Mirrors the double-pendulum pattern in test_variational_integration.cpp.
void buildPendulum2(sx::World& world)
{
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world.setTimeStep(1e-3);

  auto robot = world.addMultibody("pend2");
  auto base = robot.addLink("base");
  const auto offset = [](double x) {
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.translation() = Eigen::Vector3d(x, 0.0, 0.0);
    return transform;
  };

  sx::JointSpec spec1;
  spec1.name = "j1";
  spec1.type = sx::JointType::Revolute;
  spec1.axis = Eigen::Vector3d::UnitY();
  spec1.transformFromParent = offset(0.7);
  auto link1 = robot.addLink("link1", base, spec1);
  link1.setMass(1.5);
  link1.setInertia(Eigen::Vector3d(0.05, 0.08, 0.05).asDiagonal());

  sx::JointSpec spec2;
  spec2.name = "j2";
  spec2.type = sx::JointType::Revolute;
  spec2.axis = Eigen::Vector3d::UnitY();
  spec2.transformFromParent = offset(0.6);
  auto link2 = robot.addLink("link2", link1, spec2);
  link2.setMass(1.0);
  link2.setInertia(Eigen::Vector3d(0.04, 0.06, 0.04).asDiagonal());
}

//==============================================================================
// Post-enter releases + ran-signals.
//==============================================================================

// Release every revolute DOF of a named multibody to `angle` rad at rest. The
// release MUST run after enterSimulationMode (it sets the pose on the live
// structure), so it lives in the case's postEnter hook.
PostEnterFn makeReleaseAllJoints(std::string robotName, double angle)
{
  return [robotName = std::move(robotName), angle](sx::World& world) {
    auto robot = world.getMultibody(robotName).value();
    for (auto joint : robot.getJoints()) {
      if (joint.getDOFCount() > 0) {
        joint.setPosition(
            Eigen::VectorXd::Constant(joint.getDOFCount(), angle));
      }
    }
    world.updateKinematics();
  };
}

// Ran-signal for the multibody cases: the live total mechanical energy.
// Captured at release and after the roll; the per-family delta proves the
// integrator actually advanced the dynamics (a no-op would leave energy
// bit-identical).
double multibodyEnergySignal(sx::World& world)
{
  return world.computeStepMetrics().totalEnergy;
}

// Ran-signal for the rigid head-on case: the live total kinetic energy. The
// inelastic collision dissipates most of the input KE, so a large drop proves
// the contact fired (a fly-through no-op keeps all the input KE).
double rigidKineticSignal(sx::World& world)
{
  return world.computeStepMetrics().kineticEnergy;
}

// Ran-signal for the drop case: the sphere's center-of-mass height. It starts
// at z = 1.0 and must settle near z = 0.5; the drop proves the body fell AND
// the contact arrested it (a no-op contact would let it sink through the
// ground, a frozen scene would stay at z = 1.0).
double dropHeightSignal(sx::World& world)
{
  return world.getRigidBody("drop_sphere")
      .value()
      .getTransform()
      .translation()
      .z();
}

//==============================================================================
// The corpus.
//==============================================================================

std::vector<SceneCase> makeCorpus()
{
  std::vector<SceneCase> corpus;

  // CASE 1: rigid head-on collision, momentum conservation.
  {
    SceneCase c;
    c.id = "rigid_head_on";
    c.axis = FamilyAxis::RigidContactMethod;
    c.families = {"sequential-impulse", "boxed-lcp"};
    // 1.4 m gap closing at 2 m/s (~0.7 s) plus settle margin: 1500 steps at
    // 1 ms guarantees the contact fired and resolved for both families.
    c.steps = 1500;
    c.invariant = Invariant::ConservesLinearMomentum;
    c.build = buildRigidHeadOn;
    c.postEnter = nullptr;
    c.ranSignal = rigidKineticSignal;
    // Initial total KE = 0.5*1*1.5^2 + 0.5*2*0.5^2 = 1.375 J. A real contact
    // dissipates to the common-velocity KE P^2/(2*M) = 0.5^2/6 ~= 0.0417 J, so
    // KE must drop by well over half the input; require >= 0.6875 J removed.
    c.ranReference = 1.375; // initial KE
    c.minRanDelta = 0.6875; // >= half the input KE must be dissipated
    corpus.push_back(std::move(c));
  }

  // CASE 2: sphere-on-ground drop, settles on a static support.
  {
    SceneCase c;
    c.id = "sphere_drop";
    c.axis = FamilyAxis::RigidContactMethod;
    c.families = {"sequential-impulse", "boxed-lcp"};
    // Free-fall from z=1.0 to contact at z=0.5 is sqrt(2*0.5/9.81) ~= 0.32 s;
    // 600 steps at 5 ms = 3 s leaves ample time to fall, dissipate the impact,
    // and come fully to rest for both families.
    c.steps = 600;
    c.invariant = Invariant::SettlesOnStaticSupport;
    c.build = buildSphereDrop;
    c.postEnter = nullptr;
    c.ranSignal = dropHeightSignal;
    // Expected rest height = sphere radius (0.5). The signal starts at 1.0, so
    // a family that ran moved it by ~0.5; require it to have descended >= 0.4.
    c.ranReference = 0.5; // expected rest height of the COM
    c.minRanDelta = 0.4;  // must have descended >= 0.4 from z=1.0
    corpus.push_back(std::move(c));
  }

  // CASE 3: 1-link pendulum, total-energy conservation.
  {
    SceneCase c;
    c.id = "pendulum_1link";
    c.axis = FamilyAxis::MultibodyIntegration;
    c.families = {"semi-implicit", "variational integrator"};
    // 20000 steps at 1 ms = 20 s of swing -- many periods, enough for any
    // secular energy drift to manifest. Matches the horizon the sibling test
    // and VariationalIntegration.SelectableThroughWorldStep use.
    c.steps = 20000;
    c.invariant = Invariant::ConservesTotalEnergy;
    c.build = buildPendulum1;
    c.postEnter = makeReleaseAllJoints("pend1", 1.0);
    c.ranSignal = multibodyEnergySignal;
    // Advancement for the multibody cases is NOT proven by energy change (total
    // energy is conserved, so it barely moves). The harness instead proves a
    // real roll happened via post-roll kinetic energy: a frozen pendulum sits
    // at the raised rest pose with KE ~ 0, so KE > floor means it actually
    // swung. ranReference/minRanDelta are therefore unused on this axis.
    c.ranReference = 0.0;
    c.minRanDelta = 0.0;
    corpus.push_back(std::move(c));
  }

  // CASE 4: 2-link chain (double pendulum), total-energy conservation.
  {
    SceneCase c;
    c.id = "pendulum_2link";
    c.axis = FamilyAxis::MultibodyIntegration;
    c.families = {"semi-implicit", "variational integrator"};
    // 8 s at 1 ms. The double pendulum is chaotic, so a long horizon both
    // exercises the coupled dynamics hard and gives any energy drift time to
    // show. Shorter than the 1-link horizon because the richer dynamics make it
    // a stronger gate per second of sim time.
    c.steps = 8000;
    c.invariant = Invariant::ConservesTotalEnergy;
    c.build = buildPendulum2;
    c.postEnter = makeReleaseAllJoints("pend2", 0.8);
    c.ranSignal = multibodyEnergySignal;
    c.ranReference = 0.0;
    c.minRanDelta = 0.0;
    corpus.push_back(std::move(c));
  }

  return corpus;
}

//==============================================================================
// Harness machinery.
//==============================================================================

// One collected metric row, tagged scene:family, plus the "ran" evidence.
struct CorpusRow
{
  std::string sceneId;
  std::string family;
  sxc::StepMetrics atRelease; // pre-roll snapshot (after postEnter)
  sxc::StepMetrics afterRoll; // post-roll snapshot
  double ranAtRelease = 0.0;  // ranSignal at release
  double ranAfterRoll = 0.0;  // ranSignal after the roll
};

bool isFinite(const sxc::StepMetrics& m)
{
  return std::isfinite(m.kineticEnergy) && std::isfinite(m.potentialEnergy)
         && std::isfinite(m.totalEnergy) && m.linearMomentum.allFinite()
         && m.angularMomentum.allFinite();
}

std::string formatMetrics(const sxc::StepMetrics& m)
{
  std::ostringstream oss;
  oss << "KE=" << m.kineticEnergy << " PE=" << m.potentialEnergy
      << " E=" << m.totalEnergy << " p=[" << m.linearMomentum.x() << ", "
      << m.linearMomentum.y() << ", " << m.linearMomentum.z() << "] L=["
      << m.angularMomentum.x() << ", " << m.angularMomentum.y() << ", "
      << m.angularMomentum.z() << "]";
  return oss.str();
}

// Run ONE (case, family) pair through the ordinary World::step() pipeline and
// collect its row. This is the single place a family is configured + rolled;
// the harness calls it once per declared family of every case.
CorpusRow runCaseFamily(const SceneCase& c, const std::string& family)
{
  sx::WorldOptions options;
  if (c.axis == FamilyAxis::RigidContactMethod) {
    options.contactSolverMethod
        = (family == "boxed-lcp") ? sx::ContactSolverMethod::BoxedLcp
                                  : sx::ContactSolverMethod::SequentialImpulse;
  }
  sx::World world(options);

  if (c.axis == FamilyAxis::MultibodyIntegration) {
    // The integration family is selected on the World, before bodies exist.
    // The corpus identifies families by their documented label; map that label
    // to the typed selector here.
    world.setMultibodyOptions(
        {.integrationFamily
         = (family == "variational integrator")
               ? sx::MultibodyIntegrationFamily::Variational
               : sx::MultibodyIntegrationFamily::SemiImplicit});
  }

  c.build(world);
  world.enterSimulationMode();
  if (c.postEnter) {
    c.postEnter(world);
  }

  CorpusRow row;
  row.sceneId = c.id;
  row.family = family;
  row.atRelease = world.computeStepMetrics();
  row.ranAtRelease = c.ranSignal ? c.ranSignal(world) : 0.0;

  for (int k = 0; k < c.steps; ++k) {
    world.step();
  }

  row.afterRoll = world.computeStepMetrics();
  row.ranAfterRoll = c.ranSignal ? c.ranSignal(world) : 0.0;
  return row;
}

//==============================================================================
// The harness test: iterate the corpus, run every family of every case from one
// loop, log the rows, and assert finiteness + invariant + cross-family
// agreement + a per-(scene,family) "actually ran" signal.
//==============================================================================

TEST(CrossFamilyCorpus, AllScenesRunUnderEveryFamily)
{
  const std::vector<SceneCase> corpus = makeCorpus();
  ASSERT_FALSE(corpus.empty());

  std::ostringstream table;

  for (const SceneCase& c : corpus) {
    ASSERT_GE(c.families.size(), 2u)
        << c.id << ": a cross-family case needs >= 2 families";

    std::vector<CorpusRow> rows;
    for (const std::string& family : c.families) {
      rows.push_back(runCaseFamily(c, family));
    }

    // ----- Per-family checks (finiteness, "ran", invariant). -----
    for (const CorpusRow& row : rows) {
      const std::string tag = row.sceneId + ":" + row.family;

      // (1) Finite metrics at release AND after the roll -- a family that NaNs
      // out is caught immediately.
      ASSERT_TRUE(isFinite(row.atRelease)) << tag << " @release";
      ASSERT_TRUE(isFinite(row.afterRoll)) << tag << " @roll";

      // (4) Per-(scene,family) "actually ran" signal: prove this family did not
      // silently no-op the scene. The meaning is axis-specific.
      if (c.axis == FamilyAxis::MultibodyIntegration) {
        // A frozen pendulum sits at the raised rest pose with KE ~ 0 forever.
        // After a real roll the released pendulum has swung, so its post-roll
        // kinetic energy is non-trivial. Require KE > 1e-3 J: comfortably above
        // round-off, far below the ~O(1) J these scenes carry, so it only fires
        // for a true no-op. (The conserved-energy invariant is checked below
        // and is the actual physics gate; this is purely an anti-no-op guard.)
        EXPECT_GT(row.afterRoll.kineticEnergy, 1e-3)
            << tag << " appears to have no-opped (post-roll KE ~ 0)";
        // The release snapshot is from rest, so KE there must be ~0 -- a sanity
        // check that the at-rest split is physical.
        EXPECT_NEAR(row.atRelease.kineticEnergy, 0.0, 1e-6)
            << tag << " released-from-rest KE must be ~0";
      } else {
        // Rigid axes: the ranSignal carries scene-specific evidence.
        const double delta = std::abs(row.ranAfterRoll - row.ranAtRelease);
        EXPECT_GE(delta, c.minRanDelta)
            << tag << " ran-signal moved only " << delta << " (< "
            << c.minRanDelta << "); family may have no-opped";
      }

      // (2) Per-family declared physical invariant within a robust tolerance.
      switch (c.invariant) {
        case Invariant::ConservesLinearMomentum: {
          // Head-on collision: initial total momentum is +0.5 x by
          // construction (1*1.5 + 2*-0.5), conserved through the internal
          // contact impulses. Tolerance 1e-7 absolute on a 0.5 signal
          // (~2e-7 relative): the iterative impulse solve conserves momentum to
          // solver-residual round-off, comfortably under 1e-7 yet ~6 orders
          // below the signal, so a real third-law leak trips it.
          const Eigen::Vector3d kExpectedP(0.5, 0.0, 0.0);
          EXPECT_LT((row.afterRoll.linearMomentum - kExpectedP).norm(), 1e-7)
              << tag << " linear-momentum conservation";
          break;
        }
        case Invariant::ConservesTotalEnergy: {
          // Passive articulated swing: total mechanical energy is preserved
          // across the roll. Tolerance 1% relative drift. The variational
          // integrator is symplectic (bounded, no secular drift) and
          // semi-implicit Euler at 1 ms on these light links stays well inside
          // 1%, so the bound covers both without flaking while a real
          // energy-pumping/bleeding integrator bug blows far past it.
          ASSERT_GT(std::abs(row.atRelease.totalEnergy), 1e-6)
              << tag << " release energy must be non-trivial";
          const double drift
              = std::abs(row.afterRoll.totalEnergy - row.atRelease.totalEnergy)
                / std::abs(row.atRelease.totalEnergy);
          EXPECT_LT(drift, 1e-2) << tag << " relative energy drift " << drift;
          break;
        }
        case Invariant::SettlesOnStaticSupport: {
          // The sphere must come to rest on the ground: its COM settles near
          // the radius (0.5) and its residual KE is ~0. Height tolerance 1e-2 m
          // (2% of the 0.5 radius) absorbs the small penetration/standoff the
          // two contact formulations leave, without admitting a body that sank
          // through (which would land near -0.5 or below) or never fell.
          EXPECT_NEAR(row.ranAfterRoll, c.ranReference, 1e-2)
              << tag << " rest height";
          // Settled => negligible residual kinetic energy. 1e-3 J is far below
          // the ~5 J of potential the 1 kg sphere shed falling 0.5 m, so a body
          // still bouncing/sliding (a contact that failed to dissipate) trips
          // it, while the tiny solver jitter at rest passes.
          EXPECT_LT(row.afterRoll.kineticEnergy, 1e-3)
              << tag << " residual KE at rest";
          break;
        }
      }

      table << tag << " @release: " << formatMetrics(row.atRelease) << "\n"
            << tag << " @" << c.steps << ":   " << formatMetrics(row.afterRoll)
            << "  ran[" << row.ranAtRelease << " -> " << row.ranAfterRoll
            << "]\n";
    }

    // ----- (3) Cross-family agreement on the SAME scene's gross outcome. -----
    // Compare family 0 against every other family of the case. The band is
    // derived from the invariant's physics, not a numerical-identity claim.
    const CorpusRow& ref = rows.front();
    for (std::size_t i = 1; i < rows.size(); ++i) {
      const CorpusRow& other = rows[i];
      const std::string pair
          = c.id + " [" + ref.family + " vs " + other.family + "]";
      switch (c.invariant) {
        case Invariant::ConservesLinearMomentum: {
          // Both families conserve the same momentum, so their final momenta
          // agree to ~2e-7 (each within 1e-7 of the exact +0.5 x). 5e-7 leaves
          // slack without being loose enough to miss a divergence.
          EXPECT_LT(
              (ref.afterRoll.linearMomentum - other.afterRoll.linearMomentum)
                  .norm(),
              5e-7)
              << pair << " final-momentum agreement";
          // Both reach the same near-inelastic end state, so final KE agrees
          // within 0.05 J (~3.6% of the 1.375 J input) -- a gross-outcome band:
          // it catches a family that fails to dissipate (~1 J apart) without
          // flaking on the legitimate micro-state spread.
          EXPECT_LT(
              std::abs(
                  ref.afterRoll.kineticEnergy - other.afterRoll.kineticEnergy),
              0.05)
              << pair << " final-KE agreement";
          break;
        }
        case Invariant::ConservesTotalEnergy: {
          // Both integrators model the SAME scene from the SAME release pose,
          // so the conserved energy is identical at release to round-off...
          EXPECT_NEAR(
              ref.atRelease.totalEnergy, other.atRelease.totalEnergy, 1e-9)
              << pair << " release-energy agreement";
          // ...and stays within 2% of each other after the roll. 2% is the sum
          // of the two families' 1% conservation budgets (they may drift up to
          // 1% in opposite directions), so it is the physically-correct
          // cross-family band: a family that diverges (e.g. loses 10%) is
          // caught, while the legitimate symplectic-vs-Euler spread never
          // flakes.
          const double spread
              = std::abs(
                    ref.afterRoll.totalEnergy - other.afterRoll.totalEnergy)
                / std::abs(ref.atRelease.totalEnergy);
          EXPECT_LT(spread, 2e-2) << pair << " final-energy spread";
          break;
        }
        case Invariant::SettlesOnStaticSupport: {
          // Both contact families arrest the same drop, so their final rest
          // heights agree to ~2e-2 m (each within 1e-2 of the 0.5 reference, so
          // their difference is bounded by 2e-2). This is the cross-family
          // gross-outcome gate: a family that lets the body sink or bounce ends
          // far from the other.
          EXPECT_NEAR(ref.ranAfterRoll, other.ranAfterRoll, 2e-2)
              << pair << " rest-height agreement";
          break;
        }
      }
    }
  }

  RecordProperty("corpus_rows", table.str());
  // Echo the table to stdout too, so the comparison is visible without parsing
  // the gtest XML property -- the rows ARE the deliverable of this harness.
  std::cout << "\n[cross-family corpus rows]\n" << table.str() << std::endl;
}

} // namespace
