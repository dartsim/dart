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
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

// PLAN-091 WP-091.2: golden trajectories for the default World::step.
//
// These tests lock the default rigid-body step pipeline (semi-implicit
// integration plus the default sequential-impulse contact solve) by committing
// reference trajectories for a small scene matrix and comparing every later run
// against them. The refactor-heavy WS1+ packets (virtual finalize/prepare,
// canonical contact assembly, family-scoped source layout, executor axis) cite
// this packet in their Dependencies lines: a green run here is the
// behavior-lock evidence that those structural changes preserved default
// behavior.
//
// Two complementary oracles (see AnalyticalProfile and assertAnalyticalAnchors
// below):
//   - Correctness layer (analytical): each trajectory is validated against
//     ground truth that is INDEPENDENT of the committed golden — the exact
//     semi-implicit Euler closed form for the contact-free phase, and physical
//     invariants (no tunneling, settles at the geometric rest height, comes to
//     rest, no spurious lateral/spin) for the contact phase. This runs in both
//     compare and regen modes, so the golden cannot silently enshrine an
//     integrator bug or a physically wrong contact resolution.
//   - Behavior lock (snapshot): the full per-step trajectory is compared to the
//     committed golden to catch any change in the parts that have no closed
//     form (the contact transient). This is what the WS1+ refactors diff
//     against. Systematic energy/momentum-conservation and order-of-convergence
//     validation across the scene corpus is owned by WP-091.24, not this
//     packet.
//
// Scene matrix (default WorldOptions: SequentialImpulse contacts, gravity -Z):
//   - free_fall        : one dynamic sphere, no ground -> locks pure
//                        semi-implicit integration; contacts stay 0.
//   - sphere_on_ground : a dynamic sphere dropped onto a static box ground ->
//                        locks contact onset and impulse resolution.
//   - box_on_ground    : a dynamic box dropped onto a static box ground ->
//                        locks contact resolution plus angular state.
//
// Tolerance policy: the default CPU step is single-threaded double precision
// and deterministic — on the toolchain that generated the committed goldens it
// reproduces bitwise (verified run-to-run). The snapshot comparison tolerance
// therefore is not about same-platform noise; it is the margin that separates a
// real behavior change from legitimate floating-point reassociation (a
// behavior-preserving refactor that reorders, say, a contact-assembly
// summation) and cross-platform libm/FMA differences. States (position, linear
// and angular velocity, time) are compared with an ABSOLUTE tolerance of
// kStateTolerance = 1e-6, which is well below any real physics change (the
// packet's injected gravity perturbation drifts >1e-5 within one step and
// grows) yet above reassociation-scale noise; post-step contact counts must
// match EXACTLY as integers. kStateTolerance is the single calibration knob.
// If a real physics change is intended, regenerate the committed fixtures by
// running this binary with DART_REGENERATE_DEFAULT_STEP_GOLDEN=1 and review the
// diff before committing.

#include <dart/simulation/body/collision_shape.hpp>
#include <dart/simulation/body/contact.hpp>
#include <dart/simulation/body/rigid_body.hpp>
#include <dart/simulation/body/rigid_body_options.hpp>
#include <dart/simulation/frame/frame.hpp>
#include <dart/simulation/world.hpp>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <ios>
#include <sstream>
#include <string>
#include <vector>

#include <cmath>
#include <cstdlib>

namespace sx = dart::simulation;

namespace {

// Absolute tolerance for the recorded double-precision states (see the
// tolerance-policy note in the file header). Contact counts and the frame
// index are compared exactly. This is the single calibration knob.
constexpr double kStateTolerance = 1e-6;

struct StepRecord
{
  int frame = 0;
  double time = 0.0;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d linearVelocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d angularVelocity = Eigen::Vector3d::Zero();
  int contactCount = 0;
};

// Analytical (closed-form / physical-invariant) anchors for a scene. These are
// the correctness layer that complements the behavior-lock snapshot: they
// validate the trajectory against ground truth that is INDEPENDENT of the
// committed golden, so the golden cannot silently enshrine an integrator bug or
// a physically wrong contact resolution. They run in both compare and regen
// modes (see runSceneGoldenTest), so a regenerated golden is physics-validated
// before it is committed.
//
// "Analytical" here means the exact solution of the discrete SCHEME, not the
// continuous ODE: semi-implicit (symplectic) Euler free-fall has the closed
// form v_n = v0 - g*n*dt and z_n = z0 + v0*n*dt - g*dt^2*n(n+1)/2, which the
// default step reproduces to machine precision (forward Euler would differ by
// g*dt^2*n, easily detected). The sequential-impulse contact transient has no
// closed form, so for contact scenes we assert physical invariants instead.
struct AnalyticalProfile
{
  // Free-fall closed form (applied to the leading contact-free phase of every
  // scene; the tracked body free-falls until first contact).
  double z0 = 0.0;
  double dt = 0.0;
  double g = 9.81; // gravity magnitude along -Z
  double freeFallTol = 1e-9;

  // Contact invariants (enabled when hasGround). restCenter is the geometric
  // rest height of the tracked body's center (ground top + half-size).
  bool hasGround = false;
  double restCenter = 0.0;
  double maxPenetration = 0.05;  // center never sinks below restCenter - this
  double restHeightTol = 2e-3;   // final center within this of restCenter
  double settledSpeedTol = 1e-2; // final linear speed below this
  double maxLateralSpeed = 0.05; // bound on sqrt(vx^2+vy^2) over trajectory
  double maxAngularSpeed = 0.05; // bound on |angular velocity| over trajectory
};

// A scene = world options plus body setup performed in design mode, the name of
// the dynamic body whose trajectory is recorded, the number of steps, and the
// analytical anchors that validate its physics.
struct Scene
{
  std::string id;
  int steps = 0;
  std::string trackedBody;
  void (*build)(sx::World& world) = nullptr;
  AnalyticalProfile analytical;
};

void buildFreeFall(sx::World& world)
{
  world.setTimeStep(0.01);
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));

  sx::RigidBodyOptions sphereOpts;
  sphereOpts.mass = 1.0;
  sphereOpts.position = Eigen::Vector3d(0.0, 0.0, 5.0);
  auto sphere = world.addRigidBody("faller", sphereOpts);
  sphere.setCollisionShape(sx::CollisionShape::makeSphere(0.25));
}

void buildSphereOnGround(sx::World& world)
{
  world.setTimeStep(0.005);
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));

  sx::RigidBodyOptions groundOpts;
  groundOpts.isStatic = true;
  auto ground = world.addRigidBody("ground", groundOpts);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(5.0, 5.0, 0.5)));

  sx::RigidBodyOptions sphereOpts;
  sphereOpts.mass = 1.0;
  sphereOpts.position = Eigen::Vector3d(0.0, 0.0, 2.0);
  auto sphere = world.addRigidBody("sphere", sphereOpts);
  sphere.setCollisionShape(sx::CollisionShape::makeSphere(0.3));
}

void buildBoxOnGround(sx::World& world)
{
  world.setTimeStep(0.005);
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));

  sx::RigidBodyOptions groundOpts;
  groundOpts.isStatic = true;
  auto ground = world.addRigidBody("ground", groundOpts);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(5.0, 5.0, 0.5)));

  sx::RigidBodyOptions boxOpts;
  boxOpts.mass = 1.0;
  boxOpts.position = Eigen::Vector3d(0.0, 0.0, 1.5);
  auto box = world.addRigidBody("box", boxOpts);
  box.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(0.4, 0.4, 0.4)));
}

const std::vector<Scene>& sceneMatrix()
{
  static const std::vector<Scene> kScenes = {
      Scene{
          "free_fall",
          30,
          "faller",
          &buildFreeFall,
          AnalyticalProfile{.z0 = 5.0, .dt = 0.01, .g = 9.81}},
      Scene{
          "sphere_on_ground",
          120,
          "sphere",
          &buildSphereOnGround,
          AnalyticalProfile{
              .z0 = 2.0,
              .dt = 0.005,
              .g = 9.81,
              .hasGround = true,
              .restCenter = 0.8}}, // ground top 0.5 + sphere radius 0.3
      Scene{
          "box_on_ground",
          120,
          "box",
          &buildBoxOnGround,
          AnalyticalProfile{
              .z0 = 1.5,
              .dt = 0.005,
              .g = 9.81,
              .hasGround = true,
              .restCenter = 0.9}}, // ground top 0.5 + box half-height 0.4
  };
  return kScenes;
}

// Run a scene under the default World::step and record per-step trajectory.
std::vector<StepRecord> simulate(const Scene& scene)
{
  sx::World world;
  scene.build(world);
  world.enterSimulationMode();

  const auto body = world.getRigidBody(scene.trackedBody);
  EXPECT_TRUE(body.has_value()) << "tracked body '" << scene.trackedBody
                                << "' missing in scene " << scene.id;

  std::vector<StepRecord> records;
  records.reserve(static_cast<std::size_t>(scene.steps));
  for (int i = 0; i < scene.steps; ++i) {
    world.step();
    StepRecord record;
    record.frame = static_cast<int>(world.getFrame());
    record.time = world.getTime();
    if (body.has_value()) {
      record.position = body->getTranslation();
      record.linearVelocity = body->getLinearVelocity();
      record.angularVelocity = body->getAngularVelocity();
    }
    record.contactCount = static_cast<int>(world.collide().size());
    records.push_back(record);
  }
  return records;
}

// Index of the first step with an active contact, or records.size() if none.
std::size_t firstContactIndex(const std::vector<StepRecord>& records)
{
  for (std::size_t i = 0; i < records.size(); ++i) {
    if (records[i].contactCount > 0) {
      return i;
    }
  }
  return records.size();
}

// Validate the trajectory against analytical / physical ground truth that is
// independent of the committed golden (the correctness layer).
void assertAnalyticalAnchors(
    const Scene& scene, const std::vector<StepRecord>& records)
{
  const AnalyticalProfile& p = scene.analytical;
  const std::size_t firstContact = firstContactIndex(records);

  // (1) Exact semi-implicit (symplectic) Euler closed form during the
  //     contact-free phase: v_z(n) = -g*n*dt, z(n) = z0 - g*dt^2*n(n+1)/2,
  //     with zero lateral velocity and zero spin. This validates the integrator
  //     scheme against an independent oracle, so the golden cannot enshrine an
  //     integrator bug (a forward-Euler regression would diverge by g*dt^2*n).
  for (std::size_t i = 0; i < firstContact; ++i) {
    const StepRecord& r = records[i];
    const double n = static_cast<double>(r.frame);
    const double expectedZ = p.z0 - p.g * p.dt * p.dt * n * (n + 1.0) / 2.0;
    const double expectedVz = -p.g * n * p.dt;
    EXPECT_NEAR(r.position.z(), expectedZ, p.freeFallTol)
        << scene.id << " frame " << r.frame
        << ": contact-free phase must match the exact semi-implicit Euler "
           "position z0 - g*dt^2*n(n+1)/2.";
    EXPECT_NEAR(r.linearVelocity.z(), expectedVz, p.freeFallTol)
        << scene.id << " frame " << r.frame
        << ": contact-free vertical velocity must match -g*n*dt.";
    EXPECT_NEAR(r.linearVelocity.x(), 0.0, p.freeFallTol)
        << scene.id << " frame " << r.frame
        << ": no lateral x velocity in free fall.";
    EXPECT_NEAR(r.linearVelocity.y(), 0.0, p.freeFallTol)
        << scene.id << " frame " << r.frame
        << ": no lateral y velocity in free fall.";
    EXPECT_NEAR(r.angularVelocity.norm(), 0.0, p.freeFallTol)
        << scene.id << " frame " << r.frame << ": no spin in free fall.";
  }

  if (!p.hasGround) {
    return;
  }

  // (2) Physical invariants for the contact phase. The sequential-impulse
  //     transient has no closed form, so assert truths the solver must satisfy
  //     regardless of implementation.
  double maxLateral = 0.0;
  double maxAngular = 0.0;
  double minZ = records.front().position.z();
  for (const StepRecord& r : records) {
    maxLateral = std::max(
        maxLateral, std::hypot(r.linearVelocity.x(), r.linearVelocity.y()));
    maxAngular = std::max(maxAngular, r.angularVelocity.norm());
    minZ = std::min(minZ, r.position.z());
  }

  // The body must actually reach contact (otherwise the scene is mis-built and
  // the snapshot would lock a contact-free trajectory).
  EXPECT_LT(firstContact, records.size())
      << scene.id << ": expected a contact onset but none occurred.";
  // No tunneling: the center never sinks far below its geometric rest height.
  EXPECT_GE(minZ, p.restCenter - p.maxPenetration)
      << scene.id << ": body tunneled below the ground rest height (min z "
      << minZ << ").";
  // Settles at the geometric rest height (ground top + half-size).
  EXPECT_NEAR(records.back().position.z(), p.restCenter, p.restHeightTol)
      << scene.id << ": body did not settle at the analytical rest height "
      << p.restCenter << ".";
  // Comes to rest rather than gaining energy from the contact solve.
  EXPECT_LT(records.back().linearVelocity.norm(), p.settledSpeedTol)
      << scene.id << ": body did not settle (residual speed too high).";
  // A symmetric vertical drop must not fly sideways or spin up.
  EXPECT_LT(maxLateral, p.maxLateralSpeed)
      << scene.id << ": spurious lateral velocity from a vertical drop.";
  EXPECT_LT(maxAngular, p.maxAngularSpeed)
      << scene.id << ": spurious angular velocity from a symmetric drop.";
}

std::filesystem::path goldenPath(const std::string& sceneId)
{
  // __FILE__-relative resolution mirrors the committed-fixture convention used
  // by the rigid-IPC corpus loader. This file lives in
  // tests/unit/simulation/world/, so three parents up reaches tests/.
  return (std::filesystem::path(__FILE__).parent_path() / ".." / ".." / ".."
          / "fixtures" / "default_step_golden" / (sceneId + ".tsv"))
      .lexically_normal();
}

void writeGolden(
    const std::filesystem::path& path,
    const Scene& scene,
    const std::vector<StepRecord>& records)
{
  std::filesystem::create_directories(path.parent_path());
  std::ofstream out(path, std::ios::trunc);
  ASSERT_TRUE(out.is_open()) << "cannot write golden " << path;
  out << std::setprecision(17) << std::scientific;
  out << "# PLAN-091 WP-091.2 default-step golden trajectory: " << scene.id
      << "\n";
  out << "# columns: frame time pos_x pos_y pos_z lin_x lin_y lin_z ang_x "
         "ang_y ang_z contacts\n";
  for (const auto& r : records) {
    out << r.frame << '\t' << r.time << '\t' << r.position.x() << '\t'
        << r.position.y() << '\t' << r.position.z() << '\t'
        << r.linearVelocity.x() << '\t' << r.linearVelocity.y() << '\t'
        << r.linearVelocity.z() << '\t' << r.angularVelocity.x() << '\t'
        << r.angularVelocity.y() << '\t' << r.angularVelocity.z() << '\t'
        << r.contactCount << '\n';
  }
}

std::vector<StepRecord> loadGolden(const std::filesystem::path& path)
{
  std::vector<StepRecord> records;
  std::ifstream in(path);
  if (!in.is_open()) {
    return records;
  }
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#') {
      continue;
    }
    std::istringstream iss(line);
    StepRecord r;
    iss >> r.frame >> r.time >> r.position.x() >> r.position.y()
        >> r.position.z() >> r.linearVelocity.x() >> r.linearVelocity.y()
        >> r.linearVelocity.z() >> r.angularVelocity.x()
        >> r.angularVelocity.y() >> r.angularVelocity.z() >> r.contactCount;
    if (iss.fail()) {
      ADD_FAILURE() << "malformed golden row in " << path << ": " << line;
      continue;
    }
    records.push_back(r);
  }
  return records;
}

bool regenerateRequested()
{
  const char* flag = std::getenv("DART_REGENERATE_DEFAULT_STEP_GOLDEN");
  return flag != nullptr && flag[0] != '\0';
}

void compareComponent(
    double actual,
    double expected,
    const Scene& scene,
    int frame,
    const std::string& field)
{
  EXPECT_NEAR(actual, expected, kStateTolerance)
      << scene.id << " frame " << frame << " field " << field
      << ": default step drifted from the committed golden trajectory. "
      << "If this physics change is intended, regenerate with "
         "DART_REGENERATE_DEFAULT_STEP_GOLDEN=1.";
}

void runSceneGoldenTest(const Scene& scene)
{
  const std::vector<StepRecord> actual = simulate(scene);
  ASSERT_EQ(actual.size(), static_cast<std::size_t>(scene.steps));

  // Correctness layer: validate against analytical / physical ground truth
  // first, in BOTH modes, so a regenerated golden cannot bake in a physically
  // wrong trajectory.
  assertAnalyticalAnchors(scene, actual);

  const std::filesystem::path path = goldenPath(scene.id);

  if (regenerateRequested()) {
    // Refuse to serialize a golden the correctness layer just rejected: the
    // analytical anchors use non-fatal EXPECT_*, so guard the write on the
    // current test having no failures. Without this, a gravity / integrator /
    // contact regression would still leave an updated (physically wrong) golden
    // on disk during the regen workflow.
    ASSERT_FALSE(::testing::Test::HasFailure())
        << scene.id
        << ": analytical anchors failed; refusing to regenerate a physically "
           "invalid golden. Fix the physics, then regenerate.";
    writeGolden(path, scene, actual);
    GTEST_SKIP() << "regenerated golden " << path;
    return;
  }

  const std::vector<StepRecord> expected = loadGolden(path);
  ASSERT_FALSE(expected.empty()) << "missing or empty golden fixture " << path
                                 << ". Seed it by running this binary with "
                                    "DART_REGENERATE_DEFAULT_STEP_GOLDEN=1.";
  ASSERT_EQ(actual.size(), expected.size())
      << scene.id << ": step count differs from committed golden";

  for (std::size_t i = 0; i < actual.size(); ++i) {
    const StepRecord& a = actual[i];
    const StepRecord& e = expected[i];
    EXPECT_EQ(a.frame, e.frame) << scene.id << " row " << i << " frame index";
    compareComponent(a.time, e.time, scene, a.frame, "time");
    compareComponent(a.position.x(), e.position.x(), scene, a.frame, "pos_x");
    compareComponent(a.position.y(), e.position.y(), scene, a.frame, "pos_y");
    compareComponent(a.position.z(), e.position.z(), scene, a.frame, "pos_z");
    compareComponent(
        a.linearVelocity.x(), e.linearVelocity.x(), scene, a.frame, "lin_x");
    compareComponent(
        a.linearVelocity.y(), e.linearVelocity.y(), scene, a.frame, "lin_y");
    compareComponent(
        a.linearVelocity.z(), e.linearVelocity.z(), scene, a.frame, "lin_z");
    compareComponent(
        a.angularVelocity.x(), e.angularVelocity.x(), scene, a.frame, "ang_x");
    compareComponent(
        a.angularVelocity.y(), e.angularVelocity.y(), scene, a.frame, "ang_y");
    compareComponent(
        a.angularVelocity.z(), e.angularVelocity.z(), scene, a.frame, "ang_z");
    EXPECT_EQ(a.contactCount, e.contactCount)
        << scene.id << " frame " << a.frame
        << ": post-step contact count drifted from the committed golden.";
  }
}

} // namespace

TEST(DefaultStepGolden, FreeFall)
{
  runSceneGoldenTest(sceneMatrix()[0]);
}

TEST(DefaultStepGolden, SphereOnGround)
{
  runSceneGoldenTest(sceneMatrix()[1]);
}

TEST(DefaultStepGolden, BoxOnGround)
{
  runSceneGoldenTest(sceneMatrix()[2]);
}
