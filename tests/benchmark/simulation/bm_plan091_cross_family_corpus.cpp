/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the "BSD-style" License
 */

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
#include <benchmark/benchmark.h>

#include <cmath>
#include <cstddef>

namespace sx = dart::simulation;
namespace sxc = dart::simulation::compute;

namespace {

// The benchmark is the standalone packet-producing counterpart to
// `test_cross_family_corpus.cpp`: each row runs one scene under one resolved
// solver family and emits the public StepMetrics counters that the packet
// writer validates.
enum class SceneId
{
  RigidHeadOn,
  SphereDrop,
  Pendulum1Link,
  Pendulum2Link,
};

enum class FamilyId
{
  SequentialImpulse,
  BoxedLcp,
  SemiImplicit,
  Variational,
};

struct Scenario
{
  SceneId scene;
  FamilyId family;
  int steps;
};

bool isFinite(const sxc::StepMetrics& metrics)
{
  return std::isfinite(metrics.kineticEnergy)
         && std::isfinite(metrics.potentialEnergy)
         && std::isfinite(metrics.totalEnergy)
         && metrics.linearMomentum.allFinite()
         && metrics.angularMomentum.allFinite()
         && std::isfinite(metrics.maxPenetrationDepth)
         && std::isfinite(metrics.lastStepResidual);
}

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

void releaseAllJoints(sx::World& world, const char* robotName, double angle)
{
  auto robot = world.getMultibody(robotName).value();
  for (auto joint : robot.getJoints()) {
    if (joint.getDOFCount() > 0) {
      joint.setPosition(Eigen::VectorXd::Constant(joint.getDOFCount(), angle));
    }
  }
  world.updateKinematics();
}

void buildScene(sx::World& world, SceneId scene)
{
  switch (scene) {
    case SceneId::RigidHeadOn:
      buildRigidHeadOn(world);
      return;
    case SceneId::SphereDrop:
      buildSphereDrop(world);
      return;
    case SceneId::Pendulum1Link:
      buildPendulum1(world);
      return;
    case SceneId::Pendulum2Link:
      buildPendulum2(world);
      return;
  }
}

void configureWorld(sx::World& world, FamilyId family)
{
  if (family == FamilyId::SemiImplicit) {
    world.setMultibodyOptions(
        {.integrationFamily = sx::MultibodyIntegrationFamily::SemiImplicit});
  } else if (family == FamilyId::Variational) {
    world.setMultibodyOptions(
        {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  }
}

sx::WorldOptions makeWorldOptions(FamilyId family)
{
  sx::WorldOptions options;
  if (family == FamilyId::BoxedLcp) {
    options.contactSolverMethod = sx::ContactSolverMethod::BoxedLcp;
  } else if (family == FamilyId::SequentialImpulse) {
    options.contactSolverMethod = sx::ContactSolverMethod::SequentialImpulse;
  }
  return options;
}

void postEnter(sx::World& world, SceneId scene)
{
  if (scene == SceneId::Pendulum1Link) {
    releaseAllJoints(world, "pend1", 1.0);
  } else if (scene == SceneId::Pendulum2Link) {
    releaseAllJoints(world, "pend2", 0.8);
  }
}

sxc::StepMetrics runScenario(const Scenario& scenario)
{
  sx::World world(makeWorldOptions(scenario.family));
  configureWorld(world, scenario.family);
  buildScene(world, scenario.scene);
  world.enterSimulationMode();
  postEnter(world, scenario.scene);

  for (int step = 0; step < scenario.steps; ++step) {
    world.step();
  }
  return world.computeStepMetrics();
}

void recordMetrics(
    benchmark::State& state,
    const Scenario& scenario,
    const sxc::StepMetrics& m)
{
  state.counters["step_count"] = static_cast<double>(scenario.steps);
  state.counters["kinetic_energy"] = m.kineticEnergy;
  state.counters["potential_energy"] = m.potentialEnergy;
  state.counters["total_energy"] = m.totalEnergy;
  state.counters["linear_momentum_x"] = m.linearMomentum.x();
  state.counters["linear_momentum_y"] = m.linearMomentum.y();
  state.counters["linear_momentum_z"] = m.linearMomentum.z();
  state.counters["angular_momentum_x"] = m.angularMomentum.x();
  state.counters["angular_momentum_y"] = m.angularMomentum.y();
  state.counters["angular_momentum_z"] = m.angularMomentum.z();
  state.counters["active_contact_count"]
      = static_cast<double>(m.activeContactCount);
  state.counters["max_penetration_depth"] = m.maxPenetrationDepth;
  state.counters["last_step_iterations"]
      = static_cast<double>(m.lastStepIterations);
  state.counters["last_step_residual"] = m.lastStepResidual;
}

void BM_Plan091CrossFamilyCorpus(
    benchmark::State& state, const Scenario& scenario)
{
  sxc::StepMetrics metrics;
  for (auto _ : state) {
    metrics = runScenario(scenario);
    if (!isFinite(metrics)) {
      state.SkipWithError("non-finite StepMetrics");
      break;
    }
    benchmark::DoNotOptimize(metrics.totalEnergy);
  }
  recordMetrics(state, scenario, metrics);
  state.SetItemsProcessed(
      static_cast<int64_t>(state.iterations())
      * static_cast<int64_t>(scenario.steps));
}

} // namespace

BENCHMARK_CAPTURE(
    BM_Plan091CrossFamilyCorpus,
    rigid_head_on_sequential_impulse,
    Scenario{SceneId::RigidHeadOn, FamilyId::SequentialImpulse, 1500});
BENCHMARK_CAPTURE(
    BM_Plan091CrossFamilyCorpus,
    rigid_head_on_boxed_lcp,
    Scenario{SceneId::RigidHeadOn, FamilyId::BoxedLcp, 1500});
BENCHMARK_CAPTURE(
    BM_Plan091CrossFamilyCorpus,
    sphere_drop_sequential_impulse,
    Scenario{SceneId::SphereDrop, FamilyId::SequentialImpulse, 600});
BENCHMARK_CAPTURE(
    BM_Plan091CrossFamilyCorpus,
    sphere_drop_boxed_lcp,
    Scenario{SceneId::SphereDrop, FamilyId::BoxedLcp, 600});
BENCHMARK_CAPTURE(
    BM_Plan091CrossFamilyCorpus,
    pendulum_1link_semi_implicit,
    Scenario{SceneId::Pendulum1Link, FamilyId::SemiImplicit, 20000});
BENCHMARK_CAPTURE(
    BM_Plan091CrossFamilyCorpus,
    pendulum_1link_variational,
    Scenario{SceneId::Pendulum1Link, FamilyId::Variational, 20000});
BENCHMARK_CAPTURE(
    BM_Plan091CrossFamilyCorpus,
    pendulum_2link_semi_implicit,
    Scenario{SceneId::Pendulum2Link, FamilyId::SemiImplicit, 8000});
BENCHMARK_CAPTURE(
    BM_Plan091CrossFamilyCorpus,
    pendulum_2link_variational,
    Scenario{SceneId::Pendulum2Link, FamilyId::Variational, 8000});

BENCHMARK_MAIN();
