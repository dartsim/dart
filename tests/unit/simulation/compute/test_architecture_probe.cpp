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

// PLAN-130 WP-130.7: architecture map runtime probe.
//
// Steps a reference scene (one free rigid body, one revolute pendulum, one
// mass-spring deformable patch) through the built-in World::step() schedule
// with a recording executor, so the stage names that actually ran and the
// compute graphs that were actually executed can be compared with the drawn
// architecture map (docs/assets/architecture/world-step.dataflow.json and
// compute-graph.architecture.json).
//
// The assertions below are the stable contract. When the environment variable
// DART_ARCHITECTURE_PROBE_OUTPUT names a file, the probe also writes a JSON
// dump (stage names, graph node/edge sets, DOT text) that
// scripts/check_architecture_map_runtime.py compares with the committed
// docs/assets/architecture/compute-graph.runtime.json fixture (advisory).

#include <dart/config.hpp>

#include <dart/simulation/body/collision_shape.hpp>
#include <dart/simulation/body/deformable_body.hpp>
#include <dart/simulation/body/deformable_body_options.hpp>
#include <dart/simulation/body/rigid_body.hpp>
#include <dart/simulation/body/rigid_body_options.hpp>
#include <dart/simulation/compute/compute_executor.hpp>
#include <dart/simulation/compute/compute_graph.hpp>
#include <dart/simulation/compute/compute_graph_visualization.hpp>
#include <dart/simulation/compute/compute_node.hpp>
#include <dart/simulation/compute/sequential_executor.hpp>
#include <dart/simulation/compute/world_step_profile.hpp>
#include <dart/simulation/detail/world_step_schedule.hpp>
#include <dart/simulation/multibody/joint_type.hpp>
#include <dart/simulation/multibody/multibody.hpp>
#include <dart/simulation/world.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <fstream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <cstdlib>

namespace {

namespace sx = dart::simulation;

struct RecordedGraph
{
  std::vector<std::string> nodes;
  std::vector<std::pair<std::string, std::string>> edges;
  std::string dot;
};

/// Executor that records every graph it is asked to run, then delegates to
/// the sequential reference executor so the step itself is unchanged.
class RecordingExecutor final : public sx::compute::ComputeExecutor
{
public:
  void execute(const sx::compute::ComputeGraph& graph) override
  {
    record(graph);
    m_inner.execute(graph);
  }

  [[nodiscard]] sx::compute::ComputeExecutionProfile executeProfiled(
      const sx::compute::ComputeGraph& graph) override
  {
    record(graph);
    return m_inner.executeProfiled(graph);
  }

  [[nodiscard]] std::size_t getWorkerCount() const override
  {
    return m_inner.getWorkerCount();
  }

  [[nodiscard]] const std::vector<RecordedGraph>& graphs() const noexcept
  {
    return m_graphs;
  }

  /// Forget the graphs recorded so far, so a later step can be recorded alone.
  void clear() noexcept
  {
    m_graphs.clear();
  }

private:
  void record(const sx::compute::ComputeGraph& graph)
  {
    RecordedGraph recorded;
    for (const auto* node : graph.getTopologicalOrder()) {
      recorded.nodes.push_back(node->getName());
    }
    for (const auto& edge : graph.getEdges()) {
      recorded.edges.emplace_back(edge.from->getName(), edge.to->getName());
    }
    recorded.dot = sx::compute::toDot(graph);
    m_graphs.push_back(std::move(recorded));
  }

  sx::compute::SequentialExecutor m_inner;
  std::vector<RecordedGraph> m_graphs;
};

void buildReferenceScene(sx::World& world)
{
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world.setTimeStep(0.001);

  // One free rigid body above a static fixture.
  sx::RigidBodyOptions ballOptions;
  ballOptions.position = Eigen::Vector3d(0.0, 0.0, 1.0);
  auto ball = world.addRigidBody("probe_ball", ballOptions);
  ball.setCollisionShape(sx::CollisionShape::makeSphere(0.1));

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.2);
  auto ground = world.addRigidBody("probe_ground", groundOptions);
  ground.setCollisionShape(sx::CollisionShape::makeSphere(0.2));

  // One revolute pendulum.
  auto robot = world.addMultibody("probe_pendulum");
  auto base = robot.addLink("base");
  Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
  offset.translation() = Eigen::Vector3d(0.5, 0.0, 0.0);
  auto bob = robot.addLink(
      "bob",
      base,
      sx::JointSpec{
          .name = "hinge",
          .type = sx::JointType::Revolute,
          .axis = Eigen::Vector3d::UnitY(),
          .transformFromParent = offset,
      });
  bob.setMass(1.0);
  bob.setInertia(Eigen::Vector3d(0.01, 0.01, 0.01).asDiagonal());
  bob.getParentJoint().setPosition(Eigen::VectorXd::Constant(1, 0.3));

  // One mass-spring patch with a fixed edge.
  sx::DeformableBodyOptions patch;
  patch.positions
      = {Eigen::Vector3d(2.0, -0.5, 1.0),
         Eigen::Vector3d(3.0, -0.5, 1.0),
         Eigen::Vector3d(3.0, 0.5, 1.0),
         Eigen::Vector3d(2.0, 0.5, 1.0)};
  patch.velocities
      = {Eigen::Vector3d::Zero(),
         Eigen::Vector3d::Zero(),
         Eigen::Vector3d::Zero(),
         Eigen::Vector3d::Zero()};
  patch.masses = {1.0, 1.0, 1.0, 1.0};
  patch.fixedNodes = {0, 3};
  patch.edgeStiffness = 50.0;
  patch.surfaceTriangles
      = {sx::DeformableSurfaceTriangle{0, 1, 2},
         sx::DeformableSurfaceTriangle{0, 2, 3}};
  world.addDeformableBody("probe_patch", patch);
}

/// The name a built-in stage slot reports through `WorldStepStage::getName()`
/// and the node id the architecture map uses for it (enumerator in snake case).
std::string slotName(sx::detail::BuiltInWorldStepStageSlot slot)
{
  using Slot = sx::detail::BuiltInWorldStepStageSlot;
  switch (slot) {
    case Slot::RigidBodyVelocity:
      return "rigid_body_velocity";
    case Slot::RigidBodyContact:
      return "rigid_body_contact";
    case Slot::RigidBodyPosition:
      return "rigid_body_position";
    case Slot::RigidIpcContact:
      return "rigid_ipc_contact";
    case Slot::MultibodyVelocity:
      return "multibody_velocity";
    case Slot::MultibodyForwardDynamics:
      return "multibody_forward_dynamics";
    case Slot::MultibodyPosition:
      return "multibody_position";
    case Slot::MultibodyVariationalIntegration:
      return "multibody_variational_integration";
    case Slot::UnifiedConstraint:
      return "unified_constraint";
    case Slot::DeformableDynamics:
      return "deformable_dynamics";
    case Slot::Kinematics:
      return "kinematics";
  }
  return "unknown";
}

/// Stage names the built-in schedule selects for the reference scene with the
/// default families (SequentialImpulse rigid, SemiImplicit multibody). This
/// does not depend on DART_BUILD_PROFILE, so the probe records the schedule in
/// every build configuration.
std::vector<std::string> scheduledStageNames()
{
  sx::detail::BuiltInWorldStepScheduleOptions options;
  options.hasRigidBodies = true;
  options.hasMultibodyStructures = true;
  options.hasDeformableBodies = true;
  options.includeKinematics = true;
  std::vector<std::string> names;
  for (const auto slot : sx::detail::makeBuiltInWorldStepSchedule(options)) {
    names.push_back(slotName(slot));
  }
  return names;
}

std::string jsonEscape(const std::string& text)
{
  std::string out;
  out.reserve(text.size() + 8);
  for (const char c : text) {
    switch (c) {
      case '"':
        out += "\\\"";
        break;
      case '\\':
        out += "\\\\";
        break;
      case '\n':
        out += "\\n";
        break;
      case '\t':
        out += "\\t";
        break;
      default:
        out += c;
    }
  }
  return out;
}

std::string toJson(
    const std::vector<std::string>& stages,
    const std::vector<RecordedGraph>& graphs,
    bool executionTrace)
{
  std::ostringstream out;
  out << "{\n  \"schema_version\": 1,\n";
  out << "  \"execution_trace\": " << (executionTrace ? "true" : "false")
      << ",\n";
  out << "  \"scene\": \"free rigid sphere over static sphere, revolute "
         "pendulum, four-node mass-spring patch\",\n";
  out << "  \"stages\": [";
  for (std::size_t i = 0; i < stages.size(); ++i) {
    out << (i == 0 ? "" : ", ") << '"' << jsonEscape(stages[i]) << '"';
  }
  out << "],\n  \"graphs\": [\n";
  for (std::size_t g = 0; g < graphs.size(); ++g) {
    const auto& graph = graphs[g];
    out << "    {\n      \"nodes\": [";
    for (std::size_t i = 0; i < graph.nodes.size(); ++i) {
      out << (i == 0 ? "" : ", ") << '"' << jsonEscape(graph.nodes[i]) << '"';
    }
    out << "],\n      \"edges\": [";
    for (std::size_t i = 0; i < graph.edges.size(); ++i) {
      out << (i == 0 ? "" : ", ") << "[\"" << jsonEscape(graph.edges[i].first)
          << "\", \"" << jsonEscape(graph.edges[i].second) << "\"]";
    }
    out << "],\n      \"dot\": \"" << jsonEscape(graph.dot) << "\"\n    }";
    out << (g + 1 < graphs.size() ? ",\n" : "\n");
  }
  out << "  ]\n}\n";
  return out.str();
}

} // namespace

//==============================================================================
TEST(ArchitectureProbe, ReferenceSceneStagesAndGraphsMatchTheMap)
{
  sx::World world;
  buildReferenceScene(world);
  world.enterSimulationMode();
  world.setStepProfilingEnabled(true);

  RecordingExecutor executor;
  // Warm up once so first-step preparation does not masquerade as
  // steady-state work, then record exactly one step.
  world.step(executor);
  executor.clear();
  world.step(executor);

  // The default families (SequentialImpulse rigid, SemiImplicit multibody)
  // run the fused schedule documented by the "fused-multibody" guided view
  // of docs/assets/architecture/world-step.dataflow.json.
  const std::vector<std::string> scheduled = scheduledStageNames();
  const std::vector<std::string> expected
      = {"rigid_body_velocity",
         "multibody_velocity",
         "unified_constraint",
         "rigid_body_position",
         "multibody_position",
         "deformable_dynamics",
         "kinematics"};
  EXPECT_EQ(scheduled, expected);

  // The dump records what the World executed when profiling is compiled in;
  // otherwise it records the built-in schedule and says so, and the runtime
  // checker refuses to treat such a dump as fresh execution evidence.
  std::vector<std::string> stages = scheduled;
  bool executionTrace = false;
#if DART_BUILD_PROFILE
  stages.clear();
  for (const auto& stage : world.getLastStepProfile().stages) {
    stages.push_back(stage.name);
  }
  EXPECT_EQ(stages, scheduled);
  executionTrace = true;
#endif

  // Kinematics is graph-backed, so at least one graph ran through the
  // injected executor and every graph is a well-formed DAG.
  ASSERT_FALSE(executor.graphs().empty());
  for (const auto& graph : executor.graphs()) {
    EXPECT_FALSE(graph.nodes.empty());
    EXPECT_NE(graph.dot.find("digraph ComputeGraph"), std::string::npos);
    for (const auto& [from, to] : graph.edges) {
      EXPECT_NE(
          std::find(graph.nodes.begin(), graph.nodes.end(), from),
          graph.nodes.end());
      EXPECT_NE(
          std::find(graph.nodes.begin(), graph.nodes.end(), to),
          graph.nodes.end());
    }
  }

  if (const char* output = std::getenv("DART_ARCHITECTURE_PROBE_OUTPUT");
      output != nullptr && *output != '\0') {
    std::ofstream file(output);
    ASSERT_TRUE(file.is_open()) << "cannot write probe output to " << output;
    file << toJson(stages, executor.graphs(), executionTrace);
  }
}
