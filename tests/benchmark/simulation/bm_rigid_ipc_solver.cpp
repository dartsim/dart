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

// First performance scaffold for the experimental rigid IPC solver
// (PLAN-082 Workstream 7). These micro-benchmarks measure the cost of the
// core hot path: per-primitive reduced-coordinate barrier derivatives, the
// scene-level sparse assembly, the projected-Newton barrier solve, and the
// conservative CCD line-search bound.
//
// They are a DART-internal baseline harness, not a parity claim. Comparison
// against the current DART rigid contact path, the audited reference
// implementation, and the paper scene families is gated on completing the
// algorithm's correctness (rigorous interval CCD, corpus parity, production
// convergence). See docs/dev_tasks/rigid_ipc_solver/benchmarks.md.

#include <dart/simulation/body/collision_shape.hpp>
#include <dart/simulation/body/rigid_body.hpp>
#include <dart/simulation/body/rigid_body_options.hpp>
#include <dart/simulation/detail/rigid_ipc/rigid_ipc_barrier.hpp>
#include <dart/simulation/detail/rigid_ipc/rigid_ipc_ccd.hpp>
#include <dart/simulation/world.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <benchmark/benchmark.h>

#include <array>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include <cstdint>

namespace sx = dart::simulation;
namespace sxdetail = dart::simulation::detail;

namespace {

void consumeReduced(sxdetail::RigidIpcReducedBarrierResult& result)
{
  benchmark::DoNotOptimize(result.value);
  benchmark::DoNotOptimize(result.gradient);
  benchmark::DoNotOptimize(result.hessian);
  benchmark::ClobberMemory();
}

sxdetail::RigidIpcBarrierOptions activeBarrierOptions()
{
  sxdetail::RigidIpcBarrierOptions options;
  options.squaredActivationDistance = 1.0;
  options.stiffness = 1.0;
  options.projectReducedHessianToPsd = true;
  return options;
}

// A unit right-triangle surface placed at height z, matching the rigid IPC
// barrier unit tests.
sxdetail::RigidIpcBarrierSurface makeTriangleSurface(double z, bool dynamic)
{
  sxdetail::RigidIpcBarrierSurface surface;
  surface.dynamic = dynamic;
  surface.pose.position = Eigen::Vector3d(0.0, 0.0, z);
  surface.vertices
      = {Eigen::Vector3d(0.0, 0.0, 0.0),
         Eigen::Vector3d(1.0, 0.0, 0.0),
         Eigen::Vector3d(0.0, 1.0, 0.0)};
  surface.triangles = {Eigen::Vector3i(0, 1, 2)};
  return surface;
}

} // namespace

//==============================================================================
static void BM_RigidIpcReducedBarrier_PointTriangle(benchmark::State& state)
{
  const Eigen::Vector3d point(0.25, 0.25, 0.5);
  const sxdetail::RigidIpcPose pointPose;
  const Eigen::Vector3d a(0.0, 0.0, 0.0);
  const Eigen::Vector3d b(1.0, 0.0, 0.0);
  const Eigen::Vector3d c(0.0, 1.0, 0.0);
  const sxdetail::RigidIpcPose trianglePose;
  const auto options = activeBarrierOptions();

  for (auto _ : state) {
    auto result = sxdetail::rigidIpcPointTriangleReducedBarrier(
        point, pointPose, a, b, c, trianglePose, options);
    consumeReduced(result);
  }
}
BENCHMARK(BM_RigidIpcReducedBarrier_PointTriangle);

//==============================================================================
static void BM_RigidIpcReducedBarrier_PointEdge(benchmark::State& state)
{
  const Eigen::Vector3d point(0.5, 0.0, 0.5);
  const sxdetail::RigidIpcPose pointPose;
  const Eigen::Vector3d edgeA(0.0, 0.0, 0.0);
  const Eigen::Vector3d edgeB(1.0, 0.0, 0.0);
  const sxdetail::RigidIpcPose edgePose;
  const auto options = activeBarrierOptions();

  for (auto _ : state) {
    auto result = sxdetail::rigidIpcPointEdgeReducedBarrier(
        point, pointPose, edgeA, edgeB, edgePose, options);
    consumeReduced(result);
  }
}
BENCHMARK(BM_RigidIpcReducedBarrier_PointEdge);

//==============================================================================
static void BM_RigidIpcReducedBarrier_EdgeEdge(benchmark::State& state)
{
  const Eigen::Vector3d a0(0.0, 0.0, 0.0);
  const Eigen::Vector3d a1(1.0, 0.0, 0.0);
  const sxdetail::RigidIpcPose poseA;
  const Eigen::Vector3d b0(0.5, -0.5, 0.5);
  const Eigen::Vector3d b1(0.5, 0.5, 0.5);
  const sxdetail::RigidIpcPose poseB;
  const auto options = activeBarrierOptions();

  for (auto _ : state) {
    auto result = sxdetail::rigidIpcEdgeEdgeReducedBarrier(
        a0, a1, poseA, b0, b1, poseB, options);
    consumeReduced(result);
  }
}
BENCHMARK(BM_RigidIpcReducedBarrier_EdgeEdge);

//==============================================================================
static void BM_RigidIpcReducedBarrier_PointPoint(benchmark::State& state)
{
  const Eigen::Vector3d pointA(0.0, 0.0, 0.0);
  const sxdetail::RigidIpcPose poseA;
  const Eigen::Vector3d pointB(0.0, 0.0, 0.5);
  const sxdetail::RigidIpcPose poseB;
  const auto options = activeBarrierOptions();

  for (auto _ : state) {
    auto result = sxdetail::rigidIpcPointPointReducedBarrier(
        pointA, poseA, pointB, poseB, options);
    consumeReduced(result);
  }
}
BENCHMARK(BM_RigidIpcReducedBarrier_PointPoint);

//==============================================================================
// Scene-level sparse assembly over a stack of dynamic triangles above one
// static triangle. The body count is the benchmark argument so assembly cost
// can be tracked as the active contact set grows.
static void BM_RigidIpcAssembleBarrierSystem(benchmark::State& state)
{
  const auto bodyCount = static_cast<std::size_t>(state.range(0));
  std::vector<sxdetail::RigidIpcBarrierSurface> surfaces;
  surfaces.reserve(bodyCount + 1);
  surfaces.push_back(makeTriangleSurface(0.0, /*dynamic=*/false));
  for (std::size_t i = 0; i < bodyCount; ++i) {
    surfaces.push_back(makeTriangleSurface(
        0.4 * static_cast<double>(i + 1), /*dynamic=*/true));
  }
  const auto options = activeBarrierOptions();

  for (auto _ : state) {
    auto assembly = sxdetail::assembleRigidIpcBarrierSystem(surfaces, options);
    benchmark::DoNotOptimize(assembly.value);
    benchmark::DoNotOptimize(assembly.gradient);
    benchmark::DoNotOptimize(assembly.hessian);
    benchmark::ClobberMemory();
  }
  state.SetComplexityN(state.range(0));
}
BENCHMARK(BM_RigidIpcAssembleBarrierSystem)
    ->RangeMultiplier(2)
    ->Range(1, 32)
    ->Complexity();

//==============================================================================
// Full projected-Newton barrier solve on a two-body point contact, the core
// runtime hot path of the opt-in rigid IPC stage.
static void BM_RigidIpcProjectedNewtonSolve_TwoBody(benchmark::State& state)
{
  sxdetail::RigidIpcBarrierSurface dynamicBody;
  dynamicBody.dynamic = true;
  dynamicBody.pose.position = Eigen::Vector3d(0.0, 0.0, 0.25);
  dynamicBody.vertices.push_back(Eigen::Vector3d::Zero());

  sxdetail::RigidIpcBarrierSurface staticBody;
  staticBody.dynamic = false;
  staticBody.vertices.push_back(Eigen::Vector3d::Zero());

  const std::array<sxdetail::RigidIpcBarrierSurface, 2> surfaces{
      dynamicBody, staticBody};

  sxdetail::RigidIpcProjectedNewtonSolveOptions options;
  options.barrier.squaredActivationDistance = 1.0;
  options.newton.maxStepNorm = 0.05;
  options.maxIterations = static_cast<std::size_t>(state.range(0));

  sxdetail::RigidIpcProjectedNewtonSolveResult result;
  sxdetail::RigidIpcProjectedNewtonSolveScratch scratch;
  for (auto _ : state) {
    sxdetail::solveRigidIpcProjectedNewtonBarrierSystem(
        surfaces, options, result, scratch);
    benchmark::DoNotOptimize(result.surfaces);
    benchmark::DoNotOptimize(result.stats.finalValue);
    benchmark::ClobberMemory();
  }
}
BENCHMARK(BM_RigidIpcProjectedNewtonSolve_TwoBody)->Arg(4)->Arg(8)->Arg(16);

//==============================================================================
// Conservative curved-CCD line-search bound between a static and a dynamic
// triangle crossing it. CCD typically dominates IPC step cost.
static void BM_RigidIpcLineSearchStepBound(benchmark::State& state)
{
  const std::array<sxdetail::RigidIpcBarrierSurface, 2> startSurfaces{
      makeTriangleSurface(0.5, /*dynamic=*/true),
      makeTriangleSurface(0.0, /*dynamic=*/false)};
  std::array<sxdetail::RigidIpcBarrierSurface, 2> endSurfaces = startSurfaces;
  endSurfaces[0].pose.position.z() = -0.1;

  sxdetail::RigidIpcLineSearchOptions options;
  options.minSeparation = 0.0;

  for (auto _ : state) {
    auto result = sxdetail::computeRigidIpcLineSearchStepBound(
        startSurfaces, endSurfaces, options);
    benchmark::DoNotOptimize(result.stepBound);
    benchmark::DoNotOptimize(result.limited);
    benchmark::ClobberMemory();
  }
}
BENCHMARK(BM_RigidIpcLineSearchStepBound);

namespace {

// A stack of dynamic boxes resting in light contact above a static ground box,
// used to compare the per-step cost of the incumbent sequential-impulse rigid
// path against the opt-in rigid IPC path on an identical scene. Boxes start
// within the rigid IPC activation distance so the IPC barrier engages.
struct RigidBoxStackWorld
{
  explicit RigidBoxStackWorld(int boxCount, sx::RigidBodySolver solver)
  {
    world.setRigidBodySolver(solver);
    world.setTimeStep(1.0 / 240.0);

    sx::RigidBodyOptions groundOptions;
    groundOptions.isStatic = true;
    groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.05);
    auto ground = world.addRigidBody("ground", groundOptions);
    ground.setCollisionShape(
        sx::CollisionShape::makeBox(Eigen::Vector3d(2.0, 2.0, 0.05)));

    constexpr double half = 0.5;
    constexpr double gap = 5e-3; // Inside the runtime IPC activation distance.
    for (int i = 0; i < boxCount; ++i) {
      const double centerZ
          = half + gap + static_cast<double>(i) * (2.0 * half + gap);
      sx::RigidBodyOptions options;
      options.mass = 1.0;
      options.position = Eigen::Vector3d(0.0, 0.0, centerZ);
      auto body = world.addRigidBody("box_" + std::to_string(i), options);
      body.setCollisionShape(
          sx::CollisionShape::makeBox(Eigen::Vector3d(half, half, half)));
      boxes.push_back(body);
      initialPositions.push_back(options.position);
    }
  }

  void reset()
  {
    for (std::size_t i = 0; i < boxes.size(); ++i) {
      Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
      transform.translation() = initialPositions[i];
      boxes[i].setTransform(transform);
      boxes[i].setLinearVelocity(Eigen::Vector3d::Zero());
      boxes[i].setAngularVelocity(Eigen::Vector3d::Zero());
    }
  }

  sx::World world;
  std::vector<sx::RigidBody> boxes;
  std::vector<Eigen::Vector3d> initialPositions;
};

void runWorldStepComparison(benchmark::State& state, sx::RigidBodySolver solver)
{
  const auto boxCount = static_cast<int>(state.range(0));
  RigidBoxStackWorld fixture(boxCount, solver);
  // Prime simulation mode once so the timed region measures only stepping.
  fixture.world.step();

  for (auto _ : state) {
    state.PauseTiming();
    fixture.reset();
    state.ResumeTiming();
    fixture.world.step();
    benchmark::DoNotOptimize(fixture.boxes.back().getTranslation().z());
  }
  state.counters["boxes"] = static_cast<double>(boxCount);
}

struct LargeHashgridBodyBounds
{
  std::size_t vertexCount{0};
  std::size_t edgeCount{0};
  std::size_t faceCount{0};
  Eigen::Vector3d localMin = Eigen::Vector3d::Zero();
  Eigen::Vector3d localMax = Eigen::Vector3d::Zero();
  double radius{0.0};
  sxdetail::RigidIpcPose poseT0;
  sxdetail::RigidIpcPose poseT1;
};

struct LargeHashgridSceneBounds
{
  std::string upstreamPath;
  std::string sha256;
  std::size_t bodyCount{0};
  std::size_t vertexCount{0};
  std::size_t edgeCount{0};
  std::size_t faceCount{0};
  Eigen::Vector3d exactMin = Eigen::Vector3d::Zero();
  Eigen::Vector3d exactMax = Eigen::Vector3d::Zero();
  std::vector<LargeHashgridBodyBounds> bodies;
};

struct LargeHashgridAabb
{
  Eigen::Vector3d minimum
      = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector3d maximum
      = Eigen::Vector3d::Constant(-std::numeric_limits<double>::infinity());
};

template <typename T>
T readTsvValue(std::istringstream& input, const std::string& line)
{
  T value{};
  if (!(input >> value)) {
    throw std::runtime_error("Malformed rigid IPC large hashgrid row: " + line);
  }
  return value;
}

Eigen::Vector3d readTsvVector3(
    std::istringstream& input, const std::string& line)
{
  return {
      readTsvValue<double>(input, line),
      readTsvValue<double>(input, line),
      readTsvValue<double>(input, line)};
}

std::filesystem::path largeHashgridFixturePath()
{
  return std::filesystem::path(__FILE__)
             .parent_path()
             .parent_path()
             .parent_path()
             .parent_path()
         / "fixtures" / "rigid_ipc" / "large_hashgrid_bounds.tsv";
}

std::vector<LargeHashgridSceneBounds> loadLargeHashgridScenes()
{
  const auto path = largeHashgridFixturePath();
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error(
        "Could not open rigid IPC large hashgrid fixture: " + path.string());
  }

  std::vector<LargeHashgridSceneBounds> scenes;
  scenes.reserve(2);

  LargeHashgridSceneBounds* currentScene = nullptr;
  std::string line;
  while (std::getline(input, line)) {
    if (line.empty() || line.front() == '#') {
      continue;
    }

    std::istringstream row(line);
    const std::string kind = readTsvValue<std::string>(row, line);
    if (kind == "scene") {
      LargeHashgridSceneBounds scene;
      scene.upstreamPath = readTsvValue<std::string>(row, line);
      scene.sha256 = readTsvValue<std::string>(row, line);
      scene.bodyCount = readTsvValue<std::size_t>(row, line);
      scene.vertexCount = readTsvValue<std::size_t>(row, line);
      scene.edgeCount = readTsvValue<std::size_t>(row, line);
      scene.faceCount = readTsvValue<std::size_t>(row, line);
      scene.exactMin = readTsvVector3(row, line);
      scene.exactMax = readTsvVector3(row, line);
      scene.bodies.reserve(scene.bodyCount);
      scenes.push_back(std::move(scene));
      currentScene = &scenes.back();
      continue;
    }

    if (kind != "body") {
      throw std::runtime_error(
          "Unknown rigid IPC large hashgrid fixture row kind: " + kind);
    }
    if (currentScene == nullptr) {
      throw std::runtime_error(
          "Rigid IPC large hashgrid body row appears before any scene row");
    }

    const std::string upstreamPath = readTsvValue<std::string>(row, line);
    if (upstreamPath != currentScene->upstreamPath) {
      throw std::runtime_error(
          "Rigid IPC large hashgrid body row path does not match scene");
    }

    const auto bodyIndex = readTsvValue<std::size_t>(row, line);
    if (bodyIndex != currentScene->bodies.size()) {
      throw std::runtime_error(
          "Rigid IPC large hashgrid body rows are not contiguous");
    }

    LargeHashgridBodyBounds body;
    body.vertexCount = readTsvValue<std::size_t>(row, line);
    body.edgeCount = readTsvValue<std::size_t>(row, line);
    body.faceCount = readTsvValue<std::size_t>(row, line);
    body.localMin = readTsvVector3(row, line);
    body.localMax = readTsvVector3(row, line);
    body.radius = readTsvValue<double>(row, line);
    body.poseT0.position = readTsvVector3(row, line);
    body.poseT0.rotation = readTsvVector3(row, line);
    body.poseT1.position = readTsvVector3(row, line);
    body.poseT1.rotation = readTsvVector3(row, line);
    currentScene->bodies.push_back(body);
  }

  for (const auto& scene : scenes) {
    if (scene.bodies.size() != scene.bodyCount) {
      throw std::runtime_error(
          "Rigid IPC large hashgrid scene body count mismatch: "
          + scene.upstreamPath);
    }
  }

  return scenes;
}

const std::vector<LargeHashgridSceneBounds>& largeHashgridScenes()
{
  static const std::vector<LargeHashgridSceneBounds> scenes
      = loadLargeHashgridScenes();
  return scenes;
}

const LargeHashgridSceneBounds& findLargeHashgridScene(
    const std::string_view upstreamPath)
{
  for (const auto& scene : largeHashgridScenes()) {
    if (scene.upstreamPath == upstreamPath) {
      return scene;
    }
  }

  throw std::runtime_error(
      "Missing rigid IPC large hashgrid scene fixture for "
      + std::string(upstreamPath));
}

std::array<Eigen::Vector3d, 8> localAabbCorners(
    const Eigen::Vector3d& minimum, const Eigen::Vector3d& maximum)
{
  std::array<Eigen::Vector3d, 8> corners;
  std::size_t index = 0;
  for (int x = 0; x < 2; ++x) {
    for (int y = 0; y < 2; ++y) {
      for (int z = 0; z < 2; ++z) {
        corners[index++]
            = {x == 0 ? minimum.x() : maximum.x(),
               y == 0 ? minimum.y() : maximum.y(),
               z == 0 ? minimum.z() : maximum.z()};
      }
    }
  }
  return corners;
}

void includePoint(LargeHashgridAabb& bounds, const Eigen::Vector3d& point)
{
  bounds.minimum = bounds.minimum.cwiseMin(point);
  bounds.maximum = bounds.maximum.cwiseMax(point);
}

void includeSphere(
    LargeHashgridAabb& bounds,
    const Eigen::Vector3d& center,
    const double radius)
{
  const Eigen::Vector3d extent = Eigen::Vector3d::Constant(radius);
  includePoint(bounds, center - extent);
  includePoint(bounds, center + extent);
}

void includeBodyBounds(
    LargeHashgridAabb& bounds, const LargeHashgridBodyBounds& body)
{
  if ((body.poseT0.rotation.array() == body.poseT1.rotation.array()).all()) {
    const auto corners = localAabbCorners(body.localMin, body.localMax);
    for (const auto& pose : {body.poseT0, body.poseT1}) {
      for (const auto& corner : corners) {
        includePoint(bounds, sxdetail::transformRigidIpcPoint(corner, pose));
      }
    }
    return;
  }

  includeSphere(bounds, body.poseT0.position, body.radius);
  includeSphere(bounds, body.poseT1.position, body.radius);
}

LargeHashgridAabb computeLargeHashgridSceneBounds(
    const LargeHashgridSceneBounds& scene)
{
  LargeHashgridAabb bounds;
  for (const auto& body : scene.bodies) {
    includeBodyBounds(bounds, body);
  }
  return bounds;
}

std::string formatVector(const Eigen::Vector3d& value)
{
  std::ostringstream output;
  output << value.transpose();
  return output.str();
}

void requireContainsExactBounds(
    const LargeHashgridSceneBounds& scene, const LargeHashgridAabb& bounds)
{
  constexpr double tolerance = 1e-9;
  if ((bounds.minimum.array() > scene.exactMin.array() + tolerance).any()
      || (bounds.maximum.array() < scene.exactMax.array() - tolerance).any()) {
    throw std::runtime_error(
        "Rigid IPC large hashgrid benchmark bounds for " + scene.upstreamPath
        + " do not contain upstream exact bounds. computed min="
        + formatVector(bounds.minimum) + " max=" + formatVector(bounds.maximum)
        + " exact min=" + formatVector(scene.exactMin)
        + " max=" + formatVector(scene.exactMax));
  }
}

void recordLargeHashgridCounters(
    benchmark::State& state,
    const LargeHashgridSceneBounds& scene,
    const LargeHashgridAabb& bounds)
{
  const double slack = (scene.exactMin - bounds.minimum).sum()
                       + (bounds.maximum - scene.exactMax).sum();

  state.counters["bodies"] = static_cast<double>(scene.bodyCount);
  state.counters["source_vertices"] = static_cast<double>(scene.vertexCount);
  state.counters["source_edges"] = static_cast<double>(scene.edgeCount);
  state.counters["source_faces"] = static_cast<double>(scene.faceCount);
  state.counters["bbox_diag"] = (bounds.maximum - bounds.minimum).norm();
  state.counters["conservative_slack"] = slack;
  state.SetItemsProcessed(
      static_cast<std::int64_t>(state.iterations() * scene.bodies.size()));
}

} // namespace

//==============================================================================
// Incumbent baseline: per-step cost of the default sequential-impulse rigid
// pipeline on the box stack (benchmarks.md comparison baseline #1).
static void BM_RigidWorldStep_SequentialImpulse(benchmark::State& state)
{
  runWorldStepComparison(state, sx::RigidBodySolver::SequentialImpulse);
}
BENCHMARK(BM_RigidWorldStep_SequentialImpulse)->Arg(1)->Arg(2)->Arg(4);

//==============================================================================
// Opt-in rigid IPC path on the same scene. Compared per-step against the
// sequential-impulse baseline above; the contact models differ, so this is a
// throughput comparison of advancing the same scene one step, not a
// matched-accuracy claim.
static void BM_RigidWorldStep_Ipc(benchmark::State& state)
{
  runWorldStepComparison(state, sx::RigidBodySolver::Ipc);
}
BENCHMARK(BM_RigidWorldStep_Ipc)->Arg(1)->Arg(2)->Arg(4);

//==============================================================================
// Large rigid-body hash-grid corpus from the audited rigid-ipc data rows. This
// benchmark computes conservative swept scene bounds from compact per-body
// records and verifies those bounds contain the upstream exact scene bounds.
static void BM_RigidIpcLargeHashgridSceneBounds(
    benchmark::State& state, const char* upstreamPath)
{
  const auto& scene = findLargeHashgridScene(upstreamPath);
  LargeHashgridAabb bounds;
  for (auto _ : state) {
    bounds = computeLargeHashgridSceneBounds(scene);
    benchmark::DoNotOptimize(bounds.minimum.data());
    benchmark::DoNotOptimize(bounds.maximum.data());
  }

  requireContainsExactBounds(scene, bounds);
  recordLargeHashgridCounters(state, scene, bounds);
}
BENCHMARK_CAPTURE(
    BM_RigidIpcLargeHashgridSceneBounds,
    large_rb_hashgrid_000,
    "tests/data/large-rb-hashgrid/large-rb-hashgrid-000.json");
BENCHMARK_CAPTURE(
    BM_RigidIpcLargeHashgridSceneBounds,
    large_rb_hashgrid_001,
    "tests/data/large-rb-hashgrid/large-rb-hashgrid-001.json");
