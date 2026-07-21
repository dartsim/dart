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

// Bounded dynamics audit for the source-derived literal masonry wedges. This
// is deliberately separate from the production paper trace: the unavailable
// author scene still prevents a parity claim, while this probe establishes
// whether a documented collision backend and gap policy can support a finite,
// upright DART trajectory with the literal geometry and exact prism inertia.
//
// Usage:
//   fbf_paper_arch_wedge_dynamics_probe [25|101] [steps]
//       [native|fcl] [exact|boxed]
//       [source_offsets|nominal_touching|ground_gap_removed|
//        closure_1um|closure_10um|closure_100um|closure_500um|closure_1mm|
//        closure_2mm] [step_size_scale] [outer_iterations] [inner_sweeps]
//       [adaptive|fixed] [bootstrap_outer_iterations] [seed_mode]
//       [stabilization_mode] [simulation_threads] [legacy|colored]
//       [outer_relaxation]
//       [persistent|fresh]

// The defaults are: 25 25 native exact nominal_touching.

#include <dart/simulation/DeactivationOptions.hpp>
#include <dart/simulation/World.hpp>

#include <dart/constraint/BoxedLcpConstraintSolver.hpp>
#include <dart/constraint/ContactConstraint.hpp>
#include <dart/constraint/ExactCoulombFbfConstraintSolver.hpp>

#include <dart/collision/CollisionDetector.hpp>
#include <dart/collision/CollisionResult.hpp>
#include <dart/collision/fcl/FCLCollisionDetector.hpp>
#include <dart/collision/native/NativeCollisionDetector.hpp>

#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/ConvexMeshShape.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/Inertia.hpp>
#include <dart/dynamics/PlaneShape.hpp>
#include <dart/dynamics/ShapeNode.hpp>
#include <dart/dynamics/Skeleton.hpp>

#include <dart/math/detail/MasonryArchGeometry.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <cerrno>
#include <cmath>
#include <cstddef>
#include <cstdlib>

namespace {

constexpr double kTimeStep = 1.0 / 60.0;
constexpr double kGravity = 9.81;
constexpr double kFriction = 0.8;
constexpr double kDensity = 1000.0;
constexpr double kSourceGroundGap = 0.001;
constexpr std::size_t kMaxSteps = 600u;
constexpr std::size_t kMinimumStabilitySteps = 25u;
constexpr double kCrownDisplacementGate = 0.02;
constexpr double kCrownUprightCosGate = 0.95;
constexpr double kMaxBodyDisplacementGate = 0.05;
constexpr double kMinBodyUprightCosGate = 0.80;

enum class Backend
{
  Native,
  Fcl,
};

enum class SolverMode
{
  Exact,
  Boxed,
};

enum class SeedMode
{
  Zero,
  LocalDiagonal,
  DenseGlobal,
};

enum class StabilizationMode
{
  None,
  VelocityErp0p01,
  VelocityErp0p1,
};

enum class InnerSchedule
{
  Legacy,
  Colored,
};

struct GapPolicy
{
  const char* label;
  dart::math::detail::MasonryArchBarrierGapPolicy barrierPolicy;
  double endFaceExpansionMeters;
  double downwardShiftMeters;
};

struct Options
{
  std::size_t stoneCount = 25u;
  std::size_t steps = 25u;
  Backend backend = Backend::Native;
  SolverMode solverMode = SolverMode::Exact;
  GapPolicy gapPolicy{
      "nominal_touching",
      dart::math::detail::MasonryArchBarrierGapPolicy::OmitSourceOffsets,
      0.0,
      0.0};
  double stepSizeScale = 1.0;
  std::size_t outerIterations = 200u;
  std::size_t innerSweeps = 30u;
  bool adaptiveStepSize = true;
  std::size_t bootstrapOuterIterations = 0u;
  SeedMode seedMode = SeedMode::Zero;
  StabilizationMode stabilizationMode = StabilizationMode::None;
  std::size_t simulationThreads = 1u;
  InnerSchedule innerSchedule = InnerSchedule::Legacy;
  double outerRelaxation = 1.0;
  bool enableStepSizePersistence = true;
};

struct Scene
{
  std::shared_ptr<dart::simulation::World> world;
  std::vector<dart::dynamics::SkeletonPtr> stones;
  std::vector<Eigen::Isometry3d> initialTransforms;
  double exactVolume = 0.0;
  double exactMass = 0.0;
};

using BodyPair = std::pair<std::string, std::string>;

struct StateSample
{
  bool finite = true;
  double crownDisplacement = std::numeric_limits<double>::quiet_NaN();
  double crownVerticalDisplacement = std::numeric_limits<double>::quiet_NaN();
  double crownUprightCos = std::numeric_limits<double>::quiet_NaN();
  double maxBodyDisplacement = 0.0;
  double minBodyUprightCos = 1.0;
  double maxLinearSpeed = 0.0;
  double maxAngularSpeed = 0.0;
};

const char* backendLabel(Backend backend)
{
  return backend == Backend::Native ? "native" : "fcl_convex";
}

const char* solverLabel(SolverMode solverMode)
{
  return solverMode == SolverMode::Exact ? "exact_fbf" : "boxed_lcp";
}

const char* seedModeLabel(SeedMode seedMode)
{
  switch (seedMode) {
    case SeedMode::Zero:
      return "zero";
    case SeedMode::LocalDiagonal:
      return "local_diagonal";
    case SeedMode::DenseGlobal:
      return "dense_global";
  }
  return "unknown";
}

const char* stabilizationModeLabel(StabilizationMode mode)
{
  switch (mode) {
    case StabilizationMode::None:
      return "none";
    case StabilizationMode::VelocityErp0p01:
      return "velocity_erp_0p01";
    case StabilizationMode::VelocityErp0p1:
      return "velocity_erp_0p1";
  }
  return "unknown";
}

const char* innerScheduleLabel(InnerSchedule schedule)
{
  return schedule == InnerSchedule::Colored ? "colored" : "legacy";
}

std::string formatLogicalCpuIds(const std::vector<int>& logicalCpuIds)
{
  if (logicalCpuIds.empty())
    return "none";

  std::ostringstream output;
  for (std::size_t i = 0u; i < logicalCpuIds.size(); ++i) {
    if (i > 0u)
      output << ':';
    output << logicalCpuIds[i];
  }
  return output.str();
}

bool usesSplitImpulse(StabilizationMode mode)
{
  return mode == StabilizationMode::None;
}

double stabilizationErp(StabilizationMode mode)
{
  switch (mode) {
    case StabilizationMode::None:
      return 0.0;
    case StabilizationMode::VelocityErp0p01:
      return 0.01;
    case StabilizationMode::VelocityErp0p1:
      return 0.1;
  }
  return 0.0;
}

const char* exactStatusLabel(
    dart::constraint::ExactCoulombFbfConstraintSolverStatus status)
{
  using Status = dart::constraint::ExactCoulombFbfConstraintSolverStatus;
  switch (status) {
    case Status::NotRun:
      return "not_run";
    case Status::Success:
      return "success";
    case Status::MaxIterationsAccepted:
      return "max_iterations_accepted";
    case Status::PlateauAccepted:
      return "plateau_accepted";
    case Status::InvalidOptions:
      return "invalid_options";
    case Status::UnsupportedProblem:
      return "unsupported_problem";
    case Status::FbfFailed:
      return "fbf_failed";
    case Status::BoxedLcpFallback:
      return "boxed_lcp_fallback";
  }
  return "unknown";
}

const char* fbfStatusLabel(dart::math::detail::ExactCoulombFbfStatus status)
{
  using Status = dart::math::detail::ExactCoulombFbfStatus;
  switch (status) {
    case Status::Success:
      return "success";
    case Status::MaxIterations:
      return "max_iterations";
    case Status::InvalidInput:
      return "invalid_input";
    case Status::InnerSolverFailed:
      return "inner_solver_failed";
    case Status::StepSizeUnderflow:
      return "step_size_underflow";
    case Status::Plateau:
      return "plateau";
    case Status::NonFiniteValue:
      return "non_finite_value";
  }
  return "unknown";
}

bool parseSize(const char* value, std::size_t& output)
{
  if (value == nullptr || value[0] == '\0')
    return false;
  errno = 0;
  char* end = nullptr;
  const unsigned long long parsed = std::strtoull(value, &end, 10);
  if (end == value || *end != '\0' || errno == ERANGE
      || parsed > std::numeric_limits<std::size_t>::max()) {
    return false;
  }
  output = static_cast<std::size_t>(parsed);
  return true;
}

bool parseBackend(const std::string& value, Backend& output)
{
  if (value == "native") {
    output = Backend::Native;
    return true;
  }
  if (value == "fcl" || value == "fcl_convex") {
    output = Backend::Fcl;
    return true;
  }
  return false;
}

bool parsePositiveDouble(const char* value, double& output)
{
  if (value == nullptr || value[0] == '\0')
    return false;
  errno = 0;
  char* end = nullptr;
  const double parsed = std::strtod(value, &end);
  if (end == value || *end != '\0' || errno == ERANGE || !std::isfinite(parsed)
      || parsed <= 0.0) {
    return false;
  }
  output = parsed;
  return true;
}

bool parseSolver(const std::string& value, SolverMode& output)
{
  if (value == "exact" || value == "exact_fbf") {
    output = SolverMode::Exact;
    return true;
  }
  if (value == "boxed" || value == "boxed_lcp") {
    output = SolverMode::Boxed;
    return true;
  }
  return false;
}

bool parseSeedMode(const std::string& value, SeedMode& output)
{
  if (value == "zero") {
    output = SeedMode::Zero;
    return true;
  }
  if (value == "local_diagonal") {
    output = SeedMode::LocalDiagonal;
    return true;
  }
  if (value == "dense_global") {
    output = SeedMode::DenseGlobal;
    return true;
  }
  return false;
}

bool parseStabilizationMode(const std::string& value, StabilizationMode& output)
{
  if (value == "none") {
    output = StabilizationMode::None;
    return true;
  }
  if (value == "velocity_erp_0p01") {
    output = StabilizationMode::VelocityErp0p01;
    return true;
  }
  if (value == "velocity_erp_0p1") {
    output = StabilizationMode::VelocityErp0p1;
    return true;
  }
  return false;
}

bool parseInnerSchedule(const std::string& value, InnerSchedule& output)
{
  if (value == "legacy") {
    output = InnerSchedule::Legacy;
    return true;
  }
  if (value == "colored") {
    output = InnerSchedule::Colored;
    return true;
  }
  return false;
}

bool parseGapPolicy(const std::string& value, GapPolicy& output)
{
  using Barrier = dart::math::detail::MasonryArchBarrierGapPolicy;
  if (value == "source_offsets") {
    output = {"source_offsets", Barrier::IncludeSourceOffsets, 0.0, 0.0};
    return true;
  }
  if (value == "nominal_touching") {
    output = {"nominal_touching", Barrier::OmitSourceOffsets, 0.0, 0.0};
    return true;
  }
  if (value == "ground_gap_removed") {
    output
        = {"ground_gap_removed",
           Barrier::OmitSourceOffsets,
           0.0,
           kSourceGroundGap};
    return true;
  }
  if (value == "closure_1um") {
    output
        = {"closure_1um",
           Barrier::OmitSourceOffsets,
           1e-6,
           kSourceGroundGap + 1e-6};
    return true;
  }
  if (value == "closure_10um") {
    output
        = {"closure_10um",
           Barrier::OmitSourceOffsets,
           1e-5,
           kSourceGroundGap + 1e-5};
    return true;
  }
  if (value == "closure_100um") {
    output
        = {"closure_100um",
           Barrier::OmitSourceOffsets,
           1e-4,
           kSourceGroundGap + 1e-4};
    return true;
  }
  if (value == "closure_500um") {
    output
        = {"closure_500um",
           Barrier::OmitSourceOffsets,
           5e-4,
           kSourceGroundGap + 5e-4};
    return true;
  }
  if (value == "closure_1mm") {
    output
        = {"closure_1mm",
           Barrier::OmitSourceOffsets,
           1e-3,
           kSourceGroundGap + 1e-3};
    return true;
  }
  if (value == "closure_2mm") {
    output
        = {"closure_2mm",
           Barrier::OmitSourceOffsets,
           2e-3,
           kSourceGroundGap + 2e-3};
    return true;
  }
  return false;
}

bool parseOptions(int argc, char* argv[], Options& options)
{
  if (argc > 17)
    return false;
  if (argc > 1 && !parseSize(argv[1], options.stoneCount))
    return false;
  if (argc > 2 && !parseSize(argv[2], options.steps))
    return false;
  if (argc > 3 && !parseBackend(argv[3], options.backend))
    return false;
  if (argc > 4 && !parseSolver(argv[4], options.solverMode))
    return false;
  if (argc > 5 && !parseGapPolicy(argv[5], options.gapPolicy))
    return false;
  if (argc > 6 && !parsePositiveDouble(argv[6], options.stepSizeScale))
    return false;
  if (argc > 7 && !parseSize(argv[7], options.outerIterations))
    return false;
  if (argc > 8 && !parseSize(argv[8], options.innerSweeps))
    return false;
  if (argc > 9) {
    const std::string stepPolicy = argv[9];
    if (stepPolicy == "adaptive") {
      options.adaptiveStepSize = true;
    } else if (stepPolicy == "fixed") {
      options.adaptiveStepSize = false;
    } else {
      return false;
    }
  }
  if (argc > 10 && !parseSize(argv[10], options.bootstrapOuterIterations)) {
    return false;
  }
  if (argc > 11 && !parseSeedMode(argv[11], options.seedMode))
    return false;
  if (argc > 12
      && !parseStabilizationMode(argv[12], options.stabilizationMode)) {
    return false;
  }
  if (argc > 13 && !parseSize(argv[13], options.simulationThreads))
    return false;
  if (argc > 14 && !parseInnerSchedule(argv[14], options.innerSchedule))
    return false;
  if (argc > 15 && !parsePositiveDouble(argv[15], options.outerRelaxation))
    return false;
  if (argc > 16) {
    const std::string persistence = argv[16];
    if (persistence == "persistent") {
      options.enableStepSizePersistence = true;
    } else if (persistence == "fresh") {
      options.enableStepSizePersistence = false;
    } else {
      return false;
    }
  }
  return (options.stoneCount == 25u || options.stoneCount == 101u)
         && options.steps > 0u && options.steps <= kMaxSteps
         && options.outerIterations > 0u && options.outerIterations <= 120000u
         && options.innerSweeps > 0u && options.innerSweeps <= 1000u
         && options.bootstrapOuterIterations <= 120000u
         && options.simulationThreads > 0u && options.simulationThreads <= 256u
         && options.outerRelaxation <= 2.0
         && (options.bootstrapOuterIterations == 0u
             || options.solverMode == SolverMode::Exact)
         && (options.seedMode == SeedMode::Zero
             || options.solverMode == SolverMode::Exact)
         && (options.innerSchedule == InnerSchedule::Legacy
             || options.solverMode == SolverMode::Exact);
}

std::shared_ptr<dart::collision::CollisionDetector> makeDetector(
    Backend backend)
{
  if (backend == Backend::Native) {
    auto detector = dart::collision::NativeCollisionDetector::create();
    detector->setContactManifoldMode(dart::collision::NativeCollisionDetector::
                                         ContactManifoldMode::FourPointPlanar);
    return detector;
  }

  auto detector = dart::collision::FCLCollisionDetector::create();
  detector->setPrimitiveShapeType(
      dart::collision::FCLCollisionDetector::PRIMITIVE);
  detector->setContactPointComputationMethod(
      dart::collision::FCLCollisionDetector::FCL);
  return detector;
}

dart::constraint::ExactCoulombFbfConstraintSolverOptions makeExactOptions(
    const Options& probeOptions)
{
  dart::constraint::ExactCoulombFbfConstraintSolverOptions options;
  options.fallbackToBoxedLcp = false;
  options.maxOuterIterations = static_cast<int>(
      probeOptions.bootstrapOuterIterations > 0u
          ? probeOptions.bootstrapOuterIterations
          : probeOptions.outerIterations);
  options.acceptOuterMaxIterations = true;
  options.tolerance = 1e-6;
  options.innerMaxSweeps = static_cast<int>(probeOptions.innerSweeps);
  options.runFixedInnerSweeps = true;
  options.innerLocalSolver = dart::constraint::ExactCoulombFbfLocalBlockSolver::
      ExactMetricProjection;
  options.innerLocalIterations = 1;
  options.stepSizeScale = probeOptions.stepSizeScale;
  options.enableAdaptiveStepSize = probeOptions.adaptiveStepSize;
  options.outerRelaxation = probeOptions.outerRelaxation;
  options.enableWarmStart = true;
  options.enableProjectedGradientRetry = false;
  options.enableDenseResidualPolish = false;
  options.seedNormalImpulseFromDiagonal
      = probeOptions.seedMode != SeedMode::Zero;
  options.useMatrixFreeDelassusSeed
      = probeOptions.seedMode == SeedMode::LocalDiagonal;
  options.assembleDenseContactRowSnapshot
      = probeOptions.seedMode == SeedMode::DenseGlobal;
  options.enableStepSizePersistence = probeOptions.enableStepSizePersistence;
  return options;
}

std::shared_ptr<dart::dynamics::ConvexMeshShape> makeConvexShape(
    const dart::math::detail::MasonryArchStoneWedgeGeometry& geometry)
{
  dart::dynamics::ConvexMeshShape::Vertices vertices;
  vertices.reserve(geometry.vertices.size());
  for (const auto& vertex : geometry.vertices)
    vertices.push_back(vertex - geometry.centroid);

  dart::dynamics::ConvexMeshShape::Triangles triangles;
  const auto& sourceTriangles
      = dart::math::detail::getMasonryArchStoneWedgeTriangles();
  triangles.reserve(sourceTriangles.size());
  for (const auto& triangle : sourceTriangles) {
    triangles.emplace_back(
        static_cast<Eigen::Index>(triangle[0]),
        static_cast<Eigen::Index>(triangle[1]),
        static_cast<Eigen::Index>(triangle[2]));
  }
  return std::make_shared<dart::dynamics::ConvexMeshShape>(vertices, triangles);
}

dart::dynamics::SkeletonPtr makeGround()
{
  auto skeleton = dart::dynamics::Skeleton::create("masonry_arch_ground");
  auto* body = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>()
                   .second;
  body->setName("masonry_arch_ground_body");
  auto* shapeNode = body->createShapeNodeWith<
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(
      std::make_shared<dart::dynamics::PlaneShape>(
          Eigen::Vector3d::UnitZ(), 0.0));
  shapeNode->setName("masonry_arch_ground_shape");
  shapeNode->getDynamicsAspect()->setFrictionCoeff(kFriction);
  skeleton->setMobile(false);
  return skeleton;
}

Scene makeScene(const Options& options)
{
  Scene scene;
  scene.world = dart::simulation::World::create("literal_masonry_arch_probe");
  scene.world->setTimeStep(kTimeStep);
  scene.world->setGravity(Eigen::Vector3d(0.0, 0.0, -kGravity));
  scene.world->setNumSimulationThreads(options.simulationThreads);

  dart::simulation::DeactivationOptions deactivation;
  deactivation.mEnabled = false;
  scene.world->setDeactivationOptions(deactivation);

  const auto detector = makeDetector(options.backend);
  if (options.solverMode == SolverMode::Exact) {
    auto solver
        = std::make_unique<dart::constraint::ExactCoulombFbfConstraintSolver>(
            makeExactOptions(options));
    solver->setCollisionDetector(detector);
    solver->setNumSimulationThreads(options.simulationThreads);
    scene.world->setConstraintSolver(std::move(solver));
    auto* installed
        = static_cast<dart::constraint::ExactCoulombFbfConstraintSolver*>(
            scene.world->getConstraintSolver());
    installed->setExactCoulombColoredBlockGaussSeidelEnabled(
        options.innerSchedule == InnerSchedule::Colored);
  } else {
    auto solver
        = std::make_unique<dart::constraint::BoxedLcpConstraintSolver>();
    solver->setCollisionDetector(detector);
    solver->setNumSimulationThreads(options.simulationThreads);
    scene.world->setConstraintSolver(std::move(solver));
  }

  // This static setter remains process-local to this standalone diagnostic.
  // The velocity modes expose Baumgarte sensitivity without changing DART's
  // production defaults.
  dart::constraint::ContactConstraint::setErrorReductionParameter(
      stabilizationErp(options.stabilizationMode));
  scene.world->getConstraintSolver()->setSplitImpulseEnabled(
      usesSplitImpulse(options.stabilizationMode));
  auto& collisionOption
      = scene.world->getConstraintSolver()->getCollisionOption();
  collisionOption.maxNumContacts = options.stoneCount * 16u;
  collisionOption.maxNumContactsPerPair = 8u;

  scene.world->addSkeleton(makeGround());
  const auto geometries = dart::math::detail::generateMasonryArchStoneWedges(
      options.stoneCount,
      {},
      options.gapPolicy.barrierPolicy,
      options.gapPolicy.endFaceExpansionMeters);
  scene.stones.reserve(options.stoneCount);
  scene.initialTransforms.reserve(options.stoneCount);
  for (std::size_t i = 0u; i < options.stoneCount; ++i) {
    const auto& geometry = geometries[i];
    auto skeleton = dart::dynamics::Skeleton::create(
        "masonry_arch_wedge_" + std::to_string(i));
    auto pair
        = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>();
    auto* joint = pair.first;
    auto* body = pair.second;
    body->setName(skeleton->getName() + "_body");

    const auto shape = makeConvexShape(geometry);
    auto* shapeNode = body->createShapeNodeWith<
        dart::dynamics::CollisionAspect,
        dart::dynamics::DynamicsAspect>(shape);
    shapeNode->setName(skeleton->getName() + "_shape");
    shapeNode->getDynamicsAspect()->setFrictionCoeff(kFriction);

    const double mass = kDensity * geometry.volume;
    dart::dynamics::Inertia inertia;
    inertia.setMass(mass);
    inertia.setMoment(mass * geometry.momentPerUnitMass);
    body->setInertia(inertia);

    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.translation()
        = geometry.centroid
          - options.gapPolicy.downwardShiftMeters * Eigen::Vector3d::UnitZ();
    joint->setPositions(
        dart::dynamics::FreeJoint::convertToPositions(transform));
    if (i == 0u || i + 1u == options.stoneCount)
      skeleton->setMobile(false);

    scene.exactVolume += geometry.volume;
    scene.exactMass += mass;
    scene.initialTransforms.push_back(transform);
    scene.world->addSkeleton(skeleton);
    scene.stones.push_back(std::move(skeleton));
  }
  return scene;
}

dart::constraint::ExactCoulombFbfConstraintSolver* getExactSolver(
    const Scene& scene)
{
  return dynamic_cast<dart::constraint::ExactCoulombFbfConstraintSolver*>(
      scene.world->getConstraintSolver());
}

double rotationCosine(const Eigen::Matrix3d& relativeRotation)
{
  return std::clamp(0.5 * (relativeRotation.trace() - 1.0), -1.0, 1.0);
}

StateSample sampleState(const Scene& scene)
{
  StateSample sample;
  const std::size_t crownIndex = scene.stones.size() / 2u;
  for (std::size_t i = 0u; i < scene.stones.size(); ++i) {
    const auto* body = scene.stones[i]->getBodyNode(0);
    const auto transform = body->getWorldTransform();
    const auto& initial = scene.initialTransforms[i];
    const Eigen::Vector3d displacement
        = transform.translation() - initial.translation();
    const double uprightCos
        = rotationCosine(initial.linear().transpose() * transform.linear());
    const double linearSpeed = body->getLinearVelocity().norm();
    const double angularSpeed = body->getAngularVelocity().norm();
    sample.finite = sample.finite && transform.matrix().allFinite()
                    && body->getLinearVelocity().allFinite()
                    && body->getAngularVelocity().allFinite()
                    && std::isfinite(uprightCos) && std::isfinite(linearSpeed)
                    && std::isfinite(angularSpeed);
    if (!scene.stones[i]->isMobile())
      continue;
    sample.maxBodyDisplacement
        = std::max(sample.maxBodyDisplacement, displacement.norm());
    sample.minBodyUprightCos = std::min(sample.minBodyUprightCos, uprightCos);
    sample.maxLinearSpeed = std::max(sample.maxLinearSpeed, linearSpeed);
    sample.maxAngularSpeed = std::max(sample.maxAngularSpeed, angularSpeed);
    if (i == crownIndex) {
      sample.crownDisplacement = displacement.norm();
      sample.crownVerticalDisplacement = displacement.z();
      sample.crownUprightCos = uprightCos;
    }
  }
  return sample;
}

BodyPair orderedBodyPair(const dart::collision::Contact& contact)
{
  const auto first = contact.getBodyNodePtr1();
  const auto second = contact.getBodyNodePtr2();
  std::string firstName = first != nullptr ? first->getName() : "null";
  std::string secondName = second != nullptr ? second->getName() : "null";
  if (secondName < firstName)
    std::swap(firstName, secondName);
  return {firstName, secondName};
}

std::map<BodyPair, std::size_t> getBodyPairs(
    const dart::collision::CollisionResult& collisionResult,
    bool& contactsFinite)
{
  std::map<BodyPair, std::size_t> pairs;
  contactsFinite = true;
  for (const auto& contact : collisionResult.getContacts()) {
    ++pairs[orderedBodyPair(contact)];
    contactsFinite = contactsFinite && std::isfinite(contact.penetrationDepth)
                     && contact.point.allFinite() && contact.normal.allFinite();
  }
  return pairs;
}

} // namespace

int main(int argc, char* argv[])
{
  Options options;
  if (!parseOptions(argc, argv, options)) {
    std::cerr
        << "usage: " << argv[0]
        << " [25|101] [steps:1..600] [native|fcl] [exact|boxed] "
           "[source_offsets|nominal_touching|ground_gap_removed|"
           "closure_1um|closure_10um|closure_100um|closure_500um|"
           "closure_1mm|closure_2mm] [step_size_scale] [outer_iterations] "
           "[inner_sweeps] [adaptive|fixed] "
           "[bootstrap_outer_iterations:0..120000] "
           "[zero|local_diagonal|dense_global] "
           "[none|velocity_erp_0p01|velocity_erp_0p1] "
           "[simulation_threads:1..256] [legacy|colored] "
           "[outer_relaxation:(0,2]] [persistent|fresh]\n";
    return 2;
  }

  std::cout << std::setprecision(17);
  Scene scene;
  try {
    scene = makeScene(options);
  } catch (const std::exception& error) {
    std::cout << "verdict,completed=false,finite=false,"
              << "error=scene_construction_failed,message=" << error.what()
              << ",paper_parity_claim=false\n";
    return 1;
  }

  auto* exactSolver = getExactSolver(scene);
  std::cout
      << "metadata,stone_count=" << options.stoneCount
      << ",steps_requested=" << options.steps
      << ",backend=" << backendLabel(options.backend) << ",manifold_mode="
      << (options.backend == Backend::Native ? "four_point_planar"
                                             : "backend_default")
      << ",solver=" << solverLabel(options.solverMode)
      << ",gap_policy=" << options.gapPolicy.label << ",barrier_offsets="
      << (options.gapPolicy.barrierPolicy
                  == dart::math::detail::MasonryArchBarrierGapPolicy::
                      IncludeSourceOffsets
              ? "included"
              : "omitted")
      << ",end_face_expansion_m=" << options.gapPolicy.endFaceExpansionMeters
      << ",downward_shift_m=" << options.gapPolicy.downwardShiftMeters
      << ",dt_s=" << kTimeStep << ",friction=" << kFriction
      << ",density_kg_m3=" << kDensity
      << ",step_size_scale=" << options.stepSizeScale
      << ",outer_iterations=" << options.outerIterations
      << ",inner_sweeps=" << options.innerSweeps
      << ",adaptive_step_size=" << (options.adaptiveStepSize ? "true" : "false")
      << ",bootstrap_diagnostic="
      << (options.bootstrapOuterIterations > 0u ? "true" : "false")
      << ",bootstrap_outer_iterations=" << options.bootstrapOuterIterations
      << ",bootstrap_steps="
      << (options.bootstrapOuterIterations > 0u ? 1u : 0u)
      << ",bootstrap_paper_comparable=false"
      << ",seed_diagnostic="
      << (options.seedMode == SeedMode::Zero ? "false" : "true")
      << ",seed_mode=" << seedModeLabel(options.seedMode)
      << ",seed_paper_comparable=false"
      << ",seed_operator_contract="
      << (options.seedMode == SeedMode::DenseGlobal
              ? "dense_contact_row_snapshot"
              : "contact_row_matrix_free")
      << ",seed_parallel_contract="
      << (options.seedMode == SeedMode::DenseGlobal ? "changed" : "preserved")
      << ",stabilization_diagnostic="
      << (options.stabilizationMode == StabilizationMode::None ? "false"
                                                               : "true")
      << ",stabilization_mode="
      << stabilizationModeLabel(options.stabilizationMode)
      << ",stabilization_paper_comparable=false"
      << ",simulation_threads=" << options.simulationThreads
      << ",inner_schedule=" << innerScheduleLabel(options.innerSchedule)
      << ",outer_relaxation=" << options.outerRelaxation
      << ",step_size_persistence="
      << (options.enableStepSizePersistence ? "persistent" : "fresh")
      << ",inner_schedule_paper_comparable="
      << (options.innerSchedule == InnerSchedule::Legacy ? "unknown" : "false")
      << ",inner_schedule_contract="
      << (options.innerSchedule == InnerSchedule::Colored
              ? "dart_deterministic_manifold_colored_bgs_diagnostic"
              : "dart_legacy_serial_bgs")
      << ",paper_velocity_baumgarte_published=true"
      << ",paper_velocity_baumgarte_parameter_published=false"
      << ",exact_volume_m3=" << scene.exactVolume
      << ",exact_mass_kg=" << scene.exactMass
      << ",pinned_springers=0:" << options.stoneCount - 1u << ",split_impulse="
      << (usesSplitImpulse(options.stabilizationMode) ? "true" : "false")
      << ",error_reduction_parameter="
      << stabilizationErp(options.stabilizationMode)
      << ",error_reduction_parameter_scope=process_global_static"
      << ",max_contacts=" << options.stoneCount * 16u
      << ",max_contacts_per_pair=8"
      << ",minimum_stability_steps=" << kMinimumStabilitySteps
      << ",crown_displacement_gate_m=" << kCrownDisplacementGate
      << ",crown_upright_cos_gate=" << kCrownUprightCosGate
      << ",max_body_displacement_gate_m=" << kMaxBodyDisplacementGate
      << ",min_body_upright_cos_gate=" << kMinBodyUprightCosGate
      << ",author_scene_available=false,paper_parity_claim=false\n";

  bool valid = true;
  bool completed = true;
  std::size_t stepsCompleted = 0u;
  double elapsedTotalMs = 0.0;
  std::size_t maxContactsObserved = 0u;
  std::size_t minContactsObserved = std::numeric_limits<std::size_t>::max();
  StateSample lastState;

  for (std::size_t step = 1u; step <= options.steps; ++step) {
    if (exactSolver != nullptr && step == 2u
        && options.bootstrapOuterIterations > 0u) {
      auto standardOptions = exactSolver->getExactCoulombOptions();
      standardOptions.maxOuterIterations
          = static_cast<int>(options.outerIterations);
      exactSolver->setExactCoulombOptions(standardOptions);
    }
    const std::size_t stepOuterIterationBudget
        = step == 1u && options.bootstrapOuterIterations > 0u
              ? options.bootstrapOuterIterations
              : options.outerIterations;
    const std::size_t attemptsBefore
        = exactSolver != nullptr ? exactSolver->getNumExactCoulombAttempts()
                                 : 0u;
    const std::size_t solvesBefore
        = exactSolver != nullptr ? exactSolver->getNumExactCoulombSolves() : 0u;
    const std::size_t failuresBefore
        = exactSolver != nullptr ? exactSolver->getNumExactCoulombFailures()
                                 : 0u;
    const std::size_t fallbacksBefore
        = exactSolver != nullptr ? exactSolver->getNumBoxedLcpFallbacks() : 0u;
    const std::size_t cappedBefore
        = exactSolver != nullptr
              ? exactSolver->getNumExactCoulombMaxIterationsAccepted()
              : 0u;

    const auto start = std::chrono::steady_clock::now();
    try {
      scene.world->step();
    } catch (const std::exception& error) {
      std::cout << "error,step=" << step
                << ",kind=world_step_exception,message=" << error.what()
                << '\n';
      valid = false;
      completed = false;
      break;
    }
    const auto stop = std::chrono::steady_clock::now();
    const double elapsedMs
        = std::chrono::duration<double, std::milli>(stop - start).count();
    elapsedTotalMs += elapsedMs;
    ++stepsCompleted;

    const auto& collisionResult
        = scene.world->getConstraintSolver()->getLastCollisionResult();
    bool contactsFinite = true;
    const auto bodyPairs = getBodyPairs(collisionResult, contactsFinite);
    std::size_t maxContactsOnBodyPair = 0u;
    for (const auto& [bodyPair, pairContacts] : bodyPairs) {
      static_cast<void>(bodyPair);
      maxContactsOnBodyPair = std::max(maxContactsOnBodyPair, pairContacts);
    }
    const std::size_t contacts = collisionResult.getNumContacts();
    maxContactsObserved = std::max(maxContactsObserved, contacts);
    minContactsObserved = std::min(minContactsObserved, contacts);
    lastState = sampleState(scene);

    const std::size_t attemptsDelta
        = exactSolver != nullptr
              ? exactSolver->getNumExactCoulombAttempts() - attemptsBefore
              : 0u;
    const std::size_t solvesDelta
        = exactSolver != nullptr
              ? exactSolver->getNumExactCoulombSolves() - solvesBefore
              : 0u;
    const std::size_t failuresDelta
        = exactSolver != nullptr
              ? exactSolver->getNumExactCoulombFailures() - failuresBefore
              : 0u;
    const std::size_t fallbacksDelta
        = exactSolver != nullptr
              ? exactSolver->getNumBoxedLcpFallbacks() - fallbacksBefore
              : 0u;
    const std::size_t cappedDelta
        = exactSolver != nullptr
              ? exactSolver->getNumExactCoulombMaxIterationsAccepted()
                    - cappedBefore
              : 0u;

    std::cout << "step,index=" << step << ",bootstrap_step="
              << (step == 1u && options.bootstrapOuterIterations > 0u ? "true"
                                                                      : "false")
              << ",outer_iteration_budget=" << stepOuterIterationBudget
              << ",sim_time_s=" << scene.world->getTime()
              << ",elapsed_ms=" << elapsedMs << ",contacts=" << contacts
              << ",unique_body_pairs=" << bodyPairs.size()
              << ",max_contacts_on_body_pair=" << maxContactsOnBodyPair
              << ",contacts_finite=" << (contactsFinite ? "true" : "false")
              << ",state_finite=" << (lastState.finite ? "true" : "false")
              << ",crown_displacement_m=" << lastState.crownDisplacement
              << ",crown_vertical_displacement_m="
              << lastState.crownVerticalDisplacement
              << ",crown_upright_cos=" << lastState.crownUprightCos
              << ",max_body_displacement_m=" << lastState.maxBodyDisplacement
              << ",min_body_upright_cos=" << lastState.minBodyUprightCos
              << ",max_linear_speed_m_s=" << lastState.maxLinearSpeed
              << ",max_angular_speed_rad_s=" << lastState.maxAngularSpeed
              << ",exact_attempts=" << attemptsDelta
              << ",exact_solves=" << solvesDelta
              << ",exact_failures=" << failuresDelta
              << ",boxed_fallbacks=" << fallbacksDelta
              << ",max_iterations_accepted=" << cappedDelta;
    if (exactSolver != nullptr) {
      const auto& residualDetails
          = exactSolver->getLastExactCoulombResidualDetails();
      std::cout
          << ",exact_status="
          << exactStatusLabel(exactSolver->getLastExactCoulombStatus())
          << ",fbf_status="
          << fbfStatusLabel(exactSolver->getLastExactCoulombFbfStatus())
          << ",residual=" << exactSolver->getLastExactCoulombResidual()
          << ",best_residual=" << exactSolver->getLastExactCoulombBestResidual()
          << ",primal_residual=" << residualDetails.primalFeasibility
          << ",dual_residual=" << residualDetails.dualFeasibility
          << ",complementarity_residual=" << residualDetails.complementarity
          << ",step_size=" << exactSolver->getLastExactCoulombStepSize()
          << ",safe_step_size="
          << exactSolver->getLastExactCoulombSafeStepSize()
          << ",coupling_variation_ratio="
          << exactSolver->getLastExactCoulombCouplingVariationRatio()
          << ",iterations=" << exactSolver->getLastExactCoulombIterations()
          << ",colored_bgs_requested="
          << (exactSolver->getExactCoulombColoredBlockGaussSeidelEnabled()
                  ? "true"
                  : "false")
          << ",colored_bgs_used="
          << (exactSolver->getLastExactCoulombColoredBlockGaussSeidelUsed()
                  ? "true"
                  : "false")
          << ",colored_bgs_solves="
          << exactSolver->getLastExactCoulombColoredBlockGaussSeidelSolves()
          << ",colored_bgs_dispatches="
          << exactSolver->getLastExactCoulombColoredBlockGaussSeidelDispatches()
          << ",colored_bgs_max_participants="
          << exactSolver
                 ->getLastExactCoulombColoredBlockGaussSeidelParticipants()
          << ",colored_bgs_manifolds="
          << exactSolver->getLastExactCoulombColoredBlockGaussSeidelManifolds()
          << ",colored_bgs_colors="
          << exactSolver->getLastExactCoulombColoredBlockGaussSeidelColors()
          << ",colored_bgs_max_manifolds_per_color="
          << exactSolver
                 ->getLastExactCoulombColoredBlockGaussSeidelMaxManifoldsPerColor()
          << ",colored_bgs_logical_cpus="
          << formatLogicalCpuIds(
                 exactSolver
                     ->getLastExactCoulombColoredBlockGaussSeidelLogicalCpuIds())
          << ",colored_bgs_max_phase_logical_cpus="
          << formatLogicalCpuIds(
                 exactSolver
                     ->getLastExactCoulombColoredBlockGaussSeidelMaxPhaseLogicalCpuIds());
    } else {
      std::cout << ",exact_status=not_applicable,fbf_status=not_applicable"
                << ",residual=nan,best_residual=nan,iterations=0";
    }
    std::cout << '\n';

    for (const auto& [bodyPair, pairContacts] : bodyPairs) {
      std::cout << "pair,step=" << step << ",first=" << bodyPair.first
                << ",second=" << bodyPair.second << ",contacts=" << pairContacts
                << '\n';
    }

    const bool exactFailure = failuresDelta > 0u || fallbacksDelta > 0u;
    if (!lastState.finite || !contactsFinite || exactFailure) {
      valid = false;
      completed = false;
      std::cout << "stop,step=" << step << ",reason="
                << (!lastState.finite || !contactsFinite
                        ? "nonfinite_state_or_contact"
                        : "exact_failure_or_fallback")
                << '\n';
      break;
    }
  }

  if (stepsCompleted == 0u)
    minContactsObserved = 0u;
  const bool stabilityDurationMet = stepsCompleted >= kMinimumStabilitySteps;
  const bool boundedStable
      = stabilityDurationMet && valid && completed && lastState.finite
        && lastState.crownDisplacement <= kCrownDisplacementGate
        && lastState.crownUprightCos >= kCrownUprightCosGate
        && lastState.maxBodyDisplacement <= kMaxBodyDisplacementGate
        && lastState.minBodyUprightCos >= kMinBodyUprightCosGate;
  std::cout << "verdict,completed=" << (completed ? "true" : "false")
            << ",steps_completed=" << stepsCompleted
            << ",finite=" << (lastState.finite ? "true" : "false")
            << ",stability_duration_met="
            << (stabilityDurationMet ? "true" : "false")
            << ",bounded_stability_gate="
            << (!stabilityDurationMet ? "not_evaluated"
                                      : (boundedStable ? "pass" : "fail"))
            << ",crown_displacement_m=" << lastState.crownDisplacement
            << ",crown_upright_cos=" << lastState.crownUprightCos
            << ",max_body_displacement_m=" << lastState.maxBodyDisplacement
            << ",min_body_upright_cos=" << lastState.minBodyUprightCos
            << ",min_contacts=" << minContactsObserved
            << ",max_contacts=" << maxContactsObserved
            << ",elapsed_total_ms=" << elapsedTotalMs
            << ",elapsed_mean_step_ms="
            << (stepsCompleted > 0u
                    ? elapsedTotalMs / static_cast<double>(stepsCompleted)
                    : std::numeric_limits<double>::quiet_NaN());
  if (exactSolver != nullptr) {
    std::cout << ",exact_attempts=" << exactSolver->getNumExactCoulombAttempts()
              << ",exact_solves=" << exactSolver->getNumExactCoulombSolves()
              << ",exact_failures=" << exactSolver->getNumExactCoulombFailures()
              << ",boxed_fallbacks=" << exactSolver->getNumBoxedLcpFallbacks()
              << ",max_iterations_accepted="
              << exactSolver->getNumExactCoulombMaxIterationsAccepted()
              << ",worst_residual="
              << exactSolver->getWorstExactCoulombResidual();
  } else {
    std::cout << ",exact_attempts=0,exact_solves=0,exact_failures=0"
              << ",boxed_fallbacks=0,max_iterations_accepted=0"
              << ",worst_residual=nan";
  }
  std::cout << ",author_scene_available=false,paper_parity_claim=false\n";

  // A finite but displaced result remains useful evidence and is represented
  // by the explicit bounded-stability verdict. The process fails only on an
  // incomplete/nonfinite trajectory or an exact-solver failure/fallback.
  return valid && completed ? 0 : 1;
}
