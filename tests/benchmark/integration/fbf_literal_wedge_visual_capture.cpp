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

// Off-screen renderer for the reconstructed literal 25-stone wedge arch used
// by fbf_paper_trace's masonry_arch_25_literal_wedge scenario. This is not the
// older oriented-box demo adapter and it is not a paper-parity claim. The only
// additions to the physical trace scene are VisualAspect instances and a
// visual-only finite floor proxy; collision shapes, exact prism inertia,
// solver options, pinned springers, and initial transforms follow the current
// literal Native/FourPointPlanar contract.

#include "../../../examples/demos/scenes/FbfLiteralMasonryArchSpec.hpp"
#include "dart/collision/CollisionResult.hpp"
#include "dart/constraint/ExactCoulombFbfConstraintSolver.hpp"
#include "dart/dynamics/BodyNode.hpp"
#include "dart/gui/osg/OffscreenViewer.hpp"
#include "dart/gui/osg/Viewer.hpp"
#include "dart/gui/osg/WorldNode.hpp"
#include "dart/simulation/World.hpp"

#include <Eigen/Geometry>
#include <osg/Camera>
#include <osg/Vec3>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <cerrno>
#include <cmath>
#include <cstddef>
#include <cstdlib>

namespace {

namespace literalArch = fbf_literal_masonry_arch;

constexpr double kDt = literalArch::kTimeStep;
constexpr double kFriction = literalArch::kFriction;
constexpr double kDensity = literalArch::kDensity;
constexpr std::size_t kStoneCount = literalArch::kStoneCount;
constexpr double kEndFaceExpansion = literalArch::kEndFaceExpansion;
constexpr double kDownwardShift = literalArch::kDownwardShift;
constexpr double kTolerance = literalArch::kResidualTolerance;
constexpr std::size_t kDefaultSteps = 600u;
constexpr std::size_t kDefaultFrameStride = 10u;
constexpr std::size_t kDefaultThreads = 1u;
constexpr int kDefaultWidth = 1280;
constexpr int kDefaultHeight = 720;

struct Configuration
{
  std::filesystem::path outputDirectory;
  std::size_t steps = kDefaultSteps;
  std::size_t frameStride = kDefaultFrameStride;
  std::size_t simulationThreads = kDefaultThreads;
  int width = kDefaultWidth;
  int height = kDefaultHeight;
};

struct InitialPose
{
  const dart::dynamics::BodyNode* body = nullptr;
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
};

struct ArchMetrics
{
  bool finite = true;
  double maxDisplacement = 0.0;
  double minOrientationAlignment = 1.0;
};

struct StepMetrics
{
  std::size_t step = 0u;
  double time = 0.0;
  std::size_t contacts = 0u;
  std::size_t attempts = 0u;
  std::size_t solves = 0u;
  std::size_t failures = 0u;
  std::size_t fallbacks = 0u;
  double residual = std::numeric_limits<double>::quiet_NaN();
  int iterations = 0;
  bool coloredUsed = false;
  std::size_t coloredManifolds = 0u;
  std::size_t coloredColors = 0u;
  std::size_t coloredMaxManifoldsPerColor = 0u;
  double maxArchDisplacement = 0.0;
  double minArchOrientationAlignment = 1.0;
  double crownHeight = std::numeric_limits<double>::quiet_NaN();
};

bool parsePositiveSize(const char* text, std::size_t& value)
{
  if (text == nullptr || text[0] == '\0' || text[0] == '-')
    return false;

  errno = 0;
  char* end = nullptr;
  const unsigned long long parsed = std::strtoull(text, &end, 10);
  if (errno == ERANGE || end == text || *end != '\0' || parsed == 0u
      || parsed > std::numeric_limits<std::size_t>::max()) {
    return false;
  }
  value = static_cast<std::size_t>(parsed);
  return true;
}

bool parsePositiveInt(const char* text, int& value)
{
  std::size_t parsed = 0u;
  if (!parsePositiveSize(text, parsed)
      || parsed > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
    return false;
  }
  value = static_cast<int>(parsed);
  return true;
}

bool parseArguments(int argc, char* argv[], Configuration& config)
{
  if (argc < 2 || argv[1] == nullptr || argv[1][0] == '\0')
    return false;

  config.outputDirectory = argv[1];
  return (argc <= 2 || parsePositiveSize(argv[2], config.steps))
         && (argc <= 3 || parsePositiveSize(argv[3], config.frameStride))
         && (argc <= 4 || parsePositiveInt(argv[4], config.width))
         && (argc <= 5 || parsePositiveInt(argv[5], config.height))
         && (argc <= 6 || parsePositiveSize(argv[6], config.simulationThreads))
         && argc <= 7;
}

void printUsage(const char* executable)
{
  std::cerr << "Usage: " << executable
            << " OUTPUT_DIR [STEPS [FRAME_STRIDE [WIDTH [HEIGHT "
               "[SIMULATION_THREADS]]]]]\n";
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

dart::simulation::WorldPtr createCaptureWorld(std::size_t simulationThreads)
{
  auto world = literalArch::createWorld(
      literalArch::SolverLane::ExactFbf,
      literalArch::VisualMode::DemoPalette,
      simulationThreads);
  if (world)
    world->setName("fbf_literal_wedge_visual_capture");
  return world;
}

std::vector<InitialPose> collectInitialPoses(
    const dart::simulation::WorldPtr& world)
{
  std::vector<InitialPose> poses;
  poses.reserve(kStoneCount);
  for (std::size_t i = 0u; i < kStoneCount; ++i) {
    const auto skeleton
        = world->getSkeleton("masonry_arch_stone_" + std::to_string(i));
    if (skeleton == nullptr || skeleton->getNumBodyNodes() != 1u)
      return {};
    const auto* body = skeleton->getBodyNode(0);
    poses.push_back(InitialPose{body, body->getWorldTransform()});
  }
  return poses;
}

ArchMetrics collectArchMetrics(const std::vector<InitialPose>& initialPoses)
{
  ArchMetrics metrics;
  for (const auto& pose : initialPoses) {
    if (pose.body == nullptr) {
      metrics.finite = false;
      continue;
    }
    const Eigen::Isometry3d current = pose.body->getWorldTransform();
    const Eigen::Vector3d displacement
        = current.translation() - pose.transform.translation();
    const Eigen::Matrix3d relativeRotation
        = pose.transform.linear().transpose() * current.linear();
    if (!current.matrix().allFinite() || !displacement.allFinite()
        || !relativeRotation.allFinite()) {
      metrics.finite = false;
      continue;
    }
    metrics.maxDisplacement
        = std::max(metrics.maxDisplacement, displacement.norm());
    metrics.minOrientationAlignment = std::min(
        metrics.minOrientationAlignment,
        std::clamp(0.5 * (relativeRotation.trace() - 1.0), -1.0, 1.0));
  }
  return metrics;
}

double crownHeight(const dart::simulation::WorldPtr& world)
{
  const auto crown = world->getSkeleton("masonry_arch_stone_12");
  if (crown == nullptr || crown->getNumBodyNodes() != 1u)
    return std::numeric_limits<double>::quiet_NaN();
  return crown->getBodyNode(0)->getWorldTransform().translation().z();
}

bool captureFrame(
    dart::gui::osg::Viewer& viewer,
    const std::filesystem::path& path,
    const ::osg::Vec3& eye,
    const ::osg::Vec3& center,
    const ::osg::Vec3& up)
{
  auto* camera = viewer.getCamera();
  camera->setViewMatrixAsLookAt(eye, center, up);
  viewer.frame();
  camera->setViewMatrixAsLookAt(eye, center, up);
  viewer.captureScreen(path.string());
  viewer.frame();

  std::error_code error;
  return std::filesystem::is_regular_file(path, error) && !error
         && std::filesystem::file_size(path, error) > 0u && !error;
}

std::string frameRelativePath(std::size_t step)
{
  std::ostringstream path;
  path << "frames/step_" << std::setw(6) << std::setfill('0') << step << ".png";
  return path.str();
}

void writeRuntimeJson(
    const Configuration& config,
    const std::vector<std::size_t>& capturedSteps,
    const StepMetrics& finalMetrics,
    double worstResidual,
    double maxDisplacement,
    double minOrientationAlignment,
    std::size_t totalAttempts,
    std::size_t totalSolves,
    std::size_t totalFailures,
    std::size_t totalFallbacks,
    const std::string& initialPhysicalGeometryFingerprint)
{
  const auto runtimePath = config.outputDirectory / "capture-runtime.json";
  std::ofstream output(runtimePath);
  output << std::setprecision(17);
  output
      << "{\n"
      << "  \"schema_version\": \"dart.fbf_literal_wedge_visual_runtime/v2\",\n"
      << "  \"claim_scope\": \"current-source reconstructed literal-wedge DART "
         "evidence; not paper parity\",\n"
      << "  \"scenario\": \"masonry_arch_25_literal_wedge\",\n"
      << "  \"scene_contract\": "
         "\"reconstructed_literal_wedge_arch_nonpaper_native_collision_"
         "frontend\",\n"
      << "  \"solver\": \"exact_fbf\",\n"
      << "  \"solver_contract\": "
         "\"dart_best_nonpaper_colored_inner_bgs\",\n"
      << "  \"solver_contract_argument\": \"dart_best_colored_bgs\",\n"
      << "  \"collision_frontend\": \"native\",\n"
      << "  \"contact_manifold_mode\": \"four_point_planar\",\n"
      << "  \"visual_only_difference\": \"VisualAspect instances were added to "
         "the trace-identical wedge shapes, plus a finite floor proxy with no "
         "CollisionAspect or DynamicsAspect; the physical PlaneShape remains "
         "nonvisual\",\n"
      << "  \"steps_requested\": " << config.steps << ",\n"
      << "  \"steps_completed\": " << finalMetrics.step << ",\n"
      << "  \"frame_stride\": " << config.frameStride << ",\n"
      << "  \"simulation_threads\": " << config.simulationThreads << ",\n"
      << "  \"width\": " << config.width << ",\n"
      << "  \"height\": " << config.height << ",\n"
      << "  \"dt\": " << kDt << ",\n"
      << "  \"friction\": " << kFriction << ",\n"
      << "  \"density\": " << kDensity << ",\n"
      << "  \"stone_count\": " << kStoneCount << ",\n"
      << "  \"pinned_springers\": [0, 24],\n"
      << "  \"end_face_expansion_m\": " << kEndFaceExpansion << ",\n"
      << "  \"downward_shift_m\": " << kDownwardShift << ",\n"
      << "  \"initial_physical_geometry_fingerprint\": {\n"
      << "    \"algorithm\": \"fnv1a64_q1e-10_le_v1\",\n"
      << "    \"value\": \"" << initialPhysicalGeometryFingerprint << "\"\n"
      << "  },\n"
      << "  \"exact_options_summary\": {\n"
      << "    \"scope\": \"selected evidence controls only; the shared "
         "FbfLiteralMasonryArchSpec.hpp contract is authoritative\",\n"
      << "    \"max_outer_iterations\": 5000,\n"
      << "    \"tolerance\": " << kTolerance << ",\n"
      << "    \"inner_max_sweeps\": 30,\n"
      << "    \"run_fixed_inner_sweeps\": true,\n"
      << "    \"inner_local_solver\": \"exact_metric_projection\",\n"
      << "    \"inner_local_iterations\": 1,\n"
      << "    \"step_size_scale\": 35.0,\n"
      << "    \"outer_relaxation\": 1.1,\n"
      << "    \"enable_warm_start\": true,\n"
      << "    \"enable_step_size_persistence\": false,\n"
      << "    \"fallback_to_boxed_lcp\": false\n"
      << "  },\n"
      << "  \"outcome\": {\n"
      << "    \"exact_attempts\": " << totalAttempts << ",\n"
      << "    \"exact_solves\": " << totalSolves << ",\n"
      << "    \"exact_failures\": " << totalFailures << ",\n"
      << "    \"boxed_lcp_fallbacks\": " << totalFallbacks << ",\n"
      << "    \"worst_exact_residual\": " << worstResidual << ",\n"
      << "    \"max_arch_body_displacement_from_initial\": " << maxDisplacement
      << ",\n"
      << "    \"min_arch_body_orientation_alignment_from_initial\": "
      << minOrientationAlignment << ",\n"
      << "    \"final_crown_height\": " << finalMetrics.crownHeight << "\n"
      << "  },\n"
      << "  \"camera\": {\n"
      << "    \"eye\": [0.0, -1.45, 0.36],\n"
      << "    \"center\": [0.0, 0.0, 0.31],\n"
      << "    \"up\": [0.0, 0.0, 1.0],\n"
      << "    \"fov_y_degrees\": 30.0\n"
      << "  },\n"
      << "  \"frames\": [\n";
  for (std::size_t i = 0u; i < capturedSteps.size(); ++i) {
    output << "    {\"step\": " << capturedSteps[i] << ", \"path\": \""
           << frameRelativePath(capturedSteps[i]) << "\"}"
           << (i + 1u == capturedSteps.size() ? "\n" : ",\n");
  }
  output << "  ]\n}\n";
}

} // namespace

int main(int argc, char* argv[])
{
  Configuration config;
  if (!parseArguments(argc, argv, config)) {
    printUsage(argv[0]);
    return 2;
  }
  if (config.width < 320 || config.height < 240) {
    std::cerr << "Capture dimensions must be at least 320x240.\n";
    return 2;
  }

  std::error_code fileError;
  std::filesystem::create_directories(
      config.outputDirectory / "frames", fileError);
  if (fileError) {
    std::cerr << "Failed to create output directory: " << fileError.message()
              << "\n";
    return 1;
  }

  literalArch::ScopedContactErrorReductionParameter scopedContactErp;
  auto world = createCaptureWorld(config.simulationThreads);
  if (world == nullptr) {
    std::cerr << "Failed to build the literal 25-stone wedge world.\n";
    return 1;
  }
  const std::string initialPhysicalGeometryFingerprint
      = literalArch::physicalGeometryFingerprint(
          literalArch::inspectPhysicsContract(world));
  const auto initialPoses = collectInitialPoses(world);
  if (initialPoses.size() != kStoneCount) {
    std::cerr << "Expected 25 literal arch bodies, found "
              << initialPoses.size() << ".\n";
    return 1;
  }

  auto* exactSolver
      = dynamic_cast<dart::constraint::ExactCoulombFbfConstraintSolver*>(
          world->getConstraintSolver());
  if (exactSolver == nullptr) {
    std::cerr << "The exact-FBF solver was not installed.\n";
    return 1;
  }

  ::osg::ref_ptr<dart::gui::osg::Viewer> viewer
      = new dart::gui::osg::Viewer(::osg::Vec4(0.93f, 0.94f, 0.96f, 1.0f));
  ::osg::ref_ptr<dart::gui::osg::WorldNode> worldNode
      = new dart::gui::osg::WorldNode(world);
  viewer->addWorldNode(worldNode);

  dart::gui::osg::OffscreenSetup setup;
  setup.width = config.width;
  setup.height = config.height;
  if (!dart::gui::osg::setUpOffscreenViewer(*viewer, setup))
    return 1;

  const ::osg::Vec3 eye(0.0f, -1.45f, 0.36f);
  const ::osg::Vec3 center(0.0f, 0.0f, 0.31f);
  const ::osg::Vec3 up(0.0f, 0.0f, 1.0f);
  for (int i = 0; i < 4; ++i) {
    viewer->getCamera()->setViewMatrixAsLookAt(eye, center, up);
    viewer->frame();
  }

  std::ofstream trajectory(config.outputDirectory / "trajectory.csv");
  trajectory << std::setprecision(17)
             << "step,sim_time,contacts,exact_attempts_delta,"
                "exact_solves_delta,exact_failures_delta,boxed_lcp_fallbacks_"
                "delta,exact_status,fbf_status,residual,iterations,"
                "colored_bgs_used,colored_bgs_manifolds,colored_bgs_colors,"
                "colored_bgs_max_manifolds_per_color,"
                "max_arch_body_displacement_from_initial,"
                "min_arch_body_orientation_alignment_from_initial,crown_z\n";

  StepMetrics metrics;
  metrics.crownHeight = crownHeight(world);
  trajectory << "0,0,0,0,0,0,0,not_run,invalid_input,nan,0,0,0,0,0,0,1,"
             << metrics.crownHeight << "\n";

  std::vector<std::size_t> capturedSteps;
  const auto captureAt = [&](std::size_t step) {
    const auto relative = frameRelativePath(step);
    const auto path = config.outputDirectory / relative;
    if (!captureFrame(*viewer, path, eye, center, up)) {
      std::cerr << "Failed to capture " << path << ".\n";
      return false;
    }
    capturedSteps.push_back(step);
    return true;
  };
  if (!captureAt(0u))
    return 1;

  double worstResidual = 0.0;
  double maxDisplacement = 0.0;
  double minOrientationAlignment = 1.0;
  std::size_t previousAttempts = exactSolver->getNumExactCoulombAttempts();
  std::size_t previousSolves = exactSolver->getNumExactCoulombSolves();
  std::size_t previousFailures = exactSolver->getNumExactCoulombFailures();
  std::size_t previousFallbacks = exactSolver->getNumBoxedLcpFallbacks();

  for (std::size_t step = 1u; step <= config.steps; ++step) {
    world->step();
    metrics.step = step;
    metrics.time = world->getTime();
    metrics.contacts = exactSolver->getLastCollisionResult().getNumContacts();
    metrics.attempts
        = exactSolver->getNumExactCoulombAttempts() - previousAttempts;
    metrics.solves = exactSolver->getNumExactCoulombSolves() - previousSolves;
    metrics.failures
        = exactSolver->getNumExactCoulombFailures() - previousFailures;
    metrics.fallbacks
        = exactSolver->getNumBoxedLcpFallbacks() - previousFallbacks;
    previousAttempts = exactSolver->getNumExactCoulombAttempts();
    previousSolves = exactSolver->getNumExactCoulombSolves();
    previousFailures = exactSolver->getNumExactCoulombFailures();
    previousFallbacks = exactSolver->getNumBoxedLcpFallbacks();
    metrics.residual = exactSolver->getLastExactCoulombResidual();
    metrics.iterations = exactSolver->getLastExactCoulombIterations();
    metrics.coloredUsed
        = exactSolver->getLastExactCoulombColoredBlockGaussSeidelUsed();
    metrics.coloredManifolds
        = exactSolver->getLastExactCoulombColoredBlockGaussSeidelManifolds();
    metrics.coloredColors
        = exactSolver->getLastExactCoulombColoredBlockGaussSeidelColors();
    metrics.coloredMaxManifoldsPerColor
        = exactSolver
              ->getLastExactCoulombColoredBlockGaussSeidelMaxManifoldsPerColor();
    const ArchMetrics archMetrics = collectArchMetrics(initialPoses);
    metrics.maxArchDisplacement = archMetrics.maxDisplacement;
    metrics.minArchOrientationAlignment = archMetrics.minOrientationAlignment;
    metrics.crownHeight = crownHeight(world);

    const auto exactStatus = exactSolver->getLastExactCoulombStatus();
    const auto fbfStatus = exactSolver->getLastExactCoulombFbfStatus();
    trajectory << metrics.step << ',' << metrics.time << ',' << metrics.contacts
               << ',' << metrics.attempts << ',' << metrics.solves << ','
               << metrics.failures << ',' << metrics.fallbacks << ','
               << exactStatusLabel(exactStatus) << ','
               << fbfStatusLabel(fbfStatus) << ',' << metrics.residual << ','
               << metrics.iterations << ',' << (metrics.coloredUsed ? 1 : 0)
               << ',' << metrics.coloredManifolds << ','
               << metrics.coloredColors << ','
               << metrics.coloredMaxManifoldsPerColor << ','
               << metrics.maxArchDisplacement << ','
               << metrics.minArchOrientationAlignment << ','
               << metrics.crownHeight << '\n';

    if (!archMetrics.finite || !std::isfinite(metrics.crownHeight)
        || metrics.attempts == 0u || metrics.solves != metrics.attempts
        || metrics.failures != 0u || metrics.fallbacks != 0u
        || exactStatus
               != dart::constraint::ExactCoulombFbfConstraintSolverStatus::
                   Success
        || fbfStatus != dart::math::detail::ExactCoulombFbfStatus::Success
        || !std::isfinite(metrics.residual) || metrics.residual > kTolerance) {
      std::cerr << "Exact trajectory gate failed at step " << step
                << ": status=" << exactStatusLabel(exactStatus)
                << ", fbf_status=" << fbfStatusLabel(fbfStatus)
                << ", residual=" << metrics.residual
                << ", attempts=" << metrics.attempts
                << ", solves=" << metrics.solves
                << ", failures=" << metrics.failures
                << ", fallbacks=" << metrics.fallbacks << ".\n";
      return 1;
    }

    worstResidual = std::max(worstResidual, metrics.residual);
    maxDisplacement = std::max(maxDisplacement, metrics.maxArchDisplacement);
    minOrientationAlignment = std::min(
        minOrientationAlignment, metrics.minArchOrientationAlignment);

    if (step % config.frameStride == 0u || step == config.steps) {
      if (!captureAt(step))
        return 1;
    }
  }

  writeRuntimeJson(
      config,
      capturedSteps,
      metrics,
      worstResidual,
      maxDisplacement,
      minOrientationAlignment,
      exactSolver->getNumExactCoulombAttempts(),
      exactSolver->getNumExactCoulombSolves(),
      exactSolver->getNumExactCoulombFailures(),
      exactSolver->getNumBoxedLcpFallbacks(),
      initialPhysicalGeometryFingerprint);

  std::cout << std::setprecision(17)
            << "capture_complete=true,steps=" << metrics.step
            << ",frames=" << capturedSteps.size()
            << ",contacts=" << metrics.contacts
            << ",worst_residual=" << worstResidual
            << ",max_arch_displacement=" << maxDisplacement
            << ",min_arch_orientation_alignment=" << minOrientationAlignment
            << ",exact_failures=" << exactSolver->getNumExactCoulombFailures()
            << ",boxed_lcp_fallbacks=" << exactSolver->getNumBoxedLcpFallbacks()
            << '\n';
  return 0;
}
