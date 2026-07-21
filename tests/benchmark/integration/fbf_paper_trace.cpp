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

// Headless CSV trace exporter for the small FBF paper-fixture scenes. This is
// a comparison artifact producer, not an installed API.
//
// Usage:
//   fbf_paper_trace [scenario=backspin] [solver=exact_fbf] [sample_stride=30]
//                   [steps=paper_duration] [initial_gamma=nan]
//                   [trace_scope=tracked] [warm_start=default|0|1]
//                   [split_impulse=default|0|1] [simulation_threads=1]
//                   [solver_contract=dart_best|dart_best_colored_bgs|paper_cpu|
//                                    paper_cpu_bootstrap_diagnostic]
//                   [collision_frontend=dart|native]
//                   [local_solver=default|exact_metric|inverse_euclidean|
//                                  projected_gradient]
//                   [bootstrap_outer_iterations=0]
//                   [post_bootstrap_outer_iterations=0]
//                   [native_manifold_sensitivity=default|compact|
//                                                four_point_planar]
//                   [cross_step_policy=default|dart_current|
//                                      author_policy_inspired_b3f3c5c]
//
// Scenarios:
//   incline_mu_0_5, incline_mu_0_4, backspin,
//   turntable_mu_0_2_omega_2, turntable_mu_0_2_omega_5,
//   turntable_mu_0_5_omega_2, turntable_mu_0_5_omega_5,
//   turntable_author_mu_0_2_omega_2,
//   turntable_author_mu_0_2_omega_5,
//   turntable_author_mu_0_5_omega_2,
//   turntable_author_mu_0_5_omega_5 (Native collision required),
//   painleve_mu_0_5, painleve_mu_0_55,
//   card_house_26_reduced_contact (full natural manifold, 96 initial contacts
//   in the repaired reconstruction),
//   card_house_26_settle_projectile_reduced_contact,
//   masonry_arch_25_reduced_contact,
//   masonry_arch_25_projectile_reduced_contact,
//   masonry_arch_101_reduced_contact,
//   masonry_arch_25_full_manifold, masonry_arch_101_full_manifold,
//   masonry_arch_25_literal_wedge, masonry_arch_101_literal_wedge,
//   masonry_arch_25_literal_wedge_crown_impact_v1
//
// Solvers:
//   exact_fbf, boxed_lcp
//
// Trace scopes:
//   tracked, dynamic_bodies, full_scene, residual_history, phase_summary,
//   performance

#include "../../../examples/demos/scenes/FbfAuthorTurntableSpec.hpp"

#include <dart/simulation/DeactivationOptions.hpp>
#include <dart/simulation/World.hpp>

#include <dart/constraint/ContactConstraint.hpp>
#include <dart/constraint/ExactCoulombFbfConstraintSolver.hpp>

#include <dart/collision/dart/DARTCollisionDetector.hpp>
#include <dart/collision/native/NativeCollisionDetector.hpp>

#include <dart/dynamics/BoxShape.hpp>
#include <dart/dynamics/ConvexMeshShape.hpp>
#include <dart/dynamics/CylinderShape.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/Inertia.hpp>
#include <dart/dynamics/PlaneShape.hpp>
#include <dart/dynamics/ShapeFrame.hpp>
#include <dart/dynamics/Skeleton.hpp>
#include <dart/dynamics/SphereShape.hpp>

#include <dart/math/detail/MasonryArchGeometry.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <exception>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <cerrno>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

namespace {

constexpr double kDt = 1.0 / 60.0;
constexpr double kGravity = 9.81;
constexpr double kInclineTan = 0.5;
constexpr double kBackspinRadius = 0.25;
constexpr double kBackspinLinearVelocity = 4.0;
constexpr double kBackspinAngularVelocity = -200.0;
constexpr double kBackspinFriction = 0.5;
constexpr double kTurntableInitialRadius = 1.0;
constexpr double kTurntableRampDuration = 1.0;
constexpr double kPainleveInitialPitch = 0.08;
constexpr double kPainleveInitialVelocity = 3.6;
constexpr double kCardHouseFriction = 0.8;
constexpr double kCardHouseAngle = 0.23;
constexpr double kCardHouseHeight = 1.0;
constexpr double kCardHouseWidth = 0.45;
constexpr double kCardHouseThickness = 0.03;
// Source-informed reconstruction: Newton 1.3 defaults rigid shapes to
// 1000 kg/m^3. The unavailable paper scene may override this value.
constexpr double kCardHouseDensity = 1000.0;
constexpr double kCardHouseInitialPenetration = 0.003;
// Reconstruction choice: adjacent horizontal one-meter cards meet with the
// same nominal 3 mm overlap used at the other interfaces. The paper does not
// publish this spacing.
constexpr double kCardHouseFrameSpacing
    = kCardHouseHeight - kCardHouseInitialPenetration;
constexpr double kCardHouseStepSizeScale = 10.0;
constexpr double kCardHouseOuterRelaxation = 1.5;
constexpr std::size_t kCardHouseSettleProjectileSettleSteps = 1u;
constexpr std::size_t kCardHouseSettleProjectileDefaultSteps = 2u;
// Paper Fig. 6 timing at dt = 1/60: 6.7 s no-creep settle, four launched
// projectiles, then run to 10 s total.
constexpr std::size_t kCardHouseSettleProjectileFullSettleSteps = 402u;
// Optional CLI override for split-impulse stepping: -1 keeps the scenario
// default, 0 forces it off, 1 forces it on (diagnostics for the base
// split-impulse interaction).
int gSplitImpulseOverride = -1;
constexpr std::size_t kCardHouseSettleProjectileFullDefaultSteps = 600u;
// Optional CLI override for the scenario warm-start policy: -1 keeps the
// scenario default, 0 forces cold starts, 1 enables cross-step manifold
// warm starts (essential for bounded long-run evidence).
int gWarmStartOverride = -1;
// Optional diagnostic-only override for dart_best. paper_cpu deliberately
// fixes the rigorous exact-metric local cone solve because the paper does not
// publish enough of its local 3x3 kernel to reproduce that implementation.
bool gLocalSolverOverrideSet = false;
dart::constraint::ExactCoulombFbfLocalBlockSolver gLocalSolverOverride = dart::
    constraint::ExactCoulombFbfLocalBlockSolver::InverseEuclideanProjection;
constexpr std::size_t kCardHouseProjectileCount = 4u;
// Official paper imagery shows cube projectiles. The edge length preserves the
// former sphere diameter but remains reconstructed: the author dimensions,
// mass, and launch speed are unpublished.
constexpr double kCardHouseProjectileEdgeLength = 0.11;
constexpr double kCardHouseProjectileMass
    = kCardHouseDensity * kCardHouseProjectileEdgeLength
      * kCardHouseProjectileEdgeLength * kCardHouseProjectileEdgeLength;
constexpr double kCardHouseProjectileSpeed = 4.0;
constexpr double kCardHouseProjectileDropHeight = 4.45;
constexpr double kSmallFixtureStepSizeScale = 2.0;
constexpr std::size_t kCardHouseFourLevelCount = 4u;
// Full natural manifold for the repaired 26-card reconstruction: 512/4 caps
// observe 96 contacts in the initial configuration. This is a measured DART
// reconstruction count, not the paper timing row's 214-contact contract.
constexpr std::size_t kCardHouseReducedMaxContacts = 512u;
constexpr std::size_t kCardHouseReducedMaxContactsPerPair = 4u;
// The paper's contact-rich arch experiments explicitly use mu=0.8. The
// credited Rigid-IPC geometry scene uses 0.5, but that source value is not the
// paper experiment's contact contract.
constexpr double kArchFriction = 0.8;
// Rigid-IPC's default body density ("plastic", src/io/read_rb_scene.cpp:68).
constexpr double kArchDensity = 1000.0;
constexpr std::size_t kArchStoneCount = 25u;
constexpr std::size_t kArch101StoneCount = 101u;
constexpr std::size_t kArchReducedMaxContacts = 48u;
constexpr std::size_t kArchReducedMaxContactsPerPair = 2u;
constexpr std::size_t kArch101ReducedMaxContacts = 38u;
constexpr std::size_t kArch101ReducedMaxContactsPerPair = 2u;
// Full natural manifold for both arches at per_pair 4: the 25-stone arch
// observes 96 actual contacts (96 also at per_pair 8, i.e. the manifold is
// saturated), and the 101-stone arch observes the full 512-contact cap.
constexpr std::size_t kArchFullManifoldMaxContacts = 512u;
constexpr std::size_t kArchFullManifoldMaxContactsPerPair = 4u;
// Literal wedge reconstructions at a one-micrometer interface closure. The
// vertical shift removes the source generator's 1 mm barrier gap and seats
// the expanded springers one micrometer into the ground.
constexpr std::size_t kLiteralArchMaxContacts = kArchStoneCount * 16u;
constexpr std::size_t kLiteralArch101MaxContacts = kArch101StoneCount * 16u;
constexpr std::size_t kLiteralArchMaxContactsPerPair = 8u;
constexpr double kLiteralArchEndFaceExpansion = 1e-6;
constexpr double kLiteralArchDownwardShift = 0.001001;
constexpr std::size_t kLiteralArchDefaultSteps = 600u;
// Frozen, reconstructed/non-paper crown-impact v1 contract. See
// docs/dev_tasks/fbf_exact_coulomb_friction/LITERAL_CROWN_IMPACT_V1.md.
constexpr std::size_t kLiteralCrownImpactLaunchAfterSteps = 600u;
constexpr std::size_t kLiteralCrownImpactSteps = 120u;
constexpr std::size_t kLiteralCrownImpactDefaultSteps
    = kLiteralCrownImpactLaunchAfterSteps + kLiteralCrownImpactSteps;
constexpr std::size_t kLiteralCrownImpactProjectileCount = 3u;
constexpr double kLiteralCrownImpactProjectileEdgeLength = 0.035;
constexpr double kLiteralCrownImpactProjectileMass
    = kArchDensity * kLiteralCrownImpactProjectileEdgeLength
      * kLiteralCrownImpactProjectileEdgeLength
      * kLiteralCrownImpactProjectileEdgeLength;
constexpr double kLiteralCrownImpactProjectileSpacing = 0.045;
constexpr double kLiteralCrownImpactProjectileDropHeight = 0.95;
constexpr double kLiteralCrownImpactProjectileSpeed = 3.0;
constexpr std::size_t kLiteralCrownImpactFirstCentralStone = 9u;
constexpr std::size_t kLiteralCrownImpactLastCentralStone = 15u;
constexpr double kLiteralCrownImpactMinimumCrownResponse = 0.0001;
constexpr double kLiteralCrownImpactMaximumBodyDisplacement = 0.07;
constexpr double kLiteralCrownImpactMinimumOrientationAlignment
    = 0.8660254037844386;
constexpr double kLiteralCrownImpactMaximumFarFieldDisplacement = 0.007;
constexpr double kLiteralCrownImpactSpringerTolerance = 1e-12;
constexpr std::size_t kLiteralCrownImpactFarFieldAdjacentPairCount = 16u;
constexpr int kArchMaxOuterIterations = 120000;
constexpr double kArchOuterRelaxation = 1.5;
constexpr double kArchStepSizeScale = 10.0;
// Official paper imagery shows a horizontal row of about twelve small cubes
// dropping over the crown. These numerical values remain reconstructed.
constexpr std::size_t kArchProjectileCount = 12u;
constexpr double kArchProjectileEdgeLength = 0.035;
constexpr double kArchProjectileMass = kArchDensity * kArchProjectileEdgeLength
                                       * kArchProjectileEdgeLength
                                       * kArchProjectileEdgeLength;
constexpr double kArchProjectileSpeed = 3.0;
constexpr double kArchProjectileSpacing = 0.045;
constexpr double kArchProjectileDropHeight = 0.95;
// Retained-sample cap for the exact-FBF residual-history trace path. Must
// stay above the largest arch outer-iteration budget (120000) plus one so a
// full-manifold arch history is not truncated mid-run.
constexpr int kResidualHistoryMaxSamples = 130001;
constexpr double kPi = 3.141592653589793238462643383279502884;

enum class Scenario
{
  InclineStick,
  InclineSlide,
  Backspin,
  TurntableLowSlow,
  TurntableLowFast,
  TurntableHighSlow,
  TurntableHighFast,
  AuthorTurntableLowSlow,
  AuthorTurntableLowFast,
  AuthorTurntableHighSlow,
  AuthorTurntableHighFast,
  PainleveSlide,
  PainleveTumble,
  CardHouseFourLevelReduced,
  CardHouseFourLevelSettleProjectileReduced,
  CardHouseFourLevelSettleProjectileFull,
  MasonryArch25Reduced,
  MasonryArch25ProjectileReduced,
  MasonryArch101Reduced,
  MasonryArch25FullManifold,
  MasonryArch101FullManifold,
  MasonryArch25LiteralWedge,
  MasonryArch101LiteralWedge,
  MasonryArch25LiteralWedgeCrownImpactV1,
};

enum class SolverMode
{
  BoxedLcp,
  ExactFbf,
};

enum class TraceScope
{
  TrackedBody,
  DynamicBodies,
  ResidualHistory,
  PhaseSummary,
  Performance,
};

enum class SolverContract
{
  DartBest,
  DartBestColoredBgs,
  PaperCpu,
  PaperCpuBootstrapDiagnostic,
};

bool usesDartBestParameters(SolverContract contract)
{
  return contract == SolverContract::DartBest
         || contract == SolverContract::DartBestColoredBgs;
}

bool usesPaperCpuParameters(SolverContract contract)
{
  return contract == SolverContract::PaperCpu
         || contract == SolverContract::PaperCpuBootstrapDiagnostic;
}

enum class CollisionFrontend
{
  Dart,
  Native,
};

enum class NativeManifoldSensitivitySelector
{
  Default,
  Compact,
  FourPointPlanar,
};

enum class CrossStepPolicySelector
{
  Default,
  DartCurrent,
  AuthorPolicyInspiredB3f3c5c,
};

bool gAppendLiteralCrownImpactColumns = false;
NativeManifoldSensitivitySelector gNativeManifoldSensitivitySelector
    = NativeManifoldSensitivitySelector::Default;
CrossStepPolicySelector gCrossStepPolicySelector
    = CrossStepPolicySelector::Default;

struct PhaseSummaryMetrics
{
  std::size_t cardCount = 0u;
  std::size_t projectileCount = 0u;
  bool finiteState = true;
  double minCardAxisUp = std::numeric_limits<double>::infinity();
  double minCenterHeight = std::numeric_limits<double>::infinity();
  double maxCardHorizontalTravel = 0.0;
  double maxProjectileSpeed = 0.0;
  double maxCardCenterDisplacementFromInitial = 0.0;
  double minCardOrientationAlignmentFromInitial
      = std::numeric_limits<double>::infinity();
  std::size_t projectileCardContacts = 0u;
};

struct InitialCardPose
{
  const dart::dynamics::BodyNode* body = nullptr;
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
};

using InitialCardPoses = std::vector<InitialCardPose>;

struct InitialArchPose
{
  const dart::dynamics::BodyNode* body = nullptr;
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
};

using InitialArchPoses = std::vector<InitialArchPose>;

struct ArchOutcomeMetrics
{
  std::size_t bodyCount = 0u;
  bool finiteState = true;
  double maxBodyDisplacementFromInitial = 0.0;
  double minBodyOrientationAlignmentFromInitial
      = std::numeric_limits<double>::infinity();
};

struct TrackedBodyMetrics
{
  std::string name = "none";
  double x = std::numeric_limits<double>::quiet_NaN();
  double y = std::numeric_limits<double>::quiet_NaN();
  double z = std::numeric_limits<double>::quiet_NaN();
  double vx = std::numeric_limits<double>::quiet_NaN();
  double vy = std::numeric_limits<double>::quiet_NaN();
  double vz = std::numeric_limits<double>::quiet_NaN();
  double upZ = std::numeric_limits<double>::quiet_NaN();
};

struct ExactCounterSnapshot
{
  std::size_t exactAttempts = 0u;
  std::size_t exactSolves = 0u;
  std::size_t warmStarts = 0u;
  std::size_t exactFailures = 0u;
  std::size_t boxedFallbacks = 0u;
  std::size_t maxIterationsAccepted = 0u;
  std::size_t warmStartStepSizeCaps = 0u;
  std::size_t unconvergedCacheSkips = 0u;
  std::size_t fbfIterations = 0u;
  std::size_t contactRowDelassusProducts = 0u;
  std::size_t parallelContactRowDelassusProducts = 0u;
};

struct CollisionMetrics
{
  std::size_t uniqueBodyPairs = 0u;
  double penetrationDepthMin = std::numeric_limits<double>::quiet_NaN();
  double penetrationDepthMedian = std::numeric_limits<double>::quiet_NaN();
  double penetrationDepthP95 = std::numeric_limits<double>::quiet_NaN();
  double penetrationDepthMax = std::numeric_limits<double>::quiet_NaN();
};

struct ContactPairMultiplicityMetrics
{
  std::string pairLabels = "none";
  std::string multiplicities = "none";
};

struct LiteralCrownImpactPoseMetrics
{
  bool finiteState = false;
  double maxBodyDisplacement = std::numeric_limits<double>::quiet_NaN();
  double minOrientationAlignment = std::numeric_limits<double>::quiet_NaN();
  double maxCrownDisplacement = std::numeric_limits<double>::quiet_NaN();
  double maxFarFieldDisplacement = std::numeric_limits<double>::quiet_NaN();
  double maxSpringerDisplacement = std::numeric_limits<double>::quiet_NaN();
  double minSpringerOrientationAlignment
      = std::numeric_limits<double>::quiet_NaN();
};

struct LiteralCrownImpactTraceState
{
  InitialArchPoses preImpactArchPoses;
  bool preImpactSnapshotCaptured = false;
  std::size_t exactSolvesAtImpactLaunch = 0u;
  int preImpactStandingGate = -1;
  bool finiteStateToDate = true;
  bool prefixProjectileFreeToDate = true;
  std::size_t projectileCount = 0u;
  std::size_t stepProjectileArchContacts = 0u;
  std::size_t projectileArchContactsToDate = 0u;
  std::size_t stepProjectileGroundContacts = 0u;
  std::size_t projectileGroundContactsToDate = 0u;
  std::size_t firstProjectileArchContactStep = 0u;
  std::size_t firstProjectileGroundContactStep = 0u;
  double maximumCrownDisplacementToDate = 0.0;
  std::size_t farFieldAdjacentPairs = 0u;
  LiteralCrownImpactPoseMetrics poseMetrics;
};

const char* scenarioName(Scenario scenario)
{
  switch (scenario) {
    case Scenario::InclineStick:
      return "incline_mu_0_5";
    case Scenario::InclineSlide:
      return "incline_mu_0_4";
    case Scenario::Backspin:
      return "backspin";
    case Scenario::TurntableLowSlow:
      return "turntable_mu_0_2_omega_2";
    case Scenario::TurntableLowFast:
      return "turntable_mu_0_2_omega_5";
    case Scenario::TurntableHighSlow:
      return "turntable_mu_0_5_omega_2";
    case Scenario::TurntableHighFast:
      return "turntable_mu_0_5_omega_5";
    case Scenario::AuthorTurntableLowSlow:
      return fbf_author_turntable::kScenarios[0].traceScenario;
    case Scenario::AuthorTurntableLowFast:
      return fbf_author_turntable::kScenarios[1].traceScenario;
    case Scenario::AuthorTurntableHighSlow:
      return fbf_author_turntable::kScenarios[2].traceScenario;
    case Scenario::AuthorTurntableHighFast:
      return fbf_author_turntable::kScenarios[3].traceScenario;
    case Scenario::PainleveSlide:
      return "painleve_mu_0_5";
    case Scenario::PainleveTumble:
      return "painleve_mu_0_55";
    case Scenario::CardHouseFourLevelReduced:
      return "card_house_26_reduced_contact";
    case Scenario::CardHouseFourLevelSettleProjectileReduced:
      return "card_house_26_settle_projectile_reduced_contact";
    case Scenario::CardHouseFourLevelSettleProjectileFull:
      return "card_house_26_settle_projectile_full";
    case Scenario::MasonryArch25Reduced:
      return "masonry_arch_25_reduced_contact";
    case Scenario::MasonryArch25ProjectileReduced:
      return "masonry_arch_25_projectile_reduced_contact";
    case Scenario::MasonryArch101Reduced:
      return "masonry_arch_101_reduced_contact";
    case Scenario::MasonryArch25FullManifold:
      return "masonry_arch_25_full_manifold";
    case Scenario::MasonryArch101FullManifold:
      return "masonry_arch_101_full_manifold";
    case Scenario::MasonryArch25LiteralWedge:
      return "masonry_arch_25_literal_wedge";
    case Scenario::MasonryArch101LiteralWedge:
      return "masonry_arch_101_literal_wedge";
    case Scenario::MasonryArch25LiteralWedgeCrownImpactV1:
      return "masonry_arch_25_literal_wedge_crown_impact_v1";
  }

  return "unknown";
}

const char* solverName(SolverMode mode)
{
  switch (mode) {
    case SolverMode::BoxedLcp:
      return "boxed_lcp";
    case SolverMode::ExactFbf:
      return "exact_fbf";
  }

  return "unknown";
}

const char* solverContractName(
    SolverContract contract, std::size_t requestedThreads)
{
  switch (contract) {
    case SolverContract::DartBest:
      return requestedThreads > 1u
                 ? "dart_best_threaded_world_exact_contact_row_parallel_capable"
                 : "dart_best";
    case SolverContract::DartBestColoredBgs:
      return requestedThreads > 1u
                 ? "dart_best_nonpaper_colored_inner_bgs_threaded_world"
                 : "dart_best_nonpaper_colored_inner_bgs";
    case SolverContract::PaperCpu:
      return requestedThreads > 1u
                 ? "paper_cpu_parameters_threaded_world_serial_fbf"
                 : "paper_cpu";
    case SolverContract::PaperCpuBootstrapDiagnostic:
      return "paper_cpu_bootstrap_diagnostic";
  }

  return "unknown";
}

const char* collisionFrontendName(CollisionFrontend frontend)
{
  switch (frontend) {
    case CollisionFrontend::Dart:
      return "dart";
    case CollisionFrontend::Native:
      return "native";
  }

  return "unknown";
}

bool nativeManifoldSensitivityEnabled()
{
  return gNativeManifoldSensitivitySelector
         != NativeManifoldSensitivitySelector::Default;
}

bool crossStepPolicyEvidenceEnabled()
{
  return gCrossStepPolicySelector != CrossStepPolicySelector::Default;
}

const char* nativeManifoldSensitivitySelectorName(
    NativeManifoldSensitivitySelector selector)
{
  switch (selector) {
    case NativeManifoldSensitivitySelector::Default:
      return "default";
    case NativeManifoldSensitivitySelector::Compact:
      return "compact";
    case NativeManifoldSensitivitySelector::FourPointPlanar:
      return "four_point_planar";
  }

  return "unknown";
}

const char* crossStepPolicySelectorName(CrossStepPolicySelector selector)
{
  switch (selector) {
    case CrossStepPolicySelector::Default:
      return "default";
    case CrossStepPolicySelector::DartCurrent:
      return "dart_current";
    case CrossStepPolicySelector::AuthorPolicyInspiredB3f3c5c:
      return "author_policy_inspired_b3f3c5c";
  }

  return "unknown";
}

const char* nativeContactManifoldModeName(
    dart::collision::NativeCollisionDetector::ContactManifoldMode mode)
{
  using Mode = dart::collision::NativeCollisionDetector::ContactManifoldMode;
  switch (mode) {
    case Mode::Compact:
      return "compact";
    case Mode::FourPointPlanar:
      return "four_point_planar";
  }

  return "unknown";
}

const char* localSolverName(
    dart::constraint::ExactCoulombFbfLocalBlockSolver solver)
{
  using LocalSolver = dart::constraint::ExactCoulombFbfLocalBlockSolver;
  switch (solver) {
    case LocalSolver::ExactMetricProjection:
      return "exact_metric";
    case LocalSolver::InverseEuclideanProjection:
      return "inverse_euclidean";
    case LocalSolver::ProjectedGradient:
      return "projected_gradient";
  }

  return "unknown";
}

const char* warmStartMatchModeName(
    dart::constraint::ExactCoulombFbfWarmStartMatchMode mode)
{
  using Mode = dart::constraint::ExactCoulombFbfWarmStartMatchMode;
  switch (mode) {
    case Mode::EitherBodyLocalFeature:
      return "either_body_local_feature";
    case Mode::OrderedBodyBLocalFeature:
      return "ordered_body_b_local_feature";
  }

  return "unknown";
}

const char* rowOperatorRequestName(
    const dart::constraint::ExactCoulombFbfConstraintSolverOptions& options)
{
  if (!options.useContactRowDelassusOperator)
    return "dense_delassus";
  if (options.assembleDenseContactRowSnapshot)
    return "contact_row_with_dense_snapshot";
  return "contact_row_no_dense_snapshot";
}

const char* initialGammaContractName(
    const dart::constraint::ExactCoulombFbfConstraintSolverOptions& options)
{
  if (std::isnan(options.initialStepSize))
    return "automatic_safe_bound";
  if (options.capInitialStepSizeAtSafeBound)
    return "explicit_capped_safe_bound";
  return "explicit_uncapped";
}

const char* exactStatusName(
    dart::constraint::ExactCoulombFbfConstraintSolverStatus status)
{
  switch (status) {
    case dart::constraint::ExactCoulombFbfConstraintSolverStatus::NotRun:
      return "not_run";
    case dart::constraint::ExactCoulombFbfConstraintSolverStatus::Success:
      return "success";
    case dart::constraint::ExactCoulombFbfConstraintSolverStatus::
        MaxIterationsAccepted:
      return "max_iterations_accepted";
    case dart::constraint::ExactCoulombFbfConstraintSolverStatus::
        InvalidOptions:
      return "invalid_options";
    case dart::constraint::ExactCoulombFbfConstraintSolverStatus::
        UnsupportedProblem:
      return "unsupported_problem";
    case dart::constraint::ExactCoulombFbfConstraintSolverStatus::FbfFailed:
      return "fbf_failed";
    case dart::constraint::ExactCoulombFbfConstraintSolverStatus::
        BoxedLcpFallback:
      return "boxed_lcp_fallback";
  }

  return "unknown";
}

const char* fbfStatusName(dart::math::detail::ExactCoulombFbfStatus status)
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
  }

  return "unknown";
}

bool parseScenario(const char* value, Scenario& scenario)
{
  if (value == nullptr || std::string(value) == "backspin") {
    scenario = Scenario::Backspin;
    return true;
  }

  const std::string name(value);
  if (name == "incline_mu_0_5") {
    scenario = Scenario::InclineStick;
  } else if (name == "incline_mu_0_4") {
    scenario = Scenario::InclineSlide;
  } else if (name == "turntable_mu_0_2_omega_2") {
    scenario = Scenario::TurntableLowSlow;
  } else if (name == "turntable_mu_0_2_omega_5") {
    scenario = Scenario::TurntableLowFast;
  } else if (name == "turntable_mu_0_5_omega_2") {
    scenario = Scenario::TurntableHighSlow;
  } else if (name == "turntable_mu_0_5_omega_5") {
    scenario = Scenario::TurntableHighFast;
  } else if (name == fbf_author_turntable::kScenarios[0].traceScenario) {
    scenario = Scenario::AuthorTurntableLowSlow;
  } else if (name == fbf_author_turntable::kScenarios[1].traceScenario) {
    scenario = Scenario::AuthorTurntableLowFast;
  } else if (name == fbf_author_turntable::kScenarios[2].traceScenario) {
    scenario = Scenario::AuthorTurntableHighSlow;
  } else if (name == fbf_author_turntable::kScenarios[3].traceScenario) {
    scenario = Scenario::AuthorTurntableHighFast;
  } else if (name == "painleve_mu_0_5") {
    scenario = Scenario::PainleveSlide;
  } else if (name == "painleve_mu_0_55") {
    scenario = Scenario::PainleveTumble;
  } else if (name == "card_house_26_reduced_contact") {
    scenario = Scenario::CardHouseFourLevelReduced;
  } else if (name == "card_house_26_settle_projectile_reduced_contact") {
    scenario = Scenario::CardHouseFourLevelSettleProjectileReduced;
  } else if (name == "card_house_26_settle_projectile_full") {
    scenario = Scenario::CardHouseFourLevelSettleProjectileFull;
  } else if (name == "masonry_arch_25_reduced_contact") {
    scenario = Scenario::MasonryArch25Reduced;
  } else if (name == "masonry_arch_25_projectile_reduced_contact") {
    scenario = Scenario::MasonryArch25ProjectileReduced;
  } else if (name == "masonry_arch_101_reduced_contact") {
    scenario = Scenario::MasonryArch101Reduced;
  } else if (name == "masonry_arch_25_full_manifold") {
    scenario = Scenario::MasonryArch25FullManifold;
  } else if (name == "masonry_arch_101_full_manifold") {
    scenario = Scenario::MasonryArch101FullManifold;
  } else if (name == "masonry_arch_25_literal_wedge") {
    scenario = Scenario::MasonryArch25LiteralWedge;
  } else if (name == "masonry_arch_101_literal_wedge") {
    scenario = Scenario::MasonryArch101LiteralWedge;
  } else if (name == "masonry_arch_25_literal_wedge_crown_impact_v1") {
    scenario = Scenario::MasonryArch25LiteralWedgeCrownImpactV1;
  } else {
    return false;
  }

  return true;
}

bool parseSolver(const char* value, SolverMode& mode)
{
  if (value == nullptr || std::string(value) == "exact_fbf") {
    mode = SolverMode::ExactFbf;
    return true;
  }

  const std::string name(value);
  if (name == "boxed_lcp") {
    mode = SolverMode::BoxedLcp;
    return true;
  }

  return false;
}

bool parseSolverContract(const char* value, SolverContract& contract)
{
  if (value == nullptr || std::string(value) == "dart_best") {
    contract = SolverContract::DartBest;
    return true;
  }

  if (std::string(value) == "dart_best_colored_bgs") {
    contract = SolverContract::DartBestColoredBgs;
    return true;
  }

  if (std::string(value) == "paper_cpu") {
    contract = SolverContract::PaperCpu;
    return true;
  }
  if (std::string(value) == "paper_cpu_bootstrap_diagnostic") {
    contract = SolverContract::PaperCpuBootstrapDiagnostic;
    return true;
  }

  return false;
}

bool parseCollisionFrontend(const char* value, CollisionFrontend& frontend)
{
  if (value == nullptr || std::string(value) == "dart") {
    frontend = CollisionFrontend::Dart;
    return true;
  }

  if (std::string(value) == "native") {
    frontend = CollisionFrontend::Native;
    return true;
  }

  return false;
}

bool parseNativeManifoldSensitivitySelector(
    const char* value, NativeManifoldSensitivitySelector& selector)
{
  if (value == nullptr || std::string(value) == "default") {
    selector = NativeManifoldSensitivitySelector::Default;
    return true;
  }

  const std::string name(value);
  if (name == "compact") {
    selector = NativeManifoldSensitivitySelector::Compact;
    return true;
  }
  if (name == "four_point_planar") {
    selector = NativeManifoldSensitivitySelector::FourPointPlanar;
    return true;
  }

  return false;
}

bool parseCrossStepPolicySelector(
    const char* value, CrossStepPolicySelector& selector)
{
  if (value == nullptr || std::string(value) == "default") {
    selector = CrossStepPolicySelector::Default;
    return true;
  }

  const std::string name(value);
  if (name == "dart_current") {
    selector = CrossStepPolicySelector::DartCurrent;
    return true;
  }
  if (name == "author_policy_inspired_b3f3c5c") {
    selector = CrossStepPolicySelector::AuthorPolicyInspiredB3f3c5c;
    return true;
  }

  return false;
}

bool parseLocalSolverOverride(const char* value)
{
  using LocalSolver = dart::constraint::ExactCoulombFbfLocalBlockSolver;
  if (value == nullptr || std::string(value) == "default") {
    gLocalSolverOverrideSet = false;
    return true;
  }

  const std::string name(value);
  gLocalSolverOverrideSet = true;
  if (name == "exact_metric") {
    gLocalSolverOverride = LocalSolver::ExactMetricProjection;
  } else if (name == "inverse_euclidean") {
    gLocalSolverOverride = LocalSolver::InverseEuclideanProjection;
  } else if (name == "projected_gradient") {
    gLocalSolverOverride = LocalSolver::ProjectedGradient;
  } else {
    gLocalSolverOverrideSet = false;
    return false;
  }
  return true;
}

bool parseTraceScope(const char* value, TraceScope& scope)
{
  if (value == nullptr || std::string(value) == "tracked") {
    scope = TraceScope::TrackedBody;
    return true;
  }

  const std::string name(value);
  if (name == "dynamic_bodies" || name == "full_scene") {
    scope = TraceScope::DynamicBodies;
    return true;
  }
  if (name == "residual_history") {
    scope = TraceScope::ResidualHistory;
    return true;
  }
  if (name == "phase_summary") {
    scope = TraceScope::PhaseSummary;
    return true;
  }
  if (name == "performance") {
    scope = TraceScope::Performance;
    return true;
  }

  return false;
}

bool parseSizeArg(const char* value, std::size_t fallback, std::size_t& output)
{
  if (value == nullptr) {
    output = fallback;
    return true;
  }

  for (const char* digit = value; *digit != '\0'; ++digit) {
    if (*digit < '0' || *digit > '9')
      return false;
  }

  errno = 0;
  char* end = nullptr;
  const unsigned long long parsed = std::strtoull(value, &end, 10);
  if (value[0] == '\0' || value[0] == '-' || end == value || *end != '\0'
      || errno == ERANGE || parsed > std::numeric_limits<std::size_t>::max()) {
    return false;
  }

  output = static_cast<std::size_t>(parsed);
  return true;
}

bool parsePositiveDoubleArg(const char* value, double fallback, double& output)
{
  if (value == nullptr) {
    output = fallback;
    return true;
  }

  if (std::string(value) == "nan") {
    output = std::numeric_limits<double>::quiet_NaN();
    return true;
  }

  errno = 0;
  char* end = nullptr;
  const double parsed = std::strtod(value, &end);
  if (value[0] == '\0' || end == value || *end != '\0' || errno == ERANGE)
    return false;

  if (!std::isfinite(parsed) || parsed <= 0.0)
    return false;

  output = parsed;
  return true;
}

double scenarioFriction(Scenario scenario)
{
  switch (scenario) {
    case Scenario::InclineStick:
      return 0.5;
    case Scenario::InclineSlide:
      return 0.4;
    case Scenario::TurntableLowSlow:
    case Scenario::TurntableLowFast:
      return 0.2;
    case Scenario::AuthorTurntableLowSlow:
      return fbf_author_turntable::kScenarios[0].friction;
    case Scenario::AuthorTurntableLowFast:
      return fbf_author_turntable::kScenarios[1].friction;
    case Scenario::TurntableHighSlow:
    case Scenario::TurntableHighFast:
    case Scenario::Backspin:
    case Scenario::PainleveSlide:
      return 0.5;
    case Scenario::AuthorTurntableHighSlow:
      return fbf_author_turntable::kScenarios[2].friction;
    case Scenario::AuthorTurntableHighFast:
      return fbf_author_turntable::kScenarios[3].friction;
    case Scenario::PainleveTumble:
      return 0.55;
    case Scenario::CardHouseFourLevelReduced:
    case Scenario::CardHouseFourLevelSettleProjectileReduced:
    case Scenario::CardHouseFourLevelSettleProjectileFull:
      return kCardHouseFriction;
    case Scenario::MasonryArch25Reduced:
    case Scenario::MasonryArch25ProjectileReduced:
    case Scenario::MasonryArch101Reduced:
    case Scenario::MasonryArch25FullManifold:
    case Scenario::MasonryArch101FullManifold:
    case Scenario::MasonryArch25LiteralWedge:
    case Scenario::MasonryArch101LiteralWedge:
    case Scenario::MasonryArch25LiteralWedgeCrownImpactV1:
      return kArchFriction;
  }

  return 0.5;
}

double turntableAngularVelocity(Scenario scenario)
{
  switch (scenario) {
    case Scenario::TurntableLowSlow:
    case Scenario::TurntableHighSlow:
      return 2.0;
    case Scenario::AuthorTurntableLowSlow:
      return fbf_author_turntable::kScenarios[0].angularVelocity;
    case Scenario::AuthorTurntableHighSlow:
      return fbf_author_turntable::kScenarios[2].angularVelocity;
    case Scenario::TurntableLowFast:
    case Scenario::TurntableHighFast:
      return 5.0;
    case Scenario::AuthorTurntableLowFast:
      return fbf_author_turntable::kScenarios[1].angularVelocity;
    case Scenario::AuthorTurntableHighFast:
      return fbf_author_turntable::kScenarios[3].angularVelocity;
    default:
      return 0.0;
  }
}

std::size_t defaultScenarioSteps(Scenario scenario)
{
  switch (scenario) {
    case Scenario::InclineStick:
    case Scenario::InclineSlide:
      return static_cast<std::size_t>(std::round(2.0 / kDt));
    case Scenario::Backspin:
    case Scenario::TurntableLowSlow:
    case Scenario::TurntableLowFast:
    case Scenario::TurntableHighSlow:
    case Scenario::TurntableHighFast:
      return static_cast<std::size_t>(std::round(4.0 / kDt));
    case Scenario::AuthorTurntableLowSlow:
    case Scenario::AuthorTurntableLowFast:
    case Scenario::AuthorTurntableHighSlow:
    case Scenario::AuthorTurntableHighFast:
      return static_cast<std::size_t>(
          std::round(fbf_author_turntable::kDuration / kDt));
    case Scenario::PainleveSlide:
    case Scenario::PainleveTumble:
      return static_cast<std::size_t>(std::round(2.5 / kDt));
    case Scenario::CardHouseFourLevelReduced:
    case Scenario::MasonryArch25Reduced:
    case Scenario::MasonryArch25ProjectileReduced:
    case Scenario::MasonryArch101Reduced:
    case Scenario::MasonryArch25FullManifold:
    case Scenario::MasonryArch101FullManifold:
      return 1u;
    case Scenario::MasonryArch25LiteralWedge:
    case Scenario::MasonryArch101LiteralWedge:
      return kLiteralArchDefaultSteps;
    case Scenario::MasonryArch25LiteralWedgeCrownImpactV1:
      return kLiteralCrownImpactDefaultSteps;
    case Scenario::CardHouseFourLevelSettleProjectileReduced:
      return kCardHouseSettleProjectileDefaultSteps;
    case Scenario::CardHouseFourLevelSettleProjectileFull:
      return kCardHouseSettleProjectileFullDefaultSteps;
  }

  return 0u;
}

bool isTurntableScenario(Scenario scenario)
{
  return scenario == Scenario::TurntableLowSlow
         || scenario == Scenario::TurntableLowFast
         || scenario == Scenario::TurntableHighSlow
         || scenario == Scenario::TurntableHighFast
         || scenario == Scenario::AuthorTurntableLowSlow
         || scenario == Scenario::AuthorTurntableLowFast
         || scenario == Scenario::AuthorTurntableHighSlow
         || scenario == Scenario::AuthorTurntableHighFast;
}

bool isAuthorTurntableScenario(Scenario scenario)
{
  return scenario == Scenario::AuthorTurntableLowSlow
         || scenario == Scenario::AuthorTurntableLowFast
         || scenario == Scenario::AuthorTurntableHighSlow
         || scenario == Scenario::AuthorTurntableHighFast;
}

bool isArchScenario(Scenario scenario)
{
  return scenario == Scenario::MasonryArch25Reduced
         || scenario == Scenario::MasonryArch25ProjectileReduced
         || scenario == Scenario::MasonryArch101Reduced
         || scenario == Scenario::MasonryArch25FullManifold
         || scenario == Scenario::MasonryArch101FullManifold
         || scenario == Scenario::MasonryArch25LiteralWedge
         || scenario == Scenario::MasonryArch101LiteralWedge
         || scenario == Scenario::MasonryArch25LiteralWedgeCrownImpactV1;
}

bool isLiteralWedgeScenario(Scenario scenario)
{
  return scenario == Scenario::MasonryArch25LiteralWedge
         || scenario == Scenario::MasonryArch101LiteralWedge
         || scenario == Scenario::MasonryArch25LiteralWedgeCrownImpactV1;
}

const char* precisionContractLabel(SolverContract contract)
{
  return usesPaperCpuParameters(contract) ? "float64_vs_paper_float32"
                                          : "float64";
}

const char* sceneContractLabel(
    Scenario scenario, CollisionFrontend collisionFrontend)
{
  switch (scenario) {
    case Scenario::InclineStick:
    case Scenario::InclineSlide:
    case Scenario::Backspin:
      return collisionFrontend == CollisionFrontend::Dart
                 ? "paper_parameters_with_dart_contact_frontend"
                 : "paper_parameters_with_native_contact_frontend";
    case Scenario::TurntableLowSlow:
    case Scenario::TurntableLowFast:
    case Scenario::TurntableHighSlow:
    case Scenario::TurntableHighFast:
      return "reconstructed_scene_author_assets_unavailable";
    case Scenario::AuthorTurntableLowSlow:
    case Scenario::AuthorTurntableLowFast:
    case Scenario::AuthorTurntableHighSlow:
    case Scenario::AuthorTurntableHighFast:
      return "author_code_b3f3c5c_parameters_dart_float64_native_collision";
    case Scenario::PainleveSlide:
    case Scenario::PainleveTumble:
      return "proxy_scene_author_parameters_unavailable";
    case Scenario::CardHouseFourLevelReduced:
    case Scenario::CardHouseFourLevelSettleProjectileReduced:
    case Scenario::CardHouseFourLevelSettleProjectileFull:
      return "reconstructed_cards_density_1000_from_newton_default_"
             "cube_drop_author_overrides_unavailable";
    case Scenario::MasonryArch25Reduced:
    case Scenario::MasonryArch25ProjectileReduced:
    case Scenario::MasonryArch101Reduced:
    case Scenario::MasonryArch25FullManifold:
    case Scenario::MasonryArch101FullManifold:
      return "source_derived_rigid_ipc_geometry_density_1000_mu_0_8_"
             "pinned_springers_cube_drop_author_overrides_unavailable";
    case Scenario::MasonryArch25LiteralWedge:
    case Scenario::MasonryArch101LiteralWedge:
      return "reconstructed_literal_wedge_arch_nonpaper_native_collision_"
             "frontend";
    case Scenario::MasonryArch25LiteralWedgeCrownImpactV1:
      return "reconstructed_literal_wedge_crown_impact_v1_nonpaper_native_"
             "collision_frontend";
  }

  return "unknown";
}

const char* bodyNameForScenario(Scenario scenario)
{
  switch (scenario) {
    case Scenario::InclineStick:
    case Scenario::InclineSlide:
      return "incline_cube";
    case Scenario::Backspin:
      return "backspin_sphere";
    case Scenario::TurntableLowSlow:
    case Scenario::TurntableLowFast:
    case Scenario::TurntableHighSlow:
    case Scenario::TurntableHighFast:
    case Scenario::AuthorTurntableLowSlow:
    case Scenario::AuthorTurntableLowFast:
    case Scenario::AuthorTurntableHighSlow:
    case Scenario::AuthorTurntableHighFast:
      return "turntable_rider";
    case Scenario::PainleveSlide:
    case Scenario::PainleveTumble:
      return "painleve_box";
    case Scenario::CardHouseFourLevelReduced:
    case Scenario::CardHouseFourLevelSettleProjectileReduced:
    case Scenario::CardHouseFourLevelSettleProjectileFull:
      return "card_house_l3_f0_left";
    case Scenario::MasonryArch25Reduced:
    case Scenario::MasonryArch25FullManifold:
    case Scenario::MasonryArch25LiteralWedge:
    case Scenario::MasonryArch25LiteralWedgeCrownImpactV1:
      return "masonry_arch_stone_12";
    case Scenario::MasonryArch25ProjectileReduced:
      return "masonry_arch_projectile_0";
    case Scenario::MasonryArch101Reduced:
    case Scenario::MasonryArch101FullManifold:
    case Scenario::MasonryArch101LiteralWedge:
      return "masonry_arch_stone_50";
  }

  return "";
}

dart::constraint::ExactCoulombFbfConstraintSolverOptions makeFbfOptions(
    double initialStepSize, TraceScope traceScope)
{
  dart::constraint::ExactCoulombFbfConstraintSolverOptions options;
  options.maxOuterIterations = 500;
  options.tolerance = 1e-6;
  options.innerMaxSweeps = 120;
  options.innerLocalIterations = 32;
  options.initialStepSize = initialStepSize;
  options.stepSizeScale = kSmallFixtureStepSizeScale;
  if (traceScope == TraceScope::ResidualHistory) {
    options.maxResidualHistorySamples = kResidualHistoryMaxSamples;
    options.maxResidualHistoryRecords = 256;
  }
  if (traceScope == TraceScope::Performance) {
    // Keep one-thread and multi-thread performance rows on the same matrix-free
    // contact-row route. Dense snapshots disable the exact row-product
    // parallel path and would confound an apples-to-apples scaling matrix.
    options.assembleDenseContactRowSnapshot = false;
  }
  return options;
}

dart::constraint::ExactCoulombFbfConstraintSolverOptions makeFbfOptions(
    double initialStepSize,
    Scenario scenario,
    TraceScope traceScope,
    SolverContract contract)
{
  auto options = makeFbfOptions(initialStepSize, traceScope);
  if (isAuthorTurntableScenario(scenario)
      && contract == SolverContract::DartBest) {
    const auto authorOptions = fbf_author_turntable::dartBestSolverOptions();
    options.maxOuterIterations = authorOptions.maxOuterIterations;
    options.tolerance = authorOptions.tolerance;
    options.innerMaxSweeps = authorOptions.innerMaxSweeps;
    options.innerLocalIterations = authorOptions.innerLocalIterations;
    options.stepSizeScale = authorOptions.stepSizeScale;
  }
  if (usesPaperCpuParameters(contract)) {
    // Appendix B's CPU contract: fixed outer/inner budgets, conservative
    // adaptive step, no continuation or fallback paths. The CSV labels the
    // remaining precision, scene, and Baumgarte differences explicitly.
    options.maxOuterIterations = 200;
    options.acceptOuterMaxIterations = true;
    options.innerMaxSweeps = isArchScenario(scenario) ? 30 : 10;
    options.runFixedInnerSweeps = true;
    // Use the rigorous DART local cone-QP solve as the correctness default.
    // The paper describes 3x3 block solves but does not publish the local
    // kernel formula, so this choice must not be labeled an exact kernel match.
    options.innerLocalSolver = dart::constraint::
        ExactCoulombFbfLocalBlockSolver::ExactMetricProjection;
    // One exact 3D block update is performed per contact visit.
    options.innerLocalIterations = 1;
    options.stepSizeScale = 1.0;
    options.outerRelaxation = 1.0;
    options.enableWarmStart = true;
    options.enableProjectedGradientRetry = false;
    options.enableDenseResidualPolish = false;
    options.fallbackToBoxedLcp = false;
    // The paper publishes only the previous-step solution warm start. Keep a
    // newly appearing contact group at the cone-feasible zero start instead
    // of introducing DART's local/global diagonal seed heuristics.
    options.seedNormalImpulseFromDiagonal = false;
    options.useMatrixFreeDelassusSeed = false;
    options.assembleDenseContactRowSnapshot = false;
    options.enableStepSizePersistence = true;
  } else if (isArchScenario(scenario)) {
    options.maxOuterIterations = kArchMaxOuterIterations;
    options.stepSizeScale = kArchStepSizeScale;
    options.outerRelaxation = kArchOuterRelaxation;
    options.enableWarmStart = false;
  } else if (
      scenario == Scenario::CardHouseFourLevelReduced
      || scenario == Scenario::CardHouseFourLevelSettleProjectileReduced
      || scenario == Scenario::CardHouseFourLevelSettleProjectileFull) {
    // The long-run full sequence relies on cross-step manifold warm starts
    // and a larger outer budget (the settle phase densifies the manifold well
    // past the one-step problem); the bounded reduced scaffolds keep cold
    // starts and the promoted 30000 budget for comparability with the fixture
    // and benchmark rows.
    const bool fullSequence
        = scenario == Scenario::CardHouseFourLevelSettleProjectileFull;
    options.maxOuterIterations = fullSequence ? 120000 : 30000;
    options.projectedGradientMaxIterations = 200;
    options.stepSizeScale = kCardHouseStepSizeScale;
    options.outerRelaxation = kCardHouseOuterRelaxation;
    options.enableWarmStart = fullSequence;
  }
  if (contract == SolverContract::DartBestColoredBgs) {
    // This explicitly non-paper performance contract isolates the colored
    // exact inner schedule. Alternate projected-gradient, dense-polish, and
    // boxed-LCP paths would make colored1/coloredN timing attribution
    // ambiguous. A capped finite iterate may advance, but the evidence runner
    // still rejects any residual above tolerance.
    options.acceptOuterMaxIterations = true;
    options.enableProjectedGradientRetry = false;
    options.enableDenseResidualPolish = false;
    options.fallbackToBoxedLcp = false;
    options.seedNormalImpulseFromDiagonal = true;
    options.useMatrixFreeDelassusSeed = true;
    options.assembleDenseContactRowSnapshot = false;
    if (isLiteralWedgeScenario(scenario)) {
      // Tuned exact trajectory contract for the separately labeled,
      // non-paper literal wedge reconstruction. It preserves exact local
      // cone solves and cross-step impulse warm starts while resetting the
      // adaptive outer step size every frame.
      options.maxOuterIterations = 5000;
      options.innerMaxSweeps = 30;
      options.runFixedInnerSweeps = true;
      options.innerLocalSolver = dart::constraint::
          ExactCoulombFbfLocalBlockSolver::ExactMetricProjection;
      options.innerLocalIterations = 1;
      options.stepSizeScale = 35.0;
      options.enableAdaptiveStepSize = true;
      options.outerRelaxation = 1.1;
      options.enableWarmStart = true;
      options.enableProjectedGradientRetry = false;
      options.enableDenseResidualPolish = false;
      options.fallbackToBoxedLcp = false;
      options.seedNormalImpulseFromDiagonal = false;
      options.useMatrixFreeDelassusSeed = false;
      options.assembleDenseContactRowSnapshot = false;
      options.enableStepSizePersistence = false;
      if (scenario == Scenario::MasonryArch101LiteralWedge) {
        // Protocol v1 fails closed: reaching the 5,000-outer cap is an exact
        // failure, so the trace records that row and stops immediately.
        options.acceptOuterMaxIterations = false;
      }
    }
  }
  if (gWarmStartOverride == 0) {
    options.enableWarmStart = false;
  } else if (gWarmStartOverride == 1) {
    options.enableWarmStart = true;
  }
  if (usesDartBestParameters(contract) && gLocalSolverOverrideSet)
    options.innerLocalSolver = gLocalSolverOverride;
  return options;
}

bool equalPolicyValue(double lhs, double rhs)
{
  return lhs == rhs || (std::isnan(lhs) && std::isnan(rhs));
}

bool matchesCrossStepPolicyOptions(
    const dart::constraint::ExactCoulombFbfCrossStepPolicyOptions& actual,
    const dart::constraint::ExactCoulombFbfCrossStepPolicyOptions& expected)
{
  return actual.warmStartMatchMode == expected.warmStartMatchMode
         && equalPolicyValue(
             actual.warmStartNormalCosine, expected.warmStartNormalCosine)
         && actual.useStrictWarmStartMatchDistance
                == expected.useStrictWarmStartMatchDistance
         && actual.warmStartMaxAge == expected.warmStartMaxAge
         && equalPolicyValue(
             actual.persistentStepSizeSafeBoundScale,
             expected.persistentStepSizeSafeBoundScale)
         && equalPolicyValue(actual.minimumStepSize, expected.minimumStepSize)
         && equalPolicyValue(actual.maximumStepSize, expected.maximumStepSize)
         && equalPolicyValue(
             actual.warmStartResidualThreshold,
             expected.warmStartResidualThreshold)
         && equalPolicyValue(
             actual.warmStartStepSizeCap, expected.warmStartStepSizeCap)
         && actual.persistUncappedStepSizeOnWarmStartCap
                == expected.persistUncappedStepSizeOnWarmStartCap
         && actual.requireResidualImprovementForUnconvergedCacheSave
                == expected.requireResidualImprovementForUnconvergedCacheSave;
}

bool matchesDartCurrentCrossStepPolicy(
    const dart::constraint::ExactCoulombFbfConstraintSolverOptions& options,
    const dart::constraint::ExactCoulombFbfCrossStepPolicyOptions& policy)
{
  const dart::constraint::ExactCoulombFbfCrossStepPolicyOptions expected;
  return options.enableWarmStart && options.enableStepSizePersistence
         && equalPolicyValue(options.warmStartMatchDistance, 0.025)
         && equalPolicyValue(options.stepSizeRecoveryGrowthFactor, 1.05)
         && equalPolicyValue(options.stepSizeScale, 1.0)
         && equalPolicyValue(options.couplingVariationTolerance, 0.9)
         && equalPolicyValue(options.shrinkFactor, 0.7)
         && options.maxStepShrinkIterations == 20
         && matchesCrossStepPolicyOptions(policy, expected);
}

dart::constraint::ExactCoulombFbfCrossStepPolicyOptions
authorPolicyInspiredCrossStepOptions()
{
  using MatchMode = dart::constraint::ExactCoulombFbfWarmStartMatchMode;
  dart::constraint::ExactCoulombFbfCrossStepPolicyOptions policy;
  policy.warmStartMatchMode = MatchMode::OrderedBodyBLocalFeature;
  policy.warmStartNormalCosine = 0.9;
  policy.useStrictWarmStartMatchDistance = true;
  policy.warmStartMaxAge = 3;
  policy.persistentStepSizeSafeBoundScale = 10.0;
  policy.minimumStepSize = 1e-6;
  policy.maximumStepSize = 1e6;
  policy.warmStartResidualThreshold = 1e-4;
  policy.warmStartStepSizeCap = 1e4;
  policy.persistUncappedStepSizeOnWarmStartCap = true;
  policy.requireResidualImprovementForUnconvergedCacheSave = true;
  return policy;
}

bool matchesAuthorPolicyInspiredCrossStepPolicy(
    const dart::constraint::ExactCoulombFbfConstraintSolverOptions& options,
    const dart::constraint::ExactCoulombFbfCrossStepPolicyOptions& policy)
{
  return options.enableWarmStart && options.enableStepSizePersistence
         && equalPolicyValue(options.warmStartMatchDistance, 0.02)
         && equalPolicyValue(options.stepSizeRecoveryGrowthFactor, 1.0 / 0.7)
         && equalPolicyValue(options.stepSizeScale, 10.0)
         && equalPolicyValue(options.couplingVariationTolerance, 0.9)
         && equalPolicyValue(options.shrinkFactor, 0.7)
         && options.maxStepShrinkIterations == 8
         && matchesCrossStepPolicyOptions(
             policy, authorPolicyInspiredCrossStepOptions());
}

const char* classifyCrossStepPolicy(
    const dart::constraint::ExactCoulombFbfConstraintSolver& solver)
{
  const auto& options = solver.getExactCoulombOptions();
  const auto policy = solver.getExactCoulombCrossStepPolicyOptions();
  if (matchesDartCurrentCrossStepPolicy(options, policy))
    return "dart_current";
  if (matchesAuthorPolicyInspiredCrossStepPolicy(options, policy))
    return "author_policy_inspired_b3f3c5c";
  return "unclassified";
}

void installCrossStepPolicy(
    dart::constraint::ExactCoulombFbfConstraintSolver& solver)
{
  if (gCrossStepPolicySelector == CrossStepPolicySelector::Default)
    return;

  if (gCrossStepPolicySelector == CrossStepPolicySelector::DartCurrent) {
    solver.setExactCoulombCrossStepPolicyOptions({});
    return;
  }

  auto options = solver.getExactCoulombOptions();
  options.warmStartMatchDistance = 0.02;
  options.stepSizeRecoveryGrowthFactor = 1.0 / 0.7;
  options.stepSizeScale = 10.0;
  options.couplingVariationTolerance = 0.9;
  options.shrinkFactor = 0.7;
  options.maxStepShrinkIterations = 8;
  solver.setExactCoulombOptions(options);
  solver.setExactCoulombCrossStepPolicyOptions(
      authorPolicyInspiredCrossStepOptions());
}

std::shared_ptr<dart::collision::NativeCollisionDetector>
createFbfPaperNativeCollisionDetector(Scenario scenario)
{
  using Mode = dart::collision::NativeCollisionDetector::ContactManifoldMode;
  auto detector = dart::collision::NativeCollisionDetector::create();
  const bool useFourPointPlanar
      = scenario == Scenario::InclineStick || scenario == Scenario::InclineSlide
        || isTurntableScenario(scenario) || isLiteralWedgeScenario(scenario);
  Mode mode = useFourPointPlanar ? Mode::FourPointPlanar : Mode::Compact;
  if (gNativeManifoldSensitivitySelector
      == NativeManifoldSensitivitySelector::Compact) {
    mode = Mode::Compact;
  } else if (
      gNativeManifoldSensitivitySelector
      == NativeManifoldSensitivitySelector::FourPointPlanar) {
    mode = Mode::FourPointPlanar;
  }
  detector->setContactManifoldMode(mode);
  return detector;
}

void configureWorldSolver(
    const std::shared_ptr<dart::simulation::World>& world,
    SolverMode solverMode,
    std::size_t maxContacts,
    std::size_t maxContactsPerPair,
    Scenario scenario,
    TraceScope traceScope,
    double initialStepSize,
    std::size_t simulationThreads,
    SolverContract contract,
    CollisionFrontend collisionFrontend)
{
  world->setTimeStep(kDt);
  world->setGravity(Eigen::Vector3d(0.0, 0.0, -kGravity));
  world->setNumSimulationThreads(simulationThreads);

  dart::simulation::DeactivationOptions deactivation;
  deactivation.mEnabled = false;
  world->setDeactivationOptions(deactivation);

  if (solverMode == SolverMode::ExactFbf) {
    auto solver
        = std::make_unique<dart::constraint::ExactCoulombFbfConstraintSolver>(
            makeFbfOptions(initialStepSize, scenario, traceScope, contract));
    if (collisionFrontend == CollisionFrontend::Native) {
      solver->setCollisionDetector(
          createFbfPaperNativeCollisionDetector(scenario));
    } else {
      solver->setCollisionDetector(
          dart::collision::DARTCollisionDetector::create());
    }
    solver->setNumSimulationThreads(simulationThreads);
    world->setConstraintSolver(std::move(solver));
    auto* installedExactSolver
        = static_cast<dart::constraint::ExactCoulombFbfConstraintSolver*>(
            world->getConstraintSolver());
    installedExactSolver->setExactCoulombColoredBlockGaussSeidelEnabled(
        contract == SolverContract::DartBestColoredBgs);
    installedExactSolver
        ->setExactCoulombColoredBlockGaussSeidelParticipantAffinityEnabled(
            contract == SolverContract::DartBestColoredBgs);
    // The long-run full-manifold scenarios separate position recovery from
    // the velocity-phase friction solve, matching the paper's formulation:
    // the exact solve then sees pure dynamics instead of DART's ERP bias.
    // Measured on the 26-card settle: steps 2+ fall back without this and
    // solve exactly at 1e-6 with it, ~30x faster. This must be set AFTER
    // installation because World::setConstraintSolver copies the previous
    // solver's split-impulse setting into the new one.
    if (scenario == Scenario::CardHouseFourLevelSettleProjectileFull
        || scenario == Scenario::MasonryArch25FullManifold
        || scenario == Scenario::MasonryArch101FullManifold
        || isLiteralWedgeScenario(scenario)) {
      world->getConstraintSolver()->setSplitImpulseEnabled(true);
    }
    // The exact velocity solve already enforces non-penetration, so the
    // wedged author arches need no penetration recovery at all; the
    // position pass's error-reduction push-apart (max ERV 0.1 m/s against
    // 7-10 cm voussoirs) otherwise inflates the arch apart while it seats
    // (measured: crown +3.4 cm in 0.5 s, collapse by 1.5 s, with or
    // without seating pre-stress).
    if (scenario == Scenario::MasonryArch25FullManifold
        || scenario == Scenario::MasonryArch101FullManifold
        || isLiteralWedgeScenario(scenario)) {
      dart::constraint::ContactConstraint::setErrorReductionParameter(0.0);
    }
  } else {
    auto* solver = world->getConstraintSolver();
    if (collisionFrontend == CollisionFrontend::Native) {
      solver->setCollisionDetector(
          createFbfPaperNativeCollisionDetector(scenario));
    } else {
      solver->setCollisionDetector(
          dart::collision::DARTCollisionDetector::create());
    }
    solver->setNumSimulationThreads(simulationThreads);
    // Keep the boxed baseline comparable: the long-run full scenarios use the
    // same split-impulse stepping mode for both solver modes.
    if (scenario == Scenario::CardHouseFourLevelSettleProjectileFull
        || scenario == Scenario::MasonryArch25FullManifold
        || scenario == Scenario::MasonryArch101FullManifold
        || isLiteralWedgeScenario(scenario)) {
      solver->setSplitImpulseEnabled(true);
    }
  }

  if (gSplitImpulseOverride == 0) {
    world->getConstraintSolver()->setSplitImpulseEnabled(false);
  } else if (gSplitImpulseOverride == 1) {
    world->getConstraintSolver()->setSplitImpulseEnabled(true);
  }

  auto& collisionOption = world->getConstraintSolver()->getCollisionOption();
  collisionOption.maxNumContacts = maxContacts;
  collisionOption.maxNumContactsPerPair = maxContactsPerPair;
}

dart::dynamics::SkeletonPtr createHorizontalPlane(double frictionCoeff)
{
  auto skeleton = dart::dynamics::Skeleton::create("ground_plane");
  auto body = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>()
                  .second;
  auto* shapeNode = body->createShapeNodeWith<
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(
      std::make_shared<dart::dynamics::PlaneShape>(
          Eigen::Vector3d::UnitZ(), 0.0));
  shapeNode->getDynamicsAspect()->setFrictionCoeff(frictionCoeff);
  skeleton->setMobile(false);
  return skeleton;
}

dart::dynamics::SkeletonPtr createInclinePlane(double frictionCoeff)
{
  const double theta = std::atan(kInclineTan);
  const Eigen::Vector3d normal(-std::sin(theta), 0.0, std::cos(theta));

  auto skeleton = dart::dynamics::Skeleton::create("incline_plane");
  auto body = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>()
                  .second;
  auto* shapeNode = body->createShapeNodeWith<
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(
      std::make_shared<dart::dynamics::PlaneShape>(normal, 0.0));
  shapeNode->getDynamicsAspect()->setFrictionCoeff(frictionCoeff);
  skeleton->setMobile(false);
  return skeleton;
}

dart::dynamics::SkeletonPtr createInclineCube(double frictionCoeff)
{
  const double theta = std::atan(kInclineTan);
  const Eigen::Vector3d normal(-std::sin(theta), 0.0, std::cos(theta));
  const Eigen::Vector3d size = Eigen::Vector3d::Ones();
  constexpr double kMass = 1.0;
  constexpr double kInitialPenetration = 0.01;

  auto skeleton = dart::dynamics::Skeleton::create("incline_cube");
  dart::dynamics::GenericJoint<dart::math::SE3Space>::Properties
      jointProperties(std::string("incline_cube_joint"));
  dart::dynamics::BodyNode::Properties bodyProperties(
      dart::dynamics::BodyNode::AspectProperties("incline_cube_body"));
  bodyProperties.mInertia.setMass(kMass);
  bodyProperties.mInertia.setMoment(
      dart::dynamics::BoxShape::computeInertia(size, kMass));

  auto pair = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>(
      nullptr, jointProperties, bodyProperties);
  auto* joint = pair.first;
  auto* body = pair.second;
  auto* shapeNode = body->createShapeNodeWith<
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(
      std::make_shared<dart::dynamics::BoxShape>(size));
  shapeNode->getDynamicsAspect()->setFrictionCoeff(frictionCoeff);

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear()
      = Eigen::AngleAxisd(-theta, Eigen::Vector3d::UnitY()).toRotationMatrix();
  transform.translation() = normal * (0.5 * size.z() - kInitialPenetration);
  joint->setPositions(dart::dynamics::FreeJoint::convertToPositions(transform));
  return skeleton;
}

dart::dynamics::SkeletonPtr createBackspinSphere()
{
  constexpr double kMass = 1.0;
  constexpr double kInitialPenetration = 0.005;

  auto skeleton = dart::dynamics::Skeleton::create("backspin_sphere");
  dart::dynamics::GenericJoint<dart::math::SE3Space>::Properties
      jointProperties(std::string("backspin_sphere_joint"));
  dart::dynamics::BodyNode::Properties bodyProperties(
      dart::dynamics::BodyNode::AspectProperties("backspin_sphere_body"));
  bodyProperties.mInertia.setMass(kMass);
  bodyProperties.mInertia.setMoment(
      dart::dynamics::SphereShape::computeInertia(kBackspinRadius, kMass));

  auto pair = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>(
      nullptr, jointProperties, bodyProperties);
  auto* joint = pair.first;
  auto* body = pair.second;
  auto* shapeNode = body->createShapeNodeWith<
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(
      std::make_shared<dart::dynamics::SphereShape>(kBackspinRadius));
  shapeNode->getDynamicsAspect()->setFrictionCoeff(kBackspinFriction);

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation().z() = kBackspinRadius - kInitialPenetration;
  joint->setPositions(dart::dynamics::FreeJoint::convertToPositions(transform));
  joint->setLinearVelocity(Eigen::Vector3d(kBackspinLinearVelocity, 0.0, 0.0));
  joint->setAngularVelocity(
      Eigen::Vector3d(0.0, kBackspinAngularVelocity, 0.0));
  return skeleton;
}

dart::dynamics::SkeletonPtr createTurntableSupport(double frictionCoeff)
{
  constexpr double kThickness = 0.1;

  auto skeleton = dart::dynamics::Skeleton::create("turntable");
  dart::dynamics::GenericJoint<dart::math::SE3Space>::Properties
      jointProperties(std::string("turntable_joint"));
  dart::dynamics::BodyNode::Properties bodyProperties(
      dart::dynamics::BodyNode::AspectProperties("turntable_body"));
  bodyProperties.mInertia.setMass(1.0);

  auto pair = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>(
      nullptr, jointProperties, bodyProperties);
  auto* joint = pair.first;
  auto* body = pair.second;
  auto* shapeNode = body->createShapeNodeWith<
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(
      std::make_shared<dart::dynamics::BoxShape>(
          Eigen::Vector3d(4.0, 4.0, kThickness)));
  shapeNode->getDynamicsAspect()->setFrictionCoeff(frictionCoeff);

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation().z() = -0.5 * kThickness;
  joint->setPositions(dart::dynamics::FreeJoint::convertToPositions(transform));
  skeleton->setMobile(false);
  return skeleton;
}

dart::dynamics::SkeletonPtr createTurntableRider(double frictionCoeff)
{
  const Eigen::Vector3d size = Eigen::Vector3d::Constant(0.25);
  constexpr double kMass = 1.0;
  constexpr double kInitialPenetration = 0.005;

  auto skeleton = dart::dynamics::Skeleton::create("turntable_rider");
  dart::dynamics::GenericJoint<dart::math::SE3Space>::Properties
      jointProperties(std::string("turntable_rider_joint"));
  dart::dynamics::BodyNode::Properties bodyProperties(
      dart::dynamics::BodyNode::AspectProperties("turntable_rider_body"));
  bodyProperties.mInertia.setMass(kMass);
  bodyProperties.mInertia.setMoment(
      dart::dynamics::BoxShape::computeInertia(size, kMass));

  auto pair = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>(
      nullptr, jointProperties, bodyProperties);
  auto* joint = pair.first;
  auto* body = pair.second;
  auto* shapeNode = body->createShapeNodeWith<
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(
      std::make_shared<dart::dynamics::BoxShape>(size));
  shapeNode->getDynamicsAspect()->setFrictionCoeff(frictionCoeff);

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(
      kTurntableInitialRadius, 0.0, 0.5 * size.z() - kInitialPenetration);
  joint->setPositions(dart::dynamics::FreeJoint::convertToPositions(transform));
  return skeleton;
}

dart::dynamics::SkeletonPtr createAuthorTurntableSupport(double frictionCoeff)
{
  auto skeleton = dart::dynamics::Skeleton::create("turntable");
  dart::dynamics::GenericJoint<dart::math::SE3Space>::Properties
      jointProperties(std::string("turntable_joint"));
  dart::dynamics::BodyNode::Properties bodyProperties(
      dart::dynamics::BodyNode::AspectProperties("turntable_body"));
  bodyProperties.mInertia.setMass(1.0);

  auto pair = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>(
      nullptr, jointProperties, bodyProperties);
  auto* joint = pair.first;
  auto* body = pair.second;
  auto* shapeNode = body->createShapeNodeWith<
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(
      std::make_shared<dart::dynamics::CylinderShape>(
          fbf_author_turntable::kSupportRadius,
          2.0 * fbf_author_turntable::kSupportHalfHeight));
  shapeNode->getDynamicsAspect()->setFrictionCoeff(frictionCoeff);

  // The public author scene centers the kinematic cylinder at z=half_height,
  // so its lower and upper faces are z=0 and z=0.1. DART supplies the rigid
  // body and Native collision implementation; no renderer asset is implied.
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation().z() = fbf_author_turntable::kSupportHalfHeight;
  joint->setPositions(dart::dynamics::FreeJoint::convertToPositions(transform));
  skeleton->setMobile(false);
  return skeleton;
}

dart::dynamics::SkeletonPtr createAuthorTurntableRider(double frictionCoeff)
{
  constexpr double kSide = 2.0 * fbf_author_turntable::kRiderHalfSize;
  constexpr double kMass
      = fbf_author_turntable::kRiderDensity * kSide * kSide * kSide;
  const Eigen::Vector3d size = Eigen::Vector3d::Constant(kSide);

  auto skeleton = dart::dynamics::Skeleton::create("turntable_rider");
  dart::dynamics::GenericJoint<dart::math::SE3Space>::Properties
      jointProperties(std::string("turntable_rider_joint"));
  dart::dynamics::BodyNode::Properties bodyProperties(
      dart::dynamics::BodyNode::AspectProperties("turntable_rider_body"));
  bodyProperties.mInertia.setMass(kMass);
  bodyProperties.mInertia.setMoment(
      dart::dynamics::BoxShape::computeInertia(size, kMass));

  auto pair = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>(
      nullptr, jointProperties, bodyProperties);
  auto* joint = pair.first;
  auto* body = pair.second;
  auto* shapeNode = body->createShapeNodeWith<
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(
      std::make_shared<dart::dynamics::BoxShape>(size));
  shapeNode->getDynamicsAspect()->setFrictionCoeff(frictionCoeff);

  // Public-source placement: table top + cube half-size + geometric gap +
  // drop height = 0.1 + 0.15 + 0.005 + 0.2 = 0.455 m.
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(
      fbf_author_turntable::kInitialRadius,
      0.0,
      2.0 * fbf_author_turntable::kSupportHalfHeight
          + fbf_author_turntable::kRiderHalfSize
          + fbf_author_turntable::kGeometricGap
          + fbf_author_turntable::kDropHeight);
  joint->setPositions(dart::dynamics::FreeJoint::convertToPositions(transform));
  return skeleton;
}

dart::dynamics::SkeletonPtr createPainleveBox(double frictionCoeff)
{
  const Eigen::Vector3d size(0.6, 0.6, 1.0);
  constexpr double kMass = 1.0;
  constexpr double kInitialPenetration = 0.005;

  auto skeleton = dart::dynamics::Skeleton::create("painleve_box");
  dart::dynamics::GenericJoint<dart::math::SE3Space>::Properties
      jointProperties(std::string("painleve_box_joint"));
  dart::dynamics::BodyNode::Properties bodyProperties(
      dart::dynamics::BodyNode::AspectProperties("painleve_box_body"));
  bodyProperties.mInertia.setMass(kMass);
  bodyProperties.mInertia.setMoment(
      dart::dynamics::BoxShape::computeInertia(size, kMass));

  auto pair = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>(
      nullptr, jointProperties, bodyProperties);
  auto* joint = pair.first;
  auto* body = pair.second;
  auto* shapeNode = body->createShapeNodeWith<
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(
      std::make_shared<dart::dynamics::BoxShape>(size));
  shapeNode->getDynamicsAspect()->setFrictionCoeff(frictionCoeff);

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear()
      = Eigen::AngleAxisd(kPainleveInitialPitch, Eigen::Vector3d::UnitY())
            .toRotationMatrix();
  const double verticalHalfExtent
      = 0.5
        * (std::abs(transform.linear()(2, 0)) * size.x()
           + std::abs(transform.linear()(2, 1)) * size.y()
           + std::abs(transform.linear()(2, 2)) * size.z());
  transform.translation().z() = verticalHalfExtent - kInitialPenetration;
  joint->setPositions(dart::dynamics::FreeJoint::convertToPositions(transform));
  joint->setLinearVelocity(Eigen::Vector3d(kPainleveInitialVelocity, 0.0, 0.0));
  return skeleton;
}

double computeCardHouseVerticalHalfExtent(const Eigen::Matrix3d& rotation)
{
  const Eigen::Vector3d size(
      kCardHouseThickness, kCardHouseWidth, kCardHouseHeight);
  return 0.5
         * (std::abs(rotation(2, 0)) * size.x()
            + std::abs(rotation(2, 1)) * size.y()
            + std::abs(rotation(2, 2)) * size.z());
}

Eigen::Isometry3d createCardHouseAFrameTransform(
    double centerX, double baseZ, bool leftCard)
{
  const double angle = leftCard ? kCardHouseAngle : -kCardHouseAngle;
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear()
      = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitY()).toRotationMatrix();
  const double horizontalHalfExtent
      = 0.5
        * (kCardHouseHeight * std::sin(kCardHouseAngle)
           + kCardHouseThickness * std::cos(kCardHouseAngle));
  const double centerOffset
      = horizontalHalfExtent - 0.5 * kCardHouseInitialPenetration;
  transform.translation().x()
      = centerX + (leftCard ? -centerOffset : centerOffset);
  transform.translation().z()
      = baseZ + computeCardHouseVerticalHalfExtent(transform.linear())
        - kCardHouseInitialPenetration;
  return transform;
}

Eigen::Isometry3d createCardHouseHorizontalSupportTransform(
    double centerX, double baseZ)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() = Eigen::AngleAxisd(0.5 * kPi, Eigen::Vector3d::UnitY())
                           .toRotationMatrix();
  transform.translation().x() = centerX;
  transform.translation().z()
      = baseZ + 0.5 * kCardHouseThickness - kCardHouseInitialPenetration;
  return transform;
}

dart::dynamics::SkeletonPtr createCardPlate(
    const std::string& name, const Eigen::Isometry3d& transform)
{
  const Eigen::Vector3d size(
      kCardHouseThickness, kCardHouseWidth, kCardHouseHeight);
  const double mass = kCardHouseDensity * size.x() * size.y() * size.z();

  auto skeleton = dart::dynamics::Skeleton::create(name);
  dart::dynamics::GenericJoint<dart::math::SE3Space>::Properties
      jointProperties(name + "_joint");
  dart::dynamics::BodyNode::Properties bodyProperties(
      dart::dynamics::BodyNode::AspectProperties(name + "_body"));
  bodyProperties.mInertia.setMass(mass);
  bodyProperties.mInertia.setMoment(
      dart::dynamics::BoxShape::computeInertia(size, mass));

  auto pair = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>(
      nullptr, jointProperties, bodyProperties);
  auto* joint = pair.first;
  auto* body = pair.second;
  auto* shapeNode = body->createShapeNodeWith<
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(
      std::make_shared<dart::dynamics::BoxShape>(size));
  shapeNode->getDynamicsAspect()->setFrictionCoeff(kCardHouseFriction);
  joint->setPositions(dart::dynamics::FreeJoint::convertToPositions(transform));
  return skeleton;
}

dart::dynamics::SkeletonPtr createCardHouseProjectile(std::size_t index)
{
  auto skeleton = dart::dynamics::Skeleton::create(
      "fbf_projectile_" + std::to_string(index));
  dart::dynamics::GenericJoint<dart::math::SE3Space>::Properties
      jointProperties(skeleton->getName() + "_joint");
  dart::dynamics::BodyNode::Properties bodyProperties(
      dart::dynamics::BodyNode::AspectProperties(
          skeleton->getName() + "_body"));
  bodyProperties.mInertia.setMass(kCardHouseProjectileMass);
  bodyProperties.mInertia.setMoment(dart::dynamics::BoxShape::computeInertia(
      Eigen::Vector3d::Constant(kCardHouseProjectileEdgeLength),
      kCardHouseProjectileMass));

  auto pair = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>(
      nullptr, jointProperties, bodyProperties);
  auto* joint = pair.first;
  auto* body = pair.second;
  auto* shapeNode = body->createShapeNodeWith<
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(
      std::make_shared<dart::dynamics::BoxShape>(
          Eigen::Vector3d::Constant(kCardHouseProjectileEdgeLength)));
  shapeNode->getDynamicsAspect()->setFrictionCoeff(kCardHouseFriction);

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(
      (static_cast<double>(index) - 1.5) * 0.15,
      0.0,
      kCardHouseProjectileDropHeight);
  joint->setPositions(dart::dynamics::FreeJoint::convertToPositions(transform));
  joint->setLinearVelocity(
      Eigen::Vector3d(0.0, 0.0, -kCardHouseProjectileSpeed));
  return skeleton;
}

void addCardHouse(
    const std::shared_ptr<dart::simulation::World>& world,
    std::size_t levelCount)
{
  world->addSkeleton(createHorizontalPlane(kCardHouseFriction));

  const Eigen::Matrix3d aFrameRotation
      = Eigen::AngleAxisd(kCardHouseAngle, Eigen::Vector3d::UnitY())
            .toRotationMatrix();
  const double aFrameHeight
      = 2.0 * computeCardHouseVerticalHalfExtent(aFrameRotation);
  double baseZ = 0.0;

  for (std::size_t level = 0u; level < levelCount; ++level) {
    const std::size_t frameCount = levelCount - level;
    for (std::size_t frame = 0u; frame < frameCount; ++frame) {
      const double centerX
          = (static_cast<double>(frame) - 0.5 * (frameCount - 1))
            * kCardHouseFrameSpacing;
      world->addSkeleton(createCardPlate(
          "card_house_l" + std::to_string(level) + "_f" + std::to_string(frame)
              + "_left",
          createCardHouseAFrameTransform(centerX, baseZ, true)));
      world->addSkeleton(createCardPlate(
          "card_house_l" + std::to_string(level) + "_f" + std::to_string(frame)
              + "_right",
          createCardHouseAFrameTransform(centerX, baseZ, false)));
    }

    if (level + 1u == levelCount) {
      break;
    }

    const double supportBaseZ
        = baseZ + aFrameHeight - kCardHouseInitialPenetration;
    for (std::size_t support = 0u; support + 1u < frameCount; ++support) {
      const double centerX
          = (static_cast<double>(support) + 0.5 - 0.5 * (frameCount - 1))
            * kCardHouseFrameSpacing;
      world->addSkeleton(createCardPlate(
          "card_house_l" + std::to_string(level) + "_support"
              + std::to_string(support),
          createCardHouseHorizontalSupportTransform(centerX, supportBaseZ)));
    }

    baseZ = supportBaseZ + kCardHouseThickness - kCardHouseInitialPenetration;
  }
}

dart::dynamics::SkeletonPtr createMasonryArchStone(
    std::size_t index,
    const dart::math::detail::MasonryArchStoneBoxGeometry& geometry)
{
  auto skeleton = dart::dynamics::Skeleton::create(
      "masonry_arch_stone_" + std::to_string(index));
  dart::dynamics::GenericJoint<dart::math::SE3Space>::Properties
      jointProperties("masonry_arch_stone_" + std::to_string(index) + "_joint");
  dart::dynamics::BodyNode::Properties bodyProperties(
      dart::dynamics::BodyNode::AspectProperties(
          "masonry_arch_stone_" + std::to_string(index) + "_body"));
  auto pair = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>(
      nullptr, jointProperties, bodyProperties);
  auto* joint = pair.first;
  auto* body = pair.second;
  auto shape = std::make_shared<dart::dynamics::BoxShape>(geometry.size);
  auto* shapeNode = body->createShapeNodeWith<
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(shape);
  shapeNode->getDynamicsAspect()->setFrictionCoeff(kArchFriction);
  joint->setPositions(
      dart::dynamics::FreeJoint::convertToPositions(geometry.transform));

  // Mobility is assigned by the paper-scene builder below.
  const double mass = kArchDensity * geometry.size.x() * geometry.size.y()
                      * geometry.size.z();
  dart::dynamics::Inertia inertia;
  inertia.setMass(mass);
  inertia.setMoment(
      dart::dynamics::BoxShape::computeInertia(geometry.size, mass));
  body->setInertia(inertia);
  return skeleton;
}

std::shared_ptr<dart::dynamics::ConvexMeshShape> createLiteralMasonryArchShape(
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

dart::dynamics::SkeletonPtr createLiteralMasonryArchStone(
    std::size_t index,
    const dart::math::detail::MasonryArchStoneWedgeGeometry& geometry)
{
  const std::string name = "masonry_arch_stone_" + std::to_string(index);
  auto skeleton = dart::dynamics::Skeleton::create(name);
  dart::dynamics::GenericJoint<dart::math::SE3Space>::Properties
      jointProperties(name + "_joint");
  dart::dynamics::BodyNode::Properties bodyProperties(
      dart::dynamics::BodyNode::AspectProperties(name + "_body"));
  auto pair = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>(
      nullptr, jointProperties, bodyProperties);
  auto* joint = pair.first;
  auto* body = pair.second;

  auto* shapeNode = body->createShapeNodeWith<
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(createLiteralMasonryArchShape(geometry));
  shapeNode->setName(name + "_shape");
  shapeNode->getDynamicsAspect()->setFrictionCoeff(kArchFriction);

  const double mass = kArchDensity * geometry.volume;
  dart::dynamics::Inertia inertia;
  inertia.setMass(mass);
  inertia.setMoment(mass * geometry.momentPerUnitMass);
  body->setInertia(inertia);

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation()
      = geometry.centroid
        - kLiteralArchDownwardShift * Eigen::Vector3d::UnitZ();
  joint->setPositions(dart::dynamics::FreeJoint::convertToPositions(transform));
  return skeleton;
}

dart::dynamics::SkeletonPtr createMasonryArchProjectile(std::size_t index)
{
  auto skeleton = dart::dynamics::Skeleton::create(
      "masonry_arch_projectile_" + std::to_string(index));
  dart::dynamics::GenericJoint<dart::math::SE3Space>::Properties
      jointProperties(skeleton->getName() + "_joint");
  dart::dynamics::BodyNode::Properties bodyProperties(
      dart::dynamics::BodyNode::AspectProperties(
          skeleton->getName() + "_body"));
  bodyProperties.mInertia.setMass(kArchProjectileMass);
  bodyProperties.mInertia.setMoment(dart::dynamics::BoxShape::computeInertia(
      Eigen::Vector3d::Constant(kArchProjectileEdgeLength),
      kArchProjectileMass));

  auto pair = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>(
      nullptr, jointProperties, bodyProperties);
  auto* joint = pair.first;
  auto* body = pair.second;
  auto* shapeNode = body->createShapeNodeWith<
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(
      std::make_shared<dart::dynamics::BoxShape>(
          Eigen::Vector3d::Constant(kArchProjectileEdgeLength)));
  shapeNode->getDynamicsAspect()->setFrictionCoeff(kArchFriction);

  // Aim at the crown (topmost, narrowest) stone, just outside its depth
  // face, regardless of stone count.
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(
      (static_cast<double>(index)
       - 0.5 * static_cast<double>(kArchProjectileCount - 1u))
          * kArchProjectileSpacing,
      0.0,
      kArchProjectileDropHeight);
  joint->setPositions(dart::dynamics::FreeJoint::convertToPositions(transform));
  joint->setLinearVelocity(Eigen::Vector3d(0.0, 0.0, -kArchProjectileSpeed));
  return skeleton;
}

dart::dynamics::SkeletonPtr createLiteralCrownImpactProjectile(
    std::size_t index)
{
  auto skeleton = dart::dynamics::Skeleton::create(
      "masonry_arch_projectile_impact_v1_" + std::to_string(index));
  dart::dynamics::GenericJoint<dart::math::SE3Space>::Properties
      jointProperties(skeleton->getName() + "_joint");
  dart::dynamics::BodyNode::Properties bodyProperties(
      dart::dynamics::BodyNode::AspectProperties(
          skeleton->getName() + "_body"));
  bodyProperties.mInertia.setMass(kLiteralCrownImpactProjectileMass);
  bodyProperties.mInertia.setMoment(dart::dynamics::BoxShape::computeInertia(
      Eigen::Vector3d::Constant(kLiteralCrownImpactProjectileEdgeLength),
      kLiteralCrownImpactProjectileMass));

  auto pair = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>(
      nullptr, jointProperties, bodyProperties);
  auto* joint = pair.first;
  auto* body = pair.second;
  auto* shapeNode = body->createShapeNodeWith<
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(
      std::make_shared<dart::dynamics::BoxShape>(
          Eigen::Vector3d::Constant(kLiteralCrownImpactProjectileEdgeLength)));
  shapeNode->getDynamicsAspect()->setFrictionCoeff(kArchFriction);

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(
      (static_cast<double>(index) - 1.0) * kLiteralCrownImpactProjectileSpacing,
      0.0,
      kLiteralCrownImpactProjectileDropHeight);
  joint->setPositions(dart::dynamics::FreeJoint::convertToPositions(transform));
  joint->setLinearVelocity(
      Eigen::Vector3d(0.0, 0.0, -kLiteralCrownImpactProjectileSpeed));
  joint->setAngularVelocity(Eigen::Vector3d::Zero());
  return skeleton;
}

void addMasonryArch(
    const std::shared_ptr<dart::simulation::World>& world,
    std::size_t stoneCount)
{
  // Paper Fig. 7 pins the 25-stone springers. Fig. 8 calls the 101-stone case
  // the same setup, so both endpoints are pinned at either scale. This timed
  // path uses source-derived weighted-catenary placement but oriented boxes,
  // not the literal tapered wedges/exact inertia owned by the separate
  // collision audit. The raw Rigid-IPC scene remains all-dynamic.
  world->addSkeleton(createHorizontalPlane(kArchFriction));

  const auto stoneGeometry
      = dart::math::detail::generateMasonryArchStoneBoxes(stoneCount);
  for (std::size_t i = 0u; i < stoneCount; ++i) {
    auto stone = createMasonryArchStone(i, stoneGeometry[i]);
    // The two springer (endpoint) stones are pinned and the interior stones
    // are dynamic, matching the paper contract above.
    if (i == 0u || i + 1u == stoneCount) {
      stone->setMobile(false);
    }
    world->addSkeleton(stone);
  }
}

void addLiteralMasonryArch(
    const std::shared_ptr<dart::simulation::World>& world,
    std::size_t stoneCount)
{
  world->addSkeleton(createHorizontalPlane(kArchFriction));
  const auto geometries = dart::math::detail::generateMasonryArchStoneWedges(
      stoneCount,
      {},
      dart::math::detail::MasonryArchBarrierGapPolicy::OmitSourceOffsets,
      kLiteralArchEndFaceExpansion);
  for (std::size_t i = 0u; i < geometries.size(); ++i) {
    auto stone = createLiteralMasonryArchStone(i, geometries[i]);
    if (i == 0u || i + 1u == geometries.size())
      stone->setMobile(false);
    world->addSkeleton(stone);
  }
}

void addLiteralMasonryArch25(
    const std::shared_ptr<dart::simulation::World>& world)
{
  addLiteralMasonryArch(world, kArchStoneCount);
}

std::shared_ptr<dart::simulation::World> createTraceWorld(
    Scenario scenario,
    SolverMode solverMode,
    TraceScope traceScope,
    double initialStepSize,
    std::size_t simulationThreads,
    SolverContract contract,
    CollisionFrontend collisionFrontend)
{
  const double frictionCoeff = scenarioFriction(scenario);
  const std::size_t maxContacts
      = scenario == Scenario::Backspin ? 1u
        : scenario == Scenario::CardHouseFourLevelReduced
                || scenario
                       == Scenario::CardHouseFourLevelSettleProjectileReduced
                || scenario == Scenario::CardHouseFourLevelSettleProjectileFull
            ? kCardHouseReducedMaxContacts
        : scenario == Scenario::MasonryArch25Reduced
                || scenario == Scenario::MasonryArch25ProjectileReduced
            ? kArchReducedMaxContacts
        : scenario == Scenario::MasonryArch101Reduced
            ? kArch101ReducedMaxContacts
        : scenario == Scenario::MasonryArch25FullManifold
                || scenario == Scenario::MasonryArch101FullManifold
            ? kArchFullManifoldMaxContacts
        : scenario == Scenario::MasonryArch101LiteralWedge
            ? kLiteralArch101MaxContacts
        : scenario == Scenario::MasonryArch25LiteralWedge
                || scenario == Scenario::MasonryArch25LiteralWedgeCrownImpactV1
            ? kLiteralArchMaxContacts
        : isAuthorTurntableScenario(scenario)
            ? fbf_author_turntable::kMaxContacts
            : 4u;
  const std::size_t maxContactsPerPair
      = scenario == Scenario::CardHouseFourLevelReduced
                || scenario
                       == Scenario::CardHouseFourLevelSettleProjectileReduced
            ? kCardHouseReducedMaxContactsPerPair
        : scenario == Scenario::CardHouseFourLevelSettleProjectileFull
            // The repaired reconstruction emits the same 96 initial contacts
            // at per-pair caps 4 and 8. Keep the lower deterministic cap; the
            // trajectory still records its measured contact count each step.
            ? kCardHouseReducedMaxContactsPerPair
            : scenario == Scenario::MasonryArch25Reduced
                      || scenario == Scenario::MasonryArch25ProjectileReduced
                  ? kArchReducedMaxContactsPerPair
              : scenario == Scenario::MasonryArch101Reduced
                  ? kArch101ReducedMaxContactsPerPair
              : scenario == Scenario::MasonryArch25FullManifold
                      || scenario == Scenario::MasonryArch101FullManifold
                  ? kArchFullManifoldMaxContactsPerPair
              : scenario == Scenario::MasonryArch25LiteralWedge
                      || scenario == Scenario::MasonryArch101LiteralWedge
                      || scenario
                             == Scenario::MasonryArch25LiteralWedgeCrownImpactV1
                  ? kLiteralArchMaxContactsPerPair
              : isAuthorTurntableScenario(scenario)
                  ? fbf_author_turntable::kMaxContactsPerPair
                  : maxContacts;

  auto world = dart::simulation::World::create(
      std::string("exact_coulomb_trace_") + scenarioName(scenario));
  configureWorldSolver(
      world,
      solverMode,
      maxContacts,
      maxContactsPerPair,
      scenario,
      traceScope,
      initialStepSize,
      simulationThreads,
      contract,
      collisionFrontend);

  switch (scenario) {
    case Scenario::InclineStick:
    case Scenario::InclineSlide:
      world->addSkeleton(createInclinePlane(frictionCoeff));
      world->addSkeleton(createInclineCube(frictionCoeff));
      break;
    case Scenario::Backspin:
      world->addSkeleton(createHorizontalPlane(kBackspinFriction));
      world->addSkeleton(createBackspinSphere());
      break;
    case Scenario::TurntableLowSlow:
    case Scenario::TurntableLowFast:
    case Scenario::TurntableHighSlow:
    case Scenario::TurntableHighFast:
      world->addSkeleton(createTurntableSupport(frictionCoeff));
      world->addSkeleton(createTurntableRider(frictionCoeff));
      break;
    case Scenario::AuthorTurntableLowSlow:
    case Scenario::AuthorTurntableLowFast:
    case Scenario::AuthorTurntableHighSlow:
    case Scenario::AuthorTurntableHighFast:
      world->addSkeleton(createAuthorTurntableSupport(frictionCoeff));
      world->addSkeleton(createAuthorTurntableRider(frictionCoeff));
      break;
    case Scenario::PainleveSlide:
    case Scenario::PainleveTumble:
      world->addSkeleton(createHorizontalPlane(frictionCoeff));
      world->addSkeleton(createPainleveBox(frictionCoeff));
      break;
    case Scenario::CardHouseFourLevelReduced:
    case Scenario::CardHouseFourLevelSettleProjectileReduced:
    case Scenario::CardHouseFourLevelSettleProjectileFull:
      addCardHouse(world, kCardHouseFourLevelCount);
      break;
    case Scenario::MasonryArch25Reduced:
      addMasonryArch(world, kArchStoneCount);
      break;
    case Scenario::MasonryArch25ProjectileReduced:
      addMasonryArch(world, kArchStoneCount);
      for (std::size_t i = 0u; i < kArchProjectileCount; ++i)
        world->addSkeleton(createMasonryArchProjectile(i));
      break;
    case Scenario::MasonryArch101Reduced:
      addMasonryArch(world, kArch101StoneCount);
      break;
    case Scenario::MasonryArch25FullManifold:
      addMasonryArch(world, kArchStoneCount);
      break;
    case Scenario::MasonryArch101FullManifold:
      addMasonryArch(world, kArch101StoneCount);
      break;
    case Scenario::MasonryArch25LiteralWedge:
    case Scenario::MasonryArch25LiteralWedgeCrownImpactV1:
      addLiteralMasonryArch25(world);
      break;
    case Scenario::MasonryArch101LiteralWedge:
      addLiteralMasonryArch(world, kArch101StoneCount);
      break;
  }

  return world;
}

dart::constraint::ExactCoulombFbfConstraintSolver* getExactSolver(
    const std::shared_ptr<dart::simulation::World>& world)
{
  return dynamic_cast<dart::constraint::ExactCoulombFbfConstraintSolver*>(
      world->getConstraintSolver());
}

bool isCardHouseCardSkeletonName(const std::string& name)
{
  return name.rfind("card_house_l", 0u) == 0u;
}

bool isCardHouseProjectileSkeletonName(const std::string& name)
{
  return name.rfind("fbf_projectile_", 0u) == 0u;
}

bool isMasonryArchProjectileSkeletonName(const std::string& name)
{
  return name.rfind("masonry_arch_projectile_", 0u) == 0u;
}

void launchCardHouseProjectiles(
    const std::shared_ptr<dart::simulation::World>& world)
{
  if (world->getSkeleton("fbf_projectile_0") != nullptr)
    return;

  for (std::size_t i = 0u; i < kCardHouseProjectileCount; ++i)
    world->addSkeleton(createCardHouseProjectile(i));
}

void launchLiteralCrownImpactProjectiles(
    const std::shared_ptr<dart::simulation::World>& world)
{
  if (world->getSkeleton("masonry_arch_projectile_impact_v1_0") != nullptr)
    return;

  for (std::size_t i = 0u; i < kLiteralCrownImpactProjectileCount; ++i)
    world->addSkeleton(createLiteralCrownImpactProjectile(i));
}

void applyScenarioControl(
    const std::shared_ptr<dart::simulation::World>& world,
    Scenario scenario,
    std::size_t step)
{
  const std::size_t settleSteps
      = scenario == Scenario::CardHouseFourLevelSettleProjectileFull
            ? kCardHouseSettleProjectileFullSettleSteps
            : kCardHouseSettleProjectileSettleSteps;
  if ((scenario == Scenario::CardHouseFourLevelSettleProjectileReduced
       || scenario == Scenario::CardHouseFourLevelSettleProjectileFull)
      && step == settleSteps) {
    launchCardHouseProjectiles(world);
  }

  if (scenario == Scenario::MasonryArch25LiteralWedgeCrownImpactV1
      && step == kLiteralCrownImpactLaunchAfterSteps) {
    launchLiteralCrownImpactProjectiles(world);
  }

  if (!isTurntableScenario(scenario))
    return;

  auto turntable = world->getSkeleton("turntable");
  if (!turntable)
    return;

  auto* joint = static_cast<dart::dynamics::FreeJoint*>(turntable->getJoint(0));
  const double time = static_cast<double>(step) * kDt;
  if (isAuthorTurntableScenario(scenario)) {
    const double targetAngularVelocity = turntableAngularVelocity(scenario);
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.linear()
        = Eigen::AngleAxisd(
              fbf_author_turntable::integratedYaw(time, targetAngularVelocity),
              Eigen::Vector3d::UnitZ())
              .toRotationMatrix();
    transform.translation().z() = fbf_author_turntable::kSupportHalfHeight;
    joint->setPositions(
        dart::dynamics::FreeJoint::convertToPositions(transform));
    joint->setAngularVelocity(Eigen::Vector3d(
        0.0,
        0.0,
        fbf_author_turntable::angularVelocity(time, targetAngularVelocity)));
    return;
  }

  double ramp = std::min(time / kTurntableRampDuration, 1.0);
  joint->setAngularVelocity(
      Eigen::Vector3d(0.0, 0.0, ramp * turntableAngularVelocity(scenario)));
}

double safeValue(double value)
{
  return std::isfinite(value) ? value
                              : std::numeric_limits<double>::quiet_NaN();
}

std::string formatLogicalCpuIds(const std::vector<int>& logicalCpuIds)
{
  if (logicalCpuIds.empty())
    return "none";

  std::ostringstream output;
  for (std::size_t i = 0u; i < logicalCpuIds.size(); ++i) {
    if (i > 0u)
      output << ';';
    output << logicalCpuIds[i];
  }
  return output.str();
}

double sortedPercentile(
    const std::vector<double>& sortedValues, double percentile)
{
  if (sortedValues.empty())
    return std::numeric_limits<double>::quiet_NaN();
  if (sortedValues.size() == 1u)
    return sortedValues.front();

  const double position
      = static_cast<double>(sortedValues.size() - 1u) * percentile;
  const auto lower = static_cast<std::size_t>(std::floor(position));
  const auto upper = static_cast<std::size_t>(std::ceil(position));
  const double weight = position - static_cast<double>(lower);
  return sortedValues[lower] * (1.0 - weight) + sortedValues[upper] * weight;
}

CollisionMetrics collectCollisionMetrics(
    const std::shared_ptr<dart::simulation::World>& world)
{
  CollisionMetrics metrics;
  const auto& collisionResult
      = world->getConstraintSolver()->getLastCollisionResult();
  std::set<std::pair<std::uintptr_t, std::uintptr_t>> bodyPairs;
  std::vector<double> penetrationDepths;
  penetrationDepths.reserve(collisionResult.getNumContacts());

  for (const auto& contact : collisionResult.getContacts()) {
    const auto body1 = contact.getBodyNodePtr1();
    const auto body2 = contact.getBodyNodePtr2();
    if (body1 && body2) {
      const auto address1 = reinterpret_cast<std::uintptr_t>(body1.get());
      const auto address2 = reinterpret_cast<std::uintptr_t>(body2.get());
      bodyPairs.emplace(
          std::min(address1, address2), std::max(address1, address2));
    }
    if (std::isfinite(contact.penetrationDepth))
      penetrationDepths.push_back(contact.penetrationDepth);
  }

  metrics.uniqueBodyPairs = bodyPairs.size();
  if (!penetrationDepths.empty()) {
    std::sort(penetrationDepths.begin(), penetrationDepths.end());
    metrics.penetrationDepthMin = penetrationDepths.front();
    metrics.penetrationDepthMedian = sortedPercentile(penetrationDepths, 0.5);
    metrics.penetrationDepthP95 = sortedPercentile(penetrationDepths, 0.95);
    metrics.penetrationDepthMax = penetrationDepths.back();
  }
  return metrics;
}

ContactPairMultiplicityMetrics collectContactPairMultiplicityMetrics(
    const std::shared_ptr<dart::simulation::World>& world)
{
  std::map<std::string, std::size_t> multiplicities;
  const auto& collisionResult
      = world->getConstraintSolver()->getLastCollisionResult();
  for (const auto& contact : collisionResult.getContacts()) {
    const auto body1 = contact.getBodyNodePtr1();
    const auto body2 = contact.getBodyNodePtr2();
    std::string name1 = body1 == nullptr ? "null" : body1->getName();
    std::string name2 = body2 == nullptr ? "null" : body2->getName();
    if (name2 < name1)
      std::swap(name1, name2);
    ++multiplicities[name1 + "|" + name2];
  }

  ContactPairMultiplicityMetrics metrics;
  if (multiplicities.empty())
    return metrics;

  std::ostringstream labels;
  std::ostringstream counts;
  bool first = true;
  for (const auto& [label, count] : multiplicities) {
    if (!first) {
      labels << ';';
      counts << ';';
    }
    labels << label;
    counts << label << '=' << count;
    first = false;
  }
  metrics.pairLabels = labels.str();
  metrics.multiplicities = counts.str();
  return metrics;
}

void printHeader()
{
  std::printf(
      "step,time,scenario,solver,body,x,y,z,vx,vy,vz,up_z,"
      "contacts,exact_solves,warm_starts,fallbacks,residual,status\n");
}

void printResidualHistoryHeader()
{
  std::printf(
      "step,time,scenario,solver,solve_index,solve_contacts,"
      "outer_iteration,residual,"
      "primal_feasibility,dual_feasibility,complementarity,step_size,"
      "safe_step_size,coupling_variation_ratio,shrink_iterations,contacts,"
      "exact_solves,warm_starts,fallbacks,status\n");
}

void printPhaseSummaryHeader()
{
  std::printf(
      "step,time,scenario,solver,phase,card_count,projectile_count,"
      "finite_state,min_card_axis_up,min_center_height,"
      "max_card_horizontal_travel,max_projectile_speed,contacts,exact_solves,"
      "warm_starts,fallbacks,residual,status\n");
}

void printPerformanceHeader(SolverContract contract)
{
  std::printf(
      "step,time,scenario,solver,solver_contract,precision_contract,"
      "scene_contract,baumgarte_contract,collision_frontend,inner_local_solver,"
      "inner_sweeps_requested,fixed_inner_sweeps_requested,"
      "step_size_persistence_enabled,step_size_recovery_growth_factor,"
      "step_size_persistence_used,step_size_persistence_request,"
      "row_operator_request,row_operator_mode,wall_ms,"
      "requested_threads,actual_threads,contacts,unique_colliding_body_pairs,"
      "penetration_depth_min,penetration_depth_median,penetration_depth_p95,"
      "penetration_depth_max,exact_diagnostics_contract,step_exact_solves,"
      "step_warm_starts,step_exact_failures,step_fallbacks,"
      "step_fbf_iterations,residual,status,residual_primal_feasibility,"
      "residual_dual_feasibility,residual_complementarity,accepted_gamma,"
      "safe_gamma,shrink_iterations,coupling_variation_ratio,"
      "warm_start_matched_contacts,warm_start_matched_fraction,"
      "phase,card_count,projectile_count,finite_state,min_card_axis_up,"
      "min_center_height,max_card_horizontal_travel,max_projectile_speed,"
      "tracked_body,x,y,z,vx,vy,vz,up_z,max_outer_iterations,tolerance,"
      "accept_outer_max_iterations,inner_local_iterations,"
      "adaptive_step_size_enabled,warm_start_enabled,"
      "projected_gradient_retry_enabled,dense_residual_polish_enabled,"
      "fallback_to_boxed_lcp_enabled,diagonal_seed_enabled,"
      "matrix_free_seed_enabled,step_size_scale,outer_relaxation,"
      "initial_gamma_contract,split_impulse_enabled,"
      "step_contact_row_delassus_products,"
      "step_parallel_contact_row_delassus_products,"
      "max_contact_row_participants_to_date,"
      "exact_contact_row_logical_cpus_to_date,"
      "max_phase_contact_row_logical_cpus_to_date,"
      "max_card_center_displacement_from_initial,"
      "min_card_orientation_alignment_from_initial,"
      "projectile_card_contacts");
  if (contract == SolverContract::DartBestColoredBgs) {
    std::printf(
        ",inner_bgs_schedule_contract,last_exact_colored_bgs_used,"
        "last_exact_colored_bgs_solves,last_exact_colored_bgs_dispatches,"
        "last_exact_colored_bgs_max_participants,"
        "last_exact_colored_bgs_manifolds,last_exact_colored_bgs_colors,"
        "last_exact_colored_bgs_max_manifolds_per_color,"
        "exact_colored_bgs_logical_cpus,"
        "max_phase_exact_colored_bgs_logical_cpus,"
        "max_arch_body_displacement_from_initial,"
        "min_arch_body_orientation_alignment_from_initial");
  }
  if (nativeManifoldSensitivityEnabled()) {
    std::printf(
        ",manifold_sensitivity_contract,"
        "requested_native_contact_manifold_mode,"
        "actual_native_contact_manifold_mode,collision_max_contacts,"
        "collision_max_contacts_per_pair,"
        "step_exact_max_iterations_accepted,step_internal_fbf_status,"
        "step_internal_fbf_best_iteration,step_internal_fbf_best_residual,"
        "colliding_body_pair_labels,contact_multiplicity_by_body_pair");
  }
  if (crossStepPolicyEvidenceEnabled()) {
    std::printf(
        ",cross_step_policy_contract,requested_cross_step_policy,"
        "actual_cross_step_policy,requested_native_contact_manifold_mode,"
        "actual_native_contact_manifold_mode,collision_max_contacts,"
        "collision_max_contacts_per_pair,step_exact_attempts,"
        "step_exact_max_iterations_accepted,step_warm_start_gamma_caps,"
        "step_unconverged_warm_start_cache_skips,"
        "worst_exact_residual_to_date,last_exact_diagnostics_contract,"
        "last_exact_initial_natural_map_residual,"
        "last_exact_final_natural_map_residual,"
        "last_exact_uncapped_initial_gamma,"
        "last_exact_warm_start_gamma_cap_applied,warm_start_match_mode,"
        "warm_start_match_distance,warm_start_normal_cosine,"
        "strict_warm_start_match_distance,warm_start_max_age,"
        "persistent_gamma_safe_bound_scale,minimum_adaptive_gamma,"
        "maximum_adaptive_gamma,warm_start_gamma_natural_residual_threshold,"
        "warm_start_gamma_cap,persist_uncapped_gamma_after_warm_cap,"
        "require_residual_improvement_for_unconverged_cache_save,"
        "coupling_variation_tolerance,shrink_factor,"
        "max_step_shrink_iterations");
  }
  if (gAppendLiteralCrownImpactColumns) {
    std::printf(
        ",impact_contract,impact_phase,standing_prefix_reference_scenario,"
        "standing_prefix_comparable,final_gates_authoritative,"
        "preimpact_snapshot_captured,impact_projectile_count,"
        "step_projectile_arch_contacts,projectile_arch_contacts_to_date,"
        "step_projectile_ground_contacts,projectile_ground_contacts_to_date,"
        "first_projectile_arch_contact_step,"
        "first_projectile_arch_contact_time,"
        "first_projectile_ground_contact_step,"
        "first_projectile_ground_contact_time,exact_solves_to_date,"
        "exact_failures_to_date,boxed_fallbacks_to_date,"
        "max_iterations_accepted_to_date,finite_state_to_date,"
        "worst_exact_residual_to_date,preimpact_standing_gate,"
        "max_crown_displacement_from_preimpact,"
        "max_crown_displacement_to_date,"
        "max_arch_body_displacement_from_preimpact,"
        "min_arch_orientation_alignment_from_preimpact,"
        "max_far_field_displacement_from_preimpact,"
        "max_springer_displacement_from_preimpact,"
        "min_springer_orientation_alignment_from_preimpact,"
        "far_field_adjacent_pairs,projectile_contact_order_gate,"
        "impact_exact_gate,impact_residual_gate,impact_finite_gate,"
        "crown_response_gate,final_all_body_displacement_gate,"
        "final_orientation_gate,final_far_field_displacement_gate,"
        "final_springer_gate,final_far_field_adjacency_gate,"
        "final_impact_acceptance_gate");
  }
  std::printf("\n");
}

const char* phaseName(Scenario scenario, std::size_t completedStep)
{
  if (scenario != Scenario::CardHouseFourLevelSettleProjectileReduced
      && scenario != Scenario::CardHouseFourLevelSettleProjectileFull) {
    return "single_phase";
  }
  const std::size_t settleSteps
      = scenario == Scenario::CardHouseFourLevelSettleProjectileFull
            ? kCardHouseSettleProjectileFullSettleSteps
            : kCardHouseSettleProjectileSettleSteps;
  if (completedStep == 0u)
    return "initial";
  if (completedStep <= settleSteps)
    return "settle";
  return "projectile";
}

PhaseSummaryMetrics collectPhaseSummaryMetrics(
    const std::shared_ptr<dart::simulation::World>& world,
    const InitialCardPoses& initialCardPoses = {})
{
  PhaseSummaryMetrics metrics;
  for (std::size_t i = 0u; i < world->getNumSkeletons(); ++i) {
    const auto skeleton = world->getSkeleton(i);
    if (skeleton == nullptr || skeleton->getNumBodyNodes() == 0u)
      continue;

    const auto* body = skeleton->getBodyNode(0);
    const Eigen::Isometry3d transform = body->getWorldTransform();
    const Eigen::Vector3d position = transform.translation();
    metrics.finiteState = metrics.finiteState && position.allFinite()
                          && body->getLinearVelocity().allFinite()
                          && body->getAngularVelocity().allFinite();

    if (isCardHouseCardSkeletonName(skeleton->getName())) {
      ++metrics.cardCount;
      metrics.minCardAxisUp = std::min(
          metrics.minCardAxisUp,
          transform.linear().col(2).normalized().dot(Eigen::Vector3d::UnitZ()));
      metrics.minCenterHeight = std::min(metrics.minCenterHeight, position.z());
      metrics.maxCardHorizontalTravel = std::max(
          metrics.maxCardHorizontalTravel, position.head<2>().norm());
      const auto initialPose = std::find_if(
          initialCardPoses.begin(),
          initialCardPoses.end(),
          [body](const InitialCardPose& pose) { return pose.body == body; });
      if (initialPose != initialCardPoses.end()) {
        metrics.maxCardCenterDisplacementFromInitial = std::max(
            metrics.maxCardCenterDisplacementFromInitial,
            (position - initialPose->transform.translation()).norm());
        const Eigen::Matrix3d relativeRotation
            = initialPose->transform.linear().transpose() * transform.linear();
        const double orientationAlignment
            = std::clamp(0.5 * (relativeRotation.trace() - 1.0), -1.0, 1.0);
        metrics.minCardOrientationAlignmentFromInitial = std::min(
            metrics.minCardOrientationAlignmentFromInitial,
            orientationAlignment);
      } else if (!initialCardPoses.empty()) {
        metrics.finiteState = false;
      }
    } else if (
        isCardHouseProjectileSkeletonName(skeleton->getName())
        || isMasonryArchProjectileSkeletonName(skeleton->getName())) {
      ++metrics.projectileCount;
      metrics.maxProjectileSpeed = std::max(
          metrics.maxProjectileSpeed, body->getLinearVelocity().norm());
    }
  }

  if (metrics.cardCount == 0u) {
    metrics.minCardAxisUp = std::numeric_limits<double>::quiet_NaN();
    metrics.minCenterHeight = std::numeric_limits<double>::quiet_NaN();
    metrics.maxCardCenterDisplacementFromInitial
        = std::numeric_limits<double>::quiet_NaN();
    metrics.minCardOrientationAlignmentFromInitial
        = std::numeric_limits<double>::quiet_NaN();
  } else if (initialCardPoses.empty()) {
    metrics.maxCardCenterDisplacementFromInitial
        = std::numeric_limits<double>::quiet_NaN();
    metrics.minCardOrientationAlignmentFromInitial
        = std::numeric_limits<double>::quiet_NaN();
  }

  const auto& collisionResult
      = world->getConstraintSolver()->getLastCollisionResult();
  for (const auto& contact : collisionResult.getContacts()) {
    const auto body1 = contact.getBodyNodePtr1();
    const auto body2 = contact.getBodyNodePtr2();
    if (body1 == nullptr || body2 == nullptr)
      continue;
    const std::string& skeleton1 = body1->getSkeleton()->getName();
    const std::string& skeleton2 = body2->getSkeleton()->getName();
    const bool firstProjectileSecondCard
        = isCardHouseProjectileSkeletonName(skeleton1)
          && isCardHouseCardSkeletonName(skeleton2);
    const bool secondProjectileFirstCard
        = isCardHouseProjectileSkeletonName(skeleton2)
          && isCardHouseCardSkeletonName(skeleton1);
    if (firstProjectileSecondCard || secondProjectileFirstCard)
      ++metrics.projectileCardContacts;
  }

  return metrics;
}

InitialCardPoses collectInitialCardPoses(
    const std::shared_ptr<dart::simulation::World>& world)
{
  InitialCardPoses poses;
  for (std::size_t i = 0u; i < world->getNumSkeletons(); ++i) {
    const auto skeleton = world->getSkeleton(i);
    if (skeleton == nullptr || skeleton->getNumBodyNodes() == 0u
        || !isCardHouseCardSkeletonName(skeleton->getName())) {
      continue;
    }
    const auto* body = skeleton->getBodyNode(0);
    poses.push_back(InitialCardPose{body, body->getWorldTransform()});
  }
  return poses;
}

InitialArchPoses collectInitialArchPoses(
    const std::shared_ptr<dart::simulation::World>& world)
{
  InitialArchPoses poses;
  for (std::size_t i = 0u; i < world->getNumSkeletons(); ++i) {
    const auto skeleton = world->getSkeleton(i);
    if (skeleton == nullptr
        || skeleton->getName().rfind("masonry_arch_stone_", 0u) != 0u) {
      continue;
    }
    for (std::size_t j = 0u; j < skeleton->getNumBodyNodes(); ++j) {
      const auto* body = skeleton->getBodyNode(j);
      poses.push_back(InitialArchPose{body, body->getWorldTransform()});
    }
  }
  return poses;
}

ArchOutcomeMetrics collectArchOutcomeMetrics(
    const InitialArchPoses& initialArchPoses)
{
  ArchOutcomeMetrics metrics;
  metrics.bodyCount = initialArchPoses.size();
  for (const auto& pose : initialArchPoses) {
    if (pose.body == nullptr) {
      metrics.finiteState = false;
      continue;
    }
    const Eigen::Isometry3d transform = pose.body->getWorldTransform();
    const Eigen::Vector3d displacement
        = transform.translation() - pose.transform.translation();
    const Eigen::Matrix3d relativeRotation
        = pose.transform.linear().transpose() * transform.linear();
    if (!transform.matrix().allFinite() || !displacement.allFinite()
        || !relativeRotation.allFinite()) {
      metrics.finiteState = false;
      continue;
    }
    metrics.maxBodyDisplacementFromInitial
        = std::max(metrics.maxBodyDisplacementFromInitial, displacement.norm());
    const double orientationAlignment
        = std::clamp(0.5 * (relativeRotation.trace() - 1.0), -1.0, 1.0);
    metrics.minBodyOrientationAlignmentFromInitial = std::min(
        metrics.minBodyOrientationAlignmentFromInitial, orientationAlignment);
  }
  if (metrics.bodyCount == 0u || !metrics.finiteState) {
    metrics.maxBodyDisplacementFromInitial
        = std::numeric_limits<double>::quiet_NaN();
    metrics.minBodyOrientationAlignmentFromInitial
        = std::numeric_limits<double>::quiet_NaN();
  }
  return metrics;
}

bool isLiteralCrownImpactProjectileSkeletonName(const std::string& name)
{
  return name.rfind("masonry_arch_projectile_impact_v1_", 0u) == 0u;
}

std::size_t masonryArchStoneIndex(const dart::dynamics::BodyNode* body)
{
  constexpr const char* kPrefix = "masonry_arch_stone_";
  constexpr std::size_t kPrefixLength = 19u;
  if (body == nullptr || body->getSkeleton() == nullptr)
    return std::numeric_limits<std::size_t>::max();

  const std::string& name = body->getSkeleton()->getName();
  if (name.rfind(kPrefix, 0u) != 0u || name.size() == kPrefixLength)
    return std::numeric_limits<std::size_t>::max();

  errno = 0;
  char* end = nullptr;
  const char* value = name.c_str() + kPrefixLength;
  const unsigned long long parsed = std::strtoull(value, &end, 10);
  if (errno == ERANGE || end == value || *end != '\0'
      || parsed > std::numeric_limits<std::size_t>::max()) {
    return std::numeric_limits<std::size_t>::max();
  }
  return static_cast<std::size_t>(parsed);
}

std::size_t countLiteralCrownImpactProjectiles(
    const std::shared_ptr<dart::simulation::World>& world)
{
  std::size_t count = 0u;
  for (std::size_t i = 0u; i < world->getNumSkeletons(); ++i) {
    const auto skeleton = world->getSkeleton(i);
    if (skeleton != nullptr
        && isLiteralCrownImpactProjectileSkeletonName(skeleton->getName())) {
      ++count;
    }
  }
  return count;
}

bool hasFrozenLiteralArchMobility(
    const std::shared_ptr<dart::simulation::World>& world)
{
  std::set<std::size_t> indices;
  for (std::size_t i = 0u; i < world->getNumSkeletons(); ++i) {
    const auto skeleton = world->getSkeleton(i);
    if (skeleton == nullptr || skeleton->getNumBodyNodes() != 1u)
      continue;
    const std::size_t index = masonryArchStoneIndex(skeleton->getBodyNode(0));
    if (index == std::numeric_limits<std::size_t>::max())
      continue;
    if (index >= kArchStoneCount || !indices.insert(index).second)
      return false;
    const bool expectedMobile = index != 0u && index + 1u != kArchStoneCount;
    if (skeleton->isMobile() != expectedMobile)
      return false;
  }
  return indices.size() == kArchStoneCount;
}

LiteralCrownImpactPoseMetrics collectLiteralCrownImpactPoseMetrics(
    const InitialArchPoses& preImpactArchPoses)
{
  LiteralCrownImpactPoseMetrics metrics;
  if (preImpactArchPoses.size() != kArchStoneCount)
    return metrics;

  metrics.finiteState = true;
  metrics.maxBodyDisplacement = 0.0;
  metrics.minOrientationAlignment = 1.0;
  metrics.maxCrownDisplacement = 0.0;
  metrics.maxFarFieldDisplacement = 0.0;
  metrics.maxSpringerDisplacement = 0.0;
  metrics.minSpringerOrientationAlignment = 1.0;
  std::set<std::size_t> indices;
  for (const auto& pose : preImpactArchPoses) {
    const std::size_t index = masonryArchStoneIndex(pose.body);
    if (pose.body == nullptr || index >= kArchStoneCount
        || !indices.insert(index).second) {
      metrics.finiteState = false;
      break;
    }
    const Eigen::Isometry3d transform = pose.body->getWorldTransform();
    const Eigen::Vector3d displacement
        = transform.translation() - pose.transform.translation();
    const Eigen::Matrix3d relativeRotation
        = pose.transform.linear().transpose() * transform.linear();
    if (!transform.matrix().allFinite() || !displacement.allFinite()
        || !relativeRotation.allFinite()
        || !pose.body->getLinearVelocity().allFinite()
        || !pose.body->getAngularVelocity().allFinite()) {
      metrics.finiteState = false;
      break;
    }
    const double translation = displacement.norm();
    const double alignment
        = std::clamp(0.5 * (relativeRotation.trace() - 1.0), -1.0, 1.0);
    metrics.maxBodyDisplacement
        = std::max(metrics.maxBodyDisplacement, translation);
    metrics.minOrientationAlignment
        = std::min(metrics.minOrientationAlignment, alignment);
    if (index >= kLiteralCrownImpactFirstCentralStone
        && index <= kLiteralCrownImpactLastCentralStone) {
      metrics.maxCrownDisplacement
          = std::max(metrics.maxCrownDisplacement, translation);
    } else {
      metrics.maxFarFieldDisplacement
          = std::max(metrics.maxFarFieldDisplacement, translation);
    }
    if (index == 0u || index + 1u == kArchStoneCount) {
      metrics.maxSpringerDisplacement
          = std::max(metrics.maxSpringerDisplacement, translation);
      metrics.minSpringerOrientationAlignment
          = std::min(metrics.minSpringerOrientationAlignment, alignment);
    }
  }

  if (indices.size() != kArchStoneCount)
    metrics.finiteState = false;
  if (!metrics.finiteState) {
    metrics.maxBodyDisplacement = std::numeric_limits<double>::quiet_NaN();
    metrics.minOrientationAlignment = std::numeric_limits<double>::quiet_NaN();
    metrics.maxCrownDisplacement = std::numeric_limits<double>::quiet_NaN();
    metrics.maxFarFieldDisplacement = std::numeric_limits<double>::quiet_NaN();
    metrics.maxSpringerDisplacement = std::numeric_limits<double>::quiet_NaN();
    metrics.minSpringerOrientationAlignment
        = std::numeric_limits<double>::quiet_NaN();
  }
  return metrics;
}

std::size_t countLiteralCrownImpactFarFieldAdjacentPairs(
    const std::shared_ptr<dart::simulation::World>& world)
{
  std::set<std::pair<std::size_t, std::size_t>> pairs;
  const auto& collisionResult
      = world->getConstraintSolver()->getLastCollisionResult();
  for (const auto& contact : collisionResult.getContacts()) {
    const auto body1 = contact.getBodyNodePtr1();
    const auto body2 = contact.getBodyNodePtr2();
    const std::size_t index1 = masonryArchStoneIndex(body1.get());
    const std::size_t index2 = masonryArchStoneIndex(body2.get());
    if (index1 >= kArchStoneCount || index2 >= kArchStoneCount
        || index1 == index2) {
      continue;
    }
    const auto pair = std::minmax(index1, index2);
    if (pair.second != pair.first + 1u)
      continue;
    const bool leftFarField = pair.second <= 8u;
    const bool rightFarField = pair.first >= 16u;
    if (leftFarField || rightFarField)
      pairs.emplace(pair.first, pair.second);
  }
  return pairs.size();
}

bool literalCrownImpactWorldStateFinite(
    const std::shared_ptr<dart::simulation::World>& world)
{
  for (std::size_t i = 0u; i < world->getNumSkeletons(); ++i) {
    const auto skeleton = world->getSkeleton(i);
    if (skeleton == nullptr)
      continue;
    for (std::size_t j = 0u; j < skeleton->getNumBodyNodes(); ++j) {
      const auto* body = skeleton->getBodyNode(j);
      if (body == nullptr || !body->getWorldTransform().matrix().allFinite()
          || !body->getLinearVelocity().allFinite()
          || !body->getAngularVelocity().allFinite()) {
        return false;
      }
    }
  }
  return true;
}

void captureLiteralCrownImpactPreImpactPoses(
    const std::shared_ptr<dart::simulation::World>& world,
    const ExactCounterSnapshot& counters,
    LiteralCrownImpactTraceState& state)
{
  state.preImpactArchPoses = collectInitialArchPoses(world);
  state.exactSolvesAtImpactLaunch = counters.exactSolves;
  state.preImpactSnapshotCaptured
      = state.preImpactArchPoses.size() == kArchStoneCount;
  state.poseMetrics
      = collectLiteralCrownImpactPoseMetrics(state.preImpactArchPoses);
}

void updateLiteralCrownImpactTraceState(
    std::size_t completedStep,
    const std::shared_ptr<dart::simulation::World>& world,
    const dart::constraint::ExactCoulombFbfConstraintSolver* exactSolver,
    const InitialArchPoses& initialArchPoses,
    const ExactCounterSnapshot& counters,
    LiteralCrownImpactTraceState& state)
{
  state.projectileCount = countLiteralCrownImpactProjectiles(world);
  state.finiteStateToDate
      = state.finiteStateToDate && literalCrownImpactWorldStateFinite(world);
  if (completedStep <= kLiteralCrownImpactLaunchAfterSteps) {
    state.prefixProjectileFreeToDate
        = state.prefixProjectileFreeToDate && state.projectileCount == 0u;
  }

  state.stepProjectileArchContacts = 0u;
  state.stepProjectileGroundContacts = 0u;
  const auto& collisionResult
      = world->getConstraintSolver()->getLastCollisionResult();
  for (const auto& contact : collisionResult.getContacts()) {
    const auto body1 = contact.getBodyNodePtr1();
    const auto body2 = contact.getBodyNodePtr2();
    if (body1 == nullptr || body2 == nullptr)
      continue;
    const std::string& name1 = body1->getSkeleton()->getName();
    const std::string& name2 = body2->getSkeleton()->getName();
    const bool projectile1 = isLiteralCrownImpactProjectileSkeletonName(name1);
    const bool projectile2 = isLiteralCrownImpactProjectileSkeletonName(name2);
    const bool arch1 = masonryArchStoneIndex(body1.get()) < kArchStoneCount;
    const bool arch2 = masonryArchStoneIndex(body2.get()) < kArchStoneCount;
    const bool ground1 = name1 == "ground_plane";
    const bool ground2 = name2 == "ground_plane";
    if ((projectile1 && arch2) || (projectile2 && arch1))
      ++state.stepProjectileArchContacts;
    if ((projectile1 && ground2) || (projectile2 && ground1))
      ++state.stepProjectileGroundContacts;
  }
  state.projectileArchContactsToDate += state.stepProjectileArchContacts;
  state.projectileGroundContactsToDate += state.stepProjectileGroundContacts;
  if (state.firstProjectileArchContactStep == 0u
      && state.stepProjectileArchContacts > 0u) {
    state.firstProjectileArchContactStep = completedStep;
  }
  if (state.firstProjectileGroundContactStep == 0u
      && state.stepProjectileGroundContacts > 0u) {
    state.firstProjectileGroundContactStep = completedStep;
  }

  if (state.preImpactSnapshotCaptured) {
    state.poseMetrics
        = collectLiteralCrownImpactPoseMetrics(state.preImpactArchPoses);
    if (std::isfinite(state.poseMetrics.maxCrownDisplacement)) {
      state.maximumCrownDisplacementToDate = std::max(
          state.maximumCrownDisplacementToDate,
          state.poseMetrics.maxCrownDisplacement);
    }
    state.finiteStateToDate
        = state.finiteStateToDate && state.poseMetrics.finiteState;
    state.farFieldAdjacentPairs
        = countLiteralCrownImpactFarFieldAdjacentPairs(world);
  }

  if (completedStep == kLiteralCrownImpactLaunchAfterSteps) {
    const auto initialMetrics = collectArchOutcomeMetrics(initialArchPoses);
    const auto collisionMetrics = collectCollisionMetrics(world);
    const std::size_t contacts = collisionResult.getNumContacts();
    const double worstResidual
        = exactSolver == nullptr ? std::numeric_limits<double>::quiet_NaN()
                                 : exactSolver->getWorstExactCoulombResidual();
    const bool exactStanding
        = exactSolver != nullptr && counters.exactSolves > 0u
          && counters.exactFailures == 0u && counters.boxedFallbacks == 0u
          && counters.maxIterationsAccepted == 0u
          && std::isfinite(worstResidual) && worstResidual <= 1e-6;
    const bool stableStanding
        = initialMetrics.bodyCount == kArchStoneCount
          && initialMetrics.finiteState
          && std::isfinite(initialMetrics.maxBodyDisplacementFromInitial)
          && initialMetrics.maxBodyDisplacementFromInitial <= 0.001
          && std::isfinite(
              initialMetrics.minBodyOrientationAlignmentFromInitial)
          && initialMetrics.minBodyOrientationAlignmentFromInitial >= 0.999;
    state.preImpactStandingGate
        = state.prefixProjectileFreeToDate && state.finiteStateToDate
                  && hasFrozenLiteralArchMobility(world) && exactStanding
                  && stableStanding && contacts == 96u
                  && collisionMetrics.uniqueBodyPairs == 24u
              ? 1
              : 0;
  }
}

bool literalCrownImpactContactOrderGate(
    const LiteralCrownImpactTraceState& state)
{
  return state.firstProjectileArchContactStep > 0u
         && (state.firstProjectileGroundContactStep == 0u
             || state.firstProjectileArchContactStep
                    < state.firstProjectileGroundContactStep);
}

bool literalCrownImpactExactGate(
    const LiteralCrownImpactTraceState& state,
    const ExactCounterSnapshot& counters)
{
  return state.preImpactSnapshotCaptured
         && counters.exactSolves > state.exactSolvesAtImpactLaunch
         && counters.exactFailures == 0u && counters.boxedFallbacks == 0u
         && counters.maxIterationsAccepted == 0u;
}

bool literalCrownImpactResidualGate(
    const dart::constraint::ExactCoulombFbfConstraintSolver* exactSolver)
{
  if (exactSolver == nullptr)
    return false;
  const double residual = exactSolver->getWorstExactCoulombResidual();
  return std::isfinite(residual) && residual <= 1e-6;
}

bool literalCrownImpactFinalAcceptanceGate(
    std::size_t completedStep,
    const LiteralCrownImpactTraceState& state,
    const ExactCounterSnapshot& counters,
    const dart::constraint::ExactCoulombFbfConstraintSolver* exactSolver)
{
  const auto& pose = state.poseMetrics;
  return completedStep == kLiteralCrownImpactDefaultSteps
         && state.preImpactSnapshotCaptured && state.preImpactStandingGate == 1
         && state.projectileCount == kLiteralCrownImpactProjectileCount
         && literalCrownImpactContactOrderGate(state)
         && literalCrownImpactExactGate(state, counters)
         && literalCrownImpactResidualGate(exactSolver)
         && state.finiteStateToDate && pose.finiteState
         && state.maximumCrownDisplacementToDate
                >= kLiteralCrownImpactMinimumCrownResponse
         && pose.maxBodyDisplacement
                <= kLiteralCrownImpactMaximumBodyDisplacement
         && pose.minOrientationAlignment
                >= kLiteralCrownImpactMinimumOrientationAlignment
         && pose.maxFarFieldDisplacement
                <= kLiteralCrownImpactMaximumFarFieldDisplacement
         && pose.maxSpringerDisplacement <= kLiteralCrownImpactSpringerTolerance
         && pose.minSpringerOrientationAlignment
                >= 1.0 - kLiteralCrownImpactSpringerTolerance
         && state.farFieldAdjacentPairs
                == kLiteralCrownImpactFarFieldAdjacentPairCount;
}

TrackedBodyMetrics collectTrackedBodyMetrics(
    const std::shared_ptr<dart::simulation::World>& world, Scenario scenario)
{
  TrackedBodyMetrics metrics;
  const auto skeleton = world->getSkeleton(bodyNameForScenario(scenario));
  if (skeleton == nullptr || skeleton->getNumBodyNodes() == 0u)
    return metrics;

  const auto* body = skeleton->getBodyNode(0);
  const Eigen::Isometry3d transform = body->getWorldTransform();
  const Eigen::Vector3d position = transform.translation();
  const Eigen::Vector3d velocity = body->getLinearVelocity();
  metrics.name = body->getName();
  metrics.x = position.x();
  metrics.y = position.y();
  metrics.z = position.z();
  metrics.vx = velocity.x();
  metrics.vy = velocity.y();
  metrics.vz = velocity.z();
  metrics.upZ
      = transform.linear().col(2).normalized().dot(Eigen::Vector3d::UnitZ());
  return metrics;
}

ExactCounterSnapshot readExactCounterSnapshot(
    const dart::constraint::ExactCoulombFbfConstraintSolver* exactSolver)
{
  ExactCounterSnapshot snapshot;
  if (exactSolver == nullptr)
    return snapshot;

  snapshot.exactAttempts = exactSolver->getNumExactCoulombAttempts();
  snapshot.exactSolves = exactSolver->getNumExactCoulombSolves();
  snapshot.warmStarts = exactSolver->getNumExactCoulombWarmStarts();
  snapshot.exactFailures = exactSolver->getNumExactCoulombFailures();
  snapshot.boxedFallbacks = exactSolver->getNumBoxedLcpFallbacks();
  snapshot.maxIterationsAccepted
      = exactSolver->getNumExactCoulombMaxIterationsAccepted();
  snapshot.warmStartStepSizeCaps
      = exactSolver->getNumExactCoulombWarmStartStepSizeCaps();
  snapshot.unconvergedCacheSkips
      = exactSolver->getNumExactCoulombUnconvergedCacheSkips();
  snapshot.fbfIterations = exactSolver->getTotalExactCoulombIterations();
  snapshot.contactRowDelassusProducts
      = exactSolver->getNumExactCoulombContactRowDelassusProducts();
  snapshot.parallelContactRowDelassusProducts
      = exactSolver->getNumExactCoulombParallelContactRowDelassusProducts();
  return snapshot;
}

std::size_t counterDelta(std::size_t current, std::size_t previous)
{
  return current >= previous ? current - previous : 0u;
}

const char* baumgarteContractLabel(
    const std::shared_ptr<dart::simulation::World>& world)
{
  const auto* solver = world->getConstraintSolver();
  if (solver != nullptr && solver->isSplitImpulseEnabled()) {
    if (dart::constraint::ContactConstraint::getErrorReductionParameter()
        == 0.0) {
      return "split_impulse_no_velocity_baumgarte_erp_zero_vs_paper";
    }
    return "split_impulse_no_velocity_baumgarte_vs_paper";
  }

  return "dart_velocity_baumgarte_author_parameters_unavailable";
}

void printPerformanceRow(
    std::size_t step,
    Scenario scenario,
    SolverMode solverMode,
    SolverContract contract,
    CollisionFrontend collisionFrontend,
    std::size_t requestedThreads,
    double wallMilliseconds,
    const ExactCounterSnapshot& previousCounters,
    const ExactCounterSnapshot& currentCounters,
    const std::shared_ptr<dart::simulation::World>& world,
    const dart::constraint::ExactCoulombFbfConstraintSolver* exactSolver,
    const InitialCardPoses& initialCardPoses,
    const InitialArchPoses& initialArchPoses,
    const LiteralCrownImpactTraceState* literalCrownImpactState)
{
  const auto phaseMetrics = collectPhaseSummaryMetrics(world, initialCardPoses);
  const auto archOutcomeMetrics = collectArchOutcomeMetrics(initialArchPoses);
  const auto trackedMetrics = collectTrackedBodyMetrics(world, scenario);
  const auto collisionMetrics = collectCollisionMetrics(world);
  const auto contacts
      = world->getConstraintSolver()->getLastCollisionResult().getNumContacts();
  const char* innerLocalSolver = "not_applicable";
  int innerSweepsRequested = -1;
  int fixedInnerSweepsRequested = -1;
  int stepSizePersistenceEnabled = -1;
  double stepSizeRecoveryGrowthFactor
      = std::numeric_limits<double>::quiet_NaN();
  const char* rowOperatorRequest = "not_applicable";
  int maxOuterIterations = -1;
  double tolerance = std::numeric_limits<double>::quiet_NaN();
  int acceptOuterMaxIterations = -1;
  int innerLocalIterations = -1;
  int adaptiveStepSizeEnabled = -1;
  int warmStartEnabled = -1;
  int projectedGradientRetryEnabled = -1;
  int denseResidualPolishEnabled = -1;
  int fallbackToBoxedLcpEnabled = -1;
  int diagonalSeedEnabled = -1;
  int matrixFreeSeedEnabled = -1;
  double stepSizeScale = std::numeric_limits<double>::quiet_NaN();
  double outerRelaxation = std::numeric_limits<double>::quiet_NaN();
  const char* initialGammaContract = "not_applicable";
  if (exactSolver != nullptr) {
    const auto& options = exactSolver->getExactCoulombOptions();
    innerLocalSolver = localSolverName(options.innerLocalSolver);
    innerSweepsRequested = options.innerMaxSweeps;
    fixedInnerSweepsRequested = options.runFixedInnerSweeps ? 1 : 0;
    stepSizePersistenceEnabled = options.enableStepSizePersistence ? 1 : 0;
    stepSizeRecoveryGrowthFactor = options.stepSizeRecoveryGrowthFactor;
    rowOperatorRequest = rowOperatorRequestName(options);
    maxOuterIterations = options.maxOuterIterations;
    tolerance = options.tolerance;
    acceptOuterMaxIterations = options.acceptOuterMaxIterations ? 1 : 0;
    innerLocalIterations = options.innerLocalIterations;
    adaptiveStepSizeEnabled = options.enableAdaptiveStepSize ? 1 : 0;
    warmStartEnabled = options.enableWarmStart ? 1 : 0;
    projectedGradientRetryEnabled
        = options.enableProjectedGradientRetry ? 1 : 0;
    denseResidualPolishEnabled = options.enableDenseResidualPolish ? 1 : 0;
    fallbackToBoxedLcpEnabled = options.fallbackToBoxedLcp ? 1 : 0;
    diagonalSeedEnabled = options.seedNormalImpulseFromDiagonal ? 1 : 0;
    matrixFreeSeedEnabled = options.useMatrixFreeDelassusSeed ? 1 : 0;
    stepSizeScale = options.stepSizeScale;
    outerRelaxation = options.outerRelaxation;
    initialGammaContract = initialGammaContractName(options);
  }
  const int splitImpulseEnabled
      = world->getConstraintSolver()->isSplitImpulseEnabled() ? 1 : 0;
  const auto stepExactSolves
      = counterDelta(currentCounters.exactSolves, previousCounters.exactSolves);
  const auto stepWarmStarts
      = counterDelta(currentCounters.warmStarts, previousCounters.warmStarts);
  const auto stepExactFailures = counterDelta(
      currentCounters.exactFailures, previousCounters.exactFailures);
  const auto stepFallbacks = counterDelta(
      currentCounters.boxedFallbacks, previousCounters.boxedFallbacks);
  const auto stepFbfIterations = counterDelta(
      currentCounters.fbfIterations, previousCounters.fbfIterations);
  const auto stepMaxIterationsAccepted = counterDelta(
      currentCounters.maxIterationsAccepted,
      previousCounters.maxIterationsAccepted);
  const auto stepContactRowDelassusProducts = counterDelta(
      currentCounters.contactRowDelassusProducts,
      previousCounters.contactRowDelassusProducts);
  const auto stepParallelContactRowDelassusProducts = counterDelta(
      currentCounters.parallelContactRowDelassusProducts,
      previousCounters.parallelContactRowDelassusProducts);
  const auto maxContactRowParticipantsToDate
      = exactSolver == nullptr
            ? 0u
            : exactSolver->getMaxExactCoulombContactRowParticipants();
  const std::string exactContactRowLogicalCpusToDate
      = exactSolver == nullptr
            ? "none"
            : formatLogicalCpuIds(
                exactSolver->getExactCoulombContactRowLogicalCpuIds());
  const std::string maxPhaseContactRowLogicalCpusToDate
      = exactSolver == nullptr
            ? "none"
            : formatLogicalCpuIds(
                exactSolver->getMaxExactCoulombPhaseContactRowLogicalCpuIds());
  double residual = std::numeric_limits<double>::quiet_NaN();
  double residualPrimal = std::numeric_limits<double>::quiet_NaN();
  double residualDual = std::numeric_limits<double>::quiet_NaN();
  double residualComplementarity = std::numeric_limits<double>::quiet_NaN();
  double acceptedGamma = std::numeric_limits<double>::quiet_NaN();
  double safeGamma = std::numeric_limits<double>::quiet_NaN();
  int shrinkIterations = -1;
  double couplingVariationRatio = std::numeric_limits<double>::quiet_NaN();
  std::int64_t warmStartMatchedContacts = -1;
  double warmStartMatchedFraction = std::numeric_limits<double>::quiet_NaN();
  int stepSizePersistenceUsed = -1;
  double stepSizePersistenceRequest = std::numeric_limits<double>::quiet_NaN();
  const char* rowOperatorMode
      = exactSolver == nullptr ? "boxed_lcp" : "not_run";
  const char* exactDiagnosticsContract = "unavailable_boxed_lcp";
  const char* status = "boxed_lcp";
  if (exactSolver != nullptr) {
    if (stepExactSolves > 0u || stepExactFailures > 0u) {
      residual = exactSolver->getLastExactCoulombResidual();
      const auto& residualDetails
          = exactSolver->getLastExactCoulombResidualDetails();
      residualPrimal = residualDetails.primalFeasibility;
      residualDual = residualDetails.dualFeasibility;
      residualComplementarity = residualDetails.complementarity;
      acceptedGamma = exactSolver->getLastExactCoulombStepSize();
      safeGamma = exactSolver->getLastExactCoulombSafeStepSize();
      shrinkIterations = exactSolver->getLastExactCoulombShrinkIterations();
      couplingVariationRatio
          = exactSolver->getLastExactCoulombCouplingVariationRatio();
      warmStartMatchedContacts = static_cast<std::int64_t>(
          exactSolver->getLastExactCoulombWarmStartMatchedContacts());
      stepSizePersistenceUsed
          = exactSolver->getLastExactCoulombPersistentStepSizeUsed() ? 1 : 0;
      stepSizePersistenceRequest
          = exactSolver->getLastExactCoulombPersistentStepSizeRequest();
      if (exactSolver->getLastExactCoulombContactRowOperatorUsed()) {
        rowOperatorMode
            = exactSolver->getLastExactCoulombDenseContactRowSnapshotAssembled()
                  ? "contact_row_with_dense_snapshot"
                  : "contact_row_no_dense_snapshot";
      } else if (exactSolver
                     ->getLastExactCoulombDenseContactRowSnapshotAssembled()) {
        rowOperatorMode = "dense_delassus_snapshot";
      } else {
        rowOperatorMode = "dense_impulse_or_unknown";
      }
      if (contacts > 0u) {
        warmStartMatchedFraction = static_cast<double>(warmStartMatchedContacts)
                                   / static_cast<double>(contacts);
      }
      const std::size_t exactAttempts = stepExactSolves + stepExactFailures;
      if (exactAttempts > 1u) {
        exactDiagnosticsContract
            = "last_exact_group_only_multi_group_noncomparable";
      } else if (
          exactSolver->getLastExactCoulombContactRowOperatorUsed()
          && !exactSolver
                  ->getLastExactCoulombDenseContactRowSnapshotAssembled()) {
        exactDiagnosticsContract
            = "last_exact_group_public_getters_contact_row_no_dense_snapshot_"
              "warm_fraction_over_step_contacts";
      } else if (exactSolver
                     ->getLastExactCoulombDenseContactRowSnapshotAssembled()) {
        exactDiagnosticsContract
            = "last_exact_group_public_getters_contact_row_dense_snapshot_"
              "warm_fraction_over_step_contacts";
      } else {
        exactDiagnosticsContract
            = "last_exact_group_public_getters_dense_impulse_or_unknown_"
              "warm_fraction_over_step_contacts";
      }
      status = exactStatusName(exactSolver->getLastExactCoulombStatus());
    } else {
      exactDiagnosticsContract = "unavailable_no_exact_group_this_step";
      status = "no_exact_group";
    }
  }

  std::ostringstream row;
  row << std::setprecision(17) << step << ',' << world->getTime() << ','
      << scenarioName(scenario) << ',' << solverName(solverMode) << ','
      << solverContractName(contract, requestedThreads) << ','
      << precisionContractLabel(contract) << ','
      << sceneContractLabel(scenario, collisionFrontend) << ','
      << baumgarteContractLabel(world) << ','
      << collisionFrontendName(collisionFrontend) << ',' << innerLocalSolver
      << ',' << innerSweepsRequested << ',' << fixedInnerSweepsRequested << ','
      << stepSizePersistenceEnabled << ','
      << safeValue(stepSizeRecoveryGrowthFactor) << ','
      << stepSizePersistenceUsed << ',' << safeValue(stepSizePersistenceRequest)
      << ',' << rowOperatorRequest << ',' << rowOperatorMode << ','
      << wallMilliseconds << ',' << requestedThreads << ','
      << world->getNumSimulationThreads() << ',' << contacts << ','
      << collisionMetrics.uniqueBodyPairs << ','
      << safeValue(collisionMetrics.penetrationDepthMin) << ','
      << safeValue(collisionMetrics.penetrationDepthMedian) << ','
      << safeValue(collisionMetrics.penetrationDepthP95) << ','
      << safeValue(collisionMetrics.penetrationDepthMax) << ','
      << exactDiagnosticsContract << ',' << stepExactSolves << ','
      << stepWarmStarts << ',' << stepExactFailures << ',' << stepFallbacks
      << ',' << stepFbfIterations << ',' << safeValue(residual) << ',' << status
      << ',' << safeValue(residualPrimal) << ',' << safeValue(residualDual)
      << ',' << safeValue(residualComplementarity) << ','
      << safeValue(acceptedGamma) << ',' << safeValue(safeGamma) << ','
      << shrinkIterations << ',' << safeValue(couplingVariationRatio) << ','
      << warmStartMatchedContacts << ',' << safeValue(warmStartMatchedFraction)
      << ',' << phaseName(scenario, step) << ',' << phaseMetrics.cardCount
      << ',' << phaseMetrics.projectileCount << ','
      << (phaseMetrics.finiteState ? 1 : 0) << ','
      << safeValue(phaseMetrics.minCardAxisUp) << ','
      << safeValue(phaseMetrics.minCenterHeight) << ','
      << safeValue(phaseMetrics.maxCardHorizontalTravel) << ','
      << safeValue(phaseMetrics.maxProjectileSpeed) << ','
      << trackedMetrics.name << ',' << safeValue(trackedMetrics.x) << ','
      << safeValue(trackedMetrics.y) << ',' << safeValue(trackedMetrics.z)
      << ',' << safeValue(trackedMetrics.vx) << ','
      << safeValue(trackedMetrics.vy) << ',' << safeValue(trackedMetrics.vz)
      << ',' << safeValue(trackedMetrics.upZ) << ',' << maxOuterIterations
      << ',' << safeValue(tolerance) << ',' << acceptOuterMaxIterations << ','
      << innerLocalIterations << ',' << adaptiveStepSizeEnabled << ','
      << warmStartEnabled << ',' << projectedGradientRetryEnabled << ','
      << denseResidualPolishEnabled << ',' << fallbackToBoxedLcpEnabled << ','
      << diagonalSeedEnabled << ',' << matrixFreeSeedEnabled << ','
      << safeValue(stepSizeScale) << ',' << safeValue(outerRelaxation) << ','
      << initialGammaContract << ',' << splitImpulseEnabled << ','
      << stepContactRowDelassusProducts << ','
      << stepParallelContactRowDelassusProducts << ','
      << maxContactRowParticipantsToDate << ','
      << exactContactRowLogicalCpusToDate << ','
      << maxPhaseContactRowLogicalCpusToDate << ','
      << safeValue(phaseMetrics.maxCardCenterDisplacementFromInitial) << ','
      << safeValue(phaseMetrics.minCardOrientationAlignmentFromInitial) << ','
      << phaseMetrics.projectileCardContacts;
  if (contract == SolverContract::DartBestColoredBgs) {
    const std::size_t stepExactAttempts = stepExactSolves + stepExactFailures;
    int coloredUsed = -1;
    std::size_t coloredSolves = 0u;
    std::size_t coloredDispatches = 0u;
    std::size_t coloredParticipants = 0u;
    std::size_t coloredManifolds = 0u;
    std::size_t coloredColors = 0u;
    std::size_t coloredMaxManifoldsPerColor = 0u;
    std::string coloredLogicalCpuIds = "none";
    std::string coloredMaxPhaseLogicalCpuIds = "none";
    if (exactSolver != nullptr && stepExactAttempts == 1u) {
      coloredUsed
          = exactSolver->getLastExactCoulombColoredBlockGaussSeidelUsed() ? 1
                                                                          : 0;
      coloredSolves
          = exactSolver->getLastExactCoulombColoredBlockGaussSeidelSolves();
      coloredDispatches
          = exactSolver->getLastExactCoulombColoredBlockGaussSeidelDispatches();
      coloredParticipants
          = exactSolver
                ->getLastExactCoulombColoredBlockGaussSeidelParticipants();
      coloredManifolds
          = exactSolver->getLastExactCoulombColoredBlockGaussSeidelManifolds();
      coloredColors
          = exactSolver->getLastExactCoulombColoredBlockGaussSeidelColors();
      coloredMaxManifoldsPerColor
          = exactSolver
                ->getLastExactCoulombColoredBlockGaussSeidelMaxManifoldsPerColor();
      coloredLogicalCpuIds = formatLogicalCpuIds(
          exactSolver
              ->getLastExactCoulombColoredBlockGaussSeidelLogicalCpuIds());
      coloredMaxPhaseLogicalCpuIds = formatLogicalCpuIds(
          exactSolver
              ->getLastExactCoulombColoredBlockGaussSeidelMaxPhaseLogicalCpuIds());
    }
    row << ",dart_deterministic_manifold_colored_bgs_nonpaper," << coloredUsed
        << ',' << coloredSolves << ',' << coloredDispatches << ','
        << coloredParticipants << ',' << coloredManifolds << ','
        << coloredColors << ',' << coloredMaxManifoldsPerColor << ','
        << coloredLogicalCpuIds << ',' << coloredMaxPhaseLogicalCpuIds << ','
        << safeValue(archOutcomeMetrics.maxBodyDisplacementFromInitial) << ','
        << safeValue(archOutcomeMetrics.minBodyOrientationAlignmentFromInitial);
  }
  if (nativeManifoldSensitivityEnabled()) {
    const auto detector
        = std::dynamic_pointer_cast<dart::collision::NativeCollisionDetector>(
            world->getConstraintSolver()->getCollisionDetector());
    const char* actualMode = detector == nullptr
                                 ? "unavailable"
                                 : nativeContactManifoldModeName(
                                     detector->getContactManifoldMode());
    const auto& collisionOption
        = world->getConstraintSolver()->getCollisionOption();
    const std::size_t stepExactAttempts = stepExactSolves + stepExactFailures;
    const char* internalStatus = "not_run";
    int bestIteration = -1;
    double bestResidual = std::numeric_limits<double>::quiet_NaN();
    if (exactSolver != nullptr && stepExactAttempts > 0u) {
      internalStatus
          = fbfStatusName(exactSolver->getLastExactCoulombFbfStatus());
      bestIteration = exactSolver->getLastExactCoulombBestIteration();
      bestResidual = exactSolver->getLastExactCoulombBestResidual();
    }
    const auto pairMetrics = collectContactPairMultiplicityMetrics(world);
    row << ",card_house_native_manifold_sensitivity_v1,"
        << nativeManifoldSensitivitySelectorName(
               gNativeManifoldSensitivitySelector)
        << ',' << actualMode << ',' << collisionOption.maxNumContacts << ','
        << collisionOption.maxNumContactsPerPair << ','
        << stepMaxIterationsAccepted << ',' << internalStatus << ','
        << bestIteration << ',' << safeValue(bestResidual) << ','
        << pairMetrics.pairLabels << ',' << pairMetrics.multiplicities;
  }
  if (crossStepPolicyEvidenceEnabled()) {
    const auto detector
        = std::dynamic_pointer_cast<dart::collision::NativeCollisionDetector>(
            world->getConstraintSolver()->getCollisionDetector());
    const char* actualMode = detector == nullptr
                                 ? "unavailable"
                                 : nativeContactManifoldModeName(
                                     detector->getContactManifoldMode());
    const auto& collisionOption
        = world->getConstraintSolver()->getCollisionOption();
    const auto stepExactAttempts = counterDelta(
        currentCounters.exactAttempts, previousCounters.exactAttempts);
    const auto stepWarmStartStepSizeCaps = counterDelta(
        currentCounters.warmStartStepSizeCaps,
        previousCounters.warmStartStepSizeCaps);
    const auto stepUnconvergedCacheSkips = counterDelta(
        currentCounters.unconvergedCacheSkips,
        previousCounters.unconvergedCacheSkips);
    const char* lastDiagnosticsContract
        = stepExactAttempts > 1u
              ? "last_exact_group_only_multi_group_noncomparable"
          : stepExactAttempts == 1u ? "last_exact_group_only_single_group"
                                    : "unavailable_no_exact_attempt_this_step";
    double initialNaturalResidual = std::numeric_limits<double>::quiet_NaN();
    double finalNaturalResidual = std::numeric_limits<double>::quiet_NaN();
    double uncappedInitialGamma = std::numeric_limits<double>::quiet_NaN();
    int warmStartGammaCapApplied = -1;
    if (exactSolver != nullptr && stepExactAttempts > 0u) {
      initialNaturalResidual
          = exactSolver->getLastExactCoulombInitialNaturalMapResidual();
      finalNaturalResidual
          = exactSolver->getLastExactCoulombNaturalMapResidual();
      uncappedInitialGamma
          = exactSolver->getLastExactCoulombUncappedInitialStepSize();
      warmStartGammaCapApplied
          = exactSolver->getLastExactCoulombWarmStartStepSizeCapApplied() ? 1
                                                                          : 0;
    }
    const double worstResidual
        = exactSolver == nullptr ? std::numeric_limits<double>::quiet_NaN()
                                 : exactSolver->getWorstExactCoulombResidual();
    const auto& options = exactSolver->getExactCoulombOptions();
    const auto policy = exactSolver->getExactCoulombCrossStepPolicyOptions();
    row << ",card_house_cross_step_policy_ab_v1,"
        << crossStepPolicySelectorName(gCrossStepPolicySelector) << ','
        << classifyCrossStepPolicy(*exactSolver) << ",compact," << actualMode
        << ',' << collisionOption.maxNumContacts << ','
        << collisionOption.maxNumContactsPerPair << ',' << stepExactAttempts
        << ',' << stepMaxIterationsAccepted << ',' << stepWarmStartStepSizeCaps
        << ',' << stepUnconvergedCacheSkips << ',' << safeValue(worstResidual)
        << ',' << lastDiagnosticsContract << ','
        << safeValue(initialNaturalResidual) << ','
        << safeValue(finalNaturalResidual) << ','
        << safeValue(uncappedInitialGamma) << ',' << warmStartGammaCapApplied
        << ',' << warmStartMatchModeName(policy.warmStartMatchMode) << ','
        << safeValue(options.warmStartMatchDistance) << ','
        << safeValue(policy.warmStartNormalCosine) << ','
        << (policy.useStrictWarmStartMatchDistance ? 1 : 0) << ','
        << policy.warmStartMaxAge << ','
        << safeValue(policy.persistentStepSizeSafeBoundScale) << ','
        << safeValue(policy.minimumStepSize) << ','
        << safeValue(policy.maximumStepSize) << ','
        << safeValue(policy.warmStartResidualThreshold) << ','
        << safeValue(policy.warmStartStepSizeCap) << ','
        << (policy.persistUncappedStepSizeOnWarmStartCap ? 1 : 0) << ','
        << (policy.requireResidualImprovementForUnconvergedCacheSave ? 1 : 0)
        << ',' << safeValue(options.couplingVariationTolerance) << ','
        << safeValue(options.shrinkFactor) << ','
        << options.maxStepShrinkIterations;
  }
  if (literalCrownImpactState != nullptr) {
    const auto& impact = *literalCrownImpactState;
    const auto& pose = impact.poseMetrics;
    const bool finalGatesAuthoritative
        = step == kLiteralCrownImpactDefaultSteps;
    const double firstArchContactTime
        = impact.firstProjectileArchContactStep == 0u
              ? std::numeric_limits<double>::quiet_NaN()
              : static_cast<double>(impact.firstProjectileArchContactStep)
                    * kDt;
    const double firstGroundContactTime
        = impact.firstProjectileGroundContactStep == 0u
              ? std::numeric_limits<double>::quiet_NaN()
              : static_cast<double>(impact.firstProjectileGroundContactStep)
                    * kDt;
    const double worstResidual
        = exactSolver == nullptr ? std::numeric_limits<double>::quiet_NaN()
                                 : exactSolver->getWorstExactCoulombResidual();
    const bool exactGate = literalCrownImpactExactGate(
        *literalCrownImpactState, currentCounters);
    const bool residualGate = literalCrownImpactResidualGate(exactSolver);
    const bool finiteGate = impact.finiteStateToDate && pose.finiteState;
    const bool crownResponseGate = impact.maximumCrownDisplacementToDate
                                   >= kLiteralCrownImpactMinimumCrownResponse;
    const bool allBodyDisplacementGate
        = pose.finiteState
          && pose.maxBodyDisplacement
                 <= kLiteralCrownImpactMaximumBodyDisplacement;
    const bool orientationGate
        = pose.finiteState
          && pose.minOrientationAlignment
                 >= kLiteralCrownImpactMinimumOrientationAlignment;
    const bool farFieldDisplacementGate
        = pose.finiteState
          && pose.maxFarFieldDisplacement
                 <= kLiteralCrownImpactMaximumFarFieldDisplacement;
    const bool springerGate
        = pose.finiteState
          && pose.maxSpringerDisplacement
                 <= kLiteralCrownImpactSpringerTolerance
          && pose.minSpringerOrientationAlignment
                 >= 1.0 - kLiteralCrownImpactSpringerTolerance;
    const bool adjacencyGate = impact.farFieldAdjacentPairs
                               == kLiteralCrownImpactFarFieldAdjacentPairCount;
    row << ",literal_wedge_crown_impact_v1_reconstructed_nonpaper,"
        << (step <= kLiteralCrownImpactLaunchAfterSteps ? "standing_prefix"
                                                        : "crown_impact")
        << ",masonry_arch_25_literal_wedge,"
        << (step <= kLiteralCrownImpactLaunchAfterSteps ? 1 : 0) << ','
        << (finalGatesAuthoritative ? 1 : 0) << ','
        << (impact.preImpactSnapshotCaptured ? 1 : 0) << ','
        << impact.projectileCount << ',' << impact.stepProjectileArchContacts
        << ',' << impact.projectileArchContactsToDate << ','
        << impact.stepProjectileGroundContacts << ','
        << impact.projectileGroundContactsToDate << ','
        << (impact.firstProjectileArchContactStep == 0u
                ? -1
                : static_cast<std::int64_t>(
                    impact.firstProjectileArchContactStep))
        << ',' << safeValue(firstArchContactTime) << ','
        << (impact.firstProjectileGroundContactStep == 0u
                ? -1
                : static_cast<std::int64_t>(
                    impact.firstProjectileGroundContactStep))
        << ',' << safeValue(firstGroundContactTime) << ','
        << currentCounters.exactSolves << ',' << currentCounters.exactFailures
        << ',' << currentCounters.boxedFallbacks << ','
        << currentCounters.maxIterationsAccepted << ','
        << (impact.finiteStateToDate ? 1 : 0) << ',' << safeValue(worstResidual)
        << ',' << impact.preImpactStandingGate << ','
        << safeValue(pose.maxCrownDisplacement) << ','
        << safeValue(impact.maximumCrownDisplacementToDate) << ','
        << safeValue(pose.maxBodyDisplacement) << ','
        << safeValue(pose.minOrientationAlignment) << ','
        << safeValue(pose.maxFarFieldDisplacement) << ','
        << safeValue(pose.maxSpringerDisplacement) << ','
        << safeValue(pose.minSpringerOrientationAlignment) << ','
        << impact.farFieldAdjacentPairs << ','
        << (literalCrownImpactContactOrderGate(impact) ? 1 : 0) << ','
        << (exactGate ? 1 : 0) << ',' << (residualGate ? 1 : 0) << ','
        << (finiteGate ? 1 : 0) << ',' << (crownResponseGate ? 1 : 0) << ','
        << (allBodyDisplacementGate ? 1 : 0) << ',' << (orientationGate ? 1 : 0)
        << ',' << (farFieldDisplacementGate ? 1 : 0) << ','
        << (springerGate ? 1 : 0) << ',' << (adjacencyGate ? 1 : 0) << ','
        << (literalCrownImpactFinalAcceptanceGate(
                step, impact, currentCounters, exactSolver)
                ? 1
                : 0);
  }
  std::printf("%s\n", row.str().c_str());
  std::fflush(stdout);
}

void printPhaseSummaryRow(
    std::size_t step,
    Scenario scenario,
    SolverMode solverMode,
    const std::shared_ptr<dart::simulation::World>& world,
    const dart::constraint::ExactCoulombFbfConstraintSolver* exactSolver)
{
  const auto metrics = collectPhaseSummaryMetrics(world);
  const auto contacts
      = world->getConstraintSolver()->getLastCollisionResult().getNumContacts();
  std::size_t exactSolves = 0u;
  std::size_t warmStarts = 0u;
  std::size_t fallbacks = 0u;
  double residual = std::numeric_limits<double>::quiet_NaN();
  const char* status = "boxed_lcp";
  if (exactSolver != nullptr) {
    exactSolves = exactSolver->getNumExactCoulombSolves();
    warmStarts = exactSolver->getNumExactCoulombWarmStarts();
    fallbacks = exactSolver->getNumBoxedLcpFallbacks();
    residual = exactSolver->getLastExactCoulombResidual();
    status = exactStatusName(exactSolver->getLastExactCoulombStatus());
  }

  std::printf(
      "%zu,%.17g,%s,%s,%s,%zu,%zu,%d,%.17g,%.17g,%.17g,%.17g,"
      "%zu,%zu,%zu,%zu,%.17g,%s\n",
      step,
      world->getTime(),
      scenarioName(scenario),
      solverName(solverMode),
      phaseName(scenario, step),
      metrics.cardCount,
      metrics.projectileCount,
      metrics.finiteState ? 1 : 0,
      safeValue(metrics.minCardAxisUp),
      safeValue(metrics.minCenterHeight),
      safeValue(metrics.maxCardHorizontalTravel),
      safeValue(metrics.maxProjectileSpeed),
      contacts,
      exactSolves,
      warmStarts,
      fallbacks,
      safeValue(residual),
      status);
}

void printTraceRow(
    std::size_t step,
    Scenario scenario,
    SolverMode solverMode,
    const std::shared_ptr<dart::simulation::World>& world,
    const dart::dynamics::BodyNode* body,
    const dart::constraint::ExactCoulombFbfConstraintSolver* exactSolver)
{
  const Eigen::Isometry3d transform = body->getWorldTransform();
  const Eigen::Vector3d position = transform.translation();
  const Eigen::Vector3d velocity = body->getLinearVelocity();
  const double upZ
      = transform.linear().col(2).normalized().dot(Eigen::Vector3d::UnitZ());
  const auto contacts
      = world->getConstraintSolver()->getLastCollisionResult().getNumContacts();

  std::size_t exactSolves = 0u;
  std::size_t warmStarts = 0u;
  std::size_t fallbacks = 0u;
  double residual = std::numeric_limits<double>::quiet_NaN();
  const char* status = "boxed_lcp";
  if (exactSolver != nullptr) {
    exactSolves = exactSolver->getNumExactCoulombSolves();
    warmStarts = exactSolver->getNumExactCoulombWarmStarts();
    fallbacks = exactSolver->getNumBoxedLcpFallbacks();
    residual = exactSolver->getLastExactCoulombResidual();
    status = exactStatusName(exactSolver->getLastExactCoulombStatus());
  }

  std::printf(
      "%zu,%.17g,%s,%s,%s,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,"
      "%zu,%zu,%zu,%zu,%.17g,%s\n",
      step,
      world->getTime(),
      scenarioName(scenario),
      solverName(solverMode),
      body->getName().c_str(),
      safeValue(position.x()),
      safeValue(position.y()),
      safeValue(position.z()),
      safeValue(velocity.x()),
      safeValue(velocity.y()),
      safeValue(velocity.z()),
      safeValue(upZ),
      contacts,
      exactSolves,
      warmStarts,
      fallbacks,
      safeValue(residual),
      status);
}

std::vector<const dart::dynamics::BodyNode*> collectTraceBodies(
    const std::shared_ptr<dart::simulation::World>& world,
    Scenario scenario,
    TraceScope traceScope)
{
  std::vector<const dart::dynamics::BodyNode*> bodies;
  if (traceScope == TraceScope::TrackedBody) {
    auto skeleton = world->getSkeleton(bodyNameForScenario(scenario));
    if (skeleton && skeleton->getNumBodyNodes() > 0u)
      bodies.push_back(skeleton->getBodyNode(0));
    return bodies;
  }

  for (std::size_t i = 0u; i < world->getNumSkeletons(); ++i) {
    const auto skeleton = world->getSkeleton(i);
    if (!skeleton || !skeleton->isMobile())
      continue;

    for (std::size_t j = 0u; j < skeleton->getNumBodyNodes(); ++j)
      bodies.push_back(skeleton->getBodyNode(j));
  }

  return bodies;
}

void printTraceRows(
    std::size_t step,
    Scenario scenario,
    SolverMode solverMode,
    const std::shared_ptr<dart::simulation::World>& world,
    const std::vector<const dart::dynamics::BodyNode*>& bodies,
    const dart::constraint::ExactCoulombFbfConstraintSolver* exactSolver)
{
  for (const auto* body : bodies)
    printTraceRow(step, scenario, solverMode, world, body, exactSolver);
}

void printResidualHistoryRows(
    std::size_t step,
    Scenario scenario,
    SolverMode solverMode,
    const std::shared_ptr<dart::simulation::World>& world,
    const dart::constraint::ExactCoulombFbfConstraintSolver* exactSolver)
{
  const auto contacts
      = world->getConstraintSolver()->getLastCollisionResult().getNumContacts();
  const auto& records = exactSolver->getExactCoulombResidualHistoryRecords();
  for (const auto& record : records) {
    for (const auto& sample : record.samples) {
      std::printf(
          "%zu,%.17g,%s,%s,%zu,%zu,%d,%.17g,%.17g,%.17g,%.17g,%.17g,"
          "%.17g,%.17g,%d,%zu,%zu,%zu,%zu,%s\n",
          step,
          world->getTime(),
          scenarioName(scenario),
          solverName(solverMode),
          record.solveIndex,
          record.contactCount,
          sample.iteration,
          safeValue(sample.residual.value),
          safeValue(sample.residual.primalFeasibility),
          safeValue(sample.residual.dualFeasibility),
          safeValue(sample.residual.complementarity),
          safeValue(sample.stepSize),
          safeValue(sample.safeStepSize),
          safeValue(sample.couplingVariationRatio),
          sample.shrinkIterations,
          contacts,
          exactSolver->getNumExactCoulombSolves(),
          exactSolver->getNumExactCoulombWarmStarts(),
          exactSolver->getNumBoxedLcpFallbacks(),
          exactStatusName(record.status));
    }
  }
}

void printUsage()
{
  std::fprintf(
      stderr,
      "Query: fbf_paper_trace --author-turntable-contract SCENARIO "
      "[dart_best|paper_cpu]\n"
      "Usage: fbf_paper_trace [scenario=backspin] [solver=exact_fbf] "
      "[sample_stride=30] [steps=paper_duration] [initial_gamma=nan] "
      "[trace_scope=tracked] [warm_start=default|0|1] "
      "[split_impulse=default|0|1] [simulation_threads=1] "
      "[solver_contract=dart_best|dart_best_colored_bgs|paper_cpu|"
      "paper_cpu_bootstrap_diagnostic] "
      "[collision_frontend=dart|native] "
      "[local_solver=default|exact_metric|inverse_euclidean|"
      "projected_gradient] [bootstrap_outer_iterations=0] "
      "[post_bootstrap_outer_iterations=0] "
      "[native_manifold_sensitivity=default|compact|four_point_planar] "
      "[cross_step_policy=default|dart_current|"
      "author_policy_inspired_b3f3c5c]\n");
}

int printAuthorTurntableContract(int argc, char** argv)
{
  if (argc < 3 || argc > 4) {
    printUsage();
    return 2;
  }

  Scenario scenario = Scenario::Backspin;
  SolverContract contract = SolverContract::DartBest;
  if (!parseScenario(argv[2], scenario) || !isAuthorTurntableScenario(scenario)
      || !parseSolverContract(argc > 3 ? argv[3] : nullptr, contract)
      || (contract != SolverContract::DartBest
          && contract != SolverContract::PaperCpu)) {
    printUsage();
    return 2;
  }

  const auto* sourceSpec
      = fbf_author_turntable::findByTraceScenario(scenarioName(scenario));
  if (sourceSpec == nullptr) {
    std::fprintf(stderr, "author turntable shared scenario spec is missing\n");
    return 1;
  }

  const auto world = createTraceWorld(
      scenario,
      SolverMode::ExactFbf,
      TraceScope::TrackedBody,
      std::numeric_limits<double>::quiet_NaN(),
      1u,
      contract,
      CollisionFrontend::Native);
  try {
    const auto runtimeContract = fbf_author_turntable::inspectPhysicsContract(
        world,
        *sourceSpec,
        solverContractName(contract, 1u),
        "fbf_paper_trace",
        DART_FBF_AUTHOR_TURNTABLE_IMPLEMENTATION_SHA256);
    std::printf(
        "%s\n",
        fbf_author_turntable::physicsContractJson(runtimeContract).c_str());
  } catch (const std::exception& error) {
    std::fprintf(stderr, "%s\n", error.what());
    return 1;
  }
  return 0;
}

} // namespace

int main(int argc, char** argv)
{
  if (argc > 1 && std::string(argv[1]) == "--author-turntable-contract")
    return printAuthorTurntableContract(argc, argv);

  if (argc > 17) {
    printUsage();
    return 2;
  }

  Scenario scenario = Scenario::Backspin;
  SolverMode solverMode = SolverMode::ExactFbf;
  TraceScope traceScope = TraceScope::TrackedBody;
  SolverContract contract = SolverContract::DartBest;
  CollisionFrontend collisionFrontend = CollisionFrontend::Dart;
  if (!parseScenario(argc > 1 ? argv[1] : nullptr, scenario)
      || !parseSolver(argc > 2 ? argv[2] : nullptr, solverMode)
      || !parseTraceScope(argc > 6 ? argv[6] : nullptr, traceScope)) {
    printUsage();
    return 2;
  }
  if (traceScope == TraceScope::ResidualHistory
      && solverMode != SolverMode::ExactFbf) {
    printUsage();
    return 2;
  }
  if (traceScope == TraceScope::PhaseSummary
      && scenario != Scenario::CardHouseFourLevelSettleProjectileReduced
      && scenario != Scenario::CardHouseFourLevelSettleProjectileFull) {
    printUsage();
    return 2;
  }

  std::size_t sampleStride = 30u;
  std::size_t steps = defaultScenarioSteps(scenario);
  if (!parseSizeArg(argc > 3 ? argv[3] : nullptr, 30u, sampleStride)
      || sampleStride == 0u
      || !parseSizeArg(argc > 4 ? argv[4] : nullptr, steps, steps)) {
    printUsage();
    return 2;
  }

  double initialStepSize = std::numeric_limits<double>::quiet_NaN();
  if (!parsePositiveDoubleArg(
          argc > 5 ? argv[5] : nullptr,
          std::numeric_limits<double>::quiet_NaN(),
          initialStepSize)) {
    printUsage();
    return 2;
  }

  if (argc > 7 && argv[7] != nullptr) {
    const std::string warmStartArg = argv[7];
    if (warmStartArg == "0") {
      gWarmStartOverride = 0;
    } else if (warmStartArg == "1") {
      gWarmStartOverride = 1;
    } else if (warmStartArg != "default") {
      printUsage();
      return 2;
    }
  }

  if (argc > 8 && argv[8] != nullptr) {
    const std::string splitImpulseArg = argv[8];
    if (splitImpulseArg == "0") {
      gSplitImpulseOverride = 0;
    } else if (splitImpulseArg == "1") {
      gSplitImpulseOverride = 1;
    } else if (splitImpulseArg != "default") {
      printUsage();
      return 2;
    }
  }

  std::size_t simulationThreads = 1u;
  std::size_t bootstrapOuterIterations = 0u;
  std::size_t postBootstrapOuterIterations = 0u;
  if (!parseSizeArg(argc > 9 ? argv[9] : nullptr, 1u, simulationThreads)
      || simulationThreads == 0u
      || !parseSolverContract(argc > 10 ? argv[10] : nullptr, contract)
      || !parseCollisionFrontend(
          argc > 11 ? argv[11] : nullptr, collisionFrontend)
      || !parseLocalSolverOverride(argc > 12 ? argv[12] : nullptr)
      || !parseSizeArg(
          argc > 13 ? argv[13] : nullptr, 0u, bootstrapOuterIterations)
      || !parseSizeArg(
          argc > 14 ? argv[14] : nullptr, 0u, postBootstrapOuterIterations)
      || !parseNativeManifoldSensitivitySelector(
          argc > 15 ? argv[15] : nullptr, gNativeManifoldSensitivitySelector)
      || !parseCrossStepPolicySelector(
          argc > 16 ? argv[16] : nullptr, gCrossStepPolicySelector)
      || bootstrapOuterIterations
             > static_cast<std::size_t>(std::numeric_limits<int>::max())
      || postBootstrapOuterIterations
             > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
    printUsage();
    return 2;
  }

  // The pinned author family uses DART's Native FourPointPlanar manifold.
  // Preserve the existing CLI default for every older scenario, while making
  // a source-pinned invocation without an explicit frontend select Native.
  if (argc <= 11 && isAuthorTurntableScenario(scenario))
    collisionFrontend = CollisionFrontend::Native;

  const bool bootstrapEnabled = bootstrapOuterIterations > 0u;
  if (bootstrapEnabled != (postBootstrapOuterIterations > 0u)) {
    std::fprintf(
        stderr,
        "bootstrap and post-bootstrap outer iterations must both be zero or "
        "both be positive\n");
    return 2;
  }
  if (nativeManifoldSensitivityEnabled()) {
    const bool frozenContract
        = (argc == 16 || (argc == 17 && std::string(argv[16]) == "default"))
          && std::string(argv[1]) == "card_house_26_settle_projectile_full"
          && std::string(argv[2]) == "exact_fbf" && std::string(argv[3]) == "1"
          && std::string(argv[4]) == "600" && std::string(argv[5]) == "nan"
          && std::string(argv[6]) == "performance"
          && std::string(argv[7]) == "default"
          && std::string(argv[8]) == "default" && std::string(argv[9]) == "1"
          && std::string(argv[10]) == "paper_cpu"
          && std::string(argv[11]) == "native"
          && std::string(argv[12]) == "default" && std::string(argv[13]) == "0"
          && std::string(argv[14]) == "0";
    if (!frozenContract) {
      std::fprintf(
          stderr,
          "explicit native-manifold sensitivity requires the frozen "
          "card-house v1 scenario, solver, trace, step, thread, frontend, "
          "and bootstrap contract\n");
      return 2;
    }
  }
  if (crossStepPolicyEvidenceEnabled()) {
    const bool frozenContract
        = argc == 17
          && std::string(argv[1]) == "card_house_26_settle_projectile_full"
          && std::string(argv[2]) == "exact_fbf" && std::string(argv[3]) == "1"
          && std::string(argv[4]) == "90" && std::string(argv[5]) == "nan"
          && std::string(argv[6]) == "performance"
          && std::string(argv[7]) == "default"
          && std::string(argv[8]) == "default" && std::string(argv[9]) == "1"
          && std::string(argv[10]) == "paper_cpu"
          && std::string(argv[11]) == "native"
          && std::string(argv[12]) == "default" && std::string(argv[13]) == "0"
          && std::string(argv[14]) == "0" && std::string(argv[15]) == "default";
    if (!frozenContract) {
      std::fprintf(
          stderr,
          "explicit cross-step policy requires the frozen 90-step "
          "card-house A/B v1 scenario, solver, trace, thread, frontend, "
          "manifold, and bootstrap contract\n");
      return 2;
    }
  }
  if (!bootstrapEnabled
      && contract == SolverContract::PaperCpuBootstrapDiagnostic) {
    std::fprintf(
        stderr,
        "paper_cpu_bootstrap_diagnostic requires positive bootstrap and "
        "post-bootstrap outer-iteration budgets\n");
    return 2;
  }
  if (bootstrapEnabled
      && (contract != SolverContract::PaperCpuBootstrapDiagnostic
          || solverMode != SolverMode::ExactFbf
          || traceScope != TraceScope::Performance)) {
    std::fprintf(
        stderr,
        "bootstrap outer iterations require solver=exact_fbf, "
        "trace_scope=performance, and "
        "solver_contract=paper_cpu_bootstrap_diagnostic\n");
    return 2;
  }

  if (usesPaperCpuParameters(contract)
      && (std::isfinite(initialStepSize) || gWarmStartOverride != -1
          || gSplitImpulseOverride != -1 || gLocalSolverOverrideSet)) {
    std::fprintf(
        stderr,
        "paper-CPU parameter contracts reject initial-gamma, warm-start, "
        "split-impulse, and local-solver overrides; use dart_best for those "
        "diagnostics\n");
    return 2;
  }
  if (isLiteralWedgeScenario(scenario)
      && collisionFrontend != CollisionFrontend::Native) {
    std::fprintf(
        stderr, "literal-wedge scenarios require collision_frontend=native\n");
    return 2;
  }
  if (isAuthorTurntableScenario(scenario)
      && collisionFrontend != CollisionFrontend::Native) {
    std::fprintf(
        stderr,
        "author turntable scenarios require collision_frontend=native\n");
    return 2;
  }
  if (scenario == Scenario::MasonryArch101LiteralWedge
      && solverMode != SolverMode::ExactFbf) {
    std::fprintf(
        stderr, "masonry_arch_101_literal_wedge requires solver=exact_fbf\n");
    return 2;
  }
  if (scenario == Scenario::MasonryArch25LiteralWedgeCrownImpactV1
      && (solverMode != SolverMode::ExactFbf
          || contract != SolverContract::DartBestColoredBgs
          || traceScope != TraceScope::Performance
          || steps != kLiteralCrownImpactDefaultSteps)) {
    std::fprintf(
        stderr,
        "masonry_arch_25_literal_wedge_crown_impact_v1 requires "
        "solver=exact_fbf, trace_scope=performance, "
        "solver_contract=dart_best_colored_bgs, and exactly %zu steps\n",
        kLiteralCrownImpactDefaultSteps);
    return 2;
  }
  if (usesPaperCpuParameters(contract)
      && (solverMode != SolverMode::ExactFbf || simulationThreads != 1u
          || collisionFrontend != CollisionFrontend::Native)) {
    std::fprintf(
        stderr,
        "paper-CPU parameter contracts require solver=exact_fbf, "
        "simulation_threads=1, and collision_frontend=native; use dart_best "
        "for boxed-LCP, thread-count, or frontend diagnostics\n");
    return 2;
  }
  if (contract == SolverContract::DartBestColoredBgs
      && solverMode != SolverMode::ExactFbf) {
    std::fprintf(
        stderr,
        "dart_best_colored_bgs requires solver=exact_fbf; use dart_best for "
        "boxed-LCP diagnostics\n");
    return 2;
  }
  if (solverMode != SolverMode::ExactFbf && gLocalSolverOverrideSet) {
    std::fprintf(
        stderr,
        "local-solver overrides require solver=exact_fbf and "
        "solver_contract=dart_best\n");
    return 2;
  }

  gAppendLiteralCrownImpactColumns
      = scenario == Scenario::MasonryArch25LiteralWedgeCrownImpactV1;
  auto world = createTraceWorld(
      scenario,
      solverMode,
      traceScope,
      initialStepSize,
      simulationThreads,
      contract,
      collisionFrontend);
  if (nativeManifoldSensitivityEnabled()) {
    using Mode = dart::collision::NativeCollisionDetector::ContactManifoldMode;
    const auto detector
        = std::dynamic_pointer_cast<dart::collision::NativeCollisionDetector>(
            world->getConstraintSolver()->getCollisionDetector());
    const Mode expectedMode
        = gNativeManifoldSensitivitySelector
                  == NativeManifoldSensitivitySelector::Compact
              ? Mode::Compact
              : Mode::FourPointPlanar;
    const auto& collisionOption
        = world->getConstraintSolver()->getCollisionOption();
    if (detector == nullptr
        || detector->getContactManifoldMode() != expectedMode
        || collisionOption.maxNumContacts != kCardHouseReducedMaxContacts
        || collisionOption.maxNumContactsPerPair
               != kCardHouseReducedMaxContactsPerPair) {
      std::fprintf(
          stderr,
          "native-manifold sensitivity failed installed detector or "
          "collision-cap readback\n");
      return 1;
    }
  }
  const InitialCardPoses initialCardPoses = collectInitialCardPoses(world);
  const InitialArchPoses initialArchPoses = collectInitialArchPoses(world);
  const std::size_t expectedLiteralArchStoneCount
      = scenario == Scenario::MasonryArch101LiteralWedge ? kArch101StoneCount
                                                         : kArchStoneCount;
  if (isLiteralWedgeScenario(scenario)
      && initialArchPoses.size() != expectedLiteralArchStoneCount) {
    std::fprintf(
        stderr,
        "literal-wedge scenario expected %zu arch bodies but found "
        "%zu\n",
        expectedLiteralArchStoneCount,
        initialArchPoses.size());
    return 1;
  }
  const auto bodies = traceScope == TraceScope::ResidualHistory
                              || traceScope == TraceScope::PhaseSummary
                              || traceScope == TraceScope::Performance
                          ? std::vector<const dart::dynamics::BodyNode*>()
                          : collectTraceBodies(world, scenario, traceScope);
  if (traceScope != TraceScope::ResidualHistory
      && traceScope != TraceScope::PhaseSummary
      && traceScope != TraceScope::Performance && bodies.empty()) {
    std::fprintf(stderr, "fbf_paper_trace could not find traced bodies\n");
    return 1;
  }

  auto* exactSolver
      = solverMode == SolverMode::ExactFbf ? getExactSolver(world) : nullptr;
  if (solverMode == SolverMode::ExactFbf && exactSolver == nullptr) {
    std::fprintf(stderr, "fbf_paper_trace could not install exact solver\n");
    return 1;
  }
  if (crossStepPolicyEvidenceEnabled()) {
    installCrossStepPolicy(*exactSolver);
    const auto detector
        = std::dynamic_pointer_cast<dart::collision::NativeCollisionDetector>(
            world->getConstraintSolver()->getCollisionDetector());
    const auto& collisionOption
        = world->getConstraintSolver()->getCollisionOption();
    if (detector == nullptr
        || detector->getContactManifoldMode()
               != dart::collision::NativeCollisionDetector::
                   ContactManifoldMode::Compact
        || collisionOption.maxNumContacts != kCardHouseReducedMaxContacts
        || collisionOption.maxNumContactsPerPair
               != kCardHouseReducedMaxContactsPerPair
        || std::string(classifyCrossStepPolicy(*exactSolver))
               != crossStepPolicySelectorName(gCrossStepPolicySelector)) {
      std::fprintf(
          stderr,
          "cross-step policy failed installed solver, detector, collision-"
          "cap, or policy readback\n");
      return 1;
    }
  }
  if (bootstrapEnabled) {
    auto options = exactSolver->getExactCoulombOptions();
    options.maxOuterIterations = static_cast<int>(bootstrapOuterIterations);
    exactSolver->setExactCoulombOptions(options);
  }

  LiteralCrownImpactTraceState literalCrownImpactState;

  if (traceScope == TraceScope::ResidualHistory) {
    printResidualHistoryHeader();
  } else if (traceScope == TraceScope::PhaseSummary) {
    printPhaseSummaryHeader();
    printPhaseSummaryRow(0u, scenario, solverMode, world, exactSolver);
  } else if (traceScope == TraceScope::Performance) {
    printPerformanceHeader(contract);
  } else {
    printHeader();
    printTraceRows(0u, scenario, solverMode, world, bodies, exactSolver);
  }
  ExactCounterSnapshot previousCounters = readExactCounterSnapshot(exactSolver);
  bool authorTurntableContractViolation = false;
  for (std::size_t step = 0; step < steps; ++step) {
    if (scenario == Scenario::MasonryArch25LiteralWedgeCrownImpactV1
        && step == kLiteralCrownImpactLaunchAfterSteps) {
      captureLiteralCrownImpactPreImpactPoses(
          world, previousCounters, literalCrownImpactState);
    }
    applyScenarioControl(world, scenario, step);
    if (traceScope == TraceScope::ResidualHistory)
      exactSolver->clearExactCoulombResidualHistoryRecords();
    const auto wallStart = std::chrono::steady_clock::now();
    world->step();
    const auto wallEnd = std::chrono::steady_clock::now();
    const std::size_t completedStep = step + 1u;
    const auto currentCounters = readExactCounterSnapshot(exactSolver);
    if (scenario == Scenario::MasonryArch25LiteralWedgeCrownImpactV1) {
      updateLiteralCrownImpactTraceState(
          completedStep,
          world,
          exactSolver,
          initialArchPoses,
          currentCounters,
          literalCrownImpactState);
    }
    const bool exactGroupFailed
        = counterDelta(
              currentCounters.exactFailures, previousCounters.exactFailures)
          > 0u;
    const auto stepExactSolves = counterDelta(
        currentCounters.exactSolves, previousCounters.exactSolves);
    const auto stepAcceptedAtCap = counterDelta(
        currentCounters.maxIterationsAccepted,
        previousCounters.maxIterationsAccepted);
    if (isAuthorTurntableScenario(scenario) && stepExactSolves > 0u) {
      const double stepResidual = exactSolver->getLastExactCoulombResidual();
      authorTurntableContractViolation
          = authorTurntableContractViolation || stepAcceptedAtCap > 0u
            || !std::isfinite(stepResidual) || stepResidual > 1e-6;
    }
    if (traceScope == TraceScope::Performance) {
      const double wallMilliseconds
          = std::chrono::duration<double, std::milli>(wallEnd - wallStart)
                .count();
      printPerformanceRow(
          completedStep,
          scenario,
          solverMode,
          contract,
          collisionFrontend,
          simulationThreads,
          wallMilliseconds,
          previousCounters,
          currentCounters,
          world,
          exactSolver,
          initialCardPoses,
          initialArchPoses,
          scenario == Scenario::MasonryArch25LiteralWedgeCrownImpactV1
              ? &literalCrownImpactState
              : nullptr);
    } else if (
        completedStep % sampleStride == 0u || completedStep == steps
        || exactGroupFailed) {
      if (traceScope == TraceScope::ResidualHistory) {
        printResidualHistoryRows(
            completedStep, scenario, solverMode, world, exactSolver);
      } else if (traceScope == TraceScope::PhaseSummary) {
        printPhaseSummaryRow(
            completedStep, scenario, solverMode, world, exactSolver);
      } else {
        printTraceRows(
            completedStep, scenario, solverMode, world, bodies, exactSolver);
      }
    }
    if (bootstrapEnabled && completedStep == 1u) {
      auto options = exactSolver->getExactCoulombOptions();
      options.maxOuterIterations
          = static_cast<int>(postBootstrapOuterIterations);
      exactSolver->setExactCoulombOptions(options);
    }
    previousCounters = currentCounters;

    // Evidence runs disable boxed-LCP fallback.  A failed exact group is left
    // unsolved by contract, so advancing the world again would turn a solver
    // failure into a physically meaningless trajectory.  Preserve the failed
    // step's diagnostics above, then stop immediately.
    if (exactGroupFailed)
      return 1;
  }

  if (scenario == Scenario::MasonryArch25LiteralWedgeCrownImpactV1
      && !literalCrownImpactFinalAcceptanceGate(
          steps, literalCrownImpactState, previousCounters, exactSolver)) {
    std::fprintf(
        stderr,
        "literal-wedge crown-impact v1 failed a preregistered acceptance "
        "gate; preserve the trace as a scientific negative\n");
    return 1;
  }

  if (authorTurntableContractViolation) {
    std::fprintf(
        stderr,
        "author turntable trace observed an intermediate exact-contract "
        "violation; preserve the complete trajectory as a scientific "
        "negative\n");
    return 1;
  }

  if (solverMode == SolverMode::ExactFbf) {
    const bool ok = exactSolver->getNumExactCoulombSolves() > 0u
                    && exactSolver->getNumBoxedLcpFallbacks() == 0u
                    && std::isfinite(exactSolver->getLastExactCoulombResidual())
                    && exactSolver->getLastExactCoulombResidual() <= 1e-6;
    if (!ok)
      return 1;
  }

  return 0;
}
