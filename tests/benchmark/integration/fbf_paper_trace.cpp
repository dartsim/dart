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
//                   [trace_scope=tracked]
//
// Scenarios:
//   incline_mu_0_5, incline_mu_0_4, backspin,
//   turntable_mu_0_2_omega_2, turntable_mu_0_2_omega_5,
//   turntable_mu_0_5_omega_2, turntable_mu_0_5_omega_5,
//   painleve_mu_0_5, painleve_mu_0_55,
//   card_house_26_reduced_contact (full natural manifold, 108 contacts at
//   the current base),
//   card_house_26_settle_projectile_reduced_contact,
//   masonry_arch_25_reduced_contact,
//   masonry_arch_25_projectile_reduced_contact,
//   masonry_arch_101_reduced_contact,
//   masonry_arch_25_full_manifold, masonry_arch_101_full_manifold
//
// Solvers:
//   exact_fbf, boxed_lcp
//
// Trace scopes:
//   tracked, dynamic_bodies, full_scene, residual_history, phase_summary

#include <dart/collision/dart/DARTCollisionDetector.hpp>
#include <dart/constraint/ContactConstraint.hpp>
#include <dart/constraint/ExactCoulombFbfConstraintSolver.hpp>
#include <dart/dynamics/BoxShape.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/PlaneShape.hpp>
#include <dart/dynamics/ShapeFrame.hpp>
#include <dart/dynamics/Skeleton.hpp>
#include <dart/dynamics/SphereShape.hpp>
#include <dart/math/detail/MasonryArchGeometry.hpp>
#include <dart/simulation/DeactivationOptions.hpp>
#include <dart/simulation/World.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <cmath>
#include <cstddef>

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
constexpr double kCardHouseInitialPenetration = 0.003;
constexpr double kCardHouseFrameSpacing = 0.55;
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
constexpr std::size_t kCardHouseProjectileCount = 4u;
constexpr double kCardHouseProjectileRadius = 0.055;
constexpr double kCardHouseProjectileMass = 0.02;
constexpr double kCardHouseProjectileSpeed = 4.0;
constexpr double kSmallFixtureStepSizeScale = 2.0;
constexpr std::size_t kCardHouseFourLevelCount = 4u;
// Full natural manifold for the 26-card one-step/settle-projectile rung:
// 512/4 caps observe 108 actual contacts and solve clean (~2.5 s, zero
// fallbacks) with the card-house step-size-scale/outer-relaxation options
// below.
constexpr std::size_t kCardHouseReducedMaxContacts = 512u;
constexpr std::size_t kCardHouseReducedMaxContactsPerPair = 4u;
// Author-faithful masonry-arch friction coefficient: Rigid-IPC's
// arch-{25,101}-stones.json both specify coefficient_friction = 0.5
// uniformly (see docs/dev_tasks/fbf_exact_coulomb_friction/ PROVENANCE.txt
// section 7). DART's other paper fixtures use 0.8; the ported arch keeps
// the source's own coefficient for scientific fidelity.
constexpr double kArchFriction = 0.5;
// Rigid-IPC's default body density ("plastic", src/io/read_rb_scene.cpp:68).
constexpr double kArchDensity = 1000.0;
constexpr std::size_t kArchStoneCount = 25u;
constexpr std::size_t kArch101StoneCount = 101u;
// Crown (topmost) centroid height and cross-section width, in meters, from
// the weighted-catenary generator (fc=60cm, sqrt(Qt)=7cm); used to place
// the projectile at the arch's crown regardless of stone count.
constexpr double kArchCrownHeight = 0.60;
constexpr double kArchCrownWidth = 0.07;
constexpr std::size_t kArchReducedMaxContacts = 48u;
constexpr std::size_t kArchReducedMaxContactsPerPair = 2u;
constexpr std::size_t kArch101ReducedMaxContacts = 38u;
constexpr std::size_t kArch101ReducedMaxContactsPerPair = 2u;
// Full natural manifold for both arches at per_pair 4: the 25-stone arch
// observes 96 actual contacts (96 also at per_pair 8, i.e. the manifold is
// saturated), and the 101-stone arch observes the full 512-contact cap.
constexpr std::size_t kArchFullManifoldMaxContacts = 512u;
constexpr std::size_t kArchFullManifoldMaxContactsPerPair = 4u;
constexpr int kArchMaxOuterIterations = 120000;
constexpr double kArchOuterRelaxation = 1.5;
constexpr double kArchStepSizeScale = 10.0;
constexpr double kArchProjectileRadius = 0.08;
constexpr double kArchProjectileMass = 0.05;
constexpr double kArchProjectileSpeed = 3.0;
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
};

struct PhaseSummaryMetrics
{
  std::size_t cardCount = 0u;
  std::size_t projectileCount = 0u;
  bool finiteState = true;
  double minCardAxisUp = std::numeric_limits<double>::infinity();
  double minCenterHeight = std::numeric_limits<double>::infinity();
  double maxCardHorizontalTravel = 0.0;
  double maxProjectileSpeed = 0.0;
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

const char* exactStatusName(
    dart::constraint::ExactCoulombFbfConstraintSolverStatus status)
{
  switch (status) {
    case dart::constraint::ExactCoulombFbfConstraintSolverStatus::NotRun:
      return "not_run";
    case dart::constraint::ExactCoulombFbfConstraintSolverStatus::Success:
      return "success";
    case dart::constraint::ExactCoulombFbfConstraintSolverStatus::InvalidOptions:
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

  return false;
}

bool parseSizeArg(const char* value, std::size_t fallback, std::size_t& output)
{
  if (value == nullptr) {
    output = fallback;
    return true;
  }

  errno = 0;
  char* end = nullptr;
  const unsigned long long parsed = std::strtoull(value, &end, 10);
  if (value[0] == '\0' || end == value || *end != '\0' || errno == ERANGE)
    return false;

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
    case Scenario::TurntableHighSlow:
    case Scenario::TurntableHighFast:
    case Scenario::Backspin:
    case Scenario::PainleveSlide:
      return 0.5;
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
    case Scenario::TurntableLowFast:
    case Scenario::TurntableHighFast:
      return 5.0;
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
         || scenario == Scenario::TurntableHighFast;
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
      return "masonry_arch_stone_12";
    case Scenario::MasonryArch25ProjectileReduced:
      return "masonry_arch_projectile";
    case Scenario::MasonryArch101Reduced:
    case Scenario::MasonryArch101FullManifold:
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
  return options;
}

dart::constraint::ExactCoulombFbfConstraintSolverOptions makeFbfOptions(
    double initialStepSize, Scenario scenario, TraceScope traceScope)
{
  auto options = makeFbfOptions(initialStepSize, traceScope);
  if (scenario == Scenario::MasonryArch25Reduced
      || scenario == Scenario::MasonryArch25ProjectileReduced
      || scenario == Scenario::MasonryArch101Reduced
      || scenario == Scenario::MasonryArch25FullManifold
      || scenario == Scenario::MasonryArch101FullManifold) {
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
  if (gWarmStartOverride == 0) {
    options.enableWarmStart = false;
  } else if (gWarmStartOverride == 1) {
    options.enableWarmStart = true;
  }
  return options;
}

void configureWorldSolver(
    const std::shared_ptr<dart::simulation::World>& world,
    SolverMode solverMode,
    std::size_t maxContacts,
    std::size_t maxContactsPerPair,
    Scenario scenario,
    TraceScope traceScope,
    double initialStepSize)
{
  world->setTimeStep(kDt);
  world->setGravity(Eigen::Vector3d(0.0, 0.0, -kGravity));
  world->setNumSimulationThreads(1u);

  dart::simulation::DeactivationOptions deactivation;
  deactivation.mEnabled = false;
  world->setDeactivationOptions(deactivation);

  if (solverMode == SolverMode::ExactFbf) {
    auto solver
        = std::make_unique<dart::constraint::ExactCoulombFbfConstraintSolver>(
            makeFbfOptions(initialStepSize, scenario, traceScope));
    solver->setCollisionDetector(dart::collision::DARTCollisionDetector::create());
    solver->setNumSimulationThreads(1u);
    world->setConstraintSolver(std::move(solver));
    // The long-run full-manifold scenarios separate position recovery from
    // the velocity-phase friction solve, matching the paper's formulation:
    // the exact solve then sees pure dynamics instead of DART's ERP bias.
    // Measured on the 26-card settle: steps 2+ fall back without this and
    // solve exactly at 1e-6 with it, ~30x faster. This must be set AFTER
    // installation because World::setConstraintSolver copies the previous
    // solver's split-impulse setting into the new one.
    if (scenario == Scenario::CardHouseFourLevelSettleProjectileFull
        || scenario == Scenario::MasonryArch25FullManifold
        || scenario == Scenario::MasonryArch101FullManifold) {
      world->getConstraintSolver()->setSplitImpulseEnabled(true);
    }
    // The exact velocity solve already enforces non-penetration, so the
    // wedged author arches need no penetration recovery at all; the
    // position pass's error-reduction push-apart (max ERV 0.1 m/s against
    // 7-10 cm voussoirs) otherwise inflates the arch apart while it seats
    // (measured: crown +3.4 cm in 0.5 s, collapse by 1.5 s, with or
    // without seating pre-stress).
    if (scenario == Scenario::MasonryArch25FullManifold
        || scenario == Scenario::MasonryArch101FullManifold) {
      dart::constraint::ContactConstraint::setErrorReductionParameter(0.0);
    }
  } else {
    auto* solver = world->getConstraintSolver();
    solver->setCollisionDetector(dart::collision::DARTCollisionDetector::create());
    solver->setNumSimulationThreads(1u);
    // Keep the boxed baseline comparable: the long-run full scenarios use the
    // same split-impulse stepping mode for both solver modes.
    if (scenario == Scenario::CardHouseFourLevelSettleProjectileFull
        || scenario == Scenario::MasonryArch25FullManifold
        || scenario == Scenario::MasonryArch101FullManifold) {
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
  auto body
      = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>().second;
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
  auto body
      = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>().second;
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
  joint->setLinearVelocity(
      Eigen::Vector3d(kPainleveInitialVelocity, 0.0, 0.0));
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
  const double halfSpan = 0.5 * kCardHouseHeight * std::sin(kCardHouseAngle);
  transform.translation().x()
      = centerX + (leftCard ? -0.96 * halfSpan : 0.96 * halfSpan);
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
  constexpr double kMass = 0.05;

  auto skeleton = dart::dynamics::Skeleton::create(name);
  dart::dynamics::GenericJoint<dart::math::SE3Space>::Properties jointProperties(
      name + "_joint");
  dart::dynamics::BodyNode::Properties bodyProperties(
      dart::dynamics::BodyNode::AspectProperties(name + "_body"));
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
  shapeNode->getDynamicsAspect()->setFrictionCoeff(kCardHouseFriction);
  joint->setPositions(dart::dynamics::FreeJoint::convertToPositions(transform));
  return skeleton;
}

dart::dynamics::SkeletonPtr createCardHouseProjectile(std::size_t index)
{
  auto skeleton
      = dart::dynamics::Skeleton::create("fbf_projectile_" + std::to_string(index));
  dart::dynamics::GenericJoint<dart::math::SE3Space>::Properties
      jointProperties(skeleton->getName() + "_joint");
  dart::dynamics::BodyNode::Properties bodyProperties(
      dart::dynamics::BodyNode::AspectProperties(skeleton->getName() + "_body"));
  bodyProperties.mInertia.setMass(kCardHouseProjectileMass);
  bodyProperties.mInertia.setMoment(dart::dynamics::SphereShape::computeInertia(
      kCardHouseProjectileRadius, kCardHouseProjectileMass));

  auto pair = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>(
      nullptr, jointProperties, bodyProperties);
  auto* joint = pair.first;
  auto* body = pair.second;
  auto* shapeNode = body->createShapeNodeWith<
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(
      std::make_shared<dart::dynamics::SphereShape>(kCardHouseProjectileRadius));
  shapeNode->getDynamicsAspect()->setFrictionCoeff(kCardHouseFriction);

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(
      -1.35,
      (static_cast<double>(index) - 1.5) * 0.15,
      0.35 + 0.28 * static_cast<double>(index));
  joint->setPositions(dart::dynamics::FreeJoint::convertToPositions(transform));
  joint->setLinearVelocity(
      Eigen::Vector3d(kCardHouseProjectileSpeed, 0.0, 0.0));
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
      jointProperties(
          "masonry_arch_stone_" + std::to_string(index) + "_joint");
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

  // All stones are fully dynamic; the source scene fixes only the ground
  // plane (see PROVENANCE.txt section 7 in the geometry-port task notes).
  const double mass = kArchDensity * geometry.size.x() * geometry.size.y()
                       * geometry.size.z();
  dart::dynamics::Inertia inertia;
  inertia.setMass(mass);
  inertia.setMoment(
      dart::dynamics::BoxShape::computeInertia(geometry.size, mass));
  body->setInertia(inertia);
  return skeleton;
}

dart::dynamics::SkeletonPtr createMasonryArchProjectile()
{
  auto skeleton = dart::dynamics::Skeleton::create("masonry_arch_projectile");
  dart::dynamics::GenericJoint<dart::math::SE3Space>::Properties
      jointProperties(skeleton->getName() + "_joint");
  dart::dynamics::BodyNode::Properties bodyProperties(
      dart::dynamics::BodyNode::AspectProperties(skeleton->getName() + "_body"));
  bodyProperties.mInertia.setMass(kArchProjectileMass);
  bodyProperties.mInertia.setMoment(dart::dynamics::SphereShape::computeInertia(
      kArchProjectileRadius, kArchProjectileMass));

  auto pair = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>(
      nullptr, jointProperties, bodyProperties);
  auto* joint = pair.first;
  auto* body = pair.second;
  auto* shapeNode = body->createShapeNodeWith<
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(
      std::make_shared<dart::dynamics::SphereShape>(kArchProjectileRadius));
  shapeNode->getDynamicsAspect()->setFrictionCoeff(kArchFriction);

  // Aim at the crown (topmost, narrowest) stone, just outside its depth
  // face, regardless of stone count.
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(
      0.0,
      -0.5 * kArchCrownWidth - kArchProjectileRadius + 0.005,
      kArchCrownHeight);
  joint->setPositions(dart::dynamics::FreeJoint::convertToPositions(transform));
  joint->setLinearVelocity(Eigen::Vector3d(0.0, kArchProjectileSpeed, 0.0));
  return skeleton;
}

void addMasonryArch(
    const std::shared_ptr<dart::simulation::World>& world,
    std::size_t stoneCount)
{
  // Author boundary condition: only the ground plane is fixed; every stone
  // (including both springers) is a fully dynamic rigid body (see
  // PROVENANCE.txt section 7 in the geometry-port task notes).
  world->addSkeleton(createHorizontalPlane(kArchFriction));

  const auto stoneGeometry
      = dart::math::detail::generateMasonryArchStoneBoxes(stoneCount);
  for (std::size_t i = 0u; i < stoneCount; ++i) {
    auto stone = createMasonryArchStone(i, stoneGeometry[i]);
    // Paper Fig. 7/8 setup: the two springer (endpoint) stones are pinned
    // and the interior stones are dynamic. The raw Rigid-IPC scene leaves
    // all stones dynamic, but under DART's sampled contact manifolds the
    // all-dynamic catenary spreads at the base during seating and slumps
    // (measured with ERP on, ERP off, 0.5 percent pre-stress, and a 0.2 mm
    // pre-seat); pinning the springers matches the paper's own described
    // configuration.
    if (i == 0u || i + 1u == stoneCount) {
      stone->setMobile(false);
    }
    world->addSkeleton(stone);
  }
}

std::shared_ptr<dart::simulation::World> createTraceWorld(
    Scenario scenario,
    SolverMode solverMode,
    TraceScope traceScope,
    double initialStepSize)
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
            : 4u;
  const std::size_t maxContactsPerPair
      = scenario == Scenario::CardHouseFourLevelReduced
              || scenario
                     == Scenario::CardHouseFourLevelSettleProjectileReduced
            ? kCardHouseReducedMaxContactsPerPair
        : scenario == Scenario::CardHouseFourLevelSettleProjectileFull
            // Measured: per-pair 8 produces identical settle manifolds
            // (108/122/134 contacts) and identical convergence to per-pair 4
            // on the first settle steps, so the promoted cap is kept.
            ? kCardHouseReducedMaxContactsPerPair
        : scenario == Scenario::MasonryArch25Reduced
              || scenario == Scenario::MasonryArch25ProjectileReduced
            ? kArchReducedMaxContactsPerPair
        : scenario == Scenario::MasonryArch101Reduced
            ? kArch101ReducedMaxContactsPerPair
        : scenario == Scenario::MasonryArch25FullManifold
              || scenario == Scenario::MasonryArch101FullManifold
            ? kArchFullManifoldMaxContactsPerPair
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
      initialStepSize);

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
      world->addSkeleton(createMasonryArchProjectile());
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

void launchCardHouseProjectiles(
    const std::shared_ptr<dart::simulation::World>& world)
{
  if (world->getSkeleton("fbf_projectile_0") != nullptr)
    return;

  for (std::size_t i = 0u; i < kCardHouseProjectileCount; ++i)
    world->addSkeleton(createCardHouseProjectile(i));
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

  if (!isTurntableScenario(scenario))
    return;

  auto turntable = world->getSkeleton("turntable");
  if (!turntable)
    return;

  auto* joint = static_cast<dart::dynamics::FreeJoint*>(turntable->getJoint(0));
  const double time = static_cast<double>(step) * kDt;
  const double ramp = std::min(time / kTurntableRampDuration, 1.0);
  joint->setAngularVelocity(
      Eigen::Vector3d(0.0, 0.0, ramp * turntableAngularVelocity(scenario)));
}

double safeValue(double value)
{
  return std::isfinite(value) ? value
                              : std::numeric_limits<double>::quiet_NaN();
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
    const std::shared_ptr<dart::simulation::World>& world)
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
      metrics.minCenterHeight
          = std::min(metrics.minCenterHeight, position.z());
      metrics.maxCardHorizontalTravel = std::max(
          metrics.maxCardHorizontalTravel, position.head<2>().norm());
    } else if (isCardHouseProjectileSkeletonName(skeleton->getName())) {
      ++metrics.projectileCount;
      metrics.maxProjectileSpeed = std::max(
          metrics.maxProjectileSpeed, body->getLinearVelocity().norm());
    }
  }

  if (metrics.cardCount == 0u) {
    metrics.minCardAxisUp = std::numeric_limits<double>::quiet_NaN();
    metrics.minCenterHeight = std::numeric_limits<double>::quiet_NaN();
  }

  return metrics;
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
  const double upZ = transform.linear().col(2).normalized().dot(
      Eigen::Vector3d::UnitZ());
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
      "Usage: fbf_paper_trace [scenario=backspin] [solver=exact_fbf] "
      "[sample_stride=30] [steps=paper_duration] [initial_gamma=nan] "
      "[trace_scope=tracked] [warm_start=default|0|1] "
      "[split_impulse=default|0|1]\n");
}

} // namespace

int main(int argc, char** argv)
{
  Scenario scenario = Scenario::Backspin;
  SolverMode solverMode = SolverMode::ExactFbf;
  TraceScope traceScope = TraceScope::TrackedBody;
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

  auto world
      = createTraceWorld(scenario, solverMode, traceScope, initialStepSize);
  const auto bodies = traceScope == TraceScope::ResidualHistory
                              || traceScope == TraceScope::PhaseSummary
                          ? std::vector<const dart::dynamics::BodyNode*>()
                          : collectTraceBodies(world, scenario, traceScope);
  if (traceScope != TraceScope::ResidualHistory
      && traceScope != TraceScope::PhaseSummary && bodies.empty()) {
    std::fprintf(stderr, "fbf_paper_trace could not find traced bodies\n");
    return 1;
  }

  auto* exactSolver
      = solverMode == SolverMode::ExactFbf ? getExactSolver(world) : nullptr;
  if (solverMode == SolverMode::ExactFbf && exactSolver == nullptr) {
    std::fprintf(stderr, "fbf_paper_trace could not install exact solver\n");
    return 1;
  }

  if (traceScope == TraceScope::ResidualHistory) {
    printResidualHistoryHeader();
  } else if (traceScope == TraceScope::PhaseSummary) {
    printPhaseSummaryHeader();
    printPhaseSummaryRow(0u, scenario, solverMode, world, exactSolver);
  } else {
    printHeader();
    printTraceRows(0u, scenario, solverMode, world, bodies, exactSolver);
  }
  for (std::size_t step = 0; step < steps; ++step) {
    applyScenarioControl(world, scenario, step);
    if (traceScope == TraceScope::ResidualHistory)
      exactSolver->clearExactCoulombResidualHistoryRecords();
    world->step();
    const std::size_t completedStep = step + 1u;
    if (completedStep % sampleStride == 0u || completedStep == steps) {
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
  }

  if (solverMode == SolverMode::ExactFbf) {
    const bool ok = exactSolver->getNumExactCoulombSolves() > 0u
                    && exactSolver->getNumBoxedLcpFallbacks() == 0u
                    && std::isfinite(
                        exactSolver->getLastExactCoulombResidual())
                    && exactSolver->getLastExactCoulombResidual() <= 1e-6;
    if (!ok)
      return 1;
  }

  return 0;
}
