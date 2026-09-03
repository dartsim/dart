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

#include "dart/simulation/common/exceptions.hpp"
#include "dart/simulation/comps/collision_geometry.hpp"
#include "dart/simulation/comps/contact_material.hpp"
#include "dart/simulation/comps/dynamics.hpp"
#include "dart/simulation/comps/frame_types.hpp"
#include "dart/simulation/comps/joint.hpp"
#include "dart/simulation/comps/rigid_body.hpp"
#include "dart/simulation/compute/compute_executor.hpp"
#include "dart/simulation/compute/detail/deformable_avbd_replay_state.hpp"
#include "dart/simulation/compute/detail/stage_scratch.hpp"
#include "dart/simulation/compute/detail/world_step_stages.hpp"
#include "dart/simulation/compute/rigid_body_constraint.hpp"
#include "dart/simulation/detail/entity_conversion.hpp"
#include "dart/simulation/detail/rigid_avbd/rigid_world_contact.hpp"
#include "dart/simulation/detail/rigid_contact/boxed_lcp_contact.hpp"
#include "dart/simulation/detail/rigid_contact/rigid_contact_assembly.hpp"
#include "dart/simulation/detail/rigid_pair_constraint.hpp"
#include "dart/simulation/detail/world_registry_access.hpp"
#include "dart/simulation/detail/world_storage.hpp"
#include "dart/simulation/world.hpp"
#include "dart/simulation/world_options.hpp"

#include <dart/common/memory_manager.hpp>
#include <dart/common/stl_allocator.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <entt/entt.hpp>

#include <algorithm>
#include <array>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <string_view>
#include <vector>

#include <cmath>
#include <cstdint>

namespace dart::simulation::compute {

namespace dvbd = dart::simulation::detail::deformable_vbd;

namespace {

const comps::RigidAvbdContactConfig* enabledRigidAvbdContactConfig(
    const detail::WorldRegistry& registry, entt::entity entity)
{
  const auto* config = registry.try_get<comps::RigidAvbdContactConfig>(entity);
  if (config == nullptr || !config->enabled) {
    return nullptr;
  }
  return config;
}

//==============================================================================
bool mayHaveRigidAvbdContactConfigs(const detail::WorldRegistry& registry)
{
  const auto* configStorage = registry.storage<comps::RigidAvbdContactConfig>();
  return configStorage != nullptr && configStorage->size() != 0u;
}

struct RigidAvbdScratchCapacityPlan
{
  std::size_t bodyCapacity = 0u;
  std::size_t contactTangentRowCapacity = 0u;
  std::size_t jointAxisRowCapacity = 0u;
};

//==============================================================================
std::size_t checkedRigidAvbdCapacityProduct(
    std::size_t value, std::size_t factor, std::string_view description)
{
  DART_SIMULATION_THROW_T_IF(
      factor != 0u && value > std::numeric_limits<std::size_t>::max() / factor,
      InvalidOperationException,
      "Rigid AVBD {} capacity is not representable",
      description);
  return value * factor;
}

//==============================================================================
std::size_t checkedRigidAvbdCapacitySum(
    std::size_t first, std::size_t second, std::string_view description)
{
  DART_SIMULATION_THROW_T_IF(
      first > std::numeric_limits<std::size_t>::max() - second,
      InvalidOperationException,
      "Rigid AVBD {} capacity is not representable",
      description);
  return first + second;
}

//==============================================================================
RigidAvbdScratchCapacityPlan makeRigidAvbdScratchCapacityPlan(
    std::size_t contactCapacity,
    std::size_t jointCapacity,
    std::size_t distanceSpringCapacity)
{
  RigidAvbdScratchCapacityPlan plan;
  plan.contactTangentRowCapacity = checkedRigidAvbdCapacityProduct(
      contactCapacity, 2u, "contact tangent-row");
  plan.jointAxisRowCapacity
      = checkedRigidAvbdCapacityProduct(jointCapacity, 3u, "joint axis-row");

  // The solve concatenates contacts, three linear joint rows, and one linear
  // motor row into the point-pair family. Its row-index adjacency stores two
  // endpoints per row. The angular family has the same 3+1 joint expansion.
  const std::size_t pointPairRowCapacity = checkedRigidAvbdCapacitySum(
      checkedRigidAvbdCapacitySum(
          contactCapacity, plan.jointAxisRowCapacity, "point-pair row"),
      jointCapacity,
      "point-pair row");
  const std::size_t angularPairRowCapacity = checkedRigidAvbdCapacitySum(
      plan.jointAxisRowCapacity, jointCapacity, "angular-pair row");
  static_cast<void>(checkedRigidAvbdCapacityProduct(
      pointPairRowCapacity, 2u, "point-pair row-index"));
  static_cast<void>(checkedRigidAvbdCapacityProduct(
      angularPairRowCapacity, 2u, "angular-pair row-index"));
  static_cast<void>(checkedRigidAvbdCapacityProduct(
      distanceSpringCapacity, 2u, "distance-spring row-index"));

  const std::size_t rowEndpointCapacity = checkedRigidAvbdCapacitySum(
      checkedRigidAvbdCapacitySum(
          contactCapacity, jointCapacity, "body endpoint"),
      distanceSpringCapacity,
      "body endpoint");
  plan.bodyCapacity = checkedRigidAvbdCapacityProduct(
      rowEndpointCapacity, 2u, "body endpoint");
  return plan;
}

//==============================================================================
std::optional<comps::RigidAvbdContactConfig> rigidAvbdContactStageConfig(
    const detail::WorldRegistry& registry,
    std::span<const Contact> contacts,
    bool selectAllContacts,
    const dvbd::AvbdRigidParameterProfile* ownedFamilyProfile = nullptr)
{
  const auto validateConfig = [](const comps::RigidAvbdContactConfig& config) {
    DART_SIMULATION_THROW_T_IF(
        !std::isfinite(config.startStiffness) || config.startStiffness < 0.0,
        InvalidOperationException,
        "AVBD rigid-contact start stiffness must be finite and non-negative");
    DART_SIMULATION_THROW_T_IF(
        !std::isfinite(config.alpha) || config.alpha < 0.0
            || config.alpha > 1.0,
        InvalidOperationException,
        "AVBD rigid-contact alpha must be finite and in [0, 1]");
    DART_SIMULATION_THROW_T_IF(
        !std::isfinite(config.beta) || config.beta < 0.0,
        InvalidOperationException,
        "AVBD rigid-contact beta must be finite and non-negative");
    DART_SIMULATION_THROW_T_IF(
        !std::isfinite(config.gamma) || config.gamma < 0.0
            || config.gamma > 1.0,
        InvalidOperationException,
        "AVBD rigid-contact gamma must be finite and in [0, 1]");
    DART_SIMULATION_THROW_T_IF(
        std::isnan(config.maxStiffness)
            || config.maxStiffness < config.startStiffness,
        InvalidOperationException,
        "AVBD rigid-contact maximum stiffness must be at least the start "
        "stiffness (positive infinity is allowed)");
  };
  const auto sameConfig = [](const comps::RigidAvbdContactConfig& lhs,
                             const comps::RigidAvbdContactConfig& rhs) {
    return lhs.enabled == rhs.enabled
           && lhs.startStiffness == rhs.startStiffness && lhs.alpha == rhs.alpha
           && lhs.beta == rhs.beta && lhs.gamma == rhs.gamma
           && lhs.maxStiffness == rhs.maxStiffness;
  };

  std::optional<comps::RigidAvbdContactConfig> uniformConfig;
  if (selectAllContacts) {
    uniformConfig.emplace();
    if (ownedFamilyProfile != nullptr) {
      // The main sweeps of a post-stabilized profile run with alpha 1 for
      // contacts exactly as for joints (the reference source's currentAlpha).
      uniformConfig->alpha
          = dvbd::avbdRigidParameterProfileSweepAlpha(*ownedFamilyProfile);
      uniformConfig->beta = ownedFamilyProfile->beta;
      uniformConfig->gamma = ownedFamilyProfile->gamma;
      if (std::isfinite(ownedFamilyProfile->startStiffness)) {
        uniformConfig->startStiffness = ownedFamilyProfile->startStiffness;
      }
      if (std::isfinite(ownedFamilyProfile->maxStiffness)) {
        uniformConfig->maxStiffness = std::max(
            uniformConfig->startStiffness, ownedFamilyProfile->maxStiffness);
      }
    }
    validateConfig(*uniformConfig);
    // Public VBD/AVBD own one solver-wide contact configuration. The private
    // per-body component is a compatibility opt-in for other public families;
    // stale compatibility data must neither refine nor veto an owned family.
    return uniformConfig;
  }

  for (const Contact& contact : contacts) {
    const auto* configA = enabledRigidAvbdContactConfig(
        registry, detail::toRegistryEntity(contact.bodyA.getEntity()));
    const auto* configB = enabledRigidAvbdContactConfig(
        registry, detail::toRegistryEntity(contact.bodyB.getEntity()));
    if (configA == nullptr && configB == nullptr) {
      return std::nullopt;
    }

    const auto absorb = [&](const comps::RigidAvbdContactConfig& config) {
      validateConfig(config);
      if (!uniformConfig.has_value()) {
        uniformConfig = config;
        return;
      }
      DART_SIMULATION_THROW_T_IF(
          !sameConfig(*uniformConfig, config),
          InvalidOperationException,
          "Per-body AVBD rigid-contact refinements differ within one active "
          "contact envelope; per-contact solver parameters are not yet "
          "supported");
    };

    if (configA != nullptr) {
      absorb(*configA);
    }
    if (configB != nullptr) {
      absorb(*configB);
    }
  }
  return uniformConfig;
}

//==============================================================================
/// Whether the block sweep starts from the adaptive initial guess of AVBD
/// Algorithm 1 line 4. Only the profiles that also start every row at the
/// reference sources' PENALTY_MIN do: with DART's configured 1e5 start
/// stiffness the guess makes a hard-jointed structure that lands on a support
/// pass through it (the stiff joints stall the sweep before the contact rows
/// can lift the structure back out), while at the reference start the sweep
/// reproduces the reference sources. The paper profile therefore keeps its
/// step-start sweep origin and its sealed Figure 13 outcome; see the PLAN-104
/// finding on Table 2's beta in SI units.
bool avbdAdaptiveInitialGuessEnabled(
    const dvbd::AvbdRigidParameterProfile& profile) noexcept
{
  return std::isfinite(profile.startStiffness);
}

//==============================================================================
dvbd::AvbdRigidWorldContactSolveOptions rigidAvbdWorldSolveOptions(
    std::size_t iterations,
    dvbd::AvbdRigidWorldContactSolveOptions::Formulation formulation,
    const dvbd::AvbdRigidParameterProfile* ownedFamilyProfile = nullptr,
    const comps::RigidAvbdContactConfig* contactConfig = nullptr,
    double contactMaxStiffness = std::numeric_limits<double>::infinity(),
    const dvbd::AvbdRigidParameterProfile* springScheduleProfile = nullptr)
{
  dvbd::AvbdRigidWorldContactSolveOptions options;
  options.descent.iterations = iterations;
  options.descent.regularization = 1e-12;
  options.formulation = formulation;
  if (ownedFamilyProfile != nullptr) {
    dvbd::applyAvbdRigidParameterProfile(options, *ownedFamilyProfile);
  } else {
    // Compatibility distance springs are AVBD rows under every rigid-body
    // family and their continuation survives AVBD <-> Sequential Impulse
    // crossings, so their warm start (Equation 19) and penalty ramp follow one
    // schedule, the immutable paper profile, whichever family owns the
    // contacts and hard joints. This branch serves both non-AVBD families:
    // Sequential Impulse owns its joints and motors itself, so the values reach
    // only the spring rows; fixed-penalty VBD still builds private point-joint
    // and motor rows, but its finite-material rows bypass the Equation 18
    // regularization and the ramp, and the fixed-penalty solve clears every
    // inventory before and after, so the values are inert there too.
    const dvbd::AvbdRigidParameterProfile& profile
        = springScheduleProfile != nullptr ? *springScheduleProfile
                                           : dvbd::kAvbdRigidPaper2025Profile;
    options.warmStart.alpha = profile.postStabilize ? 1.0 : profile.alpha;
    options.warmStart.gamma = profile.gamma;
    options.warmStart.lambdaRetention
        = profile.postStabilize ? 1.0
                                : std::numeric_limits<double>::quiet_NaN();
    options.distanceSpring.beta = profile.beta;
  }
  // Equation 18 must use the same alpha that Equation 19 used to warm-start
  // the persistent row. Splitting these values changes the represented
  // constraint between inventory synchronization and the solve.
  options.row.alpha = options.warmStart.alpha;
  options.friction.alpha = options.warmStart.alpha;

  if (contactConfig != nullptr) {
    options.hasContactFamilyOverride = true;
    options.contactWarmStart.alpha = contactConfig->alpha;
    options.contactWarmStart.gamma = contactConfig->gamma;
    options.contactWarmStart.maxStiffness = contactMaxStiffness;
    options.contactRow.alpha = contactConfig->alpha;
    options.contactRow.beta = contactConfig->beta;
    options.contactRow.maxStiffness = contactMaxStiffness;
    options.friction.alpha = contactConfig->alpha;
    options.friction.beta = contactConfig->beta;
    options.friction.maxStiffness = contactMaxStiffness;
  }
  return options;
}

//==============================================================================
std::optional<std::size_t> countProjectableRigidAvbdContacts(
    const detail::WorldRegistry& registry, std::span<const Contact> contacts)
{
  std::size_t count = 0u;
  for (const Contact& contact : contacts) {
    if (contact.depth <= 0.0) {
      continue;
    }
    if (!std::isfinite(contact.depth) || !contact.point.allFinite()
        || !contact.normal.allFinite() || contact.normal.squaredNorm() <= 0.0) {
      return std::nullopt;
    }

    const entt::entity entityA
        = detail::toRegistryEntity(contact.bodyA.getEntity());
    const entt::entity entityB
        = detail::toRegistryEntity(contact.bodyB.getEntity());
    if (entityA == entt::null || entityB == entt::null || entityA == entityB) {
      return std::nullopt;
    }

    const auto bodyA = dvbd::avbdRigidWorldProjectableBody(registry, entityA);
    const auto bodyB = dvbd::avbdRigidWorldProjectableBody(registry, entityB);
    if (!bodyA || !bodyB) {
      return std::nullopt;
    }
    if (!bodyA.isStatic || !bodyB.isStatic) {
      ++count;
    }
  }
  return count;
}

struct RigidContactCandidate
{
  entt::entity entity = entt::null;
  bool prescribedResponse = false;
};

//==============================================================================
void recordRigidContactEnvelopeMetrics(
    World& world,
    const detail::WorldRegistry& registry,
    std::span<const Contact> contacts)
{
  auto& metrics = detail::storageOf(world).lastStepDiagnostics;
  for (const auto& contact : contacts) {
    const auto entityA = detail::toRegistryEntity(contact.bodyA.getEntity());
    const auto entityB = detail::toRegistryEntity(contact.bodyB.getEntity());
    if (!registry.all_of<comps::RigidBodyTag>(entityA)
        || !registry.all_of<comps::RigidBodyTag>(entityB)) {
      continue;
    }
    const bool staticA
        = detail::hasPrescribedRigidBodyContactResponse(registry, entityA);
    const bool staticB
        = detail::hasPrescribedRigidBodyContactResponse(registry, entityB);
    if (staticA && staticB) {
      continue;
    }
    ++metrics.activeContactCount;
    metrics.maxPenetrationDepth
        = std::max(metrics.maxPenetrationDepth, std::max(0.0, contact.depth));
  }
}

//==============================================================================
void recordSolverDiagnostics(
    World& world, std::size_t iterations, double residual = 0.0)
{
  auto& metrics = detail::storageOf(world).lastStepDiagnostics;
  metrics.lastStepIterations += iterations;
  if (std::isfinite(residual)) {
    metrics.lastStepResidual = std::max(metrics.lastStepResidual, residual);
  }
}

//==============================================================================
// Records one world-space reaction force per solved rigid-body contact into the
// World's last-step storage so `World::getLastContactForces()` and the debug
// overlay can draw force arrows. Called after the velocity solve but before the
// position projection, so each `point` stays consistent with the assembled
// `armA` (only velocities, not poses, change during the solve). The reaction on
// `bodyB` is `impulse / timeStep`; `bodyA` feels the opposite.
void captureRigidContactForces(
    detail::WorldRegistry& registry,
    World& world,
    std::span<const RigidBodyContactConstraint> constraints,
    double timeStep)
{
  if (!(timeStep > 0.0) || constraints.empty()) {
    return;
  }
  const double inverseTimeStep = 1.0 / timeStep;
  auto& forces = detail::storageOf(world).lastContactForces;
  forces.reserve(forces.size() + constraints.size());
  for (const RigidBodyContactConstraint& constraint : constraints) {
    const Eigen::Vector3d impulse
        = constraint.normal * constraint.normalImpulse
          + constraint.tangent1 * constraint.tangentImpulse1
          + constraint.tangent2 * constraint.tangentImpulse2;
    if (!impulse.allFinite() || impulse.squaredNorm() <= 0.0) {
      continue;
    }
    ContactForce contactForce;
    contactForce.force = impulse * inverseTimeStep;
    contactForce.point
        = registry.get<comps::Transform>(constraint.bodyA).position
          + constraint.armA;
    contactForce.bodyA
        = CollisionBody(detail::fromRegistryEntity(constraint.bodyA), &world);
    contactForce.bodyB
        = CollisionBody(detail::fromRegistryEntity(constraint.bodyB), &world);
    forces.push_back(std::move(contactForce));
  }
}

//==============================================================================
// The boxed-LCP solver leaves its impulses in allocator-backed stacked scratch
// (normal rows [0, n), then two friction rows per contact). Copy them back onto
// the constraints to share the capture path above without materializing the
// public Eigen snapshot during an allocation-free baked step.
void writeBoxedLcpImpulsesIntoConstraints(
    std::span<RigidBodyContactConstraint> constraints,
    std::span<const double> impulses)
{
  const std::size_t n = constraints.size();
  if (n == 0u || impulses.size() < n) {
    return;
  }
  const bool hasFrictionRows = n <= std::numeric_limits<std::size_t>::max() / 3u
                               && impulses.size() == 3u * n;
  for (std::size_t i = 0; i < n; ++i) {
    RigidBodyContactConstraint& constraint = constraints[i];
    constraint.normalImpulse = impulses[i];
    if (hasFrictionRows) {
      constraint.tangentImpulse1 = impulses[n + 2u * i];
      constraint.tangentImpulse2 = impulses[n + 2u * i + 1u];
    } else {
      constraint.tangentImpulse1 = 0.0;
      constraint.tangentImpulse2 = 0.0;
    }
  }
}

//==============================================================================
detail::WorldStorage::CollisionPairKey makeCollisionPairKey(
    entt::entity first, entt::entity second)
{
  if (static_cast<std::uint32_t>(second) < static_cast<std::uint32_t>(first)) {
    std::swap(first, second);
  }
  return {first, second};
}

//==============================================================================
bool isRigidContactCandidate(
    const detail::WorldRegistry& registry,
    entt::entity entity,
    const comps::CollisionGeometry& geometry)
{
  if (!geometry.hasShapes()) {
    return false;
  }

  return registry.all_of<comps::RigidBodyTag>(entity)
         || registry.all_of<comps::LinkModel>(entity);
}

//==============================================================================
bool shouldSkipRigidBodyContactQuery(const World& world)
{
  constexpr std::size_t kIgnoredPairSkipAuditLimit = 64u;

  const auto& registry = detail::registryOf(world);
  const auto& ignoredCollisionPairs
      = detail::storageOf(world).ignoredCollisionPairs;
  const bool hasIgnoredCollisionPairs = !ignoredCollisionPairs.empty();
  const auto collisionGeometryView = registry.view<comps::CollisionGeometry>();

  std::array<RigidContactCandidate, kIgnoredPairSkipAuditLimit> candidates{};
  std::size_t candidateCount = 0u;
  bool candidateAuditOverflow = false;

  bool hasNonPrescribedCandidate = false;
  for (const entt::entity entity : collisionGeometryView) {
    const auto& geometry
        = collisionGeometryView.get<comps::CollisionGeometry>(entity);
    if (!isRigidContactCandidate(registry, entity, geometry)) {
      continue;
    }

    const bool prescribedResponse
        = detail::hasPrescribedRigidBodyContactResponse(registry, entity);
    if (!prescribedResponse) {
      if (!hasIgnoredCollisionPairs) {
        return false;
      }
      hasNonPrescribedCandidate = true;
    }

    if (hasIgnoredCollisionPairs) {
      if (candidateCount < candidates.size()) {
        candidates[candidateCount++]
            = RigidContactCandidate{entity, prescribedResponse};
      } else {
        candidateAuditOverflow = true;
      }
    }
  }

  if (!hasNonPrescribedCandidate) {
    return true;
  }

  if (candidateAuditOverflow) {
    return false;
  }

  for (std::size_t i = 0; i < candidateCount; ++i) {
    for (std::size_t j = i + 1; j < candidateCount; ++j) {
      if (candidates[i].prescribedResponse
          && candidates[j].prescribedResponse) {
        continue;
      }

      if (!ignoredCollisionPairs.contains(makeCollisionPairKey(
              candidates[i].entity, candidates[j].entity))) {
        return false;
      }
    }
  }

  return true;
}

//==============================================================================
// Positional correction (projection) for rigid-body normal contacts: removes
// residual penetration beyond a small allowance without injecting velocity, so
// resting stacks do not sink. Shared by the sequential-impulse and boxed-LCP
// contact paths; the sequential path inlines an equivalent loop over its
// precomputed constraints, while the LCP path drives this from the raw
// contacts. Only rigid-body/rigid-body pairs with at least one dynamic body
// are corrected.
void resolveRigidBodyContactPositions(
    detail::WorldRegistry& registry,
    std::span<const Contact> contacts,
    double /*timeStep*/)
{
  constexpr double allowance = detail::kRigidContactPositionAllowance;
  constexpr double correctionFactor
      = detail::kRigidContactPositionCorrectionFactor;
  for (const auto& contact : contacts) {
    const auto entityA = detail::toRegistryEntity(contact.bodyA.getEntity());
    const auto entityB = detail::toRegistryEntity(contact.bodyB.getEntity());
    if (!registry.all_of<comps::RigidBodyTag>(entityA)
        || !registry.all_of<comps::RigidBodyTag>(entityB)) {
      continue;
    }

    const double penetration = contact.depth - allowance;
    if (penetration <= 0.0) {
      continue;
    }

    const bool staticA
        = detail::hasPrescribedRigidBodyContactResponse(registry, entityA);
    const bool staticB
        = detail::hasPrescribedRigidBodyContactResponse(registry, entityB);
    const double invMassA
        = staticA ? 0.0
                  : detail::inverseMassOf(
                        registry.get<comps::MassProperties>(entityA));
    const double invMassB
        = staticB ? 0.0
                  : detail::inverseMassOf(
                        registry.get<comps::MassProperties>(entityB));
    const double totalInverseMass = invMassA + invMassB;
    if (totalInverseMass <= 0.0) {
      continue;
    }

    const Eigen::Vector3d correction
        = (correctionFactor * penetration / totalInverseMass) * contact.normal;
    registry.get<comps::Transform>(entityA).position -= invMassA * correction;
    registry.get<comps::Transform>(entityB).position += invMassB * correction;
  }
}

struct RigidSequentialImpulseJointConstraint
{
  entt::entity joint = entt::null;
  entt::entity bodyA = entt::null;
  entt::entity bodyB = entt::null;
  Eigen::Vector3d localAnchorA = Eigen::Vector3d::Zero();
  Eigen::Vector3d localAnchorB = Eigen::Vector3d::Zero();
  Eigen::Quaterniond targetRelativeOrientation = Eigen::Quaterniond::Identity();
  Eigen::Matrix3d linearAxes = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d angularAxes = Eigen::Matrix3d::Identity();
  std::uint8_t linearAxisMask = detail::kRigidPairConstraintAllAxesMask;
  std::uint8_t angularAxisMask = detail::kRigidPairConstraintAllAxesMask;
  bool staticA = false;
  bool staticB = false;
  double invMassA = 0.0;
  double invMassB = 0.0;
  Eigen::Matrix3d invInertiaA = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d invInertiaB = Eigen::Matrix3d::Zero();
  bool useLinearMotor = false;
  bool useAngularMotor = false;
  double motorTargetSpeed = 0.0;
  double motorMaxForce = std::numeric_limits<double>::infinity();
  double motorMaxTorque = std::numeric_limits<double>::infinity();
  double breakForce = 0.0;
  std::array<double, 3> linearImpulses{};
  std::array<double, 3> angularImpulses{};
  double linearMotorImpulse = 0.0;
  double angularMotorImpulse = 0.0;
};

//==============================================================================
bool rigidSequentialImpulseJointAxisEnabled(std::uint8_t mask, int axis)
{
  return (mask & (std::uint8_t{1u} << static_cast<unsigned int>(axis))) != 0u;
}

//==============================================================================
Eigen::Quaterniond normalizedRigidSequentialImpulseOrientation(
    const Eigen::Quaterniond& orientation)
{
  return detail::normalizeRigidPairConstraintOrientation(orientation);
}

//==============================================================================
Eigen::Vector3d rigidSequentialImpulseJointWorldArm(
    const comps::Transform& transform, const Eigen::Vector3d& localAnchor)
{
  return normalizedRigidSequentialImpulseOrientation(transform.orientation)
         * localAnchor;
}

//==============================================================================
Eigen::Vector3d rigidSequentialImpulseJointAngularVelocity(
    const comps::Velocity& velocity, bool prescribed)
{
  return prescribed ? Eigen::Vector3d::Zero() : velocity.angular;
}

//==============================================================================
std::size_t expectedRigidSequentialImpulseJointCount(
    const detail::WorldRegistry& registry)
{
  std::size_t expected = 0u;
  const auto view = registry.view<
      comps::JointModel,
      comps::JointState,
      comps::JointActuation>();
  for (const entt::entity entity : view) {
    const auto& model = view.get<comps::JointModel>(entity);
    const auto& state = view.get<comps::JointState>(entity);
    if (state.broken || model.parentLink == entt::null
        || model.childLink == entt::null
        || model.parentLink == model.childLink) {
      continue;
    }

    const bool rigidA = registry.all_of<comps::RigidBodyTag>(model.parentLink);
    const bool rigidB = registry.all_of<comps::RigidBodyTag>(model.childLink);
    if (rigidA && rigidB) {
      DART_SIMULATION_THROW_T_IF(
          !model.hasRigidBodyPairConstraintGeometry,
          InvalidOperationException,
          "Sequential-impulse rigid joint is missing its canonical pair-row "
          "geometry");
    }

    // The count must accept exactly the joints the shared extractor accepts,
    // including its degenerate and non-finite skip rules, so a legitimately
    // skipped joint never trips the coverage check mid-step.
    const std::optional<detail::RigidPairConstraintEligibility> eligibility
        = detail::eligibleRigidPairConstraint(
            registry,
            entity,
            model,
            state,
            [](entt::entity, const comps::JointModel& joint) {
              return detail::rigidPairConstraintGeometryFromJointModel(joint);
            });
    if (eligibility.has_value()) {
      ++expected;
    }
  }
  return expected;
}

//==============================================================================
double rigidSequentialImpulseLinearEffectiveMass(
    const RigidSequentialImpulseJointConstraint& constraint,
    const Eigen::Vector3d& armA,
    const Eigen::Vector3d& armB,
    const Eigen::Vector3d& axis)
{
  const Eigen::Vector3d crossA = armA.cross(axis);
  const Eigen::Vector3d crossB = armB.cross(axis);
  return constraint.invMassA + constraint.invMassB
         + axis.dot(
             (constraint.invInertiaA * crossA).cross(armA)
             + (constraint.invInertiaB * crossB).cross(armB));
}

//==============================================================================
double rigidSequentialImpulseAngularEffectiveMass(
    const RigidSequentialImpulseJointConstraint& constraint,
    const Eigen::Vector3d& axis)
{
  return axis.dot((constraint.invInertiaA + constraint.invInertiaB) * axis);
}

//==============================================================================
void applyRigidSequentialImpulseJointLinearImpulse(
    detail::WorldRegistry& registry,
    const RigidSequentialImpulseJointConstraint& constraint,
    const Eigen::Vector3d& armA,
    const Eigen::Vector3d& armB,
    const Eigen::Vector3d& impulse)
{
  auto& velocityA = registry.get<comps::Velocity>(constraint.bodyA);
  auto& velocityB = registry.get<comps::Velocity>(constraint.bodyB);
  velocityB.linear += constraint.invMassB * impulse;
  velocityB.angular += constraint.invInertiaB * armB.cross(impulse);
  velocityA.linear -= constraint.invMassA * impulse;
  velocityA.angular -= constraint.invInertiaA * armA.cross(impulse);
}

//==============================================================================
void applyRigidSequentialImpulseJointAngularImpulse(
    detail::WorldRegistry& registry,
    const RigidSequentialImpulseJointConstraint& constraint,
    const Eigen::Vector3d& impulse)
{
  auto& velocityA = registry.get<comps::Velocity>(constraint.bodyA);
  auto& velocityB = registry.get<comps::Velocity>(constraint.bodyB);
  velocityB.angular += constraint.invInertiaB * impulse;
  velocityA.angular -= constraint.invInertiaA * impulse;
}

//==============================================================================
template <typename ConstraintVector>
std::size_t assembleRigidSequentialImpulseJointsInto(
    ConstraintVector& constraints,
    const detail::WorldRegistry& registry,
    std::span<const detail::RigidPairConstraintInput> inputs)
{
  constraints.clear();
  constraints.reserve(inputs.size());
  std::size_t activeRows = 0u;

  for (const auto& input : inputs) {
    const auto* model = registry.try_get<comps::JointModel>(input.joint);
    const auto* state = registry.try_get<comps::JointState>(input.joint);
    DART_SIMULATION_THROW_T_IF(
        model == nullptr || state == nullptr || state->broken,
        InvalidOperationException,
        "Sequential-impulse rigid joint extraction produced a stale or "
        "broken joint row");
    const bool validBodyA = registry.all_of<
        comps::RigidBodyTag,
        comps::Transform,
        comps::Velocity,
        comps::MassProperties>(input.bodyA);
    const bool validBodyB = registry.all_of<
        comps::RigidBodyTag,
        comps::Transform,
        comps::Velocity,
        comps::MassProperties>(input.bodyB);
    DART_SIMULATION_THROW_T_IF(
        input.bodyA == entt::null || input.bodyB == entt::null
            || input.bodyA == input.bodyB || !validBodyA || !validBodyB,
        NotImplementedException,
        "Sequential-impulse rigid pair constraints require two distinct free "
        "rigid-body endpoints");
    DART_SIMULATION_THROW_T_IF(
        !input.anchorsAreLocal || !input.anchorA.allFinite()
            || !input.anchorB.allFinite()
            || !input.targetRelativeOrientation.coeffs().allFinite()
            || input.targetRelativeOrientation.norm() == 0.0
            || !input.linearAxes.allFinite() || !input.angularAxes.allFinite(),
        InvalidOperationException,
        "Sequential-impulse rigid pair constraint geometry is invalid");

    RigidSequentialImpulseJointConstraint constraint;
    constraint.joint = input.joint;
    constraint.bodyA = input.bodyA;
    constraint.bodyB = input.bodyB;
    constraint.localAnchorA = input.anchorA;
    constraint.localAnchorB = input.anchorB;
    constraint.targetRelativeOrientation
        = normalizedRigidSequentialImpulseOrientation(
            input.targetRelativeOrientation);
    constraint.linearAxisMask = input.linearAxisMask;
    constraint.angularAxisMask = input.angularAxisMask;
    for (int axis = 0; axis < 3; ++axis) {
      if (rigidSequentialImpulseJointAxisEnabled(
              constraint.linearAxisMask, axis)) {
        const double norm = input.linearAxes.col(axis).norm();
        DART_SIMULATION_THROW_T_IF(
            !(norm > 0.0) || !std::isfinite(norm),
            InvalidOperationException,
            "Sequential-impulse rigid joint has an invalid linear row axis");
        constraint.linearAxes.col(axis) = input.linearAxes.col(axis) / norm;
        ++activeRows;
      }
      if (rigidSequentialImpulseJointAxisEnabled(
              constraint.angularAxisMask, axis)) {
        const double norm = input.angularAxes.col(axis).norm();
        DART_SIMULATION_THROW_T_IF(
            !(norm > 0.0) || !std::isfinite(norm),
            InvalidOperationException,
            "Sequential-impulse rigid joint has an invalid angular row axis");
        constraint.angularAxes.col(axis) = input.angularAxes.col(axis) / norm;
        ++activeRows;
      }
    }

    if (input.useLinearMotor) {
      const double norm = input.linearAxes.col(2).norm();
      DART_SIMULATION_THROW_T_IF(
          !(norm > 0.0) || !std::isfinite(norm)
              || !std::isfinite(input.motorTargetSpeed)
              || !(input.motorMaxForce > 0.0)
              || std::isnan(input.motorMaxForce),
          InvalidOperationException,
          "Sequential-impulse rigid joint has an invalid linear motor row");
      constraint.linearAxes.col(2) = input.linearAxes.col(2) / norm;
    }
    if (input.useAngularMotor) {
      const double norm = input.angularAxes.col(2).norm();
      DART_SIMULATION_THROW_T_IF(
          !(norm > 0.0) || !std::isfinite(norm)
              || !std::isfinite(input.motorTargetSpeed)
              || !(input.motorMaxTorque > 0.0)
              || std::isnan(input.motorMaxTorque),
          InvalidOperationException,
          "Sequential-impulse rigid joint has an invalid angular motor row");
      constraint.angularAxes.col(2) = input.angularAxes.col(2) / norm;
    }

    constraint.staticA
        = detail::hasPrescribedRigidBodyContactResponse(registry, input.bodyA);
    constraint.staticB
        = detail::hasPrescribedRigidBodyContactResponse(registry, input.bodyB);
    const auto& massA = registry.get<comps::MassProperties>(input.bodyA);
    const auto& massB = registry.get<comps::MassProperties>(input.bodyB);
    const auto& transformA = registry.get<comps::Transform>(input.bodyA);
    const auto& transformB = registry.get<comps::Transform>(input.bodyB);
    constraint.invMassA
        = constraint.staticA ? 0.0 : detail::inverseMassOf(massA);
    constraint.invMassB
        = constraint.staticB ? 0.0 : detail::inverseMassOf(massB);
    constraint.invInertiaA
        = constraint.staticA ? Eigen::Matrix3d::Zero()
                             : detail::inverseWorldInertiaOf(massA, transformA);
    constraint.invInertiaB
        = constraint.staticB ? Eigen::Matrix3d::Zero()
                             : detail::inverseWorldInertiaOf(massB, transformB);
    DART_SIMULATION_THROW_T_IF(
        constraint.staticA && constraint.staticB,
        InvalidOperationException,
        "Sequential-impulse rigid pair constraint has no dynamic endpoint");

    constraint.useLinearMotor = input.useLinearMotor;
    constraint.useAngularMotor = input.useAngularMotor;
    constraint.motorTargetSpeed = input.motorTargetSpeed;
    constraint.motorMaxForce = input.motorMaxForce;
    constraint.motorMaxTorque = input.motorMaxTorque;
    constraint.breakForce = model->breakForce;
    if (constraint.useLinearMotor) {
      ++activeRows;
    }
    if (constraint.useAngularMotor) {
      ++activeRows;
    }
    constraints.push_back(constraint);
  }

  return activeRows;
}

//==============================================================================
template <typename ConstraintVector>
void sweepRigidSequentialImpulseJoints(
    detail::WorldRegistry& registry,
    ConstraintVector& constraints,
    double timeStep)
{
  for (auto& constraint : constraints) {
    const auto& transformA = registry.get<comps::Transform>(constraint.bodyA);
    const auto& transformB = registry.get<comps::Transform>(constraint.bodyB);
    const Eigen::Vector3d armA = rigidSequentialImpulseJointWorldArm(
        transformA, constraint.localAnchorA);
    const Eigen::Vector3d armB = rigidSequentialImpulseJointWorldArm(
        transformB, constraint.localAnchorB);

    for (int row = 0; row < 3; ++row) {
      if (!rigidSequentialImpulseJointAxisEnabled(
              constraint.linearAxisMask, row)) {
        continue;
      }
      const Eigen::Vector3d axis = constraint.linearAxes.col(row);
      const double effectiveMass = rigidSequentialImpulseLinearEffectiveMass(
          constraint, armA, armB, axis);
      if (!(effectiveMass > 0.0) || !std::isfinite(effectiveMass)) {
        continue;
      }

      const auto& velocityA = registry.get<comps::Velocity>(constraint.bodyA);
      const auto& velocityB = registry.get<comps::Velocity>(constraint.bodyB);
      const double relativeVelocity = (computeRigidBodyContactPointVelocity(
                                           velocityB, armB, constraint.staticB)
                                       - computeRigidBodyContactPointVelocity(
                                           velocityA, armA, constraint.staticA))
                                          .dot(axis);
      const double lambda = -relativeVelocity / effectiveMass;
      if (!std::isfinite(lambda) || lambda == 0.0) {
        continue;
      }
      constraint.linearImpulses[static_cast<std::size_t>(row)] += lambda;
      applyRigidSequentialImpulseJointLinearImpulse(
          registry, constraint, armA, armB, lambda * axis);
    }

    for (int row = 0; row < 3; ++row) {
      if (!rigidSequentialImpulseJointAxisEnabled(
              constraint.angularAxisMask, row)) {
        continue;
      }
      const Eigen::Vector3d axis = constraint.angularAxes.col(row);
      const double effectiveMass
          = rigidSequentialImpulseAngularEffectiveMass(constraint, axis);
      if (!(effectiveMass > 0.0) || !std::isfinite(effectiveMass)) {
        continue;
      }

      const auto& velocityA = registry.get<comps::Velocity>(constraint.bodyA);
      const auto& velocityB = registry.get<comps::Velocity>(constraint.bodyB);
      const double relativeVelocity
          = (rigidSequentialImpulseJointAngularVelocity(
                 velocityB, constraint.staticB)
             - rigidSequentialImpulseJointAngularVelocity(
                 velocityA, constraint.staticA))
                .dot(axis);
      const double lambda = -relativeVelocity / effectiveMass;
      if (!std::isfinite(lambda) || lambda == 0.0) {
        continue;
      }
      constraint.angularImpulses[static_cast<std::size_t>(row)] += lambda;
      applyRigidSequentialImpulseJointAngularImpulse(
          registry, constraint, lambda * axis);
    }

    if (constraint.useLinearMotor) {
      const Eigen::Vector3d axis = constraint.linearAxes.col(2);
      const double effectiveMass = rigidSequentialImpulseLinearEffectiveMass(
          constraint, armA, armB, axis);
      if (effectiveMass > 0.0 && std::isfinite(effectiveMass)) {
        const auto& velocityA = registry.get<comps::Velocity>(constraint.bodyA);
        const auto& velocityB = registry.get<comps::Velocity>(constraint.bodyB);
        const double relativeVelocity
            = (computeRigidBodyContactPointVelocity(
                   velocityB, armB, constraint.staticB)
               - computeRigidBodyContactPointVelocity(
                   velocityA, armA, constraint.staticA))
                  .dot(axis);
        const double limit = constraint.motorMaxForce * timeStep;
        const double candidate
            = constraint.linearMotorImpulse
              + (constraint.motorTargetSpeed - relativeVelocity)
                    / effectiveMass;
        const double clamped = std::clamp(candidate, -limit, limit);
        const double lambda = clamped - constraint.linearMotorImpulse;
        constraint.linearMotorImpulse = clamped;
        if (std::isfinite(lambda) && lambda != 0.0) {
          applyRigidSequentialImpulseJointLinearImpulse(
              registry, constraint, armA, armB, lambda * axis);
        }
      }
    }

    if (constraint.useAngularMotor) {
      const Eigen::Vector3d axis = constraint.angularAxes.col(2);
      const double effectiveMass
          = rigidSequentialImpulseAngularEffectiveMass(constraint, axis);
      if (effectiveMass > 0.0 && std::isfinite(effectiveMass)) {
        const auto& velocityA = registry.get<comps::Velocity>(constraint.bodyA);
        const auto& velocityB = registry.get<comps::Velocity>(constraint.bodyB);
        const double relativeVelocity
            = (rigidSequentialImpulseJointAngularVelocity(
                   velocityB, constraint.staticB)
               - rigidSequentialImpulseJointAngularVelocity(
                   velocityA, constraint.staticA))
                  .dot(axis);
        const double limit = constraint.motorMaxTorque * timeStep;
        const double candidate
            = constraint.angularMotorImpulse
              + (constraint.motorTargetSpeed - relativeVelocity)
                    / effectiveMass;
        const double clamped = std::clamp(candidate, -limit, limit);
        const double lambda = clamped - constraint.angularMotorImpulse;
        constraint.angularMotorImpulse = clamped;
        if (std::isfinite(lambda) && lambda != 0.0) {
          applyRigidSequentialImpulseJointAngularImpulse(
              registry, constraint, lambda * axis);
        }
      }
    }
  }
}

//==============================================================================
template <typename ConstraintVector>
std::size_t markBrokenRigidSequentialImpulseJoints(
    detail::WorldRegistry& registry,
    const ConstraintVector& constraints,
    double timeStep)
{
  if (!(timeStep > 0.0) || !std::isfinite(timeStep)) {
    return 0u;
  }

  const double inverseTimeStep = 1.0 / timeStep;
  std::size_t broken = 0u;
  for (const auto& constraint : constraints) {
    if (!(constraint.breakForce > 0.0)
        || !std::isfinite(constraint.breakForce)) {
      continue;
    }

    double impulseSquared = 0.0;
    for (const double impulse : constraint.linearImpulses) {
      impulseSquared += impulse * impulse;
    }
    for (const double impulse : constraint.angularImpulses) {
      impulseSquared += impulse * impulse;
    }
    impulseSquared
        += constraint.linearMotorImpulse * constraint.linearMotorImpulse;
    impulseSquared
        += constraint.angularMotorImpulse * constraint.angularMotorImpulse;
    const double load = std::sqrt(impulseSquared) * inverseTimeStep;
    if (load < constraint.breakForce) {
      continue;
    }

    auto& state = registry.get<comps::JointState>(constraint.joint);
    if (!state.broken) {
      state.broken = true;
      ++broken;
    }
  }
  return broken;
}

//==============================================================================
void applyRigidSequentialImpulseOrientationCorrection(
    comps::Transform& transform, const Eigen::Vector3d& rotationVector)
{
  const double angle = rotationVector.norm();
  if (!(angle > 0.0) || !std::isfinite(angle)) {
    return;
  }

  Eigen::Quaterniond correction(
      Eigen::AngleAxisd(angle, rotationVector / angle));
  transform.orientation = normalizedRigidSequentialImpulseOrientation(
      correction
      * normalizedRigidSequentialImpulseOrientation(transform.orientation));
}

//==============================================================================
template <typename ConstraintVector>
void postStabilizeRigidSequentialImpulseJoints(
    detail::WorldRegistry& registry,
    const ConstraintVector& constraints,
    std::size_t iterations)
{
  constexpr double correctionFactor
      = detail::kRigidContactPositionCorrectionFactor;
  for (std::size_t iteration = 0; iteration < iterations; ++iteration) {
    for (const auto& constraint : constraints) {
      if (registry.get<comps::JointState>(constraint.joint).broken) {
        continue;
      }

      auto& transformA = registry.get<comps::Transform>(constraint.bodyA);
      auto& transformB = registry.get<comps::Transform>(constraint.bodyB);
      for (int row = 0; row < 3; ++row) {
        if (!rigidSequentialImpulseJointAxisEnabled(
                constraint.linearAxisMask, row)) {
          continue;
        }

        const Eigen::Vector3d armA = rigidSequentialImpulseJointWorldArm(
            transformA, constraint.localAnchorA);
        const Eigen::Vector3d armB = rigidSequentialImpulseJointWorldArm(
            transformB, constraint.localAnchorB);
        const Eigen::Vector3d axis = constraint.linearAxes.col(row);
        const double effectiveMass = rigidSequentialImpulseLinearEffectiveMass(
            constraint, armA, armB, axis);
        if (!(effectiveMass > 0.0) || !std::isfinite(effectiveMass)) {
          continue;
        }

        const Eigen::Vector3d anchorA = transformA.position + armA;
        const Eigen::Vector3d anchorB = transformB.position + armB;
        const double error = axis.dot(anchorB - anchorA);
        const double lambda = -correctionFactor * error / effectiveMass;
        if (!std::isfinite(lambda) || lambda == 0.0) {
          continue;
        }
        const Eigen::Vector3d correction = lambda * axis;
        transformB.position += constraint.invMassB * correction;
        transformA.position -= constraint.invMassA * correction;
        applyRigidSequentialImpulseOrientationCorrection(
            transformB, constraint.invInertiaB * armB.cross(correction));
        applyRigidSequentialImpulseOrientationCorrection(
            transformA, -constraint.invInertiaA * armA.cross(correction));
      }

      for (int row = 0; row < 3; ++row) {
        if (!rigidSequentialImpulseJointAxisEnabled(
                constraint.angularAxisMask, row)) {
          continue;
        }

        const Eigen::Vector3d axis = constraint.angularAxes.col(row);
        const double effectiveMass
            = rigidSequentialImpulseAngularEffectiveMass(constraint, axis);
        if (!(effectiveMass > 0.0) || !std::isfinite(effectiveMass)) {
          continue;
        }
        const Eigen::Quaterniond orientationA
            = normalizedRigidSequentialImpulseOrientation(
                transformA.orientation);
        const Eigen::Quaterniond targetOrientationB
            = normalizedRigidSequentialImpulseOrientation(
                orientationA * constraint.targetRelativeOrientation);
        const double error = axis.dot(
            detail::rigidPairConstraintOrientationError(
                transformB.orientation, targetOrientationB));
        const double lambda = -correctionFactor * error / effectiveMass;
        if (!std::isfinite(lambda) || lambda == 0.0) {
          continue;
        }
        const Eigen::Vector3d correction = lambda * axis;
        applyRigidSequentialImpulseOrientationCorrection(
            transformB, constraint.invInertiaB * correction);
        applyRigidSequentialImpulseOrientationCorrection(
            transformA, -constraint.invInertiaA * correction);
      }
    }
  }
}

} // namespace

struct RigidBodyContactStage::AvbdScratch
{
  using PointJointAllocator
      = common::StlAllocator<dvbd::AvbdRigidWorldPointJointInput>;
  using DistanceSpringAllocator
      = common::StlAllocator<dvbd::AvbdRigidWorldDistanceSpringInput>;
  using ProjectedVelocityVector
      = avbd_replay::RigidAvbdWarmStartReplayState::ProjectedVelocityVector;
  using ProjectedVelocityAllocator
      = avbd_replay::RigidAvbdWarmStartReplayState::ProjectedVelocityAllocator;

  AvbdScratch() = default;

  explicit AvbdScratch(common::MemoryAllocator& allocator)
    : snapshot(allocator),
      pointJoints(PointJointAllocator{allocator}),
      distanceSprings(DistanceSpringAllocator{allocator}),
      buildScratch(allocator),
      solveScratch(allocator),
      normalInventory(allocator),
      frictionInventory(allocator),
      jointLinearInventory(allocator),
      jointAngularInventory(allocator),
      motorInventory(allocator),
      distanceSpringInventory(allocator),
      projectedVelocities(ProjectedVelocityAllocator{allocator}),
      projectedVelocityScratch(ProjectedVelocityAllocator{allocator})
  {
  }

  void clear()
  {
    dvbd::clearAvbdRigidWorldContactSnapshot(snapshot);
    projectedVelocities.clear();
    projectedVelocityScratch.clear();
    pointJoints.clear();
    distanceSprings.clear();
    buildScratch.rowCounters.clear();
    buildScratch.contactRowOrder.clear();
    normalInventory.records().clear();
    frictionInventory.records().clear();
    jointLinearInventory.records().clear();
    jointAngularInventory.records().clear();
    motorInventory.records().clear();
    distanceSpringInventory.records().clear();
    solveScratch.clear();
    solveScratch.clearContinuationState();
  }

  void clearPointJointWarmStart()
  {
    pointJoints.clear();
    jointLinearInventory.records().clear();
    jointAngularInventory.records().clear();
    motorInventory.records().clear();
  }

  void clearContactWarmStart()
  {
    normalInventory.records().clear();
    frictionInventory.records().clear();
    solveScratch.clearContinuationState();
  }

  void clearSequentialImpulseOwnedWarmStart()
  {
    clearContactWarmStart();
    clearPointJointWarmStart();
    // A step the block family did not own projected no velocities, so the
    // adaptive initial guess restarts from a zero gravity weight.
    projectedVelocities.clear();
  }

  void reserve(
      const RigidAvbdScratchCapacityPlan& capacityPlan,
      std::size_t contactCapacity,
      std::size_t jointCapacity,
      std::size_t distanceSpringCapacity)
  {
    dvbd::reserveAvbdRigidWorldContactSnapshot(
        snapshot,
        capacityPlan.bodyCapacity,
        contactCapacity,
        jointCapacity,
        jointCapacity,
        distanceSpringCapacity);
    pointJoints.reserve(jointCapacity);
    distanceSprings.reserve(distanceSpringCapacity);
    buildScratch.rowCounters.reserve(
        std::max(
            contactCapacity, std::max(jointCapacity, distanceSpringCapacity)));
    buildScratch.contactRowOrder.reserve(contactCapacity);
    dvbd::reserveAvbdRigidWorldContactSolveScratch(
        solveScratch,
        contactCapacity,
        jointCapacity,
        jointCapacity,
        capacityPlan.bodyCapacity,
        distanceSpringCapacity);
    normalInventory.reserve(contactCapacity);
    frictionInventory.reserve(capacityPlan.contactTangentRowCapacity);
    jointLinearInventory.reserve(capacityPlan.jointAxisRowCapacity);
    jointAngularInventory.reserve(capacityPlan.jointAxisRowCapacity);
    motorInventory.reserve(jointCapacity);
    distanceSpringInventory.reserve(distanceSpringCapacity);
  }

  dvbd::AvbdRigidWorldContactSnapshot snapshot;
  std::vector<dvbd::AvbdRigidWorldPointJointInput, PointJointAllocator>
      pointJoints;
  std::vector<dvbd::AvbdRigidWorldDistanceSpringInput, DistanceSpringAllocator>
      distanceSprings;
  dvbd::AvbdRigidWorldContactBuildScratch buildScratch;
  dvbd::AvbdRigidWorldContactSolveScratch solveScratch;
  dvbd::AvbdScalarRowInventory normalInventory;
  dvbd::AvbdScalarRowInventory frictionInventory;
  dvbd::AvbdScalarRowInventory jointLinearInventory;
  dvbd::AvbdScalarRowInventory jointAngularInventory;
  dvbd::AvbdScalarRowInventory motorInventory;
  dvbd::AvbdScalarRowInventory distanceSpringInventory;
  /// Projected linear velocities of the last two owned steps (adaptive
  /// initial guess history) and its rebuild scratch.
  ProjectedVelocityVector projectedVelocities;
  ProjectedVelocityVector projectedVelocityScratch;
};

//==============================================================================
struct RigidBodyContactStage::ContactScratch
{
  using RigidPairInputAllocator
      = common::StlAllocator<detail::RigidPairConstraintInput>;
  using SequentialImpulseJointAllocator
      = common::StlAllocator<RigidSequentialImpulseJointConstraint>;

  ContactScratch() = default;

  explicit ContactScratch(common::MemoryAllocator& allocator)
    : problem(allocator),
      rigidPairInputs(RigidPairInputAllocator{allocator}),
      sequentialImpulseJoints(SequentialImpulseJointAllocator{allocator})
  {
  }

  RigidBodyContactProblem problem;
  std::vector<detail::RigidPairConstraintInput, RigidPairInputAllocator>
      rigidPairInputs;
  std::vector<
      RigidSequentialImpulseJointConstraint,
      SequentialImpulseJointAllocator>
      sequentialImpulseJoints;
};

//==============================================================================
RigidBodyContactStage::RigidBodyContactStage(std::size_t iterations)
  : RigidBodyContactStage(iterations, nullptr)
{
}

//==============================================================================
RigidBodyContactStage::RigidBodyContactStage(
    std::size_t iterations, common::MemoryManager* memoryManager)
  : m_iterations(std::max<std::size_t>(1, iterations)),
    m_memoryManager(memoryManager),
    m_avbdScratch(
        createAvbdScratch(memoryManager), AvbdScratchDeleter{memoryManager}),
    m_contactScratch(
        createContactScratch(memoryManager),
        ContactScratchDeleter{memoryManager})
{
}

//==============================================================================
RigidBodyContactStage::~RigidBodyContactStage() = default;

//==============================================================================
void RigidBodyContactStage::AvbdScratchDeleter::operator()(
    AvbdScratch* scratch) const noexcept
{
  stage_detail::destroyStageOwnedScratch(memoryManager, scratch);
}

//==============================================================================
void RigidBodyContactStage::ContactScratchDeleter::operator()(
    ContactScratch* scratch) const noexcept
{
  stage_detail::destroyStageOwnedScratch(memoryManager, scratch);
}

//==============================================================================
RigidBodyContactStage::AvbdScratch* RigidBodyContactStage::createAvbdScratch(
    common::MemoryManager* memoryManager)
{
  if (memoryManager != nullptr) {
    return stage_detail::constructStageOwnedScratch<AvbdScratch>(
        memoryManager, memoryManager->getFreeAllocator());
  }
  return stage_detail::constructStageOwnedScratch<AvbdScratch>(nullptr);
}

//==============================================================================
RigidBodyContactStage::ContactScratch*
RigidBodyContactStage::createContactScratch(
    common::MemoryManager* memoryManager)
{
  if (memoryManager != nullptr) {
    return stage_detail::constructStageOwnedScratch<ContactScratch>(
        memoryManager, memoryManager->getFreeAllocator());
  }
  return stage_detail::constructStageOwnedScratch<ContactScratch>(nullptr);
}

//==============================================================================
std::string_view RigidBodyContactStage::getName() const noexcept
{
  return "rigid_body_contact";
}

//==============================================================================
ComputeStageMetadata RigidBodyContactStage::getMetadata() const noexcept
{
  return {
      ComputeStageDomain::Constraint,
      ComputeStageAcceleration::TaskParallel
          | ComputeStageAcceleration::DataLocality};
}

//==============================================================================
void RigidBodyContactStage::preflight(World& world)
{
  const auto& registry = dart::simulation::detail::registryOf(world);
  const RigidBodySolver rigidBodySolver = world.getRigidBodySolver();
  const bool useVbdFamily = rigidBodySolver == RigidBodySolver::Vbd;
  const bool useAvbdFamily = rigidBodySolver == RigidBodySolver::Avbd;
  const bool useOwnedBlockFamily = useVbdFamily || useAvbdFamily;
  const bool mayUseAvbdContactDetails
      = useOwnedBlockFamily || mayHaveRigidAvbdContactConfigs(registry);
  const bool mayHavePointJointConfigs
      = dvbd::mayHaveAvbdRigidWorldPointJointConfigs(registry);
  const std::size_t jointCapacity
      = mayHavePointJointConfigs
            ? registry
                  .view<
                      comps::JointModel,
                      dvbd::AvbdRigidWorldPointJointConfig>()
                  .size_hint()
            : 0u;
  const auto* distanceSpringStorage
      = registry.storage<dvbd::AvbdRigidWorldDistanceSpringConfig>();
  const std::size_t distanceSpringCapacity
      = distanceSpringStorage != nullptr ? distanceSpringStorage->size() : 0u;
  const bool skipContactQuery = shouldSkipRigidBodyContactQuery(world);
  // Resolve and enforce the construction-time query boundary even when every
  // current body has prescribed response. Once simulation mode is active, the
  // all-prescribed fast path can keep skipping unchanged frames because
  // prepare() has already baked and locked the collision cache.
  const bool queryForCapacityBake = !world.isSimulationMode();
  const std::span<const Contact> contacts
      = skipContactQuery && !queryForCapacityBake
            ? std::span<const Contact>{}
            : world.queryContacts(
                  CollisionQueryOptions{},
                  /*includeShapeContactDetails=*/mayUseAvbdContactDetails);

  const std::size_t avbdContactCapacity
      = !skipContactQuery && mayUseAvbdContactDetails
            ? world.getRigidCollisionContactCapacity()
            : 0u;
  if (avbdContactCapacity != 0u || jointCapacity != 0u
      || distanceSpringCapacity != 0u) {
    // queryContacts() resolves an automatic zero policy. Validate the resolved
    // bound here, still before kinematics or any step stage mutates World
    // state, rather than accidentally validating the unresolved zero.
    static_cast<void>(makeRigidAvbdScratchCapacityPlan(
        avbdContactCapacity, jointCapacity, distanceSpringCapacity));
  }

  if (skipContactQuery) {
    return;
  }

  const dvbd::AvbdRigidParameterProfile& selectedProfile
      = dvbd::avbdRigidParameterProfileFor(
          world.getRigidAvbdParameterProfile());
  const dvbd::AvbdRigidParameterProfile* ownedFamilyProfile
      = useAvbdFamily ? &selectedProfile : nullptr;
  if (mayUseAvbdContactDetails) {
    // Validate compatibility configuration while the World is still entirely
    // at its pre-step state. Public VBD/AVBD own a solver-wide profile and
    // intentionally ignore stale private per-body compatibility components.
    (void)rigidAvbdContactStageConfig(
        registry, contacts, useOwnedBlockFamily, ownedFamilyProfile);
  }

  if (useOwnedBlockFamily) {
    const auto expectedPublicContactCount
        = countProjectableRigidAvbdContacts(registry, contacts);
    DART_SIMULATION_THROW_T_IF(
        !expectedPublicContactCount.has_value(),
        NotImplementedException,
        "The {} rigid-body solver encountered an active contact envelope it "
        "cannot represent; no fallback solver was run",
        useVbdFamily ? "VBD" : "AVBD");
  }
}

//==============================================================================
void RigidBodyContactStage::prepare(World& world)
{
  if (m_contactScratch == nullptr) {
    m_contactScratch = ContactScratchPtr(
        createContactScratch(m_memoryManager),
        ContactScratchDeleter{m_memoryManager});
  }
  auto& constraints = m_contactScratch->problem.constraints;
  constraints.clear();

  const auto& registry = dart::simulation::detail::registryOf(world);
  const RigidBodySolver rigidBodySolver = world.getRigidBodySolver();
  const bool useOwnedBlockFamily = rigidBodySolver == RigidBodySolver::Vbd
                                   || rigidBodySolver == RigidBodySolver::Avbd;
  const bool skipContactQuery = shouldSkipRigidBodyContactQuery(world);
  std::size_t contactCapacity = 0u;
  std::size_t contactReserve = 0u;
  if (skipContactQuery) {
    // A skipped response stage still bakes the public collision-query policy.
    // This is required for all-static Worlds and loaded simulation snapshots:
    // their automatic capacities must resolve and lock just like active
    // contact scenes, and explicit limits must fail at the bake boundary.
    static_cast<void>(world.queryContacts(
        CollisionQueryOptions{}, /*includeShapeContactDetails=*/false));
  } else {
    // Warm the collision-query cache and its contact buffer at bake time so
    // baked steps reuse the separately configured, locked candidate/contact
    // bounds instead of inferring a capacity from the currently active set.
    const auto contacts = world.queryContacts(CollisionQueryOptions{});
    contactCapacity = world.getRigidCollisionContactCapacity();
    contactReserve = world.getRigidCollisionContactReserve();
    constraints.reserve(contactReserve);
    detail::storageOf(world).lastContactForces.reserve(contactReserve);
    // BoxedLcp additionally warms the frame arena for its per-step
    // Delassus/Dantzig dense temporaries; persistent solver state must stay out
    // of the resettable frame scratch.
    if (world.getContactSolverMethod() == ContactSolverMethod::BoxedLcp) {
      auto& frameAllocator = world.getMemoryManager().getFrameAllocator();
      {
        detail::BoxedLcpContactScratch frameWarmup(frameAllocator);
        detail::reserveBoxedLcpContactScratch(registry, contacts, frameWarmup);
      }
      frameAllocator.reset();
    }
  }

  const bool mayHavePointJointConfigs
      = dvbd::mayHaveAvbdRigidWorldPointJointConfigs(registry);
  const std::size_t rigidPairCapacity
      = registry
            .view<comps::JointModel, comps::JointState, comps::JointActuation>()
            .size_hint();
  const std::size_t jointCapacity
      = mayHavePointJointConfigs
            ? registry
                  .view<
                      comps::JointModel,
                      dvbd::AvbdRigidWorldPointJointConfig>()
                  .size_hint()
            : 0u;
  m_contactScratch->rigidPairInputs.clear();
  m_contactScratch->rigidPairInputs.reserve(rigidPairCapacity);
  m_contactScratch->sequentialImpulseJoints.clear();
  m_contactScratch->sequentialImpulseJoints.reserve(rigidPairCapacity);
  const auto* distanceSpringStorage
      = registry.storage<dvbd::AvbdRigidWorldDistanceSpringConfig>();
  const std::size_t distanceSpringCapacity
      = distanceSpringStorage != nullptr ? distanceSpringStorage->size() : 0u;
  // The AVBD scratch reserves the baked contact reserve, not the rejection
  // envelope; its row inventories grow (allocating) if an automatic scene
  // exceeds the reserve while staying under the envelope.
  const std::size_t avbdContactCapacity
      = contactCapacity != 0u
                && (useOwnedBlockFamily
                    || mayHaveRigidAvbdContactConfigs(registry))
            ? contactReserve
            : 0u;
  if (avbdContactCapacity == 0u && jointCapacity == 0u
      && distanceSpringCapacity == 0u) {
    if (m_avbdScratch != nullptr) {
      m_avbdScratch->clear();
    }
    return;
  }

  if (m_avbdScratch == nullptr) {
    m_avbdScratch = AvbdScratchPtr(
        createAvbdScratch(m_memoryManager),
        AvbdScratchDeleter{m_memoryManager});
  }
  const auto capacityPlan = makeRigidAvbdScratchCapacityPlan(
      avbdContactCapacity, jointCapacity, distanceSpringCapacity);
  m_avbdScratch->reserve(
      capacityPlan, avbdContactCapacity, jointCapacity, distanceSpringCapacity);
}

//==============================================================================
void RigidBodyContactStage::execute(World& world, ComputeExecutor& executor)
{
  auto& registry = dart::simulation::detail::registryOf(world);
  const bool useSequentialImpulseFamily
      = world.getRigidBodySolver() == RigidBodySolver::SequentialImpulse;
  const bool useVbdFamily = world.getRigidBodySolver() == RigidBodySolver::Vbd;
  const bool useAvbdFamily
      = world.getRigidBodySolver() == RigidBodySolver::Avbd;
  const bool useOwnedBlockFamily = useVbdFamily || useAvbdFamily;
  const auto formulation
      = useVbdFamily
            ? dvbd::AvbdRigidWorldContactSolveOptions::Formulation::FixedPenalty
            : dvbd::AvbdRigidWorldContactSolveOptions::Formulation::
                  AugmentedLagrangian;
  const dvbd::AvbdRigidParameterProfile& selectedProfile
      = dvbd::avbdRigidParameterProfileFor(
          world.getRigidAvbdParameterProfile());
  const dvbd::AvbdRigidParameterProfile* ownedFamilyProfile
      = useAvbdFamily ? &selectedProfile : nullptr;
  const std::string_view solverFamilyName = useVbdFamily ? "VBD" : "AVBD";
  const auto clearFixedPenaltyScratch = [&]() noexcept {
    if (m_avbdScratch != nullptr) {
      m_avbdScratch->clear();
    }
  };
  struct FixedPenaltyScratchGuard
  {
    bool active;
    decltype(clearFixedPenaltyScratch)& clear;

    ~FixedPenaltyScratchGuard()
    {
      if (active) {
        clear();
      }
    }
  } fixedPenaltyScratchGuard{useVbdFamily, clearFixedPenaltyScratch};
  if (useVbdFamily) {
    clearFixedPenaltyScratch();
  }

  if (m_contactScratch == nullptr) {
    m_contactScratch = ContactScratchPtr(
        createContactScratch(m_memoryManager),
        ContactScratchDeleter{m_memoryManager});
  }
  auto& sequentialImpulseJoints = m_contactScratch->sequentialImpulseJoints;
  auto& rigidPairInputs = m_contactScratch->rigidPairInputs;
  rigidPairInputs.clear();
  sequentialImpulseJoints.clear();
  if (useSequentialImpulseFamily && m_avbdScratch != nullptr) {
    // Sequential Impulse owns public hard pair rows independently of AVBD.
    // Crossing through this family invalidates AVBD's accumulated joint and
    // motor multipliers, while AVBD compatibility distance-spring state
    // remains live.
    m_avbdScratch->clearPointJointWarmStart();
  }
  std::size_t sequentialImpulseJointRows = 0u;
  if (useSequentialImpulseFamily) {
    const std::size_t expectedJointCount
        = expectedRigidSequentialImpulseJointCount(registry);
    if (expectedJointCount == 0u) {
      // Broken, degenerate, or static-static rigid pair constraints own no
      // rows; the shared eligibility rule decides both the count and the
      // extraction.
    } else {
      detail::extractRigidPairConstraintInputsInto(
          registry,
          rigidPairInputs,
          /*includeWorldAnchors=*/false);
      DART_SIMULATION_THROW_T_IF(
          rigidPairInputs.size() != expectedJointCount,
          InvalidOperationException,
          "Sequential-impulse rigid joint extraction covered {} of {} active "
          "public pair constraints",
          rigidPairInputs.size(),
          expectedJointCount);
      sequentialImpulseJointRows = assembleRigidSequentialImpulseJointsInto(
          sequentialImpulseJoints, registry, rigidPairInputs);
    }
  }

  const auto sweepSequentialImpulseJoints = [&]() {
    sweepRigidSequentialImpulseJoints(
        registry, sequentialImpulseJoints, world.getTimeStep());
  };
  const auto runSequentialImpulseJointSweeps = [&]() {
    if (sequentialImpulseJointRows == 0u) {
      return;
    }
    recordSolverDiagnostics(world, m_iterations);
    for (std::size_t iteration = 0; iteration < m_iterations; ++iteration) {
      sweepSequentialImpulseJoints();
    }
  };
  const auto finalizeSequentialImpulseJoints = [&]() {
    if (sequentialImpulseJointRows == 0u) {
      return;
    }
    (void)markBrokenRigidSequentialImpulseJoints(
        registry, sequentialImpulseJoints, world.getTimeStep());
    postStabilizeRigidSequentialImpulseJoints(
        registry, sequentialImpulseJoints, m_iterations);
  };

  const auto projectRigidBlockPointJoints = [&]() {
    const bool hasPointJointConfigs
        = !useSequentialImpulseFamily
          && dvbd::mayHaveAvbdRigidWorldPointJointConfigs(registry);
    const bool hasDistanceSpringConfigs
        = dvbd::mayHaveAvbdRigidWorldDistanceSpringConfigs(registry);
    if (!hasPointJointConfigs && !hasDistanceSpringConfigs) {
      return false;
    }

    if (m_avbdScratch == nullptr) {
      m_avbdScratch = AvbdScratchPtr(
          createAvbdScratch(m_memoryManager),
          AvbdScratchDeleter{m_memoryManager});
    }
    auto& scratch = *m_avbdScratch;
    if (hasPointJointConfigs) {
      dvbd::extractAvbdRigidWorldPointJointInputsInto(
          registry, scratch.pointJoints, /*includeWorldAnchors=*/false);
    } else {
      scratch.pointJoints.clear();
    }
    if (hasDistanceSpringConfigs) {
      dvbd::extractAvbdRigidWorldDistanceSpringInputsInto(
          registry, scratch.distanceSprings, /*includeWorldAnchors=*/false);
    } else {
      scratch.distanceSprings.clear();
    }
    if (scratch.pointJoints.empty() && scratch.distanceSprings.empty()) {
      return false;
    }

    dvbd::clearAvbdRigidWorldContactSnapshot(scratch.snapshot);
    const std::size_t appendedJoints
        = scratch.pointJoints.empty()
              ? 0u
              : dvbd::appendAvbdRigidWorldPointJoints(
                    registry,
                    scratch.pointJoints,
                    scratch.snapshot,
                    scratch.buildScratch,
                    selectedProfile.angularRowsUseTorqueArm);
    const std::size_t appendedDistanceSprings
        = scratch.distanceSprings.empty()
              ? 0u
              : dvbd::appendAvbdRigidWorldDistanceSprings(
                    registry,
                    scratch.distanceSprings,
                    scratch.snapshot,
                    scratch.buildScratch);
    if (appendedJoints == 0u && appendedDistanceSprings == 0u) {
      return false;
    }

    const double timeStep = world.getTimeStep();
    dvbd::predictAvbdRigidWorldContactInertialTargets(
        registry, scratch.snapshot, timeStep);
    if (avbdAdaptiveInitialGuessEnabled(selectedProfile)) {
      dvbd::predictAvbdRigidWorldContactInitialGuess(
          scratch.snapshot,
          timeStep,
          world.getGravity(),
          std::span<const dvbd::AvbdRigidProjectedVelocityRecord>{
              scratch.projectedVelocities.data(),
              scratch.projectedVelocities.size()});
    } else {
      scratch.snapshot.initialGuessStates.clear();
    }

    const dvbd::AvbdRigidWorldContactSolveOptions solveOptions
        = rigidAvbdWorldSolveOptions(
            m_iterations,
            formulation,
            ownedFamilyProfile,
            nullptr,
            std::numeric_limits<double>::infinity(),
            &selectedProfile);
    const dvbd::AvbdRigidWorldContactSolveResult solveResult
        = dvbd::solveAvbdRigidWorldContactSnapshot(
            scratch.snapshot,
            scratch.normalInventory,
            scratch.frictionInventory,
            scratch.jointLinearInventory,
            scratch.jointAngularInventory,
            scratch.motorInventory,
            scratch.distanceSpringInventory,
            timeStep,
            scratch.solveScratch,
            solveOptions,
            &executor);
    (void)dvbd::markAvbdRigidWorldFracturedPointJoints(
        registry, scratch.snapshot, solveResult.fracturedJointIndices);
    if (solveResult.jointLinearRows == 0u && solveResult.jointAngularRows == 0u
        && solveResult.motorRows == 0u
        && solveResult.distanceSpringRows == 0u) {
      return false;
    }

    recordSolverDiagnostics(world, solveResult.stats.iterations);
    const dvbd::AvbdRigidWorldContactApplyResult projection
        = dvbd::applyAvbdRigidWorldContactVelocityProjection(
            registry, scratch.snapshot, timeStep);
    if (avbdAdaptiveInitialGuessEnabled(selectedProfile)) {
      dvbd::recordAvbdRigidWorldContactProjectedVelocities(
          registry,
          scratch.snapshot,
          scratch.projectedVelocities,
          scratch.projectedVelocityScratch);
    } else {
      scratch.projectedVelocities.clear();
    }
    (void)dvbd::applyAvbdRigidWorldContactPostStabilization(
        registry, scratch.snapshot);
    return projection.bodies != 0u;
  };

  if (shouldSkipRigidBodyContactQuery(world)) {
    const bool projectedRigidBlockRows = projectRigidBlockPointJoints();
    if (!useSequentialImpulseFamily && projectedRigidBlockRows) {
      return;
    }
    runSequentialImpulseJointSweeps();
    finalizeSequentialImpulseJoints();

    if (m_avbdScratch != nullptr) {
      if (useSequentialImpulseFamily) {
        m_avbdScratch->clearSequentialImpulseOwnedWarmStart();
      } else {
        m_avbdScratch->clear();
      }
    }
    return;
  }

  const bool mayUseAvbdContactDetails
      = useOwnedBlockFamily || mayHaveRigidAvbdContactConfigs(registry);
  const auto queriedContacts = world.queryContacts(
      CollisionQueryOptions{},
      /*includeShapeContactDetails=*/mayUseAvbdContactDetails);
  std::vector<Contact> deactivationContacts;
  std::span<const Contact> contacts = queriedContacts;
  if (world.isDeactivationActiveForStep()) {
    deactivationContacts = world.filterContactsForDeactivation(contacts);
    contacts = deactivationContacts;
  }
  recordRigidContactEnvelopeMetrics(world, registry, contacts);
  if (contacts.empty()) {
    const bool projectedRigidBlockRows = projectRigidBlockPointJoints();
    if (!useSequentialImpulseFamily && projectedRigidBlockRows) {
      return;
    }
    runSequentialImpulseJointSweeps();
    finalizeSequentialImpulseJoints();

    if (m_avbdScratch != nullptr) {
      if (useSequentialImpulseFamily) {
        m_avbdScratch->clearSequentialImpulseOwnedWarmStart();
      } else {
        m_avbdScratch->clear();
      }
    }
    return;
  }

  // Rigid block-descent contact path (PLAN-104): the public VBD and AVBD
  // families select every supported active free-rigid contact. VBD holds each
  // conservative row at a finite penalty stiffness; AVBD augments and ramps the
  // same row families. The compatibility path under another family remains
  // AVBD-only and requires every active contact to carry a private opt-in.
  const auto avbdConfig
      = mayUseAvbdContactDetails
            ? rigidAvbdContactStageConfig(
                  registry, contacts, useOwnedBlockFamily, ownedFamilyProfile)
            : std::optional<comps::RigidAvbdContactConfig>{};
  if (avbdConfig) {
    if (m_avbdScratch == nullptr) {
      m_avbdScratch = AvbdScratchPtr(
          createAvbdScratch(m_memoryManager),
          AvbdScratchDeleter{m_memoryManager});
    }
    auto& scratch = *m_avbdScratch;

    dvbd::AvbdRigidWorldContactOptions contactOptions;
    contactOptions.startStiffness = std::max(0.0, avbdConfig->startStiffness);
    contactOptions.maxStiffness
        = std::max(contactOptions.startStiffness, avbdConfig->maxStiffness);
    dvbd::buildAvbdRigidWorldContactSnapshot(
        registry,
        contacts,
        scratch.snapshot,
        scratch.buildScratch,
        contactOptions);
    const auto expectedPublicContactCount
        = useOwnedBlockFamily
              ? countProjectableRigidAvbdContacts(registry, contacts)
              : std::optional<std::size_t>{};
    DART_SIMULATION_THROW_T_IF(
        useOwnedBlockFamily && !expectedPublicContactCount.has_value(),
        NotImplementedException,
        "The {} rigid-body solver encountered an active contact envelope it "
        "cannot project; sequential-impulse fallback is disabled",
        solverFamilyName);
    const bool allContactsCovered
        = useOwnedBlockFamily
              ? scratch.snapshot.contacts.size()
                    == expectedPublicContactCount.value()
              : scratch.snapshot.contacts.size() == contacts.size();
    DART_SIMULATION_THROW_T_IF(
        useOwnedBlockFamily && !allContactsCovered,
        NotImplementedException,
        "The {} rigid-body solver could not assemble every active contact; "
        "sequential-impulse fallback is disabled",
        solverFamilyName);

    std::size_t appendedJoints = 0u;
    std::size_t appendedDistanceSprings = 0u;
    if (!useSequentialImpulseFamily
        && dvbd::mayHaveAvbdRigidWorldPointJointConfigs(registry)) {
      dvbd::extractAvbdRigidWorldPointJointInputsInto(
          registry, scratch.pointJoints, /*includeWorldAnchors=*/false);
      if (!scratch.pointJoints.empty()) {
        appendedJoints = dvbd::appendAvbdRigidWorldPointJoints(
            registry,
            scratch.pointJoints,
            scratch.snapshot,
            scratch.buildScratch,
            selectedProfile.angularRowsUseTorqueArm);
      }
    } else {
      scratch.pointJoints.clear();
    }
    if (dvbd::mayHaveAvbdRigidWorldDistanceSpringConfigs(registry)) {
      dvbd::extractAvbdRigidWorldDistanceSpringInputsInto(
          registry, scratch.distanceSprings, /*includeWorldAnchors=*/false);
      if (!scratch.distanceSprings.empty()) {
        appendedDistanceSprings = dvbd::appendAvbdRigidWorldDistanceSprings(
            registry,
            scratch.distanceSprings,
            scratch.snapshot,
            scratch.buildScratch);
      }
    } else {
      scratch.distanceSprings.clear();
    }

    if (allContactsCovered
        && (!scratch.snapshot.contacts.empty() || appendedJoints != 0u
            || appendedDistanceSprings != 0u)) {
      const double timeStep = world.getTimeStep();
      dvbd::predictAvbdRigidWorldContactInertialTargets(
          registry, scratch.snapshot, timeStep);
      if (avbdAdaptiveInitialGuessEnabled(selectedProfile)) {
        dvbd::predictAvbdRigidWorldContactInitialGuess(
            scratch.snapshot,
            timeStep,
            world.getGravity(),
            std::span<const dvbd::AvbdRigidProjectedVelocityRecord>{
                scratch.projectedVelocities.data(),
                scratch.projectedVelocities.size()});
      } else {
        scratch.snapshot.initialGuessStates.clear();
      }

      const dvbd::AvbdRigidWorldContactSolveOptions solveOptions
          = rigidAvbdWorldSolveOptions(
              m_iterations,
              formulation,
              ownedFamilyProfile,
              &*avbdConfig,
              contactOptions.maxStiffness,
              &selectedProfile);

      const dvbd::AvbdRigidWorldContactSolveResult solveResult
          = dvbd::solveAvbdRigidWorldContactSnapshot(
              scratch.snapshot,
              scratch.normalInventory,
              scratch.frictionInventory,
              scratch.jointLinearInventory,
              scratch.jointAngularInventory,
              scratch.motorInventory,
              scratch.distanceSpringInventory,
              timeStep,
              scratch.solveScratch,
              solveOptions,
              &executor);
      (void)dvbd::markAvbdRigidWorldFracturedPointJoints(
          registry, scratch.snapshot, solveResult.fracturedJointIndices);
      if (solveResult.normalRows != 0u || solveResult.frictionRows != 0u
          || solveResult.jointLinearRows != 0u
          || solveResult.jointAngularRows != 0u || solveResult.motorRows != 0u
          || solveResult.distanceSpringRows != 0u) {
        recordSolverDiagnostics(world, solveResult.stats.iterations);
        const dvbd::AvbdRigidWorldContactApplyResult projection
            = dvbd::applyAvbdRigidWorldContactVelocityProjection(
                registry, scratch.snapshot, timeStep);
        if (avbdAdaptiveInitialGuessEnabled(selectedProfile)) {
          dvbd::recordAvbdRigidWorldContactProjectedVelocities(
              registry,
              scratch.snapshot,
              scratch.projectedVelocities,
              scratch.projectedVelocityScratch);
        } else {
          scratch.projectedVelocities.clear();
        }
        (void)dvbd::applyAvbdRigidWorldContactPostStabilization(
            registry, scratch.snapshot);
        if (projection.bodies != 0u) {
          runSequentialImpulseJointSweeps();
          finalizeSequentialImpulseJoints();
          return;
        }
      }

      DART_SIMULATION_THROW_T_IF(
          useOwnedBlockFamily,
          InvalidOperationException,
          "The {} rigid-body solver assembled active constraints but produced "
          "no block-descent rows; sequential-impulse fallback is disabled",
          solverFamilyName);
    } else if (
        useOwnedBlockFamily && allContactsCovered
        && expectedPublicContactCount.value() == 0u) {
      m_avbdScratch->clear();
      return;
    }

    if (useSequentialImpulseFamily) {
      // The private AVBD contact compatibility path declined this envelope,
      // so SI owns the fallback contact and hard-joint rows. Preserve the
      // independently active distance-spring continuation inventory.
      m_avbdScratch->clearSequentialImpulseOwnedWarmStart();
    } else {
      m_avbdScratch->clear();
    }
  }

  DART_SIMULATION_THROW_T_IF(
      useOwnedBlockFamily,
      InvalidOperationException,
      "The {} rigid-body solver did not resolve the active rigid contacts; "
      "sequential-impulse fallback is disabled",
      solverFamilyName);

  if (useSequentialImpulseFamily && m_avbdScratch != nullptr) {
    // This contact envelope reached the SI-owned path rather than the explicit
    // private AVBD compatibility path above. Its normal and friction
    // multipliers must not survive a later runtime switch back to AVBD.
    m_avbdScratch->clearContactWarmStart();
  }

  if (projectRigidBlockPointJoints()) {
    // Keep explicitly configured AVBD distance springs active while ordinary
    // contacts and public hard pair constraints continue through their
    // selected non-AVBD solvers below.
  } else if (m_avbdScratch != nullptr) {
    if (useSequentialImpulseFamily) {
      m_avbdScratch->clearPointJointWarmStart();
    } else {
      m_avbdScratch->clear();
    }
  }

  // Opt-in boxed-LCP path (PLAN-080 WS4): assemble and solve the Coulomb
  // normal+tangent Delassus system with the pivoting Dantzig solver, applying
  // the resulting impulses to body velocities. The default SequentialImpulse
  // path below is unchanged.
  if (world.getContactSolverMethod() == ContactSolverMethod::BoxedLcp) {
    detail::BoxedLcpContactScratch frameScratch(
        world.getMemoryManager().getFrameAllocator());
    // Keep the step on the allocation-free apply path. The solved impulse
    // buffer remains available in frame scratch for force capture; requesting
    // the full differentiable snapshot here would allocate its Eigen matrices
    // on every step.
    detail::applyBoxedLcpContacts(
        registry, contacts, world.getTimeStep(), frameScratch);
    writeBoxedLcpImpulsesIntoConstraints(
        frameScratch.problem.constraints, frameScratch.systemF);
    captureRigidContactForces(
        registry, world, frameScratch.problem.constraints, world.getTimeStep());
    runSequentialImpulseJointSweeps();
    (void)markBrokenRigidSequentialImpulseJoints(
        registry, sequentialImpulseJoints, world.getTimeStep());
    resolveRigidBodyContactPositions(registry, contacts, world.getTimeStep());
    // The contact-unaware joint pass runs last and may re-introduce shallow
    // contact penetration; the SI wall oracles budget that residual.
    postStabilizeRigidSequentialImpulseJoints(
        registry, sequentialImpulseJoints, m_iterations);
    return;
  }

  if (m_contactScratch == nullptr) {
    m_contactScratch = ContactScratchPtr(
        createContactScratch(m_memoryManager),
        ContactScratchDeleter{m_memoryManager});
  }
  RigidBodyContactAssemblyOptions assemblyOptions;
  assemblyOptions.populateSystem = false;
  assembleRigidBodyContactProblemInto(
      m_contactScratch->problem, registry, contacts, assemblyOptions);
  auto& constraints = m_contactScratch->problem.constraints;
  const bool hasFrictionConstraints = std::any_of(
      constraints.begin(), constraints.end(), [](const auto& constraint) {
        return constraint.friction > 0.0;
      });
  if (!constraints.empty() || sequentialImpulseJointRows != 0u) {
    recordSolverDiagnostics(world, m_iterations);
  }

  // Sequential impulses (Gauss-Seidel) drive each contact's normal approach
  // velocity to its restitution target. The accumulated normal impulse is
  // clamped non-negative so contacts only push, never pull.
  const auto solveNormalImpulse = [&](RigidBodyContactConstraint& constraint) {
    const auto& velocityA = registry.get<comps::Velocity>(constraint.bodyA);
    const auto& velocityB = registry.get<comps::Velocity>(constraint.bodyB);
    const double approach
        = (computeRigidBodyContactPointVelocity(
               velocityB, constraint.armB, constraint.staticB)
           - computeRigidBodyContactPointVelocity(
               velocityA, constraint.armA, constraint.staticA))
              .dot(constraint.normal);

    double lambda = -(approach - constraint.restitutionVelocity)
                    / constraint.effectiveMass;
    const double clamped = std::max(constraint.normalImpulse + lambda, 0.0);
    lambda = clamped - constraint.normalImpulse;
    constraint.normalImpulse = clamped;

    if (lambda == 0.0) {
      return;
    }
    applyRigidBodyContactImpulse(
        registry, constraint, lambda * constraint.normal);
  };

  for (std::size_t iteration = 0; iteration < m_iterations; ++iteration) {
    sweepSequentialImpulseJoints();
    for (auto& constraint : constraints) {
      solveNormalImpulse(constraint);

      // Coulomb friction along each tangent, clamped to the friction pyramid
      // bounded by the accumulated normal impulse.
      if (hasFrictionConstraints && constraint.friction > 0.0) {
        const double frictionLimit
            = constraint.friction * constraint.normalImpulse;
        const auto solveFriction = [&](const Eigen::Vector3d& tangent,
                                       double tangentMass,
                                       double& tangentImpulse) {
          if (tangentMass <= 0.0 || frictionLimit <= 0.0) {
            return;
          }
          const auto& velocityA
              = registry.get<comps::Velocity>(constraint.bodyA);
          const auto& velocityB
              = registry.get<comps::Velocity>(constraint.bodyB);
          const Eigen::Vector3d tangentVelocity
              = computeRigidBodyContactPointVelocity(
                    velocityB, constraint.armB, constraint.staticB)
                - computeRigidBodyContactPointVelocity(
                    velocityA, constraint.armA, constraint.staticA);
          double tangentLambda = -tangentVelocity.dot(tangent) / tangentMass;
          const double clampedTangent = std::clamp(
              tangentImpulse + tangentLambda, -frictionLimit, frictionLimit);
          tangentLambda = clampedTangent - tangentImpulse;
          tangentImpulse = clampedTangent;
          if (tangentLambda == 0.0) {
            return;
          }

          const Eigen::Vector3d tangentImpulseVector = tangentLambda * tangent;
          applyRigidBodyContactImpulse(
              registry, constraint, tangentImpulseVector);
        };
        solveFriction(
            constraint.tangent1,
            constraint.tangentMass1,
            constraint.tangentImpulse1);
        solveFriction(
            constraint.tangent2,
            constraint.tangentMass2,
            constraint.tangentImpulse2);
      }
    }
  }

  captureRigidContactForces(registry, world, constraints, world.getTimeStep());
  (void)markBrokenRigidSequentialImpulseJoints(
      registry, sequentialImpulseJoints, world.getTimeStep());
  resolveRigidBodyContactPositions(registry, contacts, world.getTimeStep());
  // The contact-unaware joint pass runs last and may re-introduce shallow
  // contact penetration; the SI wall oracles budget that residual.
  postStabilizeRigidSequentialImpulseJoints(
      registry, sequentialImpulseJoints, m_iterations);
}

//==============================================================================
std::size_t RigidBodyContactStage::getIterations() const noexcept
{
  return m_iterations;
}

//==============================================================================
avbd_replay::RigidAvbdWarmStartReplayState
RigidBodyContactStage::captureAvbdWarmStartReplayState(
    common::MemoryAllocator& allocator) const
{
  avbd_replay::RigidAvbdWarmStartReplayState state(allocator);
  if (m_avbdScratch == nullptr) {
    return state;
  }

  const auto copyRecords = [](auto& destination, const auto& inventory) {
    destination.assign(inventory.records().begin(), inventory.records().end());
  };
  copyRecords(state.normalRows, m_avbdScratch->normalInventory);
  copyRecords(state.frictionRows, m_avbdScratch->frictionInventory);
  state.contactIdentities.assign(
      m_avbdScratch->solveScratch.contactRows.contactIdentities.begin(),
      m_avbdScratch->solveScratch.contactRows.contactIdentities.end());
  state.contactTangentAnchors.assign(
      m_avbdScratch->solveScratch.contactRows.contactTangentAnchors.begin(),
      m_avbdScratch->solveScratch.contactRows.contactTangentAnchors.end());
  copyRecords(state.jointLinearRows, m_avbdScratch->jointLinearInventory);
  copyRecords(state.jointAngularRows, m_avbdScratch->jointAngularInventory);
  copyRecords(state.motorRows, m_avbdScratch->motorInventory);
  copyRecords(state.distanceSpringRows, m_avbdScratch->distanceSpringInventory);
  state.projectedVelocities.assign(
      m_avbdScratch->projectedVelocities.begin(),
      m_avbdScratch->projectedVelocities.end());
  return state;
}

//==============================================================================
void RigidBodyContactStage::clearAvbdWarmStartContinuationState() noexcept
{
  if (m_avbdScratch != nullptr) {
    m_avbdScratch->clear();
  }
}

//==============================================================================
void RigidBodyContactStage::restoreAvbdWarmStartReplayState(
    const avbd_replay::RigidAvbdWarmStartReplayState& replayState)
{
  if (m_avbdScratch == nullptr) {
    m_avbdScratch = AvbdScratchPtr(
        createAvbdScratch(m_memoryManager),
        AvbdScratchDeleter{m_memoryManager});
  }

  m_avbdScratch->clear();
  const auto restoreRecords = [](auto& inventory, const auto& records) {
    inventory.records().assign(records.begin(), records.end());
  };
  restoreRecords(m_avbdScratch->normalInventory, replayState.normalRows);
  restoreRecords(m_avbdScratch->frictionInventory, replayState.frictionRows);
  m_avbdScratch->solveScratch.contactRows.contactIdentities.assign(
      replayState.contactIdentities.begin(),
      replayState.contactIdentities.end());
  m_avbdScratch->solveScratch.contactRows.contactTangentAnchors.assign(
      replayState.contactTangentAnchors.begin(),
      replayState.contactTangentAnchors.end());
  restoreRecords(
      m_avbdScratch->jointLinearInventory, replayState.jointLinearRows);
  restoreRecords(
      m_avbdScratch->jointAngularInventory, replayState.jointAngularRows);
  restoreRecords(m_avbdScratch->motorInventory, replayState.motorRows);
  restoreRecords(
      m_avbdScratch->distanceSpringInventory, replayState.distanceSpringRows);
  m_avbdScratch->projectedVelocities.assign(
      replayState.projectedVelocities.begin(),
      replayState.projectedVelocities.end());
}

//==============================================================================
bool RigidBodyContactStage::hasAnyAvbdWarmStartContinuationState()
    const noexcept
{
  if (m_avbdScratch == nullptr) {
    return false;
  }

  return !m_avbdScratch->normalInventory.records().empty()
         || !m_avbdScratch->frictionInventory.records().empty()
         || !m_avbdScratch->solveScratch.contactRows.contactIdentities.empty()
         || !m_avbdScratch->solveScratch.contactRows.contactTangentAnchors
                 .empty()
         || !m_avbdScratch->jointLinearInventory.records().empty()
         || !m_avbdScratch->jointAngularInventory.records().empty()
         || !m_avbdScratch->motorInventory.records().empty()
         || !m_avbdScratch->distanceSpringInventory.records().empty()
         || !m_avbdScratch->projectedVelocities.empty();
}

//==============================================================================
void RigidBodyContactStage::
    clearSequentialImpulseOwnedAvbdWarmStartContinuationState() noexcept
{
  if (m_avbdScratch != nullptr) {
    m_avbdScratch->clearSequentialImpulseOwnedWarmStart();
  }
}

//==============================================================================
void RigidBodyContactStage::setIterations(std::size_t iterations) noexcept
{
  m_iterations = std::max<std::size_t>(1, iterations);
}

} // namespace dart::simulation::compute
