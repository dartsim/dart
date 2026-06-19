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
#include "dart/simulation/compute/detail/stage_scratch.hpp"
#include "dart/simulation/compute/detail/world_step_stages.hpp"
#include "dart/simulation/detail/entity_conversion.hpp"
#include "dart/simulation/detail/rigid_avbd/rigid_world_contact.hpp"
#include "dart/simulation/detail/rigid_contact/boxed_lcp_contact.hpp"
#include "dart/simulation/detail/rigid_contact/rigid_contact_assembly.hpp"
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
#include <memory>
#include <optional>
#include <span>
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

//==============================================================================
std::optional<comps::RigidAvbdContactConfig> rigidAvbdContactStageConfig(
    const detail::WorldRegistry& registry, std::span<const Contact> contacts)
{
  std::optional<comps::RigidAvbdContactConfig> merged;
  for (const Contact& contact : contacts) {
    const auto* configA = enabledRigidAvbdContactConfig(
        registry, detail::toRegistryEntity(contact.bodyA.getEntity()));
    const auto* configB = enabledRigidAvbdContactConfig(
        registry, detail::toRegistryEntity(contact.bodyB.getEntity()));
    if (configA == nullptr && configB == nullptr) {
      return std::nullopt;
    }

    const auto absorb = [&](const comps::RigidAvbdContactConfig& config) {
      if (!merged.has_value()) {
        merged = config;
        return;
      }

      merged->startStiffness
          = std::max(merged->startStiffness, config.startStiffness);
      merged->alpha = std::max(merged->alpha, config.alpha);
      merged->beta = std::max(merged->beta, config.beta);
      merged->gamma = std::max(merged->gamma, config.gamma);
      merged->maxStiffness
          = std::min(merged->maxStiffness, config.maxStiffness);
    };

    if (configA != nullptr) {
      absorb(*configA);
    }
    if (configB != nullptr) {
      absorb(*configB);
    }
  }

  return merged;
}
bool hasContactNormalAngularTerm(const Eigen::Vector3d& normalArmCross)
{
  return normalArmCross.x() != 0.0 || normalArmCross.y() != 0.0
         || normalArmCross.z() != 0.0;
}

//==============================================================================
bool needsContactInverseInertia(
    const Eigen::Vector3d& normalArmCross, const double friction)
{
  return friction > 0.0 || hasContactNormalAngularTerm(normalArmCross);
}

//==============================================================================
struct RigidContactCandidate
{
  entt::entity entity = entt::null;
  bool prescribedResponse = false;
};

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

  std::vector<RigidContactCandidate> candidates;

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
      candidates.push_back(RigidContactCandidate{entity, prescribedResponse});
    }
  }

  if (!hasNonPrescribedCandidate) {
    return true;
  }

  if (candidates.size() > kIgnoredPairSkipAuditLimit) {
    return false;
  }

  for (std::size_t i = 0; i < candidates.size(); ++i) {
    for (std::size_t j = i + 1; j < candidates.size(); ++j) {
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

} // namespace

struct RigidBodyContactStage::AvbdScratch
{
  using PointJointAllocator
      = common::StlAllocator<dvbd::AvbdRigidWorldPointJointInput>;
  using DistanceSpringAllocator
      = common::StlAllocator<dvbd::AvbdRigidWorldDistanceSpringInput>;

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
      distanceSpringInventory(allocator)
  {
  }

  void clear()
  {
    dvbd::clearAvbdRigidWorldContactSnapshot(snapshot);
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
  }

  void reserve(
      std::size_t bodyCapacity,
      std::size_t contactCapacity,
      std::size_t jointCapacity,
      std::size_t distanceSpringCapacity)
  {
    dvbd::reserveAvbdRigidWorldContactSnapshot(
        snapshot,
        bodyCapacity,
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
        bodyCapacity,
        distanceSpringCapacity);
    normalInventory.reserve(contactCapacity);
    frictionInventory.reserve(2u * contactCapacity);
    jointLinearInventory.reserve(3u * jointCapacity);
    jointAngularInventory.reserve(3u * jointCapacity);
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
};

//==============================================================================
struct RigidBodyContactStage::ContactScratch
{
  struct NormalConstraint
  {
    entt::entity bodyA;
    entt::entity bodyB;
    comps::Velocity* velocityA;
    comps::Velocity* velocityB;
    Eigen::Vector3d normal;
    Eigen::Vector3d armA;
    Eigen::Vector3d armB;
    bool staticA;
    bool staticB;
    double invMassA;
    double invMassB;
    Eigen::Matrix3d invInertiaA;
    Eigen::Matrix3d invInertiaB;
    bool hasNormalAngularA;
    bool hasNormalAngularB;
    double effectiveMass;
    double inverseEffectiveMass;
    double depth;
    double restitutionVelocity;
    double normalImpulse;
    Eigen::Vector3d normalArmCrossA;
    Eigen::Vector3d normalArmCrossB;
    Eigen::Vector3d normalLinearDeltaA;
    Eigen::Vector3d normalLinearDeltaB;
    Eigen::Vector3d normalAngularDeltaA;
    Eigen::Vector3d normalAngularDeltaB;
    Eigen::Vector3d tangent1;
    Eigen::Vector3d tangent2;
    double tangentMass1;
    double tangentMass2;
    double tangentImpulse1;
    double tangentImpulse2;
    double friction;
  };

  using ConstraintAllocator = common::StlAllocator<NormalConstraint>;

  ContactScratch() = default;

  explicit ContactScratch(common::MemoryAllocator& allocator)
    : constraints(ConstraintAllocator{allocator})
  {
  }

  std::vector<NormalConstraint, ConstraintAllocator> constraints;
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
void RigidBodyContactStage::prepare(World& world)
{
  if (m_contactScratch == nullptr) {
    m_contactScratch = ContactScratchPtr(
        createContactScratch(m_memoryManager),
        ContactScratchDeleter{m_memoryManager});
  }
  auto& constraints = m_contactScratch->constraints;
  constraints.clear();

  const auto& registry = dart::simulation::detail::registryOf(world);
  const bool skipContactQuery = shouldSkipRigidBodyContactQuery(world);
  std::size_t contactCapacity = 0u;
  if (!skipContactQuery) {
    std::size_t collisionShapeCount = 0;
    const auto geometryView = registry.view<comps::CollisionGeometry>();
    for (const entt::entity entity : geometryView) {
      collisionShapeCount
          += geometryView.get<comps::CollisionGeometry>(entity).shapes.size();
    }
    constexpr std::size_t kContactConstraintCapacityPerShape = 4u;
    const std::size_t contactConstraintCapacity
        = collisionShapeCount <= std::numeric_limits<std::size_t>::max()
                                     / kContactConstraintCapacityPerShape
              ? collisionShapeCount * kContactConstraintCapacityPerShape
              : std::numeric_limits<std::size_t>::max();
    contactCapacity = contactConstraintCapacity;
    constraints.reserve(contactCapacity);
    // Warm the collision-query cache and its contact buffer at bake time so
    // baked steps reuse the reserved capacity instead of growing the world
    // allocator. BoxedLcp additionally warms the frame arena for its per-step
    // Delassus/Dantzig dense temporaries; persistent solver state must stay out
    // of the resettable frame scratch.
    [[maybe_unused]] const auto contacts
        = world.queryContacts(CollisionQueryOptions{});
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
  const std::size_t avbdContactCapacity
      = contactCapacity != 0u && mayHaveRigidAvbdContactConfigs(registry)
            ? contactCapacity
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
  const std::size_t maxSize = std::numeric_limits<std::size_t>::max();
  const std::size_t rowEndpointCapacity
      = avbdContactCapacity <= maxSize - jointCapacity
                && avbdContactCapacity + jointCapacity
                       <= maxSize - distanceSpringCapacity
            ? avbdContactCapacity + jointCapacity + distanceSpringCapacity
            : maxSize;
  const std::size_t bodyCapacity = rowEndpointCapacity <= maxSize / 2u
                                       ? 2u * rowEndpointCapacity
                                       : maxSize;
  m_avbdScratch->reserve(
      bodyCapacity, avbdContactCapacity, jointCapacity, distanceSpringCapacity);
}

//==============================================================================
void RigidBodyContactStage::execute(World& world, ComputeExecutor& /*executor*/)
{
  auto& registry = dart::simulation::detail::registryOf(world);

  const auto projectAvbdRigidPointJoints = [&]() {
    const bool hasPointJointConfigs
        = dvbd::mayHaveAvbdRigidWorldPointJointConfigs(registry);
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
        = scratch.pointJoints.empty() ? 0u
                                      : dvbd::appendAvbdRigidWorldPointJoints(
                                            registry,
                                            scratch.pointJoints,
                                            scratch.snapshot,
                                            scratch.buildScratch);
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

    dvbd::AvbdRigidWorldContactSolveOptions solveOptions;
    solveOptions.descent.iterations = m_iterations;
    solveOptions.descent.regularization = 1e-12;
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
            solveOptions);
    (void)dvbd::markAvbdRigidWorldFracturedPointJoints(
        registry, scratch.snapshot, solveResult.fracturedJointIndices);
    if (solveResult.jointLinearRows == 0u && solveResult.jointAngularRows == 0u
        && solveResult.motorRows == 0u
        && solveResult.distanceSpringRows == 0u) {
      return false;
    }

    const dvbd::AvbdRigidWorldContactApplyResult projection
        = dvbd::applyAvbdRigidWorldContactVelocityProjection(
            registry, scratch.snapshot, timeStep);
    return projection.bodies != 0u;
  };

  if (shouldSkipRigidBodyContactQuery(world)) {
    if (projectAvbdRigidPointJoints()) {
      return;
    }

    if (m_avbdScratch != nullptr) {
      m_avbdScratch->clear();
    }
    return;
  }

  const bool mayUseAvbdContactDetails
      = mayHaveRigidAvbdContactConfigs(registry);
  const auto queriedContacts = world.queryContacts(
      CollisionQueryOptions{},
      /*includeShapeContactDetails=*/mayUseAvbdContactDetails);
  std::vector<Contact> deactivationContacts;
  std::span<const Contact> contacts = queriedContacts;
  if (world.isDeactivationActiveForStep()) {
    deactivationContacts = world.filterContactsForDeactivation(contacts);
    contacts = deactivationContacts;
  }
  if (contacts.empty()) {
    if (projectAvbdRigidPointJoints()) {
      return;
    }

    if (m_avbdScratch != nullptr) {
      m_avbdScratch->clear();
    }
    return;
  }

  // Internal AVBD rigid contact path (PLAN-104 AVBD): when every active contact
  // has at least one body carrying the private opt-in config, assemble the
  // contact manifold into 6-DOF point-pair rows, solve against the
  // velocity-predicted inertial target, then project the solved displacement
  // back into velocities. The standard rigid position stage still advances
  // poses, so default pipeline ordering is unchanged. Unsupported envelopes
  // fall through to the sequential-impulse path below.
  const auto avbdConfig = mayUseAvbdContactDetails
                              ? rigidAvbdContactStageConfig(registry, contacts)
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
    std::size_t appendedJoints = 0u;
    std::size_t appendedDistanceSprings = 0u;
    if (dvbd::mayHaveAvbdRigidWorldPointJointConfigs(registry)) {
      dvbd::extractAvbdRigidWorldPointJointInputsInto(
          registry, scratch.pointJoints, /*includeWorldAnchors=*/false);
      if (!scratch.pointJoints.empty()) {
        appendedJoints = dvbd::appendAvbdRigidWorldPointJoints(
            registry,
            scratch.pointJoints,
            scratch.snapshot,
            scratch.buildScratch);
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

    if (scratch.snapshot.contacts.size() == contacts.size()
        && (!scratch.snapshot.contacts.empty() || appendedJoints != 0u
            || appendedDistanceSprings != 0u)) {
      const double timeStep = world.getTimeStep();
      dvbd::predictAvbdRigidWorldContactInertialTargets(
          registry, scratch.snapshot, timeStep);

      dvbd::AvbdRigidWorldContactSolveOptions solveOptions;
      solveOptions.warmStart.alpha = avbdConfig->alpha;
      solveOptions.warmStart.gamma = avbdConfig->gamma;
      solveOptions.warmStart.maxStiffness = contactOptions.maxStiffness;
      solveOptions.row.alpha = avbdConfig->alpha;
      solveOptions.row.beta = avbdConfig->beta;
      solveOptions.row.maxStiffness = contactOptions.maxStiffness;
      solveOptions.friction.alpha = avbdConfig->alpha;
      solveOptions.friction.beta = avbdConfig->beta;
      solveOptions.friction.maxStiffness = contactOptions.maxStiffness;
      solveOptions.distanceSpring.beta = avbdConfig->beta;
      solveOptions.distanceSpring.maxStiffness = contactOptions.maxStiffness;
      solveOptions.descent.iterations = m_iterations;
      solveOptions.descent.regularization = 1e-12;

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
              solveOptions);
      (void)dvbd::markAvbdRigidWorldFracturedPointJoints(
          registry, scratch.snapshot, solveResult.fracturedJointIndices);
      if (solveResult.normalRows != 0u || solveResult.frictionRows != 0u
          || solveResult.jointLinearRows != 0u
          || solveResult.jointAngularRows != 0u || solveResult.motorRows != 0u
          || solveResult.distanceSpringRows != 0u) {
        const dvbd::AvbdRigidWorldContactApplyResult projection
            = dvbd::applyAvbdRigidWorldContactVelocityProjection(
                registry, scratch.snapshot, timeStep);
        if (projection.bodies != 0u) {
          return;
        }
      }
    }

    m_avbdScratch->clear();
  }

  if (projectAvbdRigidPointJoints()) {
    // Keep the AVBD joint projection active while ordinary contacts continue
    // through the selected non-AVBD contact solver below.
  } else if (m_avbdScratch != nullptr) {
    m_avbdScratch->clear();
  }

  // Opt-in boxed-LCP path (PLAN-080 WS4): assemble and solve the Coulomb
  // normal+tangent Delassus system with the pivoting Dantzig solver, applying
  // the resulting impulses to body velocities. The default SequentialImpulse
  // path below is unchanged.
  if (world.getContactSolverMethod() == ContactSolverMethod::BoxedLcp) {
    detail::BoxedLcpContactScratch frameScratch(
        world.getMemoryManager().getFrameAllocator());
    detail::applyBoxedLcpContacts(
        registry, contacts, world.getTimeStep(), frameScratch);
    resolveRigidBodyContactPositions(registry, contacts, world.getTimeStep());
    return;
  }

  const auto contactPointVelocity = [](const comps::Velocity& velocity,
                                       const Eigen::Vector3d& arm,
                                       bool isStatic) -> Eigen::Vector3d {
    if (isStatic) {
      return Eigen::Vector3d::Zero();
    }
    return velocity.linear + velocity.angular.cross(arm);
  };
  const auto normalPointVelocity = [](const comps::Velocity& velocity,
                                      const Eigen::Vector3d& normal,
                                      const Eigen::Vector3d& normalArmCross,
                                      bool hasNormalAngular,
                                      bool isStatic) -> double {
    if (isStatic) {
      return 0.0;
    }
    double result = velocity.linear.dot(normal);
    if (hasNormalAngular) {
      result += velocity.angular.dot(normalArmCross);
    }
    return result;
  };

  if (m_contactScratch == nullptr) {
    m_contactScratch = ContactScratchPtr(
        createContactScratch(m_memoryManager),
        ContactScratchDeleter{m_memoryManager});
  }
  auto& constraints = m_contactScratch->constraints;
  constraints.clear();
  constraints.reserve(contacts.size());
  bool hasFrictionConstraints = false;
  for (const auto& contact : contacts) {
    const auto entityA = detail::toRegistryEntity(contact.bodyA.getEntity());
    const auto entityB = detail::toRegistryEntity(contact.bodyB.getEntity());

    // This sequential-impulse solver handles rigid-body pairs only; contacts
    // involving multibody links are resolved by the articulated contact solve.
    if (!registry.all_of<comps::RigidBodyTag>(entityA)
        || !registry.all_of<comps::RigidBodyTag>(entityB)) {
      continue;
    }

    const auto& transformA = registry.get<comps::Transform>(entityA);
    const auto& transformB = registry.get<comps::Transform>(entityB);
    const auto& massA = registry.get<comps::MassProperties>(entityA);
    const auto& massB = registry.get<comps::MassProperties>(entityB);

    const bool staticA
        = detail::hasPrescribedRigidBodyContactResponse(registry, entityA);
    const bool staticB
        = detail::hasPrescribedRigidBodyContactResponse(registry, entityB);

    ContactScratch::NormalConstraint constraint;
    constraint.bodyA = entityA;
    constraint.bodyB = entityB;
    constraint.normal = contact.normal;
    constraint.depth = contact.depth;
    constraint.staticA = staticA;
    constraint.staticB = staticB;
    constraint.armA = Eigen::Vector3d::Zero();
    if (!staticA) {
      constraint.armA = contact.point - transformA.position;
    }
    constraint.armB = Eigen::Vector3d::Zero();
    if (!staticB) {
      constraint.armB = contact.point - transformB.position;
    }
    constraint.invMassA = staticA ? 0.0 : detail::inverseMassOf(massA);
    constraint.invMassB = staticB ? 0.0 : detail::inverseMassOf(massB);
    const double frictionProduct = detail::frictionOf(registry, entityA)
                                   * detail::frictionOf(registry, entityB);
    constraint.friction
        = frictionProduct == 0.0 ? 0.0 : std::sqrt(frictionProduct);

    constraint.normalArmCrossA = staticA
                                     ? Eigen::Vector3d::Zero()
                                     : constraint.armA.cross(constraint.normal);
    constraint.normalArmCrossB = staticB
                                     ? Eigen::Vector3d::Zero()
                                     : constraint.armB.cross(constraint.normal);
    constraint.hasNormalAngularA
        = !staticA && hasContactNormalAngularTerm(constraint.normalArmCrossA);
    constraint.hasNormalAngularB
        = !staticB && hasContactNormalAngularTerm(constraint.normalArmCrossB);
    const bool needsInverseInertiaA
        = !staticA
          && needsContactInverseInertia(
              constraint.normalArmCrossA, constraint.friction);
    const bool needsInverseInertiaB
        = !staticB
          && needsContactInverseInertia(
              constraint.normalArmCrossB, constraint.friction);
    constraint.invInertiaA
        = needsInverseInertiaA
              ? detail::inverseWorldInertiaOf(massA, transformA)
              : Eigen::Matrix3d::Zero();
    constraint.invInertiaB
        = needsInverseInertiaB
              ? detail::inverseWorldInertiaOf(massB, transformB)
              : Eigen::Matrix3d::Zero();
    constraint.normalLinearDeltaA = -constraint.invMassA * constraint.normal;
    constraint.normalLinearDeltaB = constraint.invMassB * constraint.normal;
    constraint.normalAngularDeltaA = Eigen::Vector3d::Zero();
    if (constraint.hasNormalAngularA) {
      constraint.normalAngularDeltaA
          = -constraint.invInertiaA * constraint.normalArmCrossA;
    }
    constraint.normalAngularDeltaB = Eigen::Vector3d::Zero();
    if (constraint.hasNormalAngularB) {
      constraint.normalAngularDeltaB
          = constraint.invInertiaB * constraint.normalArmCrossB;
    }
    const double angular
        = (constraint.hasNormalAngularA
               ? constraint.normalArmCrossA.dot(
                     constraint.invInertiaA * constraint.normalArmCrossA)
               : 0.0)
          + (constraint.hasNormalAngularB
                 ? constraint.normalArmCrossB.dot(
                       constraint.invInertiaB * constraint.normalArmCrossB)
                 : 0.0);
    constraint.effectiveMass
        = constraint.invMassA + constraint.invMassB + angular;
    if (constraint.effectiveMass <= 0.0) {
      continue; // Both bodies are static.
    }
    constraint.inverseEffectiveMass = 1.0 / constraint.effectiveMass;

    // Restitution target from the pre-solve approach velocity (the impact
    // speed). Combine the two materials by taking the larger bounce.
    const double restitution = std::max(
        detail::restitutionOf(registry, entityA),
        detail::restitutionOf(registry, entityB));
    auto& velocityA = registry.get<comps::Velocity>(entityA);
    auto& velocityB = registry.get<comps::Velocity>(entityB);
    constraint.velocityA = &velocityA;
    constraint.velocityB = &velocityB;
    const double initialApproach = normalPointVelocity(
                                       velocityB,
                                       constraint.normal,
                                       constraint.normalArmCrossB,
                                       constraint.hasNormalAngularB,
                                       constraint.staticB)
                                   - normalPointVelocity(
                                       velocityA,
                                       constraint.normal,
                                       constraint.normalArmCrossA,
                                       constraint.hasNormalAngularA,
                                       constraint.staticA);
    constexpr double restitutionThreshold
        = detail::kRigidContactRestitutionThreshold;
    constraint.restitutionVelocity
        = (restitution > 0.0 && initialApproach < -restitutionThreshold)
              ? -restitution * initialApproach
              : 0.0;
    constraint.normalImpulse = 0.0;

    constraint.tangentImpulse1 = 0.0;
    constraint.tangentImpulse2 = 0.0;
    if (constraint.friction > 0.0) {
      // Two tangent directions spanning the contact plane, plus their effective
      // masses, for a friction-pyramid (box) Coulomb model.
      constraint.tangent1 = constraint.normal.unitOrthogonal();
      constraint.tangent2 = constraint.normal.cross(constraint.tangent1);
      const auto tangentMass = [&](const Eigen::Vector3d& tangent) {
        double result = constraint.invMassA + constraint.invMassB;
        if (!constraint.staticA) {
          const Eigen::Vector3d crossTangentA = constraint.armA.cross(tangent);
          result += tangent.dot(
              (constraint.invInertiaA * crossTangentA).cross(constraint.armA));
        }
        if (!constraint.staticB) {
          const Eigen::Vector3d crossTangentB = constraint.armB.cross(tangent);
          result += tangent.dot(
              (constraint.invInertiaB * crossTangentB).cross(constraint.armB));
        }
        return result;
      };
      constraint.tangentMass1 = tangentMass(constraint.tangent1);
      constraint.tangentMass2 = tangentMass(constraint.tangent2);
      hasFrictionConstraints = true;
    } else {
      constraint.tangent1 = Eigen::Vector3d::Zero();
      constraint.tangent2 = Eigen::Vector3d::Zero();
      constraint.tangentMass1 = 0.0;
      constraint.tangentMass2 = 0.0;
    }

    constraints.push_back(constraint);
  }

  // Sequential impulses (Gauss-Seidel) drive each contact's normal approach
  // velocity to its restitution target. The accumulated normal impulse is
  // clamped non-negative so contacts only push, never pull.
  const auto solveNormalImpulse =
      [&](ContactScratch::NormalConstraint& constraint) {
        auto& velocityA = *constraint.velocityA;
        auto& velocityB = *constraint.velocityB;

        const double approach = normalPointVelocity(
                                    velocityB,
                                    constraint.normal,
                                    constraint.normalArmCrossB,
                                    constraint.hasNormalAngularB,
                                    constraint.staticB)
                                - normalPointVelocity(
                                    velocityA,
                                    constraint.normal,
                                    constraint.normalArmCrossA,
                                    constraint.hasNormalAngularA,
                                    constraint.staticA);

        double lambda = -(approach - constraint.restitutionVelocity)
                        * constraint.inverseEffectiveMass;
        const double clamped = std::max(constraint.normalImpulse + lambda, 0.0);
        lambda = clamped - constraint.normalImpulse;
        constraint.normalImpulse = clamped;

        if (lambda == 0.0) {
          return;
        }
        if (!constraint.staticA) {
          velocityA.linear += lambda * constraint.normalLinearDeltaA;
          if (constraint.hasNormalAngularA) {
            velocityA.angular += lambda * constraint.normalAngularDeltaA;
          }
        }
        if (!constraint.staticB) {
          velocityB.linear += lambda * constraint.normalLinearDeltaB;
          if (constraint.hasNormalAngularB) {
            velocityB.angular += lambda * constraint.normalAngularDeltaB;
          }
        }
      };

  if (!hasFrictionConstraints) {
    for (std::size_t iteration = 0; iteration < m_iterations; ++iteration) {
      for (auto& constraint : constraints) {
        solveNormalImpulse(constraint);
      }
    }
  } else {
    for (std::size_t iteration = 0; iteration < m_iterations; ++iteration) {
      for (auto& constraint : constraints) {
        solveNormalImpulse(constraint);

        // Coulomb friction along each tangent, clamped to the friction pyramid
        // bounded by the accumulated normal impulse.
        if (constraint.friction > 0.0) {
          auto& velocityA = *constraint.velocityA;
          auto& velocityB = *constraint.velocityB;
          const double frictionLimit
              = constraint.friction * constraint.normalImpulse;
          const auto solveFriction = [&](const Eigen::Vector3d& tangent,
                                         double tangentMass,
                                         double& tangentImpulse) {
            if (tangentMass <= 0.0 || frictionLimit <= 0.0) {
              return;
            }
            const Eigen::Vector3d tangentVelocity
                = contactPointVelocity(
                      velocityB, constraint.armB, constraint.staticB)
                  - contactPointVelocity(
                      velocityA, constraint.armA, constraint.staticA);
            double tangentLambda = -tangentVelocity.dot(tangent) / tangentMass;
            const double clampedTangent = std::clamp(
                tangentImpulse + tangentLambda, -frictionLimit, frictionLimit);
            tangentLambda = clampedTangent - tangentImpulse;
            tangentImpulse = clampedTangent;
            if (tangentLambda == 0.0) {
              return;
            }

            const Eigen::Vector3d tangentImpulseVector
                = tangentLambda * tangent;
            if (!constraint.staticB) {
              velocityB.linear += constraint.invMassB * tangentImpulseVector;
              velocityB.angular
                  += constraint.invInertiaB
                     * constraint.armB.cross(tangentImpulseVector);
            }
            if (!constraint.staticA) {
              velocityA.linear -= constraint.invMassA * tangentImpulseVector;
              velocityA.angular
                  -= constraint.invInertiaA
                     * constraint.armA.cross(tangentImpulseVector);
            }
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
  }

  resolveRigidBodyContactPositions(registry, contacts, world.getTimeStep());
}

//==============================================================================
std::size_t RigidBodyContactStage::getIterations() const noexcept
{
  return m_iterations;
}

} // namespace dart::simulation::compute
