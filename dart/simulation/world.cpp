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

#include "dart/simulation/world.hpp"

#include "dart/collision/native/collision_object.hpp"
#include "dart/collision/native/collision_world.hpp"
#include "dart/collision/native/contact_manifold.hpp"
#include "dart/collision/native/contact_point.hpp"
#include "dart/collision/native/shapes/shape.hpp"
#include "dart/collision/native/types.hpp"
#include "dart/simulation/body/contact.hpp"
#include "dart/simulation/body/deformable_body.hpp"
#include "dart/simulation/body/rigid_body.hpp"
#include "dart/simulation/common/ecs_utils.hpp"
#include "dart/simulation/common/exceptions.hpp"
#include "dart/simulation/comps/all.hpp"
#include "dart/simulation/compute/deformable_psd_backend.hpp"
#include "dart/simulation/compute/detail/deformable_avbd_replay_state.hpp"
#include "dart/simulation/compute/multibody_dynamics.hpp"
#include "dart/simulation/compute/sequential_executor.hpp"
#include "dart/simulation/compute/variational_integration.hpp"
#include "dart/simulation/compute/world_kinematics_graph.hpp"
#include "dart/simulation/compute/world_step_stage.hpp"
#include "dart/simulation/constraint/loop_closure.hpp"
#include "dart/simulation/constraint/loop_closure_spec.hpp"
#include "dart/simulation/detail/entity_conversion.hpp"
#include "dart/simulation/detail/rigid_avbd/rigid_world_contact.hpp"
#include "dart/simulation/detail/world_registry_access.hpp"
#include "dart/simulation/detail/world_step_schedule.hpp"
#include "dart/simulation/detail/world_storage.hpp"
#include "dart/simulation/diff/physical_parameter.hpp"
#include "dart/simulation/diff/step_derivatives.hpp"
#include "dart/simulation/diff/step_gradient.hpp"
#include "dart/simulation/frame/fixed_frame.hpp"
#include "dart/simulation/frame/frame.hpp"
#include "dart/simulation/frame/free_frame.hpp"
#include "dart/simulation/io/binary_io.hpp"
#include "dart/simulation/io/serializer.hpp"
#include "dart/simulation/multibody/joint.hpp"
#include "dart/simulation/multibody/link.hpp"
#include "dart/simulation/multibody/multibody.hpp"
#include "dart/simulation/world_options.hpp"

#include <dart/config.hpp>

#ifdef DART_HAS_DIFF
  #include "dart/simulation/detail/contact_jacobians.hpp"
  #include "dart/simulation/detail/smooth_jacobians.hpp"
#endif

#include <Eigen/Cholesky>

#include <algorithm>
#include <array>
#include <format>
#include <istream>
#include <limits>
#include <map>
#include <memory>
#include <new>
#include <optional>
#include <ostream>
#include <set>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <cmath>
#include <cstdint>

#if DART_BUILD_PROFILE
  #include <chrono>
#endif

namespace {

#if DART_BUILD_PROFILE
using StepProfileClock = std::chrono::steady_clock;

struct StepProfileTimer
{
  explicit StepProfileTimer(bool enabled_) : enabled(enabled_)
  {
    if (enabled) {
      start = StepProfileClock::now();
    }
  }

  void finish(dart::simulation::compute::WorldStepProfile& profile) const
  {
    if (enabled) {
      profile.wallTime = StepProfileClock::now() - start;
    }
  }

  bool enabled = false;
  StepProfileClock::time_point start{};
};
#endif

template <typename... Components>
std::size_t countEntities(const auto& registry)
{
  std::size_t count = 0;
  auto view = registry.template view<Components...>();
  for (auto entity : view) {
    (void)entity;
    ++count;
  }
  return count;
}

template <typename Component>
bool hasEntityWithName(const auto& registry, std::string_view name)
{
  auto view
      = registry.template view<Component, dart::simulation::comps::Name>();
  for (auto entity : view) {
    const auto& info = view.template get<dart::simulation::comps::Name>(entity);
    if (info.name == name) {
      return true;
    }
  }
  return false;
}

template <typename Component>
entt::entity findEntityWithName(const auto& registry, std::string_view name)
{
  auto view
      = registry.template view<Component, dart::simulation::comps::Name>();
  for (auto entity : view) {
    const auto& info = view.template get<dart::simulation::comps::Name>(entity);
    if (info.name == name) {
      return entity;
    }
  }
  return entt::null;
}

} // namespace

namespace dart::simulation {

namespace ncol = dart::collision::native;

namespace compute {
void reserveDeformableDynamicsRegistryStorage(
    detail::WorldRegistry& registry,
    std::size_t deformableBodyCount,
    common::MemoryAllocator& allocator);
void reserveMultibodyDynamicsRegistryStorage(
    detail::WorldRegistry& registry,
    std::size_t multibodyCount,
    common::MemoryAllocator& allocator);
} // namespace compute

struct World::CollisionQueryCache
{
  template <typename Value>
  using CacheAllocator = common::StlAllocator<Value>;

  struct Key
  {
    entt::entity entity;
    std::size_t shapeIndex;
    std::uint64_t geometryRevision;
    entt::entity multibody;
    bool isLink;

    bool operator==(const Key&) const = default;
  };

  struct ObjectEntry
  {
    entt::entity entity;
    entt::entity multibody;
    bool isLink;
    ncol::CollisionObject object;
  };

  struct ShapeEntrySpec
  {
    Key key;
    const CollisionShape* shape;
    Eigen::Isometry3d pose;
    Eigen::Isometry3d inversePose;
  };

  using KeyVector = std::vector<Key, CacheAllocator<Key>>;
  using ObjectEntryVector
      = std::vector<ObjectEntry, CacheAllocator<ObjectEntry>>;
  using ObjectIdIndexVector
      = std::vector<std::size_t, CacheAllocator<std::size_t>>;
  using ShapeEntrySpecVector
      = std::vector<ShapeEntrySpec, CacheAllocator<ShapeEntrySpec>>;
  using ContactVector = std::vector<Contact, CacheAllocator<Contact>>;
  using CollisionPairVector = std::vector<
      detail::WorldStorage::CollisionPairKey,
      CacheAllocator<detail::WorldStorage::CollisionPairKey>>;

  CollisionQueryCache() = default;

  explicit CollisionQueryCache(common::MemoryAllocator& allocator)
    : keys(CacheAllocator<Key>{allocator}),
      entries(CacheAllocator<ObjectEntry>{allocator}),
      entryByObjectId(CacheAllocator<std::size_t>{allocator}),
      specs(CacheAllocator<ShapeEntrySpec>{allocator}),
      liveRigidBodyJointPairs(
          CacheAllocator<detail::WorldStorage::CollisionPairKey>{allocator}),
      contacts(CacheAllocator<Contact>{allocator})
  {
  }

  void clearObjectsAndResultsPreservingSpecs()
  {
    // queryContacts() rebuilds native objects from `specs` after this call, so
    // keep that vector intact while clearing the derived native/query state.
    collisionWorld.clear();
    keys.clear();
    entries.clear();
    entryByObjectId.clear();
    candidatePairs.pairs.clear();
    candidatePairs.numObjects = 0;
    contacts.clear();
    pairResult.clear();
  }

  void clear()
  {
    clearObjectsAndResultsPreservingSpecs();
    specs.clear();
    liveRigidBodyJointPairs.clear();
  }

  ncol::CollisionWorld collisionWorld;
  KeyVector keys;
  ObjectEntryVector entries;
  ObjectIdIndexVector entryByObjectId;
  ShapeEntrySpecVector specs;
  ncol::BroadPhaseSnapshot candidatePairs;
  CollisionPairVector liveRigidBodyJointPairs;
  ContactVector contacts;
  ncol::CollisionResult pairResult;
};

struct World::ReplayState
{
  template <typename Value>
  using SnapshotAllocator = common::StlAllocator<Value>;

  template <typename Value>
  using SnapshotVector = std::vector<Value, SnapshotAllocator<Value>>;

  template <typename Component>
  using ComponentSnapshot = SnapshotVector<std::pair<entt::entity, Component>>;

  using SnapshotCharTraits = std::char_traits<char>;
  using SnapshotString
      = std::basic_string<char, SnapshotCharTraits, SnapshotAllocator<char>>;

  using DifferentiableParameterSnapshot
      = std::pair<entt::entity, PhysicalParameter>;

  struct VectorState
  {
    explicit VectorState(common::MemoryAllocator& allocator)
      : values(SnapshotAllocator<double>{allocator})
    {
      // Empty.
    }

    SnapshotVector<double> values;
  };

  struct JointLimitsState
  {
    explicit JointLimitsState(common::MemoryAllocator& allocator)
      : lower(allocator),
        upper(allocator),
        velocityLower(allocator),
        velocityUpper(allocator),
        effortLower(allocator),
        effortUpper(allocator)
    {
      // Empty.
    }

    VectorState lower;
    VectorState upper;
    VectorState velocityLower;
    VectorState velocityUpper;
    VectorState effortLower;
    VectorState effortUpper;
  };

  struct JointLayoutState
  {
    explicit JointLayoutState(common::MemoryAllocator& allocator)
      : name(SnapshotAllocator<char>{allocator}),
        springStiffness(allocator),
        dampingCoefficient(allocator),
        restPosition(allocator),
        armature(allocator),
        coulombFriction(allocator),
        limits(allocator)
    {
      // Empty.
    }

    comps::JointType type = comps::JointType::Revolute;
    comps::ActuatorType actuatorType = comps::ActuatorType::Force;
    SnapshotString name;
    VectorState springStiffness;
    VectorState dampingCoefficient;
    VectorState restPosition;
    VectorState armature;
    VectorState coulombFriction;
    double breakForce = 0.0;
    // Mirrors the presence and values of the comps::AvbdJointStiffness sidecar
    // for replay-frame layout change detection.
    bool hasAvbdStiffnessState = false;
    comps::AvbdJointStiffness avbdStiffness;
    JointLimitsState limits;
    Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();
    Eigen::Vector3d axis2 = Eigen::Vector3d::UnitX();
    double pitch = 0.0;
    entt::entity parentLink = entt::null;
    entt::entity childLink = entt::null;
    bool hasRigidBodyFixedJointAnchors = false;
    Eigen::Vector3d rigidBodyFixedJointLocalAnchorParent
        = Eigen::Vector3d::Zero();
    Eigen::Vector3d rigidBodyFixedJointLocalAnchorChild
        = Eigen::Vector3d::Zero();
    Eigen::Quaterniond rigidBodyFixedJointTargetRelativeOrientation
        = Eigen::Quaterniond::Identity();
  };

  struct JointState
  {
    explicit JointState(common::MemoryAllocator& allocator)
      : layout(allocator),
        position(allocator),
        velocity(allocator),
        acceleration(allocator),
        torque(allocator),
        commandVelocity(allocator)
    {
      // Empty.
    }

    entt::entity entity = entt::null;
    JointLayoutState layout;
    VectorState position;
    VectorState velocity;
    VectorState acceleration;
    VectorState torque;
    VectorState commandVelocity;
    bool broken = false;
  };

  struct LinkState
  {
    entt::entity entity = entt::null;
    comps::MassProperties massProperties;
    std::optional<comps::CollisionGeometry> collisionGeometry;
    Eigen::Matrix<double, 6, 1> externalForce
        = Eigen::Matrix<double, 6, 1>::Zero();
  };

  struct PublicFrameState
  {
    entt::entity entity = entt::null;
    comps::FrameState frameState;
    std::optional<comps::FreeFrameProperties> freeFrameProperties;
    std::optional<comps::FixedFrameProperties> fixedFrameProperties;
  };

  struct LoopClosureState
  {
    explicit LoopClosureState(common::MemoryAllocator& allocator)
      : name(SnapshotAllocator<char>{allocator})
    {
      // Empty.
    }

    entt::entity entity = entt::null;
    SnapshotString name;
    comps::LoopClosure loopClosure;
  };

  struct RigidBodyState
  {
    entt::entity entity = entt::null;
    comps::Transform transform;
    comps::Velocity velocity;
    comps::Force force;
    comps::MassProperties massProperties;
    entt::entity parentFrame = entt::null;
    std::optional<comps::ContactMaterial> contactMaterial;
    std::optional<comps::CollisionGeometry> collisionGeometry;
    bool isStatic = false;
    bool isKinematic = false;
    bool hasDeformableGroundBarrier = false;
    bool hasDeformableSurfaceCcdObstacle = false;
    bool hasDeformableObstacleNoCcd = false;
  };

  struct DeformableNodeStateSnapshot
  {
    explicit DeformableNodeStateSnapshot(common::MemoryAllocator& allocator)
      : positions(SnapshotAllocator<Eigen::Vector3d>{allocator}),
        previousPositions(SnapshotAllocator<Eigen::Vector3d>{allocator}),
        velocities(SnapshotAllocator<Eigen::Vector3d>{allocator}),
        masses(SnapshotAllocator<double>{allocator}),
        fixed(SnapshotAllocator<std::uint8_t>{allocator})
    {
      // Empty.
    }

    SnapshotVector<Eigen::Vector3d> positions;
    SnapshotVector<Eigen::Vector3d> previousPositions;
    SnapshotVector<Eigen::Vector3d> velocities;
    SnapshotVector<double> masses;
    SnapshotVector<std::uint8_t> fixed;
  };

  using DeformableNodeStateSnapshotEntry
      = std::pair<entt::entity, DeformableNodeStateSnapshot>;

  struct Frame
  {
    explicit Frame(common::MemoryAllocator& allocator)
      : differentiableParameters(
            SnapshotAllocator<DifferentiableParameterSnapshot>{allocator}),
        deformableNodeStates(
            SnapshotAllocator<DeformableNodeStateSnapshotEntry>{allocator}),
        deformableAvbdWarmStartStates(
            SnapshotAllocator<
                compute::avbd_replay::DeformableAvbdWarmStartReplayState>{
                allocator}),
        multibodyVariationalStates(
            SnapshotAllocator<
                std::pair<entt::entity, compute::MultibodyVariationalState>>{
                allocator}),
        variationalContactDualStates(
            SnapshotAllocator<
                std::pair<entt::entity, comps::VariationalContactDualState>>{
                allocator}),
        deactivationStates(
            SnapshotAllocator<
                std::pair<entt::entity, comps::DeactivationState>>{allocator}),
        joints(SnapshotAllocator<JointState>{allocator}),
        links(SnapshotAllocator<LinkState>{allocator}),
        publicFrames(SnapshotAllocator<PublicFrameState>{allocator}),
        loopClosures(SnapshotAllocator<LoopClosureState>{allocator}),
        rigidBodies(SnapshotAllocator<RigidBodyState>{allocator})
    {
      // Empty.
    }

    bool simulationMode = false;
    Eigen::Vector3d gravity{0.0, 0.0, -9.81};
    RigidBodySolver rigidBodySolver{RigidBodySolver::SequentialImpulse};
    double timeStep = 0.001;
    bool differentiable = false;
    ContactSolverMethod contactSolverMethod{
        ContactSolverMethod::SequentialImpulse};
    ContactGradientMode contactGradientMode{ContactGradientMode::Analytic};
    ComputeAcceleratorPolicy computeAcceleratorPolicy{
        ComputeAcceleratorPolicy::CpuOnly};
    DeactivationOptions deactivationOptions{};
    double time = 0.0;
    std::size_t frame = 0;
    DeformableSolverDiagnostics deformableSolverDiagnostics{};
    double rigidIpcAdaptiveBarrierStiffnessLowerBound = 1.0;
    MultibodyIntegrationMethod multibodyIntegrationMethod{
        MultibodyIntegrationMethod::SemiImplicit};
    std::size_t variationalIntegratorMaxIterations = 100;
    double variationalIntegratorTolerance = 1e-10;
    std::optional<StepDerivatives> stepDerivatives;
    SnapshotVector<DifferentiableParameterSnapshot> differentiableParameters;

    SnapshotVector<DeformableNodeStateSnapshotEntry> deformableNodeStates;
    SnapshotVector<compute::avbd_replay::DeformableAvbdWarmStartReplayState>
        deformableAvbdWarmStartStates;
    ComponentSnapshot<compute::MultibodyVariationalState>
        multibodyVariationalStates;
    ComponentSnapshot<comps::VariationalContactDualState>
        variationalContactDualStates;
    ComponentSnapshot<comps::DeactivationState> deactivationStates;
    SnapshotVector<JointState> joints;
    SnapshotVector<LinkState> links;
    SnapshotVector<PublicFrameState> publicFrames;
    SnapshotVector<LoopClosureState> loopClosures;
    SnapshotVector<RigidBodyState> rigidBodies;
  };

  using FrameAllocator = common::StlAllocator<Frame>;

  explicit ReplayState(common::MemoryAllocator& allocator)
    : frames(FrameAllocator{allocator})
  {
    // Empty.
  }

  bool recordingEnabled = false;
  std::vector<Frame, FrameAllocator> frames;
  std::optional<std::size_t> cursor;
};

namespace {

template <typename Value>
using ReplayScratchAllocator = common::StlAllocator<Value>;

template <typename Value>
using ReplayScratchVector = std::vector<Value, ReplayScratchAllocator<Value>>;

template <typename View>
std::size_t countReplayView(const View& view)
{
  std::size_t count = 0;
  for (auto entity : view) {
    static_cast<void>(entity);
    ++count;
  }
  return count;
}

template <typename SourceVector, typename ReplayVector>
void captureReplayVector(const SourceVector& source, ReplayVector& target)
{
  target.values.clear();
  target.values.reserve(static_cast<std::size_t>(source.size()));
  for (Eigen::Index i = 0; i < source.size(); ++i) {
    target.values.push_back(source[i]);
  }
}

template <typename ReplayVector, typename TargetVector>
void restoreReplayVector(const ReplayVector& source, TargetVector& target)
{
  DART_SIMULATION_THROW_T_IF(
      target.size() != static_cast<Eigen::Index>(source.values.size()),
      InvalidOperationException,
      "Cannot restore replay frame: Joint runtime vector size changed");
  for (Eigen::Index i = 0; i < target.size(); ++i) {
    target[i] = source.values[static_cast<std::size_t>(i)];
  }
}

template <typename LhsVector, typename ReplayVector>
bool sameReplayVector(const LhsVector& lhs, const ReplayVector& rhs)
{
  if (lhs.size() != static_cast<Eigen::Index>(rhs.values.size())) {
    return false;
  }

  for (Eigen::Index i = 0; i < lhs.size(); ++i) {
    if (lhs[i] != rhs.values[static_cast<std::size_t>(i)]) {
      return false;
    }
  }
  return true;
}

template <typename ReplayJointLimits>
void captureReplayJointLimits(
    const comps::JointLimits& source, ReplayJointLimits& target)
{
  captureReplayVector(source.lower, target.lower);
  captureReplayVector(source.upper, target.upper);
  captureReplayVector(source.velocityLower, target.velocityLower);
  captureReplayVector(source.velocityUpper, target.velocityUpper);
  captureReplayVector(source.effortLower, target.effortLower);
  captureReplayVector(source.effortUpper, target.effortUpper);
}

template <typename ReplayJointLimits>
bool sameReplayJointLimits(
    const comps::JointLimits& lhs, const ReplayJointLimits& rhs)
{
  return sameReplayVector(lhs.lower, rhs.lower)
         && sameReplayVector(lhs.upper, rhs.upper)
         && sameReplayVector(lhs.velocityLower, rhs.velocityLower)
         && sameReplayVector(lhs.velocityUpper, rhs.velocityUpper)
         && sameReplayVector(lhs.effortLower, rhs.effortLower)
         && sameReplayVector(lhs.effortUpper, rhs.effortUpper);
}

template <typename JointLayout>
bool sameReplayJointLayout(
    const comps::JointModel& jointModel,
    const comps::JointActuation& jointActuation,
    const comps::AvbdJointStiffness* avbdStiffness,
    const JointLayout& layout)
{
  const bool hasAvbdStiffnessState = avbdStiffness != nullptr;
  const comps::AvbdJointStiffness resolvedStiffness
      = hasAvbdStiffnessState ? *avbdStiffness : comps::AvbdJointStiffness{};
  return jointModel.type == layout.type
         && jointActuation.actuatorType == layout.actuatorType
         && std::string_view{jointModel.name}
                == std::string_view{layout.name.data(), layout.name.size()}
         && sameReplayVector(jointModel.springStiffness, layout.springStiffness)
         && sameReplayVector(
             jointModel.dampingCoefficient, layout.dampingCoefficient)
         && sameReplayVector(jointModel.restPosition, layout.restPosition)
         && sameReplayVector(jointModel.armature, layout.armature)
         && sameReplayVector(jointModel.coulombFriction, layout.coulombFriction)
         && jointModel.breakForce == layout.breakForce
         && hasAvbdStiffnessState == layout.hasAvbdStiffnessState
         && resolvedStiffness.startStiffness
                == layout.avbdStiffness.startStiffness
         && resolvedStiffness.linearStiffness
                == layout.avbdStiffness.linearStiffness
         && resolvedStiffness.angularStiffness
                == layout.avbdStiffness.angularStiffness
         && resolvedStiffness.maxStiffness == layout.avbdStiffness.maxStiffness
         && sameReplayJointLimits(jointModel.limits, layout.limits)
         && jointModel.axis.isApprox(layout.axis, 0.0)
         && jointModel.axis2.isApprox(layout.axis2, 0.0)
         && jointModel.pitch == layout.pitch
         && jointModel.parentLink == layout.parentLink
         && jointModel.childLink == layout.childLink
         && jointModel.hasRigidBodyFixedJointAnchors
                == layout.hasRigidBodyFixedJointAnchors
         && jointModel.rigidBodyFixedJointLocalAnchorParent.isApprox(
             layout.rigidBodyFixedJointLocalAnchorParent, 0.0)
         && jointModel.rigidBodyFixedJointLocalAnchorChild.isApprox(
             layout.rigidBodyFixedJointLocalAnchorChild, 0.0)
         && jointModel.rigidBodyFixedJointTargetRelativeOrientation.coeffs()
                .isApprox(
                    layout.rigidBodyFixedJointTargetRelativeOrientation
                        .coeffs(),
                    0.0);
}

template <typename Component>
std::optional<Component> captureReplayOptionalComponent(
    const detail::WorldRegistry& registry, entt::entity entity)
{
  if (const auto* component = registry.try_get<Component>(entity)) {
    return *component;
  }

  return std::nullopt;
}

bool sameReplayMassProperties(
    const comps::MassProperties& lhs, const comps::MassProperties& rhs)
{
  return lhs.mass == rhs.mass && lhs.inertia.isApprox(rhs.inertia, 0.0)
         && lhs.localCenterOfMass.isApprox(rhs.localCenterOfMass, 0.0);
}

bool sameReplayContactMaterial(
    const std::optional<comps::ContactMaterial>& lhs,
    const comps::ContactMaterial* rhs)
{
  if (lhs.has_value() != (rhs != nullptr)) {
    return false;
  }

  if (!lhs) {
    return true;
  }

  return lhs->restitution == rhs->restitution && lhs->friction == rhs->friction;
}

bool sameReplayCollisionShape(
    const CollisionShape& lhs, const CollisionShape& rhs)
{
  if (lhs.type != rhs.type || lhs.radius != rhs.radius
      || !lhs.halfExtents.isApprox(rhs.halfExtents, 0.0)
      || !lhs.localTransform.matrix().isApprox(rhs.localTransform.matrix(), 0.0)
      || !lhs.normal.isApprox(rhs.normal, 0.0) || lhs.offset != rhs.offset
      || lhs.vertices.size() != rhs.vertices.size()
      || lhs.triangles.size() != rhs.triangles.size()) {
    return false;
  }

  for (std::size_t i = 0; i < lhs.vertices.size(); ++i) {
    if (!lhs.vertices[i].isApprox(rhs.vertices[i], 0.0)) {
      return false;
    }
  }

  for (std::size_t i = 0; i < lhs.triangles.size(); ++i) {
    if (!(lhs.triangles[i].array() == rhs.triangles[i].array()).all()) {
      return false;
    }
  }

  return true;
}

bool sameReplayCollisionGeometry(
    const std::optional<comps::CollisionGeometry>& lhs,
    const comps::CollisionGeometry* rhs)
{
  if (lhs.has_value() != (rhs != nullptr)) {
    return false;
  }

  if (!lhs) {
    return true;
  }

  if (lhs->revision != rhs->revision
      || lhs->shapes.size() != rhs->shapes.size()) {
    return false;
  }

  for (std::size_t i = 0; i < lhs->shapes.size(); ++i) {
    if (!sameReplayCollisionShape(lhs->shapes[i], rhs->shapes[i])) {
      return false;
    }
  }

  return true;
}

bool sameReplayLoopClosureRuntimePolicy(
    const LoopClosureRuntimePolicy& lhs, const LoopClosureRuntimePolicy& rhs)
{
  return lhs.enabled == rhs.enabled && lhs.kinematics == rhs.kinematics
         && lhs.dynamics == rhs.dynamics;
}

bool sameReplayLoopClosure(
    const comps::LoopClosure& lhs, const comps::LoopClosure& rhs)
{
  return lhs.family == rhs.family && lhs.frameA == rhs.frameA
         && lhs.frameB == rhs.frameB
         && lhs.offsetA.matrix().isApprox(rhs.offsetA.matrix(), 0.0)
         && lhs.offsetB.matrix().isApprox(rhs.offsetB.matrix(), 0.0)
         && sameReplayLoopClosureRuntimePolicy(
             lhs.runtimePolicy, rhs.runtimePolicy)
         && lhs.distance == rhs.distance;
}

template <typename RigidBodyStates>
std::size_t findReplayRigidBodyStateIndex(
    const RigidBodyStates& states, entt::entity entity)
{
  for (std::size_t i = 0; i < states.size(); ++i) {
    if (states[i].entity == entity) {
      return i;
    }
  }

  return states.size();
}

template <typename PublicFrameStates>
std::size_t findReplayPublicFrameStateIndex(
    const PublicFrameStates& states, entt::entity entity)
{
  for (std::size_t i = 0; i < states.size(); ++i) {
    if (states[i].entity == entity) {
      return i;
    }
  }

  return states.size();
}

template <typename RigidBodyStates, typename PublicFrameStates>
entt::entity findNearestReplayRigidBodyAncestor(
    const detail::WorldRegistry& registry,
    entt::entity entity,
    const RigidBodyStates& rigidBodyStates,
    const PublicFrameStates& publicFrameStates)
{
  while (entity != entt::null) {
    if (findReplayRigidBodyStateIndex(rigidBodyStates, entity)
        != rigidBodyStates.size()) {
      return entity;
    }

    const auto publicFrameStateIndex
        = findReplayPublicFrameStateIndex(publicFrameStates, entity);
    if (publicFrameStateIndex != publicFrameStates.size()) {
      entity = publicFrameStates[publicFrameStateIndex].frameState.parentFrame;
      continue;
    }

    const auto* frameState = registry.try_get<comps::FrameState>(entity);
    DART_SIMULATION_THROW_T_IF(
        !frameState,
        InvalidOperationException,
        "Cannot restore replay frame: RigidBody frame hierarchy changed");

    entity = frameState->parentFrame;
  }

  return entt::null;
}

template <
    typename RigidBodyStates,
    typename PublicFrameStates,
    typename VisitStateVector,
    typename OrderedVector>
void appendReplayRigidBodyParentBeforeChild(
    const detail::WorldRegistry& registry,
    const RigidBodyStates& states,
    const PublicFrameStates& publicFrameStates,
    VisitStateVector& visitState,
    OrderedVector& ordered,
    std::size_t index)
{
  if (visitState[index] == 2) {
    return;
  }

  DART_SIMULATION_THROW_T_IF(
      visitState[index] == 1,
      InvalidOperationException,
      "Cannot restore replay frame: RigidBody frame hierarchy contains a "
      "cycle");

  visitState[index] = 1;

  const auto parentRigidBody = findNearestReplayRigidBodyAncestor(
      registry, states[index].parentFrame, states, publicFrameStates);
  if (parentRigidBody != entt::null) {
    const auto parentIndex
        = findReplayRigidBodyStateIndex(states, parentRigidBody);
    DART_SIMULATION_THROW_T_IF(
        parentIndex == states.size(),
        InvalidOperationException,
        "Cannot restore replay frame: RigidBody ancestor is missing");

    appendReplayRigidBodyParentBeforeChild(
        registry, states, publicFrameStates, visitState, ordered, parentIndex);
  }

  visitState[index] = 2;
  ordered.push_back(index);
}

template <typename RigidBodyStates, typename PublicFrameStates>
auto orderReplayRigidBodiesParentBeforeChild(
    const detail::WorldRegistry& registry,
    const RigidBodyStates& states,
    const PublicFrameStates& publicFrameStates,
    common::MemoryAllocator& allocator)
{
  ReplayScratchVector<std::size_t> ordered(
      ReplayScratchAllocator<std::size_t>{allocator});
  ordered.reserve(states.size());

  ReplayScratchVector<int> visitState(
      states.size(), 0, ReplayScratchAllocator<int>{allocator});
  for (std::size_t i = 0; i < states.size(); ++i) {
    appendReplayRigidBodyParentBeforeChild(
        registry, states, publicFrameStates, visitState, ordered, i);
  }

  return ordered;
}

template <typename Component>
auto captureReplayComponents(
    const detail::WorldRegistry& registry, common::MemoryAllocator& allocator)
{
  using SnapshotValue = std::pair<entt::entity, Component>;
  ReplayScratchVector<SnapshotValue> snapshot(
      ReplayScratchAllocator<SnapshotValue>{allocator});
  auto view = registry.view<Component>();
  snapshot.reserve(countReplayView(view));
  for (auto entity : view) {
    snapshot.emplace_back(entity, view.template get<Component>(entity));
  }
  std::ranges::sort(snapshot, [](const auto& lhs, const auto& rhs) {
    return static_cast<std::uint32_t>(lhs.first)
           < static_cast<std::uint32_t>(rhs.first);
  });
  return snapshot;
}

template <typename ReplayVector, typename SourceVector>
void captureReplayVectorPayload(
    ReplayVector& target, const SourceVector& source)
{
  target.assign(source.begin(), source.end());
}

template <typename DeformableNodeStateSnapshot>
auto captureReplayDeformableNodeStates(
    const detail::WorldRegistry& registry, common::MemoryAllocator& allocator)
{
  using SnapshotValue = std::pair<entt::entity, DeformableNodeStateSnapshot>;
  ReplayScratchVector<SnapshotValue> snapshot(
      ReplayScratchAllocator<SnapshotValue>{allocator});
  auto view = registry.view<comps::DeformableNodeState>();
  snapshot.reserve(countReplayView(view));
  for (auto entity : view) {
    const auto& state = view.template get<comps::DeformableNodeState>(entity);
    const auto& model = registry.get<comps::DeformableNodeModel>(entity);
    DeformableNodeStateSnapshot snapshotState(allocator);
    captureReplayVectorPayload(snapshotState.positions, state.positions);
    captureReplayVectorPayload(
        snapshotState.previousPositions, state.previousPositions);
    captureReplayVectorPayload(snapshotState.velocities, state.velocities);
    captureReplayVectorPayload(snapshotState.masses, model.masses);
    captureReplayVectorPayload(snapshotState.fixed, model.fixed);
    snapshot.emplace_back(entity, std::move(snapshotState));
  }
  std::ranges::sort(snapshot, [](const auto& lhs, const auto& rhs) {
    return static_cast<std::uint32_t>(lhs.first)
           < static_cast<std::uint32_t>(rhs.first);
  });
  return snapshot;
}

template <typename Component, typename Snapshot>
void validateReplayComponents(
    const detail::WorldRegistry& registry,
    const Snapshot& snapshot,
    std::string_view componentName)
{
  auto view = registry.view<Component>();
  DART_SIMULATION_THROW_T_IF(
      countReplayView(view) != snapshot.size(),
      InvalidOperationException,
      "Cannot restore replay frame: {} component count changed",
      componentName);

  for (const auto& [entity, component] : snapshot) {
    DART_SIMULATION_THROW_T_IF(
        !registry.valid(entity),
        InvalidOperationException,
        "Cannot restore replay frame: {} references an entity that no longer "
        "exists",
        componentName);
    DART_SIMULATION_THROW_T_IF(
        !registry.all_of<Component>(entity),
        InvalidOperationException,
        "Cannot restore replay frame: {} references an entity with changed "
        "component layout",
        componentName);
  }
}

template <typename Component, typename Snapshot, typename EntityPredicate>
void validateReplayTransientComponents(
    const detail::WorldRegistry& registry,
    const Snapshot& snapshot,
    std::string_view componentName,
    EntityPredicate&& entityPredicate)
{
  for (const auto& [entity, component] : snapshot) {
    static_cast<void>(component);
    DART_SIMULATION_THROW_T_IF(
        !registry.valid(entity),
        InvalidOperationException,
        "Cannot restore replay frame: {} references an entity that no longer "
        "exists",
        componentName);
    DART_SIMULATION_THROW_T_IF(
        !entityPredicate(registry, entity),
        InvalidOperationException,
        "Cannot restore replay frame: {} references an entity with changed "
        "component layout",
        componentName);
  }
}

template <typename Component, typename Snapshot>
void restoreReplayComponents(
    detail::WorldRegistry& registry,
    const Snapshot& snapshot,
    std::string_view componentName)
{
  validateReplayComponents<Component>(registry, snapshot, componentName);

  for (const auto& [entity, component] : snapshot) {
    registry.replace<Component>(entity, component);
  }
}

template <typename SourceVector, typename TargetVector>
void restoreReplayVectorPayload(
    const SourceVector& source,
    TargetVector& target,
    std::string_view componentName,
    std::string_view payloadName)
{
  DART_SIMULATION_THROW_T_IF(
      source.size() != target.size(),
      InvalidOperationException,
      "Cannot restore replay frame: {} {} size changed",
      componentName,
      payloadName);
  std::copy(source.begin(), source.end(), target.begin());
}

template <typename Snapshot>
void restoreReplayDeformableNodeStates(
    detail::WorldRegistry& registry, const Snapshot& snapshot)
{
  constexpr std::string_view componentName = "DeformableNodeState";
  validateReplayComponents<comps::DeformableNodeState>(
      registry, snapshot, componentName);

  for (const auto& [entity, replayState] : snapshot) {
    auto& state = registry.get<comps::DeformableNodeState>(entity);
    auto& model = registry.get<comps::DeformableNodeModel>(entity);
    restoreReplayVectorPayload(
        replayState.positions, state.positions, componentName, "positions");
    restoreReplayVectorPayload(
        replayState.previousPositions,
        state.previousPositions,
        componentName,
        "previousPositions");
    restoreReplayVectorPayload(
        replayState.velocities, state.velocities, componentName, "velocities");
    restoreReplayVectorPayload(
        replayState.masses, model.masses, componentName, "masses");
    restoreReplayVectorPayload(
        replayState.fixed, model.fixed, componentName, "fixed");
  }
}

compute::MultibodyVariationalState makeReplayMultibodyVariationalState(
    common::MemoryAllocator& allocator)
{
  using State = compute::MultibodyVariationalState;
  return State{
      false,
      State::DeltaTransformVector{
          common::StlAllocator<Eigen::Isometry3d>{allocator}},
      State::MomentumVector{
          common::StlAllocator<Eigen::Matrix<double, 6, 1>>{allocator}}};
}

void restoreReplayMultibodyVariationalState(
    const compute::MultibodyVariationalState& source,
    compute::MultibodyVariationalState& target,
    common::MemoryAllocator& allocator)
{
  using State = compute::MultibodyVariationalState;
  target.bootstrapped = source.bootstrapped;

  State::DeltaTransformVector transforms{
      common::StlAllocator<Eigen::Isometry3d>{allocator}};
  transforms.assign(
      source.previousDeltaTransform.begin(),
      source.previousDeltaTransform.end());
  target.previousDeltaTransform = std::move(transforms);

  State::MomentumVector momentum{
      common::StlAllocator<Eigen::Matrix<double, 6, 1>>{allocator}};
  momentum.assign(
      source.previousMomentum.begin(), source.previousMomentum.end());
  target.previousMomentum = std::move(momentum);
}

comps::VariationalContactDualState makeReplayVariationalContactDualState(
    common::MemoryAllocator& allocator)
{
  using State = comps::VariationalContactDualState;
  return State{State::DualVector{common::StlAllocator<double>{allocator}}, 0u};
}

void restoreReplayVariationalContactDualState(
    const comps::VariationalContactDualState& source,
    comps::VariationalContactDualState& target,
    common::MemoryAllocator& allocator)
{
  using State = comps::VariationalContactDualState;
  State::DualVector duals{common::StlAllocator<double>{allocator}};
  duals.assign(source.duals.begin(), source.duals.end());
  target.duals = std::move(duals);
  target.stepsSinceDualUpdate = source.stepsSinceDualUpdate;
}

template <
    typename Component,
    typename Snapshot,
    typename EntityPredicate,
    typename Restorer>
void restoreReplayTransientComponentsWithRestorer(
    detail::WorldRegistry& registry,
    const Snapshot& snapshot,
    std::string_view componentName,
    common::MemoryAllocator& allocator,
    EntityPredicate&& entityPredicate,
    Restorer&& restorer)
{
  validateReplayTransientComponents<Component>(
      registry, snapshot, componentName, entityPredicate);

  ReplayScratchVector<std::uint32_t> snapshotEntities(
      ReplayScratchAllocator<std::uint32_t>{allocator});
  snapshotEntities.reserve(snapshot.size());
  for (const auto& [entity, component] : snapshot) {
    static_cast<void>(component);
    snapshotEntities.push_back(static_cast<std::uint32_t>(entity));
  }
  std::ranges::sort(snapshotEntities);

  ReplayScratchVector<entt::entity> staleEntities(
      ReplayScratchAllocator<entt::entity>{allocator});
  auto view = registry.view<Component>();
  for (auto entity : view) {
    if (!std::ranges::binary_search(
            snapshotEntities, static_cast<std::uint32_t>(entity))) {
      staleEntities.push_back(entity);
    }
  }
  for (auto entity : staleEntities) {
    registry.remove<Component>(entity);
  }

  for (const auto& [entity, component] : snapshot) {
    restorer(entity, component);
  }
}

template <typename Component, typename Snapshot, typename EntityPredicate>
void restoreReplayTransientComponents(
    detail::WorldRegistry& registry,
    const Snapshot& snapshot,
    std::string_view componentName,
    common::MemoryAllocator& allocator,
    EntityPredicate&& entityPredicate)
{
  restoreReplayTransientComponentsWithRestorer<Component>(
      registry,
      snapshot,
      componentName,
      allocator,
      std::forward<EntityPredicate>(entityPredicate),
      [&](entt::entity entity, const Component& component) {
        registry.emplace_or_replace<Component>(entity, component);
      });
}

bool isReplayPublicFrameEntity(
    const detail::WorldRegistry& registry, entt::entity entity)
{
  return registry.all_of<comps::FrameState, comps::FrameCache>(entity)
         && !registry.all_of<comps::RigidBodyTag>(entity)
         && !registry.all_of<comps::LinkModel>(entity)
         && (registry.all_of<comps::FreeFrameTag, comps::FreeFrameProperties>(
                 entity)
             || registry
                    .all_of<comps::FixedFrameTag, comps::FixedFrameProperties>(
                        entity));
}

std::size_t countReplayPublicFrameEntities(
    const detail::WorldRegistry& registry)
{
  std::size_t count = 0;
  auto frameView = registry.view<comps::FrameState>();
  for (auto entity : frameView) {
    if (isReplayPublicFrameEntity(registry, entity)) {
      ++count;
    }
  }
  return count;
}

template <typename LoopClosureState>
auto captureReplayLoopClosures(
    const detail::WorldRegistry& registry, common::MemoryAllocator& allocator)
{
  ReplayScratchVector<LoopClosureState> states(
      ReplayScratchAllocator<LoopClosureState>{allocator});
  auto view = registry.view<comps::LoopClosure, comps::Name>();
  states.reserve(countReplayView(view));
  for (auto entity : view) {
    LoopClosureState state(allocator);
    state.entity = entity;
    const auto& name = view.get<comps::Name>(entity).name;
    state.name.assign(name.begin(), name.end());
    state.loopClosure = view.get<comps::LoopClosure>(entity);
    states.push_back(std::move(state));
  }
  std::ranges::sort(states, [](const auto& lhs, const auto& rhs) {
    return static_cast<std::uint32_t>(lhs.entity)
           < static_cast<std::uint32_t>(rhs.entity);
  });
  return states;
}

template <typename LoopClosureStates>
void validateReplayLoopClosures(
    const detail::WorldRegistry& registry, const LoopClosureStates& states)
{
  auto view = registry.view<comps::LoopClosure>();
  DART_SIMULATION_THROW_T_IF(
      countReplayView(view) != states.size(),
      InvalidOperationException,
      "Cannot restore replay frame: LoopClosure component count changed");

  for (const auto& state : states) {
    const bool layoutChanged
        = !registry.valid(state.entity)
          || !registry.all_of<comps::LoopClosure, comps::Name>(state.entity);
    DART_SIMULATION_THROW_T_IF(
        layoutChanged,
        InvalidOperationException,
        "Cannot restore replay frame: LoopClosure entity layout changed");

    const auto& name = registry.get<comps::Name>(state.entity);
    const auto& loopClosure = registry.get<comps::LoopClosure>(state.entity);
    const std::string_view stateName{state.name.data(), state.name.size()};
    DART_SIMULATION_THROW_T_IF(
        std::string_view{name.name} != stateName
            || !sameReplayLoopClosure(loopClosure, state.loopClosure),
        InvalidOperationException,
        "Cannot restore replay frame: LoopClosure entity layout changed");
  }
}

void markFrameCachesDirty(detail::WorldRegistry& registry)
{
  auto frameView = registry.view<comps::FrameCache>();
  for (auto entity : frameView) {
    auto& cache = frameView.get<comps::FrameCache>(entity);
    cache.needTransformUpdate = true;
  }
}

//==============================================================================
// Folds the full internal solver stats into the curated public diagnostics.
DeformableSolverDiagnostics makeDeformableSolverDiagnostics(
    const compute::DeformableSolverStats& stats)
{
  DeformableSolverDiagnostics diagnostics;
  diagnostics.bodyCount = stats.bodyCount;
  diagnostics.nodeCount = stats.nodeCount;
  diagnostics.edgeCount = stats.edgeCount;
  diagnostics.solverIterations = stats.solverIterations;
  diagnostics.objectiveEvaluations = stats.objectiveEvaluations;
  diagnostics.lineSearchTrials = stats.lineSearchTrials;
  diagnostics.projectedNewtonSteps = stats.projectedNewtonSteps;
  diagnostics.projectedNewtonFallbacks = stats.projectedNewtonFallbacks;
  diagnostics.projectedNewtonHessianNonZeros
      = stats.projectedNewtonHessianNonZeros;
  diagnostics.projectedNewtonHessianStorageBytes
      = stats.projectedNewtonHessianStorageBytes;
  diagnostics.projectedNewtonIterativeSolves
      = stats.projectedNewtonIterativeSolves;
  diagnostics.projectedNewtonMatrixFreeSolves
      = stats.projectedNewtonMatrixFreeSolves;
  diagnostics.projectedNewtonIterativeIterations
      = stats.projectedNewtonIterativeIterations;
  diagnostics.projectedNewtonIterativeMaxError
      = stats.projectedNewtonIterativeMaxError;
  diagnostics.selfContactBarrierActiveContacts
      = stats.selfContactBarrierActiveContacts;
  diagnostics.surfaceContactCandidateBuilds
      = stats.surfaceContactCandidateBuilds;
  diagnostics.surfaceContactCandidatePairCapacity
      = stats.surfaceContactCandidatePairCapacity;
  diagnostics.surfaceContactCandidateRejectedPairs
      = stats.surfaceContactCandidateRejectedPairs;
  diagnostics.surfaceContactPointTriangleCandidates
      = stats.surfaceContactPointTriangleCandidates;
  diagnostics.surfaceContactEdgeEdgeCandidates
      = stats.surfaceContactEdgeEdgeCandidates;
  diagnostics.surfaceContactCcdPointTriangleChecks
      = stats.surfaceContactCcdPointTriangleChecks;
  diagnostics.surfaceContactCcdEdgeEdgeChecks
      = stats.surfaceContactCcdEdgeEdgeChecks;
  diagnostics.surfaceContactCcdHits = stats.surfaceContactCcdHits;
  diagnostics.surfaceContactCcdLimitedSteps
      = stats.surfaceContactCcdLimitedSteps;
  diagnostics.surfaceContactCcdZeroStepCount
      = stats.surfaceContactCcdZeroStepCount;
  diagnostics.interBodySurfaceContactCandidateBuilds
      = stats.interBodySurfaceContactCandidateBuilds;
  diagnostics.interBodySurfaceContactCandidatePairCapacity
      = stats.interBodySurfaceContactCandidatePairCapacity;
  diagnostics.interBodySurfaceContactCandidateRejectedPairs
      = stats.interBodySurfaceContactCandidateRejectedPairs;
  diagnostics.interBodySurfaceContactPointTriangleCandidates
      = stats.interBodySurfaceContactPointTriangleCandidates;
  diagnostics.interBodySurfaceContactEdgeEdgeCandidates
      = stats.interBodySurfaceContactEdgeEdgeCandidates;
  diagnostics.interBodySurfaceContactCcdPointTriangleChecks
      = stats.interBodySurfaceContactCcdPointTriangleChecks;
  diagnostics.interBodySurfaceContactCcdEdgeEdgeChecks
      = stats.interBodySurfaceContactCcdEdgeEdgeChecks;
  diagnostics.interBodySurfaceContactCcdHits
      = stats.interBodySurfaceContactCcdHits;
  diagnostics.interBodySurfaceContactCcdLimitedSteps
      = stats.interBodySurfaceContactCcdLimitedSteps;
  diagnostics.interBodySurfaceContactCcdZeroStepCount
      = stats.interBodySurfaceContactCcdZeroStepCount;
  diagnostics.staticRigidSurfaceCcdSnapshotBuilds
      = stats.staticRigidSurfaceCcdSnapshotBuilds;
  diagnostics.staticRigidSurfaceCcdBoxCount
      = stats.staticRigidSurfaceCcdBoxCount;
  diagnostics.staticRigidSurfaceCcdSphereCount
      = stats.staticRigidSurfaceCcdSphereCount;
  diagnostics.staticRigidSurfaceCcdTriangleCount
      = stats.staticRigidSurfaceCcdTriangleCount;
  diagnostics.staticRigidSurfaceCcdEdgeCount
      = stats.staticRigidSurfaceCcdEdgeCount;
  diagnostics.staticRigidSurfaceCcdCandidateBuilds
      = stats.staticRigidSurfaceCcdCandidateBuilds;
  diagnostics.staticRigidSurfaceCcdCandidatePairCapacity
      = stats.staticRigidSurfaceCcdCandidatePairCapacity;
  diagnostics.staticRigidSurfaceCcdCandidateRejectedPairs
      = stats.staticRigidSurfaceCcdCandidateRejectedPairs;
  diagnostics.staticRigidSurfaceCcdPointTriangleCandidates
      = stats.staticRigidSurfaceCcdPointTriangleCandidates;
  diagnostics.staticRigidSurfaceCcdEdgeEdgeCandidates
      = stats.staticRigidSurfaceCcdEdgeEdgeCandidates;
  diagnostics.staticRigidSurfaceCcdPointTriangleChecks
      = stats.staticRigidSurfaceCcdPointTriangleChecks;
  diagnostics.staticRigidSurfaceCcdEdgeEdgeChecks
      = stats.staticRigidSurfaceCcdEdgeEdgeChecks;
  diagnostics.staticRigidSurfaceCcdHits = stats.staticRigidSurfaceCcdHits;
  diagnostics.staticRigidSurfaceCcdLimitedSteps
      = stats.staticRigidSurfaceCcdLimitedSteps;
  diagnostics.staticRigidSurfaceCcdZeroStepCount
      = stats.staticRigidSurfaceCcdZeroStepCount;
  diagnostics.movingRigidSurfaceCcdSnapshotBuilds
      = stats.movingRigidSurfaceCcdSnapshotBuilds;
  diagnostics.movingRigidSurfaceCcdBoxCount
      = stats.movingRigidSurfaceCcdBoxCount;
  diagnostics.movingRigidSurfaceCcdSampleCount
      = stats.movingRigidSurfaceCcdSampleCount;
  diagnostics.movingRigidSurfaceCcdInflatedBoxCount
      = stats.movingRigidSurfaceCcdInflatedBoxCount;
  diagnostics.movingRigidSurfaceCcdTriangleCount
      = stats.movingRigidSurfaceCcdTriangleCount;
  diagnostics.movingRigidSurfaceCcdEdgeCount
      = stats.movingRigidSurfaceCcdEdgeCount;
  diagnostics.movingRigidSurfaceCcdCandidateBuilds
      = stats.movingRigidSurfaceCcdCandidateBuilds;
  diagnostics.movingRigidSurfaceCcdCandidatePairCapacity
      = stats.movingRigidSurfaceCcdCandidatePairCapacity;
  diagnostics.movingRigidSurfaceCcdCandidateRejectedPairs
      = stats.movingRigidSurfaceCcdCandidateRejectedPairs;
  diagnostics.movingRigidSurfaceCcdPointTriangleCandidates
      = stats.movingRigidSurfaceCcdPointTriangleCandidates;
  diagnostics.movingRigidSurfaceCcdEdgeEdgeCandidates
      = stats.movingRigidSurfaceCcdEdgeEdgeCandidates;
  diagnostics.movingRigidSurfaceCcdPointTriangleChecks
      = stats.movingRigidSurfaceCcdPointTriangleChecks;
  diagnostics.movingRigidSurfaceCcdEdgeEdgeChecks
      = stats.movingRigidSurfaceCcdEdgeEdgeChecks;
  diagnostics.movingRigidSurfaceCcdHits = stats.movingRigidSurfaceCcdHits;
  diagnostics.movingRigidSurfaceCcdLimitedSteps
      = stats.movingRigidSurfaceCcdLimitedSteps;
  diagnostics.movingRigidSurfaceCcdZeroStepCount
      = stats.movingRigidSurfaceCcdZeroStepCount;
  diagnostics.frictionDissipation = stats.frictionDissipation;
  diagnostics.minActiveContactDistance = stats.minActiveContactDistance;
  diagnostics.convergedActiveContactCount = stats.convergedActiveContactCount;
  return diagnostics;
}

//==============================================================================
template <typename Registry>
WorldEcsDiagnostics makeWorldEcsDiagnostics(const Registry& registry)
{
  WorldEcsDiagnostics diagnostics;

  const auto* entityStorage = registry.template storage<entt::entity>();
  if (entityStorage != nullptr) {
    for (auto entity : *entityStorage) {
      if (registry.valid(entity)) {
        ++diagnostics.entityCount;
      }
    }
    diagnostics.entityCapacity = entityStorage->capacity();
  }

  for (auto&& [id, storage] : registry.storage()) {
    const auto size = storage.size();
    const auto capacity = storage.capacity();
    diagnostics.storages.push_back(
        WorldEcsStorageDiagnostics{
            static_cast<std::size_t>(id),
            size,
            capacity,
        });
    diagnostics.componentCount += size;
    diagnostics.componentCapacity += capacity;
  }

  diagnostics.storageCount = diagnostics.storages.size();
  return diagnostics;
}

//==============================================================================
void executeKinematicsGraph(World& world, compute::ComputeExecutor& executor)
{
  compute::WorldKinematicsGraph graph(
      world, world.getMemoryManager().getFreeAllocator());
  graph.execute(executor);
}

//==============================================================================
bool isValidWorldSyncStage(WorldSyncStage stage)
{
  switch (stage) {
    case WorldSyncStage::Kinematics:
      return true;
  }

  return false;
}

//==============================================================================
bool isValidRigidBodySolver(RigidBodySolver solver)
{
  switch (solver) {
    case RigidBodySolver::SequentialImpulse:
    case RigidBodySolver::Ipc:
      return true;
  }

  return false;
}

//==============================================================================
bool isValidContactSolverMethod(ContactSolverMethod method)
{
  switch (method) {
    case ContactSolverMethod::SequentialImpulse:
    case ContactSolverMethod::BoxedLcp:
      return true;
  }

  return false;
}

//==============================================================================
bool isValidContactGradientMode(ContactGradientMode mode)
{
  switch (mode) {
    case ContactGradientMode::Analytic:
    case ContactGradientMode::ComplementarityAware:
    case ContactGradientMode::PreContactSurrogate:
      return true;
  }

  return false;
}

//==============================================================================
bool isValidComputeAcceleratorPolicy(ComputeAcceleratorPolicy policy)
{
  switch (policy) {
    case ComputeAcceleratorPolicy::CpuOnly:
    case ComputeAcceleratorPolicy::PreferAccelerated:
      return true;
  }

  return false;
}

//==============================================================================
bool isValidMultibodyIntegrationFamily(MultibodyIntegrationFamily family)
{
  switch (family) {
    case MultibodyIntegrationFamily::SemiImplicit:
    case MultibodyIntegrationFamily::Variational:
      return true;
  }

  return false;
}

//==============================================================================
void validateDeactivationOptions(const DeactivationOptions& options)
{
  DART_SIMULATION_THROW_T_IF(
      !std::isfinite(options.linearSpeedThreshold)
          || options.linearSpeedThreshold < 0.0,
      InvalidArgumentException,
      "DeactivationOptions.linearSpeedThreshold must be finite and "
      "non-negative");
  DART_SIMULATION_THROW_T_IF(
      !std::isfinite(options.angularSpeedThreshold)
          || options.angularSpeedThreshold < 0.0,
      InvalidArgumentException,
      "DeactivationOptions.angularSpeedThreshold must be finite and "
      "non-negative");
  DART_SIMULATION_THROW_T_IF(
      !std::isfinite(options.generalizedSpeedThreshold)
          || options.generalizedSpeedThreshold < 0.0,
      InvalidArgumentException,
      "DeactivationOptions.generalizedSpeedThreshold must be finite and "
      "non-negative");
  DART_SIMULATION_THROW_T_IF(
      !std::isfinite(options.timeUntilSleep) || options.timeUntilSleep < 0.0,
      InvalidArgumentException,
      "DeactivationOptions.timeUntilSleep must be finite and non-negative");
  DART_SIMULATION_THROW_T_IF(
      !std::isfinite(options.wakeThresholdScale)
          || options.wakeThresholdScale < 1.0,
      InvalidArgumentException,
      "DeactivationOptions.wakeThresholdScale must be finite and at least 1");
  DART_SIMULATION_THROW_T_IF(
      !std::isfinite(options.disturbanceForceThreshold)
          || options.disturbanceForceThreshold < 0.0,
      InvalidArgumentException,
      "DeactivationOptions.disturbanceForceThreshold must be finite and "
      "non-negative");
}

//==============================================================================
std::uint8_t encodeRigidBodySolver(RigidBodySolver solver)
{
  switch (solver) {
    case RigidBodySolver::SequentialImpulse:
      return 0u;
    case RigidBodySolver::Ipc:
      return 1u;
  }

  DART_SIMULATION_THROW_T( // LCOV_EXCL_LINE
      InvalidArgumentException, "Rigid-body solver is invalid");
  return 0u;
}

//==============================================================================
RigidBodySolver decodeRigidBodySolver(std::uint8_t value)
{
  switch (value) {
    case 0u:
      return RigidBodySolver::SequentialImpulse;
    case 1u:
      return RigidBodySolver::Ipc;
  }

  DART_SIMULATION_THROW_T(
      InvalidArgumentException,
      "Serialized World rigid-body solver value is invalid");
  return RigidBodySolver::SequentialImpulse;
}

//==============================================================================
std::uint8_t encodeContactSolverMethod(ContactSolverMethod method)
{
  switch (method) {
    case ContactSolverMethod::SequentialImpulse:
      return 0u;
    case ContactSolverMethod::BoxedLcp:
      return 1u;
  }

  DART_SIMULATION_THROW_T( // LCOV_EXCL_LINE
      InvalidArgumentException, "Contact solver method is invalid");
  return 0u;
}

//==============================================================================
ContactSolverMethod decodeContactSolverMethod(std::uint8_t value)
{
  switch (value) {
    case 0u:
      return ContactSolverMethod::SequentialImpulse;
    case 1u:
      return ContactSolverMethod::BoxedLcp;
  }

  DART_SIMULATION_THROW_T(
      InvalidArgumentException,
      "Serialized World contact solver method value is invalid");
  return ContactSolverMethod::SequentialImpulse;
}

//==============================================================================
std::uint8_t encodeContactGradientMode(ContactGradientMode mode)
{
  switch (mode) {
    case ContactGradientMode::Analytic:
      return 0u;
    case ContactGradientMode::ComplementarityAware:
      return 1u;
    case ContactGradientMode::PreContactSurrogate:
      return 2u;
  }

  DART_SIMULATION_THROW_T( // LCOV_EXCL_LINE
      InvalidArgumentException, "Contact gradient mode is invalid");
  return 0u;
}

//==============================================================================
ContactGradientMode decodeContactGradientMode(std::uint8_t value)
{
  switch (value) {
    case 0u:
      return ContactGradientMode::Analytic;
    case 1u:
      return ContactGradientMode::ComplementarityAware;
    case 2u:
      return ContactGradientMode::PreContactSurrogate;
  }

  DART_SIMULATION_THROW_T(
      InvalidArgumentException,
      "Serialized World contact gradient mode value is invalid");
  return ContactGradientMode::Analytic;
}

//==============================================================================
std::uint8_t encodeComputeAcceleratorPolicy(ComputeAcceleratorPolicy policy)
{
  switch (policy) {
    case ComputeAcceleratorPolicy::CpuOnly:
      return 0u;
    case ComputeAcceleratorPolicy::PreferAccelerated:
      return 1u;
  }

  DART_SIMULATION_THROW_T( // LCOV_EXCL_LINE
      InvalidArgumentException, "Compute accelerator policy is invalid");
  return 0u;
}

//==============================================================================
ComputeAcceleratorPolicy decodeComputeAcceleratorPolicy(std::uint8_t value)
{
  switch (value) {
    case 0u:
      return ComputeAcceleratorPolicy::CpuOnly;
    case 1u:
      return ComputeAcceleratorPolicy::PreferAccelerated;
  }

  DART_SIMULATION_THROW_T(
      InvalidArgumentException,
      "Serialized World compute accelerator policy value is invalid");
  return ComputeAcceleratorPolicy::CpuOnly;
}

//==============================================================================
detail::BuiltInRigidBodySolverFamily toBuiltInRigidBodySolverFamily(
    RigidBodySolver solver)
{
  switch (solver) {
    case RigidBodySolver::SequentialImpulse:
      return detail::BuiltInRigidBodySolverFamily::SequentialImpulse;
    case RigidBodySolver::Ipc:
      return detail::BuiltInRigidBodySolverFamily::Ipc;
  }

  DART_SIMULATION_THROW_T( // LCOV_EXCL_LINE
      InvalidArgumentException, "Rigid-body solver is invalid");
  return detail::BuiltInRigidBodySolverFamily::SequentialImpulse;
}

//==============================================================================
bool hasMultibodyStructures(const World& world)
{
  const auto view = detail::registryOf(world).view<comps::MultibodyStructure>();
  return view.begin() != view.end();
}

//==============================================================================
bool hasAdvanceableRigidBodyStructures(const World& world)
{
  const auto& registry = detail::registryOf(world);
  const auto view = registry.view<comps::RigidBodyTag>();
  for (const auto entity : view) {
    if (!registry.all_of<comps::StaticBodyTag>(entity)) {
      return true;
    }
  }
  return false;
}

//==============================================================================
bool hasDirtyFrameCaches(const World& world)
{
  const auto view = detail::registryOf(world).view<comps::FrameCache>();
  for (const auto entity : view) {
    if (view.get<comps::FrameCache>(entity).needTransformUpdate) {
      return true;
    }
  }
  return false;
}

//==============================================================================
bool canSkipDefaultStepPipelineWhenFramesClean(
    const World& world,
    bool hasAdvanceableRigidBodies,
    bool hasMultibodyStructure,
    bool hasDeformableBodies)
{
  if (hasAdvanceableRigidBodies || hasMultibodyStructure
      || hasDeformableBodies) {
    return false;
  }

  const auto& registry = detail::registryOf(world);
  const auto loopClosureView = registry.view<comps::LoopClosure>();
  if (loopClosureView.begin() != loopClosureView.end()) {
    return false;
  }
  const auto jointView = registry.view<comps::JointModel>();
  if (jointView.begin() != jointView.end()) {
    return false;
  }

  return true;
}

//==============================================================================
bool hasDeformableBodies(const World& world)
{
  const auto view = detail::registryOf(world).view<comps::DeformableBodyTag>();
  return view.begin() != view.end();
}

//==============================================================================
entt::entity findOwningMultibodyStructure(
    const detail::WorldRegistry& registry, entt::entity linkEntity)
{
  if (linkEntity == entt::null
      || !registry.all_of<comps::LinkModel>(linkEntity)) {
    return entt::null;
  }

  const auto structures = registry.view<comps::MultibodyStructure>();
  for (const entt::entity structureEntity : structures) {
    const auto& structure
        = structures.get<comps::MultibodyStructure>(structureEntity);
    if (std::find(structure.links.begin(), structure.links.end(), linkEntity)
        != structure.links.end()) {
      return structureEntity;
    }
  }

  return entt::null;
}

//==============================================================================
bool isDynamicRigidDeactivationEntity(
    const detail::WorldRegistry& registry, entt::entity entity)
{
  return registry.all_of<comps::RigidBodyTag, comps::Velocity>(entity)
         && !registry.all_of<comps::StaticBodyTag>(entity)
         && !registry.all_of<comps::KinematicBodyTag>(entity);
}

//==============================================================================
entt::entity deactivationEntityForContactBody(
    const detail::WorldRegistry& registry,
    const CollisionBody& body,
    bool rigidSupported,
    bool multibodySupported)
{
  const auto entity = detail::toRegistryEntity(body.getEntity());
  if (rigidSupported && isDynamicRigidDeactivationEntity(registry, entity)) {
    return entity;
  }

  if (multibodySupported && registry.all_of<comps::LinkModel>(entity)) {
    return findOwningMultibodyStructure(registry, entity);
  }

  return entt::null;
}

//==============================================================================
bool isRigidBodyJointType(comps::JointType type)
{
  return type == comps::JointType::Fixed || type == comps::JointType::Revolute
         || type == comps::JointType::Prismatic
         || type == comps::JointType::Spherical;
}

//==============================================================================
bool rigidBodyJointUsesAxis(comps::JointType type)
{
  return type == comps::JointType::Revolute
         || type == comps::JointType::Prismatic;
}

//==============================================================================
bool articulatedPointJointUsesAxis(comps::JointType type)
{
  return type == comps::JointType::Revolute
         || type == comps::JointType::Prismatic;
}

//==============================================================================
comps::JointType toRigidBodyComponentJointType(JointType type)
{
  switch (type) {
    case JointType::Fixed:
      return comps::JointType::Fixed;
    case JointType::Revolute:
      return comps::JointType::Revolute;
    case JointType::Prismatic:
      return comps::JointType::Prismatic;
    case JointType::Spherical:
      return comps::JointType::Spherical;
    case JointType::Screw:
    case JointType::Universal:
    case JointType::Planar:
    case JointType::Floating:
    case JointType::Custom:
      break;
  }

  DART_SIMULATION_THROW_T(
      InvalidArgumentException,
      "Rigid-body joints currently support only fixed, revolute, prismatic, "
      "and spherical joint types");
  return comps::JointType::Custom;
}

//==============================================================================
comps::JointType toArticulatedPointJointComponentJointType(JointType type)
{
  switch (type) {
    case JointType::Fixed:
      return comps::JointType::Fixed;
    case JointType::Revolute:
      return comps::JointType::Revolute;
    case JointType::Prismatic:
      return comps::JointType::Prismatic;
    case JointType::Spherical:
      return comps::JointType::Spherical;
    case JointType::Screw:
    case JointType::Universal:
    case JointType::Planar:
    case JointType::Floating:
    case JointType::Custom:
      break;
  }

  DART_SIMULATION_THROW_T(
      InvalidArgumentException,
      "Articulated point joints currently support only fixed, revolute, "
      "prismatic, and spherical joint types");
  return comps::JointType::Custom;
}

//==============================================================================
template <typename Registry>
bool isRigidBodyJoint(const Registry& registry, const comps::JointModel& joint)
{
  if (!isRigidBodyJointType(joint.type) || joint.parentLink == entt::null
      || joint.childLink == entt::null || joint.parentLink == joint.childLink) {
    return false;
  }

  return registry.template all_of<comps::RigidBodyTag>(joint.parentLink)
         && registry.template all_of<comps::RigidBodyTag>(joint.childLink);
}

//==============================================================================
template <typename Registry>
bool isRigidBodyFixedJoint(
    const Registry& registry, const comps::JointModel& joint)
{
  if (joint.type != comps::JointType::Fixed) {
    return false;
  }

  return isRigidBodyJoint(registry, joint);
}

//==============================================================================
template <typename Registry>
bool isArticulatedPointJoint(
    const Registry& registry,
    entt::entity jointEntity,
    const comps::JointModel& joint)
{
  if (!isRigidBodyJointType(joint.type) || joint.childLink == entt::null
      || joint.parentLink == joint.childLink) {
    return false;
  }

  if (joint.parentLink != entt::null
      && !registry.template all_of<comps::LinkModel>(joint.parentLink)) {
    return false;
  }
  if (!registry.template all_of<comps::LinkModel>(joint.childLink)) {
    return false;
  }

  const auto* childLink
      = registry.template try_get<comps::LinkModel>(joint.childLink);
  if (childLink != nullptr && childLink->parentJoint == jointEntity) {
    return false;
  }

  const entt::entity structureB
      = findOwningMultibodyStructure(registry, joint.childLink);
  if (joint.parentLink == entt::null) {
    return structureB != entt::null;
  }

  const entt::entity structureA
      = findOwningMultibodyStructure(registry, joint.parentLink);
  return structureA != entt::null && structureA == structureB;
}

//==============================================================================
bool hasRigidBodyJoints(const World& world)
{
  const auto& registry = detail::registryOf(world);
  const auto view = registry.view<comps::JointModel>();
  for (auto entity : view) {
    (void)entity;
    const auto& joint = view.get<comps::JointModel>(entity);
    if (isRigidBodyJoint(registry, joint)) {
      return true;
    }
  }
  return false;
}

//==============================================================================
bool hasRigidBodyDistanceSprings(const World& world)
{
  const auto& registry = detail::registryOf(world);
  const auto view
      = registry
            .view<detail::deformable_vbd::AvbdRigidWorldDistanceSpringConfig>();
  return view.begin() != view.end();
}

//==============================================================================
bool hasRigidBodyAvbdPairConstraints(const World& world)
{
  return hasRigidBodyJoints(world) || hasRigidBodyDistanceSprings(world);
}

//==============================================================================
bool hasArticulatedPointJoints(const World& world)
{
  const auto& registry = detail::registryOf(world);
  const auto view = registry.view<comps::JointModel>();
  for (const entt::entity entity : view) {
    const auto& joint = view.get<comps::JointModel>(entity);
    if (isArticulatedPointJoint(registry, entity, joint)) {
      return true;
    }
  }
  return false;
}

//==============================================================================
bool hasRigidBodyJointsUnsupportedByIpc(const World& world)
{
  const auto& registry = detail::registryOf(world);
  const auto view = registry.view<comps::JointModel>();
  for (auto entity : view) {
    const auto& joint = view.get<comps::JointModel>(entity);
    if (!isRigidBodyJoint(registry, joint)) {
      continue;
    }
    if (joint.type != comps::JointType::Fixed
        && joint.type != comps::JointType::Revolute) {
      return true;
    }
    if (!std::isfinite(joint.breakForce) || joint.breakForce > 0.0) {
      return true;
    }
  }
  return false;
}

//==============================================================================
void validateRigidBodyJointPipelineSupport(
    const World& world, RigidBodySolver solver)
{
  if (!hasRigidBodyAvbdPairConstraints(world)) {
    return;
  }

  if (solver == RigidBodySolver::Ipc) {
    DART_SIMULATION_THROW_T_IF(
        hasRigidBodyDistanceSprings(world),
        InvalidOperationException,
        "Rigid-body distance springs are not supported by the IPC rigid-body "
        "solver");
    DART_SIMULATION_THROW_T_IF(
        hasRigidBodyJointsUnsupportedByIpc(world),
        InvalidOperationException,
        "Only non-breakable fixed and revolute rigid-body joints are "
        "supported by the IPC rigid-body solver");
    DART_SIMULATION_THROW_T_IF(
        hasMultibodyStructures(world),
        InvalidOperationException,
        "Rigid-body joints are not supported in worlds with multibody "
        "structures");
    return;
  }

  DART_SIMULATION_THROW_T_IF(
      hasMultibodyStructures(world),
      InvalidOperationException,
      "Rigid-body AVBD pair constraints are not supported in worlds with "
      "multibody "
      "structures");
}

//==============================================================================
void validateArticulatedPointJointPipelineSupport(
    const World& world, bool variationalSelected)
{
  if (!hasArticulatedPointJoints(world) || variationalSelected) {
    return;
  }

  DART_SIMULATION_THROW_T(
      InvalidOperationException,
      "Articulated point joints require the variational multibody "
      "integration family before stepping");
}

//==============================================================================
void validateLoopClosureKinematicsPolicySupport(const World& world)
{
  auto view = detail::registryOf(world).view<comps::LoopClosure, comps::Name>();
  for (auto entity : view) {
    const auto& closure = view.get<comps::LoopClosure>(entity);
    if (!closure.runtimePolicy.enabled
        || closure.runtimePolicy.kinematics
               == ClosureKinematicsPolicy::ResidualOnly) {
      continue;
    }

    const auto& name = view.get<comps::Name>(entity);
    DART_SIMULATION_THROW_T(
        InvalidOperationException,
        "LoopClosure '{}' requests kinematic projection, but the active "
        "pipeline does not include a loop-closure projection stage",
        name.name);
  }
}

//==============================================================================
// `variationalSelected` is passed in (an enum comparison the caller already
// has) rather than re-derived from a string here, so the per-step default path
// carries no configuration-string work. The variational integrator solves a
// supported subset of loop closures (see compute::bindVariationalLoopClosure);
// the semi-implicit pipeline has no loop-closure solving stage and rejects
// every Solve closure.
void validateLoopClosureDynamicsPolicySupport(
    const World& world, bool variationalSelected)
{
  auto view = detail::registryOf(world).view<comps::LoopClosure, comps::Name>();
  for (auto entity : view) {
    const auto& closure = view.get<comps::LoopClosure>(entity);
    if (!closure.runtimePolicy.enabled
        || closure.runtimePolicy.dynamics
               == ClosureDynamicsPolicy::ResidualOnly) {
      continue;
    }

    const auto& name = view.get<comps::Name>(entity);
    if (variationalSelected) {
      const auto binding = compute::bindVariationalLoopClosure(
          detail::registryOf(world), entity);
      DART_SIMULATION_THROW_T_IF(
          binding.status
              == compute::VariationalLoopClosureBinding::Status::Unsupported,
          InvalidOperationException,
          "LoopClosure '{}' cannot be solved by the variational integrator: {}",
          name.name,
          binding.reason);
      continue; // Supported: the variational stage will enforce it.
    }

    DART_SIMULATION_THROW_T(
        InvalidOperationException,
        "LoopClosure '{}' requests dynamic solving, but the active pipeline "
        "does not include a loop-closure solving stage",
        name.name);
  }
}

//==============================================================================
struct WorldStepPipelineStages
{
  explicit WorldStepPipelineStages(common::MemoryManager& memoryManager)
    : rigidBodyVelocity(&memoryManager),
      rigidBodyContact(8, &memoryManager),
      rigidIpcContact(compute::RigidIpcContactStageOptions{}, &memoryManager),
      multibodyVelocity(&memoryManager),
      unifiedConstraint(8, &memoryManager),
      deformableDynamics(&memoryManager),
      kinematics(&memoryManager),
      pipeline(memoryManager.getFreeAllocator())
  {
  }

  compute::RigidBodyVelocityStage rigidBodyVelocity;
  compute::RigidBodyContactStage rigidBodyContact;
  compute::RigidBodyPositionStage rigidBodyPosition;
  compute::RigidIpcContactStage rigidIpcContact;
  compute::MultibodyVelocityStage multibodyVelocity;
  compute::MultibodyPositionStage multibodyPosition;
  compute::MultibodyForwardDynamicsStage multibodyDynamics;
  compute::UnifiedConstraintStage unifiedConstraint;
  compute::MultibodyVariationalIntegrationStage multibodyVariational;
  compute::DeformableDynamicsStage deformableDynamics;
  compute::KinematicsStage kinematics;
  compute::WorldStepPipeline pipeline;

  compute::WorldStepPipeline& buildDefault(
      RigidBodySolver rigidBodySolver,
      bool variationalSelected,
      bool hasRigidBodies,
      bool hasMultibodyStructure,
      bool hasDeformableBodies)
  {
    pipeline.clear();
    appendSchedule(makeSchedule(
        rigidBodySolver,
        variationalSelected,
        hasRigidBodies,
        hasMultibodyStructure,
        hasDeformableBodies,
        /*includeKinematics=*/true));
    return pipeline;
  }

  compute::WorldStepPipeline& buildWithFinalStage(
      RigidBodySolver rigidBodySolver,
      bool variationalSelected,
      bool hasRigidBodies,
      bool hasMultibodyStructure,
      bool hasDeformableBodies,
      compute::WorldStepStage& finalStage)
  {
    pipeline.clear();
    appendSchedule(makeSchedule(
        rigidBodySolver,
        variationalSelected,
        hasRigidBodies,
        hasMultibodyStructure,
        hasDeformableBodies,
        /*includeKinematics=*/false));
    pipeline.addStage(finalStage);
    return pipeline;
  }

  void prepare(
      World& world,
      RigidBodySolver rigidBodySolver,
      bool variationalSelected,
      bool hasRigidBodies,
      bool hasMultibodyStructure,
      bool hasDeformableBodies)
  {
    const auto schedule = makeSchedule(
        rigidBodySolver,
        variationalSelected,
        hasRigidBodies,
        hasMultibodyStructure,
        hasDeformableBodies,
        /*includeKinematics=*/true);
    for (const auto slot : schedule) {
      stageForSlot(slot).prepare(world);
    }
  }

private:
  using Schedule = detail::BuiltInWorldStepSchedule;
  using Slot = detail::BuiltInWorldStepStageSlot;

  static Schedule makeSchedule(
      RigidBodySolver rigidBodySolver,
      bool variationalSelected,
      bool hasRigidBodies,
      bool hasMultibodyStructure,
      bool hasDeformableBodies,
      bool includeKinematics)
  {
    DART_SIMULATION_THROW_T_IF(
        !isValidRigidBodySolver(rigidBodySolver),
        InvalidArgumentException,
        "Rigid-body solver is invalid");

    detail::BuiltInWorldStepScheduleOptions options;
    options.rigidBodySolver = toBuiltInRigidBodySolverFamily(rigidBodySolver);
    options.multibodyIntegration
        = variationalSelected
              ? detail::BuiltInMultibodyIntegrationFamily::Variational
              : detail::BuiltInMultibodyIntegrationFamily::SemiImplicit;
    options.hasRigidBodies = hasRigidBodies;
    options.hasMultibodyStructures = hasMultibodyStructure;
    options.hasDeformableBodies = hasDeformableBodies;
    options.includeKinematics = includeKinematics;
    return detail::makeBuiltInWorldStepSchedule(options);
  }

  compute::WorldStepStage& stageForSlot(Slot slot)
  {
    switch (slot) {
      case Slot::RigidBodyVelocity:
        return rigidBodyVelocity;
      case Slot::RigidBodyContact:
        return rigidBodyContact;
      case Slot::RigidBodyPosition:
        return rigidBodyPosition;
      case Slot::RigidIpcContact:
        return rigidIpcContact;
      case Slot::MultibodyVelocity:
        return multibodyVelocity;
      case Slot::MultibodyForwardDynamics:
        return multibodyDynamics;
      case Slot::MultibodyPosition:
        return multibodyPosition;
      case Slot::MultibodyVariationalIntegration:
        return multibodyVariational;
      case Slot::UnifiedConstraint:
        return unifiedConstraint;
      case Slot::DeformableDynamics:
        return deformableDynamics;
      case Slot::Kinematics:
        return kinematics;
    }

    DART_SIMULATION_THROW_T( // LCOV_EXCL_LINE
        InvalidArgumentException, "World step stage slot is invalid");
    return kinematics;
  }

  void appendSchedule(const Schedule& schedule)
  {
    for (const auto slot : schedule) {
      pipeline.addStage(stageForSlot(slot));
    }
  }
};

//==============================================================================
bool isSymmetricPositiveDefinite(const Eigen::Matrix3d& matrix)
{
  if (!matrix.allFinite() || !matrix.isApprox(matrix.transpose(), 1e-12)) {
    return false;
  }

  Eigen::LLT<Eigen::Matrix3d> factorization(matrix);
  return factorization.info() == Eigen::Success;
}

//==============================================================================
void validateFiniteVector(
    const Eigen::Vector3d& value, std::string_view fieldName)
{
  DART_SIMULATION_THROW_T_IF(
      !value.allFinite(),
      InvalidArgumentException,
      "RigidBodyOptions.{} must contain only finite values",
      fieldName);
}

//==============================================================================
bool isValidLoopClosureFamily(LoopClosureFamily family)
{
  switch (family) {
    case LoopClosureFamily::Rigid:
    case LoopClosureFamily::Point:
    case LoopClosureFamily::Distance:
      return true;
  }

  return false;
}

//==============================================================================
void validateLoopClosureOffset(
    const Eigen::Isometry3d& offset, std::string_view fieldName)
{
  constexpr double tolerance = 1e-9;

  DART_SIMULATION_THROW_T_IF(
      !offset.matrix().allFinite(),
      InvalidArgumentException,
      "LoopClosureSpec.{} must contain only finite values",
      fieldName);

  const auto& rotation = offset.linear();
  const double orthonormalError
      = (rotation * rotation.transpose() - Eigen::Matrix3d::Identity())
            .cwiseAbs()
            .maxCoeff();
  DART_SIMULATION_THROW_T_IF(
      orthonormalError > tolerance
          || std::abs(rotation.determinant() - 1.0) > tolerance,
      InvalidArgumentException,
      "LoopClosureSpec.{} rotation must be orthonormal",
      fieldName);
}

//==============================================================================
entt::entity resolveLoopClosureFrame(
    const World& world, const Frame& frame, std::string_view fieldName)
{
  if (frame.isWorld()) {
    return entt::null;
  }

  DART_SIMULATION_THROW_T_IF(
      !frame.isValid(),
      InvalidArgumentException,
      "LoopClosureSpec.{} is invalid or has been destroyed",
      fieldName);

  DART_SIMULATION_THROW_T_IF(
      frame.getWorld() != &world,
      InvalidArgumentException,
      "LoopClosureSpec.{} belongs to a different world",
      fieldName);

  return detail::toRegistryEntity(frame.getEntity());
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
template <typename Registry, typename PairVector>
void collectLivePublicRigidBodyJointPairsInto(
    const Registry& registry, PairVector& pairs)
{
  pairs.clear();

  const auto joints = registry.template view<comps::JointModel>();
  for (const entt::entity entity : joints) {
    const auto& joint = joints.template get<comps::JointModel>(entity);
    const auto& jointState = registry.template get<comps::JointState>(entity);
    if (jointState.broken || !isRigidBodyJoint(registry, joint)) {
      continue;
    }

    pairs.push_back(makeCollisionPairKey(joint.parentLink, joint.childLink));
  }

  std::sort(pairs.begin(), pairs.end());
  pairs.erase(std::unique(pairs.begin(), pairs.end()), pairs.end());
}

//==============================================================================
entt::entity resolveCollisionPairFrame(
    const World& world, const Frame& frame, std::string_view fieldName)
{
  DART_SIMULATION_THROW_T_IF(
      frame.isWorld() || !frame.isValid(),
      InvalidArgumentException,
      "Collision pair {} frame is invalid or has been destroyed",
      fieldName);

  DART_SIMULATION_THROW_T_IF(
      frame.getWorld() != &world,
      InvalidArgumentException,
      "Collision pair {} frame belongs to a different world",
      fieldName);

  const entt::entity entity = detail::toRegistryEntity(frame.getEntity());
  const auto& registry = detail::registryOf(world);
  DART_SIMULATION_THROW_T_IF(
      !registry.all_of<comps::RigidBodyTag>(entity)
          && !registry.all_of<comps::LinkModel>(entity),
      InvalidArgumentException,
      "Collision pair {} frame must be a rigid body or multibody link",
      fieldName);

  return entity;
}

//==============================================================================
void validateLoopClosureSpec(const World& world, const LoopClosureSpec& spec)
{
  DART_SIMULATION_THROW_T_IF(
      !isValidLoopClosureFamily(spec.family),
      InvalidArgumentException,
      "LoopClosureSpec.family is invalid");

  const auto frameA = resolveLoopClosureFrame(world, spec.frameA, "frameA");
  const auto frameB = resolveLoopClosureFrame(world, spec.frameB, "frameB");
  DART_SIMULATION_THROW_T_IF(
      frameA == frameB,
      InvalidArgumentException,
      "LoopClosureSpec endpoints must be distinct frames");

  validateLoopClosureOffset(spec.offsetA, "offsetA");
  validateLoopClosureOffset(spec.offsetB, "offsetB");
}

//==============================================================================
void validateRigidBodyOptions(const RigidBodyOptions& options)
{
  DART_SIMULATION_THROW_T_IF(
      !std::isfinite(options.mass) || options.mass <= 0.0,
      InvalidArgumentException,
      "RigidBodyOptions.mass must be positive and finite");

  DART_SIMULATION_THROW_T_IF(
      !isSymmetricPositiveDefinite(options.inertia),
      InvalidArgumentException,
      "RigidBodyOptions.inertia must be symmetric positive definite");

  validateFiniteVector(options.position, "position");
  validateFiniteVector(options.linearVelocity, "linearVelocity");
  validateFiniteVector(options.angularVelocity, "angularVelocity");

  const auto orientationNorm = options.orientation.norm();
  DART_SIMULATION_THROW_T_IF(
      !options.orientation.coeffs().allFinite()
          || !std::isfinite(orientationNorm) || orientationNorm <= 0.0,
      InvalidArgumentException,
      "RigidBodyOptions.orientation must be finite and non-zero");
}

//==============================================================================
void validateDeformableFiniteVector(
    const Eigen::Vector3d& value, std::string_view fieldName, std::size_t index)
{
  DART_SIMULATION_THROW_T_IF(
      !value.allFinite(),
      InvalidArgumentException,
      "DeformableBodyOptions.{}[{}] must contain only finite values",
      fieldName,
      index);
}

//==============================================================================
std::array<std::size_t, 3> sortedFaceKey(
    std::size_t nodeA, std::size_t nodeB, std::size_t nodeC)
{
  std::array<std::size_t, 3> key{nodeA, nodeB, nodeC};
  std::ranges::sort(key);
  return key;
}

//==============================================================================
std::array<std::size_t, 4> sortedTetrahedronKey(
    std::size_t nodeA, std::size_t nodeB, std::size_t nodeC, std::size_t nodeD)
{
  std::array<std::size_t, 4> key{nodeA, nodeB, nodeC, nodeD};
  std::ranges::sort(key);
  return key;
}

//==============================================================================
bool hasRepeatedNodes(std::span<const std::size_t> nodes)
{
  for (std::size_t i = 0; i < nodes.size(); ++i) {
    for (std::size_t j = i + 1; j < nodes.size(); ++j) {
      if (nodes[i] == nodes[j]) {
        return true;
      }
    }
  }
  return false;
}

//==============================================================================
double signedTetrahedronVolume(
    std::span<const Eigen::Vector3d> positions,
    const comps::DeformableTetrahedron& tetrahedron)
{
  const auto& a = positions[tetrahedron.nodeA];
  const auto& b = positions[tetrahedron.nodeB];
  const auto& c = positions[tetrahedron.nodeC];
  const auto& d = positions[tetrahedron.nodeD];
  return (b - a).cross(c - a).dot(d - a) / 6.0;
}

//==============================================================================
double surfaceTriangleAreaSquared(
    std::span<const Eigen::Vector3d> positions,
    const comps::DeformableSurfaceTriangle& triangle)
{
  const auto& a = positions[triangle.nodeA];
  const auto& b = positions[triangle.nodeB];
  const auto& c = positions[triangle.nodeC];
  return 0.25 * (b - a).cross(c - a).squaredNorm();
}

//==============================================================================
void validateDeformableMaterial(const DeformableMaterialProperties& material)
{
  DART_SIMULATION_THROW_T_IF(
      !std::isfinite(material.density) || material.density <= 0.0,
      InvalidArgumentException,
      "DeformableBodyOptions.material.density must be positive and finite");
  DART_SIMULATION_THROW_T_IF(
      !std::isfinite(material.youngsModulus) || material.youngsModulus <= 0.0,
      InvalidArgumentException,
      "DeformableBodyOptions.material.youngsModulus must be positive and "
      "finite");
  DART_SIMULATION_THROW_T_IF(
      !std::isfinite(material.poissonRatio) || material.poissonRatio <= -1.0
          || material.poissonRatio >= 0.5,
      InvalidArgumentException,
      "DeformableBodyOptions.material.poissonRatio must be finite and in "
      "(-1, 0.5)");
  DART_SIMULATION_THROW_T_IF(
      !std::isfinite(material.frictionCoefficient)
          || material.frictionCoefficient < 0.0,
      InvalidArgumentException,
      "DeformableBodyOptions.material.frictionCoefficient must be finite and "
      "non-negative");
}

//==============================================================================
struct PreparedDeformableBodyData
{
  using Vector3Vector = comps::DeformableNodeState::Vector3Vector;
  using ScalarVector = comps::DeformableNodeModel::ScalarVector;
  using MaskVector = comps::DeformableNodeModel::MaskVector;
  using EdgeVector = comps::DeformableSpringModel::EdgeVector;
  using SurfaceTriangleVector
      = comps::DeformableMeshTopology::SurfaceTriangleVector;
  using TetrahedronVector = comps::DeformableMeshTopology::TetrahedronVector;

  explicit PreparedDeformableBodyData(common::MemoryAllocator& allocator)
    : positions(common::StlAllocator<Eigen::Vector3d>{allocator}),
      restPositions(common::StlAllocator<Eigen::Vector3d>{allocator}),
      velocities(common::StlAllocator<Eigen::Vector3d>{allocator}),
      masses(common::StlAllocator<double>{allocator}),
      fixed(common::StlAllocator<std::uint8_t>{allocator}),
      edges(common::StlAllocator<comps::DeformableSpringEdge>{allocator}),
      surfaceTriangles(
          common::StlAllocator<comps::DeformableSurfaceTriangle>{allocator}),
      tetrahedra(common::StlAllocator<comps::DeformableTetrahedron>{allocator}),
      tetrahedronRestVolumes(common::StlAllocator<double>{allocator}),
      boundaryConditions(allocator)
  {
  }

  Vector3Vector positions;
  Vector3Vector restPositions;
  Vector3Vector velocities;
  ScalarVector masses;
  MaskVector fixed;
  EdgeVector edges;
  SurfaceTriangleVector surfaceTriangles;
  TetrahedronVector tetrahedra;
  ScalarVector tetrahedronRestVolumes;
  comps::DeformableMaterial material;
  comps::DeformableBoundaryConditions boundaryConditions;
  double stiffness = 0.0;
  double damping = 0.0;
};

using DeformableSurfaceFaceKey = std::array<std::size_t, 3>;
using DeformableTetrahedronKey = std::array<std::size_t, 4>;

template <typename Key>
using DeformableValidationSet
    = std::set<Key, std::less<Key>, common::StlAllocator<Key>>;

using DeformableSurfaceFaceValue
    = std::pair<comps::DeformableSurfaceTriangle, std::size_t>;
using DeformableSurfaceFaceEntry
    = std::pair<const DeformableSurfaceFaceKey, DeformableSurfaceFaceValue>;
using DeformableSurfaceFaceMap = std::map<
    DeformableSurfaceFaceKey,
    DeformableSurfaceFaceValue,
    std::less<DeformableSurfaceFaceKey>,
    common::StlAllocator<DeformableSurfaceFaceEntry>>;

//==============================================================================
bool hasValidBoundaryEndTime(double value)
{
  return std::isfinite(value)
         || value == std::numeric_limits<double>::infinity();
}

//==============================================================================
std::size_t validateFrameScratchInitialCapacity(std::size_t capacity)
{
  DART_SIMULATION_THROW_T_IF(
      capacity == 0,
      InvalidArgumentException,
      "WorldOptions.frameScratchInitialCapacity must be positive");

  return capacity;
}

//==============================================================================
common::MemoryAllocator& resolveBaseAllocator(const WorldOptions& options)
{
  return options.baseAllocator ? *options.baseAllocator
                               : common::MemoryAllocator::GetDefault();
}

//==============================================================================
common::MemoryManager::Options makeMemoryManagerOptions(
    const WorldOptions& options)
{
  common::MemoryManager::Options memoryOptions;
  memoryOptions.frameAllocatorInitialCapacity
      = validateFrameScratchInitialCapacity(
          options.frameScratchInitialCapacity);
  memoryOptions.freeListInitialAllocation = options.freeListInitialAllocation;
  memoryOptions.freeListGrowthPolicy = options.freeListGrowthPolicy;
  return memoryOptions;
}

//==============================================================================
void reserveExistingRegistryStorages(detail::WorldRegistry& registry)
{
  auto& entities = registry.storage<entt::entity>();
  entities.reserve(entities.size());

  for (auto&& [id, storage] : registry.storage()) {
    (void)id;
    storage.reserve(storage.size());
  }
}

//==============================================================================
template <typename Component, typename EntityRange, typename... Args>
void reserveAndPrimeComponentStorage(
    detail::WorldRegistry& registry,
    EntityRange&& entities,
    std::size_t capacity,
    Args&&... args)
{
  auto& storage = registry.template storage<Component>();
  storage.reserve(capacity);

  for (const auto entity : entities) {
    if (registry.template all_of<Component>(entity)) {
      continue;
    }

    registry.template emplace<Component>(entity, std::forward<Args>(args)...);
    registry.template remove<Component>(entity);
  }
}

//==============================================================================
template <typename Component, typename EntityRange>
void reserveAndPrimeDefaultComponentStorage(
    detail::WorldRegistry& registry,
    EntityRange&& entities,
    std::size_t capacity)
{
  reserveAndPrimeComponentStorage<Component>(
      registry, std::forward<EntityRange>(entities), capacity);
}

//==============================================================================
template <typename Component>
std::size_t existingComponentStorageSize(
    const detail::WorldRegistry& registry) noexcept
{
  const auto* storage = registry.template storage<Component>();
  return storage == nullptr ? 0u : storage->size();
}

//==============================================================================
template <typename... Components>
void ensureRegistryStorages(detail::WorldRegistry& registry)
{
  (static_cast<void>(registry.template storage<Components>()), ...);
}

//==============================================================================
bool boundaryRangesOverlap(
    double startA, double endA, double startB, double endB)
{
  return startA < endB && startB < endA;
}

//==============================================================================
void validateBoundaryTimeRange(
    double startTime, double endTime, std::string_view fieldName)
{
  DART_SIMULATION_THROW_T_IF(
      !std::isfinite(startTime) || startTime < 0.0,
      InvalidArgumentException,
      "DeformableBodyOptions.{} start time must be finite and non-negative",
      fieldName);
  DART_SIMULATION_THROW_T_IF(
      !hasValidBoundaryEndTime(endTime) || endTime < startTime,
      InvalidArgumentException,
      "DeformableBodyOptions.{} end time must be finite or infinity and must "
      "not precede the start time",
      fieldName);
}

//==============================================================================
void validateBoundaryNodes(
    std::span<const std::size_t> nodes,
    std::size_t nodeCount,
    std::string_view fieldName,
    std::size_t boundaryIndex,
    common::MemoryAllocator& allocator)
{
  DART_SIMULATION_THROW_T_IF(
      nodes.empty(),
      InvalidArgumentException,
      "DeformableBodyOptions.{}[{}] must reference at least one node",
      fieldName,
      boundaryIndex);
  DeformableValidationSet<std::size_t> uniqueNodes{
      std::less<std::size_t>{}, common::StlAllocator<std::size_t>{allocator}};
  for (const auto node : nodes) {
    DART_SIMULATION_THROW_T_IF(
        node >= nodeCount,
        InvalidArgumentException,
        "DeformableBodyOptions.{}[{}] references an out-of-range node",
        fieldName,
        boundaryIndex);
    DART_SIMULATION_THROW_T_IF(
        !uniqueNodes.insert(node).second,
        InvalidArgumentException,
        "DeformableBodyOptions.{}[{}] contains duplicate node {}",
        fieldName,
        boundaryIndex,
        node);
  }
}

//==============================================================================
void validateNoBoundaryConflicts(
    const DeformableBodyOptions& options, std::span<const std::uint8_t> fixed)
{
  for (std::size_t dirichletIndex = 0;
       dirichletIndex < options.dirichletBoundaryConditions.size();
       ++dirichletIndex) {
    const auto& dirichlet = options.dirichletBoundaryConditions[dirichletIndex];
    for (const auto node : dirichlet.nodes) {
      DART_SIMULATION_THROW_T_IF(
          fixed[node] != 0u,
          InvalidArgumentException,
          "DeformableBodyOptions.dirichletBoundaryConditions[{}] overlaps "
          "permanently fixed node {}",
          dirichletIndex,
          node);
    }

    for (std::size_t otherIndex = dirichletIndex + 1;
         otherIndex < options.dirichletBoundaryConditions.size();
         ++otherIndex) {
      const auto& other = options.dirichletBoundaryConditions[otherIndex];
      if (!boundaryRangesOverlap(
              dirichlet.startTime,
              dirichlet.endTime,
              other.startTime,
              other.endTime)) {
        continue;
      }
      for (const auto node : dirichlet.nodes) {
        DART_SIMULATION_THROW_T_IF(
            std::ranges::find(other.nodes, node) != other.nodes.end(),
            InvalidArgumentException,
            "DeformableBodyOptions.dirichletBoundaryConditions overlap on "
            "node {}",
            node);
      }
    }
  }

  for (std::size_t neumannIndex = 0;
       neumannIndex < options.neumannBoundaryConditions.size();
       ++neumannIndex) {
    const auto& neumann = options.neumannBoundaryConditions[neumannIndex];
    for (const auto node : neumann.nodes) {
      DART_SIMULATION_THROW_T_IF(
          fixed[node] != 0u,
          InvalidArgumentException,
          "DeformableBodyOptions.neumannBoundaryConditions[{}] overlaps "
          "permanently fixed node {}",
          neumannIndex,
          node);
    }

    for (const auto& dirichlet : options.dirichletBoundaryConditions) {
      if (!boundaryRangesOverlap(
              neumann.startTime,
              neumann.endTime,
              dirichlet.startTime,
              dirichlet.endTime)) {
        continue;
      }
      for (const auto node : neumann.nodes) {
        DART_SIMULATION_THROW_T_IF(
            std::ranges::find(dirichlet.nodes, node) != dirichlet.nodes.end(),
            InvalidArgumentException,
            "DeformableBodyOptions.neumannBoundaryConditions overlap active "
            "Dirichlet boundary node {}",
            node);
      }
    }
  }
}

//==============================================================================
comps::DeformableMeshTopology::SurfaceTriangleVector
validateDeformableSurfaceTriangles(
    const DeformableBodyOptions& options, common::MemoryAllocator& allocator)
{
  comps::DeformableMeshTopology::SurfaceTriangleVector surfaceTriangles(
      common::StlAllocator<comps::DeformableSurfaceTriangle>{allocator});
  surfaceTriangles.reserve(options.surfaceTriangles.size());

  DeformableValidationSet<DeformableSurfaceFaceKey> uniqueFaces{
      std::less<DeformableSurfaceFaceKey>{},
      common::StlAllocator<DeformableSurfaceFaceKey>{allocator}};
  for (std::size_t i = 0; i < options.surfaceTriangles.size(); ++i) {
    const auto& triangle = options.surfaceTriangles[i];
    const std::array<std::size_t, 3> nodes{
        triangle.nodeA, triangle.nodeB, triangle.nodeC};
    DART_SIMULATION_THROW_T_IF(
        hasRepeatedNodes(nodes),
        InvalidArgumentException,
        "DeformableBodyOptions.surfaceTriangles[{}] nodes must be distinct",
        i);
    for (const auto node : nodes) {
      DART_SIMULATION_THROW_T_IF(
          node >= options.positions.size(),
          InvalidArgumentException,
          "DeformableBodyOptions.surfaceTriangles[{}] references an "
          "out-of-range node",
          i);
    }

    comps::DeformableSurfaceTriangle internal{
        triangle.nodeA, triangle.nodeB, triangle.nodeC};
    DART_SIMULATION_THROW_T_IF(
        surfaceTriangleAreaSquared(options.positions, internal) <= 1e-24,
        InvalidArgumentException,
        "DeformableBodyOptions.surfaceTriangles[{}] is degenerate",
        i);

    const auto key
        = sortedFaceKey(internal.nodeA, internal.nodeB, internal.nodeC);
    DART_SIMULATION_THROW_T_IF(
        !uniqueFaces.insert(key).second,
        InvalidArgumentException,
        "DeformableBodyOptions.surfaceTriangles[{}] duplicates an existing "
        "face",
        i);
    surfaceTriangles.push_back(internal);
  }

  return surfaceTriangles;
}

//==============================================================================
void addBoundaryFace(
    DeformableSurfaceFaceMap& faces, comps::DeformableSurfaceTriangle face)
{
  const auto key = sortedFaceKey(face.nodeA, face.nodeB, face.nodeC);
  auto [it, inserted] = faces.emplace(key, std::pair{face, 0u});
  ++it->second.second;
  DART_SIMULATION_THROW_T_IF(
      it->second.second > 2u,
      InvalidArgumentException,
      "DeformableBodyOptions.tetrahedra creates a nonmanifold surface face");
  if (!inserted && it->second.second == 2u) {
    it->second.first = face;
  }
}

//==============================================================================
comps::DeformableMeshTopology::SurfaceTriangleVector
deriveDeformableBoundarySurface(
    std::span<const comps::DeformableTetrahedron> tetrahedra,
    common::MemoryAllocator& allocator)
{
  DeformableSurfaceFaceMap faces{
      std::less<DeformableSurfaceFaceKey>{},
      DeformableSurfaceFaceMap::allocator_type{allocator}};
  for (const auto& tet : tetrahedra) {
    addBoundaryFace(faces, {tet.nodeA, tet.nodeC, tet.nodeB});
    addBoundaryFace(faces, {tet.nodeA, tet.nodeB, tet.nodeD});
    addBoundaryFace(faces, {tet.nodeA, tet.nodeD, tet.nodeC});
    addBoundaryFace(faces, {tet.nodeB, tet.nodeC, tet.nodeD});
  }

  comps::DeformableMeshTopology::SurfaceTriangleVector surfaceTriangles(
      common::StlAllocator<comps::DeformableSurfaceTriangle>{allocator});
  for (const auto& [_, faceAndCount] : faces) {
    if (faceAndCount.second == 1u) {
      surfaceTriangles.push_back(faceAndCount.first);
    }
  }
  return surfaceTriangles;
}

//==============================================================================
PreparedDeformableBodyData prepareDeformableBodyOptions(
    const DeformableBodyOptions& options, common::MemoryAllocator& allocator)
{
  const auto nodeCount = options.positions.size();
  DART_SIMULATION_THROW_T_IF(
      nodeCount == 0,
      InvalidArgumentException,
      "DeformableBodyOptions.positions must not be empty");

  PreparedDeformableBodyData data(allocator);
  data.positions.assign(options.positions.begin(), options.positions.end());
  data.restPositions = data.positions;
  data.velocities.assign(nodeCount, Eigen::Vector3d::Zero());
  data.masses.assign(nodeCount, 1.0);
  data.fixed.assign(nodeCount, 0u);
  data.stiffness = options.edgeStiffness;
  data.damping = options.damping;
  data.material.density = options.material.density;
  data.material.youngsModulus = options.material.youngsModulus;
  data.material.poissonRatio = options.material.poissonRatio;
  data.material.frictionCoefficient = options.material.frictionCoefficient;
  data.material.useFiniteElementElasticity
      = options.material.useFiniteElementElasticity;
  data.material.useFixedCorotationalElasticity
      = options.material.useFixedCorotationalElasticity;
  data.material.useAdaptiveBarrierStiffness
      = options.material.useAdaptiveBarrierStiffness;
  data.material.useIterativeLinearSolver
      = options.material.useIterativeLinearSolver;
  data.material.useMatrixFreeLinearSolver
      = options.material.useMatrixFreeLinearSolver;

  for (std::size_t i = 0; i < nodeCount; ++i) {
    validateDeformableFiniteVector(options.positions[i], "positions", i);
  }

  validateDeformableMaterial(options.material);

  DeformableValidationSet<DeformableTetrahedronKey> uniqueTetrahedra{
      std::less<DeformableTetrahedronKey>{},
      common::StlAllocator<DeformableTetrahedronKey>{allocator}};
  data.tetrahedra.reserve(options.tetrahedra.size());
  data.tetrahedronRestVolumes.reserve(options.tetrahedra.size());
  for (std::size_t i = 0; i < options.tetrahedra.size(); ++i) {
    const auto& tetrahedron = options.tetrahedra[i];
    const std::array<std::size_t, 4> nodes{
        tetrahedron.nodeA,
        tetrahedron.nodeB,
        tetrahedron.nodeC,
        tetrahedron.nodeD};
    DART_SIMULATION_THROW_T_IF(
        hasRepeatedNodes(nodes),
        InvalidArgumentException,
        "DeformableBodyOptions.tetrahedra[{}] nodes must be distinct",
        i);
    for (const auto node : nodes) {
      DART_SIMULATION_THROW_T_IF(
          node >= nodeCount,
          InvalidArgumentException,
          "DeformableBodyOptions.tetrahedra[{}] references an out-of-range "
          "node",
          i);
    }

    const auto key = sortedTetrahedronKey(
        tetrahedron.nodeA,
        tetrahedron.nodeB,
        tetrahedron.nodeC,
        tetrahedron.nodeD);
    DART_SIMULATION_THROW_T_IF(
        !uniqueTetrahedra.insert(key).second,
        InvalidArgumentException,
        "DeformableBodyOptions.tetrahedra[{}] duplicates an existing "
        "tetrahedron",
        i);

    comps::DeformableTetrahedron internal{
        tetrahedron.nodeA,
        tetrahedron.nodeB,
        tetrahedron.nodeC,
        tetrahedron.nodeD};
    double volume = signedTetrahedronVolume(options.positions, internal);
    DART_SIMULATION_THROW_T_IF(
        !std::isfinite(volume) || std::abs(volume) <= 1e-18,
        InvalidArgumentException,
        "DeformableBodyOptions.tetrahedra[{}] has zero or nonfinite rest "
        "volume",
        i);
    if (volume < 0.0) {
      std::swap(internal.nodeC, internal.nodeD);
      volume = -volume;
    }

    data.tetrahedra.push_back(internal);
    data.tetrahedronRestVolumes.push_back(volume);
  }

  data.surfaceTriangles
      = validateDeformableSurfaceTriangles(options, allocator);
  if (data.surfaceTriangles.empty() && !data.tetrahedra.empty()) {
    data.surfaceTriangles
        = deriveDeformableBoundarySurface(data.tetrahedra, allocator);
  }

  DART_SIMULATION_THROW_T_IF(
      options.masses.empty() && options.tetrahedra.empty()
          && !options.surfaceTriangles.empty(),
      InvalidArgumentException,
      "DeformableBodyOptions.surfaceTriangles require explicit masses when no "
      "tetrahedra are provided");

  DART_SIMULATION_THROW_T_IF(
      !options.velocities.empty() && options.velocities.size() != nodeCount,
      InvalidArgumentException,
      "DeformableBodyOptions.velocities must be empty or match positions");
  for (std::size_t i = 0; i < options.velocities.size(); ++i) {
    validateDeformableFiniteVector(options.velocities[i], "velocities", i);
    data.velocities[i] = options.velocities[i];
  }

  DART_SIMULATION_THROW_T_IF(
      !options.masses.empty() && options.masses.size() != nodeCount,
      InvalidArgumentException,
      "DeformableBodyOptions.masses must be empty or match positions");
  if (!options.masses.empty()) {
    for (std::size_t i = 0; i < options.masses.size(); ++i) {
      const double mass = options.masses[i];
      DART_SIMULATION_THROW_T_IF(
          !std::isfinite(mass) || mass <= 0.0,
          InvalidArgumentException,
          "DeformableBodyOptions.masses[{}] must be positive and finite",
          i);
      data.masses[i] = mass;
    }
  } else if (!data.tetrahedra.empty()) {
    data.masses.assign(nodeCount, 0.0);
    for (std::size_t i = 0; i < data.tetrahedra.size(); ++i) {
      const auto& tetrahedron = data.tetrahedra[i];
      const double nodeMass
          = options.material.density * data.tetrahedronRestVolumes[i] / 4.0;
      data.masses[tetrahedron.nodeA] += nodeMass;
      data.masses[tetrahedron.nodeB] += nodeMass;
      data.masses[tetrahedron.nodeC] += nodeMass;
      data.masses[tetrahedron.nodeD] += nodeMass;
    }

    for (std::size_t i = 0; i < data.masses.size(); ++i) {
      DART_SIMULATION_THROW_T_IF(
          !std::isfinite(data.masses[i]) || data.masses[i] <= 0.0,
          InvalidArgumentException,
          "DeformableBodyOptions.tetrahedra leave node {} without positive "
          "finite assembled mass",
          i);
    }
  }

  for (const auto fixedNode : options.fixedNodes) {
    DART_SIMULATION_THROW_T_IF(
        fixedNode >= nodeCount,
        InvalidArgumentException,
        "DeformableBodyOptions.fixedNodes contains out-of-range node {}",
        fixedNode);
    DART_SIMULATION_THROW_T_IF(
        data.fixed[fixedNode] != 0u,
        InvalidArgumentException,
        "DeformableBodyOptions.fixedNodes contains duplicate node {}",
        fixedNode);
    data.fixed[fixedNode] = 1u;
  }

  data.boundaryConditions.dirichlet.reserve(
      options.dirichletBoundaryConditions.size());
  for (std::size_t i = 0; i < options.dirichletBoundaryConditions.size(); ++i) {
    const auto& boundary = options.dirichletBoundaryConditions[i];
    validateBoundaryNodes(
        boundary.nodes, nodeCount, "dirichletBoundaryConditions", i, allocator);
    validateDeformableFiniteVector(
        boundary.linearVelocity,
        "dirichletBoundaryConditions.linearVelocity",
        i);
    validateDeformableFiniteVector(
        boundary.angularVelocity,
        "dirichletBoundaryConditions.angularVelocity",
        i);
    validateDeformableFiniteVector(
        boundary.center, "dirichletBoundaryConditions.center", i);
    validateBoundaryTimeRange(
        boundary.startTime, boundary.endTime, "dirichletBoundaryConditions");

    comps::DeformableDirichletBoundary internal(allocator);
    internal.nodes.assign(boundary.nodes.begin(), boundary.nodes.end());
    internal.referencePositions.reserve(boundary.nodes.size());
    for (const auto node : boundary.nodes) {
      internal.referencePositions.push_back(data.restPositions[node]);
    }
    internal.center = boundary.center;
    internal.linearVelocity = boundary.linearVelocity;
    internal.angularVelocity = boundary.angularVelocity;
    internal.startTime = boundary.startTime;
    internal.endTime = boundary.endTime;
    data.boundaryConditions.dirichlet.push_back(std::move(internal));
  }

  data.boundaryConditions.neumann.reserve(
      options.neumannBoundaryConditions.size());
  for (std::size_t i = 0; i < options.neumannBoundaryConditions.size(); ++i) {
    const auto& boundary = options.neumannBoundaryConditions[i];
    validateBoundaryNodes(
        boundary.nodes, nodeCount, "neumannBoundaryConditions", i, allocator);
    validateDeformableFiniteVector(
        boundary.acceleration, "neumannBoundaryConditions.acceleration", i);
    validateBoundaryTimeRange(
        boundary.startTime, boundary.endTime, "neumannBoundaryConditions");

    comps::DeformableNeumannBoundary internal(allocator);
    internal.nodes.assign(boundary.nodes.begin(), boundary.nodes.end());
    internal.acceleration = boundary.acceleration;
    internal.startTime = boundary.startTime;
    internal.endTime = boundary.endTime;
    data.boundaryConditions.neumann.push_back(std::move(internal));
  }

  validateNoBoundaryConflicts(options, data.fixed);

  DART_SIMULATION_THROW_T_IF(
      !std::isfinite(options.edgeStiffness) || options.edgeStiffness < 0.0,
      InvalidArgumentException,
      "DeformableBodyOptions.edgeStiffness must be finite and non-negative");
  DART_SIMULATION_THROW_T_IF(
      !std::isfinite(options.damping) || options.damping < 0.0,
      InvalidArgumentException,
      "DeformableBodyOptions.damping must be finite and non-negative");

  data.edges.reserve(options.edges.size());
  for (std::size_t i = 0; i < options.edges.size(); ++i) {
    const auto& edge = options.edges[i];
    DART_SIMULATION_THROW_T_IF(
        edge.nodeA >= nodeCount || edge.nodeB >= nodeCount,
        InvalidArgumentException,
        "DeformableBodyOptions.edges[{}] references an out-of-range node",
        i);
    DART_SIMULATION_THROW_T_IF(
        edge.nodeA == edge.nodeB,
        InvalidArgumentException,
        "DeformableBodyOptions.edges[{}] endpoints must be distinct",
        i);

    double restLength = edge.restLength;
    if (restLength <= 0.0) {
      restLength
          = (options.positions[edge.nodeB] - options.positions[edge.nodeA])
                .norm();
    }
    DART_SIMULATION_THROW_T_IF(
        !std::isfinite(restLength) || restLength <= 0.0,
        InvalidArgumentException,
        "DeformableBodyOptions.edges[{}].restLength must be positive and "
        "finite",
        i);

    data.edges.push_back(
        comps::DeformableSpringEdge{edge.nodeA, edge.nodeB, restLength});
  }

  return data;
}

//==============================================================================
Eigen::Quaterniond normalizeOrIdentity(const Eigen::Quaterniond& orientation)
{
  const auto norm = orientation.norm();
  if (norm <= 0.0 || !std::isfinite(norm)) {
    return Eigen::Quaterniond::Identity();
  }

  auto normalized = orientation;
  normalized.coeffs() /= norm;
  return normalized;
}

//==============================================================================
Eigen::Isometry3d toIsometry(
    const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = position;
  transform.linear() = orientation.toRotationMatrix();
  return transform;
}

template <typename Vector>
void rebindLoadedVectorAllocator(
    Vector& vector, common::MemoryAllocator& allocator)
{
  using Value = typename Vector::value_type;
  common::StlAllocator<Value> targetAllocator{allocator};
  if (vector.get_allocator() == targetAllocator) {
    return;
  }

  Vector rebound{targetAllocator};
  rebound.reserve(vector.size());
  for (auto& value : vector) {
    rebound.emplace_back(std::move(value));
  }
  vector = std::move(rebound);
}

void rebindLoadedBoundaryAllocator(
    comps::DeformableDirichletBoundary& boundary,
    common::MemoryAllocator& allocator)
{
  rebindLoadedVectorAllocator(boundary.nodes, allocator);
  rebindLoadedVectorAllocator(boundary.referencePositions, allocator);
}

void rebindLoadedBoundaryAllocator(
    comps::DeformableNeumannBoundary& boundary,
    common::MemoryAllocator& allocator)
{
  rebindLoadedVectorAllocator(boundary.nodes, allocator);
}

void rebindLoadedWorldComponentAllocators(
    detail::WorldRegistry& registry, common::MemoryAllocator& allocator)
{
  auto multibodyView = registry.view<comps::MultibodyStructure>();
  for (const auto entity : multibodyView) {
    auto& structure = multibodyView.get<comps::MultibodyStructure>(entity);
    rebindLoadedVectorAllocator(structure.links, allocator);
    rebindLoadedVectorAllocator(structure.joints, allocator);
  }

  auto linkView = registry.view<comps::LinkModel>();
  for (const auto entity : linkView) {
    auto& link = linkView.get<comps::LinkModel>(entity);
    rebindLoadedVectorAllocator(link.childJoints, allocator);
  }

  auto deformableNodeView = registry.view<comps::DeformableNodeState>();
  for (const auto entity : deformableNodeView) {
    auto& state = deformableNodeView.get<comps::DeformableNodeState>(entity);
    auto& nodeModel = registry.get<comps::DeformableNodeModel>(entity);
    rebindLoadedVectorAllocator(state.positions, allocator);
    rebindLoadedVectorAllocator(state.previousPositions, allocator);
    rebindLoadedVectorAllocator(state.velocities, allocator);
    rebindLoadedVectorAllocator(nodeModel.masses, allocator);
    rebindLoadedVectorAllocator(nodeModel.fixed, allocator);
  }

  auto springView = registry.view<comps::DeformableSpringModel>();
  for (const auto entity : springView) {
    auto& model = springView.get<comps::DeformableSpringModel>(entity);
    rebindLoadedVectorAllocator(model.edges, allocator);
  }

  auto topologyView = registry.view<comps::DeformableMeshTopology>();
  for (const auto entity : topologyView) {
    auto& topology = topologyView.get<comps::DeformableMeshTopology>(entity);
    rebindLoadedVectorAllocator(topology.restPositions, allocator);
    rebindLoadedVectorAllocator(topology.surfaceTriangles, allocator);
    rebindLoadedVectorAllocator(topology.tetrahedra, allocator);
    rebindLoadedVectorAllocator(topology.tetrahedronRestVolumes, allocator);
  }

  auto boundaryView = registry.view<comps::DeformableBoundaryConditions>();
  for (const auto entity : boundaryView) {
    auto& boundaries
        = boundaryView.get<comps::DeformableBoundaryConditions>(entity);
    for (auto& boundary : boundaries.dirichlet) {
      rebindLoadedBoundaryAllocator(boundary, allocator);
    }
    for (auto& boundary : boundaries.neumann) {
      rebindLoadedBoundaryAllocator(boundary, allocator);
    }
    rebindLoadedVectorAllocator(boundaries.dirichlet, allocator);
    rebindLoadedVectorAllocator(boundaries.neumann, allocator);
  }

  auto variationalStateView
      = registry.view<compute::MultibodyVariationalState>();
  for (const auto entity : variationalStateView) {
    auto& state
        = variationalStateView.get<compute::MultibodyVariationalState>(entity);
    rebindLoadedVectorAllocator(state.previousDeltaTransform, allocator);
    rebindLoadedVectorAllocator(state.previousMomentum, allocator);
  }

  auto variationalContactView = registry.view<comps::VariationalContact>();
  for (const auto entity : variationalContactView) {
    auto& contact
        = variationalContactView.get<comps::VariationalContact>(entity);
    rebindLoadedVectorAllocator(contact.pointLinkIndices, allocator);
    rebindLoadedVectorAllocator(contact.pointLocalPositions, allocator);
  }

  auto variationalDualView
      = registry.view<comps::VariationalContactDualState>();
  for (const auto entity : variationalDualView) {
    auto& state
        = variationalDualView.get<comps::VariationalContactDualState>(entity);
    rebindLoadedVectorAllocator(state.duals, allocator);
  }
}

} // namespace

//==============================================================================
struct World::StepPipelineCache
{
  explicit StepPipelineCache(common::MemoryManager& memoryManager)
    : stages(memoryManager)
  {
  }

  WorldStepPipelineStages stages;
  bool hasAdvanceableRigidBodies = false;
  bool hasMultibodyStructure = false;
  bool hasDeformableBodies = false;
  bool canSkipDefaultPipelineWhenFramesClean = true;
};

//==============================================================================
World::WorldStoragePtr World::makeWorldStorage(
    common::MemoryManager& memoryManager)
{
  auto* storage = memoryManager.constructUsingFree<detail::WorldStorage>(
      memoryManager.getFreeAllocator());
  if (storage == nullptr) {
    throw std::bad_alloc();
  }

  return WorldStoragePtr(storage, WorldStorageDeleter{&memoryManager});
}

//==============================================================================
void World::WorldStorageDeleter::operator()(void* storage) const noexcept
{
  if (storage != nullptr && memoryManager != nullptr) {
    memoryManager->destroyUsingFree(
        static_cast<detail::WorldStorage*>(storage));
  }
}

//==============================================================================
World::CollisionQueryCachePtr World::makeCollisionQueryCache(
    common::MemoryManager& memoryManager)
{
  auto* cache = memoryManager.constructUsingFree<CollisionQueryCache>(
      memoryManager.getFreeAllocator());
  if (cache == nullptr) {
    throw std::bad_alloc();
  }

  return CollisionQueryCachePtr(
      cache, CollisionQueryCacheDeleter{&memoryManager});
}

//==============================================================================
void World::CollisionQueryCacheDeleter::operator()(void* cache) const noexcept
{
  if (cache != nullptr && memoryManager != nullptr) {
    memoryManager->destroyUsingFree(static_cast<CollisionQueryCache*>(cache));
  }
}

//==============================================================================
World::StepPipelineCachePtr World::makeStepPipelineCache(
    common::MemoryManager& memoryManager)
{
  auto* cache
      = memoryManager.constructUsingFree<StepPipelineCache>(memoryManager);
  if (cache == nullptr) {
    throw std::bad_alloc();
  }

  return StepPipelineCachePtr(cache, StepPipelineCacheDeleter{&memoryManager});
}

//==============================================================================
void World::StepPipelineCacheDeleter::operator()(void* cache) const noexcept
{
  if (cache != nullptr && memoryManager != nullptr) {
    memoryManager->destroyUsingFree(static_cast<StepPipelineCache*>(cache));
  }
}

//==============================================================================
World::ReplayStatePtr World::makeReplayState(
    common::MemoryManager& memoryManager)
{
  auto* replayState = memoryManager.constructUsingFree<ReplayState>(
      memoryManager.getFreeAllocator());
  if (replayState == nullptr) {
    throw std::bad_alloc();
  }

  return ReplayStatePtr(replayState, ReplayStateDeleter{&memoryManager});
}

//==============================================================================
void World::ReplayStateDeleter::operator()(void* replayState) const noexcept
{
  if (replayState != nullptr && memoryManager != nullptr) {
    memoryManager->destroyUsingFree(static_cast<ReplayState*>(replayState));
  }
}

World::World()
  : m_storage(makeWorldStorage(m_memoryManager)),
    m_collisionQueryCache(
        nullptr, CollisionQueryCacheDeleter{&m_memoryManager}),
    m_stepPipelineCache(makeStepPipelineCache(m_memoryManager)),
    m_replay(nullptr, ReplayStateDeleter{&m_memoryManager})
{
  // Empty.
}

//==============================================================================
World::World(const WorldOptions& options)
  : m_memoryManager(
        resolveBaseAllocator(options), makeMemoryManagerOptions(options)),
    m_storage(makeWorldStorage(m_memoryManager)),
    m_gravity(options.gravity),
    m_rigidBodySolver(options.rigidBodySolver),
    m_timeStep(options.timeStep),
    m_differentiable(options.differentiable),
    m_contactSolverMethod(options.contactSolverMethod),
    m_contactGradientMode(options.contactGradientMode),
    m_computeAcceleratorPolicy(options.computeAcceleratorPolicy),
    m_strictSolverResolution(options.strictSolverResolution),
    m_deactivationOptions(options.deactivationOptions),
    m_collisionQueryCache(
        nullptr, CollisionQueryCacheDeleter{&m_memoryManager}),
    m_stepPipelineCache(makeStepPipelineCache(m_memoryManager)),
    m_replay(nullptr, ReplayStateDeleter{&m_memoryManager})
{
  DART_SIMULATION_THROW_T_IF(
      !std::isfinite(options.timeStep) || options.timeStep <= 0.0,
      InvalidArgumentException,
      "WorldOptions.timeStep must be positive and finite");
  DART_SIMULATION_THROW_T_IF(
      !options.gravity.array().isFinite().all(),
      InvalidArgumentException,
      "WorldOptions.gravity must contain only finite coordinates");
  DART_SIMULATION_THROW_T_IF(
      !isValidRigidBodySolver(options.rigidBodySolver),
      InvalidArgumentException,
      "WorldOptions.rigidBodySolver is invalid");
  DART_SIMULATION_THROW_T_IF(
      !isValidContactSolverMethod(options.contactSolverMethod),
      InvalidArgumentException,
      "WorldOptions.contactSolverMethod is invalid");
  DART_SIMULATION_THROW_T_IF(
      !isValidContactGradientMode(options.contactGradientMode),
      InvalidArgumentException,
      "WorldOptions.contactGradientMode is invalid");
  DART_SIMULATION_THROW_T_IF(
      !isValidComputeAcceleratorPolicy(options.computeAcceleratorPolicy),
      InvalidArgumentException,
      "WorldOptions.computeAcceleratorPolicy is invalid");
  validateDeactivationOptions(options.deactivationOptions);
  setMultibodyOptions(options.multibodyOptions);
}

//==============================================================================
World::~World() = default;

//==============================================================================
detail::WorldStorage::WorldStorage(common::MemoryAllocator& allocator)
  : memoryAllocator(allocator),
    registry(detail::WorldRegistryAllocator{allocator}),
    differentiableParameters(DifferentiableParameterAllocator{allocator}),
    differentiableTorqueScratch(DifferentiableTorqueAllocator{allocator}),
    differentiableCoordinateScratch(
        detail::ContactFreeStepCoordinateAllocator{allocator}),
    differentiableInverseDynamicsScratch(allocator),
    differentiableDynamicsTermsScratch(allocator),
    differentiableDerivativeScratch(),
    ignoredCollisionPairs(
        std::less<CollisionPairKey>{}, CollisionPairAllocator{allocator}),
    bakedModel(allocator)
{
  // Empty.
}

//==============================================================================
detail::BakedWorldModel::BakedWorldModel(common::MemoryAllocator& allocator)
  : rigidBodyEntities(EntityAllocator{allocator}),
    dynamicRigidBodyEntities(EntityAllocator{allocator}),
    rigidBodyIsDynamic(ByteAllocator{allocator}),
    rigidBodyInverseMass(ScalarAllocator{allocator}),
    rigidBodyInertia(ScalarAllocator{allocator}),
    multibodies(MultibodyAllocator{allocator}),
    multibodyLinkEntities(EntityAllocator{allocator}),
    multibodyJointEntities(EntityAllocator{allocator}),
    multibodyLinkDofOffsets(IndexAllocator{allocator}),
    multibodyLinkDofs(IndexAllocator{allocator}),
    multibodyLinkMass(ScalarAllocator{allocator}),
    multibodyLinkInertia(ScalarAllocator{allocator})
{
  // Empty.
}

//==============================================================================
void detail::BakedWorldModel::clear() noexcept
{
  valid = false;
  rigidBodyEntities.clear();
  dynamicRigidBodyEntities.clear();
  rigidBodyIsDynamic.clear();
  rigidBodyInverseMass.clear();
  rigidBodyInertia.clear();
  multibodies.clear();
  multibodyLinkEntities.clear();
  multibodyJointEntities.clear();
  multibodyLinkDofOffsets.clear();
  multibodyLinkDofs.clear();
  multibodyLinkMass.clear();
  multibodyLinkInertia.clear();
}

//==============================================================================
detail::WorldStorage& detail::storageOf(World& world)
{
  return *world.m_storage;
}

//==============================================================================
const detail::WorldStorage& detail::storageOf(const World& world)
{
  return *world.m_storage;
}

//==============================================================================
detail::WorldRegistry& detail::registryOf(World& world)
{
  return detail::storageOf(world).registry;
}

//==============================================================================
const detail::WorldRegistry& detail::registryOf(const World& world)
{
  return detail::storageOf(world).registry;
}

//==============================================================================
const detail::BakedWorldModel& detail::ensureBakedWorldModelCurrent(
    const World& world)
{
  auto& storage = const_cast<detail::WorldStorage&>(detail::storageOf(world));
  auto& model = storage.bakedModel;
  if (model.valid) {
    return model;
  }

  auto& registry = storage.registry;
  model.clear();

  auto rigidBodyView = registry.view<
      comps::RigidBodyTag,
      comps::Transform,
      comps::Velocity,
      comps::MassProperties,
      comps::Force>();
  model.rigidBodyEntities.reserve(rigidBodyView.size_hint());
  model.dynamicRigidBodyEntities.reserve(rigidBodyView.size_hint());
  model.rigidBodyIsDynamic.reserve(rigidBodyView.size_hint());
  model.rigidBodyInverseMass.reserve(rigidBodyView.size_hint());
  model.rigidBodyInertia.reserve(9 * rigidBodyView.size_hint());
  for (const auto entity : rigidBodyView) {
    model.rigidBodyEntities.push_back(entity);
  }
  std::ranges::sort(model.rigidBodyEntities, [](auto lhs, auto rhs) {
    return entt::to_integral(lhs) < entt::to_integral(rhs);
  });
  for (const auto entity : model.rigidBodyEntities) {
    const bool dynamic = !registry.all_of<comps::StaticBodyTag>(entity);
    model.rigidBodyIsDynamic.push_back(dynamic ? 1u : 0u);
    if (dynamic) {
      model.dynamicRigidBodyEntities.push_back(entity);
    }

    const auto& mass = registry.get<comps::MassProperties>(entity);
    const double inverse
        = (mass.mass > 0.0 && std::isfinite(mass.mass)) ? 1.0 / mass.mass : 0.0;
    model.rigidBodyInverseMass.push_back(inverse);
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        model.rigidBodyInertia.push_back(mass.inertia(row, col));
      }
    }
  }

  auto multibodyView = registry.view<comps::MultibodyStructure>();
  std::vector<entt::entity, detail::BakedWorldModel::EntityAllocator>
      multibodyEntities(model.rigidBodyEntities.get_allocator());
  for (const auto entity : multibodyView) {
    multibodyEntities.push_back(entity);
  }
  std::ranges::sort(multibodyEntities, [](auto lhs, auto rhs) {
    return entt::to_integral(lhs) < entt::to_integral(rhs);
  });

  std::size_t multibodyDofOffset = 0;
  for (const auto entity : multibodyEntities) {
    const auto& structure = registry.get<comps::MultibodyStructure>(entity);
    detail::BakedMultibodyModel baked;
    baked.entity = entity;
    baked.linkOffset = model.multibodyLinkEntities.size();
    baked.linkCount = structure.links.size();
    baked.jointOffset = model.multibodyJointEntities.size();
    baked.jointCount = structure.joints.size();
    baked.dofOffset = multibodyDofOffset;

    model.multibodyLinkEntities.insert(
        model.multibodyLinkEntities.end(),
        structure.links.begin(),
        structure.links.end());
    model.multibodyJointEntities.insert(
        model.multibodyJointEntities.end(),
        structure.joints.begin(),
        structure.joints.end());

    std::size_t localDofOffset = 0;
    for (const auto linkEntity : structure.links) {
      const auto& link = registry.get<comps::LinkModel>(linkEntity);
      std::size_t linkDofs = 0;
      if (link.parentJoint != entt::null) {
        linkDofs = registry.get<comps::JointModel>(link.parentJoint).getDOF();
      }
      model.multibodyLinkDofOffsets.push_back(localDofOffset);
      model.multibodyLinkDofs.push_back(linkDofs);
      localDofOffset += linkDofs;

      model.multibodyLinkMass.push_back(link.mass.mass);
      for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
          model.multibodyLinkInertia.push_back(link.mass.inertia(row, col));
        }
      }
    }
    baked.dofCount = localDofOffset;
    multibodyDofOffset += baked.dofCount;
    model.multibodies.push_back(baked);
  }

  ++model.rigidBodyModelBuildCount;
  model.valid = true;
  return model;
}

//==============================================================================
common::MemoryManager& World::getMemoryManager()
{
  return m_memoryManager;
}

//==============================================================================
const common::MemoryManager& World::getMemoryManager() const
{
  return m_memoryManager;
}

//==============================================================================
WorldMemoryDiagnostics World::getMemoryDiagnostics() const
{
  WorldMemoryDiagnostics diagnostics = m_memoryDiagnostics;
  diagnostics.allocatorDebugDiagnostics = m_memoryManager.getDebugDiagnostics();
  diagnostics.ecsDiagnostics
      = makeWorldEcsDiagnostics(detail::registryOf(*this));
  const auto& frameAllocator = m_memoryManager.getFrameAllocator();
  const auto overflowBytes = frameAllocator.overflowBytes();
  diagnostics.frameScratchCapacityBytes = frameAllocator.usableCapacity();
  diagnostics.frameScratchUsedBytes = frameAllocator.used() + overflowBytes;
  diagnostics.frameScratchOverflowCount = frameAllocator.overflowCount();
  diagnostics.frameScratchOverflowBytes = overflowBytes;
  diagnostics.frameScratchPeakUsedBytes = std::max(
      diagnostics.frameScratchPeakUsedBytes, diagnostics.frameScratchUsedBytes);
  return diagnostics;
}

void World::clear()
{
  // Recreate the opaque storage at the rebuild boundary so registry/component
  // capacities and other allocator-backed build artifacts release their live
  // allocations instead of surviving as stale storage in an empty World.
  m_collisionQueryCache.reset();
  m_stepPipelineCache.reset();
  m_replay.reset();
  m_storage.reset();
  m_storage = makeWorldStorage(m_memoryManager);
  markFrameTopologyChanged();
  m_simulationMode = false;
  m_gravity = Eigen::Vector3d(0.0, 0.0, -9.81);
  m_rigidBodySolver = RigidBodySolver::SequentialImpulse;
  m_multibodyIntegrationMethod = MultibodyIntegrationMethod::SemiImplicit;
  m_variationalIntegratorMaxIterations = 100;
  m_variationalIntegratorTolerance = 1e-10;
  m_timeStep = 0.001;
  m_differentiable = false;
  m_contactSolverMethod = ContactSolverMethod::SequentialImpulse;
  m_contactGradientMode = ContactGradientMode::Analytic;
  m_computeAcceleratorPolicy = ComputeAcceleratorPolicy::CpuOnly;
  m_deformablePsdProjector = nullptr;
  m_deformablePsdAcceleratedResolved = false;
  m_deactivationOptions = {};
  resetRigidIpcAdaptiveBarrierStiffnessLowerBound();
  m_lastDeformableSolverDiagnostics = {};
  m_time = 0.0;
  m_frame = 0;
  m_memoryManager.getFrameAllocator().reset();
  m_memoryDiagnostics = {};
#if DART_BUILD_PROFILE
  m_stepProfilingEnabled = false;
  m_lastStepProfile.reset();
  m_stepProfileScratch.reset();
#endif
  m_freeFrameCounter = 0;
  m_fixedFrameCounter = 0;
  m_multibodyCounter = 0;
  m_loopClosureCounter = 0;
  m_rigidBodyCounter = 0;
  m_deformableBodyCounter = 0;
  m_linkCounter = 0;
  m_jointCounter = 0;
  m_stepPipelineCache = makeStepPipelineCache(m_memoryManager);
}

//==============================================================================
void World::ensureDesignMode() const
{
  DART_SIMULATION_THROW_T_IF(
      m_simulationMode,
      InvalidOperationException,
      "World modifications are not allowed while in simulation mode");
}

//==============================================================================
void World::markFrameTopologyChanged() noexcept
{
  ++m_frameTopologyRevision;
  if (m_storage != nullptr) {
    m_storage->bakedModel.valid = false;
  }
}

//==============================================================================
void World::markModelChanged() noexcept
{
  if (m_storage != nullptr) {
    m_storage->bakedModel.valid = false;
  }
}

//==============================================================================
std::uint64_t World::getFrameTopologyRevision() const noexcept
{
  return m_frameTopologyRevision;
}

//==============================================================================
void World::reserveRegistryStorageForSimulation()
{
  auto& registry = m_storage->registry;

  // Creating queried storage objects is part of the bake boundary: EnTT's
  // non-const view/all_of paths materialize missing component pools even when
  // no entity owns that component. Doing this here keeps repeated steps from
  // changing the registry storage set; absent components keep zero payload
  // capacity.
  ensureRegistryStorages<
      comps::Name,
      comps::ContactMaterial,
      comps::CollisionGeometry,
      comps::DeformableGroundBarrierTag,
      comps::DeformableSurfaceCcdObstacleTag,
      comps::DeformableObstacleNoCcdTag,
      comps::RigidBodyTag,
      comps::StaticBodyTag,
      comps::KinematicBodyTag,
      comps::KinematicBodyStepTrace,
      comps::RigidAvbdContactConfig,
      comps::MultibodyTag,
      comps::MultibodyStructure,
      comps::LoopClosure,
      comps::LinkModel,
      comps::LinkState,
      comps::LinkControl,
      comps::JointModel,
      comps::JointState,
      comps::JointActuation,
      comps::FrameTag,
      comps::FixedFrameTag,
      comps::FreeFrameTag,
      comps::FrameState,
      comps::FrameCache,
      comps::FixedFrameProperties,
      comps::FreeFrameProperties,
      comps::Transform,
      comps::Velocity,
      comps::MassProperties,
      comps::Force,
      comps::DeactivationState,
      comps::DeformableBodyTag,
      comps::DeformableNodeState,
      comps::DeformableSpringModel,
      comps::DeformableMeshTopology,
      comps::DeformableMaterial,
      comps::DeformableBoundaryConditions,
      comps::DeformableVbdConfig,
      comps::DeformableSolverScratch,
      comps::VariationalContact,
      comps::VariationalContactDualState,
      compute::MultibodyVariationalState,
      compute::MultibodyVariationalScratch,
      detail::deformable_vbd::AvbdRigidWorldPointJointConfig,
      detail::deformable_vbd::AvbdRigidWorldDistanceSpringConfig>(registry);

  reserveExistingRegistryStorages(registry);

  const auto kinematicBodyCount
      = existingComponentStorageSize<comps::KinematicBodyTag>(registry);
  if (kinematicBodyCount > 0u) {
    auto kinematicBodies = registry.view<comps::KinematicBodyTag>();
    reserveAndPrimeDefaultComponentStorage<comps::KinematicBodyStepTrace>(
        registry, kinematicBodies, kinematicBodyCount);
  }

  const auto deformableBodyCount
      = existingComponentStorageSize<comps::DeformableBodyTag>(registry);
  if (deformableBodyCount > 0u) {
    auto deformableBodies = registry.view<comps::DeformableBodyTag>();
    reserveAndPrimeComponentStorage<comps::DeformableSolverScratch>(
        registry,
        deformableBodies,
        deformableBodyCount,
        m_memoryManager.getFreeAllocator());
    compute::reserveDeformableDynamicsRegistryStorage(
        registry, deformableBodyCount, m_memoryManager.getFreeAllocator());
  }

  const auto multibodyCount
      = existingComponentStorageSize<comps::MultibodyStructure>(registry);
  if (multibodyCount > 0u) {
    compute::reserveMultibodyDynamicsRegistryStorage(
        registry, multibodyCount, m_memoryManager.getFreeAllocator());
  }
  if (m_multibodyIntegrationMethod == MultibodyIntegrationMethod::Variational
      && multibodyCount > 0u) {
    compute::reserveMultibodyVariationalRegistryStorage(
        registry, multibodyCount, m_memoryManager.getFreeAllocator());
  }

  const auto jointCount
      = existingComponentStorageSize<comps::JointModel>(registry);
  if (jointCount > 0u) {
    registry.storage<detail::deformable_vbd::AvbdRigidWorldPointJointConfig>()
        .reserve(jointCount);
  }
  const auto springCount = existingComponentStorageSize<
      detail::deformable_vbd::AvbdRigidWorldDistanceSpringConfig>(registry);
  if (springCount > 0u) {
    registry
        .storage<detail::deformable_vbd::AvbdRigidWorldDistanceSpringConfig>()
        .reserve(springCount);
  }

  reserveExistingRegistryStorages(registry);
}

//==============================================================================
void World::prepareStepPipelineCacheForCurrentConfiguration()
{
  reserveRegistryStorageForSimulation();
  (void)detail::ensureBakedWorldModelCurrent(*this);
  auto& cache = *m_stepPipelineCache;
  cache.hasAdvanceableRigidBodies = hasAdvanceableRigidBodyStructures(*this);
  cache.hasMultibodyStructure = hasMultibodyStructures(*this);
  cache.hasDeformableBodies = hasDeformableBodies(*this);
  cache.canSkipDefaultPipelineWhenFramesClean
      = canSkipDefaultStepPipelineWhenFramesClean(
          *this,
          cache.hasAdvanceableRigidBodies,
          cache.hasMultibodyStructure,
          cache.hasDeformableBodies);
  cache.stages.prepare(
      *this,
      m_rigidBodySolver,
      m_multibodyIntegrationMethod == MultibodyIntegrationMethod::Variational,
      cache.hasAdvanceableRigidBodies,
      cache.hasMultibodyStructure,
      cache.hasDeformableBodies);
  resolveComputeAcceleratorForCurrentConfiguration();
  recordResolvedConfiguration();
}

//==============================================================================
void World::resolveComputeAcceleratorForCurrentConfiguration()
{
  m_deformablePsdProjector = &compute::projectSymmetricBlocksToPsdCpu;
  m_deformablePsdAcceleratedResolved = false;

  switch (m_computeAcceleratorPolicy) {
    case ComputeAcceleratorPolicy::CpuOnly:
      return;
    case ComputeAcceleratorPolicy::PreferAccelerated:
      if (compute::DeformablePsdBlockProjector projector
          = compute::deformablePsdAcceleratorProjector();
          projector != nullptr) {
        m_deformablePsdProjector = projector;
        m_deformablePsdAcceleratedResolved = true;
      }
      return;
  }
}

//==============================================================================
void World::recordResolvedConfiguration()
{
  // PLAN-091 WP-091.11: snapshot the resolved per-domain method families
  // (requested -> resolved, with reasons). Known silent substitutions are
  // recorded as substitution notes; under strictSolverResolution they become an
  // error instead.
  m_resolvedConfiguration.reset();

  const char* rigidSolver = "unknown";
  switch (m_rigidBodySolver) {
    case RigidBodySolver::SequentialImpulse:
      rigidSolver = "sequential-impulse";
      break;
    case RigidBodySolver::Ipc:
      rigidSolver = "ipc";
      break;
  }
  m_resolvedConfiguration.notes.push_back(
      {"rigid-body", rigidSolver, rigidSolver, "as requested"});

  const char* contactMethod = "unknown";
  switch (m_contactSolverMethod) {
    case ContactSolverMethod::SequentialImpulse:
      contactMethod = "sequential-impulse";
      break;
    case ContactSolverMethod::BoxedLcp:
      contactMethod = "boxed-lcp";
      break;
  }
  // The internal AVBD rigid-contact opt-in is not facade-selectable (it is
  // emplaced per body), so when it is present the resolved contact path differs
  // from the requested `ContactSolverMethod`: configured contacts run AVBD and
  // the rest fall back to sequential impulse (PLAN-091 WP-091.1). Record that
  // substitution explicitly instead of letting it happen silently.
  const detail::WorldRegistry& registry = m_storage->registry;
  const auto* avbdStorage = registry.storage<comps::RigidAvbdContactConfig>();
  const bool hasAvbdContactConfigs
      = avbdStorage != nullptr && avbdStorage->size() != 0u;
  if (hasAvbdContactConfigs) {
    m_resolvedConfiguration.notes.push_back(
        {"rigid-contact",
         contactMethod,
         std::string(contactMethod) + " + avbd (opt-in)",
         "internal AVBD rigid-contact opt-in active on some bodies; configured "
         "contacts run AVBD, the rest run sequential impulse (not "
         "facade-selectable -- PLAN-091 WP-091.1)"});
  } else {
    m_resolvedConfiguration.notes.push_back(
        {"rigid-contact", contactMethod, contactMethod, "as requested"});
  }

  const char* multibody = "unknown";
  switch (m_multibodyIntegrationMethod) {
    case MultibodyIntegrationMethod::SemiImplicit:
      multibody = "semi-implicit";
      break;
    case MultibodyIntegrationMethod::Variational:
      multibody = "variational";
      break;
  }
  m_resolvedConfiguration.notes.push_back(
      {"multibody", multibody, multibody, "as requested"});

  const char* requestedAccelerator = "unknown";
  switch (m_computeAcceleratorPolicy) {
    case ComputeAcceleratorPolicy::CpuOnly:
      requestedAccelerator = "cpu";
      break;
    case ComputeAcceleratorPolicy::PreferAccelerated:
      requestedAccelerator = "accelerated";
      break;
  }
  const char* resolvedAccelerator
      = m_deformablePsdAcceleratedResolved ? "accelerated" : "cpu";
  const char* acceleratorReason = "as requested";
  if (m_computeAcceleratorPolicy == ComputeAcceleratorPolicy::PreferAccelerated
      && !m_deformablePsdAcceleratedResolved) {
    acceleratorReason = "no available accelerator registered";
  }
  m_resolvedConfiguration.notes.push_back(
      {"deformable-psd",
       requestedAccelerator,
       resolvedAccelerator,
       acceleratorReason});

  DART_SIMULATION_THROW_T_IF(
      m_strictSolverResolution && m_resolvedConfiguration.hasSubstitution(),
      InvalidArgumentException,
      "strict solver resolution is enabled and the World substituted a solver "
      "method it did not request; inspect getResolvedConfiguration() for the "
      "recorded substitution");
}

//==============================================================================
FreeFrame World::addFreeFrame()
{
  return addFreeFrame("", Frame::world());
}

//==============================================================================
FreeFrame World::addFreeFrame(std::string_view name)
{
  return addFreeFrame(name, Frame::world());
}

//==============================================================================
FreeFrame World::addFreeFrame(std::string_view name, const Frame& parent)
{
  ensureDesignMode();
  Frame parentFrame = resolveParentFrame(parent);

  std::string actualName;
  auto entity = createFrameEntity(
      name,
      parentFrame,
      Eigen::Isometry3d::Identity(),
      &m_freeFrameCounter,
      "free_frame",
      false,
      actualName);

  return FreeFrame(entity, this);
}

//==============================================================================
FixedFrame World::addFixedFrame(std::string_view name, const Frame& parent)
{
  return addFixedFrame(name, parent, Eigen::Isometry3d::Identity());
}

//==============================================================================
FixedFrame World::addFixedFrame(
    std::string_view name, const Frame& parent, const Eigen::Isometry3d& offset)
{
  ensureDesignMode();
  Frame parentFrame = resolveParentFrame(parent);

  DART_SIMULATION_THROW_T_IF(
      name.empty(),
      InvalidArgumentException,
      "FixedFrame requires a non-empty name");

  DART_SIMULATION_THROW_T_IF(
      parentFrame.isWorld(),
      InvalidArgumentException,
      "FixedFrame cannot be attached directly to the world frame");

  std::string actualName;
  auto entity = createFrameEntity(
      name,
      parentFrame,
      offset,
      &m_fixedFrameCounter,
      "fixed_frame",
      true,
      actualName);

  return FixedFrame(entity, this);
}

//==============================================================================
Entity World::createFrameEntity(
    std::string_view name,
    const Frame& parentFrame,
    const Eigen::Isometry3d& localTransform,
    std::size_t* autoNameCounter,
    std::string_view autoNamePrefix,
    bool isFixedFrame,
    std::string& outName)
{
  std::string actualName;
  if (name.empty()) {
    if (autoNameCounter) {
      actualName
          = std::format("{}_{:03d}", autoNamePrefix, ++(*autoNameCounter));
    } else {
      actualName = std::string(autoNamePrefix);
    }
  } else {
    actualName = std::string(name);
  }

  auto entity = m_storage->registry.create();
  m_storage->registry.emplace<comps::Name>(entity, actualName);
  m_storage->registry.emplace<comps::FrameTag>(entity);

  if (isFixedFrame) {
    m_storage->registry.emplace<comps::FixedFrameTag>(entity);
  } else {
    m_storage->registry.emplace<comps::FreeFrameTag>(entity);
  }

  auto& state = m_storage->registry.emplace<comps::FrameState>(entity);
  state.parentFrame = parentFrame.isWorld()
                          ? entt::null
                          : detail::toRegistryEntity(parentFrame.getEntity());

  auto& cache = m_storage->registry.emplace<comps::FrameCache>(entity);
  cache.worldTransform = Eigen::Isometry3d::Identity();
  cache.needTransformUpdate = true;

  if (isFixedFrame) {
    auto& props
        = m_storage->registry.emplace<comps::FixedFrameProperties>(entity);
    props.localTransform = localTransform;
  } else {
    auto& props
        = m_storage->registry.emplace<comps::FreeFrameProperties>(entity);
    props.localTransform = localTransform;
  }

  outName = actualName;
  markFrameTopologyChanged();
  return detail::fromRegistryEntity(entity);
}

//==============================================================================
Frame World::resolveParentFrame(const Frame& parent) const
{
  if (parent.isWorld()) {
    return Frame(Entity{}, const_cast<World*>(this));
  }

  DART_SIMULATION_THROW_T_IF(
      !parent.isValid(),
      InvalidArgumentException,
      "Parent frame is invalid or has been destroyed");

  DART_SIMULATION_THROW_T_IF(
      parent.getWorld() != this,
      InvalidArgumentException,
      "Parent frame belongs to a different world");

  return parent;
}

//==============================================================================
Multibody World::addMultibody(std::string_view name)
{
  ensureDesignMode();
  DART_SIMULATION_THROW_T_IF(
      hasRigidBodyAvbdPairConstraints(*this),
      InvalidOperationException,
      "Multibody structures are not supported in worlds with rigid-body "
      "AVBD pair constraints");

  std::string candidateName;
  if (name.empty()) {
    do {
      candidateName = std::format("multibody_{:03d}", ++m_multibodyCounter);
    } while (hasEntityWithName<comps::MultibodyTag>(
        m_storage->registry, candidateName));
  } else {
    candidateName = std::string(name);
    DART_SIMULATION_THROW_T_IF(
        hasEntityWithName<comps::MultibodyTag>(
            m_storage->registry, candidateName),
        InvalidArgumentException,
        "Multibody '{}' already exists",
        candidateName);
  }

  auto entity = m_storage->registry.create();
  m_storage->registry.emplace<comps::Name>(entity, candidateName);
  m_storage->registry.emplace<comps::MultibodyTag>(entity);
  auto& structure
      = m_storage->registry.emplace<comps::MultibodyStructure>(entity);
  structure.links = comps::MultibodyStructure::EntityVector{
      common::StlAllocator<entt::entity>{
          getMemoryManager().getFreeAllocator()}};
  structure.joints = comps::MultibodyStructure::EntityVector{
      common::StlAllocator<entt::entity>{
          getMemoryManager().getFreeAllocator()}};

  return Multibody(detail::fromRegistryEntity(entity), this);
}

//==============================================================================
std::optional<Multibody> World::getMultibody(std::string_view name)
{
  auto view = m_storage->registry.view<comps::MultibodyTag, comps::Name>();
  for (auto entity : view) {
    const auto& info = view.get<comps::Name>(entity);
    if (info.name == name) {
      return Multibody(detail::fromRegistryEntity(entity), this);
    }
  }
  return std::nullopt;
}

//==============================================================================
bool World::hasMultibody(std::string_view name) const
{
  return hasEntityWithName<comps::MultibodyTag>(m_storage->registry, name);
}

//==============================================================================
std::size_t World::getMultibodyCount() const
{
  return countEntities<comps::MultibodyTag>(m_storage->registry);
}

//==============================================================================
Joint World::addArticulatedFixedJoint(
    std::string_view name, const Link& parent, const Link& child)
{
  return addArticulatedJoint(
      name, &parent, child, JointType::Fixed, Eigen::Vector3d::UnitZ());
}

//==============================================================================
Joint World::addArticulatedFixedJoint(
    std::string_view name,
    const Link& parent,
    const Link& child,
    const Eigen::Vector3d& parentAnchor,
    const Eigen::Vector3d& childAnchor)
{
  return addArticulatedJoint(
      name,
      &parent,
      child,
      JointType::Fixed,
      Eigen::Vector3d::UnitZ(),
      parentAnchor,
      childAnchor);
}

//==============================================================================
Joint World::addArticulatedFixedJoint(std::string_view name, const Link& child)
{
  return addArticulatedJoint(
      name, nullptr, child, JointType::Fixed, Eigen::Vector3d::UnitZ());
}

//==============================================================================
Joint World::addArticulatedFixedJoint(
    std::string_view name,
    const Link& child,
    const Eigen::Vector3d& worldAnchor,
    const Eigen::Vector3d& childAnchor)
{
  return addArticulatedJoint(
      name,
      nullptr,
      child,
      JointType::Fixed,
      Eigen::Vector3d::UnitZ(),
      worldAnchor,
      childAnchor);
}

//==============================================================================
Joint World::addArticulatedRevoluteJoint(
    std::string_view name,
    const Link& parent,
    const Link& child,
    const Eigen::Vector3d& axis)
{
  return addArticulatedJoint(name, &parent, child, JointType::Revolute, axis);
}

//==============================================================================
Joint World::addArticulatedRevoluteJoint(
    std::string_view name,
    const Link& parent,
    const Link& child,
    const Eigen::Vector3d& axis,
    const Eigen::Vector3d& parentAnchor,
    const Eigen::Vector3d& childAnchor)
{
  return addArticulatedJoint(
      name,
      &parent,
      child,
      JointType::Revolute,
      axis,
      parentAnchor,
      childAnchor);
}

//==============================================================================
Joint World::addArticulatedRevoluteJoint(
    std::string_view name, const Link& child, const Eigen::Vector3d& axis)
{
  return addArticulatedJoint(name, nullptr, child, JointType::Revolute, axis);
}

//==============================================================================
Joint World::addArticulatedRevoluteJoint(
    std::string_view name,
    const Link& child,
    const Eigen::Vector3d& axis,
    const Eigen::Vector3d& worldAnchor,
    const Eigen::Vector3d& childAnchor)
{
  return addArticulatedJoint(
      name,
      nullptr,
      child,
      JointType::Revolute,
      axis,
      worldAnchor,
      childAnchor);
}

//==============================================================================
Joint World::addArticulatedPrismaticJoint(
    std::string_view name,
    const Link& parent,
    const Link& child,
    const Eigen::Vector3d& axis)
{
  return addArticulatedJoint(name, &parent, child, JointType::Prismatic, axis);
}

//==============================================================================
Joint World::addArticulatedPrismaticJoint(
    std::string_view name,
    const Link& parent,
    const Link& child,
    const Eigen::Vector3d& axis,
    const Eigen::Vector3d& parentAnchor,
    const Eigen::Vector3d& childAnchor)
{
  return addArticulatedJoint(
      name,
      &parent,
      child,
      JointType::Prismatic,
      axis,
      parentAnchor,
      childAnchor);
}

//==============================================================================
Joint World::addArticulatedPrismaticJoint(
    std::string_view name, const Link& child, const Eigen::Vector3d& axis)
{
  return addArticulatedJoint(name, nullptr, child, JointType::Prismatic, axis);
}

//==============================================================================
Joint World::addArticulatedPrismaticJoint(
    std::string_view name,
    const Link& child,
    const Eigen::Vector3d& axis,
    const Eigen::Vector3d& worldAnchor,
    const Eigen::Vector3d& childAnchor)
{
  return addArticulatedJoint(
      name,
      nullptr,
      child,
      JointType::Prismatic,
      axis,
      worldAnchor,
      childAnchor);
}

//==============================================================================
Joint World::addArticulatedSphericalJoint(
    std::string_view name, const Link& parent, const Link& child)
{
  return addArticulatedJoint(
      name, &parent, child, JointType::Spherical, Eigen::Vector3d::UnitZ());
}

//==============================================================================
Joint World::addArticulatedSphericalJoint(
    std::string_view name,
    const Link& parent,
    const Link& child,
    const Eigen::Vector3d& parentAnchor,
    const Eigen::Vector3d& childAnchor)
{
  return addArticulatedJoint(
      name,
      &parent,
      child,
      JointType::Spherical,
      Eigen::Vector3d::UnitZ(),
      parentAnchor,
      childAnchor);
}

//==============================================================================
Joint World::addArticulatedSphericalJoint(
    std::string_view name, const Link& child)
{
  return addArticulatedJoint(
      name, nullptr, child, JointType::Spherical, Eigen::Vector3d::UnitZ());
}

//==============================================================================
Joint World::addArticulatedSphericalJoint(
    std::string_view name,
    const Link& child,
    const Eigen::Vector3d& worldAnchor,
    const Eigen::Vector3d& childAnchor)
{
  return addArticulatedJoint(
      name,
      nullptr,
      child,
      JointType::Spherical,
      Eigen::Vector3d::UnitZ(),
      worldAnchor,
      childAnchor);
}

//==============================================================================
Joint World::addArticulatedJoint(
    std::string_view name,
    const Link* parent,
    const Link& child,
    JointType type,
    const Eigen::Vector3d& axis,
    std::optional<Eigen::Vector3d> parentAnchor,
    std::optional<Eigen::Vector3d> childAnchor)
{
  ensureDesignMode();

  const comps::JointType componentType
      = toArticulatedPointJointComponentJointType(type);
  DART_SIMULATION_THROW_T_IF(
      articulatedPointJointUsesAxis(componentType)
          && (!axis.allFinite() || axis.squaredNorm() <= 0.0),
      InvalidArgumentException,
      "Articulated point-joint axis must be finite and non-zero");
  DART_SIMULATION_THROW_T_IF(
      parent != nullptr && !parent->isValid(),
      InvalidArgumentException,
      "Articulated point-joint parent link is invalid or has been destroyed");
  DART_SIMULATION_THROW_T_IF(
      !child.isValid(),
      InvalidArgumentException,
      "Articulated point-joint child link is invalid or has been destroyed");
  DART_SIMULATION_THROW_T_IF(
      (parent != nullptr && parent->getWorld() != this)
          || child.getWorld() != this,
      InvalidArgumentException,
      "Articulated point-joint links must belong to this World");
  DART_SIMULATION_THROW_T_IF(
      parent != nullptr && parent->getEntity() == child.getEntity(),
      InvalidArgumentException,
      "Articulated point-joint parent and child links must be distinct");
  DART_SIMULATION_THROW_T_IF(
      parentAnchor.has_value() != childAnchor.has_value(),
      InvalidArgumentException,
      "Articulated point-joint anchors must be provided for both endpoints");
  DART_SIMULATION_THROW_T_IF(
      parentAnchor.has_value()
          && (!parentAnchor->allFinite() || !childAnchor->allFinite()),
      InvalidArgumentException,
      "Articulated point-joint anchors must be finite");

  const entt::entity parentEntity
      = parent == nullptr ? entt::null
                          : detail::toRegistryEntity(parent->getEntity());
  const entt::entity childEntity = detail::toRegistryEntity(child.getEntity());
  const entt::entity parentStructure
      = parentEntity == entt::null
            ? entt::null
            : findOwningMultibodyStructure(m_storage->registry, parentEntity);
  const entt::entity childStructure
      = findOwningMultibodyStructure(m_storage->registry, childEntity);
  DART_SIMULATION_THROW_T_IF(
      childStructure == entt::null,
      InvalidArgumentException,
      "Articulated point-joint child link must belong to a multibody");
  DART_SIMULATION_THROW_T_IF(
      parentEntity != entt::null
          && (parentStructure == entt::null
              || parentStructure != childStructure),
      InvalidArgumentException,
      "Articulated point-joint links must belong to the same multibody");

  std::string actualName;
  if (name.empty()) {
    do {
      actualName = std::format("joint_{:03d}", ++m_jointCounter);
    } while (
        hasEntityWithName<comps::JointModel>(m_storage->registry, actualName));
  } else {
    actualName = std::string(name);
    DART_SIMULATION_THROW_T_IF(
        hasEntityWithName<comps::JointModel>(m_storage->registry, actualName),
        InvalidArgumentException,
        "Joint '{}' already exists",
        actualName);
  }

  const entt::entity jointEntity = m_storage->registry.create();
  m_storage->registry.emplace<comps::Name>(jointEntity, actualName);

  auto& jointModel
      = m_storage->registry.emplace<comps::JointModel>(jointEntity);
  auto& jointState
      = m_storage->registry.emplace<comps::JointState>(jointEntity);
  auto& jointActuation
      = m_storage->registry.emplace<comps::JointActuation>(jointEntity);
  jointModel.type = componentType;
  jointModel.name = std::move(actualName);
  jointModel.parentLink = parentEntity;
  jointModel.childLink = childEntity;
  if (articulatedPointJointUsesAxis(componentType)) {
    jointModel.axis = axis.normalized();
  }

  const Eigen::Index dof = static_cast<Eigen::Index>(jointModel.getDOF());
  jointState.position = comps::makeJointVector(dof, 0.0);
  jointState.velocity = comps::makeJointVector(dof, 0.0);
  jointState.acceleration = comps::makeJointVector(dof, 0.0);
  jointActuation.torque = comps::makeJointVector(dof, 0.0);
  jointModel.springStiffness = comps::makeJointVector(dof, 0.0);
  jointModel.dampingCoefficient = comps::makeJointVector(dof, 0.0);
  jointModel.restPosition = comps::makeJointVector(dof, 0.0);
  jointModel.armature = comps::makeJointVector(dof, 0.0);
  jointModel.coulombFriction = comps::makeJointVector(dof, 0.0);
  jointActuation.commandVelocity = comps::makeJointVector(dof, 0.0);

  const double infinity = std::numeric_limits<double>::infinity();
  jointModel.limits.lower = comps::makeJointVector(dof, -infinity);
  jointModel.limits.upper = comps::makeJointVector(dof, infinity);
  jointModel.limits.velocityLower = comps::makeJointVector(dof, -infinity);
  jointModel.limits.velocityUpper = comps::makeJointVector(dof, infinity);
  jointModel.limits.effortLower = comps::makeJointVector(dof, -infinity);
  jointModel.limits.effortUpper = comps::makeJointVector(dof, infinity);

  if (parentAnchor.has_value()) {
    Eigen::Matrix3d parentRotation = Eigen::Matrix3d::Identity();
    if (parent != nullptr) {
      parentRotation = parent->getWorldTransform().linear();
    }
    const Eigen::Matrix3d childRotation = child.getWorldTransform().linear();
    DART_SIMULATION_THROW_T_IF(
        !parentRotation.allFinite() || !childRotation.allFinite(),
        InvalidArgumentException,
        "Articulated point-joint endpoint transforms must be finite");

    Eigen::Quaterniond parentOrientation(parentRotation);
    Eigen::Quaterniond childOrientation(childRotation);
    DART_SIMULATION_THROW_T_IF(
        !parentOrientation.coeffs().allFinite()
            || !childOrientation.coeffs().allFinite()
            || parentOrientation.norm() == 0.0
            || childOrientation.norm() == 0.0,
        InvalidArgumentException,
        "Articulated point-joint endpoint orientations must be finite");
    parentOrientation.normalize();
    childOrientation.normalize();

    jointModel.hasRigidBodyFixedJointAnchors = true;
    jointModel.rigidBodyFixedJointLocalAnchorParent = *parentAnchor;
    jointModel.rigidBodyFixedJointLocalAnchorChild = *childAnchor;
    jointModel.rigidBodyFixedJointTargetRelativeOrientation
        = parentOrientation.conjugate() * childOrientation;
    jointModel.rigidBodyFixedJointTargetRelativeOrientation.normalize();
  }

  return Joint(detail::fromRegistryEntity(jointEntity), this);
}

//==============================================================================
std::optional<Joint> World::getArticulatedJoint(std::string_view name)
{
  auto view = m_storage->registry.view<comps::JointModel, comps::Name>();
  for (auto entity : view) {
    const auto& joint = view.get<comps::JointModel>(entity);
    const auto& info = view.get<comps::Name>(entity);
    if (info.name == name
        && isArticulatedPointJoint(m_storage->registry, entity, joint)) {
      return Joint(detail::fromRegistryEntity(entity), this);
    }
  }
  return std::nullopt;
}

//==============================================================================
bool World::hasArticulatedJoint(std::string_view name) const
{
  const auto view = m_storage->registry.view<comps::JointModel, comps::Name>();
  for (auto entity : view) {
    const auto& joint = view.get<comps::JointModel>(entity);
    const auto& info = view.get<comps::Name>(entity);
    if (info.name == name
        && isArticulatedPointJoint(m_storage->registry, entity, joint)) {
      return true;
    }
  }
  return false;
}

//==============================================================================
std::size_t World::getArticulatedJointCount() const
{
  std::size_t count = 0;
  const auto view = m_storage->registry.view<comps::JointModel>();
  for (auto entity : view) {
    const auto& joint = view.get<comps::JointModel>(entity);
    if (isArticulatedPointJoint(m_storage->registry, entity, joint)) {
      ++count;
    }
  }
  return count;
}

//==============================================================================
std::vector<Joint> World::getArticulatedJoints()
{
  std::vector<Joint> joints;
  const auto view = m_storage->registry.view<comps::JointModel>();
  for (auto entity : view) {
    const auto& joint = view.get<comps::JointModel>(entity);
    if (isArticulatedPointJoint(m_storage->registry, entity, joint)) {
      joints.emplace_back(detail::fromRegistryEntity(entity), this);
    }
  }
  return joints;
}

//==============================================================================
LoopClosure World::addLoopClosure(
    std::string_view name, const LoopClosureSpec& spec)
{
  ensureDesignMode();
  validateLoopClosureSpec(*this, spec);

  std::string candidateName;
  if (name.empty()) {
    do {
      candidateName
          = std::format("loop_closure_{:03d}", ++m_loopClosureCounter);
    } while (hasEntityWithName<comps::LoopClosure>(
        m_storage->registry, candidateName));
  } else {
    candidateName = std::string(name);
    DART_SIMULATION_THROW_T_IF(
        hasEntityWithName<comps::LoopClosure>(
            m_storage->registry, candidateName),
        InvalidArgumentException,
        "LoopClosure '{}' already exists",
        candidateName);
  }

  auto entity = m_storage->registry.create();
  m_storage->registry.emplace<comps::Name>(entity, candidateName);

  auto& closure = m_storage->registry.emplace<comps::LoopClosure>(entity);
  closure.family = spec.family;
  closure.frameA = resolveLoopClosureFrame(*this, spec.frameA, "frameA");
  closure.frameB = resolveLoopClosureFrame(*this, spec.frameB, "frameB");
  closure.offsetA = spec.offsetA;
  closure.offsetB = spec.offsetB;
  closure.distance = spec.distance;

  return LoopClosure(detail::fromRegistryEntity(entity), this);
}

//==============================================================================
std::optional<LoopClosure> World::getLoopClosure(std::string_view name)
{
  auto view = m_storage->registry.view<comps::LoopClosure, comps::Name>();
  for (auto entity : view) {
    const auto& info = view.get<comps::Name>(entity);
    if (info.name == name) {
      return LoopClosure(detail::fromRegistryEntity(entity), this);
    }
  }
  return std::nullopt;
}

//==============================================================================
bool World::hasLoopClosure(std::string_view name) const
{
  return hasEntityWithName<comps::LoopClosure>(m_storage->registry, name);
}

//==============================================================================
std::size_t World::getLoopClosureCount() const
{
  return countEntities<comps::LoopClosure>(m_storage->registry);
}

//==============================================================================
RigidBody World::addRigidBody(
    std::string_view name, const RigidBodyOptions& options)
{
  ensureDesignMode();

  std::string candidateName;
  if (name.empty()) {
    do {
      candidateName = std::format("rigid_body_{:03d}", ++m_rigidBodyCounter);
    } while (hasEntityWithName<comps::RigidBodyTag>(
        m_storage->registry, candidateName));
  } else {
    candidateName = std::string(name);
    DART_SIMULATION_THROW_T_IF(
        hasEntityWithName<comps::RigidBodyTag>(
            m_storage->registry, candidateName),
        InvalidArgumentException,
        "RigidBody '{}' already exists",
        candidateName);
  }

  validateRigidBodyOptions(options);

  Frame parent = Frame(Entity{}, this);
  const auto orientation = normalizeOrIdentity(options.orientation);
  const auto initialTransform = toIsometry(options.position, orientation);

  std::string actualName;
  const Entity entity = createFrameEntity(
      candidateName,
      parent,
      initialTransform,
      &m_rigidBodyCounter,
      "rigid_body",
      false,
      actualName);
  const auto enttEntity = detail::toRegistryEntity(entity);

  m_storage->registry.emplace<comps::RigidBodyTag>(enttEntity);

  auto& transform = m_storage->registry.emplace<comps::Transform>(enttEntity);
  transform.position = options.position;
  transform.orientation = orientation;

  auto& velocity = m_storage->registry.emplace<comps::Velocity>(enttEntity);
  velocity.linear = options.linearVelocity;
  velocity.angular = options.angularVelocity;

  auto& mass = m_storage->registry.emplace<comps::MassProperties>(enttEntity);
  mass.mass = options.mass;
  mass.inertia = options.inertia;

  m_storage->registry.emplace<comps::Force>(enttEntity);

  if (options.isStatic) {
    m_storage->registry.emplace<comps::StaticBodyTag>(enttEntity);
  }

  return RigidBody(entity, this);
}

//==============================================================================
Joint World::addRigidBodyFixedJoint(
    std::string_view name, const RigidBody& parent, const RigidBody& child)
{
  return addRigidBodyJoint(
      name, parent, child, JointType::Fixed, Eigen::Vector3d::UnitZ());
}

//==============================================================================
Joint World::addRigidBodyRevoluteJoint(
    std::string_view name,
    const RigidBody& parent,
    const RigidBody& child,
    const Eigen::Vector3d& axis)
{
  return addRigidBodyJoint(name, parent, child, JointType::Revolute, axis);
}

//==============================================================================
Joint World::addRigidBodyPrismaticJoint(
    std::string_view name,
    const RigidBody& parent,
    const RigidBody& child,
    const Eigen::Vector3d& axis)
{
  return addRigidBodyJoint(name, parent, child, JointType::Prismatic, axis);
}

//==============================================================================
Joint World::addRigidBodySphericalJoint(
    std::string_view name, const RigidBody& parent, const RigidBody& child)
{
  return addRigidBodyJoint(
      name, parent, child, JointType::Spherical, Eigen::Vector3d::UnitZ());
}

//==============================================================================
Joint World::addRigidBodySphericalJoint(
    std::string_view name,
    const RigidBody& parent,
    const RigidBody& child,
    const Eigen::Vector3d& parentAnchor,
    const Eigen::Vector3d& childAnchor)
{
  return addRigidBodyJoint(
      name,
      parent,
      child,
      JointType::Spherical,
      Eigen::Vector3d::UnitZ(),
      parentAnchor,
      childAnchor);
}

//==============================================================================
void World::addRigidBodyDistanceSpring(
    std::string_view name,
    const RigidBody& parent,
    const RigidBody& child,
    double restLength,
    double stiffness)
{
  addRigidBodyDistanceSpringImpl(
      name, parent, child, restLength, stiffness, std::nullopt, std::nullopt);
}

//==============================================================================
void World::addRigidBodyDistanceSpring(
    std::string_view name,
    const RigidBody& parent,
    const RigidBody& child,
    double restLength,
    double stiffness,
    const Eigen::Vector3d& parentAnchor,
    const Eigen::Vector3d& childAnchor)
{
  addRigidBodyDistanceSpringImpl(
      name,
      parent,
      child,
      restLength,
      stiffness,
      std::optional<Eigen::Vector3d>{parentAnchor},
      std::optional<Eigen::Vector3d>{childAnchor});
}

//==============================================================================
void World::addRigidBodyDistanceSpringImpl(
    std::string_view name,
    const RigidBody& parent,
    const RigidBody& child,
    double restLength,
    double stiffness,
    std::optional<Eigen::Vector3d> parentAnchor,
    std::optional<Eigen::Vector3d> childAnchor)
{
  ensureDesignMode();

  DART_SIMULATION_THROW_T_IF(
      !std::isfinite(restLength) || restLength < 0.0,
      InvalidArgumentException,
      "Rigid-body distance spring rest length must be finite and non-negative");
  DART_SIMULATION_THROW_T_IF(
      !std::isfinite(stiffness) || stiffness <= 0.0,
      InvalidArgumentException,
      "Rigid-body distance spring stiffness must be finite and positive");
  DART_SIMULATION_THROW_T_IF(
      parentAnchor.has_value() != childAnchor.has_value(),
      InvalidArgumentException,
      "Rigid-body distance spring anchors must provide both endpoints");
  DART_SIMULATION_THROW_T_IF(
      parentAnchor.has_value()
          && (!parentAnchor->allFinite() || !childAnchor->allFinite()),
      InvalidArgumentException,
      "Rigid-body distance spring anchors must be finite");
  DART_SIMULATION_THROW_T_IF(
      !parent.isValid(),
      InvalidArgumentException,
      "Distance spring parent rigid body is invalid or has been destroyed");
  DART_SIMULATION_THROW_T_IF(
      !child.isValid(),
      InvalidArgumentException,
      "Distance spring child rigid body is invalid or has been destroyed");
  DART_SIMULATION_THROW_T_IF(
      parent.getWorld() != this || child.getWorld() != this,
      InvalidArgumentException,
      "Distance spring rigid bodies must belong to this World");
  DART_SIMULATION_THROW_T_IF(
      parent.getEntity() == child.getEntity(),
      InvalidArgumentException,
      "Distance spring parent and child rigid bodies must be distinct");

  const entt::entity parentEntity
      = detail::toRegistryEntity(parent.getEntity());
  const entt::entity childEntity = detail::toRegistryEntity(child.getEntity());
  const bool parentIsRigidBody = m_storage->registry.all_of<
      comps::RigidBodyTag,
      comps::Transform,
      comps::MassProperties>(parentEntity);
  const bool childIsRigidBody = m_storage->registry.all_of<
      comps::RigidBodyTag,
      comps::Transform,
      comps::MassProperties>(childEntity);
  DART_SIMULATION_THROW_T_IF(
      !parentIsRigidBody || !childIsRigidBody,
      InvalidArgumentException,
      "Distance spring endpoints must be valid rigid bodies");
  DART_SIMULATION_THROW_T_IF(
      m_rigidBodySolver == RigidBodySolver::Ipc,
      InvalidOperationException,
      "Rigid-body distance springs are not supported by the IPC rigid-body "
      "solver");
  DART_SIMULATION_THROW_T_IF(
      hasMultibodyStructures(*this),
      InvalidOperationException,
      "Rigid-body distance springs are not supported in worlds with multibody "
      "structures");

  std::string actualName;
  if (name.empty()) {
    do {
      actualName
          = std::format("rigid_distance_spring_{:03d}", ++m_jointCounter);
    } while (hasEntityWithName<
             detail::deformable_vbd::AvbdRigidWorldDistanceSpringConfig>(
        m_storage->registry, actualName));
  } else {
    actualName = std::string(name);
    DART_SIMULATION_THROW_T_IF(
        hasEntityWithName<
            detail::deformable_vbd::AvbdRigidWorldDistanceSpringConfig>(
            m_storage->registry, actualName),
        InvalidArgumentException,
        "Rigid-body distance spring '{}' already exists",
        actualName);
  }

  const entt::entity springEntity = m_storage->registry.create();
  m_storage->registry.emplace<comps::Name>(springEntity, actualName);
  auto& config = m_storage->registry.emplace<
      detail::deformable_vbd::AvbdRigidWorldDistanceSpringConfig>(springEntity);
  config.enabled = true;
  config.bodyA = parentEntity;
  config.bodyB = childEntity;
  if (parentAnchor.has_value()) {
    config.localAnchorA = *parentAnchor;
    config.localAnchorB = *childAnchor;
  }
  config.restLength = restLength;
  config.startStiffness = stiffness;
  config.materialStiffness = stiffness;
  config.maxStiffness = stiffness;
}

//==============================================================================
bool World::hasRigidBodyDistanceSpring(std::string_view name) const
{
  return hasEntityWithName<
      detail::deformable_vbd::AvbdRigidWorldDistanceSpringConfig>(
      m_storage->registry, name);
}

//==============================================================================
std::pair<double, double> World::getRigidBodyDistanceSpringParameters(
    std::string_view name) const
{
  using DistanceSpringConfig
      = detail::deformable_vbd::AvbdRigidWorldDistanceSpringConfig;

  const entt::entity entity
      = findEntityWithName<DistanceSpringConfig>(m_storage->registry, name);
  DART_SIMULATION_THROW_T_IF(
      entity == entt::null,
      InvalidArgumentException,
      "Rigid-body distance spring '{}' does not exist",
      name);

  const auto& config = m_storage->registry.get<DistanceSpringConfig>(entity);
  return {config.restLength, config.materialStiffness};
}

//==============================================================================
void World::setRigidBodyDistanceSpringParameters(
    std::string_view name, double restLength, double stiffness)
{
  DART_SIMULATION_THROW_T_IF(
      !std::isfinite(restLength) || restLength < 0.0,
      InvalidArgumentException,
      "Rigid-body distance spring rest length must be finite and non-negative");
  DART_SIMULATION_THROW_T_IF(
      !std::isfinite(stiffness) || stiffness <= 0.0,
      InvalidArgumentException,
      "Rigid-body distance spring stiffness must be finite and positive");

  using DistanceSpringConfig
      = detail::deformable_vbd::AvbdRigidWorldDistanceSpringConfig;

  const entt::entity entity
      = findEntityWithName<DistanceSpringConfig>(m_storage->registry, name);
  DART_SIMULATION_THROW_T_IF(
      entity == entt::null,
      InvalidArgumentException,
      "Rigid-body distance spring '{}' does not exist",
      name);

  auto& config = m_storage->registry.get<DistanceSpringConfig>(entity);
  config.restLength = restLength;
  config.startStiffness = stiffness;
  config.materialStiffness = stiffness;
  config.maxStiffness = stiffness;
}

//==============================================================================
Joint World::addRigidBodyJoint(
    std::string_view name,
    const RigidBody& parent,
    const RigidBody& child,
    JointType type,
    const Eigen::Vector3d& axis,
    std::optional<Eigen::Vector3d> parentAnchor,
    std::optional<Eigen::Vector3d> childAnchor)
{
  ensureDesignMode();

  const comps::JointType componentType = toRigidBodyComponentJointType(type);
  DART_SIMULATION_THROW_T_IF(
      rigidBodyJointUsesAxis(componentType)
          && (!axis.allFinite() || axis.squaredNorm() <= 0.0),
      InvalidArgumentException,
      "Rigid-body joint axis must be finite and non-zero");
  DART_SIMULATION_THROW_T_IF(
      parentAnchor.has_value() != childAnchor.has_value(),
      InvalidArgumentException,
      "Rigid-body joint anchors must provide both endpoints");
  DART_SIMULATION_THROW_T_IF(
      parentAnchor.has_value()
          && (!parentAnchor->allFinite() || !childAnchor->allFinite()),
      InvalidArgumentException,
      "Rigid-body joint anchors must be finite");
  DART_SIMULATION_THROW_T_IF(
      !parent.isValid(),
      InvalidArgumentException,
      "Joint parent rigid body is invalid or has been destroyed");
  DART_SIMULATION_THROW_T_IF(
      !child.isValid(),
      InvalidArgumentException,
      "Joint child rigid body is invalid or has been destroyed");
  DART_SIMULATION_THROW_T_IF(
      parent.getWorld() != this || child.getWorld() != this,
      InvalidArgumentException,
      "Joint rigid bodies must belong to this World");
  DART_SIMULATION_THROW_T_IF(
      parent.getEntity() == child.getEntity(),
      InvalidArgumentException,
      "Joint parent and child rigid bodies must be distinct");

  const entt::entity parentEntity
      = detail::toRegistryEntity(parent.getEntity());
  const entt::entity childEntity = detail::toRegistryEntity(child.getEntity());
  const bool parentIsRigidBody = m_storage->registry.all_of<
      comps::RigidBodyTag,
      comps::Transform,
      comps::MassProperties>(parentEntity);
  const bool childIsRigidBody = m_storage->registry.all_of<
      comps::RigidBodyTag,
      comps::Transform,
      comps::MassProperties>(childEntity);
  DART_SIMULATION_THROW_T_IF(
      !parentIsRigidBody || !childIsRigidBody,
      InvalidArgumentException,
      "Joint endpoints must be valid rigid bodies");
  DART_SIMULATION_THROW_T_IF(
      m_rigidBodySolver == RigidBodySolver::Ipc
          && componentType != comps::JointType::Fixed
          && componentType != comps::JointType::Revolute,
      InvalidOperationException,
      "Only non-breakable fixed and revolute rigid-body joints are supported "
      "by the IPC rigid-body solver");
  DART_SIMULATION_THROW_T_IF(
      hasMultibodyStructures(*this),
      InvalidOperationException,
      "Rigid-body joints are not supported in worlds with multibody "
      "structures");

  std::string actualName;
  if (name.empty()) {
    do {
      actualName = std::format("joint_{:03d}", ++m_jointCounter);
    } while (
        hasEntityWithName<comps::JointModel>(m_storage->registry, actualName));
  } else {
    actualName = std::string(name);
    DART_SIMULATION_THROW_T_IF(
        hasEntityWithName<comps::JointModel>(m_storage->registry, actualName),
        InvalidArgumentException,
        "Joint '{}' already exists",
        actualName);
  }

  const entt::entity jointEntity = m_storage->registry.create();
  m_storage->registry.emplace<comps::Name>(jointEntity, actualName);

  auto& jointModel
      = m_storage->registry.emplace<comps::JointModel>(jointEntity);
  auto& jointState
      = m_storage->registry.emplace<comps::JointState>(jointEntity);
  auto& jointActuation
      = m_storage->registry.emplace<comps::JointActuation>(jointEntity);
  jointModel.type = componentType;
  jointModel.name = std::move(actualName);
  jointModel.parentLink = parentEntity;
  jointModel.childLink = childEntity;
  if (rigidBodyJointUsesAxis(componentType)) {
    jointModel.axis = axis.normalized();
  }

  if (parentAnchor.has_value()) {
    const auto& parentTransform
        = m_storage->registry.get<comps::Transform>(parentEntity);
    const auto& childTransform
        = m_storage->registry.get<comps::Transform>(childEntity);
    Eigen::Quaterniond parentOrientation = parentTransform.orientation;
    Eigen::Quaterniond childOrientation = childTransform.orientation;
    DART_SIMULATION_THROW_T_IF(
        !parentOrientation.coeffs().allFinite()
            || !childOrientation.coeffs().allFinite()
            || parentOrientation.norm() == 0.0
            || childOrientation.norm() == 0.0,
        InvalidArgumentException,
        "Rigid-body joint endpoint orientations must be finite");
    parentOrientation.normalize();
    childOrientation.normalize();

    jointModel.hasRigidBodyFixedJointAnchors = true;
    jointModel.rigidBodyFixedJointLocalAnchorParent = *parentAnchor;
    jointModel.rigidBodyFixedJointLocalAnchorChild = *childAnchor;
    jointModel.rigidBodyFixedJointTargetRelativeOrientation
        = parentOrientation.conjugate() * childOrientation;
    jointModel.rigidBodyFixedJointTargetRelativeOrientation.normalize();
  }

  const Eigen::Index dof = static_cast<Eigen::Index>(jointModel.getDOF());
  jointState.position = comps::makeJointVector(dof, 0.0);
  jointState.velocity = comps::makeJointVector(dof, 0.0);
  jointState.acceleration = comps::makeJointVector(dof, 0.0);
  jointActuation.torque = comps::makeJointVector(dof, 0.0);
  jointModel.springStiffness = comps::makeJointVector(dof, 0.0);
  jointModel.dampingCoefficient = comps::makeJointVector(dof, 0.0);
  jointModel.restPosition = comps::makeJointVector(dof, 0.0);
  jointModel.armature = comps::makeJointVector(dof, 0.0);
  jointModel.coulombFriction = comps::makeJointVector(dof, 0.0);
  jointActuation.commandVelocity = comps::makeJointVector(dof, 0.0);

  const double infinity = std::numeric_limits<double>::infinity();
  jointModel.limits.lower = comps::makeJointVector(dof, -infinity);
  jointModel.limits.upper = comps::makeJointVector(dof, infinity);
  jointModel.limits.velocityLower = comps::makeJointVector(dof, -infinity);
  jointModel.limits.velocityUpper = comps::makeJointVector(dof, infinity);
  jointModel.limits.effortLower = comps::makeJointVector(dof, -infinity);
  jointModel.limits.effortUpper = comps::makeJointVector(dof, infinity);

  const comps::RigidAvbdContactConfig defaultAvbdConfig;
  comps::AvbdJointStiffness defaultStiffness;
  defaultStiffness.startStiffness = defaultAvbdConfig.startStiffness;
  defaultStiffness.maxStiffness = defaultAvbdConfig.maxStiffness;
  m_storage->registry.emplace_or_replace<comps::AvbdJointStiffness>(
      jointEntity, defaultStiffness);
  if (!detail::deformable_vbd::configureAvbdRigidWorldPointJointFromCurrentPose(
          m_storage->registry,
          jointEntity,
          defaultAvbdConfig.startStiffness,
          defaultAvbdConfig.maxStiffness)) {
    m_storage->registry.destroy(jointEntity);
    DART_SIMULATION_THROW_T(
        InvalidOperationException,
        "Failed to configure rigid-body joint '{}' from current poses",
        name);
  }

  return Joint(detail::fromRegistryEntity(jointEntity), this);
}

//==============================================================================
std::optional<Joint> World::getRigidBodyJoint(std::string_view name)
{
  auto view = m_storage->registry.view<comps::JointModel, comps::Name>();
  for (auto entity : view) {
    const auto& joint = view.get<comps::JointModel>(entity);
    const auto& info = view.get<comps::Name>(entity);
    if (info.name == name && isRigidBodyJoint(m_storage->registry, joint)) {
      return Joint(detail::fromRegistryEntity(entity), this);
    }
  }
  return std::nullopt;
}

//==============================================================================
bool World::hasRigidBodyJoint(std::string_view name) const
{
  const auto view = m_storage->registry.view<comps::JointModel, comps::Name>();
  for (auto entity : view) {
    const auto& joint = view.get<comps::JointModel>(entity);
    const auto& info = view.get<comps::Name>(entity);
    if (info.name == name && isRigidBodyJoint(m_storage->registry, joint)) {
      return true;
    }
  }
  return false;
}

//==============================================================================
std::size_t World::getRigidBodyJointCount() const
{
  std::size_t count = 0;
  const auto view = m_storage->registry.view<comps::JointModel>();
  for (auto entity : view) {
    (void)entity;
    const auto& joint = view.get<comps::JointModel>(entity);
    if (isRigidBodyJoint(m_storage->registry, joint)) {
      ++count;
    }
  }
  return count;
}

//==============================================================================
std::vector<Joint> World::getRigidBodyJoints()
{
  std::vector<Joint> joints;
  joints.reserve(getRigidBodyJointCount());
  const auto view = m_storage->registry.view<comps::JointModel>();
  for (auto entity : view) {
    const auto& joint = view.get<comps::JointModel>(entity);
    if (isRigidBodyJoint(m_storage->registry, joint)) {
      joints.emplace_back(detail::fromRegistryEntity(entity), this);
    }
  }
  return joints;
}

//==============================================================================
std::optional<Joint> World::getRigidBodyFixedJoint(std::string_view name)
{
  auto view = m_storage->registry.view<comps::JointModel, comps::Name>();
  for (auto entity : view) {
    const auto& joint = view.get<comps::JointModel>(entity);
    const auto& info = view.get<comps::Name>(entity);
    if (info.name == name
        && isRigidBodyFixedJoint(m_storage->registry, joint)) {
      return Joint(detail::fromRegistryEntity(entity), this);
    }
  }
  return std::nullopt;
}

//==============================================================================
bool World::hasRigidBodyFixedJoint(std::string_view name) const
{
  const auto view = m_storage->registry.view<comps::JointModel, comps::Name>();
  for (auto entity : view) {
    const auto& joint = view.get<comps::JointModel>(entity);
    const auto& info = view.get<comps::Name>(entity);
    if (info.name == name
        && isRigidBodyFixedJoint(m_storage->registry, joint)) {
      return true;
    }
  }
  return false;
}

//==============================================================================
std::size_t World::getRigidBodyFixedJointCount() const
{
  std::size_t count = 0;
  const auto view = m_storage->registry.view<comps::JointModel>();
  for (auto entity : view) {
    (void)entity;
    const auto& joint = view.get<comps::JointModel>(entity);
    if (isRigidBodyFixedJoint(m_storage->registry, joint)) {
      ++count;
    }
  }
  return count;
}

//==============================================================================
std::vector<Joint> World::getRigidBodyFixedJoints()
{
  std::vector<Joint> joints;
  joints.reserve(getRigidBodyFixedJointCount());
  const auto view = m_storage->registry.view<comps::JointModel>();
  for (auto entity : view) {
    const auto& joint = view.get<comps::JointModel>(entity);
    if (isRigidBodyFixedJoint(m_storage->registry, joint)) {
      joints.emplace_back(detail::fromRegistryEntity(entity), this);
    }
  }
  return joints;
}

//==============================================================================
std::optional<RigidBody> World::getRigidBody(std::string_view name)
{
  auto view = m_storage->registry.view<comps::RigidBodyTag, comps::Name>();
  for (auto entity : view) {
    const auto& info = view.get<comps::Name>(entity);
    if (info.name == name) {
      return RigidBody(detail::fromRegistryEntity(entity), this);
    }
  }
  return std::nullopt;
}

//==============================================================================
bool World::hasRigidBody(std::string_view name) const
{
  return hasEntityWithName<comps::RigidBodyTag>(m_storage->registry, name);
}

//==============================================================================
std::size_t World::getRigidBodyCount() const
{
  return countEntities<comps::RigidBodyTag>(m_storage->registry);
}

//==============================================================================
DeformableBody World::addDeformableBody(
    std::string_view name, const DeformableBodyOptions& options)
{
  ensureDesignMode();
  auto& allocator = m_memoryManager.getFreeAllocator();
  auto data = prepareDeformableBodyOptions(options, allocator);

  std::string candidateName;
  if (name.empty()) {
    do {
      candidateName
          = std::format("deformable_body_{:03d}", ++m_deformableBodyCounter);
    } while (hasEntityWithName<comps::DeformableBodyTag>(
        m_storage->registry, candidateName));
  } else {
    candidateName = std::string(name);
    DART_SIMULATION_THROW_T_IF(
        hasEntityWithName<comps::DeformableBodyTag>(
            m_storage->registry, candidateName),
        InvalidArgumentException,
        "DeformableBody '{}' already exists",
        candidateName);
  }

  auto entity = m_storage->registry.create();
  m_storage->registry.emplace<comps::Name>(entity, candidateName);
  m_storage->registry.emplace<comps::DeformableBodyTag>(entity);

  auto& state = m_storage->registry.emplace<comps::DeformableNodeState>(
      entity, allocator);
  state.positions = std::move(data.positions);
  state.previousPositions = state.positions;
  state.velocities = std::move(data.velocities);

  auto& nodeModel = m_storage->registry.emplace<comps::DeformableNodeModel>(
      entity, allocator);
  nodeModel.masses = std::move(data.masses);
  nodeModel.fixed = std::move(data.fixed);

  auto& model = m_storage->registry.emplace<comps::DeformableSpringModel>(
      entity, allocator);
  model.edges = std::move(data.edges);
  model.stiffness = data.stiffness;
  model.damping = data.damping;

  auto& topology = m_storage->registry.emplace<comps::DeformableMeshTopology>(
      entity, allocator);
  topology.restPositions = std::move(data.restPositions);
  topology.surfaceTriangles = std::move(data.surfaceTriangles);
  topology.tetrahedra = std::move(data.tetrahedra);
  topology.tetrahedronRestVolumes = std::move(data.tetrahedronRestVolumes);

  auto& material
      = m_storage->registry.emplace<comps::DeformableMaterial>(entity);
  material = data.material;

  if (!data.boundaryConditions.dirichlet.empty()
      || !data.boundaryConditions.neumann.empty()) {
    auto& boundaryConditions
        = m_storage->registry.emplace<comps::DeformableBoundaryConditions>(
            entity, allocator);
    boundaryConditions = std::move(data.boundaryConditions);
  }

  return DeformableBody(detail::fromRegistryEntity(entity), this);
}

//==============================================================================
std::optional<DeformableBody> World::getDeformableBody(std::string_view name)
{
  auto view = m_storage->registry.view<comps::DeformableBodyTag, comps::Name>();
  for (auto entity : view) {
    const auto& info = view.get<comps::Name>(entity);
    if (info.name == name) {
      return DeformableBody(detail::fromRegistryEntity(entity), this);
    }
  }
  return std::nullopt;
}

//==============================================================================
bool World::hasDeformableBody(std::string_view name) const
{
  return hasEntityWithName<comps::DeformableBodyTag>(m_storage->registry, name);
}

//==============================================================================
std::size_t World::getDeformableBodyCount() const
{
  return countEntities<comps::DeformableBodyTag>(m_storage->registry);
}

//==============================================================================
void World::configureDeformableSolver(
    std::string_view name, const DeformableSolverOptions& options)
{
  auto view = m_storage->registry.view<comps::DeformableBodyTag, comps::Name>();
  for (auto entity : view) {
    if (view.get<comps::Name>(entity).name != name) {
      continue;
    }
    // Translate the public, solver-agnostic options into the internal opt-in
    // inner-solver component. The mapping lives here (the World step pipeline
    // owns the concrete solver) so the public facade stays algorithm-neutral.
    m_storage->registry.emplace_or_replace<comps::DeformableVbdConfig>(
        entity,
        comps::DeformableVbdConfig{/*enabled=*/true,
                                   options.iterations,
                                   options.convergenceTolerance,
                                   options.useAcceleration,
                                   options.accelerationSpectralRadius,
                                   options.stiffnessDamping,
                                   options.groundContactStiffness});
    return;
  }
  DART_SIMULATION_THROW_T(
      InvalidArgumentException,
      "configureDeformableSolver: no deformable body named '{}'",
      name);
}

//==============================================================================
void World::enterSimulationMode()
{
  DART_SIMULATION_THROW_T_IF(
      m_simulationMode,
      InvalidArgumentException,
      "World is already in simulation mode");

  validateLoopClosureKinematicsPolicySupport(*this);
  validateRigidBodyJointPipelineSupport(*this, m_rigidBodySolver);
  validateArticulatedPointJointPipelineSupport(
      *this,
      m_multibodyIntegrationMethod == MultibodyIntegrationMethod::Variational);
  m_simulationMode = true;

  // Initial bake so that cached transforms are up-to-date.
  updateKinematics();
  detail::deformable_vbd::configureAvbdRigidWorldPointJointsFromCurrentPoses(
      m_storage->registry);
  prepareStepPipelineCacheForCurrentConfiguration();
}

//==============================================================================
void World::setGravity(const Eigen::Vector3d& gravity)
{
  DART_SIMULATION_THROW_T_IF(
      !gravity.array().isFinite().all(),
      InvalidArgumentException,
      "Gravity must contain only finite coordinates");

  m_gravity = gravity;
}

//==============================================================================
const Eigen::Vector3d& World::getGravity() const noexcept
{
  return m_gravity;
}

//==============================================================================
void World::setRigidBodySolver(RigidBodySolver solver)
{
  DART_SIMULATION_THROW_T_IF(
      !isValidRigidBodySolver(solver),
      InvalidArgumentException,
      "Rigid-body solver is invalid");

  if (m_rigidBodySolver == solver) {
    return;
  }

  validateRigidBodyJointPipelineSupport(*this, solver);
  m_rigidBodySolver = solver;
  if (m_simulationMode) {
    prepareStepPipelineCacheForCurrentConfiguration();
  }
}

//==============================================================================
RigidBodySolver World::getRigidBodySolver() const noexcept
{
  return m_rigidBodySolver;
}

//==============================================================================
void World::setTimeStep(double timeStep)
{
  DART_SIMULATION_THROW_T_IF(
      !std::isfinite(timeStep) || timeStep <= 0.0,
      InvalidArgumentException,
      "Time step must be positive and finite");

  m_timeStep = timeStep;
}

//==============================================================================
double World::getTimeStep() const noexcept
{
  return m_timeStep;
}

//==============================================================================
bool World::isDifferentiable() const noexcept
{
  return m_differentiable;
}

//==============================================================================
ContactSolverMethod World::getContactSolverMethod() const noexcept
{
  return m_contactSolverMethod;
}

//==============================================================================
void World::setContactSolverMethod(ContactSolverMethod method)
{
  DART_SIMULATION_THROW_T_IF(
      !isValidContactSolverMethod(method),
      InvalidArgumentException,
      "Contact solver method is invalid");

  if (m_contactSolverMethod == method) {
    return;
  }

  m_contactSolverMethod = method;
  if (m_simulationMode) {
    prepareStepPipelineCacheForCurrentConfiguration();
  }
}

//==============================================================================
ContactGradientMode World::getContactGradientMode() const noexcept
{
  return m_contactGradientMode;
}

//==============================================================================
void World::setContactGradientMode(ContactGradientMode mode)
{
  DART_SIMULATION_THROW_T_IF(
      !isValidContactGradientMode(mode),
      InvalidArgumentException,
      "Contact gradient mode is invalid");

  m_contactGradientMode = mode;
}

//==============================================================================
void World::setComputeAcceleratorPolicy(ComputeAcceleratorPolicy policy)
{
  DART_SIMULATION_THROW_T_IF(
      !isValidComputeAcceleratorPolicy(policy),
      InvalidArgumentException,
      "Compute accelerator policy is invalid");

  m_computeAcceleratorPolicy = policy;
  if (m_simulationMode) {
    prepareStepPipelineCacheForCurrentConfiguration();
  }
}

//==============================================================================
ComputeAcceleratorPolicy World::getComputeAcceleratorPolicy() const noexcept
{
  return m_computeAcceleratorPolicy;
}

//==============================================================================
void World::setDeactivationOptions(const DeactivationOptions& options)
{
  validateDeactivationOptions(options);
  m_deactivationOptions = options;
  if (!m_deactivationOptions.enabled) {
    clearDeactivationState();
  }
}

//==============================================================================
const DeactivationOptions& World::getDeactivationOptions() const noexcept
{
  return m_deactivationOptions;
}

//==============================================================================
bool World::isDeactivationEnabled() const noexcept
{
  return m_deactivationOptions.enabled;
}

//==============================================================================
bool World::isDeactivationActiveForStep() const noexcept
{
  if (!m_deactivationOptions.enabled || m_differentiable) {
    return false;
  }

  return m_rigidBodySolver == RigidBodySolver::SequentialImpulse
         || m_multibodyIntegrationMethod
                == MultibodyIntegrationMethod::SemiImplicit;
}

//==============================================================================
bool World::isDeactivationEntitySleeping(Entity entity) const
{
  if (!isDeactivationActiveForStep()) {
    return false;
  }

  const auto registryEntity = detail::toRegistryEntity(entity);
  if (!m_storage->registry.valid(registryEntity)) {
    return false;
  }
  if (m_storage->registry.all_of<comps::RigidBodyTag>(registryEntity)) {
    if (m_rigidBodySolver != RigidBodySolver::SequentialImpulse
        || !isDynamicRigidDeactivationEntity(
            m_storage->registry, registryEntity)) {
      return false;
    }
  } else if (
      m_storage->registry.all_of<comps::MultibodyStructure>(registryEntity)) {
    if (m_multibodyIntegrationMethod
        != MultibodyIntegrationMethod::SemiImplicit) {
      return false;
    }
  } else {
    return false;
  }
  const auto* state
      = m_storage->registry.try_get<comps::DeactivationState>(registryEntity);
  return state != nullptr && state->sleeping;
}

//==============================================================================
int World::getDeactivationGroupIndex(Entity entity) const
{
  if (!isDeactivationActiveForStep()) {
    return -1;
  }

  const auto registryEntity = detail::toRegistryEntity(entity);
  if (!m_storage->registry.valid(registryEntity)) {
    return -1;
  }
  if (m_storage->registry.all_of<comps::RigidBodyTag>(registryEntity)) {
    if (m_rigidBodySolver != RigidBodySolver::SequentialImpulse
        || !isDynamicRigidDeactivationEntity(
            m_storage->registry, registryEntity)) {
      return -1;
    }
  } else if (
      m_storage->registry.all_of<comps::MultibodyStructure>(registryEntity)) {
    if (m_multibodyIntegrationMethod
        != MultibodyIntegrationMethod::SemiImplicit) {
      return -1;
    }
  } else {
    return -1;
  }
  const auto* state
      = m_storage->registry.try_get<comps::DeactivationState>(registryEntity);
  return state != nullptr ? state->groupIndex : -1;
}

//==============================================================================
void World::wakeDeactivationEntity(Entity entity)
{
  const auto registryEntity = detail::toRegistryEntity(entity);
  if (!m_storage->registry.valid(registryEntity)) {
    return;
  }
  auto* state
      = m_storage->registry.try_get<comps::DeactivationState>(registryEntity);
  if (state == nullptr) {
    return;
  }

  state->sleeping = false;
  state->sleepCandidate = false;
  state->quietTime = 0.0;
  state->smoothedLinearSpeed = 0.0;
  state->smoothedAngularSpeed = 0.0;
  state->smoothedGeneralizedSpeed = 0.0;
  state->groupIndex = -1;
}

//==============================================================================
void World::clearDeactivationState()
{
  std::vector<entt::entity> entities;
  auto view = m_storage->registry.view<comps::DeactivationState>();
  entities.reserve(countReplayView(view));
  for (auto entity : view) {
    entities.push_back(entity);
  }
  for (auto entity : entities) {
    m_storage->registry.remove<comps::DeactivationState>(entity);
  }
}

//==============================================================================
void World::prepareDeactivationForStep()
{
  if (!m_deactivationOptions.enabled || !isDeactivationActiveForStep()) {
    clearDeactivationState();
    return;
  }

  auto& registry = m_storage->registry;
  const bool rigidSupported
      = m_rigidBodySolver == RigidBodySolver::SequentialImpulse;
  const bool multibodySupported = m_multibodyIntegrationMethod
                                  == MultibodyIntegrationMethod::SemiImplicit;
  const double disturbanceThresholdSquared
      = m_deactivationOptions.disturbanceForceThreshold
        * m_deactivationOptions.disturbanceForceThreshold;

  std::vector<entt::entity> stale;
  auto stateView = registry.view<comps::DeactivationState>();
  stale.reserve(countReplayView(stateView));
  for (auto entity : stateView) {
    const bool keepRigid
        = rigidSupported && isDynamicRigidDeactivationEntity(registry, entity);
    const bool keepMultibody
        = multibodySupported
          && registry.all_of<comps::MultibodyStructure>(entity);
    if (!keepRigid && !keepMultibody) {
      stale.push_back(entity);
    }
  }
  for (auto entity : stale) {
    registry.remove<comps::DeactivationState>(entity);
  }

  if (rigidSupported) {
    auto rigidView = registry.view<comps::RigidBodyTag, comps::Velocity>();
    for (auto entity : rigidView) {
      if (!isDynamicRigidDeactivationEntity(registry, entity)) {
        continue;
      }

      auto& state = registry.get_or_emplace<comps::DeactivationState>(entity);
      if (const auto* force = registry.try_get<comps::Force>(entity)) {
        const bool disturbed
            = force->force.squaredNorm() > disturbanceThresholdSquared
              || force->torque.squaredNorm() > disturbanceThresholdSquared;
        if (disturbed) {
          wakeDeactivationEntity(detail::fromRegistryEntity(entity));
        }
      }
      (void)state;
    }
  }

  if (multibodySupported) {
    auto multibodyView = registry.view<comps::MultibodyStructure>();
    for (auto entity : multibodyView) {
      const auto& structure
          = multibodyView.get<comps::MultibodyStructure>(entity);
      auto& state = registry.get_or_emplace<comps::DeactivationState>(entity);
      bool disturbed = false;
      for (const auto jointEntity : structure.joints) {
        const auto* jointActuation
            = registry.try_get<comps::JointActuation>(jointEntity);
        if (jointActuation == nullptr) {
          continue;
        }
        disturbed = disturbed
                    || jointActuation->torque.squaredNorm()
                           > disturbanceThresholdSquared
                    || jointActuation->commandVelocity.squaredNorm()
                           > disturbanceThresholdSquared;
      }
      for (const auto linkEntity : structure.links) {
        const auto* linkControl
            = registry.try_get<comps::LinkControl>(linkEntity);
        if (linkControl == nullptr) {
          continue;
        }
        disturbed = disturbed
                    || linkControl->externalForce.squaredNorm()
                           > disturbanceThresholdSquared;
      }
      if (disturbed) {
        wakeDeactivationEntity(detail::fromRegistryEntity(entity));
      }
      (void)state;
    }
  }
}

//==============================================================================
std::vector<Contact> World::filterContactsForDeactivation(
    std::span<const Contact> contacts)
{
  if (!isDeactivationActiveForStep()) {
    return {contacts.begin(), contacts.end()};
  }

  auto& registry = m_storage->registry;
  const bool rigidSupported
      = m_rigidBodySolver == RigidBodySolver::SequentialImpulse;
  const bool multibodySupported = m_multibodyIntegrationMethod
                                  == MultibodyIntegrationMethod::SemiImplicit;

  std::vector<Contact> activeContacts;
  activeContacts.reserve(contacts.size());
  for (const auto& contact : contacts) {
    entt::entity entityA = deactivationEntityForContactBody(
        registry, contact.bodyA, rigidSupported, multibodySupported);
    entt::entity entityB = deactivationEntityForContactBody(
        registry, contact.bodyB, rigidSupported, multibodySupported);

    const auto isSleeping = [&](entt::entity entity) {
      if (entity == entt::null) {
        return false;
      }
      const auto* state = registry.try_get<comps::DeactivationState>(entity);
      return state != nullptr && state->sleeping;
    };

    bool sleepingA = isSleeping(entityA);
    bool sleepingB = isSleeping(entityB);
    const bool dynamicA = entityA != entt::null;
    const bool dynamicB = entityB != entt::null;

    if (sleepingA && dynamicB && !sleepingB) {
      wakeDeactivationEntity(detail::fromRegistryEntity(entityA));
      sleepingA = false;
    }
    if (sleepingB && dynamicA && !sleepingA) {
      wakeDeactivationEntity(detail::fromRegistryEntity(entityB));
      sleepingB = false;
    }

    const bool inactiveA = !dynamicA || sleepingA;
    const bool inactiveB = !dynamicB || sleepingB;
    if (inactiveA && inactiveB) {
      continue;
    }

    activeContacts.push_back(contact);
  }

  return activeContacts;
}

//==============================================================================
void World::updateDeactivationAfterStep()
{
  if (!m_deactivationOptions.enabled || !isDeactivationActiveForStep()) {
    clearDeactivationState();
    return;
  }

  auto& registry = m_storage->registry;
  const bool rigidSupported
      = m_rigidBodySolver == RigidBodySolver::SequentialImpulse;
  const bool multibodySupported = m_multibodyIntegrationMethod
                                  == MultibodyIntegrationMethod::SemiImplicit;
  const double disturbanceThresholdSquared
      = m_deactivationOptions.disturbanceForceThreshold
        * m_deactivationOptions.disturbanceForceThreshold;

  std::vector<entt::entity> participants;
  if (rigidSupported) {
    auto rigidView = registry.view<comps::RigidBodyTag, comps::Velocity>();
    for (auto entity : rigidView) {
      if (!isDynamicRigidDeactivationEntity(registry, entity)) {
        continue;
      }
      (void)registry.get_or_emplace<comps::DeactivationState>(entity);
      participants.push_back(entity);
    }
  }
  if (multibodySupported) {
    auto multibodyView = registry.view<comps::MultibodyStructure>();
    for (auto entity : multibodyView) {
      (void)registry.get_or_emplace<comps::DeactivationState>(entity);
      participants.push_back(entity);
    }
  }
  if (participants.empty()) {
    return;
  }

  std::vector<std::size_t> parent(participants.size());
  for (std::size_t i = 0; i < parent.size(); ++i) {
    parent[i] = i;
  }
  const auto participantIndex = [&](entt::entity entity) {
    for (std::size_t i = 0; i < participants.size(); ++i) {
      if (participants[i] == entity) {
        return i;
      }
    }
    return participants.size();
  };
  const auto findRoot = [&](std::size_t index) {
    while (parent[index] != index) {
      parent[index] = parent[parent[index]];
      index = parent[index];
    }
    return index;
  };
  const auto unite = [&](std::size_t a, std::size_t b) {
    auto rootA = findRoot(a);
    auto rootB = findRoot(b);
    if (rootA != rootB) {
      parent[rootB] = rootA;
    }
  };

  const auto& contacts = queryContacts(CollisionQueryOptions{});
  for (const auto& contact : contacts) {
    const auto entityA = deactivationEntityForContactBody(
        registry, contact.bodyA, rigidSupported, multibodySupported);
    const auto entityB = deactivationEntityForContactBody(
        registry, contact.bodyB, rigidSupported, multibodySupported);
    if (entityA == entt::null || entityB == entt::null) {
      continue;
    }
    const auto indexA = participantIndex(entityA);
    const auto indexB = participantIndex(entityB);
    if (indexA < participants.size() && indexB < participants.size()) {
      unite(indexA, indexB);
    }
  }

  // A contact island may sleep only once its penetration correction has
  // essentially converged. Mirrors DART 6 (#2920): keep an island awake while
  // any of its contacts still overlaps beyond this tolerance so bodies are
  // never frozen mid-interpenetration, including penetration against static
  // geometry.
  constexpr double kSleepContactPenetrationTolerance = 1e-3;
  std::vector<bool> rootPenetrationBlocked(participants.size(), false);
  for (const auto& contact : contacts) {
    if (contact.depth <= kSleepContactPenetrationTolerance) {
      continue;
    }
    const auto entityA = deactivationEntityForContactBody(
        registry, contact.bodyA, rigidSupported, multibodySupported);
    const auto entityB = deactivationEntityForContactBody(
        registry, contact.bodyB, rigidSupported, multibodySupported);
    for (const auto entity : {entityA, entityB}) {
      if (entity == entt::null) {
        continue;
      }
      const auto index = participantIndex(entity);
      if (index < participants.size()) {
        rootPenetrationBlocked[findRoot(index)] = true;
      }
    }
  }

  std::map<std::size_t, int> groupIndices;
  int nextGroupIndex = 0;
  std::vector<bool> ready(participants.size(), false);

  for (std::size_t i = 0; i < participants.size(); ++i) {
    const auto entity = participants[i];
    auto& state = registry.get<comps::DeactivationState>(entity);
    const auto root = findRoot(i);
    auto [it, inserted] = groupIndices.emplace(root, nextGroupIndex);
    if (inserted) {
      ++nextGroupIndex;
    }
    state.groupIndex = it->second;

    bool disturbed = false;
    double linearSpeed = 0.0;
    double angularSpeed = 0.0;
    double generalizedSpeed = 0.0;
    if (registry.all_of<comps::RigidBodyTag>(entity)) {
      const auto& velocity = registry.get<comps::Velocity>(entity);
      linearSpeed = velocity.linear.norm();
      angularSpeed = velocity.angular.norm();
      if (const auto* force = registry.try_get<comps::Force>(entity)) {
        disturbed
            = force->force.squaredNorm() > disturbanceThresholdSquared
              || force->torque.squaredNorm() > disturbanceThresholdSquared;
      }
    } else if (
        const auto* structure
        = registry.try_get<comps::MultibodyStructure>(entity)) {
      for (const auto jointEntity : structure->joints) {
        const auto* jointState
            = registry.try_get<comps::JointState>(jointEntity);
        if (jointState != nullptr && jointState->velocity.size() > 0) {
          generalizedSpeed = std::max(
              generalizedSpeed, jointState->velocity.cwiseAbs().maxCoeff());
        }
        const auto* jointActuation
            = registry.try_get<comps::JointActuation>(jointEntity);
        if (jointActuation != nullptr) {
          disturbed = disturbed
                      || jointActuation->torque.squaredNorm()
                             > disturbanceThresholdSquared
                      || jointActuation->commandVelocity.squaredNorm()
                             > disturbanceThresholdSquared;
        }
      }
      for (const auto linkEntity : structure->links) {
        const auto* linkControl
            = registry.try_get<comps::LinkControl>(linkEntity);
        if (linkControl == nullptr) {
          continue;
        }
        disturbed = disturbed
                    || linkControl->externalForce.squaredNorm()
                           > disturbanceThresholdSquared;
      }
    }

    if (disturbed) {
      wakeDeactivationEntity(detail::fromRegistryEntity(entity));
      state.groupIndex = it->second;
      continue;
    }

    constexpr double alpha = 0.2;
    state.smoothedLinearSpeed
        = alpha * linearSpeed + (1.0 - alpha) * state.smoothedLinearSpeed;
    state.smoothedAngularSpeed
        = alpha * angularSpeed + (1.0 - alpha) * state.smoothedAngularSpeed;
    state.smoothedGeneralizedSpeed
        = alpha * generalizedSpeed
          + (1.0 - alpha) * state.smoothedGeneralizedSpeed;

    const bool overWakeBand
        = state.smoothedLinearSpeed
              > m_deactivationOptions.linearSpeedThreshold
                    * m_deactivationOptions.wakeThresholdScale
          || state.smoothedAngularSpeed
                 > m_deactivationOptions.angularSpeedThreshold
                       * m_deactivationOptions.wakeThresholdScale
          || state.smoothedGeneralizedSpeed
                 > m_deactivationOptions.generalizedSpeedThreshold
                       * m_deactivationOptions.wakeThresholdScale;
    if (state.sleeping && overWakeBand) {
      wakeDeactivationEntity(detail::fromRegistryEntity(entity));
      state.groupIndex = it->second;
      continue;
    }
    if (state.sleeping) {
      ready[i] = true;
      continue;
    }

    const bool belowSleepBand
        = state.smoothedLinearSpeed
              <= m_deactivationOptions.linearSpeedThreshold
          && state.smoothedAngularSpeed
                 <= m_deactivationOptions.angularSpeedThreshold
          && state.smoothedGeneralizedSpeed
                 <= m_deactivationOptions.generalizedSpeedThreshold;
    if (!belowSleepBand) {
      state.sleepCandidate = false;
      state.quietTime = 0.0;
      continue;
    }

    state.quietTime += m_timeStep;
    state.sleepCandidate
        = state.quietTime >= m_deactivationOptions.timeUntilSleep;
    ready[i] = state.sleepCandidate;
  }

  for (const auto& [root, groupIndex] : groupIndices) {
    bool groupReady = !rootPenetrationBlocked[root];
    std::vector<entt::entity> groupMembers;
    for (std::size_t i = 0; i < participants.size(); ++i) {
      if (findRoot(i) != root) {
        continue;
      }
      groupMembers.push_back(participants[i]);
      const auto& state
          = registry.get<comps::DeactivationState>(participants[i]);
      groupReady = groupReady && (state.sleeping || ready[i]);
    }

    if (!groupReady) {
      for (const auto entity : groupMembers) {
        auto& state = registry.get<comps::DeactivationState>(entity);
        if (state.sleeping && groupMembers.size() > 1u) {
          wakeDeactivationEntity(detail::fromRegistryEntity(entity));
        }
        state.groupIndex = groupIndex;
      }
      continue;
    }

    for (const auto entity : groupMembers) {
      auto& state = registry.get<comps::DeactivationState>(entity);
      state.sleeping = true;
      state.sleepCandidate = true;
      state.groupIndex = groupIndex;
      if (auto* velocity = registry.try_get<comps::Velocity>(entity)) {
        velocity->linear.setZero();
        velocity->angular.setZero();
      }
      if (const auto* structure
          = registry.try_get<comps::MultibodyStructure>(entity)) {
        for (const auto jointEntity : structure->joints) {
          if (auto* jointState
              = registry.try_get<comps::JointState>(jointEntity)) {
            jointState->velocity.setZero();
            jointState->acceleration.setZero();
          }
        }
      }
    }
  }
}

//==============================================================================
StepDerivatives World::getStepDerivatives() const
{
  DART_SIMULATION_THROW_T_IF(
      !m_differentiable,
      InvalidOperationException,
      "World::getStepDerivatives() requires a differentiable World (construct "
      "with WorldOptions::differentiable set to true)");

#ifdef DART_HAS_DIFF
  DART_SIMULATION_THROW_T_IF(
      !m_storage->stepDerivatives.has_value(),
      InvalidOperationException,
      "World::getStepDerivatives() has no derivatives yet; call step() first");
  return *m_storage->stepDerivatives;
#else
  DART_SIMULATION_THROW_T(
      InvalidOperationException,
      "World::getStepDerivatives() requires differentiable support to be built "
      "(enable the DART_BUILD_DIFF CMake option)");
#endif
}

//==============================================================================
StepGradient World::applyStepVjp(const Eigen::VectorXd& dLossDNextState) const
{
  // Reuse getStepDerivatives() for the not-differentiable / not-built /
  // no-derivatives-yet gating so applyStepVjp throws the identical errors. When
  // differentiable support is not compiled this call already throws, so the
  // explicit VJP below is only reached with valid cached Jacobians.
  const StepDerivatives derivatives = getStepDerivatives();

  DART_SIMULATION_THROW_T_IF(
      dLossDNextState.size() != derivatives.stateJacobian.rows(),
      InvalidArgumentException,
      "World::applyStepVjp(): dLossDNextState has size {} but the cached step "
      "Jacobian expects next-state size {}",
      dLossDNextState.size(),
      derivatives.stateJacobian.rows());

  StepGradient gradient;
  gradient.state = derivatives.stateJacobian.transpose() * dLossDNextState;
  gradient.control = derivatives.controlJacobian.transpose() * dLossDNextState;
  return gradient;
}

//==============================================================================
void World::addDifferentiableParameter(
    const PhysicalParameterSelector& selector)
{
  DART_SIMULATION_THROW_T_IF(
      !m_differentiable,
      InvalidOperationException,
      "World::addDifferentiableParameter() requires a differentiable World "
      "(construct with WorldOptions::differentiable set to true)");

#ifndef DART_HAS_DIFF
  (void)selector;
  DART_SIMULATION_THROW_T(
      InvalidOperationException,
      "World::addDifferentiableParameter() requires differentiable support to "
      "be built (enable the DART_BUILD_DIFF CMake option)");
#else
  DART_SIMULATION_THROW_T_IF(
      !selector.body.isValid() || selector.body.getWorld() != this,
      InvalidArgumentException,
      "World::addDifferentiableParameter(): the selector's body does not "
      "belong to this World");

  const auto entity = detail::toRegistryEntity(selector.body.getEntity());
  DART_SIMULATION_THROW_T_IF(
      !m_storage->registry.all_of<comps::RigidBodyTag>(entity),
      InvalidArgumentException,
      "World::addDifferentiableParameter(): the selector's body is not a valid "
      "rigid body");

  // Supported parameters (PLAN-110 WS4): MASS (1 column), INERTIA (3 diagonal
  // principal-moment columns), and FRICTION (1 column). CENTER_OF_MASS stays
  // unsupported: the rigid-body forward step assumes the center of mass at the
  // body origin (`MassProperties::localCenterOfMass` is unused outside the
  // multibody path), so its single-step gradient is identically zero — there is
  // no meaningful Jacobian to assemble. Reject it with a clear
  // NotImplementedException rather than producing a vacuous zero column.
  DART_SIMULATION_THROW_T_IF(
      selector.parameter == PhysicalParameter::CENTER_OF_MASS,
      NotImplementedException,
      "World::addDifferentiableParameter(): PhysicalParameter::CENTER_OF_MASS "
      "is not supported for rigid bodies; the rigid-body step assumes the "
      "center of mass at the body origin, so the gradient is identically zero. "
      "Supported: MASS, INERTIA, FRICTION");

  m_storage->differentiableParameters.emplace_back(entity, selector.parameter);
#endif
}

//==============================================================================
void World::addDifferentiableParameter(
    const RigidBody& body, PhysicalParameter parameter)
{
  addDifferentiableParameter(PhysicalParameterSelector(body, parameter));
}

//==============================================================================
std::size_t World::getNumDifferentiableParameters() const noexcept
{
  return m_storage->differentiableParameters.size();
}

//==============================================================================
std::size_t World::getNumDofs() const
{
  return 3
         * detail::ensureBakedWorldModelCurrent(*this)
               .dynamicRigidBodyEntities.size();
}

//==============================================================================
std::size_t World::getNumEfforts() const
{
  return getNumDofs();
}

//==============================================================================
Eigen::VectorXd World::getStateVector() const
{
  const auto& bodies
      = detail::ensureBakedWorldModelCurrent(*this).dynamicRigidBodyEntities;
  const Eigen::Index dofs = 3 * static_cast<Eigen::Index>(bodies.size());
  Eigen::VectorXd state(2 * dofs);
  for (std::size_t k = 0; k < bodies.size(); ++k) {
    const auto base = 3 * static_cast<Eigen::Index>(k);
    const auto& transform
        = m_storage->registry.get<comps::Transform>(bodies[k]);
    const auto& velocity = m_storage->registry.get<comps::Velocity>(bodies[k]);
    state.segment<3>(base) = transform.position;
    state.segment<3>(dofs + base) = velocity.linear;
  }
  return state;
}

//==============================================================================
void World::setStateVector(const Eigen::VectorXd& state)
{
  const auto& bodies
      = detail::ensureBakedWorldModelCurrent(*this).dynamicRigidBodyEntities;
  const Eigen::Index dofs = 3 * static_cast<Eigen::Index>(bodies.size());
  DART_SIMULATION_THROW_T_IF(
      state.size() != 2 * dofs,
      InvalidArgumentException,
      "World::setStateVector(): expected size {} (= 2 * num_dofs) but got {}",
      2 * dofs,
      state.size());
  for (std::size_t k = 0; k < bodies.size(); ++k) {
    const auto base = 3 * static_cast<Eigen::Index>(k);
    auto& transform = m_storage->registry.get<comps::Transform>(bodies[k]);
    auto& velocity = m_storage->registry.get<comps::Velocity>(bodies[k]);
    transform.position = state.segment<3>(base);
    velocity.linear = state.segment<3>(dofs + base);
  }
}

//==============================================================================
Eigen::VectorXd World::getControlVector() const
{
  const auto& bodies
      = detail::ensureBakedWorldModelCurrent(*this).dynamicRigidBodyEntities;
  const Eigen::Index dofs = 3 * static_cast<Eigen::Index>(bodies.size());
  Eigen::VectorXd control(dofs);
  for (std::size_t k = 0; k < bodies.size(); ++k) {
    const auto base = 3 * static_cast<Eigen::Index>(k);
    const auto& force = m_storage->registry.get<comps::Force>(bodies[k]);
    control.segment<3>(base) = force.force;
  }
  return control;
}

//==============================================================================
void World::setControlVector(const Eigen::VectorXd& control)
{
  const auto& bodies
      = detail::ensureBakedWorldModelCurrent(*this).dynamicRigidBodyEntities;
  const Eigen::Index dofs = 3 * static_cast<Eigen::Index>(bodies.size());
  DART_SIMULATION_THROW_T_IF(
      control.size() != dofs,
      InvalidArgumentException,
      "World::setControlVector(): expected size {} (= num_efforts) but got {}",
      dofs,
      control.size());
  for (std::size_t k = 0; k < bodies.size(); ++k) {
    const auto base = 3 * static_cast<Eigen::Index>(k);
    auto& force = m_storage->registry.get<comps::Force>(bodies[k]);
    force.force = control.segment<3>(base);
  }
}

//==============================================================================
void World::setTime(double time)
{
  DART_SIMULATION_THROW_T_IF(
      !std::isfinite(time) || time < 0.0,
      InvalidArgumentException,
      "Time must be non-negative and finite");

  m_time = time;
}

//==============================================================================
double World::getTime() const noexcept
{
  return m_time;
}

//==============================================================================
std::size_t World::getFrame() const noexcept
{
  return m_frame;
}

//==============================================================================
void World::sync(WorldSyncStage stage)
{
  compute::SequentialExecutor executor;
  sync(stage, executor);
}

//==============================================================================
void World::sync(WorldSyncStage stage, compute::ComputeExecutor& executor)
{
  DART_SIMULATION_THROW_T_IF(
      !m_simulationMode,
      InvalidArgumentException,
      "World::sync() requires simulation mode");

  DART_SIMULATION_THROW_T_IF(
      !isValidWorldSyncStage(stage),
      InvalidArgumentException,
      "World::sync() stage is invalid");

  switch (stage) {
    case WorldSyncStage::Kinematics:
      validateLoopClosureKinematicsPolicySupport(*this);
      executeKinematicsGraph(*this, executor);
      return;
  }
}

//==============================================================================
void World::updateKinematics()
{
  sync(WorldSyncStage::Kinematics);
}

//==============================================================================
void World::updateKinematics(compute::ComputeExecutor& executor)
{
  sync(WorldSyncStage::Kinematics, executor);
}

//==============================================================================
void World::resetFrameScratchForStep()
{
  m_memoryManager.getFrameAllocator().reset();
  ++m_memoryDiagnostics.frameScratchResetCount;
  refreshMemoryDiagnostics();
}

//==============================================================================
void World::refreshMemoryDiagnostics()
{
  m_memoryDiagnostics.allocatorDebugDiagnostics
      = m_memoryManager.getDebugDiagnostics();
  const auto& frameAllocator = m_memoryManager.getFrameAllocator();
  const auto overflowBytes = frameAllocator.overflowBytes();
  m_memoryDiagnostics.frameScratchCapacityBytes
      = frameAllocator.usableCapacity();
  m_memoryDiagnostics.frameScratchUsedBytes
      = frameAllocator.used() + overflowBytes;
  m_memoryDiagnostics.frameScratchOverflowCount
      = frameAllocator.overflowCount();
  m_memoryDiagnostics.frameScratchOverflowBytes = overflowBytes;
  m_memoryDiagnostics.frameScratchPeakUsedBytes = std::max(
      m_memoryDiagnostics.frameScratchPeakUsedBytes,
      m_memoryDiagnostics.frameScratchUsedBytes);
}

//==============================================================================
void World::step()
{
#if DART_BUILD_PROFILE
  if (!m_stepProfilingEnabled && tryStepCleanNoWorkDefaultPipeline()) {
    return;
  }
#else
  if (tryStepCleanNoWorkDefaultPipeline()) {
    return;
  }
#endif

  compute::SequentialExecutor executor;
  step(executor);
}

//==============================================================================
class ScopedWorldStepPipelineClear
{
public:
  explicit ScopedWorldStepPipelineClear(compute::WorldStepPipeline& pipeline)
    : m_pipeline(pipeline)
  {
    // Empty.
  }

  ~ScopedWorldStepPipelineClear() noexcept
  {
    m_pipeline.clear();
  }

  ScopedWorldStepPipelineClear(const ScopedWorldStepPipelineClear&) = delete;
  ScopedWorldStepPipelineClear& operator=(const ScopedWorldStepPipelineClear&)
      = delete;

private:
  compute::WorldStepPipeline& m_pipeline;
};

//==============================================================================
void World::step(std::size_t count)
{
  compute::SequentialExecutor executor;
  step(count, executor);
}

//==============================================================================
void World::setMultibodyOptions(const MultibodyOptions& options)
{
  DART_SIMULATION_THROW_T_IF(
      !isValidMultibodyIntegrationFamily(options.integrationFamily),
      InvalidArgumentException,
      "MultibodyOptions.integrationFamily is invalid");
  MultibodyIntegrationMethod method = MultibodyIntegrationMethod::SemiImplicit;
  switch (options.integrationFamily) {
    case MultibodyIntegrationFamily::SemiImplicit:
      method = MultibodyIntegrationMethod::SemiImplicit;
      break;
    case MultibodyIntegrationFamily::Variational:
      method = MultibodyIntegrationMethod::Variational;
      break;
  }
  DART_SIMULATION_THROW_T_IF(
      options.variationalMaxIterations == 0,
      InvalidArgumentException,
      "MultibodyOptions.variationalMaxIterations must be positive");
  DART_SIMULATION_THROW_T_IF(
      options.variationalMaxIterations
          > static_cast<std::size_t>(std::numeric_limits<int>::max()),
      InvalidArgumentException,
      "MultibodyOptions.variationalMaxIterations must fit in int");
  DART_SIMULATION_THROW_T_IF(
      !std::isfinite(options.variationalTolerance)
          || options.variationalTolerance <= 0.0,
      InvalidArgumentException,
      "MultibodyOptions.variationalTolerance must be positive and finite");

  m_multibodyIntegrationMethod = method;
  m_variationalIntegratorMaxIterations = options.variationalMaxIterations;
  m_variationalIntegratorTolerance = options.variationalTolerance;

  if (m_simulationMode) {
    prepareStepPipelineCacheForCurrentConfiguration();
  }
}

//==============================================================================
MultibodyOptions World::getMultibodyOptions() const
{
  MultibodyOptions options;
  options.integrationFamily
      = m_multibodyIntegrationMethod == MultibodyIntegrationMethod::Variational
            ? MultibodyIntegrationFamily::Variational
            : MultibodyIntegrationFamily::SemiImplicit;
  options.variationalMaxIterations = m_variationalIntegratorMaxIterations;
  options.variationalTolerance = m_variationalIntegratorTolerance;
  return options;
}

//==============================================================================
bool World::tryStepCleanNoWorkDefaultPipeline()
{
  if (!m_simulationMode) {
    enterSimulationMode();
  }

  if (m_differentiable
#if DART_BUILD_PROFILE
      || m_stepProfilingEnabled
#endif
      || !m_stepPipelineCache->canSkipDefaultPipelineWhenFramesClean
      || hasDirtyFrameCaches(*this)) {
    return false;
  }

  m_memoryManager.getFrameAllocator().reset();
  ++m_memoryDiagnostics.frameScratchResetCount;
  m_memoryDiagnostics.frameScratchUsedBytes = 0;
  m_memoryDiagnostics.frameScratchOverflowCount = 0;
  m_memoryDiagnostics.frameScratchOverflowBytes = 0;
  m_lastDeformableSolverDiagnostics = {};
  m_time += m_timeStep;
  ++m_frame;
  if (m_replay && m_replay->recordingEnabled) {
    recordReplayFrame();
  }
  return true;
}

//==============================================================================
void World::step(compute::ComputeExecutor& executor)
{
#if DART_BUILD_PROFILE
  StepProfileTimer profileTimer(m_stepProfilingEnabled);
#endif

  if (tryStepCleanNoWorkDefaultPipeline()) {
#if DART_BUILD_PROFILE
    profileTimer.finish(m_lastStepProfile);
#endif
    return;
  }

  auto& stages = m_stepPipelineCache->stages;
  compute::WorldStepPipeline& pipeline = stages.buildDefault(
      m_rigidBodySolver,
      m_multibodyIntegrationMethod == MultibodyIntegrationMethod::Variational,
      m_stepPipelineCache->hasAdvanceableRigidBodies,
      m_stepPipelineCache->hasMultibodyStructure,
      m_stepPipelineCache->hasDeformableBodies);

  stepPipelineOnce(executor, pipeline);
  m_lastDeformableSolverDiagnostics = makeDeformableSolverDiagnostics(
      stages.deformableDynamics.getLastStats());
  recordReplayFrame();
#if DART_BUILD_PROFILE
  profileTimer.finish(m_lastStepProfile);
#endif
}

//==============================================================================
void World::step(std::size_t count, compute::ComputeExecutor& executor)
{
  auto& stages = m_stepPipelineCache->stages;
  compute::WorldStepPipeline* pipeline = nullptr;
  for (std::size_t i = 0; i < count; ++i) {
#if DART_BUILD_PROFILE
    StepProfileTimer profileTimer(m_stepProfilingEnabled);
#endif

    if (tryStepCleanNoWorkDefaultPipeline()) {
#if DART_BUILD_PROFILE
      profileTimer.finish(m_lastStepProfile);
#endif
      continue;
    }

    if (pipeline == nullptr) {
      pipeline = &stages.buildDefault(
          m_rigidBodySolver,
          m_multibodyIntegrationMethod
              == MultibodyIntegrationMethod::Variational,
          m_stepPipelineCache->hasAdvanceableRigidBodies,
          m_stepPipelineCache->hasMultibodyStructure,
          m_stepPipelineCache->hasDeformableBodies);
    }
    stepPipelineOnce(executor, *pipeline);
    m_lastDeformableSolverDiagnostics = makeDeformableSolverDiagnostics(
        stages.deformableDynamics.getLastStats());
    recordReplayFrame();
#if DART_BUILD_PROFILE
    profileTimer.finish(m_lastStepProfile);
#endif
  }
}

//==============================================================================
void World::step(
    compute::ComputeExecutor& executor, compute::WorldStepStage& stage)
{
  auto& stages = m_stepPipelineCache->stages;
  compute::WorldStepPipeline& pipeline = stages.buildWithFinalStage(
      m_rigidBodySolver,
      m_multibodyIntegrationMethod == MultibodyIntegrationMethod::Variational,
      hasAdvanceableRigidBodyStructures(*this),
      hasMultibodyStructures(*this),
      hasDeformableBodies(*this),
      stage);
  ScopedWorldStepPipelineClear clearCustomPipeline(pipeline);

#if DART_BUILD_PROFILE
  StepProfileTimer profileTimer(m_stepProfilingEnabled);
#endif

  stepPipelineOnce(executor, pipeline);
  m_lastDeformableSolverDiagnostics = makeDeformableSolverDiagnostics(
      stages.deformableDynamics.getLastStats());
  recordReplayFrame();
#if DART_BUILD_PROFILE
  profileTimer.finish(m_lastStepProfile);
#endif
}

//==============================================================================
void World::step(
    std::size_t count,
    compute::ComputeExecutor& executor,
    compute::WorldStepStage& stage)
{
  auto& stages = m_stepPipelineCache->stages;
  compute::WorldStepPipeline& pipeline = stages.buildWithFinalStage(
      m_rigidBodySolver,
      m_multibodyIntegrationMethod == MultibodyIntegrationMethod::Variational,
      hasAdvanceableRigidBodyStructures(*this),
      hasMultibodyStructures(*this),
      hasDeformableBodies(*this),
      stage);
  ScopedWorldStepPipelineClear clearCustomPipeline(pipeline);
  for (std::size_t i = 0; i < count; ++i) {
#if DART_BUILD_PROFILE
    StepProfileTimer profileTimer(m_stepProfilingEnabled);
#endif

    stepPipelineOnce(executor, pipeline);
    m_lastDeformableSolverDiagnostics = makeDeformableSolverDiagnostics(
        stages.deformableDynamics.getLastStats());
    recordReplayFrame();
#if DART_BUILD_PROFILE
    profileTimer.finish(m_lastStepProfile);
#endif
  }
}

//==============================================================================
void World::step(
    compute::ComputeExecutor& executor, compute::WorldStepPipeline& pipeline)
{
#if DART_BUILD_PROFILE
  StepProfileTimer profileTimer(m_stepProfilingEnabled);
#endif

  stepPipelineOnce(executor, pipeline);
  recordReplayFrame();
#if DART_BUILD_PROFILE
  profileTimer.finish(m_lastStepProfile);
#endif
}

//==============================================================================
void World::stepPipelineOnce(
    compute::ComputeExecutor& executor, compute::WorldStepPipeline& pipeline)
{
  validateLoopClosureKinematicsPolicySupport(*this);
  validateLoopClosureDynamicsPolicySupport(
      *this,
      m_multibodyIntegrationMethod == MultibodyIntegrationMethod::Variational);
  validateRigidBodyJointPipelineSupport(*this, m_rigidBodySolver);
  validateArticulatedPointJointPipelineSupport(
      *this,
      m_multibodyIntegrationMethod == MultibodyIntegrationMethod::Variational);

  if (!m_simulationMode) {
    enterSimulationMode();
  }

  resetFrameScratchForStep();
  prepareDeactivationForStep();

  // Differentiable opt-in: record the analytic contact-free step Jacobians at
  // the pre-step state before integration. This is a single predictable branch;
  // when off (the default) nothing extra runs and the forward result is
  // bitwise-identical.
  if (m_differentiable) {
    captureStepDerivatives();
  }

  compute::ScopedDeformablePsdBlockProjector psdProjectorScope(
      m_deformablePsdProjector);

#if DART_BUILD_PROFILE
  if (m_stepProfilingEnabled) {
    pipeline.executeProfiled(*this, executor, m_stepProfileScratch);
    std::swap(m_lastStepProfile, m_stepProfileScratch);
  } else {
    pipeline.execute(*this, executor);
  }
#else
  pipeline.execute(*this, executor);
#endif

  updateDeactivationAfterStep();
  m_time += m_timeStep;
  ++m_frame;
  refreshMemoryDiagnostics();
}

//==============================================================================
void World::captureStepDerivatives()
{
#ifdef DART_HAS_DIFF
  // Contact-aware path (PLAN-110 WS2): when the boxed-LCP contact solver is
  // selected, the differentiable step Jacobian must include the analytic
  // frictionless normal-contact gradient. Capture the active contacts at the
  // pre-step state from the same query source the BoxedLcp contact stage
  // consumes, validate they fall inside the WS2 slice's scope, then route
  // through detail::contactStepDerivatives(). When there are no active contacts
  // this reduces exactly to the contact-free (free-fall) Jacobian for the
  // dynamic rigid bodies; that is a mathematically exact reduction, not a
  // fallback.
  if (m_contactSolverMethod == ContactSolverMethod::BoxedLcp) {
    const std::span<const Contact> contacts
        = queryContacts(CollisionQueryOptions{});

    // Scope guard: the contact gradient now covers Coulomb-friction rigid-body
    // contacts including those that excite the angular DOFs (lever arm not
    // parallel to the normal, e.g. an off-COM contact) and multiple
    // simultaneous contacts (e.g. a box resting on its corners). The full 6-DOF
    // Delassus solve and the screw-axis angular rows of J inside
    // detail::contactStepDerivatives() handle that coupling; the differentiated
    // output stays the body's translational [pos; linvel] (the single-step
    // angular state does not feed back into the translational output). The one
    // case still out of scope is an articulated-link (multibody) contact, which
    // the rigid-body contact assembly does not handle: reject it (rather than
    // silently returning a wrong matrix).
    for (const auto& contact : contacts) {
      const auto entityA = detail::toRegistryEntity(contact.bodyA.getEntity());
      const auto entityB = detail::toRegistryEntity(contact.bodyB.getEntity());

      const bool rigidA
          = m_storage->registry.all_of<comps::RigidBodyTag>(entityA);
      const bool rigidB
          = m_storage->registry.all_of<comps::RigidBodyTag>(entityB);
      DART_SIMULATION_THROW_T_IF(
          !rigidA || !rigidB,
          NotImplementedException,
          "World::getStepDerivatives(): differentiable contact gradient not "
          "yet supported for multibody/articulated-link contact; supported: "
          "rigid-body normal/friction contact (PLAN-110 WS2, incl. rotational "
          "and multi-contact)");
    }

    // When physical parameters are registered, additionally assemble the
    // parameter Jacobian ∂x'/∂θ alongside the state/control Jacobians;
    // otherwise skip the extra FD work and leave parameterJacobian empty.
    StepDerivatives contactDerivatives
        = m_storage->differentiableParameters.empty()
              ? detail::contactStepDerivatives(
                    m_storage->registry,
                    contacts,
                    m_gravity,
                    m_timeStep,
                    m_contactGradientMode,
                    m_storage->memoryAllocator)
              : detail::contactStepDerivativesWithParameters(
                    m_storage->registry,
                    contacts,
                    m_gravity,
                    m_timeStep,
                    m_storage->differentiableParameters,
                    m_contactGradientMode,
                    m_storage->memoryAllocator);
    // Non-empty only when a dynamic rigid body is in scope. When empty (e.g. a
    // pure multibody scene under BoxedLcp), fall through to the WS1 path below.
    if (contactDerivatives.stateJacobian.size() != 0) {
      m_storage->stepDerivatives = std::move(contactDerivatives);
      return;
    }
  }

  // WS1 covers a single contact-free multibody. The joint-type-keyed position
  // Jacobian in detail::contactFreeStepDerivatives covers fixed, revolute,
  // prismatic, screw, universal, planar, ball (Spherical), and free (Floating)
  // joints. Assemble tau from the joint efforts in construction (DOF) order and
  // compute the analytic Jacobian at the current (pre-step) state.
  auto view = m_storage->registry.view<comps::MultibodyStructure>();
  for (auto entity : view) {
    const auto& structure = view.get<comps::MultibodyStructure>(entity);

    auto& torques = m_storage->differentiableTorqueScratch;
    torques.clear();
    torques.reserve(structure.links.size());
    for (const auto linkEntity : structure.links) {
      const auto& link = m_storage->registry.get<comps::LinkModel>(linkEntity);
      if (link.parentJoint == entt::null) {
        continue;
      }
      const auto& jointActuation
          = m_storage->registry.get<comps::JointActuation>(link.parentJoint);
      for (Eigen::Index d = 0; d < jointActuation.torque.size(); ++d) {
        torques.push_back(jointActuation.torque[d]);
      }
    }
    if (torques.empty()) {
      continue;
    }
    const Eigen::Map<const Eigen::VectorXd> tau(
        torques.data(), static_cast<Eigen::Index>(torques.size()));

    if (!m_storage->stepDerivatives.has_value()) {
      m_storage->stepDerivatives.emplace();
    }
    detail::contactFreeStepDerivativesInto(
        m_storage->registry,
        structure,
        m_gravity,
        m_timeStep,
        tau,
        *m_storage->stepDerivatives,
        &m_storage->differentiableCoordinateScratch,
        &m_storage->differentiableInverseDynamicsScratch,
        &m_storage->differentiableDynamicsTermsScratch,
        &m_storage->differentiableDerivativeScratch);
    return; // WS1: one multibody.
  }
#endif
}

//==============================================================================
double World::getRigidIpcAdaptiveBarrierStiffnessLowerBound() const noexcept
{
  return m_rigidIpcAdaptiveBarrierStiffnessLowerBound;
}

//==============================================================================
void World::setRigidIpcAdaptiveBarrierStiffnessLowerBound(
    const double value) noexcept
{
  m_rigidIpcAdaptiveBarrierStiffnessLowerBound
      = std::isfinite(value) && value > 0.0 ? value : 1.0;
}

//==============================================================================
void World::resetRigidIpcAdaptiveBarrierStiffnessLowerBound() noexcept
{
  m_rigidIpcAdaptiveBarrierStiffnessLowerBound = 1.0;
}

//==============================================================================
void World::step(
    std::size_t count,
    compute::ComputeExecutor& executor,
    compute::WorldStepPipeline& pipeline)
{
  for (std::size_t i = 0; i < count; ++i) {
    step(executor, pipeline);
  }
}

//==============================================================================
const DeformableSolverDiagnostics& World::getLastDeformableSolverDiagnostics()
    const
{
  return m_lastDeformableSolverDiagnostics;
}

//==============================================================================
void World::setStepProfilingEnabled(bool enabled) noexcept
{
#if DART_BUILD_PROFILE
  m_stepProfilingEnabled = enabled;
#else
  (void)enabled;
#endif
}

//==============================================================================
bool World::isStepProfilingEnabled() const noexcept
{
#if DART_BUILD_PROFILE
  return m_stepProfilingEnabled;
#else
  return false;
#endif
}

//==============================================================================
const compute::ResolvedSolverConfiguration& World::getResolvedConfiguration()
    const noexcept
{
  return m_resolvedConfiguration;
}

//==============================================================================
compute::StepMetrics World::computeStepMetrics() const
{
  // PLAN-091 WP-091.24 (foundational slice): a read-only snapshot of the
  // World's physical invariants. This is a pure query -- it reads body state
  // and gravity and runs no step, mutates no registry/cache field, and so
  // cannot perturb a trajectory (the default-step goldens stay bit-identical).
  // Conventions match the rest of the library exactly (RigidBody energy/
  // momentum accessors and compute::computeMultibodyMechanicalEnergy), so the
  // numbers are consistent across the facade.
  compute::StepMetrics metrics;

  const detail::WorldRegistry& registry = m_storage->registry;
  const Eigen::Vector3d& gravity = m_gravity;

  // Free rigid bodies: full world-frame state is stored, so kinetic/potential
  // energy and linear/angular momentum are computed directly. Static and
  // kinematic bodies carry no dynamics degrees of freedom and contribute no
  // mechanical energy or momentum, so they are skipped (mirroring the contact
  // solver's infinite-mass treatment).
  const auto rigidView = registry.view<comps::RigidBodyTag>();
  for (const auto entity : rigidView) {
    if (registry.all_of<comps::StaticBodyTag>(entity)
        || registry.all_of<comps::KinematicBodyTag>(entity)) {
      continue;
    }
    const auto& mass = registry.get<comps::MassProperties>(entity);
    const auto& velocity = registry.get<comps::Velocity>(entity);
    const auto& transform = registry.get<comps::Transform>(entity);

    const Eigen::Matrix3d rotation
        = transform.orientation.normalized().toRotationMatrix();
    const Eigen::Matrix3d worldInertia
        = rotation * mass.inertia * rotation.transpose();

    metrics.kineticEnergy
        += 0.5 * mass.mass * velocity.linear.squaredNorm()
           + 0.5 * velocity.angular.dot(worldInertia * velocity.angular);
    metrics.potentialEnergy += -mass.mass * gravity.dot(transform.position);

    const Eigen::Vector3d linear = mass.mass * velocity.linear;
    metrics.linearMomentum += linear;
    metrics.angularMomentum
        += transform.position.cross(linear) + worldInertia * velocity.angular;
  }

  // Multibodies: link world-frame velocities are not stored (they are derived
  // from the joint degrees of freedom by forward kinematics), so both the
  // kinetic and potential terms are taken from the variational integrator's own
  // energy helper, which evaluates them on the VarTree's forward-kinematics
  // world transforms. This makes the per-domain split physical -- an earlier
  // version derived potential from the stored comps::LinkState::worldTransform
  // cache, whose gauge did not match the helper (yielding negative kinetic
  // energy at rest). World-frame multibody-link momentum aggregation still
  // needs the link Jacobian and is deferred to a later WP-091.24 slice.
  const auto structureView = registry.view<comps::MultibodyStructure>();
  for (const auto structureEntity : structureView) {
    const auto& structure
        = structureView.get<comps::MultibodyStructure>(structureEntity);

    const auto terms = compute::computeMultibodyMechanicalEnergyTerms(
        registry, structure, gravity);
    metrics.kineticEnergy += terms.kinetic;
    metrics.potentialEnergy += terms.potential;
  }

  metrics.totalEnergy = metrics.kineticEnergy + metrics.potentialEnergy;
  return metrics;
}

//==============================================================================
const compute::WorldStepProfile& World::getLastStepProfile() const noexcept
{
#if DART_BUILD_PROFILE
  return m_lastStepProfile;
#else
  static const compute::WorldStepProfile kEmptyProfile;
  return kEmptyProfile;
#endif
}

//==============================================================================
void World::setReplayRecordingEnabled(bool enabled)
{
  if (!m_replay) {
    m_replay = makeReplayState(m_memoryManager);
  }

  if (enabled == m_replay->recordingEnabled) {
    return;
  }

  m_replay->recordingEnabled = enabled;
  if (enabled) {
    m_replay->frames.clear();
    m_replay->cursor.reset();
    recordReplayFrame();
  }
}

//==============================================================================
bool World::isReplayRecordingEnabled() const noexcept
{
  return m_replay && m_replay->recordingEnabled;
}

//==============================================================================
void World::clearReplayRecording()
{
  if (!m_replay) {
    return;
  }

  m_replay->frames.clear();
  m_replay->cursor.reset();
  if (m_replay->recordingEnabled) {
    recordReplayFrame();
  }
}

//==============================================================================
std::size_t World::getReplayFrameCount() const noexcept
{
  return m_replay ? m_replay->frames.size() : 0u;
}

//==============================================================================
std::optional<std::size_t> World::getReplayCursor() const noexcept
{
  if (!m_replay) {
    return std::nullopt;
  }
  return m_replay->cursor;
}

//==============================================================================
double World::getReplayFrameTime(std::size_t index) const
{
  DART_SIMULATION_THROW_T_IF(
      !m_replay || index >= m_replay->frames.size(),
      InvalidArgumentException,
      "Replay frame index {} is out of range",
      index);
  return m_replay->frames[index].time;
}

//==============================================================================
std::size_t World::getReplaySimulationFrame(std::size_t index) const
{
  DART_SIMULATION_THROW_T_IF(
      !m_replay || index >= m_replay->frames.size(),
      InvalidArgumentException,
      "Replay frame index {} is out of range",
      index);
  return m_replay->frames[index].frame;
}

//==============================================================================
void World::restoreReplayFrame(std::size_t index)
{
  DART_SIMULATION_THROW_T_IF(
      !m_replay || index >= m_replay->frames.size(),
      InvalidArgumentException,
      "Replay frame index {} is out of range",
      index);

  const ReplayState::Frame& replayFrame = m_replay->frames[index];
  const bool wasSimulationMode = m_simulationMode;
  const auto previousRigidBodySolver = m_rigidBodySolver;
  const auto previousMultibodyIntegrationMethod = m_multibodyIntegrationMethod;
  const auto previousContactSolverMethod = m_contactSolverMethod;
  const auto previousComputeAcceleratorPolicy = m_computeAcceleratorPolicy;

  validateReplayComponents<comps::DeformableNodeState>(
      m_storage->registry,
      replayFrame.deformableNodeStates,
      "DeformableNodeState");
  validateReplayTransientComponents<compute::MultibodyVariationalState>(
      m_storage->registry,
      replayFrame.multibodyVariationalStates,
      "MultibodyVariationalState",
      [](const detail::WorldRegistry& registry, entt::entity entity) {
        return registry.all_of<comps::MultibodyStructure>(entity);
      });
  validateReplayTransientComponents<comps::VariationalContactDualState>(
      m_storage->registry,
      replayFrame.variationalContactDualStates,
      "VariationalContactDualState",
      [](const detail::WorldRegistry& registry, entt::entity entity) {
        return registry
            .all_of<comps::MultibodyStructure, comps::VariationalContact>(
                entity);
      });
  validateReplayTransientComponents<comps::DeactivationState>(
      m_storage->registry,
      replayFrame.deactivationStates,
      "DeactivationState",
      [](const detail::WorldRegistry& registry, entt::entity entity) {
        return registry.all_of<comps::RigidBodyTag>(entity)
               || registry.all_of<comps::MultibodyStructure>(entity);
      });

  DART_SIMULATION_THROW_T_IF(
      countReplayView(m_storage->registry.view<comps::JointModel>())
          != replayFrame.joints.size(),
      InvalidOperationException,
      "Cannot restore replay frame: Joint component count changed");
  for (const auto& state : replayFrame.joints) {
    DART_SIMULATION_THROW_T_IF(
        !m_storage->registry.valid(state.entity)
            || !m_storage->registry.all_of<comps::JointModel>(state.entity),
        InvalidOperationException,
        "Cannot restore replay frame: Joint entity layout changed");
    DART_SIMULATION_THROW_T_IF(
        !sameReplayJointLayout(
            m_storage->registry.get<comps::JointModel>(state.entity),
            m_storage->registry.get<comps::JointActuation>(state.entity),
            m_storage->registry.try_get<comps::AvbdJointStiffness>(
                state.entity),
            state.layout),
        InvalidOperationException,
        "Cannot restore replay frame: Joint entity layout changed");
  }

  DART_SIMULATION_THROW_T_IF(
      countReplayView(m_storage->registry.view<comps::LinkModel>())
          != replayFrame.links.size(),
      InvalidOperationException,
      "Cannot restore replay frame: Link component count changed");
  for (const auto& state : replayFrame.links) {
    DART_SIMULATION_THROW_T_IF(
        !m_storage->registry.valid(state.entity)
            || !m_storage->registry.all_of<comps::LinkModel>(state.entity),
        InvalidOperationException,
        "Cannot restore replay frame: Link entity layout changed");
    const auto& link = m_storage->registry.get<comps::LinkModel>(state.entity);
    DART_SIMULATION_THROW_T_IF(
        !sameReplayMassProperties(link.mass, state.massProperties)
            || !sameReplayCollisionGeometry(
                state.collisionGeometry,
                m_storage->registry.try_get<comps::CollisionGeometry>(
                    state.entity)),
        InvalidOperationException,
        "Cannot restore replay frame: Link entity layout changed");
  }

  DART_SIMULATION_THROW_T_IF(
      countReplayPublicFrameEntities(m_storage->registry)
          != replayFrame.publicFrames.size(),
      InvalidOperationException,
      "Cannot restore replay frame: public Frame component count changed");
  for (const auto& state : replayFrame.publicFrames) {
    const bool expectedFree = state.freeFrameProperties.has_value();
    const bool expectedFixed = state.fixedFrameProperties.has_value();
    const bool entityValid = m_storage->registry.valid(state.entity);
    const bool currentFree
        = entityValid
          && m_storage->registry
                 .all_of<comps::FreeFrameTag, comps::FreeFrameProperties>(
                     state.entity);
    const bool currentFixed
        = entityValid
          && m_storage->registry
                 .all_of<comps::FixedFrameTag, comps::FixedFrameProperties>(
                     state.entity);
    const bool layoutChanged
        = !entityValid
          || !m_storage->registry.all_of<comps::FrameState, comps::FrameCache>(
              state.entity)
          || m_storage->registry.all_of<comps::RigidBodyTag>(state.entity)
          || m_storage->registry.all_of<comps::LinkModel>(state.entity)
          || currentFree != expectedFree || currentFixed != expectedFixed
          || currentFree == currentFixed;
    DART_SIMULATION_THROW_T_IF(
        layoutChanged,
        InvalidOperationException,
        "Cannot restore replay frame: public Frame entity layout changed");
  }

  validateReplayLoopClosures(m_storage->registry, replayFrame.loopClosures);

  DART_SIMULATION_THROW_T_IF(
      countReplayView(m_storage->registry.view<
                      comps::RigidBodyTag,
                      comps::FrameState,
                      comps::Transform,
                      comps::Velocity,
                      comps::Force,
                      comps::MassProperties>())
          != replayFrame.rigidBodies.size(),
      InvalidOperationException,
      "Cannot restore replay frame: RigidBody component count changed");
  for (const auto& state : replayFrame.rigidBodies) {
    const bool layoutChanged = !m_storage->registry.valid(state.entity)
                               || !m_storage->registry.all_of<
                                   comps::RigidBodyTag,
                                   comps::FrameState,
                                   comps::Transform,
                                   comps::Velocity,
                                   comps::Force,
                                   comps::MassProperties>(state.entity);
    DART_SIMULATION_THROW_T_IF(
        layoutChanged,
        InvalidOperationException,
        "Cannot restore replay frame: RigidBody entity layout changed");
    const auto& frameState
        = m_storage->registry.get<comps::FrameState>(state.entity);
    const auto& massProperties
        = m_storage->registry.get<comps::MassProperties>(state.entity);
    DART_SIMULATION_THROW_T_IF(
        frameState.parentFrame != state.parentFrame
            || !sameReplayMassProperties(massProperties, state.massProperties)
            || !sameReplayContactMaterial(
                state.contactMaterial,
                m_storage->registry.try_get<comps::ContactMaterial>(
                    state.entity))
            || !sameReplayCollisionGeometry(
                state.collisionGeometry,
                m_storage->registry.try_get<comps::CollisionGeometry>(
                    state.entity))
            || m_storage->registry.all_of<comps::DeformableGroundBarrierTag>(
                   state.entity)
                   != state.hasDeformableGroundBarrier
            || m_storage->registry
                       .all_of<comps::DeformableSurfaceCcdObstacleTag>(
                           state.entity)
                   != state.hasDeformableSurfaceCcdObstacle
            || m_storage->registry.all_of<comps::DeformableObstacleNoCcdTag>(
                   state.entity)
                   != state.hasDeformableObstacleNoCcd,
        InvalidOperationException,
        "Cannot restore replay frame: RigidBody entity layout changed");
    DART_SIMULATION_THROW_T_IF(
        m_storage->registry.all_of<comps::StaticBodyTag>(state.entity)
                != state.isStatic
            || m_storage->registry.all_of<comps::KinematicBodyTag>(state.entity)
                   != state.isKinematic,
        InvalidOperationException,
        "Cannot restore replay frame: RigidBody entity mode changed");
  }

  auto& replayAllocator = m_memoryManager.getFreeAllocator();
  const auto rigidBodyRestoreOrder = orderReplayRigidBodiesParentBeforeChild(
      m_storage->registry,
      replayFrame.rigidBodies,
      replayFrame.publicFrames,
      replayAllocator);

  compute::avbd_replay::restoreDeformableAvbdWarmStartReplayState(
      m_storage->registry, replayFrame.deformableAvbdWarmStartStates);

  restoreReplayDeformableNodeStates(
      m_storage->registry, replayFrame.deformableNodeStates);
  restoreReplayTransientComponentsWithRestorer<
      compute::MultibodyVariationalState>(
      m_storage->registry,
      replayFrame.multibodyVariationalStates,
      "MultibodyVariationalState",
      replayAllocator,
      [](const detail::WorldRegistry& registry, entt::entity entity) {
        return registry.all_of<comps::MultibodyStructure>(entity);
      },
      [&](entt::entity entity,
          const compute::MultibodyVariationalState& replayState) {
        auto* state
            = m_storage->registry.try_get<compute::MultibodyVariationalState>(
                entity);
        if (state == nullptr) {
          state
              = &m_storage->registry
                     .emplace<compute::MultibodyVariationalState>(
                         entity,
                         makeReplayMultibodyVariationalState(replayAllocator));
        }
        restoreReplayMultibodyVariationalState(
            replayState, *state, replayAllocator);
      });
  restoreReplayTransientComponentsWithRestorer<
      comps::VariationalContactDualState>(
      m_storage->registry,
      replayFrame.variationalContactDualStates,
      "VariationalContactDualState",
      replayAllocator,
      [](const detail::WorldRegistry& registry, entt::entity entity) {
        return registry
            .all_of<comps::MultibodyStructure, comps::VariationalContact>(
                entity);
      },
      [&](entt::entity entity,
          const comps::VariationalContactDualState& replayState) {
        auto* state
            = m_storage->registry.try_get<comps::VariationalContactDualState>(
                entity);
        if (state == nullptr) {
          state = &m_storage->registry
                       .emplace<comps::VariationalContactDualState>(
                           entity,
                           makeReplayVariationalContactDualState(
                               replayAllocator));
        }
        restoreReplayVariationalContactDualState(
            replayState, *state, replayAllocator);
      });
  restoreReplayTransientComponents<comps::DeactivationState>(
      m_storage->registry,
      replayFrame.deactivationStates,
      "DeactivationState",
      replayAllocator,
      [](const detail::WorldRegistry& registry, entt::entity entity) {
        return registry.all_of<comps::RigidBodyTag>(entity)
               || registry.all_of<comps::MultibodyStructure>(entity);
      });

  for (const auto& state : replayFrame.joints) {
    auto& jointState = m_storage->registry.get<comps::JointState>(state.entity);
    auto& jointActuation
        = m_storage->registry.get<comps::JointActuation>(state.entity);
    restoreReplayVector(state.position, jointState.position);
    restoreReplayVector(state.velocity, jointState.velocity);
    restoreReplayVector(state.acceleration, jointState.acceleration);
    restoreReplayVector(state.torque, jointActuation.torque);
    restoreReplayVector(state.commandVelocity, jointActuation.commandVelocity);
    jointState.broken = state.broken;
  }

  for (const auto& state : replayFrame.links) {
    m_storage->registry.get<comps::LinkControl>(state.entity).externalForce
        = state.externalForce;
  }

  bool frameTopologyChanged = false;
  for (const auto& state : replayFrame.publicFrames) {
    const auto& currentFrameState
        = m_storage->registry.get<comps::FrameState>(state.entity);
    frameTopologyChanged
        = frameTopologyChanged
          || currentFrameState.parentFrame != state.frameState.parentFrame;
    m_storage->registry.replace<comps::FrameState>(
        state.entity, state.frameState);
    if (state.freeFrameProperties) {
      m_storage->registry.replace<comps::FreeFrameProperties>(
          state.entity, *state.freeFrameProperties);
    }
    if (state.fixedFrameProperties) {
      m_storage->registry.replace<comps::FixedFrameProperties>(
          state.entity, *state.fixedFrameProperties);
    }
  }
  if (frameTopologyChanged) {
    markFrameTopologyChanged();
  }
  markFrameCachesDirty(m_storage->registry);

  for (const auto stateIndex : rigidBodyRestoreOrder) {
    const auto& state = replayFrame.rigidBodies[stateIndex];
    const auto worldTransform
        = toIsometry(state.transform.position, state.transform.orientation);
    if (auto* freeFrame
        = m_storage->registry.try_get<comps::FreeFrameProperties>(
            state.entity)) {
      const Frame parentFrame(
          detail::fromRegistryEntity(state.parentFrame), this);
      freeFrame->localTransform
          = parentFrame.getTransform().inverse() * worldTransform;
    }
    m_storage->registry.replace<comps::Transform>(
        state.entity, state.transform);
    m_storage->registry.replace<comps::Velocity>(state.entity, state.velocity);
    m_storage->registry.replace<comps::Force>(state.entity, state.force);
  }

  m_simulationMode = replayFrame.simulationMode;
  m_gravity = replayFrame.gravity;
  m_rigidBodySolver = replayFrame.rigidBodySolver;
  m_timeStep = replayFrame.timeStep;
  m_differentiable = replayFrame.differentiable;
  m_contactSolverMethod = replayFrame.contactSolverMethod;
  m_contactGradientMode = replayFrame.contactGradientMode;
  m_computeAcceleratorPolicy = replayFrame.computeAcceleratorPolicy;
  m_deactivationOptions = replayFrame.deactivationOptions;
  m_time = replayFrame.time;
  m_frame = replayFrame.frame;
  m_lastDeformableSolverDiagnostics = replayFrame.deformableSolverDiagnostics;
  m_rigidIpcAdaptiveBarrierStiffnessLowerBound
      = replayFrame.rigidIpcAdaptiveBarrierStiffnessLowerBound;
  m_multibodyIntegrationMethod = replayFrame.multibodyIntegrationMethod;
  m_variationalIntegratorMaxIterations
      = replayFrame.variationalIntegratorMaxIterations;
  m_variationalIntegratorTolerance = replayFrame.variationalIntegratorTolerance;
  m_storage->stepDerivatives = replayFrame.stepDerivatives;
  m_storage->differentiableParameters.assign(
      replayFrame.differentiableParameters.begin(),
      replayFrame.differentiableParameters.end());

  if (m_collisionQueryCache) {
    m_collisionQueryCache.reset();
  }
  markFrameCachesDirty(m_storage->registry);
  if (m_simulationMode) {
    updateKinematics();
    const bool stepPipelinePolicyChanged
        = !wasSimulationMode || frameTopologyChanged
          || previousRigidBodySolver != m_rigidBodySolver
          || previousMultibodyIntegrationMethod != m_multibodyIntegrationMethod
          || previousContactSolverMethod != m_contactSolverMethod
          || previousComputeAcceleratorPolicy != m_computeAcceleratorPolicy;
    if (stepPipelinePolicyChanged) {
      prepareStepPipelineCacheForCurrentConfiguration();
    }
  }

  m_replay->cursor = index;
}

//==============================================================================
void World::recordReplayFrame()
{
  if (!m_replay || !m_replay->recordingEnabled) {
    return;
  }

  if (m_replay->cursor && *m_replay->cursor + 1u < m_replay->frames.size()) {
    m_replay->frames.erase(
        m_replay->frames.begin()
            + static_cast<std::ptrdiff_t>(*m_replay->cursor + 1u),
        m_replay->frames.end());
  }

  auto& replayAllocator = m_memoryManager.getFreeAllocator();
  ReplayState::Frame replayFrame(replayAllocator);
  replayFrame.simulationMode = m_simulationMode;
  replayFrame.gravity = m_gravity;
  replayFrame.rigidBodySolver = m_rigidBodySolver;
  replayFrame.timeStep = m_timeStep;
  replayFrame.differentiable = m_differentiable;
  replayFrame.contactSolverMethod = m_contactSolverMethod;
  replayFrame.contactGradientMode = m_contactGradientMode;
  replayFrame.computeAcceleratorPolicy = m_computeAcceleratorPolicy;
  replayFrame.deactivationOptions = m_deactivationOptions;
  replayFrame.time = m_time;
  replayFrame.frame = m_frame;
  replayFrame.deformableSolverDiagnostics = m_lastDeformableSolverDiagnostics;
  replayFrame.rigidIpcAdaptiveBarrierStiffnessLowerBound
      = m_rigidIpcAdaptiveBarrierStiffnessLowerBound;
  replayFrame.multibodyIntegrationMethod = m_multibodyIntegrationMethod;
  replayFrame.variationalIntegratorMaxIterations
      = m_variationalIntegratorMaxIterations;
  replayFrame.variationalIntegratorTolerance = m_variationalIntegratorTolerance;
  replayFrame.stepDerivatives = m_storage->stepDerivatives;
  replayFrame.differentiableParameters.assign(
      m_storage->differentiableParameters.begin(),
      m_storage->differentiableParameters.end());

  replayFrame.deformableNodeStates = captureReplayDeformableNodeStates<
      ReplayState::DeformableNodeStateSnapshot>(
      m_storage->registry, replayAllocator);
  replayFrame.deformableAvbdWarmStartStates
      = compute::avbd_replay::captureDeformableAvbdWarmStartReplayState(
          m_storage->registry, replayAllocator);
  replayFrame.multibodyVariationalStates
      = captureReplayComponents<compute::MultibodyVariationalState>(
          m_storage->registry, replayAllocator);
  replayFrame.variationalContactDualStates
      = captureReplayComponents<comps::VariationalContactDualState>(
          m_storage->registry, replayAllocator);
  replayFrame.deactivationStates
      = captureReplayComponents<comps::DeactivationState>(
          m_storage->registry, replayAllocator);

  auto jointView = m_storage->registry.view<comps::JointModel>();
  replayFrame.joints.reserve(countReplayView(jointView));
  for (auto entity : jointView) {
    const auto& joint = jointView.get<comps::JointModel>(entity);
    const auto& jointState = m_storage->registry.get<comps::JointState>(entity);
    const auto& jointActuation
        = m_storage->registry.get<comps::JointActuation>(entity);
    ReplayState::JointState state(replayAllocator);
    state.entity = entity;
    state.layout.type = joint.type;
    state.layout.actuatorType = jointActuation.actuatorType;
    state.layout.name.assign(joint.name.begin(), joint.name.end());
    captureReplayVector(joint.springStiffness, state.layout.springStiffness);
    captureReplayVector(
        joint.dampingCoefficient, state.layout.dampingCoefficient);
    captureReplayVector(joint.restPosition, state.layout.restPosition);
    captureReplayVector(joint.armature, state.layout.armature);
    captureReplayVector(joint.coulombFriction, state.layout.coulombFriction);
    state.layout.breakForce = joint.breakForce;
    if (const auto* avbdStiffness
        = m_storage->registry.try_get<comps::AvbdJointStiffness>(entity)) {
      state.layout.hasAvbdStiffnessState = true;
      state.layout.avbdStiffness = *avbdStiffness;
    } else {
      state.layout.hasAvbdStiffnessState = false;
      state.layout.avbdStiffness = comps::AvbdJointStiffness{};
    }
    captureReplayJointLimits(joint.limits, state.layout.limits);
    state.layout.axis = joint.axis;
    state.layout.axis2 = joint.axis2;
    state.layout.pitch = joint.pitch;
    state.layout.parentLink = joint.parentLink;
    state.layout.childLink = joint.childLink;
    state.layout.hasRigidBodyFixedJointAnchors
        = joint.hasRigidBodyFixedJointAnchors;
    state.layout.rigidBodyFixedJointLocalAnchorParent
        = joint.rigidBodyFixedJointLocalAnchorParent;
    state.layout.rigidBodyFixedJointLocalAnchorChild
        = joint.rigidBodyFixedJointLocalAnchorChild;
    state.layout.rigidBodyFixedJointTargetRelativeOrientation
        = joint.rigidBodyFixedJointTargetRelativeOrientation;
    captureReplayVector(jointState.position, state.position);
    captureReplayVector(jointState.velocity, state.velocity);
    captureReplayVector(jointState.acceleration, state.acceleration);
    captureReplayVector(jointActuation.torque, state.torque);
    captureReplayVector(jointActuation.commandVelocity, state.commandVelocity);
    state.broken = jointState.broken;
    replayFrame.joints.push_back(std::move(state));
  }
  std::ranges::sort(replayFrame.joints, [](const auto& lhs, const auto& rhs) {
    return static_cast<std::uint32_t>(lhs.entity)
           < static_cast<std::uint32_t>(rhs.entity);
  });

  auto linkView = m_storage->registry.view<comps::LinkModel>();
  replayFrame.links.reserve(countReplayView(linkView));
  for (auto entity : linkView) {
    const auto& link = linkView.get<comps::LinkModel>(entity);
    const auto& linkControl
        = m_storage->registry.get<comps::LinkControl>(entity);
    replayFrame.links.push_back(
        ReplayState::LinkState{
            entity,
            link.mass,
            captureReplayOptionalComponent<comps::CollisionGeometry>(
                m_storage->registry, entity),
            linkControl.externalForce});
  }
  std::ranges::sort(replayFrame.links, [](const auto& lhs, const auto& rhs) {
    return static_cast<std::uint32_t>(lhs.entity)
           < static_cast<std::uint32_t>(rhs.entity);
  });

  auto publicFrameView = m_storage->registry.view<comps::FrameState>();
  replayFrame.publicFrames.reserve(
      countReplayPublicFrameEntities(m_storage->registry));
  for (auto entity : publicFrameView) {
    if (!isReplayPublicFrameEntity(m_storage->registry, entity)) {
      continue;
    }

    replayFrame.publicFrames.push_back(
        ReplayState::PublicFrameState{
            entity,
            publicFrameView.get<comps::FrameState>(entity),
            captureReplayOptionalComponent<comps::FreeFrameProperties>(
                m_storage->registry, entity),
            captureReplayOptionalComponent<comps::FixedFrameProperties>(
                m_storage->registry, entity)});
  }
  std::ranges::sort(
      replayFrame.publicFrames, [](const auto& lhs, const auto& rhs) {
        return static_cast<std::uint32_t>(lhs.entity)
               < static_cast<std::uint32_t>(rhs.entity);
      });

  replayFrame.loopClosures
      = captureReplayLoopClosures<ReplayState::LoopClosureState>(
          m_storage->registry, replayAllocator);

  auto rigidBodyView = m_storage->registry.view<
      comps::RigidBodyTag,
      comps::FrameState,
      comps::Transform,
      comps::Velocity,
      comps::Force,
      comps::MassProperties>();
  replayFrame.rigidBodies.reserve(countReplayView(rigidBodyView));
  for (auto entity : rigidBodyView) {
    replayFrame.rigidBodies.push_back(
        ReplayState::RigidBodyState{
            entity,
            rigidBodyView.get<comps::Transform>(entity),
            rigidBodyView.get<comps::Velocity>(entity),
            rigidBodyView.get<comps::Force>(entity),
            rigidBodyView.get<comps::MassProperties>(entity),
            rigidBodyView.get<comps::FrameState>(entity).parentFrame,
            captureReplayOptionalComponent<comps::ContactMaterial>(
                m_storage->registry, entity),
            captureReplayOptionalComponent<comps::CollisionGeometry>(
                m_storage->registry, entity),
            m_storage->registry.all_of<comps::StaticBodyTag>(entity),
            m_storage->registry.all_of<comps::KinematicBodyTag>(entity),
            m_storage->registry.all_of<comps::DeformableGroundBarrierTag>(
                entity),
            m_storage->registry.all_of<comps::DeformableSurfaceCcdObstacleTag>(
                entity),
            m_storage->registry.all_of<comps::DeformableObstacleNoCcdTag>(
                entity)});
  }
  std::ranges::sort(
      replayFrame.rigidBodies, [](const auto& lhs, const auto& rhs) {
        return static_cast<std::uint32_t>(lhs.entity)
               < static_cast<std::uint32_t>(rhs.entity);
      });

  m_replay->frames.push_back(std::move(replayFrame));
  m_replay->cursor = m_replay->frames.size() - 1u;
}

//==============================================================================
std::vector<Contact> World::collide()
{
  return collide(CollisionQueryOptions{});
}

//==============================================================================
void World::setCollisionPairIgnored(
    const Frame& first, const Frame& second, bool ignored)
{
  const entt::entity firstEntity
      = resolveCollisionPairFrame(*this, first, "first");
  const entt::entity secondEntity
      = resolveCollisionPairFrame(*this, second, "second");
  DART_SIMULATION_THROW_T_IF(
      firstEntity == secondEntity,
      InvalidArgumentException,
      "Collision pair frames must be distinct");

  const auto key = makeCollisionPairKey(firstEntity, secondEntity);
  if (ignored) {
    m_storage->ignoredCollisionPairs.insert(key);
  } else {
    m_storage->ignoredCollisionPairs.erase(key);
  }
}

//==============================================================================
bool World::isCollisionPairIgnored(
    const Frame& first, const Frame& second) const
{
  const entt::entity firstEntity
      = resolveCollisionPairFrame(*this, first, "first");
  const entt::entity secondEntity
      = resolveCollisionPairFrame(*this, second, "second");
  DART_SIMULATION_THROW_T_IF(
      firstEntity == secondEntity,
      InvalidArgumentException,
      "Collision pair frames must be distinct");

  return m_storage->ignoredCollisionPairs.contains(
      makeCollisionPairKey(firstEntity, secondEntity));
}

//==============================================================================
void World::clearIgnoredCollisionPairs()
{
  m_storage->ignoredCollisionPairs.clear();
}

//==============================================================================
std::size_t World::getIgnoredCollisionPairCount() const noexcept
{
  return m_storage->ignoredCollisionPairs.size();
}

//==============================================================================
std::vector<Contact> World::collide(const CollisionQueryOptions& options)
{
  const std::span<const Contact> contacts = queryContacts(options);
  return std::vector<Contact>(contacts.begin(), contacts.end());
}

//==============================================================================
std::span<const Contact> World::queryContacts(
    const CollisionQueryOptions& options, bool includeShapeContactDetails)
{
  return updateCollisionQueryCache(
      options, includeShapeContactDetails, /*collectContacts=*/true);
}

//==============================================================================
void World::prepareCollisionQueryCache(
    const CollisionQueryOptions& options, bool includeShapeContactDetails)
{
  (void)updateCollisionQueryCache(
      options, includeShapeContactDetails, /*collectContacts=*/false);
}

//==============================================================================
std::span<const Contact> World::updateCollisionQueryCache(
    const CollisionQueryOptions& options,
    bool includeShapeContactDetails,
    bool collectContacts)
{
  if (!m_collisionQueryCache) {
    m_collisionQueryCache = makeCollisionQueryCache(m_memoryManager);
  }
  auto& cache = *m_collisionQueryCache;
  auto& specs = cache.specs;
  specs.clear();

  const auto findMultibodyOwningLink = [&](entt::entity linkEntity) {
    auto view = m_storage->registry.view<comps::MultibodyStructure>();
    for (auto multibody : view) {
      const auto& structure = view.get<comps::MultibodyStructure>(multibody);
      if (std::find(structure.links.begin(), structure.links.end(), linkEntity)
          != structure.links.end()) {
        return multibody;
      }
    }
    return static_cast<entt::entity>(entt::null);
  };

  const auto makeNativeShape =
      [](const CollisionShape& collisionShape) -> std::unique_ptr<ncol::Shape> {
    std::unique_ptr<ncol::Shape> shape;
    switch (collisionShape.type) {
      case CollisionShapeType::Sphere:
        shape = std::make_unique<ncol::SphereShape>(collisionShape.radius);
        break;
      case CollisionShapeType::Box:
        shape = std::make_unique<ncol::BoxShape>(collisionShape.halfExtents);
        break;
      case CollisionShapeType::Capsule:
        shape = std::make_unique<ncol::CapsuleShape>(
            collisionShape.radius, 2.0 * collisionShape.halfExtents.z());
        break;
      case CollisionShapeType::Cylinder:
        shape = std::make_unique<ncol::CylinderShape>(
            collisionShape.radius, 2.0 * collisionShape.halfExtents.z());
        break;
      case CollisionShapeType::Plane:
        shape = std::make_unique<ncol::PlaneShape>(
            collisionShape.normal, collisionShape.offset);
        break;
      case CollisionShapeType::Mesh:
        shape = std::make_unique<ncol::MeshShape>(
            collisionShape.vertices, collisionShape.triangles);
        break;
    }
    return shape;
  };

  const auto supportsNativeShape = [](const CollisionShape& collisionShape) {
    switch (collisionShape.type) {
      case CollisionShapeType::Sphere:
      case CollisionShapeType::Box:
      case CollisionShapeType::Capsule:
      case CollisionShapeType::Cylinder:
      case CollisionShapeType::Plane:
      case CollisionShapeType::Mesh:
        return true;
    }
    return false;
  };

  if (options.includeRigidBodyPairs) {
    collectLivePublicRigidBodyJointPairsInto(
        m_storage->registry, cache.liveRigidBodyJointPairs);
  } else {
    cache.liveRigidBodyJointPairs.clear();
  }
  const bool hasIgnoredCollisionPairs
      = !m_storage->ignoredCollisionPairs.empty();
  const bool hasLiveRigidBodyJointPairs
      = !cache.liveRigidBodyJointPairs.empty();

  const auto addSpecs = [&](entt::entity entity,
                            entt::entity multibody,
                            bool isLink,
                            const comps::CollisionGeometry& geometry,
                            const Eigen::Isometry3d& pose) {
    for (std::size_t i = 0; i < geometry.shapes.size(); ++i) {
      const auto& shape = geometry.shapes[i];
      if (!supportsNativeShape(shape)) {
        continue;
      }
      const Eigen::Isometry3d worldPose = pose * shape.localTransform;
      specs.push_back(
          CollisionQueryCache::ShapeEntrySpec{
              CollisionQueryCache::Key{
                  entity, i, geometry.revision, multibody, isLink},
              &shape,
              worldPose,
              includeShapeContactDetails ? worldPose.inverse()
                                         : Eigen::Isometry3d::Identity()});
    }
  };

  const auto includesPair = [&](const CollisionQueryCache::ObjectEntry& a,
                                const CollisionQueryCache::ObjectEntry& b) {
    if (a.entity == b.entity) {
      return false;
    }
    const bool rigidBodyPair = !a.isLink && !b.isLink;
    if (hasIgnoredCollisionPairs
        || (rigidBodyPair && hasLiveRigidBodyJointPairs)) {
      const auto pairKey = makeCollisionPairKey(a.entity, b.entity);
      if (hasIgnoredCollisionPairs
          && m_storage->ignoredCollisionPairs.contains(pairKey)) {
        return false;
      }
      if (rigidBodyPair && hasLiveRigidBodyJointPairs
          && std::binary_search(
              cache.liveRigidBodyJointPairs.begin(),
              cache.liveRigidBodyJointPairs.end(),
              pairKey)) {
        return false;
      }
    }

    if (a.isLink && b.isLink) {
      return options.includeLinkPairs
             && (options.includeSameMultibodyLinkPairs
                 || a.multibody == entt::null || a.multibody != b.multibody);
    }

    if (a.isLink || b.isLink) {
      return options.includeRigidBodyLinkPairs;
    }

    return options.includeRigidBodyPairs;
  };

  // Rigid bodies pose their collision shapes from the rigid-body transform.
  auto rigidBodyView = m_storage->registry.view<
      comps::CollisionGeometry,
      comps::Transform,
      comps::RigidBodyTag>();
  for (auto entity : rigidBodyView) {
    const auto& geometry = rigidBodyView.get<comps::CollisionGeometry>(entity);
    const auto& transform = rigidBodyView.get<comps::Transform>(entity);

    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.linear() = transform.orientation.normalized().toRotationMatrix();
    pose.translation() = transform.position;
    addSpecs(entity, entt::null, false, geometry, pose);
  }

  // Multibody links pose their collision shapes through the frame accessor so
  // dirty joint-driven caches are refreshed before the query.
  auto linkView = m_storage->registry.view<
      comps::CollisionGeometry,
      comps::LinkModel,
      comps::FrameCache>();
  for (auto entity : linkView) {
    const auto& geometry = linkView.get<comps::CollisionGeometry>(entity);
    const Link link(detail::fromRegistryEntity(entity), this);
    const entt::entity multibody = findMultibodyOwningLink(entity);
    addSpecs(entity, multibody, true, geometry, link.getWorldTransform());
  }

  if (specs.empty()) {
    cache.contacts.clear();
    return std::span<const Contact>{
        cache.contacts.data(), cache.contacts.size()};
  }

  const bool rebuildCache
      = cache.keys.size() != specs.size()
        || !std::equal(
            specs.begin(),
            specs.end(),
            cache.keys.begin(),
            [](const CollisionQueryCache::ShapeEntrySpec& spec,
               const CollisionQueryCache::Key& key) {
              return spec.key == key;
            });

  if (rebuildCache) {
    cache.clearObjectsAndResultsPreservingSpecs();
    cache.collisionWorld.reserveObjects(specs.size());
    cache.keys.reserve(specs.size());
    cache.entries.reserve(specs.size());
    for (const auto& spec : specs) {
      ncol::CollisionObject object = cache.collisionWorld.createObject(
          makeNativeShape(*spec.shape), spec.pose);
      const std::size_t entryIndex = cache.entries.size();
      const std::size_t objectId = object.getId();
      if (objectId >= cache.entryByObjectId.size()) {
        cache.entryByObjectId.resize(
            objectId + 1, std::numeric_limits<std::size_t>::max());
      }
      cache.entryByObjectId[objectId] = entryIndex;
      cache.keys.push_back(spec.key);
      cache.entries.push_back(
          CollisionQueryCache::ObjectEntry{
              spec.key.entity, spec.key.multibody, spec.key.isLink, object});
    }
  } else {
    for (std::size_t i = 0; i < specs.size(); ++i) {
      auto& object = cache.entries[i].object;
      object.setTransform(specs[i].pose);
      cache.collisionWorld.updateObject(object);
    }
  }

  // Broad-phase-pruned narrow-phase queries. Each candidate pair's bodies are
  // known here, so the contacts map back to the right simulation bodies without
  // relying on the result carrying object identity.
  const auto option = ncol::CollisionOption::fullContacts();
  cache.collisionWorld.buildBroadPhaseSnapshot(cache.candidatePairs);
  auto& contacts = cache.contacts;
  contacts.clear();
  if (!collectContacts) {
    return contacts;
  }
  for (const auto& pair : cache.candidatePairs.pairs) {
    if (pair.first >= cache.entryByObjectId.size()
        || pair.second >= cache.entryByObjectId.size()) {
      continue;
    }
    const std::size_t i = cache.entryByObjectId[pair.first];
    const std::size_t j = cache.entryByObjectId[pair.second];
    if (i >= cache.entries.size() || j >= cache.entries.size()) {
      continue;
    }
    if (!includesPair(cache.entries[i], cache.entries[j])) {
      continue;
    }

    auto& result = cache.pairResult;
    result.clear();
    if (!cache.collisionWorld.collide(
            cache.entries[i].object, cache.entries[j].object, option, result)) {
      continue;
    }

    result.forEachContact([&](const ncol::ContactPoint& point) {
      // The native narrow phase reports the normal pointing from the second
      // object toward the first; the public Contact convention points from
      // bodyA (entries[i]) toward bodyB (entries[j]), so negate it.
      Contact contact{
          CollisionBody(
              detail::fromRegistryEntity(cache.entries[i].entity), this),
          CollisionBody(
              detail::fromRegistryEntity(cache.entries[j].entity), this),
          point.position,
          -point.normal,
          point.depth,
          specs[i].key.shapeIndex,
          specs[j].key.shapeIndex};
      if (includeShapeContactDetails) {
        contact.localPointA = specs[i].inversePose * point.position;
        contact.localPointB = specs[j].inversePose * point.position;
      }
      contacts.push_back(std::move(contact));
    });
  }

  return std::span<const Contact>{contacts.data(), contacts.size()};
}

//==============================================================================
void World::saveBinary(std::ostream& output) const
{
  io::writeFormatHeader(output);

  io::EntityMap entityMap;
  io::SerializerRegistry::instance().saveAllEntities(
      output, m_storage->registry, entityMap);

  const std::uint8_t simulationFlag = m_simulationMode ? 1 : 0;
  io::writePOD(output, simulationFlag);
  io::writePOD(output, m_freeFrameCounter);
  io::writePOD(output, m_fixedFrameCounter);
  io::writePOD(output, m_multibodyCounter);
  io::writePOD(output, m_rigidBodyCounter);
  io::writePOD(output, m_linkCounter);
  io::writePOD(output, m_jointCounter);
  io::writePOD(output, m_timeStep);
  io::writePOD(output, m_time);
  io::writePOD(output, m_frame);
  io::writePOD(output, m_gravity.x());
  io::writePOD(output, m_gravity.y());
  io::writePOD(output, m_gravity.z());
  io::writePOD(output, m_deformableBodyCounter);

  const std::uint8_t differentiableFlag = m_differentiable ? 1 : 0;
  io::writePOD(output, differentiableFlag);

  io::writePOD(output, encodeRigidBodySolver(m_rigidBodySolver));
  io::writePOD(output, encodeContactSolverMethod(m_contactSolverMethod));
  io::writePOD(output, encodeContactGradientMode(m_contactGradientMode));
  const std::uint8_t multibodyIntegrationMethod
      = m_multibodyIntegrationMethod == MultibodyIntegrationMethod::Variational
            ? 1u
            : 0u;
  io::writePOD(output, multibodyIntegrationMethod);

  std::vector<
      detail::WorldStorage::CollisionPairKey,
      detail::WorldStorage::CollisionPairAllocator>
      savedIgnoredPairs{m_storage->ignoredCollisionPairs.get_allocator()};
  savedIgnoredPairs.reserve(m_storage->ignoredCollisionPairs.size());
  for (const auto& pair : m_storage->ignoredCollisionPairs) {
    if (entityMap.contains(pair.first) && entityMap.contains(pair.second)) {
      savedIgnoredPairs.push_back(pair);
    }
  }

  io::writePOD(output, savedIgnoredPairs.size());
  for (const auto& [first, second] : savedIgnoredPairs) {
    io::writePOD(output, static_cast<std::uint32_t>(entityMap.at(first)));
    io::writePOD(output, static_cast<std::uint32_t>(entityMap.at(second)));
  }

  io::writePOD(output, m_variationalIntegratorMaxIterations);
  io::writePOD(output, m_variationalIntegratorTolerance);

  const std::uint8_t deactivationEnabled
      = m_deactivationOptions.enabled ? 1u : 0u;
  io::writePOD(output, deactivationEnabled);
  io::writePOD(output, m_deactivationOptions.linearSpeedThreshold);
  io::writePOD(output, m_deactivationOptions.angularSpeedThreshold);
  io::writePOD(output, m_deactivationOptions.generalizedSpeedThreshold);
  io::writePOD(output, m_deactivationOptions.timeUntilSleep);
  io::writePOD(output, m_deactivationOptions.wakeThresholdScale);
  io::writePOD(output, m_deactivationOptions.disturbanceForceThreshold);

  io::writePOD(
      output, encodeComputeAcceleratorPolicy(m_computeAcceleratorPolicy));
}

//==============================================================================
void World::loadBinary(std::istream& input)
{
  clear();

  const auto formatVersion = io::readFormatHeader(input);

  io::EntityMap entityMap;
  io::SerializerRegistry::instance().loadAllEntities(
      input, m_storage->registry, entityMap, formatVersion);
  rebindLoadedWorldComponentAllocators(
      m_storage->registry, getMemoryManager().getFreeAllocator());

  // World metadata (optional for forward-compatibility)
  if (input.peek() != std::char_traits<char>::eof()) {
    std::uint8_t simulationFlag = 0;
    io::readPOD(input, simulationFlag);
    m_simulationMode = simulationFlag != 0;

    io::readPOD(input, m_freeFrameCounter);
    io::readPOD(input, m_fixedFrameCounter);
    io::readPOD(input, m_multibodyCounter);
    io::readPOD(input, m_rigidBodyCounter);
    io::readPOD(input, m_linkCounter);
    io::readPOD(input, m_jointCounter);

    if (input.peek() != std::char_traits<char>::eof()) {
      io::readPOD(input, m_timeStep);
      io::readPOD(input, m_time);
      io::readPOD(input, m_frame);
    }

    if (formatVersion >= 2) {
      double gravityX = 0.0;
      double gravityY = 0.0;
      double gravityZ = 0.0;
      io::readPOD(input, gravityX);
      io::readPOD(input, gravityY);
      io::readPOD(input, gravityZ);
      m_gravity = Eigen::Vector3d(gravityX, gravityY, gravityZ);
    }

    if (input.peek() != std::char_traits<char>::eof()) {
      io::readPOD(input, m_deformableBodyCounter);
    }

    if (formatVersion >= 6 && input.peek() != std::char_traits<char>::eof()) {
      std::uint8_t differentiableFlag = 0;
      io::readPOD(input, differentiableFlag);
      m_differentiable = differentiableFlag != 0;
    }

    if (formatVersion >= 15 && input.peek() != std::char_traits<char>::eof()) {
      std::uint8_t rigidBodySolver = 0u;
      std::uint8_t contactSolverMethod = 0u;
      std::uint8_t contactGradientMode = 0u;
      std::uint8_t multibodyIntegrationMethod = 0u;
      io::readPOD(input, rigidBodySolver);
      io::readPOD(input, contactSolverMethod);
      io::readPOD(input, contactGradientMode);
      io::readPOD(input, multibodyIntegrationMethod);

      m_rigidBodySolver = decodeRigidBodySolver(rigidBodySolver);
      m_contactSolverMethod = decodeContactSolverMethod(contactSolverMethod);
      m_contactGradientMode = decodeContactGradientMode(contactGradientMode);
      switch (multibodyIntegrationMethod) {
        case 0u:
          m_multibodyIntegrationMethod
              = MultibodyIntegrationMethod::SemiImplicit;
          break;
        case 1u:
          m_multibodyIntegrationMethod
              = MultibodyIntegrationMethod::Variational;
          break;
        default:
          DART_SIMULATION_THROW_T(
              InvalidArgumentException,
              "Serialized World multibody integration method value is invalid");
      }
    }

    if (formatVersion >= 16 && input.peek() != std::char_traits<char>::eof()) {
      std::size_t ignoredPairCount = 0;
      io::readPOD(input, ignoredPairCount);
      for (std::size_t i = 0; i < ignoredPairCount; ++i) {
        std::uint32_t serializedFirst = 0;
        std::uint32_t serializedSecond = 0;
        io::readPOD(input, serializedFirst);
        io::readPOD(input, serializedSecond);
        const auto first
            = entityMap.at(static_cast<entt::entity>(serializedFirst));
        const auto second
            = entityMap.at(static_cast<entt::entity>(serializedSecond));
        m_storage->ignoredCollisionPairs.insert(
            makeCollisionPairKey(first, second));
      }
    }

    if (formatVersion >= 18 && input.peek() != std::char_traits<char>::eof()) {
      std::size_t variationalMaxIterations = 0;
      double variationalTolerance = 0.0;
      io::readPOD(input, variationalMaxIterations);
      io::readPOD(input, variationalTolerance);
      DART_SIMULATION_THROW_T_IF(
          variationalMaxIterations == 0,
          InvalidArgumentException,
          "Serialized World variational max-iteration budget is invalid");
      DART_SIMULATION_THROW_T_IF(
          variationalMaxIterations
              > static_cast<std::size_t>(std::numeric_limits<int>::max()),
          InvalidArgumentException,
          "Serialized World variational max-iteration budget is too large");
      DART_SIMULATION_THROW_T_IF(
          !std::isfinite(variationalTolerance) || variationalTolerance <= 0.0,
          InvalidArgumentException,
          "Serialized World variational tolerance is invalid");
      m_variationalIntegratorMaxIterations = variationalMaxIterations;
      m_variationalIntegratorTolerance = variationalTolerance;
    }

    if (formatVersion >= 23 && input.peek() != std::char_traits<char>::eof()) {
      std::uint8_t deactivationEnabled = 0u;
      DeactivationOptions options;
      io::readPOD(input, deactivationEnabled);
      io::readPOD(input, options.linearSpeedThreshold);
      io::readPOD(input, options.angularSpeedThreshold);
      io::readPOD(input, options.generalizedSpeedThreshold);
      io::readPOD(input, options.timeUntilSleep);
      io::readPOD(input, options.wakeThresholdScale);
      io::readPOD(input, options.disturbanceForceThreshold);
      options.enabled = deactivationEnabled != 0u;
      validateDeactivationOptions(options);
      m_deactivationOptions = options;
    }

    if (formatVersion >= 26 && input.peek() != std::char_traits<char>::eof()) {
      std::uint8_t computeAcceleratorPolicy = 0u;
      io::readPOD(input, computeAcceleratorPolicy);
      m_computeAcceleratorPolicy
          = decodeComputeAcceleratorPolicy(computeAcceleratorPolicy);
    }
  }

  // Ensure all frame entities have cache components (not serialized)
  auto frameView = m_storage->registry.view<comps::FrameTag>();
  for (auto entity : frameView) {
    if (!m_storage->registry.any_of<comps::FrameCache>(entity)) {
      auto& cache = m_storage->registry.emplace<comps::FrameCache>(entity);
      cache.worldTransform = Eigen::Isometry3d::Identity();
      cache.needTransformUpdate = true;
    } else {
      auto& cache = m_storage->registry.get<comps::FrameCache>(entity);
      cache.needTransformUpdate = true;
    }
  }

  resetCountersFromRegistry();
  m_storage->bakedModel.valid = false;

  if (m_simulationMode) {
    updateKinematics();
    detail::deformable_vbd::configureAvbdRigidWorldPointJointsFromCurrentPoses(
        m_storage->registry);
    prepareStepPipelineCacheForCurrentConfiguration();
  }
}

//==============================================================================
void World::resetCountersFromRegistry()
{
  m_freeFrameCounter = std::max(
      m_freeFrameCounter,
      countEntities<comps::FreeFrameTag>(m_storage->registry));
  m_fixedFrameCounter = std::max(
      m_fixedFrameCounter,
      countEntities<comps::FixedFrameTag>(m_storage->registry));
  m_multibodyCounter = std::max(
      m_multibodyCounter,
      countEntities<comps::MultibodyTag>(m_storage->registry));
  m_loopClosureCounter = std::max(
      m_loopClosureCounter,
      countEntities<comps::LoopClosure>(m_storage->registry));
  m_rigidBodyCounter = std::max(
      m_rigidBodyCounter,
      countEntities<comps::RigidBodyTag>(m_storage->registry));
  m_deformableBodyCounter = std::max(
      m_deformableBodyCounter,
      countEntities<comps::DeformableBodyTag>(m_storage->registry));
  m_linkCounter = std::max(
      m_linkCounter, countEntities<comps::LinkModel>(m_storage->registry));
  m_jointCounter = std::max(
      m_jointCounter, countEntities<comps::JointModel>(m_storage->registry));
}

} // namespace dart::simulation
