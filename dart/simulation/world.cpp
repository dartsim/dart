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
#include "dart/simulation/compute/detail/world_step_stages.hpp"
#include "dart/simulation/compute/multibody_dynamics.hpp"
#include "dart/simulation/compute/sequential_executor.hpp"
#include "dart/simulation/compute/variational_integration.hpp"
#include "dart/simulation/compute/world_kinematics_graph.hpp"
#include "dart/simulation/constraint/loop_closure.hpp"
#include "dart/simulation/constraint/loop_closure_spec.hpp"
#include "dart/simulation/detail/entity_conversion.hpp"
#include "dart/simulation/detail/memory_diagnostics.hpp"
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
#include <exception>
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
#include <type_traits>
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

void coldStartMultibodyVariationalContinuation(
    dart::simulation::detail::WorldRegistry& registry) noexcept
{
  auto variationalStates
      = registry.view<dart::simulation::compute::MultibodyVariationalState>();
  for (const entt::entity entity : variationalStates) {
    auto& state
        = variationalStates
              .get<dart::simulation::compute::MultibodyVariationalState>(
                  entity);
    state.bootstrapped = false;
    for (Eigen::Isometry3d& transform : state.previousDeltaTransform) {
      transform.setIdentity();
    }
    for (auto& momentum : state.previousMomentum) {
      momentum.setZero();
    }
  }

  auto dualStates
      = registry.view<dart::simulation::comps::VariationalContactDualState>();
  for (const entt::entity entity : dualStates) {
    auto& state
        = dualStates.get<dart::simulation::comps::VariationalContactDualState>(
            entity);
    std::fill(state.duals.begin(), state.duals.end(), 0.0);
    state.stepsSinceDualUpdate = 0u;
  }
}

class PersistentPreparationRegistrySnapshot
{
public:
  explicit PersistentPreparationRegistrySnapshot(dart::simulation::World& world)
    : m_registry(dart::simulation::detail::registryOf(world))
  {
    capture<dart::simulation::compute::MultibodyVariationalState>(
        m_variationalStates);
    capture<dart::simulation::comps::VariationalContactDualState>(
        m_variationalDuals);
    capture<dart::simulation::comps::DeformableContactConfig>(
        m_deformableContactConfigs);
  }

  void restore() noexcept
  {
    using dart::simulation::comps::DeformableBodyTag;
    using dart::simulation::comps::DeformableContactConfig;
    using dart::simulation::comps::MultibodyStructure;
    using dart::simulation::comps::VariationalContactDualState;
    using dart::simulation::compute::MultibodyVariationalState;

    for (const entt::entity entity : m_registry.view<MultibodyStructure>()) {
      if (m_registry.all_of<MultibodyVariationalState>(entity)
          && !contains(m_variationalStates, entity)) {
        m_registry.remove<MultibodyVariationalState>(entity);
      }
      if (m_registry.all_of<VariationalContactDualState>(entity)
          && !contains(m_variationalDuals, entity)) {
        m_registry.remove<VariationalContactDualState>(entity);
      }
    }
    for (auto& [entity, previous] : m_variationalStates) {
      auto& current = m_registry.get<MultibodyVariationalState>(entity);
      std::swap(current.bootstrapped, previous.bootstrapped);
      current.previousDeltaTransform.swap(previous.previousDeltaTransform);
      current.previousMomentum.swap(previous.previousMomentum);
    }
    for (auto& [entity, previous] : m_variationalDuals) {
      auto& current = m_registry.get<VariationalContactDualState>(entity);
      current.duals.swap(previous.duals);
      std::swap(current.stepsSinceDualUpdate, previous.stepsSinceDualUpdate);
    }
    for (const entt::entity entity : m_registry.view<DeformableBodyTag>()) {
      if (m_registry.all_of<DeformableContactConfig>(entity)
          && !contains(m_deformableContactConfigs, entity)) {
        m_registry.remove<DeformableContactConfig>(entity);
      }
    }
    for (const auto& [entity, previous] : m_deformableContactConfigs) {
      m_registry.get<DeformableContactConfig>(entity) = previous;
    }
  }

private:
  template <typename Component>
  using Snapshot = std::vector<std::pair<entt::entity, Component>>;

  template <typename Component>
  void capture(Snapshot<Component>& snapshot)
  {
    auto view = m_registry.view<Component>();
    std::size_t count = 0u;
    for (const entt::entity entity : view) {
      static_cast<void>(entity);
      ++count;
    }
    snapshot.reserve(count);
    for (const entt::entity entity : view) {
      snapshot.emplace_back(entity, view.template get<Component>(entity));
    }
  }

  template <typename Component>
  static bool contains(
      const Snapshot<Component>& snapshot, entt::entity entity) noexcept
  {
    for (const auto& [snapshotEntity, component] : snapshot) {
      static_cast<void>(component);
      if (snapshotEntity == entity) {
        return true;
      }
    }
    return false;
  }

  dart::simulation::detail::WorldRegistry& m_registry;
  Snapshot<dart::simulation::compute::MultibodyVariationalState>
      m_variationalStates;
  Snapshot<dart::simulation::comps::VariationalContactDualState>
      m_variationalDuals;
  Snapshot<dart::simulation::comps::DeformableContactConfig>
      m_deformableContactConfigs;
};

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
  std::size_t candidatePairCapacity = 0u;
  std::size_t contactCapacity = 0u;
  /// Storage actually reserved at bake. Equal to the capacity for explicit
  /// limits; capped by a fixed budget for automatic envelopes, which can be
  /// quadratic in the shape count and serve only as rejection thresholds.
  std::size_t candidatePairReserve = 0u;
  std::size_t contactReserve = 0u;
  bool capacitiesPrepared = false;
  bool capacitiesLocked = false;
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
  using IgnoredCollisionPairSnapshot = detail::WorldStorage::CollisionPairKey;

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
    bool hasAvbdPointJointConfig = false;
    detail::deformable_vbd::AvbdRigidWorldPointJointConfig avbdPointJointConfig;
    JointLimitsState limits;
    Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();
    Eigen::Vector3d axis2 = Eigen::Vector3d::UnitX();
    double pitch = 0.0;
    entt::entity parentLink = entt::null;
    entt::entity childLink = entt::null;
    bool hasRigidBodyPairConstraintGeometry = false;
    Eigen::Vector3d rigidBodyPairConstraintLocalAnchorParent
        = Eigen::Vector3d::Zero();
    Eigen::Vector3d rigidBodyPairConstraintLocalAnchorChild
        = Eigen::Vector3d::Zero();
    Eigen::Quaterniond rigidBodyPairConstraintTargetRelativeOrientation
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
        commandVelocity(allocator),
        commandAcceleration(allocator)
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
    VectorState commandAcceleration;
    bool broken = false;
  };

  struct LinkState
  {
    entt::entity entity = entt::null;
    comps::LinkState linkState;
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
    comps::FreeFrameProperties freeFrameProperties;
  };

  struct RigidCollisionShapeLayout
  {
    explicit RigidCollisionShapeLayout(common::MemoryAllocator& allocator)
      : vertices(SnapshotAllocator<Eigen::Vector3d>{allocator}),
        triangles(SnapshotAllocator<Eigen::Vector3i>{allocator})
    {
      // Empty.
    }

    CollisionShapeType type = CollisionShapeType::Sphere;
    double radius = 0.5;
    Eigen::Vector3d halfExtents = Eigen::Vector3d::Constant(0.5);
    Eigen::Isometry3d localTransform = Eigen::Isometry3d::Identity();
    Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
    double offset = 0.0;
    SnapshotVector<Eigen::Vector3d> vertices;
    SnapshotVector<Eigen::Vector3i> triangles;
  };

  struct RigidBodyLayout
  {
    using CollisionShapeLayout = RigidCollisionShapeLayout;

    explicit RigidBodyLayout(common::MemoryAllocator& allocator)
      : collisionShapes(SnapshotAllocator<RigidCollisionShapeLayout>{allocator})
    {
      // Empty.
    }

    entt::entity entity = entt::null;
    bool hasFrameState = false;
    bool hasTransform = false;
    bool hasVelocity = false;
    bool hasForce = false;
    bool hasMassProperties = false;
    bool hasContactMaterial = false;
    bool hasCollisionGeometry = false;
    entt::entity parentFrame = entt::null;
    comps::MassProperties massProperties;
    comps::ContactMaterial contactMaterial;
    std::uint64_t collisionGeometryRevision = 0;
    SnapshotVector<RigidCollisionShapeLayout> collisionShapes;
    bool isStatic = false;
    bool isKinematic = false;
    std::optional<double> kinematicMaxTime;
    bool hasDeformableGroundBarrier = false;
    bool hasDeformableSurfaceCcdObstacle = false;
    bool hasDeformableObstacleNoCcd = false;
    bool hasRigidAvbdContactConfig = false;
    comps::RigidAvbdContactConfig rigidAvbdContactConfig;
  };

  struct LinkLayout
  {
    using CollisionShapeLayout = RigidCollisionShapeLayout;

    explicit LinkLayout(common::MemoryAllocator& allocator)
      : name(SnapshotAllocator<char>{allocator}),
        childJoints(SnapshotAllocator<entt::entity>{allocator}),
        collisionShapes(SnapshotAllocator<RigidCollisionShapeLayout>{allocator})
    {
      // Empty.
    }

    entt::entity entity = entt::null;
    SnapshotString name;
    bool hasLinkState = false;
    bool hasLinkControl = false;
    bool hasContactMaterial = false;
    bool hasCollisionGeometry = false;
    comps::MassProperties massProperties;
    comps::ContactMaterial contactMaterial;
    Eigen::Isometry3d transformFromParentToJoint
        = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d transformFromParentJoint = Eigen::Isometry3d::Identity();
    entt::entity parentJoint = entt::null;
    SnapshotVector<entt::entity> childJoints;
    std::uint64_t collisionGeometryRevision = 0;
    SnapshotVector<RigidCollisionShapeLayout> collisionShapes;
  };

  struct RigidDistanceSpringLayout
  {
    explicit RigidDistanceSpringLayout(common::MemoryAllocator& allocator)
      : name(SnapshotAllocator<char>{allocator})
    {
      // Empty.
    }

    entt::entity entity = entt::null;
    SnapshotString name;
    detail::deformable_vbd::AvbdRigidWorldDistanceSpringConfig config;
  };

  struct MultibodyStructureLayout
  {
    explicit MultibodyStructureLayout(common::MemoryAllocator& allocator)
      : links(SnapshotAllocator<entt::entity>{allocator}),
        joints(SnapshotAllocator<entt::entity>{allocator}),
        contactPointLinkIndices(SnapshotAllocator<std::size_t>{allocator}),
        contactPointLocalPositions(
            SnapshotAllocator<Eigen::Vector3d>{allocator})
    {
      // Empty.
    }

    entt::entity entity = entt::null;
    bool hasMultibodyTag = false;
    bool hasVariationalContact = false;
    SnapshotVector<entt::entity> links;
    SnapshotVector<entt::entity> joints;
    Eigen::Vector3d contactPlaneNormal = Eigen::Vector3d::UnitZ();
    Eigen::Vector3d contactPlanePoint = Eigen::Vector3d::Zero();
    double contactStiffness = 0.0;
    double contactFrictionCoefficient = 0.0;
    double contactFrictionRegularization = 1.0e-4;
    double contactDampingCoefficient = 0.0;
    std::size_t contactDualUpdateCadence = 0u;
    SnapshotVector<std::size_t> contactPointLinkIndices;
    SnapshotVector<Eigen::Vector3d> contactPointLocalPositions;
  };

  struct DeformableNodeStateSnapshot
  {
    explicit DeformableNodeStateSnapshot(common::MemoryAllocator& allocator)
      : positions(SnapshotAllocator<Eigen::Vector3d>{allocator}),
        previousPositions(SnapshotAllocator<Eigen::Vector3d>{allocator}),
        velocities(SnapshotAllocator<Eigen::Vector3d>{allocator}),
        attachmentTargets(SnapshotAllocator<Eigen::Vector3d>{allocator}),
        masses(SnapshotAllocator<double>{allocator}),
        fixed(SnapshotAllocator<std::uint8_t>{allocator})
    {
      // Empty.
    }

    SnapshotVector<Eigen::Vector3d> positions;
    SnapshotVector<Eigen::Vector3d> previousPositions;
    SnapshotVector<Eigen::Vector3d> velocities;
    SnapshotVector<Eigen::Vector3d> attachmentTargets;
    SnapshotVector<double> masses;
    SnapshotVector<std::uint8_t> fixed;
    std::optional<comps::DeformableVbdConfig> vbdConfig;
  };

  using DeformableNodeStateSnapshotEntry
      = std::pair<entt::entity, DeformableNodeStateSnapshot>;

  struct DeformableDirichletLayout
  {
    explicit DeformableDirichletLayout(common::MemoryAllocator& allocator)
      : nodes(SnapshotAllocator<std::size_t>{allocator}),
        referencePositions(SnapshotAllocator<Eigen::Vector3d>{allocator})
    {
      // Empty.
    }

    SnapshotVector<std::size_t> nodes;
    SnapshotVector<Eigen::Vector3d> referencePositions;
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    Eigen::Vector3d linearVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d angularVelocity = Eigen::Vector3d::Zero();
    double startTime = 0.0;
    double endTime = 0.0;
  };

  struct DeformableNeumannLayout
  {
    explicit DeformableNeumannLayout(common::MemoryAllocator& allocator)
      : nodes(SnapshotAllocator<std::size_t>{allocator})
    {
      // Empty.
    }

    SnapshotVector<std::size_t> nodes;
    Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
    double startTime = 0.0;
    double endTime = 0.0;
  };

  struct DeformableLayout
  {
    using DirichletLayout = DeformableDirichletLayout;
    using NeumannLayout = DeformableNeumannLayout;

    explicit DeformableLayout(common::MemoryAllocator& allocator)
      : masses(SnapshotAllocator<double>{allocator}),
        fixed(SnapshotAllocator<std::uint8_t>{allocator}),
        springEdges(SnapshotAllocator<comps::DeformableSpringEdge>{allocator}),
        restPositions(SnapshotAllocator<Eigen::Vector3d>{allocator}),
        surfaceTriangles(
            SnapshotAllocator<comps::DeformableSurfaceTriangle>{allocator}),
        tetrahedra(SnapshotAllocator<comps::DeformableTetrahedron>{allocator}),
        tetrahedronRestVolumes(SnapshotAllocator<double>{allocator}),
        dirichlet(SnapshotAllocator<DeformableDirichletLayout>{allocator}),
        neumann(SnapshotAllocator<DeformableNeumannLayout>{allocator})
    {
      // Empty.
    }

    entt::entity entity = entt::null;
    bool hasBodyTag = false;
    bool hasNodeState = false;
    bool hasNodeModel = false;
    bool hasSpringModel = false;
    bool hasMeshTopology = false;
    bool hasMaterial = false;
    bool hasContactConfig = false;
    bool hasBoundaryConditions = false;

    std::size_t positionCount = 0;
    std::size_t previousPositionCount = 0;
    std::size_t velocityCount = 0;
    std::size_t attachmentTargetCount = 0;
    SnapshotVector<double> masses;
    SnapshotVector<std::uint8_t> fixed;

    SnapshotVector<comps::DeformableSpringEdge> springEdges;
    double springStiffness = 0.0;
    double springDamping = 0.0;

    SnapshotVector<Eigen::Vector3d> restPositions;
    SnapshotVector<comps::DeformableSurfaceTriangle> surfaceTriangles;
    SnapshotVector<comps::DeformableTetrahedron> tetrahedra;
    SnapshotVector<double> tetrahedronRestVolumes;

    comps::DeformableMaterial material;
    comps::DeformableContactConfig contactConfig;
    SnapshotVector<DeformableDirichletLayout> dirichlet;
    SnapshotVector<DeformableNeumannLayout> neumann;
  };

  struct DeformableComponentCounts
  {
    std::size_t bodyTags = 0;
    std::size_t nodeStates = 0;
    std::size_t nodeModels = 0;
    std::size_t springModels = 0;
    std::size_t meshTopologies = 0;
    std::size_t materials = 0;
    std::size_t contactConfigs = 0;
    std::size_t boundaryConditions = 0;
  };

  struct Frame
  {
    explicit Frame(common::MemoryAllocator& allocator)
      : lastContactForces(SnapshotAllocator<ContactForce>{allocator}),
        differentiableParameters(
            SnapshotAllocator<DifferentiableParameterSnapshot>{allocator}),
        deformableNodeStates(
            SnapshotAllocator<DeformableNodeStateSnapshotEntry>{allocator}),
        deformableAvbdWarmStartStates(
            SnapshotAllocator<
                compute::avbd_replay::DeformableAvbdWarmStartReplayState>{
                allocator}),
        rigidAvbdWarmStartState(allocator),
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
        kinematicBodyStepTraces(
            SnapshotAllocator<
                std::pair<entt::entity, comps::KinematicBodyStepTrace>>{
                allocator}),
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
    std::size_t rigidConstraintIterations = 8;
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
    bool stepDerivativesValid = false;
    compute::StepMetrics lastStepDiagnostics;
    SnapshotVector<ContactForce> lastContactForces;
    SnapshotVector<DifferentiableParameterSnapshot> differentiableParameters;

    SnapshotVector<DeformableNodeStateSnapshotEntry> deformableNodeStates;
    SnapshotVector<compute::avbd_replay::DeformableAvbdWarmStartReplayState>
        deformableAvbdWarmStartStates;
    compute::avbd_replay::RigidAvbdWarmStartReplayState rigidAvbdWarmStartState;
    ComponentSnapshot<compute::MultibodyVariationalState>
        multibodyVariationalStates;
    ComponentSnapshot<comps::VariationalContactDualState>
        variationalContactDualStates;
    ComponentSnapshot<comps::DeactivationState> deactivationStates;
    ComponentSnapshot<comps::KinematicBodyStepTrace> kinematicBodyStepTraces;
    SnapshotVector<JointState> joints;
    SnapshotVector<LinkState> links;
    SnapshotVector<PublicFrameState> publicFrames;
    SnapshotVector<LoopClosureState> loopClosures;
    SnapshotVector<RigidBodyState> rigidBodies;
  };

  using FrameAllocator = common::StlAllocator<Frame>;

  explicit ReplayState(common::MemoryAllocator& allocator)
    : rigidBodyLayouts(SnapshotAllocator<RigidBodyLayout>{allocator}),
      linkLayouts(SnapshotAllocator<LinkLayout>{allocator}),
      rigidDistanceSpringLayouts(
          SnapshotAllocator<RigidDistanceSpringLayout>{allocator}),
      multibodyStructureLayouts(
          SnapshotAllocator<MultibodyStructureLayout>{allocator}),
      deformableLayouts(SnapshotAllocator<DeformableLayout>{allocator}),
      ignoredCollisionPairs(
          SnapshotAllocator<IgnoredCollisionPairSnapshot>{allocator}),
      frames(FrameAllocator{allocator})
  {
    // Empty.
  }

  bool recordingEnabled = false;
  bool hasConstructionSnapshot = false;
  std::size_t rigidBodyCount = 0;
  SnapshotVector<RigidBodyLayout> rigidBodyLayouts;
  std::size_t linkCount = 0;
  SnapshotVector<LinkLayout> linkLayouts;
  std::size_t rigidDistanceSpringCount = 0;
  SnapshotVector<RigidDistanceSpringLayout> rigidDistanceSpringLayouts;
  std::size_t multibodyStructureCount = 0;
  SnapshotVector<MultibodyStructureLayout> multibodyStructureLayouts;
  DeformableComponentCounts deformableComponentCounts;
  SnapshotVector<DeformableLayout> deformableLayouts;
  SnapshotVector<IgnoredCollisionPairSnapshot> ignoredCollisionPairs;
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

bool sameReplayAvbdPointJointConfig(
    const detail::deformable_vbd::AvbdRigidWorldPointJointConfig& lhs,
    const detail::deformable_vbd::AvbdRigidWorldPointJointConfig& rhs)
{
  return lhs.enabled == rhs.enabled
         && lhs.localAnchorA.isApprox(rhs.localAnchorA, 0.0)
         && lhs.localAnchorB.isApprox(rhs.localAnchorB, 0.0)
         && lhs.targetRelativeOrientation.coeffs().isApprox(
             rhs.targetRelativeOrientation.coeffs(), 0.0)
         && lhs.linearAxes.isApprox(rhs.linearAxes, 0.0)
         && lhs.angularAxes.isApprox(rhs.angularAxes, 0.0)
         && lhs.linearAxisMask == rhs.linearAxisMask
         && lhs.angularAxisMask == rhs.angularAxisMask
         && lhs.startStiffness == rhs.startStiffness
         && lhs.linearMaterialStiffness == rhs.linearMaterialStiffness
         && lhs.angularMaterialStiffness == rhs.angularMaterialStiffness
         && lhs.maxStiffness == rhs.maxStiffness;
}

template <typename JointLayout>
bool sameReplayJointLayout(
    const comps::JointModel& jointModel,
    const comps::JointActuation& jointActuation,
    const comps::AvbdJointStiffness* avbdStiffness,
    const detail::deformable_vbd::AvbdRigidWorldPointJointConfig*
        avbdPointJointConfig,
    const JointLayout& layout)
{
  const bool hasAvbdStiffnessState = avbdStiffness != nullptr;
  const comps::AvbdJointStiffness resolvedStiffness
      = hasAvbdStiffnessState ? *avbdStiffness : comps::AvbdJointStiffness{};
  const bool hasAvbdPointJointConfig = avbdPointJointConfig != nullptr;
  const auto resolvedPointJointConfig
      = hasAvbdPointJointConfig
            ? *avbdPointJointConfig
            : detail::deformable_vbd::AvbdRigidWorldPointJointConfig{};
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
         && hasAvbdPointJointConfig == layout.hasAvbdPointJointConfig
         && sameReplayAvbdPointJointConfig(
             resolvedPointJointConfig, layout.avbdPointJointConfig)
         && sameReplayJointLimits(jointModel.limits, layout.limits)
         && jointModel.axis.isApprox(layout.axis, 0.0)
         && jointModel.axis2.isApprox(layout.axis2, 0.0)
         && jointModel.pitch == layout.pitch
         && jointModel.parentLink == layout.parentLink
         && jointModel.childLink == layout.childLink
         && jointModel.hasRigidBodyPairConstraintGeometry
                == layout.hasRigidBodyPairConstraintGeometry
         && jointModel.rigidBodyPairConstraintLocalAnchorParent.isApprox(
             layout.rigidBodyPairConstraintLocalAnchorParent, 0.0)
         && jointModel.rigidBodyPairConstraintLocalAnchorChild.isApprox(
             layout.rigidBodyPairConstraintLocalAnchorChild, 0.0)
         && jointModel.rigidBodyPairConstraintTargetRelativeOrientation.coeffs()
                .isApprox(
                    layout.rigidBodyPairConstraintTargetRelativeOrientation
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

template <typename Component, typename EntityVector>
void appendReplayComponentEntities(
    const detail::WorldRegistry& registry, EntityVector& entities)
{
  const auto view = registry.view<Component>();
  for (const entt::entity entity : view) {
    entities.push_back(entity);
  }
}

template <typename DeformableLayout, typename ComponentCounts>
auto captureReplayDeformableLayouts(
    const detail::WorldRegistry& registry,
    common::MemoryAllocator& allocator,
    ComponentCounts& counts)
{
  ReplayScratchVector<entt::entity> entities(
      ReplayScratchAllocator<entt::entity>{allocator});
  counts.bodyTags = countReplayView(registry.view<comps::DeformableBodyTag>());
  counts.nodeStates
      = countReplayView(registry.view<comps::DeformableNodeState>());
  counts.nodeModels
      = countReplayView(registry.view<comps::DeformableNodeModel>());
  counts.springModels
      = countReplayView(registry.view<comps::DeformableSpringModel>());
  counts.meshTopologies
      = countReplayView(registry.view<comps::DeformableMeshTopology>());
  counts.materials
      = countReplayView(registry.view<comps::DeformableMaterial>());
  counts.contactConfigs
      = countReplayView(registry.view<comps::DeformableContactConfig>());
  counts.boundaryConditions
      = countReplayView(registry.view<comps::DeformableBoundaryConditions>());
  entities.reserve(
      counts.bodyTags + counts.nodeStates + counts.nodeModels
      + counts.springModels + counts.meshTopologies + counts.materials
      + counts.contactConfigs + counts.boundaryConditions);
  appendReplayComponentEntities<comps::DeformableBodyTag>(registry, entities);
  appendReplayComponentEntities<comps::DeformableNodeState>(registry, entities);
  appendReplayComponentEntities<comps::DeformableNodeModel>(registry, entities);
  appendReplayComponentEntities<comps::DeformableSpringModel>(
      registry, entities);
  appendReplayComponentEntities<comps::DeformableMeshTopology>(
      registry, entities);
  appendReplayComponentEntities<comps::DeformableMaterial>(registry, entities);
  appendReplayComponentEntities<comps::DeformableContactConfig>(
      registry, entities);
  appendReplayComponentEntities<comps::DeformableBoundaryConditions>(
      registry, entities);
  std::ranges::sort(entities, [](entt::entity lhs, entt::entity rhs) {
    return static_cast<std::uint32_t>(lhs) < static_cast<std::uint32_t>(rhs);
  });
  entities.erase(std::unique(entities.begin(), entities.end()), entities.end());

  ReplayScratchVector<DeformableLayout> layouts(
      ReplayScratchAllocator<DeformableLayout>{allocator});
  layouts.reserve(entities.size());
  for (const entt::entity entity : entities) {
    DeformableLayout layout(allocator);
    layout.entity = entity;
    layout.hasBodyTag = registry.all_of<comps::DeformableBodyTag>(entity);
    layout.hasNodeState = registry.all_of<comps::DeformableNodeState>(entity);
    layout.hasNodeModel = registry.all_of<comps::DeformableNodeModel>(entity);
    layout.hasSpringModel
        = registry.all_of<comps::DeformableSpringModel>(entity);
    layout.hasMeshTopology
        = registry.all_of<comps::DeformableMeshTopology>(entity);
    layout.hasMaterial = registry.all_of<comps::DeformableMaterial>(entity);
    layout.hasContactConfig
        = registry.all_of<comps::DeformableContactConfig>(entity);
    layout.hasBoundaryConditions
        = registry.all_of<comps::DeformableBoundaryConditions>(entity);

    if (const auto* state
        = registry.try_get<comps::DeformableNodeState>(entity)) {
      layout.positionCount = state->positions.size();
      layout.previousPositionCount = state->previousPositions.size();
      layout.velocityCount = state->velocities.size();
      layout.attachmentTargetCount = state->attachmentTargets.size();
    }
    if (const auto* model
        = registry.try_get<comps::DeformableNodeModel>(entity)) {
      layout.masses.assign(model->masses.begin(), model->masses.end());
      layout.fixed.assign(model->fixed.begin(), model->fixed.end());
    }
    if (const auto* springs
        = registry.try_get<comps::DeformableSpringModel>(entity)) {
      layout.springEdges.assign(springs->edges.begin(), springs->edges.end());
      layout.springStiffness = springs->stiffness;
      layout.springDamping = springs->damping;
    }
    if (const auto* mesh
        = registry.try_get<comps::DeformableMeshTopology>(entity)) {
      layout.restPositions.assign(
          mesh->restPositions.begin(), mesh->restPositions.end());
      layout.surfaceTriangles.assign(
          mesh->surfaceTriangles.begin(), mesh->surfaceTriangles.end());
      layout.tetrahedra.assign(
          mesh->tetrahedra.begin(), mesh->tetrahedra.end());
      layout.tetrahedronRestVolumes.assign(
          mesh->tetrahedronRestVolumes.begin(),
          mesh->tetrahedronRestVolumes.end());
    }
    if (const auto* material
        = registry.try_get<comps::DeformableMaterial>(entity)) {
      layout.material = *material;
    }
    if (const auto* config
        = registry.try_get<comps::DeformableContactConfig>(entity)) {
      layout.contactConfig = *config;
    }
    if (const auto* boundaries
        = registry.try_get<comps::DeformableBoundaryConditions>(entity)) {
      layout.dirichlet.reserve(boundaries->dirichlet.size());
      for (const auto& source : boundaries->dirichlet) {
        typename DeformableLayout::DirichletLayout target(allocator);
        target.nodes.assign(source.nodes.begin(), source.nodes.end());
        target.referencePositions.assign(
            source.referencePositions.begin(), source.referencePositions.end());
        target.center = source.center;
        target.linearVelocity = source.linearVelocity;
        target.angularVelocity = source.angularVelocity;
        target.startTime = source.startTime;
        target.endTime = source.endTime;
        layout.dirichlet.push_back(std::move(target));
      }
      layout.neumann.reserve(boundaries->neumann.size());
      for (const auto& source : boundaries->neumann) {
        typename DeformableLayout::NeumannLayout target(allocator);
        target.nodes.assign(source.nodes.begin(), source.nodes.end());
        target.acceleration = source.acceleration;
        target.startTime = source.startTime;
        target.endTime = source.endTime;
        layout.neumann.push_back(std::move(target));
      }
    }
    layouts.push_back(std::move(layout));
  }
  return layouts;
}

template <typename LhsVector, typename RhsVector, typename Equal>
bool sameReplaySequence(
    const LhsVector& lhs, const RhsVector& rhs, Equal&& equal)
{
  return lhs.size() == rhs.size()
         && std::ranges::equal(lhs, rhs, std::forward<Equal>(equal));
}

template <typename LhsVector, typename RhsVector>
bool sameReplayVector3Sequence(const LhsVector& lhs, const RhsVector& rhs)
{
  return sameReplaySequence(lhs, rhs, [](const auto& a, const auto& b) {
    return a.isApprox(b, 0.0);
  });
}

bool sameReplayDeformableMaterial(
    const comps::DeformableMaterial& lhs, const comps::DeformableMaterial& rhs)
{
  return lhs.density == rhs.density && lhs.youngsModulus == rhs.youngsModulus
         && lhs.poissonRatio == rhs.poissonRatio
         && lhs.frictionCoefficient == rhs.frictionCoefficient
         && lhs.useFiniteElementElasticity == rhs.useFiniteElementElasticity
         && lhs.useFixedCorotationalElasticity
                == rhs.useFixedCorotationalElasticity
         && lhs.useAdaptiveBarrierStiffness == rhs.useAdaptiveBarrierStiffness
         && lhs.useIterativeLinearSolver == rhs.useIterativeLinearSolver
         && lhs.useMatrixFreeLinearSolver == rhs.useMatrixFreeLinearSolver;
}

template <typename Layout>
bool sameReplayDeformableLayoutPayload(
    const detail::WorldRegistry& registry, const Layout& layout)
{
  if (layout.hasNodeState) {
    const auto& state = registry.get<comps::DeformableNodeState>(layout.entity);
    if (state.positions.size() != layout.positionCount
        || state.previousPositions.size() != layout.previousPositionCount
        || state.velocities.size() != layout.velocityCount
        || state.attachmentTargets.size() != layout.attachmentTargetCount) {
      return false;
    }
  }
  if (layout.hasNodeModel) {
    const auto& model = registry.get<comps::DeformableNodeModel>(layout.entity);
    if (!std::ranges::equal(model.masses, layout.masses)
        || !std::ranges::equal(model.fixed, layout.fixed)) {
      return false;
    }
  }
  if (layout.hasSpringModel) {
    const auto& springs
        = registry.get<comps::DeformableSpringModel>(layout.entity);
    const bool sameEdges = sameReplaySequence(
        springs.edges,
        layout.springEdges,
        [](const auto& lhs, const auto& rhs) {
          return lhs.nodeA == rhs.nodeA && lhs.nodeB == rhs.nodeB
                 && lhs.restLength == rhs.restLength;
        });
    if (!sameEdges || springs.stiffness != layout.springStiffness
        || springs.damping != layout.springDamping) {
      return false;
    }
  }
  if (layout.hasMeshTopology) {
    const auto& mesh
        = registry.get<comps::DeformableMeshTopology>(layout.entity);
    const bool sameTriangles = sameReplaySequence(
        mesh.surfaceTriangles,
        layout.surfaceTriangles,
        [](const auto& lhs, const auto& rhs) {
          return lhs.nodeA == rhs.nodeA && lhs.nodeB == rhs.nodeB
                 && lhs.nodeC == rhs.nodeC;
        });
    const bool sameTetrahedra = sameReplaySequence(
        mesh.tetrahedra,
        layout.tetrahedra,
        [](const auto& lhs, const auto& rhs) {
          return lhs.nodeA == rhs.nodeA && lhs.nodeB == rhs.nodeB
                 && lhs.nodeC == rhs.nodeC && lhs.nodeD == rhs.nodeD;
        });
    if (!sameReplayVector3Sequence(mesh.restPositions, layout.restPositions)
        || !sameTriangles || !sameTetrahedra
        || !std::ranges::equal(
            mesh.tetrahedronRestVolumes, layout.tetrahedronRestVolumes)) {
      return false;
    }
  }
  if (layout.hasMaterial
      && !sameReplayDeformableMaterial(
          registry.get<comps::DeformableMaterial>(layout.entity),
          layout.material)) {
    return false;
  }
  if (layout.hasContactConfig
      && registry.get<comps::DeformableContactConfig>(layout.entity)
                 .surfaceCandidateCapacity
             != layout.contactConfig.surfaceCandidateCapacity) {
    return false;
  }
  if (layout.hasBoundaryConditions) {
    const auto& boundaries
        = registry.get<comps::DeformableBoundaryConditions>(layout.entity);
    const bool sameDirichlet = sameReplaySequence(
        boundaries.dirichlet,
        layout.dirichlet,
        [](const auto& lhs, const auto& rhs) {
          return std::ranges::equal(lhs.nodes, rhs.nodes)
                 && sameReplayVector3Sequence(
                     lhs.referencePositions, rhs.referencePositions)
                 && lhs.center.isApprox(rhs.center, 0.0)
                 && lhs.linearVelocity.isApprox(rhs.linearVelocity, 0.0)
                 && lhs.angularVelocity.isApprox(rhs.angularVelocity, 0.0)
                 && lhs.startTime == rhs.startTime
                 && lhs.endTime == rhs.endTime;
        });
    const bool sameNeumann = sameReplaySequence(
        boundaries.neumann,
        layout.neumann,
        [](const auto& lhs, const auto& rhs) {
          return std::ranges::equal(lhs.nodes, rhs.nodes)
                 && lhs.acceleration.isApprox(rhs.acceleration, 0.0)
                 && lhs.startTime == rhs.startTime
                 && lhs.endTime == rhs.endTime;
        });
    if (!sameDirichlet || !sameNeumann) {
      return false;
    }
  }
  return true;
}

template <typename ComponentCounts, typename Layouts>
void validateReplayDeformableLayouts(
    const detail::WorldRegistry& registry,
    const ComponentCounts& counts,
    const Layouts& layouts)
{
  const bool componentCountsChanged
      = countReplayView(registry.view<comps::DeformableBodyTag>())
            != counts.bodyTags
        || countReplayView(registry.view<comps::DeformableNodeState>())
               != counts.nodeStates
        || countReplayView(registry.view<comps::DeformableNodeModel>())
               != counts.nodeModels
        || countReplayView(registry.view<comps::DeformableSpringModel>())
               != counts.springModels
        || countReplayView(registry.view<comps::DeformableMeshTopology>())
               != counts.meshTopologies
        || countReplayView(registry.view<comps::DeformableMaterial>())
               != counts.materials
        || countReplayView(registry.view<comps::DeformableContactConfig>())
               != counts.contactConfigs
        || countReplayView(registry.view<comps::DeformableBoundaryConditions>())
               != counts.boundaryConditions;
  DART_SIMULATION_THROW_T_IF(
      componentCountsChanged,
      InvalidOperationException,
      "Cannot continue replay: deformable construction component counts "
      "changed");

  for (const auto& layout : layouts) {
    const bool entityValid = registry.valid(layout.entity);
    const bool componentLayoutChanged
        = !entityValid
          || registry.all_of<comps::DeformableBodyTag>(layout.entity)
                 != layout.hasBodyTag
          || registry.all_of<comps::DeformableNodeState>(layout.entity)
                 != layout.hasNodeState
          || registry.all_of<comps::DeformableNodeModel>(layout.entity)
                 != layout.hasNodeModel
          || registry.all_of<comps::DeformableSpringModel>(layout.entity)
                 != layout.hasSpringModel
          || registry.all_of<comps::DeformableMeshTopology>(layout.entity)
                 != layout.hasMeshTopology
          || registry.all_of<comps::DeformableMaterial>(layout.entity)
                 != layout.hasMaterial
          || registry.all_of<comps::DeformableContactConfig>(layout.entity)
                 != layout.hasContactConfig
          || registry.all_of<comps::DeformableBoundaryConditions>(layout.entity)
                 != layout.hasBoundaryConditions;
    DART_SIMULATION_THROW_T_IF(
        componentLayoutChanged,
        InvalidOperationException,
        "Cannot continue replay: deformable construction component layout "
        "changed");
    DART_SIMULATION_THROW_T_IF(
        !sameReplayDeformableLayoutPayload(registry, layout),
        InvalidOperationException,
        "Cannot continue replay: deformable construction data changed");
  }
}

bool sameReplayDeformableVbdConfig(
    const std::optional<comps::DeformableVbdConfig>& lhs,
    const comps::DeformableVbdConfig* rhs)
{
  if (lhs.has_value() != (rhs != nullptr)) {
    return false;
  }
  if (!lhs) {
    return true;
  }

  return lhs->enabled == rhs->enabled && lhs->iterations == rhs->iterations
         && lhs->convergenceDisplacement == rhs->convergenceDisplacement
         && lhs->useChebyshev == rhs->useChebyshev
         && lhs->chebyshevRho == rhs->chebyshevRho
         && lhs->rayleighDamping == rhs->rayleighDamping
         && lhs->contactStiffness == rhs->contactStiffness
         && lhs->requireVbdExecution == rhs->requireVbdExecution
         && lhs->useAvbdContactNormalRows == rhs->useAvbdContactNormalRows
         && lhs->useAvbdSelfContactNormalRows
                == rhs->useAvbdSelfContactNormalRows
         && lhs->useAvbdAttachmentRows == rhs->useAvbdAttachmentRows
         && lhs->avbdAttachmentStiffness == rhs->avbdAttachmentStiffness
         && lhs->useAvbdFiniteStiffnessRows == rhs->useAvbdFiniteStiffnessRows
         && lhs->avbdFiniteStiffnessStart == rhs->avbdFiniteStiffnessStart
         && lhs->avbdAlpha == rhs->avbdAlpha && lhs->avbdBeta == rhs->avbdBeta
         && lhs->avbdGamma == rhs->avbdGamma
         && lhs->avbdMaxStiffness == rhs->avbdMaxStiffness;
}

bool sameReplayMassProperties(
    const comps::MassProperties& lhs, const comps::MassProperties& rhs)
{
  return lhs.mass == rhs.mass && lhs.inertia.isApprox(rhs.inertia, 0.0)
         && lhs.localCenterOfMass.isApprox(rhs.localCenterOfMass, 0.0);
}

template <typename RigidBodyLayout>
auto captureReplayRigidBodyLayouts(
    const detail::WorldRegistry& registry,
    common::MemoryAllocator& allocator,
    std::size_t& rigidBodyCount)
{
  ReplayScratchVector<RigidBodyLayout> layouts(
      ReplayScratchAllocator<RigidBodyLayout>{allocator});
  const auto view = registry.view<comps::RigidBodyTag>();
  rigidBodyCount = countReplayView(view);
  layouts.reserve(rigidBodyCount);
  for (const entt::entity entity : view) {
    RigidBodyLayout layout(allocator);
    layout.entity = entity;
    layout.hasFrameState = registry.all_of<comps::FrameState>(entity);
    layout.hasTransform = registry.all_of<comps::Transform>(entity);
    layout.hasVelocity = registry.all_of<comps::Velocity>(entity);
    layout.hasForce = registry.all_of<comps::Force>(entity);
    layout.hasMassProperties = registry.all_of<comps::MassProperties>(entity);
    layout.hasContactMaterial = registry.all_of<comps::ContactMaterial>(entity);
    layout.hasCollisionGeometry
        = registry.all_of<comps::CollisionGeometry>(entity);
    layout.isStatic = registry.all_of<comps::StaticBodyTag>(entity);
    layout.isKinematic = registry.all_of<comps::KinematicBodyTag>(entity);
    layout.hasDeformableGroundBarrier
        = registry.all_of<comps::DeformableGroundBarrierTag>(entity);
    layout.hasDeformableSurfaceCcdObstacle
        = registry.all_of<comps::DeformableSurfaceCcdObstacleTag>(entity);
    layout.hasDeformableObstacleNoCcd
        = registry.all_of<comps::DeformableObstacleNoCcdTag>(entity);
    layout.hasRigidAvbdContactConfig
        = registry.all_of<comps::RigidAvbdContactConfig>(entity);

    if (const auto* frameState = registry.try_get<comps::FrameState>(entity)) {
      layout.parentFrame = frameState->parentFrame;
    }
    if (const auto* mass = registry.try_get<comps::MassProperties>(entity)) {
      layout.massProperties = *mass;
    }
    if (const auto* material
        = registry.try_get<comps::ContactMaterial>(entity)) {
      layout.contactMaterial = *material;
    }
    if (const auto* kinematic
        = registry.try_get<comps::KinematicBodyTag>(entity)) {
      layout.kinematicMaxTime = kinematic->maxTime;
    }
    if (const auto* config
        = registry.try_get<comps::RigidAvbdContactConfig>(entity)) {
      layout.rigidAvbdContactConfig = *config;
    }
    if (const auto* geometry
        = registry.try_get<comps::CollisionGeometry>(entity)) {
      layout.collisionGeometryRevision = geometry->revision;
      layout.collisionShapes.reserve(geometry->shapes.size());
      for (const CollisionShape& source : geometry->shapes) {
        typename RigidBodyLayout::CollisionShapeLayout target(allocator);
        target.type = source.type;
        target.radius = source.radius;
        target.halfExtents = source.halfExtents;
        target.localTransform = source.localTransform;
        target.normal = source.normal;
        target.offset = source.offset;
        target.vertices.assign(source.vertices.begin(), source.vertices.end());
        target.triangles.assign(
            source.triangles.begin(), source.triangles.end());
        layout.collisionShapes.push_back(std::move(target));
      }
    }
    layouts.push_back(std::move(layout));
  }
  std::ranges::sort(layouts, [](const auto& lhs, const auto& rhs) {
    return static_cast<std::uint32_t>(lhs.entity)
           < static_cast<std::uint32_t>(rhs.entity);
  });
  return layouts;
}

template <typename CollisionShapeLayout>
bool sameReplayRigidCollisionShape(
    const CollisionShape& shape, const CollisionShapeLayout& layout)
{
  return shape.type == layout.type && shape.radius == layout.radius
         && shape.halfExtents.isApprox(layout.halfExtents, 0.0)
         && shape.localTransform.matrix().isApprox(
             layout.localTransform.matrix(), 0.0)
         && shape.normal.isApprox(layout.normal, 0.0)
         && shape.offset == layout.offset
         && sameReplayVector3Sequence(shape.vertices, layout.vertices)
         && sameReplaySequence(
             shape.triangles,
             layout.triangles,
             [](const auto& lhs, const auto& rhs) {
               return (lhs.array() == rhs.array()).all();
             });
}

template <typename Layouts>
void validateReplayRigidBodyLayouts(
    const detail::WorldRegistry& registry,
    std::size_t rigidBodyCount,
    const Layouts& layouts)
{
  DART_SIMULATION_THROW_T_IF(
      countReplayView(registry.view<comps::RigidBodyTag>()) != rigidBodyCount
          || layouts.size() != rigidBodyCount,
      InvalidOperationException,
      "Cannot continue replay: rigid-body construction count changed");
  for (const auto& layout : layouts) {
    const bool entityValid = registry.valid(layout.entity);
    const bool componentLayoutChanged
        = !entityValid || !registry.all_of<comps::RigidBodyTag>(layout.entity)
          || registry.all_of<comps::FrameState>(layout.entity)
                 != layout.hasFrameState
          || registry.all_of<comps::Transform>(layout.entity)
                 != layout.hasTransform
          || registry.all_of<comps::Velocity>(layout.entity)
                 != layout.hasVelocity
          || registry.all_of<comps::Force>(layout.entity) != layout.hasForce
          || registry.all_of<comps::MassProperties>(layout.entity)
                 != layout.hasMassProperties
          || registry.all_of<comps::ContactMaterial>(layout.entity)
                 != layout.hasContactMaterial
          || registry.all_of<comps::CollisionGeometry>(layout.entity)
                 != layout.hasCollisionGeometry
          || registry.all_of<comps::StaticBodyTag>(layout.entity)
                 != layout.isStatic
          || registry.all_of<comps::KinematicBodyTag>(layout.entity)
                 != layout.isKinematic
          || registry.all_of<comps::DeformableGroundBarrierTag>(layout.entity)
                 != layout.hasDeformableGroundBarrier
          || registry.all_of<comps::DeformableSurfaceCcdObstacleTag>(
                 layout.entity)
                 != layout.hasDeformableSurfaceCcdObstacle
          || registry.all_of<comps::DeformableObstacleNoCcdTag>(layout.entity)
                 != layout.hasDeformableObstacleNoCcd
          || registry.all_of<comps::RigidAvbdContactConfig>(layout.entity)
                 != layout.hasRigidAvbdContactConfig;
    DART_SIMULATION_THROW_T_IF(
        componentLayoutChanged,
        InvalidOperationException,
        "Cannot continue replay: rigid-body construction component layout "
        "changed");

    bool constructionChanged = false;
    if (layout.hasFrameState) {
      constructionChanged
          = registry.get<comps::FrameState>(layout.entity).parentFrame
            != layout.parentFrame;
    }
    if (layout.hasMassProperties) {
      constructionChanged
          = constructionChanged
            || !sameReplayMassProperties(
                registry.get<comps::MassProperties>(layout.entity),
                layout.massProperties);
    }
    if (layout.hasContactMaterial) {
      const auto& material
          = registry.get<comps::ContactMaterial>(layout.entity);
      constructionChanged
          = constructionChanged
            || material.restitution != layout.contactMaterial.restitution
            || material.friction != layout.contactMaterial.friction;
    }
    if (layout.isKinematic) {
      constructionChanged
          = constructionChanged
            || registry.get<comps::KinematicBodyTag>(layout.entity).maxTime
                   != layout.kinematicMaxTime;
    }
    if (layout.hasRigidAvbdContactConfig) {
      const auto& config
          = registry.get<comps::RigidAvbdContactConfig>(layout.entity);
      constructionChanged
          = constructionChanged
            || config.enabled != layout.rigidAvbdContactConfig.enabled
            || config.startStiffness
                   != layout.rigidAvbdContactConfig.startStiffness
            || config.alpha != layout.rigidAvbdContactConfig.alpha
            || config.beta != layout.rigidAvbdContactConfig.beta
            || config.gamma != layout.rigidAvbdContactConfig.gamma
            || config.maxStiffness
                   != layout.rigidAvbdContactConfig.maxStiffness;
    }
    if (layout.hasCollisionGeometry) {
      const auto& geometry
          = registry.get<comps::CollisionGeometry>(layout.entity);
      constructionChanged
          = constructionChanged
            || geometry.revision != layout.collisionGeometryRevision
            || geometry.shapes.size() != layout.collisionShapes.size();
      if (!constructionChanged) {
        for (std::size_t i = 0; i < geometry.shapes.size(); ++i) {
          if (!sameReplayRigidCollisionShape(
                  geometry.shapes[i], layout.collisionShapes[i])) {
            constructionChanged = true;
            break;
          }
        }
      }
    }
    DART_SIMULATION_THROW_T_IF(
        constructionChanged,
        InvalidOperationException,
        "Cannot continue replay: rigid-body construction data changed");
  }
}

template <typename LinkLayout>
auto captureReplayLinkLayouts(
    const detail::WorldRegistry& registry,
    common::MemoryAllocator& allocator,
    std::size_t& linkCount)
{
  ReplayScratchVector<LinkLayout> layouts(
      ReplayScratchAllocator<LinkLayout>{allocator});
  const auto view = registry.view<comps::LinkModel>();
  linkCount = countReplayView(view);
  layouts.reserve(linkCount);
  for (const entt::entity entity : view) {
    const auto& link = view.get<comps::LinkModel>(entity);
    LinkLayout layout(allocator);
    layout.entity = entity;
    layout.name.assign(link.name.begin(), link.name.end());
    layout.hasLinkState = registry.all_of<comps::LinkState>(entity);
    layout.hasLinkControl = registry.all_of<comps::LinkControl>(entity);
    layout.hasContactMaterial = registry.all_of<comps::ContactMaterial>(entity);
    layout.hasCollisionGeometry
        = registry.all_of<comps::CollisionGeometry>(entity);
    layout.massProperties = link.mass;
    layout.transformFromParentToJoint = link.transformFromParentToJoint;
    layout.transformFromParentJoint = link.transformFromParentJoint;
    layout.parentJoint = link.parentJoint;
    layout.childJoints.assign(link.childJoints.begin(), link.childJoints.end());
    if (const auto* material
        = registry.try_get<comps::ContactMaterial>(entity)) {
      layout.contactMaterial = *material;
    }
    if (const auto* geometry
        = registry.try_get<comps::CollisionGeometry>(entity)) {
      layout.collisionGeometryRevision = geometry->revision;
      layout.collisionShapes.reserve(geometry->shapes.size());
      for (const CollisionShape& source : geometry->shapes) {
        typename LinkLayout::CollisionShapeLayout target(allocator);
        target.type = source.type;
        target.radius = source.radius;
        target.halfExtents = source.halfExtents;
        target.localTransform = source.localTransform;
        target.normal = source.normal;
        target.offset = source.offset;
        target.vertices.assign(source.vertices.begin(), source.vertices.end());
        target.triangles.assign(
            source.triangles.begin(), source.triangles.end());
        layout.collisionShapes.push_back(std::move(target));
      }
    }
    layouts.push_back(std::move(layout));
  }
  std::ranges::sort(layouts, [](const auto& lhs, const auto& rhs) {
    return static_cast<std::uint32_t>(lhs.entity)
           < static_cast<std::uint32_t>(rhs.entity);
  });
  return layouts;
}

template <typename Layouts>
void validateReplayLinkLayouts(
    const detail::WorldRegistry& registry,
    std::size_t linkCount,
    const Layouts& layouts)
{
  DART_SIMULATION_THROW_T_IF(
      countReplayView(registry.view<comps::LinkModel>()) != linkCount
          || layouts.size() != linkCount,
      InvalidOperationException,
      "Cannot continue replay: link construction count changed");
  for (const auto& layout : layouts) {
    const bool entityValid = registry.valid(layout.entity);
    const bool componentLayoutChanged
        = !entityValid || !registry.all_of<comps::LinkModel>(layout.entity)
          || registry.all_of<comps::LinkState>(layout.entity)
                 != layout.hasLinkState
          || registry.all_of<comps::LinkControl>(layout.entity)
                 != layout.hasLinkControl
          || registry.all_of<comps::ContactMaterial>(layout.entity)
                 != layout.hasContactMaterial
          || registry.all_of<comps::CollisionGeometry>(layout.entity)
                 != layout.hasCollisionGeometry;
    DART_SIMULATION_THROW_T_IF(
        componentLayoutChanged,
        InvalidOperationException,
        "Cannot continue replay: link construction component layout changed");

    const auto& link = registry.get<comps::LinkModel>(layout.entity);
    bool constructionChanged
        = link.name.size() != layout.name.size()
          || !std::ranges::equal(link.name, layout.name)
          || !sameReplayMassProperties(link.mass, layout.massProperties)
          || !link.transformFromParentToJoint.isApprox(
              layout.transformFromParentToJoint, 0.0)
          || !link.transformFromParentJoint.isApprox(
              layout.transformFromParentJoint, 0.0)
          || link.parentJoint != layout.parentJoint
          || !std::ranges::equal(link.childJoints, layout.childJoints);
    if (layout.hasContactMaterial) {
      const auto& material
          = registry.get<comps::ContactMaterial>(layout.entity);
      constructionChanged
          = constructionChanged
            || material.restitution != layout.contactMaterial.restitution
            || material.friction != layout.contactMaterial.friction;
    }
    if (layout.hasCollisionGeometry) {
      const auto& geometry
          = registry.get<comps::CollisionGeometry>(layout.entity);
      constructionChanged
          = constructionChanged
            || geometry.revision != layout.collisionGeometryRevision
            || geometry.shapes.size() != layout.collisionShapes.size();
      if (!constructionChanged) {
        for (std::size_t i = 0; i < geometry.shapes.size(); ++i) {
          if (!sameReplayRigidCollisionShape(
                  geometry.shapes[i], layout.collisionShapes[i])) {
            constructionChanged = true;
            break;
          }
        }
      }
    }
    DART_SIMULATION_THROW_T_IF(
        constructionChanged,
        InvalidOperationException,
        "Cannot continue replay: link construction data changed");
  }
}

bool sameReplayRigidDistanceSpringConfig(
    const detail::deformable_vbd::AvbdRigidWorldDistanceSpringConfig& lhs,
    const detail::deformable_vbd::AvbdRigidWorldDistanceSpringConfig& rhs)
{
  return lhs.enabled == rhs.enabled && lhs.bodyA == rhs.bodyA
         && lhs.bodyB == rhs.bodyB
         && lhs.localAnchorA.isApprox(rhs.localAnchorA, 0.0)
         && lhs.localAnchorB.isApprox(rhs.localAnchorB, 0.0)
         && lhs.restLength == rhs.restLength
         && lhs.startStiffness == rhs.startStiffness
         && lhs.materialStiffness == rhs.materialStiffness
         && lhs.maxStiffness == rhs.maxStiffness;
}

template <typename RigidDistanceSpringLayout>
auto captureReplayRigidDistanceSpringLayouts(
    const detail::WorldRegistry& registry,
    common::MemoryAllocator& allocator,
    std::size_t& springCount)
{
  using Config = detail::deformable_vbd::AvbdRigidWorldDistanceSpringConfig;
  ReplayScratchVector<RigidDistanceSpringLayout> layouts(
      ReplayScratchAllocator<RigidDistanceSpringLayout>{allocator});
  const auto view = registry.view<Config>();
  springCount = countReplayView(view);
  layouts.reserve(springCount);
  for (const entt::entity entity : view) {
    RigidDistanceSpringLayout layout(allocator);
    layout.entity = entity;
    layout.config = view.get<Config>(entity);
    if (const auto* name = registry.try_get<comps::Name>(entity)) {
      layout.name.assign(name->name.begin(), name->name.end());
    }
    layouts.push_back(std::move(layout));
  }
  std::ranges::sort(layouts, [](const auto& lhs, const auto& rhs) {
    return static_cast<std::uint32_t>(lhs.entity)
           < static_cast<std::uint32_t>(rhs.entity);
  });
  return layouts;
}

template <typename Layouts>
void validateReplayRigidDistanceSpringLayouts(
    const detail::WorldRegistry& registry,
    std::size_t springCount,
    const Layouts& layouts)
{
  using Config = detail::deformable_vbd::AvbdRigidWorldDistanceSpringConfig;
  DART_SIMULATION_THROW_T_IF(
      countReplayView(registry.view<Config>()) != springCount
          || layouts.size() != springCount,
      InvalidOperationException,
      "Cannot continue replay: rigid distance-spring construction count "
      "changed");
  for (const auto& layout : layouts) {
    const bool validLayout
        = registry.valid(layout.entity)
          && registry.all_of<Config, comps::Name>(layout.entity);
    DART_SIMULATION_THROW_T_IF(
        !validLayout,
        InvalidOperationException,
        "Cannot continue replay: rigid distance-spring component layout "
        "changed");
    const auto& name = registry.get<comps::Name>(layout.entity).name;
    const std::string_view recordedName{layout.name.data(), layout.name.size()};
    DART_SIMULATION_THROW_T_IF(
        std::string_view{name} != recordedName
            || !sameReplayRigidDistanceSpringConfig(
                registry.get<Config>(layout.entity), layout.config),
        InvalidOperationException,
        "Cannot continue replay: rigid distance-spring construction data "
        "changed");
  }
}

template <typename MultibodyStructureLayout>
auto captureReplayMultibodyStructureLayouts(
    const detail::WorldRegistry& registry,
    common::MemoryAllocator& allocator,
    std::size_t& structureCount)
{
  ReplayScratchVector<MultibodyStructureLayout> layouts(
      ReplayScratchAllocator<MultibodyStructureLayout>{allocator});
  const auto view = registry.view<comps::MultibodyStructure>();
  structureCount = countReplayView(view);
  layouts.reserve(structureCount);
  for (const entt::entity entity : view) {
    const auto& structure = view.get<comps::MultibodyStructure>(entity);
    MultibodyStructureLayout layout(allocator);
    layout.entity = entity;
    layout.hasMultibodyTag = registry.all_of<comps::MultibodyTag>(entity);
    layout.hasVariationalContact
        = registry.all_of<comps::VariationalContact>(entity);
    layout.links.assign(structure.links.begin(), structure.links.end());
    layout.joints.assign(structure.joints.begin(), structure.joints.end());
    if (const auto* contact
        = registry.try_get<comps::VariationalContact>(entity)) {
      layout.contactPlaneNormal = contact->planeNormal;
      layout.contactPlanePoint = contact->planePoint;
      layout.contactStiffness = contact->stiffness;
      layout.contactFrictionCoefficient = contact->frictionCoefficient;
      layout.contactFrictionRegularization = contact->frictionRegularization;
      layout.contactDampingCoefficient = contact->dampingCoefficient;
      layout.contactDualUpdateCadence = contact->dualUpdateCadence;
      layout.contactPointLinkIndices.assign(
          contact->pointLinkIndices.begin(), contact->pointLinkIndices.end());
      layout.contactPointLocalPositions.assign(
          contact->pointLocalPositions.begin(),
          contact->pointLocalPositions.end());
    }
    layouts.push_back(std::move(layout));
  }
  std::ranges::sort(layouts, [](const auto& lhs, const auto& rhs) {
    return static_cast<std::uint32_t>(lhs.entity)
           < static_cast<std::uint32_t>(rhs.entity);
  });
  return layouts;
}

template <typename Layouts>
void validateReplayMultibodyStructureLayouts(
    const detail::WorldRegistry& registry,
    std::size_t structureCount,
    const Layouts& layouts)
{
  DART_SIMULATION_THROW_T_IF(
      countReplayView(registry.view<comps::MultibodyStructure>())
              != structureCount
          || layouts.size() != structureCount,
      InvalidOperationException,
      "Cannot continue replay: multibody construction count changed");
  for (const auto& layout : layouts) {
    const bool validLayout
        = registry.valid(layout.entity)
          && registry.all_of<comps::MultibodyStructure>(layout.entity)
          && registry.all_of<comps::MultibodyTag>(layout.entity)
                 == layout.hasMultibodyTag
          && registry.all_of<comps::VariationalContact>(layout.entity)
                 == layout.hasVariationalContact;
    DART_SIMULATION_THROW_T_IF(
        !validLayout,
        InvalidOperationException,
        "Cannot continue replay: multibody construction component layout "
        "changed");
    const auto& structure
        = registry.get<comps::MultibodyStructure>(layout.entity);
    bool constructionChanged
        = !std::ranges::equal(structure.links, layout.links)
          || !std::ranges::equal(structure.joints, layout.joints);
    if (layout.hasVariationalContact) {
      const auto& contact
          = registry.get<comps::VariationalContact>(layout.entity);
      constructionChanged
          = constructionChanged
            || !contact.planeNormal.isApprox(layout.contactPlaneNormal, 0.0)
            || !contact.planePoint.isApprox(layout.contactPlanePoint, 0.0)
            || contact.stiffness != layout.contactStiffness
            || contact.frictionCoefficient != layout.contactFrictionCoefficient
            || contact.frictionRegularization
                   != layout.contactFrictionRegularization
            || contact.dampingCoefficient != layout.contactDampingCoefficient
            || contact.dualUpdateCadence != layout.contactDualUpdateCadence
            || !std::ranges::equal(
                contact.pointLinkIndices, layout.contactPointLinkIndices)
            || !sameReplayVector3Sequence(
                contact.pointLocalPositions, layout.contactPointLocalPositions);
    }
    DART_SIMULATION_THROW_T_IF(
        constructionChanged,
        InvalidOperationException,
        "Cannot continue replay: multibody construction data changed");
  }
}

bool isPositiveFiniteVector(const Eigen::Vector3d& value)
{
  return value.allFinite() && (value.array() > 0.0).all();
}

bool hasValidMeshTriangleIndices(
    const Eigen::Vector3i& triangle, const std::size_t vertexCount)
{
  if (triangle.minCoeff() < 0) {
    return false;
  }

  const auto maxVertex = static_cast<std::size_t>(triangle.maxCoeff());
  if (maxVertex >= vertexCount) {
    return false;
  }

  return triangle.x() != triangle.y() && triangle.x() != triangle.z()
         && triangle.y() != triangle.z();
}

bool isValidNativeCollisionShape(const CollisionShape& shape)
{
  if (!shape.localTransform.matrix().allFinite()) {
    return false;
  }

  switch (shape.type) {
    case CollisionShapeType::Sphere:
      return std::isfinite(shape.radius) && shape.radius > 0.0;
    case CollisionShapeType::Box:
      return isPositiveFiniteVector(shape.halfExtents);
    case CollisionShapeType::Capsule:
    case CollisionShapeType::Cylinder:
      return std::isfinite(shape.radius) && shape.radius > 0.0
             && std::isfinite(shape.halfExtents.z())
             && shape.halfExtents.z() > 0.0;
    case CollisionShapeType::Plane:
      return shape.normal.allFinite() && shape.normal.squaredNorm() > 0.0
             && std::isfinite(shape.offset);
    case CollisionShapeType::Mesh:
      if (shape.vertices.empty() || shape.triangles.empty()) {
        return false;
      }
      for (const Eigen::Vector3d& vertex : shape.vertices) {
        if (!vertex.allFinite()) {
          return false;
        }
      }
      for (const Eigen::Vector3i& triangle : shape.triangles) {
        if (!hasValidMeshTriangleIndices(triangle, shape.vertices.size())) {
          return false;
        }
      }
      return true;
  }

  return false;
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
      registry,
      registry.get<comps::FrameState>(states[index].entity).parentFrame,
      states,
      publicFrameStates);
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
    captureReplayVectorPayload(
        snapshotState.attachmentTargets, state.attachmentTargets);
    captureReplayVectorPayload(snapshotState.masses, model.masses);
    captureReplayVectorPayload(snapshotState.fixed, model.fixed);
    snapshotState.vbdConfig
        = captureReplayOptionalComponent<comps::DeformableVbdConfig>(
            registry, entity);
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

void validateDeformableAvbdFiniteStiffnessTopology(
    const detail::WorldRegistry& registry,
    entt::entity entity,
    const comps::DeformableVbdConfig& config)
{
  if (!config.enabled || !config.useAvbdFiniteStiffnessRows) {
    return;
  }

  const auto* topology
      = registry.try_get<comps::DeformableMeshTopology>(entity);
  DART_SIMULATION_THROW_T_IF(
      topology != nullptr && !topology->tetrahedra.empty(),
      InvalidArgumentException,
      "AVBD finite-stiffness rows are not defined for tetrahedral materials; "
      "use spring constraints or disable the finite-stiffness row option");
}

template <
    typename PositionVector,
    typename PreviousPositionVector,
    typename VelocityVector,
    typename AttachmentTargetVector,
    typename MassVector,
    typename FixedVector>
void validateRequiredDeformableVbdBodyLayoutData(
    const detail::WorldRegistry& registry,
    entt::entity entity,
    const PositionVector& positions,
    const PreviousPositionVector& previousPositions,
    const VelocityVector& velocities,
    const AttachmentTargetVector& attachmentTargets,
    const MassVector& masses,
    const FixedVector& fixed);

template <typename Snapshot>
bool validateReplayDeformableVbdConfigs(
    const detail::WorldRegistry& registry, const Snapshot& snapshot)
{
  bool changed = false;
  for (const auto& [entity, replayState] : snapshot) {
    const bool validLayout = registry.all_of<
        comps::DeformableBodyTag,
        comps::DeformableNodeState,
        comps::DeformableNodeModel>(entity);
    DART_SIMULATION_THROW_T_IF(
        !validLayout,
        InvalidOperationException,
        "Cannot restore replay frame: DeformableVbdConfig entity layout "
        "changed");
    const auto& currentState = registry.get<comps::DeformableNodeState>(entity);
    const auto& currentModel = registry.get<comps::DeformableNodeModel>(entity);
    DART_SIMULATION_THROW_T_IF(
        currentState.positions.size() != replayState.positions.size()
            || currentState.previousPositions.size()
                   != replayState.previousPositions.size()
            || currentState.velocities.size() != replayState.velocities.size()
            || currentState.attachmentTargets.size()
                   != replayState.attachmentTargets.size()
            || currentModel.masses.size() != replayState.masses.size()
            || currentModel.fixed.size() != replayState.fixed.size(),
        InvalidOperationException,
        "Cannot restore replay frame: DeformableNodeState payload size "
        "changed");
    if (replayState.vbdConfig) {
      comps::validateDeformableVbdConfig(*replayState.vbdConfig);
      validateDeformableAvbdFiniteStiffnessTopology(
          registry, entity, *replayState.vbdConfig);
      if (replayState.vbdConfig->enabled
          && replayState.vbdConfig->requireVbdExecution) {
        validateRequiredDeformableVbdBodyLayoutData(
            registry,
            entity,
            replayState.positions,
            replayState.previousPositions,
            replayState.velocities,
            replayState.attachmentTargets,
            replayState.masses,
            replayState.fixed);
      }
    }
    changed = changed
              || !sameReplayDeformableVbdConfig(
                  replayState.vbdConfig,
                  registry.try_get<comps::DeformableVbdConfig>(entity));
  }
  return changed;
}

template <typename CollisionShapeLike>
bool isValidPublicDeformableVbdObstacleShape(const CollisionShapeLike& shape);
bool isValidPublicDeformableVbdObstacleTransform(
    const comps::Transform& transform);

//==============================================================================
template <typename ReplayFrame, typename RigidBodyLayouts>
void validateReplayRequiredDeformableVbdConfiguration(
    const ReplayFrame& replayFrame, const RigidBodyLayouts& rigidBodyLayouts)
{
  const bool requiresObstacleSupport = std::ranges::any_of(
      replayFrame.deformableNodeStates, [](const auto& entry) {
        const auto& config = entry.second.vbdConfig;
        return config && config->enabled && config->requireVbdExecution
               && config->contactStiffness > 0.0;
      });
  if (!requiresObstacleSupport) {
    return;
  }

  for (const auto& layout : rigidBodyLayouts) {
    if (!layout.hasDeformableGroundBarrier
        && !layout.hasDeformableSurfaceCcdObstacle) {
      continue;
    }

    const auto stateIt = std::ranges::find_if(
        replayFrame.rigidBodies,
        [&](const auto& state) { return state.entity == layout.entity; });
    DART_SIMULATION_THROW_T_IF(
        stateIt == replayFrame.rigidBodies.end(),
        InvalidOperationException,
        "Cannot restore replay frame: public deformable VBD obstacle runtime "
        "state is missing");

    DART_SIMULATION_THROW_T_IF(
        !layout.isStatic || layout.isKinematic,
        InvalidArgumentException,
        "Cannot restore replay frame: public deformable VBD contact requires "
        "every opted-in rigid obstacle to be static");
    DART_SIMULATION_THROW_T_IF(
        layout.parentFrame != entt::null,
        InvalidArgumentException,
        "Cannot restore replay frame: public deformable VBD contact requires "
        "every opted-in rigid obstacle to be attached directly to the world "
        "frame");
    DART_SIMULATION_THROW_T_IF(
        !isValidPublicDeformableVbdObstacleTransform(stateIt->transform),
        InvalidArgumentException,
        "Cannot restore replay frame: public deformable VBD contact requires "
        "every opted-in rigid obstacle to have a finite normalized pose");
    DART_SIMULATION_THROW_T_IF(
        !layout.hasCollisionGeometry || layout.collisionShapes.size() != 1u,
        InvalidArgumentException,
        "Cannot restore replay frame: public deformable VBD contact requires "
        "every opted-in rigid obstacle to have exactly one collision shape");
    DART_SIMULATION_THROW_T_IF(
        !isValidPublicDeformableVbdObstacleShape(
            layout.collisionShapes.front()),
        InvalidArgumentException,
        "Cannot restore replay frame: public deformable VBD contact supports "
        "only one valid static sphere or box obstacle shape");
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
        replayState.attachmentTargets,
        state.attachmentTargets,
        componentName,
        "attachmentTargets");
    restoreReplayVectorPayload(
        replayState.masses, model.masses, componentName, "masses");
    restoreReplayVectorPayload(
        replayState.fixed, model.fixed, componentName, "fixed");
    if (replayState.vbdConfig) {
      registry.emplace_or_replace<comps::DeformableVbdConfig>(
          entity, *replayState.vbdConfig);
    } else {
      registry.remove<comps::DeformableVbdConfig>(entity);
    }
  }
}

template <typename Snapshot>
void restoreReplayDeformableNodeStatesNoAlloc(
    detail::WorldRegistry& registry, Snapshot& snapshot) noexcept
{
  for (auto& [entity, replayState] : snapshot) {
    auto& state = registry.get<comps::DeformableNodeState>(entity);
    auto& model = registry.get<comps::DeformableNodeModel>(entity);
    std::copy(
        replayState.positions.begin(),
        replayState.positions.end(),
        state.positions.begin());
    std::copy(
        replayState.previousPositions.begin(),
        replayState.previousPositions.end(),
        state.previousPositions.begin());
    std::copy(
        replayState.velocities.begin(),
        replayState.velocities.end(),
        state.velocities.begin());
    std::copy(
        replayState.attachmentTargets.begin(),
        replayState.attachmentTargets.end(),
        state.attachmentTargets.begin());
    std::copy(
        replayState.masses.begin(),
        replayState.masses.end(),
        model.masses.begin());
    std::copy(
        replayState.fixed.begin(),
        replayState.fixed.end(),
        model.fixed.begin());
    if (replayState.vbdConfig) {
      if (auto* config = registry.try_get<comps::DeformableVbdConfig>(entity)) {
        *config = *replayState.vbdConfig;
      } else {
        // The component existed when the rollback frame was captured, so its
        // EnTT pool retains enough capacity even if the failed target restore
        // removed it.
        registry.emplace<comps::DeformableVbdConfig>(
            entity, *replayState.vbdConfig);
      }
    } else {
      registry.remove<comps::DeformableVbdConfig>(entity);
    }
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

template <typename Component, typename Snapshot>
void restoreReplayTransientComponentsNoAlloc(
    detail::WorldRegistry& registry, Snapshot& snapshot) noexcept
{
  static_assert(std::is_nothrow_move_constructible_v<Component>);
  static_assert(std::is_nothrow_move_assignable_v<Component>);

  // Removing while iterating an EnTT packed pool invalidates the current
  // iterator. Restart after each removal; replay component sets are small and
  // this failure-only O(n^2) path avoids allocator-backed scratch entirely.
  for (;;) {
    entt::entity staleEntity = entt::null;
    for (auto entity : registry.view<Component>()) {
      const bool existedBeforeRestore = std::ranges::any_of(
          snapshot,
          [entity](const auto& entry) { return entry.first == entity; });
      if (!existedBeforeRestore) {
        staleEntity = entity;
        break;
      }
    }
    if (staleEntity == entt::null) {
      break;
    }
    registry.remove<Component>(staleEntity);
  }

  for (auto& [entity, component] : snapshot) {
    if (auto* live = registry.try_get<Component>(entity)) {
      *live = std::move(component);
    } else {
      // The pool held this component before the transaction and removal never
      // shrinks pool capacity, so emplacing the moved payload cannot allocate.
      registry.emplace<Component>(entity, std::move(component));
    }
  }
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

template <typename ReplayFrame>
void validateReplayCanonicalConstruction(
    const detail::WorldRegistry& registry, const ReplayFrame& canonicalFrame)
{
  DART_SIMULATION_THROW_T_IF(
      countReplayView(registry.view<comps::JointModel>())
          != canonicalFrame.joints.size(),
      InvalidOperationException,
      "Cannot continue replay: Joint component count changed");
  for (const auto& state : canonicalFrame.joints) {
    const bool layoutChanged = !registry.valid(state.entity)
                               || !registry.all_of<
                                   comps::JointModel,
                                   comps::JointState,
                                   comps::JointActuation>(state.entity);
    DART_SIMULATION_THROW_T_IF(
        layoutChanged,
        InvalidOperationException,
        "Cannot continue replay: Joint entity layout changed");
    DART_SIMULATION_THROW_T_IF(
        !sameReplayJointLayout(
            registry.get<comps::JointModel>(state.entity),
            registry.get<comps::JointActuation>(state.entity),
            registry.try_get<comps::AvbdJointStiffness>(state.entity),
            registry.try_get<
                detail::deformable_vbd::AvbdRigidWorldPointJointConfig>(
                state.entity),
            state.layout),
        InvalidOperationException,
        "Cannot continue replay: Joint construction data changed");
    const auto& jointState = registry.get<comps::JointState>(state.entity);
    const auto& jointActuation
        = registry.get<comps::JointActuation>(state.entity);
    DART_SIMULATION_THROW_T_IF(
        static_cast<std::size_t>(jointState.position.size())
                != state.position.values.size()
            || static_cast<std::size_t>(jointState.velocity.size())
                   != state.velocity.values.size()
            || static_cast<std::size_t>(jointState.acceleration.size())
                   != state.acceleration.values.size()
            || static_cast<std::size_t>(jointActuation.torque.size())
                   != state.torque.values.size()
            || static_cast<std::size_t>(jointActuation.commandVelocity.size())
                   != state.commandVelocity.values.size()
            || static_cast<std::size_t>(
                   jointActuation.commandAcceleration.size())
                   != state.commandAcceleration.values.size(),
        InvalidOperationException,
        "Cannot continue replay: Joint runtime vector dimensions changed");
  }

  DART_SIMULATION_THROW_T_IF(
      countReplayView(registry.view<comps::LinkModel>())
          != canonicalFrame.links.size(),
      InvalidOperationException,
      "Cannot continue replay: Link component count changed");
  for (const auto& state : canonicalFrame.links) {
    const bool layoutChanged
        = !registry.valid(state.entity)
          || !registry.all_of<comps::LinkModel, comps::LinkControl>(
              state.entity);
    DART_SIMULATION_THROW_T_IF(
        layoutChanged,
        InvalidOperationException,
        "Cannot continue replay: Link entity layout changed");
  }

  DART_SIMULATION_THROW_T_IF(
      countReplayPublicFrameEntities(registry)
          != canonicalFrame.publicFrames.size(),
      InvalidOperationException,
      "Cannot continue replay: public Frame component count changed");
  for (const auto& state : canonicalFrame.publicFrames) {
    const bool expectedFree = state.freeFrameProperties.has_value();
    const bool expectedFixed = state.fixedFrameProperties.has_value();
    const bool entityValid = registry.valid(state.entity);
    const bool currentFree
        = entityValid
          && registry.all_of<comps::FreeFrameTag, comps::FreeFrameProperties>(
              state.entity);
    const bool currentFixed
        = entityValid
          && registry.all_of<comps::FixedFrameTag, comps::FixedFrameProperties>(
              state.entity);
    const bool layoutChanged
        = !entityValid
          || !registry.all_of<comps::FrameState, comps::FrameCache>(
              state.entity)
          || registry.all_of<comps::RigidBodyTag>(state.entity)
          || registry.all_of<comps::LinkModel>(state.entity)
          || currentFree != expectedFree || currentFixed != expectedFixed
          || currentFree == currentFixed;
    DART_SIMULATION_THROW_T_IF(
        layoutChanged,
        InvalidOperationException,
        "Cannot continue replay: public Frame entity layout changed");
  }

  validateReplayLoopClosures(registry, canonicalFrame.loopClosures);
}

template <typename IgnoredCollisionPairs>
void validateReplayIgnoredCollisionPairs(
    const detail::WorldStorage::IgnoredCollisionPairSet& current,
    const IgnoredCollisionPairs& canonical)
{
  DART_SIMULATION_THROW_T_IF(
      current.size() != canonical.size()
          || !std::ranges::equal(current, canonical),
      InvalidOperationException,
      "Cannot continue replay: ignored collision-pair policy changed");
}

void markFrameCachesDirty(detail::WorldRegistry& registry)
{
  auto frameView = registry.view<comps::FrameCache>();
  for (auto entity : frameView) {
    auto& cache = frameView.get<comps::FrameCache>(entity);
    cache.needTransformUpdate = true;
  }
}

std::size_t countRigidBodyDofs(const detail::BakedWorldModel& model)
{
  return 3 * model.dynamicRigidBodyEntities.size();
}

std::size_t countMultibodyDofs(const detail::BakedWorldModel& model)
{
  std::size_t dofs = 0;
  for (const auto& multibody : model.multibodies) {
    dofs += multibody.dofCount;
  }
  return dofs;
}

#ifdef DART_HAS_DIFF
std::size_t countNonzeroMultibodies(const detail::BakedWorldModel& model)
{
  std::size_t count = 0;
  for (const auto& multibody : model.multibodies) {
    if (multibody.dofCount != 0) {
      ++count;
    }
  }
  return count;
}
#endif

std::size_t countWorldDofs(const detail::BakedWorldModel& model)
{
  return countRigidBodyDofs(model) + countMultibodyDofs(model);
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
  diagnostics.vbdBodyCount = stats.vbdBodyCount;
  diagnostics.vbdSweeps = stats.vbdSweeps;
  diagnostics.vbdVertexUpdates = stats.vbdVertexUpdates;
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
  diagnostics.surfaceContactCandidateCapacityRequested
      = stats.surfaceContactCandidateCapacityRequested;
  diagnostics.surfaceContactCandidateCapacityResolved
      = stats.surfaceContactCandidateCapacityResolved;
  diagnostics.surfaceContactCandidateCountPeak
      = stats.surfaceContactCandidateCountPeak;
  diagnostics.surfaceContactCandidateOverflowCount
      = stats.surfaceContactCandidateOverflowCount;
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
  diagnostics.maxActiveContactCount = stats.maxActiveContactCount;
  return diagnostics;
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
    case RigidBodySolver::Avbd:
    case RigidBodySolver::Vbd:
    case RigidBodySolver::Ipc:
      return true;
  }

  return false;
}

//==============================================================================
bool isRigidBlockDescentSolver(RigidBodySolver solver)
{
  return solver == RigidBodySolver::Vbd || solver == RigidBodySolver::Avbd;
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
void validateRigidSolverContactMethodCompatibility(
    RigidBodySolver solver, ContactSolverMethod contactMethod)
{
  DART_SIMULATION_THROW_T_IF(
      isRigidBlockDescentSolver(solver)
          && contactMethod != ContactSolverMethod::SequentialImpulse,
      InvalidArgumentException,
      "The VBD and AVBD rigid-body solvers own rigid contact resolution and "
      "cannot be combined with a non-default contact solver method");
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

void validateDeformableSolverOptions(const DeformableSolverOptions& options)
{
  DART_SIMULATION_THROW_T_IF(
      options.iterations == 0u,
      InvalidArgumentException,
      "DeformableSolverOptions.iterations must be positive");
  DART_SIMULATION_THROW_T_IF(
      !std::isfinite(options.convergenceTolerance)
          || options.convergenceTolerance < 0.0,
      InvalidArgumentException,
      "DeformableSolverOptions.convergenceTolerance must be finite and "
      "non-negative");
  DART_SIMULATION_THROW_T_IF(
      !std::isfinite(options.accelerationSpectralRadius)
          || options.accelerationSpectralRadius <= 0.0
          || options.accelerationSpectralRadius >= 1.0,
      InvalidArgumentException,
      "DeformableSolverOptions.accelerationSpectralRadius must be finite and "
      "strictly between 0 and 1");
  DART_SIMULATION_THROW_T_IF(
      !std::isfinite(options.stiffnessDamping)
          || options.stiffnessDamping < 0.0,
      InvalidArgumentException,
      "DeformableSolverOptions.stiffnessDamping must be finite and "
      "non-negative");
  DART_SIMULATION_THROW_T_IF(
      !std::isfinite(options.groundContactStiffness)
          || options.groundContactStiffness < 0.0,
      InvalidArgumentException,
      "DeformableSolverOptions.groundContactStiffness must be finite and "
      "non-negative");
}

//==============================================================================
bool requiresPublicDeformableVbdObstacleSupport(const World& world)
{
  const auto& registry = detail::registryOf(world);
  const auto configView = registry.view<comps::DeformableVbdConfig>();
  for (const entt::entity entity : configView) {
    const auto& config = configView.get<comps::DeformableVbdConfig>(entity);
    if (config.enabled && config.requireVbdExecution
        && config.contactStiffness > 0.0) {
      return true;
    }
  }
  return false;
}

//==============================================================================
bool requiresPublicDeformableVbdExecution(const World& world)
{
  const auto& registry = detail::registryOf(world);
  const auto configView = registry.view<comps::DeformableVbdConfig>();
  for (const entt::entity entity : configView) {
    const auto& config = configView.get<comps::DeformableVbdConfig>(entity);
    if (config.enabled && config.requireVbdExecution) {
      return true;
    }
  }
  return false;
}

//==============================================================================
void validateAllDeformableVbdConfigs(const detail::WorldRegistry& registry)
{
  const auto view = registry.view<comps::DeformableVbdConfig>();
  for (const entt::entity entity : view) {
    const auto& config = view.get<comps::DeformableVbdConfig>(entity);
    comps::validateDeformableVbdConfig(config);
    validateDeformableAvbdFiniteStiffnessTopology(registry, entity, config);
  }
}

//==============================================================================
template <typename Key>
using RequiredVbdValidationSet
    = std::set<Key, std::less<Key>, common::StlAllocator<Key>>;

//==============================================================================
template <
    typename PositionVector,
    typename PreviousPositionVector,
    typename VelocityVector,
    typename AttachmentTargetVector,
    typename MassVector,
    typename FixedVector>
void validateRequiredDeformableVbdBodyLayoutData(
    const detail::WorldRegistry& registry,
    entt::entity entity,
    const PositionVector& positions,
    const PreviousPositionVector& previousPositions,
    const VelocityVector& velocities,
    const AttachmentTargetVector& attachmentTargets,
    const MassVector& masses,
    const FixedVector& fixed)
{
  const auto* name = registry.try_get<comps::Name>(entity);
  const std::string_view bodyName
      = name == nullptr ? std::string_view{"<unnamed>"} : name->name;
  const bool hasRequiredComponents = registry.all_of<
      comps::DeformableBodyTag,
      comps::DeformableNodeState,
      comps::DeformableNodeModel,
      comps::DeformableSpringModel,
      comps::DeformableMeshTopology,
      comps::DeformableMaterial>(entity);
  DART_SIMULATION_THROW_T_IF(
      !hasRequiredComponents,
      InvalidArgumentException,
      "Public deformable VBD body '{}' is missing a required execution "
      "component",
      bodyName);

  const auto& springModel = registry.get<comps::DeformableSpringModel>(entity);
  const auto& topology = registry.get<comps::DeformableMeshTopology>(entity);
  const auto& material = registry.get<comps::DeformableMaterial>(entity);
  const std::size_t nodeCount = positions.size();

  DART_SIMULATION_THROW_T_IF(
      nodeCount == 0u || previousPositions.size() != nodeCount
          || velocities.size() != nodeCount
          || attachmentTargets.size() != nodeCount || masses.size() != nodeCount
          || fixed.size() != nodeCount
          || topology.restPositions.size() != nodeCount,
      InvalidArgumentException,
      "Public deformable VBD body '{}' has inconsistent per-node vector "
      "sizes",
      bodyName);
  for (std::size_t i = 0; i < nodeCount; ++i) {
    DART_SIMULATION_THROW_T_IF(
        !positions[i].allFinite() || !previousPositions[i].allFinite()
            || !velocities[i].allFinite() || !attachmentTargets[i].allFinite()
            || !topology.restPositions[i].allFinite()
            || !std::isfinite(masses[i]) || masses[i] <= 0.0 || fixed[i] > 1u,
        InvalidArgumentException,
        "Public deformable VBD body '{}' has invalid node data at index {}",
        bodyName,
        i);
  }

  DART_SIMULATION_THROW_T_IF(
      !std::isfinite(springModel.stiffness) || springModel.stiffness < 0.0
          || !std::isfinite(springModel.damping) || springModel.damping < 0.0,
      InvalidArgumentException,
      "Public deformable VBD body '{}' has invalid spring material values",
      bodyName);
  for (std::size_t i = 0; i < springModel.edges.size(); ++i) {
    const auto& edge = springModel.edges[i];
    DART_SIMULATION_THROW_T_IF(
        edge.nodeA >= nodeCount || edge.nodeB >= nodeCount
            || edge.nodeA == edge.nodeB || !std::isfinite(edge.restLength)
            || edge.restLength <= 0.0,
        InvalidArgumentException,
        "Public deformable VBD body '{}' has an invalid spring edge at "
        "index {}",
        bodyName,
        i);
  }

  RequiredVbdValidationSet<std::array<std::size_t, 3>> uniqueSurfaceTriangles{
      std::less<std::array<std::size_t, 3>>{},
      common::StlAllocator<std::array<std::size_t, 3>>{
          registry.get_allocator()}};
  for (std::size_t i = 0; i < topology.surfaceTriangles.size(); ++i) {
    const auto& triangle = topology.surfaceTriangles[i];
    std::array<std::size_t, 3> nodes{
        triangle.nodeA, triangle.nodeB, triangle.nodeC};
    const bool hasOutOfRangeNode = std::ranges::any_of(
        nodes, [nodeCount](std::size_t node) { return node >= nodeCount; });
    const bool hasRepeatedNode
        = nodes[0] == nodes[1] || nodes[0] == nodes[2] || nodes[1] == nodes[2];
    double areaSquared = 0.0;
    if (!hasOutOfRangeNode && !hasRepeatedNode) {
      areaSquared = 0.25
                    * (topology.restPositions[nodes[1]]
                       - topology.restPositions[nodes[0]])
                          .cross(
                              topology.restPositions[nodes[2]]
                              - topology.restPositions[nodes[0]])
                          .squaredNorm();
    }
    std::ranges::sort(nodes);
    DART_SIMULATION_THROW_T_IF(
        hasOutOfRangeNode || hasRepeatedNode || !std::isfinite(areaSquared)
            || areaSquared <= 1e-24
            || !uniqueSurfaceTriangles.insert(nodes).second,
        InvalidArgumentException,
        "Public deformable VBD body '{}' has an invalid, degenerate, or "
        "duplicate surface triangle at index {}",
        bodyName,
        i);
  }

  DART_SIMULATION_THROW_T_IF(
      topology.tetrahedronRestVolumes.size() != topology.tetrahedra.size(),
      InvalidArgumentException,
      "Public deformable VBD body '{}' has inconsistent tetrahedron and "
      "rest-volume counts",
      bodyName);
  RequiredVbdValidationSet<std::array<std::size_t, 4>> uniqueTetrahedra{
      std::less<std::array<std::size_t, 4>>{},
      common::StlAllocator<std::array<std::size_t, 4>>{
          registry.get_allocator()}};
  for (std::size_t i = 0; i < topology.tetrahedra.size(); ++i) {
    const auto& tetrahedron = topology.tetrahedra[i];
    std::array<std::size_t, 4> nodes{
        tetrahedron.nodeA,
        tetrahedron.nodeB,
        tetrahedron.nodeC,
        tetrahedron.nodeD};
    const bool hasOutOfRangeNode = std::ranges::any_of(
        nodes, [nodeCount](std::size_t node) { return node >= nodeCount; });
    const bool hasRepeatedNode = nodes[0] == nodes[1] || nodes[0] == nodes[2]
                                 || nodes[0] == nodes[3] || nodes[1] == nodes[2]
                                 || nodes[1] == nodes[3]
                                 || nodes[2] == nodes[3];
    double computedVolume = 0.0;
    if (!hasOutOfRangeNode && !hasRepeatedNode) {
      const auto& a = topology.restPositions[tetrahedron.nodeA];
      const auto& b = topology.restPositions[tetrahedron.nodeB];
      const auto& c = topology.restPositions[tetrahedron.nodeC];
      const auto& d = topology.restPositions[tetrahedron.nodeD];
      computedVolume = (b - a).cross(c - a).dot(d - a) / 6.0;
    }
    const double storedVolume = topology.tetrahedronRestVolumes[i];
    const double volumeScale
        = std::max({1e-18, std::abs(computedVolume), std::abs(storedVolume)});
    std::ranges::sort(nodes);
    DART_SIMULATION_THROW_T_IF(
        hasOutOfRangeNode || hasRepeatedNode || !std::isfinite(computedVolume)
            || computedVolume <= 1e-18 || !std::isfinite(storedVolume)
            || storedVolume <= 1e-18
            || std::abs(storedVolume - computedVolume) > 1e-12 * volumeScale
            || !uniqueTetrahedra.insert(nodes).second,
        InvalidArgumentException,
        "Public deformable VBD body '{}' has invalid, degenerate, duplicate, "
        "or inconsistent tetrahedron data at index {}",
        bodyName,
        i);
  }

  const double shearModulus
      = material.youngsModulus / (2.0 * (1.0 + material.poissonRatio));
  const double lameFirstParameter
      = material.youngsModulus * material.poissonRatio
        / ((1.0 + material.poissonRatio) * (1.0 - 2.0 * material.poissonRatio));
  DART_SIMULATION_THROW_T_IF(
      !std::isfinite(material.density) || material.density <= 0.0
          || !std::isfinite(material.youngsModulus)
          || material.youngsModulus <= 0.0
          || !std::isfinite(material.poissonRatio)
          || material.poissonRatio <= -1.0 || material.poissonRatio >= 0.5
          || !std::isfinite(material.frictionCoefficient)
          || material.frictionCoefficient < 0.0
          || (material.useFiniteElementElasticity
              && material.poissonRatio < 0.0)
          || (!topology.tetrahedra.empty()
              && (!std::isfinite(shearModulus) || shearModulus <= 0.0
                  || !std::isfinite(lameFirstParameter))),
      InvalidArgumentException,
      "Public deformable VBD body '{}' has invalid material values, an "
      "auxetic finite-element material, or nonfinite derived Lame parameters",
      bodyName);

  const auto* boundaries
      = registry.try_get<comps::DeformableBoundaryConditions>(entity);
  if (boundaries == nullptr) {
    return;
  }
  const auto rangesOverlap
      = [](double startA, double endA, double startB, double endB) {
          return startA < endB && startB < endA;
        };
  const auto shareNode = [](const auto& lhs, const auto& rhs) {
    return std::ranges::any_of(lhs, [&rhs](std::size_t node) {
      return std::ranges::find(rhs, node) != rhs.end();
    });
  };

  for (std::size_t i = 0; i < boundaries->dirichlet.size(); ++i) {
    const auto& boundary = boundaries->dirichlet[i];
    RequiredVbdValidationSet<std::size_t> uniqueNodes{
        std::less<std::size_t>{},
        common::StlAllocator<std::size_t>{registry.get_allocator()}};
    bool invalidNode = false;
    bool conflictsWithFixed = false;
    bool invalidReference = false;
    for (std::size_t j = 0; j < boundary.nodes.size(); ++j) {
      const std::size_t node = boundary.nodes[j];
      invalidNode = invalidNode || node >= nodeCount
                    || !uniqueNodes.insert(node).second;
      if (node < nodeCount) {
        conflictsWithFixed = conflictsWithFixed || fixed[node] != 0u;
        if (j < boundary.referencePositions.size()) {
          invalidReference = invalidReference
                             || !boundary.referencePositions[j].allFinite()
                             || !boundary.referencePositions[j].isApprox(
                                 topology.restPositions[node], 0.0);
        }
      }
    }
    const bool validEnd
        = std::isfinite(boundary.endTime)
          || boundary.endTime == std::numeric_limits<double>::infinity();
    DART_SIMULATION_THROW_T_IF(
        boundary.nodes.empty()
            || boundary.nodes.size() != boundary.referencePositions.size()
            || invalidNode || conflictsWithFixed || invalidReference
            || !boundary.center.allFinite()
            || !boundary.linearVelocity.allFinite()
            || !boundary.angularVelocity.allFinite()
            || !std::isfinite(boundary.startTime) || boundary.startTime < 0.0
            || !validEnd || boundary.endTime < boundary.startTime,
        InvalidArgumentException,
        "Public deformable VBD body '{}' has an invalid Dirichlet boundary "
        "at index {}",
        bodyName,
        i);
    for (std::size_t j = i + 1; j < boundaries->dirichlet.size(); ++j) {
      const auto& other = boundaries->dirichlet[j];
      DART_SIMULATION_THROW_T_IF(
          rangesOverlap(
              boundary.startTime,
              boundary.endTime,
              other.startTime,
              other.endTime)
              && shareNode(boundary.nodes, other.nodes),
          InvalidArgumentException,
          "Public deformable VBD body '{}' has overlapping Dirichlet "
          "boundaries at indices {} and {}",
          bodyName,
          i,
          j);
    }
  }

  for (std::size_t i = 0; i < boundaries->neumann.size(); ++i) {
    const auto& boundary = boundaries->neumann[i];
    RequiredVbdValidationSet<std::size_t> uniqueNodes{
        std::less<std::size_t>{},
        common::StlAllocator<std::size_t>{registry.get_allocator()}};
    bool invalidNode = false;
    bool conflictsWithFixed = false;
    for (const std::size_t node : boundary.nodes) {
      invalidNode = invalidNode || node >= nodeCount
                    || !uniqueNodes.insert(node).second;
      if (node < nodeCount) {
        conflictsWithFixed = conflictsWithFixed || fixed[node] != 0u;
      }
    }
    const bool validEnd
        = std::isfinite(boundary.endTime)
          || boundary.endTime == std::numeric_limits<double>::infinity();
    DART_SIMULATION_THROW_T_IF(
        boundary.nodes.empty() || invalidNode || conflictsWithFixed
            || !boundary.acceleration.allFinite()
            || !std::isfinite(boundary.startTime) || boundary.startTime < 0.0
            || !validEnd || boundary.endTime < boundary.startTime,
        InvalidArgumentException,
        "Public deformable VBD body '{}' has an invalid Neumann boundary at "
        "index {}",
        bodyName,
        i);
    for (std::size_t j = 0; j < boundaries->dirichlet.size(); ++j) {
      const auto& dirichlet = boundaries->dirichlet[j];
      DART_SIMULATION_THROW_T_IF(
          rangesOverlap(
              boundary.startTime,
              boundary.endTime,
              dirichlet.startTime,
              dirichlet.endTime)
              && shareNode(boundary.nodes, dirichlet.nodes),
          InvalidArgumentException,
          "Public deformable VBD body '{}' has overlapping Neumann {} and "
          "Dirichlet {} boundaries",
          bodyName,
          i,
          j);
    }
  }
}

//==============================================================================
void validateRequiredDeformableVbdBodyLayouts(
    const detail::WorldRegistry& registry)
{
  const auto configView = registry.view<comps::DeformableVbdConfig>();
  for (const entt::entity entity : configView) {
    const auto& config = configView.get<comps::DeformableVbdConfig>(entity);
    if (!config.enabled || !config.requireVbdExecution) {
      continue;
    }

    const bool hasNodeData
        = registry
              .all_of<comps::DeformableNodeState, comps::DeformableNodeModel>(
                  entity);
    if (!hasNodeData) {
      const auto* name = registry.try_get<comps::Name>(entity);
      const std::string_view bodyName
          = name == nullptr ? std::string_view{"<unnamed>"} : name->name;
      DART_SIMULATION_THROW_T(
          InvalidArgumentException,
          "Public deformable VBD body '{}' is missing a required execution "
          "component",
          bodyName);
    }
    const auto& state = registry.get<comps::DeformableNodeState>(entity);
    const auto& nodeModel = registry.get<comps::DeformableNodeModel>(entity);
    validateRequiredDeformableVbdBodyLayoutData(
        registry,
        entity,
        state.positions,
        state.previousPositions,
        state.velocities,
        state.attachmentTargets,
        nodeModel.masses,
        nodeModel.fixed);
  }
}

//==============================================================================
template <typename CollisionShapeLike>
bool isValidPublicDeformableVbdObstacleShape(const CollisionShapeLike& shape)
{
  const Eigen::Matrix3d rotation = shape.localTransform.linear();
  if (!shape.localTransform.matrix().allFinite()
      || !(rotation.transpose() * rotation)
              .isApprox(Eigen::Matrix3d::Identity(), 1e-9)
      || std::abs(rotation.determinant() - 1.0) > 1e-9) {
    return false;
  }

  switch (shape.type) {
    case CollisionShapeType::Sphere:
      return std::isfinite(shape.radius) && shape.radius > 0.0;
    case CollisionShapeType::Box:
      return shape.halfExtents.allFinite()
             && (shape.halfExtents.array() > 0.0).all();
    case CollisionShapeType::Mesh:
    case CollisionShapeType::Capsule:
    case CollisionShapeType::Cylinder:
    case CollisionShapeType::Plane:
      return false;
  }

  return false; // LCOV_EXCL_LINE
}

//==============================================================================
bool isValidPublicDeformableVbdObstacleTransform(
    const comps::Transform& transform)
{
  const double orientationNorm = transform.orientation.norm();
  return transform.position.allFinite()
         && transform.orientation.coeffs().allFinite()
         && std::isfinite(orientationNorm)
         && std::abs(orientationNorm - 1.0) <= 1e-9;
}

//==============================================================================
void validateRequiredDeformableVbdConfiguration(const World& world)
{
  const auto& registry = detail::registryOf(world);
  validateAllDeformableVbdConfigs(registry);
  validateRequiredDeformableVbdBodyLayouts(registry);
  if (!requiresPublicDeformableVbdObstacleSupport(world)) {
    return;
  }

  const auto validateObstacle = [&registry](entt::entity entity) {
    const auto* name = registry.try_get<comps::Name>(entity);
    const std::string_view bodyName
        = name == nullptr ? std::string_view{"<unnamed>"} : name->name;
    DART_SIMULATION_THROW_T_IF(
        !registry.all_of<comps::RigidBodyTag>(entity),
        InvalidArgumentException,
        "Public deformable VBD obstacle '{}' is missing RigidBodyTag",
        bodyName);
    DART_SIMULATION_THROW_T_IF(
        !registry.all_of<comps::StaticBodyTag>(entity)
            || registry.all_of<comps::KinematicBodyTag>(entity),
        InvalidArgumentException,
        "Public deformable VBD contact requires every opted-in rigid "
        "obstacle to be static; obstacle '{}' is dynamic or kinematic",
        bodyName);
    const auto* frameState = registry.try_get<comps::FrameState>(entity);
    DART_SIMULATION_THROW_T_IF(
        frameState == nullptr || frameState->parentFrame != entt::null,
        InvalidArgumentException,
        "Public deformable VBD contact requires obstacle '{}' to be attached "
        "directly to the world frame",
        bodyName);
    const auto* transform = registry.try_get<comps::Transform>(entity);
    DART_SIMULATION_THROW_T_IF(
        transform == nullptr
            || !isValidPublicDeformableVbdObstacleTransform(*transform),
        InvalidArgumentException,
        "Public deformable VBD contact requires obstacle '{}' to have a "
        "finite normalized pose",
        bodyName);

    const auto* geometry = registry.try_get<comps::CollisionGeometry>(entity);
    DART_SIMULATION_THROW_T_IF(
        geometry == nullptr || geometry->shapes.size() != 1u,
        InvalidArgumentException,
        "Public deformable VBD contact requires obstacle '{}' to have "
        "exactly one collision shape",
        bodyName);
    DART_SIMULATION_THROW_T_IF(
        !isValidPublicDeformableVbdObstacleShape(geometry->shapes.front()),
        InvalidArgumentException,
        "Public deformable VBD contact supports only one valid static sphere "
        "or box shape; obstacle '{}' uses an unsupported or invalid shape",
        bodyName);
  };

  const auto groundObstacles
      = registry.view<comps::DeformableGroundBarrierTag>();
  for (const entt::entity entity : groundObstacles) {
    validateObstacle(entity);
  }
  const auto surfaceObstacles
      = registry.view<comps::DeformableSurfaceCcdObstacleTag>();
  for (const entt::entity entity : surfaceObstacles) {
    if (!registry.all_of<comps::DeformableGroundBarrierTag>(entity)) {
      validateObstacle(entity);
    }
  }
}

//==============================================================================
std::uint8_t encodeRigidBodySolver(RigidBodySolver solver)
{
  switch (solver) {
    case RigidBodySolver::SequentialImpulse:
      return 0u;
    case RigidBodySolver::Avbd:
      return 2u;
    case RigidBodySolver::Vbd:
      return 3u;
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
    case 2u:
      return RigidBodySolver::Avbd;
    case 3u:
      return RigidBodySolver::Vbd;
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
std::uint8_t encodeDifferentiablePhysicalParameter(PhysicalParameter parameter)
{
  switch (parameter) {
    case PhysicalParameter::MASS:
      return 0u;
    case PhysicalParameter::INERTIA:
      return 1u;
    case PhysicalParameter::FRICTION:
      return 2u;
    case PhysicalParameter::CENTER_OF_MASS:
      DART_SIMULATION_THROW_T(
          NotImplementedException,
          "PhysicalParameter::CENTER_OF_MASS is not a supported "
          "differentiable parameter");
  }

  DART_SIMULATION_THROW_T( // LCOV_EXCL_LINE
      InvalidArgumentException, "Differentiable physical parameter is invalid");
  return 0u;
}

//==============================================================================
PhysicalParameter decodeDifferentiablePhysicalParameter(std::uint8_t value)
{
  switch (value) {
    case 0u:
      return PhysicalParameter::MASS;
    case 1u:
      return PhysicalParameter::INERTIA;
    case 2u:
      return PhysicalParameter::FRICTION;
  }

  DART_SIMULATION_THROW_T(
      InvalidArgumentException,
      "Serialized World differentiable physical-parameter value is invalid");
  return PhysicalParameter::MASS;
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
    case RigidBodySolver::Avbd:
      return detail::BuiltInRigidBodySolverFamily::Avbd;
    case RigidBodySolver::Vbd:
      return detail::BuiltInRigidBodySolverFamily::Vbd;
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
template <typename Registry>
bool isWorldOwnedJoint(
    const Registry& registry,
    entt::entity jointEntity,
    const comps::JointModel& joint)
{
  return isRigidBodyJoint(registry, joint)
         || isArticulatedPointJoint(registry, jointEntity, joint);
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
bool hasFiniteStiffnessRigidBodyJointRows(const World& world)
{
  const auto& registry = detail::registryOf(world);
  const auto view = registry.view<comps::JointModel>();
  for (const entt::entity entity : view) {
    const auto& joint = view.get<comps::JointModel>(entity);
    if (!isRigidBodyJoint(registry, joint)) {
      continue;
    }

    const auto* config
        = registry
              .try_get<detail::deformable_vbd::AvbdRigidWorldPointJointConfig>(
                  entity);
    if (config == nullptr || !config->enabled) {
      continue;
    }

    const bool hasFiniteLinearRows
        = config->linearAxisMask != 0u
          && !std::isinf(config->linearMaterialStiffness);
    const bool hasFiniteAngularRows
        = config->angularAxisMask != 0u
          && !std::isinf(config->angularMaterialStiffness);
    if (hasFiniteLinearRows || hasFiniteAngularRows) {
      return true;
    }
  }
  return false;
}

//==============================================================================
bool hasHardStiffnessRigidBodyJointRows(const World& world)
{
  const auto& registry = detail::registryOf(world);
  const auto view = registry.view<comps::JointModel>();
  for (const entt::entity entity : view) {
    const auto& joint = view.get<comps::JointModel>(entity);
    if (!isRigidBodyJoint(registry, joint)) {
      continue;
    }

    const auto* config
        = registry
              .try_get<detail::deformable_vbd::AvbdRigidWorldPointJointConfig>(
                  entity);
    if (config == nullptr || !config->enabled) {
      continue;
    }

    const bool hasHardLinearRows
        = config->linearAxisMask != 0u
          && std::isinf(config->linearMaterialStiffness);
    const bool hasHardAngularRows
        = config->angularAxisMask != 0u
          && std::isinf(config->angularMaterialStiffness);
    if (hasHardLinearRows || hasHardAngularRows) {
      return true;
    }
  }
  return false;
}

//==============================================================================
void validateRigidBodyJointPipelineSupport(
    const World& world, RigidBodySolver solver)
{
  DART_SIMULATION_THROW_T_IF(
      isRigidBlockDescentSolver(solver) && hasMultibodyStructures(world),
      InvalidOperationException,
      "The VBD and AVBD rigid-body solvers currently support free rigid-body "
      "worlds only; multibody structures require a different rigid-body "
      "solver");

  if (!hasRigidBodyAvbdPairConstraints(world)) {
    return;
  }

  DART_SIMULATION_THROW_T_IF(
      solver == RigidBodySolver::SequentialImpulse
          && hasFiniteStiffnessRigidBodyJointRows(world),
      InvalidOperationException,
      "The Sequential Impulse rigid-body solver supports hard rigid-body "
      "joint rows only; finite constraint projection stiffness requires the "
      "VBD or AVBD rigid-body solver");

  DART_SIMULATION_THROW_T_IF(
      solver == RigidBodySolver::Vbd
          && hasHardStiffnessRigidBodyJointRows(world),
      InvalidOperationException,
      "The VBD rigid-body solver supports finite-penalty rigid-body joint "
      "rows only; configure finite linear and angular constraint projection "
      "stiffness or select Sequential Impulse or AVBD for hard rows");

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
      "Public rigid-body pair constraints are not supported in worlds with "
      "multibody structures under Sequential Impulse, VBD, or AVBD");
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
  explicit WorldStepPipelineStages(
      common::MemoryManager& memoryManager,
      std::size_t rigidConstraintIterations)
    : rigidBodyVelocity(&memoryManager),
      rigidBodyContact(rigidConstraintIterations, &memoryManager),
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
      material.useFiniteElementElasticity && material.poissonRatio < 0.0,
      InvalidArgumentException,
      "DeformableBodyOptions.material finite-element elasticity requires "
      "poissonRatio in [0, 0.5)");
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
  comps::DeformableContactConfig contactConfig;
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
std::size_t validateRigidConstraintIterations(std::size_t iterations)
{
  DART_SIMULATION_THROW_T_IF(
      iterations == 0,
      InvalidArgumentException,
      "World rigid constraint iteration budget must be positive");
  DART_SIMULATION_THROW_T_IF(
      iterations > static_cast<std::size_t>(std::numeric_limits<int>::max()),
      InvalidArgumentException,
      "World rigid constraint iteration budget is too large");
  return iterations;
}

//==============================================================================
RigidConstraintOptions validateRigidConstraintOptions(
    const RigidConstraintOptions& options, RigidBodySolver solver)
{
  RigidConstraintOptions validated = options;
  validated.iterations = validateRigidConstraintIterations(options.iterations);
  DART_SIMULATION_THROW_T_IF(
      solver == RigidBodySolver::Ipc
          && validated.iterations != RigidConstraintOptions{}.iterations,
      InvalidArgumentException,
      "RigidConstraintOptions are not applicable to the IPC rigid-body "
      "family; use the default options");
  return validated;
}

//==============================================================================
bool usesUnifiedConstraintStage(
    const World& world, RigidBodySolver solver, bool variationalSelected)
{
  return !variationalSelected
         && detail::builtInRigidSolverUsesSplitPipeline(
             toBuiltInRigidBodySolverFamily(solver))
         && hasMultibodyStructures(world);
}

//==============================================================================
void validateRigidConstraintOptionsPipelineSupport(
    const World& world,
    const RigidConstraintOptions& options,
    RigidBodySolver solver,
    bool variationalSelected)
{
  DART_SIMULATION_THROW_T_IF(
      usesUnifiedConstraintStage(world, solver, variationalSelected)
          && options.iterations != RigidConstraintOptions{}.iterations,
      InvalidOperationException,
      "Non-default RigidConstraintOptions are not applicable when "
      "semi-implicit multibody structures select the unified constraint "
      "stage; use the default options");
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
  data.contactConfig.surfaceCandidateCapacity
      = options.surfaceContactCandidateCapacity;

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
    rebindLoadedVectorAllocator(state.positions, allocator);
    rebindLoadedVectorAllocator(state.previousPositions, allocator);
    rebindLoadedVectorAllocator(state.velocities, allocator);
    rebindLoadedVectorAllocator(state.attachmentTargets, allocator);
    if (auto* nodeModel
        = registry.try_get<comps::DeformableNodeModel>(entity)) {
      rebindLoadedVectorAllocator(nodeModel->masses, allocator);
      rebindLoadedVectorAllocator(nodeModel->fixed, allocator);
    }
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
  explicit StepPipelineCache(
      common::MemoryManager& memoryManager,
      std::size_t rigidConstraintIterations)
    : stages(memoryManager, rigidConstraintIterations)
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
    common::MemoryManager& memoryManager, std::size_t rigidConstraintIterations)
{
  auto* cache = memoryManager.constructUsingFree<StepPipelineCache>(
      memoryManager, rigidConstraintIterations);
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
    m_stepPipelineCache(makeStepPipelineCache(
        m_memoryManager, m_rigidConstraintOptions.iterations)),
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
    m_rigidConstraintOptions(options.rigidConstraintOptions),
    m_rigidCollisionCapacityOptions(options.rigidCollisionCapacityOptions),
    m_timeStep(options.timeStep),
    m_differentiable(options.differentiable),
    m_contactSolverMethod(options.contactSolverMethod),
    m_contactGradientMode(options.contactGradientMode),
    m_computeAcceleratorPolicy(options.computeAcceleratorPolicy),
    m_strictSolverResolution(options.strictSolverResolution),
    m_deactivationOptions(options.deactivationOptions),
    m_collisionQueryCache(
        nullptr, CollisionQueryCacheDeleter{&m_memoryManager}),
    m_stepPipelineCache(makeStepPipelineCache(
        m_memoryManager, m_rigidConstraintOptions.iterations)),
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
      options.rigidCollisionCapacityOptions.contactCapacity
          > std::numeric_limits<std::size_t>::max() / 2u,
      InvalidArgumentException,
      "WorldOptions.rigidCollisionCapacityOptions.contactCapacity is too "
      "large to represent the two rigid-contact tangent rows required per "
      "contact");
  m_rigidConstraintOptions = validateRigidConstraintOptions(
      options.rigidConstraintOptions, options.rigidBodySolver);
  DART_SIMULATION_THROW_T_IF(
      !isValidContactSolverMethod(options.contactSolverMethod),
      InvalidArgumentException,
      "WorldOptions.contactSolverMethod is invalid");
  validateRigidSolverContactMethodCompatibility(
      options.rigidBodySolver, options.contactSolverMethod);
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
  return getMemoryDiagnostics(WorldMemoryDiagnosticsOptions{});
}

//==============================================================================
WorldMemoryDiagnostics World::getMemoryDiagnostics(
    const WorldMemoryDiagnosticsOptions& options) const
{
#if DART_BUILD_MEMORY_DIAGNOSTICS
  return m_storage->memoryDiagnostics.collect(
      m_memoryManager, detail::registryOf(*this), options);
#else
  // No step-path instrumentation is compiled in, so the frame-scratch peak and
  // reset tally are unavailable and report zero. Everything else is still read
  // live from the allocator and the registry at query time.
  return detail::MemoryDiagnosticsTracker{}.collect(
      m_memoryManager, detail::registryOf(*this), options);
#endif
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
  m_rigidConstraintOptions = {};
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
  m_resolvedConfiguration.reset();
  m_time = 0.0;
  m_frame = 0;
  m_memoryManager.getFrameAllocator().reset();
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
  m_callerOwnedPipelineContinuationStateMayBeLive = false;
  m_stepPipelineCache = makeStepPipelineCache(
      m_memoryManager, m_rigidConstraintOptions.iterations);
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
  m_resolvedConfiguration.reset();
  if (m_storage != nullptr) {
    m_storage->bakedModel.valid = false;
  }
}

//==============================================================================
void World::markModelChanged() noexcept
{
  m_resolvedConfiguration.reset();
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
  try {
    validateRequiredDeformableVbdConfiguration(*this);
    validateRigidConstraintOptionsPipelineSupport(
        *this,
        m_rigidConstraintOptions,
        m_rigidBodySolver,
        m_multibodyIntegrationMethod
            == MultibodyIntegrationMethod::Variational);
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
    prepareDifferentiableContactFreeScratchForCurrentConfiguration();
    resolveComputeAcceleratorForCurrentConfiguration();
    if (m_simulationMode && m_collisionQueryCache) {
      m_collisionQueryCache->capacitiesLocked = true;
    }
    recordResolvedConfiguration();
  } catch (...) {
    // A partially prepared cache is never current. In particular, strict
    // resolution can reject after the dense model and stage scratch were
    // prepared; leaving the bake valid would let the next step skip the
    // rejected policy boundary.
    m_storage->bakedModel.valid = false;
    m_resolvedConfiguration.reset();
    throw;
  }
}

//==============================================================================
void World::applyPreparedConfigurationTransactionally(
    std::function<void()> apply, std::function<void()> rollback)
{
  // Complete every fallible snapshot before changing the public selection.
  // The old stage cache remains the authoritative continuation until the new
  // configuration has prepared and accepted an exact warm-state copy.
  PersistentPreparationRegistrySnapshot persistentRegistrySnapshot(*this);
  auto& allocator = m_memoryManager.getFreeAllocator();
  auto rigidAvbdWarmStart = m_stepPipelineCache->stages.rigidBodyContact
                                .captureAvbdWarmStartReplayState(allocator);
  detail::BakedWorldModel previousBakedModel(allocator);
  compute::ResolvedSolverConfiguration previousResolvedConfiguration;
  std::optional<StepDerivatives> previousStepDerivatives;
  const bool previousStepDerivativesValid = m_storage->stepDerivativesValid;
  const DeformablePsdProjector previousDeformablePsdProjector
      = m_deformablePsdProjector;
  const bool previousDeformablePsdAcceleratedResolved
      = m_deformablePsdAcceleratedResolved;

  apply();
  StepPipelineCachePtr replacementStepPipelineCache(
      nullptr, StepPipelineCacheDeleter{&m_memoryManager});
  try {
    replacementStepPipelineCache = makeStepPipelineCache(
        m_memoryManager, m_rigidConstraintOptions.iterations);
  } catch (...) {
    rollback();
    throw;
  }

  auto previousStepPipelineCache = std::move(m_stepPipelineCache);
  m_stepPipelineCache = std::move(replacementStepPipelineCache);
  auto previousCollisionQueryCache = std::move(m_collisionQueryCache);
  std::swap(previousBakedModel, m_storage->bakedModel);
  previousResolvedConfiguration.notes.swap(m_resolvedConfiguration.notes);
  previousStepDerivatives.swap(m_storage->stepDerivatives);
  m_storage->stepDerivativesValid = false;

  try {
    prepareStepPipelineCacheForCurrentConfiguration();
    m_stepPipelineCache->stages.rigidBodyContact
        .restoreAvbdWarmStartReplayState(rigidAvbdWarmStart);
  } catch (...) {
    const std::exception_ptr configurationFailure = std::current_exception();
    rollback();
    persistentRegistrySnapshot.restore();
    std::swap(previousBakedModel, m_storage->bakedModel);
    previousResolvedConfiguration.notes.swap(m_resolvedConfiguration.notes);
    previousStepDerivatives.swap(m_storage->stepDerivatives);
    m_storage->stepDerivativesValid = previousStepDerivativesValid;
    m_deformablePsdProjector = previousDeformablePsdProjector;
    m_deformablePsdAcceleratedResolved
        = previousDeformablePsdAcceleratedResolved;
    m_collisionQueryCache.reset();
    m_collisionQueryCache = std::move(previousCollisionQueryCache);
    m_stepPipelineCache.reset();
    m_stepPipelineCache = std::move(previousStepPipelineCache);
    std::rethrow_exception(configurationFailure);
  }
}

//==============================================================================
void World::ensureStepPipelineCacheCurrent()
{
  if (!m_simulationMode) {
    enterSimulationMode();
    return;
  }

  validateReplayConstruction();

  if (!m_storage->bakedModel.valid) {
    prepareStepPipelineCacheForCurrentConfiguration();
  }
}

//==============================================================================
void World::prepareDifferentiableContactFreeScratchForCurrentConfiguration()
{
#ifdef DART_HAS_DIFF
  if (!m_differentiable) {
    return;
  }

  const bool prepared = captureContactFreeStepDerivativesForFirstMultibody();
  if (prepared) {
    m_storage->stepDerivativesValid = false;
  }
#endif
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
compute::ResolvedSolverConfiguration World::buildResolvedConfiguration(
    bool deformablePsdAcceleratedResolved) const
{
  // PLAN-091 WP-091.11: snapshot the resolved per-domain method families
  // (requested -> resolved, with reasons). Known silent substitutions are
  // recorded as substitution notes; under strictSolverResolution they become an
  // error instead.
  compute::ResolvedSolverConfiguration resolvedConfiguration;

  const char* rigidSolver = "unknown";
  switch (m_rigidBodySolver) {
    case RigidBodySolver::SequentialImpulse:
      rigidSolver = "sequential-impulse";
      break;
    case RigidBodySolver::Avbd:
      rigidSolver = "avbd";
      break;
    case RigidBodySolver::Vbd:
      rigidSolver = "vbd";
      break;
    case RigidBodySolver::Ipc:
      rigidSolver = "ipc";
      break;
  }
  resolvedConfiguration.notes.push_back(
      {"rigid-body", rigidSolver, rigidSolver, "as requested"});
  if (m_rigidBodySolver == RigidBodySolver::Avbd) {
    const auto& profile = detail::deformable_vbd::kAvbdRigidPaper2025Profile;
    resolvedConfiguration.notes.push_back(
        {"rigid-avbd-parameter-profile",
         "paper-2025-table-2",
         "paper-2025-table-2",
         std::format(
             "immutable public AVBD parameters: alpha={}, beta={}, gamma={}",
             profile.alpha,
             profile.beta,
             profile.gamma)});
  }

  const char* contactMethod = "unknown";
  switch (m_contactSolverMethod) {
    case ContactSolverMethod::SequentialImpulse:
      contactMethod = "sequential-impulse";
      break;
    case ContactSolverMethod::BoxedLcp:
      contactMethod = "boxed-lcp";
      break;
  }
  // The compatibility-only internal AVBD rigid-contact opt-in is emplaced per
  // body. When it is present under a non-VBD/AVBD public family, the resolved
  // contact path differs from the requested `ContactSolverMethod`: configured
  // contacts run AVBD only when every active contact has an enabled config on
  // at least one endpoint (PLAN-091 WP-091.1). Record that substitution only
  // when the baked collision topology guarantees this all-or-nothing stage
  // gate. The public VBD and AVBD families resolve their owned formulations as
  // requested and do not use this substitution path.
  const detail::WorldRegistry& registry = m_storage->registry;
  const auto* avbdStorage = registry.storage<comps::RigidAvbdContactConfig>();
  const bool hasAvbdContactConfigs
      = avbdStorage != nullptr && avbdStorage->size() != 0u;
  std::size_t collisionEntityCount = 0u;
  std::size_t collisionEntitiesWithoutEnabledAvbdConfig = 0u;
  const auto collisionGeometryView = registry.view<comps::CollisionGeometry>();
  for (const entt::entity entity : collisionGeometryView) {
    if (!collisionGeometryView.get<comps::CollisionGeometry>(entity)
             .hasShapes()) {
      continue;
    }
    ++collisionEntityCount;
    const auto* config
        = registry.try_get<comps::RigidAvbdContactConfig>(entity);
    if (config == nullptr || !config->enabled) {
      ++collisionEntitiesWithoutEnabledAvbdConfig;
    }
  }
  // The compatibility contact stage is all-or-nothing for an active manifold:
  // every contact must have an enabled opt-in on at least one endpoint. Before
  // contacts exist, reporting AVBD is therefore safe only when every possible
  // pair of collision entities is covered, which is equivalent to at most one
  // collision entity lacking an enabled config.
  const bool avbdContactOptInCoversEveryPotentialPair
      = collisionEntityCount >= 2u
        && collisionEntitiesWithoutEnabledAvbdConfig <= 1u;
  if (m_rigidBodySolver == RigidBodySolver::Vbd) {
    resolvedConfiguration.notes.push_back(
        {"rigid-contact", "vbd", "vbd", "as requested"});
  } else if (m_rigidBodySolver == RigidBodySolver::Avbd) {
    resolvedConfiguration.notes.push_back(
        {"rigid-contact", "avbd", "avbd", "as requested"});
  } else if (
      hasAvbdContactConfigs && avbdContactOptInCoversEveryPotentialPair) {
    resolvedConfiguration.notes.push_back(
        {"rigid-contact",
         contactMethod,
         std::string(contactMethod) + " + avbd (opt-in)",
         "internal AVBD rigid-contact opt-in covers every potential active "
         "contact, so the compatibility stage resolves AVBD "
         "(not facade-selectable -- PLAN-091 WP-091.1)"});
  } else {
    resolvedConfiguration.notes.push_back(
        {"rigid-contact",
         contactMethod,
         contactMethod,
         hasAvbdContactConfigs
             ? "as requested; compatibility AVBD opt-in is disabled or does "
               "not cover every potential active contact"
             : "as requested"});
  }

  const bool hasRigidPointJoints
      = detail::deformable_vbd::mayHaveAvbdRigidWorldPointJointConfigs(
          registry);
  const bool hasRigidDistanceSprings
      = detail::deformable_vbd::mayHaveAvbdRigidWorldDistanceSpringConfigs(
          registry);
  if (!hasRigidPointJoints && !hasRigidDistanceSprings) {
    resolvedConfiguration.notes.push_back(
        {"rigid-pair-constraint",
         "inactive",
         "inactive",
         "no rigid pair constraints configured"});
  } else if (m_rigidBodySolver == RigidBodySolver::Vbd) {
    resolvedConfiguration.notes.push_back(
        {"rigid-pair-constraint", "vbd", "vbd", "as requested"});
  } else if (m_rigidBodySolver == RigidBodySolver::Avbd) {
    resolvedConfiguration.notes.push_back(
        {"rigid-pair-constraint", "avbd", "avbd", "as requested"});
  } else if (m_rigidBodySolver == RigidBodySolver::Ipc) {
    resolvedConfiguration.notes.push_back(
        {"rigid-pair-constraint",
         "ipc",
         "ipc",
         "public fixed and revolute pair constraints enter the IPC "
         "articulation equality solve"});
  } else if (hasRigidPointJoints && !hasRigidDistanceSprings) {
    resolvedConfiguration.notes.push_back(
        {"rigid-pair-constraint",
         rigidSolver,
         rigidSolver,
         "hard public rigid pair constraints use solver-owned "
         "sequential-impulse rows"});
  } else if (!hasRigidPointJoints && hasRigidDistanceSprings) {
    resolvedConfiguration.notes.push_back(
        {"rigid-pair-constraint",
         rigidSolver,
         "avbd-distance-spring",
         "experimental finite-stiffness distance springs retain their "
         "explicit AVBD compatibility projection"});
  } else {
    resolvedConfiguration.notes.push_back(
        {"rigid-pair-constraint",
         rigidSolver,
         std::string(rigidSolver) + " + avbd-distance-spring",
         "hard public rigid pair constraints use solver-owned "
         "sequential-impulse rows while experimental finite-stiffness "
         "distance springs retain their explicit AVBD compatibility "
         "projection"});
  }

  const std::string rigidConstraintIterations
      = std::to_string(m_rigidConstraintOptions.iterations);
  if (m_rigidBodySolver == RigidBodySolver::Ipc) {
    resolvedConfiguration.notes.push_back(
        {"rigid-constraint-iterations",
         "not-applicable",
         "not-applicable",
         "the IPC family bypasses the split rigid contact stage"});
  } else if (
      usesUnifiedConstraintStage(
          *this,
          m_rigidBodySolver,
          m_multibodyIntegrationMethod
              == MultibodyIntegrationMethod::Variational)) {
    resolvedConfiguration.notes.push_back(
        {"rigid-constraint-iterations",
         "not-applicable",
         "not-applicable",
         "semi-implicit multibody structures select the unified constraint "
         "stage"});
  } else {
    resolvedConfiguration.notes.push_back(
        {"rigid-constraint-iterations",
         rigidConstraintIterations,
         rigidConstraintIterations,
         "as requested"});
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
  resolvedConfiguration.notes.push_back(
      {"multibody", multibody, multibody, "as requested"});

  const auto deformableView = registry.view<comps::DeformableBodyTag>();
  std::size_t deformableBodyCount = 0u;
  std::size_t vbdBodyCount = 0u;
  std::size_t requiredVbdBodyCount = 0u;
  for (const entt::entity entity : deformableView) {
    ++deformableBodyCount;
    const auto* config = registry.try_get<comps::DeformableVbdConfig>(entity);
    if (config == nullptr || !config->enabled) {
      continue;
    }
    ++vbdBodyCount;
    if (config->requireVbdExecution) {
      ++requiredVbdBodyCount;
    }
  }
  if (deformableBodyCount == 0u) {
    resolvedConfiguration.notes.push_back(
        {"deformable-inner-solver",
         "inactive",
         "inactive",
         "no deformable bodies configured"});
  } else if (vbdBodyCount == 0u) {
    resolvedConfiguration.notes.push_back(
        {"deformable-inner-solver",
         "projected-newton",
         "projected-newton",
         "as requested"});
  } else if (
      vbdBodyCount == deformableBodyCount
      && requiredVbdBodyCount == vbdBodyCount) {
    resolvedConfiguration.notes.push_back(
        {"deformable-inner-solver",
         "vbd",
         "vbd",
         "public per-body VBD selections are required to execute or fail "
         "closed"});
  } else if (requiredVbdBodyCount != vbdBodyCount) {
    const bool allBodiesSelectVbd = vbdBodyCount == deformableBodyCount;
    resolvedConfiguration.notes.push_back(
        {"deformable-inner-solver",
         allBodiesSelectVbd ? (requiredVbdBodyCount == 0u
                                   ? "vbd (internal opt-in)"
                                   : "vbd (mixed public/internal selections)")
                            : "per-body mixed",
         requiredVbdBodyCount == 0u
             ? "runtime vbd-or-projected-newton"
             : "per-body required-vbd + runtime vbd-or-projected-newton",
         std::format(
             "{} of {} VBD selections require VBD execution; {} internal "
             "compatibility opt-ins may fall back to projected Newton for "
             "unsupported runtime envelopes",
             requiredVbdBodyCount,
             vbdBodyCount,
             vbdBodyCount - requiredVbdBodyCount)});
  } else {
    resolvedConfiguration.notes.push_back(
        {"deformable-inner-solver",
         "per-body mixed",
         "per-body mixed",
         std::format(
             "{} of {} deformable bodies select VBD; {} selections require "
             "VBD execution",
             vbdBodyCount,
             deformableBodyCount,
             requiredVbdBodyCount)});
  }

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
      = deformablePsdAcceleratedResolved ? "accelerated" : "cpu";
  const char* acceleratorReason = "as requested";
  if (m_computeAcceleratorPolicy == ComputeAcceleratorPolicy::PreferAccelerated
      && !deformablePsdAcceleratedResolved) {
    acceleratorReason = "no available accelerator registered";
  }
  resolvedConfiguration.notes.push_back(
      {"deformable-psd",
       requestedAccelerator,
       resolvedAccelerator,
       acceleratorReason});

  DART_SIMULATION_THROW_T_IF(
      m_strictSolverResolution && resolvedConfiguration.hasSubstitution(),
      InvalidArgumentException,
      "strict solver resolution is enabled and the World substituted a solver "
      "method it did not request; inspect getResolvedConfiguration() for the "
      "recorded substitution");

  return resolvedConfiguration;
}

//==============================================================================
void World::preflightStrictSolverResolution() const
{
  if (!m_strictSolverResolution) {
    return;
  }

  const bool acceleratedResolved
      = m_computeAcceleratorPolicy
            == ComputeAcceleratorPolicy::PreferAccelerated
        && compute::deformablePsdAcceleratorProjector() != nullptr;
  static_cast<void>(buildResolvedConfiguration(acceleratedResolved));
}

//==============================================================================
void World::recordResolvedConfiguration()
{
  m_resolvedConfiguration
      = buildResolvedConfiguration(m_deformablePsdAcceleratedResolved);
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
      isRigidBlockDescentSolver(m_rigidBodySolver),
      InvalidOperationException,
      "Multibody structures are not supported by the VBD or AVBD rigid-body "
      "solver");
  DART_SIMULATION_THROW_T_IF(
      hasRigidBodyAvbdPairConstraints(*this),
      InvalidOperationException,
      "Multibody structures are not supported in worlds with rigid-body "
      "pair constraints under Sequential Impulse, VBD, or AVBD");

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
Joint World::addJoint(
    const Frame& parent, const Frame& child, const JointSpec& spec)
{
  ensureDesignMode();

  DART_SIMULATION_THROW_T_IF(
      !parent.isWorld() && !parent.isValid(),
      InvalidArgumentException,
      "Joint parent frame is invalid or has been destroyed");
  DART_SIMULATION_THROW_T_IF(
      child.isWorld() || !child.isValid(),
      InvalidArgumentException,
      "Joint child frame is invalid or cannot be the world frame");
  DART_SIMULATION_THROW_T_IF(
      !parent.isWorld() && parent.getWorld() != this,
      InvalidArgumentException,
      "Joint parent frame must belong to this World");
  DART_SIMULATION_THROW_T_IF(
      child.getWorld() != this,
      InvalidArgumentException,
      "Joint child frame must belong to this World");

  const entt::entity parentEntity
      = parent.isWorld() ? entt::null
                         : detail::toRegistryEntity(parent.getEntity());
  const entt::entity childEntity = detail::toRegistryEntity(child.getEntity());
  const bool parentIsLink
      = parentEntity != entt::null
        && m_storage->registry.all_of<comps::LinkModel>(parentEntity);
  const bool childIsLink
      = m_storage->registry.all_of<comps::LinkModel>(childEntity);
  const bool parentIsRigidBody
      = parentEntity != entt::null
        && m_storage->registry.all_of<comps::RigidBodyTag>(parentEntity);
  const bool childIsRigidBody
      = m_storage->registry.all_of<comps::RigidBodyTag>(childEntity);

  if (childIsLink && (parent.isWorld() || parentIsLink)) {
    Link childLink(child.getEntity(), this);
    if (parent.isWorld()) {
      return createArticulatedPointJoint(
          spec.name,
          nullptr,
          childLink,
          spec.type,
          spec.axis,
          spec.parentAnchor,
          spec.childAnchor);
    }

    Link parentLink(parent.getEntity(), this);
    return createArticulatedPointJoint(
        spec.name,
        &parentLink,
        childLink,
        spec.type,
        spec.axis,
        spec.parentAnchor,
        spec.childAnchor);
  }

  if (parentIsRigidBody && childIsRigidBody) {
    return createRigidBodyPointJoint(
        spec.name,
        RigidBody(parent.getEntity(), this),
        RigidBody(child.getEntity(), this),
        spec.type,
        spec.axis,
        spec.parentAnchor,
        spec.childAnchor);
  }

  DART_SIMULATION_THROW_T(
      InvalidArgumentException,
      "World joints currently require Link-Link, world-Link, or "
      "RigidBody-RigidBody frame endpoints");
}

//==============================================================================
Joint World::addJoint(const Frame& child, const JointSpec& spec)
{
  return addJoint(Frame::world(), child, spec);
}

//==============================================================================
Joint World::createArticulatedPointJoint(
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
  jointActuation.commandAcceleration = comps::makeJointVector(dof, 0.0);

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

    jointModel.hasRigidBodyPairConstraintGeometry = true;
    jointModel.rigidBodyPairConstraintLocalAnchorParent = *parentAnchor;
    jointModel.rigidBodyPairConstraintLocalAnchorChild = *childAnchor;
    jointModel.rigidBodyPairConstraintTargetRelativeOrientation
        = parentOrientation.conjugate() * childOrientation;
    jointModel.rigidBodyPairConstraintTargetRelativeOrientation.normalize();
  }

  return Joint(detail::fromRegistryEntity(jointEntity), this);
}

//==============================================================================
std::optional<Joint> World::getJoint(std::string_view name)
{
  auto view = m_storage->registry.view<comps::JointModel, comps::Name>();
  for (auto entity : view) {
    const auto& joint = view.get<comps::JointModel>(entity);
    const auto& info = view.get<comps::Name>(entity);
    if (info.name == name
        && isWorldOwnedJoint(m_storage->registry, entity, joint)) {
      return Joint(detail::fromRegistryEntity(entity), this);
    }
  }
  return std::nullopt;
}

//==============================================================================
bool World::hasJoint(std::string_view name) const
{
  const auto view = m_storage->registry.view<comps::JointModel, comps::Name>();
  for (auto entity : view) {
    const auto& joint = view.get<comps::JointModel>(entity);
    const auto& info = view.get<comps::Name>(entity);
    if (info.name == name
        && isWorldOwnedJoint(m_storage->registry, entity, joint)) {
      return true;
    }
  }
  return false;
}

//==============================================================================
std::size_t World::getJointCount() const
{
  std::size_t count = 0;
  const auto view = m_storage->registry.view<comps::JointModel>();
  for (auto entity : view) {
    const auto& joint = view.get<comps::JointModel>(entity);
    if (isWorldOwnedJoint(m_storage->registry, entity, joint)) {
      ++count;
    }
  }
  return count;
}

//==============================================================================
std::vector<Joint> World::getJoints()
{
  std::vector<Joint> joints;
  const auto view = m_storage->registry.view<comps::JointModel>();
  for (auto entity : view) {
    const auto& joint = view.get<comps::JointModel>(entity);
    if (isWorldOwnedJoint(m_storage->registry, entity, joint)) {
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
Joint World::createRigidBodyPointJoint(
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

    jointModel.hasRigidBodyPairConstraintGeometry = true;
    jointModel.rigidBodyPairConstraintLocalAnchorParent = *parentAnchor;
    jointModel.rigidBodyPairConstraintLocalAnchorChild = *childAnchor;
    jointModel.rigidBodyPairConstraintTargetRelativeOrientation
        = parentOrientation.conjugate() * childOrientation;
    jointModel.rigidBodyPairConstraintTargetRelativeOrientation.normalize();
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
  jointActuation.commandAcceleration = comps::makeJointVector(dof, 0.0);

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
std::vector<std::string> World::getRigidBodyNames() const
{
  std::vector<std::string> names;
  auto view = m_storage->registry.view<comps::RigidBodyTag, comps::Name>();
  for (auto entity : view) {
    names.push_back(view.get<comps::Name>(entity).name);
  }
  std::sort(names.begin(), names.end());
  return names;
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
  state.attachmentTargets = state.positions;

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

  m_storage->registry.emplace<comps::DeformableContactConfig>(
      entity, data.contactConfig);

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
  ensureDesignMode();
  validateDeformableSolverOptions(options);
  auto view = m_storage->registry.view<comps::DeformableBodyTag, comps::Name>();
  for (auto entity : view) {
    if (view.get<comps::Name>(entity).name != name) {
      continue;
    }
    // Translate the public, solver-agnostic options into the internal opt-in
    // inner-solver component. The mapping lives here (the World step pipeline
    // owns the concrete solver) so the public facade stays algorithm-neutral.
    comps::DeformableVbdConfig config{/*enabled=*/true,
                                      options.iterations,
                                      options.convergenceTolerance,
                                      options.useAcceleration,
                                      options.accelerationSpectralRadius,
                                      options.stiffnessDamping,
                                      options.groundContactStiffness};
    config.requireVbdExecution = true;
    m_storage->registry.emplace_or_replace<comps::DeformableVbdConfig>(
        entity, config);
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

  validateReplayConstruction();

  // Public VBD selection is fail-closed. Validate its complete obstacle
  // envelope before setting simulation mode or updating any cached state so a
  // rejected entry is observationally atomic.
  validateRequiredDeformableVbdConfiguration(*this);
  validateLoopClosureKinematicsPolicySupport(*this);
  validateRigidSolverContactMethodCompatibility(
      m_rigidBodySolver, m_contactSolverMethod);
  validateRigidBodyJointPipelineSupport(*this, m_rigidBodySolver);
  validateRigidConstraintOptionsPipelineSupport(
      *this,
      m_rigidConstraintOptions,
      m_rigidBodySolver,
      m_multibodyIntegrationMethod == MultibodyIntegrationMethod::Variational);
  validateArticulatedPointJointPipelineSupport(
      *this,
      m_multibodyIntegrationMethod == MultibodyIntegrationMethod::Variational);
  // Strict resolution is a logical rejection boundary, not a preparation
  // side effect. Build and discard the candidate report before collision
  // preflight can bake automatic capacities or any kinematics can refresh
  // articulated reference poses. Accelerator availability is read directly,
  // leaving both the public report and the live projector selection untouched.
  preflightStrictSolverResolution();

  struct EntryFrameCacheSnapshot
  {
    entt::entity entity = entt::null;
    comps::FrameCache cache;
  };
  struct EntryPointJointSnapshot
  {
    entt::entity entity = entt::null;
    bool hasGeometry = false;
    Eigen::Vector3d localAnchorParent = Eigen::Vector3d::Zero();
    Eigen::Vector3d localAnchorChild = Eigen::Vector3d::Zero();
    Eigen::Quaterniond targetRelativeOrientation
        = Eigen::Quaterniond::Identity();
    std::optional<detail::deformable_vbd::AvbdRigidWorldPointJointConfig>
        config;
    std::optional<comps::AvbdJointStiffness> stiffness;
  };

  // Capture every registry value that entry may update before constructing
  // replacement caches. All allocation therefore happens while the live World
  // is still untouched; rollback below is assignment/removal only.
  auto& rollbackAllocator = m_memoryManager.getFreeAllocator();
  std::vector<EntryFrameCacheSnapshot> frameCacheSnapshots;
  const auto frameCacheView = m_storage->registry.view<comps::FrameCache>();
  const auto& frameCacheStorage
      = m_storage->registry.storage<comps::FrameCache>();
  frameCacheSnapshots.reserve(frameCacheStorage.size());
  for (const entt::entity entity : frameCacheView) {
    frameCacheSnapshots.push_back(
        {entity, frameCacheView.get<comps::FrameCache>(entity)});
  }

  std::vector<EntryPointJointSnapshot> pointJointSnapshots;
  const auto jointView = m_storage->registry.view<comps::JointModel>();
  const auto& jointStorage = m_storage->registry.storage<comps::JointModel>();
  pointJointSnapshots.reserve(jointStorage.size());
  for (const entt::entity entity : jointView) {
    const auto& joint = jointView.get<comps::JointModel>(entity);
    EntryPointJointSnapshot snapshot;
    snapshot.entity = entity;
    snapshot.hasGeometry = joint.hasRigidBodyPairConstraintGeometry;
    snapshot.localAnchorParent = joint.rigidBodyPairConstraintLocalAnchorParent;
    snapshot.localAnchorChild = joint.rigidBodyPairConstraintLocalAnchorChild;
    snapshot.targetRelativeOrientation
        = joint.rigidBodyPairConstraintTargetRelativeOrientation;
    if (const auto* config
        = m_storage->registry
              .try_get<detail::deformable_vbd::AvbdRigidWorldPointJointConfig>(
                  entity)) {
      snapshot.config = *config;
    }
    if (const auto* stiffness
        = m_storage->registry.try_get<comps::AvbdJointStiffness>(entity)) {
      snapshot.stiffness = *stiffness;
    }
    pointJointSnapshots.push_back(std::move(snapshot));
  }

  auto variationalStateSnapshots
      = captureReplayComponents<compute::MultibodyVariationalState>(
          m_storage->registry, rollbackAllocator);
  auto variationalDualSnapshots
      = captureReplayComponents<comps::VariationalContactDualState>(
          m_storage->registry, rollbackAllocator);
  auto deformableContactConfigSnapshots
      = captureReplayComponents<comps::DeformableContactConfig>(
          m_storage->registry, rollbackAllocator);

  auto replacementStepPipelineCache = makeStepPipelineCache(
      m_memoryManager, m_rigidConstraintOptions.iterations);
  auto previousStepPipelineCache = std::move(m_stepPipelineCache);
  m_stepPipelineCache = std::move(replacementStepPipelineCache);
  auto previousCollisionQueryCache = std::move(m_collisionQueryCache);

  detail::BakedWorldModel previousBakedModel(rollbackAllocator);
  std::swap(previousBakedModel, m_storage->bakedModel);
  compute::ResolvedSolverConfiguration previousResolvedConfiguration;
  previousResolvedConfiguration.notes.swap(m_resolvedConfiguration.notes);
  std::optional<StepDerivatives> previousStepDerivatives;
  previousStepDerivatives.swap(m_storage->stepDerivatives);
  const bool previousStepDerivativesValid = m_storage->stepDerivativesValid;
  m_storage->stepDerivativesValid = false;
  const DeformablePsdProjector previousDeformablePsdProjector
      = m_deformablePsdProjector;
  const bool previousDeformablePsdAcceleratedResolved
      = m_deformablePsdAcceleratedResolved;

  try {
    // Rigid collision capacities and compatibility-contact configuration are
    // construction-time rejection boundaries. Check them before kinematics or
    // joint reference geometry can be updated so failed entry is atomic with
    // respect to World state.
    m_stepPipelineCache->stages.rigidBodyContact.preflight(*this);
    // Initial kinematic bake without crossing the public mode boundary. The
    // public sync() facade intentionally requires simulation mode, but the
    // owning stage can prepare and execute safely while entry is still being
    // validated. This keeps a later strict-resolution rejection in design mode.
    compute::SequentialExecutor executor;
    m_stepPipelineCache->stages.kinematics.execute(*this, executor);
    detail::deformable_vbd::configureAvbdRigidWorldPointJointsFromCurrentPoses(
        m_storage->registry);
    prepareStepPipelineCacheForCurrentConfiguration();
    m_simulationMode = true;
    if (m_collisionQueryCache) {
      m_collisionQueryCache->capacitiesLocked = true;
    }
  } catch (...) {
    const std::exception_ptr entryFailure = std::current_exception();

    for (const entt::entity entity :
         m_storage->registry.view<comps::FrameTag>()) {
      bool existedAtEntry = false;
      for (const auto& snapshot : frameCacheSnapshots) {
        if (snapshot.entity == entity) {
          existedAtEntry = true;
          break;
        }
      }
      if (!existedAtEntry
          && m_storage->registry.all_of<comps::FrameCache>(entity)) {
        m_storage->registry.remove<comps::FrameCache>(entity);
      }
    }
    for (const auto& snapshot : frameCacheSnapshots) {
      m_storage->registry.get<comps::FrameCache>(snapshot.entity)
          = snapshot.cache;
    }
    for (const auto& snapshot : pointJointSnapshots) {
      auto& joint = m_storage->registry.get<comps::JointModel>(snapshot.entity);
      joint.hasRigidBodyPairConstraintGeometry = snapshot.hasGeometry;
      joint.rigidBodyPairConstraintLocalAnchorParent
          = snapshot.localAnchorParent;
      joint.rigidBodyPairConstraintLocalAnchorChild = snapshot.localAnchorChild;
      joint.rigidBodyPairConstraintTargetRelativeOrientation
          = snapshot.targetRelativeOrientation;
      if (snapshot.config) {
        m_storage->registry
            .get<detail::deformable_vbd::AvbdRigidWorldPointJointConfig>(
                snapshot.entity) = *snapshot.config;
      } else {
        m_storage->registry
            .remove<detail::deformable_vbd::AvbdRigidWorldPointJointConfig>(
                snapshot.entity);
      }
      if (snapshot.stiffness) {
        m_storage->registry.get<comps::AvbdJointStiffness>(snapshot.entity)
            = *snapshot.stiffness;
      } else {
        m_storage->registry.remove<comps::AvbdJointStiffness>(snapshot.entity);
      }
    }

    const auto containsEntity
        = [](const auto& snapshots, entt::entity entity) noexcept {
            for (const auto& [snapshotEntity, component] : snapshots) {
              static_cast<void>(component);
              if (snapshotEntity == entity) {
                return true;
              }
            }
            return false;
          };

    // reserve/prepare may lazily create serialized persistent components.
    // Remove components that did not exist at the entry boundary, then swap
    // allocator-bearing payloads back into every pre-existing component. The
    // snapshot allocations were completed before any mutation, so this
    // rollback remains non-allocating even when the base allocator keeps
    // failing.
    for (const entt::entity entity :
         m_storage->registry.view<comps::MultibodyStructure>()) {
      if (m_storage->registry.all_of<compute::MultibodyVariationalState>(entity)
          && !containsEntity(variationalStateSnapshots, entity)) {
        m_storage->registry.remove<compute::MultibodyVariationalState>(entity);
      }
      if (m_storage->registry.all_of<comps::VariationalContactDualState>(entity)
          && !containsEntity(variationalDualSnapshots, entity)) {
        m_storage->registry.remove<comps::VariationalContactDualState>(entity);
      }
    }
    for (auto& [entity, previous] : variationalStateSnapshots) {
      auto& current
          = m_storage->registry.get<compute::MultibodyVariationalState>(entity);
      std::swap(current.bootstrapped, previous.bootstrapped);
      current.previousDeltaTransform.swap(previous.previousDeltaTransform);
      current.previousMomentum.swap(previous.previousMomentum);
    }
    for (auto& [entity, previous] : variationalDualSnapshots) {
      auto& current
          = m_storage->registry.get<comps::VariationalContactDualState>(entity);
      current.duals.swap(previous.duals);
      std::swap(current.stepsSinceDualUpdate, previous.stepsSinceDualUpdate);
    }
    for (const entt::entity entity :
         m_storage->registry.view<comps::DeformableBodyTag>()) {
      if (m_storage->registry.all_of<comps::DeformableContactConfig>(entity)
          && !containsEntity(deformableContactConfigSnapshots, entity)) {
        m_storage->registry.remove<comps::DeformableContactConfig>(entity);
      }
    }
    for (const auto& [entity, previous] : deformableContactConfigSnapshots) {
      m_storage->registry.get<comps::DeformableContactConfig>(entity)
          = previous;
    }

    std::swap(previousBakedModel, m_storage->bakedModel);
    previousResolvedConfiguration.notes.swap(m_resolvedConfiguration.notes);
    previousStepDerivatives.swap(m_storage->stepDerivatives);
    m_storage->stepDerivativesValid = previousStepDerivativesValid;
    m_deformablePsdProjector = previousDeformablePsdProjector;
    m_deformablePsdAcceleratedResolved
        = previousDeformablePsdAcceleratedResolved;

    m_collisionQueryCache.reset();
    m_collisionQueryCache = std::move(previousCollisionQueryCache);
    m_stepPipelineCache.reset();
    m_stepPipelineCache = std::move(previousStepPipelineCache);
    std::rethrow_exception(entryFailure);
  }
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
  validateRigidSolverContactMethodCompatibility(solver, m_contactSolverMethod);
  (void)validateRigidConstraintOptions(m_rigidConstraintOptions, solver);

  if (m_rigidBodySolver == solver) {
    return;
  }

  validateRigidBodyJointPipelineSupport(*this, solver);
  validateRigidConstraintOptionsPipelineSupport(
      *this,
      m_rigidConstraintOptions,
      solver,
      m_multibodyIntegrationMethod == MultibodyIntegrationMethod::Variational);
  const RigidBodySolver previousSolver = m_rigidBodySolver;
  if (!m_simulationMode) {
    m_rigidBodySolver = solver;
    if (solver == RigidBodySolver::Ipc
        || previousSolver == RigidBodySolver::Ipc) {
      resetRigidIpcAdaptiveBarrierStiffnessLowerBound();
    }
    return;
  }

  applyPreparedConfigurationTransactionally(
      [&] { m_rigidBodySolver = solver; },
      [&] { m_rigidBodySolver = previousSolver; });
  if (solver == RigidBodySolver::Ipc
      || previousSolver == RigidBodySolver::Ipc) {
    // IPC's adaptive barrier lower bound is continuation state for one
    // uninterrupted IPC trajectory. Entering or leaving the family creates an
    // inactive gap, so commit a cold start only after the transactional
    // transition succeeds.
    resetRigidIpcAdaptiveBarrierStiffnessLowerBound();
  }
  if (solver == RigidBodySolver::Vbd || solver == RigidBodySolver::Ipc
      || previousSolver == RigidBodySolver::Vbd
      || previousSolver == RigidBodySolver::Ipc) {
    // Fixed-penalty VBD must not inherit AVBD dual/stiffness continuation, and
    // IPC bypasses this stage entirely. A transition out of either family also
    // cold-starts all AVBD inventories because no compatible continuation ran
    // across the inactive gap. Clearing during tentative preparation makes a
    // rejected public setter observably destructive, so commit the infallible
    // invalidation only after the complete transition succeeds.
    m_stepPipelineCache->stages.rigidBodyContact
        .clearAvbdWarmStartContinuationState();
  } else {
    // Crossing between public AVBD and Sequential Impulse invalidates ordinary
    // contact plus public joint and motor rows. Preserve only the explicitly
    // active compatibility distance-spring continuation, which both families
    // project through this stage on the same paper-profile schedule.
    m_stepPipelineCache->stages.rigidBodyContact
        .clearSequentialImpulseOwnedAvbdWarmStartContinuationState();
  }
}

//==============================================================================
RigidBodySolver World::getRigidBodySolver() const noexcept
{
  return m_rigidBodySolver;
}

//==============================================================================
void World::setRigidConstraintOptions(const RigidConstraintOptions& options)
{
  const auto validated
      = validateRigidConstraintOptions(options, m_rigidBodySolver);
  validateRigidConstraintOptionsPipelineSupport(
      *this,
      validated,
      m_rigidBodySolver,
      m_multibodyIntegrationMethod == MultibodyIntegrationMethod::Variational);
  if (!m_simulationMode) {
    m_rigidConstraintOptions = validated;
    m_stepPipelineCache->stages.rigidBodyContact.setIterations(
        m_rigidConstraintOptions.iterations);
    return;
  }

  const RigidConstraintOptions previousOptions = m_rigidConstraintOptions;
  applyPreparedConfigurationTransactionally(
      [&] { m_rigidConstraintOptions = validated; },
      [&] { m_rigidConstraintOptions = previousOptions; });
}

//==============================================================================
const RigidConstraintOptions& World::getRigidConstraintOptions() const noexcept
{
  return m_rigidConstraintOptions;
}

//==============================================================================
const RigidCollisionCapacityOptions& World::getRigidCollisionCapacityOptions()
    const noexcept
{
  return m_rigidCollisionCapacityOptions;
}

//==============================================================================
std::size_t World::getRigidCollisionCandidatePairCapacity() const noexcept
{
  if (m_collisionQueryCache && m_collisionQueryCache->capacitiesPrepared) {
    return m_collisionQueryCache->candidatePairCapacity;
  }
  return m_rigidCollisionCapacityOptions.candidatePairCapacity;
}

//==============================================================================
std::size_t World::getRigidCollisionContactCapacity() const noexcept
{
  if (m_collisionQueryCache && m_collisionQueryCache->capacitiesPrepared) {
    return m_collisionQueryCache->contactCapacity;
  }
  return m_rigidCollisionCapacityOptions.contactCapacity;
}

//==============================================================================
std::size_t World::getRigidCollisionContactReserve() const noexcept
{
  if (m_collisionQueryCache != nullptr
      && m_collisionQueryCache->capacitiesPrepared) {
    return m_collisionQueryCache->contactReserve;
  }
  return m_rigidCollisionCapacityOptions.contactCapacity;
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
  validateRigidSolverContactMethodCompatibility(m_rigidBodySolver, method);

  if (m_contactSolverMethod == method) {
    return;
  }

  if (!m_simulationMode) {
    m_contactSolverMethod = method;
    return;
  }

  const ContactSolverMethod previousMethod = m_contactSolverMethod;
  applyPreparedConfigurationTransactionally(
      [&] { m_contactSolverMethod = method; },
      [&] { m_contactSolverMethod = previousMethod; });
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

  if (!m_simulationMode) {
    m_computeAcceleratorPolicy = policy;
    return;
  }

  const ComputeAcceleratorPolicy previousPolicy = m_computeAcceleratorPolicy;
  applyPreparedConfigurationTransactionally(
      [&] { m_computeAcceleratorPolicy = policy; },
      [&] { m_computeAcceleratorPolicy = previousPolicy; });
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
                           > disturbanceThresholdSquared
                    || jointActuation->commandAcceleration.squaredNorm()
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
                             > disturbanceThresholdSquared
                      || jointActuation->commandAcceleration.squaredNorm()
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
      !m_storage->stepDerivatives.has_value()
          || !m_storage->stepDerivativesValid,
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
  return countWorldDofs(detail::ensureBakedWorldModelCurrent(*this));
}

//==============================================================================
std::size_t World::getNumEfforts() const
{
  return getNumDofs();
}

//==============================================================================
Eigen::VectorXd World::getStateVector() const
{
  const auto& model = detail::ensureBakedWorldModelCurrent(*this);
  const auto& bodies = model.dynamicRigidBodyEntities;
  const Eigen::Index rigidDofs
      = static_cast<Eigen::Index>(countRigidBodyDofs(model));
  const Eigen::Index dofs = static_cast<Eigen::Index>(countWorldDofs(model));
  Eigen::VectorXd state(2 * dofs);
  for (std::size_t k = 0; k < bodies.size(); ++k) {
    const auto base = 3 * static_cast<Eigen::Index>(k);
    const auto& transform
        = m_storage->registry.get<comps::Transform>(bodies[k]);
    const auto& velocity = m_storage->registry.get<comps::Velocity>(bodies[k]);
    state.segment<3>(base) = transform.position;
    state.segment<3>(dofs + base) = velocity.linear;
  }
  Eigen::Index offset = rigidDofs;
  for (const auto jointEntity : model.multibodyJointEntities) {
    const auto& jointState
        = m_storage->registry.get<comps::JointState>(jointEntity);
    const auto jointDofs = jointState.position.size();
    state.segment(offset, jointDofs) = jointState.position;
    state.segment(dofs + offset, jointDofs) = jointState.velocity;
    offset += jointDofs;
  }
  return state;
}

//==============================================================================
void World::setStateVector(const Eigen::VectorXd& state)
{
  const auto& model = detail::ensureBakedWorldModelCurrent(*this);
  const auto& bodies = model.dynamicRigidBodyEntities;
  const Eigen::Index rigidDofs
      = static_cast<Eigen::Index>(countRigidBodyDofs(model));
  const Eigen::Index dofs = static_cast<Eigen::Index>(countWorldDofs(model));
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
  Eigen::Index offset = rigidDofs;
  bool wroteMultibodyPosition = false;
  for (const auto jointEntity : model.multibodyJointEntities) {
    auto& jointState = m_storage->registry.get<comps::JointState>(jointEntity);
    const auto jointDofs = jointState.position.size();
    jointState.position = state.segment(offset, jointDofs);
    jointState.velocity = state.segment(dofs + offset, jointDofs);
    wroteMultibodyPosition = wroteMultibodyPosition || jointDofs > 0;
    offset += jointDofs;
  }
  if (wroteMultibodyPosition) {
    markFrameCachesDirty(m_storage->registry);
  }
}

//==============================================================================
Eigen::VectorXd World::getControlVector() const
{
  const auto& model = detail::ensureBakedWorldModelCurrent(*this);
  const auto& bodies = model.dynamicRigidBodyEntities;
  const Eigen::Index rigidDofs
      = static_cast<Eigen::Index>(countRigidBodyDofs(model));
  const Eigen::Index dofs = static_cast<Eigen::Index>(countWorldDofs(model));
  Eigen::VectorXd control(dofs);
  for (std::size_t k = 0; k < bodies.size(); ++k) {
    const auto base = 3 * static_cast<Eigen::Index>(k);
    const auto& force = m_storage->registry.get<comps::Force>(bodies[k]);
    control.segment<3>(base) = force.force;
  }
  Eigen::Index offset = rigidDofs;
  for (const auto jointEntity : model.multibodyJointEntities) {
    const auto& actuation
        = m_storage->registry.get<comps::JointActuation>(jointEntity);
    const auto jointDofs = actuation.torque.size();
    control.segment(offset, jointDofs) = actuation.torque;
    offset += jointDofs;
  }
  return control;
}

//==============================================================================
void World::setControlVector(const Eigen::VectorXd& control)
{
  const auto& model = detail::ensureBakedWorldModelCurrent(*this);
  const auto& bodies = model.dynamicRigidBodyEntities;
  const Eigen::Index rigidDofs
      = static_cast<Eigen::Index>(countRigidBodyDofs(model));
  const Eigen::Index dofs = static_cast<Eigen::Index>(countWorldDofs(model));
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
  Eigen::Index offset = rigidDofs;
  for (const auto jointEntity : model.multibodyJointEntities) {
    auto& actuation
        = m_storage->registry.get<comps::JointActuation>(jointEntity);
    const auto jointDofs = actuation.torque.size();
    actuation.torque = control.segment(offset, jointDofs);
    offset += jointDofs;
  }
}

//==============================================================================
std::size_t World::getNumRigidBodyDofs() const
{
  return countRigidBodyDofs(detail::ensureBakedWorldModelCurrent(*this));
}

//==============================================================================
std::size_t World::getNumRigidBodyEfforts() const
{
  return getNumRigidBodyDofs();
}

//==============================================================================
Eigen::VectorXd World::getRigidBodyStateVector() const
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
void World::setRigidBodyStateVector(const Eigen::VectorXd& state)
{
  const auto& bodies
      = detail::ensureBakedWorldModelCurrent(*this).dynamicRigidBodyEntities;
  const Eigen::Index dofs = 3 * static_cast<Eigen::Index>(bodies.size());
  DART_SIMULATION_THROW_T_IF(
      state.size() != 2 * dofs,
      InvalidArgumentException,
      "World::setRigidBodyStateVector(): expected size {} "
      "(= 2 * num_rigid_body_dofs) but got {}",
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
Eigen::VectorXd World::getRigidBodyControlVector() const
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
void World::setRigidBodyControlVector(const Eigen::VectorXd& control)
{
  const auto& bodies
      = detail::ensureBakedWorldModelCurrent(*this).dynamicRigidBodyEntities;
  const Eigen::Index dofs = 3 * static_cast<Eigen::Index>(bodies.size());
  DART_SIMULATION_THROW_T_IF(
      control.size() != dofs,
      InvalidArgumentException,
      "World::setRigidBodyControlVector(): expected size {} "
      "(= num_rigid_body_efforts) but got {}",
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
      m_stepPipelineCache->stages.kinematics.execute(*this, executor);
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
#if DART_BUILD_MEMORY_DIAGNOSTICS
  m_storage->memoryDiagnostics.resetFrameScratch(m_memoryManager);
#else
  // Releasing the frame arena is functional, not diagnostic; only the reset
  // tally is instrumentation, so it compiles out with the option.
  m_memoryManager.getFrameAllocator().reset();
#endif
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
  validateRigidConstraintOptionsPipelineSupport(
      *this,
      m_rigidConstraintOptions,
      m_rigidBodySolver,
      method == MultibodyIntegrationMethod::Variational);

  const MultibodyIntegrationMethod previousMethod
      = m_multibodyIntegrationMethod;
  if (!m_simulationMode) {
    m_multibodyIntegrationMethod = method;
    m_variationalIntegratorMaxIterations = options.variationalMaxIterations;
    m_variationalIntegratorTolerance = options.variationalTolerance;
    if (previousMethod != method) {
      coldStartMultibodyVariationalContinuation(m_storage->registry);
    }
    return;
  }

  const std::size_t previousMaxIterations
      = m_variationalIntegratorMaxIterations;
  const double previousTolerance = m_variationalIntegratorTolerance;
  applyPreparedConfigurationTransactionally(
      [&] {
        m_multibodyIntegrationMethod = method;
        m_variationalIntegratorMaxIterations = options.variationalMaxIterations;
        m_variationalIntegratorTolerance = options.variationalTolerance;
      },
      [&] {
        m_multibodyIntegrationMethod = previousMethod;
        m_variationalIntegratorMaxIterations = previousMaxIterations;
        m_variationalIntegratorTolerance = previousTolerance;
      });
  if (previousMethod != method) {
    // The two-step DEL history and augmented-Lagrangian contact duals are
    // continuation state for an uninterrupted variational trajectory. A
    // semi-implicit gap advances the physical state without advancing either
    // history, so any successful family crossing must cold-start them. Keep
    // their allocated storage for baked-step allocation guarantees.
    coldStartMultibodyVariationalContinuation(m_storage->registry);
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
  // Runtime model edits can change both stage preparation and which domains
  // belong in the default schedule. Refresh before consulting any cached skip
  // or domain flags and before a stage can mutate physical state.
  ensureStepPipelineCacheCurrent();

  if (m_differentiable
#if DART_BUILD_PROFILE
      || m_stepProfilingEnabled
#endif
      || !m_stepPipelineCache->canSkipDefaultPipelineWhenFramesClean
      || hasDirtyFrameCaches(*this)) {
    return false;
  }

#if DART_BUILD_MEMORY_DIAGNOSTICS
  m_storage->memoryDiagnostics.resetFrameScratch(m_memoryManager);
#else
  m_memoryManager.getFrameAllocator().reset();
#endif
  m_lastDeformableSolverDiagnostics = {};
  m_storage->lastStepDiagnostics = {};
  m_storage->lastContactForces.clear();
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
  WorldStepPipelineStages* stages = nullptr;
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
      // The clean-step probe above enters simulation mode on demand and may
      // replace the pipeline cache. Bind the stage owner only after that
      // transition so the cached pipeline cannot retain pointers into the
      // discarded design-mode cache.
      stages = &m_stepPipelineCache->stages;
      pipeline = &stages->buildDefault(
          m_rigidBodySolver,
          m_multibodyIntegrationMethod
              == MultibodyIntegrationMethod::Variational,
          m_stepPipelineCache->hasAdvanceableRigidBodies,
          m_stepPipelineCache->hasMultibodyStructure,
          m_stepPipelineCache->hasDeformableBodies);
    }
    stepPipelineOnce(executor, *pipeline);
    m_lastDeformableSolverDiagnostics = makeDeformableSolverDiagnostics(
        stages->deformableDynamics.getLastStats());
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
  DART_SIMULATION_THROW_T_IF(
      requiresPublicDeformableVbdExecution(*this)
          || (m_replay && m_replay->recordingEnabled),
      InvalidOperationException,
      "Caller-owned final stages are not supported with required public "
      "deformable VBD or while replay recording is enabled; use the built-in "
      "step overload");
  ensureStepPipelineCacheCurrent();
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
  m_callerOwnedPipelineContinuationStateMayBeLive = true;
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
  DART_SIMULATION_THROW_T_IF(
      count > 0u
          && (requiresPublicDeformableVbdExecution(*this)
              || (m_replay && m_replay->recordingEnabled)),
      InvalidOperationException,
      "Caller-owned final stages are not supported with required public "
      "deformable VBD or while replay recording is enabled; use the built-in "
      "step overload");
  for (std::size_t i = 0; i < count; ++i) {
#if DART_BUILD_PROFILE
    StepProfileTimer profileTimer(m_stepProfilingEnabled);
#endif

    // A caller-owned final stage may edit the model. Rebuild on every
    // iteration so the following step observes any changed domain schedule.
    ensureStepPipelineCacheCurrent();
    auto& stages = m_stepPipelineCache->stages;
    compute::WorldStepPipeline& pipeline = stages.buildWithFinalStage(
        m_rigidBodySolver,
        m_multibodyIntegrationMethod == MultibodyIntegrationMethod::Variational,
        hasAdvanceableRigidBodyStructures(*this),
        hasMultibodyStructures(*this),
        hasDeformableBodies(*this),
        stage);
    ScopedWorldStepPipelineClear clearCustomPipeline(pipeline);
    stepPipelineOnce(executor, pipeline);
    m_callerOwnedPipelineContinuationStateMayBeLive = true;
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
  // An arbitrary caller-owned pipeline can run a mutating stage before its
  // deformable stage, invalidating a public VBD obstacle envelope after the
  // only atomic preflight. Until pipeline capabilities and mutation boundaries
  // are sealed, fail closed before entering simulation or executing any stage.
  DART_SIMULATION_THROW_T_IF(
      requiresPublicDeformableVbdExecution(*this)
          || m_rigidCollisionCapacityOptions.candidatePairCapacity != 0u
          || m_rigidCollisionCapacityOptions.contactCapacity != 0u
          || (m_replay && m_replay->recordingEnabled),
      InvalidOperationException,
      "Caller-owned World step pipelines are not supported with required "
      "public deformable VBD, explicit rigid collision capacities, or while "
      "replay recording is enabled; use the built-in step overload");

  ensureStepPipelineCacheCurrent();
#if DART_BUILD_PROFILE
  StepProfileTimer profileTimer(m_stepProfilingEnabled);
#endif

  stepPipelineOnce(executor, pipeline);
  m_callerOwnedPipelineContinuationStateMayBeLive = true;
  recordReplayFrame();
#if DART_BUILD_PROFILE
  profileTimer.finish(m_lastStepProfile);
#endif
}

//==============================================================================
void World::stepPipelineOnce(
    compute::ComputeExecutor& executor, compute::WorldStepPipeline& pipeline)
{
  // Refresh World-owned baked data and built-in stage caches. This does not
  // prepare arbitrary caller-owned stages: their owner must call prepare() and
  // manage their cache/diagnostic lifecycle before supplying the pipeline.
  ensureStepPipelineCacheCurrent();
  validateLoopClosureKinematicsPolicySupport(*this);
  validateRigidSolverContactMethodCompatibility(
      m_rigidBodySolver, m_contactSolverMethod);
  m_rigidConstraintOptions = validateRigidConstraintOptions(
      m_rigidConstraintOptions, m_rigidBodySolver);
  validateRigidBodyJointPipelineSupport(*this, m_rigidBodySolver);
  validateRigidConstraintOptionsPipelineSupport(
      *this,
      m_rigidConstraintOptions,
      m_rigidBodySolver,
      m_multibodyIntegrationMethod == MultibodyIntegrationMethod::Variational);
  validateLoopClosureDynamicsPolicySupport(
      *this,
      m_multibodyIntegrationMethod == MultibodyIntegrationMethod::Variational);
  validateArticulatedPointJointPipelineSupport(
      *this,
      m_multibodyIntegrationMethod == MultibodyIntegrationMethod::Variational);

  // Run every rejection check before clearing diagnostics, resetting frame
  // scratch, waking/sleeping bodies, capturing derivative state, or executing
  // velocity/contact/position stages. Built-in rigid collision capacity and
  // contact-configuration failures are therefore observationally atomic.
  pipeline.preflight(*this);

  resetFrameScratchForStep();
  m_storage->lastStepDiagnostics = {};
  m_storage->lastContactForces.clear();
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
#if DART_BUILD_MEMORY_DIAGNOSTICS
  m_storage->memoryDiagnostics.recordFrameScratch(m_memoryManager);
#endif
}

//==============================================================================
void World::captureStepDerivatives()
{
  m_storage->stepDerivativesValid = false;

#ifdef DART_HAS_DIFF
  const auto& bakedModel = detail::ensureBakedWorldModelCurrent(*this);
  const std::size_t nonzeroMultibodyCount = countNonzeroMultibodies(bakedModel);

  DART_SIMULATION_THROW_T_IF(
      countRigidBodyDofs(bakedModel) != 0 && nonzeroMultibodyCount != 0,
      NotImplementedException,
      "World::step(): differentiable mixed rigid-body plus multibody worlds "
      "are not supported until full-world step Jacobians are assembled; use a "
      "rigid-only or multibody-only differentiable World");

  DART_SIMULATION_THROW_T_IF(
      nonzeroMultibodyCount > 1,
      NotImplementedException,
      "World::step(): differentiable worlds with multiple nonzero-DOF "
      "multibodies are not supported until full-world step Jacobians are "
      "assembled; use a rigid-only or single-multibody differentiable World");

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
      m_storage->stepDerivativesValid = true;
      return;
    }
  }

  // WS1 covers a single contact-free multibody. The joint-type-keyed position
  // Jacobian in detail::contactFreeStepDerivatives covers fixed, revolute,
  // prismatic, screw, universal, planar, ball (Spherical), and free (Floating)
  // joints. Assemble tau from the joint efforts in construction (DOF) order and
  // compute the analytic Jacobian at the current (pre-step) state.
  if (captureContactFreeStepDerivativesForFirstMultibody()) {
    m_storage->stepDerivativesValid = true;
  }
#endif
}

//==============================================================================
bool World::captureContactFreeStepDerivativesForFirstMultibody()
{
#ifdef DART_HAS_DIFF
  auto view = m_storage->registry.view<comps::MultibodyStructure>();
  for (auto entity : view) {
    const auto& structure = view.get<comps::MultibodyStructure>(entity);

    auto& torques = m_storage->differentiableTorqueScratch;
    torques.clear();
    std::size_t dofCount = 0u;
    for (const auto linkEntity : structure.links) {
      const auto& link = m_storage->registry.get<comps::LinkModel>(linkEntity);
      if (link.parentJoint == entt::null) {
        continue;
      }
      const auto& jointActuation
          = m_storage->registry.get<comps::JointActuation>(link.parentJoint);
      const bool contributesDofs = jointActuation.torque.size() > 0;
      DART_SIMULATION_THROW_T_IF(
          jointActuation.actuatorType == comps::ActuatorType::Locked
              && contributesDofs,
          NotImplementedException,
          "World::step(): differentiable Locked actuators are not supported "
          "until step derivatives apply the locked velocity projection");
      DART_SIMULATION_THROW_T_IF(
          jointActuation.actuatorType == comps::ActuatorType::Acceleration
              && contributesDofs,
          NotImplementedException,
          "World::step(): differentiable Acceleration actuators are not "
          "supported until step derivatives model commanded acceleration");
      dofCount += static_cast<std::size_t>(jointActuation.torque.size());
    }
    torques.reserve(dofCount);
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
    return true; // WS1: one multibody.
  }
#endif

  return false;
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
const std::vector<ContactForce>& World::getLastContactForces() const
{
  return m_storage->lastContactForces;
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

  // Multibodies: link world-frame velocities are derived from the joint degrees
  // of freedom by forward kinematics, so energy and momentum are taken from the
  // variational integrator's helper. It evaluates them on the VarTree's
  // forward-kinematics world transforms and link-frame spatial velocities. This
  // keeps the per-domain split physical -- an earlier version derived potential
  // from the stored comps::LinkState::worldTransform cache, whose gauge did not
  // match the helper (yielding negative kinetic energy at rest).
  const auto structureView = registry.view<comps::MultibodyStructure>();
  for (const auto structureEntity : structureView) {
    const auto& structure
        = structureView.get<comps::MultibodyStructure>(structureEntity);

    const auto terms = compute::computeMultibodyMechanicalEnergyTerms(
        registry, structure, gravity);
    metrics.kineticEnergy += terms.kinetic;
    metrics.potentialEnergy += terms.potential;
    metrics.linearMomentum += terms.linearMomentum;
    metrics.angularMomentum += terms.angularMomentum;
  }

  metrics.activeContactCount
      = m_storage->lastStepDiagnostics.activeContactCount;
  metrics.maxPenetrationDepth
      = m_storage->lastStepDiagnostics.maxPenetrationDepth;
  metrics.lastStepIterations
      = m_storage->lastStepDiagnostics.lastStepIterations;
  metrics.lastStepResidual = m_storage->lastStepDiagnostics.lastStepResidual;

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
void World::validateReplayConstruction() const
{
  if (!m_replay || !m_replay->recordingEnabled) {
    return;
  }

  DART_SIMULATION_THROW_T_IF(
      !m_replay->hasConstructionSnapshot,
      InvalidOperationException,
      "Cannot continue replay: construction snapshot is missing");
  validateReplayRigidBodyLayouts(
      m_storage->registry,
      m_replay->rigidBodyCount,
      m_replay->rigidBodyLayouts);
  validateReplayLinkLayouts(
      m_storage->registry, m_replay->linkCount, m_replay->linkLayouts);
  validateReplayRigidDistanceSpringLayouts(
      m_storage->registry,
      m_replay->rigidDistanceSpringCount,
      m_replay->rigidDistanceSpringLayouts);
  validateReplayMultibodyStructureLayouts(
      m_storage->registry,
      m_replay->multibodyStructureCount,
      m_replay->multibodyStructureLayouts);
  validateReplayDeformableLayouts(
      m_storage->registry,
      m_replay->deformableComponentCounts,
      m_replay->deformableLayouts);
  validateReplayIgnoredCollisionPairs(
      m_storage->ignoredCollisionPairs, m_replay->ignoredCollisionPairs);
  if (!m_replay->frames.empty()) {
    validateReplayCanonicalConstruction(
        m_storage->registry, m_replay->frames.front());
  }
}

//==============================================================================
void World::setReplayRecordingEnabled(bool enabled)
{
  if (enabled) {
    validateRequiredDeformableVbdConfiguration(*this);
  }

  if (enabled == (m_replay && m_replay->recordingEnabled)) {
    return;
  }

  if (!enabled) {
    m_replay->recordingEnabled = false;
    return;
  }

  // Assemble a complete replacement session off to the side. None of the
  // existing replay state becomes observable until the immutable construction
  // snapshot and canonical first frame have both been captured successfully.
  // This keeps replay enable exception-atomic under allocator exhaustion.
  auto candidateReplay = makeReplayState(m_memoryManager);
  {
    auto& replayAllocator = m_memoryManager.getFreeAllocator();
    std::size_t rigidBodyCount = 0;
    auto rigidBodyLayouts
        = captureReplayRigidBodyLayouts<ReplayState::RigidBodyLayout>(
            m_storage->registry, replayAllocator, rigidBodyCount);
    std::size_t linkCount = 0;
    auto linkLayouts = captureReplayLinkLayouts<ReplayState::LinkLayout>(
        m_storage->registry, replayAllocator, linkCount);
    std::size_t rigidDistanceSpringCount = 0;
    auto rigidDistanceSpringLayouts = captureReplayRigidDistanceSpringLayouts<
        ReplayState::RigidDistanceSpringLayout>(
        m_storage->registry, replayAllocator, rigidDistanceSpringCount);
    std::size_t multibodyStructureCount = 0;
    auto multibodyStructureLayouts = captureReplayMultibodyStructureLayouts<
        ReplayState::MultibodyStructureLayout>(
        m_storage->registry, replayAllocator, multibodyStructureCount);
    ReplayState::DeformableComponentCounts componentCounts;
    auto deformableLayouts
        = captureReplayDeformableLayouts<ReplayState::DeformableLayout>(
            m_storage->registry, replayAllocator, componentCounts);

    candidateReplay->rigidBodyCount = rigidBodyCount;
    candidateReplay->rigidBodyLayouts = std::move(rigidBodyLayouts);
    candidateReplay->linkCount = linkCount;
    candidateReplay->linkLayouts = std::move(linkLayouts);
    candidateReplay->rigidDistanceSpringCount = rigidDistanceSpringCount;
    candidateReplay->rigidDistanceSpringLayouts
        = std::move(rigidDistanceSpringLayouts);
    candidateReplay->multibodyStructureCount = multibodyStructureCount;
    candidateReplay->multibodyStructureLayouts
        = std::move(multibodyStructureLayouts);
    candidateReplay->deformableComponentCounts = componentCounts;
    candidateReplay->deformableLayouts = std::move(deformableLayouts);
    candidateReplay->ignoredCollisionPairs.assign(
        m_storage->ignoredCollisionPairs.begin(),
        m_storage->ignoredCollisionPairs.end());
    candidateReplay->hasConstructionSnapshot = true;
    candidateReplay->recordingEnabled = true;
  }

  auto previousReplay = std::move(m_replay);
  m_replay = std::move(candidateReplay);
  try {
    recordReplayFrame();
  } catch (...) {
    m_replay.reset();
    m_replay = std::move(previousReplay);
    throw;
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

  // Preflight before erasing the existing history. recordReplayFrame() repeats
  // the checks defensively, but running them here preserves the old frames and
  // cursor if the current simulation configuration is no longer recordable.
  if (m_replay->recordingEnabled) {
    validateRequiredDeformableVbdConfiguration(*this);
    validateReplayConstruction();
    auto previousFrames = std::move(m_replay->frames);
    const auto previousCursor = m_replay->cursor;
    m_replay->frames = decltype(previousFrames){previousFrames.get_allocator()};
    m_replay->cursor.reset();
    try {
      recordReplayFrame();
    } catch (...) {
      m_replay->frames.clear();
      m_replay->frames = std::move(previousFrames);
      m_replay->cursor = previousCursor;
      throw;
    }
    return;
  }

  m_replay->frames.clear();
  m_replay->cursor.reset();
  m_replay->rigidBodyLayouts.clear();
  m_replay->rigidBodyCount = 0;
  m_replay->linkLayouts.clear();
  m_replay->linkCount = 0;
  m_replay->rigidDistanceSpringLayouts.clear();
  m_replay->rigidDistanceSpringCount = 0;
  m_replay->multibodyStructureLayouts.clear();
  m_replay->multibodyStructureCount = 0;
  m_replay->deformableLayouts.clear();
  m_replay->deformableComponentCounts = {};
  m_replay->ignoredCollisionPairs.clear();
  m_replay->hasConstructionSnapshot = false;
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

  DART_SIMULATION_THROW_T_IF(
      !m_replay->hasConstructionSnapshot,
      InvalidOperationException,
      "Cannot restore replay frame: construction snapshot is missing");
  validateReplayRigidBodyLayouts(
      m_storage->registry,
      m_replay->rigidBodyCount,
      m_replay->rigidBodyLayouts);
  validateReplayLinkLayouts(
      m_storage->registry, m_replay->linkCount, m_replay->linkLayouts);
  validateReplayRigidDistanceSpringLayouts(
      m_storage->registry,
      m_replay->rigidDistanceSpringCount,
      m_replay->rigidDistanceSpringLayouts);
  validateReplayMultibodyStructureLayouts(
      m_storage->registry,
      m_replay->multibodyStructureCount,
      m_replay->multibodyStructureLayouts);
  validateReplayDeformableLayouts(
      m_storage->registry,
      m_replay->deformableComponentCounts,
      m_replay->deformableLayouts);
  validateReplayIgnoredCollisionPairs(
      m_storage->ignoredCollisionPairs, m_replay->ignoredCollisionPairs);
  validateReplayCanonicalConstruction(
      m_storage->registry, m_replay->frames.front());

  const bool wasSimulationMode = m_simulationMode;
  const Eigen::Vector3d previousGravity = m_gravity;
  const auto previousRigidBodySolver = m_rigidBodySolver;
  const auto previousRigidConstraintOptions = m_rigidConstraintOptions;
  const double previousTimeStep = m_timeStep;
  const bool previousDifferentiable = m_differentiable;
  const auto previousMultibodyIntegrationMethod = m_multibodyIntegrationMethod;
  const auto previousContactSolverMethod = m_contactSolverMethod;
  const auto previousContactGradientMode = m_contactGradientMode;
  const auto previousComputeAcceleratorPolicy = m_computeAcceleratorPolicy;
  const auto previousDeactivationOptions = m_deactivationOptions;
  const auto previousDeformablePsdProjector = m_deformablePsdProjector;
  const bool previousDeformablePsdAcceleratedResolved
      = m_deformablePsdAcceleratedResolved;
  const double previousTime = m_time;
  const std::size_t previousFrame = m_frame;
  const auto previousDeformableSolverDiagnostics
      = m_lastDeformableSolverDiagnostics;
  const double previousRigidIpcAdaptiveBarrierStiffnessLowerBound
      = m_rigidIpcAdaptiveBarrierStiffnessLowerBound;
  const auto previousVariationalIntegratorMaxIterations
      = m_variationalIntegratorMaxIterations;
  const auto previousVariationalIntegratorTolerance
      = m_variationalIntegratorTolerance;
  const std::uint64_t previousFrameTopologyRevision = m_frameTopologyRevision;
  const bool previousCallerOwnedPipelineContinuationStateMayBeLive
      = m_callerOwnedPipelineContinuationStateMayBeLive;
  const auto previousLastStepDiagnostics = m_storage->lastStepDiagnostics;
  const bool previousStepDerivativesValid = m_storage->stepDerivativesValid;
  bool deformableVbdConfigurationChanged = false;

  {
    const ReplayState::Frame& replayFrame = m_replay->frames[index];
    validateReplayComponents<comps::DeformableNodeState>(
        m_storage->registry,
        replayFrame.deformableNodeStates,
        "DeformableNodeState");
    deformableVbdConfigurationChanged = validateReplayDeformableVbdConfigs(
        m_storage->registry, replayFrame.deformableNodeStates);
    validateReplayRequiredDeformableVbdConfiguration(
        replayFrame, m_replay->rigidBodyLayouts);
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
    validateReplayTransientComponents<comps::KinematicBodyStepTrace>(
        m_storage->registry,
        replayFrame.kinematicBodyStepTraces,
        "KinematicBodyStepTrace",
        [](const detail::WorldRegistry& registry, entt::entity entity) {
          return registry.all_of<comps::RigidBodyTag, comps::KinematicBodyTag>(
              entity);
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
              m_storage->registry.try_get<
                  detail::deformable_vbd::AvbdRigidWorldPointJointConfig>(
                  state.entity),
              state.layout),
          InvalidOperationException,
          "Cannot restore replay frame: Joint entity layout changed");
      const auto& jointState
          = m_storage->registry.get<comps::JointState>(state.entity);
      const auto& jointActuation
          = m_storage->registry.get<comps::JointActuation>(state.entity);
      DART_SIMULATION_THROW_T_IF(
          static_cast<std::size_t>(jointState.position.size())
                  != state.position.values.size()
              || static_cast<std::size_t>(jointState.velocity.size())
                     != state.velocity.values.size()
              || static_cast<std::size_t>(jointState.acceleration.size())
                     != state.acceleration.values.size()
              || static_cast<std::size_t>(jointActuation.torque.size())
                     != state.torque.values.size()
              || static_cast<std::size_t>(jointActuation.commandVelocity.size())
                     != state.commandVelocity.values.size()
              || static_cast<std::size_t>(
                     jointActuation.commandAcceleration.size())
                     != state.commandAcceleration.values.size(),
          InvalidOperationException,
          "Cannot restore replay frame: Joint runtime vector dimensions "
          "changed");
    }

    DART_SIMULATION_THROW_T_IF(
        countReplayView(m_storage->registry.view<comps::LinkModel>())
            != replayFrame.links.size(),
        InvalidOperationException,
        "Cannot restore replay frame: Link component count changed");
    for (const auto& state : replayFrame.links) {
      DART_SIMULATION_THROW_T_IF(
          !m_storage->registry.valid(state.entity)
              || !m_storage->registry.all_of<comps::LinkModel>(state.entity)
              || !m_storage->registry.all_of<comps::LinkState>(state.entity)
              || !m_storage->registry.all_of<comps::LinkControl>(state.entity),
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
            || !m_storage->registry
                    .all_of<comps::FrameState, comps::FrameCache>(state.entity)
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
                        comps::Force>())
            != replayFrame.rigidBodies.size(),
        InvalidOperationException,
        "Cannot restore replay frame: RigidBody component count changed");
    for (const auto& state : replayFrame.rigidBodies) {
      const bool layoutChanged = !m_storage->registry.valid(state.entity)
                                 || !m_storage->registry.all_of<
                                     comps::RigidBodyTag,
                                     comps::FrameState,
                                     comps::FrameCache,
                                     comps::FreeFrameProperties,
                                     comps::Transform,
                                     comps::Velocity,
                                     comps::Force>(state.entity);
      DART_SIMULATION_THROW_T_IF(
          layoutChanged,
          InvalidOperationException,
          "Cannot restore replay frame: RigidBody entity layout changed");
    }
  }

  const auto originalReplayCursor = m_replay->cursor;
  const std::size_t originalReplayFrameCount = m_replay->frames.size();
  DART_SIMULATION_THROW_T_IF(
      originalReplayFrameCount == std::numeric_limits<std::size_t>::max(),
      InvalidOperationException,
      "Cannot restore replay frame: rollback frame count overflow");

  // Capture the complete live state before any allocator-bearing restore.
  // Appending at the history end avoids truncating a branch. A scope guard
  // removes this temporary frame and restores the public cursor on every exit,
  // including an unexpected exception while handling the original failure.
  m_replay->frames.reserve(originalReplayFrameCount + 1u);
  m_replay->cursor = originalReplayFrameCount - 1u;
  const bool originalReplayRecordingEnabled = m_replay->recordingEnabled;
  // Disabling recording intentionally retains readable history. Capture the
  // private rollback frame through the same complete recorder without making
  // the public restore operation depend on recording still being enabled.
  m_replay->recordingEnabled = true;
  try {
    recordReplayFrame();
  } catch (...) {
    m_replay->recordingEnabled = originalReplayRecordingEnabled;
    m_replay->cursor = originalReplayCursor;
    throw;
  }
  m_replay->recordingEnabled = originalReplayRecordingEnabled;
  const std::size_t rollbackFrameIndex = originalReplayFrameCount;

  struct TemporaryReplayFrameGuard
  {
    ReplayState& replay;
    std::size_t originalFrameCount;
    std::optional<std::size_t> originalCursor;
    std::optional<std::size_t> committedCursor;

    ~TemporaryReplayFrameGuard() noexcept
    {
      if (replay.frames.size() > originalFrameCount) {
        replay.frames.pop_back();
      }
      replay.cursor = committedCursor ? committedCursor : originalCursor;
    }
  } temporaryFrameGuard{
      *m_replay, originalReplayFrameCount, originalReplayCursor, std::nullopt};

  const ReplayState::Frame& replayFrame = m_replay->frames[index];
  ReplayState::Frame& rollbackFrame = m_replay->frames[rollbackFrameIndex];
  const auto rigidAvbdWarmStartIsEmpty = [](const auto& state) noexcept {
    return state.normalRows.empty() && state.frictionRows.empty()
           && state.contactIdentities.empty()
           && state.contactTangentAnchors.empty()
           && state.jointLinearRows.empty() && state.jointAngularRows.empty()
           && state.motorRows.empty() && state.distanceSpringRows.empty();
  };
  const bool frameTopologyWillChange
      = std::ranges::any_of(replayFrame.publicFrames, [&](const auto& state) {
          return m_storage->registry.get<comps::FrameState>(state.entity)
                     .parentFrame
                 != state.frameState.parentFrame;
        });
  const bool rigidCollisionPosesWillChange
      = std::ranges::any_of(
            replayFrame.rigidBodies,
            [&](const auto& state) {
              const auto& current
                  = m_storage->registry.get<comps::Transform>(state.entity);
              return (current.position.array()
                      != state.transform.position.array())
                         .any()
                     || (current.orientation.coeffs().array()
                         != state.transform.orientation.coeffs().array())
                            .any();
            })
        || std::ranges::any_of(replayFrame.links, [&](const auto& state) {
             const auto& current
                 = m_storage->registry.get<comps::LinkState>(state.entity);
             return (current.worldTransform.matrix().array()
                     != state.linkState.worldTransform.matrix().array())
                 .any();
           });
  const auto sameDeactivationOptions = [](const DeactivationOptions& lhs,
                                          const DeactivationOptions& rhs) {
    return lhs.enabled == rhs.enabled
           && lhs.linearSpeedThreshold == rhs.linearSpeedThreshold
           && lhs.angularSpeedThreshold == rhs.angularSpeedThreshold
           && lhs.generalizedSpeedThreshold == rhs.generalizedSpeedThreshold
           && lhs.timeUntilSleep == rhs.timeUntilSleep
           && lhs.wakeThresholdScale == rhs.wakeThresholdScale
           && lhs.disturbanceForceThreshold == rhs.disturbanceForceThreshold;
  };
  // Reuse the already-prepared stage owner only when replay cannot change its
  // configuration, topology, or allocator-backed AVBD continuation state.
  // Besides preserving those live caches transactionally, this keeps the
  // common same-configuration restore on the World allocator: constructing a
  // second kinematics graph would consult default-allocator task-graph
  // internals and violate the baked zero-global-allocation contract.
  const bool reusePreparedStepPipelineCache
      = replayFrame.simulationMode == wasSimulationMode
        && replayFrame.rigidBodySolver == previousRigidBodySolver
        && replayFrame.rigidConstraintIterations
               == previousRigidConstraintOptions.iterations
        && replayFrame.differentiable == previousDifferentiable
        && replayFrame.multibodyIntegrationMethod
               == previousMultibodyIntegrationMethod
        && replayFrame.contactSolverMethod == previousContactSolverMethod
        && replayFrame.contactGradientMode == previousContactGradientMode
        && replayFrame.computeAcceleratorPolicy
               == previousComputeAcceleratorPolicy
        && sameDeactivationOptions(
            replayFrame.deactivationOptions, previousDeactivationOptions)
        && replayFrame.variationalIntegratorMaxIterations
               == previousVariationalIntegratorMaxIterations
        && replayFrame.variationalIntegratorTolerance
               == previousVariationalIntegratorTolerance
        && !deformableVbdConfigurationChanged && !frameTopologyWillChange
        && rigidAvbdWarmStartIsEmpty(replayFrame.rigidAvbdWarmStartState)
        && rigidAvbdWarmStartIsEmpty(rollbackFrame.rigidAvbdWarmStartState)
        && (!wasSimulationMode || m_storage->bakedModel.valid);
  const bool reusePreparedCollisionQueryCache
      = reusePreparedStepPipelineCache && !rigidCollisionPosesWillChange
        && (wasSimulationMode ? m_collisionQueryCache != nullptr
                              : m_collisionQueryCache == nullptr);
  auto& replayAllocator = m_memoryManager.getFreeAllocator();
  ReplayScratchVector<std::size_t> rigidBodyRestoreOrder(
      ReplayScratchAllocator<std::size_t>{replayAllocator});
  ReplayScratchVector<std::size_t> rollbackRigidBodyRestoreOrder(
      ReplayScratchAllocator<std::size_t>{replayAllocator});
  using FrameCacheSnapshot = std::pair<entt::entity, comps::FrameCache>;
  ReplayScratchVector<FrameCacheSnapshot> previousFrameCaches(
      ReplayScratchAllocator<FrameCacheSnapshot>{replayAllocator});

  detail::BakedWorldModel previousBakedModel(replayAllocator);
  compute::ResolvedSolverConfiguration previousResolvedConfiguration;
  std::optional<StepDerivatives> previousStepDerivatives;
  std::vector<ContactForce> previousLastContactForces;
  decltype(m_storage
               ->differentiableParameters) previousDifferentiableParameters{
      m_storage->differentiableParameters.get_allocator()};
  std::optional<CollisionQueryCachePtr> previousCollisionQueryCache;
  std::optional<StepPipelineCachePtr> previousStepPipelineCache;
  bool rollbackPayloadStashed = false;
  bool configurationCachePayloadStashed = false;

  try {
    rigidBodyRestoreOrder = orderReplayRigidBodiesParentBeforeChild(
        m_storage->registry,
        replayFrame.rigidBodies,
        replayFrame.publicFrames,
        replayAllocator);
    rollbackRigidBodyRestoreOrder = orderReplayRigidBodiesParentBeforeChild(
        m_storage->registry,
        rollbackFrame.rigidBodies,
        rollbackFrame.publicFrames,
        replayAllocator);
    const auto frameCacheView = m_storage->registry.view<comps::FrameCache>();
    previousFrameCaches.reserve(countReplayView(frameCacheView));
    for (const entt::entity entity : frameCacheView) {
      previousFrameCaches.emplace_back(
          entity, frameCacheView.get<comps::FrameCache>(entity));
    }

    if (!reusePreparedStepPipelineCache) {
      std::swap(previousBakedModel, m_storage->bakedModel);
      previousResolvedConfiguration.notes.swap(m_resolvedConfiguration.notes);
      configurationCachePayloadStashed = true;
    }
    previousStepDerivatives.swap(m_storage->stepDerivatives);
    previousLastContactForces.swap(m_storage->lastContactForces);
    previousDifferentiableParameters.swap(m_storage->differentiableParameters);
    rollbackPayloadStashed = true;

    compute::avbd_replay::restoreDeformableAvbdWarmStartReplayState(
        m_storage->registry,
        replayFrame.deformableAvbdWarmStartStates,
        replayAllocator);

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
            state = &m_storage->registry
                         .emplace<compute::MultibodyVariationalState>(
                             entity,
                             makeReplayMultibodyVariationalState(
                                 replayAllocator));
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
    restoreReplayTransientComponents<comps::KinematicBodyStepTrace>(
        m_storage->registry,
        replayFrame.kinematicBodyStepTraces,
        "KinematicBodyStepTrace",
        replayAllocator,
        [](const detail::WorldRegistry& registry, entt::entity entity) {
          return registry.all_of<comps::RigidBodyTag, comps::KinematicBodyTag>(
              entity);
        });

    for (const auto& state : replayFrame.joints) {
      auto& jointState
          = m_storage->registry.get<comps::JointState>(state.entity);
      auto& jointActuation
          = m_storage->registry.get<comps::JointActuation>(state.entity);
      restoreReplayVector(state.position, jointState.position);
      restoreReplayVector(state.velocity, jointState.velocity);
      restoreReplayVector(state.acceleration, jointState.acceleration);
      restoreReplayVector(state.torque, jointActuation.torque);
      restoreReplayVector(
          state.commandVelocity, jointActuation.commandVelocity);
      restoreReplayVector(
          state.commandAcceleration, jointActuation.commandAcceleration);
      jointState.broken = state.broken;
    }

    for (const auto& state : replayFrame.links) {
      m_storage->registry.get<comps::LinkState>(state.entity) = state.linkState;
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
      m_storage->registry.replace<comps::FreeFrameProperties>(
          state.entity, state.freeFrameProperties);
      m_storage->registry.replace<comps::Transform>(
          state.entity, state.transform);
      m_storage->registry.replace<comps::Velocity>(
          state.entity, state.velocity);
      m_storage->registry.replace<comps::Force>(state.entity, state.force);
    }

    if (!reusePreparedStepPipelineCache) {
      // Build a fresh stage owner before replacing the live cache. The
      // previous cache contains allocator-backed continuation state and
      // remains untouched until construction succeeds, so rollback can
      // restore it with one move.
      auto replacementStepPipelineCache = makeStepPipelineCache(
          m_memoryManager, replayFrame.rigidConstraintIterations);
      previousStepPipelineCache.emplace(std::move(m_stepPipelineCache));
      m_stepPipelineCache = std::move(replacementStepPipelineCache);
    }

    m_simulationMode = replayFrame.simulationMode;
    m_gravity = replayFrame.gravity;
    m_rigidBodySolver = replayFrame.rigidBodySolver;
    m_rigidConstraintOptions = validateRigidConstraintOptions(
        RigidConstraintOptions{
            .iterations = replayFrame.rigidConstraintIterations,
        },
        m_rigidBodySolver);
    m_stepPipelineCache->stages.rigidBodyContact.setIterations(
        m_rigidConstraintOptions.iterations);
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
    m_variationalIntegratorTolerance
        = replayFrame.variationalIntegratorTolerance;
    m_storage->stepDerivatives = replayFrame.stepDerivatives;
    m_storage->stepDerivativesValid = replayFrame.stepDerivativesValid;
    m_storage->lastStepDiagnostics = replayFrame.lastStepDiagnostics;
    m_storage->lastContactForces.assign(
        replayFrame.lastContactForces.begin(),
        replayFrame.lastContactForces.end());
    m_storage->differentiableParameters.assign(
        replayFrame.differentiableParameters.begin(),
        replayFrame.differentiableParameters.end());

    if (!reusePreparedCollisionQueryCache) {
      previousCollisionQueryCache.emplace(std::move(m_collisionQueryCache));
    }
    markFrameCachesDirty(m_storage->registry);
    if (m_simulationMode) {
      updateKinematics();
      const bool stepPipelinePolicyChanged
          = !wasSimulationMode || frameTopologyChanged
            || previousRigidBodySolver != m_rigidBodySolver
            || previousRigidConstraintOptions.iterations
                   != m_rigidConstraintOptions.iterations
            || previousMultibodyIntegrationMethod
                   != m_multibodyIntegrationMethod
            || previousContactSolverMethod != m_contactSolverMethod
            || previousComputeAcceleratorPolicy != m_computeAcceleratorPolicy
            || deformableVbdConfigurationChanged
            || previousStepPipelineCache.has_value();
      if (stepPipelinePolicyChanged) {
        prepareStepPipelineCacheForCurrentConfiguration();
      }
      // Replay restoration deliberately discards the pose-dependent native
      // collision cache above. Re-bake it even when the solver schedule did
      // not change so automatic capacities are immediately resolved again and
      // the restored simulation frame cannot silently reopen their lock.
      if (!reusePreparedCollisionQueryCache) {
        static_cast<void>(queryContacts(
            CollisionQueryOptions{},
            /*includeShapeContactDetails=*/false));
      }
      m_collisionQueryCache->capacitiesLocked = true;
    } else {
      // Resolution is defined only for a finalized simulation configuration.
      // Restoring the design-mode recording boundary must not retain the solver
      // identity of a later simulated frame.
      m_resolvedConfiguration.reset();
    }
    if (!reusePreparedStepPipelineCache) {
      m_stepPipelineCache->stages.rigidBodyContact
          .restoreAvbdWarmStartReplayState(replayFrame.rigidAvbdWarmStartState);
    }

    temporaryFrameGuard.committedCursor = index;
  } catch (...) {
    const std::exception_ptr restoreFailure = std::current_exception();
    if (rollbackPayloadStashed) {
      compute::avbd_replay::restoreDeformableAvbdWarmStartReplayStateNoAlloc(
          m_storage->registry, rollbackFrame.deformableAvbdWarmStartStates);
      restoreReplayDeformableNodeStatesNoAlloc(
          m_storage->registry, rollbackFrame.deformableNodeStates);
      restoreReplayTransientComponentsNoAlloc<
          compute::MultibodyVariationalState>(
          m_storage->registry, rollbackFrame.multibodyVariationalStates);
      restoreReplayTransientComponentsNoAlloc<
          comps::VariationalContactDualState>(
          m_storage->registry, rollbackFrame.variationalContactDualStates);
      restoreReplayTransientComponentsNoAlloc<comps::DeactivationState>(
          m_storage->registry, rollbackFrame.deactivationStates);
      restoreReplayTransientComponentsNoAlloc<comps::KinematicBodyStepTrace>(
          m_storage->registry, rollbackFrame.kinematicBodyStepTraces);

      const auto restoreJointVectorNoAlloc
          = [](const auto& source, auto& target) noexcept {
              for (Eigen::Index i = 0; i < target.size(); ++i) {
                target[i] = source.values[static_cast<std::size_t>(i)];
              }
            };
      for (const auto& state : rollbackFrame.joints) {
        auto& jointState
            = m_storage->registry.get<comps::JointState>(state.entity);
        auto& jointActuation
            = m_storage->registry.get<comps::JointActuation>(state.entity);
        restoreJointVectorNoAlloc(state.position, jointState.position);
        restoreJointVectorNoAlloc(state.velocity, jointState.velocity);
        restoreJointVectorNoAlloc(state.acceleration, jointState.acceleration);
        restoreJointVectorNoAlloc(state.torque, jointActuation.torque);
        restoreJointVectorNoAlloc(
            state.commandVelocity, jointActuation.commandVelocity);
        restoreJointVectorNoAlloc(
            state.commandAcceleration, jointActuation.commandAcceleration);
        jointState.broken = state.broken;
      }

      for (const auto& state : rollbackFrame.links) {
        m_storage->registry.get<comps::LinkState>(state.entity)
            = state.linkState;
        m_storage->registry.get<comps::LinkControl>(state.entity).externalForce
            = state.externalForce;
      }

      for (const auto& state : rollbackFrame.publicFrames) {
        m_storage->registry.get<comps::FrameState>(state.entity)
            = state.frameState;
        if (state.freeFrameProperties) {
          m_storage->registry.get<comps::FreeFrameProperties>(state.entity)
              = *state.freeFrameProperties;
        }
        if (state.fixedFrameProperties) {
          m_storage->registry.get<comps::FixedFrameProperties>(state.entity)
              = *state.fixedFrameProperties;
        }
      }
      for (const auto stateIndex : rollbackRigidBodyRestoreOrder) {
        const auto& state = rollbackFrame.rigidBodies[stateIndex];
        m_storage->registry.get<comps::FreeFrameProperties>(state.entity)
            = state.freeFrameProperties;
        m_storage->registry.get<comps::Transform>(state.entity)
            = state.transform;
        m_storage->registry.get<comps::Velocity>(state.entity) = state.velocity;
        m_storage->registry.get<comps::Force>(state.entity) = state.force;
      }
      for (const auto& [entity, cache] : previousFrameCaches) {
        m_storage->registry.get<comps::FrameCache>(entity) = cache;
      }

      m_simulationMode = wasSimulationMode;
      m_gravity = previousGravity;
      m_rigidBodySolver = previousRigidBodySolver;
      m_rigidConstraintOptions = previousRigidConstraintOptions;
      m_timeStep = previousTimeStep;
      m_differentiable = previousDifferentiable;
      m_contactSolverMethod = previousContactSolverMethod;
      m_contactGradientMode = previousContactGradientMode;
      m_computeAcceleratorPolicy = previousComputeAcceleratorPolicy;
      m_deactivationOptions = previousDeactivationOptions;
      m_deformablePsdProjector = previousDeformablePsdProjector;
      m_deformablePsdAcceleratedResolved
          = previousDeformablePsdAcceleratedResolved;
      m_time = previousTime;
      m_frame = previousFrame;
      m_lastDeformableSolverDiagnostics = previousDeformableSolverDiagnostics;
      m_rigidIpcAdaptiveBarrierStiffnessLowerBound
          = previousRigidIpcAdaptiveBarrierStiffnessLowerBound;
      m_multibodyIntegrationMethod = previousMultibodyIntegrationMethod;
      m_variationalIntegratorMaxIterations
          = previousVariationalIntegratorMaxIterations;
      m_variationalIntegratorTolerance = previousVariationalIntegratorTolerance;
      m_frameTopologyRevision = previousFrameTopologyRevision;
      m_callerOwnedPipelineContinuationStateMayBeLive
          = previousCallerOwnedPipelineContinuationStateMayBeLive;
      m_storage->lastStepDiagnostics = previousLastStepDiagnostics;
      m_storage->stepDerivativesValid = previousStepDerivativesValid;

      if (configurationCachePayloadStashed) {
        std::swap(previousBakedModel, m_storage->bakedModel);
        previousResolvedConfiguration.notes.swap(m_resolvedConfiguration.notes);
      }
      previousStepDerivatives.swap(m_storage->stepDerivatives);
      previousLastContactForces.swap(m_storage->lastContactForces);
      previousDifferentiableParameters.swap(
          m_storage->differentiableParameters);

      if (previousCollisionQueryCache) {
        m_collisionQueryCache.reset();
        m_collisionQueryCache = std::move(*previousCollisionQueryCache);
      }
      if (previousStepPipelineCache) {
        m_stepPipelineCache.reset();
        m_stepPipelineCache = std::move(*previousStepPipelineCache);
      }
    }
    std::rethrow_exception(restoreFailure);
  }
}

//==============================================================================
void World::recordReplayFrame()
{
  if (!m_replay || !m_replay->recordingEnabled) {
    return;
  }

  validateReplayConstruction();
  validateRequiredDeformableVbdConfiguration(*this);

  const std::size_t retainedFrameCount
      = m_replay->cursor
            ? std::min(*m_replay->cursor + 1u, m_replay->frames.size())
            : m_replay->frames.size();
  // Reserve before constructing the frame or erasing a branched history tail.
  // Allocation failure therefore leaves the complete prior history intact.
  m_replay->frames.reserve(retainedFrameCount + 1u);

  auto& replayAllocator = m_memoryManager.getFreeAllocator();
  ReplayState::Frame replayFrame(replayAllocator);
  replayFrame.simulationMode = m_simulationMode;
  replayFrame.gravity = m_gravity;
  replayFrame.rigidBodySolver = m_rigidBodySolver;
  replayFrame.rigidConstraintIterations = m_rigidConstraintOptions.iterations;
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
  replayFrame.stepDerivativesValid = m_storage->stepDerivativesValid;
  replayFrame.lastStepDiagnostics = m_storage->lastStepDiagnostics;
  replayFrame.lastContactForces.assign(
      m_storage->lastContactForces.begin(), m_storage->lastContactForces.end());
  replayFrame.differentiableParameters.assign(
      m_storage->differentiableParameters.begin(),
      m_storage->differentiableParameters.end());

  replayFrame.deformableNodeStates = captureReplayDeformableNodeStates<
      ReplayState::DeformableNodeStateSnapshot>(
      m_storage->registry, replayAllocator);
  replayFrame.deformableAvbdWarmStartStates
      = compute::avbd_replay::captureDeformableAvbdWarmStartReplayState(
          m_storage->registry, replayAllocator);
  replayFrame.rigidAvbdWarmStartState
      = m_stepPipelineCache->stages.rigidBodyContact
            .captureAvbdWarmStartReplayState(replayAllocator);
  replayFrame.multibodyVariationalStates
      = captureReplayComponents<compute::MultibodyVariationalState>(
          m_storage->registry, replayAllocator);
  replayFrame.variationalContactDualStates
      = captureReplayComponents<comps::VariationalContactDualState>(
          m_storage->registry, replayAllocator);
  replayFrame.deactivationStates
      = captureReplayComponents<comps::DeactivationState>(
          m_storage->registry, replayAllocator);
  replayFrame.kinematicBodyStepTraces
      = captureReplayComponents<comps::KinematicBodyStepTrace>(
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
    if (const auto* pointJointConfig
        = m_storage->registry
              .try_get<detail::deformable_vbd::AvbdRigidWorldPointJointConfig>(
                  entity)) {
      state.layout.hasAvbdPointJointConfig = true;
      state.layout.avbdPointJointConfig = *pointJointConfig;
    } else {
      state.layout.hasAvbdPointJointConfig = false;
      state.layout.avbdPointJointConfig
          = detail::deformable_vbd::AvbdRigidWorldPointJointConfig{};
    }
    captureReplayJointLimits(joint.limits, state.layout.limits);
    state.layout.axis = joint.axis;
    state.layout.axis2 = joint.axis2;
    state.layout.pitch = joint.pitch;
    state.layout.parentLink = joint.parentLink;
    state.layout.childLink = joint.childLink;
    state.layout.hasRigidBodyPairConstraintGeometry
        = joint.hasRigidBodyPairConstraintGeometry;
    state.layout.rigidBodyPairConstraintLocalAnchorParent
        = joint.rigidBodyPairConstraintLocalAnchorParent;
    state.layout.rigidBodyPairConstraintLocalAnchorChild
        = joint.rigidBodyPairConstraintLocalAnchorChild;
    state.layout.rigidBodyPairConstraintTargetRelativeOrientation
        = joint.rigidBodyPairConstraintTargetRelativeOrientation;
    captureReplayVector(jointState.position, state.position);
    captureReplayVector(jointState.velocity, state.velocity);
    captureReplayVector(jointState.acceleration, state.acceleration);
    captureReplayVector(jointActuation.torque, state.torque);
    captureReplayVector(jointActuation.commandVelocity, state.commandVelocity);
    captureReplayVector(
        jointActuation.commandAcceleration, state.commandAcceleration);
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
    const auto& linkState = m_storage->registry.get<comps::LinkState>(entity);
    const auto& linkControl
        = m_storage->registry.get<comps::LinkControl>(entity);
    replayFrame.links.push_back(
        ReplayState::LinkState{entity, linkState, linkControl.externalForce});
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
      comps::Force>();
  replayFrame.rigidBodies.reserve(countReplayView(rigidBodyView));
  for (auto entity : rigidBodyView) {
    replayFrame.rigidBodies.push_back(
        ReplayState::RigidBodyState{
            entity,
            rigidBodyView.get<comps::Transform>(entity),
            rigidBodyView.get<comps::Velocity>(entity),
            rigidBodyView.get<comps::Force>(entity),
            m_storage->registry.get<comps::FreeFrameProperties>(entity)});
  }
  std::ranges::sort(
      replayFrame.rigidBodies, [](const auto& lhs, const auto& rhs) {
        return static_cast<std::uint32_t>(lhs.entity)
               < static_cast<std::uint32_t>(rhs.entity);
      });

  // ReplayState::Frame owns allocator-backed vectors and moves without
  // allocation. Once capture has succeeded and capacity is reserved, tail
  // erasure plus insertion is the non-throwing commit point.
  static_assert(std::is_nothrow_move_constructible_v<ReplayState::Frame>);
  if (retainedFrameCount < m_replay->frames.size()) {
    m_replay->frames.erase(
        m_replay->frames.begin()
            + static_cast<std::ptrdiff_t>(retainedFrameCount),
        m_replay->frames.end());
  }
  m_replay->frames.push_back(std::move(replayFrame));
  m_replay->cursor = retainedFrameCount;
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
      if (!isValidNativeCollisionShape(shape)) {
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

  // Per-pair contact budget of the automatic policy. Primitive pairs emit one
  // four-point manifold; mesh pairs may emit several manifolds and are
  // budgeted at this value by the automatic envelope and the probe reserve.
  constexpr std::size_t kMaxRigidContactsPerPair = 1000u;
  // Automatic-policy storage budgets (entries). Explicit limits reserve
  // exactly; automatic envelopes reserve at most these and grow beyond them.
  constexpr std::size_t kAutomaticRigidCandidatePairReserveBudget = 65536u;
  constexpr std::size_t kAutomaticRigidContactReserveBudget = 16384u;
  const auto checkedContactOverflowProbeCapacity = [](std::size_t capacity) {
    DART_SIMULATION_THROW_T_IF(
        capacity == std::numeric_limits<std::size_t>::max(),
        InvalidOperationException,
        "Rigid collision contact capacity is too large to reserve its "
        "overflow probe");
    return capacity + 1u;
  };

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

  if (!cache.capacitiesPrepared || (rebuildCache && !cache.capacitiesLocked)) {
    const auto completePairBound = [](std::size_t shapeCount) {
      if (shapeCount < 2u) {
        return std::size_t{0u};
      }
      std::size_t first = shapeCount;
      std::size_t second = shapeCount - 1u;
      if (first % 2u == 0u) {
        first /= 2u;
      } else {
        second /= 2u;
      }
      DART_SIMULATION_THROW_T_IF(
          second != 0u
              && first > std::numeric_limits<std::size_t>::max() / second,
          InvalidOperationException,
          "Rigid collision shape count is too large to represent its complete "
          "candidate-pair bound");
      return first * second;
    };
    const std::size_t automaticCandidatePairCapacity
        = completePairBound(specs.size());
    const std::size_t primitiveShapeCount = static_cast<std::size_t>(
        std::count_if(specs.begin(), specs.end(), [](const auto& spec) {
          return spec.shape->type != CollisionShapeType::Mesh;
        }));
    const std::size_t primitivePairCount
        = completePairBound(primitiveShapeCount);
    const std::size_t meshInvolvingPairCount
        = automaticCandidatePairCapacity - primitivePairCount;
    const auto checkedContactContribution = [](std::size_t pairCount,
                                               std::size_t contactsPerPair) {
      DART_SIMULATION_THROW_T_IF(
          contactsPerPair != 0u
              && pairCount > std::numeric_limits<std::size_t>::max()
                                 / contactsPerPair,
          InvalidOperationException,
          "Rigid collision shapes require a contact capacity that cannot be "
          "represented by size_t");
      return pairCount * contactsPerPair;
    };
    // Every built-in primitive pair emits at most one four-point manifold.
    // Mesh paths can emit multiple manifolds, so the automatic policy assigns
    // the conservative per-pair contact budget to every mesh-involving pair.
    // The per-pair budget-plus-one query below rejects rather than truncates
    // if any pair exceeds that budget.
    const std::size_t primitiveContactCapacity = checkedContactContribution(
        primitivePairCount, ncol::ContactManifold::kMaxContacts);
    const std::size_t meshContactCapacity = checkedContactContribution(
        meshInvolvingPairCount, kMaxRigidContactsPerPair);
    DART_SIMULATION_THROW_T_IF(
        primitiveContactCapacity
            > std::numeric_limits<std::size_t>::max() - meshContactCapacity,
        InvalidOperationException,
        "Rigid collision shapes require a contact capacity that cannot be "
        "represented by size_t");
    const std::size_t automaticContactCapacity
        = primitiveContactCapacity + meshContactCapacity;

    cache.candidatePairCapacity
        = m_rigidCollisionCapacityOptions.candidatePairCapacity != 0u
              ? m_rigidCollisionCapacityOptions.candidatePairCapacity
              : automaticCandidatePairCapacity;
    cache.contactCapacity
        = m_rigidCollisionCapacityOptions.contactCapacity != 0u
              ? m_rigidCollisionCapacityOptions.contactCapacity
              : automaticContactCapacity;
    DART_SIMULATION_THROW_T_IF(
        cache.contactCapacity > std::numeric_limits<std::size_t>::max() / 2u,
        InvalidOperationException,
        "Resolved rigid collision contact capacity is too large to represent "
        "the two rigid-contact tangent rows required per contact");
    // Reservation policy. Explicit limits reserve exactly, so baked steps stay
    // allocation-free and fail closed at the same number. Automatic envelopes
    // are rejection thresholds only: the complete shape-pair bound is quadratic
    // in the shape count (100 mesh shapes resolve to 4,950,000 contacts), so
    // the automatic policy reserves at most a fixed budget and lets the
    // buffers grow, allocating, between the budget and the envelope.
    cache.candidatePairReserve
        = m_rigidCollisionCapacityOptions.candidatePairCapacity != 0u
              ? cache.candidatePairCapacity
              : std::min(
                    cache.candidatePairCapacity,
                    kAutomaticRigidCandidatePairReserveBudget);
    cache.contactReserve
        = m_rigidCollisionCapacityOptions.contactCapacity != 0u
              ? cache.contactCapacity
              : std::min(
                    cache.contactCapacity, kAutomaticRigidContactReserveBudget);
    cache.candidatePairs.pairs.reserve(cache.candidatePairReserve);
    cache.collisionWorld.reserveBroadPhasePairCapacity(
        cache.candidatePairReserve);
    cache.contacts.reserve(cache.contactReserve);
    // Every per-pair query is capped at the aggregate remainder plus one
    // overflow witness, so an exact explicit cap stays exact. The probe
    // buffer only needs storage for the per-pair budget plus that witness:
    // sizing it from the aggregate cap allocated one manifold slot per
    // aggregate contact, and a pair that legitimately exceeds the per-pair
    // budget under a larger explicit cap simply grows the buffer.
    cache.pairResult.reserveContacts(
        std::min(
            checkedContactOverflowProbeCapacity(cache.contactCapacity),
            kMaxRigidContactsPerPair + 1u));
    cache.capacitiesPrepared = true;
  }

  if (specs.empty()) {
    cache.candidatePairs.clear();
    cache.contacts.clear();
    return std::span<const Contact>{
        cache.contacts.data(), cache.contacts.size()};
  }

  // Broad-phase-pruned narrow-phase queries. Each candidate pair's bodies are
  // known here, so the contacts map back to the right simulation bodies without
  // relying on the result carrying object identity.
  const bool completeCandidateSnapshot
      = cache.collisionWorld.buildBroadPhaseSnapshotBounded(
          cache.candidatePairs, cache.candidatePairCapacity);
  DART_SIMULATION_THROW_T_IF(
      !completeCandidateSnapshot,
      InvalidOperationException,
      "Rigid collision candidate-pair capacity {} exceeded; increase "
      "WorldOptions::rigidCollisionCapacityOptions.candidatePairCapacity or "
      "select zero for the automatic complete-pair bound",
      cache.candidatePairCapacity);
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
    const std::size_t remainingContactCapacity
        = cache.contactCapacity - contacts.size();
    const std::size_t contactProbeCapacity
        = checkedContactOverflowProbeCapacity(remainingContactCapacity);
    const auto option
        = ncol::CollisionOption::fullContacts(contactProbeCapacity);
    if (!cache.collisionWorld.collide(
            cache.entries[i].object, cache.entries[j].object, option, result)) {
      continue;
    }

    DART_SIMULATION_THROW_T_IF(
        result.numContacts() > remainingContactCapacity,
        InvalidOperationException,
        "Rigid collision contact capacity {} exceeded; increase "
        "WorldOptions::rigidCollisionCapacityOptions.contactCapacity or "
        "select zero for the automatic shape-pair bound",
        cache.contactCapacity);

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

namespace {

template <typename T>
void readRequiredWorldMetadataPOD(
    std::istream& input, T& value, std::string_view field)
{
  io::readPOD(input, value);
  DART_SIMULATION_THROW_T_IF(
      !input,
      InvalidArgumentException,
      "Serialized World metadata is truncated while reading {}",
      field);
}

} // namespace

//==============================================================================
void World::saveBinary(std::ostream& output) const
{
  const auto& rigidStage = m_stepPipelineCache->stages.rigidBodyContact;
  const bool hasRigidAvbdContinuationState
      = rigidStage.hasAnyAvbdWarmStartContinuationState();
  const bool hasDeformableAvbdContinuationState
      = compute::avbd_replay::hasDeformableAvbdWarmStartContinuationState(
          m_storage->registry);
  DART_SIMULATION_THROW_T_IF(
      m_callerOwnedPipelineContinuationStateMayBeLive
          || hasRigidAvbdContinuationState
          || hasDeformableAvbdContinuationState,
      InvalidOperationException,
      "Cannot save World binary snapshot with continuation state that the "
      "binary format does not encode (live AVBD warm-start inventories or "
      "opaque caller-owned pipeline state); use replay for exact built-in "
      "continuation, save before those paths execute, or clear/load the "
      "World");

  // Reject invalid public-VBD snapshots before writing even the format header.
  // Other serializer failures can still occur after output has begun; this is
  // intentionally not a general transactional-stream guarantee.
  validateRequiredDeformableVbdConfiguration(*this);

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

  using DifferentiableParameter = detail::WorldStorage::DifferentiableParameter;
  std::vector<
      DifferentiableParameter,
      detail::WorldStorage::DifferentiableParameterAllocator>
      savedDifferentiableParameters{
          m_storage->differentiableParameters.get_allocator()};
  savedDifferentiableParameters.reserve(
      m_storage->differentiableParameters.size());
  for (const auto& parameter : m_storage->differentiableParameters) {
    const auto entity = parameter.first;
    DART_SIMULATION_THROW_T_IF(
        !entityMap.contains(entity),
        InvalidOperationException,
        "World::saveBinary(): differentiable parameter references an entity "
        "that is not serialized");
    savedDifferentiableParameters.push_back(parameter);
  }

  io::writePOD(output, savedDifferentiableParameters.size());
  for (const auto& [entity, parameter] : savedDifferentiableParameters) {
    io::writePOD(output, static_cast<std::uint32_t>(entityMap.at(entity)));
    io::writePOD(output, encodeDifferentiablePhysicalParameter(parameter));
  }

  io::writePOD(output, m_rigidConstraintOptions.iterations);
  io::writePOD(output, m_rigidIpcAdaptiveBarrierStiffnessLowerBound);
}

//==============================================================================
void World::loadBinary(std::istream& input)
{
  // Keep the complete pre-load World alive until parsing, validation,
  // kinematics, and built-in stage preparation all succeed. This uses the
  // same MemoryManager for the staged storage, so every component retains the
  // World's allocator discipline and a malformed snapshot cannot invalidate
  // existing handles or erase replay history.
  auto previousStorage = std::move(m_storage);
  auto previousCollisionQueryCache = std::move(m_collisionQueryCache);
  auto previousStepPipelineCache = std::move(m_stepPipelineCache);
  auto previousReplay = std::move(m_replay);
  const bool previousSimulationMode = m_simulationMode;
  const Eigen::Vector3d previousGravity = m_gravity;
  const RigidBodySolver previousRigidBodySolver = m_rigidBodySolver;
  const RigidConstraintOptions previousRigidConstraintOptions
      = m_rigidConstraintOptions;
  const double previousTimeStep = m_timeStep;
  const bool previousDifferentiable = m_differentiable;
  const ContactSolverMethod previousContactSolverMethod = m_contactSolverMethod;
  const ContactGradientMode previousContactGradientMode = m_contactGradientMode;
  const ComputeAcceleratorPolicy previousComputeAcceleratorPolicy
      = m_computeAcceleratorPolicy;
  const bool previousStrictSolverResolution = m_strictSolverResolution;
  const DeactivationOptions previousDeactivationOptions = m_deactivationOptions;
  const DeformablePsdProjector previousDeformablePsdProjector
      = m_deformablePsdProjector;
  const bool previousDeformablePsdAcceleratedResolved
      = m_deformablePsdAcceleratedResolved;
  const double previousTime = m_time;
  const DeformableSolverDiagnostics previousDeformableSolverDiagnostics
      = m_lastDeformableSolverDiagnostics;
#if DART_BUILD_PROFILE
  const bool previousStepProfilingEnabled = m_stepProfilingEnabled;
  auto previousLastStepProfile = std::move(m_lastStepProfile);
  auto previousStepProfileScratch = std::move(m_stepProfileScratch);
#endif
  auto previousResolvedConfiguration = std::move(m_resolvedConfiguration);
  const double previousRigidIpcAdaptiveBarrierStiffnessLowerBound
      = m_rigidIpcAdaptiveBarrierStiffnessLowerBound;
  const std::uint64_t previousFrameTopologyRevision = m_frameTopologyRevision;
  const MultibodyIntegrationMethod previousMultibodyIntegrationMethod
      = m_multibodyIntegrationMethod;
  const std::size_t previousVariationalIntegratorMaxIterations
      = m_variationalIntegratorMaxIterations;
  const double previousVariationalIntegratorTolerance
      = m_variationalIntegratorTolerance;
  const std::size_t previousFrame = m_frame;
  const std::size_t previousFreeFrameCounter = m_freeFrameCounter;
  const std::size_t previousFixedFrameCounter = m_fixedFrameCounter;
  const std::size_t previousMultibodyCounter = m_multibodyCounter;
  const std::size_t previousLoopClosureCounter = m_loopClosureCounter;
  const std::size_t previousRigidBodyCounter = m_rigidBodyCounter;
  const std::size_t previousDeformableBodyCounter = m_deformableBodyCounter;
  const std::size_t previousLinkCounter = m_linkCounter;
  const std::size_t previousJointCounter = m_jointCounter;
  const bool previousCallerOwnedPipelineContinuationStateMayBeLive
      = m_callerOwnedPipelineContinuationStateMayBeLive;

  try {
    clear();

    const auto formatVersion = io::readFormatHeader(input);

    io::EntityMap entityMap;
    io::SerializerRegistry::instance().loadAllEntities(
        input, m_storage->registry, entityMap, formatVersion);
    rebindLoadedWorldComponentAllocators(
        m_storage->registry, getMemoryManager().getFreeAllocator());

    // World metadata was historically optional through format 14. Format 15
    // introduced a declared World-option tail; from that version onward an
    // entity table without its complete versioned metadata is malformed.
    const bool hasWorldMetadata = input.peek() != std::char_traits<char>::eof();
    DART_SIMULATION_THROW_T_IF(
        !hasWorldMetadata && formatVersion >= 15,
        InvalidArgumentException,
        "Serialized World metadata is missing for binary format version {}",
        formatVersion);
    if (hasWorldMetadata) {
      std::uint8_t simulationFlag = 0;
      readRequiredWorldMetadataPOD(
          input, simulationFlag, "simulation-mode flag");
      DART_SIMULATION_THROW_T_IF(
          simulationFlag > 1u,
          InvalidArgumentException,
          "Serialized World simulation-mode flag must be 0 or 1");
      m_simulationMode = simulationFlag == 1u;

      readRequiredWorldMetadataPOD(
          input, m_freeFrameCounter, "free-frame counter");
      readRequiredWorldMetadataPOD(
          input, m_fixedFrameCounter, "fixed-frame counter");
      readRequiredWorldMetadataPOD(
          input, m_multibodyCounter, "multibody counter");
      readRequiredWorldMetadataPOD(
          input, m_rigidBodyCounter, "rigid-body counter");
      readRequiredWorldMetadataPOD(input, m_linkCounter, "link counter");
      readRequiredWorldMetadataPOD(input, m_jointCounter, "joint counter");

      double timeStep = 0.0;
      double time = 0.0;
      readRequiredWorldMetadataPOD(input, timeStep, "time step");
      readRequiredWorldMetadataPOD(input, time, "time");
      readRequiredWorldMetadataPOD(input, m_frame, "frame counter");
      setTimeStep(timeStep);
      setTime(time);

      if (formatVersion >= 2) {
        double gravityX = 0.0;
        double gravityY = 0.0;
        double gravityZ = 0.0;
        readRequiredWorldMetadataPOD(input, gravityX, "gravity x");
        readRequiredWorldMetadataPOD(input, gravityY, "gravity y");
        readRequiredWorldMetadataPOD(input, gravityZ, "gravity z");
        setGravity(Eigen::Vector3d(gravityX, gravityY, gravityZ));
      }

      readRequiredWorldMetadataPOD(
          input, m_deformableBodyCounter, "deformable-body counter");

      if (formatVersion >= 6) {
        std::uint8_t differentiableFlag = 0;
        readRequiredWorldMetadataPOD(
            input, differentiableFlag, "differentiable flag");
        DART_SIMULATION_THROW_T_IF(
            differentiableFlag > 1u,
            InvalidArgumentException,
            "Serialized World differentiable flag must be 0 or 1");
        m_differentiable = differentiableFlag == 1u;
      }

      if (formatVersion >= 15) {
        std::uint8_t rigidBodySolver = 0u;
        std::uint8_t contactSolverMethod = 0u;
        std::uint8_t contactGradientMode = 0u;
        std::uint8_t multibodyIntegrationMethod = 0u;
        readRequiredWorldMetadataPOD(
            input, rigidBodySolver, "rigid-body solver");
        readRequiredWorldMetadataPOD(
            input, contactSolverMethod, "contact solver method");
        readRequiredWorldMetadataPOD(
            input, contactGradientMode, "contact gradient mode");
        readRequiredWorldMetadataPOD(
            input, multibodyIntegrationMethod, "multibody integration method");

        DART_SIMULATION_THROW_T_IF(
            formatVersion < 29 && rigidBodySolver == 2u,
            InvalidArgumentException,
            "Serialized World AVBD rigid-body solver requires binary format "
            "version 29 or newer");
        DART_SIMULATION_THROW_T_IF(
            formatVersion < 30 && rigidBodySolver == 3u,
            InvalidArgumentException,
            "Serialized World VBD rigid-body solver requires binary format "
            "version 30 or newer");
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
                "Serialized World multibody integration method value is "
                "invalid");
        }
      }

      if (formatVersion >= 16) {
        std::size_t ignoredPairCount = 0;
        readRequiredWorldMetadataPOD(
            input, ignoredPairCount, "ignored collision-pair count");
        for (std::size_t i = 0; i < ignoredPairCount; ++i) {
          std::uint32_t serializedFirst = 0;
          std::uint32_t serializedSecond = 0;
          readRequiredWorldMetadataPOD(
              input, serializedFirst, "ignored collision-pair first entity");
          readRequiredWorldMetadataPOD(
              input, serializedSecond, "ignored collision-pair second entity");
          const auto first
              = entityMap.at(static_cast<entt::entity>(serializedFirst));
          const auto second
              = entityMap.at(static_cast<entt::entity>(serializedSecond));
          m_storage->ignoredCollisionPairs.insert(
              makeCollisionPairKey(first, second));
        }
      }

      if (formatVersion >= 18) {
        std::size_t variationalMaxIterations = 0;
        double variationalTolerance = 0.0;
        readRequiredWorldMetadataPOD(
            input,
            variationalMaxIterations,
            "variational max-iteration budget");
        readRequiredWorldMetadataPOD(
            input, variationalTolerance, "variational tolerance");
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

      if (formatVersion >= 23) {
        std::uint8_t deactivationEnabled = 0u;
        DeactivationOptions options;
        readRequiredWorldMetadataPOD(
            input, deactivationEnabled, "deactivation-enabled flag");
        readRequiredWorldMetadataPOD(
            input,
            options.linearSpeedThreshold,
            "deactivation linear-speed threshold");
        readRequiredWorldMetadataPOD(
            input,
            options.angularSpeedThreshold,
            "deactivation angular-speed threshold");
        readRequiredWorldMetadataPOD(
            input,
            options.generalizedSpeedThreshold,
            "deactivation generalized-speed threshold");
        readRequiredWorldMetadataPOD(
            input, options.timeUntilSleep, "deactivation sleep time");
        readRequiredWorldMetadataPOD(
            input, options.wakeThresholdScale, "deactivation wake scale");
        readRequiredWorldMetadataPOD(
            input,
            options.disturbanceForceThreshold,
            "deactivation disturbance-force threshold");
        DART_SIMULATION_THROW_T_IF(
            deactivationEnabled > 1u,
            InvalidArgumentException,
            "Serialized World deactivation-enabled flag must be 0 or 1");
        options.enabled = deactivationEnabled == 1u;
        validateDeactivationOptions(options);
        m_deactivationOptions = options;
      }

      if (formatVersion >= 26) {
        std::uint8_t computeAcceleratorPolicy = 0u;
        readRequiredWorldMetadataPOD(
            input, computeAcceleratorPolicy, "compute accelerator policy");
        m_computeAcceleratorPolicy
            = decodeComputeAcceleratorPolicy(computeAcceleratorPolicy);
      }

      if (formatVersion >= 27) {
        std::size_t differentiableParameterCount = 0;
        readRequiredWorldMetadataPOD(
            input,
            differentiableParameterCount,
            "differentiable-parameter count");
        for (std::size_t i = 0; i < differentiableParameterCount; ++i) {
          std::uint32_t serializedEntity = 0;
          std::uint8_t serializedParameter = 0u;
          readRequiredWorldMetadataPOD(
              input, serializedEntity, "differentiable-parameter entity");
          readRequiredWorldMetadataPOD(
              input, serializedParameter, "differentiable-parameter kind");
          const auto serializedEntityKey
              = static_cast<entt::entity>(serializedEntity);
          DART_SIMULATION_THROW_T_IF(
              !entityMap.contains(serializedEntityKey),
              InvalidArgumentException,
              "Serialized World differentiable parameter references an unknown "
              "entity");
          const auto entity = entityMap.at(serializedEntityKey);
          DART_SIMULATION_THROW_T_IF(
              !m_storage->registry.all_of<comps::RigidBodyTag>(entity),
              InvalidArgumentException,
              "Serialized World differentiable parameter references an entity "
              "that is not a rigid body");
          m_storage->differentiableParameters.emplace_back(
              entity,
              decodeDifferentiablePhysicalParameter(serializedParameter));
        }
      }

      if (formatVersion >= 29) {
        std::size_t rigidConstraintIterations = 0;
        readRequiredWorldMetadataPOD(
            input,
            rigidConstraintIterations,
            "rigid constraint iteration budget");
        m_rigidConstraintOptions.iterations
            = validateRigidConstraintIterations(rigidConstraintIterations);
        m_stepPipelineCache->stages.rigidBodyContact.setIterations(
            m_rigidConstraintOptions.iterations);
      }

      if (formatVersion >= 34) {
        double rigidIpcAdaptiveBarrierStiffnessLowerBound = 0.0;
        readRequiredWorldMetadataPOD(
            input,
            rigidIpcAdaptiveBarrierStiffnessLowerBound,
            "rigid IPC adaptive barrier-stiffness lower bound");
        DART_SIMULATION_THROW_T_IF(
            !std::isfinite(rigidIpcAdaptiveBarrierStiffnessLowerBound)
                || rigidIpcAdaptiveBarrierStiffnessLowerBound <= 0.0,
            InvalidArgumentException,
            "Serialized World rigid IPC adaptive barrier-stiffness lower "
            "bound is invalid");
        m_rigidIpcAdaptiveBarrierStiffnessLowerBound
            = rigidIpcAdaptiveBarrierStiffnessLowerBound;
      }
    }

    if (m_rigidBodySolver != RigidBodySolver::Ipc) {
      resetRigidIpcAdaptiveBarrierStiffnessLowerBound();
    }
    if (m_multibodyIntegrationMethod
        != MultibodyIntegrationMethod::Variational) {
      coldStartMultibodyVariationalContinuation(m_storage->registry);
    }

    validateRigidSolverContactMethodCompatibility(
        m_rigidBodySolver, m_contactSolverMethod);
    m_rigidConstraintOptions = validateRigidConstraintOptions(
        m_rigidConstraintOptions, m_rigidBodySolver);
    validateRigidBodyJointPipelineSupport(*this, m_rigidBodySolver);
    validateRigidConstraintOptionsPipelineSupport(
        *this,
        m_rigidConstraintOptions,
        m_rigidBodySolver,
        m_multibodyIntegrationMethod
            == MultibodyIntegrationMethod::Variational);

    // Component deserializers reject malformed numeric fields. Validate the
    // complete executable obstacle envelope before frame caches, counters, or
    // kinematics are rebuilt.
    validateRequiredDeformableVbdConfiguration(*this);

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

    // Every loaded deformable node-state vector must match its node count
    // before any handle or solver may index it; a crafted snapshot with a
    // short vector must fail here, not write out of bounds later.
    const auto loadedNodeStates
        = m_storage->registry.view<comps::DeformableNodeState>();
    for (const entt::entity nodeStateEntity : loadedNodeStates) {
      const auto& state
          = loadedNodeStates.get<comps::DeformableNodeState>(nodeStateEntity);
      const std::size_t nodeCount = state.positions.size();
      DART_SIMULATION_THROW_T_IF(
          state.previousPositions.size() != nodeCount
              || state.velocities.size() != nodeCount
              || state.attachmentTargets.size() != nodeCount,
          InvalidArgumentException,
          "Loaded deformable node state has inconsistent vector sizes: {} "
          "positions, {} previous positions, {} velocities, {} attachment "
          "targets",
          nodeCount,
          state.previousPositions.size(),
          state.velocities.size(),
          state.attachmentTargets.size());
    }

    resetCountersFromRegistry();
    m_storage->bakedModel.valid = false;

    if (m_simulationMode) {
      updateKinematics();
      detail::deformable_vbd::
          configureAvbdRigidWorldPointJointsFromCurrentPoses(
              m_storage->registry);
      prepareStepPipelineCacheForCurrentConfiguration();
    }
  } catch (...) {
    m_collisionQueryCache.reset();
    m_stepPipelineCache.reset();
    m_replay.reset();
    m_storage.reset();

    m_simulationMode = previousSimulationMode;
    m_gravity = previousGravity;
    m_rigidBodySolver = previousRigidBodySolver;
    m_rigidConstraintOptions = previousRigidConstraintOptions;
    m_timeStep = previousTimeStep;
    m_differentiable = previousDifferentiable;
    m_contactSolverMethod = previousContactSolverMethod;
    m_contactGradientMode = previousContactGradientMode;
    m_computeAcceleratorPolicy = previousComputeAcceleratorPolicy;
    m_strictSolverResolution = previousStrictSolverResolution;
    m_deactivationOptions = previousDeactivationOptions;
    m_deformablePsdProjector = previousDeformablePsdProjector;
    m_deformablePsdAcceleratedResolved
        = previousDeformablePsdAcceleratedResolved;
    m_time = previousTime;
    m_lastDeformableSolverDiagnostics = previousDeformableSolverDiagnostics;
#if DART_BUILD_PROFILE
    m_stepProfilingEnabled = previousStepProfilingEnabled;
    m_lastStepProfile = std::move(previousLastStepProfile);
    m_stepProfileScratch = std::move(previousStepProfileScratch);
#endif
    m_resolvedConfiguration = std::move(previousResolvedConfiguration);
    m_rigidIpcAdaptiveBarrierStiffnessLowerBound
        = previousRigidIpcAdaptiveBarrierStiffnessLowerBound;
    m_frameTopologyRevision = previousFrameTopologyRevision;
    m_multibodyIntegrationMethod = previousMultibodyIntegrationMethod;
    m_variationalIntegratorMaxIterations
        = previousVariationalIntegratorMaxIterations;
    m_variationalIntegratorTolerance = previousVariationalIntegratorTolerance;
    m_frame = previousFrame;
    m_freeFrameCounter = previousFreeFrameCounter;
    m_fixedFrameCounter = previousFixedFrameCounter;
    m_multibodyCounter = previousMultibodyCounter;
    m_loopClosureCounter = previousLoopClosureCounter;
    m_rigidBodyCounter = previousRigidBodyCounter;
    m_deformableBodyCounter = previousDeformableBodyCounter;
    m_linkCounter = previousLinkCounter;
    m_jointCounter = previousJointCounter;
    m_callerOwnedPipelineContinuationStateMayBeLive
        = previousCallerOwnedPipelineContinuationStateMayBeLive;
    m_storage = std::move(previousStorage);
    m_collisionQueryCache = std::move(previousCollisionQueryCache);
    m_stepPipelineCache = std::move(previousStepPipelineCache);
    m_replay = std::move(previousReplay);
    throw;
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
