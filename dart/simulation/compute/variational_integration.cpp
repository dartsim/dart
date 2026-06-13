/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the "BSD-style" License
 */

#include "dart/simulation/compute/variational_integration.hpp"

#include "dart/simulation/common/exceptions.hpp"
#include "dart/simulation/comps/dynamics.hpp"
#include "dart/simulation/comps/frame_types.hpp"
#include "dart/simulation/comps/joint.hpp"
#include "dart/simulation/comps/link.hpp"
#include "dart/simulation/comps/loop_closure.hpp"
#include "dart/simulation/comps/multibody.hpp"
#include "dart/simulation/comps/variational_contact.hpp"
#include "dart/simulation/comps/variational_contact_dual_state.hpp"
#include "dart/simulation/compute/multibody_dynamics.hpp"
#include "dart/simulation/detail/deformable_vbd/rigid_world_contact.hpp"
#include "dart/simulation/detail/multibody_spatial_algebra.hpp"
#include "dart/simulation/detail/variational/discrete_mechanics_math.hpp"
#include "dart/simulation/detail/world_registry_access.hpp"
#include "dart/simulation/world.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Geometry>
#include <Eigen/QR>
#include <entt/entt.hpp>

#include <algorithm>
#include <limits>
#include <new>
#include <optional>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

#include <cmath>

namespace dart::simulation::compute {

namespace {

namespace dm = detail::variational;
namespace dvbd = detail::deformable_vbd;

template <typename Vector>
void rebindVectorAllocator(
    Vector& values, const typename Vector::allocator_type& targetAllocator)
{
  if (values.get_allocator() == targetAllocator) {
    return;
  }
  Vector rebound{targetAllocator};
  rebound.assign(values.begin(), values.end());
  values = std::move(rebound);
}

template <typename ConstraintVector>
std::span<const VariationalLoopConstraint> variationalLoopConstraintSpan(
    const ConstraintVector& constraints)
{
  return {constraints.data(), constraints.size()};
}

MultibodyVariationalState makeMultibodyVariationalState(
    dart::common::MemoryAllocator& allocator)
{
  using State = MultibodyVariationalState;
  return State{
      false,
      State::DeltaTransformVector{
          dart::common::StlAllocator<Eigen::Isometry3d>{allocator}},
      State::MomentumVector{
          dart::common::StlAllocator<Eigen::Matrix<double, 6, 1>>{allocator}}};
}

void ensureMultibodyVariationalStateAllocator(
    MultibodyVariationalState& state, dart::common::MemoryAllocator& allocator)
{
  using State = MultibodyVariationalState;
  const dart::common::StlAllocator<Eigen::Isometry3d> targetTransformAllocator{
      allocator};
  if (state.previousDeltaTransform.get_allocator()
      != targetTransformAllocator) {
    State::DeltaTransformVector transforms{targetTransformAllocator};
    transforms.assign(
        state.previousDeltaTransform.begin(),
        state.previousDeltaTransform.end());
    state.previousDeltaTransform = std::move(transforms);
  }

  const dart::common::StlAllocator<Eigen::Matrix<double, 6, 1>>
      targetMomentumAllocator{allocator};
  if (state.previousMomentum.get_allocator() != targetMomentumAllocator) {
    State::MomentumVector momentum{targetMomentumAllocator};
    momentum.assign(
        state.previousMomentum.begin(), state.previousMomentum.end());
    state.previousMomentum = std::move(momentum);
  }
}

comps::VariationalContactDualState makeVariationalContactDualState(
    dart::common::MemoryAllocator& allocator)
{
  using DualState = comps::VariationalContactDualState;
  return DualState{
      DualState::DualVector{dart::common::StlAllocator<double>{allocator}}, 0u};
}

void ensureVariationalContactDualStateAllocator(
    comps::VariationalContactDualState& dualState,
    dart::common::MemoryAllocator& allocator)
{
  using DualState = comps::VariationalContactDualState;
  const dart::common::StlAllocator<double> targetAllocator{allocator};
  if (dualState.duals.get_allocator() == targetAllocator) {
    return;
  }

  DualState::DualVector duals{targetAllocator};
  duals.assign(dualState.duals.begin(), dualState.duals.end());
  dualState.duals = std::move(duals);
}

void ensureVariationalContactAllocator(
    comps::VariationalContact& contact,
    dart::common::MemoryAllocator& allocator)
{
  const dart::common::StlAllocator<std::size_t> targetLinkAllocator{allocator};
  if (contact.pointLinkIndices.get_allocator() != targetLinkAllocator) {
    comps::VariationalContact::LinkIndexVector linkIndices{targetLinkAllocator};
    linkIndices.assign(
        contact.pointLinkIndices.begin(), contact.pointLinkIndices.end());
    contact.pointLinkIndices = std::move(linkIndices);
  }

  const dart::common::StlAllocator<Eigen::Vector3d> targetPointAllocator{
      allocator};
  if (contact.pointLocalPositions.get_allocator() != targetPointAllocator) {
    comps::VariationalContact::PointVector localPositions{targetPointAllocator};
    localPositions.assign(
        contact.pointLocalPositions.begin(), contact.pointLocalPositions.end());
    contact.pointLocalPositions = std::move(localPositions);
  }
}

// The shared 6D spatial-algebra primitives (skew/adjoint/spatialInertia) and
// the Vector6/Matrix6/Subspace aliases live in
// detail/multibody_spatial_algebra.hpp, shared with
// compute/multibody_dynamics.cpp. The VI-specific discrete-mechanics kernels
// (dexpInv et al.) stay in detail/variational/discrete_mechanics_math.hpp (the
// `dm` alias above). The joint relative-transform/subspace and retract helpers
// below are VI-specific (Phase-A1 joint subset) and stay local.
using detail::adjoint;
using detail::Matrix6;
using detail::skew;
using detail::spatialInertia;
using detail::Subspace;
using detail::Vector6;

// SO(3) exponential / logarithm on rotation vectors (axis * angle).
Eigen::Matrix3d rotationExp3(const Eigen::Vector3d& v)
{
  const double angle = v.norm();
  if (angle < 1e-12) {
    return Eigen::Matrix3d::Identity();
  }
  return Eigen::AngleAxisd(angle, v / angle).toRotationMatrix();
}

Eigen::Vector3d rotationLog3(const Eigen::Matrix3d& rotation)
{
  const Eigen::AngleAxisd angleAxis(rotation);
  return angleAxis.angle() * angleAxis.axis();
}

// Relative transform produced by a joint at the given generalized position
// (Phase A1: fixed/revolute/prismatic), before the post-joint link offset.
Eigen::Isometry3d jointMotionTransform(
    const comps::Joint& joint,
    const Eigen::Ref<const Eigen::VectorXd>& position)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  switch (joint.type) {
    case comps::JointType::Fixed:
      return transform;
    case comps::JointType::Revolute:
      transform.linear()
          = Eigen::AngleAxisd(position[0], joint.axis).toRotationMatrix();
      return transform;
    case comps::JointType::Prismatic:
      transform.translation() = joint.axis * position[0];
      return transform;
    case comps::JointType::Spherical:
      // Orientation stored as a rotation vector (exponential coordinates).
      transform.linear() = rotationExp3(position.head<3>());
      return transform;
    case comps::JointType::Floating:
      // 6-DOF pose: translation (position[0..2]) then orientation as a rotation
      // vector (position[3..5]), matching the kinematics convention.
      transform.linear() = rotationExp3(position.tail<3>());
      transform.translation() = position.head<3>();
      return transform;
    default:
      DART_SIMULATION_THROW_T(
          InvalidOperationException,
          "The variational integrator supports fixed, revolute, prismatic, "
          "spherical, and floating joints");
  }
}

// Retract a joint configuration `q` along a generalized-coordinate increment
// `delta` (the same space as generalized velocity). Euclidean joints add
// directly; spherical/floating joints apply the increment on the SO(3)/SE(3)
// manifold, matching the semi-implicit integration convention
// (R_new = R exp(omega), p_new = p + R v).
void jointRetractInto(
    const comps::Joint& joint,
    const Eigen::Ref<const Eigen::VectorXd>& q,
    const Eigen::Ref<const Eigen::VectorXd>& delta,
    Eigen::Ref<Eigen::VectorXd> result)
{
  switch (joint.type) {
    case comps::JointType::Spherical: {
      result.head<3>() = rotationLog3(
          rotationExp3(q.head<3>()) * rotationExp3(delta.head<3>()));
      break;
    }
    case comps::JointType::Floating: {
      const Eigen::Matrix3d rotation = rotationExp3(q.tail<3>());
      result.head<3>() = q.head<3>() + rotation * delta.head<3>();
      result.tail<3>() = rotationLog3(rotation * rotationExp3(delta.tail<3>()));
      break;
    }
    default:
      result = q + delta;
      break;
  }
}

// The generalized-coordinate tangent `tau` with `jointRetract(q, tau) == qNext`
// (so the generalized velocity is `tau / dt`). Inverse of jointRetract.
void jointLogDifferenceInto(
    const comps::Joint& joint,
    const Eigen::Ref<const Eigen::VectorXd>& qNext,
    const Eigen::Ref<const Eigen::VectorXd>& q,
    Eigen::Ref<Eigen::VectorXd> result)
{
  switch (joint.type) {
    case comps::JointType::Spherical: {
      result.head<3>() = rotationLog3(
          rotationExp3(q.head<3>()).transpose()
          * rotationExp3(qNext.head<3>()));
      break;
    }
    case comps::JointType::Floating: {
      const Eigen::Matrix3d rotation = rotationExp3(q.tail<3>());
      result.head<3>() = rotation.transpose() * (qNext.head<3>() - q.head<3>());
      result.tail<3>()
          = rotationLog3(rotation.transpose() * rotationExp3(qNext.tail<3>()));
      break;
    }
    default:
      result = qNext - q;
      break;
  }
}

// Per-link data for the discrete recursion, in construction order
// (parent-before-child).
struct VarLink
{
  using ChildAllocator = dart::common::StlAllocator<int>;
  using ChildVector = std::vector<int, ChildAllocator>;

  VarLink() : VarLink(dart::common::MemoryAllocator::GetDefault()) {}

  explicit VarLink(dart::common::MemoryAllocator& allocator)
    : children(ChildAllocator{allocator})
  {
  }

  int parent = -1;
  entt::entity joint = entt::null;
  std::size_t dof = 0;
  std::size_t dofOffset = 0;
  Eigen::Isometry3d parentToJoint
      = Eigen::Isometry3d::Identity();                      // joint from parent
  Eigen::Isometry3d offset = Eigen::Isometry3d::Identity(); // link from joint
  Matrix6 inertia = Matrix6::Zero();                        // G_i (link frame)
  Subspace subspace{6, 0};                                  // S_i (link frame)
  Eigen::Isometry3d currentRelative
      = Eigen::Isometry3d::Identity(); // T_{lambda(i),i} at q^k
  Eigen::Isometry3d worldTransform = Eigen::Isometry3d::Identity();
  Vector6 externalForce
      = Vector6::Zero(); // body-frame wrench [angular; linear]
  ChildVector children;
  // Per-residual-evaluation scratch.
  Eigen::Isometry3d deltaTransform = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d nextRelative
      = Eigen::Isometry3d::Identity(); // T_{lambda(i),i} at q^{k+1}
  Vector6 averageVelocity = Vector6::Zero();
  Vector6 momentum = Vector6::Zero();
};

using VarLinkAllocator = dart::common::StlAllocator<VarLink>;
using VarLinkVector = std::vector<VarLink, VarLinkAllocator>;
using LinkIndexMapEntry = std::pair<const entt::entity, std::size_t>;
using LinkIndexMapAllocator = dart::common::StlAllocator<LinkIndexMapEntry>;
using LinkIndexMap = std::unordered_map<
    entt::entity,
    std::size_t,
    std::hash<entt::entity>,
    std::equal_to<entt::entity>,
    LinkIndexMapAllocator>;

struct VarTree
{
  VarTree() : VarTree(dart::common::MemoryAllocator::GetDefault()) {}

  explicit VarTree(dart::common::MemoryAllocator& allocator)
    : links(VarLinkAllocator{allocator}), allocator(&allocator)
  {
  }

  VarLinkVector links;
  dart::common::MemoryAllocator* allocator = nullptr;
  std::size_t dofCount = 0;
};

LinkIndexMap makeLinkIndexMap(dart::common::MemoryAllocator& allocator)
{
  return LinkIndexMap{
      0,
      std::hash<entt::entity>{},
      std::equal_to<entt::entity>{},
      LinkIndexMapAllocator{allocator}};
}

void resetVarLinkForBuild(
    VarLink& link, dart::common::MemoryAllocator& allocator)
{
  link.parent = -1;
  link.joint = entt::null;
  link.dof = 0;
  link.dofOffset = 0;
  link.parentToJoint.setIdentity();
  link.offset.setIdentity();
  link.inertia.setZero();
  link.currentRelative.setIdentity();
  link.worldTransform.setIdentity();
  link.externalForce.setZero();
  const VarLink::ChildAllocator targetChildAllocator{allocator};
  if (link.children.get_allocator() != targetChildAllocator) {
    link.children = VarLink::ChildVector{targetChildAllocator};
  } else {
    link.children.clear();
  }
  link.deltaTransform.setIdentity();
  link.nextRelative.setIdentity();
  link.averageVelocity.setZero();
  link.momentum.setZero();
}

void setLinkFrameJointSubspaceInto(
    const comps::Joint& joint,
    const Eigen::Isometry3d& linkFromJoint,
    Subspace& subspace)
{
  const Matrix6 jointToLinkAdjoint = adjoint(linkFromJoint.inverse());
  switch (joint.type) {
    case comps::JointType::Fixed:
      subspace.resize(6, 0);
      return;
    case comps::JointType::Revolute: {
      subspace.resize(6, 1);
      Vector6 jointFrameSubspace = Vector6::Zero();
      jointFrameSubspace.head<3>() = joint.axis;
      subspace.col(0).noalias() = jointToLinkAdjoint * jointFrameSubspace;
      return;
    }
    case comps::JointType::Prismatic: {
      subspace.resize(6, 1);
      Vector6 jointFrameSubspace = Vector6::Zero();
      jointFrameSubspace.tail<3>() = joint.axis;
      subspace.col(0).noalias() = jointToLinkAdjoint * jointFrameSubspace;
      return;
    }
    case comps::JointType::Spherical: {
      subspace.resize(6, 3);
      Eigen::Matrix<double, 6, 3> jointFrameSubspace;
      jointFrameSubspace.setZero();
      jointFrameSubspace.topRows<3>().setIdentity();
      subspace.noalias() = jointToLinkAdjoint * jointFrameSubspace;
      return;
    }
    case comps::JointType::Floating: {
      subspace.resize(6, 6);
      Matrix6 jointFrameSubspace = Matrix6::Zero();
      jointFrameSubspace.bottomLeftCorner<3, 3>().setIdentity();
      jointFrameSubspace.topRightCorner<3, 3>().setIdentity();
      subspace.noalias() = jointToLinkAdjoint * jointFrameSubspace;
      return;
    }
    default:
      DART_SIMULATION_THROW_T(
          InvalidOperationException,
          "The variational integrator supports fixed, revolute, prismatic, "
          "spherical, and floating joints");
  }
}

void buildVarTreeInto(
    const detail::WorldRegistry& registry,
    const comps::MultibodyStructure& structure,
    VarTree& tree,
    LinkIndexMap& indexOf)
{
  const auto& linkEntities = structure.links;
  tree.links.resize(linkEntities.size());
  tree.dofCount = 0;
  dart::common::MemoryAllocator& allocator
      = tree.allocator != nullptr ? *tree.allocator
                                  : dart::common::MemoryAllocator::GetDefault();
  for (auto& link : tree.links) {
    resetVarLinkForBuild(link, allocator);
  }

  // O(1) link-index lookup so building the tree stays O(n); keep the map nodes
  // alive across same-shape steps instead of clearing and reallocating them.
  bool indexMatches = indexOf.size() == linkEntities.size();
  if (indexMatches) {
    for (std::size_t i = 0; i < linkEntities.size(); ++i) {
      const auto it = indexOf.find(linkEntities[i]);
      if (it == indexOf.end() || it->second != i) {
        indexMatches = false;
        break;
      }
    }
  }
  if (!indexMatches) {
    indexOf.clear();
    indexOf.reserve(linkEntities.size());
    for (std::size_t i = 0; i < linkEntities.size(); ++i) {
      indexOf.emplace(linkEntities[i], i);
    }
  }

  for (std::size_t i = 0; i < linkEntities.size(); ++i) {
    const auto linkEntity = linkEntities[i];
    const auto& linkComp = registry.get<comps::Link>(linkEntity);
    auto& link = tree.links[i];
    link.inertia = spatialInertia(linkComp.mass);
    link.parentToJoint = linkComp.transformFromParentToJoint;
    link.offset = linkComp.transformFromParentJoint;
    link.externalForce = linkComp.externalForce;

    if (linkComp.parentJoint == entt::null) {
      const auto& cache = registry.get<comps::FrameCache>(linkEntity);
      link.worldTransform = cache.worldTransform;
      link.subspace.resize(6, 0);
      link.parent = -1;
      continue;
    }

    const auto& joint = registry.get<comps::Joint>(linkComp.parentJoint);
    link.joint = linkComp.parentJoint;
    const auto parentIt = indexOf.find(joint.parentLink);
    DART_SIMULATION_THROW_T_IF(
        parentIt == indexOf.end(),
        InvalidOperationException,
        "Multibody link parent is not part of the same multibody");
    link.parent = static_cast<int>(parentIt->second);

    setLinkFrameJointSubspaceInto(joint, link.offset, link.subspace);
    link.dof = static_cast<std::size_t>(link.subspace.cols());
    link.dofOffset = tree.dofCount;
    tree.dofCount += link.dof;
    link.currentRelative = link.parentToJoint
                           * jointMotionTransform(joint, joint.position)
                           * link.offset;
    link.worldTransform
        = tree.links[static_cast<std::size_t>(link.parent)].worldTransform
          * link.currentRelative;
    tree.links[static_cast<std::size_t>(link.parent)].children.push_back(
        static_cast<int>(i));
  }
}

VarTree buildVarTree(
    const detail::WorldRegistry& registry,
    const comps::MultibodyStructure& structure)
{
  VarTree tree;
  LinkIndexMap indexOf
      = makeLinkIndexMap(dart::common::MemoryAllocator::GetDefault());
  buildVarTreeInto(registry, structure, tree, indexOf);
  return tree;
}

void updateVarTreeConfigurationInto(
    const detail::WorldRegistry& registry,
    VarTree& tree,
    const Eigen::VectorXd& position)
{
  for (std::size_t i = 0; i < tree.links.size(); ++i) {
    auto& link = tree.links[i];
    if (link.parent < 0) {
      continue;
    }
    const auto& joint = registry.get<comps::Joint>(link.joint);
    const auto seg = static_cast<Eigen::Index>(link.dofOffset);
    const auto n = static_cast<Eigen::Index>(link.dof);
    link.currentRelative
        = link.parentToJoint
          * jointMotionTransform(joint, position.segment(seg, n)) * link.offset;
    link.worldTransform
        = tree.links[static_cast<std::size_t>(link.parent)].worldTransform
          * link.currentRelative;
  }
}

void reserveVariationalAndersonScratch(
    const VarTree& tree,
    std::size_t andersonDepth,
    VariationalAndersonScratch& scratch)
{
  const auto dofCount = static_cast<Eigen::Index>(tree.dofCount);
  const auto depth = static_cast<Eigen::Index>(andersonDepth);
  scratch.historyCount = 0;
  scratch.stepDeltas.resize(andersonDepth);
  scratch.iterateDeltas.resize(andersonDepth);
  for (auto& delta : scratch.stepDeltas) {
    delta.resize(dofCount);
  }
  for (auto& delta : scratch.iterateDeltas) {
    delta.resize(dofCount);
  }
  scratch.previousStep.resize(dofCount);
  scratch.previousPosition.resize(dofCount);
  scratch.trialPosition.resize(dofCount);
  scratch.andersonPosition.resize(dofCount);
  scratch.andersonIncrement.resize(dofCount);
  Eigen::Index maxJointDof = 0;
  for (const auto& link : tree.links) {
    maxJointDof = std::max(maxJointDof, static_cast<Eigen::Index>(link.dof));
  }
  scratch.jointDelta.resize(maxJointDof);
  scratch.stepMatrix.resize(dofCount, depth);
  scratch.mixMatrix.resize(dofCount, depth);
  scratch.ftf.resize(depth, depth);
  scratch.regularized.resize(depth, depth);
  scratch.normalRhs.resize(depth);
  scratch.gamma.resize(depth);
}

void reserveVariationalLinearSolveScratch(
    const VarTree& tree, VariationalLinearSolveScratch& scratch)
{
  const std::size_t linkCount = tree.links.size();
  const auto dofCount = static_cast<Eigen::Index>(tree.dofCount);
  scratch.articulated.resize(linkCount);
  scratch.bias.resize(linkCount);
  scratch.motionToChild.resize(linkCount);
  scratch.spatial.resize(linkCount);
  scratch.forceProjector.resize(linkCount);
  scratch.motionProjector.resize(linkCount);
  scratch.jointMatrix.resize(linkCount);
  scratch.jointMatrixInverse.resize(linkCount);
  scratch.jointRhs.resize(linkCount);

  Eigen::Index maxJointDof = 0;
  for (std::size_t i = 0; i < linkCount; ++i) {
    const auto dof = static_cast<Eigen::Index>(tree.links[i].dof);
    maxJointDof = std::max(maxJointDof, dof);
    scratch.forceProjector[i].resize(6, dof);
    scratch.motionProjector[i].resize(6, dof);
    scratch.jointMatrix[i].resize(dof, dof);
    scratch.jointMatrixInverse[i].resize(dof, dof);
    scratch.jointRhs[i].resize(dof);
  }
  scratch.jointWork.resize(maxJointDof);
  scratch.jointSolveWork.resize(maxJointDof);
  scratch.rhs.resize(dofCount);
  scratch.result.resize(dofCount);
}

void reserveVariationalStepScratch(
    const VarTree& tree, VariationalStepScratch& scratch)
{
  const auto dofCount = static_cast<Eigen::Index>(tree.dofCount);
  Eigen::Index maxJointDof = 0;
  for (const auto& link : tree.links) {
    maxJointDof = std::max(maxJointDof, static_cast<Eigen::Index>(link.dof));
  }
  scratch.currentSpatialVelocities.resize(tree.links.size());
  scratch.position.resize(dofCount);
  scratch.velocity.resize(dofCount);
  scratch.appliedForce.resize(dofCount);
  scratch.nextPosition.resize(dofCount);
  scratch.residual.resize(dofCount);
  scratch.zeroAcceleration.resize(dofCount);
  scratch.previousJointVelocity.resize(maxJointDof);
}

bool isTopologyMultibodyJoint(
    const detail::WorldRegistry& registry,
    entt::entity jointEntity,
    const comps::Joint& joint)
{
  const auto* childLink = registry.try_get<comps::Link>(joint.childLink);
  return childLink != nullptr && childLink->parentJoint == jointEntity;
}

bool isHardAvbdRigidWorldPointJointConfig(
    const dvbd::AvbdRigidWorldPointJointConfig& config)
{
  return std::isinf(config.startStiffness) && std::isinf(config.maxStiffness);
}

std::optional<double> avbdRigidWorldCompliantPointJointStiffness(
    const dvbd::AvbdRigidWorldPointJointConfig& config)
{
  if (!std::isfinite(config.startStiffness) || config.startStiffness <= 0.0) {
    return std::nullopt;
  }
  if (std::isfinite(config.maxStiffness)
      && config.maxStiffness < config.startStiffness) {
    return std::nullopt;
  }
  return config.startStiffness;
}

void markMultibodyLinkFrameCachesDirty(
    detail::WorldRegistry& registry, const comps::MultibodyStructure& structure)
{
  for (const auto linkEntity : structure.links) {
    if (auto* cache = registry.try_get<comps::FrameCache>(linkEntity)) {
      cache->needTransformUpdate = true;
    }
  }
}

std::size_t activeVariationalLoopAxisCount(std::uint8_t mask)
{
  std::size_t count = 0;
  for (std::uint8_t axis = 0; axis < 3u; ++axis) {
    if (dvbd::detail::avbdRigidJointAxisEnabled(mask, axis)) {
      ++count;
    }
  }
  return count;
}

std::optional<std::uint8_t> singleInactiveVariationalLoopAxis(std::uint8_t mask)
{
  std::optional<std::uint8_t> inactiveAxis;
  for (std::uint8_t axis = 0; axis < 3u; ++axis) {
    if (dvbd::detail::avbdRigidJointAxisEnabled(mask, axis)) {
      continue;
    }
    if (inactiveAxis.has_value()) {
      return std::nullopt;
    }
    inactiveAxis = axis;
  }
  return inactiveAxis;
}

Eigen::Index variationalLoopConstraintRowCount(
    const VariationalLoopConstraint& constraint)
{
  if (constraint.distance) {
    return 1;
  }

  Eigen::Index rows = static_cast<Eigen::Index>(
      activeVariationalLoopAxisCount(constraint.linearAxisMask));
  if (constraint.linearMotor) {
    rows += 1;
  }
  if (constraint.rigid) {
    rows += static_cast<Eigen::Index>(
        activeVariationalLoopAxisCount(constraint.angularAxisMask));
    if (constraint.angularMotor) {
      rows += 1;
    }
  }
  return rows;
}

VariationalProjectionRowBounds variationalMotorProjectionBounds(
    double maxEffort, double timeStep)
{
  if (maxEffort <= 0.0 || std::isnan(maxEffort) || timeStep <= 0.0
      || !std::isfinite(timeStep)) {
    return {0.0, 0.0};
  }

  double maxImpulse = std::numeric_limits<double>::infinity();
  if (std::isfinite(maxEffort)) {
    maxImpulse = maxEffort * timeStep * timeStep;
  }
  return {-maxImpulse, maxImpulse};
}

bool variationalProjectionRowHasFiniteBounds(
    const VariationalProjectionRowBounds& bounds)
{
  return std::isfinite(bounds.lower) || std::isfinite(bounds.upper);
}

double clampVariationalProjectionRowForce(
    double value, const VariationalProjectionRowBounds& bounds)
{
  return std::clamp(value, bounds.lower, bounds.upper);
}

template <typename BoundsVector>
void variationalLoopConstraintRowBoundsInto(
    std::span<const VariationalLoopConstraint> constraints,
    double timeStep,
    BoundsVector& bounds)
{
  bounds.clear();
  for (const auto& constraint : constraints) {
    bounds.reserve(
        bounds.size() + variationalLoopConstraintRowCount(constraint));

    if (constraint.distance) {
      bounds.push_back({});
      continue;
    }

    for (std::uint8_t axis = 0; axis < 3u; ++axis) {
      if (dvbd::detail::avbdRigidJointAxisEnabled(
              constraint.linearAxisMask, axis)) {
        bounds.push_back({});
      }
    }
    if (constraint.linearMotor) {
      bounds.push_back(variationalMotorProjectionBounds(
          constraint.linearMotorMaxForce, timeStep));
    }

    if (!constraint.rigid) {
      continue;
    }

    for (std::uint8_t axis = 0; axis < 3u; ++axis) {
      if (dvbd::detail::avbdRigidJointAxisEnabled(
              constraint.angularAxisMask, axis)) {
        bounds.push_back({});
      }
    }
    if (constraint.angularMotor) {
      bounds.push_back(variationalMotorProjectionBounds(
          constraint.angularMotorMaxTorque, timeStep));
    }
  }
}

Eigen::Vector3d normalizedVariationalLoopAxis(
    const Eigen::Vector3d& axis, const Eigen::Vector3d& fallback)
{
  const double norm = axis.norm();
  if (!axis.allFinite() || norm <= 0.0) {
    return fallback;
  }
  return axis / norm;
}

Eigen::Vector3d variationalLoopAxis(
    const Eigen::Matrix3d& axes, std::uint8_t axis)
{
  if (axis >= 3u) {
    return Eigen::Vector3d::UnitX();
  }

  return normalizedVariationalLoopAxis(
      axes.col(axis), Eigen::Vector3d::Unit(axis));
}

struct VariationalCompliantLoopConstraint
{
  struct AxisRow
  {
    std::uint8_t axis = 0;
    dvbd::AvbdScalarRowDescriptor descriptor;
    double stiffness = 0.0;
  };

  using AxisRowAllocator = dart::common::StlAllocator<AxisRow>;
  using AxisRowVector = std::vector<AxisRow, AxisRowAllocator>;

  VariationalCompliantLoopConstraint() = default;

  explicit VariationalCompliantLoopConstraint(
      dart::common::MemoryAllocator& allocator)
    : linearRows(AxisRowAllocator{allocator}),
      angularRows(AxisRowAllocator{allocator})
  {
  }

  int linkA = -1; ///< -1 => pointA is a world anchor.
  Eigen::Vector3d pointA = Eigen::Vector3d::Zero();
  int linkB = -1; ///< -1 => pointB is a world anchor.
  Eigen::Vector3d pointB = Eigen::Vector3d::Zero();
  bool rigid = false;
  Eigen::Matrix3d rotationA = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d rotationB = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d linearAxes = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d angularAxes = Eigen::Matrix3d::Identity();
  std::uint8_t linearAxisMask = dvbd::kAvbdRigidJointAllAxesMask;
  std::uint8_t angularAxisMask = dvbd::kAvbdRigidJointAllAxesMask;
  AxisRowVector linearRows;
  AxisRowVector angularRows;
};

struct VariationalCompliantLoopScratch
{
  using ConstraintAllocator
      = dart::common::StlAllocator<VariationalCompliantLoopConstraint>;
  using ConstraintVector
      = std::vector<VariationalCompliantLoopConstraint, ConstraintAllocator>;
  using DescriptorAllocator
      = dart::common::StlAllocator<dvbd::AvbdScalarRowDescriptor>;
  using DescriptorVector
      = std::vector<dvbd::AvbdScalarRowDescriptor, DescriptorAllocator>;

  explicit VariationalCompliantLoopScratch(
      dart::common::MemoryAllocator& allocator)
    : memoryAllocator(&allocator),
      constraints(ConstraintAllocator{allocator}),
      linearInventory(allocator),
      angularInventory(allocator),
      linearDescriptors(DescriptorAllocator{allocator}),
      angularDescriptors(DescriptorAllocator{allocator}),
      structureLinkIndex(makeLinkIndexMap(allocator))
  {
  }

  void clear()
  {
    constraints.clear();
    linearDescriptors.clear();
    angularDescriptors.clear();
    linearInventory.records().clear();
    angularInventory.records().clear();
  }

  dart::common::MemoryAllocator* memoryAllocator = nullptr;
  ConstraintVector constraints;
  dvbd::AvbdScalarRowInventory linearInventory;
  dvbd::AvbdScalarRowInventory angularInventory;
  DescriptorVector linearDescriptors;
  DescriptorVector angularDescriptors;
  // Kept populated across same-shape steps so map nodes do not churn.
  LinkIndexMap structureLinkIndex;
};

VariationalCompliantLoopScratch& getOrCreateVariationalCompliantLoopScratch(
    detail::WorldRegistry& registry,
    entt::entity entity,
    dart::common::MemoryAllocator& allocator)
{
  if (auto* scratch
      = registry.try_get<VariationalCompliantLoopScratch>(entity)) {
    return *scratch;
  }
  return registry.emplace<VariationalCompliantLoopScratch>(entity, allocator);
}

dvbd::AvbdScalarRowDescriptor makeVariationalCompliantLoopRowDescriptor(
    dvbd::AvbdScalarRowRole role,
    entt::entity jointEntity,
    const comps::Joint& joint,
    const dvbd::AvbdRigidWorldPointJointConfig& config,
    std::uint8_t axis)
{
  dvbd::AvbdScalarRowDescriptor descriptor;
  descriptor.key.role = role;
  descriptor.key.objectA = dvbd::avbdRigidWorldContactObjectId(jointEntity);
  descriptor.key.objectB = 0;
  descriptor.key.featureA = dvbd::avbdRigidWorldContactObjectId(
      joint.parentLink == entt::null ? jointEntity : joint.parentLink);
  descriptor.key.featureB = dvbd::avbdRigidWorldContactObjectId(
      joint.childLink == entt::null ? jointEntity : joint.childLink);
  descriptor.key.axis = axis;
  descriptor.kind = dvbd::AvbdScalarRowKind::FiniteStiffness;
  descriptor.startStiffness = config.startStiffness;
  descriptor.materialStiffness = role == dvbd::AvbdScalarRowRole::JointLinear
                                     ? config.linearMaterialStiffness
                                     : config.angularMaterialStiffness;
  descriptor.maxStiffness = config.maxStiffness;
  return descriptor;
}

template <typename ConstraintVector>
void appendAvbdRigidWorldArticulatedPointJointConstraints(
    const detail::WorldRegistry& registry,
    entt::entity structureEntity,
    ConstraintVector& constraints,
    VariationalCompliantLoopScratch* compliantScratch = nullptr)
{
  const auto& structure
      = registry.get<comps::MultibodyStructure>(structureEntity);
  constexpr std::size_t kStructureLinkIndexMapThreshold = 16u;
  LinkIndexMap localStructureLinkIndex
      = makeLinkIndexMap(dart::common::MemoryAllocator::GetDefault());
  LinkIndexMap* structureLinkIndex = nullptr;
  if (structure.links.size() > kStructureLinkIndexMapThreshold) {
    structureLinkIndex = compliantScratch != nullptr
                             ? &compliantScratch->structureLinkIndex
                             : &localStructureLinkIndex;
    bool indexMatches = structureLinkIndex->size() == structure.links.size();
    if (indexMatches) {
      for (std::size_t i = 0; i < structure.links.size(); ++i) {
        const auto it = structureLinkIndex->find(structure.links[i]);
        if (it == structureLinkIndex->end() || it->second != i) {
          indexMatches = false;
          break;
        }
      }
    }
    if (!indexMatches) {
      structureLinkIndex->clear();
      structureLinkIndex->reserve(structure.links.size());
      for (std::size_t i = 0; i < structure.links.size(); ++i) {
        structureLinkIndex->emplace(structure.links[i], i);
      }
    }
  }
  const auto linkIndexInStructure = [&](entt::entity link) -> int {
    if (link == entt::null) {
      return -1;
    }
    if (structureLinkIndex != nullptr && !structureLinkIndex->empty()) {
      const auto it = structureLinkIndex->find(link);
      return it == structureLinkIndex->end() ? -1
                                             : static_cast<int>(it->second);
    }
    const auto it
        = std::find(structure.links.begin(), structure.links.end(), link);
    return it == structure.links.end()
               ? -1
               : static_cast<int>(it - structure.links.begin());
  };

  auto view
      = registry.view<comps::Joint, dvbd::AvbdRigidWorldPointJointConfig>();
  for (const entt::entity jointEntity : view) {
    const auto& joint = view.get<comps::Joint>(jointEntity);
    const auto& config
        = view.get<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);

    const bool hard = isHardAvbdRigidWorldPointJointConfig(config);
    const std::optional<double> compliantStiffness
        = avbdRigidWorldCompliantPointJointStiffness(config);
    if (!config.enabled || joint.broken
        || !dvbd::isAvbdRigidWorldPointJointType(joint.type)
        || isTopologyMultibodyJoint(registry, jointEntity, joint)) {
      continue;
    }
    if (!hard
        && (compliantScratch == nullptr || !compliantStiffness.has_value()
            || joint.type != comps::JointType::Fixed)) {
      continue;
    }
    if (!dvbd::detail::hasValidActiveAvbdRigidJointAxes(
            config.linearAxes, config.linearAxisMask)
        || !dvbd::detail::hasValidActiveAvbdRigidJointAxes(
            config.angularAxes, config.angularAxisMask)) {
      continue;
    }
    if (!config.localAnchorA.allFinite() || !config.localAnchorB.allFinite()
        || !config.targetRelativeOrientation.coeffs().allFinite()
        || config.targetRelativeOrientation.norm() == 0.0) {
      continue;
    }
    const std::size_t linearRows
        = activeVariationalLoopAxisCount(config.linearAxisMask);
    const std::size_t angularRows
        = activeVariationalLoopAxisCount(config.angularAxisMask);
    if (linearRows == 0u && angularRows == 0u) {
      continue;
    }

    const bool worldA = joint.parentLink == entt::null;
    const bool worldB = joint.childLink == entt::null;
    const dvbd::AvbdRigidWorldEndpoint endpointA
        = dvbd::classifyAvbdRigidWorldEndpoint(registry, joint.parentLink);
    const dvbd::AvbdRigidWorldEndpoint endpointB
        = dvbd::classifyAvbdRigidWorldEndpoint(registry, joint.childLink);
    if ((!worldA
         && endpointA.kind != dvbd::AvbdRigidWorldEndpointKind::MultibodyLink)
        || (!worldB
            && endpointB.kind
                   != dvbd::AvbdRigidWorldEndpointKind::MultibodyLink)) {
      continue;
    }

    const int linkIndexA = worldA ? -1 : linkIndexInStructure(joint.parentLink);
    const int linkIndexB = worldB ? -1 : linkIndexInStructure(joint.childLink);
    if ((!worldA && linkIndexA < 0) || (!worldB && linkIndexB < 0)) {
      continue;
    }
    if (linkIndexA < 0 && linkIndexB < 0) {
      continue;
    }

    VariationalLoopConstraint constraint;
    constraint.linkA = linkIndexA >= 0 ? joint.parentLink : entt::null;
    constraint.pointA = config.localAnchorA;
    constraint.linkB = linkIndexB >= 0 ? joint.childLink : entt::null;
    constraint.pointB = config.localAnchorB;
    constraint.rigid = angularRows > 0u;
    constraint.rotationA
        = dvbd::normalizeAvbdRigidOrientation(config.targetRelativeOrientation)
              .toRotationMatrix();
    constraint.rotationB = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d worldRotationA = Eigen::Matrix3d::Identity();
    if (!worldA) {
      worldRotationA = dvbd::avbdRigidWorldContactFrameWorldTransform(
                           registry, joint.parentLink)
                           .linear();
    }
    Eigen::Matrix3d worldRotationB = Eigen::Matrix3d::Identity();
    if (!worldB) {
      worldRotationB = dvbd::avbdRigidWorldContactFrameWorldTransform(
                           registry, joint.childLink)
                           .linear();
    }
    const Eigen::Matrix3d axisFrame = worldRotationA;

    if (!hard) {
      VariationalCompliantLoopConstraint compliantConstraint{
          *compliantScratch->memoryAllocator};
      compliantConstraint.linkA = linkIndexA;
      compliantConstraint.pointA = config.localAnchorA;
      compliantConstraint.linkB = linkIndexB;
      compliantConstraint.pointB = config.localAnchorB;
      compliantConstraint.rigid = angularRows > 0u;
      compliantConstraint.rotationA = dvbd::normalizeAvbdRigidOrientation(
                                          config.targetRelativeOrientation)
                                          .toRotationMatrix();
      compliantConstraint.rotationB = Eigen::Matrix3d::Identity();
      compliantConstraint.linearAxes = axisFrame * config.linearAxes;
      compliantConstraint.angularAxes = axisFrame * config.angularAxes;
      compliantConstraint.linearAxisMask = config.linearAxisMask;
      compliantConstraint.angularAxisMask = config.angularAxisMask;
      compliantConstraint.linearRows.reserve(linearRows);
      for (std::uint8_t axis = 0; axis < 3u; ++axis) {
        if (!dvbd::detail::avbdRigidJointAxisEnabled(
                config.linearAxisMask, axis)) {
          continue;
        }
        compliantConstraint.linearRows.push_back(
            VariationalCompliantLoopConstraint::AxisRow{
                axis,
                makeVariationalCompliantLoopRowDescriptor(
                    dvbd::AvbdScalarRowRole::JointLinear,
                    jointEntity,
                    joint,
                    config,
                    axis),
                *compliantStiffness});
      }
      compliantConstraint.angularRows.reserve(angularRows);
      for (std::uint8_t axis = 0; axis < 3u; ++axis) {
        if (!dvbd::detail::avbdRigidJointAxisEnabled(
                config.angularAxisMask, axis)) {
          continue;
        }
        compliantConstraint.angularRows.push_back(
            VariationalCompliantLoopConstraint::AxisRow{
                axis,
                makeVariationalCompliantLoopRowDescriptor(
                    dvbd::AvbdScalarRowRole::JointAngular,
                    jointEntity,
                    joint,
                    config,
                    axis),
                *compliantStiffness});
      }
      compliantScratch->constraints.push_back(std::move(compliantConstraint));
      continue;
    }

    constraint.linearAxes = axisFrame * config.linearAxes;
    constraint.angularAxes = axisFrame * config.angularAxes;
    constraint.linearAxisMask = config.linearAxisMask;
    constraint.angularAxisMask = config.angularAxisMask;
    if (joint.type == comps::JointType::Prismatic
        && joint.actuatorType == comps::ActuatorType::Velocity
        && joint.commandVelocity.size() == 1
        && joint.commandVelocity.allFinite()) {
      const double maxForce = dvbd::avbdRigidWorldSymmetricEffortLimit(joint);
      const std::optional<std::uint8_t> freeAxis
          = singleInactiveVariationalLoopAxis(config.linearAxisMask);
      if (maxForce > 0.0 && !std::isnan(maxForce) && freeAxis.has_value()) {
        constraint.linearMotor = true;
        constraint.linearMotorAxis
            = variationalLoopAxis(constraint.linearAxes, *freeAxis);
        const Eigen::Vector3d pointA
            = (worldA ? Eigen::Isometry3d::Identity()
                      : dvbd::avbdRigidWorldContactFrameWorldTransform(
                            registry, joint.parentLink))
              * constraint.pointA;
        const Eigen::Vector3d pointB
            = (worldB ? Eigen::Isometry3d::Identity()
                      : dvbd::avbdRigidWorldContactFrameWorldTransform(
                            registry, joint.childLink))
              * constraint.pointB;
        const Eigen::Vector3d positionResidual = pointA - pointB;
        constraint.linearMotorReferencePosition
            = -constraint.linearMotorAxis.dot(positionResidual);
        constraint.linearMotorTargetSpeed = joint.commandVelocity[0];
        constraint.linearMotorMaxForce = maxForce;
      }
    }
    if (joint.type == comps::JointType::Revolute
        && joint.actuatorType == comps::ActuatorType::Velocity
        && joint.commandVelocity.size() == 1
        && joint.commandVelocity.allFinite()) {
      const double maxTorque = dvbd::avbdRigidWorldSymmetricEffortLimit(joint);
      const std::optional<std::uint8_t> freeAxis
          = singleInactiveVariationalLoopAxis(config.angularAxisMask);
      if (maxTorque > 0.0 && !std::isnan(maxTorque) && freeAxis.has_value()) {
        constraint.angularMotor = true;
        constraint.angularMotorAxis
            = variationalLoopAxis(constraint.angularAxes, *freeAxis);
        const Eigen::Matrix3d rotA = worldRotationA * constraint.rotationA;
        const Eigen::Matrix3d rotB = worldRotationB * constraint.rotationB;
        const Eigen::Vector3d angularResidual
            = rotB * rotationLog3(rotB.transpose() * rotA);
        constraint.angularMotorReferencePosition
            = -constraint.angularMotorAxis.dot(angularResidual);
        constraint.angularMotorTargetSpeed = joint.commandVelocity[0];
        constraint.angularMotorMaxTorque = maxTorque;
      }
    }
    constraint.sourceJoint = jointEntity;
    constraint.breakForce = joint.breakForce;
    constraints.push_back(constraint);
  }
}

void markBrokenAvbdVariationalLoopConstraints(
    detail::WorldRegistry& registry,
    std::span<const VariationalLoopConstraint> constraints,
    const Eigen::VectorXd& lambda)
{
  Eigen::Index row = 0;
  for (const auto& constraint : constraints) {
    const Eigen::Index rows = variationalLoopConstraintRowCount(constraint);
    if (rows <= 0) {
      continue;
    }

    if (constraint.sourceJoint != entt::null && constraint.breakForce > 0.0
        && std::isfinite(constraint.breakForce) && row + rows <= lambda.size()
        && registry.all_of<comps::Joint>(constraint.sourceJoint)) {
      const double load = lambda.segment(row, rows).norm();
      auto& joint = registry.get<comps::Joint>(constraint.sourceJoint);
      if (load >= constraint.breakForce) {
        joint.broken = true;
      }
    }

    row += rows;
  }
}

constexpr double kVariationalCompliantLoopStiffnessGrowthBeta = 1000.0;

void syncVariationalCompliantLoopConstraintRows(
    VariationalCompliantLoopScratch& scratch)
{
  auto& linearDescriptors = scratch.linearDescriptors;
  auto& angularDescriptors = scratch.angularDescriptors;
  linearDescriptors.clear();
  angularDescriptors.clear();
  for (const VariationalCompliantLoopConstraint& constraint :
       scratch.constraints) {
    for (const auto& row : constraint.linearRows) {
      linearDescriptors.push_back(row.descriptor);
    }
    for (const auto& row : constraint.angularRows) {
      angularDescriptors.push_back(row.descriptor);
    }
  }

  const dvbd::AvbdRowWarmStartOptions warmStartOptions;
  scratch.linearInventory.syncActiveRows(linearDescriptors, warmStartOptions);
  scratch.angularInventory.syncActiveRows(angularDescriptors, warmStartOptions);

  std::size_t linearCursor = 0;
  std::size_t angularCursor = 0;
  for (VariationalCompliantLoopConstraint& constraint : scratch.constraints) {
    for (auto& row : constraint.linearRows) {
      if (linearCursor >= scratch.linearInventory.size()) {
        return;
      }
      row.stiffness = scratch.linearInventory[linearCursor++].state.stiffness;
    }
    for (auto& row : constraint.angularRows) {
      if (angularCursor >= scratch.angularInventory.size()) {
        return;
      }
      row.stiffness = scratch.angularInventory[angularCursor++].state.stiffness;
    }
  }
}

VarTree& buildVarTreeIntoScratch(
    MultibodyVariationalTreeScratch& scratch,
    const detail::WorldRegistry& registry,
    const comps::MultibodyStructure& structure);

void updateVariationalCompliantLoopConstraintRows(
    const detail::WorldRegistry& registry,
    const comps::MultibodyStructure& structure,
    MultibodyVariationalTreeScratch& treeScratch,
    VariationalCompliantLoopScratch& scratch)
{
  const auto& constraints = scratch.constraints;
  if (constraints.empty()) {
    scratch.clear();
    return;
  }

  const VarTree& tree
      = buildVarTreeIntoScratch(treeScratch, registry, structure);
  const auto worldPoint = [&](int linkIndex, const Eigen::Vector3d& point) {
    if (linkIndex < 0) {
      return point;
    }
    const auto index = static_cast<std::size_t>(linkIndex);
    DART_SIMULATION_THROW_T_IF(
        index >= tree.links.size(),
        InvalidOperationException,
        "Compliant variational loop constraint references an out-of-range "
        "link index");
    return tree.links[index].worldTransform * point;
  };
  const auto worldRotation
      = [&](int linkIndex, const Eigen::Matrix3d& offset) -> Eigen::Matrix3d {
    if (linkIndex < 0) {
      return offset;
    }
    const auto index = static_cast<std::size_t>(linkIndex);
    DART_SIMULATION_THROW_T_IF(
        index >= tree.links.size(),
        InvalidOperationException,
        "Compliant variational loop constraint references an out-of-range "
        "link index");
    return tree.links[index].worldTransform.linear() * offset;
  };
  const auto updateStiffness
      = [](dvbd::AvbdScalarRowRecord& record, double constraintValue) {
          const double materialStiffness = dvbd::maxAvbdDescriptorStiffness(
              record.descriptor, dvbd::AvbdRowWarmStartOptions{});
          record.state.stiffness = dvbd::updateAvbdFiniteStiffness(
              record.state.stiffness,
              constraintValue,
              kVariationalCompliantLoopStiffnessGrowthBeta,
              materialStiffness);
        };

  std::size_t linearCursor = 0;
  std::size_t angularCursor = 0;
  for (const VariationalCompliantLoopConstraint& constraint : constraints) {
    const Eigen::Vector3d pointA
        = worldPoint(constraint.linkA, constraint.pointA);
    const Eigen::Vector3d pointB
        = worldPoint(constraint.linkB, constraint.pointB);
    const Eigen::Vector3d positionResidual = pointA - pointB;
    for (const auto& row : constraint.linearRows) {
      if (linearCursor >= scratch.linearInventory.size()) {
        return;
      }
      const Eigen::Vector3d rowAxis
          = variationalLoopAxis(constraint.linearAxes, row.axis);
      updateStiffness(
          scratch.linearInventory[linearCursor++],
          rowAxis.dot(positionResidual));
    }

    if (!constraint.rigid) {
      angularCursor += constraint.angularRows.size();
      continue;
    }
    const Eigen::Matrix3d rotA
        = worldRotation(constraint.linkA, constraint.rotationA);
    const Eigen::Matrix3d rotB
        = worldRotation(constraint.linkB, constraint.rotationB);
    const Eigen::Vector3d angularResidual
        = rotB * rotationLog3(rotB.transpose() * rotA);
    for (const auto& row : constraint.angularRows) {
      if (angularCursor >= scratch.angularInventory.size()) {
        return;
      }
      const Eigen::Vector3d rowAxis
          = variationalLoopAxis(constraint.angularAxes, row.axis);
      updateStiffness(
          scratch.angularInventory[angularCursor++],
          rowAxis.dot(angularResidual));
    }
  }
}

// Gather the generalized velocity and the applied (forcing-side) generalized
// effort: commanded effort (Force actuator, clamped to limits) minus passive
// spring and damping forces. Coriolis/gravity are handled by the integrator.
void gatherState(
    const detail::WorldRegistry& registry,
    const VarTree& tree,
    Eigen::VectorXd& position,
    Eigen::VectorXd& velocity,
    Eigen::VectorXd& appliedForce)
{
  const auto dof = static_cast<Eigen::Index>(tree.dofCount);
  position.resize(dof);
  velocity.resize(dof);
  appliedForce.resize(dof);
  position.setZero();
  velocity.setZero();
  appliedForce.setZero();
  for (const auto& link : tree.links) {
    if (link.dof == 0) {
      continue;
    }
    const auto& joint = registry.get<comps::Joint>(link.joint);
    const auto seg = static_cast<Eigen::Index>(link.dofOffset);
    const auto n = static_cast<Eigen::Index>(link.dof);
    position.segment(seg, n) = joint.position;
    velocity.segment(seg, n) = joint.velocity;

    auto effort = appliedForce.segment(seg, n);
    effort.setZero();
    if (joint.actuatorType == comps::ActuatorType::Force
        && joint.torque.size() == n) {
      effort = joint.torque;
      if (joint.limits.effortLower.size() == n
          && joint.limits.effortUpper.size() == n) {
        effort = effort.cwiseMax(joint.limits.effortLower)
                     .cwiseMin(joint.limits.effortUpper);
      }
    }
    if (joint.springStiffness.size() == n && joint.restPosition.size() == n) {
      effort -= joint.springStiffness.cwiseProduct(
          joint.position - joint.restPosition);
    }
    if (joint.dampingCoefficient.size() == n) {
      effort -= joint.dampingCoefficient.cwiseProduct(joint.velocity);
    }
  }
}

void gatherVelocity(
    const detail::WorldRegistry& registry,
    const VarTree& tree,
    Eigen::VectorXd& velocity)
{
  const auto dof = static_cast<Eigen::Index>(tree.dofCount);
  velocity.resize(dof);
  velocity.setZero();
  for (const auto& link : tree.links) {
    if (link.dof == 0) {
      continue;
    }
    const auto& joint = registry.get<comps::Joint>(link.joint);
    const auto seg = static_cast<Eigen::Index>(link.dofOffset);
    const auto n = static_cast<Eigen::Index>(link.dof);
    velocity.segment(seg, n) = joint.velocity;
  }
}

bool isVariationalVelocityProjectionJoint(const comps::Joint& joint)
{
  return joint.actuatorType == comps::ActuatorType::Velocity
         && (joint.type == comps::JointType::Revolute
             || joint.type == comps::JointType::Prismatic);
}

Eigen::Index variationalVelocityProjectionRowCount(
    const detail::WorldRegistry& registry, const VarTree& tree)
{
  Eigen::Index rows = 0;
  for (const auto& link : tree.links) {
    if (link.dof == 0) {
      continue;
    }
    const auto& joint = registry.get<comps::Joint>(link.joint);
    if (!isVariationalVelocityProjectionJoint(joint)) {
      continue;
    }
    rows += static_cast<Eigen::Index>(link.dof);
  }
  return rows;
}

Eigen::Index writeVariationalVelocityProjectionRows(
    const detail::WorldRegistry& registry,
    const VarTree& tree,
    const Eigen::VectorXd& position,
    const Eigen::VectorXd& nextPosition,
    double timeStep,
    Eigen::Index firstRow,
    Eigen::VectorXd& residual,
    Eigen::MatrixXd& jacobian)
{
  Eigen::Index row = firstRow;
  for (const auto& link : tree.links) {
    if (link.dof == 0) {
      continue;
    }
    const auto& joint = registry.get<comps::Joint>(link.joint);
    if (!isVariationalVelocityProjectionJoint(joint)) {
      continue;
    }

    const auto seg = static_cast<Eigen::Index>(link.dofOffset);
    const auto n = static_cast<Eigen::Index>(link.dof);
    const bool useCommand = joint.commandVelocity.size() == n
                            && joint.commandVelocity.allFinite();
    for (Eigen::Index d = 0; d < n; ++d) {
      if (row >= residual.size()) {
        return row;
      }
      const Eigen::Index dof = seg + d;
      const double command = useCommand ? joint.commandVelocity[d] : 0.0;
      residual[row] = nextPosition[dof] - (position[dof] + timeStep * command);
      jacobian(row, dof) = 1.0;
      ++row;
    }
  }
  return row;
}

//==============================================================================
std::optional<VariationalSolveReport> tryIntegrateSinglePrismaticVariational(
    detail::WorldRegistry& registry,
    const comps::MultibodyStructure& structure,
    const Eigen::Vector3d& gravity,
    double timeStep,
    MultibodyVariationalState& state,
    std::span<const VariationalLoopConstraint> constraints,
    const VariationalContactHook& contactHook,
    const VariationalCompliantLoopScratch* compliantLoopScratch,
    const VariationalGroundContact* groundContact,
    std::span<const double> groundContactDuals)
{
  if (!constraints.empty() || contactHook
      || (compliantLoopScratch != nullptr
          && !compliantLoopScratch->constraints.empty())
      || structure.links.size() != 2u || structure.joints.size() != 1u) {
    return std::nullopt;
  }

  const entt::entity baseEntity = structure.links[0];
  const entt::entity childEntity = structure.links[1];
  const auto& baseLink = registry.get<comps::Link>(baseEntity);
  const auto& childLink = registry.get<comps::Link>(childEntity);
  if (baseLink.parentJoint != entt::null
      || childLink.parentJoint == entt::null) {
    return std::nullopt;
  }

  auto& joint = registry.get<comps::Joint>(childLink.parentJoint);
  if (joint.type != comps::JointType::Prismatic || joint.position.size() != 1
      || joint.velocity.size() != 1 || joint.acceleration.size() != 1
      || joint.parentLink != baseEntity || joint.childLink != childEntity) {
    return std::nullopt;
  }
  if (joint.actuatorType == comps::ActuatorType::Velocity) {
    return std::nullopt;
  }

  Vector6 jointFrameSubspace = Vector6::Zero();
  jointFrameSubspace.tail<3>() = joint.axis;
  const Vector6 linkFrameSubspace
      = adjoint(childLink.transformFromParentJoint.inverse())
        * jointFrameSubspace;
  const Matrix6 inertia = spatialInertia(childLink.mass);
  const double effectiveMass
      = linkFrameSubspace.dot(inertia * linkFrameSubspace);
  if (!(effectiveMass > 0.0) || !std::isfinite(effectiveMass)) {
    return std::nullopt;
  }

  double effort = 0.0;
  if (joint.actuatorType == comps::ActuatorType::Force
      && joint.torque.size() == 1) {
    effort = joint.torque[0];
    if (joint.limits.effortLower.size() == 1
        && joint.limits.effortUpper.size() == 1) {
      effort = std::clamp(
          effort, joint.limits.effortLower[0], joint.limits.effortUpper[0]);
    }
  }
  if (joint.springStiffness.size() == 1 && joint.restPosition.size() == 1) {
    effort -= joint.springStiffness[0]
              * (joint.position[0] - joint.restPosition[0]);
  }
  if (joint.dampingCoefficient.size() == 1) {
    effort -= joint.dampingCoefficient[0] * joint.velocity[0];
  }

  const auto& baseCache = registry.get<comps::FrameCache>(baseEntity);
  const Eigen::Vector3d axisWorld
      = baseCache.worldTransform.linear()
        * childLink.transformFromParentToJoint.linear() * joint.axis;
  const double generalizedGravity = effectiveMass * axisWorld.dot(gravity);
  const double generalizedExternal
      = linkFrameSubspace.dot(childLink.externalForce);
  const double noContactGeneralizedForce
      = effort + generalizedExternal + generalizedGravity;

  const double previousPosition = joint.position[0];
  const double previousVelocity = joint.velocity[0];
  const Eigen::Isometry3d childOffset = childLink.transformFromParentJoint;
  const Eigen::Isometry3d parentToJoint = childLink.transformFromParentToJoint;

  const auto contactGeneralizedForce = [&](double trialPosition) {
    if (groundContact == nullptr) {
      return 0.0;
    }
    if (groundContactDuals.size() != groundContact->points.size()) {
      return std::numeric_limits<double>::quiet_NaN();
    }

    double generalizedForce = 0.0;
    Eigen::Isometry3d jointMotion = Eigen::Isometry3d::Identity();
    jointMotion.translation() = joint.axis * trialPosition;
    const Eigen::Isometry3d childWorld
        = baseCache.worldTransform * parentToJoint * jointMotion * childOffset;
    const Eigen::Vector3d pointVelocity = axisWorld * previousVelocity;
    const double normalVelocity = pointVelocity.dot(groundContact->planeNormal);

    for (std::size_t i = 0; i < groundContact->points.size(); ++i) {
      const VariationalContactPoint& point = groundContact->points[i];
      if (point.linkIndex != 1u) {
        return std::numeric_limits<double>::quiet_NaN();
      }
      const Eigen::Vector3d worldPoint = childWorld * point.localPoint;
      const double signedDistance = groundContact->planeNormal.dot(
          worldPoint - groundContact->planePoint);
      double rawNormal = groundContactDuals[i]
                         + groundContact->stiffness * (-signedDistance);
      if (timeStep > 0.0) {
        rawNormal -= groundContact->dampingCoefficient * normalVelocity;
      }
      const double normalMagnitude = std::max(0.0, rawNormal);
      if (normalMagnitude <= 0.0) {
        continue;
      }
      Eigen::Vector3d contactForce
          = normalMagnitude * groundContact->planeNormal;
      if (groundContact->frictionCoefficient > 0.0 && timeStep > 0.0) {
        const Eigen::Vector3d tangentVelocity
            = pointVelocity - normalVelocity * groundContact->planeNormal;
        const double epsilon = groundContact->frictionRegularization;
        contactForce
            -= groundContact->frictionCoefficient * normalMagnitude
               * tangentVelocity
               / std::sqrt(tangentVelocity.squaredNorm() + epsilon * epsilon);
      }
      generalizedForce += axisWorld.dot(contactForce);
    }
    return generalizedForce;
  };

  double acceleration = noContactGeneralizedForce / effectiveMass;
  double nextPosition = previousPosition + timeStep * previousVelocity
                        + timeStep * timeStep * acceleration;
  if (groundContact != nullptr) {
    constexpr int maxScalarIterations = 12;
    for (int iteration = 0; iteration < maxScalarIterations; ++iteration) {
      const double contactForce = contactGeneralizedForce(nextPosition);
      if (!std::isfinite(contactForce)) {
        return std::nullopt;
      }
      acceleration = (noContactGeneralizedForce + contactForce) / effectiveMass;
      const double candidate = previousPosition + timeStep * previousVelocity
                               + timeStep * timeStep * acceleration;
      if (std::abs(candidate - nextPosition) <= 1e-14) {
        nextPosition = candidate;
        break;
      }
      nextPosition = candidate;
    }
    const double contactForce = contactGeneralizedForce(nextPosition);
    if (!std::isfinite(contactForce)) {
      return std::nullopt;
    }
    acceleration = (noContactGeneralizedForce + contactForce) / effectiveMass;
    nextPosition = previousPosition + timeStep * previousVelocity
                   + timeStep * timeStep * acceleration;
  }

  const double nextVelocity = previousVelocity + timeStep * acceleration;

  joint.position[0] = nextPosition;
  joint.velocity[0] = nextVelocity;
  joint.acceleration[0] = acceleration;

  state.previousDeltaTransform.resize(
      structure.links.size(), Eigen::Isometry3d::Identity());
  state.previousMomentum.resize(structure.links.size(), Vector6::Zero());
  state.previousDeltaTransform[0] = Eigen::Isometry3d::Identity();
  state.previousMomentum[0].setZero();

  Eigen::Isometry3d jointDelta = Eigen::Isometry3d::Identity();
  jointDelta.translation() = joint.axis * (nextPosition - previousPosition);
  state.previousDeltaTransform[1] = childLink.transformFromParentJoint.inverse()
                                    * jointDelta
                                    * childLink.transformFromParentJoint;
  const Vector6 averageVelocity = linkFrameSubspace * nextVelocity;
  state.previousMomentum[1] = dm::dexpInvTranspose(
      timeStep * averageVelocity, inertia * averageVelocity);
  state.bootstrapped = true;

  VariationalSolveReport report;
  report.iterations = 1;
  report.residualNorm = 0.0;
  report.converged = true;
  return report;
}

// Forward velocity recursion at the current configuration: body spatial
// velocity V_i = Ad(T_i^{-1}) V_parent + S_i qdot_i.
template <typename SpatialVelocityVector>
void currentSpatialVelocitiesInto(
    const VarTree& tree,
    const Eigen::VectorXd& velocity,
    SpatialVelocityVector& spatialVelocities)
{
  spatialVelocities.resize(tree.links.size());
  for (std::size_t i = 0; i < tree.links.size(); ++i) {
    const auto& link = tree.links[i];
    if (link.parent < 0) {
      spatialVelocities[i].setZero();
    } else {
      spatialVelocities[i].noalias()
          = adjoint(link.currentRelative.inverse())
            * spatialVelocities[static_cast<std::size_t>(link.parent)];
    }
    if (link.dof > 0) {
      spatialVelocities[i].noalias()
          += link.subspace
             * velocity.segment(
                 static_cast<Eigen::Index>(link.dofOffset),
                 static_cast<Eigen::Index>(link.dof));
    }
  }
}

// Evaluate the forced discrete Euler-Lagrange residual f(qNext) in O(n) via a
// forward (average-velocity) sweep and a backward (momentum/impulse) sweep.
void computeResidualInto(
    const detail::WorldRegistry& registry,
    VarTree& tree,
    const Eigen::VectorXd& nextPosition,
    const MultibodyVariationalState& state,
    const Eigen::Vector3d& gravity,
    double timeStep,
    const Eigen::VectorXd& appliedForce,
    Eigen::VectorXd& residual)
{
  // Forward sweep: relative displacement dT_i and average velocity V_i.
  for (std::size_t i = 0; i < tree.links.size(); ++i) {
    auto& link = tree.links[i];
    if (link.parent < 0) {
      link.deltaTransform = Eigen::Isometry3d::Identity();
      link.averageVelocity.setZero();
      continue;
    }
    const auto& joint = registry.get<comps::Joint>(link.joint);
    const auto seg = nextPosition.segment(
        static_cast<Eigen::Index>(link.dofOffset),
        static_cast<Eigen::Index>(link.dof));
    link.nextRelative
        = link.parentToJoint * jointMotionTransform(joint, seg) * link.offset;
    link.deltaTransform
        = link.currentRelative.inverse()
          * tree.links[static_cast<std::size_t>(link.parent)].deltaTransform
          * link.nextRelative;
    link.averageVelocity = dm::se3Log(link.deltaTransform) / timeStep;
  }

  // Backward sweep: discrete momentum, transmitted impulse, joint residual.
  residual.resize(static_cast<Eigen::Index>(tree.dofCount));
  residual.setZero();
  for (std::size_t reverse = 0; reverse < tree.links.size(); ++reverse) {
    const auto i = tree.links.size() - 1 - reverse;
    auto& link = tree.links[i];
    const Matrix6& g = link.inertia;
    link.momentum = dm::dexpInvTranspose(
        timeStep * link.averageVelocity, g * link.averageVelocity);

    Vector6 force = link.momentum;
    force
        -= dm::dAdT(state.previousDeltaTransform[i], state.previousMomentum[i]);
    // Gravity as a forcing-side spatial impulse (not a Lagrangian potential).
    force -= (g * dm::adInvRLinear(link.worldTransform, gravity)) * timeStep;
    // External link wrench (body frame) on the same forcing side as gravity, so
    // an upward force of m*g cancels gravity.
    force -= link.externalForce * timeStep;
    for (const int child : link.children) {
      force += dm::dAdInvT(
          tree.links[static_cast<std::size_t>(child)].currentRelative,
          tree.links[static_cast<std::size_t>(child)].momentum);
    }
    link.momentum = force; // store transmitted impulse for child accumulation

    if (link.dof > 0) {
      const auto seg = static_cast<Eigen::Index>(link.dofOffset);
      const auto n = static_cast<Eigen::Index>(link.dof);
      residual.segment(seg, n) = link.subspace.transpose() * force
                                 - timeStep * appliedForce.segment(seg, n);
    }
  }
}

// O(n) articulated-body-inertia solve of M(q)^{-1} * b for the precomputed tree
// (Featherstone ABA with zero velocity and gravity): a backward articulated-
// inertia sweep followed by a forward acceleration sweep. This is the
// linear-time RIQN quasi-Newton step (replacing a dense factorization) and
// equals massMatrix.ldlt().solve(b) to machine precision (cross-checked by
// test). Frames follow the [angular; linear] convention: parentToChild maps a
// spatial motion parent->child, its transpose maps a spatial force
// child->parent.
void applyArticulatedInverseMassInto(
    const VarTree& tree,
    const Eigen::VectorXd& b,
    VariationalLinearSolveScratch& scratch,
    Eigen::VectorXd& qddot)
{
  const std::size_t n = tree.links.size();
  reserveVariationalLinearSolveScratch(tree, scratch);
  qddot.resize(static_cast<Eigen::Index>(tree.dofCount));
  qddot.setZero();
  for (std::size_t i = 0; i < n; ++i) {
    scratch.articulated[i] = tree.links[i].inertia;
    scratch.bias[i].setZero();
    scratch.motionToChild[i].setIdentity();
    scratch.spatial[i].setZero();
  }

  // Backward sweep: accumulate articulated inertia and bias toward the root.
  for (std::size_t reverse = 0; reverse < n; ++reverse) {
    const std::size_t i = n - 1 - reverse;
    const VarLink& link = tree.links[i];
    if (link.dof > 0) {
      const auto seg = static_cast<Eigen::Index>(link.dofOffset);
      const auto dof = static_cast<Eigen::Index>(link.dof);
      scratch.forceProjector[i].noalias()
          = scratch.articulated[i] * link.subspace;
      scratch.jointMatrix[i].noalias()
          = link.subspace.transpose() * scratch.forceProjector[i];
      scratch.jointMatrixInverse[i] = scratch.jointMatrix[i].inverse();
      scratch.jointRhs[i] = b.segment(seg, dof);
      scratch.jointWork.head(dof).noalias()
          = link.subspace.transpose() * scratch.bias[i];
      scratch.jointRhs[i] -= scratch.jointWork.head(dof);
    }
    if (link.parent >= 0) {
      Matrix6 ia = scratch.articulated[i];
      Vector6 pa = scratch.bias[i];
      if (link.dof > 0) {
        const auto dof = static_cast<Eigen::Index>(link.dof);
        ia.noalias() -= scratch.forceProjector[i]
                        * scratch.jointMatrixInverse[i]
                        * scratch.forceProjector[i].transpose();
        scratch.jointWork.head(dof).noalias()
            = scratch.jointMatrixInverse[i] * scratch.jointRhs[i];
        pa.noalias() += scratch.forceProjector[i] * scratch.jointWork.head(dof);
      }
      scratch.motionToChild[i] = adjoint(link.currentRelative.inverse());
      const Matrix6 forceToParent = scratch.motionToChild[i].transpose();
      const auto p = static_cast<std::size_t>(link.parent);
      scratch.articulated[p].noalias()
          += forceToParent * ia * scratch.motionToChild[i];
      scratch.bias[p].noalias() += forceToParent * pa;
    }
  }

  // Forward sweep: propagate spatial accelerations and read joint
  // accelerations.
  for (std::size_t i = 0; i < n; ++i) {
    const VarLink& link = tree.links[i];
    if (link.parent < 0) {
      scratch.spatial[i].setZero();
      continue;
    }
    scratch.spatial[i].noalias()
        = scratch.motionToChild[i]
          * scratch.spatial[static_cast<std::size_t>(link.parent)];
    if (link.dof > 0) {
      const auto seg = static_cast<Eigen::Index>(link.dofOffset);
      const auto dof = static_cast<Eigen::Index>(link.dof);
      scratch.jointWork.head(dof) = scratch.jointRhs[i];
      scratch.jointSolveWork.head(dof).noalias()
          = scratch.forceProjector[i].transpose() * scratch.spatial[i];
      scratch.jointWork.head(dof) -= scratch.jointSolveWork.head(dof);
      scratch.jointSolveWork.head(dof).noalias()
          = scratch.jointMatrixInverse[i] * scratch.jointWork.head(dof);
      qddot.segment(seg, dof) = scratch.jointSolveWork.head(dof);
      scratch.spatial[i].noalias()
          += link.subspace * scratch.jointSolveWork.head(dof);
    }
  }
}

Eigen::VectorXd applyArticulatedInverseMass(
    const VarTree& tree, const Eigen::VectorXd& b)
{
  VariationalLinearSolveScratch scratch;
  applyArticulatedInverseMassInto(tree, b, scratch, scratch.result);
  return scratch.result;
}

// O(n) exact recursive-Jacobian (Newton) step: solve J(q^k+1) * dq = b for the
// forced discrete Euler-Lagrange Jacobian J = d(residual)/d(q^{k+1}), via a
// non-symmetric articulated-body recursion (a backward articulated-operator
// sweep then a forward delta-twist sweep, the exact analog of
// `applyArticulatedInverseMass`). `b` is the residual; the returned dq is the
// Newton increment to *subtract* from the current iterate.
//
// Why this is exact and O(n): with the residual built by `computeResidual`, the
// only dependence on q^{k+1} flows through the per-link average velocity
// `Vbar_i = log(dT_i)/dt`; the force-transport operators (Ad of the q^k
// relative transforms), the previous-step momentum-transport term, and the
// gravity wrench are all constants w.r.t. q^{k+1}. Linearizing about the
// current iterate:
//   forward (motion):  xi_i = Ad(nextRel_i^{-1}) xi_parent + S_i dq_i,
//                      dVbar_i = (1/dt) Jr_i xi_i,   Jr_i = dexp^{-1}_right(dt
//                      Vbar_i)
//   backward (force):  dP_i  = D_i dVbar_i + sum_c Ad(relK_c^{-1})^T dP_c,
//   joint residual:    df_a  = S_a^T dP_a,
// where `xi_i` is the body-frame perturbation of the relative transform dT_i,
// `nextRel_i` is the relative transform at q^{k+1} (the motion transport, NOT
// the q^k relative), `relK_c` is the q^k relative (the constant force
// transport), and `D_i = dmu_i/dVbar_i` is the exact sensitivity of the
// discrete-momentum map `mu(V) = dexpInvTranspose(dt V, G V)`. Defining the
// per-link effective operator K_i = (1/dt) D_i Jr_i (maps a body-frame motion
// perturbation to a body-frame force), the linear map dq -> df is articulated
// exactly like the mass matrix, with G_i replaced by the (generally
// non-symmetric) K_i and the motion/force transports kept distinct. The
// articulated factorization below therefore solves it in O(n) without any dense
// factorization, and reduces to `applyArticulatedInverseMass` (up to the dt
// scaling) in the small-step limit Jr_i -> I, D_i -> G_i (dt Vbar -> 0).
// Keeping D_i exact (rather than its leading dexp^{-1}(dt Vbar)^T G term)
// matters on stiff mid-rollout configurations: with the leading-term-only D the
// full Newton step overshoots there and the damped iteration stalls, whereas
// the exact D makes alpha = 1 accepted with quadratic convergence regardless of
// chain length.
//
// Scope: Euclidean joints (revolute/prismatic, single-DOF here). The motion
// subspace S_i is the link-frame joint subspace (a 6-vector for A1 joints).
void applyExactNewtonStepInto(
    const VarTree& tree,
    const Eigen::VectorXd& b,
    double timeStep,
    VariationalLinearSolveScratch& scratch,
    Eigen::VectorXd& dq)
{
  const std::size_t n = tree.links.size();
  reserveVariationalLinearSolveScratch(tree, scratch);
  dq.resize(static_cast<Eigen::Index>(tree.dofCount));
  dq.setZero();
  for (std::size_t i = 0; i < n; ++i) {
    scratch.bias[i].setZero();
    scratch.motionToChild[i].setIdentity();
    scratch.spatial[i].setZero();
  }

  // Per-link effective operator K_i = (1/dt) D_i Jr_i, where
  //   D_i = dmu_i/dVbar_i is the *exact* sensitivity of the discrete momentum
  //         mu(V) = dexpInvTranspose(dt V, G V) (so the full step is a true
  //         Newton step, not just the leading dexpInv^T G term -- the dropped
  //         second-order curvature is what made the undamped step overshoot and
  //         stall on stiff mid-rollout configurations), and
  //   Jr_i  = dexp^{-1}_right(dt Vbar_i) is the velocity sensitivity.
  // D_i is formed by a 6-column central difference of the exact momentum map --
  // an O(1)-per-link analytic-operator derivative consistent with the
  // residual's own `dexpInvTranspose`, so the whole sweep stays O(n).
  for (std::size_t i = 0; i < n; ++i) {
    const VarLink& link = tree.links[i];
    const Vector6 velocity = link.averageVelocity;
    const Matrix6& g = link.inertia;
    Matrix6 momentumSensitivity;
    constexpr double eps = 1e-7;
    for (int c = 0; c < 6; ++c) {
      Vector6 plus = velocity;
      Vector6 minus = velocity;
      plus[c] += eps;
      minus[c] -= eps;
      momentumSensitivity.col(c)
          = (dm::dexpInvTranspose(timeStep * plus, g * plus)
             - dm::dexpInvTranspose(timeStep * minus, g * minus))
            / (2.0 * eps);
    }
    const Matrix6 velocitySensitivity
        = dm::dexpInvMatrixRight(timeStep * velocity);
    scratch.articulated[i]
        = (1.0 / timeStep) * momentumSensitivity * velocitySensitivity;
  }

  // Backward sweep: accumulate the articulated operator and bias toward the
  // root (non-symmetric rank-1 elimination: the force-side projector is
  // forceProjector_i = Pi_i S_i and the motion-side row is S_i^T Pi_i, so the
  // update subtracts (Pi_i S_i) D^{-1} (S_i^T Pi_i); for a symmetric Pi_i this
  // is the usual U D^{-1} U^T).
  for (std::size_t reverse = 0; reverse < n; ++reverse) {
    const std::size_t i = n - 1 - reverse;
    const VarLink& link = tree.links[i];
    if (link.dof > 0) {
      const auto seg = static_cast<Eigen::Index>(link.dofOffset);
      const auto dof = static_cast<Eigen::Index>(link.dof);
      scratch.forceProjector[i].noalias()
          = scratch.articulated[i] * link.subspace; // 6 x dof
      scratch.motionProjector[i].noalias()
          = scratch.articulated[i].transpose() * link.subspace; // 6 x dof
      scratch.jointMatrix[i].noalias()
          = link.subspace.transpose() * scratch.forceProjector[i];
      scratch.jointMatrixInverse[i] = scratch.jointMatrix[i].inverse();
      scratch.jointRhs[i] = b.segment(seg, dof);
      scratch.jointWork.head(dof).noalias()
          = link.subspace.transpose() * scratch.bias[i];
      scratch.jointRhs[i] -= scratch.jointWork.head(dof);
    }
    if (link.parent >= 0) {
      Matrix6 ia = scratch.articulated[i];
      Vector6 pa = scratch.bias[i];
      if (link.dof > 0) {
        const auto dof = static_cast<Eigen::Index>(link.dof);
        // Non-symmetric rank-1 elimination: force-side projector Pi_i S_i times
        // the motion-side row S_i^T Pi_i (= motionProjector_i^T). For the
        // symmetric mass-matrix limit (D_i -> G_i) this reduces to the standard
        // U D^{-1} U^T of `applyArticulatedInverseMass`.
        ia.noalias() -= scratch.forceProjector[i]
                        * scratch.jointMatrixInverse[i]
                        * (scratch.motionProjector[i].transpose());
        scratch.jointWork.head(dof).noalias()
            = scratch.jointMatrixInverse[i] * scratch.jointRhs[i];
        pa.noalias() += scratch.forceProjector[i] * scratch.jointWork.head(dof);
      }
      // Distinct transports: motion parent->child uses the q^{k+1} relative;
      // force child->parent uses the q^k relative (the constant force transport
      // of the residual's backward sweep).
      scratch.motionToChild[i] = adjoint(link.nextRelative.inverse());
      const Matrix6 forceToParent
          = adjoint(link.currentRelative.inverse()).transpose();
      const auto p = static_cast<std::size_t>(link.parent);
      scratch.articulated[p].noalias()
          += forceToParent * ia * scratch.motionToChild[i];
      scratch.bias[p].noalias() += forceToParent * pa;
    }
  }

  // Forward sweep: propagate the body-frame delta-twist xi and read the joint
  // increments dq.
  for (std::size_t i = 0; i < n; ++i) {
    const VarLink& link = tree.links[i];
    if (link.parent < 0) {
      scratch.spatial[i].setZero();
      continue;
    }
    scratch.spatial[i].noalias()
        = scratch.motionToChild[i]
          * scratch.spatial[static_cast<std::size_t>(link.parent)];
    if (link.dof > 0) {
      const auto seg = static_cast<Eigen::Index>(link.dofOffset);
      const auto dof = static_cast<Eigen::Index>(link.dof);
      // Eliminated joint equation: D_i dq_i = uForce_i - S_i^T Pi_i
      // parentTwist. The parent-twist coupling is the motion-side row S_i^T
      // Pi_i
      // (= motionProjector_i^T, matching the backward sweep's elimination), not
      // the force-side S_i^T Pi_i^T. They coincide only for a symmetric
      // articulated operator; for finite-rotation / high-average-twist links
      // Pi_i is non-symmetric and the force-side row solves the wrong system.
      scratch.jointWork.head(dof) = scratch.jointRhs[i];
      scratch.jointSolveWork.head(dof).noalias()
          = scratch.motionProjector[i].transpose() * scratch.spatial[i];
      scratch.jointWork.head(dof) -= scratch.jointSolveWork.head(dof);
      scratch.jointSolveWork.head(dof).noalias()
          = scratch.jointMatrixInverse[i] * scratch.jointWork.head(dof);
      dq.segment(seg, dof) = scratch.jointSolveWork.head(dof);
      scratch.spatial[i].noalias()
          += link.subspace * scratch.jointSolveWork.head(dof);
    }
  }
}

template <typename JacobianVector>
void resizeBodyJacobianScratch(
    JacobianVector& jacobians,
    const std::size_t linkCount,
    const Eigen::Index dof)
{
  if (jacobians.size() != linkCount) {
    jacobians.resize(linkCount);
  }
  for (auto& jacobian : jacobians) {
    jacobian.setZero(6, dof);
  }
}

Eigen::Index constraintRowCount(
    std::span<const VariationalLoopConstraint> constraints)
{
  Eigen::Index rows = 0;
  for (const auto& constraint : constraints) {
    rows += variationalLoopConstraintRowCount(constraint);
  }
  return rows;
}

void reserveConstraintProjectionScratch(
    const VarTree& tree,
    Eigen::Index rows,
    VariationalConstraintProjectionScratch& scratch)
{
  const auto linkCount = tree.links.size();
  const auto dof = static_cast<Eigen::Index>(tree.dofCount);
  resizeBodyJacobianScratch(scratch.jacobians, linkCount, dof);
  scratch.pointJacobianA.resize(3, dof);
  scratch.pointJacobianB.resize(3, dof);
  scratch.angularJacobianA.resize(3, dof);
  scratch.angularJacobianB.resize(3, dof);
  scratch.pointJacobianWork.resize(3, dof);
  scratch.residual.resize(rows);
  scratch.jacobian.resize(rows, dof);
  scratch.inverseMassJt.resize(dof, rows);
  scratch.constraintMass.resize(rows, rows);
  if (scratch.constraintFactorization.rows() != rows) {
    scratch.constraintFactorization = Eigen::LDLT<Eigen::MatrixXd>(rows);
  }
  scratch.lambdaRhs.resize(rows);
  scratch.lambda.resize(rows);
  scratch.correction.resize(dof);
  scratch.projectionBounds.reserve(static_cast<std::size_t>(rows));
  scratch.projectionBounds.resize(static_cast<std::size_t>(rows));
  scratch.hardRows.reserve(static_cast<std::size_t>(rows));
  scratch.boundedRows.reserve(static_cast<std::size_t>(rows));
  scratch.hardConstraintMass.resize(rows, rows);
  if (scratch.hardConstraintFactorization.rows() != rows) {
    scratch.hardConstraintFactorization = Eigen::LDLT<Eigen::MatrixXd>(rows);
  }
  scratch.hardConstraintRhs.resize(rows);
  scratch.hardConstraintLambda.resize(rows);
  scratch.projectionLambda.resize(rows);
}

template <typename JacobianVector>
void bodyJacobiansInto(const VarTree& tree, JacobianVector& jacobians)
{
  const auto dof = static_cast<Eigen::Index>(tree.dofCount);
  resizeBodyJacobianScratch(jacobians, tree.links.size(), dof);
  for (std::size_t i = 0; i < tree.links.size(); ++i) {
    const VarLink& link = tree.links[i];
    if (link.parent >= 0) {
      jacobians[i].noalias()
          = adjoint(link.currentRelative.inverse())
            * jacobians[static_cast<std::size_t>(link.parent)];
    }
    if (link.dof > 0) {
      jacobians[i].middleCols(
          static_cast<Eigen::Index>(link.dofOffset),
          static_cast<Eigen::Index>(link.dof)) = link.subspace;
    }
  }
}

template <typename TransformVector, typename JacobianVector>
void bodyJacobiansInto(
    const VarTree& tree,
    const TransformVector& relativeTransforms,
    JacobianVector& jacobians)
{
  const auto dof = static_cast<Eigen::Index>(tree.dofCount);
  resizeBodyJacobianScratch(jacobians, tree.links.size(), dof);
  for (std::size_t i = 0; i < tree.links.size(); ++i) {
    const VarLink& link = tree.links[i];
    if (link.parent >= 0) {
      jacobians[i].noalias()
          = adjoint(relativeTransforms[i].inverse())
            * jacobians[static_cast<std::size_t>(link.parent)];
    }
    if (link.dof > 0) {
      jacobians[i].middleCols(
          static_cast<Eigen::Index>(link.dofOffset),
          static_cast<Eigen::Index>(link.dof)) = link.subspace;
    }
  }
}

void reserveContactEvaluationScratch(
    const VarTree& tree, VariationalContactEvaluationScratch& scratch)
{
  const auto linkCount = tree.links.size();
  const auto dof = static_cast<Eigen::Index>(tree.dofCount);
  scratch.previousWorldTransforms.resize(linkCount);
  scratch.trialRelativeTransforms.resize(linkCount);
  scratch.trialWorldTransforms.resize(linkCount);
  resizeBodyJacobianScratch(scratch.previousJacobians, linkCount, dof);
  resizeBodyJacobianScratch(scratch.trialJacobians, linkCount, dof);
  scratch.contactForce.resize(dof);
  scratch.forcing.resize(dof);
}

// World-frame translational Jacobian (3 x dofCount) of a body-fixed point at
// local offset `point` on link `i`: J_world = R_i (J_linear - [point]
// J_angular).
template <typename JacobianVector>
void worldPointJacobianInto(
    const VarTree& tree,
    const JacobianVector& jacobians,
    std::size_t i,
    const Eigen::Vector3d& point,
    VariationalConstraintProjectionScratch& scratch,
    Eigen::MatrixXd& output)
{
  const Eigen::Matrix3d rotation = tree.links[i].worldTransform.linear();
  scratch.pointJacobianWork.noalias()
      = skew(point) * jacobians[i].template topRows<3>();
  output.noalias()
      = jacobians[i].template bottomRows<3>() - scratch.pointJacobianWork;
  scratch.pointJacobianWork = output;
  output.noalias() = rotation * scratch.pointJacobianWork;
}

template <typename JacobianVector>
void worldAngularJacobianInto(
    const VarTree& tree,
    const JacobianVector& jacobians,
    std::size_t i,
    Eigen::MatrixXd& output)
{
  output.noalias() = tree.links[i].worldTransform.linear()
                     * jacobians[i].template topRows<3>();
}

// Stack the holonomic residual g(q) and Jacobian J = dg/dq for the loop
// closures on a tree built at the current configuration.
template <typename JacobianVector>
void constraintResidualAndJacobianInto(
    const comps::MultibodyStructure& structure,
    const VarTree& tree,
    const JacobianVector& jacobians,
    std::span<const VariationalLoopConstraint> constraints,
    VariationalConstraintProjectionScratch& scratch,
    double timeStep = 0.0,
    Eigen::Index storageRows = -1,
    Eigen::Index rowOffset = 0)
{
  const auto& links = structure.links;
  const auto indexOf = [&](entt::entity e) -> int {
    const auto it = std::find(links.begin(), links.end(), e);
    return it == links.end() ? -1 : static_cast<int>(it - links.begin());
  };
  const auto dof = static_cast<Eigen::Index>(tree.dofCount);
  const Eigen::Index rows = constraintRowCount(constraints);
  const Eigen::Index totalRows = storageRows >= 0 ? storageRows : rows;
  scratch.pointJacobianA.resize(3, dof);
  scratch.pointJacobianB.resize(3, dof);
  scratch.angularJacobianA.resize(3, dof);
  scratch.angularJacobianB.resize(3, dof);
  scratch.pointJacobianWork.resize(3, dof);
  scratch.residual.resize(totalRows);
  scratch.jacobian.resize(totalRows, dof);
  scratch.inverseMassJt.resize(dof, totalRows);
  scratch.constraintMass.resize(totalRows, totalRows);
  scratch.lambdaRhs.resize(totalRows);
  scratch.lambda.resize(totalRows);
  scratch.correction.resize(dof);
  Eigen::VectorXd& g = scratch.residual;
  Eigen::MatrixXd& jac = scratch.jacobian;
  jac.setZero();

  Eigen::Index row = rowOffset;
  for (const auto& c : constraints) {
    const int ia = (c.linkA != entt::null) ? indexOf(c.linkA) : -1;
    const int ib = (c.linkB != entt::null) ? indexOf(c.linkB) : -1;

    Eigen::Vector3d pointA = c.pointA;
    scratch.pointJacobianA.setZero();
    if (ia >= 0) {
      pointA
          = tree.links[static_cast<std::size_t>(ia)].worldTransform * c.pointA;
      worldPointJacobianInto(
          tree,
          jacobians,
          static_cast<std::size_t>(ia),
          c.pointA,
          scratch,
          scratch.pointJacobianA);
    }
    Eigen::Vector3d pointB = c.pointB;
    scratch.pointJacobianB.setZero();
    if (ib >= 0) {
      pointB
          = tree.links[static_cast<std::size_t>(ib)].worldTransform * c.pointB;
      worldPointJacobianInto(
          tree,
          jacobians,
          static_cast<std::size_t>(ib),
          c.pointB,
          scratch,
          scratch.pointJacobianB);
    }

    if (c.distance) {
      const Eigen::Vector3d offset = pointA - pointB;
      const double dist = offset.norm();
      const Eigen::Vector3d dir = dist > 1e-12 ? Eigen::Vector3d(offset / dist)
                                               : Eigen::Vector3d::UnitX();
      g[row] = dist - c.length;
      scratch.pointJacobianWork.noalias()
          = scratch.pointJacobianA - scratch.pointJacobianB;
      jac.row(row).noalias() = dir.transpose() * scratch.pointJacobianWork;
      row += 1;
      continue;
    }

    const Eigen::Vector3d positionResidual = pointA - pointB;
    scratch.pointJacobianWork.noalias()
        = scratch.pointJacobianA - scratch.pointJacobianB;
    for (std::uint8_t axis = 0; axis < 3u; ++axis) {
      if (!dvbd::detail::avbdRigidJointAxisEnabled(c.linearAxisMask, axis)) {
        continue;
      }
      const Eigen::Vector3d rowAxis = variationalLoopAxis(c.linearAxes, axis);
      g[row] = rowAxis.dot(positionResidual);
      jac.row(row).noalias() = rowAxis.transpose() * scratch.pointJacobianWork;
      row += 1;
    }
    if (c.linearMotor) {
      const Eigen::Vector3d rowAxis = normalizedVariationalLoopAxis(
          c.linearMotorAxis, Eigen::Vector3d::UnitX());
      const double targetPosition = c.linearMotorReferencePosition
                                    + c.linearMotorTargetSpeed * timeStep;
      g[row] = -rowAxis.dot(positionResidual) - targetPosition;
      jac.row(row).noalias() = -rowAxis.transpose() * scratch.pointJacobianWork;
      row += 1;
    }

    if (c.rigid) {
      // Orientation rows: world-frame rotation residual R_B * log(R_B^T R_A)
      // (matching LoopClosure::computeResidual) and the relative angular
      // Jacobian J_w_A - J_w_B, where the world angular Jacobian of an endpoint
      // is R_link * (body angular Jacobian). A rigid offset adds no angular
      // velocity, so only the link rotation enters the Jacobian.
      const auto worldRotation
          = [&](int i, const Eigen::Matrix3d& offset) -> Eigen::Matrix3d {
        Eigen::Matrix3d linkRotation = Eigen::Matrix3d::Identity();
        if (i >= 0) {
          linkRotation
              = tree.links[static_cast<std::size_t>(i)].worldTransform.linear();
        }
        return linkRotation * offset;
      };
      const Eigen::Matrix3d rotA = worldRotation(ia, c.rotationA);
      const Eigen::Matrix3d rotB = worldRotation(ib, c.rotationB);
      const Eigen::Vector3d angularResidual
          = rotB * rotationLog3(rotB.transpose() * rotA);

      scratch.angularJacobianA.setZero();
      scratch.angularJacobianB.setZero();
      if (ia >= 0) {
        worldAngularJacobianInto(
            tree,
            jacobians,
            static_cast<std::size_t>(ia),
            scratch.angularJacobianA);
      }
      if (ib >= 0) {
        worldAngularJacobianInto(
            tree,
            jacobians,
            static_cast<std::size_t>(ib),
            scratch.angularJacobianB);
      }
      scratch.pointJacobianWork.noalias()
          = scratch.angularJacobianA - scratch.angularJacobianB;
      for (std::uint8_t axis = 0; axis < 3u; ++axis) {
        if (!dvbd::detail::avbdRigidJointAxisEnabled(c.angularAxisMask, axis)) {
          continue;
        }
        const Eigen::Vector3d rowAxis
            = variationalLoopAxis(c.angularAxes, axis);
        g[row] = rowAxis.dot(angularResidual);
        jac.row(row).noalias()
            = rowAxis.transpose() * scratch.pointJacobianWork;
        row += 1;
      }
      if (c.angularMotor) {
        const Eigen::Vector3d rowAxis = normalizedVariationalLoopAxis(
            c.angularMotorAxis, Eigen::Vector3d::UnitZ());
        const double targetPosition = c.angularMotorReferencePosition
                                      + c.angularMotorTargetSpeed * timeStep;
        g[row] = -rowAxis.dot(angularResidual) - targetPosition;
        jac.row(row).noalias()
            = -rowAxis.transpose() * scratch.pointJacobianWork;
        row += 1;
      }
    }
  }
}

bool groundContactPointForceInto(
    const VariationalContactContext& context,
    const VariationalGroundContact& contact,
    const VariationalContactPoint& point,
    double dual,
    Eigen::VectorXd& generalizedForce);

void evaluateVariationalCompliantLoopForceInto(
    const VariationalContactContext& context,
    const VariationalCompliantLoopScratch& scratch,
    Eigen::VectorXd& generalizedForce);

void evaluateContactForceInto(
    const detail::WorldRegistry& registry,
    const VarTree& tree,
    const Eigen::VectorXd& nextPosition,
    double timeStep,
    const Eigen::VectorXd& previousVelocity,
    const VariationalContactHook& contactHook,
    const VariationalCompliantLoopScratch* compliantLoopScratch,
    const VariationalGroundContact* groundContact,
    const VariationalGroundContactSolver* groundContactSolver,
    VariationalContactEvaluationScratch& scratch,
    Eigen::VectorXd& contactForce)
{
  const bool hasCompliantLoopForce
      = compliantLoopScratch != nullptr
        && !compliantLoopScratch->constraints.empty();
  if (groundContact == nullptr && groundContactSolver == nullptr && !contactHook
      && !hasCompliantLoopForce) {
    contactForce.resize(0);
    return;
  }

  const auto linkCount = tree.links.size();
  if (scratch.previousWorldTransforms.size() != linkCount
      || scratch.trialRelativeTransforms.size() != linkCount
      || scratch.trialWorldTransforms.size() != linkCount
      || scratch.contactForce.size() != static_cast<Eigen::Index>(tree.dofCount)
      || scratch.forcing.size() != static_cast<Eigen::Index>(tree.dofCount)) {
    reserveContactEvaluationScratch(tree, scratch);
  }

  for (std::size_t i = 0; i < linkCount; ++i) {
    const VarLink& link = tree.links[i];
    scratch.previousWorldTransforms[i] = link.worldTransform;
    scratch.trialRelativeTransforms[i] = link.currentRelative;
    scratch.trialWorldTransforms[i] = link.worldTransform;
  }
  bodyJacobiansInto(tree, scratch.previousJacobians);

  for (std::size_t i = 0; i < linkCount; ++i) {
    const VarLink& link = tree.links[i];
    if (link.parent < 0) {
      continue; // root keeps its fixed world transform from buildVarTree.
    }
    const auto& joint = registry.get<comps::Joint>(link.joint);
    const auto seg = nextPosition.segment(
        static_cast<Eigen::Index>(link.dofOffset),
        static_cast<Eigen::Index>(link.dof));
    scratch.trialRelativeTransforms[i]
        = link.parentToJoint * jointMotionTransform(joint, seg) * link.offset;
    scratch.trialWorldTransforms[i]
        = scratch.trialWorldTransforms[static_cast<std::size_t>(link.parent)]
          * scratch.trialRelativeTransforms[i];
  }
  bodyJacobiansInto(
      tree, scratch.trialRelativeTransforms, scratch.trialJacobians);

  const VariationalContactContext context{
      std::span<const Eigen::Isometry3d>{
          scratch.trialWorldTransforms.data(),
          scratch.trialWorldTransforms.size()},
      std::span<const Eigen::MatrixXd>{
          scratch.trialJacobians.data(), scratch.trialJacobians.size()},
      tree.dofCount,
      std::span<const Eigen::Isometry3d>{
          scratch.previousWorldTransforms.data(),
          scratch.previousWorldTransforms.size()},
      std::span<const Eigen::MatrixXd>{
          scratch.previousJacobians.data(), scratch.previousJacobians.size()},
      previousVelocity,
      timeStep};
  contactForce.setZero(static_cast<Eigen::Index>(tree.dofCount));
  if (groundContactSolver != nullptr) {
    groundContactSolver->computeForceInto(context, contactForce);
  } else if (groundContact != nullptr) {
    for (const VariationalContactPoint& point : groundContact->points) {
      groundContactPointForceInto(
          context, *groundContact, point, /*dual=*/0.0, contactForce);
    }
  }
  if (hasCompliantLoopForce) {
    evaluateVariationalCompliantLoopForceInto(
        context, *compliantLoopScratch, contactForce);
  }
  if (contactHook) {
    const Eigen::VectorXd hookForce = contactHook(context);
    DART_SIMULATION_THROW_T_IF(
        hookForce.size() != static_cast<Eigen::Index>(tree.dofCount),
        InvalidOperationException,
        "Variational contact hook returned a generalized force of the wrong "
        "dimension");
    contactForce += hookForce;
  }

  DART_SIMULATION_THROW_T_IF(
      contactForce.size() != static_cast<Eigen::Index>(tree.dofCount),
      InvalidOperationException,
      "Variational contact hook returned a generalized force of the wrong "
      "dimension");
}

} // namespace

//==============================================================================
struct MultibodyVariationalTreeScratch::Impl
{
  explicit Impl(dart::common::MemoryAllocator& allocator)
    : tree(allocator), linkIndexOf(makeLinkIndexMap(allocator))
  {
  }

  VarTree tree;
  LinkIndexMap linkIndexOf;
};

//==============================================================================
MultibodyVariationalTreeScratch::MultibodyVariationalTreeScratch()
  : MultibodyVariationalTreeScratch(dart::common::MemoryAllocator::GetDefault())
{
}

//==============================================================================
MultibodyVariationalTreeScratch::MultibodyVariationalTreeScratch(
    dart::common::MemoryAllocator& allocator)
  : m_allocator(&allocator), m_impl(nullptr, ImplDeleter{&allocator})
{
}

//==============================================================================
MultibodyVariationalTreeScratch::~MultibodyVariationalTreeScratch() = default;

//==============================================================================
MultibodyVariationalTreeScratch::MultibodyVariationalTreeScratch(
    MultibodyVariationalTreeScratch&&) noexcept = default;

//==============================================================================
MultibodyVariationalTreeScratch& MultibodyVariationalTreeScratch::operator=(
    MultibodyVariationalTreeScratch&&) noexcept = default;

//==============================================================================
void MultibodyVariationalTreeScratch::ImplDeleter::operator()(
    Impl* impl) const noexcept
{
  if (impl == nullptr) {
    return;
  }
  auto& targetAllocator = allocator != nullptr
                              ? *allocator
                              : dart::common::MemoryAllocator::GetDefault();
  targetAllocator.destroy(impl);
}

//==============================================================================
void MultibodyVariationalTreeScratch::setAllocator(
    dart::common::MemoryAllocator& allocator)
{
  if (m_allocator == &allocator) {
    return;
  }
  m_impl.reset();
  m_allocator = &allocator;
  m_impl.get_deleter().allocator = &allocator;
}

//==============================================================================
const dart::common::MemoryAllocator&
MultibodyVariationalTreeScratch::getAllocator() const noexcept
{
  return m_allocator != nullptr ? *m_allocator
                                : dart::common::MemoryAllocator::GetDefault();
}

//==============================================================================
std::size_t MultibodyVariationalTreeScratch::linkCount() const noexcept
{
  return m_impl == nullptr ? 0u : m_impl->tree.links.size();
}

//==============================================================================
std::size_t MultibodyVariationalTreeScratch::dofCount() const noexcept
{
  return m_impl == nullptr ? 0u : m_impl->tree.dofCount;
}

//==============================================================================
struct MultibodyVariationalTreeScratchAccess
{
  static auto& ensure(MultibodyVariationalTreeScratch& scratch)
  {
    if (scratch.m_impl == nullptr) {
      auto& allocator = scratch.m_allocator != nullptr
                            ? *scratch.m_allocator
                            : dart::common::MemoryAllocator::GetDefault();
      auto* impl = allocator.construct<MultibodyVariationalTreeScratch::Impl>(
          allocator);
      if (impl == nullptr) {
        throw std::bad_alloc();
      }
      scratch.m_impl.reset(impl);
    }
    return *scratch.m_impl;
  }
};

namespace {

VarTree& buildVarTreeIntoScratch(
    MultibodyVariationalTreeScratch& scratch,
    const detail::WorldRegistry& registry,
    const comps::MultibodyStructure& structure)
{
  auto& storage = MultibodyVariationalTreeScratchAccess::ensure(scratch);
  buildVarTreeInto(registry, structure, storage.tree, storage.linkIndexOf);
  return storage.tree;
}

} // namespace

//==============================================================================
Eigen::VectorXd variationalContactPointForce(
    const VariationalContactContext& context,
    std::size_t linkIndex,
    const Eigen::Vector3d& localPoint,
    const Eigen::Vector3d& worldForce)
{
  // World-frame translational point Jacobian J(p) = R_i (J_linear - [p]
  // J_angular) of the body-fixed point, then the generalized force J(p)^T F.
  const Eigen::Matrix3d rotation
      = context.linkWorldTransforms[linkIndex].linear();
  const Eigen::MatrixXd& jacobian = context.linkBodyJacobians[linkIndex];
  const Eigen::MatrixXd angular = jacobian.topRows<3>();
  const Eigen::MatrixXd linear = jacobian.bottomRows<3>();
  const Eigen::MatrixXd worldPointJacobian
      = rotation * (linear - detail::skew(localPoint) * angular);
  return worldPointJacobian.transpose() * worldForce;
}

namespace {

void variationalContactPointForceInto(
    const VariationalContactContext& context,
    std::size_t linkIndex,
    const Eigen::Vector3d& localPoint,
    const Eigen::Vector3d& worldForce,
    Eigen::VectorXd& generalizedForce)
{
  const Eigen::Matrix3d rotation
      = context.linkWorldTransforms[linkIndex].linear();
  const Eigen::MatrixXd& jacobian = context.linkBodyJacobians[linkIndex];
  const Eigen::Vector3d bodyForce = rotation.transpose() * worldForce;
  generalizedForce.noalias()
      += jacobian.bottomRows<3>().transpose() * bodyForce;
  generalizedForce.noalias()
      -= jacobian.topRows<3>().transpose()
         * (detail::skew(localPoint).transpose() * bodyForce);
}

void variationalWorldTorqueInto(
    const VariationalContactContext& context,
    int linkIndex,
    const Eigen::Vector3d& worldTorque,
    Eigen::VectorXd& generalizedForce)
{
  if (linkIndex < 0) {
    return;
  }
  const auto index = static_cast<std::size_t>(linkIndex);
  DART_SIMULATION_THROW_T_IF(
      index >= context.linkWorldTransforms.size(),
      InvalidOperationException,
      "Compliant variational loop constraint references an out-of-range link "
      "index");
  const Eigen::Vector3d bodyTorque
      = context.linkWorldTransforms[index].linear().transpose() * worldTorque;
  generalizedForce.noalias()
      += context.linkBodyJacobians[index].topRows<3>().transpose() * bodyTorque;
}

void evaluateVariationalCompliantLoopForceInto(
    const VariationalContactContext& context,
    const VariationalCompliantLoopScratch& scratch,
    Eigen::VectorXd& generalizedForce)
{
  if (scratch.constraints.empty()) {
    return;
  }

  const auto worldPoint = [&](int linkIndex, const Eigen::Vector3d& point) {
    if (linkIndex < 0) {
      return point;
    }
    const auto index = static_cast<std::size_t>(linkIndex);
    DART_SIMULATION_THROW_T_IF(
        index >= context.linkWorldTransforms.size(),
        InvalidOperationException,
        "Compliant variational loop constraint references an out-of-range "
        "link index");
    return context.linkWorldTransforms[index] * point;
  };
  const auto worldRotation
      = [&](int linkIndex, const Eigen::Matrix3d& offset) -> Eigen::Matrix3d {
    if (linkIndex < 0) {
      return offset;
    }
    const auto index = static_cast<std::size_t>(linkIndex);
    DART_SIMULATION_THROW_T_IF(
        index >= context.linkWorldTransforms.size(),
        InvalidOperationException,
        "Compliant variational loop constraint references an "
        "out-of-range link index");
    return context.linkWorldTransforms[index].linear() * offset;
  };

  for (const VariationalCompliantLoopConstraint& constraint :
       scratch.constraints) {
    const Eigen::Vector3d pointA
        = worldPoint(constraint.linkA, constraint.pointA);
    const Eigen::Vector3d pointB
        = worldPoint(constraint.linkB, constraint.pointB);
    const Eigen::Vector3d positionResidual = pointA - pointB;
    for (const auto& row : constraint.linearRows) {
      if (row.stiffness <= 0.0 || !std::isfinite(row.stiffness)) {
        continue;
      }
      const Eigen::Vector3d rowAxis
          = variationalLoopAxis(constraint.linearAxes, row.axis);
      const Eigen::Vector3d forceA
          = -row.stiffness * rowAxis.dot(positionResidual) * rowAxis;
      if (constraint.linkA >= 0) {
        variationalContactPointForceInto(
            context,
            static_cast<std::size_t>(constraint.linkA),
            constraint.pointA,
            forceA,
            generalizedForce);
      }
      if (constraint.linkB >= 0) {
        variationalContactPointForceInto(
            context,
            static_cast<std::size_t>(constraint.linkB),
            constraint.pointB,
            -forceA,
            generalizedForce);
      }
    }

    if (!constraint.rigid) {
      continue;
    }
    const Eigen::Matrix3d rotA
        = worldRotation(constraint.linkA, constraint.rotationA);
    const Eigen::Matrix3d rotB
        = worldRotation(constraint.linkB, constraint.rotationB);
    const Eigen::Vector3d angularResidual
        = rotB * rotationLog3(rotB.transpose() * rotA);
    for (const auto& row : constraint.angularRows) {
      if (row.stiffness <= 0.0 || !std::isfinite(row.stiffness)) {
        continue;
      }
      const Eigen::Vector3d rowAxis
          = variationalLoopAxis(constraint.angularAxes, row.axis);
      const Eigen::Vector3d torqueA
          = -row.stiffness * rowAxis.dot(angularResidual) * rowAxis;
      variationalWorldTorqueInto(
          context, constraint.linkA, torqueA, generalizedForce);
      variationalWorldTorqueInto(
          context, constraint.linkB, -torqueA, generalizedForce);
    }
  }
}

} // namespace

//==============================================================================
VariationalContactHook makeVariationalGroundContactHook(
    VariationalGroundContact contact)
{
  const double normalNorm = contact.planeNormal.norm();
  DART_SIMULATION_THROW_T_IF(
      normalNorm < 1e-12,
      InvalidOperationException,
      "VariationalGroundContact plane normal must be non-zero");
  contact.planeNormal /= normalNorm;
  DART_SIMULATION_THROW_T_IF(
      contact.stiffness < 0.0,
      InvalidOperationException,
      "VariationalGroundContact stiffness must be non-negative");
  DART_SIMULATION_THROW_T_IF(
      contact.frictionCoefficient < 0.0,
      InvalidOperationException,
      "VariationalGroundContact friction coefficient must be non-negative");
  DART_SIMULATION_THROW_T_IF(
      contact.frictionCoefficient > 0.0
          && contact.frictionRegularization <= 0.0,
      InvalidOperationException,
      "VariationalGroundContact friction regularization must be positive when "
      "friction is enabled");
  DART_SIMULATION_THROW_T_IF(
      contact.dampingCoefficient < 0.0,
      InvalidOperationException,
      "VariationalGroundContact damping coefficient must be non-negative");

  // Compliant (penalty) ground contact: for each body-fixed point penetrating
  // the half-space at the trial configuration, the one-sided quadratic
  // potential E = 1/2 k max(0,-d)^2 contributes the normal force k(-d) n,
  // mapped to a generalized force by the trial-config point Jacobian. Folded
  // into the forced-DEL residual on the forcing side like gravity/applied
  // force.
  return [contact = std::move(contact)](
             const VariationalContactContext& context) -> Eigen::VectorXd {
    Eigen::VectorXd generalizedForce
        = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(context.dofCount));
    if (contact.stiffness == 0.0) {
      return generalizedForce;
    }
    for (const VariationalContactPoint& point : contact.points) {
      DART_SIMULATION_THROW_T_IF(
          point.linkIndex >= context.linkWorldTransforms.size(),
          InvalidOperationException,
          "VariationalGroundContact contact point references an out-of-range "
          "link index");
      const Eigen::Vector3d worldPoint
          = context.linkWorldTransforms[point.linkIndex] * point.localPoint;
      const double signedDistance
          = contact.planeNormal.dot(worldPoint - contact.planePoint);
      if (signedDistance < 0.0) {
        // Lagged (q^k) contact-point world velocity J_world(p) v = R (J_lin -
        // [p] J_ang) v, shared by the Kelvin-Voigt normal damping and the C1
        // friction below. Both are taken from q^k, so the per-point force stays
        // constant across the step's RIQN iterates -- smooth for the root-find.
        Eigen::Vector3d pointVelocity = Eigen::Vector3d::Zero();
        const bool useVelocity = (contact.dampingCoefficient > 0.0
                                  || contact.frictionCoefficient > 0.0)
                                 && context.timeStep > 0.0;
        if (useVelocity) {
          const Eigen::Matrix3d previousRotation
              = context.previousLinkWorldTransforms[point.linkIndex].linear();
          const Eigen::MatrixXd& previousJacobian
              = context.previousLinkBodyJacobians[point.linkIndex];
          pointVelocity = previousRotation
                          * (previousJacobian.bottomRows<3>()
                             - detail::skew(point.localPoint)
                                   * previousJacobian.topRows<3>())
                          * context.previousVelocity;
        }
        // C2 compliant normal force with Kelvin-Voigt damping:
        // max(0, k(-d) - c (v . n)) n. The max(0, .) keeps the contact
        // non-adhesive when the point separates faster than the penalty pushes.
        const double normalMagnitude = std::max(
            0.0,
            contact.stiffness * (-signedDistance)
                - contact.dampingCoefficient
                      * pointVelocity.dot(contact.planeNormal));
        if (normalMagnitude <= 0.0) {
          continue;
        }
        Eigen::Vector3d contactForce = normalMagnitude * contact.planeNormal;
        // C1 lagged friction: a regularized Coulomb force opposing the contact
        // point's lagged (q^k) sliding velocity, bounded by mu times the
        // *lagged* normal magnitude k(-d^k) (not the trial normal), so the
        // friction force is constant across the step's RIQN iterates --
        // converging like the frictionless case while saturating at mu*|Fn|.
        if (contact.frictionCoefficient > 0.0 && context.timeStep > 0.0) {
          const Eigen::Vector3d previousPoint
              = context.previousLinkWorldTransforms[point.linkIndex]
                * point.localPoint;
          const double previousDistance
              = contact.planeNormal.dot(previousPoint - contact.planePoint);
          if (previousDistance < 0.0) {
            const double laggedNormal = contact.stiffness * (-previousDistance);
            const Eigen::Vector3d tangentVelocity
                = pointVelocity
                  - contact.planeNormal.dot(pointVelocity)
                        * contact.planeNormal;
            const double epsilon = contact.frictionRegularization;
            contactForce
                -= contact.frictionCoefficient * laggedNormal * tangentVelocity
                   / std::sqrt(
                       tangentVelocity.squaredNorm() + epsilon * epsilon);
          }
        }
        variationalContactPointForceInto(
            context,
            point.linkIndex,
            point.localPoint,
            contactForce,
            generalizedForce);
      }
    }
    return generalizedForce;
  };
}

namespace {

void ensureVariationalGroundContactAllocator(
    VariationalGroundContact& contact, dart::common::MemoryAllocator& allocator)
{
  const VariationalGroundContact::PointAllocator targetAllocator{allocator};
  if (contact.points.get_allocator() == targetAllocator) {
    return;
  }

  VariationalGroundContact::PointVector points{targetAllocator};
  points.assign(contact.points.begin(), contact.points.end());
  contact.points = std::move(points);
}

void ensureVariationalContactEvaluationScratchAllocator(
    VariationalContactEvaluationScratch& scratch,
    dart::common::MemoryAllocator& allocator)
{
  const VariationalContactEvaluationScratch::TransformAllocator
      targetTransformAllocator{allocator};
  const auto rebindTransforms = [&](auto& transforms) {
    if (transforms.get_allocator() == targetTransformAllocator) {
      return;
    }
    std::decay_t<decltype(transforms)> rebound{targetTransformAllocator};
    rebound.assign(transforms.begin(), transforms.end());
    transforms = std::move(rebound);
  };
  rebindTransforms(scratch.previousWorldTransforms);
  rebindTransforms(scratch.trialRelativeTransforms);
  rebindTransforms(scratch.trialWorldTransforms);

  const VariationalContactEvaluationScratch::JacobianAllocator
      targetJacobianAllocator{allocator};
  const auto rebindJacobians = [&](auto& jacobians) {
    if (jacobians.get_allocator() == targetJacobianAllocator) {
      return;
    }
    std::decay_t<decltype(jacobians)> rebound{targetJacobianAllocator};
    rebound.assign(jacobians.begin(), jacobians.end());
    jacobians = std::move(rebound);
  };
  rebindJacobians(scratch.previousJacobians);
  rebindJacobians(scratch.trialJacobians);
}

void ensureVariationalLinearSolveScratchAllocator(
    VariationalLinearSolveScratch& scratch,
    dart::common::MemoryAllocator& allocator)
{
  using Scratch = VariationalLinearSolveScratch;
  const Scratch::Matrix6Allocator matrix6Allocator{allocator};
  rebindVectorAllocator(scratch.articulated, matrix6Allocator);
  rebindVectorAllocator(scratch.motionToChild, matrix6Allocator);

  const Scratch::Vector6Allocator vector6Allocator{allocator};
  rebindVectorAllocator(scratch.bias, vector6Allocator);
  rebindVectorAllocator(scratch.spatial, vector6Allocator);

  const Scratch::MatrixAllocator matrixAllocator{allocator};
  rebindVectorAllocator(scratch.forceProjector, matrixAllocator);
  rebindVectorAllocator(scratch.motionProjector, matrixAllocator);
  rebindVectorAllocator(scratch.jointMatrix, matrixAllocator);
  rebindVectorAllocator(scratch.jointMatrixInverse, matrixAllocator);

  const Scratch::VectorAllocator vectorAllocator{allocator};
  rebindVectorAllocator(scratch.jointRhs, vectorAllocator);
}

void ensureVariationalStepScratchAllocator(
    VariationalStepScratch& scratch, dart::common::MemoryAllocator& allocator)
{
  const VariationalStepScratch::SpatialVelocityAllocator targetAllocator{
      allocator};
  rebindVectorAllocator(scratch.currentSpatialVelocities, targetAllocator);
}

void ensureVariationalProjectionScratchAllocator(
    VariationalConstraintProjectionScratch& scratch,
    dart::common::MemoryAllocator& allocator)
{
  const VariationalConstraintProjectionScratch::JacobianAllocator
      jacobianAllocator{allocator};
  rebindVectorAllocator(scratch.jacobians, jacobianAllocator);

  const VariationalConstraintProjectionScratch::ProjectionBoundsAllocator
      boundsAllocator{allocator};
  rebindVectorAllocator(scratch.projectionBounds, boundsAllocator);

  const VariationalConstraintProjectionScratch::RowIndexAllocator rowAllocator{
      allocator};
  rebindVectorAllocator(scratch.hardRows, rowAllocator);
  rebindVectorAllocator(scratch.boundedRows, rowAllocator);
}

void ensureVariationalAndersonScratchAllocator(
    VariationalAndersonScratch& scratch,
    dart::common::MemoryAllocator& allocator)
{
  const VariationalAndersonScratch::VectorAllocator targetAllocator{allocator};
  rebindVectorAllocator(scratch.stepDeltas, targetAllocator);
  rebindVectorAllocator(scratch.iterateDeltas, targetAllocator);
}

void ensureVariationalPostContactTransformsAllocator(
    MultibodyVariationalScratch& scratch,
    dart::common::MemoryAllocator& allocator)
{
  const MultibodyVariationalScratch::PostContactTransformAllocator
      targetAllocator{allocator};
  rebindVectorAllocator(scratch.postContactTransforms, targetAllocator);
}

void ensureVariationalConstraintsAllocator(
    MultibodyVariationalScratch& scratch,
    dart::common::MemoryAllocator& allocator)
{
  const MultibodyVariationalScratch::ConstraintAllocator targetAllocator{
      allocator};
  rebindVectorAllocator(scratch.constraints, targetAllocator);
}

// Validate + normalize a ground-contact config (shared by the AL solver).
void normalizeGroundContact(VariationalGroundContact& contact)
{
  const double normalNorm = contact.planeNormal.norm();
  DART_SIMULATION_THROW_T_IF(
      normalNorm < 1e-12,
      InvalidOperationException,
      "VariationalGroundContact plane normal must be non-zero");
  contact.planeNormal /= normalNorm;
  DART_SIMULATION_THROW_T_IF(
      contact.stiffness < 0.0,
      InvalidOperationException,
      "VariationalGroundContact stiffness must be non-negative");
  DART_SIMULATION_THROW_T_IF(
      contact.frictionCoefficient < 0.0,
      InvalidOperationException,
      "VariationalGroundContact friction coefficient must be non-negative");
  DART_SIMULATION_THROW_T_IF(
      contact.frictionCoefficient > 0.0
          && contact.frictionRegularization <= 0.0,
      InvalidOperationException,
      "VariationalGroundContact friction regularization must be positive when "
      "friction is enabled");
  DART_SIMULATION_THROW_T_IF(
      contact.dampingCoefficient < 0.0,
      InvalidOperationException,
      "VariationalGroundContact damping coefficient must be non-negative");
}

void configureGroundContactScratch(
    const comps::VariationalContact& config,
    MultibodyVariationalScratch& scratch,
    dart::common::MemoryAllocator& allocator)
{
  auto& contact = scratch.groundContact;
  ensureVariationalGroundContactAllocator(contact, allocator);
  contact.planeNormal = config.planeNormal;
  contact.planePoint = config.planePoint;
  contact.stiffness = config.stiffness;
  contact.frictionCoefficient = config.frictionCoefficient;
  contact.frictionRegularization = config.frictionRegularization;
  contact.dampingCoefficient = config.dampingCoefficient;
  contact.points.clear();
  contact.points.reserve(config.pointLinkIndices.size());
  for (std::size_t i = 0; i < config.pointLinkIndices.size(); ++i) {
    contact.points.push_back(
        {config.pointLinkIndices[i], config.pointLocalPositions[i]});
  }
}

// Per-contact-point generalized force at the trial config: the augmented-
// Lagrangian normal force max(0, dual + k(-d)) plus C1 lagged regularized-
// Coulomb friction bounded by that normal magnitude (the friction direction is
// taken at q^k, so it stays smooth across the RIQN iterates). `dual = 0`
// recovers the C2 compliant penalty. Empty result when inactive (force <= 0).
bool groundContactPointForceInto(
    const VariationalContactContext& context,
    const VariationalGroundContact& contact,
    const VariationalContactPoint& point,
    double dual,
    Eigen::VectorXd& generalizedForce)
{
  DART_SIMULATION_THROW_T_IF(
      point.linkIndex >= context.linkWorldTransforms.size(),
      InvalidOperationException,
      "VariationalGroundContact contact point references an out-of-range link "
      "index");
  const Eigen::Vector3d worldPoint
      = context.linkWorldTransforms[point.linkIndex] * point.localPoint;
  const double signedDistance
      = contact.planeNormal.dot(worldPoint - contact.planePoint);
  double rawNormal = dual + contact.stiffness * (-signedDistance);
  // Lagged q^k contact-point world velocity J_world(p) v, shared by the
  // Kelvin-Voigt normal damping and the Coulomb friction (both lagged, so the
  // per-point force stays constant across the step's RIQN iterates).
  Eigen::Vector3d tangentVelocity = Eigen::Vector3d::Zero();
  const bool useVelocity
      = (contact.dampingCoefficient > 0.0 || contact.frictionCoefficient > 0.0)
        && context.timeStep > 0.0;
  if (useVelocity) {
    const Eigen::Matrix3d previousRotation
        = context.previousLinkWorldTransforms[point.linkIndex].linear();
    const Eigen::MatrixXd& previousJacobian
        = context.previousLinkBodyJacobians[point.linkIndex];
    const Eigen::Vector3d pointVelocity
        = previousRotation
          * (previousJacobian.bottomRows<3>()
             - detail::skew(point.localPoint) * previousJacobian.topRows<3>())
          * context.previousVelocity;
    // Kelvin-Voigt normal damping: resist the approach velocity (the max(0,...)
    // below keeps the total force non-pulling).
    rawNormal
        -= contact.dampingCoefficient * pointVelocity.dot(contact.planeNormal);
    tangentVelocity
        = pointVelocity
          - contact.planeNormal.dot(pointVelocity) * contact.planeNormal;
  }
  const double normalMagnitude = std::max(0.0, rawNormal);
  if (normalMagnitude <= 0.0) {
    return false;
  }
  Eigen::Vector3d contactForce = normalMagnitude * contact.planeNormal;
  if (contact.frictionCoefficient > 0.0 && context.timeStep > 0.0) {
    const double epsilon = contact.frictionRegularization;
    contactForce
        -= contact.frictionCoefficient * normalMagnitude * tangentVelocity
           / std::sqrt(tangentVelocity.squaredNorm() + epsilon * epsilon);
  }
  variationalContactPointForceInto(
      context,
      point.linkIndex,
      point.localPoint,
      contactForce,
      generalizedForce);
  return true;
}

} // namespace

//==============================================================================
VariationalGroundContactSolver::VariationalGroundContactSolver(
    VariationalGroundContact contact)
  : VariationalGroundContactSolver(
        std::move(contact), dart::common::MemoryAllocator::GetDefault())
{
}

//==============================================================================
VariationalGroundContactSolver::VariationalGroundContactSolver(
    VariationalGroundContact contact, dart::common::MemoryAllocator& allocator)
  : m_allocator(&allocator),
    mContact(allocator),
    mDuals(DualAllocator{allocator})
{
  resetContact(contact);
}

//==============================================================================
VariationalContactHook VariationalGroundContactSolver::hook() const
{
  return [this](const VariationalContactContext& context) -> Eigen::VectorXd {
    Eigen::VectorXd generalizedForce;
    computeForceInto(context, generalizedForce);
    return generalizedForce;
  };
}

//==============================================================================
void VariationalGroundContactSolver::setAllocator(
    dart::common::MemoryAllocator& allocator)
{
  if (m_allocator == &allocator) {
    return;
  }

  ensureVariationalGroundContactAllocator(mContact, allocator);
  DualVector duals{DualAllocator{allocator}};
  duals.assign(mDuals.begin(), mDuals.end());
  mDuals = std::move(duals);
  m_allocator = &allocator;
}

//==============================================================================
const dart::common::MemoryAllocator&
VariationalGroundContactSolver::getAllocator() const noexcept
{
  return m_allocator != nullptr ? *m_allocator
                                : dart::common::MemoryAllocator::GetDefault();
}

//==============================================================================
void VariationalGroundContactSolver::computeForceInto(
    const VariationalContactContext& context,
    Eigen::VectorXd& generalizedForce) const
{
  generalizedForce.setZero(static_cast<Eigen::Index>(context.dofCount));
  for (std::size_t i = 0; i < mContact.points.size(); ++i) {
    groundContactPointForceInto(
        context, mContact, mContact.points[i], mDuals[i], generalizedForce);
  }
}

//==============================================================================
void VariationalGroundContactSolver::setDuals(std::span<const double> duals)
{
  DART_SIMULATION_THROW_T_IF(
      duals.size() != mContact.points.size(),
      InvalidOperationException,
      "VariationalGroundContactSolver::setDuals size must match the contact "
      "point count");
  std::copy(duals.begin(), duals.end(), mDuals.begin());
}

//==============================================================================
void VariationalGroundContactSolver::resetContact(
    const VariationalGroundContact& contact)
{
  mContact.planeNormal = contact.planeNormal;
  mContact.planePoint = contact.planePoint;
  mContact.stiffness = contact.stiffness;
  mContact.frictionCoefficient = contact.frictionCoefficient;
  mContact.frictionRegularization = contact.frictionRegularization;
  mContact.dampingCoefficient = contact.dampingCoefficient;
  mContact.points.assign(contact.points.begin(), contact.points.end());
  normalizeGroundContact(mContact);

  if (mDuals.size() != mContact.points.size()) {
    mDuals.assign(mContact.points.size(), 0.0);
  } else {
    std::fill(mDuals.begin(), mDuals.end(), 0.0);
  }
}

//==============================================================================
void VariationalGroundContactSolver::updateDuals(
    std::span<const Eigen::Isometry3d> linkWorldTransforms)
{
  for (std::size_t i = 0; i < mContact.points.size(); ++i) {
    const VariationalContactPoint& point = mContact.points[i];
    DART_SIMULATION_THROW_T_IF(
        point.linkIndex >= linkWorldTransforms.size(),
        InvalidOperationException,
        "VariationalGroundContactSolver::updateDuals link index out of range");
    const Eigen::Vector3d worldPoint
        = linkWorldTransforms[point.linkIndex] * point.localPoint;
    const double signedDistance
        = mContact.planeNormal.dot(worldPoint - mContact.planePoint);
    // Dual ascent: lambda <- max(0, lambda + k(-d)) (= the converged AL force).
    mDuals[i]
        = std::max(0.0, mDuals[i] + mContact.stiffness * (-signedDistance));
  }
}

//==============================================================================
VariationalContactHook makeVariationalLinkSphereContactHook(
    double stiffness,
    double dampingCoefficient,
    std::vector<VariationalSphereContactPair> pairs)
{
  DART_SIMULATION_THROW_T_IF(
      stiffness < 0.0,
      InvalidOperationException,
      "Link-sphere contact stiffness must be non-negative");
  DART_SIMULATION_THROW_T_IF(
      dampingCoefficient < 0.0,
      InvalidOperationException,
      "Link-sphere contact damping coefficient must be non-negative");

  // Compliant sphere-sphere contact between links: for each overlapping pair at
  // the trial configuration, an equal-and-opposite penalty force k*penetration
  // (with lagged Kelvin-Voigt damping along the center line) pushes the two
  // link spheres apart, mapped to a generalized force via both links' point
  // Jacobians.
  return [stiffness, dampingCoefficient, pairs = std::move(pairs)](
             const VariationalContactContext& context) -> Eigen::VectorXd {
    Eigen::VectorXd generalizedForce
        = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(context.dofCount));
    if (stiffness == 0.0) {
      return generalizedForce;
    }
    const std::size_t linkCount = context.linkWorldTransforms.size();
    for (const VariationalSphereContactPair& pair : pairs) {
      DART_SIMULATION_THROW_T_IF(
          pair.linkA >= linkCount || pair.linkB >= linkCount,
          InvalidOperationException,
          "Link-sphere contact pair references an out-of-range link index");
      const Eigen::Vector3d worldCenterA
          = context.linkWorldTransforms[pair.linkA] * pair.centerA;
      const Eigen::Vector3d worldCenterB
          = context.linkWorldTransforms[pair.linkB] * pair.centerB;
      const Eigen::Vector3d delta = worldCenterB - worldCenterA;
      const double distance = delta.norm();
      if (distance < 1e-12) {
        continue; // coincident centers: undefined normal, skip.
      }
      const Eigen::Vector3d normal = delta / distance; // unit, from A to B.
      const double penetration = (pair.radiusA + pair.radiusB) - distance;
      if (penetration <= 0.0) {
        continue; // separated.
      }
      double forceMagnitude = stiffness * penetration;
      if (dampingCoefficient > 0.0 && context.timeStep > 0.0) {
        // Lagged q^k center velocities J_world(c) v, for Kelvin-Voigt damping
        // along the center line (constant across the step's RIQN iterates).
        const Eigen::MatrixXd& jacobianA
            = context.previousLinkBodyJacobians[pair.linkA];
        const Eigen::Vector3d velocityA
            = context.previousLinkWorldTransforms[pair.linkA].linear()
              * (jacobianA.bottomRows<3>()
                 - detail::skew(pair.centerA) * jacobianA.topRows<3>())
              * context.previousVelocity;
        const Eigen::MatrixXd& jacobianB
            = context.previousLinkBodyJacobians[pair.linkB];
        const Eigen::Vector3d velocityB
            = context.previousLinkWorldTransforms[pair.linkB].linear()
              * (jacobianB.bottomRows<3>()
                 - detail::skew(pair.centerB) * jacobianB.topRows<3>())
              * context.previousVelocity;
        const double approachRate = (velocityB - velocityA).dot(normal);
        forceMagnitude
            = std::max(0.0, forceMagnitude - dampingCoefficient * approachRate);
      }
      // Equal and opposite: push A along -n, B along +n.
      variationalContactPointForceInto(
          context,
          pair.linkA,
          pair.centerA,
          -forceMagnitude * normal,
          generalizedForce);
      variationalContactPointForceInto(
          context,
          pair.linkB,
          pair.centerB,
          forceMagnitude * normal,
          generalizedForce);
    }
    return generalizedForce;
  };
}

//==============================================================================
namespace {

VariationalSolveReport integrateMultibodyVariationalImpl(
    detail::WorldRegistry& registry,
    const comps::MultibodyStructure& structure,
    const Eigen::Vector3d& gravity,
    double timeStep,
    MultibodyVariationalState& state,
    int maxIterations,
    double tolerance,
    std::span<const VariationalLoopConstraint> constraints,
    std::size_t andersonDepth,
    const VariationalContactHook& contactHook,
    const VariationalCompliantLoopScratch* compliantLoopScratch,
    const VariationalGroundContact* fastGroundContact,
    const VariationalGroundContactSolver* groundContactSolver,
    VariationalContactEvaluationScratch* contactScratch,
    MultibodyVariationalTreeScratch* treeScratch,
    VariationalStepScratch* stepScratch,
    VariationalLinearSolveScratch* linearSolveScratch,
    MultibodyInverseDynamicsScratch* inverseDynamicsScratch,
    VariationalConstraintProjectionScratch* projectionScratch,
    VariationalAndersonScratch* andersonScratch)
{
  VariationalSolveReport report;
  if (structure.links.empty()) {
    return report;
  }

  if (groundContactSolver == nullptr) {
    std::span<const double> groundContactDuals;
    if (auto fastReport = tryIntegrateSinglePrismaticVariational(
            registry,
            structure,
            gravity,
            timeStep,
            state,
            constraints,
            contactHook,
            compliantLoopScratch,
            fastGroundContact,
            groundContactDuals)) {
      return *fastReport;
    }
  } else {
    const auto& duals = groundContactSolver->duals();
    std::span<const double> groundContactDuals{duals.data(), duals.size()};
    if (auto fastReport = tryIntegrateSinglePrismaticVariational(
            registry,
            structure,
            gravity,
            timeStep,
            state,
            constraints,
            contactHook,
            compliantLoopScratch,
            fastGroundContact,
            groundContactDuals)) {
      return *fastReport;
    }
  }

  MultibodyVariationalTreeScratch localTreeScratch;
  VarTree& tree
      = treeScratch != nullptr
            ? buildVarTreeIntoScratch(*treeScratch, registry, structure)
            : buildVarTreeIntoScratch(localTreeScratch, registry, structure);
  if (tree.dofCount == 0) {
    return report;
  }
  VariationalAndersonScratch localAndersonScratch;
  auto& anderson
      = andersonScratch != nullptr ? *andersonScratch : localAndersonScratch;
  VariationalStepScratch localStepScratch;
  auto& stepScratchStorage
      = stepScratch != nullptr ? *stepScratch : localStepScratch;
  reserveVariationalStepScratch(tree, stepScratchStorage);
  VariationalLinearSolveScratch localLinearSolveScratch;
  auto& linearSolve = linearSolveScratch != nullptr ? *linearSolveScratch
                                                    : localLinearSolveScratch;
  reserveVariationalLinearSolveScratch(tree, linearSolve);
  MultibodyInverseDynamicsScratch localInverseDynamicsScratch;
  auto& inverseDynamics = inverseDynamicsScratch != nullptr
                              ? *inverseDynamicsScratch
                              : localInverseDynamicsScratch;
  VariationalConstraintProjectionScratch localProjectionScratch;
  auto& projectionScratchStorage = projectionScratch != nullptr
                                       ? *projectionScratch
                                       : localProjectionScratch;
  const Eigen::Index loopProjectionRows
      = constraints.empty() ? 0 : constraintRowCount(constraints);
  const Eigen::Index velocityProjectionRows
      = variationalVelocityProjectionRowCount(registry, tree);
  reserveConstraintProjectionScratch(
      tree,
      loopProjectionRows + velocityProjectionRows,
      projectionScratchStorage);

  Eigen::VectorXd& position = stepScratchStorage.position;
  Eigen::VectorXd& velocity = stepScratchStorage.velocity;
  Eigen::VectorXd& appliedForce = stepScratchStorage.appliedForce;
  gatherState(registry, tree, position, velocity, appliedForce);

  // Bootstrap the two-step history from the current generalized velocity: seed
  // dT_prev = exp(dt * V_i) and mu_prev = dexp^{-T}(dt V_i, G V_i) so the first
  // step's momentum-transport term is consistent.
  if (!state.bootstrapped
      || state.previousDeltaTransform.size() != tree.links.size()) {
    currentSpatialVelocitiesInto(
        tree, velocity, stepScratchStorage.currentSpatialVelocities);
    state.previousDeltaTransform.assign(
        tree.links.size(), Eigen::Isometry3d::Identity());
    state.previousMomentum.assign(tree.links.size(), Vector6::Zero());
    for (std::size_t i = 0; i < tree.links.size(); ++i) {
      const Vector6 scaled
          = timeStep * stepScratchStorage.currentSpatialVelocities[i];
      state.previousDeltaTransform[i] = dm::se3Exp(scaled);
      state.previousMomentum[i] = dm::dexpInvTranspose(
          scaled,
          tree.links[i].inertia
              * stepScratchStorage.currentSpatialVelocities[i]);
    }
    state.bootstrapped = true;
  }

  // Initial guess IG3 (semi-implicit Euler from forward dynamics): far closer
  // to the DEL root than explicit Euler, which keeps the RIQN iteration count
  // low and roughly uniform across chain length. The forward-dynamics
  // acceleration ddq = M(q)^{-1}(appliedForce - (C(q,qdot) qdot + g(q))) reuses
  // the O(n) inverse-mass apply, so the initial guess stays linear-time. A1
  // joints are Euclidean, so the guess uses plain vector arithmetic.
  Eigen::VectorXd& zeroAcceleration = stepScratchStorage.zeroAcceleration;
  zeroAcceleration.resize(static_cast<Eigen::Index>(tree.dofCount));
  zeroAcceleration.setZero();
  computeMultibodyInverseDynamicsInto(
      inverseDynamics,
      registry,
      structure,
      gravity,
      zeroAcceleration,
      linearSolve.result);
  linearSolve.rhs = appliedForce;
  linearSolve.rhs -= linearSolve.result;
  applyArticulatedInverseMassInto(
      tree, linearSolve.rhs, linearSolve, linearSolve.result);
  const Eigen::VectorXd& guessAcceleration = linearSolve.result;
  Eigen::VectorXd& nextPosition = stepScratchStorage.nextPosition;
  nextPosition = position;
  for (const auto& link : tree.links) {
    if (link.dof == 0) {
      continue;
    }
    const auto& joint = registry.get<comps::Joint>(link.joint);
    const auto seg = static_cast<Eigen::Index>(link.dofOffset);
    const auto n = static_cast<Eigen::Index>(link.dof);
    linearSolve.jointWork.head(n).noalias()
        = timeStep * velocity.segment(seg, n);
    linearSolve.jointWork.head(n).noalias()
        += timeStep * timeStep * guessAcceleration.segment(seg, n);
    jointRetractInto(
        joint,
        position.segment(seg, n),
        linearSolve.jointWork.head(n),
        nextPosition.segment(seg, n));
  }

  // Root-find policy. Two linear-time preconditioners are available:
  //
  //  - Exact recursive-Jacobian Newton (`applyExactNewtonStep`): solves
  //    J(q^k+1) dq = residual for the *exact* forced-DEL Jacobian J =
  //    d(residual)/d(q^{k+1}) by a non-symmetric articulated-body recursion.
  //    Its convergence rate is configuration-/length-independent (a few
  //    iterations regardless of chain length), so long/stiff chains converge
  //    well within the iteration budget. Requires Euclidean generalized
  //    coordinates (the delta-twist / momentum-sensitivity linearization and
  //    the additive joint update are vector-space operations).
  //
  //  - Fixed quasi-Newton step dt * M(q^k)^{-1} * residual via the O(n)
  //    articulated inverse-mass apply, retracted per joint so
  //    spherical/floating coordinates stay on their SO(3)/SE(3) manifolds. This
  //    is only an approximate inverse Jacobian -- it is well conditioned enough
  //    to converge in a few iterations, but on a long/stiff manifold chain it
  //    reaches a per-step residual plateau and cannot drive the residual below
  //    a length-dependent floor -- but it is manifold-correct, which the exact
  //    step is not yet. It is accelerated by tangent-space Anderson mixing (see
  //    the Anderson history below), which pushes through that plateau so the
  //    manifold path converges to tolerances the plain step cannot reach.
  //
  // Policy: use the exact Newton step whenever every movable joint is Euclidean
  // (revolute/prismatic) -- that is exactly where the long-chain convergence
  // problem lives, and the exact step is strictly more accurate there, so short
  // chains converge identically (the converged DEL root is preconditioner-
  // independent). Spherical/floating coordinates keep the manifold-correct
  // quasi-Newton step, now Anderson-accelerated in the tangent space.
  bool euclideanCoordinates = true;
  for (const auto& link : tree.links) {
    if (link.dof == 0) {
      continue;
    }
    const auto type = registry.get<comps::Joint>(link.joint).type;
    if (type == comps::JointType::Spherical
        || type == comps::JointType::Floating) {
      euclideanCoordinates = false;
      break;
    }
  }
  const std::size_t reservedAndersonDepth
      = (andersonScratch != nullptr || !euclideanCoordinates) ? andersonDepth
                                                              : 0u;
  reserveVariationalAndersonScratch(tree, reservedAndersonDepth, anderson);

  // Retract `base` by `-scale * increment` per joint (the Newton update,
  // damped by `scale`), so spherical/floating coordinates stay on their
  // manifolds. Shared by the line search and the accepted update.
  const auto retractStepInto = [&](const Eigen::VectorXd& base,
                                   const Eigen::VectorXd& increment,
                                   double scale,
                                   Eigen::VectorXd& result) {
    result = base;
    for (const auto& link : tree.links) {
      if (link.dof == 0) {
        continue;
      }
      const auto& joint = registry.get<comps::Joint>(link.joint);
      const auto seg = static_cast<Eigen::Index>(link.dofOffset);
      const auto n = static_cast<Eigen::Index>(link.dof);
      anderson.jointDelta.head(n) = -scale * increment.segment(seg, n);
      jointRetractInto(
          joint,
          base.segment(seg, n),
          anderson.jointDelta.head(n),
          result.segment(seg, n));
    }
  };

  // The tangent displacement from iterate `from` to iterate `to`, stacked per
  // joint: tau_a = jointLogDifference(to_a, from_a) so jointRetract(from_a,
  // tau_a) == to_a. This is the manifold-correct "to - from" the tangent-space
  // Anderson history mixes (a spherical/floating joint's raw coordinate
  // difference is not a tangent vector; its per-joint log is).
  const auto perJointLogDifferenceInto = [&](const Eigen::VectorXd& to,
                                             const Eigen::VectorXd& from,
                                             Eigen::VectorXd& tangent) {
    tangent.setZero();
    for (const auto& link : tree.links) {
      if (link.dof == 0) {
        continue;
      }
      const auto& joint = registry.get<comps::Joint>(link.joint);
      const auto seg = static_cast<Eigen::Index>(link.dofOffset);
      const auto n = static_cast<Eigen::Index>(link.dof);
      jointLogDifferenceInto(
          joint,
          to.segment(seg, n),
          from.segment(seg, n),
          tangent.segment(seg, n));
    }
  };

  // Tangent-space Anderson (type-II) acceleration of the manifold quasi-Newton
  // fixed point. The fixed-point map is q^{k+1} = retract(q^k, -step^k) with
  // the tangent update step^k = dt M(q^k)^{-1} f(q^k); the fixed-point residual
  // in tangent coordinates at the current iterate is exactly step^k (zero at a
  // fixed point). We keep the last m differences of the tangent updates (F
  // columns) and of the iterates' tangent displacements (X columns, the
  // per-joint logs of q^k relative to q^{k-1}), and at each iteration solve
  // gamma = argmin ||step^k - F gamma|| (a small dofCount x m least squares)
  // for the accelerated increment step^k + (X - F) gamma, retracted with
  // jointRetract. Mixing validity: near convergence the consecutive iterates --
  // and thus the base points of every per-joint tangent vector -- are close, so
  // treating the stacked per-joint tangents as one Euclidean vector (no
  // parallel transport) introduces only higher-order error that vanishes as
  // step -> 0; this matched the test without transport (the globalization below
  // also caps any early-iterate excursion). m = 1 recovers Aitken/secant
  // acceleration; the Euclidean exact-Newton path never enters this branch.
  VariationalContactEvaluationScratch localContactScratch;
  auto& contactEvaluation
      = contactScratch != nullptr ? *contactScratch : localContactScratch;

  // EXPERIMENTAL SPIKE (contact-roadmap gate 2): evaluate the forced-DEL
  // residual at a trial configuration, folding the in-loop contact hook's
  // generalized force `Q_c(trial)` onto the same forcing side as the applied
  // force (`residual -= dt * Q_c`). Re-evaluating the hook here is what couples
  // the contact potential's curvature into every RIQN iterate and every
  // line-search trial. With no hook/solver the contact force is empty and the
  // call reduces to the plain `computeResidual(appliedForce)`, so the
  // no-contact path is numerically identical and does no extra work.
  const auto residualAt = [&](const Eigen::VectorXd& trialPosition,
                              Eigen::VectorXd& residual) -> Eigen::VectorXd& {
    evaluateContactForceInto(
        registry,
        tree,
        trialPosition,
        timeStep,
        velocity,
        contactHook,
        compliantLoopScratch,
        fastGroundContact,
        groundContactSolver,
        contactEvaluation,
        contactEvaluation.contactForce);
    if (contactEvaluation.contactForce.size() == 0) {
      computeResidualInto(
          registry,
          tree,
          trialPosition,
          state,
          gravity,
          timeStep,
          appliedForce,
          residual);
      return residual;
    }
    contactEvaluation.forcing = appliedForce;
    contactEvaluation.forcing += contactEvaluation.contactForce;
    computeResidualInto(
        registry,
        tree,
        trialPosition,
        state,
        gravity,
        timeStep,
        contactEvaluation.forcing,
        residual);
    return residual;
  };

  // `tolerance` is a per-coordinate accuracy; the convergence test is on the
  // L2 norm of the (dofCount-dimensional) residual, so scale by sqrt(dofCount)
  // to keep the per-coordinate accuracy uniform across chain lengths. Without
  // this, the fixed norm threshold is sqrt(n) times stricter for an n-DOF
  // system and the hardest steps of long chains stall just above it.
  const double normTolerance
      = tolerance * std::sqrt(static_cast<double>(tree.dofCount));
  for (int iteration = 0; iteration < maxIterations; ++iteration) {
    Eigen::VectorXd& residual
        = residualAt(nextPosition, stepScratchStorage.residual);
    report.iterations = static_cast<std::size_t>(iteration) + 1;
    report.residualNorm = residual.norm();
    const double residualNorm = report.residualNorm;
    if (residualNorm <= normTolerance) {
      report.converged = true;
      break;
    }

    // The exact Newton step already solves J dq = residual, so it is applied
    // directly; the quasi-Newton step uses dt * M^{-1} * residual. Both yield
    // an increment to *subtract* from the current iterate. `computeResidual`
    // populated the per-link average velocity and q^{k+1} relative transforms
    // the exact step needs.
    if (euclideanCoordinates) {
      applyExactNewtonStepInto(
          tree, residual, timeStep, linearSolve, linearSolve.result);
    } else {
      linearSolve.rhs.noalias() = timeStep * residual;
      applyArticulatedInverseMassInto(
          tree, linearSolve.rhs, linearSolve, linearSolve.result);
    }
    const Eigen::VectorXd& step = linearSolve.result;

    // Manifold tangent-space Anderson mixing (spherical/floating path only).
    // Accumulate the step/iterate-displacement history and form the accelerated
    // increment; the Euclidean exact-Newton path keeps `step` unchanged. The
    // least squares is Tikhonov-regularized (a small ridge relative to the
    // column scale) because the step-difference columns become nearly linearly
    // dependent as the iterates settle, and an unregularized type-II solve then
    // produces a wild gamma that throws the increment far off -- the
    // regularizer (plus the opportunistic accept/safeguard below) keeps the
    // acceleration stable on long stiff manifold chains.
    bool haveAnderson = false;
    if (!euclideanCoordinates && andersonDepth > 0 && iteration > 0) {
      const auto depthLimit = anderson.stepDeltas.size();
      std::size_t slot = 0;
      if (anderson.historyCount < depthLimit) {
        slot = anderson.historyCount++;
      } else {
        for (std::size_t i = 1; i < depthLimit; ++i) {
          anderson.stepDeltas[i - 1] = anderson.stepDeltas[i];
          anderson.iterateDeltas[i - 1] = anderson.iterateDeltas[i];
        }
        slot = depthLimit - 1;
      }
      anderson.stepDeltas[slot].noalias() = step - anderson.previousStep;
      perJointLogDifferenceInto(
          nextPosition,
          anderson.previousPosition,
          anderson.iterateDeltas[slot]);
      const auto m = static_cast<Eigen::Index>(anderson.historyCount);
      auto stepMatrix = anderson.stepMatrix.leftCols(m);
      auto mixMatrix = anderson.mixMatrix.leftCols(m);
      for (Eigen::Index c = 0; c < m; ++c) {
        const auto index = static_cast<std::size_t>(c);
        stepMatrix.col(c) = anderson.stepDeltas[index];
        mixMatrix.col(c).noalias()
            = anderson.iterateDeltas[index] - anderson.stepDeltas[index];
      }
      // Solve (F^T F + lambda I) gamma = F^T step (normal equations with a
      // ridge scaled by the column magnitudes) for the type-II mixing
      // coefficients.
      auto ftf = anderson.ftf.topLeftCorner(m, m);
      ftf.noalias() = stepMatrix.transpose() * stepMatrix;
      const double ridge
          = 1e-12 * (ftf.trace() / static_cast<double>(m) + 1e-30);
      auto regularized = anderson.regularized.topLeftCorner(m, m);
      regularized = ftf;
      regularized.diagonal().array() += ridge;
      auto normalRhs = anderson.normalRhs.head(m);
      normalRhs.noalias() = stepMatrix.transpose() * step;
      auto gamma = anderson.gamma.head(m);
      gamma = regularized.ldlt().solve(normalRhs);
      if (gamma.allFinite()) {
        anderson.andersonIncrement = step;
        anderson.andersonIncrement.noalias() += mixMatrix * gamma;
        haveAnderson = anderson.andersonIncrement.allFinite();
      }
    }
    anderson.previousStep = step;
    anderson.previousPosition = nextPosition;

    // Globalization differs by preconditioner because the two steps have
    // different convergence structure.
    //
    // Euclidean exact-Newton path: a damped-Newton backtracking line search.
    // The exact step has fast *local* convergence but an undamped full step can
    // overshoot when an iterate wanders far from the root (the hardest
    // mid-rollout steps of a long/stiff chain reach such states); try the full
    // step first and backtrack by halving until it reduces the residual. Near
    // the root the full step is always accepted (alpha = 1), preserving the
    // quadratic rate; each trial residual is O(n), so a handful of backtracks
    // keeps the step linear-time.
    //
    // Manifold quasi-Newton path: the *undamped* full step. The fixed-point
    // iteration q <- retract(q, -dt M^{-1} f) is contractive but NOT monotone
    // in the residual norm (an intermediate iterate can transiently raise ||f||
    // while still converging), so a residual-decrease line search would
    // wrongly reject legitimate full steps and strangle the iteration to a
    // standstill. Anderson is layered on top as an opportunistic accelerator:
    // accept its full step when it strictly reduces the residual (a guaranteed
    // improvement over the plain iterate this iteration), otherwise take the
    // plain full step. The opportunistic accept never lets a poor Anderson
    // direction make progress worse than the plain fixed point.
    if (euclideanCoordinates) {
      double alpha = 1.0;
      retractStepInto(nextPosition, step, alpha, anderson.trialPosition);
      double trialNorm
          = residualAt(anderson.trialPosition, stepScratchStorage.residual)
                .norm();
      constexpr int kMaxBacktracks = 20;
      for (int backtrack = 0;
           backtrack < kMaxBacktracks && !(trialNorm < residualNorm);
           ++backtrack) {
        alpha *= 0.5;
        retractStepInto(nextPosition, step, alpha, anderson.trialPosition);
        trialNorm
            = residualAt(anderson.trialPosition, stepScratchStorage.residual)
                  .norm();
      }
      nextPosition = anderson.trialPosition;
      continue;
    }

    if (haveAnderson) {
      retractStepInto(
          nextPosition,
          anderson.andersonIncrement,
          1.0,
          anderson.andersonPosition);
      const double andersonNorm
          = residualAt(anderson.andersonPosition, stepScratchStorage.residual)
                .norm();
      if (andersonNorm < residualNorm) {
        nextPosition = anderson.andersonPosition;
        continue;
      }
    }
    retractStepInto(nextPosition, step, 1.0, anderson.trialPosition);
    nextPosition = anderson.trialPosition;
  }

  // Non-convergence is a hard error, not a silent best-effort step: the caller
  // must learn the forced-DEL root was not found rather than integrate a bogus
  // (possibly NaN) configuration forward.
  if (!report.converged) {
    DART_SIMULATION_THROW_T(
        InvalidOperationException,
        "Variational integrator failed to converge: forced discrete "
        "Euler-Lagrange residual {} after {} RIQN iterations exceeds tolerance "
        "{}",
        report.residualNorm,
        report.iterations,
        normTolerance);
  }

  // Enforce holonomic loop closures and velocity-actuated joint coordinates:
  // Newton-project the next configuration onto the constraint manifold g(q) = 0
  // (the paper's Sec. 5 extension), impulse-based and reusing the O(n)
  // inverse-mass apply. Velocity actuators become coordinate targets
  // q^{k+1} = retract(q^k, dt * qdot_command) for the supported one-DOF
  // articulated joints, matching the existing velocity-level actuator contract.
  if (!constraints.empty() || velocityProjectionRows > 0) {
    constexpr double constraintTolerance = 1e-10;
    constexpr int maxProjectionIterations = 32;
    bool projectionLambdaInitialized = false;
    bool treeConfigurationChanged = false;
    for (int projection = 0; projection < maxProjectionIterations;
         ++projection) {
      // Reuse the step tree topology and refresh only the configuration-
      // dependent transforms/Jacobians for the projection candidate. The tree
      // is restored to the step base configuration before final residual
      // evaluation so the DEL history update remains based on q^k -> q^{k+1}.
      updateVarTreeConfigurationInto(registry, tree, nextPosition);
      treeConfigurationChanged = true;
      bodyJacobiansInto(tree, projectionScratchStorage.jacobians);
      const auto dof = static_cast<Eigen::Index>(tree.dofCount);
      const Eigen::Index rows = loopProjectionRows + velocityProjectionRows;
      auto& projectionBounds = projectionScratchStorage.projectionBounds;
      if (constraints.empty()) {
        projectionBounds.clear();
      } else {
        variationalLoopConstraintRowBoundsInto(
            constraints, timeStep, projectionBounds);
      }
      projectionBounds.resize(static_cast<std::size_t>(rows));
      if (!constraints.empty()) {
        constraintResidualAndJacobianInto(
            structure,
            tree,
            projectionScratchStorage.jacobians,
            constraints,
            projectionScratchStorage,
            timeStep,
            rows);
      } else {
        projectionScratchStorage.residual.setZero(rows);
        projectionScratchStorage.jacobian.setZero(rows, dof);
        projectionScratchStorage.inverseMassJt.resize(dof, rows);
        projectionScratchStorage.constraintMass.resize(rows, rows);
        projectionScratchStorage.lambdaRhs.resize(rows);
        projectionScratchStorage.lambda.resize(rows);
        projectionScratchStorage.correction.resize(dof);
      }

      Eigen::VectorXd& g = projectionScratchStorage.residual;
      Eigen::MatrixXd& jacobian = projectionScratchStorage.jacobian;
      writeVariationalVelocityProjectionRows(
          registry,
          tree,
          position,
          nextPosition,
          timeStep,
          loopProjectionRows,
          g,
          jacobian);
      if (g.norm() <= constraintTolerance) {
        break;
      }
      for (Eigen::Index r = 0; r < rows; ++r) {
        linearSolve.rhs = jacobian.row(r).transpose();
        applyArticulatedInverseMassInto(
            tree, linearSolve.rhs, linearSolve, linearSolve.result);
        projectionScratchStorage.inverseMassJt.col(r) = linearSolve.result;
      }
      projectionScratchStorage.constraintMass.noalias()
          = jacobian * projectionScratchStorage.inverseMassJt;
      projectionScratchStorage.constraintFactorization.compute(
          projectionScratchStorage.constraintMass);
      projectionScratchStorage.lambdaRhs = -g;
      projectionScratchStorage.lambda
          = projectionScratchStorage.constraintFactorization.solve(
              projectionScratchStorage.lambdaRhs);
      auto& hardRows = projectionScratchStorage.hardRows;
      auto& boundedRows = projectionScratchStorage.boundedRows;
      hardRows.clear();
      boundedRows.clear();
      for (Eigen::Index r = 0; r < projectionScratchStorage.lambda.size();
           ++r) {
        const auto& bounds = projectionBounds[static_cast<std::size_t>(r)];
        if (variationalProjectionRowHasFiniteBounds(bounds)) {
          boundedRows.push_back(r);
          projectionScratchStorage.lambda[r]
              = clampVariationalProjectionRowForce(
                  projectionScratchStorage.lambda[r], bounds);
        } else {
          hardRows.push_back(r);
        }
      }
      if (!boundedRows.empty() && !hardRows.empty()) {
        // Bounded motor rows may saturate before reaching their target. Keep
        // hard joint/anchor rows authoritative after those impulses are
        // clamped so finite effort cannot leak into locked coordinates.
        const auto hardRowCount = static_cast<Eigen::Index>(hardRows.size());
        auto hardMass
            = projectionScratchStorage.hardConstraintMass.topLeftCorner(
                hardRowCount, hardRowCount);
        auto hardRhs
            = projectionScratchStorage.hardConstraintRhs.head(hardRowCount);
        for (std::size_t i = 0; i < hardRows.size(); ++i) {
          const Eigen::Index hardRow = hardRows[i];
          hardRhs[static_cast<Eigen::Index>(i)] = -g[hardRow];
          for (const Eigen::Index boundedRow : boundedRows) {
            hardRhs[static_cast<Eigen::Index>(i)]
                -= projectionScratchStorage.constraintMass(hardRow, boundedRow)
                   * projectionScratchStorage.lambda[boundedRow];
          }
          for (std::size_t j = 0; j < hardRows.size(); ++j) {
            hardMass(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(j))
                = projectionScratchStorage.constraintMass(hardRow, hardRows[j]);
          }
        }
        projectionScratchStorage.hardConstraintFactorization.compute(hardMass);
        projectionScratchStorage.hardConstraintLambda.head(hardRowCount)
            = projectionScratchStorage.hardConstraintFactorization.solve(
                hardRhs);
        for (std::size_t i = 0; i < hardRows.size(); ++i) {
          projectionScratchStorage.lambda[hardRows[i]]
              = projectionScratchStorage
                    .hardConstraintLambda[static_cast<Eigen::Index>(i)];
        }
      }
      auto& projectionLambda = projectionScratchStorage.projectionLambda;
      if (!projectionLambdaInitialized
          || projectionLambda.size()
                 != projectionScratchStorage.lambda.size()) {
        projectionLambda = projectionScratchStorage.lambda.cwiseAbs();
        projectionLambdaInitialized = true;
      } else {
        projectionLambda = projectionLambda.cwiseMax(
            projectionScratchStorage.lambda.cwiseAbs());
      }
      projectionScratchStorage.correction.noalias()
          = projectionScratchStorage.inverseMassJt
            * projectionScratchStorage.lambda;
      for (const VarLink& link : tree.links) {
        if (link.dof == 0) {
          continue;
        }
        const auto& joint = registry.get<comps::Joint>(link.joint);
        const auto seg = static_cast<Eigen::Index>(link.dofOffset);
        const auto n = static_cast<Eigen::Index>(link.dof);
        jointRetractInto(
            joint,
            nextPosition.segment(seg, n),
            projectionScratchStorage.correction.segment(seg, n),
            anderson.trialPosition.segment(seg, n));
        nextPosition.segment(seg, n) = anderson.trialPosition.segment(seg, n);
      }
    }
    if (projectionLambdaInitialized
        && projectionScratchStorage.projectionLambda.size() > 0) {
      markBrokenAvbdVariationalLoopConstraints(
          registry, constraints, projectionScratchStorage.projectionLambda);
    }
    if (treeConfigurationChanged) {
      updateVarTreeConfigurationInto(registry, tree, position);
    }
  }

  // Refresh the per-link scratch at the accepted configuration so the history
  // shift uses dT and momentum consistent with nextPosition. `residualAt` folds
  // in the contact force (if any) so the reported residual reflects the actual
  // forced-DEL root; the forward (dT/average-velocity) sweep is independent of
  // the forcing, so the history shift is unaffected by the hook.
  report.residualNorm
      = residualAt(nextPosition, stepScratchStorage.residual).norm();

  // Write back joint position/velocity/acceleration (Euclidean, Phase A1).
  bool wroteJointPosition = false;
  for (auto& link : tree.links) {
    if (link.dof == 0) {
      continue;
    }
    wroteJointPosition = true;
    auto& joint = registry.get<comps::Joint>(link.joint);
    const auto seg = static_cast<Eigen::Index>(link.dofOffset);
    const auto n = static_cast<Eigen::Index>(link.dof);
    const bool hasPreviousVelocity = joint.velocity.size() == n;
    if (hasPreviousVelocity) {
      stepScratchStorage.previousJointVelocity.head(n) = joint.velocity;
    }
    // q^k is the captured `position`; loop-closure projection works on the
    // scratch tree and leaves registry joint state unchanged until writeback.
    joint.velocity.resize(n);
    jointLogDifferenceInto(
        joint,
        nextPosition.segment(seg, n),
        position.segment(seg, n),
        joint.velocity);
    joint.velocity /= timeStep;
    joint.position = nextPosition.segment(seg, n);
    if (hasPreviousVelocity) {
      joint.acceleration.resize(n);
      joint.acceleration = joint.velocity;
      joint.acceleration -= stepScratchStorage.previousJointVelocity.head(n);
      joint.acceleration /= timeStep;
    }
  }
  if (wroteJointPosition) {
    markMultibodyLinkFrameCachesDirty(registry, structure);
  }

  // Shift the two-step history: dT_prev <- dT, mu_prev <- discrete momentum.
  // Note: computeResidual stored the transmitted impulse in `momentum`; the
  // discrete momentum we must carry forward is dexp^{-T}(dt V, G V), recomputed
  // here from the accepted average velocity.
  for (std::size_t i = 0; i < tree.links.size(); ++i) {
    const auto& link = tree.links[i];
    state.previousDeltaTransform[i] = link.deltaTransform;
    state.previousMomentum[i] = dm::dexpInvTranspose(
        timeStep * link.averageVelocity, link.inertia * link.averageVelocity);
  }

  return report;
}

} // namespace

//==============================================================================
VariationalSolveReport integrateMultibodyVariational(
    detail::WorldRegistry& registry,
    const comps::MultibodyStructure& structure,
    const Eigen::Vector3d& gravity,
    double timeStep,
    MultibodyVariationalState& state,
    int maxIterations,
    double tolerance,
    const std::vector<VariationalLoopConstraint>& constraints,
    std::size_t andersonDepth,
    const VariationalContactHook& contactHook)
{
  return integrateMultibodyVariationalImpl(
      registry,
      structure,
      gravity,
      timeStep,
      state,
      maxIterations,
      tolerance,
      variationalLoopConstraintSpan(constraints),
      andersonDepth,
      contactHook,
      nullptr,
      nullptr,
      nullptr,
      nullptr,
      nullptr,
      nullptr,
      nullptr,
      nullptr,
      nullptr,
      nullptr);
}

//==============================================================================
double computeMultibodyMechanicalEnergy(
    const detail::WorldRegistry& registry,
    const comps::MultibodyStructure& structure,
    const Eigen::Vector3d& gravity)
{
  MultibodyVariationalTreeScratch treeScratch;
  VariationalStepScratch stepScratch;
  return computeMultibodyMechanicalEnergy(
      registry, structure, gravity, treeScratch, stepScratch);
}

//==============================================================================
double computeMultibodyMechanicalEnergy(
    const detail::WorldRegistry& registry,
    const comps::MultibodyStructure& structure,
    const Eigen::Vector3d& gravity,
    MultibodyVariationalTreeScratch& treeScratch,
    VariationalStepScratch& stepScratch)
{
  if (structure.links.empty()) {
    return 0.0;
  }
  const VarTree& tree
      = buildVarTreeIntoScratch(treeScratch, registry, structure);

  stepScratch.currentSpatialVelocities.resize(tree.links.size());
  gatherVelocity(registry, tree, stepScratch.velocity);
  currentSpatialVelocitiesInto(
      tree, stepScratch.velocity, stepScratch.currentSpatialVelocities);

  double kinetic = 0.0;
  double potential = 0.0;
  for (std::size_t i = 0; i < tree.links.size(); ++i) {
    const auto& link = tree.links[i];
    const Vector6& spatialVelocity = stepScratch.currentSpatialVelocities[i];
    kinetic += 0.5 * spatialVelocity.dot(link.inertia * spatialVelocity);
    const auto& linkComp = registry.get<comps::Link>(structure.links[i]);
    const Eigen::Vector3d comWorld
        = link.worldTransform * linkComp.mass.localCenterOfMass;
    potential -= linkComp.mass.mass * gravity.dot(comWorld);
  }
  return kinetic + potential;
}

//==============================================================================
Eigen::VectorXd computeMultibodyInverseMassProduct(
    detail::WorldRegistry& registry,
    const comps::MultibodyStructure& structure,
    const Eigen::VectorXd& impulse)
{
  if (structure.links.empty()) {
    return {};
  }
  const VarTree tree = buildVarTree(registry, structure);
  if (tree.dofCount == 0) {
    return {};
  }
  DART_SIMULATION_THROW_T_IF(
      impulse.size() != static_cast<Eigen::Index>(tree.dofCount),
      InvalidArgumentException,
      "Impulse dimension must match the multibody movable DOF count");
  return applyArticulatedInverseMass(tree, impulse);
}

//==============================================================================
VariationalConstraintLinearization computeVariationalConstraintLinearization(
    detail::WorldRegistry& registry,
    const comps::MultibodyStructure& structure,
    const std::vector<VariationalLoopConstraint>& constraints)
{
  VariationalConstraintLinearization result;
  if (structure.links.empty() || constraints.empty()) {
    return result;
  }
  const VarTree tree = buildVarTree(registry, structure);
  VariationalConstraintProjectionScratch scratch;
  bodyJacobiansInto(tree, scratch.jacobians);
  constraintResidualAndJacobianInto(
      structure,
      tree,
      scratch.jacobians,
      variationalLoopConstraintSpan(constraints),
      scratch);
  result.residual = std::move(scratch.residual);
  result.jacobian = std::move(scratch.jacobian);
  return result;
}

//==============================================================================
VariationalLoopClosureBinding bindVariationalLoopClosure(
    const detail::WorldRegistry& registry, entt::entity closureEntity)
{
  using Status = VariationalLoopClosureBinding::Status;
  VariationalLoopClosureBinding binding;

  const auto* closure = registry.try_get<comps::LoopClosure>(closureEntity);
  if (closure == nullptr || !closure->runtimePolicy.enabled
      || closure->runtimePolicy.dynamics != ClosureDynamicsPolicy::Solve) {
    return binding; // Ignored: no closure, disabled, or residual-only.
  }

  // Resolve an endpoint frame to its owning multibody structure: entt::null for
  // a world anchor, the structure entity for a link, or report failure for a
  // non-link frame (e.g. a rigid body).
  const auto resolve
      = [&](entt::entity frame, entt::entity& structureOut) -> bool {
    if (frame == entt::null) {
      structureOut = entt::null; // world anchor
      return true;
    }
    if (!registry.all_of<comps::Link>(frame)) {
      return false; // rigid-body or other non-link frame
    }
    for (auto entity : registry.view<comps::MultibodyStructure>()) {
      const auto& structure = registry.get<comps::MultibodyStructure>(entity);
      if (std::find(structure.links.begin(), structure.links.end(), frame)
          != structure.links.end()) {
        structureOut = entity;
        return true;
      }
    }
    return false; // a link not owned by any structure (should not happen)
  };

  entt::entity structureA = entt::null;
  entt::entity structureB = entt::null;
  if (!resolve(closure->frameA, structureA)
      || !resolve(closure->frameB, structureB)) {
    binding.status = Status::Unsupported;
    binding.reason
        = "a variational loop-closure endpoint must be a multibody link or the "
          "world frame (rigid-body endpoints are not supported)";
    return binding;
  }
  if (structureA == entt::null && structureB == entt::null) {
    binding.status = Status::Unsupported;
    binding.reason
        = "a variational loop closure must touch at least one multibody link";
    return binding;
  }
  if (structureA != entt::null && structureB != entt::null
      && structureA != structureB) {
    binding.status = Status::Unsupported;
    binding.reason
        = "a variational loop closure spanning two multibodies is "
          "not supported";
    return binding;
  }

  binding.status = Status::Supported;
  binding.structure = structureA != entt::null ? structureA : structureB;
  // The endpoint expressed in a link body frame is offset.translation() (the
  // frame IS the link body frame; a null link keeps the point in world
  // coordinates as the anchor). Point constrains the 3D offset to zero;
  // Distance constrains the separation to the closure's target distance; Rigid
  // adds the relative-orientation rows and uses the offset rotation.
  const bool isDistance = closure->family == LoopClosureFamily::Distance;
  const bool isRigid = closure->family == LoopClosureFamily::Rigid;
  binding.constraint.linkA = closure->frameA;
  binding.constraint.pointA = closure->offsetA.translation();
  binding.constraint.linkB = closure->frameB;
  binding.constraint.pointB = closure->offsetB.translation();
  binding.constraint.distance = isDistance;
  binding.constraint.length = isDistance ? closure->distance : 0.0;
  binding.constraint.rigid = isRigid;
  binding.constraint.rotationA = closure->offsetA.linear();
  binding.constraint.rotationB = closure->offsetB.linear();
  return binding;
}

//==============================================================================
void reserveMultibodyVariationalRegistryStorage(
    detail::WorldRegistry& registry,
    std::size_t multibodyCount,
    dart::common::MemoryAllocator& allocator)
{
  auto& stateStorage = registry.storage<MultibodyVariationalState>();
  auto& scratchStorage = registry.storage<MultibodyVariationalScratch>();
  stateStorage.reserve(multibodyCount);
  scratchStorage.reserve(multibodyCount);
  auto pointJointConfigs
      = registry.view<comps::Joint, dvbd::AvbdRigidWorldPointJointConfig>();
  const bool hasPointJointConfigs
      = pointJointConfigs.begin() != pointJointConfigs.end();
  if (hasPointJointConfigs) {
    registry.storage<VariationalCompliantLoopScratch>().reserve(multibodyCount);
  }

  if (multibodyCount == 0u) {
    return;
  }

  auto view = registry.view<comps::MultibodyStructure>();
  auto closures = registry.view<comps::LoopClosure>();
  for (auto entity : view) {
    const auto& structure = view.get<comps::MultibodyStructure>(entity);
    auto* existingState = registry.try_get<MultibodyVariationalState>(entity);
    auto& state = existingState != nullptr
                      ? *existingState
                      : registry.emplace<MultibodyVariationalState>(
                            entity, makeMultibodyVariationalState(allocator));
    ensureMultibodyVariationalStateAllocator(state, allocator);
    state.previousDeltaTransform.reserve(structure.links.size());
    state.previousMomentum.reserve(structure.links.size());

    auto& scratch
        = registry.get_or_emplace<MultibodyVariationalScratch>(entity);
    scratch.tree.setAllocator(allocator);
    scratch.inverseDynamics.setAllocator(allocator);
    ensureVariationalContactEvaluationScratchAllocator(
        scratch.contactEvaluation, allocator);
    ensureVariationalStepScratchAllocator(scratch.step, allocator);
    ensureVariationalLinearSolveScratchAllocator(
        scratch.linearSolve, allocator);
    ensureVariationalProjectionScratchAllocator(scratch.projection, allocator);
    ensureVariationalAndersonScratchAllocator(scratch.anderson, allocator);
    ensureVariationalPostContactTransformsAllocator(scratch, allocator);
    ensureVariationalConstraintsAllocator(scratch, allocator);
    scratch.postContactTransforms.resize(structure.links.size());
    scratch.constraints.clear();
    for (auto closureEntity : closures) {
      const auto binding = bindVariationalLoopClosure(registry, closureEntity);
      if (binding.status == VariationalLoopClosureBinding::Status::Supported
          && binding.structure == entity) {
        scratch.constraints.push_back(binding.constraint);
      }
    }
    VariationalCompliantLoopScratch* compliantScratch = nullptr;
    if (hasPointJointConfigs) {
      compliantScratch = &getOrCreateVariationalCompliantLoopScratch(
          registry, entity, allocator);
      compliantScratch->constraints.clear();
    }
    appendAvbdRigidWorldArticulatedPointJointConstraints(
        registry, entity, scratch.constraints, compliantScratch);
    if (compliantScratch != nullptr) {
      if (!compliantScratch->constraints.empty()) {
        syncVariationalCompliantLoopConstraintRows(*compliantScratch);
      } else {
        compliantScratch->clear();
      }
    }
    const Eigen::Index loopProjectionRows = constraintRowCount(
        variationalLoopConstraintSpan(scratch.constraints));
    scratch.constraints.clear();

    const VarTree& tree
        = buildVarTreeIntoScratch(scratch.tree, registry, structure);
    if (tree.dofCount > 0u) {
      const Eigen::Index projectionRows
          = loopProjectionRows
            + variationalVelocityProjectionRowCount(registry, tree);
      reserveVariationalStepScratch(tree, scratch.step);
      reserveVariationalLinearSolveScratch(tree, scratch.linearSolve);
      reserveMultibodyInverseDynamicsScratch(
          scratch.inverseDynamics, registry, structure);
      reserveConstraintProjectionScratch(
          tree, projectionRows, scratch.projection);
      reserveVariationalAndersonScratch(tree, 5, scratch.anderson);
    }

    auto* contactConfig = registry.try_get<comps::VariationalContact>(entity);
    if (contactConfig == nullptr || contactConfig->pointLinkIndices.empty()) {
      continue;
    }
    ensureVariationalContactAllocator(*contactConfig, allocator);

    configureGroundContactScratch(*contactConfig, scratch, allocator);
    if (scratch.groundContact.stiffness <= 0.0
        || scratch.groundContact.points.empty()) {
      continue;
    }
    normalizeGroundContact(scratch.groundContact);
    if (tree.dofCount > 0u) {
      reserveContactEvaluationScratch(tree, scratch.contactEvaluation);
    }
    if (contactConfig->dualUpdateCadence == 0u) {
      scratch.groundContactSolver.reset();
      continue;
    }
    if (!scratch.groundContactSolver.has_value()) {
      scratch.groundContactSolver.emplace(scratch.groundContact, allocator);
    } else {
      scratch.groundContactSolver->setAllocator(allocator);
      scratch.groundContactSolver->resetContact(scratch.groundContact);
    }

    auto& dualState
        = registry.get_or_emplace<comps::VariationalContactDualState>(
            entity, makeVariationalContactDualState(allocator));
    ensureVariationalContactDualStateAllocator(dualState, allocator);
    if (dualState.duals.size() != scratch.groundContact.points.size()) {
      dualState.duals.assign(scratch.groundContact.points.size(), 0.0);
      dualState.stepsSinceDualUpdate = 0;
    }
    scratch.groundContactSolver->setDuals(
        std::span<const double>{
            dualState.duals.data(), dualState.duals.size()});
  }
}

//==============================================================================
std::string_view MultibodyVariationalIntegrationStage::getName() const noexcept
{
  return "multibody_variational_integration";
}

//==============================================================================
ComputeStageMetadata MultibodyVariationalIntegrationStage::getMetadata()
    const noexcept
{
  return {
      ComputeStageDomain::ArticulatedBody,
      toMask(ComputeStageAcceleration::TaskParallel)};
}

//==============================================================================
void MultibodyVariationalIntegrationStage::prepare(World& world)
{
  const auto options = world.getMultibodyOptions();
  m_maxIterations = options.variationalMaxIterations;
  m_tolerance = options.variationalTolerance;

  auto& registry = dart::simulation::detail::registryOf(world);
  auto view = registry.view<comps::MultibodyStructure>();
  std::size_t multibodyCount = 0;
  for (auto entity : view) {
    static_cast<void>(entity);
    ++multibodyCount;
  }
  reserveMultibodyVariationalRegistryStorage(
      registry, multibodyCount, world.getMemoryManager().getFreeAllocator());
}

//==============================================================================
void MultibodyVariationalIntegrationStage::execute(
    World& world, ComputeExecutor& /*executor*/)
{
  auto& registry = dart::simulation::detail::registryOf(world);
  const Eigen::Vector3d gravity = world.getGravity();
  const double timeStep = world.getTimeStep();
  auto& worldFreeAllocator = world.getMemoryManager().getFreeAllocator();

  // Gather enabled loop closures that request dynamic solving, grouped by the
  // multibody they constrain. World::step validates the dynamics policy first,
  // so by here every binding is Ignored or Supported (never Unsupported).
  auto closures = registry.view<comps::LoopClosure>();
  auto pointJointConfigs
      = registry.view<comps::Joint, dvbd::AvbdRigidWorldPointJointConfig>();
  const bool hasPointJointConfigs
      = pointJointConfigs.begin() != pointJointConfigs.end();

  auto view = registry.view<comps::MultibodyStructure>();
  for (auto entity : view) {
    const auto& structure = view.get<comps::MultibodyStructure>(entity);
    auto* existingState = registry.try_get<MultibodyVariationalState>(entity);
    auto& state
        = existingState != nullptr
              ? *existingState
              : registry.emplace<MultibodyVariationalState>(
                    entity, makeMultibodyVariationalState(worldFreeAllocator));
    ensureMultibodyVariationalStateAllocator(state, worldFreeAllocator);
    auto& scratch
        = registry.get_or_emplace<MultibodyVariationalScratch>(entity);
    scratch.tree.setAllocator(worldFreeAllocator);
    scratch.inverseDynamics.setAllocator(worldFreeAllocator);
    ensureVariationalContactEvaluationScratchAllocator(
        scratch.contactEvaluation, worldFreeAllocator);
    ensureVariationalStepScratchAllocator(scratch.step, worldFreeAllocator);
    ensureVariationalLinearSolveScratchAllocator(
        scratch.linearSolve, worldFreeAllocator);
    ensureVariationalProjectionScratchAllocator(
        scratch.projection, worldFreeAllocator);
    ensureVariationalAndersonScratchAllocator(
        scratch.anderson, worldFreeAllocator);
    ensureVariationalPostContactTransformsAllocator(
        scratch, worldFreeAllocator);
    ensureVariationalConstraintsAllocator(scratch, worldFreeAllocator);
    auto& constraints = scratch.constraints;
    constraints.clear();
    for (auto closureEntity : closures) {
      const auto binding = bindVariationalLoopClosure(registry, closureEntity);
      if (binding.status == VariationalLoopClosureBinding::Status::Supported
          && binding.structure == entity) {
        constraints.push_back(binding.constraint);
      }
    }
    VariationalCompliantLoopScratch* compliantScratch = nullptr;
    if (hasPointJointConfigs) {
      compliantScratch = &getOrCreateVariationalCompliantLoopScratch(
          registry, entity, worldFreeAllocator);
      compliantScratch->constraints.clear();
    } else {
      compliantScratch
          = registry.try_get<VariationalCompliantLoopScratch>(entity);
    }
    appendAvbdRigidWorldArticulatedPointJointConstraints(
        registry, entity, constraints, compliantScratch);
    if (compliantScratch != nullptr && !compliantScratch->constraints.empty()) {
      syncVariationalCompliantLoopConstraintRows(*compliantScratch);
    } else if (compliantScratch != nullptr) {
      compliantScratch->clear();
    }
    // Build the opt-in contact hook from the multibody's contact config
    // (PLAN-082 Phase C). cadence 0 => C1/C2 (lagged friction + compliant
    // penalty); cadence > 0 => the stateful C3 augmented-Lagrangian rung, whose
    // duals persist in VariationalContactDualState and advance on an outer-loop
    // cadence after the step. Absent => contact-free.
    auto* contactConfig = registry.try_get<comps::VariationalContact>(entity);
    const VariationalGroundContact* groundContact = nullptr;
    VariationalGroundContactSolver* groundContactSolver = nullptr;
    VariationalGroundContactSolver* alSolver = nullptr;
    std::size_t dualUpdateCadence = 0;
    VariationalContactHook contactHook;
    if (contactConfig != nullptr) {
      ensureVariationalContactAllocator(*contactConfig, worldFreeAllocator);
      configureGroundContactScratch(
          *contactConfig, scratch, worldFreeAllocator);
      const auto& contact = scratch.groundContact;
      if (contact.stiffness > 0.0 && !contact.points.empty()) {
        normalizeGroundContact(scratch.groundContact);
        groundContact = &scratch.groundContact;
        if (contactConfig->dualUpdateCadence > 0) {
          // C3: a stateful AL solver seeded from the persisted (warm-started)
          // duals. The solver must outlive the integrate call and the later
          // dual update, so it is also held in `alSolver` here.
          if (!scratch.groundContactSolver.has_value()) {
            scratch.groundContactSolver.emplace(contact, worldFreeAllocator);
          } else {
            scratch.groundContactSolver->setAllocator(worldFreeAllocator);
            scratch.groundContactSolver->resetContact(contact);
          }
          VariationalGroundContactSolver& solver = *scratch.groundContactSolver;
          groundContactSolver = &solver;
          dualUpdateCadence = contactConfig->dualUpdateCadence;
          auto& dualState
              = registry.get_or_emplace<comps::VariationalContactDualState>(
                  entity,
                  makeVariationalContactDualState(
                      world.getMemoryManager().getFreeAllocator()));
          ensureVariationalContactDualStateAllocator(
              dualState, world.getMemoryManager().getFreeAllocator());
          if (dualState.duals.size() != contact.points.size()) {
            dualState.duals.assign(contact.points.size(), 0.0);
            dualState.stepsSinceDualUpdate = 0;
          }
          solver.setDuals(
              std::span<const double>{
                  dualState.duals.data(), dualState.duals.size()});
          alSolver = &solver;
        } else {
          scratch.groundContactSolver.reset();
        }
      }
    }
    integrateMultibodyVariationalImpl(
        registry,
        structure,
        gravity,
        timeStep,
        state,
        static_cast<int>(m_maxIterations),
        m_tolerance,
        variationalLoopConstraintSpan(constraints),
        5,
        contactHook,
        compliantScratch,
        groundContact,
        groundContactSolver,
        &scratch.contactEvaluation,
        &scratch.tree,
        &scratch.step,
        &scratch.linearSolve,
        &scratch.inverseDynamics,
        &scratch.projection,
        &scratch.anderson);
    if (compliantScratch != nullptr && !compliantScratch->constraints.empty()) {
      updateVariationalCompliantLoopConstraintRows(
          registry, structure, scratch.tree, *compliantScratch);
    }

    // C3 outer loop: after the converged step, advance the AL duals on the
    // configured cadence from the post-step per-link world transforms (a fresh
    // VarTree reads the converged joint positions), then persist them for the
    // next step and for save/load.
    if (alSolver != nullptr) {
      auto& dualState
          = registry.get<comps::VariationalContactDualState>(entity);
      if (++dualState.stepsSinceDualUpdate >= dualUpdateCadence) {
        dualState.stepsSinceDualUpdate = 0;
        const VarTree& postTree
            = buildVarTreeIntoScratch(scratch.tree, registry, structure);
        auto& postTransforms = scratch.postContactTransforms;
        postTransforms.resize(postTree.links.size());
        for (std::size_t i = 0; i < postTree.links.size(); ++i) {
          postTransforms[i] = postTree.links[i].worldTransform;
        }
        alSolver->updateDuals(
            std::span<const Eigen::Isometry3d>{
                postTransforms.data(), postTransforms.size()});
        const auto& solverDuals = alSolver->duals();
        dualState.duals.assign(solverDuals.begin(), solverDuals.end());
      }
    }

    // External forces are one-shot per step (like legacy
    // BodyNode::addExtForce): clear them after they have been consumed by this
    // step's dynamics.
    for (const auto linkEntity : structure.links) {
      registry.get<comps::Link>(linkEntity).externalForce.setZero();
    }
  }
}

} // namespace dart::simulation::compute
