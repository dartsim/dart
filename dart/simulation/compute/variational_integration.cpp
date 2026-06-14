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
#include <optional>
#include <unordered_map>
#include <utility>
#include <vector>

#include <cmath>

namespace dart::simulation::compute {

namespace {

namespace dm = detail::variational;
namespace dvbd = detail::deformable_vbd;

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

// Joint motion subspace in the joint frame (Phase A1), [angular; linear].
Subspace jointSubspaceInJointFrame(const comps::Joint& joint)
{
  switch (joint.type) {
    case comps::JointType::Fixed:
      return Subspace(6, 0);
    case comps::JointType::Revolute: {
      Subspace subspace(6, 1);
      subspace.col(0).head<3>() = joint.axis;
      subspace.col(0).tail<3>().setZero();
      return subspace;
    }
    case comps::JointType::Prismatic: {
      Subspace subspace(6, 1);
      subspace.col(0).head<3>().setZero();
      subspace.col(0).tail<3>() = joint.axis;
      return subspace;
    }
    case comps::JointType::Spherical: {
      // Generalized velocity is the body angular velocity (matching the
      // rotation-vector position): angular = I, linear = 0.
      Subspace subspace = Subspace::Zero(6, 3);
      subspace.topRows<3>() = Eigen::Matrix3d::Identity();
      return subspace;
    }
    case comps::JointType::Floating: {
      // Generalized velocity is [linear; angular] body twist (matching the
      // position layout [translation; rotation vector]); the subspace permutes
      // it into the [angular; linear] spatial convention.
      Subspace subspace = Subspace::Zero(6, 6);
      subspace.bottomLeftCorner<3, 3>() = Eigen::Matrix3d::Identity();
      subspace.topRightCorner<3, 3>() = Eigen::Matrix3d::Identity();
      return subspace;
    }
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
Eigen::VectorXd jointRetract(
    const comps::Joint& joint,
    const Eigen::VectorXd& q,
    const Eigen::VectorXd& delta)
{
  switch (joint.type) {
    case comps::JointType::Spherical:
      return rotationLog3(
          rotationExp3(q.head<3>()) * rotationExp3(delta.head<3>()));
    case comps::JointType::Floating: {
      const Eigen::Matrix3d rotation = rotationExp3(q.tail<3>());
      Eigen::VectorXd result(6);
      result.head<3>() = q.head<3>() + rotation * delta.head<3>();
      result.tail<3>() = rotationLog3(rotation * rotationExp3(delta.tail<3>()));
      return result;
    }
    default:
      return q + delta;
  }
}

// The generalized-coordinate tangent `tau` with `jointRetract(q, tau) == qNext`
// (so the generalized velocity is `tau / dt`). Inverse of jointRetract.
Eigen::VectorXd jointLogDifference(
    const comps::Joint& joint,
    const Eigen::VectorXd& qNext,
    const Eigen::VectorXd& q)
{
  switch (joint.type) {
    case comps::JointType::Spherical:
      return rotationLog3(
          rotationExp3(q.head<3>()).transpose()
          * rotationExp3(qNext.head<3>()));
    case comps::JointType::Floating: {
      const Eigen::Matrix3d rotation = rotationExp3(q.tail<3>());
      Eigen::VectorXd result(6);
      result.head<3>() = rotation.transpose() * (qNext.head<3>() - q.head<3>());
      result.tail<3>()
          = rotationLog3(rotation.transpose() * rotationExp3(qNext.tail<3>()));
      return result;
    }
    default:
      return qNext - q;
  }
}

// Per-link data for the discrete recursion, in construction order
// (parent-before-child).
struct VarLink
{
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
  std::vector<int> children;
  // Per-residual-evaluation scratch.
  Eigen::Isometry3d deltaTransform = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d nextRelative
      = Eigen::Isometry3d::Identity(); // T_{lambda(i),i} at q^{k+1}
  Vector6 averageVelocity = Vector6::Zero();
  Vector6 momentum = Vector6::Zero();
};

struct VarTree
{
  std::vector<VarLink> links;
  std::size_t dofCount = 0;
};

VarTree buildVarTree(
    const detail::WorldRegistry& registry,
    const comps::MultibodyStructure& structure)
{
  const auto& linkEntities = structure.links;
  VarTree tree;
  tree.links.assign(linkEntities.size(), VarLink{});

  // O(1) link-index lookup so building the tree stays O(n); a linear scan per
  // link would make the whole step O(n^2) and defeat the linear-time solve.
  std::unordered_map<entt::entity, std::size_t> indexOf;
  indexOf.reserve(linkEntities.size());
  for (std::size_t i = 0; i < linkEntities.size(); ++i) {
    indexOf.emplace(linkEntities[i], i);
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

    const Subspace jointFrameSubspace = jointSubspaceInJointFrame(joint);
    link.dof = static_cast<std::size_t>(jointFrameSubspace.cols());
    link.dofOffset = tree.dofCount;
    tree.dofCount += link.dof;
    link.subspace = adjoint(link.offset.inverse()) * jointFrameSubspace;
    link.currentRelative = link.parentToJoint
                           * jointMotionTransform(joint, joint.position)
                           * link.offset;
    link.worldTransform
        = tree.links[static_cast<std::size_t>(link.parent)].worldTransform
          * link.currentRelative;
    tree.links[static_cast<std::size_t>(link.parent)].children.push_back(
        static_cast<int>(i));
  }
  return tree;
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

dvbd::AvbdScalarRowBounds variationalMotorProjectionBounds(
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
    const dvbd::AvbdScalarRowBounds& bounds)
{
  return std::isfinite(bounds.lower) || std::isfinite(bounds.upper);
}

std::vector<dvbd::AvbdScalarRowBounds> variationalLoopConstraintRowBounds(
    const std::vector<VariationalLoopConstraint>& constraints, double timeStep)
{
  std::vector<dvbd::AvbdScalarRowBounds> bounds;
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
  return bounds;
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
  std::vector<AxisRow> linearRows;
  std::vector<AxisRow> angularRows;
};

struct VariationalCompliantLoopScratch
{
  void clear()
  {
    linearInventory.records().clear();
    angularInventory.records().clear();
  }

  dvbd::AvbdScalarRowInventory linearInventory;
  dvbd::AvbdScalarRowInventory angularInventory;
};

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

void appendAvbdRigidWorldArticulatedPointJointConstraints(
    const detail::WorldRegistry& registry,
    entt::entity structureEntity,
    std::vector<VariationalLoopConstraint>& constraints,
    std::vector<VariationalCompliantLoopConstraint>* compliantConstraints
    = nullptr)
{
  const auto& structure
      = registry.get<comps::MultibodyStructure>(structureEntity);
  constexpr std::size_t kStructureLinkIndexMapThreshold = 16u;
  std::unordered_map<entt::entity, int> structureLinkIndex;
  if (structure.links.size() > kStructureLinkIndexMapThreshold) {
    structureLinkIndex.reserve(structure.links.size());
    for (std::size_t i = 0; i < structure.links.size(); ++i) {
      structureLinkIndex.emplace(structure.links[i], static_cast<int>(i));
    }
  }
  const auto linkIndexInStructure = [&](entt::entity link) -> int {
    if (link == entt::null) {
      return -1;
    }
    if (!structureLinkIndex.empty()) {
      const auto it = structureLinkIndex.find(link);
      return it == structureLinkIndex.end() ? -1 : it->second;
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
        && (compliantConstraints == nullptr || !compliantStiffness.has_value()
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
      VariationalCompliantLoopConstraint constraint;
      constraint.linkA = linkIndexA;
      constraint.pointA = config.localAnchorA;
      constraint.linkB = linkIndexB;
      constraint.pointB = config.localAnchorB;
      constraint.rigid = angularRows > 0u;
      constraint.rotationA = dvbd::normalizeAvbdRigidOrientation(
                                 config.targetRelativeOrientation)
                                 .toRotationMatrix();
      constraint.rotationB = Eigen::Matrix3d::Identity();
      constraint.linearAxes = axisFrame * config.linearAxes;
      constraint.angularAxes = axisFrame * config.angularAxes;
      constraint.linearAxisMask = config.linearAxisMask;
      constraint.angularAxisMask = config.angularAxisMask;
      constraint.linearRows.reserve(linearRows);
      for (std::uint8_t axis = 0; axis < 3u; ++axis) {
        if (!dvbd::detail::avbdRigidJointAxisEnabled(
                config.linearAxisMask, axis)) {
          continue;
        }
        constraint.linearRows.push_back(
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
      constraint.angularRows.reserve(angularRows);
      for (std::uint8_t axis = 0; axis < 3u; ++axis) {
        if (!dvbd::detail::avbdRigidJointAxisEnabled(
                config.angularAxisMask, axis)) {
          continue;
        }
        constraint.angularRows.push_back(
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
      compliantConstraints->push_back(constraint);
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
    const std::vector<VariationalLoopConstraint>& constraints,
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
    std::vector<VariationalCompliantLoopConstraint>& constraints,
    VariationalCompliantLoopScratch& scratch)
{
  std::vector<dvbd::AvbdScalarRowDescriptor> linearDescriptors;
  std::vector<dvbd::AvbdScalarRowDescriptor> angularDescriptors;
  for (const VariationalCompliantLoopConstraint& constraint : constraints) {
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
  for (VariationalCompliantLoopConstraint& constraint : constraints) {
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

void updateVariationalCompliantLoopConstraintRows(
    const detail::WorldRegistry& registry,
    const comps::MultibodyStructure& structure,
    const std::vector<VariationalCompliantLoopConstraint>& constraints,
    VariationalCompliantLoopScratch& scratch)
{
  if (constraints.empty()) {
    scratch.clear();
    return;
  }

  const VarTree tree = buildVarTree(registry, structure);
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
  position = Eigen::VectorXd::Zero(dof);
  velocity = Eigen::VectorXd::Zero(dof);
  appliedForce = Eigen::VectorXd::Zero(dof);
  for (const auto& link : tree.links) {
    if (link.dof == 0) {
      continue;
    }
    const auto& joint = registry.get<comps::Joint>(link.joint);
    const auto seg = static_cast<Eigen::Index>(link.dofOffset);
    const auto n = static_cast<Eigen::Index>(link.dof);
    position.segment(seg, n) = joint.position;
    velocity.segment(seg, n) = joint.velocity;

    Eigen::VectorXd effort = Eigen::VectorXd::Zero(n);
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
    appliedForce.segment(seg, n) = effort;
  }
}

struct VariationalVelocityConstraint
{
  Eigen::Index dof = 0;
  double targetPosition = 0.0;
};

std::vector<VariationalVelocityConstraint> buildVariationalVelocityConstraints(
    const detail::WorldRegistry& registry,
    const VarTree& tree,
    const Eigen::VectorXd& position,
    double timeStep)
{
  std::vector<VariationalVelocityConstraint> constraints;
  for (const auto& link : tree.links) {
    if (link.dof == 0) {
      continue;
    }
    const auto& joint = registry.get<comps::Joint>(link.joint);
    if (joint.actuatorType != comps::ActuatorType::Velocity) {
      continue;
    }
    if (joint.type != comps::JointType::Revolute
        && joint.type != comps::JointType::Prismatic) {
      continue;
    }

    const auto seg = static_cast<Eigen::Index>(link.dofOffset);
    const auto n = static_cast<Eigen::Index>(link.dof);
    Eigen::VectorXd command = Eigen::VectorXd::Zero(n);
    if (joint.commandVelocity.size() == n
        && joint.commandVelocity.allFinite()) {
      command = joint.commandVelocity;
    }
    const Eigen::VectorXd target
        = jointRetract(joint, position.segment(seg, n), timeStep * command);
    for (Eigen::Index d = 0; d < n; ++d) {
      constraints.push_back({seg + d, target[d]});
    }
  }
  return constraints;
}

//==============================================================================
std::optional<VariationalSolveReport> tryIntegrateSinglePrismaticVariational(
    detail::WorldRegistry& registry,
    const comps::MultibodyStructure& structure,
    const Eigen::Vector3d& gravity,
    double timeStep,
    MultibodyVariationalState& state,
    const std::vector<VariationalLoopConstraint>& constraints,
    const VariationalContactHook& contactHook,
    const VariationalGroundContact* groundContact,
    std::span<const double> groundContactDuals)
{
  if (!constraints.empty() || contactHook || structure.links.size() != 2u
      || structure.joints.size() != 1u) {
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
std::vector<Vector6> currentSpatialVelocities(
    const VarTree& tree, const Eigen::VectorXd& velocity)
{
  std::vector<Vector6> v(tree.links.size(), Vector6::Zero());
  for (std::size_t i = 0; i < tree.links.size(); ++i) {
    const auto& link = tree.links[i];
    if (link.parent < 0) {
      v[i].setZero();
    } else {
      v[i] = adjoint(link.currentRelative.inverse())
             * v[static_cast<std::size_t>(link.parent)];
    }
    if (link.dof > 0) {
      v[i] += link.subspace
              * velocity.segment(
                  static_cast<Eigen::Index>(link.dofOffset),
                  static_cast<Eigen::Index>(link.dof));
    }
  }
  return v;
}

// Evaluate the forced discrete Euler-Lagrange residual f(qNext) in O(n) via a
// forward (average-velocity) sweep and a backward (momentum/impulse) sweep.
Eigen::VectorXd computeResidual(
    const detail::WorldRegistry& registry,
    VarTree& tree,
    const Eigen::VectorXd& nextPosition,
    const MultibodyVariationalState& state,
    const Eigen::Vector3d& gravity,
    double timeStep,
    const Eigen::VectorXd& appliedForce)
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
  Eigen::VectorXd residual
      = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(tree.dofCount));
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
  return residual;
}

// O(n) articulated-body-inertia solve of M(q)^{-1} * b for the precomputed tree
// (Featherstone ABA with zero velocity and gravity): a backward articulated-
// inertia sweep followed by a forward acceleration sweep. This is the
// linear-time RIQN quasi-Newton step (replacing a dense factorization) and
// equals massMatrix.ldlt().solve(b) to machine precision (cross-checked by
// test). Frames follow the [angular; linear] convention: parentToChild maps a
// spatial motion parent->child, its transpose maps a spatial force
// child->parent.
Eigen::VectorXd applyArticulatedInverseMass(
    const VarTree& tree, const Eigen::VectorXd& b)
{
  const std::size_t n = tree.links.size();
  std::vector<Matrix6> articulatedInertia(n);
  std::vector<Vector6> biasForce(n, Vector6::Zero());
  std::vector<Eigen::MatrixXd> u(n); // U_i = I^A_i S_i        (6 x dof)
  std::vector<Eigen::MatrixXd> dInverse(
      n);                                 // (S_i^T U_i)^{-1}       (dof x dof)
  std::vector<Eigen::VectorXd> uForce(n); // u_i = b_i - S_i^T p^A_i (dof)
  std::vector<Matrix6> parentToChild(n, Matrix6::Identity());
  for (std::size_t i = 0; i < n; ++i) {
    articulatedInertia[i] = tree.links[i].inertia;
  }

  // Backward sweep: accumulate articulated inertia and bias toward the root.
  for (std::size_t reverse = 0; reverse < n; ++reverse) {
    const std::size_t i = n - 1 - reverse;
    const VarLink& link = tree.links[i];
    if (link.dof > 0) {
      const auto seg = static_cast<Eigen::Index>(link.dofOffset);
      const auto dof = static_cast<Eigen::Index>(link.dof);
      u[i] = articulatedInertia[i] * link.subspace;
      const Eigen::MatrixXd d = link.subspace.transpose() * u[i];
      dInverse[i] = d.inverse();
      uForce[i]
          = b.segment(seg, dof) - link.subspace.transpose() * biasForce[i];
    }
    if (link.parent >= 0) {
      Matrix6 ia = articulatedInertia[i];
      Vector6 pa = biasForce[i];
      if (link.dof > 0) {
        ia.noalias() -= u[i] * dInverse[i] * u[i].transpose();
        pa.noalias() += u[i] * (dInverse[i] * uForce[i]);
      }
      parentToChild[i] = adjoint(link.currentRelative.inverse());
      const Matrix6 forceToParent = parentToChild[i].transpose();
      const auto p = static_cast<std::size_t>(link.parent);
      articulatedInertia[p].noalias() += forceToParent * ia * parentToChild[i];
      biasForce[p].noalias() += forceToParent * pa;
    }
  }

  // Forward sweep: propagate spatial accelerations and read joint
  // accelerations.
  Eigen::VectorXd qddot
      = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(tree.dofCount));
  std::vector<Vector6> acceleration(n, Vector6::Zero());
  for (std::size_t i = 0; i < n; ++i) {
    const VarLink& link = tree.links[i];
    if (link.parent < 0) {
      acceleration[i].setZero();
      continue;
    }
    const Vector6 parentAccel
        = parentToChild[i]
          * acceleration[static_cast<std::size_t>(link.parent)];
    if (link.dof > 0) {
      const auto seg = static_cast<Eigen::Index>(link.dofOffset);
      const auto dof = static_cast<Eigen::Index>(link.dof);
      const Eigen::VectorXd jointAccel
          = dInverse[i] * (uForce[i] - u[i].transpose() * parentAccel);
      qddot.segment(seg, dof) = jointAccel;
      acceleration[i] = parentAccel + link.subspace * jointAccel;
    } else {
      acceleration[i] = parentAccel;
    }
  }
  return qddot;
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
Eigen::VectorXd applyExactNewtonStep(
    const VarTree& tree, const Eigen::VectorXd& b, double timeStep)
{
  const std::size_t n = tree.links.size();
  std::vector<Matrix6> articulated(n, Matrix6::Zero());
  std::vector<Vector6> bias(n, Vector6::Zero());
  std::vector<Matrix6> motionToChild(n, Matrix6::Identity()); // A_i (motion)
  std::vector<Eigen::MatrixXd> forceProjector(
      n); // U_i = Pi_i S_i      (6 x dof)
  std::vector<Eigen::MatrixXd> motionProjector(
      n); // M_i = Pi_i^T S_i (so M_i^T = S_i^T Pi_i)            (6 x dof)
  std::vector<Eigen::MatrixXd> dInverse(n); // (S_i^T Pi_i S_i)^{-1}
  std::vector<Eigen::VectorXd> uForce(n);   // b_i - S_i^T bias_i  (dof)

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
    articulated[i]
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
      forceProjector[i] = articulated[i] * link.subspace; // 6 x dof
      motionProjector[i]
          = articulated[i].transpose() * link.subspace; // 6 x dof
      const Eigen::MatrixXd d = link.subspace.transpose() * forceProjector[i];
      dInverse[i] = d.inverse();
      uForce[i] = b.segment(seg, dof) - link.subspace.transpose() * bias[i];
    }
    if (link.parent >= 0) {
      Matrix6 ia = articulated[i];
      Vector6 pa = bias[i];
      if (link.dof > 0) {
        // Non-symmetric rank-1 elimination: force-side projector Pi_i S_i times
        // the motion-side row S_i^T Pi_i (= motionProjector_i^T). For the
        // symmetric mass-matrix limit (D_i -> G_i) this reduces to the standard
        // U D^{-1} U^T of `applyArticulatedInverseMass`.
        ia.noalias() -= forceProjector[i] * dInverse[i]
                        * (motionProjector[i].transpose());
        pa.noalias() += forceProjector[i] * (dInverse[i] * uForce[i]);
      }
      // Distinct transports: motion parent->child uses the q^{k+1} relative;
      // force child->parent uses the q^k relative (the constant force transport
      // of the residual's backward sweep).
      motionToChild[i] = adjoint(link.nextRelative.inverse());
      const Matrix6 forceToParent
          = adjoint(link.currentRelative.inverse()).transpose();
      const auto p = static_cast<std::size_t>(link.parent);
      articulated[p].noalias() += forceToParent * ia * motionToChild[i];
      bias[p].noalias() += forceToParent * pa;
    }
  }

  // Forward sweep: propagate the body-frame delta-twist xi and read the joint
  // increments dq.
  Eigen::VectorXd dq
      = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(tree.dofCount));
  std::vector<Vector6> twist(n, Vector6::Zero());
  for (std::size_t i = 0; i < n; ++i) {
    const VarLink& link = tree.links[i];
    if (link.parent < 0) {
      twist[i].setZero();
      continue;
    }
    const Vector6 parentTwist
        = motionToChild[i] * twist[static_cast<std::size_t>(link.parent)];
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
      const Eigen::VectorXd jointDelta
          = dInverse[i]
            * (uForce[i] - motionProjector[i].transpose() * parentTwist);
      dq.segment(seg, dof) = jointDelta;
      twist[i] = parentTwist + link.subspace * jointDelta;
    } else {
      twist[i] = parentTwist;
    }
  }
  return dq;
}

// Body-frame spatial Jacobian of every link (6 x dofCount, [angular; linear] in
// the link frame): J_i = Ad(T_i^{-1}) J_parent with the link's own joint
// columns set to its motion subspace.
std::vector<Eigen::MatrixXd> bodyJacobians(const VarTree& tree)
{
  const auto dof = static_cast<Eigen::Index>(tree.dofCount);
  std::vector<Eigen::MatrixXd> jacobian(
      tree.links.size(), Eigen::MatrixXd::Zero(6, dof));
  for (std::size_t i = 0; i < tree.links.size(); ++i) {
    const VarLink& link = tree.links[i];
    if (link.parent >= 0) {
      jacobian[i] = adjoint(link.currentRelative.inverse())
                    * jacobian[static_cast<std::size_t>(link.parent)];
    }
    if (link.dof > 0) {
      jacobian[i].middleCols(
          static_cast<Eigen::Index>(link.dofOffset),
          static_cast<Eigen::Index>(link.dof)) = link.subspace;
    }
  }
  return jacobian;
}

void resizeBodyJacobianScratch(
    std::vector<Eigen::MatrixXd>& jacobians,
    const std::size_t linkCount,
    const Eigen::Index dof)
{
  if (jacobians.size() != linkCount) {
    jacobians.resize(linkCount);
  }
  for (Eigen::MatrixXd& jacobian : jacobians) {
    jacobian.setZero(6, dof);
  }
}

void bodyJacobiansInto(
    const VarTree& tree, std::vector<Eigen::MatrixXd>& jacobians)
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

void bodyJacobiansInto(
    const VarTree& tree,
    const std::vector<Eigen::Isometry3d>& relativeTransforms,
    std::vector<Eigen::MatrixXd>& jacobians)
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
Eigen::MatrixXd worldPointJacobian(
    const VarTree& tree,
    const std::vector<Eigen::MatrixXd>& jacobians,
    std::size_t i,
    const Eigen::Vector3d& point)
{
  const Eigen::Matrix3d rotation = tree.links[i].worldTransform.linear();
  const Eigen::MatrixXd angular = jacobians[i].topRows<3>();
  const Eigen::MatrixXd linear = jacobians[i].bottomRows<3>();
  return rotation * (linear - skew(point) * angular);
}

// Stack the holonomic residual g(q) and Jacobian J = dg/dq for the loop
// closures on a tree built at the current configuration.
std::pair<Eigen::VectorXd, Eigen::MatrixXd> constraintResidualAndJacobian(
    const comps::MultibodyStructure& structure,
    const VarTree& tree,
    const std::vector<Eigen::MatrixXd>& jacobians,
    const std::vector<VariationalLoopConstraint>& constraints,
    double timeStep = 0.0)
{
  const auto& links = structure.links;
  const auto indexOf = [&](entt::entity e) -> int {
    const auto it = std::find(links.begin(), links.end(), e);
    return it == links.end() ? -1 : static_cast<int>(it - links.begin());
  };
  const auto dof = static_cast<Eigen::Index>(tree.dofCount);

  Eigen::Index rows = 0;
  for (const auto& c : constraints) {
    rows += variationalLoopConstraintRowCount(c);
  }
  Eigen::VectorXd g(rows);
  Eigen::MatrixXd jac = Eigen::MatrixXd::Zero(rows, dof);

  Eigen::Index row = 0;
  for (const auto& c : constraints) {
    const int ia = (c.linkA != entt::null) ? indexOf(c.linkA) : -1;
    const int ib = (c.linkB != entt::null) ? indexOf(c.linkB) : -1;

    Eigen::Vector3d pointA = c.pointA;
    Eigen::MatrixXd jacA = Eigen::MatrixXd::Zero(3, dof);
    if (ia >= 0) {
      pointA
          = tree.links[static_cast<std::size_t>(ia)].worldTransform * c.pointA;
      jacA = worldPointJacobian(
          tree, jacobians, static_cast<std::size_t>(ia), c.pointA);
    }
    Eigen::Vector3d pointB = c.pointB;
    Eigen::MatrixXd jacB = Eigen::MatrixXd::Zero(3, dof);
    if (ib >= 0) {
      pointB
          = tree.links[static_cast<std::size_t>(ib)].worldTransform * c.pointB;
      jacB = worldPointJacobian(
          tree, jacobians, static_cast<std::size_t>(ib), c.pointB);
    }

    if (c.distance) {
      const Eigen::Vector3d offset = pointA - pointB;
      const double dist = offset.norm();
      const Eigen::Vector3d dir = dist > 1e-12 ? Eigen::Vector3d(offset / dist)
                                               : Eigen::Vector3d::UnitX();
      g[row] = dist - c.length;
      jac.row(row) = dir.transpose() * (jacA - jacB);
      row += 1;
      continue;
    }

    const Eigen::Vector3d positionResidual = pointA - pointB;
    const Eigen::MatrixXd positionJacobian = jacA - jacB;
    for (std::uint8_t axis = 0; axis < 3u; ++axis) {
      if (!dvbd::detail::avbdRigidJointAxisEnabled(c.linearAxisMask, axis)) {
        continue;
      }
      const Eigen::Vector3d rowAxis = variationalLoopAxis(c.linearAxes, axis);
      g[row] = rowAxis.dot(positionResidual);
      jac.row(row) = rowAxis.transpose() * positionJacobian;
      row += 1;
    }
    if (c.linearMotor) {
      const Eigen::Vector3d rowAxis = normalizedVariationalLoopAxis(
          c.linearMotorAxis, Eigen::Vector3d::UnitX());
      const double targetPosition = c.linearMotorReferencePosition
                                    + c.linearMotorTargetSpeed * timeStep;
      g[row] = -rowAxis.dot(positionResidual) - targetPosition;
      jac.row(row) = -rowAxis.transpose() * positionJacobian;
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

      Eigen::MatrixXd angA = Eigen::MatrixXd::Zero(3, dof);
      Eigen::MatrixXd angB = Eigen::MatrixXd::Zero(3, dof);
      if (ia >= 0) {
        angA = tree.links[static_cast<std::size_t>(ia)].worldTransform.linear()
               * jacobians[static_cast<std::size_t>(ia)].topRows<3>();
      }
      if (ib >= 0) {
        angB = tree.links[static_cast<std::size_t>(ib)].worldTransform.linear()
               * jacobians[static_cast<std::size_t>(ib)].topRows<3>();
      }
      const Eigen::MatrixXd angularJacobian = angA - angB;
      for (std::uint8_t axis = 0; axis < 3u; ++axis) {
        if (!dvbd::detail::avbdRigidJointAxisEnabled(c.angularAxisMask, axis)) {
          continue;
        }
        const Eigen::Vector3d rowAxis
            = variationalLoopAxis(c.angularAxes, axis);
        g[row] = rowAxis.dot(angularResidual);
        jac.row(row) = rowAxis.transpose() * angularJacobian;
        row += 1;
      }
      if (c.angularMotor) {
        const Eigen::Vector3d rowAxis = normalizedVariationalLoopAxis(
            c.angularMotorAxis, Eigen::Vector3d::UnitZ());
        const double targetPosition = c.angularMotorReferencePosition
                                      + c.angularMotorTargetSpeed * timeStep;
        g[row] = -rowAxis.dot(angularResidual) - targetPosition;
        jac.row(row) = -rowAxis.transpose() * angularJacobian;
        row += 1;
      }
    }
  }
  return {g, jac};
}

void evaluateContactForceInto(
    const detail::WorldRegistry& registry,
    const VarTree& tree,
    const Eigen::VectorXd& nextPosition,
    double timeStep,
    const Eigen::VectorXd& previousVelocity,
    const VariationalContactHook& contactHook,
    const VariationalGroundContactSolver* groundContactSolver,
    VariationalContactEvaluationScratch& scratch,
    Eigen::VectorXd& contactForce)
{
  if (groundContactSolver == nullptr && !contactHook) {
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
      scratch.trialWorldTransforms,
      scratch.trialJacobians,
      tree.dofCount,
      scratch.previousWorldTransforms,
      scratch.previousJacobians,
      previousVelocity,
      timeStep};
  if (groundContactSolver != nullptr) {
    groundContactSolver->computeForceInto(context, contactForce);
  } else {
    contactForce = contactHook(context);
  }

  DART_SIMULATION_THROW_T_IF(
      contactForce.size() != static_cast<Eigen::Index>(tree.dofCount),
      InvalidOperationException,
      "Variational contact hook returned a generalized force of the wrong "
      "dimension");
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

Eigen::VectorXd variationalWorldTorque(
    const VariationalContactContext& context,
    int linkIndex,
    const Eigen::Vector3d& worldTorque)
{
  if (linkIndex < 0) {
    return Eigen::VectorXd::Zero(static_cast<Eigen::Index>(context.dofCount));
  }
  const auto index = static_cast<std::size_t>(linkIndex);
  DART_SIMULATION_THROW_T_IF(
      index >= context.linkWorldTransforms.size(),
      InvalidOperationException,
      "Compliant variational loop constraint references an out-of-range link "
      "index");
  const Eigen::MatrixXd worldAngularJacobian
      = context.linkWorldTransforms[index].linear()
        * context.linkBodyJacobians[index].topRows<3>();
  return worldAngularJacobian.transpose() * worldTorque;
}

VariationalContactHook makeVariationalCompliantLoopConstraintHook(
    std::vector<VariationalCompliantLoopConstraint> constraints)
{
  if (constraints.empty()) {
    return {};
  }

  return [constraints = std::move(constraints)](
             const VariationalContactContext& context) -> Eigen::VectorXd {
    Eigen::VectorXd generalizedForce
        = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(context.dofCount));
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

    for (const VariationalCompliantLoopConstraint& constraint : constraints) {
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
        generalizedForce
            += variationalWorldTorque(context, constraint.linkA, torqueA);
        generalizedForce
            += variationalWorldTorque(context, constraint.linkB, -torqueA);
      }
    }
    return generalizedForce;
  };
}

VariationalContactHook combineVariationalContactHooks(
    VariationalContactHook first, VariationalContactHook second)
{
  if (!first) {
    return second;
  }
  if (!second) {
    return first;
  }
  return [first = std::move(first), second = std::move(second)](
             const VariationalContactContext& context) -> Eigen::VectorXd {
    const Eigen::VectorXd firstForce = first(context);
    const Eigen::VectorXd secondForce = second(context);
    if (firstForce.size() == 0) {
      return secondForce;
    }
    if (secondForce.size() == 0) {
      return firstForce;
    }
    return firstForce + secondForce;
  };
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
    MultibodyVariationalScratch& scratch)
{
  auto& contact = scratch.groundContact;
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
    const std::vector<Eigen::Isometry3d>& linkWorldTransforms)
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
    const std::vector<VariationalLoopConstraint>& constraints,
    std::size_t andersonDepth,
    const VariationalContactHook& contactHook,
    const VariationalGroundContact* fastGroundContact,
    const VariationalGroundContactSolver* groundContactSolver,
    VariationalContactEvaluationScratch* contactScratch)
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
            fastGroundContact,
            groundContactDuals)) {
      return *fastReport;
    }
  }

  VarTree tree = buildVarTree(registry, structure);
  if (tree.dofCount == 0) {
    return report;
  }

  Eigen::VectorXd position;
  Eigen::VectorXd velocity;
  Eigen::VectorXd appliedForce;
  gatherState(registry, tree, position, velocity, appliedForce);

  // Bootstrap the two-step history from the current generalized velocity: seed
  // dT_prev = exp(dt * V_i) and mu_prev = dexp^{-T}(dt V_i, G V_i) so the first
  // step's momentum-transport term is consistent.
  if (!state.bootstrapped
      || state.previousDeltaTransform.size() != tree.links.size()) {
    const std::vector<Vector6> v0 = currentSpatialVelocities(tree, velocity);
    state.previousDeltaTransform.assign(
        tree.links.size(), Eigen::Isometry3d::Identity());
    state.previousMomentum.assign(tree.links.size(), Vector6::Zero());
    for (std::size_t i = 0; i < tree.links.size(); ++i) {
      const Vector6 scaled = timeStep * v0[i];
      state.previousDeltaTransform[i] = dm::se3Exp(scaled);
      state.previousMomentum[i]
          = dm::dexpInvTranspose(scaled, tree.links[i].inertia * v0[i]);
    }
    state.bootstrapped = true;
  }

  // Initial guess IG3 (semi-implicit Euler from forward dynamics): far closer
  // to the DEL root than explicit Euler, which keeps the RIQN iteration count
  // low and roughly uniform across chain length. The forward-dynamics
  // acceleration ddq = M(q)^{-1}(appliedForce - (C(q,qdot) qdot + g(q))) reuses
  // the O(n) inverse-mass apply, so the initial guess stays linear-time. A1
  // joints are Euclidean, so the guess uses plain vector arithmetic.
  const Eigen::VectorXd bias = computeMultibodyInverseDynamics(
      registry,
      structure,
      gravity,
      Eigen::VectorXd::Zero(static_cast<Eigen::Index>(tree.dofCount)));
  const Eigen::VectorXd guessAcceleration
      = applyArticulatedInverseMass(tree, appliedForce - bias);
  Eigen::VectorXd nextPosition = position;
  for (const auto& link : tree.links) {
    if (link.dof == 0) {
      continue;
    }
    const auto& joint = registry.get<comps::Joint>(link.joint);
    const auto seg = static_cast<Eigen::Index>(link.dofOffset);
    const auto n = static_cast<Eigen::Index>(link.dof);
    const Eigen::VectorXd tangent
        = timeStep * velocity.segment(seg, n)
          + timeStep * timeStep * guessAcceleration.segment(seg, n);
    nextPosition.segment(seg, n)
        = jointRetract(joint, position.segment(seg, n), tangent);
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

  // Retract `base` by `-scale * increment` per joint (the Newton update,
  // damped by `scale`), so spherical/floating coordinates stay on their
  // manifolds. Shared by the line search and the accepted update.
  const auto retractStep = [&](const Eigen::VectorXd& base,
                               const Eigen::VectorXd& increment,
                               double scale) {
    Eigen::VectorXd result = base;
    for (const auto& link : tree.links) {
      if (link.dof == 0) {
        continue;
      }
      const auto& joint = registry.get<comps::Joint>(link.joint);
      const auto seg = static_cast<Eigen::Index>(link.dofOffset);
      const auto n = static_cast<Eigen::Index>(link.dof);
      result.segment(seg, n) = jointRetract(
          joint, base.segment(seg, n), -scale * increment.segment(seg, n));
    }
    return result;
  };

  // The tangent displacement from iterate `from` to iterate `to`, stacked per
  // joint: tau_a = jointLogDifference(to_a, from_a) so jointRetract(from_a,
  // tau_a) == to_a. This is the manifold-correct "to - from" the tangent-space
  // Anderson history mixes (a spherical/floating joint's raw coordinate
  // difference is not a tangent vector; its per-joint log is).
  const auto perJointLogDifference
      = [&](const Eigen::VectorXd& to, const Eigen::VectorXd& from) {
          Eigen::VectorXd tangent
              = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(tree.dofCount));
          for (const auto& link : tree.links) {
            if (link.dof == 0) {
              continue;
            }
            const auto& joint = registry.get<comps::Joint>(link.joint);
            const auto seg = static_cast<Eigen::Index>(link.dofOffset);
            const auto n = static_cast<Eigen::Index>(link.dof);
            tangent.segment(seg, n) = jointLogDifference(
                joint, to.segment(seg, n), from.segment(seg, n));
          }
          return tangent;
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
  std::vector<Eigen::VectorXd> stepDeltas;
  std::vector<Eigen::VectorXd> iterateDeltas;
  Eigen::VectorXd previousStep;     // tangent update at q^{k-1}
  Eigen::VectorXd previousPosition; // q^{k-1}
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
  const auto residualAt = [&](const Eigen::VectorXd& trialPosition) {
    evaluateContactForceInto(
        registry,
        tree,
        trialPosition,
        timeStep,
        velocity,
        contactHook,
        groundContactSolver,
        contactEvaluation,
        contactEvaluation.contactForce);
    if (contactEvaluation.contactForce.size() == 0) {
      return computeResidual(
          registry,
          tree,
          trialPosition,
          state,
          gravity,
          timeStep,
          appliedForce);
    }
    contactEvaluation.forcing = appliedForce;
    contactEvaluation.forcing += contactEvaluation.contactForce;
    return computeResidual(
        registry,
        tree,
        trialPosition,
        state,
        gravity,
        timeStep,
        contactEvaluation.forcing);
  };

  // `tolerance` is a per-coordinate accuracy; the convergence test is on the
  // L2 norm of the (dofCount-dimensional) residual, so scale by sqrt(dofCount)
  // to keep the per-coordinate accuracy uniform across chain lengths. Without
  // this, the fixed norm threshold is sqrt(n) times stricter for an n-DOF
  // system and the hardest steps of long chains stall just above it.
  const double normTolerance
      = tolerance * std::sqrt(static_cast<double>(tree.dofCount));
  for (int iteration = 0; iteration < maxIterations; ++iteration) {
    const Eigen::VectorXd residual = residualAt(nextPosition);
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
    const Eigen::VectorXd step
        = euclideanCoordinates
              ? applyExactNewtonStep(tree, residual, timeStep)
              : applyArticulatedInverseMass(tree, timeStep * residual);

    // Manifold tangent-space Anderson mixing (spherical/floating path only).
    // Accumulate the step/iterate-displacement history and form the accelerated
    // increment; the Euclidean exact-Newton path keeps `step` unchanged. The
    // least squares is Tikhonov-regularized (a small ridge relative to the
    // column scale) because the step-difference columns become nearly linearly
    // dependent as the iterates settle, and an unregularized type-II solve then
    // produces a wild gamma that throws the increment far off -- the
    // regularizer (plus the opportunistic accept/safeguard below) keeps the
    // acceleration stable on long stiff manifold chains.
    Eigen::VectorXd andersonIncrement;
    bool haveAnderson = false;
    if (!euclideanCoordinates && andersonDepth > 0 && iteration > 0) {
      stepDeltas.push_back(step - previousStep);
      iterateDeltas.push_back(
          perJointLogDifference(nextPosition, previousPosition));
      if (stepDeltas.size() > andersonDepth) {
        stepDeltas.erase(stepDeltas.begin());
        iterateDeltas.erase(iterateDeltas.begin());
      }
      const auto m = static_cast<Eigen::Index>(stepDeltas.size());
      Eigen::MatrixXd stepMatrix(step.size(), m);
      Eigen::MatrixXd iterateMatrix(step.size(), m);
      for (Eigen::Index c = 0; c < m; ++c) {
        stepMatrix.col(c) = stepDeltas[static_cast<std::size_t>(c)];
        iterateMatrix.col(c) = iterateDeltas[static_cast<std::size_t>(c)];
      }
      // Solve (F^T F + lambda I) gamma = F^T step (normal equations with a
      // ridge scaled by the column magnitudes) for the type-II mixing
      // coefficients.
      const Eigen::MatrixXd ftf = stepMatrix.transpose() * stepMatrix;
      const double ridge
          = 1e-12 * (ftf.trace() / static_cast<double>(m) + 1e-30);
      const Eigen::MatrixXd regularized
          = ftf + ridge * Eigen::MatrixXd::Identity(m, m);
      const Eigen::VectorXd gamma
          = regularized.ldlt().solve(stepMatrix.transpose() * step);
      if (gamma.allFinite()) {
        andersonIncrement = step + (iterateMatrix - stepMatrix) * gamma;
        haveAnderson = andersonIncrement.allFinite();
      }
    }
    previousStep = step;
    previousPosition = nextPosition;

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
      Eigen::VectorXd trialPosition = retractStep(nextPosition, step, alpha);
      double trialNorm = residualAt(trialPosition).norm();
      constexpr int kMaxBacktracks = 20;
      for (int backtrack = 0;
           backtrack < kMaxBacktracks && !(trialNorm < residualNorm);
           ++backtrack) {
        alpha *= 0.5;
        trialPosition = retractStep(nextPosition, step, alpha);
        trialNorm = residualAt(trialPosition).norm();
      }
      nextPosition = std::move(trialPosition);
      continue;
    }

    if (haveAnderson) {
      Eigen::VectorXd andersonPosition
          = retractStep(nextPosition, andersonIncrement, 1.0);
      const double andersonNorm = residualAt(andersonPosition).norm();
      if (andersonNorm < residualNorm) {
        nextPosition = std::move(andersonPosition);
        continue;
      }
    }
    nextPosition = retractStep(nextPosition, step, 1.0);
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
  const auto velocityConstraints
      = buildVariationalVelocityConstraints(registry, tree, position, timeStep);
  if (!constraints.empty() || !velocityConstraints.empty()) {
    constexpr double constraintTolerance = 1e-10;
    constexpr int maxProjectionIterations = 32;
    Eigen::VectorXd projectionLambda;
    for (int projection = 0; projection < maxProjectionIterations;
         ++projection) {
      // Reflect the candidate configuration in the joints so a freshly built
      // tree carries the world transforms and Jacobians at nextPosition.
      for (const VarLink& link : tree.links) {
        if (link.dof == 0) {
          continue;
        }
        registry.get<comps::Joint>(link.joint).position = nextPosition.segment(
            static_cast<Eigen::Index>(link.dofOffset),
            static_cast<Eigen::Index>(link.dof));
      }
      const VarTree nextTree = buildVarTree(registry, structure);
      const auto dof = static_cast<Eigen::Index>(nextTree.dofCount);
      Eigen::Index rows = static_cast<Eigen::Index>(velocityConstraints.size());
      Eigen::VectorXd loopResidual;
      Eigen::MatrixXd loopJacobian;
      std::vector<dvbd::AvbdScalarRowBounds> projectionBounds;
      if (!constraints.empty()) {
        const std::vector<Eigen::MatrixXd> jacobians = bodyJacobians(nextTree);
        auto [gLoop, jacLoop] = constraintResidualAndJacobian(
            structure, nextTree, jacobians, constraints, timeStep);
        loopResidual = std::move(gLoop);
        loopJacobian = std::move(jacLoop);
        rows += loopResidual.size();
        projectionBounds
            = variationalLoopConstraintRowBounds(constraints, timeStep);
      }
      projectionBounds.resize(static_cast<std::size_t>(rows));

      Eigen::VectorXd g = Eigen::VectorXd::Zero(rows);
      Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(rows, dof);
      Eigen::Index row = 0;
      if (loopResidual.size() > 0) {
        g.segment(row, loopResidual.size()) = loopResidual;
        jacobian.middleRows(row, loopJacobian.rows()) = loopJacobian;
        row += loopResidual.size();
      }
      for (const auto& velocityConstraint : velocityConstraints) {
        if (velocityConstraint.dof < 0 || velocityConstraint.dof >= dof) {
          continue;
        }
        g[row] = nextPosition[velocityConstraint.dof]
                 - velocityConstraint.targetPosition;
        jacobian(row, velocityConstraint.dof) = 1.0;
        ++row;
      }
      if (g.norm() <= constraintTolerance) {
        break;
      }
      Eigen::MatrixXd inverseMassJt(
          static_cast<Eigen::Index>(nextTree.dofCount), rows);
      for (Eigen::Index r = 0; r < rows; ++r) {
        inverseMassJt.col(r) = applyArticulatedInverseMass(
            nextTree, jacobian.row(r).transpose());
      }
      const Eigen::MatrixXd constraintMass = jacobian * inverseMassJt;
      Eigen::VectorXd lambda = constraintMass.ldlt().solve(-g);
      std::vector<Eigen::Index> hardRows;
      std::vector<Eigen::Index> boundedRows;
      hardRows.reserve(static_cast<std::size_t>(lambda.size()));
      boundedRows.reserve(static_cast<std::size_t>(lambda.size()));
      for (Eigen::Index r = 0; r < lambda.size(); ++r) {
        const auto& bounds = projectionBounds[static_cast<std::size_t>(r)];
        if (variationalProjectionRowHasFiniteBounds(bounds)) {
          boundedRows.push_back(r);
          lambda[r] = dvbd::clampAvbdRowForce(lambda[r], bounds);
        } else {
          hardRows.push_back(r);
        }
      }
      if (!boundedRows.empty() && !hardRows.empty()) {
        // Bounded motor rows may saturate before reaching their target. Keep
        // hard joint/anchor rows authoritative after those impulses are
        // clamped so finite effort cannot leak into locked coordinates.
        Eigen::MatrixXd hardMass(hardRows.size(), hardRows.size());
        Eigen::VectorXd hardRhs(hardRows.size());
        for (std::size_t i = 0; i < hardRows.size(); ++i) {
          const Eigen::Index hardRow = hardRows[i];
          hardRhs[static_cast<Eigen::Index>(i)] = -g[hardRow];
          for (const Eigen::Index boundedRow : boundedRows) {
            hardRhs[static_cast<Eigen::Index>(i)]
                -= constraintMass(hardRow, boundedRow) * lambda[boundedRow];
          }
          for (std::size_t j = 0; j < hardRows.size(); ++j) {
            hardMass(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(j))
                = constraintMass(hardRow, hardRows[j]);
          }
        }
        const Eigen::VectorXd hardLambda = hardMass.ldlt().solve(hardRhs);
        for (std::size_t i = 0; i < hardRows.size(); ++i) {
          lambda[hardRows[i]] = hardLambda[static_cast<Eigen::Index>(i)];
        }
      }
      if (projectionLambda.size() != lambda.size()) {
        projectionLambda = lambda.cwiseAbs();
      } else {
        projectionLambda = projectionLambda.cwiseMax(lambda.cwiseAbs());
      }
      const Eigen::VectorXd correction = inverseMassJt * lambda;
      for (const VarLink& link : tree.links) {
        if (link.dof == 0) {
          continue;
        }
        const auto& joint = registry.get<comps::Joint>(link.joint);
        const auto seg = static_cast<Eigen::Index>(link.dofOffset);
        const auto n = static_cast<Eigen::Index>(link.dof);
        nextPosition.segment(seg, n) = jointRetract(
            joint, nextPosition.segment(seg, n), correction.segment(seg, n));
      }
    }
    if (projectionLambda.size() > 0) {
      markBrokenAvbdVariationalLoopConstraints(
          registry, constraints, projectionLambda);
    }
  }

  // Refresh the per-link scratch at the accepted configuration so the history
  // shift uses dT and momentum consistent with nextPosition. `residualAt` folds
  // in the contact force (if any) so the reported residual reflects the actual
  // forced-DEL root; the forward (dT/average-velocity) sweep is independent of
  // the forcing, so the history shift is unaffected by the hook.
  const Eigen::VectorXd finalResidual = residualAt(nextPosition);
  report.residualNorm = finalResidual.norm();

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
    const Eigen::VectorXd previousVelocity = joint.velocity;
    const Eigen::VectorXd newPosition = nextPosition.segment(seg, n);
    // q^k is the captured `position` (the loop-closure projection may have
    // already mutated `joint.position` to the candidate qNext).
    joint.velocity
        = jointLogDifference(joint, newPosition, position.segment(seg, n))
          / timeStep;
    joint.position = newPosition;
    if (previousVelocity.size() == n) {
      joint.acceleration = (joint.velocity - previousVelocity) / timeStep;
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
      constraints,
      andersonDepth,
      contactHook,
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
  if (structure.links.empty()) {
    return 0.0;
  }
  const VarTree tree = buildVarTree(registry, structure);

  Eigen::VectorXd position;
  Eigen::VectorXd velocity;
  Eigen::VectorXd appliedForce;
  gatherState(registry, tree, position, velocity, appliedForce);
  const std::vector<Vector6> v = currentSpatialVelocities(tree, velocity);

  double kinetic = 0.0;
  double potential = 0.0;
  for (std::size_t i = 0; i < tree.links.size(); ++i) {
    const auto& link = tree.links[i];
    kinetic += 0.5 * v[i].dot(link.inertia * v[i]);
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
  const std::vector<Eigen::MatrixXd> jacobians = bodyJacobians(tree);
  auto [g, jac]
      = constraintResidualAndJacobian(structure, tree, jacobians, constraints);
  result.residual = std::move(g);
  result.jacobian = std::move(jac);
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
    detail::WorldRegistry& registry, std::size_t multibodyCount)
{
  auto& stateStorage = registry.storage<MultibodyVariationalState>();
  auto& scratchStorage = registry.storage<MultibodyVariationalScratch>();
  stateStorage.reserve(multibodyCount);
  scratchStorage.reserve(multibodyCount);

  if (multibodyCount == 0u) {
    return;
  }

  auto view = registry.view<comps::MultibodyStructure>();
  auto closures = registry.view<comps::LoopClosure>();
  for (auto entity : view) {
    const auto& structure = view.get<comps::MultibodyStructure>(entity);
    auto& state = registry.get_or_emplace<MultibodyVariationalState>(entity);
    state.previousDeltaTransform.reserve(structure.links.size());
    state.previousMomentum.reserve(structure.links.size());

    auto& scratch
        = registry.get_or_emplace<MultibodyVariationalScratch>(entity);
    scratch.postContactTransforms.resize(structure.links.size());
    scratch.constraints.clear();
    for (auto closureEntity : closures) {
      const auto binding = bindVariationalLoopClosure(registry, closureEntity);
      if (binding.status == VariationalLoopClosureBinding::Status::Supported
          && binding.structure == entity) {
        scratch.constraints.push_back(binding.constraint);
      }
    }
    appendAvbdRigidWorldArticulatedPointJointConstraints(
        registry, entity, scratch.constraints);
    scratch.constraints.clear();

    const auto* contactConfig
        = registry.try_get<comps::VariationalContact>(entity);
    if (contactConfig == nullptr || contactConfig->pointLinkIndices.empty()) {
      continue;
    }

    configureGroundContactScratch(*contactConfig, scratch);
    if (scratch.groundContact.stiffness <= 0.0
        || scratch.groundContact.points.empty()) {
      continue;
    }
    normalizeGroundContact(scratch.groundContact);
    if (!scratch.groundContactSolver.has_value()) {
      scratch.groundContactSolver.emplace(scratch.groundContact);
    } else {
      scratch.groundContactSolver->resetContact(scratch.groundContact);
    }
    const VarTree tree = buildVarTree(registry, structure);
    if (tree.dofCount > 0u) {
      reserveContactEvaluationScratch(tree, scratch.contactEvaluation);
    }
    if (contactConfig->dualUpdateCadence == 0u) {
      continue;
    }

    auto& dualState
        = registry.get_or_emplace<comps::VariationalContactDualState>(entity);
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
  reserveMultibodyVariationalRegistryStorage(registry, multibodyCount);
}

//==============================================================================
void MultibodyVariationalIntegrationStage::execute(
    World& world, ComputeExecutor& /*executor*/)
{
  auto& registry = dart::simulation::detail::registryOf(world);
  const Eigen::Vector3d gravity = world.getGravity();
  const double timeStep = world.getTimeStep();

  // Gather enabled loop closures that request dynamic solving, grouped by the
  // multibody they constrain. World::step validates the dynamics policy first,
  // so by here every binding is Ignored or Supported (never Unsupported).
  auto closures = registry.view<comps::LoopClosure>();

  auto view = registry.view<comps::MultibodyStructure>();
  for (auto entity : view) {
    const auto& structure = view.get<comps::MultibodyStructure>(entity);
    auto& state = registry.get_or_emplace<MultibodyVariationalState>(entity);
    std::vector<VariationalCompliantLoopConstraint> compliantConstraints;
    auto& scratch
        = registry.get_or_emplace<MultibodyVariationalScratch>(entity);
    auto& constraints = scratch.constraints;
    constraints.clear();
    for (auto closureEntity : closures) {
      const auto binding = bindVariationalLoopClosure(registry, closureEntity);
      if (binding.status == VariationalLoopClosureBinding::Status::Supported
          && binding.structure == entity) {
        constraints.push_back(binding.constraint);
      }
    }
    appendAvbdRigidWorldArticulatedPointJointConstraints(
        registry, entity, constraints, &compliantConstraints);
    VariationalCompliantLoopScratch* compliantScratch
        = registry.try_get<VariationalCompliantLoopScratch>(entity);
    if (!compliantConstraints.empty()) {
      compliantScratch
          = &registry.get_or_emplace<VariationalCompliantLoopScratch>(entity);
      syncVariationalCompliantLoopConstraintRows(
          compliantConstraints, *compliantScratch);
    } else if (compliantScratch != nullptr) {
      compliantScratch->clear();
    }
    // Build the opt-in contact hook from the multibody's contact config
    // (PLAN-084 Phase C). cadence 0 => C1/C2 (lagged friction + compliant
    // penalty); cadence > 0 => the stateful C3 augmented-Lagrangian rung, whose
    // duals persist in VariationalContactDualState and advance on an outer-loop
    // cadence after the step. Absent => contact-free.
    const auto* contactConfig
        = registry.try_get<comps::VariationalContact>(entity);
    VariationalGroundContactSolver* groundContactSolver = nullptr;
    VariationalGroundContactSolver* alSolver = nullptr;
    std::size_t dualUpdateCadence = 0;
    VariationalContactHook contactHook;
    if (contactConfig != nullptr) {
      configureGroundContactScratch(*contactConfig, scratch);
      const auto& contact = scratch.groundContact;
      if (contact.stiffness > 0.0 && !contact.points.empty()) {
        normalizeGroundContact(scratch.groundContact);
        if (!scratch.groundContactSolver.has_value()) {
          scratch.groundContactSolver.emplace(contact);
        } else {
          scratch.groundContactSolver->resetContact(contact);
        }
        VariationalGroundContactSolver& solver = *scratch.groundContactSolver;
        groundContactSolver = &solver;
        if (contactConfig->dualUpdateCadence > 0) {
          // C3: a stateful AL solver seeded from the persisted (warm-started)
          // duals. The solver must outlive the integrate call and the later
          // dual update, so it is also held in `alSolver` here.
          dualUpdateCadence = contactConfig->dualUpdateCadence;
          auto& dualState
              = registry.get_or_emplace<comps::VariationalContactDualState>(
                  entity);
          if (dualState.duals.size() != contact.points.size()) {
            dualState.duals.assign(contact.points.size(), 0.0);
            dualState.stepsSinceDualUpdate = 0;
          }
          solver.setDuals(
              std::span<const double>{
                  dualState.duals.data(), dualState.duals.size()});
          alSolver = &solver;
        }
      }
    }
    contactHook = combineVariationalContactHooks(
        std::move(contactHook),
        makeVariationalCompliantLoopConstraintHook(compliantConstraints));
    integrateMultibodyVariationalImpl(
        registry,
        structure,
        gravity,
        timeStep,
        state,
        static_cast<int>(m_maxIterations),
        m_tolerance,
        constraints,
        5,
        contactHook,
        groundContactSolver != nullptr ? &scratch.groundContact : nullptr,
        groundContactSolver,
        &scratch.contactEvaluation);
    if (compliantScratch != nullptr && !compliantConstraints.empty()) {
      updateVariationalCompliantLoopConstraintRows(
          registry, structure, compliantConstraints, *compliantScratch);
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
        const VarTree postTree = buildVarTree(registry, structure);
        auto& postTransforms = scratch.postContactTransforms;
        postTransforms.resize(postTree.links.size());
        for (std::size_t i = 0; i < postTree.links.size(); ++i) {
          postTransforms[i] = postTree.links[i].worldTransform;
        }
        alSolver->updateDuals(postTransforms);
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
