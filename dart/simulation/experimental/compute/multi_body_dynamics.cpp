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

#include "dart/simulation/experimental/compute/multi_body_dynamics.hpp"

#include "dart/simulation/experimental/common/exceptions.hpp"
#include "dart/simulation/experimental/comps/dynamics.hpp"
#include "dart/simulation/experimental/comps/frame_types.hpp"
#include "dart/simulation/experimental/comps/joint.hpp"
#include "dart/simulation/experimental/comps/link.hpp"
#include "dart/simulation/experimental/comps/multi_body.hpp"
#include "dart/simulation/experimental/world.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Geometry>
#include <entt/entt.hpp>

#include <algorithm>
#include <unordered_map>
#include <vector>

namespace dart::simulation::experimental::compute {

namespace {

using Vector6 = Eigen::Matrix<double, 6, 1>;
using Matrix6 = Eigen::Matrix<double, 6, 6>;
using Subspace = Eigen::Matrix<double, 6, Eigen::Dynamic>;

//==============================================================================
Eigen::Matrix3d skew(const Eigen::Vector3d& v)
{
  Eigen::Matrix3d m;
  m << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
  return m;
}

//==============================================================================
// Spatial motion adjoint for the [angular; linear] convention. For a transform
// T = (R, p) that expresses frame B in frame A (x_A = R x_B + p), this maps a
// spatial motion vector from B to A.
Matrix6 adjoint(const Eigen::Isometry3d& transform)
{
  const Eigen::Matrix3d rotation = transform.linear();
  const Eigen::Vector3d translation = transform.translation();

  Matrix6 result = Matrix6::Zero();
  result.topLeftCorner<3, 3>() = rotation;
  result.bottomLeftCorner<3, 3>() = skew(translation) * rotation;
  result.bottomRightCorner<3, 3>() = rotation;
  return result;
}

//==============================================================================
// Spatial motion cross product (crm) for V = [w; v].
Matrix6 motionCross(const Vector6& value)
{
  const Eigen::Vector3d angular = value.head<3>();
  const Eigen::Vector3d linear = value.tail<3>();

  Matrix6 result = Matrix6::Zero();
  result.topLeftCorner<3, 3>() = skew(angular);
  result.bottomLeftCorner<3, 3>() = skew(linear);
  result.bottomRightCorner<3, 3>() = skew(angular);
  return result;
}

//==============================================================================
// Spatial force cross product (crf) = -crm(V)^T for V = [w; v].
Matrix6 forceCross(const Vector6& value)
{
  return -motionCross(value).transpose();
}

//==============================================================================
Matrix6 spatialInertia(const comps::MassProperties& mass)
{
  // The center of mass is assumed at the link frame origin, so the spatial
  // inertia is block diagonal in the [angular; linear] convention.
  Matrix6 result = Matrix6::Zero();
  result.topLeftCorner<3, 3>() = mass.inertia;
  result.bottomRightCorner<3, 3>() = mass.mass * Eigen::Matrix3d::Identity();
  return result;
}

//==============================================================================
Eigen::Isometry3d jointMotionTransform(const comps::Joint& joint)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  switch (joint.type) {
    case comps::JointType::Fixed:
      return transform;
    case comps::JointType::Revolute:
      transform.linear()
          = Eigen::AngleAxisd(joint.position[0], joint.axis).toRotationMatrix();
      return transform;
    case comps::JointType::Prismatic:
      transform.translation() = joint.axis * joint.position[0];
      return transform;
    default:
      DART_EXPERIMENTAL_THROW_T(
          InvalidOperationException,
          "Articulated-body forward dynamics is not yet implemented for this "
          "joint type; supported types are fixed, revolute, and prismatic");
  }
}

//==============================================================================
// Joint motion subspace expressed in the joint frame (before the post-joint
// link offset), in the [angular; linear] convention.
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
    default:
      DART_EXPERIMENTAL_THROW_T(
          InvalidOperationException,
          "Articulated-body forward dynamics is not yet implemented for this "
          "joint type; supported types are fixed, revolute, and prismatic");
  }
}

//==============================================================================
// Per-link data precomputed at the current configuration. Index 0..count-1 in
// multibody construction order, which is guaranteed parent-before-child.
struct LinkDynamics
{
  int parentIndex = -1; // -1 for the fixed base (root)
  entt::entity jointEntity = entt::null;
  std::size_t dof = 0;
  std::size_t dofOffset = 0;
  Matrix6 parentToChild = Matrix6::Identity();      // X = Ad(T^{-1})
  Matrix6 childToParentForce = Matrix6::Identity(); // X^T
  Subspace subspace{6, 0};                          // S in child frame
  Matrix6 inertia = Matrix6::Zero();
};

//==============================================================================
// A multibody's per-link dynamics precomputed at the current configuration,
// plus the joint owning each link and the total movable DOF count.
struct DynamicsTree
{
  std::vector<LinkDynamics> links;
  std::vector<entt::entity> jointOf;
  std::size_t dofCount = 0;
  Eigen::VectorXd armature; // Rotor inertia per generalized coordinate.
};

//==============================================================================
// Build the per-link spatial dynamics for a multibody at its current
// configuration. Links are in construction order (parent-before-child).
DynamicsTree buildDynamicsTree(
    const entt::registry& registry, const comps::MultiBodyStructure& structure)
{
  const auto& linkEntities = structure.links;

  DynamicsTree tree;
  tree.links.assign(linkEntities.size(), LinkDynamics{});
  tree.jointOf.assign(
      linkEntities.size(), static_cast<entt::entity>(entt::null));

  std::unordered_map<entt::entity, std::size_t> indexOf;
  indexOf.reserve(linkEntities.size());
  for (std::size_t i = 0; i < linkEntities.size(); ++i) {
    indexOf.emplace(linkEntities[i], i);
  }

  for (std::size_t i = 0; i < linkEntities.size(); ++i) {
    const auto linkEntity = linkEntities[i];
    const auto& linkComp = registry.get<comps::Link>(linkEntity);
    auto& dynamics = tree.links[i];
    dynamics.inertia = spatialInertia(linkComp.mass);

    if (linkComp.parentJoint == entt::null) {
      // Fixed base: map the world base acceleration into the root frame.
      const auto& cache = registry.get<comps::FrameCache>(linkEntity);
      dynamics.parentToChild = adjoint(cache.worldTransform.inverse());
      dynamics.childToParentForce = dynamics.parentToChild.transpose();
      dynamics.parentIndex = -1;
      continue;
    }

    const auto& joint = registry.get<comps::Joint>(linkComp.parentJoint);
    const auto parentIt = indexOf.find(joint.parentLink);
    DART_EXPERIMENTAL_THROW_T_IF(
        parentIt == indexOf.end(),
        InvalidOperationException,
        "Multibody link parent is not part of the same multibody");
    dynamics.parentIndex = static_cast<int>(parentIt->second);
    dynamics.jointEntity = linkComp.parentJoint;
    tree.jointOf[i] = linkComp.parentJoint;

    const Eigen::Isometry3d childInParent
        = jointMotionTransform(joint) * linkComp.transformFromParentJoint;
    dynamics.parentToChild = adjoint(childInParent.inverse());
    dynamics.childToParentForce = dynamics.parentToChild.transpose();

    const Subspace jointFrameSubspace = jointSubspaceInJointFrame(joint);
    dynamics.subspace = adjoint(linkComp.transformFromParentJoint.inverse())
                        * jointFrameSubspace;
    dynamics.dof = static_cast<std::size_t>(jointFrameSubspace.cols());
    dynamics.dofOffset = tree.dofCount;
    tree.dofCount += dynamics.dof;
  }

  // Gather per-coordinate rotor inertia (armature) in generalized-coordinate
  // order; defaults to zero for any joint without an armature set.
  tree.armature
      = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(tree.dofCount));
  for (std::size_t i = 0; i < tree.links.size(); ++i) {
    const auto dof = tree.links[i].dof;
    if (dof == 0) {
      continue;
    }
    const auto& joint = registry.get<comps::Joint>(tree.jointOf[i]);
    if (joint.armature.size() == static_cast<Eigen::Index>(dof)) {
      tree.armature.segment(tree.links[i].dofOffset, dof) = joint.armature;
    }
  }

  return tree;
}

//==============================================================================
// Recursive Newton-Euler inverse dynamics over a precomputed tree.
// Returns generalized forces for the supplied qddot and qdot.
Eigen::VectorXd recursiveNewtonEuler(
    const std::vector<LinkDynamics>& links,
    const Vector6& baseAcceleration,
    std::size_t dofCount,
    const Eigen::VectorXd& qddot,
    const Eigen::VectorXd& qdot)
{
  const auto count = links.size();
  std::vector<Vector6> velocity(count, Vector6::Zero());
  std::vector<Vector6> acceleration(count, Vector6::Zero());
  std::vector<Vector6> force(count, Vector6::Zero());

  for (std::size_t i = 0; i < count; ++i) {
    const auto& link = links[i];
    if (link.parentIndex < 0) {
      velocity[i].setZero();
      acceleration[i] = link.parentToChild * baseAcceleration;
    } else {
      const auto parent = static_cast<std::size_t>(link.parentIndex);
      Vector6 jointVelocity = Vector6::Zero();
      Vector6 jointAcceleration = Vector6::Zero();
      if (link.dof > 0) {
        jointVelocity = link.subspace * qdot.segment(link.dofOffset, link.dof);
        jointAcceleration
            = link.subspace * qddot.segment(link.dofOffset, link.dof);
      }
      velocity[i] = link.parentToChild * velocity[parent] + jointVelocity;
      acceleration[i] = link.parentToChild * acceleration[parent]
                        + jointAcceleration
                        + motionCross(velocity[i]) * jointVelocity;
    }
    force[i] = link.inertia * acceleration[i]
               + forceCross(velocity[i]) * (link.inertia * velocity[i]);
  }

  Eigen::VectorXd tau
      = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(dofCount));
  for (std::size_t reverse = 0; reverse < count; ++reverse) {
    const auto i = count - 1 - reverse;
    const auto& link = links[i];
    if (link.dof > 0) {
      tau.segment(link.dofOffset, link.dof)
          = link.subspace.transpose() * force[i];
    }
    if (link.parentIndex >= 0) {
      force[static_cast<std::size_t>(link.parentIndex)]
          += link.childToParentForce * force[i];
    }
  }

  return tau;
}

//==============================================================================
// Joint-space mass matrix and bias (Coriolis/centrifugal + gravity) forces for
// a precomputed tree at the supplied generalized velocity.
struct MassAndBias
{
  Eigen::MatrixXd massMatrix;  // M(q)
  Eigen::VectorXd bias;        // C(q, qdot) qdot + g(q)
  Eigen::VectorXd gravityOnly; // g(q)
};

MassAndBias computeMassAndBias(
    const std::vector<LinkDynamics>& links,
    std::size_t dofCount,
    const Eigen::Vector3d& gravity,
    const Eigen::VectorXd& qdot,
    const Eigen::VectorXd& armature)
{
  Vector6 baseAcceleration = Vector6::Zero();
  baseAcceleration.tail<3>() = -gravity;

  const Eigen::VectorXd zero
      = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(dofCount));

  MassAndBias result;

  // Coriolis + gravity bias forces at the current velocity.
  result.bias
      = recursiveNewtonEuler(links, baseAcceleration, dofCount, zero, qdot);

  // Static gravity-only forces (zero velocity) used to isolate the mass matrix.
  result.gravityOnly
      = recursiveNewtonEuler(links, baseAcceleration, dofCount, zero, zero);

  // Joint-space mass matrix via unit-acceleration columns.
  result.massMatrix.resize(
      static_cast<Eigen::Index>(dofCount), static_cast<Eigen::Index>(dofCount));
  for (std::size_t column = 0; column < dofCount; ++column) {
    Eigen::VectorXd unit
        = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(dofCount));
    unit[static_cast<Eigen::Index>(column)] = 1.0;
    const Eigen::VectorXd response
        = recursiveNewtonEuler(links, baseAcceleration, dofCount, unit, zero);
    result.massMatrix.col(static_cast<Eigen::Index>(column))
        = response - result.gravityOnly;
  }

  // Rotor inertia (armature) adds to the joint-space mass-matrix diagonal.
  if (armature.size() == static_cast<Eigen::Index>(dofCount)) {
    result.massMatrix.diagonal() += armature;
  }

  return result;
}

//==============================================================================
void simulateMultiBody(
    entt::registry& registry,
    const comps::MultiBodyStructure& structure,
    const Eigen::Vector3d& gravity,
    double timeStep)
{
  if (structure.links.empty()) {
    return;
  }

  const DynamicsTree tree = buildDynamicsTree(registry, structure);
  if (tree.dofCount == 0) {
    return;
  }

  // Current generalized velocity plus applied and passive (spring/damping)
  // generalized efforts.
  Eigen::VectorXd qdot
      = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(tree.dofCount));
  Eigen::VectorXd appliedForce
      = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(tree.dofCount));
  for (std::size_t i = 0; i < tree.links.size(); ++i) {
    if (tree.links[i].dof == 0) {
      continue;
    }
    const auto& joint = registry.get<comps::Joint>(tree.jointOf[i]);
    qdot.segment(tree.links[i].dofOffset, tree.links[i].dof) = joint.velocity;

    // Clamp the commanded actuation effort to its limits; passive spring and
    // damping forces are not subject to the effort limits.
    const Eigen::VectorXd effort
        = joint.torque.cwiseMax(joint.limits.effortLower)
              .cwiseMin(joint.limits.effortUpper);
    appliedForce.segment(tree.links[i].dofOffset, tree.links[i].dof)
        = effort
          - joint.springStiffness.cwiseProduct(
              joint.position - joint.restPosition)
          - joint.dampingCoefficient.cwiseProduct(joint.velocity);
  }

  const MassAndBias mb = computeMassAndBias(
      tree.links, tree.dofCount, gravity, qdot, tree.armature);

  const Eigen::VectorXd qddot
      = mb.massMatrix.ldlt().solve(appliedForce - mb.bias);

  // Semi-implicit Euler integration of the generalized coordinates.
  for (std::size_t i = 0; i < tree.links.size(); ++i) {
    if (tree.links[i].dof == 0) {
      continue;
    }
    auto& joint = registry.get<comps::Joint>(tree.jointOf[i]);
    const auto block
        = qddot.segment(tree.links[i].dofOffset, tree.links[i].dof);
    joint.acceleration = block;
    joint.velocity += block * timeStep;

    // Apply Coulomb (dry) joint friction as a bounded velocity-level impulse:
    // it stops the coordinate when the holding impulse is within the friction
    // bound (stiction) and otherwise opposes motion at the friction magnitude.
    if (joint.coulombFriction.size() == joint.velocity.size()) {
      for (Eigen::Index d = 0; d < joint.velocity.size(); ++d) {
        const double bound = joint.coulombFriction[d] * timeStep;
        if (bound <= 0.0) {
          continue;
        }
        const auto globalDof
            = static_cast<Eigen::Index>(tree.links[i].dofOffset) + d;
        const double effInertia = mb.massMatrix(globalDof, globalDof);
        const double stopImpulse = effInertia * joint.velocity[d];
        const double frictionImpulse = std::clamp(stopImpulse, -bound, bound);
        joint.velocity[d] -= frictionImpulse / effInertia;
      }
    }

    // Enforce velocity limits by clamping the generalized velocity.
    if (joint.limits.velocityLower.size() == joint.velocity.size()
        && joint.limits.velocityUpper.size() == joint.velocity.size()) {
      joint.velocity = joint.velocity.cwiseMax(joint.limits.velocityLower)
                           .cwiseMin(joint.limits.velocityUpper);
    }

    joint.position += joint.velocity * timeStep;

    // Enforce position limits as hard stops: clamp the coordinate and arrest
    // the velocity component driving it past the limit.
    if (joint.limits.lower.size() == joint.position.size()
        && joint.limits.upper.size() == joint.position.size()) {
      for (Eigen::Index d = 0; d < joint.position.size(); ++d) {
        if (joint.position[d] < joint.limits.lower[d]) {
          joint.position[d] = joint.limits.lower[d];
          joint.velocity[d] = std::max(joint.velocity[d], 0.0);
        } else if (joint.position[d] > joint.limits.upper[d]) {
          joint.position[d] = joint.limits.upper[d];
          joint.velocity[d] = std::min(joint.velocity[d], 0.0);
        }
      }
    }
  }
}

} // namespace

//==============================================================================
MultiBodyDynamicsTerms computeMultiBodyDynamicsTerms(
    entt::registry& registry,
    const comps::MultiBodyStructure& structure,
    const Eigen::Vector3d& gravity)
{
  MultiBodyDynamicsTerms terms;
  if (structure.links.empty()) {
    return terms;
  }

  const DynamicsTree tree = buildDynamicsTree(registry, structure);
  if (tree.dofCount == 0) {
    return terms;
  }

  Eigen::VectorXd qdot
      = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(tree.dofCount));
  for (std::size_t i = 0; i < tree.links.size(); ++i) {
    if (tree.links[i].dof == 0) {
      continue;
    }
    const auto& joint = registry.get<comps::Joint>(tree.jointOf[i]);
    qdot.segment(tree.links[i].dofOffset, tree.links[i].dof) = joint.velocity;
  }

  const MassAndBias mb = computeMassAndBias(
      tree.links, tree.dofCount, gravity, qdot, tree.armature);

  terms.massMatrix = mb.massMatrix;
  terms.gravityForces = mb.gravityOnly;
  terms.coriolisForces = mb.bias - mb.gravityOnly;
  return terms;
}

//==============================================================================
std::string_view MultiBodyForwardDynamicsStage::getName() const noexcept
{
  return "multi_body_forward_dynamics";
}

//==============================================================================
ComputeStageMetadata MultiBodyForwardDynamicsStage::getMetadata() const noexcept
{
  return {
      ComputeStageDomain::ArticulatedBody,
      toMask(ComputeStageAcceleration::TaskParallel)};
}

//==============================================================================
void MultiBodyForwardDynamicsStage::execute(
    World& world, ComputeExecutor& /*executor*/)
{
  auto& registry = world.getRegistry();
  const Eigen::Vector3d gravity = world.getGravity();
  const double timeStep = world.getTimeStep();

  auto view = registry.view<comps::MultiBodyStructure>();
  for (auto entity : view) {
    const auto& structure = view.get<comps::MultiBodyStructure>(entity);
    simulateMultiBody(registry, structure, gravity, timeStep);
  }
}

} // namespace dart::simulation::experimental::compute
