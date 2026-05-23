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

#include "dart/simulation/experimental/body/contact.hpp"
#include "dart/simulation/experimental/common/exceptions.hpp"
#include "dart/simulation/experimental/comps/contact_material.hpp"
#include "dart/simulation/experimental/comps/dynamics.hpp"
#include "dart/simulation/experimental/comps/frame_types.hpp"
#include "dart/simulation/experimental/comps/joint.hpp"
#include "dart/simulation/experimental/comps/link.hpp"
#include "dart/simulation/experimental/comps/multi_body.hpp"
#include "dart/simulation/experimental/comps/rigid_body.hpp"
#include "dart/simulation/experimental/world.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Geometry>
#include <entt/entt.hpp>

#include <algorithm>
#include <unordered_map>
#include <vector>

#include <cmath>

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
    case comps::JointType::Screw:
      // Coupled rotation + translation along the axis: theta rotates and
      // advances by pitch * theta.
      transform.linear()
          = Eigen::AngleAxisd(joint.position[0], joint.axis).toRotationMatrix();
      transform.translation() = joint.axis * (joint.pitch * joint.position[0]);
      return transform;
    case comps::JointType::Universal:
      // Two sequential rotations: theta1 about axis, then theta2 about axis2
      // (axis2 expressed in the intermediate frame).
      transform.linear() = (Eigen::AngleAxisd(joint.position[0], joint.axis)
                            * Eigen::AngleAxisd(joint.position[1], joint.axis2))
                               .toRotationMatrix();
      return transform;
    default:
      DART_EXPERIMENTAL_THROW_T(
          InvalidOperationException,
          "Articulated-body forward dynamics is not yet implemented for this "
          "joint type; supported types are fixed, revolute, prismatic, screw, "
          "and universal");
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
    case comps::JointType::Screw: {
      // Twist of a screw: angular = axis, linear = pitch * axis.
      Subspace subspace(6, 1);
      subspace.col(0).head<3>() = joint.axis;
      subspace.col(0).tail<3>() = joint.axis * joint.pitch;
      return subspace;
    }
    case comps::JointType::Universal: {
      // Both columns expressed in the joint output frame (after both
      // rotations). Column 0 (theta1 about axis) is carried by the second
      // rotation, so it becomes R(theta2, axis2)^T * axis; column 1 (theta2
      // about axis2) is axis2 itself. This makes column 0 configuration
      // dependent, which the velocity-product term cJ accounts for.
      Subspace subspace(6, 2);
      const Eigen::Matrix3d secondRotation
          = Eigen::AngleAxisd(joint.position[1], joint.axis2)
                .toRotationMatrix();
      subspace.col(0).head<3>() = secondRotation.transpose() * joint.axis;
      subspace.col(0).tail<3>().setZero();
      subspace.col(1).head<3>() = joint.axis2;
      subspace.col(1).tail<3>().setZero();
      return subspace;
    }
    default:
      DART_EXPERIMENTAL_THROW_T(
          InvalidOperationException,
          "Articulated-body forward dynamics is not yet implemented for this "
          "joint type; supported types are fixed, revolute, prismatic, screw, "
          "and universal");
  }
}

//==============================================================================
// A single quadratic term of a joint's velocity-product bias cJ = Sdot qdot,
// expressed in the child link frame: contributes `coeff * qdot[a] * qdot[b]`,
// where a and b index the joint's local generalized coordinates. Constant
// subspaces (revolute, prismatic, screw) have no such terms; configuration-
// dependent multi-DOF subspaces (universal) do.
struct JointBiasTerm
{
  std::size_t a = 0;
  std::size_t b = 0;
  Vector6 coeff = Vector6::Zero();
};

//==============================================================================
// Velocity-product bias terms cJ = Sdot qdot for a joint, mapped into the child
// link frame by the post-joint offset adjoint (the same transform applied to
// the motion subspace).
std::vector<JointBiasTerm> jointBiasTerms(
    const comps::Joint& joint, const Matrix6& offsetAdjoint)
{
  switch (joint.type) {
    case comps::JointType::Universal: {
      // d/dt(col0) = (s1 x axis2) * theta2dot with s1 = R(theta2, axis2)^T
      // axis, so cJ = (s1 x axis2) * theta1dot * theta2dot (angular; linear
      // zero).
      const Eigen::Matrix3d secondRotation
          = Eigen::AngleAxisd(joint.position[1], joint.axis2)
                .toRotationMatrix();
      const Eigen::Vector3d s1 = secondRotation.transpose() * joint.axis;
      Vector6 coeff = Vector6::Zero();
      coeff.head<3>() = s1.cross(joint.axis2);
      return {JointBiasTerm{0, 1, offsetAdjoint * coeff}};
    }
    default:
      return {};
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
  std::vector<JointBiasTerm> biasTerms; // cJ = Sdot qdot (child frame)
  Matrix6 inertia = Matrix6::Zero();
  Eigen::Isometry3d worldTransform = Eigen::Isometry3d::Identity();
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
// A contact acting on one link of a multibody against an immovable obstacle.
struct LinkContact
{
  entt::entity link = entt::null;
  Eigen::Vector3d normal = Eigen::Vector3d::UnitZ(); // world, points into link
  Eigen::Vector3d point = Eigen::Vector3d::Zero();   // world contact point
  double depth = 0.0;
  double friction = 1.0;    // combined Coulomb friction coefficient
  double restitution = 0.0; // combined normal restitution coefficient
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
      dynamics.worldTransform = cache.worldTransform;
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
    dynamics.worldTransform
        = tree.links[parentIt->second].worldTransform * childInParent;

    const Subspace jointFrameSubspace = jointSubspaceInJointFrame(joint);
    const Matrix6 offsetAdjoint
        = adjoint(linkComp.transformFromParentJoint.inverse());
    dynamics.subspace = offsetAdjoint * jointFrameSubspace;
    dynamics.biasTerms = jointBiasTerms(joint, offsetAdjoint);
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
// Body-frame spatial Jacobian of every link via the recursion
// J_i = X_i J_parent, with the link's own joint columns set to its motion
// subspace S_i (already expressed in the link frame). Each Jacobian is 6 x
// dofCount, [angular; linear] in the link's own frame.
std::vector<Eigen::MatrixXd> linkBodyJacobians(const DynamicsTree& tree)
{
  const auto dofCount = static_cast<Eigen::Index>(tree.dofCount);
  std::vector<Eigen::MatrixXd> jacobian(
      tree.links.size(), Eigen::MatrixXd::Zero(6, dofCount));
  for (std::size_t i = 0; i < tree.links.size(); ++i) {
    const auto& link = tree.links[i];
    if (link.parentIndex >= 0) {
      jacobian[i] = link.parentToChild
                    * jacobian[static_cast<std::size_t>(link.parentIndex)];
    }
    if (link.dof > 0) {
      jacobian[i].middleCols(
          static_cast<Eigen::Index>(link.dofOffset),
          static_cast<Eigen::Index>(link.dof))
          = link.subspace;
    }
  }
  return jacobian;
}

//==============================================================================
// Index of a link entity within a multibody structure.
std::size_t linkIndexOf(
    const comps::MultiBodyStructure& structure, entt::entity linkEntity)
{
  const auto& linkEntities = structure.links;
  const auto it
      = std::find(linkEntities.begin(), linkEntities.end(), linkEntity);
  DART_EXPERIMENTAL_THROW_T_IF(
      it == linkEntities.end(),
      InvalidArgumentException,
      "Link does not belong to this multibody");
  return static_cast<std::size_t>(it - linkEntities.begin());
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
      // Velocity-product bias of a configuration-dependent joint subspace,
      // cJ = Sdot qdot (zero for constant-subspace joints).
      Vector6 jointBias = Vector6::Zero();
      for (const auto& term : link.biasTerms) {
        jointBias
            += term.coeff
               * (qdot[static_cast<Eigen::Index>(link.dofOffset + term.a)]
                  * qdot[static_cast<Eigen::Index>(link.dofOffset + term.b)]);
      }

      velocity[i] = link.parentToChild * velocity[parent] + jointVelocity;
      acceleration[i] = link.parentToChild * acceleration[parent]
                        + jointAcceleration + jointBias
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
    double timeStep,
    const std::vector<LinkContact>& linkContacts)
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

    // Determine the commanded actuation effort by actuator type. Force applies
    // the clamped joint effort; Passive applies none. Passive spring and
    // damping forces are not subject to the effort limits and always apply.
    Eigen::VectorXd effort;
    switch (joint.actuatorType) {
      case comps::ActuatorType::Force:
        effort = joint.torque.cwiseMax(joint.limits.effortLower)
                     .cwiseMin(joint.limits.effortUpper);
        break;
      case comps::ActuatorType::Passive:
      case comps::ActuatorType::Velocity:
        // Passive applies no commanded effort; Velocity is driven by a
        // velocity-level constraint solved after the unconstrained step.
        effort = Eigen::VectorXd::Zero(
            static_cast<Eigen::Index>(tree.links[i].dof));
        break;
      default:
        DART_EXPERIMENTAL_THROW_T(
            InvalidOperationException,
            "Joint actuator type is not yet implemented in the "
            "articulated-body forward dynamics; supported types are Force, "
            "Passive, and Velocity");
    }
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

  // Unconstrained next generalized velocity (semi-implicit Euler), then apply
  // velocity-level effects on the global vector before integrating positions.
  Eigen::VectorXd nextVelocity = qdot + qddot * timeStep;

  // Coulomb (dry) joint friction as a bounded velocity-level impulse: it stops
  // a coordinate when the holding impulse is within the friction bound
  // (stiction) and otherwise opposes motion at the friction magnitude.
  for (std::size_t i = 0; i < tree.links.size(); ++i) {
    const auto dof = tree.links[i].dof;
    if (dof == 0) {
      continue;
    }
    const auto& joint = registry.get<comps::Joint>(tree.jointOf[i]);
    if (joint.coulombFriction.size() != static_cast<Eigen::Index>(dof)) {
      continue;
    }
    for (std::size_t d = 0; d < dof; ++d) {
      const double bound
          = joint.coulombFriction[static_cast<Eigen::Index>(d)] * timeStep;
      if (bound <= 0.0) {
        continue;
      }
      const auto globalDof
          = static_cast<Eigen::Index>(tree.links[i].dofOffset + d);
      const double effInertia = mb.massMatrix(globalDof, globalDof);
      const double stopImpulse = effInertia * nextVelocity[globalDof];
      const double frictionImpulse = std::clamp(stopImpulse, -bound, bound);
      nextVelocity[globalDof] -= frictionImpulse / effInertia;
    }
  }

  // Velocity-actuated joints: solve a coupled velocity-level equality
  // constraint that drives the selected coordinates to their commanded
  // velocities, lambda = (J M^-1 J^T)^-1 (target - J nextVelocity),
  // nextVelocity += M^-1 J^T lambda, where J selects the constrained
  // coordinates.
  std::vector<Eigen::Index> constrainedDof;
  std::vector<double> constrainedTarget;
  for (std::size_t i = 0; i < tree.links.size(); ++i) {
    const auto dof = tree.links[i].dof;
    if (dof == 0) {
      continue;
    }
    const auto& joint = registry.get<comps::Joint>(tree.jointOf[i]);
    if (joint.actuatorType != comps::ActuatorType::Velocity) {
      continue;
    }
    for (std::size_t d = 0; d < dof; ++d) {
      constrainedDof.push_back(
          static_cast<Eigen::Index>(tree.links[i].dofOffset + d));
      constrainedTarget.push_back(
          joint.commandVelocity.size() == static_cast<Eigen::Index>(dof)
              ? joint.commandVelocity[static_cast<Eigen::Index>(d)]
              : 0.0);
    }
  }
  if (!constrainedDof.empty()) {
    const Eigen::MatrixXd inverseMass = mb.massMatrix.inverse();
    const auto k = static_cast<Eigen::Index>(constrainedDof.size());
    Eigen::MatrixXd constraintMatrix(k, k);
    Eigen::VectorXd residual(k);
    for (Eigen::Index a = 0; a < k; ++a) {
      residual[a] = constrainedTarget[static_cast<std::size_t>(a)]
                    - nextVelocity[constrainedDof[static_cast<std::size_t>(a)]];
      for (Eigen::Index b = 0; b < k; ++b) {
        constraintMatrix(a, b) = inverseMass(
            constrainedDof[static_cast<std::size_t>(a)],
            constrainedDof[static_cast<std::size_t>(b)]);
      }
    }
    const Eigen::VectorXd lambda = constraintMatrix.ldlt().solve(residual);
    for (Eigen::Index a = 0; a < k; ++a) {
      nextVelocity
          += inverseMass.col(constrainedDof[static_cast<std::size_t>(a)])
             * lambda[a];
    }
  }

  // Resolve contacts on this multibody's links against immovable obstacles with
  // a sequential, unilateral normal-impulse solve (one-sided: the obstacle does
  // not move). For a contact on link `i` with world normal `n` (pointing into
  // the link) at point `p`, the contact-point normal Jacobian is
  // `jn = (R J_point_linear)^T n`, where `J_point_linear` shifts the link's
  // body linear Jacobian to `p`. A Baumgarte bias removes residual penetration.
  if (!linkContacts.empty()) {
    const Eigen::MatrixXd inverseMass = mb.massMatrix.inverse();
    const std::vector<Eigen::MatrixXd> bodyJacobian = linkBodyJacobians(tree);
    constexpr double penetrationSlop = 1e-4;
    constexpr double baumgarteFactor = 0.2;
    constexpr int contactIterations = 8;

    // Precompute the contact-space Jacobians (normal + two tangents) once.
    struct ContactRow
    {
      Eigen::VectorXd normalJacobian;
      Eigen::VectorXd tangentJacobian1;
      Eigen::VectorXd tangentJacobian2;
      double normalDenominator = 0.0;
      double tangentDenominator1 = 0.0;
      double tangentDenominator2 = 0.0;
      double bias = 0.0;
      double restitutionTarget = 0.0;
      double friction = 1.0;
      double normalImpulse = 0.0;
      double tangentImpulse1 = 0.0;
      double tangentImpulse2 = 0.0;
      bool active = false;
    };
    std::vector<ContactRow> rows(linkContacts.size());

    for (std::size_t c = 0; c < linkContacts.size(); ++c) {
      const auto& contact = linkContacts[c];
      const auto linkIt = std::find(
          structure.links.begin(), structure.links.end(), contact.link);
      if (linkIt == structure.links.end()) {
        continue;
      }
      const auto index
          = static_cast<std::size_t>(linkIt - structure.links.begin());

      const Eigen::Matrix3d rotation
          = tree.links[index].worldTransform.linear();
      const Eigen::Vector3d origin
          = tree.links[index].worldTransform.translation();
      const Eigen::MatrixXd angularJacobian
          = rotation * bodyJacobian[index].topRows(3);
      const Eigen::MatrixXd pointLinearJacobian
          = rotation * bodyJacobian[index].bottomRows(3)
            - skew(contact.point - origin) * angularJacobian;

      auto& row = rows[c];
      row.normalJacobian = pointLinearJacobian.transpose() * contact.normal;
      row.normalDenominator
          = row.normalJacobian.dot(inverseMass * row.normalJacobian);
      if (row.normalDenominator <= 0.0) {
        continue; // the contact cannot move this link (e.g. fixed base)
      }

      // Two tangent directions orthogonal to the contact normal.
      const Eigen::Vector3d normal = contact.normal.normalized();
      const Eigen::Vector3d reference = std::abs(normal.x()) < 0.9
                                            ? Eigen::Vector3d::UnitX()
                                            : Eigen::Vector3d::UnitY();
      const Eigen::Vector3d tangent1 = normal.cross(reference).normalized();
      const Eigen::Vector3d tangent2 = normal.cross(tangent1);
      row.tangentJacobian1 = pointLinearJacobian.transpose() * tangent1;
      row.tangentJacobian2 = pointLinearJacobian.transpose() * tangent2;
      row.tangentDenominator1
          = row.tangentJacobian1.dot(inverseMass * row.tangentJacobian1);
      row.tangentDenominator2
          = row.tangentJacobian2.dot(inverseMass * row.tangentJacobian2);
      row.bias = baumgarteFactor
                 * std::max(0.0, contact.depth - penetrationSlop) / timeStep;
      row.friction = contact.friction;

      // Restitution target: rebound at -e * (approaching normal velocity),
      // ignoring slow approaches to avoid jitter at rest.
      constexpr double restitutionThreshold = 1e-2;
      const double approachingVelocity = row.normalJacobian.dot(nextVelocity);
      row.restitutionTarget = (approachingVelocity < -restitutionThreshold)
                                  ? -contact.restitution * approachingVelocity
                                  : 0.0;
      row.active = true;
    }

    for (int iteration = 0; iteration < contactIterations; ++iteration) {
      for (auto& row : rows) {
        if (!row.active) {
          continue;
        }

        // Normal impulse with accumulation (unilateral: lambda_n >= 0).
        const double normalVelocity = row.normalJacobian.dot(nextVelocity);
        const double deltaNormal
            = (-normalVelocity + std::max(row.bias, row.restitutionTarget))
              / row.normalDenominator;
        const double newNormal = std::max(0.0, row.normalImpulse + deltaNormal);
        nextVelocity += inverseMass * row.normalJacobian
                        * (newNormal - row.normalImpulse);
        row.normalImpulse = newNormal;

        // Two-tangent Coulomb friction, each impulse bounded by mu * lambda_n.
        const double bound = row.friction * row.normalImpulse;
        if (row.tangentDenominator1 > 0.0) {
          const double vt = row.tangentJacobian1.dot(nextVelocity);
          const double newImpulse = std::clamp(
              row.tangentImpulse1 - vt / row.tangentDenominator1,
              -bound,
              bound);
          nextVelocity += inverseMass * row.tangentJacobian1
                          * (newImpulse - row.tangentImpulse1);
          row.tangentImpulse1 = newImpulse;
        }
        if (row.tangentDenominator2 > 0.0) {
          const double vt = row.tangentJacobian2.dot(nextVelocity);
          const double newImpulse = std::clamp(
              row.tangentImpulse2 - vt / row.tangentDenominator2,
              -bound,
              bound);
          nextVelocity += inverseMass * row.tangentJacobian2
                          * (newImpulse - row.tangentImpulse2);
          row.tangentImpulse2 = newImpulse;
        }
      }
    }
  }

  // Enforce velocity limits by clamping the generalized velocity.
  for (std::size_t i = 0; i < tree.links.size(); ++i) {
    const auto dof = tree.links[i].dof;
    if (dof == 0) {
      continue;
    }
    const auto& joint = registry.get<comps::Joint>(tree.jointOf[i]);
    if (joint.limits.velocityLower.size() != static_cast<Eigen::Index>(dof)
        || joint.limits.velocityUpper.size()
               != static_cast<Eigen::Index>(dof)) {
      continue;
    }
    for (std::size_t d = 0; d < dof; ++d) {
      const auto globalDof
          = static_cast<Eigen::Index>(tree.links[i].dofOffset + d);
      nextVelocity[globalDof] = std::clamp(
          nextVelocity[globalDof],
          joint.limits.velocityLower[static_cast<Eigen::Index>(d)],
          joint.limits.velocityUpper[static_cast<Eigen::Index>(d)]);
    }
  }

  // Write back the new velocity/acceleration and integrate positions, applying
  // position limits as hard stops (clamp the coordinate and arrest the velocity
  // component driving it past the limit).
  for (std::size_t i = 0; i < tree.links.size(); ++i) {
    if (tree.links[i].dof == 0) {
      continue;
    }
    auto& joint = registry.get<comps::Joint>(tree.jointOf[i]);
    joint.acceleration
        = qddot.segment(tree.links[i].dofOffset, tree.links[i].dof);
    joint.velocity
        = nextVelocity.segment(tree.links[i].dofOffset, tree.links[i].dof);
    joint.position += joint.velocity * timeStep;

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
Eigen::VectorXd computeMultiBodyInverseDynamics(
    entt::registry& registry,
    const comps::MultiBodyStructure& structure,
    const Eigen::Vector3d& gravity,
    const Eigen::VectorXd& desiredAcceleration)
{
  if (structure.links.empty()) {
    return {};
  }

  const DynamicsTree tree = buildDynamicsTree(registry, structure);
  if (tree.dofCount == 0) {
    return {};
  }

  DART_EXPERIMENTAL_THROW_T_IF(
      desiredAcceleration.size() != static_cast<Eigen::Index>(tree.dofCount),
      InvalidArgumentException,
      "Desired acceleration dimension ({}) must match the multibody DOF count "
      "({})",
      desiredAcceleration.size(),
      tree.dofCount);

  Eigen::VectorXd qdot
      = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(tree.dofCount));
  for (std::size_t i = 0; i < tree.links.size(); ++i) {
    if (tree.links[i].dof == 0) {
      continue;
    }
    const auto& joint = registry.get<comps::Joint>(tree.jointOf[i]);
    qdot.segment(tree.links[i].dofOffset, tree.links[i].dof) = joint.velocity;
  }

  Vector6 baseAcceleration = Vector6::Zero();
  baseAcceleration.tail<3>() = -gravity;

  Eigen::VectorXd tau = recursiveNewtonEuler(
      tree.links, baseAcceleration, tree.dofCount, desiredAcceleration, qdot);

  // Armature contributes diag(armature) * qddot to the joint forces.
  if (tree.armature.size() == desiredAcceleration.size()) {
    tau += tree.armature.cwiseProduct(desiredAcceleration);
  }

  return tau;
}

//==============================================================================
Eigen::MatrixXd computeMultiBodyLinkJacobian(
    entt::registry& registry,
    const comps::MultiBodyStructure& structure,
    entt::entity linkEntity)
{
  const auto targetIndex = linkIndexOf(structure, linkEntity);
  const DynamicsTree tree = buildDynamicsTree(registry, structure);
  return linkBodyJacobians(tree)[targetIndex];
}

//==============================================================================
Eigen::MatrixXd computeMultiBodyLinkWorldJacobian(
    entt::registry& registry,
    const comps::MultiBodyStructure& structure,
    entt::entity linkEntity)
{
  const auto targetIndex = linkIndexOf(structure, linkEntity);
  const DynamicsTree tree = buildDynamicsTree(registry, structure);
  const Eigen::MatrixXd bodyJacobian = linkBodyJacobians(tree)[targetIndex];

  // Rotate both the angular and linear blocks into world axes, keeping the link
  // origin as the reference point: this is the geometric (world-frame) Jacobian
  // [angular; linear of the link origin].
  const Eigen::Matrix3d rotation
      = tree.links[targetIndex].worldTransform.linear();
  Eigen::MatrixXd worldJacobian(6, bodyJacobian.cols());
  worldJacobian.topRows<3>() = rotation * bodyJacobian.topRows<3>();
  worldJacobian.bottomRows<3>() = rotation * bodyJacobian.bottomRows<3>();
  return worldJacobian;
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

  // Collision query once per step; route each contact that touches a link to
  // the multibody that owns it. Only link-vs-static-rigid-body contacts are
  // resolved here (one-sided); link-vs-dynamic and link-vs-link contacts are a
  // later, two-sided slice.
  const std::vector<Contact> contacts = world.collide();

  const auto isStaticRigidBody = [&](entt::entity entity) {
    return registry.all_of<comps::RigidBodyTag, comps::StaticBodyTag>(entity);
  };
  const auto frictionOf = [&](entt::entity entity) {
    if (const auto* material
        = registry.try_get<comps::ContactMaterial>(entity)) {
      return material->friction;
    }
    return 1.0;
  };
  const auto restitutionOf = [&](entt::entity entity) {
    if (const auto* material
        = registry.try_get<comps::ContactMaterial>(entity)) {
      return material->restitution;
    }
    return 0.0;
  };

  auto view = registry.view<comps::MultiBodyStructure>();
  for (auto entity : view) {
    const auto& structure = view.get<comps::MultiBodyStructure>(entity);

    std::vector<LinkContact> linkContacts;
    for (const auto& contact : contacts) {
      const auto entityA = contact.bodyA.getEntity();
      const auto entityB = contact.bodyB.getEntity();
      const auto& links = structure.links;
      const bool aInBody
          = std::find(links.begin(), links.end(), entityA) != links.end();
      const bool bInBody
          = std::find(links.begin(), links.end(), entityB) != links.end();
      const double friction
          = std::sqrt(frictionOf(entityA) * frictionOf(entityB));
      const double restitution
          = std::max(restitutionOf(entityA), restitutionOf(entityB));

      // The contact normal points bodyA -> bodyB; orient it into the link.
      if (aInBody && !bInBody && isStaticRigidBody(entityB)) {
        linkContacts.push_back(
            {entityA,
             -contact.normal,
             contact.point,
             contact.depth,
             friction,
             restitution});
      } else if (bInBody && !aInBody && isStaticRigidBody(entityA)) {
        linkContacts.push_back(
            {entityB,
             contact.normal,
             contact.point,
             contact.depth,
             friction,
             restitution});
      }
    }

    simulateMultiBody(registry, structure, gravity, timeStep, linkContacts);
  }
}

} // namespace dart::simulation::experimental::compute
