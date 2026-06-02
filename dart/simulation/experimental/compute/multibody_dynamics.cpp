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

#include "dart/simulation/experimental/compute/multibody_dynamics.hpp"

#include "dart/simulation/experimental/body/contact.hpp"
#include "dart/simulation/experimental/common/exceptions.hpp"
#include "dart/simulation/experimental/comps/contact_material.hpp"
#include "dart/simulation/experimental/comps/dynamics.hpp"
#include "dart/simulation/experimental/comps/frame_types.hpp"
#include "dart/simulation/experimental/comps/joint.hpp"
#include "dart/simulation/experimental/comps/link.hpp"
#include "dart/simulation/experimental/comps/multibody.hpp"
#include "dart/simulation/experimental/comps/rigid_body.hpp"
#include "dart/simulation/experimental/compute/multibody_constraint.hpp"
#include "dart/simulation/experimental/compute/unified_constraint.hpp"
#include "dart/simulation/experimental/detail/entity_conversion.hpp"
#include "dart/simulation/experimental/detail/multibody_spatial_algebra.hpp"
#include "dart/simulation/experimental/world.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Geometry>
#include <entt/entt.hpp>

#include <algorithm>
#include <span>
#include <unordered_map>
#include <utility>
#include <vector>

#include <cmath>

namespace dart::simulation::experimental::compute {

namespace {

// Shared 6D spatial-algebra primitives (skew/adjoint/spatialInertia) and the
// Vector6/Matrix6/Subspace aliases live in
// detail/multibody_spatial_algebra.hpp.
using detail::adjoint;
using detail::Matrix6;
using detail::skew;
using detail::spatialInertia;
using detail::Subspace;
using detail::Vector6;

//==============================================================================
bool hasPrescribedRigidBodyContactResponse(
    const entt::registry& registry, entt::entity entity)
{
  return registry.all_of<comps::StaticBodyTag>(entity)
         || registry.all_of<comps::KinematicBodyTag>(entity);
}

//==============================================================================
// SO(3) exponential map: a rotation vector (axis * angle) to a rotation matrix.
Eigen::Matrix3d rotationExp(const Eigen::Vector3d& rotationVector)
{
  const double angle = rotationVector.norm();
  if (angle < 1e-12) {
    return Eigen::Matrix3d::Identity();
  }
  return Eigen::AngleAxisd(angle, rotationVector / angle).toRotationMatrix();
}

//==============================================================================
// Motion cross product crm(m) applied to a motion vector x, without forming the
// 6x6 matrix: crm(m) x = [ m.w × x.w ; m.v × x.w + m.w × x.v ]. Equivalent to
// `motionCross(m) * x` but ~3x fewer flops (used on the analytic-derivative hot
// path). Uses the shared `adjoint`/`skew`/`spatialInertia` from the dedup
// header.
inline Vector6 crossMotion(const Vector6& m, const Vector6& x)
{
  Vector6 out;
  const Eigen::Vector3d mAngular = m.head<3>();
  const Eigen::Vector3d mLinear = m.tail<3>();
  out.head<3>() = mAngular.cross(x.head<3>());
  out.tail<3>() = mLinear.cross(x.head<3>()) + mAngular.cross(x.tail<3>());
  return out;
}

//==============================================================================
// Force cross product crf(v) = -crm(v)^T applied to a force vector f, without
// forming the 6x6 matrix: crf(v) f = [ v.w × f.top + v.v × f.bot ; v.w × f.bot
// ]. Equivalent to `forceCross(v) * f`.
inline Vector6 crossForce(const Vector6& v, const Vector6& f)
{
  Vector6 out;
  const Eigen::Vector3d vAngular = v.head<3>();
  const Eigen::Vector3d vLinear = v.tail<3>();
  out.head<3>() = vAngular.cross(f.head<3>()) + vLinear.cross(f.tail<3>());
  out.tail<3>() = vAngular.cross(f.tail<3>());
  return out;
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
    case comps::JointType::Planar: {
      // In-plane translation (position[0], position[1]) along two orthogonal
      // directions plus rotation (position[2]) about the plane normal. This
      // matches the kinematics convention in world_kinematics_graph/frame.
      const Eigen::Vector3d normal = joint.axis.normalized();
      const Eigen::Vector3d inPlane1 = joint.axis2.normalized();
      const Eigen::Vector3d inPlane2 = normal.cross(inPlane1).normalized();
      transform.linear()
          = Eigen::AngleAxisd(joint.position[2], normal).toRotationMatrix();
      transform.translation()
          = inPlane1 * joint.position[0] + inPlane2 * joint.position[1];
      return transform;
    }
    case comps::JointType::Spherical:
      // Orientation stored as a rotation vector (exponential coordinates).
      transform.linear() = rotationExp(joint.position.head<3>());
      return transform;
    case comps::JointType::Floating:
      // 6-DOF pose: translation (position[0..2]) then orientation as a rotation
      // vector (position[3..5]), matching the kinematics convention.
      transform.linear() = rotationExp(joint.position.tail<3>());
      transform.translation() = joint.position.head<3>();
      return transform;
    default:
      DART_EXPERIMENTAL_THROW_T(
          InvalidOperationException,
          "Articulated-body forward dynamics is not yet implemented for this "
          "joint type; supported types are fixed, revolute, prismatic, screw, "
          "universal, planar, ball, and free");
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
    case comps::JointType::Planar: {
      // Two in-plane translations and a rotation about the normal, in the joint
      // output frame. The translation directions are carried by the rotation
      // (R(theta_rot, normal)^T * in-plane axis), making them configuration
      // dependent; the rotation column is the constant normal.
      const Eigen::Vector3d normal = joint.axis.normalized();
      const Eigen::Vector3d inPlane1 = joint.axis2.normalized();
      const Eigen::Vector3d inPlane2 = normal.cross(inPlane1).normalized();
      const Eigen::Matrix3d rotation
          = Eigen::AngleAxisd(joint.position[2], normal).toRotationMatrix();
      Subspace subspace(6, 3);
      subspace.col(0).head<3>().setZero();
      subspace.col(0).tail<3>() = rotation.transpose() * inPlane1;
      subspace.col(1).head<3>().setZero();
      subspace.col(1).tail<3>() = rotation.transpose() * inPlane2;
      subspace.col(2).head<3>() = normal;
      subspace.col(2).tail<3>().setZero();
      return subspace;
    }
    case comps::JointType::Spherical: {
      // Generalized velocity is the body angular velocity (matching the
      // rotation-vector position), so the subspace is constant: angular = I,
      // linear = 0.
      Subspace subspace = Subspace::Zero(6, 3);
      subspace.topRows<3>() = Eigen::Matrix3d::Identity();
      return subspace;
    }
    case comps::JointType::Floating: {
      // Generalized velocity is [linear; angular] body twist to match the
      // position layout [translation; rotation vector]; the subspace permutes
      // it into the [angular; linear] spatial convention. Constant subspace.
      Subspace subspace = Subspace::Zero(6, 6);
      subspace.bottomLeftCorner<3, 3>() = Eigen::Matrix3d::Identity();
      subspace.topRightCorner<3, 3>() = Eigen::Matrix3d::Identity();
      return subspace;
    }
    default:
      DART_EXPERIMENTAL_THROW_T(
          InvalidOperationException,
          "Articulated-body forward dynamics is not yet implemented for this "
          "joint type; supported types are fixed, revolute, prismatic, screw, "
          "universal, planar, ball, and free");
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
    case comps::JointType::Planar: {
      // The two in-plane translation columns rotate with position[2], so
      // d/dt(col_t) = -theta_rot_dot * R^T (normal x inplane_t). Each couples a
      // translation rate with the rotation rate (angular part zero).
      const Eigen::Vector3d normal = joint.axis.normalized();
      const Eigen::Vector3d inPlane1 = joint.axis2.normalized();
      const Eigen::Vector3d inPlane2 = normal.cross(inPlane1).normalized();
      const Eigen::Matrix3d rotationTranspose
          = Eigen::AngleAxisd(joint.position[2], normal)
                .toRotationMatrix()
                .transpose();
      Vector6 coeff0 = Vector6::Zero();
      coeff0.tail<3>() = -(rotationTranspose * normal.cross(inPlane1));
      Vector6 coeff1 = Vector6::Zero();
      coeff1.tail<3>() = -(rotationTranspose * normal.cross(inPlane2));
      return {
          JointBiasTerm{0, 2, offsetAdjoint * coeff0},
          JointBiasTerm{1, 2, offsetAdjoint * coeff1}};
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
// Internal handoff between the split semi-implicit multibody velocity, contact,
// and position stages. This intentionally stores only velocities, not mass
// matrices or dynamics trees, so each solve stage recomputes configuration
// dependent operators from the current registry state.
struct PendingMultibodyVelocity
{
  Eigen::VectorXd velocity;
};

//==============================================================================
// Build the per-link spatial dynamics for a multibody at its current
// configuration. Links are in construction order (parent-before-child).
DynamicsTree buildDynamicsTree(
    const entt::registry& registry, const comps::MultibodyStructure& structure)
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

    const Eigen::Isometry3d childInParent = linkComp.transformFromParentToJoint
                                            * jointMotionTransform(joint)
                                            * linkComp.transformFromParentJoint;
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
          static_cast<Eigen::Index>(link.dof)) = link.subspace;
    }
  }
  return jacobian;
}

//==============================================================================
// Index of a link entity within a multibody structure.
std::size_t linkIndexOf(
    const comps::MultibodyStructure& structure, entt::entity linkEntity)
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
                        + crossMotion(velocity[i], jointVelocity);
    }
    force[i] = link.inertia * acceleration[i]
               + crossForce(velocity[i], link.inertia * velocity[i]);
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
Eigen::VectorXd gatherMultibodyVelocity(
    const entt::registry& registry, const DynamicsTree& tree)
{
  Eigen::VectorXd qdot
      = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(tree.dofCount));
  for (std::size_t i = 0; i < tree.links.size(); ++i) {
    if (tree.links[i].dof == 0) {
      continue;
    }
    const auto& joint = registry.get<comps::Joint>(tree.jointOf[i]);
    qdot.segment(tree.links[i].dofOffset, tree.links[i].dof) = joint.velocity;
  }
  return qdot;
}

//==============================================================================
Eigen::VectorXd gatherMultibodyVelocity(
    entt::registry& registry, const comps::MultibodyStructure& structure)
{
  const DynamicsTree tree = buildDynamicsTree(registry, structure);
  if (tree.dofCount == 0) {
    return {};
  }
  return gatherMultibodyVelocity(registry, tree);
}

//==============================================================================
Eigen::VectorXd computeUnconstrainedMultibodyVelocity(
    entt::registry& registry,
    const comps::MultibodyStructure& structure,
    const Eigen::Vector3d& gravity,
    double timeStep)
{
  if (structure.links.empty()) {
    return {};
  }

  const DynamicsTree tree = buildDynamicsTree(registry, structure);
  if (tree.dofCount == 0) {
    return {};
  }

  // Current generalized velocity plus applied and passive (spring/damping)
  // generalized efforts.
  const Eigen::VectorXd qdot = gatherMultibodyVelocity(registry, tree);
  Eigen::VectorXd appliedForce
      = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(tree.dofCount));
  for (std::size_t i = 0; i < tree.links.size(); ++i) {
    if (tree.links[i].dof == 0) {
      continue;
    }
    const auto& joint = registry.get<comps::Joint>(tree.jointOf[i]);

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

  // Map each link's accumulated external wrench (body frame, [angular; linear])
  // to a generalized force via its body Jacobian: appliedForce += J_i^T w_i.
  // tree.links[i] is in structure order, so structure.links[i] is link i.
  {
    bool anyExternalForce = false;
    for (const auto linkEntity : structure.links) {
      if (!registry.get<comps::Link>(linkEntity).externalForce.isZero()) {
        anyExternalForce = true;
        break;
      }
    }
    if (anyExternalForce) {
      const std::vector<Eigen::MatrixXd> bodyJacobian = linkBodyJacobians(tree);
      for (std::size_t i = 0; i < tree.links.size(); ++i) {
        const Vector6 wrench
            = registry.get<comps::Link>(structure.links[i]).externalForce;
        if (!wrench.isZero()) {
          appliedForce.noalias() += bodyJacobian[i].transpose() * wrench;
        }
      }
    }
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

  return nextVelocity;
}

//==============================================================================
// Relative contact-point velocity along a world direction: the link's point
// velocity (via its Jacobian and the generalized velocity) minus a two-sided
// dynamic rigid obstacle's point velocity (zero for an immovable obstacle).
double linkContactRelativeVelocity(
    const entt::registry& registry,
    const MultibodyLinkContactRow& row,
    const Eigen::VectorXd& jacobian,
    const Eigen::Vector3d& direction,
    const Eigen::VectorXd& generalizedVelocity)
{
  double value = jacobian.dot(generalizedVelocity);
  if (row.otherBody != entt::null) {
    const auto& velocity = registry.get<comps::Velocity>(row.otherBody);
    value -= (velocity.linear + velocity.angular.cross(row.otherArm))
                 .dot(direction);
  }
  return value;
}

//==============================================================================
Eigen::MatrixXd pointLinearJacobian(
    const DynamicsTree& tree,
    const std::vector<Eigen::MatrixXd>& bodyJacobian,
    std::size_t linkIndex,
    const Eigen::Vector3d& point)
{
  const Eigen::Matrix3d rotation
      = tree.links[linkIndex].worldTransform.linear();
  const Eigen::Vector3d origin
      = tree.links[linkIndex].worldTransform.translation();
  const Eigen::MatrixXd angularJacobian
      = rotation * bodyJacobian[linkIndex].topRows(3);
  return rotation * bodyJacobian[linkIndex].bottomRows(3)
         - skew(point - origin) * angularJacobian;
}

//==============================================================================
void solveMultibodyLinkContacts(
    entt::registry& registry,
    const comps::MultibodyStructure& structure,
    Eigen::VectorXd& nextVelocity,
    double timeStep,
    const std::vector<LinkContact>& linkContacts)
{
  // Assemble the per-contact rows (Jacobians, denominators, bias, restitution,
  // two-sided obstacle coupling) once, then resolve them with an accumulated
  // velocity-level Gauss-Seidel sweep. The assembly is shared with the unified
  // constraint solve as the link-side counterpart of the rigid-body assembler.
  MultibodyLinkContactProblem problem = assembleMultibodyLinkContactProblem(
      registry, structure, nextVelocity, timeStep, linkContacts);
  auto& rows = problem.rows;
  const Eigen::MatrixXd& inverseMass = problem.inverseMass;
  if (rows.empty() || inverseMass.size() == 0) {
    return;
  }

  constexpr int contactIterations = 8;

  // Apply a contact impulse: drive the link via M^-1 J^T and apply the
  // equal-and-opposite impulse to the rigid obstacle (Newton's third law).
  const auto applyImpulse = [&](const MultibodyLinkContactRow& row,
                                const Eigen::VectorXd& jacobian,
                                const Eigen::Vector3d& direction,
                                double deltaImpulse) {
    nextVelocity += inverseMass * jacobian * deltaImpulse;
    if (row.otherBody != entt::null) {
      auto& velocity = registry.get<comps::Velocity>(row.otherBody);
      velocity.linear -= deltaImpulse * row.otherInvMass * direction;
      velocity.angular
          -= deltaImpulse * row.otherInvInertia * row.otherArm.cross(direction);
    }
  };

  for (int iteration = 0; iteration < contactIterations; ++iteration) {
    for (auto& row : rows) {
      if (!row.active) {
        continue;
      }

      // Normal impulse with accumulation (unilateral: lambda_n >= 0).
      const double normalVelocity = linkContactRelativeVelocity(
          registry, row, row.normalJacobian, row.normal, nextVelocity);
      const double deltaNormal
          = (-normalVelocity + std::max(row.bias, row.restitutionTarget))
            / row.normalDenominator;
      const double newNormal = std::max(0.0, row.normalImpulse + deltaNormal);
      applyImpulse(
          row, row.normalJacobian, row.normal, newNormal - row.normalImpulse);
      row.normalImpulse = newNormal;

      // Two-tangent Coulomb friction, each impulse bounded by mu * lambda_n.
      const double bound = row.friction * row.normalImpulse;
      if (row.tangentDenominator1 > 0.0) {
        const double vt = linkContactRelativeVelocity(
            registry, row, row.tangentJacobian1, row.tangent1, nextVelocity);
        const double newImpulse = std::clamp(
            row.tangentImpulse1 - vt / row.tangentDenominator1, -bound, bound);
        applyImpulse(
            row,
            row.tangentJacobian1,
            row.tangent1,
            newImpulse - row.tangentImpulse1);
        row.tangentImpulse1 = newImpulse;
      }
      if (row.tangentDenominator2 > 0.0) {
        const double vt = linkContactRelativeVelocity(
            registry, row, row.tangentJacobian2, row.tangent2, nextVelocity);
        const double newImpulse = std::clamp(
            row.tangentImpulse2 - vt / row.tangentDenominator2, -bound, bound);
        applyImpulse(
            row,
            row.tangentJacobian2,
            row.tangent2,
            newImpulse - row.tangentImpulse2);
        row.tangentImpulse2 = newImpulse;
      }
    }
  }
}

//==============================================================================
void enforceMultibodyVelocityLimits(
    entt::registry& registry,
    const comps::MultibodyStructure& structure,
    Eigen::VectorXd& nextVelocity)
{
  Eigen::Index expectedDof = 0;
  for (const auto linkEntity : structure.links) {
    const auto& link = registry.get<comps::Link>(linkEntity);
    if (link.parentJoint == entt::null) {
      continue;
    }

    const auto& joint = registry.get<comps::Joint>(link.parentJoint);
    expectedDof += static_cast<Eigen::Index>(joint.getDOF());
  }

  DART_EXPERIMENTAL_THROW_T_IF(
      nextVelocity.size() != expectedDof,
      InvalidArgumentException,
      "Staged multibody velocity dimension ({}) does not match the expected "
      "DOF count ({})",
      nextVelocity.size(),
      expectedDof);

  // Enforce velocity limits by clamping the generalized velocity.
  Eigen::Index velocityOffset = 0;
  for (const auto linkEntity : structure.links) {
    const auto& link = registry.get<comps::Link>(linkEntity);
    if (link.parentJoint == entt::null) {
      continue;
    }

    const auto& joint = registry.get<comps::Joint>(link.parentJoint);
    const auto dof = static_cast<Eigen::Index>(joint.getDOF());
    if (dof == 0) {
      continue;
    }
    if (joint.limits.velocityLower.size() != static_cast<Eigen::Index>(dof)
        || joint.limits.velocityUpper.size()
               != static_cast<Eigen::Index>(dof)) {
      velocityOffset += dof;
      continue;
    }
    for (Eigen::Index d = 0; d < dof; ++d) {
      const auto globalDof = velocityOffset + d;
      nextVelocity[globalDof] = std::clamp(
          nextVelocity[globalDof],
          joint.limits.velocityLower[d],
          joint.limits.velocityUpper[d]);
    }
    velocityOffset += dof;
  }
}

//==============================================================================
std::vector<LinkContact> collectMultibodyLinkContacts(
    entt::registry& registry,
    const comps::MultibodyStructure& structure,
    const std::vector<Contact>& contacts)
{
  const auto isRigidBody = [&](entt::entity entity) {
    return registry.all_of<comps::RigidBodyTag>(entity);
  };
  const auto isLink = [&](entt::entity entity) {
    return registry.all_of<comps::Link>(entity);
  };
  const auto isDynamic = [&](entt::entity entity) {
    return !hasPrescribedRigidBodyContactResponse(registry, entity);
  };
  const auto findMultibodyOwningLink = [&](entt::entity linkEntity) {
    auto view = registry.view<comps::MultibodyStructure>();
    for (auto multibody : view) {
      const auto& candidate = view.get<comps::MultibodyStructure>(multibody);
      if (std::find(candidate.links.begin(), candidate.links.end(), linkEntity)
          != candidate.links.end()) {
        return multibody;
      }
    }
    return static_cast<entt::entity>(entt::null);
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

  std::vector<LinkContact> linkContacts;
  for (const auto& contact : contacts) {
    const auto entityA = detail::toRegistryEntity(contact.bodyA.getEntity());
    const auto entityB = detail::toRegistryEntity(contact.bodyB.getEntity());
    const auto& links = structure.links;
    const bool aInBody
        = std::find(links.begin(), links.end(), entityA) != links.end();
    const bool bInBody
        = std::find(links.begin(), links.end(), entityB) != links.end();
    const double friction
        = std::sqrt(frictionOf(entityA) * frictionOf(entityB));
    const double restitution
        = std::max(restitutionOf(entityA), restitutionOf(entityB));

    // The contact normal points bodyA -> bodyB; orient it into the link. A
    // dynamic rigid-body obstacle is passed as otherBody for the two-sided
    // impulse; a static obstacle leaves it null (one-sided). Same-multibody
    // link obstacles use one relative point-Jacobian row. Cross-multibody
    // link pairs are routed once, from bodyA's multibody, and carry the other
    // multibody as a second articulated end for the unified solve.
    if (aInBody && bInBody) {
      linkContacts.push_back(
          {entityA,
           -contact.normal,
           contact.point,
           contact.depth,
           friction,
           restitution,
           entt::null,
           entityB});
    } else if (aInBody && !bInBody && isLink(entityB)) {
      const entt::entity otherMultibody = findMultibodyOwningLink(entityB);
      if (otherMultibody != entt::null) {
        linkContacts.push_back(
            {entityA,
             -contact.normal,
             contact.point,
             contact.depth,
             friction,
             restitution,
             entt::null,
             entityB,
             otherMultibody});
      }
    } else if (aInBody && !bInBody && isRigidBody(entityB)) {
      linkContacts.push_back(
          {entityA,
           -contact.normal,
           contact.point,
           contact.depth,
           friction,
           restitution,
           isDynamic(entityB) ? entityB : entt::null});
    } else if (bInBody && !aInBody && isRigidBody(entityA)) {
      linkContacts.push_back(
          {entityB,
           contact.normal,
           contact.point,
           contact.depth,
           friction,
           restitution,
           isDynamic(entityA) ? entityA : entt::null});
    }
  }

  return linkContacts;
}

//==============================================================================
void clearMultibodyExternalForces(
    entt::registry& registry, const comps::MultibodyStructure& structure)
{
  for (const auto linkEntity : structure.links) {
    registry.get<comps::Link>(linkEntity).externalForce.setZero();
  }
}

//==============================================================================
void simulateMultibody(
    entt::registry& registry,
    const comps::MultibodyStructure& structure,
    const Eigen::Vector3d& gravity,
    double timeStep,
    const std::vector<LinkContact>& linkContacts)
{
  Eigen::VectorXd nextVelocity = computeUnconstrainedMultibodyVelocity(
      registry, structure, gravity, timeStep);
  if (nextVelocity.size() == 0) {
    return;
  }

  solveMultibodyLinkContacts(
      registry, structure, nextVelocity, timeStep, linkContacts);
  enforceMultibodyVelocityLimits(registry, structure, nextVelocity);
  integrateMultibodyPositions(registry, structure, nextVelocity, timeStep);
}

//==============================================================================
struct CrossMultibodyAssemblyContext
{
  const comps::MultibodyStructure* structure = nullptr;
  DynamicsTree tree;
  std::vector<Eigen::MatrixXd> bodyJacobian;
};

//==============================================================================
void completeCrossMultibodyLinkRows(
    const entt::registry& registry,
    std::span<UnifiedMultibodyContact> multibodyContacts,
    std::span<const Eigen::VectorXd> multibodyVelocities)
{
  std::unordered_map<entt::entity, std::size_t> multibodyIndex;
  multibodyIndex.reserve(multibodyContacts.size());
  std::vector<CrossMultibodyAssemblyContext> contexts;
  contexts.reserve(multibodyContacts.size());

  for (std::size_t k = 0; k < multibodyContacts.size(); ++k) {
    const auto multibody = multibodyContacts[k].multibody;
    multibodyIndex[multibody] = k;

    const auto& structure = registry.get<comps::MultibodyStructure>(multibody);
    CrossMultibodyAssemblyContext context;
    context.structure = &structure;
    context.tree = buildDynamicsTree(registry, structure);
    context.bodyJacobian = linkBodyJacobians(context.tree);
    contexts.push_back(std::move(context));
  }

  constexpr double restitutionThreshold = 1e-2;
  for (std::size_t k = 0; k < multibodyContacts.size(); ++k) {
    auto& rows = multibodyContacts[k].problem.rows;
    const Eigen::VectorXd& primaryVelocity = multibodyVelocities[k];
    for (auto& row : rows) {
      if (row.otherMultibody == entt::null) {
        continue;
      }

      const auto otherMultibodyIt = multibodyIndex.find(row.otherMultibody);
      if (otherMultibodyIt == multibodyIndex.end()) {
        continue;
      }
      const std::size_t otherIndex = otherMultibodyIt->second;
      const auto& otherContext = contexts[otherIndex];
      const auto& otherLinks = otherContext.structure->links;
      const auto otherLinkIt
          = std::find(otherLinks.begin(), otherLinks.end(), row.otherLink);
      if (otherLinkIt == otherLinks.end()) {
        continue;
      }
      const auto otherLinkIndex
          = static_cast<std::size_t>(otherLinkIt - otherLinks.begin());
      const Eigen::MatrixXd otherPointJacobian = pointLinearJacobian(
          otherContext.tree,
          otherContext.bodyJacobian,
          otherLinkIndex,
          row.point);

      row.otherNormalJacobian = otherPointJacobian.transpose() * row.normal;
      row.otherTangentJacobian1 = otherPointJacobian.transpose() * row.tangent1;
      row.otherTangentJacobian2 = otherPointJacobian.transpose() * row.tangent2;

      const Eigen::MatrixXd& otherInverseMass
          = multibodyContacts[otherIndex].problem.inverseMass;
      row.normalDenominator += row.otherNormalJacobian.dot(
          otherInverseMass * row.otherNormalJacobian);
      row.tangentDenominator1 += row.otherTangentJacobian1.dot(
          otherInverseMass * row.otherTangentJacobian1);
      row.tangentDenominator2 += row.otherTangentJacobian2.dot(
          otherInverseMass * row.otherTangentJacobian2);

      const Eigen::VectorXd& otherVelocity = multibodyVelocities[otherIndex];
      const auto relativeVelocity = [&](const Eigen::VectorXd& primaryJacobian,
                                        const Eigen::VectorXd& otherJacobian) {
        return primaryJacobian.dot(primaryVelocity)
               - otherJacobian.dot(otherVelocity);
      };

      const double approachingVelocity
          = relativeVelocity(row.normalJacobian, row.otherNormalJacobian);
      row.restitutionTarget = (approachingVelocity < -restitutionThreshold)
                                  ? -row.restitution * approachingVelocity
                                  : 0.0;
      row.normalRhs
          = -approachingVelocity + std::max(row.bias, row.restitutionTarget);
      row.tangentRhs1
          = -relativeVelocity(row.tangentJacobian1, row.otherTangentJacobian1);
      row.tangentRhs2
          = -relativeVelocity(row.tangentJacobian2, row.otherTangentJacobian2);
      row.active = row.normalDenominator > 0.0;
    }
  }
}

//==============================================================================
// True when every movable joint of the tree has a single-DOF constant
// unit-twist motion subspace (Fixed contributes no DOF). For these joints the
// motion subspace S is configuration independent and the joint transform
// derivative is ∂X/∂q = -crm(S) X, which the analytic RNEA-derivative recursion
// below relies on. Other joint types (Universal/Planar configuration-dependent
// subspaces; Spherical/Floating multi-DOF manifold subspaces) are rejected so
// the caller falls back to finite differencing.
bool treeSupportsAnalyticDerivatives(
    const entt::registry& registry, const DynamicsTree& tree)
{
  for (std::size_t i = 0; i < tree.links.size(); ++i) {
    if (tree.links[i].dof == 0) {
      continue;
    }
    if (tree.links[i].dof != 1) {
      return false;
    }
    const auto& joint = registry.get<comps::Joint>(tree.jointOf[i]);
    if (joint.type != comps::JointType::Revolute
        && joint.type != comps::JointType::Prismatic
        && joint.type != comps::JointType::Screw) {
      return false;
    }
  }
  return true;
}

//==============================================================================
// Analytic ∂τ/∂q and ∂τ/∂q̇ of inverse dynamics τ = ID(q, q̇, qddot) via
// Recursive-Newton-Euler derivative recursions, for trees of constant
// unit-twist joints (see treeSupportsAnalyticDerivatives). [angular; linear]
// spatial convention; crm = motionCross, crf = forceCross; ∂X_i/∂q_i =
// -crm(S_i) X_i. O(dof²): a down/up sweep per source coordinate.
InverseDynamicsDerivatives rneaDerivatives(
    const std::vector<LinkDynamics>& links,
    const Vector6& baseAcceleration,
    std::size_t dofCount,
    const Eigen::VectorXd& qddot,
    const Eigen::VectorXd& qdot)
{
  const auto count = links.size();
  const auto n = static_cast<Eigen::Index>(dofCount);

  // Forward pass: per-link spatial velocity v, acceleration a, force f, and the
  // joint velocity contribution jv = S_i q̇_i.
  std::vector<Vector6> v(count, Vector6::Zero());
  std::vector<Vector6> a(count, Vector6::Zero());
  std::vector<Vector6> f(count, Vector6::Zero());
  std::vector<Vector6> jv(count, Vector6::Zero());
  for (std::size_t i = 0; i < count; ++i) {
    const auto& link = links[i];
    if (link.parentIndex < 0) {
      a[i] = link.parentToChild * baseAcceleration;
    } else {
      const auto p = static_cast<std::size_t>(link.parentIndex);
      if (link.dof > 0) {
        jv[i] = link.subspace * qdot.segment(link.dofOffset, link.dof);
        a[i] = link.subspace * qddot.segment(link.dofOffset, link.dof);
      }
      v[i] = link.parentToChild * v[p] + jv[i];
      a[i] += link.parentToChild * a[p] + crossMotion(v[i], jv[i]);
    }
    f[i] = link.inertia * a[i] + crossForce(v[i], link.inertia * v[i]);
  }

  // Total accumulated force F_i = f_i + Σ_children X_c^T F_c (needed by the
  // δ(X^T) term in the position-derivative backward sweep).
  std::vector<Vector6> totalForce = f;
  for (std::size_t r = 0; r < count; ++r) {
    const auto i = count - 1 - r;
    if (links[i].parentIndex >= 0) {
      totalForce[static_cast<std::size_t>(links[i].parentIndex)]
          += links[i].childToParentForce * totalForce[i];
    }
  }

  InverseDynamicsDerivatives out;
  out.dTau_dq = Eigen::MatrixXd::Zero(n, n);
  out.dTau_dqdot = Eigen::MatrixXd::Zero(n, n);
  out.valid = true;

  for (std::size_t m = 0; m < count; ++m) {
    if (links[m].dof == 0) {
      continue;
    }
    const auto j = static_cast<Eigen::Index>(links[m].dofOffset);
    const Vector6 subspaceM = links[m].subspace.col(0);

    // ---------- ∂/∂q_j ----------
    {
      std::vector<Vector6> dv(count, Vector6::Zero());
      std::vector<Vector6> da(count, Vector6::Zero());
      std::vector<Vector6> df(count, Vector6::Zero());
      for (std::size_t i = 0; i < count; ++i) {
        const auto& link = links[i];
        if (link.parentIndex >= 0) {
          const auto p = static_cast<std::size_t>(link.parentIndex);
          dv[i] = link.parentToChild * dv[p];
          da[i] = link.parentToChild * da[p];
          if (i == m) {
            // ∂X_m/∂q_j = -crm(S_m) X_m acting on the parent motion/accel.
            dv[i] += -crossMotion(subspaceM, link.parentToChild * v[p]);
            da[i] += -crossMotion(subspaceM, link.parentToChild * a[p]);
          }
          da[i] += crossMotion(dv[i], jv[i]);
        }
        df[i] = link.inertia * da[i] + crossForce(dv[i], link.inertia * v[i])
                + crossForce(v[i], link.inertia * dv[i]);
      }
      std::vector<Vector6> dForce = df;
      for (std::size_t r = 0; r < count; ++r) {
        const auto i = count - 1 - r;
        const auto& link = links[i];
        if (link.dof > 0) {
          out.dTau_dq(static_cast<Eigen::Index>(link.dofOffset), j)
              = link.subspace.col(0).dot(dForce[i]);
        }
        if (link.parentIndex >= 0) {
          const auto p = static_cast<std::size_t>(link.parentIndex);
          dForce[p] += link.childToParentForce * dForce[i];
          if (i == m) {
            // δ(X_m^T) F_m = X_m^T crf(S_m) F_m.
            dForce[p] += link.childToParentForce
                         * crossForce(subspaceM, totalForce[i]);
          }
        }
      }
    }

    // ---------- ∂/∂q̇_j ----------
    {
      std::vector<Vector6> dv(count, Vector6::Zero());
      std::vector<Vector6> da(count, Vector6::Zero());
      std::vector<Vector6> df(count, Vector6::Zero());
      for (std::size_t i = 0; i < count; ++i) {
        const auto& link = links[i];
        if (link.parentIndex >= 0) {
          const auto p = static_cast<std::size_t>(link.parentIndex);
          dv[i] = link.parentToChild * dv[p];
          da[i] = link.parentToChild * da[p];
          if (i == m) {
            dv[i] += subspaceM;
            da[i] += crossMotion(v[i], subspaceM);
          }
          da[i] += crossMotion(dv[i], jv[i]);
        }
        df[i] = link.inertia * da[i] + crossForce(dv[i], link.inertia * v[i])
                + crossForce(v[i], link.inertia * dv[i]);
      }
      std::vector<Vector6> dForce = df;
      for (std::size_t r = 0; r < count; ++r) {
        const auto i = count - 1 - r;
        const auto& link = links[i];
        if (link.dof > 0) {
          out.dTau_dqdot(static_cast<Eigen::Index>(link.dofOffset), j)
              = link.subspace.col(0).dot(dForce[i]);
        }
        if (link.parentIndex >= 0) {
          dForce[static_cast<std::size_t>(link.parentIndex)]
              += link.childToParentForce * dForce[i];
        }
      }
    }
  }

  return out;
}

} // namespace

//==============================================================================
MultibodyLinkContactProblem assembleMultibodyLinkContactProblem(
    const entt::registry& registry,
    const comps::MultibodyStructure& structure,
    const Eigen::VectorXd& nextVelocity,
    double timeStep,
    std::span<const LinkContact> linkContacts)
{
  MultibodyLinkContactProblem problem;
  if (structure.links.empty()) {
    return problem;
  }

  const DynamicsTree tree = buildDynamicsTree(registry, structure);

  DART_EXPERIMENTAL_THROW_T_IF(
      nextVelocity.size() != static_cast<Eigen::Index>(tree.dofCount),
      InvalidArgumentException,
      "Staged multibody velocity dimension ({}) does not match the expected "
      "DOF count ({})",
      nextVelocity.size(),
      tree.dofCount);

  if (tree.dofCount == 0) {
    problem.inverseMass.resize(0, 0);
  } else {
    const MassAndBias mb = computeMassAndBias(
        tree.links,
        tree.dofCount,
        Eigen::Vector3d::Zero(),
        nextVelocity,
        tree.armature);
    problem.inverseMass = mb.massMatrix.inverse();
  }
  const Eigen::MatrixXd& inverseMass = problem.inverseMass;
  const std::vector<Eigen::MatrixXd> bodyJacobian = linkBodyJacobians(tree);
  constexpr double penetrationSlop = 1e-4;
  constexpr double baumgarteFactor = 0.2;

  // Precompute the contact-space Jacobians (normal + two tangents) once. A
  // contact against another link in the same multibody uses the relative point
  // Jacobian. A cross-multibody link contact keeps only this multibody's point
  // Jacobian here; UnifiedConstraintStage completes the second articulated end
  // once every multibody block has been assembled. A contact against a dynamic
  // rigid body also stores that body's inverse mass and inertia and the contact
  // arm, so the impulse is applied to both bodies (two-sided); an immovable
  // obstacle leaves these zero (one-sided).
  problem.rows.assign(linkContacts.size(), MultibodyLinkContactRow{});

  for (std::size_t c = 0; c < linkContacts.size(); ++c) {
    const auto& contact = linkContacts[c];
    const auto linkIt = std::find(
        structure.links.begin(), structure.links.end(), contact.link);
    if (linkIt == structure.links.end()) {
      continue;
    }
    const auto index
        = static_cast<std::size_t>(linkIt - structure.links.begin());

    auto& row = problem.rows[c];
    row.point = contact.point;
    row.otherLink = contact.otherLink;
    row.otherMultibody = contact.otherMultibody;
    row.restitution = contact.restitution;

    Eigen::MatrixXd pointJacobian
        = pointLinearJacobian(tree, bodyJacobian, index, contact.point);
    if (contact.otherLink != entt::null) {
      const auto otherLinkIt = std::find(
          structure.links.begin(), structure.links.end(), contact.otherLink);
      if (otherLinkIt == linkIt) {
        continue;
      }
      if (otherLinkIt != structure.links.end()) {
        const auto otherIndex
            = static_cast<std::size_t>(otherLinkIt - structure.links.begin());
        pointJacobian -= pointLinearJacobian(
            tree, bodyJacobian, otherIndex, contact.point);
      } else if (contact.otherMultibody == entt::null) {
        continue;
      }
    }

    // Couple a dynamic rigid-body obstacle: cache its inverse mass, inverse
    // world inertia, and the contact arm.
    if (contact.otherLink == entt::null && contact.otherBody != entt::null
        && registry.all_of<comps::MassProperties, comps::Transform>(
            contact.otherBody)
        && !hasPrescribedRigidBodyContactResponse(
            registry, contact.otherBody)) {
      const auto& otherMass
          = registry.get<comps::MassProperties>(contact.otherBody);
      const auto& otherTransform
          = registry.get<comps::Transform>(contact.otherBody);
      if (otherMass.mass > 0.0 && std::isfinite(otherMass.mass)) {
        row.otherBody = contact.otherBody;
        row.otherInvMass = 1.0 / otherMass.mass;
        const Eigen::Matrix3d orientation
            = otherTransform.orientation.normalized().toRotationMatrix();
        const Eigen::Matrix3d worldInertia
            = orientation * otherMass.inertia * orientation.transpose();
        Eigen::LDLT<Eigen::Matrix3d> inertiaSolver(worldInertia);
        if (inertiaSolver.info() == Eigen::Success
            && inertiaSolver.isPositive()) {
          row.otherInvInertia
              = inertiaSolver.solve(Eigen::Matrix3d::Identity());
        }
        row.otherArm = contact.point - otherTransform.position;
      }
    }

    const Eigen::Vector3d normal = contact.normal.normalized();
    row.normal = normal;
    row.normalJacobian = pointJacobian.transpose() * normal;
    const Eigen::Vector3d normalArm = row.otherArm.cross(normal);
    row.normalDenominator
        = row.normalJacobian.dot(inverseMass * row.normalJacobian)
          + row.otherInvMass + normalArm.dot(row.otherInvInertia * normalArm);
    if (row.normalDenominator <= 0.0 && contact.otherMultibody == entt::null) {
      continue; // the contact cannot move either body (e.g. fixed base)
    }

    // Two tangent directions orthogonal to the contact normal.
    const Eigen::Vector3d reference = std::abs(normal.x()) < 0.9
                                          ? Eigen::Vector3d::UnitX()
                                          : Eigen::Vector3d::UnitY();
    const Eigen::Vector3d tangent1 = normal.cross(reference).normalized();
    const Eigen::Vector3d tangent2 = normal.cross(tangent1);
    row.tangent1 = tangent1;
    row.tangent2 = tangent2;
    row.tangentJacobian1 = pointJacobian.transpose() * tangent1;
    row.tangentJacobian2 = pointJacobian.transpose() * tangent2;
    const Eigen::Vector3d tangentArm1 = row.otherArm.cross(tangent1);
    const Eigen::Vector3d tangentArm2 = row.otherArm.cross(tangent2);
    row.tangentDenominator1
        = row.tangentJacobian1.dot(inverseMass * row.tangentJacobian1)
          + row.otherInvMass
          + tangentArm1.dot(row.otherInvInertia * tangentArm1);
    row.tangentDenominator2
        = row.tangentJacobian2.dot(inverseMass * row.tangentJacobian2)
          + row.otherInvMass
          + tangentArm2.dot(row.otherInvInertia * tangentArm2);
    row.bias = baumgarteFactor * std::max(0.0, contact.depth - penetrationSlop)
               / timeStep;
    row.friction = contact.friction;

    // Restitution target: rebound at -e * (approaching normal velocity),
    // ignoring slow approaches to avoid jitter at rest.
    constexpr double restitutionThreshold = 1e-2;
    const double approachingVelocity = linkContactRelativeVelocity(
        registry, row, row.normalJacobian, normal, nextVelocity);
    row.restitutionTarget = (approachingVelocity < -restitutionThreshold)
                                ? -contact.restitution * approachingVelocity
                                : 0.0;

    // Pre-solve boxed-LCP right-hand sides. The normal target mirrors the
    // Gauss-Seidel normal update (`-v_n + max(bias, restitution)`); the tangent
    // targets drive the tangential relative velocity to zero. Computed from the
    // staged pre-solve velocity so the unified constraint solve can consume
    // them without re-reading the registry.
    row.normalRhs
        = -approachingVelocity + std::max(row.bias, row.restitutionTarget);
    row.tangentRhs1 = -linkContactRelativeVelocity(
        registry, row, row.tangentJacobian1, tangent1, nextVelocity);
    row.tangentRhs2 = -linkContactRelativeVelocity(
        registry, row, row.tangentJacobian2, tangent2, nextVelocity);
    row.active = contact.otherMultibody == entt::null;
  }

  return problem;
}

//==============================================================================
MultibodyDynamicsTerms computeMultibodyDynamicsTerms(
    entt::registry& registry,
    const comps::MultibodyStructure& structure,
    const Eigen::Vector3d& gravity)
{
  MultibodyDynamicsTerms terms;
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
Eigen::VectorXd computeMultibodyInverseDynamics(
    entt::registry& registry,
    const comps::MultibodyStructure& structure,
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
InverseDynamicsDerivatives computeMultibodyInverseDynamicsDerivatives(
    entt::registry& registry,
    const comps::MultibodyStructure& structure,
    const Eigen::Vector3d& gravity,
    const Eigen::VectorXd& generalizedAcceleration)
{
  InverseDynamicsDerivatives result;
  if (structure.links.empty()) {
    return result;
  }

  const DynamicsTree tree = buildDynamicsTree(registry, structure);
  if (tree.dofCount == 0
      || generalizedAcceleration.size()
             != static_cast<Eigen::Index>(tree.dofCount)
      || !treeSupportsAnalyticDerivatives(registry, tree)) {
    return result; // valid == false: caller falls back to finite differencing
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

  Vector6 baseAcceleration = Vector6::Zero();
  baseAcceleration.tail<3>() = -gravity;

  return rneaDerivatives(
      tree.links,
      baseAcceleration,
      tree.dofCount,
      generalizedAcceleration,
      qdot);
}

//==============================================================================
Eigen::MatrixXd computeMultibodyLinkJacobian(
    entt::registry& registry,
    const comps::MultibodyStructure& structure,
    entt::entity linkEntity)
{
  const auto targetIndex = linkIndexOf(structure, linkEntity);
  const DynamicsTree tree = buildDynamicsTree(registry, structure);
  return linkBodyJacobians(tree)[targetIndex];
}

//==============================================================================
Eigen::MatrixXd computeMultibodyLinkWorldJacobian(
    entt::registry& registry,
    const comps::MultibodyStructure& structure,
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
std::string_view MultibodyForwardDynamicsStage::getName() const noexcept
{
  return "multibody_forward_dynamics";
}

//==============================================================================
ComputeStageMetadata MultibodyForwardDynamicsStage::getMetadata() const noexcept
{
  return {
      ComputeStageDomain::ArticulatedBody,
      toMask(ComputeStageAcceleration::TaskParallel)};
}

//==============================================================================
void MultibodyForwardDynamicsStage::execute(
    World& world, ComputeExecutor& /*executor*/)
{
  auto& registry = world.getRegistry();
  const Eigen::Vector3d gravity = world.getGravity();
  const double timeStep = world.getTimeStep();

  // Collision query once per step; route each contact that touches a link to
  // the multibody that owns it. Link-vs-rigid-body contacts are resolved here:
  // a static obstacle is one-sided, a dynamic rigid body receives the
  // equal-and-opposite impulse (two-sided). Same-multibody link-vs-link
  // contacts use one relative-Jacobian row; cross-multibody link-vs-link
  // contacts remain a later unified-solve slice.
  const std::vector<Contact> contacts = world.collide();

  auto view = registry.view<comps::MultibodyStructure>();
  for (auto entity : view) {
    const auto& structure = view.get<comps::MultibodyStructure>(entity);
    const std::vector<LinkContact> linkContacts
        = collectMultibodyLinkContacts(registry, structure, contacts);

    simulateMultibody(registry, structure, gravity, timeStep, linkContacts);
    if (registry.all_of<PendingMultibodyVelocity>(entity)) {
      registry.remove<PendingMultibodyVelocity>(entity);
    }

    // External forces are one-shot per step (like legacy
    // BodyNode::addExtForce): clear them after they have been consumed by this
    // step's dynamics.
    clearMultibodyExternalForces(registry, structure);
  }
}

//==============================================================================
std::string_view MultibodyVelocityStage::getName() const noexcept
{
  return "multibody_velocity";
}

//==============================================================================
ComputeStageMetadata MultibodyVelocityStage::getMetadata() const noexcept
{
  return {
      ComputeStageDomain::ArticulatedBody,
      toMask(ComputeStageAcceleration::TaskParallel)};
}

//==============================================================================
void MultibodyVelocityStage::execute(
    World& world, ComputeExecutor& /*executor*/)
{
  auto& registry = world.getRegistry();
  const Eigen::Vector3d gravity = world.getGravity();
  const double timeStep = world.getTimeStep();

  auto view = registry.view<comps::MultibodyStructure>();
  for (auto entity : view) {
    const auto& structure = view.get<comps::MultibodyStructure>(entity);
    Eigen::VectorXd nextVelocity = computeUnconstrainedMultibodyVelocity(
        registry, structure, gravity, timeStep);
    if (nextVelocity.size() == 0) {
      if (registry.all_of<PendingMultibodyVelocity>(entity)) {
        registry.remove<PendingMultibodyVelocity>(entity);
      }
    } else {
      registry.emplace_or_replace<PendingMultibodyVelocity>(
          entity, PendingMultibodyVelocity{std::move(nextVelocity)});
    }

    clearMultibodyExternalForces(registry, structure);
  }
}

//==============================================================================
std::string_view MultibodyContactStage::getName() const noexcept
{
  return "multibody_contact";
}

//==============================================================================
ComputeStageMetadata MultibodyContactStage::getMetadata() const noexcept
{
  return {
      ComputeStageDomain::ArticulatedBody,
      toMask(ComputeStageAcceleration::TaskParallel)};
}

//==============================================================================
void MultibodyContactStage::execute(World& world, ComputeExecutor& /*executor*/)
{
  auto& registry = world.getRegistry();
  const double timeStep = world.getTimeStep();
  const std::vector<Contact> contacts = world.collide();

  auto view = registry.view<comps::MultibodyStructure>();
  for (auto entity : view) {
    const auto& structure = view.get<comps::MultibodyStructure>(entity);
    const std::vector<LinkContact> linkContacts
        = collectMultibodyLinkContacts(registry, structure, contacts);
    if (linkContacts.empty()) {
      continue;
    }

    auto* pendingVelocity = registry.try_get<PendingMultibodyVelocity>(entity);
    if (pendingVelocity == nullptr) {
      Eigen::VectorXd currentVelocity
          = gatherMultibodyVelocity(registry, structure);
      if (currentVelocity.size() == 0) {
        continue;
      }
      pendingVelocity = &registry.emplace_or_replace<PendingMultibodyVelocity>(
          entity, PendingMultibodyVelocity{std::move(currentVelocity)});
    }

    solveMultibodyLinkContacts(
        registry, structure, pendingVelocity->velocity, timeStep, linkContacts);
  }
}

//==============================================================================
std::string_view MultibodyPositionStage::getName() const noexcept
{
  return "multibody_position";
}

//==============================================================================
ComputeStageMetadata MultibodyPositionStage::getMetadata() const noexcept
{
  return {
      ComputeStageDomain::ArticulatedBody,
      toMask(ComputeStageAcceleration::TaskParallel)};
}

//==============================================================================
void MultibodyPositionStage::execute(
    World& world, ComputeExecutor& /*executor*/)
{
  auto& registry = world.getRegistry();
  const double timeStep = world.getTimeStep();

  auto view = registry.view<comps::MultibodyStructure>();
  for (auto entity : view) {
    const auto& structure = view.get<comps::MultibodyStructure>(entity);
    Eigen::VectorXd nextVelocity;
    if (const auto* pendingVelocity
        = registry.try_get<PendingMultibodyVelocity>(entity)) {
      nextVelocity = pendingVelocity->velocity;
    } else {
      nextVelocity = gatherMultibodyVelocity(registry, structure);
    }

    if (nextVelocity.size() == 0) {
      continue;
    }

    enforceMultibodyVelocityLimits(registry, structure, nextVelocity);
    integrateMultibodyPositions(registry, structure, nextVelocity, timeStep);

    if (registry.all_of<PendingMultibodyVelocity>(entity)) {
      registry.remove<PendingMultibodyVelocity>(entity);
    }
  }
}

//==============================================================================
UnifiedConstraintStage::UnifiedConstraintStage(std::size_t frictionIterations)
  : m_frictionIterations(std::max<std::size_t>(1, frictionIterations))
{
}

//==============================================================================
std::string_view UnifiedConstraintStage::getName() const noexcept
{
  return "unified_constraint";
}

//==============================================================================
ComputeStageMetadata UnifiedConstraintStage::getMetadata() const noexcept
{
  return {
      ComputeStageDomain::Constraint,
      ComputeStageAcceleration::TaskParallel
          | ComputeStageAcceleration::DataLocality};
}

//==============================================================================
std::size_t UnifiedConstraintStage::getFrictionIterations() const noexcept
{
  return m_frictionIterations;
}

//==============================================================================
void UnifiedConstraintStage::execute(
    World& world, ComputeExecutor& /*executor*/)
{
  auto& registry = world.getRegistry();
  const double timeStep = world.getTimeStep();
  const std::vector<Contact> contacts = world.collide();
  if (contacts.empty()) {
    return;
  }

  // Rigid-rigid block. The rigid assembler filters to rigid-rigid pairs, so it
  // is given the full contact set.
  RigidBodyContactProblem rigidProblem
      = assembleRigidBodyContactProblem(registry, contacts);

  // Per-multibody link blocks. Recompute each multibody's dynamics tree and
  // inverse mass in-stage (never cached across stages). Mirror
  // MultibodyContactStage's PendingMultibodyVelocity create-from-gather so a
  // multibody whose velocity has not been staged still resolves. Cross-
  // multibody link rows are owned by bodyA's multibody, but the other
  // multibody still needs a block and staged velocity so the solved impulse can
  // update both articulated ends.
  std::vector<entt::entity> multibodyEntities;
  std::unordered_map<entt::entity, std::vector<LinkContact>>
      contactsByMultibody;
  std::unordered_map<entt::entity, bool> requiredMultibody;
  auto view = registry.view<comps::MultibodyStructure>();
  for (auto entity : view) {
    multibodyEntities.push_back(entity);
    const auto& structure = view.get<comps::MultibodyStructure>(entity);
    std::vector<LinkContact> linkContacts
        = collectMultibodyLinkContacts(registry, structure, contacts);
    for (const auto& contact : linkContacts) {
      if (contact.otherMultibody != entt::null) {
        requiredMultibody[contact.otherMultibody] = true;
      }
    }
    if (!linkContacts.empty()) {
      requiredMultibody[entity] = true;
    }
    contactsByMultibody.emplace(entity, std::move(linkContacts));
  }

  std::vector<UnifiedMultibodyContact> multibodyContacts;
  std::vector<Eigen::VectorXd> multibodyVelocities;
  for (auto entity : multibodyEntities) {
    auto contactsIt = contactsByMultibody.find(entity);
    const std::vector<LinkContact>& linkContacts = contactsIt->second;
    if (linkContacts.empty()
        && requiredMultibody.find(entity) == requiredMultibody.end()) {
      continue;
    }

    const auto& structure = registry.get<comps::MultibodyStructure>(entity);
    Eigen::VectorXd stagedVelocity;
    auto* pendingVelocity = registry.try_get<PendingMultibodyVelocity>(entity);
    if (pendingVelocity != nullptr) {
      stagedVelocity = pendingVelocity->velocity;
    } else {
      stagedVelocity = gatherMultibodyVelocity(registry, structure);
    }

    MultibodyLinkContactProblem linkProblem
        = assembleMultibodyLinkContactProblem(
            registry, structure, stagedVelocity, timeStep, linkContacts);
    multibodyVelocities.push_back(std::move(stagedVelocity));
    multibodyContacts.push_back({entity, std::move(linkProblem)});
  }
  completeCrossMultibodyLinkRows(
      registry, multibodyContacts, multibodyVelocities);

  bool hasActiveLinkRows = false;
  for (const auto& contact : multibodyContacts) {
    for (const auto& row : contact.problem.rows) {
      hasActiveLinkRows = hasActiveLinkRows || row.active;
    }
  }
  if (rigidProblem.constraints.empty() && !hasActiveLinkRows) {
    return;
  }

  const UnifiedConstraintProblem problem
      = assembleUnifiedConstraintProblem(rigidProblem, multibodyContacts);

  resolveUnifiedConstraints(
      registry,
      problem,
      std::span<Eigen::VectorXd>(multibodyVelocities),
      m_frictionIterations);

  // Write each multibody's resolved generalized velocity back to its staging
  // component so the position stage integrates the post-contact velocity. The
  // blocks are in the same order the staged velocities were collected.
  for (std::size_t k = 0; k < problem.multibodyBlocks.size(); ++k) {
    if (multibodyVelocities[k].size() == 0) {
      if (registry.all_of<PendingMultibodyVelocity>(
              problem.multibodyBlocks[k].multibody)) {
        registry.remove<PendingMultibodyVelocity>(
            problem.multibodyBlocks[k].multibody);
      }
      continue;
    }
    registry.emplace_or_replace<PendingMultibodyVelocity>(
        problem.multibodyBlocks[k].multibody,
        PendingMultibodyVelocity{std::move(multibodyVelocities[k])});
  }

  // Rigid positional projection: remove residual penetration beyond a small
  // allowance without injecting velocity (verbatim from RigidBodyContactStage).
  constexpr double allowance = 1e-4;
  constexpr double correctionFactor = 0.2;
  for (const auto& constraint : problem.rigidConstraints) {
    const double penetration = constraint.depth - allowance;
    if (penetration <= 0.0) {
      continue;
    }
    const double totalInverseMass = constraint.invMassA + constraint.invMassB;
    if (totalInverseMass <= 0.0) {
      continue;
    }
    const Eigen::Vector3d correction
        = (correctionFactor * penetration / totalInverseMass)
          * constraint.normal;
    registry.get<comps::Transform>(constraint.bodyA).position
        -= constraint.invMassA * correction;
    registry.get<comps::Transform>(constraint.bodyB).position
        += constraint.invMassB * correction;
  }
}

} // namespace dart::simulation::experimental::compute
