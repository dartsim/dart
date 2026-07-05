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

#include "dart/simulation/compute/multibody_dynamics.hpp"

#include "dart/simulation/body/contact.hpp"
#include "dart/simulation/common/exceptions.hpp"
#include "dart/simulation/comps/collision_geometry.hpp"
#include "dart/simulation/comps/contact_material.hpp"
#include "dart/simulation/comps/dynamics.hpp"
#include "dart/simulation/comps/frame_types.hpp"
#include "dart/simulation/comps/joint.hpp"
#include "dart/simulation/comps/link.hpp"
#include "dart/simulation/comps/multibody.hpp"
#include "dart/simulation/comps/rigid_body.hpp"
#include "dart/simulation/compute/compute_executor.hpp"
#include "dart/simulation/compute/multibody_constraint.hpp"
#include "dart/simulation/compute/unified_constraint.hpp"
#include "dart/simulation/detail/articulated_inverse_mass.hpp"
#include "dart/simulation/detail/entity_conversion.hpp"
#include "dart/simulation/detail/multibody_spatial_algebra.hpp"
#include "dart/simulation/detail/world_registry_access.hpp"
#include "dart/simulation/world.hpp"

#include <dart/common/memory_manager.hpp>
#include <dart/common/stl_allocator.hpp>

#include <Eigen/Cholesky>
#include <Eigen/Geometry>
#include <entt/entt.hpp>

#include <algorithm>
#include <new>
#include <span>
#include <unordered_map>
#include <utility>
#include <vector>

#include <cmath>

namespace dart::simulation::compute {

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

using LinkOwnerMapEntry = std::pair<const entt::entity, entt::entity>;
using LinkOwnerMapAllocator = common::StlAllocator<LinkOwnerMapEntry>;
using LinkOwnerMap = std::unordered_map<
    entt::entity,
    entt::entity,
    std::hash<entt::entity>,
    std::equal_to<entt::entity>,
    LinkOwnerMapAllocator>;

template <typename T, typename... Args>
[[nodiscard]] T* constructStageOwnedScratch(
    common::MemoryManager* memoryManager, Args&&... args)
{
  if (memoryManager != nullptr) {
    auto* scratch
        = memoryManager->constructUsingFree<T>(std::forward<Args>(args)...);
    if (scratch == nullptr) {
      throw std::bad_alloc();
    }
    return scratch;
  }

  return new T(std::forward<Args>(args)...);
}

template <typename T>
void destroyStageOwnedScratch(
    common::MemoryManager* memoryManager, T* scratch) noexcept
{
  if (scratch == nullptr) {
    return;
  }
  if (memoryManager != nullptr) {
    memoryManager->destroyUsingFree(scratch);
  } else {
    delete scratch;
  }
}

//==============================================================================
bool hasPrescribedRigidBodyContactResponse(
    const detail::WorldRegistry& registry, entt::entity entity)
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
Eigen::Isometry3d jointMotionTransform(
    const comps::JointModel& jointModel, const comps::JointState& jointState)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  switch (jointModel.type) {
    case comps::JointType::Fixed:
      return transform;
    case comps::JointType::Revolute:
      transform.linear()
          = Eigen::AngleAxisd(jointState.position[0], jointModel.axis)
                .toRotationMatrix();
      return transform;
    case comps::JointType::Prismatic:
      transform.translation() = jointModel.axis * jointState.position[0];
      return transform;
    case comps::JointType::Screw:
      // Coupled rotation + translation along the axis: theta rotates and
      // advances by pitch * theta.
      transform.linear()
          = Eigen::AngleAxisd(jointState.position[0], jointModel.axis)
                .toRotationMatrix();
      transform.translation()
          = jointModel.axis * (jointModel.pitch * jointState.position[0]);
      return transform;
    case comps::JointType::Universal:
      // Two sequential rotations: theta1 about axis, then theta2 about axis2
      // (axis2 expressed in the intermediate frame).
      transform.linear()
          = (Eigen::AngleAxisd(jointState.position[0], jointModel.axis)
             * Eigen::AngleAxisd(jointState.position[1], jointModel.axis2))
                .toRotationMatrix();
      return transform;
    case comps::JointType::Planar: {
      // In-plane translation (position[0], position[1]) along two orthogonal
      // directions plus rotation (position[2]) about the plane normal. This
      // matches the kinematics convention in world_kinematics_graph/frame.
      const Eigen::Vector3d normal = jointModel.axis.normalized();
      const Eigen::Vector3d inPlane1 = jointModel.axis2.normalized();
      const Eigen::Vector3d inPlane2 = normal.cross(inPlane1).normalized();
      transform.linear() = Eigen::AngleAxisd(jointState.position[2], normal)
                               .toRotationMatrix();
      transform.translation() = inPlane1 * jointState.position[0]
                                + inPlane2 * jointState.position[1];
      return transform;
    }
    case comps::JointType::Spherical:
      // Orientation stored as a rotation vector (exponential coordinates).
      transform.linear() = rotationExp(jointState.position.head<3>());
      return transform;
    case comps::JointType::Floating:
      // 6-DOF pose: translation (position[0..2]) then orientation as a rotation
      // vector (position[3..5]), matching the kinematics convention.
      transform.linear() = rotationExp(jointState.position.tail<3>());
      transform.translation() = jointState.position.head<3>();
      return transform;
    default:
      DART_SIMULATION_THROW_T(
          InvalidOperationException,
          "Articulated-body forward dynamics is not yet implemented for this "
          "joint type; supported types are fixed, revolute, prismatic, screw, "
          "universal, planar, ball, and free");
  }
}

//==============================================================================
// Joint motion subspace expressed in the joint frame (before the post-joint
// link offset), in the [angular; linear] convention.
void setJointSubspaceInJointFrame(
    const comps::JointModel& jointModel,
    const comps::JointState& jointState,
    Subspace& subspace)
{
  switch (jointModel.type) {
    case comps::JointType::Fixed:
      subspace.resize(6, 0);
      return;
    case comps::JointType::Revolute: {
      subspace.resize(6, 1);
      subspace.col(0).head<3>() = jointModel.axis;
      subspace.col(0).tail<3>().setZero();
      return;
    }
    case comps::JointType::Prismatic: {
      subspace.resize(6, 1);
      subspace.col(0).head<3>().setZero();
      subspace.col(0).tail<3>() = jointModel.axis;
      return;
    }
    case comps::JointType::Screw: {
      // Twist of a screw: angular = axis, linear = pitch * axis.
      subspace.resize(6, 1);
      subspace.col(0).head<3>() = jointModel.axis;
      subspace.col(0).tail<3>() = jointModel.axis * jointModel.pitch;
      return;
    }
    case comps::JointType::Universal: {
      // Both columns expressed in the joint output frame (after both
      // rotations). Column 0 (theta1 about axis) is carried by the second
      // rotation, so it becomes R(theta2, axis2)^T * axis; column 1 (theta2
      // about axis2) is axis2 itself. This makes column 0 configuration
      // dependent, which the velocity-product term cJ accounts for.
      subspace.resize(6, 2);
      const Eigen::Matrix3d secondRotation
          = Eigen::AngleAxisd(jointState.position[1], jointModel.axis2)
                .toRotationMatrix();
      subspace.col(0).head<3>() = secondRotation.transpose() * jointModel.axis;
      subspace.col(0).tail<3>().setZero();
      subspace.col(1).head<3>() = jointModel.axis2;
      subspace.col(1).tail<3>().setZero();
      return;
    }
    case comps::JointType::Planar: {
      // Two in-plane translations and a rotation about the normal, in the joint
      // output frame. The translation directions are carried by the rotation
      // (R(theta_rot, normal)^T * in-plane axis), making them configuration
      // dependent; the rotation column is the constant normal.
      const Eigen::Vector3d normal = jointModel.axis.normalized();
      const Eigen::Vector3d inPlane1 = jointModel.axis2.normalized();
      const Eigen::Vector3d inPlane2 = normal.cross(inPlane1).normalized();
      const Eigen::Matrix3d rotation
          = Eigen::AngleAxisd(jointState.position[2], normal)
                .toRotationMatrix();
      subspace.resize(6, 3);
      subspace.col(0).head<3>().setZero();
      subspace.col(0).tail<3>() = rotation.transpose() * inPlane1;
      subspace.col(1).head<3>().setZero();
      subspace.col(1).tail<3>() = rotation.transpose() * inPlane2;
      subspace.col(2).head<3>() = normal;
      subspace.col(2).tail<3>().setZero();
      return;
    }
    case comps::JointType::Spherical: {
      // Generalized velocity is the body angular velocity (matching the
      // rotation-vector position), so the subspace is constant: angular = I,
      // linear = 0.
      subspace.resize(6, 3);
      subspace.setZero();
      subspace.topRows<3>() = Eigen::Matrix3d::Identity();
      return;
    }
    case comps::JointType::Floating: {
      // Generalized velocity is [linear; angular] body twist to match the
      // position layout [translation; rotation vector]; the subspace permutes
      // it into the [angular; linear] spatial convention. Constant subspace.
      subspace.resize(6, 6);
      subspace.setZero();
      subspace.bottomLeftCorner<3, 3>() = Eigen::Matrix3d::Identity();
      subspace.topRightCorner<3, 3>() = Eigen::Matrix3d::Identity();
      return;
    }
    default:
      DART_SIMULATION_THROW_T(
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
template <typename JointBiasTermVector>
void setJointBiasTerms(
    const comps::JointModel& jointModel,
    const comps::JointState& jointState,
    const Matrix6& offsetAdjoint,
    JointBiasTermVector& terms)
{
  terms.clear();
  switch (jointModel.type) {
    case comps::JointType::Universal: {
      // d/dt(col0) = (s1 x axis2) * theta2dot with s1 = R(theta2, axis2)^T
      // axis, so cJ = (s1 x axis2) * theta1dot * theta2dot (angular; linear
      // zero).
      const Eigen::Matrix3d secondRotation
          = Eigen::AngleAxisd(jointState.position[1], jointModel.axis2)
                .toRotationMatrix();
      const Eigen::Vector3d s1 = secondRotation.transpose() * jointModel.axis;
      Vector6 coeff = Vector6::Zero();
      coeff.head<3>() = s1.cross(jointModel.axis2);
      terms.push_back(JointBiasTerm{0, 1, offsetAdjoint * coeff});
      return;
    }
    case comps::JointType::Planar: {
      // The two in-plane translation columns rotate with position[2], so
      // d/dt(col_t) = -theta_rot_dot * R^T (normal x inplane_t). Each couples a
      // translation rate with the rotation rate (angular part zero).
      const Eigen::Vector3d normal = jointModel.axis.normalized();
      const Eigen::Vector3d inPlane1 = jointModel.axis2.normalized();
      const Eigen::Vector3d inPlane2 = normal.cross(inPlane1).normalized();
      const Eigen::Matrix3d rotationTranspose
          = Eigen::AngleAxisd(jointState.position[2], normal)
                .toRotationMatrix()
                .transpose();
      Vector6 coeff0 = Vector6::Zero();
      coeff0.tail<3>() = -(rotationTranspose * normal.cross(inPlane1));
      Vector6 coeff1 = Vector6::Zero();
      coeff1.tail<3>() = -(rotationTranspose * normal.cross(inPlane2));
      terms.push_back(JointBiasTerm{0, 2, offsetAdjoint * coeff0});
      terms.push_back(JointBiasTerm{1, 2, offsetAdjoint * coeff1});
      return;
    }
    default:
      return;
  }
}

//==============================================================================
// Per-link data precomputed at the current configuration. Index 0..count-1 in
// multibody construction order, which is guaranteed parent-before-child.
struct LinkDynamics
{
  using BiasTermAllocator = common::StlAllocator<JointBiasTerm>;

  LinkDynamics() = default;

  explicit LinkDynamics(common::MemoryAllocator& allocator)
    : biasTerms(BiasTermAllocator{allocator})
  {
  }

  int parentIndex = -1; // -1 for the fixed base (root)
  entt::entity jointEntity = entt::null;
  std::size_t dof = 0;
  std::size_t dofOffset = 0;
  Matrix6 parentToChild = Matrix6::Identity();      // X = Ad(T^{-1})
  Matrix6 childToParentForce = Matrix6::Identity(); // X^T
  Subspace subspace{6, 0};                          // S in child frame
  std::vector<JointBiasTerm, BiasTermAllocator>
      biasTerms; // cJ = Sdot qdot (child frame)
  Matrix6 inertia = Matrix6::Zero();
  Eigen::Isometry3d worldTransform = Eigen::Isometry3d::Identity();
};

//==============================================================================
// A multibody's per-link dynamics precomputed at the current configuration,
// plus the joint owning each link and the total movable DOF count.
struct DynamicsTree
{
  using LinkAllocator = common::StlAllocator<LinkDynamics>;
  using JointAllocator = common::StlAllocator<entt::entity>;

  DynamicsTree() = default;

  explicit DynamicsTree(common::MemoryAllocator& allocator)
    : memoryAllocator(&allocator),
      links(LinkAllocator{allocator}),
      jointOf(JointAllocator{allocator})
  {
  }

  void resizeLinks(std::size_t count)
  {
    while (links.size() < count) {
      if (memoryAllocator != nullptr) {
        links.emplace_back(*memoryAllocator);
      } else {
        links.emplace_back();
      }
    }
    links.resize(count);
  }

  common::MemoryAllocator* memoryAllocator = nullptr;
  std::vector<LinkDynamics, LinkAllocator> links;
  std::vector<entt::entity, JointAllocator> jointOf;
  std::size_t dofCount = 0;
  Eigen::VectorXd armature; // Rotor inertia per generalized coordinate.
};

LinkOwnerMap makeLinkOwnerMap(common::MemoryAllocator& allocator)
{
  return LinkOwnerMap{
      0,
      std::hash<entt::entity>{},
      std::equal_to<entt::entity>{},
      LinkOwnerMapAllocator{allocator}};
}

template <typename MultibodyView>
void rebuildLinkOwnerMapInto(
    const MultibodyView& multibodyView, LinkOwnerMap& linkOwner)
{
  linkOwner.clear();
  for (const auto multibody : multibodyView) {
    const auto& structure
        = multibodyView.template get<comps::MultibodyStructure>(multibody);
    for (const auto link : structure.links) {
      linkOwner.insert_or_assign(link, multibody);
    }
  }
}

template <typename MultibodyView>
bool linkOwnerMapMatches(
    const MultibodyView& multibodyView, const LinkOwnerMap& linkOwner)
{
  std::size_t linkCount = 0;
  for (const auto multibody : multibodyView) {
    const auto& structure
        = multibodyView.template get<comps::MultibodyStructure>(multibody);
    for (const auto link : structure.links) {
      ++linkCount;
      const auto it = linkOwner.find(link);
      if (it == linkOwner.end() || it->second != multibody) {
        return false;
      }
    }
  }
  return linkOwner.size() == linkCount;
}

template <typename MultibodyView>
void ensureLinkOwnerMapInto(
    const MultibodyView& multibodyView, LinkOwnerMap& linkOwner)
{
  if (linkOwnerMapMatches(multibodyView, linkOwner)) {
    return;
  }

  std::size_t linkCount = 0;
  for (const auto multibody : multibodyView) {
    const auto& structure
        = multibodyView.template get<comps::MultibodyStructure>(multibody);
    linkCount += structure.links.size();
  }
  linkOwner.reserve(linkCount);
  rebuildLinkOwnerMapInto(multibodyView, linkOwner);
}

entt::entity findMultibodyOwningLink(
    const LinkOwnerMap& linkOwner, entt::entity linkEntity)
{
  const auto it = linkOwner.find(linkEntity);
  if (it == linkOwner.end()) {
    return static_cast<entt::entity>(entt::null);
  }
  return it->second;
}

std::span<const LinkDynamics> linkSpan(const DynamicsTree& tree) noexcept
{
  return {tree.links.data(), tree.links.size()};
}

//==============================================================================
// Internal handoff between the split semi-implicit multibody velocity, contact,
// and position stages. This intentionally stores only velocities, not mass
// matrices or dynamics trees, so each solve stage recomputes configuration
// dependent operators from the current registry state.
struct PendingMultibodyVelocity
{
  Eigen::VectorXd velocity;
  bool active = false;
};

//==============================================================================
// Build the per-link spatial dynamics for a multibody at its current
// configuration. Links are in construction order (parent-before-child).
template <typename LinkIndexVector>
void buildDynamicsTreeInto(
    const detail::WorldRegistry& registry,
    const comps::MultibodyStructure& structure,
    DynamicsTree& tree,
    LinkIndexVector& indexOf,
    Subspace& jointFrameSubspace)
{
  const auto& linkEntities = structure.links;

  tree.resizeLinks(linkEntities.size());
  tree.jointOf.resize(linkEntities.size());
  std::fill(
      tree.jointOf.begin(),
      tree.jointOf.end(),
      static_cast<entt::entity>(entt::null));
  tree.dofCount = 0;

  indexOf.clear();
  indexOf.reserve(linkEntities.size());
  for (std::size_t i = 0; i < linkEntities.size(); ++i) {
    indexOf.emplace_back(linkEntities[i], i);
  }

  for (std::size_t i = 0; i < linkEntities.size(); ++i) {
    const auto linkEntity = linkEntities[i];
    const auto& linkComp = registry.get<comps::LinkModel>(linkEntity);
    auto& dynamics = tree.links[i];
    dynamics.parentIndex = -1;
    dynamics.jointEntity = entt::null;
    dynamics.dof = 0;
    dynamics.dofOffset = 0;
    dynamics.parentToChild.setIdentity();
    dynamics.childToParentForce.setIdentity();
    dynamics.biasTerms.clear();
    dynamics.inertia = spatialInertia(linkComp.mass);

    if (linkComp.parentJoint == entt::null) {
      // Fixed base: map the world base acceleration into the root frame.
      const auto& cache = registry.get<comps::FrameCache>(linkEntity);
      dynamics.parentToChild = adjoint(cache.worldTransform.inverse());
      dynamics.childToParentForce = dynamics.parentToChild.transpose();
      dynamics.worldTransform = cache.worldTransform;
      dynamics.parentIndex = -1;
      dynamics.subspace.resize(6, 0);
      continue;
    }

    const auto& jointModel
        = registry.get<comps::JointModel>(linkComp.parentJoint);
    const auto& jointState
        = registry.get<comps::JointState>(linkComp.parentJoint);
    const auto parentIt
        = std::find_if(indexOf.begin(), indexOf.end(), [&](const auto& entry) {
            return entry.first == jointModel.parentLink;
          });
    DART_SIMULATION_THROW_T_IF(
        parentIt == indexOf.end(),
        InvalidOperationException,
        "Multibody link parent is not part of the same multibody");
    dynamics.parentIndex = static_cast<int>(parentIt->second);
    dynamics.jointEntity = linkComp.parentJoint;
    tree.jointOf[i] = linkComp.parentJoint;

    const Eigen::Isometry3d childInParent
        = linkComp.transformFromParentToJoint
          * jointMotionTransform(jointModel, jointState)
          * linkComp.transformFromParentJoint;
    dynamics.parentToChild = adjoint(childInParent.inverse());
    dynamics.childToParentForce = dynamics.parentToChild.transpose();
    dynamics.worldTransform
        = tree.links[parentIt->second].worldTransform * childInParent;

    setJointSubspaceInJointFrame(jointModel, jointState, jointFrameSubspace);
    const Matrix6 offsetAdjoint
        = adjoint(linkComp.transformFromParentJoint.inverse());
    dynamics.subspace.resize(6, jointFrameSubspace.cols());
    dynamics.subspace.noalias() = offsetAdjoint * jointFrameSubspace;
    setJointBiasTerms(
        jointModel, jointState, offsetAdjoint, dynamics.biasTerms);
    dynamics.dof = static_cast<std::size_t>(jointFrameSubspace.cols());
    dynamics.dofOffset = tree.dofCount;
    tree.dofCount += dynamics.dof;
  }

  // Gather per-coordinate rotor inertia (armature) in generalized-coordinate
  // order; defaults to zero for any joint without an armature set.
  tree.armature.resize(static_cast<Eigen::Index>(tree.dofCount));
  tree.armature.setZero();
  for (std::size_t i = 0; i < tree.links.size(); ++i) {
    const auto dof = tree.links[i].dof;
    if (dof == 0) {
      continue;
    }
    const auto& jointModel = registry.get<comps::JointModel>(tree.jointOf[i]);
    if (jointModel.armature.size() == static_cast<Eigen::Index>(dof)) {
      tree.armature.segment(tree.links[i].dofOffset, dof) = jointModel.armature;
    }
  }
}

//==============================================================================
// Body-frame spatial Jacobian of every link via the recursion
// J_i = X_i J_parent, with the link's own joint columns set to its motion
// subspace S_i (already expressed in the link frame). Each Jacobian is 6 x
// dofCount, [angular; linear] in the link's own frame.
template <typename JacobianVector>
void linkBodyJacobiansInto(const DynamicsTree& tree, JacobianVector& jacobian)
{
  const auto dofCount = static_cast<Eigen::Index>(tree.dofCount);
  jacobian.resize(tree.links.size());
  for (std::size_t i = 0; i < tree.links.size(); ++i) {
    auto& linkJacobian = jacobian[i];
    linkJacobian.resize(6, dofCount);
    linkJacobian.setZero();

    const auto& link = tree.links[i];
    if (link.parentIndex >= 0) {
      linkJacobian.noalias()
          = link.parentToChild
            * jacobian[static_cast<std::size_t>(link.parentIndex)];
    }
    if (link.dof > 0) {
      linkJacobian.middleCols(
          static_cast<Eigen::Index>(link.dofOffset),
          static_cast<Eigen::Index>(link.dof)) = link.subspace;
    }
  }
}

//==============================================================================
// Index of a link entity within a multibody structure.
std::size_t linkIndexOf(
    const comps::MultibodyStructure& structure, entt::entity linkEntity)
{
  const auto& linkEntities = structure.links;
  const auto it
      = std::find(linkEntities.begin(), linkEntities.end(), linkEntity);
  DART_SIMULATION_THROW_T_IF(
      it == linkEntities.end(),
      InvalidArgumentException,
      "Link does not belong to this multibody");
  return static_cast<std::size_t>(it - linkEntities.begin());
}

//==============================================================================
bool hasCollisionShapes(
    const detail::WorldRegistry& registry, entt::entity entity)
{
  const auto* geometry = registry.try_get<comps::CollisionGeometry>(entity);
  return geometry != nullptr && geometry->hasShapes();
}

bool hasAnyCollisionShapes(const detail::WorldRegistry& registry)
{
  auto view = registry.view<comps::CollisionGeometry>();
  for (auto entity : view) {
    if (view.get<comps::CollisionGeometry>(entity).hasShapes()) {
      return true;
    }
  }
  return false;
}

template <typename MultibodyView>
bool hasAnyCollisionShapesForMultibodyContacts(
    const detail::WorldRegistry& registry, const MultibodyView& view)
{
  for (auto entity : view) {
    const auto& structure
        = view.template get<comps::MultibodyStructure>(entity);
    for (const auto linkEntity : structure.links) {
      if (hasCollisionShapes(registry, linkEntity)) {
        return true;
      }
    }
  }
  return false;
}

//==============================================================================
// Recursive Newton-Euler inverse dynamics over a precomputed tree.
// Returns generalized forces for the supplied qddot and qdot.
void recursiveNewtonEulerInto(
    std::span<const LinkDynamics> links,
    const Vector6& baseAcceleration,
    std::size_t dofCount,
    const Eigen::VectorXd& qddot,
    const Eigen::VectorXd& qdot,
    Eigen::VectorXd& tau,
    auto& velocity,
    auto& acceleration,
    auto& force)
{
  const auto count = links.size();
  velocity.resize(count);
  acceleration.resize(count);
  force.resize(count);
  for (std::size_t i = 0; i < count; ++i) {
    velocity[i].setZero();
    acceleration[i].setZero();
    force[i].setZero();
  }

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

  tau.resize(static_cast<Eigen::Index>(dofCount));
  tau.setZero();
  for (std::size_t reverse = 0; reverse < count; ++reverse) {
    const auto i = count - 1 - reverse;
    const auto& link = links[i];
    if (link.dof > 0) {
      // noalias() writes the (dynamic-length) joint-space projection directly
      // into the tau segment, avoiding a per-call heap-allocated Eigen
      // temporary on every warmed step.
      tau.segment(link.dofOffset, link.dof).noalias()
          = link.subspace.transpose() * force[i];
    }
    if (link.parentIndex >= 0) {
      force[static_cast<std::size_t>(link.parentIndex)]
          += link.childToParentForce * force[i];
    }
  }
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

void computeMassAndBiasInto(
    std::span<const LinkDynamics> links,
    std::size_t dofCount,
    const Eigen::Vector3d& gravity,
    const Eigen::VectorXd& qdot,
    const Eigen::VectorXd& armature,
    MassAndBias& result,
    Eigen::VectorXd& zero,
    Eigen::VectorXd& unit,
    Eigen::VectorXd& response,
    auto& rneaVelocity,
    auto& rneaAcceleration,
    auto& rneaForce)
{
  Vector6 baseAcceleration = Vector6::Zero();
  baseAcceleration.tail<3>() = -gravity;

  const auto dof = static_cast<Eigen::Index>(dofCount);
  zero.resize(dof);
  zero.setZero();

  // Coriolis + gravity bias forces at the current velocity.
  recursiveNewtonEulerInto(
      links,
      baseAcceleration,
      dofCount,
      zero,
      qdot,
      result.bias,
      rneaVelocity,
      rneaAcceleration,
      rneaForce);

  // Static gravity-only forces (zero velocity) used to isolate the mass matrix.
  recursiveNewtonEulerInto(
      links,
      baseAcceleration,
      dofCount,
      zero,
      zero,
      result.gravityOnly,
      rneaVelocity,
      rneaAcceleration,
      rneaForce);

  // Joint-space mass matrix via unit-acceleration columns.
  result.massMatrix.resize(dof, dof);
  unit.resize(dof);
  response.resize(dof);
  for (std::size_t column = 0; column < dofCount; ++column) {
    unit.setZero();
    unit[static_cast<Eigen::Index>(column)] = 1.0;
    recursiveNewtonEulerInto(
        links,
        baseAcceleration,
        dofCount,
        unit,
        zero,
        response,
        rneaVelocity,
        rneaAcceleration,
        rneaForce);
    result.massMatrix.col(static_cast<Eigen::Index>(column))
        = response - result.gravityOnly;
  }

  // Rotor inertia (armature) adds to the joint-space mass-matrix diagonal.
  if (armature.size() == static_cast<Eigen::Index>(dofCount)) {
    result.massMatrix.diagonal() += armature;
  }
}

//==============================================================================
struct MultibodyDynamicsScratch
{
  using LinkIndex = std::pair<entt::entity, std::size_t>;
  using LinkIndexAllocator = common::StlAllocator<LinkIndex>;
  using SpatialVectorAllocator = common::StlAllocator<Vector6>;
  using LinkContactAllocator = common::StlAllocator<LinkContact>;
  using ConstrainedDofAllocator = common::StlAllocator<Eigen::Index>;
  using ConstrainedTargetAllocator = common::StlAllocator<double>;
  using Matrix6Allocator = common::StlAllocator<Matrix6>;
  using BodyJacobianAllocator = common::StlAllocator<Eigen::MatrixXd>;
  using VectorAllocator = common::StlAllocator<Eigen::VectorXd>;
  using LinkIndexVector = std::vector<LinkIndex, LinkIndexAllocator>;
  using Matrix6Vector = std::vector<Matrix6, Matrix6Allocator>;
  using SpatialVectorVector = std::vector<Vector6, SpatialVectorAllocator>;
  using LinkContactVector = std::vector<LinkContact, LinkContactAllocator>;
  using ConstrainedDofVector
      = std::vector<Eigen::Index, ConstrainedDofAllocator>;
  using ConstrainedTargetVector
      = std::vector<double, ConstrainedTargetAllocator>;
  using BodyJacobianVector
      = std::vector<Eigen::MatrixXd, BodyJacobianAllocator>;
  using VectorVector = std::vector<Eigen::VectorXd, VectorAllocator>;

  MultibodyDynamicsScratch() = default;

  explicit MultibodyDynamicsScratch(common::MemoryAllocator& allocator)
    : tree(allocator),
      linkIndexOf(LinkIndexAllocator{allocator}),
      rneaVelocity(SpatialVectorAllocator{allocator}),
      rneaAcceleration(SpatialVectorAllocator{allocator}),
      rneaForce(SpatialVectorAllocator{allocator}),
      rneaDerivativeVelocity(SpatialVectorAllocator{allocator}),
      rneaDerivativeAcceleration(SpatialVectorAllocator{allocator}),
      rneaDerivativeForce(SpatialVectorAllocator{allocator}),
      rneaDerivativeJointVelocity(SpatialVectorAllocator{allocator}),
      rneaDerivativeTotalForce(SpatialVectorAllocator{allocator}),
      rneaDerivativeDeltaVelocity(SpatialVectorAllocator{allocator}),
      rneaDerivativeDeltaAcceleration(SpatialVectorAllocator{allocator}),
      rneaDerivativeDeltaForce(SpatialVectorAllocator{allocator}),
      rneaDerivativeAccumulatedForce(SpatialVectorAllocator{allocator}),
      linkContacts(LinkContactAllocator{allocator}),
      linkOwnerMap(makeLinkOwnerMap(allocator)),
      constrainedDof(ConstrainedDofAllocator{allocator}),
      constrainedTarget(ConstrainedTargetAllocator{allocator}),
      articulated(Matrix6Allocator{allocator}),
      bias(SpatialVectorAllocator{allocator}),
      motionToChild(Matrix6Allocator{allocator}),
      spatial(SpatialVectorAllocator{allocator}),
      forceProjector(BodyJacobianAllocator{allocator}),
      jointMatrix(BodyJacobianAllocator{allocator}),
      jointMatrixInverse(BodyJacobianAllocator{allocator}),
      jointRhs(VectorAllocator{allocator}),
      contactProblem(allocator),
      bodyJacobian(BodyJacobianAllocator{allocator})
  {
  }

  DynamicsTree tree;
  LinkIndexVector linkIndexOf;
  Subspace jointFrameSubspace;
  Eigen::VectorXd qdot;
  Eigen::VectorXd appliedForce;
  Eigen::VectorXd zero;
  Eigen::VectorXd unitAcceleration;
  Eigen::VectorXd rneaResponse;
  MassAndBias massAndBias;
  Eigen::VectorXd rhs;
  Eigen::VectorXd qddot;
  Eigen::VectorXd nextVelocity;
  SpatialVectorVector rneaVelocity;
  SpatialVectorVector rneaAcceleration;
  SpatialVectorVector rneaForce;
  SpatialVectorVector rneaDerivativeVelocity;
  SpatialVectorVector rneaDerivativeAcceleration;
  SpatialVectorVector rneaDerivativeForce;
  SpatialVectorVector rneaDerivativeJointVelocity;
  SpatialVectorVector rneaDerivativeTotalForce;
  SpatialVectorVector rneaDerivativeDeltaVelocity;
  SpatialVectorVector rneaDerivativeDeltaAcceleration;
  SpatialVectorVector rneaDerivativeDeltaForce;
  SpatialVectorVector rneaDerivativeAccumulatedForce;
  LinkContactVector linkContacts;
  LinkOwnerMap linkOwnerMap;
  ConstrainedDofVector constrainedDof;
  ConstrainedTargetVector constrainedTarget;
  Matrix6Vector articulated;
  SpatialVectorVector bias;
  Matrix6Vector motionToChild;
  SpatialVectorVector spatial;
  BodyJacobianVector forceProjector;
  BodyJacobianVector jointMatrix;
  BodyJacobianVector jointMatrixInverse;
  VectorVector jointRhs;
  Eigen::VectorXd jointWork;
  Eigen::VectorXd jointSolveWork;
  MultibodyLinkContactProblem contactProblem;
  std::size_t activeContactRowCount = 0;
  BodyJacobianVector bodyJacobian;
  Eigen::MatrixXd pointJacobian;
  Eigen::MatrixXd otherPointJacobian;
  Eigen::VectorXd contactWork;
  Eigen::VectorXd contactImpulseDelta;
  Eigen::MatrixXd lockedInverseMassBlock;   // k x k locked-coordinate block
  Eigen::MatrixXd lockedInverseMassColumns; // n x k original locked columns
  Eigen::MatrixXd lockedInverseMassRhs;     // k x n original locked rows
  Eigen::MatrixXd lockedInverseMassSolve;   // k x n A_ll^-1 * A_l*

  // Persistent buffers for the shared articulated inverse-mass apply and the
  // velocity-actuator constraint solve. Reused across warmed steps so neither
  // path allocates Eigen temporaries once dimensions stabilize.
  Eigen::MatrixXd velocityMassInverse;                 // n x k constrained cols
  Eigen::MatrixXd velocityConstraintMatrix;            // k x k
  Eigen::VectorXd velocityConstraintResidual;          // k
  Eigen::LDLT<Eigen::MatrixXd> velocityConstraintLdlt; // k x k
  Eigen::VectorXd velocityConstraintLambda;            // k
  Eigen::MatrixXd symmetricFactor;                     // reusable lower factor
  Eigen::VectorXd symmetricForwardWork;                // reusable solve work
  Eigen::VectorXd symmetricUnitRhs;                    // reusable inverse RHS
  Eigen::VectorXd symmetricSolution; // reusable inverse column
};

MultibodyDynamicsScratch& getOrEmplaceMultibodyDynamicsScratch(
    World& world, detail::WorldRegistry& registry, entt::entity entity)
{
  if (auto* scratch = registry.try_get<MultibodyDynamicsScratch>(entity)) {
    return *scratch;
  }
  return registry.emplace<MultibodyDynamicsScratch>(
      entity, world.getMemoryManager().getFreeAllocator());
}

void reserveDynamicsTreeInverseMassScratch(
    const DynamicsTree& tree, MultibodyDynamicsScratch& scratch)
{
  detail::reserveArticulatedInverseMassScratch(
      tree.links.size(),
      static_cast<Eigen::Index>(tree.dofCount),
      [&](std::size_t i) { return tree.links[i].dof; },
      scratch);
}

void applyDynamicsTreeInverseMassInto(
    const DynamicsTree& tree,
    const Eigen::VectorXd& impulse,
    MultibodyDynamicsScratch& scratch,
    Eigen::VectorXd& result)
{
  const auto linkAt = [&](std::size_t i) {
    const LinkDynamics& link = tree.links[i];
    return detail::ArticulatedInverseMassLink{
        link.parentIndex,
        link.dof,
        link.dofOffset,
        &link.inertia,
        &link.subspace,
        link.parentToChild};
  };
  detail::applyArticulatedInverseMassInto(
      tree.links.size(),
      tree.dofCount,
      tree.armature,
      impulse,
      linkAt,
      scratch,
      result);
}

void inverseMassMatrixColumnsInto(
    const DynamicsTree& tree,
    MultibodyDynamicsScratch& scratch,
    Eigen::MatrixXd& inverseMass)
{
  const auto dof = static_cast<Eigen::Index>(tree.dofCount);
  inverseMass.resize(dof, dof);
  scratch.unitAcceleration.resize(dof);
  for (Eigen::Index column = 0; column < dof; ++column) {
    scratch.unitAcceleration.setZero();
    scratch.unitAcceleration[column] = 1.0;
    applyDynamicsTreeInverseMassInto(
        tree, scratch.unitAcceleration, scratch, scratch.contactWork);
    inverseMass.col(column) = scratch.contactWork;
  }
}

//==============================================================================
bool solveSpdSystemInto(
    const Eigen::MatrixXd& matrix,
    const Eigen::VectorXd& rhs,
    Eigen::VectorXd& solution,
    Eigen::MatrixXd& lowerFactor,
    Eigen::VectorXd& forwardWork);

//==============================================================================
bool invertSpdMatrixInto(
    const Eigen::MatrixXd& matrix,
    Eigen::MatrixXd& inverse,
    Eigen::MatrixXd& lowerFactor,
    Eigen::VectorXd& forwardWork,
    Eigen::VectorXd& unitRhs,
    Eigen::VectorXd& solution);

//==============================================================================
void gatherMultibodyVelocityInto(
    const detail::WorldRegistry& registry,
    const DynamicsTree& tree,
    Eigen::VectorXd& qdot)
{
  qdot.resize(static_cast<Eigen::Index>(tree.dofCount));
  qdot.setZero();
  for (std::size_t i = 0; i < tree.links.size(); ++i) {
    if (tree.links[i].dof == 0) {
      continue;
    }
    const auto& jointState = registry.get<comps::JointState>(tree.jointOf[i]);
    qdot.segment(tree.links[i].dofOffset, tree.links[i].dof)
        = jointState.velocity;
  }
}

bool computeUnconstrainedMultibodyVelocityInto(
    detail::WorldRegistry& registry,
    const comps::MultibodyStructure& structure,
    const Eigen::Vector3d& gravity,
    double timeStep,
    MultibodyDynamicsScratch& scratch)
{
  if (structure.links.empty()) {
    scratch.nextVelocity.resize(0);
    return false;
  }

  buildDynamicsTreeInto(
      registry,
      structure,
      scratch.tree,
      scratch.linkIndexOf,
      scratch.jointFrameSubspace);
  if (scratch.tree.dofCount == 0) {
    scratch.nextVelocity.resize(0);
    return false;
  }
  const auto n = static_cast<Eigen::Index>(scratch.tree.dofCount);

  // Current generalized velocity plus applied and passive (spring/damping)
  // generalized efforts.
  gatherMultibodyVelocityInto(registry, scratch.tree, scratch.qdot);
  scratch.appliedForce.resize(n);
  scratch.appliedForce.setZero();
  for (std::size_t i = 0; i < scratch.tree.links.size(); ++i) {
    if (scratch.tree.links[i].dof == 0) {
      continue;
    }
    const auto& jointModel
        = registry.get<comps::JointModel>(scratch.tree.jointOf[i]);
    const auto& jointState
        = registry.get<comps::JointState>(scratch.tree.jointOf[i]);
    const auto& jointActuation
        = registry.get<comps::JointActuation>(scratch.tree.jointOf[i]);

    // Determine the commanded actuation effort by actuator type. Force applies
    // the clamped joint effort; Passive applies none. Passive spring and
    // damping forces are not subject to the effort limits and always apply.
    const auto dof = scratch.tree.links[i].dof;
    switch (jointActuation.actuatorType) {
      case comps::ActuatorType::Force:
        for (std::size_t d = 0; d < dof; ++d) {
          const auto local = static_cast<Eigen::Index>(d);
          const auto global
              = static_cast<Eigen::Index>(scratch.tree.links[i].dofOffset + d);
          const double effort = std::clamp(
              jointActuation.torque[local],
              jointModel.limits.effortLower[local],
              jointModel.limits.effortUpper[local]);
          scratch.appliedForce[global]
              = effort
                - jointModel.springStiffness[local]
                      * (jointState.position[local]
                         - jointModel.restPosition[local])
                - jointModel.dampingCoefficient[local]
                      * jointState.velocity[local];
        }
        break;
      case comps::ActuatorType::Passive:
      case comps::ActuatorType::Velocity:
        // Passive applies no commanded effort; Velocity is driven by a
        // velocity-level constraint solved after the unconstrained step.
        for (std::size_t d = 0; d < dof; ++d) {
          const auto local = static_cast<Eigen::Index>(d);
          const auto global
              = static_cast<Eigen::Index>(scratch.tree.links[i].dofOffset + d);
          scratch.appliedForce[global]
              = -jointModel.springStiffness[local]
                    * (jointState.position[local]
                       - jointModel.restPosition[local])
                - jointModel.dampingCoefficient[local]
                      * jointState.velocity[local];
        }
        break;
      case comps::ActuatorType::Locked:
        // A locked joint is held rigidly by a velocity-level equality
        // constraint (target velocity 0) solved after the unconstrained step,
        // so no commanded effort or passive spring/damping force is applied to
        // its coordinates: the constraint reaction supplies whatever holding
        // force is required. The projected result is provably independent of
        // any generalized force applied to a fully constrained coordinate, so
        // zeroing here keeps the "kinematically prescribed" intent explicit and
        // avoids computing forces the constraint would only cancel.
        for (std::size_t d = 0; d < dof; ++d) {
          const auto global
              = static_cast<Eigen::Index>(scratch.tree.links[i].dofOffset + d);
          scratch.appliedForce[global] = 0.0;
        }
        break;
      default:
        DART_SIMULATION_THROW_T(
            InvalidOperationException,
            "Joint actuator type is not yet implemented in the "
            "articulated-body forward dynamics; supported types are Force, "
            "Passive, Velocity, and Locked");
    }
  }

  // Map each link's accumulated external wrench (body frame, [angular; linear])
  // to a generalized force via its body Jacobian: appliedForce += J_i^T w_i.
  // tree.links[i] is in structure order, so structure.links[i] is link i.
  {
    bool anyExternalForce = false;
    for (const auto linkEntity : structure.links) {
      if (!registry.get<comps::LinkControl>(linkEntity)
               .externalForce.isZero()) {
        anyExternalForce = true;
        break;
      }
    }
    if (anyExternalForce) {
      linkBodyJacobiansInto(scratch.tree, scratch.bodyJacobian);
      for (std::size_t i = 0; i < scratch.tree.links.size(); ++i) {
        const Vector6 wrench
            = registry.get<comps::LinkControl>(structure.links[i])
                  .externalForce;
        if (!wrench.isZero()) {
          scratch.appliedForce.noalias()
              += scratch.bodyJacobian[i].transpose() * wrench;
        }
      }
    }
  }

  computeMassAndBiasInto(
      linkSpan(scratch.tree),
      scratch.tree.dofCount,
      gravity,
      scratch.qdot,
      scratch.tree.armature,
      scratch.massAndBias,
      scratch.zero,
      scratch.unitAcceleration,
      scratch.rneaResponse,
      scratch.rneaVelocity,
      scratch.rneaAcceleration,
      scratch.rneaForce);

  scratch.rhs.resize(n);
  scratch.rhs = scratch.appliedForce - scratch.massAndBias.bias;
  applyDynamicsTreeInverseMassInto(
      scratch.tree, scratch.rhs, scratch, scratch.qddot);

  // Unconstrained next generalized velocity (semi-implicit Euler), then apply
  // velocity-level effects on the global vector before integrating positions.
  scratch.nextVelocity.resize(n);
  scratch.nextVelocity = scratch.qdot + scratch.qddot * timeStep;

  // Coulomb (dry) joint friction as a bounded velocity-level impulse: it stops
  // a coordinate when the holding impulse is within the friction bound
  // (stiction) and otherwise opposes motion at the friction magnitude.
  for (std::size_t i = 0; i < scratch.tree.links.size(); ++i) {
    const auto dof = scratch.tree.links[i].dof;
    if (dof == 0) {
      continue;
    }
    const auto& jointActuation
        = registry.get<comps::JointActuation>(scratch.tree.jointOf[i]);
    if (jointActuation.actuatorType == comps::ActuatorType::Locked) {
      continue;
    }
    const auto& jointModel
        = registry.get<comps::JointModel>(scratch.tree.jointOf[i]);
    if (jointModel.coulombFriction.size() != static_cast<Eigen::Index>(dof)) {
      continue;
    }
    for (std::size_t d = 0; d < dof; ++d) {
      const double bound
          = jointModel.coulombFriction[static_cast<Eigen::Index>(d)] * timeStep;
      if (bound <= 0.0) {
        continue;
      }
      const auto globalDof
          = static_cast<Eigen::Index>(scratch.tree.links[i].dofOffset + d);
      const double effInertia
          = scratch.massAndBias.massMatrix(globalDof, globalDof);
      const double stopImpulse = effInertia * scratch.nextVelocity[globalDof];
      const double frictionImpulse = std::clamp(stopImpulse, -bound, bound);
      scratch.nextVelocity[globalDof] -= frictionImpulse / effInertia;
    }
  }

  // Velocity- and Locked-actuated joints: solve a coupled velocity-level
  // equality constraint that drives the selected coordinates to their target
  // velocities (the commanded velocity for Velocity, zero for Locked),
  // lambda = (J M^-1 J^T)^-1 (target - J nextVelocity),
  // nextVelocity += M^-1 J^T lambda, where J selects the constrained
  // coordinates.
  scratch.constrainedDof.clear();
  scratch.constrainedTarget.clear();
  for (std::size_t i = 0; i < scratch.tree.links.size(); ++i) {
    const auto dof = scratch.tree.links[i].dof;
    if (dof == 0) {
      continue;
    }
    const auto& jointActuation
        = registry.get<comps::JointActuation>(scratch.tree.jointOf[i]);
    // Velocity and Locked actuators both drive their coordinates through the
    // same velocity-level equality constraint: Velocity to its commanded
    // velocity, Locked to zero (holding the joint at its current position).
    const bool velocityActuated
        = jointActuation.actuatorType == comps::ActuatorType::Velocity;
    const bool lockedActuated
        = jointActuation.actuatorType == comps::ActuatorType::Locked;
    if (!velocityActuated && !lockedActuated) {
      continue;
    }
    for (std::size_t d = 0; d < dof; ++d) {
      scratch.constrainedDof.push_back(
          static_cast<Eigen::Index>(scratch.tree.links[i].dofOffset + d));
      scratch.constrainedTarget.push_back(
          lockedActuated ? 0.0
          : jointActuation.commandVelocity.size()
                  == static_cast<Eigen::Index>(dof)
              ? jointActuation.commandVelocity[static_cast<Eigen::Index>(d)]
              : 0.0);
    }
  }
  if (!scratch.constrainedDof.empty()) {
    const auto k = static_cast<Eigen::Index>(scratch.constrainedDof.size());
    // Build only the constrained columns M^-1 e_i through the shared
    // articulated inverse-mass apply. The k x k constraint solve remains dense,
    // but the multibody side no longer forms or factorizes a full inverse.
    scratch.velocityMassInverse.resize(n, k);
    scratch.unitAcceleration.resize(n);
    for (Eigen::Index b = 0; b < k; ++b) {
      scratch.unitAcceleration.setZero();
      scratch
          .unitAcceleration[scratch.constrainedDof[static_cast<std::size_t>(b)]]
          = 1.0;
      applyDynamicsTreeInverseMassInto(
          scratch.tree, scratch.unitAcceleration, scratch, scratch.contactWork);
      scratch.velocityMassInverse.col(b) = scratch.contactWork;
    }
    scratch.velocityConstraintMatrix.resize(k, k);
    scratch.velocityConstraintResidual.resize(k);
    for (Eigen::Index a = 0; a < k; ++a) {
      scratch.velocityConstraintResidual[a]
          = scratch.constrainedTarget[static_cast<std::size_t>(a)]
            - scratch.nextVelocity
                  [scratch.constrainedDof[static_cast<std::size_t>(a)]];
      for (Eigen::Index b = 0; b < k; ++b) {
        scratch.velocityConstraintMatrix(a, b) = scratch.velocityMassInverse(
            scratch.constrainedDof[static_cast<std::size_t>(a)], b);
      }
    }
    if (!solveSpdSystemInto(
            scratch.velocityConstraintMatrix,
            scratch.velocityConstraintResidual,
            scratch.velocityConstraintLambda,
            scratch.symmetricFactor,
            scratch.symmetricForwardWork)) {
      return false;
    }
    for (Eigen::Index a = 0; a < k; ++a) {
      scratch.nextVelocity.noalias() += scratch.velocityMassInverse.col(a)
                                        * scratch.velocityConstraintLambda[a];
    }
  }

  return true;
}

// Relative contact-point velocity along a world direction: the link's point
// velocity (via its Jacobian and the generalized velocity) minus a two-sided
// dynamic rigid obstacle's point velocity (zero for an immovable obstacle).
double linkContactRelativeVelocity(
    const detail::WorldRegistry& registry,
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
template <typename BodyJacobianVector>
void pointLinearJacobianInto(
    const DynamicsTree& tree,
    const BodyJacobianVector& bodyJacobian,
    std::size_t linkIndex,
    const Eigen::Vector3d& point,
    Eigen::MatrixXd& pointJacobian)
{
  const Eigen::Index dofCount = static_cast<Eigen::Index>(tree.dofCount);
  pointJacobian.resize(3, dofCount);
  if (dofCount == 0) {
    return;
  }

  const Eigen::Matrix3d rotation
      = tree.links[linkIndex].worldTransform.linear();
  const Eigen::Vector3d origin
      = tree.links[linkIndex].worldTransform.translation();
  const Eigen::Vector3d arm = point - origin;
  const auto& linkJacobian = bodyJacobian[linkIndex];
  for (Eigen::Index column = 0; column < dofCount; ++column) {
    const Eigen::Vector3d angular
        = rotation * linkJacobian.col(column).template head<3>();
    const Eigen::Vector3d linear
        = rotation * linkJacobian.col(column).template tail<3>();
    pointJacobian.col(column) = linear - arm.cross(angular);
  }
}

//==============================================================================
void multiplyPointJacobianTransposeInto(
    const Eigen::MatrixXd& pointJacobian,
    const Eigen::Vector3d& direction,
    Eigen::VectorXd& jacobian)
{
  const Eigen::Index dofCount = pointJacobian.cols();
  jacobian.resize(dofCount);
  for (Eigen::Index column = 0; column < dofCount; ++column) {
    jacobian[column] = pointJacobian.col(column).dot(direction);
  }
}

//==============================================================================
double jointSpaceDenominator(
    const Eigen::MatrixXd& inverseMass,
    const Eigen::VectorXd& jacobian,
    Eigen::VectorXd& work)
{
  if (jacobian.size() == 0) {
    return 0.0;
  }
  if (jacobian.size() == 1) {
    return jacobian[0] * inverseMass(0, 0) * jacobian[0];
  }

  work.resize(jacobian.size());
  work.noalias() = inverseMass * jacobian;
  return jacobian.dot(work);
}

//==============================================================================
bool factorSpdLowerInto(
    const Eigen::MatrixXd& matrix, Eigen::MatrixXd& lowerFactor)
{
  const Eigen::Index n = matrix.rows();
  if (matrix.cols() != n) {
    return false;
  }

  if (lowerFactor.rows() < n || lowerFactor.cols() < n) {
    lowerFactor.resize(
        std::max(lowerFactor.rows(), n), std::max(lowerFactor.cols(), n));
  }
  lowerFactor.topLeftCorner(n, n).setZero();
  for (Eigen::Index i = 0; i < n; ++i) {
    for (Eigen::Index j = 0; j <= i; ++j) {
      double sum = matrix(i, j);
      for (Eigen::Index k = 0; k < j; ++k) {
        sum -= lowerFactor(i, k) * lowerFactor(j, k);
      }
      if (i == j) {
        if (sum <= 0.0 || !std::isfinite(sum)) {
          return false;
        }
        lowerFactor(i, j) = std::sqrt(sum);
      } else {
        const double diagonal = lowerFactor(j, j);
        if (diagonal == 0.0) {
          return false;
        }
        lowerFactor(i, j) = sum / diagonal;
      }
    }
  }
  return true;
}

//==============================================================================
bool solveSpdSystemInto(
    const Eigen::MatrixXd& matrix,
    const Eigen::VectorXd& rhs,
    Eigen::VectorXd& solution,
    Eigen::MatrixXd& lowerFactor,
    Eigen::VectorXd& forwardWork)
{
  const Eigen::Index n = matrix.rows();
  if (matrix.cols() != n || rhs.size() != n) {
    return false;
  }
  solution.resize(n);
  if (n == 0) {
    return true;
  }
  if (n == 1) {
    const double diagonal = matrix(0, 0);
    if (diagonal == 0.0) {
      return false;
    }
    solution[0] = rhs[0] / diagonal;
    return true;
  }

  if (!factorSpdLowerInto(matrix, lowerFactor)) {
    return false;
  }

  if (forwardWork.size() < n) {
    forwardWork.resize(n);
  }
  for (Eigen::Index i = 0; i < n; ++i) {
    double sum = rhs[i];
    for (Eigen::Index j = 0; j < i; ++j) {
      sum -= lowerFactor(i, j) * forwardWork[j];
    }
    forwardWork[i] = sum / lowerFactor(i, i);
  }
  for (Eigen::Index i = n; i-- > 0;) {
    double sum = forwardWork[i];
    for (Eigen::Index j = i + 1; j < n; ++j) {
      sum -= lowerFactor(j, i) * solution[j];
    }
    solution[i] = sum / lowerFactor(i, i);
  }
  return true;
}

//==============================================================================
bool solveSpdMatrixInto(
    const Eigen::MatrixXd& matrix,
    const Eigen::MatrixXd& rhs,
    Eigen::MatrixXd& solution,
    Eigen::MatrixXd& lowerFactor,
    Eigen::VectorXd& forwardWork)
{
  const Eigen::Index n = matrix.rows();
  if (matrix.cols() != n || rhs.rows() != n) {
    return false;
  }
  solution.resize(n, rhs.cols());
  if (n == 0 || rhs.cols() == 0) {
    return true;
  }
  if (n == 1) {
    const double diagonal = matrix(0, 0);
    if (diagonal == 0.0) {
      return false;
    }
    solution.row(0) = rhs.row(0) / diagonal;
    return true;
  }

  if (!factorSpdLowerInto(matrix, lowerFactor)) {
    return false;
  }

  if (forwardWork.size() < n) {
    forwardWork.resize(n);
  }
  for (Eigen::Index column = 0; column < rhs.cols(); ++column) {
    for (Eigen::Index i = 0; i < n; ++i) {
      double sum = rhs(i, column);
      for (Eigen::Index j = 0; j < i; ++j) {
        sum -= lowerFactor(i, j) * forwardWork[j];
      }
      forwardWork[i] = sum / lowerFactor(i, i);
    }
    for (Eigen::Index i = n; i-- > 0;) {
      double sum = forwardWork[i];
      for (Eigen::Index j = i + 1; j < n; ++j) {
        sum -= lowerFactor(j, i) * solution(j, column);
      }
      solution(i, column) = sum / lowerFactor(i, i);
    }
  }
  return true;
}

//==============================================================================
bool invertSpdMatrixInto(
    const Eigen::MatrixXd& matrix,
    Eigen::MatrixXd& inverse,
    Eigen::MatrixXd& lowerFactor,
    Eigen::VectorXd& forwardWork,
    Eigen::VectorXd& unitRhs,
    Eigen::VectorXd& solution)
{
  const Eigen::Index n = matrix.rows();
  if (matrix.cols() != n) {
    return false;
  }
  inverse.resize(n, n);
  if (n == 0) {
    return true;
  }
  if (n == 1) {
    const double diagonal = matrix(0, 0);
    if (diagonal == 0.0) {
      return false;
    }
    inverse(0, 0) = 1.0 / diagonal;
    return true;
  }

  if (!factorSpdLowerInto(matrix, lowerFactor)) {
    return false;
  }

  if (unitRhs.size() < n) {
    unitRhs.resize(n);
  }
  if (solution.size() < n) {
    solution.resize(n);
  }
  if (forwardWork.size() < n) {
    forwardWork.resize(n);
  }
  for (Eigen::Index column = 0; column < n; ++column) {
    unitRhs.head(n).setZero();
    unitRhs[column] = 1.0;
    for (Eigen::Index i = 0; i < n; ++i) {
      double sum = unitRhs[i];
      for (Eigen::Index j = 0; j < i; ++j) {
        sum -= lowerFactor(i, j) * forwardWork[j];
      }
      forwardWork[i] = sum / lowerFactor(i, i);
    }
    for (Eigen::Index i = n; i-- > 0;) {
      double sum = forwardWork[i];
      for (Eigen::Index j = i + 1; j < n; ++j) {
        sum -= lowerFactor(j, i) * solution[j];
      }
      solution[i] = sum / lowerFactor(i, i);
    }
    inverse.col(column) = solution.head(n);
  }
  return true;
}

//==============================================================================
void resetMultibodyLinkContactRow(
    MultibodyLinkContactRow& row, Eigen::Index dof)
{
  row.normalJacobian.resize(dof);
  row.normalJacobian.setZero();
  row.tangentJacobian1.resize(dof);
  row.tangentJacobian1.setZero();
  row.tangentJacobian2.resize(dof);
  row.tangentJacobian2.setZero();
  row.otherNormalJacobian.setZero();
  row.otherTangentJacobian1.setZero();
  row.otherTangentJacobian2.setZero();
  row.normal = Eigen::Vector3d::UnitZ();
  row.tangent1.setZero();
  row.tangent2.setZero();
  row.point.setZero();
  row.normalDenominator = 0.0;
  row.tangentDenominator1 = 0.0;
  row.tangentDenominator2 = 0.0;
  row.bias = 0.0;
  row.restitutionTarget = 0.0;
  row.friction = 1.0;
  row.normalRhs = 0.0;
  row.tangentRhs1 = 0.0;
  row.tangentRhs2 = 0.0;
  row.normalImpulse = 0.0;
  row.tangentImpulse1 = 0.0;
  row.tangentImpulse2 = 0.0;
  row.restitution = 0.0;
  row.otherBody = entt::null;
  row.otherLink = entt::null;
  row.otherMultibody = entt::null;
  row.otherMultibodyIndex = -1;
  row.otherInvMass = 0.0;
  row.otherInvInertia.setZero();
  row.otherArm.setZero();
  row.active = false;
}

//==============================================================================
void ensureMultibodyLinkContactRowStorage(
    MultibodyDynamicsScratch& scratch, std::size_t rowCount, Eigen::Index dof)
{
  auto& rows = scratch.contactProblem.rows;
  if (rows.size() < rowCount) {
    const std::size_t oldSize = rows.size();
    rows.resize(rowCount);
    for (std::size_t i = oldSize; i < rows.size(); ++i) {
      resetMultibodyLinkContactRow(rows[i], dof);
    }
  } else if (rows.size() > rowCount) {
    rows.resize(rowCount);
  }
  for (std::size_t i = 0; i < rowCount; ++i) {
    resetMultibodyLinkContactRow(rows[i], dof);
  }
  scratch.activeContactRowCount = rowCount;
}

//==============================================================================
bool projectLockedDofsOutOfContactInverseMassInto(
    const detail::WorldRegistry& registry,
    const DynamicsTree& tree,
    MultibodyDynamicsScratch& scratch,
    Eigen::MatrixXd& inverseMass)
{
  const auto n = static_cast<Eigen::Index>(tree.dofCount);
  if (inverseMass.rows() != n || inverseMass.cols() != n) {
    return false;
  }

  scratch.constrainedDof.clear();
  for (std::size_t i = 0; i < tree.links.size(); ++i) {
    const auto& link = tree.links[i];
    if (link.dof == 0 || link.jointEntity == entt::null) {
      continue;
    }
    const auto& jointActuation
        = registry.get<comps::JointActuation>(link.jointEntity);
    if (jointActuation.actuatorType != comps::ActuatorType::Locked) {
      continue;
    }
    for (std::size_t d = 0; d < link.dof; ++d) {
      scratch.constrainedDof.push_back(
          static_cast<Eigen::Index>(link.dofOffset + d));
    }
  }
  const auto k = static_cast<Eigen::Index>(scratch.constrainedDof.size());
  if (k == 0) {
    return true;
  }

  scratch.lockedInverseMassBlock.resize(k, k);
  scratch.lockedInverseMassColumns.resize(n, k);
  scratch.lockedInverseMassRhs.resize(k, n);
  scratch.lockedInverseMassSolve.resize(k, n);
  for (Eigen::Index a = 0; a < k; ++a) {
    const Eigen::Index lockedA
        = scratch.constrainedDof[static_cast<std::size_t>(a)];
    scratch.lockedInverseMassColumns.col(a) = inverseMass.col(lockedA);
    scratch.lockedInverseMassRhs.row(a) = inverseMass.row(lockedA);
    for (Eigen::Index b = 0; b < k; ++b) {
      const Eigen::Index lockedB
          = scratch.constrainedDof[static_cast<std::size_t>(b)];
      scratch.lockedInverseMassBlock(a, b) = inverseMass(lockedA, lockedB);
    }
  }

  if (!solveSpdMatrixInto(
          scratch.lockedInverseMassBlock,
          scratch.lockedInverseMassRhs,
          scratch.lockedInverseMassSolve,
          scratch.symmetricFactor,
          scratch.symmetricForwardWork)) {
    return false;
  }

  for (Eigen::Index row = 0; row < n; ++row) {
    for (Eigen::Index column = 0; column < n; ++column) {
      double correction = 0.0;
      for (Eigen::Index a = 0; a < k; ++a) {
        correction += scratch.lockedInverseMassColumns(row, a)
                      * scratch.lockedInverseMassSolve(a, column);
      }
      inverseMass(row, column) -= correction;
    }
  }

  for (Eigen::Index a = 0; a < k; ++a) {
    const Eigen::Index locked
        = scratch.constrainedDof[static_cast<std::size_t>(a)];
    inverseMass.row(locked).setZero();
    inverseMass.col(locked).setZero();
  }
  return true;
}

//==============================================================================
bool prepareMultibodyContactDynamicsInto(
    const detail::WorldRegistry& registry,
    const comps::MultibodyStructure& structure,
    const Eigen::VectorXd& nextVelocity,
    MultibodyDynamicsScratch& scratch)
{
  if (structure.links.empty()) {
    return false;
  }

  buildDynamicsTreeInto(
      registry,
      structure,
      scratch.tree,
      scratch.linkIndexOf,
      scratch.jointFrameSubspace);

  DART_SIMULATION_THROW_T_IF(
      nextVelocity.size() != static_cast<Eigen::Index>(scratch.tree.dofCount),
      InvalidArgumentException,
      "Staged multibody velocity dimension ({}) does not match the expected "
      "DOF count ({})",
      nextVelocity.size(),
      scratch.tree.dofCount);

  auto& problem = scratch.contactProblem;
  if (scratch.tree.dofCount == 0) {
    problem.inverseMass.resize(0, 0);
    linkBodyJacobiansInto(scratch.tree, scratch.bodyJacobian);
    return true;
  }

  inverseMassMatrixColumnsInto(scratch.tree, scratch, problem.inverseMass);
  if (!projectLockedDofsOutOfContactInverseMassInto(
          registry, scratch.tree, scratch, problem.inverseMass)) {
    return false;
  }

  linkBodyJacobiansInto(scratch.tree, scratch.bodyJacobian);
  return true;
}

//==============================================================================
bool assembleMultibodyLinkContactProblemInto(
    const detail::WorldRegistry& registry,
    const comps::MultibodyStructure& structure,
    const Eigen::VectorXd& nextVelocity,
    double timeStep,
    std::span<const LinkContact> linkContacts,
    MultibodyDynamicsScratch& scratch)
{
  // Assemble the per-contact rows (Jacobians, denominators, bias, restitution,
  // two-sided obstacle coupling) once. The assembly is shared with the unified
  // constraint solve as the link-side counterpart of the rigid-body assembler.
  auto& problem = scratch.contactProblem;
  scratch.activeContactRowCount = 0;
  if (structure.links.empty()) {
    problem.rows.resize(0);
    problem.inverseMass.resize(0, 0);
    return false;
  }

  if (!prepareMultibodyContactDynamicsInto(
          registry, structure, nextVelocity, scratch)) {
    problem.rows.resize(0);
    problem.inverseMass.resize(0, 0);
    return false;
  }

  const auto dof = static_cast<Eigen::Index>(scratch.tree.dofCount);
  ensureMultibodyLinkContactRowStorage(scratch, linkContacts.size(), dof);
  constexpr double penetrationSlop = 1e-4;
  constexpr double baumgarteFactor = 0.2;

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

    pointLinearJacobianInto(
        scratch.tree,
        scratch.bodyJacobian,
        index,
        contact.point,
        scratch.pointJacobian);
    if (contact.otherLink != entt::null) {
      const auto otherLinkIt = std::find(
          structure.links.begin(), structure.links.end(), contact.otherLink);
      if (otherLinkIt == linkIt) {
        continue;
      }
      if (otherLinkIt != structure.links.end()) {
        const auto otherIndex
            = static_cast<std::size_t>(otherLinkIt - structure.links.begin());
        pointLinearJacobianInto(
            scratch.tree,
            scratch.bodyJacobian,
            otherIndex,
            contact.point,
            scratch.otherPointJacobian);
        scratch.pointJacobian -= scratch.otherPointJacobian;
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
    multiplyPointJacobianTransposeInto(
        scratch.pointJacobian, normal, row.normalJacobian);
    const Eigen::Vector3d normalArm = row.otherArm.cross(normal);
    row.normalDenominator
        = jointSpaceDenominator(
              problem.inverseMass, row.normalJacobian, scratch.contactWork)
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
    multiplyPointJacobianTransposeInto(
        scratch.pointJacobian, tangent1, row.tangentJacobian1);
    multiplyPointJacobianTransposeInto(
        scratch.pointJacobian, tangent2, row.tangentJacobian2);
    const Eigen::Vector3d tangentArm1 = row.otherArm.cross(tangent1);
    const Eigen::Vector3d tangentArm2 = row.otherArm.cross(tangent2);
    row.tangentDenominator1
        = jointSpaceDenominator(
              problem.inverseMass, row.tangentJacobian1, scratch.contactWork)
          + row.otherInvMass
          + tangentArm1.dot(row.otherInvInertia * tangentArm1);
    row.tangentDenominator2
        = jointSpaceDenominator(
              problem.inverseMass, row.tangentJacobian2, scratch.contactWork)
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

  return true;
}

//==============================================================================
void solveMultibodyLinkContacts(
    detail::WorldRegistry& registry,
    const comps::MultibodyStructure& structure,
    Eigen::VectorXd& nextVelocity,
    double timeStep,
    std::span<const LinkContact> linkContacts,
    MultibodyDynamicsScratch& scratch)
{
  if (!assembleMultibodyLinkContactProblemInto(
          registry, structure, nextVelocity, timeStep, linkContacts, scratch)) {
    return;
  }

  auto& problem = scratch.contactProblem;
  auto& rows = problem.rows;
  const Eigen::MatrixXd& inverseMass = problem.inverseMass;
  if (scratch.activeContactRowCount == 0 || inverseMass.size() == 0) {
    return;
  }

  constexpr int contactIterations = 8;

  // Apply a contact impulse: drive the link via M^-1 J^T and apply the
  // equal-and-opposite impulse to the rigid obstacle (Newton's third law).
  const auto applyImpulse = [&](const MultibodyLinkContactRow& row,
                                const Eigen::VectorXd& jacobian,
                                const Eigen::Vector3d& direction,
                                double deltaImpulse) {
    if (jacobian.size() == 1) {
      nextVelocity[0] += inverseMass(0, 0) * jacobian[0] * deltaImpulse;
    } else {
      scratch.contactImpulseDelta.resize(jacobian.size());
      scratch.contactImpulseDelta.noalias() = inverseMass * jacobian;
      nextVelocity.noalias() += scratch.contactImpulseDelta * deltaImpulse;
    }
    if (row.otherBody != entt::null) {
      auto& velocity = registry.get<comps::Velocity>(row.otherBody);
      velocity.linear -= deltaImpulse * row.otherInvMass * direction;
      velocity.angular
          -= deltaImpulse * row.otherInvInertia * row.otherArm.cross(direction);
    }
  };

  for (int iteration = 0; iteration < contactIterations; ++iteration) {
    for (auto& row : std::span<MultibodyLinkContactRow>(
             rows.data(), scratch.activeContactRowCount)) {
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
void completeSequentialCrossMultibodyLinkRows(
    detail::WorldRegistry& registry,
    Eigen::VectorXd& primaryVelocity,
    MultibodyDynamicsScratch& scratch)
{
  auto& problem = scratch.contactProblem;
  const auto rowCount = scratch.activeContactRowCount;
  if (rowCount == 0 || problem.inverseMass.size() == 0) {
    return;
  }

  constexpr double restitutionThreshold = 1e-2;
  for (auto& row :
       std::span<MultibodyLinkContactRow>(problem.rows.data(), rowCount)) {
    if (row.otherMultibody == entt::null) {
      continue;
    }

    auto* otherScratch
        = registry.try_get<MultibodyDynamicsScratch>(row.otherMultibody);
    auto* otherPending
        = registry.try_get<PendingMultibodyVelocity>(row.otherMultibody);
    if (otherScratch == nullptr || otherPending == nullptr
        || !otherPending->active) {
      continue;
    }

    const auto& otherStructure
        = registry.get<comps::MultibodyStructure>(row.otherMultibody);
    const auto otherIndex = linkIndexOf(otherStructure, row.otherLink);
    pointLinearJacobianInto(
        otherScratch->tree,
        otherScratch->bodyJacobian,
        otherIndex,
        row.point,
        otherScratch->otherPointJacobian);

    multiplyPointJacobianTransposeInto(
        otherScratch->otherPointJacobian, row.normal, row.otherNormalJacobian);
    multiplyPointJacobianTransposeInto(
        otherScratch->otherPointJacobian,
        row.tangent1,
        row.otherTangentJacobian1);
    multiplyPointJacobianTransposeInto(
        otherScratch->otherPointJacobian,
        row.tangent2,
        row.otherTangentJacobian2);

    const auto& otherInverseMass = otherScratch->contactProblem.inverseMass;
    row.normalDenominator += jointSpaceDenominator(
        otherInverseMass, row.otherNormalJacobian, otherScratch->contactWork);
    row.tangentDenominator1 += jointSpaceDenominator(
        otherInverseMass, row.otherTangentJacobian1, otherScratch->contactWork);
    row.tangentDenominator2 += jointSpaceDenominator(
        otherInverseMass, row.otherTangentJacobian2, otherScratch->contactWork);

    const auto relativeVelocity = [&](const Eigen::VectorXd& primaryJacobian,
                                      const Eigen::VectorXd& otherJacobian) {
      return primaryJacobian.dot(primaryVelocity)
             - otherJacobian.dot(otherPending->velocity);
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

//==============================================================================
void applyMultibodyImpulse(
    Eigen::VectorXd& velocity,
    const Eigen::MatrixXd& inverseMass,
    const Eigen::VectorXd& jacobian,
    double deltaImpulse,
    Eigen::VectorXd& work)
{
  if (jacobian.size() == 0) {
    return;
  }
  if (jacobian.size() == 1) {
    velocity[0] += inverseMass(0, 0) * jacobian[0] * deltaImpulse;
    return;
  }

  work.resize(jacobian.size());
  work.noalias() = inverseMass * jacobian;
  velocity.noalias() += work * deltaImpulse;
}

//==============================================================================
void solveSequentialCrossMultibodyLinkRows(detail::WorldRegistry& registry)
{
  constexpr int contactIterations = 8;
  auto view = registry.view<comps::MultibodyStructure>();

  for (int iteration = 0; iteration < contactIterations; ++iteration) {
    for (auto entity : view) {
      auto* pendingVelocity
          = registry.try_get<PendingMultibodyVelocity>(entity);
      if (pendingVelocity == nullptr || !pendingVelocity->active) {
        continue;
      }

      auto& scratch = registry.get<MultibodyDynamicsScratch>(entity);
      auto& problem = scratch.contactProblem;
      const auto rowCount = scratch.activeContactRowCount;
      if (rowCount == 0 || problem.inverseMass.size() == 0) {
        continue;
      }

      for (auto& row :
           std::span<MultibodyLinkContactRow>(problem.rows.data(), rowCount)) {
        if (!row.active || row.otherMultibody == entt::null) {
          continue;
        }

        auto* otherPending
            = registry.try_get<PendingMultibodyVelocity>(row.otherMultibody);
        auto* otherScratch
            = registry.try_get<MultibodyDynamicsScratch>(row.otherMultibody);
        if (otherPending == nullptr || !otherPending->active
            || otherScratch == nullptr) {
          continue;
        }

        const auto relativeVelocity
            = [&](const Eigen::VectorXd& primaryJacobian,
                  const Eigen::VectorXd& otherJacobian) {
                return primaryJacobian.dot(pendingVelocity->velocity)
                       - otherJacobian.dot(otherPending->velocity);
              };
        const auto applyImpulse = [&](const Eigen::VectorXd& primaryJacobian,
                                      const Eigen::VectorXd& otherJacobian,
                                      double deltaImpulse) {
          applyMultibodyImpulse(
              pendingVelocity->velocity,
              problem.inverseMass,
              primaryJacobian,
              deltaImpulse,
              scratch.contactImpulseDelta);
          applyMultibodyImpulse(
              otherPending->velocity,
              otherScratch->contactProblem.inverseMass,
              otherJacobian,
              -deltaImpulse,
              otherScratch->contactImpulseDelta);
        };

        const double normalVelocity
            = relativeVelocity(row.normalJacobian, row.otherNormalJacobian);
        const double deltaNormal
            = (-normalVelocity + std::max(row.bias, row.restitutionTarget))
              / row.normalDenominator;
        const double newNormal = std::max(0.0, row.normalImpulse + deltaNormal);
        applyImpulse(
            row.normalJacobian,
            row.otherNormalJacobian,
            newNormal - row.normalImpulse);
        row.normalImpulse = newNormal;

        const double bound = row.friction * row.normalImpulse;
        if (row.tangentDenominator1 > 0.0) {
          const double tangentVelocity = relativeVelocity(
              row.tangentJacobian1, row.otherTangentJacobian1);
          const double newImpulse = std::clamp(
              row.tangentImpulse1 - tangentVelocity / row.tangentDenominator1,
              -bound,
              bound);
          applyImpulse(
              row.tangentJacobian1,
              row.otherTangentJacobian1,
              newImpulse - row.tangentImpulse1);
          row.tangentImpulse1 = newImpulse;
        }
        if (row.tangentDenominator2 > 0.0) {
          const double tangentVelocity = relativeVelocity(
              row.tangentJacobian2, row.otherTangentJacobian2);
          const double newImpulse = std::clamp(
              row.tangentImpulse2 - tangentVelocity / row.tangentDenominator2,
              -bound,
              bound);
          applyImpulse(
              row.tangentJacobian2,
              row.otherTangentJacobian2,
              newImpulse - row.tangentImpulse2);
          row.tangentImpulse2 = newImpulse;
        }
      }
    }
  }
}

//==============================================================================
void enforceMultibodyVelocityLimits(
    detail::WorldRegistry& registry,
    const comps::MultibodyStructure& structure,
    Eigen::VectorXd& nextVelocity)
{
  Eigen::Index expectedDof = 0;
  for (const auto linkEntity : structure.links) {
    const auto& link = registry.get<comps::LinkModel>(linkEntity);
    if (link.parentJoint == entt::null) {
      continue;
    }

    const auto& jointModel = registry.get<comps::JointModel>(link.parentJoint);
    expectedDof += static_cast<Eigen::Index>(jointModel.getDOF());
  }

  DART_SIMULATION_THROW_T_IF(
      nextVelocity.size() != expectedDof,
      InvalidArgumentException,
      "Staged multibody velocity dimension ({}) does not match the expected "
      "DOF count ({})",
      nextVelocity.size(),
      expectedDof);

  // Enforce velocity limits by clamping the generalized velocity.
  Eigen::Index velocityOffset = 0;
  for (const auto linkEntity : structure.links) {
    const auto& link = registry.get<comps::LinkModel>(linkEntity);
    if (link.parentJoint == entt::null) {
      continue;
    }

    const auto& jointModel = registry.get<comps::JointModel>(link.parentJoint);
    const auto dof = static_cast<Eigen::Index>(jointModel.getDOF());
    if (dof == 0) {
      continue;
    }
    if (jointModel.limits.velocityLower.size() != static_cast<Eigen::Index>(dof)
        || jointModel.limits.velocityUpper.size()
               != static_cast<Eigen::Index>(dof)) {
      velocityOffset += dof;
      continue;
    }
    for (Eigen::Index d = 0; d < dof; ++d) {
      const auto globalDof = velocityOffset + d;
      nextVelocity[globalDof] = std::clamp(
          nextVelocity[globalDof],
          jointModel.limits.velocityLower[d],
          jointModel.limits.velocityUpper[d]);
    }
    velocityOffset += dof;
  }
}

//==============================================================================
void projectLockedMultibodyVelocityInto(
    detail::WorldRegistry& registry,
    const comps::MultibodyStructure& structure,
    Eigen::VectorXd& nextVelocity)
{
  Eigen::Index velocityOffset = 0;
  for (const auto linkEntity : structure.links) {
    const auto& link = registry.get<comps::LinkModel>(linkEntity);
    if (link.parentJoint == entt::null) {
      continue;
    }

    const auto& jointModel = registry.get<comps::JointModel>(link.parentJoint);
    const auto dof = static_cast<Eigen::Index>(jointModel.getDOF());
    if (dof == 0) {
      continue;
    }

    const auto& jointActuation
        = registry.get<comps::JointActuation>(link.parentJoint);
    if (jointActuation.actuatorType == comps::ActuatorType::Locked) {
      nextVelocity.segment(velocityOffset, dof).setZero();
    }
    velocityOffset += dof;
  }
}

//==============================================================================
template <typename LinkContactVector>
void collectMultibodyLinkContactsInto(
    detail::WorldRegistry& registry,
    const comps::MultibodyStructure& structure,
    const LinkOwnerMap& linkOwnerMap,
    std::span<const Contact> contacts,
    LinkContactVector& linkContacts)
{
  const auto isRigidBody = [&](entt::entity entity) {
    return registry.all_of<comps::RigidBodyTag>(entity);
  };
  const auto isLink = [&](entt::entity entity) {
    return registry.all_of<comps::LinkModel>(entity);
  };
  const auto isDynamic = [&](entt::entity entity) {
    return !hasPrescribedRigidBodyContactResponse(registry, entity);
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

  linkContacts.clear();
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
      const entt::entity otherMultibody
          = findMultibodyOwningLink(linkOwnerMap, entityB);
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
}

void clearMultibodyExternalForces(
    detail::WorldRegistry& registry, const comps::MultibodyStructure& structure)
{
  for (const auto linkEntity : structure.links) {
    registry.get<comps::LinkControl>(linkEntity).externalForce.setZero();
  }
}

//==============================================================================
void simulateMultibody(
    detail::WorldRegistry& registry,
    const comps::MultibodyStructure& structure,
    const Eigen::Vector3d& gravity,
    double timeStep,
    std::span<const LinkContact> linkContacts,
    MultibodyDynamicsScratch& scratch)
{
  if (!computeUnconstrainedMultibodyVelocityInto(
          registry, structure, gravity, timeStep, scratch)
      || scratch.nextVelocity.size() == 0) {
    return;
  }

  solveMultibodyLinkContacts(
      registry,
      structure,
      scratch.nextVelocity,
      timeStep,
      linkContacts,
      scratch);
  enforceMultibodyVelocityLimits(registry, structure, scratch.nextVelocity);
  projectLockedMultibodyVelocityInto(registry, structure, scratch.nextVelocity);
  integrateMultibodyPositions(
      registry, structure, scratch.nextVelocity, timeStep);
}

//==============================================================================
std::size_t findUnifiedMultibodyContactIndex(
    std::span<const UnifiedMultibodyContact> multibodyContacts,
    entt::entity multibody)
{
  for (std::size_t k = 0; k < multibodyContacts.size(); ++k) {
    if (multibodyContacts[k].multibody == multibody) {
      return k;
    }
  }
  return multibodyContacts.size();
}

//==============================================================================
const MultibodyLinkContactProblem& problemOf(
    const UnifiedMultibodyContact& contact)
{
  return contact.borrowedProblem != nullptr ? *contact.borrowedProblem
                                            : contact.problem;
}

//==============================================================================
MultibodyLinkContactProblem& problemOf(UnifiedMultibodyContact& contact)
{
  return contact.borrowedProblem != nullptr ? *contact.borrowedProblem
                                            : contact.problem;
}

//==============================================================================
std::size_t findScratchLinkIndex(
    const MultibodyDynamicsScratch& scratch, entt::entity link)
{
  const auto it = std::find_if(
      scratch.linkIndexOf.begin(),
      scratch.linkIndexOf.end(),
      [link](const auto& entry) { return entry.first == link; });
  if (it == scratch.linkIndexOf.end()) {
    return scratch.tree.links.size();
  }
  return it->second;
}

//==============================================================================
void completeCrossMultibodyLinkRows(
    std::span<UnifiedMultibodyContact> multibodyContacts,
    std::span<MultibodyDynamicsScratch*> multibodyScratch,
    std::span<const Eigen::VectorXd> multibodyVelocities)
{
  constexpr double restitutionThreshold = 1e-2;
  for (std::size_t k = 0; k < multibodyContacts.size(); ++k) {
    auto& rows = problemOf(multibodyContacts[k]).rows;
    const Eigen::VectorXd& primaryVelocity = multibodyVelocities[k];
    for (auto& row : rows) {
      if (row.otherMultibody == entt::null) {
        continue;
      }

      const std::size_t otherIndex = findUnifiedMultibodyContactIndex(
          multibodyContacts, row.otherMultibody);
      if (otherIndex >= multibodyContacts.size()
          || otherIndex >= multibodyScratch.size()
          || multibodyScratch[otherIndex] == nullptr) {
        continue;
      }
      auto& otherScratch = *multibodyScratch[otherIndex];
      const auto otherLinkIndex
          = findScratchLinkIndex(otherScratch, row.otherLink);
      if (otherLinkIndex >= otherScratch.tree.links.size()) {
        continue;
      }

      pointLinearJacobianInto(
          otherScratch.tree,
          otherScratch.bodyJacobian,
          otherLinkIndex,
          row.point,
          otherScratch.otherPointJacobian);
      multiplyPointJacobianTransposeInto(
          otherScratch.otherPointJacobian, row.normal, row.otherNormalJacobian);
      multiplyPointJacobianTransposeInto(
          otherScratch.otherPointJacobian,
          row.tangent1,
          row.otherTangentJacobian1);
      multiplyPointJacobianTransposeInto(
          otherScratch.otherPointJacobian,
          row.tangent2,
          row.otherTangentJacobian2);

      const Eigen::MatrixXd& otherInverseMass
          = problemOf(multibodyContacts[otherIndex]).inverseMass;
      row.normalDenominator += jointSpaceDenominator(
          otherInverseMass, row.otherNormalJacobian, otherScratch.contactWork);
      row.tangentDenominator1 += jointSpaceDenominator(
          otherInverseMass,
          row.otherTangentJacobian1,
          otherScratch.contactWork);
      row.tangentDenominator2 += jointSpaceDenominator(
          otherInverseMass,
          row.otherTangentJacobian2,
          otherScratch.contactWork);

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
    const detail::WorldRegistry& registry, const DynamicsTree& tree)
{
  for (std::size_t i = 0; i < tree.links.size(); ++i) {
    if (tree.links[i].dof == 0) {
      continue;
    }
    if (tree.links[i].dof != 1) {
      return false;
    }
    const auto& jointModel = registry.get<comps::JointModel>(tree.jointOf[i]);
    if (jointModel.type != comps::JointType::Revolute
        && jointModel.type != comps::JointType::Prismatic
        && jointModel.type != comps::JointType::Screw) {
      return false;
    }
  }
  return true;
}

//==============================================================================
using SpatialVectorScratch = MultibodyDynamicsScratch::SpatialVectorVector;

void resetSpatialVectorScratch(SpatialVectorScratch& scratch, std::size_t count)
{
  scratch.resize(count);
  const Vector6 zero = Vector6::Zero();
  std::fill(scratch.begin(), scratch.end(), zero);
}

//==============================================================================
// Analytic ∂τ/∂q and ∂τ/∂q̇ of inverse dynamics τ = ID(q, q̇, qddot) via
// Recursive-Newton-Euler derivative recursions, for trees of constant
// unit-twist joints (see treeSupportsAnalyticDerivatives). [angular; linear]
// spatial convention; crm = motionCross, crf = forceCross; ∂X_i/∂q_i =
// -crm(S_i) X_i. O(dof²): a down/up sweep per source coordinate.
void rneaDerivativesInto(
    std::span<const LinkDynamics> links,
    const Vector6& baseAcceleration,
    std::size_t dofCount,
    const Eigen::VectorXd& qddot,
    const Eigen::VectorXd& qdot,
    SpatialVectorScratch& velocity,
    SpatialVectorScratch& acceleration,
    SpatialVectorScratch& force,
    SpatialVectorScratch& jointVelocity,
    SpatialVectorScratch& totalForce,
    SpatialVectorScratch& deltaVelocity,
    SpatialVectorScratch& deltaAcceleration,
    SpatialVectorScratch& deltaForce,
    SpatialVectorScratch& accumulatedForce,
    InverseDynamicsDerivatives& out)
{
  const auto count = links.size();
  const auto n = static_cast<Eigen::Index>(dofCount);

  // Forward pass: per-link spatial velocity v, acceleration a, force f, and the
  // joint velocity contribution jv = S_i q̇_i.
  resetSpatialVectorScratch(velocity, count);
  resetSpatialVectorScratch(acceleration, count);
  resetSpatialVectorScratch(force, count);
  resetSpatialVectorScratch(jointVelocity, count);
  for (std::size_t i = 0; i < count; ++i) {
    const auto& link = links[i];
    if (link.parentIndex < 0) {
      acceleration[i] = link.parentToChild * baseAcceleration;
    } else {
      const auto p = static_cast<std::size_t>(link.parentIndex);
      if (link.dof > 0) {
        jointVelocity[i]
            = link.subspace * qdot.segment(link.dofOffset, link.dof);
        acceleration[i]
            = link.subspace * qddot.segment(link.dofOffset, link.dof);
      }
      velocity[i] = link.parentToChild * velocity[p] + jointVelocity[i];
      acceleration[i] += link.parentToChild * acceleration[p]
                         + crossMotion(velocity[i], jointVelocity[i]);
    }
    force[i] = link.inertia * acceleration[i]
               + crossForce(velocity[i], link.inertia * velocity[i]);
  }

  // Total accumulated force F_i = f_i + Σ_children X_c^T F_c (needed by the
  // δ(X^T) term in the position-derivative backward sweep).
  totalForce.assign(force.begin(), force.end());
  for (std::size_t r = 0; r < count; ++r) {
    const auto i = count - 1 - r;
    if (links[i].parentIndex >= 0) {
      totalForce[static_cast<std::size_t>(links[i].parentIndex)]
          += links[i].childToParentForce * totalForce[i];
    }
  }

  out.dTau_dq.resize(n, n);
  out.dTau_dq.setZero();
  out.dTau_dqdot.resize(n, n);
  out.dTau_dqdot.setZero();
  out.valid = true;

  for (std::size_t m = 0; m < count; ++m) {
    if (links[m].dof == 0) {
      continue;
    }
    const auto j = static_cast<Eigen::Index>(links[m].dofOffset);
    const Vector6 subspaceM = links[m].subspace.col(0);

    // ---------- ∂/∂q_j ----------
    {
      resetSpatialVectorScratch(deltaVelocity, count);
      resetSpatialVectorScratch(deltaAcceleration, count);
      resetSpatialVectorScratch(deltaForce, count);
      for (std::size_t i = 0; i < count; ++i) {
        const auto& link = links[i];
        if (link.parentIndex >= 0) {
          const auto p = static_cast<std::size_t>(link.parentIndex);
          deltaVelocity[i] = link.parentToChild * deltaVelocity[p];
          deltaAcceleration[i] = link.parentToChild * deltaAcceleration[p];
          if (i == m) {
            // ∂X_m/∂q_j = -crm(S_m) X_m acting on the parent motion/accel.
            deltaVelocity[i]
                += -crossMotion(subspaceM, link.parentToChild * velocity[p]);
            deltaAcceleration[i] += -crossMotion(
                subspaceM, link.parentToChild * acceleration[p]);
          }
          deltaAcceleration[i]
              += crossMotion(deltaVelocity[i], jointVelocity[i]);
        }
        deltaForce[i]
            = link.inertia * deltaAcceleration[i]
              + crossForce(deltaVelocity[i], link.inertia * velocity[i])
              + crossForce(velocity[i], link.inertia * deltaVelocity[i]);
      }
      accumulatedForce.assign(deltaForce.begin(), deltaForce.end());
      for (std::size_t r = 0; r < count; ++r) {
        const auto i = count - 1 - r;
        const auto& link = links[i];
        if (link.dof > 0) {
          out.dTau_dq(static_cast<Eigen::Index>(link.dofOffset), j)
              = link.subspace.col(0).dot(accumulatedForce[i]);
        }
        if (link.parentIndex >= 0) {
          const auto p = static_cast<std::size_t>(link.parentIndex);
          accumulatedForce[p] += link.childToParentForce * accumulatedForce[i];
          if (i == m) {
            // δ(X_m^T) F_m = X_m^T crf(S_m) F_m.
            accumulatedForce[p] += link.childToParentForce
                                   * crossForce(subspaceM, totalForce[i]);
          }
        }
      }
    }

    // ---------- ∂/∂q̇_j ----------
    {
      resetSpatialVectorScratch(deltaVelocity, count);
      resetSpatialVectorScratch(deltaAcceleration, count);
      resetSpatialVectorScratch(deltaForce, count);
      for (std::size_t i = 0; i < count; ++i) {
        const auto& link = links[i];
        if (link.parentIndex >= 0) {
          const auto p = static_cast<std::size_t>(link.parentIndex);
          deltaVelocity[i] = link.parentToChild * deltaVelocity[p];
          deltaAcceleration[i] = link.parentToChild * deltaAcceleration[p];
          if (i == m) {
            deltaVelocity[i] += subspaceM;
            deltaAcceleration[i] += crossMotion(velocity[i], subspaceM);
          }
          deltaAcceleration[i]
              += crossMotion(deltaVelocity[i], jointVelocity[i]);
        }
        deltaForce[i]
            = link.inertia * deltaAcceleration[i]
              + crossForce(deltaVelocity[i], link.inertia * velocity[i])
              + crossForce(velocity[i], link.inertia * deltaVelocity[i]);
      }
      accumulatedForce.assign(deltaForce.begin(), deltaForce.end());
      for (std::size_t r = 0; r < count; ++r) {
        const auto i = count - 1 - r;
        const auto& link = links[i];
        if (link.dof > 0) {
          out.dTau_dqdot(static_cast<Eigen::Index>(link.dofOffset), j)
              = link.subspace.col(0).dot(accumulatedForce[i]);
        }
        if (link.parentIndex >= 0) {
          accumulatedForce[static_cast<std::size_t>(link.parentIndex)]
              += link.childToParentForce * accumulatedForce[i];
        }
      }
    }
  }
}

} // namespace

//==============================================================================
struct MultibodyLinkContactAssemblyScratch::Impl
{
  MultibodyDynamicsScratch scratch;
};

//==============================================================================
struct MultibodyDynamicsTermsScratch::Impl
{
  explicit Impl(common::MemoryAllocator& allocator) : scratch(allocator) {}

  MultibodyDynamicsScratch scratch;
};

//==============================================================================
struct MultibodyLinkJacobianScratch::Impl
{
  explicit Impl(common::MemoryAllocator& allocator) : scratch(allocator) {}

  MultibodyDynamicsScratch scratch;
};

//==============================================================================
struct MultibodyInverseDynamicsScratch::Impl
{
  explicit Impl(common::MemoryAllocator& allocator) : scratch(allocator) {}

  MultibodyDynamicsScratch scratch;
};

//==============================================================================
MultibodyDynamicsTermsScratch::MultibodyDynamicsTermsScratch()
  : MultibodyDynamicsTermsScratch(common::MemoryAllocator::GetDefault())
{
}

//==============================================================================
MultibodyDynamicsTermsScratch::MultibodyDynamicsTermsScratch(
    common::MemoryAllocator& allocator)
  : m_allocator(&allocator), m_impl(nullptr, ImplDeleter{&allocator})
{
}

//==============================================================================
MultibodyDynamicsTermsScratch::~MultibodyDynamicsTermsScratch() = default;

//==============================================================================
MultibodyDynamicsTermsScratch::MultibodyDynamicsTermsScratch(
    MultibodyDynamicsTermsScratch&&) noexcept = default;

//==============================================================================
MultibodyDynamicsTermsScratch& MultibodyDynamicsTermsScratch::operator=(
    MultibodyDynamicsTermsScratch&&) noexcept = default;

//==============================================================================
void MultibodyDynamicsTermsScratch::ImplDeleter::operator()(
    Impl* impl) const noexcept
{
  if (impl == nullptr) {
    return;
  }
  auto& targetAllocator = allocator != nullptr
                              ? *allocator
                              : common::MemoryAllocator::GetDefault();
  targetAllocator.destroy(impl);
}

//==============================================================================
void MultibodyDynamicsTermsScratch::setAllocator(
    common::MemoryAllocator& allocator)
{
  if (m_allocator == &allocator) {
    return;
  }
  m_impl.reset();
  m_allocator = &allocator;
  m_impl.get_deleter().allocator = &allocator;
}

//==============================================================================
const common::MemoryAllocator& MultibodyDynamicsTermsScratch::getAllocator()
    const noexcept
{
  return m_allocator != nullptr ? *m_allocator
                                : common::MemoryAllocator::GetDefault();
}

//==============================================================================
MultibodyLinkJacobianScratch::MultibodyLinkJacobianScratch()
  : MultibodyLinkJacobianScratch(common::MemoryAllocator::GetDefault())
{
}

//==============================================================================
MultibodyLinkJacobianScratch::MultibodyLinkJacobianScratch(
    common::MemoryAllocator& allocator)
  : m_allocator(&allocator), m_impl(nullptr, ImplDeleter{&allocator})
{
}

//==============================================================================
MultibodyLinkJacobianScratch::~MultibodyLinkJacobianScratch() = default;

//==============================================================================
MultibodyLinkJacobianScratch::MultibodyLinkJacobianScratch(
    MultibodyLinkJacobianScratch&&) noexcept = default;

//==============================================================================
MultibodyLinkJacobianScratch& MultibodyLinkJacobianScratch::operator=(
    MultibodyLinkJacobianScratch&&) noexcept = default;

//==============================================================================
void MultibodyLinkJacobianScratch::ImplDeleter::operator()(
    Impl* impl) const noexcept
{
  if (impl == nullptr) {
    return;
  }
  auto& targetAllocator = allocator != nullptr
                              ? *allocator
                              : common::MemoryAllocator::GetDefault();
  targetAllocator.destroy(impl);
}

//==============================================================================
void MultibodyLinkJacobianScratch::setAllocator(
    common::MemoryAllocator& allocator)
{
  if (m_allocator == &allocator) {
    return;
  }
  m_impl.reset();
  m_allocator = &allocator;
  m_impl.get_deleter().allocator = &allocator;
}

//==============================================================================
const common::MemoryAllocator& MultibodyLinkJacobianScratch::getAllocator()
    const noexcept
{
  return m_allocator != nullptr ? *m_allocator
                                : common::MemoryAllocator::GetDefault();
}

//==============================================================================
MultibodyInverseDynamicsScratch::MultibodyInverseDynamicsScratch()
  : MultibodyInverseDynamicsScratch(common::MemoryAllocator::GetDefault())
{
}

//==============================================================================
MultibodyInverseDynamicsScratch::MultibodyInverseDynamicsScratch(
    common::MemoryAllocator& allocator)
  : m_allocator(&allocator), m_impl(nullptr, ImplDeleter{&allocator})
{
}

//==============================================================================
MultibodyInverseDynamicsScratch::~MultibodyInverseDynamicsScratch() = default;

//==============================================================================
MultibodyInverseDynamicsScratch::MultibodyInverseDynamicsScratch(
    MultibodyInverseDynamicsScratch&&) noexcept = default;

//==============================================================================
MultibodyInverseDynamicsScratch& MultibodyInverseDynamicsScratch::operator=(
    MultibodyInverseDynamicsScratch&&) noexcept = default;

//==============================================================================
void MultibodyInverseDynamicsScratch::ImplDeleter::operator()(
    Impl* impl) const noexcept
{
  if (impl == nullptr) {
    return;
  }
  auto& targetAllocator = allocator != nullptr
                              ? *allocator
                              : common::MemoryAllocator::GetDefault();
  targetAllocator.destroy(impl);
}

//==============================================================================
void MultibodyInverseDynamicsScratch::setAllocator(
    common::MemoryAllocator& allocator)
{
  if (m_allocator == &allocator) {
    return;
  }
  m_impl.reset();
  m_allocator = &allocator;
  m_impl.get_deleter().allocator = &allocator;
}

//==============================================================================
const common::MemoryAllocator& MultibodyInverseDynamicsScratch::getAllocator()
    const noexcept
{
  return m_allocator != nullptr ? *m_allocator
                                : common::MemoryAllocator::GetDefault();
}

//==============================================================================
void reserveMultibodyInverseDynamicsScratch(
    MultibodyInverseDynamicsScratch& scratch,
    detail::WorldRegistry& registry,
    const comps::MultibodyStructure& structure)
{
  if (scratch.m_impl == nullptr) {
    auto& allocator = scratch.m_allocator != nullptr
                          ? *scratch.m_allocator
                          : common::MemoryAllocator::GetDefault();
    auto* impl
        = allocator.construct<MultibodyInverseDynamicsScratch::Impl>(allocator);
    if (impl == nullptr) {
      throw std::bad_alloc();
    }
    scratch.m_impl.reset(impl);
  }

  auto& storage = scratch.m_impl->scratch;
  buildDynamicsTreeInto(
      registry,
      structure,
      storage.tree,
      storage.linkIndexOf,
      storage.jointFrameSubspace);

  const auto linkCount = storage.tree.links.size();
  const auto dof = static_cast<Eigen::Index>(storage.tree.dofCount);
  storage.qdot.resize(dof);
  storage.rneaResponse.resize(dof);
  storage.rneaVelocity.resize(linkCount);
  storage.rneaAcceleration.resize(linkCount);
  storage.rneaForce.resize(linkCount);
}

//==============================================================================
void computeMultibodyInverseDynamicsInto(
    MultibodyInverseDynamicsScratch& scratch,
    detail::WorldRegistry& registry,
    const comps::MultibodyStructure& structure,
    const Eigen::Vector3d& gravity,
    const Eigen::VectorXd& desiredAcceleration,
    Eigen::VectorXd& result)
{
  if (scratch.m_impl == nullptr) {
    auto& allocator = scratch.m_allocator != nullptr
                          ? *scratch.m_allocator
                          : common::MemoryAllocator::GetDefault();
    auto* impl
        = allocator.construct<MultibodyInverseDynamicsScratch::Impl>(allocator);
    if (impl == nullptr) {
      throw std::bad_alloc();
    }
    scratch.m_impl.reset(impl);
  }

  if (structure.links.empty()) {
    result.resize(0);
    return;
  }

  auto& storage = scratch.m_impl->scratch;
  buildDynamicsTreeInto(
      registry,
      structure,
      storage.tree,
      storage.linkIndexOf,
      storage.jointFrameSubspace);
  if (storage.tree.dofCount == 0) {
    result.resize(0);
    return;
  }

  DART_SIMULATION_THROW_T_IF(
      desiredAcceleration.size()
          != static_cast<Eigen::Index>(storage.tree.dofCount),
      InvalidArgumentException,
      "Desired acceleration dimension ({}) must match the multibody DOF count "
      "({})",
      desiredAcceleration.size(),
      storage.tree.dofCount);

  gatherMultibodyVelocityInto(registry, storage.tree, storage.qdot);

  Vector6 baseAcceleration = Vector6::Zero();
  baseAcceleration.tail<3>() = -gravity;
  recursiveNewtonEulerInto(
      linkSpan(storage.tree),
      baseAcceleration,
      storage.tree.dofCount,
      desiredAcceleration,
      storage.qdot,
      result,
      storage.rneaVelocity,
      storage.rneaAcceleration,
      storage.rneaForce);

  // Armature contributes diag(armature) * qddot to the joint forces.
  if (storage.tree.armature.size() == desiredAcceleration.size()) {
    result += storage.tree.armature.cwiseProduct(desiredAcceleration);
  }
}

//==============================================================================
MultibodyLinkContactAssemblyScratch::MultibodyLinkContactAssemblyScratch()
  : m_impl(std::make_unique<Impl>())
{
}

//==============================================================================
MultibodyLinkContactAssemblyScratch::~MultibodyLinkContactAssemblyScratch()
    = default;

//==============================================================================
MultibodyLinkContactAssemblyScratch::MultibodyLinkContactAssemblyScratch(
    MultibodyLinkContactAssemblyScratch&&) noexcept = default;

//==============================================================================
MultibodyLinkContactAssemblyScratch&
MultibodyLinkContactAssemblyScratch::operator=(
    MultibodyLinkContactAssemblyScratch&&) noexcept = default;

//==============================================================================
MultibodyLinkContactProblem&
MultibodyLinkContactAssemblyScratch::getProblem() noexcept
{
  return m_impl->scratch.contactProblem;
}

//==============================================================================
const MultibodyLinkContactProblem&
MultibodyLinkContactAssemblyScratch::getProblem() const noexcept
{
  return m_impl->scratch.contactProblem;
}

//==============================================================================
MultibodyLinkContactProblem assembleMultibodyLinkContactProblem(
    const detail::WorldRegistry& registry,
    const comps::MultibodyStructure& structure,
    const Eigen::VectorXd& nextVelocity,
    double timeStep,
    std::span<const LinkContact> linkContacts)
{
  MultibodyDynamicsScratch scratch;
  assembleMultibodyLinkContactProblemInto(
      registry, structure, nextVelocity, timeStep, linkContacts, scratch);
  return std::move(scratch.contactProblem);
}

//==============================================================================
bool assembleMultibodyLinkContactProblemInto(
    MultibodyLinkContactAssemblyScratch& scratch,
    const detail::WorldRegistry& registry,
    const comps::MultibodyStructure& structure,
    const Eigen::VectorXd& nextVelocity,
    double timeStep,
    std::span<const LinkContact> linkContacts)
{
  if (scratch.m_impl == nullptr) {
    scratch.m_impl
        = std::make_unique<MultibodyLinkContactAssemblyScratch::Impl>();
  }
  return assembleMultibodyLinkContactProblemInto(
      registry,
      structure,
      nextVelocity,
      timeStep,
      linkContacts,
      scratch.m_impl->scratch);
}

//==============================================================================
void reserveMultibodyDynamicsTermsScratch(
    MultibodyDynamicsTermsScratch& scratch,
    detail::WorldRegistry& registry,
    const comps::MultibodyStructure& structure)
{
  if (scratch.m_impl == nullptr) {
    auto& allocator = scratch.m_allocator != nullptr
                          ? *scratch.m_allocator
                          : common::MemoryAllocator::GetDefault();
    auto* impl
        = allocator.construct<MultibodyDynamicsTermsScratch::Impl>(allocator);
    if (impl == nullptr) {
      throw std::bad_alloc();
    }
    scratch.m_impl.reset(impl);
  }

  auto& storage = scratch.m_impl->scratch;
  buildDynamicsTreeInto(
      registry,
      structure,
      storage.tree,
      storage.linkIndexOf,
      storage.jointFrameSubspace);

  const auto linkCount = storage.tree.links.size();
  const auto dof = static_cast<Eigen::Index>(storage.tree.dofCount);
  storage.qdot.resize(dof);
  storage.zero.resize(dof);
  storage.unitAcceleration.resize(dof);
  storage.rneaResponse.resize(dof);
  storage.massAndBias.massMatrix.resize(dof, dof);
  storage.massAndBias.bias.resize(dof);
  storage.massAndBias.gravityOnly.resize(dof);
  storage.rneaVelocity.resize(linkCount);
  storage.rneaAcceleration.resize(linkCount);
  storage.rneaForce.resize(linkCount);
}

//==============================================================================
void computeMultibodyDynamicsTermsInto(
    MultibodyDynamicsTermsScratch& scratch,
    detail::WorldRegistry& registry,
    const comps::MultibodyStructure& structure,
    const Eigen::Vector3d& gravity,
    MultibodyDynamicsTerms& result)
{
  if (structure.links.empty()) {
    result.massMatrix.resize(0, 0);
    result.coriolisForces.resize(0);
    result.gravityForces.resize(0);
    return;
  }

  reserveMultibodyDynamicsTermsScratch(scratch, registry, structure);

  auto& storage = scratch.m_impl->scratch;
  if (storage.tree.dofCount == 0) {
    result.massMatrix.resize(0, 0);
    result.coriolisForces.resize(0);
    result.gravityForces.resize(0);
    return;
  }

  gatherMultibodyVelocityInto(registry, storage.tree, storage.qdot);
  computeMassAndBiasInto(
      linkSpan(storage.tree),
      storage.tree.dofCount,
      gravity,
      storage.qdot,
      storage.tree.armature,
      storage.massAndBias,
      storage.zero,
      storage.unitAcceleration,
      storage.rneaResponse,
      storage.rneaVelocity,
      storage.rneaAcceleration,
      storage.rneaForce);

  const auto dof = static_cast<Eigen::Index>(storage.tree.dofCount);
  result.massMatrix.resize(dof, dof);
  result.massMatrix = storage.massAndBias.massMatrix;
  result.gravityForces.resize(dof);
  result.gravityForces = storage.massAndBias.gravityOnly;
  result.coriolisForces.resize(dof);
  result.coriolisForces = storage.massAndBias.bias;
  result.coriolisForces -= storage.massAndBias.gravityOnly;
}

//==============================================================================
MultibodyDynamicsTerms computeMultibodyDynamicsTerms(
    detail::WorldRegistry& registry,
    const comps::MultibodyStructure& structure,
    const Eigen::Vector3d& gravity)
{
  MultibodyDynamicsTermsScratch scratch;
  MultibodyDynamicsTerms terms;
  computeMultibodyDynamicsTermsInto(
      scratch, registry, structure, gravity, terms);
  return terms;
}

//==============================================================================
Eigen::VectorXd computeMultibodyInverseDynamics(
    detail::WorldRegistry& registry,
    const comps::MultibodyStructure& structure,
    const Eigen::Vector3d& gravity,
    const Eigen::VectorXd& desiredAcceleration)
{
  MultibodyInverseDynamicsScratch scratch;
  Eigen::VectorXd result;
  computeMultibodyInverseDynamicsInto(
      scratch, registry, structure, gravity, desiredAcceleration, result);
  return result;
}

//==============================================================================
InverseDynamicsDerivatives computeMultibodyInverseDynamicsDerivatives(
    detail::WorldRegistry& registry,
    const comps::MultibodyStructure& structure,
    const Eigen::Vector3d& gravity,
    const Eigen::VectorXd& generalizedAcceleration)
{
  MultibodyInverseDynamicsScratch scratch;
  InverseDynamicsDerivatives result;
  computeMultibodyInverseDynamicsDerivativesInto(
      scratch, registry, structure, gravity, generalizedAcceleration, result);
  return result;
}

//==============================================================================
void computeMultibodyInverseDynamicsDerivativesInto(
    MultibodyInverseDynamicsScratch& scratch,
    detail::WorldRegistry& registry,
    const comps::MultibodyStructure& structure,
    const Eigen::Vector3d& gravity,
    const Eigen::VectorXd& generalizedAcceleration,
    InverseDynamicsDerivatives& result)
{
  result.valid = false;
  if (structure.links.empty()) {
    result.dTau_dq.resize(0, 0);
    result.dTau_dqdot.resize(0, 0);
    return;
  }

  if (scratch.m_impl == nullptr) {
    auto& allocator = scratch.m_allocator != nullptr
                          ? *scratch.m_allocator
                          : common::MemoryAllocator::GetDefault();
    auto* impl
        = allocator.construct<MultibodyInverseDynamicsScratch::Impl>(allocator);
    if (impl == nullptr) {
      throw std::bad_alloc();
    }
    scratch.m_impl.reset(impl);
  }

  auto& storage = scratch.m_impl->scratch;
  buildDynamicsTreeInto(
      registry,
      structure,
      storage.tree,
      storage.linkIndexOf,
      storage.jointFrameSubspace);
  if (storage.tree.dofCount == 0
      || generalizedAcceleration.size()
             != static_cast<Eigen::Index>(storage.tree.dofCount)
      || !treeSupportsAnalyticDerivatives(registry, storage.tree)) {
    result.dTau_dq.resize(0, 0);
    result.dTau_dqdot.resize(0, 0);
    return; // valid == false: caller falls back to finite differencing
  }

  gatherMultibodyVelocityInto(registry, storage.tree, storage.qdot);

  Vector6 baseAcceleration = Vector6::Zero();
  baseAcceleration.tail<3>() = -gravity;

  rneaDerivativesInto(
      linkSpan(storage.tree),
      baseAcceleration,
      storage.tree.dofCount,
      generalizedAcceleration,
      storage.qdot,
      storage.rneaDerivativeVelocity,
      storage.rneaDerivativeAcceleration,
      storage.rneaDerivativeForce,
      storage.rneaDerivativeJointVelocity,
      storage.rneaDerivativeTotalForce,
      storage.rneaDerivativeDeltaVelocity,
      storage.rneaDerivativeDeltaAcceleration,
      storage.rneaDerivativeDeltaForce,
      storage.rneaDerivativeAccumulatedForce,
      result);
}

//==============================================================================
Eigen::MatrixXd computeMultibodyLinkJacobian(
    detail::WorldRegistry& registry,
    const comps::MultibodyStructure& structure,
    entt::entity linkEntity)
{
  MultibodyLinkJacobianScratch scratch;
  Eigen::MatrixXd result;
  computeMultibodyLinkJacobianInto(
      scratch, registry, structure, linkEntity, result);
  return result;
}

//==============================================================================
Eigen::MatrixXd computeMultibodyLinkWorldJacobian(
    detail::WorldRegistry& registry,
    const comps::MultibodyStructure& structure,
    entt::entity linkEntity)
{
  MultibodyLinkJacobianScratch scratch;
  Eigen::MatrixXd result;
  computeMultibodyLinkWorldJacobianInto(
      scratch, registry, structure, linkEntity, result);
  return result;
}

//==============================================================================
void reserveMultibodyLinkJacobianScratch(
    MultibodyLinkJacobianScratch& scratch,
    detail::WorldRegistry& registry,
    const comps::MultibodyStructure& structure)
{
  if (scratch.m_impl == nullptr) {
    auto& allocator = scratch.m_allocator != nullptr
                          ? *scratch.m_allocator
                          : common::MemoryAllocator::GetDefault();
    auto* impl
        = allocator.construct<MultibodyLinkJacobianScratch::Impl>(allocator);
    if (impl == nullptr) {
      throw std::bad_alloc();
    }
    scratch.m_impl.reset(impl);
  }

  auto& storage = scratch.m_impl->scratch;
  buildDynamicsTreeInto(
      registry,
      structure,
      storage.tree,
      storage.linkIndexOf,
      storage.jointFrameSubspace);
  linkBodyJacobiansInto(storage.tree, storage.bodyJacobian);
}

//==============================================================================
void computeMultibodyLinkJacobianInto(
    MultibodyLinkJacobianScratch& scratch,
    detail::WorldRegistry& registry,
    const comps::MultibodyStructure& structure,
    entt::entity linkEntity,
    Eigen::MatrixXd& result)
{
  const auto targetIndex = linkIndexOf(structure, linkEntity);
  reserveMultibodyLinkJacobianScratch(scratch, registry, structure);

  const auto& bodyJacobian = scratch.m_impl->scratch.bodyJacobian[targetIndex];
  result.resize(bodyJacobian.rows(), bodyJacobian.cols());
  result = bodyJacobian;
}

//==============================================================================
void computeMultibodyLinkWorldJacobianInto(
    MultibodyLinkJacobianScratch& scratch,
    detail::WorldRegistry& registry,
    const comps::MultibodyStructure& structure,
    entt::entity linkEntity,
    Eigen::MatrixXd& result)
{
  const auto targetIndex = linkIndexOf(structure, linkEntity);
  reserveMultibodyLinkJacobianScratch(scratch, registry, structure);

  const auto& storage = scratch.m_impl->scratch;
  const auto& bodyJacobian = storage.bodyJacobian[targetIndex];

  // Rotate both the angular and linear blocks into world axes, keeping the link
  // origin as the reference point: this is the geometric (world-frame) Jacobian
  // [angular; linear of the link origin].
  const Eigen::Matrix3d rotation
      = storage.tree.links[targetIndex].worldTransform.linear();
  result.resize(6, bodyJacobian.cols());
  result.topRows<3>().noalias() = rotation * bodyJacobian.topRows<3>();
  result.bottomRows<3>().noalias() = rotation * bodyJacobian.bottomRows<3>();
}

//==============================================================================
Eigen::Vector3d computeMultibodyCenterOfMass(
    detail::WorldRegistry& registry, const comps::MultibodyStructure& structure)
{
  MultibodyDynamicsScratch scratch;
  buildDynamicsTreeInto(
      registry,
      structure,
      scratch.tree,
      scratch.linkIndexOf,
      scratch.jointFrameSubspace);

  Eigen::Vector3d weighted = Eigen::Vector3d::Zero();
  double totalMass = 0.0;
  for (std::size_t i = 0; i < structure.links.size(); ++i) {
    const auto& mass = registry.get<comps::LinkModel>(structure.links[i]).mass;
    if (mass.mass <= 0.0) {
      continue;
    }
    weighted.noalias()
        += mass.mass
           * (scratch.tree.links[i].worldTransform * mass.localCenterOfMass);
    totalMass += mass.mass;
  }
  if (totalMass <= 0.0) {
    return Eigen::Vector3d::Zero();
  }
  return weighted / totalMass;
}

//==============================================================================
void computeMultibodyCenterOfMassJacobianInto(
    MultibodyLinkJacobianScratch& scratch,
    detail::WorldRegistry& registry,
    const comps::MultibodyStructure& structure,
    Eigen::MatrixXd& result)
{
  reserveMultibodyLinkJacobianScratch(scratch, registry, structure);
  const auto& storage = scratch.m_impl->scratch;
  const auto dof = static_cast<Eigen::Index>(storage.tree.dofCount);
  result.setZero(3, dof);

  double totalMass = 0.0;
  for (std::size_t i = 0; i < structure.links.size(); ++i) {
    const auto& mass = registry.get<comps::LinkModel>(structure.links[i]).mass;
    if (mass.mass <= 0.0) {
      continue;
    }
    // The body Jacobian rows are [angular; linear] in the link frame; rotate
    // both blocks into world axes (matching computeMultibodyLinkWorldJacobian).
    const auto& bodyJacobian = storage.bodyJacobian[i];
    const Eigen::Matrix3d rotation
        = storage.tree.links[i].worldTransform.linear();
    const Eigen::Matrix3Xd angular = rotation * bodyJacobian.topRows<3>();
    const Eigen::Matrix3Xd originLinear
        = rotation * bodyJacobian.bottomRows<3>();
    // Shift the linear reference point from the link origin to the link's
    // center of mass: v_com = v_origin + omega x (R * c) = originLinear -
    // skew(Rc) * w.
    const Eigen::Vector3d comOffset = rotation * mass.localCenterOfMass;
    result.noalias() += mass.mass * (originLinear - skew(comOffset) * angular);
    totalMass += mass.mass;
  }
  if (totalMass > 0.0) {
    result /= totalMass;
  }
}

//==============================================================================
Eigen::MatrixXd computeMultibodyCenterOfMassJacobian(
    detail::WorldRegistry& registry, const comps::MultibodyStructure& structure)
{
  MultibodyLinkJacobianScratch scratch;
  Eigen::MatrixXd result;
  computeMultibodyCenterOfMassJacobianInto(
      scratch, registry, structure, result);
  return result;
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
  auto& registry = dart::simulation::detail::registryOf(world);
  const Eigen::Vector3d gravity = world.getGravity();
  const double timeStep = world.getTimeStep();
  auto view = registry.view<comps::MultibodyStructure>();
  if (view.begin() == view.end()) {
    return;
  }

  // Collision query once per step; route each contact that touches a link to
  // the multibody that owns it. Link-vs-rigid-body contacts are resolved here:
  // a static obstacle is one-sided, a dynamic rigid body receives the
  // equal-and-opposite impulse (two-sided). Same-multibody link-vs-link
  // contacts use one relative-Jacobian row; cross-multibody link-vs-link
  // contacts remain a later unified-solve slice.
  const bool hasContactShapes
      = hasAnyCollisionShapesForMultibodyContacts(registry, view);
  std::span<const Contact> contacts;
  const LinkOwnerMap* linkOwnerMap = nullptr;
  if (hasContactShapes) {
    contacts = world.queryContacts(CollisionQueryOptions{});
    for (auto entity : view) {
      auto& scratch
          = getOrEmplaceMultibodyDynamicsScratch(world, registry, entity);
      ensureLinkOwnerMapInto(view, scratch.linkOwnerMap);
      linkOwnerMap = &scratch.linkOwnerMap;
      break;
    }
  }

  for (auto entity : view) {
    const auto& structure = view.get<comps::MultibodyStructure>(entity);
    auto& scratch
        = getOrEmplaceMultibodyDynamicsScratch(world, registry, entity);
    if (linkOwnerMap != nullptr) {
      collectMultibodyLinkContactsInto(
          registry, structure, *linkOwnerMap, contacts, scratch.linkContacts);
    } else {
      scratch.linkContacts.clear();
    }

    simulateMultibody(
        registry, structure, gravity, timeStep, scratch.linkContacts, scratch);
    if (auto* pendingVelocity
        = registry.try_get<PendingMultibodyVelocity>(entity)) {
      pendingVelocity->active = false;
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
struct MultibodyVelocityStage::Scratch
{
  using EntityAllocator = common::StlAllocator<entt::entity>;

  struct WorkItem
  {
    const comps::MultibodyStructure* structure = nullptr;
    MultibodyDynamicsScratch* scratch = nullptr;
    PendingMultibodyVelocity* pendingVelocity = nullptr;
  };

  using WorkItemAllocator = common::StlAllocator<WorkItem>;

  Scratch() = default;

  explicit Scratch(common::MemoryAllocator& allocator)
    : entities(EntityAllocator{allocator}),
      workItems(WorkItemAllocator{allocator})
  {
  }

  std::vector<entt::entity, EntityAllocator> entities;
  std::vector<WorkItem, WorkItemAllocator> workItems;
};

//==============================================================================
MultibodyVelocityStage::MultibodyVelocityStage(
    common::MemoryManager* memoryManager)
  : m_scratch(
        memoryManager != nullptr
            ? constructStageOwnedScratch<Scratch>(
                  memoryManager, memoryManager->getFreeAllocator())
            : constructStageOwnedScratch<Scratch>(nullptr),
        ScratchDeleter{memoryManager})
{
}

//==============================================================================
MultibodyVelocityStage::~MultibodyVelocityStage() = default;

//==============================================================================
void MultibodyVelocityStage::ScratchDeleter::operator()(
    Scratch* scratch) const noexcept
{
  destroyStageOwnedScratch(memoryManager, scratch);
}

//==============================================================================
void MultibodyVelocityStage::prepare(World& world)
{
  auto& registry = dart::simulation::detail::registryOf(world);
  std::size_t multibodyCount = 0;
  for (auto entity : registry.view<comps::MultibodyStructure>()) {
    (void)entity;
    ++multibodyCount;
  }

  auto& scratch = *m_scratch;
  scratch.entities.clear();
  scratch.entities.reserve(multibodyCount);
  scratch.workItems.clear();
  scratch.workItems.reserve(multibodyCount);
}

//==============================================================================
void MultibodyVelocityStage::execute(World& world, ComputeExecutor& executor)
{
  auto& registry = dart::simulation::detail::registryOf(world);
  const Eigen::Vector3d gravity = world.getGravity();
  const double timeStep = world.getTimeStep();

  auto view = registry.view<comps::MultibodyStructure>();
  auto& scratch = *m_scratch;
  scratch.entities.clear();
  for (auto entity : view) {
    if (world.isDeactivationActiveForStep()
        && world.isDeactivationEntitySleeping(
            detail::fromRegistryEntity(entity))) {
      continue;
    }

    getOrEmplaceMultibodyDynamicsScratch(world, registry, entity);
    auto& pendingVelocity
        = registry.get_or_emplace<PendingMultibodyVelocity>(entity);
    (void)pendingVelocity;
    scratch.entities.push_back(entity);
  }

  scratch.workItems.clear();
  for (auto entity : scratch.entities) {
    const auto& structure = view.get<comps::MultibodyStructure>(entity);
    auto& dynamicsScratch = registry.get<MultibodyDynamicsScratch>(entity);
    auto& pendingVelocity = registry.get<PendingMultibodyVelocity>(entity);
    scratch.workItems.push_back(
        {&structure, &dynamicsScratch, &pendingVelocity});
  }

  struct RangeContext
  {
    detail::WorldRegistry* registry = nullptr;
    const Eigen::Vector3d* gravity = nullptr;
    double timeStep = 0.0;
    Scratch* scratch = nullptr;
  };
  struct RangeRunner
  {
    static void run(RangeContext& context, std::size_t begin, std::size_t end)
    {
      auto& registry = *context.registry;
      auto& workItems = context.scratch->workItems;
      for (std::size_t i = begin; i < end; ++i) {
        auto& item = workItems[i];
        auto& pendingVelocity = *item.pendingVelocity;
        if (!computeUnconstrainedMultibodyVelocityInto(
                registry,
                *item.structure,
                *context.gravity,
                context.timeStep,
                *item.scratch)) {
          pendingVelocity.active = false;
        } else {
          pendingVelocity.velocity = item.scratch->nextVelocity;
          pendingVelocity.active = true;
        }

        clearMultibodyExternalForces(registry, *item.structure);
      }
    }
  };

  RangeContext context{&registry, &gravity, timeStep, &scratch};
  const auto workItemCount = scratch.workItems.size();
  if (executor.getWorkerCount() <= 1u || workItemCount <= 1u) {
    RangeRunner::run(context, 0u, workItemCount);
    return;
  }

  executor.parallelFor(
      workItemCount, 1u, [&context](std::size_t begin, std::size_t end) {
        RangeRunner::run(context, begin, end);
      });
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
  auto& registry = dart::simulation::detail::registryOf(world);
  const double timeStep = world.getTimeStep();
  auto view = registry.view<comps::MultibodyStructure>();
  if (view.begin() == view.end()) {
    return;
  }
  if (!hasAnyCollisionShapesForMultibodyContacts(registry, view)) {
    return;
  }

  const auto queriedContacts = world.queryContacts(CollisionQueryOptions{});
  std::vector<Contact> deactivationContacts;
  std::span<const Contact> contacts = queriedContacts;
  if (world.isDeactivationActiveForStep()) {
    deactivationContacts = world.filterContactsForDeactivation(contacts);
    contacts = deactivationContacts;
  }
  const LinkOwnerMap* linkOwnerMap = nullptr;
  for (auto entity : view) {
    auto& scratch
        = getOrEmplaceMultibodyDynamicsScratch(world, registry, entity);
    ensureLinkOwnerMapInto(view, scratch.linkOwnerMap);
    linkOwnerMap = &scratch.linkOwnerMap;
    break;
  }
  if (linkOwnerMap == nullptr) {
    return;
  }
  for (auto entity : view) {
    const auto& structure = view.get<comps::MultibodyStructure>(entity);
    auto& scratch
        = getOrEmplaceMultibodyDynamicsScratch(world, registry, entity);
    collectMultibodyLinkContactsInto(
        registry, structure, *linkOwnerMap, contacts, scratch.linkContacts);
    if (scratch.linkContacts.empty()) {
      continue;
    }

    auto* pendingVelocity = registry.try_get<PendingMultibodyVelocity>(entity);
    if (pendingVelocity == nullptr || !pendingVelocity->active) {
      buildDynamicsTreeInto(
          registry,
          structure,
          scratch.tree,
          scratch.linkIndexOf,
          scratch.jointFrameSubspace);
      if (scratch.tree.dofCount == 0) {
        continue;
      }
      pendingVelocity
          = &registry.get_or_emplace<PendingMultibodyVelocity>(entity);
      gatherMultibodyVelocityInto(
          registry, scratch.tree, pendingVelocity->velocity);
      pendingVelocity->active = true;
    }

    solveMultibodyLinkContacts(
        registry,
        structure,
        pendingVelocity->velocity,
        timeStep,
        scratch.linkContacts,
        scratch);
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
  auto& registry = dart::simulation::detail::registryOf(world);
  const double timeStep = world.getTimeStep();

  auto view = registry.view<comps::MultibodyStructure>();
  for (auto entity : view) {
    if (world.isDeactivationActiveForStep()
        && world.isDeactivationEntitySleeping(
            detail::fromRegistryEntity(entity))) {
      if (auto* pendingVelocity
          = registry.try_get<PendingMultibodyVelocity>(entity)) {
        pendingVelocity->active = false;
      }
      continue;
    }

    const auto& structure = view.get<comps::MultibodyStructure>(entity);
    auto& scratch
        = getOrEmplaceMultibodyDynamicsScratch(world, registry, entity);
    Eigen::VectorXd* nextVelocity = nullptr;
    if (auto* pendingVelocity
        = registry.try_get<PendingMultibodyVelocity>(entity);
        pendingVelocity != nullptr && pendingVelocity->active) {
      nextVelocity = &pendingVelocity->velocity;
    } else {
      buildDynamicsTreeInto(
          registry,
          structure,
          scratch.tree,
          scratch.linkIndexOf,
          scratch.jointFrameSubspace);
      gatherMultibodyVelocityInto(registry, scratch.tree, scratch.nextVelocity);
      nextVelocity = &scratch.nextVelocity;
    }

    if (nextVelocity->size() == 0) {
      continue;
    }

    enforceMultibodyVelocityLimits(registry, structure, *nextVelocity);
    projectLockedMultibodyVelocityInto(registry, structure, *nextVelocity);
    integrateMultibodyPositions(registry, structure, *nextVelocity, timeStep);

    if (auto* pendingVelocity
        = registry.try_get<PendingMultibodyVelocity>(entity)) {
      pendingVelocity->active = false;
    }
  }
}

//==============================================================================
void reserveMultibodyDynamicsRegistryStorage(
    detail::WorldRegistry& registry,
    std::size_t multibodyCount,
    common::MemoryAllocator& allocator)
{
  auto& pendingVelocityStorage = registry.storage<PendingMultibodyVelocity>();
  auto& dynamicsScratchStorage = registry.storage<MultibodyDynamicsScratch>();
  pendingVelocityStorage.reserve(multibodyCount);
  dynamicsScratchStorage.reserve(multibodyCount);

  if (multibodyCount == 0u) {
    return;
  }

  auto view = registry.view<comps::MultibodyStructure>();
  for (auto entity : view) {
    const auto& structure = view.get<comps::MultibodyStructure>(entity);
    auto* existingScratch = registry.try_get<MultibodyDynamicsScratch>(entity);
    auto& scratch
        = existingScratch != nullptr
              ? *existingScratch
              : registry.emplace<MultibodyDynamicsScratch>(entity, allocator);
    buildDynamicsTreeInto(
        registry,
        structure,
        scratch.tree,
        scratch.linkIndexOf,
        scratch.jointFrameSubspace);

    const auto linkCount = scratch.tree.links.size();
    const auto dof = static_cast<Eigen::Index>(scratch.tree.dofCount);
    scratch.qdot.resize(dof);
    scratch.appliedForce.resize(dof);
    scratch.zero.resize(dof);
    scratch.unitAcceleration.resize(dof);
    scratch.rneaResponse.resize(dof);
    scratch.massAndBias.bias.resize(dof);
    scratch.massAndBias.gravityOnly.resize(dof);
    scratch.massAndBias.massMatrix.resize(dof, dof);
    scratch.rhs.resize(dof);
    scratch.qddot.resize(dof);
    scratch.nextVelocity.resize(dof);
    scratch.rneaVelocity.resize(linkCount);
    scratch.rneaAcceleration.resize(linkCount);
    scratch.rneaForce.resize(linkCount);
    const std::size_t contactCapacity
        = std::max<std::size_t>(4u, linkCount * 4u);
    scratch.linkContacts.reserve(contactCapacity);
    scratch.constrainedDof.reserve(static_cast<std::size_t>(dof));
    scratch.constrainedTarget.reserve(static_cast<std::size_t>(dof));
    scratch.contactProblem.inverseMass.resize(dof, dof);
    scratch.contactProblem.rows.resize(contactCapacity);
    for (auto& row : scratch.contactProblem.rows) {
      resetMultibodyLinkContactRow(row, dof);
    }
    scratch.activeContactRowCount = 0;
    scratch.bodyJacobian.resize(linkCount);
    for (auto& jacobian : scratch.bodyJacobian) {
      jacobian.resize(6, dof);
    }
    scratch.pointJacobian.resize(3, dof);
    scratch.otherPointJacobian.resize(3, dof);
    scratch.contactWork.resize(dof);
    scratch.contactImpulseDelta.resize(dof);
    reserveDynamicsTreeInverseMassScratch(scratch.tree, scratch);

    if (dof > 0) {
      gatherMultibodyVelocityInto(registry, scratch.tree, scratch.qdot);
      computeMassAndBiasInto(
          linkSpan(scratch.tree),
          scratch.tree.dofCount,
          Eigen::Vector3d::Zero(),
          scratch.qdot,
          scratch.tree.armature,
          scratch.massAndBias,
          scratch.zero,
          scratch.unitAcceleration,
          scratch.rneaResponse,
          scratch.rneaVelocity,
          scratch.rneaAcceleration,
          scratch.rneaForce);
      if (dof > 1) {
        scratch.qddot.setZero(dof);
        (void)solveSpdSystemInto(
            scratch.massAndBias.massMatrix,
            scratch.qddot,
            scratch.symmetricSolution,
            scratch.symmetricFactor,
            scratch.symmetricForwardWork);
      }
      (void)invertSpdMatrixInto(
          scratch.massAndBias.massMatrix,
          scratch.velocityMassInverse,
          scratch.symmetricFactor,
          scratch.symmetricForwardWork,
          scratch.symmetricUnitRhs,
          scratch.symmetricSolution);
      scratch.contactProblem.inverseMass = scratch.velocityMassInverse;

      Eigen::Index velocityConstraintDofs = 0;
      Eigen::Index lockedConstraintDofs = 0;
      for (std::size_t i = 0; i < scratch.tree.links.size(); ++i) {
        const auto dof = scratch.tree.links[i].dof;
        if (dof == 0) {
          continue;
        }
        const auto& jointActuation
            = registry.get<comps::JointActuation>(scratch.tree.jointOf[i]);
        // Velocity and Locked actuators both add rows to the velocity-level
        // equality constraint solved in computeUnconstrainedMultibodyVelocity.
        if (jointActuation.actuatorType == comps::ActuatorType::Velocity
            || jointActuation.actuatorType == comps::ActuatorType::Locked) {
          velocityConstraintDofs += static_cast<Eigen::Index>(dof);
        }
        if (jointActuation.actuatorType == comps::ActuatorType::Locked) {
          lockedConstraintDofs += static_cast<Eigen::Index>(dof);
        }
      }
      if (lockedConstraintDofs > 0) {
        scratch.lockedInverseMassBlock.setIdentity(
            lockedConstraintDofs, lockedConstraintDofs);
        scratch.lockedInverseMassColumns.setZero(dof, lockedConstraintDofs);
        scratch.lockedInverseMassRhs.setZero(lockedConstraintDofs, dof);
        scratch.lockedInverseMassSolve.setZero(lockedConstraintDofs, dof);
        (void)solveSpdMatrixInto(
            scratch.lockedInverseMassBlock,
            scratch.lockedInverseMassRhs,
            scratch.lockedInverseMassSolve,
            scratch.symmetricFactor,
            scratch.symmetricForwardWork);
      }
      if (velocityConstraintDofs > 0) {
        scratch.velocityConstraintMatrix.setIdentity(
            velocityConstraintDofs, velocityConstraintDofs);
        scratch.velocityConstraintResidual.setZero(velocityConstraintDofs);
        scratch.velocityConstraintLambda.setZero(velocityConstraintDofs);
        (void)solveSpdSystemInto(
            scratch.velocityConstraintMatrix,
            scratch.velocityConstraintResidual,
            scratch.velocityConstraintLambda,
            scratch.symmetricFactor,
            scratch.symmetricForwardWork);
      }
    }

    auto& pendingVelocity
        = registry.get_or_emplace<PendingMultibodyVelocity>(entity);
    pendingVelocity.velocity.resize(dof);
    pendingVelocity.active = false;
  }
}

//==============================================================================
struct UnifiedConstraintStage::Scratch
{
  using EntityAllocator = common::StlAllocator<entt::entity>;
  using LinkContactAllocator = common::StlAllocator<LinkContact>;
  using RequiredMultibodyAllocator = common::StlAllocator<char>;
  using UnifiedContactAllocator = common::StlAllocator<UnifiedMultibodyContact>;
  using ScratchPointerAllocator
      = common::StlAllocator<MultibodyDynamicsScratch*>;
  using VelocityAllocator = common::StlAllocator<Eigen::VectorXd>;

  struct LinkContactBucket
  {
    using ContactVector = std::vector<LinkContact, LinkContactAllocator>;

    LinkContactBucket() = default;

    explicit LinkContactBucket(common::MemoryAllocator& allocator)
      : contacts(LinkContactAllocator{allocator})
    {
    }

    ContactVector contacts;
  };

  using LinkContactBucketAllocator = common::StlAllocator<LinkContactBucket>;

  Scratch() = default;

  explicit Scratch(common::MemoryAllocator& allocator)
    : memoryAllocator(&allocator),
      rigidProblem(allocator),
      multibodyEntities(EntityAllocator{allocator}),
      linkOwnerMap(makeLinkOwnerMap(allocator)),
      contactsByMultibody(LinkContactBucketAllocator{allocator}),
      requiredMultibody(RequiredMultibodyAllocator{allocator}),
      multibodyContacts(UnifiedContactAllocator{allocator}),
      multibodyScratch(ScratchPointerAllocator{allocator}),
      multibodyVelocities(VelocityAllocator{allocator}),
      problem(allocator),
      solveScratch(allocator)
  {
  }

  void resizeContactBuckets(std::size_t count)
  {
    while (contactsByMultibody.size() < count) {
      if (memoryAllocator != nullptr) {
        contactsByMultibody.emplace_back(*memoryAllocator);
      } else {
        contactsByMultibody.emplace_back();
      }
    }
    contactsByMultibody.resize(count);
  }

  common::MemoryAllocator* memoryAllocator = nullptr;
  RigidBodyContactProblem rigidProblem;
  std::vector<entt::entity, EntityAllocator> multibodyEntities;
  LinkOwnerMap linkOwnerMap;
  std::vector<LinkContactBucket, LinkContactBucketAllocator>
      contactsByMultibody;
  std::vector<char, RequiredMultibodyAllocator> requiredMultibody;
  std::vector<UnifiedMultibodyContact, UnifiedContactAllocator>
      multibodyContacts;
  std::vector<MultibodyDynamicsScratch*, ScratchPointerAllocator>
      multibodyScratch;
  std::vector<Eigen::VectorXd, VelocityAllocator> multibodyVelocities;
  UnifiedConstraintProblem problem;
  UnifiedConstraintSolveScratch solveScratch;
};

//==============================================================================
bool UnifiedConstraintStage::assembleProblemIntoScratch(
    World& world, std::span<const Contact> contacts)
{
  auto& registry = dart::simulation::detail::registryOf(world);
  const double timeStep = world.getTimeStep();
  auto& scratch = *m_scratch;
  assembleRigidBodyContactProblemInto(scratch.rigidProblem, registry, contacts);

  // Per-multibody link blocks. Recompute each multibody's dynamics tree and
  // inverse mass in-stage (never cached across stages). Mirror
  // MultibodyContactStage's PendingMultibodyVelocity create-from-gather so a
  // multibody whose velocity has not been staged still resolves. Cross-
  // multibody link rows are owned by bodyA's multibody, but the other
  // multibody still needs a block and staged velocity so the solved impulse can
  // update both articulated ends.
  scratch.multibodyEntities.clear();
  auto view = registry.view<comps::MultibodyStructure>();
  for (auto entity : view) {
    scratch.multibodyEntities.push_back(entity);
  }
  ensureLinkOwnerMapInto(view, scratch.linkOwnerMap);

  scratch.resizeContactBuckets(scratch.multibodyEntities.size());
  scratch.requiredMultibody.assign(scratch.multibodyEntities.size(), 0);

  const auto findMultibodyIndex = [&](entt::entity multibody) {
    const auto it = std::find(
        scratch.multibodyEntities.begin(),
        scratch.multibodyEntities.end(),
        multibody);
    if (it == scratch.multibodyEntities.end()) {
      return scratch.multibodyEntities.size();
    }
    return static_cast<std::size_t>(it - scratch.multibodyEntities.begin());
  };

  for (std::size_t k = 0; k < scratch.multibodyEntities.size(); ++k) {
    const auto entity = scratch.multibodyEntities[k];
    const auto& structure = view.get<comps::MultibodyStructure>(entity);
    auto& linkContacts = scratch.contactsByMultibody[k].contacts;
    collectMultibodyLinkContactsInto(
        registry, structure, scratch.linkOwnerMap, contacts, linkContacts);
    if (!linkContacts.empty()) {
      scratch.requiredMultibody[k] = 1;
    }
  }
  for (const auto& linkContacts : scratch.contactsByMultibody) {
    for (const auto& contact : linkContacts.contacts) {
      if (contact.otherMultibody != entt::null) {
        const auto otherIndex = findMultibodyIndex(contact.otherMultibody);
        if (otherIndex < scratch.requiredMultibody.size()) {
          scratch.requiredMultibody[otherIndex] = 1;
        }
      }
    }
  }

  std::size_t stagedBlockCount = 0;
  for (std::size_t k = 0; k < scratch.multibodyEntities.size(); ++k) {
    const auto entity = scratch.multibodyEntities[k];
    const auto& linkContacts = scratch.contactsByMultibody[k].contacts;
    if (linkContacts.empty() && scratch.requiredMultibody[k] == 0) {
      continue;
    }

    const auto& structure = registry.get<comps::MultibodyStructure>(entity);
    if (stagedBlockCount == scratch.multibodyVelocities.size()) {
      scratch.multibodyVelocities.emplace_back();
    }
    auto& stagedVelocity = scratch.multibodyVelocities[stagedBlockCount];
    auto& dynamicsScratch
        = getOrEmplaceMultibodyDynamicsScratch(world, registry, entity);
    auto* pendingVelocity = registry.try_get<PendingMultibodyVelocity>(entity);
    if (pendingVelocity != nullptr && pendingVelocity->active) {
      stagedVelocity = pendingVelocity->velocity;
    } else {
      buildDynamicsTreeInto(
          registry,
          structure,
          dynamicsScratch.tree,
          dynamicsScratch.linkIndexOf,
          dynamicsScratch.jointFrameSubspace);
      gatherMultibodyVelocityInto(
          registry, dynamicsScratch.tree, stagedVelocity);
    }

    assembleMultibodyLinkContactProblemInto(
        registry,
        structure,
        stagedVelocity,
        timeStep,
        linkContacts,
        dynamicsScratch);
    if (stagedBlockCount == scratch.multibodyContacts.size()) {
      scratch.multibodyContacts.emplace_back();
    }
    if (stagedBlockCount == scratch.multibodyScratch.size()) {
      scratch.multibodyScratch.push_back(nullptr);
    }
    auto& stagedContact = scratch.multibodyContacts[stagedBlockCount];
    stagedContact.multibody = entity;
    stagedContact.borrowedProblem = &dynamicsScratch.contactProblem;
    scratch.multibodyScratch[stagedBlockCount] = &dynamicsScratch;
    ++stagedBlockCount;
  }
  scratch.multibodyContacts.resize(stagedBlockCount);
  scratch.multibodyScratch.resize(stagedBlockCount);
  scratch.multibodyVelocities.resize(stagedBlockCount);
  completeCrossMultibodyLinkRows(
      scratch.multibodyContacts,
      scratch.multibodyScratch,
      scratch.multibodyVelocities);

  bool hasActiveLinkRows = false;
  for (const auto& contact : scratch.multibodyContacts) {
    for (const auto& row : problemOf(contact).rows) {
      hasActiveLinkRows = hasActiveLinkRows || row.active;
    }
  }
  if (scratch.rigidProblem.constraints.empty() && !hasActiveLinkRows) {
    return false;
  }

  assembleUnifiedConstraintProblemInto(
      scratch.problem, scratch.rigidProblem, scratch.multibodyContacts);
  return true;
}

//==============================================================================
UnifiedConstraintStage::UnifiedConstraintStage(std::size_t frictionIterations)
  : UnifiedConstraintStage(frictionIterations, nullptr)
{
}

//==============================================================================
UnifiedConstraintStage::UnifiedConstraintStage(
    std::size_t frictionIterations, common::MemoryManager* memoryManager)
  : m_frictionIterations(std::max<std::size_t>(1, frictionIterations)),
    m_scratch(
        memoryManager != nullptr
            ? constructStageOwnedScratch<Scratch>(
                  memoryManager, memoryManager->getFreeAllocator())
            : constructStageOwnedScratch<Scratch>(nullptr),
        ScratchDeleter{memoryManager})
{
}

//==============================================================================
UnifiedConstraintStage::~UnifiedConstraintStage() = default;

//==============================================================================
void UnifiedConstraintStage::ScratchDeleter::operator()(
    Scratch* scratch) const noexcept
{
  destroyStageOwnedScratch(memoryManager, scratch);
}

//==============================================================================
UnifiedConstraintStage::UnifiedConstraintStage(
    UnifiedConstraintStage&&) noexcept = default;

//==============================================================================
UnifiedConstraintStage& UnifiedConstraintStage::operator=(
    UnifiedConstraintStage&&) noexcept = default;

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
void UnifiedConstraintStage::prepare(World& world)
{
  if (world.getContactSolverMethod() != ContactSolverMethod::BoxedLcp) {
    return;
  }
  auto& registry = dart::simulation::detail::registryOf(world);
  if (!hasAnyCollisionShapes(registry)) {
    return;
  }

  const auto contacts = world.queryContacts(CollisionQueryOptions{});
  if (contacts.empty()) {
    return;
  }
  if (!assembleProblemIntoScratch(world, contacts)) {
    return;
  }

  // Bake-time shape priming only. This allocates same-shape solver storage at
  // enterSimulationMode() without writing impulses back to the World state.
  auto& scratch = *m_scratch;
  solveUnifiedConstraintProblemInto(scratch.problem, scratch.solveScratch);
  primeUnifiedConstraintFallbackScratch(scratch.problem, scratch.solveScratch);
}

//==============================================================================
bool tryResolveSequentialMultibodyContacts(
    World& world,
    detail::WorldRegistry& registry,
    std::span<const Contact> contacts,
    double timeStep)
{
  if (world.getContactSolverMethod() == ContactSolverMethod::BoxedLcp) {
    return false;
  }

  const auto isRigidBody = [&](entt::entity entity) {
    return registry.all_of<comps::RigidBodyTag>(entity);
  };
  const auto isLink = [&](entt::entity entity) {
    return registry.all_of<comps::LinkModel>(entity);
  };

  for (const auto& contact : contacts) {
    const auto entityA = detail::toRegistryEntity(contact.bodyA.getEntity());
    const auto entityB = detail::toRegistryEntity(contact.bodyB.getEntity());
    const bool aIsRigid = isRigidBody(entityA);
    const bool bIsRigid = isRigidBody(entityB);
    if (aIsRigid && bIsRigid) {
      return false;
    }
    if (!isLink(entityA) && !isLink(entityB)) {
      return false;
    }
  }

  auto view = registry.view<comps::MultibodyStructure>();
  const LinkOwnerMap* linkOwnerMap = nullptr;
  for (auto entity : view) {
    auto& scratch
        = getOrEmplaceMultibodyDynamicsScratch(world, registry, entity);
    ensureLinkOwnerMapInto(view, scratch.linkOwnerMap);
    linkOwnerMap = &scratch.linkOwnerMap;
    break;
  }
  if (linkOwnerMap == nullptr) {
    return false;
  }
  bool hasHandledLinkContact = false;
  bool hasCrossMultibodyLinkContact = false;
  bool hasNonCrossLinkContact = false;
  std::size_t crossMultibodyLinkContactCount = 0;
  for (auto entity : view) {
    const auto& structure = view.get<comps::MultibodyStructure>(entity);
    auto& scratch
        = getOrEmplaceMultibodyDynamicsScratch(world, registry, entity);
    collectMultibodyLinkContactsInto(
        registry, structure, *linkOwnerMap, contacts, scratch.linkContacts);
    for (const auto& contact : scratch.linkContacts) {
      if (contact.otherMultibody != entt::null) {
        hasCrossMultibodyLinkContact = true;
        ++crossMultibodyLinkContactCount;
      } else {
        hasNonCrossLinkContact = true;
      }
    }
    hasHandledLinkContact
        = hasHandledLinkContact || !scratch.linkContacts.empty();
  }
  if (!hasHandledLinkContact) {
    return false;
  }
  if (hasCrossMultibodyLinkContact
      && (hasNonCrossLinkContact || crossMultibodyLinkContactCount != 1)) {
    return false;
  }

  const auto isRequiredCrossMultibody = [&](entt::entity candidate) {
    if (!hasCrossMultibodyLinkContact) {
      return false;
    }
    for (auto entity : view) {
      const auto* scratch = registry.try_get<MultibodyDynamicsScratch>(entity);
      if (scratch == nullptr) {
        continue;
      }
      for (const auto& contact : scratch->linkContacts) {
        if (contact.otherMultibody == candidate) {
          return true;
        }
      }
    }
    return false;
  };

  for (auto entity : view) {
    const auto& structure = view.get<comps::MultibodyStructure>(entity);
    auto& scratch = registry.get<MultibodyDynamicsScratch>(entity);
    const bool hasOwnedLinkContacts = !scratch.linkContacts.empty();
    if (!hasOwnedLinkContacts && !isRequiredCrossMultibody(entity)) {
      continue;
    }
    if (!hasOwnedLinkContacts) {
      scratch.activeContactRowCount = 0;
    }

    auto* pendingVelocity = registry.try_get<PendingMultibodyVelocity>(entity);
    if (pendingVelocity != nullptr && pendingVelocity->active) {
      if (!prepareMultibodyContactDynamicsInto(
              registry, structure, pendingVelocity->velocity, scratch)) {
        return false;
      }
      continue;
    }

    buildDynamicsTreeInto(
        registry,
        structure,
        scratch.tree,
        scratch.linkIndexOf,
        scratch.jointFrameSubspace);
    if (scratch.tree.dofCount == 0) {
      return false;
    }
    pendingVelocity
        = &registry.get_or_emplace<PendingMultibodyVelocity>(entity);
    gatherMultibodyVelocityInto(
        registry, scratch.tree, pendingVelocity->velocity);
    pendingVelocity->active = true;
    if (!prepareMultibodyContactDynamicsInto(
            registry, structure, pendingVelocity->velocity, scratch)) {
      return false;
    }
  }

  if (hasCrossMultibodyLinkContact) {
    for (auto entity : view) {
      const auto& scratch = registry.get<MultibodyDynamicsScratch>(entity);
      for (const auto& contact : scratch.linkContacts) {
        if (contact.otherMultibody == entt::null) {
          continue;
        }

        const auto* otherScratch = registry.try_get<MultibodyDynamicsScratch>(
            contact.otherMultibody);
        if (otherScratch == nullptr
            || scratch.tree.dofCount != otherScratch->tree.dofCount) {
          return false;
        }
      }
    }
  }

  for (auto entity : view) {
    const auto& structure = view.get<comps::MultibodyStructure>(entity);
    auto& scratch = registry.get<MultibodyDynamicsScratch>(entity);
    if (scratch.linkContacts.empty()) {
      continue;
    }

    auto* pendingVelocity = registry.try_get<PendingMultibodyVelocity>(entity);
    if (pendingVelocity == nullptr || !pendingVelocity->active) {
      buildDynamicsTreeInto(
          registry,
          structure,
          scratch.tree,
          scratch.linkIndexOf,
          scratch.jointFrameSubspace);
      if (scratch.tree.dofCount == 0) {
        return false;
      }
      pendingVelocity
          = &registry.get_or_emplace<PendingMultibodyVelocity>(entity);
      gatherMultibodyVelocityInto(
          registry, scratch.tree, pendingVelocity->velocity);
      pendingVelocity->active = true;
    }

    solveMultibodyLinkContacts(
        registry,
        structure,
        pendingVelocity->velocity,
        timeStep,
        scratch.linkContacts,
        scratch);
  }
  if (hasCrossMultibodyLinkContact) {
    for (auto entity : view) {
      auto* pendingVelocity
          = registry.try_get<PendingMultibodyVelocity>(entity);
      if (pendingVelocity == nullptr || !pendingVelocity->active) {
        continue;
      }

      auto& scratch = registry.get<MultibodyDynamicsScratch>(entity);
      completeSequentialCrossMultibodyLinkRows(
          registry, pendingVelocity->velocity, scratch);
    }
    solveSequentialCrossMultibodyLinkRows(registry);
  }
  return true;
}

//==============================================================================
void UnifiedConstraintStage::execute(World& world, ComputeExecutor& executor)
{
  auto& registry = dart::simulation::detail::registryOf(world);
  const double timeStep = world.getTimeStep();
  if (!hasAnyCollisionShapes(registry)) {
    return;
  }
  const auto queriedContacts = world.queryContacts(CollisionQueryOptions{});
  std::vector<Contact> deactivationContacts;
  std::span<const Contact> contacts = queriedContacts;
  if (world.isDeactivationActiveForStep()) {
    deactivationContacts = world.filterContactsForDeactivation(contacts);
    contacts = deactivationContacts;
  }
  if (contacts.empty()) {
    return;
  }
  if (tryResolveSequentialMultibodyContacts(
          world, registry, contacts, timeStep)) {
    return;
  }

  auto& scratch = *m_scratch;
  if (!assembleProblemIntoScratch(world, contacts)) {
    return;
  }

  if (solveUnifiedConstraintProblemInto(
          scratch.problem, scratch.solveScratch, executor)) {
    applyUnifiedConstraintImpulses(
        registry,
        scratch.problem,
        scratch.solveScratch.lambda,
        std::span<Eigen::VectorXd>(scratch.multibodyVelocities),
        scratch.solveScratch);
  } else {
    applyUnifiedConstraintFallback(
        registry,
        scratch.problem,
        std::span<Eigen::VectorXd>(scratch.multibodyVelocities),
        m_frictionIterations,
        scratch.solveScratch);
  }

  // Write each multibody's resolved generalized velocity back to its staging
  // component so the position stage integrates the post-contact velocity. The
  // blocks are in the same order the staged velocities were collected.
  for (std::size_t k = 0; k < scratch.problem.multibodyBlocks.size(); ++k) {
    auto& pendingVelocity = registry.get_or_emplace<PendingMultibodyVelocity>(
        scratch.problem.multibodyBlocks[k].multibody);
    if (scratch.multibodyVelocities[k].size() == 0) {
      pendingVelocity.active = false;
      continue;
    }
    pendingVelocity.velocity = scratch.multibodyVelocities[k];
    pendingVelocity.active = true;
  }

  // Rigid positional projection: remove residual penetration beyond a small
  // allowance without injecting velocity (verbatim from RigidBodyContactStage).
  constexpr double allowance = 1e-4;
  constexpr double correctionFactor = 0.2;
  for (const auto& constraint : scratch.problem.rigidConstraints) {
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

} // namespace dart::simulation::compute
