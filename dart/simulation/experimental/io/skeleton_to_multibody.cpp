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

#include "dart/simulation/experimental/io/skeleton_to_multibody.hpp"

#include "dart/simulation/experimental/common/exceptions.hpp"
#include "dart/simulation/experimental/multibody/joint.hpp"
#include "dart/simulation/experimental/multibody/joint_type.hpp"
#include "dart/simulation/experimental/multibody/link.hpp"
#include "dart/simulation/experimental/world.hpp"

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/inertia.hpp>
#include <dart/dynamics/joint.hpp>
#include <dart/dynamics/prismatic_joint.hpp>
#include <dart/dynamics/revolute_joint.hpp>
#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/weld_joint.hpp>

#include <Eigen/Geometry>

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace dart::simulation::experimental::io {

namespace {

// Translation offset whose direction perpendicular to a joint axis the
// experimental anchoring convention cannot represent. Anything below this is
// treated as "the joint is anchored at the parent link origin".
constexpr double kAnchorTolerance = 1e-9;

/// A legacy joint mapped onto the experimental facade.
struct MappedJoint
{
  JointType type;
  Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();
  Eigen::Vector3d axis2 = Eigen::Vector3d::UnitX();
  std::size_t dof = 0;
};

/// Map a legacy joint type/axis onto the experimental facade, or throw with a
/// message naming the unsupported type.
MappedJoint mapJoint(const dynamics::Joint& joint)
{
  const auto type = joint.getType();
  if (type == dynamics::WeldJoint::getStaticType()) {
    // A fixed joint carries no motion axis, but the facade still validates a
    // non-zero axis, so keep the default unit axes.
    return MappedJoint{JointType::Fixed};
  }
  if (type == dynamics::RevoluteJoint::getStaticType()) {
    const auto& revolute = static_cast<const dynamics::RevoluteJoint&>(joint);
    return MappedJoint{
        JointType::Revolute, revolute.getAxis(), Eigen::Vector3d::UnitX(), 1};
  }
  if (type == dynamics::PrismaticJoint::getStaticType()) {
    const auto& prismatic = static_cast<const dynamics::PrismaticJoint&>(joint);
    return MappedJoint{
        JointType::Prismatic, prismatic.getAxis(), Eigen::Vector3d::UnitX(), 1};
  }

  DART_EXPERIMENTAL_THROW_T(
      InvalidOperationException,
      "buildMultibodyFromSkeleton does not yet support the joint type '{}' "
      "(joint '{}'); the supported types are weld, revolute, and prismatic",
      type,
      joint.getName());
}

/// The constant offset that places body `b`'s experimental link frame onto its
/// outgoing joint, so that joint stays anchored at this link's frame origin.
/// Leaf bodies keep the legacy body frame (identity offset).
Eigen::Isometry3d outgoingJointOffset(const dynamics::BodyNode& body)
{
  if (body.getNumChildBodyNodes() == 0) {
    return Eigen::Isometry3d::Identity();
  }
  return body.getChildBodyNode(0)
      ->getParentJoint()
      ->getTransformFromParentBodyNode();
}

/// A fully-resolved plan for one experimental link, computed from the skeleton
/// alone (no World mutation). Resolving every link up front lets the conversion
/// reject an unrepresentable skeleton before anything is added to the World, so
/// a rejected load leaves the World untouched rather than registering a partial
/// multibody.
struct LinkPlan
{
  const dynamics::BodyNode* body = nullptr;
  const dynamics::BodyNode* parentBody = nullptr; // null => synthetic base
  std::string linkName;
  JointSpec spec;
  double mass = 0.0;
  Eigen::Vector3d com = Eigen::Vector3d::Zero();
  Eigen::Matrix3d inertia = Eigen::Matrix3d::Identity();
  Eigen::VectorXd position; // copied 1-DOF state; empty when not copied
  Eigen::VectorXd velocity;
};

} // namespace

//==============================================================================
Multibody buildMultibodyFromSkeleton(
    World& world,
    const dynamics::Skeleton& skeleton,
    const SkeletonToMultibodyOptions& options)
{
  const std::size_t bodyCount = skeleton.getNumBodyNodes();

  // Per-body frame offset placing each link onto its own outgoing joint.
  std::unordered_map<const dynamics::BodyNode*, Eigen::Isometry3d> offsetOf;
  offsetOf.reserve(bodyCount);
  for (std::size_t i = 0; i < bodyCount; ++i) {
    const auto* body = skeleton.getBodyNode(i);
    offsetOf.emplace(body, outgoingJointOffset(*body));
  }

  // Plan pass: resolve every link from the skeleton alone and reject anything
  // unrepresentable before touching the World, so a rejected load leaves the
  // World unchanged. Bodies are visited in skeleton index order; a parent
  // always precedes its children, which also makes the experimental
  // joint-construction order match the legacy DOF ordering.
  std::vector<LinkPlan> plans;
  plans.reserve(bodyCount);
  for (std::size_t i = 0; i < bodyCount; ++i) {
    const auto* body = skeleton.getBodyNode(i);
    const auto* joint = body->getParentJoint();
    const auto* parentBody = body->getParentBodyNode();

    const Eigen::Isometry3d parentOffset
        = parentBody ? offsetOf.at(parentBody) : Eigen::Isometry3d::Identity();
    const Eigen::Isometry3d& childOffset = offsetOf.at(body);

    const MappedJoint mapped = mapJoint(*joint);

    // childInParentLegacy(q) = A * Q(q) * C^-1, with A/C the legacy
    // parent/child offsets. Re-expressed between the placed experimental frames
    // it becomes
    //   M * Q(q) * C^-1 * childOffset,   M = parentOffset^-1 * A.
    // The experimental facade applies the joint motion at the parent origin, so
    // M must be a pure rotation for a moving joint (otherwise the motion is not
    // anchored at the parent origin: an offset root joint or a branch whose
    // siblings do not share a parent-side frame).
    const Eigen::Isometry3d A = joint->getTransformFromParentBodyNode();
    const Eigen::Isometry3d C = joint->getTransformFromChildBodyNode();
    const Eigen::Isometry3d M = parentOffset.inverse() * A;

    if (mapped.type != JointType::Fixed
        && M.translation().norm() > kAnchorTolerance) {
      DART_EXPERIMENTAL_THROW_T(
          InvalidOperationException,
          "buildMultibodyFromSkeleton cannot anchor joint '{}' at its parent "
          "link origin (the parent-side offset is non-zero). Offset root "
          "joints and branches whose sibling joints do not share a parent-side "
          "frame are not yet supported.",
          joint->getName());
    }

    const double mass = body->getMass();
    if (!(mass > 0.0)) {
      DART_EXPERIMENTAL_THROW_T(
          InvalidOperationException,
          "buildMultibodyFromSkeleton requires a positive link mass; body '{}' "
          "has non-positive mass (massless links are not yet supported)",
          body->getName());
    }

    LinkPlan plan;
    plan.body = body;
    plan.parentBody = parentBody;
    plan.linkName = body->getName();
    plan.spec.name = joint->getName();
    plan.spec.type = mapped.type;
    plan.spec.axis = M.linear() * mapped.axis;
    plan.spec.axis2 = M.linear() * mapped.axis2;
    plan.spec.transformFromParent = M * C.inverse() * childOffset;
    plan.mass = mass;

    // Inertial properties re-expressed in the placed link frame. The link frame
    // equals the legacy body frame composed with childOffset, so a body-frame
    // point p maps to childOffset^-1 * p and the moment about the center of
    // mass rotates by R = childOffset.linear().
    const Eigen::Matrix3d rotation = childOffset.linear();
    plan.com = childOffset.inverse() * body->getLocalCOM();
    plan.inertia
        = rotation.transpose() * body->getInertia().getMoment() * rotation;

    if (options.copyState && mapped.dof == 1) {
      plan.position = Eigen::VectorXd::Constant(1, joint->getPosition(0));
      plan.velocity = Eigen::VectorXd::Constant(1, joint->getVelocity(0));
    }

    plans.push_back(std::move(plan));
  }

  // Apply pass: every link is representable, so the World is mutated only now.
  const std::string multibodyName
      = options.name.empty() ? skeleton.getName() : options.name;
  Multibody multibody = world.addMultibody(multibodyName);

  // Synthetic fixed base link representing the world frame. The skeleton's root
  // bodies attach beneath it so that a root body's parent-joint offset is
  // realized rather than dropped.
  Link base = multibody.addLink(options.baseLinkName);
  base.setMass(1.0);
  base.setInertia(Eigen::Matrix3d::Identity());

  std::unordered_map<const dynamics::BodyNode*, Link> linkOf;
  linkOf.reserve(plans.size());
  for (const auto& plan : plans) {
    const Link parentLink = plan.parentBody ? linkOf.at(plan.parentBody) : base;
    Link link = multibody.addLink(plan.linkName, parentLink, plan.spec);
    link.setMass(plan.mass);
    link.setCenterOfMass(plan.com);
    link.setInertia(plan.inertia);

    if (plan.position.size() == 1) {
      link.getParentJoint().setPosition(plan.position);
      link.getParentJoint().setVelocity(plan.velocity);
    }

    linkOf.emplace(plan.body, link);
  }

  return multibody;
}

} // namespace dart::simulation::experimental::io
