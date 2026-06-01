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

#include <dart/simulation/world.hpp>

#include <dart/dynamics/ball_joint.hpp>
#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/capsule_shape.hpp>
#include <dart/dynamics/convex_mesh_shape.hpp>
#include <dart/dynamics/cylinder_shape.hpp>
#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/inertia.hpp>
#include <dart/dynamics/joint.hpp>
#include <dart/dynamics/mesh_shape.hpp>
#include <dart/dynamics/planar_joint.hpp>
#include <dart/dynamics/plane_shape.hpp>
#include <dart/dynamics/prismatic_joint.hpp>
#include <dart/dynamics/revolute_joint.hpp>
#include <dart/dynamics/screw_joint.hpp>
#include <dart/dynamics/shape.hpp>
#include <dart/dynamics/shape_frame.hpp>
#include <dart/dynamics/shape_node.hpp>
#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/sphere_shape.hpp>
#include <dart/dynamics/universal_joint.hpp>
#include <dart/dynamics/weld_joint.hpp>

#include <Eigen/Geometry>

#include <numbers>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace dart::simulation::experimental::io {

namespace {

// Tolerance for treating a joint offset's rotation as identity
// (ball/free/planar joints require identity-rotation parent/child offsets).
constexpr double kAnchorTolerance = 1e-9;

/// Rotation matrix from an exponential-coordinate rotation vector (Rodrigues).
Eigen::Matrix3d rotationFromVector(const Eigen::Vector3d& rotationVector)
{
  const double angle = rotationVector.norm();
  if (angle < 1e-12) {
    return Eigen::Matrix3d::Identity();
  }
  return Eigen::AngleAxisd(angle, rotationVector / angle).toRotationMatrix();
}

/// A legacy joint mapped onto the experimental facade.
struct MappedJoint
{
  JointType type = JointType::Fixed;
  Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();
  Eigen::Vector3d axis2 = Eigen::Vector3d::UnitX();
  double pitch = 0.0; // experimental screw pitch (translation per radian)
  std::size_t dof = 0;
  // Axis-based joints (revolute/prismatic/screw/universal) and weld joints
  // accept arbitrary parent/child offsets. The orientation-coordinate joints
  // (ball/free/planar) require identity-rotation offsets because their
  // generalized coordinates are not yet re-expressed under a rotated joint
  // frame; reframable is false for them.
  bool reframable = true;
};

/// Map a legacy joint type/axis onto the experimental facade, or throw with a
/// message naming the unsupported type.
MappedJoint mapJoint(const dynamics::Joint& joint)
{
  const auto type = joint.getType();

  if (type == dynamics::WeldJoint::getStaticType()) {
    // A fixed joint carries no motion axis, but the facade still validates a
    // non-zero axis, so keep the default unit axes.
    return MappedJoint{};
  }
  if (type == dynamics::RevoluteJoint::getStaticType()) {
    MappedJoint mapped;
    mapped.type = JointType::Revolute;
    mapped.axis = static_cast<const dynamics::RevoluteJoint&>(joint).getAxis();
    mapped.dof = 1;
    return mapped;
  }
  if (type == dynamics::PrismaticJoint::getStaticType()) {
    MappedJoint mapped;
    mapped.type = JointType::Prismatic;
    mapped.axis = static_cast<const dynamics::PrismaticJoint&>(joint).getAxis();
    mapped.dof = 1;
    return mapped;
  }
  if (type == dynamics::ScrewJoint::getStaticType()) {
    const auto& screw = static_cast<const dynamics::ScrewJoint&>(joint);
    MappedJoint mapped;
    mapped.type = JointType::Screw;
    mapped.axis = screw.getAxis();
    // Legacy pitch is translation per revolution; the experimental screw uses
    // translation per radian.
    mapped.pitch = screw.getPitch() / (2.0 * std::numbers::pi);
    mapped.dof = 1;
    return mapped;
  }
  if (type == dynamics::UniversalJoint::getStaticType()) {
    const auto& universal = static_cast<const dynamics::UniversalJoint&>(joint);
    MappedJoint mapped;
    mapped.type = JointType::Universal;
    mapped.axis = universal.getAxis1();
    mapped.axis2 = universal.getAxis2();
    mapped.dof = 2;
    return mapped;
  }
  if (type == dynamics::BallJoint::getStaticType()) {
    MappedJoint mapped;
    mapped.type = JointType::Spherical;
    mapped.dof = 3;
    mapped.reframable = false;
    return mapped;
  }
  if (type == dynamics::FreeJoint::getStaticType()) {
    MappedJoint mapped;
    mapped.type = JointType::Floating;
    mapped.dof = 6;
    mapped.reframable = false;
    return mapped;
  }
  if (type == dynamics::PlanarJoint::getStaticType()) {
    const auto& planar = static_cast<const dynamics::PlanarJoint&>(joint);
    MappedJoint mapped;
    mapped.type = JointType::Planar;
    mapped.axis = planar.getRotationalAxis();      // plane normal
    mapped.axis2 = planar.getTranslationalAxis1(); // first in-plane direction
    mapped.dof = 3;
    mapped.reframable = false;
    return mapped;
  }

  DART_EXPERIMENTAL_THROW_T(
      InvalidOperationException,
      "buildMultibodyFromSkeleton does not yet support the joint type '{}' "
      "(joint '{}'); the supported types are weld, revolute, prismatic, screw, "
      "universal, ball, free, and planar",
      type,
      joint.getName());
}

/// The legacy joint's current generalized position and velocity, re-expressed
/// in the experimental coordinate convention.
std::pair<Eigen::VectorXd, Eigen::VectorXd> mapJointState(
    const dynamics::Joint& joint, const MappedJoint& mapped)
{
  const auto dof = static_cast<Eigen::Index>(joint.getNumDofs());
  Eigen::VectorXd position(dof);
  Eigen::VectorXd velocity(dof);
  for (Eigen::Index i = 0; i < dof; ++i) {
    position[i] = joint.getPosition(static_cast<std::size_t>(i));
    velocity[i] = joint.getVelocity(static_cast<std::size_t>(i));
  }

  if (mapped.type == JointType::Floating) {
    // Legacy free joint: position [rotation(0..2); translation(3..5)] and a
    // generalized velocity [angular; linear] expressed in the PARENT frame (its
    // relative Jacobian is diag(R^T, R^T)). Experimental floating joint:
    // position [translation(0..2); rotation(3..5)] and a [linear; angular] body
    // twist. So the position halves swap, and the velocity halves swap and
    // rotate into the body frame by R^T, where R is the current joint rotation.
    const Eigen::Matrix3d rotation = rotationFromVector(position.head<3>());
    Eigen::VectorXd mappedPosition(6);
    Eigen::VectorXd mappedVelocity(6);
    mappedPosition.head<3>() = position.tail<3>();
    mappedPosition.tail<3>() = position.head<3>();
    mappedVelocity.head<3>() = rotation.transpose() * velocity.tail<3>();
    mappedVelocity.tail<3>() = rotation.transpose() * velocity.head<3>();
    return {mappedPosition, mappedVelocity};
  }

  // The remaining supported joints share the legacy coordinate layout (the
  // screw differs only by the pitch scale, not the angle coordinate itself).
  return {position, velocity};
}

/// Copy per-coordinate joint properties (position/velocity/effort limits,
/// damping, spring stiffness, rest position, Coulomb friction) from a legacy
/// joint to the experimental joint. Both joints share the same coordinate count
/// and order (used for revolute and prismatic joints).
void copyJointProperties(const dynamics::Joint& legacy, Joint& experimental)
{
  const auto dof = static_cast<Eigen::Index>(legacy.getNumDofs());
  Eigen::VectorXd lower(dof);
  Eigen::VectorXd upper(dof);
  Eigen::VectorXd damping(dof);
  Eigen::VectorXd stiffness(dof);
  Eigen::VectorXd restPosition(dof);
  Eigen::VectorXd friction(dof);

  for (Eigen::Index i = 0; i < dof; ++i) {
    const auto c = static_cast<std::size_t>(i);
    damping[i] = legacy.getDampingCoefficient(c);
    stiffness[i] = legacy.getSpringStiffness(c);
    restPosition[i] = legacy.getRestPosition(c);
    friction[i] = legacy.getCoulombFriction(c);
  }
  experimental.setDampingCoefficient(damping);
  experimental.setSpringStiffness(stiffness);
  experimental.setRestPosition(restPosition);
  experimental.setCoulombFriction(friction);

  for (Eigen::Index i = 0; i < dof; ++i) {
    const auto c = static_cast<std::size_t>(i);
    lower[i] = legacy.getPositionLowerLimit(c);
    upper[i] = legacy.getPositionUpperLimit(c);
  }
  experimental.setPositionLimits(lower, upper);
  for (Eigen::Index i = 0; i < dof; ++i) {
    const auto c = static_cast<std::size_t>(i);
    lower[i] = legacy.getVelocityLowerLimit(c);
    upper[i] = legacy.getVelocityUpperLimit(c);
  }
  experimental.setVelocityLimits(lower, upper);
  for (Eigen::Index i = 0; i < dof; ++i) {
    const auto c = static_cast<std::size_t>(i);
    lower[i] = legacy.getForceLowerLimit(c);
    upper[i] = legacy.getForceUpperLimit(c);
  }
  experimental.setEffortLimits(lower, upper);
}

std::optional<CollisionShape> makeMeshCollisionShape(
    const std::shared_ptr<math::TriMesh<double>>& triMesh,
    const Eigen::Vector3d& scale)
{
  if (triMesh == nullptr || triMesh->getVertices().empty()
      || triMesh->getTriangles().empty()) {
    return std::nullopt;
  }

  std::vector<Eigen::Vector3d> vertices;
  vertices.reserve(triMesh->getVertices().size());
  for (const auto& vertex : triMesh->getVertices()) {
    vertices.push_back(vertex.cwiseProduct(scale));
  }

  std::vector<Eigen::Vector3i> triangles;
  triangles.reserve(triMesh->getTriangles().size());
  for (const auto& triangle : triMesh->getTriangles()) {
    triangles.emplace_back(
        static_cast<int>(triangle[0]),
        static_cast<int>(triangle[1]),
        static_cast<int>(triangle[2]));
  }

  return CollisionShape::makeMesh(std::move(vertices), std::move(triangles));
}

/// The first sphere, box, capsule, cylinder, plane, triangular mesh, or convex
/// mesh collision shape of a body (a shape node with a collision aspect), as an
/// experimental CollisionShape. The shape node's relative transform becomes the
/// CollisionShape local transform.
/// Returns nullopt when the body has no representable collision shape;
/// unsupported mesh-like variants and additional shapes per body are skipped.
std::optional<CollisionShape> translateCollisionShape(
    const dynamics::BodyNode& body)
{
  std::optional<CollisionShape> result;
  body.eachShapeNodeWith<dynamics::CollisionAspect>(
      [&result](const dynamics::ShapeNode* shapeNode) {
        if (result.has_value() || shapeNode == nullptr) {
          return;
        }
        const auto shape = shapeNode->getShape();
        if (!shape) {
          return;
        }
        const auto type = shape->getType();
        if (type == dynamics::SphereShape::getStaticType()) {
          result = CollisionShape::makeSphere(
              static_cast<const dynamics::SphereShape&>(*shape).getRadius());
        } else if (type == dynamics::BoxShape::getStaticType()) {
          // Legacy box size is the full extent; the facade uses half extents.
          result = CollisionShape::makeBox(
              static_cast<const dynamics::BoxShape&>(*shape).getSize() / 2.0);
        } else if (type == dynamics::CapsuleShape::getStaticType()) {
          const auto& capsule
              = static_cast<const dynamics::CapsuleShape&>(*shape);
          result = CollisionShape::makeCapsule(
              capsule.getRadius(), capsule.getHeight());
        } else if (type == dynamics::CylinderShape::getStaticType()) {
          const auto& cylinder
              = static_cast<const dynamics::CylinderShape&>(*shape);
          result = CollisionShape::makeCylinder(
              cylinder.getRadius(), cylinder.getHeight());
        } else if (type == dynamics::PlaneShape::getStaticType()) {
          const auto& plane = static_cast<const dynamics::PlaneShape&>(*shape);
          result
              = CollisionShape::makePlane(plane.getNormal(), plane.getOffset());
        } else if (type == dynamics::MeshShape::getStaticType()) {
          const auto& mesh = static_cast<const dynamics::MeshShape&>(*shape);
          result = makeMeshCollisionShape(mesh.getTriMesh(), mesh.getScale());
        } else if (type == dynamics::ConvexMeshShape::getStaticType()) {
          const auto& convex
              = static_cast<const dynamics::ConvexMeshShape&>(*shape);
          result = makeMeshCollisionShape(
              convex.getMesh(), Eigen::Vector3d::Ones());
        }
        if (result.has_value()) {
          result->localTransform = shapeNode->getRelativeTransform();
        }
        // Soft and heightmap mesh-like variants are not yet translated.
      });
  return result;
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
  const dynamics::Joint* joint = nullptr;         // legacy parent joint
  std::string linkName;
  JointSpec spec;
  double pitch = 0.0; // experimental screw pitch (applied after construction)
  double mass = 0.0;
  Eigen::Vector3d com = Eigen::Vector3d::Zero();
  Eigen::Matrix3d inertia = Eigen::Matrix3d::Identity();
  Eigen::VectorXd position; // copied generalized state; empty when not copied
  Eigen::VectorXd velocity;
  std::optional<CollisionShape> collisionShape;
};

} // namespace

//==============================================================================
Multibody buildMultibodyFromSkeleton(
    World& world,
    const dynamics::Skeleton& skeleton,
    const SkeletonToMultibodyOptions& options)
{
  const std::size_t bodyCount = skeleton.getNumBodyNodes();

  // Plan pass: resolve every link from the skeleton alone and reject anything
  // unrepresentable before touching the World, so a rejected load leaves the
  // World unchanged. Bodies are visited in skeleton index order; a parent
  // always precedes its children, which also makes the experimental
  // joint-construction order match the legacy DOF ordering.
  //
  // The experimental joint relative transform mirrors the legacy one,
  //   child_in_parent = transformToParent * jointMotion(q) *
  //   transformFromParent,
  // so each experimental link frame coincides with its legacy body frame: the
  // axis, mass, center of mass, and inertia map across directly, and the joint
  // is placed by transformToParent = A and transformFromParent = C^-1 (A/C the
  // legacy parent/child offsets). This represents offset joints and branching
  // parents without reframing.
  std::vector<LinkPlan> plans;
  plans.reserve(bodyCount);
  for (std::size_t i = 0; i < bodyCount; ++i) {
    const auto* body = skeleton.getBodyNode(i);
    const auto* joint = body->getParentJoint();
    const auto* parentBody = body->getParentBodyNode();

    const MappedJoint mapped = mapJoint(*joint);

    const Eigen::Isometry3d A = joint->getTransformFromParentBodyNode();
    const Eigen::Isometry3d C = joint->getTransformFromChildBodyNode();

    // The ball/free/planar coordinate conventions assume the joint frame is
    // aligned with the parent and child body frames; a rotated parent- or
    // child-side offset would require re-expressing their orientation
    // coordinates, which is not yet implemented. A translational offset is fine
    // (it is carried by transformToParent / transformFromParent).
    if (!mapped.reframable) {
      const bool rotated
          = !A.linear().isApprox(Eigen::Matrix3d::Identity(), kAnchorTolerance)
            || !C.linear().isApprox(
                Eigen::Matrix3d::Identity(), kAnchorTolerance);
      if (rotated) {
        DART_EXPERIMENTAL_THROW_T(
            InvalidOperationException,
            "buildMultibodyFromSkeleton does not yet support a rotated parent- "
            "or child-side offset on the ball/free/planar joint '{}'",
            joint->getName());
      }
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
    plan.joint = joint;
    plan.linkName = body->getName();
    plan.spec.name = joint->getName();
    plan.spec.type = mapped.type;
    plan.spec.axis = mapped.axis;
    plan.spec.axis2 = mapped.axis2;
    plan.spec.transformToParent = A;
    plan.spec.transformFromParent = C.inverse();
    plan.pitch = mapped.pitch;
    plan.mass = mass;
    plan.com = body->getLocalCOM();
    plan.inertia = body->getInertia().getMoment();

    if (options.copyState && mapped.dof > 0) {
      auto [position, velocity] = mapJointState(*joint, mapped);
      plan.position = std::move(position);
      plan.velocity = std::move(velocity);
    }
    if (options.loadCollisionShapes) {
      plan.collisionShape = translateCollisionShape(*body);
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
    Joint parentJoint = link.getParentJoint();

    if (plan.spec.type == JointType::Screw) {
      parentJoint.setPitch(plan.pitch);
    }
    link.setMass(plan.mass);
    link.setCenterOfMass(plan.com);
    link.setInertia(plan.inertia);

    if (plan.position.size() > 0) {
      parentJoint.setPosition(plan.position);
      parentJoint.setVelocity(plan.velocity);
    }
    if (options.copyJointProperties && plan.joint != nullptr
        && (plan.spec.type == JointType::Revolute
            || plan.spec.type == JointType::Prismatic)) {
      copyJointProperties(*plan.joint, parentJoint);
    }
    if (plan.collisionShape.has_value()) {
      link.setCollisionShape(*plan.collisionShape);
    }

    linkOf.emplace(plan.body, link);
  }

  return multibody;
}

//==============================================================================
std::vector<Multibody> buildMultibodiesFromWorld(
    World& world,
    const dart::simulation::World& legacyWorld,
    const SkeletonToMultibodyOptions& options)
{
  const std::size_t count = legacyWorld.getNumSkeletons();
  std::vector<Multibody> multibodies;
  multibodies.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    const auto skeleton = legacyWorld.getSkeleton(i);
    if (!skeleton) {
      continue;
    }
    // Name each multibody after its skeleton (skeleton names are unique within
    // the legacy world); the other options apply to every skeleton.
    SkeletonToMultibodyOptions perSkeleton = options;
    perSkeleton.name.clear();
    multibodies.push_back(
        buildMultibodyFromSkeleton(world, *skeleton, perSkeleton));
  }
  return multibodies;
}

} // namespace dart::simulation::experimental::io
