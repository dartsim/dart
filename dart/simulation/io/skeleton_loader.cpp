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
 *     copyright notice and this list of conditions in the documentation
 *     and/or other materials provided with the distribution.
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

#include "dart/simulation/io/skeleton_loader.hpp"

#include "dart/simulation/common/exceptions.hpp"
#include "dart/simulation/comps/link.hpp"
#include "dart/simulation/detail/entity_conversion.hpp"
#include "dart/simulation/detail/world_registry_access.hpp"
#include "dart/simulation/multibody/joint.hpp"
#include "dart/simulation/multibody/link.hpp"
#include "dart/simulation/world.hpp"

#include <dart/dynamics/ball_joint.hpp>
#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/capsule_shape.hpp>
#include <dart/dynamics/cylinder_shape.hpp>
#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/inertia.hpp>
#include <dart/dynamics/joint.hpp>
#include <dart/dynamics/mesh_shape.hpp>
#include <dart/dynamics/planar_joint.hpp>
#include <dart/dynamics/prismatic_joint.hpp>
#include <dart/dynamics/revolute_joint.hpp>
#include <dart/dynamics/screw_joint.hpp>
#include <dart/dynamics/shape_frame.hpp>
#include <dart/dynamics/shape_node.hpp>
#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/sphere_shape.hpp>
#include <dart/dynamics/universal_joint.hpp>
#include <dart/dynamics/weld_joint.hpp>

#include <dart/math/tri_mesh.hpp>

#include <dart/io/read.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <format>
#include <functional>
#include <memory>
#include <numbers>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_set>
#include <vector>

#include <cstddef>

namespace dart::simulation::io {

namespace detail {

class SkeletonLoaderWorldAccess
{
public:
  static std::size_t multibodyCounter(const World& world)
  {
    return world.m_multibodyCounter;
  }

  static void copySkeletonLoaderCounters(World& target, const World& source)
  {
    target.m_multibodyCounter = source.m_multibodyCounter;
    target.m_linkCounter = source.m_linkCounter;
    target.m_jointCounter = source.m_jointCounter;
  }
};

} // namespace detail

namespace {

Multibody addSkeletonUnchecked(
    World& world,
    const dynamics::Skeleton& skeleton,
    const SkeletonLoadOptions& options);

std::string uniqueName(
    std::string_view base, const std::unordered_set<std::string>& usedNames)
{
  std::string candidate(base);
  std::size_t suffix = 1u;
  while (usedNames.contains(candidate)) {
    candidate = std::string(base) + "_" + std::to_string(++suffix);
  }
  return candidate;
}

void validateCoordinateChart(const dynamics::Joint& joint)
{
  if (const auto* ball = dynamic_cast<const dynamics::BallJoint*>(&joint)) {
    DART_SIMULATION_THROW_T_IF(
        ball->getCoordinateChart()
            != dynamics::BallJoint::CoordinateChart::EXP_MAP,
        InvalidArgumentException,
        "Cannot translate legacy BallJoint '{}' with a non-exp-map "
        "coordinate chart",
        joint.getName());
  }
  if (const auto* free = dynamic_cast<const dynamics::FreeJoint*>(&joint)) {
    DART_SIMULATION_THROW_T_IF(
        free->getCoordinateChart()
            != dynamics::FreeJoint::CoordinateChart::EXP_MAP,
        InvalidArgumentException,
        "Cannot translate legacy FreeJoint '{}' with a non-exp-map "
        "coordinate chart",
        joint.getName());
  }
}

JointType mapJointType(const dynamics::Joint& joint)
{
  if (dynamic_cast<const dynamics::WeldJoint*>(&joint) != nullptr) {
    return JointType::Fixed;
  }
  if (dynamic_cast<const dynamics::RevoluteJoint*>(&joint) != nullptr) {
    return JointType::Revolute;
  }
  if (dynamic_cast<const dynamics::PrismaticJoint*>(&joint) != nullptr) {
    return JointType::Prismatic;
  }
  if (dynamic_cast<const dynamics::ScrewJoint*>(&joint) != nullptr) {
    return JointType::Screw;
  }
  if (dynamic_cast<const dynamics::UniversalJoint*>(&joint) != nullptr) {
    return JointType::Universal;
  }
  if (dynamic_cast<const dynamics::BallJoint*>(&joint) != nullptr) {
    return JointType::Spherical;
  }
  if (dynamic_cast<const dynamics::PlanarJoint*>(&joint) != nullptr) {
    return JointType::Planar;
  }
  if (dynamic_cast<const dynamics::FreeJoint*>(&joint) != nullptr) {
    return JointType::Floating;
  }

  DART_SIMULATION_THROW_T(
      InvalidArgumentException,
      "Cannot translate legacy joint '{}' of type '{}' into an experimental "
      "Multibody yet",
      joint.getName(),
      joint.getType());
}

Eigen::Vector3d mapJointAxis(const dynamics::Joint& joint)
{
  if (const auto* revolute
      = dynamic_cast<const dynamics::RevoluteJoint*>(&joint)) {
    return revolute->getAxis();
  }
  if (const auto* prismatic
      = dynamic_cast<const dynamics::PrismaticJoint*>(&joint)) {
    return prismatic->getAxis();
  }
  if (const auto* screw = dynamic_cast<const dynamics::ScrewJoint*>(&joint)) {
    return screw->getAxis();
  }
  if (const auto* universal
      = dynamic_cast<const dynamics::UniversalJoint*>(&joint)) {
    return universal->getAxis1();
  }
  if (const auto* planar = dynamic_cast<const dynamics::PlanarJoint*>(&joint)) {
    return planar->getRotationalAxis();
  }
  return Eigen::Vector3d::UnitZ();
}

Eigen::Vector3d mapJointAxis2(const dynamics::Joint& joint)
{
  if (const auto* universal
      = dynamic_cast<const dynamics::UniversalJoint*>(&joint)) {
    return universal->getAxis2();
  }
  if (const auto* planar = dynamic_cast<const dynamics::PlanarJoint*>(&joint)) {
    return planar->getTranslationalAxis1();
  }
  return Eigen::Vector3d::UnitX();
}

void validateJointAxesLoadSupported(const dynamics::Joint& joint)
{
  const JointType jointType = mapJointType(joint);
  const Eigen::Vector3d axis = mapJointAxis(joint);
  const Eigen::Vector3d axis2 = mapJointAxis2(joint);

  DART_SIMULATION_THROW_T_IF(
      !axis.allFinite(),
      InvalidArgumentException,
      "Cannot translate legacy Joint '{}' because its axis contains non-finite "
      "values",
      joint.getName());
  const double axisNorm = axis.norm();
  DART_SIMULATION_THROW_T_IF(
      axisNorm <= 1e-9,
      InvalidArgumentException,
      "Cannot translate legacy Joint '{}' because its axis is zero",
      joint.getName());

  DART_SIMULATION_THROW_T_IF(
      !axis2.allFinite(),
      InvalidArgumentException,
      "Cannot translate legacy Joint '{}' because its axis2 contains "
      "non-finite "
      "values",
      joint.getName());
  const double axis2Norm = axis2.norm();
  DART_SIMULATION_THROW_T_IF(
      axis2Norm <= 1e-9,
      InvalidArgumentException,
      "Cannot translate legacy Joint '{}' because its axis2 is zero",
      joint.getName());

  if (jointType == JointType::Planar || jointType == JointType::Universal) {
    DART_SIMULATION_THROW_T_IF(
        (axis / axisNorm).cross(axis2 / axis2Norm).norm() <= 1e-6,
        InvalidArgumentException,
        "Cannot translate legacy Joint '{}' because its axis must not be "
        "parallel to axis2 for planar and universal joints",
        joint.getName());
  }
}

double mapJointPitch(const dynamics::Joint& joint)
{
  if (const auto* screw = dynamic_cast<const dynamics::ScrewJoint*>(&joint)) {
    return screw->getPitch() / (2.0 * std::numbers::pi);
  }
  return 0.0;
}

void validateJointLoadSupported(const dynamics::Joint& joint)
{
  DART_SIMULATION_THROW_T_IF(
      joint.getActuatorType() == dynamics::Joint::MIMIC,
      InvalidArgumentException,
      "Cannot translate legacy mimic actuator on Joint '{}' into an "
      "experimental Joint yet",
      joint.getName());

  validateCoordinateChart(joint);
  validateJointAxesLoadSupported(joint);
}

ActuatorType mapActuatorType(const dynamics::Joint& joint)
{
  validateJointLoadSupported(joint);

  switch (joint.getActuatorType()) {
    case dynamics::Joint::FORCE:
      return ActuatorType::Force;
    case dynamics::Joint::PASSIVE:
      return ActuatorType::Passive;
    case dynamics::Joint::SERVO:
      return ActuatorType::Servo;
    case dynamics::Joint::VELOCITY:
      return ActuatorType::Velocity;
    case dynamics::Joint::ACCELERATION:
      return ActuatorType::Acceleration;
    case dynamics::Joint::LOCKED:
      return ActuatorType::Locked;
    case dynamics::Joint::MIMIC:
      break;
  }

  return ActuatorType::Force;
}

std::vector<const dynamics::ShapeNode*> getCollidableShapeNodes(
    const dynamics::BodyNode& source)
{
  std::vector<const dynamics::ShapeNode*> shapeNodes;
  source.eachShapeNodeWith<dynamics::CollisionAspect>(
      [&](const dynamics::ShapeNode* shapeNode) {
        const dynamics::CollisionAspect* collisionAspect
            = shapeNode->getCollisionAspect();
        if (collisionAspect != nullptr && collisionAspect->isCollidable()) {
          shapeNodes.push_back(shapeNode);
        }
      });
  return shapeNodes;
}

void validateMeshCollisionShape(
    const std::shared_ptr<math::TriMesh<double>>& mesh,
    const Eigen::Vector3d& scale,
    const dynamics::BodyNode& bodyNode)
{
  DART_SIMULATION_THROW_T_IF(
      !mesh,
      InvalidArgumentException,
      "Cannot translate null mesh collision shape on BodyNode '{}'",
      bodyNode.getName());
  DART_SIMULATION_THROW_T_IF(
      !scale.allFinite(),
      InvalidArgumentException,
      "Cannot translate mesh collision shape with non-finite scale on "
      "BodyNode '{}'",
      bodyNode.getName());

  DART_SIMULATION_THROW_T_IF(
      mesh->getVertices().empty() || mesh->getTriangles().empty(),
      InvalidArgumentException,
      "Cannot translate empty mesh collision shape on BodyNode '{}'",
      bodyNode.getName());
}

void validateCollisionShapeType(
    const dynamics::Shape& shape, const dynamics::BodyNode& bodyNode)
{
  if (dynamic_cast<const dynamics::BoxShape*>(&shape) != nullptr
      || dynamic_cast<const dynamics::SphereShape*>(&shape) != nullptr
      || dynamic_cast<const dynamics::CapsuleShape*>(&shape) != nullptr
      || dynamic_cast<const dynamics::CylinderShape*>(&shape) != nullptr) {
    return;
  }

  if (const auto* mesh = dynamic_cast<const dynamics::MeshShape*>(&shape)) {
    validateMeshCollisionShape(mesh->getTriMesh(), mesh->getScale(), bodyNode);
    return;
  }

  DART_SIMULATION_THROW_T(
      InvalidArgumentException,
      "Cannot translate legacy collision shape type '{}' on BodyNode '{}' into "
      "an experimental CollisionShape yet",
      shape.getType(),
      bodyNode.getName());
}

const dynamics::ShapeNode* getValidatedCollisionShapeNode(
    const dynamics::BodyNode& source)
{
  if (!source.isCollidable()) {
    return nullptr;
  }

  const std::vector<const dynamics::ShapeNode*> shapeNodes
      = getCollidableShapeNodes(source);

  if (shapeNodes.empty()) {
    return nullptr;
  }

  DART_SIMULATION_THROW_T_IF(
      shapeNodes.size() > 1u,
      InvalidArgumentException,
      "Cannot translate BodyNode '{}' with multiple collidable collision "
      "shapes into an experimental Link yet",
      source.getName());

  const dynamics::ShapeNode& shapeNode = *shapeNodes.front();
  const auto shape = shapeNode.getShape();
  DART_SIMULATION_THROW_T_IF(
      shape == nullptr,
      InvalidArgumentException,
      "Cannot translate null collision shape on BodyNode '{}'",
      source.getName());

  const Eigen::Isometry3d identity = Eigen::Isometry3d::Identity();
  DART_SIMULATION_THROW_T_IF(
      !shapeNode.getRelativeTransform().matrix().isApprox(
          identity.matrix(), 1e-12),
      InvalidArgumentException,
      "Cannot translate non-identity collision shape offset on BodyNode '{}' "
      "into an experimental Link yet",
      source.getName());

  validateCollisionShapeType(*shape, source);
  return &shapeNode;
}

void validateSkeletonLoadSupported(const dynamics::Skeleton& skeleton)
{
  for (std::size_t i = 0; i < skeleton.getNumJoints(); ++i) {
    const dynamics::Joint* joint = skeleton.getJoint(i);
    DART_SIMULATION_THROW_T_IF(
        joint == nullptr,
        InvalidArgumentException,
        "Cannot translate legacy Skeleton '{}' because joint index {} is null",
        skeleton.getName(),
        i);
    validateJointLoadSupported(*joint);
  }

  for (std::size_t i = 0; i < skeleton.getNumBodyNodes(); ++i) {
    const dynamics::BodyNode* bodyNode = skeleton.getBodyNode(i);
    DART_SIMULATION_THROW_T_IF(
        bodyNode == nullptr,
        InvalidArgumentException,
        "Cannot translate legacy Skeleton '{}' because body index {} is null",
        skeleton.getName(),
        i);
    (void)getValidatedCollisionShapeNode(*bodyNode);
  }
}

std::string resolveWorldMultibodyName(
    const World& targetWorld,
    const dynamics::Skeleton& skeleton,
    std::size_t& generatedMultibodyCounter)
{
  std::string resolvedName = skeleton.getName();
  if (resolvedName.empty()) {
    do {
      resolvedName
          = std::format("multibody_{:03d}", ++generatedMultibodyCounter);
    } while (targetWorld.hasMultibody(resolvedName));
  }

  DART_SIMULATION_THROW_T_IF(
      targetWorld.hasMultibody(resolvedName),
      InvalidArgumentException,
      "Cannot translate legacy Skeleton to Multibody '{}' because "
      "the target World already contains a Multibody with that name",
      resolvedName);

  return resolvedName;
}

void validateSkeletonLoadSupportedForTarget(
    const World& targetWorld,
    const dynamics::Skeleton& skeleton,
    const SkeletonLoadOptions& options)
{
  validateSkeletonLoadSupported(skeleton);

  std::size_t generatedMultibodyCounter
      = detail::SkeletonLoaderWorldAccess::multibodyCounter(targetWorld);
  (void)resolveWorldMultibodyName(
      targetWorld, skeleton, generatedMultibodyCounter);

  World scratchWorld;
  detail::SkeletonLoaderWorldAccess::copySkeletonLoaderCounters(
      scratchWorld, targetWorld);
  (void)addSkeletonUnchecked(scratchWorld, skeleton, options);
}

Eigen::VectorXd perDofVector(
    const dynamics::Joint& joint,
    const std::function<double(std::size_t)>& getter)
{
  Eigen::VectorXd values(joint.getNumDofs());
  for (std::size_t i = 0; i < joint.getNumDofs(); ++i) {
    values[static_cast<Eigen::Index>(i)] = getter(i);
  }
  return values;
}

Eigen::VectorXd mapJointVector(
    const dynamics::Joint& joint, const Eigen::VectorXd& values)
{
  if (dynamic_cast<const dynamics::FreeJoint*>(&joint) == nullptr) {
    return values;
  }

  DART_SIMULATION_THROW_T_IF(
      values.size() != 6,
      InvalidArgumentException,
      "Expected 6 values while translating FreeJoint '{}', got {}",
      joint.getName(),
      values.size());

  Eigen::VectorXd mapped(6);
  mapped.head<3>() = values.tail<3>();
  mapped.tail<3>() = values.head<3>();
  return mapped;
}

JointSpec makeJointSpec(const dynamics::Joint& joint)
{
  validateCoordinateChart(joint);

  return JointSpec{
      .name = joint.getName(),
      .type = mapJointType(joint),
      .axis = mapJointAxis(joint),
      .axis2 = mapJointAxis2(joint),
      .transformFromParent = joint.getTransformFromChildBodyNode().inverse(),
      .parentAnchor = std::nullopt,
      .childAnchor = std::nullopt,
  };
}

void setParentToJointTransform(Link& link, const Eigen::Isometry3d& transform)
{
  DART_SIMULATION_THROW_T_IF(
      !transform.matrix().allFinite(),
      InvalidArgumentException,
      "Link parent-to-joint transform must contain only finite values");

  World* world = link.getWorld();
  DART_SIMULATION_THROW_T_IF(
      world == nullptr,
      InvalidArgumentException,
      "Cannot set parent-to-joint transform on an invalid loaded Link");

  auto& registry = dart::simulation::detail::registryOf(*world);
  auto& linkModel = registry.get<comps::LinkModel>(
      dart::simulation::detail::toRegistryEntity(link.getEntity()));
  linkModel.transformFromParentToJoint = transform;
}

void copyBodyInertia(const dynamics::BodyNode& source, Link& target)
{
  const dynamics::Inertia& inertia = source.getInertia();
  target.setMass(inertia.getMass());
  target.setInertia(inertia.getMoment());
  target.setCenterOfMass(inertia.getLocalCOM());
}

CollisionShape makeMeshCollisionShape(
    const std::shared_ptr<math::TriMesh<double>>& mesh,
    const Eigen::Vector3d& scale,
    const dynamics::BodyNode& bodyNode)
{
  validateMeshCollisionShape(mesh, scale, bodyNode);

  const auto& sourceVertices = mesh->getVertices();
  const auto& sourceTriangles = mesh->getTriangles();

  std::vector<Eigen::Vector3d> vertices;
  vertices.reserve(sourceVertices.size());
  for (const Eigen::Vector3d& vertex : sourceVertices) {
    vertices.push_back(scale.cwiseProduct(vertex));
  }

  std::vector<Eigen::Vector3i> triangles;
  triangles.reserve(sourceTriangles.size());
  for (const auto& triangle : sourceTriangles) {
    triangles.emplace_back(
        static_cast<int>(triangle[0]),
        static_cast<int>(triangle[1]),
        static_cast<int>(triangle[2]));
  }

  return CollisionShape::makeMesh(std::move(vertices), std::move(triangles));
}

void copyCollisionShape(const dynamics::BodyNode& source, Link& target)
{
  const dynamics::ShapeNode* shapeNode = getValidatedCollisionShapeNode(source);
  if (shapeNode == nullptr) {
    return;
  }

  const auto shape = shapeNode->getShape();

  if (const auto* box = dynamic_cast<const dynamics::BoxShape*>(shape.get())) {
    target.setCollisionShape(CollisionShape::makeBox(0.5 * box->getSize()));
    return;
  }
  if (const auto* sphere
      = dynamic_cast<const dynamics::SphereShape*>(shape.get())) {
    target.setCollisionShape(CollisionShape::makeSphere(sphere->getRadius()));
    return;
  }
  if (const auto* capsule
      = dynamic_cast<const dynamics::CapsuleShape*>(shape.get())) {
    target.setCollisionShape(
        CollisionShape::makeCapsule(
            capsule->getRadius(), 0.5 * capsule->getHeight()));
    return;
  }
  if (const auto* cylinder
      = dynamic_cast<const dynamics::CylinderShape*>(shape.get())) {
    target.setCollisionShape(
        CollisionShape::makeCylinder(
            cylinder->getRadius(), 0.5 * cylinder->getHeight()));
    return;
  }
  if (const auto* mesh
      = dynamic_cast<const dynamics::MeshShape*>(shape.get())) {
    target.setCollisionShape(
        makeMeshCollisionShape(mesh->getTriMesh(), mesh->getScale(), source));
    return;
  }

  DART_SIMULATION_THROW_T(
      InvalidArgumentException,
      "Cannot translate legacy collision shape type '{}' on BodyNode '{}' into "
      "an experimental CollisionShape yet",
      shape->getType(),
      source.getName());
}

void copyJointState(const dynamics::Joint& source, Joint& target)
{
  target.setActuatorType(mapActuatorType(source));
  if (dynamic_cast<const dynamics::ScrewJoint*>(&source) != nullptr) {
    target.setPitch(mapJointPitch(source));
  }
  target.setPosition(mapJointVector(source, source.getPositions()));
  target.setVelocity(mapJointVector(source, source.getVelocities()));
  const Eigen::VectorXd sourceForce
      = source.getActuatorType() == dynamics::Joint::FORCE
            ? source.getCommands()
            : source.getForces();
  target.setForce(mapJointVector(source, sourceForce));
  target.setPositionLimits(
      mapJointVector(source, source.getPositionLowerLimits()),
      mapJointVector(source, source.getPositionUpperLimits()));
  target.setVelocityLimits(
      mapJointVector(source, source.getVelocityLowerLimits()),
      mapJointVector(source, source.getVelocityUpperLimits()));
  target.setEffortLimits(
      mapJointVector(source, source.getForceLowerLimits()),
      mapJointVector(source, source.getForceUpperLimits()));
  target.setSpringStiffness(
      mapJointVector(source, perDofVector(source, [&](std::size_t index) {
                       return source.getSpringStiffness(index);
                     })));
  target.setRestPosition(
      mapJointVector(source, perDofVector(source, [&](std::size_t index) {
                       return source.getRestPosition(index);
                     })));
  target.setDampingCoefficient(
      mapJointVector(source, perDofVector(source, [&](std::size_t index) {
                       return source.getDampingCoefficient(index);
                     })));
  target.setCoulombFriction(
      mapJointVector(source, perDofVector(source, [&](std::size_t index) {
                       return source.getCoulombFriction(index);
                     })));

  if (source.getActuatorType() == dynamics::Joint::VELOCITY) {
    target.setCommandVelocity(mapJointVector(source, source.getCommands()));
  }
}

Multibody addSkeletonUnchecked(
    World& world,
    const dynamics::Skeleton& skeleton,
    const SkeletonLoadOptions& options)
{
  validateSkeletonLoadSupported(skeleton);

  Multibody multibody = world.addMultibody(skeleton.getName());

  std::unordered_set<std::string> usedLinkNames;
  for (std::size_t i = 0; i < skeleton.getNumBodyNodes(); ++i) {
    const dynamics::BodyNode* bodyNode = skeleton.getBodyNode(i);
    usedLinkNames.insert(bodyNode->getName());
  }

  const auto addBodyTree = [&](const dynamics::BodyNode& bodyNode,
                               Link* parentLink,
                               const auto& addBodyTreeRef) -> void {
    const dynamics::Joint* parentJoint = bodyNode.getParentJoint();
    DART_SIMULATION_THROW_T_IF(
        parentJoint == nullptr,
        InvalidArgumentException,
        "BodyNode '{}' has no parent joint",
        bodyNode.getName());

    Link link(Entity{}, nullptr);
    if (parentLink == nullptr) {
      const std::string anchorName = uniqueName(
          options.rootAnchorPrefix + bodyNode.getName(), usedLinkNames);
      usedLinkNames.insert(anchorName);
      Link rootAnchor = multibody.addLink(anchorName);
      link = multibody.addLink(
          bodyNode.getName(), rootAnchor, makeJointSpec(*parentJoint));
    } else {
      link = multibody.addLink(
          bodyNode.getName(), *parentLink, makeJointSpec(*parentJoint));
    }
    setParentToJointTransform(
        link, parentJoint->getTransformFromParentBodyNode());

    copyBodyInertia(bodyNode, link);
    copyCollisionShape(bodyNode, link);
    Joint joint = link.getParentJoint();
    copyJointState(*parentJoint, joint);

    for (std::size_t i = 0; i < bodyNode.getNumChildBodyNodes(); ++i) {
      addBodyTreeRef(*bodyNode.getChildBodyNode(i), &link, addBodyTreeRef);
    }
  };

  for (std::size_t i = 0; i < skeleton.getNumTrees(); ++i) {
    addBodyTree(*skeleton.getRootBodyNode(i), nullptr, addBodyTree);
  }

  return multibody;
}

} // namespace

Multibody addSkeleton(
    World& world,
    const dynamics::Skeleton& skeleton,
    const SkeletonLoadOptions& options)
{
  validateSkeletonLoadSupportedForTarget(world, skeleton, options);
  return addSkeletonUnchecked(world, skeleton, options);
}

Multibody addSkeleton(
    World& world, const common::Uri& uri, const SkeletonLoadOptions& options)
{
  return addSkeleton(world, uri, ::dart::io::ReadOptions{}, options);
}

Multibody addSkeleton(
    World& world,
    const common::Uri& uri,
    const ::dart::io::ReadOptions& readOptions,
    const SkeletonLoadOptions& options)
{
  const auto skeleton = ::dart::io::readSkeleton(uri, readOptions);
  DART_SIMULATION_THROW_T_IF(
      !skeleton,
      InvalidArgumentException,
      "Failed to read Skeleton from URI '{}'",
      uri.toString());

  return addSkeleton(world, *skeleton, options);
}

} // namespace dart::simulation::io
