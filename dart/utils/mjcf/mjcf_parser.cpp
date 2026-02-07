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

#include "dart/utils/mjcf/mjcf_parser.hpp"

#include "dart/collision/All.hpp"
#include "dart/collision/collision_filter.hpp"
#include "dart/common/All.hpp"
#include "dart/common/macros.hpp"
#include "dart/config.hpp"
#include "dart/constraint/All.hpp"
#include "dart/dynamics/All.hpp"
#include "dart/utils/composite_resource_retriever.hpp"
#include "dart/utils/dart_resource_retriever.hpp"
#include "dart/utils/mjcf/detail/actuator.hpp"
#include "dart/utils/mjcf/detail/contact.hpp"
#include "dart/utils/mjcf/detail/mujoco_model.hpp"
#include "dart/utils/mjcf/detail/utils.hpp"
#include "dart/utils/mjcf/detail/worldbody.hpp"
#include "dart/utils/xml_helpers.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <limits>
#include <unordered_set>
#include <vector>

#include <cstddef>

namespace dart {
namespace utils {
namespace MjcfParser {

//=============================================================================
dynamics::BodyNode* getUniqueBodyOrNull(
    const simulation::World& world, std::string_view name);

//==============================================================================
dynamics::BodyNode::Properties createBodyProperties(
    dart::dynamics::SkeletonPtr skel, const detail::Body& mjcfBody)
{
  dynamics::BodyNode::Properties bodyProps;

  // Name
  if (!mjcfBody.getName().empty()) {
    bodyProps.mName = mjcfBody.getName();
  } else {
    bodyProps.mName
        = "Nonamed (" + std::to_string(skel->getNumBodyNodes()) + ")";
  }

  // Inertial
  if (mjcfBody.getNumGeoms() > 0) {
    const detail::Inertial& mjcfInertial = mjcfBody.getInertial();

    bodyProps.mInertia.setMass(mjcfInertial.getMass());
    bodyProps.mInertia.setLocalCOM(
        mjcfInertial.getRelativeTransform().translation());
  }

  return bodyProps;
}

//==============================================================================
void createJointCommonProperties(
    dynamics::Joint::Properties& props,
    const dynamics::BodyNode* parentBodyNode,
    const detail::Body& /*mjcfBody*/,
    const detail::Joint& mjcfJoint)
{
  if (!mjcfJoint.getName().empty()) {
    props.mName = mjcfJoint.getName();
  } else if (parentBodyNode != nullptr) {
    props.mName = parentBodyNode->getName() + "_Joint";
  } else {
    props.mName = "root_Joint";
  }
}

//==============================================================================
dynamics::WeldJoint::Properties createWeldJointProperties(
    const dynamics::BodyNode* parentBodyNode,
    const dynamics::BodyNode::Properties& bodyNodeProps,
    const detail::Body& mjcfBody)
{
  dynamics::WeldJoint::Properties props;

  // Name
  if (parentBodyNode != nullptr) {
    props.mName = "WeldJoint_from_" + parentBodyNode->getName() + "_to_"
                  + bodyNodeProps.mName;
  } else {
    props.mName = "WeldJoint_from_World_to_" + bodyNodeProps.mName;
  }

  // Set transform from parent body frame to joint frame
  const Eigen::Isometry3d& parentBodyToChildBody
      = mjcfBody.getRelativeTransform();
  props.mT_ChildBodyToJoint.setIdentity();
  props.mT_ParentBodyToJoint
      = parentBodyToChildBody * props.mT_ChildBodyToJoint;

  return props;
}

//==============================================================================
dynamics::FreeJoint::Properties createFreeJointProperties(
    const dynamics::BodyNode* parentBodyNode,
    const dynamics::BodyNode::Properties& /*bodyNodeProps*/,
    const detail::Body& mjcfBody,
    const detail::Joint& mjcfJoint)
{
  dynamics::FreeJoint::Properties props;

  // Common joint properties
  createJointCommonProperties(props, parentBodyNode, mjcfBody, mjcfJoint);

  // Set transform from parent body frame to joint frame
  const Eigen::Isometry3d& parentBodyToChildBody
      = mjcfBody.getRelativeTransform();
  props.mT_ChildBodyToJoint.setIdentity();
  props.mT_ParentBodyToJoint
      = parentBodyToChildBody * props.mT_ChildBodyToJoint;

  return props;
}

//==============================================================================
dynamics::BallJoint::Properties createBallJointProperties(
    const dynamics::BodyNode* parentBodyNode,
    const dynamics::BodyNode::Properties& /*bodyNodeProps*/,
    const detail::Body& mjcfBody,
    const detail::Joint& mjcfJoint)
{
  dynamics::BallJoint::Properties props;

  // Common joint properties
  createJointCommonProperties(props, parentBodyNode, mjcfBody, mjcfJoint);

  // Set transform from parent body frame to joint frame
  const Eigen::Isometry3d& parentBodyToChildBody
      = mjcfBody.getRelativeTransform();
  Eigen::Isometry3d childBodyToJoint = Eigen::Isometry3d::Identity();
  childBodyToJoint.translation() = mjcfJoint.getPos();
  props.mT_ChildBodyToJoint = childBodyToJoint;
  props.mT_ParentBodyToJoint
      = parentBodyToChildBody * props.mT_ChildBodyToJoint;

  // Damping
  props.mDampingCoefficients.setConstant(mjcfJoint.getDamping());

  // Spring stiffness
  props.mSpringStiffnesses.setConstant(mjcfJoint.getStiffness());

  // Coulomb friction
  props.mFrictions.setConstant(mjcfJoint.getFrictionLoss());

  // Armature warning
  if (mjcfJoint.getArmature() != 0.0) {
    DART_WARN(
        "[MjcfParser] Joint '{}' has non-zero armature ({}) which is not "
        "mapped to DART (no reflected inertia concept).",
        mjcfJoint.getName(),
        mjcfJoint.getArmature());
  }

  return props;
}

//==============================================================================
dynamics::PrismaticJoint::Properties createPrismaticJointProperties(
    const dynamics::BodyNode* parentBodyNode,
    const dynamics::BodyNode::Properties& /*bodyNodeProps*/,
    const detail::Body& mjcfBody,
    const detail::Joint& mjcfJoint)
{
  dynamics::PrismaticJoint::Properties props;

  // Common joint properties
  createJointCommonProperties(props, parentBodyNode, mjcfBody, mjcfJoint);

  // Axis
  props.mAxis = Eigen::Vector3d::UnitZ();

  // Set transform from parent body frame to joint frame
  const Eigen::Isometry3d& parentBodyToChildBody
      = mjcfBody.getRelativeTransform();
  Eigen::Isometry3d childBodyToJoint = Eigen::Isometry3d::Identity();
  childBodyToJoint.translation() = mjcfJoint.getPos();
  childBodyToJoint.linear() = Eigen::Quaterniond::FromTwoVectors(
                                  Eigen::Vector3d::UnitZ(), mjcfJoint.getAxis())
                                  .toRotationMatrix();
  props.mT_ChildBodyToJoint = childBodyToJoint;
  props.mT_ParentBodyToJoint
      = parentBodyToChildBody * props.mT_ChildBodyToJoint;

  // Limits
  props.mIsPositionLimitEnforced = mjcfJoint.isLimited();
  props.mPositionLowerLimits[0] = mjcfJoint.getRange()[0];
  props.mPositionUpperLimits[0] = mjcfJoint.getRange()[1];

  // Damping
  props.mDampingCoefficients[0] = mjcfJoint.getDamping();

  // Spring rest position
  props.mRestPositions[0] = mjcfJoint.getSpringRef();

  // Spring stiffness
  props.mSpringStiffnesses[0] = mjcfJoint.getStiffness();

  // Coulomb friction
  props.mFrictions[0] = mjcfJoint.getFrictionLoss();

  // Armature warning
  if (mjcfJoint.getArmature() != 0.0) {
    DART_WARN(
        "[MjcfParser] Joint '{}' has non-zero armature ({}) which is not "
        "mapped to DART (no reflected inertia concept).",
        mjcfJoint.getName(),
        mjcfJoint.getArmature());
  }

  return props;
}

//==============================================================================
dynamics::RevoluteJoint::Properties createRevoluteJointProperties(
    const dynamics::BodyNode* parentBodyNode,
    const dynamics::BodyNode::Properties& /*bodyNodeProps*/,
    const detail::Body& mjcfBody,
    const detail::Joint& mjcfJoint)
{
  dynamics::RevoluteJoint::Properties props;

  // Common joint properties
  createJointCommonProperties(props, parentBodyNode, mjcfBody, mjcfJoint);

  // Axis
  props.mAxis = Eigen::Vector3d::UnitZ();

  // Set transform from parent body frame to joint frame
  const Eigen::Isometry3d& parentBodyToChildBody
      = mjcfBody.getRelativeTransform();
  Eigen::Isometry3d childBodyToJoint = Eigen::Isometry3d::Identity();
  childBodyToJoint.translation() = mjcfJoint.getPos();
  childBodyToJoint.linear() = Eigen::Quaterniond::FromTwoVectors(
                                  Eigen::Vector3d::UnitZ(), mjcfJoint.getAxis())
                                  .toRotationMatrix();
  props.mT_ChildBodyToJoint = childBodyToJoint;
  props.mT_ParentBodyToJoint
      = parentBodyToChildBody * props.mT_ChildBodyToJoint;

  // Limits
  props.mIsPositionLimitEnforced = mjcfJoint.isLimited();
  props.mPositionLowerLimits[0] = mjcfJoint.getRange()[0];
  props.mPositionUpperLimits[0] = mjcfJoint.getRange()[1];

  // Damping
  props.mDampingCoefficients[0] = mjcfJoint.getDamping();

  // Spring rest position
  props.mRestPositions[0] = mjcfJoint.getSpringRef();

  // Spring stiffness
  props.mSpringStiffnesses[0] = mjcfJoint.getStiffness();

  // Coulomb friction
  props.mFrictions[0] = mjcfJoint.getFrictionLoss();

  // Armature warning
  if (mjcfJoint.getArmature() != 0.0) {
    DART_WARN(
        "[MjcfParser] Joint '{}' has non-zero armature ({}) which is not "
        "mapped to DART (no reflected inertia concept).",
        mjcfJoint.getName(),
        mjcfJoint.getArmature());
  }

  return props;
}

//==============================================================================
std::pair<dynamics::Joint*, dynamics::BodyNode*>
createJointAndBodyNodePairForMultipleJoints(
    dynamics::SkeletonPtr skel,
    dynamics::BodyNode* parentBodyNode,
    const dynamics::BodyNode::Properties& bodyProps,
    const detail::Body& mjcfBody)
{
  // TODO(JS): This should be replaced with CompositeJoint once supported in
  // DART. As an workaround, we only allow to create predefined multi dofs
  // joints for some supported combination of MJCF joint types.

  if (mjcfBody.getNumJoints() == 2u) {
    //----------------------------------------
    // Try to create predefined 2 DOFs joints
    //----------------------------------------

    if (mjcfBody.getJoint(0).getType() == detail::JointType::SLIDE
        && mjcfBody.getJoint(1).getType() == detail::JointType::SLIDE) {
      const detail::Joint& mjcfJoint0 = mjcfBody.getJoint(0);
      const detail::Joint& mjcfJoint1 = mjcfBody.getJoint(1);

      dynamics::TranslationalJoint2D::Properties props;

      // Common joint properties with the first MJCF joint, which is a hack
      createJointCommonProperties(props, parentBodyNode, mjcfBody, mjcfJoint0);

      // Axes
      props.setArbitraryPlane(mjcfJoint0.getAxis(), mjcfJoint1.getAxis());

      // Set transform from parent body frame to joint frame with the first MJCF
      // joint assuming there is no offset in between joints, which is a hack.
      const Eigen::Isometry3d& parentBodyToChildBody
          = mjcfBody.getRelativeTransform();
      Eigen::Isometry3d childBodyToJoint = Eigen::Isometry3d::Identity();
      childBodyToJoint.translation() = mjcfJoint0.getPos();
      props.mT_ChildBodyToJoint = childBodyToJoint;
      props.mT_ParentBodyToJoint
          = parentBodyToChildBody * props.mT_ChildBodyToJoint;

      // Limits
      props.mIsPositionLimitEnforced
          = mjcfJoint0.isLimited() && mjcfJoint1.isLimited();
      props.mPositionLowerLimits[0] = mjcfJoint0.getRange()[0];
      props.mPositionLowerLimits[1] = mjcfJoint1.getRange()[0];
      props.mPositionUpperLimits[0] = mjcfJoint0.getRange()[1];
      props.mPositionUpperLimits[1] = mjcfJoint1.getRange()[1];

      // Damping
      props.mDampingCoefficients[0] = mjcfJoint0.getDamping();
      props.mDampingCoefficients[1] = mjcfJoint1.getDamping();

      // Spring rest positions
      props.mRestPositions[0] = mjcfJoint0.getSpringRef();
      props.mRestPositions[1] = mjcfJoint1.getSpringRef();

      // Spring stiffness
      props.mSpringStiffnesses[0] = mjcfJoint0.getStiffness();
      props.mSpringStiffnesses[1] = mjcfJoint1.getStiffness();

      // Coulomb friction
      props.mFrictions[0] = mjcfJoint0.getFrictionLoss();
      props.mFrictions[1] = mjcfJoint1.getFrictionLoss();

      // Dof names
      props.mDofNames[0] = mjcfJoint0.getName();
      props.mDofNames[1] = mjcfJoint1.getName();
      props.mPreserveDofNames[0] = true;
      props.mPreserveDofNames[1] = true;

      return skel->createJointAndBodyNodePair<dynamics::TranslationalJoint2D>(
          parentBodyNode, props, bodyProps);
    }
  } else if (mjcfBody.getNumJoints() == 3u) {
    //----------------------------------------
    // Try to create predefined 3 DOFs joints
    //----------------------------------------

    if (mjcfBody.getJoint(0).getType() == detail::JointType::SLIDE
        && mjcfBody.getJoint(1).getType() == detail::JointType::SLIDE
        && mjcfBody.getJoint(2).getType() == detail::JointType::SLIDE) {
      const detail::Joint& mjcfJoint0 = mjcfBody.getJoint(0);
      const detail::Joint& mjcfJoint1 = mjcfBody.getJoint(1);
      const detail::Joint& mjcfJoint2 = mjcfBody.getJoint(2);

      dynamics::TranslationalJoint::Properties props;

      // Common joint properties with the first MJCF joint, which is a hack
      createJointCommonProperties(props, parentBodyNode, mjcfBody, mjcfJoint0);

      // Axes
      // Warning: We assume the axes are X, Y, and Z in order, which can be not
      // true.

      // Set transform from parent body frame to joint frame with the first MJCF
      // joint assuming there is no offset in between joints, which is a hack.
      const Eigen::Isometry3d& parentBodyToChildBody
          = mjcfBody.getRelativeTransform();
      Eigen::Isometry3d childBodyToJoint = Eigen::Isometry3d::Identity();
      childBodyToJoint.translation() = mjcfJoint0.getPos();
      props.mT_ChildBodyToJoint = childBodyToJoint;
      props.mT_ParentBodyToJoint
          = parentBodyToChildBody * props.mT_ChildBodyToJoint;

      // Limits
      props.mIsPositionLimitEnforced
          = mjcfJoint0.isLimited() && mjcfJoint1.isLimited();
      props.mPositionLowerLimits[0] = mjcfJoint0.getRange()[0];
      props.mPositionLowerLimits[1] = mjcfJoint1.getRange()[0];
      props.mPositionLowerLimits[2] = mjcfJoint2.getRange()[0];
      props.mPositionUpperLimits[0] = mjcfJoint0.getRange()[1];
      props.mPositionUpperLimits[1] = mjcfJoint1.getRange()[1];
      props.mPositionUpperLimits[2] = mjcfJoint2.getRange()[1];

      // Damping
      props.mDampingCoefficients[0] = mjcfJoint0.getDamping();
      props.mDampingCoefficients[1] = mjcfJoint1.getDamping();
      props.mDampingCoefficients[2] = mjcfJoint2.getDamping();

      // Spring rest positions
      props.mRestPositions[0] = mjcfJoint0.getSpringRef();
      props.mRestPositions[1] = mjcfJoint1.getSpringRef();
      props.mRestPositions[2] = mjcfJoint2.getSpringRef();

      // Spring stiffness
      props.mSpringStiffnesses[0] = mjcfJoint0.getStiffness();
      props.mSpringStiffnesses[1] = mjcfJoint1.getStiffness();
      props.mSpringStiffnesses[2] = mjcfJoint2.getStiffness();

      // Coulomb friction
      props.mFrictions[0] = mjcfJoint0.getFrictionLoss();
      props.mFrictions[1] = mjcfJoint1.getFrictionLoss();
      props.mFrictions[2] = mjcfJoint2.getFrictionLoss();

      // Dof names
      props.mDofNames[0] = mjcfJoint0.getName();
      props.mDofNames[1] = mjcfJoint1.getName();
      props.mDofNames[2] = mjcfJoint2.getName();
      props.mPreserveDofNames[0] = true;
      props.mPreserveDofNames[1] = true;
      props.mPreserveDofNames[2] = true;

      return skel->createJointAndBodyNodePair<dynamics::TranslationalJoint>(
          parentBodyNode, props, bodyProps);
    }
  }

  DART_ERROR("[MjcfParser] Attempted to create unsupported joint composition.");

  return std::make_pair(nullptr, nullptr);
}

//==============================================================================
template <typename GeomOrSite>
dynamics::ShapePtr createShape(const GeomOrSite& geomOrSite)
{
  dynamics::ShapePtr shape = nullptr;

  switch (geomOrSite.getType()) {
    case detail::GeomType::SPHERE: {
      shape = std::make_shared<dynamics::SphereShape>(
          geomOrSite.getSphereRadius());
      break;
    }
    case detail::GeomType::CAPSULE: {
      shape = std::make_shared<dynamics::CapsuleShape>(
          geomOrSite.getCapsuleRadius(), geomOrSite.getCapsuleLength());
      break;
    }
    case detail::GeomType::ELLIPSOID: {
      // DART takes diameters while MJCF has radii
      shape = std::make_shared<dynamics::EllipsoidShape>(
          geomOrSite.getEllipsoidDiameters());
      break;
    }
    case detail::GeomType::CYLINDER: {
      shape = std::make_shared<dynamics::CylinderShape>(
          geomOrSite.getCylinderRadius(), geomOrSite.getCylinderLength());
      break;
    }
    case detail::GeomType::BOX: {
      // DART takes full-sizes while MJCF has half-sizes
      shape = std::make_shared<dynamics::BoxShape>(geomOrSite.getBoxSize());
      break;
    }
    default: {
      break;
    }
  }

  return shape;
}

//==============================================================================
dynamics::ShapePtr createShape(
    const detail::Geom& geom, const detail::Asset& mjcfAsset)
{
  dynamics::ShapePtr shape = nullptr;

  switch (geom.getType()) {
    case detail::GeomType::PLANE: {
      // TODO(JS): Needs to properly parse PLANE.
      Eigen::Vector3d size;
      size.head<2>() = 2.0 * geom.getPlaneHalfSize();
      size[2] = 0.01; // depth
      shape = std::make_shared<dynamics::BoxShape>(size);
      break;
    }
    case detail::GeomType::HFIELD: {
      DART_ERROR("[MjcfParser] Not implemented for 'HFIELD' geom type.");
      break;
    }
    case detail::GeomType::MESH: {
      const detail::Mesh* mjcfMesh = mjcfAsset.getMesh(geom.getMesh());
      DART_ASSERT(mjcfMesh);
      shape = mjcfMesh->getMeshShape();
      DART_ASSERT(shape);
      break;
    }
    default: {
      return createShape(geom);
    }
  }

  return shape;
}

//==============================================================================
bool createShapeNodes(
    dynamics::BodyNode* bodyNode,
    const detail::Body& mjcfBody,
    const detail::Asset& mjcfAsset)
{
  // Create ShapeNodes for <geom>s
  for (std::size_t i = 0u; i < mjcfBody.getNumGeoms(); ++i) {
    const detail::Geom& geom = mjcfBody.getGeom(i);

    const dynamics::ShapePtr shape = createShape(geom, mjcfAsset);
    if (shape == nullptr) {
      DART_ERROR(
          "[MjcfParser] Failed to create ShapeNode given <geom> in the MJCF "
          "file.");
      return false;
    }

    // Create ShapeNode with the shape created above
    dynamics::ShapeNode* shapeNode = nullptr;
    if (mjcfBody.getMocap()) {
      // Disable collision and dynamics for mocap bodies
      shapeNode = bodyNode->createShapeNodeWith<dynamics::VisualAspect>(
          shape, geom.getName());
    } else {
      shapeNode = bodyNode->createShapeNodeWith<
          dynamics::VisualAspect,
          dynamics::CollisionAspect,
          dynamics::DynamicsAspect>(shape, geom.getName());
    }

    // RGBA
    dynamics::VisualAspect* visualAspect = shapeNode->getVisualAspect();

    // TODO(PR2): Material resolution from <asset><material> is not yet wired.
    // The Geom class would need to parse and expose the 'material' attribute.
    visualAspect->setRGBA(geom.getRGBA());

    // Set relative transform of the ShapeNode
    shapeNode->setRelativeTransform(geom.getRelativeTransform());
  }

  // Create ShapeNodes for <site>s
  for (std::size_t i = 0u; i < mjcfBody.getNumSites(); ++i) {
    const detail::Site& site = mjcfBody.getSite(i);

    const dynamics::ShapePtr shape = createShape(site);
    if (shape == nullptr) {
      DART_ERROR(
          "[MjcfParser] Failed to create ShapeNode given <geom> in the MJCF "
          "file.");
      return false;
    }

    // Create ShapeNode with the shape created above
    dynamics::ShapeNode* shapeNode
        = bodyNode->createShapeNodeWith<dynamics::VisualAspect>(
            shape, "site:" + site.getName());

    // RGBA
    dynamics::VisualAspect* visualAspect = shapeNode->getVisualAspect();

    // TODO(PR2): Material resolution from <asset><material> is not yet wired.
    // The Geom class would need to parse and expose the 'material' attribute.
    visualAspect->setRGBA(site.getRGBA());

    // Set relative transform of the ShapeNode
    shapeNode->setRelativeTransform(site.getRelativeTransform());
  }

  return true;
}

//==============================================================================
bool populateSkeletonRecurse(
    dynamics::SkeletonPtr skel,
    dynamics::BodyNode* parentBodyNode,
    const detail::Body& mjcfBody,
    const detail::Asset& mjcfAsset)
{
  dynamics::BodyNode* bodyNode = nullptr;
  dynamics::Joint* joint = nullptr;

  // Create BodyNode properties
  const dynamics::BodyNode::Properties bodyProps
      = createBodyProperties(skel, mjcfBody);

  // Create BodyNode and Joint based on the joint type
  const auto numJoints = mjcfBody.getNumJoints();
  if (numJoints == 0u) {
    const auto jointProps
        = createWeldJointProperties(parentBodyNode, bodyProps, mjcfBody);
    std::tie(joint, bodyNode)
        = skel->createJointAndBodyNodePair<dynamics::WeldJoint>(
            parentBodyNode, jointProps, bodyProps);
  } else if (numJoints == 1u) {
    const auto& mjcfJoint = mjcfBody.getJoint(0);
    if (mjcfJoint.getType() == detail::JointType::FREE) {
      const auto jointProps = createFreeJointProperties(
          parentBodyNode, bodyProps, mjcfBody, mjcfJoint);
      std::tie(joint, bodyNode)
          = skel->createJointAndBodyNodePair<dynamics::FreeJoint>(
              parentBodyNode, jointProps, bodyProps);
    } else if (mjcfJoint.getType() == detail::JointType::BALL) {
      const auto jointProps = createBallJointProperties(
          parentBodyNode, bodyProps, mjcfBody, mjcfJoint);
      std::tie(joint, bodyNode)
          = skel->createJointAndBodyNodePair<dynamics::BallJoint>(
              parentBodyNode, jointProps, bodyProps);
    } else if (mjcfJoint.getType() == detail::JointType::SLIDE) {
      const auto jointProps = createPrismaticJointProperties(
          parentBodyNode, bodyProps, mjcfBody, mjcfJoint);
      std::tie(joint, bodyNode)
          = skel->createJointAndBodyNodePair<dynamics::PrismaticJoint>(
              parentBodyNode, jointProps, bodyProps);
    } else if (mjcfJoint.getType() == detail::JointType::HINGE) {
      const auto jointProps = createRevoluteJointProperties(
          parentBodyNode, bodyProps, mjcfBody, mjcfJoint);
      std::tie(joint, bodyNode)
          = skel->createJointAndBodyNodePair<dynamics::RevoluteJoint>(
              parentBodyNode, jointProps, bodyProps);
    }
  } else {
    std::tie(joint, bodyNode) = createJointAndBodyNodePairForMultipleJoints(
        skel, parentBodyNode, bodyProps, mjcfBody);
  }

  if (bodyNode == nullptr || joint == nullptr) {
    return false;
  }

  DART_ASSERT(bodyNode != nullptr);
  DART_ASSERT(joint != nullptr);

  // Create ShapeNodes for the current BodyNode
  if (!createShapeNodes(bodyNode, mjcfBody, mjcfAsset)) {
    return false;
  }

  for (auto i = 0u; i < mjcfBody.getNumChildBodies(); ++i) {
    if (!populateSkeletonRecurse(
            skel, bodyNode, mjcfBody.getChildBody(i), mjcfAsset)) {
      return false;
    }
  }

  return true;
}

//==============================================================================
dynamics::SkeletonPtr createSkeleton(
    const detail::Body& mjcfBody, const detail::Asset& mjcfAsset)
{
  dynamics::SkeletonPtr skel = dynamics::Skeleton::create();

  const bool success
      = populateSkeletonRecurse(skel, nullptr, mjcfBody, mjcfAsset);
  if (!success) {
    const std::string bodyName
        = mjcfBody.getName().empty() ? "(noname)" : mjcfBody.getName();
    DART_ERROR(
        "[MjcfParser] Failed to create Skeleton from Body '{}'.", bodyName);
    return nullptr;
  }

  return skel;
}

//==============================================================================
simulation::WorldPtr createWorld(
    const detail::MujocoModel& mujoco, const Options& options)
{
  simulation::WorldPtr world = simulation::World::create();
  world->setName(mujoco.getModel());

  // Wire MJCF <option> values into DART World
  const detail::Option& option = mujoco.getOption();
  world->setTimeStep(option.getTimestep());
  world->setGravity(option.getGravity());
  // Note: Other Option values (solver, collision, cone) don't have direct DART
  // World API mappings and are intentionally not wired.

  const detail::Asset& mjcfAsset = mujoco.getAsset();
  const detail::Worldbody& mjcfWorldbody = mujoco.getWorldbody();

  // Parse root <body> elements
  for (std::size_t i = 0; i < mjcfWorldbody.getNumRootBodies(); ++i) {
    const detail::Body& mjcfRootBody = mjcfWorldbody.getRootBody(i);
    const dynamics::SkeletonPtr& skel = createSkeleton(mjcfRootBody, mjcfAsset);

    if (skel == nullptr) {
      DART_ERROR(
          "[MjcfParser] Failed to parse a Skeleton. Stop parsing the rest of "
          "elements.");
      return nullptr;
    }

    // MuJoCo doesn't have a concept that corresponds to DART Skeleton so no
    // name for Skeleton. We use the name of the root body instead. The root
    // body name will be unique anyway because all the MuJoCo body name are
    // unique in the same <worldbody>.
    skel->setName(skel->getRootBodyNode()->getName());

    // Skeleton name should be unique in the World.
    DART_WARN_IF(
        world->hasSkeleton(skel->getName()),
        "[MjcfParser] World '{}' already contains skeleton has the same name "
        "'{}', which is an error. Please report this error.",
        world->getName(),
        skel->getName());

    world->addSkeleton(skel);
  }

  // Map MJCF <actuator> entries to DART Joint APIs
  const detail::Actuator& actuator = mujoco.getActuator();
  std::unordered_set<std::string> actuatedJoints;
  for (std::size_t i = 0; i < actuator.getNumEntries(); ++i) {
    const detail::Actuator::Entry& entry = actuator.getEntry(i);

    if (entry.mJoint.empty()) {
      DART_WARN(
          "[MjcfParser] Actuator '{}' has no joint specified, skipping.",
          entry.mName);
      continue;
    }

    if (!actuatedJoints.insert(entry.mJoint).second) {
      DART_WARN(
          "[MjcfParser] Multiple actuators target joint '{}'. Actuator '{}' "
          "overwrites the previously applied actuator type and force limits.",
          entry.mJoint,
          entry.mName);
    }

    // Find the DART Joint by name across all skeletons
    dynamics::Joint* dartJoint = nullptr;
    for (std::size_t s = 0; s < world->getNumSkeletons(); ++s) {
      auto skel = world->getSkeleton(s);
      dartJoint = skel->getJoint(entry.mJoint);
      if (dartJoint != nullptr) {
        break;
      }
    }

    if (dartJoint == nullptr) {
      DART_WARN(
          "[MjcfParser] Actuator '{}' references joint '{}' which was not "
          "found in any skeleton.",
          entry.mName,
          entry.mJoint);
      continue;
    }

    switch (entry.mType) {
      case detail::ActuatorType::MOTOR:
        dartJoint->setActuatorType(dynamics::Joint::FORCE);
        break;
      case detail::ActuatorType::POSITION:
        dartJoint->setActuatorType(dynamics::Joint::SERVO);
        break;
      case detail::ActuatorType::VELOCITY:
        dartJoint->setActuatorType(dynamics::Joint::VELOCITY);
        break;
      case detail::ActuatorType::GENERAL:
        // General actuator — default to FORCE, log for transparency
        dartJoint->setActuatorType(dynamics::Joint::FORCE);
        DART_WARN(
            "[MjcfParser] Actuator '{}' uses <general> type which is mapped "
            "to FORCE. gainprm/biasprm are not fully supported.",
            entry.mName);
        break;
    }

    const std::size_t numDofs = dartJoint->getNumDofs();
    if (entry.mForceLimited) {
      for (std::size_t d = 0; d < numDofs; ++d) {
        const double scaledLo = entry.mForceRange[0] * entry.mGear[0];
        const double scaledHi = entry.mForceRange[1] * entry.mGear[0];
        dartJoint->setForceLowerLimit(d, std::min(scaledLo, scaledHi));
        dartJoint->setForceUpperLimit(d, std::max(scaledLo, scaledHi));
      }
    } else {
      for (std::size_t d = 0; d < numDofs; ++d) {
        dartJoint->setForceLowerLimit(
            d, -std::numeric_limits<double>::infinity());
        dartJoint->setForceUpperLimit(
            d, std::numeric_limits<double>::infinity());
      }
    }
  }

  // Map MJCF <contact><exclude> entries to DART collision filtering
  const detail::Contact& contact = mujoco.getContact();
  if (contact.getNumExcludes() > 0) {
    auto bodyFilter = std::make_shared<collision::BodyNodeCollisionFilter>();
    for (std::size_t i = 0; i < contact.getNumExcludes(); ++i) {
      const detail::Contact::Exclude& exclude = contact.getExclude(i);

      dynamics::BodyNode* body1 = getUniqueBodyOrNull(*world, exclude.mBody1);
      dynamics::BodyNode* body2 = getUniqueBodyOrNull(*world, exclude.mBody2);

      if (body1 != nullptr && body2 != nullptr) {
        bodyFilter->addBodyNodePairToBlackList(body1, body2);
      } else {
        DART_WARN(
            "[MjcfParser] <contact><exclude> references body '{}' or '{}' "
            "which was not found.",
            exclude.mBody1,
            exclude.mBody2);
      }
    }

    world->getConstraintSolver()->getCollisionOption().collisionFilter
        = bodyFilter;
  }

  // Note: <contact><pair> elements are not yet mapped to DART
  // (would require per-pair friction/condim which DART doesn't support)

  // Parse root <geom> elements
  for (std::size_t i = 0; i < mjcfWorldbody.getNumGeoms(); ++i) {
    const detail::Geom& mjcfGeom = mjcfWorldbody.getGeom(i);

    dynamics::SkeletonPtr skel = dynamics::Skeleton::create();

    dynamics::BodyNode* body;
    dynamics::WeldJoint* joint;
    std::tie(joint, body)
        = skel->createJointAndBodyNodePair<dynamics::WeldJoint>();

    if (!mjcfGeom.getName().empty()) {
      skel->setName(options.mGeomSkeletonNamePrefix + mjcfGeom.getName());
    } else {
      skel->setName(options.mGeomSkeletonNamePrefix);
    }

    joint->setTransformFromParentBodyNode(mjcfGeom.getRelativeTransform());
    dynamics::ShapePtr shape = createShape(mjcfGeom, mjcfAsset);
    dynamics::ShapeNode* shapeNode = body->createShapeNodeWith<
        dynamics::VisualAspect,
        dynamics::CollisionAspect,
        dynamics::DynamicsAspect>(shape);
    dynamics::VisualAspect* visualAspect = shapeNode->getVisualAspect();

    // TODO(PR2): Material resolution from <asset><material> is not yet wired.
    visualAspect->setRGBA(mjcfGeom.getRGBA());

    // Root <geom>s are meant to be kinematic objects.
    skel->setMobile(false);

    world->addSkeleton(skel);
  }

  // Parse root <site> elements
  for (std::size_t i = 0; i < mjcfWorldbody.getNumSites(); ++i) {
    const detail::Site& mjcfSite = mjcfWorldbody.getSite(i);

    dynamics::SkeletonPtr skel = dynamics::Skeleton::create();

    dynamics::BodyNode* body;
    dynamics::WeldJoint* joint;
    std::tie(joint, body)
        = skel->createJointAndBodyNodePair<dynamics::WeldJoint>();

    if (!mjcfSite.getName().empty()) {
      skel->setName(options.mSiteSkeletonNamePrefix + mjcfSite.getName());
    } else {
      skel->setName(options.mSiteSkeletonNamePrefix);
    }

    joint->setTransformFromParentBodyNode(mjcfSite.getRelativeTransform());
    dynamics::ShapePtr shape = createShape(mjcfSite);
    dynamics::ShapeNode* shapeNode
        = body->createShapeNodeWith<dynamics::VisualAspect>(shape);
    dynamics::VisualAspect* visualAspect = shapeNode->getVisualAspect();

    // TODO(PR2): Material resolution from <asset><material> is not yet wired.
    visualAspect->setRGBA(mjcfSite.getRGBA());

    // Root <site>s are meant to be kinematic objects.
    skel->setMobile(false);

    world->addSkeleton(skel);
  }

  return world;
}

//==============================================================================
Options::Options(
    const common::ResourceRetrieverPtr& retrieverOrNullptr,
    std::string_view geomSkeletonNamePrefix,
    std::string_view siteSkeletonNamePrefix)
  : mRetriever(retrieverOrNullptr),
    mGeomSkeletonNamePrefix(std::string(geomSkeletonNamePrefix)),
    mSiteSkeletonNamePrefix(std::string(siteSkeletonNamePrefix))
{
  // Do nothing
}

//==============================================================================
dynamics::BodyNode* getUniqueBodyOrNull(
    const simulation::World& world, std::string_view name)
{
  const auto& bodyNodes = detail::getBodyNodes(world, name);

  if (bodyNodes.empty()) {
    return nullptr;
  } else if (bodyNodes.size() != 1u) {
    DART_ERROR(
        "[MjcfParser] Found multiple BodyNodes have the same name. Please "
        "report this bug.");
    return nullptr;
  }

  return bodyNodes[0];
}

//==============================================================================
simulation::WorldPtr readWorld(const common::Uri& uri, const Options& options)
{
  auto mujoco = detail::MujocoModel();
  const detail::Errors errors = mujoco.read(uri, options.mRetriever);
  if (!errors.empty()) {
    DART_ERROR(
        "{}",
        "[MjcfParser] Failed to parse MJCF file for the following "
        "reason(s):\n");
    for (const auto& error : errors) {
      DART_ERROR(" - {}", error.getMessage());
    }

    return nullptr;
  }

  auto world = createWorld(mujoco, options);

  // Parse <equality> element
  const detail::Equality& equality = mujoco.getEquality();
  for (std::size_t i = 0; i < equality.getNumWelds(); ++i) {
    const detail::Weld& weld = equality.getWeld(i);

    // Body1
    dynamics::BodyNode* body1 = getUniqueBodyOrNull(*world, weld.getBody1());
    if (body1 == nullptr) {
      DART_ERROR(
          "[MjcfParser] Failed to find BodyNode by name '{}' Not adding weld "
          "joint constraint.",
          weld.getBody1());
      continue;
    }

    // Body2
    dynamics::BodyNode* body2 = nullptr;
    const std::string& bodyName2 = weld.getBody2();
    if (!bodyName2.empty()) {
      body2 = getUniqueBodyOrNull(*world, bodyName2);
      if (body2 == nullptr) {
        DART_ERROR(
            "[MjcfParser] Failed to find BodyNode by name '{}' Not adding weld "
            "joint constraint.",
            bodyName2);
        continue;
      }

      if (body1 == body2) {
        DART_ERROR("[MjcfParser] Not allowed to set <weld> with same bodies.");
        continue;
      }
    }

    auto weldJointConstraint
        = std::make_shared<constraint::WeldJointConstraint>(body1, body2);
    if (weld.getRelativeTransform()) {
      weldJointConstraint->setRelativeTransform(
          (*weld.getRelativeTransform()).inverse());
    }

    world->getConstraintSolver()->addConstraint(std::move(weldJointConstraint));
  }

  return world;
}

} // namespace MjcfParser
} // namespace utils
} // namespace dart
