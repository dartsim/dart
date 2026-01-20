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

#include "dart/dynamics/weld_joint.hpp"

#include "dart/common/logging.hpp"
#include "dart/dynamics/body_node.hpp"
#include "dart/dynamics/fixed_frame.hpp"
#include "dart/dynamics/skeleton.hpp"
#include "dart/math/geometry.hpp"
#include "dart/math/helpers.hpp"

#include <string>
#include <vector>

namespace dart {
namespace dynamics {

//==============================================================================
WeldJoint::Properties::Properties(const Joint::Properties& _properties)
  : ZeroDofJoint::Properties(_properties)
{
  // Do nothing
}

//==============================================================================
WeldJoint::~WeldJoint()
{
  // Do nothing
}

//==============================================================================
std::string_view WeldJoint::getType() const
{
  return getStaticType();
}

//==============================================================================
std::string_view WeldJoint::getStaticType()
{
  static constexpr std::string_view name = "WeldJoint";
  return name;
}

//==============================================================================
bool WeldJoint::isCyclic(std::size_t /*_index*/) const
{
  return false;
}

//==============================================================================
WeldJoint::Properties WeldJoint::getWeldJointProperties() const
{
  return getZeroDofJointProperties();
}

//==============================================================================
BodyNode* WeldJoint::merge()
{
  // Before:
  //   parent --[WeldJoint]--> child --[any joints]--> grandchildren...
  // After:
  //   parent absorbs child (inertia, forces, nodes) and grandchildren attach to
  //   parent.
  BodyNode* parent = getParentBodyNode();
  BodyNode* child = getChildBodyNode();

  if (nullptr == parent || nullptr == child) {
    DART_ERROR(
        "[WeldJoint::merge] Merge failed because the joint does not have a "
        "valid parent or child BodyNode.");
    return nullptr;
  }

  const SkeletonPtr& parentSkeleton = parent->getSkeleton();
  const SkeletonPtr& childSkeleton = child->getSkeleton();

  if (!parentSkeleton || parentSkeleton != childSkeleton) {
    DART_ERROR(
        "[WeldJoint::merge] Merge failed because the parent and child "
        "BodyNodes are not in the same Skeleton.");
    return nullptr;
  }

  if (parent == child) {
    DART_ERROR(
        "[WeldJoint::merge] Merge failed because the parent and child "
        "BodyNodes are identical.");
    return nullptr;
  }

  const Eigen::Isometry3d parentToChild = getRelativeTransform();

  // Combine inertial properties in the parent frame.
  const Eigen::Matrix6d parentSpatial = parent->getSpatialInertia();
  const Eigen::Matrix6d childSpatial = child->getSpatialInertia();
  const Eigen::Matrix6d childInParent
      = math::transformInertia(parentToChild, childSpatial);
  parent->setInertia(Inertia(parentSpatial + childInParent));

  // Preserve accumulated external forces.
  const Eigen::Vector6d childExternal
      = math::dAdInvT(parentToChild, child->getExternalForceLocal());
  parent->addExtForce(
      childExternal.tail<3>(), Eigen::Vector3d::Zero(), true, true);
  parent->addExtTorque(childExternal.head<3>(), true);

  // Clone Nodes that belong to the child onto the parent while preserving
  // world-space pose for FixedFrame-derived Nodes.
  const Eigen::Isometry3d worldToParent = parent->getWorldTransform().inverse();
  const auto nodes = child->getNodes();
  for (const Node* node : nodes) {
    const Frame* frame = dynamic_cast<const Frame*>(node);
    Eigen::Isometry3d worldTf = Eigen::Isometry3d::Identity();
    if (frame)
      worldTf = frame->getTransform();

    Node* clone = node->cloneNode(parent);
    clone->attach();

    if (auto* fixed = dynamic_cast<FixedFrame*>(clone)) {
      fixed->setRelativeTransform(worldToParent * worldTf);
    }
  }

  // Reparent grandchildren while keeping their world transforms intact.
  std::vector<BodyNode*> grandchildren;
  grandchildren.reserve(child->getNumChildBodyNodes());
  for (std::size_t i = 0; i < child->getNumChildBodyNodes(); ++i)
    grandchildren.push_back(child->getChildBodyNode(i));

  for (BodyNode* grandchild : grandchildren) {
    Joint* joint = grandchild->getParentJoint();
    const Eigen::Isometry3d updated
        = parentToChild * joint->getTransformFromParentBodyNode();
    joint->setTransformFromParentBodyNode(updated);
    grandchild->moveTo(parent);
  }

  // Remove the child BodyNode (and this WeldJoint) from the Skeleton.
  // Keep the temporary Skeleton alive until we return.
  const auto removedSkeleton = child->remove();
  (void)removedSkeleton;
  return parent;
}

//==============================================================================
void WeldJoint::setTransformFromParentBodyNode(const Eigen::Isometry3d& _T)
{
  Joint::setTransformFromParentBodyNode(_T);

  mT = Joint::mAspectProperties.mT_ParentBodyToJoint
       * Joint::mAspectProperties.mT_ChildBodyToJoint.inverse();
}

//==============================================================================
void WeldJoint::setTransformFromChildBodyNode(const Eigen::Isometry3d& _T)
{
  Joint::setTransformFromChildBodyNode(_T);

  mT = Joint::mAspectProperties.mT_ParentBodyToJoint
       * Joint::mAspectProperties.mT_ChildBodyToJoint.inverse();
}

//==============================================================================
WeldJoint::WeldJoint(const Properties& properties)
{
  // Inherited Aspects must be created in the final joint class or else we
  // get pure virtual function calls
  createJointAspect(properties);
}

//==============================================================================
Joint* WeldJoint::clone() const
{
  return new WeldJoint(getWeldJointProperties());
}

//==============================================================================
void WeldJoint::updateRelativeTransform() const
{
  // Do nothing
}

//==============================================================================
void WeldJoint::updateRelativeSpatialVelocity() const
{
  // Do nothing
  // Should we have mSpatialVelocity.setZero() here instead?
}

//==============================================================================
void WeldJoint::updateRelativeSpatialAcceleration() const
{
  // Do nothing
  // Should we have mSpatialAcceleration.setZero() here instead?
}

//==============================================================================
void WeldJoint::updateRelativePrimaryAcceleration() const
{
  // Do nothing
}

//==============================================================================
void WeldJoint::updateRelativeJacobian(bool) const
{
  // Do nothing
}

//==============================================================================
void WeldJoint::updateRelativeJacobianTimeDeriv() const
{
  // Do nothing
}

} // namespace dynamics
} // namespace dart
