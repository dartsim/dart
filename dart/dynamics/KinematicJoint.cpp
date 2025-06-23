/*
 * Copyright (c) 2011-2024, The DART development contributors
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

#include "dart/dynamics/KinematicJoint.hpp"

#include "dart/dynamics/DegreeOfFreedom.hpp"
#include "dart/math/Geometry.hpp"
#include "dart/math/Helpers.hpp"

#include <string>

namespace dart {
namespace dynamics {

//==============================================================================
KinematicJoint::Properties::Properties(const Base::Properties& properties)
  : Base::Properties(properties)
{
  // Do nothing
}

//==============================================================================
KinematicJoint::~KinematicJoint()
{
  // Do nothing
}

//==============================================================================
KinematicJoint::Properties KinematicJoint::getKinematicJointProperties() const
{
  return getGenericJointProperties();
}

//==============================================================================
Eigen::Vector6d KinematicJoint::convertToPositions(const Eigen::Isometry3d& _tf)
{
  Eigen::Vector6d x;
  x.head<3>() = math::logMap(_tf.linear());
  x.tail<3>() = _tf.translation();
  return x;
}

//==============================================================================
Eigen::Isometry3d KinematicJoint::convertToTransform(
    const Eigen::Vector6d& _positions)
{
  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  tf.linear() = math::expMapRot(_positions.head<3>());
  tf.translation() = _positions.tail<3>();
  return tf;
}

//==============================================================================
void KinematicJoint::setTransformOf(
    Joint* joint, const Eigen::Isometry3d& tf, const Frame* withRespectTo)
{
  if (nullptr == joint)
    return;

  KinematicJoint* kinematicJoint = dynamic_cast<KinematicJoint*>(joint);

  if (nullptr == kinematicJoint) {
    dtwarn
        << "[KinematicJoint::setTransform] Invalid joint type. Setting "
           "transform "
        << "is only allowed to KinematicJoint. The joint type of given joint ["
        << joint->getName() << "] is [" << joint->getType() << "].\n";
    return;
  }

  kinematicJoint->setTransform(tf, withRespectTo);
}

//==============================================================================
void KinematicJoint::setTransformOf(
    BodyNode* bodyNode, const Eigen::Isometry3d& tf, const Frame* withRespectTo)
{
  if (nullptr == bodyNode)
    return;

  setTransformOf(bodyNode->getParentJoint(), tf, withRespectTo);
}

//==============================================================================
void KinematicJoint::setTransformOf(
    Skeleton* skeleton,
    const Eigen::Isometry3d& tf,
    const Frame* withRespectTo,
    bool applyToAllRootBodies)
{
  if (!skeleton)
    return;

  if (skeleton->getNumTrees() == 0)
    return;

  if (applyToAllRootBodies) {
    for (std::size_t i = 0; i < skeleton->getNumTrees(); ++i) {
      setTransformOf(skeleton->getRootBodyNode(i), tf, withRespectTo);
    }
  } else {
    setTransformOf(skeleton->getRootBodyNode(), tf, withRespectTo);
  }
}

//==============================================================================
void KinematicJoint::setSpatialMotion(
    const Eigen::Isometry3d* newTransform,
    const Frame* withRespectTo,
    const Eigen::Vector6d* newSpatialVelocity,
    const Frame* velRelativeTo,
    const Frame* velInCoordinatesOf)
{
  if (newTransform)
    setTransform(*newTransform, withRespectTo);

  if (newSpatialVelocity)
    setSpatialVelocity(*newSpatialVelocity, velRelativeTo, velInCoordinatesOf);
}

//==============================================================================
void KinematicJoint::setRelativeTransform(const Eigen::Isometry3d& newTransform)
{
  setPositionsStatic(convertToPositions(
      Joint::mAspectProperties.mT_ParentBodyToJoint.inverse() * newTransform
      * Joint::mAspectProperties.mT_ChildBodyToJoint));
}

//==============================================================================
void KinematicJoint::setTransform(
    const Eigen::Isometry3d& newTransform, const Frame* withRespectTo)
{
  assert(nullptr != withRespectTo);

  setRelativeTransform(
      withRespectTo->getTransform(getChildBodyNode()->getParentFrame())
      * newTransform);
}

//==============================================================================
void KinematicJoint::setRelativeSpatialVelocity(
    const Eigen::Vector6d& newSpatialVelocity)
{
  setVelocitiesStatic(newSpatialVelocity);
}

//==============================================================================
void KinematicJoint::setRelativeSpatialVelocity(
    const Eigen::Vector6d& newSpatialVelocity, const Frame* inCoordinatesOf)
{
  assert(nullptr != inCoordinatesOf);

  if (getChildBodyNode() == inCoordinatesOf) {
    setRelativeSpatialVelocity(newSpatialVelocity);
  } else {
    setRelativeSpatialVelocity(
        math::AdR(
            inCoordinatesOf->getTransform(getChildBodyNode()),
            newSpatialVelocity));
  }
}

//==============================================================================
void KinematicJoint::setSpatialVelocity(
    const Eigen::Vector6d& newSpatialVelocity,
    const Frame* relativeTo,
    const Frame* inCoordinatesOf)
{
  assert(nullptr != relativeTo);
  assert(nullptr != inCoordinatesOf);

  if (getChildBodyNode() == relativeTo) {
    dtwarn << "[KinematicJoint::setSpatialVelocity] Invalid reference frame "
              "for newSpatialVelocity. It shouldn't be the child BodyNode.\n";
    return;
  }

  // Transform newSpatialVelocity into the child body node frame if needed
  Eigen::Vector6d targetRelSpatialVel = (getChildBodyNode() == inCoordinatesOf)
      ? newSpatialVelocity
      : math::AdR(inCoordinatesOf->getTransform(getChildBodyNode()), newSpatialVelocity);

  // Adjust for parent frame if necessary
  if (getChildBodyNode()->getParentFrame() != relativeTo) {
    const Eigen::Vector6d parentVelocity = math::AdInvT(
        getRelativeTransform(),
        getChildBodyNode()->getParentFrame()->getSpatialVelocity());

    if (relativeTo->isWorld()) {
      targetRelSpatialVel -= parentVelocity;
    } else {
      const Eigen::Vector6d arbitraryVelocity = math::AdT(
          relativeTo->getTransform(getChildBodyNode()),
          relativeTo->getSpatialVelocity());
      targetRelSpatialVel += -parentVelocity + arbitraryVelocity;
    }
  }

  setRelativeSpatialVelocity(targetRelSpatialVel);
}

//==============================================================================
void KinematicJoint::setLinearVelocity(
    const Eigen::Vector3d& newLinearVelocity, const Frame* relativeTo)
{
  assert(nullptr != relativeTo);  
  assert(nullptr != inCoordinatesOf);

  Eigen::Vector6d targetSpatialVelocity;

  if (Frame::World() == relativeTo) {
    targetSpatialVelocity.head<3>()
        = getChildBodyNode()->getSpatialVelocity().head<3>();
  } else {
    targetSpatialVelocity.head<3>()
        = getChildBodyNode()
              ->getSpatialVelocity(relativeTo, getChildBodyNode())
              .head<3>();
  }

  targetSpatialVelocity.tail<3>() = newLinearVelocity;
  setSpatialVelocity(targetSpatialVelocity, relativeTo, getChildBodyNode());
}

//==============================================================================
void KinematicJoint::setAngularVelocity(
    const Eigen::Vector3d& newAngularVelocity,
    const Frame* relativeTo,
    const Frame* inCoordinatesOf)
{
  assert(nullptr != relativeTo);
  assert(nullptr != inCoordinatesOf);

  Eigen::Vector6d targetSpatialVelocity;

  // Translate Velocities if a non world coordiinate frame is used
  if (Frame::World() == relativeTo) {
    targetSpatialVelocity.tail<3>()
        = getChildBodyNode()->getSpatialVelocity().tail<3>();
  } else {
    targetSpatialVelocity.tail<3>()
        = getChildBodyNode()
              ->getSpatialVelocity(relativeTo, getChildBodyNode())
              .tail<3>();
  }

  // Hean Angular Velocities
  targetSpatialVelocity.head<3>()
      = getChildBodyNode()->getWorldTransform().linear().transpose()
        * inCoordinatesOf->getWorldTransform().linear() * newAngularVelocity;

  setSpatialVelocity(targetSpatialVelocity, relativeTo, getChildBodyNode());
}

//==============================================================================
Eigen::Matrix6d KinematicJoint::getRelativeJacobianStatic(
    const Eigen::Vector6d& /*positions*/) const
{
  return mJacobian;
}

//==============================================================================
Eigen::Vector6d KinematicJoint::getPositionDifferencesStatic(
    const Eigen::Vector6d& _q2, const Eigen::Vector6d& _q1) const
{
  return convertToPositions(
      convertToTransform(_q1).inverse() * convertToTransform(_q2));
}

//==============================================================================
KinematicJoint::KinematicJoint(const Properties& properties)
  : Base(properties), mQ(Eigen::Isometry3d::Identity())
{
  mJacobianDeriv = Eigen::Matrix6d::Zero();
  createGenericJointAspect(properties);
  createJointAspect(properties);
}

//==============================================================================
Joint* KinematicJoint::clone() const
{
  return new KinematicJoint(getKinematicJointProperties());
}

//==============================================================================
const std::string& KinematicJoint::getType() const
{
  return getStaticType();
}

//==============================================================================
const std::string& KinematicJoint::getStaticType()
{
  static const std::string name = "KinematicJoint";
  return name;
}

//==============================================================================
bool KinematicJoint::isCyclic(std::size_t _index) const
{
  return _index < 3 && !hasPositionLimit(0) && !hasPositionLimit(1)
         && !hasPositionLimit(2);
}

//==============================================================================
void KinematicJoint::integratePositions(double _dt)
{
  const Eigen::Isometry3d Qdiff
      = convertToTransform(getVelocitiesStatic() * _dt);
  const Eigen::Isometry3d Qnext = getQ() * Qdiff;
  const Eigen::Isometry3d QdiffInv = Qdiff.inverse();

  setVelocitiesStatic(math::AdR(QdiffInv, getVelocitiesStatic()));
  setPositionsStatic(convertToPositions(Qnext));
}

//==============================================ss================================
void KinematicJoint::updateDegreeOfFreedomNames()
{
  static const char* dofNames[6]
      = {"_rot_x", "_rot_y", "_rot_z", "_pos_x", "_pos_y", "_pos_z"};

  for (std::size_t i = 0; i < 6; ++i) {
    if (!mDofs[i]->isNamePreserved()) {
      mDofs[i]->setName(Joint::mAspectProperties.mName + dofNames[i], false);
    }
  }
}

//==============================================================================
void KinematicJoint::updateRelativeTransform() const
{
  mT = Joint::mAspectProperties.mT_ParentBodyToJoint
       * convertToTransform(getPositionsStatic())
       * Joint::mAspectProperties.mT_ChildBodyToJoint.inverse();

  assert(math::verifyTransform(mT));
}

//==============================================================================
void KinematicJoint::updateRelativeJacobian(bool _mandatory) const
{
  if (_mandatory)
    mJacobian
        = math::getAdTMatrix(Joint::mAspectProperties.mT_ChildBodyToJoint);
}

//==============================================================================
void KinematicJoint::updateRelativeJacobianTimeDeriv() const
{
  assert(Eigen::Matrix6d::Zero() == mJacobianDeriv);
}

//==============================================================================
const Eigen::Isometry3d& KinematicJoint::getQ() const
{
  if (mNeedTransformUpdate) {
    updateRelativeTransform();
    mNeedTransformUpdate = false;
  }

  return mQ;
}

} // namespace dynamics
} // namespace dart
