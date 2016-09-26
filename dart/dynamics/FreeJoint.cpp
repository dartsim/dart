/*
 * Copyright (c) 2013-2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2013-2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
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

#include "dart/dynamics/FreeJoint.hpp"

#include <string>

#include "dart/math/Helpers.hpp"
#include "dart/math/Geometry.hpp"
#include "dart/dynamics/DegreeOfFreedom.hpp"

namespace dart {
namespace dynamics {

//==============================================================================
FreeJoint::Properties::Properties(const Base::Properties& properties)
  : Base::Properties(properties)
{
  // Do nothing
}

//==============================================================================
FreeJoint::~FreeJoint()
{
  // Do nothing
}

//==============================================================================
FreeJoint::Properties FreeJoint::getFreeJointProperties() const
{
  return getGenericJointProperties();
}

//==============================================================================
Eigen::Vector6d FreeJoint::convertToPositions(const Eigen::Isometry3d& _tf)
{
  Eigen::Vector6d x;
  x.head<3>() = math::logMap(_tf.linear());
  x.tail<3>() = _tf.translation();
  return x;
}

//==============================================================================
Eigen::Isometry3d FreeJoint::convertToTransform(
    const Eigen::Vector6d& _positions)
{
  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  tf.linear() = math::expMapRot(_positions.head<3>());
  tf.translation() = _positions.tail<3>();
  return tf;
}

//==============================================================================
void FreeJoint::setTransform(Joint* joint,
                             const Eigen::Isometry3d& tf,
                             const Frame* withRespectTo)
{
  if (nullptr == joint)
    return;

  FreeJoint* freeJoint = dynamic_cast<FreeJoint*>(joint);

  if (nullptr == freeJoint)
  {
    dtwarn << "[FreeJoint::setTransform] Invalid joint type. Setting transform "
           << "is only allowed to FreeJoint. The joint type of given joint ["
           << joint->getName() << "] is [" << joint->getType() << "].\n";
    return;
  }

  freeJoint->setTransform(tf, withRespectTo);
}

//==============================================================================
void FreeJoint::setTransform(BodyNode* bodyNode,
                             const Eigen::Isometry3d& tf,
                             const Frame* withRespectTo)
{
  if (nullptr == bodyNode)
    return;

  setTransform(bodyNode->getParentJoint(), tf, withRespectTo);
}

//==============================================================================
void FreeJoint::setTransform(Skeleton* skeleton,
                             const Eigen::Isometry3d& tf,
                             const Frame* withRespectTo,
                             bool applyToAllRootBodies)
{
  if (nullptr == skeleton)
    return;

  const std::size_t numTrees = skeleton->getNumTrees();

  if (0 == numTrees)
    return;

  if (!applyToAllRootBodies)
  {
    setTransform(skeleton->getRootBodyNode(), tf, withRespectTo);
    return;
  }

  for (std::size_t i = 0; i < numTrees; ++i)
    setTransform(skeleton->getRootBodyNode(i), tf, withRespectTo);
}

//==============================================================================
void FreeJoint::setSpatialMotion(const Eigen::Isometry3d* newTransform,
                                 const Frame* withRespectTo,
                                 const Eigen::Vector6d* newSpatialVelocity,
                                 const Frame* velRelativeTo,
                                 const Frame* velInCoordinatesOf,
                                 const Eigen::Vector6d* newSpatialAcceleration,
                                 const Frame* accRelativeTo,
                                 const Frame* accInCoordinatesOf)
{
  if (newTransform)
    setTransform(*newTransform, withRespectTo);

  if (newSpatialVelocity)
    setSpatialVelocity(*newSpatialVelocity, velRelativeTo, velInCoordinatesOf);

  if (newSpatialAcceleration)
  {
    setSpatialAcceleration(*newSpatialAcceleration,
                           accRelativeTo,
                           accInCoordinatesOf);
  }
}

//==============================================================================
void FreeJoint::setRelativeTransform(const Eigen::Isometry3d& newTransform)
{
  setPositionsStatic(convertToPositions(
    Joint::mAspectProperties.mT_ParentBodyToJoint.inverse() *
    newTransform *
    Joint::mAspectProperties.mT_ChildBodyToJoint));
}

//==============================================================================
void FreeJoint::setTransform(const Eigen::Isometry3d& newTransform,
                             const Frame* withRespectTo)
{
  assert(nullptr != withRespectTo);

  setRelativeTransform(
        withRespectTo->getTransform(getChildBodyNode()->getParentFrame())
        * newTransform);
}

//==============================================================================
void FreeJoint::setRelativeSpatialVelocity(
    const Eigen::Vector6d& newSpatialVelocity)
{
  setVelocitiesStatic(getRelativeJacobianStatic().inverse() * newSpatialVelocity);
}

//==============================================================================
void FreeJoint::setRelativeSpatialVelocity(
    const Eigen::Vector6d& newSpatialVelocity,
    const Frame* inCoordinatesOf)
{
  assert(nullptr != inCoordinatesOf);

  if (getChildBodyNode() == inCoordinatesOf)
  {
    setRelativeSpatialVelocity(newSpatialVelocity);
  }
  else
  {
    setRelativeSpatialVelocity(
          math::AdR(inCoordinatesOf->getTransform(getChildBodyNode()),
                    newSpatialVelocity));
  }
}

//==============================================================================
void FreeJoint::setSpatialVelocity(const Eigen::Vector6d& newSpatialVelocity,
                                   const Frame* relativeTo,
                                   const Frame* inCoordinatesOf)
{
  assert(nullptr != relativeTo);
  assert(nullptr != inCoordinatesOf);

  if (getChildBodyNode() == relativeTo)
  {
    dtwarn << "[FreeJoint::setSpatialVelocity] Invalid reference frame "
              "for newSpatialVelocity. It shouldn't be the child BodyNode.\n";
    return;
  }

  // Change the reference frame of "newSpatialVelocity" to the child body node
  // frame.
  Eigen::Vector6d targetRelSpatialVel = newSpatialVelocity;
  if (getChildBodyNode() != inCoordinatesOf)
  {
    targetRelSpatialVel
        = math::AdR(inCoordinatesOf->getTransform(getChildBodyNode()),
                    newSpatialVelocity);
  }

  // Compute the target relative spatial velocity from the parent body node to
  // the child body node.
  if (getChildBodyNode()->getParentFrame() != relativeTo)
  {
    if (relativeTo->isWorld())
    {
      const Eigen::Vector6d parentVelocity = math::AdInvT(
            getRelativeTransform(),
            getChildBodyNode()->getParentFrame()->getSpatialVelocity());

      targetRelSpatialVel -= parentVelocity;
    }
    else
    {
      const Eigen::Vector6d parentVelocity = math::AdInvT(
            getRelativeTransform(),
            getChildBodyNode()->getParentFrame()->getSpatialVelocity());
      const Eigen::Vector6d arbitraryVelocity = math::AdT(
            relativeTo->getTransform(getChildBodyNode()),
            relativeTo->getSpatialVelocity());

      targetRelSpatialVel += -parentVelocity + arbitraryVelocity;
    }
  }

  setRelativeSpatialVelocity(targetRelSpatialVel);
}

//==============================================================================
void FreeJoint::setLinearVelocity(const Eigen::Vector3d& newLinearVelocity,
                                  const Frame* relativeTo,
                                  const Frame* inCoordinatesOf)
{
  assert(nullptr != relativeTo);
  assert(nullptr != inCoordinatesOf);

  Eigen::Vector6d targetSpatialVelocity;

  if (Frame::World() == relativeTo)
  {
    targetSpatialVelocity.head<3>()
        = getChildBodyNode()->getSpatialVelocity().head<3>();
  }
  else
  {
    targetSpatialVelocity.head<3>()
        = getChildBodyNode()->getSpatialVelocity(
            relativeTo, getChildBodyNode()).head<3>();
  }

  targetSpatialVelocity.tail<3>()
      = getChildBodyNode()->getWorldTransform().linear().transpose()
        * inCoordinatesOf->getWorldTransform().linear()
        * newLinearVelocity;
  // Above code is equivalent to:
  // targetSpatialVelocity.tail<3>()
  //     = getChildBodyNode()->getTransform(
  //         inCoordinatesOf).linear().transpose()
  //       * newLinearVelocity;
  // but faster.

  setSpatialVelocity(targetSpatialVelocity, relativeTo, getChildBodyNode());
}

//==============================================================================
void FreeJoint::setAngularVelocity(const Eigen::Vector3d& newAngularVelocity,
                                   const Frame* relativeTo,
                                   const Frame* inCoordinatesOf)
{
  assert(nullptr != relativeTo);
  assert(nullptr != inCoordinatesOf);

  Eigen::Vector6d targetSpatialVelocity;

  targetSpatialVelocity.head<3>()
      = getChildBodyNode()->getWorldTransform().linear().transpose()
        * inCoordinatesOf->getWorldTransform().linear()
        * newAngularVelocity;
  // Above code is equivalent to:
  // targetSpatialVelocity.head<3>()
  //     = getChildBodyNode()->getTransform(
  //         inCoordinatesOf).linear().transpose()
  //       * newAngularVelocity;
  // but faster.

  if (Frame::World() == relativeTo)
  {
    targetSpatialVelocity.tail<3>()
        = getChildBodyNode()->getSpatialVelocity().tail<3>();
  }
  else
  {
    targetSpatialVelocity.tail<3>()
        = getChildBodyNode()->getSpatialVelocity(
            relativeTo, getChildBodyNode()).tail<3>();
  }

  setSpatialVelocity(targetSpatialVelocity, relativeTo, getChildBodyNode());
}

//==============================================================================
void FreeJoint::setRelativeSpatialAcceleration(
    const Eigen::Vector6d& newSpatialAcceleration)
{
  const Eigen::Matrix6d& J = getRelativeJacobianStatic();
  const Eigen::Matrix6d& dJ = getRelativeJacobianTimeDerivStatic();

  setAccelerationsStatic(
    J.inverse() * (newSpatialAcceleration - dJ * getVelocitiesStatic()));
}

//==============================================================================
void FreeJoint::setRelativeSpatialAcceleration(
    const Eigen::Vector6d& newSpatialAcceleration,
    const Frame* inCoordinatesOf)
{
  assert(nullptr != inCoordinatesOf);

  if (getChildBodyNode() == inCoordinatesOf)
  {
    setRelativeSpatialAcceleration(newSpatialAcceleration);
  }
  else
  {
    setRelativeSpatialAcceleration(
          math::AdR(inCoordinatesOf->getTransform(getChildBodyNode()),
                    newSpatialAcceleration));
  }
}

//==============================================================================
void FreeJoint::setSpatialAcceleration(
    const Eigen::Vector6d& newSpatialAcceleration,
    const Frame* relativeTo,
    const Frame* inCoordinatesOf)
{
  assert(nullptr != relativeTo);
  assert(nullptr != inCoordinatesOf);

  if (getChildBodyNode() == relativeTo)
  {
    dtwarn << "[FreeJoint::setSpatialAcceleration] Invalid reference "
           << "frame for newSpatialAcceleration. It shouldn't be the child "
           << "BodyNode.\n";
    return;
  }

  // Change the reference frame of "newSpatialAcceleration" to the child body
  // node frame.
  Eigen::Vector6d targetRelSpatialAcc = newSpatialAcceleration;
  if (getChildBodyNode() != inCoordinatesOf)
  {
    targetRelSpatialAcc
        = math::AdR(inCoordinatesOf->getTransform(getChildBodyNode()),
                    newSpatialAcceleration);
  }

  // Compute the target relative spatial acceleration from the parent body node
  // to the child body node.
  if (getChildBodyNode()->getParentFrame() != relativeTo)
  {
    if (relativeTo->isWorld())
    {
      const Eigen::Vector6d parentAcceleration
          = math::AdInvT(
            getRelativeTransform(),
            getChildBodyNode()->getParentFrame()->getSpatialAcceleration())
            + math::ad(getChildBodyNode()->getSpatialVelocity(),
                       getRelativeJacobianStatic() * getVelocitiesStatic());

      targetRelSpatialAcc -= parentAcceleration;
    }
    else
    {
      const Eigen::Vector6d parentAcceleration
          = math::AdInvT(
            getRelativeTransform(),
            getChildBodyNode()->getParentFrame()->getSpatialAcceleration())
            + math::ad(getChildBodyNode()->getSpatialVelocity(),
                       getRelativeJacobianStatic() * getVelocitiesStatic());
      const Eigen::Vector6d arbitraryAcceleration =
          math::AdT(relativeTo->getTransform(getChildBodyNode()),
                    relativeTo->getSpatialAcceleration())
          - math::ad(getChildBodyNode()->getSpatialVelocity(),
                     math::AdT(relativeTo->getTransform(getChildBodyNode()),
                               relativeTo->getSpatialVelocity()));

      targetRelSpatialAcc += -parentAcceleration + arbitraryAcceleration;
    }
  }

  setRelativeSpatialAcceleration(targetRelSpatialAcc);
}

//==============================================================================
void FreeJoint::setLinearAcceleration(
    const Eigen::Vector3d& newLinearAcceleration,
    const Frame* relativeTo,
    const Frame* inCoordinatesOf)
{
  assert(nullptr != relativeTo);
  assert(nullptr != inCoordinatesOf);

  Eigen::Vector6d targetSpatialAcceleration;

  if (Frame::World() == relativeTo)
  {
    targetSpatialAcceleration.head<3>()
        = getChildBodyNode()->getSpatialAcceleration().head<3>();
  }
  else
  {
    targetSpatialAcceleration.head<3>()
        = getChildBodyNode()->getSpatialAcceleration(
            relativeTo, getChildBodyNode()).head<3>();
  }

  const Eigen::Vector6d& V
      = getChildBodyNode()->getSpatialVelocity(relativeTo, inCoordinatesOf);
  targetSpatialAcceleration.tail<3>()
      = getChildBodyNode()->getWorldTransform().linear().transpose()
        * inCoordinatesOf->getWorldTransform().linear()
        * (newLinearAcceleration - V.head<3>().cross(V.tail<3>()));
  // Above code is equivalent to:
  // targetSpatialAcceleration.tail<3>()
  //     = getChildBodyNode()->getTransform(
  //         inCoordinatesOf).linear().transpose()
  //       * (newLinearAcceleration - V.head<3>().cross(V.tail<3>()));
  // but faster.

  setSpatialAcceleration(
        targetSpatialAcceleration, relativeTo, getChildBodyNode());
}

//==============================================================================
void FreeJoint::setAngularAcceleration(
    const Eigen::Vector3d& newAngularAcceleration,
    const Frame* relativeTo,
    const Frame* inCoordinatesOf)
{
  assert(nullptr != relativeTo);
  assert(nullptr != inCoordinatesOf);

  Eigen::Vector6d targetSpatialAcceleration;

  targetSpatialAcceleration.head<3>()
      = getChildBodyNode()->getWorldTransform().linear().transpose()
        * inCoordinatesOf->getWorldTransform().linear()
        * newAngularAcceleration;
  // Above code is equivalent to:
  // targetSpatialAcceleration.head<3>()
  //     = getChildBodyNode()->getTransform(
  //         inCoordinatesOf).linear().transpose()
  //       * newAngularAcceleration;
  // but faster.

  if (Frame::World() == relativeTo)
  {
    targetSpatialAcceleration.tail<3>()
        = getChildBodyNode()->getSpatialAcceleration().tail<3>();
  }
  else
  {
    targetSpatialAcceleration.tail<3>()
        = getChildBodyNode()->getSpatialAcceleration(
            relativeTo, getChildBodyNode()).tail<3>();
  }

  setSpatialAcceleration(
        targetSpatialAcceleration, relativeTo, getChildBodyNode());
}

//==============================================================================
Eigen::Matrix6d FreeJoint::getRelativeJacobianStatic(
    const Eigen::Vector6d& /*positions*/) const
{
  return mJacobian;
}

//==============================================================================
Eigen::Vector6d FreeJoint::getPositionDifferencesStatic(
    const Eigen::Vector6d& _q2,
    const Eigen::Vector6d& _q1) const
{
  const Eigen::Isometry3d T1 = convertToTransform(_q1);
  const Eigen::Isometry3d T2 = convertToTransform(_q2);

  return convertToPositions(T1.inverse() * T2);
}

//==============================================================================
FreeJoint::FreeJoint(const Properties& properties)
  : Base(properties),
    mQ(Eigen::Isometry3d::Identity())
{
  mJacobianDeriv = Eigen::Matrix6d::Zero();

  // Inherited Aspects must be created in the final joint class in reverse order
  // or else we get pure virtual function calls
  createGenericJointAspect(properties);
  createJointAspect(properties);
}

//==============================================================================
Joint* FreeJoint::clone() const
{
  return new FreeJoint(getFreeJointProperties());
}

//==============================================================================
const std::string& FreeJoint::getType() const
{
  return getStaticType();
}

//==============================================================================
const std::string& FreeJoint::getStaticType()
{
  static const std::string name = "FreeJoint";
  return name;
}

//==============================================================================
bool FreeJoint::isCyclic(std::size_t _index) const
{
  return _index < 3
      && !hasPositionLimit(0) && !hasPositionLimit(1) && !hasPositionLimit(2);
}

//==============================================================================
void FreeJoint::integratePositions(double _dt)
{
  const Eigen::Isometry3d Qnext
      = getQ() * convertToTransform(getVelocitiesStatic() * _dt);

  setPositionsStatic(convertToPositions(Qnext));
}

//==============================================================================
void FreeJoint::updateDegreeOfFreedomNames()
{
  if(!mDofs[0]->isNamePreserved())
    mDofs[0]->setName(Joint::mAspectProperties.mName + "_rot_x", false);
  if(!mDofs[1]->isNamePreserved())
    mDofs[1]->setName(Joint::mAspectProperties.mName + "_rot_y", false);
  if(!mDofs[2]->isNamePreserved())
    mDofs[2]->setName(Joint::mAspectProperties.mName + "_rot_z", false);
  if(!mDofs[3]->isNamePreserved())
    mDofs[3]->setName(Joint::mAspectProperties.mName + "_pos_x", false);
  if(!mDofs[4]->isNamePreserved())
    mDofs[4]->setName(Joint::mAspectProperties.mName + "_pos_y", false);
  if(!mDofs[5]->isNamePreserved())
    mDofs[5]->setName(Joint::mAspectProperties.mName + "_pos_z", false);
}

//==============================================================================
void FreeJoint::updateRelativeTransform() const
{
  mQ = convertToTransform(getPositionsStatic());

  mT = Joint::mAspectProperties.mT_ParentBodyToJoint * mQ
      * Joint::mAspectProperties.mT_ChildBodyToJoint.inverse();

  assert(math::verifyTransform(mT));
}

//==============================================================================
void FreeJoint::updateRelativeJacobian(bool _mandatory) const
{
  if (_mandatory)
    mJacobian = math::getAdTMatrix(Joint::mAspectProperties.mT_ChildBodyToJoint);
}

//==============================================================================
void FreeJoint::updateRelativeJacobianTimeDeriv() const
{
  assert(Eigen::Matrix6d::Zero() == mJacobianDeriv);
}

//==============================================================================
const Eigen::Isometry3d& FreeJoint::getQ() const
{
  if(mNeedTransformUpdate)
  {
    updateRelativeTransform();
    mNeedTransformUpdate = false;
  }

  return mQ;
}

}  // namespace dynamics
}  // namespace dart
