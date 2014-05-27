/*
 * Copyright (c) 2013, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
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

#include "dart/dynamics/WeldJoint.h"

#include <string>

#include "dart/math/Helpers.h"
#include "dart/math/Geometry.h"

namespace dart {
namespace dynamics {

//==============================================================================
WeldJoint::WeldJoint(const std::string& _name)
  : Joint(_name)
{
}

//==============================================================================
WeldJoint::~WeldJoint()
{
}

//==============================================================================
void WeldJoint::setTransformFromParentBodyNode(const Eigen::Isometry3d& _T)
{
  Joint::setTransformFromParentBodyNode(_T);

  mT = mT_ParentBodyToJoint * mT_ChildBodyToJoint.inverse();
}

//==============================================================================
void WeldJoint::setTransformFromChildBodyNode(const Eigen::Isometry3d& _T)
{
  Joint::setTransformFromChildBodyNode(_T);

  mT = mT_ParentBodyToJoint * mT_ChildBodyToJoint.inverse();
}

//==============================================================================
void WeldJoint::updateLocalTransform()
{
  // Do nothing
}

//==============================================================================
void WeldJoint::updateLocalJacobian()
{
  // Do nothing
}

//==============================================================================
void WeldJoint::addVelocityTo(Eigen::Vector6d& /*_vel*/)
{
  // Do nothing
}

//==============================================================================
void WeldJoint::addVelocityChangeTo(Eigen::Vector6d& /*_velocityChange*/)
{
  // Do nothing
}

//==============================================================================
void WeldJoint::setPartialAccelerationTo(
    Eigen::Vector6d& _partialAcceleration,
    const Eigen::Vector6d& /*_childVelocity*/)
{
  _partialAcceleration.setZero();
}

//==============================================================================
void WeldJoint::updateLocalJacobianTimeDeriv()
{
  // Do nothing
}

//==============================================================================
void WeldJoint::addAccelerationTo(Eigen::Vector6d& /*_acc*/)
{
  // Do nothing
}

//==============================================================================
void WeldJoint::addChildArtInertiaTo(Eigen::Matrix6d& _parentArtInertia,
                                     const Eigen::Matrix6d& _childArtInertia)
{
  // Add child body's articulated inertia to parent body's articulated inertia.
  // Note that mT should be updated.
  _parentArtInertia += math::transformInertia(mT.inverse(), _childArtInertia);
}

//==============================================================================
void WeldJoint::addChildArtInertiaImplicitTo(
    Eigen::Matrix6d& _parentArtInertia,
    const Eigen::Matrix6d& _childArtInertia)
{
  // Add child body's articulated inertia to parent body's articulated inertia.
  // Note that mT should be updated.
  _parentArtInertia += math::transformInertia(mT.inverse(), _childArtInertia);
}

//==============================================================================
void WeldJoint::updateInvProjArtInertia(const Eigen::Matrix6d& /*_artInertia*/)
{
  // Do nothing
}

//==============================================================================
void WeldJoint::updateInvProjArtInertiaImplicit(
    const Eigen::Matrix6d& /*_artInertia*/,
    double /*_timeStep*/)
{
  // Do nothing
}

//==============================================================================
void WeldJoint::addChildBiasForceTo(Eigen::Vector6d& _parentBiasForce,
                                    const Eigen::Matrix6d& _childArtInertia,
                                    const Eigen::Vector6d& _childBiasForce,
                                    const Eigen::Vector6d& _childPartialAcc)
{
  // Add child body's bias force to parent body's bias force. Note that mT
  // should be updated.
  _parentBiasForce += math::dAdInvT(mT, _childBiasForce
                                    + _childArtInertia * _childPartialAcc);
}

//==============================================================================
void WeldJoint::addChildBiasImpulseTo(
    Eigen::Vector6d& _parentBiasImpulse,
    const Eigen::Matrix6d& /*_childArtInertia*/,
    const Eigen::Vector6d& _childBiasImpulse)
{
  // Add child body's bias force to parent body's bias impulse. Note that mT
  // should be updated.
  _parentBiasImpulse += math::dAdInvT(mT, _childBiasImpulse);
}

//==============================================================================
void WeldJoint::updateTotalForce(const Eigen::Vector6d& /*_bodyForce*/,
                                 double /*_timeStep*/)
{
  // Do nothing
}

//==============================================================================
void WeldJoint::updateTotalImpulse(const Eigen::Vector6d& /*_bodyImpulse*/)
{
  // Do nothing
}

//==============================================================================
void WeldJoint::updateAcceleration(const Eigen::Matrix6d& /*_artInertia*/,
                                   const Eigen::Vector6d& /*_spatialAcc*/)
{
  // Do nothing
}

//==============================================================================
void WeldJoint::updateVelocityChange(const Eigen::Matrix6d& /*_artInertia*/,
                                     const Eigen::Vector6d& /*_velocityChange*/)
{
  // Do nothing
}

//==============================================================================
void WeldJoint::clearConstraintImpulse()
{
  // Do nothing
}

//==============================================================================
void WeldJoint::updateVelocityWithVelocityChange()
{
  // Do nothing
}

//==============================================================================
void WeldJoint::updateAccelerationWithVelocityChange(double _timeStep)
{
  // Do nothing
}

//==============================================================================
void WeldJoint::updateForceWithImpulse(double _timeStep)
{
  // Do nothing
}

//==============================================================================
void WeldJoint::addChildBiasForceForInvMassMatrix(
    Eigen::Vector6d& _parentBiasForce,
    const Eigen::Matrix6d& _childArtInertia,
    const Eigen::Vector6d& _childBiasForce)
{
  // TODO(JS)
}

//==============================================================================
void WeldJoint::addChildBiasForceForInvAugMassMatrix(
    Eigen::Vector6d& _parentBiasForce,
    const Eigen::Matrix6d& _childArtInertia,
    const Eigen::Vector6d& _childBiasForce)
{
  // TODO(JS)
}

//==============================================================================
void WeldJoint::updateTotalForceForInvMassMatrix(
    const Eigen::Vector6d& _bodyForce)
{
  // TODO(JS)
}

//==============================================================================
void WeldJoint::getInvMassMatrixSegment(
    Eigen::MatrixXd& _invMassMat,
    const size_t _col,
    const Eigen::Matrix6d& _artInertia,
    const Eigen::Vector6d& _spatialAcc)
{
  // TODO(JS)
}

//==============================================================================
void WeldJoint::getInvAugMassMatrixSegment(
    Eigen::MatrixXd& _invMassMat,
    const size_t _col,
    const Eigen::Matrix6d& _artInertia,
    const Eigen::Vector6d& _spatialAcc)
{
  // TODO(JS)
}

//==============================================================================
void WeldJoint::addInvMassMatrixSegmentTo(Eigen::Vector6d& _acc)
{
  // TODO(JS)
}

}  // namespace dynamics
}  // namespace dart
