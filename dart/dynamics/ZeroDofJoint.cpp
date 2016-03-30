/*
 * Copyright (c) 2014-2015, Georgia Tech Research Corporation
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

#include "dart/dynamics/ZeroDofJoint.h"

#include "dart/common/Console.h"
#include "dart/math/Helpers.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Skeleton.h"

namespace dart {
namespace dynamics {

//==============================================================================
ZeroDofJoint::Properties::Properties(const Joint::Properties& _properties)
  : Joint::Properties(_properties)
{
  // Do nothing
}

//==============================================================================
ZeroDofJoint::~ZeroDofJoint()
{
  // Do nothing
}

//==============================================================================
ZeroDofJoint::Properties ZeroDofJoint::getZeroDofJointProperties() const
{
  return getJointProperties();
}

//==============================================================================
DegreeOfFreedom* ZeroDofJoint::getDof(size_t)
{
  dterr << "[ZeroDofJoint::getDof] Attempting to get a DegreeOfFreedom from a "
        << "ZeroDofJoint. This is not allowed!\n";
  assert(false);
  return nullptr;
}

//==============================================================================
const DegreeOfFreedom* ZeroDofJoint::getDof(size_t) const
{
  dterr << "[ZeroDofJoint::getDof] Attempting to get a DegreeOfFreedom from a "
        << "ZeroDofJoint. This is not allowed!\n";
  assert(false);
  return nullptr;
}

//==============================================================================
const std::string& ZeroDofJoint::setDofName(size_t, const std::string &, bool)
{
  return emptyString;
}

//==============================================================================
void ZeroDofJoint::preserveDofName(size_t, bool)
{
  // Do nothing
}

//==============================================================================
bool ZeroDofJoint::isDofNamePreserved(size_t) const
{
  return false;
}

//==============================================================================
const std::string& ZeroDofJoint::getDofName(size_t) const
{
  return emptyString;
}

//==============================================================================
size_t ZeroDofJoint::getNumDofs() const
{
  return 0;
}

//==============================================================================
size_t ZeroDofJoint::getIndexInSkeleton(size_t _index) const
{
  dterr << "[ZeroDofJoint::getIndexInSkeleton] This function should never be "
        << "called (" << _index << ")!\n";
  assert(false);

  return 0;
}

//==============================================================================
size_t ZeroDofJoint::getIndexInTree(size_t _index) const
{
  dterr << "ZeroDofJoint::getIndexInTree] This function should never be "
        << "called (" << _index << ")!\n";
  assert(false);

  return 0;
}

//==============================================================================
void ZeroDofJoint::setCommand(size_t /*_index*/, double /*_command*/)
{
  // Do nothing
}

//==============================================================================
double ZeroDofJoint::getCommand(size_t _index) const
{
  dterr << "[ZeroDofJoint::getCommand]: index[" << _index << "] out of range"
        << std::endl;

  return 0.0;
}

//==============================================================================
void ZeroDofJoint::setCommands(const Eigen::VectorXd& /*_commands*/)
{
  // Do nothing
}

//==============================================================================
Eigen::VectorXd ZeroDofJoint::getCommands() const
{
  return Eigen::Matrix<double, 0, 1>();
}

//==============================================================================
void ZeroDofJoint::resetCommands()
{
  // Do nothing
}

//==============================================================================
void ZeroDofJoint::setPosition(size_t, double)
{
  // Do nothing
}

//==============================================================================
double ZeroDofJoint::getPosition(size_t _index) const
{
  dterr << "getPosition index[" << _index << "] out of range"
    << std::endl;

  return 0.0;
}

//==============================================================================
void ZeroDofJoint::setPositions(const Eigen::VectorXd& /*_positions*/)
{
  // Do nothing
}

//==============================================================================
Eigen::VectorXd ZeroDofJoint::getPositions() const
{
  return Eigen::Matrix<double, 0, 1>();
}

//==============================================================================
void ZeroDofJoint::setPositionLowerLimit(size_t /*_index*/,
                                         double /*_position*/)
{
  // Do nothing
}

//==============================================================================
double ZeroDofJoint::getPositionLowerLimit(size_t /*_index*/) const
{
  return 0.0;
}

//==============================================================================
void ZeroDofJoint::setPositionUpperLimit(size_t /*_index*/,
                                         double /*_position*/)
{
  // Do nothing
}

//==============================================================================
double ZeroDofJoint::getPositionUpperLimit(size_t /*_index*/) const
{
  return 0.0;
}

//==============================================================================
bool ZeroDofJoint::hasPositionLimit(size_t /*_index*/) const
{
  return true;
}

//==============================================================================
void ZeroDofJoint::resetPosition(size_t /*_index*/)
{
  // Do nothing
}

//==============================================================================
void ZeroDofJoint::resetPositions()
{
  // Do nothing
}

//==============================================================================
void ZeroDofJoint::setInitialPosition(size_t /*_index*/, double /*_initial*/)
{
  // Do nothing
}

//==============================================================================
double ZeroDofJoint::getInitialPosition(size_t /*_index*/) const
{
  return 0.0;
}

//==============================================================================
void ZeroDofJoint::setInitialPositions(const Eigen::VectorXd& /*_initial*/)
{
  // Do nothing
}

//==============================================================================
Eigen::VectorXd ZeroDofJoint::getInitialPositions() const
{
  return Eigen::VectorXd();
}

//==============================================================================
void ZeroDofJoint::setVelocity(size_t /*_index*/, double /*_velocity*/)
{
  // Do nothing
}

//==============================================================================
double ZeroDofJoint::getVelocity(size_t /*_index*/) const
{
  return 0.0;
}

//==============================================================================
void ZeroDofJoint::setVelocities(const Eigen::VectorXd& /*_velocities*/)
{
  // Do nothing
}

//==============================================================================
Eigen::VectorXd ZeroDofJoint::getVelocities() const
{
  return Eigen::Matrix<double, 0, 1>();
}

//==============================================================================
void ZeroDofJoint::setVelocityLowerLimit(size_t /*_index*/,
                                         double /*_velocity*/)
{
  // Do nothing
}

//==============================================================================
double ZeroDofJoint::getVelocityLowerLimit(size_t /*_index*/) const
{
  return 0.0;
}

//==============================================================================
void ZeroDofJoint::setVelocityUpperLimit(size_t /*_index*/,
                                         double /*_velocity*/)
{
  // Do nothing
}

//==============================================================================
double ZeroDofJoint::getVelocityUpperLimit(size_t /*_index*/) const
{
  return 0.0;
}

//==============================================================================
void ZeroDofJoint::resetVelocity(size_t /*_index*/)
{
  // Do nothing
}

//==============================================================================
void ZeroDofJoint::resetVelocities()
{
  // Do nothing
}

//==============================================================================
void ZeroDofJoint::setInitialVelocity(size_t /*_index*/, double /*_initial*/)
{
  // Do nothing
}

//==============================================================================
double ZeroDofJoint::getInitialVelocity(size_t /*_index*/) const
{
  return 0.0;
}

//==============================================================================
void ZeroDofJoint::setInitialVelocities(const Eigen::VectorXd& /*_initial*/)
{
  // Do nothing
}

//==============================================================================
Eigen::VectorXd ZeroDofJoint::getInitialVelocities() const
{
  return Eigen::VectorXd();
}

//==============================================================================
void ZeroDofJoint::setAcceleration(size_t /*_index*/, double /*_acceleration*/)
{
  // Do nothing
}

//==============================================================================
double ZeroDofJoint::getAcceleration(size_t /*_index*/) const
{
  return 0.0;
}

//==============================================================================
void ZeroDofJoint::setAccelerations(const Eigen::VectorXd& /*_accelerations*/)
{
  // Do nothing
}

//==============================================================================
Eigen::VectorXd ZeroDofJoint::getAccelerations() const
{
  return Eigen::Matrix<double, 0, 1>();
}

//==============================================================================
void ZeroDofJoint::resetAccelerations()
{
  // Do nothing
}

//==============================================================================
void ZeroDofJoint::setAccelerationLowerLimit(size_t /*_index*/,
                                             double /*_acceleration*/)
{
  // Do nothing
}

//==============================================================================
double ZeroDofJoint::getAccelerationLowerLimit(size_t /*_index*/) const
{
  return 0.0;
}

//==============================================================================
void ZeroDofJoint::setAccelerationUpperLimit(size_t /*_index*/,
                                             double /*_acceleration*/)
{
  // Do nothing
}

//==============================================================================
double ZeroDofJoint::getAccelerationUpperLimit(size_t /*_index*/) const
{
  return 0.0;
}

//==============================================================================
void ZeroDofJoint::setForce(size_t /*_index*/, double /*_force*/)
{
  // Do nothing
}

//==============================================================================
double ZeroDofJoint::getForce(size_t /*_index*/)
{
  return 0.0;
}

//==============================================================================
void ZeroDofJoint::setForces(const Eigen::VectorXd& /*_forces*/)
{
  // Do nothing
}

//==============================================================================
Eigen::VectorXd ZeroDofJoint::getForces() const
{
  return Eigen::Matrix<double, 0, 1>();
}

//==============================================================================
void ZeroDofJoint::resetForces()
{
  // Do nothing
}

//==============================================================================
void ZeroDofJoint::setForceLowerLimit(size_t /*_index*/, double /*_force*/)
{
  // Do nothing
}

//==============================================================================
double ZeroDofJoint::getForceLowerLimit(size_t /*_index*/) const
{
  return 0.0;
}

//==============================================================================
void ZeroDofJoint::setForceUpperLimit(size_t /*_index*/, double /*_force*/)
{
  // Do nothing
}

//==============================================================================
double ZeroDofJoint::getForceUpperLimit(size_t /*_index*/) const
{
  return 0.0;
}

//==============================================================================
void ZeroDofJoint::setVelocityChange(size_t /*_index*/,
                                     double /*_velocityChange*/)
{
  // Do nothing
}

//==============================================================================
double ZeroDofJoint::getVelocityChange(size_t /*_index*/) const
{
  return 0.0;
}

//==============================================================================
void ZeroDofJoint::resetVelocityChanges()
{
  // Do nothing
}

//==============================================================================
void ZeroDofJoint::setConstraintImpulse(size_t /*_index*/, double /*_impulse*/)
{
  // Do nothing
}

//==============================================================================
double ZeroDofJoint::getConstraintImpulse(size_t /*_index*/) const
{
  return 0.0;
}

//==============================================================================
void ZeroDofJoint::resetConstraintImpulses()
{
  // Do nothing
}

//==============================================================================
void ZeroDofJoint::integratePositions(double /*_dt*/)
{
  // Do nothing
}

//==============================================================================
void ZeroDofJoint::integrateVelocities(double /*_dt*/)
{
  // Do nothing
}

//==============================================================================
Eigen::VectorXd ZeroDofJoint::getPositionDifferences(
    const Eigen::VectorXd& /*_q2*/,
    const Eigen::VectorXd& /*_q1*/) const
{
  return Eigen::VectorXd::Zero(0);
}

//==============================================================================
void ZeroDofJoint::setSpringStiffness(size_t /*_index*/, double /*_k*/)
{
  // Do nothing
}

//==============================================================================
double ZeroDofJoint::getSpringStiffness(size_t /*_index*/) const
{
  return 0.0;
}

//==============================================================================
void ZeroDofJoint::setRestPosition(size_t /*_index*/, double /*_q0*/)
{
  // Do nothing
}

//==============================================================================
double ZeroDofJoint::getRestPosition(size_t /*_index*/) const
{
  return 0.0;
}

//==============================================================================
void ZeroDofJoint::setDampingCoefficient(size_t /*_index*/, double /*_d*/)
{
  // Do nothing
}

//==============================================================================
double ZeroDofJoint::getDampingCoefficient(size_t /*_index*/) const
{
  return 0.0;
}

//==============================================================================
void ZeroDofJoint::setCoulombFriction(size_t /*_index*/, double /*_friction*/)
{
  // Do nothing
}

//==============================================================================
double ZeroDofJoint::getCoulombFriction(size_t /*_index*/) const
{
  return 0.0;
}

//==============================================================================
double ZeroDofJoint::getPotentialEnergy() const
{
  return 0.0;
}

//==============================================================================
ZeroDofJoint::ZeroDofJoint()
{
  // Do nothing. The Joint Aspect must be created by the most derived class.
}

//==============================================================================
void ZeroDofJoint::registerDofs()
{
  // Do nothing
}

//==============================================================================
void ZeroDofJoint::updateDegreeOfFreedomNames()
{
  // Do nothing
}

//==============================================================================
Eigen::Vector6d ZeroDofJoint::getBodyConstraintWrench() const
{
  assert(mChildBodyNode);
  return mChildBodyNode->getBodyForce();
}

//==============================================================================
const math::Jacobian ZeroDofJoint::getLocalJacobian() const
{
  return Eigen::Matrix<double, 6, 0>();
}

//==============================================================================
math::Jacobian ZeroDofJoint::getLocalJacobian(
    const Eigen::VectorXd& /*_positions*/) const
{
  return Eigen::Matrix<double, 6, 0>();
}

//==============================================================================
const math::Jacobian ZeroDofJoint::getLocalJacobianTimeDeriv() const
{
  return Eigen::Matrix<double, 6, 0>();
}

//==============================================================================
void ZeroDofJoint::addVelocityTo(Eigen::Vector6d& /*_vel*/)
{
  // Do nothing
}

//==============================================================================
void ZeroDofJoint::addVelocityChangeTo(Eigen::Vector6d& /*_velocityChange*/)
{
  // Do nothing
}

//==============================================================================
void ZeroDofJoint::setPartialAccelerationTo(
    Eigen::Vector6d& _partialAcceleration,
    const Eigen::Vector6d& /*_childVelocity*/)
{
  _partialAcceleration.setZero();
}

//==============================================================================
void ZeroDofJoint::addAccelerationTo(Eigen::Vector6d& /*_acc*/)
{
  // Do nothing
}

//==============================================================================
void ZeroDofJoint::addChildArtInertiaTo(
    Eigen::Matrix6d& _parentArtInertia,
    const Eigen::Matrix6d& _childArtInertia)
{
  // Add child body's articulated inertia to parent body's articulated inertia.
  // Note that mT should be updated.
  _parentArtInertia += math::transformInertia(getLocalTransform().inverse(),
                                              _childArtInertia);
}

//==============================================================================
void ZeroDofJoint::addChildArtInertiaImplicitTo(
    Eigen::Matrix6d& _parentArtInertia, const Eigen::Matrix6d& _childArtInertia)
{
  // Add child body's articulated inertia to parent body's articulated inertia.
  // Note that mT should be updated.
  _parentArtInertia += math::transformInertia(getLocalTransform().inverse(),
                                              _childArtInertia);
}

//==============================================================================
void ZeroDofJoint::updateInvProjArtInertia(
    const Eigen::Matrix6d& /*_artInertia*/)
{
  // Do nothing
}

//==============================================================================
void ZeroDofJoint::updateInvProjArtInertiaImplicit(
    const Eigen::Matrix6d& /*_artInertia*/,
    double /*_timeStep*/)
{
  // Do nothing
}

//==============================================================================
void ZeroDofJoint::addChildBiasForceTo(
    Eigen::Vector6d& _parentBiasForce,
    const Eigen::Matrix6d& _childArtInertia,
    const Eigen::Vector6d& _childBiasForce,
    const Eigen::Vector6d& _childPartialAcc)
{
  // Add child body's bias force to parent body's bias force. Note that mT
  // should be updated.
  _parentBiasForce += math::dAdInvT(getLocalTransform(), _childBiasForce
                                    + _childArtInertia*_childPartialAcc);
}

//==============================================================================
void ZeroDofJoint::addChildBiasImpulseTo(
    Eigen::Vector6d& _parentBiasImpulse,
    const Eigen::Matrix6d& /*_childArtInertia*/,
    const Eigen::Vector6d& _childBiasImpulse)
{
  // Add child body's bias force to parent body's bias impulse. Note that mT
  // should be updated.
  _parentBiasImpulse += math::dAdInvT(getLocalTransform(), _childBiasImpulse);
}

//==============================================================================
void ZeroDofJoint::updateTotalForce(const Eigen::Vector6d& /*_bodyForce*/,
                                    double /*_timeStep*/)
{
  // Do nothing
}

//==============================================================================
void ZeroDofJoint::updateTotalImpulse(const Eigen::Vector6d& /*_bodyImpulse*/)
{
  // Do nothing
}

//==============================================================================
void ZeroDofJoint::resetTotalImpulses()
{
  // Do nothing
}

//==============================================================================
void ZeroDofJoint::updateAcceleration(const Eigen::Matrix6d& /*_artInertia*/,
                                      const Eigen::Vector6d& /*_spatialAcc*/)
{
  // Do nothing
}

//==============================================================================
void ZeroDofJoint::updateVelocityChange(
    const Eigen::Matrix6d& /*_artInertia*/,
    const Eigen::Vector6d& /*_velocityChange*/)
{
  // Do nothing
}

//==============================================================================
void ZeroDofJoint::updateForceID(const Eigen::Vector6d& /*_bodyForce*/,
                                 double /*_timeStep*/,
                                 bool /*_withDampingForces*/,
                                 bool /*_withSpringForces*/)
{
  // Do nothing
}

//==============================================================================
void ZeroDofJoint::updateForceFD(const Eigen::Vector6d& /*_bodyForce*/,
                                 double /*_timeStep*/,
                                 bool /*_withDampingForces*/,
                                 bool /*_withSpringForces*/)
{
  // Do nothing
}

//==============================================================================
void ZeroDofJoint::updateImpulseID(const Eigen::Vector6d& /*_bodyImpulse*/)
{
  // Do nothing
}

//==============================================================================
void ZeroDofJoint::updateImpulseFD(const Eigen::Vector6d& /*_bodyImpulse*/)
{
  // Do nothing
}

//==============================================================================
void ZeroDofJoint::updateConstrainedTerms(double /*_timeStep*/)
{
  // Do nothing
}

//==============================================================================
void ZeroDofJoint::addChildBiasForceForInvMassMatrix(
    Eigen::Vector6d& /*_parentBiasForce*/,
    const Eigen::Matrix6d& /*_childArtInertia*/,
    const Eigen::Vector6d& /*_childBiasForce*/)
{
  // TODO(JS)
}

//==============================================================================
void ZeroDofJoint::addChildBiasForceForInvAugMassMatrix(
    Eigen::Vector6d& /*_parentBiasForce*/,
    const Eigen::Matrix6d& /*_childArtInertia*/,
    const Eigen::Vector6d& /*_childBiasForce*/)
{
  // TODO(JS)
}

//==============================================================================
void ZeroDofJoint::updateTotalForceForInvMassMatrix(
    const Eigen::Vector6d& /*_bodyForce*/)
{
  // TODO(JS)
}

//==============================================================================
void ZeroDofJoint::getInvMassMatrixSegment(
    Eigen::MatrixXd& /*_invMassMat*/,
    const size_t /*_col*/,
    const Eigen::Matrix6d& /*_artInertia*/,
    const Eigen::Vector6d& /*_spatialAcc*/)
{
  // TODO(JS)
}

//==============================================================================
void ZeroDofJoint::getInvAugMassMatrixSegment(
    Eigen::MatrixXd& /*_invMassMat*/,
    const size_t /*_col*/,
    const Eigen::Matrix6d& /*_artInertia*/,
    const Eigen::Vector6d& /*_spatialAcc*/)
{
  // TODO(JS)
}

//==============================================================================
void ZeroDofJoint::addInvMassMatrixSegmentTo(Eigen::Vector6d& /*_acc*/)
{
  // TODO(JS)
}

//==============================================================================
Eigen::VectorXd ZeroDofJoint::getSpatialToGeneralized(
    const Eigen::Vector6d& /*_spatial*/)
{
  // Return zero size vector
  return Eigen::VectorXd::Zero(0);
}

}  // namespace dynamics
}  // namespace dart
