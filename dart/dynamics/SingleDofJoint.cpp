/*
 * Copyright (c) 2014, Georgia Tech Research Corporation
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

#include "dart/dynamics/SingleDofJoint.h"

#include "dart/common/Console.h"
#include "dart/math/Helpers.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Skeleton.h"

namespace dart {
namespace dynamics {

//==============================================================================
SingleDofJoint::SingleDofJoint(const std::string& _name)
  : Joint(_name),
    mIndexInSkeleton(0u),
    mCommand(0.0),
    mPosition(0.0),
    mPositionLowerLimit(-DART_DBL_INF),
    mPositionUpperLimit(DART_DBL_INF),
    mPositionDeriv(0.0),
    mVelocity(0.0),
    mVelocityLowerLimit(-DART_DBL_INF),
    mVelocityUpperLimit(DART_DBL_INF),
    mVelocityDeriv(0.0),
    mAcceleration(0.0),
    mAccelerationLowerLimit(-DART_DBL_INF),
    mAccelerationUpperLimit(DART_DBL_INF),
    mAccelerationDeriv(0.0),
    mForce(0.0),
    mForceLowerLimit(-DART_DBL_INF),
    mForceUpperLimit(DART_DBL_INF),
    mForceDeriv(0.0),
    mVelocityChange(0.0),
    mImpulse(0.0),
    mConstraintImpulse(0.0),
    mSpringStiffness(0.0),
    mRestPosition(0.0),
    mDampingCoefficient(0.0),
    mJacobian(Eigen::Vector6d::Zero()),
    mJacobianDeriv(Eigen::Vector6d::Zero()),
    mInvProjArtInertia(0.0),
    mInvProjArtInertiaImplicit(0.0),
    mTotalForce(0.0),
    mTotalImpulse(0.0)
{
}

//==============================================================================
SingleDofJoint::~SingleDofJoint()
{
}

//==============================================================================
size_t SingleDofJoint::getDof() const
{
  return getNumDofs();
}

//==============================================================================
size_t SingleDofJoint::getNumDofs() const
{
  return 1;
}

//==============================================================================
void SingleDofJoint::setIndexInSkeleton(size_t _index, size_t _indexInSkeleton)
{
  if (_index != 0)
  {
    dterr << "setIndexInSkeleton index[" << _index << "] out of range"
          << std::endl;
    return;
  }

  mIndexInSkeleton = _indexInSkeleton;
}

//==============================================================================
size_t SingleDofJoint::getIndexInSkeleton(size_t _index) const
{
  if (_index != 0)
  {
    dterr << "getIndexInSkeleton index[" << _index << "] out of range"
          << std::endl;
    return 0;
  }

  return mIndexInSkeleton;
}

//==============================================================================
void SingleDofJoint::setCommand(size_t _index, double _command)
{
  if (_index != 0)
  {
    dterr << "[SingleDofJoint::setCommand]: index[" << _index << "] out of range"
          << std::endl;
    return;
  }

  mCommand = _command;
}

//==============================================================================
double SingleDofJoint::getCommand(size_t _index) const
{
  if (_index != 0)
  {
    dterr << "[SingleDofJoint::getCommand]: index[" << _index << "] out of range"
          << std::endl;
    return 0.0;
  }

  return mCommand;
}

//==============================================================================
void SingleDofJoint::setCommands(const Eigen::VectorXd& _commands)
{
  if (static_cast<size_t>(_commands.size()) != getNumDofs())
  {
    dterr << "[SingleDofJoint::setCommands]: commands's size["
          << _commands.size() << "] is different with the dof [" << getNumDofs()
          << "]" << std::endl;
    return;
  }

  mCommand = _commands[0];
}

//==============================================================================
Eigen::VectorXd SingleDofJoint::getCommands() const
{
  return Eigen::Matrix<double, 1, 1>::Constant(mCommand);
}

//==============================================================================
void SingleDofJoint::resetCommands()
{
  mCommand = 0.0;
}

//==============================================================================
void SingleDofJoint::setPosition(size_t _index, double _position)
{
  if (_index != 0)
  {
    dterr << "setPosition index[" << _index << "] out of range" << std::endl;
    return;
  }

  mPosition = _position;
}

//==============================================================================
double SingleDofJoint::getPosition(size_t _index) const
{
  if (_index != 0)
  {
    dterr << "getPosition index[" << _index << "] out of range" << std::endl;
    return 0.0;
  }

  return mPosition;
}

//==============================================================================
void SingleDofJoint::setPositions(const Eigen::VectorXd& _positions)
{
  if (static_cast<size_t>(_positions.size()) != getNumDofs())
  {
    dterr << "setPositions positions's size[" << _positions.size()
          << "] is different with the dof [" << getNumDofs() << "]" << std::endl;
    return;
  }

  mPosition = _positions[0];
}

//==============================================================================
Eigen::VectorXd SingleDofJoint::getPositions() const
{
  return Eigen::Matrix<double, 1, 1>::Constant(mPosition);
}

//==============================================================================
void SingleDofJoint::resetPositions()
{
  mPosition = 0.0;
}

//==============================================================================
void SingleDofJoint::setPositionLowerLimit(size_t _index, double _position)
{
  if (_index != 0)
  {
    dterr << "setPositionLowerLimit index[" << _index << "] out of range"
          << std::endl;
    return;
  }

  mPositionLowerLimit = _position;
}

//==============================================================================
double SingleDofJoint::getPositionLowerLimit(size_t _index) const
{
  if (_index != 0)
  {
    dterr << "getPositionLowerLimit index[" << _index << "] out of range"
          << std::endl;
    return 0.0;
  }

  return mPositionLowerLimit;
}

//==============================================================================
void SingleDofJoint::setPositionUpperLimit(size_t _index, double _position)
{
  if (_index != 0)
  {
    dterr << "setPositionUpperLimit index[" << _index << "] out of range"
          << std::endl;
    return;
  }

  mPositionUpperLimit = _position;
}

//==============================================================================
double SingleDofJoint::getPositionUpperLimit(size_t _index) const
{
  if (_index != 0)
  {
    dterr << "getPositionUpperLimit index[" << _index << "] out of range"
          << std::endl;
    return 0.0;
  }

  return mPositionUpperLimit;
}

//==============================================================================
void SingleDofJoint::setVelocity(size_t _index, double _velocity)
{
  if (_index != 0)
  {
    dterr << "setVelocity index[" << _index << "] out of range" << std::endl;
    return;
  }

  mVelocity = _velocity;
}

//==============================================================================
double SingleDofJoint::getVelocity(size_t _index) const
{
  if (_index != 0)
  {
    dterr << "getVelocity index[" << _index << "] out of range" << std::endl;
    return 0.0;
  }

  return mVelocity;
}

//==============================================================================
void SingleDofJoint::setVelocities(const Eigen::VectorXd& _velocities)
{
  if (static_cast<size_t>(_velocities.size()) != getNumDofs())
  {
    dterr << "setVelocities velocities's size[" << _velocities.size()
          << "] is different with the dof [" << getNumDofs() << "]" << std::endl;
    return;
  }

  mVelocity = _velocities[0];
}

//==============================================================================
Eigen::VectorXd SingleDofJoint::getVelocities() const
{
  return Eigen::Matrix<double, 1, 1>::Constant(mVelocity);
}

//==============================================================================
void SingleDofJoint::resetVelocities()
{
  mVelocity = 0.0;
}

//==============================================================================
void SingleDofJoint::setVelocityLowerLimit(size_t _index, double _velocity)
{
  if (_index != 0)
  {
    dterr << "setVelocityLowerLimit index[" << _index << "] out of range"
          << std::endl;
    return;
  }

  mVelocityLowerLimit = _velocity;
}

//==============================================================================
double SingleDofJoint::getVelocityLowerLimit(size_t _index) const
{
  if (_index != 0)
  {
    dterr << "getVelocityLowerLimit index[" << _index << "] out of range"
          << std::endl;
    return 0.0;
  }

  return mVelocityLowerLimit;
}

//==============================================================================
void SingleDofJoint::setVelocityUpperLimit(size_t _index, double _velocity)
{
  if (_index != 0)
  {
    dterr << "setVelocityUpperLimit index[" << _index << "] out of range"
          << std::endl;
    return;
  }

  mVelocityUpperLimit = _velocity;
}

//==============================================================================
double SingleDofJoint::getVelocityUpperLimit(size_t _index) const
{
  if (_index != 0)
  {
    dterr << "getVelocityUpperLimit index[" << _index << "] out of range"
          << std::endl;
    return 0.0;
  }

  return mVelocityUpperLimit;
}

//==============================================================================
void SingleDofJoint::setAcceleration(size_t _index, double _acceleration)
{
  if (_index != 0)
  {
    dterr << "setAcceleration index[" << _index << "] out of range"
          << std::endl;
    return;
  }

  mAcceleration = _acceleration;

#if DART_MAJOR_VERSION == 4
  if (mActuatorType == ACCELERATION)
    mCommand = mAcceleration;
#endif
}

//==============================================================================
double SingleDofJoint::getAcceleration(size_t _index) const
{
  if (_index != 0)
  {
    dterr << "getAcceleration index[" << _index << "] out of range"
          << std::endl;
    return 0.0;
  }

  return mAcceleration;
}

//==============================================================================
void SingleDofJoint::setAccelerations(const Eigen::VectorXd& _accelerations)
{
  if (static_cast<size_t>(_accelerations.size()) != getNumDofs())
  {
    dterr << "setAccelerations accelerations's size[" << _accelerations.size()
          << "] is different with the dof [" << getNumDofs() << "]" << std::endl;
    return;
  }

  mAcceleration = _accelerations[0];

#if DART_MAJOR_VERSION == 4
  if (mActuatorType == ACCELERATION)
    mCommand = mAcceleration;
#endif
}

//==============================================================================
Eigen::VectorXd SingleDofJoint::getAccelerations() const
{
  return Eigen::Matrix<double, 1, 1>::Constant(mAcceleration);
}

//==============================================================================
void SingleDofJoint::resetAccelerations()
{
  mAcceleration = 0.0;
}

//==============================================================================
void SingleDofJoint::setAccelerationLowerLimit(size_t _index,
                                               double _acceleration)
{
  if (_index != 0)
  {
    dterr << "setAccelerationLowerLimit index[" << _index << "] out of range"
          << std::endl;
    return;
  }

  mAccelerationLowerLimit = _acceleration;
}

//==============================================================================
double SingleDofJoint::getAccelerationLowerLimit(size_t _index) const
{
  if (_index != 0)
  {
    dterr << "getAccelerationLowerLimit index[" << _index << "] out of range"
          << std::endl;
    return 0.0;
  }

  return mAccelerationLowerLimit;
}

//==============================================================================
void SingleDofJoint::setAccelerationUpperLimit(size_t _index,
                                               double _acceleration)
{
  if (_index != 0)
  {
    dterr << "setAccelerationUpperLimit index[" << _index << "] out of range"
          << std::endl;
    return;
  }

  mAccelerationUpperLimit = _acceleration;
}

//==============================================================================
double SingleDofJoint::getAccelerationUpperLimit(size_t _index) const
{
  if (_index != 0)
  {
    dterr << "getAccelerationUpperLimit index[" << _index << "] out of range"
          << std::endl;
    return 0.0;
  }

  return mAccelerationUpperLimit;
}

//==============================================================================
void SingleDofJoint::setForce(size_t _index, double _force)
{
  if (_index != 0)
  {
    dterr << "setForce index[" << _index << "] out of range" << std::endl;
    return;
  }

  mForce = _force;

#if DART_MAJOR_VERSION == 4
  if (mActuatorType == FORCE)
    mCommand = mForce;
#endif
  // TODO: Remove at DART 5.0.
}

//==============================================================================
double SingleDofJoint::getForce(size_t _index)
{
  if (_index != 0)
  {
    dterr << "getForce index[" << _index << "] out of range" << std::endl;
    return 0.0;
  }

  return mForce;
}

//==============================================================================
void SingleDofJoint::setForces(const Eigen::VectorXd& _forces)
{
  if (static_cast<size_t>(_forces.size()) != getNumDofs())
  {
    dterr << "setForces forces's size[" << _forces.size()
          << "] is different with the dof [" << getNumDofs() << "]" << std::endl;
    return;
  }

  mForce = _forces[0];

#if DART_MAJOR_VERSION == 4
  if (mActuatorType == FORCE)
    mCommand = mForce;
#endif
  // TODO: Remove at DART 5.0.
}

//==============================================================================
Eigen::VectorXd SingleDofJoint::getForces() const
{
  return Eigen::Matrix<double, 1, 1>::Constant(mForce);
}

//==============================================================================
void SingleDofJoint::resetForces()
{
  mForce = 0.0;

#if DART_MAJOR_VERSION == 4
  if (mActuatorType == FORCE)
    mCommand = mForce;
#endif
}

//==============================================================================
void SingleDofJoint::setForceLowerLimit(size_t _index, double _force)
{
  if (_index != 0)
  {
    dterr << "setForceLowerLimit index[" << _index << "] out of range"
          << std::endl;
    return;
  }

  mForceLowerLimit = _force;
}

//==============================================================================
double SingleDofJoint::getForceLowerLimit(size_t _index) const
{
  if (_index != 0)
  {
    dterr << "getForceMin index[" << _index << "] out of range" << std::endl;
    return 0.0;
  }

  return mForceLowerLimit;
}

//==============================================================================
void SingleDofJoint::setForceUpperLimit(size_t _index, double _force)
{
  if (_index != 0)
  {
    dterr << "setForceUpperLimit index[" << _index << "] out of range"
          << std::endl;
    return;
  }

  mForceUpperLimit = _force;
}

//==============================================================================
double SingleDofJoint::getForceUpperLimit(size_t _index) const
{
  if (_index != 0)
  {
    dterr << "getForceUpperLimit index[" << _index << "] out of range"
          << std::endl;
    return 0.0;
  }

  return mForceUpperLimit;
}

//==============================================================================
void SingleDofJoint::setVelocityChange(size_t _index, double _velocityChange)
{
  if (_index != 0)
  {
    dterr << "setVelocityChange index[" << _index << "] out of range"
          << std::endl;
    return;
  }

  mVelocityChange = _velocityChange;
}

//==============================================================================
double SingleDofJoint::getVelocityChange(size_t _index) const
{
  if (_index != 0)
  {
    dterr << "getVelocityChange index[" << _index << "] out of range"
          << std::endl;
    return 0.0;
  }

  return mVelocityChange;
}

//==============================================================================
void SingleDofJoint::resetVelocityChanges()
{
  mVelocityChange = 0.0;
}

//==============================================================================
void SingleDofJoint::setConstraintImpulse(size_t _index, double _impulse)
{
  if (_index != 0)
  {
    dterr << "setConstraintImpulse index[" << _index << "] out of range"
          << std::endl;
    return;
  }

  mConstraintImpulse = _impulse;
}

//==============================================================================
double SingleDofJoint::getConstraintImpulse(size_t _index) const
{
  if (_index != 0)
  {
    dterr << "getConstraintImpulse index[" << _index << "] out of range"
          << std::endl;
    return 0.0;
  }

  return mConstraintImpulse;
}

//==============================================================================
void SingleDofJoint::resetConstraintImpulses()
{
  mConstraintImpulse = 0.0;
}

//==============================================================================
void SingleDofJoint::integratePositions(double _dt)
{
  mPosition += mVelocity * _dt;
}

//==============================================================================
void SingleDofJoint::integrateVelocities(double _dt)
{
  mVelocity += mAcceleration * _dt;
}

//==============================================================================
void SingleDofJoint::setSpringStiffness(size_t _index, double _k)
{
  if (_index != 0)
  {
    dterr << "setSpringStiffness index[" << _index << "] out of range\n";
    return;
  }

  assert(_k >= 0.0);

  mSpringStiffness = _k;
}

//==============================================================================
double SingleDofJoint::getSpringStiffness(size_t _index) const
{
  if (_index != 0)
  {
    dterr << "getSpringStiffness index[" << _index << "] out of range\n";
    return 0.0;
  }

  return mSpringStiffness;
}

//==============================================================================
void SingleDofJoint::setRestPosition(size_t _index, double _q0)
{
  if (_index != 0)
  {
    dterr << "setRestPosition index[" << _index << "] out of range\n";
    return;
  }

  if (mPositionLowerLimit > _q0 || mPositionUpperLimit < _q0)
  {
    dterr << "Rest position of joint[" << getName() << "], " << _q0
          << ", is out of the limit range["
          << mPositionLowerLimit << ", "
          << mPositionUpperLimit << "] in index[" << _index
          << "].\n";
    return;
  }

  mRestPosition = _q0;
}

//==============================================================================
double SingleDofJoint::getRestPosition(size_t _index) const
{
  if (_index != 0)
  {
    dterr << "getRestPosition index[" << _index << "] out of range\n";
    return 0.0;
  }

  return mRestPosition;
}

//==============================================================================
void SingleDofJoint::setDampingCoefficient(size_t _index, double _d)
{
  if (_index != 0)
  {
    dterr << "setDampingCoefficient index[" << _index << "] out of range\n";
    return;
  }

  assert(_d >= 0.0);

  mDampingCoefficient = _d;
}

//==============================================================================
double SingleDofJoint::getDampingCoefficient(size_t _index) const
{
  if (_index != 0)
  {
    dterr << "getDampingCoefficient index[" << _index << "] out of range\n";
    return 0.0;
  }

  return mDampingCoefficient;
}

//==============================================================================
double SingleDofJoint::getPotentialEnergy() const
{
  // Spring energy
  double pe = 0.5 * mSpringStiffness
       * (mPosition - mRestPosition)
       * (mPosition - mRestPosition);

  return pe;
}

//==============================================================================
Eigen::Vector6d SingleDofJoint::getBodyConstraintWrench() const
{
  assert(mChildBodyNode);
  return mChildBodyNode->getBodyForce() - mJacobian * mForce;
}

//==============================================================================
const math::Jacobian SingleDofJoint::getLocalJacobian() const
{
  return mJacobian;
}

//==============================================================================
const math::Jacobian SingleDofJoint::getLocalJacobianTimeDeriv() const
{
  return mJacobianDeriv;
}

//==============================================================================
void SingleDofJoint::addVelocityTo(Eigen::Vector6d& _vel)
{
  // Add joint velocity to _vel
  _vel.noalias() += mJacobian * mVelocity;

  // Verification
  assert(!math::isNan(_vel));
}

//==============================================================================
void SingleDofJoint::setPartialAccelerationTo(
    Eigen::Vector6d& _partialAcceleration,
    const Eigen::Vector6d& _childVelocity)
{
  // ad(V, S * dq) + dS * dq
  _partialAcceleration = math::ad(_childVelocity, mJacobian * mVelocity)
                         + mJacobianDeriv * mVelocity;

  // Verification
  assert(!math::isNan(_partialAcceleration));
}


//==============================================================================
void SingleDofJoint::addAccelerationTo(Eigen::Vector6d& _acc)
{
  //
  _acc += mJacobian * mAcceleration;
}

//==============================================================================
void SingleDofJoint::addVelocityChangeTo(Eigen::Vector6d& _velocityChange)
{
  //
  _velocityChange += mJacobian * mVelocityChange;
}

//==============================================================================
void SingleDofJoint::addChildArtInertiaTo(
    Eigen::Matrix6d& _parentArtInertia, const Eigen::Matrix6d& _childArtInertia)
{
  switch (mActuatorType)
  {
    case FORCE:
    case PASSIVE:
    case SERVO:
      addChildArtInertiaToForceType(_parentArtInertia,
                                       _childArtInertia);
      break;
    case ACCELERATION:
    case VELOCITY:
      addChildArtInertiaToAccelerationType(_parentArtInertia,
                                             _childArtInertia);
      break;
    default:
      dterr << "Unsupported actuator type." << std::endl;
      break;
  }
}

//==============================================================================
void SingleDofJoint::addChildArtInertiaToForceType(Eigen::Matrix6d& _parentArtInertia, const Eigen::Matrix6d& _childArtInertia)
{
  // Child body's articulated inertia
  Eigen::Vector6d AIS = _childArtInertia * mJacobian;
  Eigen::Matrix6d PI = _childArtInertia;
  PI.noalias() -= mInvProjArtInertia * AIS * AIS.transpose();
  assert(!math::isNan(PI));

  // Add child body's articulated inertia to parent body's articulated inertia.
  // Note that mT should be updated.
  _parentArtInertia += math::transformInertia(mT.inverse(), PI);
}

//==============================================================================
void SingleDofJoint::addChildArtInertiaToAccelerationType(
    Eigen::Matrix6d& _parentArtInertia, const Eigen::Matrix6d& _childArtInertia)
{
  // Add child body's articulated inertia to parent body's articulated inertia.
  // Note that mT should be updated.
  _parentArtInertia += math::transformInertia(mT.inverse(), _childArtInertia);
}

//==============================================================================
void SingleDofJoint::addChildArtInertiaImplicitTo(
    Eigen::Matrix6d& _parentArtInertia, const Eigen::Matrix6d& _childArtInertia)
{
  switch (mActuatorType)
  {
    case FORCE:
    case PASSIVE:
    case SERVO:
      addChildArtInertiaImplicitToForceType(_parentArtInertia,
                                             _childArtInertia);
      break;
    case ACCELERATION:
    case VELOCITY:
      addChildArtInertiaImplicitToAccelerationType(_parentArtInertia,
                                                   _childArtInertia);
      break;
    default:
      dterr << "Unsupported actuator type." << std::endl;
      break;
  }
}

//==============================================================================
void SingleDofJoint::addChildArtInertiaImplicitToForceType(
    Eigen::Matrix6d& _parentArtInertia, const Eigen::Matrix6d& _childArtInertia)
{
  // Child body's articulated inertia
  const Eigen::Vector6d AIS = _childArtInertia * mJacobian;
  Eigen::Matrix6d PI = _childArtInertia;
  PI.noalias() -= mInvProjArtInertiaImplicit * AIS * AIS.transpose();
  assert(!math::isNan(PI));

  // Add child body's articulated inertia to parent body's articulated inertia.
  // Note that mT should be updated.
  _parentArtInertia += math::transformInertia(mT.inverse(), PI);
}

//==============================================================================
void SingleDofJoint::addChildArtInertiaImplicitToAccelerationType(
    Eigen::Matrix6d& _parentArtInertia, const Eigen::Matrix6d& _childArtInertia)
{
  // Add child body's articulated inertia to parent body's articulated inertia.
  // Note that mT should be updated.
  _parentArtInertia += math::transformInertia(mT.inverse(), _childArtInertia);
}

//==============================================================================
void SingleDofJoint::updateInvProjArtInertia(
    const Eigen::Matrix6d& _artInertia)
{
  switch (mActuatorType)
  {
    case FORCE:
    case PASSIVE:
    case SERVO:
      updateInvProjArtInertiaForceType(_artInertia);
      break;
    case ACCELERATION:
    case VELOCITY:
      updateInvProjArtInertiaAccelerationType(_artInertia);
      break;
    default:
      dterr << "Unsupported actuator type." << std::endl;
      break;
  }
}

//==============================================================================
void SingleDofJoint::updateInvProjArtInertiaForceType(
    const Eigen::Matrix6d& _artInertia)
{
  // Projected articulated inertia
  double projAI = mJacobian.dot(_artInertia * mJacobian);

  // Inversion of projected articulated inertia
  mInvProjArtInertia = 1.0 / projAI;

  // Verification
  assert(!math::isNan(mInvProjArtInertia));
}

//==============================================================================
void SingleDofJoint::updateInvProjArtInertiaAccelerationType(
    const Eigen::Matrix6d& /*_artInertia*/)
{
  // Do nothing
}

//==============================================================================
void SingleDofJoint::updateInvProjArtInertiaImplicit(
    const Eigen::Matrix6d& _artInertia,
    double _timeStep)
{
  switch (mActuatorType)
  {
    case FORCE:
    case PASSIVE:
    case SERVO:
      updateInvProjArtInertiaImplicitForceType(_artInertia, _timeStep);
      break;
    case ACCELERATION:
    case VELOCITY:
      updateInvProjArtInertiaImplicitAccelerationType(_artInertia, _timeStep);
      break;
    default:
      dterr << "Unsupported actuator type." << std::endl;
      break;
  }
}

//==============================================================================
void SingleDofJoint::updateInvProjArtInertiaImplicitForceType(
    const Eigen::Matrix6d& _artInertia, double _timeStep)
{
  // Projected articulated inertia
  double projAI = mJacobian.dot(_artInertia * mJacobian);

  // Add additional inertia for implicit damping and spring force
  projAI += _timeStep * mDampingCoefficient
            + _timeStep * _timeStep * mSpringStiffness;

  // Inversion of the projected articulated inertia for implicit damping and
  // spring force
  mInvProjArtInertiaImplicit = 1.0 / projAI;

  // Verification
  assert(!math::isNan(mInvProjArtInertiaImplicit));
}

//==============================================================================
void SingleDofJoint::updateInvProjArtInertiaImplicitAccelerationType(
    const Eigen::Matrix6d& /*_artInertia*/, double /*_timeStep*/)
{
  // Do nothing
}

//==============================================================================
void SingleDofJoint::addChildBiasForceTo(
    Eigen::Vector6d& _parentBiasForce,
    const Eigen::Matrix6d& _childArtInertia,
    const Eigen::Vector6d& _childBiasForce,
    const Eigen::Vector6d& _childPartialAcc)
{
  switch (mActuatorType)
  {
    case FORCE:
    case PASSIVE:
    case SERVO:
      addChildBiasForceToForceType(_parentBiasForce,
                                    _childArtInertia,
                                    _childBiasForce,
                                    _childPartialAcc);
      break;
    case ACCELERATION:
    case VELOCITY:
      addChildBiasForceToAccelerationType(_parentBiasForce,
                                          _childArtInertia,
                                          _childBiasForce,
                                          _childPartialAcc);
      break;
    default:
      dterr << "Unsupported actuator type." << std::endl;
      break;
  }
}

//==============================================================================
void SingleDofJoint::addChildBiasForceToForceType(
    Eigen::Vector6d& _parentBiasForce,
    const Eigen::Matrix6d& _childArtInertia,
    const Eigen::Vector6d& _childBiasForce,
    const Eigen::Vector6d& _childPartialAcc)
{
  // Compute beta
  Eigen::Vector6d beta = _childBiasForce;
  const double coeff = mInvProjArtInertiaImplicit * mTotalForce;
  beta.noalias() += _childArtInertia*(_childPartialAcc + coeff*mJacobian);

  // Verification
  assert(!math::isNan(beta));

  // Add child body's bias force to parent body's bias force. Note that mT
  // should be updated.
  _parentBiasForce += math::dAdInvT(mT, beta);
}

//==============================================================================
void SingleDofJoint::addChildBiasForceToAccelerationType(
    Eigen::Vector6d& _parentBiasForce,
    const Eigen::Matrix6d& _childArtInertia,
    const Eigen::Vector6d& _childBiasForce,
    const Eigen::Vector6d& _childPartialAcc)
{
  // Compute beta
  Eigen::Vector6d beta = _childBiasForce;
  beta.noalias() += _childArtInertia*(_childPartialAcc
                                      + mAcceleration*mJacobian);

  // Verification
  assert(!math::isNan(beta));

  // Add child body's bias force to parent body's bias force. Note that mT
  // should be updated.
  _parentBiasForce += math::dAdInvT(mT, beta);
}

//==============================================================================
void SingleDofJoint::addChildBiasImpulseTo(
    Eigen::Vector6d& _parentBiasImpulse,
    const Eigen::Matrix6d& _childArtInertia,
    const Eigen::Vector6d& _childBiasImpulse)
{
  switch (mActuatorType)
  {
    case FORCE:
    case PASSIVE:
    case SERVO:
      addChildBiasImpulseToForceType(_parentBiasImpulse,
                                      _childArtInertia,
                                      _childBiasImpulse);
      break;
    case ACCELERATION:
    case VELOCITY:
      addChildBiasImpulseToAccelerationType(_parentBiasImpulse,
                                            _childArtInertia,
                                            _childBiasImpulse);
      break;
    default:
      dterr << "Unsupported actuator type." << std::endl;
      break;
  }
}

//==============================================================================
void SingleDofJoint::addChildBiasImpulseToForceType(
    Eigen::Vector6d& _parentBiasImpulse,
    const Eigen::Matrix6d& _childArtInertia,
    const Eigen::Vector6d& _childBiasImpulse)
{
  // Compute beta
  const Eigen::Vector6d beta
      = _childBiasImpulse
        + _childArtInertia * mJacobian * mInvProjArtInertia * mTotalImpulse;

  // Verification
  assert(!math::isNan(beta));

  // Add child body's bias force to parent body's bias force. Note that mT
  // should be updated.
  _parentBiasImpulse += math::dAdInvT(mT, beta);
}

//==============================================================================
void SingleDofJoint::addChildBiasImpulseToAccelerationType(
    Eigen::Vector6d& _parentBiasImpulse,
    const Eigen::Matrix6d& /*_childArtInertia*/,
    const Eigen::Vector6d& _childBiasImpulse)
{
  // Add child body's bias force to parent body's bias force. Note that mT
  // should be updated.
  _parentBiasImpulse += math::dAdInvT(mT, _childBiasImpulse);
}

//==============================================================================
void SingleDofJoint::updateTotalForce(const Eigen::Vector6d& _bodyForce,
                                      double _timeStep)
{
  assert(_timeStep > 0.0);

  switch (mActuatorType)
  {
    case FORCE:
      mForce = mCommand;
      updateTotalForceForceType(_bodyForce, _timeStep);
      break;
    case PASSIVE:
    case SERVO:
      mForce = 0.0;
      updateTotalForceForceType(_bodyForce, _timeStep);
      break;
    case ACCELERATION:
      mAcceleration = mCommand;
      updateTotalForceAccelerationType(_bodyForce, _timeStep);
      break;
    case VELOCITY:
      mAcceleration = (mCommand - mVelocity) / _timeStep;
      updateTotalForceAccelerationType(_bodyForce, _timeStep);
      break;
    default:
      dterr << "Unsupported actuator type." << std::endl;
      break;
  }
}

//==============================================================================
void SingleDofJoint::updateTotalForceForceType(
    const Eigen::Vector6d& _bodyForce, double _timeStep)
{
  // Spring force
  const double nextPosition = mPosition + _timeStep*mVelocity;
  const double springForce = -mSpringStiffness * (nextPosition - mRestPosition);

  // Damping force
  const double dampingForce = -mDampingCoefficient * mVelocity;

  // Compute alpha
  mTotalForce = mForce + springForce + dampingForce - mJacobian.dot(_bodyForce);
}

//==============================================================================
void SingleDofJoint::updateTotalForceAccelerationType(
    const Eigen::Vector6d& /*_bodyForce*/, double /*_timeStep*/)
{
  // Do nothing
}

//==============================================================================
void SingleDofJoint::updateTotalImpulse(const Eigen::Vector6d& _bodyImpulse)
{
  switch (mActuatorType)
  {
    case FORCE:
    case PASSIVE:
    case SERVO:
      updateTotalImpulseForceType(_bodyImpulse);
      break;
    case ACCELERATION:
    case VELOCITY:
      updateTotalImpulseAccelerationType(_bodyImpulse);
      break;
    default:
      dterr << "Unsupported actuator type." << std::endl;
      break;
  }
}

//==============================================================================
void SingleDofJoint::updateTotalImpulseForceType(
    const Eigen::Vector6d& _bodyImpulse)
{
  mTotalImpulse = mConstraintImpulse - mJacobian.dot(_bodyImpulse);
}

//==============================================================================
void SingleDofJoint::updateTotalImpulseAccelerationType(
    const Eigen::Vector6d& /*_bodyImpulse*/)
{
  // Do nothing
}

//==============================================================================
void SingleDofJoint::resetTotalImpulses()
{
  mTotalImpulse = 0.0;
}

//==============================================================================
void SingleDofJoint::updateAcceleration(const Eigen::Matrix6d& _artInertia,
                                        const Eigen::Vector6d& _spatialAcc)
{
  switch (mActuatorType)
  {
    case FORCE:
    case PASSIVE:
    case SERVO:
      updateAccelerationForceType(_artInertia, _spatialAcc);
      break;
    case ACCELERATION:
    case VELOCITY:
      updateAccelerationAccelerationType(_artInertia, _spatialAcc);
      break;
    default:
      dterr << "Unsupported actuator type." << std::endl;
      break;
  }
}

//==============================================================================
void SingleDofJoint::updateAccelerationForceType(
    const Eigen::Matrix6d& _artInertia, const Eigen::Vector6d& _spatialAcc)
{
  //
  const double val = mJacobian.dot(_artInertia * math::AdInvT(mT, _spatialAcc));
  mAcceleration = mInvProjArtInertiaImplicit * (mTotalForce - val);

  // Verification
  assert(!math::isNan(mAcceleration));
}

//==============================================================================
void SingleDofJoint::updateAccelerationAccelerationType(
    const Eigen::Matrix6d& /*_artInertia*/,
    const Eigen::Vector6d& /*_spatialAcc*/)
{
  // Do nothing
}

//==============================================================================
void SingleDofJoint::updateVelocityChange(
    const Eigen::Matrix6d& _artInertia,
    const Eigen::Vector6d& _velocityChange)
{
  switch (mActuatorType)
  {
    case FORCE:
    case PASSIVE:
    case SERVO:
      updateVelocityChangeForceType(_artInertia, _velocityChange);
      break;
    case ACCELERATION:
    case VELOCITY:
      updateVelocityChangeAccelerationType(_artInertia, _velocityChange);
      break;
    default:
      dterr << "Unsupported actuator type." << std::endl;
      break;
  }
}

//==============================================================================
void SingleDofJoint::updateVelocityChangeForceType(
    const Eigen::Matrix6d& _artInertia, const Eigen::Vector6d& _velocityChange)
{
  //
  mVelocityChange
      = mInvProjArtInertia
        * (mTotalImpulse
           - mJacobian.dot(_artInertia * math::AdInvT(mT, _velocityChange)));

  // Verification
  assert(!math::isNan(mVelocityChange));
}

//==============================================================================
void SingleDofJoint::updateVelocityChangeAccelerationType(
    const Eigen::Matrix6d& /*_artInertia*/,
    const Eigen::Vector6d& /*_velocityChange*/)
{
  // Do nothing
}

//==============================================================================
void SingleDofJoint::updateForceID(const Eigen::Vector6d& _bodyForce,
                                   double _timeStep,
                                   bool _withDampingForces,
                                   bool _withSpringForces)
{
  mForce = mJacobian.dot(_bodyForce);

  // Damping force
  if (_withDampingForces)
  {
    const double dampingForce = -mDampingCoefficient * mVelocity;
    mForce -= dampingForce;
  }

  // Spring force
  if (_withSpringForces)
  {
    const double nextPosition = mPosition + _timeStep*mVelocity;
    const double springForce = -mSpringStiffness*(nextPosition - mRestPosition);
    mForce -= springForce;
  }
}

//==============================================================================
void SingleDofJoint::updateForceFD(const Eigen::Vector6d& _bodyForce,
                                   double _timeStep,
                                   bool _withDampingForces,
                                   bool _withSpringForces)
{
  switch (mActuatorType)
  {
    case FORCE:
    case PASSIVE:
    case SERVO:
      break;
    case ACCELERATION:
    case VELOCITY:
      updateForceID(_bodyForce, _timeStep, _withDampingForces,
                    _withSpringForces);
      break;
    default:
      dterr << "Unsupported actuator type." << std::endl;
      break;
  }
}

//==============================================================================
void SingleDofJoint::updateImpulseID(const Eigen::Vector6d& _bodyImpulse)
{
  mImpulse = mJacobian.dot(_bodyImpulse);
}

//==============================================================================
void SingleDofJoint::updateImpulseFD(const Eigen::Vector6d& _bodyImpulse)
{
  switch (mActuatorType)
  {
    case FORCE:
    case PASSIVE:
    case SERVO:
      break;
    case ACCELERATION:
    case VELOCITY:
      updateImpulseID(_bodyImpulse);
      break;
    default:
      dterr << "Unsupported actuator type." << std::endl;
      break;
  }
}

//==============================================================================
void SingleDofJoint::updateConstrainedTerms(double _timeStep)
{
  switch (mActuatorType)
  {
    case FORCE:
    case PASSIVE:
    case SERVO:
      updateConstrainedTermsForceType(_timeStep);
      break;
    case ACCELERATION:
    case VELOCITY:
      updateConstrainedTermsAccelerationType(_timeStep);
      break;
    default:
      dterr << "Unsupported actuator type." << std::endl;
      break;
  }
}

//==============================================================================
void SingleDofJoint::updateConstrainedTermsForceType(double _timeStep)
{
  const double invTimeStep = 1.0 / _timeStep;

  mVelocity     += mVelocityChange;
  mAcceleration += mVelocityChange*invTimeStep;
  mForce        += mConstraintImpulse*invTimeStep;
}

//==============================================================================
void SingleDofJoint::updateConstrainedTermsAccelerationType(double _timeStep)
{
  mForce += mImpulse / _timeStep;
}

//==============================================================================
void SingleDofJoint::addChildBiasForceForInvMassMatrix(
    Eigen::Vector6d& _parentBiasForce,
    const Eigen::Matrix6d& _childArtInertia,
    const Eigen::Vector6d& _childBiasForce)
{
  // Compute beta
  Eigen::Vector6d beta = _childBiasForce;
  beta.noalias() += _childArtInertia *  mJacobian * mInvProjArtInertia
                    * mInvM_a;

  // Verification
  assert(!math::isNan(beta));

  // Add child body's bias force to parent body's bias force. Note that mT
  // should be updated.
  _parentBiasForce += math::dAdInvT(mT, beta);
}

//==============================================================================
void SingleDofJoint::addChildBiasForceForInvAugMassMatrix(
    Eigen::Vector6d& _parentBiasForce,
    const Eigen::Matrix6d& _childArtInertia,
    const Eigen::Vector6d& _childBiasForce)
{
  // Compute beta
  Eigen::Vector6d beta = _childBiasForce;
  beta.noalias() += _childArtInertia *  mJacobian * mInvProjArtInertiaImplicit
                    * mInvM_a;

  // Verification
  assert(!math::isNan(beta));

  // Add child body's bias force to parent body's bias force. Note that mT
  // should be updated.
  _parentBiasForce += math::dAdInvT(mT, beta);
}

//==============================================================================
void SingleDofJoint::updateTotalForceForInvMassMatrix(
    const Eigen::Vector6d& _bodyForce)
{
  // Compute alpha
  mInvM_a = mForce - mJacobian.dot(_bodyForce);
}

//==============================================================================
void SingleDofJoint::getInvMassMatrixSegment(Eigen::MatrixXd& _invMassMat,
                                             const size_t _col,
                                             const Eigen::Matrix6d& _artInertia,
                                             const Eigen::Vector6d& _spatialAcc)
{
  //
  mInvMassMatrixSegment
      = mInvProjArtInertia
        * (mInvM_a
           - mJacobian.dot(_artInertia * math::AdInvT(mT, _spatialAcc)));

  // Verification
  assert(!math::isNan(mInvMassMatrixSegment));

  // Index
  size_t iStart = mIndexInSkeleton;

  // Assign
  _invMassMat(iStart, _col) = mInvMassMatrixSegment;
}

//==============================================================================
void SingleDofJoint::getInvAugMassMatrixSegment(
    Eigen::MatrixXd& _invMassMat,
    const size_t _col,
    const Eigen::Matrix6d& _artInertia,
    const Eigen::Vector6d& _spatialAcc)
{
  //
  mInvMassMatrixSegment
      = mInvProjArtInertiaImplicit
        * (mInvM_a
           - mJacobian.dot(_artInertia * math::AdInvT(mT, _spatialAcc)));

  // Verification
  assert(!math::isNan(mInvMassMatrixSegment));

  // Index
  size_t iStart = mIndexInSkeleton;

  // Assign
  _invMassMat(iStart, _col) = mInvMassMatrixSegment;
}

//==============================================================================
void SingleDofJoint::addInvMassMatrixSegmentTo(Eigen::Vector6d& _acc)
{
  //
  _acc += mJacobian * mInvMassMatrixSegment;
}

//==============================================================================
Eigen::VectorXd SingleDofJoint::getSpatialToGeneralized(
    const Eigen::Vector6d& _spatial)
{
  Eigen::VectorXd generalized = Eigen::VectorXd::Zero(1);
  generalized[0] = mJacobian.dot(_spatial);
  return generalized;
}

}  // namespace dynamics
}  // namespace dart
