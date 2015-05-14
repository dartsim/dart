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

#include "dart/dynamics/SingleDofJoint.h"

#include "dart/common/Console.h"
#include "dart/math/Helpers.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/dynamics/DegreeOfFreedom.h"

namespace dart {
namespace dynamics {

//==============================================================================
SingleDofJoint::UniqueProperties::UniqueProperties(
    double _positionLowerLimit,
    double _positionUpperLimit,
    double _velocityLowerLimit,
    double _velocityUpperLimit,
    double _accelerationLowerLimit,
    double _accelerationUpperLimit,
    double _forceLowerLimit,
    double _forceUpperLimit,
    double _springStiffness,
    double _restPosition,
    double _dampingCoefficient,
    double _coulombFriction,
    bool _preserveDofName,
    std::string _dofName)
  : mPositionLowerLimit(_positionLowerLimit),
    mPositionUpperLimit(_positionUpperLimit),
    mVelocityLowerLimit(_velocityLowerLimit),
    mVelocityUpperLimit(_velocityUpperLimit),
    mAccelerationLowerLimit(_accelerationLowerLimit),
    mAccelerationUpperLimit(_accelerationUpperLimit),
    mForceLowerLimit(_forceLowerLimit),
    mForceUpperLimit(_forceUpperLimit),
    mSpringStiffness(_springStiffness),
    mRestPosition(_restPosition),
    mDampingCoefficient(_dampingCoefficient),
    mFriction(_coulombFriction),
    mPreserveDofName(_preserveDofName),
    mDofName(_dofName)
{
  // Do nothing
}

//==============================================================================
SingleDofJoint::Properties::Properties(
    const Joint::Properties& _jointProperties,
    const UniqueProperties& _singleDofProperties)
  : Joint::Properties(_jointProperties),
    UniqueProperties(_singleDofProperties)
{
  // Do nothing
}

//==============================================================================
SingleDofJoint::SingleDofJoint(const std::string& _name)
  : Joint(_name),
    mDof(createDofPointer(0)),
    mCommand(0.0),
    mPosition(0.0),
    mPositionDeriv(0.0),
    mVelocity(0.0),
    mVelocityDeriv(0.0),
    mAcceleration(0.0),
    mAccelerationDeriv(0.0),
    mForce(0.0),
    mForceDeriv(0.0),
    mVelocityChange(0.0),
    mImpulse(0.0),
    mConstraintImpulse(0.0),
    mJacobian(Eigen::Vector6d::Zero()),
    mJacobianDeriv(Eigen::Vector6d::Zero()),
    mInvProjArtInertia(0.0),
    mInvProjArtInertiaImplicit(0.0),
    mTotalForce(0.0),
    mTotalImpulse(0.0),
    mInvM_a(0.0),
    mInvMassMatrixSegment(0.0)
{
  updateDegreeOfFreedomNames();
}

//==============================================================================
SingleDofJoint::~SingleDofJoint()
{
  delete mDof;
}

//==============================================================================
void SingleDofJoint::setProperties(const Properties& _properties)
{
  Joint::setProperties(static_cast<const Joint::Properties&>(_properties));
  setProperties(static_cast<const UniqueProperties&>(_properties));
}

//==============================================================================
void SingleDofJoint::setProperties(const UniqueProperties& _properties)
{
  setDofName(0, _properties.mDofName, _properties.mPreserveDofName);
  setPositionLowerLimit(0, _properties.mPositionLowerLimit);
  setPositionUpperLimit(0, _properties.mPositionUpperLimit);
  setVelocityLowerLimit(0, _properties.mVelocityLowerLimit);
  setVelocityUpperLimit(0, _properties.mVelocityUpperLimit);
  setAccelerationLowerLimit(0, _properties.mAccelerationLowerLimit);
  setAccelerationUpperLimit(0, _properties.mAccelerationUpperLimit);
  setForceLowerLimit(0, _properties.mForceLowerLimit);
  setForceUpperLimit(0, _properties.mForceUpperLimit);
  setSpringStiffness(0, _properties.mSpringStiffness);
  setRestPosition(0, _properties.mRestPosition);
  setDampingCoefficient(0, _properties.mDampingCoefficient);
  setCoulombFriction(0, _properties.mFriction);
}

//==============================================================================
SingleDofJoint::Properties SingleDofJoint::getSingleDofJointProperties() const
{
  return Properties(mJointP, mSingleDofP);
}

//==============================================================================
void SingleDofJoint::copy(const SingleDofJoint& _otherSingleDofJoint)
{
  if(this == &_otherSingleDofJoint)
    return;

  setProperties(_otherSingleDofJoint.getSingleDofJointProperties());
}

//==============================================================================
void SingleDofJoint::copy(const SingleDofJoint* _otherSingleDofJoint)
{
  if(nullptr == _otherSingleDofJoint)
    return;

  copy(*_otherSingleDofJoint);
}

//==============================================================================
SingleDofJoint& SingleDofJoint::operator=(const SingleDofJoint& _otherJoint)
{
  copy(_otherJoint);
  return *this;
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
size_t SingleDofJoint::getIndexInSkeleton(size_t _index) const
{
  if (_index != 0)
  {
    dterr << "[SingleDofJoint::getIndexInSkeleton] index (" << _index
          << ") may only be 0\n";
    assert(false);
    return 0;
  }

  return mDof->mIndexInSkeleton;
}

//==============================================================================
size_t SingleDofJoint::getIndexInTree(size_t _index) const
{
  if (_index != 0)
  {
    dterr << "[SingleDofJoint::getIndexInTree] index (" << _index
          << ") may only be 0\n";
    assert(false);
    return 0;
  }

  return mDof->mIndexInTree;
}

//==============================================================================
DegreeOfFreedom* SingleDofJoint::getDof(size_t _index)
{
  if (0 == _index)
    return mDof;
  return nullptr;
}

//==============================================================================
const DegreeOfFreedom* SingleDofJoint::getDof(size_t _index) const
{
  if (0 == _index)
    return mDof;
  return nullptr;
}

//==============================================================================
const std::string& SingleDofJoint::setDofName(size_t _index,
                                              const std::string& _name,
                                              bool _preserveName)
{
  if (0 < _index)
  {
    dtwarn << "[SingleDofJoint::getDofName] Attempting to set the name of DOF "
           << "index " << _index << ", which is out of bounds. We will set "
           << "the name of DOF index 0 instead\n";
    _index = 0;
  }

  preserveDofName(_index, _preserveName);

  if (_name == mSingleDofP.mDofName)
    return mSingleDofP.mDofName;

  SkeletonPtr skel = mChildBodyNode? mChildBodyNode->getSkeleton() : nullptr;
  if(skel)
  {
    mSingleDofP.mDofName =
        skel->mNameMgrForDofs.changeObjectName(mDof, _name);
  }
  else
    mSingleDofP.mDofName = _name;

  return mSingleDofP.mDofName;
}

//==============================================================================
void SingleDofJoint::preserveDofName(size_t _index, bool _preserve)
{
  if (0 < _index)
    dtwarn << "[SingleDofJoint::preserveDofName] Attempting to preserve the "
           << "name of DOF index " << _index << ", which is out of bounds. We "
           << "will preserve the name of DOF index 0 instead\n";

  mSingleDofP.mPreserveDofName = _preserve;
}

//==============================================================================
bool SingleDofJoint::isDofNamePreserved(size_t _index) const
{
  if (0 < _index)
    dtwarn << "[SingleDofJoint::isDofNamePreserved] Requesting whether DOF "
           << "index " << _index << " is preserved, but this is out of bounds. "
           << "We will return the result of DOF index 0 instead\n";

  return mSingleDofP.mPreserveDofName;
}

//==============================================================================
const std::string& SingleDofJoint::getDofName(size_t _index) const
{
  if (0 < _index)
    dtwarn << "[SingleDofJoint::getDofName] Requested name of DOF index "
           << _index << ", which is out of bounds. Returning name of index 0\n";

  return mSingleDofP.mDofName;
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

  setPositionStatic(_position);
}

//==============================================================================
double SingleDofJoint::getPosition(size_t _index) const
{
  if (_index != 0)
  {
    dterr << "getPosition index[" << _index << "] out of range" << std::endl;
    return 0.0;
  }

  return getPositionStatic();
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

  setPositionStatic(_positions[0]);
}

//==============================================================================
Eigen::VectorXd SingleDofJoint::getPositions() const
{
  return Eigen::Matrix<double, 1, 1>::Constant(getPositionStatic());
}

//==============================================================================
void SingleDofJoint::resetPositions()
{
  setPositionStatic(0.0);
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

  mSingleDofP.mPositionLowerLimit = _position;
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

  return mSingleDofP.mPositionLowerLimit;
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

  mSingleDofP.mPositionUpperLimit = _position;
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

  return mSingleDofP.mPositionUpperLimit;
}

//==============================================================================
void SingleDofJoint::setVelocity(size_t _index, double _velocity)
{
  if (_index != 0)
  {
    dterr << "setVelocity index[" << _index << "] out of range" << std::endl;
    return;
  }

  setVelocityStatic(_velocity);
}

//==============================================================================
double SingleDofJoint::getVelocity(size_t _index) const
{
  if (_index != 0)
  {
    dterr << "getVelocity index[" << _index << "] out of range" << std::endl;
    return 0.0;
  }

  return getVelocityStatic();
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

  setVelocityStatic(_velocities[0]);
}

//==============================================================================
Eigen::VectorXd SingleDofJoint::getVelocities() const
{
  return Eigen::Matrix<double, 1, 1>::Constant(getVelocityStatic());
}

//==============================================================================
void SingleDofJoint::resetVelocities()
{
  setVelocityStatic(0.0);
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

  mSingleDofP.mVelocityLowerLimit = _velocity;
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

  return mSingleDofP.mVelocityLowerLimit;
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

  mSingleDofP.mVelocityUpperLimit = _velocity;
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

  return mSingleDofP.mVelocityUpperLimit;
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

  setAccelerationStatic(_acceleration);

#if DART_MAJOR_VERSION == 4
  if (mJointP.mActuatorType == ACCELERATION)
    mCommand = getAccelerationStatic();
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

  return getAccelerationStatic();
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

  setAccelerationStatic(_accelerations[0]);

#if DART_MAJOR_VERSION == 4
  if (mJointP.mActuatorType == ACCELERATION)
    mCommand = getAccelerationStatic();
#endif
}

//==============================================================================
Eigen::VectorXd SingleDofJoint::getAccelerations() const
{
  return Eigen::Matrix<double, 1, 1>::Constant(getAccelerationStatic());
}

//==============================================================================
void SingleDofJoint::resetAccelerations()
{
  setAccelerationStatic(0.0);
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

  mSingleDofP.mAccelerationLowerLimit = _acceleration;
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

  return mSingleDofP.mAccelerationLowerLimit;
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

  mSingleDofP.mAccelerationUpperLimit = _acceleration;
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

  return mSingleDofP.mAccelerationUpperLimit;
}

//==============================================================================
void SingleDofJoint::setPositionStatic(const double& _position)
{
  mPosition = _position;
  notifyPositionUpdate();
}

//==============================================================================
const double& SingleDofJoint::getPositionStatic() const
{
  return mPosition;
}

//==============================================================================
void SingleDofJoint::setVelocityStatic(const double& _velocity)
{
  mVelocity = _velocity;
  notifyVelocityUpdate();
}

//==============================================================================
const double& SingleDofJoint::getVelocityStatic() const
{
  return mVelocity;
}

//==============================================================================
void SingleDofJoint::setAccelerationStatic(const double& _acceleration)
{
  mAcceleration = _acceleration;
  notifyAccelerationUpdate();
}

//==============================================================================
const double& SingleDofJoint::getAccelerationStatic() const
{
  return mAcceleration;
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
  if (mJointP.mActuatorType == FORCE)
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
  if (mJointP.mActuatorType == FORCE)
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
  if (mJointP.mActuatorType == FORCE)
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

  mSingleDofP.mForceLowerLimit = _force;
}

//==============================================================================
double SingleDofJoint::getForceLowerLimit(size_t _index) const
{
  if (_index != 0)
  {
    dterr << "getForceMin index[" << _index << "] out of range" << std::endl;
    return 0.0;
  }

  return mSingleDofP.mForceLowerLimit;
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

  mSingleDofP.mForceUpperLimit = _force;
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

  return mSingleDofP.mForceUpperLimit;
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
  setPositionStatic(getPositionStatic() + getVelocityStatic() * _dt);
}

//==============================================================================
void SingleDofJoint::integrateVelocities(double _dt)
{
  setVelocityStatic(getVelocityStatic() + getAccelerationStatic() * _dt);
}

//==============================================================================
void SingleDofJoint::setSpringStiffness(size_t _index, double _k)
{
  if (_index != 0)
  {
    dterr << "[SingleDofJoint::setSpringStiffness]: index[" << _index
          << "] out of range." << std::endl;
    return;
  }

  assert(_k >= 0.0);

  mSingleDofP.mSpringStiffness = _k;
}

//==============================================================================
double SingleDofJoint::getSpringStiffness(size_t _index) const
{
  if (_index != 0)
  {
    dterr << "[SingleDofJoint::getSpringStiffness]: index[" << _index
          << "] out of range." << std::endl;
    return 0.0;
  }

  return mSingleDofP.mSpringStiffness;
}

//==============================================================================
void SingleDofJoint::setRestPosition(size_t _index, double _q0)
{
  if (_index != 0)
  {
    dterr << "[SingleDofJoint::setRestPosition]: index[" << _index
          << "] out of range." << std::endl;
    return;
  }

  if (mSingleDofP.mPositionLowerLimit > _q0
      || mSingleDofP.mPositionUpperLimit < _q0)
  {
    dterr << "Rest position of joint[" << getName() << "], " << _q0
          << ", is out of the limit range["
          << mSingleDofP.mPositionLowerLimit << ", "
          << mSingleDofP.mPositionUpperLimit << "] in index[" << _index
          << "].\n";
    return;
  }

  mSingleDofP.mRestPosition = _q0;
}

//==============================================================================
double SingleDofJoint::getRestPosition(size_t _index) const
{
  if (_index != 0)
  {
    dterr << "[SingleDofJoint::getRestPosition]: index[" << _index
          << "] out of range." << std::endl;
    return 0.0;
  }

  return mSingleDofP.mRestPosition;
}

//==============================================================================
void SingleDofJoint::setDampingCoefficient(size_t _index, double _d)
{
  if (_index != 0)
  {
    dterr << "[SingleDofJoint::setDampingCoefficient]: index[" << _index
          << "] out of range." << std::endl;
    return;
  }

  assert(_d >= 0.0);

  mSingleDofP.mDampingCoefficient = _d;
}

//==============================================================================
double SingleDofJoint::getDampingCoefficient(size_t _index) const
{
  if (_index != 0)
  {
    dterr << "[SingleDofJoint::getDampingCoefficient]: index[" << _index
          << "] out of range." << std::endl;
    return 0.0;
  }

  return mSingleDofP.mDampingCoefficient;
}

//==============================================================================
void SingleDofJoint::setCoulombFriction(size_t _index, double _friction)
{
  if (_index != 0)
  {
    dterr << "[SingleDofJoint::setFriction]: index[" << _index
          << "] out of range." << std::endl;
    return;
  }

  assert(_friction >= 0.0);

  mSingleDofP.mFriction = _friction;
}

//==============================================================================
double SingleDofJoint::getCoulombFriction(size_t _index) const
{
  if (_index != 0)
  {
    dterr << "[SingleDofJoint::getFriction]: index[" << _index
          << "] out of range." << std::endl;
    return 0.0;
  }

  return mSingleDofP.mFriction;
}

//==============================================================================
double SingleDofJoint::getPotentialEnergy() const
{
  // Spring energy
  double pe = 0.5 * mSingleDofP.mSpringStiffness
       * (getPositionStatic() - mSingleDofP.mRestPosition)
       * (getPositionStatic() - mSingleDofP.mRestPosition);

  return pe;
}

//==============================================================================
SingleDofJoint::SingleDofJoint(const Properties& _properties)
  : Joint(_properties),
    mDof(createDofPointer(0)),
    mCommand(0.0),
    mPosition(0.0),
    mPositionDeriv(0.0),
    mVelocity(0.0),
    mVelocityDeriv(0.0),
    mAcceleration(0.0),
    mAccelerationDeriv(0.0),
    mForce(0.0),
    mForceDeriv(0.0),
    mVelocityChange(0.0),
    mImpulse(0.0),
    mConstraintImpulse(0.0),
    mJacobian(Eigen::Vector6d::Zero()),
    mJacobianDeriv(Eigen::Vector6d::Zero()),
    mInvProjArtInertia(0.0),
    mInvProjArtInertiaImplicit(0.0),
    mTotalForce(0.0),
    mTotalImpulse(0.0),
    mInvM_a(0.0),
    mInvMassMatrixSegment(0.0)
{
  // Do nothing
}

//==============================================================================
void SingleDofJoint::registerDofs()
{
  SkeletonPtr skel = getSkeleton();
  if(skel)
    mSingleDofP.mDofName =
        skel->mNameMgrForDofs.issueNewNameAndAdd(mDof->getName(), mDof);
}

//==============================================================================
void SingleDofJoint::updateDegreeOfFreedomNames()
{
  // Same name as the joint it belongs to.
  if (!mDof->isNamePreserved())
    mDof->setName(mJointP.mName, false);
}

//==============================================================================
void SingleDofJoint::updateLocalSpatialVelocity() const
{
  mSpatialVelocity = getLocalJacobianStatic() * getVelocityStatic();
}

//==============================================================================
void SingleDofJoint::updateLocalSpatialAcceleration() const
{
  mSpatialAcceleration = getLocalPrimaryAcceleration()
                       + getLocalJacobianTimeDerivStatic() * getVelocityStatic();
}

//==============================================================================
void SingleDofJoint::updateLocalPrimaryAcceleration() const
{
  mPrimaryAcceleration = getLocalJacobianStatic() * getAccelerationStatic();
}

//==============================================================================
Eigen::Vector6d SingleDofJoint::getBodyConstraintWrench() const
{
  assert(mChildBodyNode);
  return mChildBodyNode->getBodyForce() - getLocalJacobianStatic() * mForce;
}

//==============================================================================
const math::Jacobian SingleDofJoint::getLocalJacobian() const
{
  if(mIsLocalJacobianDirty)
  {
    updateLocalJacobian(false);
    mIsLocalJacobianDirty = false;
  }
  return mJacobian;
}

//==============================================================================
const Eigen::Vector6d& SingleDofJoint::getLocalJacobianStatic() const
{
  if(mIsLocalJacobianDirty)
  {
    updateLocalJacobian(false);
    mIsLocalJacobianDirty = false;
  }
  return mJacobian;
}

//==============================================================================
const math::Jacobian SingleDofJoint::getLocalJacobianTimeDeriv() const
{
  if(mIsLocalJacobianTimeDerivDirty)
  {
    updateLocalJacobianTimeDeriv();
    mIsLocalJacobianTimeDerivDirty = false;
  }
  return mJacobianDeriv;
}

//==============================================================================
const Eigen::Vector6d& SingleDofJoint::getLocalJacobianTimeDerivStatic() const
{
  if(mIsLocalJacobianTimeDerivDirty)
  {
    updateLocalJacobianTimeDeriv();
    mIsLocalJacobianTimeDerivDirty = false;
  }
  return mJacobianDeriv;
}

//==============================================================================
const double& SingleDofJoint::getInvProjArtInertia() const
{
  Joint::updateArticulatedInertia();
  return mInvProjArtInertia;
}

//==============================================================================
const double& SingleDofJoint::getInvProjArtInertiaImplicit() const
{
  Joint::updateArticulatedInertia();
  return mInvProjArtInertiaImplicit;
}

//==============================================================================
void SingleDofJoint::addVelocityTo(Eigen::Vector6d& _vel)
{
  // Add joint velocity to _vel
  _vel.noalias() += getLocalJacobianStatic() * getVelocityStatic();

  // Verification
  assert(!math::isNan(_vel));
}

//==============================================================================
void SingleDofJoint::setPartialAccelerationTo(
    Eigen::Vector6d& _partialAcceleration,
    const Eigen::Vector6d& _childVelocity)
{
  // ad(V, S * dq) + dS * dq
  _partialAcceleration = math::ad(_childVelocity, getLocalJacobianStatic()
                                  * getVelocityStatic())
                         + mJacobianDeriv * getVelocityStatic();
  // Verification
  assert(!math::isNan(_partialAcceleration));
}


//==============================================================================
void SingleDofJoint::addAccelerationTo(Eigen::Vector6d& _acc)
{
  //
  _acc += getLocalJacobianStatic() * getAccelerationStatic();
}

//==============================================================================
void SingleDofJoint::addVelocityChangeTo(Eigen::Vector6d& _velocityChange)
{
  //
  _velocityChange += getLocalJacobianStatic() * mVelocityChange;
}

//==============================================================================
void SingleDofJoint::addChildArtInertiaTo(
    Eigen::Matrix6d& _parentArtInertia, const Eigen::Matrix6d& _childArtInertia)
{
  switch (mJointP.mActuatorType)
  {
    case FORCE:
    case PASSIVE:
    case SERVO:
      addChildArtInertiaToDynamic(_parentArtInertia,
                                       _childArtInertia);
      break;
    case ACCELERATION:
    case VELOCITY:
    case LOCKED:
      addChildArtInertiaToKinematic(_parentArtInertia,
                                             _childArtInertia);
      break;
    default:
      dterr << "Unsupported actuator type." << std::endl;
      break;
  }
}

//==============================================================================
void SingleDofJoint::addChildArtInertiaToDynamic(
    Eigen::Matrix6d& _parentArtInertia, const Eigen::Matrix6d& _childArtInertia)
{
  // Child body's articulated inertia
  Eigen::Vector6d AIS = _childArtInertia * getLocalJacobianStatic();
  Eigen::Matrix6d PI = _childArtInertia;
  PI.noalias() -= mInvProjArtInertia * AIS * AIS.transpose();
  assert(!math::isNan(PI));

  // Add child body's articulated inertia to parent body's articulated inertia.
  // Note that mT should be updated.
  _parentArtInertia += math::transformInertia(getLocalTransform().inverse(), PI);
}

//==============================================================================
void SingleDofJoint::addChildArtInertiaToKinematic(
    Eigen::Matrix6d& _parentArtInertia, const Eigen::Matrix6d& _childArtInertia)
{
  // Add child body's articulated inertia to parent body's articulated inertia.
  // Note that mT should be updated.
  _parentArtInertia += math::transformInertia(getLocalTransform().inverse(),
                                              _childArtInertia);
}

//==============================================================================
void SingleDofJoint::addChildArtInertiaImplicitTo(
    Eigen::Matrix6d& _parentArtInertia, const Eigen::Matrix6d& _childArtInertia)
{
  switch (mJointP.mActuatorType)
  {
    case FORCE:
    case PASSIVE:
    case SERVO:
      addChildArtInertiaImplicitToDynamic(_parentArtInertia,
                                             _childArtInertia);
      break;
    case ACCELERATION:
    case VELOCITY:
    case LOCKED:
      addChildArtInertiaImplicitToKinematic(_parentArtInertia,
                                                   _childArtInertia);
      break;
    default:
      dterr << "Unsupported actuator type." << std::endl;
      break;
  }
}

//==============================================================================
void SingleDofJoint::addChildArtInertiaImplicitToDynamic(
    Eigen::Matrix6d& _parentArtInertia, const Eigen::Matrix6d& _childArtInertia)
{
  // Child body's articulated inertia
  const Eigen::Vector6d AIS = _childArtInertia * getLocalJacobianStatic();
  Eigen::Matrix6d PI = _childArtInertia;
  PI.noalias() -= mInvProjArtInertiaImplicit * AIS * AIS.transpose();
  assert(!math::isNan(PI));

  // Add child body's articulated inertia to parent body's articulated inertia.
  // Note that mT should be updated.
  _parentArtInertia += math::transformInertia(getLocalTransform().inverse(), PI);
}

//==============================================================================
void SingleDofJoint::addChildArtInertiaImplicitToKinematic(
    Eigen::Matrix6d& _parentArtInertia, const Eigen::Matrix6d& _childArtInertia)
{
  // Add child body's articulated inertia to parent body's articulated inertia.
  // Note that mT should be updated.
  _parentArtInertia += math::transformInertia(getLocalTransform().inverse(),
                                              _childArtInertia);
}

//==============================================================================
void SingleDofJoint::updateInvProjArtInertia(
    const Eigen::Matrix6d& _artInertia)
{
  switch (mJointP.mActuatorType)
  {
    case FORCE:
    case PASSIVE:
    case SERVO:
      updateInvProjArtInertiaDynamic(_artInertia);
      break;
    case ACCELERATION:
    case VELOCITY:
    case LOCKED:
      updateInvProjArtInertiaKinematic(_artInertia);
      break;
    default:
      dterr << "Unsupported actuator type." << std::endl;
      break;
  }
}

//==============================================================================
void SingleDofJoint::updateInvProjArtInertiaDynamic(
    const Eigen::Matrix6d& _artInertia)
{
  // Projected articulated inertia
  const Eigen::Vector6d& Jacobian = getLocalJacobianStatic();
  double projAI = Jacobian.dot(_artInertia * Jacobian);

  // Inversion of projected articulated inertia
  mInvProjArtInertia = 1.0 / projAI;

  // Verification
  assert(!math::isNan(mInvProjArtInertia));
}

//==============================================================================
void SingleDofJoint::updateInvProjArtInertiaKinematic(
    const Eigen::Matrix6d& /*_artInertia*/)
{
  // Do nothing
}

//==============================================================================
void SingleDofJoint::updateInvProjArtInertiaImplicit(
    const Eigen::Matrix6d& _artInertia,
    double _timeStep)
{
  switch (mJointP.mActuatorType)
  {
    case FORCE:
    case PASSIVE:
    case SERVO:
      updateInvProjArtInertiaImplicitDynamic(_artInertia, _timeStep);
      break;
    case ACCELERATION:
    case VELOCITY:
    case LOCKED:
      updateInvProjArtInertiaImplicitKinematic(_artInertia, _timeStep);
      break;
    default:
      dterr << "Unsupported actuator type." << std::endl;
      break;
  }
}

//==============================================================================
void SingleDofJoint::updateInvProjArtInertiaImplicitDynamic(
    const Eigen::Matrix6d& _artInertia, double _timeStep)
{
  // Projected articulated inertia
  const Eigen::Vector6d& Jacobian = getLocalJacobianStatic();
  double projAI = Jacobian.dot(_artInertia * Jacobian);

  // Add additional inertia for implicit damping and spring force
  projAI += _timeStep * mSingleDofP.mDampingCoefficient
            + _timeStep * _timeStep * mSingleDofP.mSpringStiffness;

  // Inversion of the projected articulated inertia for implicit damping and
  // spring force
  mInvProjArtInertiaImplicit = 1.0 / projAI;

  // Verification
  assert(!math::isNan(mInvProjArtInertiaImplicit));
}

//==============================================================================
void SingleDofJoint::updateInvProjArtInertiaImplicitKinematic(
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
  switch (mJointP.mActuatorType)
  {
    case FORCE:
    case PASSIVE:
    case SERVO:
      addChildBiasForceToDynamic(_parentBiasForce,
                                    _childArtInertia,
                                    _childBiasForce,
                                    _childPartialAcc);
      break;
    case ACCELERATION:
    case VELOCITY:
    case LOCKED:
      addChildBiasForceToKinematic(_parentBiasForce,
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
void SingleDofJoint::addChildBiasForceToDynamic(
    Eigen::Vector6d& _parentBiasForce,
    const Eigen::Matrix6d& _childArtInertia,
    const Eigen::Vector6d& _childBiasForce,
    const Eigen::Vector6d& _childPartialAcc)
{
  // Compute beta
  Eigen::Vector6d beta = _childBiasForce;
  const double coeff = getInvProjArtInertiaImplicit() * mTotalForce;
  beta.noalias() += _childArtInertia*(_childPartialAcc
                                      + coeff*getLocalJacobianStatic());

  // Verification
  assert(!math::isNan(beta));

  // Add child body's bias force to parent body's bias force. Note that mT
  // should be updated.
  _parentBiasForce += math::dAdInvT(getLocalTransform(), beta);
}

//==============================================================================
void SingleDofJoint::addChildBiasForceToKinematic(
    Eigen::Vector6d& _parentBiasForce,
    const Eigen::Matrix6d& _childArtInertia,
    const Eigen::Vector6d& _childBiasForce,
    const Eigen::Vector6d& _childPartialAcc)
{
  // Compute beta
  Eigen::Vector6d beta = _childBiasForce;
  beta.noalias() += _childArtInertia*(_childPartialAcc
                            + getAccelerationStatic()*getLocalJacobianStatic());

  // Verification
  assert(!math::isNan(beta));

  // Add child body's bias force to parent body's bias force. Note that mT
  // should be updated.
  _parentBiasForce += math::dAdInvT(getLocalTransform(), beta);
}

//==============================================================================
void SingleDofJoint::addChildBiasImpulseTo(
    Eigen::Vector6d& _parentBiasImpulse,
    const Eigen::Matrix6d& _childArtInertia,
    const Eigen::Vector6d& _childBiasImpulse)
{
  switch (mJointP.mActuatorType)
  {
    case FORCE:
    case PASSIVE:
    case SERVO:
      addChildBiasImpulseToDynamic(_parentBiasImpulse,
                                      _childArtInertia,
                                      _childBiasImpulse);
      break;
    case ACCELERATION:
    case VELOCITY:
    case LOCKED:
      addChildBiasImpulseToKinematic(_parentBiasImpulse,
                                            _childArtInertia,
                                            _childBiasImpulse);
      break;
    default:
      dterr << "Unsupported actuator type." << std::endl;
      break;
  }
}

//==============================================================================
void SingleDofJoint::addChildBiasImpulseToDynamic(
    Eigen::Vector6d& _parentBiasImpulse,
    const Eigen::Matrix6d& _childArtInertia,
    const Eigen::Vector6d& _childBiasImpulse)
{
  // Compute beta
  const Eigen::Vector6d beta
      = _childBiasImpulse
        + _childArtInertia * getLocalJacobianStatic()
          * getInvProjArtInertia() * mTotalImpulse;

  // Verification
  assert(!math::isNan(beta));

  // Add child body's bias force to parent body's bias force. Note that mT
  // should be updated.
  _parentBiasImpulse += math::dAdInvT(getLocalTransform(), beta);
}

//==============================================================================
void SingleDofJoint::addChildBiasImpulseToKinematic(
    Eigen::Vector6d& _parentBiasImpulse,
    const Eigen::Matrix6d& /*_childArtInertia*/,
    const Eigen::Vector6d& _childBiasImpulse)
{
  // Add child body's bias force to parent body's bias force. Note that mT
  // should be updated.
  _parentBiasImpulse += math::dAdInvT(getLocalTransform(), _childBiasImpulse);
}

//==============================================================================
void SingleDofJoint::updateTotalForce(const Eigen::Vector6d& _bodyForce,
                                      double _timeStep)
{
  assert(_timeStep > 0.0);

  switch (mJointP.mActuatorType)
  {
    case FORCE:
      mForce = mCommand;
      updateTotalForceDynamic(_bodyForce, _timeStep);
      break;
    case PASSIVE:
    case SERVO:
      mForce = 0.0;
      updateTotalForceDynamic(_bodyForce, _timeStep);
      break;
    case ACCELERATION:
      setAccelerationStatic(mCommand);
      updateTotalForceKinematic(_bodyForce, _timeStep);
      break;
    case VELOCITY:
      setAccelerationStatic( (mCommand - getVelocityStatic()) / _timeStep );
      updateTotalForceKinematic(_bodyForce, _timeStep);
      break;
    case LOCKED:
      setVelocityStatic(0.0);
      setAccelerationStatic(0.0);
      updateTotalForceKinematic(_bodyForce, _timeStep);
      break;
    default:
      dterr << "Unsupported actuator type." << std::endl;
      break;
  }
}

//==============================================================================
void SingleDofJoint::updateTotalForceDynamic(
    const Eigen::Vector6d& _bodyForce, double _timeStep)
{
  // Spring force
  const double nextPosition =
      getPositionStatic() + _timeStep*getVelocityStatic();
  const double springForce =
     -mSingleDofP.mSpringStiffness * (nextPosition - mSingleDofP.mRestPosition);

  // Damping force
  const double dampingForce =
      -mSingleDofP.mDampingCoefficient * getVelocityStatic();

  // Compute alpha
  mTotalForce = mForce + springForce + dampingForce
                - getLocalJacobianStatic().dot(_bodyForce);
}

//==============================================================================
void SingleDofJoint::updateTotalForceKinematic(
    const Eigen::Vector6d& /*_bodyForce*/, double /*_timeStep*/)
{
  // Do nothing
}

//==============================================================================
void SingleDofJoint::updateTotalImpulse(const Eigen::Vector6d& _bodyImpulse)
{
  switch (mJointP.mActuatorType)
  {
    case FORCE:
    case PASSIVE:
    case SERVO:
      updateTotalImpulseDynamic(_bodyImpulse);
      break;
    case ACCELERATION:
    case VELOCITY:
    case LOCKED:
      updateTotalImpulseKinematic(_bodyImpulse);
      break;
    default:
      dterr << "Unsupported actuator type." << std::endl;
      break;
  }
}

//==============================================================================
void SingleDofJoint::updateTotalImpulseDynamic(
    const Eigen::Vector6d& _bodyImpulse)
{
  mTotalImpulse = mConstraintImpulse - getLocalJacobianStatic().dot(_bodyImpulse);
}

//==============================================================================
void SingleDofJoint::updateTotalImpulseKinematic(
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
  switch (mJointP.mActuatorType)
  {
    case FORCE:
    case PASSIVE:
    case SERVO:
      updateAccelerationDynamic(_artInertia, _spatialAcc);
      break;
    case ACCELERATION:
    case VELOCITY:
    case LOCKED:
      updateAccelerationKinematic(_artInertia, _spatialAcc);
      break;
    default:
      dterr << "Unsupported actuator type." << std::endl;
      break;
  }
}

//==============================================================================
void SingleDofJoint::updateAccelerationDynamic(
    const Eigen::Matrix6d& _artInertia, const Eigen::Vector6d& _spatialAcc)
{
  //
  const double val = getLocalJacobianStatic().dot(
        _artInertia * math::AdInvT(getLocalTransform(), _spatialAcc));
  setAccelerationStatic(getInvProjArtInertiaImplicit() * (mTotalForce - val));

  // Verification
  assert(!math::isNan(getAccelerationStatic()));
}

//==============================================================================
void SingleDofJoint::updateAccelerationKinematic(
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
  switch (mJointP.mActuatorType)
  {
    case FORCE:
    case PASSIVE:
    case SERVO:
      updateVelocityChangeDynamic(_artInertia, _velocityChange);
      break;
    case ACCELERATION:
    case VELOCITY:
    case LOCKED:
      updateVelocityChangeKinematic(_artInertia, _velocityChange);
      break;
    default:
      dterr << "Unsupported actuator type." << std::endl;
      break;
  }
}

//==============================================================================
void SingleDofJoint::updateVelocityChangeDynamic(
    const Eigen::Matrix6d& _artInertia, const Eigen::Vector6d& _velocityChange)
{
  //
  mVelocityChange
      = getInvProjArtInertia()
        * (mTotalImpulse
           - getLocalJacobianStatic().dot(
             _artInertia * math::AdInvT(getLocalTransform(), _velocityChange)));

  // Verification
  assert(!math::isNan(mVelocityChange));
}

//==============================================================================
void SingleDofJoint::updateVelocityChangeKinematic(
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
  mForce = getLocalJacobianStatic().dot(_bodyForce);

  // Damping force
  if (_withDampingForces)
  {
    const double dampingForce =
        -mSingleDofP.mDampingCoefficient * getVelocityStatic();
    mForce -= dampingForce;
  }

  // Spring force
  if (_withSpringForces)
  {
    const double nextPosition = getPositionStatic()
                              + _timeStep*getVelocityStatic();
    const double springForce =
       -mSingleDofP.mSpringStiffness*(nextPosition - mSingleDofP.mRestPosition);
    mForce -= springForce;
  }
}

//==============================================================================
void SingleDofJoint::updateForceFD(const Eigen::Vector6d& _bodyForce,
                                   double _timeStep,
                                   bool _withDampingForces,
                                   bool _withSpringForces)
{
  switch (mJointP.mActuatorType)
  {
    case FORCE:
    case PASSIVE:
    case SERVO:
      break;
    case ACCELERATION:
    case VELOCITY:
    case LOCKED:
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
  mImpulse = getLocalJacobianStatic().dot(_bodyImpulse);
}

//==============================================================================
void SingleDofJoint::updateImpulseFD(const Eigen::Vector6d& _bodyImpulse)
{
  switch (mJointP.mActuatorType)
  {
    case FORCE:
    case PASSIVE:
    case SERVO:
      break;
    case ACCELERATION:
    case VELOCITY:
    case LOCKED:
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
  switch (mJointP.mActuatorType)
  {
    case FORCE:
    case PASSIVE:
    case SERVO:
      updateConstrainedTermsDynamic(_timeStep);
      break;
    case ACCELERATION:
    case VELOCITY:
    case LOCKED:
      updateConstrainedTermsKinematic(_timeStep);
      break;
    default:
      dterr << "Unsupported actuator type." << std::endl;
      break;
  }
}

//==============================================================================
void SingleDofJoint::updateConstrainedTermsDynamic(double _timeStep)
{
  const double invTimeStep = 1.0 / _timeStep;

  setVelocityStatic(getVelocityStatic() + mVelocityChange);
  setAccelerationStatic(getAccelerationStatic() + mVelocityChange*invTimeStep);
  mForce        += mConstraintImpulse*invTimeStep;
}

//==============================================================================
void SingleDofJoint::updateConstrainedTermsKinematic(double _timeStep)
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
  beta.noalias() += _childArtInertia * getLocalJacobianStatic()
                    * getInvProjArtInertia() * mInvM_a;

  // Verification
  assert(!math::isNan(beta));

  // Add child body's bias force to parent body's bias force. Note that mT
  // should be updated.
  _parentBiasForce += math::dAdInvT(getLocalTransform(), beta);
}

//==============================================================================
void SingleDofJoint::addChildBiasForceForInvAugMassMatrix(
    Eigen::Vector6d& _parentBiasForce,
    const Eigen::Matrix6d& _childArtInertia,
    const Eigen::Vector6d& _childBiasForce)
{
  // Compute beta
  Eigen::Vector6d beta = _childBiasForce;
  beta.noalias() += _childArtInertia * getLocalJacobianStatic()
                    * getInvProjArtInertiaImplicit() * mInvM_a;

  // Verification
  assert(!math::isNan(beta));

  // Add child body's bias force to parent body's bias force. Note that mT
  // should be updated.
  _parentBiasForce += math::dAdInvT(getLocalTransform(), beta);
}

//==============================================================================
void SingleDofJoint::updateTotalForceForInvMassMatrix(
    const Eigen::Vector6d& _bodyForce)
{
  // Compute alpha
  mInvM_a = mForce - getLocalJacobianStatic().dot(_bodyForce);
}

//==============================================================================
void SingleDofJoint::getInvMassMatrixSegment(Eigen::MatrixXd& _invMassMat,
                                             const size_t _col,
                                             const Eigen::Matrix6d& _artInertia,
                                             const Eigen::Vector6d& _spatialAcc)
{
  //
  mInvMassMatrixSegment
      = getInvProjArtInertia()
        * (mInvM_a - getLocalJacobianStatic().dot(
             _artInertia * math::AdInvT(getLocalTransform(), _spatialAcc)));

  // Verification
  assert(!math::isNan(mInvMassMatrixSegment));

  // Index
  size_t iStart = mDof->mIndexInTree;

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
      = getInvProjArtInertiaImplicit()
        * (mInvM_a - getLocalJacobianStatic().dot(
             _artInertia * math::AdInvT(getLocalTransform(), _spatialAcc)));

  // Verification
  assert(!math::isNan(mInvMassMatrixSegment));

  // Index
  size_t iStart = mDof->mIndexInTree;

  // Assign
  _invMassMat(iStart, _col) = mInvMassMatrixSegment;
}

//==============================================================================
void SingleDofJoint::addInvMassMatrixSegmentTo(Eigen::Vector6d& _acc)
{
  //
  _acc += getLocalJacobianStatic() * mInvMassMatrixSegment;
}

//==============================================================================
Eigen::VectorXd SingleDofJoint::getSpatialToGeneralized(
    const Eigen::Vector6d& _spatial)
{
  Eigen::VectorXd generalized = Eigen::VectorXd::Zero(1);
  generalized[0] = getLocalJacobianStatic().dot(_spatial);
  return generalized;
}

}  // namespace dynamics
}  // namespace dart
