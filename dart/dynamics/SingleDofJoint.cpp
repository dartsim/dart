/*
 * Copyright (c) 2014-2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2014-2016, Humanoid Lab, Georgia Tech Research Corporation
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

#include "dart/dynamics/SingleDofJoint.hpp"

#include "dart/common/Console.hpp"
#include "dart/math/Helpers.hpp"
#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/dynamics/DegreeOfFreedom.hpp"

#define SINGLEDOFJOINT_REPORT_DIM_MISMATCH( func, arg )                        \
  dterr << "[SingleDofJoint::" #func "] Size of " << #arg << "[" << arg .size()\
        << "] should be exactly 1 for Joint named [" << getName() << "].\n";   \
  assert(false);

#define SINGLEDOFJOINT_REPORT_OUT_OF_RANGE( func, index )         \
  dterr << "[SingleDofJoint::" #func "] The index [" << index     \
        << "] is out of range for Joint named [" << getName()     \
        << "] which only has a zeroth index.\n";                  \
  assert(false);

#define SINGLEDOFJOINT_REPORT_UNSUPPORTED_ACTUATOR( func )                  \
  dterr << "[SingleDofJoint::" # func "] Unsupported actuator type ("       \
        << Joint::mAspectProperties.mActuatorType << ") for Joint [" << getName() << "].\n"; \
  assert(false);

#define SINGLEDOFJOINT_SET_IF_DIFFERENT( mField, value )\
  if( value == mAspectProperties. mField )\
    return;\
  mAspectProperties. mField = value;\
  Joint::incrementVersion();

namespace dart {
namespace dynamics {

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
  setAspectProperties(_properties);
}

//==============================================================================
void SingleDofJoint::setAspectState(const AspectState& state)
{
  setPositionStatic(state.mPosition);
  setVelocityStatic(state.mVelocity);
  setAccelerationStatic(state.mAcceleration);
  setForce(0, state.mForce);

  // We call Command last so that it does not get overwritten due to actuator
  // type interference.
  setCommand(0, state.mCommand);
}

//==============================================================================
void SingleDofJoint::setAspectProperties(const AspectProperties& properties)
{
  setDofName(0, properties.mDofName, properties.mPreserveDofName);
  setPositionLowerLimit(0, properties.mPositionLowerLimit);
  setPositionUpperLimit(0, properties.mPositionUpperLimit);
  setInitialPosition(0, properties.mInitialPosition);
  setVelocityLowerLimit(0, properties.mVelocityLowerLimit);
  setVelocityUpperLimit(0, properties.mVelocityUpperLimit);
  setInitialVelocity(0, properties.mInitialVelocity);
  setAccelerationLowerLimit(0, properties.mAccelerationLowerLimit);
  setAccelerationUpperLimit(0, properties.mAccelerationUpperLimit);
  setForceLowerLimit(0, properties.mForceLowerLimit);
  setForceUpperLimit(0, properties.mForceUpperLimit);
  setSpringStiffness(0, properties.mSpringStiffness);
  setRestPosition(0, properties.mRestPosition);
  setDampingCoefficient(0, properties.mDampingCoefficient);
  setCoulombFriction(0, properties.mFriction);
}

//==============================================================================
SingleDofJoint::Properties SingleDofJoint::getSingleDofJointProperties() const
{
  return Properties(Joint::mAspectProperties, mAspectProperties);
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
std::size_t SingleDofJoint::getNumDofs() const
{
  return 1;
}

//==============================================================================
std::size_t SingleDofJoint::getIndexInSkeleton(std::size_t _index) const
{
  if (_index != 0)
  {
    SINGLEDOFJOINT_REPORT_OUT_OF_RANGE( getIndexInSkeleton, _index );
    return 0;
  }

  return mDof->mIndexInSkeleton;
}

//==============================================================================
std::size_t SingleDofJoint::getIndexInTree(std::size_t _index) const
{
  if (_index != 0)
  {
    SINGLEDOFJOINT_REPORT_OUT_OF_RANGE( getIndexInTree, _index );
    return 0;
  }

  return mDof->mIndexInTree;
}

//==============================================================================
DegreeOfFreedom* SingleDofJoint::getDof(std::size_t _index)
{
  if (0 == _index)
    return mDof;

  SINGLEDOFJOINT_REPORT_OUT_OF_RANGE( getDof, _index );
  return nullptr;
}

//==============================================================================
const DegreeOfFreedom* SingleDofJoint::getDof(std::size_t _index) const
{
  if (0 == _index)
    return mDof;

  SINGLEDOFJOINT_REPORT_OUT_OF_RANGE( getDof, _index );
  return nullptr;
}

//==============================================================================
const std::string& SingleDofJoint::setDofName(std::size_t _index,
                                              const std::string& _name,
                                              bool _preserveName)
{
  if (0 < _index)
  {
    dterr << "[SingleDofJoint::getDofName] Attempting to set the name of DOF "
          << "index " << _index << ", which is out of bounds for the Joint ["
          << getName() << "]. We will set the name of DOF index 0 instead\n";
    assert(false);
    _index = 0;
  }

  preserveDofName(0, _preserveName);

  if(_name == mAspectProperties.mDofName)
    return mAspectProperties.mDofName;

  const SkeletonPtr& skel = getSkeleton();
  if(skel)
  {
    mAspectProperties.mDofName =
        skel->mNameMgrForDofs.changeObjectName(mDof, _name);
    Joint::incrementVersion();
  }
  else
    mAspectProperties.mDofName = _name;

  return mAspectProperties.mDofName;
}

//==============================================================================
void SingleDofJoint::preserveDofName(std::size_t _index, bool _preserve)
{
  if (0 < _index)
  {
    SINGLEDOFJOINT_REPORT_OUT_OF_RANGE( preserveDofName, _index );
    return;
  }

  SINGLEDOFJOINT_SET_IF_DIFFERENT( mPreserveDofName, _preserve);
}

//==============================================================================
bool SingleDofJoint::isDofNamePreserved(std::size_t _index) const
{
  if (0 < _index)
  {
    SINGLEDOFJOINT_REPORT_OUT_OF_RANGE( isDofNamePreserved, _index );
  }

  return mAspectProperties.mPreserveDofName;
}

//==============================================================================
const std::string& SingleDofJoint::getDofName(std::size_t _index) const
{
  if (0 < _index)
  {
    dterr << "[SingleDofJoint::getDofName] Requested name of DOF index ["
          << _index << "] in Joint [" << getName() << "], which is out of "
          << "bounds. Returning the name of the only DOF available.\n";
    assert(false);
  }

  return mAspectProperties.mDofName;
}

//==============================================================================
void SingleDofJoint::setCommand(std::size_t _index, double _command)
{
  if (_index != 0)
  {
    SINGLEDOFJOINT_REPORT_OUT_OF_RANGE( setCommand, _index );
    return;
  }

  switch (Joint::mAspectProperties.mActuatorType)
  {
    case FORCE:
      mAspectState.mCommand = math::clip(_command,
                            mAspectProperties.mForceLowerLimit,
                            mAspectProperties.mForceUpperLimit);
      break;
    case PASSIVE:
      if(_command != 0.0)
      {
        dtwarn << "[SingleDofJoint::setCommand] Attempting to set a non-zero ("
               << _command << ") command for a PASSIVE joint [" << getName()
               << "].\n";
      }
      mAspectState.mCommand = _command;
      break;
    case SERVO:
      mAspectState.mCommand = math::clip(_command,
                            mAspectProperties.mVelocityLowerLimit,
                            mAspectProperties.mVelocityUpperLimit);
      break;
    case ACCELERATION:
      mAspectState.mCommand = math::clip(_command,
                            mAspectProperties.mAccelerationLowerLimit,
                            mAspectProperties.mAccelerationUpperLimit);
      break;
    case VELOCITY:
      mAspectState.mCommand = math::clip(_command,
                            mAspectProperties.mVelocityLowerLimit,
                            mAspectProperties.mVelocityUpperLimit);
      // TODO: This possibly makes the acceleration to exceed the limits.
      break;
    case LOCKED:
      if(_command != 0.0)
      {
        dtwarn << "[SingleDofJoint::setCommand] Attempting to set a non-zero ("
               << _command << ") command for a LOCKED joint [" << getName()
               << "].\n";
      }
      mAspectState.mCommand = _command;
      break;
    default:
      assert(false);
      break;
  }
}

//==============================================================================
double SingleDofJoint::getCommand(std::size_t _index) const
{
  if (_index != 0)
  {
    SINGLEDOFJOINT_REPORT_OUT_OF_RANGE( getCommand, _index );
    return 0.0;
  }

  return mAspectState.mCommand;
}

//==============================================================================
void SingleDofJoint::setCommands(const Eigen::VectorXd& _commands)
{
  if (static_cast<std::size_t>(_commands.size()) != getNumDofs())
  {
    SINGLEDOFJOINT_REPORT_DIM_MISMATCH( setCommands, _commands );
    return;
  }

  setCommand(0, _commands[0]);
}

//==============================================================================
Eigen::VectorXd SingleDofJoint::getCommands() const
{
  return Eigen::Matrix<double, 1, 1>::Constant(mAspectState.mCommand);
}

//==============================================================================
void SingleDofJoint::resetCommands()
{
  mAspectState.mCommand = 0.0;
}

//==============================================================================
void SingleDofJoint::setPosition(std::size_t _index, double _position)
{
  if (_index != 0)
  {
    SINGLEDOFJOINT_REPORT_OUT_OF_RANGE( setPosition, _index );
    return;
  }

  setPositionStatic(_position);
}

//==============================================================================
double SingleDofJoint::getPosition(std::size_t _index) const
{
  if (_index != 0)
  {
    SINGLEDOFJOINT_REPORT_OUT_OF_RANGE( getPosition, _index );
    return 0.0;
  }

  return getPositionStatic();
}

//==============================================================================
void SingleDofJoint::setPositions(const Eigen::VectorXd& _positions)
{
  if (static_cast<std::size_t>(_positions.size()) != getNumDofs())
  {
    SINGLEDOFJOINT_REPORT_DIM_MISMATCH( setPositions, _positions );
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
void SingleDofJoint::setPositionLowerLimit(std::size_t _index, double _position)
{
  if (_index != 0)
  {
    SINGLEDOFJOINT_REPORT_OUT_OF_RANGE( setPositionLowerLimit, _index );
    return;
  }

  SINGLEDOFJOINT_SET_IF_DIFFERENT(mPositionLowerLimit, _position)
}

//==============================================================================
double SingleDofJoint::getPositionLowerLimit(std::size_t _index) const
{
  if (_index != 0)
  {
    SINGLEDOFJOINT_REPORT_OUT_OF_RANGE( getPositionLowerLimit, _index );
    return 0.0;
  }

  return mAspectProperties.mPositionLowerLimit;
}

//==============================================================================
void SingleDofJoint::setPositionUpperLimit(std::size_t _index, double _position)
{
  if (_index != 0)
  {
    SINGLEDOFJOINT_REPORT_OUT_OF_RANGE( setPositionUpperLimit, _index );
    return;
  }

  SINGLEDOFJOINT_SET_IF_DIFFERENT( mPositionUpperLimit, _position )
}

//==============================================================================
double SingleDofJoint::getPositionUpperLimit(std::size_t _index) const
{
  if (_index != 0)
  {
    SINGLEDOFJOINT_REPORT_OUT_OF_RANGE( getPositionUpperLimit, _index );
    return 0.0;
  }

  return mAspectProperties.mPositionUpperLimit;
}

//==============================================================================
bool SingleDofJoint::hasPositionLimit(std::size_t _index) const
{
  if (_index != 0)
  {
    SINGLEDOFJOINT_REPORT_OUT_OF_RANGE( hasPositionLimit, _index );
    return true;
  }

  return std::isfinite(mAspectProperties.mPositionLowerLimit)
      || std::isfinite(mAspectProperties.mPositionUpperLimit);
}

//==============================================================================
void SingleDofJoint::resetPosition(std::size_t _index)
{
  if (_index != 0)
  {
    SINGLEDOFJOINT_REPORT_OUT_OF_RANGE( resetPosition, _index );
    return;
  }

  setPositionStatic(mAspectProperties.mInitialPosition);
}

//==============================================================================
void SingleDofJoint::resetPositions()
{
  setPositionStatic(mAspectProperties.mInitialPosition);
}

//==============================================================================
void SingleDofJoint::setInitialPosition(std::size_t _index, double _initial)
{
  if (_index != 0)
  {
    SINGLEDOFJOINT_REPORT_OUT_OF_RANGE( setInitialPosition, _index );
    return;
  }

  SINGLEDOFJOINT_SET_IF_DIFFERENT( mInitialPosition, _initial);
}

//==============================================================================
double SingleDofJoint::getInitialPosition(std::size_t _index) const
{
  if (_index != 0)
  {
    SINGLEDOFJOINT_REPORT_OUT_OF_RANGE( getInitialPosition, _index );
    return 0.0;
  }

  return mAspectProperties.mInitialPosition;
}

//==============================================================================
void SingleDofJoint::setInitialPositions(const Eigen::VectorXd& _initial)
{
  if( static_cast<std::size_t>(_initial.size()) != getNumDofs() )
  {
    SINGLEDOFJOINT_REPORT_DIM_MISMATCH( setInitialPositions, _initial );
    return;
  }

  setInitialPosition(0, _initial[0]);
}

//==============================================================================
Eigen::VectorXd SingleDofJoint::getInitialPositions() const
{
  return Eigen::Matrix<double, 1, 1>::Constant(
        mAspectProperties.mInitialPosition);
}

//==============================================================================
void SingleDofJoint::setVelocity(std::size_t _index, double _velocity)
{
  if (_index != 0)
  {
    SINGLEDOFJOINT_REPORT_OUT_OF_RANGE( setVelocity, _index );
    return;
  }

  setVelocityStatic(_velocity);

  if (Joint::mAspectProperties.mActuatorType == VELOCITY)
    mAspectState.mCommand = getVelocityStatic();
}

//==============================================================================
double SingleDofJoint::getVelocity(std::size_t _index) const
{
  if (_index != 0)
  {
    SINGLEDOFJOINT_REPORT_OUT_OF_RANGE( getVelocity, _index );
    return 0.0;
  }

  return getVelocityStatic();
}

//==============================================================================
void SingleDofJoint::setVelocities(const Eigen::VectorXd& _velocities)
{
  if (static_cast<std::size_t>(_velocities.size()) != getNumDofs())
  {
    SINGLEDOFJOINT_REPORT_DIM_MISMATCH( setVelocities, _velocities );
    return;
  }

  setVelocityStatic(_velocities[0]);

  if (Joint::mAspectProperties.mActuatorType == VELOCITY)
    mAspectState.mCommand = getVelocityStatic();
}

//==============================================================================
Eigen::VectorXd SingleDofJoint::getVelocities() const
{
  return Eigen::Matrix<double, 1, 1>::Constant(getVelocityStatic());
}

//==============================================================================
void SingleDofJoint::setVelocityLowerLimit(std::size_t _index, double _velocity)
{
  if (_index != 0)
  {
    SINGLEDOFJOINT_REPORT_OUT_OF_RANGE( setVelocityLowerLimit, _index );
    return;
  }

  SINGLEDOFJOINT_SET_IF_DIFFERENT( mVelocityLowerLimit, _velocity);
}

//==============================================================================
double SingleDofJoint::getVelocityLowerLimit(std::size_t _index) const
{
  if (_index != 0)
  {
    SINGLEDOFJOINT_REPORT_OUT_OF_RANGE( getVelocityLowerLimit, _index );
    return 0.0;
  }

  return mAspectProperties.mVelocityLowerLimit;
}

//==============================================================================
void SingleDofJoint::setVelocityUpperLimit(std::size_t _index, double _velocity)
{
  if (_index != 0)
  {
    SINGLEDOFJOINT_REPORT_OUT_OF_RANGE( setVelocityUpperLimit, _index );
    return;
  }

  SINGLEDOFJOINT_SET_IF_DIFFERENT( mVelocityUpperLimit, _velocity);
}

//==============================================================================
double SingleDofJoint::getVelocityUpperLimit(std::size_t _index) const
{
  if (_index != 0)
  {
    SINGLEDOFJOINT_REPORT_OUT_OF_RANGE( getVelocityUpperLimit, _index );
    return 0.0;
  }

  return mAspectProperties.mVelocityUpperLimit;
}

//==============================================================================
void SingleDofJoint::resetVelocity(std::size_t _index)
{
  if (_index != 0)
  {
    SINGLEDOFJOINT_REPORT_OUT_OF_RANGE( resetVelocity, _index );
    return;
  }

  setVelocityStatic(mAspectProperties.mInitialVelocity);
}

//==============================================================================
void SingleDofJoint::resetVelocities()
{
  setVelocityStatic(mAspectProperties.mInitialVelocity);
}

//==============================================================================
void SingleDofJoint::setInitialVelocity(std::size_t _index, double _initial)
{
  if (_index != 0)
  {
    SINGLEDOFJOINT_REPORT_OUT_OF_RANGE( setInitialVelocity, _index );
    return;
  }

  SINGLEDOFJOINT_SET_IF_DIFFERENT( mInitialVelocity, _initial );
}

//==============================================================================
double SingleDofJoint::getInitialVelocity(std::size_t _index) const
{
  if (_index != 0)
  {
    SINGLEDOFJOINT_REPORT_OUT_OF_RANGE( getInitialVelocity, _index );
    return 0.0;
  }

  return mAspectProperties.mInitialVelocity;
}

//==============================================================================
void SingleDofJoint::setInitialVelocities(const Eigen::VectorXd& _initial)
{
  if( static_cast<std::size_t>(_initial.size()) != getNumDofs() )
  {
    SINGLEDOFJOINT_REPORT_DIM_MISMATCH( setInitialVelocities, _initial );
    return;
  }

  setInitialVelocity(0, _initial[0]);
}

//==============================================================================
Eigen::VectorXd SingleDofJoint::getInitialVelocities() const
{
  return Eigen::Matrix<double, 1, 1>::Constant(
        mAspectProperties.mInitialVelocity);
}

//==============================================================================
void SingleDofJoint::setAcceleration(std::size_t _index, double _acceleration)
{
  if (_index != 0)
  {
    SINGLEDOFJOINT_REPORT_OUT_OF_RANGE( setAcceleration, _index );
    return;
  }

  setAccelerationStatic(_acceleration);

  if (Joint::mAspectProperties.mActuatorType == ACCELERATION)
    mAspectState.mCommand = getAccelerationStatic();
}

//==============================================================================
double SingleDofJoint::getAcceleration(std::size_t _index) const
{
  if (_index != 0)
  {
    SINGLEDOFJOINT_REPORT_OUT_OF_RANGE( getAcceleration, _index );
    return 0.0;
  }

  return getAccelerationStatic();
}

//==============================================================================
void SingleDofJoint::setAccelerations(const Eigen::VectorXd& _accelerations)
{
  if (static_cast<std::size_t>(_accelerations.size()) != getNumDofs())
  {
    SINGLEDOFJOINT_REPORT_DIM_MISMATCH( setAccelerations, _accelerations );
    return;
  }

  setAccelerationStatic(_accelerations[0]);

  if (Joint::mAspectProperties.mActuatorType == ACCELERATION)
    mAspectState.mCommand = getAccelerationStatic();
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
void SingleDofJoint::setAccelerationLowerLimit(std::size_t _index,
                                               double _acceleration)
{
  if (_index != 0)
  {
    SINGLEDOFJOINT_REPORT_OUT_OF_RANGE( setAccelerationLowerLimit, _index );
    return;
  }

  SINGLEDOFJOINT_SET_IF_DIFFERENT( mAccelerationLowerLimit, _acceleration);
}

//==============================================================================
double SingleDofJoint::getAccelerationLowerLimit(std::size_t _index) const
{
  if (_index != 0)
  {
    SINGLEDOFJOINT_REPORT_OUT_OF_RANGE( getAccelerationLowerLimit, _index );
    return 0.0;
  }

  return mAspectProperties.mAccelerationLowerLimit;
}

//==============================================================================
void SingleDofJoint::setAccelerationUpperLimit(std::size_t _index,
                                               double _acceleration)
{
  if (_index != 0)
  {
    SINGLEDOFJOINT_REPORT_OUT_OF_RANGE( getAccelerationUpperLimit, _index );
    return;
  }

  SINGLEDOFJOINT_SET_IF_DIFFERENT( mAccelerationUpperLimit, _acceleration);
}

//==============================================================================
double SingleDofJoint::getAccelerationUpperLimit(std::size_t _index) const
{
  if (_index != 0)
  {
    SINGLEDOFJOINT_REPORT_OUT_OF_RANGE( getAccelerationUpperLimit, _index );
    return 0.0;
  }

  return mAspectProperties.mAccelerationUpperLimit;
}

//==============================================================================
void SingleDofJoint::setPositionStatic(const double& _position)
{
  if(mAspectState.mPosition == _position)
    return;

  mAspectState.mPosition = _position;
  notifyPositionUpdate();
}

//==============================================================================
const double& SingleDofJoint::getPositionStatic() const
{
  return mAspectState.mPosition;
}

//==============================================================================
void SingleDofJoint::setVelocityStatic(const double& _velocity)
{
  if(mAspectState.mVelocity == _velocity)
    return;

  mAspectState.mVelocity = _velocity;
  notifyVelocityUpdate();
}

//==============================================================================
const double& SingleDofJoint::getVelocityStatic() const
{
  return mAspectState.mVelocity;
}

//==============================================================================
void SingleDofJoint::setAccelerationStatic(const double& _acceleration)
{
  if(mAspectState.mAcceleration == _acceleration)
    return;

  mAspectState.mAcceleration = _acceleration;
  notifyAccelerationUpdate();
}

//==============================================================================
const double& SingleDofJoint::getAccelerationStatic() const
{
  return mAspectState.mAcceleration;
}

//==============================================================================
void SingleDofJoint::setForce(std::size_t _index, double _force)
{
  if (_index != 0)
  {
    SINGLEDOFJOINT_REPORT_OUT_OF_RANGE( setForce, _index );
    return;
  }

  mAspectState.mForce = _force;

  if (Joint::mAspectProperties.mActuatorType == FORCE)
    mAspectState.mCommand = mAspectState.mForce;
}

//==============================================================================
double SingleDofJoint::getForce(std::size_t _index)
{
  if (_index != 0)
  {
    SINGLEDOFJOINT_REPORT_OUT_OF_RANGE( getForce, _index );
    return 0.0;
  }

  return mAspectState.mForce;
}

//==============================================================================
void SingleDofJoint::setForces(const Eigen::VectorXd& _forces)
{
  if (static_cast<std::size_t>(_forces.size()) != getNumDofs())
  {
    SINGLEDOFJOINT_REPORT_DIM_MISMATCH( setForces, _forces );
    return;
  }

  mAspectState.mForce = _forces[0];

  if (Joint::mAspectProperties.mActuatorType == FORCE)
    mAspectState.mCommand = mAspectState.mForce;
}

//==============================================================================
Eigen::VectorXd SingleDofJoint::getForces() const
{
  return Eigen::Matrix<double, 1, 1>::Constant(mAspectState.mForce);
}

//==============================================================================
void SingleDofJoint::resetForces()
{
  mAspectState.mForce = 0.0;

  if (Joint::mAspectProperties.mActuatorType == FORCE)
    mAspectState.mCommand = mAspectState.mForce;
}

//==============================================================================
void SingleDofJoint::setForceLowerLimit(std::size_t _index, double _force)
{
  if (_index != 0)
  {
    SINGLEDOFJOINT_REPORT_OUT_OF_RANGE( setForceLowerLimit, _index );
    return;
  }

  SINGLEDOFJOINT_SET_IF_DIFFERENT( mForceLowerLimit, _force );
}

//==============================================================================
double SingleDofJoint::getForceLowerLimit(std::size_t _index) const
{
  if (_index != 0)
  {
    SINGLEDOFJOINT_REPORT_OUT_OF_RANGE( getForceLowerLimit, _index );
    return 0.0;
  }

  return mAspectProperties.mForceLowerLimit;
}

//==============================================================================
void SingleDofJoint::setForceUpperLimit(std::size_t _index, double _force)
{
  if (_index != 0)
  {
    SINGLEDOFJOINT_REPORT_OUT_OF_RANGE( setForceUpperLimit, _index );
    return;
  }

  SINGLEDOFJOINT_SET_IF_DIFFERENT( mForceUpperLimit, _force);
}

//==============================================================================
double SingleDofJoint::getForceUpperLimit(std::size_t _index) const
{
  if (_index != 0)
  {
    SINGLEDOFJOINT_REPORT_OUT_OF_RANGE( getForceUpperLimit, _index );
    return 0.0;
  }

  return mAspectProperties.mForceUpperLimit;
}

//==============================================================================
void SingleDofJoint::setVelocityChange(std::size_t _index, double _velocityChange)
{
  if (_index != 0)
  {
    SINGLEDOFJOINT_REPORT_OUT_OF_RANGE( setVelocityChange, _index );
    return;
  }

  mVelocityChange = _velocityChange;
}

//==============================================================================
double SingleDofJoint::getVelocityChange(std::size_t _index) const
{
  if (_index != 0)
  {
    SINGLEDOFJOINT_REPORT_OUT_OF_RANGE( getVelocityChange, _index );
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
void SingleDofJoint::setConstraintImpulse(std::size_t _index, double _impulse)
{
  if (_index != 0)
  {
    SINGLEDOFJOINT_REPORT_OUT_OF_RANGE( setConstraintImpulse, _index );
    return;
  }

  mConstraintImpulse = _impulse;
}

//==============================================================================
double SingleDofJoint::getConstraintImpulse(std::size_t _index) const
{
  if (_index != 0)
  {
    SINGLEDOFJOINT_REPORT_OUT_OF_RANGE( getConstraintImpulse, _index );
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
Eigen::VectorXd SingleDofJoint::getPositionDifferences(
    const Eigen::VectorXd& _q2, const Eigen::VectorXd& _q1) const
{
  if (static_cast<std::size_t>(_q1.size()) != getNumDofs()
      || static_cast<std::size_t>(_q2.size()) != getNumDofs())
  {
    dterr << "[SingleDofJoint::getPositionsDifference] q1's size ["
          << _q1.size() << "] and q2's size [" << _q2.size() << "] must both "
          << "equal 1 for Joint [" << getName() << "].\n";
    assert(false);
    return Eigen::Matrix<double, 1, 1>::Zero();
  }

  return Eigen::Matrix<double, 1, 1>::Constant(
        getPositionDifferenceStatic(_q2[0], _q1[0]));
}

//==============================================================================
double SingleDofJoint::getPositionDifferenceStatic(double _q2,
                                                    double _q1) const
{
  return _q2 - _q1;
}

//==============================================================================
void SingleDofJoint::setSpringStiffness(std::size_t _index, double _k)
{
  if (_index != 0)
  {
    SINGLEDOFJOINT_REPORT_OUT_OF_RANGE( setSpringStiffness, _index );
    return;
  }

  assert(_k >= 0.0);

  SINGLEDOFJOINT_SET_IF_DIFFERENT( mSpringStiffness, _k);
}

//==============================================================================
double SingleDofJoint::getSpringStiffness(std::size_t _index) const
{
  if (_index != 0)
  {
    SINGLEDOFJOINT_REPORT_OUT_OF_RANGE( getSpringStiffness, _index );
    return 0.0;
  }

  return mAspectProperties.mSpringStiffness;
}

//==============================================================================
void SingleDofJoint::setRestPosition(std::size_t _index, double _q0)
{
  if (_index != 0)
  {
    SINGLEDOFJOINT_REPORT_OUT_OF_RANGE( setRestPosition, _index );
    return;
  }

  if (mAspectProperties.mPositionLowerLimit > _q0
      || mAspectProperties.mPositionUpperLimit < _q0)
  {
    dtwarn << "[SingleDofJoint::setRestPosition] Value of _q0 [" << _q0
           << "] is out of the limit range ["
           << mAspectProperties.mPositionLowerLimit << ", "
           << mAspectProperties.mPositionUpperLimit
           << "] for index [" << _index << "] of Joint ["
           << getName() << "].\n";
    return;
  }

  SINGLEDOFJOINT_SET_IF_DIFFERENT( mRestPosition, _q0);
}

//==============================================================================
double SingleDofJoint::getRestPosition(std::size_t _index) const
{
  if (_index != 0)
  {
    SINGLEDOFJOINT_REPORT_OUT_OF_RANGE( getRestPosition, _index );
    return 0.0;
  }

  return mAspectProperties.mRestPosition;
}

//==============================================================================
void SingleDofJoint::setDampingCoefficient(std::size_t _index, double _d)
{
  if (_index != 0)
  {
    SINGLEDOFJOINT_REPORT_OUT_OF_RANGE( setDampingCoefficient, _index );
    return;
  }

  assert(_d >= 0.0);

  SINGLEDOFJOINT_SET_IF_DIFFERENT( mDampingCoefficient, _d);
}

//==============================================================================
double SingleDofJoint::getDampingCoefficient(std::size_t _index) const
{
  if (_index != 0)
  {
    SINGLEDOFJOINT_REPORT_OUT_OF_RANGE( getDampingCoefficient, _index );
    return 0.0;
  }

  return mAspectProperties.mDampingCoefficient;
}

//==============================================================================
void SingleDofJoint::setCoulombFriction(std::size_t _index, double _friction)
{
  if (_index != 0)
  {
    SINGLEDOFJOINT_REPORT_OUT_OF_RANGE( setCoulombFriction, _index );
    return;
  }

  assert(_friction >= 0.0);

  SINGLEDOFJOINT_SET_IF_DIFFERENT( mFriction, _friction);
}

//==============================================================================
double SingleDofJoint::getCoulombFriction(std::size_t _index) const
{
  if (_index != 0)
  {
    SINGLEDOFJOINT_REPORT_OUT_OF_RANGE( getCoulombFriction, _index );
    return 0.0;
  }

  return mAspectProperties.mFriction;
}

//==============================================================================
double SingleDofJoint::computePotentialEnergy() const
{
  // Spring energy
  double pe = 0.5 * mAspectProperties.mSpringStiffness
       * (getPositionStatic() - mAspectProperties.mRestPosition)
       * (getPositionStatic() - mAspectProperties.mRestPosition);

  return pe;
}

//==============================================================================
SingleDofJoint::SingleDofJoint(const Properties& properties)
  : mDof(createDofPointer(0)),
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
  mAspectState.mPosition = properties.mInitialPosition;
  mAspectState.mVelocity = properties.mInitialVelocity;
}

//==============================================================================
void SingleDofJoint::registerDofs()
{
  SkeletonPtr skel = getSkeleton();
  if(skel)
    mAspectProperties.mDofName =
        skel->mNameMgrForDofs.issueNewNameAndAdd(mDof->getName(), mDof);
}

//==============================================================================
void SingleDofJoint::updateDegreeOfFreedomNames()
{
  // Same name as the joint it belongs to.
  if (!mDof->isNamePreserved())
    mDof->setName(Joint::mAspectProperties.mName, false);
}

//==============================================================================
void SingleDofJoint::updateRelativeSpatialVelocity() const
{
  mSpatialVelocity = getRelativeJacobianStatic() * getVelocityStatic();
}

//==============================================================================
void SingleDofJoint::updateRelativeSpatialAcceleration() const
{
  mSpatialAcceleration = getRelativePrimaryAcceleration()
                       + getRelativeJacobianTimeDerivStatic() * getVelocityStatic();
}

//==============================================================================
void SingleDofJoint::updateRelativePrimaryAcceleration() const
{
  mPrimaryAcceleration = getRelativeJacobianStatic() * getAccelerationStatic();
}

//==============================================================================
Eigen::Vector6d SingleDofJoint::getBodyConstraintWrench() const
{
  assert(mChildBodyNode);
  return mChildBodyNode->getBodyForce()
      - getRelativeJacobianStatic() * mAspectState.mForce;
}

//==============================================================================
const math::Jacobian SingleDofJoint::getRelativeJacobian() const
{
  return getRelativeJacobianStatic();
}

//==============================================================================
const Eigen::Vector6d& SingleDofJoint::getRelativeJacobianStatic() const
{
  if(mIsRelativeJacobianDirty)
  {
    updateRelativeJacobian(false);
    mIsRelativeJacobianDirty = false;
  }
  return mJacobian;
}

//==============================================================================
math::Jacobian SingleDofJoint::getRelativeJacobian(
    const Eigen::VectorXd& /*_positions*/) const
{
  // The Jacobian is always constant w.r.t. the generalized coordinates.
  return getRelativeJacobianStatic();
}

//==============================================================================
const math::Jacobian SingleDofJoint::getRelativeJacobianTimeDeriv() const
{
  if(mIsRelativeJacobianTimeDerivDirty)
  {
    updateRelativeJacobianTimeDeriv();
    mIsRelativeJacobianTimeDerivDirty = false;
  }
  return mJacobianDeriv;
}

//==============================================================================
const Eigen::Vector6d& SingleDofJoint::getRelativeJacobianTimeDerivStatic() const
{
  if(mIsRelativeJacobianTimeDerivDirty)
  {
    updateRelativeJacobianTimeDeriv();
    mIsRelativeJacobianTimeDerivDirty = false;
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
  _vel.noalias() += getRelativeJacobianStatic() * getVelocityStatic();

  // Verification
  assert(!math::isNan(_vel));
}

//==============================================================================
void SingleDofJoint::setPartialAccelerationTo(
    Eigen::Vector6d& _partialAcceleration,
    const Eigen::Vector6d& _childVelocity)
{
  // ad(V, S * dq) + dS * dq
  _partialAcceleration = math::ad(_childVelocity, getRelativeJacobianStatic()
                                  * getVelocityStatic())
                         + mJacobianDeriv * getVelocityStatic();
  // Verification
  assert(!math::isNan(_partialAcceleration));
}


//==============================================================================
void SingleDofJoint::addAccelerationTo(Eigen::Vector6d& _acc)
{
  //
  _acc += getRelativeJacobianStatic() * getAccelerationStatic();
}

//==============================================================================
void SingleDofJoint::addVelocityChangeTo(Eigen::Vector6d& _velocityChange)
{
  //
  _velocityChange += getRelativeJacobianStatic() * mVelocityChange;
}

//==============================================================================
void SingleDofJoint::addChildArtInertiaTo(
    Eigen::Matrix6d& _parentArtInertia, const Eigen::Matrix6d& _childArtInertia)
{
  switch (Joint::mAspectProperties.mActuatorType)
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
      SINGLEDOFJOINT_REPORT_UNSUPPORTED_ACTUATOR( addChildArtInertiaTo );
      break;
  }
}

//==============================================================================
void SingleDofJoint::addChildArtInertiaToDynamic(
    Eigen::Matrix6d& _parentArtInertia, const Eigen::Matrix6d& _childArtInertia)
{
  // Child body's articulated inertia
  Eigen::Vector6d AIS = _childArtInertia * getRelativeJacobianStatic();
  Eigen::Matrix6d PI = _childArtInertia;
  PI.noalias() -= mInvProjArtInertia * AIS * AIS.transpose();
  assert(!math::isNan(PI));

  // Add child body's articulated inertia to parent body's articulated inertia.
  // Note that mT should be updated.
  _parentArtInertia += math::transformInertia(getRelativeTransform().inverse(), PI);
}

//==============================================================================
void SingleDofJoint::addChildArtInertiaToKinematic(
    Eigen::Matrix6d& _parentArtInertia, const Eigen::Matrix6d& _childArtInertia)
{
  // Add child body's articulated inertia to parent body's articulated inertia.
  // Note that mT should be updated.
  _parentArtInertia += math::transformInertia(getRelativeTransform().inverse(),
                                              _childArtInertia);
}

//==============================================================================
void SingleDofJoint::addChildArtInertiaImplicitTo(
    Eigen::Matrix6d& _parentArtInertia, const Eigen::Matrix6d& _childArtInertia)
{
  switch (Joint::mAspectProperties.mActuatorType)
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
      SINGLEDOFJOINT_REPORT_UNSUPPORTED_ACTUATOR(addChildArtInertiaImplicitTo);
      break;
  }
}

//==============================================================================
void SingleDofJoint::addChildArtInertiaImplicitToDynamic(
    Eigen::Matrix6d& _parentArtInertia, const Eigen::Matrix6d& _childArtInertia)
{
  // Child body's articulated inertia
  const Eigen::Vector6d AIS = _childArtInertia * getRelativeJacobianStatic();
  Eigen::Matrix6d PI = _childArtInertia;
  PI.noalias() -= mInvProjArtInertiaImplicit * AIS * AIS.transpose();
  assert(!math::isNan(PI));

  // Add child body's articulated inertia to parent body's articulated inertia.
  // Note that mT should be updated.
  _parentArtInertia += math::transformInertia(getRelativeTransform().inverse(), PI);
}

//==============================================================================
void SingleDofJoint::addChildArtInertiaImplicitToKinematic(
    Eigen::Matrix6d& _parentArtInertia, const Eigen::Matrix6d& _childArtInertia)
{
  // Add child body's articulated inertia to parent body's articulated inertia.
  // Note that mT should be updated.
  _parentArtInertia += math::transformInertia(getRelativeTransform().inverse(),
                                              _childArtInertia);
}

//==============================================================================
void SingleDofJoint::updateInvProjArtInertia(
    const Eigen::Matrix6d& _artInertia)
{
  switch (Joint::mAspectProperties.mActuatorType)
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
      SINGLEDOFJOINT_REPORT_UNSUPPORTED_ACTUATOR(updateInvProjArtInertia);
      break;
  }
}

//==============================================================================
void SingleDofJoint::updateInvProjArtInertiaDynamic(
    const Eigen::Matrix6d& _artInertia)
{
  // Projected articulated inertia
  const Eigen::Vector6d& Jacobian = getRelativeJacobianStatic();
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
  switch (Joint::mAspectProperties.mActuatorType)
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
      SINGLEDOFJOINT_REPORT_UNSUPPORTED_ACTUATOR(
            updateInvProjArtInertiaImplicit);
      break;
  }
}

//==============================================================================
void SingleDofJoint::updateInvProjArtInertiaImplicitDynamic(
    const Eigen::Matrix6d& _artInertia, double _timeStep)
{
  // Projected articulated inertia
  const Eigen::Vector6d& Jacobian = getRelativeJacobianStatic();
  double projAI = Jacobian.dot(_artInertia * Jacobian);

  // Add additional inertia for implicit damping and spring force
  projAI += _timeStep * mAspectProperties.mDampingCoefficient
      + _timeStep * _timeStep * mAspectProperties.mSpringStiffness;

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
  switch (Joint::mAspectProperties.mActuatorType)
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
      SINGLEDOFJOINT_REPORT_UNSUPPORTED_ACTUATOR(addChildBiasForceTo);
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
                                      + coeff*getRelativeJacobianStatic());

  // Verification
  assert(!math::isNan(beta));

  // Add child body's bias force to parent body's bias force. Note that mT
  // should be updated.
  _parentBiasForce += math::dAdInvT(getRelativeTransform(), beta);
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
                            + getAccelerationStatic()*getRelativeJacobianStatic());

  // Verification
  assert(!math::isNan(beta));

  // Add child body's bias force to parent body's bias force. Note that mT
  // should be updated.
  _parentBiasForce += math::dAdInvT(getRelativeTransform(), beta);
}

//==============================================================================
void SingleDofJoint::addChildBiasImpulseTo(
    Eigen::Vector6d& _parentBiasImpulse,
    const Eigen::Matrix6d& _childArtInertia,
    const Eigen::Vector6d& _childBiasImpulse)
{
  switch (Joint::mAspectProperties.mActuatorType)
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
      SINGLEDOFJOINT_REPORT_UNSUPPORTED_ACTUATOR(addChildBiasImpulseTo);
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
        + _childArtInertia * getRelativeJacobianStatic()
          * getInvProjArtInertia() * mTotalImpulse;

  // Verification
  assert(!math::isNan(beta));

  // Add child body's bias force to parent body's bias force. Note that mT
  // should be updated.
  _parentBiasImpulse += math::dAdInvT(getRelativeTransform(), beta);
}

//==============================================================================
void SingleDofJoint::addChildBiasImpulseToKinematic(
    Eigen::Vector6d& _parentBiasImpulse,
    const Eigen::Matrix6d& /*_childArtInertia*/,
    const Eigen::Vector6d& _childBiasImpulse)
{
  // Add child body's bias force to parent body's bias force. Note that mT
  // should be updated.
  _parentBiasImpulse += math::dAdInvT(getRelativeTransform(), _childBiasImpulse);
}

//==============================================================================
void SingleDofJoint::updateTotalForce(const Eigen::Vector6d& _bodyForce,
                                      double _timeStep)
{
  assert(_timeStep > 0.0);

  switch (Joint::mAspectProperties.mActuatorType)
  {
    case FORCE:
      mAspectState.mForce = mAspectState.mCommand;
      updateTotalForceDynamic(_bodyForce, _timeStep);
      break;
    case PASSIVE:
    case SERVO:
      mAspectState.mForce = 0.0;
      updateTotalForceDynamic(_bodyForce, _timeStep);
      break;
    case ACCELERATION:
      setAccelerationStatic(mAspectState.mCommand);
      updateTotalForceKinematic(_bodyForce, _timeStep);
      break;
    case VELOCITY:
      setAccelerationStatic( (mAspectState.mCommand - getVelocityStatic()) / _timeStep );
      updateTotalForceKinematic(_bodyForce, _timeStep);
      break;
    case LOCKED:
      setVelocityStatic(0.0);
      setAccelerationStatic(0.0);
      updateTotalForceKinematic(_bodyForce, _timeStep);
      break;
    default:
      SINGLEDOFJOINT_REPORT_UNSUPPORTED_ACTUATOR(updateTotalForce);
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
     -mAspectProperties.mSpringStiffness
      * (nextPosition - mAspectProperties.mRestPosition);

  // Damping force
  const double dampingForce =
      -mAspectProperties.mDampingCoefficient * getVelocityStatic();

  // Compute alpha
  mTotalForce = mAspectState.mForce + springForce + dampingForce
                - getRelativeJacobianStatic().dot(_bodyForce);
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
  switch (Joint::mAspectProperties.mActuatorType)
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
      SINGLEDOFJOINT_REPORT_UNSUPPORTED_ACTUATOR(updateTotalImpulse);
      break;
  }
}

//==============================================================================
void SingleDofJoint::updateTotalImpulseDynamic(
    const Eigen::Vector6d& _bodyImpulse)
{
  mTotalImpulse = mConstraintImpulse - getRelativeJacobianStatic().dot(_bodyImpulse);
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
  switch (Joint::mAspectProperties.mActuatorType)
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
      SINGLEDOFJOINT_REPORT_UNSUPPORTED_ACTUATOR(updateAcceleration);
      break;
  }
}

//==============================================================================
void SingleDofJoint::updateAccelerationDynamic(
    const Eigen::Matrix6d& _artInertia, const Eigen::Vector6d& _spatialAcc)
{
  //
  const double val = getRelativeJacobianStatic().dot(
        _artInertia * math::AdInvT(getRelativeTransform(), _spatialAcc));
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
  switch (Joint::mAspectProperties.mActuatorType)
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
      SINGLEDOFJOINT_REPORT_UNSUPPORTED_ACTUATOR(updateVelocityChange);
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
           - getRelativeJacobianStatic().dot(
             _artInertia * math::AdInvT(getRelativeTransform(), _velocityChange)));

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
  mAspectState.mForce = getRelativeJacobianStatic().dot(_bodyForce);

  // Damping force
  if (_withDampingForces)
  {
    const double dampingForce =
        -mAspectProperties.mDampingCoefficient * getVelocityStatic();
    mAspectState.mForce -= dampingForce;
  }

  // Spring force
  if (_withSpringForces)
  {
    const double nextPosition = getPositionStatic()
                              + _timeStep*getVelocityStatic();
    const double springForce =
       -mAspectProperties.mSpringStiffness
        *(nextPosition - mAspectProperties.mRestPosition);
    mAspectState.mForce -= springForce;
  }
}

//==============================================================================
void SingleDofJoint::updateForceFD(const Eigen::Vector6d& _bodyForce,
                                   double _timeStep,
                                   bool _withDampingForces,
                                   bool _withSpringForces)
{
  switch (Joint::mAspectProperties.mActuatorType)
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
      SINGLEDOFJOINT_REPORT_UNSUPPORTED_ACTUATOR(updateForceFD);
      break;
  }
}

//==============================================================================
void SingleDofJoint::updateImpulseID(const Eigen::Vector6d& _bodyImpulse)
{
  mImpulse = getRelativeJacobianStatic().dot(_bodyImpulse);
}

//==============================================================================
void SingleDofJoint::updateImpulseFD(const Eigen::Vector6d& _bodyImpulse)
{
  switch (Joint::mAspectProperties.mActuatorType)
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
      SINGLEDOFJOINT_REPORT_UNSUPPORTED_ACTUATOR(updateImpulseFD);
      break;
  }
}

//==============================================================================
void SingleDofJoint::updateConstrainedTerms(double _timeStep)
{
  switch (Joint::mAspectProperties.mActuatorType)
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
      SINGLEDOFJOINT_REPORT_UNSUPPORTED_ACTUATOR(updateConstrainedTerms);
      break;
  }
}

//==============================================================================
void SingleDofJoint::updateConstrainedTermsDynamic(double _timeStep)
{
  const double invTimeStep = 1.0 / _timeStep;

  setVelocityStatic(getVelocityStatic() + mVelocityChange);
  setAccelerationStatic(getAccelerationStatic() + mVelocityChange*invTimeStep);
  mAspectState.mForce += mConstraintImpulse*invTimeStep;
}

//==============================================================================
void SingleDofJoint::updateConstrainedTermsKinematic(double _timeStep)
{
  mAspectState.mForce += mImpulse / _timeStep;
}

//==============================================================================
void SingleDofJoint::addChildBiasForceForInvMassMatrix(
    Eigen::Vector6d& _parentBiasForce,
    const Eigen::Matrix6d& _childArtInertia,
    const Eigen::Vector6d& _childBiasForce)
{
  // Compute beta
  Eigen::Vector6d beta = _childBiasForce;
  beta.noalias() += _childArtInertia * getRelativeJacobianStatic()
                    * getInvProjArtInertia() * mInvM_a;

  // Verification
  assert(!math::isNan(beta));

  // Add child body's bias force to parent body's bias force. Note that mT
  // should be updated.
  _parentBiasForce += math::dAdInvT(getRelativeTransform(), beta);
}

//==============================================================================
void SingleDofJoint::addChildBiasForceForInvAugMassMatrix(
    Eigen::Vector6d& _parentBiasForce,
    const Eigen::Matrix6d& _childArtInertia,
    const Eigen::Vector6d& _childBiasForce)
{
  // Compute beta
  Eigen::Vector6d beta = _childBiasForce;
  beta.noalias() += _childArtInertia * getRelativeJacobianStatic()
                    * getInvProjArtInertiaImplicit() * mInvM_a;

  // Verification
  assert(!math::isNan(beta));

  // Add child body's bias force to parent body's bias force. Note that mT
  // should be updated.
  _parentBiasForce += math::dAdInvT(getRelativeTransform(), beta);
}

//==============================================================================
void SingleDofJoint::updateTotalForceForInvMassMatrix(
    const Eigen::Vector6d& _bodyForce)
{
  // Compute alpha
  mInvM_a = mAspectState.mForce - getRelativeJacobianStatic().dot(_bodyForce);
}

//==============================================================================
void SingleDofJoint::getInvMassMatrixSegment(Eigen::MatrixXd& _invMassMat,
                                             const std::size_t _col,
                                             const Eigen::Matrix6d& _artInertia,
                                             const Eigen::Vector6d& _spatialAcc)
{
  //
  mInvMassMatrixSegment
      = getInvProjArtInertia()
        * (mInvM_a - getRelativeJacobianStatic().dot(
             _artInertia * math::AdInvT(getRelativeTransform(), _spatialAcc)));

  // Verification
  assert(!math::isNan(mInvMassMatrixSegment));

  // Index
  std::size_t iStart = mDof->mIndexInTree;

  // Assign
  _invMassMat(iStart, _col) = mInvMassMatrixSegment;
}

//==============================================================================
void SingleDofJoint::getInvAugMassMatrixSegment(
    Eigen::MatrixXd& _invMassMat,
    const std::size_t _col,
    const Eigen::Matrix6d& _artInertia,
    const Eigen::Vector6d& _spatialAcc)
{
  //
  mInvMassMatrixSegment
      = getInvProjArtInertiaImplicit()
        * (mInvM_a - getRelativeJacobianStatic().dot(
             _artInertia * math::AdInvT(getRelativeTransform(), _spatialAcc)));

  // Verification
  assert(!math::isNan(mInvMassMatrixSegment));

  // Index
  std::size_t iStart = mDof->mIndexInTree;

  // Assign
  _invMassMat(iStart, _col) = mInvMassMatrixSegment;
}

//==============================================================================
void SingleDofJoint::addInvMassMatrixSegmentTo(Eigen::Vector6d& _acc)
{
  //
  _acc += getRelativeJacobianStatic() * mInvMassMatrixSegment;
}

//==============================================================================
Eigen::VectorXd SingleDofJoint::getSpatialToGeneralized(
    const Eigen::Vector6d& _spatial)
{
  Eigen::VectorXd generalized = Eigen::VectorXd::Zero(1);
  generalized[0] = getRelativeJacobianStatic().dot(_spatial);
  return generalized;
}

}  // namespace dynamics
}  // namespace dart
