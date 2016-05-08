/*
 * Copyright (c) 2015-2016, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Michael X. Grey <mxgrey@gatech.edu>
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

#ifndef DART_DYNAMICS_DETAIL_MULTIDOFJOINT_HPP_
#define DART_DYNAMICS_DETAIL_MULTIDOFJOINT_HPP_

#include "dart/dynamics/MultiDofJoint.hpp"

#define MULTIDOFJOINT_REPORT_DIM_MISMATCH( func, arg )\
  dterr << "[MultiDofJoint::" #func "] Mismatch beteween size of "\
        << #arg " [" << arg .size() << "] and the number of "\
        << "DOFs [" << this->getNumDofs() << "] for Joint named ["\
        << this->getName() << "].\n";\
  assert(false);

#define MULTIDOFJOINT_REPORT_OUT_OF_RANGE( func, index )\
  dterr << "[MultiDofJoint::" << #func << "] The index [" << index\
        << "] is out of range for Joint named [" << this->getName()\
        << "] which has " << this->getNumDofs() << " DOFs.\n";\
  assert(false);

#define MULTIDOFJOINT_REPORT_UNSUPPORTED_ACTUATOR( func )\
  dterr << "[MultiDofJoint::" # func "] Unsupported actuator type ("\
        << Joint::mAspectProperties.mActuatorType << ") for Joint ["\
        << this->getName() << "].\n";\
  assert(false);

#define MULTIDOFJOINT_SET_IF_DIFFERENT( mField, value )\
  if( value == Base::mAspectProperties. mField )\
    return;\
  Base::mAspectProperties. mField = value;\
  Joint::incrementVersion();

namespace dart {
namespace dynamics {

//==============================================================================
//
// These namespace-level definitions are required to enable ODR-use of static
// constexpr member variables.
//
// See this StackOverflow answer: http://stackoverflow.com/a/14396189/111426
//
template <std::size_t DOF>
constexpr std::size_t MultiDofJoint<DOF>::NumDofs;

//==============================================================================
template <std::size_t DOF>
MultiDofJoint<DOF>::~MultiDofJoint()
{
  for (std::size_t i = 0; i < DOF; ++i)
    delete mDofs[i];
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::setProperties(const Properties& _properties)
{
  Joint::setProperties(static_cast<const Joint::Properties&>(_properties));
  setProperties(static_cast<const UniqueProperties&>(_properties));
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::setProperties(const UniqueProperties& _properties)
{
  setAspectProperties(_properties);
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::setAspectState(const AspectState& state)
{
  setCommands(state.mCommands);
  setPositionsStatic(state.mPositions);
  setVelocitiesStatic(state.mVelocities);
  setAccelerationsStatic(state.mAccelerations);
  setForces(state.mForces);
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::setAspectProperties(const AspectProperties& properties)
{
  for(std::size_t i=0; i<DOF; ++i)
  {
    setDofName(i, properties.mDofNames[i], properties.mPreserveDofNames[i]);
    setPositionLowerLimit(i, properties.mPositionLowerLimits[i]);
    setPositionUpperLimit(i, properties.mPositionUpperLimits[i]);
    setInitialPosition(i, properties.mInitialPositions[i]);
    setVelocityLowerLimit(i, properties.mVelocityLowerLimits[i]);
    setVelocityUpperLimit(i, properties.mVelocityUpperLimits[i]);
    setInitialVelocity(i, properties.mInitialVelocities[i]);
    setAccelerationLowerLimit(i, properties.mAccelerationLowerLimits[i]);
    setAccelerationUpperLimit(i, properties.mAccelerationUpperLimits[i]);
    setForceLowerLimit(i, properties.mForceLowerLimits[i]);
    setForceUpperLimit(i, properties.mForceUpperLimits[i]);
    setSpringStiffness(i, properties.mSpringStiffnesses[i]);
    setRestPosition(i, properties.mRestPositions[i]);
    setDampingCoefficient(i, properties.mDampingCoefficients[i]);
    setCoulombFriction(i, properties.mFrictions[i]);
  }
}

//==============================================================================
template <std::size_t DOF>
typename MultiDofJoint<DOF>::Properties
MultiDofJoint<DOF>::getMultiDofJointProperties() const
{
  return MultiDofJoint<DOF>::Properties(
        Joint::mAspectProperties, Base::mAspectProperties);
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::copy(const MultiDofJoint<DOF>& _otherJoint)
{
  if(this == &_otherJoint)
    return;

  setProperties(_otherJoint.getMultiDofJointProperties());
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::copy(const MultiDofJoint<DOF>* _otherJoint)
{
  if(nullptr == _otherJoint)
    return;

  copy(*_otherJoint);
}

//==============================================================================
template <std::size_t DOF>
MultiDofJoint<DOF>& MultiDofJoint<DOF>::operator=(
    const MultiDofJoint<DOF>& _otherJoint)
{
  copy(_otherJoint);
  return *this;
}

//==============================================================================
template <std::size_t DOF>
DegreeOfFreedom* MultiDofJoint<DOF>::getDof(std::size_t _index)
{
  if (_index < DOF)
    return mDofs[_index];

  MULTIDOFJOINT_REPORT_OUT_OF_RANGE(getDof, _index);

  return nullptr;
}

//==============================================================================
template <std::size_t DOF>
const DegreeOfFreedom* MultiDofJoint<DOF>::getDof(std::size_t _index) const
{
  if (_index < DOF)
    return mDofs[_index];

  MULTIDOFJOINT_REPORT_OUT_OF_RANGE(getDof, _index);

  return nullptr;
}

//==============================================================================
template <std::size_t DOF>
const std::string& MultiDofJoint<DOF>::setDofName(std::size_t _index,
                                                  const std::string& _name,
                                                  bool _preserveName)
{
  if(DOF <= _index)
  {
    dterr << "[MultiDofJoint::setDofName] Attempting to set the name of DOF "
          << "index " << _index << ", which is out of bounds for the Joint ["
          << this->getName() << "]. We will set the name of DOF index 0 instead.\n";
    assert(false);
    _index = 0;
  }

  preserveDofName(_index, _preserveName);
  std::string& dofName = Base::mAspectProperties.mDofNames[_index];
  if(_name == dofName)
    return dofName;

  const SkeletonPtr& skel = this->mChildBodyNode?
        this->mChildBodyNode->getSkeleton() : nullptr;
  if(skel)
  {
    dofName =
        skel->mNameMgrForDofs.changeObjectName(mDofs[_index], _name);
  }
  else
    dofName = _name;

  return dofName;
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::preserveDofName(std::size_t _index, bool _preserve)
{
  if (DOF <= _index)
  {
    MULTIDOFJOINT_REPORT_OUT_OF_RANGE(preserveDofName, _index);
    return;
  }

  MULTIDOFJOINT_SET_IF_DIFFERENT( mPreserveDofNames[_index], _preserve );
}

//==============================================================================
template <std::size_t DOF>
bool MultiDofJoint<DOF>::isDofNamePreserved(std::size_t _index) const
{
  if(DOF <= _index)
  {
    MULTIDOFJOINT_REPORT_OUT_OF_RANGE(isDofNamePreserved, _index);
    _index = 0;
  }

  return Base::mAspectProperties.mPreserveDofNames[_index];
}

//==============================================================================
template <std::size_t DOF>
const std::string& MultiDofJoint<DOF>::getDofName(std::size_t _index) const
{
  if(DOF <= _index)
  {
    dterr << "[MultiDofJoint::getDofName] Requested name of DOF index ["
          << _index << "] in Joint [" << this->getName() << "], but that is "
          << "out of bounds (max " << DOF-1 << "). Returning name of DOF 0.\n";
    assert(false);
    return Base::mAspectProperties.mDofNames[0];
  }

  return Base::mAspectProperties.mDofNames[_index];
}

//==============================================================================
template <std::size_t DOF>
std::size_t MultiDofJoint<DOF>::getNumDofs() const
{
  return DOF;
}

//==============================================================================
template <std::size_t DOF>
std::size_t MultiDofJoint<DOF>::getIndexInSkeleton(std::size_t _index) const
{
  if (_index >= getNumDofs())
  {
    MULTIDOFJOINT_REPORT_OUT_OF_RANGE(getIndexInSkeleton, _index);
    return 0;
  }

  return mDofs[_index]->mIndexInSkeleton;
}

//==============================================================================
template <std::size_t DOF>
std::size_t MultiDofJoint<DOF>::getIndexInTree(std::size_t _index) const
{
  if (_index >= getNumDofs())
  {
    MULTIDOFJOINT_REPORT_OUT_OF_RANGE(getIndexInTree, _index);
    return 0;
  }

  return mDofs[_index]->mIndexInTree;
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::setCommand(std::size_t _index, double command)
{
  if (_index >= getNumDofs())
  {
    MULTIDOFJOINT_REPORT_OUT_OF_RANGE(setCommand, _index);
  }

  switch (Joint::mAspectProperties.mActuatorType)
  {
    case Joint::FORCE:
      this->mAspectState.mCommands[_index] = math::clip(command,
            Base::mAspectProperties.mForceLowerLimits[_index],
            Base::mAspectProperties.mForceUpperLimits[_index]);
      break;
    case Joint::PASSIVE:
      if(0.0 != command)
      {
        dtwarn << "[MultiDofJoint::setCommand] Attempting to set a non-zero ("
               << command << ") command for a PASSIVE joint ["
               << this->getName() << "].\n";
      }
      this->mAspectState.mCommands[_index] = command;
      break;
    case Joint::SERVO:
      this->mAspectState.mCommands[_index] = math::clip(command,
            Base::mAspectProperties.mVelocityLowerLimits[_index],
            Base::mAspectProperties.mVelocityUpperLimits[_index]);
      break;
    case Joint::ACCELERATION:
      this->mAspectState.mCommands[_index] = math::clip(command,
            Base::mAspectProperties.mAccelerationLowerLimits[_index],
            Base::mAspectProperties.mAccelerationUpperLimits[_index]);
      break;
    case Joint::VELOCITY:
      this->mAspectState.mCommands[_index] = math::clip(command,
            Base::mAspectProperties.mVelocityLowerLimits[_index],
            Base::mAspectProperties.mVelocityUpperLimits[_index]);
      // TODO: This possibly makes the acceleration to exceed the limits.
      break;
    case Joint::LOCKED:
      if(0.0 != command)
      {
        dtwarn << "[MultiDofJoint::setCommand] Attempting to set a non-zero ("
               << command << ") command for a LOCKED joint [" << this->getName()
               << "].\n";
      }
      this->mAspectState.mCommands[_index] = command;
      break;
    default:
      assert(false);
      break;
  }
}

//==============================================================================
template <std::size_t DOF>
double MultiDofJoint<DOF>::getCommand(std::size_t _index) const
{
  if (_index >= getNumDofs())
  {
    MULTIDOFJOINT_REPORT_OUT_OF_RANGE(getCommand, _index);
    return 0.0;
  }

  return this->mAspectState.mCommands[_index];
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::setCommands(const Eigen::VectorXd& _commands)
{
  if (static_cast<std::size_t>(_commands.size()) != getNumDofs())
  {
    MULTIDOFJOINT_REPORT_DIM_MISMATCH(setCommands, _commands);
    return;
  }

  switch (Joint::mAspectProperties.mActuatorType)
  {
    case Joint::FORCE:
      this->mAspectState.mCommands = math::clip(_commands,
            Base::mAspectProperties.mForceLowerLimits,
            Base::mAspectProperties.mForceUpperLimits);
      break;
    case Joint::PASSIVE:
      if(Vector::Zero() != _commands)
      {
        dtwarn << "[MultiDofJoint::setCommands] Attempting to set a non-zero ("
               << _commands.transpose() << ") command for a PASSIVE joint ["
               << this->getName() << "].\n";
      }
      this->mAspectState.mCommands = _commands;
      break;
    case Joint::SERVO:
      this->mAspectState.mCommands = math::clip(_commands,
            Base::mAspectProperties.mVelocityLowerLimits,
            Base::mAspectProperties.mVelocityUpperLimits);
      break;
    case Joint::ACCELERATION:
      this->mAspectState.mCommands = math::clip(_commands,
            Base::mAspectProperties.mAccelerationLowerLimits,
            Base::mAspectProperties.mAccelerationUpperLimits);
      break;
    case Joint::VELOCITY:
      this->mAspectState.mCommands = math::clip(_commands,
            Base::mAspectProperties.mVelocityLowerLimits,
            Base::mAspectProperties.mVelocityUpperLimits);
      // TODO: This possibly makes the acceleration to exceed the limits.
      break;
    case Joint::LOCKED:
      if(Vector::Zero() != _commands)
      {
        dtwarn << "[MultiDofJoint::setCommands] Attempting to set a non-zero ("
               << _commands.transpose() << ") command for a LOCKED joint ["
               << this->getName() << "].\n";
      }
      this->mAspectState.mCommands = _commands;
      break;
    default:
      assert(false);
      break;
  }
}

//==============================================================================
template <std::size_t DOF>
Eigen::VectorXd MultiDofJoint<DOF>::getCommands() const
{
  return this->mAspectState.mCommands;
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::resetCommands()
{
  this->mAspectState.mCommands.setZero();
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::setPosition(std::size_t _index, double _position)
{
  if (_index >= getNumDofs())
  {
    MULTIDOFJOINT_REPORT_OUT_OF_RANGE(setPosition, _index);
    return;
  }

  if(this->mAspectState.mPositions[_index] == _position)
    return;

  // Note: It would not make much sense to use setPositionsStatic() here
  this->mAspectState.mPositions[_index] = _position;
  this->notifyPositionUpdate();
}

//==============================================================================
template <std::size_t DOF>
double MultiDofJoint<DOF>::getPosition(std::size_t _index) const
{
  if (_index >= getNumDofs())
  {
    MULTIDOFJOINT_REPORT_OUT_OF_RANGE(getPosition, _index);
    return 0.0;
  }

  return getPositionsStatic()[_index];
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::setPositions(const Eigen::VectorXd& _positions)
{
  if (static_cast<std::size_t>(_positions.size()) != getNumDofs())
  {
    MULTIDOFJOINT_REPORT_DIM_MISMATCH(setPositions, _positions);
    return;
  }

  setPositionsStatic(_positions);
}

//==============================================================================
template <std::size_t DOF>
Eigen::VectorXd MultiDofJoint<DOF>::getPositions() const
{
  return getPositionsStatic();
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::setPositionLowerLimit(std::size_t _index, double _position)
{
  if (_index >= getNumDofs())
  {
    MULTIDOFJOINT_REPORT_OUT_OF_RANGE(setPositionLowerLimit, _index);
    return;
  }

  MULTIDOFJOINT_SET_IF_DIFFERENT( mPositionLowerLimits[_index], _position );
}

//==============================================================================
template <std::size_t DOF>
double MultiDofJoint<DOF>::getPositionLowerLimit(std::size_t _index) const
{
  if (_index >= getNumDofs())
  {
    MULTIDOFJOINT_REPORT_OUT_OF_RANGE(getPositionLowerLimit, _index);
    return 0.0;
  }

  return Base::mAspectProperties.mPositionLowerLimits[_index];
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::setPositionUpperLimit(std::size_t _index, double _position)
{
  if (_index >= getNumDofs())
  {
    MULTIDOFJOINT_REPORT_OUT_OF_RANGE(setPositionUpperLimit, _index);
    return;
  }

  MULTIDOFJOINT_SET_IF_DIFFERENT( mPositionUpperLimits[_index], _position );
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::resetPosition(std::size_t _index)
{
  if (_index >= getNumDofs())
  {
    MULTIDOFJOINT_REPORT_OUT_OF_RANGE(resetPosition, _index);
    return;
  }

  setPosition(_index, Base::mAspectProperties.mInitialPositions[_index]);
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::resetPositions()
{
  setPositionsStatic(Base::mAspectProperties.mInitialPositions);
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::setInitialPosition(std::size_t _index, double _initial)
{
  if (_index >= getNumDofs())
  {
    MULTIDOFJOINT_REPORT_OUT_OF_RANGE(setInitialPosition, _index);
    return;
  }

  MULTIDOFJOINT_SET_IF_DIFFERENT( mInitialPositions[_index], _initial );
}

//==============================================================================
template <std::size_t DOF>
double MultiDofJoint<DOF>::getInitialPosition(std::size_t _index) const
{
  if (_index >= getNumDofs())
  {
    MULTIDOFJOINT_REPORT_OUT_OF_RANGE(getInitialPosition, _index);
    return 0.0;
  }

  return Base::mAspectProperties.mInitialPositions[_index];
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::setInitialPositions(const Eigen::VectorXd& _initial)
{
  if ( static_cast<std::size_t>(_initial.size()) != getNumDofs() )
  {
    MULTIDOFJOINT_REPORT_DIM_MISMATCH(setInitialPositions, _initial);
    return;
  }

  MULTIDOFJOINT_SET_IF_DIFFERENT( mInitialPositions, _initial);
}

//==============================================================================
template <std::size_t DOF>
Eigen::VectorXd MultiDofJoint<DOF>::getInitialPositions() const
{
  return Base::mAspectProperties.mInitialPositions;
}

//==============================================================================
template <std::size_t DOF>
double MultiDofJoint<DOF>::getPositionUpperLimit(std::size_t _index) const
{
  if (_index >= getNumDofs())
  {
    MULTIDOFJOINT_REPORT_OUT_OF_RANGE(getPositionUpperLimit, _index);
    return 0.0;
  }

  return Base::mAspectProperties.mPositionUpperLimits[_index];
}

//==============================================================================
template <std::size_t DOF>
bool MultiDofJoint<DOF>::hasPositionLimit(std::size_t _index) const
{
  if (_index >= getNumDofs())
  {
    MULTIDOFJOINT_REPORT_OUT_OF_RANGE(hasPositionLimit, _index);
    return true;
  }

  return std::isfinite(Base::mAspectProperties.mPositionUpperLimits[_index])
      || std::isfinite(Base::mAspectProperties.mPositionLowerLimits[_index]);
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::setVelocity(std::size_t _index, double _velocity)
{
  if (_index >= getNumDofs())
  {
    MULTIDOFJOINT_REPORT_OUT_OF_RANGE(setVelocity, _index);
    return;
  }

  if(this->mAspectState.mVelocities[_index] == _velocity)
    return;

  // Note: It would not make much sense to use setVelocitiesStatic() here
  this->mAspectState.mVelocities[_index] = _velocity;
  this->notifyVelocityUpdate();

  if (Joint::mAspectProperties.mActuatorType == Joint::VELOCITY)
    this->mAspectState.mCommands[_index] = this->getVelocitiesStatic()[_index];
}

//==============================================================================
template <std::size_t DOF>
double MultiDofJoint<DOF>::getVelocity(std::size_t _index) const
{
  if (_index >= getNumDofs())
  {
    MULTIDOFJOINT_REPORT_OUT_OF_RANGE(getVelocity, _index);
    return 0.0;
  }

  return getVelocitiesStatic()[_index];
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::setVelocities(const Eigen::VectorXd& _velocities)
{
  if (static_cast<std::size_t>(_velocities.size()) != getNumDofs())
  {
    MULTIDOFJOINT_REPORT_DIM_MISMATCH(setVelocities, _velocities);
    return;
  }

  setVelocitiesStatic(_velocities);

  if (Joint::mAspectProperties.mActuatorType == Joint::VELOCITY)
    this->mAspectState.mCommands = this->getVelocitiesStatic();
}

//==============================================================================
template <std::size_t DOF>
Eigen::VectorXd MultiDofJoint<DOF>::getVelocities() const
{
  return getVelocitiesStatic();
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::setVelocityLowerLimit(std::size_t _index, double _velocity)
{
  if (_index >= getNumDofs())
  {
    MULTIDOFJOINT_REPORT_OUT_OF_RANGE( setVelocityLowerLimit, _index );
    return;
  }

  MULTIDOFJOINT_SET_IF_DIFFERENT( mVelocityLowerLimits[_index], _velocity );
}

//==============================================================================
template <std::size_t DOF>
double MultiDofJoint<DOF>::getVelocityLowerLimit(std::size_t _index) const
{
  if (_index >= getNumDofs())
  {
    MULTIDOFJOINT_REPORT_OUT_OF_RANGE( getVelocityLowerLimit, _index);
    return 0.0;
  }

  return Base::mAspectProperties.mVelocityLowerLimits[_index];
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::setVelocityUpperLimit(std::size_t _index, double _velocity)
{
  if (_index >= getNumDofs())
  {
    MULTIDOFJOINT_REPORT_OUT_OF_RANGE( setVelocityUpperLimit, _index );
    return;
  }

  MULTIDOFJOINT_SET_IF_DIFFERENT( mVelocityUpperLimits[_index], _velocity );
}

//==============================================================================
template <std::size_t DOF>
double MultiDofJoint<DOF>::getVelocityUpperLimit(std::size_t _index) const
{
  if (_index >= getNumDofs())
  {
    MULTIDOFJOINT_REPORT_OUT_OF_RANGE( getVelocityUpperLimit, _index );
    return 0.0;
  }

  return Base::mAspectProperties.mVelocityUpperLimits[_index];
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::resetVelocity(std::size_t _index)
{
  if (_index >= getNumDofs())
  {
    MULTIDOFJOINT_REPORT_OUT_OF_RANGE( resetVelocity, _index );
    return;
  }

  setVelocity(_index, Base::mAspectProperties.mInitialVelocities[_index]);
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::resetVelocities()
{
  setVelocitiesStatic(Base::mAspectProperties.mInitialVelocities);
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::setInitialVelocity(std::size_t _index, double _initial)
{
  if (_index >= getNumDofs())
  {
    MULTIDOFJOINT_REPORT_OUT_OF_RANGE( setInitialVelocity, _index );
    return;
  }

  MULTIDOFJOINT_SET_IF_DIFFERENT( mInitialVelocities[_index], _initial );
}

//==============================================================================
template <std::size_t DOF>
double MultiDofJoint<DOF>::getInitialVelocity(std::size_t _index) const
{
  if (_index >= getNumDofs())
  {
    MULTIDOFJOINT_REPORT_OUT_OF_RANGE( getInitialVelocity, _index );
    return 0.0;
  }

  return Base::mAspectProperties.mInitialVelocities[_index];
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::setInitialVelocities(const Eigen::VectorXd& _initial)
{
  if ( static_cast<std::size_t>(_initial.size()) != getNumDofs() )
  {
    MULTIDOFJOINT_REPORT_DIM_MISMATCH( setInitialVelocities, _initial );
    return;
  }

  MULTIDOFJOINT_SET_IF_DIFFERENT( mInitialVelocities, _initial );
}

//==============================================================================
template <std::size_t DOF>
Eigen::VectorXd MultiDofJoint<DOF>::getInitialVelocities() const
{
  return Base::mAspectProperties.mInitialVelocities;
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::setAcceleration(std::size_t _index, double _acceleration)
{
  if (_index >= getNumDofs())
  {
    MULTIDOFJOINT_REPORT_OUT_OF_RANGE( setAcceleration, _index );
    return;
  }

  if(this->mAspectState.mAccelerations[_index] == _acceleration)
    return;

  // Note: It would not make much sense to use setAccelerationsStatic() here
  this->mAspectState.mAccelerations[_index] = _acceleration;
  this->notifyAccelerationUpdate();

  if (Joint::mAspectProperties.mActuatorType == Joint::ACCELERATION)
    this->mAspectState.mCommands[_index] = this->getAccelerationsStatic()[_index];
}

//==============================================================================
template <std::size_t DOF>
double MultiDofJoint<DOF>::getAcceleration(std::size_t _index) const
{
  if (_index >= getNumDofs())
  {
    MULTIDOFJOINT_REPORT_OUT_OF_RANGE( getAcceleration, _index );
    return 0.0;
  }

  return getAccelerationsStatic()[_index];
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::setAccelerations(const Eigen::VectorXd& _accelerations)
{
  if (static_cast<std::size_t>(_accelerations.size()) != getNumDofs())
  {
    MULTIDOFJOINT_REPORT_DIM_MISMATCH( setAccelerations, _accelerations );
    return;
  }

  setAccelerationsStatic(_accelerations);

  if (Joint::mAspectProperties.mActuatorType == Joint::ACCELERATION)
    this->mAspectState.mCommands = this->getAccelerationsStatic();
}

//==============================================================================
template <std::size_t DOF>
Eigen::VectorXd MultiDofJoint<DOF>::getAccelerations() const
{
  return getAccelerationsStatic();
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::resetAccelerations()
{
  setAccelerationsStatic(Eigen::Matrix<double, DOF, 1>::Zero());
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::setAccelerationLowerLimit(std::size_t _index,
                                                   double _acceleration)
{
  if (_index >= getNumDofs())
  {
    MULTIDOFJOINT_REPORT_OUT_OF_RANGE(setAccelerationLowerLimit, _index);
    return;
  }

  MULTIDOFJOINT_SET_IF_DIFFERENT( mAccelerationLowerLimits[_index],
                                  _acceleration );
}

//==============================================================================
template <std::size_t DOF>
double MultiDofJoint<DOF>::getAccelerationLowerLimit(std::size_t _index) const
{
  if (_index >= getNumDofs())
  {
    MULTIDOFJOINT_REPORT_OUT_OF_RANGE(getAccelerationLowerLimit, _index);
    return 0.0;
  }

  return Base::mAspectProperties.mAccelerationLowerLimits[_index];
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::setAccelerationUpperLimit(std::size_t _index,
                                                   double _acceleration)
{
  if (_index >= getNumDofs())
  {
    MULTIDOFJOINT_REPORT_OUT_OF_RANGE(setAccelerationUpperLimit, _index)
    return;
  }

  MULTIDOFJOINT_SET_IF_DIFFERENT( mAccelerationUpperLimits[_index],
                                  _acceleration);
}

//==============================================================================
template <std::size_t DOF>
double MultiDofJoint<DOF>::getAccelerationUpperLimit(std::size_t _index) const
{
  if (_index >= getNumDofs())
  {
    MULTIDOFJOINT_REPORT_OUT_OF_RANGE(getAccelerationUpperLimit, _index);
    return 0.0;
  }

  return Base::mAspectProperties.mAccelerationUpperLimits[_index];
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::setPositionsStatic(const Vector& _positions)
{
  if(this->mAspectState.mPositions == _positions)
    return;

  this->mAspectState.mPositions = _positions;
  this->notifyPositionUpdate();
}

//==============================================================================
template <std::size_t DOF>
const typename MultiDofJoint<DOF>::Vector&
MultiDofJoint<DOF>::getPositionsStatic() const
{
  return this->mAspectState.mPositions;
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::setVelocitiesStatic(const Vector& _velocities)
{
  if(this->mAspectState.mVelocities == _velocities)
    return;

  this->mAspectState.mVelocities = _velocities;
  this->notifyVelocityUpdate();
}

//==============================================================================
template <std::size_t DOF>
const typename MultiDofJoint<DOF>::Vector&
MultiDofJoint<DOF>::getVelocitiesStatic() const
{
  return this->mAspectState.mVelocities;
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::setAccelerationsStatic(const Vector& _accels)
{
  if(this->mAspectState.mAccelerations == _accels)
    return;

  this->mAspectState.mAccelerations = _accels;
  this->notifyAccelerationUpdate();
}

//==============================================================================
template <std::size_t DOF>
const typename MultiDofJoint<DOF>::Vector&
MultiDofJoint<DOF>::getAccelerationsStatic() const
{
  return this->mAspectState.mAccelerations;
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::setForce(std::size_t _index, double _force)
{
  if (_index >= getNumDofs())
  {
    MULTIDOFJOINT_REPORT_OUT_OF_RANGE(setForce, _index);
    return;
  }

  this->mAspectState.mForces[_index] = _force;

  if (Joint::mAspectProperties.mActuatorType == Joint::FORCE)
    this->mAspectState.mCommands[_index] = this->mAspectState.mForces[_index];
}

//==============================================================================
template <std::size_t DOF>
double MultiDofJoint<DOF>::getForce(std::size_t _index)
{
  if (_index >= getNumDofs())
  {
    MULTIDOFJOINT_REPORT_OUT_OF_RANGE(getForce, _index);
    return 0.0;
  }

  return this->mAspectState.mForces[_index];
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::setForces(const Eigen::VectorXd& _forces)
{
  if (static_cast<std::size_t>(_forces.size()) != getNumDofs())
  {
    MULTIDOFJOINT_REPORT_DIM_MISMATCH(setForces, _forces);
    return;
  }

  this->mAspectState.mForces = _forces;

  if (Joint::mAspectProperties.mActuatorType == Joint::FORCE)
    this->mAspectState.mCommands = this->mAspectState.mForces;
}

//==============================================================================
template <std::size_t DOF>
Eigen::VectorXd MultiDofJoint<DOF>::getForces() const
{
  return this->mAspectState.mForces;
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::resetForces()
{
  this->mAspectState.mForces.setZero();

  if (Joint::mAspectProperties.mActuatorType == Joint::FORCE)
    this->mAspectState.mCommands = this->mAspectState.mForces;
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::setForceLowerLimit(std::size_t _index, double _force)
{
  if (_index >= getNumDofs())
  {
    MULTIDOFJOINT_REPORT_OUT_OF_RANGE(setForceLowerLimit, _index);
    return;
  }

  MULTIDOFJOINT_SET_IF_DIFFERENT( mForceLowerLimits[_index], _force );
}

//==============================================================================
template <std::size_t DOF>
double MultiDofJoint<DOF>::getForceLowerLimit(std::size_t _index) const
{
  if (_index >= getNumDofs())
  {
    MULTIDOFJOINT_REPORT_OUT_OF_RANGE(getForceLowerLimit, _index);
    return 0.0;
  }

  return Base::mAspectProperties.mForceLowerLimits[_index];
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::setForceUpperLimit(std::size_t _index, double _force)
{
  if (_index >= getNumDofs())
  {
    MULTIDOFJOINT_REPORT_OUT_OF_RANGE(setForceUpperLimit, _index);
    return;
  }

  MULTIDOFJOINT_SET_IF_DIFFERENT( mForceUpperLimits[_index], _force );
}

//==============================================================================
template <std::size_t DOF>
double MultiDofJoint<DOF>::getForceUpperLimit(std::size_t _index) const
{
  if (_index >= getNumDofs())
  {
    MULTIDOFJOINT_REPORT_OUT_OF_RANGE(getForceUpperLimit, _index);
    return 0.0;
  }

  return Base::mAspectProperties.mForceUpperLimits[_index];
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::setVelocityChange(std::size_t _index,
                                           double _velocityChange)
{
  if (_index >= getNumDofs())
  {
    MULTIDOFJOINT_REPORT_OUT_OF_RANGE(setVelocityChange, _index);
    return;
  }

  mVelocityChanges[_index] = _velocityChange;
}

//==============================================================================
template <std::size_t DOF>
double MultiDofJoint<DOF>::getVelocityChange(std::size_t _index) const
{
  if (_index >= getNumDofs())
  {
    MULTIDOFJOINT_REPORT_OUT_OF_RANGE(getVelocityChange, _index);
    return 0.0;
  }

  return mVelocityChanges[_index];
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::resetVelocityChanges()
{
  mVelocityChanges.setZero();
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::setConstraintImpulse(std::size_t _index, double _impulse)
{
  if (_index >= getNumDofs())
  {
    MULTIDOFJOINT_REPORT_OUT_OF_RANGE(setConstraintImpulse, _index);
    return;
  }

  mConstraintImpulses[_index] = _impulse;
}

//==============================================================================
template <std::size_t DOF>
double MultiDofJoint<DOF>::getConstraintImpulse(std::size_t _index) const
{
  if (_index >= getNumDofs())
  {
    MULTIDOFJOINT_REPORT_OUT_OF_RANGE(getConstraintImpulse, _index);
    return 0.0;
  }

  return mConstraintImpulses[_index];
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::resetConstraintImpulses()
{
  mConstraintImpulses.setZero();
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::integratePositions(double _dt)
{
  setPositionsStatic(getPositionsStatic() + getVelocitiesStatic() * _dt);
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::integrateVelocities(double _dt)
{
  setVelocitiesStatic(getVelocitiesStatic() + getAccelerationsStatic() * _dt);
}

//==============================================================================
template <std::size_t DOF>
Eigen::VectorXd MultiDofJoint<DOF>::getPositionDifferences(
    const Eigen::VectorXd& _q2, const Eigen::VectorXd& _q1) const
{
  if (static_cast<std::size_t>(_q1.size()) != getNumDofs()
      || static_cast<std::size_t>(_q2.size()) != getNumDofs())
  {
    dterr << "[MultiDofJoint::getPositionsDifference] q1's size [" << _q1.size()
          << "] or q2's size [" << _q2.size() << "] must both equal the dof ["
          << this->getNumDofs() << "] for Joint [" << this->getName() << "].\n";
    assert(false);
    return Eigen::VectorXd::Zero(getNumDofs());
  }

  return getPositionDifferencesStatic(_q2, _q1);
}

//==============================================================================
template <std::size_t DOF>
Eigen::Matrix<double, DOF, 1> MultiDofJoint<DOF>::getPositionDifferencesStatic(
    const Vector& _q2, const Vector& _q1) const
{
  return _q2 - _q1;
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::setSpringStiffness(std::size_t _index, double _k)
{
  if (_index >= getNumDofs())
  {
    MULTIDOFJOINT_REPORT_OUT_OF_RANGE(setSpringStiffness, _index);
    return;
  }

  assert(_k >= 0.0);

  MULTIDOFJOINT_SET_IF_DIFFERENT( mSpringStiffnesses[_index], _k );
}

//==============================================================================
template <std::size_t DOF>
double MultiDofJoint<DOF>::getSpringStiffness(std::size_t _index) const
{
  if (_index >= getNumDofs())
  {
    MULTIDOFJOINT_REPORT_OUT_OF_RANGE(getSpringStiffness, _index);
    return 0.0;
  }

  return Base::mAspectProperties.mSpringStiffnesses[_index];
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::setRestPosition(std::size_t _index, double _q0)
{
  if (_index >= getNumDofs())
  {
    MULTIDOFJOINT_REPORT_OUT_OF_RANGE(setRestPosition, _index);
    return;
  }

  if (Base::mAspectProperties.mPositionLowerLimits[_index] > _q0
      || Base::mAspectProperties.mPositionUpperLimits[_index] < _q0)
  {
    dtwarn << "[MultiDofJoint::setRestPosition] Value of _q0 [" << _q0
           << "], is out of the limit range ["
           << Base::mAspectProperties.mPositionLowerLimits[_index] << ", "
           << Base::mAspectProperties.mPositionUpperLimits[_index]
           << "] for index [" << _index << "] of Joint [" << this->getName()
           << "].\n";
    return;
  }

  MULTIDOFJOINT_SET_IF_DIFFERENT( mRestPositions[_index], _q0 );
}

//==============================================================================
template <std::size_t DOF>
double MultiDofJoint<DOF>::getRestPosition(std::size_t _index) const
{
  if (_index >= getNumDofs())
  {
    MULTIDOFJOINT_REPORT_OUT_OF_RANGE(getRestPosition, _index);
    return 0.0;
  }

  return Base::mAspectProperties.mRestPositions[_index];
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::setDampingCoefficient(std::size_t _index, double _d)
{
  if (_index >= getNumDofs())
  {
    MULTIDOFJOINT_REPORT_OUT_OF_RANGE(setDampingCoefficient, _index);
    return;
  }

  assert(_d >= 0.0);

  MULTIDOFJOINT_SET_IF_DIFFERENT( mDampingCoefficients[_index], _d );
}

//==============================================================================
template <std::size_t DOF>
double MultiDofJoint<DOF>::getDampingCoefficient(std::size_t _index) const
{
  if (_index >= getNumDofs())
  {
    MULTIDOFJOINT_REPORT_OUT_OF_RANGE(getDampingCoefficient, _index);
    return 0.0;
  }

  return Base::mAspectProperties.mDampingCoefficients[_index];
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::setCoulombFriction(std::size_t _index, double _friction)
{
  if (_index >= getNumDofs())
  {
    MULTIDOFJOINT_REPORT_OUT_OF_RANGE(setCoulombFriction, _index);
    return;
  }

  assert(_friction >= 0.0);

  MULTIDOFJOINT_SET_IF_DIFFERENT( mFrictions[_index], _friction );
}

//==============================================================================
template <std::size_t DOF>
double MultiDofJoint<DOF>::getCoulombFriction(std::size_t _index) const
{
  if (_index >= getNumDofs())
  {
    MULTIDOFJOINT_REPORT_OUT_OF_RANGE(getCoulombFriction, _index);
    return 0.0;
  }

  return Base::mAspectProperties.mFrictions[_index];
}

//==============================================================================
template <std::size_t DOF>
double MultiDofJoint<DOF>::getPotentialEnergy() const
{
  // Spring energy
  Eigen::VectorXd displacement =
      getPositionsStatic() - Base::mAspectProperties.mRestPositions;
  double pe = 0.5 * displacement.dot(
        Base::mAspectProperties.mSpringStiffnesses.asDiagonal()
        * displacement);

  return pe;
}

//==============================================================================
template <std::size_t DOF>
Eigen::Vector6d MultiDofJoint<DOF>::getBodyConstraintWrench() const
{
  assert(this->mChildBodyNode);
  return this->mChildBodyNode->getBodyForce()
          - this->getRelativeJacobianStatic() * this->mAspectState.mForces;
}

//==============================================================================
template <std::size_t DOF>
MultiDofJoint<DOF>::MultiDofJoint(const Properties& properties)
  : mVelocityChanges(Vector::Zero()),
    mImpulses(Vector::Zero()),
    mConstraintImpulses(Vector::Zero()),
    mJacobian(Eigen::Matrix<double, 6, DOF>::Zero()),
    mJacobianDeriv(Eigen::Matrix<double, 6, DOF>::Zero()),
    mInvProjArtInertia(Eigen::Matrix<double, DOF, DOF>::Zero()),
    mInvProjArtInertiaImplicit(Eigen::Matrix<double, DOF, DOF>::Zero()),
    mTotalForce(Vector::Zero()),
    mTotalImpulse(Vector::Zero())
{
  for (std::size_t i = 0; i < DOF; ++i)
    mDofs[i] = this->createDofPointer(i);

  // Joint and MultiDofJoint Aspects must be created by the most derived class.
  this->mAspectState.mPositions = properties.mInitialPositions;
  this->mAspectState.mVelocities = properties.mInitialVelocities;
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::registerDofs()
{
  const SkeletonPtr& skel = this->mChildBodyNode->getSkeleton();
  for (std::size_t i = 0; i < DOF; ++i)
  {
    Base::mAspectProperties.mDofNames[i] =
        skel->mNameMgrForDofs.issueNewNameAndAdd(mDofs[i]->getName(), mDofs[i]);
  }
}

//==============================================================================
template <std::size_t DOF>
const math::Jacobian MultiDofJoint<DOF>::getRelativeJacobian() const
{
  return getRelativeJacobianStatic();
}

//==============================================================================
template <std::size_t DOF>
const Eigen::Matrix<double, 6, DOF>&
MultiDofJoint<DOF>::getRelativeJacobianStatic() const
{
  if(this->mIsRelativeJacobianDirty)
  {
    this->updateRelativeJacobian(false);
    this->mIsRelativeJacobianDirty = false;
  }
  return mJacobian;
}

//==============================================================================
template <std::size_t DOF>
math::Jacobian MultiDofJoint<DOF>::getRelativeJacobian(
    const Eigen::VectorXd& _positions) const
{
  return getRelativeJacobianStatic(_positions);
}

//==============================================================================
template <std::size_t DOF>
const math::Jacobian MultiDofJoint<DOF>::getRelativeJacobianTimeDeriv() const
{
  if(this->mIsRelativeJacobianTimeDerivDirty)
  {
    this->updateRelativeJacobianTimeDeriv();
    this->mIsRelativeJacobianTimeDerivDirty = false;
  }
  return mJacobianDeriv;
}

//==============================================================================
template <std::size_t DOF>
const Eigen::Matrix<double, 6, DOF>&
MultiDofJoint<DOF>::getRelativeJacobianTimeDerivStatic() const
{
  if(this->mIsRelativeJacobianTimeDerivDirty)
  {
    this->updateRelativeJacobianTimeDeriv();
    this->mIsRelativeJacobianTimeDerivDirty = false;
  }
  return mJacobianDeriv;
}

//==============================================================================
template <std::size_t DOF>
const Eigen::Matrix<double, DOF, DOF>&
MultiDofJoint<DOF>::getInvProjArtInertia() const
{
  Joint::updateArticulatedInertia();
  return mInvProjArtInertia;
}

//==============================================================================
template <std::size_t DOF>
const Eigen::Matrix<double, DOF, DOF>&
MultiDofJoint<DOF>::getInvProjArtInertiaImplicit() const
{
  Joint::updateArticulatedInertia();
  return mInvProjArtInertiaImplicit;
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::updateRelativeSpatialVelocity() const
{
  this->mSpatialVelocity =
      this->getRelativeJacobianStatic() * this->getVelocitiesStatic();
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::updateRelativeSpatialAcceleration() const
{
  this->mSpatialAcceleration =
      this->getRelativePrimaryAcceleration()
      + this->getRelativeJacobianTimeDerivStatic() * this->getVelocitiesStatic();
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::updateRelativePrimaryAcceleration() const
{
  this->mPrimaryAcceleration =
      this->getRelativeJacobianStatic() * this->getAccelerationsStatic();
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::addVelocityTo(Eigen::Vector6d& _vel)
{
  // Add joint velocity to _vel
  _vel.noalias() += getRelativeJacobianStatic() * getVelocitiesStatic();

  // Verification
  assert(!math::isNan(_vel));
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::setPartialAccelerationTo(
    Eigen::Vector6d& _partialAcceleration,
    const Eigen::Vector6d& _childVelocity)
{
  // ad(V, S * dq) + dS * dq
  _partialAcceleration = math::ad(_childVelocity,
                      getRelativeJacobianStatic() * getVelocitiesStatic())
                    + getRelativeJacobianTimeDerivStatic() * getVelocitiesStatic();
  // Verification
  assert(!math::isNan(_partialAcceleration));
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::addAccelerationTo(Eigen::Vector6d& _acc)
{
  // Add joint acceleration to _acc
  _acc.noalias() += getRelativeJacobianStatic() * getAccelerationsStatic();

  // Verification
  assert(!math::isNan(_acc));
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::addVelocityChangeTo(Eigen::Vector6d& _velocityChange)
{
  // Add joint velocity change to _velocityChange
  _velocityChange.noalias() += getRelativeJacobianStatic() * mVelocityChanges;

  // Verification
  assert(!math::isNan(_velocityChange));
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::addChildArtInertiaTo(
    Eigen::Matrix6d& _parentArtInertia,
    const Eigen::Matrix6d& _childArtInertia)
{
  switch (Joint::mAspectProperties.mActuatorType)
  {
    case Joint::FORCE:
    case Joint::PASSIVE:
    case Joint::SERVO:
      addChildArtInertiaToDynamic(_parentArtInertia,
                                       _childArtInertia);
      break;
    case Joint::ACCELERATION:
    case Joint::VELOCITY:
    case Joint::LOCKED:
      addChildArtInertiaToKinematic(_parentArtInertia,
                                    _childArtInertia);
      break;
    default:
      MULTIDOFJOINT_REPORT_UNSUPPORTED_ACTUATOR(addChildArtInertiaTo);
      break;
  }
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::addChildArtInertiaToDynamic(
    Eigen::Matrix6d& _parentArtInertia,
    const Eigen::Matrix6d& _childArtInertia)
{
  // Child body's articulated inertia
  Eigen::Matrix<double, 6, DOF> AIS = _childArtInertia * getRelativeJacobianStatic();
  Eigen::Matrix6d PI = _childArtInertia;
  PI.noalias() -= AIS * mInvProjArtInertia * AIS.transpose();
  assert(!math::isNan(PI));

  // Add child body's articulated inertia to parent body's articulated inertia.
  // Note that mT should be updated.
  _parentArtInertia += math::transformInertia(
        this->getRelativeTransform().inverse(), PI);
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::addChildArtInertiaToKinematic(
    Eigen::Matrix6d& _parentArtInertia,
    const Eigen::Matrix6d& _childArtInertia)
{
  // Add child body's articulated inertia to parent body's articulated inertia.
  // Note that mT should be updated.
  _parentArtInertia += math::transformInertia(
        this->getRelativeTransform().inverse(), _childArtInertia);
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::addChildArtInertiaImplicitTo(
    Eigen::Matrix6d& _parentArtInertia,
    const Eigen::Matrix6d& _childArtInertia)
{
  switch (Joint::mAspectProperties.mActuatorType)
  {
    case Joint::FORCE:
    case Joint::PASSIVE:
    case Joint::SERVO:
      addChildArtInertiaImplicitToDynamic(_parentArtInertia,
                                          _childArtInertia);
      break;
    case Joint::ACCELERATION:
    case Joint::VELOCITY:
    case Joint::LOCKED:
      addChildArtInertiaImplicitToKinematic(_parentArtInertia,
                                            _childArtInertia);
      break;
    default:
      MULTIDOFJOINT_REPORT_UNSUPPORTED_ACTUATOR(addChildArtInertiaImplicitTo);
      break;
  }
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::addChildArtInertiaImplicitToDynamic(
    Eigen::Matrix6d& _parentArtInertia,
    const Eigen::Matrix6d& _childArtInertia)
{
  // Child body's articulated inertia
  Eigen::Matrix<double, 6, DOF> AIS = _childArtInertia * getRelativeJacobianStatic();
  Eigen::Matrix6d PI = _childArtInertia;
  PI.noalias() -= AIS * mInvProjArtInertiaImplicit * AIS.transpose();
  assert(!math::isNan(PI));

  // Add child body's articulated inertia to parent body's articulated inertia.
  // Note that mT should be updated.
  _parentArtInertia += math::transformInertia(
        this->getRelativeTransform().inverse(), PI);
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::addChildArtInertiaImplicitToKinematic(
    Eigen::Matrix6d& _parentArtInertia,
    const Eigen::Matrix6d& _childArtInertia)
{
  // Add child body's articulated inertia to parent body's articulated inertia.
  // Note that mT should be updated.
  _parentArtInertia += math::transformInertia(
        this->getRelativeTransform().inverse(), _childArtInertia);
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::updateInvProjArtInertia(
    const Eigen::Matrix6d& _artInertia)
{
  switch (Joint::mAspectProperties.mActuatorType)
  {
    case Joint::FORCE:
    case Joint::PASSIVE:
    case Joint::SERVO:
      updateInvProjArtInertiaDynamic(_artInertia);
      break;
    case Joint::ACCELERATION:
    case Joint::VELOCITY:
    case Joint::LOCKED:
      updateInvProjArtInertiaKinematic(_artInertia);
      break;
    default:
      MULTIDOFJOINT_REPORT_UNSUPPORTED_ACTUATOR(updateInvProjArtInertia);
      break;
  }
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::updateInvProjArtInertiaDynamic(
    const Eigen::Matrix6d& _artInertia)
{
  // Projected articulated inertia
  const Eigen::Matrix<double, 6, DOF>& Jacobian = getRelativeJacobianStatic();
  const Eigen::Matrix<double, DOF, DOF> projAI
      = Jacobian.transpose() * _artInertia * Jacobian;

  // Inversion of projected articulated inertia
  //mInvProjArtInertia = projAI.inverse();
  mInvProjArtInertia
      = projAI.ldlt().solve(Eigen::Matrix<double, DOF, DOF>::Identity());

  // Verification
  assert(!math::isNan(mInvProjArtInertia));
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::updateInvProjArtInertiaKinematic(
    const Eigen::Matrix6d& /*_artInertia*/)
{
  // Do nothing
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::updateInvProjArtInertiaImplicit(
    const Eigen::Matrix6d& _artInertia,
    double _timeStep)
{
  switch (Joint::mAspectProperties.mActuatorType)
  {
    case Joint::FORCE:
    case Joint::PASSIVE:
    case Joint::SERVO:
      updateInvProjArtInertiaImplicitDynamic(_artInertia, _timeStep);
      break;
    case Joint::ACCELERATION:
    case Joint::VELOCITY:
    case Joint::LOCKED:
      updateInvProjArtInertiaImplicitKinematic(_artInertia, _timeStep);
      break;
    default:
      MULTIDOFJOINT_REPORT_UNSUPPORTED_ACTUATOR(
            updateInvProjArtInertiaImplicit);
      break;
  }
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::updateInvProjArtInertiaImplicitDynamic(
    const Eigen::Matrix6d& _artInertia,
    double _timeStep)
{
  // Projected articulated inertia
  const Eigen::Matrix<double, 6, DOF>& Jacobian = getRelativeJacobianStatic();
  Eigen::Matrix<double, DOF, DOF> projAI
      = Jacobian.transpose() * _artInertia * Jacobian;

  // Add additional inertia for implicit damping and spring force
  for (std::size_t i = 0; i < DOF; ++i)
  {
    projAI(i, i) += _timeStep * Base::mAspectProperties.mDampingCoefficients[i]
        + _timeStep * _timeStep * Base::mAspectProperties.mSpringStiffnesses[i];
  }

  // Inversion of projected articulated inertia
  //    mInvProjArtInertiaImplicit = projAI.inverse();
  mInvProjArtInertiaImplicit
      = projAI.ldlt().solve(Eigen::Matrix<double, DOF, DOF>::Identity());

  // Verification
  assert(!math::isNan(mInvProjArtInertiaImplicit));
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::updateInvProjArtInertiaImplicitKinematic(
    const Eigen::Matrix6d& /*_artInertia*/,
    double /*_timeStep*/)
{
  // Do nothing
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::addChildBiasForceTo(
    Eigen::Vector6d& _parentBiasForce,
    const Eigen::Matrix6d& _childArtInertia,
    const Eigen::Vector6d& _childBiasForce,
    const Eigen::Vector6d& _childPartialAcc)
{
  switch (Joint::mAspectProperties.mActuatorType)
  {
    case Joint::FORCE:
    case Joint::PASSIVE:
    case Joint::SERVO:
      addChildBiasForceToDynamic(_parentBiasForce,
                                 _childArtInertia,
                                 _childBiasForce,
                                 _childPartialAcc);
      break;
    case Joint::ACCELERATION:
    case Joint::VELOCITY:
    case Joint::LOCKED:
      addChildBiasForceToKinematic(_parentBiasForce,
                                   _childArtInertia,
                                   _childBiasForce,
                                   _childPartialAcc);
      break;
    default:
      MULTIDOFJOINT_REPORT_UNSUPPORTED_ACTUATOR(addChildBiasForceTo);
      break;
  }
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::addChildBiasForceToDynamic(
    Eigen::Vector6d& _parentBiasForce,
    const Eigen::Matrix6d& _childArtInertia,
    const Eigen::Vector6d& _childBiasForce,
    const Eigen::Vector6d& _childPartialAcc)
{
  // Compute beta
  const Eigen::Vector6d beta
      = _childBiasForce
        + _childArtInertia
          * (_childPartialAcc
             + getRelativeJacobianStatic()*getInvProjArtInertiaImplicit()
               *mTotalForce);

  //    Eigen::Vector6d beta
  //        = _childBiasForce;
  //    beta.noalias() += _childArtInertia * _childPartialAcc;
  //    beta.noalias() += _childArtInertia *  mJacobian * getInvProjArtInertiaImplicit() * mTotalForce;

  // Verification
  assert(!math::isNan(beta));

  // Add child body's bias force to parent body's bias force. Note that mT
  // should be updated.
  _parentBiasForce += math::dAdInvT(this->getRelativeTransform(), beta);
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::addChildBiasForceToKinematic(
    Eigen::Vector6d& _parentBiasForce,
    const Eigen::Matrix6d& _childArtInertia,
    const Eigen::Vector6d& _childBiasForce,
    const Eigen::Vector6d& _childPartialAcc)
{
  // Compute beta
  const Eigen::Vector6d beta
      = _childBiasForce
        + _childArtInertia*(_childPartialAcc
                            + getRelativeJacobianStatic()*getAccelerationsStatic());

  //    Eigen::Vector6d beta
  //        = _childBiasForce;
  //    beta.noalias() += _childArtInertia * _childPartialAcc;
  //    beta.noalias() += _childArtInertia *  mJacobian * getInvProjArtInertiaImplicit() * mTotalForce;

  // Verification
  assert(!math::isNan(beta));

  // Add child body's bias force to parent body's bias force. Note that mT
  // should be updated.
  _parentBiasForce += math::dAdInvT(this->getRelativeTransform(), beta);
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::addChildBiasImpulseTo(
    Eigen::Vector6d& _parentBiasImpulse,
    const Eigen::Matrix6d& _childArtInertia,
    const Eigen::Vector6d& _childBiasImpulse)
{
  switch (Joint::mAspectProperties.mActuatorType)
  {
    case Joint::FORCE:
    case Joint::PASSIVE:
    case Joint::SERVO:
      addChildBiasImpulseToDynamic(_parentBiasImpulse,
                                   _childArtInertia,
                                   _childBiasImpulse);
      break;
    case Joint::ACCELERATION:
    case Joint::VELOCITY:
    case Joint::LOCKED:
      addChildBiasImpulseToKinematic(_parentBiasImpulse,
                                     _childArtInertia,
                                     _childBiasImpulse);
      break;
    default:
      MULTIDOFJOINT_REPORT_UNSUPPORTED_ACTUATOR(addChildBiasImpulseTo);
      break;
  }
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::addChildBiasImpulseToDynamic(
    Eigen::Vector6d& _parentBiasImpulse,
    const Eigen::Matrix6d& _childArtInertia,
    const Eigen::Vector6d& _childBiasImpulse)
{
  // Compute beta
  const Eigen::Vector6d beta
      = _childBiasImpulse
        + _childArtInertia*getRelativeJacobianStatic()
          *getInvProjArtInertia()*mTotalImpulse;

  // Verification
  assert(!math::isNan(beta));

  // Add child body's bias force to parent body's bias force. Note that mT
  // should be updated.
  _parentBiasImpulse += math::dAdInvT(this->getRelativeTransform(), beta);
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::addChildBiasImpulseToKinematic(
    Eigen::Vector6d& _parentBiasImpulse,
    const Eigen::Matrix6d& /*_childArtInertia*/,
    const Eigen::Vector6d& _childBiasImpulse)
{
  // Add child body's bias force to parent body's bias force. Note that mT
  // should be updated.
  _parentBiasImpulse += math::dAdInvT(
        this->getRelativeTransform(), _childBiasImpulse);
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::updateTotalForce(
    const Eigen::Vector6d& _bodyForce,
    double _timeStep)
{
  assert(_timeStep > 0.0);

  switch (Joint::mAspectProperties.mActuatorType)
  {
    case Joint::FORCE:
      this->mAspectState.mForces = this->mAspectState.mCommands;
      updateTotalForceDynamic(_bodyForce, _timeStep);
      break;
    case Joint::PASSIVE:
    case Joint::SERVO:
      this->mAspectState.mForces.setZero();
      updateTotalForceDynamic(_bodyForce, _timeStep);
      break;
    case Joint::ACCELERATION:
      setAccelerationsStatic(this->mAspectState.mCommands);
      updateTotalForceKinematic(_bodyForce, _timeStep);
      break;
    case Joint::VELOCITY:
      setAccelerationsStatic( (this->mAspectState.mCommands - getVelocitiesStatic()) / _timeStep );
      updateTotalForceKinematic(_bodyForce, _timeStep);
      break;
    case Joint::LOCKED:
      setVelocitiesStatic(Eigen::Matrix<double, DOF, 1>::Zero());
      setAccelerationsStatic(Eigen::Matrix<double, DOF, 1>::Zero());
      updateTotalForceKinematic(_bodyForce, _timeStep);
      break;
    default:
      MULTIDOFJOINT_REPORT_UNSUPPORTED_ACTUATOR(updateTotalForce);
      break;
  }
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::updateTotalForceDynamic(
    const Eigen::Vector6d& _bodyForce,
    double _timeStep)
{
  // Spring force
  const Eigen::Matrix<double, DOF, 1> springForce
      = (-Base::mAspectProperties.mSpringStiffnesses).asDiagonal()
        *(getPositionsStatic() - Base::mAspectProperties.mRestPositions
          + getVelocitiesStatic()*_timeStep);

  // Damping force
  const Eigen::Matrix<double, DOF, 1> dampingForce
      = (-Base::mAspectProperties.mDampingCoefficients).asDiagonal()*
        getVelocitiesStatic();

  //
  mTotalForce = this->mAspectState.mForces + springForce + dampingForce;
  mTotalForce.noalias() -= getRelativeJacobianStatic().transpose()*_bodyForce;
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::updateTotalForceKinematic(
    const Eigen::Vector6d& /*_bodyForce*/,
    double /*_timeStep*/)
{
  // Do nothing
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::updateTotalImpulse(
    const Eigen::Vector6d& _bodyImpulse)
{
  switch (Joint::mAspectProperties.mActuatorType)
  {
    case Joint::FORCE:
    case Joint::PASSIVE:
    case Joint::SERVO:
      updateTotalImpulseDynamic(_bodyImpulse);
      break;
    case Joint::ACCELERATION:
    case Joint::VELOCITY:
    case Joint::LOCKED:
      updateTotalImpulseKinematic(_bodyImpulse);
      break;
    default:
      MULTIDOFJOINT_REPORT_UNSUPPORTED_ACTUATOR(updateTotalImpulse);
      break;
  }
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::updateTotalImpulseDynamic(
    const Eigen::Vector6d& _bodyImpulse)
{
  //
  mTotalImpulse = mConstraintImpulses;
  mTotalImpulse.noalias() -= getRelativeJacobianStatic().transpose()*_bodyImpulse;
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::updateTotalImpulseKinematic(
    const Eigen::Vector6d& /*_bodyImpulse*/)
{
  // Do nothing
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::resetTotalImpulses()
{
  mTotalImpulse.setZero();
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::updateAcceleration(
    const Eigen::Matrix6d& _artInertia,
    const Eigen::Vector6d& _spatialAcc)
{
  switch (Joint::mAspectProperties.mActuatorType)
  {
    case Joint::FORCE:
    case Joint::PASSIVE:
    case Joint::SERVO:
      updateAccelerationDynamic(_artInertia, _spatialAcc);
      break;
    case Joint::ACCELERATION:
    case Joint::VELOCITY:
    case Joint::LOCKED:
      updateAccelerationKinematic(_artInertia, _spatialAcc);
      break;
    default:
      MULTIDOFJOINT_REPORT_UNSUPPORTED_ACTUATOR(updateAcceleration);
      break;
  }
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::updateAccelerationDynamic(
    const Eigen::Matrix6d& _artInertia,
    const Eigen::Vector6d& _spatialAcc)
{
  //
  setAccelerationsStatic( getInvProjArtInertiaImplicit()
        * (mTotalForce - getRelativeJacobianStatic().transpose()
           *_artInertia*math::AdInvT(this->getRelativeTransform(), _spatialAcc)) );

  // Verification
  assert(!math::isNan(getAccelerationsStatic()));
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::updateAccelerationKinematic(
    const Eigen::Matrix6d& /*_artInertia*/,
    const Eigen::Vector6d& /*_spatialAcc*/)
{
  // Do nothing
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::updateVelocityChange(
    const Eigen::Matrix6d& _artInertia,
    const Eigen::Vector6d& _velocityChange)
{
  switch (Joint::mAspectProperties.mActuatorType)
  {
    case Joint::FORCE:
    case Joint::PASSIVE:
    case Joint::SERVO:
      updateVelocityChangeDynamic(_artInertia, _velocityChange);
      break;
    case Joint::ACCELERATION:
    case Joint::VELOCITY:
    case Joint::LOCKED:
      updateVelocityChangeKinematic(_artInertia, _velocityChange);
      break;
    default:
      MULTIDOFJOINT_REPORT_UNSUPPORTED_ACTUATOR(updateVelocityChange);
      break;
  }
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::updateVelocityChangeDynamic(
    const Eigen::Matrix6d& _artInertia,
    const Eigen::Vector6d& _velocityChange)
{
  //
  mVelocityChanges
      = getInvProjArtInertia()
      * (mTotalImpulse - getRelativeJacobianStatic().transpose()
         *_artInertia*math::AdInvT(this->getRelativeTransform(), _velocityChange));

  // Verification
  assert(!math::isNan(mVelocityChanges));
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::updateVelocityChangeKinematic(
    const Eigen::Matrix6d& /*_artInertia*/,
    const Eigen::Vector6d& /*_velocityChange*/)
{
  // Do nothing
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::updateForceID(const Eigen::Vector6d& _bodyForce,
                                       double _timeStep,
                                       bool _withDampingForces,
                                       bool _withSpringForces)
{
  this->mAspectState.mForces = getRelativeJacobianStatic().transpose()*_bodyForce;

  // Damping force
  if (_withDampingForces)
  {
    const Eigen::Matrix<double, DOF, 1> dampingForces
        = (-Base::mAspectProperties.mDampingCoefficients).asDiagonal()
          *getVelocitiesStatic();
    this->mAspectState.mForces -= dampingForces;
  }

  // Spring force
  if (_withSpringForces)
  {
    const Eigen::Matrix<double, DOF, 1> springForces
        = (-Base::mAspectProperties.mSpringStiffnesses).asDiagonal()
          *(getPositionsStatic() - Base::mAspectProperties.mRestPositions
            + getVelocitiesStatic()*_timeStep);
    this->mAspectState.mForces -= springForces;
  }
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::updateForceFD(const Eigen::Vector6d& _bodyForce,
                                       double _timeStep,
                                       bool _withDampingForces,
                                       bool _withSpringForces)
{
  switch (Joint::mAspectProperties.mActuatorType)
  {
    case Joint::FORCE:
    case Joint::PASSIVE:
    case Joint::SERVO:
      break;
    case Joint::ACCELERATION:
    case Joint::VELOCITY:
    case Joint::LOCKED:
      updateForceID(_bodyForce, _timeStep, _withDampingForces,
                    _withSpringForces);
      break;
    default:
      MULTIDOFJOINT_REPORT_UNSUPPORTED_ACTUATOR(updateForceFD);
      break;
  }
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::updateImpulseID(const Eigen::Vector6d& _bodyImpulse)
{
  mImpulses = getRelativeJacobianStatic().transpose()*_bodyImpulse;
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::updateImpulseFD(const Eigen::Vector6d& _bodyImpulse)
{
  switch (Joint::mAspectProperties.mActuatorType)
  {
    case Joint::FORCE:
    case Joint::PASSIVE:
    case Joint::SERVO:
      break;
    case Joint::ACCELERATION:
    case Joint::VELOCITY:
    case Joint::LOCKED:
      updateImpulseID(_bodyImpulse);
      break;
    default:
      MULTIDOFJOINT_REPORT_UNSUPPORTED_ACTUATOR(updateImpulseFD);
      break;
  }
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::updateConstrainedTerms(double _timeStep)
{
  switch (Joint::mAspectProperties.mActuatorType)
  {
    case Joint::FORCE:
    case Joint::PASSIVE:
    case Joint::SERVO:
      updateConstrainedTermsDynamic(_timeStep);
      break;
    case Joint::ACCELERATION:
    case Joint::VELOCITY:
    case Joint::LOCKED:
      updateConstrainedTermsKinematic(_timeStep);
      break;
    default:
      MULTIDOFJOINT_REPORT_UNSUPPORTED_ACTUATOR(updateConstrainedTerms);
      break;
  }
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::updateConstrainedTermsDynamic(double _timeStep)
{
  const double invTimeStep = 1.0 / _timeStep;

  setVelocitiesStatic(getVelocitiesStatic() + mVelocityChanges);
  setAccelerationsStatic(getAccelerationsStatic()
                         + mVelocityChanges*invTimeStep);
  this->mAspectState.mForces.noalias() += mImpulses*invTimeStep;
  // Note: As long as this is only called from BodyNode::updateConstrainedTerms
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::updateConstrainedTermsKinematic(
    double _timeStep)
{
  this->mAspectState.mForces.noalias() += mImpulses / _timeStep;
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::addChildBiasForceForInvMassMatrix(
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
  _parentBiasForce += math::dAdInvT(this->getRelativeTransform(), beta);
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::addChildBiasForceForInvAugMassMatrix(
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
  _parentBiasForce += math::dAdInvT(this->getRelativeTransform(), beta);
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::updateTotalForceForInvMassMatrix(
    const Eigen::Vector6d& _bodyForce)
{
  // Compute alpha
  mInvM_a = this->mAspectState.mForces;
  mInvM_a.noalias() -= getRelativeJacobianStatic().transpose() * _bodyForce;
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::getInvMassMatrixSegment(
    Eigen::MatrixXd& _invMassMat,
    const std::size_t _col,
    const Eigen::Matrix6d& _artInertia,
    const Eigen::Vector6d& _spatialAcc)
{
  //
  mInvMassMatrixSegment
      = getInvProjArtInertia()
      * (mInvM_a - getRelativeJacobianStatic().transpose()
         * _artInertia * math::AdInvT(this->getRelativeTransform(), _spatialAcc));

  // Verification
  assert(!math::isNan(mInvMassMatrixSegment));

  // Index
  std::size_t iStart = mDofs[0]->mIndexInTree;

  // Assign
  _invMassMat.block<DOF, 1>(iStart, _col) = mInvMassMatrixSegment;
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::getInvAugMassMatrixSegment(
    Eigen::MatrixXd& _invMassMat,
    const std::size_t _col,
    const Eigen::Matrix6d& _artInertia,
    const Eigen::Vector6d& _spatialAcc)
{
  //
  mInvMassMatrixSegment
      = getInvProjArtInertiaImplicit()
      * (mInvM_a - getRelativeJacobianStatic().transpose()
         * _artInertia * math::AdInvT(this->getRelativeTransform(), _spatialAcc));

  // Verification
  assert(!math::isNan(mInvMassMatrixSegment));

  // Index
  std::size_t iStart = mDofs[0]->mIndexInTree;

  // Assign
  _invMassMat.block<DOF, 1>(iStart, _col) = mInvMassMatrixSegment;
}

//==============================================================================
template <std::size_t DOF>
void MultiDofJoint<DOF>::addInvMassMatrixSegmentTo(Eigen::Vector6d& _acc)
{
  //
  _acc += getRelativeJacobianStatic() * mInvMassMatrixSegment;
}

//==============================================================================
template <std::size_t DOF>
Eigen::VectorXd MultiDofJoint<DOF>::getSpatialToGeneralized(
    const Eigen::Vector6d& _spatial)
{
  return getRelativeJacobianStatic().transpose() * _spatial;
}

} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_DETAIL_MULTIDOFJOINT_HPP_
