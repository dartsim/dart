/*
 * Copyright (c) 2015-2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2015-2016, Humanoid Lab, Georgia Tech Research Corporation
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

#ifndef DART_DYNAMICS_DETAIL_GenericJoint_HPP_
#define DART_DYNAMICS_DETAIL_GenericJoint_HPP_

#include "dart/dynamics/GenericJoint.hpp"

#include "dart/config.hpp"
#include "dart/math/Helpers.hpp"
#include "dart/math/ConfigurationSpace.hpp"
#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/DegreeOfFreedom.hpp"

#define GenericJoint_REPORT_DIM_MISMATCH( func, arg )\
  {\
    dterr << "[GenericJoint::" #func "] Mismatch beteween size of "\
          << #arg " [" << arg .size() << "] and the number of "\
          << "DOFs [" << getNumDofs() << "] for Joint named ["\
          << this->getName() << "].\n";\
    assert(false);\
  }

#define GenericJoint_REPORT_OUT_OF_RANGE( func, index )\
  {\
    dterr << "[GenericJoint::" << #func << "] The index [" << index\
          << "] is out of range for Joint named [" << this->getName()\
          << "] which has " << this->getNumDofs() << " DOFs.\n";\
    assert(false);\
  }

#define GenericJoint_REPORT_UNSUPPORTED_ACTUATOR( func )\
  {\
    dterr << "[GenericJoint::" # func "] Unsupported actuator type ("\
          << Joint::mAspectProperties.mActuatorType << ") for Joint ["\
          << this->getName() << "].\n";\
    assert(false);\
  }

#define GenericJoint_SET_IF_DIFFERENT( mField, value )\
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
template <class ConfigSpaceT>
constexpr size_t GenericJoint<ConfigSpaceT>::NumDofs;

//==============================================================================
template <class ConfigSpaceT>
GenericJoint<ConfigSpaceT>::~GenericJoint()
{
  for (auto i = 0u; i < NumDofs; ++i)
    delete mDofs[i];
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::setProperties(const Properties& properties)
{
  Joint::setProperties(static_cast<const Joint::Properties&>(properties));
  setProperties(static_cast<const UniqueProperties&>(properties));
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::setProperties(
    const UniqueProperties& properties)
{
  setAspectProperties(properties);
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::setAspectState(const AspectState& state)
{
  setCommands(state.mCommands);
  setPositionsStatic(state.mPositions);
  setVelocitiesStatic(state.mVelocities);
  setAccelerationsStatic(state.mAccelerations);
  setForces(state.mForces);
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::setAspectProperties(
    const AspectProperties& properties)
{
  for (auto i = 0u; i < NumDofs; ++i)
  {
    setDofName(i, properties.mDofNames[i], properties.mPreserveDofNames[i]);
    setPositionLowerLimit    (i, properties.mPositionLowerLimits[i]    );
    setPositionUpperLimit    (i, properties.mPositionUpperLimits[i]    );
    setInitialPosition       (i, properties.mInitialPositions[i]       );
    setVelocityLowerLimit    (i, properties.mVelocityLowerLimits[i]    );
    setVelocityUpperLimit    (i, properties.mVelocityUpperLimits[i]    );
    setInitialVelocity       (i, properties.mInitialVelocities[i]      );
    setAccelerationLowerLimit(i, properties.mAccelerationLowerLimits[i]);
    setAccelerationUpperLimit(i, properties.mAccelerationUpperLimits[i]);
    setForceLowerLimit       (i, properties.mForceLowerLimits[i]       );
    setForceUpperLimit       (i, properties.mForceUpperLimits[i]       );
    setSpringStiffness       (i, properties.mSpringStiffnesses[i]      );
    setRestPosition          (i, properties.mRestPositions[i]          );
    setDampingCoefficient    (i, properties.mDampingCoefficients[i]    );
    setCoulombFriction       (i, properties.mFrictions[i]              );
  }
}

//==============================================================================
template <class ConfigSpaceT>
typename GenericJoint<ConfigSpaceT>::Properties
GenericJoint<ConfigSpaceT>::getGenericJointProperties() const
{
  return GenericJoint<ConfigSpaceT>::Properties(
        Joint::mAspectProperties, Base::mAspectProperties);
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::copy(
    const GenericJoint<ConfigSpaceT>& other)
{
  if (this == &other)
    return;

  setProperties(other.getGenericJointProperties());
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::copy(
    const GenericJoint<ConfigSpaceT>* other)
{
  if (nullptr == other)
    return;

  copy(*other);
}

//==============================================================================
template <class ConfigSpaceT>
GenericJoint<ConfigSpaceT>&
GenericJoint<ConfigSpaceT>::operator=(
    const GenericJoint<ConfigSpaceT>& other)
{
  copy(other);
  return *this;
}

//==============================================================================
template <class ConfigSpaceT>
DegreeOfFreedom* GenericJoint<ConfigSpaceT>::getDof(size_t index)
{
  if (index < NumDofs)
    return mDofs[index];

  GenericJoint_REPORT_OUT_OF_RANGE(getDof, index);

  return nullptr;
}

//==============================================================================
template <class ConfigSpaceT>
const DegreeOfFreedom* GenericJoint<ConfigSpaceT>::getDof(size_t index) const
{
  if (index < NumDofs)
    return mDofs[index];

  GenericJoint_REPORT_OUT_OF_RANGE(getDof, index);

  return nullptr;
}

//==============================================================================
template <class ConfigSpaceT>
size_t GenericJoint<ConfigSpaceT>::getNumDofs() const
{
  return NumDofs;
}

//==============================================================================
template <class ConfigSpaceT>
const std::string& GenericJoint<ConfigSpaceT>::setDofName(
    size_t index,
    const std::string& name,
    bool preserveName)
{
  if (NumDofs <= index)
  {
    dterr << "[GenericJoint::setDofName] Attempting to set the name of DOF "
          << "index " << index << ", which is out of bounds for the Joint ["
          << this->getName()
          << "]. We will set the name of DOF index 0 instead.\n";
    assert(false);
    index = 0u;
  }

  preserveDofName(index, preserveName);

  std::string& dofName = Base::mAspectProperties.mDofNames[index];

  if (name == dofName)
    return dofName;

  const SkeletonPtr& skel
      = this->mChildBodyNode ? this->mChildBodyNode->getSkeleton() : nullptr;
  if (skel)
    dofName = skel->mNameMgrForDofs.changeObjectName(mDofs[index], name);
  else
    dofName = name;

  return dofName;
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::preserveDofName(size_t index, bool preserve)
{
  if (NumDofs <= index)
  {
    GenericJoint_REPORT_OUT_OF_RANGE(preserveDofName, index);
    return;
  }

  GenericJoint_SET_IF_DIFFERENT( mPreserveDofNames[index], preserve );
}

//==============================================================================
template <class ConfigSpaceT>
bool GenericJoint<ConfigSpaceT>::isDofNamePreserved(size_t index) const
{
  if(NumDofs <= index)
  {
    GenericJoint_REPORT_OUT_OF_RANGE(isDofNamePreserved, index);
    index = 0;
  }

  return Base::mAspectProperties.mPreserveDofNames[index];
}

//==============================================================================
template <class ConfigSpaceT>
const std::string& GenericJoint<ConfigSpaceT>::getDofName(size_t index) const
{
  if(NumDofs <= index)
  {
    dterr << "[GenericJoint::getDofName] Requested name of DOF index ["
          << index << "] in Joint [" << this->getName() << "], but that is "
          << "out of bounds (max " << NumDofs - 1
          << "). Returning name of DOF 0.\n";
    assert(false);
    return Base::mAspectProperties.mDofNames[0];
  }

  return Base::mAspectProperties.mDofNames[index];
}

//==============================================================================
template <class ConfigSpaceT>
size_t
GenericJoint<ConfigSpaceT>::getIndexInSkeleton(size_t index) const
{
  if (index >= getNumDofs())
  {
    GenericJoint_REPORT_OUT_OF_RANGE(getIndexInSkeleton, index);
    return 0;
  }

  return mDofs[index]->mIndexInSkeleton;
}

//==============================================================================
template <class ConfigSpaceT>
size_t GenericJoint<ConfigSpaceT>::getIndexInTree(size_t index) const
{
  if (index >= getNumDofs())
  {
    GenericJoint_REPORT_OUT_OF_RANGE(getIndexInTree, index);
    return 0;
  }

  return mDofs[index]->mIndexInTree;
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::setCommand(size_t index, double command)
{
  if (index >= getNumDofs())
    GenericJoint_REPORT_OUT_OF_RANGE(setCommand, index);

  switch (Joint::mAspectProperties.mActuatorType)
  {
    case Joint::FORCE:
      this->mAspectState.mCommands[index] = math::clip(command,
          Base::mAspectProperties.mForceLowerLimits[index],
          Base::mAspectProperties.mForceUpperLimits[index]);
      break;
    case Joint::PASSIVE:
      if (0.0 != command)
      {
        dtwarn << "[GenericJoint::setCommand] Attempting to set a non-zero ("
               << command << ") command for a PASSIVE joint ["
               << this->getName() << "].\n";
      }
      this->mAspectState.mCommands[index] = command;
      break;
    case Joint::SERVO:
      this->mAspectState.mCommands[index] = math::clip(command,
          Base::mAspectProperties.mVelocityLowerLimits[index],
          Base::mAspectProperties.mVelocityUpperLimits[index]);
      break;
    case Joint::ACCELERATION:
      this->mAspectState.mCommands[index] = math::clip(command,
          Base::mAspectProperties.mAccelerationLowerLimits[index],
          Base::mAspectProperties.mAccelerationUpperLimits[index]);
      break;
    case Joint::VELOCITY:
      this->mAspectState.mCommands[index] = math::clip(command,
          Base::mAspectProperties.mVelocityLowerLimits[index],
          Base::mAspectProperties.mVelocityUpperLimits[index]);
      // TODO: This possibly makes the acceleration to exceed the limits.
      break;
    case Joint::LOCKED:
      if (0.0 != command)
      {
        dtwarn << "[GenericJoint::setCommand] Attempting to set a non-zero ("
               << command << ") command for a LOCKED joint [" << this->getName()
               << "].\n";
      }
      this->mAspectState.mCommands[index] = command;
      break;
    default:
      assert(false);
      break;
  }
}

//==============================================================================
template <class ConfigSpaceT>
double GenericJoint<ConfigSpaceT>::getCommand(size_t index) const
{
  if (index >= getNumDofs())
  {
    GenericJoint_REPORT_OUT_OF_RANGE(getCommand, index);
    return 0.0;
  }

  return this->mAspectState.mCommands[index];
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::setCommands(
    const Eigen::VectorXd& commands)
{
  if (static_cast<size_t>(commands.size()) != getNumDofs())
  {
    GenericJoint_REPORT_DIM_MISMATCH(setCommands, commands);
    return;
  }

  switch (Joint::mAspectProperties.mActuatorType)
  {
    case Joint::FORCE:
      this->mAspectState.mCommands = math::clip(commands,
          Base::mAspectProperties.mForceLowerLimits,
          Base::mAspectProperties.mForceUpperLimits);
      break;
    case Joint::PASSIVE:
      if(Vector::Zero() != commands)
      {
        dtwarn << "[GenericJoint::setCommands] Attempting to set a non-zero ("
               << commands.transpose() << ") command for a PASSIVE joint ["
               << this->getName() << "].\n";
      }
      this->mAspectState.mCommands = commands;
      break;
    case Joint::SERVO:
      this->mAspectState.mCommands = math::clip(commands,
          Base::mAspectProperties.mVelocityLowerLimits,
          Base::mAspectProperties.mVelocityUpperLimits);
      break;
    case Joint::ACCELERATION:
      this->mAspectState.mCommands = math::clip(commands,
          Base::mAspectProperties.mAccelerationLowerLimits,
          Base::mAspectProperties.mAccelerationUpperLimits);
      break;
    case Joint::VELOCITY:
      this->mAspectState.mCommands = math::clip(commands,
          Base::mAspectProperties.mVelocityLowerLimits,
          Base::mAspectProperties.mVelocityUpperLimits);
      // TODO: This possibly makes the acceleration to exceed the limits.
      break;
    case Joint::LOCKED:
      if(Vector::Zero() != commands)
      {
        dtwarn << "[GenericJoint::setCommands] Attempting to set a non-zero ("
               << commands.transpose() << ") command for a LOCKED joint ["
               << this->getName() << "].\n";
      }
      this->mAspectState.mCommands = commands;
      break;
    default:
      assert(false);
      break;
  }
}

//==============================================================================
template <class ConfigSpaceT>
Eigen::VectorXd GenericJoint<ConfigSpaceT>::getCommands() const
{
  return this->mAspectState.mCommands;
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::resetCommands()
{
  this->mAspectState.mCommands.setZero();
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::setPosition(size_t index, double position)
{
  if (index >= getNumDofs())
  {
    GenericJoint_REPORT_OUT_OF_RANGE(setPosition, index);
    return;
  }

  if (this->mAspectState.mPositions[index] == position)
    return;
  // TODO(JS): Above code should be changed something like:
//  if (ConfigSpaceT::getEuclideanPoint(mPositions, index) == position)
//    return;

  // Note: It would not make much sense to use setPositionsStatic() here
  this->mAspectState.mPositions[index] = position;
  this->notifyPositionUpdate();
}

//==============================================================================
template <class ConfigSpaceT>
double GenericJoint<ConfigSpaceT>::getPosition(size_t index) const
{
  if (index >= getNumDofs())
  {
    GenericJoint_REPORT_OUT_OF_RANGE(getPosition, index);
    return 0.0;
  }

  return getPositionsStatic()[index];
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::setPositions(
    const Eigen::VectorXd& positions)
{
  if (static_cast<size_t>(positions.size()) != getNumDofs())
  {
    GenericJoint_REPORT_DIM_MISMATCH(setPositions, positions);
    return;
  }

  setPositionsStatic(positions);
}

//==============================================================================
template <class ConfigSpaceT>
Eigen::VectorXd GenericJoint<ConfigSpaceT>::getPositions() const
{
  return getPositionsStatic();
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::setPositionLowerLimit(
    size_t index, double position)
{
  if (index >= getNumDofs())
  {
    GenericJoint_REPORT_OUT_OF_RANGE(setPositionLowerLimit, index);
    return;
  }

  GenericJoint_SET_IF_DIFFERENT( mPositionLowerLimits[index], position );
}

//==============================================================================
template <class ConfigSpaceT>
double GenericJoint<ConfigSpaceT>::getPositionLowerLimit(size_t index) const
{
  if (index >= getNumDofs())
  {
    GenericJoint_REPORT_OUT_OF_RANGE(getPositionLowerLimit, index);
    return 0.0;
  }

  return Base::mAspectProperties.mPositionLowerLimits[index];
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::setPositionUpperLimit(size_t index,
                                                       double position)
{
  if (index >= getNumDofs())
  {
    GenericJoint_REPORT_OUT_OF_RANGE(setPositionUpperLimit, index);
    return;
  }

  GenericJoint_SET_IF_DIFFERENT( mPositionUpperLimits[index], position );
}

//==============================================================================
template <class ConfigSpaceT>
double GenericJoint<ConfigSpaceT>::getPositionUpperLimit(size_t index) const
{
  if (index >= getNumDofs())
  {
    GenericJoint_REPORT_OUT_OF_RANGE(getPositionUpperLimit, index);
    return 0.0;
  }

  return Base::mAspectProperties.mPositionUpperLimits[index];
}

//==============================================================================
template <class ConfigSpaceT>
bool GenericJoint<ConfigSpaceT>::hasPositionLimit(size_t index) const
{
  if (index >= getNumDofs())
  {
    GenericJoint_REPORT_OUT_OF_RANGE(hasPositionLimit, index);
    return true;
  }

  return std::isfinite(Base::mAspectProperties.mPositionUpperLimits[index])
      || std::isfinite(Base::mAspectProperties.mPositionLowerLimits[index]);
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::resetPosition(size_t index)
{
  if (index >= getNumDofs())
  {
    GenericJoint_REPORT_OUT_OF_RANGE(resetPosition, index);
    return;
  }

  setPosition(index, Base::mAspectProperties.mInitialPositions[index]);
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::resetPositions()
{
  setPositionsStatic(Base::mAspectProperties.mInitialPositions);
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::setInitialPosition(
    size_t index, double initial)
{
  if (index >= getNumDofs())
  {
    GenericJoint_REPORT_OUT_OF_RANGE(setInitialPosition, index);
    return;
  }

  GenericJoint_SET_IF_DIFFERENT( mInitialPositions[index], initial );
}

//==============================================================================
template <class ConfigSpaceT>
double GenericJoint<ConfigSpaceT>::getInitialPosition(size_t index) const
{
  if (index >= getNumDofs())
  {
    GenericJoint_REPORT_OUT_OF_RANGE(getInitialPosition, index);
    return 0.0;
  }

  return Base::mAspectProperties.mInitialPositions[index];
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::setInitialPositions(
    const Eigen::VectorXd& initial)
{
  if ( static_cast<size_t>(initial.size()) != getNumDofs() )
  {
    GenericJoint_REPORT_DIM_MISMATCH(setInitialPositions, initial);
    return;
  }

  GenericJoint_SET_IF_DIFFERENT( mInitialPositions, initial);
}

//==============================================================================
template <class ConfigSpaceT>
Eigen::VectorXd GenericJoint<ConfigSpaceT>::getInitialPositions() const
{
  return Base::mAspectProperties.mInitialPositions;
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::setPositionsStatic(const Vector& positions)
{
  if (this->mAspectState.mPositions == positions)
    return;

  this->mAspectState.mPositions = positions;
  this->notifyPositionUpdate();
}

//==============================================================================
template <class ConfigSpaceT>
const typename GenericJoint<ConfigSpaceT>::Vector&
GenericJoint<ConfigSpaceT>::getPositionsStatic() const
{
  return this->mAspectState.mPositions;
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::setVelocitiesStatic(
    const Vector& velocities)
{
  if (this->mAspectState.mVelocities == velocities)
    return;

  this->mAspectState.mVelocities = velocities;
  this->notifyVelocityUpdate();
}

//==============================================================================
template <class ConfigSpaceT>
const typename GenericJoint<ConfigSpaceT>::Vector&
GenericJoint<ConfigSpaceT>::getVelocitiesStatic() const
{
  return this->mAspectState.mVelocities;
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::setAccelerationsStatic(const Vector& accels)
{
  if (this->mAspectState.mAccelerations == accels)
    return;

  this->mAspectState.mAccelerations = accels;
  this->notifyAccelerationUpdate();
}

//==============================================================================
template <class ConfigSpaceT>
const typename GenericJoint<ConfigSpaceT>::Vector&
GenericJoint<ConfigSpaceT>::getAccelerationsStatic() const
{
  return this->mAspectState.mAccelerations;
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::setVelocity(size_t index, double velocity)
{
  if (index >= getNumDofs())
  {
    GenericJoint_REPORT_OUT_OF_RANGE(setVelocity, index);
    return;
  }

  if (this->mAspectState.mVelocities[index] == velocity)
    return;

  // Note: It would not make much sense to use setVelocitiesStatic() here
  this->mAspectState.mVelocities[index] = velocity;
  this->notifyVelocityUpdate();

  if (Joint::mAspectProperties.mActuatorType == Joint::VELOCITY)
    this->mAspectState.mCommands[index] = this->getVelocitiesStatic()[index];
}

//==============================================================================
template <class ConfigSpaceT>
double GenericJoint<ConfigSpaceT>::getVelocity(size_t index) const
{
  if (index >= getNumDofs())
  {
    GenericJoint_REPORT_OUT_OF_RANGE(getVelocity, index);
    return 0.0;
  }

  return getVelocitiesStatic()[index];
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::setVelocities(
    const Eigen::VectorXd& velocities)
{
  if (static_cast<size_t>(velocities.size()) != getNumDofs())
  {
    GenericJoint_REPORT_DIM_MISMATCH(setVelocities, velocities);
    return;
  }

  setVelocitiesStatic(velocities);

  if (Joint::mAspectProperties.mActuatorType == Joint::VELOCITY)
    this->mAspectState.mCommands = this->getVelocitiesStatic();
}

//==============================================================================
template <class ConfigSpaceT>
Eigen::VectorXd GenericJoint<ConfigSpaceT>::getVelocities() const
{
  return getVelocitiesStatic();
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::setVelocityLowerLimit(size_t index,
                                                          double velocity)
{
  if (index >= getNumDofs())
  {
    GenericJoint_REPORT_OUT_OF_RANGE(setVelocityLowerLimit, index);
    return;
  }

  GenericJoint_SET_IF_DIFFERENT( mVelocityLowerLimits[index], velocity );
}

//==============================================================================
template <class ConfigSpaceT>
double GenericJoint<ConfigSpaceT>::getVelocityLowerLimit(size_t index) const
{
  if (index >= getNumDofs())
  {
    GenericJoint_REPORT_OUT_OF_RANGE(getVelocityLowerLimit, index);
    return 0.0;
  }

  return Base::mAspectProperties.mVelocityLowerLimits[index];
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::setVelocityUpperLimit(size_t index,
                                                          double velocity)
{
  if (index >= getNumDofs())
  {
    GenericJoint_REPORT_OUT_OF_RANGE(setVelocityUpperLimit, index);
    return;
  }

  GenericJoint_SET_IF_DIFFERENT( mVelocityUpperLimits[index], velocity );
}

//==============================================================================
template <class ConfigSpaceT>
double GenericJoint<ConfigSpaceT>::getVelocityUpperLimit(
    size_t index) const
{
  if (index >= getNumDofs())
  {
    GenericJoint_REPORT_OUT_OF_RANGE(getVelocityUpperLimit, index);
    return 0.0;
  }

  return Base::mAspectProperties.mVelocityUpperLimits[index];
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::resetVelocity(size_t index)
{
  if (index >= getNumDofs())
  {
    GenericJoint_REPORT_OUT_OF_RANGE(resetVelocity, index);
    return;
  }

  setVelocity(index, Base::mAspectProperties.mInitialVelocities[index]);
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::resetVelocities()
{
  setVelocitiesStatic(Base::mAspectProperties.mInitialVelocities);
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::setInitialVelocity(size_t index,
                                                       double initial)
{
  if (index >= getNumDofs())
  {
    GenericJoint_REPORT_OUT_OF_RANGE(setInitialVelocity, index);
    return;
  }

  GenericJoint_SET_IF_DIFFERENT( mInitialVelocities[index], initial );
}

//==============================================================================
template <class ConfigSpaceT>
double GenericJoint<ConfigSpaceT>::getInitialVelocity(size_t index) const
{
  if (index >= getNumDofs())
  {
    GenericJoint_REPORT_OUT_OF_RANGE(getInitialVelocity, index);
    return 0.0;
  }

  return Base::mAspectProperties.mInitialVelocities[index];
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::setInitialVelocities(
    const Eigen::VectorXd& initial)
{
  if ( static_cast<size_t>(initial.size()) != getNumDofs() )
  {
    GenericJoint_REPORT_DIM_MISMATCH( setInitialVelocities, initial );
    return;
  }

  GenericJoint_SET_IF_DIFFERENT( mInitialVelocities, initial );
}

//==============================================================================
template <class ConfigSpaceT>
Eigen::VectorXd GenericJoint<ConfigSpaceT>::getInitialVelocities() const
{
  return Base::mAspectProperties.mInitialVelocities;
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::setAcceleration(
    size_t index, double acceleration)
{
  if (index >= getNumDofs())
  {
    GenericJoint_REPORT_OUT_OF_RANGE( setAcceleration, index );
    return;
  }

  if(this->mAspectState.mAccelerations[index] == acceleration)
    return;

  // Note: It would not make much sense to use setAccelerationsStatic() here
  this->mAspectState.mAccelerations[index] = acceleration;
  this->notifyAccelerationUpdate();

  if (Joint::mAspectProperties.mActuatorType == Joint::ACCELERATION)
    this->mAspectState.mCommands[index] = this->getAccelerationsStatic()[index];
}

//==============================================================================
template <class ConfigSpaceT>
double GenericJoint<ConfigSpaceT>::getAcceleration(size_t index) const
{
  if (index >= getNumDofs())
  {
    GenericJoint_REPORT_OUT_OF_RANGE(getAcceleration, index);
    return 0.0;
  }

  return getAccelerationsStatic()[index];
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::setAccelerations(
    const Eigen::VectorXd& accelerations)
{
  if (static_cast<size_t>(accelerations.size()) != getNumDofs())
  {
    GenericJoint_REPORT_DIM_MISMATCH( setAccelerations, accelerations );
    return;
  }

  setAccelerationsStatic(accelerations);

  if (Joint::mAspectProperties.mActuatorType == Joint::ACCELERATION)
    this->mAspectState.mCommands = this->getAccelerationsStatic();
}

//==============================================================================
template <class ConfigSpaceT>
Eigen::VectorXd GenericJoint<ConfigSpaceT>::getAccelerations() const
{
  return getAccelerationsStatic();
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::setAccelerationLowerLimit(
    size_t index, double acceleration)
{
  if (index >= getNumDofs())
  {
    GenericJoint_REPORT_OUT_OF_RANGE(setAccelerationLowerLimit, index);
    return;
  }

  GenericJoint_SET_IF_DIFFERENT( mAccelerationLowerLimits[index],
                                  acceleration );
}

//==============================================================================
template <class ConfigSpaceT>
double GenericJoint<ConfigSpaceT>::getAccelerationLowerLimit(
    size_t index) const
{
  if (index >= getNumDofs())
  {
    GenericJoint_REPORT_OUT_OF_RANGE(getAccelerationLowerLimit, index);
    return 0.0;
  }

  return Base::mAspectProperties.mAccelerationLowerLimits[index];
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::setAccelerationUpperLimit(
    size_t index, double acceleration)
{
  if (index >= getNumDofs())
  {
    GenericJoint_REPORT_OUT_OF_RANGE(setAccelerationUpperLimit, index)
    return;
  }

  GenericJoint_SET_IF_DIFFERENT( mAccelerationUpperLimits[index],
                                  acceleration);
}

//==============================================================================
template <class ConfigSpaceT>
double GenericJoint<ConfigSpaceT>::getAccelerationUpperLimit(
    size_t index) const
{
  if (index >= getNumDofs())
  {
    GenericJoint_REPORT_OUT_OF_RANGE(getAccelerationUpperLimit, index);
    return 0.0;
  }

  return Base::mAspectProperties.mAccelerationUpperLimits[index];
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::resetAccelerations()
{
  setAccelerationsStatic(Vector::Zero());
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::setForce(size_t index, double force)
{
  if (index >= getNumDofs())
  {
    GenericJoint_REPORT_OUT_OF_RANGE(setForce, index);
    return;
  }

  this->mAspectState.mForces[index] = force;

  if (Joint::mAspectProperties.mActuatorType == Joint::FORCE)
    this->mAspectState.mCommands[index] = this->mAspectState.mForces[index];
}

//==============================================================================
template <class ConfigSpaceT>
double GenericJoint<ConfigSpaceT>::getForce(size_t index)
{
  if (index >= getNumDofs())
  {
    GenericJoint_REPORT_OUT_OF_RANGE(getForce, index);
    return 0.0;
  }

  return this->mAspectState.mForces[index];
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::setForces(const Eigen::VectorXd& forces)
{
  if (static_cast<size_t>(forces.size()) != getNumDofs())
  {
    GenericJoint_REPORT_DIM_MISMATCH(setForces, forces);
    return;
  }

  this->mAspectState.mForces = forces;

  if (Joint::mAspectProperties.mActuatorType == Joint::FORCE)
    this->mAspectState.mCommands = this->mAspectState.mForces;
}

//==============================================================================
template <class ConfigSpaceT>
Eigen::VectorXd GenericJoint<ConfigSpaceT>::getForces() const
{
  return this->mAspectState.mForces;
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::setForceLowerLimit(size_t index, double force)
{
  if (index >= getNumDofs())
  {
    GenericJoint_REPORT_OUT_OF_RANGE(setForceLowerLimit, index);
    return;
  }

  GenericJoint_SET_IF_DIFFERENT( mForceLowerLimits[index], force );
}

//==============================================================================
template <class ConfigSpaceT>
double GenericJoint<ConfigSpaceT>::getForceLowerLimit(
    size_t index) const
{
  if (index >= getNumDofs())
  {
    GenericJoint_REPORT_OUT_OF_RANGE(getForceLowerLimit, index);
    return 0.0;
  }

  return Base::mAspectProperties.mForceLowerLimits[index];
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::setForceUpperLimit(size_t index, double force)
{
  if (index >= getNumDofs())
  {
    GenericJoint_REPORT_OUT_OF_RANGE(setForceUpperLimit, index);
    return;
  }

  GenericJoint_SET_IF_DIFFERENT( mForceUpperLimits[index], force );
}

//==============================================================================
template <class ConfigSpaceT>
double GenericJoint<ConfigSpaceT>::getForceUpperLimit(
    size_t index) const
{
  if (index >= getNumDofs())
  {
    GenericJoint_REPORT_OUT_OF_RANGE(getForceUpperLimit, index);
    return 0.0;
  }

  return Base::mAspectProperties.mForceUpperLimits[index];
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::resetForces()
{
  this->mAspectState.mForces.setZero();

  if (Joint::mAspectProperties.mActuatorType == Joint::FORCE)
    this->mAspectState.mCommands = this->mAspectState.mForces;
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::setVelocityChange(
    size_t index, double velocityChange)
{
  if (index >= getNumDofs())
  {
    GenericJoint_REPORT_OUT_OF_RANGE(setVelocityChange, index);
    return;
  }

  mVelocityChanges[index] = velocityChange;
}

//==============================================================================
template <class ConfigSpaceT>
double GenericJoint<ConfigSpaceT>::getVelocityChange(size_t index) const
{
  if (index >= getNumDofs())
  {
    GenericJoint_REPORT_OUT_OF_RANGE(getVelocityChange, index);
    return 0.0;
  }

  return mVelocityChanges[index];
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::resetVelocityChanges()
{
  mVelocityChanges.setZero();
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::setConstraintImpulse(size_t index,
                                                         double impulse)
{
  if (index >= getNumDofs())
  {
    GenericJoint_REPORT_OUT_OF_RANGE(setConstraintImpulse, index);
    return;
  }

  mConstraintImpulses[index] = impulse;
}

//==============================================================================
template <class ConfigSpaceT>
double GenericJoint<ConfigSpaceT>::getConstraintImpulse(size_t index) const
{
  if (index >= getNumDofs())
  {
    GenericJoint_REPORT_OUT_OF_RANGE(getConstraintImpulse, index);
    return 0.0;
  }

  return mConstraintImpulses[index];
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::resetConstraintImpulses()
{
  mConstraintImpulses.setZero();
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::integratePositions(double dt)
{
  const Point& point = math::integratePosition<ConfigSpaceT>(
        math::toManifoldPoint<ConfigSpaceT>(getPositionsStatic()),
        getVelocitiesStatic(), dt);

  setPositionsStatic(math::toEuclideanPoint<ConfigSpaceT>(point));
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::integrateVelocities(double dt)
{
  setVelocitiesStatic(math::integrateVelocity<ConfigSpaceT>(
                        getVelocitiesStatic(),
                        getAccelerationsStatic(), dt));
}

//==============================================================================
template <class ConfigSpaceT>
Eigen::VectorXd GenericJoint<ConfigSpaceT>::getPositionDifferences(
    const Eigen::VectorXd& q2, const Eigen::VectorXd& q1) const
{
  if (static_cast<size_t>(q1.size()) != getNumDofs()
      || static_cast<size_t>(q2.size()) != getNumDofs())
  {
    dterr << "[GenericJoint::getPositionsDifference] q1's size [" << q1.size()
          << "] or q2's size [" << q2.size() << "] must both equal the dof ["
          << this->getNumDofs() << "] for Joint [" << this->getName() << "].\n";
    assert(false);
    return Eigen::VectorXd::Zero(getNumDofs());
  }

  return getPositionDifferencesStatic(q2, q1);
}

//==============================================================================
template <class ConfigSpaceT>
typename ConfigSpaceT::Vector
GenericJoint<ConfigSpaceT>::getPositionDifferencesStatic(
    const Vector& q2, const Vector& q1) const
{
  return q2 - q1;
  // TODO(JS): Move this implementation to each configuration space classes.
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::setSpringStiffness(size_t index, double k)
{
  if (index >= getNumDofs())
  {
    GenericJoint_REPORT_OUT_OF_RANGE(setSpringStiffness, index);
    return;
  }

  assert(k >= 0.0);

  GenericJoint_SET_IF_DIFFERENT( mSpringStiffnesses[index], k );
}

//==============================================================================
template <class ConfigSpaceT>
double GenericJoint<ConfigSpaceT>::getSpringStiffness(size_t index) const
{
  if (index >= getNumDofs())
  {
    GenericJoint_REPORT_OUT_OF_RANGE(getSpringStiffness, index);
    return 0.0;
  }

  return Base::mAspectProperties.mSpringStiffnesses[index];
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::setRestPosition(size_t index, double q0)
{
  if (index >= getNumDofs())
  {
    GenericJoint_REPORT_OUT_OF_RANGE(setRestPosition, index);
    return;
  }

  if (Base::mAspectProperties.mPositionLowerLimits[index] > q0
      || Base::mAspectProperties.mPositionUpperLimits[index] < q0)
  {
    dtwarn << "[GenericJoint::setRestPosition] Value of _q0 [" << q0
           << "], is out of the limit range ["
           << Base::mAspectProperties.mPositionLowerLimits[index] << ", "
           << Base::mAspectProperties.mPositionUpperLimits[index]
           << "] for index [" << index << "] of Joint [" << this->getName()
           << "].\n";
    return;
  }

    GenericJoint_SET_IF_DIFFERENT( mRestPositions[index], q0 );
}

//==============================================================================
template <class ConfigSpaceT>
double GenericJoint<ConfigSpaceT>::getRestPosition(size_t index) const
{
  if (index >= getNumDofs())
  {
    GenericJoint_REPORT_OUT_OF_RANGE(getRestPosition, index);
    return 0.0;
  }

  return Base::mAspectProperties.mRestPositions[index];
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::setDampingCoefficient(size_t index,
                                                          double d)
{
  if (index >= getNumDofs())
  {
    GenericJoint_REPORT_OUT_OF_RANGE(setDampingCoefficient, index);
    return;
  }

  assert(d >= 0.0);

  GenericJoint_SET_IF_DIFFERENT( mDampingCoefficients[index], d );
}

//==============================================================================
template <class ConfigSpaceT>
double GenericJoint<ConfigSpaceT>::getDampingCoefficient(size_t index) const
{
  if (index >= getNumDofs())
  {
    GenericJoint_REPORT_OUT_OF_RANGE(getDampingCoefficient, index);
    return 0.0;
  }

  return Base::mAspectProperties.mDampingCoefficients[index];
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::setCoulombFriction(
    size_t index, double friction)
{
  if (index >= getNumDofs())
  {
    GenericJoint_REPORT_OUT_OF_RANGE(setCoulombFriction, index);
    return;
  }

  assert(friction >= 0.0);

  GenericJoint_SET_IF_DIFFERENT( mFrictions[index], friction );
}

//==============================================================================
template <class ConfigSpaceT>
double GenericJoint<ConfigSpaceT>::getCoulombFriction(size_t index) const
{
  if (index >= getNumDofs())
  {
    GenericJoint_REPORT_OUT_OF_RANGE(getCoulombFriction, index);
    return 0.0;
  }

  return Base::mAspectProperties.mFrictions[index];
}

//==============================================================================
template <class ConfigSpaceT>
double GenericJoint<ConfigSpaceT>::computePotentialEnergy() const
{
  // Spring energy
  Vector displacement
      = getPositionsStatic() - Base::mAspectProperties.mRestPositions;

  const double pe = 0.5 * displacement.dot(
        Base::mAspectProperties.mSpringStiffnesses.cwiseProduct(displacement));

  return pe;
}

//==============================================================================
template <class ConfigSpaceT>
const math::Jacobian
GenericJoint<ConfigSpaceT>::getRelativeJacobian() const
{
  return getRelativeJacobianStatic();
}

//==============================================================================
template <class ConfigSpaceT>
const typename GenericJoint<ConfigSpaceT>::JacobianMatrix&
GenericJoint<ConfigSpaceT>::getRelativeJacobianStatic() const
{
  if (this->mIsRelativeJacobianDirty)
  {
    this->updateRelativeJacobian(false);
    this->mIsRelativeJacobianDirty = false;
  }

  return mJacobian;
}

//==============================================================================
template <class ConfigSpaceT>
math::Jacobian GenericJoint<ConfigSpaceT>::getRelativeJacobian(
    const Eigen::VectorXd& positions) const
{
  return getRelativeJacobianStatic(positions);
}

//==============================================================================
template <class ConfigSpaceT>
const math::Jacobian
GenericJoint<ConfigSpaceT>::getRelativeJacobianTimeDeriv() const
{
  return getRelativeJacobianTimeDerivStatic();
}

//==============================================================================
template <class ConfigSpaceT>
const typename GenericJoint<ConfigSpaceT>::JacobianMatrix&
GenericJoint<ConfigSpaceT>::getRelativeJacobianTimeDerivStatic() const
{
  if (this->mIsRelativeJacobianTimeDerivDirty)
  {
    this->updateRelativeJacobianTimeDeriv();
    this->mIsRelativeJacobianTimeDerivDirty = false;
  }

  return mJacobianDeriv;
}

//==============================================================================
template <class ConfigSpaceT>
GenericJoint<ConfigSpaceT>::GenericJoint(
    const Properties& properties)
  : mVelocityChanges(Vector::Zero()),
    mImpulses(Vector::Zero()),
    mConstraintImpulses(Vector::Zero()),
    mJacobian(JacobianMatrix::Zero()),
    mJacobianDeriv(JacobianMatrix::Zero()),
    mInvProjArtInertia(Matrix::Zero()),
    mInvProjArtInertiaImplicit(Matrix::Zero()),
    mTotalForce(Vector::Zero()),
    mTotalImpulse(Vector::Zero())
{
  for (auto i = 0u; i < NumDofs; ++i)
    mDofs[i] = this->createDofPointer(i);

  // Joint and GenericJoint Aspects must be created by the most derived class.
  this->mAspectState.mPositions = properties.mInitialPositions;
  this->mAspectState.mVelocities = properties.mInitialVelocities;
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::registerDofs()
{
  const SkeletonPtr& skel = this->mChildBodyNode->getSkeleton();
  for (auto i = 0u; i < NumDofs; ++i)
  {
    Base::mAspectProperties.mDofNames[i]
        = skel->mNameMgrForDofs.issueNewNameAndAdd(mDofs[i]->getName(),
                                                   mDofs[i]);
  }
}

//==============================================================================
template <class ConfigSpaceT>
Eigen::Vector6d
GenericJoint<ConfigSpaceT>::getBodyConstraintWrench() const
{
  assert(this->mChildBodyNode);
  return this->mChildBodyNode->getBodyForce()
      - this->getRelativeJacobianStatic() * this->mAspectState.mForces;
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::updateRelativeSpatialVelocity() const
{
  this->mSpatialVelocity =
      this->getRelativeJacobianStatic() * this->getVelocitiesStatic();
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::updateRelativeSpatialAcceleration() const
{
  this->mSpatialAcceleration =
      this->getRelativePrimaryAcceleration()
      + this->getRelativeJacobianTimeDerivStatic() * this->getVelocitiesStatic();
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::updateRelativePrimaryAcceleration() const
{
  this->mPrimaryAcceleration =
      this->getRelativeJacobianStatic() * this->getAccelerationsStatic();
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::addVelocityTo(Eigen::Vector6d& vel)
{
  // Add joint velocity to _vel
  vel.noalias() += getRelativeJacobianStatic() * getVelocitiesStatic();

  // Verification
  assert(!math::isNan(vel));
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::setPartialAccelerationTo(
    Eigen::Vector6d& partialAcceleration,
    const Eigen::Vector6d& childVelocity)
{
  // ad(V, S * dq) + dS * dq
  partialAcceleration = math::ad(childVelocity,
                      getRelativeJacobianStatic() * getVelocitiesStatic())
                    + getRelativeJacobianTimeDerivStatic() * getVelocitiesStatic();
  // Verification
  assert(!math::isNan(partialAcceleration));
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::addAccelerationTo(
    Eigen::Vector6d& acc)
{
  // Add joint acceleration to _acc
  acc.noalias() += getRelativeJacobianStatic() * getAccelerationsStatic();

  // Verification
  assert(!math::isNan(acc));
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::addVelocityChangeTo(
    Eigen::Vector6d& velocityChange)
{
  // Add joint velocity change to velocityChange
  velocityChange.noalias() += getRelativeJacobianStatic() * mVelocityChanges;

  // Verification
  assert(!math::isNan(velocityChange));
}

//==============================================================================
template <class ConfigSpaceT>
const typename GenericJoint<ConfigSpaceT>::Matrix&
GenericJoint<ConfigSpaceT>::getInvProjArtInertia() const
{
  Joint::updateArticulatedInertia();

  return mInvProjArtInertia;
}

//==============================================================================
template <class ConfigSpaceT>
const typename GenericJoint<ConfigSpaceT>::Matrix&
GenericJoint<ConfigSpaceT>::getInvProjArtInertiaImplicit() const
{
  Joint::updateArticulatedInertia();

  return mInvProjArtInertiaImplicit;
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::addChildArtInertiaTo(
    Eigen::Matrix6d& parentArtInertia,
    const Eigen::Matrix6d& childArtInertia)
{
  switch (Joint::mAspectProperties.mActuatorType)
  {
    case Joint::FORCE:
    case Joint::PASSIVE:
    case Joint::SERVO:
      addChildArtInertiaToDynamic(parentArtInertia,
                                       childArtInertia);
      break;
    case Joint::ACCELERATION:
    case Joint::VELOCITY:
    case Joint::LOCKED:
      addChildArtInertiaToKinematic(parentArtInertia,
                                             childArtInertia);
      break;
    default:
      GenericJoint_REPORT_UNSUPPORTED_ACTUATOR(addChildArtInertiaTo);
      break;
  }
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::addChildArtInertiaToDynamic(
    Eigen::Matrix6d& parentArtInertia,
    const Eigen::Matrix6d& childArtInertia)
{
  // Child body's articulated inertia
  JacobianMatrix AIS = childArtInertia * getRelativeJacobianStatic();
  Eigen::Matrix6d PI = childArtInertia;
  PI.noalias() -= AIS * mInvProjArtInertia * AIS.transpose();
  assert(!math::isNan(PI));

  // Add child body's articulated inertia to parent body's articulated inertia.
  // Note that mT should be updated.
  parentArtInertia += math::transformInertia(
        this->getRelativeTransform().inverse(), PI);
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::addChildArtInertiaToKinematic(
    Eigen::Matrix6d& parentArtInertia,
    const Eigen::Matrix6d& childArtInertia)
{
  // Add child body's articulated inertia to parent body's articulated inertia.
  // Note that mT should be updated.
  parentArtInertia += math::transformInertia(
        this->getRelativeTransform().inverse(), childArtInertia);
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::addChildArtInertiaImplicitTo(
    Eigen::Matrix6d& parentArtInertia,
    const Eigen::Matrix6d& childArtInertia)
{
  switch (Joint::mAspectProperties.mActuatorType)
  {
    case Joint::FORCE:
    case Joint::PASSIVE:
    case Joint::SERVO:
      addChildArtInertiaImplicitToDynamic(parentArtInertia,
                                                childArtInertia);
      break;
    case Joint::ACCELERATION:
    case Joint::VELOCITY:
    case Joint::LOCKED:
      addChildArtInertiaImplicitToKinematic(parentArtInertia,
                                                childArtInertia);
      break;
    default:
      GenericJoint_REPORT_UNSUPPORTED_ACTUATOR(addChildArtInertiaImplicitTo);
      break;
  }
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::addChildArtInertiaImplicitToDynamic(
    Eigen::Matrix6d& parentArtInertia,
    const Eigen::Matrix6d& childArtInertia)
{
  // Child body's articulated inertia
  JacobianMatrix AIS = childArtInertia * getRelativeJacobianStatic();
  Eigen::Matrix6d PI = childArtInertia;
  PI.noalias() -= AIS * mInvProjArtInertiaImplicit * AIS.transpose();
  assert(!math::isNan(PI));

  // Add child body's articulated inertia to parent body's articulated inertia.
  // Note that mT should be updated.
  parentArtInertia += math::transformInertia(
        this->getRelativeTransform().inverse(), PI);
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::addChildArtInertiaImplicitToKinematic(
    Eigen::Matrix6d& parentArtInertia,
    const Eigen::Matrix6d& childArtInertia)
{
  // Add child body's articulated inertia to parent body's articulated inertia.
  // Note that mT should be updated.
  parentArtInertia += math::transformInertia(
        this->getRelativeTransform().inverse(), childArtInertia);
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::updateInvProjArtInertia(
    const Eigen::Matrix6d& artInertia)
{
  switch (Joint::mAspectProperties.mActuatorType)
  {
    case Joint::FORCE:
    case Joint::PASSIVE:
    case Joint::SERVO:
      updateInvProjArtInertiaDynamic(artInertia);
      break;
    case Joint::ACCELERATION:
    case Joint::VELOCITY:
    case Joint::LOCKED:
      updateInvProjArtInertiaKinematic(artInertia);
      break;
    default:
      GenericJoint_REPORT_UNSUPPORTED_ACTUATOR(updateInvProjArtInertia);
      break;
  }
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::updateInvProjArtInertiaDynamic(
    const Eigen::Matrix6d& artInertia)
{
  // Projected articulated inertia
  const JacobianMatrix& Jacobian = getRelativeJacobianStatic();
  const Matrix projAI = Jacobian.transpose() * artInertia * Jacobian;

  // Inversion of projected articulated inertia
  mInvProjArtInertia = math::inverse<ConfigSpaceT>(projAI);

  // Verification
  assert(!math::isNan(mInvProjArtInertia));
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::updateInvProjArtInertiaKinematic(
    const Eigen::Matrix6d& /*_artInertia*/)
{
  // Do nothing
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::updateInvProjArtInertiaImplicit(
    const Eigen::Matrix6d& artInertia,
    double timeStep)
{
  switch (Joint::mAspectProperties.mActuatorType)
  {
    case Joint::FORCE:
    case Joint::PASSIVE:
    case Joint::SERVO:
      updateInvProjArtInertiaImplicitDynamic(artInertia, timeStep);
      break;
    case Joint::ACCELERATION:
    case Joint::VELOCITY:
    case Joint::LOCKED:
      updateInvProjArtInertiaImplicitKinematic(artInertia, timeStep);
      break;
    default:
      GenericJoint_REPORT_UNSUPPORTED_ACTUATOR(
            updateInvProjArtInertiaImplicit);
      break;
  }
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::updateInvProjArtInertiaImplicitDynamic(
    const Eigen::Matrix6d& artInertia,
    double timeStep)
{
  // Projected articulated inertia
  const JacobianMatrix& Jacobian = getRelativeJacobianStatic();
  Matrix projAI = Jacobian.transpose() * artInertia * Jacobian;

  // Add additional inertia for implicit damping and spring force
  projAI +=
      (timeStep * Base::mAspectProperties.mDampingCoefficients
       + timeStep * timeStep * Base::mAspectProperties.mSpringStiffnesses).asDiagonal();

  // Inversion of projected articulated inertia
  mInvProjArtInertiaImplicit = math::inverse<ConfigSpaceT>(projAI);

  // Verification
  assert(!math::isNan(mInvProjArtInertiaImplicit));
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::updateInvProjArtInertiaImplicitKinematic(
    const Eigen::Matrix6d& /*artInertia*/, double /*timeStep*/)
{
  // Do nothing
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::addChildBiasForceTo(
    Eigen::Vector6d& parentBiasForce,
    const Eigen::Matrix6d& childArtInertia,
    const Eigen::Vector6d& childBiasForce,
    const Eigen::Vector6d& childPartialAcc)
{
  switch (Joint::mAspectProperties.mActuatorType)
  {
    case Joint::FORCE:
    case Joint::PASSIVE:
    case Joint::SERVO:
      addChildBiasForceToDynamic(parentBiasForce,
                                 childArtInertia,
                                 childBiasForce,
                                 childPartialAcc);
      break;
    case Joint::ACCELERATION:
    case Joint::VELOCITY:
    case Joint::LOCKED:
      addChildBiasForceToKinematic(parentBiasForce,
                                   childArtInertia,
                                   childBiasForce,
                                   childPartialAcc);
      break;
    default:
      GenericJoint_REPORT_UNSUPPORTED_ACTUATOR(addChildBiasForceTo);
      break;
  }
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::addChildBiasForceToDynamic(
    Eigen::Vector6d& parentBiasForce,
    const Eigen::Matrix6d& childArtInertia,
    const Eigen::Vector6d& childBiasForce,
    const Eigen::Vector6d& childPartialAcc)
{
  // Compute beta
  const Eigen::Vector6d beta
      = childBiasForce
        + childArtInertia
          * (childPartialAcc
             + getRelativeJacobianStatic() * getInvProjArtInertiaImplicit()
               *mTotalForce);

  //    Eigen::Vector6d beta
  //        = _childBiasForce;
  //    beta.noalias() += _childArtInertia * _childPartialAcc;
  //    beta.noalias() += _childArtInertia *  mJacobian * getInvProjArtInertiaImplicit() * mTotalForce;

  // Verification
  assert(!math::isNan(beta));

  // Add child body's bias force to parent body's bias force. Note that mT
  // should be updated.
  parentBiasForce += math::dAdInvT(this->getRelativeTransform(), beta);
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::addChildBiasForceToKinematic(
    Eigen::Vector6d& _parentBiasForce,
    const Eigen::Matrix6d& childArtInertia,
    const Eigen::Vector6d& childBiasForce,
    const Eigen::Vector6d& childPartialAcc)
{
  // Compute beta
  const Eigen::Vector6d beta
      = childBiasForce
        + childArtInertia*(childPartialAcc
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
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::addChildBiasImpulseTo(
    Eigen::Vector6d& parentBiasImpulse,
    const Eigen::Matrix6d& childArtInertia,
    const Eigen::Vector6d& childBiasImpulse)
{
  switch (Joint::mAspectProperties.mActuatorType)
  {
    case Joint::FORCE:
    case Joint::PASSIVE:
    case Joint::SERVO:
      addChildBiasImpulseToDynamic(parentBiasImpulse,
                                   childArtInertia,
                                   childBiasImpulse);
      break;
    case Joint::ACCELERATION:
    case Joint::VELOCITY:
    case Joint::LOCKED:
      addChildBiasImpulseToKinematic(parentBiasImpulse,
                                     childArtInertia,
                                     childBiasImpulse);
      break;
    default:
      GenericJoint_REPORT_UNSUPPORTED_ACTUATOR(addChildBiasImpulseTo);
      break;
  }
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::addChildBiasImpulseToDynamic(
    Eigen::Vector6d& _parentBiasImpulse,
    const Eigen::Matrix6d& childArtInertia,
    const Eigen::Vector6d& childBiasImpulse)
{
  // Compute beta
  const Eigen::Vector6d beta
      = childBiasImpulse
        + childArtInertia*getRelativeJacobianStatic()
          *getInvProjArtInertia()*mTotalImpulse;

  // Verification
  assert(!math::isNan(beta));

  // Add child body's bias force to parent body's bias force. Note that mT
  // should be updated.
  _parentBiasImpulse += math::dAdInvT(this->getRelativeTransform(), beta);
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::addChildBiasImpulseToKinematic(
    Eigen::Vector6d& parentBiasImpulse,
    const Eigen::Matrix6d& /*childArtInertia*/,
    const Eigen::Vector6d& childBiasImpulse)
{
  // Add child body's bias force to parent body's bias force. Note that mT
  // should be updated.
  parentBiasImpulse += math::dAdInvT(this->getRelativeTransform(), childBiasImpulse);
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::updateTotalForce(
    const Eigen::Vector6d& bodyForce,
    double timeStep)
{
  assert(timeStep > 0.0);

  switch (Joint::mAspectProperties.mActuatorType)
  {
    case Joint::FORCE:
      this->mAspectState.mForces = this->mAspectState.mCommands;
      updateTotalForceDynamic(bodyForce, timeStep);
      break;
    case Joint::PASSIVE:
    case Joint::SERVO:
      this->mAspectState.mForces.setZero();
      updateTotalForceDynamic(bodyForce, timeStep);
      break;
    case Joint::ACCELERATION:
      setAccelerationsStatic(this->mAspectState.mCommands);
      updateTotalForceKinematic(bodyForce, timeStep);
      break;
    case Joint::VELOCITY:
      setAccelerationsStatic(
            (this->mAspectState.mCommands - getVelocitiesStatic()) / timeStep );
      updateTotalForceKinematic(bodyForce, timeStep);
      break;
    case Joint::LOCKED:
      setVelocitiesStatic(Vector::Zero());
      setAccelerationsStatic(Vector::Zero());
      updateTotalForceKinematic(bodyForce, timeStep);
      break;
    default:
      GenericJoint_REPORT_UNSUPPORTED_ACTUATOR(updateTotalForce);
      break;
  }
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::updateTotalForceDynamic(
    const Eigen::Vector6d& bodyForce,
    double timeStep)
{
  // Spring force
  const Vector springForce
      = -Base::mAspectProperties.mSpringStiffnesses.cwiseProduct(
        getPositionsStatic()
        - Base::mAspectProperties.mRestPositions
        + getVelocitiesStatic() * timeStep);

  // Damping force
  const Vector dampingForce
      = -Base::mAspectProperties.mDampingCoefficients.cwiseProduct(
        getVelocitiesStatic());

  //
  mTotalForce = this->mAspectState.mForces
      + springForce
      + dampingForce
      - getRelativeJacobianStatic().transpose() * bodyForce;
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::updateTotalForceKinematic(
    const Eigen::Vector6d& /*bodyForce*/,
    double /*timeStep*/)
{
  // Do nothing
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::updateTotalImpulse(
    const Eigen::Vector6d& bodyImpulse)
{
  switch (Joint::mAspectProperties.mActuatorType)
  {
    case Joint::FORCE:
    case Joint::PASSIVE:
    case Joint::SERVO:
      updateTotalImpulseDynamic(bodyImpulse);
      break;
    case Joint::ACCELERATION:
    case Joint::VELOCITY:
    case Joint::LOCKED:
      updateTotalImpulseKinematic(bodyImpulse);
      break;
    default:
      GenericJoint_REPORT_UNSUPPORTED_ACTUATOR(updateTotalImpulse);
      break;
  }
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::updateTotalImpulseDynamic(
    const Eigen::Vector6d& bodyImpulse)
{
  //
  mTotalImpulse = mConstraintImpulses
      - getRelativeJacobianStatic().transpose() * bodyImpulse;
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::updateTotalImpulseKinematic(
    const Eigen::Vector6d& /*bodyImpulse*/)
{
  // Do nothing
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::resetTotalImpulses()
{
  mTotalImpulse.setZero();
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::updateAcceleration(
    const Eigen::Matrix6d& artInertia,
    const Eigen::Vector6d& spatialAcc)
{
  switch (Joint::mAspectProperties.mActuatorType)
  {
    case Joint::FORCE:
    case Joint::PASSIVE:
    case Joint::SERVO:
      updateAccelerationDynamic(artInertia, spatialAcc);
      break;
    case Joint::ACCELERATION:
    case Joint::VELOCITY:
    case Joint::LOCKED:
      updateAccelerationKinematic(artInertia, spatialAcc);
      break;
    default:
      GenericJoint_REPORT_UNSUPPORTED_ACTUATOR(updateAcceleration);
      break;
  }
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::updateAccelerationDynamic(
    const Eigen::Matrix6d& artInertia,
    const Eigen::Vector6d& spatialAcc)
{
  //
  setAccelerationsStatic( getInvProjArtInertiaImplicit()
        * (mTotalForce - getRelativeJacobianStatic().transpose()
           *artInertia*math::AdInvT(this->getRelativeTransform(), spatialAcc)) );

  // Verification
  assert(!math::isNan(getAccelerationsStatic()));
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::updateAccelerationKinematic(
    const Eigen::Matrix6d& /*artInertia*/,
    const Eigen::Vector6d& /*spatialAcc*/)
{
  // Do nothing
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::updateVelocityChange(
    const Eigen::Matrix6d& artInertia,
    const Eigen::Vector6d& velocityChange)
{
  switch (Joint::mAspectProperties.mActuatorType)
  {
    case Joint::FORCE:
    case Joint::PASSIVE:
    case Joint::SERVO:
      updateVelocityChangeDynamic(artInertia, velocityChange);
      break;
    case Joint::ACCELERATION:
    case Joint::VELOCITY:
    case Joint::LOCKED:
      updateVelocityChangeKinematic(artInertia, velocityChange);
      break;
    default:
      GenericJoint_REPORT_UNSUPPORTED_ACTUATOR(updateVelocityChange);
      break;
  }
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::updateVelocityChangeDynamic(
    const Eigen::Matrix6d& artInertia,
    const Eigen::Vector6d& velocityChange)
{
  //
  mVelocityChanges
      = getInvProjArtInertia()
      * (mTotalImpulse - getRelativeJacobianStatic().transpose()
         *artInertia*math::AdInvT(this->getRelativeTransform(), velocityChange));

  // Verification
  assert(!math::isNan(mVelocityChanges));
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::updateVelocityChangeKinematic(
    const Eigen::Matrix6d& /*artInertia*/,
    const Eigen::Vector6d& /*velocityChange*/)
{
  // Do nothing
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::updateForceID(
    const Eigen::Vector6d& bodyForce,
    double timeStep,
    bool withDampingForces,
    bool withSpringForces)
{
  this->mAspectState.mForces = getRelativeJacobianStatic().transpose() * bodyForce;

  // Damping force
  if (withDampingForces)
  {
    const typename ConfigSpaceT::Vector dampingForces
        = -Base::mAspectProperties.mDampingCoefficients.cwiseProduct(
          getVelocitiesStatic());
    this->mAspectState.mForces -= dampingForces;
  }

  // Spring force
  if (withSpringForces)
  {
    const typename ConfigSpaceT::Vector springForces
        = -Base::mAspectProperties.mSpringStiffnesses.cwiseProduct(
          getPositionsStatic()
          - Base::mAspectProperties.mRestPositions
          + getVelocitiesStatic() * timeStep);
    this->mAspectState.mForces -= springForces;
  }
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::updateForceFD(
    const Eigen::Vector6d& bodyForce,
    double timeStep,
    bool withDampingForces,
    bool withSpringForces)
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
      updateForceID(bodyForce, timeStep, withDampingForces, withSpringForces);
      break;
    default:
      GenericJoint_REPORT_UNSUPPORTED_ACTUATOR(updateForceFD);
      break;
  }
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::updateImpulseID(
    const Eigen::Vector6d& bodyImpulse)
{
  mImpulses = getRelativeJacobianStatic().transpose()*bodyImpulse;
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::updateImpulseFD(
    const Eigen::Vector6d& bodyImpulse)
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
      updateImpulseID(bodyImpulse);
      break;
    default:
      GenericJoint_REPORT_UNSUPPORTED_ACTUATOR(updateImpulseFD);
      break;
  }
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::updateConstrainedTerms(double timeStep)
{
  switch (Joint::mAspectProperties.mActuatorType)
  {
    case Joint::FORCE:
    case Joint::PASSIVE:
    case Joint::SERVO:
      updateConstrainedTermsDynamic(timeStep);
      break;
    case Joint::ACCELERATION:
    case Joint::VELOCITY:
    case Joint::LOCKED:
      updateConstrainedTermsKinematic(timeStep);
      break;
    default:
      GenericJoint_REPORT_UNSUPPORTED_ACTUATOR(updateConstrainedTerms);
      break;
  }
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::updateConstrainedTermsDynamic(
    double timeStep)
{
  const double invTimeStep = 1.0 / timeStep;

  setVelocitiesStatic(getVelocitiesStatic() + mVelocityChanges);
  setAccelerationsStatic(getAccelerationsStatic()
                         + mVelocityChanges * invTimeStep);
  this->mAspectState.mForces.noalias() += mImpulses * invTimeStep;
  // Note: As long as this is only called from BodyNode::updateConstrainedTerms
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::updateConstrainedTermsKinematic(
    double timeStep)
{
  this->mAspectState.mForces.noalias() += mImpulses / timeStep;
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::addChildBiasForceForInvMassMatrix(
    Eigen::Vector6d& parentBiasForce,
    const Eigen::Matrix6d& childArtInertia,
    const Eigen::Vector6d& childBiasForce)
{
  // Compute beta
  Eigen::Vector6d beta = childBiasForce;
  beta.noalias() += childArtInertia * getRelativeJacobianStatic()
                    * getInvProjArtInertia() * mInvM_a;

  // Verification
  assert(!math::isNan(beta));

  // Add child body's bias force to parent body's bias force. Note that mT
  // should be updated.
  parentBiasForce += math::dAdInvT(this->getRelativeTransform(), beta);
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::addChildBiasForceForInvAugMassMatrix(
    Eigen::Vector6d& parentBiasForce,
    const Eigen::Matrix6d& childArtInertia,
    const Eigen::Vector6d& childBiasForce)
{
  // Compute beta
  Eigen::Vector6d beta = childBiasForce;
  beta.noalias() += childArtInertia * getRelativeJacobianStatic()
                    * getInvProjArtInertiaImplicit() * mInvM_a;

  // Verification
  assert(!math::isNan(beta));

  // Add child body's bias force to parent body's bias force. Note that mT
  // should be updated.
  parentBiasForce += math::dAdInvT(this->getRelativeTransform(), beta);
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::updateTotalForceForInvMassMatrix(
    const Eigen::Vector6d& bodyForce)
{
  // Compute alpha
  mInvM_a = this->mAspectState.mForces
      - getRelativeJacobianStatic().transpose() * bodyForce;
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::getInvMassMatrixSegment(
    Eigen::MatrixXd& _invMassMat,
    const size_t _col,
    const Eigen::Matrix6d& artInertia,
    const Eigen::Vector6d& spatialAcc)
{
  //
  mInvMassMatrixSegment
      = getInvProjArtInertia()
      * (mInvM_a - getRelativeJacobianStatic().transpose()
         * artInertia * math::AdInvT(this->getRelativeTransform(), spatialAcc));

  // Verification
  assert(!math::isNan(mInvMassMatrixSegment));

  // Index
  size_t iStart = mDofs[0]->mIndexInTree;

  // Assign
  _invMassMat.block<NumDofs, 1>(iStart, _col) = mInvMassMatrixSegment;
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::getInvAugMassMatrixSegment(
    Eigen::MatrixXd& invMassMat,
    const size_t col,
    const Eigen::Matrix6d& artInertia,
    const Eigen::Vector6d& spatialAcc)
{
  //
  mInvMassMatrixSegment
      = getInvProjArtInertiaImplicit()
      * (mInvM_a - getRelativeJacobianStatic().transpose()
         * artInertia * math::AdInvT(this->getRelativeTransform(), spatialAcc));

  // Verification
  assert(!math::isNan(mInvMassMatrixSegment));

  // Index
  size_t iStart = mDofs[0]->mIndexInTree;

  // Assign
  invMassMat.block<NumDofs, 1>(iStart, col) = mInvMassMatrixSegment;
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::addInvMassMatrixSegmentTo(
    Eigen::Vector6d& acc)
{
  //
  acc += getRelativeJacobianStatic() * mInvMassMatrixSegment;
}

//==============================================================================
template <class ConfigSpaceT>
Eigen::VectorXd GenericJoint<ConfigSpaceT>::getSpatialToGeneralized(
    const Eigen::Vector6d& spatial)
{
  return getRelativeJacobianStatic().transpose() * spatial;
}

} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_DETAIL_GenericJoint_HPP_
