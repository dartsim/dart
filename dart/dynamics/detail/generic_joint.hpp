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

#ifndef DART_DYNAMICS_DETAIL_GenericJoint_HPP_
#define DART_DYNAMICS_DETAIL_GenericJoint_HPP_

#include "dart/common/macros.hpp"

#include <dart/config.hpp>

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/degree_of_freedom.hpp>
#include <dart/dynamics/generic_joint.hpp>

#include <dart/math/configuration_space.hpp>
#include <dart/math/helpers.hpp>

#include <sstream>

#include <cmath>

#define GenericJoint_REPORT_DIM_MISMATCH(func, arg)                            \
  {                                                                            \
    DART_ERROR(                                                                \
        "The size of {} [{}] does not match the number of "                    \
        "DOFs [{}] for Joint named [{}].",                                     \
        #func,                                                                 \
        #arg,                                                                  \
        (arg).size(),                                                          \
        this->getNumDofs(),                                                    \
        this->getName());                                                      \
    DART_ASSERT(false);                                                        \
  }

#define GenericJoint_REPORT_OUT_OF_RANGE(func, index)                          \
  {                                                                            \
    DART_ERROR(                                                                \
        "The index [{}] is out of range for Joint named "                      \
        "[{}] which has {} DOFs.",                                             \
        #func,                                                                 \
        (index),                                                               \
        this->getName(),                                                       \
        this->getNumDofs());                                                   \
    DART_ASSERT(false);                                                        \
  }

#define GenericJoint_REPORT_UNSUPPORTED_ACTUATOR(func, typeExpr)               \
  {                                                                            \
    DART_ERROR(                                                                \
        "Unsupported actuator type ({}) for Joint [{}].",                      \
        #func,                                                                 \
        static_cast<int>(typeExpr),                                            \
        this->getName());                                                      \
    DART_ASSERT(false);                                                        \
  }

#define GenericJoint_SET_IF_DIFFERENT(mField, value)                           \
  if (dart::math::valueEqual(value, Base::mAspectProperties.mField))           \
    return;                                                                    \
  Base::mAspectProperties.mField = value;                                      \
  Joint::incrementVersion();

namespace dart::dynamics::detail {

inline std::string formatCommandVector(const Eigen::VectorXd& commands)
{
  std::ostringstream oss;
  oss << commands.transpose();
  return oss.str();
}

template <typename Derived>
inline void assertFiniteState(
    const Eigen::MatrixBase<Derived>& values,
    const Joint* joint,
    const char* context,
    const char* valueName)
{
#ifndef NDEBUG
  const bool finite = values.array().isFinite().all();
  if (!finite) {
    DART_ERROR(
        "Non-finite {} passed to {} for Joint [{}] ({}).",
        valueName,
        context,
        joint->getName(),
        static_cast<const void*>(joint));
  }
  DART_ASSERT(finite);
#else
  (void)values;
  (void)joint;
  (void)context;
  (void)valueName;
#endif
}

inline void assertFiniteState(
    double value,
    const Joint* joint,
    const char* context,
    const char* valueName)
{
#ifndef NDEBUG
  const bool finite = std::isfinite(value);
  if (!finite) {
    DART_ERROR(
        "Non-finite {} passed to {} for Joint [{}] ({}).",
        valueName,
        context,
        joint->getName(),
        static_cast<const void*>(joint));
  }
  DART_ASSERT(finite);
#else
  (void)value;
  (void)joint;
  (void)context;
  (void)valueName;
#endif
}

} // namespace dart::dynamics::detail

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
  for (auto i = 0u; i < NumDofs; ++i) {
    delete mDofs[i];
  }
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
  this->mAspectState.mVelocityChanges = state.mVelocityChanges;
  this->mAspectState.mImpulses = state.mImpulses;
  this->mAspectState.mConstraintImpulses = state.mConstraintImpulses;
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::setAspectProperties(
    const AspectProperties& properties)
{
  for (auto i = 0u; i < NumDofs; ++i) {
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
template <class ConfigSpaceT>
typename GenericJoint<ConfigSpaceT>::Properties
GenericJoint<ConfigSpaceT>::getGenericJointProperties() const
{
  return GenericJoint<ConfigSpaceT>::Properties(
      Joint::mAspectProperties, Base::mAspectProperties);
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::copy(const GenericJoint<ConfigSpaceT>& other)
{
  if (this == &other) {
    return;
  }

  setProperties(other.getGenericJointProperties());
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::copy(const GenericJoint<ConfigSpaceT>* other)
{
  if (nullptr == other) {
    return;
  }

  copy(*other);
}

//==============================================================================
template <class ConfigSpaceT>
GenericJoint<ConfigSpaceT>& GenericJoint<ConfigSpaceT>::operator=(
    const GenericJoint<ConfigSpaceT>& other)
{
  copy(other);
  return *this;
}

//==============================================================================
template <class ConfigSpaceT>
DegreeOfFreedom* GenericJoint<ConfigSpaceT>::getDof(size_t index)
{
  if (index < NumDofs) {
    return mDofs[index];
  }

  GenericJoint_REPORT_OUT_OF_RANGE(getDof, index);

  return nullptr;
}

//==============================================================================
template <class ConfigSpaceT>
const DegreeOfFreedom* GenericJoint<ConfigSpaceT>::getDof(size_t index) const
{
  if (index < NumDofs) {
    return mDofs[index];
  }

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
    size_t index, const std::string& name, bool preserveName)
{
  if (NumDofs <= index) {
    DART_ERROR(
        "Attempting to set the name of DOF index {}, which is out of bounds "
        "for the Joint [{}]. We will set the name of DOF index 0 instead.",
        index,
        this->getName());
    DART_ASSERT(false);
    index = 0u;
  }

  preserveDofName(index, preserveName);

  std::string& dofName = Base::mAspectProperties.mDofNames[index];

  if (name == dofName) {
    return dofName;
  }

  const SkeletonPtr& skel
      = this->mChildBodyNode ? this->mChildBodyNode->getSkeleton() : nullptr;
  if (skel) {
    dofName = skel->mNameMgrForDofs.changeObjectName(mDofs[index], name);
  } else {
    dofName = name;
  }

  return dofName;
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::preserveDofName(size_t index, bool preserve)
{
  if (NumDofs <= index) {
    GenericJoint_REPORT_OUT_OF_RANGE(preserveDofName, index);
    return;
  }

  GenericJoint_SET_IF_DIFFERENT(mPreserveDofNames[index], preserve);
}

//==============================================================================
template <class ConfigSpaceT>
bool GenericJoint<ConfigSpaceT>::isDofNamePreserved(size_t index) const
{
  if (NumDofs <= index) {
    GenericJoint_REPORT_OUT_OF_RANGE(isDofNamePreserved, index);
    index = 0;
  }

  return Base::mAspectProperties.mPreserveDofNames[index];
}

//==============================================================================
template <class ConfigSpaceT>
const std::string& GenericJoint<ConfigSpaceT>::getDofName(size_t index) const
{
  if (NumDofs <= index) {
    DART_ERROR(
        "Requested name of DOF index [{}] in Joint [{}], but that is out of "
        "bounds (max {}). Returning name of DOF 0.",
        index,
        this->getName(),
        NumDofs - 1);
    DART_ASSERT(false);
    return Base::mAspectProperties.mDofNames[0];
  }

  return Base::mAspectProperties.mDofNames[index];
}

//==============================================================================
template <class ConfigSpaceT>
size_t GenericJoint<ConfigSpaceT>::getIndexInSkeleton(size_t index) const
{
  if (index >= getNumDofs()) {
    GenericJoint_REPORT_OUT_OF_RANGE(getIndexInSkeleton, index);
    return 0;
  }

  return mDofs[index]->mIndexInSkeleton;
}

//==============================================================================
template <class ConfigSpaceT>
size_t GenericJoint<ConfigSpaceT>::getIndexInTree(size_t index) const
{
  if (index >= getNumDofs()) {
    GenericJoint_REPORT_OUT_OF_RANGE(getIndexInTree, index);
    return 0;
  }

  return mDofs[index]->mIndexInTree;
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::setCommand(size_t index, double command)
{
  if (index >= getNumDofs()) {
    GenericJoint_REPORT_OUT_OF_RANGE(setCommand, index);
  }

  // Validate command is finite to prevent NaN/Inf from propagating through
  // the simulation and causing assertion failures (gz-physics#845)
  if (!std::isfinite(command)) {
    DART_WARN(
        "Non-finite command ({}) passed to setCommand() for Joint [{}] "
        "DOF [{}]. Command ignored.",
        command,
        this->getName(),
        index);
    return;
  }

  const auto actuatorType = this->getActuatorType(index);
  switch (actuatorType) {
    case Joint::FORCE:
      this->mAspectState.mCommands[index] = math::clip(
          command,
          Base::mAspectProperties.mForceLowerLimits[index],
          Base::mAspectProperties.mForceUpperLimits[index]);
      break;
    case Joint::PASSIVE:
      DART_WARN_IF(
          0.0 != command,
          "Attempting to set a non-zero ({}) command for a PASSIVE joint [{}].",
          command,
          this->getName());
      this->mAspectState.mCommands[index] = 0.0;
      break;
    case Joint::SERVO:
      this->mAspectState.mCommands[index] = math::clip(
          command,
          Base::mAspectProperties.mVelocityLowerLimits[index],
          Base::mAspectProperties.mVelocityUpperLimits[index]);
      break;
    case Joint::MIMIC:
      DART_WARN_IF(
          0.0 != command,
          "Attempting to set a non-zero ({}) command for a MIMIC joint [{}].",
          command,
          this->getName());
      this->mAspectState.mCommands[index] = 0.0;
      break;
    case Joint::ACCELERATION:
      this->mAspectState.mCommands[index] = math::clip(
          command,
          Base::mAspectProperties.mAccelerationLowerLimits[index],
          Base::mAspectProperties.mAccelerationUpperLimits[index]);
      break;
    case Joint::VELOCITY:
      this->mAspectState.mCommands[index] = math::clip(
          command,
          Base::mAspectProperties.mVelocityLowerLimits[index],
          Base::mAspectProperties.mVelocityUpperLimits[index]);
      // TODO: This possibly makes the acceleration to exceed the limits.
      break;
    case Joint::LOCKED:
      DART_WARN_IF(
          0.0 != command,
          "Attempting to set a non-zero ({}) command for a LOCKED joint [{}].",
          command,
          this->getName());
      this->mAspectState.mCommands[index] = 0.0;
      break;
    default: {
      GenericJoint_REPORT_UNSUPPORTED_ACTUATOR(setCommand, actuatorType);
      break;
    }
  }
}

//==============================================================================
template <class ConfigSpaceT>
double GenericJoint<ConfigSpaceT>::getCommand(size_t index) const
{
  if (index >= getNumDofs()) {
    GenericJoint_REPORT_OUT_OF_RANGE(getCommand, index);
    return 0.0;
  }

  return this->mAspectState.mCommands[index];
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::setCommands(const Eigen::VectorXd& commands)
{
  if (static_cast<size_t>(commands.size()) != getNumDofs()) {
    GenericJoint_REPORT_DIM_MISMATCH(setCommands, commands);
    return;
  }

  for (std::size_t i = 0; i < getNumDofs(); ++i) {
    setCommand(i, commands[static_cast<Eigen::Index>(i)]);
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
  if (index >= getNumDofs()) {
    GenericJoint_REPORT_OUT_OF_RANGE(setPosition, index);
    return;
  }

  detail::assertFiniteState(position, this, "setPosition", "position");

  if (this->mAspectState.mPositions[index] == position) {
    return;
  }
  // TODO(JS): Above code should be changed something like:
  //  if (ConfigSpaceT::getEuclideanPoint(mPositions, index) == position)
  //    return;

  // Note: It would not make much sense to use setPositionsStatic() here
  this->mAspectState.mPositions[index] = position;
  this->notifyPositionUpdated();
}

//==============================================================================
template <class ConfigSpaceT>
double GenericJoint<ConfigSpaceT>::getPosition(size_t index) const
{
  if (index >= getNumDofs()) {
    GenericJoint_REPORT_OUT_OF_RANGE(getPosition, index);
    return 0.0;
  }

  return getPositionsStatic()[index];
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::setPositions(const Eigen::VectorXd& positions)
{
  if (static_cast<size_t>(positions.size()) != getNumDofs()) {
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
  if (index >= getNumDofs()) {
    GenericJoint_REPORT_OUT_OF_RANGE(setPositionLowerLimit, index);
    return;
  }

  GenericJoint_SET_IF_DIFFERENT(mPositionLowerLimits[index], position);
}

//==============================================================================
template <class ConfigSpaceT>
double GenericJoint<ConfigSpaceT>::getPositionLowerLimit(size_t index) const
{
  if (index >= getNumDofs()) {
    GenericJoint_REPORT_OUT_OF_RANGE(getPositionLowerLimit, index);
    return 0.0;
  }

  return Base::mAspectProperties.mPositionLowerLimits[index];
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::setPositionLowerLimits(
    const Eigen::VectorXd& lowerLimits)
{
  if (static_cast<size_t>(lowerLimits.size()) != getNumDofs()) {
    GenericJoint_REPORT_DIM_MISMATCH(setPositionLowerLimits, lowerLimits);
    return;
  }

  GenericJoint_SET_IF_DIFFERENT(mPositionLowerLimits, lowerLimits);
}

//==============================================================================
template <class ConfigSpaceT>
Eigen::VectorXd GenericJoint<ConfigSpaceT>::getPositionLowerLimits() const
{
  return Base::mAspectProperties.mPositionLowerLimits;
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::setPositionUpperLimit(
    size_t index, double position)
{
  if (index >= getNumDofs()) {
    GenericJoint_REPORT_OUT_OF_RANGE(setPositionUpperLimit, index);
    return;
  }

  GenericJoint_SET_IF_DIFFERENT(mPositionUpperLimits[index], position);
}

//==============================================================================
template <class ConfigSpaceT>
double GenericJoint<ConfigSpaceT>::getPositionUpperLimit(size_t index) const
{
  if (index >= getNumDofs()) {
    GenericJoint_REPORT_OUT_OF_RANGE(getPositionUpperLimit, index);
    return 0.0;
  }

  return Base::mAspectProperties.mPositionUpperLimits[index];
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::setPositionUpperLimits(
    const Eigen::VectorXd& upperLimits)
{
  if (static_cast<size_t>(upperLimits.size()) != getNumDofs()) {
    GenericJoint_REPORT_DIM_MISMATCH(setPositionUpperLimits, upperLimits);
    return;
  }

  GenericJoint_SET_IF_DIFFERENT(mPositionUpperLimits, upperLimits);
}

//==============================================================================
template <class ConfigSpaceT>
Eigen::VectorXd GenericJoint<ConfigSpaceT>::getPositionUpperLimits() const
{
  return Base::mAspectProperties.mPositionUpperLimits;
}

//==============================================================================
template <class ConfigSpaceT>
bool GenericJoint<ConfigSpaceT>::hasPositionLimit(size_t index) const
{
  if (index >= getNumDofs()) {
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
  if (index >= getNumDofs()) {
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
  if (index >= getNumDofs()) {
    GenericJoint_REPORT_OUT_OF_RANGE(setInitialPosition, index);
    return;
  }

  GenericJoint_SET_IF_DIFFERENT(mInitialPositions[index], initial);
}

//==============================================================================
template <class ConfigSpaceT>
double GenericJoint<ConfigSpaceT>::getInitialPosition(size_t index) const
{
  if (index >= getNumDofs()) {
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
  if (static_cast<size_t>(initial.size()) != getNumDofs()) {
    GenericJoint_REPORT_DIM_MISMATCH(setInitialPositions, initial);
    return;
  }

  GenericJoint_SET_IF_DIFFERENT(mInitialPositions, initial);
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
  detail::assertFiniteState(positions, this, "setPositions", "positions");

  if (this->mAspectState.mPositions == positions) {
    return;
  }

  this->mAspectState.mPositions = positions;
  this->notifyPositionUpdated();
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
void GenericJoint<ConfigSpaceT>::setVelocitiesStatic(const Vector& velocities)
{
  detail::assertFiniteState(velocities, this, "setVelocities", "velocities");

  if (this->mAspectState.mVelocities == velocities) {
    return;
  }

  this->mAspectState.mVelocities = velocities;
  this->notifyVelocityUpdated();
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
  detail::assertFiniteState(accels, this, "setAccelerations", "accelerations");

  if (this->mAspectState.mAccelerations == accels) {
    return;
  }

  this->mAspectState.mAccelerations = accels;
  this->notifyAccelerationUpdated();
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
  if (index >= getNumDofs()) {
    GenericJoint_REPORT_OUT_OF_RANGE(setVelocity, index);
    return;
  }

  detail::assertFiniteState(velocity, this, "setVelocity", "velocity");

  if (this->mAspectState.mVelocities[index] == velocity) {
    return;
  }

  // Note: It would not make much sense to use setVelocitiesStatic() here
  this->mAspectState.mVelocities[index] = velocity;
  this->notifyVelocityUpdated();

  if (this->getActuatorType(index) == Joint::VELOCITY) {
    this->mAspectState.mCommands[index] = this->getVelocitiesStatic()[index];
  }
}

//==============================================================================
template <class ConfigSpaceT>
double GenericJoint<ConfigSpaceT>::getVelocity(size_t index) const
{
  if (index >= getNumDofs()) {
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
  if (static_cast<size_t>(velocities.size()) != getNumDofs()) {
    GenericJoint_REPORT_DIM_MISMATCH(setVelocities, velocities);
    return;
  }

  setVelocitiesStatic(velocities);

  for (std::size_t i = 0; i < getNumDofs(); ++i) {
    if (this->getActuatorType(i) == Joint::VELOCITY) {
      this->mAspectState.mCommands[i] = this->mAspectState.mVelocities[i];
    }
  }
}

//==============================================================================
template <class ConfigSpaceT>
Eigen::VectorXd GenericJoint<ConfigSpaceT>::getVelocities() const
{
  return getVelocitiesStatic();
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::setVelocityLowerLimit(
    size_t index, double velocity)
{
  if (index >= getNumDofs()) {
    GenericJoint_REPORT_OUT_OF_RANGE(setVelocityLowerLimit, index);
    return;
  }

  GenericJoint_SET_IF_DIFFERENT(mVelocityLowerLimits[index], velocity);
}

//==============================================================================
template <class ConfigSpaceT>
double GenericJoint<ConfigSpaceT>::getVelocityLowerLimit(size_t index) const
{
  if (index >= getNumDofs()) {
    GenericJoint_REPORT_OUT_OF_RANGE(getVelocityLowerLimit, index);
    return 0.0;
  }

  return Base::mAspectProperties.mVelocityLowerLimits[index];
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::setVelocityLowerLimits(
    const Eigen::VectorXd& lowerLimits)
{
  if (static_cast<size_t>(lowerLimits.size()) != getNumDofs()) {
    GenericJoint_REPORT_DIM_MISMATCH(setVelocityLowerLimits, lowerLimits);
    return;
  }

  GenericJoint_SET_IF_DIFFERENT(mVelocityLowerLimits, lowerLimits);
}

//==============================================================================
template <class ConfigSpaceT>
Eigen::VectorXd GenericJoint<ConfigSpaceT>::getVelocityLowerLimits() const
{
  return Base::mAspectProperties.mVelocityLowerLimits;
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::setVelocityUpperLimit(
    size_t index, double velocity)
{
  if (index >= getNumDofs()) {
    GenericJoint_REPORT_OUT_OF_RANGE(setVelocityUpperLimit, index);
    return;
  }

  GenericJoint_SET_IF_DIFFERENT(mVelocityUpperLimits[index], velocity);
}

//==============================================================================
template <class ConfigSpaceT>
double GenericJoint<ConfigSpaceT>::getVelocityUpperLimit(size_t index) const
{
  if (index >= getNumDofs()) {
    GenericJoint_REPORT_OUT_OF_RANGE(getVelocityUpperLimit, index);
    return 0.0;
  }

  return Base::mAspectProperties.mVelocityUpperLimits[index];
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::setVelocityUpperLimits(
    const Eigen::VectorXd& upperLimits)
{
  if (static_cast<size_t>(upperLimits.size()) != getNumDofs()) {
    GenericJoint_REPORT_DIM_MISMATCH(setVelocityUpperLimits, upperLimits);
    return;
  }

  GenericJoint_SET_IF_DIFFERENT(mVelocityUpperLimits, upperLimits);
}

//==============================================================================
template <class ConfigSpaceT>
Eigen::VectorXd GenericJoint<ConfigSpaceT>::getVelocityUpperLimits() const
{
  return Base::mAspectProperties.mVelocityUpperLimits;
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::resetVelocity(size_t index)
{
  if (index >= getNumDofs()) {
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
void GenericJoint<ConfigSpaceT>::setInitialVelocity(
    size_t index, double initial)
{
  if (index >= getNumDofs()) {
    GenericJoint_REPORT_OUT_OF_RANGE(setInitialVelocity, index);
    return;
  }

  GenericJoint_SET_IF_DIFFERENT(mInitialVelocities[index], initial);
}

//==============================================================================
template <class ConfigSpaceT>
double GenericJoint<ConfigSpaceT>::getInitialVelocity(size_t index) const
{
  if (index >= getNumDofs()) {
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
  if (static_cast<size_t>(initial.size()) != getNumDofs()) {
    GenericJoint_REPORT_DIM_MISMATCH(setInitialVelocities, initial);
    return;
  }

  GenericJoint_SET_IF_DIFFERENT(mInitialVelocities, initial);
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
  if (index >= getNumDofs()) {
    GenericJoint_REPORT_OUT_OF_RANGE(setAcceleration, index);
    return;
  }

  detail::assertFiniteState(
      acceleration, this, "setAcceleration", "acceleration");

  if (this->mAspectState.mAccelerations[index] == acceleration) {
    return;
  }

  // Note: It would not make much sense to use setAccelerationsStatic() here
  this->mAspectState.mAccelerations[index] = acceleration;
  this->notifyAccelerationUpdated();

  if (this->getActuatorType(index) == Joint::ACCELERATION) {
    this->mAspectState.mCommands[index] = this->getAccelerationsStatic()[index];
  }
}

//==============================================================================
template <class ConfigSpaceT>
double GenericJoint<ConfigSpaceT>::getAcceleration(size_t index) const
{
  if (index >= getNumDofs()) {
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
  if (static_cast<size_t>(accelerations.size()) != getNumDofs()) {
    GenericJoint_REPORT_DIM_MISMATCH(setAccelerations, accelerations);
    return;
  }

  setAccelerationsStatic(accelerations);

  for (std::size_t i = 0; i < getNumDofs(); ++i) {
    if (this->getActuatorType(i) == Joint::ACCELERATION) {
      this->mAspectState.mCommands[i] = this->mAspectState.mAccelerations[i];
    }
  }
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
  if (index >= getNumDofs()) {
    GenericJoint_REPORT_OUT_OF_RANGE(setAccelerationLowerLimit, index);
    return;
  }

  GenericJoint_SET_IF_DIFFERENT(mAccelerationLowerLimits[index], acceleration);
}

//==============================================================================
template <class ConfigSpaceT>
double GenericJoint<ConfigSpaceT>::getAccelerationLowerLimit(size_t index) const
{
  if (index >= getNumDofs()) {
    GenericJoint_REPORT_OUT_OF_RANGE(getAccelerationLowerLimit, index);
    return 0.0;
  }

  return Base::mAspectProperties.mAccelerationLowerLimits[index];
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::setAccelerationLowerLimits(
    const Eigen::VectorXd& lowerLimits)
{
  if (static_cast<size_t>(lowerLimits.size()) != getNumDofs()) {
    GenericJoint_REPORT_DIM_MISMATCH(setAccelerationLowerLimits, lowerLimits);
    return;
  }

  GenericJoint_SET_IF_DIFFERENT(mAccelerationLowerLimits, lowerLimits);
}

//==============================================================================
template <class ConfigSpaceT>
Eigen::VectorXd GenericJoint<ConfigSpaceT>::getAccelerationLowerLimits() const
{
  return Base::mAspectProperties.mAccelerationLowerLimits;
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::setAccelerationUpperLimit(
    size_t index, double acceleration)
{
  if (index >= getNumDofs()) {
    GenericJoint_REPORT_OUT_OF_RANGE(setAccelerationUpperLimit, index) return;
  }

  GenericJoint_SET_IF_DIFFERENT(mAccelerationUpperLimits[index], acceleration);
}

//==============================================================================
template <class ConfigSpaceT>
double GenericJoint<ConfigSpaceT>::getAccelerationUpperLimit(size_t index) const
{
  if (index >= getNumDofs()) {
    GenericJoint_REPORT_OUT_OF_RANGE(getAccelerationUpperLimit, index);
    return 0.0;
  }

  return Base::mAspectProperties.mAccelerationUpperLimits[index];
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::setAccelerationUpperLimits(
    const Eigen::VectorXd& upperLimits)
{
  if (static_cast<size_t>(upperLimits.size()) != getNumDofs()) {
    GenericJoint_REPORT_DIM_MISMATCH(setAccelerationUpperLimits, upperLimits);
    return;
  }

  GenericJoint_SET_IF_DIFFERENT(mAccelerationUpperLimits, upperLimits);
}

//==============================================================================
template <class ConfigSpaceT>
Eigen::VectorXd GenericJoint<ConfigSpaceT>::getAccelerationUpperLimits() const
{
  return Base::mAspectProperties.mAccelerationUpperLimits;
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
  if (index >= getNumDofs()) {
    GenericJoint_REPORT_OUT_OF_RANGE(setForce, index);
    return;
  }

  this->mAspectState.mForces[index] = force;

  if (this->getActuatorType(index) == Joint::FORCE) {
    this->mAspectState.mCommands[index] = this->mAspectState.mForces[index];
  }
}

//==============================================================================
template <class ConfigSpaceT>
double GenericJoint<ConfigSpaceT>::getForce(size_t index) const
{
  if (index >= getNumDofs()) {
    GenericJoint_REPORT_OUT_OF_RANGE(getForce, index);
    return 0.0;
  }

  return this->mAspectState.mForces[index];
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::setForces(const Eigen::VectorXd& forces)
{
  if (static_cast<size_t>(forces.size()) != getNumDofs()) {
    GenericJoint_REPORT_DIM_MISMATCH(setForces, forces);
    return;
  }

  this->mAspectState.mForces = forces;

  for (std::size_t i = 0; i < getNumDofs(); ++i) {
    if (this->getActuatorType(i) == Joint::FORCE) {
      this->mAspectState.mCommands[i] = this->mAspectState.mForces[i];
    }
  }
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
  if (index >= getNumDofs()) {
    GenericJoint_REPORT_OUT_OF_RANGE(setForceLowerLimit, index);
    return;
  }

  GenericJoint_SET_IF_DIFFERENT(mForceLowerLimits[index], force);
}

//==============================================================================
template <class ConfigSpaceT>
double GenericJoint<ConfigSpaceT>::getForceLowerLimit(size_t index) const
{
  if (index >= getNumDofs()) {
    GenericJoint_REPORT_OUT_OF_RANGE(getForceLowerLimit, index);
    return 0.0;
  }

  return Base::mAspectProperties.mForceLowerLimits[index];
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::setForceLowerLimits(
    const Eigen::VectorXd& lowerLimits)
{
  if (static_cast<size_t>(lowerLimits.size()) != getNumDofs()) {
    GenericJoint_REPORT_DIM_MISMATCH(setForceLowerLimits, lowerLimits);
    return;
  }

  GenericJoint_SET_IF_DIFFERENT(mForceLowerLimits, lowerLimits);
}

//==============================================================================
template <class ConfigSpaceT>
Eigen::VectorXd GenericJoint<ConfigSpaceT>::getForceLowerLimits() const
{
  return Base::mAspectProperties.mForceLowerLimits;
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::setForceUpperLimit(size_t index, double force)
{
  if (index >= getNumDofs()) {
    GenericJoint_REPORT_OUT_OF_RANGE(setForceUpperLimit, index);
    return;
  }

  GenericJoint_SET_IF_DIFFERENT(mForceUpperLimits[index], force);
}

//==============================================================================
template <class ConfigSpaceT>
double GenericJoint<ConfigSpaceT>::getForceUpperLimit(size_t index) const
{
  if (index >= getNumDofs()) {
    GenericJoint_REPORT_OUT_OF_RANGE(getForceUpperLimit, index);
    return 0.0;
  }

  return Base::mAspectProperties.mForceUpperLimits[index];
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::setForceUpperLimits(
    const Eigen::VectorXd& upperLimits)
{
  if (static_cast<size_t>(upperLimits.size()) != getNumDofs()) {
    GenericJoint_REPORT_DIM_MISMATCH(setForceUpperLimits, upperLimits);
    return;
  }

  GenericJoint_SET_IF_DIFFERENT(mForceUpperLimits, upperLimits);
}

//==============================================================================
template <class ConfigSpaceT>
Eigen::VectorXd GenericJoint<ConfigSpaceT>::getForceUpperLimits() const
{
  return Base::mAspectProperties.mForceUpperLimits;
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::resetForces()
{
  this->mAspectState.mForces.setZero();

  for (std::size_t i = 0; i < getNumDofs(); ++i) {
    if (this->getActuatorType(i) == Joint::FORCE) {
      this->mAspectState.mCommands[i] = this->mAspectState.mForces[i];
    }
  }
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::setVelocityChange(
    size_t index, double velocityChange)
{
  if (index >= getNumDofs()) {
    GenericJoint_REPORT_OUT_OF_RANGE(setVelocityChange, index);
    return;
  }

  this->mAspectState.mVelocityChanges[index] = velocityChange;
}

//==============================================================================
template <class ConfigSpaceT>
double GenericJoint<ConfigSpaceT>::getVelocityChange(size_t index) const
{
  if (index >= getNumDofs()) {
    GenericJoint_REPORT_OUT_OF_RANGE(getVelocityChange, index);
    return 0.0;
  }

  return this->mAspectState.mVelocityChanges[index];
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::resetVelocityChanges()
{
  this->mAspectState.mVelocityChanges.setZero();
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::setConstraintImpulse(
    size_t index, double impulse)
{
  if (index >= getNumDofs()) {
    GenericJoint_REPORT_OUT_OF_RANGE(setConstraintImpulse, index);
    return;
  }

  this->mAspectState.mConstraintImpulses[index] = impulse;
}

//==============================================================================
template <class ConfigSpaceT>
double GenericJoint<ConfigSpaceT>::getConstraintImpulse(size_t index) const
{
  if (index >= getNumDofs()) {
    GenericJoint_REPORT_OUT_OF_RANGE(getConstraintImpulse, index);
    return 0.0;
  }

  return this->mAspectState.mConstraintImpulses[index];
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::resetConstraintImpulses()
{
  this->mAspectState.mConstraintImpulses.setZero();
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::integratePositions(double dt)
{
  const Point& point = math::integratePosition<ConfigSpaceT>(
      math::toManifoldPoint<ConfigSpaceT>(getPositionsStatic()),
      getVelocitiesStatic(),
      dt);

  setPositionsStatic(math::toEuclideanPoint<ConfigSpaceT>(point));
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::integratePositions(
    const Eigen::VectorXd& q0,
    const Eigen::VectorXd& v,
    double dt,
    Eigen::VectorXd& result) const
{
  if (static_cast<std::size_t>(q0.size()) != getNumDofs()
      || static_cast<std::size_t>(v.size()) != getNumDofs()) {
    DART_ERROR(
        "q0's size [{}] and v's size [{}] must both equal the dof [{}] for "
        "Joint [{}].",
        q0.size(),
        v.size(),
        this->getNumDofs(),
        this->getName());
    DART_ASSERT(false);
    result = Eigen::VectorXd::Zero(getNumDofs());
    return;
  }

  detail::assertFiniteState(q0, this, "integratePositions", "q0");
  detail::assertFiniteState(v, this, "integratePositions", "v");
  detail::assertFiniteState(dt, this, "integratePositions", "dt");

  const EuclideanPoint q0Static = q0;
  const Vector vStatic = v;

  const Point& point = math::integratePosition<ConfigSpaceT>(
      math::toManifoldPoint<ConfigSpaceT>(q0Static), vStatic, dt);

  result = math::toEuclideanPoint<ConfigSpaceT>(point);
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::integrateVelocities(double dt)
{
  setVelocitiesStatic(
      math::integrateVelocity<ConfigSpaceT>(
          getVelocitiesStatic(), getAccelerationsStatic(), dt));
}

//==============================================================================
template <class ConfigSpaceT>
Eigen::VectorXd GenericJoint<ConfigSpaceT>::getPositionDifferences(
    const Eigen::VectorXd& q2, const Eigen::VectorXd& q1) const
{
  if (static_cast<size_t>(q1.size()) != getNumDofs()
      || static_cast<size_t>(q2.size()) != getNumDofs()) {
    DART_ERROR(
        "q1's size [{}] or q2's size [{}] must both equal the dof [{}] for "
        "Joint [{}].",
        q1.size(),
        q2.size(),
        this->getNumDofs(),
        this->getName());
    DART_ASSERT(false);
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
  if (index >= getNumDofs()) {
    GenericJoint_REPORT_OUT_OF_RANGE(setSpringStiffness, index);
    return;
  }

  DART_WARN_IF(
      !std::isfinite(k) || k < 0.0,
      "[GenericJoint] Invalid spring stiffness ({}) set for joint [{}]. "
      "Spring stiffness must be non-negative and finite. Clamping to 0.",
      k,
      this->getName());
  if (!std::isfinite(k) || k < 0.0) {
    k = 0.0;
  }

  GenericJoint_SET_IF_DIFFERENT(mSpringStiffnesses[index], k);
}

//==============================================================================
template <class ConfigSpaceT>
double GenericJoint<ConfigSpaceT>::getSpringStiffness(size_t index) const
{
  if (index >= getNumDofs()) {
    GenericJoint_REPORT_OUT_OF_RANGE(getSpringStiffness, index);
    return 0.0;
  }

  return Base::mAspectProperties.mSpringStiffnesses[index];
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::setRestPosition(size_t index, double q0)
{
  if (index >= getNumDofs()) {
    GenericJoint_REPORT_OUT_OF_RANGE(setRestPosition, index);
    return;
  }

  GenericJoint_SET_IF_DIFFERENT(mRestPositions[index], q0);
}

//==============================================================================
template <class ConfigSpaceT>
double GenericJoint<ConfigSpaceT>::getRestPosition(size_t index) const
{
  if (index >= getNumDofs()) {
    GenericJoint_REPORT_OUT_OF_RANGE(getRestPosition, index);
    return 0.0;
  }

  return Base::mAspectProperties.mRestPositions[index];
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::setRestPositions(
    const Eigen::VectorXd& restPositions)
{
  if (static_cast<size_t>(restPositions.size()) != getNumDofs()) {
    GenericJoint_REPORT_DIM_MISMATCH(setRestPositions, restPositions);
    return;
  }

  GenericJoint_SET_IF_DIFFERENT(mRestPositions, restPositions);
}

//==============================================================================
template <class ConfigSpaceT>
Eigen::VectorXd GenericJoint<ConfigSpaceT>::getRestPositions() const
{
  return Base::mAspectProperties.mRestPositions;
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::setDampingCoefficient(size_t index, double d)
{
  if (index >= getNumDofs()) {
    GenericJoint_REPORT_OUT_OF_RANGE(setDampingCoefficient, index);
    return;
  }

  DART_WARN_IF(
      !std::isfinite(d) || d < 0.0,
      "[GenericJoint] Invalid damping coefficient ({}) set for joint [{}]. "
      "Damping must be non-negative and finite (negative damping adds energy). "
      "Clamping to 0.",
      d,
      this->getName());
  if (!std::isfinite(d) || d < 0.0) {
    d = 0.0;
  }

  GenericJoint_SET_IF_DIFFERENT(mDampingCoefficients[index], d);
}

//==============================================================================
template <class ConfigSpaceT>
double GenericJoint<ConfigSpaceT>::getDampingCoefficient(size_t index) const
{
  if (index >= getNumDofs()) {
    GenericJoint_REPORT_OUT_OF_RANGE(getDampingCoefficient, index);
    return 0.0;
  }

  return Base::mAspectProperties.mDampingCoefficients[index];
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::setDampingCoefficients(
    const Eigen::VectorXd& dampingCoefficients)
{
  if (static_cast<size_t>(dampingCoefficients.size()) != getNumDofs()) {
    GenericJoint_REPORT_DIM_MISMATCH(
        setDampingCoefficients, dampingCoefficients);
    return;
  }

  DART_WARN_IF(
      (!dampingCoefficients.array().isFinite()
       || dampingCoefficients.array() < 0.0)
          .any(),
      "[GenericJoint] Invalid damping coefficient(s) in [{}] for joint [{}]. "
      "Damping must be non-negative and finite (negative damping adds energy). "
      "Clamping to 0.",
      dampingCoefficients.transpose(),
      this->getName());
  const Eigen::VectorXd clamped = dampingCoefficients.unaryExpr(
      [](double x) { return std::isfinite(x) && x >= 0.0 ? x : 0.0; });

  GenericJoint_SET_IF_DIFFERENT(mDampingCoefficients, clamped);
}

//==============================================================================
template <class ConfigSpaceT>
Eigen::VectorXd GenericJoint<ConfigSpaceT>::getDampingCoefficients() const
{
  return Base::mAspectProperties.mDampingCoefficients;
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::setCoulombFriction(
    size_t index, double friction)
{
  if (index >= getNumDofs()) {
    GenericJoint_REPORT_OUT_OF_RANGE(setCoulombFriction, index);
    return;
  }

  DART_WARN_IF(
      !std::isfinite(friction) || friction < 0.0,
      "[GenericJoint] Invalid Coulomb friction ({}) set for joint [{}]. "
      "Friction must be non-negative and finite. Clamping to 0.",
      friction,
      this->getName());
  if (!std::isfinite(friction) || friction < 0.0) {
    friction = 0.0;
  }

  GenericJoint_SET_IF_DIFFERENT(mFrictions[index], friction);
}

//==============================================================================
template <class ConfigSpaceT>
double GenericJoint<ConfigSpaceT>::getCoulombFriction(size_t index) const
{
  if (index >= getNumDofs()) {
    GenericJoint_REPORT_OUT_OF_RANGE(getCoulombFriction, index);
    return 0.0;
  }

  return Base::mAspectProperties.mFrictions[index];
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::setFrictions(const Eigen::VectorXd& frictions)
{
  if (static_cast<size_t>(frictions.size()) != getNumDofs()) {
    GenericJoint_REPORT_DIM_MISMATCH(setFrictions, frictions);
    return;
  }

  DART_WARN_IF(
      (!frictions.array().isFinite() || frictions.array() < 0.0).any(),
      "[GenericJoint] Invalid Coulomb friction(s) in [{}] for joint [{}]. "
      "Friction must be non-negative and finite. Clamping to 0.",
      frictions.transpose(),
      this->getName());
  const Eigen::VectorXd clamped = frictions.unaryExpr(
      [](double x) { return std::isfinite(x) && x >= 0.0 ? x : 0.0; });

  GenericJoint_SET_IF_DIFFERENT(mFrictions, clamped);
}

//==============================================================================
template <class ConfigSpaceT>
Eigen::VectorXd GenericJoint<ConfigSpaceT>::getFrictions() const
{
  return Base::mAspectProperties.mFrictions;
}

//==============================================================================
template <class ConfigSpaceT>
double GenericJoint<ConfigSpaceT>::computePotentialEnergy() const
{
  // Spring energy
  Vector displacement
      = getPositionsStatic() - Base::mAspectProperties.mRestPositions;

  const double pe = 0.5
                    * displacement.dot(
                        Base::mAspectProperties.mSpringStiffnesses.cwiseProduct(
                            displacement));

  return pe;
}

//==============================================================================
template <class ConfigSpaceT>
const math::Jacobian GenericJoint<ConfigSpaceT>::getRelativeJacobian() const
{
  return getRelativeJacobianStatic();
}

//==============================================================================
template <class ConfigSpaceT>
const typename GenericJoint<ConfigSpaceT>::JacobianMatrix&
GenericJoint<ConfigSpaceT>::getRelativeJacobianStatic() const
{
  if (this->mIsRelativeJacobianDirty) {
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
const math::Jacobian GenericJoint<ConfigSpaceT>::getRelativeJacobianTimeDeriv()
    const
{
  return getRelativeJacobianTimeDerivStatic();
}

//==============================================================================
template <class ConfigSpaceT>
const typename GenericJoint<ConfigSpaceT>::JacobianMatrix&
GenericJoint<ConfigSpaceT>::getRelativeJacobianTimeDerivStatic() const
{
  if (this->mIsRelativeJacobianTimeDerivDirty) {
    this->updateRelativeJacobianTimeDeriv();
    this->mIsRelativeJacobianTimeDerivDirty = false;
  }

  return mJacobianDeriv;
}

//==============================================================================
template <class ConfigSpaceT>
GenericJoint<ConfigSpaceT>::GenericJoint(const Properties& properties)
  : mJacobian(JacobianMatrix::Zero()),
    mJacobianDeriv(JacobianMatrix::Zero()),
    mInvProjArtInertia(Matrix::Zero()),
    mInvProjArtInertiaImplicit(Matrix::Zero()),
    mTotalForce(Vector::Zero()),
    mTotalImpulse(Vector::Zero())
{
  for (auto i = 0u; i < NumDofs; ++i) {
    mDofs[i] = this->createDofPointer(i);
  }

  // Joint and GenericJoint Aspects must be created by the most derived class.
  this->mAspectState.mPositions = properties.mInitialPositions;
  this->mAspectState.mVelocities = properties.mInitialVelocities;
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::registerDofs()
{
  const SkeletonPtr& skel = this->mChildBodyNode->getSkeleton();
  for (auto i = 0u; i < NumDofs; ++i) {
    Base::mAspectProperties.mDofNames[i]
        = skel->mNameMgrForDofs.issueNewNameAndAdd(
            mDofs[i]->getName(), mDofs[i]);
  }
}

//==============================================================================
template <class ConfigSpaceT>
Eigen::Vector6d GenericJoint<ConfigSpaceT>::getBodyConstraintWrench() const
{
  DART_ASSERT(this->mChildBodyNode);
  return this->mChildBodyNode->getBodyForce()
         - this->getRelativeJacobianStatic() * this->mAspectState.mForces;
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::updateRelativeSpatialVelocity() const
{
  this->mSpatialVelocity
      = this->getRelativeJacobianStatic() * this->getVelocitiesStatic();
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::updateRelativeSpatialAcceleration() const
{
  this->mSpatialAcceleration = this->getRelativePrimaryAcceleration()
                               + this->getRelativeJacobianTimeDerivStatic()
                                     * this->getVelocitiesStatic();
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::updateRelativePrimaryAcceleration() const
{
  this->mPrimaryAcceleration
      = this->getRelativeJacobianStatic() * this->getAccelerationsStatic();
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::addVelocityTo(Eigen::Vector6d& vel)
{
  // Add joint velocity to _vel
  vel.noalias() += getRelativeJacobianStatic() * getVelocitiesStatic();

  // Verification
  DART_ASSERT(!math::isNan(vel));
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::setPartialAccelerationTo(
    Eigen::Vector6d& partialAcceleration, const Eigen::Vector6d& childVelocity)
{
  // ad(V, S * dq) + dS * dq
  partialAcceleration
      = math::ad(
            childVelocity, getRelativeJacobianStatic() * getVelocitiesStatic())
        + getRelativeJacobianTimeDerivStatic() * getVelocitiesStatic();
  // Verification
  DART_ASSERT(!math::isNan(partialAcceleration));
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::addAccelerationTo(Eigen::Vector6d& acc)
{
  // Add joint acceleration to _acc
  acc.noalias() += getRelativeJacobianStatic() * getAccelerationsStatic();

  // Verification
  DART_ASSERT(!math::isNan(acc));
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::addVelocityChangeTo(
    Eigen::Vector6d& velocityChange)
{
  // Add joint velocity change to velocityChange
  velocityChange.noalias()
      += getRelativeJacobianStatic() * this->mAspectState.mVelocityChanges;

  // Verification
  DART_ASSERT(!math::isNan(velocityChange));
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
    Eigen::Matrix6d& parentArtInertia, const Eigen::Matrix6d& childArtInertia)
{
  if (Joint::isDynamic()) {
    addChildArtInertiaToDynamic(parentArtInertia, childArtInertia);
  } else if (Joint::isKinematic()) {
    addChildArtInertiaToKinematic(parentArtInertia, childArtInertia);
  } else {
    GenericJoint_REPORT_UNSUPPORTED_ACTUATOR(
        addChildArtInertiaTo, Joint::mAspectProperties.mActuatorType);
  }
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::addChildArtInertiaToDynamic(
    Eigen::Matrix6d& parentArtInertia, const Eigen::Matrix6d& childArtInertia)
{
  // Child body's articulated inertia
  JacobianMatrix AIS = childArtInertia * getRelativeJacobianStatic();
  Eigen::Matrix6d PI = childArtInertia;
  PI.noalias() -= AIS * mInvProjArtInertia * AIS.transpose();
  if (math::isNan(PI) || math::isInf(PI)) {
    DART_WARN(
        "[GenericJoint::addChildArtInertiaToDynamic] Non-finite articulated "
        "inertia detected for joint [{}]. Skipping child inertia "
        "contribution.",
        this->getName());
    return;
  }

  // Add child body's articulated inertia to parent body's articulated inertia.
  // Note that mT should be updated.
  parentArtInertia
      += math::transformInertia(this->getRelativeTransform().inverse(), PI);
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::addChildArtInertiaToKinematic(
    Eigen::Matrix6d& parentArtInertia, const Eigen::Matrix6d& childArtInertia)
{
  // Add child body's articulated inertia to parent body's articulated inertia.
  // Note that mT should be updated.
  parentArtInertia += math::transformInertia(
      this->getRelativeTransform().inverse(), childArtInertia);
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::addChildArtInertiaImplicitTo(
    Eigen::Matrix6d& parentArtInertia, const Eigen::Matrix6d& childArtInertia)
{
  if (Joint::isDynamic()) {
    addChildArtInertiaImplicitToDynamic(parentArtInertia, childArtInertia);
  } else if (Joint::isKinematic()) {
    addChildArtInertiaImplicitToKinematic(parentArtInertia, childArtInertia);
  } else {
    GenericJoint_REPORT_UNSUPPORTED_ACTUATOR(
        addChildArtInertiaImplicitTo, Joint::mAspectProperties.mActuatorType);
  }
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::addChildArtInertiaImplicitToDynamic(
    Eigen::Matrix6d& parentArtInertia, const Eigen::Matrix6d& childArtInertia)
{
  // Child body's articulated inertia
  JacobianMatrix AIS = childArtInertia * getRelativeJacobianStatic();
  Eigen::Matrix6d PI = childArtInertia;
  PI.noalias() -= AIS * mInvProjArtInertiaImplicit * AIS.transpose();
  if (math::isNan(PI) || math::isInf(PI)) {
    DART_WARN(
        "[GenericJoint::addChildArtInertiaImplicitToDynamic] Non-finite "
        "articulated inertia detected for joint [{}]. Skipping child inertia "
        "contribution.",
        this->getName());
    return;
  }

  // Add child body's articulated inertia to parent body's articulated inertia.
  // Note that mT should be updated.
  parentArtInertia
      += math::transformInertia(this->getRelativeTransform().inverse(), PI);
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::addChildArtInertiaImplicitToKinematic(
    Eigen::Matrix6d& parentArtInertia, const Eigen::Matrix6d& childArtInertia)
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
  if (Joint::isDynamic()) {
    updateInvProjArtInertiaDynamic(artInertia);
  } else if (Joint::isKinematic()) {
    updateInvProjArtInertiaKinematic(artInertia);
  } else {
    GenericJoint_REPORT_UNSUPPORTED_ACTUATOR(
        updateInvProjArtInertia, Joint::mAspectProperties.mActuatorType);
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
  if (math::isNan(mInvProjArtInertia) || math::isInf(mInvProjArtInertia)) {
    DART_WARN(
        "[GenericJoint::updateInvProjArtInertiaDynamic] Non-finite inverse "
        "projected articulated inertia detected for joint [{}]. Setting "
        "inverse inertia to zero to avoid instability.",
        this->getName());
    mInvProjArtInertia.setZero();
  }
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
    const Eigen::Matrix6d& artInertia, double timeStep)
{
  if (Joint::isDynamic()) {
    updateInvProjArtInertiaImplicitDynamic(artInertia, timeStep);
  } else if (Joint::isKinematic()) {
    updateInvProjArtInertiaImplicitKinematic(artInertia, timeStep);
  } else {
    GenericJoint_REPORT_UNSUPPORTED_ACTUATOR(
        updateInvProjArtInertiaImplicit,
        Joint::mAspectProperties.mActuatorType);
  }
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::updateInvProjArtInertiaImplicitDynamic(
    const Eigen::Matrix6d& artInertia, double timeStep)
{
  // Projected articulated inertia
  const JacobianMatrix& Jacobian = getRelativeJacobianStatic();
  Matrix projAI = Jacobian.transpose() * artInertia * Jacobian;

  // Add additional inertia for implicit damping and spring force
  projAI += (timeStep * Base::mAspectProperties.mDampingCoefficients
             + timeStep * timeStep * Base::mAspectProperties.mSpringStiffnesses)
                .asDiagonal();

  // Inversion of projected articulated inertia
  mInvProjArtInertiaImplicit = math::inverse<ConfigSpaceT>(projAI);

  // Verification
  DART_ASSERT(!math::isNan(mInvProjArtInertiaImplicit));
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
  if (Joint::isDynamic()) {
    addChildBiasForceToDynamic(
        parentBiasForce, childArtInertia, childBiasForce, childPartialAcc);
  } else if (Joint::isKinematic()) {
    addChildBiasForceToKinematic(
        parentBiasForce, childArtInertia, childBiasForce, childPartialAcc);
  } else {
    GenericJoint_REPORT_UNSUPPORTED_ACTUATOR(
        addChildBiasForceTo, Joint::mAspectProperties.mActuatorType);
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
                       * mTotalForce);

  //    Eigen::Vector6d beta
  //        = _childBiasForce;
  //    beta.noalias() += _childArtInertia * _childPartialAcc;
  //    beta.noalias() += _childArtInertia *  mJacobian *
  //    getInvProjArtInertiaImplicit() * mTotalForce;

  // Verification
  if (math::isNan(beta) || math::isInf(beta)) {
    DART_WARN(
        "[GenericJoint::addChildBiasForceToDynamic] Non-finite bias force "
        "detected for joint [{}]. Skipping bias force contribution.",
        this->getName());
    return;
  }

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
        + childArtInertia
              * (childPartialAcc
                 + getRelativeJacobianStatic() * getAccelerationsStatic());

  //    Eigen::Vector6d beta
  //        = _childBiasForce;
  //    beta.noalias() += _childArtInertia * _childPartialAcc;
  //    beta.noalias() += _childArtInertia *  mJacobian *
  //    getInvProjArtInertiaImplicit() * mTotalForce;

  // Verification
  DART_ASSERT(!math::isNan(beta));

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
  if (Joint::isDynamic()) {
    addChildBiasImpulseToDynamic(
        parentBiasImpulse, childArtInertia, childBiasImpulse);
  } else if (Joint::isKinematic()) {
    addChildBiasImpulseToKinematic(
        parentBiasImpulse, childArtInertia, childBiasImpulse);
  } else {
    GenericJoint_REPORT_UNSUPPORTED_ACTUATOR(
        addChildBiasImpulseTo, Joint::mAspectProperties.mActuatorType);
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
  const Eigen::Vector6d beta = childBiasImpulse
                               + childArtInertia * getRelativeJacobianStatic()
                                     * getInvProjArtInertia() * mTotalImpulse;

  // Verification
  DART_ASSERT(!math::isNan(beta));

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
  parentBiasImpulse
      += math::dAdInvT(this->getRelativeTransform(), childBiasImpulse);
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::updateTotalForce(
    const Eigen::Vector6d& bodyForce, double timeStep)
{
  DART_ASSERT(timeStep > 0.0);

  switch (Joint::mAspectProperties.mActuatorType) {
    case Joint::FORCE:
      this->mAspectState.mForces = this->mAspectState.mCommands;
      updateTotalForceDynamic(bodyForce, timeStep);
      break;
    case Joint::PASSIVE:
    case Joint::SERVO:
    case Joint::MIMIC:
      this->mAspectState.mForces.setZero();
      updateTotalForceDynamic(bodyForce, timeStep);
      break;
    case Joint::ACCELERATION:
      setAccelerationsStatic(this->mAspectState.mCommands);
      updateTotalForceKinematic(bodyForce, timeStep);
      break;
    case Joint::VELOCITY:
      setAccelerationsStatic(
          (this->mAspectState.mCommands - getVelocitiesStatic()) / timeStep);
      updateTotalForceKinematic(bodyForce, timeStep);
      break;
    case Joint::LOCKED:
      setVelocitiesStatic(Vector::Zero());
      setAccelerationsStatic(Vector::Zero());
      updateTotalForceKinematic(bodyForce, timeStep);
      break;
    default:
      GenericJoint_REPORT_UNSUPPORTED_ACTUATOR(
          updateTotalForce, Joint::mAspectProperties.mActuatorType);
      break;
  }
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::updateTotalForceDynamic(
    const Eigen::Vector6d& bodyForce, double timeStep)
{
  // Spring force
  const Vector springForce
      = -Base::mAspectProperties.mSpringStiffnesses.cwiseProduct(
          getPositionsStatic() - Base::mAspectProperties.mRestPositions
          + getVelocitiesStatic() * timeStep);

  // Damping force
  const Vector dampingForce
      = -Base::mAspectProperties.mDampingCoefficients.cwiseProduct(
          getVelocitiesStatic());

  //
  mTotalForce = this->mAspectState.mForces + springForce + dampingForce
                - getRelativeJacobianStatic().transpose() * bodyForce;
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::updateTotalForceKinematic(
    const Eigen::Vector6d& /*bodyForce*/, double /*timeStep*/)
{
  // Do nothing
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::updateTotalImpulse(
    const Eigen::Vector6d& bodyImpulse)
{
  switch (Joint::mAspectProperties.mActuatorType) {
    case Joint::FORCE:
    case Joint::PASSIVE:
    case Joint::SERVO:
    case Joint::MIMIC:
      updateTotalImpulseDynamic(bodyImpulse);
      break;
    case Joint::ACCELERATION:
    case Joint::VELOCITY:
    case Joint::LOCKED:
      updateTotalImpulseKinematic(bodyImpulse);
      break;
    default:
      GenericJoint_REPORT_UNSUPPORTED_ACTUATOR(
          updateTotalImpulse, Joint::mAspectProperties.mActuatorType);
      break;
  }
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::updateTotalImpulseDynamic(
    const Eigen::Vector6d& bodyImpulse)
{
  //
  mTotalImpulse = this->mAspectState.mConstraintImpulses
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
    const Eigen::Matrix6d& artInertia, const Eigen::Vector6d& spatialAcc)
{
  switch (Joint::mAspectProperties.mActuatorType) {
    case Joint::FORCE:
    case Joint::PASSIVE:
    case Joint::SERVO:
    case Joint::MIMIC:
      updateAccelerationDynamic(artInertia, spatialAcc);
      break;
    case Joint::ACCELERATION:
    case Joint::VELOCITY:
    case Joint::LOCKED:
      updateAccelerationKinematic(artInertia, spatialAcc);
      break;
    default:
      GenericJoint_REPORT_UNSUPPORTED_ACTUATOR(
          updateAcceleration, Joint::mAspectProperties.mActuatorType);
      break;
  }
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::updateAccelerationDynamic(
    const Eigen::Matrix6d& artInertia, const Eigen::Vector6d& spatialAcc)
{
  //
  setAccelerationsStatic(
      getInvProjArtInertiaImplicit()
      * (mTotalForce
         - getRelativeJacobianStatic().transpose() * artInertia
               * math::AdInvT(this->getRelativeTransform(), spatialAcc)));

  // Verification
  const Vector accelerations = getAccelerationsStatic();
  if (math::isNan(accelerations) || math::isInf(accelerations)) {
    DART_WARN(
        "[GenericJoint::updateAccelerationDynamic] Non-finite joint "
        "accelerations detected for joint [{}]. Setting accelerations to zero "
        "to avoid instability.",
        this->getName());
    setAccelerationsStatic(Vector::Zero());
  }
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
    const Eigen::Matrix6d& artInertia, const Eigen::Vector6d& velocityChange)
{
  switch (Joint::mAspectProperties.mActuatorType) {
    case Joint::FORCE:
    case Joint::PASSIVE:
    case Joint::SERVO:
    case Joint::MIMIC:
      updateVelocityChangeDynamic(artInertia, velocityChange);
      break;
    case Joint::ACCELERATION:
    case Joint::VELOCITY:
    case Joint::LOCKED:
      updateVelocityChangeKinematic(artInertia, velocityChange);
      break;
    default:
      GenericJoint_REPORT_UNSUPPORTED_ACTUATOR(
          updateVelocityChange, Joint::mAspectProperties.mActuatorType);
      break;
  }
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::updateVelocityChangeDynamic(
    const Eigen::Matrix6d& artInertia, const Eigen::Vector6d& velocityChange)
{
  //
  this->mAspectState.mVelocityChanges
      = getInvProjArtInertia()
        * (mTotalImpulse
           - getRelativeJacobianStatic().transpose() * artInertia
                 * math::AdInvT(this->getRelativeTransform(), velocityChange));

  // Verification
  DART_ASSERT(!math::isNan(this->mAspectState.mVelocityChanges));
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
  this->mAspectState.mForces
      = getRelativeJacobianStatic().transpose() * bodyForce;

  // Implicit damping force:
  //   tau_d = -Kd * dq - Kd * h * ddq
  if (withDampingForces) {
    const typename ConfigSpaceT::Vector dampingForces
        = -Base::mAspectProperties.mDampingCoefficients.cwiseProduct(
            getVelocitiesStatic() + getAccelerationsStatic() * timeStep);
    this->mAspectState.mForces -= dampingForces;
  }

  // Implicit spring force:
  //   tau_s = -Kp * (q - q0) - Kp * h * dq - Kp * h^2 * ddq
  if (withSpringForces) {
    const typename ConfigSpaceT::Vector springForces
        = -Base::mAspectProperties.mSpringStiffnesses.cwiseProduct(
            getPositionsStatic() - Base::mAspectProperties.mRestPositions
            + getVelocitiesStatic() * timeStep
            + getAccelerationsStatic() * timeStep * timeStep);
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
  switch (Joint::mAspectProperties.mActuatorType) {
    case Joint::FORCE:
    case Joint::PASSIVE:
    case Joint::SERVO:
    case Joint::MIMIC:
      break;
    case Joint::ACCELERATION:
    case Joint::VELOCITY:
    case Joint::LOCKED:
      updateForceID(bodyForce, timeStep, withDampingForces, withSpringForces);
      break;
    default:
      GenericJoint_REPORT_UNSUPPORTED_ACTUATOR(
          updateForceFD, Joint::mAspectProperties.mActuatorType);
      break;
  }
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::updateImpulseID(
    const Eigen::Vector6d& bodyImpulse)
{
  this->mAspectState.mImpulses
      = getRelativeJacobianStatic().transpose() * bodyImpulse;
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::updateImpulseFD(
    const Eigen::Vector6d& bodyImpulse)
{
  switch (Joint::mAspectProperties.mActuatorType) {
    case Joint::FORCE:
    case Joint::PASSIVE:
    case Joint::SERVO:
    case Joint::MIMIC:
      break;
    case Joint::ACCELERATION:
    case Joint::VELOCITY:
    case Joint::LOCKED:
      updateImpulseID(bodyImpulse);
      break;
    default:
      GenericJoint_REPORT_UNSUPPORTED_ACTUATOR(
          updateImpulseFD, Joint::mAspectProperties.mActuatorType);
      break;
  }
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::updateConstrainedTerms(double timeStep)
{
  switch (Joint::mAspectProperties.mActuatorType) {
    case Joint::FORCE:
    case Joint::PASSIVE:
    case Joint::SERVO:
    case Joint::MIMIC:
      updateConstrainedTermsDynamic(timeStep);
      break;
    case Joint::ACCELERATION:
    case Joint::VELOCITY:
    case Joint::LOCKED:
      updateConstrainedTermsKinematic(timeStep);
      break;
    default:
      GenericJoint_REPORT_UNSUPPORTED_ACTUATOR(
          updateConstrainedTerms, Joint::mAspectProperties.mActuatorType);
      break;
  }
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::updateConstrainedTermsDynamic(double timeStep)
{
  const double invTimeStep = 1.0 / timeStep;

  setVelocitiesStatic(
      getVelocitiesStatic() + this->mAspectState.mVelocityChanges);
  setAccelerationsStatic(
      getAccelerationsStatic()
      + this->mAspectState.mVelocityChanges * invTimeStep);
  this->mAspectState.mForces.noalias()
      += this->mAspectState.mImpulses * invTimeStep;
  // Note: As long as this is only called from BodyNode::updateConstrainedTerms
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::updateConstrainedTermsKinematic(
    double timeStep)
{
  this->mAspectState.mForces.noalias()
      += this->mAspectState.mImpulses / timeStep;
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
  DART_ASSERT(!math::isNan(beta));

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
  DART_ASSERT(!math::isNan(beta));

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
        * (mInvM_a
           - getRelativeJacobianStatic().transpose() * artInertia
                 * math::AdInvT(this->getRelativeTransform(), spatialAcc));

  // Verification
  DART_ASSERT(!math::isNan(mInvMassMatrixSegment));

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
        * (mInvM_a
           - getRelativeJacobianStatic().transpose() * artInertia
                 * math::AdInvT(this->getRelativeTransform(), spatialAcc));

  // Verification
  DART_ASSERT(!math::isNan(mInvMassMatrixSegment));

  // Index
  size_t iStart = mDofs[0]->mIndexInTree;

  // Assign
  invMassMat.block<NumDofs, 1>(iStart, col) = mInvMassMatrixSegment;
}

//==============================================================================
template <class ConfigSpaceT>
void GenericJoint<ConfigSpaceT>::addInvMassMatrixSegmentTo(Eigen::Vector6d& acc)
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
