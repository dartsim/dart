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

#ifndef DART_DYNAMICS_DETAIL_MULTIDOFJOINTASPECT_H_
#define DART_DYNAMICS_DETAIL_MULTIDOFJOINTASPECT_H_

#include "dart/math/Helpers.h"
#include "dart/dynamics/Joint.h"
#include "dart/common/AspectWithVersion.h"

namespace dart {
namespace dynamics {

// Forward declare the MultiDofJoint class
template <size_t DOF> class MultiDofJoint;

namespace detail {

//==============================================================================
template <size_t DOF>
struct MultiDofJointState
{
  constexpr static size_t NumDofs = DOF;
  using Vector = Eigen::Matrix<double, DOF, 1>;

  /// Position
  Vector mPositions;

  /// Generalized velocity
  Vector mVelocities;

  /// Generalized acceleration
  Vector mAccelerations;

  /// Generalized force
  Vector mForces;

  /// Command
  Vector mCommands;

  MultiDofJointState(
      const Vector& positions = Vector::Zero(),
      const Vector& velocities = Vector::Zero(),
      const Vector& accelerations = Vector::Zero(),
      const Vector& forces = Vector::Zero(),
      const Vector& commands = Vector::Zero());

  virtual ~MultiDofJointState() = default;

  // To get byte-aligned Eigen vectors
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

//==============================================================================
template <size_t DOF>
struct MultiDofJointUniqueProperties
{
  constexpr static size_t NumDofs = DOF;
  using Vector = Eigen::Matrix<double, DOF, 1>;
  using BoolArray = std::array<bool, DOF>;
  using StringArray = std::array<std::string, DOF>;

  /// Lower limit of position
  Vector mPositionLowerLimits;

  /// Upper limit of position
  Vector mPositionUpperLimits;

  /// Initial positions
  Vector mInitialPositions;

  /// Min value allowed.
  Vector mVelocityLowerLimits;

  /// Max value allowed.
  Vector mVelocityUpperLimits;

  /// Initial velocities
  Vector mInitialVelocities;

  /// Min value allowed.
  Vector mAccelerationLowerLimits;

  /// upper limit of generalized acceleration
  Vector mAccelerationUpperLimits;

  /// Min value allowed.
  Vector mForceLowerLimits;

  /// Max value allowed.
  Vector mForceUpperLimits;

  /// Joint spring stiffness
  Vector mSpringStiffnesses;

  /// Rest joint position for joint spring
  Vector mRestPositions;

  /// Joint damping coefficient
  Vector mDampingCoefficients;

  /// Joint Coulomb friction
  Vector mFrictions;

  /// True if the name of the corresponding DOF is not allowed to be
  /// overwritten
  BoolArray mPreserveDofNames;

  /// The name of the DegreesOfFreedom for this Joint
  StringArray mDofNames;

  /// Default constructor
  MultiDofJointUniqueProperties(
      const Vector& _positionLowerLimits = Vector::Constant(-math::constantsd::inf()),
      const Vector& _positionUpperLimits = Vector::Constant( math::constantsd::inf()),
      const Vector& _velocityLowerLimits = Vector::Constant(-math::constantsd::inf()),
      const Vector& _velocityUpperLimits = Vector::Constant( math::constantsd::inf()),
      const Vector& _accelerationLowerLimits = Vector::Constant(-math::constantsd::inf()),
      const Vector& _accelerationUpperLimits = Vector::Constant( math::constantsd::inf()),
      const Vector& _forceLowerLimits = Vector::Constant(-math::constantsd::inf()),
      const Vector& _forceUpperLimits = Vector::Constant( math::constantsd::inf()),
      const Vector& _springStiffness = Vector::Constant(0.0),
      const Vector& _restPosition = Vector::Constant(0.0),
      const Vector& _dampingCoefficient = Vector::Constant(0.0),
      const Vector& _coulombFrictions = Vector::Constant(0.0));
  // TODO(MXG): In version 6.0, we should add mInitialPositions and
  // mInitialVelocities to the constructor arguments. For now we must wait in
  // order to avoid breaking the API.

  /// Copy constructor
  // Note: we only need this because VS2013 lacks full support for std::array
  // Once std::array is properly supported, this should be removed.
  MultiDofJointUniqueProperties(const MultiDofJointUniqueProperties& _other);

  virtual ~MultiDofJointUniqueProperties() = default;

public:
  // To get byte-aligned Eigen vectors
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

//==============================================================================
template <size_t DOF>
struct MultiDofJointProperties :
    Joint::Properties,
    MultiDofJointUniqueProperties<DOF>
{
  MultiDofJointProperties(
      const Joint::Properties& _jointProperties = Joint::Properties(),
      const MultiDofJointUniqueProperties<DOF>& _multiDofProperties =
          MultiDofJointUniqueProperties<DOF>());

  virtual ~MultiDofJointProperties() = default;

public:
  // To get byte-aligned Eigen vectors
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

//==============================================================================
//
// These namespace-level definitions are required to enable ODR-use of static
// constexpr member variables.
//
// See this StackOverflow answer: http://stackoverflow.com/a/14396189/111426
//
template <size_t DOF>
constexpr size_t MultiDofJointState<DOF>::NumDofs;

template <size_t DOF>
constexpr size_t MultiDofJointUniqueProperties<DOF>::NumDofs;

//==============================================================================
template <size_t DOF>
MultiDofJointState<DOF>::MultiDofJointState(
    const Vector& positions,
    const Vector& velocities,
    const Vector& accelerations,
    const Vector& forces,
    const Vector& commands)
  : mPositions(positions),
    mVelocities(velocities),
    mAccelerations(accelerations),
    mForces(forces),
    mCommands(commands)
{
  // Do nothing
}

//==============================================================================
template <size_t DOF>
MultiDofJointUniqueProperties<DOF>::MultiDofJointUniqueProperties(
    const Vector& _positionLowerLimits,
    const Vector& _positionUpperLimits,
    const Vector& _velocityLowerLimits,
    const Vector& _velocityUpperLimits,
    const Vector& _accelerationLowerLimits,
    const Vector& _accelerationUpperLimits,
    const Vector& _forceLowerLimits,
    const Vector& _forceUpperLimits,
    const Vector& _springStiffness,
    const Vector& _restPosition,
    const Vector& _dampingCoefficient,
    const Vector& _coulombFrictions)
  : mPositionLowerLimits(_positionLowerLimits),
    mPositionUpperLimits(_positionUpperLimits),
    mInitialPositions(Vector::Zero()),
    mVelocityLowerLimits(_velocityLowerLimits),
    mVelocityUpperLimits(_velocityUpperLimits),
    mInitialVelocities(Vector::Zero()),
    mAccelerationLowerLimits(_accelerationLowerLimits),
    mAccelerationUpperLimits(_accelerationUpperLimits),
    mForceLowerLimits(_forceLowerLimits),
    mForceUpperLimits(_forceUpperLimits),
    mSpringStiffnesses(_springStiffness),
    mRestPositions(_restPosition),
    mDampingCoefficients(_dampingCoefficient),
    mFrictions(_coulombFrictions)
{
  for (size_t i = 0; i < DOF; ++i)
  {
    mPreserveDofNames[i] = false;
    mDofNames[i] = std::string();
  }
}

//==============================================================================
template <size_t DOF>
MultiDofJointUniqueProperties<DOF>::MultiDofJointUniqueProperties(
    const MultiDofJointUniqueProperties& _other)
  : mPositionLowerLimits(_other.mPositionLowerLimits),
    mPositionUpperLimits(_other.mPositionUpperLimits),
    mInitialPositions(_other.mInitialPositions),
    mVelocityLowerLimits(_other.mVelocityLowerLimits),
    mVelocityUpperLimits(_other.mVelocityUpperLimits),
    mInitialVelocities(_other.mInitialVelocities),
    mAccelerationLowerLimits(_other.mAccelerationLowerLimits),
    mAccelerationUpperLimits(_other.mAccelerationUpperLimits),
    mForceLowerLimits(_other.mForceLowerLimits),
    mForceUpperLimits(_other.mForceUpperLimits),
    mSpringStiffnesses(_other.mSpringStiffnesses),
    mRestPositions(_other.mRestPositions),
    mDampingCoefficients(_other.mDampingCoefficients),
    mFrictions(_other.mFrictions)
{
  for (size_t i = 0; i < DOF; ++i)
  {
    mPreserveDofNames[i] = _other.mPreserveDofNames[i];
    mDofNames[i] = _other.mDofNames[i];
  }
}

//==============================================================================
template <size_t DOF>
MultiDofJointProperties<DOF>::MultiDofJointProperties(
    const Joint::Properties& _jointProperties,
    const MultiDofJointUniqueProperties<DOF>& _multiDofProperties)
  : Joint::Properties(_jointProperties),
    MultiDofJointUniqueProperties<DOF>(_multiDofProperties)
{
  // Do nothing
}

template <class Derived, size_t DOF>
using MultiDofJointBase = common::EmbedStateAndPropertiesOnTopOf<
    Derived, MultiDofJointState<DOF>, MultiDofJointUniqueProperties<DOF>, Joint>;

} // namespace detail
} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_DETAIL_MULTIDOFJOINTASPECT_H_

