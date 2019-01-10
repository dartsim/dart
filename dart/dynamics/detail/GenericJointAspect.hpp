/*
 * Copyright (c) 2011-2019, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#ifndef DART_DYNAMICS_DETAIL_GenericJointASPECT_HPP_
#define DART_DYNAMICS_DETAIL_GenericJointASPECT_HPP_

#include "dart/math/Helpers.hpp"
#include "dart/dynamics/Joint.hpp"
#include "dart/common/AspectWithVersion.hpp"

namespace dart {
namespace dynamics {

// Forward declare the GenericJoint class
template <class ConfigSpaceT> class GenericJoint;

namespace detail {

//==============================================================================
template <class ConfigSpaceT>
struct GenericJointState
{
  constexpr static std::size_t NumDofs = ConfigSpaceT::NumDofs;
  using EuclideanPoint = typename ConfigSpaceT::EuclideanPoint;
  using Vector = typename ConfigSpaceT::Vector;

  /// Position
  EuclideanPoint mPositions;

  /// Generalized velocity
  Vector mVelocities;

  /// Generalized acceleration
  Vector mAccelerations;

  /// Generalized force
  Vector mForces;

  /// Command
  Vector mCommands;

  GenericJointState(
      const EuclideanPoint& positions = EuclideanPoint::Zero(),
      const Vector& velocities = Vector::Zero(),
      const Vector& accelerations = Vector::Zero(),
      const Vector& forces = Vector::Zero(),
      const Vector& commands = Vector::Zero());

  virtual ~GenericJointState() = default;

  // To get byte-aligned Eigen vectors
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

//==============================================================================
template <class ConfigSpaceT>
struct GenericJointUniqueProperties
{
  constexpr static std::size_t NumDofs = ConfigSpaceT::NumDofs;
  using EuclideanPoint = typename ConfigSpaceT::EuclideanPoint;
  using Vector = typename ConfigSpaceT::Vector;
  using BoolArray = std::array<bool, NumDofs>;
  using StringArray = std::array<std::string, NumDofs>;

  /// Lower limit of position
  EuclideanPoint mPositionLowerLimits;

  /// Upper limit of position
  EuclideanPoint mPositionUpperLimits;

  /// Initial positions
  EuclideanPoint mInitialPositions;

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
  EuclideanPoint mRestPositions;

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
  GenericJointUniqueProperties(
      const EuclideanPoint& positionLowerLimits = EuclideanPoint::Constant(-math::constantsd::inf()),
      const EuclideanPoint& positionUpperLimits = EuclideanPoint::Constant( math::constantsd::inf()),
      const EuclideanPoint& initialPositions = EuclideanPoint::Zero(),
      const Vector& velocityLowerLimits = Vector::Constant(-math::constantsd::inf()),
      const Vector& velocityUpperLimits = Vector::Constant( math::constantsd::inf()),
      const Vector& initialVelocities = Vector::Zero(),
      const Vector& accelerationLowerLimits = Vector::Constant(-math::constantsd::inf()),
      const Vector& accelerationUpperLimits = Vector::Constant( math::constantsd::inf()),
      const Vector& forceLowerLimits = Vector::Constant(-math::constantsd::inf()),
      const Vector& forceUpperLimits = Vector::Constant( math::constantsd::inf()),
      const Vector& springStiffness = Vector::Zero(),
      const EuclideanPoint& restPosition = EuclideanPoint::Zero(),
      const Vector& dampingCoefficient = Vector::Zero(),
      const Vector& coulombFrictions = Vector::Zero());

  /// Copy constructor
  // Note: we only need this because VS2013 lacks full support for std::array
  // Once std::array is properly supported, this should be removed.
  GenericJointUniqueProperties(const GenericJointUniqueProperties& other);

  virtual ~GenericJointUniqueProperties() = default;

public:
  // To get byte-aligned Eigen vectors
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

//==============================================================================
template <class ConfigSpaceT>
struct GenericJointProperties :
    Joint::Properties,
    GenericJointUniqueProperties<ConfigSpaceT>
{
  GenericJointProperties(
      const Joint::Properties& jointProperties = Joint::Properties(),
      const GenericJointUniqueProperties<ConfigSpaceT>& genericProperties =
          GenericJointUniqueProperties<ConfigSpaceT>());

  virtual ~GenericJointProperties() = default;

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
template <class ConfigSpaceT>
constexpr std::size_t GenericJointState<ConfigSpaceT>::NumDofs;

template <class ConfigSpaceT>
constexpr std::size_t GenericJointUniqueProperties<ConfigSpaceT>::NumDofs;

//==============================================================================
template <class ConfigSpaceT>
GenericJointState<ConfigSpaceT>::GenericJointState(
    const EuclideanPoint& positions,
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
template <class ConfigSpaceT>
GenericJointUniqueProperties<ConfigSpaceT>::GenericJointUniqueProperties(
    const EuclideanPoint& positionLowerLimits,
    const EuclideanPoint& positionUpperLimits,
    const EuclideanPoint& initialPositions,
    const Vector& velocityLowerLimits,
    const Vector& velocityUpperLimits,
    const Vector& initialVelocities,
    const Vector& accelerationLowerLimits,
    const Vector& accelerationUpperLimits,
    const Vector& forceLowerLimits,
    const Vector& forceUpperLimits,
    const Vector& springStiffness,
    const EuclideanPoint& restPosition,
    const Vector& dampingCoefficient,
    const Vector& coulombFrictions)
  : mPositionLowerLimits(positionLowerLimits),
    mPositionUpperLimits(positionUpperLimits),
    mInitialPositions(initialPositions),
    mVelocityLowerLimits(velocityLowerLimits),
    mVelocityUpperLimits(velocityUpperLimits),
    mInitialVelocities(initialVelocities),
    mAccelerationLowerLimits(accelerationLowerLimits),
    mAccelerationUpperLimits(accelerationUpperLimits),
    mForceLowerLimits(forceLowerLimits),
    mForceUpperLimits(forceUpperLimits),
    mSpringStiffnesses(springStiffness),
    mRestPositions(restPosition),
    mDampingCoefficients(dampingCoefficient),
    mFrictions(coulombFrictions)
{
  for (auto i = 0u; i < NumDofs; ++i)
  {
    mPreserveDofNames[i] = false;
    mDofNames[i] = std::string();
  }
}

//==============================================================================
template <class ConfigSpaceT>
GenericJointUniqueProperties<ConfigSpaceT>::GenericJointUniqueProperties(
    const GenericJointUniqueProperties& _other)
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
  for (auto i = 0u; i < NumDofs; ++i)
  {
    mPreserveDofNames[i] = _other.mPreserveDofNames[i];
    mDofNames[i] = _other.mDofNames[i];
  }
}

//==============================================================================
template <class ConfigSpaceT>
GenericJointProperties<ConfigSpaceT>::GenericJointProperties(
    const Joint::Properties& jointProperties,
    const GenericJointUniqueProperties<ConfigSpaceT>& genericProperties)
  : Joint::Properties(jointProperties),
    GenericJointUniqueProperties<ConfigSpaceT>(genericProperties)
{
  // Do nothing
}

//==============================================================================
template <class Derived, class ConfigSpaceT>
using GenericJointBase = common::EmbedStateAndPropertiesOnTopOf<
    Derived,
    GenericJointState<ConfigSpaceT>,
    GenericJointUniqueProperties<ConfigSpaceT>,
    Joint>;

} // namespace detail

} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_DETAIL_GenericJointASPECT_HPP_
