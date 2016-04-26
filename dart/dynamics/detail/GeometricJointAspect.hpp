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

#ifndef DART_DYNAMICS_DETAIL_GEOMETRICJOINTASPECT_HPP_
#define DART_DYNAMICS_DETAIL_GEOMETRICJOINTASPECT_HPP_

#include "dart/math/Helpers.hpp"
#include "dart/dynamics/Joint.hpp"
#include "dart/common/AspectWithVersion.hpp"

namespace dart {
namespace dynamics {

// Forward declare the GeometricJoint class
template <class ConfigSpaceT> class GeometricJoint;

namespace detail {

//==============================================================================
template <class ConfigSpaceT>
struct GeometricJointState
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

  GeometricJointState(
      const EuclideanPoint& positions = EuclideanPoint::Zero(),
      const Vector& velocities = Vector::Zero(),
      const Vector& accelerations = Vector::Zero(),
      const Vector& forces = Vector::Zero(),
      const Vector& commands = Vector::Zero());

  virtual ~GeometricJointState() = default;

  // To get byte-aligned Eigen vectors
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

//==============================================================================
template <class ConfigSpaceT>
struct GeometricJointUniqueProperties
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
  GeometricJointUniqueProperties(
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
  GeometricJointUniqueProperties(const GeometricJointUniqueProperties& other);

  virtual ~GeometricJointUniqueProperties() = default;

public:
  // To get byte-aligned Eigen vectors
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

//==============================================================================
template <class ConfigSpaceT>
struct GeometricJointProperties :
    Joint::Properties,
    GeometricJointUniqueProperties<ConfigSpaceT>
{
  GeometricJointProperties(
      const Joint::Properties& jointProperties = Joint::Properties(),
      const GeometricJointUniqueProperties<ConfigSpaceT>& genericProperties =
          GeometricJointUniqueProperties<ConfigSpaceT>());

  virtual ~GeometricJointProperties() = default;

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
constexpr std::size_t GeometricJointState<ConfigSpaceT>::NumDofs;

template <class ConfigSpaceT>
constexpr std::size_t GeometricJointUniqueProperties<ConfigSpaceT>::NumDofs;

//==============================================================================
template <class ConfigSpaceT>
GeometricJointState<ConfigSpaceT>::GeometricJointState(
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
GeometricJointUniqueProperties<ConfigSpaceT>::GeometricJointUniqueProperties(
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
GeometricJointUniqueProperties<ConfigSpaceT>::GeometricJointUniqueProperties(
    const GeometricJointUniqueProperties& _other)
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
GeometricJointProperties<ConfigSpaceT>::GeometricJointProperties(
    const Joint::Properties& jointProperties,
    const GeometricJointUniqueProperties<ConfigSpaceT>& genericProperties)
  : Joint::Properties(jointProperties),
    GeometricJointUniqueProperties<ConfigSpaceT>(genericProperties)
{
  // Do nothing
}

//==============================================================================
template <class Derived, class ConfigSpaceT>
using GeometricJointBase = common::EmbedStateAndPropertiesOnTopOf<
    Derived,
    GeometricJointState<ConfigSpaceT>,
    GeometricJointUniqueProperties<ConfigSpaceT>,
    Joint>;

} // namespace detail

} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_DETAIL_GEOMETRICJOINTASPECT_HPP_
