/*
 * Copyright (c) 2015, Georgia Tech Research Corporation
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

#ifndef DART_DYNAMICS_DETAIL_SINGLEDOFJOINTPROPERTIES_H_
#define DART_DYNAMICS_DETAIL_SINGLEDOFJOINTPROPERTIES_H_

#include "dart/common/AddonWithVersion.h"
#include "dart/dynamics/Joint.h"

namespace dart {
namespace dynamics {

// Forward declare the SingleDofJoint class
class SingleDofJoint;

namespace detail {

struct SingleDofJointUniqueProperties
{
  /// Lower limit of position
  double mPositionLowerLimit;

  /// Upper limit of position
  double mPositionUpperLimit;

  /// Initial position
  double mInitialPosition;

  /// Lower limit of velocity
  double mVelocityLowerLimit;

  /// Upper limit of velocity
  double mVelocityUpperLimit;

  /// Initial velocity
  double mInitialVelocity;

  /// Lower limit of acceleration
  double mAccelerationLowerLimit;

  /// Upper limit of acceleration
  double mAccelerationUpperLimit;

  /// Lower limit of force
  double mForceLowerLimit;

  /// Upper limit of force
  double mForceUpperLimit;

  /// Joint spring stiffness
  double mSpringStiffness;

  /// Rest position for joint spring
  double mRestPosition;

  /// Joint damping coefficient
  double mDampingCoefficient;

  /// Coulomb friction force
  double mFriction;

  /// True if the name of this Joint's DOF is not allowed to be overwritten
  bool mPreserveDofName;

  /// The name of the DegreeOfFreedom for this Joint
  std::string mDofName;

  /// Constructor
  SingleDofJointUniqueProperties(double _positionLowerLimit = -DART_DBL_INF,
                   double _positionUpperLimit =  DART_DBL_INF,
                   double _velocityLowerLimit = -DART_DBL_INF,
                   double _velocityUpperLimit =  DART_DBL_INF,
                   double _accelerationLowerLimit = -DART_DBL_INF,
                   double _accelerationUpperLimit =  DART_DBL_INF,
                   double _forceLowerLimit = -DART_DBL_INF,
                   double _forceUpperLimit =  DART_DBL_INF,
                   double _springStiffness = 0.0,
                   double _restPosition = 0.0,
                   double _dampingCoefficient = 0.0,
                   double _coulombFriction = 0.0,
                   bool _preserveDofName = false,
                   std::string _dofName = "");
  // TODO(MXG): In version 6.0, we should add mInitialPosition and
  // mInitialVelocity to the constructor arguments. For now we must wait in
  // order to avoid breaking the API

  virtual ~SingleDofJointUniqueProperties() = default;
};

//==============================================================================
struct SingleDofJointProperties :
    Joint::Properties,
    SingleDofJointUniqueProperties
{
  SingleDofJointProperties(
      const Joint::Properties& _jointProperties = Joint::Properties(),
      const SingleDofJointUniqueProperties& _singleDofProperties =
          SingleDofJointUniqueProperties());

  virtual ~SingleDofJointProperties() = default;
};

//==============================================================================
class SingleDofJointAddon final :
    public common::AddonWithVersionedProperties<
        SingleDofJointAddon, SingleDofJointUniqueProperties,
        SingleDofJoint, common::detail::NoOp>
{
public:
  DART_COMMON_ADDON_PROPERTY_CONSTRUCTOR( SingleDofJointAddon, &common::detail::NoOp )

  DART_COMMON_SET_GET_ADDON_PROPERTY(double, PositionLowerLimit)
  DART_COMMON_SET_GET_ADDON_PROPERTY(double, PositionUpperLimit)
  DART_COMMON_SET_GET_ADDON_PROPERTY(double, InitialPosition)
  DART_COMMON_SET_GET_ADDON_PROPERTY(double, VelocityLowerLimit)
  DART_COMMON_SET_GET_ADDON_PROPERTY(double, VelocityUpperLimit)
  DART_COMMON_SET_GET_ADDON_PROPERTY(double, InitialVelocity)
  DART_COMMON_SET_GET_ADDON_PROPERTY(double, AccelerationLowerLimit)
  DART_COMMON_SET_GET_ADDON_PROPERTY(double, AccelerationUpperLimit)
  DART_COMMON_SET_GET_ADDON_PROPERTY(double, ForceLowerLimit)
  DART_COMMON_SET_GET_ADDON_PROPERTY(double, ForceUpperLimit)
  DART_COMMON_SET_GET_ADDON_PROPERTY(double, SpringStiffness)
  DART_COMMON_SET_GET_ADDON_PROPERTY(double, RestPosition)
  DART_COMMON_SET_GET_ADDON_PROPERTY(double, DampingCoefficient)
  DART_COMMON_SET_GET_ADDON_PROPERTY(double, Friction)
  DART_COMMON_SET_GET_ADDON_PROPERTY(bool, PreserveDofName)

  const std::string& setDofName(const std::string& name,
                                bool preserveName = true);
  DART_COMMON_GET_ADDON_PROPERTY(std::string, DofName)

  friend class dart::dynamics::SingleDofJoint;
};

} // namespace detail
} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_DETAIL_SINGLEDOFJOINTPROPERTIES_H_
