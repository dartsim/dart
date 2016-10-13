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

#ifndef DART_DYNAMICS_DETAIL_PLANARJOINTASPECT_HPP_
#define DART_DYNAMICS_DETAIL_PLANARJOINTASPECT_HPP_

#include <string>

#include "dart/dynamics/GenericJoint.hpp"

namespace dart {
namespace dynamics {

class PlanarJoint;

namespace detail {

//==============================================================================
/// Plane type
enum class PlaneType : int
{
  XY,
  YZ,
  ZX,
  ARBITRARY
};

//==============================================================================
/// Properties that are unique to PlanarJoints. Note that the mPlaneType
/// member has greater authority than the mTransAxis1 and mTransAxis2 members.
/// When copying properties into a PlanarJoint, it will first defer to
/// mPlaneType. If mPlaneType is PlaneType::ARBITRARY, then and only then will
/// it use mTransAxis1 and mTransAxis2. mRotAxis has no authority; it will
/// always be recomputed from mTransAxis1 and mTransAxis2 when copying it into a
/// PlanarJoint
struct PlanarJointUniqueProperties
{
  /// Plane type
  PlaneType mPlaneType;

  /// First translational axis
  Eigen::Vector3d mTransAxis1;

  /// Second translational axis
  Eigen::Vector3d mTransAxis2;

  /// Rotational axis
  Eigen::Vector3d mRotAxis;

  /// Constructor for pre-defined plane types. Defaults to the XY plane if
  /// PlaneType::ARBITRARY is specified.
  PlanarJointUniqueProperties(PlaneType _planeType = PlaneType::XY);

  /// Constructor for arbitrary plane types. mPlaneType will be set to
  /// PlaneType::ARBITRARY
  PlanarJointUniqueProperties(const Eigen::Vector3d& _transAxis1,
                   const Eigen::Vector3d& _transAxis2);

  /// Copy-constructor, customized for robustness
  PlanarJointUniqueProperties(const PlanarJointUniqueProperties& other);

  virtual ~PlanarJointUniqueProperties() = default;

  /// Set plane type as XY-plane
  void setXYPlane();

  /// Set plane type as YZ-plane
  void setYZPlane();

  /// Set plane type as ZX-plane
  void setZXPlane();

  /// Set plane type as arbitrary plane with two orthogonal translational axes
  void setArbitraryPlane(const Eigen::Vector3d& _transAxis1,
                         const Eigen::Vector3d& _transAxis2);
};

//==============================================================================
struct PlanarJointProperties :
    GenericJoint<math::R3Space>::Properties,
    PlanarJointUniqueProperties
{
  PlanarJointProperties(
      const GenericJoint<math::R3Space>::Properties& genericJointProperties =
          GenericJoint<math::R3Space>::Properties(),
      const PlanarJointUniqueProperties& planarProperties =
          PlanarJointUniqueProperties());

  virtual ~PlanarJointProperties() = default;
};

//==============================================================================
using PlanarJointBase = common::EmbedPropertiesOnTopOf<
    PlanarJoint, PlanarJointUniqueProperties, GenericJoint<math::R3Space> >;

} // namespace detail
} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_DETAIL_PLANARJOINTASPECT_HPP_
