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

#ifndef DART_DYNAMICS_DETAIL_PLANARJOINTPROPERTIES_HPP_
#define DART_DYNAMICS_DETAIL_PLANARJOINTPROPERTIES_HPP_

#include <string>

#include "dart/dynamics/MultiDofJoint.hpp"

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
    MultiDofJoint<3>::Properties,
    PlanarJointUniqueProperties
{
  PlanarJointProperties(
      const MultiDofJoint<3>::Properties& _multiDofProperties =
          MultiDofJoint<3>::Properties(),
      const PlanarJointUniqueProperties& _planarProperties =
          PlanarJointUniqueProperties());

  virtual ~PlanarJointProperties() = default;
};

//==============================================================================
class PlanarJointAddon final :
    public common::AddonWithVersionedProperties<
        PlanarJointAddon, PlanarJointUniqueProperties, PlanarJoint,
        detail::JointPropertyUpdate<PlanarJointAddon> >
{
public:
  DART_COMMON_JOINT_ADDON_CONSTRUCTOR( PlanarJointAddon )

  void setXYPlane();
  void setYZPlane();
  void setZXPlane();
  void setArbitraryPlane(const Eigen::Vector3d& _axis1,
                         const Eigen::Vector3d& _axis2);

  DART_COMMON_GET_ADDON_PROPERTY( PlaneType, PlaneType )
  DART_COMMON_GET_ADDON_PROPERTY( Eigen::Vector3d, TransAxis1 )
  DART_COMMON_GET_ADDON_PROPERTY( Eigen::Vector3d, TransAxis2 )
  DART_COMMON_GET_ADDON_PROPERTY( Eigen::Vector3d, RotAxis )
};

//==============================================================================
using PlanarJointBase = common::AddonManagerJoiner<
    MultiDofJoint<3>, common::RequiresAddon<PlanarJointAddon> >;

} // namespace detail
} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_DETAIL_PLANARJOINTPROPERTIES_HPP_
