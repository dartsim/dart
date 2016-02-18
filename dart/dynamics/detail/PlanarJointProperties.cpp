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

#include "dart/dynamics/PlanarJoint.h"

namespace dart {
namespace dynamics {
namespace detail {

//==============================================================================
PlanarJointUniqueProperties::PlanarJointUniqueProperties(PlaneType _planeType)
{
  switch(_planeType)
  {
    case PlaneType::ARBITRARY:
    case PlaneType::XY:
      setXYPlane();
      mPlaneType = _planeType; // In case the PlaneType was supposed to be arbitrary
      break;
    case PlaneType::YZ:
      setYZPlane();
       break;
    case PlaneType::ZX:
      setZXPlane();
      break;
  }
}

//==============================================================================
PlanarJointUniqueProperties::PlanarJointUniqueProperties(
    const Eigen::Vector3d& _transAxis1,
    const Eigen::Vector3d& _transAxis2)
{
  setArbitraryPlane(_transAxis1, _transAxis2);
}

//==============================================================================
PlanarJointUniqueProperties::PlanarJointUniqueProperties(
    const PlanarJointUniqueProperties& other)
{
  switch(other.mPlaneType)
  {
    case PlaneType::ARBITRARY:
      setArbitraryPlane(other.mTransAxis1, other.mTransAxis2);
      break;
    case PlaneType::XY:
      setXYPlane();
      break;
    case PlaneType::YZ:
      setYZPlane();
      break;
    case PlaneType::ZX:
      setZXPlane();
      break;
  }
}

//==============================================================================
void PlanarJointUniqueProperties::setXYPlane()
{
  mPlaneType = PlaneType::XY;
  mRotAxis   = Eigen::Vector3d::UnitZ();
  mTransAxis1 = Eigen::Vector3d::UnitX();
  mTransAxis2 = Eigen::Vector3d::UnitY();
}

//==============================================================================
void PlanarJointUniqueProperties::setYZPlane()
{
  mPlaneType = PlaneType::YZ;
  mRotAxis   = Eigen::Vector3d::UnitX();
  mTransAxis1 = Eigen::Vector3d::UnitY();
  mTransAxis2 = Eigen::Vector3d::UnitZ();
}

//==============================================================================
void PlanarJointUniqueProperties::setZXPlane()
{
  mPlaneType = PlaneType::ZX;
  mRotAxis   = Eigen::Vector3d::UnitY();
  mTransAxis1 = Eigen::Vector3d::UnitZ();
  mTransAxis2 = Eigen::Vector3d::UnitX();
}

//==============================================================================
void PlanarJointUniqueProperties::setArbitraryPlane(
    const Eigen::Vector3d& _transAxis1,
    const Eigen::Vector3d& _transAxis2)
{
  // Set plane type as arbitrary plane
  mPlaneType = PlaneType::ARBITRARY;

  // First translational axis
  mTransAxis1 = _transAxis1.normalized();

  // Second translational axis
  mTransAxis2 = _transAxis2.normalized();

  // Orthogonalize translational axese
  double dotProduct = mTransAxis1.dot(mTransAxis2);
  assert(std::abs(dotProduct) < 1.0 - 1e-6);
  if (std::abs(dotProduct) > 1e-6)
    mTransAxis2 = (mTransAxis2 - dotProduct * mTransAxis1).normalized();

  // Rotational axis
  mRotAxis = (mTransAxis1.cross(mTransAxis2)).normalized();
}

//==============================================================================
PlanarJointProperties::PlanarJointProperties(
    const MultiDofJoint<3>::Properties& _multiDofProperties,
    const PlanarJointUniqueProperties& _planarProperties)
  : MultiDofJoint<3>::Properties(_multiDofProperties),
    PlanarJointUniqueProperties(_planarProperties)
{
  // Do nothing
}

//==============================================================================
void PlanarJointAddon::setXYPlane()
{
  mProperties.setXYPlane();
  UpdateProperties(this);
  incrementSkeletonVersion();
}

//==============================================================================
void PlanarJointAddon::setYZPlane()
{
  mProperties.setYZPlane();
  UpdateProperties(this);
  incrementSkeletonVersion();
}

//==============================================================================
void PlanarJointAddon::setZXPlane()
{
  mProperties.setZXPlane();
  UpdateProperties(this);
  incrementSkeletonVersion();
}

//==============================================================================
void PlanarJointAddon::setArbitraryPlane(const Eigen::Vector3d& _axis1,
                                         const Eigen::Vector3d& _axis2)
{
  mProperties.setArbitraryPlane(_axis1, _axis2);
  UpdateProperties(this);
  incrementSkeletonVersion();
}

} // namespace detail
} // namespace dynamics
} // namespace dart
