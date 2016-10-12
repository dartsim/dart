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

#include "dart/dynamics/PlanarJoint.hpp"

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
  mPlaneType  = PlaneType::XY;
  mRotAxis    = Eigen::Vector3d::UnitZ();
  mTransAxis1 = Eigen::Vector3d::UnitX();
  mTransAxis2 = Eigen::Vector3d::UnitY();
}

//==============================================================================
void PlanarJointUniqueProperties::setYZPlane()
{
  mPlaneType  = PlaneType::YZ;
  mRotAxis    = Eigen::Vector3d::UnitX();
  mTransAxis1 = Eigen::Vector3d::UnitY();
  mTransAxis2 = Eigen::Vector3d::UnitZ();
}

//==============================================================================
void PlanarJointUniqueProperties::setZXPlane()
{
  mPlaneType  = PlaneType::ZX;
  mRotAxis    = Eigen::Vector3d::UnitY();
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
    const GenericJoint<math::R3Space>::Properties& genericJointProperties,
    const PlanarJointUniqueProperties& planarProperties)
  : GenericJoint<math::R3Space>::Properties(genericJointProperties),
    PlanarJointUniqueProperties(planarProperties)
{
  // Do nothing
}

} // namespace detail
} // namespace dynamics
} // namespace dart
