/*
 * Copyright (c) 2011-2017, The DART development contributors
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

#include "dart/dynamics/TranslationalJoint2d.hpp"

namespace dart {
namespace dynamics {
namespace detail {

//==============================================================================
TranslationalJoint2dUniqueProperties::TranslationalJoint2dUniqueProperties(
    PlaneType planeType)
{
  switch (planeType)
  {
    case PlaneType::ARBITRARY:
    case PlaneType::XY:
      setXYPlane();
      mPlaneType
          = planeType; // In case the PlaneType was supposed to be arbitrary
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
TranslationalJoint2dUniqueProperties::TranslationalJoint2dUniqueProperties(
    const Eigen::Vector3d& transAxis1, const Eigen::Vector3d& transAxis2)
{
  setArbitraryPlane(transAxis1, transAxis2);
}

//==============================================================================
TranslationalJoint2dUniqueProperties::TranslationalJoint2dUniqueProperties(
    const TranslationalJoint2dUniqueProperties& other)
{
  switch (other.mPlaneType)
  {
    case PlaneType::ARBITRARY:
      setArbitraryPlane(other.mTransAxes.col(0), other.mTransAxes.col(1));
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
void TranslationalJoint2dUniqueProperties::setXYPlane()
{
  mPlaneType = PlaneType::XY;
  mTransAxes.col(0) = Eigen::Vector3d::UnitX();
  mTransAxes.col(1) = Eigen::Vector3d::UnitY();
}

//==============================================================================
void TranslationalJoint2dUniqueProperties::setYZPlane()
{
  mPlaneType = PlaneType::YZ;
  mTransAxes.col(0) = Eigen::Vector3d::UnitY();
  mTransAxes.col(1) = Eigen::Vector3d::UnitZ();
}

//==============================================================================
void TranslationalJoint2dUniqueProperties::setZXPlane()
{
  mPlaneType = PlaneType::ZX;
  mTransAxes.col(0) = Eigen::Vector3d::UnitZ();
  mTransAxes.col(1) = Eigen::Vector3d::UnitX();
}

//==============================================================================
void TranslationalJoint2dUniqueProperties::setArbitraryPlane(
    const Eigen::Vector3d& transAxis1, const Eigen::Vector3d& transAxis2)
{
  // Set plane type as arbitrary plane
  mPlaneType = PlaneType::ARBITRARY;

  // First translational axis
  mTransAxes.col(0) = transAxis1.normalized();

  // Second translational axis
  mTransAxes.col(1) = transAxis2.normalized();

  // Orthogonalize translational axese
  double dotProduct = mTransAxes.col(0).dot(mTransAxes.col(1));
  assert(std::abs(dotProduct) < 1.0 - 1e-6);
  if (std::abs(dotProduct) > 1e-6)
    mTransAxes.col(1)
        = (mTransAxes.col(1) - dotProduct * mTransAxes.col(0)).normalized();
}

//==============================================================================
TranslationalJoint2dProperties::TranslationalJoint2dProperties(
    const GenericJoint<math::R2Space>::Properties& genericJointProperties,
    const TranslationalJoint2dUniqueProperties& universalProperties)
  : GenericJoint<math::R2Space>::Properties(genericJointProperties),
    TranslationalJoint2dUniqueProperties(universalProperties)
{
  // Do nothing
}

} // namespace detail
} // namespace dynamics
} // namespace dart
