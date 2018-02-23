/*
 * Copyright (c) 2011-2018, The DART development contributors
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

#ifndef DART_DYNAMICS_DETAIL_TranslationalJoint2dASPECT_HPP_
#define DART_DYNAMICS_DETAIL_TranslationalJoint2dASPECT_HPP_

#include <string>

#include <Eigen/Dense>

#include "dart/dynamics/GenericJoint.hpp"
#include "dart/dynamics/detail/PlanarJointAspect.hpp"

namespace dart {
namespace dynamics {

class TranslationalJoint2d;

namespace detail {

//==============================================================================
struct TranslationalJoint2dUniqueProperties
{
  /// Plane type
  PlaneType mPlaneType;

  /// First and second translational axis
  Eigen::Matrix<double, 3, 2> mTransAxes;

  /// Constructor for pre-defined plane types. Defaults to the XY plane if
  /// PlaneType::ARBITRARY is specified.
  explicit TranslationalJoint2dUniqueProperties(
      PlaneType planeType = PlaneType::XY);

  /// Constructor for arbitrary plane types. mPlaneType will be set to
  /// PlaneType::ARBITRARY
  TranslationalJoint2dUniqueProperties(
      const Eigen::Vector3d& transAxis1, const Eigen::Vector3d& transAxis2);

  /// Copy-constructor, customized for robustness
  TranslationalJoint2dUniqueProperties(
      const TranslationalJoint2dUniqueProperties& other);

  virtual ~TranslationalJoint2dUniqueProperties() = default;

  /// Set plane type as XY-plane
  void setXYPlane();

  /// Set plane type as YZ-plane
  void setYZPlane();

  /// Set plane type as ZX-plane
  void setZXPlane();

  /// Set plane type as arbitrary plane with two orthogonal translational axes
  void setArbitraryPlane(
      const Eigen::Vector3d& transAxis1, const Eigen::Vector3d& transAxis2);
};

//==============================================================================
struct TranslationalJoint2dProperties : GenericJoint<math::R2Space>::Properties,
                                        TranslationalJoint2dUniqueProperties
{
  DART_DEFINE_ALIGNED_SHARED_OBJECT_CREATOR(TranslationalJoint2dProperties)

  TranslationalJoint2dProperties(
      const GenericJoint<math::R2Space>::Properties& genericJointProperties
      = GenericJoint<math::R2Space>::Properties(),
      const TranslationalJoint2dUniqueProperties& universalProperties
      = TranslationalJoint2dUniqueProperties());

  virtual ~TranslationalJoint2dProperties() = default;
};

//==============================================================================
using TranslationalJoint2dBase
    = common::EmbedPropertiesOnTopOf<TranslationalJoint2d,
                                     TranslationalJoint2dUniqueProperties,
                                     GenericJoint<math::R2Space>>;

} // namespace detail
} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_DETAIL_TranslationalJoint2dASPECT_HPP_
