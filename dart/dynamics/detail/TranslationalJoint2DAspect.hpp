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

#ifndef DART_DYNAMICS_DETAIL_TRANSLATIONALJOINT2DASPECT_HPP_
#define DART_DYNAMICS_DETAIL_TRANSLATIONALJOINT2DASPECT_HPP_

#include <string>

#include <Eigen/Dense>

#include "dart/dynamics/GenericJoint.hpp"
#include "dart/dynamics/detail/PlanarJointAspect.hpp"

namespace dart {
namespace dynamics {

class TranslationalJoint2D;

namespace detail {

//==============================================================================
class TranslationalJoint2DUniqueProperties
{
public:
  /// Constructor for pre-defined plane types. Defaults to the XY plane if
  /// PlaneType::ARBITRARY is specified.
  explicit TranslationalJoint2DUniqueProperties(
      PlaneType planeType = PlaneType::XY);

  /// Constructor for arbitrary plane types. mPlaneType will be set to
  /// PlaneType::ARBITRARY
  explicit TranslationalJoint2DUniqueProperties(
      const Eigen::Matrix<double, 3, 2>& transAxes);

  /// Constructor for arbitrary plane types. mPlaneType will be set to
  /// PlaneType::ARBITRARY
  TranslationalJoint2DUniqueProperties(
      const Eigen::Vector3d& transAxis1, const Eigen::Vector3d& transAxis2);

  /// Copy-constructor, customized for robustness
  TranslationalJoint2DUniqueProperties(
      const TranslationalJoint2DUniqueProperties& other);

  virtual ~TranslationalJoint2DUniqueProperties() = default;

  /// Sets plane type as XY-plane
  void setXYPlane();

  /// Sets plane type as YZ-plane
  void setYZPlane();

  /// Sets plane type as ZX-plane
  void setZXPlane();

  /// Sets plane type as arbitrary plane with two orthogonal translational axes
  void setArbitraryPlane(const Eigen::Matrix<double, 3, 2>& transAxes);

  /// Sets plane type as arbitrary plane with two orthogonal translational axes
  void setArbitraryPlane(
      const Eigen::Vector3d& transAxis1, const Eigen::Vector3d& transAxis2);

  /// Returns first and second translational axes
  const Eigen::Matrix<double, 3, 2>& getTranslationalAxes() const;

  /// Returns first translational axis
  Eigen::Vector3d getTranslationalAxis1() const;

  /// Returns second translational axis
  Eigen::Vector3d getTranslationalAxis2() const;

  /// Returns plane type
  PlaneType getPlaneType() const;

private:
  /// Plane type
  PlaneType mPlaneType;

  /// First and second translational axes
  Eigen::Matrix<double, 3, 2> mTransAxes;
};

//==============================================================================
struct TranslationalJoint2DProperties : GenericJoint<math::R2Space>::Properties,
                                        TranslationalJoint2DUniqueProperties
{
  DART_DEFINE_ALIGNED_SHARED_OBJECT_CREATOR(TranslationalJoint2DProperties)

  TranslationalJoint2DProperties(
      const GenericJoint<math::R2Space>::Properties& genericJointProperties
      = GenericJoint<math::R2Space>::Properties(),
      const TranslationalJoint2DUniqueProperties& universalProperties
      = TranslationalJoint2DUniqueProperties());

  virtual ~TranslationalJoint2DProperties() = default;
};

//==============================================================================
using TranslationalJoint2DBase = common::EmbedPropertiesOnTopOf<
    TranslationalJoint2D,
    TranslationalJoint2DUniqueProperties,
    GenericJoint<math::R2Space>>;

} // namespace detail
} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_DETAIL_TRANSLATIONALJOINT2DASPECT_HPP_
