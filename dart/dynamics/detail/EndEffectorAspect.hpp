/*
 * Copyright (c) 2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Humanoid Lab, Georgia Tech Research Corporation
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

#ifndef DART_DYNAMICS_DETAIL_ENDEFFECTORASPECT_HPP_
#define DART_DYNAMICS_DETAIL_ENDEFFECTORASPECT_HPP_

#include <Eigen/Geometry>
#include "dart/dynamics/CompositeNode.hpp"
#include "dart/common/SpecializedForAspect.hpp"

namespace dart {
namespace dynamics {

class FixedJacobianNode;
class Support;

namespace detail {

//==============================================================================
struct EndEffectorProperties
{
  /// The default relative transform for the EndEffector. If the relative
  /// transform of the EndEffector is ever changed, you can call
  /// resetRelativeTransform() to return the relative transform to this one.
  Eigen::Isometry3d mDefaultTransform;

  EndEffectorProperties(
      const Eigen::Isometry3d& defaultTf = Eigen::Isometry3d::Identity());

  // To get byte-aligned Eigen vectors
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

//==============================================================================
struct SupportStateData
{
  /// Whether or not this EndEffector is currently being used to support the
  /// weight of the robot.
  bool mActive;

  inline SupportStateData(bool active = false) : mActive(active) { }

  // To get byte-aligned Eigen vectors
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

//==============================================================================
struct SupportPropertiesData
{
  /// A set of points representing the support polygon that can be provided by
  /// the EndEffector. These points must be defined relative to the EndEffector
  /// frame.
  math::SupportGeometry mGeometry;

  inline SupportPropertiesData(
      const math::SupportGeometry& geometry = math::SupportGeometry())
    : mGeometry(geometry) { }

  // To get byte-aligned Eigen vectors
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

void SupportUpdate(Support* support);

using EndEffectorCompositeBase = CompositeNode<
    common::CompositeJoiner<
        FixedJacobianNode,
        common::SpecializedForAspect<Support>
    >
>;

} // namespace detail
} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_DETAIL_ENDEFFECTORASPECT_HPP_
