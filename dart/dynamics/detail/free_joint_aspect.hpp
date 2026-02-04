/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
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

#ifndef DART_DYNAMICS_DETAIL_FREEJOINTASPECT_HPP_
#define DART_DYNAMICS_DETAIL_FREEJOINTASPECT_HPP_

#include <dart/dynamics/detail/joint_coordinate_chart.hpp>
#include <dart/dynamics/generic_joint.hpp>

#include <dart/export.hpp>

namespace dart {
namespace dynamics {

class FreeJoint;

namespace detail {

//==============================================================================
struct DART_API FreeJointUniqueProperties
{
  /// Coordinate chart for the rotational portion of generalized positions.
  CoordinateChart mCoordinateChart;

  /// Constructor
  FreeJointUniqueProperties(CoordinateChart _chart = CoordinateChart::EXP_MAP);

  virtual ~FreeJointUniqueProperties() = default;
};

//==============================================================================
struct DART_API FreeJointProperties : GenericJoint<math::SE3Space>::Properties,
                                      FreeJointUniqueProperties
{
  DART_DEFINE_ALIGNED_SHARED_OBJECT_CREATOR(FreeJointProperties)

  /// Composed constructor
  FreeJointProperties(
      const GenericJoint<math::SE3Space>::Properties& genericJointProperties
      = GenericJoint<math::SE3Space>::Properties(),
      const FreeJointUniqueProperties& freeJointProperties
      = FreeJointUniqueProperties());

  virtual ~FreeJointProperties() = default;
};

//==============================================================================
using FreeJointBase = common::EmbedPropertiesOnTopOf<
    FreeJoint,
    FreeJointUniqueProperties,
    GenericJoint<math::SE3Space>>;

} // namespace detail
} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_DETAIL_FREEJOINTASPECT_HPP_
