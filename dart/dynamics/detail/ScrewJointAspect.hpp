/*
 * Copyright (c) 2011-2025, The DART development contributors
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

#ifndef DART_DYNAMICS_DETAIL_SCREWJOINTASPECT_HPP_
#define DART_DYNAMICS_DETAIL_SCREWJOINTASPECT_HPP_

#include <dart/dynamics/GenericJoint.hpp>

#include <Eigen/Dense>

#include <string>

namespace dart {
namespace dynamics {

class ScrewJoint;

namespace detail {

//==============================================================================
struct ScrewJointUniqueProperties
{
  /// Rotational axis
  Eigen::Vector3d mAxis;

  /// Translational pitch
  double mPitch;

  ScrewJointUniqueProperties(
      const Eigen::Vector3d& _axis = Eigen::Vector3d::UnitZ(),
      double _pitch = 0.1);

  virtual ~ScrewJointUniqueProperties() = default;
};

//==============================================================================
struct ScrewJointProperties : GenericJoint<math::R1Space>::Properties,
                              ScrewJointUniqueProperties
{
  DART_DEFINE_ALIGNED_SHARED_OBJECT_CREATOR(ScrewJointProperties)

  ScrewJointProperties(
      const GenericJoint<math::R1Space>::Properties& genericJointProperties
      = GenericJoint<math::R1Space>::Properties(),
      const ScrewJointUniqueProperties& screwProperties
      = ScrewJointUniqueProperties());

  virtual ~ScrewJointProperties() = default;
};

//==============================================================================
using ScrewJointBase = common::EmbedPropertiesOnTopOf<
    ScrewJoint,
    ScrewJointUniqueProperties,
    GenericJoint<math::R1Space>>;

} // namespace detail
} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_DETAIL_SCREWJOINTASPECT_HPP_
