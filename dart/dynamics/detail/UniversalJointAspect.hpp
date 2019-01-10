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

#ifndef DART_DYNAMICS_DETAIL_UNIVERSALJOINTASPECT_HPP_
#define DART_DYNAMICS_DETAIL_UNIVERSALJOINTASPECT_HPP_

#include <string>

#include <Eigen/Dense>

#include "dart/dynamics/GenericJoint.hpp"

namespace dart {
namespace dynamics {

class UniversalJoint;

namespace detail {

//==============================================================================
struct UniversalJointUniqueProperties
{
  std::array<Eigen::Vector3d,2> mAxis;

  UniversalJointUniqueProperties(
      const Eigen::Vector3d& _axis1 = Eigen::Vector3d::UnitX(),
      const Eigen::Vector3d& _axis2 = Eigen::Vector3d::UnitY());

  virtual ~UniversalJointUniqueProperties() = default;
};

//==============================================================================
struct UniversalJointProperties :
    GenericJoint<math::R2Space>::Properties,
    UniversalJointUniqueProperties
{
  DART_DEFINE_ALIGNED_SHARED_OBJECT_CREATOR(UniversalJointProperties)

  UniversalJointProperties(
      const GenericJoint<math::R2Space>::Properties& genericJointProperties =
          GenericJoint<math::R2Space>::Properties(),
      const UniversalJointUniqueProperties& universalProperties =
          UniversalJointUniqueProperties());

  virtual ~UniversalJointProperties() = default;
};

//==============================================================================
using UniversalJointBase = common::EmbedPropertiesOnTopOf<
    UniversalJoint, UniversalJointUniqueProperties,  GenericJoint<math::R2Space> >;

} // namespace detail
} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_DETAIL_UNIVERSALJOINTASPECT_HPP_
