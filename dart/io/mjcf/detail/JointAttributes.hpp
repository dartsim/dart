/*
 * Copyright (c) 2011-2021, The DART development contributors
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

#ifndef DART_IO_MJCF_DETAIL_JOINTATTRIBUTES_HPP_
#define DART_IO_MJCF_DETAIL_JOINTATTRIBUTES_HPP_

#include <Eigen/Core>
#include <tinyxml2.h>

#include "dart/common/Optional.hpp"
#include "dart/io/mjcf/detail/Error.hpp"
#include "dart/io/mjcf/detail/Types.hpp"

namespace dart {
namespace io {
namespace MjcfParser {
namespace detail {

struct JointAttributes final
{
  common::optional<std::string> mName;

  JointType mType{JointType::HINGE};

  int mGroup{0};

  /// Position of the joint, specified in local or global coordinates as
  /// determined by the coordinate attribute of compiler. For free joints this
  /// attribute is ignored.
  Eigen::Vector3d mPos{Eigen::Vector3d::Zero()};

  /// This attribute specifies the axis of rotation for hinge joints and the
  /// direction of translation for slide joints. It is ignored for free and ball
  /// joints.
  Eigen::Vector3d mAxis{Eigen::Vector3d::UnitZ()};

  Eigen::Vector2d mSpringDamper{Eigen::Vector2d::Zero()};

  /// This attribute specifies if the joint has limits.
  bool mLimited{false};

  double mStiffness{0};

  /// The joint limits.
  Eigen::Vector2d mRange{Eigen::Vector2d::Zero()};

  double mMargin{0};

  double mRef{0};

  double mSpringRef{0};

  double mArmature{0};

  double mDamping{0};

  double mFrictionLoss{0};

  Eigen::VectorXd mUser;
};

Errors appendJointAttributes(
    JointAttributes& attributes, tinyxml2::XMLElement* element);

} // namespace detail
} // namespace MjcfParser
} // namespace io
} // namespace dart

#endif // #ifndef DART_IO_MJCF_DETAIL_JOINTATTRIBUTES_HPP_
