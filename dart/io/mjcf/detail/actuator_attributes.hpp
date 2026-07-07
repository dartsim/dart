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

#ifndef DART_IO_MJCF_DETAIL_ACTUATORATTRIBUTES_HPP_
#define DART_IO_MJCF_DETAIL_ACTUATORATTRIBUTES_HPP_

#include <dart/math/math_types.hpp>

#include <dart/io/export.hpp>
#include <dart/io/mjcf/detail/error.hpp>
#include <dart/io/mjcf/detail/types.hpp>

#include <Eigen/Core>
#include <tinyxml2.h>

#include <optional>
#include <string>

namespace dart {
namespace io {
namespace MjcfParser {
namespace detail {

struct DART_IO_API ActuatorAttributes final
{
  std::optional<std::string> mName;
  std::string mJoint;
  ActuatorType mType{ActuatorType::GENERAL};
  std::optional<bool> mCtrlLimited;
  Eigen::Vector2d mCtrlRange{Eigen::Vector2d::Zero()};
  std::optional<bool> mForceLimited;
  Eigen::Vector2d mForceRange{Eigen::Vector2d::Zero()};
  Eigen::Vector6d mGear{(Eigen::Vector6d() << 1, 0, 0, 0, 0, 0).finished()};
  Eigen::Vector3d mGainPrm{Eigen::Vector3d::Zero()};
  Eigen::Vector3d mBiasPrm{Eigen::Vector3d::Zero()};
};

DART_IO_API Errors appendActuatorAttributes(
    ActuatorAttributes& attributes, tinyxml2::XMLElement* element);

} // namespace detail
} // namespace MjcfParser
} // namespace io
} // namespace dart

#endif // #ifndef DART_IO_MJCF_DETAIL_ACTUATORATTRIBUTES_HPP_
