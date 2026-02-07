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

#ifndef DART_UTILS_MJCF_DETAIL_ACTUATOR_HPP_
#define DART_UTILS_MJCF_DETAIL_ACTUATOR_HPP_

#include <dart/utils/export.hpp>
#include <dart/utils/mjcf/detail/error.hpp>
#include <dart/utils/mjcf/detail/types.hpp>

#include <dart/math/math_types.hpp>

#include <Eigen/Core>
#include <tinyxml2.h>

#include <string>
#include <vector>

namespace dart {
namespace utils {
namespace MjcfParser {
namespace detail {

class DART_UTILS_API Actuator final
{
public:
  Actuator() = default;

  struct Entry
  {
    std::string mName;
    std::string mJoint;
    ActuatorType mType{ActuatorType::GENERAL};
    bool mCtrlLimited{false};
    Eigen::Vector2d mCtrlRange{Eigen::Vector2d::Zero()};
    bool mForceLimited{false};
    Eigen::Vector2d mForceRange{Eigen::Vector2d::Zero()};
    Eigen::Vector6d mGear{(Eigen::Vector6d() << 1, 0, 0, 0, 0, 0).finished()};
    Eigen::Vector3d mGainPrm{Eigen::Vector3d::Zero()};
    Eigen::Vector3d mBiasPrm{Eigen::Vector3d::Zero()};
  };

  std::size_t getNumEntries() const;
  const Entry& getEntry(std::size_t index) const;

private:
  friend class MujocoModel;
  Errors read(tinyxml2::XMLElement* element);

  std::vector<Entry> mEntries;
};

} // namespace detail
} // namespace MjcfParser
} // namespace utils
} // namespace dart

#endif // #ifndef DART_UTILS_MJCF_DETAIL_ACTUATOR_HPP_
