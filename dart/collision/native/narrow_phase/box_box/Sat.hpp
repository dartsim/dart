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

#pragma once

#include <dart/collision/native/Export.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <limits>

namespace dart::collision::native::box_box {

enum class SatAxisType
{
  Face,
  Edge
};

struct BoxData
{
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  Eigen::Vector3d halfExtents = Eigen::Vector3d::Zero();
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
};

struct SatResult
{
  double penetration = std::numeric_limits<double>::max();
  Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
  int axisIndex = -1;
  SatAxisType axisType = SatAxisType::Face;
  int referenceBox = -1;
  int referenceAxis = -1;
};

[[nodiscard]] DART_COLLISION_NATIVE_API double projectBox(
    const BoxData& box, const Eigen::Vector3d& axis);

[[nodiscard]] DART_COLLISION_NATIVE_API bool computeBoxBoxSat(
    const BoxData& box1, const BoxData& box2, SatResult& result);

} // namespace dart::collision::native::box_box
