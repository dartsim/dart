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

#ifndef DART_COLLISION_DART_DARTCOLLISIONCORE_HPP_
#define DART_COLLISION_DART_DARTCOLLISIONCORE_HPP_

#include <Eigen/Dense>

#include <cstdint>

namespace dart {
namespace collision {

enum class CoreShapeType : std::uint8_t
{
  kNone = 0u,
  kSphere,
  kBox,
  kUnsupported
};

struct CoreShape
{
  CoreShapeType type{CoreShapeType::kNone};
  Eigen::Vector3d size{Eigen::Vector3d::Zero()};
  double radius{0.0};
};

struct CoreObject
{
  CoreShape shape;
  Eigen::Isometry3d worldTransform{Eigen::Isometry3d::Identity()};
  Eigen::Vector3d worldAabbMin{Eigen::Vector3d::Zero()};
  Eigen::Vector3d worldAabbMax{Eigen::Vector3d::Zero()};
};

} // namespace collision
} // namespace dart

#endif // DART_COLLISION_DART_DARTCOLLISIONCORE_HPP_
