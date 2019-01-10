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

#ifndef DART_COLLISION_FCL_FCLTTYPES_HPP_
#define DART_COLLISION_FCL_FCLTTYPES_HPP_

#include <Eigen/Dense>
#include "dart/collision/fcl/BackwardCompatibility.hpp"

namespace dart {
namespace collision {

class FCLTypes
{
public:
#if !FCL_VERSION_AT_LEAST(0,6,0)
  /// Convert Eigen vector3 type to FCL vector3 type
  static dart::collision::fcl::Vector3 convertVector3(const Eigen::Vector3d& _vec);
#endif
  /// Convert FCL vector3 type to Eigen vector3 type
  static Eigen::Vector3d convertVector3(const dart::collision::fcl::Vector3& _vec);

  /// Convert FCL matrix3x3 type to Eigen matrix3x3 type
  static dart::collision::fcl::Matrix3 convertMatrix3x3(const Eigen::Matrix3d& _R);

  /// Convert FCL transformation type to Eigen transformation type
  static dart::collision::fcl::Transform3 convertTransform(const Eigen::Isometry3d& _T);
};

} // namespace collision
} // namespace dart

#endif // DART_COLLISION_FCL_FCLTTYPES_HPP_
