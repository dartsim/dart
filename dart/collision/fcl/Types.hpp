/*
 * Copyright (c) 2011-2021, The DART development contributors
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

#pragma once

#include "dart/collision/fcl/BackwardCompatibility.hpp"
#include "dart/common/SmartPointer.hpp"
#include "dart/math/Types.hpp"

namespace dart {
namespace collision2 {

template <typename S>
class FCLTypes
{
public:
#if !FCL_VERSION_AT_LEAST(0, 6, 0)
  /// Convert Eigen vector3 type to FCL vector3 type
  static dart::collision2::fcl::Vector3<S> convertVector3(
      const math::Vector3<S>& vec);
#endif
  /// Convert FCL vector3 type to Eigen vector3 type
  static math::Vector3<S> convertVector3(
      const dart::collision2::fcl::Vector3<S>& vec);

  /// Convert FCL matrix3x3 type to Eigen matrix3x3 type
  static dart::collision2::fcl::Matrix3<S> convertMatrix3x3(
      const math::Matrix3<S>& R);

  /// Convert FCL transformation type to Eigen transformation type
  static dart::collision2::fcl::Transform3<S> convertTransform(
      const math::Isometry3<S>& T);
};

DART_DEFINE_CLASS_POINTERS_T1(FCLCollisionDetector);
DART_DEFINE_CLASS_POINTERS_T1(FCLCollisionGroup);

} // namespace collision2
} // namespace dart

#include "dart/collision/fcl/detail/FCLTypes-impl.hpp"
