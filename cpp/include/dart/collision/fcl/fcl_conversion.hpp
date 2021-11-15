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

#include "dart/collision/fcl/backward_compatibility.hpp"
#include "dart/math/lie_group/type.hpp"
#include "dart/math/type.hpp"

namespace dart {
namespace collision {

/// Converts Eigen vector3 to FCL vector3
template <typename Scalar>
FclVector3<Scalar> to_fcl_vector3(const math::Vector3<Scalar>& vec);

/// Converts FCL vector3 to Eigen vector3
template <typename Scalar>
math::Vector3<Scalar> to_vector3(const FclVector3<Scalar>& vec);

/// Converts Eigen matrix3x3 to FCL matrix3x3
template <typename Scalar>
FclMatrix3<Scalar> to_fcl_matrix3(const math::Matrix3<Scalar>& R);

/// Converts FCL matrix3x3 to Eigen matrix3x3
template <typename Scalar>
math::Matrix3<Scalar> to_matrix3(const FclMatrix3<Scalar>& R);

/// Converts Eigen transform to FCL transform
template <typename Scalar>
FclTransform3<Scalar> to_fcl_pose3(const math::Isometry3<Scalar>& T);

/// Converts FCL transform to Eigen transform
template <typename Scalar>
math::Isometry3<Scalar> to_pose3(const FclTransform3<Scalar>& T);

/// Converts FCL vector3 to R3
template <typename Scalar>
math::R3<Scalar> to_r3(const FclVector3<Scalar>& vec);

/// Converts FCL matrix3x3 to SO3
template <typename Scalar>
math::SO3<Scalar> to_so3(const FclMatrix3<Scalar>& R);

/// Converts Eigen transform to FCL transform
template <typename Scalar>
FclTransform3<Scalar> to_fcl_pose3(const math::SE3<Scalar>& T);

/// Converts FCL transform to SE3
template <typename Scalar>
math::SE3<Scalar> to_se3(const FclTransform3<Scalar>& T);

} // namespace collision
} // namespace dart

#include "dart/collision/fcl/detail/fcl_conversion_impl.hpp"
