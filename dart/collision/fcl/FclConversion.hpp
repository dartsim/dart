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
#include "dart/math/Types.hpp"

namespace dart {
namespace collision2 {

/// Converts Eigen vector3 to FCL vector3
template <typename S>
FclVector3<S> toFclVector3(const math::Vector3<S>& vec);

/// Converts FCL vector3 to Eigen vector3
template <typename S>
math::Vector3<S> toVector3(const FclVector3<S>& vec);

/// Converts Eigen matrix3x3 to FCL matrix3x3
template <typename S>
FclMatrix3<S> toFclMatrix3(const math::Matrix3<S>& R);

/// Converts FCL matrix3x3 to Eigen matrix3x3
template <typename S>
math::Matrix3<S> toMatrix3(const FclMatrix3<S>& R);

/// Converts Eigen transform to FCL transform
template <typename S>
FclTransform3<S> toFclTransform3(const math::Isometry3<S>& T);

/// Converts FCL transform to Eigen transform
template <typename S>
math::Isometry3<S> toTransform3(const FclTransform3<S>& T);

} // namespace collision2
} // namespace dart

#include "dart/collision/fcl/detail/FclConversion-impl.hpp"
