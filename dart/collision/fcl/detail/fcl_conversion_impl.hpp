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

#include "dart/collision/fcl/fcl_conversion.hpp"

namespace dart {
namespace collision {

//==============================================================================
template <typename S>
FclVector3<S> toFclVector3(const math::Vector3<S>& vec) {
  return FclVector3<S>(vec[0], vec[1], vec[2]);
}

//==============================================================================
template <typename S>
math::Vector3<S> toVector3(const FclVector3<S>& vec) {
#if FCL_VERSION_AT_LEAST(0, 6, 0)
  return vec;
#else
  return math::Vector3<S>(vec[0], vec[1], vec[2]);
#endif
}

//==============================================================================
template <typename S>
FclMatrix3<S> toFclMatrix3(const math::Matrix3<S>& R) {
#if FCL_VERSION_AT_LEAST(0, 6, 0)
  return R;
#else
  return FclMatrix3<S>(
      R(0, 0),
      R(0, 1),
      R(0, 2),
      R(1, 0),
      R(1, 1),
      R(1, 2),
      R(2, 0),
      R(2, 1),
      R(2, 2));
#endif
}

//==============================================================================
template <typename S>
math::Matrix3<S> toMatrix3(const FclMatrix3<S>& R) {
#if FCL_VERSION_AT_LEAST(0, 6, 0)
  return R;
#else
  math::Matrix3<S> out;
  out << R(0, 0), R(0, 1), R(0, 2), R(1, 0), R(1, 1), R(1, 2), R(2, 0), R(2, 1),
      R(2, 2);
  return out;
#endif
}

//==============================================================================
template <typename S>
FclTransform3<S> toFclTransform3(const math::Isometry3<S>& T) {
#if FCL_VERSION_AT_LEAST(0, 6, 0)
  return T;
#else
  FclTransform3<S> trans;

  trans.setTranslation(toFclVector3<S>(T.translation()));
  trans.setRotation(toFclMatrix3<S>(T.linear()));

  return trans;
#endif
}

//==============================================================================
template <typename S>
math::Isometry3<S> toTransform3(const FclTransform3<S>& T) {
#if FCL_VERSION_AT_LEAST(0, 6, 0)
  return T;
#else
  math::Isometry3<S> trans = math::Isometry3<S>::Identity();

  trans.translation() = toVector3<S>(T.getTranslation());
  trans.linear() = toMatrix3<S>(T.getRotation());

  return trans;
#endif
}

} // namespace collision
} // namespace dart
