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

#include "dart/collision/fcl/Types.hpp"

namespace dart {
namespace collision2 {

#if !FCL_VERSION_AT_LEAST(0, 6, 0)
//==============================================================================
template <typename S>
dart::collision2::fcl::Vector3<S> FCLTypes<S>::convertVector3(
    const math::Vector3<S>& vec)
{
  return dart::collision2::fcl::Vector3<S>(vec[0], vec[1], vec[2]);
}
#endif

//==============================================================================
template <typename S>
math::Vector3<S> FCLTypes<S>::convertVector3(
    const dart::collision2::fcl::Vector3<S>& vec)
{
#if FCL_VERSION_AT_LEAST(0, 6, 0)
  return vec;
#else
  return Eigen::Vector3d(vec[0], vec[1], vec[2]);
#endif
}

//==============================================================================
template <typename S>
dart::collision2::fcl::Matrix3<S> FCLTypes<S>::convertMatrix3x3(
    const math::Matrix3<S>& R)
{
#if FCL_VERSION_AT_LEAST(0, 6, 0)
  return R;
#else
  return dart::collision2::fcl::Matrix3<S>(
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
dart::collision2::fcl::Transform3<S> FCLTypes<S>::convertTransform(
    const math::Isometry3<S>& T)
{
#if FCL_VERSION_AT_LEAST(0, 6, 0)
  return T;
#else
  dart::collision2::fcl::Transform3<S> trans;

  trans.setTranslation(convertVector3(T.translation()));
  trans.setRotation(convertMatrix3x3(T.linear()));

  return trans;
#endif
}

} // namespace collision2
} // namespace dart
