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
template <typename Scalar>
FclVector3<Scalar> to_fcl_vector3(const math::Vector3<Scalar>& vec)
{
  return FclVector3<Scalar>(vec[0], vec[1], vec[2]);
}

//==============================================================================
template <typename Scalar>
math::Vector3<Scalar> to_vector3(const FclVector3<Scalar>& vec)
{
#if FCL_VERSION_AT_LEAST(0, 6, 0)
  return vec;
#else
  return math::Vector3<Scalar>(vec[0], vec[1], vec[2]);
#endif
}

//==============================================================================
template <typename Scalar>
FclMatrix3<Scalar> to_fcl_matrix3(const math::Matrix3<Scalar>& R)
{
#if FCL_VERSION_AT_LEAST(0, 6, 0)
  return R;
#else
  return FclMatrix3<Scalar>(
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
template <typename Scalar>
math::Matrix3<Scalar> to_matrix3(const FclMatrix3<Scalar>& R)
{
#if FCL_VERSION_AT_LEAST(0, 6, 0)
  return R;
#else
  math::Matrix3<Scalar> out;
  // clang-format off
  out <<
     R(0, 0), R(0, 1), R(0, 2),
     R(1, 0), R(1, 1), R(1, 2),
     R(2, 0), R(2, 1), R(2, 2);
  // clang-format on
  return out;
#endif
}

//==============================================================================
template <typename Scalar>
FclTransform3<Scalar> to_fcl_pose3(const math::Isometry3<Scalar>& T)
{
#if FCL_VERSION_AT_LEAST(0, 6, 0)
  return T;
#else
  FclTransform3<Scalar> trans;

  trans.setTranslation(to_fcl_vector3<Scalar>(T.translation()));
  trans.setRotation(to_fcl_matrix3<Scalar>(T.linear()));

  return trans;
#endif
}

//==============================================================================
template <typename Scalar>
math::Isometry3<Scalar> to_pose3(const FclTransform3<Scalar>& T)
{
#if FCL_VERSION_AT_LEAST(0, 6, 0)
  return T;
#else
  math::Isometry3<Scalar> trans = math::Isometry3<Scalar>::Identity();

  trans.translation() = to_vector3<Scalar>(T.getTranslation());
  trans.linear() = to_matrix3<Scalar>(T.getRotation());

  return trans;
#endif
}

//==============================================================================
template <typename Scalar>
math::R3<Scalar> to_r3(const FclVector3<Scalar>& vec)
{
#if FCL_VERSION_AT_LEAST(0, 6, 0)
  return math::R3<Scalar>(vec);
#else
  return math::R3<Scalar>(vec[0], vec[1], vec[2]);
#endif
}

//==============================================================================
template <typename Scalar>
math::SO3<Scalar> to_so3(const FclMatrix3<Scalar>& R)
{
  math::SO3<Scalar> out;
  out.set_from_rotation_matrix(to_matrix3<Scalar>(R));
  return out;
}

//==============================================================================
template <typename Scalar>
FclTransform3<Scalar> to_fcl_pose3(const math::SE3<Scalar>& T)
{
#if FCL_VERSION_AT_LEAST(0, 6, 0)
  return T.to_transformation();
#else
  FclTransform3<Scalar> out;
  out.setTranslation(to_fcl_vector3<Scalar>(T.to_translation()));
  out.setRotation(to_fcl_matrix3<Scalar>(T.to_rotation()));
  return out;
#endif
}

//==============================================================================
template <typename Scalar>
math::SE3<Scalar> to_se3(const FclTransform3<Scalar>& T)
{
#if FCL_VERSION_AT_LEAST(0, 6, 0)
  return math::SE3<Scalar>(T);
#else
  math::SE3<Scalar> out;
  out.position() = to_r3<Scalar>(T.getTranslation());
  out.orientation() = to_so3<Scalar>(T.getRotation());
  return out;
#endif
}

} // namespace collision
} // namespace dart
