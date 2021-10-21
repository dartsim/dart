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

namespace dart {
namespace collision {

//==============================================================================
template <typename Scalar>
Scalar length(const FclVector3<Scalar>& t)
{
#if FCL_VERSION_AT_LEAST(0, 6, 0)
  return t.norm();
#else
  return t.length();
#endif
}

//==============================================================================
template <typename Scalar>
Scalar length2(const FclVector3<Scalar>& t)
{
#if FCL_VERSION_AT_LEAST(0, 6, 0)
  return t.squaredNorm();
#else
  return t.sqrLength();
#endif
}

//==============================================================================
template <typename Scalar>
FclVector3<Scalar> getTranslation(const FclTransform3<Scalar>& T)
{
#if FCL_VERSION_AT_LEAST(0, 6, 0)
  return T.translation();
#else
  return T.getTranslation();
#endif
}

//==============================================================================
template <typename Scalar>
void setTranslation(FclTransform3<Scalar>& T, const FclVector3<Scalar>& t)
{
#if FCL_VERSION_AT_LEAST(0, 6, 0)
  T.translation() = t;
#else
  T.setTranslation(t);
#endif
}

//==============================================================================
template <typename Scalar>
FclMatrix3<Scalar> getRotation(const FclTransform3<Scalar>& T)
{
#if FCL_VERSION_AT_LEAST(0, 6, 0)
  return T.linear();
#else
  return T.getRotation();
#endif
}

//==============================================================================
template <typename Scalar>
void setRotation(FclTransform3<Scalar>& T, const FclMatrix3<Scalar>& R)
{
#if FCL_VERSION_AT_LEAST(0, 6, 0)
  T.linear() = R;
#else
  T.setRotation(R);
#endif
}

//==============================================================================
template <typename Scalar>
void setEulerZYX(
    FclMatrix3<Scalar>& rot, Scalar eulerX, Scalar eulerY, Scalar eulerZ)
{
#if FCL_VERSION_AT_LEAST(0, 6, 0)
  Scalar ci(cos(eulerX));
  Scalar cj(cos(eulerY));
  Scalar ch(cos(eulerZ));
  Scalar si(sin(eulerX));
  Scalar sj(sin(eulerY));
  Scalar sh(sin(eulerZ));
  Scalar cc = ci * ch;
  Scalar cs = ci * sh;
  Scalar sc = si * ch;
  Scalar ss = si * sh;

  // clang-format off
  rot << cj * ch, sj * sc - cs, sj * cc + ss,
         cj * sh, sj * ss + cc, sj * cs - sc,
         -sj,     cj * si,      cj * ci;
  // clang-format on
#else
  rot.setEulerZYX(eulerX, eulerY, eulerZ);
#endif
}

//==============================================================================
template <typename Scalar>
FclVector3<Scalar> transform(
    const FclTransform3<Scalar>& t, const FclVector3<Scalar>& v)
{
#if FCL_VERSION_AT_LEAST(0, 6, 0)
  return t * v;
#else
  return t.transform(v);
#endif
}

//==============================================================================
template <typename Scalar>
FclTransform3<Scalar> get_identity_transform()
{
#if FCL_VERSION_AT_LEAST(0, 6, 0)
  return FclTransform3<Scalar>::Identity();
#else
  return FclTransform3<Scalar>();
#endif
}

} // namespace collision
} // namespace dart
