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

#include "dart/collision/fcl/BackwardCompatibility.hpp"

namespace dart {
namespace collision {
namespace fcl {

//==============================================================================
double length(const dart::collision::fcl::Vector3& t)
{
  return t.norm();
}

//==============================================================================
double length2(const dart::collision::fcl::Vector3& t)
{
  return t.squaredNorm();
}

//==============================================================================
dart::collision::fcl::Vector3 getTranslation(
    const dart::collision::fcl::Transform3& T)
{
  return T.translation();
}

//==============================================================================
void setTranslation(
    dart::collision::fcl::Transform3& T, const dart::collision::fcl::Vector3& t)
{
  T.translation() = t;
}

//==============================================================================
dart::collision::fcl::Matrix3 getRotation(
    const dart::collision::fcl::Transform3& T)
{
  return T.linear();
}

//==============================================================================
void setRotation(
    dart::collision::fcl::Transform3& T, const dart::collision::fcl::Matrix3& R)
{
  T.linear() = R;
}

//==============================================================================
void setEulerZYX(
    dart::collision::fcl::Matrix3& rot,
    double eulerX,
    double eulerY,
    double eulerZ)
{
  double ci(cos(eulerX));
  double cj(cos(eulerY));
  double ch(cos(eulerZ));
  double si(sin(eulerX));
  double sj(sin(eulerY));
  double sh(sin(eulerZ));
  double cc = ci * ch;
  double cs = ci * sh;
  double sc = si * ch;
  double ss = si * sh;

  // clang-format off
  rot << cj * ch, sj * sc - cs, sj * cc + ss,
         cj * sh, sj * ss + cc, sj * cs - sc,
         -sj,     cj * si,      cj * ci;
  // clang-format on
}

//==============================================================================
dart::collision::fcl::Vector3 transform(
    const dart::collision::fcl::Transform3& t,
    const dart::collision::fcl::Vector3& v)
{
  return t * v;
}

} // namespace fcl
} // namespace collision
} // namespace dart
