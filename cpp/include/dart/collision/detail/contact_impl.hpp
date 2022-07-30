/*
 * Copyright (c) 2011-2022, The DART development contributors
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

#include "dart/collision/contact.hpp"

namespace dart {
namespace collision {

//==============================================================================
template <typename Scalar>
constexpr Scalar Contact<Scalar>::get_normal_epsilon()
{
  return 1e-6;
}

//==============================================================================
template <typename Scalar>
constexpr Scalar Contact<Scalar>::get_normal_epsilon_squared()
{
  return 1e-12;
}

//==============================================================================
template <typename Scalar>
Contact<Scalar>::Contact()
  : point(math::Vector3<Scalar>::Zero()),
    normal(math::Vector3<Scalar>::Zero()),
    force(math::Vector3<Scalar>::Zero()),
    collision_object1(nullptr),
    collision_object2(nullptr),
    depth(0)
{
  // TODO(MXG): Consider using NaN instead of zero for uninitialized quantities
  // Do nothing
}

//==============================================================================
template <typename Scalar>
bool Contact<Scalar>::is_zero_normal(const math::Vector3<Scalar>& normal)
{
  if (normal.squaredNorm() < get_normal_epsilon_squared())
    return true;
  else
    return false;
}

//==============================================================================
template <typename Scalar>
bool Contact<Scalar>::is_non_zero_normal(const math::Vector3<Scalar>& normal)
{
  return !is_zero_normal(normal);
}

} // namespace collision
} // namespace dart
