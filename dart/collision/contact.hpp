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

#include "dart/collision/types.hpp"
#include "dart/math/Types.hpp"

namespace dart {
namespace collision2 {

/// Contact information
template <typename S_>
struct Contact
{
  using S = S_;

  /// Default constructor
  Contact();

  /// Contact point w.r.t. the world frame
  math::Vector3<S> point;

  /// Contact normal vector from bodyNode2 to bodyNode1 w.r.t. the world frame
  math::Vector3<S> normal;

  /// Contact force acting on bodyNode1 w.r.t. the world frame
  ///
  /// The contact force acting on bodyNode2 is -force, which is the opposite
  /// direction of the force.
  math::Vector3<S> force;

  /// First colliding collision object
  Object<S>* collision_object1;

  /// Second colliding collision object
  Object<S>* collision_object2;

  /// Penetration depth
  S depth;

  /// Returns the epsilon to be used for determination of zero-length normal.
  constexpr static S get_normal_epsilon();

  /// Returns the squired epsilon to be used for determination of zero-length
  /// normal.
  constexpr static S get_normal_epsilon_squared();

  /// Returns true if the length of a normal is less than the epsilon.
  static bool is_zero_normal(const math::Vector3<S>& normal);

  /// Returns !isZeroNormal().
  static bool is_non_zero_normal(const math::Vector3<S>& normal);
};

using Contactf = Contact<float>;
using Contactd = Contact<double>;

extern template struct Contact<double>;

} // namespace collision2
} // namespace dart

#include "dart/collision/detail/contact_impl.hpp"
