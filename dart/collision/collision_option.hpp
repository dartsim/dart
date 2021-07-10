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

namespace dart {
namespace collision {

template <typename S>
struct CollisionOption
{
  /// Flag whether the collision detector computes contact information (contact
  /// point, normal, and penetration depth). If it is set to false, only the
  /// result of that which pairs are colliding will be stored in the
  /// CollisionResult without the contact information.
  bool enable_contact;

  /// Maximum number of contacts to detect. Once the contacts are found up to
  /// this number, the collision checking will terminate at that moment. Set
  /// this to 1 for binary check.
  int max_num_contacts;

  /// CollisionFilter
  CollisionFilterPtr<S> collision_filter;

  /// Constructor
  CollisionOption(
      bool enable_contact = true,
      int max_num_contacts = 1000,
      const CollisionFilterPtr<S>& collision_filter = nullptr);
};

using CollisionOptionf = CollisionOption<float>;
using CollisionOptiond = CollisionOption<double>;

extern template struct CollisionOption<double>;

} // namespace collision
} // namespace dart

#include "dart/collision/detail/collision_option_impl.hpp"
