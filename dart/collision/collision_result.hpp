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

#include <vector>

#include "dart/collision/contact.hpp"

namespace dart {
namespace collision {

template <typename S_>
class CollisionResult {
public:
  using S = S_;

  /// Adds one contact
  void add_contact(const Contact<S>& contact);

  /// Returns number of contacts
  std::size_t get_num_contacts() const;

  /// Returns the index-th contact
  Contact<S>& get_mutable_contact(int index);

  /// Returns (const) the index-th contact
  const Contact<S>& get_contact(int index) const;

  /// Returns contacts
  const std::vector<Contact<S>>& get_contacts() const;

  /// Returns binary collision result
  bool is_collision() const;

  /// Implicitly converts this CollisionResult to the value of isCollision()
  operator bool() const;

  /// Clears all the contacts
  void clear();

protected:
  /// List of contact information for each contact
  std::vector<Contact<S>> m_contacts;
};

extern template class CollisionResult<double>;

} // namespace collision
} // namespace dart

#include "dart/collision/detail/collision_result_impl.hpp"
