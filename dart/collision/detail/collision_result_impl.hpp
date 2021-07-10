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

#include "dart/collision/collision_result.hpp"

namespace dart {
namespace collision {

//==============================================================================
template <typename S>
void CollisionResult<S>::add_contact(const Contact<S>& contact) {
  m_contacts.push_back(contact);
}

//==============================================================================
template <typename S>
std::size_t CollisionResult<S>::get_num_contacts() const {
  return m_contacts.size();
}

//==============================================================================
template <typename S>
Contact<S>& CollisionResult<S>::get_mutable_contact(int index) {
  assert(0 <= index && static_cast<std::size_t>(index) < m_contacts.size());

  return m_contacts[index];
}

//==============================================================================
template <typename S>
const Contact<S>& CollisionResult<S>::get_contact(int index) const {
  assert(0 <= index && static_cast<std::size_t>(index) < m_contacts.size());

  return m_contacts[index];
}

//==============================================================================
template <typename S>
const std::vector<Contact<S>>& CollisionResult<S>::get_contacts() const {
  return m_contacts;
}

//==============================================================================
template <typename S>
bool CollisionResult<S>::is_collision() const {
  return !m_contacts.empty();
}

//==============================================================================
template <typename S>
CollisionResult<S>::operator bool() const {
  return is_collision();
}

//==============================================================================
template <typename S>
void CollisionResult<S>::clear() {
  m_contacts.clear();
}

} // namespace collision
} // namespace dart
