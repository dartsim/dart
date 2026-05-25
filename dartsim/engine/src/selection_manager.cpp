/*
 * Copyright (c) 2011, The DART development contributors
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

#include <dartsim_engine/selection_manager.hpp>

#include <algorithm>

namespace dartsim {

void SelectionManager::select(ObjectId id, bool additive)
{
  if (id == kNoObject) {
    if (!additive) {
      clear();
    }
    return;
  }
  if (!additive) {
    m_state.ids.clear();
  }
  if (std::find(m_state.ids.begin(), m_state.ids.end(), id)
      == m_state.ids.end()) {
    m_state.ids.push_back(id);
  }
  m_state.primary = id;
}

void SelectionManager::deselect(ObjectId id)
{
  m_state.ids.erase(
      std::remove(m_state.ids.begin(), m_state.ids.end(), id),
      m_state.ids.end());
  if (m_state.primary == id) {
    m_state.primary = m_state.ids.empty() ? kNoObject : m_state.ids.back();
  }
}

void SelectionManager::toggle(ObjectId id, bool additive)
{
  if (isSelected(id)) {
    deselect(id);
  } else {
    select(id, additive);
  }
}

void SelectionManager::clear()
{
  m_state.ids.clear();
  m_state.primary = kNoObject;
}

bool SelectionManager::isSelected(ObjectId id) const
{
  return std::find(m_state.ids.begin(), m_state.ids.end(), id)
         != m_state.ids.end();
}

} // namespace dartsim
