/*
 * Copyright (c) 2011-2023, The DART development contributors
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

#include <dart/math/LieGroups.hpp>

#include <dart/common/Containers.hpp>

#include <dart/physics/Fwd.hpp>
#include <dart/physics/MultiBodyBatchData.hpp>
#include <dart/physics/MultiBodyJoint.hpp>
#include <dart/physics/MultiBodyLink.hpp>

#include <unordered_map>
#include <unordered_set>
#include <variant>

namespace dart::physics {

template <typename S>
class MultiBodyBatch
{
public:
  using Scalar = S;

  /// Constructor
  ///
  /// @param[in] world The world that this MultiBodyBatch belongs to.
  explicit MultiBodyBatch(World<S>* world);

  /// Destructor
  ~MultiBodyBatch();

  /// Create a new MultiBody
  ///
  /// @return A pointer to the newly created MultiBody.
  MultiBody<S>* createMultiBody();

  /// Returns the world that this MultiBodyBatch belongs to.
  World<S>* getWorld();

  /// Returns the world that this MultiBodyBatch belongs to.
  const World<S>* getWorld() const;

private:
  friend class World<S>;

  void bake();

  using Data = MultiBodyBatchData<S>;

  World<S>* m_world;

  std::unordered_map<MultiBody<S>*, std::shared_ptr<Data>> m_multibodies_new;

  Data m_data;
};

} // namespace dart::physics

//==============================================================================
// Implementation
//==============================================================================

#include <dart/common/StlUtils.hpp>

#include <dart/physics/MultiBody.hpp>
#include <dart/physics/World.hpp>

namespace dart::physics {

//==============================================================================
template <typename S>
MultiBodyBatch<S>::MultiBodyBatch(World<S>* world) : m_world(world)
{
  DART_ASSERT(m_world);
}

//==============================================================================
template <typename S>
MultiBodyBatch<S>::~MultiBodyBatch()
{
  // Empty
}

//==============================================================================
template <typename S>
MultiBody<S>* MultiBodyBatch<S>::createMultiBody()
{
  auto mb = new MultiBody<S>(this);

  // Create a new data
  auto data = std::make_shared<Data>();
  // TODO: Consider resuing discarded data

  if (!mb->init(data.get())) {
    DART_WARN(
        "Failed to initialize newly created multibody. Returning nullptr.");
    return nullptr;
  }

  m_multibodies_new[mb] = std::move(data);

  return mb;
}

//==============================================================================
template <typename S>
World<S>* MultiBodyBatch<S>::getWorld()
{
  return m_world;
}

//==============================================================================
template <typename S>
const World<S>* MultiBodyBatch<S>::getWorld() const
{
  return m_world;
}

//==============================================================================
template <typename S>
void MultiBodyBatch<S>::bake()
{
  // Update link count and offset for each non-removed multibody
  size_t multibody_index_new = 0;
  size_t link_index_new = 0;
  size_t link_offset = 0;
  size_t dofs_offset = 0;
  const size_t num_multibodies_old = m_data.link_count.size();
  for (auto i : common::Range(num_multibodies_old)) {
    if (m_data.removed[i]) {
      continue;
    }
    const auto& link_count = m_data.link_count[i];
    const auto& dofs_count = m_data.dofs_count[i];

    m_data.link_count[multibody_index_new] = link_count;
    m_data.link_offset[multibody_index_new] = link_offset;
    m_data.dofs_count[multibody_index_new] = dofs_count;
    m_data.dofs_offset[multibody_index_new] = dofs_offset;
    m_data.removed[multibody_index_new] = false;

    for (auto j = 0u; j < link_count; ++j) {
      m_data.transforms[link_index_new]
          = m_data.transforms[m_data.link_offset[i] + j];
      link_index_new++;
    }

    m_data.positions.segment(dofs_offset, dofs_count)
        = m_data.positions.segment(m_data.dofs_offset[i], dofs_count);

    link_offset += link_count;
    dofs_offset += dofs_count;
    multibody_index_new++;
  }

  const size_t num_multibodies_new
      = multibody_index_new + m_multibodies_new.size();
  size_t num_links_new = link_index_new;
  size_t num_dofs_new = dofs_offset;
  for (const auto& it : m_multibodies_new) {
    const auto& data = *it.second;
    const auto& link_count = data.link_count[0];
    const auto& dofs_count = data.dofs_count[0];
    num_links_new += link_count;
    num_dofs_new += dofs_count;
  }

  m_data.link_count.resize(num_multibodies_new);
  m_data.link_offset.resize(num_multibodies_new);
  m_data.dofs_count.resize(num_multibodies_new);
  m_data.dofs_offset.resize(num_multibodies_new);
  m_data.removed.resize(num_multibodies_new, false);
  m_data.transforms.resize(num_links_new);
  m_data.positions.conservativeResize(num_dofs_new);

  for (const auto& it : m_multibodies_new) {
    const auto& data = *it.second;
    const auto& link_count = data.link_count[0];
    const auto& dofs_count = data.dofs_count[0];

    m_data.link_count[multibody_index_new] = link_count;
    m_data.link_offset[multibody_index_new] = link_offset;
    m_data.dofs_count[multibody_index_new] = dofs_count;
    m_data.dofs_offset[multibody_index_new] = dofs_offset;

    for (auto j = 0u; j < link_count; ++j) {
      m_data.transforms[link_index_new] = data.transforms[j];
      link_index_new++;
    }

    m_data.positions.segment(dofs_offset, dofs_count) = data.positions;

    link_offset += link_count;
    dofs_offset += dofs_count;
    multibody_index_new++;
  }
  DART_ASSERT(multibody_index_new == num_multibodies_new);

  m_multibodies_new.clear();
}

} // namespace dart::physics
