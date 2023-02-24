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
#include <dart/physics/RigidBodyBatchData.hpp>

#include <unordered_map>
#include <unordered_set>
#include <variant>

namespace dart::physics {

template <typename S>
class RigidBodyBatch
{
public:
  using Scalar = S;

  /// Constructor
  ///
  /// @param[in] world The world that this RigidBodyBatch belongs to.
  explicit RigidBodyBatch(World<S>* world);

  /// Destructor
  ~RigidBodyBatch();

  /// Create a new RigidBody
  ///
  /// @return A pointer to the newly created RigidBody.
  RigidBody<S>* createRigidBody();

  void destroyRigidBody(RigidBody<S>* rigid_body);

  [[nodiscard]] auto getPosition(size_t index) const
  {
    return m_data.transforms[index].translation();
  }

  /// Returns the world that this RigidBodyBatch belongs to.
  World<S>* getWorld();

  /// Returns the world that this RigidBodyBatch belongs to.
  const World<S>* getWorld() const;

private:
  friend class World<S>;

  size_t index = 0;

  void bake();

  using Data = RigidBodyBatchData<S>;

  World<S>* m_world;

  std::unordered_map<RigidBody<S>*, size_t> m_rigid_body_new;

  Data m_data;
};

} // namespace dart::physics

//==============================================================================
// Implementation
//==============================================================================

#include <dart/common/StlUtils.hpp>

#include <dart/physics/RigidBody.hpp>
#include <dart/physics/World.hpp>

namespace dart::physics {

//==============================================================================
template <typename S>
RigidBodyBatch<S>::RigidBodyBatch(World<S>* world) : m_world(world)
{
  DART_ASSERT(m_world);
}

//==============================================================================
template <typename S>
RigidBodyBatch<S>::~RigidBodyBatch()
{
  // Empty
}

//==============================================================================
template <typename S>
RigidBody<S>* RigidBodyBatch<S>::createRigidBody()
{
  auto mb = new RigidBody<S>(this);

  const size_t index = m_data.getSize();

  m_data.addBack();

  if (!mb->init(index)) {
    DART_WARN(
        "Failed to initialize newly created RigidBody. Returning nullptr.");
    m_data.removeBack();
    return nullptr;
  }

  m_rigid_body_new[mb] = index;

  return mb;
}

//==============================================================================
template <typename S>
void RigidBodyBatch<S>::destroyRigidBody(RigidBody<S>* rigid_body)
{
  (void)rigid_body;
}

//==============================================================================
template <typename S>
World<S>* RigidBodyBatch<S>::getWorld()
{
  return m_world;
}

//==============================================================================
template <typename S>
const World<S>* RigidBodyBatch<S>::getWorld() const
{
  return m_world;
}

//==============================================================================
template <typename S>
void RigidBodyBatch<S>::bake()
{
  for (const auto [i, removed] : common::Enumerate(m_data.removed)) {
    if (!removed) {
      continue;
    }

    if (m_rigid_body_new.empty()) {
      // TODO
    } else {
      // TODO
    }
  }
}

} // namespace dart::physics
