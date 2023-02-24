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

#include <dart/common/allocator/AlignedAllocator.hpp>

#include <dart/physics/Fwd.hpp>
#include <dart/physics/MultiBodyBatch.hpp>
#include <dart/physics/RigidBodyBatch.hpp>

#include <vector>

namespace dart::physics {

/// A class representing a physical simulation world
///
/// @tparam S Scalar type (e.g. float, double)
template <typename S = double>
class World
{
public:
  struct StepInfo
  {
    /// Current simulation time
    S time_s = 0.0;

    /// Current simulation time step
    S time_step_s = 0.001;
  };

  /// Construct a new World object
  ///
  /// @param[in] base_allocator The base allocator to use for memory allocation
  ///
  /// The default allocator is the default aligned allocator.
  explicit World(
      common::AlignedAllocator& base_allocator
      = common::AlignedAllocator::GetDefault());

  /// Destroy the World object
  ///
  /// This destructor finalizes and deletes all the MultiBody objects
  /// created in this world.
  virtual ~World();

  [[nodiscard]] RigidBody<S>* createRigidBody();

  /// Create a new MultiBody object
  ///
  /// @return MultiBody<S>* Pointer to the newly created MultiBody object.
  ///
  /// This method creates a new MultiBody object and adds it to the world.
  /// If the MultiBody object fails to initialize, the method returns a
  /// null pointer.
  [[nodiscard]] MultiBody<S>* createMultiBody();

  [[nodiscard]] bool isDesignMode() const
  {
    return m_is_design_mode;
  }

  [[nodiscard]] bool isSimulationMode() const
  {
    return !isDesignMode();
  }

  void setDesignMode()
  {
    if (m_is_design_mode) {
      return;
    }
  }

  void setSimulationMode()
  {
    if (!m_is_design_mode) {
      return;
    }

    m_rigid_body_batch.bake();
    m_multibody_batch.bake();
  }

  const StepInfo& step();

  /// Get the base allocator
  ///
  /// @return common::AlignedAllocator& Reference to the base allocator
  ///
  /// This method returns a reference to the base allocator used by this
  /// world.
  [[nodiscard]] common::AlignedAllocator& getBaseAllocator();

private:
  /// Reference to the base allocator
  common::AlignedAllocator& m_base_allocator;

  /// Step information
  StepInfo m_step_info;

  RigidBodyBatch<S> m_rigid_body_batch;

  MultiBodyBatch<S> m_multibody_batch;

  bool m_is_design_mode{true};
};

DART_TEMPLATE_CLASS_HEADER(PHYSICS, World);

} // namespace dart::physics

//==============================================================================
// Implementation
//==============================================================================

#include <dart/physics/MultiBody.hpp>

namespace dart::physics {

//==============================================================================
template <typename S>
World<S>::World(common::AlignedAllocator& base_allocator)
  : m_base_allocator(base_allocator),
    m_rigid_body_batch(this),
    m_multibody_batch(this)
{
  // Empty
}

//==============================================================================
template <typename S>
World<S>::~World()
{
  // Empty
}

//==============================================================================
template <typename S>
RigidBody<S>* World<S>::createRigidBody()
{
  return m_rigid_body_batch.createRigidBody();
}

//==============================================================================
template <typename S>
MultiBody<S>* World<S>::createMultiBody()
{
  return m_multibody_batch.createMultiBody();
}

//==============================================================================
template <typename S>
const typename World<S>::StepInfo& World<S>::step()
{
  setSimulationMode();
  return m_step_info;
}

//==============================================================================
template <typename S>
common::AlignedAllocator& World<S>::getBaseAllocator()
{
  return m_base_allocator;
}

} // namespace dart::physics
