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

#pragma once

#include <dart/simulation/experimental/export.hpp>

#include <vector>

#include <cstddef>

namespace dart::simulation::experimental {
class World;
} // namespace dart::simulation::experimental

namespace dart::simulation::experimental::compute {

/// Batched structure-of-arrays snapshot of rigid-body state.
///
/// Each field is a flat, world-major @c double array sized
/// `worldCount * bodyCount` per body, so a leading world dimension can be added
/// without changing the layout: position/linear/angular use three components
/// and orientation uses four (w, x, y, z). Plain scalar arrays avoid Eigen
/// alignment constraints in containers and map directly onto SIMD lanes and
/// device buffers.
///
/// This is the seed of the batched state representation that later multi-core,
/// SIMD, and GPU paths consume. It is an internal experimental value type;
/// per-world public handles remain the user-facing API.
struct DART_EXPERIMENTAL_API RigidBodyStateBatch
{
  std::size_t worldCount = 1;
  std::size_t bodyCount = 0;
  std::vector<double> position;        ///< 3 * worldCount * bodyCount (x, y, z)
  std::vector<double> orientation;     ///< 4 * worldCount * bodyCount (w,x,y,z)
  std::vector<double> linearVelocity;  ///< 3 * worldCount * bodyCount (x, y, z)
  std::vector<double> angularVelocity; ///< 3 * worldCount * bodyCount (x, y, z)

  [[nodiscard]] bool isEmpty() const noexcept
  {
    return worldCount == 0 || bodyCount == 0;
  }
};

/// Extract a single-world (`worldCount == 1`) snapshot of all rigid bodies in
/// the world's rigid-body iteration order.
[[nodiscard]] DART_EXPERIMENTAL_API RigidBodyStateBatch
extractRigidBodyState(const World& world);

/// Apply a single-world snapshot back to the world's rigid bodies, matched by
/// iteration order. Throws if @p state is not single-world or its body count
/// does not match the world's rigid-body count.
DART_EXPERIMENTAL_API void applyRigidBodyState(
    World& world, const RigidBodyStateBatch& state);

/// Extract a homogeneous multi-world snapshot with a leading world dimension
/// (`worldCount == worlds.size()`), laid out world-major. All worlds must
/// expose the same rigid-body count; throws otherwise.
[[nodiscard]] DART_EXPERIMENTAL_API RigidBodyStateBatch
extractRigidBodyStateBatch(const std::vector<const World*>& worlds);

/// Apply a homogeneous multi-world snapshot back to @p worlds, slice by slice
/// in world-major order. Throws if `state.worldCount` does not match the number
/// of worlds or the array sizes are inconsistent with `worldCount * bodyCount`.
DART_EXPERIMENTAL_API void applyRigidBodyStateBatch(
    const std::vector<World*>& worlds, const RigidBodyStateBatch& state);

} // namespace dart::simulation::experimental::compute
