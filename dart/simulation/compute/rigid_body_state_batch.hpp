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

#include <dart/simulation/export.hpp>

#include <span>
#include <vector>

#include <cstddef>

namespace dart::simulation {
class World;
} // namespace dart::simulation

namespace dart::simulation::compute {

/// Batched structure-of-arrays snapshot of rigid-body state.
///
/// Each field is a flat, world-major @c double array sized
/// `worldCount * bodyCount` per body, so a leading world dimension can be added
/// without changing the layout: position/linear/angular use three components
/// and orientation uses four (w, x, y, z). Plain scalar arrays avoid Eigen
/// alignment constraints in containers and map directly onto SIMD lanes and
/// device buffers.
///
/// This is the canonical-direction seed of the batched state representation
/// that later multi-core, SIMD, and GPU paths consume: one immutable Model plus
/// mutable SoA State blocks with a leading world dimension. It is an internal
/// experimental value type; per-world public handles remain the user-facing
/// API.
struct DART_SIMULATION_API RigidBodyStateBatch
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

/// Immutable per-body model parameters for a batch, kept separate from the
/// mutable @c RigidBodyStateBatch.
///
/// This is the Model side of the Model/State split: built once and read during
/// integration. It carries the per-body inverse mass
/// (length `worldCount * bodyCount`) and body-frame inertia tensor (nine
/// row-major components per body, length `9 * worldCount * bodyCount`). The
/// inertia is only required by the torque-aware integrator overload; the
/// linear and torque-free paths ignore it.
struct DART_SIMULATION_API RigidBodyModelBatch
{
  std::size_t worldCount = 1;
  std::size_t bodyCount = 0;
  std::vector<double> inverseMass; ///< worldCount * bodyCount
  std::vector<double> inertia;     ///< 9 * worldCount * bodyCount (row-major)

  [[nodiscard]] bool isEmpty() const noexcept
  {
    return worldCount == 0 || bodyCount == 0;
  }
};

/// Extract a single-world (`worldCount == 1`) snapshot of all rigid bodies in
/// the World's baked dense rigid-body index order.
[[nodiscard]] DART_SIMULATION_API RigidBodyStateBatch
extractRigidBodyState(const World& world);

/// Apply a single-world snapshot back to the World's rigid bodies, matched by
/// baked dense index. Throws if @p state is not single-world or its body count
/// does not match the World's rigid-body count.
DART_SIMULATION_API void applyRigidBodyState(
    World& world, const RigidBodyStateBatch& state);

/// Extract a homogeneous multi-world snapshot with a leading world dimension
/// (`worldCount == worlds.size()`), laid out world-major. All worlds must
/// expose the same rigid-body count; throws otherwise.
[[nodiscard]] DART_SIMULATION_API RigidBodyStateBatch
extractRigidBodyStateBatch(const std::vector<const World*>& worlds);

/// Apply a homogeneous multi-world snapshot back to @p worlds, slice by slice
/// in world-major order. Throws if `state.worldCount` does not match the number
/// of worlds or the array sizes are inconsistent with `worldCount * bodyCount`.
DART_SIMULATION_API void applyRigidBodyStateBatch(
    const std::vector<World*>& worlds, const RigidBodyStateBatch& state);

/// Integrate a state batch one semi-implicit Euler step in place: linear
/// velocities are updated from @p force and @p inverseMass, then positions from
/// the updated velocities, via the scalar-generic SoA kernels.
///
/// @p force has three components per body (length `3 * worldCount * bodyCount`)
/// and @p inverseMass one per body (length `worldCount * bodyCount`); they are
/// kept out of the State batch as Control/Model inputs, foreshadowing the
/// Model/State split. Throws on size mismatch.
DART_SIMULATION_API void integrateRigidBodyStateBatchLinear(
    RigidBodyStateBatch& state,
    std::span<const double> force,
    std::span<const double> inverseMass,
    double timeStep);

/// Extract immutable model parameters from a single World in baked dense index
/// order. A non-positive or non-finite mass maps to zero inverse mass (a static
/// body).
[[nodiscard]] DART_SIMULATION_API RigidBodyModelBatch
extractRigidBodyModelBatch(const World& world);

/// Integrate a state batch one semi-implicit Euler linear step using an
/// explicit model. @p state and @p model must have matching world and body
/// counts and
/// @p force three components per body. Throws on mismatch.
DART_SIMULATION_API void integrateRigidBodyStateBatchLinear(
    RigidBodyStateBatch& state,
    const RigidBodyModelBatch& model,
    std::span<const double> force,
    double timeStep);

/// Integrate a state batch one full semi-implicit Euler kinematic step in
/// place: the linear step (velocity from @p force / @p model, then position)
/// followed by the orientation step (quaternion from the batch angular
/// velocity). Angular velocity is treated as given (no torque/inertia term
/// yet). Throws on size mismatch.
DART_SIMULATION_API void integrateRigidBodyStateBatch(
    RigidBodyStateBatch& state,
    const RigidBodyModelBatch& model,
    std::span<const double> force,
    double timeStep);

/// Integrate a state batch one full semi-implicit Euler dynamic step in place,
/// including the angular-velocity-from-torque update. The order matches the
/// per-entity integrator: linear velocity from @p force / @p model, angular
/// velocity from @p torque via the world-frame inertia (`R I R^T`, solved by
/// LDLT) using @p model 's inertia and the current orientation, position from
/// the updated linear velocity, then orientation from the updated angular
/// velocity. A non-finite torque/inertia or a non-positive-definite world
/// inertia leaves the angular velocity unchanged, as in the per-entity path.
/// @p torque has three components per body; @p model.inertia must be populated.
/// Throws on size mismatch.
DART_SIMULATION_API void integrateRigidBodyStateBatch(
    RigidBodyStateBatch& state,
    const RigidBodyModelBatch& model,
    std::span<const double> force,
    std::span<const double> torque,
    double timeStep);

/// Roll out a state batch over a control sequence on the SoA arrays, with no
/// World or device types involved.
///
/// Starting from @p initialState, for each per-step force batch in
/// @p controlSequence (each of length `3 * worldCount * bodyCount`), applies
/// one `integrateRigidBodyStateBatch` step and returns the final state. An
/// empty control sequence returns @p initialState unchanged. Throws on size
/// mismatch.
[[nodiscard]] DART_SIMULATION_API RigidBodyStateBatch
rolloutRigidBodyStateBatch(
    const RigidBodyStateBatch& initialState,
    const RigidBodyModelBatch& model,
    const std::vector<std::vector<double>>& controlSequence,
    double timeStep);

/// Total translational kinetic energy of a state batch: the sum over all bodies
/// of `0.5 * mass * |linearVelocity|^2`, with `mass = 1 / inverseMass` (bodies
/// with non-positive inverse mass are static and contribute nothing).
///
/// The sum is a floating-point reduction, which is non-associative, so it is
/// computed deterministically as fixed-size chunk partials merged in chunk
/// order. This is the reduction shape later parallel stages reuse: each chunk
/// can run on a separate thread, and the fixed merge order makes the result
/// independent of how chunks are scheduled, so it is identical across worker
/// counts. It differs from a naive left-to-right sum only by reduction order,
/// within a small relative tolerance (the Phase 3 reduction-determinism gate).
/// Throws on size mismatch.
[[nodiscard]] DART_SIMULATION_API double totalKineticEnergy(
    const RigidBodyStateBatch& state,
    const RigidBodyModelBatch& model,
    std::size_t chunkSize = 256);

} // namespace dart::simulation::compute
