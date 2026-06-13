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

#include <span>

#include <cstddef>

namespace dart::simulation::compute::rigid_body_batch_ops {

struct MutableRigidBodyStateBatchView
{
  std::size_t worldCount = 1;
  std::size_t bodyCount = 0;
  std::span<double> position;
  std::span<double> orientation;
  std::span<double> linearVelocity;
  std::span<double> angularVelocity;
};

struct RigidBodyModelBatchView
{
  std::size_t worldCount = 1;
  std::size_t bodyCount = 0;
  std::span<const double> inverseMass;
  std::span<const double> inertia;
};

template <typename StateBatch>
MutableRigidBodyStateBatchView mutableStateBatchView(StateBatch& state)
{
  return {
      state.worldCount,
      state.bodyCount,
      std::span<double>{state.position.data(), state.position.size()},
      std::span<double>{state.orientation.data(), state.orientation.size()},
      std::span<double>{
          state.linearVelocity.data(), state.linearVelocity.size()},
      std::span<double>{
          state.angularVelocity.data(), state.angularVelocity.size()}};
}

template <typename ModelBatch>
RigidBodyModelBatchView modelBatchView(const ModelBatch& model)
{
  return {
      model.worldCount,
      model.bodyCount,
      std::span<const double>{
          model.inverseMass.data(), model.inverseMass.size()},
      std::span<const double>{model.inertia.data(), model.inertia.size()}};
}

void integrateRigidBodyStateBatchLinear(
    MutableRigidBodyStateBatchView state,
    std::span<const double> force,
    std::span<const double> inverseMass,
    double timeStep);

void integrateRigidBodyStateBatchLinear(
    MutableRigidBodyStateBatchView state,
    RigidBodyModelBatchView model,
    std::span<const double> force,
    double timeStep);

void integrateRigidBodyStateBatch(
    MutableRigidBodyStateBatchView state,
    RigidBodyModelBatchView model,
    std::span<const double> force,
    double timeStep);

void integrateRigidBodyStateBatch(
    MutableRigidBodyStateBatchView state,
    RigidBodyModelBatchView model,
    std::span<const double> force,
    std::span<const double> torque,
    double timeStep);

} // namespace dart::simulation::compute::rigid_body_batch_ops
