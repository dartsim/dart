/*
 * Copyright (c) 2011-2025, The DART development contributors
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

#ifndef DART_SIMULATION_DETAIL_LEGACYSKELETONSYNC_HPP_
#define DART_SIMULATION_DETAIL_LEGACYSKELETONSYNC_HPP_

#include <dart/simulation/comps/SkeletonComponents.hpp>

#include <dart/dynamics/Skeleton.hpp>

#include <algorithm>

#include <cstddef>

namespace dart::simulation::detail {

inline void syncLegacySkeletonState(
    comps::SkeletonState& state, const dynamics::Skeleton& skeleton)
{
  const auto dofs = static_cast<std::size_t>(skeleton.getNumDofs());
  state.positions.assign(dofs, 0.0);
  state.velocities.assign(dofs, 0.0);

  const auto& positions = skeleton.getPositions();
  const auto& velocities = skeleton.getVelocities();

  const auto positionsCount = static_cast<std::size_t>(positions.size());
  const auto velocitiesCount = static_cast<std::size_t>(velocities.size());

  const auto copyCount = std::min({dofs, positionsCount, velocitiesCount});
  std::copy_n(positions.data(), copyCount, state.positions.begin());
  std::copy_n(velocities.data(), copyCount, state.velocities.begin());
}

} // namespace dart::simulation::detail

#endif // DART_SIMULATION_DETAIL_LEGACYSKELETONSYNC_HPP_
