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

#include <string>
#include <string_view>

#include <cstdint>

namespace dart::simulation::experimental::compute {

/// Broad domain handled by a compute stage.
///
/// The domains are intentionally not limited to physics. They describe where a
/// stage participates today while leaving room for long-term rendering and
/// user-defined pipelines to reuse the same compute-graph substrate.
enum class ComputeStageDomain
{
  Generic,
  Simulation,
  Kinematics,
  RigidBody,
  ArticulatedBody,
  DeformableBody,
  Fluid,
  Collision,
  Constraint,
  Control,
  Sensor,
  Rendering,
  Custom,
};

/// Acceleration opportunities advertised by a compute stage.
enum class ComputeStageAcceleration : std::uint32_t
{
  None = 0u,
  TaskParallel = 1u << 0u,
  DataParallel = 1u << 1u,
  Simd = 1u << 2u,
  DataLocality = 1u << 3u,
  Gpu = 1u << 4u,
};

using ComputeStageAccelerationMask = std::uint32_t;

/// Stage metadata used by debugging, profiling, and graph-shaping heuristics.
struct DART_EXPERIMENTAL_API ComputeStageMetadata
{
  ComputeStageDomain domain = ComputeStageDomain::Generic;
  ComputeStageAccelerationMask acceleration = 0u;
};

[[nodiscard]] constexpr ComputeStageAccelerationMask toMask(
    ComputeStageAcceleration acceleration) noexcept
{
  return static_cast<ComputeStageAccelerationMask>(acceleration);
}

[[nodiscard]] constexpr ComputeStageAccelerationMask operator|(
    ComputeStageAcceleration lhs, ComputeStageAcceleration rhs) noexcept
{
  return toMask(lhs) | toMask(rhs);
}

[[nodiscard]] constexpr ComputeStageAccelerationMask operator|(
    ComputeStageAccelerationMask lhs, ComputeStageAcceleration rhs) noexcept
{
  return lhs | toMask(rhs);
}

[[nodiscard]] constexpr bool hasAcceleration(
    ComputeStageAccelerationMask mask,
    ComputeStageAcceleration acceleration) noexcept
{
  return (mask & toMask(acceleration)) != 0u;
}

[[nodiscard]] DART_EXPERIMENTAL_API std::string_view toString(
    ComputeStageDomain domain) noexcept;

[[nodiscard]] DART_EXPERIMENTAL_API std::string_view toString(
    ComputeStageAcceleration acceleration) noexcept;

[[nodiscard]] DART_EXPERIMENTAL_API std::string formatAccelerationMask(
    ComputeStageAccelerationMask mask);

} // namespace dart::simulation::experimental::compute
