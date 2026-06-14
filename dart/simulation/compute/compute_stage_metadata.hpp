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

#include <dart/common/memory_allocator.hpp>
#include <dart/common/stl_allocator.hpp>

#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <cstdint>

namespace dart::simulation::compute {

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

/// How a compute stage accesses a named resource.
///
/// This metadata is descriptive. It powers hazard validation, visualization,
/// and profiling context, but explicit graph dependencies remain the
/// correctness source of truth.
enum class ComputeAccessMode : std::uint8_t
{
  Read,      ///< Reads a resource without mutating it.
  Write,     ///< Overwrites a resource; does not depend on the prior value.
  ReadWrite, ///< Reads and mutates a resource in place.
  Reduce,    ///< Contributes to a deterministic reduction shared with peers.
  Scratch, ///< Node-local temporary; carries no inter-node dependency meaning.
};

/// A declared access to a named resource by a compute stage.
///
/// Resource identifiers are stable strings in this first iteration, for example
/// component or stage names such as "Transform" or "FrameCache".
using ComputeResourceString = std::basic_string<
    char,
    std::char_traits<char>,
    dart::common::StlAllocator<char>>;

struct DART_SIMULATION_API ComputeResourceAccess
{
  ComputeResourceAccess() = default;

  ComputeResourceAccess(
      std::string_view resourceName,
      ComputeAccessMode accessMode,
      dart::common::MemoryAllocator& allocator
      = dart::common::MemoryAllocator::GetDefault())
    : resource(resourceName, dart::common::StlAllocator<char>{allocator}),
      mode(accessMode)
  {
    // Empty.
  }

  ComputeResourceAccess(const char* resourceName, ComputeAccessMode accessMode)
    : ComputeResourceAccess(std::string_view{resourceName}, accessMode)
  {
    // Empty.
  }

  ComputeResourceAccess(
      ComputeResourceString&& resourceName, ComputeAccessMode accessMode)
    : resource(std::move(resourceName)), mode(accessMode)
  {
    // Empty.
  }

  ComputeResourceString resource;
  ComputeAccessMode mode = ComputeAccessMode::Read;
};

using ComputeResourceAccessAllocator
    = dart::common::StlAllocator<ComputeResourceAccess>;
using ComputeResourceAccessVector
    = std::vector<ComputeResourceAccess, ComputeResourceAccessAllocator>;

/// Stage metadata used by debugging, profiling, and graph-shaping heuristics.
struct DART_SIMULATION_API ComputeStageMetadata
{
  ComputeStageMetadata()
    : resources(
          ComputeResourceAccessAllocator{
              dart::common::MemoryAllocator::GetDefault()})
  {
    // Empty.
  }

  explicit ComputeStageMetadata(dart::common::MemoryAllocator& allocator)
    : resources(ComputeResourceAccessAllocator{allocator})
  {
    // Empty.
  }

  ComputeStageMetadata(
      ComputeStageDomain stageDomain,
      dart::common::MemoryAllocator& allocator
      = dart::common::MemoryAllocator::GetDefault())
    : domain(stageDomain), resources(ComputeResourceAccessAllocator{allocator})
  {
    // Empty.
  }

  ComputeStageMetadata(
      ComputeStageDomain stageDomain,
      ComputeStageAccelerationMask accelerationMask,
      dart::common::MemoryAllocator& allocator
      = dart::common::MemoryAllocator::GetDefault())
    : domain(stageDomain),
      acceleration(accelerationMask),
      resources(ComputeResourceAccessAllocator{allocator})
  {
    // Empty.
  }

  ComputeStageMetadata(
      ComputeStageDomain stageDomain,
      ComputeStageAccelerationMask accelerationMask,
      ComputeResourceAccessVector resourceAccesses)
    : domain(stageDomain),
      acceleration(accelerationMask),
      resources(std::move(resourceAccesses))
  {
    // Empty.
  }

  ComputeStageDomain domain = ComputeStageDomain::Generic;
  ComputeStageAccelerationMask acceleration = 0u;
  ComputeResourceAccessVector resources;
};

/// Returns true when two accesses to the same resource form a data hazard if
/// the accessing nodes are not ordered by an explicit dependency.
///
/// Read/read sharing, scratch access, and mutually declared reductions are not
/// hazards; any other combination involving a write or mutation is.
[[nodiscard]] constexpr bool accessesConflict(
    ComputeAccessMode lhs, ComputeAccessMode rhs) noexcept
{
  if (lhs == ComputeAccessMode::Scratch || rhs == ComputeAccessMode::Scratch) {
    return false;
  }
  if (lhs == ComputeAccessMode::Read && rhs == ComputeAccessMode::Read) {
    return false;
  }
  if (lhs == ComputeAccessMode::Reduce && rhs == ComputeAccessMode::Reduce) {
    return false;
  }
  return true;
}

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

[[nodiscard]] DART_SIMULATION_API std::string_view toString(
    ComputeStageDomain domain) noexcept;

[[nodiscard]] DART_SIMULATION_API std::string_view toString(
    ComputeStageAcceleration acceleration) noexcept;

[[nodiscard]] DART_SIMULATION_API std::string formatAccelerationMask(
    ComputeStageAccelerationMask mask);

[[nodiscard]] DART_SIMULATION_API std::string_view toString(
    ComputeAccessMode mode) noexcept;

} // namespace dart::simulation::compute
