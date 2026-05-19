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

#include "dart/simulation/experimental/compute/compute_stage_metadata.hpp"

#include <array>

namespace dart::simulation::experimental::compute {

//==============================================================================
std::string_view toString(ComputeStageDomain domain) noexcept
{
  switch (domain) {
    case ComputeStageDomain::Generic:
      return "generic";
    case ComputeStageDomain::Simulation:
      return "simulation";
    case ComputeStageDomain::Kinematics:
      return "kinematics";
    case ComputeStageDomain::RigidBody:
      return "rigid_body";
    case ComputeStageDomain::ArticulatedBody:
      return "articulated_body";
    case ComputeStageDomain::DeformableBody:
      return "deformable_body";
    case ComputeStageDomain::Fluid:
      return "fluid";
    case ComputeStageDomain::Collision:
      return "collision";
    case ComputeStageDomain::Constraint:
      return "constraint";
    case ComputeStageDomain::Control:
      return "control";
    case ComputeStageDomain::Sensor:
      return "sensor";
    case ComputeStageDomain::Rendering:
      return "rendering";
    case ComputeStageDomain::Custom:
      return "custom";
  }

  return "unknown";
}

//==============================================================================
std::string_view toString(ComputeStageAcceleration acceleration) noexcept
{
  switch (acceleration) {
    case ComputeStageAcceleration::None:
      return "none";
    case ComputeStageAcceleration::TaskParallel:
      return "task_parallel";
    case ComputeStageAcceleration::DataParallel:
      return "data_parallel";
    case ComputeStageAcceleration::Simd:
      return "simd";
    case ComputeStageAcceleration::DataLocality:
      return "data_locality";
    case ComputeStageAcceleration::Gpu:
      return "gpu";
  }

  return "unknown";
}

//==============================================================================
std::string formatAccelerationMask(ComputeStageAccelerationMask mask)
{
  if (mask == toMask(ComputeStageAcceleration::None)) {
    return std::string(toString(ComputeStageAcceleration::None));
  }

  constexpr std::array accelerations{
      ComputeStageAcceleration::TaskParallel,
      ComputeStageAcceleration::DataParallel,
      ComputeStageAcceleration::Simd,
      ComputeStageAcceleration::DataLocality,
      ComputeStageAcceleration::Gpu,
  };

  std::string result;
  for (const auto acceleration : accelerations) {
    if (!hasAcceleration(mask, acceleration)) {
      continue;
    }

    if (!result.empty()) {
      result += "|";
    }
    result += toString(acceleration);
  }

  return result.empty() ? std::string("unknown") : result;
}

} // namespace dart::simulation::experimental::compute
