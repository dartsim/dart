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

#include <dart/simulation/experimental/dynamics/spatial_math.hpp>
#include <dart/simulation/experimental/export.hpp>

#include <Eigen/Dense>

#include <vector>

#include <cstddef>

namespace dart::simulation::experimental::dynamics {

/// Per-link ABA intermediate quantities
/// These are computed during the ABA backward/forward passes
struct DART_EXPERIMENTAL_API LinkABAData
{
  SpatialInertia articulatedInertia = SpatialInertia::Zero();
  SpatialForce biasForce = SpatialForce::Zero();
  SpatialVelocity spatialVelocity = SpatialVelocity::Zero();
  SpatialAcceleration spatialAcceleration = SpatialAcceleration::Zero();
  SpatialAcceleration partialAcceleration = SpatialAcceleration::Zero();
  SpatialForce transmittedForce = SpatialForce::Zero();
};

/// Per-joint ABA intermediate quantities
struct DART_EXPERIMENTAL_API JointABAData
{
  Eigen::MatrixXd projectedInertiaInverse;
  Eigen::VectorXd totalForce;

  void resize(std::size_t dof)
  {
    projectedInertiaInverse.resize(dof, dof);
    totalForce.resize(dof);
    projectedInertiaInverse.setZero();
    totalForce.setZero();
  }
};

/// Workspace for ABA computation on a single MultiBody
/// Pre-allocated to avoid per-step allocations
class DART_EXPERIMENTAL_API ABAWorkspace
{
public:
  void resize(std::size_t numLinks, const std::vector<std::size_t>& jointDOFs);

  void reset();

  [[nodiscard]] std::size_t getNumLinks() const
  {
    return m_linkData.size();
  }

  [[nodiscard]] LinkABAData& getLinkData(std::size_t index)
  {
    return m_linkData[index];
  }

  [[nodiscard]] const LinkABAData& getLinkData(std::size_t index) const
  {
    return m_linkData[index];
  }

  [[nodiscard]] JointABAData& getJointData(std::size_t index)
  {
    return m_jointData[index];
  }

  [[nodiscard]] const JointABAData& getJointData(std::size_t index) const
  {
    return m_jointData[index];
  }

private:
  std::vector<LinkABAData> m_linkData;
  std::vector<JointABAData> m_jointData;
};

} // namespace dart::simulation::experimental::dynamics
