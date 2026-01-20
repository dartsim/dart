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

#include <dart/simulation/experimental/dynamics/articulated_body.hpp>
#include <dart/simulation/experimental/dynamics/motion_subspace.hpp>
#include <dart/simulation/experimental/dynamics/spatial_math.hpp>
#include <dart/simulation/experimental/export.hpp>

#include <Eigen/Dense>

#include <span>
#include <vector>

namespace dart::simulation::experimental {
class World;
class MultiBody;
} // namespace dart::simulation::experimental

namespace dart::simulation::experimental::dynamics {

/// Configuration for forward dynamics computation
struct DART_EXPERIMENTAL_API ForwardDynamicsConfig
{
  Eigen::Vector3d gravity = Eigen::Vector3d(0.0, 0.0, -9.81);
  double timeStep = 0.001;
  bool useImplicitDamping = true;
};

/// Forward dynamics system using Articulated Body Algorithm (ABA)
///
/// Implements Featherstone's ABA for computing joint accelerations:
///   ddq = M^{-1} * (tau - C(q, dq) - g(q))
///
/// The algorithm runs in O(n) time where n is the number of links.
class DART_EXPERIMENTAL_API ForwardDynamicsSystem
{
public:
  ForwardDynamicsSystem() = default;
  explicit ForwardDynamicsSystem(const ForwardDynamicsConfig& config);

  void setConfig(const ForwardDynamicsConfig& config)
  {
    m_config = config;
  }
  [[nodiscard]] const ForwardDynamicsConfig& getConfig() const
  {
    return m_config;
  }

  /// Compute forward dynamics for a single MultiBody
  /// Updates joint accelerations based on current state and applied torques
  void compute(World& world, MultiBody& multiBody);

  /// Compute forward dynamics for all MultiBodies in a World
  void computeAll(World& world);

private:
  void initializeWorkspace(const MultiBody& multiBody);

  void computeVelocities(
      World& world,
      const MultiBody& multiBody,
      std::span<const Eigen::Isometry3d> linkTransforms);

  void computeArticulatedInertias(
      World& world,
      const MultiBody& multiBody,
      std::span<const Eigen::Isometry3d> linkTransforms);

  void computeBiasForces(
      World& world,
      const MultiBody& multiBody,
      std::span<const Eigen::Isometry3d> linkTransforms);

  void computeAccelerations(
      World& world,
      MultiBody& multiBody,
      std::span<const Eigen::Isometry3d> linkTransforms);

  ForwardDynamicsConfig m_config;
  ABAWorkspace m_workspace;
  std::vector<Eigen::Isometry3d> m_linkTransforms;
  std::vector<MotionSubspace> m_motionSubspaces;
};

} // namespace dart::simulation::experimental::dynamics
