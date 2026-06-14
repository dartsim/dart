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
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR
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

#include <dart/simulation/detail/deformable_vbd/acceleration.hpp>
#include <dart/simulation/detail/deformable_vbd/block_descent.hpp>

#include <Eigen/Core>

#include <span>
#include <vector>

#include <cstddef>
#include <cstdint>

namespace dart::simulation::detail::deformable_vbd {

/// Tuning for one VBD implicit-Euler step.
struct VbdStepOptions
{
  std::size_t iterations = 20;    ///< Block-descent sweeps over all colors.
  double regularization = 0.0;    ///< Diagonal damping added to each block.
  bool clampSpringHessian = true; ///< PSD-project the spring Hessian blocks.
  bool useAdaptiveInit = true; ///< Adaptive inertial/previous-step warm start.
  bool useChebyshev = false;   ///< Chebyshev over-relaxation across sweeps.
  double chebyshevRho = 0.95;  ///< Estimated spectral radius in (0, 1).
};

/// Outcome of one VBD step.
struct VbdStepResult
{
  BlockDescentStats descentStats;
};

//==============================================================================
/// Advance a mass-spring body by one VBD implicit-Euler step.
///
/// The step (1) computes the inertial target `y_i = x_i^t + h v_i + h^2 g` for
/// free vertices, (2) warm-starts the optimization with the adaptive initial
/// guess, (3) runs graph-colored Gauss-Seidel block descent on the variational
/// objective, optionally over-relaxing each sweep with Chebyshev acceleration,
/// and (4) updates velocities `v_i = (x_i - x_i^t) / h`. `previousVelocities`
/// is advanced to the pre-step velocities so the next step's adaptive guess can
/// use `v^{t-1}`.
///
/// Fixed vertices are held in place (zero velocity). `coloring`/`adjacency`
/// must be built for `springs`. This is single-body CPU stepping; element
/// damping and World-stage integration are not included here.
inline VbdStepResult vbdStepMassSpring(
    std::vector<Eigen::Vector3d>& positions,
    std::vector<Eigen::Vector3d>& velocities,
    std::vector<Eigen::Vector3d>& previousVelocities,
    std::span<const double> masses,
    const std::vector<std::uint8_t>& fixed,
    std::span<const SpringElement> springs,
    double springStiffness,
    const Eigen::Vector3d& gravity,
    double timeStep,
    const VertexColoring& coloring,
    const SpringAdjacency& adjacency,
    const VbdStepOptions& options,
    bool hasPreviousVelocities)
{
  const std::size_t vertexCount = positions.size();
  const std::vector<Eigen::Vector3d> stepStartPositions = positions;
  const std::vector<Eigen::Vector3d> stepStartVelocities = velocities;

  // Inertial targets y_i and the adaptive warm start.
  std::vector<Eigen::Vector3d> inertialTargets = positions;
  for (std::size_t i = 0; i < vertexCount; ++i) {
    if (fixed[i] != 0u) {
      continue;
    }
    inertialTargets[i] = stepStartPositions[i] + timeStep * velocities[i]
                         + timeStep * timeStep * gravity;
    if (options.useAdaptiveInit) {
      positions[i] = adaptiveInitialPosition(
          stepStartPositions[i],
          velocities[i],
          previousVelocities[i],
          gravity,
          timeStep,
          hasPreviousVelocities);
    } else {
      positions[i] = inertialTargets[i];
    }
  }

  BlockDescentOptions descentOptions;
  descentOptions.iterations = 1;
  descentOptions.regularization = options.regularization;
  descentOptions.clampSpringHessian = options.clampSpringHessian;

  VbdStepResult result;
  std::vector<Eigen::Vector3d> twoStepsBack = positions;
  double omega = 1.0;
  for (std::size_t sweep = 1; sweep <= options.iterations; ++sweep) {
    const std::vector<Eigen::Vector3d> beforeSweep = positions;
    const BlockDescentStats stats = blockDescentMassSpring(
        positions,
        masses,
        fixed,
        inertialTargets,
        springs,
        springStiffness,
        timeStep,
        coloring,
        adjacency,
        descentOptions);
    result.descentStats.iterations += stats.iterations;
    result.descentStats.vertexUpdates += stats.vertexUpdates;
    result.descentStats.finalResidualNormSquared
        = stats.finalResidualNormSquared;

    if (options.useChebyshev) {
      omega = chebyshevOmega(sweep, options.chebyshevRho, omega);
      if (omega > 1.0) {
        for (std::size_t i = 0; i < vertexCount; ++i) {
          if (fixed[i] == 0u) {
            positions[i] = applyChebyshev(omega, positions[i], twoStepsBack[i]);
          }
        }
      }
      twoStepsBack = beforeSweep;
    }
  }

  // Velocity update and previous-velocity bookkeeping.
  for (std::size_t i = 0; i < vertexCount; ++i) {
    previousVelocities[i] = stepStartVelocities[i];
    if (fixed[i] != 0u) {
      velocities[i].setZero();
      positions[i] = stepStartPositions[i];
      continue;
    }
    velocities[i] = (positions[i] - stepStartPositions[i]) / timeStep;
  }
  return result;
}

} // namespace dart::simulation::detail::deformable_vbd
