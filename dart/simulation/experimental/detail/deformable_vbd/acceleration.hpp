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

#include <dart/simulation/experimental/detail/deformable_vbd/vertex_block_kernel.hpp>

#include <Eigen/Core>

#include <algorithm>

#include <cmath>

namespace dart::simulation::experimental::detail::deformable_vbd {

//==============================================================================
/// VBD adaptive initial guess for a vertex at the start of a timestep.
///
/// Instead of warm-starting the optimization at the full inertial prediction
/// `x^t + h v^t + h^2 a_ext`, VBD blends in only the component of the previous
/// step's acceleration that points along the external-force direction, clamped
/// to `[0, ||a_ext||]` (TinyVBD `forwardStep`):
///   a_prev   = (v^t - v^{t-1}) / h
///   a_tilde  = clamp(a_prev . g_hat, 0, ||a_ext||)         (g_hat = a_ext dir)
///   x_init   = x^t + h v^t + h^2 g_hat a_tilde
/// This reduces the residual the descent has to remove when the motion is in
/// near-free-fall while avoiding overshoot when the body is supported.
///
/// `hasPreviousVelocity` should be false on the very first step (no `v^{t-1}`),
/// in which case the full inertial prediction is returned.
inline Eigen::Vector3d adaptiveInitialPosition(
    const Eigen::Vector3d& position,
    const Eigen::Vector3d& velocity,
    const Eigen::Vector3d& previousVelocity,
    const Eigen::Vector3d& externalAcceleration,
    double timeStep,
    bool hasPreviousVelocity)
{
  const Eigen::Vector3d inertial = position + timeStep * velocity
                                   + timeStep * timeStep * externalAcceleration;

  const double externalMagnitude = externalAcceleration.norm();
  if (!hasPreviousVelocity || externalMagnitude <= 0.0) {
    return inertial;
  }

  const Eigen::Vector3d direction = externalAcceleration / externalMagnitude;
  const Eigen::Vector3d previousAcceleration
      = (velocity - previousVelocity) / timeStep;
  const double projected = previousAcceleration.dot(direction);
  const double blended = std::clamp(projected, 0.0, externalMagnitude);

  return position + timeStep * velocity
         + timeStep * timeStep * direction * blended;
}

//==============================================================================
/// Chebyshev semi-iterative acceleration weight for VBD iteration `iteration`
/// (1-based), given the estimated spectral radius `rho` in (0, 1) and the
/// previous iteration's weight. The recurrence is (TinyVBD
/// `getAcceleratorOmega`):
///   omega_1 = 1
///   omega_2 = 2 / (2 - rho^2)
///   omega_n = 4 / (4 - rho^2 omega_{n-1})   (n >= 3)
inline double chebyshevOmega(
    std::size_t iteration, double rho, double previousOmega)
{
  if (iteration <= 1) {
    return 1.0;
  }
  if (iteration == 2) {
    return 2.0 / (2.0 - rho * rho);
  }
  return 4.0 / (4.0 - rho * rho * previousOmega);
}

//==============================================================================
/// Apply Chebyshev acceleration to a freshly-swept position. Given the new
/// iterate `swept` (= x_bar^n), the iterate two steps back `twoStepsBack`
/// (= x^{n-2}), and the weight `omega`, returns
///   x^n = omega (x_bar^n - x^{n-2}) + x^{n-2}.
/// The weight is only over-relaxing when `omega > 1`; callers should leave the
/// swept value unchanged otherwise.
inline Eigen::Vector3d applyChebyshev(
    double omega,
    const Eigen::Vector3d& swept,
    const Eigen::Vector3d& twoStepsBack)
{
  if (omega <= 1.0) {
    return swept;
  }
  return omega * (swept - twoStepsBack) + twoStepsBack;
}

//==============================================================================
/// Add stiffness-proportional Rayleigh damping for a vertex to `block`.
///
/// Given the vertex's accumulated *elastic* Hessian block `elasticHessian`
/// (excluding the inertia term), the per-step displacement
/// `displacement = x_i - x_i^t`, the damping coefficient `dampingCoeff`
/// (`k_d`), and the timestep, this adds (reference `VBD_NeoHookean.cpp`):
///   H_i += (k_d / h) elasticHessian
///   f_i -= (k_d / h) elasticHessian displacement
/// The damping force opposes motion through the elastic stiffness, so the
/// per-step work `f_damp . displacement = -(k_d/h) d^T H_el d <= 0` is
/// dissipative for a positive-semidefinite elastic Hessian.
inline void addRayleighDamping(
    VertexBlock& block,
    const Eigen::Matrix3d& elasticHessian,
    const Eigen::Vector3d& displacement,
    double dampingCoeff,
    double timeStep)
{
  if (dampingCoeff <= 0.0) {
    return;
  }
  const double scale = dampingCoeff / timeStep;
  block.hessian.noalias() += scale * elasticHessian;
  block.force.noalias() -= scale * (elasticHessian * displacement);
}

} // namespace dart::simulation::experimental::detail::deformable_vbd
