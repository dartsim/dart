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

#include <cmath>
#include <cstddef>

namespace dart::simulation::experimental::compute {

/// Scalar-generic semi-implicit Euler position update over flat, world-major
/// structure-of-arrays state (three components per body, matching
/// @c RigidBodyStateBatch).
///
/// The kernel is templated on @c Scalar so the same code can later be
/// instantiated for an autodiff scalar (Jet/dual) or a wide SIMD batch type,
/// not only @c double. It deliberately operates on plain scalar pointers and
/// uses no Eigen expression templates, so it has no container-alignment or
/// autodiff-vectorization pitfalls and maps directly onto SIMD lanes and device
/// buffers. Production code instantiates it for @c double.
///
/// @param positions       In/out array of length @p componentCount.
/// @param velocities      Input array of length @p componentCount.
/// @param timeStep        Integration step size.
/// @param componentCount  Number of scalar components (3 * worldCount *
/// bodyCount
///                        for a @c RigidBodyStateBatch position array).
template <typename Scalar>
void integratePositionsSemiImplicit(
    Scalar* positions,
    const Scalar* velocities,
    Scalar timeStep,
    std::size_t componentCount)
{
  for (std::size_t i = 0; i < componentCount; ++i) {
    positions[i] += velocities[i] * timeStep;
  }
}

/// Scalar-generic semi-implicit Euler linear-velocity update over flat,
/// world-major structure-of-arrays state.
///
/// Applies `velocity += force * inverseMass * timeStep` per body. Velocities
/// and forces are three components per body (length `3 * bodyCount`); inverse
/// masses are one per body (length `bodyCount`). Keeping force (control) and
/// inverse mass (model) as separate inputs mirrors the intended Model/Control
/// vs State separation. Like the position kernel, it is templated on @c Scalar
/// and uses plain scalar pointers with no Eigen expression templates.
template <typename Scalar>
void integrateVelocitiesSemiImplicit(
    Scalar* velocities,
    const Scalar* forces,
    const Scalar* inverseMasses,
    Scalar timeStep,
    std::size_t bodyCount)
{
  for (std::size_t b = 0; b < bodyCount; ++b) {
    const Scalar scale = inverseMasses[b] * timeStep;
    velocities[3 * b + 0] += forces[3 * b + 0] * scale;
    velocities[3 * b + 1] += forces[3 * b + 1] * scale;
    velocities[3 * b + 2] += forces[3 * b + 2] * scale;
  }
}

/// Scalar-generic semi-implicit Euler orientation update over flat, world-major
/// structure-of-arrays state.
///
/// Integrates each unit quaternion `q` (stored w, x, y, z, length
/// `4 * bodyCount`) from its body angular velocity (length `3 * bodyCount`):
/// `q <- normalize(q + 0.5 * (0, omega) * q * timeStep)`. It is templated on
/// @c Scalar and uses `using std::sqrt` so an autodiff or wide scalar can
/// supply its own `sqrt` via ADL.
template <typename Scalar>
void integrateOrientationsSemiImplicit(
    Scalar* orientations,
    const Scalar* angularVelocities,
    Scalar timeStep,
    std::size_t bodyCount)
{
  using std::sqrt;
  const Scalar half = static_cast<Scalar>(0.5);
  for (std::size_t b = 0; b < bodyCount; ++b) {
    const Scalar w = orientations[4 * b + 0];
    const Scalar x = orientations[4 * b + 1];
    const Scalar y = orientations[4 * b + 2];
    const Scalar z = orientations[4 * b + 3];
    const Scalar ox = angularVelocities[3 * b + 0];
    const Scalar oy = angularVelocities[3 * b + 1];
    const Scalar oz = angularVelocities[3 * b + 2];

    // dq = 0.5 * (0, omega) * q (quaternion product).
    const Scalar dw = -(ox * x + oy * y + oz * z);
    const Scalar dx = ox * w + oy * z - oz * y;
    const Scalar dy = oy * w + oz * x - ox * z;
    const Scalar dz = oz * w + ox * y - oy * x;

    Scalar nw = w + half * dw * timeStep;
    Scalar nx = x + half * dx * timeStep;
    Scalar ny = y + half * dy * timeStep;
    Scalar nz = z + half * dz * timeStep;

    const Scalar norm = sqrt(nw * nw + nx * nx + ny * ny + nz * nz);
    if (norm > static_cast<Scalar>(0)) {
      const Scalar inverse = static_cast<Scalar>(1) / norm;
      nw *= inverse;
      nx *= inverse;
      ny *= inverse;
      nz *= inverse;
    }

    orientations[4 * b + 0] = nw;
    orientations[4 * b + 1] = nx;
    orientations[4 * b + 2] = ny;
    orientations[4 * b + 3] = nz;
  }
}

} // namespace dart::simulation::experimental::compute
