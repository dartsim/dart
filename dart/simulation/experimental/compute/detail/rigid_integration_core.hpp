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

// Per-body scalar cores shared by the CPU batch kernels
// (rigid_body_integration_kernel.hpp) and the experimental CUDA batch kernels
// (compute/cuda/rigid_body_state_batch_cuda.cu). Single-sourcing the
// orientation update here removes the previously duplicated (and divergent)
// device re-derivation, so the CPU and GPU paths run one body of math. See
// docs/design/shared_cuda_device_substrate.md (PLAN-031).
//
// DART_EXPERIMENTAL_HD expands to nothing under a host compiler and to the CUDA
// host/device qualifiers when compiled by nvcc, so the same definitions serve
// the host batch loop and the device kernel. This is a detail/ header: it is
// excluded from Doxygen and from the compute-backend boundary scanner, so the
// device qualifiers never leak into a scanned public header. The spelling
// carries no backend token.

#if defined(__CUDACC__)
  #define DART_EXPERIMENTAL_HD __host__ __device__
#else
  #define DART_EXPERIMENTAL_HD
#endif

namespace dart::simulation::experimental::compute::detail {

/// Integrate body @p b's unit quaternion (stored w, x, y, z at
/// `orientations[4*b ...]`) from its body angular velocity
/// (`angularVelocities[3*b ...]`) by the exact rotation for a constant angular
/// velocity over @p timeStep: `q <- normalize(dq * q)` with
/// `dq = (cos(a/2), sin(a/2) * axis)`, `a = |omega| * timeStep`. A
/// non-normalizable result maps to identity, matching the per-entity
/// normalizeOrIdentity. Templated on @c Scalar (instantiated for @c double)
/// and using `using std::sin/cos/sqrt` so the host and device math libraries
/// are each selected by ADL.
template <typename Scalar>
DART_EXPERIMENTAL_HD inline void integrateOrientationSemiImplicitBody(
    Scalar* orientations,
    const Scalar* angularVelocities,
    Scalar timeStep,
    std::size_t b)
{
  using std::cos;
  using std::isfinite;
  using std::sin;
  using std::sqrt;
  const Scalar half = static_cast<Scalar>(0.5);

  const Scalar w = orientations[4 * b + 0];
  const Scalar x = orientations[4 * b + 1];
  const Scalar y = orientations[4 * b + 2];
  const Scalar z = orientations[4 * b + 3];
  const Scalar ox = angularVelocities[3 * b + 0];
  const Scalar oy = angularVelocities[3 * b + 1];
  const Scalar oz = angularVelocities[3 * b + 2];

  Scalar nw = w;
  Scalar nx = x;
  Scalar ny = y;
  Scalar nz = z;

  const Scalar speed = sqrt(ox * ox + oy * oy + oz * oz);
  if (speed > static_cast<Scalar>(0)) {
    // Angle-axis delta dq = (cos(a/2), sin(a/2) * axis); the sin(a/2)/|omega|
    // factor folds the axis normalization in without a separate divide.
    const Scalar halfAngle = half * speed * timeStep;
    const Scalar axisScale = sin(halfAngle) / speed;
    const Scalar dw = cos(halfAngle);
    const Scalar dx = ox * axisScale;
    const Scalar dy = oy * axisScale;
    const Scalar dz = oz * axisScale;

    // dq * q (Hamilton product), left-multiplying the world-frame delta.
    nw = dw * w - dx * x - dy * y - dz * z;
    nx = dw * x + dx * w + dy * z - dz * y;
    ny = dw * y - dx * z + dy * w + dz * x;
    nz = dw * z + dx * y - dy * x + dz * w;
  }

  const Scalar norm = sqrt(nw * nw + nx * nx + ny * ny + nz * nz);
  if (norm > static_cast<Scalar>(0) && isfinite(norm)) {
    const Scalar inverse = static_cast<Scalar>(1) / norm;
    nw *= inverse;
    nx *= inverse;
    ny *= inverse;
    nz *= inverse;
  } else {
    nw = static_cast<Scalar>(1);
    nx = static_cast<Scalar>(0);
    ny = static_cast<Scalar>(0);
    nz = static_cast<Scalar>(0);
  }

  orientations[4 * b + 0] = nw;
  orientations[4 * b + 1] = nx;
  orientations[4 * b + 2] = ny;
  orientations[4 * b + 3] = nz;
}

} // namespace dart::simulation::experimental::compute::detail

// Keep the host/device qualifier macro local to this header.
#undef DART_EXPERIMENTAL_HD
