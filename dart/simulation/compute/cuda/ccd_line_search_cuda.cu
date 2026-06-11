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
 *     copyright notice, this list of conditions in the documentation
 *     and/or other materials provided with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *   ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 *   TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 *   THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 *   SUCH DAMAGE.
 */

#include <cuda_runtime.h>
#include <dart/simulation/compute/cuda/ccd_line_search_cuda.cuh>
#include <dart/simulation/compute/cuda/cuda_runtime.cuh>

#include <cstddef>
#include <cstdint>

namespace dart::simulation::compute::cuda::detail {
namespace {

struct Vec3
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

__device__ Vec3 operator-(const Vec3 a, const Vec3 b)
{
  return Vec3{a.x - b.x, a.y - b.y, a.z - b.z};
}

__device__ Vec3 operator+(const Vec3 a, const Vec3 b)
{
  return Vec3{a.x + b.x, a.y + b.y, a.z + b.z};
}

__device__ Vec3 operator*(const double scale, const Vec3 value)
{
  return Vec3{scale * value.x, scale * value.y, scale * value.z};
}

__device__ double dot(const Vec3 a, const Vec3 b)
{
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

__device__ Vec3 cross(const Vec3 a, const Vec3 b)
{
  return Vec3{
      a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

__device__ double squaredNorm(const Vec3 value)
{
  return dot(value, value);
}

__device__ Vec3 loadVec3(const double values[3])
{
  return Vec3{values[0], values[1], values[2]};
}

__device__ bool pointInTriangle(
    const Vec3 point, const Vec3 a, const Vec3 b, const Vec3 c)
{
  const Vec3 v0 = b - a;
  const Vec3 v1 = c - a;
  const Vec3 v2 = point - a;
  const double d00 = dot(v0, v0);
  const double d01 = dot(v0, v1);
  const double d11 = dot(v1, v1);
  const double d20 = dot(v2, v0);
  const double d21 = dot(v2, v1);
  const double denom = d00 * d11 - d01 * d01;
  if (fabs(denom) <= 1e-24) {
    return false;
  }

  const double v = (d11 * d20 - d01 * d21) / denom;
  const double w = (d00 * d21 - d01 * d20) / denom;
  const double u = 1.0 - v - w;
  constexpr double kTolerance = 1e-12;
  return u >= -kTolerance && v >= -kTolerance && w >= -kTolerance;
}

__global__ void pointTriangleCcdLineSearchKernel(
    const PointTriangleCcdLineSearchPair* pairs,
    const double minSeparation,
    const double tolerance,
    double* stepBounds,
    std::uint8_t* hits,
    std::uint8_t* indeterminate,
    const std::size_t pairCount)
{
  const auto index
      = static_cast<std::size_t>(blockIdx.x * blockDim.x + threadIdx.x);
  if (index >= pairCount) {
    return;
  }

  stepBounds[index] = 1.0;
  hits[index] = 0u;
  indeterminate[index] = 0u;

  const PointTriangleCcdLineSearchPair pair = pairs[index];
  const Vec3 p0 = loadVec3(pair.pointStart);
  const Vec3 p1 = loadVec3(pair.pointEnd);
  const Vec3 a = loadVec3(pair.triangleA);
  const Vec3 b = loadVec3(pair.triangleB);
  const Vec3 c = loadVec3(pair.triangleC);
  const Vec3 normal = cross(b - a, c - a);
  const double normalNorm = sqrt(squaredNorm(normal));
  if (normalNorm <= 1e-12) {
    indeterminate[index] = 1u;
    stepBounds[index] = 0.0;
    return;
  }

  const double signedStart = dot(p0 - a, normal) / normalNorm;
  const double signedEnd = dot(p1 - a, normal) / normalNorm;
  const double threshold = fmax(0.0, minSeparation);
  const double effectiveTolerance = fmax(0.0, tolerance);

  if (signedStart <= threshold + effectiveTolerance) {
    if (pointInTriangle(p0, a, b, c)) {
      hits[index] = 1u;
      stepBounds[index] = 0.0;
    }
    return;
  }

  if (signedEnd > threshold + effectiveTolerance) {
    return;
  }

  const double denom = signedStart - signedEnd;
  if (denom <= 0.0) {
    return;
  }

  const double alpha = (signedStart - threshold) / denom;
  if (!(alpha >= 0.0 && alpha <= 1.0)) {
    return;
  }

  const Vec3 pointAtImpact = p0 + alpha * (p1 - p0);
  if (!pointInTriangle(pointAtImpact, a, b, c)) {
    return;
  }

  const double conservativeScale = fmax(0.0, 1.0 - effectiveTolerance);
  hits[index] = 1u;
  stepBounds[index] = fmin(1.0, fmax(0.0, alpha * conservativeScale));
}

} // namespace

//==============================================================================
cudaError_t launchPointTriangleCcdLineSearchKernel(
    const PointTriangleCcdLineSearchPair* pairs,
    double minSeparation,
    double tolerance,
    double* stepBounds,
    std::uint8_t* hits,
    std::uint8_t* indeterminate,
    std::size_t pairCount)
{
  if (pairCount == 0) {
    return cudaSuccess;
  }

  constexpr unsigned int blockSize = 256;
  const unsigned int gridSize = launchGrid1D(pairCount, blockSize);
  pointTriangleCcdLineSearchKernel<<<gridSize, blockSize>>>(
      pairs,
      minSeparation,
      tolerance,
      stepBounds,
      hits,
      indeterminate,
      pairCount);

  return cudaGetLastError();
}

} // namespace dart::simulation::compute::cuda::detail
