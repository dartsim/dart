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

__device__ double squaredNorm(const Vec3 value)
{
  return dot(value, value);
}

__device__ double norm(const Vec3 value)
{
  return sqrt(squaredNorm(value));
}

__device__ double clamp01(const double value)
{
  return fmin(1.0, fmax(0.0, value));
}

__device__ Vec3 loadVec3(const double values[3])
{
  return Vec3{values[0], values[1], values[2]};
}

__device__ Vec3 closestPointOnTriangle(
    const Vec3 point, const Vec3 a, const Vec3 b, const Vec3 c)
{
  const Vec3 ab = b - a;
  const Vec3 ac = c - a;
  const Vec3 ap = point - a;

  const double d1 = dot(ab, ap);
  const double d2 = dot(ac, ap);
  if (d1 <= 0.0 && d2 <= 0.0) {
    return a;
  }

  const Vec3 bp = point - b;
  const double d3 = dot(ab, bp);
  const double d4 = dot(ac, bp);
  if (d3 >= 0.0 && d4 <= d3) {
    return b;
  }

  const double vc = d1 * d4 - d3 * d2;
  if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
    const double v = d1 / (d1 - d3);
    return a + v * ab;
  }

  const Vec3 cp = point - c;
  const double d5 = dot(ab, cp);
  const double d6 = dot(ac, cp);
  if (d6 >= 0.0 && d5 <= d6) {
    return c;
  }

  const double vb = d5 * d2 - d1 * d6;
  if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
    const double w = d2 / (d2 - d6);
    return a + w * ac;
  }

  const double va = d3 * d6 - d5 * d4;
  if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0) {
    const double w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
    return b + w * (c - b);
  }

  const double denom = 1.0 / (va + vb + vc);
  const double v = vb * denom;
  const double w = vc * denom;
  return a + v * ab + w * ac;
}

__device__ double pointTriangleDistanceAt(
    const PointTriangleCcdLineSearchPair& pair,
    const Vec3 pointVelocity,
    const Vec3 triangleAVelocity,
    const Vec3 triangleBVelocity,
    const Vec3 triangleCVelocity,
    const double t)
{
  const Vec3 point = loadVec3(pair.pointStart) + t * pointVelocity;
  const Vec3 a = loadVec3(pair.triangleAStart) + t * triangleAVelocity;
  const Vec3 b = loadVec3(pair.triangleBStart) + t * triangleBVelocity;
  const Vec3 c = loadVec3(pair.triangleCStart) + t * triangleCVelocity;
  return norm(point - closestPointOnTriangle(point, a, b, c));
}

__device__ double edgeEdgeSquaredDistance(
    const Vec3 p1, const Vec3 q1, const Vec3 p2, const Vec3 q2)
{
  constexpr double kEpsilon = 1e-12;
  const Vec3 d1 = q1 - p1;
  const Vec3 d2 = q2 - p2;
  const Vec3 r = p1 - p2;
  const double a = dot(d1, d1);
  const double e = dot(d2, d2);
  const double f = dot(d2, r);

  double s = 0.0;
  double t = 0.0;

  if (a <= kEpsilon && e <= kEpsilon) {
    s = 0.0;
    t = 0.0;
  } else if (a <= kEpsilon) {
    s = 0.0;
    t = clamp01(f / e);
  } else {
    const double c = dot(d1, r);
    if (e <= kEpsilon) {
      t = 0.0;
      s = clamp01(-c / a);
    } else {
      const double b = dot(d1, d2);
      const double denom = a * e - b * b;
      if (denom > kEpsilon) {
        s = clamp01((b * f - c * e) / denom);
      } else {
        s = 0.0;
      }
      t = (b * s + f) / e;
      if (t < 0.0) {
        t = 0.0;
        s = clamp01(-c / a);
      } else if (t > 1.0) {
        t = 1.0;
        s = clamp01((b - c) / a);
      }
    }
  }

  const Vec3 c1 = p1 + s * d1;
  const Vec3 c2 = p2 + t * d2;
  return squaredNorm(c1 - c2);
}

__global__ void pointTriangleCcdLineSearchKernel(
    const PointTriangleCcdLineSearchPair* pairs,
    const double minSeparation,
    const double tolerance,
    const int maxIterations,
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
  Vec3 pointVelocity = loadVec3(pair.pointEnd) - loadVec3(pair.pointStart);
  Vec3 triangleAVelocity
      = loadVec3(pair.triangleAEnd) - loadVec3(pair.triangleAStart);
  Vec3 triangleBVelocity
      = loadVec3(pair.triangleBEnd) - loadVec3(pair.triangleBStart);
  Vec3 triangleCVelocity
      = loadVec3(pair.triangleCEnd) - loadVec3(pair.triangleCStart);

  const Vec3 mean = 0.25
                    * (pointVelocity + triangleAVelocity + triangleBVelocity
                       + triangleCVelocity);
  pointVelocity = pointVelocity - mean;
  triangleAVelocity = triangleAVelocity - mean;
  triangleBVelocity = triangleBVelocity - mean;
  triangleCVelocity = triangleCVelocity - mean;

  const double relativeSpeedBound
      = norm(pointVelocity)
        + fmax(
            fmax(norm(triangleAVelocity), norm(triangleBVelocity)),
            norm(triangleCVelocity));
  const double threshold = fmax(0.0, minSeparation);
  const double effectiveTolerance = fmax(0.0, tolerance);
  constexpr double kEpsilon = 1e-12;

  double distance = pointTriangleDistanceAt(
      pair,
      pointVelocity,
      triangleAVelocity,
      triangleBVelocity,
      triangleCVelocity,
      0.0);
  const double initialClearance = distance - threshold;
  if (initialClearance <= 0.0) {
    hits[index] = 1u;
    stepBounds[index] = 0.0;
    return;
  }

  if (relativeSpeedBound <= kEpsilon) {
    return;
  }

  constexpr double kStepFactor = 0.9;
  const double convergeAbs = fmax(effectiveTolerance, kEpsilon);
  const int iterationCount = maxIterations < 1 ? 1 : maxIterations;

  double t = 0.0;
  for (int iter = 0; iter < iterationCount; ++iter) {
    const double clearance = distance - threshold;
    if (clearance <= convergeAbs) {
      hits[index] = 1u;
      stepBounds[index] = clamp01(t);
      return;
    }

    if (t + clearance / relativeSpeedBound >= 1.0) {
      const double finalDistance = pointTriangleDistanceAt(
          pair,
          pointVelocity,
          triangleAVelocity,
          triangleBVelocity,
          triangleCVelocity,
          1.0);
      if (finalDistance - threshold <= convergeAbs) {
        hits[index] = 1u;
        stepBounds[index] = 1.0;
      }
      return;
    }

    t += kStepFactor * clearance / relativeSpeedBound;
    distance = pointTriangleDistanceAt(
        pair,
        pointVelocity,
        triangleAVelocity,
        triangleBVelocity,
        triangleCVelocity,
        t);
  }

  indeterminate[index] = 1u;
  stepBounds[index] = 0.0;
}

__device__ double edgeEdgeDistanceAt(
    const EdgeEdgeCcdLineSearchPair& pair,
    const Vec3 va0,
    const Vec3 va1,
    const Vec3 vb0,
    const Vec3 vb1,
    const double t)
{
  const Vec3 a0 = loadVec3(pair.edgeA0Start) + t * va0;
  const Vec3 a1 = loadVec3(pair.edgeA1Start) + t * va1;
  const Vec3 b0 = loadVec3(pair.edgeB0Start) + t * vb0;
  const Vec3 b1 = loadVec3(pair.edgeB1Start) + t * vb1;
  return sqrt(edgeEdgeSquaredDistance(a0, a1, b0, b1));
}

__global__ void edgeEdgeCcdLineSearchKernel(
    const EdgeEdgeCcdLineSearchPair* pairs,
    const double minSeparation,
    const double tolerance,
    const int maxIterations,
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

  const EdgeEdgeCcdLineSearchPair pair = pairs[index];
  Vec3 va0 = loadVec3(pair.edgeA0End) - loadVec3(pair.edgeA0Start);
  Vec3 va1 = loadVec3(pair.edgeA1End) - loadVec3(pair.edgeA1Start);
  Vec3 vb0 = loadVec3(pair.edgeB0End) - loadVec3(pair.edgeB0Start);
  Vec3 vb1 = loadVec3(pair.edgeB1End) - loadVec3(pair.edgeB1Start);

  const Vec3 mean = 0.25 * (va0 + va1 + vb0 + vb1);
  va0 = va0 - mean;
  va1 = va1 - mean;
  vb0 = vb0 - mean;
  vb1 = vb1 - mean;

  const double relativeSpeedBound
      = fmax(norm(va0), norm(va1)) + fmax(norm(vb0), norm(vb1));
  const double threshold = fmax(0.0, minSeparation);
  const double effectiveTolerance = fmax(0.0, tolerance);
  constexpr double kEpsilon = 1e-12;

  double distance = edgeEdgeDistanceAt(pair, va0, va1, vb0, vb1, 0.0);
  const double initialClearance = distance - threshold;
  if (initialClearance <= 0.0) {
    hits[index] = 1u;
    stepBounds[index] = 0.0;
    return;
  }

  if (relativeSpeedBound <= kEpsilon) {
    return;
  }

  constexpr double kStepFactor = 0.9;
  const double convergeAbs = fmax(effectiveTolerance, kEpsilon);
  const int iterationCount = maxIterations < 1 ? 1 : maxIterations;

  double t = 0.0;
  for (int iter = 0; iter < iterationCount; ++iter) {
    const double clearance = distance - threshold;
    if (clearance <= convergeAbs) {
      hits[index] = 1u;
      stepBounds[index] = clamp01(t);
      return;
    }

    if (t + clearance / relativeSpeedBound >= 1.0) {
      const double finalDistance
          = edgeEdgeDistanceAt(pair, va0, va1, vb0, vb1, 1.0);
      if (finalDistance - threshold <= convergeAbs) {
        hits[index] = 1u;
        stepBounds[index] = 1.0;
      }
      return;
    }

    t += kStepFactor * clearance / relativeSpeedBound;
    distance = edgeEdgeDistanceAt(pair, va0, va1, vb0, vb1, t);
  }

  indeterminate[index] = 1u;
  stepBounds[index] = 0.0;
}

} // namespace

//==============================================================================
cudaError_t launchPointTriangleCcdLineSearchKernel(
    const PointTriangleCcdLineSearchPair* pairs,
    double minSeparation,
    double tolerance,
    int maxIterations,
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
      maxIterations,
      stepBounds,
      hits,
      indeterminate,
      pairCount);

  return cudaGetLastError();
}

//==============================================================================
cudaError_t launchEdgeEdgeCcdLineSearchKernel(
    const EdgeEdgeCcdLineSearchPair* pairs,
    double minSeparation,
    double tolerance,
    int maxIterations,
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
  edgeEdgeCcdLineSearchKernel<<<gridSize, blockSize>>>(
      pairs,
      minSeparation,
      tolerance,
      maxIterations,
      stepBounds,
      hits,
      indeterminate,
      pairCount);

  return cudaGetLastError();
}

} // namespace dart::simulation::compute::cuda::detail
