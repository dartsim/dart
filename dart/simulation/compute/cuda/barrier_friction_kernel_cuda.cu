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
 *     copyright notice and this list of conditions in the documentation
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
#include <dart/simulation/compute/cuda/barrier_friction_kernel_cuda.cuh>
#include <dart/simulation/compute/cuda/cuda_runtime.cuh>

#include <cstddef>
#include <cstdint>

namespace dart::simulation::compute::cuda::detail {
namespace {

struct BarrierScalar
{
  double value = 0.0;
  double firstDerivative = 0.0;
  double secondDerivative = 0.0;
  std::uint8_t active = 0u;
};

struct SmoothFriction
{
  double value = 0.0;
  double firstDerivative = 0.0;
  double secondDerivative = 0.0;
  std::uint8_t active = 0u;
  std::uint8_t dynamic = 0u;
};

struct Vec3
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct PointEdgeDistanceDevice
{
  double squaredDistance = 0.0;
  double edgeCoordinate = 0.0;
  Vec3 closestPoint;
};

struct PointTriangleDistanceDevice
{
  double squaredDistance = 0.0;
  double barycentric[3] = {1.0, 0.0, 0.0};
  Vec3 closestPoint;
};

__device__ Vec3 makeVec3(const double values[3])
{
  return {values[0], values[1], values[2]};
}

__device__ Vec3 add(const Vec3 lhs, const Vec3 rhs)
{
  return {lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z};
}

__device__ Vec3 subtract(const Vec3 lhs, const Vec3 rhs)
{
  return {lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z};
}

__device__ Vec3 scale(const double factor, const Vec3 value)
{
  return {factor * value.x, factor * value.y, factor * value.z};
}

__device__ double dot(const Vec3 lhs, const Vec3 rhs)
{
  return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

__device__ Vec3 cross(const Vec3 lhs, const Vec3 rhs)
{
  return {
      lhs.y * rhs.z - lhs.z * rhs.y,
      lhs.z * rhs.x - lhs.x * rhs.z,
      lhs.x * rhs.y - lhs.y * rhs.x};
}

__device__ double squaredNorm(const Vec3 value)
{
  return dot(value, value);
}

__device__ bool isDegenerateSegmentDevice(const double squaredLength)
{
  constexpr double kDoubleMin = 2.2250738585072014e-308;
  return squaredLength <= kDoubleMin;
}

__device__ bool isDegenerateTriangleDevice(
    const double normalSquaredNorm,
    const double edgeASquaredNorm,
    const double edgeBSquaredNorm)
{
  constexpr double kDoubleMin = 2.2250738585072014e-308;
  constexpr double kRelativeEpsilon = 64.0 * 2.2204460492503131e-16;
  const double scale = fmax(edgeASquaredNorm * edgeBSquaredNorm, kDoubleMin);
  return normalSquaredNorm <= kRelativeEpsilon * scale;
}

__device__ PointEdgeDistanceDevice
pointEdgeSquaredDistanceDevice(const Vec3 p, const Vec3 a, const Vec3 b)
{
  PointEdgeDistanceDevice result;
  const Vec3 edge = subtract(b, a);
  const double edgeSquaredNorm = squaredNorm(edge);
  if (isDegenerateSegmentDevice(edgeSquaredNorm)) {
    result.edgeCoordinate = 0.0;
    result.closestPoint = a;
    result.squaredDistance = squaredNorm(subtract(p, a));
    return result;
  }

  const double unclampedT = dot(edge, subtract(p, a)) / edgeSquaredNorm;
  result.edgeCoordinate = fmin(fmax(unclampedT, 0.0), 1.0);
  result.closestPoint = add(a, scale(result.edgeCoordinate, edge));
  result.squaredDistance = squaredNorm(subtract(p, result.closestPoint));
  return result;
}

__device__ PointTriangleDistanceDevice pointTriangleSquaredDistanceDevice(
    const Vec3 p, const Vec3 a, const Vec3 b, const Vec3 c)
{
  PointTriangleDistanceDevice result;
  const Vec3 ab = subtract(b, a);
  const Vec3 ac = subtract(c, a);
  const Vec3 normal = cross(ab, ac);
  if (isDegenerateTriangleDevice(
          squaredNorm(normal), squaredNorm(ab), squaredNorm(ac))) {
    const auto abDistance = pointEdgeSquaredDistanceDevice(p, a, b);
    const auto bcDistance = pointEdgeSquaredDistanceDevice(p, b, c);
    const auto caDistance = pointEdgeSquaredDistanceDevice(p, c, a);

    result.squaredDistance = abDistance.squaredDistance;
    result.closestPoint = abDistance.closestPoint;
    result.barycentric[0] = 1.0 - abDistance.edgeCoordinate;
    result.barycentric[1] = abDistance.edgeCoordinate;
    result.barycentric[2] = 0.0;

    if (bcDistance.squaredDistance < result.squaredDistance) {
      result.squaredDistance = bcDistance.squaredDistance;
      result.closestPoint = bcDistance.closestPoint;
      result.barycentric[0] = 0.0;
      result.barycentric[1] = 1.0 - bcDistance.edgeCoordinate;
      result.barycentric[2] = bcDistance.edgeCoordinate;
    }
    if (caDistance.squaredDistance < result.squaredDistance) {
      result.squaredDistance = caDistance.squaredDistance;
      result.closestPoint = caDistance.closestPoint;
      result.barycentric[0] = caDistance.edgeCoordinate;
      result.barycentric[1] = 0.0;
      result.barycentric[2] = 1.0 - caDistance.edgeCoordinate;
    }
    return result;
  }

  const Vec3 ap = subtract(p, a);
  const double d1 = dot(ab, ap);
  const double d2 = dot(ac, ap);
  if (d1 <= 0.0 && d2 <= 0.0) {
    result.barycentric[0] = 1.0;
    result.barycentric[1] = 0.0;
    result.barycentric[2] = 0.0;
    result.closestPoint = a;
  } else {
    const Vec3 bp = subtract(p, b);
    const double d3 = dot(ab, bp);
    const double d4 = dot(ac, bp);
    if (d3 >= 0.0 && d4 <= d3) {
      result.barycentric[0] = 0.0;
      result.barycentric[1] = 1.0;
      result.barycentric[2] = 0.0;
      result.closestPoint = b;
    } else {
      const double vc = d1 * d4 - d3 * d2;
      if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
        const double v = d1 / (d1 - d3);
        result.barycentric[0] = 1.0 - v;
        result.barycentric[1] = v;
        result.barycentric[2] = 0.0;
        result.closestPoint = add(a, scale(v, ab));
      } else {
        const Vec3 cp = subtract(p, c);
        const double d5 = dot(ab, cp);
        const double d6 = dot(ac, cp);
        if (d6 >= 0.0 && d5 <= d6) {
          result.barycentric[0] = 0.0;
          result.barycentric[1] = 0.0;
          result.barycentric[2] = 1.0;
          result.closestPoint = c;
        } else {
          const double vb = d5 * d2 - d1 * d6;
          if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
            const double w = d2 / (d2 - d6);
            result.barycentric[0] = 1.0 - w;
            result.barycentric[1] = 0.0;
            result.barycentric[2] = w;
            result.closestPoint = add(a, scale(w, ac));
          } else {
            const double va = d3 * d6 - d5 * d4;
            if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0) {
              const double w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
              result.barycentric[0] = 0.0;
              result.barycentric[1] = 1.0 - w;
              result.barycentric[2] = w;
              result.closestPoint = add(b, scale(w, subtract(c, b)));
            } else {
              const double denominator = 1.0 / (va + vb + vc);
              const double v = vb * denominator;
              const double w = vc * denominator;
              result.barycentric[0] = 1.0 - v - w;
              result.barycentric[1] = v;
              result.barycentric[2] = w;
              result.closestPoint = add(a, add(scale(v, ab), scale(w, ac)));
            }
          }
        }
      }
    }
  }

  result.squaredDistance = squaredNorm(subtract(p, result.closestPoint));
  return result;
}

__device__ void writePointTriangleDistanceGradient(
    const PointTriangleDistanceDevice& distance, const Vec3 p, double* gradient)
{
  const Vec3 residual = subtract(p, distance.closestPoint);
  const Vec3 blocks[4] = {
      scale(2.0, residual),
      scale(-2.0 * distance.barycentric[0], residual),
      scale(-2.0 * distance.barycentric[1], residual),
      scale(-2.0 * distance.barycentric[2], residual),
  };
  for (int block = 0; block < 4; ++block) {
    gradient[3 * block] = blocks[block].x;
    gradient[3 * block + 1] = blocks[block].y;
    gradient[3 * block + 2] = blocks[block].z;
  }
}

__device__ BarrierScalar c2ClampedLogBarrierDevice(
    const double squaredDistance, const double squaredActivationDistance)
{
  BarrierScalar result;
  if (!isfinite(squaredActivationDistance) || squaredActivationDistance <= 0.0
      || !isfinite(squaredDistance)) {
    return result;
  }

  if (squaredDistance >= squaredActivationDistance) {
    return result;
  }

  const double activeInteriorLimit = nextafter(squaredActivationDistance, 0.0);
  if (!(activeInteriorLimit > 0.0)) {
    return result;
  }

  constexpr double kDistanceFloorScale = 1e-16;
  constexpr double kDenormMin = 4.9406564584124654e-324;
  const double floor = fmin(
      fmax(kDenormMin, kDistanceFloorScale * squaredActivationDistance),
      activeInteriorLimit);
  const double d = fmax(squaredDistance, floor);
  const double dHat = squaredActivationDistance;
  const double offset = d - dHat;
  const double logRatio = log(d / dHat);
  const double offsetSquared = offset * offset;

  result.value = -offsetSquared * logRatio;
  result.firstDerivative = -2.0 * offset * logRatio - offsetSquared / d;
  result.secondDerivative
      = -2.0 * logRatio - 4.0 * offset / d + offsetSquared / (d * d);
  result.active = 1u;
  return result;
}

__device__ SmoothFriction
smoothFrictionNormDevice(const double norm, const double staticDisplacement)
{
  SmoothFriction result;
  if (!(norm >= 0.0) || !isfinite(norm) || !(staticDisplacement > 0.0)
      || !isfinite(staticDisplacement)) {
    return result;
  }

  result.active = 1u;
  if (norm > staticDisplacement) {
    result.value = norm;
    result.firstDerivative = 1.0;
    result.secondDerivative = 0.0;
    result.dynamic = 1u;
    return result;
  }

  const double invEps = 1.0 / staticDisplacement;
  const double invEpsSquared = invEps * invEps;
  result.value = norm * norm * (1.0 - norm * invEps / 3.0) * invEps
                 + staticDisplacement / 3.0;
  result.firstDerivative = 2.0 * norm * invEps - norm * norm * invEpsSquared;
  result.secondDerivative = 2.0 * invEps - 2.0 * norm * invEpsSquared;
  return result;
}

__global__ void barrierFrictionLocalKernel(
    const BarrierFrictionLocalInput* inputs,
    double* barrierValues,
    double* barrierFirstDerivatives,
    double* barrierSecondDerivatives,
    double* frictionValues,
    double* frictionWorks,
    double* frictionFirstDerivatives,
    double* frictionSecondDerivatives,
    std::uint8_t* activeBarriers,
    std::uint8_t* activeFrictions,
    std::uint8_t* dynamicFrictions,
    const std::size_t inputCount)
{
  const auto index
      = static_cast<std::size_t>(blockIdx.x * blockDim.x + threadIdx.x);
  if (index >= inputCount) {
    return;
  }

  barrierValues[index] = 0.0;
  barrierFirstDerivatives[index] = 0.0;
  barrierSecondDerivatives[index] = 0.0;
  frictionValues[index] = 0.0;
  frictionWorks[index] = 0.0;
  frictionFirstDerivatives[index] = 0.0;
  frictionSecondDerivatives[index] = 0.0;
  activeBarriers[index] = 0u;
  activeFrictions[index] = 0u;
  dynamicFrictions[index] = 0u;

  const BarrierFrictionLocalInput input = inputs[index];
  const double stiffness
      = isfinite(input.stiffness) ? fmax(0.0, input.stiffness) : 0.0;
  const BarrierScalar barrier = c2ClampedLogBarrierDevice(
      input.squaredDistance, input.squaredActivationDistance);
  if (barrier.active != 0u && stiffness > 0.0) {
    activeBarriers[index] = 1u;
    barrierValues[index] = stiffness * barrier.value;
    barrierFirstDerivatives[index] = stiffness * barrier.firstDerivative;
    barrierSecondDerivatives[index] = stiffness * barrier.secondDerivative;
  }

  const double weight
      = isfinite(input.frictionWeight) ? fmax(0.0, input.frictionWeight) : 0.0;
  const SmoothFriction friction = smoothFrictionNormDevice(
      input.tangentialDisplacementNorm, input.staticFrictionDisplacement);
  if (friction.active != 0u && weight > 0.0) {
    activeFrictions[index] = 1u;
    dynamicFrictions[index] = friction.dynamic;
    frictionValues[index] = weight * friction.value;
    frictionWorks[index]
        = weight * friction.firstDerivative * input.tangentialDisplacementNorm;
    frictionFirstDerivatives[index] = friction.firstDerivative;
    frictionSecondDerivatives[index] = friction.secondDerivative;
  }
}

__global__ void pointTriangleBarrierGradientKernel(
    const PointTriangleBarrierInput* inputs,
    double* squaredDistances,
    double* barrierValues,
    double* barrierGradients,
    std::uint8_t* activeBarriers,
    const std::size_t inputCount)
{
  const auto index
      = static_cast<std::size_t>(blockIdx.x * blockDim.x + threadIdx.x);
  if (index >= inputCount) {
    return;
  }

  squaredDistances[index] = 0.0;
  barrierValues[index] = 0.0;
  activeBarriers[index] = 0u;
  double distanceGradient[12];
  for (double& entry : distanceGradient) {
    entry = 0.0;
  }

  const PointTriangleBarrierInput input = inputs[index];
  const Vec3 p = makeVec3(input.point);
  const Vec3 a = makeVec3(input.triangleA);
  const Vec3 b = makeVec3(input.triangleB);
  const Vec3 c = makeVec3(input.triangleC);
  const auto distance = pointTriangleSquaredDistanceDevice(p, a, b, c);
  squaredDistances[index] = distance.squaredDistance;
  writePointTriangleDistanceGradient(distance, p, distanceGradient);

  const double stiffness
      = isfinite(input.stiffness) ? fmax(0.0, input.stiffness) : 0.0;
  const BarrierScalar barrier = c2ClampedLogBarrierDevice(
      distance.squaredDistance, input.squaredActivationDistance);
  const double gradientScale = stiffness * barrier.firstDerivative;
  if (barrier.active != 0u && stiffness > 0.0) {
    activeBarriers[index] = 1u;
    barrierValues[index] = stiffness * barrier.value;
  }

  const std::size_t gradientOffset = 12 * index;
  for (int entry = 0; entry < 12; ++entry) {
    barrierGradients[gradientOffset + static_cast<std::size_t>(entry)]
        = activeBarriers[index] != 0u ? gradientScale * distanceGradient[entry]
                                      : 0.0;
  }
}

} // namespace

//==============================================================================
cudaError_t launchBarrierFrictionLocalKernel(
    const BarrierFrictionLocalInput* inputs,
    double* barrierValues,
    double* barrierFirstDerivatives,
    double* barrierSecondDerivatives,
    double* frictionValues,
    double* frictionWorks,
    double* frictionFirstDerivatives,
    double* frictionSecondDerivatives,
    std::uint8_t* activeBarriers,
    std::uint8_t* activeFrictions,
    std::uint8_t* dynamicFrictions,
    std::size_t inputCount)
{
  if (inputCount == 0) {
    return cudaSuccess;
  }

  constexpr unsigned int blockSize = 256;
  const unsigned int gridSize = launchGrid1D(inputCount, blockSize);
  barrierFrictionLocalKernel<<<gridSize, blockSize>>>(
      inputs,
      barrierValues,
      barrierFirstDerivatives,
      barrierSecondDerivatives,
      frictionValues,
      frictionWorks,
      frictionFirstDerivatives,
      frictionSecondDerivatives,
      activeBarriers,
      activeFrictions,
      dynamicFrictions,
      inputCount);

  return cudaGetLastError();
}

//==============================================================================
cudaError_t launchPointTriangleBarrierGradientKernel(
    const PointTriangleBarrierInput* inputs,
    double* squaredDistances,
    double* barrierValues,
    double* barrierGradients,
    std::uint8_t* activeBarriers,
    std::size_t inputCount)
{
  if (inputCount == 0) {
    return cudaSuccess;
  }

  constexpr unsigned int blockSize = 256;
  const unsigned int gridSize = launchGrid1D(inputCount, blockSize);
  pointTriangleBarrierGradientKernel<<<gridSize, blockSize>>>(
      inputs,
      squaredDistances,
      barrierValues,
      barrierGradients,
      activeBarriers,
      inputCount);

  return cudaGetLastError();
}

} // namespace dart::simulation::compute::cuda::detail
