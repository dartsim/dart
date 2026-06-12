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
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include <cuda_runtime.h>
#include <dart/simulation/compute/cuda/contact_candidate_filter_cuda.cuh>
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

__device__ double clamp01(const double value)
{
  return fmin(1.0, fmax(0.0, value));
}

__device__ bool isDegenerateSegment(const double squaredLength)
{
  return squaredLength <= 2.2250738585072014e-308;
}

__device__ bool isParallelEdges(
    const double denominator,
    const double edgeASquaredNorm,
    const double edgeBSquaredNorm)
{
  constexpr double kRelativeEpsilon = 64.0 * 2.2204460492503131e-16;
  const double scale
      = fmax(edgeASquaredNorm * edgeBSquaredNorm, 2.2250738585072014e-308);
  return denominator <= kRelativeEpsilon * scale;
}

__device__ Vec3 loadPoint(const double* positions, const std::uint32_t point)
{
  const std::size_t base = 3u * static_cast<std::size_t>(point);
  return Vec3{positions[base], positions[base + 1u], positions[base + 2u]};
}

__device__ double pointSegmentSquaredDistance(
    const Vec3 p, const Vec3 a, const Vec3 b)
{
  const Vec3 ab = b - a;
  const double denom = squaredNorm(ab);
  if (denom <= 0.0) {
    return squaredNorm(p - a);
  }
  const double unclamped = dot(p - a, ab) / denom;
  const double t = fmin(1.0, fmax(0.0, unclamped));
  return squaredNorm(p - (a + t * ab));
}

__device__ double pointTriangleSquaredDistance(
    const Vec3 p, const Vec3 a, const Vec3 b, const Vec3 c)
{
  constexpr double kRelativeEpsilon = 64.0 * 2.2204460492503131e-16;
  const Vec3 ab = b - a;
  const Vec3 ac = c - a;
  const double normalSquared = squaredNorm(cross(ab, ac));
  const double scale
      = fmax(squaredNorm(ab) * squaredNorm(ac), 2.2250738585072014e-308);
  if (normalSquared <= kRelativeEpsilon * scale) {
    return fmin(
        pointSegmentSquaredDistance(p, a, b),
        fmin(
            pointSegmentSquaredDistance(p, b, c),
            pointSegmentSquaredDistance(p, c, a)));
  }

  const Vec3 ap = p - a;
  const double d1 = dot(ab, ap);
  const double d2 = dot(ac, ap);
  if (d1 <= 0.0 && d2 <= 0.0) {
    return squaredNorm(p - a);
  }

  const Vec3 bp = p - b;
  const double d3 = dot(ab, bp);
  const double d4 = dot(ac, bp);
  if (d3 >= 0.0 && d4 <= d3) {
    return squaredNorm(p - b);
  }

  const double vc = d1 * d4 - d3 * d2;
  if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
    const double v = d1 / (d1 - d3);
    return squaredNorm(p - (a + v * ab));
  }

  const Vec3 cp = p - c;
  const double d5 = dot(ab, cp);
  const double d6 = dot(ac, cp);
  if (d6 >= 0.0 && d5 <= d6) {
    return squaredNorm(p - c);
  }

  const double vb = d5 * d2 - d1 * d6;
  if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
    const double w = d2 / (d2 - d6);
    return squaredNorm(p - (a + w * ac));
  }

  const double va = d3 * d6 - d5 * d4;
  if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0) {
    const double w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
    return squaredNorm(p - (b + w * (c - b)));
  }

  const double denom = 1.0 / (va + vb + vc);
  const double v = vb * denom;
  const double w = vc * denom;
  return squaredNorm(p - (a + v * ab + w * ac));
}

__device__ double edgeEdgeSquaredDistance(
    const Vec3 a, const Vec3 b, const Vec3 c, const Vec3 d)
{
  const Vec3 edgeA = b - a;
  const Vec3 edgeB = d - c;
  const Vec3 r = a - c;
  const double edgeASquaredNorm = squaredNorm(edgeA);
  const double edgeBSquaredNorm = squaredNorm(edgeB);
  const double edgeBDotR = dot(edgeB, r);

  double s = 0.0;
  double t = 0.0;

  if (isDegenerateSegment(edgeASquaredNorm)
      && isDegenerateSegment(edgeBSquaredNorm)) {
    s = 0.0;
    t = 0.0;
  } else if (isDegenerateSegment(edgeASquaredNorm)) {
    s = 0.0;
    t = clamp01(edgeBDotR / edgeBSquaredNorm);
  } else {
    const double edgeADotR = dot(edgeA, r);
    if (isDegenerateSegment(edgeBSquaredNorm)) {
      t = 0.0;
      s = clamp01(-edgeADotR / edgeASquaredNorm);
    } else {
      const double edgeADotB = dot(edgeA, edgeB);
      const double denominator
          = edgeASquaredNorm * edgeBSquaredNorm - edgeADotB * edgeADotB;
      if (!isParallelEdges(denominator, edgeASquaredNorm, edgeBSquaredNorm)) {
        s = clamp01(
            (edgeADotB * edgeBDotR - edgeADotR * edgeBSquaredNorm)
            / denominator);
      } else {
        s = 0.0;
      }

      t = (edgeADotB * s + edgeBDotR) / edgeBSquaredNorm;
      if (t < 0.0) {
        t = 0.0;
        s = clamp01(-edgeADotR / edgeASquaredNorm);
      } else if (t > 1.0) {
        t = 1.0;
        s = clamp01((edgeADotB - edgeADotR) / edgeASquaredNorm);
      }
    }
  }

  const Vec3 closestA = a + s * edgeA;
  const Vec3 closestB = c + t * edgeB;
  return squaredNorm(closestA - closestB);
}

__global__ void filterPointTriangleContactStencilKernel(
    const double* positions,
    const std::uint32_t* triangleIndices,
    const PointTriangleContactStencil* stencils,
    const double activationDistance,
    double* squaredDistances,
    std::uint8_t* accepted,
    const std::size_t stencilCount)
{
  const auto index
      = static_cast<std::size_t>(blockIdx.x * blockDim.x + threadIdx.x);
  if (index >= stencilCount) {
    return;
  }

  const PointTriangleContactStencil stencil = stencils[index];
  const std::size_t triangleBase
      = 3u * static_cast<std::size_t>(stencil.triangle);
  const Vec3 p = loadPoint(positions, stencil.point);
  const Vec3 a = loadPoint(positions, triangleIndices[triangleBase]);
  const Vec3 b = loadPoint(positions, triangleIndices[triangleBase + 1u]);
  const Vec3 c = loadPoint(positions, triangleIndices[triangleBase + 2u]);

  const double squaredDistance = pointTriangleSquaredDistance(p, a, b, c);
  const double threshold = activationDistance * activationDistance;
  constexpr double kRelativeEpsilon = 64.0 * 2.2204460492503131e-16;
  const double tolerance = kRelativeEpsilon * fmax(1.0, threshold);
  squaredDistances[index] = squaredDistance;
  accepted[index] = squaredDistance <= threshold + tolerance ? 1u : 0u;
}

__global__ void buildPointTriangleContactCandidateMaskKernel(
    const double* positions,
    const std::uint32_t* pointIndices,
    const std::uint32_t* triangleIndices,
    const double activationDistance,
    double* squaredDistances,
    std::uint8_t* accepted,
    const std::size_t pointCount,
    const std::size_t triangleCount)
{
  const auto index
      = static_cast<std::size_t>(blockIdx.x * blockDim.x + threadIdx.x);
  const std::size_t pairCount = pointCount * triangleCount;
  if (index >= pairCount) {
    return;
  }

  const std::size_t pointSlot = index / triangleCount;
  const std::size_t triangle = index - pointSlot * triangleCount;
  const std::uint32_t point = pointIndices[pointSlot];
  const std::size_t triangleBase = 3u * triangle;
  const std::uint32_t aIndex = triangleIndices[triangleBase];
  const std::uint32_t bIndex = triangleIndices[triangleBase + 1u];
  const std::uint32_t cIndex = triangleIndices[triangleBase + 2u];
  if (point == aIndex || point == bIndex || point == cIndex) {
    squaredDistances[index] = 0.0;
    accepted[index] = 0u;
    return;
  }

  const Vec3 p = loadPoint(positions, point);
  const Vec3 a = loadPoint(positions, aIndex);
  const Vec3 b = loadPoint(positions, bIndex);
  const Vec3 c = loadPoint(positions, cIndex);

  const double squaredDistance = pointTriangleSquaredDistance(p, a, b, c);
  const double threshold = activationDistance * activationDistance;
  constexpr double kRelativeEpsilon = 64.0 * 2.2204460492503131e-16;
  const double tolerance = kRelativeEpsilon * fmax(1.0, threshold);
  squaredDistances[index] = squaredDistance;
  accepted[index] = squaredDistance <= threshold + tolerance ? 1u : 0u;
}

__global__ void countPointTriangleAcceptedBlocksKernel(
    const std::uint8_t* accepted,
    std::uint32_t* acceptedBlockCounts,
    const std::size_t pairCount)
{
  extern __shared__ std::uint32_t counts[];

  const auto local = static_cast<unsigned int>(threadIdx.x);
  const auto index = static_cast<std::size_t>(blockIdx.x * blockDim.x + local);
  counts[local] = index < pairCount && accepted[index] != 0u ? 1u : 0u;
  __syncthreads();

  for (unsigned int stride = blockDim.x / 2u; stride > 0u; stride >>= 1u) {
    if (local < stride) {
      counts[local] += counts[local + stride];
    }
    __syncthreads();
  }

  if (local == 0u) {
    acceptedBlockCounts[blockIdx.x] = counts[0];
  }
}

__global__ void prefixPointTriangleAcceptedBlockCountsKernel(
    const std::uint32_t* acceptedBlockCounts,
    std::uint32_t* acceptedBlockOffsets,
    std::uint32_t* compactedCount,
    const std::size_t blockCount)
{
  if (blockIdx.x != 0u || threadIdx.x != 0u) {
    return;
  }

  std::uint32_t count = 0u;
  for (std::size_t block = 0; block < blockCount; ++block) {
    acceptedBlockOffsets[block] = count;
    count += acceptedBlockCounts[block];
  }
  *compactedCount = count;
}

__global__ void compactPointTriangleContactCandidateMaskKernel(
    const std::uint32_t* pointIndices,
    const std::uint8_t* accepted,
    const std::uint32_t* acceptedBlockOffsets,
    std::uint32_t* acceptedPointIndices,
    std::uint32_t* acceptedTriangleIndices,
    const std::size_t pointCount,
    const std::size_t triangleCount)
{
  extern __shared__ std::uint32_t localRanks[];

  const auto local = static_cast<unsigned int>(threadIdx.x);
  const auto index = static_cast<std::size_t>(blockIdx.x * blockDim.x + local);
  const std::size_t pairCount = pointCount * triangleCount;
  const std::uint32_t acceptedFlag
      = index < pairCount && accepted[index] != 0u ? 1u : 0u;
  localRanks[local] = acceptedFlag;
  __syncthreads();

  for (unsigned int offset = 1u; offset < blockDim.x; offset <<= 1u) {
    std::uint32_t previous = 0u;
    if (local >= offset) {
      previous = localRanks[local - offset];
    }
    __syncthreads();
    localRanks[local] += previous;
    __syncthreads();
  }

  if (acceptedFlag == 0u) {
    return;
  }

  const std::size_t pointSlot = index / triangleCount;
  const std::size_t triangle = index - pointSlot * triangleCount;
  const std::uint32_t compactedIndex
      = acceptedBlockOffsets[blockIdx.x] + localRanks[local] - 1u;
  acceptedPointIndices[compactedIndex] = pointIndices[pointSlot];
  acceptedTriangleIndices[compactedIndex]
      = static_cast<std::uint32_t>(triangle);
}

__device__ bool edgesShareVertex(
    const std::uint32_t a0,
    const std::uint32_t a1,
    const std::uint32_t b0,
    const std::uint32_t b1)
{
  return a0 == b0 || a0 == b1 || a1 == b0 || a1 == b1;
}

__global__ void filterEdgeEdgeContactStencilKernel(
    const double* positions,
    const EdgeEdgeContactStencil* stencils,
    const double activationDistance,
    double* squaredDistances,
    std::uint8_t* accepted,
    const std::size_t stencilCount)
{
  const auto index
      = static_cast<std::size_t>(blockIdx.x * blockDim.x + threadIdx.x);
  if (index >= stencilCount) {
    return;
  }

  const EdgeEdgeContactStencil stencil = stencils[index];
  const Vec3 a = loadPoint(positions, stencil.edgeAStart);
  const Vec3 b = loadPoint(positions, stencil.edgeAEnd);
  const Vec3 c = loadPoint(positions, stencil.edgeBStart);
  const Vec3 d = loadPoint(positions, stencil.edgeBEnd);

  const double squaredDistance = edgeEdgeSquaredDistance(a, b, c, d);
  const double threshold = activationDistance * activationDistance;
  constexpr double kRelativeEpsilon = 64.0 * 2.2204460492503131e-16;
  const double tolerance = kRelativeEpsilon * fmax(1.0, threshold);
  squaredDistances[index] = squaredDistance;
  accepted[index] = squaredDistance <= threshold + tolerance ? 1u : 0u;
}

__global__ void buildEdgeEdgeContactCandidateMaskKernel(
    const double* positions,
    const std::uint32_t* edgeIndices,
    const double activationDistance,
    double* squaredDistances,
    std::uint8_t* accepted,
    const std::size_t edgeCount)
{
  const auto index
      = static_cast<std::size_t>(blockIdx.x * blockDim.x + threadIdx.x);
  const std::size_t pairCount = edgeCount * edgeCount;
  if (index >= pairCount) {
    return;
  }

  const std::size_t edgeA = index / edgeCount;
  const std::size_t edgeB = index - edgeA * edgeCount;
  if (edgeA >= edgeB) {
    squaredDistances[index] = 0.0;
    accepted[index] = 0u;
    return;
  }

  const std::uint32_t a0 = edgeIndices[2u * edgeA];
  const std::uint32_t a1 = edgeIndices[2u * edgeA + 1u];
  const std::uint32_t b0 = edgeIndices[2u * edgeB];
  const std::uint32_t b1 = edgeIndices[2u * edgeB + 1u];
  if (edgesShareVertex(a0, a1, b0, b1)) {
    squaredDistances[index] = 0.0;
    accepted[index] = 0u;
    return;
  }

  const Vec3 a = loadPoint(positions, a0);
  const Vec3 b = loadPoint(positions, a1);
  const Vec3 c = loadPoint(positions, b0);
  const Vec3 d = loadPoint(positions, b1);

  const double squaredDistance = edgeEdgeSquaredDistance(a, b, c, d);
  const double threshold = activationDistance * activationDistance;
  constexpr double kRelativeEpsilon = 64.0 * 2.2204460492503131e-16;
  const double tolerance = kRelativeEpsilon * fmax(1.0, threshold);
  squaredDistances[index] = squaredDistance;
  accepted[index] = squaredDistance <= threshold + tolerance ? 1u : 0u;
}

__global__ void compactEdgeEdgeContactCandidateMaskKernel(
    const std::uint8_t* accepted,
    const std::uint32_t* acceptedBlockOffsets,
    std::uint32_t* acceptedEdgeAIndices,
    std::uint32_t* acceptedEdgeBIndices,
    const std::size_t edgeCount)
{
  extern __shared__ std::uint32_t localRanks[];

  const auto local = static_cast<unsigned int>(threadIdx.x);
  const auto index = static_cast<std::size_t>(blockIdx.x * blockDim.x + local);
  const std::size_t pairCount = edgeCount * edgeCount;
  const std::uint32_t acceptedFlag
      = index < pairCount && accepted[index] != 0u ? 1u : 0u;
  localRanks[local] = acceptedFlag;
  __syncthreads();

  for (unsigned int offset = 1u; offset < blockDim.x; offset <<= 1u) {
    std::uint32_t previous = 0u;
    if (local >= offset) {
      previous = localRanks[local - offset];
    }
    __syncthreads();
    localRanks[local] += previous;
    __syncthreads();
  }

  if (acceptedFlag == 0u) {
    return;
  }

  const std::size_t edgeA = index / edgeCount;
  const std::size_t edgeB = index - edgeA * edgeCount;
  const std::uint32_t compactedIndex
      = acceptedBlockOffsets[blockIdx.x] + localRanks[local] - 1u;
  acceptedEdgeAIndices[compactedIndex] = static_cast<std::uint32_t>(edgeA);
  acceptedEdgeBIndices[compactedIndex] = static_cast<std::uint32_t>(edgeB);
}

} // namespace

//==============================================================================
cudaError_t launchPointTriangleContactStencilFilterKernel(
    const double* positions,
    const std::uint32_t* triangleIndices,
    const PointTriangleContactStencil* stencils,
    double activationDistance,
    double* squaredDistances,
    std::uint8_t* accepted,
    std::size_t stencilCount)
{
  if (stencilCount == 0) {
    return cudaSuccess;
  }

  constexpr unsigned int blockSize = 256;
  const unsigned int gridSize = launchGrid1D(stencilCount, blockSize);
  filterPointTriangleContactStencilKernel<<<gridSize, blockSize>>>(
      positions,
      triangleIndices,
      stencils,
      activationDistance,
      squaredDistances,
      accepted,
      stencilCount);

  return cudaGetLastError();
}

//==============================================================================
cudaError_t launchPointTriangleContactCandidateMaskKernel(
    const double* positions,
    const std::uint32_t* pointIndices,
    const std::uint32_t* triangleIndices,
    double activationDistance,
    double* squaredDistances,
    std::uint8_t* accepted,
    std::uint32_t* acceptedPointIndices,
    std::uint32_t* acceptedTriangleIndices,
    std::uint32_t* compactedCount,
    std::uint32_t* acceptedBlockCounts,
    std::uint32_t* acceptedBlockOffsets,
    std::size_t pointCount,
    std::size_t triangleCount)
{
  const std::size_t pairCount = pointCount * triangleCount;
  if (pairCount == 0) {
    return cudaSuccess;
  }

  constexpr unsigned int blockSize = 256;
  const unsigned int gridSize = launchGrid1D(pairCount, blockSize);
  buildPointTriangleContactCandidateMaskKernel<<<gridSize, blockSize>>>(
      positions,
      pointIndices,
      triangleIndices,
      activationDistance,
      squaredDistances,
      accepted,
      pointCount,
      triangleCount);
  cudaError_t error = cudaGetLastError();
  if (error != cudaSuccess) {
    return error;
  }

  countPointTriangleAcceptedBlocksKernel<<<
      gridSize,
      blockSize,
      blockSize * sizeof(std::uint32_t)>>>(
      accepted, acceptedBlockCounts, pairCount);
  error = cudaGetLastError();
  if (error != cudaSuccess) {
    return error;
  }

  prefixPointTriangleAcceptedBlockCountsKernel<<<1u, 1u>>>(
      acceptedBlockCounts, acceptedBlockOffsets, compactedCount, gridSize);
  error = cudaGetLastError();
  if (error != cudaSuccess) {
    return error;
  }

  compactPointTriangleContactCandidateMaskKernel<<<
      gridSize,
      blockSize,
      blockSize * sizeof(std::uint32_t)>>>(
      pointIndices,
      accepted,
      acceptedBlockOffsets,
      acceptedPointIndices,
      acceptedTriangleIndices,
      pointCount,
      triangleCount);

  return cudaGetLastError();
}

//==============================================================================
cudaError_t launchEdgeEdgeContactStencilFilterKernel(
    const double* positions,
    const EdgeEdgeContactStencil* stencils,
    double activationDistance,
    double* squaredDistances,
    std::uint8_t* accepted,
    std::size_t stencilCount)
{
  if (stencilCount == 0) {
    return cudaSuccess;
  }

  constexpr unsigned int blockSize = 256;
  const unsigned int gridSize = launchGrid1D(stencilCount, blockSize);
  filterEdgeEdgeContactStencilKernel<<<gridSize, blockSize>>>(
      positions,
      stencils,
      activationDistance,
      squaredDistances,
      accepted,
      stencilCount);

  return cudaGetLastError();
}

//==============================================================================
cudaError_t launchEdgeEdgeContactCandidateMaskKernel(
    const double* positions,
    const std::uint32_t* edgeIndices,
    double activationDistance,
    double* squaredDistances,
    std::uint8_t* accepted,
    std::uint32_t* acceptedEdgeAIndices,
    std::uint32_t* acceptedEdgeBIndices,
    std::uint32_t* compactedCount,
    std::uint32_t* acceptedBlockCounts,
    std::uint32_t* acceptedBlockOffsets,
    std::size_t edgeCount)
{
  const std::size_t pairCount = edgeCount * edgeCount;
  if (pairCount == 0) {
    return cudaSuccess;
  }

  constexpr unsigned int blockSize = 256;
  const unsigned int gridSize = launchGrid1D(pairCount, blockSize);
  buildEdgeEdgeContactCandidateMaskKernel<<<gridSize, blockSize>>>(
      positions,
      edgeIndices,
      activationDistance,
      squaredDistances,
      accepted,
      edgeCount);
  cudaError_t error = cudaGetLastError();
  if (error != cudaSuccess) {
    return error;
  }

  countPointTriangleAcceptedBlocksKernel<<<
      gridSize,
      blockSize,
      blockSize * sizeof(std::uint32_t)>>>(
      accepted, acceptedBlockCounts, pairCount);
  error = cudaGetLastError();
  if (error != cudaSuccess) {
    return error;
  }

  prefixPointTriangleAcceptedBlockCountsKernel<<<1u, 1u>>>(
      acceptedBlockCounts, acceptedBlockOffsets, compactedCount, gridSize);
  error = cudaGetLastError();
  if (error != cudaSuccess) {
    return error;
  }

  compactEdgeEdgeContactCandidateMaskKernel<<<
      gridSize,
      blockSize,
      blockSize * sizeof(std::uint32_t)>>>(
      accepted,
      acceptedBlockOffsets,
      acceptedEdgeAIndices,
      acceptedEdgeBIndices,
      edgeCount);

  return cudaGetLastError();
}

} // namespace dart::simulation::compute::cuda::detail
