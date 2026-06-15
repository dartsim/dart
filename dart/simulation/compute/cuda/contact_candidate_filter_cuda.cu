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

__device__ Vec3 componentwiseMin(const Vec3 a, const Vec3 b)
{
  return Vec3{fmin(a.x, b.x), fmin(a.y, b.y), fmin(a.z, b.z)};
}

__device__ Vec3 componentwiseMax(const Vec3 a, const Vec3 b)
{
  return Vec3{fmax(a.x, b.x), fmax(a.y, b.y), fmax(a.z, b.z)};
}

struct CandidateAabb
{
  Vec3 min;
  Vec3 max;
};

__device__ CandidateAabb itemAabb(const ContactCandidateSweepAabbItem item)
{
  return CandidateAabb{
      Vec3{item.minX, item.minY, item.minZ},
      Vec3{item.maxX, item.maxY, item.maxZ}};
}

__device__ ContactCandidateSweepAabbItem
makeSweepItem(const CandidateAabb aabb, const std::uint32_t id)
{
  return ContactCandidateSweepAabbItem{
      aabb.min.x,
      aabb.min.y,
      aabb.min.z,
      aabb.max.x,
      aabb.max.y,
      aabb.max.z,
      id};
}

__device__ ContactCandidateSweepAabbItem sentinelSweepItem()
{
  constexpr double kSentinel = 1.0e300;
  return ContactCandidateSweepAabbItem{
      kSentinel,
      kSentinel,
      kSentinel,
      kSentinel,
      kSentinel,
      kSentinel,
      0xffffffffu};
}

__device__ bool lessSweepItem(
    const ContactCandidateSweepAabbItem lhs,
    const ContactCandidateSweepAabbItem rhs)
{
  if (lhs.minX != rhs.minX) {
    return lhs.minX < rhs.minX;
  }
  if (lhs.maxX != rhs.maxX) {
    return lhs.maxX < rhs.maxX;
  }
  return lhs.id < rhs.id;
}

__device__ CandidateAabb expandAabb(CandidateAabb aabb, const double margin)
{
  const double nonnegativeMargin = fmax(0.0, margin);
  aabb.min.x -= nonnegativeMargin;
  aabb.min.y -= nonnegativeMargin;
  aabb.min.z -= nonnegativeMargin;
  aabb.max.x += nonnegativeMargin;
  aabb.max.y += nonnegativeMargin;
  aabb.max.z += nonnegativeMargin;
  return aabb;
}

__device__ bool overlaps(const CandidateAabb a, const CandidateAabb b)
{
  return a.min.x <= b.max.x && a.max.x >= b.min.x && a.min.y <= b.max.y
         && a.max.y >= b.min.y && a.min.z <= b.max.z && a.max.z >= b.min.z;
}

__device__ CandidateAabb
makeSweptPointAabb(const Vec3 start, const Vec3 end, const double margin)
{
  return expandAabb(
      CandidateAabb{componentwiseMin(start, end), componentwiseMax(start, end)},
      margin);
}

__device__ CandidateAabb makeSweptSegmentAabb(
    const Vec3 aStart,
    const Vec3 aEnd,
    const Vec3 bStart,
    const Vec3 bEnd,
    const double margin)
{
  return expandAabb(
      CandidateAabb{
          componentwiseMin(
              componentwiseMin(aStart, aEnd), componentwiseMin(bStart, bEnd)),
          componentwiseMax(
              componentwiseMax(aStart, aEnd), componentwiseMax(bStart, bEnd))},
      margin);
}

__device__ CandidateAabb makeSweptTriangleAabb(
    const Vec3 aStart,
    const Vec3 aEnd,
    const Vec3 bStart,
    const Vec3 bEnd,
    const Vec3 cStart,
    const Vec3 cEnd,
    const double margin)
{
  return expandAabb(
      CandidateAabb{
          componentwiseMin(
              componentwiseMin(
                  componentwiseMin(aStart, aEnd),
                  componentwiseMin(bStart, bEnd)),
              componentwiseMin(cStart, cEnd)),
          componentwiseMax(
              componentwiseMax(
                  componentwiseMax(aStart, aEnd),
                  componentwiseMax(bStart, bEnd)),
              componentwiseMax(cStart, cEnd))},
      margin);
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
    const double* squaredDistances,
    const std::uint8_t* accepted,
    const std::uint32_t* acceptedBlockOffsets,
    std::uint32_t* acceptedPointIndices,
    std::uint32_t* acceptedTriangleIndices,
    double* acceptedSquaredDistances,
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
  acceptedSquaredDistances[compactedIndex] = squaredDistances[index];
}

__global__ void buildSweptPointTriangleContactCandidateMaskKernel(
    const double* startPositions,
    const double* endPositions,
    const std::uint32_t* pointIndices,
    const std::uint32_t* triangleIndices,
    const double activationDistance,
    double* endpointSquaredDistances,
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
    endpointSquaredDistances[index] = 0.0;
    accepted[index] = 0u;
    return;
  }

  const double margin = 0.5 * activationDistance;
  const CandidateAabb pointAabb = makeSweptPointAabb(
      loadPoint(startPositions, point), loadPoint(endPositions, point), margin);
  const CandidateAabb triangleAabb = makeSweptTriangleAabb(
      loadPoint(startPositions, aIndex),
      loadPoint(endPositions, aIndex),
      loadPoint(startPositions, bIndex),
      loadPoint(endPositions, bIndex),
      loadPoint(startPositions, cIndex),
      loadPoint(endPositions, cIndex),
      margin);

  if (!overlaps(pointAabb, triangleAabb)) {
    endpointSquaredDistances[index] = 0.0;
    accepted[index] = 0u;
    return;
  }

  const double startSquaredDistance = pointTriangleSquaredDistance(
      loadPoint(startPositions, point),
      loadPoint(startPositions, aIndex),
      loadPoint(startPositions, bIndex),
      loadPoint(startPositions, cIndex));
  const double endSquaredDistance = pointTriangleSquaredDistance(
      loadPoint(endPositions, point),
      loadPoint(endPositions, aIndex),
      loadPoint(endPositions, bIndex),
      loadPoint(endPositions, cIndex));
  endpointSquaredDistances[index]
      = fmin(startSquaredDistance, endSquaredDistance);
  accepted[index] = 1u;
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
    const double* squaredDistances,
    const std::uint8_t* accepted,
    const std::uint32_t* acceptedBlockOffsets,
    std::uint32_t* acceptedEdgeAIndices,
    std::uint32_t* acceptedEdgeBIndices,
    double* acceptedSquaredDistances,
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
  acceptedSquaredDistances[compactedIndex] = squaredDistances[index];
}

__global__ void buildSweptEdgeEdgeContactCandidateMaskKernel(
    const double* startPositions,
    const double* endPositions,
    const std::uint32_t* edgeIndices,
    const double activationDistance,
    double* endpointSquaredDistances,
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
    endpointSquaredDistances[index] = 0.0;
    accepted[index] = 0u;
    return;
  }

  const std::uint32_t a0 = edgeIndices[2u * edgeA];
  const std::uint32_t a1 = edgeIndices[2u * edgeA + 1u];
  const std::uint32_t b0 = edgeIndices[2u * edgeB];
  const std::uint32_t b1 = edgeIndices[2u * edgeB + 1u];
  if (edgesShareVertex(a0, a1, b0, b1)) {
    endpointSquaredDistances[index] = 0.0;
    accepted[index] = 0u;
    return;
  }

  const double margin = 0.5 * activationDistance;
  const CandidateAabb edgeAAabb = makeSweptSegmentAabb(
      loadPoint(startPositions, a0),
      loadPoint(endPositions, a0),
      loadPoint(startPositions, a1),
      loadPoint(endPositions, a1),
      margin);
  const CandidateAabb edgeBAabb = makeSweptSegmentAabb(
      loadPoint(startPositions, b0),
      loadPoint(endPositions, b0),
      loadPoint(startPositions, b1),
      loadPoint(endPositions, b1),
      margin);

  if (!overlaps(edgeAAabb, edgeBAabb)) {
    endpointSquaredDistances[index] = 0.0;
    accepted[index] = 0u;
    return;
  }

  const double startSquaredDistance = edgeEdgeSquaredDistance(
      loadPoint(startPositions, a0),
      loadPoint(startPositions, a1),
      loadPoint(startPositions, b0),
      loadPoint(startPositions, b1));
  const double endSquaredDistance = edgeEdgeSquaredDistance(
      loadPoint(endPositions, a0),
      loadPoint(endPositions, a1),
      loadPoint(endPositions, b0),
      loadPoint(endPositions, b1));
  endpointSquaredDistances[index]
      = fmin(startSquaredDistance, endSquaredDistance);
  accepted[index] = 1u;
}

__global__ void buildSweptPointSweepItemsKernel(
    const double* startPositions,
    const double* endPositions,
    const std::uint32_t* pointIndices,
    const double activationDistance,
    ContactCandidateSweepAabbItem* pointItems,
    const std::size_t pointCount,
    const std::size_t paddedPointCount)
{
  const auto index
      = static_cast<std::size_t>(blockIdx.x * blockDim.x + threadIdx.x);
  if (index >= paddedPointCount) {
    return;
  }
  if (index >= pointCount) {
    pointItems[index] = sentinelSweepItem();
    return;
  }

  const std::uint32_t point = pointIndices[index];
  pointItems[index] = makeSweepItem(
      makeSweptPointAabb(
          loadPoint(startPositions, point),
          loadPoint(endPositions, point),
          0.5 * activationDistance),
      point);
}

__global__ void buildSweptTriangleSweepItemsKernel(
    const double* startPositions,
    const double* endPositions,
    const std::uint32_t* triangleIndices,
    const double activationDistance,
    ContactCandidateSweepAabbItem* triangleItems,
    const std::size_t triangleCount,
    const std::size_t paddedTriangleCount)
{
  const auto triangle
      = static_cast<std::size_t>(blockIdx.x * blockDim.x + threadIdx.x);
  if (triangle >= paddedTriangleCount) {
    return;
  }
  if (triangle >= triangleCount) {
    triangleItems[triangle] = sentinelSweepItem();
    return;
  }

  const std::size_t triangleBase = 3u * triangle;
  const std::uint32_t aIndex = triangleIndices[triangleBase];
  const std::uint32_t bIndex = triangleIndices[triangleBase + 1u];
  const std::uint32_t cIndex = triangleIndices[triangleBase + 2u];
  triangleItems[triangle] = makeSweepItem(
      makeSweptTriangleAabb(
          loadPoint(startPositions, aIndex),
          loadPoint(endPositions, aIndex),
          loadPoint(startPositions, bIndex),
          loadPoint(endPositions, bIndex),
          loadPoint(startPositions, cIndex),
          loadPoint(endPositions, cIndex),
          0.5 * activationDistance),
      static_cast<std::uint32_t>(triangle));
}

__global__ void buildSweptEdgeSweepItemsKernel(
    const double* startPositions,
    const double* endPositions,
    const std::uint32_t* edgeIndices,
    const double activationDistance,
    ContactCandidateSweepAabbItem* edgeItems,
    const std::size_t edgeCount,
    const std::size_t paddedEdgeCount)
{
  const auto edge
      = static_cast<std::size_t>(blockIdx.x * blockDim.x + threadIdx.x);
  if (edge >= paddedEdgeCount) {
    return;
  }
  if (edge >= edgeCount) {
    edgeItems[edge] = sentinelSweepItem();
    return;
  }

  const std::uint32_t a0 = edgeIndices[2u * edge];
  const std::uint32_t a1 = edgeIndices[2u * edge + 1u];
  edgeItems[edge] = makeSweepItem(
      makeSweptSegmentAabb(
          loadPoint(startPositions, a0),
          loadPoint(endPositions, a0),
          loadPoint(startPositions, a1),
          loadPoint(endPositions, a1),
          0.5 * activationDistance),
      static_cast<std::uint32_t>(edge));
}

__global__ void bitonicSortSweepItemsKernel(
    ContactCandidateSweepAabbItem* items,
    const std::size_t count,
    const std::size_t stride,
    const std::size_t stage)
{
  const auto index
      = static_cast<std::size_t>(blockIdx.x * blockDim.x + threadIdx.x);
  if (index >= count) {
    return;
  }

  const std::size_t other = index ^ stride;
  if (other <= index || other >= count) {
    return;
  }

  const bool ascending = (index & stage) == 0u;
  const ContactCandidateSweepAabbItem lhs = items[index];
  const ContactCandidateSweepAabbItem rhs = items[other];
  const bool swap
      = ascending ? lessSweepItem(rhs, lhs) : lessSweepItem(lhs, rhs);
  if (swap) {
    items[index] = rhs;
    items[other] = lhs;
  }
}

__global__ void countSweptPointTriangleSweepPairsKernel(
    const ContactCandidateSweepAabbItem* pointItems,
    const ContactCandidateSweepAabbItem* triangleItems,
    const std::uint32_t* triangleIndices,
    std::uint32_t* pairCounts,
    const std::size_t pointCount,
    const std::size_t triangleCount)
{
  const auto pointSlot
      = static_cast<std::size_t>(blockIdx.x * blockDim.x + threadIdx.x);
  if (pointSlot >= pointCount) {
    return;
  }

  const ContactCandidateSweepAabbItem pointItem = pointItems[pointSlot];
  const CandidateAabb pointAabb = itemAabb(pointItem);
  std::uint32_t count = 0u;
  for (std::size_t triangleSlot = 0; triangleSlot < triangleCount;
       ++triangleSlot) {
    const ContactCandidateSweepAabbItem triangleItem
        = triangleItems[triangleSlot];
    if (triangleItem.minX > pointItem.maxX) {
      break;
    }
    if (triangleItem.maxX < pointItem.minX) {
      continue;
    }
    const std::size_t triangleBase
        = 3u * static_cast<std::size_t>(triangleItem.id);
    const std::uint32_t aIndex = triangleIndices[triangleBase];
    const std::uint32_t bIndex = triangleIndices[triangleBase + 1u];
    const std::uint32_t cIndex = triangleIndices[triangleBase + 2u];
    if (pointItem.id == aIndex || pointItem.id == bIndex
        || pointItem.id == cIndex) {
      continue;
    }
    if (overlaps(pointAabb, itemAabb(triangleItem))) {
      ++count;
    }
  }
  pairCounts[pointSlot] = count;
}

__global__ void prefixSweepPairCountsKernel(
    const std::uint32_t* pairCounts,
    std::uint32_t* pairOffsets,
    std::uint32_t* compactedCount,
    const std::size_t count)
{
  if (blockIdx.x != 0u || threadIdx.x != 0u) {
    return;
  }

  std::uint32_t offset = 0u;
  for (std::size_t i = 0; i < count; ++i) {
    pairOffsets[i] = offset;
    offset += pairCounts[i];
  }
  *compactedCount = offset;
}

__global__ void scatterSweptPointTriangleSweepPairsKernel(
    const double* startPositions,
    const double* endPositions,
    const ContactCandidateSweepAabbItem* pointItems,
    const ContactCandidateSweepAabbItem* triangleItems,
    const std::uint32_t* triangleIndices,
    const std::uint32_t* pairOffsets,
    std::uint32_t* acceptedPointIndices,
    std::uint32_t* acceptedTriangleIndices,
    double* acceptedEndpointSquaredDistances,
    const std::size_t pointCount,
    const std::size_t triangleCount)
{
  const auto pointSlot
      = static_cast<std::size_t>(blockIdx.x * blockDim.x + threadIdx.x);
  if (pointSlot >= pointCount) {
    return;
  }

  const ContactCandidateSweepAabbItem pointItem = pointItems[pointSlot];
  const CandidateAabb pointAabb = itemAabb(pointItem);
  std::uint32_t local = 0u;
  for (std::size_t triangleSlot = 0; triangleSlot < triangleCount;
       ++triangleSlot) {
    const ContactCandidateSweepAabbItem triangleItem
        = triangleItems[triangleSlot];
    if (triangleItem.minX > pointItem.maxX) {
      break;
    }
    if (triangleItem.maxX < pointItem.minX) {
      continue;
    }
    const std::size_t triangleBase
        = 3u * static_cast<std::size_t>(triangleItem.id);
    const std::uint32_t aIndex = triangleIndices[triangleBase];
    const std::uint32_t bIndex = triangleIndices[triangleBase + 1u];
    const std::uint32_t cIndex = triangleIndices[triangleBase + 2u];
    if (pointItem.id == aIndex || pointItem.id == bIndex
        || pointItem.id == cIndex) {
      continue;
    }
    if (!overlaps(pointAabb, itemAabb(triangleItem))) {
      continue;
    }

    const std::uint32_t output = pairOffsets[pointSlot] + local;
    const double startSquaredDistance = pointTriangleSquaredDistance(
        loadPoint(startPositions, pointItem.id),
        loadPoint(startPositions, aIndex),
        loadPoint(startPositions, bIndex),
        loadPoint(startPositions, cIndex));
    const double endSquaredDistance = pointTriangleSquaredDistance(
        loadPoint(endPositions, pointItem.id),
        loadPoint(endPositions, aIndex),
        loadPoint(endPositions, bIndex),
        loadPoint(endPositions, cIndex));
    acceptedPointIndices[output] = pointItem.id;
    acceptedTriangleIndices[output] = triangleItem.id;
    acceptedEndpointSquaredDistances[output]
        = fmin(startSquaredDistance, endSquaredDistance);
    ++local;
  }
}

__global__ void countSweptEdgeEdgeSweepPairsKernel(
    const ContactCandidateSweepAabbItem* edgeItems,
    const std::uint32_t* edgeIndices,
    std::uint32_t* pairCounts,
    const std::size_t edgeCount)
{
  const auto edgeSlot
      = static_cast<std::size_t>(blockIdx.x * blockDim.x + threadIdx.x);
  if (edgeSlot >= edgeCount) {
    return;
  }

  const ContactCandidateSweepAabbItem edgeItem = edgeItems[edgeSlot];
  const CandidateAabb edgeAabb = itemAabb(edgeItem);
  const std::uint32_t a0 = edgeIndices[2u * edgeItem.id];
  const std::uint32_t a1 = edgeIndices[2u * edgeItem.id + 1u];
  std::uint32_t count = 0u;
  for (std::size_t otherSlot = edgeSlot + 1u; otherSlot < edgeCount;
       ++otherSlot) {
    const ContactCandidateSweepAabbItem otherItem = edgeItems[otherSlot];
    if (otherItem.minX > edgeItem.maxX) {
      break;
    }
    const std::uint32_t b0 = edgeIndices[2u * otherItem.id];
    const std::uint32_t b1 = edgeIndices[2u * otherItem.id + 1u];
    if (edgesShareVertex(a0, a1, b0, b1)) {
      continue;
    }
    if (overlaps(edgeAabb, itemAabb(otherItem))) {
      ++count;
    }
  }
  pairCounts[edgeSlot] = count;
}

__global__ void scatterSweptEdgeEdgeSweepPairsKernel(
    const double* startPositions,
    const double* endPositions,
    const ContactCandidateSweepAabbItem* edgeItems,
    const std::uint32_t* edgeIndices,
    const std::uint32_t* pairOffsets,
    std::uint32_t* acceptedEdgeAIndices,
    std::uint32_t* acceptedEdgeBIndices,
    double* acceptedEndpointSquaredDistances,
    const std::size_t edgeCount)
{
  const auto edgeSlot
      = static_cast<std::size_t>(blockIdx.x * blockDim.x + threadIdx.x);
  if (edgeSlot >= edgeCount) {
    return;
  }

  const ContactCandidateSweepAabbItem edgeItem = edgeItems[edgeSlot];
  const CandidateAabb edgeAabb = itemAabb(edgeItem);
  const std::uint32_t a0 = edgeIndices[2u * edgeItem.id];
  const std::uint32_t a1 = edgeIndices[2u * edgeItem.id + 1u];
  std::uint32_t local = 0u;
  for (std::size_t otherSlot = edgeSlot + 1u; otherSlot < edgeCount;
       ++otherSlot) {
    const ContactCandidateSweepAabbItem otherItem = edgeItems[otherSlot];
    if (otherItem.minX > edgeItem.maxX) {
      break;
    }
    const std::uint32_t b0 = edgeIndices[2u * otherItem.id];
    const std::uint32_t b1 = edgeIndices[2u * otherItem.id + 1u];
    if (edgesShareVertex(a0, a1, b0, b1)) {
      continue;
    }
    if (!overlaps(edgeAabb, itemAabb(otherItem))) {
      continue;
    }

    const std::uint32_t output = pairOffsets[edgeSlot] + local;
    const double startSquaredDistance = edgeEdgeSquaredDistance(
        loadPoint(startPositions, a0),
        loadPoint(startPositions, a1),
        loadPoint(startPositions, b0),
        loadPoint(startPositions, b1));
    const double endSquaredDistance = edgeEdgeSquaredDistance(
        loadPoint(endPositions, a0),
        loadPoint(endPositions, a1),
        loadPoint(endPositions, b0),
        loadPoint(endPositions, b1));
    acceptedEdgeAIndices[output] = edgeItem.id;
    acceptedEdgeBIndices[output] = otherItem.id;
    acceptedEndpointSquaredDistances[output]
        = fmin(startSquaredDistance, endSquaredDistance);
    ++local;
  }
}

__global__ void evaluateSweptPointTriangleCandidateBufferKernel(
    const double* startPositions,
    const double* endPositions,
    const std::uint32_t* triangleIndices,
    const std::uint32_t* candidatePointIndices,
    const std::uint32_t* candidateTriangleIndices,
    double* endpointSquaredDistances,
    const std::size_t pairCount)
{
  const auto index
      = static_cast<std::size_t>(blockIdx.x * blockDim.x + threadIdx.x);
  if (index >= pairCount) {
    return;
  }

  const std::uint32_t point = candidatePointIndices[index];
  const std::size_t triangleBase
      = 3u * static_cast<std::size_t>(candidateTriangleIndices[index]);
  const std::uint32_t aIndex = triangleIndices[triangleBase];
  const std::uint32_t bIndex = triangleIndices[triangleBase + 1u];
  const std::uint32_t cIndex = triangleIndices[triangleBase + 2u];

  const double startSquaredDistance = pointTriangleSquaredDistance(
      loadPoint(startPositions, point),
      loadPoint(startPositions, aIndex),
      loadPoint(startPositions, bIndex),
      loadPoint(startPositions, cIndex));
  const double endSquaredDistance = pointTriangleSquaredDistance(
      loadPoint(endPositions, point),
      loadPoint(endPositions, aIndex),
      loadPoint(endPositions, bIndex),
      loadPoint(endPositions, cIndex));
  endpointSquaredDistances[index]
      = fmin(startSquaredDistance, endSquaredDistance);
}

__global__ void evaluateSweptEdgeEdgeCandidateBufferKernel(
    const double* startPositions,
    const double* endPositions,
    const std::uint32_t* edgeIndices,
    const std::uint32_t* candidateEdgeAIndices,
    const std::uint32_t* candidateEdgeBIndices,
    double* endpointSquaredDistances,
    const std::size_t pairCount)
{
  const auto index
      = static_cast<std::size_t>(blockIdx.x * blockDim.x + threadIdx.x);
  if (index >= pairCount) {
    return;
  }

  const std::size_t edgeA = candidateEdgeAIndices[index];
  const std::size_t edgeB = candidateEdgeBIndices[index];
  const std::uint32_t a0 = edgeIndices[2u * edgeA];
  const std::uint32_t a1 = edgeIndices[2u * edgeA + 1u];
  const std::uint32_t b0 = edgeIndices[2u * edgeB];
  const std::uint32_t b1 = edgeIndices[2u * edgeB + 1u];

  const double startSquaredDistance = edgeEdgeSquaredDistance(
      loadPoint(startPositions, a0),
      loadPoint(startPositions, a1),
      loadPoint(startPositions, b0),
      loadPoint(startPositions, b1));
  const double endSquaredDistance = edgeEdgeSquaredDistance(
      loadPoint(endPositions, a0),
      loadPoint(endPositions, a1),
      loadPoint(endPositions, b0),
      loadPoint(endPositions, b1));
  endpointSquaredDistances[index]
      = fmin(startSquaredDistance, endSquaredDistance);
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
    double* acceptedSquaredDistances,
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
      squaredDistances,
      accepted,
      acceptedBlockOffsets,
      acceptedPointIndices,
      acceptedTriangleIndices,
      acceptedSquaredDistances,
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
    double* acceptedSquaredDistances,
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
      squaredDistances,
      accepted,
      acceptedBlockOffsets,
      acceptedEdgeAIndices,
      acceptedEdgeBIndices,
      acceptedSquaredDistances,
      edgeCount);

  return cudaGetLastError();
}

//==============================================================================
cudaError_t launchSweptPointTriangleContactCandidateMaskKernel(
    const double* startPositions,
    const double* endPositions,
    const std::uint32_t* pointIndices,
    const std::uint32_t* triangleIndices,
    double activationDistance,
    double* endpointSquaredDistances,
    std::uint8_t* accepted,
    std::uint32_t* acceptedPointIndices,
    std::uint32_t* acceptedTriangleIndices,
    double* acceptedEndpointSquaredDistances,
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
  buildSweptPointTriangleContactCandidateMaskKernel<<<gridSize, blockSize>>>(
      startPositions,
      endPositions,
      pointIndices,
      triangleIndices,
      activationDistance,
      endpointSquaredDistances,
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
      endpointSquaredDistances,
      accepted,
      acceptedBlockOffsets,
      acceptedPointIndices,
      acceptedTriangleIndices,
      acceptedEndpointSquaredDistances,
      pointCount,
      triangleCount);

  return cudaGetLastError();
}

//==============================================================================
cudaError_t launchSweptEdgeEdgeContactCandidateMaskKernel(
    const double* startPositions,
    const double* endPositions,
    const std::uint32_t* edgeIndices,
    double activationDistance,
    double* endpointSquaredDistances,
    std::uint8_t* accepted,
    std::uint32_t* acceptedEdgeAIndices,
    std::uint32_t* acceptedEdgeBIndices,
    double* acceptedEndpointSquaredDistances,
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
  buildSweptEdgeEdgeContactCandidateMaskKernel<<<gridSize, blockSize>>>(
      startPositions,
      endPositions,
      edgeIndices,
      activationDistance,
      endpointSquaredDistances,
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
      endpointSquaredDistances,
      accepted,
      acceptedBlockOffsets,
      acceptedEdgeAIndices,
      acceptedEdgeBIndices,
      acceptedEndpointSquaredDistances,
      edgeCount);

  return cudaGetLastError();
}

//==============================================================================
cudaError_t sortSweepItemsOnDevice(
    ContactCandidateSweepAabbItem* items, const std::size_t count)
{
  if (count <= 1u) {
    return cudaSuccess;
  }

  constexpr unsigned int blockSize = 256;
  const unsigned int gridSize = launchGrid1D(count, blockSize);
  for (std::size_t stage = 2u; stage <= count; stage <<= 1u) {
    for (std::size_t stride = stage >> 1u; stride > 0u; stride >>= 1u) {
      bitonicSortSweepItemsKernel<<<gridSize, blockSize>>>(
          items, count, stride, stage);
      cudaError_t error = cudaGetLastError();
      if (error != cudaSuccess) {
        return error;
      }
    }
  }

  return cudaSuccess;
}

//==============================================================================
// Count phase: build + sort sweep items, count per-point accepted pairs, and
// exclusive-prefix them into pairOffsets. After this the host can read
// compactedCount and allocate the accepted output buffers to the exact number
// of candidates instead of the all-pairs capacity, then call the scatter phase.
cudaError_t launchSweptPointTriangleSweepCountKernel(
    const double* startPositions,
    const double* endPositions,
    const std::uint32_t* pointIndices,
    const std::uint32_t* triangleIndices,
    double activationDistance,
    ContactCandidateSweepAabbItem* pointItems,
    ContactCandidateSweepAabbItem* triangleItems,
    std::uint32_t* pairCounts,
    std::uint32_t* pairOffsets,
    std::uint32_t* compactedCount,
    std::size_t pointCount,
    std::size_t triangleCount,
    std::size_t paddedPointCount,
    std::size_t paddedTriangleCount)
{
  if (pointCount == 0 || triangleCount == 0) {
    return cudaSuccess;
  }

  constexpr unsigned int blockSize = 256;
  unsigned int gridSize = launchGrid1D(paddedPointCount, blockSize);
  buildSweptPointSweepItemsKernel<<<gridSize, blockSize>>>(
      startPositions,
      endPositions,
      pointIndices,
      activationDistance,
      pointItems,
      pointCount,
      paddedPointCount);
  cudaError_t error = cudaGetLastError();
  if (error != cudaSuccess) {
    return error;
  }

  gridSize = launchGrid1D(paddedTriangleCount, blockSize);
  buildSweptTriangleSweepItemsKernel<<<gridSize, blockSize>>>(
      startPositions,
      endPositions,
      triangleIndices,
      activationDistance,
      triangleItems,
      triangleCount,
      paddedTriangleCount);
  error = cudaGetLastError();
  if (error != cudaSuccess) {
    return error;
  }

  error = sortSweepItemsOnDevice(pointItems, paddedPointCount);
  if (error != cudaSuccess) {
    return error;
  }
  error = sortSweepItemsOnDevice(triangleItems, paddedTriangleCount);
  if (error != cudaSuccess) {
    return error;
  }

  gridSize = launchGrid1D(pointCount, blockSize);
  countSweptPointTriangleSweepPairsKernel<<<gridSize, blockSize>>>(
      pointItems,
      triangleItems,
      triangleIndices,
      pairCounts,
      pointCount,
      triangleCount);
  error = cudaGetLastError();
  if (error != cudaSuccess) {
    return error;
  }

  prefixSweepPairCountsKernel<<<1u, 1u>>>(
      pairCounts, pairOffsets, compactedCount, pointCount);
  return cudaGetLastError();
}

// Scatter phase: write the accepted candidate pairs into the right-sized output
// buffers using the offsets from the count phase. pointItems/triangleItems must
// already be the sorted items produced by the count phase.
cudaError_t launchSweptPointTriangleSweepScatterKernel(
    const double* startPositions,
    const double* endPositions,
    const std::uint32_t* triangleIndices,
    const ContactCandidateSweepAabbItem* pointItems,
    const ContactCandidateSweepAabbItem* triangleItems,
    const std::uint32_t* pairOffsets,
    std::uint32_t* acceptedPointIndices,
    std::uint32_t* acceptedTriangleIndices,
    double* acceptedEndpointSquaredDistances,
    std::size_t pointCount,
    std::size_t triangleCount)
{
  if (pointCount == 0 || triangleCount == 0) {
    return cudaSuccess;
  }

  constexpr unsigned int blockSize = 256;
  const unsigned int gridSize = launchGrid1D(pointCount, blockSize);
  scatterSweptPointTriangleSweepPairsKernel<<<gridSize, blockSize>>>(
      startPositions,
      endPositions,
      pointItems,
      triangleItems,
      triangleIndices,
      pairOffsets,
      acceptedPointIndices,
      acceptedTriangleIndices,
      acceptedEndpointSquaredDistances,
      pointCount,
      triangleCount);

  return cudaGetLastError();
}

//==============================================================================
cudaError_t launchSweptEdgeEdgeSweepCountKernel(
    const double* startPositions,
    const double* endPositions,
    const std::uint32_t* edgeIndices,
    double activationDistance,
    ContactCandidateSweepAabbItem* edgeItems,
    std::uint32_t* pairCounts,
    std::uint32_t* pairOffsets,
    std::uint32_t* compactedCount,
    std::size_t edgeCount,
    std::size_t paddedEdgeCount)
{
  if (edgeCount == 0) {
    return cudaSuccess;
  }

  constexpr unsigned int blockSize = 256;
  unsigned int gridSize = launchGrid1D(paddedEdgeCount, blockSize);
  buildSweptEdgeSweepItemsKernel<<<gridSize, blockSize>>>(
      startPositions,
      endPositions,
      edgeIndices,
      activationDistance,
      edgeItems,
      edgeCount,
      paddedEdgeCount);
  cudaError_t error = cudaGetLastError();
  if (error != cudaSuccess) {
    return error;
  }

  error = sortSweepItemsOnDevice(edgeItems, paddedEdgeCount);
  if (error != cudaSuccess) {
    return error;
  }

  gridSize = launchGrid1D(edgeCount, blockSize);
  countSweptEdgeEdgeSweepPairsKernel<<<gridSize, blockSize>>>(
      edgeItems, edgeIndices, pairCounts, edgeCount);
  error = cudaGetLastError();
  if (error != cudaSuccess) {
    return error;
  }

  prefixSweepPairCountsKernel<<<1u, 1u>>>(
      pairCounts, pairOffsets, compactedCount, edgeCount);
  return cudaGetLastError();
}

cudaError_t launchSweptEdgeEdgeSweepScatterKernel(
    const double* startPositions,
    const double* endPositions,
    const std::uint32_t* edgeIndices,
    const ContactCandidateSweepAabbItem* edgeItems,
    const std::uint32_t* pairOffsets,
    std::uint32_t* acceptedEdgeAIndices,
    std::uint32_t* acceptedEdgeBIndices,
    double* acceptedEndpointSquaredDistances,
    std::size_t edgeCount)
{
  if (edgeCount == 0) {
    return cudaSuccess;
  }

  constexpr unsigned int blockSize = 256;
  const unsigned int gridSize = launchGrid1D(edgeCount, blockSize);
  scatterSweptEdgeEdgeSweepPairsKernel<<<gridSize, blockSize>>>(
      startPositions,
      endPositions,
      edgeItems,
      edgeIndices,
      pairOffsets,
      acceptedEdgeAIndices,
      acceptedEdgeBIndices,
      acceptedEndpointSquaredDistances,
      edgeCount);

  return cudaGetLastError();
}

//==============================================================================
cudaError_t launchSweptPointTriangleCandidateBufferKernel(
    const double* startPositions,
    const double* endPositions,
    const std::uint32_t* triangleIndices,
    const std::uint32_t* candidatePointIndices,
    const std::uint32_t* candidateTriangleIndices,
    double* endpointSquaredDistances,
    std::size_t pairCount)
{
  if (pairCount == 0) {
    return cudaSuccess;
  }

  constexpr unsigned int blockSize = 256;
  const unsigned int gridSize = launchGrid1D(pairCount, blockSize);
  evaluateSweptPointTriangleCandidateBufferKernel<<<gridSize, blockSize>>>(
      startPositions,
      endPositions,
      triangleIndices,
      candidatePointIndices,
      candidateTriangleIndices,
      endpointSquaredDistances,
      pairCount);

  return cudaGetLastError();
}

//==============================================================================
cudaError_t launchSweptEdgeEdgeCandidateBufferKernel(
    const double* startPositions,
    const double* endPositions,
    const std::uint32_t* edgeIndices,
    const std::uint32_t* candidateEdgeAIndices,
    const std::uint32_t* candidateEdgeBIndices,
    double* endpointSquaredDistances,
    std::size_t pairCount)
{
  if (pairCount == 0) {
    return cudaSuccess;
  }

  constexpr unsigned int blockSize = 256;
  const unsigned int gridSize = launchGrid1D(pairCount, blockSize);
  evaluateSweptEdgeEdgeCandidateBufferKernel<<<gridSize, blockSize>>>(
      startPositions,
      endPositions,
      edgeIndices,
      candidateEdgeAIndices,
      candidateEdgeBIndices,
      endpointSquaredDistances,
      pairCount);

  return cudaGetLastError();
}

} // namespace dart::simulation::compute::cuda::detail
