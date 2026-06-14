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
  int feature = 0;
};

constexpr int kPointTriangleVertexA = 0;
constexpr int kPointTriangleVertexB = 1;
constexpr int kPointTriangleVertexC = 2;
constexpr int kPointTriangleEdgeAB = 3;
constexpr int kPointTriangleEdgeBC = 4;
constexpr int kPointTriangleEdgeCA = 5;
constexpr int kPointTriangleFace = 6;

struct EdgeEdgeDistanceDevice
{
  double squaredDistance = 0.0;
  double edgeACoordinate = 0.0;
  double edgeBCoordinate = 0.0;
  Vec3 closestPointOnA;
  Vec3 closestPointOnB;
  int feature = 0;
};

constexpr int kEdgeEdgeAStartBStart = 0;
constexpr int kEdgeEdgeAStartBEnd = 1;
constexpr int kEdgeEdgeAEndBStart = 2;
constexpr int kEdgeEdgeAEndBEnd = 3;
constexpr int kEdgeEdgeAInteriorBStart = 4;
constexpr int kEdgeEdgeAInteriorBEnd = 5;
constexpr int kEdgeEdgeAStartBInterior = 6;
constexpr int kEdgeEdgeAEndBInterior = 7;
constexpr int kEdgeEdgeAInteriorBInterior = 8;

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

__device__ double component(const Vec3 value, const int index)
{
  return index == 0 ? value.x : (index == 1 ? value.y : value.z);
}

__device__ double identityEntry(const int row, const int col)
{
  return row == col ? 1.0 : 0.0;
}

__device__ double skewEntry(const Vec3 value, const int row, const int col)
{
  if (row == col) {
    return 0.0;
  }
  if (row == 0 && col == 1) {
    return -value.z;
  }
  if (row == 0 && col == 2) {
    return value.y;
  }
  if (row == 1 && col == 0) {
    return value.z;
  }
  if (row == 1 && col == 2) {
    return -value.x;
  }
  if (row == 2 && col == 0) {
    return -value.y;
  }
  return value.x;
}

__device__ bool isFinite(const Vec3 value)
{
  return isfinite(value.x) && isfinite(value.y) && isfinite(value.z);
}

__device__ bool normalize(const Vec3 value, Vec3& normalizedValue)
{
  constexpr double kTangentBasisEpsilon = 64.0 * 2.2204460492503131e-16;
  if (!isFinite(value)) {
    normalizedValue = {};
    return false;
  }

  const double normSquared = squaredNorm(value);
  if (!(normSquared > kTangentBasisEpsilon)) {
    normalizedValue = {};
    return false;
  }

  normalizedValue = scale(1.0 / sqrt(normSquared), value);
  return true;
}

__device__ Vec3 unitOrthogonal(const Vec3 normal)
{
  if (fabs(normal.x) > fabs(normal.z)) {
    const double invNorm
        = 1.0 / sqrt(normal.x * normal.x + normal.y * normal.y);
    return {-normal.y * invNorm, normal.x * invNorm, 0.0};
  }

  const double invNorm = 1.0 / sqrt(normal.y * normal.y + normal.z * normal.z);
  return {0.0, -normal.z * invNorm, normal.y * invNorm};
}

__device__ void fallbackBasisFromNormal(
    const Vec3 normal, Vec3& basis0, Vec3& basis1)
{
  Vec3 unitNormal;
  if (!normalize(normal, unitNormal)) {
    unitNormal = {0.0, 0.0, 1.0};
  }

  basis0 = unitOrthogonal(unitNormal);
  Vec3 crossed = cross(unitNormal, basis0);
  if (!normalize(crossed, basis1)) {
    basis1 = {0.0, 1.0, 0.0};
  }
}

__device__ std::uint8_t basisFromFirstTangentAndSecondHint(
    const Vec3 firstTangent,
    const Vec3 secondHint,
    const Vec3 fallbackNormal,
    Vec3& basis0,
    Vec3& basis1)
{
  if (normalize(firstTangent, basis0)) {
    const Vec3 orthogonalHint
        = subtract(secondHint, scale(dot(secondHint, basis0), basis0));
    if (normalize(orthogonalHint, basis1)) {
      return 0u;
    }
  }

  fallbackBasisFromNormal(fallbackNormal, basis0, basis1);
  return 1u;
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

__device__ bool isParallelEdgesDevice(
    const double denominator,
    const double edgeASquaredNorm,
    const double edgeBSquaredNorm)
{
  constexpr double kDoubleMin = 2.2250738585072014e-308;
  constexpr double kRelativeEpsilon = 64.0 * 2.2204460492503131e-16;
  const double scale = fmax(edgeASquaredNorm * edgeBSquaredNorm, kDoubleMin);
  return denominator <= kRelativeEpsilon * scale;
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

__device__ void writePointEdgeDistanceGradient(
    const PointEdgeDistanceDevice& distance, const Vec3 p, double* gradient)
{
  const Vec3 residual = subtract(p, distance.closestPoint);
  const double t = distance.edgeCoordinate;
  const Vec3 blocks[3] = {
      scale(2.0, residual),
      scale(-2.0 * (1.0 - t), residual),
      scale(-2.0 * t, residual),
  };
  for (int block = 0; block < 3; ++block) {
    gradient[3 * block] = blocks[block].x;
    gradient[3 * block + 1] = blocks[block].y;
    gradient[3 * block + 2] = blocks[block].z;
  }
}

__device__ void writePointPointDistanceHessianMapped(
    const int blockA, const int blockB, double* hessian)
{
  for (int i = 0; i < 81; ++i) {
    hessian[i] = 0.0;
  }
  for (int componentIndex = 0; componentIndex < 3; ++componentIndex) {
    const int a = 3 * blockA + componentIndex;
    const int b = 3 * blockB + componentIndex;
    hessian[9 * a + a] += 2.0;
    hessian[9 * a + b] -= 2.0;
    hessian[9 * b + a] -= 2.0;
    hessian[9 * b + b] += 2.0;
  }
}

__device__ void writePointEdgeDistanceHessian(
    const Vec3 p,
    const Vec3 a,
    const Vec3 b,
    const PointEdgeDistanceDevice& distance,
    double* hessian)
{
  if (distance.edgeCoordinate <= 0.0) {
    writePointPointDistanceHessianMapped(0, 1, hessian);
    return;
  }
  if (distance.edgeCoordinate >= 1.0) {
    writePointPointDistanceHessianMapped(0, 2, hessian);
    return;
  }

  const Vec3 u = subtract(b, a);
  const Vec3 w = subtract(p, a);
  const double numerator = squaredNorm(cross(u, w));
  const double denominator = squaredNorm(u);
  if (!(denominator > 0.0) || !isfinite(denominator)) {
    writePointPointDistanceHessianMapped(0, 1, hessian);
    return;
  }

  constexpr double coeffU[4] = {0.0, -1.0, 1.0, 0.0};
  constexpr double coeffW[4] = {1.0, -1.0, 0.0, 0.0};
  const double u2 = squaredNorm(u);
  const double w2 = squaredNorm(w);
  const double uw = dot(u, w);
  const Vec3 gradientU = subtract(scale(2.0 * w2, u), scale(2.0 * uw, w));
  const Vec3 gradientW = subtract(scale(2.0 * u2, w), scale(2.0 * uw, u));

  double numeratorGradient[12];
  double denominatorGradient[12];
  for (int block = 0; block < 4; ++block) {
    for (int c = 0; c < 3; ++c) {
      numeratorGradient[3 * block + c]
          = coeffU[block] * component(gradientU, c)
            + coeffW[block] * component(gradientW, c);
      denominatorGradient[3 * block + c]
          = 2.0 * coeffU[block] * component(u, c);
    }
  }

  const double denominatorSquared = denominator * denominator;
  const double denominatorCubed = denominatorSquared * denominator;
  for (int row = 0; row < 9; ++row) {
    const int rowBlock = row / 3;
    const int rowComponent = row % 3;
    for (int col = 0; col < 9; ++col) {
      const int colBlock = col / 3;
      const int colComponent = col % 3;
      const double identity = identityEntry(rowComponent, colComponent);
      const double hUU
          = 2.0 * w2 * identity
            - 2.0 * component(w, rowComponent) * component(w, colComponent);
      const double hWW
          = 2.0 * u2 * identity
            - 2.0 * component(u, rowComponent) * component(u, colComponent);
      const double hUW
          = 4.0 * component(u, rowComponent) * component(w, colComponent)
            - 2.0 * component(w, rowComponent) * component(u, colComponent)
            - 2.0 * uw * identity;
      const double hWU
          = 4.0 * component(u, colComponent) * component(w, rowComponent)
            - 2.0 * component(w, colComponent) * component(u, rowComponent)
            - 2.0 * uw * identity;
      const double numeratorHessian
          = coeffU[rowBlock] * coeffU[colBlock] * hUU
            + coeffU[rowBlock] * coeffW[colBlock] * hUW
            + coeffW[rowBlock] * coeffU[colBlock] * hWU
            + coeffW[rowBlock] * coeffW[colBlock] * hWW;
      const double denominatorHessian
          = 2.0 * coeffU[rowBlock] * coeffU[colBlock] * identity;
      hessian[9 * row + col]
          = numeratorHessian / denominator
            - numerator * denominatorHessian / denominatorSquared
            - (numeratorGradient[row] * denominatorGradient[col]
               + denominatorGradient[row] * numeratorGradient[col])
                  / denominatorSquared
            + 2.0 * numerator * denominatorGradient[row]
                  * denominatorGradient[col] / denominatorCubed;
    }
  }
}

__device__ void writePointPointDistanceHessianMapped12(
    const int blockA, const int blockB, double* hessian)
{
  for (int i = 0; i < 144; ++i) {
    hessian[i] = 0.0;
  }
  for (int componentIndex = 0; componentIndex < 3; ++componentIndex) {
    const int a = 3 * blockA + componentIndex;
    const int b = 3 * blockB + componentIndex;
    hessian[12 * a + a] += 2.0;
    hessian[12 * a + b] -= 2.0;
    hessian[12 * b + a] -= 2.0;
    hessian[12 * b + b] += 2.0;
  }
}

__device__ void writeScatteredThreeBlockHessian(
    const double* source,
    const int block0,
    const int block1,
    const int block2,
    double* hessian)
{
  for (int i = 0; i < 144; ++i) {
    hessian[i] = 0.0;
  }
  const int blocks[3] = {block0, block1, block2};
  for (int localRow = 0; localRow < 9; ++localRow) {
    const int rowBlock = blocks[localRow / 3];
    const int rowComponent = localRow % 3;
    const int row = 3 * rowBlock + rowComponent;
    for (int localCol = 0; localCol < 9; ++localCol) {
      const int colBlock = blocks[localCol / 3];
      const int colComponent = localCol % 3;
      const int col = 3 * colBlock + colComponent;
      hessian[12 * row + col] += source[9 * localRow + localCol];
    }
  }
}

__device__ void writePointTriangleEdgeDistanceHessian(
    const Vec3 p,
    const Vec3 a,
    const Vec3 b,
    const int block0,
    const int block1,
    const int block2,
    double* hessian)
{
  const auto distance = pointEdgeSquaredDistanceDevice(p, a, b);
  double edgeHessian[81];
  writePointEdgeDistanceHessian(p, a, b, distance, edgeHessian);
  writeScatteredThreeBlockHessian(edgeHessian, block0, block1, block2, hessian);
}

__device__ double pointTriangleFaceDenominatorHessianRaw(
    const Vec3 u,
    const Vec3 v,
    const double u2,
    const double v2,
    const double uv,
    const double* coeffU,
    const double* coeffV,
    const int rowBlock,
    const int rowComponent,
    const int colBlock,
    const int colComponent)
{
  const double identity = identityEntry(rowComponent, colComponent);
  const double hUU
      = 2.0 * v2 * identity
        - 2.0 * component(v, rowComponent) * component(v, colComponent);
  const double hVV
      = 2.0 * u2 * identity
        - 2.0 * component(u, rowComponent) * component(u, colComponent);
  const double hUV
      = 4.0 * component(u, rowComponent) * component(v, colComponent)
        - 2.0 * component(v, rowComponent) * component(u, colComponent)
        - 2.0 * uv * identity;
  const double hVU
      = 4.0 * component(u, colComponent) * component(v, rowComponent)
        - 2.0 * component(v, colComponent) * component(u, rowComponent)
        - 2.0 * uv * identity;
  return coeffU[rowBlock] * coeffU[colBlock] * hUU
         + coeffU[rowBlock] * coeffV[colBlock] * hUV
         + coeffV[rowBlock] * coeffU[colBlock] * hVU
         + coeffV[rowBlock] * coeffV[colBlock] * hVV;
}

__device__ double pointTriangleFaceNumeratorHessianRaw(
    const Vec3 u,
    const Vec3 v,
    const Vec3 w,
    const double* coeffU,
    const double* coeffV,
    const double* coeffW,
    const int rowBlock,
    const int rowComponent,
    const int colBlock,
    const int colComponent)
{
  const double hUV = -skewEntry(w, rowComponent, colComponent);
  const double hUW = skewEntry(v, rowComponent, colComponent);
  const double hVU = -skewEntry(w, colComponent, rowComponent);
  const double hVW = -skewEntry(u, rowComponent, colComponent);
  const double hWU = skewEntry(v, colComponent, rowComponent);
  const double hWV = -skewEntry(u, colComponent, rowComponent);
  return coeffU[rowBlock] * coeffV[colBlock] * hUV
         + coeffU[rowBlock] * coeffW[colBlock] * hUW
         + coeffV[rowBlock] * coeffU[colBlock] * hVU
         + coeffV[rowBlock] * coeffW[colBlock] * hVW
         + coeffW[rowBlock] * coeffU[colBlock] * hWU
         + coeffW[rowBlock] * coeffV[colBlock] * hWV;
}

__device__ void writePointTriangleDistanceHessian(
    const Vec3 p,
    const Vec3 a,
    const Vec3 b,
    const Vec3 c,
    const PointTriangleDistanceDevice& distance,
    double* hessian)
{
  if (distance.feature == kPointTriangleVertexA) {
    writePointPointDistanceHessianMapped12(0, 1, hessian);
    return;
  }
  if (distance.feature == kPointTriangleVertexB) {
    writePointPointDistanceHessianMapped12(0, 2, hessian);
    return;
  }
  if (distance.feature == kPointTriangleVertexC) {
    writePointPointDistanceHessianMapped12(0, 3, hessian);
    return;
  }
  if (distance.feature == kPointTriangleEdgeAB) {
    writePointTriangleEdgeDistanceHessian(p, a, b, 0, 1, 2, hessian);
    return;
  }
  if (distance.feature == kPointTriangleEdgeBC) {
    writePointTriangleEdgeDistanceHessian(p, b, c, 0, 2, 3, hessian);
    return;
  }
  if (distance.feature == kPointTriangleEdgeCA) {
    writePointTriangleEdgeDistanceHessian(p, c, a, 0, 3, 1, hessian);
    return;
  }

  const Vec3 u = subtract(b, a);
  const Vec3 v = subtract(c, a);
  const Vec3 w = subtract(p, a);
  const Vec3 uxv = cross(u, v);
  const double numerator = dot(uxv, w);
  const double denominator = squaredNorm(uxv);
  if (!(denominator > 0.0) || !isfinite(denominator)) {
    writePointTriangleEdgeDistanceHessian(p, a, b, 0, 1, 2, hessian);
    return;
  }

  constexpr double coeffU[4] = {0.0, -1.0, 1.0, 0.0};
  constexpr double coeffV[4] = {0.0, -1.0, 0.0, 1.0};
  constexpr double coeffW[4] = {1.0, -1.0, 0.0, 0.0};
  const double u2 = squaredNorm(u);
  const double v2 = squaredNorm(v);
  const double uv = dot(u, v);
  const Vec3 numeratorGradientU = cross(v, w);
  const Vec3 numeratorGradientV = cross(w, u);
  const Vec3 numeratorGradientW = uxv;
  const Vec3 denominatorGradientU
      = subtract(scale(2.0 * v2, u), scale(2.0 * uv, v));
  const Vec3 denominatorGradientV
      = subtract(scale(2.0 * u2, v), scale(2.0 * uv, u));

  double numeratorGradient[12];
  double denominatorGradient[12];
  for (int block = 0; block < 4; ++block) {
    for (int componentIndex = 0; componentIndex < 3; ++componentIndex) {
      numeratorGradient[3 * block + componentIndex]
          = coeffU[block] * component(numeratorGradientU, componentIndex)
            + coeffV[block] * component(numeratorGradientV, componentIndex)
            + coeffW[block] * component(numeratorGradientW, componentIndex);
      denominatorGradient[3 * block + componentIndex]
          = coeffU[block] * component(denominatorGradientU, componentIndex)
            + coeffV[block] * component(denominatorGradientV, componentIndex);
    }
  }

  const double denominatorSquared = denominator * denominator;
  const double denominatorCubed = denominatorSquared * denominator;
  for (int row = 0; row < 12; ++row) {
    const int rowBlock = row / 3;
    const int rowComponent = row % 3;
    for (int col = 0; col < 12; ++col) {
      const int colBlock = col / 3;
      const int colComponent = col % 3;
      const double denominatorHessian
          = 0.5
            * (pointTriangleFaceDenominatorHessianRaw(
                   u,
                   v,
                   u2,
                   v2,
                   uv,
                   coeffU,
                   coeffV,
                   rowBlock,
                   rowComponent,
                   colBlock,
                   colComponent)
               + pointTriangleFaceDenominatorHessianRaw(
                   u,
                   v,
                   u2,
                   v2,
                   uv,
                   coeffU,
                   coeffV,
                   colBlock,
                   colComponent,
                   rowBlock,
                   rowComponent));
      const double numeratorHessian = 0.5
                                      * (pointTriangleFaceNumeratorHessianRaw(
                                             u,
                                             v,
                                             w,
                                             coeffU,
                                             coeffV,
                                             coeffW,
                                             rowBlock,
                                             rowComponent,
                                             colBlock,
                                             colComponent)
                                         + pointTriangleFaceNumeratorHessianRaw(
                                             u,
                                             v,
                                             w,
                                             coeffU,
                                             coeffV,
                                             coeffW,
                                             colBlock,
                                             colComponent,
                                             rowBlock,
                                             rowComponent));
      hessian[12 * row + col]
          = (2.0 * numerator / denominator) * numeratorHessian
            - (numerator * numerator / denominatorSquared) * denominatorHessian
            + (2.0 / denominator) * numeratorGradient[row]
                  * numeratorGradient[col]
            - (2.0 * numerator / denominatorSquared)
                  * (numeratorGradient[row] * denominatorGradient[col]
                     + denominatorGradient[row] * numeratorGradient[col])
            + (2.0 * numerator * numerator / denominatorCubed)
                  * denominatorGradient[row] * denominatorGradient[col];
    }
  }
}

__device__ EdgeEdgeDistanceDevice edgeEdgeSquaredDistanceDevice(
    const Vec3 a, const Vec3 b, const Vec3 c, const Vec3 d)
{
  EdgeEdgeDistanceDevice result;
  const Vec3 edgeA = subtract(b, a);
  const Vec3 edgeB = subtract(d, c);
  const Vec3 r = subtract(a, c);
  const double edgeASquaredNorm = squaredNorm(edgeA);
  const double edgeBSquaredNorm = squaredNorm(edgeB);
  const double edgeBDotR = dot(edgeB, r);

  double s = 0.0;
  double t = 0.0;

  if (isDegenerateSegmentDevice(edgeASquaredNorm)
      && isDegenerateSegmentDevice(edgeBSquaredNorm)) {
    s = 0.0;
    t = 0.0;
  } else if (isDegenerateSegmentDevice(edgeASquaredNorm)) {
    s = 0.0;
    t = fmin(fmax(edgeBDotR / edgeBSquaredNorm, 0.0), 1.0);
  } else {
    const double edgeADotR = dot(edgeA, r);
    if (isDegenerateSegmentDevice(edgeBSquaredNorm)) {
      t = 0.0;
      s = fmin(fmax(-edgeADotR / edgeASquaredNorm, 0.0), 1.0);
    } else {
      const double edgeADotB = dot(edgeA, edgeB);
      const double denominator
          = edgeASquaredNorm * edgeBSquaredNorm - edgeADotB * edgeADotB;
      if (!isParallelEdgesDevice(
              denominator, edgeASquaredNorm, edgeBSquaredNorm)) {
        s = fmin(
            fmax(
                (edgeADotB * edgeBDotR - edgeADotR * edgeBSquaredNorm)
                    / denominator,
                0.0),
            1.0);
      } else {
        s = 0.0;
      }

      t = (edgeADotB * s + edgeBDotR) / edgeBSquaredNorm;
      if (t < 0.0) {
        t = 0.0;
        s = fmin(fmax(-edgeADotR / edgeASquaredNorm, 0.0), 1.0);
      } else if (t > 1.0) {
        t = 1.0;
        s = fmin(fmax((edgeADotB - edgeADotR) / edgeASquaredNorm, 0.0), 1.0);
      }
    }
  }

  result.edgeACoordinate = s;
  result.edgeBCoordinate = t;
  result.closestPointOnA = add(a, scale(s, edgeA));
  result.closestPointOnB = add(c, scale(t, edgeB));
  result.squaredDistance
      = squaredNorm(subtract(result.closestPointOnA, result.closestPointOnB));

  const bool aStart = s <= 0.0;
  const bool aEnd = s >= 1.0;
  const bool bStart = t <= 0.0;
  const bool bEnd = t >= 1.0;
  if (aStart && bStart) {
    result.feature = kEdgeEdgeAStartBStart;
  } else if (aStart && bEnd) {
    result.feature = kEdgeEdgeAStartBEnd;
  } else if (aEnd && bStart) {
    result.feature = kEdgeEdgeAEndBStart;
  } else if (aEnd && bEnd) {
    result.feature = kEdgeEdgeAEndBEnd;
  } else if (bStart) {
    result.feature = kEdgeEdgeAInteriorBStart;
  } else if (bEnd) {
    result.feature = kEdgeEdgeAInteriorBEnd;
  } else if (aStart) {
    result.feature = kEdgeEdgeAStartBInterior;
  } else if (aEnd) {
    result.feature = kEdgeEdgeAEndBInterior;
  } else {
    result.feature = kEdgeEdgeAInteriorBInterior;
  }
  return result;
}

__device__ void writeEdgeEdgeDistanceGradient(
    const EdgeEdgeDistanceDevice& distance, double* gradient)
{
  const Vec3 residual
      = subtract(distance.closestPointOnA, distance.closestPointOnB);
  const double s = distance.edgeACoordinate;
  const double t = distance.edgeBCoordinate;

  const Vec3 a0 = scale(2.0 * (1.0 - s), residual);
  const Vec3 a1 = scale(2.0 * s, residual);
  const Vec3 b0 = scale(-2.0 * (1.0 - t), residual);
  const Vec3 b1 = scale(-2.0 * t, residual);
  const Vec3 blocks[4] = {a0, a1, b0, b1};
  for (int block = 0; block < 4; ++block) {
    for (int componentIndex = 0; componentIndex < 3; ++componentIndex) {
      gradient[3 * block + componentIndex]
          = component(blocks[block], componentIndex);
    }
  }
}

__device__ void writeEdgeEdgeDistanceHessian(
    const Vec3 a,
    const Vec3 b,
    const Vec3 c,
    const Vec3 d,
    const EdgeEdgeDistanceDevice& distance,
    double* hessian)
{
  if (distance.feature == kEdgeEdgeAStartBStart) {
    writePointPointDistanceHessianMapped12(0, 2, hessian);
    return;
  }
  if (distance.feature == kEdgeEdgeAStartBEnd) {
    writePointPointDistanceHessianMapped12(0, 3, hessian);
    return;
  }
  if (distance.feature == kEdgeEdgeAEndBStart) {
    writePointPointDistanceHessianMapped12(1, 2, hessian);
    return;
  }
  if (distance.feature == kEdgeEdgeAEndBEnd) {
    writePointPointDistanceHessianMapped12(1, 3, hessian);
    return;
  }
  if (distance.feature == kEdgeEdgeAInteriorBStart) {
    writePointTriangleEdgeDistanceHessian(c, a, b, 2, 0, 1, hessian);
    return;
  }
  if (distance.feature == kEdgeEdgeAInteriorBEnd) {
    writePointTriangleEdgeDistanceHessian(d, a, b, 3, 0, 1, hessian);
    return;
  }
  if (distance.feature == kEdgeEdgeAStartBInterior) {
    writePointTriangleEdgeDistanceHessian(a, c, d, 0, 2, 3, hessian);
    return;
  }
  if (distance.feature == kEdgeEdgeAEndBInterior) {
    writePointTriangleEdgeDistanceHessian(b, c, d, 1, 2, 3, hessian);
    return;
  }

  const Vec3 u = subtract(b, a);
  const Vec3 v = subtract(d, c);
  const Vec3 w = subtract(a, c);
  const Vec3 uxv = cross(u, v);
  const double numerator = dot(uxv, w);
  const double denominator = squaredNorm(uxv);
  if (!(denominator > 0.0) || !isfinite(denominator)) {
    writePointPointDistanceHessianMapped12(0, 2, hessian);
    return;
  }

  constexpr double coeffU[4] = {-1.0, 1.0, 0.0, 0.0};
  constexpr double coeffV[4] = {0.0, 0.0, -1.0, 1.0};
  constexpr double coeffW[4] = {1.0, 0.0, -1.0, 0.0};
  const double u2 = squaredNorm(u);
  const double v2 = squaredNorm(v);
  const double uv = dot(u, v);
  const Vec3 numeratorGradientU = cross(v, w);
  const Vec3 numeratorGradientV = cross(w, u);
  const Vec3 numeratorGradientW = uxv;
  const Vec3 denominatorGradientU
      = subtract(scale(2.0 * v2, u), scale(2.0 * uv, v));
  const Vec3 denominatorGradientV
      = subtract(scale(2.0 * u2, v), scale(2.0 * uv, u));

  double numeratorGradient[12];
  double denominatorGradient[12];
  for (int block = 0; block < 4; ++block) {
    for (int componentIndex = 0; componentIndex < 3; ++componentIndex) {
      numeratorGradient[3 * block + componentIndex]
          = coeffU[block] * component(numeratorGradientU, componentIndex)
            + coeffV[block] * component(numeratorGradientV, componentIndex)
            + coeffW[block] * component(numeratorGradientW, componentIndex);
      denominatorGradient[3 * block + componentIndex]
          = coeffU[block] * component(denominatorGradientU, componentIndex)
            + coeffV[block] * component(denominatorGradientV, componentIndex);
    }
  }

  const double denominatorSquared = denominator * denominator;
  const double denominatorCubed = denominatorSquared * denominator;
  for (int row = 0; row < 12; ++row) {
    const int rowBlock = row / 3;
    const int rowComponent = row % 3;
    for (int col = 0; col < 12; ++col) {
      const int colBlock = col / 3;
      const int colComponent = col % 3;
      const double denominatorHessian
          = 0.5
            * (pointTriangleFaceDenominatorHessianRaw(
                   u,
                   v,
                   u2,
                   v2,
                   uv,
                   coeffU,
                   coeffV,
                   rowBlock,
                   rowComponent,
                   colBlock,
                   colComponent)
               + pointTriangleFaceDenominatorHessianRaw(
                   u,
                   v,
                   u2,
                   v2,
                   uv,
                   coeffU,
                   coeffV,
                   colBlock,
                   colComponent,
                   rowBlock,
                   rowComponent));
      const double numeratorHessian = 0.5
                                      * (pointTriangleFaceNumeratorHessianRaw(
                                             u,
                                             v,
                                             w,
                                             coeffU,
                                             coeffV,
                                             coeffW,
                                             rowBlock,
                                             rowComponent,
                                             colBlock,
                                             colComponent)
                                         + pointTriangleFaceNumeratorHessianRaw(
                                             u,
                                             v,
                                             w,
                                             coeffU,
                                             coeffV,
                                             coeffW,
                                             colBlock,
                                             colComponent,
                                             rowBlock,
                                             rowComponent));
      hessian[12 * row + col]
          = (2.0 * numerator / denominator) * numeratorHessian
            - (numerator * numerator / denominatorSquared) * denominatorHessian
            + (2.0 / denominator) * numeratorGradient[row]
                  * numeratorGradient[col]
            - (2.0 * numerator / denominatorSquared)
                  * (numeratorGradient[row] * denominatorGradient[col]
                     + denominatorGradient[row] * numeratorGradient[col])
            + (2.0 * numerator * numerator / denominatorCubed)
                  * denominatorGradient[row] * denominatorGradient[col];
    }
  }
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
    result.feature = kPointTriangleEdgeAB;

    if (bcDistance.squaredDistance < result.squaredDistance) {
      result.squaredDistance = bcDistance.squaredDistance;
      result.closestPoint = bcDistance.closestPoint;
      result.barycentric[0] = 0.0;
      result.barycentric[1] = 1.0 - bcDistance.edgeCoordinate;
      result.barycentric[2] = bcDistance.edgeCoordinate;
      result.feature = kPointTriangleEdgeBC;
    }
    if (caDistance.squaredDistance < result.squaredDistance) {
      result.squaredDistance = caDistance.squaredDistance;
      result.closestPoint = caDistance.closestPoint;
      result.barycentric[0] = caDistance.edgeCoordinate;
      result.barycentric[1] = 0.0;
      result.barycentric[2] = 1.0 - caDistance.edgeCoordinate;
      result.feature = kPointTriangleEdgeCA;
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
    result.feature = kPointTriangleVertexA;
  } else {
    const Vec3 bp = subtract(p, b);
    const double d3 = dot(ab, bp);
    const double d4 = dot(ac, bp);
    if (d3 >= 0.0 && d4 <= d3) {
      result.barycentric[0] = 0.0;
      result.barycentric[1] = 1.0;
      result.barycentric[2] = 0.0;
      result.closestPoint = b;
      result.feature = kPointTriangleVertexB;
    } else {
      const double vc = d1 * d4 - d3 * d2;
      if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
        const double v = d1 / (d1 - d3);
        result.barycentric[0] = 1.0 - v;
        result.barycentric[1] = v;
        result.barycentric[2] = 0.0;
        result.closestPoint = add(a, scale(v, ab));
        result.feature = kPointTriangleEdgeAB;
      } else {
        const Vec3 cp = subtract(p, c);
        const double d5 = dot(ab, cp);
        const double d6 = dot(ac, cp);
        if (d6 >= 0.0 && d5 <= d6) {
          result.barycentric[0] = 0.0;
          result.barycentric[1] = 0.0;
          result.barycentric[2] = 1.0;
          result.closestPoint = c;
          result.feature = kPointTriangleVertexC;
        } else {
          const double vb = d5 * d2 - d1 * d6;
          if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
            const double w = d2 / (d2 - d6);
            result.barycentric[0] = 1.0 - w;
            result.barycentric[1] = 0.0;
            result.barycentric[2] = w;
            result.closestPoint = add(a, scale(w, ac));
            result.feature = kPointTriangleEdgeCA;
          } else {
            const double va = d3 * d6 - d5 * d4;
            if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0) {
              const double w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
              result.barycentric[0] = 0.0;
              result.barycentric[1] = 1.0 - w;
              result.barycentric[2] = w;
              result.closestPoint = add(b, scale(w, subtract(c, b)));
              result.feature = kPointTriangleEdgeBC;
            } else {
              const double denominator = 1.0 / (va + vb + vc);
              const double v = vb * denominator;
              const double w = vc * denominator;
              result.barycentric[0] = 1.0 - v - w;
              result.barycentric[1] = v;
              result.barycentric[2] = w;
              result.closestPoint = add(a, add(scale(v, ab), scale(w, ac)));
              result.feature = kPointTriangleFace;
            }
          }
        }
      }
    }
  }

  result.squaredDistance = squaredNorm(subtract(p, result.closestPoint));
  return result;
}

__device__ void pointTriangleCoordinatesDevice(
    const Vec3 p,
    const Vec3 a,
    const Vec3 b,
    const Vec3 c,
    double& beta1,
    double& beta2)
{
  constexpr double kTangentBasisEpsilon = 64.0 * 2.2204460492503131e-16;
  const Vec3 ab = subtract(b, a);
  const Vec3 ac = subtract(c, a);
  const double aa = dot(ab, ab);
  const double acDot = dot(ab, ac);
  const double cc = dot(ac, ac);
  const double determinant = aa * cc - acDot * acDot;
  if (isfinite(determinant) && fabs(determinant) > kTangentBasisEpsilon) {
    const Vec3 ap = subtract(p, a);
    const double rhs0 = dot(ap, ab);
    const double rhs1 = dot(ap, ac);
    beta1 = (cc * rhs0 - acDot * rhs1) / determinant;
    beta2 = (aa * rhs1 - acDot * rhs0) / determinant;
    return;
  }

  const auto distance = pointTriangleSquaredDistanceDevice(p, a, b, c);
  beta1 = distance.barycentric[1];
  beta2 = distance.barycentric[2];
}

__device__ void edgeEdgeCoordinatesDevice(
    const Vec3 a,
    const Vec3 b,
    const Vec3 c,
    const Vec3 d,
    double& gamma1,
    double& gamma2)
{
  constexpr double kTangentBasisEpsilon = 64.0 * 2.2204460492503131e-16;
  const Vec3 e20 = subtract(a, c);
  const Vec3 e01 = subtract(b, a);
  const Vec3 e23 = subtract(d, c);
  const double aa = dot(e01, e01);
  const double edgeDot = dot(e23, e01);
  const double bb = dot(e23, e23);
  const double determinant = aa * bb - edgeDot * edgeDot;
  if (isfinite(determinant) && fabs(determinant) > kTangentBasisEpsilon) {
    const double rhs0 = -dot(e20, e01);
    const double rhs1 = dot(e20, e23);
    gamma1 = (bb * rhs0 + edgeDot * rhs1) / determinant;
    gamma2 = (edgeDot * rhs0 + aa * rhs1) / determinant;
    return;
  }

  const auto distance = edgeEdgeSquaredDistanceDevice(a, b, c, d);
  gamma1 = distance.edgeACoordinate;
  gamma2 = distance.edgeBCoordinate;
}

__device__ double pointEdgeCoordinateDevice(
    const Vec3 p, const Vec3 a, const Vec3 b)
{
  constexpr double kTangentBasisEpsilon = 64.0 * 2.2204460492503131e-16;
  const Vec3 ab = subtract(b, a);
  const double squaredLength = squaredNorm(ab);
  if (isfinite(squaredLength) && squaredLength > kTangentBasisEpsilon) {
    return dot(subtract(p, a), ab) / squaredLength;
  }

  return 0.0;
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

__device__ void writePointTriangleTangentStencil(
    const PointTriangleTangentInput& input,
    double* basisValues,
    double* coordinates,
    double* projectionValues,
    std::uint8_t& fallbackBasis)
{
  const Vec3 p = makeVec3(input.point);
  const Vec3 a = makeVec3(input.triangleA);
  const Vec3 b = makeVec3(input.triangleB);
  const Vec3 c = makeVec3(input.triangleC);
  const Vec3 ab = subtract(b, a);
  const Vec3 ac = subtract(c, a);
  const Vec3 triangleNormal = cross(ab, ac);

  Vec3 basis0;
  Vec3 basis1;
  fallbackBasis = basisFromFirstTangentAndSecondHint(
      ab, cross(triangleNormal, ab), subtract(p, a), basis0, basis1);
  basisValues[0] = basis0.x;
  basisValues[1] = basis0.y;
  basisValues[2] = basis0.z;
  basisValues[3] = basis1.x;
  basisValues[4] = basis1.y;
  basisValues[5] = basis1.z;

  double beta1 = 0.0;
  double beta2 = 0.0;
  pointTriangleCoordinatesDevice(p, a, b, c, beta1, beta2);
  coordinates[0] = beta1;
  coordinates[1] = beta2;

  const double coefficients[4] = {1.0, -1.0 + beta1 + beta2, -beta1, -beta2};
  for (int block = 0; block < 4; ++block) {
    const double coefficient = coefficients[block];
    const int offset = 6 * block;
    projectionValues[offset] = coefficient * basis0.x;
    projectionValues[offset + 1] = coefficient * basis0.y;
    projectionValues[offset + 2] = coefficient * basis0.z;
    projectionValues[offset + 3] = coefficient * basis1.x;
    projectionValues[offset + 4] = coefficient * basis1.y;
    projectionValues[offset + 5] = coefficient * basis1.z;
  }
}

__device__ void writePointEdgeTangentStencil(
    const PointEdgeTangentInput& input,
    double* basisValues,
    double* coordinates,
    double* projectionValues,
    std::uint8_t& fallbackBasis)
{
  const Vec3 p = makeVec3(input.point);
  const Vec3 a = makeVec3(input.edgeA);
  const Vec3 b = makeVec3(input.edgeB);
  const Vec3 ab = subtract(b, a);
  const Vec3 pa = subtract(p, a);

  Vec3 basis0;
  Vec3 basis1;
  fallbackBasis = basisFromFirstTangentAndSecondHint(
      ab, cross(ab, pa), pa, basis0, basis1);
  basisValues[0] = basis0.x;
  basisValues[1] = basis0.y;
  basisValues[2] = basis0.z;
  basisValues[3] = basis1.x;
  basisValues[4] = basis1.y;
  basisValues[5] = basis1.z;

  const double eta = pointEdgeCoordinateDevice(p, a, b);
  coordinates[0] = eta;

  const double coefficients[3] = {1.0, eta - 1.0, -eta};
  for (int block = 0; block < 3; ++block) {
    const double coefficient = coefficients[block];
    const int offset = 6 * block;
    projectionValues[offset] = coefficient * basis0.x;
    projectionValues[offset + 1] = coefficient * basis0.y;
    projectionValues[offset + 2] = coefficient * basis0.z;
    projectionValues[offset + 3] = coefficient * basis1.x;
    projectionValues[offset + 4] = coefficient * basis1.y;
    projectionValues[offset + 5] = coefficient * basis1.z;
  }
}

__device__ void writePointPointTangentStencil(
    const PointPointTangentInput& input,
    double* basisValues,
    double* projectionValues,
    std::uint8_t& fallbackBasis)
{
  const Vec3 a = makeVec3(input.pointA);
  const Vec3 b = makeVec3(input.pointB);
  const Vec3 ab = subtract(b, a);
  const Vec3 unitX{1.0, 0.0, 0.0};
  const Vec3 unitY{0.0, 1.0, 0.0};
  const Vec3 xCross = cross(unitX, ab);
  const Vec3 yCross = cross(unitY, ab);
  const Vec3 firstTangent = fabs(ab.y) > fabs(ab.x) ? xCross : yCross;

  Vec3 basis0;
  Vec3 basis1;
  fallbackBasis = basisFromFirstTangentAndSecondHint(
      firstTangent, cross(ab, firstTangent), ab, basis0, basis1);
  basisValues[0] = basis0.x;
  basisValues[1] = basis0.y;
  basisValues[2] = basis0.z;
  basisValues[3] = basis1.x;
  basisValues[4] = basis1.y;
  basisValues[5] = basis1.z;

  const double coefficients[2] = {1.0, -1.0};
  for (int block = 0; block < 2; ++block) {
    const double coefficient = coefficients[block];
    const int offset = 6 * block;
    projectionValues[offset] = coefficient * basis0.x;
    projectionValues[offset + 1] = coefficient * basis0.y;
    projectionValues[offset + 2] = coefficient * basis0.z;
    projectionValues[offset + 3] = coefficient * basis1.x;
    projectionValues[offset + 4] = coefficient * basis1.y;
    projectionValues[offset + 5] = coefficient * basis1.z;
  }
}

__device__ void writeEdgeEdgeTangentStencil(
    const EdgeEdgeTangentInput& input,
    double* basisValues,
    double* coordinates,
    double* projectionValues,
    std::uint8_t& fallbackBasis)
{
  const Vec3 a = makeVec3(input.edgeA0);
  const Vec3 b = makeVec3(input.edgeA1);
  const Vec3 c = makeVec3(input.edgeB0);
  const Vec3 d = makeVec3(input.edgeB1);
  const Vec3 ab = subtract(b, a);
  const Vec3 cd = subtract(d, c);
  const Vec3 ac = subtract(c, a);
  const Vec3 normal = cross(ab, cd);

  Vec3 abUnit;
  Vec3 fallbackHint = ac;
  if (normalize(ab, abUnit)) {
    fallbackHint = subtract(ac, scale(dot(ac, abUnit), abUnit));
  }

  Vec3 basis0;
  Vec3 basis1;
  fallbackBasis = basisFromFirstTangentAndSecondHint(
      ab, cross(normal, ab), fallbackHint, basis0, basis1);
  basisValues[0] = basis0.x;
  basisValues[1] = basis0.y;
  basisValues[2] = basis0.z;
  basisValues[3] = basis1.x;
  basisValues[4] = basis1.y;
  basisValues[5] = basis1.z;

  double gamma1 = 0.0;
  double gamma2 = 0.0;
  edgeEdgeCoordinatesDevice(a, b, c, d, gamma1, gamma2);
  coordinates[0] = gamma1;
  coordinates[1] = gamma2;

  const double coefficients[4] = {1.0 - gamma1, gamma1, gamma2 - 1.0, -gamma2};
  for (int block = 0; block < 4; ++block) {
    const double coefficient = coefficients[block];
    const int offset = 6 * block;
    projectionValues[offset] = coefficient * basis0.x;
    projectionValues[offset + 1] = coefficient * basis0.y;
    projectionValues[offset + 2] = coefficient * basis0.z;
    projectionValues[offset + 3] = coefficient * basis1.x;
    projectionValues[offset + 4] = coefficient * basis1.y;
    projectionValues[offset + 5] = coefficient * basis1.z;
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

__global__ void pointTriangleBarrierHessianKernel(
    const PointTriangleBarrierInput* inputs,
    double* squaredDistances,
    double* barrierValues,
    double* barrierGradients,
    double* barrierHessians,
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

  const std::size_t gradientOffset = 12 * index;
  const std::size_t hessianOffset = 144 * index;
  double distanceGradient[12];
  double distanceHessian[144];
  for (double& entry : distanceGradient) {
    entry = 0.0;
  }
  for (double& entry : distanceHessian) {
    entry = 0.0;
  }
  for (int entry = 0; entry < 12; ++entry) {
    barrierGradients[gradientOffset + static_cast<std::size_t>(entry)] = 0.0;
  }
  for (int entry = 0; entry < 144; ++entry) {
    barrierHessians[hessianOffset + static_cast<std::size_t>(entry)] = 0.0;
  }

  const PointTriangleBarrierInput input = inputs[index];
  const Vec3 p = makeVec3(input.point);
  const Vec3 a = makeVec3(input.triangleA);
  const Vec3 b = makeVec3(input.triangleB);
  const Vec3 c = makeVec3(input.triangleC);
  const auto distance = pointTriangleSquaredDistanceDevice(p, a, b, c);
  squaredDistances[index] = distance.squaredDistance;
  writePointTriangleDistanceGradient(distance, p, distanceGradient);
  writePointTriangleDistanceHessian(p, a, b, c, distance, distanceHessian);

  const double stiffness
      = isfinite(input.stiffness) ? fmax(0.0, input.stiffness) : 0.0;
  const BarrierScalar barrier = c2ClampedLogBarrierDevice(
      distance.squaredDistance, input.squaredActivationDistance);
  if (barrier.active == 0u || !(stiffness > 0.0)) {
    return;
  }

  activeBarriers[index] = 1u;
  barrierValues[index] = stiffness * barrier.value;
  for (int entry = 0; entry < 12; ++entry) {
    barrierGradients[gradientOffset + static_cast<std::size_t>(entry)]
        = stiffness * barrier.firstDerivative * distanceGradient[entry];
  }

  for (int row = 0; row < 12; ++row) {
    for (int col = 0; col < 12; ++col) {
      barrierHessians[hessianOffset + static_cast<std::size_t>(12 * row + col)]
          = stiffness
            * (barrier.secondDerivative * distanceGradient[row]
                   * distanceGradient[col]
               + barrier.firstDerivative * distanceHessian[12 * row + col]);
    }
  }
}

__global__ void pointPointBarrierHessianKernel(
    const PointPointBarrierInput* inputs,
    double* squaredDistances,
    double* barrierValues,
    double* barrierGradients,
    double* barrierHessians,
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

  const std::size_t gradientOffset = 6 * index;
  const std::size_t hessianOffset = 36 * index;
  double distanceGradient[6];
  for (double& entry : distanceGradient) {
    entry = 0.0;
  }
  for (int entry = 0; entry < 6; ++entry) {
    barrierGradients[gradientOffset + static_cast<std::size_t>(entry)] = 0.0;
  }
  for (int entry = 0; entry < 36; ++entry) {
    barrierHessians[hessianOffset + static_cast<std::size_t>(entry)] = 0.0;
  }

  const PointPointBarrierInput input = inputs[index];
  const Vec3 a = makeVec3(input.pointA);
  const Vec3 b = makeVec3(input.pointB);
  const Vec3 residual = subtract(a, b);
  const double squaredDistance = squaredNorm(residual);
  squaredDistances[index] = squaredDistance;

  distanceGradient[0] = 2.0 * residual.x;
  distanceGradient[1] = 2.0 * residual.y;
  distanceGradient[2] = 2.0 * residual.z;
  distanceGradient[3] = -2.0 * residual.x;
  distanceGradient[4] = -2.0 * residual.y;
  distanceGradient[5] = -2.0 * residual.z;

  const double stiffness
      = isfinite(input.stiffness) ? fmax(0.0, input.stiffness) : 0.0;
  const BarrierScalar barrier = c2ClampedLogBarrierDevice(
      squaredDistance, input.squaredActivationDistance);
  if (barrier.active == 0u || !(stiffness > 0.0)) {
    return;
  }

  activeBarriers[index] = 1u;
  barrierValues[index] = stiffness * barrier.value;
  for (int entry = 0; entry < 6; ++entry) {
    barrierGradients[gradientOffset + static_cast<std::size_t>(entry)]
        = stiffness * barrier.firstDerivative * distanceGradient[entry];
  }

  for (int row = 0; row < 6; ++row) {
    for (int col = 0; col < 6; ++col) {
      const int rowPoint = row < 3 ? 0 : 1;
      const int colPoint = col < 3 ? 0 : 1;
      const int rowComponent = row % 3;
      const int colComponent = col % 3;
      const double distanceHessian = rowComponent == colComponent
                                         ? (rowPoint == colPoint ? 2.0 : -2.0)
                                         : 0.0;
      barrierHessians[hessianOffset + static_cast<std::size_t>(6 * row + col)]
          = stiffness
            * (barrier.secondDerivative * distanceGradient[row]
                   * distanceGradient[col]
               + barrier.firstDerivative * distanceHessian);
    }
  }
}

__global__ void pointEdgeBarrierHessianKernel(
    const PointEdgeBarrierInput* inputs,
    double* squaredDistances,
    double* barrierValues,
    double* barrierGradients,
    double* barrierHessians,
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

  const std::size_t gradientOffset = 9 * index;
  const std::size_t hessianOffset = 81 * index;
  double distanceGradient[9];
  double distanceHessian[81];
  for (double& entry : distanceGradient) {
    entry = 0.0;
  }
  for (double& entry : distanceHessian) {
    entry = 0.0;
  }
  for (int entry = 0; entry < 9; ++entry) {
    barrierGradients[gradientOffset + static_cast<std::size_t>(entry)] = 0.0;
  }
  for (int entry = 0; entry < 81; ++entry) {
    barrierHessians[hessianOffset + static_cast<std::size_t>(entry)] = 0.0;
  }

  const PointEdgeBarrierInput input = inputs[index];
  const Vec3 p = makeVec3(input.point);
  const Vec3 a = makeVec3(input.edgeA);
  const Vec3 b = makeVec3(input.edgeB);
  const auto distance = pointEdgeSquaredDistanceDevice(p, a, b);
  squaredDistances[index] = distance.squaredDistance;
  writePointEdgeDistanceGradient(distance, p, distanceGradient);
  writePointEdgeDistanceHessian(p, a, b, distance, distanceHessian);

  const double stiffness
      = isfinite(input.stiffness) ? fmax(0.0, input.stiffness) : 0.0;
  const BarrierScalar barrier = c2ClampedLogBarrierDevice(
      distance.squaredDistance, input.squaredActivationDistance);
  if (barrier.active == 0u || !(stiffness > 0.0)) {
    return;
  }

  activeBarriers[index] = 1u;
  barrierValues[index] = stiffness * barrier.value;
  for (int entry = 0; entry < 9; ++entry) {
    barrierGradients[gradientOffset + static_cast<std::size_t>(entry)]
        = stiffness * barrier.firstDerivative * distanceGradient[entry];
  }

  for (int row = 0; row < 9; ++row) {
    for (int col = 0; col < 9; ++col) {
      barrierHessians[hessianOffset + static_cast<std::size_t>(9 * row + col)]
          = stiffness
            * (barrier.secondDerivative * distanceGradient[row]
                   * distanceGradient[col]
               + barrier.firstDerivative * distanceHessian[9 * row + col]);
    }
  }
}

__global__ void edgeEdgeBarrierHessianKernel(
    const EdgeEdgeBarrierInput* inputs,
    double* squaredDistances,
    double* barrierValues,
    double* barrierGradients,
    double* barrierHessians,
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

  const std::size_t gradientOffset = 12 * index;
  const std::size_t hessianOffset = 144 * index;
  double distanceGradient[12];
  double distanceHessian[144];
  for (double& entry : distanceGradient) {
    entry = 0.0;
  }
  for (double& entry : distanceHessian) {
    entry = 0.0;
  }
  for (int entry = 0; entry < 12; ++entry) {
    barrierGradients[gradientOffset + static_cast<std::size_t>(entry)] = 0.0;
  }
  for (int entry = 0; entry < 144; ++entry) {
    barrierHessians[hessianOffset + static_cast<std::size_t>(entry)] = 0.0;
  }

  const EdgeEdgeBarrierInput input = inputs[index];
  const Vec3 a = makeVec3(input.edgeA0);
  const Vec3 b = makeVec3(input.edgeA1);
  const Vec3 c = makeVec3(input.edgeB0);
  const Vec3 d = makeVec3(input.edgeB1);
  const auto distance = edgeEdgeSquaredDistanceDevice(a, b, c, d);
  squaredDistances[index] = distance.squaredDistance;
  writeEdgeEdgeDistanceGradient(distance, distanceGradient);
  writeEdgeEdgeDistanceHessian(a, b, c, d, distance, distanceHessian);

  const double stiffness
      = isfinite(input.stiffness) ? fmax(0.0, input.stiffness) : 0.0;
  const BarrierScalar barrier = c2ClampedLogBarrierDevice(
      distance.squaredDistance, input.squaredActivationDistance);
  if (barrier.active == 0u || !(stiffness > 0.0)) {
    return;
  }

  activeBarriers[index] = 1u;
  barrierValues[index] = stiffness * barrier.value;
  for (int entry = 0; entry < 12; ++entry) {
    barrierGradients[gradientOffset + static_cast<std::size_t>(entry)]
        = stiffness * barrier.firstDerivative * distanceGradient[entry];
  }

  for (int row = 0; row < 12; ++row) {
    for (int col = 0; col < 12; ++col) {
      barrierHessians[hessianOffset + static_cast<std::size_t>(12 * row + col)]
          = stiffness
            * (barrier.secondDerivative * distanceGradient[row]
                   * distanceGradient[col]
               + barrier.firstDerivative * distanceHessian[12 * row + col]);
    }
  }
}

__global__ void pointTriangleTangentStencilKernel(
    const PointTriangleTangentInput* inputs,
    double* basisValues,
    double* coordinates,
    double* projectionValues,
    std::uint8_t* fallbackBases,
    const std::size_t inputCount)
{
  const auto index
      = static_cast<std::size_t>(blockIdx.x * blockDim.x + threadIdx.x);
  if (index >= inputCount) {
    return;
  }

  const std::size_t basisOffset = 6 * index;
  const std::size_t coordinateOffset = 2 * index;
  const std::size_t projectionOffset = 24 * index;
  for (int entry = 0; entry < 6; ++entry) {
    basisValues[basisOffset + static_cast<std::size_t>(entry)] = 0.0;
  }
  coordinates[coordinateOffset] = 0.0;
  coordinates[coordinateOffset + 1] = 0.0;
  for (int entry = 0; entry < 24; ++entry) {
    projectionValues[projectionOffset + static_cast<std::size_t>(entry)] = 0.0;
  }
  fallbackBases[index] = 0u;

  writePointTriangleTangentStencil(
      inputs[index],
      basisValues + basisOffset,
      coordinates + coordinateOffset,
      projectionValues + projectionOffset,
      fallbackBases[index]);
}

__global__ void edgeEdgeTangentStencilKernel(
    const EdgeEdgeTangentInput* inputs,
    double* basisValues,
    double* coordinates,
    double* projectionValues,
    std::uint8_t* fallbackBases,
    const std::size_t inputCount)
{
  const auto index
      = static_cast<std::size_t>(blockIdx.x * blockDim.x + threadIdx.x);
  if (index >= inputCount) {
    return;
  }

  const std::size_t basisOffset = 6 * index;
  const std::size_t coordinateOffset = 2 * index;
  const std::size_t projectionOffset = 24 * index;
  for (int entry = 0; entry < 6; ++entry) {
    basisValues[basisOffset + static_cast<std::size_t>(entry)] = 0.0;
  }
  coordinates[coordinateOffset] = 0.0;
  coordinates[coordinateOffset + 1] = 0.0;
  for (int entry = 0; entry < 24; ++entry) {
    projectionValues[projectionOffset + static_cast<std::size_t>(entry)] = 0.0;
  }
  fallbackBases[index] = 0u;

  writeEdgeEdgeTangentStencil(
      inputs[index],
      basisValues + basisOffset,
      coordinates + coordinateOffset,
      projectionValues + projectionOffset,
      fallbackBases[index]);
}

__global__ void pointEdgeTangentStencilKernel(
    const PointEdgeTangentInput* inputs,
    double* basisValues,
    double* coordinates,
    double* projectionValues,
    std::uint8_t* fallbackBases,
    const std::size_t inputCount)
{
  const auto index
      = static_cast<std::size_t>(blockIdx.x * blockDim.x + threadIdx.x);
  if (index >= inputCount) {
    return;
  }

  const std::size_t basisOffset = 6 * index;
  const std::size_t projectionOffset = 18 * index;
  for (int entry = 0; entry < 6; ++entry) {
    basisValues[basisOffset + static_cast<std::size_t>(entry)] = 0.0;
  }
  coordinates[index] = 0.0;
  for (int entry = 0; entry < 18; ++entry) {
    projectionValues[projectionOffset + static_cast<std::size_t>(entry)] = 0.0;
  }
  fallbackBases[index] = 0u;

  writePointEdgeTangentStencil(
      inputs[index],
      basisValues + basisOffset,
      coordinates + index,
      projectionValues + projectionOffset,
      fallbackBases[index]);
}

__global__ void pointPointTangentStencilKernel(
    const PointPointTangentInput* inputs,
    double* basisValues,
    double* projectionValues,
    std::uint8_t* fallbackBases,
    const std::size_t inputCount)
{
  const auto index
      = static_cast<std::size_t>(blockIdx.x * blockDim.x + threadIdx.x);
  if (index >= inputCount) {
    return;
  }

  const std::size_t basisOffset = 6 * index;
  const std::size_t projectionOffset = 12 * index;
  for (int entry = 0; entry < 6; ++entry) {
    basisValues[basisOffset + static_cast<std::size_t>(entry)] = 0.0;
  }
  for (int entry = 0; entry < 12; ++entry) {
    projectionValues[projectionOffset + static_cast<std::size_t>(entry)] = 0.0;
  }
  fallbackBases[index] = 0u;

  writePointPointTangentStencil(
      inputs[index],
      basisValues + basisOffset,
      projectionValues + projectionOffset,
      fallbackBases[index]);
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

//==============================================================================
cudaError_t launchPointTriangleBarrierHessianKernel(
    const PointTriangleBarrierInput* inputs,
    double* squaredDistances,
    double* barrierValues,
    double* barrierGradients,
    double* barrierHessians,
    std::uint8_t* activeBarriers,
    std::size_t inputCount)
{
  if (inputCount == 0) {
    return cudaSuccess;
  }

  constexpr unsigned int blockSize = 256;
  const unsigned int gridSize = launchGrid1D(inputCount, blockSize);
  pointTriangleBarrierHessianKernel<<<gridSize, blockSize>>>(
      inputs,
      squaredDistances,
      barrierValues,
      barrierGradients,
      barrierHessians,
      activeBarriers,
      inputCount);

  return cudaGetLastError();
}

//==============================================================================
cudaError_t launchPointTriangleTangentStencilKernel(
    const PointTriangleTangentInput* inputs,
    double* basisValues,
    double* coordinates,
    double* projectionValues,
    std::uint8_t* fallbackBases,
    std::size_t inputCount)
{
  if (inputCount == 0) {
    return cudaSuccess;
  }

  constexpr unsigned int blockSize = 256;
  const unsigned int gridSize = launchGrid1D(inputCount, blockSize);
  pointTriangleTangentStencilKernel<<<gridSize, blockSize>>>(
      inputs,
      basisValues,
      coordinates,
      projectionValues,
      fallbackBases,
      inputCount);

  return cudaGetLastError();
}

//==============================================================================
cudaError_t launchPointPointBarrierHessianKernel(
    const PointPointBarrierInput* inputs,
    double* squaredDistances,
    double* barrierValues,
    double* barrierGradients,
    double* barrierHessians,
    std::uint8_t* activeBarriers,
    std::size_t inputCount)
{
  if (inputCount == 0) {
    return cudaSuccess;
  }

  constexpr unsigned int blockSize = 256;
  const unsigned int gridSize = launchGrid1D(inputCount, blockSize);
  pointPointBarrierHessianKernel<<<gridSize, blockSize>>>(
      inputs,
      squaredDistances,
      barrierValues,
      barrierGradients,
      barrierHessians,
      activeBarriers,
      inputCount);

  return cudaGetLastError();
}

//==============================================================================
cudaError_t launchPointEdgeBarrierHessianKernel(
    const PointEdgeBarrierInput* inputs,
    double* squaredDistances,
    double* barrierValues,
    double* barrierGradients,
    double* barrierHessians,
    std::uint8_t* activeBarriers,
    std::size_t inputCount)
{
  if (inputCount == 0) {
    return cudaSuccess;
  }

  constexpr unsigned int blockSize = 256;
  const unsigned int gridSize = launchGrid1D(inputCount, blockSize);
  pointEdgeBarrierHessianKernel<<<gridSize, blockSize>>>(
      inputs,
      squaredDistances,
      barrierValues,
      barrierGradients,
      barrierHessians,
      activeBarriers,
      inputCount);

  return cudaGetLastError();
}

//==============================================================================
cudaError_t launchEdgeEdgeBarrierHessianKernel(
    const EdgeEdgeBarrierInput* inputs,
    double* squaredDistances,
    double* barrierValues,
    double* barrierGradients,
    double* barrierHessians,
    std::uint8_t* activeBarriers,
    std::size_t inputCount)
{
  if (inputCount == 0) {
    return cudaSuccess;
  }

  constexpr unsigned int blockSize = 256;
  const unsigned int gridSize = launchGrid1D(inputCount, blockSize);
  edgeEdgeBarrierHessianKernel<<<gridSize, blockSize>>>(
      inputs,
      squaredDistances,
      barrierValues,
      barrierGradients,
      barrierHessians,
      activeBarriers,
      inputCount);

  return cudaGetLastError();
}

//==============================================================================
cudaError_t launchEdgeEdgeTangentStencilKernel(
    const EdgeEdgeTangentInput* inputs,
    double* basisValues,
    double* coordinates,
    double* projectionValues,
    std::uint8_t* fallbackBases,
    std::size_t inputCount)
{
  if (inputCount == 0) {
    return cudaSuccess;
  }

  constexpr unsigned int blockSize = 256;
  const unsigned int gridSize = launchGrid1D(inputCount, blockSize);
  edgeEdgeTangentStencilKernel<<<gridSize, blockSize>>>(
      inputs,
      basisValues,
      coordinates,
      projectionValues,
      fallbackBases,
      inputCount);

  return cudaGetLastError();
}

//==============================================================================
cudaError_t launchPointEdgeTangentStencilKernel(
    const PointEdgeTangentInput* inputs,
    double* basisValues,
    double* coordinates,
    double* projectionValues,
    std::uint8_t* fallbackBases,
    std::size_t inputCount)
{
  if (inputCount == 0) {
    return cudaSuccess;
  }

  constexpr unsigned int blockSize = 256;
  const unsigned int gridSize = launchGrid1D(inputCount, blockSize);
  pointEdgeTangentStencilKernel<<<gridSize, blockSize>>>(
      inputs,
      basisValues,
      coordinates,
      projectionValues,
      fallbackBases,
      inputCount);

  return cudaGetLastError();
}

//==============================================================================
cudaError_t launchPointPointTangentStencilKernel(
    const PointPointTangentInput* inputs,
    double* basisValues,
    double* projectionValues,
    std::uint8_t* fallbackBases,
    std::size_t inputCount)
{
  if (inputCount == 0) {
    return cudaSuccess;
  }

  constexpr unsigned int blockSize = 256;
  const unsigned int gridSize = launchGrid1D(inputCount, blockSize);
  pointPointTangentStencilKernel<<<gridSize, blockSize>>>(
      inputs, basisValues, projectionValues, fallbackBases, inputCount);

  return cudaGetLastError();
}

} // namespace dart::simulation::compute::cuda::detail
