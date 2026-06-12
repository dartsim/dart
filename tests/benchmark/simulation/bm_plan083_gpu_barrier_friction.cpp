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

#include <dart/simulation/detail/newton_barrier/barrier_kernel.hpp>
#include <dart/simulation/detail/newton_barrier/friction_kernel.hpp>
#include <dart/simulation/detail/newton_barrier/tangent_stencil.hpp>

#include <benchmark/benchmark.h>
#include <dart/simulation/compute/cuda/barrier_friction_kernel_cuda.cuh>
#include <dart/simulation/compute/cuda/cuda_runtime.cuh>

#include <algorithm>
#include <vector>

#include <cmath>
#include <cstdint>

namespace cuda = dart::simulation::compute::cuda;
namespace nb = dart::simulation::detail::newton_barrier;

namespace {

struct CpuLocalResult
{
  std::vector<double> barrierValues;
  std::vector<double> barrierFirstDerivatives;
  std::vector<double> barrierSecondDerivatives;
  std::vector<double> frictionValues;
  std::vector<double> frictionWorks;
  std::vector<double> frictionFirstDerivatives;
  std::vector<double> frictionSecondDerivatives;
  std::vector<std::uint8_t> activeBarriers;
  std::vector<std::uint8_t> activeFrictions;
  std::vector<std::uint8_t> dynamicFrictions;
  std::size_t activeBarrierCount = 0;
  std::size_t activeFrictionCount = 0;
  std::size_t dynamicFrictionCount = 0;
  double maxBarrierValue = 0.0;
  double maxFrictionWork = 0.0;
};

struct LocalFixture
{
  std::vector<cuda::BarrierFrictionLocalInput> inputs;
  CpuLocalResult cpu;
};

struct CpuPointTriangleResult
{
  std::vector<double> squaredDistances;
  std::vector<double> barrierValues;
  std::vector<double> barrierGradients;
  std::vector<std::uint8_t> activeBarriers;
  std::size_t activeBarrierCount = 0;
  double maxBarrierValue = 0.0;
};

struct CpuPointPointBarrierResult
{
  std::vector<double> squaredDistances;
  std::vector<double> barrierValues;
  std::vector<double> barrierGradients;
  std::vector<double> barrierHessians;
  std::vector<std::uint8_t> activeBarriers;
  std::size_t activeBarrierCount = 0;
  double maxBarrierValue = 0.0;
};

struct CpuPointTriangleTangentResult
{
  std::vector<double> basisValues;
  std::vector<double> coordinates;
  std::vector<double> projectionValues;
  std::vector<std::uint8_t> fallbackBases;
  std::size_t fallbackBasisCount = 0;
};

using CpuEdgeEdgeTangentResult = CpuPointTriangleTangentResult;

struct CpuPointEdgeTangentResult
{
  std::vector<double> basisValues;
  std::vector<double> coordinates;
  std::vector<double> projectionValues;
  std::vector<std::uint8_t> fallbackBases;
  std::size_t fallbackBasisCount = 0;
};

struct CpuPointPointTangentResult
{
  std::vector<double> basisValues;
  std::vector<double> projectionValues;
  std::vector<std::uint8_t> fallbackBases;
  std::size_t fallbackBasisCount = 0;
};

struct PointTriangleFixture
{
  std::vector<cuda::PointTriangleBarrierInput> inputs;
  CpuPointTriangleResult cpu;
};

struct PointPointBarrierFixture
{
  std::vector<cuda::PointPointBarrierInput> inputs;
  CpuPointPointBarrierResult cpu;
};

struct PointTriangleTangentFixture
{
  std::vector<cuda::PointTriangleTangentInput> inputs;
  CpuPointTriangleTangentResult cpu;
};

struct EdgeEdgeTangentFixture
{
  std::vector<cuda::EdgeEdgeTangentInput> inputs;
  CpuEdgeEdgeTangentResult cpu;
};

struct PointEdgeTangentFixture
{
  std::vector<cuda::PointEdgeTangentInput> inputs;
  CpuPointEdgeTangentResult cpu;
};

struct PointPointTangentFixture
{
  std::vector<cuda::PointPointTangentInput> inputs;
  CpuPointPointTangentResult cpu;
};

cuda::BarrierFrictionLocalInput makeInput(const int i)
{
  const bool activeBarrier = (i % 5) != 0;
  const bool dynamicFriction = (i % 3) == 0;
  const double normalized = static_cast<double>((i % 17) + 1) / 32.0;
  return {
      .squaredDistance = activeBarrier ? normalized : 1.25 + normalized,
      .squaredActivationDistance = 1.0,
      .stiffness = 1.0 + static_cast<double>(i % 7) * 0.125,
      .tangentialDisplacementNorm
      = dynamicFriction ? 0.3 + 0.01 * static_cast<double>(i % 5)
                        : 0.03 + 0.005 * static_cast<double>(i % 9),
      .frictionWeight = 0.5 + static_cast<double>(i % 11) * 0.25,
      .staticFrictionDisplacement = 0.2,
  };
}

void writeVec3(double destination[3], const Eigen::Vector3d& source)
{
  destination[0] = source.x();
  destination[1] = source.y();
  destination[2] = source.z();
}

Eigen::Vector3d readVec3(const double values[3])
{
  return {values[0], values[1], values[2]};
}

cuda::PointTriangleBarrierInput makePointTriangleInput(const int i)
{
  const double column = static_cast<double>(i % 257) / 257.0;
  const double row = static_cast<double>((i / 257) % 251) / 251.0;
  const bool outsideEdge = (i % 7) == 0;
  const bool inactive = (i % 11) == 0;
  const bool degenerate = (i % 23) == 0;

  const Eigen::Vector3d a(0.0, 0.0, 0.0);
  const Eigen::Vector3d b = degenerate ? Eigen::Vector3d(1e-8, 0.0, 0.0)
                                       : Eigen::Vector3d(1.0, 0.0, 0.0);
  const Eigen::Vector3d c = degenerate ? Eigen::Vector3d(0.0, 1e-8, 0.0)
                                       : Eigen::Vector3d(0.0, 1.0, 0.0);
  const double z = inactive ? 1.5 : 0.03 + 0.0001 * static_cast<double>(i % 19);
  const Eigen::Vector3d p(
      outsideEdge ? 1.1 : 0.1 + 0.6 * column,
      outsideEdge ? -0.2 : 0.1 + 0.6 * row,
      z);

  cuda::PointTriangleBarrierInput input;
  writeVec3(input.point, p);
  writeVec3(input.triangleA, a);
  writeVec3(input.triangleB, b);
  writeVec3(input.triangleC, c);
  input.squaredActivationDistance = 0.25;
  input.stiffness = 1.0 + 0.125 * static_cast<double>(i % 13);
  return input;
}

cuda::PointPointBarrierInput makePointPointBarrierInput(const int i)
{
  const bool inactive = (i % 11) == 0;
  const double column = static_cast<double>(i % 257) / 257.0;
  const double row = static_cast<double>((i / 257) % 251) / 251.0;
  const Eigen::Vector3d a(
      -0.2 + 0.4 * column,
      0.15 * row,
      0.05 + 0.0005 * static_cast<double>(i % 17));
  const Eigen::Vector3d offset
      = inactive ? Eigen::Vector3d(0.9, 0.6, 0.5)
                 : Eigen::Vector3d(
                       0.04 + 0.001 * static_cast<double>(i % 23),
                       0.05 + 0.001 * static_cast<double>(i % 19),
                       0.06 + 0.001 * static_cast<double>(i % 29));

  cuda::PointPointBarrierInput input;
  writeVec3(input.pointA, a);
  writeVec3(input.pointB, a + offset);
  input.squaredActivationDistance = 0.25;
  input.stiffness = 1.0 + 0.125 * static_cast<double>(i % 13);
  return input;
}

cuda::PointTriangleTangentInput makePointTriangleTangentInput(const int i)
{
  const double column = static_cast<double>(i % 257) / 257.0;
  const double row = static_cast<double>((i / 257) % 251) / 251.0;
  const bool outsideEdge = (i % 7) == 0;

  const Eigen::Vector3d a(0.0, 0.0, 0.0);
  const Eigen::Vector3d b(1.0 + 0.001 * static_cast<double>(i % 13), 0.0, 0.0);
  const Eigen::Vector3d c(
      0.02 * static_cast<double>(i % 5),
      1.0 + 0.001 * static_cast<double>(i % 17),
      0.03 * static_cast<double>(i % 11));
  const Eigen::Vector3d p(
      outsideEdge ? 1.1 : 0.1 + 0.6 * column,
      outsideEdge ? -0.2 : 0.1 + 0.6 * row,
      0.03 + 0.0001 * static_cast<double>(i % 19));

  cuda::PointTriangleTangentInput input;
  writeVec3(input.point, p);
  writeVec3(input.triangleA, a);
  writeVec3(input.triangleB, b);
  writeVec3(input.triangleC, c);
  return input;
}

cuda::EdgeEdgeTangentInput makeEdgeEdgeTangentInput(const int i)
{
  const double column = static_cast<double>(i % 257) / 257.0;
  const double row = static_cast<double>((i / 257) % 251) / 251.0;
  const double skew = 0.001 * static_cast<double>(i % 19);

  const Eigen::Vector3d a(
      0.05 * static_cast<double>(i % 11), -0.1 + 0.2 * row, skew);
  const Eigen::Vector3d b = a
                            + Eigen::Vector3d(
                                1.0 + 0.001 * static_cast<double>(i % 13),
                                0.1 * row,
                                0.2 + 0.01 * static_cast<double>(i % 7));
  const Eigen::Vector3d c(
      0.15 + 0.6 * column,
      -0.25 + 0.75 * row,
      0.35 + 0.0005 * static_cast<double>(i % 17));
  const Eigen::Vector3d d = c
                            + Eigen::Vector3d(
                                0.1 + 0.05 * column,
                                1.0 + 0.001 * static_cast<double>(i % 23),
                                0.3 + 0.01 * static_cast<double>(i % 5));

  cuda::EdgeEdgeTangentInput input;
  writeVec3(input.edgeA0, a);
  writeVec3(input.edgeA1, b);
  writeVec3(input.edgeB0, c);
  writeVec3(input.edgeB1, d);
  return input;
}

cuda::PointEdgeTangentInput makePointEdgeTangentInput(const int i)
{
  const double column = static_cast<double>(i % 257) / 257.0;
  const double row = static_cast<double>((i / 257) % 251) / 251.0;
  const Eigen::Vector3d a(
      -0.15 + 0.1 * row,
      0.05 * static_cast<double>(i % 11),
      0.02 * static_cast<double>(i % 17));
  const Eigen::Vector3d b = a
                            + Eigen::Vector3d(
                                1.0 + 0.001 * static_cast<double>(i % 13),
                                0.3 + 0.1 * row,
                                0.2 + 0.01 * static_cast<double>(i % 7));
  const Eigen::Vector3d p(
      0.1 + 0.8 * column,
      -0.2 + 0.9 * row,
      0.4 + 0.0005 * static_cast<double>(i % 19));

  cuda::PointEdgeTangentInput input;
  writeVec3(input.point, p);
  writeVec3(input.edgeA, a);
  writeVec3(input.edgeB, b);
  return input;
}

cuda::PointPointTangentInput makePointPointTangentInput(const int i)
{
  const double column = static_cast<double>(i % 257) / 257.0;
  const double row = static_cast<double>((i / 257) % 251) / 251.0;
  const Eigen::Vector3d a(
      -0.4 + column, -0.3 + row, 0.1 + 0.0005 * static_cast<double>(i % 17));
  const Eigen::Vector3d b = a
                            + Eigen::Vector3d(
                                0.2 + 0.01 * static_cast<double>(i % 23),
                                0.4 + 0.01 * static_cast<double>(i % 19),
                                1.0 + 0.001 * static_cast<double>(i % 29));

  cuda::PointPointTangentInput input;
  writeVec3(input.pointA, a);
  writeVec3(input.pointB, b);
  return input;
}

void resizeCpuResult(CpuLocalResult& result, const std::size_t count)
{
  result.barrierValues.assign(count, 0.0);
  result.barrierFirstDerivatives.assign(count, 0.0);
  result.barrierSecondDerivatives.assign(count, 0.0);
  result.frictionValues.assign(count, 0.0);
  result.frictionWorks.assign(count, 0.0);
  result.frictionFirstDerivatives.assign(count, 0.0);
  result.frictionSecondDerivatives.assign(count, 0.0);
  result.activeBarriers.assign(count, 0u);
  result.activeFrictions.assign(count, 0u);
  result.dynamicFrictions.assign(count, 0u);
  result.activeBarrierCount = 0;
  result.activeFrictionCount = 0;
  result.dynamicFrictionCount = 0;
  result.maxBarrierValue = 0.0;
  result.maxFrictionWork = 0.0;
}

void resizeCpuResult(CpuPointTriangleResult& result, const std::size_t count)
{
  result.squaredDistances.assign(count, 0.0);
  result.barrierValues.assign(count, 0.0);
  result.barrierGradients.assign(12 * count, 0.0);
  result.activeBarriers.assign(count, 0u);
  result.activeBarrierCount = 0;
  result.maxBarrierValue = 0.0;
}

void resizeCpuResult(
    CpuPointPointBarrierResult& result, const std::size_t count)
{
  result.squaredDistances.assign(count, 0.0);
  result.barrierValues.assign(count, 0.0);
  result.barrierGradients.assign(6 * count, 0.0);
  result.barrierHessians.assign(36 * count, 0.0);
  result.activeBarriers.assign(count, 0u);
  result.activeBarrierCount = 0;
  result.maxBarrierValue = 0.0;
}

void resizeCpuResult(
    CpuPointTriangleTangentResult& result, const std::size_t count)
{
  result.basisValues.assign(6 * count, 0.0);
  result.coordinates.assign(2 * count, 0.0);
  result.projectionValues.assign(24 * count, 0.0);
  result.fallbackBases.assign(count, 0u);
  result.fallbackBasisCount = 0;
}

void resizeCpuResult(CpuPointEdgeTangentResult& result, const std::size_t count)
{
  result.basisValues.assign(6 * count, 0.0);
  result.coordinates.assign(count, 0.0);
  result.projectionValues.assign(18 * count, 0.0);
  result.fallbackBases.assign(count, 0u);
  result.fallbackBasisCount = 0;
}

void resizeCpuResult(
    CpuPointPointTangentResult& result, const std::size_t count)
{
  result.basisValues.assign(6 * count, 0.0);
  result.projectionValues.assign(12 * count, 0.0);
  result.fallbackBases.assign(count, 0u);
  result.fallbackBasisCount = 0;
}

void evaluateCpu(
    const std::vector<cuda::BarrierFrictionLocalInput>& inputs,
    CpuLocalResult& result)
{
  resizeCpuResult(result, inputs.size());
  for (std::size_t i = 0; i < inputs.size(); ++i) {
    const auto& input = inputs[i];
    const double stiffness = std::max(0.0, input.stiffness);
    const auto barrier = nb::c2ClampedLogBarrier(
        input.squaredDistance, input.squaredActivationDistance);
    if (barrier.active && stiffness > 0.0) {
      result.activeBarriers[i] = 1u;
      ++result.activeBarrierCount;
      result.barrierValues[i] = stiffness * barrier.value;
      result.barrierFirstDerivatives[i] = stiffness * barrier.firstDerivative;
      result.barrierSecondDerivatives[i] = stiffness * barrier.secondDerivative;
      result.maxBarrierValue
          = std::max(result.maxBarrierValue, result.barrierValues[i]);
    }

    const auto smooth = nb::smoothFrictionNorm(
        input.tangentialDisplacementNorm, input.staticFrictionDisplacement);
    const auto work = nb::frictionWorkContribution(
        input.tangentialDisplacementNorm,
        input.frictionWeight,
        input.staticFrictionDisplacement);
    if (work.active) {
      result.activeFrictions[i] = 1u;
      ++result.activeFrictionCount;
      result.dynamicFrictions[i] = smooth.dynamicBranch ? 1u : 0u;
      result.dynamicFrictionCount += smooth.dynamicBranch ? 1u : 0u;
      result.frictionValues[i] = input.frictionWeight * smooth.value;
      result.frictionWorks[i] = work.work;
      result.frictionFirstDerivatives[i] = smooth.firstDerivative;
      result.frictionSecondDerivatives[i] = smooth.secondDerivative;
      result.maxFrictionWork
          = std::max(result.maxFrictionWork, result.frictionWorks[i]);
    }
  }
}

void evaluateCpu(
    const std::vector<cuda::PointTriangleBarrierInput>& inputs,
    CpuPointTriangleResult& result)
{
  resizeCpuResult(result, inputs.size());
  for (std::size_t i = 0; i < inputs.size(); ++i) {
    const auto& input = inputs[i];
    const auto barrier = nb::pointTriangleBarrier(
        readVec3(input.point),
        readVec3(input.triangleA),
        readVec3(input.triangleB),
        readVec3(input.triangleC),
        input.squaredActivationDistance,
        input.stiffness);
    result.squaredDistances[i] = barrier.squaredDistance;
    result.barrierValues[i] = barrier.value;
    if (barrier.active) {
      result.activeBarriers[i] = 1u;
      ++result.activeBarrierCount;
      result.maxBarrierValue
          = std::max(result.maxBarrierValue, result.barrierValues[i]);
    }
    for (int entry = 0; entry < 12; ++entry) {
      result.barrierGradients[12 * i + static_cast<std::size_t>(entry)]
          = barrier.gradient[entry];
    }
  }
}

void evaluateCpu(
    const std::vector<cuda::PointPointBarrierInput>& inputs,
    CpuPointPointBarrierResult& result)
{
  resizeCpuResult(result, inputs.size());
  for (std::size_t i = 0; i < inputs.size(); ++i) {
    const auto& input = inputs[i];
    const auto barrier = nb::pointPointBarrier(
        readVec3(input.pointA),
        readVec3(input.pointB),
        input.squaredActivationDistance,
        input.stiffness);
    result.squaredDistances[i] = barrier.squaredDistance;
    result.barrierValues[i] = barrier.value;
    if (barrier.active) {
      result.activeBarriers[i] = 1u;
      ++result.activeBarrierCount;
      result.maxBarrierValue
          = std::max(result.maxBarrierValue, result.barrierValues[i]);
    }
    for (int entry = 0; entry < 6; ++entry) {
      result.barrierGradients[6 * i + static_cast<std::size_t>(entry)]
          = barrier.gradient[entry];
    }
    for (int row = 0; row < 6; ++row) {
      for (int col = 0; col < 6; ++col) {
        result.barrierHessians[36 * i + static_cast<std::size_t>(6 * row + col)]
            = barrier.hessian(row, col);
      }
    }
  }
}

void evaluateCpu(
    const std::vector<cuda::PointTriangleTangentInput>& inputs,
    CpuPointTriangleTangentResult& result)
{
  resizeCpuResult(result, inputs.size());
  for (std::size_t i = 0; i < inputs.size(); ++i) {
    const auto& input = inputs[i];
    const auto stencil = nb::pointTriangleTangentStencil(
        readVec3(input.point),
        readVec3(input.triangleA),
        readVec3(input.triangleB),
        readVec3(input.triangleC));

    const std::size_t basisOffset = 6 * i;
    for (int component = 0; component < 3; ++component) {
      result.basisValues[basisOffset + static_cast<std::size_t>(component)]
          = stencil.basis(component, 0);
      result.basisValues[basisOffset + 3u + static_cast<std::size_t>(component)]
          = stencil.basis(component, 1);
    }

    const std::size_t coordinateOffset = 2 * i;
    result.coordinates[coordinateOffset] = stencil.coordinates.x();
    result.coordinates[coordinateOffset + 1] = stencil.coordinates.y();

    const std::size_t projectionOffset = 24 * i;
    for (int block = 0; block < 4; ++block) {
      for (int component = 0; component < 3; ++component) {
        const std::size_t blockOffset
            = projectionOffset + static_cast<std::size_t>(6 * block);
        result
            .projectionValues[blockOffset + static_cast<std::size_t>(component)]
            = stencil.projection(0, 3 * block + component);
        result.projectionValues
            [blockOffset + 3u + static_cast<std::size_t>(component)]
            = stencil.projection(1, 3 * block + component);
      }
    }

    if (stencil.usedFallbackBasis) {
      result.fallbackBases[i] = 1u;
      ++result.fallbackBasisCount;
    }
  }
}

void evaluateCpu(
    const std::vector<cuda::EdgeEdgeTangentInput>& inputs,
    CpuEdgeEdgeTangentResult& result)
{
  resizeCpuResult(result, inputs.size());
  for (std::size_t i = 0; i < inputs.size(); ++i) {
    const auto& input = inputs[i];
    const auto stencil = nb::edgeEdgeTangentStencil(
        readVec3(input.edgeA0),
        readVec3(input.edgeA1),
        readVec3(input.edgeB0),
        readVec3(input.edgeB1));

    const std::size_t basisOffset = 6 * i;
    for (int component = 0; component < 3; ++component) {
      result.basisValues[basisOffset + static_cast<std::size_t>(component)]
          = stencil.basis(component, 0);
      result.basisValues[basisOffset + 3u + static_cast<std::size_t>(component)]
          = stencil.basis(component, 1);
    }

    const std::size_t coordinateOffset = 2 * i;
    result.coordinates[coordinateOffset] = stencil.coordinates.x();
    result.coordinates[coordinateOffset + 1] = stencil.coordinates.y();

    const std::size_t projectionOffset = 24 * i;
    for (int block = 0; block < 4; ++block) {
      for (int component = 0; component < 3; ++component) {
        const std::size_t blockOffset
            = projectionOffset + static_cast<std::size_t>(6 * block);
        result
            .projectionValues[blockOffset + static_cast<std::size_t>(component)]
            = stencil.projection(0, 3 * block + component);
        result.projectionValues
            [blockOffset + 3u + static_cast<std::size_t>(component)]
            = stencil.projection(1, 3 * block + component);
      }
    }

    if (stencil.usedFallbackBasis) {
      result.fallbackBases[i] = 1u;
      ++result.fallbackBasisCount;
    }
  }
}

void evaluateCpu(
    const std::vector<cuda::PointEdgeTangentInput>& inputs,
    CpuPointEdgeTangentResult& result)
{
  resizeCpuResult(result, inputs.size());
  for (std::size_t i = 0; i < inputs.size(); ++i) {
    const auto& input = inputs[i];
    const auto stencil = nb::pointEdgeTangentStencil(
        readVec3(input.point), readVec3(input.edgeA), readVec3(input.edgeB));

    const std::size_t basisOffset = 6 * i;
    for (int component = 0; component < 3; ++component) {
      result.basisValues[basisOffset + static_cast<std::size_t>(component)]
          = stencil.basis(component, 0);
      result.basisValues[basisOffset + 3u + static_cast<std::size_t>(component)]
          = stencil.basis(component, 1);
    }

    result.coordinates[i] = stencil.coordinate;

    const std::size_t projectionOffset = 18 * i;
    for (int block = 0; block < 3; ++block) {
      for (int component = 0; component < 3; ++component) {
        const std::size_t blockOffset
            = projectionOffset + static_cast<std::size_t>(6 * block);
        result
            .projectionValues[blockOffset + static_cast<std::size_t>(component)]
            = stencil.projection(0, 3 * block + component);
        result.projectionValues
            [blockOffset + 3u + static_cast<std::size_t>(component)]
            = stencil.projection(1, 3 * block + component);
      }
    }

    if (stencil.usedFallbackBasis) {
      result.fallbackBases[i] = 1u;
      ++result.fallbackBasisCount;
    }
  }
}

void evaluateCpu(
    const std::vector<cuda::PointPointTangentInput>& inputs,
    CpuPointPointTangentResult& result)
{
  resizeCpuResult(result, inputs.size());
  for (std::size_t i = 0; i < inputs.size(); ++i) {
    const auto& input = inputs[i];
    const auto stencil = nb::pointPointTangentStencil(
        readVec3(input.pointA), readVec3(input.pointB));

    const std::size_t basisOffset = 6 * i;
    for (int component = 0; component < 3; ++component) {
      result.basisValues[basisOffset + static_cast<std::size_t>(component)]
          = stencil.basis(component, 0);
      result.basisValues[basisOffset + 3u + static_cast<std::size_t>(component)]
          = stencil.basis(component, 1);
    }

    const std::size_t projectionOffset = 12 * i;
    for (int block = 0; block < 2; ++block) {
      for (int component = 0; component < 3; ++component) {
        const std::size_t blockOffset
            = projectionOffset + static_cast<std::size_t>(6 * block);
        result
            .projectionValues[blockOffset + static_cast<std::size_t>(component)]
            = stencil.projection(0, 3 * block + component);
        result.projectionValues
            [blockOffset + 3u + static_cast<std::size_t>(component)]
            = stencil.projection(1, 3 * block + component);
      }
    }

    if (stencil.usedFallbackBasis) {
      result.fallbackBases[i] = 1u;
      ++result.fallbackBasisCount;
    }
  }
}

LocalFixture makeFixture(const int sampleCount)
{
  LocalFixture fixture;
  fixture.inputs.reserve(static_cast<std::size_t>(sampleCount));
  for (int i = 0; i < sampleCount; ++i) {
    fixture.inputs.push_back(makeInput(i));
  }
  evaluateCpu(fixture.inputs, fixture.cpu);
  return fixture;
}

PointTriangleFixture makePointTriangleFixture(const int sampleCount)
{
  PointTriangleFixture fixture;
  fixture.inputs.reserve(static_cast<std::size_t>(sampleCount));
  for (int i = 0; i < sampleCount; ++i) {
    fixture.inputs.push_back(makePointTriangleInput(i));
  }
  evaluateCpu(fixture.inputs, fixture.cpu);
  return fixture;
}

PointPointBarrierFixture makePointPointBarrierFixture(const int sampleCount)
{
  PointPointBarrierFixture fixture;
  fixture.inputs.reserve(static_cast<std::size_t>(sampleCount));
  for (int i = 0; i < sampleCount; ++i) {
    fixture.inputs.push_back(makePointPointBarrierInput(i));
  }
  evaluateCpu(fixture.inputs, fixture.cpu);
  return fixture;
}

PointTriangleTangentFixture makePointTriangleTangentFixture(
    const int sampleCount)
{
  PointTriangleTangentFixture fixture;
  fixture.inputs.reserve(static_cast<std::size_t>(sampleCount));
  for (int i = 0; i < sampleCount; ++i) {
    fixture.inputs.push_back(makePointTriangleTangentInput(i));
  }
  evaluateCpu(fixture.inputs, fixture.cpu);
  return fixture;
}

EdgeEdgeTangentFixture makeEdgeEdgeTangentFixture(const int sampleCount)
{
  EdgeEdgeTangentFixture fixture;
  fixture.inputs.reserve(static_cast<std::size_t>(sampleCount));
  for (int i = 0; i < sampleCount; ++i) {
    fixture.inputs.push_back(makeEdgeEdgeTangentInput(i));
  }
  evaluateCpu(fixture.inputs, fixture.cpu);
  return fixture;
}

PointEdgeTangentFixture makePointEdgeTangentFixture(const int sampleCount)
{
  PointEdgeTangentFixture fixture;
  fixture.inputs.reserve(static_cast<std::size_t>(sampleCount));
  for (int i = 0; i < sampleCount; ++i) {
    fixture.inputs.push_back(makePointEdgeTangentInput(i));
  }
  evaluateCpu(fixture.inputs, fixture.cpu);
  return fixture;
}

PointPointTangentFixture makePointPointTangentFixture(const int sampleCount)
{
  PointPointTangentFixture fixture;
  fixture.inputs.reserve(static_cast<std::size_t>(sampleCount));
  for (int i = 0; i < sampleCount; ++i) {
    fixture.inputs.push_back(makePointPointTangentInput(i));
  }
  evaluateCpu(fixture.inputs, fixture.cpu);
  return fixture;
}

template <typename Lhs, typename Rhs>
double maxAbsDifference(const Lhs& lhs, const Rhs& rhs)
{
  double maxError = 0.0;
  for (std::size_t i = 0; i < lhs.size(); ++i) {
    maxError = std::max(maxError, std::abs(lhs[i] - rhs[i]));
  }
  return maxError;
}

double maxOutputError(
    const CpuLocalResult& expected,
    const cuda::BarrierFrictionLocalResult& actual)
{
  double maxError = 0.0;
  maxError = std::max(
      maxError, maxAbsDifference(expected.barrierValues, actual.barrierValues));
  maxError = std::max(
      maxError,
      maxAbsDifference(
          expected.barrierFirstDerivatives, actual.barrierFirstDerivatives));
  maxError = std::max(
      maxError,
      maxAbsDifference(
          expected.barrierSecondDerivatives, actual.barrierSecondDerivatives));
  maxError = std::max(
      maxError,
      maxAbsDifference(expected.frictionValues, actual.frictionValues));
  maxError = std::max(
      maxError, maxAbsDifference(expected.frictionWorks, actual.frictionWorks));
  maxError = std::max(
      maxError,
      maxAbsDifference(
          expected.frictionFirstDerivatives, actual.frictionFirstDerivatives));
  maxError = std::max(
      maxError,
      maxAbsDifference(
          expected.frictionSecondDerivatives,
          actual.frictionSecondDerivatives));
  return maxError;
}

double maxOutputError(
    const CpuPointTriangleResult& expected,
    const cuda::PointTriangleBarrierGradientResult& actual)
{
  double maxError = 0.0;
  maxError = std::max(
      maxError,
      maxAbsDifference(expected.squaredDistances, actual.squaredDistances));
  maxError = std::max(
      maxError, maxAbsDifference(expected.barrierValues, actual.barrierValues));
  maxError = std::max(
      maxError,
      maxAbsDifference(expected.barrierGradients, actual.barrierGradients));
  return maxError;
}

double maxOutputError(
    const CpuPointPointBarrierResult& expected,
    const cuda::PointPointBarrierHessianResult& actual)
{
  double maxError = 0.0;
  maxError = std::max(
      maxError,
      maxAbsDifference(expected.squaredDistances, actual.squaredDistances));
  maxError = std::max(
      maxError, maxAbsDifference(expected.barrierValues, actual.barrierValues));
  maxError = std::max(
      maxError,
      maxAbsDifference(expected.barrierGradients, actual.barrierGradients));
  maxError = std::max(
      maxError,
      maxAbsDifference(expected.barrierHessians, actual.barrierHessians));
  return maxError;
}

double maxOutputError(
    const CpuPointTriangleTangentResult& expected,
    const cuda::PointTriangleTangentStencilResult& actual)
{
  double maxError = 0.0;
  maxError = std::max(
      maxError, maxAbsDifference(expected.basisValues, actual.basisValues));
  maxError = std::max(
      maxError, maxAbsDifference(expected.coordinates, actual.coordinates));
  maxError = std::max(
      maxError,
      maxAbsDifference(expected.projectionValues, actual.projectionValues));
  return maxError;
}

double maxOutputError(
    const CpuEdgeEdgeTangentResult& expected,
    const cuda::EdgeEdgeTangentStencilResult& actual)
{
  double maxError = 0.0;
  maxError = std::max(
      maxError, maxAbsDifference(expected.basisValues, actual.basisValues));
  maxError = std::max(
      maxError, maxAbsDifference(expected.coordinates, actual.coordinates));
  maxError = std::max(
      maxError,
      maxAbsDifference(expected.projectionValues, actual.projectionValues));
  return maxError;
}

double maxOutputError(
    const CpuPointEdgeTangentResult& expected,
    const cuda::PointEdgeTangentStencilResult& actual)
{
  double maxError = 0.0;
  maxError = std::max(
      maxError, maxAbsDifference(expected.basisValues, actual.basisValues));
  maxError = std::max(
      maxError, maxAbsDifference(expected.coordinates, actual.coordinates));
  maxError = std::max(
      maxError,
      maxAbsDifference(expected.projectionValues, actual.projectionValues));
  return maxError;
}

double maxOutputError(
    const CpuPointPointTangentResult& expected,
    const cuda::PointPointTangentStencilResult& actual)
{
  double maxError = 0.0;
  maxError = std::max(
      maxError, maxAbsDifference(expected.basisValues, actual.basisValues));
  maxError = std::max(
      maxError,
      maxAbsDifference(expected.projectionValues, actual.projectionValues));
  return maxError;
}

void recordCounters(
    benchmark::State& state, const LocalFixture& fixture, const double maxError)
{
  state.counters["samples"] = static_cast<double>(fixture.inputs.size());
  state.counters["active_barriers"]
      = static_cast<double>(fixture.cpu.activeBarrierCount);
  state.counters["active_friction"]
      = static_cast<double>(fixture.cpu.activeFrictionCount);
  state.counters["dynamic_friction"]
      = static_cast<double>(fixture.cpu.dynamicFrictionCount);
  state.counters["max_barrier_value"] = fixture.cpu.maxBarrierValue;
  state.counters["max_friction_work"] = fixture.cpu.maxFrictionWork;
  state.counters["max_result_abs_error"] = maxError;
  state.SetItemsProcessed(
      static_cast<std::int64_t>(state.iterations() * fixture.inputs.size()));
}

void recordCounters(
    benchmark::State& state,
    const PointTriangleFixture& fixture,
    const double maxError)
{
  state.counters["samples"] = static_cast<double>(fixture.inputs.size());
  state.counters["active_barriers"]
      = static_cast<double>(fixture.cpu.activeBarrierCount);
  state.counters["max_barrier_value"] = fixture.cpu.maxBarrierValue;
  state.counters["max_result_abs_error"] = maxError;
  state.SetItemsProcessed(
      static_cast<std::int64_t>(state.iterations() * fixture.inputs.size()));
}

void recordCounters(
    benchmark::State& state,
    const PointPointBarrierFixture& fixture,
    const double maxError)
{
  state.counters["samples"] = static_cast<double>(fixture.inputs.size());
  state.counters["active_barriers"]
      = static_cast<double>(fixture.cpu.activeBarrierCount);
  state.counters["max_barrier_value"] = fixture.cpu.maxBarrierValue;
  state.counters["max_result_abs_error"] = maxError;
  state.SetItemsProcessed(
      static_cast<std::int64_t>(state.iterations() * fixture.inputs.size()));
}

void recordCounters(
    benchmark::State& state,
    const PointTriangleTangentFixture& fixture,
    const double maxError)
{
  state.counters["samples"] = static_cast<double>(fixture.inputs.size());
  state.counters["fallback_bases"]
      = static_cast<double>(fixture.cpu.fallbackBasisCount);
  state.counters["max_result_abs_error"] = maxError;
  state.SetItemsProcessed(
      static_cast<std::int64_t>(state.iterations() * fixture.inputs.size()));
}

void recordCounters(
    benchmark::State& state,
    const EdgeEdgeTangentFixture& fixture,
    const double maxError)
{
  state.counters["samples"] = static_cast<double>(fixture.inputs.size());
  state.counters["fallback_bases"]
      = static_cast<double>(fixture.cpu.fallbackBasisCount);
  state.counters["max_result_abs_error"] = maxError;
  state.SetItemsProcessed(
      static_cast<std::int64_t>(state.iterations() * fixture.inputs.size()));
}

void recordCounters(
    benchmark::State& state,
    const PointEdgeTangentFixture& fixture,
    const double maxError)
{
  state.counters["samples"] = static_cast<double>(fixture.inputs.size());
  state.counters["fallback_bases"]
      = static_cast<double>(fixture.cpu.fallbackBasisCount);
  state.counters["max_result_abs_error"] = maxError;
  state.SetItemsProcessed(
      static_cast<std::int64_t>(state.iterations() * fixture.inputs.size()));
}

void recordCounters(
    benchmark::State& state,
    const PointPointTangentFixture& fixture,
    const double maxError)
{
  state.counters["samples"] = static_cast<double>(fixture.inputs.size());
  state.counters["fallback_bases"]
      = static_cast<double>(fixture.cpu.fallbackBasisCount);
  state.counters["max_result_abs_error"] = maxError;
  state.SetItemsProcessed(
      static_cast<std::int64_t>(state.iterations() * fixture.inputs.size()));
}

} // namespace

//==============================================================================
static void BM_Plan083BarrierFrictionLocalCpu(benchmark::State& state)
{
  const auto fixture = makeFixture(static_cast<int>(state.range(0)));
  CpuLocalResult result;

  for (auto _ : state) {
    evaluateCpu(fixture.inputs, result);
    benchmark::DoNotOptimize(result.barrierValues.data());
    benchmark::DoNotOptimize(result.frictionWorks.data());
  }

  recordCounters(state, fixture, 0.0);
}
BENCHMARK(BM_Plan083BarrierFrictionLocalCpu)
    ->Arg(4096)
    ->Arg(65536)
    ->UseRealTime();

//==============================================================================
static void BM_Plan083BarrierFrictionLocalCuda(benchmark::State& state)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    state.SkipWithError("CUDA runtime has no available device");
    return;
  }

  const auto fixture = makeFixture(static_cast<int>(state.range(0)));
  cuda::BarrierFrictionLocalResult result;

  for (auto _ : state) {
    cuda::evaluateBarrierFrictionLocalKernelsCuda(fixture.inputs, result);
    benchmark::DoNotOptimize(result.barrierValues.data());
    benchmark::DoNotOptimize(result.frictionWorks.data());
  }

  recordCounters(state, fixture, maxOutputError(fixture.cpu, result));
  state.counters["gpu_active_barriers"]
      = static_cast<double>(result.activeBarrierCount);
  state.counters["gpu_active_friction"]
      = static_cast<double>(result.activeFrictionCount);
  state.counters["gpu_dynamic_friction"]
      = static_cast<double>(result.dynamicFrictionCount);
  state.counters["host_setup_ns"] = result.timing.setupNs;
  state.counters["host_to_device_ns"] = result.timing.hostToDeviceNs;
  state.counters["kernel_ns"] = result.timing.kernelNs;
  state.counters["device_to_host_ns"] = result.timing.deviceToHostNs;
}
BENCHMARK(BM_Plan083BarrierFrictionLocalCuda)
    ->Arg(4096)
    ->Arg(65536)
    ->UseRealTime();

//==============================================================================
static void BM_Plan083PointTriangleBarrierGradientCpu(benchmark::State& state)
{
  const auto fixture
      = makePointTriangleFixture(static_cast<int>(state.range(0)));
  CpuPointTriangleResult result;

  for (auto _ : state) {
    evaluateCpu(fixture.inputs, result);
    benchmark::DoNotOptimize(result.barrierValues.data());
    benchmark::DoNotOptimize(result.barrierGradients.data());
  }

  recordCounters(state, fixture, 0.0);
}
BENCHMARK(BM_Plan083PointTriangleBarrierGradientCpu)
    ->Arg(4096)
    ->Arg(65536)
    ->UseRealTime();

//==============================================================================
static void BM_Plan083PointTriangleBarrierGradientCuda(benchmark::State& state)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    state.SkipWithError("CUDA runtime has no available device");
    return;
  }

  const auto fixture
      = makePointTriangleFixture(static_cast<int>(state.range(0)));
  cuda::PointTriangleBarrierGradientResult result;

  for (auto _ : state) {
    cuda::evaluatePointTriangleBarrierGradientsCuda(fixture.inputs, result);
    benchmark::DoNotOptimize(result.barrierValues.data());
    benchmark::DoNotOptimize(result.barrierGradients.data());
  }

  recordCounters(state, fixture, maxOutputError(fixture.cpu, result));
  state.counters["gpu_active_barriers"]
      = static_cast<double>(result.activeBarrierCount);
  state.counters["host_setup_ns"] = result.timing.setupNs;
  state.counters["host_to_device_ns"] = result.timing.hostToDeviceNs;
  state.counters["kernel_ns"] = result.timing.kernelNs;
  state.counters["device_to_host_ns"] = result.timing.deviceToHostNs;
}
BENCHMARK(BM_Plan083PointTriangleBarrierGradientCuda)
    ->Arg(4096)
    ->Arg(65536)
    ->UseRealTime();

//==============================================================================
static void BM_Plan083PointPointBarrierHessianCpu(benchmark::State& state)
{
  const auto fixture
      = makePointPointBarrierFixture(static_cast<int>(state.range(0)));
  CpuPointPointBarrierResult result;

  for (auto _ : state) {
    evaluateCpu(fixture.inputs, result);
    benchmark::DoNotOptimize(result.barrierValues.data());
    benchmark::DoNotOptimize(result.barrierHessians.data());
  }

  recordCounters(state, fixture, 0.0);
}
BENCHMARK(BM_Plan083PointPointBarrierHessianCpu)
    ->Arg(4096)
    ->Arg(65536)
    ->UseRealTime();

//==============================================================================
static void BM_Plan083PointPointBarrierHessianCuda(benchmark::State& state)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    state.SkipWithError("CUDA runtime has no available device");
    return;
  }

  const auto fixture
      = makePointPointBarrierFixture(static_cast<int>(state.range(0)));
  cuda::PointPointBarrierHessianResult result;

  for (auto _ : state) {
    cuda::evaluatePointPointBarrierHessiansCuda(fixture.inputs, result);
    benchmark::DoNotOptimize(result.barrierValues.data());
    benchmark::DoNotOptimize(result.barrierHessians.data());
  }

  recordCounters(state, fixture, maxOutputError(fixture.cpu, result));
  state.counters["gpu_active_barriers"]
      = static_cast<double>(result.activeBarrierCount);
  state.counters["host_setup_ns"] = result.timing.setupNs;
  state.counters["host_to_device_ns"] = result.timing.hostToDeviceNs;
  state.counters["kernel_ns"] = result.timing.kernelNs;
  state.counters["device_to_host_ns"] = result.timing.deviceToHostNs;
}
BENCHMARK(BM_Plan083PointPointBarrierHessianCuda)
    ->Arg(4096)
    ->Arg(65536)
    ->UseRealTime();

//==============================================================================
static void BM_Plan083PointTriangleTangentStencilCpu(benchmark::State& state)
{
  const auto fixture
      = makePointTriangleTangentFixture(static_cast<int>(state.range(0)));
  CpuPointTriangleTangentResult result;

  for (auto _ : state) {
    evaluateCpu(fixture.inputs, result);
    benchmark::DoNotOptimize(result.projectionValues.data());
  }

  recordCounters(state, fixture, 0.0);
}
BENCHMARK(BM_Plan083PointTriangleTangentStencilCpu)
    ->Arg(4096)
    ->Arg(65536)
    ->UseRealTime();

//==============================================================================
static void BM_Plan083PointTriangleTangentStencilCuda(benchmark::State& state)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    state.SkipWithError("CUDA runtime has no available device");
    return;
  }

  const auto fixture
      = makePointTriangleTangentFixture(static_cast<int>(state.range(0)));
  cuda::PointTriangleTangentStencilResult result;

  for (auto _ : state) {
    cuda::evaluatePointTriangleTangentStencilsCuda(fixture.inputs, result);
    benchmark::DoNotOptimize(result.projectionValues.data());
  }

  recordCounters(state, fixture, maxOutputError(fixture.cpu, result));
  state.counters["gpu_fallback_bases"]
      = static_cast<double>(result.fallbackBasisCount);
  state.counters["host_setup_ns"] = result.timing.setupNs;
  state.counters["host_to_device_ns"] = result.timing.hostToDeviceNs;
  state.counters["kernel_ns"] = result.timing.kernelNs;
  state.counters["device_to_host_ns"] = result.timing.deviceToHostNs;
}
BENCHMARK(BM_Plan083PointTriangleTangentStencilCuda)
    ->Arg(4096)
    ->Arg(65536)
    ->UseRealTime();

//==============================================================================
static void BM_Plan083EdgeEdgeTangentStencilCpu(benchmark::State& state)
{
  const auto fixture
      = makeEdgeEdgeTangentFixture(static_cast<int>(state.range(0)));
  CpuEdgeEdgeTangentResult result;

  for (auto _ : state) {
    evaluateCpu(fixture.inputs, result);
    benchmark::DoNotOptimize(result.projectionValues.data());
  }

  recordCounters(state, fixture, 0.0);
}
BENCHMARK(BM_Plan083EdgeEdgeTangentStencilCpu)
    ->Arg(4096)
    ->Arg(65536)
    ->UseRealTime();

//==============================================================================
static void BM_Plan083EdgeEdgeTangentStencilCuda(benchmark::State& state)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    state.SkipWithError("CUDA runtime has no available device");
    return;
  }

  const auto fixture
      = makeEdgeEdgeTangentFixture(static_cast<int>(state.range(0)));
  cuda::EdgeEdgeTangentStencilResult result;

  for (auto _ : state) {
    cuda::evaluateEdgeEdgeTangentStencilsCuda(fixture.inputs, result);
    benchmark::DoNotOptimize(result.projectionValues.data());
  }

  recordCounters(state, fixture, maxOutputError(fixture.cpu, result));
  state.counters["gpu_fallback_bases"]
      = static_cast<double>(result.fallbackBasisCount);
  state.counters["host_setup_ns"] = result.timing.setupNs;
  state.counters["host_to_device_ns"] = result.timing.hostToDeviceNs;
  state.counters["kernel_ns"] = result.timing.kernelNs;
  state.counters["device_to_host_ns"] = result.timing.deviceToHostNs;
}
BENCHMARK(BM_Plan083EdgeEdgeTangentStencilCuda)
    ->Arg(4096)
    ->Arg(65536)
    ->UseRealTime();

//==============================================================================
static void BM_Plan083PointEdgeTangentStencilCpu(benchmark::State& state)
{
  const auto fixture
      = makePointEdgeTangentFixture(static_cast<int>(state.range(0)));
  CpuPointEdgeTangentResult result;

  for (auto _ : state) {
    evaluateCpu(fixture.inputs, result);
    benchmark::DoNotOptimize(result.projectionValues.data());
  }

  recordCounters(state, fixture, 0.0);
}
BENCHMARK(BM_Plan083PointEdgeTangentStencilCpu)
    ->Arg(4096)
    ->Arg(65536)
    ->UseRealTime();

//==============================================================================
static void BM_Plan083PointEdgeTangentStencilCuda(benchmark::State& state)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    state.SkipWithError("CUDA runtime has no available device");
    return;
  }

  const auto fixture
      = makePointEdgeTangentFixture(static_cast<int>(state.range(0)));
  cuda::PointEdgeTangentStencilResult result;

  for (auto _ : state) {
    cuda::evaluatePointEdgeTangentStencilsCuda(fixture.inputs, result);
    benchmark::DoNotOptimize(result.projectionValues.data());
  }

  recordCounters(state, fixture, maxOutputError(fixture.cpu, result));
  state.counters["gpu_fallback_bases"]
      = static_cast<double>(result.fallbackBasisCount);
  state.counters["host_setup_ns"] = result.timing.setupNs;
  state.counters["host_to_device_ns"] = result.timing.hostToDeviceNs;
  state.counters["kernel_ns"] = result.timing.kernelNs;
  state.counters["device_to_host_ns"] = result.timing.deviceToHostNs;
}
BENCHMARK(BM_Plan083PointEdgeTangentStencilCuda)
    ->Arg(4096)
    ->Arg(65536)
    ->UseRealTime();

//==============================================================================
static void BM_Plan083PointPointTangentStencilCpu(benchmark::State& state)
{
  const auto fixture
      = makePointPointTangentFixture(static_cast<int>(state.range(0)));
  CpuPointPointTangentResult result;

  for (auto _ : state) {
    evaluateCpu(fixture.inputs, result);
    benchmark::DoNotOptimize(result.projectionValues.data());
  }

  recordCounters(state, fixture, 0.0);
}
BENCHMARK(BM_Plan083PointPointTangentStencilCpu)
    ->Arg(4096)
    ->Arg(65536)
    ->UseRealTime();

//==============================================================================
static void BM_Plan083PointPointTangentStencilCuda(benchmark::State& state)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    state.SkipWithError("CUDA runtime has no available device");
    return;
  }

  const auto fixture
      = makePointPointTangentFixture(static_cast<int>(state.range(0)));
  cuda::PointPointTangentStencilResult result;

  for (auto _ : state) {
    cuda::evaluatePointPointTangentStencilsCuda(fixture.inputs, result);
    benchmark::DoNotOptimize(result.projectionValues.data());
  }

  recordCounters(state, fixture, maxOutputError(fixture.cpu, result));
  state.counters["gpu_fallback_bases"]
      = static_cast<double>(result.fallbackBasisCount);
  state.counters["host_setup_ns"] = result.timing.setupNs;
  state.counters["host_to_device_ns"] = result.timing.hostToDeviceNs;
  state.counters["kernel_ns"] = result.timing.kernelNs;
  state.counters["device_to_host_ns"] = result.timing.deviceToHostNs;
}
BENCHMARK(BM_Plan083PointPointTangentStencilCuda)
    ->Arg(4096)
    ->Arg(65536)
    ->UseRealTime();

BENCHMARK_MAIN();
