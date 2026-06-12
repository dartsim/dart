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

#pragma once

#include <vector>

#include <cstddef>
#include <cstdint>

namespace dart::simulation::compute::cuda {

struct BarrierFrictionLocalInput
{
  double squaredDistance = 0.0;
  double squaredActivationDistance = 1.0;
  double stiffness = 1.0;
  double tangentialDisplacementNorm = 0.0;
  double frictionWeight = 1.0;
  double staticFrictionDisplacement = 1.0;
};

struct PointTriangleBarrierInput
{
  double point[3] = {0.0, 0.0, 0.0};
  double triangleA[3] = {0.0, 0.0, 0.0};
  double triangleB[3] = {0.0, 0.0, 0.0};
  double triangleC[3] = {0.0, 0.0, 0.0};
  double squaredActivationDistance = 1.0;
  double stiffness = 1.0;
};

struct PointTriangleTangentInput
{
  double point[3] = {0.0, 0.0, 0.0};
  double triangleA[3] = {0.0, 0.0, 0.0};
  double triangleB[3] = {0.0, 0.0, 0.0};
  double triangleC[3] = {0.0, 0.0, 0.0};
};

struct BarrierFrictionLocalTiming
{
  double setupNs = 0.0;
  double hostToDeviceNs = 0.0;
  double kernelNs = 0.0;
  double deviceToHostNs = 0.0;
};

struct BarrierFrictionLocalResult
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
  BarrierFrictionLocalTiming timing;
};

struct PointTriangleBarrierGradientResult
{
  std::vector<double> squaredDistances;
  std::vector<double> barrierValues;
  std::vector<double> barrierGradients;
  std::vector<std::uint8_t> activeBarriers;
  std::size_t activeBarrierCount = 0;
  double maxBarrierValue = 0.0;
  BarrierFrictionLocalTiming timing;
};

struct PointTriangleTangentStencilResult
{
  std::vector<double> basisValues;
  std::vector<double> coordinates;
  std::vector<double> projectionValues;
  std::vector<std::uint8_t> fallbackBases;
  std::size_t fallbackBasisCount = 0;
  BarrierFrictionLocalTiming timing;
};

/// Evaluate private local barrier and friction scalar kernels on CUDA.
///
/// This packet covers the clamped-log scalar barrier and smoothed Coulomb
/// friction norm/work contracts. It intentionally does not cover primitive
/// distance gradients, tangent basis construction, Hessian assembly, or a
/// public GPU backend.
void evaluateBarrierFrictionLocalKernelsCuda(
    const std::vector<BarrierFrictionLocalInput>& inputs,
    BarrierFrictionLocalResult& result);

/// Evaluate private point-triangle barrier values and gradients on CUDA.
///
/// This packet extends local-kernel evidence to primitive distance gradients.
/// It intentionally does not cover tangent basis construction, Hessian
/// assembly, PSD coupling, runtime contact rows, or a public GPU backend.
void evaluatePointTriangleBarrierGradientsCuda(
    const std::vector<PointTriangleBarrierInput>& inputs,
    PointTriangleBarrierGradientResult& result);

/// Evaluate private point-triangle tangent stencils on CUDA.
///
/// This packet extends local-kernel evidence to tangent basis construction and
/// projection assembly for the point-triangle primitive family. It
/// intentionally does not cover Hessian assembly, PSD coupling, runtime contact
/// rows, or a public GPU backend.
void evaluatePointTriangleTangentStencilsCuda(
    const std::vector<PointTriangleTangentInput>& inputs,
    PointTriangleTangentStencilResult& result);

} // namespace dart::simulation::compute::cuda
