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

#include <dart/simulation/common/exceptions.hpp>
#include <dart/simulation/detail/newton_barrier/barrier_kernel.hpp>
#include <dart/simulation/detail/newton_barrier/friction_kernel.hpp>
#include <dart/simulation/detail/newton_barrier/tangent_stencil.hpp>

#include <dart/simulation/compute/cuda/barrier_friction_kernel_cuda.cuh>
#include <dart/simulation/compute/cuda/cuda_runtime.cuh>
#include <dart/simulation/compute/cuda/deformable_psd_projection_cuda.cuh>
#include <gtest/gtest.h>

#include <algorithm>
#include <limits>
#include <vector>

namespace cuda = dart::simulation::compute::cuda;
namespace nb = dart::simulation::detail::newton_barrier;

namespace {

std::vector<cuda::BarrierFrictionLocalInput> makeFixture()
{
  return {
      {
          .squaredDistance = 0.25,
          .squaredActivationDistance = 1.0,
          .stiffness = 2.0,
          .tangentialDisplacementNorm = 0.05,
          .frictionWeight = 4.0,
          .staticFrictionDisplacement = 0.2,
      },
      {
          .squaredDistance = 1.25,
          .squaredActivationDistance = 1.0,
          .stiffness = 2.0,
          .tangentialDisplacementNorm = 0.4,
          .frictionWeight = 4.0,
          .staticFrictionDisplacement = 0.2,
      },
      {
          .squaredDistance = 0.64,
          .squaredActivationDistance = 1.0,
          .stiffness = 3.0,
          .tangentialDisplacementNorm = 0.0,
          .frictionWeight = 5.0,
          .staticFrictionDisplacement = 0.25,
      },
  };
}

void writeVec3(double destination[3], const Eigen::Vector3d& source)
{
  destination[0] = source.x();
  destination[1] = source.y();
  destination[2] = source.z();
}

cuda::PointTriangleBarrierInput makePointTriangleInput(
    const Eigen::Vector3d& p,
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c,
    const double squaredActivationDistance,
    const double stiffness)
{
  cuda::PointTriangleBarrierInput input;
  writeVec3(input.point, p);
  writeVec3(input.triangleA, a);
  writeVec3(input.triangleB, b);
  writeVec3(input.triangleC, c);
  input.squaredActivationDistance = squaredActivationDistance;
  input.stiffness = stiffness;
  return input;
}

cuda::PointTriangleTangentInput makePointTriangleTangentInput(
    const Eigen::Vector3d& p,
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c)
{
  cuda::PointTriangleTangentInput input;
  writeVec3(input.point, p);
  writeVec3(input.triangleA, a);
  writeVec3(input.triangleB, b);
  writeVec3(input.triangleC, c);
  return input;
}

cuda::EdgeEdgeTangentInput makeEdgeEdgeTangentInput(
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c,
    const Eigen::Vector3d& d)
{
  cuda::EdgeEdgeTangentInput input;
  writeVec3(input.edgeA0, a);
  writeVec3(input.edgeA1, b);
  writeVec3(input.edgeB0, c);
  writeVec3(input.edgeB1, d);
  return input;
}

cuda::PointEdgeTangentInput makePointEdgeTangentInput(
    const Eigen::Vector3d& p,
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b)
{
  cuda::PointEdgeTangentInput input;
  writeVec3(input.point, p);
  writeVec3(input.edgeA, a);
  writeVec3(input.edgeB, b);
  return input;
}

cuda::PointPointTangentInput makePointPointTangentInput(
    const Eigen::Vector3d& a, const Eigen::Vector3d& b)
{
  cuda::PointPointTangentInput input;
  writeVec3(input.pointA, a);
  writeVec3(input.pointB, b);
  return input;
}

cuda::PointPointBarrierInput makePointPointBarrierInput(
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const double squaredActivationDistance,
    const double stiffness)
{
  cuda::PointPointBarrierInput input;
  writeVec3(input.pointA, a);
  writeVec3(input.pointB, b);
  input.squaredActivationDistance = squaredActivationDistance;
  input.stiffness = stiffness;
  return input;
}

cuda::PointEdgeBarrierInput makePointEdgeBarrierInput(
    const Eigen::Vector3d& p,
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const double squaredActivationDistance,
    const double stiffness)
{
  cuda::PointEdgeBarrierInput input;
  writeVec3(input.point, p);
  writeVec3(input.edgeA, a);
  writeVec3(input.edgeB, b);
  input.squaredActivationDistance = squaredActivationDistance;
  input.stiffness = stiffness;
  return input;
}

cuda::EdgeEdgeBarrierInput makeEdgeEdgeBarrierInput(
    const Eigen::Vector3d& a0,
    const Eigen::Vector3d& a1,
    const Eigen::Vector3d& b0,
    const Eigen::Vector3d& b1,
    const double squaredActivationDistance,
    const double stiffness)
{
  cuda::EdgeEdgeBarrierInput input;
  writeVec3(input.edgeA0, a0);
  writeVec3(input.edgeA1, a1);
  writeVec3(input.edgeB0, b0);
  writeVec3(input.edgeB1, b1);
  input.squaredActivationDistance = squaredActivationDistance;
  input.stiffness = stiffness;
  return input;
}

std::vector<cuda::PointTriangleBarrierInput> makePointTriangleFixture()
{
  return {
      makePointTriangleInput(
          Eigen::Vector3d(0.25, 0.25, 0.1),
          Eigen::Vector3d(0.0, 0.0, 0.0),
          Eigen::Vector3d(1.0, 0.0, 0.0),
          Eigen::Vector3d(0.0, 1.0, 0.0),
          0.25,
          2.0),
      makePointTriangleInput(
          Eigen::Vector3d(0.5, -0.1, 0.05),
          Eigen::Vector3d(0.0, 0.0, 0.0),
          Eigen::Vector3d(1.0, 0.0, 0.0),
          Eigen::Vector3d(0.0, 1.0, 0.0),
          0.25,
          1.5),
      makePointTriangleInput(
          Eigen::Vector3d(0.2, 0.2, 2.0),
          Eigen::Vector3d(0.0, 0.0, 0.0),
          Eigen::Vector3d(1.0, 0.0, 0.0),
          Eigen::Vector3d(0.0, 1.0, 0.0),
          0.25,
          3.0),
      makePointTriangleInput(
          Eigen::Vector3d(0.25e-8, 0.5e-8, 0.05),
          Eigen::Vector3d(0.0, 0.0, 0.0),
          Eigen::Vector3d(1e-8, 0.0, 0.0),
          Eigen::Vector3d(0.0, 1e-8, 0.0),
          0.25,
          2.5),
  };
}

std::vector<cuda::PointTriangleBarrierInput> makePointTriangleHessianFixture()
{
  return {
      makePointTriangleInput(
          Eigen::Vector3d(0.25, 0.25, 0.1),
          Eigen::Vector3d(0.0, 0.0, 0.0),
          Eigen::Vector3d(1.0, 0.0, 0.0),
          Eigen::Vector3d(0.0, 1.0, 0.0),
          0.25,
          2.0),
      makePointTriangleInput(
          Eigen::Vector3d(0.5, -0.1, 0.05),
          Eigen::Vector3d(0.0, 0.0, 0.0),
          Eigen::Vector3d(1.0, 0.0, 0.0),
          Eigen::Vector3d(0.0, 1.0, 0.0),
          0.25,
          1.5),
      makePointTriangleInput(
          Eigen::Vector3d(-0.1, -0.05, 0.04),
          Eigen::Vector3d(0.0, 0.0, 0.0),
          Eigen::Vector3d(1.0, 0.0, 0.0),
          Eigen::Vector3d(0.0, 1.0, 0.0),
          0.25,
          2.5),
      makePointTriangleInput(
          Eigen::Vector3d(0.2, 0.2, 2.0),
          Eigen::Vector3d(0.0, 0.0, 0.0),
          Eigen::Vector3d(1.0, 0.0, 0.0),
          Eigen::Vector3d(0.0, 1.0, 0.0),
          0.25,
          3.0),
  };
}

std::vector<cuda::PointTriangleTangentInput> makePointTriangleTangentFixture()
{
  return {
      makePointTriangleTangentInput(
          Eigen::Vector3d(0.25, 0.25, 0.1),
          Eigen::Vector3d(0.0, 0.0, 0.0),
          Eigen::Vector3d(1.0, 0.0, 0.0),
          Eigen::Vector3d(0.0, 1.0, 0.0)),
      makePointTriangleTangentInput(
          Eigen::Vector3d(0.5, -0.1, 0.05),
          Eigen::Vector3d(0.0, 0.0, 0.0),
          Eigen::Vector3d(1.5, 0.25, 0.1),
          Eigen::Vector3d(-0.2, 1.25, 0.3)),
      makePointTriangleTangentInput(
          Eigen::Vector3d(-0.1, 0.4, 0.3),
          Eigen::Vector3d(0.1, -0.2, 0.3),
          Eigen::Vector3d(1.1, 0.2, 0.5),
          Eigen::Vector3d(0.25, 1.0, 0.9)),
  };
}

std::vector<cuda::EdgeEdgeTangentInput> makeEdgeEdgeTangentFixture()
{
  return {
      makeEdgeEdgeTangentInput(
          Eigen::Vector3d(0.0, 0.0, 0.0),
          Eigen::Vector3d(1.0, 0.0, 0.0),
          Eigen::Vector3d(0.25, 0.2, 0.4),
          Eigen::Vector3d(0.25, 1.2, 0.6)),
      makeEdgeEdgeTangentInput(
          Eigen::Vector3d(-0.2, 0.1, 0.3),
          Eigen::Vector3d(1.2, 0.4, 0.5),
          Eigen::Vector3d(0.4, -0.5, 0.2),
          Eigen::Vector3d(0.8, 0.9, 1.1)),
      makeEdgeEdgeTangentInput(
          Eigen::Vector3d(0.1, -0.2, 0.4),
          Eigen::Vector3d(0.9, 0.3, 0.8),
          Eigen::Vector3d(-0.3, 0.7, 0.1),
          Eigen::Vector3d(0.6, 1.1, 0.9)),
  };
}

std::vector<cuda::PointEdgeTangentInput> makePointEdgeTangentFixture()
{
  return {
      makePointEdgeTangentInput(
          Eigen::Vector3d(0.2, 0.3, 0.4),
          Eigen::Vector3d(0.0, 0.0, 0.0),
          Eigen::Vector3d(1.0, 0.1, 0.2)),
      makePointEdgeTangentInput(
          Eigen::Vector3d(-0.1, 0.6, 0.2),
          Eigen::Vector3d(0.3, -0.2, 0.1),
          Eigen::Vector3d(1.2, 0.4, 0.6)),
      makePointEdgeTangentInput(
          Eigen::Vector3d(0.7, -0.3, 0.8),
          Eigen::Vector3d(-0.4, 0.2, 0.0),
          Eigen::Vector3d(0.6, 0.8, 0.9)),
  };
}

cuda::PointEdgeTangentInput makeGeneratedPointEdgeTangentInput(const int i)
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
  return makePointEdgeTangentInput(p, a, b);
}

std::vector<cuda::PointPointTangentInput> makePointPointTangentFixture()
{
  return {
      makePointPointTangentInput(
          Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(0.2, 0.4, 1.0)),
      makePointPointTangentInput(
          Eigen::Vector3d(-0.3, 0.2, 0.5), Eigen::Vector3d(1.0, 0.1, 0.7)),
      makePointPointTangentInput(
          Eigen::Vector3d(0.4, -0.5, 0.3), Eigen::Vector3d(-0.2, 0.9, 0.6)),
  };
}

cuda::PointPointTangentInput makeGeneratedPointPointTangentInput(const int i)
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
  return makePointPointTangentInput(a, b);
}

std::vector<cuda::PointPointBarrierInput> makePointPointBarrierFixture()
{
  return {
      makePointPointBarrierInput(
          Eigen::Vector3d(0.0, 0.0, 0.0),
          Eigen::Vector3d(0.1, 0.2, 0.15),
          0.25,
          2.0),
      makePointPointBarrierInput(
          Eigen::Vector3d(-0.3, 0.2, 0.5),
          Eigen::Vector3d(-0.2, 0.3, 0.55),
          0.16,
          1.5),
      makePointPointBarrierInput(
          Eigen::Vector3d(0.4, -0.5, 0.3),
          Eigen::Vector3d(1.4, 0.5, 1.3),
          0.25,
          3.0),
  };
}

std::vector<cuda::PointEdgeBarrierInput> makePointEdgeBarrierFixture()
{
  return {
      makePointEdgeBarrierInput(
          Eigen::Vector3d(0.2, 0.25, 0.2),
          Eigen::Vector3d(0.0, 0.0, 0.0),
          Eigen::Vector3d(1.0, 0.1, 0.2),
          0.25,
          2.0),
      makePointEdgeBarrierInput(
          Eigen::Vector3d(-0.08, 0.04, 0.05),
          Eigen::Vector3d(0.0, 0.0, 0.0),
          Eigen::Vector3d(1.0, 0.2, 0.1),
          0.16,
          1.5),
      makePointEdgeBarrierInput(
          Eigen::Vector3d(1.12, 0.25, 0.18),
          Eigen::Vector3d(0.0, 0.0, 0.0),
          Eigen::Vector3d(1.0, 0.2, 0.1),
          0.16,
          1.75),
      makePointEdgeBarrierInput(
          Eigen::Vector3d(0.3, 1.5, 1.2),
          Eigen::Vector3d(-0.2, 0.1, 0.0),
          Eigen::Vector3d(0.8, 0.4, 0.2),
          0.25,
          3.0),
  };
}

std::vector<cuda::EdgeEdgeBarrierInput> makeEdgeEdgeBarrierFixture()
{
  return {
      makeEdgeEdgeBarrierInput(
          Eigen::Vector3d(0.0, 0.0, 0.0),
          Eigen::Vector3d(1.0, 0.1, 0.0),
          Eigen::Vector3d(0.2, 0.25, 0.1),
          Eigen::Vector3d(0.2, 1.25, 0.35),
          0.25,
          2.0),
      makeEdgeEdgeBarrierInput(
          Eigen::Vector3d(0.0, 0.0, 0.0),
          Eigen::Vector3d(1.0, 0.0, 0.0),
          Eigen::Vector3d(-0.08, 0.04, 0.05),
          Eigen::Vector3d(-0.08, 0.8, 0.15),
          0.16,
          1.5),
      makeEdgeEdgeBarrierInput(
          Eigen::Vector3d(0.0, 0.0, 0.0),
          Eigen::Vector3d(1.0, 0.2, 0.1),
          Eigen::Vector3d(1.12, 0.25, 0.18),
          Eigen::Vector3d(1.12, 0.9, 0.45),
          0.16,
          1.75),
      makeEdgeEdgeBarrierInput(
          Eigen::Vector3d(-0.2, 0.1, 0.0),
          Eigen::Vector3d(0.8, 0.4, 0.2),
          Eigen::Vector3d(0.3, 1.5, 1.2),
          Eigen::Vector3d(0.9, 1.8, 1.5),
          0.25,
          3.0),
  };
}

cuda::PointPointBarrierInput makeGeneratedPointPointBarrierInput(const int i)
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
  return makePointPointBarrierInput(
      a, a + offset, 0.25, 1.0 + 0.125 * static_cast<double>(i % 13));
}

cuda::PointEdgeBarrierInput makeGeneratedPointEdgeBarrierInput(const int i)
{
  const bool inactive = (i % 11) == 0;
  const double column = static_cast<double>(i % 257) / 257.0;
  const double row = static_cast<double>((i / 257) % 251) / 251.0;
  const Eigen::Vector3d a(
      -0.2 + 0.15 * row,
      0.05 * static_cast<double>(i % 11),
      0.02 * static_cast<double>(i % 17));
  const Eigen::Vector3d edge(
      1.0 + 0.001 * static_cast<double>(i % 13),
      0.25 + 0.1 * row,
      0.15 + 0.01 * static_cast<double>(i % 7));
  Eigen::Vector3d normal = edge.cross(Eigen::Vector3d::UnitZ());
  if (normal.squaredNorm() < 1e-12) {
    normal = edge.cross(Eigen::Vector3d::UnitY());
  }
  normal.normalize();

  const double t = 0.15 + 0.7 * column;
  const double offset = inactive ? 0.75 : 0.03 + 0.0005 * (i % 23);
  const Eigen::Vector3d p = a + t * edge + offset * normal;
  return makePointEdgeBarrierInput(
      p, a, a + edge, 0.25, 1.0 + 0.125 * static_cast<double>(i % 13));
}

cuda::EdgeEdgeBarrierInput makeGeneratedEdgeEdgeBarrierInput(const int i)
{
  const bool inactive = (i % 11) == 0;
  const double column = static_cast<double>(i % 257) / 257.0;
  const double row = static_cast<double>((i / 257) % 251) / 251.0;
  const Eigen::Vector3d a0(
      -0.2 + 0.15 * row,
      0.05 * static_cast<double>(i % 11),
      0.02 * static_cast<double>(i % 17));
  const Eigen::Vector3d edgeA(
      1.0 + 0.001 * static_cast<double>(i % 13),
      0.25 + 0.1 * row,
      0.15 + 0.01 * static_cast<double>(i % 7));
  Eigen::Vector3d normal = edgeA.cross(Eigen::Vector3d::UnitZ());
  if (normal.squaredNorm() < 1e-12) {
    normal = edgeA.cross(Eigen::Vector3d::UnitY());
  }
  normal.normalize();

  const double s = 0.15 + 0.7 * column;
  const double offset = inactive ? 0.75 : 0.03 + 0.0005 * (i % 23);
  const Eigen::Vector3d b0 = a0 + s * edgeA + offset * normal;
  const Eigen::Vector3d edgeB(
      0.02 * static_cast<double>((i + 3) % 7),
      0.75 + 0.002 * static_cast<double>(i % 17),
      0.2 + 0.001 * static_cast<double>(i % 19));
  return makeEdgeEdgeBarrierInput(
      a0,
      a0 + edgeA,
      b0,
      b0 + edgeB,
      0.25,
      1.0 + 0.125 * static_cast<double>(i % 13));
}

Eigen::Vector3d readVec3(const double values[3])
{
  return {values[0], values[1], values[2]};
}

} // namespace

//==============================================================================
TEST(BarrierFrictionKernelCuda, MatchesCpuScalarContracts)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  const auto inputs = makeFixture();

  cuda::BarrierFrictionLocalResult result;
  cuda::evaluateBarrierFrictionLocalKernelsCuda(inputs, result);

  ASSERT_EQ(result.barrierValues.size(), inputs.size());
  ASSERT_EQ(result.frictionWorks.size(), inputs.size());
  ASSERT_EQ(result.activeBarriers.size(), inputs.size());
  ASSERT_EQ(result.activeFrictions.size(), inputs.size());
  EXPECT_EQ(result.activeBarrierCount, 2u);
  EXPECT_EQ(result.activeFrictionCount, 3u);
  EXPECT_EQ(result.dynamicFrictionCount, 1u);

  for (std::size_t i = 0; i < inputs.size(); ++i) {
    const auto& input = inputs[i];
    const double stiffness = std::max(0.0, input.stiffness);
    const auto barrier = nb::c2ClampedLogBarrier(
        input.squaredDistance, input.squaredActivationDistance);
    const auto friction = nb::smoothFrictionNorm(
        input.tangentialDisplacementNorm, input.staticFrictionDisplacement);
    const auto work = nb::frictionWorkContribution(
        input.tangentialDisplacementNorm,
        input.frictionWeight,
        input.staticFrictionDisplacement);

    EXPECT_EQ(result.activeBarriers[i] != 0u, barrier.active && stiffness > 0.0)
        << i;
    EXPECT_EQ(result.activeFrictions[i] != 0u, work.active) << i;
    EXPECT_EQ(result.dynamicFrictions[i] != 0u, friction.dynamicBranch) << i;

    EXPECT_NEAR(result.barrierValues[i], stiffness * barrier.value, 1e-12) << i;
    EXPECT_NEAR(
        result.barrierFirstDerivatives[i],
        stiffness * barrier.firstDerivative,
        1e-12)
        << i;
    EXPECT_NEAR(
        result.barrierSecondDerivatives[i],
        stiffness * barrier.secondDerivative,
        1e-11)
        << i;
    EXPECT_NEAR(
        result.frictionValues[i], input.frictionWeight * friction.value, 1e-12)
        << i;
    EXPECT_NEAR(result.frictionWorks[i], work.work, 1e-12) << i;
    EXPECT_NEAR(
        result.frictionFirstDerivatives[i], friction.firstDerivative, 1e-12)
        << i;
    EXPECT_NEAR(
        result.frictionSecondDerivatives[i], friction.secondDerivative, 1e-12)
        << i;
  }
}

//==============================================================================
TEST(BarrierFrictionKernelCuda, MatchesCpuPointTriangleBarrierGradients)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  const auto inputs = makePointTriangleFixture();

  cuda::PointTriangleBarrierGradientResult result;
  cuda::evaluatePointTriangleBarrierGradientsCuda(inputs, result);

  ASSERT_EQ(result.squaredDistances.size(), inputs.size());
  ASSERT_EQ(result.barrierValues.size(), inputs.size());
  ASSERT_EQ(result.barrierGradients.size(), 12 * inputs.size());
  ASSERT_EQ(result.activeBarriers.size(), inputs.size());
  EXPECT_EQ(result.activeBarrierCount, 3u);

  for (std::size_t i = 0; i < inputs.size(); ++i) {
    const auto& input = inputs[i];
    const auto expected = nb::pointTriangleBarrier(
        readVec3(input.point),
        readVec3(input.triangleA),
        readVec3(input.triangleB),
        readVec3(input.triangleC),
        input.squaredActivationDistance,
        input.stiffness);

    EXPECT_EQ(result.activeBarriers[i] != 0u, expected.active) << i;
    EXPECT_NEAR(result.squaredDistances[i], expected.squaredDistance, 1e-12)
        << i;
    EXPECT_NEAR(result.barrierValues[i], expected.value, 1e-12) << i;
    for (int entry = 0; entry < 12; ++entry) {
      EXPECT_NEAR(
          result.barrierGradients[12 * i + static_cast<std::size_t>(entry)],
          expected.gradient[entry],
          1e-10)
          << "fixture=" << i << " entry=" << entry;
    }
  }
}

//==============================================================================
TEST(BarrierFrictionKernelCuda, MatchesCpuPointTriangleBarrierHessians)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  const auto inputs = makePointTriangleHessianFixture();

  cuda::PointTriangleBarrierHessianResult result;
  cuda::evaluatePointTriangleBarrierHessiansCuda(inputs, result);

  ASSERT_EQ(result.squaredDistances.size(), inputs.size());
  ASSERT_EQ(result.barrierValues.size(), inputs.size());
  ASSERT_EQ(result.barrierGradients.size(), 12 * inputs.size());
  ASSERT_EQ(result.barrierHessians.size(), 144 * inputs.size());
  ASSERT_EQ(result.activeBarriers.size(), inputs.size());
  EXPECT_EQ(result.activeBarrierCount, 3u);

  for (std::size_t i = 0; i < inputs.size(); ++i) {
    const auto& input = inputs[i];
    const auto expected = nb::pointTriangleBarrier(
        readVec3(input.point),
        readVec3(input.triangleA),
        readVec3(input.triangleB),
        readVec3(input.triangleC),
        input.squaredActivationDistance,
        input.stiffness);

    EXPECT_EQ(result.activeBarriers[i] != 0u, expected.active) << i;
    EXPECT_NEAR(result.squaredDistances[i], expected.squaredDistance, 1e-12)
        << i;
    EXPECT_NEAR(result.barrierValues[i], expected.value, 1e-12) << i;
    for (int entry = 0; entry < 12; ++entry) {
      EXPECT_NEAR(
          result.barrierGradients[12 * i + static_cast<std::size_t>(entry)],
          expected.gradient[entry],
          1e-10)
          << "fixture=" << i << " gradient entry=" << entry;
    }
    for (int row = 0; row < 12; ++row) {
      for (int col = 0; col < 12; ++col) {
        EXPECT_NEAR(
            result.barrierHessians
                [144 * i + static_cast<std::size_t>(12 * row + col)],
            expected.hessian(row, col),
            1e-8)
            << "fixture=" << i << " hessian row=" << row << " col=" << col;
      }
    }
  }
}

//==============================================================================
TEST(BarrierFrictionKernelCuda, ProjectsPointTriangleBarrierHessiansToPsd)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  const auto inputs = makePointTriangleHessianFixture();

  std::vector<double> expected(144 * inputs.size(), 0.0);
  for (std::size_t i = 0; i < inputs.size(); ++i) {
    const auto& input = inputs[i];
    const auto barrier = nb::pointTriangleBarrier(
        readVec3(input.point),
        readVec3(input.triangleA),
        readVec3(input.triangleB),
        readVec3(input.triangleC),
        input.squaredActivationDistance,
        input.stiffness);
    for (int row = 0; row < 12; ++row) {
      for (int col = 0; col < 12; ++col) {
        expected[144 * i + static_cast<std::size_t>(12 * row + col)]
            = barrier.hessian(row, col);
      }
    }
  }
  cuda::projectSymmetricBlocksToPsdReference(expected, 12, inputs.size());

  cuda::PointTriangleBarrierHessianResult result;
  cuda::evaluatePointTriangleBarrierHessiansCuda(inputs, result);
  cuda::projectSymmetricBlocksToPsdCuda(
      result.barrierHessians, 12, inputs.size());

  ASSERT_EQ(result.barrierHessians.size(), expected.size());
  for (std::size_t entry = 0; entry < expected.size(); ++entry) {
    EXPECT_NEAR(result.barrierHessians[entry], expected[entry], 1e-7)
        << "entry=" << entry;
  }
}

//==============================================================================
TEST(BarrierFrictionKernelCuda, MatchesCpuPointPointBarrierHessians)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  auto inputs = makePointPointBarrierFixture();
  for (int i = 0; i < 4096; ++i) {
    inputs.push_back(makeGeneratedPointPointBarrierInput(i));
  }

  cuda::PointPointBarrierHessianResult result;
  cuda::evaluatePointPointBarrierHessiansCuda(inputs, result);

  ASSERT_EQ(result.squaredDistances.size(), inputs.size());
  ASSERT_EQ(result.barrierValues.size(), inputs.size());
  ASSERT_EQ(result.barrierGradients.size(), 6 * inputs.size());
  ASSERT_EQ(result.barrierHessians.size(), 36 * inputs.size());
  ASSERT_EQ(result.activeBarriers.size(), inputs.size());

  std::size_t activeCount = 0;
  for (std::size_t i = 0; i < inputs.size(); ++i) {
    const auto& input = inputs[i];
    const auto expected = nb::pointPointBarrier(
        readVec3(input.pointA),
        readVec3(input.pointB),
        input.squaredActivationDistance,
        input.stiffness);

    activeCount += expected.active ? 1u : 0u;
    EXPECT_EQ(result.activeBarriers[i] != 0u, expected.active) << i;
    EXPECT_NEAR(result.squaredDistances[i], expected.squaredDistance, 1e-12)
        << i;
    EXPECT_NEAR(result.barrierValues[i], expected.value, 1e-12) << i;
    for (int entry = 0; entry < 6; ++entry) {
      EXPECT_NEAR(
          result.barrierGradients[6 * i + static_cast<std::size_t>(entry)],
          expected.gradient[entry],
          1e-10)
          << "fixture=" << i << " gradient entry=" << entry;
    }
    for (int row = 0; row < 6; ++row) {
      for (int col = 0; col < 6; ++col) {
        EXPECT_NEAR(
            result.barrierHessians
                [36 * i + static_cast<std::size_t>(6 * row + col)],
            expected.hessian(row, col),
            1e-9)
            << "fixture=" << i << " hessian row=" << row << " col=" << col;
      }
    }
  }
  EXPECT_EQ(result.activeBarrierCount, activeCount);
}

//==============================================================================
TEST(BarrierFrictionKernelCuda, ProjectsPointPointBarrierHessiansToPsd)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  auto inputs = makePointPointBarrierFixture();
  for (int i = 0; i < 4096; ++i) {
    inputs.push_back(makeGeneratedPointPointBarrierInput(i));
  }

  std::vector<double> expected(36 * inputs.size(), 0.0);
  for (std::size_t i = 0; i < inputs.size(); ++i) {
    const auto& input = inputs[i];
    const auto barrier = nb::pointPointBarrier(
        readVec3(input.pointA),
        readVec3(input.pointB),
        input.squaredActivationDistance,
        input.stiffness);
    for (int row = 0; row < 6; ++row) {
      for (int col = 0; col < 6; ++col) {
        expected[36 * i + static_cast<std::size_t>(6 * row + col)]
            = barrier.hessian(row, col);
      }
    }
  }
  cuda::projectSymmetricBlocksToPsdReference(expected, 6, inputs.size());

  cuda::PointPointBarrierHessianResult result;
  cuda::evaluatePointPointBarrierHessiansCuda(inputs, result);
  cuda::projectSymmetricBlocksToPsdCuda(
      result.barrierHessians, 6, inputs.size());

  ASSERT_EQ(result.barrierHessians.size(), expected.size());
  for (std::size_t entry = 0; entry < expected.size(); ++entry) {
    EXPECT_NEAR(result.barrierHessians[entry], expected[entry], 1e-7)
        << "entry=" << entry;
  }
}

//==============================================================================
TEST(BarrierFrictionKernelCuda, MatchesCpuPointEdgeBarrierHessians)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  auto inputs = makePointEdgeBarrierFixture();
  for (int i = 0; i < 4096; ++i) {
    inputs.push_back(makeGeneratedPointEdgeBarrierInput(i));
  }

  cuda::PointEdgeBarrierHessianResult result;
  cuda::evaluatePointEdgeBarrierHessiansCuda(inputs, result);

  ASSERT_EQ(result.squaredDistances.size(), inputs.size());
  ASSERT_EQ(result.barrierValues.size(), inputs.size());
  ASSERT_EQ(result.barrierGradients.size(), 9 * inputs.size());
  ASSERT_EQ(result.barrierHessians.size(), 81 * inputs.size());
  ASSERT_EQ(result.activeBarriers.size(), inputs.size());

  std::size_t activeCount = 0;
  for (std::size_t i = 0; i < inputs.size(); ++i) {
    const auto& input = inputs[i];
    const auto expected = nb::pointEdgeBarrier(
        readVec3(input.point),
        readVec3(input.edgeA),
        readVec3(input.edgeB),
        input.squaredActivationDistance,
        input.stiffness);

    activeCount += expected.active ? 1u : 0u;
    EXPECT_EQ(result.activeBarriers[i] != 0u, expected.active) << i;
    EXPECT_NEAR(result.squaredDistances[i], expected.squaredDistance, 1e-12)
        << i;
    EXPECT_NEAR(result.barrierValues[i], expected.value, 1e-12) << i;
    for (int entry = 0; entry < 9; ++entry) {
      EXPECT_NEAR(
          result.barrierGradients[9 * i + static_cast<std::size_t>(entry)],
          expected.gradient[entry],
          1e-10)
          << "fixture=" << i << " gradient entry=" << entry;
    }
    for (int row = 0; row < 9; ++row) {
      for (int col = 0; col < 9; ++col) {
        EXPECT_NEAR(
            result.barrierHessians
                [81 * i + static_cast<std::size_t>(9 * row + col)],
            expected.hessian(row, col),
            1e-8)
            << "fixture=" << i << " hessian row=" << row << " col=" << col;
      }
    }
  }
  EXPECT_EQ(result.activeBarrierCount, activeCount);
}

//==============================================================================
TEST(BarrierFrictionKernelCuda, ProjectsPointEdgeBarrierHessiansToPsd)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  auto inputs = makePointEdgeBarrierFixture();
  for (int i = 0; i < 4096; ++i) {
    inputs.push_back(makeGeneratedPointEdgeBarrierInput(i));
  }

  std::vector<double> expected(81 * inputs.size(), 0.0);
  for (std::size_t i = 0; i < inputs.size(); ++i) {
    const auto& input = inputs[i];
    const auto barrier = nb::pointEdgeBarrier(
        readVec3(input.point),
        readVec3(input.edgeA),
        readVec3(input.edgeB),
        input.squaredActivationDistance,
        input.stiffness);
    for (int row = 0; row < 9; ++row) {
      for (int col = 0; col < 9; ++col) {
        expected[81 * i + static_cast<std::size_t>(9 * row + col)]
            = barrier.hessian(row, col);
      }
    }
  }
  cuda::projectSymmetricBlocksToPsdReference(expected, 9, inputs.size());

  cuda::PointEdgeBarrierHessianResult result;
  cuda::evaluatePointEdgeBarrierHessiansCuda(inputs, result);
  cuda::projectSymmetricBlocksToPsdCuda(
      result.barrierHessians, 9, inputs.size());

  ASSERT_EQ(result.barrierHessians.size(), expected.size());
  for (std::size_t entry = 0; entry < expected.size(); ++entry) {
    EXPECT_NEAR(result.barrierHessians[entry], expected[entry], 1e-7)
        << "entry=" << entry;
  }
}

//==============================================================================
TEST(BarrierFrictionKernelCuda, MatchesCpuEdgeEdgeBarrierHessians)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  auto inputs = makeEdgeEdgeBarrierFixture();
  for (int i = 0; i < 4096; ++i) {
    inputs.push_back(makeGeneratedEdgeEdgeBarrierInput(i));
  }

  cuda::EdgeEdgeBarrierHessianResult result;
  cuda::evaluateEdgeEdgeBarrierHessiansCuda(inputs, result);

  ASSERT_EQ(result.squaredDistances.size(), inputs.size());
  ASSERT_EQ(result.barrierValues.size(), inputs.size());
  ASSERT_EQ(result.barrierGradients.size(), 12 * inputs.size());
  ASSERT_EQ(result.barrierHessians.size(), 144 * inputs.size());
  ASSERT_EQ(result.activeBarriers.size(), inputs.size());

  std::size_t activeCount = 0;
  for (std::size_t i = 0; i < inputs.size(); ++i) {
    const auto& input = inputs[i];
    const auto expected = nb::edgeEdgeBarrier(
        readVec3(input.edgeA0),
        readVec3(input.edgeA1),
        readVec3(input.edgeB0),
        readVec3(input.edgeB1),
        input.squaredActivationDistance,
        input.stiffness);

    activeCount += expected.active ? 1u : 0u;
    EXPECT_EQ(result.activeBarriers[i] != 0u, expected.active) << i;
    EXPECT_NEAR(result.squaredDistances[i], expected.squaredDistance, 1e-12)
        << i;
    EXPECT_NEAR(result.barrierValues[i], expected.value, 1e-12) << i;
    for (int entry = 0; entry < 12; ++entry) {
      EXPECT_NEAR(
          result.barrierGradients[12 * i + static_cast<std::size_t>(entry)],
          expected.gradient[entry],
          1e-10)
          << "fixture=" << i << " gradient entry=" << entry;
    }
    for (int row = 0; row < 12; ++row) {
      for (int col = 0; col < 12; ++col) {
        EXPECT_NEAR(
            result.barrierHessians
                [144 * i + static_cast<std::size_t>(12 * row + col)],
            expected.hessian(row, col),
            1e-7)
            << "fixture=" << i << " hessian row=" << row << " col=" << col;
      }
    }
  }
  EXPECT_EQ(result.activeBarrierCount, activeCount);
}

//==============================================================================
TEST(BarrierFrictionKernelCuda, ProjectsEdgeEdgeBarrierHessiansToPsd)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  auto inputs = makeEdgeEdgeBarrierFixture();
  for (int i = 0; i < 4096; ++i) {
    inputs.push_back(makeGeneratedEdgeEdgeBarrierInput(i));
  }

  std::vector<double> expected(144 * inputs.size(), 0.0);
  for (std::size_t i = 0; i < inputs.size(); ++i) {
    const auto& input = inputs[i];
    const auto barrier = nb::edgeEdgeBarrier(
        readVec3(input.edgeA0),
        readVec3(input.edgeA1),
        readVec3(input.edgeB0),
        readVec3(input.edgeB1),
        input.squaredActivationDistance,
        input.stiffness);
    for (int row = 0; row < 12; ++row) {
      for (int col = 0; col < 12; ++col) {
        expected[144 * i + static_cast<std::size_t>(12 * row + col)]
            = barrier.hessian(row, col);
      }
    }
  }
  cuda::projectSymmetricBlocksToPsdReference(expected, 12, inputs.size());

  cuda::EdgeEdgeBarrierHessianResult result;
  cuda::evaluateEdgeEdgeBarrierHessiansCuda(inputs, result);
  cuda::projectSymmetricBlocksToPsdCuda(
      result.barrierHessians, 12, inputs.size());

  ASSERT_EQ(result.barrierHessians.size(), expected.size());
  for (std::size_t entry = 0; entry < expected.size(); ++entry) {
    EXPECT_NEAR(result.barrierHessians[entry], expected[entry], 1e-7)
        << "entry=" << entry;
  }
}

//==============================================================================
TEST(BarrierFrictionKernelCuda, MatchesCpuPointTriangleTangentStencils)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  const auto inputs = makePointTriangleTangentFixture();

  cuda::PointTriangleTangentStencilResult result;
  cuda::evaluatePointTriangleTangentStencilsCuda(inputs, result);

  ASSERT_EQ(result.basisValues.size(), 6 * inputs.size());
  ASSERT_EQ(result.coordinates.size(), 2 * inputs.size());
  ASSERT_EQ(result.projectionValues.size(), 24 * inputs.size());
  ASSERT_EQ(result.fallbackBases.size(), inputs.size());
  EXPECT_EQ(result.fallbackBasisCount, 0u);

  for (std::size_t i = 0; i < inputs.size(); ++i) {
    const auto& input = inputs[i];
    const auto expected = nb::pointTriangleTangentStencil(
        readVec3(input.point),
        readVec3(input.triangleA),
        readVec3(input.triangleB),
        readVec3(input.triangleC));
    EXPECT_EQ(result.fallbackBases[i] != 0u, expected.usedFallbackBasis) << i;

    const std::size_t basisOffset = 6 * i;
    for (int component = 0; component < 3; ++component) {
      EXPECT_NEAR(
          result.basisValues[basisOffset + static_cast<std::size_t>(component)],
          expected.basis(component, 0),
          1e-12)
          << "fixture=" << i << " basis0 component=" << component;
      EXPECT_NEAR(
          result.basisValues
              [basisOffset + 3u + static_cast<std::size_t>(component)],
          expected.basis(component, 1),
          1e-12)
          << "fixture=" << i << " basis1 component=" << component;
    }

    const std::size_t coordinateOffset = 2 * i;
    EXPECT_NEAR(
        result.coordinates[coordinateOffset], expected.coordinates.x(), 1e-12)
        << i;
    EXPECT_NEAR(
        result.coordinates[coordinateOffset + 1],
        expected.coordinates.y(),
        1e-12)
        << i;

    const std::size_t projectionOffset = 24 * i;
    for (int block = 0; block < 4; ++block) {
      for (int component = 0; component < 3; ++component) {
        const std::size_t blockOffset
            = projectionOffset + static_cast<std::size_t>(6 * block);
        EXPECT_NEAR(
            result.projectionValues
                [blockOffset + static_cast<std::size_t>(component)],
            expected.projection(0, 3 * block + component),
            1e-12)
            << "fixture=" << i << " row=0 block=" << block
            << " component=" << component;
        EXPECT_NEAR(
            result.projectionValues
                [blockOffset + 3u + static_cast<std::size_t>(component)],
            expected.projection(1, 3 * block + component),
            1e-12)
            << "fixture=" << i << " row=1 block=" << block
            << " component=" << component;
      }
    }
  }
}

//==============================================================================
TEST(BarrierFrictionKernelCuda, MatchesCpuEdgeEdgeTangentStencils)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  const auto inputs = makeEdgeEdgeTangentFixture();

  cuda::EdgeEdgeTangentStencilResult result;
  cuda::evaluateEdgeEdgeTangentStencilsCuda(inputs, result);

  ASSERT_EQ(result.basisValues.size(), 6 * inputs.size());
  ASSERT_EQ(result.coordinates.size(), 2 * inputs.size());
  ASSERT_EQ(result.projectionValues.size(), 24 * inputs.size());
  ASSERT_EQ(result.fallbackBases.size(), inputs.size());
  EXPECT_EQ(result.fallbackBasisCount, 0u);

  for (std::size_t i = 0; i < inputs.size(); ++i) {
    const auto& input = inputs[i];
    const auto expected = nb::edgeEdgeTangentStencil(
        readVec3(input.edgeA0),
        readVec3(input.edgeA1),
        readVec3(input.edgeB0),
        readVec3(input.edgeB1));
    EXPECT_EQ(result.fallbackBases[i] != 0u, expected.usedFallbackBasis) << i;

    const std::size_t basisOffset = 6 * i;
    for (int component = 0; component < 3; ++component) {
      EXPECT_NEAR(
          result.basisValues[basisOffset + static_cast<std::size_t>(component)],
          expected.basis(component, 0),
          1e-12)
          << "fixture=" << i << " basis0 component=" << component;
      EXPECT_NEAR(
          result.basisValues
              [basisOffset + 3u + static_cast<std::size_t>(component)],
          expected.basis(component, 1),
          1e-12)
          << "fixture=" << i << " basis1 component=" << component;
    }

    const std::size_t coordinateOffset = 2 * i;
    EXPECT_NEAR(
        result.coordinates[coordinateOffset], expected.coordinates.x(), 1e-12)
        << i;
    EXPECT_NEAR(
        result.coordinates[coordinateOffset + 1],
        expected.coordinates.y(),
        1e-12)
        << i;

    const std::size_t projectionOffset = 24 * i;
    for (int block = 0; block < 4; ++block) {
      for (int component = 0; component < 3; ++component) {
        const std::size_t blockOffset
            = projectionOffset + static_cast<std::size_t>(6 * block);
        EXPECT_NEAR(
            result.projectionValues
                [blockOffset + static_cast<std::size_t>(component)],
            expected.projection(0, 3 * block + component),
            1e-12)
            << "fixture=" << i << " row=0 block=" << block
            << " component=" << component;
        EXPECT_NEAR(
            result.projectionValues
                [blockOffset + 3u + static_cast<std::size_t>(component)],
            expected.projection(1, 3 * block + component),
            1e-12)
            << "fixture=" << i << " row=1 block=" << block
            << " component=" << component;
      }
    }
  }
}

//==============================================================================
TEST(BarrierFrictionKernelCuda, MatchesCpuPointEdgeTangentStencils)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  auto inputs = makePointEdgeTangentFixture();
  for (int i = 0; i < 4096; ++i) {
    inputs.push_back(makeGeneratedPointEdgeTangentInput(i));
  }

  cuda::PointEdgeTangentStencilResult result;
  cuda::evaluatePointEdgeTangentStencilsCuda(inputs, result);

  ASSERT_EQ(result.basisValues.size(), 6 * inputs.size());
  ASSERT_EQ(result.coordinates.size(), inputs.size());
  ASSERT_EQ(result.projectionValues.size(), 18 * inputs.size());
  ASSERT_EQ(result.fallbackBases.size(), inputs.size());
  EXPECT_EQ(result.fallbackBasisCount, 0u);

  for (std::size_t i = 0; i < inputs.size(); ++i) {
    const auto& input = inputs[i];
    const auto expected = nb::pointEdgeTangentStencil(
        readVec3(input.point), readVec3(input.edgeA), readVec3(input.edgeB));
    EXPECT_EQ(result.fallbackBases[i] != 0u, expected.usedFallbackBasis) << i;

    const std::size_t basisOffset = 6 * i;
    for (int component = 0; component < 3; ++component) {
      EXPECT_NEAR(
          result.basisValues[basisOffset + static_cast<std::size_t>(component)],
          expected.basis(component, 0),
          1e-12)
          << "fixture=" << i << " basis0 component=" << component;
      EXPECT_NEAR(
          result.basisValues
              [basisOffset + 3u + static_cast<std::size_t>(component)],
          expected.basis(component, 1),
          1e-12)
          << "fixture=" << i << " basis1 component=" << component;
    }

    EXPECT_NEAR(result.coordinates[i], expected.coordinate, 1e-12) << i;

    const std::size_t projectionOffset = 18 * i;
    for (int block = 0; block < 3; ++block) {
      for (int component = 0; component < 3; ++component) {
        const std::size_t blockOffset
            = projectionOffset + static_cast<std::size_t>(6 * block);
        EXPECT_NEAR(
            result.projectionValues
                [blockOffset + static_cast<std::size_t>(component)],
            expected.projection(0, 3 * block + component),
            1e-12)
            << "fixture=" << i << " row=0 block=" << block
            << " component=" << component;
        EXPECT_NEAR(
            result.projectionValues
                [blockOffset + 3u + static_cast<std::size_t>(component)],
            expected.projection(1, 3 * block + component),
            1e-12)
            << "fixture=" << i << " row=1 block=" << block
            << " component=" << component;
      }
    }
  }
}

//==============================================================================
TEST(BarrierFrictionKernelCuda, MatchesCpuPointPointTangentStencils)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  auto inputs = makePointPointTangentFixture();
  for (int i = 0; i < 65536; ++i) {
    inputs.push_back(makeGeneratedPointPointTangentInput(i));
  }

  cuda::PointPointTangentStencilResult result;
  cuda::evaluatePointPointTangentStencilsCuda(inputs, result);

  ASSERT_EQ(result.basisValues.size(), 6 * inputs.size());
  ASSERT_EQ(result.projectionValues.size(), 12 * inputs.size());
  ASSERT_EQ(result.fallbackBases.size(), inputs.size());
  EXPECT_EQ(result.fallbackBasisCount, 0u);

  for (std::size_t i = 0; i < inputs.size(); ++i) {
    const auto& input = inputs[i];
    const auto expected = nb::pointPointTangentStencil(
        readVec3(input.pointA), readVec3(input.pointB));
    EXPECT_EQ(result.fallbackBases[i] != 0u, expected.usedFallbackBasis) << i;

    const std::size_t basisOffset = 6 * i;
    for (int component = 0; component < 3; ++component) {
      EXPECT_NEAR(
          result.basisValues[basisOffset + static_cast<std::size_t>(component)],
          expected.basis(component, 0),
          1e-12)
          << "fixture=" << i << " basis0 component=" << component;
      EXPECT_NEAR(
          result.basisValues
              [basisOffset + 3u + static_cast<std::size_t>(component)],
          expected.basis(component, 1),
          1e-12)
          << "fixture=" << i << " basis1 component=" << component;
    }

    const std::size_t projectionOffset = 12 * i;
    for (int block = 0; block < 2; ++block) {
      for (int component = 0; component < 3; ++component) {
        const std::size_t blockOffset
            = projectionOffset + static_cast<std::size_t>(6 * block);
        EXPECT_NEAR(
            result.projectionValues
                [blockOffset + static_cast<std::size_t>(component)],
            expected.projection(0, 3 * block + component),
            1e-12)
            << "fixture=" << i << " row=0 block=" << block
            << " component=" << component;
        EXPECT_NEAR(
            result.projectionValues
                [blockOffset + 3u + static_cast<std::size_t>(component)],
            expected.projection(1, 3 * block + component),
            1e-12)
            << "fixture=" << i << " row=1 block=" << block
            << " component=" << component;
      }
    }
  }
}

//==============================================================================
TEST(BarrierFrictionKernelCuda, RejectsNonFiniteInput)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  auto inputs = makeFixture();
  inputs.front().squaredDistance = std::numeric_limits<double>::quiet_NaN();

  cuda::BarrierFrictionLocalResult result;
  EXPECT_THROW(
      cuda::evaluateBarrierFrictionLocalKernelsCuda(inputs, result),
      dart::simulation::InvalidArgumentException);

  auto pointTriangleInputs = makePointTriangleFixture();
  pointTriangleInputs.front().point[0]
      = std::numeric_limits<double>::quiet_NaN();

  cuda::PointTriangleBarrierGradientResult pointTriangleResult;
  EXPECT_THROW(
      cuda::evaluatePointTriangleBarrierGradientsCuda(
          pointTriangleInputs, pointTriangleResult),
      dart::simulation::InvalidArgumentException);

  cuda::PointTriangleBarrierHessianResult pointTriangleHessianResult;
  EXPECT_THROW(
      cuda::evaluatePointTriangleBarrierHessiansCuda(
          pointTriangleInputs, pointTriangleHessianResult),
      dart::simulation::InvalidArgumentException);

  auto pointPointBarrierInputs = makePointPointBarrierFixture();
  pointPointBarrierInputs.front().pointB[2]
      = std::numeric_limits<double>::quiet_NaN();

  cuda::PointPointBarrierHessianResult pointPointBarrierResult;
  EXPECT_THROW(
      cuda::evaluatePointPointBarrierHessiansCuda(
          pointPointBarrierInputs, pointPointBarrierResult),
      dart::simulation::InvalidArgumentException);

  auto pointEdgeBarrierInputs = makePointEdgeBarrierFixture();
  pointEdgeBarrierInputs.front().edgeB[0]
      = std::numeric_limits<double>::quiet_NaN();

  cuda::PointEdgeBarrierHessianResult pointEdgeBarrierResult;
  EXPECT_THROW(
      cuda::evaluatePointEdgeBarrierHessiansCuda(
          pointEdgeBarrierInputs, pointEdgeBarrierResult),
      dart::simulation::InvalidArgumentException);

  auto edgeEdgeBarrierInputs = makeEdgeEdgeBarrierFixture();
  edgeEdgeBarrierInputs.front().edgeB1[0]
      = std::numeric_limits<double>::quiet_NaN();

  cuda::EdgeEdgeBarrierHessianResult edgeEdgeBarrierResult;
  EXPECT_THROW(
      cuda::evaluateEdgeEdgeBarrierHessiansCuda(
          edgeEdgeBarrierInputs, edgeEdgeBarrierResult),
      dart::simulation::InvalidArgumentException);

  auto tangentInputs = makePointTriangleTangentFixture();
  tangentInputs.front().triangleB[1] = std::numeric_limits<double>::quiet_NaN();

  cuda::PointTriangleTangentStencilResult tangentResult;
  EXPECT_THROW(
      cuda::evaluatePointTriangleTangentStencilsCuda(
          tangentInputs, tangentResult),
      dart::simulation::InvalidArgumentException);

  auto edgeEdgeTangentInputs = makeEdgeEdgeTangentFixture();
  edgeEdgeTangentInputs.front().edgeB1[2]
      = std::numeric_limits<double>::quiet_NaN();

  cuda::EdgeEdgeTangentStencilResult edgeEdgeTangentResult;
  EXPECT_THROW(
      cuda::evaluateEdgeEdgeTangentStencilsCuda(
          edgeEdgeTangentInputs, edgeEdgeTangentResult),
      dart::simulation::InvalidArgumentException);

  auto pointEdgeTangentInputs = makePointEdgeTangentFixture();
  pointEdgeTangentInputs.front().edgeA[0]
      = std::numeric_limits<double>::quiet_NaN();

  cuda::PointEdgeTangentStencilResult pointEdgeTangentResult;
  EXPECT_THROW(
      cuda::evaluatePointEdgeTangentStencilsCuda(
          pointEdgeTangentInputs, pointEdgeTangentResult),
      dart::simulation::InvalidArgumentException);

  auto pointPointTangentInputs = makePointPointTangentFixture();
  pointPointTangentInputs.front().pointB[1]
      = std::numeric_limits<double>::quiet_NaN();

  cuda::PointPointTangentStencilResult pointPointTangentResult;
  EXPECT_THROW(
      cuda::evaluatePointPointTangentStencilsCuda(
          pointPointTangentInputs, pointPointTangentResult),
      dart::simulation::InvalidArgumentException);
}
