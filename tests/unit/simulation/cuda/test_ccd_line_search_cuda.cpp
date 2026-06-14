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
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include <dart/simulation/common/exceptions.hpp>
#include <dart/simulation/detail/deformable_contact/continuous_collision_step.hpp>

#include <Eigen/Core>
#include <dart/simulation/compute/cuda/ccd_line_search_cuda.cuh>
#include <dart/simulation/compute/cuda/cuda_runtime.cuh>
#include <gtest/gtest.h>

#include <limits>
#include <vector>

namespace cuda = dart::simulation::compute::cuda;
namespace dc = dart::simulation::detail::deformable_contact;

namespace {

cuda::PointTriangleCcdLineSearchPair makePair(
    const Eigen::Vector3d& pointStart,
    const Eigen::Vector3d& pointEnd,
    const Eigen::Vector3d& aStart,
    const Eigen::Vector3d& aEnd,
    const Eigen::Vector3d& bStart,
    const Eigen::Vector3d& bEnd,
    const Eigen::Vector3d& cStart,
    const Eigen::Vector3d& cEnd)
{
  cuda::PointTriangleCcdLineSearchPair pair;
  for (int i = 0; i < 3; ++i) {
    pair.pointStart[i] = pointStart[i];
    pair.pointEnd[i] = pointEnd[i];
    pair.triangleAStart[i] = aStart[i];
    pair.triangleAEnd[i] = aEnd[i];
    pair.triangleBStart[i] = bStart[i];
    pair.triangleBEnd[i] = bEnd[i];
    pair.triangleCStart[i] = cStart[i];
    pair.triangleCEnd[i] = cEnd[i];
  }
  return pair;
}

cuda::PointTriangleCcdLineSearchPair makePair(
    const Eigen::Vector3d& pointStart,
    const Eigen::Vector3d& pointEnd,
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c)
{
  return makePair(pointStart, pointEnd, a, a, b, b, c, c);
}

cuda::EdgeEdgeCcdLineSearchPair makeEdgeEdgePair(
    const Eigen::Vector3d& edgeA0Start,
    const Eigen::Vector3d& edgeA0End,
    const Eigen::Vector3d& edgeA1Start,
    const Eigen::Vector3d& edgeA1End,
    const Eigen::Vector3d& edgeB0Start,
    const Eigen::Vector3d& edgeB0End,
    const Eigen::Vector3d& edgeB1Start,
    const Eigen::Vector3d& edgeB1End)
{
  cuda::EdgeEdgeCcdLineSearchPair pair;
  for (int i = 0; i < 3; ++i) {
    pair.edgeA0Start[i] = edgeA0Start[i];
    pair.edgeA0End[i] = edgeA0End[i];
    pair.edgeA1Start[i] = edgeA1Start[i];
    pair.edgeA1End[i] = edgeA1End[i];
    pair.edgeB0Start[i] = edgeB0Start[i];
    pair.edgeB0End[i] = edgeB0End[i];
    pair.edgeB1Start[i] = edgeB1Start[i];
    pair.edgeB1End[i] = edgeB1End[i];
  }
  return pair;
}

Eigen::Vector3d vec3(const double values[3])
{
  return {values[0], values[1], values[2]};
}

dc::ContinuousCollisionStepResult cpuResult(
    const cuda::PointTriangleCcdLineSearchPair& pair,
    const cuda::CcdLineSearchOptions& options)
{
  dc::ContinuousCollisionStepOptions cpuOptions;
  cpuOptions.minSeparation = options.minSeparation;
  cpuOptions.tolerance = options.tolerance;
  return dc::pointTriangleStepBound(
      vec3(pair.pointStart),
      vec3(pair.pointEnd),
      vec3(pair.triangleAStart),
      vec3(pair.triangleAEnd),
      vec3(pair.triangleBStart),
      vec3(pair.triangleBEnd),
      vec3(pair.triangleCStart),
      vec3(pair.triangleCEnd),
      cpuOptions);
}

dc::ContinuousCollisionStepResult cpuEdgeEdgeResult(
    const cuda::EdgeEdgeCcdLineSearchPair& pair,
    const cuda::CcdLineSearchOptions& options)
{
  dc::ContinuousCollisionStepOptions cpuOptions;
  cpuOptions.minSeparation = options.minSeparation;
  cpuOptions.tolerance = options.tolerance;
  cpuOptions.maxIterations = options.maxIterations;
  return dc::edgeEdgeStepBound(
      vec3(pair.edgeA0Start),
      vec3(pair.edgeA0End),
      vec3(pair.edgeA1Start),
      vec3(pair.edgeA1End),
      vec3(pair.edgeB0Start),
      vec3(pair.edgeB0End),
      vec3(pair.edgeB1Start),
      vec3(pair.edgeB1End),
      cpuOptions);
}

std::vector<cuda::PointTriangleCcdLineSearchPair> makeFixture()
{
  const Eigen::Vector3d a(-1.0, -1.0, 0.0);
  const Eigen::Vector3d b(1.0, -1.0, 0.0);
  const Eigen::Vector3d c(0.0, 1.0, 0.0);
  return {
      makePair({0.0, 0.0, 0.2}, {0.0, 0.0, -0.2}, a, b, c),
      makePair({0.0, 0.0, -0.2}, {0.0, 0.0, 0.2}, a, b, c),
      makePair({0.0, 0.0, -0.2}, {0.0, 0.0, 0.2}, a, c, b),
      makePair({0.25, 0.0, 0.2}, {0.25, 0.0, 0.1}, a, b, c),
      makePair({2.0, 0.0, 0.2}, {2.0, 0.0, -0.2}, a, b, c),
      makePair({0.0, 0.0, 0.3}, {0.0, 0.0, -0.1}, a, b, c),
      makePair(
          {0.0, 0.0, 0.0},
          {0.0, 0.0, 0.0},
          {-1.0, -1.0, 0.2},
          {-1.0, -1.0, -0.2},
          {1.0, -1.0, 0.2},
          {1.0, -1.0, -0.2},
          {0.0, 1.0, 0.2},
          {0.0, 1.0, -0.2}),
      makePair(
          {0.25, 0.25, 0.0},
          {0.25, 0.25, 0.0},
          {0.0, 0.0, 0.2},
          {0.0, 0.0, -0.2},
          {1.0, 0.0, 0.2},
          {1.0, 0.0, -0.2},
          {0.0, 1.0, 0.2},
          {0.0, 1.0, 0.2}),
  };
}

std::vector<cuda::EdgeEdgeCcdLineSearchPair> makeEdgeEdgeFixture()
{
  const Eigen::Vector3d staticB0(0.0, -0.5, 0.0);
  const Eigen::Vector3d staticB1(0.0, 0.5, 0.0);
  return {
      makeEdgeEdgePair(
          {-0.5, 0.0, 0.0},
          {0.5, 0.0, 0.0},
          {-0.5, 1.0, 0.0},
          {0.5, 1.0, 0.0},
          staticB0,
          staticB0,
          staticB1,
          staticB1),
      makeEdgeEdgePair(
          {-0.5, 0.0, 0.0},
          {-0.25, 0.0, 0.0},
          {-0.5, 1.0, 0.0},
          {-0.25, 1.0, 0.0},
          staticB0,
          staticB0,
          staticB1,
          staticB1),
      makeEdgeEdgePair(
          {0.0, -0.5, 0.0},
          {0.0, -0.5, 0.0},
          {0.0, 0.5, 0.0},
          {0.0, 0.5, 0.0},
          {-0.5, 0.0, 0.0},
          {-0.5, 0.0, 0.0},
          {0.5, 0.0, 0.0},
          {0.5, 0.0, 0.0}),
  };
}

} // namespace

//==============================================================================
TEST(CcdLineSearchCuda, MatchesCpuPointTriangleStepBounds)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  const std::vector<cuda::PointTriangleCcdLineSearchPair> pairs = makeFixture();
  const cuda::CcdLineSearchOptions options;

  cuda::PointTriangleCcdLineSearchResult result;
  cuda::evaluatePointTriangleCcdLineSearchCuda(pairs, options, result);

  ASSERT_EQ(result.stepBounds.size(), pairs.size());
  ASSERT_EQ(result.hits.size(), pairs.size());
  ASSERT_EQ(result.indeterminate.size(), pairs.size());
  EXPECT_LT(result.minStepBound, 1.0);

  std::size_t expectedHits = 0;
  for (std::size_t i = 0; i < pairs.size(); ++i) {
    const auto expected = cpuResult(pairs[i], options);
    if (expected.hit) {
      ++expectedHits;
    }
    EXPECT_EQ(result.hits[i] != 0u, expected.hit) << i;
    EXPECT_EQ(result.indeterminate[i] != 0u, expected.indeterminate) << i;
    EXPECT_NEAR(result.stepBounds[i], expected.stepBound, 1e-8) << i;
  }
  EXPECT_EQ(result.hitCount, expectedHits);
  EXPECT_NEAR(result.stepBounds[1], result.stepBounds[2], 1e-12);
}

//==============================================================================
TEST(CcdLineSearchCuda, MatchesCpuEdgeEdgeStepBounds)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  const std::vector<cuda::EdgeEdgeCcdLineSearchPair> pairs
      = makeEdgeEdgeFixture();
  const cuda::CcdLineSearchOptions options;

  cuda::EdgeEdgeCcdLineSearchResult result;
  cuda::evaluateEdgeEdgeCcdLineSearchCuda(pairs, options, result);

  ASSERT_EQ(result.stepBounds.size(), pairs.size());
  ASSERT_EQ(result.hits.size(), pairs.size());
  ASSERT_EQ(result.indeterminate.size(), pairs.size());
  EXPECT_LT(result.minStepBound, 1.0);

  std::size_t expectedHits = 0;
  for (std::size_t i = 0; i < pairs.size(); ++i) {
    const auto expected = cpuEdgeEdgeResult(pairs[i], options);
    if (expected.hit) {
      ++expectedHits;
    }
    EXPECT_EQ(result.hits[i] != 0u, expected.hit) << i;
    EXPECT_EQ(result.indeterminate[i] != 0u, expected.indeterminate) << i;
    EXPECT_NEAR(result.stepBounds[i], expected.stepBound, 1e-8) << i;
  }
  EXPECT_EQ(result.hitCount, expectedHits);
}

//==============================================================================
TEST(CcdLineSearchCuda, RejectsInvalidOptions)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  cuda::CcdLineSearchOptions options;
  options.tolerance = std::numeric_limits<double>::quiet_NaN();

  cuda::PointTriangleCcdLineSearchResult result;
  EXPECT_THROW(
      cuda::evaluatePointTriangleCcdLineSearchCuda(
          makeFixture(), options, result),
      dart::simulation::InvalidArgumentException);
}
