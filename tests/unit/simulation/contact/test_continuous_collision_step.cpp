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
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include <dart/simulation/detail/deformable_contact/continuous_collision_step.hpp>
#include <dart/simulation/detail/deformable_contact/primitive_distance.hpp>

#include <dart/collision/native/narrow_phase/primitive_ccd.hpp>

#include <gtest/gtest.h>

#include <vector>

#include <cmath>

namespace sx = dart::simulation;
namespace dc = dart::simulation::detail::deformable_contact;
namespace nc = dart::collision::native;

namespace {

const Eigen::Vector3d kA(-1.0, -1.0, 0.0);
const Eigen::Vector3d kB(1.0, -1.0, 0.0);
const Eigen::Vector3d kC(0.0, 1.0, 0.0);

//==============================================================================
Eigen::Vector3d
at(const Eigen::Vector3d& start, const Eigen::Vector3d& end, const double alpha)
{
  return start + alpha * (end - start);
}

//==============================================================================
void expectPointTriangleSamplesSafeBeforeBound(
    const Eigen::Vector3d& pointStart,
    const Eigen::Vector3d& pointEnd,
    const Eigen::Vector3d& aStart,
    const Eigen::Vector3d& aEnd,
    const Eigen::Vector3d& bStart,
    const Eigen::Vector3d& bEnd,
    const Eigen::Vector3d& cStart,
    const Eigen::Vector3d& cEnd,
    const dc::ContinuousCollisionStepOptions& options,
    const double stepBound)
{
  if (stepBound <= options.tolerance) {
    return;
  }

  constexpr int kSamples = 256;
  for (int i = 0; i < kSamples; ++i) {
    const double alpha = stepBound * static_cast<double>(i) / kSamples;
    const double distance = std::sqrt(
        dc::pointTriangleSquaredDistance(
            at(pointStart, pointEnd, alpha),
            at(aStart, aEnd, alpha),
            at(bStart, bEnd, alpha),
            at(cStart, cEnd, alpha))
            .squaredDistance);
    EXPECT_GE(distance, options.minSeparation - 2.0 * options.tolerance);
  }
}

//==============================================================================
void expectEdgeEdgeSamplesSafeBeforeBound(
    const Eigen::Vector3d& aStart,
    const Eigen::Vector3d& aEnd,
    const Eigen::Vector3d& bStart,
    const Eigen::Vector3d& bEnd,
    const Eigen::Vector3d& cStart,
    const Eigen::Vector3d& cEnd,
    const Eigen::Vector3d& dStart,
    const Eigen::Vector3d& dEnd,
    const dc::ContinuousCollisionStepOptions& options,
    const double stepBound)
{
  if (stepBound <= options.tolerance) {
    return;
  }

  constexpr int kSamples = 256;
  for (int i = 0; i < kSamples; ++i) {
    const double alpha = stepBound * static_cast<double>(i) / kSamples;
    const double distance = std::sqrt(
        dc::edgeEdgeSquaredDistance(
            at(aStart, aEnd, alpha),
            at(bStart, bEnd, alpha),
            at(cStart, cEnd, alpha),
            at(dStart, dEnd, alpha))
            .squaredDistance);
    EXPECT_GE(distance, options.minSeparation - 2.0 * options.tolerance);
  }
}

//==============================================================================
nc::CcdOption nativeOption(const dc::ContinuousCollisionStepOptions& options)
{
  nc::CcdOption native;
  native.tolerance = options.tolerance;
  native.maxIterations = options.maxIterations;
  native.minSeparation = options.minSeparation;
  native.advancement = nc::CcdAdvancement::Conservative;
  return native;
}

} // namespace

//==============================================================================
TEST(IpcContinuousCollisionStep, PointTriangleDirectDropMatchesExactBound)
{
  dc::ContinuousCollisionStepOptions options;
  const Eigen::Vector3d p0(0.0, 0.0, 1.0);
  const Eigen::Vector3d p1(0.0, 0.0, -1.0);

  const auto result
      = dc::pointTriangleStepBound(p0, p1, kA, kA, kB, kB, kC, kC, options);

  nc::CcdPrimitiveResult exact;
  ASSERT_TRUE(
      nc::pointTriangleCcdExact(
          p0, p1, kA, kA, kB, kB, kC, kC, nativeOption(options), exact));

  EXPECT_TRUE(result.hit);
  EXPECT_EQ(
      result.limitingPrimitive,
      dc::ContinuousCollisionPrimitive::PointTriangle);
  EXPECT_LE(result.stepBound, exact.timeOfImpact + 1e-6);
  EXPECT_NEAR(result.stepBound, 0.5, 1e-2);
  EXPECT_EQ(result.stats.pointTriangleChecks, 1u);
  EXPECT_EQ(result.stats.hits, 1u);
  expectPointTriangleSamplesSafeBeforeBound(
      p0, p1, kA, kA, kB, kB, kC, kC, options, result.stepBound);
}

//==============================================================================
TEST(IpcContinuousCollisionStep, PointTriangleMovingTriangleAndSeparation)
{
  dc::ContinuousCollisionStepOptions options;
  options.minSeparation = 0.2;

  const Eigen::Vector3d p0(0.0, 0.0, 1.0);
  const Eigen::Vector3d p1(0.0, 0.0, 0.0);
  const Eigen::Vector3d up(0.0, 0.0, 0.5);
  const auto result = dc::pointTriangleStepBound(
      p0, p1, kA, kA + up, kB, kB + up, kC, kC + up, options);

  EXPECT_TRUE(result.hit);
  EXPECT_LE(result.stepBound, 8.0 / 15.0 + 1e-9);
  EXPECT_NEAR(result.stepBound, 8.0 / 15.0, 1e-2);
  expectPointTriangleSamplesSafeBeforeBound(
      p0, p1, kA, kA + up, kB, kB + up, kC, kC + up, options, result.stepBound);
}

//==============================================================================
TEST(IpcContinuousCollisionStep, PointTriangleMissAndInitialBand)
{
  dc::ContinuousCollisionStepOptions options;
  const auto miss = dc::pointTriangleStepBound(
      Eigen::Vector3d(5.0, 0.0, 1.0),
      Eigen::Vector3d(5.0, 0.0, -1.0),
      kA,
      kA,
      kB,
      kB,
      kC,
      kC,
      options);

  EXPECT_FALSE(miss.hit);
  EXPECT_EQ(miss.stepBound, 1.0);
  EXPECT_EQ(miss.stats.misses, 1u);

  options.minSeparation = 0.1;
  const auto initial = dc::pointTriangleStepBound(
      Eigen::Vector3d(0.0, 0.0, 0.05),
      Eigen::Vector3d(0.0, 0.0, 0.05),
      kA,
      kA,
      kB,
      kB,
      kC,
      kC,
      options);

  EXPECT_TRUE(initial.hit);
  EXPECT_EQ(initial.stepBound, 0.0);
  EXPECT_EQ(initial.stats.zeroStepCount, 1u);
}

//==============================================================================
TEST(IpcContinuousCollisionStep, EdgeEdgeCrossingMatchesExactBound)
{
  dc::ContinuousCollisionStepOptions options;
  const Eigen::Vector3d a0(-1.0, 0.0, 0.5);
  const Eigen::Vector3d a1(-1.0, 0.0, -0.5);
  const Eigen::Vector3d b0(1.0, 0.0, 0.5);
  const Eigen::Vector3d b1(1.0, 0.0, -0.5);
  const Eigen::Vector3d c0(0.0, -1.0, 0.0);
  const Eigen::Vector3d d0(0.0, 1.0, 0.0);

  const auto result
      = dc::edgeEdgeStepBound(a0, a1, b0, b1, c0, c0, d0, d0, options);

  nc::CcdPrimitiveResult exact;
  ASSERT_TRUE(
      nc::edgeEdgeCcdExact(
          a0, a1, b0, b1, c0, c0, d0, d0, nativeOption(options), exact));

  EXPECT_TRUE(result.hit);
  EXPECT_EQ(
      result.limitingPrimitive, dc::ContinuousCollisionPrimitive::EdgeEdge);
  EXPECT_LE(result.stepBound, exact.timeOfImpact + 1e-6);
  EXPECT_NEAR(result.stepBound, 0.5, 1e-2);
  EXPECT_EQ(result.stats.edgeEdgeChecks, 1u);
  expectEdgeEdgeSamplesSafeBeforeBound(
      a0, a1, b0, b1, c0, c0, d0, d0, options, result.stepBound);
}

//==============================================================================
TEST(IpcContinuousCollisionStep, EdgeEdgeNearParallelMissAndInitialBand)
{
  dc::ContinuousCollisionStepOptions options;
  const auto miss = dc::edgeEdgeStepBound(
      Eigen::Vector3d(-1.0, 0.0, 0.0),
      Eigen::Vector3d(-1.0, 0.0, 0.0),
      Eigen::Vector3d(1.0, 0.0, 0.0),
      Eigen::Vector3d(1.0, 0.0, 0.0),
      Eigen::Vector3d(-1.0, 0.2, 0.0),
      Eigen::Vector3d(-1.0, 0.2, 0.0),
      Eigen::Vector3d(1.0, 0.2, 0.0),
      Eigen::Vector3d(1.0, 0.2, 0.0),
      options);

  EXPECT_FALSE(miss.hit);
  EXPECT_EQ(miss.stepBound, 1.0);

  options.minSeparation = 0.25;
  const auto initial = dc::edgeEdgeStepBound(
      Eigen::Vector3d(-1.0, 0.0, 0.0),
      Eigen::Vector3d(-1.0, 0.0, 0.0),
      Eigen::Vector3d(1.0, 0.0, 0.0),
      Eigen::Vector3d(1.0, 0.0, 0.0),
      Eigen::Vector3d(-1.0, 0.2, 0.0),
      Eigen::Vector3d(-1.0, 0.2, 0.0),
      Eigen::Vector3d(1.0, 0.2, 0.0),
      Eigen::Vector3d(1.0, 0.2, 0.0),
      options);

  EXPECT_TRUE(initial.hit);
  EXPECT_EQ(initial.stepBound, 0.0);
  EXPECT_EQ(initial.stats.zeroStepCount, 1u);
}

//==============================================================================
TEST(IpcContinuousCollisionStep, CandidateSetChoosesDeterministicMinimum)
{
  const std::vector<Eigen::Vector3d> start = {
      {-1.0, -1.0, 0.0},
      {1.0, -1.0, 0.0},
      {0.0, 1.0, 0.0},
      {0.0, 0.0, 1.0},
      {-1.0, 0.0, 0.5},
      {1.0, 0.0, 0.5},
      {0.0, -1.0, 0.0},
      {0.0, 1.0, 0.0},
  };
  std::vector<Eigen::Vector3d> end = start;
  end[3] = Eigen::Vector3d(0.0, 0.0, -3.0);
  end[4] = Eigen::Vector3d(-1.0, 0.0, -0.5);
  end[5] = Eigen::Vector3d(1.0, 0.0, -0.5);

  const std::vector<sx::DeformableSurfaceTriangle> triangles = {{0, 1, 2}};
  dc::ContactCandidateSet candidates;
  candidates.surfaceEdges = {
      dc::SurfaceEdge{4, 5},
      dc::SurfaceEdge{6, 7},
  };
  candidates.pointTriangleCandidates.push_back(
      dc::PointTriangleCandidate{3, 0, 0.0});
  candidates.edgeEdgeCandidates.push_back(dc::EdgeEdgeCandidate{0, 1, 0.0});

  dc::ContinuousCollisionStepOptions options;
  const auto first = dc::contactCandidateStepBound(
      start, end, triangles, candidates, options);
  const auto second = dc::contactCandidateStepBound(
      start, end, triangles, candidates, options);

  EXPECT_TRUE(first.hit);
  EXPECT_EQ(first.stepBound, second.stepBound);
  EXPECT_EQ(
      first.limitingPrimitive, dc::ContinuousCollisionPrimitive::PointTriangle);
  EXPECT_EQ(first.limitingCandidate, 0u);
  EXPECT_NEAR(first.stepBound, 0.25, 1e-2);
  EXPECT_EQ(first.stats.pointTriangleChecks, 1u);
  EXPECT_EQ(first.stats.edgeEdgeChecks, 1u);
  EXPECT_EQ(first.stats.hits, 2u);
}

//==============================================================================
TEST(IpcContinuousCollisionStep, MotionAwareCandidatesRecoverFastCrossing)
{
  const std::vector<Eigen::Vector3d> start = {
      {-1.0, -1.0, 0.0},
      {1.0, -1.0, 0.0},
      {0.0, 1.0, 0.0},
      {0.0, 0.0, 1.0},
  };
  auto end = start;
  end[3] = Eigen::Vector3d(0.0, 0.0, -1.0);

  const std::vector<sx::DeformableSurfaceTriangle> triangles = {{0, 1, 2}};

  dc::ContactCandidateOptions candidateOptions;
  candidateOptions.activationDistance = 0.05;
  candidateOptions.exactDistanceFilter = true;

  const auto staticCandidates
      = dc::buildContactCandidatesSweep(start, triangles, candidateOptions);
  const auto staticResult
      = dc::contactCandidateStepBound(start, end, triangles, staticCandidates);
  EXPECT_FALSE(staticResult.hit);

  const auto motionCandidates = dc::buildMotionAwareContactCandidatesSweep(
      start, end, triangles, candidateOptions);
  ASSERT_EQ(motionCandidates.pointTriangleCandidates.size(), 1u);

  const auto motionResult
      = dc::contactCandidateStepBound(start, end, triangles, motionCandidates);
  EXPECT_TRUE(motionResult.hit);
  EXPECT_EQ(
      motionResult.limitingPrimitive,
      dc::ContinuousCollisionPrimitive::PointTriangle);
  EXPECT_NEAR(motionResult.stepBound, 0.5, 1e-2);
  EXPECT_EQ(motionResult.stats.pointTriangleChecks, 1u);
  EXPECT_EQ(motionResult.stats.hits, 1u);
}
