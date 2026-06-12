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
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR
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
#include <dart/simulation/detail/deformable_contact/candidate_set.hpp>

#include <Eigen/Core>
#include <dart/simulation/compute/cuda/contact_candidate_filter_cuda.cuh>
#include <dart/simulation/compute/cuda/cuda_runtime.cuh>
#include <gtest/gtest.h>

#include <array>
#include <vector>

#include <cstdint>

namespace cuda = dart::simulation::compute::cuda;
namespace dc = dart::simulation::detail::deformable_contact;

namespace {

struct Fixture
{
  std::vector<double> positions;
  std::vector<std::uint32_t> triangles;
  std::vector<cuda::PointTriangleContactStencil> stencils;
};

void appendPoint(Fixture& fixture, const Eigen::Vector3d& point)
{
  fixture.positions.push_back(point.x());
  fixture.positions.push_back(point.y());
  fixture.positions.push_back(point.z());
}

Fixture makeFixture()
{
  Fixture fixture;
  fixture.positions.reserve(24);
  fixture.triangles.reserve(6);
  fixture.stencils.reserve(2);

  appendPoint(fixture, {0.0, 0.0, 0.0});
  appendPoint(fixture, {1.0, 0.0, 0.0});
  appendPoint(fixture, {0.0, 1.0, 0.0});
  appendPoint(fixture, {0.25, 0.25, 0.02});
  fixture.triangles.insert(fixture.triangles.end(), {0u, 1u, 2u});
  fixture.stencils.push_back({3u, 0u});

  appendPoint(fixture, {2.0, 0.0, 0.0});
  appendPoint(fixture, {3.0, 0.0, 0.0});
  appendPoint(fixture, {2.0, 1.0, 0.0});
  appendPoint(fixture, {2.25, 0.25, 0.08});
  fixture.triangles.insert(fixture.triangles.end(), {4u, 5u, 6u});
  fixture.stencils.push_back({7u, 1u});

  return fixture;
}

Eigen::Vector3d pointAt(
    const std::vector<double>& positions, const std::size_t i)
{
  const std::size_t base = 3u * i;
  return {positions[base], positions[base + 1u], positions[base + 2u]};
}

std::vector<double> cpuSquaredDistances(const Fixture& fixture)
{
  std::vector<double> distances;
  distances.reserve(fixture.stencils.size());
  for (const auto& stencil : fixture.stencils) {
    const std::size_t tri = 3u * static_cast<std::size_t>(stencil.triangle);
    const auto distance = dc::pointTriangleSquaredDistance(
        pointAt(fixture.positions, stencil.point),
        pointAt(fixture.positions, fixture.triangles[tri]),
        pointAt(fixture.positions, fixture.triangles[tri + 1u]),
        pointAt(fixture.positions, fixture.triangles[tri + 2u]));
    distances.push_back(distance.squaredDistance);
  }
  return distances;
}

} // namespace

//==============================================================================
TEST(ContactCandidateFilterCuda, MatchesCpuPointTriangleStencilFilter)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  const Fixture fixture = makeFixture();
  const std::vector<double> expectedDistances = cpuSquaredDistances(fixture);

  cuda::PointTriangleCandidateFilterResult result;
  cuda::filterPointTriangleContactStencilsCuda(
      fixture.positions, fixture.triangles, fixture.stencils, 0.05, result);

  ASSERT_EQ(result.squaredDistances.size(), expectedDistances.size());
  ASSERT_EQ(result.accepted.size(), expectedDistances.size());
  ASSERT_EQ(result.acceptedCount, 1u);
  EXPECT_NEAR(result.squaredDistances[0], expectedDistances[0], 1e-14);
  EXPECT_NEAR(result.squaredDistances[1], expectedDistances[1], 1e-14);
  EXPECT_EQ(result.accepted[0], 1u);
  EXPECT_EQ(result.accepted[1], 0u);
  EXPECT_GT(result.timing.kernelNs, 0.0);
}

//==============================================================================
TEST(ContactCandidateFilterCuda, KeepsSmallValidTriangleNondegenerate)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  Fixture fixture;
  appendPoint(fixture, {0.0, 0.0, 0.0});
  appendPoint(fixture, {1e-8, 0.0, 0.0});
  appendPoint(fixture, {0.0, 1e-8, 0.0});
  appendPoint(fixture, {2.5e-9, 2.5e-9, 0.0});
  fixture.triangles.insert(fixture.triangles.end(), {0u, 1u, 2u});
  fixture.stencils.push_back({3u, 0u});

  const std::vector<double> expectedDistances = cpuSquaredDistances(fixture);

  cuda::PointTriangleCandidateFilterResult result;
  cuda::filterPointTriangleContactStencilsCuda(
      fixture.positions, fixture.triangles, fixture.stencils, 1e-10, result);

  ASSERT_EQ(result.squaredDistances.size(), expectedDistances.size());
  ASSERT_EQ(result.accepted.size(), expectedDistances.size());
  EXPECT_EQ(result.acceptedCount, 1u);
  EXPECT_NEAR(result.squaredDistances[0], expectedDistances[0], 1e-30);
  EXPECT_EQ(result.accepted[0], 1u);
}

//==============================================================================
TEST(ContactCandidateFilterCuda, RejectsInvalidStencil)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  Fixture fixture = makeFixture();
  fixture.stencils.push_back({99u, 0u});

  cuda::PointTriangleCandidateFilterResult result;
  EXPECT_THROW(
      cuda::filterPointTriangleContactStencilsCuda(
          fixture.positions, fixture.triangles, fixture.stencils, 0.05, result),
      dart::simulation::InvalidArgumentException);
}
