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

// First ABD performance packet for PLAN-083. These rows measure the internal
// affine primitive mapping foundation against a matched rigid IPC oracle row;
// they are not paper-scale performance claims.

#include <dart/simulation/detail/affine_body_dynamics.hpp>
#include <dart/simulation/detail/rigid_ipc_barrier.hpp>

#include <Eigen/Core>
#include <benchmark/benchmark.h>

namespace sxdetail = dart::simulation::detail;

namespace {

//==============================================================================
sxdetail::AffineBarrierOptions activeAffineBarrierOptions()
{
  sxdetail::AffineBarrierOptions options;
  options.squaredActivationDistance = 1.0;
  options.stiffness = 1.0;
  options.projectHessianToPsd = true;
  return options;
}

//==============================================================================
sxdetail::RigidIpcBarrierOptions activeRigidBarrierOptions()
{
  sxdetail::RigidIpcBarrierOptions options;
  options.squaredActivationDistance = 1.0;
  options.stiffness = 1.0;
  options.projectReducedHessianToPsd = true;
  return options;
}

//==============================================================================
sxdetail::AffineBodyState makeAffineBody(const Eigen::Vector3d& translation)
{
  sxdetail::AffineBodyState state;
  state.translation = translation;
  return state;
}

//==============================================================================
void consumeAffineBarrier(sxdetail::AffinePrimitiveBarrierResult& result)
{
  benchmark::DoNotOptimize(result.value);
  benchmark::DoNotOptimize(result.gradient);
  benchmark::DoNotOptimize(result.hessian);
  benchmark::ClobberMemory();
}

//==============================================================================
void consumeOrthogonality(sxdetail::AffineOrthogonalityEnergyResult& result)
{
  benchmark::DoNotOptimize(result.value);
  benchmark::DoNotOptimize(result.gradient);
  benchmark::DoNotOptimize(result.hessian);
  benchmark::ClobberMemory();
}

//==============================================================================
void consumeRigidReduced(sxdetail::RigidIpcReducedBarrierResult& result)
{
  benchmark::DoNotOptimize(result.value);
  benchmark::DoNotOptimize(result.gradient);
  benchmark::DoNotOptimize(result.hessian);
  benchmark::ClobberMemory();
}

} // namespace

//==============================================================================
static void BM_AffineBodyPointTriangleBarrier(benchmark::State& state)
{
  const sxdetail::AffineBodyState pointBody
      = makeAffineBody(Eigen::Vector3d(0.0, 0.0, 0.48));
  const sxdetail::AffineBodyState triangleBody
      = makeAffineBody(Eigen::Vector3d::Zero());
  const Eigen::Vector3d point(0.22, 0.18, 0.0);
  const Eigen::Vector3d triangleA(0.0, 0.0, 0.0);
  const Eigen::Vector3d triangleB(1.0, 0.0, 0.0);
  const Eigen::Vector3d triangleC(0.0, 1.0, 0.0);
  const auto options = activeAffineBarrierOptions();

  sxdetail::AffinePrimitiveBarrierResult result;
  for (auto _ : state) {
    result = sxdetail::affinePointTriangleBarrier(
        pointBody,
        point,
        triangleBody,
        triangleA,
        triangleB,
        triangleC,
        options);
    consumeAffineBarrier(result);
  }
}
BENCHMARK(BM_AffineBodyPointTriangleBarrier);

//==============================================================================
static void BM_AffineBodyRigidIpcPointTriangleOracle(benchmark::State& state)
{
  const Eigen::Vector3d point(0.22, 0.18, 0.0);
  const sxdetail::RigidIpcPose pointPose{
      Eigen::Vector3d(0.0, 0.0, 0.48), Eigen::Vector3d::Zero()};
  const Eigen::Vector3d triangleA(0.0, 0.0, 0.0);
  const Eigen::Vector3d triangleB(1.0, 0.0, 0.0);
  const Eigen::Vector3d triangleC(0.0, 1.0, 0.0);
  const sxdetail::RigidIpcPose trianglePose;
  const auto options = activeRigidBarrierOptions();

  sxdetail::RigidIpcReducedBarrierResult result;
  for (auto _ : state) {
    result = sxdetail::rigidIpcPointTriangleReducedBarrier(
        point,
        pointPose,
        triangleA,
        triangleB,
        triangleC,
        trianglePose,
        options);
    consumeRigidReduced(result);
  }
}
BENCHMARK(BM_AffineBodyRigidIpcPointTriangleOracle);

//==============================================================================
static void BM_AffineBodyOrthogonalityEnergy(benchmark::State& state)
{
  sxdetail::AffineBodyState body = makeAffineBody(Eigen::Vector3d::Zero());
  body.linearMap << 1.02, 0.03, -0.01, -0.02, 0.97, 0.04, 0.01, -0.03, 1.05;
  constexpr double kStiffness = 10.0;

  sxdetail::AffineOrthogonalityEnergyResult result;
  for (auto _ : state) {
    result = sxdetail::affineOrthogonalityEnergy(body, kStiffness);
    consumeOrthogonality(result);
  }
}
BENCHMARK(BM_AffineBodyOrthogonalityEnergy);
