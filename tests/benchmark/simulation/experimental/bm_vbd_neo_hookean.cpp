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

#include <dart/simulation/experimental/detail/deformable_vbd/neo_hookean.hpp>

#include <Eigen/Core>
#include <benchmark/benchmark.h>

#include <array>

namespace vbd = dart::simulation::experimental::detail::deformable_vbd;

namespace {

using Vec3 = Eigen::Vector3d;
using Tet = std::array<Vec3, 4>;

Tet deformedTet()
{
  Tet tet
      = {Vec3(0.0, 0.0, 0.0),
         Vec3(1.0, 0.0, 0.0),
         Vec3(0.0, 1.0, 0.0),
         Vec3(0.0, 0.0, 1.0)};
  tet[1] += Vec3(0.20, 0.05, -0.02);
  tet[2] += Vec3(-0.04, 0.18, 0.03);
  tet[3] += Vec3(0.02, -0.06, 0.22);
  return tet;
}

} // namespace

//==============================================================================
static void BM_VbdNeoHookeanEnergy(benchmark::State& state)
{
  const Tet tet = deformedTet();
  const vbd::TetRestShape shape = vbd::makeTetRestShape(
      {Vec3(0, 0, 0), Vec3(1, 0, 0), Vec3(0, 1, 0), Vec3(0, 0, 1)});
  for (auto _ : state) {
    const Eigen::Matrix3d F
        = vbd::deformationGradient(shape.restShapeInverse, tet);
    double energy = vbd::stableNeoHookeanEnergyDensity(F, 3000.0, 6000.0);
    benchmark::DoNotOptimize(energy);
  }
}
BENCHMARK(BM_VbdNeoHookeanEnergy);

//==============================================================================
static void BM_VbdNeoHookeanStress(benchmark::State& state)
{
  const Tet tet = deformedTet();
  const vbd::TetRestShape shape = vbd::makeTetRestShape(
      {Vec3(0, 0, 0), Vec3(1, 0, 0), Vec3(0, 1, 0), Vec3(0, 0, 1)});
  const Eigen::Matrix3d F
      = vbd::deformationGradient(shape.restShapeInverse, tet);
  for (auto _ : state) {
    Eigen::Matrix3d stress = vbd::stableNeoHookeanStress(F, 3000.0, 6000.0);
    benchmark::DoNotOptimize(stress);
  }
}
BENCHMARK(BM_VbdNeoHookeanStress);

//==============================================================================
// Full per-vertex force + Hessian block for one tetrahedron vertex (the cost of
// one tet's contribution to a VBD vertex update).
static void BM_VbdNeoHookeanVertexBlock(benchmark::State& state)
{
  const Tet tet = deformedTet();
  const vbd::TetRestShape shape = vbd::makeTetRestShape(
      {Vec3(0, 0, 0), Vec3(1, 0, 0), Vec3(0, 1, 0), Vec3(0, 0, 1)});
  for (auto _ : state) {
    vbd::VertexBlock block;
    vbd::addNeoHookeanTetTerm(block, 0, shape, tet, 3000.0, 6000.0);
    benchmark::DoNotOptimize(block.force);
    benchmark::DoNotOptimize(block.hessian);
    benchmark::ClobberMemory();
  }
}
BENCHMARK(BM_VbdNeoHookeanVertexBlock);

BENCHMARK_MAIN();
