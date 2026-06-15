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

// Headless driver for the "boxes" scene. It serves two evidence-based roles
// with no rendering in the measured loop:
//
//   1. Determinism / correctness oracle. It prints a high-precision checksum of
//      every skeleton's positions and velocities at fixed checkpoints, so an
//      optimized build can be proven to produce byte-for-byte identical
//      simulation results as a baseline build (`diff` the two outputs).
//
//   2. Headless profiling target. When DART is built with -DDART_BUILD_PROFILE,
//      it dumps the built-in text profiler summary at the end so hot paths can
//      be inspected without a GUI.
//
// Usage: boxes_headless [dim=8] [steps=2000] [checkpoint=500]

#include "boxes_scene.hpp"

#include <dart/common/Profile.hpp>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <string>

using namespace dart;

namespace {

struct Checksum
{
  double posL1 = 0.0;
  double posSq = 0.0;
  double velL1 = 0.0;
  double velSq = 0.0;
  std::size_t dofs = 0;
};

[[nodiscard]] Checksum computeChecksum(const simulation::WorldPtr& world)
{
  Checksum c;
  for (std::size_t i = 0; i < world->getNumSkeletons(); ++i) {
    const auto skel = world->getSkeleton(i);
    const Eigen::VectorXd q = skel->getPositions();
    const Eigen::VectorXd dq = skel->getVelocities();
    c.posL1 += q.cwiseAbs().sum();
    c.posSq += q.squaredNorm();
    c.velL1 += dq.cwiseAbs().sum();
    c.velSq += dq.squaredNorm();
    c.dofs += static_cast<std::size_t>(q.size());
  }
  return c;
}

void printChecksum(std::size_t step, const Checksum& c)
{
  // %.17g round-trips a double exactly, enabling bit-for-bit comparison.
  std::printf(
      "step %6zu  dofs %5zu  posL1 %.17g  posSq %.17g  velL1 %.17g  velSq "
      "%.17g\n",
      step,
      c.dofs,
      c.posL1,
      c.posSq,
      c.velL1,
      c.velSq);
}

} // namespace

int main(int argc, char** argv)
{
  const std::size_t dim
      = argc > 1 ? static_cast<std::size_t>(std::atoi(argv[1])) : 8u;
  const std::size_t steps
      = argc > 2 ? static_cast<std::size_t>(std::atoi(argv[2])) : 2000u;
  const std::size_t checkpoint
      = argc > 3 ? static_cast<std::size_t>(std::atoi(argv[3])) : 500u;

  auto world = test::createBoxesWorld(dim);

  std::printf(
      "# boxes_headless dim=%zu steps=%zu boxes=%zu\n",
      dim,
      steps,
      dim * dim * dim);

  const auto t0 = std::chrono::steady_clock::now();
  for (std::size_t s = 0; s < steps; ++s) {
    world->step();
    const std::size_t done = s + 1;
    if ((checkpoint != 0 && done % checkpoint == 0) || done == steps)
      printChecksum(done, computeChecksum(world));
  }
  const auto t1 = std::chrono::steady_clock::now();
  const double elapsedMs
      = std::chrono::duration<double, std::milli>(t1 - t0).count();
  // Stepping-loop wall time only (world construction excluded), so this is a
  // clean signal for A/B before/after comparison of the physics hot path.
  std::printf(
      "elapsed_ms %.3f  steps_per_s %.1f\n",
      elapsedMs,
      elapsedMs > 0.0 ? (1000.0 * static_cast<double>(steps) / elapsedMs)
                      : 0.0);

  // No-op unless DART was built with -DDART_BUILD_PROFILE (text backend on).
  DART_PROFILE_TEXT_DUMP();

  return 0;
}
