// clang-format off
#include "detail/dart_nb.hpp"
// clang-format on

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
 *     copyright notice, this list of conditions and the following disclaimer
 *     in the documentation and/or other materials provided with the
 *     distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "demo_models/SoftWormModel.hpp"

#include <stdexcept>

namespace dart::python {
namespace soft_worm_binding {

void requireWorm(const dart::dynamics::SkeletonPtr& worm)
{
  if (!worm)
    throw std::invalid_argument("worm must not be null");
}

} // namespace soft_worm_binding

void bind_soft_worm(nb::module_& demos)
{
  namespace model = dart_demos::soft_worm_model;
  auto m = demos.def_submodule(
      "soft_worm", "Private GUI-free five-link soft-worm model.");

  m.attr("num_links") = model::kNumLinks;
  m.attr("gait_amplitude") = model::kGaitAmplitude;
  m.attr("gait_frequency_hz") = model::kGaitFrequencyHz;

  m.def(
      "create_world",
      [] {
        dart::dynamics::SkeletonPtr worm;
        auto world = model::createWorld(worm);
        return nb::make_tuple(world, worm);
      },
      "Build the soft-worm scene and return its (world, worm) owners.");
  m.def(
      "apply_gait",
      [](const dart::dynamics::SkeletonPtr& worm, double time, bool enabled) {
        soft_worm_binding::requireWorm(worm);
        model::applyGait(worm, time, enabled);
      },
      nb::arg("worm"),
      nb::arg("time"),
      nb::arg("enabled"),
      "Apply the traveling-wave servo commands for the simulation time.");
  m.def(
      "root_x",
      [](const dart::dynamics::SkeletonPtr& worm) {
        soft_worm_binding::requireWorm(worm);
        if (worm->getNumBodyNodes() == 0)
          throw std::invalid_argument("worm must have a root body");
        return model::getRootX(worm);
      },
      nb::arg("worm"),
      "Return the root body's world-frame x coordinate.");
  m.def(
      "position_checksum",
      [](const dart::dynamics::SkeletonPtr& worm) {
        soft_worm_binding::requireWorm(worm);
        return model::positionChecksum(worm);
      },
      nb::arg("worm"),
      "Return the deterministic weighted skeleton and soft-point checksum.");
  m.def(
      "is_finite",
      [](const dart::dynamics::SkeletonPtr& worm) {
        soft_worm_binding::requireWorm(worm);
        return model::isFinite(worm);
      },
      nb::arg("worm"),
      "Return whether the skeleton and soft-point state is finite.");
}

} // namespace dart::python
