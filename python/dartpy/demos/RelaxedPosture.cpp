// clang-format off
#include "detail/dart_nb.hpp"
// clang-format on

#include "detail/eigen.hpp"

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

#include "demo_models/RelaxedPosture.hpp"

#include <memory>

namespace dart::python {
namespace {

void requirePosture(
    const dart_demos::RelaxedPosture& posture, const Eigen::VectorXd& x)
{
  if (x.size() != posture.getNumDofs() || !x.allFinite())
    throw nb::value_error("posture must contain one finite value per DOF");
}

} // namespace

void bind_relaxed_posture(nb::module_& demos)
{
  using Posture = dart_demos::RelaxedPosture;
  dartnb::dart_class<Posture, dart::optimizer::Function>(
      demos, "RelaxedPosture")
      .def(
          dartnb::factory([](const Eigen::VectorXd& ideal,
                             const Eigen::VectorXd& lower,
                             const Eigen::VectorXd& upper,
                             const Eigen::VectorXd& weights) {
            if (lower.size() != ideal.size() || upper.size() != ideal.size()
                || weights.size() != ideal.size())
              throw nb::value_error("posture vectors must have equal lengths");
            if (!ideal.allFinite() || !weights.allFinite()
                || (weights.array() < 0).any() || lower.array().isNaN().any()
                || upper.array().isNaN().any()
                || (lower.array() > upper.array()).any())
              throw nb::value_error(
                  "posture must be finite, with nonnegative weights and "
                  "ordered bounds");
            return std::make_shared<Posture>(ideal, lower, upper, weights);
          }),
          nb::arg("ideal_posture"),
          nb::arg("lower"),
          nb::arg("upper"),
          nb::arg("weights"))
      .def_rw("enforce_ideal_posture", &Posture::enforceIdealPosture)
      .def(
          "eval",
          [](Posture& self, const Eigen::VectorXd& x) {
            requirePosture(self, x);
            return self.eval(x);
          },
          nb::arg("posture"))
      .def(
          "eval_gradient",
          [](Posture& self, const Eigen::VectorXd& x) {
            requirePosture(self, x);
            Eigen::VectorXd gradient(x.size());
            self.evalGradient(
                x,
                Eigen::Map<Eigen::VectorXd>(gradient.data(), gradient.size()));
            return gradient;
          },
          nb::arg("posture"));
}

} // namespace dart::python
