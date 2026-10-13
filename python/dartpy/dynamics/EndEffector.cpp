// clang-format off
#include "detail/dart_nb.hpp"
// clang-format on

#include "detail/eigen.hpp"

/*
 * Copyright (c) 2011-2026, The DART development contributors
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

#include <dart/dynamics/EndEffector.hpp>
#include <dart/dynamics/JacobianNode.hpp>

#include <nanobind/stl/vector.h>

namespace dart {
namespace python {

void EndEffector(nb::module_& m)
{
  dartnb::dart_class<dart::dynamics::Support>(m, "Support")
      .def(
          "setGeometry",
          +[](dart::dynamics::Support* self,
              const dart::math::SupportGeometry& points) {
            self->setGeometry(points);
          },
          nb::arg("points"))
      .def(
          "getGeometry",
          +[](const dart::dynamics::Support* self)
              -> dart::math::SupportGeometry { return self->getGeometry(); })
      .def(
          "setActive",
          +[](dart::dynamics::Support* self, bool active) {
            self->setActive(active);
          },
          nb::arg("active") = true)
      .def(
          "isActive", +[](const dart::dynamics::Support* self) {
            return self->isActive();
          });

  dartnb::dart_class<dart::dynamics::EndEffector, dart::dynamics::JacobianNode>(
      m, "EndEffector")
      .def(
          "setRelativeTransform",
          +[](dart::dynamics::EndEffector* self, const Eigen::Isometry3d& tf) {
            self->setRelativeTransform(tf);
          },
          nb::arg("tf"))
      .def(
          "setDefaultRelativeTransform",
          +[](dart::dynamics::EndEffector* self,
              const Eigen::Isometry3d& tf,
              bool useNow) { self->setDefaultRelativeTransform(tf, useNow); },
          nb::arg("tf"),
          nb::arg("useNow") = false)
      .def(
          "resetRelativeTransform",
          +[](dart::dynamics::EndEffector* self) {
            self->resetRelativeTransform();
          })
      .def(
          "getSupport",
          +[](dart::dynamics::EndEffector* self,
              bool createIfNull) -> dart::dynamics::Support* {
            return self->getSupport(createIfNull);
          },
          nb::arg("createIfNull") = false,
          nb::rv_policy::reference_internal,
          nb::sig("def getSupport(self, createIfNull: bool = False) -> "
                  "dartpy.dynamics.Support | None"))
      .def(
          "hasSupport", +[](const dart::dynamics::EndEffector* self) {
            return self->hasSupport();
          });
}

} // namespace python
} // namespace dart
