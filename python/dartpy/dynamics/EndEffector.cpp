/*
 * Copyright (c) 2011-2025, The DART development contributors
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

#include "eigen_geometry_pybind.h"
#include "eigen_pybind.h"
#include "pointers.hpp"

#include <dart/All.hpp>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

namespace dart {
namespace python {

void EndEffector(py::module& m)
{
  ::py::class_<dart::dynamics::Support>(m, "Support")
      .def(
          "setGeometry",
          +[](dart::dynamics::Support* self,
              const dart::math::SupportGeometry& geometry) {
            self->setGeometry(geometry);
          },
          ::py::arg("geometry"))
      .def(
          "getGeometry",
          +[](const dart::dynamics::Support* self)
              -> dart::math::SupportGeometry {
            return self->getGeometry();
          })
      .def(
          "setActive",
          +[](dart::dynamics::Support* self, bool supporting) {
            self->setActive(supporting);
          },
          ::py::arg("supporting") = true)
      .def(
          "isActive",
          +[](const dart::dynamics::Support* self) -> bool {
            return self->isActive();
          });

  ::py::class_<
      dart::dynamics::EndEffector,
      dart::dynamics::JacobianNode,
      std::shared_ptr<dart::dynamics::EndEffector>>(m, "EndEffector")
      .def(
          "setDefaultRelativeTransform",
          +[](dart::dynamics::EndEffector* self,
              const Eigen::Isometry3d& transform,
              bool useNow) {
            self->setDefaultRelativeTransform(transform, useNow);
          },
          ::py::arg("transform"),
          ::py::arg("useNow") = false)
      .def(
          "resetRelativeTransform",
          +[](dart::dynamics::EndEffector* self) {
            self->resetRelativeTransform();
          })
      .def(
          "createSupport",
          +[](dart::dynamics::EndEffector* self)
              -> dart::dynamics::Support* {
            return self->createSupport();
          },
          ::py::return_value_policy::reference_internal)
      .def(
          "getSupport",
          +[](dart::dynamics::EndEffector* self)
              -> dart::dynamics::Support* { return self->getSupport(); },
          ::py::return_value_policy::reference_internal)
      .def(
          "getSupport",
          +[](dart::dynamics::EndEffector* self, bool createIfNull)
              -> dart::dynamics::Support* {
            return self->getSupport(createIfNull);
          },
          ::py::return_value_policy::reference_internal,
          ::py::arg("createIfNull"))
      .def(
          "hasSupport",
          +[](const dart::dynamics::EndEffector* self) -> bool {
            return self->hasSupport();
          })
      .def(
          "removeSupport",
          +[](dart::dynamics::EndEffector* self) { self->removeSupport(); });
}

} // namespace python
} // namespace dart
