/*
 * Copyright (c) 2011-2022, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#include "dart/collision/all.hpp"

#include "include_pybind11.h"

namespace py = pybind11;

namespace dart::python {

void py_engine(py::module& m)
{
  py::class_<collision::Engined, std::shared_ptr<collision::Engined>>(
      m, "Engine")
      .def(
          py::init(
              [](const std::string& engine_name)
                  -> std::shared_ptr<collision::Engined> {
                if (engine_name == collision::DartEngined::GetType()) {
                  return collision::DartEngined::Create();
#if DART_HAVE_fcl
                } else if (engine_name == collision::FclEngined::GetType()) {
                  return collision::FclEngined::Create();
#endif
                } else {
                  DART_WARN(
                      "Unsupported collision engine '{}'. Creating DART "
                      "collision engine.",
                      engine_name);
                  return collision::DartEngined::Create();
                }
              }),
          py::arg("engine_name") = collision::DartEngined::GetType())
      .def("get_type", &collision::Engined::get_type)
      .def(
          "create_scene",
          &collision::Engined::create_scene,
          py::return_value_policy::reference_internal)
      .def(
          "create_sphere_object",
          [](collision::Engined* self,
             double radius) -> collision::Object<double>* {
            return self->create_sphere_object(radius);
          },
          py::arg("radius") = 0.5,
          py::return_value_policy::reference_internal)
      .def(
          "collide",
          &collision::Engined::collide,
          py::arg("object_a"),
          py::arg("object_b"),
          py::arg("option") = collision::CollisionOptiond(),
          py::arg("result") = nullptr);
}

} // namespace dart::python
