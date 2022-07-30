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

#include "dart/multibody/all.hpp"

#include "include_pybind11.h"

namespace py = pybind11;

namespace dart::python {

void py_frame(py::module& m)
{
  py::class_<multibody::Framed, std::shared_ptr<multibody::Framed>>(m, "Frame")
      .def(
          "set_name",
          &multibody::Framed::set_name,
          py::arg("name"),
          "Sets the frame name.")
      .def("get_name", &multibody::Framed::get_name, "Returns the frame name.")
      .def(
          "is_inertial_frame",
          &multibody::Framed::is_inertial_frame,
          "Returns whether this frame is the inertial frame.")
      .def(
          "get_pose",
          py::overload_cast<>(&multibody::Framed::get_pose, py::const_),
          "Returns the global pose in the coordinates of the inertial frame.")
      .def(
          "get_pose",
          py::overload_cast<const multibody::Framed*>(
              &multibody::Framed::get_pose, py::const_),
          py::arg("with_respect_to"),
          "Returns the global pose in the coordinates of the inertial frame.")
      .def(
          "get_orientation",
          [](const multibody::Framed* self) -> math::SO3d {
            return self->get_orientation();
          })
      .def(
          "get_position",
          [](const multibody::Framed* self) -> math::R3d {
            return self->get_position();
          })
      .def(
          "get_spatial_velocity",
          py::overload_cast<>(
              &multibody::Framed::get_spatial_velocity, py::const_))
      .def(
          "get_angular_velocity",
          &multibody::Framed::get_angular_velocity,
          py::arg("relative_to") = multibody::Framed::GetInertialFrame(),
          py::arg("in_coordinates_of") = multibody::Framed::GetInertialFrame())
      .def(
          "get_linear_velocity",
          py::overload_cast<const multibody::Framed*, const multibody::Framed*>(
              &multibody::Framed::get_linear_velocity, py::const_),
          py::arg("relative_to") = multibody::Framed::GetInertialFrame(),
          py::arg("in_coordinates_of") = multibody::Framed::GetInertialFrame())
      .def(
          "get_spatial_acceleration",
          &multibody::Framed::get_spatial_acceleration)
      .def(
          "get_angular_acceleration",
          &multibody::Framed::get_angular_acceleration)
      .def(
          "get_linear_acceleration",
          &multibody::Framed::get_linear_acceleration);

  // InertialFrame
  py::class_<
      multibody::InertialFramed,
      multibody::Framed,
      std::shared_ptr<multibody::InertialFramed>>(m, "InertialFrame")
      .def(py::init([]() -> std::shared_ptr<multibody::InertialFramed> {
        struct EnableMakeShared : multibody::InertialFramed
        {
          EnableMakeShared() = default;
        };
        static auto inertial_frame = std::make_shared<EnableMakeShared>();
        return inertial_frame;
      }))
      .def_static("GetName", &multibody::InertialFramed::GetName);

  // RelativeFrame
  ::py::class_<
      multibody::RelativeFramed,
      multibody::Framed,
      std::shared_ptr<multibody::RelativeFramed>>(m, "RelativeFrame");

  // FreeRelativeFrame
  ::py::class_<
      multibody::FreeRelativeFramed,
      multibody::RelativeFramed,
      std::shared_ptr<multibody::FreeRelativeFramed>>(m, "FreeRelativeFrame")
      .def(::py::init<>());
}

} // namespace dart::python
