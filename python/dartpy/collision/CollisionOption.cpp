/*
 * Copyright (c) 2011-2021, The DART development contributors
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

#include <dart/dart.hpp>
#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace dart {
namespace python {

void CollisionOption(py::module& m)
{
  ::py::class_<dart::collision::CollisionOption>(m, "CollisionOption")
      .def(::py::init<>())
      .def(::py::init<bool>(), ::py::arg("enableContact"))
      .def(
          ::py::init<bool, std::size_t>(),
          ::py::arg("enableContact"),
          ::py::arg("maxNumContacts"))
      .def(
          ::py::init<
              bool,
              std::size_t,
              const std::shared_ptr<dart::collision::CollisionFilter>&>(),
          ::py::arg("enableContact"),
          ::py::arg("maxNumContacts"),
          ::py::arg("collisionFilter"))
      .def_readwrite(
          "enableContact", &dart::collision::CollisionOption::enableContact)
      .def_readwrite(
          "maxNumContacts", &dart::collision::CollisionOption::maxNumContacts)
      .def_readwrite(
          "collisionFilter",
          &dart::collision::CollisionOption::collisionFilter);
}

} // namespace python
} // namespace dart
