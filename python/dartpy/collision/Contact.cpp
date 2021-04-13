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
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace dart {
namespace python {

void Contact(py::module& m)
{
  ::py::class_<dart::collision::Contact>(m, "Contact")
      .def(::py::init<>())
      .def_static(
          "getNormalEpsilon",
          +[]() -> double {
            return dart::collision::Contact::getNormalEpsilon();
          })
      .def_static(
          "getNormalEpsilonSquared",
          +[]() -> double {
            return dart::collision::Contact::getNormalEpsilonSquared();
          })
      .def_static(
          "isZeroNormal",
          +[](const Eigen::Vector3d& normal) -> bool {
            return dart::collision::Contact::isZeroNormal(normal);
          },
          ::py::arg("normal"))
      .def_static(
          "isNonZeroNormal",
          +[](const Eigen::Vector3d& normal) -> bool {
            return dart::collision::Contact::isNonZeroNormal(normal);
          },
          ::py::arg("normal"))
      .def_readwrite("point", &dart::collision::Contact::point)
      .def_readwrite("normal", &dart::collision::Contact::normal)
      .def_readwrite("force", &dart::collision::Contact::force)
      .def_readwrite(
          "collisionObject1", &dart::collision::Contact::collisionObject1)
      .def_readwrite(
          "collisionObject2", &dart::collision::Contact::collisionObject2)
      .def_readwrite(
          "penetrationDepth", &dart::collision::Contact::penetrationDepth)
      .def_readwrite("triID1", &dart::collision::Contact::triID1)
      .def_readwrite("triID2", &dart::collision::Contact::triID2)
      .def_readwrite("userData", &dart::collision::Contact::userData);
}

} // namespace python
} // namespace dart
