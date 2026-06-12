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

#include "eigen_geometry_pybind.h"
#include "eigen_pybind.h"
#include "pointers.hpp"

#include <dart/dart.hpp>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

namespace dart {
namespace python {

void ContactInverseDynamics(py::module& m)
{
  using CID = dart::dynamics::ContactInverseDynamics;

  ::py::class_<CID> cid(m, "ContactInverseDynamics");

  ::py::class_<CID::Contact>(cid, "Contact")
      .def(::py::init<>())
      .def_property(
          "bodyNode",
          +[](const CID::Contact& self) -> dart::dynamics::BodyNodePtr {
            return dart::dynamics::BodyNodePtr(self.bodyNode);
          },
          // keep_alive ties the Python BodyNode (and hence its Skeleton) to
          // the Contact so the stored raw pointer cannot dangle.
          ::py::cpp_function(
              +[](CID::Contact& self, dart::dynamics::BodyNode* bodyNode) {
                self.bodyNode = bodyNode;
              },
              ::py::keep_alive<1, 2>()))
      .def_readwrite("localOffset", &CID::Contact::localOffset)
      .def_readwrite("normal", &CID::Contact::normal)
      .def_readwrite("frictionCoeff", &CID::Contact::frictionCoeff)
      .def_readwrite("numBasis", &CID::Contact::numBasis);

  ::py::class_<CID::Result>(cid, "Result")
      .def(::py::init<>())
      .def_readonly("jointForces", &CID::Result::jointForces)
      .def_readonly("contactForces", &CID::Result::contactForces)
      .def_readonly("unactuatedResidual", &CID::Result::unactuatedResidual)
      .def_readonly("feasible", &CID::Result::feasible);

  cid.def(::py::init<dart::dynamics::SkeletonPtr>(), ::py::arg("skeleton"))
      .def(
          "getSkeleton",
          +[](const CID* self) -> dart::dynamics::SkeletonPtr {
            return self->getSkeleton();
          })
      .def(
          "setContacts",
          +[](CID* self, const std::vector<CID::Contact>& contacts) {
            self->setContacts(contacts);
          },
          ::py::arg("contacts"))
      .def(
          "getContacts",
          +[](const CID* self) -> std::vector<CID::Contact> {
            return self->getContacts();
          })
      .def(
          "setRegularization",
          +[](CID* self, double regularization) {
            self->setRegularization(regularization);
          },
          ::py::arg("regularization"))
      .def(
          "getRegularization",
          +[](const CID* self) -> double { return self->getRegularization(); })
      .def(
          "setResidualTolerance",
          +[](CID* self, double tolerance) {
            self->setResidualTolerance(tolerance);
          },
          ::py::arg("tolerance"))
      .def(
          "getResidualTolerance",
          +[](const CID* self) -> double {
            return self->getResidualTolerance();
          })
      .def(
          "setUnactuatedDofs",
          +[](CID* self, const std::vector<std::size_t>& indices) {
            self->setUnactuatedDofs(indices);
          },
          ::py::arg("indices"))
      .def(
          "getUnactuatedDofs",
          +[](const CID* self) -> std::vector<std::size_t> {
            return self->getUnactuatedDofs();
          })
      .def(
          "compute",
          +[](CID* self,
              bool withExternalForces,
              bool withDampingForces,
              bool withSpringForces) -> CID::Result {
            return self->compute(
                withExternalForces, withDampingForces, withSpringForces);
          },
          ::py::arg("withExternalForces") = false,
          ::py::arg("withDampingForces") = false,
          ::py::arg("withSpringForces") = false);
}

} // namespace python
} // namespace dart
