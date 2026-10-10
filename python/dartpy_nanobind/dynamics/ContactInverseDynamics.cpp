// clang-format off
#include "detail/dart_nb.hpp"
// clang-format on

#include <nanobind/stl/vector.h>

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

#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/ContactInverseDynamics.hpp>
#include <dart/dynamics/Skeleton.hpp>

#include <vector>

#include <cstddef>

namespace dart {
namespace python {

void ContactInverseDynamics(nb::module_& m)
{
  using CID = dart::dynamics::ContactInverseDynamics;

  dartnb::dart_class<CID> cid(m, "ContactInverseDynamics");

  dartnb::dart_class<CID::Contact>(cid, "Contact")
      .def(dartnb::init<>())
      .def_prop_rw(
          "bodyNode",
          +[](const CID::Contact& self) -> dart::dynamics::BodyNodePtr {
            return dart::dynamics::BodyNodePtr(self.bodyNode);
          },
          // keep_alive ties the Python BodyNode (and hence its Skeleton) to
          // the Contact so the stored raw pointer cannot dangle.
          +[](CID::Contact& self, dart::dynamics::BodyNode* bodyNode) {
            self.bodyNode = bodyNode;
          },
          nb::for_setter(nb::keep_alive<1, 2>()),
          nb::for_setter(nb::arg("value").none()))
      .def_rw(
          "localOffset",
          &CID::Contact::localOffset,
          dartnb::setterArgument(&CID::Contact::localOffset))
      .def_rw(
          "normal",
          &CID::Contact::normal,
          dartnb::setterArgument(&CID::Contact::normal))
      .def_rw(
          "frictionCoeff",
          &CID::Contact::frictionCoeff,
          dartnb::setterArgument(&CID::Contact::frictionCoeff))
      .def_rw(
          "numBasis",
          &CID::Contact::numBasis,
          dartnb::setterArgument(&CID::Contact::numBasis));

  dartnb::dart_class<CID::Result>(cid, "Result")
      .def(dartnb::init<>())
      .def_ro("jointForces", &CID::Result::jointForces)
      .def_ro("contactForces", &CID::Result::contactForces)
      .def_ro("unactuatedResidual", &CID::Result::unactuatedResidual)
      .def_ro("feasible", &CID::Result::feasible);

  cid.def(
         dartnb::init<dart::dynamics::SkeletonPtr>(),
         nb::arg("skeleton").none())
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
          nb::arg("contacts"))
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
          nb::arg("regularization"))
      .def(
          "getRegularization",
          +[](const CID* self) -> double { return self->getRegularization(); })
      .def(
          "setResidualTolerance",
          +[](CID* self, double tolerance) {
            self->setResidualTolerance(tolerance);
          },
          nb::arg("tolerance"))
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
          nb::arg("indices"))
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
          nb::arg("withExternalForces") = false,
          nb::arg("withDampingForces") = false,
          nb::arg("withSpringForces") = false);
}

} // namespace python
} // namespace dart
