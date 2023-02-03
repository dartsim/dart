/*
 * Copyright (c) 2011-2023, The DART development contributors
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

#include <dart/dynamics/dynamics.hpp>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

namespace dart {
namespace python {

void CollisionResult(py::module& m)
{
  ::py::class_<dart::collision::CollisionResult>(m, "CollisionResult")
      .def(::py::init<>())
      .def(
          "addContact",
          +[](dart::collision::CollisionResult* self,
              const dart::collision::Contact& contact) {
            self->addContact(contact);
          },
          ::py::arg("contact"),
          "Add one contact.")
      .def(
          "getNumContacts",
          +[](const dart::collision::CollisionResult* self) -> std::size_t {
            return self->getNumContacts();
          },
          "Return number of contacts.")
      .def(
          "getContact",
          +[](dart::collision::CollisionResult* self, std::size_t index)
              -> dart::collision::Contact& { return self->getContact(index); },
          ::py::arg("index"),
          ::py::return_value_policy::reference,
          "Return the index-th contact.")
      .def(
          "getContact",
          +[](const dart::collision::CollisionResult* self,
              std::size_t index) -> const dart::collision::Contact& {
            return self->getContact(index);
          },
          ::py::arg("index"),
          ::py::return_value_policy::reference,
          "Return (const) the index-th contact.")
      .def(
          "getContacts",
          +[](const dart::collision::CollisionResult* self)
              -> const std::vector<dart::collision::Contact>& {
            return self->getContacts();
          },
          ::py::return_value_policy::reference,
          "Return contacts.")
      .def(
          "getCollidingBodyNodes",
          +[](const dart::collision::CollisionResult* self)
              -> const std::unordered_set<const dynamics::BodyNode*>& {
            return self->getCollidingBodyNodes();
          },
          ::py::return_value_policy::reference,
          "Return the set of BodyNodes that are in collision.")
      .def(
          "getCollidingShapeFrames",
          +[](const dart::collision::CollisionResult* self)
              -> const std::unordered_set<const dynamics::ShapeFrame*>& {
            return self->getCollidingShapeFrames();
          },
          ::py::return_value_policy::reference,
          "Return the set of ShapeFrames that are in collision.")
      .def(
          "inCollision",
          +[](const dart::collision::CollisionResult* self,
              const dart::dynamics::BodyNode* bn) -> bool {
            return self->inCollision(bn);
          },
          ::py::arg("bn"),
          "Returns true if the given BodyNode is in collision.")
      .def(
          "inCollision",
          +[](const dart::collision::CollisionResult* self,
              const dart::dynamics::ShapeFrame* frame) -> bool {
            return self->inCollision(frame);
          },
          ::py::arg("frame"),
          "Returns true if the given ShapeFrame is in collision.")
      .def(
          "isCollision",
          +[](const dart::collision::CollisionResult* self) -> bool {
            return self->isCollision();
          },
          "Return binary collision result.")
      .def(
          "clear",
          +[](dart::collision::CollisionResult* self) { self->clear(); },
          "Clear all the contacts.");
}

} // namespace python
} // namespace dart
