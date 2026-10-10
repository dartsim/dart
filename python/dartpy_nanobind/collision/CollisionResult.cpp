// clang-format off
#include "detail/dart_nb.hpp"
// clang-format on

#include <nanobind/stl/unordered_set.h>
#include <nanobind/stl/vector.h>

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

#include <dart/collision/CollisionResult.hpp>
#include <dart/collision/Contact.hpp>

#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/ShapeFrame.hpp>

#include <unordered_set>
#include <vector>

#include <cstddef>

namespace dart {
namespace python {

void CollisionResult(nb::module_& m)
{
  dartnb::dart_class<dart::collision::CollisionResult>(m, "CollisionResult")
      .def(dartnb::init<>())
      .def(
          "addContact",
          +[](dart::collision::CollisionResult* self,
              const dart::collision::Contact& contact) {
            self->addContact(contact);
          },
          nb::arg("contact"),
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
          nb::arg("index"),
          nb::rv_policy::reference_internal,
          "Return the index-th contact.")
      .def(
          "getContact",
          +[](const dart::collision::CollisionResult* self,
              std::size_t index) -> const dart::collision::Contact& {
            return self->getContact(index);
          },
          nb::arg("index"),
          nb::rv_policy::reference_internal,
          "Return (const) the index-th contact.")
      .def(
          "getContacts",
          +[](const dart::collision::CollisionResult* self)
              -> const std::vector<dart::collision::Contact>& {
            return self->getContacts();
          },
          nb::rv_policy::reference_internal,
          "Return contacts.")
      .def(
          "getCollidingBodyNodes",
          +[](const dart::collision::CollisionResult* self)
              -> const std::unordered_set<const dynamics::BodyNode*>& {
            return self->getCollidingBodyNodes();
          },
          nb::rv_policy::reference_internal,
          "Return the set of BodyNodes that are in collision.")
      .def(
          "getCollidingShapeFrames",
          +[](const dart::collision::CollisionResult* self) -> nb::set {
            return nb::cast<nb::set>(nb::cast(
                self->getCollidingShapeFrames(),
                nb::rv_policy::reference_internal,
                nb::cast(self, nb::rv_policy::reference)));
          },
          nb::rv_policy::reference_internal,
          "Return the set of ShapeFrames that are in collision.")
      .def(
          "inCollision",
          +[](const dart::collision::CollisionResult* self,
              const dart::dynamics::BodyNode* bn) -> bool {
            return self->inCollision(bn);
          },
          nb::arg("bn").none(),
          "Returns true if the given BodyNode is in collision.")
      .def(
          "inCollision",
          +[](const dart::collision::CollisionResult* self,
              const dart::dynamics::ShapeFrame* frame) -> bool {
            return self->inCollision(frame);
          },
          nb::arg("frame").none(),
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
