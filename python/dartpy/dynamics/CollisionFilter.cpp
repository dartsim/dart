/*
 * Copyright (c) 2011-2021, The DART development contributors
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

#include <dart/dart.hpp>
#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace dart {
namespace python {

template <class CollisionFilterBase = dart::dynamics::CollisionFilter>
class PyCollisionFilter : public CollisionFilterBase {
public:
  using CollisionFilterBase::CollisionFilterBase; // Inherit constructors

  bool ignoresCollision(
      const dart::dynamics::CollisionObject* object1,
      const dart::dynamics::CollisionObject* object2) const override {
    PYBIND11_OVERLOAD_PURE(
        bool,
        CollisionFilterBase,
        ignoresCollision,
        object1,
        object2,
        "Returns true if the given two CollisionObjects should be checked by "
        "the collision detector, false otherwise.");
  }
};

void CollisionFilter(py::module& m) {
  ::py::class_<
      dart::dynamics::CollisionFilter,
      PyCollisionFilter<>,
      std::shared_ptr<dart::dynamics::CollisionFilter>>(m, "CollisionFilter");

  ::py::class_<
      dart::dynamics::CompositeCollisionFilter,
      PyCollisionFilter<dart::dynamics::CompositeCollisionFilter>,
      dart::dynamics::CollisionFilter,
      std::shared_ptr<dart::dynamics::CompositeCollisionFilter>>(
      m, "CompositeCollisionFilter")
      .def(::py::init<>())
      .def(
          "addCollisionFilter",
          +[](dart::dynamics::CompositeCollisionFilter* self,
              const dart::dynamics::CollisionFilter* filter) {
            self->addCollisionFilter(filter);
          },
          ::py::arg("filter"),
          "Adds a collision filter to this CompositeCollisionFilter.")
      .def(
          "removeCollisionFilter",
          +[](dart::dynamics::CompositeCollisionFilter* self,
              const dart::dynamics::CollisionFilter* filter) {
            self->removeCollisionFilter(filter);
          },
          ::py::arg("filter"),
          "Removes a collision filter from this CompositeCollisionFilter.")
      .def(
          "removeAllCollisionFilters",
          +[](dart::dynamics::CompositeCollisionFilter* self) {
            self->removeAllCollisionFilters();
          },
          "Removes all the collision filters from this "
          "CompositeCollisionFilter.");

  ::py::class_<
      dart::dynamics::BodyNodeCollisionFilter,
      PyCollisionFilter<dart::dynamics::BodyNodeCollisionFilter>,
      dart::dynamics::CollisionFilter,
      std::shared_ptr<dart::dynamics::BodyNodeCollisionFilter>>(
      m, "BodyNodeCollisionFilter")
      .def(::py::init<>())
      .def(
          "addBodyNodePairToBlackList",
          +[](dart::dynamics::BodyNodeCollisionFilter* self,
              const dart::dynamics::BodyNode* bodyNode1,
              const dart::dynamics::BodyNode* bodyNode2) {
            self->addBodyNodePairToBlackList(bodyNode1, bodyNode2);
          },
          ::py::arg("bodyNode1"),
          ::py::arg("bodyNode2"),
          "Add a BodyNode pair to the blacklist.")
      .def(
          "removeBodyNodePairFromBlackList",
          +[](dart::dynamics::BodyNodeCollisionFilter* self,
              const dart::dynamics::BodyNode* bodyNode1,
              const dart::dynamics::BodyNode* bodyNode2) {
            self->removeBodyNodePairFromBlackList(bodyNode1, bodyNode2);
          },
          ::py::arg("bodyNode1"),
          ::py::arg("bodyNode2"),
          "Remove a BodyNode pair from the blacklist.")
      .def(
          "removeAllBodyNodePairsFromBlackList",
          +[](dart::dynamics::BodyNodeCollisionFilter* self) {
            self->removeAllBodyNodePairsFromBlackList();
          },
          "Remove all the BodyNode pairs from the blacklist.")
      .def(
          "ignoresCollision",
          +[](const dart::dynamics::BodyNodeCollisionFilter* self,
              const dart::dynamics::CollisionObject* object1,
              const dart::dynamics::CollisionObject* object2) -> bool {
            return self->ignoresCollision(object1, object2);
          },
          ::py::arg("object1"),
          ::py::arg("object2"),
          "Returns true if the given two CollisionObjects should be checked by "
          "the collision detector, false otherwise.");
}

} // namespace python
} // namespace dart
