// clang-format off
#include "detail/dart_nb.hpp"
// clang-format on

#include <nanobind/trampoline.h>

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

#include <dart/collision/CollisionFilter.hpp>
#include <dart/collision/CollisionObject.hpp>

#include <dart/dynamics/BodyNode.hpp>

#include <memory>

namespace dart {
namespace python {

template <class CollisionFilterBase = dart::collision::CollisionFilter>
class PyCollisionFilter : public CollisionFilterBase
{
public:
  NB_TRAMPOLINE(CollisionFilterBase, 1); // Inherit constructors

  bool ignoresCollision(
      const dart::collision::CollisionObject* object1,
      const dart::collision::CollisionObject* object2) const override
  {
    if constexpr (std::is_same_v<
                      CollisionFilterBase,
                      dart::collision::CollisionFilter>) {
      NB_OVERRIDE_PURE(ignoresCollision, object1, object2);
    } else {
      nb::gil_scoped_acquire guard;
      // CompositeCollisionFilter has no public Python ignoresCollision method.
      if (!nb::hasattr(nb_trampoline.base(), "ignoresCollision"))
        return CollisionFilterBase::ignoresCollision(object1, object2);
      NB_OVERRIDE(ignoresCollision, object1, object2);
    }
  }
};

void CollisionFilter(nb::module_& m)
{
  dartnb::dart_class<dart::collision::CollisionFilter, PyCollisionFilter<>>(
      m, "CollisionFilter");

  dartnb::dart_class<
      dart::collision::CompositeCollisionFilter,
      PyCollisionFilter<dart::collision::CompositeCollisionFilter>,
      dart::collision::CollisionFilter>(m, "CompositeCollisionFilter")
      .def(dartnb::init<>())
      .def(
          "addCollisionFilter",
          +[](dart::collision::CompositeCollisionFilter* self,
              const dart::collision::CollisionFilter* filter) {
            self->addCollisionFilter(filter);
          },
          nb::arg("filter").none(),
          "Adds a collision filter to this CompositeCollisionFilter.")
      .def(
          "removeCollisionFilter",
          +[](dart::collision::CompositeCollisionFilter* self,
              const dart::collision::CollisionFilter* filter) {
            self->removeCollisionFilter(filter);
          },
          nb::arg("filter").none(),
          "Removes a collision filter from this CompositeCollisionFilter.")
      .def(
          "removeAllCollisionFilters",
          +[](dart::collision::CompositeCollisionFilter* self) {
            self->removeAllCollisionFilters();
          },
          "Removes all the collision filters from this "
          "CompositeCollisionFilter.");

  dartnb::dart_class<
      dart::collision::BodyNodeCollisionFilter,
      PyCollisionFilter<dart::collision::BodyNodeCollisionFilter>,
      dart::collision::CollisionFilter>(m, "BodyNodeCollisionFilter")
      .def(dartnb::init<>())
      .def(
          "addBodyNodePairToBlackList",
          +[](dart::collision::BodyNodeCollisionFilter* self,
              const dart::dynamics::BodyNode* bodyNode1,
              const dart::dynamics::BodyNode* bodyNode2) {
            self->addBodyNodePairToBlackList(bodyNode1, bodyNode2);
          },
          nb::arg("bodyNode1").none(),
          nb::arg("bodyNode2").none(),
          "Add a BodyNode pair to the blacklist.")
      .def(
          "removeBodyNodePairFromBlackList",
          +[](dart::collision::BodyNodeCollisionFilter* self,
              const dart::dynamics::BodyNode* bodyNode1,
              const dart::dynamics::BodyNode* bodyNode2) {
            self->removeBodyNodePairFromBlackList(bodyNode1, bodyNode2);
          },
          nb::arg("bodyNode1").none(),
          nb::arg("bodyNode2").none(),
          "Remove a BodyNode pair from the blacklist.")
      .def(
          "removeAllBodyNodePairsFromBlackList",
          +[](dart::collision::BodyNodeCollisionFilter* self) {
            self->removeAllBodyNodePairsFromBlackList();
          },
          "Remove all the BodyNode pairs from the blacklist.")
      .def(
          "ignoresCollision",
          +[](const dart::collision::BodyNodeCollisionFilter* self,
              const dart::collision::CollisionObject* object1,
              const dart::collision::CollisionObject* object2) -> bool {
            return self->ignoresCollision(object1, object2);
          },
          nb::arg("object1").none(),
          nb::arg("object2").none(),
          "Returns true if the given two CollisionObjects should be checked by "
          "the collision detector, false otherwise.");
}

} // namespace python
} // namespace dart
