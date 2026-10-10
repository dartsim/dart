// clang-format off
#include "detail/dart_nb.hpp"
// clang-format on

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

#include <dart/collision/CollisionDetector.hpp>
#include <dart/collision/CollisionGroup.hpp>
#include <dart/collision/fcl/FCLCollisionDetector.hpp>

#include <memory>
#include <string>

namespace dart {
namespace python {

void FCLCollisionDetector(nb::module_& m)
{
  auto fclCollisionDetector
      = dartnb::dart_class<
            dart::collision::FCLCollisionDetector,
            dart::collision::CollisionDetector>(m, "FCLCollisionDetector")
            .def(dartnb::factory(
                +[]()
                    -> std::shared_ptr<dart::collision::FCLCollisionDetector> {
                  return dart::collision::FCLCollisionDetector::create();
                }))
            .def(
                "cloneWithoutCollisionObjects",
                +[](dart::collision::FCLCollisionDetector* self)
                    -> std::shared_ptr<dart::collision::CollisionDetector> {
                  return self->cloneWithoutCollisionObjects();
                })
            .def(
                "getType",
                +[](const dart::collision::FCLCollisionDetector* self)
                    -> const std::string& { return self->getType(); },
                nb::rv_policy::reference_internal)
            .def(
                "createCollisionGroup",
                +[](dart::collision::FCLCollisionDetector* self)
                    -> std::shared_ptr<dart::collision::CollisionGroup> {
                  return self->createCollisionGroup();
                })
            .def(
                "setPrimitiveShapeType",
                +[](dart::collision::FCLCollisionDetector* self,
                    dart::collision::FCLCollisionDetector::PrimitiveShape
                        type) { self->setPrimitiveShapeType(type); },
                nb::arg("type"))
            .def(
                "getPrimitiveShapeType",
                +[](const dart::collision::FCLCollisionDetector* self)
                    -> dart::collision::FCLCollisionDetector::PrimitiveShape {
                  return self->getPrimitiveShapeType();
                })
            .def(
                "setContactPointComputationMethod",
                +[](dart::collision::FCLCollisionDetector* self,
                    dart::collision::FCLCollisionDetector::
                        ContactPointComputationMethod method) {
                  self->setContactPointComputationMethod(method);
                },
                nb::arg("method"))
            .def(
                "getContactPointComputationMethod",
                +[](const dart::collision::FCLCollisionDetector* self)
                    -> dart::collision::FCLCollisionDetector::
                        ContactPointComputationMethod {
                          return self->getContactPointComputationMethod();
                        })
            .def_static(
                "getStaticType",
                +[]() -> const std::string& {
                  return dart::collision::FCLCollisionDetector::getStaticType();
                },
                nb::rv_policy::reference_internal);

  nb::enum_<dart::collision::FCLCollisionDetector::PrimitiveShape>(
      fclCollisionDetector, "PrimitiveShape", nb::is_arithmetic())
      .value(
          "PRIMITIVE",
          dart::collision::FCLCollisionDetector::PrimitiveShape::PRIMITIVE)
      .value(
          "MESH", dart::collision::FCLCollisionDetector::PrimitiveShape::MESH)
      .export_values();

  nb::enum_<
      dart::collision::FCLCollisionDetector::ContactPointComputationMethod>(
      fclCollisionDetector,
      "ContactPointComputationMethod",
      nb::is_arithmetic())
      .value(
          "FCL",
          dart::collision::FCLCollisionDetector::ContactPointComputationMethod::
              FCL)
      .value(
          "DART",
          dart::collision::FCLCollisionDetector::ContactPointComputationMethod::
              DART)
      .export_values();
}

} // namespace python
} // namespace dart
