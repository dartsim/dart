/*
 * Copyright (c) 2011-2019, The DART development contributors
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
#include "eigen_geometry_pybind.h"
#include "eigen_pybind.h"

namespace dart {
namespace python {

void ShapeFrame(pybind11::module& m)
{
  ::pybind11::class_<
      dart::dynamics::ShapeFrame,
      // dart::common::EmbedPropertiesOnTopOf<
      //     dart::dynamics::ShapeFrame,
      //     dart::dynamics::detail::ShapeFrameProperties,
      //     dart::common::SpecializedForAspect<
      //         dart::dynamics::VisualAspect,
      //         dart::dynamics::CollisionAspect,
      //         dart::dynamics::DynamicsAspect> >,
      dart::dynamics::Frame,
      std::shared_ptr<dart::dynamics::ShapeFrame>>(m, "ShapeFrame")
      .def(
          "setProperties",
          +[](dart::dynamics::ShapeFrame* self,
              const dart::dynamics::ShapeFrame::UniqueProperties& properties) {
            self->setProperties(properties);
          },
          ::pybind11::arg("properties"))
      .def(
          "setAspectProperties",
          +[](dart::dynamics::ShapeFrame* self,
              const dart::common::EmbedPropertiesOnTopOf<
                  dart::dynamics::ShapeFrame,
                  dart::dynamics::detail::ShapeFrameProperties,
                  dart::common::SpecializedForAspect<
                      dart::dynamics::VisualAspect,
                      dart::dynamics::CollisionAspect,
                      dart::dynamics::DynamicsAspect>>::AspectProperties&
                  properties) { self->setAspectProperties(properties); },
          ::pybind11::arg("properties"))
      .def(
          "setShape",
          +[](dart::dynamics::ShapeFrame* self,
              const dart::dynamics::ShapePtr& shape) { self->setShape(shape); },
          ::pybind11::arg("shape"))
      .def(
          "getShape",
          +[](dart::dynamics::ShapeFrame* self) -> dart::dynamics::ShapePtr {
            return self->getShape();
          })
      .def(
          "getShape",
          +[](const dart::dynamics::ShapeFrame* self)
              -> dart::dynamics::ConstShapePtr { return self->getShape(); })
      .def(
          "hasVisualAspect",
          +[](const dart::dynamics::ShapeFrame* self) -> bool {
            return self->hasVisualAspect();
          })
      .def(
          "setVisualAspect",
          +[](dart::dynamics::ShapeFrame* self,
              const dart::dynamics::VisualAspect* aspect) {
            self->setVisualAspect(aspect);
          },
          ::pybind11::arg("aspect"))
      .def(
          "removeVisualAspect",
          +[](dart::dynamics::ShapeFrame* self) { self->removeVisualAspect(); })
      .def(
          "releaseVisualAspect",
          +[](dart::dynamics::ShapeFrame* self)
              -> std::unique_ptr<dart::dynamics::VisualAspect> {
            return self->releaseVisualAspect();
          })
      .def(
          "hasCollisionAspect",
          +[](const dart::dynamics::ShapeFrame* self) -> bool {
            return self->hasCollisionAspect();
          })
      .def(
          "setCollisionAspect",
          +[](dart::dynamics::ShapeFrame* self,
              const dart::dynamics::CollisionAspect* aspect) {
            self->setCollisionAspect(aspect);
          },
          ::pybind11::arg("aspect"))
      .def(
          "removeCollisionAspect",
          +[](dart::dynamics::ShapeFrame* self) {
            self->removeCollisionAspect();
          })
      .def(
          "releaseCollisionAspect",
          +[](dart::dynamics::ShapeFrame* self)
              -> std::unique_ptr<dart::dynamics::CollisionAspect> {
            return self->releaseCollisionAspect();
          })
      .def(
          "hasDynamicsAspect",
          +[](const dart::dynamics::ShapeFrame* self) -> bool {
            return self->hasDynamicsAspect();
          })
      .def(
          "setDynamicsAspect",
          +[](dart::dynamics::ShapeFrame* self,
              const dart::dynamics::DynamicsAspect* aspect) {
            self->setDynamicsAspect(aspect);
          },
          ::pybind11::arg("aspect"))
      .def(
          "removeDynamicsAspect",
          +[](dart::dynamics::ShapeFrame* self) {
            self->removeDynamicsAspect();
          })
      .def(
          "releaseDynamicsAspect",
          +[](dart::dynamics::ShapeFrame* self)
              -> std::unique_ptr<dart::dynamics::DynamicsAspect> {
            return self->releaseDynamicsAspect();
          })
      .def("isShapeNode", +[](const dart::dynamics::ShapeFrame* self) -> bool {
        return self->isShapeNode();
      });
}

} // namespace python
} // namespace dart
