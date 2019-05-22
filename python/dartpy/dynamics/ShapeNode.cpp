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

void ShapeNode(pybind11::module& m)
{
  ::pybind11::class_<
      dart::dynamics::ShapeNode,
      dart::dynamics::ShapeFrame,
      std::shared_ptr<dart::dynamics::ShapeNode>>(m, "ShapeNode")
      .def(
          "setProperties",
          +[](dart::dynamics::ShapeNode* self,
              const dart::dynamics::ShapeNode::Properties& properties) {
            self->setProperties(properties);
          },
          ::pybind11::arg("properties"))
      .def(
          "getShapeNodeProperties",
          +[](const dart::dynamics::ShapeNode* self)
              -> const dart::dynamics::ShapeNode::Properties {
            return self->getShapeNodeProperties();
          })
      .def(
          "copy",
          +[](dart::dynamics::ShapeNode* self,
              const dart::dynamics::ShapeNode* other) { self->copy(other); },
          ::pybind11::arg("other"))
      .def(
          "setRelativeTransform",
          +[](dart::dynamics::ShapeNode* self,
              const Eigen::Isometry3d& transform) {
            self->setRelativeTransform(transform);
          },
          ::pybind11::arg("transform"))
      .def(
          "setRelativeRotation",
          +[](dart::dynamics::ShapeNode* self,
              const Eigen::Matrix3d& rotation) {
            self->setRelativeRotation(rotation);
          },
          ::pybind11::arg("rotation"))
      .def(
          "getRelativeRotation",
          +[](const dart::dynamics::ShapeNode* self) -> Eigen::Matrix3d {
            return self->getRelativeRotation();
          })
      .def(
          "setRelativeTranslation",
          +[](dart::dynamics::ShapeNode* self,
              const Eigen::Vector3d& translation) {
            self->setRelativeTranslation(translation);
          },
          ::pybind11::arg("translation"))
      .def(
          "setOffset",
          +[](dart::dynamics::ShapeNode* self, const Eigen::Vector3d& offset) {
            self->setOffset(offset);
          },
          ::pybind11::arg("offset"))
      .def(
          "getRelativeTranslation",
          +[](const dart::dynamics::ShapeNode* self) -> Eigen::Vector3d {
            return self->getRelativeTranslation();
          })
      .def(
          "getOffset",
          +[](const dart::dynamics::ShapeNode* self) -> Eigen::Vector3d {
            return self->getOffset();
          });
}

} // namespace python
} // namespace dart
