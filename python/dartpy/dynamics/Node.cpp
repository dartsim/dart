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
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "eigen_geometry_pybind.h"
#include "eigen_pybind.h"
#include "pointers.hpp"

namespace py = pybind11;

namespace dart {
namespace python {

void Node(py::module& m)
{
  ::py::class_<
      dart::dynamics::Node,
      /*dart::common::VersionCounter,*/ dart::common::Subject,
      std::shared_ptr<dart::dynamics::Node> >(m, "Node")
      .def(
          "setName",
          +[](dart::dynamics::Node* self, const std::string& newName)
              -> const std::string& { return self->setName(newName); },
          ::py::return_value_policy::reference_internal,
          ::py::arg("newName"))
      .def(
          "getName",
          +[](const dart::dynamics::Node* self) -> const std::string& {
            return self->getName();
          },
          ::py::return_value_policy::reference_internal)
      .def(
          "setNodeState",
          +[](dart::dynamics::Node* self,
              const dart::dynamics::Node::State& otherState) {
            self->setNodeState(otherState);
          },
          ::py::arg("otherState"))
      .def(
          "getNodeState",
          +[](const dart::dynamics::Node* self)
              -> std::unique_ptr<dart::dynamics::Node::State> {
            return self->getNodeState();
          })
      .def(
          "setNodeProperties",
          +[](dart::dynamics::Node* self,
              const dart::dynamics::Node::Properties& properties) {
            self->setNodeProperties(properties);
          },
          ::py::arg("properties"))
      .def(
          "getNodeProperties",
          +[](const dart::dynamics::Node* self)
              -> std::unique_ptr<dart::dynamics::Node::Properties> {
            return self->getNodeProperties();
          })
      .def(
          "getBodyNodePtr",
          +[](dart::dynamics::Node* self) -> dart::dynamics::BodyNodePtr {
            return self->getBodyNodePtr();
          })
      .def(
          "getBodyNodePtr",
          +[](const dart::dynamics::Node* self)
              -> dart::dynamics::ConstBodyNodePtr {
            return self->getBodyNodePtr();
          })
      .def(
          "getBodyNode",
          +[](dart::dynamics::Node* self) -> dart::dynamics::BodyNode* {
            return self->getBodyNodePtr().get();
          })
      .def(
          "isRemoved",
          +[](const dart::dynamics::Node* self) -> bool {
            return self->isRemoved();
          })
      .def(
          "getSkeleton",
          +[](dart::dynamics::Node* self)
              -> std::shared_ptr<dart::dynamics::Skeleton> {
            return self->getSkeleton();
          })
      .def(
          "getSkeleton",
          +[](const dart::dynamics::Node* self)
              -> std::shared_ptr<const dart::dynamics::Skeleton> {
            return self->getSkeleton();
          });
}

} // namespace python
} // namespace dart
