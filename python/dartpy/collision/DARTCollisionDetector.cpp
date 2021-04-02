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

namespace py = pybind11;

namespace dart {
namespace python {

void DARTCollisionDetector(py::module& m)
{
  ::py::class_<
      dart::collision::DARTCollisionDetector,
      std::shared_ptr<dart::collision::DARTCollisionDetector>,
      dart::collision::CollisionDetector>(m, "DARTCollisionDetector")
      .def(::py::init(
          +[]() -> std::shared_ptr<dart::collision::DARTCollisionDetector> {
            return dart::collision::DARTCollisionDetector::create();
          }))
      .def(
          "cloneWithoutCollisionObjects",
          +[](const dart::collision::DARTCollisionDetector* self)
              -> std::shared_ptr<dart::collision::CollisionDetector> {
            return self->cloneWithoutCollisionObjects();
          })
      .def(
          "getType",
          +[](const dart::collision::DARTCollisionDetector* self)
              -> const std::string& { return self->getType(); },
          ::py::return_value_policy::reference_internal)
      .def(
          "createCollisionGroup",
          +[](dart::collision::DARTCollisionDetector* self)
              -> std::shared_ptr<dart::collision::CollisionGroup> {
            return self->createCollisionGroupAsSharedPtr();
          })
      .def_static(
          "getStaticType",
          +[]() -> const std::string& {
            return dart::collision::DARTCollisionDetector::getStaticType();
          },
          ::py::return_value_policy::reference_internal);
}

} // namespace python
} // namespace dart
