// clang-format off
#include "detail/dart_nb.hpp"
// clang-format on

#include "detail/secondary_methods.hpp"

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

#include "eigen_geometry_pybind.h"
#include "eigen_pybind.h"

#include <dart/dynamics/Entity.hpp>
#include <dart/dynamics/Frame.hpp>

#include <dart/common/Subject.hpp>

#include <memory>
#include <string>

namespace dart {
namespace python {

template <class Cls>
void defEntityMethods(Cls& cls)
{
  cls.def(
         "setName",
         +[](dart::dynamics::Entity* self, const std::string& name)
             -> const std::string& { return self->setName(name); },
         nb::rv_policy::reference_internal,
         nb::arg("name"))
      .def(
          "getName",
          +[](const dart::dynamics::Entity* self) -> const std::string& {
            return self->getName();
          },
          nb::rv_policy::reference_internal)
      .def(
          "getParentFrame",
          +[](const dart::dynamics::Entity* self)
              -> const dart::dynamics::Frame* {
            return self->getParentFrame();
          },
          nb::rv_policy::reference_internal)
      .def(
          "descendsFrom",
          +[](const dart::dynamics::Entity* self,
              const dart::dynamics::Frame* someFrame) -> bool {
            return self->descendsFrom(someFrame);
          },
          nb::arg("someFrame").none())
      .def(
          "isFrame",
          +[](const dart::dynamics::Entity* self) -> bool {
            return self->isFrame();
          })
      .def(
          "isQuiet",
          +[](const dart::dynamics::Entity* self) -> bool {
            return self->isQuiet();
          })
      .def(
          "dirtyTransform",
          +[](dart::dynamics::Entity* self) { self->dirtyTransform(); })
      .def(
          "needsTransformUpdate",
          +[](const dart::dynamics::Entity* self) -> bool {
            return self->needsTransformUpdate();
          })
      .def(
          "dirtyVelocity",
          +[](dart::dynamics::Entity* self) { self->dirtyVelocity(); })
      .def(
          "needsVelocityUpdate",
          +[](const dart::dynamics::Entity* self) -> bool {
            return self->needsVelocityUpdate();
          })
      .def(
          "dirtyAcceleration",
          +[](dart::dynamics::Entity* self) { self->dirtyAcceleration(); })
      .def(
          "needsAccelerationUpdate",
          +[](const dart::dynamics::Entity* self) -> bool {
            return self->needsAccelerationUpdate();
          });
}

void Entity(nb::module_& m)
{
  auto cls = dartnb::dart_class<dart::dynamics::Entity, dart::common::Subject>(
      m, "Entity");
  defEntityMethods(cls);
  dartnb::register_methods(
      typeid(dart::dynamics::Entity), [](nb::handle target) {
        dartnb::SecondaryMethods<dart::dynamics::Entity> rebound(target);
        defEntityMethods(rebound);
      });

  dartnb::dart_class<dart::dynamics::Detachable, dart::dynamics::Entity>(
      m, "Detachable")
      .def(
          "setParentFrame",
          +[](dart::dynamics::Detachable* self,
              dart::dynamics::Frame* _newParentFrame) {
            self->setParentFrame(_newParentFrame);
          },
          nb::arg("newParentFrame").none());
}

} // namespace python
} // namespace dart
