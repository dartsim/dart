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

#include <dart/common/Composite.hpp>

#include <memory>

namespace dart {
namespace python {

void Composite(nb::module_& m)
{
  dartnb::dart_class<dart::common::Composite>(m, "Composite")
      .def(dartnb::init<>())
      .def(
          "setCompositeState",
          +[](dart::common::Composite* self,
              const dart::common::Composite::State& newStates) {
            self->setCompositeState(newStates);
          },
          nb::arg("newStates"))
      .def(
          "getCompositeState",
          +[](const dart::common::Composite* self)
              -> dart::common::Composite::State {
            return self->getCompositeState();
          })
      .def(
          "copyCompositeStateTo",
          +[](const dart::common::Composite* self,
              dart::common::Composite::State& outgoingStates) {
            self->copyCompositeStateTo(outgoingStates);
          },
          nb::arg("outgoingStates"))
      .def(
          "setCompositeProperties",
          +[](dart::common::Composite* self,
              const dart::common::Composite::Properties& newProperties) {
            self->setCompositeProperties(newProperties);
          },
          nb::arg("newProperties"))
      .def(
          "getCompositeProperties",
          +[](const dart::common::Composite* self)
              -> dart::common::Composite::Properties {
            return self->getCompositeProperties();
          })
      .def(
          "copyCompositePropertiesTo",
          +[](const dart::common::Composite* self,
              dart::common::Composite::Properties& outgoingProperties) {
            self->copyCompositePropertiesTo(outgoingProperties);
          },
          nb::arg("outgoingProperties"))
      .def(
          "duplicateAspects",
          +[](dart::common::Composite* self,
              const dart::common::Composite* fromComposite) {
            self->duplicateAspects(fromComposite);
          },
          nb::arg("fromComposite").none())
      .def(
          "matchAspects",
          +[](dart::common::Composite* self,
              const dart::common::Composite* otherComposite) {
            self->matchAspects(otherComposite);
          },
          nb::arg("otherComposite").none());
}

} // namespace python
} // namespace dart
