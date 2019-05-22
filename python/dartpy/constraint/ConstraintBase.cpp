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

namespace dart {
namespace python {

void ConstraintBase(pybind11::module& m)
{
  ::pybind11::class_<
      dart::constraint::ConstraintBase,
      std::shared_ptr<dart::constraint::ConstraintBase> >(m, "ConstraintBase")
      .def(
          "getDimension",
          +[](const dart::constraint::ConstraintBase* self) -> std::size_t {
            return self->getDimension();
          })
      .def(
          "update",
          +[](dart::constraint::ConstraintBase* self) { self->update(); })
      .def(
          "getInformation",
          +[](dart::constraint::ConstraintBase* self,
              dart::constraint::ConstraintInfo* info) {
            self->getInformation(info);
          },
          ::pybind11::arg("info"))
      .def(
          "applyUnitImpulse",
          +[](dart::constraint::ConstraintBase* self, std::size_t index) {
            self->applyUnitImpulse(index);
          },
          ::pybind11::arg("index"))
      .def(
          "getVelocityChange",
          +[](dart::constraint::ConstraintBase* self,
              double* vel,
              bool withCfm) { self->getVelocityChange(vel, withCfm); },
          ::pybind11::arg("vel"),
          ::pybind11::arg("withCfm"))
      .def(
          "excite",
          +[](dart::constraint::ConstraintBase* self) { self->excite(); })
      .def(
          "unexcite",
          +[](dart::constraint::ConstraintBase* self) { self->unexcite(); })
      .def(
          "applyImpulse",
          +[](dart::constraint::ConstraintBase* self, double* lambda) {
            self->applyImpulse(lambda);
          },
          ::pybind11::arg("lambda"))
      .def(
          "isActive",
          +[](const dart::constraint::ConstraintBase* self) -> bool {
            return self->isActive();
          })
      .def(
          "getRootSkeleton",
          +[](const dart::constraint::ConstraintBase* self)
              -> dart::dynamics::SkeletonPtr {
            return self->getRootSkeleton();
          })
      .def(
          "uniteSkeletons",
          +[](dart::constraint::ConstraintBase* self) {
            self->uniteSkeletons();
          })
      .def_static(
          "compressPath",
          +[](dart::dynamics::SkeletonPtr skeleton)
              -> dart::dynamics::SkeletonPtr {
            return dart::constraint::ConstraintBase::compressPath(skeleton);
          },
          ::pybind11::arg("skeleton"))
      .def_static(
          "getRootSkeletonOf",
          +[](dart::dynamics::SkeletonPtr skeleton)
              -> dart::dynamics::SkeletonPtr {
            return dart::constraint::ConstraintBase::getRootSkeleton(skeleton);
          },
          ::pybind11::arg("skeleton"));
}

} // namespace python
} // namespace dart
