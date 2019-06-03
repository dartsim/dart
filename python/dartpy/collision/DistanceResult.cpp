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

namespace py = pybind11;

namespace dart {
namespace python {

void DistanceResult(py::module& m)
{
  ::pybind11::class_<dart::collision::DistanceResult>(m, "DistanceResult")
      .def(::pybind11::init<>())
      .def(
          "clear",
          +[](dart::collision::DistanceResult* self) { self->clear(); })
      .def(
          "found",
          +[](const dart::collision::DistanceResult* self) -> bool {
            return self->found();
          })
      .def(
          "isMinDistanceClamped",
          +[](const dart::collision::DistanceResult* self) -> bool {
            return self->isMinDistanceClamped();
          })
      .def_readwrite(
          "minDistance", &dart::collision::DistanceResult::minDistance)
      .def_readwrite(
          "unclampedMinDistance",
          &dart::collision::DistanceResult::unclampedMinDistance)
      .def_readwrite(
          "shapeFrame1", &dart::collision::DistanceResult::shapeFrame1)
      .def_readwrite(
          "shapeFrame2", &dart::collision::DistanceResult::shapeFrame2)
      .def_readwrite(
          "nearestPoint1", &dart::collision::DistanceResult::nearestPoint1)
      .def_readwrite(
          "nearestPoint2", &dart::collision::DistanceResult::nearestPoint2);
}

} // namespace python
} // namespace dart
