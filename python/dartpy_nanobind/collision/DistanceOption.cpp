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

#include <dart/collision/DistanceFilter.hpp>
#include <dart/collision/DistanceOption.hpp>

#include <memory>

namespace dart {
namespace python {

void DistanceOption(nb::module_& m)
{
  dartnb::dart_class<dart::collision::DistanceOption>(m, "DistanceOption")
      .def(dartnb::init<>())
      .def(dartnb::init<bool>(), nb::arg("enableNearestPoints"))
      .def(
          dartnb::init<bool, double>(),
          nb::arg("enableNearestPoints"),
          nb::arg("distanceLowerBound"))
      .def(
          dartnb::init<
              bool,
              double,
              const std::shared_ptr<dart::collision::DistanceFilter>&>(),
          nb::arg("enableNearestPoints"),
          nb::arg("distanceLowerBound"),
          nb::arg("distanceFilter").none())
      .def_rw(
          "enableNearestPoints",
          &dart::collision::DistanceOption::enableNearestPoints,
          dartnb::setterArgument(
              &dart::collision::DistanceOption::enableNearestPoints))
      .def_rw(
          "distanceLowerBound",
          &dart::collision::DistanceOption::distanceLowerBound,
          dartnb::setterArgument(
              &dart::collision::DistanceOption::distanceLowerBound))
      .def_rw(
          "distanceFilter",
          &dart::collision::DistanceOption::distanceFilter,
          dartnb::setterArgument(
              &dart::collision::DistanceOption::distanceFilter));
}

} // namespace python
} // namespace dart
