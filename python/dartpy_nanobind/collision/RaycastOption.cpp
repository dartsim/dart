// clang-format off
#include "detail/dart_nb.hpp"
// clang-format on

#include <nanobind/stl/function.h>

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

#include <dart/collision/CollisionObject.hpp>
#include <dart/collision/RaycastOption.hpp>

namespace dart {
namespace python {

void RaycastOption(nb::module_& m)
{
  dartnb::dart_class<dart::collision::RaycastOption>(m, "RaycastOption")
      .def(dartnb::init<>())
      .def(dartnb::init<bool>(), nb::arg("enableAllHits"))
      .def(
          dartnb::init<bool, bool>(),
          nb::arg("enableAllHits"),
          nb::arg("sortByClosest"))
      .def(
          dartnb::
              init<bool, bool, dart::collision::RaycastOption::RaycastFilter>(),
          nb::arg("enableAllHits"),
          nb::arg("sortByClosest"),
          nb::arg("filter") = nullptr)
      .def_rw(
          "mEnableAllHits",
          &dart::collision::RaycastOption::mEnableAllHits,
          dartnb::setterArgument(
              &dart::collision::RaycastOption::mEnableAllHits))
      .def_rw(
          "mSortByClosest",
          &dart::collision::RaycastOption::mSortByClosest,
          dartnb::setterArgument(
              &dart::collision::RaycastOption::mSortByClosest))
      .def_rw(
          "mFilter",
          &dart::collision::RaycastOption::mFilter,
          dartnb::setterArgument(&dart::collision::RaycastOption::mFilter));
}

} // namespace python
} // namespace dart
