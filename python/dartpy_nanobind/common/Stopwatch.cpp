// clang-format off
#include "detail/dart_nb.hpp"
// clang-format on

#include <sstream>

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

#include <dart/common/Stopwatch.hpp>

#include <iostream>

namespace dart::python {

void Stopwatch(nb::module_& m)
{
  dartnb::dart_class<common::StopwatchNS>(m, "Stopwatch")
      .def(dartnb::init<bool>(), nb::arg("start") = true)
      .def("isStarted", &common::StopwatchNS::isStarted)
      .def("start", &common::StopwatchNS::start)
      .def("stop", &common::StopwatchNS::stop)
      .def("reset", &common::StopwatchNS::reset)
      .def("elapsedS", &common::StopwatchNS::elapsedS)
      .def("elapsedMS", &common::StopwatchNS::elapsedMS)
      .def("elapsedUS", &common::StopwatchNS::elapsedUS)
      .def("elapsedNS", &common::StopwatchNS::elapsedNS)
      .def("print", [](const common::StopwatchNS* self) {
        std::ostringstream stream;
        stream << *self;
        nb::module_::import_("sys").attr("stdout").attr("write")(stream.str());
      });

  m.def("tic", &common::tic);
  m.def("toc", &common::toc, nb::arg("print") = false);
  m.def("tocS", &common::tocS, nb::arg("print") = false);
  m.def("tocMS", &common::tocMS, nb::arg("print") = false);
  m.def("tocUS", &common::tocUS, nb::arg("print") = false);
  m.def("tocNS", &common::tocNS, nb::arg("print") = false);
}

} // namespace dart::python
