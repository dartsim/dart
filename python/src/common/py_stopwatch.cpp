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

#include <pybind11/chrono.h>
#include <pybind11/iostream.h>
#include <pybind11/pybind11.h>

#include "dart/common/all.hpp"

namespace py = pybind11;

namespace dart::python {

void py_stopwatch(py::module& m)
{
  py::class_<common::StopwatchNS>(m, "Stopwatch")
      .def(py::init<bool>(), py::arg("start") = true)
      .def("is_started", &common::StopwatchNS::is_started)
      .def("start", &common::StopwatchNS::start)
      .def("stop", &common::StopwatchNS::stop)
      .def("reset", &common::StopwatchNS::reset)
      .def("elapsed_s", &common::StopwatchNS::elapsed_s)
      .def("print", [](const common::StopwatchNS* self) {
        py::scoped_ostream_redirect stream(
            std::cout, py::module::import("sys").attr("stdout"));
        self->print();
      });

  m.def("tic", &common::tic);
  m.def("toc", &common::toc, py::arg("print") = false);
  m.def("toc_s", &common::toc_s, py::arg("print") = false);
  m.def("toc_ms", &common::toc_ms, py::arg("print") = false);
  m.def("toc_us", &common::toc_us, py::arg("print") = false);
  m.def("toc_ns", &common::toc_ns, py::arg("print") = false);
}

} // namespace dart::python
