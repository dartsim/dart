#include "common/stopwatch.hpp"

#include "dart/common/stopwatch.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>

#include <sstream>

namespace nb = nanobind;

namespace dart::python_nb {

namespace {

void print_stopwatch(const dart::common::StopwatchNS& sw)
{
  std::ostringstream oss;
  oss << sw;
  nb::module_::import_("sys").attr("stdout").attr("write")(oss.str());
}

} // namespace

void defStopwatch(nb::module_& m)
{
  nb::class_<dart::common::StopwatchNS>(m, "Stopwatch")
      .def(nb::init<bool>(), nb::arg("start") = true)
      .def("isStarted", &dart::common::StopwatchNS::isStarted)
      .def("start", &dart::common::StopwatchNS::start)
      .def("stop", &dart::common::StopwatchNS::stop)
      .def("reset", &dart::common::StopwatchNS::reset)
      .def("elapsedS", &dart::common::StopwatchNS::elapsedS)
      .def("elapsedMS", &dart::common::StopwatchNS::elapsedMS)
      .def("elapsedUS", &dart::common::StopwatchNS::elapsedUS)
      .def("elapsedNS", &dart::common::StopwatchNS::elapsedNS)
      .def("print", [](const dart::common::StopwatchNS& self) {
        print_stopwatch(self);
      });

  m.def("tic", &dart::common::tic);
  m.def("toc", &dart::common::toc, nb::arg("print") = false);
  m.def("tocS", &dart::common::tocS, nb::arg("print") = false);
  m.def("tocMS", &dart::common::tocMS, nb::arg("print") = false);
  m.def("tocUS", &dart::common::tocUS, nb::arg("print") = false);
  m.def("tocNS", &dart::common::tocNS, nb::arg("print") = false);
}

} // namespace dart::python_nb
