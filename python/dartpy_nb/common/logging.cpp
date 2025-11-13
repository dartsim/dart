#include "common/logging.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>

#include "dart/common/Logging.hpp"

namespace nb = nanobind;

namespace dart::python_nb {

void defLogging(nb::module_& m)
{
  m.def("trace", [](const std::string& message) { common::trace(message); }, nb::arg("message"));
  m.def("debug", [](const std::string& message) { common::debug(message); }, nb::arg("message"));
  m.def("info", [](const std::string& message) { common::info(message); }, nb::arg("message"));
  m.def("warn", [](const std::string& message) { common::warn(message); }, nb::arg("message"));
  m.def("error", [](const std::string& message) { common::error(message); }, nb::arg("message"));
  m.def("fatal", [](const std::string& message) { common::fatal(message); }, nb::arg("message"));
}

} // namespace dart::python_nb
