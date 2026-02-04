#include "common/string.hpp"

#include "dart/common/string.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

namespace nb = nanobind;

namespace dart::python_nb {

void defString(nb::module_& m)
{
  m.def("toUpper", &dart::common::toUpper, nb::arg("str"));
  m.def("toLower", &dart::common::toLower, nb::arg("str"));
  m.def(
      "trim",
      [](const std::string& str, const std::string& whitespaces) {
        return dart::common::trim(str, whitespaces);
      },
      nb::arg("str"),
      nb::arg("whitespaces") = " \n\r\t");
  m.def(
      "trimLeft",
      [](const std::string& str, const std::string& whitespaces) {
        return dart::common::trimLeft(str, whitespaces);
      },
      nb::arg("str"),
      nb::arg("whitespaces") = " \n\r\t");
  m.def(
      "trimRight",
      [](const std::string& str, const std::string& whitespaces) {
        return dart::common::trimRight(str, whitespaces);
      },
      nb::arg("str"),
      nb::arg("whitespaces") = " \n\r\t");
  m.def(
      "split",
      [](const std::string& str, const std::string& delimiters) {
        return dart::common::split(str, delimiters);
      },
      nb::arg("str"),
      nb::arg("delimiters") = " \n\r\t");
}

} // namespace dart::python_nb
