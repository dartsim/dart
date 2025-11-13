#include "common/string.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include "dart/common/String.hpp"

namespace nb = nanobind;

namespace dart::python_nb {

void defString(nb::module_& m)
{
  m.def("toUpper", &dart::common::toUpper, nb::arg("str"));
  m.def("toLower", &dart::common::toLower, nb::arg("str"));
  m.def("trim", &dart::common::trim, nb::arg("str"), nb::arg("whitespaces") = " \n\r\t");
  m.def("trimLeft", &dart::common::trimLeft, nb::arg("str"), nb::arg("whitespaces") = " \n\r\t");
  m.def("trimRight", &dart::common::trimRight, nb::arg("str"), nb::arg("whitespaces") = " \n\r\t");
  m.def("split", &dart::common::split, nb::arg("str"), nb::arg("delimiters") = " \n\r\t");
}

} // namespace dart::python_nb
