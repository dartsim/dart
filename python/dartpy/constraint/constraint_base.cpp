#include "constraint/constraint_base.hpp"

#include "dart/constraint/constraint_base.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>

namespace nb = nanobind;

namespace dart::python_nb {

void defConstraintBase(nb::module_& m)
{
  using ConstraintBase = dart::constraint::ConstraintBase;

  nb::class_<ConstraintBase>(m, "ConstraintBase")
      .def(
          "getType",
          [](const ConstraintBase& self) {
            return std::string(self.getType());
          })
      .def("getDimension", &ConstraintBase::getDimension)
      .def("update", &ConstraintBase::update);
}

} // namespace dart::python_nb
