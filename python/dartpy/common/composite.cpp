#include "common/composite.hpp"

#include "dart/common/composite.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

namespace nb = nanobind;

namespace dart::python_nb {

void defComposite(nb::module_& m)
{
  using Composite = dart::common::Composite;

  nb::class_<Composite>(m, "Composite")
      .def(nb::init<>())
      .def(
          "setCompositeState",
          &Composite::setCompositeState,
          nb::arg("new_states"))
      .def("getCompositeState", &Composite::getCompositeState)
      .def(
          "copyCompositeStateTo",
          &Composite::copyCompositeStateTo,
          nb::arg("outgoing_states"))
      .def(
          "setCompositeProperties",
          &Composite::setCompositeProperties,
          nb::arg("new_properties"))
      .def("getCompositeProperties", &Composite::getCompositeProperties)
      .def(
          "copyCompositePropertiesTo",
          &Composite::copyCompositePropertiesTo,
          nb::arg("outgoing_properties"))
      .def(
          "duplicateAspects",
          &Composite::duplicateAspects,
          nb::arg("from_composite"))
      .def(
          "matchAspects", &Composite::matchAspects, nb::arg("other_composite"));
}

} // namespace dart::python_nb
