#include "common/composite.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include "dart/common/Composite.hpp"

namespace nb = nanobind;

namespace dart::python_nb {

void defComposite(nb::module_& m)
{
  using Composite = dart::common::Composite;

  nb::class_<Composite, std::shared_ptr<Composite>>(m, "Composite")
      .def(nb::init<>())
      .def("setCompositeState", &Composite::setCompositeState, nb::arg("newStates"))
      .def("getCompositeState", &Composite::getCompositeState)
      .def("copyCompositeStateTo", &Composite::copyCompositeStateTo, nb::arg("outgoingStates"))
      .def("setCompositeProperties", &Composite::setCompositeProperties, nb::arg("newProperties"))
      .def("getCompositeProperties", &Composite::getCompositeProperties)
      .def(
          "copyCompositePropertiesTo",
          &Composite::copyCompositePropertiesTo,
          nb::arg("outgoingProperties"))
      .def("duplicateAspects", &Composite::duplicateAspects, nb::arg("fromComposite"))
      .def("matchAspects", &Composite::matchAspects, nb::arg("otherComposite"));
}

} // namespace dart::python_nb
