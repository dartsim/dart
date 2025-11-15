#include "dynamics/revolute_joint.hpp"

#include "dart/dynamics/RevoluteJoint.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

namespace nb = nanobind;

namespace dart::python_nb {

void defRevoluteJoint(nb::module_& m)
{
  using RevoluteJoint = dart::dynamics::RevoluteJoint;
  using Properties = RevoluteJoint::Properties;

  nb::class_<Properties>(m, "RevoluteJointProperties")
      .def(nb::init<>())
      .def_readwrite("mName", &Properties::mName)
      .def_readwrite("mAxis", &Properties::mAxis)
      .def_readwrite("mT_ParentBodyToJoint", &Properties::mT_ParentBodyToJoint)
      .def_readwrite("mT_ChildBodyToJoint", &Properties::mT_ChildBodyToJoint);

  nb::class_<RevoluteJoint, dart::dynamics::Joint>(m, "RevoluteJoint")
      .def("setAxis", &RevoluteJoint::setAxis, nb::arg("axis"))
      .def("getAxis", &RevoluteJoint::getAxis)
      .def(
          "setPosition",
          [](RevoluteJoint& self, std::size_t idx, double val) {
            self.setPosition(idx, val);
          },
          nb::arg("index"),
          nb::arg("value"))
      .def(
          "getPosition",
          [](const RevoluteJoint& self, std::size_t idx) {
            return self.getPosition(idx);
          },
          nb::arg("index"));
}

} // namespace dart::python_nb
