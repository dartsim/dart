#include "dynamics/revolute_joint.hpp"

#include "dart/dynamics/RevoluteJoint.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include "common/type_casters.hpp"

namespace nb = nanobind;

namespace dart::python_nb {

void defRevoluteJoint(nb::module_& m)
{
  using RevoluteJoint = dart::dynamics::RevoluteJoint;
  using Properties = RevoluteJoint::Properties;

  nb::class_<Properties>(m, "RevoluteJointProperties")
      .def(nb::init<>())
      .def_rw("mName", &Properties::mName)
      .def_rw("mAxis", &Properties::mAxis)
      .def_rw("mT_ParentBodyToJoint", &Properties::mT_ParentBodyToJoint)
      .def_rw("mT_ChildBodyToJoint", &Properties::mT_ChildBodyToJoint);

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

  registerPolymorphicCaster<dart::dynamics::Joint, RevoluteJoint>();
}

} // namespace dart::python_nb
