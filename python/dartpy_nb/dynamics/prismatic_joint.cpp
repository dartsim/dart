#include "dynamics/prismatic_joint.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include "dart/dynamics/PrismaticJoint.hpp"

namespace nb = nanobind;

namespace dart::python_nb {

void defPrismaticJoint(nb::module_& m)
{
  using PrismaticJoint = dart::dynamics::PrismaticJoint;
  using Properties = PrismaticJoint::Properties;

  nb::class_<Properties>(m, "PrismaticJointProperties")
      .def(nb::init<>())
      .def_readwrite("mAxis",
          &dart::dynamics::detail::PrismaticJointUniqueProperties::mAxis);

  nb::class_<PrismaticJoint, dart::dynamics::Joint, std::shared_ptr<PrismaticJoint>>(m, "PrismaticJoint")
      .def("setAxis", &PrismaticJoint::setAxis, nb::arg("axis"))
      .def("getAxis",
          [](const PrismaticJoint& self) -> const Eigen::Vector3d& {
            return self.getAxis();
          },
          nb::rv_policy::reference_internal)
      .def("setPosition",
          [](PrismaticJoint& self, std::size_t index, double value) {
            self.setPosition(index, value);
          },
          nb::arg("index"),
          nb::arg("value"))
      .def("getPosition",
          [](const PrismaticJoint& self, std::size_t index) {
            return self.getPosition(index);
          },
          nb::arg("index"));
}

} // namespace dart::python_nb
