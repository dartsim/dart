#include "constraint/dynamic_joint_constraint.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include "dart/constraint/BallJointConstraint.hpp"
#include "dart/dynamics/BodyNode.hpp"

namespace nb = nanobind;

namespace dart::python_nb {

void defDynamicJointConstraint(nb::module_& m)
{
  using BallJointConstraint = dart::constraint::BallJointConstraint;

  nb::class_<BallJointConstraint, dart::constraint::ConstraintBase, std::shared_ptr<BallJointConstraint>>(m, "BallJointConstraint")
      .def(nb::init<dart::dynamics::BodyNode*, dart::dynamics::BodyNode*, const Eigen::Vector3d&>(),
          nb::arg("bodyNode1"),
          nb::arg("bodyNode2"),
          nb::arg("jointPosition"))
      .def("getType",
          [](const BallJointConstraint& self) -> const std::string& {
            return self.getType();
          },
          nb::rv_policy::reference_internal)
      .def_static("getStaticType",
          []() -> const std::string& {
            return BallJointConstraint::getStaticType();
          },
          nb::rv_policy::reference_internal);
}

} // namespace dart::python_nb
