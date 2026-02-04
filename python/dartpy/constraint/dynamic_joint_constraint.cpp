#include "constraint/dynamic_joint_constraint.hpp"

#include "common/eigen_utils.hpp"
#include "common/polymorphic_utils.hpp"
#include "dart/constraint/ball_joint_constraint.hpp"
#include "dart/constraint/dynamic_joint_constraint.hpp"
#include "dart/constraint/revolute_joint_constraint.hpp"
#include "dart/constraint/weld_joint_constraint.hpp"
#include "dart/dynamics/body_node.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

namespace nb = nanobind;

namespace dart::python_nb {

void defDynamicJointConstraint(nb::module_& m)
{
  using DynamicJointConstraint = dart::constraint::DynamicJointConstraint;
  using BallJointConstraint = dart::constraint::BallJointConstraint;
  using RevoluteJointConstraint = dart::constraint::RevoluteJointConstraint;
  using WeldJointConstraint = dart::constraint::WeldJointConstraint;

  nb::class_<DynamicJointConstraint, dart::constraint::ConstraintBase>(
      m, "DynamicJointConstraint")
      .def_static(
          "setErrorAllowance",
          &DynamicJointConstraint::setErrorAllowance,
          nb::arg("allowance"))
      .def_static(
          "getErrorAllowance", &DynamicJointConstraint::getErrorAllowance)
      .def_static(
          "setErrorReductionParameter",
          &DynamicJointConstraint::setErrorReductionParameter,
          nb::arg("erp"))
      .def_static(
          "getErrorReductionParameter",
          &DynamicJointConstraint::getErrorReductionParameter)
      .def_static(
          "setMaxErrorReductionVelocity",
          &DynamicJointConstraint::setMaxErrorReductionVelocity,
          nb::arg("erv"))
      .def_static(
          "getMaxErrorReductionVelocity",
          &DynamicJointConstraint::getMaxErrorReductionVelocity)
      .def_static(
          "setConstraintForceMixing",
          &DynamicJointConstraint::setConstraintForceMixing,
          nb::arg("cfm"))
      .def_static(
          "getConstraintForceMixing",
          &DynamicJointConstraint::getConstraintForceMixing);

  nb::class_<BallJointConstraint, DynamicJointConstraint>(
      m, "BallJointConstraint")
      .def(
          nb::new_([](dart::dynamics::BodyNode* bodyNode,
                      nb::handle jointPosition) {
            return new BallJointConstraint(bodyNode, toVector3(jointPosition));
          }),
          nb::arg("body_node"),
          nb::arg("joint_position"),
          nb::keep_alive<1, 2>())
      .def(
          nb::new_([](nb::handle bodyNode1,
                      nb::handle bodyNode2,
                      nb::handle jointPosition) {
            auto& node1 = refFromHandle<dart::dynamics::BodyNode>(bodyNode1);
            auto& node2 = refFromHandle<dart::dynamics::BodyNode>(bodyNode2);
            return new BallJointConstraint(
                &node1, &node2, toVector3(jointPosition));
          }),
          nb::arg("body_node1"),
          nb::arg("body_node2"),
          nb::arg("joint_position"),
          nb::keep_alive<1, 2>(),
          nb::keep_alive<1, 3>())
      .def(
          "getType",
          [](const BallJointConstraint& self) {
            return std::string(self.getType());
          })
      .def_static("getStaticType", []() {
        return std::string(BallJointConstraint::getStaticType());
      });

  nb::class_<RevoluteJointConstraint, DynamicJointConstraint>(
      m, "RevoluteJointConstraint")
      .def(
          nb::new_([](dart::dynamics::BodyNode* bodyNode,
                      nb::handle jointPosition,
                      nb::handle axis) {
            return new RevoluteJointConstraint(
                bodyNode, toVector3(jointPosition), toVector3(axis));
          }),
          nb::arg("body_node"),
          nb::arg("joint_position"),
          nb::arg("axis"),
          nb::keep_alive<1, 2>())
      .def(
          nb::new_([](nb::handle bodyNode1,
                      nb::handle bodyNode2,
                      nb::handle jointPosition,
                      nb::handle axis1,
                      nb::handle axis2) {
            auto& node1 = refFromHandle<dart::dynamics::BodyNode>(bodyNode1);
            auto& node2 = refFromHandle<dart::dynamics::BodyNode>(bodyNode2);
            return new RevoluteJointConstraint(
                &node1,
                &node2,
                toVector3(jointPosition),
                toVector3(axis1),
                toVector3(axis2));
          }),
          nb::arg("body_node1"),
          nb::arg("body_node2"),
          nb::arg("joint_position"),
          nb::arg("axis1"),
          nb::arg("axis2"),
          nb::keep_alive<1, 2>(),
          nb::keep_alive<1, 3>())
      .def(
          "getType",
          [](const RevoluteJointConstraint& self) {
            return std::string(self.getType());
          })
      .def_static("getStaticType", []() {
        return std::string(RevoluteJointConstraint::getStaticType());
      });

  nb::class_<WeldJointConstraint, DynamicJointConstraint>(
      m, "WeldJointConstraint")
      .def(
          nb::new_([](dart::dynamics::BodyNode* bodyNode) {
            return new WeldJointConstraint(bodyNode);
          }),
          nb::arg("body_node"),
          nb::keep_alive<1, 2>())
      .def(
          nb::new_([](dart::dynamics::BodyNode* bodyNode1,
                      dart::dynamics::BodyNode* bodyNode2) {
            return new WeldJointConstraint(bodyNode1, bodyNode2);
          }),
          nb::arg("body_node1"),
          nb::arg("body_node2"),
          nb::keep_alive<1, 2>(),
          nb::keep_alive<1, 3>())
      .def(
          "setRelativeTransform",
          [](WeldJointConstraint& self, const Eigen::Isometry3d& tf) {
            self.setRelativeTransform(tf);
          },
          nb::arg("transform"))
      .def(
          "getRelativeTransform",
          [](const WeldJointConstraint& self) -> const Eigen::Isometry3d& {
            return self.getRelativeTransform();
          },
          nb::rv_policy::reference_internal)
      .def(
          "getType",
          [](const WeldJointConstraint& self) {
            return std::string(self.getType());
          })
      .def_static("getStaticType", []() {
        return std::string(WeldJointConstraint::getStaticType());
      });
}

} // namespace dart::python_nb
