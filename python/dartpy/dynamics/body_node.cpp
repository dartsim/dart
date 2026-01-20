#include "dynamics/body_node.hpp"

#include "common/repr.hpp"
#include "common/type_casters.hpp"
#include "dart/dynamics/body_node.hpp"
#include "dart/dynamics/end_effector.hpp"
#include "dart/dynamics/inverse_kinematics.hpp"
#include "dart/dynamics/joint.hpp"
#include "dart/dynamics/shape.hpp"
#include "dart/dynamics/shape_node.hpp"
#include "dart/dynamics/simple_frame.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

namespace nb = nanobind;

namespace dart::python_nb {

void defBodyNode(nb::module_& m)
{
  using BodyNode = dart::dynamics::BodyNode;
  using ShapePtr = dart::dynamics::ShapePtr;

  nb::class_<BodyNode, dart::dynamics::JacobianNode>(m, "BodyNode")
      .def(
          "getName",
          [](const BodyNode& self) -> const std::string& {
            return self.getName();
          },
          nb::rv_policy::reference_internal)
      .def(
          "createShapeNode",
          [](BodyNode& self,
             const ShapePtr& shape) -> dart::dynamics::ShapeNode* {
            return self.createShapeNode(shape);
          },
          nb::arg("shape"),
          nb::rv_policy::reference_internal)
      .def(
          "getShapeNodes",
          [](BodyNode& self) {
            std::vector<dart::dynamics::ShapeNode*> nodes;
            self.eachShapeNode([&](dart::dynamics::ShapeNode* node) {
              nodes.push_back(node);
            });
            return nodes;
          },
          nb::rv_policy::reference_internal)
      .def("getNumShapeNodes", &BodyNode::getNumShapeNodes)
      .def("getBodyForce", &BodyNode::getBodyForce)
      .def(
          "getSpatialVelocity",
          [](const BodyNode& self) { return self.getSpatialVelocity(); })
      .def(
          "getBodyNodePtr",
          [](BodyNode& self) -> BodyNode* { return &self; },
          nb::rv_policy::reference_internal)
      .def(
          "getChildBodyNode",
          [](BodyNode& self, std::size_t index) -> BodyNode* {
            return self.getChildBodyNode(index);
          },
          nb::rv_policy::reference_internal,
          nb::arg("index"))
      .def(
          "getChildJoint",
          [](BodyNode& self, std::size_t index) -> dart::dynamics::Joint* {
            return self.getChildJoint(index);
          },
          nb::rv_policy::reference_internal,
          nb::arg("index"))
      .def(
          "getParentBodyNode",
          [](BodyNode& self) -> BodyNode* { return self.getParentBodyNode(); },
          nb::rv_policy::reference_internal)
      .def(
          "getParentJoint",
          [](BodyNode& self) -> dart::dynamics::Joint* {
            return self.getParentJoint();
          },
          nb::rv_policy::reference_internal)
      .def("getNumEndEffectors", &BodyNode::getNumEndEffectors)
      .def(
          "getEndEffector",
          [](BodyNode& self, std::size_t index) {
            return self.getEndEffector(index);
          },
          nb::rv_policy::reference_internal,
          nb::arg("index"))
      .def(
          "getInertia",
          [](BodyNode& self) -> dart::dynamics::Inertia {
            return self.getInertia();
          })
      .def("getOrCreateIK", [](BodyNode& self) { return self.getOrCreateIK(); })
      .def(
          "getIK",
          [](BodyNode& self, bool createIfNull) {
            return self.getIK(createIfNull);
          },
          nb::arg("create_if_null") = false)
      .def(
          "__repr__",
          [](const BodyNode& self) {
            const auto skeleton = self.getSkeleton();
            const auto* parent = self.getParentBodyNode();
            std::vector<std::pair<std::string, std::string>> fields;
            fields.emplace_back("name", repr_string(self.getName()));
            fields.emplace_back(
                "skeleton",
                skeleton ? repr_string(skeleton->getName()) : "None");
            fields.emplace_back(
                "parent", parent ? repr_string(parent->getName()) : "None");
            fields.emplace_back(
                "child_joints", std::to_string(self.getNumChildJoints()));
            fields.emplace_back(
                "shape_nodes", std::to_string(self.getNumShapeNodes()));
            return format_repr("BodyNode", fields);
          })
      .def(
          "createEndEffector",
          [](BodyNode& self, const std::string& name) {
            auto* ee = self.createEndEffector(name);
            auto skeletonHandle = self.getSkeleton();
            return std::shared_ptr<dart::dynamics::EndEffector>(
                skeletonHandle, ee);
          },
          nb::arg("name"),
          nb::rv_policy::reference_internal)
      .def(
          "createSimpleFrame",
          [](BodyNode& self,
             const std::string& name,
             const Eigen::Isometry3d& tf) {
            return std::make_shared<dart::dynamics::SimpleFrame>(
                &self, name, tf);
          },
          nb::arg("name"),
          nb::arg("transform"));

  registerPolymorphicCaster<dart::dynamics::Frame, BodyNode>();
  registerPolymorphicCaster<dart::dynamics::JacobianNode, BodyNode>();
}

} // namespace dart::python_nb
