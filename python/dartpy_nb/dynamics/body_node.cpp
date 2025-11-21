#include "dynamics/body_node.hpp"

#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/EndEffector.hpp"
#include "dart/dynamics/InverseKinematics.hpp"
#include "dart/dynamics/Joint.hpp"
#include "dart/dynamics/Shape.hpp"
#include "dart/dynamics/ShapeNode.hpp"
#include "dart/dynamics/SimpleFrame.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include "common/type_casters.hpp"

namespace nb = nanobind;

namespace dart::python_nb {

void defBodyNode(nb::module_& m)
{
  using BodyNode = dart::dynamics::BodyNode;
  using ShapePtr = dart::dynamics::ShapePtr;

  nb::class_<BodyNode, dart::dynamics::Frame>(m, "BodyNode")
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
            self.eachShapeNode(
                [&](dart::dynamics::ShapeNode* node) { nodes.push_back(node); });
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
          nb::arg("createIfNull") = false)
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
            auto* frame = new dart::dynamics::SimpleFrame(&self, name, tf);
            return std::shared_ptr<dart::dynamics::SimpleFrame>(
                frame); // owned by Python
          },
          nb::arg("name"),
          nb::arg("transform"),
          nb::rv_policy::take_ownership);

  registerPolymorphicCaster<dart::dynamics::Frame, BodyNode>();
}

} // namespace dart::python_nb
