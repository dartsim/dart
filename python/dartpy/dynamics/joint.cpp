#include "dynamics/joint.hpp"

#include "common/eigen_utils.hpp"
#include "common/repr.hpp"
#include "common/type_casters.hpp"
#include "dart/dynamics/body_node.hpp"
#include "dart/dynamics/degree_of_freedom.hpp"
#include "dart/dynamics/frame.hpp"
#include "dart/dynamics/joint.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <span>

namespace nb = nanobind;

namespace dart::python_nb {

void defJoint(nb::module_& m)
{
  using Joint = dart::dynamics::Joint;
  using ActuatorType = Joint::ActuatorType;

  nb::enum_<ActuatorType>(m, "ActuatorType")
      .value("FORCE", Joint::FORCE)
      .value("PASSIVE", Joint::PASSIVE)
      .value("SERVO", Joint::SERVO)
      .value("MIMIC", Joint::MIMIC)
      .value("ACCELERATION", Joint::ACCELERATION)
      .value("VELOCITY", Joint::VELOCITY)
      .value("LOCKED", Joint::LOCKED)
      .export_values();

  m.attr("DefaultActuatorType") = Joint::DefaultActuatorType;

  nb::enum_<dart::dynamics::MimicConstraintType>(m, "MimicConstraintType")
      .value("Motor", dart::dynamics::MimicConstraintType::Motor)
      .value("Coupler", dart::dynamics::MimicConstraintType::Coupler)
      .export_values();

  nb::class_<dart::dynamics::MimicDofProperties>(m, "MimicDofProperties")
      .def(nb::init<>())
      .def_rw(
          "mReferenceJoint",
          &dart::dynamics::MimicDofProperties::mReferenceJoint)
      .def_rw(
          "mReferenceDofIndex",
          &dart::dynamics::MimicDofProperties::mReferenceDofIndex)
      .def_rw("mMultiplier", &dart::dynamics::MimicDofProperties::mMultiplier)
      .def_rw("mOffset", &dart::dynamics::MimicDofProperties::mOffset)
      .def_rw(
          "mConstraintType",
          &dart::dynamics::MimicDofProperties::mConstraintType);

  nb::class_<Joint>(m, "Joint")
      .def(
          "getName",
          [](const Joint& self) -> const std::string& {
            return self.getName();
          },
          nb::rv_policy::reference_internal)
      .def(
          "setName",
          &Joint::setName,
          nb::arg("name"),
          nb::arg("preserve_name") = true)
      .def(
          "getType",
          [](const Joint& self) { return std::string(self.getType()); })
      .def(
          "__repr__",
          [](const Joint& self) {
            const auto* parent = self.getParentBodyNode();
            const auto* child = self.getChildBodyNode();
            std::vector<std::pair<std::string, std::string>> fields;
            fields.emplace_back("name", repr_string(self.getName()));
            fields.emplace_back("type", repr_string(self.getType()));
            fields.emplace_back("dofs", std::to_string(self.getNumDofs()));
            fields.emplace_back(
                "parent", parent ? repr_string(parent->getName()) : "None");
            fields.emplace_back(
                "child", child ? repr_string(child->getName()) : "None");
            return format_repr("Joint", fields);
          })
      .def(
          "setActuatorType",
          [](Joint& self, ActuatorType actuatorType) {
            self.setActuatorType(actuatorType);
          },
          nb::arg("actuator_type"))
      .def(
          "setActuatorTypeForDof",
          [](Joint& self, std::size_t index, ActuatorType actuatorType) {
            self.setActuatorType(index, actuatorType);
          },
          nb::arg("index"),
          nb::arg("actuator_type"))
      .def(
          "setActuatorTypes",
          [](Joint& self, const std::vector<ActuatorType>& actuatorTypes) {
            self.setActuatorTypes(std::span<const ActuatorType>(actuatorTypes));
          },
          nb::arg("actuator_types"))
      .def(
          "getActuatorType",
          [](const Joint& self) { return self.getActuatorType(); })
      .def(
          "getActuatorTypeForDof",
          [](const Joint& self, std::size_t index) {
            return self.getActuatorType(index);
          },
          nb::arg("index"))
      .def(
          "getActuatorTypes",
          [](const Joint& self) { return self.getActuatorTypes(); })
      .def(
          "hasActuatorType",
          [](const Joint& self, ActuatorType actuatorType) {
            return self.hasActuatorType(actuatorType);
          },
          nb::arg("actuator_type"))
      .def(
          "setUseCouplerConstraint",
          &Joint::setUseCouplerConstraint,
          nb::arg("enable"))
      .def("isUsingCouplerConstraint", &Joint::isUsingCouplerConstraint)
      .def("isKinematic", &Joint::isKinematic)
      .def("isDynamic", &Joint::isDynamic)
      .def("getNumDofs", &Joint::getNumDofs)
      .def(
          "getDof",
          static_cast<dart::dynamics::DegreeOfFreedom* (Joint::*)(std::size_t)>(
              &Joint::getDof),
          nb::rv_policy::reference_internal,
          nb::arg("index"))
      .def(
          "setDofName",
          [](Joint& self, std::size_t index, const std::string& name)
              -> const std::string& { return self.setDofName(index, name); },
          nb::rv_policy::reference_internal,
          nb::arg("index"),
          nb::arg("name"))
      .def(
          "setDofName",
          [](Joint& self,
             std::size_t index,
             const std::string& name,
             bool preserve) -> const std::string& {
            return self.setDofName(index, name, preserve);
          },
          nb::rv_policy::reference_internal,
          nb::arg("index"),
          nb::arg("name"),
          nb::arg("preserve_name"))
      .def(
          "getDofName",
          [](const Joint& self, std::size_t index) -> const std::string& {
            return self.getDofName(index);
          },
          nb::rv_policy::reference_internal,
          nb::arg("index"))
      .def(
          "preserveDofName",
          [](Joint& self, std::size_t index, bool preserve) {
            self.preserveDofName(index, preserve);
          },
          nb::arg("index"),
          nb::arg("preserve"))
      .def(
          "isDofNamePreserved",
          [](const Joint& self, std::size_t index) -> bool {
            return self.isDofNamePreserved(index);
          },
          nb::arg("index"))
      .def(
          "setCommand",
          [](Joint& self, std::size_t index, double command) {
            self.setCommand(index, command);
          },
          nb::arg("index"),
          nb::arg("command"))
      .def(
          "getCommand",
          [](const Joint& self, std::size_t index) -> double {
            return self.getCommand(index);
          },
          nb::arg("index"))
      .def(
          "setCommands",
          [](Joint& self, const nb::handle& commands) {
            self.setCommands(toVector(commands));
          },
          nb::arg("commands"))
      .def("getCommands", [](const Joint& self) { return self.getCommands(); })
      .def("resetCommands", [](Joint& self) { self.resetCommands(); })
      .def(
          "setPosition",
          [](Joint& self, std::size_t index, double value) {
            self.setPosition(index, value);
          },
          nb::arg("index"),
          nb::arg("value"))
      .def(
          "setPositions",
          [](Joint& self, const nb::handle& positions) {
            self.setPositions(toVector(positions));
          },
          nb::arg("positions"))
      .def(
          "getPosition",
          [](const Joint& self, std::size_t index) -> double {
            return self.getPosition(index);
          },
          nb::arg("index"))
      .def(
          "getPositions", [](const Joint& self) { return self.getPositions(); })
      .def("resetPositions", [](Joint& self) { self.resetPositions(); })
      .def(
          "setVelocity",
          [](Joint& self, std::size_t index, double value) {
            self.setVelocity(index, value);
          },
          nb::arg("index"),
          nb::arg("value"))
      .def(
          "setVelocities",
          [](Joint& self, const nb::handle& velocities) {
            self.setVelocities(toVector(velocities));
          },
          nb::arg("velocities"))
      .def(
          "getVelocity",
          [](const Joint& self, std::size_t index) -> double {
            return self.getVelocity(index);
          },
          nb::arg("index"))
      .def(
          "getVelocities",
          [](const Joint& self) { return self.getVelocities(); })
      .def("resetVelocities", [](Joint& self) { self.resetVelocities(); })
      .def(
          "getParentBodyNode",
          [](Joint& self) -> dart::dynamics::BodyNode* {
            return self.getParentBodyNode();
          },
          nb::rv_policy::reference_internal)
      .def(
          "getChildBodyNode",
          [](Joint& self) -> dart::dynamics::BodyNode* {
            return self.getChildBodyNode();
          },
          nb::rv_policy::reference_internal)
      .def("getRelativeTransform", &Joint::getRelativeTransform)
      .def(
          "getRelativeJacobian",
          [](Joint& self, const nb::handle& positions) {
            return self.getRelativeJacobian(toVector(positions));
          },
          nb::arg("positions"))
      .def(
          "getRelativeJacobian",
          [](Joint& self) { return self.getRelativeJacobian(); })
      .def("getRelativeJacobianTimeDeriv", &Joint::getRelativeJacobianTimeDeriv)
      .def(
          "getTransformFromChildBodyNode",
          &Joint::getTransformFromChildBodyNode)
      .def(
          "getTransformFromParentBodyNode",
          &Joint::getTransformFromParentBodyNode)
      .def(
          "setTransformFromChildBodyNode",
          &Joint::setTransformFromChildBodyNode,
          nb::arg("transform"))
      .def(
          "setTransformFromParentBodyNode",
          &Joint::setTransformFromParentBodyNode,
          nb::arg("transform"))
      .def(
          "getWrenchToParentBodyNode",
          [](Joint& self, const dart::dynamics::Frame* frame) {
            return self.getWrenchToParentBodyNode(frame);
          },
          nb::arg("frame") = nullptr)
      .def(
          "getWrenchToChildBodyNode",
          [](Joint& self, const dart::dynamics::Frame* frame) {
            return self.getWrenchToChildBodyNode(frame);
          },
          nb::arg("frame") = nullptr)
      .def(
          "setLimitEnforcement",
          &Joint::setLimitEnforcement,
          nb::arg("enforce"))
      .def(
          "setAcceleration",
          [](Joint& self, std::size_t index, double value) {
            self.setAcceleration(index, value);
          },
          nb::arg("index"),
          nb::arg("value"))
      .def(
          "getAcceleration",
          [](const Joint& self, std::size_t index) -> double {
            return self.getAcceleration(index);
          },
          nb::arg("index"))
      .def(
          "setAccelerations",
          [](Joint& self, const nb::handle& accelerations) {
            self.setAccelerations(toVector(accelerations));
          },
          nb::arg("accelerations"))
      .def(
          "getAccelerations",
          [](Joint& self) { return self.getAccelerations(); })
      .def("resetAccelerations", [](Joint& self) { self.resetAccelerations(); })
      .def(
          "setForce",
          [](Joint& self, std::size_t index, double value) {
            self.setForce(index, value);
          },
          nb::arg("index"),
          nb::arg("value"))
      .def(
          "getForce",
          [](const Joint& self, std::size_t index) -> double {
            return self.getForce(index);
          },
          nb::arg("index"))
      .def(
          "setForces",
          [](Joint& self, const nb::handle& forces) {
            self.setForces(toVector(forces));
          },
          nb::arg("forces"))
      .def("getForces", [](Joint& self) { return self.getForces(); })
      .def("resetForces", [](Joint& self) { self.resetForces(); })
      .def(
          "setDampingCoefficient",
          [](Joint& self, std::size_t index, double coefficient) {
            self.setDampingCoefficient(index, coefficient);
          },
          nb::arg("index"),
          nb::arg("coefficient"))
      .def(
          "getDampingCoefficient",
          [](const Joint& self, std::size_t index) {
            return self.getDampingCoefficient(index);
          },
          nb::arg("index"));
}

} // namespace dart::python_nb
