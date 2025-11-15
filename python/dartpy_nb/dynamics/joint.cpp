#include "dynamics/joint.hpp"

#include "dart/dynamics/DegreeOfFreedom.hpp"
#include "dart/dynamics/Joint.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

namespace nb = nanobind;

namespace dart::python_nb {

void defJoint(nb::module_& m)
{
  using Joint = dart::dynamics::Joint;

  nb::class_<Joint>(m, "Joint")
      .def(
          "getName",
          [](const Joint& self) -> const std::string& {
            return self.getName();
          },
          nb::rv_policy::reference_internal)
      .def("setName", &Joint::setName, nb::arg("name"))
      .def(
          "getType",
          [](const Joint& self) -> const std::string& {
            return self.getType();
          },
          nb::rv_policy::reference_internal)
      .def("getNumDofs", &Joint::getNumDofs)
      .def(
          "getDof",
          [](Joint& self,
             std::size_t index) -> dart::dynamics::DegreeOfFreedom* {
            return self.getDof(index);
          },
          nb::rv_policy::reference_internal,
          nb::arg("index"))
      .def(
          "getDof",
          [](Joint& self, const std::string& name)
              -> dart::dynamics::DegreeOfFreedom* { return self.getDof(name); },
          nb::rv_policy::reference_internal,
          nb::arg("name"))
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
          nb::arg("preserveName"))
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
          [](Joint& self, const Eigen::VectorXd& commands) {
            self.setCommands(commands);
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
          [](Joint& self, const Eigen::VectorXd& positions) {
            self.setPositions(positions);
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
          [](Joint& self, const Eigen::VectorXd& velocities) {
            self.setVelocities(velocities);
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
      .def("getRelativeTransform", &Joint::getRelativeTransform)
      .def(
          "setRelativeTransform",
          [](Joint& self, const Eigen::Isometry3d& tf) {
            self.setRelativeTransform(tf);
          },
          nb::arg("transform"))
      .def(
          "getRelativeJacobian",
          [](Joint& self, const Eigen::VectorXd& positions) {
            return self.getRelativeJacobian(positions);
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
          [](Joint& self, const dart::dynamics::Frame& frame) {
            return self.getWrenchToParentBodyNode(frame);
          },
          nb::arg("frame"))
      .def(
          "getWrenchToChildBodyNode",
          [](Joint& self, const dart::dynamics::Frame& frame) {
            return self.getWrenchToChildBodyNode(frame);
          },
          nb::arg("frame"))
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
          [](Joint& self, const Eigen::VectorXd& accelerations) {
            self.setAccelerations(accelerations);
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
          [](Joint& self, const Eigen::VectorXd& forces) {
            self.setForces(forces);
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
