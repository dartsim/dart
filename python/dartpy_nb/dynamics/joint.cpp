#include "dynamics/joint.hpp"

#include "dart/dynamics/DegreeOfFreedom.hpp"
#include "dart/dynamics/Frame.hpp"
#include "dart/dynamics/Joint.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include <utility>

#include "common/polymorphic_utils.hpp"
#include "common/type_casters.hpp"

namespace nb = nanobind;

namespace dart::python_nb {

namespace {

inline dart::dynamics::Joint& joint_ref(nb::handle self)
{
  return refFromHandle<dart::dynamics::Joint>(self);
}

template <typename Func>
auto joint_method(Func&& func)
{
  return [fn = std::forward<Func>(func)](nb::handle self, auto&&... args)
             -> decltype(auto) {
    return fn(joint_ref(self), std::forward<decltype(args)>(args)...);
  };
}

template <typename Func>
auto joint_const_method(Func&& func)
{
  return [fn = std::forward<Func>(func)](nb::handle self, auto&&... args)
             -> decltype(auto) {
    const auto& joint = joint_ref(self);
    return fn(joint, std::forward<decltype(args)>(args)...);
  };
}

} // namespace

void defJoint(nb::module_& m)
{
  using Joint = dart::dynamics::Joint;

  nb::class_<Joint>(m, "Joint")
      .def(
          "getName",
          joint_const_method([](const Joint& self) -> const std::string& {
            return self.getName();
          }),
          nb::is_method(),
          nb::rv_policy::reference_internal)
      .def(
          "setName",
          joint_method(
              [](Joint& self, const std::string& name, bool preserve)
                  -> const std::string& { return self.setName(name, preserve); }),
          nb::is_method(),
          nb::rv_policy::reference_internal,
          nb::arg("name"),
          nb::arg("preserveName") = true)
      .def(
          "getType",
          joint_const_method([](const Joint& self) -> const std::string& {
            return self.getType();
          }),
          nb::is_method(),
          nb::rv_policy::reference_internal)
      .def(
          "getNumDofs",
          joint_const_method([](const Joint& self) { return self.getNumDofs(); }),
          nb::is_method())
      .def(
          "getDof",
          joint_method(
              [](Joint& self, std::size_t index) -> dart::dynamics::DegreeOfFreedom* {
                return self.getDof(index);
              }),
          nb::is_method(),
          nb::rv_policy::reference_internal,
          nb::arg("index"))
      .def(
          "setDofName",
          joint_method(
              [](Joint& self, std::size_t index, const std::string& name)
                  -> const std::string& { return self.setDofName(index, name); }),
          nb::is_method(),
          nb::rv_policy::reference_internal,
          nb::arg("index"),
          nb::arg("name"))
      .def(
          "setDofName",
          joint_method(
              [](Joint& self,
                 std::size_t index,
                 const std::string& name,
                 bool preserve) -> const std::string& {
                return self.setDofName(index, name, preserve);
              }),
          nb::is_method(),
          nb::rv_policy::reference_internal,
          nb::arg("index"),
          nb::arg("name"),
          nb::arg("preserveName"))
      .def(
          "getDofName",
          joint_const_method(
              [](const Joint& self, std::size_t index) -> const std::string& {
                return self.getDofName(index);
              }),
          nb::is_method(),
          nb::rv_policy::reference_internal,
          nb::arg("index"))
      .def(
          "preserveDofName",
          joint_method([](Joint& self, std::size_t index, bool preserve) {
            self.preserveDofName(index, preserve);
          }),
          nb::is_method(),
          nb::arg("index"),
          nb::arg("preserve"))
      .def(
          "isDofNamePreserved",
          joint_const_method([](const Joint& self, std::size_t index) -> bool {
            return self.isDofNamePreserved(index);
          }),
          nb::is_method(),
          nb::arg("index"))
      .def(
          "setCommand",
          joint_method([](Joint& self, std::size_t index, double command) {
            self.setCommand(index, command);
          }),
          nb::is_method(),
          nb::arg("index"),
          nb::arg("command"))
      .def(
          "getCommand",
          joint_const_method([](const Joint& self, std::size_t index) -> double {
            return self.getCommand(index);
          }),
          nb::is_method(),
          nb::arg("index"))
      .def(
          "setCommands",
          joint_method([](Joint& self, const Eigen::VectorXd& commands) {
            self.setCommands(commands);
          }),
          nb::is_method(),
          nb::arg("commands"))
      .def(
          "getCommands",
          joint_const_method([](const Joint& self) { return self.getCommands(); }),
          nb::is_method())
      .def(
          "resetCommands",
          joint_method([](Joint& self) { self.resetCommands(); }),
          nb::is_method())
      .def(
          "setPosition",
          joint_method([](Joint& self, std::size_t index, double value) {
            self.setPosition(index, value);
          }),
          nb::is_method(),
          nb::arg("index"),
          nb::arg("value"))
      .def(
          "setPositions",
          joint_method([](Joint& self, const Eigen::VectorXd& positions) {
            self.setPositions(positions);
          }),
          nb::is_method(),
          nb::arg("positions"))
      .def(
          "getPosition",
          joint_const_method([](const Joint& self, std::size_t index) -> double {
            return self.getPosition(index);
          }),
          nb::is_method(),
          nb::arg("index"))
      .def(
          "getPositions",
          joint_const_method([](const Joint& self) { return self.getPositions(); }),
          nb::is_method())
      .def(
          "resetPositions",
          joint_method([](Joint& self) { self.resetPositions(); }),
          nb::is_method())
      .def(
          "setVelocity",
          joint_method([](Joint& self, std::size_t index, double value) {
            self.setVelocity(index, value);
          }),
          nb::is_method(),
          nb::arg("index"),
          nb::arg("value"))
      .def(
          "setVelocities",
          joint_method([](Joint& self, const Eigen::VectorXd& velocities) {
            self.setVelocities(velocities);
          }),
          nb::is_method(),
          nb::arg("velocities"))
      .def(
          "getVelocity",
          joint_const_method([](const Joint& self, std::size_t index) -> double {
            return self.getVelocity(index);
          }),
          nb::is_method(),
          nb::arg("index"))
      .def(
          "getVelocities",
          joint_const_method([](const Joint& self) { return self.getVelocities(); }),
          nb::is_method())
      .def(
          "resetVelocities",
          joint_method([](Joint& self) { self.resetVelocities(); }),
          nb::is_method())
      .def(
          "getRelativeTransform",
          joint_method([](Joint& self) { return self.getRelativeTransform(); }),
          nb::is_method())
      .def(
          "getRelativeJacobian",
          joint_method([](Joint& self, const Eigen::VectorXd& positions) {
            return self.getRelativeJacobian(positions);
          }),
          nb::is_method(),
          nb::arg("positions"))
      .def(
          "getRelativeJacobian",
          joint_method([](Joint& self) { return self.getRelativeJacobian(); }),
          nb::is_method())
      .def(
          "getRelativeJacobianTimeDeriv",
          joint_method([](Joint& self) { return self.getRelativeJacobianTimeDeriv(); }),
          nb::is_method())
      .def(
          "getTransformFromChildBodyNode",
          joint_method([](Joint& self) { return self.getTransformFromChildBodyNode(); }),
          nb::is_method())
      .def(
          "getTransformFromParentBodyNode",
          joint_method([](Joint& self) { return self.getTransformFromParentBodyNode(); }),
          nb::is_method())
      .def(
          "setTransformFromChildBodyNode",
          joint_method(
              [](Joint& self, const Eigen::Isometry3d& transform) {
                self.setTransformFromChildBodyNode(transform);
              }),
          nb::is_method(),
          nb::arg("transform"))
      .def(
          "setTransformFromParentBodyNode",
          joint_method(
              [](Joint& self, const Eigen::Isometry3d& transform) {
                self.setTransformFromParentBodyNode(transform);
              }),
          nb::is_method(),
          nb::arg("transform"))
      .def(
          "getWrenchToParentBodyNode",
          joint_method(
              [](Joint& self, const dart::dynamics::Frame* frame) {
                return self.getWrenchToParentBodyNode(frame);
              }),
          nb::is_method(),
          nb::arg("frame") = nullptr)
      .def(
          "getWrenchToChildBodyNode",
          joint_method(
              [](Joint& self, const dart::dynamics::Frame* frame) {
                return self.getWrenchToChildBodyNode(frame);
              }),
          nb::is_method(),
          nb::arg("frame") = nullptr)
      .def(
          "setLimitEnforcement",
          joint_method(
              [](Joint& self, bool enforce) { self.setLimitEnforcement(enforce); }),
          nb::is_method(),
          nb::arg("enforce"))
      .def(
          "setAcceleration",
          joint_method([](Joint& self, std::size_t index, double value) {
            self.setAcceleration(index, value);
          }),
          nb::is_method(),
          nb::arg("index"),
          nb::arg("value"))
      .def(
          "getAcceleration",
          joint_const_method([](const Joint& self, std::size_t index) -> double {
            return self.getAcceleration(index);
          }),
          nb::is_method(),
          nb::arg("index"))
      .def(
          "setAccelerations",
          joint_method([](Joint& self, const Eigen::VectorXd& accelerations) {
            self.setAccelerations(accelerations);
          }),
          nb::is_method(),
          nb::arg("accelerations"))
      .def(
          "getAccelerations",
          joint_const_method([](const Joint& self) { return self.getAccelerations(); }),
          nb::is_method())
      .def(
          "resetAccelerations",
          joint_method([](Joint& self) { self.resetAccelerations(); }),
          nb::is_method())
      .def(
          "setForce",
          joint_method([](Joint& self, std::size_t index, double value) {
            self.setForce(index, value);
          }),
          nb::is_method(),
          nb::arg("index"),
          nb::arg("value"))
      .def(
          "getForce",
          joint_const_method([](const Joint& self, std::size_t index) -> double {
            return self.getForce(index);
          }),
          nb::is_method(),
          nb::arg("index"))
      .def(
          "setForces",
          joint_method([](Joint& self, const Eigen::VectorXd& forces) {
            self.setForces(forces);
          }),
          nb::is_method(),
          nb::arg("forces"))
      .def(
          "getForces",
          joint_const_method([](const Joint& self) { return self.getForces(); }),
          nb::is_method())
      .def(
          "resetForces",
          joint_method([](Joint& self) { self.resetForces(); }),
          nb::is_method())
      .def(
          "setDampingCoefficient",
          joint_method([](Joint& self, std::size_t index, double coefficient) {
            self.setDampingCoefficient(index, coefficient);
          }),
          nb::is_method(),
          nb::arg("index"),
          nb::arg("coefficient"))
      .def(
          "getDampingCoefficient",
          joint_const_method([](const Joint& self, std::size_t index) {
            return self.getDampingCoefficient(index);
          }),
          nb::is_method(),
          nb::arg("index"));

  registerPolymorphicCaster<Joint, Joint>();
}

} // namespace dart::python_nb
