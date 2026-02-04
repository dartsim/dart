#include "dynamics/degree_of_freedom.hpp"

#include "common/repr.hpp"
#include "dart/dynamics/degree_of_freedom.hpp"
#include "dart/dynamics/skeleton.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

namespace nb = nanobind;

namespace dart::python_nb {

void defDegreeOfFreedom(nb::module_& m)
{
  using DegreeOfFreedom = dart::dynamics::DegreeOfFreedom;

  nb::class_<DegreeOfFreedom, dart::common::Subject>(m, "DegreeOfFreedom")
      .def(
          "setName",
          [](DegreeOfFreedom& self, const std::string& name)
              -> const std::string& { return self.setName(name); },
          nb::rv_policy::reference_internal,
          nb::arg("name"))
      .def(
          "setName",
          [](DegreeOfFreedom& self,
             const std::string& name,
             bool preserve_name) -> const std::string& {
            return self.setName(name, preserve_name);
          },
          nb::rv_policy::reference_internal,
          nb::arg("name"),
          nb::arg("preserve_name"))
      .def(
          "getName",
          [](const DegreeOfFreedom& self) -> const std::string& {
            return self.getName();
          },
          nb::rv_policy::reference_internal)
      .def(
          "__repr__",
          [](const DegreeOfFreedom& self) {
            const auto skeleton = self.getSkeleton();
            const auto* joint = self.getJoint();
            std::vector<std::pair<std::string, std::string>> fields;
            fields.emplace_back("name", repr_string(self.getName()));
            fields.emplace_back(
                "index", std::to_string(self.getIndexInSkeleton()));
            fields.emplace_back(
                "index_in_joint", std::to_string(self.getIndexInJoint()));
            fields.emplace_back(
                "joint", joint ? repr_string(joint->getName()) : "None");
            fields.emplace_back(
                "skeleton",
                skeleton ? repr_string(skeleton->getName()) : "None");
            return format_repr("DegreeOfFreedom", fields);
          })
      .def(
          "preserveName",
          [](DegreeOfFreedom& self, bool preserve) {
            self.preserveName(preserve);
          },
          nb::arg("preserve"))
      .def(
          "isNamePreserved",
          [](const DegreeOfFreedom& self) -> bool {
            return self.isNamePreserved();
          })
      .def(
          "getIndexInSkeleton",
          [](const DegreeOfFreedom& self) -> std::size_t {
            return self.getIndexInSkeleton();
          })
      .def(
          "getIndexInTree",
          [](const DegreeOfFreedom& self) -> std::size_t {
            return self.getIndexInTree();
          })
      .def(
          "getIndexInJoint",
          [](const DegreeOfFreedom& self) -> std::size_t {
            return self.getIndexInJoint();
          })
      .def(
          "getTreeIndex",
          [](const DegreeOfFreedom& self) -> std::size_t {
            return self.getTreeIndex();
          })
      .def(
          "setCommand",
          [](DegreeOfFreedom& self, double command) {
            self.setCommand(command);
          },
          nb::arg("command"))
      .def(
          "getCommand",
          [](const DegreeOfFreedom& self) -> double {
            return self.getCommand();
          })
      .def("resetCommand", [](DegreeOfFreedom& self) { self.resetCommand(); })
      .def(
          "setPosition",
          [](DegreeOfFreedom& self, double position) {
            self.setPosition(position);
          },
          nb::arg("position"))
      .def(
          "getPosition",
          [](const DegreeOfFreedom& self) -> double {
            return self.getPosition();
          })
      .def(
          "setPositionLimits",
          [](DegreeOfFreedom& self, double lower_limit, double upper_limit) {
            self.setPositionLimits(lower_limit, upper_limit);
          },
          nb::arg("lower_limit"),
          nb::arg("upper_limit"))
      .def(
          "setPositionLimits",
          [](DegreeOfFreedom& self, const std::pair<double, double>& limits) {
            self.setPositionLimits(limits);
          },
          nb::arg("limits"))
      .def(
          "getPositionLimits",
          [](const DegreeOfFreedom& self) { return self.getPositionLimits(); })
      .def(
          "setPositionLowerLimit",
          [](DegreeOfFreedom& self, double limit) {
            self.setPositionLowerLimit(limit);
          },
          nb::arg("limit"))
      .def(
          "getPositionLowerLimit",
          [](const DegreeOfFreedom& self) -> double {
            return self.getPositionLowerLimit();
          })
      .def(
          "setPositionUpperLimit",
          [](DegreeOfFreedom& self, double limit) {
            self.setPositionUpperLimit(limit);
          },
          nb::arg("limit"))
      .def(
          "getPositionUpperLimit",
          [](const DegreeOfFreedom& self) -> double {
            return self.getPositionUpperLimit();
          })
      .def(
          "isCyclic",
          [](const DegreeOfFreedom& self) -> bool { return self.isCyclic(); })
      .def(
          "hasPositionLimit",
          [](const DegreeOfFreedom& self) -> bool {
            return self.hasPositionLimit();
          })
      .def("resetPosition", [](DegreeOfFreedom& self) { self.resetPosition(); })
      .def(
          "setInitialPosition",
          [](DegreeOfFreedom& self, double initial) {
            self.setInitialPosition(initial);
          },
          nb::arg("initial"))
      .def(
          "getInitialPosition",
          [](const DegreeOfFreedom& self) -> double {
            return self.getInitialPosition();
          })
      .def(
          "setVelocity",
          [](DegreeOfFreedom& self, double velocity) {
            self.setVelocity(velocity);
          },
          nb::arg("velocity"))
      .def(
          "getVelocity",
          [](const DegreeOfFreedom& self) -> double {
            return self.getVelocity();
          })
      .def(
          "setVelocityLimits",
          [](DegreeOfFreedom& self, double lower_limit, double upper_limit) {
            self.setVelocityLimits(lower_limit, upper_limit);
          },
          nb::arg("lower_limit"),
          nb::arg("upper_limit"))
      .def(
          "setVelocityLimits",
          [](DegreeOfFreedom& self, const std::pair<double, double>& limits) {
            self.setVelocityLimits(limits);
          },
          nb::arg("limits"))
      .def(
          "getVelocityLimits",
          [](const DegreeOfFreedom& self) { return self.getVelocityLimits(); })
      .def(
          "setVelocityLowerLimit",
          [](DegreeOfFreedom& self, double limit) {
            self.setVelocityLowerLimit(limit);
          },
          nb::arg("limit"))
      .def(
          "getVelocityLowerLimit",
          [](const DegreeOfFreedom& self) -> double {
            return self.getVelocityLowerLimit();
          })
      .def(
          "setVelocityUpperLimit",
          [](DegreeOfFreedom& self, double limit) {
            self.setVelocityUpperLimit(limit);
          },
          nb::arg("limit"))
      .def(
          "getVelocityUpperLimit",
          [](const DegreeOfFreedom& self) -> double {
            return self.getVelocityUpperLimit();
          })
      .def("resetVelocity", [](DegreeOfFreedom& self) { self.resetVelocity(); })
      .def(
          "setInitialVelocity",
          [](DegreeOfFreedom& self, double initial) {
            self.setInitialVelocity(initial);
          },
          nb::arg("initial"))
      .def(
          "getInitialVelocity",
          [](const DegreeOfFreedom& self) -> double {
            return self.getInitialVelocity();
          })
      .def(
          "setAcceleration",
          [](DegreeOfFreedom& self, double acceleration) {
            self.setAcceleration(acceleration);
          },
          nb::arg("acceleration"))
      .def(
          "getAcceleration",
          [](const DegreeOfFreedom& self) -> double {
            return self.getAcceleration();
          })
      .def(
          "resetAcceleration",
          [](DegreeOfFreedom& self) { self.resetAcceleration(); })
      .def(
          "setAccelerationLimits",
          [](DegreeOfFreedom& self, double lower_limit, double upper_limit) {
            self.setAccelerationLimits(lower_limit, upper_limit);
          },
          nb::arg("lower_limit"),
          nb::arg("upper_limit"))
      .def(
          "setAccelerationLimits",
          [](DegreeOfFreedom& self, const std::pair<double, double>& limits) {
            self.setAccelerationLimits(limits);
          },
          nb::arg("limits"))
      .def(
          "getAccelerationLimits",
          [](const DegreeOfFreedom& self) {
            return self.getAccelerationLimits();
          })
      .def(
          "setAccelerationLowerLimit",
          [](DegreeOfFreedom& self, double limit) {
            self.setAccelerationLowerLimit(limit);
          },
          nb::arg("limit"))
      .def(
          "getAccelerationLowerLimit",
          [](const DegreeOfFreedom& self) -> double {
            return self.getAccelerationLowerLimit();
          })
      .def(
          "setAccelerationUpperLimit",
          [](DegreeOfFreedom& self, double limit) {
            self.setAccelerationUpperLimit(limit);
          },
          nb::arg("limit"))
      .def(
          "getAccelerationUpperLimit",
          [](const DegreeOfFreedom& self) -> double {
            return self.getAccelerationUpperLimit();
          })
      .def(
          "setForce",
          [](DegreeOfFreedom& self, double force) { self.setForce(force); },
          nb::arg("force"))
      .def(
          "getForce",
          [](const DegreeOfFreedom& self) -> double { return self.getForce(); })
      .def("resetForce", [](DegreeOfFreedom& self) { self.resetForce(); })
      .def(
          "setForceLimits",
          [](DegreeOfFreedom& self, double lower_limit, double upper_limit) {
            self.setForceLimits(lower_limit, upper_limit);
          },
          nb::arg("lower_limit"),
          nb::arg("upper_limit"))
      .def(
          "setForceLimits",
          [](DegreeOfFreedom& self, const std::pair<double, double>& limits) {
            self.setForceLimits(limits);
          },
          nb::arg("limits"))
      .def(
          "getForceLimits",
          [](const DegreeOfFreedom& self) { return self.getForceLimits(); })
      .def(
          "setForceLowerLimit",
          [](DegreeOfFreedom& self, double limit) {
            self.setForceLowerLimit(limit);
          },
          nb::arg("limit"))
      .def(
          "getForceLowerLimit",
          [](const DegreeOfFreedom& self) -> double {
            return self.getForceLowerLimit();
          })
      .def(
          "setForceUpperLimit",
          [](DegreeOfFreedom& self, double limit) {
            self.setForceUpperLimit(limit);
          },
          nb::arg("limit"))
      .def(
          "getForceUpperLimit",
          [](const DegreeOfFreedom& self) -> double {
            return self.getForceUpperLimit();
          })
      .def(
          "setVelocityChange",
          [](DegreeOfFreedom& self, double velocity_change) {
            self.setVelocityChange(velocity_change);
          },
          nb::arg("velocity_change"))
      .def(
          "getVelocityChange",
          [](const DegreeOfFreedom& self) -> double {
            return self.getVelocityChange();
          })
      .def(
          "resetVelocityChange",
          [](DegreeOfFreedom& self) { self.resetVelocityChange(); })
      .def(
          "setConstraintImpulse",
          [](DegreeOfFreedom& self, double impulse) {
            self.setConstraintImpulse(impulse);
          },
          nb::arg("impulse"))
      .def(
          "getConstraintImpulse",
          [](const DegreeOfFreedom& self) -> double {
            return self.getConstraintImpulse();
          })
      .def(
          "resetConstraintImpulse",
          [](DegreeOfFreedom& self) { self.resetConstraintImpulse(); })
      .def(
          "setSpringStiffness",
          [](DegreeOfFreedom& self, double stiffness) {
            self.setSpringStiffness(stiffness);
          },
          nb::arg("k"))
      .def(
          "getSpringStiffness",
          [](const DegreeOfFreedom& self) -> double {
            return self.getSpringStiffness();
          })
      .def(
          "setRestPosition",
          [](DegreeOfFreedom& self, double rest) {
            self.setRestPosition(rest);
          },
          nb::arg("q0"))
      .def(
          "getRestPosition",
          [](const DegreeOfFreedom& self) -> double {
            return self.getRestPosition();
          })
      .def(
          "setDampingCoefficient",
          [](DegreeOfFreedom& self, double coefficient) {
            self.setDampingCoefficient(coefficient);
          },
          nb::arg("coeff"))
      .def(
          "getDampingCoefficient",
          [](const DegreeOfFreedom& self) -> double {
            return self.getDampingCoefficient();
          })
      .def(
          "setCoulombFriction",
          [](DegreeOfFreedom& self, double friction) {
            self.setCoulombFriction(friction);
          },
          nb::arg("friction"))
      .def(
          "getCoulombFriction",
          [](const DegreeOfFreedom& self) -> double {
            return self.getCoulombFriction();
          })
      .def(
          "getSkeleton",
          [](DegreeOfFreedom& self) { return self.getSkeleton(); })
      .def("getSkeleton", [](const DegreeOfFreedom& self) {
        return self.getSkeleton();
      });
}

} // namespace dart::python_nb
