/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include "simulation_experimental/module.hpp"

#include "common/eigen_utils.hpp"
#include "common/repr.hpp"

#include <dart/simulation/experimental/body/rigid_body.hpp>
#include <dart/simulation/experimental/body/rigid_body_options.hpp>
#include <dart/simulation/experimental/multi_body/joint.hpp>
#include <dart/simulation/experimental/multi_body/link.hpp>
#include <dart/simulation/experimental/multi_body/multi_body.hpp>
#include <dart/simulation/experimental/world.hpp>

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/array.h>
#include <nanobind/stl/string.h>

#include <array>
#include <string>
#include <utility>
#include <vector>

namespace nb = nanobind;

namespace dart::python_nb {

namespace {

namespace sim = dart::simulation::experimental;

struct JointSpec
{
  std::string name;
  sim::JointType type = sim::JointType::Revolute;
  Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();
};

Eigen::Quaterniond toQuaternionWxyz(const nb::handle& value)
{
  const auto data = nb::cast<std::array<double, 4>>(value);
  return Eigen::Quaterniond(data[0], data[1], data[2], data[3]);
}

Eigen::Vector4d toWxyz(const Eigen::Quaterniond& quaternion)
{
  Eigen::Vector4d out;
  out << quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z();
  return out;
}

sim::LinkOptions makeLinkOptions(
    const sim::Link& parent, const JointSpec& joint)
{
  return sim::LinkOptions{
      .parentLink = parent,
      .jointName = joint.name,
      .jointType = joint.type,
      .axis = joint.axis,
  };
}

sim::RigidBodyOptions makeRigidBodyOptions(
    double mass,
    const nb::handle& position,
    const nb::handle& orientation,
    const nb::handle& linearVelocity,
    const nb::handle& angularVelocity,
    const nb::handle& inertia)
{
  sim::RigidBodyOptions options;
  options.mass = mass;

  if (!position.is_none()) {
    options.position = toVector3(position);
  }
  if (!orientation.is_none()) {
    options.orientation = toQuaternionWxyz(orientation);
  }
  if (!linearVelocity.is_none()) {
    options.linearVelocity = toVector3(linearVelocity);
  }
  if (!angularVelocity.is_none()) {
    options.angularVelocity = toVector3(angularVelocity);
  }
  if (!inertia.is_none()) {
    options.inertia = toMatrix3(inertia);
  }

  return options;
}

sim::RigidBodyOptions mergeRigidBodyOptions(
    sim::RigidBodyOptions options,
    const nb::handle& mass,
    const nb::handle& position,
    const nb::handle& orientation,
    const nb::handle& linearVelocity,
    const nb::handle& angularVelocity,
    const nb::handle& inertia)
{
  if (!mass.is_none()) {
    options.mass = nb::cast<double>(mass);
  }
  if (!position.is_none()) {
    options.position = toVector3(position);
  }
  if (!orientation.is_none()) {
    options.orientation = toQuaternionWxyz(orientation);
  }
  if (!linearVelocity.is_none()) {
    options.linearVelocity = toVector3(linearVelocity);
  }
  if (!angularVelocity.is_none()) {
    options.angularVelocity = toVector3(angularVelocity);
  }
  if (!inertia.is_none()) {
    options.inertia = toMatrix3(inertia);
  }

  return options;
}

} // namespace

void defSimulationExperimentalModule(nb::module_& m)
{
  nb::enum_<sim::JointType>(m, "JointType")
      .value("FIXED", sim::JointType::Fixed)
      .value("REVOLUTE", sim::JointType::Revolute)
      .value("PRISMATIC", sim::JointType::Prismatic)
      .value("SCREW", sim::JointType::Screw)
      .value("UNIVERSAL", sim::JointType::Universal)
      .value("BALL", sim::JointType::Ball)
      .value("PLANAR", sim::JointType::Planar)
      .value("FREE", sim::JointType::Free)
      .value("CUSTOM", sim::JointType::Custom);

  nb::class_<JointSpec>(m, "JointSpec")
      .def(
          nb::new_([](std::string name,
                      sim::JointType type,
                      const nb::handle& axis) {
            JointSpec spec;
            spec.name = std::move(name);
            spec.type = type;
            if (!axis.is_none()) {
              spec.axis = toVector3(axis);
            }
            return spec;
          }),
          nb::arg("name") = "",
          nb::arg("type") = sim::JointType::Revolute,
          nb::arg("axis") = nb::none())
      .def_rw("name", &JointSpec::name)
      .def_rw("type", &JointSpec::type)
      .def_prop_rw(
          "axis",
          [](const JointSpec& self) { return self.axis; },
          [](JointSpec& self, const nb::handle& axis) {
            self.axis = toVector3(axis);
          })
      .def("__repr__", [](const JointSpec& self) {
        std::vector<std::pair<std::string, std::string>> fields;
        fields.emplace_back("name", repr_string(self.name));
        fields.emplace_back(
            "type", nb::cast<std::string>(nb::repr(nb::cast(self.type))));
        fields.emplace_back(
            "axis", nb::cast<std::string>(nb::repr(nb::cast(self.axis))));
        return format_repr("JointSpec", fields);
      });

  auto jointClass = nb::class_<sim::Joint>(m, "Joint");
  auto linkClass = nb::class_<sim::Link>(m, "Link");

  jointClass
      .def(
          "getName",
          [](const sim::Joint& self) { return std::string(self.getName()); })
      .def(
          "get_name",
          [](const sim::Joint& self) { return std::string(self.getName()); })
      .def("getType", &sim::Joint::getType)
      .def("get_type", &sim::Joint::getType)
      .def("getAxis", &sim::Joint::getAxis)
      .def("get_axis", &sim::Joint::getAxis)
      .def("getParentLink", &sim::Joint::getParentLink)
      .def("get_parent_link", &sim::Joint::getParentLink)
      .def("getChildLink", &sim::Joint::getChildLink)
      .def("get_child_link", &sim::Joint::getChildLink)
      .def("isValid", &sim::Joint::isValid)
      .def_prop_ro(
          "name",
          [](const sim::Joint& self) { return std::string(self.getName()); })
      .def_prop_ro("type", &sim::Joint::getType)
      .def_prop_ro("axis", &sim::Joint::getAxis)
      .def_prop_ro("parent_link", &sim::Joint::getParentLink)
      .def_prop_ro("child_link", &sim::Joint::getChildLink)
      .def_prop_ro("is_valid", &sim::Joint::isValid)
      .def("__repr__", [](const sim::Joint& self) {
        std::vector<std::pair<std::string, std::string>> fields;
        fields.emplace_back("valid", self.isValid() ? "True" : "False");
        if (self.isValid()) {
          fields.emplace_back("name", repr_string(std::string(self.getName())));
          fields.emplace_back(
              "type",
              nb::cast<std::string>(nb::repr(nb::cast(self.getType()))));
        }
        return format_repr("Joint", fields);
      });

  linkClass
      .def(
          "getName",
          [](const sim::Link& self) { return std::string(self.getName()); })
      .def(
          "get_name",
          [](const sim::Link& self) { return std::string(self.getName()); })
      .def("getParentJoint", &sim::Link::getParentJoint)
      .def("get_parent_joint", &sim::Link::getParentJoint)
      .def("isValid", &sim::Link::isValid)
      .def_prop_ro(
          "name",
          [](const sim::Link& self) { return std::string(self.getName()); })
      .def_prop_ro("parent_joint", &sim::Link::getParentJoint)
      .def_prop_ro("translation", &sim::Link::getTranslation)
      .def_prop_ro("rotation", &sim::Link::getRotation)
      .def_prop_ro(
          "quaternion",
          [](const sim::Link& self) { return toWxyz(self.getQuaternion()); })
      .def_prop_ro("transform", &sim::Link::getTransformMatrix)
      .def_prop_ro("is_valid", &sim::Link::isValid)
      .def("__repr__", [](const sim::Link& self) {
        std::vector<std::pair<std::string, std::string>> fields;
        fields.emplace_back("valid", self.isValid() ? "True" : "False");
        if (self.isValid()) {
          fields.emplace_back("name", repr_string(std::string(self.getName())));
        }
        return format_repr("Link", fields);
      });

  nb::class_<sim::MultiBody>(m, "MultiBody")
      .def(
          "getName",
          [](const sim::MultiBody& self) {
            return std::string(self.getName());
          })
      .def(
          "setName",
          [](sim::MultiBody& self, const std::string& name) {
            self.setName(name);
          },
          nb::arg("name"))
      .def("getLinkCount", &sim::MultiBody::getLinkCount)
      .def("getJointCount", &sim::MultiBody::getJointCount)
      .def("getDOFCount", &sim::MultiBody::getDOFCount)
      .def(
          "addLink",
          [](sim::MultiBody& self, const std::string& name) {
            return self.addLink(name);
          },
          nb::arg("name") = "",
          nb::keep_alive<0, 1>())
      .def(
          "add_link",
          [](sim::MultiBody& self, const std::string& name) {
            return self.addLink(name);
          },
          nb::arg("name") = "",
          nb::keep_alive<0, 1>())
      .def(
          "add_link",
          [](sim::MultiBody& self,
             const std::string& name,
             const sim::Link& parent,
             const JointSpec& joint) {
            return self.addLink(name, makeLinkOptions(parent, joint));
          },
          nb::arg("name"),
          nb::kw_only(),
          nb::arg("parent"),
          nb::arg("joint") = JointSpec{},
          nb::keep_alive<0, 1>())
      .def(
          "getLink",
          [](sim::MultiBody& self, const std::string& name) -> nb::object {
            auto link = self.getLink(name);
            if (!link.has_value()) {
              return nb::none();
            }
            return nb::cast(*link, nb::rv_policy::move);
          },
          nb::arg("name"),
          nb::keep_alive<0, 1>())
      .def(
          "get_link",
          [](sim::MultiBody& self, const std::string& name) -> nb::object {
            auto link = self.getLink(name);
            if (!link.has_value()) {
              return nb::none();
            }
            return nb::cast(*link, nb::rv_policy::move);
          },
          nb::arg("name"),
          nb::keep_alive<0, 1>())
      .def(
          "getJoint",
          [](sim::MultiBody& self, const std::string& name) -> nb::object {
            auto joint = self.getJoint(name);
            if (!joint.has_value()) {
              return nb::none();
            }
            return nb::cast(*joint, nb::rv_policy::move);
          },
          nb::arg("name"),
          nb::keep_alive<0, 1>())
      .def(
          "get_joint",
          [](sim::MultiBody& self, const std::string& name) -> nb::object {
            auto joint = self.getJoint(name);
            if (!joint.has_value()) {
              return nb::none();
            }
            return nb::cast(*joint, nb::rv_policy::move);
          },
          nb::arg("name"),
          nb::keep_alive<0, 1>())
      .def_prop_rw(
          "name",
          [](const sim::MultiBody& self) {
            return std::string(self.getName());
          },
          [](sim::MultiBody& self, const std::string& name) {
            self.setName(name);
          })
      .def_prop_ro("num_links", &sim::MultiBody::getLinkCount)
      .def_prop_ro("num_joints", &sim::MultiBody::getJointCount)
      .def_prop_ro("num_dofs", &sim::MultiBody::getDOFCount)
      .def("__repr__", [](const sim::MultiBody& self) {
        std::vector<std::pair<std::string, std::string>> fields;
        fields.emplace_back("name", repr_string(std::string(self.getName())));
        fields.emplace_back("links", std::to_string(self.getLinkCount()));
        fields.emplace_back("joints", std::to_string(self.getJointCount()));
        fields.emplace_back("dofs", std::to_string(self.getDOFCount()));
        return format_repr("MultiBody", fields);
      });

  nb::class_<sim::RigidBody>(m, "RigidBody")
      .def("getName", [](const sim::RigidBody& self) { return self.getName(); })
      .def_prop_ro(
          "name", [](const sim::RigidBody& self) { return self.getName(); })
      .def_prop_ro("translation", &sim::RigidBody::getTranslation)
      .def_prop_ro("rotation", &sim::RigidBody::getRotation)
      .def_prop_ro(
          "quaternion",
          [](const sim::RigidBody& self) {
            return toWxyz(self.getQuaternion());
          })
      .def_prop_ro("transform", &sim::RigidBody::getTransformMatrix)
      .def("__repr__", [](const sim::RigidBody& self) {
        std::vector<std::pair<std::string, std::string>> fields;
        fields.emplace_back("name", repr_string(self.getName()));
        return format_repr("RigidBody", fields);
      });

  nb::class_<sim::RigidBodyOptions>(m, "RigidBodyOptions")
      .def(
          nb::new_([](double mass,
                      const nb::handle& position,
                      const nb::handle& orientation,
                      const nb::handle& linearVelocity,
                      const nb::handle& angularVelocity,
                      const nb::handle& inertia) {
            return makeRigidBodyOptions(
                mass,
                position,
                orientation,
                linearVelocity,
                angularVelocity,
                inertia);
          }),
          nb::arg("mass") = 1.0,
          nb::arg("position") = nb::none(),
          nb::arg("orientation") = nb::none(),
          nb::arg("linear_velocity") = nb::none(),
          nb::arg("angular_velocity") = nb::none(),
          nb::arg("inertia") = nb::none())
      .def_rw("mass", &sim::RigidBodyOptions::mass)
      .def_prop_rw(
          "position",
          [](const sim::RigidBodyOptions& self) { return self.position; },
          [](sim::RigidBodyOptions& self, const nb::handle& position) {
            self.position = toVector3(position);
          })
      .def_prop_rw(
          "orientation",
          [](const sim::RigidBodyOptions& self) {
            return toWxyz(self.orientation);
          },
          [](sim::RigidBodyOptions& self, const nb::handle& orientation) {
            self.orientation = toQuaternionWxyz(orientation);
          })
      .def_prop_rw(
          "linear_velocity",
          [](const sim::RigidBodyOptions& self) { return self.linearVelocity; },
          [](sim::RigidBodyOptions& self, const nb::handle& linearVelocity) {
            self.linearVelocity = toVector3(linearVelocity);
          })
      .def_prop_rw(
          "angular_velocity",
          [](const sim::RigidBodyOptions& self) {
            return self.angularVelocity;
          },
          [](sim::RigidBodyOptions& self, const nb::handle& angularVelocity) {
            self.angularVelocity = toVector3(angularVelocity);
          })
      .def_prop_rw(
          "inertia",
          [](const sim::RigidBodyOptions& self) { return self.inertia; },
          [](sim::RigidBodyOptions& self, const nb::handle& inertia) {
            self.inertia = toMatrix3(inertia);
          })
      .def("__repr__", [](const sim::RigidBodyOptions& self) {
        std::vector<std::pair<std::string, std::string>> fields;
        fields.emplace_back("mass", repr_double(self.mass));
        fields.emplace_back(
            "position",
            nb::cast<std::string>(nb::repr(nb::cast(self.position))));
        fields.emplace_back(
            "linear_velocity",
            nb::cast<std::string>(nb::repr(nb::cast(self.linearVelocity))));
        fields.emplace_back(
            "angular_velocity",
            nb::cast<std::string>(nb::repr(nb::cast(self.angularVelocity))));
        return format_repr("RigidBodyOptions", fields);
      });

  nb::class_<sim::World>(m, "World")
      .def(
          "__init__",
          [](sim::World* self, double timeStep) {
            new (self) sim::World();
            self->setTimeStep(timeStep);
          },
          nb::arg("time_step") = 0.001)
      .def(
          "addMultiBody",
          [](sim::World& self, const std::string& name) {
            return self.addMultiBody(name);
          },
          nb::arg("name"),
          nb::keep_alive<0, 1>())
      .def(
          "add_multi_body",
          [](sim::World& self, const std::string& name) {
            return self.addMultiBody(name);
          },
          nb::arg("name"),
          nb::keep_alive<0, 1>())
      .def(
          "getMultiBody",
          [](sim::World& self, const std::string& name) -> nb::object {
            auto multiBody = self.getMultiBody(name);
            if (!multiBody.has_value()) {
              return nb::none();
            }
            return nb::cast(*multiBody, nb::rv_policy::move);
          },
          nb::arg("name"),
          nb::keep_alive<0, 1>())
      .def(
          "get_multi_body",
          [](sim::World& self, const std::string& name) -> nb::object {
            auto multiBody = self.getMultiBody(name);
            if (!multiBody.has_value()) {
              return nb::none();
            }
            return nb::cast(*multiBody, nb::rv_policy::move);
          },
          nb::arg("name"),
          nb::keep_alive<0, 1>())
      .def("getMultiBodyCount", &sim::World::getMultiBodyCount)
      .def("get_multi_body_count", &sim::World::getMultiBodyCount)
      .def(
          "addRigidBody",
          [](sim::World& self, const std::string& name) {
            return self.addRigidBody(name);
          },
          nb::arg("name"),
          nb::keep_alive<0, 1>())
      .def(
          "add_rigid_body",
          [](sim::World& self,
             const std::string& name,
             const sim::RigidBodyOptions& options,
             const nb::handle& mass,
             const nb::handle& position,
             const nb::handle& orientation,
             const nb::handle& linearVelocity,
             const nb::handle& angularVelocity,
             const nb::handle& inertia) {
            return self.addRigidBody(
                name,
                mergeRigidBodyOptions(
                    options,
                    mass,
                    position,
                    orientation,
                    linearVelocity,
                    angularVelocity,
                    inertia));
          },
          nb::arg("name"),
          nb::arg("options") = sim::RigidBodyOptions{},
          nb::kw_only(),
          nb::arg("mass") = nb::none(),
          nb::arg("position") = nb::none(),
          nb::arg("orientation") = nb::none(),
          nb::arg("linear_velocity") = nb::none(),
          nb::arg("angular_velocity") = nb::none(),
          nb::arg("inertia") = nb::none(),
          nb::keep_alive<0, 1>())
      .def(
          "hasRigidBody",
          [](const sim::World& self, const std::string& name) {
            return self.hasRigidBody(name);
          },
          nb::arg("name"))
      .def(
          "has_rigid_body",
          [](const sim::World& self, const std::string& name) {
            return self.hasRigidBody(name);
          },
          nb::arg("name"))
      .def("getRigidBodyCount", &sim::World::getRigidBodyCount)
      .def("get_rigid_body_count", &sim::World::getRigidBodyCount)
      .def("isSimulationMode", &sim::World::isSimulationMode)
      .def_prop_ro("is_simulation_mode", &sim::World::isSimulationMode)
      .def("enterSimulationMode", &sim::World::enterSimulationMode)
      .def("enter_simulation_mode", &sim::World::enterSimulationMode)
      .def(
          "updateKinematics",
          &sim::World::updateKinematics,
          nb::call_guard<nb::gil_scoped_release>())
      .def(
          "update_kinematics",
          &sim::World::updateKinematics,
          nb::call_guard<nb::gil_scoped_release>())
      .def(
          "step",
          [](sim::World& self, std::size_t n) {
            if (n == 0) {
              return;
            }
            if (!self.isSimulationMode()) {
              self.enterSimulationMode();
            }
            self.step(n);
          },
          nb::arg("n") = 1,
          nb::call_guard<nb::gil_scoped_release>())
      .def_prop_rw(
          "time_step", &sim::World::getTimeStep, &sim::World::setTimeStep)
      .def_prop_rw("time", &sim::World::getTime, &sim::World::setTime)
      .def_prop_ro("frame", &sim::World::getFrame)
      .def_prop_ro("num_multi_bodies", &sim::World::getMultiBodyCount)
      .def_prop_ro("num_rigid_bodies", &sim::World::getRigidBodyCount)
      .def("clear", &sim::World::clear)
      .def("__repr__", [](const sim::World& self) {
        std::vector<std::pair<std::string, std::string>> fields;
        fields.emplace_back(
            "multi_bodies", std::to_string(self.getMultiBodyCount()));
        fields.emplace_back(
            "rigid_bodies", std::to_string(self.getRigidBodyCount()));
        fields.emplace_back("time", repr_double(self.getTime()));
        fields.emplace_back("time_step", repr_double(self.getTimeStep()));
        fields.emplace_back("frame", std::to_string(self.getFrame()));
        fields.emplace_back(
            "simulation_mode",
            self.isSimulationMode() ? std::string("True")
                                    : std::string("False"));
        return format_repr("World", fields);
      });
}

} // namespace dart::python_nb
