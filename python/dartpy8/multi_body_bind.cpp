/*
 * Copyright (c) 2011-2025, The DART development contributors
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

#include "multi_body_bind.hpp"

#include "dart8/common/logging.hpp"
#include "dart8/comps/joint.hpp"
#include "dart8/multi_body/joint.hpp"
#include "dart8/multi_body/link.hpp"
#include "dart8/multi_body/multi_body.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/string_view.h>

namespace nb = nanobind;

namespace dartpy8 {

void defMultiBody(nb::module_& m)
{
  DART8_DEBUG("Registering MultiBody class");

  // JointType enum
  nb::enum_<dart8::comps::JointType>(m, "JointType", "Joint type enumeration")
      .value("Fixed", dart8::comps::JointType::Fixed, "Fixed joint (0 DOF)")
      .value(
          "Revolute",
          dart8::comps::JointType::Revolute,
          "Revolute joint (1 DOF rotation)")
      .value(
          "Prismatic",
          dart8::comps::JointType::Prismatic,
          "Prismatic joint (1 DOF translation)")
      .value("Screw", dart8::comps::JointType::Screw, "Screw joint (1 DOF)")
      .value(
          "Universal",
          dart8::comps::JointType::Universal,
          "Universal joint (2 DOF rotation)")
      .value("Ball", dart8::comps::JointType::Ball, "Ball joint (3 DOF)")
      .value("Planar", dart8::comps::JointType::Planar, "Planar joint (3 DOF)")
      .value("Free", dart8::comps::JointType::Free, "Free joint (6 DOF)")
      .value("Custom", dart8::comps::JointType::Custom, "Custom joint type");

  // Link class
  nb::class_<dart8::Link, dart8::Frame>(m, "Link", "Link in a multibody")
      .def("get_name", &dart8::Link::getName, "Get the link name")
      .def(
          "get_parent_joint",
          &dart8::Link::getParentJoint,
          "Get the parent joint (invalid for root link)")
      .def("__repr__", [](const dart8::Link& self) {
        return "<dart8.Link '" + std::string(self.getName()) + "'>";
      });

  // Joint class
  nb::class_<dart8::Joint>(m, "Joint", "Joint in a multibody")
      .def("get_name", &dart8::Joint::getName, "Get the joint name")
      .def("get_type", &dart8::Joint::getType, "Get the joint type")
      .def("get_axis", &dart8::Joint::getAxis, "Get the primary joint axis")
      .def("get_axis2", &dart8::Joint::getAxis2, "Get the secondary joint axis")
      .def("get_pitch", &dart8::Joint::getPitch, "Get the screw pitch")
      .def("__repr__", [](const dart8::Joint& self) {
        return "<dart8.Joint '" + std::string(self.getName()) + "'>";
      });

  // LinkOptions struct - custom constructor to avoid default init issues
  nb::class_<dart8::LinkOptions>(
      m, "LinkOptions", "Options for creating a link with parent joint")
      .def(
          nb::init<
              dart8::Link,
              std::string,
              dart8::comps::JointType,
              Eigen::Vector3d>(),
          nb::arg("parent_link"),
          nb::arg("joint_name"),
          nb::arg("joint_type") = dart8::comps::JointType::Revolute,
          nb::arg("axis") = Eigen::Vector3d::UnitZ(),
          "Create LinkOptions\n\n"
          "Args:\n"
          "    parent_link: Parent link handle\n"
          "    joint_name: Name of the connecting joint\n"
          "    joint_type: Type of joint (default: Revolute)\n"
          "    axis: Joint axis (default: [0, 0, 1])")
      .def_rw("parent_link", &dart8::LinkOptions::parentLink, "Parent link")
      .def_rw("joint_name", &dart8::LinkOptions::jointName, "Joint name")
      .def_rw("joint_type", &dart8::LinkOptions::jointType, "Joint type")
      .def_rw("axis", &dart8::LinkOptions::axis, "Joint axis");

  // MultiBody class
  nb::class_<dart8::MultiBody>(
      m,
      "MultiBody",
      "Articulated rigid body system (robot, character, or mechanism)\n\n"
      "Note: MultiBody is a lightweight handle and cannot be created "
      "directly.\n"
      "      Use World.add_multi_body() to create instances.")
      .def_prop_rw(
          "name",
          &dart8::MultiBody::getName,
          &dart8::MultiBody::setName,
          "Name of this MultiBody")
      .def(
          "get_name",
          &dart8::MultiBody::getName,
          "Get the name of this MultiBody")
      .def(
          "set_name",
          &dart8::MultiBody::setName,
          nb::arg("name"),
          "Set the name of this MultiBody")
      .def(
          "get_link_count",
          &dart8::MultiBody::getLinkCount,
          "Get the number of links")
      .def(
          "get_joint_count",
          &dart8::MultiBody::getJointCount,
          "Get the number of joints")
      .def(
          "get_dof_count",
          &dart8::MultiBody::getDOFCount,
          "Get the total number of degrees of freedom")
      .def(
          "get_link",
          [](const dart8::MultiBody& self,
             const std::string& name) -> nb::object {
            auto result = self.getLink(name);
            if (result.has_value()) {
              return nb::cast(result.value());
            }
            return nb::none();
          },
          nb::arg("name"),
          "Get a link by name\n\n"
          "Args:\n"
          "    name: Name of the link\n\n"
          "Returns:\n"
          "    Link handle if found, None otherwise")
      .def(
          "get_joint",
          [](const dart8::MultiBody& self,
             const std::string& name) -> nb::object {
            auto result = self.getJoint(name);
            if (result.has_value()) {
              return nb::cast(result.value());
            }
            return nb::none();
          },
          nb::arg("name"),
          "Get a joint by name\n\n"
          "Args:\n"
          "    name: Name of the joint\n\n"
          "Returns:\n"
          "    Joint handle if found, None otherwise")
      .def(
          "add_link",
          nb::overload_cast<std::string_view>(&dart8::MultiBody::addLink),
          nb::arg("name") = "",
          "Add a root link (no parent joint)\n\n"
          "Args:\n"
          "    name: Link name (empty = auto-generate)\n\n"
          "Returns:\n"
          "    Link handle")
      .def(
          "add_link",
          nb::overload_cast<std::string_view, const dart8::LinkOptions&>(
              &dart8::MultiBody::addLink),
          nb::arg("name"),
          nb::arg("options"),
          "Add a link with parent joint\n\n"
          "Args:\n"
          "    name: Link name (empty = auto-generate)\n"
          "    options: LinkOptions specifying parent and joint config\n\n"
          "Returns:\n"
          "    Link handle")
      .def(
          "add_link",
          [](dart8::MultiBody& self,
             const std::string& name,
             const dart8::Link& parent_link,
             const std::string& joint_name,
             dart8::comps::JointType joint_type,
             const Eigen::Vector3d& axis) -> dart8::Link {
            dart8::LinkOptions opts{parent_link, joint_name, joint_type, axis};
            return self.addLink(name, opts);
          },
          nb::arg("name"),
          nb::arg("parent_link"),
          nb::arg("joint_name"),
          nb::arg("joint_type") = dart8::comps::JointType::Revolute,
          nb::arg("axis") = Eigen::Vector3d::UnitZ(),
          "Add a link with parent joint (convenience overload)\n\n"
          "Args:\n"
          "    name: Link name (empty = auto-generate)\n"
          "    parent_link: Parent link handle\n"
          "    joint_name: Name of the connecting joint\n"
          "    joint_type: Type of joint (default: Revolute)\n"
          "    axis: Joint axis (default: [0, 0, 1])\n\n"
          "Returns:\n"
          "    Link handle")
      .def("__repr__", [](const dart8::MultiBody& self) {
        std::string repr
            = "<dart8.MultiBody '" + std::string(self.getName()) + "': ";
        repr += std::to_string(self.getLinkCount()) + " link"
                + (self.getLinkCount() == 1 ? "" : "s") + ", ";
        repr += std::to_string(self.getJointCount()) + " joint"
                + (self.getJointCount() == 1 ? "" : "s") + ", ";
        repr += std::to_string(self.getDOFCount()) + " DOF"
                + (self.getDOFCount() == 1 ? "" : "s") + ">";
        return repr;
      });
}

} // namespace dartpy8
