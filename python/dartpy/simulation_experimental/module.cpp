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

#include "common/repr.hpp"

#include <dart/simulation/experimental/body/rigid_body.hpp>
#include <dart/simulation/experimental/multi_body/multi_body.hpp>
#include <dart/simulation/experimental/world.hpp>

#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>

#include <string>
#include <utility>
#include <vector>

namespace nb = nanobind;

namespace dart::python_nb {

void defSimulationExperimentalModule(nb::module_& m)
{
  namespace sim = dart::simulation::experimental;

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
      .def("__repr__", [](const sim::RigidBody& self) {
        std::vector<std::pair<std::string, std::string>> fields;
        fields.emplace_back("name", repr_string(self.getName()));
        return format_repr("RigidBody", fields);
      });

  nb::class_<sim::World>(m, "World")
      .def(nb::init<>())
      .def(
          "addMultiBody",
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
      .def("getMultiBodyCount", &sim::World::getMultiBodyCount)
      .def(
          "addRigidBody",
          [](sim::World& self, const std::string& name) {
            return self.addRigidBody(name);
          },
          nb::arg("name"),
          nb::keep_alive<0, 1>())
      .def(
          "hasRigidBody",
          [](const sim::World& self, const std::string& name) {
            return self.hasRigidBody(name);
          },
          nb::arg("name"))
      .def("getRigidBodyCount", &sim::World::getRigidBodyCount)
      .def("isSimulationMode", &sim::World::isSimulationMode)
      .def("enterSimulationMode", &sim::World::enterSimulationMode)
      .def("updateKinematics", &sim::World::updateKinematics)
      .def("clear", &sim::World::clear)
      .def("__repr__", [](const sim::World& self) {
        std::vector<std::pair<std::string, std::string>> fields;
        fields.emplace_back(
            "multi_bodies", std::to_string(self.getMultiBodyCount()));
        fields.emplace_back(
            "rigid_bodies", std::to_string(self.getRigidBodyCount()));
        fields.emplace_back(
            "simulation_mode",
            self.isSimulationMode() ? std::string("True")
                                    : std::string("False"));
        return format_repr("World", fields);
      });
}

} // namespace dart::python_nb
