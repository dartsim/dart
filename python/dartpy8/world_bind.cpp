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

#include "world_bind.hpp"

#include "dart8/body/rigid_body.hpp"
#include "dart8/common/logging.hpp"
#include "dart8/frame/fixed_frame.hpp"
#include "dart8/frame/free_frame.hpp"
#include "dart8/multi_body/multi_body.hpp"
#include "dart8/world.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/string_view.h>

#include <fstream>
#include <sstream>

namespace nb = nanobind;

namespace dartpy8 {

void defWorld(nb::module_& m)
{
  DART8_DEBUG("Registering World class");

  // WorldOptions struct
  nb::class_<dart8::WorldOptions>(
      m, "WorldOptions", "Options for creating a World")
      .def(nb::init<>(), "Create default WorldOptions");

  // World class
  nb::class_<dart8::World>(m, "World", "Physics world simulation")
      .def(nb::init<>(), "Create a new physics world")
      .def(
          nb::init<const dart8::WorldOptions&>(),
          nb::arg("options"),
          "Create a new physics world with options")
      .def(
          "add_multi_body",
          &dart8::World::addMultiBody,
          nb::arg("name") = "",
          "Create a MultiBody\n\n"
          "Args:\n"
          "    name: Name of the multibody system (empty = auto-generate)\n\n"
          "Returns:\n"
          "    MultiBody handle to the created entity")
      .def(
          "get_multi_body_count",
          &dart8::World::getMultiBodyCount,
          "Get the number of MultiBody entities\n\n"
          "Returns:\n"
          "    Number of multibodies in the world")
      // RigidBody management
      .def(
          "add_rigid_body",
          [](dart8::World& self) -> dart8::RigidBody {
            return self.addRigidBody();
          },
          "Create a RigidBody with auto-generated name\n\n"
          "Returns:\n"
          "    RigidBody handle to the created entity")
      .def(
          "add_rigid_body",
          [](dart8::World& self, const std::string& name) -> dart8::RigidBody {
            return self.addRigidBody(name);
          },
          nb::arg("name"),
          "Create a RigidBody with a name\n\n"
          "Args:\n"
          "    name: Name of the rigid body\n\n"
          "Returns:\n"
          "    RigidBody handle to the created entity")
      .def(
          "get_rigid_body_count",
          &dart8::World::getRigidBodyCount,
          "Get the number of RigidBody entities\n\n"
          "Returns:\n"
          "    Number of rigid bodies in the world")
      .def(
          "get_rigid_body_count",
          &dart8::World::getRigidBodyCount,
          "Get the number of RigidBody entities (alias)\n\n"
          "Returns:\n"
          "    Number of rigid bodies in the world")
      .def(
          "add_rigid_body",
          [](dart8::World& self) -> dart8::RigidBody {
            return self.addRigidBody();
          },
          "Create a RigidBody with default options (alias)\n\n"
          "Returns:\n"
          "    RigidBody handle to the created entity")
      .def(
          "add_rigid_body",
          [](dart8::World& self, const std::string& name) -> dart8::RigidBody {
            return self.addRigidBody(name);
          },
          nb::arg("name"),
          "Create a RigidBody with a name (alias)\n\n"
          "Args:\n"
          "    name: Name of the rigid body\n\n"
          "Returns:\n"
          "    RigidBody handle to the created entity")
      .def(
          "get_multi_body_count",
          &dart8::World::getMultiBodyCount,
          "Get the number of MultiBody entities (alias)\n\n"
          "Returns:\n"
          "    Number of multibodies in the world")
      .def(
          "has_multi_body",
          &dart8::World::hasMultiBody,
          nb::arg("name"),
          "Check if a MultiBody with the given name exists\n\n"
          "Args:\n"
          "    name: Name to check\n\n"
          "Returns:\n"
          "    True if exists")
      .def(
          "get_multi_body",
          [](const dart8::World& self, const std::string& name) -> nb::object {
            auto result = self.getMultiBody(name);
            if (result.has_value()) {
              return nb::cast(result.value());
            }
            return nb::none();
          },
          nb::arg("name"),
          "Get a MultiBody by name\n\n"
          "Args:\n"
          "    name: Name of the multibody\n\n"
          "Returns:\n"
          "    MultiBody handle if found, None otherwise")
      .def(
          "has_rigid_body",
          &dart8::World::hasRigidBody,
          nb::arg("name"),
          "Check if a RigidBody with the given name exists\n\n"
          "Args:\n"
          "    name: Name to check\n\n"
          "Returns:\n"
          "    True if exists")
      .def(
          "get_rigid_body",
          [](const dart8::World& self, const std::string& name) -> nb::object {
            auto result = self.getRigidBody(name);
            if (result.has_value()) {
              return nb::cast(result.value());
            }
            return nb::none();
          },
          nb::arg("name"),
          "Get a RigidBody by name\n\n"
          "Args:\n"
          "    name: Name of the rigid body\n\n"
          "Returns:\n"
          "    RigidBody handle if found, None otherwise")
      // Mode control
      .def(
          "enter_simulation_mode",
          &dart8::World::enterSimulationMode,
          "Switch to simulation mode (locks design-time operations)")
      .def(
          "is_simulation_mode",
          &dart8::World::isSimulationMode,
          "Check if in simulation mode\n\n"
          "Returns:\n"
          "    True if in simulation mode")
      // Frame management
      .def(
          "get_frame",
          &dart8::World::getFrame,
          "Get the world frame\n\n"
          "Returns:\n"
          "    World frame handle")
      // FreeFrame management
      .def(
          "add_free_frame",
          [](dart8::World& self, const std::string& name) -> dart8::FreeFrame {
            return self.addFreeFrame(name);
          },
          nb::arg("name") = "",
          "Create a FreeFrame\n\n"
          "Args:\n"
          "    name: Name of the frame (empty = auto-generate)\n\n"
          "Returns:\n"
          "    FreeFrame handle to the created frame")
      .def(
          "add_free_frame",
          [](dart8::World& self,
             const std::string& name,
             const dart8::Frame& parent) -> dart8::FreeFrame {
            return self.addFreeFrame(name, parent);
          },
          nb::arg("name"),
          nb::arg("parent"),
          "Create a FreeFrame with a parent\n\n"
          "Args:\n"
          "    name: Name of the frame (empty = auto-generate)\n"
          "    parent: Parent frame\n\n"
          "Returns:\n"
          "    FreeFrame handle to the created frame")
      // FixedFrame management
      .def(
          "add_fixed_frame",
          [](dart8::World& self,
             const std::string& name,
             const dart8::Frame& parent) -> dart8::FixedFrame {
            return self.addFixedFrame(name, parent);
          },
          nb::arg("name"),
          nb::arg("parent"),
          "Create a FixedFrame rigidly attached to parent frame\n\n"
          "Note: FixedFrame cannot be attached to world frame.\n"
          "      Use add_free_frame() for frames in world.\n\n"
          "Args:\n"
          "    name: Name of the frame (required, non-empty)\n"
          "    parent: Parent frame (cannot be world frame)\n\n"
          "Returns:\n"
          "    FixedFrame handle to the created frame")
      .def(
          "add_fixed_frame",
          [](dart8::World& self,
             const std::string& name,
             const dart8::Frame& parent,
             const Eigen::Matrix4d& offset_matrix) -> dart8::FixedFrame {
            // Convert 4x4 matrix to Isometry3d
            Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
            offset.matrix() = offset_matrix;
            return self.addFixedFrame(name, parent, offset);
          },
          nb::arg("name"),
          nb::arg("parent"),
          nb::arg("offset"),
          "Create a FixedFrame with an offset\n\n"
          "Args:\n"
          "    name: Name of the frame (required)\n"
          "    parent: Parent frame\n"
          "    offset: Fixed transformation offset as 4x4 matrix\n\n"
          "Returns:\n"
          "    FixedFrame handle to the created frame")
      .def(
          "save_binary",
          [](const dart8::World& self, nb::object file_or_path) {
            // Check if it's a string (file path)
            if (nb::isinstance<nb::str>(file_or_path)) {
              std::string filename = nb::cast<std::string>(file_or_path);
              std::ofstream file(filename, std::ios::binary);
              if (!file) {
                throw std::runtime_error(
                    "Failed to open file for writing: " + filename);
              }
              self.saveBinary(file);
            } else {
              // Assume it's a BytesIO-like object with write() method
              std::ostringstream oss(std::ios::binary);
              self.saveBinary(oss);
              std::string data = oss.str();
              nb::object write_method = file_or_path.attr("write");
              write_method(nb::bytes(data.data(), data.size()));
            }
          },
          nb::arg("file"),
          "Save world state to binary file or BytesIO\n\n"
          "Args:\n"
          "    file: File path (str) or file-like object (BytesIO)")
      .def(
          "load_binary",
          [](dart8::World& self, nb::object file_or_path) {
            // Check if it's a string (file path)
            if (nb::isinstance<nb::str>(file_or_path)) {
              std::string filename = nb::cast<std::string>(file_or_path);
              std::ifstream file(filename, std::ios::binary);
              if (!file) {
                throw std::runtime_error(
                    "Failed to open file for reading: " + filename);
              }
              self.loadBinary(file);
            } else {
              // Assume it's a BytesIO-like object with read() method
              // Reset position to beginning before reading
              try {
                nb::object seek_method = file_or_path.attr("seek");
                seek_method(0);
              } catch (...) {
                // Ignore if seek doesn't exist
              }
              nb::object read_method = file_or_path.attr("read");
              nb::bytes data = nb::cast<nb::bytes>(read_method());
              std::string str_data(data.c_str(), data.size());
              std::istringstream iss(str_data, std::ios::binary);
              self.loadBinary(iss);
            }
          },
          nb::arg("file"),
          "Load world state from binary file or BytesIO\n\n"
          "Args:\n"
          "    file: File path (str) or file-like object (BytesIO)")
      .def("__repr__", [](const dart8::World& self) {
        std::size_t numMultiBodies = self.getMultiBodyCount();
        std::size_t numRigidBodies = self.getRigidBodyCount();
        return "<dart8.World: " + std::to_string(numMultiBodies) + " multibody"
               + (numMultiBodies == 1 ? "" : "s") + ", "
               + std::to_string(numRigidBodies) + " rigidbody"
               + (numRigidBodies == 1 ? "" : "s") + ">";
      });
}

} // namespace dartpy8
