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
#include <dart/simulation/experimental/common/exceptions.hpp>
#include <dart/simulation/experimental/frame/fixed_frame.hpp>
#include <dart/simulation/experimental/frame/frame.hpp>
#include <dart/simulation/experimental/frame/free_frame.hpp>
#include <dart/simulation/experimental/multi_body/joint.hpp>
#include <dart/simulation/experimental/multi_body/link.hpp>
#include <dart/simulation/experimental/multi_body/multi_body.hpp>
#include <dart/simulation/experimental/world.hpp>

#include <Eigen/Cholesky>
#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/array.h>
#include <nanobind/stl/string.h>

#include <array>
#include <string>
#include <utility>
#include <vector>

#include <cmath>

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

void validateIsometryMatrix(const Eigen::Matrix4d& matrix)
{
  constexpr double tolerance = 1e-9;

  DART_EXPERIMENTAL_THROW_T_IF(
      !matrix.allFinite(),
      sim::InvalidArgumentException,
      "Transform matrix must contain only finite values");

  Eigen::RowVector4d expectedBottom;
  expectedBottom << 0.0, 0.0, 0.0, 1.0;
  DART_EXPERIMENTAL_THROW_T_IF(
      (matrix.bottomRows<1>() - expectedBottom).cwiseAbs().maxCoeff()
          > tolerance,
      sim::InvalidArgumentException,
      "Transform matrix bottom row must be [0, 0, 0, 1]");

  const Eigen::Matrix3d rotation = matrix.topLeftCorner<3, 3>();
  const double orthonormalError
      = (rotation * rotation.transpose() - Eigen::Matrix3d::Identity())
            .cwiseAbs()
            .maxCoeff();
  DART_EXPERIMENTAL_THROW_T_IF(
      orthonormalError > tolerance
          || std::abs(rotation.determinant() - 1.0) > tolerance,
      sim::InvalidArgumentException,
      "Transform matrix rotation must be orthonormal");
}

Eigen::Matrix4d toMatrix4(const nb::handle& value)
{
  if (nb::isinstance<nb::ndarray<double>>(value)) {
    const auto array = nb::cast<nb::ndarray<double>>(value);
    if (array.ndim() != 2 || array.shape(0) != 4 || array.shape(1) != 4) {
      throw nb::type_error("Expected a 4x4 matrix");
    }

    auto stride = [&](size_t axis) -> int64_t {
      if (array.stride_ptr()) {
        return array.stride(axis);
      }
      int64_t strideValue = 1;
      for (size_t i = array.ndim(); i-- > axis + 1;) {
        strideValue *= array.shape(i);
      }
      return strideValue;
    };

    Eigen::Matrix4d matrix;
    const double* base = array.data();
    const int64_t s0 = stride(0);
    const int64_t s1 = stride(1);
    for (Eigen::Index r = 0; r < 4; ++r) {
      for (Eigen::Index c = 0; c < 4; ++c) {
        matrix(r, c) = base[r * s0 + c * s1];
      }
    }
    return matrix;
  }

  const auto nested = nb::cast<std::array<std::array<double, 4>, 4>>(value);
  Eigen::Matrix4d matrix;
  for (std::size_t r = 0; r < nested.size(); ++r) {
    for (std::size_t c = 0; c < nested[r].size(); ++c) {
      matrix(static_cast<Eigen::Index>(r), static_cast<Eigen::Index>(c))
          = nested[r][c];
    }
  }
  return matrix;
}

Eigen::Isometry3d toIsometry(const nb::handle& value)
{
  Eigen::Matrix4d matrix;
  if (nb::hasattr(value, "matrix")) {
    matrix = toMatrix4(value.attr("matrix")());
  } else {
    matrix = toMatrix4(value);
  }

  validateIsometryMatrix(matrix);

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.matrix() = matrix;
  return transform;
}

sim::Frame toFrameHandle(const nb::handle& value)
{
  if (nb::isinstance<sim::FreeFrame>(value)) {
    auto frame = nb::cast<sim::FreeFrame>(value);
    return sim::Frame(frame.getEntity(), frame.getWorld());
  }
  if (nb::isinstance<sim::FixedFrame>(value)) {
    auto frame = nb::cast<sim::FixedFrame>(value);
    return sim::Frame(frame.getEntity(), frame.getWorld());
  }
  if (nb::isinstance<sim::Link>(value)) {
    auto frame = nb::cast<sim::Link>(value);
    return sim::Frame(frame.getEntity(), frame.getWorld());
  }
  if (nb::isinstance<sim::RigidBody>(value)) {
    auto frame = nb::cast<sim::RigidBody>(value);
    return sim::Frame(frame.getEntity(), frame.getWorld());
  }

  return nb::cast<sim::Frame>(value);
}

bool isSymmetricPositiveDefinite(const Eigen::Matrix3d& matrix)
{
  if (!matrix.allFinite() || !matrix.isApprox(matrix.transpose(), 1e-12)) {
    return false;
  }

  Eigen::LLT<Eigen::Matrix3d> factorization(matrix);
  return factorization.info() == Eigen::Success;
}

void validateMass(double mass)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !std::isfinite(mass) || mass <= 0.0,
      sim::InvalidArgumentException,
      "RigidBodyOptions.mass must be positive and finite");
}

void validateFiniteVector(const Eigen::Vector3d& value, const char* fieldName)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !value.allFinite(),
      sim::InvalidArgumentException,
      "RigidBodyOptions.{} must contain only finite values",
      fieldName);
}

void validateOrientation(const Eigen::Quaterniond& orientation)
{
  const auto orientationNorm = orientation.norm();
  DART_EXPERIMENTAL_THROW_T_IF(
      !orientation.coeffs().allFinite() || !std::isfinite(orientationNorm)
          || orientationNorm <= 0.0,
      sim::InvalidArgumentException,
      "RigidBodyOptions.orientation must be finite and non-zero");
}

void validateInertia(const Eigen::Matrix3d& inertia)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isSymmetricPositiveDefinite(inertia),
      sim::InvalidArgumentException,
      "RigidBodyOptions.inertia must be symmetric positive definite");
}

void validateRigidBodyOptions(const sim::RigidBodyOptions& options)
{
  validateMass(options.mass);
  validateInertia(options.inertia);
  validateFiniteVector(options.position, "position");
  validateFiniteVector(options.linearVelocity, "linearVelocity");
  validateFiniteVector(options.angularVelocity, "angularVelocity");
  validateOrientation(options.orientation);
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

  validateRigidBodyOptions(options);
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

  validateRigidBodyOptions(options);
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

  auto frameClass = nb::class_<sim::Frame>(m, "Frame");
  auto freeFrameClass = nb::class_<sim::FreeFrame, sim::Frame>(m, "FreeFrame");
  auto fixedFrameClass
      = nb::class_<sim::FixedFrame, sim::Frame>(m, "FixedFrame");
  auto jointClass = nb::class_<sim::Joint>(m, "Joint");
  auto linkClass = nb::class_<sim::Link, sim::Frame>(m, "Link");

  frameClass.def_static("world", &sim::Frame::world)
      .def(
          "getName",
          [](const sim::Frame& self) { return std::string(self.getName()); })
      .def(
          "get_name",
          [](const sim::Frame& self) { return std::string(self.getName()); })
      .def("getParentFrame", &sim::Frame::getParentFrame)
      .def("get_parent_frame", &sim::Frame::getParentFrame)
      .def(
          "setParentFrame",
          [](sim::Frame& self, const nb::handle& parent) {
            self.setParentFrame(toFrameHandle(parent));
          },
          nb::arg("parent"))
      .def(
          "set_parent_frame",
          [](sim::Frame& self, const nb::handle& parent) {
            self.setParentFrame(toFrameHandle(parent));
          },
          nb::arg("parent"))
      .def(
          "getLocalTransform",
          [](const sim::Frame& self) {
            return self.getLocalTransform().matrix();
          })
      .def(
          "get_local_transform",
          [](const sim::Frame& self) {
            return self.getLocalTransform().matrix();
          })
      .def(
          "getTransform",
          [](const sim::Frame& self) { return self.getTransformMatrix(); })
      .def(
          "get_transform",
          [](const sim::Frame& self) { return self.getTransformMatrix(); })
      .def(
          "get_transform",
          [](const sim::Frame& self, const nb::handle& relativeTo) {
            return self.getTransform(toFrameHandle(relativeTo)).matrix();
          },
          nb::arg("relative_to"))
      .def(
          "get_transform",
          [](const sim::Frame& self,
             const nb::handle& to,
             const nb::handle& expressedIn) {
            return self
                .getTransform(toFrameHandle(to), toFrameHandle(expressedIn))
                .matrix();
          },
          nb::arg("to"),
          nb::arg("expressed_in"))
      .def("isValid", &sim::Frame::isValid)
      .def("is_valid_handle", &sim::Frame::isValid)
      .def("isWorld", &sim::Frame::isWorld)
      .def(
          "isSameInstanceAs",
          [](const sim::Frame& self, const nb::handle& other) {
            return self.isSameInstanceAs(toFrameHandle(other));
          },
          nb::arg("other"))
      .def(
          "is_same_instance_as",
          [](const sim::Frame& self, const nb::handle& other) {
            return self.isSameInstanceAs(toFrameHandle(other));
          },
          nb::arg("other"))
      .def_prop_ro(
          "name",
          [](const sim::Frame& self) { return std::string(self.getName()); })
      .def_prop_rw(
          "parent_frame",
          &sim::Frame::getParentFrame,
          [](sim::Frame& self, const nb::handle& parent) {
            self.setParentFrame(toFrameHandle(parent));
          })
      .def_prop_ro(
          "local_transform",
          [](const sim::Frame& self) {
            return self.getLocalTransform().matrix();
          })
      .def_prop_ro("translation", &sim::Frame::getTranslation)
      .def_prop_ro("rotation", &sim::Frame::getRotation)
      .def_prop_ro(
          "quaternion",
          [](const sim::Frame& self) { return toWxyz(self.getQuaternion()); })
      .def_prop_ro("transform", &sim::Frame::getTransformMatrix)
      .def_prop_ro("is_valid", &sim::Frame::isValid)
      .def_prop_ro("is_world", &sim::Frame::isWorld)
      .def(
          "__eq__",
          [](const sim::Frame& self, const nb::handle& other) {
            try {
              return self == toFrameHandle(other);
            } catch (const nb::cast_error&) {
              return false;
            }
          },
          nb::is_operator())
      .def(
          "__ne__",
          [](const sim::Frame& self, const nb::handle& other) {
            try {
              return self != toFrameHandle(other);
            } catch (const nb::cast_error&) {
              return true;
            }
          },
          nb::is_operator())
      .def("__repr__", [](const sim::Frame& self) {
        std::vector<std::pair<std::string, std::string>> fields;
        fields.emplace_back("valid", self.isValid() ? "True" : "False");
        if (self.isValid()) {
          fields.emplace_back("name", repr_string(std::string(self.getName())));
        }
        return format_repr("Frame", fields);
      });

  freeFrameClass
      .def(
          "setLocalTransform",
          [](sim::FreeFrame& self, const nb::handle& transform) {
            self.setLocalTransform(toIsometry(transform));
          },
          nb::arg("transform"))
      .def(
          "set_local_transform",
          [](sim::FreeFrame& self, const nb::handle& transform) {
            self.setLocalTransform(toIsometry(transform));
          },
          nb::arg("transform"))
      .def(
          "getLocalTransform",
          [](const sim::FreeFrame& self) {
            return self.getLocalTransform().matrix();
          })
      .def(
          "get_local_transform",
          [](const sim::FreeFrame& self) {
            return self.getLocalTransform().matrix();
          })
      .def_prop_rw(
          "local_transform",
          [](const sim::FreeFrame& self) {
            return self.getLocalTransform().matrix();
          },
          [](sim::FreeFrame& self, const nb::handle& transform) {
            self.setLocalTransform(toIsometry(transform));
          })
      .def("__repr__", [](const sim::FreeFrame& self) {
        std::vector<std::pair<std::string, std::string>> fields;
        fields.emplace_back("valid", self.isValid() ? "True" : "False");
        if (self.isValid()) {
          fields.emplace_back("name", repr_string(std::string(self.getName())));
        }
        return format_repr("FreeFrame", fields);
      });

  fixedFrameClass
      .def(
          "setLocalTransform",
          [](sim::FixedFrame& self, const nb::handle& transform) {
            self.setLocalTransform(toIsometry(transform));
          },
          nb::arg("transform"))
      .def(
          "set_local_transform",
          [](sim::FixedFrame& self, const nb::handle& transform) {
            self.setLocalTransform(toIsometry(transform));
          },
          nb::arg("transform"))
      .def(
          "getLocalTransform",
          [](const sim::FixedFrame& self) {
            return self.getLocalTransform().matrix();
          })
      .def(
          "get_local_transform",
          [](const sim::FixedFrame& self) {
            return self.getLocalTransform().matrix();
          })
      .def_prop_rw(
          "local_transform",
          [](const sim::FixedFrame& self) {
            return self.getLocalTransform().matrix();
          },
          [](sim::FixedFrame& self, const nb::handle& transform) {
            self.setLocalTransform(toIsometry(transform));
          })
      .def("__repr__", [](const sim::FixedFrame& self) {
        std::vector<std::pair<std::string, std::string>> fields;
        fields.emplace_back("valid", self.isValid() ? "True" : "False");
        if (self.isValid()) {
          fields.emplace_back("name", repr_string(std::string(self.getName())));
        }
        return format_repr("FixedFrame", fields);
      });

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

  nb::class_<sim::RigidBody, sim::Frame>(m, "RigidBody")
      .def("getName", [](const sim::RigidBody& self) { return self.getName(); })
      .def(
          "setTransform",
          [](sim::RigidBody& self, const nb::handle& transform) {
            self.setTransform(toIsometry(transform));
          },
          nb::arg("transform"))
      .def(
          "set_transform",
          [](sim::RigidBody& self, const nb::handle& transform) {
            self.setTransform(toIsometry(transform));
          },
          nb::arg("transform"))
      .def_prop_ro(
          "name", [](const sim::RigidBody& self) { return self.getName(); })
      .def_prop_ro("translation", &sim::RigidBody::getTranslation)
      .def_prop_ro("rotation", &sim::RigidBody::getRotation)
      .def_prop_ro(
          "quaternion",
          [](const sim::RigidBody& self) {
            return toWxyz(self.getQuaternion());
          })
      .def_prop_rw(
          "transform",
          &sim::RigidBody::getTransformMatrix,
          [](sim::RigidBody& self, const nb::handle& transform) {
            self.setTransform(toIsometry(transform));
          })
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
      .def_prop_rw(
          "mass",
          [](const sim::RigidBodyOptions& self) { return self.mass; },
          [](sim::RigidBodyOptions& self, double mass) {
            validateMass(mass);
            self.mass = mass;
          })
      .def_prop_rw(
          "position",
          [](const sim::RigidBodyOptions& self) { return self.position; },
          [](sim::RigidBodyOptions& self, const nb::handle& position) {
            const auto value = toVector3(position);
            validateFiniteVector(value, "position");
            self.position = value;
          })
      .def_prop_rw(
          "orientation",
          [](const sim::RigidBodyOptions& self) {
            return toWxyz(self.orientation);
          },
          [](sim::RigidBodyOptions& self, const nb::handle& orientation) {
            const auto value = toQuaternionWxyz(orientation);
            validateOrientation(value);
            self.orientation = value;
          })
      .def_prop_rw(
          "linear_velocity",
          [](const sim::RigidBodyOptions& self) { return self.linearVelocity; },
          [](sim::RigidBodyOptions& self, const nb::handle& linearVelocity) {
            const auto value = toVector3(linearVelocity);
            validateFiniteVector(value, "linearVelocity");
            self.linearVelocity = value;
          })
      .def_prop_rw(
          "angular_velocity",
          [](const sim::RigidBodyOptions& self) {
            return self.angularVelocity;
          },
          [](sim::RigidBodyOptions& self, const nb::handle& angularVelocity) {
            const auto value = toVector3(angularVelocity);
            validateFiniteVector(value, "angularVelocity");
            self.angularVelocity = value;
          })
      .def_prop_rw(
          "inertia",
          [](const sim::RigidBodyOptions& self) { return self.inertia; },
          [](sim::RigidBodyOptions& self, const nb::handle& inertia) {
            const auto value = toMatrix3(inertia);
            validateInertia(value);
            self.inertia = value;
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
          "addFreeFrame",
          [](sim::World& self, const std::string& name) {
            return self.addFreeFrame(name);
          },
          nb::arg("name") = "",
          nb::keep_alive<0, 1>())
      .def(
          "addFreeFrame",
          [](sim::World& self,
             const std::string& name,
             const nb::handle& parent) {
            return self.addFreeFrame(name, toFrameHandle(parent));
          },
          nb::arg("name"),
          nb::arg("parent"),
          nb::keep_alive<0, 1>())
      .def(
          "add_free_frame",
          [](sim::World& self,
             const std::string& name,
             const nb::handle& parent) {
            if (parent.is_none()) {
              return self.addFreeFrame(name);
            }
            return self.addFreeFrame(name, toFrameHandle(parent));
          },
          nb::arg("name") = "",
          nb::kw_only(),
          nb::arg("parent") = nb::none(),
          nb::keep_alive<0, 1>())
      .def(
          "addFixedFrame",
          [](sim::World& self,
             const std::string& name,
             const nb::handle& parent,
             const nb::handle& offset) {
            if (offset.is_none()) {
              return self.addFixedFrame(name, toFrameHandle(parent));
            }
            return self.addFixedFrame(
                name, toFrameHandle(parent), toIsometry(offset));
          },
          nb::arg("name"),
          nb::arg("parent"),
          nb::arg("offset") = nb::none(),
          nb::keep_alive<0, 1>())
      .def(
          "add_fixed_frame",
          [](sim::World& self,
             const std::string& name,
             const nb::handle& parent,
             const nb::handle& offset) {
            if (offset.is_none()) {
              return self.addFixedFrame(name, toFrameHandle(parent));
            }
            return self.addFixedFrame(
                name, toFrameHandle(parent), toIsometry(offset));
          },
          nb::arg("name"),
          nb::arg("parent"),
          nb::arg("offset") = nb::none(),
          nb::keep_alive<0, 1>())
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
          "getRigidBody",
          [](sim::World& self, const std::string& name) -> nb::object {
            auto body = self.getRigidBody(name);
            if (!body.has_value()) {
              return nb::none();
            }
            return nb::cast(*body, nb::rv_policy::move);
          },
          nb::arg("name"),
          nb::keep_alive<0, 1>())
      .def(
          "has_rigid_body",
          [](const sim::World& self, const std::string& name) {
            return self.hasRigidBody(name);
          },
          nb::arg("name"))
      .def(
          "get_rigid_body",
          [](sim::World& self, const std::string& name) -> nb::object {
            auto body = self.getRigidBody(name);
            if (!body.has_value()) {
              return nb::none();
            }
            return nb::cast(*body, nb::rv_policy::move);
          },
          nb::arg("name"),
          nb::keep_alive<0, 1>())
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
