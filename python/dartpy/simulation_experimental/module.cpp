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

#include <dart/simulation/experimental/body/collision_shape.hpp>
#include <dart/simulation/experimental/body/contact.hpp>
#include <dart/simulation/experimental/body/rigid_body.hpp>
#include <dart/simulation/experimental/body/rigid_body_options.hpp>
#include <dart/simulation/experimental/common/exceptions.hpp>
#include <dart/simulation/experimental/constraint/loop_closure.hpp>
#include <dart/simulation/experimental/constraint/loop_closure_spec.hpp>
#include <dart/simulation/experimental/frame/fixed_frame.hpp>
#include <dart/simulation/experimental/frame/frame.hpp>
#include <dart/simulation/experimental/frame/free_frame.hpp>
#include <dart/simulation/experimental/multibody/joint.hpp>
#include <dart/simulation/experimental/multibody/link.hpp>
#include <dart/simulation/experimental/multibody/multibody.hpp>
#include <dart/simulation/experimental/space/state_space.hpp>
#include <dart/simulation/experimental/world.hpp>

#include <Eigen/Cholesky>
#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/array.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <array>
#include <limits>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include <cmath>
#include <cstddef>

namespace nb = nanobind;

namespace dart::python_nb {

namespace {

namespace sim = dart::simulation::experimental;

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

Eigen::VectorXd toVectorX(const nb::handle& value)
{
  if (nb::isinstance<nb::ndarray<double>>(value)) {
    const auto array = nb::cast<nb::ndarray<double>>(value);
    if (array.ndim() != 1) {
      throw nb::type_error("Expected a 1-D vector");
    }

    const auto length = static_cast<Eigen::Index>(array.shape(0));
    const int64_t stride = array.stride_ptr() ? array.stride(0) : 1;
    const double* base = array.data();

    Eigen::VectorXd vector(length);
    for (Eigen::Index i = 0; i < length; ++i) {
      vector[i] = base[i * stride];
    }
    return vector;
  }

  if (nb::isinstance<nb::sequence>(value)) {
    const auto sequence = nb::cast<nb::sequence>(value);
    Eigen::VectorXd vector(static_cast<Eigen::Index>(nb::len(sequence)));
    for (Eigen::Index i = 0; i < vector.size(); ++i) {
      vector[i] = nb::cast<double>(sequence[static_cast<nb::ssize_t>(i)]);
    }
    return vector;
  }

  return nb::cast<Eigen::VectorXd>(value);
}

Eigen::VectorXd toVectorX(const std::vector<double>& values)
{
  Eigen::VectorXd vector(static_cast<Eigen::Index>(values.size()));
  for (Eigen::Index i = 0; i < vector.size(); ++i) {
    vector[i] = values[static_cast<std::size_t>(i)];
  }
  return vector;
}

std::string toOptionalName(const nb::handle& value)
{
  if (value.is_none()) {
    return "";
  }
  return nb::cast<std::string>(value);
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

void validateJointSpecAxis(
    const Eigen::Vector3d& axis, const char* field = "axis")
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !axis.allFinite(),
      sim::InvalidArgumentException,
      "JointSpec.{} must contain only finite values",
      field);

  DART_EXPERIMENTAL_THROW_T_IF(
      axis.norm() <= 1e-9,
      sim::InvalidArgumentException,
      "JointSpec.{} must be non-zero",
      field);
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
  validateFiniteVector(options.linearVelocity, "linear_velocity");
  validateFiniteVector(options.angularVelocity, "angular_velocity");
  validateOrientation(options.orientation);
}

sim::LoopClosureSpec makeLoopClosureSpec(
    const nb::handle& frameA,
    const nb::handle& frameB,
    sim::LoopClosureFamily family,
    const nb::handle& offsetA,
    const nb::handle& offsetB)
{
  sim::LoopClosureSpec spec;
  spec.frameA = toFrameHandle(frameA);
  spec.frameB = toFrameHandle(frameB);
  spec.family = family;

  if (!offsetA.is_none()) {
    spec.offsetA = toIsometry(offsetA);
  }
  if (!offsetB.is_none()) {
    spec.offsetB = toIsometry(offsetB);
  }

  return spec;
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
      .value("SPHERICAL", sim::JointType::Spherical)
      .value("PLANAR", sim::JointType::Planar)
      .value("FLOATING", sim::JointType::Floating)
      .value("CUSTOM", sim::JointType::Custom);

  nb::enum_<sim::ActuatorType>(m, "ActuatorType")
      .value("FORCE", sim::ActuatorType::Force)
      .value("PASSIVE", sim::ActuatorType::Passive)
      .value("SERVO", sim::ActuatorType::Servo)
      .value("VELOCITY", sim::ActuatorType::Velocity)
      .value("ACCELERATION", sim::ActuatorType::Acceleration)
      .value("LOCKED", sim::ActuatorType::Locked)
      .value("MIMIC", sim::ActuatorType::Mimic);

  nb::enum_<sim::LoopClosureFamily>(m, "LoopClosureFamily")
      .value("RIGID", sim::LoopClosureFamily::Rigid)
      .value("POINT", sim::LoopClosureFamily::Point)
      .value("DISTANCE", sim::LoopClosureFamily::Distance);

  nb::enum_<sim::LoopClosureResidualCoordinates>(
      m, "LoopClosureResidualCoordinates")
      .value("WORLD", sim::LoopClosureResidualCoordinates::World);

  nb::enum_<sim::WorldSyncStage>(m, "WorldSyncStage")
      .value("KINEMATICS", sim::WorldSyncStage::Kinematics);

  nb::enum_<sim::RigidBodySolver>(m, "RigidBodySolver")
      .value("SEQUENTIAL_IMPULSE", sim::RigidBodySolver::SequentialImpulse)
      .value("IPC", sim::RigidBodySolver::Ipc);

  nb::enum_<sim::CollisionShapeType>(m, "CollisionShapeType")
      .value("SPHERE", sim::CollisionShapeType::Sphere)
      .value("BOX", sim::CollisionShapeType::Box);

  nb::class_<sim::CollisionShape>(m, "CollisionShape")
      .def_static("sphere", &sim::CollisionShape::makeSphere, nb::arg("radius"))
      .def_static(
          "box",
          [](const nb::handle& halfExtents) {
            return sim::CollisionShape::makeBox(toVector3(halfExtents));
          },
          nb::arg("half_extents"))
      .def_prop_ro(
          "type", [](const sim::CollisionShape& self) { return self.type; })
      .def_prop_ro(
          "radius", [](const sim::CollisionShape& self) { return self.radius; })
      .def_prop_ro("half_extents", [](const sim::CollisionShape& self) {
        return self.halfExtents;
      });

  nb::enum_<sim::ClosureKinematicsPolicy>(m, "ClosureKinematicsPolicy")
      .value("RESIDUAL_ONLY", sim::ClosureKinematicsPolicy::ResidualOnly)
      .value("PROJECT", sim::ClosureKinematicsPolicy::Project);

  nb::enum_<sim::ClosureDynamicsPolicy>(m, "ClosureDynamicsPolicy")
      .value("RESIDUAL_ONLY", sim::ClosureDynamicsPolicy::ResidualOnly)
      .value("SOLVE", sim::ClosureDynamicsPolicy::Solve);

  nb::class_<sim::StateSpace::Variable>(m, "StateVariable")
      .def_prop_ro(
          "name",
          [](const sim::StateSpace::Variable& self) { return self.name; })
      .def_prop_ro(
          "start_index",
          [](const sim::StateSpace::Variable& self) { return self.startIndex; })
      .def_prop_ro(
          "dimension",
          [](const sim::StateSpace::Variable& self) { return self.dimension; })
      .def_prop_ro(
          "lower_bound",
          [](const sim::StateSpace::Variable& self) { return self.lowerBound; })
      .def_prop_ro(
          "upper_bound",
          [](const sim::StateSpace::Variable& self) { return self.upperBound; })
      .def("__repr__", [](const sim::StateSpace::Variable& self) {
        std::vector<std::pair<std::string, std::string>> fields;
        fields.emplace_back("name", repr_string(self.name));
        fields.emplace_back("start_index", std::to_string(self.startIndex));
        fields.emplace_back("dimension", std::to_string(self.dimension));
        fields.emplace_back("lower_bound", repr_double(self.lowerBound));
        fields.emplace_back("upper_bound", repr_double(self.upperBound));
        return format_repr("StateVariable", fields);
      });

  nb::class_<sim::StateSpace>(m, "StateSpace")
      .def(nb::init<>())
      .def(
          "add_variable",
          [](sim::StateSpace& self,
             const std::string& name,
             std::size_t dimension,
             double lower,
             double upper) -> sim::StateSpace& {
            return self.addVariable(name, dimension, lower, upper);
          },
          nb::arg("name"),
          nb::arg("dimension"),
          nb::kw_only(),
          nb::arg("lower") = -std::numeric_limits<double>::infinity(),
          nb::arg("upper") = std::numeric_limits<double>::infinity(),
          nb::rv_policy::reference_internal)
      .def(
          "add_variables",
          [](sim::StateSpace& self,
             const std::vector<std::string>& names,
             double lower,
             double upper) -> sim::StateSpace& {
            return self.addVariables(
                std::span<const std::string>(names.data(), names.size()),
                lower,
                upper);
          },
          nb::arg("names"),
          nb::kw_only(),
          nb::arg("lower") = -std::numeric_limits<double>::infinity(),
          nb::arg("upper") = std::numeric_limits<double>::infinity(),
          nb::rv_policy::reference_internal)
      .def("finalize", &sim::StateSpace::finalize)
      .def("has_variable", &sim::StateSpace::hasVariable, nb::arg("name"))
      .def(
          "get_variable",
          [](const sim::StateSpace& self,
             const std::string& name) -> nb::object {
            auto variable = self.getVariable(name);
            if (!variable.has_value()) {
              return nb::none();
            }
            return nb::cast(*variable, nb::rv_policy::move);
          },
          nb::arg("name"))
      .def(
          "get_variable_index",
          [](const sim::StateSpace& self,
             const std::string& name) -> nb::object {
            auto index = self.getVariableIndex(name);
            if (!index.has_value()) {
              return nb::none();
            }
            return nb::cast(*index);
          },
          nb::arg("name"))
      .def_prop_ro("dimension", &sim::StateSpace::getDimension)
      .def_prop_ro("num_variables", &sim::StateSpace::getNumVariables)
      .def_prop_ro("is_finalized", &sim::StateSpace::isFinalized)
      .def_prop_ro("variables", &sim::StateSpace::getVariables)
      .def_prop_ro("variable_names", &sim::StateSpace::getVariableNames)
      .def_prop_ro(
          "lower_bounds",
          [](const sim::StateSpace& self) {
            return toVectorX(self.getLowerBounds());
          })
      .def_prop_ro(
          "upper_bounds",
          [](const sim::StateSpace& self) {
            return toVectorX(self.getUpperBounds());
          })
      .def("__repr__", [](const sim::StateSpace& self) {
        std::vector<std::pair<std::string, std::string>> fields;
        fields.emplace_back("dimension", std::to_string(self.getDimension()));
        fields.emplace_back(
            "variables", std::to_string(self.getNumVariables()));
        fields.emplace_back("finalized", self.isFinalized() ? "True" : "False");
        return format_repr("StateSpace", fields);
      });

  nb::class_<sim::JointSpec>(m, "JointSpec")
      .def(
          nb::new_([](std::string name,
                      sim::JointType type,
                      const nb::handle& axis,
                      const nb::handle& axis2,
                      const nb::handle& transform_from_parent) {
            sim::JointSpec spec;
            spec.name = std::move(name);
            spec.type = type;
            if (!axis.is_none()) {
              spec.axis = toVector3(axis);
              validateJointSpecAxis(spec.axis);
            }
            if (!axis2.is_none()) {
              spec.axis2 = toVector3(axis2);
              validateJointSpecAxis(spec.axis2, "axis2");
            }
            if (!transform_from_parent.is_none()) {
              spec.transformFromParent = toIsometry(transform_from_parent);
            }
            return spec;
          }),
          nb::arg("name") = "",
          nb::arg("type") = sim::JointType::Revolute,
          nb::arg("axis") = nb::none(),
          nb::arg("axis2") = nb::none(),
          nb::arg("transform_from_parent") = nb::none())
      .def_rw("name", &sim::JointSpec::name)
      .def_rw("type", &sim::JointSpec::type)
      .def_prop_rw(
          "axis",
          [](const sim::JointSpec& self) { return self.axis; },
          [](sim::JointSpec& self, const nb::handle& axis) {
            auto value = toVector3(axis);
            validateJointSpecAxis(value);
            self.axis = value;
          })
      .def_prop_rw(
          "axis2",
          [](const sim::JointSpec& self) { return self.axis2; },
          [](sim::JointSpec& self, const nb::handle& axis2) {
            auto value = toVector3(axis2);
            validateJointSpecAxis(value, "axis2");
            self.axis2 = value;
          })
      .def_prop_rw(
          "transform_from_parent",
          [](const sim::JointSpec& self) {
            return self.transformFromParent.matrix();
          },
          [](sim::JointSpec& self, const nb::handle& transform) {
            self.transformFromParent = toIsometry(transform);
          })
      .def("__repr__", [](const sim::JointSpec& self) {
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
  auto loopClosureClass = nb::class_<sim::LoopClosure>(m, "LoopClosure");

  frameClass.def_static("world", &sim::Frame::world)
      .def(
          "relative_transform",
          [](const sim::Frame& self, const nb::handle& relativeTo) {
            return self.getTransform(toFrameHandle(relativeTo)).matrix();
          },
          nb::arg("relative_to"))
      .def(
          "relative_transform",
          [](const sim::Frame& self,
             const nb::handle& to,
             const nb::handle& expressedIn) {
            return self
                .getTransform(toFrameHandle(to), toFrameHandle(expressedIn))
                .matrix();
          },
          nb::arg("relative_to"),
          nb::arg("expressed_in"))
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
      .def_prop_ro(
          "name",
          [](const sim::Joint& self) { return std::string(self.getName()); })
      .def_prop_ro("type", &sim::Joint::getType)
      .def_prop_rw(
          "actuator_type",
          &sim::Joint::getActuatorType,
          &sim::Joint::setActuatorType)
      .def_prop_rw(
          "command_velocity",
          &sim::Joint::getCommandVelocity,
          [](sim::Joint& self, const nb::handle& value) {
            self.setCommandVelocity(toVectorX(value));
          })
      .def_prop_ro("axis", &sim::Joint::getAxis)
      .def_prop_ro("axis2", &sim::Joint::getAxis2)
      .def_prop_rw("pitch", &sim::Joint::getPitch, &sim::Joint::setPitch)
      .def_prop_ro("num_dofs", &sim::Joint::getDOFCount)
      .def_prop_rw(
          "position",
          &sim::Joint::getPosition,
          [](sim::Joint& self, const nb::handle& position) {
            self.setPosition(toVectorX(position));
          })
      .def_prop_rw(
          "velocity",
          &sim::Joint::getVelocity,
          [](sim::Joint& self, const nb::handle& velocity) {
            self.setVelocity(toVectorX(velocity));
          })
      .def_prop_rw(
          "force",
          &sim::Joint::getForce,
          [](sim::Joint& self, const nb::handle& force) {
            self.setForce(toVectorX(force));
          })
      .def_prop_ro("acceleration", &sim::Joint::getAcceleration)
      .def_prop_rw(
          "spring_stiffness",
          &sim::Joint::getSpringStiffness,
          [](sim::Joint& self, const nb::handle& value) {
            self.setSpringStiffness(toVectorX(value));
          })
      .def_prop_rw(
          "rest_position",
          &sim::Joint::getRestPosition,
          [](sim::Joint& self, const nb::handle& value) {
            self.setRestPosition(toVectorX(value));
          })
      .def_prop_rw(
          "damping_coefficient",
          &sim::Joint::getDampingCoefficient,
          [](sim::Joint& self, const nb::handle& value) {
            self.setDampingCoefficient(toVectorX(value));
          })
      .def_prop_rw(
          "armature",
          &sim::Joint::getArmature,
          [](sim::Joint& self, const nb::handle& value) {
            self.setArmature(toVectorX(value));
          })
      .def_prop_rw(
          "coulomb_friction",
          &sim::Joint::getCoulombFriction,
          [](sim::Joint& self, const nb::handle& value) {
            self.setCoulombFriction(toVectorX(value));
          })
      .def(
          "set_position_limits",
          [](sim::Joint& self,
             const nb::handle& lower,
             const nb::handle& upper) {
            self.setPositionLimits(toVectorX(lower), toVectorX(upper));
          },
          nb::arg("lower"),
          nb::arg("upper"))
      .def_prop_ro("position_lower_limits", &sim::Joint::getPositionLowerLimits)
      .def_prop_ro("position_upper_limits", &sim::Joint::getPositionUpperLimits)
      .def(
          "set_velocity_limits",
          [](sim::Joint& self,
             const nb::handle& lower,
             const nb::handle& upper) {
            self.setVelocityLimits(toVectorX(lower), toVectorX(upper));
          },
          nb::arg("lower"),
          nb::arg("upper"))
      .def_prop_ro("velocity_lower_limits", &sim::Joint::getVelocityLowerLimits)
      .def_prop_ro("velocity_upper_limits", &sim::Joint::getVelocityUpperLimits)
      .def(
          "set_effort_limits",
          [](sim::Joint& self,
             const nb::handle& lower,
             const nb::handle& upper) {
            self.setEffortLimits(toVectorX(lower), toVectorX(upper));
          },
          nb::arg("lower"),
          nb::arg("upper"))
      .def_prop_ro("effort_lower_limits", &sim::Joint::getEffortLowerLimits)
      .def_prop_ro("effort_upper_limits", &sim::Joint::getEffortUpperLimits)
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
      .def_prop_ro(
          "name",
          [](const sim::Link& self) { return std::string(self.getName()); })
      .def_prop_ro("parent_joint", &sim::Link::getParentJoint)
      .def_prop_rw("mass", &sim::Link::getMass, &sim::Link::setMass)
      .def_prop_rw(
          "inertia",
          &sim::Link::getInertia,
          [](sim::Link& self, const nb::handle& inertia) {
            self.setInertia(toMatrix3(inertia));
          })
      .def_prop_rw(
          "center_of_mass",
          &sim::Link::getCenterOfMass,
          [](sim::Link& self, const nb::handle& centerOfMass) {
            self.setCenterOfMass(toVector3(centerOfMass));
          })
      .def_prop_ro("translation", &sim::Link::getTranslation)
      .def_prop_ro("rotation", &sim::Link::getRotation)
      .def_prop_ro(
          "quaternion",
          [](const sim::Link& self) { return toWxyz(self.getQuaternion()); })
      .def_prop_ro("transform", &sim::Link::getTransformMatrix)
      .def(
          "set_collision_shape",
          &sim::Link::setCollisionShape,
          nb::arg("shape"))
      .def_prop_ro("collision_shape", &sim::Link::getCollisionShape)
      .def_prop_ro("has_collision_shape", &sim::Link::hasCollisionShape)
      .def_prop_ro("is_valid", &sim::Link::isValid)
      .def("__repr__", [](const sim::Link& self) {
        std::vector<std::pair<std::string, std::string>> fields;
        fields.emplace_back("valid", self.isValid() ? "True" : "False");
        if (self.isValid()) {
          fields.emplace_back("name", repr_string(std::string(self.getName())));
        }
        return format_repr("Link", fields);
      });

  nb::class_<sim::LoopClosureSpec>(m, "LoopClosureSpec")
      .def(
          nb::new_([](const nb::handle& frameA,
                      const nb::handle& frameB,
                      sim::LoopClosureFamily family,
                      const nb::handle& offsetA,
                      const nb::handle& offsetB) {
            return makeLoopClosureSpec(
                frameA, frameB, family, offsetA, offsetB);
          }),
          nb::arg("frame_a"),
          nb::arg("frame_b"),
          nb::arg("family") = sim::LoopClosureFamily::Rigid,
          nb::kw_only(),
          nb::arg("offset_a") = nb::none(),
          nb::arg("offset_b") = nb::none())
      .def_prop_rw(
          "frame_a",
          [](const sim::LoopClosureSpec& self) { return self.frameA; },
          [](sim::LoopClosureSpec& self, const nb::handle& frame) {
            self.frameA = toFrameHandle(frame);
          })
      .def_prop_rw(
          "frame_b",
          [](const sim::LoopClosureSpec& self) { return self.frameB; },
          [](sim::LoopClosureSpec& self, const nb::handle& frame) {
            self.frameB = toFrameHandle(frame);
          })
      .def_rw("family", &sim::LoopClosureSpec::family)
      .def_prop_rw(
          "offset_a",
          [](const sim::LoopClosureSpec& self) {
            return self.offsetA.matrix();
          },
          [](sim::LoopClosureSpec& self, const nb::handle& offset) {
            self.offsetA = toIsometry(offset);
          })
      .def_prop_rw(
          "offset_b",
          [](const sim::LoopClosureSpec& self) {
            return self.offsetB.matrix();
          },
          [](sim::LoopClosureSpec& self, const nb::handle& offset) {
            self.offsetB = toIsometry(offset);
          })
      .def("__repr__", [](const sim::LoopClosureSpec& self) {
        std::vector<std::pair<std::string, std::string>> fields;
        fields.emplace_back(
            "frame_a", nb::cast<std::string>(nb::repr(nb::cast(self.frameA))));
        fields.emplace_back(
            "frame_b", nb::cast<std::string>(nb::repr(nb::cast(self.frameB))));
        fields.emplace_back(
            "family", nb::cast<std::string>(nb::repr(nb::cast(self.family))));
        return format_repr("LoopClosureSpec", fields);
      });

  nb::class_<sim::LoopClosureRuntimePolicy>(m, "LoopClosureRuntimePolicy")
      .def(
          nb::new_([](bool enabled,
                      sim::ClosureKinematicsPolicy kinematics,
                      sim::ClosureDynamicsPolicy dynamics) {
            return sim::LoopClosureRuntimePolicy{
                .enabled = enabled,
                .kinematics = kinematics,
                .dynamics = dynamics};
          }),
          nb::arg("enabled") = true,
          nb::arg("kinematics") = sim::ClosureKinematicsPolicy::ResidualOnly,
          nb::arg("dynamics") = sim::ClosureDynamicsPolicy::ResidualOnly)
      .def_rw("enabled", &sim::LoopClosureRuntimePolicy::enabled)
      .def_rw("kinematics", &sim::LoopClosureRuntimePolicy::kinematics)
      .def_rw("dynamics", &sim::LoopClosureRuntimePolicy::dynamics)
      .def("__repr__", [](const sim::LoopClosureRuntimePolicy& self) {
        std::vector<std::pair<std::string, std::string>> fields;
        fields.emplace_back("enabled", self.enabled ? "True" : "False");
        fields.emplace_back(
            "kinematics",
            nb::cast<std::string>(nb::repr(nb::cast(self.kinematics))));
        fields.emplace_back(
            "dynamics",
            nb::cast<std::string>(nb::repr(nb::cast(self.dynamics))));
        return format_repr("LoopClosureRuntimePolicy", fields);
      });

  nb::class_<sim::LoopClosureResidual>(m, "LoopClosureResidual")
      .def_prop_ro(
          "value",
          [](const sim::LoopClosureResidual& self) { return self.value; })
      .def_ro("norm", &sim::LoopClosureResidual::norm)
      .def_ro("enabled", &sim::LoopClosureResidual::enabled)
      .def_ro("active", &sim::LoopClosureResidual::active)
      .def_ro("coordinates", &sim::LoopClosureResidual::coordinates)
      .def_ro("force_available", &sim::LoopClosureResidual::forceAvailable)
      .def("__repr__", [](const sim::LoopClosureResidual& self) {
        std::vector<std::pair<std::string, std::string>> fields;
        fields.emplace_back(
            "value", nb::cast<std::string>(nb::repr(nb::cast(self.value))));
        fields.emplace_back("norm", std::to_string(self.norm));
        fields.emplace_back("enabled", self.enabled ? "True" : "False");
        fields.emplace_back("active", self.active ? "True" : "False");
        fields.emplace_back(
            "coordinates",
            nb::cast<std::string>(nb::repr(nb::cast(self.coordinates))));
        fields.emplace_back(
            "force_available", self.forceAvailable ? "True" : "False");
        return format_repr("LoopClosureResidual", fields);
      });

  loopClosureClass.def("compute_residual", &sim::LoopClosure::computeResidual)
      .def_prop_ro(
          "name",
          [](const sim::LoopClosure& self) {
            return std::string(self.getName());
          })
      .def_prop_ro("family", &sim::LoopClosure::getFamily)
      .def_prop_ro("frame_a", &sim::LoopClosure::getFrameA)
      .def_prop_ro("frame_b", &sim::LoopClosure::getFrameB)
      .def_prop_ro(
          "offset_a",
          [](const sim::LoopClosure& self) {
            return self.getOffsetA().matrix();
          })
      .def_prop_ro(
          "offset_b",
          [](const sim::LoopClosure& self) {
            return self.getOffsetB().matrix();
          })
      .def_prop_rw(
          "runtime_policy",
          &sim::LoopClosure::getRuntimePolicy,
          &sim::LoopClosure::setRuntimePolicy)
      .def_prop_rw(
          "enabled",
          [](const sim::LoopClosure& self) {
            return self.getRuntimePolicy().enabled;
          },
          [](sim::LoopClosure& self, bool enabled) {
            auto policy = self.getRuntimePolicy();
            policy.enabled = enabled;
            self.setRuntimePolicy(policy);
          })
      .def_prop_rw(
          "kinematics",
          [](const sim::LoopClosure& self) {
            return self.getRuntimePolicy().kinematics;
          },
          [](sim::LoopClosure& self, sim::ClosureKinematicsPolicy kinematics) {
            auto policy = self.getRuntimePolicy();
            policy.kinematics = kinematics;
            self.setRuntimePolicy(policy);
          })
      .def_prop_rw(
          "dynamics",
          [](const sim::LoopClosure& self) {
            return self.getRuntimePolicy().dynamics;
          },
          [](sim::LoopClosure& self, sim::ClosureDynamicsPolicy dynamics) {
            auto policy = self.getRuntimePolicy();
            policy.dynamics = dynamics;
            self.setRuntimePolicy(policy);
          })
      .def_prop_ro("is_valid", &sim::LoopClosure::isValid)
      .def("__repr__", [](const sim::LoopClosure& self) {
        std::vector<std::pair<std::string, std::string>> fields;
        fields.emplace_back("valid", self.isValid() ? "True" : "False");
        if (self.isValid()) {
          fields.emplace_back("name", repr_string(std::string(self.getName())));
          fields.emplace_back(
              "family",
              nb::cast<std::string>(nb::repr(nb::cast(self.getFamily()))));
        }
        return format_repr("LoopClosure", fields);
      });

  nb::class_<sim::Multibody>(m, "Multibody")
      .def(
          "add_link",
          [](sim::Multibody& self, const std::string& name) {
            return self.addLink(name);
          },
          nb::arg("name") = "",
          nb::keep_alive<0, 1>())
      .def(
          "add_link",
          [](sim::Multibody& self,
             const std::string& name,
             const sim::Link& parent,
             const sim::JointSpec& joint) {
            return self.addLink(name, parent, joint);
          },
          nb::arg("name"),
          nb::kw_only(),
          nb::arg("parent"),
          nb::arg("joint") = sim::JointSpec{},
          nb::keep_alive<0, 1>())
      .def(
          "get_link",
          [](sim::Multibody& self, const std::string& name) -> nb::object {
            auto link = self.getLink(name);
            if (!link.has_value()) {
              return nb::none();
            }
            return nb::cast(*link, nb::rv_policy::move);
          },
          nb::arg("name"),
          nb::keep_alive<0, 1>())
      .def(
          "get_joint",
          [](sim::Multibody& self, const std::string& name) -> nb::object {
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
          [](const sim::Multibody& self) {
            return std::string(self.getName());
          },
          [](sim::Multibody& self, const std::string& name) {
            self.setName(name);
          })
      .def_prop_ro("num_links", &sim::Multibody::getLinkCount)
      .def_prop_ro("num_joints", &sim::Multibody::getJointCount)
      .def_prop_ro("num_dofs", &sim::Multibody::getDOFCount)
      .def_prop_ro("links", &sim::Multibody::getLinks)
      .def_prop_ro("joints", &sim::Multibody::getJoints)
      .def_prop_ro("link_names", &sim::Multibody::getLinkNames)
      .def_prop_ro("joint_names", &sim::Multibody::getJointNames)
      .def_prop_ro("mass_matrix", &sim::Multibody::getMassMatrix)
      .def_prop_ro("inverse_mass_matrix", &sim::Multibody::getInverseMassMatrix)
      .def_prop_ro("coriolis_forces", &sim::Multibody::getCoriolisForces)
      .def_prop_ro("gravity_forces", &sim::Multibody::getGravityForces)
      .def_prop_ro(
          "coriolis_and_gravity_forces",
          &sim::Multibody::getCoriolisAndGravityForces)
      .def(
          "compute_inverse_dynamics",
          [](const sim::Multibody& self, const nb::handle& acceleration) {
            return self.computeInverseDynamics(toVectorX(acceleration));
          },
          nb::arg("desired_acceleration"))
      .def(
          "compute_impulse_response",
          [](const sim::Multibody& self, const nb::handle& impulse) {
            return self.computeImpulseResponse(toVectorX(impulse));
          },
          nb::arg("joint_impulse"))
      .def(
          "get_jacobian",
          [](const sim::Multibody& self, const sim::Link& link) {
            return self.getJacobian(link);
          },
          nb::arg("link"))
      .def(
          "get_world_jacobian",
          [](const sim::Multibody& self, const sim::Link& link) {
            return self.getWorldJacobian(link);
          },
          nb::arg("link"))
      .def_prop_ro("is_valid", &sim::Multibody::isValid)
      .def("__repr__", [](const sim::Multibody& self) {
        std::vector<std::pair<std::string, std::string>> fields;
        fields.emplace_back("valid", self.isValid() ? "True" : "False");
        if (self.isValid()) {
          fields.emplace_back("name", repr_string(std::string(self.getName())));
          fields.emplace_back("links", std::to_string(self.getLinkCount()));
          fields.emplace_back("joints", std::to_string(self.getJointCount()));
          fields.emplace_back("dofs", std::to_string(self.getDOFCount()));
        }
        return format_repr("Multibody", fields);
      });

  nb::class_<sim::RigidBody, sim::Frame>(m, "RigidBody")
      .def(
          "apply_force",
          [](sim::RigidBody& self, const nb::handle& force) {
            self.applyForce(toVector3(force));
          },
          nb::arg("force"))
      .def("clear_force", &sim::RigidBody::clearForce)
      .def(
          "apply_torque",
          [](sim::RigidBody& self, const nb::handle& torque) {
            self.applyTorque(toVector3(torque));
          },
          nb::arg("torque"))
      .def("clear_torque", &sim::RigidBody::clearTorque)
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
      .def_prop_rw(
          "linear_velocity",
          &sim::RigidBody::getLinearVelocity,
          [](sim::RigidBody& self, const nb::handle& velocity) {
            self.setLinearVelocity(toVector3(velocity));
          })
      .def_prop_rw(
          "angular_velocity",
          &sim::RigidBody::getAngularVelocity,
          [](sim::RigidBody& self, const nb::handle& velocity) {
            self.setAngularVelocity(toVector3(velocity));
          })
      .def_prop_rw("mass", &sim::RigidBody::getMass, &sim::RigidBody::setMass)
      .def_prop_rw(
          "inertia",
          &sim::RigidBody::getInertia,
          [](sim::RigidBody& self, const nb::handle& inertia) {
            self.setInertia(toMatrix3(inertia));
          })
      .def_prop_rw(
          "force",
          &sim::RigidBody::getForce,
          [](sim::RigidBody& self, const nb::handle& force) {
            self.setForce(toVector3(force));
          })
      .def_prop_rw(
          "torque",
          &sim::RigidBody::getTorque,
          [](sim::RigidBody& self, const nb::handle& torque) {
            self.setTorque(toVector3(torque));
          })
      .def_prop_rw(
          "is_static", &sim::RigidBody::isStatic, &sim::RigidBody::setStatic)
      .def_prop_rw(
          "restitution",
          &sim::RigidBody::getRestitution,
          &sim::RigidBody::setRestitution)
      .def_prop_rw(
          "friction",
          &sim::RigidBody::getFriction,
          &sim::RigidBody::setFriction)
      .def(
          "set_collision_shape",
          &sim::RigidBody::setCollisionShape,
          nb::arg("shape"))
      .def_prop_rw(
          "is_deformable_surface_ccd_obstacle",
          &sim::RigidBody::isDeformableSurfaceCcdObstacle,
          &sim::RigidBody::setDeformableSurfaceCcdObstacle)
      .def_prop_ro("collision_shape", &sim::RigidBody::getCollisionShape)
      .def_prop_ro("has_collision_shape", &sim::RigidBody::hasCollisionShape)
      .def_prop_ro("linear_momentum", &sim::RigidBody::getLinearMomentum)
      .def_prop_ro("angular_momentum", &sim::RigidBody::getAngularMomentum)
      .def_prop_ro("kinetic_energy", &sim::RigidBody::getKineticEnergy)
      .def_prop_ro("potential_energy", &sim::RigidBody::getPotentialEnergy)
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
            validateFiniteVector(value, "linear_velocity");
            self.linearVelocity = value;
          })
      .def_prop_rw(
          "angular_velocity",
          [](const sim::RigidBodyOptions& self) {
            return self.angularVelocity;
          },
          [](sim::RigidBodyOptions& self, const nb::handle& angularVelocity) {
            const auto value = toVector3(angularVelocity);
            validateFiniteVector(value, "angular_velocity");
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
      .def_prop_rw(
          "is_static",
          [](const sim::RigidBodyOptions& self) { return self.isStatic; },
          [](sim::RigidBodyOptions& self, bool isStatic) {
            self.isStatic = isStatic;
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

  nb::class_<sim::CollisionBody>(m, "CollisionBody")
      .def_prop_ro(
          "name", [](const sim::CollisionBody& self) { return self.getName(); })
      .def_prop_ro("is_rigid_body", &sim::CollisionBody::isRigidBody)
      .def_prop_ro("is_link", &sim::CollisionBody::isLink)
      .def_prop_ro("is_valid", &sim::CollisionBody::isValid)
      .def(
          "as_rigid_body",
          [](const sim::CollisionBody& self) -> nb::object {
            auto body = self.asRigidBody();
            if (!body.has_value()) {
              return nb::none();
            }
            return nb::cast(*body, nb::rv_policy::move);
          })
      .def("as_link", [](const sim::CollisionBody& self) -> nb::object {
        auto link = self.asLink();
        if (!link.has_value()) {
          return nb::none();
        }
        return nb::cast(*link, nb::rv_policy::move);
      });

  nb::class_<sim::Contact>(m, "Contact")
      .def_prop_ro(
          "body_a", [](const sim::Contact& self) { return self.bodyA; })
      .def_prop_ro(
          "body_b", [](const sim::Contact& self) { return self.bodyB; })
      .def_prop_ro("point", [](const sim::Contact& self) { return self.point; })
      .def_prop_ro(
          "normal", [](const sim::Contact& self) { return self.normal; })
      .def_prop_ro(
          "depth", [](const sim::Contact& self) { return self.depth; });

  nb::class_<sim::World>(m, "World")
      .def(
          "__init__",
          [](sim::World* self, double timeStep) {
            new (self) sim::World();
            self->setTimeStep(timeStep);
          },
          nb::arg("time_step") = 0.001)
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
          "add_multibody",
          [](sim::World& self, const std::string& name) {
            return self.addMultibody(name);
          },
          nb::arg("name"),
          nb::keep_alive<0, 1>())
      .def(
          "get_multibody",
          [](sim::World& self, const std::string& name) -> nb::object {
            auto multibody = self.getMultibody(name);
            if (!multibody.has_value()) {
              return nb::none();
            }
            return nb::cast(*multibody, nb::rv_policy::move);
          },
          nb::arg("name"),
          nb::keep_alive<0, 1>())
      .def(
          "has_multibody",
          [](const sim::World& self, const std::string& name) {
            return self.hasMultibody(name);
          },
          nb::arg("name"))
      .def(
          "add_loop_closure",
          [](sim::World& self,
             const sim::LoopClosureSpec& spec,
             const nb::handle& name) {
            return self.addLoopClosure(toOptionalName(name), spec);
          },
          nb::arg("spec"),
          nb::kw_only(),
          nb::arg("name") = nb::none(),
          nb::keep_alive<0, 1>())
      .def(
          "add_loop_closure",
          [](sim::World& self,
             const nb::handle& name,
             const sim::LoopClosureSpec& spec) {
            return self.addLoopClosure(toOptionalName(name), spec);
          },
          nb::arg("name"),
          nb::arg("spec"),
          nb::keep_alive<0, 1>())
      .def(
          "add_loop_closure",
          [](sim::World& self,
             const nb::handle& name,
             const nb::handle& frameA,
             const nb::handle& frameB,
             sim::LoopClosureFamily family,
             const nb::handle& offsetA,
             const nb::handle& offsetB) {
            return self.addLoopClosure(
                toOptionalName(name),
                makeLoopClosureSpec(frameA, frameB, family, offsetA, offsetB));
          },
          nb::arg("name") = nb::none(),
          nb::kw_only(),
          nb::arg("frame_a"),
          nb::arg("frame_b"),
          nb::arg("family") = sim::LoopClosureFamily::Rigid,
          nb::arg("offset_a") = nb::none(),
          nb::arg("offset_b") = nb::none(),
          nb::keep_alive<0, 1>())
      .def(
          "has_loop_closure",
          [](const sim::World& self, const std::string& name) {
            return self.hasLoopClosure(name);
          },
          nb::arg("name"))
      .def(
          "get_loop_closure",
          [](sim::World& self, const std::string& name) -> nb::object {
            auto closure = self.getLoopClosure(name);
            if (!closure.has_value()) {
              return nb::none();
            }
            return nb::cast(*closure, nb::rv_policy::move);
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
      .def_prop_ro("is_simulation_mode", &sim::World::isSimulationMode)
      .def("enter_simulation_mode", &sim::World::enterSimulationMode)
      .def(
          "update_kinematics",
          [](sim::World& self) { self.updateKinematics(); },
          nb::call_guard<nb::gil_scoped_release>())
      .def(
          "sync",
          [](sim::World& self, sim::WorldSyncStage stage) { self.sync(stage); },
          nb::arg("stage") = sim::WorldSyncStage::Kinematics,
          nb::call_guard<nb::gil_scoped_release>())
      .def(
          "step",
          [](sim::World& self, std::ptrdiff_t n) {
            DART_EXPERIMENTAL_THROW_T_IF(
                n < 0,
                sim::InvalidArgumentException,
                "World.step(n=...) requires a non-negative step count");
            if (n == 0) {
              return;
            }
            if (!self.isSimulationMode()) {
              self.enterSimulationMode();
            }
            self.step(static_cast<std::size_t>(n));
          },
          nb::arg("n") = 1,
          nb::call_guard<nb::gil_scoped_release>())
      .def_prop_rw(
          "time_step", &sim::World::getTimeStep, &sim::World::setTimeStep)
      .def_prop_rw("time", &sim::World::getTime, &sim::World::setTime)
      .def_prop_rw(
          "gravity",
          &sim::World::getGravity,
          [](sim::World& self, const nb::handle& gravity) {
            self.setGravity(toVector3(gravity));
          })
      .def_prop_rw(
          "rigid_body_solver",
          &sim::World::getRigidBodySolver,
          &sim::World::setRigidBodySolver)
      .def_prop_ro("frame", &sim::World::getFrame)
      .def_prop_ro("num_multibodies", &sim::World::getMultibodyCount)
      .def_prop_ro("num_loop_closures", &sim::World::getLoopClosureCount)
      .def_prop_ro("num_rigid_bodies", &sim::World::getRigidBodyCount)
      .def("collide", &sim::World::collide)
      .def("clear", &sim::World::clear)
      .def("__repr__", [](const sim::World& self) {
        std::vector<std::pair<std::string, std::string>> fields;
        fields.emplace_back(
            "multibodies", std::to_string(self.getMultibodyCount()));
        fields.emplace_back(
            "loop_closures", std::to_string(self.getLoopClosureCount()));
        fields.emplace_back(
            "rigid_bodies", std::to_string(self.getRigidBodyCount()));
        fields.emplace_back("time", repr_double(self.getTime()));
        fields.emplace_back("time_step", repr_double(self.getTimeStep()));
        fields.emplace_back(
            "gravity",
            "(" + repr_double(self.getGravity().x()) + ", "
                + repr_double(self.getGravity().y()) + ", "
                + repr_double(self.getGravity().z()) + ")");
        fields.emplace_back("frame", std::to_string(self.getFrame()));
        fields.emplace_back(
            "simulation_mode",
            self.isSimulationMode() ? std::string("True")
                                    : std::string("False"));
        return format_repr("World", fields);
      });
}

} // namespace dart::python_nb
