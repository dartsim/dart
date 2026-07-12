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

#pragma once

// module.hpp transitively pulls in <nanobind/nanobind.h> and Python's
// <pyconfig.h>, which must be seen before any libc system header that defines
// _POSIX_C_SOURCE / _XOPEN_SOURCE. Pin it first; the guard stops clang-format
// (SortIncludes) from demoting it below the other headers.
// clang-format off
#include "simulation/module.hpp"
// clang-format on

#include "common/eigen_utils.hpp"
#include "common/repr.hpp"

#include <dart/simulation/body/collision_shape.hpp>
#include <dart/simulation/body/contact.hpp>
#include <dart/simulation/body/contact_force.hpp>
#include <dart/simulation/body/deformable_body.hpp>
#include <dart/simulation/body/deformable_body_options.hpp>
#include <dart/simulation/body/rigid_body.hpp>
#include <dart/simulation/body/rigid_body_options.hpp>
#include <dart/simulation/common/exceptions.hpp>
#include <dart/simulation/compute/compute_stage_metadata.hpp>
#include <dart/simulation/compute/deformable_psd_backend.hpp>
#include <dart/simulation/compute/parallel_executor.hpp>
#include <dart/simulation/compute/sequential_executor.hpp>
#include <dart/simulation/compute/world_step_profile.hpp>
#include <dart/simulation/constraint/loop_closure.hpp>
#include <dart/simulation/constraint/loop_closure_spec.hpp>
#include <dart/simulation/diff/physical_parameter.hpp>
#include <dart/simulation/diff/step_derivatives.hpp>
#include <dart/simulation/diff/step_gradient.hpp>
#ifdef DART_HAS_DIFF
  #include <dart/simulation/diff/rollout.hpp>
#endif
#include <dart/simulation/frame/fixed_frame.hpp>
#include <dart/simulation/frame/frame.hpp>
#include <dart/simulation/frame/free_frame.hpp>
#include <dart/simulation/io/codim_mesh.hpp>
#include <dart/simulation/io/deformable_scene_io.hpp>
#include <dart/simulation/io/gmsh_tet_mesh.hpp>
#include <dart/simulation/io/obj_triangle_mesh.hpp>
#include <dart/simulation/io/skeleton_loader.hpp>
#include <dart/simulation/io/skeleton_to_multibody.hpp>
#include <dart/simulation/multibody/joint.hpp>
#include <dart/simulation/multibody/link.hpp>
#include <dart/simulation/multibody/multibody.hpp>
#include <dart/simulation/space/state_space.hpp>
#include <dart/simulation/world.hpp>

#include <dart/dynamics/skeleton.hpp>

#include <dart/common/memory_manager.hpp>
#include <dart/common/uri.hpp>

#include <dart/io/read.hpp>

#include <Eigen/Cholesky>
#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/array.h>
#include <nanobind/stl/filesystem.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <array>
#include <chrono>
#include <fstream>
#include <limits>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include <cmath>
#include <cstddef>

namespace nb = nanobind;

namespace dart::python_nb {

namespace sim = dart::simulation;

// ---------------------------------------------------------------------------
// Shared, file-local helpers for the dartpy simulation bindings.
//
// These were previously an anonymous namespace inside simulation/module.cpp.
// That single translation unit (~3983 lines, ~624 bindings) compiled for
// 2h41m+ in Debug (-O0 -g) and timed out the required CI "Debug Tests" job.
// Because nanobind type registration is order-sensitive, the bindings were
// split -- WITHOUT reordering -- into several module_*.cpp parts that compile
// in parallel. The helpers live here as inline definitions so every part
// shares one ODR-safe copy.
// ---------------------------------------------------------------------------

class PyJointConstraintProjectionPolicy
{
public:
  explicit PyJointConstraintProjectionPolicy(
      sim::JointConstraintProjectionPolicy policy)
    : mPolicy(policy)
  {
  }

  explicit PyJointConstraintProjectionPolicy(sim::Joint joint)
    : mPolicy(joint.getConstraintProjectionPolicy()), mJoint(std::move(joint))
  {
  }

  [[nodiscard]] const sim::JointConstraintProjectionPolicy& getPolicy() const
  {
    return mPolicy;
  }

  [[nodiscard]] double getStartStiffness() const
  {
    return mPolicy.startStiffness;
  }

  void setStartStiffness(double value)
  {
    auto policy = mPolicy;
    policy.startStiffness = value;
    setPolicy(policy);
  }

  [[nodiscard]] double getLinearStiffness() const
  {
    return mPolicy.linearStiffness;
  }

  void setLinearStiffness(double value)
  {
    auto policy = mPolicy;
    policy.linearStiffness = value;
    setPolicy(policy);
  }

  [[nodiscard]] double getAngularStiffness() const
  {
    return mPolicy.angularStiffness;
  }

  void setAngularStiffness(double value)
  {
    auto policy = mPolicy;
    policy.angularStiffness = value;
    setPolicy(policy);
  }

private:
  void setPolicy(const sim::JointConstraintProjectionPolicy& policy)
  {
    if (mJoint.has_value()) {
      mJoint->setConstraintProjectionPolicy(policy);
      mPolicy = mJoint->getConstraintProjectionPolicy();
      return;
    }

    mPolicy = policy;
  }

  sim::JointConstraintProjectionPolicy mPolicy;
  std::optional<sim::Joint> mJoint;
};

class PyDeformableObstaclePolicy
{
public:
  explicit PyDeformableObstaclePolicy(sim::DeformableObstaclePolicy policy)
    : mPolicy(policy)
  {
  }

  explicit PyDeformableObstaclePolicy(sim::RigidBody body)
    : mPolicy(body.getDeformableObstaclePolicy()), mBody(std::move(body))
  {
  }

  [[nodiscard]] const sim::DeformableObstaclePolicy& getPolicy() const
  {
    return mPolicy;
  }

  [[nodiscard]] bool getGroundBarrier() const
  {
    return mPolicy.groundBarrier;
  }

  void setGroundBarrier(bool value)
  {
    auto policy = mPolicy;
    policy.groundBarrier = value;
    setPolicy(policy);
  }

  [[nodiscard]] bool getSurfaceObstacle() const
  {
    return mPolicy.surfaceObstacle;
  }

  void setSurfaceObstacle(bool value)
  {
    auto policy = mPolicy;
    policy.surfaceObstacle = value;
    setPolicy(policy);
  }

  [[nodiscard]] bool getBarrierOnly() const
  {
    return mPolicy.barrierOnly;
  }

  void setBarrierOnly(bool value)
  {
    auto policy = mPolicy;
    policy.barrierOnly = value;
    setPolicy(policy);
  }

private:
  void setPolicy(const sim::DeformableObstaclePolicy& policy)
  {
    if (mBody.has_value()) {
      mBody->setDeformableObstaclePolicy(policy);
      mPolicy = mBody->getDeformableObstaclePolicy();
      return;
    }

    mPolicy = policy;
  }

  sim::DeformableObstaclePolicy mPolicy;
  std::optional<sim::RigidBody> mBody;
};

// Backend-neutral acceleration control for direct deformable PSD backend calls.
// World stepping uses the per-World ComputeAcceleratorPolicy, but these legacy
// wrappers remain useful for tests and direct backend probes. They never name a
// device technology; with no accelerator registered they are safe no-ops that
// report acceleration as unavailable and stay on the CPU backend.
inline bool acceleratedDeformableSolveAvailable()
{
  return sim::compute::isDeformablePsdAccelerationAvailable();
}

inline bool setAcceleratedDeformableSolve(bool enabled)
{
  return sim::compute::setDeformablePsdAccelerated(enabled);
}

inline bool acceleratedDeformableSolveEnabled()
{
  return sim::compute::isDeformablePsdAccelerated();
}

inline Eigen::Quaterniond toQuaternionWxyz(const nb::handle& value)
{
  const auto data = nb::cast<std::array<double, 4>>(value);
  return Eigen::Quaterniond(data[0], data[1], data[2], data[3]);
}

inline Eigen::Vector4d toWxyz(const Eigen::Quaterniond& quaternion)
{
  Eigen::Vector4d out;
  out << quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z();
  return out;
}

inline void validateIsometryMatrix(const Eigen::Matrix4d& matrix)
{
  constexpr double tolerance = 1e-9;

  DART_SIMULATION_THROW_T_IF(
      !matrix.allFinite(),
      sim::InvalidArgumentException,
      "Transform matrix must contain only finite values");

  Eigen::RowVector4d expectedBottom;
  expectedBottom << 0.0, 0.0, 0.0, 1.0;
  DART_SIMULATION_THROW_T_IF(
      (matrix.bottomRows<1>() - expectedBottom).cwiseAbs().maxCoeff()
          > tolerance,
      sim::InvalidArgumentException,
      "Transform matrix bottom row must be [0, 0, 0, 1]");

  const Eigen::Matrix3d rotation = matrix.topLeftCorner<3, 3>();
  const double orthonormalError
      = (rotation * rotation.transpose() - Eigen::Matrix3d::Identity())
            .cwiseAbs()
            .maxCoeff();
  DART_SIMULATION_THROW_T_IF(
      orthonormalError > tolerance
          || std::abs(rotation.determinant() - 1.0) > tolerance,
      sim::InvalidArgumentException,
      "Transform matrix rotation must be orthonormal");
}

inline Eigen::Matrix4d toMatrix4(const nb::handle& value)
{
  if (nb::isinstance<nb::ndarray<double>>(value)) {
    const auto array = nb::cast<nb::ndarray<double>>(value);
    if (array.ndim() != 2 || array.shape(0) != 4 || array.shape(1) != 4) {
      throw nb::type_error("Expected a 4x4 matrix");
    }

    Eigen::Matrix4d matrix;
    const double* base = array.data();
    const int64_t s0 = ::dart::python_nb::numpyStride(array, 0);
    const int64_t s1 = ::dart::python_nb::numpyStride(array, 1);
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

inline Eigen::VectorXd toVectorX(const nb::handle& value)
{
  if (nb::isinstance<nb::ndarray<double>>(value)) {
    const auto array = nb::cast<nb::ndarray<double>>(value);
    if (array.ndim() != 1) {
      throw nb::type_error("Expected a 1-D vector");
    }

    const auto length = static_cast<Eigen::Index>(array.shape(0));
    const int64_t stride = ::dart::python_nb::numpyStride(array, 0);
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

inline Eigen::VectorXd toVectorX(const std::vector<double>& values)
{
  Eigen::VectorXd vector(static_cast<Eigen::Index>(values.size()));
  for (Eigen::Index i = 0; i < vector.size(); ++i) {
    vector[i] = values[static_cast<std::size_t>(i)];
  }
  return vector;
}

inline std::vector<Eigen::Vector3d> toVector3List(const nb::handle& value)
{
  const auto sequence = nb::cast<nb::sequence>(value);
  std::vector<Eigen::Vector3d> result;
  const nb::ssize_t length = nb::len(sequence);
  result.reserve(static_cast<std::size_t>(length));
  for (nb::ssize_t i = 0; i < length; ++i) {
    result.push_back(toVector3(sequence[i]));
  }
  return result;
}

inline std::optional<Eigen::Vector3d> toOptionalVector3(const nb::handle& value)
{
  if (value.is_none()) {
    return std::nullopt;
  }
  return toVector3(value);
}

inline std::vector<Eigen::Vector3i> toTriangleList(const nb::handle& value)
{
  const auto sequence = nb::cast<nb::sequence>(value);
  std::vector<Eigen::Vector3i> result;
  const nb::ssize_t length = nb::len(sequence);
  result.reserve(static_cast<std::size_t>(length));
  for (nb::ssize_t i = 0; i < length; ++i) {
    const auto triangle = nb::cast<nb::sequence>(sequence[i]);
    if (nb::len(triangle) != 3) {
      throw nb::type_error("Expected triangle indices with length 3");
    }
    result.emplace_back(
        nb::cast<int>(triangle[0]),
        nb::cast<int>(triangle[1]),
        nb::cast<int>(triangle[2]));
  }
  return result;
}

inline std::string toOptionalName(const nb::handle& value)
{
  if (value.is_none()) {
    return "";
  }
  return nb::cast<std::string>(value);
}

inline Eigen::Isometry3d toIsometry(const nb::handle& value)
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

inline sim::Frame toFrameHandle(const nb::handle& value)
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

inline nb::list castJointsKeepingWorldAlive(
    std::vector<sim::Joint> joints, const nb::handle& world)
{
  nb::list result;
  for (auto& joint : joints) {
    nb::object jointObject = nb::cast(std::move(joint), nb::rv_policy::move);
    nb::detail::keep_alive(jointObject.ptr(), world.ptr());
    result.append(jointObject);
  }
  return result;
}

inline bool isSymmetricPositiveDefinite(const Eigen::Matrix3d& matrix)
{
  if (!matrix.allFinite() || !matrix.isApprox(matrix.transpose(), 1e-12)) {
    return false;
  }

  Eigen::LLT<Eigen::Matrix3d> factorization(matrix);
  return factorization.info() == Eigen::Success;
}

inline void validateMass(double mass)
{
  DART_SIMULATION_THROW_T_IF(
      !std::isfinite(mass) || mass <= 0.0,
      sim::InvalidArgumentException,
      "RigidBodyOptions.mass must be positive and finite");
}

inline void validateFiniteVector(
    const Eigen::Vector3d& value, const char* fieldName)
{
  DART_SIMULATION_THROW_T_IF(
      !value.allFinite(),
      sim::InvalidArgumentException,
      "RigidBodyOptions.{} must contain only finite values",
      fieldName);
}

inline void validateJointSpecAxis(
    const Eigen::Vector3d& axis, const char* field = "axis")
{
  DART_SIMULATION_THROW_T_IF(
      !axis.allFinite(),
      sim::InvalidArgumentException,
      "JointSpec.{} must contain only finite values",
      field);

  DART_SIMULATION_THROW_T_IF(
      axis.norm() <= 1e-9,
      sim::InvalidArgumentException,
      "JointSpec.{} must be non-zero",
      field);
}

inline void validateJointSpecAnchor(
    const std::optional<Eigen::Vector3d>& anchor, const char* field)
{
  DART_SIMULATION_THROW_T_IF(
      anchor.has_value() && !anchor->allFinite(),
      sim::InvalidArgumentException,
      "JointSpec anchors must be finite ({} must contain only finite values)",
      field);
}

inline void setJointSpecAnchor(
    std::optional<Eigen::Vector3d>& target,
    const nb::handle& anchor,
    const char* field)
{
  target = toOptionalVector3(anchor);
  validateJointSpecAnchor(target, field);
}

inline PyJointConstraintProjectionPolicy getJointConstraintProjectionPolicy(
    const sim::Joint& joint)
{
  return PyJointConstraintProjectionPolicy(joint);
}

inline void setJointConstraintProjectionPolicy(
    sim::Joint& joint, const PyJointConstraintProjectionPolicy& policy)
{
  joint.setConstraintProjectionPolicy(policy.getPolicy());
}

inline PyDeformableObstaclePolicy getDeformableObstaclePolicy(
    const sim::RigidBody& body)
{
  return PyDeformableObstaclePolicy(body);
}

inline void setDeformableObstaclePolicy(
    sim::RigidBody& body, const PyDeformableObstaclePolicy& policy)
{
  body.setDeformableObstaclePolicy(policy.getPolicy());
}

inline void validateOrientation(const Eigen::Quaterniond& orientation)
{
  const auto orientationNorm = orientation.norm();
  DART_SIMULATION_THROW_T_IF(
      !orientation.coeffs().allFinite() || !std::isfinite(orientationNorm)
          || orientationNorm <= 0.0,
      sim::InvalidArgumentException,
      "RigidBodyOptions.orientation must be finite and non-zero");
}

inline void validateInertia(const Eigen::Matrix3d& inertia)
{
  DART_SIMULATION_THROW_T_IF(
      !isSymmetricPositiveDefinite(inertia),
      sim::InvalidArgumentException,
      "RigidBodyOptions.inertia must be symmetric positive definite");
}

inline void validateRigidBodyOptions(const sim::RigidBodyOptions& options)
{
  validateMass(options.mass);
  validateInertia(options.inertia);
  validateFiniteVector(options.position, "position");
  validateFiniteVector(options.linearVelocity, "linear_velocity");
  validateFiniteVector(options.angularVelocity, "angular_velocity");
  validateOrientation(options.orientation);
}

inline sim::LoopClosureSpec makeLoopClosureSpec(
    const nb::handle& frameA,
    const nb::handle& frameB,
    sim::LoopClosureFamily family,
    const nb::handle& offsetA,
    const nb::handle& offsetB,
    double distance)
{
  sim::LoopClosureSpec spec;
  spec.frameA = toFrameHandle(frameA);
  spec.frameB = toFrameHandle(frameB);
  spec.family = family;
  spec.distance = distance;

  if (!offsetA.is_none()) {
    spec.offsetA = toIsometry(offsetA);
  }
  if (!offsetB.is_none()) {
    spec.offsetB = toIsometry(offsetB);
  }

  return spec;
}

inline sim::RigidBodyOptions makeRigidBodyOptions(
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

inline sim::RigidBodyOptions mergeRigidBodyOptions(
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

// Binding parts. defSimulationModule() in module.cpp calls these in order;
// the order is load-bearing because later parts reference types registered
// by earlier parts. Do NOT reorder.
//
// core enums, CollisionShape, StateSpace, JointSpec, policy wrappers
void defSimPartCore(nb::module_& m);
// Frame/FreeFrame/FixedFrame/Joint/Link classes and loop-closure/multibody
// option structs
void defSimPartFrames(nb::module_& m);
// Multibody/RigidBody/Contact/StepGradient and (optional) diff rollout bindings
void defSimPartBodies(nb::module_& m);
// compute executors/profiles and ECS/memory diagnostics
void defSimPartCompute(nb::module_& m);
// deformable material/mesh/body types and io (skeleton/scene) loaders
void defSimPartDeformableIo(nb::module_& m);
// World class and backend-neutral deformable acceleration controls
void defSimPartWorld(nb::module_& m);

} // namespace dart::python_nb
