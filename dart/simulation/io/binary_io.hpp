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

#include <dart/simulation/export.hpp>

#include <Eigen/Dense>
#include <entt/entt.hpp>

#include <iostream>
#include <span>
#include <string>
#include <string_view>
#include <type_traits>
#include <unordered_map>
#include <vector>

#include <cstddef>
#include <cstdint>

namespace dart::simulation::io {

/// Entity ID mapping table for serialization
/// Maps old entity IDs (before serialization) to new IDs (after
/// deserialization)
using EntityMap = std::unordered_map<entt::entity, entt::entity>;

//==============================================================================
// Binary Format Versioning
//==============================================================================

// Simulation binary format version
// Increment this when making breaking changes to the binary format
// Version history:
//   1: Initial implementation
//   2: World gravity serialized after timing metadata; Joint v1 migration
//   3: MultibodyVariationalState (variational-integrator two-step history)
//      registered as a serializable component
//   4: LoopClosure gained a `distance` field (target separation for the
//      Distance family)
//   5: Link gained an `externalForce` field (per-step external spatial wrench
//      applied via Link::applyForce; serialized, round-trips to zero)
//   6: World differentiable flag serialized after the deformable-body counter
//   7: CollisionShape mesh vertices and triangle indices serialized
//   8: VariationalContact (variational-integrator ground-contact config, incl.
//      the augmented-Lagrangian dual-update cadence) and its
//      VariationalContactDualState (per-point duals + cadence counter)
//      registered as serializable components
//   9: Link gained a separate parent-to-joint transform before the legacy
//      joint-to-link transform for preserving imported joint origins
//   10: DeformableMaterial gained `useMatrixFreeLinearSolver`
//   11: Reserved by the experimental-model-loader branch for collision shape
//      local transforms, Plane, Mesh, and multiple CollisionGeometry shapes
//   12: Merged DART 7 format: mainline versions 6-10 plus the branch collision
//      geometry record layout, storing capsule/cylinder half-height in
//      CollisionShape::halfExtents.z()
//   13: Joint stores captured rigid-body fixed-joint local anchors and target
//      relative orientation.
//   14: Joint stores AVBD break-force and broken-state fields.
//   15: World solver-family metadata serialized after the differentiable flag
//      (rigid-body solver, contact method, contact-gradient mode, and
//      multibody integration method).
//   16: World stores persistent ignored collision pairs after solver-family
//      metadata.
//   17: Joint stores public AVBD point-joint stiffness facade fields.
//   18: World stores variational multibody solve budget metadata.
//   19: Private AVBD rigid-world point-joint and distance-spring configs are
//      serializable.
//   20: AVBD point-joint stiffness moved off the Joint record into a separate
//      comps::AvbdJointStiffness component (present only when materialized).
//   21: Component on-disk identity is now an explicit, stable string ID
//      ("comps.<Type>"/"compute.<Type>") instead of the compiler-mangled
//      typeid(T).name(). This decouples the on-disk identity from the C++ type
//      spelling and makes the format compiler-independent. Clean break:
//      DART 7 has no compatibility debt, so packets written with mangled-name
//      component identities (versions <= 20) no longer round-trip their
//      components. See WP-091.23 and comps/component_category.hpp.
//   22: The unified comps.Joint record is split into three contract-aligned
//      components: comps.JointModel (frozen topology, geometry, and solver
//      parameters), comps.JointState (per-step position/velocity/acceleration
//      and the runtime broken flag), and comps.JointActuation (actuator mode,
//      commanded effort, commanded velocity). Clean break: DART 7 has no
//      compatibility debt, so packets written with the unified comps.Joint
//      record (versions <= 21) no longer round-trip the joint. See WP-091.20.
//   23: World automatic deactivation options serialized after variational
//      multibody solve budget metadata, plus DeactivationState as a
//      serializable runtime component.
constexpr std::uint32_t kBinaryFormatVersion = 23;

//==============================================================================
// Low-level Binary I/O for POD types
//==============================================================================

namespace detail {

template <typename T>
concept TriviallyCopyable
    = std::is_trivially_copyable_v<std::remove_cvref_t<T>>;

inline void writeBytes(std::ostream& out, std::span<const std::byte> bytes)
{
  out.write(
      reinterpret_cast<const char*>(bytes.data()),
      static_cast<std::streamsize>(bytes.size()));
}

inline void readBytes(std::istream& in, std::span<std::byte> bytes)
{
  in.read(
      reinterpret_cast<char*>(bytes.data()),
      static_cast<std::streamsize>(bytes.size()));
}

template <typename T>
std::span<const std::byte> asBytes(const T& value)
{
  return std::as_bytes(std::span<const T>(&value, 1u));
}

template <typename T>
std::span<std::byte> asWritableBytes(T& value)
{
  return std::as_writable_bytes(std::span<T>(&value, 1u));
}

} // namespace detail

// Write a POD (Plain Old Data) type to binary stream
template <detail::TriviallyCopyable T>
void writePOD(std::ostream& out, const T& value)
{
  detail::writeBytes(out, detail::asBytes(value));
}

// Read a POD (Plain Old Data) type from binary stream
template <detail::TriviallyCopyable T>
void readPOD(std::istream& in, T& value)
{
  detail::readBytes(in, detail::asWritableBytes(value));
}

//==============================================================================
// String I/O
//==============================================================================

// Write std::string to binary stream (length-prefixed)
void DART_SIMULATION_API writeString(std::ostream& out, std::string_view str);

// Read std::string from binary stream
void DART_SIMULATION_API readString(std::istream& in, std::string& str);

//==============================================================================
// Eigen Type I/O
//==============================================================================

// Write Eigen::Vector3d to binary stream
void DART_SIMULATION_API
writeVector3d(std::ostream& out, const Eigen::Vector3d& vec);

// Read Eigen::Vector3d from binary stream
void DART_SIMULATION_API readVector3d(std::istream& in, Eigen::Vector3d& vec);

// Write Eigen::Isometry3d to binary stream
// Format: 9 doubles for rotation matrix + 3 doubles for translation
void DART_SIMULATION_API
writeIsometry3d(std::ostream& out, const Eigen::Isometry3d& T);

// Read Eigen::Isometry3d from binary stream
void DART_SIMULATION_API readIsometry3d(std::istream& in, Eigen::Isometry3d& T);

// Write Eigen::VectorXd to binary stream (size-prefixed)
void DART_SIMULATION_API
writeVectorXd(std::ostream& out, const Eigen::VectorXd& vec);

template <typename Derived>
  requires(
      std::is_same_v<typename Derived::Scalar, double>
      && Derived::ColsAtCompileTime == 1
      && Derived::RowsAtCompileTime == Eigen::Dynamic)
void writeVectorXd(std::ostream& out, const Eigen::MatrixBase<Derived>& vec)
{
  std::size_t size = static_cast<std::size_t>(vec.size());
  writePOD(out, size);
  if (size > 0) {
    detail::writeBytes(
        out,
        std::as_bytes(std::span<const double>(vec.derived().data(), size)));
  }
}

// Read Eigen::VectorXd from binary stream
void DART_SIMULATION_API readVectorXd(std::istream& in, Eigen::VectorXd& vec);

template <typename Derived>
  requires(
      std::is_same_v<typename Derived::Scalar, double>
      && Derived::ColsAtCompileTime == 1
      && Derived::RowsAtCompileTime == Eigen::Dynamic)
void readVectorXd(std::istream& in, Eigen::PlainObjectBase<Derived>& vec)
{
  std::size_t size;
  readPOD(in, size);
  vec.derived().resize(static_cast<Eigen::Index>(size));
  if (size > 0) {
    detail::readBytes(
        in,
        std::as_writable_bytes(std::span<double>(vec.derived().data(), size)));
  }
}

// Write Eigen::MatrixXd to binary stream (rows/cols prefixed)
void DART_SIMULATION_API
writeMatrixXd(std::ostream& out, const Eigen::MatrixXd& mat);

// Read Eigen::MatrixXd from binary stream
void DART_SIMULATION_API readMatrixXd(std::istream& in, Eigen::MatrixXd& mat);

//==============================================================================
// Container I/O
//==============================================================================

// Write a POD span to binary stream (size-prefixed)
template <detail::TriviallyCopyable T>
void writeVector(std::ostream& out, std::span<const T> vec)
{
  std::size_t size = vec.size();
  writePOD(out, size);
  if (size > 0) {
    detail::writeBytes(out, std::as_bytes(vec));
  }
}

// Read std::vector of POD types from binary stream
template <detail::TriviallyCopyable T>
void readVector(std::istream& in, std::vector<T>& vec)
{
  std::size_t size;
  readPOD(in, size);
  vec.resize(size);
  if (size > 0) {
    detail::readBytes(
        in, std::as_writable_bytes(std::span<T>(vec.data(), vec.size())));
  }
}

//==============================================================================
// Format Header I/O
//==============================================================================

// Write format header with version and magic number
// Magic number: "DRT7" (0x44525437)
void DART_SIMULATION_API writeFormatHeader(std::ostream& out);

// Read and validate format header
// Returns the format version number
// Throws std::runtime_error if magic number is invalid or version incompatible
std::uint32_t DART_SIMULATION_API readFormatHeader(std::istream& in);

} // namespace dart::simulation::io
