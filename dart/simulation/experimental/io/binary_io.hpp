/*
 * Copyright (c) 2011-2026, The DART development contributors
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

#include <dart/simulation/experimental/export.hpp>

#include <Eigen/Dense>
#include <entt/entt.hpp>

#include <iostream>
#include <span>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include <cstdint>

namespace dart::simulation::experimental::io {

/// Entity ID mapping table for serialization
/// Maps old entity IDs (before serialization) to new IDs (after
/// deserialization)
using EntityMap = std::unordered_map<entt::entity, entt::entity>;

//==============================================================================
// Binary Format Versioning
//==============================================================================

// Simulation-experimental binary format version
// Increment this when making breaking changes to the binary format
// Version history:
//   1: Initial implementation
constexpr std::uint32_t kBinaryFormatVersion = 1;

//==============================================================================
// Low-level Binary I/O for POD types
//==============================================================================

// Write a POD (Plain Old Data) type to binary stream
template <typename T>
void writePOD(std::ostream& out, const T& value)
{
  static_assert(
      std::is_trivially_copyable_v<T>,
      "writePOD requires trivially copyable type");
  out.write(reinterpret_cast<const char*>(&value), sizeof(T));
}

// Read a POD (Plain Old Data) type from binary stream
template <typename T>
void readPOD(std::istream& in, T& value)
{
  static_assert(
      std::is_trivially_copyable_v<T>,
      "readPOD requires trivially copyable type");
  in.read(reinterpret_cast<char*>(&value), sizeof(T));
}

//==============================================================================
// String I/O
//==============================================================================

// Write std::string to binary stream (length-prefixed)
void DART_EXPERIMENTAL_API writeString(std::ostream& out, std::string_view str);

// Read std::string from binary stream
void DART_EXPERIMENTAL_API readString(std::istream& in, std::string& str);

//==============================================================================
// Eigen Type I/O
//==============================================================================

// Write Eigen::Vector3d to binary stream
void DART_EXPERIMENTAL_API
writeVector3d(std::ostream& out, const Eigen::Vector3d& vec);

// Read Eigen::Vector3d from binary stream
void DART_EXPERIMENTAL_API readVector3d(std::istream& in, Eigen::Vector3d& vec);

// Write Eigen::Isometry3d to binary stream
// Format: 9 doubles for rotation matrix + 3 doubles for translation
void DART_EXPERIMENTAL_API
writeIsometry3d(std::ostream& out, const Eigen::Isometry3d& T);

// Read Eigen::Isometry3d from binary stream
void DART_EXPERIMENTAL_API
readIsometry3d(std::istream& in, Eigen::Isometry3d& T);

// Write Eigen::VectorXd to binary stream (size-prefixed)
void DART_EXPERIMENTAL_API
writeVectorXd(std::ostream& out, const Eigen::VectorXd& vec);

// Read Eigen::VectorXd from binary stream
void DART_EXPERIMENTAL_API readVectorXd(std::istream& in, Eigen::VectorXd& vec);

// Write Eigen::MatrixXd to binary stream (rows/cols prefixed)
void DART_EXPERIMENTAL_API
writeMatrixXd(std::ostream& out, const Eigen::MatrixXd& mat);

// Read Eigen::MatrixXd from binary stream
void DART_EXPERIMENTAL_API readMatrixXd(std::istream& in, Eigen::MatrixXd& mat);

//==============================================================================
// Container I/O
//==============================================================================

// Write a POD span to binary stream (size-prefixed)
template <typename T>
void writeVector(std::ostream& out, std::span<const T> vec)
{
  static_assert(
      std::is_trivially_copyable_v<T>,
      "writeVector requires trivially copyable type");
  std::size_t size = vec.size();
  writePOD(out, size);
  if (size > 0) {
    out.write(reinterpret_cast<const char*>(vec.data()), size * sizeof(T));
  }
}

// Read std::vector of POD types from binary stream
template <typename T>
void readVector(std::istream& in, std::vector<T>& vec)
{
  static_assert(
      std::is_trivially_copyable_v<T>,
      "readVector requires trivially copyable type");
  std::size_t size;
  readPOD(in, size);
  vec.resize(size);
  if (size > 0) {
    in.read(reinterpret_cast<char*>(vec.data()), size * sizeof(T));
  }
}

//==============================================================================
// Format Header I/O
//==============================================================================

// Write format header with version and magic number
// Magic number: "DRT7" (0x44525437)
void DART_EXPERIMENTAL_API writeFormatHeader(std::ostream& out);

// Read and validate format header
// Returns the format version number
// Throws std::runtime_error if magic number is invalid or version incompatible
std::uint32_t DART_EXPERIMENTAL_API readFormatHeader(std::istream& in);

} // namespace dart::simulation::experimental::io
