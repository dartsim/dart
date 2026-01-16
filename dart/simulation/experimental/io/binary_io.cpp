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

#include "dart/simulation/experimental/io/binary_io.hpp"

#include <stdexcept>

namespace dart::simulation::experimental::io {

//==============================================================================
// String I/O
//==============================================================================

void writeString(std::ostream& out, std::string_view str)
{
  std::size_t size = str.size();
  writePOD(out, size);
  if (size > 0) {
    out.write(str.data(), size);
  }
}

//==============================================================================
void readString(std::istream& in, std::string& str)
{
  std::size_t size;
  readPOD(in, size);
  str.resize(size);
  if (size > 0) {
    in.read(str.data(), size);
  }
}

//==============================================================================
// Eigen Type I/O
//==============================================================================

void writeVector3d(std::ostream& out, const Eigen::Vector3d& vec)
{
  writePOD(out, vec.x());
  writePOD(out, vec.y());
  writePOD(out, vec.z());
}

//==============================================================================
void readVector3d(std::istream& in, Eigen::Vector3d& vec)
{
  double x, y, z;
  readPOD(in, x);
  readPOD(in, y);
  readPOD(in, z);
  vec = Eigen::Vector3d(x, y, z);
}

//==============================================================================
void writeIsometry3d(std::ostream& out, const Eigen::Isometry3d& T)
{
  // Write rotation matrix (3x3 = 9 doubles)
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      writePOD(out, T.linear()(i, j));
    }
  }
  // Write translation (3 doubles)
  writeVector3d(out, T.translation());
}

//==============================================================================
void readIsometry3d(std::istream& in, Eigen::Isometry3d& T)
{
  // Read rotation matrix
  Eigen::Matrix3d R;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      readPOD(in, R(i, j));
    }
  }
  // Read translation
  Eigen::Vector3d t;
  readVector3d(in, t);

  T = Eigen::Isometry3d::Identity();
  T.linear() = R;
  T.translation() = t;
}

//==============================================================================
void writeVectorXd(std::ostream& out, const Eigen::VectorXd& vec)
{
  std::size_t size = vec.size();
  writePOD(out, size);
  for (Eigen::Index i = 0; i < vec.size(); ++i) {
    writePOD(out, vec(i));
  }
}

//==============================================================================
void readVectorXd(std::istream& in, Eigen::VectorXd& vec)
{
  std::size_t size;
  readPOD(in, size);
  vec.resize(size);
  for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(size); ++i) {
    readPOD(in, vec(i));
  }
}

//==============================================================================
void writeMatrixXd(std::ostream& out, const Eigen::MatrixXd& mat)
{
  std::size_t rows = mat.rows();
  std::size_t cols = mat.cols();
  writePOD(out, rows);
  writePOD(out, cols);
  for (Eigen::Index i = 0; i < mat.rows(); ++i) {
    for (Eigen::Index j = 0; j < mat.cols(); ++j) {
      writePOD(out, mat(i, j));
    }
  }
}

//==============================================================================
void readMatrixXd(std::istream& in, Eigen::MatrixXd& mat)
{
  std::size_t rows, cols;
  readPOD(in, rows);
  readPOD(in, cols);
  mat.resize(rows, cols);
  for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(rows); ++i) {
    for (Eigen::Index j = 0; j < static_cast<Eigen::Index>(cols); ++j) {
      readPOD(in, mat(i, j));
    }
  }
}

//==============================================================================
// Format Header I/O
//==============================================================================

void writeFormatHeader(std::ostream& out)
{
  // Magic number: "DRT7" (0x44525437)
  constexpr std::uint32_t kMagicNumber = 0x44525437;
  writePOD(out, kMagicNumber);
  writePOD(out, kBinaryFormatVersion);
}

//==============================================================================
std::uint32_t readFormatHeader(std::istream& in)
{
  // Read and validate magic number
  constexpr std::uint32_t kMagicNumber = 0x44525437;
  std::uint32_t magic;
  readPOD(in, magic);
  if (magic != kMagicNumber) {
    throw std::runtime_error(
        "Invalid simulation-experimental binary format: magic number mismatch");
  }

  // Read format version
  std::uint32_t version;
  readPOD(in, version);

  // Check version compatibility
  if (version > kBinaryFormatVersion) {
    throw std::runtime_error(
        "Unsupported simulation-experimental binary format version: file "
        "version "
        + std::to_string(version) + " is newer than supported version "
        + std::to_string(kBinaryFormatVersion));
  }

  return version;
}

} // namespace dart::simulation::experimental::io
