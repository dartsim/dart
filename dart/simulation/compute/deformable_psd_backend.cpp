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

#include "dart/simulation/compute/deformable_psd_backend.hpp"

#include <Eigen/Dense>

#include <atomic>

namespace dart::simulation::compute {

namespace {

template <int Size>
void projectFixedSymmetricBlocksToPsd(double* blocks, std::size_t blockCount)
{
  using Matrix = Eigen::Matrix<double, Size, Size>;
  using RowMajorMatrix = Eigen::Matrix<double, Size, Size, Eigen::RowMajor>;
  using Vector = Eigen::Matrix<double, Size, 1>;

  constexpr std::size_t stride = Size * Size;
  for (std::size_t k = 0; k < blockCount; ++k) {
    double* base = blocks + k * stride;
    Eigen::Map<RowMajorMatrix> block(base);
    Matrix symmetric = block;
    symmetric = 0.5 * (symmetric + symmetric.transpose()).eval();
    const Eigen::SelfAdjointEigenSolver<Matrix> solver(symmetric);
    if (solver.info() != Eigen::Success) {
      block.setZero();
      continue;
    }
    const Vector eigenvalues = solver.eigenvalues().cwiseMax(0.0);
    block = solver.eigenvectors() * eigenvalues.asDiagonal()
            * solver.eigenvectors().transpose();
  }
}

void projectDynamicSymmetricBlocksToPsd(
    double* blocks, std::size_t dimension, std::size_t blockCount)
{
  const auto dim = static_cast<Eigen::Index>(dimension);
  const std::size_t stride = dimension * dimension;
  for (std::size_t k = 0; k < blockCount; ++k) {
    double* base = blocks + k * stride;
    // Symmetric blocks: the row-major and column-major views coincide.
    Eigen::Map<Eigen::MatrixXd> block(base, dim, dim);
    const Eigen::MatrixXd symmetric = 0.5 * (block + block.transpose());
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(symmetric);
    if (solver.info() != Eigen::Success) {
      // Drop this element's curvature rather than scatter a possibly-indefinite
      // block; zero keeps the assembled Hessian positive definite (the inertia
      // diagonal still supplies the free-DOF curvature), the conservative IPC
      // choice for a failed per-element eigensolve.
      block.setZero();
      continue;
    }
    const Eigen::VectorXd eigenvalues = solver.eigenvalues().cwiseMax(0.0);
    block = solver.eigenvectors() * eigenvalues.asDiagonal()
            * solver.eigenvectors().transpose();
  }
}

} // namespace

//==============================================================================
void projectSymmetricBlocksToPsdCpu(
    double* blocks, std::size_t dimension, std::size_t blockCount)
{
  if (blocks == nullptr || dimension == 0 || blockCount == 0) {
    return;
  }

  switch (dimension) {
    case 6:
      projectFixedSymmetricBlocksToPsd<6>(blocks, blockCount);
      return;
    case 12:
      projectFixedSymmetricBlocksToPsd<12>(blocks, blockCount);
      return;
    default:
      projectDynamicSymmetricBlocksToPsd(blocks, dimension, blockCount);
      return;
  }
}

namespace {

// The installed batched PSD projection backend. Atomic so installation is
// visible without a separate lock; backends are expected to be set during
// setup, not swapped concurrently with a running solve.
std::atomic<DeformablePsdBlockProjector> g_projector{
    &projectSymmetricBlocksToPsdCpu};

} // namespace

//==============================================================================
void projectSymmetricBlocksToPsd(
    double* blocks, std::size_t dimension, std::size_t blockCount)
{
  DeformablePsdBlockProjector projector
      = g_projector.load(std::memory_order_acquire);
  if (projector == nullptr) {
    projector = &projectSymmetricBlocksToPsdCpu;
  }
  projector(blocks, dimension, blockCount);
}

//==============================================================================
DeformablePsdBlockProjector setDeformablePsdBlockProjector(
    DeformablePsdBlockProjector projector)
{
  if (projector == nullptr) {
    projector = &projectSymmetricBlocksToPsdCpu;
  }
  return g_projector.exchange(projector, std::memory_order_acq_rel);
}

//==============================================================================
DeformablePsdBlockProjector deformablePsdBlockProjector()
{
  return g_projector.load(std::memory_order_acquire);
}

namespace {

// The optional registered accelerator and whether it is currently installed.
// The core never names or links a concrete backend; an accelerator sidecar
// registers itself here. Set during setup, not concurrently with a solve.
DeformablePsdAccelerator g_accelerator{};
std::atomic<bool> g_accelerated{false};

} // namespace

//==============================================================================
void setDeformablePsdAccelerator(const DeformablePsdAccelerator& accelerator)
{
  g_accelerator = accelerator;
}

//==============================================================================
bool isDeformablePsdAccelerationAvailable()
{
  return g_accelerator.available != nullptr && g_accelerator.available();
}

//==============================================================================
bool setDeformablePsdAccelerated(bool enabled)
{
  if (enabled && g_accelerator.available != nullptr && g_accelerator.available()
      && g_accelerator.enable != nullptr && g_accelerator.enable()) {
    g_accelerated.store(true, std::memory_order_release);
    return true;
  }
  if (g_accelerator.disable != nullptr) {
    g_accelerator.disable();
  } else {
    setDeformablePsdBlockProjector(nullptr);
  }
  g_accelerated.store(false, std::memory_order_release);
  return false;
}

//==============================================================================
bool isDeformablePsdAccelerated()
{
  return g_accelerated.load(std::memory_order_acquire);
}

} // namespace dart::simulation::compute
