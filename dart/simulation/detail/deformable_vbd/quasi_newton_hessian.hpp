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
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR
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

#include <Eigen/Core>

#include <algorithm>

#include <cmath>

namespace dart::simulation::detail::deformable_vbd {

/// Return the Section 3.5 column-norm diagonal for
/// `scale * (I - direction * direction^T)`.
///
/// Distance constraints use this projector repeatedly. Computing its column
/// norms analytically avoids materializing a 3 x 3 matrix in the solver loop:
/// the squared norm of column c is `scale^2 * (1 - direction[c]^2)`.
/// Callers provide a unit direction. Non-finite input is deliberately
/// propagated so the owning block validation can reject the solve fail-closed.
inline Eigen::Vector3d avbdQuasiNewtonProjectedDistanceDiagonal(
    const Eigen::Vector3d& direction, double scale)
{
  Eigen::Vector3d diagonal;
  const double magnitude = std::abs(scale);
  for (Eigen::Index column = 0; column < 3; ++column) {
    diagonal[column]
        = magnitude
          * std::sqrt(
              std::max(1.0 - direction[column] * direction[column], 0.0));
  }
  return diagonal;
}

/// Return the AVBD Section 3.5 diagonal approximation of a force-scaled
/// geometric-stiffness matrix.
///
/// For G = f * d^2 C / dx^2, AVBD replaces G with a diagonal matrix whose
/// c-th entry is the Euclidean norm of column c of G. The entries are therefore
/// non-negative even when the force is compressive or G is non-symmetric.
/// Combined with the rank-one penalty term and inertia, this keeps each local
/// block positive-definite and suitable for LDLT.
///
/// AVBD's local blocks are fixed-size. Keeping that requirement explicit here
/// prevents a paper-equation helper from introducing hidden solve-loop
/// allocation.
template <typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, Derived::ColsAtCompileTime, 1>
avbdQuasiNewtonGeometricDiagonal(
    const Eigen::MatrixBase<Derived>& geometricStiffness)
{
  static_assert(
      Derived::RowsAtCompileTime != Eigen::Dynamic
          && Derived::ColsAtCompileTime != Eigen::Dynamic,
      "AVBD quasi-Newton blocks must be fixed-size");
  static_assert(
      Derived::RowsAtCompileTime == Derived::ColsAtCompileTime,
      "AVBD geometric stiffness must be square");

  Eigen::Matrix<typename Derived::Scalar, Derived::ColsAtCompileTime, 1>
      diagonal;
  for (Eigen::Index column = 0; column < Derived::ColsAtCompileTime; ++column) {
    diagonal[column] = geometricStiffness.col(column).norm();
  }
  return diagonal;
}

} // namespace dart::simulation::detail::deformable_vbd
