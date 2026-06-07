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

#include <dart/simulation/detail/deformable_vbd/vertex_block_kernel.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>

#include <array>

namespace dart::simulation::detail::deformable_vbd {

/// Lame parameters derived from Young's modulus `E` and Poisson ratio `nu`.
struct LameParameters
{
  double mu = 0.0;
  double lambda = 0.0;
};

/// Convert (Young's modulus, Poisson ratio) to Lame parameters.
inline LameParameters lameFromYoungPoisson(double youngsModulus, double poisson)
{
  LameParameters lame;
  lame.mu = youngsModulus / (2.0 * (1.0 + poisson));
  lame.lambda
      = youngsModulus * poisson / ((1.0 + poisson) * (1.0 - 2.0 * poisson));
  return lame;
}

/// Rest-shape data for one tetrahedron, computed once from rest positions.
///
/// `restShapeInverse` is `Dm^{-1}` for `Dm = [X1-X0 | X2-X0 | X3-X0]`, and
/// `restVolume` is the positive rest volume `|det(Dm)| / 6`.
struct TetRestShape
{
  Eigen::Matrix3d restShapeInverse = Eigen::Matrix3d::Identity();
  double restVolume = 0.0;
};

//==============================================================================
inline TetRestShape makeTetRestShape(
    const std::array<Eigen::Vector3d, 4>& restPositions)
{
  Eigen::Matrix3d dm;
  dm.col(0) = restPositions[1] - restPositions[0];
  dm.col(1) = restPositions[2] - restPositions[0];
  dm.col(2) = restPositions[3] - restPositions[0];

  TetRestShape shape;
  shape.restShapeInverse = dm.inverse();
  shape.restVolume = std::abs(dm.determinant()) / 6.0;
  return shape;
}

//==============================================================================
/// Deformation gradient `F = Ds Dm^{-1}` for a tetrahedron, where
/// `Ds = [x1-x0 | x2-x0 | x3-x0]` are the current edge vectors.
inline Eigen::Matrix3d deformationGradient(
    const Eigen::Matrix3d& restShapeInverse,
    const std::array<Eigen::Vector3d, 4>& positions)
{
  Eigen::Matrix3d ds;
  ds.col(0) = positions[1] - positions[0];
  ds.col(1) = positions[2] - positions[0];
  ds.col(2) = positions[3] - positions[0];
  return ds * restShapeInverse;
}

//==============================================================================
/// Stable Neo-Hookean energy density (Smith et al. 2018 form used by the
/// reference): `Psi = (mu/2)(||F||_F^2 - 3) + (lambda/2)(det F - a)^2` with the
/// rest-stabilizing offset `a = 1 + mu/lambda`. There is no log term, so the
/// energy stays finite and smooth under element inversion (`det F <= 0`).
inline double stableNeoHookeanEnergyDensity(
    const Eigen::Matrix3d& F, double mu, double lambda)
{
  const double iC = F.squaredNorm();
  const double j = F.determinant();
  const double a = 1.0 + mu / lambda;
  return 0.5 * mu * (iC - 3.0) + 0.5 * lambda * (j - a) * (j - a);
}

//==============================================================================
/// First Piola-Kirchhoff stress `P = dPsi/dF` of the stable Neo-Hookean energy:
/// `P = mu F + lambda (det F - a) cof(F)`, where `cof(F)` (the column-wise
/// cross-product cofactor) equals `dJ/dF`. At rest (`F = I`) the stress is
/// zero, so the rest state is a force equilibrium.
inline Eigen::Matrix3d stableNeoHookeanStress(
    const Eigen::Matrix3d& F, double mu, double lambda)
{
  const Eigen::Vector3d f0 = F.col(0);
  const Eigen::Vector3d f1 = F.col(1);
  const Eigen::Vector3d f2 = F.col(2);
  Eigen::Matrix3d cofactor;
  cofactor.col(0) = f1.cross(f2);
  cofactor.col(1) = f2.cross(f0);
  cofactor.col(2) = f0.cross(f1);

  const double j = F.determinant();
  const double a = 1.0 + mu / lambda;
  return mu * F + lambda * (j - a) * cofactor;
}

//==============================================================================
/// Accumulate one tetrahedron's stable Neo-Hookean contribution to the block of
/// its local vertex `localVertex` (0..3): the force `f += -A P g_i` and the
/// exact (non-PSD-projected) 3x3 Hessian `H += A d(P g_i)/dx_i`, where `A` is
/// the rest volume and `g_i` is the vertex's shape gradient.
///
/// The Hessian is assembled column by column from the analytic stress
/// differential `dP` along each coordinate direction, using only cofactor
/// cross-products — this avoids hand-indexing the 9x9 `d^2Psi/dF^2`. The result
/// is the true energy Hessian (indefinite under inversion); the VBD driver's
/// inertia term anchors the assembled per-vertex block to stay
/// positive-definite, matching the reference, which does not PSD-project the
/// element Hessian by default.
inline void addNeoHookeanTetTerm(
    VertexBlock& block,
    int localVertex,
    const TetRestShape& rest,
    const std::array<Eigen::Vector3d, 4>& positions,
    double mu,
    double lambda)
{
  // Shape gradients g_i with F = sum_i x_i g_i^T. Columns of Dm^{-T} give
  // g_1, g_2, g_3; g_0 closes the partition of unity.
  const Eigen::Matrix3d gradient = rest.restShapeInverse.transpose();
  std::array<Eigen::Vector3d, 4> g;
  g[1] = gradient.col(0);
  g[2] = gradient.col(1);
  g[3] = gradient.col(2);
  g[0] = -(g[1] + g[2] + g[3]);

  Eigen::Matrix3d F = Eigen::Matrix3d::Zero();
  for (int i = 0; i < 4; ++i) {
    F.noalias() += positions[i] * g[i].transpose();
  }

  const Eigen::Vector3d f0 = F.col(0);
  const Eigen::Vector3d f1 = F.col(1);
  const Eigen::Vector3d f2 = F.col(2);
  Eigen::Matrix3d cofactor;
  cofactor.col(0) = f1.cross(f2);
  cofactor.col(1) = f2.cross(f0);
  cofactor.col(2) = f0.cross(f1);

  const double j = F.determinant();
  const double a = 1.0 + mu / lambda;
  const Eigen::Matrix3d stress = mu * F + lambda * (j - a) * cofactor;

  const Eigen::Vector3d gi = g[localVertex];
  const double volume = rest.restVolume;

  // Force on this vertex: f = -A P g_i.
  block.force.noalias() -= volume * (stress * gi);

  // Hessian column d: A * dP(dF = e_d g_i^T) * g_i.
  for (int d = 0; d < 3; ++d) {
    const Eigen::Vector3d ed = Eigen::Vector3d::Unit(d);
    const Eigen::Vector3d df0 = gi[0] * ed;
    const Eigen::Vector3d df1 = gi[1] * ed;
    const Eigen::Vector3d df2 = gi[2] * ed;
    Eigen::Matrix3d dF;
    dF.col(0) = df0;
    dF.col(1) = df1;
    dF.col(2) = df2;

    const double dJ = (cofactor.array() * dF.array()).sum();
    Eigen::Matrix3d dCofactor;
    dCofactor.col(0) = df1.cross(f2) + f1.cross(df2);
    dCofactor.col(1) = df2.cross(f0) + f2.cross(df0);
    dCofactor.col(2) = df0.cross(f1) + f0.cross(df1);

    const Eigen::Matrix3d dStress
        = mu * dF + lambda * (dJ * cofactor + (j - a) * dCofactor);
    block.hessian.col(d).noalias() += volume * (dStress * gi);
  }
}

} // namespace dart::simulation::detail::deformable_vbd
