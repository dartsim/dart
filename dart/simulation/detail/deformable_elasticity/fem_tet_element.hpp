/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary form, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice and this list of conditions in the documentation
 *     and/or other materials provided with the distribution.
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

// Stable neo-Hookean tetrahedral finite-element strain energy.
//
// This is the IPC-class volumetric elasticity kernel: it produces, for one
// linear tetrahedron, the elastic strain energy, its 12x1 nodal force gradient,
// and the (true, not yet PSD-projected) 12x12 nodal Hessian. The solver feeds
// the 12x12 block through the existing batched PSD-projection seam before the
// projected-Newton assembly, exactly like the spring and barrier blocks.
//
// Material: the *stable* neo-Hookean model of Smith, Goes, and Kim,
// "Stable Neo-Hookean Flesh Simulation" (ACM TOG 2018). It is C-infinity and
// finite for *every* deformation gradient F, including degenerate/inverted
// elements (det F <= 0), so it cannot blow up during the Newton line search.
// The original IPC paper (Li et al. 2020) uses neo-Hookean / fixed-corotational
// elasticity; this inversion-robust variant is the documented DART material for
// the FEM elasticity slice (PLAN-081), with fixed-corotational as a follow-up.
//
// Energy density (rest-state-zeroed so a tet at its rest shape stores no
// energy; the constant is dropped in a lambda-stable form):
//
//   psi(F) = (mu/2)(Ic - 3) - (mu/2) ln((Ic + 1)/4)
//          + (lambda/2)(J^2 - 1) - (lambda + 3*mu/4)(J - 1)
//
// with Ic = ||F||_F^2 = tr(F^T F) and J = det F. The rest-stability constant
// alpha = 1 + 3*mu/(4*lambda) only ever appears as lambda*alpha = lambda +
// 3*mu/4, so the kernel stays finite as lambda -> 0 (Poisson ratio -> 0).

#include <Eigen/Core>
#include <Eigen/Dense>

#include <cmath>

namespace dart::simulation::detail::deformable_elasticity {

using Vector9d = Eigen::Matrix<double, 9, 1>;
using Vector12d = Eigen::Matrix<double, 12, 1>;
using Matrix9d = Eigen::Matrix<double, 9, 9>;
using Matrix12d = Eigen::Matrix<double, 12, 12>;
using Matrix9x12d = Eigen::Matrix<double, 9, 12>;

/// Lame parameters (shear modulus mu, first Lame parameter lambda).
struct LameParameters
{
  double mu = 0.0;
  double lambda = 0.0;
};

/// Convert engineering (Young's modulus, Poisson ratio) to Lame parameters.
/// Valid for poissonRatio in [0, 0.5); the near-incompressible limit
/// (poissonRatio -> 0.5) sends lambda -> +infinity, which the caller should
/// clamp at the material level.
inline LameParameters lameParameters(
    const double youngsModulus, const double poissonRatio)
{
  LameParameters lame;
  const double e = std::max(0.0, youngsModulus);
  const double nu = poissonRatio;
  lame.mu = e / (2.0 * (1.0 + nu));
  const double denom = (1.0 + nu) * (1.0 - 2.0 * nu);
  lame.lambda = (std::abs(denom) > 0.0) ? (e * nu / denom) : 0.0;
  return lame;
}

/// Rest configuration of a linear tetrahedron: the inverse of the rest edge
/// matrix Dm = [X1-X0 | X2-X0 | X3-X0] and the rest volume |det Dm| / 6.
struct TetRestShape
{
  Eigen::Matrix3d inverseRestEdges = Eigen::Matrix3d::Identity();
  double restVolume = 0.0;
  bool valid = false;
};

/// Build the rest shape from the four rest-position corners. A degenerate
/// (near-zero-volume) tetrahedron yields ``valid == false`` and is skipped by
/// the element evaluator.
inline TetRestShape makeTetRestShape(
    const Eigen::Vector3d& restX0,
    const Eigen::Vector3d& restX1,
    const Eigen::Vector3d& restX2,
    const Eigen::Vector3d& restX3)
{
  TetRestShape rest;
  Eigen::Matrix3d dm;
  dm.col(0) = restX1 - restX0;
  dm.col(1) = restX2 - restX0;
  dm.col(2) = restX3 - restX0;
  const double det = dm.determinant();
  constexpr double kMinRestVolume = 1e-14;
  if (!std::isfinite(det) || std::abs(det) <= 6.0 * kMinRestVolume) {
    return rest;
  }
  rest.inverseRestEdges = dm.inverse();
  rest.restVolume = std::abs(det) / 6.0;
  rest.valid = true;
  return rest;
}

/// Deformation gradient F = Ds * Bm, with Ds = [x1-x0 | x2-x0 | x3-x0].
inline Eigen::Matrix3d deformationGradient(
    const Eigen::Vector3d& x0,
    const Eigen::Vector3d& x1,
    const Eigen::Vector3d& x2,
    const Eigen::Vector3d& x3,
    const Eigen::Matrix3d& inverseRestEdges)
{
  Eigen::Matrix3d ds;
  ds.col(0) = x1 - x0;
  ds.col(1) = x2 - x0;
  ds.col(2) = x3 - x0;
  return ds * inverseRestEdges;
}

namespace detail {

/// Skew-symmetric cross-product matrix [v]x with [v]x a = v x a.
inline Eigen::Matrix3d skew(const Eigen::Vector3d& v)
{
  Eigen::Matrix3d m;
  m << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
  return m;
}

/// dJ/dF as a 3x3 matrix (the cofactor matrix), columns f1xf2, f2xf0, f0xf1.
/// Robust for every F (no inverse), so it is well defined at det F = 0.
inline Eigen::Matrix3d determinantGradient(const Eigen::Matrix3d& f)
{
  Eigen::Matrix3d g;
  g.col(0) = f.col(1).cross(f.col(2));
  g.col(1) = f.col(2).cross(f.col(0));
  g.col(2) = f.col(0).cross(f.col(1));
  return g;
}

/// d^2 J / dF^2 as a 9x9 matrix in column-stacked vec(F) ordering
/// [f0; f1; f2]. Block (i,j) is the 3x3 second derivative w.r.t. columns i, j.
inline Matrix9d determinantHessian(const Eigen::Matrix3d& f)
{
  const Eigen::Matrix3d s0 = skew(f.col(0));
  const Eigen::Matrix3d s1 = skew(f.col(1));
  const Eigen::Matrix3d s2 = skew(f.col(2));
  Matrix9d h = Matrix9d::Zero();
  h.block<3, 3>(0, 3) = -s2;
  h.block<3, 3>(0, 6) = s1;
  h.block<3, 3>(3, 0) = s2;
  h.block<3, 3>(3, 6) = -s0;
  h.block<3, 3>(6, 0) = -s1;
  h.block<3, 3>(6, 3) = s0;
  return h;
}

/// Derivative d vec(R) / d vec(F) (column-stacked vec ordering) of the polar
/// rotation R from F = R S, given R and the symmetric stretch S = R^T F.
///
/// Differentiating F = R S and isolating the skew part gives
/// (tr(S) I - S) w = axl(R^T dF - dF^T R), with dR = R [w]x. Applying this to
/// each of the nine unit perturbations of F assembles the 9x9 derivative. The
/// matrix (tr(S) I - S) has eigenvalues s_j + s_k (sums of the *other* two
/// singular values), so it is invertible for any non-degenerate non-inverted
/// element; ``false`` is returned when it is singular (a collapsed/inverted
/// element), so the caller can fall back to the Gauss-Newton Hessian.
inline bool rotationGradient(
    const Eigen::Matrix3d& r, const Eigen::Matrix3d& s, Matrix9d& dRdF)
{
  const Eigen::Matrix3d a = s.trace() * Eigen::Matrix3d::Identity() - s;
  const double detA = a.determinant();
  if (!std::isfinite(detA) || std::abs(detA) < 1e-12) {
    return false;
  }
  const Eigen::Matrix3d aInv = a.inverse();
  for (int col = 0; col < 3; ++col) {
    for (int row = 0; row < 3; ++row) {
      // Unit perturbation dF = e_row e_col^T, so M = R^T dF places the row-th
      // column of R^T into column `col` and is zero elsewhere.
      Eigen::Matrix3d m = Eigen::Matrix3d::Zero();
      m.col(col) = r.transpose().col(row);
      Eigen::Vector3d axl;
      axl.x() = m(2, 1) - m(1, 2);
      axl.y() = m(0, 2) - m(2, 0);
      axl.z() = m(1, 0) - m(0, 1);
      const Eigen::Vector3d w = aInv * axl;
      const Eigen::Matrix3d dR = r * skew(w);
      dRdF.col(row + 3 * col) = Eigen::Map<const Vector9d>(dR.data());
    }
  }
  return true;
}

} // namespace detail

/// Stable neo-Hookean strain-energy density (rest-state-zeroed).
inline double stableNeoHookeanEnergyDensity(
    const Eigen::Matrix3d& f, const LameParameters& lame)
{
  const double ic = f.squaredNorm();
  const double j = f.determinant();
  const double lambdaAlpha = lame.lambda + 0.75 * lame.mu;
  return 0.5 * lame.mu * (ic - 3.0) - 0.5 * lame.mu * std::log((ic + 1.0) / 4.0)
         + 0.5 * lame.lambda * (j * j - 1.0) - lambdaAlpha * (j - 1.0);
}

/// First Piola-Kirchhoff stress P = dpsi/dF (a 3x3 matrix).
inline Eigen::Matrix3d stableNeoHookeanFirstPiola(
    const Eigen::Matrix3d& f, const LameParameters& lame)
{
  const double ic = f.squaredNorm();
  const double j = f.determinant();
  const double lambdaAlpha = lame.lambda + 0.75 * lame.mu;
  Eigen::Matrix3d p = lame.mu * (ic / (ic + 1.0)) * f;
  p.noalias()
      += (lame.lambda * j - lambdaAlpha) * detail::determinantGradient(f);
  return p;
}

/// Material Hessian d^2 psi / dF^2 (9x9, column-stacked vec(F) ordering).
/// This is the *true* energy Hessian; it may be indefinite for large
/// deformations and is meant to be PSD-projected by the caller before the
/// projected-Newton solve.
inline Matrix9d stableNeoHookeanEnergyHessian(
    const Eigen::Matrix3d& f, const LameParameters& lame)
{
  const double ic = f.squaredNorm();
  const double j = f.determinant();
  const double lambdaAlpha = lame.lambda + 0.75 * lame.mu;
  const double denom = ic + 1.0;

  const Eigen::Map<const Vector9d> vecF(f.data());
  const Eigen::Matrix3d g = detail::determinantGradient(f);
  const Eigen::Map<const Vector9d> vecG(g.data());

  Matrix9d h = Matrix9d::Zero();
  h.diagonal().array() += lame.mu * (ic / denom);
  h.noalias() += (2.0 * lame.mu / (denom * denom)) * (vecF * vecF.transpose());
  h.noalias() += lame.lambda * (vecG * vecG.transpose());
  h.noalias()
      += (lame.lambda * j - lambdaAlpha) * detail::determinantHessian(f);
  return h;
}

/// Constant Jacobian dvec(F)/dq (9x12), mapping nodal coordinates
/// q = [x0; x1; x2; x3] to the column-stacked deformation gradient.
inline Matrix9x12d deformationGradientJacobian(
    const Eigen::Matrix3d& inverseRestEdges)
{
  Matrix9x12d dFdq = Matrix9x12d::Zero();
  for (int k = 0; k < 3; ++k) {
    const double b0 = inverseRestEdges(0, k);
    const double b1 = inverseRestEdges(1, k);
    const double b2 = inverseRestEdges(2, k);
    const double c0 = -(b0 + b1 + b2);
    for (int d = 0; d < 3; ++d) {
      const int row = 3 * k + d;
      dFdq(row, 0 + d) = c0; // node 0
      dFdq(row, 3 + d) = b0; // node 1
      dFdq(row, 6 + d) = b1; // node 2
      dFdq(row, 9 + d) = b2; // node 3
    }
  }
  return dFdq;
}

/// Per-element energy, 12x1 gradient, and 12x12 Hessian for one tetrahedron.
struct TetElementResult
{
  double energy = 0.0;
  Vector12d gradient = Vector12d::Zero();
  Matrix12d hessian = Matrix12d::Zero();
  bool valid = false;
};

/// Evaluate the stable neo-Hookean element. The gradient and Hessian are scaled
/// by the rest volume so they integrate the energy over the element. Set
/// ``computeHessian`` to false to skip the (more expensive) Hessian assembly,
/// e.g. inside a gradient-only line-search probe.
inline TetElementResult evaluateStableNeoHookeanTet(
    const Eigen::Vector3d& x0,
    const Eigen::Vector3d& x1,
    const Eigen::Vector3d& x2,
    const Eigen::Vector3d& x3,
    const TetRestShape& rest,
    const LameParameters& lame,
    const bool computeHessian = true)
{
  TetElementResult result;
  if (!rest.valid) {
    return result;
  }

  const Eigen::Matrix3d f
      = deformationGradient(x0, x1, x2, x3, rest.inverseRestEdges);
  const double w = rest.restVolume;

  result.energy = w * stableNeoHookeanEnergyDensity(f, lame);

  const Eigen::Matrix3d p = stableNeoHookeanFirstPiola(f, lame);
  const Matrix9x12d dFdq = deformationGradientJacobian(rest.inverseRestEdges);
  const Eigen::Map<const Vector9d> vecP(p.data());
  result.gradient.noalias() = w * (dFdq.transpose() * vecP);

  if (computeHessian) {
    const Matrix9d hMaterial = stableNeoHookeanEnergyHessian(f, lame);
    result.hessian.noalias() = w * (dFdq.transpose() * hMaterial * dFdq);
  }

  result.valid = true;
  return result;
}

// ---------------------------------------------------------------------------
// Fixed-corotational elasticity (the IPC paper's other isotropic material).
//
//   psi(F) = mu ||F - R||_F^2 + (lambda/2)(J - 1)^2
//
// with R the rotation from the polar decomposition F = R S. The first
// Piola-Kirchhoff stress is exactly P = 2*mu*(F - R) + lambda*(J - 1) dJ/dF:
// the rotation-gradient cross term <F - R, dR/dF> vanishes because R^T dR is
// skew and (F - R) = R(S - I) with S symmetric. The element Hessian uses the
// positive-definite Gauss-Newton approximation 2*mu*I9 + lambda
// (dJ/dF)(dJ/dF)^T (treating the rotation as fixed), which gives a valid Newton
// descent direction without the intricate rotation Hessian; the exact analytic
// eigensystem is a later accuracy/perf follow-up.
// ---------------------------------------------------------------------------

/// Rotation R from the polar decomposition F = R S (a proper rotation, det +1).
inline Eigen::Matrix3d polarRotation(const Eigen::Matrix3d& f)
{
  Eigen::JacobiSVD<Eigen::Matrix3d, Eigen::ComputeFullU | Eigen::ComputeFullV>
      svd(f);
  Eigen::Matrix3d u = svd.matrixU();
  const Eigen::Matrix3d& v = svd.matrixV();
  // Reflect the smallest-singular-value axis if U V^T is a reflection, so R is
  // a proper rotation (det R = +1).
  if ((u * v.transpose()).determinant() < 0.0) {
    u.col(2) *= -1.0;
  }
  return u * v.transpose();
}

/// Fixed-corotational strain-energy density, given the precomputed polar
/// rotation ``r`` of ``f`` so the per-element SVD can be shared with the
/// stress.
inline double fixedCorotationalEnergyDensity(
    const Eigen::Matrix3d& f,
    const Eigen::Matrix3d& r,
    const LameParameters& lame)
{
  const double j = f.determinant();
  return lame.mu * (f - r).squaredNorm()
         + 0.5 * lame.lambda * (j - 1.0) * (j - 1.0);
}

/// Fixed-corotational strain-energy density.
inline double fixedCorotationalEnergyDensity(
    const Eigen::Matrix3d& f, const LameParameters& lame)
{
  return fixedCorotationalEnergyDensity(f, polarRotation(f), lame);
}

/// Fixed-corotational first Piola-Kirchhoff stress (exact), given the
/// precomputed polar rotation ``r`` of ``f``.
inline Eigen::Matrix3d fixedCorotationalFirstPiola(
    const Eigen::Matrix3d& f,
    const Eigen::Matrix3d& r,
    const LameParameters& lame)
{
  const double j = f.determinant();
  Eigen::Matrix3d p = 2.0 * lame.mu * (f - r);
  p.noalias() += lame.lambda * (j - 1.0) * detail::determinantGradient(f);
  return p;
}

/// Fixed-corotational first Piola-Kirchhoff stress (exact).
inline Eigen::Matrix3d fixedCorotationalFirstPiola(
    const Eigen::Matrix3d& f, const LameParameters& lame)
{
  return fixedCorotationalFirstPiola(f, polarRotation(f), lame);
}

/// Positive-definite Gauss-Newton material Hessian for the fixed-corotational
/// model (column-stacked vec(F) ordering). Used directly for
/// inverted/degenerate elements (where the exact rotation Hessian is undefined)
/// and as the inversion- robust fallback inside the element evaluator.
inline Matrix9d fixedCorotationalGaussNewtonHessian(
    const Eigen::Matrix3d& f, const LameParameters& lame)
{
  const Eigen::Matrix3d g = detail::determinantGradient(f);
  const Eigen::Map<const Vector9d> vecG(g.data());
  Matrix9d h = Matrix9d::Zero();
  h.diagonal().array() += 2.0 * lame.mu;
  h.noalias() += lame.lambda * (vecG * vecG.transpose());
  return h;
}

/// Exact fixed-corotational material Hessian d^2 psi / d vec(F)^2
/// (column-stacked vec ordering), given the precomputed polar rotation ``r``.
/// It is
///
///   2*mu*(I9 - dR/dF) + lambda*( vec(g) vec(g)^T + (J - 1) * d^2 J/dF^2 ),
///
/// with g = dJ/dF the cofactor. Unlike the Gauss-Newton approximation this is
/// in general indefinite, exactly like the IPC paper's per-element Hessian, so
/// the solver projects it to PSD through the existing batched seam before
/// assembly. Returns ``false`` (leaving ``h`` untouched) when the rotation
/// gradient is undefined for a collapsed/inverted element, so the caller uses
/// Gauss-Newton.
inline bool fixedCorotationalExactMaterialHessian(
    const Eigen::Matrix3d& f,
    const Eigen::Matrix3d& r,
    const LameParameters& lame,
    Matrix9d& h)
{
  const Eigen::Matrix3d s = r.transpose() * f;
  Matrix9d dRdF;
  if (!detail::rotationGradient(r, s, dRdF)) {
    return false;
  }
  const double j = f.determinant();
  const Eigen::Matrix3d g = detail::determinantGradient(f);
  const Eigen::Map<const Vector9d> vecG(g.data());
  h.noalias() = 2.0 * lame.mu * (Matrix9d::Identity() - dRdF);
  h.noalias() += lame.lambda
                 * (vecG * vecG.transpose()
                    + (j - 1.0) * detail::determinantHessian(f));
  // The true Hessian is symmetric; symmetrize away the rotation gradient's tiny
  // numerical asymmetry so the PSD projection sees an exactly symmetric block.
  h = 0.5 * (h + h.transpose()).eval();
  return true;
}

/// Evaluate the fixed-corotational element (energy + 12x1 gradient + PD 12x12
/// Gauss-Newton Hessian), scaled by the rest volume.
inline TetElementResult evaluateFixedCorotationalTet(
    const Eigen::Vector3d& x0,
    const Eigen::Vector3d& x1,
    const Eigen::Vector3d& x2,
    const Eigen::Vector3d& x3,
    const TetRestShape& rest,
    const LameParameters& lame,
    const bool computeHessian = true)
{
  TetElementResult result;
  if (!rest.valid) {
    return result;
  }

  const Eigen::Matrix3d f
      = deformationGradient(x0, x1, x2, x3, rest.inverseRestEdges);
  const double w = rest.restVolume;

  // The polar-decomposition rotation is the only SVD in the kernel; compute it
  // once and share it between the energy and the stress (the line search
  // evaluates both every probe, so this halves the per-element SVD cost).
  const Eigen::Matrix3d r = polarRotation(f);

  result.energy = w * fixedCorotationalEnergyDensity(f, r, lame);

  const Eigen::Matrix3d p = fixedCorotationalFirstPiola(f, r, lame);
  const Matrix9x12d dFdq = deformationGradientJacobian(rest.inverseRestEdges);
  const Eigen::Map<const Vector9d> vecP(p.data());
  result.gradient.noalias() = w * (dFdq.transpose() * vecP);

  if (computeHessian) {
    // Use the exact (generally indefinite) material Hessian, which the solver
    // PSD-projects; fall back to the always-PSD Gauss-Newton form only when the
    // rotation gradient is undefined (a collapsed/inverted element).
    Matrix9d hMaterial;
    if (!fixedCorotationalExactMaterialHessian(f, r, lame, hMaterial)) {
      hMaterial = fixedCorotationalGaussNewtonHessian(f, lame);
    }
    result.hessian.noalias() = w * (dFdq.transpose() * hMaterial * dFdq);
  }

  result.valid = true;
  return result;
}

} // namespace dart::simulation::detail::deformable_elasticity
