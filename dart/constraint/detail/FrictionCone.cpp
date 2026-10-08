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

#include <dart/constraint/detail/FrictionCone.hpp>

#include <Eigen/Cholesky>

#include <algorithm>
#include <limits>

#include <cmath>

namespace dart::constraint::detail {
namespace {

using Real = long double;
using Vector = Eigen::Matrix<Real, 3, 1>;
using Matrix = Eigen::Matrix<Real, 3, 3>;
constexpr Real kPi = 3.141592653589793238462643383279502884L;
constexpr Real kTolerance = 1e-10L;

bool valid(const FrictionCone& cone)
{
  return cone.mu.allFinite() && (cone.mu.array() >= 0.0).all();
}

Real support(const Vector& v, const FrictionCone& cone)
{
  const Real a = Real(cone.mu[0]) * v[1];
  const Real b = Real(cone.mu[1]) * v[2];
  return cone.law == FrictionConeLaw::Box ? std::abs(a) + std::abs(b)
                                          : std::hypot(a, b);
}

Real primalViolation(const Vector& v, const FrictionCone& cone)
{
  Real t[2] = {0.0L, 0.0L};
  Real violation = std::max(0.0L, -v[0]);
  for (int i = 0; i < 2; ++i) {
    if (cone.mu[i] > 0.0)
      t[i] = std::abs(v[i + 1]) / Real(cone.mu[i]);
    else
      violation = std::max(violation, std::abs(v[i + 1]));
  }
  const Real gauge = cone.law == FrictionConeLaw::Box ? std::max(t[0], t[1])
                                                      : std::hypot(t[0], t[1]);
  return std::max(violation, gauge - v[0]);
}

bool certificate(
    const Matrix& H,
    const Vector& c,
    const Vector& x,
    const FrictionCone& cone,
    Real tolerance = kTolerance)
{
  if (!x.allFinite())
    return false;
  const Vector Hx = H * x;
  const Vector v = Hx + c;
  const Vector magnitude = H.cwiseAbs() * x.cwiseAbs() + c.cwiseAbs();
  const Real dualScale = 1.0L + magnitude[0] + support(magnitude, cone);
  const Real dotScale = 1.0L + x.cwiseAbs().dot(magnitude);
  return primalViolation(x, cone) <= tolerance * std::max(1.0L, std::abs(x[0]))
         && support(v, cone) - v[0] <= tolerance * dualScale
         && std::abs(x.dot(v)) <= tolerance * dotScale;
}

bool prepare(
    const Eigen::Matrix3d& input,
    const Eigen::Vector3d& c,
    const FrictionCone& cone,
    Matrix& H,
    double& regularization)
{
  if (!input.allFinite() || !c.allFinite() || !valid(cone))
    return false;
  const double scale = input.cwiseAbs().maxCoeff();
  if ((input - input.transpose()).cwiseAbs().maxCoeff()
      > 1e-12 * std::max(1.0, scale))
    return false;
  H = (0.5 * (input + input.transpose())).cast<Real>();
  Eigen::LDLT<Matrix> ldlt(H);
  const Real trace = H.trace();
  if (trace < 0.0L
      || ldlt.vectorD().minCoeff() < -1e-12L * std::max(1.0L, trace))
    return false;
  if (ldlt.vectorD().minCoeff() <= 1e-18L * std::max(1.0L, trace)) {
    regularization = double(1e-12L * (trace > 0.0L ? trace : 1.0L));
    H.diagonal().array() += Real(regularization);
    ldlt.compute(H);
  }
  return ldlt.info() == Eigen::Success && ldlt.vectorD().minCoeff() > 0.0L;
}

struct AnglePoint
{
  Real value = 0.0L;
  Real derivative = 0.0L;
  Real derivativeSlope = 0.0L;
  Vector impulse = Vector::Zero();
};

AnglePoint anglePoint(
    const Matrix& H, const Vector& c, const FrictionCone& cone, Real theta)
{
  const Real ct = std::cos(theta), st = std::sin(theta);
  const Real m1 = cone.mu[0], m2 = cone.mu[1];
  const Vector a(1.0L, m1 * ct, m2 * st);
  const Vector da(0.0L, -m1 * st, m2 * ct);
  const Vector dda(0.0L, -m1 * ct, -m2 * st);
  const Vector Ha = H * a;
  const Real p = c.dot(a), dp = c.dot(da), ddp = c.dot(dda);
  const Real h = a.dot(Ha), dh = 2.0L * da.dot(Ha);
  const Real ddh = 2.0L * (dda.dot(Ha) + da.dot(H * da));
  AnglePoint point;
  // The derivative has the sign of 2 p' h - p h' whenever p < 0.
  point.derivative = 2.0L * dp * h - p * dh;
  point.derivativeSlope = 2.0L * ddp * h + dp * dh - p * ddh;
  if (p < 0.0L && h > 0.0L) {
    point.value = -p * p / (2.0L * h);
    point.impulse = (-p / h) * a;
  }
  return point;
}

Vector refineAngle(
    const Matrix& H,
    const Vector& c,
    const FrictionCone& cone,
    Real theta,
    Real step,
    bool newton)
{
  Real lo = theta - step, hi = theta + step;
  for (int iteration = 0; iteration < (newton ? 48 : 90); ++iteration) {
    const auto point = anglePoint(H, c, cone, theta);
    if (newton && point.derivativeSlope > 0.0L
        && std::abs(point.derivative / point.derivativeSlope)
               <= 4.0L * std::numeric_limits<Real>::epsilon()
                      * (1.0L + std::abs(theta)))
      break;
    if (point.derivative < 0.0L)
      lo = theta;
    else
      hi = theta;
    Real next = (lo + hi) / 2.0L;
    if (newton && point.derivativeSlope > 0.0L) {
      const Real proposal = theta - point.derivative / point.derivativeSlope;
      if (proposal > lo && proposal < hi)
        next = proposal;
    }
    if (next == theta)
      break;
    theta = next;
  }
  return anglePoint(H, c, cone, theta).impulse;
}

Vector scanAngles(
    const Matrix& H,
    const Vector& c,
    const FrictionCone& cone,
    int count,
    bool newton)
{
  const Real step = 2.0L * kPi / count;
  Vector best = Vector::Zero();
  Real bestValue = 0.0L;
  Real before = anglePoint(H, c, cone, -step).value;
  Real here = anglePoint(H, c, cone, 0.0L).value;
  // Refine every sampled local minimum, not just the lowest sample. An
  // uncertified stationary point cannot replace a certified global optimum.
  for (int k = 0; k < count; ++k) {
    const Real next = anglePoint(H, c, cone, (k + 1) * step).value;
    if (here < 0.0L && here <= before && here <= next) {
      const Vector x = refineAngle(H, c, cone, k * step, step, newton);
      const Real value = 0.5L * x.dot(H * x) + c.dot(x);
      if (value < bestValue) {
        bestValue = value;
        best = x;
      }
      if (certificate(H, c, x, cone))
        return x;
    }
    before = here;
    here = next;
  }
  return best;
}

Vector polyhedralQp(const Matrix& H, const Vector& c, const FrictionCone& cone)
{
  Vector best = Vector::Zero();
  Real bestValue = 0.0L;
  // Each axis is free, on its negative face, or on its positive face. This
  // enumerates interior, four faces and four edges (opposite faces meet only
  // at the apex). A zero axis is fixed and has no face multiplier.
  for (int first = -1; first <= 1; ++first) {
    if (cone.mu[0] == 0.0 && first != 0)
      continue;
    for (int second = -1; second <= 1; ++second) {
      if (cone.mu[1] == 0.0 && second != 0)
        continue;
      Matrix T = Matrix::Zero();
      T.col(0) = Vector(1.0L, first * cone.mu[0], second * cone.mu[1]);
      int size = 1;
      if (first == 0 && cone.mu[0] > 0.0)
        T(1, size++) = 1.0L;
      if (second == 0 && cone.mu[1] > 0.0)
        T(2, size++) = 1.0L;
      Matrix reduced = T.transpose() * H * T;
      Vector rhs = -T.transpose() * c;
      // Pad with independent positive diagonal entries to keep storage fixed.
      for (int i = size; i < 3; ++i)
        reduced(i, i) = 1.0L;
      const Vector x = T * reduced.ldlt().solve(rhs);
      if (!certificate(H, c, x.cast<double>().cast<Real>(), cone))
        continue;
      const Real value = 0.5L * x.dot(H * x) + c.dot(x);
      if (value < bestValue) {
        bestValue = value;
        best = x;
      }
    }
  }
  return best;
}

LocalSolveResult solvePrepared(
    const Matrix& H, const Vector& c, const FrictionCone& cone)
{
  LocalSolveResult result;
  result.numQpSolves = 1;
  Vector x = Vector::Zero();
  if (c[0] >= support(c, cone)) {
    result.certified = true;
    return result;
  }
  x = H.ldlt().solve(-c);
  if (primalViolation(x, cone) <= 0.0L
      && certificate(H, c, x.cast<double>().cast<Real>(), cone)) {
    result.impulse = x.cast<double>();
    result.certified = true;
    return result;
  }
  if (cone.law == FrictionConeLaw::Box || cone.mu.minCoeff() == 0.0) {
    x = polyhedralQp(H, c, cone);
  } else {
    x = scanAngles(H, c, cone, 32, true);
    if (!certificate(H, c, x.cast<double>().cast<Real>(), cone)) {
      ++result.numLocalFallbacks;
      // Dense reference scan plus bisection. Increase the sampling density
      // for narrow minima at the randomized gate's highest condition numbers.
      for (int count : {720, 5760, 46080}) {
        x = scanAngles(H, c, cone, count, false);
        if (certificate(H, c, x.cast<double>().cast<Real>(), cone))
          break;
      }
    }
  }
  result.impulse = x.cast<double>();
  // Certify the returned double value, not only the extended precision iterate.
  result.certified = certificate(H, c, result.impulse.cast<Real>(), cone);
  return result;
}

} // namespace

LocalSolveResult solveConeQp(
    const Eigen::Matrix3d& H,
    const Eigen::Vector3d& c,
    const FrictionCone& cone)
{
  LocalSolveResult result;
  Matrix effective;
  if (!prepare(H, c, cone, effective, result.regularization))
    return result;
  const double regularization = result.regularization;
  result = solvePrepared(effective, c.cast<Real>(), cone);
  result.regularization = regularization;
  return result;
}

Eigen::Vector3d projectCone(
    const Eigen::Vector3d& impulse, const FrictionCone& cone)
{
  const auto result = solveConeQp(Eigen::Matrix3d::Identity(), -impulse, cone);
  return result.certified ? result.impulse
                          : Eigen::Vector3d::Constant(
                              std::numeric_limits<double>::quiet_NaN());
}

Eigen::Vector3d deSaxce(
    const Eigen::Vector3d& velocity, const FrictionCone& cone)
{
  Eigen::Vector3d result = velocity;
  result[0] += double(support(velocity.cast<Real>(), cone));
  return result;
}

double coneViolation(const Eigen::Vector3d& impulse, const FrictionCone& cone)
{
  if (!impulse.allFinite() || !valid(cone))
    return std::numeric_limits<double>::infinity();
  return double(primalViolation(impulse.cast<Real>(), cone));
}

double contactViolation(
    const Eigen::Vector3d& impulse,
    const Eigen::Vector3d& velocity,
    double maxDiagonal,
    const FrictionCone& cone,
    bool associated)
{
  if (!impulse.allFinite() || !velocity.allFinite() || !valid(cone)
      || !std::isfinite(maxDiagonal) || maxDiagonal <= 0.0)
    return std::numeric_limits<double>::infinity();
  const Eigen::Vector3d shifted
      = associated ? velocity : deSaxce(velocity, cone);
  return maxDiagonal
         * (impulse - projectCone(impulse - shifted / maxDiagonal, cone))
               .norm();
}

bool coneQpCertificate(
    const Eigen::Matrix3d& H,
    const Eigen::Vector3d& c,
    const Eigen::Vector3d& impulse,
    const FrictionCone& cone,
    double tolerance)
{
  return H.allFinite() && c.allFinite() && valid(cone)
         && std::isfinite(tolerance) && tolerance > 0.0
         && certificate(
             H.cast<Real>(),
             c.cast<Real>(),
             impulse.cast<Real>(),
             cone,
             tolerance);
}

LocalSolveResult solveExactContact(
    const Eigen::Matrix3d& H,
    const Eigen::Vector3d& c,
    const FrictionCone& cone,
    double normalShift)
{
  LocalSolveResult result;
  Matrix effective;
  if (!prepare(H, c, cone, effective, result.regularization)
      || !std::isfinite(normalShift))
    return result;
  const Vector q = c.cast<Real>();
  const auto finish = [&](const Vector& x, Real shift) {
    result.impulse = x.cast<double>();
    result.normalShift = double(shift);
    Vector shifted = q;
    shifted[0] += support(effective * x + q, cone);
    result.certified = certificate(effective, shifted, x, cone);
  };
  const auto evaluate = [&](Real shift, Vector& x, bool& certified) {
    Vector shifted = q;
    shifted[0] += shift;
    const auto qp = solvePrepared(effective, shifted, cone);
    result.numQpSolves += qp.numQpSolves;
    result.numLocalFallbacks += qp.numLocalFallbacks;
    x = qp.impulse.cast<Real>();
    certified = qp.certified;
    return support(effective * x + q, cone) - shift;
  };
  Real lo = 0.0L;
  Real hi = support(q, cone) + std::max(0.0L, -q[0]) + 1.0L;
  Vector x;
  bool certified = false;
  Real flo = evaluate(lo, x, certified);
  if (!certified)
    return result;
  const Real rootTolerance = 1e-12L * (1.0L + hi);
  if (std::abs(flo) <= rootTolerance) {
    finish(x, 0.0L);
    return result;
  }
  Real fhi = evaluate(hi, x, certified);
  if (!certified)
    return result;
  // An apex is an exact contact solution whenever the free normal velocity is
  // nonnegative; no tangential impulse can help an opening contact.
  if (q[0] >= 0.0L) {
    finish(Vector::Zero(), support(q, cone));
    return result;
  }
  for (int expand = 0; fhi >= 0.0L && expand < 32; ++expand) {
    hi *= 2.0L;
    fhi = evaluate(hi, x, certified);
    if (!certified)
      return result;
  }
  if (!(fhi < 0.0L))
    return result;
  if (normalShift > 0.0 && Real(normalShift) < hi) {
    const Real warm = normalShift;
    const Real fwarm = evaluate(warm, x, certified);
    if (!certified)
      return result;
    if (std::abs(fwarm) <= rootTolerance) {
      finish(x, warm);
      return result;
    }
    if (fwarm > 0.0L) {
      lo = warm;
      flo = fwarm;
    } else {
      hi = warm;
      fhi = fwarm;
    }
  }
  int lastSide = 0;
  for (int iteration = 0; iteration < 200; ++iteration) {
    Real shift = (lo * fhi - hi * flo) / (fhi - flo);
    if (!(shift > lo && shift < hi))
      shift = (lo + hi) / 2.0L;
    const Real value = evaluate(shift, x, certified);
    if (!certified)
      return result;
    if (std::abs(value) <= rootTolerance || hi - lo <= rootTolerance) {
      finish(x, shift);
      return result;
    }
    if (value > 0.0L) {
      lo = shift;
      flo = value;
      if (lastSide == 1)
        fhi *= 0.5L;
      lastSide = 1;
    } else {
      hi = shift;
      fhi = value;
      if (lastSide == -1)
        flo *= 0.5L;
      lastSide = -1;
    }
  }
  return result;
}

} // namespace dart::constraint::detail
