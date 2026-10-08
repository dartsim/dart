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
#include <Eigen/LU>

#include <algorithm>
#include <limits>

#include <cmath>

namespace dart::constraint::detail {
namespace {

using Real = double;
using Vector = Eigen::Matrix<Real, 3, 1>;
using Matrix = Eigen::Matrix<Real, 3, 3>;
constexpr Real kPi = 3.141592653589793238462643383279502884;
constexpr Real kTolerance = 1e-10;

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
  Real t[2] = {0.0, 0.0};
  Real violation = std::max(0.0, -v[0]);
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
  const Real hScale = H.cwiseAbs().maxCoeff();
  const Real cScale = c.cwiseAbs().maxCoeff();
  const Real dataScale = std::max(hScale, cScale);
  Matrix normalizedH = H;
  Vector normalizedC = c;
  if (dataScale > 0.0) {
    normalizedH /= dataScale;
    normalizedC /= dataScale;
  }
  // Scale impulses too, so complementarity cannot underflow or overflow.
  const Real impulseScale
      = std::max(x.cwiseAbs().maxCoeff(), hScale > 0.0 ? cScale / hScale : 0.0);
  if (!std::isfinite(impulseScale))
    return false;
  Vector normalizedX = x;
  if (impulseScale > 0.0) {
    normalizedX /= impulseScale;
    if (hScale > 0.0)
      normalizedC /= impulseScale;
  }
  const Vector v = normalizedH * normalizedX + normalizedC;
  const Vector magnitude = normalizedH.cwiseAbs() * normalizedX.cwiseAbs()
                           + normalizedC.cwiseAbs();
  const Real dualScale = magnitude[0] + support(magnitude, cone);
  const Real dotScale = normalizedX.cwiseAbs().dot(magnitude);
  return v.allFinite() && magnitude.allFinite()
         && primalViolation(normalizedX, cone) <= tolerance
         && support(v, cone) - v[0] <= tolerance * dualScale
         && std::abs(normalizedX.dot(v)) <= tolerance * dotScale;
}

bool positiveSemidefinite(const Matrix& H)
{
  // H is normalized to a largest entry of 1, so an absolute tolerance works.
  constexpr Real tolerance = 1e-12;
  for (int i = 0; i < 3; ++i) {
    if (H(i, i) < -tolerance)
      return false;
    const int j = (i + 1) % 3;
    if (H(i, i) * H(j, j) - H(i, j) * H(j, i) < -tolerance)
      return false;
  }
  return H.determinant() >= -tolerance;
}

bool prepare(
    const Eigen::Matrix3d& input,
    const Eigen::Vector3d& c,
    const FrictionCone& cone,
    Matrix& H,
    double& regularization,
    bool regularize = true)
{
  if (!input.allFinite() || !c.allFinite() || !valid(cone))
    return false;
  const double scale = input.cwiseAbs().maxCoeff();
  H = input;
  if (scale > 0.0)
    H /= scale;
  if ((H - H.transpose()).cwiseAbs().maxCoeff() > 1e-12)
    return false;
  H = (0.5 * H + 0.5 * H.transpose()).eval();
  Eigen::LDLT<Matrix> ldlt(H);
  // LDLT pivots cannot tell a singular PSD block (zero pivots, NumericalIssue)
  // from an indefinite one with a zero diagonal, so require every principal
  // minor of the normalized block to be nonnegative up to rounding.
  if (!positiveSemidefinite(H))
    return false;
  H = (0.5 * input + 0.5 * input.transpose()).cast<Real>();
  if (!regularize)
    return true;
  const Real trace = H.trace();
  if (scale * ldlt.vectorD().minCoeff() <= 1e-18 * std::max(1.0, trace)) {
    regularization = double(1e-12 * (trace > 0.0 ? trace : 1.0));
    H.diagonal().array() += Real(regularization);
    ldlt.compute(H);
  }
  return ldlt.info() == Eigen::Success && ldlt.vectorD().minCoeff() > 0.0;
}

struct AnglePoint
{
  Real value = 0.0;
  Real derivative = 0.0;
  Vector impulse = Vector::Zero();
};

AnglePoint anglePoint(
    const Matrix& H, const Vector& c, const FrictionCone& cone, Real theta)
{
  const Real ct = std::cos(theta), st = std::sin(theta);
  const Real m1 = cone.mu[0], m2 = cone.mu[1];
  const Vector a(1.0, m1 * ct, m2 * st);
  const Vector da(0.0, -m1 * st, m2 * ct);
  const Vector Ha = H * a;
  const Real p = c.dot(a), dp = c.dot(da);
  const Real h = a.dot(Ha), dh = 2.0 * da.dot(Ha);
  AnglePoint point;
  // The derivative has the sign of 2 p' h - p h' whenever p < 0.
  point.derivative = 2.0 * dp * h - p * dh;
  if (p < 0.0 && h > 0.0) {
    point.value = -p * p / (2.0 * h);
    point.impulse = (-p / h) * a;
  }
  return point;
}

Vector refineAngle(
    const Matrix& H,
    const Vector& c,
    const FrictionCone& cone,
    Real theta,
    Real step)
{
  Real lo = theta - step, hi = theta + step;
  for (int iteration = 0; iteration < 90; ++iteration) {
    const auto point = anglePoint(H, c, cone, theta);
    if (point.derivative < 0.0)
      lo = theta;
    else
      hi = theta;
    const Real next = (lo + hi) / 2.0;
    if (next == theta)
      break;
    theta = next;
  }
  return anglePoint(H, c, cone, theta).impulse;
}

Vector scanAngles(
    const Matrix& H, const Vector& c, const FrictionCone& cone, int count)
{
  const Real step = 2.0 * kPi / count;
  Vector best = Vector::Zero();
  Real bestValue = 0.0;
  Real before = anglePoint(H, c, cone, -step).value;
  Real here = anglePoint(H, c, cone, 0.0).value;
  // Refine every sampled local minimum, not just the lowest sample. An
  // uncertified stationary point cannot replace a certified global optimum.
  for (int k = 0; k < count; ++k) {
    const Real next = anglePoint(H, c, cone, (k + 1) * step).value;
    if (here < 0.0 && here <= before && here <= next) {
      const Vector x = refineAngle(H, c, cone, k * step, step);
      const Real value = 0.5 * x.dot(H * x) + c.dot(x);
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

Vector secularBoundary(
    const Matrix& H, const Vector& c, const FrictionCone& cone)
{
  // In y coordinates x = D*y, the boundary is y_n^2 = |y_t|^2 and
  // (D*H*D - nu*diag(1,-1,-1))*y = -D*c. Only C + nu*I is
  // inverted below: no matrix containing 1/mu^2 is formed or inverted.
  const Vector d(1.0, cone.mu[0], cone.mu[1]);
  const Matrix B = d.asDiagonal() * H * d.asDiagonal();
  const Vector g = d.cwiseProduct(c);
  const Eigen::Vector2d b = B.bottomLeftCorner<2, 1>();
  struct Point
  {
    Real delta, numerator, value, slope;
    Eigen::Vector2d u, w, tangent;
  };
  const auto evaluate = [&](Real nu) {
    const Real a = B(1, 1) + nu, e = B(2, 2) + nu, f = B(1, 2);
    Eigen::Matrix2d inverse;
    inverse << e, -f, -f, a;
    inverse /= a * e - f * f;
    Point p;
    p.u = inverse * g.tail<2>();
    p.w = inverse * b;
    p.delta = B(0, 0) - nu - b.dot(p.w);
    p.numerator = -g[0] + b.dot(p.u);
    // y_n = numerator/delta, y_t = -tangent/delta. This secular
    // residual avoids division by delta at the generalized eigenvalue.
    p.tangent = p.delta * p.u + p.numerator * p.w;
    const Real length = p.tangent.norm();
    p.value = length - std::abs(p.numerator);
    const Real dd = p.w.squaredNorm() - 1.0;
    const Real dn = -p.w.dot(p.u);
    const Eigen::Vector2d dt = dd * p.u - p.delta * (inverse * p.u) + dn * p.w
                               - p.numerator * (inverse * p.w);
    p.slope = length > 0.0 ? p.tangent.dot(dt) / length
                                 - std::copysign(1.0, p.numerator) * dn
                           : 0.0;
    return p;
  };
  const auto onRay = [&](const Eigen::Vector2d& tangent) -> Vector {
    const Vector ray(1.0, d[1] * tangent[0], d[2] * tangent[1]);
    return (-c.dot(ray) / ray.dot(H * ray)) * ray;
  };

  // The Schur complement delta has one positive zero nu_plus. It is
  // concave, so Newton from B_nn approaches this pole from above.
  Real lo = 0.0, hi = B(0, 0), pole = hi;
  for (int i = 0; i < 64; ++i) {
    const auto p = evaluate(pole);
    if (std::abs(p.delta)
        <= 8.0 * std::numeric_limits<Real>::epsilon() * B(0, 0))
      break;
    if (p.delta > 0.0)
      lo = pole;
    else
      hi = pole;
    Real next = pole - p.delta / (p.w.squaredNorm() - 1.0);
    if (!(next > lo && next < hi))
      next = (lo + hi) / 2.0;
    if (next == pole)
      break;
    pole = next;
  }
  const auto atPole = evaluate(pole);
  if (std::abs(atPole.numerator)
      <= 8.0 * std::numeric_limits<Real>::epsilon() * g.norm()) {
    // At the singular hard case, solve the cone equation along the null
    // direction (1,-w), rather than dividing by the zero Schur complement.
    const Real a = 1.0 - atPole.w.squaredNorm();
    const Real dot = atPole.w.dot(atPole.u);
    const Real root = std::sqrt(dot * dot + a * atPole.u.squaredNorm());
    const Real normal
        = dot < 0.0 ? atPole.u.squaredNorm() / (root - dot) : (dot + root) / a;
    const Eigen::Vector2d tangent = -atPole.u - normal * atPole.w;
    return onRay(tangent.normalized());
  }
  const bool lower = atPole.numerator > 0.0;
  // A negative pole numerator puts the positive-nappe solution ABOVE
  // nu_plus; the root below it then lies on the negative nappe.
  lo = lower ? 0.0 : pole;
  hi = lower ? pole : 2.0 * pole;
  if (!lower) {
    for (int i = 0; i < 64 && evaluate(hi).value <= 0.0; ++i)
      hi *= 2.0;
    if (!(evaluate(hi).value > 0.0))
      return Vector::Zero();
  }
  Real nu = (lo + hi) / 2.0;
  for (int i = 0; i < 64; ++i) {
    const auto p = evaluate(nu);
    if (std::abs(p.value) <= 8.0 * std::numeric_limits<Real>::epsilon()
                                 * (p.tangent.norm() + std::abs(p.numerator)))
      break;
    if ((p.value > 0.0) == lower)
      lo = nu;
    else
      hi = nu;
    Real next = nu - p.value / p.slope;
    if (!(next > lo && next < hi))
      next = (lo + hi) / 2.0;
    if (next == nu)
      break;
    nu = next;
  }
  const auto p = evaluate(nu);
  return onRay((lower ? -1.0 : 1.0) * p.tangent.normalized());
}

Vector polyhedralQp(const Matrix& H, const Vector& c, const FrictionCone& cone)
{
  Vector best = Vector::Zero();
  Real bestValue = 0.0;
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
      T.col(0) = Vector(1.0, first * cone.mu[0], second * cone.mu[1]);
      int size = 1;
      if (first == 0 && cone.mu[0] > 0.0)
        T(1, size++) = 1.0;
      if (second == 0 && cone.mu[1] > 0.0)
        T(2, size++) = 1.0;
      Matrix reduced = T.transpose() * H * T;
      Vector rhs = -T.transpose() * c;
      // Pad with independent positive diagonal entries to keep storage fixed.
      for (int i = size; i < 3; ++i)
        reduced(i, i) = 1.0;
      const Vector x = T * reduced.ldlt().solve(rhs);
      if (!certificate(H, c, x.cast<double>().cast<Real>(), cone))
        continue;
      const Real value = 0.5 * x.dot(H * x) + c.dot(x);
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
  if (primalViolation(x, cone) <= 0.0
      && certificate(H, c, x.cast<double>().cast<Real>(), cone)) {
    result.impulse = x.cast<double>();
    result.certified = true;
    return result;
  }
  if (cone.law == FrictionConeLaw::Box || cone.mu.minCoeff() == 0.0) {
    x = polyhedralQp(H, c, cone);
  } else {
    x = secularBoundary(H, c, cone);
    if (!certificate(H, c, x.cast<double>().cast<Real>(), cone)) {
      ++result.numLocalFallbacks;
      // Dense reference scan plus bisection. Increase the sampling density
      // for narrow minima at the randomized gate's highest condition numbers.
      for (int count : {720, 5760, 46080}) {
        x = scanAngles(H, c, cone, count);
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
  Matrix checked;
  double regularization = 0.0;
  return std::isfinite(tolerance) && tolerance > 0.0
         && prepare(H, c, cone, checked, regularization, false)
         && certificate(
             checked, c.cast<Real>(), impulse.cast<Real>(), cone, tolerance);
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
  // Every exit certifies the contact itself: a scaled anisotropic block can
  // meet the root tolerance before the contact certificate passes.
  const auto finish = [&](const Vector& x, Real shift) {
    result.impulse = x.cast<double>();
    result.normalShift = double(shift);
    Vector shifted = q;
    shifted[0] += support(effective * x + q, cone);
    result.certified = certificate(effective, shifted, x, cone);
    return result.certified;
  };
  // An apex is an exact contact solution whenever the free normal velocity is
  // nonnegative; no tangential impulse can help an opening contact.
  if (q[0] >= 0.0) {
    finish(Vector::Zero(), support(q, cone));
    return result;
  }
  Real lastShift = 0.0;
  const auto evaluate = [&](Real shift, Vector& x, bool& certified) {
    lastShift = shift;
    Vector shifted = q;
    shifted[0] += shift;
    const auto qp = solvePrepared(effective, shifted, cone);
    result.numQpSolves += qp.numQpSolves;
    result.numLocalFallbacks += qp.numLocalFallbacks;
    x = qp.impulse.cast<Real>();
    certified = qp.certified;
    return support(effective * x + q, cone) - shift;
  };
  Real lo = 0.0;
  Real hi = support(q, cone) + std::max(0.0, -q[0]) + 1.0;
  Vector x;
  bool certified = false;
  Real flo = evaluate(lo, x, certified);
  if (!certified)
    return result;
  const Real rootTolerance = 1e-12 * (1.0 + hi);
  if (std::abs(flo) <= rootTolerance && finish(x, 0.0))
    return result;
  Real fhi = evaluate(hi, x, certified);
  if (!certified)
    return result;
  for (int expand = 0; fhi >= 0.0 && expand < 32; ++expand) {
    hi *= 2.0;
    fhi = evaluate(hi, x, certified);
    if (!certified)
      return result;
  }
  if (!(fhi < 0.0))
    return result;
  if (normalShift > 0.0 && Real(normalShift) < hi) {
    const Real warm = normalShift;
    const Real fwarm = evaluate(warm, x, certified);
    if (!certified)
      return result;
    if (std::abs(fwarm) <= rootTolerance && finish(x, warm))
      return result;
    if (fwarm > 0.0) {
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
    if (!(shift > lo && shift < hi)) {
      shift = (lo + hi) / 2.0;
      if (!(shift > lo && shift < hi))
        break;
    }
    const Real value = evaluate(shift, x, certified);
    if (!certified)
      return result;
    if ((std::abs(value) <= rootTolerance || hi - lo <= rootTolerance)
        && finish(x, shift))
      return result;
    if (value > 0.0) {
      lo = shift;
      flo = value;
      if (lastSide == 1)
        fhi *= 0.5;
      lastSide = 1;
    } else {
      hi = shift;
      fhi = value;
      if (lastSide == -1)
        flo *= 0.5;
      lastSide = -1;
    }
  }
  finish(x, lastShift);
  return result;
}

} // namespace dart::constraint::detail
