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

#include <dart/collision/native/narrow_phase/PrimitiveCcd.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <vector>

#include <cmath>

namespace dart::collision::native {

namespace {

constexpr double kEpsilon = 1e-12;

// Conservative advancement scaling s in (0, 1). Each step covers (1 - s) of the
// maximal provably-safe advance, which (a) keeps the iterate strictly short of
// contact and (b) gives geometric convergence so the loop terminates in O(1/s)
// iterations. s = 0.1 matches the additive-CCD inner loop (factor 0.9).
constexpr double kConservativeScale = 0.1;

//==============================================================================
// Closest-distance primitives (Ericson, Real-Time Collision Detection)
//==============================================================================

Eigen::Vector3d closestPointOnTriangle(
    const Eigen::Vector3d& p,
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c)
{
  const Eigen::Vector3d ab = b - a;
  const Eigen::Vector3d ac = c - a;
  const Eigen::Vector3d ap = p - a;

  const double d1 = ab.dot(ap);
  const double d2 = ac.dot(ap);
  if (d1 <= 0.0 && d2 <= 0.0) {
    return a;
  }

  const Eigen::Vector3d bp = p - b;
  const double d3 = ab.dot(bp);
  const double d4 = ac.dot(bp);
  if (d3 >= 0.0 && d4 <= d3) {
    return b;
  }

  const double vc = d1 * d4 - d3 * d2;
  if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
    const double v = d1 / (d1 - d3);
    return a + v * ab;
  }

  const Eigen::Vector3d cp = p - c;
  const double d5 = ab.dot(cp);
  const double d6 = ac.dot(cp);
  if (d6 >= 0.0 && d5 <= d6) {
    return c;
  }

  const double vb = d5 * d2 - d1 * d6;
  if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
    const double w = d2 / (d2 - d6);
    return a + w * ac;
  }

  const double va = d3 * d6 - d5 * d4;
  if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0) {
    const double w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
    return b + w * (c - b);
  }

  const double denom = 1.0 / (va + vb + vc);
  const double v = vb * denom;
  const double w = vc * denom;
  return a + ab * v + ac * w;
}

double distancePointTriangle(
    const Eigen::Vector3d& p,
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c)
{
  return (p - closestPointOnTriangle(p, a, b, c)).norm();
}

double distanceSegmentSegment(
    const Eigen::Vector3d& p1,
    const Eigen::Vector3d& q1,
    const Eigen::Vector3d& p2,
    const Eigen::Vector3d& q2,
    double& sOut,
    double& tOut)
{
  const Eigen::Vector3d d1 = q1 - p1;
  const Eigen::Vector3d d2 = q2 - p2;
  const Eigen::Vector3d r = p1 - p2;
  const double a = d1.dot(d1);
  const double e = d2.dot(d2);
  const double f = d2.dot(r);

  double s = 0.0;
  double t = 0.0;

  if (a <= kEpsilon && e <= kEpsilon) {
    s = 0.0;
    t = 0.0;
  } else if (a <= kEpsilon) {
    s = 0.0;
    t = std::clamp(f / e, 0.0, 1.0);
  } else {
    const double c = d1.dot(r);
    if (e <= kEpsilon) {
      t = 0.0;
      s = std::clamp(-c / a, 0.0, 1.0);
    } else {
      const double b = d1.dot(d2);
      const double denom = a * e - b * b;
      if (denom > kEpsilon) {
        s = std::clamp((b * f - c * e) / denom, 0.0, 1.0);
      } else {
        s = 0.0;
      }
      t = (b * s + f) / e;
      if (t < 0.0) {
        t = 0.0;
        s = std::clamp(-c / a, 0.0, 1.0);
      } else if (t > 1.0) {
        t = 1.0;
        s = std::clamp((b - c) / a, 0.0, 1.0);
      }
    }
  }

  sOut = s;
  tOut = t;
  const Eigen::Vector3d c1 = p1 + d1 * s;
  const Eigen::Vector3d c2 = p2 + d2 * t;
  return (c1 - c2).norm();
}

double distanceSegmentSegment(
    const Eigen::Vector3d& p1,
    const Eigen::Vector3d& q1,
    const Eigen::Vector3d& p2,
    const Eigen::Vector3d& q2)
{
  double s = 0.0;
  double t = 0.0;
  return distanceSegmentSegment(p1, q1, p2, q2, s, t);
}

//==============================================================================
// Additive conservative advancement (ACCD)
//==============================================================================

// Generic ACCD loop. lp is the additive relative-velocity bound on the
// (already mean-centered) trajectories; distAt(t) returns the unsigned distance
// between the two primitives at time t along those trajectories.
template <typename DistFn>
bool accdAdvance(
    double lp,
    const CcdOption& option,
    DistFn&& distAt,
    CcdPrimitiveResult& result)
{
  result.clear();

  const double minSeparation = std::max(0.0, option.minSeparation);
  double d = distAt(0.0);

  const double gap0 = d - minSeparation;
  if (gap0 <= 0.0) {
    result.hit = true;
    result.status = CcdPrimitiveStatus::Hit;
    result.timeOfImpact = 0.0;
    return true;
  }

  if (lp <= kEpsilon) {
    // No relative motion: the clearance is constant, so no new contact forms.
    result.status = CcdPrimitiveStatus::Miss;
    return false;
  }

  const double stepFactor = 1.0 - kConservativeScale;
  // Absolute contact band. A hit means the primitives close to within
  // minSeparation, so the threshold must be an absolute distance -- scaling it
  // by the initial gap would report a hit for a non-colliding near-miss that
  // merely ends close relative to a large starting separation.
  const double convergeAbs = std::max(option.tolerance, kEpsilon);
  const int maxIter = std::max(1, option.maxIterations);

  double t = 0.0;
  for (int iter = 0; iter < maxIter; ++iter) {
    const double clearance = d - minSeparation;
    if (clearance <= convergeAbs) {
      result.hit = true;
      result.status = CcdPrimitiveStatus::Hit;
      result.timeOfImpact = t;
      return true;
    }

    // The distance shrinks no faster than the additive velocity bound lp, so
    // the earliest the gap can reach contact is t + clearance / lp. If that is
    // at or beyond the end of the step, the only contact possible in the
    // inclusive interval [0, 1] is exactly at the final endpoint, so check
    // t = 1 directly before giving up (an end-of-step contact is still a hit).
    // (This also bounds the loop: each iteration adds at least
    // stepFactor * convergeAbs / lp to t, so t reaches this guard or the
    // convergence threshold in finite steps for any well-posed input.)
    if (t + clearance / lp >= 1.0) {
      if (distAt(1.0) - minSeparation <= convergeAbs) {
        result.hit = true;
        result.status = CcdPrimitiveStatus::Hit;
        result.timeOfImpact = 1.0;
        return true;
      }
      result.status = CcdPrimitiveStatus::Miss;
      return false;
    }

    t += stepFactor * clearance / lp;
    d = distAt(t);
  }

  // Exhausted the iteration budget without converging to contact or proving
  // none can occur -- a slow, near-tangential approach that has not been shown
  // to close to minSeparation in [0, 1].
  result.status = CcdPrimitiveStatus::Indeterminate;
  return false;
}

//==============================================================================
// Exact coplanarity cubic (validation only)
//==============================================================================

// Coplanarity cubic f(t) = (z0 + t dz) . ((x0 + t dx) x (y0 + t dy)) for the
// four moving points, returned as coefficients [c0, c1, c2, c3] (ascending).
std::array<double, 4> coplanarityCubic(
    const Eigen::Vector3d& x0,
    const Eigen::Vector3d& dx,
    const Eigen::Vector3d& y0,
    const Eigen::Vector3d& dy,
    const Eigen::Vector3d& z0,
    const Eigen::Vector3d& dz)
{
  const Eigen::Vector3d c0 = x0.cross(y0);
  const Eigen::Vector3d c1 = x0.cross(dy) + dx.cross(y0);
  const Eigen::Vector3d c2 = dx.cross(dy);

  return {
      z0.dot(c0), z0.dot(c1) + dz.dot(c0), z0.dot(c2) + dz.dot(c1), dz.dot(c2)};
}

double evalPoly(const std::array<double, 4>& coef, double t)
{
  return ((coef[3] * t + coef[2]) * t + coef[1]) * t + coef[0];
}

// Real roots of a*t^2 + b*t + c in (0, 1), appended to out.
void quadraticRootsInUnit(
    double a, double b, double c, std::vector<double>& out)
{
  if (std::abs(a) <= kEpsilon) {
    if (std::abs(b) > kEpsilon) {
      const double r = -c / b;
      if (r > 0.0 && r < 1.0) {
        out.push_back(r);
      }
    }
    return;
  }
  const double disc = b * b - 4.0 * a * c;
  if (disc < 0.0) {
    return;
  }
  const double sq = std::sqrt(disc);
  for (const double r : {(-b - sq) / (2.0 * a), (-b + sq) / (2.0 * a)}) {
    if (r > 0.0 && r < 1.0) {
      out.push_back(r);
    }
  }
}

double refineRootBisection(
    const std::array<double, 4>& coef, double lo, double hi)
{
  double flo = evalPoly(coef, lo);
  for (int i = 0; i < 80; ++i) {
    const double mid = 0.5 * (lo + hi);
    const double fmid = evalPoly(coef, mid);
    if ((flo < 0.0) == (fmid < 0.0)) {
      lo = mid;
      flo = fmid;
    } else {
      hi = mid;
    }
  }
  return 0.5 * (lo + hi);
}

// All real roots of the cubic in [0, 1], sorted ascending.
std::vector<double> cubicRootsInUnit(const std::array<double, 4>& coef)
{
  std::vector<double> breaks;
  breaks.push_back(0.0);
  // Critical points partition [0,1] into monotonic intervals.
  quadraticRootsInUnit(3.0 * coef[3], 2.0 * coef[2], coef[1], breaks);
  breaks.push_back(1.0);
  std::sort(breaks.begin(), breaks.end());

  std::vector<double> roots;
  for (std::size_t i = 0; i + 1 < breaks.size(); ++i) {
    const double lo = breaks[i];
    const double hi = breaks[i + 1];
    if (hi - lo <= kEpsilon) {
      continue;
    }
    const double flo = evalPoly(coef, lo);
    const double fhi = evalPoly(coef, hi);
    if (std::abs(flo) <= kEpsilon) {
      roots.push_back(lo);
    }
    if ((flo < 0.0) != (fhi < 0.0)) {
      roots.push_back(refineRootBisection(coef, lo, hi));
    }
  }
  const double fEnd = evalPoly(coef, 1.0);
  if (std::abs(fEnd) <= kEpsilon) {
    roots.push_back(1.0);
  }

  std::sort(roots.begin(), roots.end());
  roots.erase(
      std::unique(
          roots.begin(),
          roots.end(),
          [](double x, double y) { return std::abs(x - y) <= 1e-9; }),
      roots.end());
  return roots;
}

bool pointInsideTriangle(
    const Eigen::Vector3d& p,
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c,
    double slack)
{
  const Eigen::Vector3d v0 = b - a;
  const Eigen::Vector3d v1 = c - a;
  const Eigen::Vector3d v2 = p - a;
  const double d00 = v0.dot(v0);
  const double d01 = v0.dot(v1);
  const double d11 = v1.dot(v1);
  const double d20 = v2.dot(v0);
  const double d21 = v2.dot(v1);
  const double denom = d00 * d11 - d01 * d01;
  if (std::abs(denom) <= kEpsilon) {
    return false;
  }
  const double v = (d11 * d20 - d01 * d21) / denom;
  const double w = (d00 * d21 - d01 * d20) / denom;
  const double u = 1.0 - v - w;
  return u >= -slack && v >= -slack && w >= -slack;
}

} // namespace

//==============================================================================
// Public ACCD queries
//==============================================================================

bool pointTriangleCcd(
    const Eigen::Vector3d& pStart,
    const Eigen::Vector3d& pEnd,
    const Eigen::Vector3d& aStart,
    const Eigen::Vector3d& aEnd,
    const Eigen::Vector3d& bStart,
    const Eigen::Vector3d& bEnd,
    const Eigen::Vector3d& cStart,
    const Eigen::Vector3d& cEnd,
    const CcdOption& option,
    CcdPrimitiveResult& result)
{
  Eigen::Vector3d dp = pEnd - pStart;
  Eigen::Vector3d da = aEnd - aStart;
  Eigen::Vector3d db = bEnd - bStart;
  Eigen::Vector3d dc = cEnd - cStart;

  // Mean-center the displacements: collision depends only on relative motion,
  // so this is exact and shrinks the velocity bound (additive CCD).
  const Eigen::Vector3d mean = 0.25 * (dp + da + db + dc);
  dp -= mean;
  da -= mean;
  db -= mean;
  dc -= mean;

  const double lp = dp.norm() + std::max({da.norm(), db.norm(), dc.norm()});

  const auto distAt = [&](double t) {
    return distancePointTriangle(
        pStart + t * dp, aStart + t * da, bStart + t * db, cStart + t * dc);
  };

  return accdAdvance(lp, option, distAt, result);
}

bool edgeEdgeCcd(
    const Eigen::Vector3d& aStart,
    const Eigen::Vector3d& aEnd,
    const Eigen::Vector3d& bStart,
    const Eigen::Vector3d& bEnd,
    const Eigen::Vector3d& cStart,
    const Eigen::Vector3d& cEnd,
    const Eigen::Vector3d& dStart,
    const Eigen::Vector3d& dEnd,
    const CcdOption& option,
    CcdPrimitiveResult& result)
{
  Eigen::Vector3d da = aEnd - aStart;
  Eigen::Vector3d db = bEnd - bStart;
  Eigen::Vector3d dc = cEnd - cStart;
  Eigen::Vector3d dd = dEnd - dStart;

  const Eigen::Vector3d mean = 0.25 * (da + db + dc + dd);
  da -= mean;
  db -= mean;
  dc -= mean;
  dd -= mean;

  const double lp
      = std::max(da.norm(), db.norm()) + std::max(dc.norm(), dd.norm());

  const auto distAt = [&](double t) {
    return distanceSegmentSegment(
        aStart + t * da, bStart + t * db, cStart + t * dc, dStart + t * dd);
  };

  return accdAdvance(lp, option, distAt, result);
}

//==============================================================================
// Public exact coplanarity-cubic queries (validation only)
//==============================================================================

bool pointTriangleCcdExact(
    const Eigen::Vector3d& pStart,
    const Eigen::Vector3d& pEnd,
    const Eigen::Vector3d& aStart,
    const Eigen::Vector3d& aEnd,
    const Eigen::Vector3d& bStart,
    const Eigen::Vector3d& bEnd,
    const Eigen::Vector3d& cStart,
    const Eigen::Vector3d& cEnd,
    const CcdOption& option,
    CcdPrimitiveResult& result)
{
  result.clear();

  const Eigen::Vector3d dp = pEnd - pStart;
  const Eigen::Vector3d da = aEnd - aStart;
  const Eigen::Vector3d db = bEnd - bStart;
  const Eigen::Vector3d dc = cEnd - cStart;

  const Eigen::Vector3d x0 = bStart - aStart;
  const Eigen::Vector3d dx = db - da;
  const Eigen::Vector3d y0 = cStart - aStart;
  const Eigen::Vector3d dy = dc - da;
  const Eigen::Vector3d z0 = pStart - aStart;
  const Eigen::Vector3d dz = dp - da;

  const auto coef = coplanarityCubic(x0, dx, y0, dy, z0, dz);
  const double slack = std::max(option.tolerance, 1e-9);

  for (const double t : cubicRootsInUnit(coef)) {
    const Eigen::Vector3d p = pStart + t * dp;
    const Eigen::Vector3d a = aStart + t * da;
    const Eigen::Vector3d b = bStart + t * db;
    const Eigen::Vector3d c = cStart + t * dc;
    if (pointInsideTriangle(p, a, b, c, slack)) {
      result.hit = true;
      result.status = CcdPrimitiveStatus::Hit;
      result.timeOfImpact = std::clamp(t, 0.0, 1.0);
      return true;
    }
  }
  result.status = CcdPrimitiveStatus::Miss;
  return false;
}

bool edgeEdgeCcdExact(
    const Eigen::Vector3d& aStart,
    const Eigen::Vector3d& aEnd,
    const Eigen::Vector3d& bStart,
    const Eigen::Vector3d& bEnd,
    const Eigen::Vector3d& cStart,
    const Eigen::Vector3d& cEnd,
    const Eigen::Vector3d& dStart,
    const Eigen::Vector3d& dEnd,
    const CcdOption& option,
    CcdPrimitiveResult& result)
{
  result.clear();

  const Eigen::Vector3d da = aEnd - aStart;
  const Eigen::Vector3d db = bEnd - bStart;
  const Eigen::Vector3d dc = cEnd - cStart;
  const Eigen::Vector3d dd = dEnd - dStart;

  const Eigen::Vector3d x0 = bStart - aStart;
  const Eigen::Vector3d dx = db - da;
  const Eigen::Vector3d y0 = dStart - cStart;
  const Eigen::Vector3d dy = dd - dc;
  const Eigen::Vector3d z0 = cStart - aStart;
  const Eigen::Vector3d dz = dc - da;

  const auto coef = coplanarityCubic(x0, dx, y0, dy, z0, dz);
  const double slack = std::max(option.tolerance, 1e-9);

  // Validation distance scale: a fraction of the edge lengths at the start.
  const double edgeScale = std::max({x0.norm(), y0.norm(), 1.0});
  const double distTol = slack * edgeScale + 1e-9;

  const auto contactAt = [&](double t) {
    const Eigen::Vector3d a = aStart + t * da;
    const Eigen::Vector3d b = bStart + t * db;
    const Eigen::Vector3d c = cStart + t * dc;
    const Eigen::Vector3d d = dStart + t * dd;
    double s = 0.0;
    double u = 0.0;
    const double dist = distanceSegmentSegment(a, b, c, d, s, u);
    return dist <= distTol && s >= -slack && s <= 1.0 + slack && u >= -slack
           && u <= 1.0 + slack;
  };

  std::vector<double> candidates = cubicRootsInUnit(coef);

  // Degenerate case: when the coplanarity cubic is identically zero the four
  // points stay coplanar for the whole step, so it yields no interior roots
  // even though the segments can still cross mid-step. Bracket the
  // segment-segment distance over [0, 1] and add interior crossing times so
  // they get checked.
  const double coefMag = std::max(
      {std::abs(coef[0]),
       std::abs(coef[1]),
       std::abs(coef[2]),
       std::abs(coef[3])});
  if (coefMag <= 1e-10 * edgeScale * edgeScale * edgeScale) {
    const auto segGap = [&](double t) {
      double s = 0.0;
      double u = 0.0;
      return distanceSegmentSegment(
                 aStart + t * da,
                 bStart + t * db,
                 cStart + t * dc,
                 dStart + t * dd,
                 s,
                 u)
             - distTol;
    };
    constexpr int kSamples = 256;
    double prevGap = segGap(0.0);
    for (int i = 1; i <= kSamples; ++i) {
      double hi = static_cast<double>(i) / static_cast<double>(kSamples);
      const double curGap = segGap(hi);
      if (prevGap > 0.0 && curGap <= 0.0) {
        // Entering contact between (i-1)/N and i/N; bisect for the entry time.
        double lo = static_cast<double>(i - 1) / static_cast<double>(kSamples);
        for (int b = 0; b < 40; ++b) {
          const double mid = 0.5 * (lo + hi);
          if (segGap(mid) > 0.0) {
            lo = mid;
          } else {
            hi = mid;
          }
        }
        candidates.push_back(hi);
      }
      prevGap = curGap;
    }
    std::sort(candidates.begin(), candidates.end());
  }

  for (const double t : candidates) {
    if (contactAt(t)) {
      result.hit = true;
      result.status = CcdPrimitiveStatus::Hit;
      result.timeOfImpact = std::clamp(t, 0.0, 1.0);
      return true;
    }
  }
  result.status = CcdPrimitiveStatus::Miss;
  return false;
}

} // namespace dart::collision::native
