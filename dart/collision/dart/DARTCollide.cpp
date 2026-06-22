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

#include "dart/collision/dart/DARTCollide.hpp"

#include "dart/collision/CollisionObject.hpp"
#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/BoxShape.hpp"
#include "dart/dynamics/CylinderShape.hpp"
#include "dart/dynamics/EllipsoidShape.hpp"
#include "dart/dynamics/PlaneShape.hpp"
#include "dart/dynamics/SphereShape.hpp"
#include "dart/math/Helpers.hpp"

#include <algorithm>
#include <array>
#include <limits>
#include <memory>

#include <cmath>

namespace dart {
namespace collision {

// point : world coordinate vector
// normal : normal vector from right to left 0 <- 1
// penetration : real positive means penetration

#define DART_COLLISION_EPS 1E-6

typedef double dVector3[4];
typedef double dVector3[4];
typedef double dVector4[4];
typedef double dMatrix3[12];
typedef double dMatrix4[16];
typedef double dMatrix6[48];
typedef double dQuaternion[4];

inline void convVector(const Eigen::Vector3d& p0, dVector3& p1)
{
  p1[0] = p0[0];
  p1[1] = p0[1];
  p1[2] = p0[2];
}

inline void convMatrix(const Eigen::Isometry3d& T0, dMatrix3& R0)
{
  R0[0] = T0(0, 0);
  R0[1] = T0(0, 1);
  R0[2] = T0(0, 2);
  R0[3] = T0(0, 3);
  R0[4] = T0(1, 0);
  R0[5] = T0(1, 1);
  R0[6] = T0(1, 2);
  R0[7] = T0(1, 3);
  R0[8] = T0(2, 0);
  R0[9] = T0(2, 1);
  R0[10] = T0(2, 2);
  R0[11] = T0(2, 3);
}

struct dContactGeom
{
  dVector3 pos;
  double depth;
};

inline double Inner(const double* a, const double* b)
{
  return ((a)[0] * (b)[0] + (a)[1] * (b)[1] + (a)[2] * (b)[2]);
}

inline double Inner14(const double* a, const double* b)
{
  return ((a)[0] * (b)[0] + (a)[1] * (b)[4] + (a)[2] * (b)[8]);
}

inline double Inner41(const double* a, const double* b)
{
  return ((a)[0] * (b)[0] + (a)[4] * (b)[1] + (a)[8] * (b)[2]);
}

inline double Inner44(const double* a, const double* b)
{
  return ((a)[0] * (b)[0] + (a)[4] * (b)[4] + (a)[8] * (b)[8]);
}

#define dMULTIPLYOP0_331(A, op, B, C)                                          \
  (A)[0] op Inner((B), (C));                                                   \
  (A)[1] op Inner((B + 4), (C));                                               \
  (A)[2] op Inner((B + 8), (C));

#define dMULTIPLYOP1_331(A, op, B, C)                                          \
  (A)[0] op Inner41((B), (C));                                                 \
  (A)[1] op Inner41((B + 1), (C));                                             \
  (A)[2] op Inner41((B + 2), (C));

inline void dMULTIPLY0_331(double* A, const double* B, const double* C)
{
  dMULTIPLYOP0_331(A, =, B, C)
}

inline void dMULTIPLY1_331(double* A, const double* B, const double* C)
{
  dMULTIPLYOP1_331(A, =, B, C)
}

#define dRecip(x) (1.0 / (x))

// given n points in the plane (array p, of size 2*n), generate m points that
// best represent the whole set. the definition of 'best' here is not
// predetermined - the idea is to select points that give good box-box
// collision detection behavior. the chosen point indexes are returned in the
// array iret (of size m). 'i0' is always the first entry in the array.
// n must be in the range [1..8]. m must be in the range [1..n]. i0 must be
// in the range [0..n-1].

void cullPoints(int n, double p[], int m, int i0, int iret[])
{
  // compute the centroid of the polygon in cx,cy
  int i, j;
  double a, cx, cy, q;
  if (n == 1) {
    cx = p[0];
    cy = p[1];
  } else if (n == 2) {
    cx = 0.5 * (p[0] + p[2]);
    cy = 0.5 * (p[1] + p[3]);
  } else {
    a = 0;
    cx = 0;
    cy = 0;
    for (i = 0; i < (n - 1); i++) {
      q = p[i * 2] * p[i * 2 + 3] - p[i * 2 + 2] * p[i * 2 + 1];
      a += q;
      cx += q * (p[i * 2] + p[i * 2 + 2]);
      cy += q * (p[i * 2 + 1] + p[i * 2 + 3]);
    }
    q = p[n * 2 - 2] * p[1] - p[0] * p[n * 2 - 1];
    a = dRecip(3.0 * (a + q));
    cx = a * (cx + q * (p[n * 2 - 2] + p[0]));
    cy = a * (cy + q * (p[n * 2 - 1] + p[1]));
  }

  // compute the angle of each point w.r.t. the centroid
  double A[8];
  for (i = 0; i < n; i++)
    A[i] = atan2(p[i * 2 + 1] - cy, p[i * 2] - cx);

  // search for points that have angles closest to A[i0] + i*(2*pi/m).
  int avail[8];
  for (i = 0; i < n; i++)
    avail[i] = 1;
  avail[i0] = 0;
  iret[0] = i0;
  iret++;
  for (j = 1; j < m; j++) {
    a = double(j) * (2 * math::constantsd::pi() / m) + A[i0];
    if (a > math::constantsd::pi())
      a -= 2 * math::constantsd::pi();
    double maxdiff = 1e9, diff;
    for (i = 0; i < n; i++) {
      if (avail[i]) {
        diff = fabs(A[i] - a);
        if (diff > math::constantsd::pi())
          diff = 2 * math::constantsd::pi() - diff;
        if (diff < maxdiff) {
          maxdiff = diff;
          *iret = i;
        }
      }
    }
    avail[*iret] = 0;
    iret++;
  }
}

void dLineClosestApproach(
    const dVector3 pa,
    const dVector3 ua,
    const dVector3 pb,
    const dVector3 ub,
    double* alpha,
    double* beta)
{
  dVector3 p;
  p[0] = pb[0] - pa[0];
  p[1] = pb[1] - pa[1];
  p[2] = pb[2] - pa[2];
  double uaub = Inner(ua, ub);
  double q1 = Inner(ua, p);
  double q2 = -Inner(ub, p);
  double d = 1 - uaub * uaub;
  if (d <= 0) {
    // @@@ this needs to be made more robust
    *alpha = 0;
    *beta = 0;
  } else {
    d = dRecip(d);
    *alpha = (q1 + uaub * q2) * d;
    *beta = (uaub * q1 + q2) * d;
  }
}

int intersectRectQuad(double h[2], double p[8], double ret[16])
{
  // q (and r) contain nq (and nr) coordinate points for the current (and
  // chopped) polygons
  int nq = 4, nr = 0;
  double buffer[16];
  double* q = p;
  double* r = ret;
  for (int dir = 0; dir <= 1; dir++) {
    // direction notation: xy[0] = x axis, xy[1] = y axis
    for (int sign = -1; sign <= 1; sign += 2) {
      // chop q along the line xy[dir] = sign*h[dir]
      double* pq = q;
      double* pr = r;
      nr = 0;
      for (int i = nq; i > 0; i--) {
        // go through all points in q and all lines between adjacent points
        if (sign * pq[dir] < h[dir]) {
          // this point is inside the chopping line
          pr[0] = pq[0];
          pr[1] = pq[1];
          pr += 2;
          nr++;
          if (nr & 8) {
            q = r;
            goto done;
          }
        }
        double* nextq = (i > 1) ? pq + 2 : q;
        if ((sign * pq[dir] < h[dir]) ^ (sign * nextq[dir] < h[dir])) {
          // this line crosses the chopping line
          pr[1 - dir] = pq[1 - dir]
                        + (nextq[1 - dir] - pq[1 - dir])
                              / (nextq[dir] - pq[dir])
                              * (sign * h[dir] - pq[dir]);
          pr[dir] = sign * h[dir];
          pr += 2;
          nr++;
          if (nr & 8) {
            q = r;
            goto done;
          }
        }
        pq += 2;
      }
      q = r;
      r = (q == ret) ? buffer : ret;
      nq = nr;
    }
  }
done:
  if (q != ret)
    memcpy(ret, q, nr * 2 * sizeof(double));
  return nr;
}

// a simple root finding algorithm is used to find the value of 't' that
// satisfies:
//		d|D(t)|^2/dt = 0
// where:
//		|D(t)| = |p(t)-b(t)|
// where p(t) is a point on the line parameterized by t:
//		p(t) = p1 + t*(p2-p1)
// and b(t) is that same point clipped to the boundary of the box. in box-
// relative coordinates d|D(t)|^2/dt is the sum of three x,y,z components
// each of which looks like this:
//
//	    t_lo     /
//	      ______/    -->t
//	     /     t_hi
//	    /
//
// t_lo and t_hi are the t values where the line passes through the planes
// corresponding to the sides of the box. the algorithm computes d|D(t)|^2/dt
// in a piecewise fashion from t=0 to t=1, stopping at the point where
// d|D(t)|^2/dt crosses from negative to positive.

void dClosestLineBoxPoints(
    const dVector3 p1,
    const dVector3 p2,
    const dVector3 c,
    const dMatrix3 R,
    const dVector3 side,
    dVector3 lret,
    dVector3 bret)
{
  int i;

  // compute the start and delta of the line p1-p2 relative to the box.
  // we will do all subsequent computations in this box-relative coordinate
  // system. we have to do a translation and rotation for each point.
  dVector3 tmp, s, v;
  tmp[0] = p1[0] - c[0];
  tmp[1] = p1[1] - c[1];
  tmp[2] = p1[2] - c[2];
  dMULTIPLY1_331(s, R, tmp);
  tmp[0] = p2[0] - p1[0];
  tmp[1] = p2[1] - p1[1];
  tmp[2] = p2[2] - p1[2];
  dMULTIPLY1_331(v, R, tmp);

  // mirror the line so that v has all components >= 0
  dVector3 sign;
  for (i = 0; i < 3; i++) {
    if (v[i] < 0) {
      s[i] = -s[i];
      v[i] = -v[i];
      sign[i] = -1;
    } else
      sign[i] = 1;
  }

  // compute v^2
  dVector3 v2;
  v2[0] = v[0] * v[0];
  v2[1] = v[1] * v[1];
  v2[2] = v[2] * v[2];

  // compute the half-sides of the box
  double h[3];
  h[0] = side[0];
  h[1] = side[1];
  h[2] = side[2];

  // region is -1,0,+1 depending on which side of the box planes each
  // coordinate is on. tanchor is the next t value at which there is a
  // transition, or the last one if there are no more.
  int region[3];
  double tanchor[3];

  // find the region and tanchor values for p1
  for (i = 0; i < 3; i++) {
    if (v[i] > 0) {
      if (s[i] < -h[i]) {
        region[i] = -1;
        tanchor[i] = (-h[i] - s[i]) / v[i];
      } else {
        region[i] = (s[i] > h[i]);
        tanchor[i] = (h[i] - s[i]) / v[i];
      }
    } else {
      region[i] = 0;
      tanchor[i] = 2; // this will never be a valid tanchor
    }
  }

  // compute d|d|^2/dt for t=0. if it's >= 0 then p1 is the closest point
  double t = 0;
  double dd2dt = 0;
  for (i = 0; i < 3; i++)
    dd2dt -= (region[i] ? v2[i] : 0) * tanchor[i];
  if (dd2dt >= 0)
    goto got_answer;

  do {
    // find the point on the line that is at the next clip plane boundary
    double next_t = 1;
    for (i = 0; i < 3; i++) {
      if (tanchor[i] > t && tanchor[i] < 1 && tanchor[i] < next_t)
        next_t = tanchor[i];
    }

    // compute d|d|^2/dt for the next t
    double next_dd2dt = 0;
    for (i = 0; i < 3; i++) {
      next_dd2dt += (region[i] ? v2[i] : 0) * (next_t - tanchor[i]);
    }

    // if the sign of d|d|^2/dt has changed, solution = the crossover point
    if (next_dd2dt >= 0) {
      double m = (next_dd2dt - dd2dt) / (next_t - t);
      t -= dd2dt / m;
      goto got_answer;
    }

    // advance to the next anchor point / region
    for (i = 0; i < 3; i++) {
      if (tanchor[i] == next_t) {
        tanchor[i] = (h[i] - s[i]) / v[i];
        region[i]++;
      }
    }
    t = next_t;
    dd2dt = next_dd2dt;
  } while (t < 1);
  t = 1;

got_answer:

  // compute closest point on the line
  for (i = 0; i < 3; i++)
    lret[i] = p1[i] + t * tmp[i]; // note: tmp=p2-p1

  // compute closest point on the box
  for (i = 0; i < 3; i++) {
    tmp[i] = sign[i] * (s[i] + t * v[i]);
    if (tmp[i] < -h[i])
      tmp[i] = -h[i];
    else if (tmp[i] > h[i])
      tmp[i] = h[i];
  }
  dMULTIPLY0_331(s, R, tmp);
  for (i = 0; i < 3; i++)
    bret[i] = s[i] + c[i];
}

// given two boxes (p1,R1,side1) and (p2,R2,side2), collide them together and
// generate contact points. this returns 0 if there is no contact otherwise
// it returns the number of contacts generated.
// `normal' returns the contact normal.
// `depth' returns the maximum penetration depth along that normal.
// `return_code' returns a number indicating the type of contact that was
// detected:
//        1,2,3 = box 2 intersects with a face of box 1
//        4,5,6 = box 1 intersects with a face of box 2
//        7..15 = edge-edge contact
// `maxc' is the maximum number of contacts allowed to be generated, i.e.
// the size of the `contact' array.
// `contact' and `skip' are the contact array information provided to the
// collision functions. this function only fills in the position and depth
// fields.
int dBoxBox(
    CollisionObject* o1,
    CollisionObject* o2,
    const dVector3 p1,
    const dMatrix3 R1,
    const dVector3 side1,
    const dVector3 p2,
    const dMatrix3 R2,
    const dVector3 side2,
    CollisionResult& result)
{
  const double fudge_factor = 1.05;
  dVector3 p, pp, normalC = {0.0, 0.0, 0.0, 0.0};
  const double* normalR = 0;
  double A[3], B[3], R11, R12, R13, R21, R22, R23, R31, R32, R33, Q11, Q12, Q13,
      Q21, Q22, Q23, Q31, Q32, Q33, s, s2, l;
  int i, j, invert_normal, code;

  // get vector from centers of box 1 to box 2, relative to box 1
  p[0] = p2[0] - p1[0];
  p[1] = p2[1] - p1[1];
  p[2] = p2[2] - p1[2];
  dMULTIPLY1_331(pp, R1, p); // get pp = p relative to body 1

  // get side lengths / 2
  A[0] = side1[0];
  A[1] = side1[1];
  A[2] = side1[2];
  B[0] = side2[0];
  B[1] = side2[1];
  B[2] = side2[2];

  // Rij is R1'*R2, i.e. the relative rotation between R1 and R2
  R11 = Inner44(R1 + 0, R2 + 0);
  R12 = Inner44(R1 + 0, R2 + 1);
  R13 = Inner44(R1 + 0, R2 + 2);
  R21 = Inner44(R1 + 1, R2 + 0);
  R22 = Inner44(R1 + 1, R2 + 1);
  R23 = Inner44(R1 + 1, R2 + 2);
  R31 = Inner44(R1 + 2, R2 + 0);
  R32 = Inner44(R1 + 2, R2 + 1);
  R33 = Inner44(R1 + 2, R2 + 2);

  Q11 = std::abs(R11);
  Q12 = std::abs(R12);
  Q13 = std::abs(R13);
  Q21 = std::abs(R21);
  Q22 = std::abs(R22);
  Q23 = std::abs(R23);
  Q31 = std::abs(R31);
  Q32 = std::abs(R32);
  Q33 = std::abs(R33);

  // for all 15 possible separating axes:
  //   * see if the axis separates the boxes. if so, return 0.
  //   * find the depth of the penetration along the separating axis (s2)
  //   * if this is the largest depth so far, record it.
  // the normal vector will be set to the separating axis with the smallest
  // depth. note: normalR is set to point to a column of R1 or R2 if that is
  // the smallest depth normal so far. otherwise normalR is 0 and normalC is
  // set to a vector relative to body 1. invert_normal is 1 if the sign of
  // the normal should be flipped.

#define TST(expr1, expr2, norm, cc)                                            \
  s2 = std::abs(expr1) - (expr2);                                              \
  if (s2 > s) {                                                                \
    s = s2;                                                                    \
    normalR = norm;                                                            \
    invert_normal = ((expr1) < 0);                                             \
    code = (cc);                                                               \
  }

  s = -1E12;
  invert_normal = 0;
  code = 0;

  // separating axis = u1,u2,u3
  TST(pp[0], (A[0] + B[0] * Q11 + B[1] * Q12 + B[2] * Q13), R1 + 0, 1);
  TST(pp[1], (A[1] + B[0] * Q21 + B[1] * Q22 + B[2] * Q23), R1 + 1, 2);
  TST(pp[2], (A[2] + B[0] * Q31 + B[1] * Q32 + B[2] * Q33), R1 + 2, 3);

  // separating axis = v1,v2,v3
  TST(Inner41(R2 + 0, p),
      (A[0] * Q11 + A[1] * Q21 + A[2] * Q31 + B[0]),
      R2 + 0,
      4);
  TST(Inner41(R2 + 1, p),
      (A[0] * Q12 + A[1] * Q22 + A[2] * Q32 + B[1]),
      R2 + 1,
      5);
  TST(Inner41(R2 + 2, p),
      (A[0] * Q13 + A[1] * Q23 + A[2] * Q33 + B[2]),
      R2 + 2,
      6);

  // note: cross product axes need to be scaled when s is computed.
  // normal (n1,n2,n3) is relative to box 1.
#undef TST
#define TST(expr1, expr2, n1, n2, n3, cc)                                      \
  s2 = std::abs(expr1) - (expr2);                                              \
  l = sqrt((n1) * (n1) + (n2) * (n2) + (n3) * (n3));                           \
  if (l > 0) {                                                                 \
    s2 /= l;                                                                   \
    if (s2 * fudge_factor > s) {                                               \
      s = s2;                                                                  \
      normalR = 0;                                                             \
      normalC[0] = (n1) / l;                                                   \
      normalC[1] = (n2) / l;                                                   \
      normalC[2] = (n3) / l;                                                   \
      invert_normal = ((expr1) < 0);                                           \
      code = (cc);                                                             \
    }                                                                          \
  }

  // separating axis = u1 x (v1,v2,v3)
  TST(pp[2] * R21 - pp[1] * R31,
      (A[1] * Q31 + A[2] * Q21 + B[1] * Q13 + B[2] * Q12),
      0,
      -R31,
      R21,
      7);
  TST(pp[2] * R22 - pp[1] * R32,
      (A[1] * Q32 + A[2] * Q22 + B[0] * Q13 + B[2] * Q11),
      0,
      -R32,
      R22,
      8);
  TST(pp[2] * R23 - pp[1] * R33,
      (A[1] * Q33 + A[2] * Q23 + B[0] * Q12 + B[1] * Q11),
      0,
      -R33,
      R23,
      9);

  // separating axis = u2 x (v1,v2,v3)
  TST(pp[0] * R31 - pp[2] * R11,
      (A[0] * Q31 + A[2] * Q11 + B[1] * Q23 + B[2] * Q22),
      R31,
      0,
      -R11,
      10);
  TST(pp[0] * R32 - pp[2] * R12,
      (A[0] * Q32 + A[2] * Q12 + B[0] * Q23 + B[2] * Q21),
      R32,
      0,
      -R12,
      11);
  TST(pp[0] * R33 - pp[2] * R13,
      (A[0] * Q33 + A[2] * Q13 + B[0] * Q22 + B[1] * Q21),
      R33,
      0,
      -R13,
      12);

  // separating axis = u3 x (v1,v2,v3)
  TST(pp[1] * R11 - pp[0] * R21,
      (A[0] * Q21 + A[1] * Q11 + B[1] * Q33 + B[2] * Q32),
      -R21,
      R11,
      0,
      13);
  TST(pp[1] * R12 - pp[0] * R22,
      (A[0] * Q22 + A[1] * Q12 + B[0] * Q33 + B[2] * Q31),
      -R22,
      R12,
      0,
      14);
  TST(pp[1] * R13 - pp[0] * R23,
      (A[0] * Q23 + A[1] * Q13 + B[0] * Q32 + B[1] * Q31),
      -R23,
      R13,
      0,
      15);

#undef TST

  if (!code)
    return 0;
  if (s > 0.0)
    return 0;

  // if we get to this point, the boxes interpenetrate. compute the normal
  // in global coordinates.

  Eigen::Vector3d normal;
  Eigen::Vector3d point_vec;
  double penetration;

  if (normalR) {
    normal << normalR[0], normalR[4], normalR[8];
  } else {
    normal << Inner((R1), (normalC)), Inner((R1 + 4), (normalC)),
        Inner((R1 + 8), (normalC));
    // dMULTIPLY0_331 (normal,R1,normalC);
  }
  if (invert_normal) {
    normal *= -1.0;
  }

  // compute contact point(s)

  // single point
  if (code > 6) {
    // an edge from box 1 touches an edge from box 2.
    // find a point pa on the intersecting edge of box 1
    dVector3 pa;
    double sign;
    for (i = 0; i < 3; i++)
      pa[i] = p1[i];
    for (j = 0; j < 3; j++) {
#define TEMP_INNER14(a, b) (a[0] * (b)[0] + a[1] * (b)[4] + a[2] * (b)[8])
      sign = (TEMP_INNER14(normal, R1 + j) > 0) ? 1.0 : -1.0;

      // sign = (Inner14(normal,R1+j) > 0) ? 1.0 : -1.0;

      for (i = 0; i < 3; i++)
        pa[i] += sign * A[j] * R1[i * 4 + j];
    }

    // find a point pb on the intersecting edge of box 2
    dVector3 pb;
    for (i = 0; i < 3; i++)
      pb[i] = p2[i];
    for (j = 0; j < 3; j++) {
      sign = (TEMP_INNER14(normal, R2 + j) > 0) ? -1.0 : 1.0;
#undef TEMP_INNER14
      for (i = 0; i < 3; i++)
        pb[i] += sign * B[j] * R2[i * 4 + j];
    }

    double alpha, beta;
    dVector3 ua, ub;
    for (i = 0; i < 3; i++)
      ua[i] = R1[((code)-7) / 3 + i * 4];
    for (i = 0; i < 3; i++)
      ub[i] = R2[((code)-7) % 3 + i * 4];

    dLineClosestApproach(pa, ua, pb, ub, &alpha, &beta);
    for (i = 0; i < 3; i++)
      pa[i] += ua[i] * alpha;
    for (i = 0; i < 3; i++)
      pb[i] += ub[i] * beta;

    {
      point_vec << 0.5 * (pa[0] + pb[0]), 0.5 * (pa[1] + pb[1]),
          0.5 * (pa[2] + pb[2]);
      penetration = -s;

      Contact contact;
      contact.collisionObject1 = o1;
      contact.collisionObject2 = o2;
      contact.point = point_vec;
      contact.normal = normal;
      contact.penetrationDepth = penetration;
      result.addContact(contact);
    }
    return 1;
  }

  // okay, we have a face-something intersection (because the separating
  // axis is perpendicular to a face). define face 'a' to be the reference
  // face (i.e. the normal vector is perpendicular to this) and face 'b' to be
  // the incident face (the closest face of the other box).

  const double *Ra, *Rb, *pa, *pb, *Sa, *Sb;
  if (code <= 3) {
    Ra = R1;
    Rb = R2;
    pa = p1;
    pb = p2;
    Sa = A;
    Sb = B;
  } else {
    Ra = R2;
    Rb = R1;
    pa = p2;
    pb = p1;
    Sa = B;
    Sb = A;
  }

  // nr = normal vector of reference face dotted with axes of incident box.
  // anr = absolute values of nr.
  dVector3 normal2, nr, anr;
  if (code <= 3) {
    normal2[0] = normal[0];
    normal2[1] = normal[1];
    normal2[2] = normal[2];
  } else {
    normal2[0] = -normal[0];
    normal2[1] = -normal[1];
    normal2[2] = -normal[2];
  }
  dMULTIPLY1_331(nr, Rb, normal2);
  anr[0] = fabs(nr[0]);
  anr[1] = fabs(nr[1]);
  anr[2] = fabs(nr[2]);

  // find the largest compontent of anr: this corresponds to the normal
  // for the indident face. the other axis numbers of the indicent face
  // are stored in a1,a2.
  int lanr, a1, a2;
  if (anr[1] > anr[0]) {
    if (anr[1] > anr[2]) {
      a1 = 0;
      lanr = 1;
      a2 = 2;
    } else {
      a1 = 0;
      a2 = 1;
      lanr = 2;
    }
  } else {
    if (anr[0] > anr[2]) {
      lanr = 0;
      a1 = 1;
      a2 = 2;
    } else {
      a1 = 0;
      a2 = 1;
      lanr = 2;
    }
  }

  // compute center point of incident face, in reference-face coordinates
  dVector3 center;
  if (nr[lanr] < 0) {
    for (i = 0; i < 3; i++)
      center[i] = pb[i] - pa[i] + Sb[lanr] * Rb[i * 4 + lanr];
  } else {
    for (i = 0; i < 3; i++)
      center[i] = pb[i] - pa[i] - Sb[lanr] * Rb[i * 4 + lanr];
  }

  // find the normal and non-normal axis numbers of the reference box
  int codeN, code1, code2;
  if (code <= 3)
    codeN = code - 1;
  else
    codeN = code - 4;
  if (codeN == 0) {
    code1 = 1;
    code2 = 2;
  } else if (codeN == 1) {
    code1 = 0;
    code2 = 2;
  } else {
    code1 = 0;
    code2 = 1;
  }

  // find the four corners of the incident face, in reference-face coordinates
  double quad[8]; // 2D coordinate of incident face (x,y pairs)
  double c1, c2, m11, m12, m21, m22;
  c1 = Inner14(center, Ra + code1);
  c2 = Inner14(center, Ra + code2);
  // optimize this? - we have already computed this data above, but it is not
  // stored in an easy-to-index format. for now it's quicker just to recompute
  // the four dot products.
  m11 = Inner44(Ra + code1, Rb + a1);
  m12 = Inner44(Ra + code1, Rb + a2);
  m21 = Inner44(Ra + code2, Rb + a1);
  m22 = Inner44(Ra + code2, Rb + a2);
  {
    double k1 = m11 * Sb[a1];
    double k2 = m21 * Sb[a1];
    double k3 = m12 * Sb[a2];
    double k4 = m22 * Sb[a2];
    quad[0] = c1 - k1 - k3;
    quad[1] = c2 - k2 - k4;
    quad[2] = c1 - k1 + k3;
    quad[3] = c2 - k2 + k4;
    quad[4] = c1 + k1 + k3;
    quad[5] = c2 + k2 + k4;
    quad[6] = c1 + k1 - k3;
    quad[7] = c2 + k2 - k4;
  }

  // find the size of the reference face
  double rect[2];
  rect[0] = Sa[code1];
  rect[1] = Sa[code2];

  // intersect the incident and reference faces
  double ret[16];
  int n = intersectRectQuad(rect, quad, ret);
  if (n < 1)
    return 0; // this should never happen

  // convert the intersection points into reference-face coordinates,
  // and compute the contact position and depth for each point. only keep
  // those points that have a positive (penetrating) depth. delete points in
  // the 'ret' array as necessary so that 'point' and 'ret' correspond.
  // real point[3*8];		// penetrating contact points
  double point[24]; // penetrating contact points
  double dep[8];    // depths for those points
  double det1 = dRecip(m11 * m22 - m12 * m21);
  m11 *= det1;
  m12 *= det1;
  m21 *= det1;
  m22 *= det1;
  int cnum = 0; // number of penetrating contact points found
  for (j = 0; j < n; j++) {
    double k1 = m22 * (ret[j * 2] - c1) - m12 * (ret[j * 2 + 1] - c2);
    double k2 = -m21 * (ret[j * 2] - c1) + m11 * (ret[j * 2 + 1] - c2);
    for (i = 0; i < 3; i++) {
      point[cnum * 3 + i]
          = center[i] + k1 * Rb[i * 4 + a1] + k2 * Rb[i * 4 + a2];
    }
    dep[cnum] = Sa[codeN] - Inner(normal2, point + cnum * 3);
    if (dep[cnum] >= 0) {
      ret[cnum * 2] = ret[j * 2];
      ret[cnum * 2 + 1] = ret[j * 2 + 1];
      cnum++;
    }
  }
  if (cnum < 1)
    return 0; // this should never happen

  // we can't generate more contacts than we actually have
  int maxc = 4;
  if (maxc > cnum)
    maxc = cnum;
  // if (maxc < 1) maxc = 1;

  if (cnum <= maxc) {
    // we have less contacts than we need, so we use them all
    for (j = 0; j < cnum; j++) {
      point_vec << point[j * 3 + 0] + pa[0], point[j * 3 + 1] + pa[1],
          point[j * 3 + 2] + pa[2];

      Contact contact;
      contact.collisionObject1 = o1;
      contact.collisionObject2 = o2;
      contact.point = point_vec;
      contact.normal = normal;
      contact.penetrationDepth = dep[j];
      result.addContact(contact);
    }
  } else {
    // we have more contacts than are wanted, some of them must be culled.
    // find the deepest point, it is always the first contact.
    int i1 = 0;
    double maxdepth = dep[0];
    for (i = 1; i < cnum; i++) {
      if (dep[i] > maxdepth) {
        maxdepth = dep[i];
        i1 = i;
      }
    }

    int iret[8];
    cullPoints(cnum, ret, maxc, i1, iret);

    cnum = maxc;
    for (j = 0; j < cnum; j++) {
      point_vec << point[iret[j] * 3 + 0] + pa[0],
          point[iret[j] * 3 + 1] + pa[1], point[iret[j] * 3 + 2] + pa[2];

      Contact contact;
      contact.collisionObject1 = o1;
      contact.collisionObject2 = o2;
      contact.point = point_vec;
      contact.normal = normal;
      contact.penetrationDepth = dep[iret[j]];
      result.addContact(contact);
    }
  }
  return cnum;
}

int collideBoxBox(
    CollisionObject* o1,
    CollisionObject* o2,
    const Eigen::Vector3d& size0,
    const Eigen::Isometry3d& T0,
    const Eigen::Vector3d& size1,
    const Eigen::Isometry3d& T1,
    CollisionResult& result)
{
  dVector3 halfSize0;
  dVector3 halfSize1;

  convVector(0.5 * size0, halfSize0);
  convVector(0.5 * size1, halfSize1);

  dMatrix3 R0, R1;

  convMatrix(T0, R0);
  convMatrix(T1, R1);

  dVector3 p0;
  dVector3 p1;

  convVector(T0.translation(), p0);
  convVector(T1.translation(), p1);

  return dBoxBox(o1, o2, p1, R1, halfSize1, p0, R0, halfSize0, result);
}

int collideBoxSphere(
    CollisionObject* o1,
    CollisionObject* o2,
    const Eigen::Vector3d& size0,
    const Eigen::Isometry3d& T0,
    const double& r1,
    const Eigen::Isometry3d& T1,
    CollisionResult& result)
{
  Eigen::Vector3d halfSize = 0.5 * size0;
  bool inside_box = true;

  // clipping a center of the sphere to a boundary of the box
  // Vec3 c0(&T0[9]);
  Eigen::Vector3d c0 = T1.translation();
  Eigen::Vector3d p = T0.inverse() * c0;

  if (p[0] < -halfSize[0]) {
    p[0] = -halfSize[0];
    inside_box = false;
  }
  if (p[0] > halfSize[0]) {
    p[0] = halfSize[0];
    inside_box = false;
  }

  if (p[1] < -halfSize[1]) {
    p[1] = -halfSize[1];
    inside_box = false;
  }
  if (p[1] > halfSize[1]) {
    p[1] = halfSize[1];
    inside_box = false;
  }

  if (p[2] < -halfSize[2]) {
    p[2] = -halfSize[2];
    inside_box = false;
  }
  if (p[2] > halfSize[2]) {
    p[2] = halfSize[2];
    inside_box = false;
  }

  Eigen::Vector3d normal(0.0, 0.0, 0.0);
  double penetration;

  if (inside_box) {
    // find nearest side from the sphere center
    double min = halfSize[0] - std::abs(p[0]);
    double tmin = halfSize[1] - std::abs(p[1]);
    int idx = 0;

    if (tmin < min) {
      min = tmin;
      idx = 1;
    }
    tmin = halfSize[2] - std::abs(p[2]);
    if (tmin < min) {
      min = tmin;
      idx = 2;
    }

    // normal[idx] = (p[idx] > 0.0 ? 1.0 : -1.0);
    normal[idx] = (p[idx] > 0.0 ? -1.0 : 1.0);
    normal = T0.linear() * normal;
    penetration = min + r1;

    Contact contact;
    contact.collisionObject1 = o1;
    contact.collisionObject2 = o2;
    contact.point = c0;
    contact.normal = normal;
    contact.penetrationDepth = penetration;
    result.addContact(contact);
    return 1;
  }

  Eigen::Vector3d contactpt = T0 * p;
  // normal = c0 - contactpt;
  normal = contactpt - c0;
  double mag = normal.norm();
  penetration = r1 - mag;

  if (penetration < 0.0) {
    return 0;
  }

  if (mag > DART_COLLISION_EPS) {
    normal *= (1.0 / mag);

    Contact contact;
    contact.collisionObject1 = o1;
    contact.collisionObject2 = o2;
    contact.point = contactpt;
    contact.normal = normal;
    contact.penetrationDepth = penetration;
    result.addContact(contact);
  } else {
    double min = halfSize[0] - std::abs(p[0]);
    double tmin = halfSize[1] - std::abs(p[1]);
    int idx = 0;

    if (tmin < min) {
      min = tmin;
      idx = 1;
    }
    tmin = halfSize[2] - std::abs(p[2]);
    if (tmin < min) {
      min = tmin;
      idx = 2;
    }
    normal.setZero();
    // normal[idx] = (p[idx] > 0.0 ? 1.0 : -1.0);
    normal[idx] = (p[idx] > 0.0 ? -1.0 : 1.0);
    normal = T0.linear() * normal;

    Contact contact;
    contact.collisionObject1 = o1;
    contact.collisionObject2 = o2;
    contact.point = contactpt;
    contact.normal = normal;
    contact.penetrationDepth = penetration;
    result.addContact(contact);
  }
  return 1;
}

int collideSphereBox(
    CollisionObject* o1,
    CollisionObject* o2,
    const double& r0,
    const Eigen::Isometry3d& T0,
    const Eigen::Vector3d& size1,
    const Eigen::Isometry3d& T1,
    CollisionResult& result)
{
  Eigen::Vector3d size = 0.5 * size1;
  bool inside_box = true;

  // clipping a center of the sphere to a boundary of the box
  Eigen::Vector3d c0 = T0.translation();
  Eigen::Vector3d p = T1.inverse() * c0;

  if (p[0] < -size[0]) {
    p[0] = -size[0];
    inside_box = false;
  }
  if (p[0] > size[0]) {
    p[0] = size[0];
    inside_box = false;
  }

  if (p[1] < -size[1]) {
    p[1] = -size[1];
    inside_box = false;
  }
  if (p[1] > size[1]) {
    p[1] = size[1];
    inside_box = false;
  }

  if (p[2] < -size[2]) {
    p[2] = -size[2];
    inside_box = false;
  }
  if (p[2] > size[2]) {
    p[2] = size[2];
    inside_box = false;
  }

  Eigen::Vector3d normal(0.0, 0.0, 0.0);
  double penetration;

  if (inside_box) {
    // find nearest side from the sphere center
    double min = size[0] - std::abs(p[0]);
    double tmin = size[1] - std::abs(p[1]);
    int idx = 0;

    if (tmin < min) {
      min = tmin;
      idx = 1;
    }
    tmin = size[2] - std::abs(p[2]);
    if (tmin < min) {
      min = tmin;
      idx = 2;
    }

    normal[idx] = (p[idx] > 0.0 ? 1.0 : -1.0);
    normal = T1.linear() * normal;
    penetration = min + r0;

    Contact contact;
    contact.collisionObject1 = o1;
    contact.collisionObject2 = o2;
    contact.point = c0;
    contact.normal = normal;
    contact.penetrationDepth = penetration;
    result.addContact(contact);
    return 1;
  }

  Eigen::Vector3d contactpt = T1 * p;
  normal = c0 - contactpt;
  double mag = normal.norm();
  penetration = r0 - mag;

  if (penetration < 0.0) {
    return 0;
  }

  if (mag > DART_COLLISION_EPS) {
    normal *= (1.0 / mag);

    Contact contact;
    contact.collisionObject1 = o1;
    contact.collisionObject2 = o2;
    contact.point = contactpt;
    contact.normal = normal;
    contact.penetrationDepth = penetration;
    result.addContact(contact);
  } else {
    double min = size[0] - std::abs(p[0]);
    double tmin = size[1] - std::abs(p[1]);
    int idx = 0;

    if (tmin < min) {
      min = tmin;
      idx = 1;
    }
    tmin = size[2] - std::abs(p[2]);
    if (tmin < min) {
      min = tmin;
      idx = 2;
    }
    normal.setZero();
    normal[idx] = (p[idx] > 0.0 ? 1.0 : -1.0);
    normal = T1.linear() * normal;

    Contact contact;
    contact.collisionObject1 = o1;
    contact.collisionObject2 = o2;
    contact.point = contactpt;
    contact.normal = normal;
    contact.penetrationDepth = penetration;
    result.addContact(contact);
  }
  return 1;
}

int collideSphereSphere(
    CollisionObject* o1,
    CollisionObject* o2,
    const double& _r0,
    const Eigen::Isometry3d& c0,
    const double& _r1,
    const Eigen::Isometry3d& c1,
    CollisionResult& result)
{
  double r0 = _r0;
  double r1 = _r1;
  double rsum = r0 + r1;
  Eigen::Vector3d normal = c0.translation() - c1.translation();
  double normal_sqr = normal.squaredNorm();

  if (normal_sqr > rsum * rsum) {
    return 0;
  }

  r0 /= rsum;
  r1 /= rsum;

  Eigen::Vector3d point = r1 * c0.translation() + r0 * c1.translation();
  double penetration;

  if (normal_sqr < DART_COLLISION_EPS) {
    normal.setZero();
    penetration = rsum;

    Contact contact;
    contact.collisionObject1 = o1;
    contact.collisionObject2 = o2;
    contact.point = point;
    contact.normal = normal;
    contact.penetrationDepth = penetration;
    result.addContact(contact);
    return 1;
  }

  normal_sqr = sqrt(normal_sqr);
  normal *= (1.0 / normal_sqr);
  penetration = rsum - normal_sqr;

  Contact contact;
  contact.collisionObject1 = o1;
  contact.collisionObject2 = o2;
  contact.point = point;
  contact.normal = normal;
  contact.penetrationDepth = penetration;
  result.addContact(contact);
  return 1;
}

int collidePlaneSphere(
    CollisionObject* o1,
    CollisionObject* o2,
    const Eigen::Vector3d& plane_normal,
    const double& plane_offset,
    const Eigen::Isometry3d& T0,
    const double& sphere_rad,
    const Eigen::Isometry3d& T1,
    CollisionResult& result)
{
  const Eigen::Vector3d worldNormal = T0.linear() * plane_normal;
  const Eigen::Vector3d planePoint
      = T0.translation() + worldNormal * plane_offset;
  const Eigen::Vector3d sphereCenter = T1.translation();
  const double signedDist = worldNormal.dot(sphereCenter - planePoint);

  if (signedDist > sphere_rad)
    return 0;

  Contact contact;
  contact.collisionObject1 = o1;
  contact.collisionObject2 = o2;
  contact.point = sphereCenter - worldNormal * signedDist;
  contact.normal = -worldNormal;
  contact.penetrationDepth = sphere_rad - signedDist;
  result.addContact(contact);
  return 1;
}

int collideSpherePlane(
    CollisionObject* o1,
    CollisionObject* o2,
    const double& sphere_rad,
    const Eigen::Isometry3d& T0,
    const Eigen::Vector3d& plane_normal,
    const double& plane_offset,
    const Eigen::Isometry3d& T1,
    CollisionResult& result)
{
  const Eigen::Vector3d worldNormal = T1.linear() * plane_normal;
  const Eigen::Vector3d planePoint
      = T1.translation() + worldNormal * plane_offset;
  const Eigen::Vector3d sphereCenter = T0.translation();
  const double signedDist = worldNormal.dot(sphereCenter - planePoint);

  if (signedDist > sphere_rad)
    return 0;

  Contact contact;
  contact.collisionObject1 = o1;
  contact.collisionObject2 = o2;
  contact.point = sphereCenter - worldNormal * signedDist;
  contact.normal = worldNormal;
  contact.penetrationDepth = sphere_rad - signedDist;
  result.addContact(contact);
  return 1;
}

int collidePlaneBox(
    CollisionObject* o1,
    CollisionObject* o2,
    const Eigen::Vector3d& plane_normal,
    const double& plane_offset,
    const Eigen::Isometry3d& T0,
    const Eigen::Vector3d& size1,
    const Eigen::Isometry3d& T1,
    CollisionResult& result)
{
  const Eigen::Vector3d worldNormal = T0.linear() * plane_normal;
  const Eigen::Vector3d planePoint
      = T0.translation() + worldNormal * plane_offset;
  const Eigen::Vector3d halfExtents = 0.5 * size1;

  double minDist = std::numeric_limits<double>::max();
  Eigen::Vector3d worldCorners[8];
  double signedDistances[8];

  for (int i = 0; i < 8; ++i) {
    const Eigen::Vector3d localCorner(
        (i & 1) ? halfExtents.x() : -halfExtents.x(),
        (i & 2) ? halfExtents.y() : -halfExtents.y(),
        (i & 4) ? halfExtents.z() : -halfExtents.z());

    const Eigen::Vector3d worldCorner = T1 * localCorner;
    worldCorners[i] = worldCorner;
    const double signedDist = worldNormal.dot(worldCorner - planePoint);
    signedDistances[i] = signedDist;

    if (signedDist < minDist)
      minDist = signedDist;
  }

  if (minDist > 0.0)
    return 0;

  auto numContacts = 0;
  constexpr double contactTolerance = 1e-9;
  for (int i = 0; i < 8; ++i) {
    const double signedDist = signedDistances[i];
    if (signedDist > contactTolerance)
      continue;

    Contact contact;
    contact.collisionObject1 = o1;
    contact.collisionObject2 = o2;
    contact.point = worldCorners[i] - worldNormal * signedDist;
    contact.normal = -worldNormal;
    contact.penetrationDepth = std::max(0.0, -signedDist);
    result.addContact(contact);
    ++numContacts;
    if (numContacts >= 3)
      break;
  }

  if (numContacts > 0)
    return numContacts;

  Contact contact;
  contact.collisionObject1 = o1;
  contact.collisionObject2 = o2;
  contact.point = worldCorners[0] - worldNormal * signedDistances[0];
  contact.normal = -worldNormal;
  contact.penetrationDepth = 0.0;
  result.addContact(contact);
  return 1;
}

int collideBoxPlane(
    CollisionObject* o1,
    CollisionObject* o2,
    const Eigen::Vector3d& size0,
    const Eigen::Isometry3d& T0,
    const Eigen::Vector3d& plane_normal,
    const double& plane_offset,
    const Eigen::Isometry3d& T1,
    CollisionResult& result)
{
  const Eigen::Vector3d worldNormal = T1.linear() * plane_normal;
  const Eigen::Vector3d planePoint
      = T1.translation() + worldNormal * plane_offset;
  const Eigen::Vector3d halfExtents = 0.5 * size0;

  double minDist = std::numeric_limits<double>::max();
  Eigen::Vector3d worldCorners[8];
  double signedDistances[8];

  for (int i = 0; i < 8; ++i) {
    const Eigen::Vector3d localCorner(
        (i & 1) ? halfExtents.x() : -halfExtents.x(),
        (i & 2) ? halfExtents.y() : -halfExtents.y(),
        (i & 4) ? halfExtents.z() : -halfExtents.z());

    const Eigen::Vector3d worldCorner = T0 * localCorner;
    worldCorners[i] = worldCorner;
    const double signedDist = worldNormal.dot(worldCorner - planePoint);
    signedDistances[i] = signedDist;

    if (signedDist < minDist)
      minDist = signedDist;
  }

  if (minDist > 0.0)
    return 0;

  auto numContacts = 0;
  constexpr double contactTolerance = 1e-9;
  for (int i = 0; i < 8; ++i) {
    const double signedDist = signedDistances[i];
    if (signedDist > contactTolerance)
      continue;

    Contact contact;
    contact.collisionObject1 = o1;
    contact.collisionObject2 = o2;
    contact.point = worldCorners[i] - worldNormal * signedDist;
    contact.normal = worldNormal;
    contact.penetrationDepth = std::max(0.0, -signedDist);
    result.addContact(contact);
    ++numContacts;
    if (numContacts >= 3)
      break;
  }

  if (numContacts > 0)
    return numContacts;

  Contact contact;
  contact.collisionObject1 = o1;
  contact.collisionObject2 = o2;
  contact.point = worldCorners[0] - worldNormal * signedDistances[0];
  contact.normal = worldNormal;
  contact.penetrationDepth = 0.0;
  result.addContact(contact);
  return 1;
}

namespace {

enum class SupportShapeType
{
  Box,
  Cylinder
};

struct SupportShape
{
  SupportShapeType type = SupportShapeType::Box;
  Eigen::Vector3d size = Eigen::Vector3d::Zero();
  double radius = 0.0;
  double halfHeight = 0.0;
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();

  Eigen::Vector3d center() const
  {
    return transform.translation();
  }

  Eigen::Vector3d support(const Eigen::Vector3d& direction) const
  {
    const Eigen::Vector3d localDir = transform.linear().transpose() * direction;
    Eigen::Vector3d localSupport;

    if (type == SupportShapeType::Box) {
      const Eigen::Vector3d halfExtents = 0.5 * size;
      localSupport.x()
          = (localDir.x() >= 0.0) ? halfExtents.x() : -halfExtents.x();
      localSupport.y()
          = (localDir.y() >= 0.0) ? halfExtents.y() : -halfExtents.y();
      localSupport.z()
          = (localDir.z() >= 0.0) ? halfExtents.z() : -halfExtents.z();
    } else {
      const double xyLen = std::sqrt(
          localDir.x() * localDir.x() + localDir.y() * localDir.y());
      if (xyLen < 1e-10) {
        localSupport.x() = radius;
        localSupport.y() = 0.0;
      } else {
        localSupport.x() = radius * localDir.x() / xyLen;
        localSupport.y() = radius * localDir.y() / xyLen;
      }
      localSupport.z() = (localDir.z() >= 0.0) ? halfHeight : -halfHeight;
    }

    return transform * localSupport;
  }
};

struct SupportPoint
{
  Eigen::Vector3d v = Eigen::Vector3d::Zero();
  Eigen::Vector3d v1 = Eigen::Vector3d::Zero();
  Eigen::Vector3d v2 = Eigen::Vector3d::Zero();
};

struct MprPortal
{
  std::array<SupportPoint, 4> points;
  int size = 0;
};

struct MprResult
{
  bool success = false;
  double depth = 0.0;
  Eigen::Vector3d normal = Eigen::Vector3d::Zero();
  Eigen::Vector3d pointOnA = Eigen::Vector3d::Zero();
  Eigen::Vector3d pointOnB = Eigen::Vector3d::Zero();
};

constexpr double kMprEpsilon = 1e-12;
constexpr double kMprTolerance = 1e-6;
constexpr int kMprMaxIterations = 64;

Eigen::Vector3d getCylinderRadialDirection(
    const Eigen::Vector3d& center, double dist)
{
  if (dist <= 1e-12)
    return Eigen::Vector3d::UnitX();

  return Eigen::Vector3d(center[0] / dist, center[1] / dist, 0.0);
}

SupportPoint computeSupport(
    const SupportShape& supportA,
    const SupportShape& supportB,
    const Eigen::Vector3d& direction)
{
  Eigen::Vector3d dir = direction;
  if (dir.squaredNorm() < kMprEpsilon)
    dir = Eigen::Vector3d::UnitX();

  SupportPoint point;
  point.v1 = supportA.support(dir);
  point.v2 = supportB.support(-dir);
  point.v = point.v1 - point.v2;
  return point;
}

SupportPoint makeCenterPoint(
    const Eigen::Vector3d& centerA, const Eigen::Vector3d& centerB)
{
  SupportPoint point;
  point.v1 = centerA;
  point.v2 = centerB;
  point.v = centerA - centerB;
  return point;
}

bool isZero(double value)
{
  return std::abs(value) < kMprEpsilon;
}

bool normalizeSafe(Eigen::Vector3d& v)
{
  const double norm = v.norm();
  if (norm < kMprEpsilon)
    return false;

  v /= norm;
  return true;
}

void portalDir(const MprPortal& portal, Eigen::Vector3d& dir)
{
  const Eigen::Vector3d v2v1 = portal.points[2].v - portal.points[1].v;
  const Eigen::Vector3d v3v1 = portal.points[3].v - portal.points[1].v;
  dir = v2v1.cross(v3v1);
  if (!normalizeSafe(dir))
    dir = Eigen::Vector3d::UnitX();
}

bool portalEncapsulatesOrigin(
    const MprPortal& portal, const Eigen::Vector3d& dir)
{
  const double dot = dir.dot(portal.points[1].v);
  return isZero(dot) || dot > 0.0;
}

bool portalReachTolerance(
    const MprPortal& portal, const SupportPoint& v4, const Eigen::Vector3d& dir)
{
  const double dv1 = portal.points[1].v.dot(dir);
  const double dv2 = portal.points[2].v.dot(dir);
  const double dv3 = portal.points[3].v.dot(dir);
  const double dv4 = v4.v.dot(dir);

  const double delta = std::min({dv4 - dv1, dv4 - dv2, dv4 - dv3});
  return delta <= kMprTolerance || isZero(delta - kMprTolerance);
}

bool portalCanEncapsulateOrigin(
    const SupportPoint& v4, const Eigen::Vector3d& dir)
{
  const double dot = v4.v.dot(dir);
  return isZero(dot) || dot > 0.0;
}

void expandPortal(MprPortal& portal, const SupportPoint& v4)
{
  const Eigen::Vector3d v4v0 = v4.v.cross(portal.points[0].v);
  double dot = portal.points[1].v.dot(v4v0);
  if (dot > 0.0) {
    dot = portal.points[2].v.dot(v4v0);
    if (dot > 0.0)
      portal.points[1] = v4;
    else
      portal.points[3] = v4;
  } else {
    dot = portal.points[3].v.dot(v4v0);
    if (dot > 0.0)
      portal.points[2] = v4;
    else
      portal.points[1] = v4;
  }
}

int discoverPortal(
    const SupportShape& supportA,
    const SupportShape& supportB,
    const Eigen::Vector3d& centerA,
    const Eigen::Vector3d& centerB,
    MprPortal& portal)
{
  portal.points[0] = makeCenterPoint(centerA, centerB);
  portal.size = 1;

  if (portal.points[0].v.squaredNorm() < kMprEpsilon)
    portal.points[0].v += Eigen::Vector3d(kMprTolerance * 10.0, 0.0, 0.0);

  Eigen::Vector3d dir = -portal.points[0].v;
  if (!normalizeSafe(dir))
    dir = Eigen::Vector3d::UnitX();

  portal.points[1] = computeSupport(supportA, supportB, dir);
  portal.size = 2;

  double dot = portal.points[1].v.dot(dir);
  if (dot < 0.0 && !isZero(dot))
    return -1;
  if (isZero(dot))
    return 1;

  dir = portal.points[0].v.cross(portal.points[1].v);
  if (dir.squaredNorm() < kMprEpsilon) {
    if (portal.points[1].v.squaredNorm() < kMprEpsilon)
      return 1;

    return 2;
  }

  dir.normalize();
  portal.points[2] = computeSupport(supportA, supportB, dir);
  dot = portal.points[2].v.dot(dir);
  if (dot < 0.0 && !isZero(dot))
    return -1;
  if (isZero(dot)) {
    portal.points[1] = portal.points[2];
    return 1;
  }

  portal.size = 3;

  Eigen::Vector3d va = portal.points[1].v - portal.points[0].v;
  Eigen::Vector3d vb = portal.points[2].v - portal.points[0].v;
  dir = va.cross(vb);
  if (!normalizeSafe(dir))
    return -1;

  dot = dir.dot(portal.points[0].v);
  if (dot > 0.0) {
    std::swap(portal.points[1], portal.points[2]);
    dir = -dir;
  }

  while (portal.size < 4) {
    portal.points[3] = computeSupport(supportA, supportB, dir);
    dot = portal.points[3].v.dot(dir);
    if (dot < 0.0 && !isZero(dot))
      return -1;
    if (isZero(dot)) {
      portal.points[1] = portal.points[3];
      return 1;
    }

    auto shouldContinue = false;
    Eigen::Vector3d cross = portal.points[1].v.cross(portal.points[3].v);
    dot = cross.dot(portal.points[0].v);
    if (dot < 0.0 && !isZero(dot)) {
      portal.points[2] = portal.points[3];
      shouldContinue = true;
    }

    if (!shouldContinue) {
      cross = portal.points[3].v.cross(portal.points[2].v);
      dot = cross.dot(portal.points[0].v);
      if (dot < 0.0 && !isZero(dot)) {
        portal.points[1] = portal.points[3];
        shouldContinue = true;
      }
    }

    if (shouldContinue) {
      va = portal.points[1].v - portal.points[0].v;
      vb = portal.points[2].v - portal.points[0].v;
      dir = va.cross(vb);
      if (!normalizeSafe(dir))
        return -1;
    } else {
      portal.size = 4;
    }
  }

  return 0;
}

int refinePortal(
    const SupportShape& supportA,
    const SupportShape& supportB,
    MprPortal& portal,
    bool& mayTouch)
{
  mayTouch = false;

  for (int iter = 0; iter < kMprMaxIterations; ++iter) {
    Eigen::Vector3d dir;
    portalDir(portal, dir);

    if (portalEncapsulatesOrigin(portal, dir))
      return 0;

    SupportPoint v4 = computeSupport(supportA, supportB, dir);
    if (!portalCanEncapsulateOrigin(v4, dir))
      return -1;

    if (portalReachTolerance(portal, v4, dir)) {
      mayTouch = true;
      return -1;
    }

    expandPortal(portal, v4);
  }

  return -1;
}

struct SegmentClosestResult
{
  Eigen::Vector3d closest = Eigen::Vector3d::Zero();
  double t = 0.0;
};

SegmentClosestResult closestPointOnSegmentToOrigin(
    const Eigen::Vector3d& a, const Eigen::Vector3d& b)
{
  SegmentClosestResult result;
  const Eigen::Vector3d ab = b - a;
  const double abLen2 = ab.squaredNorm();
  if (abLen2 < kMprEpsilon) {
    result.closest = a;
    result.t = 0.0;
    return result;
  }

  result.t = std::clamp(-a.dot(ab) / abLen2, 0.0, 1.0);
  result.closest = a + result.t * ab;
  return result;
}

Eigen::Vector3d closestPointOnTriangleToOrigin(
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c)
{
  const Eigen::Vector3d ab = b - a;
  const Eigen::Vector3d ac = c - a;
  const Eigen::Vector3d ap = -a;

  const double d1 = ab.dot(ap);
  const double d2 = ac.dot(ap);
  if (d1 <= 0.0 && d2 <= 0.0)
    return a;

  const Eigen::Vector3d bp = -b;
  const double d3 = ab.dot(bp);
  const double d4 = ac.dot(bp);
  if (d3 >= 0.0 && d4 <= d3)
    return b;

  const Eigen::Vector3d cp = -c;
  const double d5 = ab.dot(cp);
  const double d6 = ac.dot(cp);
  if (d6 >= 0.0 && d5 <= d6)
    return c;

  const double vc = d1 * d4 - d3 * d2;
  if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0)
    return a + (d1 / (d1 - d3)) * ab;

  const double vb = d5 * d2 - d1 * d6;
  if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0)
    return a + (d2 / (d2 - d6)) * ac;

  const double va = d3 * d6 - d5 * d4;
  if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0)
    return b + ((d4 - d3) / ((d4 - d3) + (d5 - d6))) * (c - b);

  const double denom = va + vb + vc;
  if (isZero(denom)) {
    auto closest = closestPointOnSegmentToOrigin(a, b).closest;
    double bestDist2 = closest.squaredNorm();

    const Eigen::Vector3d acClosest
        = closestPointOnSegmentToOrigin(a, c).closest;
    double dist2 = acClosest.squaredNorm();
    if (dist2 < bestDist2) {
      closest = acClosest;
      bestDist2 = dist2;
    }

    const Eigen::Vector3d bcClosest
        = closestPointOnSegmentToOrigin(b, c).closest;
    dist2 = bcClosest.squaredNorm();
    if (dist2 < bestDist2)
      closest = bcClosest;

    return closest;
  }

  const double inv = 1.0 / denom;
  const double v = vb * inv;
  const double w = vc * inv;
  const double u = 1.0 - v - w;
  return u * a + v * b + w * c;
}

void findPos(
    const MprPortal& portal, Eigen::Vector3d& pointA, Eigen::Vector3d& pointB)
{
  Eigen::Vector3d dir;
  portalDir(portal, dir);

  std::array<double, 4> b{};
  Eigen::Vector3d vec;

  vec = portal.points[1].v.cross(portal.points[2].v);
  b[0] = vec.dot(portal.points[3].v);

  vec = portal.points[3].v.cross(portal.points[2].v);
  b[1] = vec.dot(portal.points[0].v);

  vec = portal.points[0].v.cross(portal.points[1].v);
  b[2] = vec.dot(portal.points[3].v);

  vec = portal.points[2].v.cross(portal.points[1].v);
  b[3] = vec.dot(portal.points[0].v);

  double sum = b[0] + b[1] + b[2] + b[3];
  if (isZero(sum) || sum < 0.0) {
    b[0] = 0.0;
    vec = portal.points[2].v.cross(portal.points[3].v);
    b[1] = vec.dot(dir);
    vec = portal.points[3].v.cross(portal.points[1].v);
    b[2] = vec.dot(dir);
    vec = portal.points[1].v.cross(portal.points[2].v);
    b[3] = vec.dot(dir);
    sum = b[1] + b[2] + b[3];
  }

  if (isZero(sum)) {
    pointA = portal.points[1].v1;
    pointB = portal.points[1].v2;
    return;
  }

  const double inv = 1.0 / sum;

  pointA.setZero();
  pointB.setZero();
  for (int i = 0; i < 4; ++i) {
    pointA += b[i] * portal.points[i].v1;
    pointB += b[i] * portal.points[i].v2;
  }
  pointA *= inv;
  pointB *= inv;
}

void findPenetration(
    const SupportShape& supportA,
    const SupportShape& supportB,
    MprPortal& portal,
    MprResult& result)
{
  for (int iter = 0; iter < kMprMaxIterations; ++iter) {
    Eigen::Vector3d dir;
    portalDir(portal, dir);
    SupportPoint v4 = computeSupport(supportA, supportB, dir);

    if (portalReachTolerance(portal, v4, dir)
        || iter + 1 >= kMprMaxIterations) {
      const Eigen::Vector3d closest = closestPointOnTriangleToOrigin(
          portal.points[1].v, portal.points[2].v, portal.points[3].v);
      result.depth = closest.norm();
      if (result.depth < kMprEpsilon)
        result.normal = Eigen::Vector3d::Zero();
      else
        result.normal = closest / result.depth;
      findPos(portal, result.pointOnA, result.pointOnB);
      result.success = true;
      return;
    }

    expandPortal(portal, v4);
  }
}

void findPenetrationTouch(MprPortal& portal, MprResult& result)
{
  result.depth = 0.0;
  result.normal = Eigen::Vector3d::Zero();
  result.pointOnA = portal.points[1].v1;
  result.pointOnB = portal.points[1].v2;
  result.success = true;
}

void findPenetrationSegment(MprPortal& portal, MprResult& result)
{
  result.pointOnA = portal.points[1].v1;
  result.pointOnB = portal.points[1].v2;
  result.normal = portal.points[1].v;
  result.depth = result.normal.norm();
  if (result.depth > kMprEpsilon)
    result.normal /= result.depth;
  else
    result.normal = Eigen::Vector3d::Zero();

  result.success = true;
}

void findPenetrationDegenerateTouch(MprPortal& portal, MprResult& result)
{
  const Eigen::Vector3d closest = closestPointOnTriangleToOrigin(
      portal.points[1].v, portal.points[2].v, portal.points[3].v);
  if (closest.norm() > kMprTolerance)
    return;

  result.depth = 0.0;
  result.normal = Eigen::Vector3d::Zero();
  portalDir(portal, result.normal);
  findPos(portal, result.pointOnA, result.pointOnB);
  result.success = result.pointOnA.allFinite() && result.pointOnB.allFinite()
                   && result.normal.allFinite();
}

MprResult computeMprPenetration(
    const SupportShape& supportA,
    const SupportShape& supportB,
    const Eigen::Vector3d& centerA,
    const Eigen::Vector3d& centerB)
{
  MprResult result;
  MprPortal portal;

  const int res = discoverPortal(supportA, supportB, centerA, centerB, portal);
  if (res < 0)
    return result;

  if (res == 1) {
    findPenetrationTouch(portal, result);
    return result;
  }

  if (res == 2) {
    findPenetrationSegment(portal, result);
    return result;
  }

  bool mayTouch = false;
  if (refinePortal(supportA, supportB, portal, mayTouch) < 0) {
    if (mayTouch)
      findPenetrationDegenerateTouch(portal, result);
    return result;
  }

  findPenetration(supportA, supportB, portal, result);
  return result;
}

SupportShape makeBoxSupportShape(
    const Eigen::Vector3d& size, const Eigen::Isometry3d& transform)
{
  SupportShape shape;
  shape.type = SupportShapeType::Box;
  shape.size = size;
  shape.transform = transform;
  return shape;
}

SupportShape makeCylinderSupportShape(
    double radius, double halfHeight, const Eigen::Isometry3d& transform)
{
  SupportShape shape;
  shape.type = SupportShapeType::Cylinder;
  shape.radius = radius;
  shape.halfHeight = halfHeight;
  shape.transform = transform;
  return shape;
}

int collideSupportMappedShapes(
    CollisionObject* o1,
    CollisionObject* o2,
    const SupportShape& supportA,
    const SupportShape& supportB,
    CollisionResult& result)
{
  const auto mpr = computeMprPenetration(
      supportA, supportB, supportA.center(), supportB.center());
  if (!mpr.success)
    return 0;
  if (!mpr.pointOnA.allFinite() || !mpr.pointOnB.allFinite()
      || !mpr.normal.allFinite() || !std::isfinite(mpr.depth)) {
    return 0;
  }

  Contact contact;
  contact.collisionObject1 = o1;
  contact.collisionObject2 = o2;
  contact.point = 0.5 * (mpr.pointOnA + mpr.pointOnB);
  contact.normal = -mpr.normal;
  if (contact.normal.squaredNorm() < 1e-12) {
    contact.normal = supportA.center() - supportB.center();
    if (contact.normal.squaredNorm() < 1e-12)
      contact.normal = Eigen::Vector3d::UnitZ();
    else
      contact.normal.normalize();
  } else {
    contact.normal.normalize();
  }
  contact.penetrationDepth = std::abs(mpr.depth);
  result.addContact(contact);
  return 1;
}

} // namespace

int collideBoxCylinder(
    CollisionObject* o1,
    CollisionObject* o2,
    const Eigen::Vector3d& size0,
    const Eigen::Isometry3d& T0,
    const double& cyl_rad,
    const double& half_height,
    const Eigen::Isometry3d& T1,
    CollisionResult& result)
{
  return collideSupportMappedShapes(
      o1,
      o2,
      makeBoxSupportShape(size0, T0),
      makeCylinderSupportShape(cyl_rad, half_height, T1),
      result);
}

int collideCylinderBox(
    CollisionObject* o1,
    CollisionObject* o2,
    const double& cyl_rad,
    const double& half_height,
    const Eigen::Isometry3d& T0,
    const Eigen::Vector3d& size1,
    const Eigen::Isometry3d& T1,
    CollisionResult& result)
{
  return collideSupportMappedShapes(
      o1,
      o2,
      makeCylinderSupportShape(cyl_rad, half_height, T0),
      makeBoxSupportShape(size1, T1),
      result);
}

int collideCylinderCylinder(
    CollisionObject* o1,
    CollisionObject* o2,
    const double& cyl_rad1,
    const double& half_height1,
    const Eigen::Isometry3d& T0,
    const double& cyl_rad2,
    const double& half_height2,
    const Eigen::Isometry3d& T1,
    CollisionResult& result)
{
  return collideSupportMappedShapes(
      o1,
      o2,
      makeCylinderSupportShape(cyl_rad1, half_height1, T0),
      makeCylinderSupportShape(cyl_rad2, half_height2, T1),
      result);
}

int collideCylinderSphere(
    CollisionObject* o1,
    CollisionObject* o2,
    const double& cyl_rad,
    const double& half_height,
    const Eigen::Isometry3d& T0,
    const double& sphere_rad,
    const Eigen::Isometry3d& T1,
    CollisionResult& result)
{
  Eigen::Vector3d center = T0.inverse() * T1.translation();

  double dist = sqrt(center[0] * center[0] + center[1] * center[1]);
  const double penetration = cyl_rad + sphere_rad - dist;
  const double absZ = std::abs(center[2]);

  if (dist <= cyl_rad && absZ <= half_height + sphere_rad) {
    const double capSign = center[2] >= 0.0 ? 1.0 : -1.0;
    const double capPenetration
        = half_height + sphere_rad - capSign * center[2];

    if (absZ <= half_height && penetration <= capPenetration) {
      Eigen::Vector3d point = getCylinderRadialDirection(center, dist);
      Eigen::Vector3d normal = -(T0.linear() * point);
      point *= (cyl_rad - 0.5 * penetration);
      point[2] = center[2];
      point = T0 * point;

      Contact contact;
      contact.collisionObject1 = o1;
      contact.collisionObject2 = o2;
      contact.point = point;
      contact.normal = normal;
      contact.penetrationDepth = penetration;
      result.addContact(contact);
      return 1;
    }

    Contact contact;
    contact.collisionObject1 = o1;
    contact.collisionObject2 = o2;
    contact.penetrationDepth = capPenetration;
    contact.point
        = T0
          * Eigen::Vector3d(
              center[0],
              center[1],
              capSign * (half_height - 0.5 * contact.penetrationDepth));
    contact.normal = -(T0.linear() * Eigen::Vector3d(0.0, 0.0, capSign));
    result.addContact(contact);
    return 1;
  } else {
    if (penetration >= 0.0) {
      if (std::abs(center[2]) > half_height) {
        Eigen::Vector3d point = getCylinderRadialDirection(center, dist);
        point *= cyl_rad;
        point[2] = math::sign(center[2]) * half_height;
        Eigen::Vector3d normal = point - center;
        const double edgePenetration = sphere_rad - normal.norm();
        normal = (T0.linear() * normal).normalized();
        point = T0 * point;

        if (edgePenetration >= 0.0) {
          Contact contact;
          contact.collisionObject1 = o1;
          contact.collisionObject2 = o2;
          contact.point = point;
          contact.normal = normal;
          contact.penetrationDepth = edgePenetration;
          result.addContact(contact);
          return 1;
        }
      } else // if( center[2] >= -half_height && center[2] <= half_height )
      {
        Eigen::Vector3d point = getCylinderRadialDirection(center, dist);
        Eigen::Vector3d normal = -(T0.linear() * point);
        point *= (cyl_rad - 0.5 * penetration);
        point[2] = center[2];
        point = T0 * point;

        Contact contact;
        contact.collisionObject1 = o1;
        contact.collisionObject2 = o2;
        contact.point = point;
        contact.normal = normal;
        contact.penetrationDepth = penetration;
        result.addContact(contact);
        return 1;
      }
    }
  }
  return 0;
}

int collideSphereCylinder(
    CollisionObject* o1,
    CollisionObject* o2,
    const double& sphere_rad,
    const Eigen::Isometry3d& T0,
    const double& cyl_rad,
    const double& half_height,
    const Eigen::Isometry3d& T1,
    CollisionResult& result)
{
  Eigen::Vector3d center = T1.inverse() * T0.translation();

  double dist = sqrt(center[0] * center[0] + center[1] * center[1]);
  const double penetration = cyl_rad + sphere_rad - dist;
  const double absZ = std::abs(center[2]);

  if (dist <= cyl_rad && absZ <= half_height + sphere_rad) {
    const double capSign = center[2] >= 0.0 ? 1.0 : -1.0;
    const double capPenetration
        = half_height + sphere_rad - capSign * center[2];

    if (absZ <= half_height && penetration <= capPenetration) {
      Eigen::Vector3d point = getCylinderRadialDirection(center, dist);
      Eigen::Vector3d normal = T1.linear() * point;
      point *= (cyl_rad - 0.5 * penetration);
      point[2] = center[2];
      point = T1 * point;

      Contact contact;
      contact.collisionObject1 = o1;
      contact.collisionObject2 = o2;
      contact.point = point;
      contact.normal = normal;
      contact.penetrationDepth = penetration;
      result.addContact(contact);
      return 1;
    }

    Contact contact;
    contact.collisionObject1 = o1;
    contact.collisionObject2 = o2;
    contact.penetrationDepth = capPenetration;
    contact.point
        = T1
          * Eigen::Vector3d(
              center[0],
              center[1],
              capSign * (half_height - 0.5 * contact.penetrationDepth));
    contact.normal = T1.linear() * Eigen::Vector3d(0.0, 0.0, capSign);
    result.addContact(contact);
    return 1;
  } else {
    if (penetration >= 0.0) {
      if (std::abs(center[2]) > half_height) {
        Eigen::Vector3d point = getCylinderRadialDirection(center, dist);
        point *= cyl_rad;
        point[2] = math::sign(center[2]) * half_height;
        Eigen::Vector3d normal = point - center;
        const double edgePenetration = sphere_rad - normal.norm();
        normal = -(T1.linear() * normal).normalized();
        point = T1 * point;

        if (edgePenetration >= 0.0) {
          Contact contact;
          contact.collisionObject1 = o1;
          contact.collisionObject2 = o2;
          contact.point = point;
          contact.normal = normal;
          contact.penetrationDepth = edgePenetration;
          result.addContact(contact);
          return 1;
        }
      } else // if( center[2] >= -half_height && center[2] <= half_height )
      {
        Eigen::Vector3d point = getCylinderRadialDirection(center, dist);
        Eigen::Vector3d normal = T1.linear() * point;
        point *= (cyl_rad - 0.5 * penetration);
        point[2] = center[2];
        point = T1 * point;

        Contact contact;
        contact.collisionObject1 = o1;
        contact.collisionObject2 = o2;
        contact.point = point;
        contact.normal = normal;
        contact.penetrationDepth = penetration;
        result.addContact(contact);
        return 1;
      }
    }
  }
  return 0;
}

int collideCylinderPlane(
    CollisionObject* o1,
    CollisionObject* o2,
    const double& cyl_rad,
    const double& half_height,
    const Eigen::Isometry3d& T0,
    const Eigen::Vector3d& plane_normal,
    const double& plane_offset,
    const Eigen::Isometry3d& T1,
    CollisionResult& result)
{
  const Eigen::Vector3d worldNormal = T1.linear() * plane_normal;
  const Eigen::Vector3d planePoint
      = T1.translation() + worldNormal * plane_offset;
  const Eigen::Vector3d cylAxis = T0.linear().col(2);
  const Eigen::Vector3d cylCenter = T0.translation();
  const Eigen::Vector3d cylTop = cylCenter + cylAxis * half_height;
  const Eigen::Vector3d cylBot = cylCenter - cylAxis * half_height;
  const double dotAxis = cylAxis.dot(worldNormal);
  const Eigen::Vector3d radialRaw = worldNormal - cylAxis * dotAxis;

  Eigen::Vector3d deepestPoint;
  if (radialRaw.squaredNorm() < 1e-10) {
    const double distTop = worldNormal.dot(cylTop - planePoint);
    const double distBot = worldNormal.dot(cylBot - planePoint);
    deepestPoint = (distTop < distBot) ? cylTop : cylBot;
  } else {
    const Eigen::Vector3d radialDir = radialRaw.normalized();
    const Eigen::Vector3d rimOffset = -radialDir * cyl_rad;
    const Eigen::Vector3d topRim = cylTop + rimOffset;
    const Eigen::Vector3d botRim = cylBot + rimOffset;

    const double distTopRim = worldNormal.dot(topRim - planePoint);
    const double distBotRim = worldNormal.dot(botRim - planePoint);

    deepestPoint = (distTopRim < distBotRim) ? topRim : botRim;
  }

  const double signedDist = worldNormal.dot(deepestPoint - planePoint);
  if (signedDist > 0.0)
    return 0;

  const double penetration = -signedDist;

  Contact contact;
  contact.collisionObject1 = o1;
  contact.collisionObject2 = o2;
  contact.point = deepestPoint + worldNormal * (penetration * 0.5);
  contact.normal = worldNormal;
  contact.penetrationDepth = penetration;
  result.addContact(contact);
  return 1;
}

int collideCylinderPlane(
    CollisionObject* o1,
    CollisionObject* o2,
    const double& cyl_rad,
    const double& half_height,
    const Eigen::Isometry3d& T0,
    const Eigen::Vector3d& plane_normal,
    const Eigen::Isometry3d& T1,
    CollisionResult& result)
{
  return collideCylinderPlane(
      o1, o2, cyl_rad, half_height, T0, plane_normal, 0.0, T1, result);
}

int collidePlaneCylinder(
    CollisionObject* o1,
    CollisionObject* o2,
    const Eigen::Vector3d& plane_normal,
    const double& plane_offset,
    const Eigen::Isometry3d& T0,
    const double& cyl_rad,
    const double& half_height,
    const Eigen::Isometry3d& T1,
    CollisionResult& result)
{
  const Eigen::Vector3d worldNormal = T0.linear() * plane_normal;
  const Eigen::Vector3d planePoint
      = T0.translation() + worldNormal * plane_offset;
  const Eigen::Vector3d cylAxis = T1.linear().col(2);
  const Eigen::Vector3d cylCenter = T1.translation();
  const Eigen::Vector3d cylTop = cylCenter + cylAxis * half_height;
  const Eigen::Vector3d cylBot = cylCenter - cylAxis * half_height;
  const double dotAxis = cylAxis.dot(worldNormal);
  const Eigen::Vector3d radialRaw = worldNormal - cylAxis * dotAxis;

  Eigen::Vector3d deepestPoint;
  if (radialRaw.squaredNorm() < 1e-10) {
    const double distTop = worldNormal.dot(cylTop - planePoint);
    const double distBot = worldNormal.dot(cylBot - planePoint);
    deepestPoint = (distTop < distBot) ? cylTop : cylBot;
  } else {
    const Eigen::Vector3d radialDir = radialRaw.normalized();
    const Eigen::Vector3d rimOffset = -radialDir * cyl_rad;
    const Eigen::Vector3d topRim = cylTop + rimOffset;
    const Eigen::Vector3d botRim = cylBot + rimOffset;

    const double distTopRim = worldNormal.dot(topRim - planePoint);
    const double distBotRim = worldNormal.dot(botRim - planePoint);

    deepestPoint = (distTopRim < distBotRim) ? topRim : botRim;
  }

  const double signedDist = worldNormal.dot(deepestPoint - planePoint);
  if (signedDist > 0.0)
    return 0;

  const double penetration = -signedDist;

  Contact contact;
  contact.collisionObject1 = o1;
  contact.collisionObject2 = o2;
  contact.point = deepestPoint + worldNormal * (penetration * 0.5);
  contact.normal = -worldNormal;
  contact.penetrationDepth = penetration;
  result.addContact(contact);
  return 1;
}

//==============================================================================
int collide(CollisionObject* o1, CollisionObject* o2, CollisionResult& result)
{
  // TODO(JS): We could make the contact point computation as optional for
  // the case that we want only binary check.

  const auto& shape1 = o1->getShape();
  const auto& shape2 = o2->getShape();

  const auto& shapeType1 = shape1->getType();
  const auto& shapeType2 = shape2->getType();

  const Eigen::Isometry3d& T1 = o1->getTransform();
  const Eigen::Isometry3d& T2 = o2->getTransform();

  if (dynamics::SphereShape::getStaticType() == shapeType1) {
    const auto* sphere0
        = static_cast<const dynamics::SphereShape*>(shape1.get());

    if (dynamics::SphereShape::getStaticType() == shapeType2) {
      const auto* sphere1
          = static_cast<const dynamics::SphereShape*>(shape2.get());

      return collideSphereSphere(
          o1, o2, sphere0->getRadius(), T1, sphere1->getRadius(), T2, result);
    } else if (dynamics::BoxShape::getStaticType() == shapeType2) {
      const auto* box1 = static_cast<const dynamics::BoxShape*>(shape2.get());

      return collideSphereBox(
          o1, o2, sphere0->getRadius(), T1, box1->getSize(), T2, result);
    } else if (dynamics::PlaneShape::getStaticType() == shapeType2) {
      const auto* plane1
          = static_cast<const dynamics::PlaneShape*>(shape2.get());

      return collideSpherePlane(
          o1,
          o2,
          sphere0->getRadius(),
          T1,
          plane1->getNormal(),
          plane1->getOffset(),
          T2,
          result);
    } else if (dynamics::CylinderShape::getStaticType() == shapeType2) {
      const auto* cylinder1
          = static_cast<const dynamics::CylinderShape*>(shape2.get());

      return collideSphereCylinder(
          o1,
          o2,
          sphere0->getRadius(),
          T1,
          cylinder1->getRadius(),
          0.5 * cylinder1->getHeight(),
          T2,
          result);
    } else if (dynamics::EllipsoidShape::getStaticType() == shapeType2) {
      const auto* ellipsoid1
          = static_cast<const dynamics::EllipsoidShape*>(shape2.get());

      return collideSphereSphere(
          o1,
          o2,
          sphere0->getRadius(),
          T1,
          ellipsoid1->getRadii()[0],
          T2,
          result);
    }
  } else if (dynamics::BoxShape::getStaticType() == shapeType1) {
    const auto* box0 = static_cast<const dynamics::BoxShape*>(shape1.get());

    if (dynamics::SphereShape::getStaticType() == shapeType2) {
      const auto* sphere1
          = static_cast<const dynamics::SphereShape*>(shape2.get());

      return collideBoxSphere(
          o1, o2, box0->getSize(), T1, sphere1->getRadius(), T2, result);
    } else if (dynamics::BoxShape::getStaticType() == shapeType2) {
      const auto* box1 = static_cast<const dynamics::BoxShape*>(shape2.get());

      return collideBoxBox(
          o1, o2, box0->getSize(), T1, box1->getSize(), T2, result);
    } else if (dynamics::PlaneShape::getStaticType() == shapeType2) {
      const auto* plane1
          = static_cast<const dynamics::PlaneShape*>(shape2.get());

      return collideBoxPlane(
          o1,
          o2,
          box0->getSize(),
          T1,
          plane1->getNormal(),
          plane1->getOffset(),
          T2,
          result);
    } else if (dynamics::CylinderShape::getStaticType() == shapeType2) {
      const auto* cylinder1
          = static_cast<const dynamics::CylinderShape*>(shape2.get());

      return collideBoxCylinder(
          o1,
          o2,
          box0->getSize(),
          T1,
          cylinder1->getRadius(),
          0.5 * cylinder1->getHeight(),
          T2,
          result);
    } else if (dynamics::EllipsoidShape::getStaticType() == shapeType2) {
      const auto* ellipsoid1
          = static_cast<const dynamics::EllipsoidShape*>(shape2.get());

      return collideBoxSphere(
          o1, o2, box0->getSize(), T1, ellipsoid1->getRadii()[0], T2, result);
    }
  } else if (dynamics::EllipsoidShape::getStaticType() == shapeType1) {
    const auto* ellipsoid0
        = static_cast<const dynamics::EllipsoidShape*>(shape1.get());

    if (dynamics::SphereShape::getStaticType() == shapeType2) {
      const auto* sphere1
          = static_cast<const dynamics::SphereShape*>(shape2.get());

      return collideSphereSphere(
          o1,
          o2,
          ellipsoid0->getRadii()[0],
          T1,
          sphere1->getRadius(),
          T2,
          result);
    } else if (dynamics::BoxShape::getStaticType() == shapeType2) {
      const auto* box1 = static_cast<const dynamics::BoxShape*>(shape2.get());

      return collideSphereBox(
          o1, o2, ellipsoid0->getRadii()[0], T1, box1->getSize(), T2, result);
    } else if (dynamics::CylinderShape::getStaticType() == shapeType2) {
      const auto* cylinder1
          = static_cast<const dynamics::CylinderShape*>(shape2.get());

      if (ellipsoid0->isSphere()) {
        return collideSphereCylinder(
            o1,
            o2,
            ellipsoid0->getRadii()[0],
            T1,
            cylinder1->getRadius(),
            0.5 * cylinder1->getHeight(),
            T2,
            result);
      }
    } else if (dynamics::PlaneShape::getStaticType() == shapeType2) {
      const auto* plane1
          = static_cast<const dynamics::PlaneShape*>(shape2.get());

      if (ellipsoid0->isSphere()) {
        return collideSpherePlane(
            o1,
            o2,
            ellipsoid0->getRadii()[0],
            T1,
            plane1->getNormal(),
            plane1->getOffset(),
            T2,
            result);
      }
    } else if (dynamics::EllipsoidShape::getStaticType() == shapeType2) {
      const auto* ellipsoid1
          = static_cast<const dynamics::EllipsoidShape*>(shape2.get());

      return collideSphereSphere(
          o1,
          o2,
          ellipsoid0->getRadii()[0],
          T1,
          ellipsoid1->getRadii()[0],
          T2,
          result);
    }
  } else if (dynamics::CylinderShape::getStaticType() == shapeType1) {
    const auto* cylinder0
        = static_cast<const dynamics::CylinderShape*>(shape1.get());

    if (dynamics::SphereShape::getStaticType() == shapeType2) {
      const auto* sphere1
          = static_cast<const dynamics::SphereShape*>(shape2.get());

      return collideCylinderSphere(
          o1,
          o2,
          cylinder0->getRadius(),
          0.5 * cylinder0->getHeight(),
          T1,
          sphere1->getRadius(),
          T2,
          result);
    } else if (dynamics::EllipsoidShape::getStaticType() == shapeType2) {
      const auto* ellipsoid1
          = static_cast<const dynamics::EllipsoidShape*>(shape2.get());

      if (ellipsoid1->isSphere()) {
        return collideCylinderSphere(
            o1,
            o2,
            cylinder0->getRadius(),
            0.5 * cylinder0->getHeight(),
            T1,
            ellipsoid1->getRadii()[0],
            T2,
            result);
      }
    } else if (dynamics::BoxShape::getStaticType() == shapeType2) {
      const auto* box1 = static_cast<const dynamics::BoxShape*>(shape2.get());

      return collideCylinderBox(
          o1,
          o2,
          cylinder0->getRadius(),
          0.5 * cylinder0->getHeight(),
          T1,
          box1->getSize(),
          T2,
          result);
    } else if (dynamics::CylinderShape::getStaticType() == shapeType2) {
      const auto* cylinder1
          = static_cast<const dynamics::CylinderShape*>(shape2.get());

      return collideCylinderCylinder(
          o1,
          o2,
          cylinder0->getRadius(),
          0.5 * cylinder0->getHeight(),
          T1,
          cylinder1->getRadius(),
          0.5 * cylinder1->getHeight(),
          T2,
          result);
    } else if (dynamics::PlaneShape::getStaticType() == shapeType2) {
      const auto* plane1
          = static_cast<const dynamics::PlaneShape*>(shape2.get());

      return collideCylinderPlane(
          o1,
          o2,
          cylinder0->getRadius(),
          0.5 * cylinder0->getHeight(),
          T1,
          plane1->getNormal(),
          plane1->getOffset(),
          T2,
          result);
    }
  } else if (dynamics::PlaneShape::getStaticType() == shapeType1) {
    const auto* plane0 = static_cast<const dynamics::PlaneShape*>(shape1.get());

    if (dynamics::SphereShape::getStaticType() == shapeType2) {
      const auto* sphere1
          = static_cast<const dynamics::SphereShape*>(shape2.get());

      return collidePlaneSphere(
          o1,
          o2,
          plane0->getNormal(),
          plane0->getOffset(),
          T1,
          sphere1->getRadius(),
          T2,
          result);
    } else if (dynamics::BoxShape::getStaticType() == shapeType2) {
      const auto* box1 = static_cast<const dynamics::BoxShape*>(shape2.get());

      return collidePlaneBox(
          o1,
          o2,
          plane0->getNormal(),
          plane0->getOffset(),
          T1,
          box1->getSize(),
          T2,
          result);
    } else if (dynamics::CylinderShape::getStaticType() == shapeType2) {
      const auto* cylinder1
          = static_cast<const dynamics::CylinderShape*>(shape2.get());

      return collidePlaneCylinder(
          o1,
          o2,
          plane0->getNormal(),
          plane0->getOffset(),
          T1,
          cylinder1->getRadius(),
          0.5 * cylinder1->getHeight(),
          T2,
          result);
    } else if (dynamics::EllipsoidShape::getStaticType() == shapeType2) {
      const auto* ellipsoid1
          = static_cast<const dynamics::EllipsoidShape*>(shape2.get());

      if (ellipsoid1->isSphere()) {
        return collidePlaneSphere(
            o1,
            o2,
            plane0->getNormal(),
            plane0->getOffset(),
            T1,
            ellipsoid1->getRadii()[0],
            T2,
            result);
      }
    }
  }

  dterr << "[DARTCollisionDetector] Attempting to check for an "
        << "unsupported shape pair: [" << shape1->getType() << "] - ["
        << shape2->getType() << "]. Returning false.\n";

  return false;
}

} // namespace collision
} // namespace dart
