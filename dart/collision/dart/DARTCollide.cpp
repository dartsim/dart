/*
 * Copyright (c) 2013, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
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

#include "dart/collision/dart/DARTCollide.h"

#include <memory>

#include "dart/math/Helpers.h"
#include "dart/dynamics/BoxShape.h"
#include "dart/dynamics/EllipsoidShape.h"
#include "dart/dynamics/CylinderShape.h"

namespace dart {
namespace collision {

// point : world coordinate vector
// normal : normal vector from right to left 0 <- 1
// penetration : real positive means penetration

#define DART_COLLISION_EPS  1E-6
static const int MAX_CYLBOX_CLIP_POINTS  = 16;
static const int nCYLINDER_AXIS			 = 2;
// Number of segment of cylinder base circle.
// Must be divisible by 4.
static const int nCYLINDER_SEGMENT		 = 8;

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
  R0[0] = T0(0,0);   R0[1] = T0(0,1);   R0[ 2] = T0(0,2);   R0[ 3] = T0(0,3);
  R0[4] = T0(1,0);   R0[5] = T0(1,1);   R0[ 6] = T0(1,2);   R0[ 7] = T0(1,3);
  R0[8] = T0(2,0);   R0[9] = T0(2,1);   R0[10] = T0(2,2);   R0[11] = T0(2,3);
}

struct dContactGeom
{
  dVector3 pos;
  double depth;
};

inline double Inner (const double *a, const double *b)
{ return ((a)[0]*(b)[0] + (a)[1]*(b)[1] + (a)[2]*(b)[2]); }

inline double Inner14(const double *a, const double *b)
{ return ((a)[0]*(b)[0] + (a)[1]*(b)[4] + (a)[2]*(b)[8]); }

inline double Inner41(const double *a, const double *b)
{ return ((a)[0]*(b)[0] + (a)[4]*(b)[1] + (a)[8]*(b)[2]); }

inline double Inner44(const double *a, const double *b)
{ return ((a)[0]*(b)[0] + (a)[4]*(b)[4] + (a)[8]*(b)[8]); }

#define dMULTIPLYOP0_331(A,op,B,C) \
  (A)[0] op Inner((B),(C)); \
  (A)[1] op Inner((B+4),(C)); \
  (A)[2] op Inner((B+8),(C));

#define dMULTIPLYOP1_331(A,op,B,C) \
  (A)[0] op Inner41((B),(C)); \
  (A)[1] op Inner41((B+1),(C)); \
  (A)[2] op Inner41((B+2),(C));

inline void dMULTIPLY0_331(double *A, const double *B, const double *C)
{ dMULTIPLYOP0_331(A,=,B,C) }

inline void dMULTIPLY1_331(double *A, const double *B, const double *C)
{ dMULTIPLYOP1_331(A,=,B,C) }

#define dRecip(x) (1.0/(x))

// given n points in the plane (array p, of size 2*n), generate m points that
// best represent the whole set. the definition of 'best' here is not
// predetermined - the idea is to select points that give good box-box
// collision detection behavior. the chosen point indexes are returned in the
// array iret (of size m). 'i0' is always the first entry in the array.
// n must be in the range [1..8]. m must be in the range [1..n]. i0 must be
// in the range [0..n-1].

void cullPoints (int n, double p[], int m, int i0, int iret[])
{
  // compute the centroid of the polygon in cx,cy
  int i,j;
  double a,cx,cy,q;
  if (n==1) {
    cx = p[0];
    cy = p[1];
  }
  else if (n==2) {
    cx = 0.5*(p[0] + p[2]);
    cy = 0.5*(p[1] + p[3]);
  }
  else {
    a = 0;
    cx = 0;
    cy = 0;
    for (i=0; i<(n-1); i++) {
      q = p[i*2]*p[i*2+3] - p[i*2+2]*p[i*2+1];
      a += q;
      cx += q*(p[i*2]+p[i*2+2]);
      cy += q*(p[i*2+1]+p[i*2+3]);
    }
    q = p[n*2-2]*p[1] - p[0]*p[n*2-1];
    a = dRecip(3.0*(a+q));
    cx = a*(cx + q*(p[n*2-2]+p[0]));
    cy = a*(cy + q*(p[n*2-1]+p[1]));
  }

  // compute the angle of each point w.r.t. the centroid
  double A[8];
  for (i=0; i<n; i++) A[i] = atan2(p[i*2+1]-cy,p[i*2]-cx);

  // search for points that have angles closest to A[i0] + i*(2*pi/m).
  int avail[8];
  for (i=0; i<n; i++) avail[i] = 1;
  avail[i0] = 0;
  iret[0] = i0;
  iret++;
  for (j=1; j<m; j++) {
    a = double(j)*(2*DART_PI/m) + A[i0];
    if (a > DART_PI) a -= 2*DART_PI;
    double maxdiff=1e9,diff;
    for (i=0; i<n; i++) {
      if (avail[i]) {
        diff = fabs (A[i]-a);
        if (diff > DART_PI) diff = 2*DART_PI - diff;
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

void dLineClosestApproach (const dVector3 pa, const dVector3 ua,
                           const dVector3 pb, const dVector3 ub,
                           double *alpha, double *beta)
{
  dVector3 p;
  p[0] = pb[0] - pa[0];
  p[1] = pb[1] - pa[1];
  p[2] = pb[2] - pa[2];
  double uaub = Inner(ua,ub);
  double q1 =  Inner(ua,p);
  double q2 = -Inner(ub,p);
  double d = 1-uaub*uaub;
  if (d <= 0) {
    // @@@ this needs to be made more robust
    *alpha = 0;
    *beta  = 0;
  }
  else {
    d = dRecip(d);
    *alpha = (q1 + uaub*q2)*d;
    *beta  = (uaub*q1 + q2)*d;
  }
}

int intersectRectQuad (double h[2], double p[8], double ret[16])
{
  // q (and r) contain nq (and nr) coordinate points for the current (and
  // chopped) polygons
  int nq=4,nr=0;
  double buffer[16];
  double *q = p;
  double *r = ret;
  for (int dir=0; dir <= 1; dir++) {
    // direction notation: xy[0] = x axis, xy[1] = y axis
    for (int sign=-1; sign <= 1; sign += 2) {
      // chop q along the line xy[dir] = sign*h[dir]
      double *pq = q;
      double *pr = r;
      nr = 0;
      for (int i=nq; i > 0; i--) {
        // go through all points in q and all lines between adjacent points
        if (sign*pq[dir] < h[dir]) {
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
        double *nextq = (i > 1) ? pq+2 : q;
        if ((sign*pq[dir] < h[dir]) ^ (sign*nextq[dir] < h[dir])) {
          // this line crosses the chopping line
          pr[1-dir] = pq[1-dir] + (nextq[1-dir]-pq[1-dir]) /
              (nextq[dir]-pq[dir]) * (sign*h[dir]-pq[dir]);
          pr[dir] = sign*h[dir];
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
      r = (q==ret) ? buffer : ret;
      nq = nr;
    }
  }
done:
  if (q != ret) memcpy (ret,q,nr*2*sizeof(double));
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

void dClosestLineBoxPoints (const dVector3 p1, const dVector3 p2,
                            const dVector3 c, const dMatrix3 R,
                            const dVector3 side,
                            dVector3 lret, dVector3 bret)
{
  int i;

  // compute the start and delta of the line p1-p2 relative to the box.
  // we will do all subsequent computations in this box-relative coordinate
  // system. we have to do a translation and rotation for each point.
  dVector3 tmp,s,v;
  tmp[0] = p1[0] - c[0];
  tmp[1] = p1[1] - c[1];
  tmp[2] = p1[2] - c[2];
  dMULTIPLY1_331 (s,R,tmp);
  tmp[0] = p2[0] - p1[0];
  tmp[1] = p2[1] - p1[1];
  tmp[2] = p2[2] - p1[2];
  dMULTIPLY1_331 (v,R,tmp);

  // mirror the line so that v has all components >= 0
  dVector3 sign;
  for (i=0; i<3; i++) {
    if (v[i] < 0) {
      s[i] = -s[i];
      v[i] = -v[i];
      sign[i] = -1;
    }
    else sign[i] = 1;
  }

  // compute v^2
  dVector3 v2;
  v2[0] = v[0]*v[0];
  v2[1] = v[1]*v[1];
  v2[2] = v[2]*v[2];

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
  for (i=0; i<3; i++) {
    if (v[i] > 0) {
      if (s[i] < -h[i]) {
        region[i] = -1;
        tanchor[i] = (-h[i]-s[i])/v[i];
      }
      else {
        region[i] = (s[i] > h[i]);
        tanchor[i] = (h[i]-s[i])/v[i];
      }
    }
    else {
      region[i] = 0;
      tanchor[i] = 2;		// this will never be a valid tanchor
    }
  }

  // compute d|d|^2/dt for t=0. if it's >= 0 then p1 is the closest point
  double t=0;
  double dd2dt = 0;
  for (i=0; i<3; i++) dd2dt -= (region[i] ? v2[i] : 0) * tanchor[i];
  if (dd2dt >= 0) goto got_answer;

  do {
    // find the point on the line that is at the next clip plane boundary
    double next_t = 1;
    for (i=0; i<3; i++) {
      if (tanchor[i] > t && tanchor[i] < 1 && tanchor[i] < next_t)
        next_t = tanchor[i];
    }

    // compute d|d|^2/dt for the next t
    double next_dd2dt = 0;
    for (i=0; i<3; i++) {
      next_dd2dt += (region[i] ? v2[i] : 0) * (next_t - tanchor[i]);
    }

    // if the sign of d|d|^2/dt has changed, solution = the crossover point
    if (next_dd2dt >= 0) {
      double m = (next_dd2dt-dd2dt)/(next_t - t);
      t -= dd2dt/m;
      goto got_answer;
    }

    // advance to the next anchor point / region
    for (i=0; i<3; i++) {
      if (tanchor[i] == next_t) {
        tanchor[i] = (h[i]-s[i])/v[i];
        region[i]++;
      }
    }
    t = next_t;
    dd2dt = next_dd2dt;
  }
  while (t < 1);
  t = 1;

got_answer:

  // compute closest point on the line
  for (i=0; i<3; i++) lret[i] = p1[i] + t*tmp[i];	// note: tmp=p2-p1

  // compute closest point on the box
  for (i=0; i<3; i++) {
    tmp[i] = sign[i] * (s[i] + t*v[i]);
    if (tmp[i] < -h[i]) tmp[i] = -h[i];
    else if (tmp[i] > h[i]) tmp[i] = h[i];
  }
  dMULTIPLY0_331 (s,R,tmp);
  for (i=0; i<3; i++) bret[i] = s[i] + c[i];
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
int dBoxBox(const dVector3 p1, const dMatrix3 R1, const dVector3 side1,
            const dVector3 p2, const dMatrix3 R2, const dVector3 side2,
            std::vector<Contact>& result)
{
  const double fudge_factor = 1.05;
  dVector3 p,pp,normalC;
  const double *normalR = 0;
  double A[3],B[3],R11,R12,R13,R21,R22,R23,R31,R32,R33,Q11,Q12,Q13,Q21,Q22,Q23,Q31,Q32,Q33,s,s2,l;
  int i,j,invert_normal,code;

  // get vector from centers of box 1 to box 2, relative to box 1
  p[0] = p2[0] - p1[0];
  p[1] = p2[1] - p1[1];
  p[2] = p2[2] - p1[2];
  dMULTIPLY1_331 (pp,R1,p);		// get pp = p relative to body 1

  // get side lengths / 2
  A[0] = side1[0];
  A[1] = side1[1];
  A[2] = side1[2];
  B[0] = side2[0];
  B[1] = side2[1];
  B[2] = side2[2];

  // Rij is R1'*R2, i.e. the relative rotation between R1 and R2
  R11 = Inner44(R1+0,R2+0); R12 = Inner44(R1+0,R2+1); R13 = Inner44(R1+0,R2+2);
  R21 = Inner44(R1+1,R2+0); R22 = Inner44(R1+1,R2+1); R23 = Inner44(R1+1,R2+2);
  R31 = Inner44(R1+2,R2+0); R32 = Inner44(R1+2,R2+1); R33 = Inner44(R1+2,R2+2);

  Q11 = fabs(R11); Q12 = fabs(R12); Q13 = fabs(R13);
  Q21 = fabs(R21); Q22 = fabs(R22); Q23 = fabs(R23);
  Q31 = fabs(R31); Q32 = fabs(R32); Q33 = fabs(R33);

  // for all 15 possible separating axes:
  //   * see if the axis separates the boxes. if so, return 0.
  //   * find the depth of the penetration along the separating axis (s2)
  //   * if this is the largest depth so far, record it.
  // the normal vector will be set to the separating axis with the smallest
  // depth. note: normalR is set to point to a column of R1 or R2 if that is
  // the smallest depth normal so far. otherwise normalR is 0 and normalC is
  // set to a vector relative to body 1. invert_normal is 1 if the sign of
  // the normal should be flipped.

#define TST(expr1,expr2,norm,cc) \
  s2 = fabs(expr1) - (expr2); \
  if (s2 > s) { \
  s = s2; \
  normalR = norm; \
  invert_normal = ((expr1) < 0); \
  code = (cc); \
}

  s = -1E12;
  invert_normal = 0;
  code = 0;

  // separating axis = u1,u2,u3
  TST (pp[0],(A[0] + B[0]*Q11 + B[1]*Q12 + B[2]*Q13),R1+0,1);
  TST (pp[1],(A[1] + B[0]*Q21 + B[1]*Q22 + B[2]*Q23),R1+1,2);
  TST (pp[2],(A[2] + B[0]*Q31 + B[1]*Q32 + B[2]*Q33),R1+2,3);

  // separating axis = v1,v2,v3
  TST (Inner41(R2+0,p),(A[0]*Q11 + A[1]*Q21 + A[2]*Q31 + B[0]),R2+0,4);
  TST (Inner41(R2+1,p),(A[0]*Q12 + A[1]*Q22 + A[2]*Q32 + B[1]),R2+1,5);
  TST (Inner41(R2+2,p),(A[0]*Q13 + A[1]*Q23 + A[2]*Q33 + B[2]),R2+2,6);

  // note: cross product axes need to be scaled when s is computed.
  // normal (n1,n2,n3) is relative to box 1.
#undef TST
#define TST(expr1,expr2,n1,n2,n3,cc) \
  s2 = fabs(expr1) - (expr2); \
  l = sqrt ((n1)*(n1) + (n2)*(n2) + (n3)*(n3)); \
  if (l > 0) { \
  s2 /= l; \
  if (s2*fudge_factor > s) { \
  s = s2; \
  normalR = 0; \
  normalC[0] = (n1)/l; normalC[1] = (n2)/l; normalC[2] = (n3)/l; \
  invert_normal = ((expr1) < 0); \
  code = (cc); \
} \
}

  // separating axis = u1 x (v1,v2,v3)
  TST(pp[2]*R21-pp[1]*R31,(A[1]*Q31+A[2]*Q21+B[1]*Q13+B[2]*Q12),0,-R31,R21,7);
  TST(pp[2]*R22-pp[1]*R32,(A[1]*Q32+A[2]*Q22+B[0]*Q13+B[2]*Q11),0,-R32,R22,8);
  TST(pp[2]*R23-pp[1]*R33,(A[1]*Q33+A[2]*Q23+B[0]*Q12+B[1]*Q11),0,-R33,R23,9);

  // separating axis = u2 x (v1,v2,v3)
  TST(pp[0]*R31-pp[2]*R11,(A[0]*Q31+A[2]*Q11+B[1]*Q23+B[2]*Q22),R31,0,-R11,10);
  TST(pp[0]*R32-pp[2]*R12,(A[0]*Q32+A[2]*Q12+B[0]*Q23+B[2]*Q21),R32,0,-R12,11);
  TST(pp[0]*R33-pp[2]*R13,(A[0]*Q33+A[2]*Q13+B[0]*Q22+B[1]*Q21),R33,0,-R13,12);

  // separating axis = u3 x (v1,v2,v3)
  TST(pp[1]*R11-pp[0]*R21,(A[0]*Q21+A[1]*Q11+B[1]*Q33+B[2]*Q32),-R21,R11,0,13);
  TST(pp[1]*R12-pp[0]*R22,(A[0]*Q22+A[1]*Q12+B[0]*Q33+B[2]*Q31),-R22,R12,0,14);
  TST(pp[1]*R13-pp[0]*R23,(A[0]*Q23+A[1]*Q13+B[0]*Q32+B[1]*Q31),-R23,R13,0,15);

#undef TST

  if (!code) return 0;
  if (s > 0.0) return 0;

  // if we get to this point, the boxes interpenetrate. compute the normal
  // in global coordinates.

  Eigen::Vector3d normal;
  Eigen::Vector3d point_vec;
  double penetration;

  if (normalR) {
    normal << normalR[0],normalR[4],normalR[8];
  }
  else {
    normal << Inner((R1),(normalC)), Inner((R1+4),(normalC)), Inner((R1+8),(normalC));
    //dMULTIPLY0_331 (normal,R1,normalC);
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
    for (i=0; i<3; i++) pa[i] = p1[i];
    for (j=0; j<3; j++)
    {
#define TEMP_INNER14(a,b) (a[0]*(b)[0] + a[1]*(b)[4] + a[2]*(b)[8])
      sign = (TEMP_INNER14(normal,R1+j) > 0) ? 1.0 : -1.0;

      //sign = (Inner14(normal,R1+j) > 0) ? 1.0 : -1.0;

      for (i=0; i<3; i++) pa[i] += sign * A[j] * R1[i*4+j];
    }

    // find a point pb on the intersecting edge of box 2
    dVector3 pb;
    for (i=0; i<3; i++) pb[i] = p2[i];
    for (j=0; j<3; j++) {
      sign = (TEMP_INNER14(normal,R2+j) > 0) ? -1.0 : 1.0;
#undef TEMP_INNER14
      for (i=0; i<3; i++) pb[i] += sign * B[j] * R2[i*4+j];
    }

    double alpha,beta;
    dVector3 ua,ub;
    for (i=0; i<3; i++) ua[i] = R1[((code)-7)/3 + i*4];
    for (i=0; i<3; i++) ub[i] = R2[((code)-7)%3 + i*4];

    dLineClosestApproach (pa,ua,pb,ub,&alpha,&beta);
    for (i=0; i<3; i++) pa[i] += ua[i]*alpha;
    for (i=0; i<3; i++) pb[i] += ub[i]*beta;


    {
      point_vec << 0.5*(pa[0]+pb[0]), 0.5*(pa[1]+pb[1]), 0.5*(pa[2]+pb[2]);
      penetration = -s;

      Contact contact;
      contact.point = point_vec;
      contact.normal = normal;
      contact.penetrationDepth = penetration;
      result.push_back(contact);
    }
    return 1;
  }

  // okay, we have a face-something intersection (because the separating
  // axis is perpendicular to a face). define face 'a' to be the reference
  // face (i.e. the normal vector is perpendicular to this) and face 'b' to be
  // the incident face (the closest face of the other box).

  const double *Ra,*Rb,*pa,*pb,*Sa,*Sb;
  if (code <= 3) {
    Ra = R1;
    Rb = R2;
    pa = p1;
    pb = p2;
    Sa = A;
    Sb = B;
  }
  else {
    Ra = R2;
    Rb = R1;
    pa = p2;
    pb = p1;
    Sa = B;
    Sb = A;
  }

  // nr = normal vector of reference face dotted with axes of incident box.
  // anr = absolute values of nr.
  dVector3 normal2,nr,anr;
  if (code <= 3) {
    normal2[0] = normal[0];
    normal2[1] = normal[1];
    normal2[2] = normal[2];
  }
  else {
    normal2[0] = -normal[0];
    normal2[1] = -normal[1];
    normal2[2] = -normal[2];
  }
  dMULTIPLY1_331 (nr,Rb,normal2);
  anr[0] = fabs (nr[0]);
  anr[1] = fabs (nr[1]);
  anr[2] = fabs (nr[2]);

  // find the largest compontent of anr: this corresponds to the normal
  // for the indident face. the other axis numbers of the indicent face
  // are stored in a1,a2.
  int lanr,a1,a2;
  if (anr[1] > anr[0]) {
    if (anr[1] > anr[2]) {
      a1 = 0;
      lanr = 1;
      a2 = 2;
    }
    else {
      a1 = 0;
      a2 = 1;
      lanr = 2;
    }
  }
  else {
    if (anr[0] > anr[2]) {
      lanr = 0;
      a1 = 1;
      a2 = 2;
    }
    else {
      a1 = 0;
      a2 = 1;
      lanr = 2;
    }
  }

  // compute center point of incident face, in reference-face coordinates
  dVector3 center;
  if (nr[lanr] < 0) {
    for (i=0; i<3; i++) center[i] = pb[i] - pa[i] + Sb[lanr] * Rb[i*4+lanr];
  }
  else {
    for (i=0; i<3; i++) center[i] = pb[i] - pa[i] - Sb[lanr] * Rb[i*4+lanr];
  }

  // find the normal and non-normal axis numbers of the reference box
  int codeN,code1,code2;
  if (code <= 3) codeN = code-1; else codeN = code-4;
  if (codeN==0) {
    code1 = 1;
    code2 = 2;
  }
  else if (codeN==1) {
    code1 = 0;
    code2 = 2;
  }
  else {
    code1 = 0;
    code2 = 1;
  }

  // find the four corners of the incident face, in reference-face coordinates
  double quad[8];	// 2D coordinate of incident face (x,y pairs)
  double c1,c2,m11,m12,m21,m22;
  c1 = Inner14 (center,Ra+code1);
  c2 = Inner14 (center,Ra+code2);
  // optimize this? - we have already computed this data above, but it is not
  // stored in an easy-to-index format. for now it's quicker just to recompute
  // the four dot products.
  m11 = Inner44 (Ra+code1,Rb+a1);
  m12 = Inner44 (Ra+code1,Rb+a2);
  m21 = Inner44 (Ra+code2,Rb+a1);
  m22 = Inner44 (Ra+code2,Rb+a2);
  {
    double k1 = m11*Sb[a1];
    double k2 = m21*Sb[a1];
    double k3 = m12*Sb[a2];
    double k4 = m22*Sb[a2];
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
  int n = intersectRectQuad (rect,quad,ret);
  if (n < 1) return 0;		// this should never happen

  // convert the intersection points into reference-face coordinates,
  // and compute the contact position and depth for each point. only keep
  // those points that have a positive (penetrating) depth. delete points in
  // the 'ret' array as necessary so that 'point' and 'ret' correspond.
  //real point[3*8];		// penetrating contact points
  double point[24];		// penetrating contact points
  double dep[8];			// depths for those points
  double det1 = dRecip(m11*m22 - m12*m21);
  m11 *= det1;
  m12 *= det1;
  m21 *= det1;
  m22 *= det1;
  int cnum = 0;			// number of penetrating contact points found
  for (j=0; j < n; j++) {
    double k1 =  m22*(ret[j*2]-c1) - m12*(ret[j*2+1]-c2);
    double k2 = -m21*(ret[j*2]-c1) + m11*(ret[j*2+1]-c2);
    for (i=0; i<3; i++)
    {
      point[cnum*3+i] = center[i] + k1*Rb[i*4+a1] + k2*Rb[i*4+a2];
    }
    dep[cnum] = Sa[codeN] - Inner(normal2,point+cnum*3);
    if (dep[cnum] >= 0) {
      ret[cnum*2] = ret[j*2];
      ret[cnum*2+1] = ret[j*2+1];
      cnum++;
    }
  }
  if (cnum < 1) return 0;	// this should never happen

  // we can't generate more contacts than we actually have
  int maxc = 4;
  if (maxc > cnum) maxc = cnum;
  //if (maxc < 1) maxc = 1;

  if (cnum <= maxc) {
    // we have less contacts than we need, so we use them all
    for (j=0; j < cnum; j++)
    {
      point_vec << point[j*3+0] + pa[0], point[j*3+1] + pa[1], point[j*3+2] + pa[2];

      Contact contact;
      contact.point = point_vec;
      contact.normal = normal;
      contact.penetrationDepth = dep[j];
      result.push_back(contact);
    }
  }
  else {
    // we have more contacts than are wanted, some of them must be culled.
    // find the deepest point, it is always the first contact.
    int i1 = 0;
    double maxdepth = dep[0];
    for (i=1; i<cnum; i++) {
      if (dep[i] > maxdepth) {
        maxdepth = dep[i];
        i1 = i;
      }
    }

    int iret[8];
    cullPoints (cnum,ret,maxc,i1,iret);

    cnum = maxc;
    for (j=0; j < cnum; j++)
    {
      point_vec << point[iret[j]*3+0] + pa[0], point[iret[j]*3+1] + pa[1], point[iret[j]*3+2] + pa[2];

      Contact contact;
      contact.point = point_vec;
      contact.normal = normal;
      contact.penetrationDepth = dep[iret[j]];
      result.push_back(contact);
    }
  }
  return cnum;
}

int collideBoxBox(const Eigen::Vector3d& size0, const Eigen::Isometry3d& T0,
                  const Eigen::Vector3d& size1, const Eigen::Isometry3d& T1,
                  std::vector<Contact>* result)
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

  return dBoxBox(p1, R1, halfSize1, p0, R0, halfSize0, *result);
}

int	collideBoxSphere(const Eigen::Vector3d& size0, const Eigen::Isometry3d& T0,
                     const double& r1, const Eigen::Isometry3d& T1,
                     std::vector<Contact>* result)
{
  Eigen::Vector3d halfSize = 0.5 * size0;
  bool inside_box = true;

  // clipping a center of the sphere to a boundary of the box
  //Vec3 c0(&T0[9]);
  Eigen::Vector3d c0 = T1.translation();
  Eigen::Vector3d p = T0.inverse() * c0;

  if (p[0] < -halfSize[0]) { p[0] = -halfSize[0]; inside_box = false; }
  if (p[0] >  halfSize[0]) { p[0] =  halfSize[0]; inside_box = false; }

  if (p[1] < -halfSize[1]) { p[1] = -halfSize[1]; inside_box = false; }
  if (p[1] >  halfSize[1]) { p[1] =  halfSize[1]; inside_box = false; }

  if (p[2] < -halfSize[2]) { p[2] = -halfSize[2]; inside_box = false; }
  if (p[2] >  halfSize[2]) { p[2] =  halfSize[2]; inside_box = false; }


  Eigen::Vector3d normal(0.0, 0.0, 0.0);
  double penetration;

  if ( inside_box )
  {
    // find nearest side from the sphere center
    double min = halfSize[0] - fabs(p[0]);
    double tmin = halfSize[1] - fabs(p[1]);
    int idx = 0;

    if ( tmin < min )
    {
      min = tmin;
      idx = 1;
    }
    tmin = halfSize[2] - fabs(p[2]);
    if ( tmin < min )
    {
      min = tmin;
      idx = 2;
    }

    //normal[idx] = (p[idx] > 0.0 ? 1.0 : -1.0);
    normal[idx] = (p[idx] > 0.0 ? -1.0 : 1.0);
    normal = T0.linear() * normal;
    penetration = min + r1;

    Contact contact;
    contact.point = c0;
    contact.normal = normal;
    contact.penetrationDepth = penetration;
    result->push_back(contact);
    return 1;
  }

  Eigen::Vector3d contactpt = T0 * p;
  //normal = c0 - contactpt;
  normal = contactpt - c0;
  double mag = normal.norm();
  penetration = r1 - mag;

  if (penetration < 0.0)
  {
    return 0;
  }

  if (mag > DART_COLLISION_EPS)
  {
    normal *= (1.0/mag);

    Contact contact;
    contact.point = contactpt;
    contact.normal = normal;
    contact.penetrationDepth = penetration;
    result->push_back(contact);
  }
  else
  {
    double min = halfSize[0] - fabs(p[0]);
    double tmin = halfSize[1] - fabs(p[1]);
    int idx = 0;

    if ( tmin < min )
    {
      min = tmin;
      idx = 1;
    }
    tmin = halfSize[2] - fabs(p[2]);
    if ( tmin < min )
    {
      min = tmin;
      idx = 2;
    }
    normal.setZero();
    //normal[idx] = (p[idx] > 0.0 ? 1.0 : -1.0);
    normal[idx] = (p[idx] > 0.0 ? -1.0 : 1.0);
    normal = T0.linear() * normal;

    Contact contact;
    contact.point = contactpt;
    contact.normal = normal;
    contact.penetrationDepth = penetration;
    result->push_back(contact);
  }
  return 1;
}

int collideSphereBox(const double& r0, const Eigen::Isometry3d& T0,
                     const Eigen::Vector3d& size1, const Eigen::Isometry3d& T1,
                     std::vector<Contact>* result)
{
  Eigen::Vector3d size = 0.5 * size1;
  bool inside_box = true;

  // clipping a center of the sphere to a boundary of the box
  Eigen::Vector3d c0 = T0.translation();
  Eigen::Vector3d p = T1.inverse() * c0;

  if (p[0] < -size[0]) { p[0] = -size[0]; inside_box = false; }
  if (p[0] >  size[0]) { p[0] =  size[0]; inside_box = false; }

  if (p[1] < -size[1]) { p[1] = -size[1]; inside_box = false; }
  if (p[1] >  size[1]) { p[1] =  size[1]; inside_box = false; }

  if (p[2] < -size[2]) { p[2] = -size[2]; inside_box = false; }
  if (p[2] >  size[2]) { p[2] =  size[2]; inside_box = false; }


  Eigen::Vector3d normal(0.0, 0.0, 0.0);
  double penetration;

  if ( inside_box )
  {
    // find nearest side from the sphere center
    double min = size[0] - fabs(p[0]);
    double tmin = size[1] - fabs(p[1]);
    int idx = 0;

    if ( tmin < min )
    {
      min = tmin;
      idx = 1;
    }
    tmin = size[2] - fabs(p[2]);
    if ( tmin < min )
    {
      min = tmin;
      idx = 2;
    }

    normal[idx] = (p[idx] > 0.0 ? 1.0 : -1.0);
    normal = T1.linear() * normal;
    penetration = min + r0;

    Contact contact;
    contact.point = c0;
    contact.normal = normal;
    contact.penetrationDepth = penetration;
    result->push_back(contact);
    return 1;
  }


  Eigen::Vector3d contactpt = T1 * p;
  normal = c0 - contactpt;
  double mag = normal.norm();
  penetration = r0 - mag;

  if (penetration < 0.0)
  {
    return 0;
  }

  if (mag > DART_COLLISION_EPS)
  {
    normal *= (1.0/mag);

    Contact contact;
    contact.point = contactpt;
    contact.normal = normal;
    contact.penetrationDepth = penetration;
    result->push_back(contact);}
  else
  {
    double min = size[0] - fabs(p[0]);
    double tmin = size[1] - fabs(p[1]);
    int idx = 0;

    if ( tmin < min )
    {
      min = tmin;
      idx = 1;
    }
    tmin = size[2] - fabs(p[2]);
    if ( tmin < min )
    {
      min = tmin;
      idx = 2;
    }
    normal.setZero();
    normal[idx] = (p[idx] > 0.0 ? 1.0 : -1.0);
    normal = T1.linear() * normal;

    Contact contact;
    contact.point = contactpt;
    contact.normal = normal;
    contact.penetrationDepth = penetration;
    result->push_back(contact);
  }
  return 1;
}

int collideSphereSphere(const double& _r0, const Eigen::Isometry3d& c0,
                        const double& _r1, const Eigen::Isometry3d& c1,
                        std::vector<Contact>* result)
{
  double r0 = _r0;
  double r1 = _r1;
  double rsum = r0 + r1;
  Eigen::Vector3d normal = c0.translation() - c1.translation();
  double normal_sqr = normal.squaredNorm();

  if ( normal_sqr > rsum * rsum )
  {
    return 0;
  }

  r0 /= rsum;
  r1 /= rsum;

  Eigen::Vector3d point = r1 * c0.translation() + r0 * c1.translation();
  double penetration;

  if (normal_sqr < DART_COLLISION_EPS)
  {
    normal.setZero();
    penetration = rsum;

    Contact contact;
    contact.point = point;
    contact.normal = normal;
    contact.penetrationDepth = penetration;
    result->push_back(contact);
    return 1;
  }

  normal_sqr = sqrt(normal_sqr);
  normal *= (1.0/normal_sqr);
  penetration = rsum - normal_sqr;

  Contact contact;
  contact.point = point;
  contact.normal = normal;
  contact.penetrationDepth = penetration;
  result->push_back(contact);
  return 1;

}

int collideCylinderSphere(const double& cyl_rad, const double& half_height, const Eigen::Isometry3d& T0,
                          const double& sphere_rad, const Eigen::Isometry3d& T1,
                          std::vector<Contact>* result)
{
  Eigen::Vector3d center = T0.inverse() * T1.translation();

  double dist = sqrt(center[0] * center[0] + center[1] * center[1]);

  if ( dist < cyl_rad && fabs(center[2]) < half_height + sphere_rad )
  {
    Contact contact;
    contact.penetrationDepth = 0.5 * (half_height + sphere_rad - math::sgn(center[2]) * center[2]);
    contact.point = T0 * Eigen::Vector3d(center[0], center[1], half_height - contact.penetrationDepth);
    contact.normal = T0.linear() * Eigen::Vector3d(0.0, 0.0, math::sgn(center[2]));
    result->push_back(contact);
    return 1;
  }
  else
  {
    double penetration = 0.5 * (cyl_rad + sphere_rad - dist);
    if ( penetration > 0.0 )
    {
      if ( fabs(center[2]) > half_height )
      {
        Eigen::Vector3d point = (Eigen::Vector3d(center[0], center[1], 0.0).normalized());
        point *= cyl_rad;
        point[2] = math::sgn(center[2]) * half_height;
        Eigen::Vector3d normal = point - center;
        penetration = sphere_rad - normal.norm();
        normal = (T0.linear() * normal).normalized();
        point = T0 * point;

        if (penetration > 0.0)
        {
          Contact contact;
          contact.point = point;
          contact.normal = normal;
          contact.penetrationDepth = penetration;
          result->push_back(contact);
          return 1;
        }
      }
      else // if( center[2] >= -half_height && center[2] <= half_height )
      {
        Eigen::Vector3d point = (Eigen::Vector3d(center[0], center[1], 0.0)).normalized();
        Eigen::Vector3d normal = -(T0.linear() * point);
        point *= (cyl_rad - penetration);
        point[2] = center[2];
        point = T0 * point;

        Contact contact;
        contact.point = point;
        contact.normal = normal;
        contact.penetrationDepth = penetration;
        result->push_back(contact);
        return 1;
      }
    }
  }
  return 0;
}

int collideCylinderPlane(const double& cyl_rad, const double& half_height, const Eigen::Isometry3d& T0,
                         const Eigen::Vector3d& plane_normal, const Eigen::Isometry3d& T1,
                         std::vector<Contact>* result)
{
  Eigen::Vector3d normal = T1.linear() * plane_normal;
  Eigen::Vector3d Rx = T0.linear().rightCols(1);
  Eigen::Vector3d Ry = normal - normal.dot(Rx) * Rx;
  double mag = Ry.norm();
  Ry.normalize();
  if (mag < DART_COLLISION_EPS)
  {
    if (fabs(Rx[2]) > 1.0 - DART_COLLISION_EPS)
      Ry = Eigen::Vector3d::UnitX();
    else
      Ry = (Eigen::Vector3d(Rx[1], -Rx[0], 0.0)).normalized();
  }

  Eigen::Vector3d Rz = Rx.cross(Ry);
  Eigen::Isometry3d T;
  T.linear().col(0) = Rx;
  T.linear().col(1) = Ry;
  T.linear().col(2) = Rz;
  T.translation() = T0.translation();

  Eigen::Vector3d nn = T.linear().transpose() * normal;
  Eigen::Vector3d pn = T.inverse() * T1.translation();

  // four corners c0 = ( -h/2, -r ), c1 = ( +h/2, -r ), c2 = ( +h/2, +r ), c3 = ( -h/2, +r )
  Eigen::Vector3d c[4] = {
    Eigen::Vector3d(-half_height, -cyl_rad, 0.0),
    Eigen::Vector3d(+half_height, -cyl_rad, 0.0),
    Eigen::Vector3d(+half_height, +cyl_rad, 0.0),
    Eigen::Vector3d(-half_height, +cyl_rad, 0.0) };

  double depth[4] = { (pn - c[0]).dot(nn), (pn - c[1]).dot(nn), (pn - c[2]).dot(nn), (pn - c[3]).dot(nn) };

  double penetration = -1.0;
  int found = -1;
  for (int i = 0; i < 4; i++)
  {
    if (depth[i] > penetration)
    {
      penetration = depth[i];
      found = i;
    }
  }

  Eigen::Vector3d point;

  if (fabs(depth[found] - depth[(found+1)%4]) < DART_COLLISION_EPS)
    point = T * (0.5 * (c[found] + c[(found+1)%4]));
  else if (fabs(depth[found] - depth[(found+3)%4]) < DART_COLLISION_EPS)
    point = T * (0.5 * (c[found] + c[(found+3)%4]));
  else
    point = T * c[found];

  if (penetration > 0.0)
  {
    Contact contact;
    contact.point = point;
    contact.normal = normal;
    contact.penetrationDepth = penetration;
    result->push_back(contact);
    return 1;
  }

  return 0;
}

int collide(const dynamics::Shape* _shape0, const Eigen::Isometry3d& _T0,
            const dynamics::Shape* _shape1, const Eigen::Isometry3d& _T1,
            std::vector<Contact>* _result)
{
  dynamics::Shape::ShapeType LeftType = _shape0->getShapeType();
  dynamics::Shape::ShapeType RightType = _shape1->getShapeType();

  switch(LeftType)
  {
    case dynamics::Shape::BOX:
    {
      const dynamics::BoxShape* box0 = static_cast<const dynamics::BoxShape*>(_shape0);

      switch(RightType)
      {
        case dynamics::Shape::BOX:
        {
          const dynamics::BoxShape* box1 = static_cast<const dynamics::BoxShape*>(_shape1);
          return collideBoxBox(box0->getSize(), _T0,
                               box1->getSize(), _T1, _result);
        }
        case dynamics::Shape::ELLIPSOID:
        {
          const dynamics::EllipsoidShape* ellipsoid1 = static_cast<const dynamics::EllipsoidShape*>(_shape1);
          return collideBoxSphere(box0->getSize(), _T0,
                                  ellipsoid1->getSize()[0] * 0.5, _T1,
              _result);
        }
        case dynamics::Shape::CYLINDER:
        {
          //----------------------------------------------------------
          // NOT SUPPORT CYLINDER
          //----------------------------------------------------------
          const dynamics::CylinderShape* cylinder1 = static_cast<const dynamics::CylinderShape*>(_shape1);

          Eigen::Vector3d dimTemp(cylinder1->getRadius() * sqrt(2.0),
                                  cylinder1->getRadius() * sqrt(2.0),
                                  cylinder1->getHeight());
          return collideBoxBox(box0->getSize(), _T0, dimTemp, _T1, _result);
        }
        default:
          return false;

          break;
      }

      break;
    }
    case dynamics::Shape::ELLIPSOID:
    {
      const dynamics::EllipsoidShape* ellipsoid0 = static_cast<const dynamics::EllipsoidShape*>(_shape0);

      switch(RightType)
      {
        case dynamics::Shape::BOX:
        {
          const dynamics::BoxShape* box1 = static_cast<const dynamics::BoxShape*>(_shape1);
          return collideSphereBox(ellipsoid0->getSize()[0] * 0.5, _T0,
                                  box1->getSize(), _T1,
                                  _result);
        }
        case dynamics::Shape::ELLIPSOID:
        {
          const dynamics::EllipsoidShape* ellipsoid1 = static_cast<const dynamics::EllipsoidShape*>(_shape0);
          return collideSphereSphere(ellipsoid0->getSize()[0] * 0.5, _T0,
                                     ellipsoid1->getSize()[0] * 0.5, _T1,
                                     _result);
        }
        case dynamics::Shape::CYLINDER:
        {
          //----------------------------------------------------------
          // NOT SUPPORT CYLINDER
          //----------------------------------------------------------
          const dynamics::CylinderShape* cylinder1 = static_cast<const dynamics::CylinderShape*>(_shape1);

          Eigen::Vector3d dimTemp1(cylinder1->getRadius() * sqrt(2.0),
                                   cylinder1->getRadius() * sqrt(2.0),
                                   cylinder1->getHeight());
          return collideSphereBox(
                ellipsoid0->getSize()[0] * 0.5, _T0, dimTemp1, _T1, _result);
        }
        default:
          return false;

          break;
      }

      break;
    }
    case dynamics::Shape::CYLINDER:
    {
      //----------------------------------------------------------
      // NOT SUPPORT CYLINDER
      //----------------------------------------------------------
      const dynamics::CylinderShape* cylinder0 = static_cast<const dynamics::CylinderShape*>(_shape0);

      Eigen::Vector3d dimTemp0(cylinder0->getRadius() * sqrt(2.0),
                               cylinder0->getRadius() * sqrt(2.0),
                               cylinder0->getHeight());
      switch(RightType)
      {
        case dynamics::Shape::BOX:
        {
          const dynamics::BoxShape* box1 = static_cast<const dynamics::BoxShape*>(_shape1);
          return collideBoxBox(dimTemp0, _T0, box1->getSize(), _T1, _result);
        }
        case dynamics::Shape::ELLIPSOID:
        {
          const dynamics::EllipsoidShape* ellipsoid1 = static_cast<const dynamics::EllipsoidShape*>(_shape1);
          return collideBoxSphere(dimTemp0, _T0, ellipsoid1->getSize()[0] * 0.5, _T1, _result);
        }
        case dynamics::Shape::CYLINDER:
        {
          //----------------------------------------------------------
          // NOT SUPPORT CYLINDER
          //----------------------------------------------------------
          const dynamics::CylinderShape* cylinder1 = static_cast<const dynamics::CylinderShape*>(_shape1);

          Eigen::Vector3d dimTemp1(cylinder1->getRadius() * sqrt(2.0),
                                   cylinder1->getRadius() * sqrt(2.0),
                                   cylinder1->getHeight());
          return collideBoxBox(dimTemp0, _T0, dimTemp1, _T1, _result);
        }
        default:
        {
          return false;
        }
      }

      break;
    }
    default:
      return false;

      break;
  }
}



} // namespace collision
} // namespace dart
