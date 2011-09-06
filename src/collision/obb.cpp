/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Jia Pan */

#include "obb.h"

namespace collision_checking
{
OBB::SimpleQuaternion::SimpleQuaternion() {}

OBB::SimpleQuaternion::SimpleQuaternion(BVH_REAL a, BVH_REAL b, BVH_REAL c, BVH_REAL d)
{
  data[0] = a;
  data[1] = b;
  data[2] = c;
  data[3] = d;
}

void OBB::SimpleQuaternion::fromRotation(const Vec3f axis[3])
{
  // Algorithm in Ken Shoemake's article in 1987 SIGGRAPH course notes
  // article "Quaternion Calculus and Fast Animation".

  const int next[3] = {1, 2, 0};

  BVH_REAL trace = axis[0][0] + axis[1][1] + axis[2][2];
  BVH_REAL root;

  if(trace > 0.0)
  {
    // |w| > 1/2, may as well choose w > 1/2
    root = sqrt(trace + 1.0);  // 2w
    data[0] = 0.5 * root;
    root = 0.5 / root;  // 1/(4w)
    data[1] = (axis[1][2] - axis[2][1])*root;
    data[2] = (axis[2][0] - axis[0][2])*root;
    data[3] = (axis[0][1] - axis[1][0])*root;
  }
  else
  {
    // |w| <= 1/2
    int i = 0;
    if(axis[1][1] > axis[0][0])
    {
        i = 1;
    }
    if(axis[2][2] > axis[i][i])
    {
        i = 2;
    }
    int j = next[i];
    int k = next[j];

    root = sqrt(axis[i][i] - axis[j][j] - axis[k][k] + 1.0);
    BVH_REAL* quat[3] = { &data[1], &data[2], &data[3] };
    *quat[i] = 0.5 * root;
    root = 0.5 / root;
    data[0] = (axis[j][k] - axis[k][j]) * root;
    *quat[j] = (axis[i][j] + axis[j][i]) * root;
    *quat[k] = (axis[i][k] + axis[k][i]) * root;
  }
}

void OBB::SimpleQuaternion::toRotation(Vec3f axis[3]) const
{
  BVH_REAL twoX  = 2.0*data[1];
  BVH_REAL twoY  = 2.0*data[2];
  BVH_REAL twoZ  = 2.0*data[3];
  BVH_REAL twoWX = twoX*data[0];
  BVH_REAL twoWY = twoY*data[0];
  BVH_REAL twoWZ = twoZ*data[0];
  BVH_REAL twoXX = twoX*data[1];
  BVH_REAL twoXY = twoY*data[1];
  BVH_REAL twoXZ = twoZ*data[1];
  BVH_REAL twoYY = twoY*data[2];
  BVH_REAL twoYZ = twoZ*data[2];
  BVH_REAL twoZZ = twoZ*data[3];

  axis[0] = Vec3f(1.0 - (twoYY + twoZZ), twoXY + twoWZ, twoXZ - twoWY);
  axis[1] = Vec3f(twoXY - twoWZ, 1.0 - (twoXX + twoZZ), twoYZ + twoWX);
  axis[2] = Vec3f(twoXZ + twoWY, twoYZ - twoWX, 1.0 - (twoXX + twoYY));
}

BVH_REAL OBB::SimpleQuaternion::dot(const SimpleQuaternion& other) const
{
  return data[0] * other.data[0] + data[1] * other.data[1] + data[2] * other.data[2] + data[3] * other.data[3];
}

OBB::SimpleQuaternion OBB::SimpleQuaternion::operator + (const SimpleQuaternion& other) const
{
  return SimpleQuaternion(data[0] + other.data[0], data[1] + other.data[1],
                          data[2] + other.data[2], data[3] + other.data[3]);
}

OBB::SimpleQuaternion OBB::SimpleQuaternion::operator - () const
{
  return SimpleQuaternion(-data[0], -data[1], -data[2], -data[3]);
}

OBB::SimpleQuaternion OBB::SimpleQuaternion::operator * (BVH_REAL t) const
{
  return SimpleQuaternion(data[0] * t, data[1] * t, data[2] * t, data[3] * t);
}

bool OBB::overlap(const OBB& other) const
{
  // compute what transform [R,T] that takes us from cs1 to cs2.
  // [R,T] = [R1,T1]'[R2,T2] = [R1',-R1'T][R2,T2] = [R1'R2, R1'(T2-T1)]
  // First compute the rotation part, then translation part
  Vec3f t = other.To - To; // T2 - T1
  Vec3f T(t.dot(axis[0]), t.dot(axis[1]), t.dot(axis[2])); // R1'(T2-T1)
  Vec3f R[3];
  R[0] = Vec3f(axis[0].dot(other.axis[0]), axis[0].dot(other.axis[1]), axis[0].dot(other.axis[2]));
  R[1] = Vec3f(axis[1].dot(other.axis[0]), axis[1].dot(other.axis[1]), axis[1].dot(other.axis[2]));
  R[2] = Vec3f(axis[2].dot(other.axis[0]), axis[2].dot(other.axis[1]), axis[2].dot(other.axis[2]));

  // R is row first
  return (obbDisjoint(R, T, extent, other.extent) == 0);
}


bool OBB::contain(const Vec3f& p) const
{
  Vec3f local_p = p - To;
  BVH_REAL proj = local_p.dot(axis[0]);
  if((proj > extent[0]) || (proj < -extent[0]))
    return false;

  proj = local_p.dot(axis[1]);
  if((proj > extent[1]) || (proj < -extent[1]))
    return false;

  proj = local_p.dot(axis[2]);
  if((proj > extent[2]) || (proj < -extent[2]))
    return false;

  return true;
}

/** \brief A simple way to add new point to the OBB, not compact. */
OBB& OBB::operator += (const Vec3f& p)
{
  OBB bvp;
  bvp.To = p;
  bvp.axis[0] = axis[0];
  bvp.axis[1] = axis[1];
  bvp.axis[2] = axis[2];
  bvp.extent = Vec3f(0, 0, 0);

  *this += bvp;
  return *this;
}

OBB OBB::operator + (const OBB& other) const
{
  Vec3f center_diff = To - other.To;
  BVH_REAL max_extent = std::max(std::max(extent[0], extent[1]), extent[2]);
  BVH_REAL max_extent2 = std::max(std::max(other.extent[0], other.extent[1]), other.extent[2]);
  if(center_diff.length() > 2 * (max_extent + max_extent2))
  {
    return merge_largedist(*this, other);
  }
  else
  {
    return merge_smalldist(*this, other);
  }
}


bool OBB::obbDisjoint(const Vec3f B[3], const Vec3f& T, const Vec3f& a, const Vec3f& b)
{
  register BVH_REAL t, s;
  Vec3f Bf[3];
  const BVH_REAL reps = 1e-6;

  Bf[0] = abs(B[0]);
  Bf[1] = abs(B[1]);
  Bf[2] = abs(B[2]);

  Vec3f reps_vec(reps, reps, reps);

  Bf[0] += reps_vec;
  Bf[1] += reps_vec;
  Bf[2] += reps_vec;

  Vec3f Bf_col[3] = {Vec3f(Bf[0][0], Bf[1][0], Bf[2][0]),
                     Vec3f(Bf[0][1], Bf[1][1], Bf[2][1]),
                     Vec3f(Bf[0][2], Bf[1][2], Bf[2][2])};

  Vec3f B_col[3] = {Vec3f(B[0][0], B[1][0], B[2][0]),
                    Vec3f(B[0][1], B[1][1], B[2][1]),
                    Vec3f(B[0][2], B[1][2], B[2][2])};


  // if any of these tests are one-sided, then the polyhedra are disjoint

  // A1 x A2 = A0
  t = ((T[0] < 0) ? -T[0] : T[0]);

  if(t > (a[0] + b.dot(Bf[0])))
    return true;

  // B1 x B2 = B0
  s =  T.dot(B_col[0]);
  t = ((s < 0) ? -s : s);

  if(t > (b[0] + a.dot(Bf_col[0])))
    return true;

  // A2 x A0 = A1
  t = ((T[1] < 0) ? -T[1] : T[1]);

  if(t > (a[1] + b.dot(Bf[1])))
    return true;

  // A0 x A1 = A2
  t =((T[2] < 0) ? -T[2] : T[2]);

  if(t > (a[2] + b.dot(Bf[2])))
    return true;

  // B2 x B0 = B1
  s = T.dot(B_col[1]);
  t = ((s < 0) ? -s : s);

  if(t > (b[1] + a.dot(Bf_col[1])))
    return true;

  // B0 x B1 = B2
  s = T.dot(B_col[2]);
  t = ((s < 0) ? -s : s);

  if(t > (b[2] + a.dot(Bf_col[2])))
    return true;

  // A0 x B0
  s = T[2] * B[1][0] - T[1] * B[2][0];
  t = ((s < 0) ? -s : s);

  if(t > (a[1] * Bf[2][0] + a[2] * Bf[1][0] +
          b[1] * Bf[0][2] + b[2] * Bf[0][1]))
    return true;

  // A0 x B1
  s = T[2] * B[1][1] - T[1] * B[2][1];
  t = ((s < 0) ? -s : s);

  if(t > (a[1] * Bf[2][1] + a[2] * Bf[1][1] +
          b[0] * Bf[0][2] + b[2] * Bf[0][0]))
    return true;

  // A0 x B2
  s = T[2] * B[1][2] - T[1] * B[2][2];
  t = ((s < 0) ? -s : s);

  if(t > (a[1] * Bf[2][2] + a[2] * Bf[1][2] +
          b[0] * Bf[0][1] + b[1] * Bf[0][0]))
    return true;

  // A1 x B0
  s = T[0] * B[2][0] - T[2] * B[0][0];
  t = ((s < 0) ? -s : s);

  if(t > (a[0] * Bf[2][0] + a[2] * Bf[0][0] +
          b[1] * Bf[1][2] + b[2] * Bf[1][1]))
    return true;

  // A1 x B1
  s = T[0] * B[2][1] - T[2] * B[0][1];
  t = ((s < 0) ? -s : s);

  if(t > (a[0] * Bf[2][1] + a[2] * Bf[0][1] +
          b[0] * Bf[1][2] + b[2] * Bf[1][0]))
    return true;

  // A1 x B2
  s = T[0] * B[2][2] - T[2] * B[0][2];
  t = ((s < 0) ? -s : s);

  if(t > (a[0] * Bf[2][2] + a[2] * Bf[0][2] +
          b[0] * Bf[1][1] + b[1] * Bf[1][0]))
    return true;

  // A2 x B0
  s = T[1] * B[0][0] - T[0] * B[1][0];
  t = ((s < 0) ? -s : s);

  if(t > (a[0] * Bf[1][0] + a[1] * Bf[0][0] +
          b[1] * Bf[2][2] + b[2] * Bf[2][1]))
    return true;

  // A2 x B1
  s = T[1] * B[0][1] - T[0] * B[1][1];
  t = ((s < 0) ? -s : s);

  if(t > (a[0] * Bf[1][1] + a[1] * Bf[0][1] +
          b[0] * Bf[2][2] + b[2] * Bf[2][0]))
    return true;

  // A2 x B2
  s = T[1] * B[0][2] - T[0] * B[1][2];
  t = ((s < 0) ? -s : s);

  if(t > (a[0] * Bf[1][2] + a[1] * Bf[0][2] +
          b[0] * Bf[2][1] + b[1] * Bf[2][0]))
    return true;

  return false;

  /*
  register int r;

  // if any of these tests are one-sided, then the polyhedra are disjoint
  r = 1;

  // A1 x A2 = A0
  t = ((T[0] < 0) ? -T[0] : T[0]);

  r &= (t <=
          (a[0] + b[0] * Bf[0][0] + b[1] * Bf[0][1] + b[2] * Bf[0][2]));
  if (!r) return 1;

  // B1 x B2 = B0
  s = T[0]*B[0][0] + T[1]*B[1][0] + T[2]*B[2][0];
  t = ((s < 0) ? -s : s);

  r &= ( t <=
          (b[0] + a[0] * Bf[0][0] + a[1] * Bf[1][0] + a[2] * Bf[2][0]));
  if (!r) return 2;

  // A2 x A0 = A1
  t = ((T[1] < 0) ? -T[1] : T[1]);

  r &= ( t <=
          (a[1] + b[0] * Bf[1][0] + b[1] * Bf[1][1] + b[2] * Bf[1][2]));
  if (!r) return 3;

  // A0 x A1 = A2
  t =((T[2] < 0) ? -T[2] : T[2]);

  r &= ( t <=
          (a[2] + b[0] * Bf[2][0] + b[1] * Bf[2][1] + b[2] * Bf[2][2]));
  if (!r) return 4;

  // B2 x B0 = B1
  s = T[0]*B[0][1] + T[1]*B[1][1] + T[2]*B[2][1];
  t = ((s < 0) ? -s : s);

  r &= ( t <=
          (b[1] + a[0] * Bf[0][1] + a[1] * Bf[1][1] + a[2] * Bf[2][1]));
  if (!r) return 5;

  // B0 x B1 = B2
  s = T[0]*B[0][2] + T[1]*B[1][2] + T[2]*B[2][2];
  t = ((s < 0) ? -s : s);

  r &= ( t <=
          (b[2] + a[0] * Bf[0][2] + a[1] * Bf[1][2] + a[2] * Bf[2][2]));
  if (!r) return 6;

  // A0 x B0
  s = T[2] * B[1][0] - T[1] * B[2][0];
  t = ((s < 0) ? -s : s);

  r &= ( t <=
        (a[1] * Bf[2][0] + a[2] * Bf[1][0] +
         b[1] * Bf[0][2] + b[2] * Bf[0][1]));
  if (!r) return 7;

  // A0 x B1
  s = T[2] * B[1][1] - T[1] * B[2][1];
  t = ((s < 0) ? -s : s);

  r &= ( t <=
        (a[1] * Bf[2][1] + a[2] * Bf[1][1] +
         b[0] * Bf[0][2] + b[2] * Bf[0][0]));
  if (!r) return 8;

  // A0 x B2
  s = T[2] * B[1][2] - T[1] * B[2][2];
  t = ((s < 0) ? -s : s);

  r &= ( t <=
          (a[1] * Bf[2][2] + a[2] * Bf[1][2] +
           b[0] * Bf[0][1] + b[1] * Bf[0][0]));
  if (!r) return 9;

  // A1 x B0
  s = T[0] * B[2][0] - T[2] * B[0][0];
  t = ((s < 0) ? -s : s);

  r &= ( t <=
          (a[0] * Bf[2][0] + a[2] * Bf[0][0] +
           b[1] * Bf[1][2] + b[2] * Bf[1][1]));
  if (!r) return 10;

  // A1 x B1
  s = T[0] * B[2][1] - T[2] * B[0][1];
  t = ((s < 0) ? -s : s);

  r &= ( t <=
          (a[0] * Bf[2][1] + a[2] * Bf[0][1] +
           b[0] * Bf[1][2] + b[2] * Bf[1][0]));
  if (!r) return 11;

  // A1 x B2
  s = T[0] * B[2][2] - T[2] * B[0][2];
  t = ((s < 0) ? -s : s);

  r &= (t <=
          (a[0] * Bf[2][2] + a[2] * Bf[0][2] +
           b[0] * Bf[1][1] + b[1] * Bf[1][0]));
  if (!r) return 12;

  // A2 x B0
  s = T[1] * B[0][0] - T[0] * B[1][0];
  t = ((s < 0) ? -s : s);

  r &= (t <=
          (a[0] * Bf[1][0] + a[1] * Bf[0][0] +
           b[1] * Bf[2][2] + b[2] * Bf[2][1]));
  if (!r) return 13;

  // A2 x B1
  s = T[1] * B[0][1] - T[0] * B[1][1];
  t = ((s < 0) ? -s : s);

  r &= ( t <=
          (a[0] * Bf[1][1] + a[1] * Bf[0][1] +
           b[0] * Bf[2][2] + b[2] * Bf[2][0]));
  if (!r) return 14;

  // A2 x B2
  s = T[1] * B[0][2] - T[0] * B[1][2];
  t = ((s < 0) ? -s : s);

  r &= ( t <=
          (a[0] * Bf[1][2] + a[1] * Bf[0][2] +
           b[0] * Bf[2][1] + b[1] * Bf[2][0]));
  if (!r) return 15;

  return 0;  // should equal 0
  */
}


void OBB::computeVertices(Vec3f vertex[8]) const
{
  Vec3f extAxis0 = axis[0] * extent[0];
  Vec3f extAxis1 = axis[1] * extent[1];
  Vec3f extAxis2 = axis[2] * extent[2];

  vertex[0] = To - extAxis0 - extAxis1 - extAxis2;
  vertex[1] = To + extAxis0 - extAxis1 - extAxis2;
  vertex[2] = To + extAxis0 + extAxis1 - extAxis2;
  vertex[3] = To - extAxis0 + extAxis1 - extAxis2;
  vertex[4] = To - extAxis0 - extAxis1 + extAxis2;
  vertex[5] = To + extAxis0 - extAxis1 + extAxis2;
  vertex[6] = To + extAxis0 + extAxis1 + extAxis2;
  vertex[7] = To - extAxis0 + extAxis1 + extAxis2;
}

void OBB::getCovariance(Vec3f* ps, int n, Vec3f M[3])
{
  Vec3f S1;
  Vec3f S2[3];

  for(int i = 0; i < n; ++i)
  {
    Vec3f p = ps[i];
    S1 += p;
    S2[0][0] += (p[0] * p[0]);
    S2[1][1] += (p[1] * p[1]);
    S2[2][2] += (p[2] * p[2]);
    S2[0][1] += (p[0] * p[1]);
    S2[0][2] += (p[0] * p[2]);
    S2[1][2] += (p[1] * p[2]);
  }

  M[0][0] = S2[0][0] - S1[0]*S1[0] / n;
  M[1][1] = S2[1][1] - S1[1]*S1[1] / n;
  M[2][2] = S2[2][2] - S1[2]*S1[2] / n;
  M[0][1] = S2[0][1] - S1[0]*S1[1] / n;
  M[1][2] = S2[1][2] - S1[1]*S1[2] / n;
  M[0][2] = S2[0][2] - S1[0]*S1[2] / n;
  M[1][0] = M[0][1];
  M[2][0] = M[0][2];
  M[2][1] = M[1][2];
}

void OBB::Meigen(Vec3f a[3], BVH_REAL dout[3], Vec3f vout[3])
{
  int n = 3;
  int j, iq, ip, i;
  BVH_REAL tresh, theta, tau, t, sm, s, h, g, c;
  int nrot;
  BVH_REAL b[3];
  BVH_REAL z[3];
  BVH_REAL v[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  BVH_REAL d[3];

  for(ip = 0; ip < n; ++ip)
  {
    b[ip] = a[ip][ip];
    d[ip] = a[ip][ip];
    z[ip] = 0.0;
  }

  nrot = 0;

  for(i = 0; i < 50; ++i)
  {
    sm = 0.0;
    for(ip = 0; ip < n; ++ip)
      for(iq = ip + 1; iq < n; ++iq)
        sm += fabs(a[ip][iq]);
    if(sm == 0.0)
    {
      vout[0] = Vec3f(v[0][0], v[0][1], v[0][2]);
      vout[1] = Vec3f(v[1][0], v[1][1], v[1][2]);
      vout[2] = Vec3f(v[2][0], v[2][1], v[2][2]);
      dout[0] = d[0]; dout[1] = d[1]; dout[2] = d[2];
      return;
    }

    if(i < 3) tresh = 0.2 * sm / (n * n);
    else tresh = 0.0;

    for(ip = 0; ip < n; ++ip)
    {
      for(iq= ip + 1; iq < n; ++iq)
      {
        g = 100.0 * fabs(a[ip][iq]);
        if(i > 3 &&
            fabs(d[ip]) + g == fabs(d[ip]) &&
            fabs(d[iq]) + g == fabs(d[iq]))
          a[ip][iq] = 0.0;
        else if(fabs(a[ip][iq]) > tresh)
        {
          h = d[iq] - d[ip];
          if(fabs(h) + g == fabs(h)) t = (a[ip][iq]) / h;
          else
          {
            theta = 0.5 * h / (a[ip][iq]);
            t = 1.0 /(fabs(theta) + sqrt(1.0 + theta * theta));
            if(theta < 0.0) t = -t;
          }
          c = 1.0 / sqrt(1 + t * t);
          s = t * c;
          tau = s / (1.0 + c);
          h = t * a[ip][iq];
          z[ip] -= h;
          z[iq] += h;
          d[ip] -= h;
          d[iq] += h;
          a[ip][iq] = 0.0;
          for(j = 0; j < ip; ++j) { g = a[j][ip]; h = a[j][iq]; a[j][ip] = g - s * (h + g * tau); a[j][iq] = h + s * (g - h * tau); }
          for(j = ip + 1; j < iq; ++j) { g = a[ip][j]; h = a[j][iq]; a[ip][j] = g - s * (h + g * tau); a[j][iq] = h + s * (g - h * tau); }
          for(j = iq + 1; j < n; ++j) { g = a[ip][j]; h = a[iq][j]; a[ip][j] = g - s * (h + g * tau); a[iq][j] = h + s * (g - h * tau); }
          for(j = 0; j < n; ++j) { g = v[j][ip]; h = v[j][iq]; v[j][ip] = g - s * (h + g * tau); v[j][iq] = h + s * (g - h * tau); }
          nrot++;
        }
      }
    }
    for(ip = 0; ip < n; ++ip)
    {
      b[ip] += z[ip];
      d[ip] = b[ip];
      z[ip] = 0.0;
    }
  }

  std::cerr << "eigen: too many iterations in Jacobi transform." << std::endl;

  return;
}

void OBB::getExtentAndCenter(Vec3f* ps, int n, Vec3f axis[3], Vec3f& center, Vec3f& extent)
{
  BVH_REAL real_max = std::numeric_limits<BVH_REAL>::max();
  BVH_REAL min_coord[3] = {real_max, real_max, real_max};
  BVH_REAL max_coord[3] = {-real_max, -real_max, -real_max};

  for(int i = 0; i < n; ++i)
  {
    Vec3f v = ps[i];
    BVH_REAL proj[3];
    proj[0] = axis[0].dot(v);
    proj[1] = axis[1].dot(v);
    proj[2] = axis[2].dot(v);

    for(int j = 0; j < 3; ++j)
    {
      if(proj[j] > max_coord[j]) max_coord[j] = proj[j];
      if(proj[j] < min_coord[j]) min_coord[j] = proj[j];
    }
  }

  Vec3f o = Vec3f((max_coord[0] + min_coord[0]) / 2,
                 (max_coord[1] + min_coord[1]) / 2,
                 (max_coord[2] + min_coord[2]) / 2);

  center = axis[0] * o[0] + axis[1] * o[1] + axis[2] * o[2];

  extent = Vec3f((max_coord[0] - min_coord[0]) / 2,
                 (max_coord[1] - min_coord[1]) / 2,
                 (max_coord[2] - min_coord[2]) / 2);

}

OBB OBB::merge_largedist(const OBB& b1, const OBB& b2)
{
  OBB b;
  Vec3f vertex[16];
  b1.computeVertices(vertex);
  b2.computeVertices(vertex + 8);
  Vec3f M[3];
  Vec3f E[3];
  BVH_REAL s[3] = {0, 0, 0};
  Vec3f R[3];

  R[0] = b1.To - b2.To;
  R[0].normalize();

  Vec3f vertex_proj[16];
  for(int i = 0; i < 16; ++i)
    vertex_proj[i] = vertex[i] - R[0] * vertex[i].dot(R[0]);

  getCovariance(vertex_proj, 16, M);
  Meigen(M, s, E);

  int min, mid, max;
  if (s[0] > s[1]) { max = 0; min = 1; }
  else { min = 0; max = 1; }
  if (s[2] < s[min]) { mid = min; min = 2; }
  else if (s[2] > s[max]) { mid = max; max = 2; }
  else { mid = 2; }


  R[1] = Vec3f(E[0][max], E[1][max], E[2][max]);
  R[2] = Vec3f(E[0][mid], E[1][mid], E[2][mid]);

  // set obb axes
  b.axis[0] = R[0];
  b.axis[1] = R[1];
  b.axis[2] = R[2];

  // set obb centers and extensions
  Vec3f center, extent;
  getExtentAndCenter(vertex, 16, R, center, extent);

  b.To = center;
  b.extent = extent;

  return b;
}

OBB OBB::merge_smalldist(const OBB& b1, const OBB& b2)
{
  OBB b;
  b.To = (b1.To + b2.To) * 0.5;
  SimpleQuaternion q0, q1;
  q0.fromRotation(b1.axis);
  q1.fromRotation(b2.axis);
  if(q0.dot(q1) < 0)
    q1 = -q1;

  SimpleQuaternion q = q0 + q1;
  BVH_REAL inv_length = 1.0 / sqrt(q.dot(q));
  q = q * inv_length;
  q.toRotation(b.axis);


  Vec3f vertex[8], diff;
  BVH_REAL real_max = std::numeric_limits<BVH_REAL>::max();
  Vec3f pmin(real_max, real_max, real_max);
  Vec3f pmax(-real_max, -real_max, -real_max);

  b1.computeVertices(vertex);
  for(int i = 0; i < 8; ++i)
  {
    diff = vertex[i] - b.To;
    for(int j = 0; j < 3; ++j)
    {
      BVH_REAL dot = diff.dot(b.axis[j]);
      if(dot > pmax[j])
        pmax[j] = dot;
      else if(dot < pmin[j])
        pmin[j] = dot;
    }
  }

  b2.computeVertices(vertex);
  for(int i = 0; i < 8; ++i)
  {
    diff = vertex[i] - b.To;
    for(int j = 0; j < 3; ++j)
    {
      BVH_REAL dot = diff.dot(b.axis[j]);
      if(dot > pmax[j])
        pmax[j] = dot;
      else if(dot < pmin[j])
        pmin[j] = dot;
    }
  }

  for(int j = 0; j < 3; ++j)
  {
    b.To += (b.axis[j] * (0.5 * (pmax[j] + pmin[j])));
    b.extent[j] = 0.5 * (pmax[j] - pmin[j]);
  }

  return b;
}

// R is row first
bool overlap(const Vec3f R0[3], const Vec3f& T0, const OBB& b1, const OBB& b2)
{
  // R0 R2
  Vec3f Rtemp_col[3];
  Rtemp_col[0] = Vec3f(R0[0].dot(b2.axis[0]), R0[1].dot(b2.axis[0]), R0[2].dot(b2.axis[0]));
  Rtemp_col[1] = Vec3f(R0[0].dot(b2.axis[1]), R0[1].dot(b2.axis[1]), R0[2].dot(b2.axis[1]));
  Rtemp_col[2] = Vec3f(R0[0].dot(b2.axis[2]), R0[1].dot(b2.axis[2]), R0[2].dot(b2.axis[2]));

  // R1'Rtemp
  Vec3f R[3];
  R[0] = Vec3f(b1.axis[0].dot(Rtemp_col[0]), b1.axis[0].dot(Rtemp_col[1]), b1.axis[0].dot(Rtemp_col[2]));
  R[1] = Vec3f(b1.axis[1].dot(Rtemp_col[0]), b1.axis[1].dot(Rtemp_col[1]), b1.axis[1].dot(Rtemp_col[2]));
  R[2] = Vec3f(b1.axis[2].dot(Rtemp_col[0]), b1.axis[2].dot(Rtemp_col[1]), b1.axis[2].dot(Rtemp_col[2]));

  Vec3f Ttemp = Vec3f(R0[0].dot(b2.To), R0[1].dot(b2.To), R0[2].dot(b2.To)) + T0 - b1.To;

  Vec3f T = Vec3f(Ttemp.dot(b1.axis[0]), Ttemp.dot(b1.axis[1]), Ttemp.dot(b1.axis[2]));

  return (OBB::obbDisjoint(R, T, b1.extent, b2.extent) == 0);
}

}
