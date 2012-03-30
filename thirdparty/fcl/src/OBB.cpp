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

#include "fcl/OBB.h"
#include "fcl/BVH_utility.h"
#include "fcl/transform.h"

#include <iostream>
#include <limits>

namespace fcl
{

bool OBB::overlap(const OBB& other) const
{
  // compute what transform [R,T] that takes us from cs1 to cs2.
  // [R,T] = [R1,T1]'[R2,T2] = [R1',-R1'T][R2,T2] = [R1'R2, R1'(T2-T1)]
  // First compute the rotation part, then translation part
  Vec3f t = other.To - To; // T2 - T1
  Vec3f T(t.dot(axis[0]), t.dot(axis[1]), t.dot(axis[2])); // R1'(T2-T1)
  Matrix3f R(axis[0].dot(other.axis[0]), axis[0].dot(other.axis[1]), axis[0].dot(other.axis[2]),
             axis[1].dot(other.axis[0]), axis[1].dot(other.axis[1]), axis[1].dot(other.axis[2]),
             axis[2].dot(other.axis[0]), axis[2].dot(other.axis[1]), axis[2].dot(other.axis[2]));

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
  bvp.extent.setValue(0);

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


bool OBB::obbDisjoint(const Matrix3f& B, const Vec3f& T, const Vec3f& a, const Vec3f& b)
{
  register BVH_REAL t, s;
  const BVH_REAL reps = 1e-6;

  Matrix3f Bf = B.abs();
  Bf += reps;

  // if any of these tests are one-sided, then the polyhedra are disjoint

  // A1 x A2 = A0
  t = ((T[0] < 0.0) ? -T[0] : T[0]);

  if(t > (a[0] + b.dot(Bf[0])))
    return true;

  // B1 x B2 = B0
  s =  B.transposeDotX(T);
  t = ((s < 0.0) ? -s : s);

  if(t > (b[0] + Bf.transposeDotX(a)))
    return true;

  // A2 x A0 = A1
  t = ((T[1] < 0.0) ? -T[1] : T[1]);

  if(t > (a[1] + b.dot(Bf[1])))
    return true;

  // A0 x A1 = A2
  t =((T[2] < 0.0) ? -T[2] : T[2]);

  if(t > (a[2] + b.dot(Bf[2])))
    return true;

  // B2 x B0 = B1
  s = B.transposeDotY(T);
  t = ((s < 0.0) ? -s : s);

  if(t > (b[1] + Bf.transposeDotY(a)))
    return true;

  // B0 x B1 = B2
  s = B.transposeDotZ(T);
  t = ((s < 0.0) ? -s : s);

  if(t > (b[2] + Bf.transposeDotZ(a)))
    return true;

  // A0 x B0
  s = T[2] * B[1][0] - T[1] * B[2][0];
  t = ((s < 0.0) ? -s : s);

  if(t > (a[1] * Bf[2][0] + a[2] * Bf[1][0] +
          b[1] * Bf[0][2] + b[2] * Bf[0][1]))
    return true;

  // A0 x B1
  s = T[2] * B[1][1] - T[1] * B[2][1];
  t = ((s < 0.0) ? -s : s);

  if(t > (a[1] * Bf[2][1] + a[2] * Bf[1][1] +
          b[0] * Bf[0][2] + b[2] * Bf[0][0]))
    return true;

  // A0 x B2
  s = T[2] * B[1][2] - T[1] * B[2][2];
  t = ((s < 0.0) ? -s : s);

  if(t > (a[1] * Bf[2][2] + a[2] * Bf[1][2] +
          b[0] * Bf[0][1] + b[1] * Bf[0][0]))
    return true;

  // A1 x B0
  s = T[0] * B[2][0] - T[2] * B[0][0];
  t = ((s < 0.0) ? -s : s);

  if(t > (a[0] * Bf[2][0] + a[2] * Bf[0][0] +
          b[1] * Bf[1][2] + b[2] * Bf[1][1]))
    return true;

  // A1 x B1
  s = T[0] * B[2][1] - T[2] * B[0][1];
  t = ((s < 0.0) ? -s : s);

  if(t > (a[0] * Bf[2][1] + a[2] * Bf[0][1] +
          b[0] * Bf[1][2] + b[2] * Bf[1][0]))
    return true;

  // A1 x B2
  s = T[0] * B[2][2] - T[2] * B[0][2];
  t = ((s < 0.0) ? -s : s);

  if(t > (a[0] * Bf[2][2] + a[2] * Bf[0][2] +
          b[0] * Bf[1][1] + b[1] * Bf[1][0]))
    return true;

  // A2 x B0
  s = T[1] * B[0][0] - T[0] * B[1][0];
  t = ((s < 0.0) ? -s : s);

  if(t > (a[0] * Bf[1][0] + a[1] * Bf[0][0] +
          b[1] * Bf[2][2] + b[2] * Bf[2][1]))
    return true;

  // A2 x B1
  s = T[1] * B[0][1] - T[0] * B[1][1];
  t = ((s < 0.0) ? -s : s);

  if(t > (a[0] * Bf[1][1] + a[1] * Bf[0][1] +
          b[0] * Bf[2][2] + b[2] * Bf[2][0]))
    return true;

  // A2 x B2
  s = T[1] * B[0][2] - T[0] * B[1][2];
  t = ((s < 0.0) ? -s : s);

  if(t > (a[0] * Bf[1][2] + a[1] * Bf[0][2] +
          b[0] * Bf[2][1] + b[1] * Bf[2][0]))
    return true;

  return false;

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


OBB OBB::merge_largedist(const OBB& b1, const OBB& b2)
{
  OBB b;
  Vec3f vertex[16];
  b1.computeVertices(vertex);
  b2.computeVertices(vertex + 8);
  Matrix3f M;
  Vec3f E[3];
  BVH_REAL s[3] = {0, 0, 0};

  // obb axes
  Vec3f& R0 = b.axis[0];
  Vec3f& R1 = b.axis[1];
  Vec3f& R2 = b.axis[2];

  R0 = b1.To - b2.To;
  R0.normalize();

  Vec3f vertex_proj[16];
  for(int i = 0; i < 16; ++i)
    vertex_proj[i] = vertex[i] - R0 * vertex[i].dot(R0);

  getCovariance(vertex_proj, NULL, NULL, 16, M);
  matEigen(M, s, E);

  int min, mid, max;
  if (s[0] > s[1]) { max = 0; min = 1; }
  else { min = 0; max = 1; }
  if (s[2] < s[min]) { mid = min; min = 2; }
  else if (s[2] > s[max]) { mid = max; max = 2; }
  else { mid = 2; }


  R1.setValue(E[0][max], E[1][max], E[2][max]);
  R2.setValue(E[0][mid], E[1][mid], E[2][mid]);

  // set obb centers and extensions
  Vec3f center, extent;
  getExtentAndCenter(vertex, NULL, NULL, 16, b.axis, center, extent);

  b.To = center;
  b.extent = extent;

  return b;
}

OBB OBB::merge_smalldist(const OBB& b1, const OBB& b2)
{
  OBB b;
  b.To = (b1.To + b2.To) * 0.5;
  SimpleQuaternion q0, q1;
  q0.fromAxes(b1.axis);
  q1.fromAxes(b2.axis);
  if(q0.dot(q1) < 0)
    q1 = -q1;

  SimpleQuaternion q = q0 + q1;
  BVH_REAL inv_length = 1.0 / sqrt(q.dot(q));
  q = q * inv_length;
  q.toAxes(b.axis);


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


BVH_REAL OBB::distance(const OBB& other, Vec3f* P, Vec3f* Q) const
{
  std::cerr << "OBB distance not implemented!" << std::endl;
  return 0.0;
}

// R is row first
bool overlap(const Matrix3f& R0, const Vec3f& T0, const OBB& b1, const OBB& b2)
{
  Matrix3f R0b2(R0[0].dot(b2.axis[0]), R0[0].dot(b2.axis[1]), R0[0].dot(b2.axis[2]),
                R0[1].dot(b2.axis[0]), R0[1].dot(b2.axis[1]), R0[1].dot(b2.axis[2]),
                R0[2].dot(b2.axis[0]), R0[2].dot(b2.axis[1]), R0[2].dot(b2.axis[2]));

  Matrix3f R(R0b2.transposeDotX(b1.axis[0]), R0b2.transposeDotY(b1.axis[0]), R0b2.transposeDotZ(b1.axis[0]),
             R0b2.transposeDotX(b1.axis[1]), R0b2.transposeDotY(b1.axis[1]), R0b2.transposeDotZ(b1.axis[1]),
             R0b2.transposeDotX(b1.axis[2]), R0b2.transposeDotY(b1.axis[2]), R0b2.transposeDotZ(b1.axis[2]));

  Vec3f Ttemp = R0 * b2.To + T0 - b1.To;
  Vec3f T(Ttemp.dot(b1.axis[0]), Ttemp.dot(b1.axis[1]), Ttemp.dot(b1.axis[2]));

  return (OBB::obbDisjoint(R, T, b1.extent, b2.extent) == 0);
}

}
