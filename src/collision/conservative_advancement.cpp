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

#include "conservative_advancement.h"
#include "intersect.h"
#include "collision.h"

/** \brief Main namespace */
namespace collision_checking
{

void InterpMotion::SimpleQuaternion::fromRotation(const Vec3f R[3])
{
  const int next[3] = {1, 2, 0};

  BVH_REAL trace = R[0][0] + R[1][1] + R[2][2];
  BVH_REAL root;

  if(trace > 0.0)
  {
    // |w| > 1/2, may as well choose w > 1/2
    root = sqrt(trace + 1.0);  // 2w
    data[0] = 0.5 * root;
    root = 0.5 / root;  // 1/(4w)
    data[1] = (R[2][1] - R[1][2])*root;
    data[2] = (R[0][2] - R[2][0])*root;
    data[3] = (R[1][0] - R[0][1])*root;
  }
  else
  {
    // |w| <= 1/2
    int i = 0;
    if(R[1][1] > R[0][0])
    {
        i = 1;
    }
    if(R[2][2] > R[i][i])
    {
        i = 2;
    }
    int j = next[i];
    int k = next[j];

    root = sqrt(R[i][i] - R[j][j] - R[k][k] + 1.0);
    BVH_REAL* quat[3] = { &data[1], &data[2], &data[3] };
    *quat[i] = 0.5 * root;
    root = 0.5 / root;
    data[0] = (R[k][j] - R[j][k]) * root;
    *quat[j] = (R[j][i] + R[i][j]) * root;
    *quat[k] = (R[k][i] + R[i][k]) * root;
  }
}

void InterpMotion::SimpleQuaternion::toRotation(Vec3f R[3]) const
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

  R[0] = Vec3f(1.0 - (twoYY + twoZZ), twoXY - twoWZ, twoXZ + twoWY);
  R[1] = Vec3f(twoXY + twoWZ, 1.0 - (twoXX + twoZZ), twoYZ - twoWX);
  R[2] = Vec3f(twoXZ - twoWY, twoYZ + twoWX, 1.0 - (twoXX + twoYY));
}

void InterpMotion::SimpleQuaternion::fromAxisAngle(const Vec3f& axis, BVH_REAL angle)
{
  BVH_REAL half_angle = 0.5 * angle;
  BVH_REAL sn = sin((double)half_angle);
  data[0] = cos((double)half_angle);
  data[1] = sn * axis[0];
  data[2] = sn * axis[1];
  data[3] = sn * axis[2];
}

void InterpMotion::SimpleQuaternion::toAxisAngle(Vec3f& axis, BVH_REAL& angle) const
{
  double sqr_length = data[1] * data[1] + data[2] * data[2] + data[3] * data[3];
  if(sqr_length > 0)
  {
    angle = 2.0 * acos((double)data[0]);
    double inv_length = 1.0 / sqrt(sqr_length);
    axis[0] = inv_length * data[1];
    axis[1] = inv_length * data[2];
    axis[2] = inv_length * data[3];
  }
  else
  {
    angle = 0;
    axis[0] = 1;
    axis[1] = 0;
    axis[2] = 0;
  }
}

BVH_REAL InterpMotion::SimpleQuaternion::dot(const SimpleQuaternion& other) const
{
  return data[0] * other.data[0] + data[1] * other.data[1] + data[2] * other.data[2] + data[3] * other.data[3];
}

InterpMotion::SimpleQuaternion InterpMotion::SimpleQuaternion::operator + (const SimpleQuaternion& other) const
{
  return SimpleQuaternion(data[0] + other.data[0], data[1] + other.data[1],
                          data[2] + other.data[2], data[3] + other.data[3]);
}

InterpMotion::SimpleQuaternion InterpMotion::SimpleQuaternion::operator - (const SimpleQuaternion& other) const
{
  return SimpleQuaternion(data[0] - other.data[0], data[1] - other.data[1],
                          data[2] - other.data[2], data[3] - other.data[3]);
}

InterpMotion::SimpleQuaternion InterpMotion::SimpleQuaternion::operator * (const SimpleQuaternion& other) const
{
  return SimpleQuaternion(data[0] * other.data[0] - data[1] * other.data[1] - data[2] * other.data[2] - data[3] * other.data[3],
                          data[0] * other.data[1] + data[1] * other.data[0] + data[2] * other.data[3] - data[3] * other.data[2],
                          data[0] * other.data[2] - data[1] * other.data[3] + data[2] * other.data[0] + data[3] * other.data[1],
                          data[0] * other.data[3] + data[1] * other.data[2] - data[2] * other.data[1] + data[3] * other.data[0]);
}

InterpMotion::SimpleQuaternion InterpMotion::SimpleQuaternion::operator - () const
{
  return SimpleQuaternion(-data[0], -data[1], -data[2], -data[3]);
}

InterpMotion::SimpleQuaternion InterpMotion::SimpleQuaternion::operator * (BVH_REAL t) const
{
  return SimpleQuaternion(data[0] * t, data[1] * t, data[2] * t, data[3] * t);
}

InterpMotion::SimpleQuaternion InterpMotion::SimpleQuaternion::conj() const
{
  return SimpleQuaternion(data[0], -data[1], -data[2], -data[3]);
}

InterpMotion::SimpleQuaternion InterpMotion::SimpleQuaternion::inverse() const
{
  double sqr_length = data[0] * data[0] + data[1] * data[1] + data[2] * data[2] + data[3] * data[3];
  if(sqr_length > 0)
  {
    double inv_length = 1.0 / sqrt(sqr_length);
    return SimpleQuaternion(data[0] * inv_length, -data[1] * inv_length, -data[2] * inv_length, -data[3] * inv_length);
  }
  else
  {
    return SimpleQuaternion(data[0], -data[1], -data[2], -data[3]);
  }
}

Vec3f InterpMotion::SimpleQuaternion::transform(const Vec3f& v) const
{
  SimpleQuaternion r = (*this) * SimpleQuaternion(0, v[0], v[1], v[2]) * (this->conj());
  return Vec3f(r.data[1], r.data[2], r.data[3]);
}

BVH_REAL InterpMotion::computeMotionBound(const RSS& bv, const Vec3f& n) const
{
  BVH_REAL c_proj_max = (bv.Tr.cross(angular_axis)).sqrLength();
  BVH_REAL tmp;
  tmp = ((bv.Tr + bv.axis[0] * bv.l[0]).cross(angular_axis)).sqrLength();
  if(tmp > c_proj_max) c_proj_max = tmp;
  tmp = ((bv.Tr + bv.axis[1] * bv.l[1]).cross(angular_axis)).sqrLength();
  if(tmp > c_proj_max) c_proj_max = tmp;
  tmp = ((bv.Tr + bv.axis[0] * bv.l[0] + bv.axis[1] * bv.l[1]).cross(angular_axis)).sqrLength();
  if(tmp > c_proj_max) c_proj_max = tmp;

  c_proj_max = sqrt(c_proj_max);

  BVH_REAL v_dot_n = linear_vel.dot(n);
  BVH_REAL w_cross_n = (angular_axis.cross(n)).length() * angular_vel;
  BVH_REAL mu = v_dot_n + w_cross_n * (bv.r + c_proj_max);

  return mu;
}

BVH_REAL InterpMotion::computeMotionBound(const Vec3f& a, const Vec3f& b, const Vec3f& c, const Vec3f& n) const
{
  BVH_REAL c_proj_max = (a.cross(angular_axis)).sqrLength();
  BVH_REAL tmp;
  tmp = (b.cross(angular_axis)).sqrLength();
  if(tmp > c_proj_max) c_proj_max = tmp;
  tmp = (c.cross(angular_axis)).sqrLength();
  if(tmp > c_proj_max) c_proj_max = tmp;

  c_proj_max = sqrt(c_proj_max);

  BVH_REAL v_dot_n = linear_vel.dot(n);
  BVH_REAL w_cross_n = (angular_axis.cross(n)).length() * angular_vel;
  BVH_REAL mu = v_dot_n + w_cross_n * c_proj_max;

  return mu;
}


void conservativeAdvancementRecurse(BVNode<RSS>* tree1, BVNode<RSS>* tree2,
                                    const Vec3f R[3], const Vec3f& T,
                                    int b1, int b2,
                                    Vec3f* vertices1, Vec3f* vertices2,
                                    Triangle* tri_indices1, Triangle* tri_indices2,
                                    BVH_CAResult* res, BVHFrontList* front_list)
{
  BVNode<RSS>* node1 = tree1 + b1;
  BVNode<RSS>* node2 = tree2 + b2;

  bool l1 = node1->isLeaf();
  bool l2 = node2->isLeaf();

  if(l1 && l2)
  {
    if(front_list) front_list->push_back(BVHFrontNode(b1, b2));

    res->num_tri_tests++;

    const Triangle& tri_id1 = tri_indices1[-node1->first_child - 1];
    const Triangle& tri_id2 = tri_indices2[-node2->first_child - 1];

    const Vec3f& p1 = vertices1[tri_id1[0]];
    const Vec3f& p2 = vertices1[tri_id1[1]];
    const Vec3f& p3 = vertices1[tri_id1[2]];

    const Vec3f& q1 = vertices2[tri_id2[0]];
    const Vec3f& q2 = vertices2[tri_id2[1]];
    const Vec3f& q3 = vertices2[tri_id2[2]];

    // nearest point pair
    Vec3f P1, P2;

    BVH_REAL d = TriangleDistance::triDistance(p1, p2, p3, q1, q2, q3,
                                               R, T,
                                               P1, P2);

    if(d < res->distance)
    {
      res->distance = d;

      res->p1 = P1;
      res->p2 = P2;

      res->last_tri_id1 = -node1->first_child - 1;
      res->last_tri_id2 = -node2->first_child - 1;
    }


    /** n is the local frame of object 1 */
    Vec3f n = P2 - P1;
    /** turn n into the global frame */
    Vec3f n_transformed = MxV(res->motion1.t.R, n);
    n_transformed.normalize();
    BVH_REAL bound1 = res->motion1.computeMotionBound(p1, p2, p3, n_transformed);
    BVH_REAL bound2 = res->motion2.computeMotionBound(q1, q2, q3, n_transformed);

    BVH_REAL delta_t = d / (bound1 + bound2);

    if(delta_t < res->delta_t)
      res->delta_t = delta_t;
  }

  BVH_REAL sz1 = node1->bv.size();
  BVH_REAL sz2 = node2->bv.size();

  int a1, a2, c1, c2;

  if(l2 || (!l1 && (sz1 > sz2)))
  {
    a1 = node1->first_child;
    a2 = b2;
    c1 = node1->first_child + 1;
    c2 = b2;
  }
  else
  {
    a1 = b1;
    a2 = node2->first_child;
    c1 = b1;
    c2 = node2->first_child + 1;
  }

  res->num_bv_tests += 2;

  // all in RSS1's local coordinate
  Vec3f P11, P12;
  Vec3f P21, P22;

  BVH_REAL d1 = distance(R, T, (tree1 + a1)->bv, (tree2 + a2)->bv, &P11, &P21);
  BVH_REAL d2 = distance(R, T, (tree1 + c1)->bv, (tree2 + c2)->bv, &P12, &P22);


  if(d2 < d1)
  {
    if((d2 < res->w * (res->distance - res->abs_err)) || (d2 * (1 + res->rel_err) < res->w * res->distance))
    {
      conservativeAdvancementRecurse(tree1, tree2, R, T, c1, c2, vertices1, vertices2, tri_indices1, tri_indices2, res, front_list);
    }
    else
    {
      /** n is in local frame of RSS c1 */
      Vec3f n = P22 - P12;
      /** turn n into the global frame */
      Vec3f n_transformed = (tree1 + c1)->bv.axis[0] * n[0] + (tree1 + c1)->bv.axis[1] * n[1] + (tree1 + c1)->bv.axis[2] * n[2];
      n_transformed = MxV(R, n_transformed);
      n_transformed.normalize();

      BVH_REAL bound1 = res->motion1.computeMotionBound((tree1 + c1)->bv, n_transformed);
      BVH_REAL bound2 = res->motion2.computeMotionBound((tree2 + c2)->bv, n_transformed);

      BVH_REAL delta_t = d2 / (bound1 + bound2);

      if(delta_t < res->delta_t) res->delta_t = delta_t;
      if(front_list) front_list->push_back(BVHFrontNode(c1, c2));
    }

    if((d1 < res->w * (res->distance - res->abs_err)) || (d1 * (1 + res->rel_err) < res->w * res->distance))
    {
      conservativeAdvancementRecurse(tree1, tree2, R, T, a1, a2, vertices1, vertices2, tri_indices1, tri_indices2, res, front_list);
    }
    else
    {
      /** n is in the local frame of RSS a1 */
      Vec3f n = P21 - P11;
      /** turn n into the global frame */
      Vec3f n_transformed = (tree1 + a1)->bv.axis[0] * n[0] + (tree1 + a1)->bv.axis[1] * n[1] + (tree1 + a1)->bv.axis[2] * n[2];
      n_transformed = MxV(R, n_transformed);
      n_transformed.normalize();

      BVH_REAL bound1 = res->motion1.computeMotionBound((tree1 + a1)->bv, n_transformed);
      BVH_REAL bound2 = res->motion2.computeMotionBound((tree2 + a2)->bv, n_transformed);

      BVH_REAL delta_t = d1 / (bound1 + bound2);

      if(delta_t < res->delta_t) res->delta_t = delta_t;      if(delta_t < res->delta_t) res->delta_t = delta_t;
      if(front_list) front_list->push_back(BVHFrontNode(a1, a2));
    }
  }
  else
  {
    if((d1 < res->w * (res->distance - res->abs_err)) || (d1 * (1 + res->rel_err) < res->w * res->distance))
    {
      conservativeAdvancementRecurse(tree1, tree2, R, T, a1, a2, vertices1, vertices2, tri_indices1, tri_indices2, res, front_list);
    }
    else
    {
      /** n is in the local frame of RSS a1 */
      Vec3f n = P21 - P11;
      /** turn n into the global frame */
      Vec3f n_transformed = (tree1 + a1)->bv.axis[0] * n[0] + (tree1 + a1)->bv.axis[1] * n[1] + (tree1 + a1)->bv.axis[2] * n[2];
      n_transformed = MxV(R, n_transformed);
      n_transformed.normalize();

      BVH_REAL bound1 = res->motion1.computeMotionBound((tree1 + a1)->bv, n_transformed);
      BVH_REAL bound2 = res->motion2.computeMotionBound((tree2 + a2)->bv, n_transformed);

      BVH_REAL delta_t = d1 / (bound1 + bound2);

      if(delta_t < res->delta_t) res->delta_t = delta_t;
      if(front_list) front_list->push_back(BVHFrontNode(a1, a2));
    }

    if((d2 < res->w * (res->distance - res->abs_err)) || (d2 * (1 + res->rel_err) < res->w * res->distance))
    {
      conservativeAdvancementRecurse(tree1, tree2, R, T, c1, c2, vertices1, vertices2, tri_indices1, tri_indices2, res, front_list);
    }
    else
    {
      /** n is the local frame of RSS c1 */
      Vec3f n = P22 - P12;
      /** turn n into the global frame */
      Vec3f n_transformed = (tree1 + c1)->bv.axis[0] * n[0] + (tree1 + c1)->bv.axis[1] * n[1] + (tree1 + c1)->bv.axis[2] * n[2];
      n_transformed = MxV(R, n_transformed);
      n_transformed.normalize();

      BVH_REAL bound1 = res->motion1.computeMotionBound((tree1 + c1)->bv, n_transformed);
      BVH_REAL bound2 = res->motion2.computeMotionBound((tree2 + c2)->bv, n_transformed);

      BVH_REAL delta_t = d2 / (bound1 + bound2);

      if(delta_t < res->delta_t) res->delta_t = delta_t;
      if(front_list) front_list->push_back(BVHFrontNode(c1, c2));
    }
  }
}


void continuousCollide_CA(const BVHModel<RSS>& model1, const Vec3f R1_1[3], const Vec3f& T1_1, const Vec3f R1_2[3], const Vec3f& T1_2,
                          const BVHModel<RSS>& model2, const Vec3f R2_1[3], const Vec3f& T2_1, const Vec3f R2_2[3], const Vec3f& T2_2,
                          BVH_CAResult* res, BVHFrontList* front_list)
{
  res->motion1 = InterpMotion(R1_1, T1_1, R1_2, T1_2);
  res->motion2 = InterpMotion(R2_1, T2_1, R2_2, T2_2);

  BVH_CollideResult c_res;
  collide(model1, R1_1, T1_1, model2, R2_1, T2_1, &c_res);

  if(c_res.numPairs() > 0)
  {
    res->toc = 0;
    return;
  }

  do
  {
    Vec3f R1_t[3];
    Vec3f R2_t[3];
    Vec3f T1_t;
    Vec3f T2_t;

    res->motion1.getCurrentTransformation(R1_t, T1_t);
    res->motion2.getCurrentTransformation(R2_t, T2_t);

    // compute the transformation from 1 to 2
    Vec3f R1_col[3];
    R1_col[0] = Vec3f(R1_t[0][0], R1_t[1][0], R1_t[2][0]);
    R1_col[1] = Vec3f(R1_t[0][1], R1_t[1][1], R1_t[2][1]);
    R1_col[2] = Vec3f(R1_t[0][2], R1_t[1][2], R1_t[2][2]);

    Vec3f R2_col[3];
    R2_col[0] = Vec3f(R2_t[0][0], R2_t[1][0], R2_t[2][0]);
    R2_col[1] = Vec3f(R2_t[0][1], R2_t[1][1], R2_t[2][1]);
    R2_col[2] = Vec3f(R2_t[0][2], R2_t[1][2], R2_t[2][2]);

    Vec3f R[3];
    R[0] = Vec3f(R1_col[0].dot(R2_col[0]), R1_col[0].dot(R2_col[1]), R1_col[0].dot(R2_col[2]));
    R[1] = Vec3f(R1_col[1].dot(R2_col[0]), R1_col[1].dot(R2_col[1]), R1_col[1].dot(R2_col[2]));
    R[2] = Vec3f(R1_col[2].dot(R2_col[0]), R1_col[2].dot(R2_col[1]), R1_col[2].dot(R2_col[2]));

    Vec3f Ttemp = T2_t - T1_t;
    Vec3f T(R1_col[0].dot(Ttemp), R1_col[1].dot(Ttemp), R1_col[2].dot(Ttemp));

    res->delta_t = 1;
    res->distance = std::numeric_limits<BVH_REAL>::max();
    conservativeAdvancementRecurse(model1.bvs, model2.bvs, R, T, 0, 0, model1.vertices, model2.vertices, model1.tri_indices, model2.tri_indices, res, front_list);

    if(res->delta_t < res->t_err)
    {
      return;
    }

    res->toc += res->delta_t;

    if(res->toc > 1)
    {
      res->toc = 1;
      return;
    }

    res->motion1.integrate(res->toc);
    res->motion2.integrate(res->toc);
  }
  while(1);
}

}
