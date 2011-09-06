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


#include "distance_primitive.h"
#include "intersect.h"
#include <limits>
#include <queue>

namespace collision_checking
{


/** \brief Bounding volume test structure */
struct BVT
{
  /** \brief distance between bvs */
  BVH_REAL d;

  /** \brief bv indices for a pair of bvs in two models */
  int b1, b2;
};

/** \brief Comparer between two BVT */
struct BVT_Comparer
{
  bool operator() (const BVT& lhs, const BVT& rhs) const
  {
    return lhs.d > rhs.d;
  }
};

typedef std::priority_queue<BVT, std::vector<BVT>, BVT_Comparer> BVTQ;

BVH_DistanceResult::BVH_DistanceResult()
{
  num_bv_tests = 0;
  num_tri_tests = 0;

  query_time_seconds = 0;

  /* default queue size is 2 */
  qsize = 2;

  /* default last_tris are the first triangle on each model */
  last_tri_id1 = 0;
  last_tri_id2 = 0;

  /** default relative and absolute error */
  rel_err = (BVH_REAL)0.01;
  abs_err = (BVH_REAL)0.01;

  distance = std::numeric_limits<BVH_REAL>::max();
}


BVH_DistanceResult::~BVH_DistanceResult()
{

}


/** \brief Compute the distance between two RSS modes */
void distanceQueueRecurse(BVNode<RSS>* tree1, BVNode<RSS>* tree2,
                          const Vec3f R[3], const Vec3f& T,
                          int b1, int b2,
                          Vec3f* vertices1, Vec3f* vertices2,
                          Triangle* tri_indices1, Triangle* tri_indices2,
                          BVH_DistanceResult* res, BVHFrontList* front_list)
{
  BVTQ bvtq;

  BVT min_test;
  min_test.b1 = b1;
  min_test.b2 = b2;

  while(1)
  {
    BVNode<RSS>* node1 = tree1 + min_test.b1;
    BVNode<RSS>* node2 = tree2 + min_test.b2;

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
    }
    else if((int)bvtq.size() == res->qsize - 1)
    {
      // queue should not get two more tests, recur

      distanceQueueRecurse(tree1, tree2, R, T, min_test.b1, min_test.b2, vertices1, vertices2, tri_indices1, tri_indices2, res, front_list);
    }
    else
    {
      // queue capacity still small than qsize


      BVH_REAL sz1 = node1->bv.size();
      BVH_REAL sz2 = node2->bv.size();

      res->num_bv_tests += 2;

      BVT bvt1, bvt2;

      if(l2 || (!l1 && (sz1 > sz2)))
      {
        int c1 = node1->first_child;
        int c2 = c1 + 1;

        bvt1.b1 = c1;
        bvt1.b2 = min_test.b2;
        bvt1.d = distance(R, T, (tree1 + bvt1.b1)->bv, (tree2 + bvt1.b2)->bv);

        bvt2.b1 = c2;
        bvt2.b2 = min_test.b2;
        bvt2.d = distance(R, T, (tree1 + bvt2.b1)->bv, (tree2 + bvt2.b2)->bv);
      }
      else
      {
        int c1 = node2->first_child;
        int c2 = c1 + 1;

        bvt1.b1 = min_test.b1;
        bvt1.b2 = c1;
        bvt1.d = distance(R, T, (tree1 + bvt1.b1)->bv, (tree2 + bvt1.b2)->bv);

        bvt2.b1 = min_test.b1;
        bvt2.b2 = c2;
        bvt2.d = distance(R, T, (tree1 + bvt2.b1)->bv, (tree2 + bvt2.b2)->bv);
      }

      bvtq.push(bvt1);
      bvtq.push(bvt2);
    }

    if(bvtq.empty())
    {
      break;
    }
    else
    {
      min_test = bvtq.top();
      bvtq.pop();

      if((min_test.d + res->abs_err >= res->distance) &&
          ((min_test.d * (1 + res->rel_err)) >= res->distance))
      {
        if(front_list) front_list->push_back(BVHFrontNode(min_test.b1, min_test.b2));

        break;
      }
    }
  }
}


void distanceRecurse(BVNode<RSS>* tree1, BVNode<RSS>* tree2,
                    const Vec3f R[3], const Vec3f& T,
                    int b1, int b2,
                    Vec3f* vertices1, Vec3f* vertices2,
                    Triangle* tri_indices1, Triangle* tri_indices2,
                    BVH_DistanceResult* res, BVHFrontList* front_list)
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

    return;
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

  BVH_REAL d1 = distance(R, T, (tree1 + a1)->bv, (tree2 + a2)->bv);
  BVH_REAL d2 = distance(R, T, (tree1 + c1)->bv, (tree2 + c2)->bv);

  if(d2 < d1)
  {
    if((d2 < (res->distance - res->abs_err)) || (d2 * (1 + res->rel_err) < res->distance))
    {
      distanceRecurse(tree1, tree2, R, T, c1, c2, vertices1, vertices2, tri_indices1, tri_indices2, res, front_list);
    }
    else
    {
      if(front_list) front_list->push_back(BVHFrontNode(c1, c2));
    }

    if((d1 < (res->distance - res->abs_err)) || (d1 * (1 + res->rel_err) < res->distance))
    {
      distanceRecurse(tree1, tree2, R, T, a1, a2, vertices1, vertices2, tri_indices1, tri_indices2, res, front_list);
    }
    else
    {
      if(front_list) front_list->push_back(BVHFrontNode(a1, a2));
    }


  }
  else
  {
    if((d1 < (res->distance - res->abs_err)) || (d1 * (1 + res->rel_err) < res->distance))
    {
      distanceRecurse(tree1, tree2, R, T, a1, a2, vertices1, vertices2, tri_indices1, tri_indices2, res, front_list);
    }
    else
    {
      if(front_list) front_list->push_back(BVHFrontNode(a1, a2));
    }

    if((d2 < (res->distance - res->abs_err)) || (d2 * (1 + res->rel_err) < res->distance))
    {
      distanceRecurse(tree1, tree2, R, T, c1, c2, vertices1, vertices2, tri_indices1, tri_indices2, res, front_list);
    }
    else
    {
      if(front_list) front_list->push_back(BVHFrontNode(c1, c2));
    }
  }
}


}
