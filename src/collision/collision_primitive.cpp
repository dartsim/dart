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

#include "collision_primitive.h"

namespace collision_checking
{

BVH_CollideResult::BVH_CollideResult()
{
  num_bv_tests = 0;
  num_tri_tests = 0;
  num_vf_tests = 0;
  num_ee_tests = 0;

  query_time_seconds = 0;

  num_pairs = num_pairs_allocated = 0;
  pairs = NULL;

  num_max_contacts = 0;
}


BVH_CollideResult::~BVH_CollideResult()
{
  delete [] pairs;
}


void BVH_CollideResult::sizeTo(int n)
{
  BVHCollisionPair* temp;

  if(n < num_pairs) return;
  temp = new BVHCollisionPair[n];
  memcpy(temp, pairs, sizeof(BVHCollisionPair) * num_pairs);
  delete [] pairs;
  pairs = temp;
  num_pairs_allocated = n;
  return;
}

void BVH_CollideResult::add(int id1, int id2, BVH_REAL time)
{
  if(num_pairs >= num_pairs_allocated)
    sizeTo(num_pairs_allocated * 2 + 8);

  pairs[num_pairs].id1 = id1;
  pairs[num_pairs].id2 = id2;
  pairs[num_pairs].collide_time = time;
  num_pairs++;
}

void BVH_CollideResult::add(int id1, int id2, Vec3f contact_point, BVH_REAL penetration_depth, const Vec3f& normal, BVH_REAL time)
{
  if(num_pairs >= num_pairs_allocated)
    sizeTo(num_pairs_allocated * 2 + 8);

  pairs[num_pairs].id1 = id1;
  pairs[num_pairs].id2 = id2;
  pairs[num_pairs].collide_time = time;
  pairs[num_pairs].penetration_depth = penetration_depth;
  pairs[num_pairs].normal = normal;
  pairs[num_pairs].contact_point = contact_point;
  num_pairs++;
}


void collideRecurse(BVNode<OBB>* tree1, BVNode<OBB>* tree2,
                    const Vec3f R[3], const Vec3f& T,
                    int b1, int b2,
                    Vec3f* vertices1, Vec3f* vertices2,
                    Triangle* tri_indices1, Triangle* tri_indices2,
                    BVH_CollideResult* res, BVHFrontList* front_list)
{
  BVNode<OBB>* node1 = tree1 + b1;
  BVNode<OBB>* node2 = tree2 + b2;

  bool l1 = node1->isLeaf();
  bool l2 = node2->isLeaf();

  if(l1 && l2)
  {
    if(front_list) front_list->push_back(BVHFrontNode(b1, b2));

    res->num_bv_tests++;
    if(!overlap(R, T, node1->bv, node2->bv)) return;

    res->num_tri_tests++;

    const Triangle& tri_id1 = tri_indices1[-node1->first_child - 1];
    const Triangle& tri_id2 = tri_indices2[-node2->first_child - 1];

    const Vec3f& p1 = vertices1[tri_id1[0]];
    const Vec3f& p2 = vertices1[tri_id1[1]];
    const Vec3f& p3 = vertices1[tri_id1[2]];

    const Vec3f& q1 = vertices2[tri_id2[0]];
    const Vec3f& q2 = vertices2[tri_id2[1]];
    const Vec3f& q3 = vertices2[tri_id2[2]];

    BVH_REAL penetration;
    Vec3f normal;
    int n_contacts;
    Vec3f contacts[2];


    if(res->num_max_contacts == 0) // only interested in collision or not
    {
      if(Intersect::intersect_Triangle(p1, p2, p3, q1, q2, q3,
                                       R, T))
      {
          res->add(-node1->first_child - 1, -node2->first_child - 1);
      }
    }
    else  // need compute the contact information
    {
      if(Intersect::intersect_Triangle(p1, p2, p3, q1, q2, q3,
                                       R, T,
                                       contacts,
                                       (unsigned int*)&n_contacts,
                                       &penetration,
                                       &normal))
      {
        for(int i = 0; i < n_contacts; ++i)
        {
          if(res->num_max_contacts <= res->numPairs()) break;
          res->add(-node1->first_child - 1, -node2->first_child - 1, contacts[i], penetration, normal);
        }
      }
    }



    return;
  }

  res->num_bv_tests++;
  if(!overlap(R, T, node1->bv, node2->bv))
  {
    if(front_list) front_list->push_back(BVHFrontNode(b1, b2));
    return;
  }

  BVH_REAL sz1 = node1->bv.size();
  BVH_REAL sz2 = node2->bv.size();

  if(l2 || (!l1 && (sz1 > sz2)))
  {
    int c1 = node1->first_child;
    int c2 = c1 + 1;

    collideRecurse(tree1, tree2, R, T, c1, b2, vertices1, vertices2, tri_indices1, tri_indices2, res, front_list);

    if(res->numPairs() > 0 && ((res->num_max_contacts == 0) || (res->num_max_contacts <= res->numPairs()))) return;

    collideRecurse(tree1, tree2, R, T, c2, b2, vertices1, vertices2, tri_indices1, tri_indices2, res, front_list);
  }
  else
  {
    int c1 = node2->first_child;
    int c2 = c1 + 1;

    collideRecurse(tree1, tree2, R, T, b1, c1, vertices1, vertices2, tri_indices1, tri_indices2, res, front_list);

    if(res->numPairs() > 0 && ((res->num_max_contacts == 0) || (res->num_max_contacts <= res->numPairs()))) return;

    collideRecurse(tree1, tree2, R, T, b1, c2, vertices1, vertices2, tri_indices1, tri_indices2, res, front_list);
  }
}

void collideRecurse(BVNode<RSS>* tree1, BVNode<RSS>* tree2,
                    const Vec3f R[3], const Vec3f& T,
                    int b1, int b2,
                    Vec3f* vertices1, Vec3f* vertices2,
                    Triangle* tri_indices1, Triangle* tri_indices2,
                    BVH_CollideResult* res, BVHFrontList* front_list)
{
  BVNode<RSS>* node1 = tree1 + b1;
  BVNode<RSS>* node2 = tree2 + b2;

  bool l1 = node1->isLeaf();
  bool l2 = node2->isLeaf();

  if(l1 && l2)
  {
    if(front_list) front_list->push_back(BVHFrontNode(b1, b2));

    res->num_bv_tests++;
    if(!overlap(R, T, node1->bv, node2->bv)) return;

    res->num_tri_tests++;

    const Triangle& tri_id1 = tri_indices1[-node1->first_child - 1];
    const Triangle& tri_id2 = tri_indices2[-node2->first_child - 1];

    const Vec3f& p1 = vertices1[tri_id1[0]];
    const Vec3f& p2 = vertices1[tri_id1[1]];
    const Vec3f& p3 = vertices1[tri_id1[2]];

    const Vec3f& q1 = vertices2[tri_id2[0]];
    const Vec3f& q2 = vertices2[tri_id2[1]];
    const Vec3f& q3 = vertices2[tri_id2[2]];

    BVH_REAL penetration;
    Vec3f normal;
    int n_contacts;
    Vec3f contacts[2];


    if(res->num_max_contacts == 0) // only interested in collision or not
    {
      if(Intersect::intersect_Triangle(p1, p2, p3, q1, q2, q3,
                                       R, T))
      {
          res->add(-node1->first_child - 1, -node2->first_child - 1);
      }
    }
    else // need compute the contact information
    {
      if(Intersect::intersect_Triangle(p1, p2, p3, q1, q2, q3,
                                       R, T,
                                       contacts,
                                       (unsigned int*)&n_contacts,
                                       &penetration,
                                       &normal))
      {
        for(int i = 0; i < n_contacts; ++i)
        {
          if(res->num_max_contacts <= res->numPairs()) break;
          res->add(-node1->first_child - 1, -node2->first_child - 1, contacts[i], penetration, normal);
        }
      }
    }


    return;
  }

  res->num_bv_tests++;
  if(!overlap(R, T, node1->bv, node2->bv))
  {
    if(front_list) front_list->push_back(BVHFrontNode(b1, b2));
    return;
  }

  BVH_REAL sz1 = node1->bv.size();
  BVH_REAL sz2 = node2->bv.size();

  if(l2 || (!l1 && (sz1 > sz2)))
  {
    int c1 = node1->first_child;
    int c2 = c1 + 1;

    collideRecurse(tree1, tree2, R, T, c1, b2, vertices1, vertices2, tri_indices1, tri_indices2, res, front_list);

    if(res->numPairs() > 0 && ((res->num_max_contacts == 0) || (res->num_max_contacts <= res->numPairs()))) return;

    collideRecurse(tree1, tree2, R, T, c2, b2, vertices1, vertices2, tri_indices1, tri_indices2, res, front_list);
  }
  else
  {
    int c1 = node2->first_child;
    int c2 = c1 + 1;

    collideRecurse(tree1, tree2, R, T, b1, c1, vertices1, vertices2, tri_indices1, tri_indices2, res, front_list);

    if(res->numPairs() > 0 && ((res->num_max_contacts == 0) || (res->num_max_contacts <= res->numPairs()))) return;

    collideRecurse(tree1, tree2, R, T, b1, c2, vertices1, vertices2, tri_indices1, tri_indices2, res, front_list);
  }
}

void propagateBVHFrontList(BVNode<OBB>* tree1, BVNode<OBB>* tree2,
                           Vec3f R[3], const Vec3f& T,
                           Vec3f* vertices1, Vec3f* vertices2,
                           Triangle* tri_indices1, Triangle* tri_indices2,
                           BVH_CollideResult* res,
                           BVHFrontList* front_list)
{
  BVHFrontList::iterator front_iter;
  BVHFrontList append;
  for(front_iter = front_list->begin(); front_iter != front_list->end(); ++front_iter)
  {
    int b1 = front_iter->left;
    int b2 = front_iter->right;
    BVNode<OBB>* node1 = tree1 + b1;
    BVNode<OBB>* node2 = tree2 + b2;


    bool l1 = node1->isLeaf();
    bool l2 = node2->isLeaf();

    if(l1 & l2)
    {
      front_iter->valid = false; // the front node is no longer valid, in collideRecurse will add again.
      collideRecurse(tree1, tree2, R, T, b1, b2, vertices1, vertices2, tri_indices1, tri_indices2, res, &append);
    }
    else
    {
      res->num_bv_tests++;
      if(!overlap(R, T, node1->bv, node2->bv))
      {
        front_iter->valid = false; // the front node is no longer valid

        BVH_REAL sz1 = node1->bv.size();
        BVH_REAL sz2 = node2->bv.size();

        if(l2 || (!l1 && (sz1 > sz2)))
        {
          int c1 = node1->first_child;
          int c2 = c1 + 1;

          collideRecurse(tree1, tree2, R, T, c1, b2, vertices1, vertices2, tri_indices1, tri_indices2, res, &append);

          collideRecurse(tree1, tree2, R, T, c2, b2, vertices1, vertices2, tri_indices1, tri_indices2, res, &append);
        }
        else
        {
          int c1 = node2->first_child;
          int c2 = c1 + 1;

          collideRecurse(tree1, tree2, R, T, b1, c1, vertices1, vertices2, tri_indices1, tri_indices2, res, &append);

          collideRecurse(tree1, tree2, R, T, b1, c2, vertices1, vertices2, tri_indices1, tri_indices2, res, &append);
        }
      }
    }
  }


  // clean the old front list (remove invalid node)
  for(front_iter = front_list->begin(); front_iter != front_list->end();)
  {
    if(!front_iter->valid)
      front_iter = front_list->erase(front_iter);
    else
      ++front_iter;
  }

  for(front_iter = append.begin(); front_iter != append.end(); ++front_iter)
  {
    front_list->push_back(*front_iter);
  }
}


void propagateBVHFrontList(BVNode<RSS>* tree1, BVNode<RSS>* tree2,
                           Vec3f R[3], const Vec3f& T,
                           Vec3f* vertices1, Vec3f* vertices2,
                           Triangle* tri_indices1, Triangle* tri_indices2,
                           BVH_CollideResult* res,
                           BVHFrontList* front_list)
{
  BVHFrontList::iterator front_iter;
  BVHFrontList append;
  for(front_iter = front_list->begin(); front_iter != front_list->end(); ++front_iter)
  {
    int b1 = front_iter->left;
    int b2 = front_iter->right;
    BVNode<RSS>* node1 = tree1 + b1;
    BVNode<RSS>* node2 = tree2 + b2;


    bool l1 = node1->isLeaf();
    bool l2 = node2->isLeaf();

    if(l1 & l2)
    {
      front_iter->valid = false; // the front node is no longer valid, in collideRecurse will add again.
      collideRecurse(tree1, tree2, R, T, b1, b2, vertices1, vertices2, tri_indices1, tri_indices2, res, &append);
    }
    else
    {
      res->num_bv_tests++;
      if(!overlap(R, T, node1->bv, node2->bv))
      {
        front_iter->valid = false; // the front node is no longer valid

        BVH_REAL sz1 = node1->bv.size();
        BVH_REAL sz2 = node2->bv.size();

        if(l2 || (!l1 && (sz1 > sz2)))
        {
          int c1 = node1->first_child;
          int c2 = c1 + 1;

          collideRecurse(tree1, tree2, R, T, c1, b2, vertices1, vertices2, tri_indices1, tri_indices2, res, &append);

          collideRecurse(tree1, tree2, R, T, c2, b2, vertices1, vertices2, tri_indices1, tri_indices2, res, &append);
        }
        else
        {
          int c1 = node2->first_child;
          int c2 = c1 + 1;

          collideRecurse(tree1, tree2, R, T, b1, c1, vertices1, vertices2, tri_indices1, tri_indices2, res, &append);

          collideRecurse(tree1, tree2, R, T, b1, c2, vertices1, vertices2, tri_indices1, tri_indices2, res, &append);
        }
      }
    }
  }


  // clean the old front list (remove invalid node)
  for(front_iter = front_list->begin(); front_iter != front_list->end();)
  {
    if(!front_iter->valid)
      front_iter = front_list->erase(front_iter);
    else
      ++front_iter;
  }

  for(front_iter = append.begin(); front_iter != append.end(); ++front_iter)
  {
    front_list->push_back(*front_iter);
  }
}

}
