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


#ifndef COLLISION_CHECKING_COLLISION_PRIMITIVE_H
#define COLLISION_CHECKING_COLLISION_PRIMITIVE_H

#include "BVH_defs.h"
#include "intersect.h"
#include "BVH_model.h"
#include "BVH_front.h"

/** \brief Main namespace */
namespace collision_checking
{

/** \brief The indices of in-collision primitives of objects */
struct BVHCollisionPair
{
  /** \brief The index of one in-collision primitive */
  int id1;

  /** \brief The index of the other in-collision primitive */
  int id2;

  /** \brief Collision time normalized in [0, 1]. The collision time out of [0, 1] means collision-free */
  BVH_REAL collide_time;

  /** \brief Contact normal */
  Vec3f normal;

  /** \brief Contact points */
  Vec3f contact_point;

  /** \brief Penetration depth for two triangles */
  BVH_REAL penetration_depth;
};

/** \brief A class describing the collision result */
struct BVH_CollideResult
{
  /** \brief Number of BV collision test performed */
  int num_bv_tests;

  /** \brief Number of triangle collision test performed */
  int num_tri_tests;

  /** \brief Number of vertex-face test performed (in CCD) */
  int num_vf_tests;

  /** \brief Number of edge-edge test performed (in CCD) */
  int num_ee_tests;

  /** \brief Query time used */
  BVH_REAL query_time_seconds;

  /** \brief Maximum number of contacts, default 0 contact
   * contact = 0, means only interested in collision
   * contact > 0, means we need to compute contact pos, normal
   */
  int num_max_contacts;

  BVH_CollideResult();

  ~BVH_CollideResult();

  /** \brief Add one collision pair */
  void add(int id1, int id2, BVH_REAL time = 0);

  void add(int id1, int id2, Vec3f contact_point, BVH_REAL penetration_depth, const Vec3f& normal, BVH_REAL time = 0);

  /** \brief Whether a collision happens */
  int colliding() { return (num_pairs > 0); }

  /** \brief Reset the collision statistics */
  void resetRecord()
  {
    num_pairs = 0;
    num_bv_tests = 0;
    num_tri_tests = 0;
    num_vf_tests = 0;
    num_ee_tests = 0;
  }

  /** \brief Number of collision pairs found */
  int numPairs() const
  {
    return num_pairs;
  }

  /** \brief Return the collision primitive id of one object */
  int id1(int i) const
  {
    return pairs[i].id1;
  }

  /** \brief Return the collision primitive id of the other object */
  int id2(int i) const
  {
    return pairs[i].id2;
  }

  /** \brief Return the collision pairs */
  BVHCollisionPair* collidePairs()
  {
    return pairs;
  }

private:
  int num_pairs_allocated;
  int num_pairs;
  BVHCollisionPair* pairs;

  void sizeTo(int n);
};


/** \brief Recursive collision kernel between between two BV trees */
template<typename BV>
void collideRecurse(BVNode<BV>* tree1, BVNode<BV>* tree2, int b1, int b2,
                    Vec3f* vertices1, Vec3f* vertices2,
                    Triangle* tri_indices1, Triangle* tri_indices2,
                    BVH_CollideResult* res, BVHFrontList* front_list = NULL)
{
  BVNode<BV>* node1 = tree1 + b1;
  BVNode<BV>* node2 = tree2 + b2;

  bool l1 = node1->isLeaf();
  bool l2 = node2->isLeaf();

  if(l1 && l2)
  {
    if(front_list) front_list->push_back(BVHFrontNode(b1, b2));

    res->num_bv_tests++;
    if(!node1->overlap(*node2)) return;

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
      if(Intersect::intersect_Triangle(p1, p2, p3, q1, q2, q3))
      {
          res->add(-node1->first_child - 1, -node2->first_child - 1);
      }
    }
    else // need compute the contact information
    {
      if(Intersect::intersect_Triangle(p1, p2, p3, q1, q2, q3,
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
  if(!node1->overlap(*node2))
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

    collideRecurse(tree1, tree2, c1, b2, vertices1, vertices2, tri_indices1, tri_indices2, res, front_list);

    if(res->numPairs() > 0 && ((res->num_max_contacts == 0) || (res->num_max_contacts <= res->numPairs()))) return;

    collideRecurse(tree1, tree2, c2, b2, vertices1, vertices2, tri_indices1, tri_indices2, res, front_list);
  }
  else
  {
    int c1 = node2->first_child;
    int c2 = c1 + 1;

    collideRecurse(tree1, tree2, b1, c1, vertices1, vertices2, tri_indices1, tri_indices2, res, front_list);

    if(res->numPairs() > 0 && ((res->num_max_contacts == 0) || (res->num_max_contacts <= res->numPairs()))) return;

    collideRecurse(tree1, tree2, b1, c2, vertices1, vertices2, tri_indices1, tri_indices2, res, front_list);
  }
}

/** \brief Recursive collision kernel between two OBB trees: for rigid motion */
void collideRecurse(BVNode<OBB>* tree1, BVNode<OBB>* tree2,
                    const Vec3f R[3], const Vec3f& T,
                    int b1, int b2,
                    Vec3f* vertices1, Vec3f* vertices2,
                    Triangle* tri_indices1, Triangle* tri_indices2,
                    BVH_CollideResult* res, BVHFrontList* front_list = NULL);

/** \brief Recursive collision kernel between two RSS trees: for rigid motion */
void collideRecurse(BVNode<RSS>* tree1, BVNode<RSS>* tree2,
                    const Vec3f R[3], const Vec3f& T,
                    int b1, int b2,
                    Vec3f* vertices1, Vec3f* vertices2,
                    Triangle* tri_indices1, Triangle* tri_indices2,
                    BVH_CollideResult* res, BVHFrontList* front_list = NULL);

/** \brief Recursive self collision kernel on one BV tree */
template<typename BV>
void selfCollideRecurse(BVNode<BV>* tree, int b,
                        Vec3f* vertices, Triangle* tri_indices,
                        BVH_CollideResult* res, BVHFrontList* front_list = NULL)
{
  BVNode<BV>* node = tree + b;
  bool l = node->isLeaf();

  if(l) return;

  int c1 = node->first_child;
  int c2 = c1 + 1;

  selfCollideRecurse(tree, c1, vertices, tri_indices, res, front_list);
  selfCollideRecurse(tree, c2, vertices, tri_indices, res, front_list);
  collideRecurse<BV>(tree, tree, c1, c2, vertices, vertices, tri_indices, tri_indices, res, front_list);
}

/** \brief Recursive continuous collision kernel between between two BV trees */
template<typename BV>
void continuousCollideRecurse(BVNode<BV>* tree1, BVNode<BV>* tree2, int b1, int b2,
                              Vec3f* vertices1, Vec3f* vertices2,
                              Vec3f* prev_vertices1, Vec3f* prev_vertices2,
                              Triangle* tri_indices1, Triangle* tri_indices2,
                              BVH_CollideResult* res, BVHFrontList* front_list = NULL)
{
  BVNode<BV>* node1 = tree1 + b1;
  BVNode<BV>* node2 = tree2 + b2;

  bool l1 = node1->isLeaf();
  bool l2 = node2->isLeaf();

  if(l1 && l2)
  {
    if(front_list) front_list->push_back(BVHFrontNode(b1, b2));

    res->num_bv_tests++;
    if(!node1->overlap(*node2)) return;


    BVH_REAL collide_time = 2;
    Vec3f collide_pos;

    if(tri_indices1 && tri_indices2) // both are triangle mesh
    {
      const Triangle& tri_id1 = tri_indices1[-node1->first_child - 1];
      const Triangle& tri_id2 = tri_indices2[-node2->first_child - 1];

      Vec3f S0[3];
      Vec3f S1[3];
      Vec3f T0[3];
      Vec3f T1[3];


      for(int i = 0; i < 3; ++i)
      {
        S0[i] = prev_vertices1[tri_id1[i]];
        S1[i] = vertices1[tri_id1[i]];
        T0[i] = prev_vertices2[tri_id2[i]];
        T1[i] = vertices2[tri_id2[i]];
      }

      BVH_REAL tmp;
      Vec3f tmpv;

      // 6 VF checks
      for(int i = 0; i < 3; ++i)
      {
        res->num_vf_tests++;
        if(Intersect::intersect_VF(S0[0], S0[1], S0[2], T0[i], S1[0], S1[1], S1[2], T1[i], &tmp, &tmpv))
        {
          if(collide_time > tmp)
          {
            collide_time = tmp; collide_pos = tmpv;
          }
        }

        res->num_vf_tests++;
        if(Intersect::intersect_VF(T0[0], T0[1], T0[2], S0[i], T1[0], T1[1], T1[2], S1[i], &tmp, &tmpv))
        {
          if(collide_time > tmp)
          {
            collide_time = tmp; collide_pos = tmpv;
          }
        }
      }

      // 9 EE checks
      for(int i = 0; i < 3; ++i)
      {
        int S_id1 = i;
        int S_id2 = i + 1;
        if(S_id2 == 3) S_id2 = 0;
        for(int j = 0; j < 3; ++j)
        {
          int T_id1 = j;
          int T_id2 = j + 1;
          if(T_id2 == 3) T_id2 = 0;

          res->num_ee_tests++;
          if(Intersect::intersect_EE(S0[S_id1], S0[S_id2], T0[T_id1], T0[T_id2], S1[S_id1], S1[S_id2], T1[T_id1], T1[T_id2], &tmp, &tmpv))
          {
            if(collide_time > tmp)
            {
              collide_time = tmp; collide_pos = tmpv;
            }
          }
        }
      }


    }
    else if(tri_indices1) // the first is triangle mesh
    {
      BVH_REAL collide_time = 2;
      Vec3f collide_pos;

      const Triangle& tri_id1 = tri_indices1[-node1->first_child - 1];
      int vertex_id2 = -node2->first_child - 1;

      Vec3f S0[3];
      Vec3f S1[3];

      for(int i = 0; i < 3; ++i)
      {
        S0[i] = prev_vertices1[tri_id1[i]];
        S1[i] = vertices1[tri_id1[i]];
      }
      Vec3f& T0 = prev_vertices2[vertex_id2];
      Vec3f& T1 = vertices2[vertex_id2];

      BVH_REAL tmp;
      Vec3f tmpv;

      // 3 VF checks
      for(int i = 0; i < 3; ++i)
      {
        res->num_vf_tests++;
        if(Intersect::intersect_VF(S0[0], S0[1], S0[2], T0, S1[0], S1[1], S1[2], T1, &tmp, &tmpv))
        {
          if(collide_time > tmp)
          {
            collide_time = tmp; collide_pos = tmpv;
          }
        }
      }


    }
    else if(tri_indices2) // the second is triangle mesh
    {
      BVH_REAL collide_time = 2;
      Vec3f collide_pos;

      int vertex_id1 = -node1->first_child - 1;
      const Triangle& tri_id2 = tri_indices2[-node2->first_child - 1];

      Vec3f& S0 = prev_vertices1[vertex_id1];
      Vec3f& S1 = vertices1[vertex_id1];

      Vec3f T0[3];
      Vec3f T1[3];
      for(int i = 0; i < 3; ++i)
      {
        T0[i] = prev_vertices2[tri_id2[i]];
        T1[i] = vertices2[tri_id2[i]];
      }

      BVH_REAL tmp;
      Vec3f tmpv;

      // 3 VF checks
      for(int i = 0; i < 3; ++i)
      {
        res->num_vf_tests++;
        if(Intersect::intersect_VF(T0[0], T0[1], T0[2], S0, T1[0], T1[1], T1[2], S1, &tmp, &tmpv))
        {
          if(collide_time > tmp)
          {
            collide_time = tmp; collide_pos = tmpv;
          }
        }
      }


    }
    else // both are point clouds
    {
      // can not handle now!
    }

    if(!(collide_time > 1)) // collision happens
    {
      res->add(-node1->first_child - 1, -node2->first_child - 1, collide_time);
    }

    return;
  }

  res->num_bv_tests++;
  if(!node1->overlap(*node2))
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

    continuousCollideRecurse(tree1, tree2, c1, b2, vertices1, vertices2, prev_vertices1, prev_vertices2, tri_indices1, tri_indices2, res, front_list);

    continuousCollideRecurse(tree1, tree2, c2, b2, vertices1, vertices2, prev_vertices1, prev_vertices2, tri_indices1, tri_indices2, res, front_list);
  }
  else
  {
    int c1 = node2->first_child;
    int c2 = c1 + 1;

    continuousCollideRecurse(tree1, tree2, b1, c1, vertices1, vertices2, prev_vertices1, prev_vertices2, tri_indices1, tri_indices2, res, front_list);

    continuousCollideRecurse(tree1, tree2, b1, c2, vertices1, vertices2, prev_vertices1, prev_vertices2, tri_indices1, tri_indices2, res, front_list);
  }
}

/** \brief Recursive continuous self collision kernel between between two BV trees */
template<typename BV>
void continuousSelfCollideRecurse(BVNode<BV>* tree, int b,
                        Vec3f* vertices, Vec3f* prev_vertices,
                        Triangle* tri_indices,
                        BVH_CollideResult* res, BVHFrontList* front_list = NULL)
{
  BVNode<BV>* node = tree + b;
  bool l = node->isLeaf();

  if(l) return;

  int c1 = node->first_child;
  int c2 = c1 + 1;

  continuousSelfCollideRecurse(tree, c1, vertices, prev_vertices, tri_indices, res, front_list);
  continuousSelfCollideRecurse(tree, c2, vertices, prev_vertices, tri_indices, res, front_list);
  continuousCollideRecurse<BV>(tree, tree, c1, c2, vertices, vertices, prev_vertices, prev_vertices, tri_indices, tri_indices, res, front_list);
}


/** \brief BVH front list propagation for collision */
template<typename BV>
void propagateBVHFrontList(BVNode<BV>* tree1, BVNode<BV>* tree2,
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
    BVNode<BV>* node1 = tree1 + b1;
    BVNode<BV>* node2 = tree2 + b2;


    bool l1 = node1->isLeaf();
    bool l2 = node2->isLeaf();

    if(l1 & l2)
    {
      front_iter->valid = false; // the front node is no longer valid, in collideRecurse will add again.
      collideRecurse(tree1, tree2, b1, b2, vertices1, vertices2, tri_indices1, tri_indices2, res, &append);
    }
    else
    {
      res->num_bv_tests++;
      if(node1->overlap(*node2))
      {
        front_iter->valid = false; // the front node is no longer valid

        BVH_REAL sz1 = node1->bv.size();
        BVH_REAL sz2 = node2->bv.size();

        if(l2 || (!l1 && (sz1 > sz2)))
        {
          int c1 = node1->first_child;
          int c2 = c1 + 1;

          collideRecurse(tree1, tree2, c1, b2, vertices1, vertices2, tri_indices1, tri_indices2, res, &append);

          collideRecurse(tree1, tree2, c2, b2, vertices1, vertices2, tri_indices1, tri_indices2, res, &append);
        }
        else
        {
          int c1 = node2->first_child;
          int c2 = c1 + 1;

          collideRecurse(tree1, tree2, b1, c1, vertices1, vertices2, tri_indices1, tri_indices2, res, &append);

          collideRecurse(tree1, tree2, b1, c2, vertices1, vertices2, tri_indices1, tri_indices2, res, &append);
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

/** \brief BVH front list propagation for collision of OBB trees */
void propagateBVHFrontList(BVNode<OBB>* tree1, BVNode<OBB>* tree2,
                           Vec3f R[3], const Vec3f& T,
                           Vec3f* vertices1, Vec3f* vertices2,
                           Triangle* tri_indices1, Triangle* tri_indices2,
                           BVH_CollideResult* res,
                           BVHFrontList* front_list);

/** \brief RSS front list propagation for collision of RSS trees */
void propagateBVHFrontList(BVNode<RSS>* tree1, BVNode<RSS>* tree2,
                           Vec3f R[3], const Vec3f& T,
                           Vec3f* vertices1, Vec3f* vertices2,
                           Triangle* tri_indices1, Triangle* tri_indices2,
                           BVH_CollideResult* res,
                           BVHFrontList* front_list);

/** \brief BVH front list propagation for continuous collision */
template<typename BV>
void continuousPropagateBVHFrontList(BVNode<BV>* tree1, BVNode<BV>* tree2,
                        Vec3f* vertices1, Vec3f* vertices2,
                        Vec3f* prev_vertices1, Vec3f* prev_vertices2,
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
    BVNode<BV>* node1 = tree1 + b1;
    BVNode<BV>* node2 = tree2 + b2;


    bool l1 = node1->isLeaf();
    bool l2 = node2->isLeaf();

    if(l1 & l2)
    {
      front_iter->valid = false; // the front node is no longer valid, in collideRecurse will add again.
      continuousCollideRecurse(tree1, tree2, b1, b2, vertices1, vertices2, prev_vertices1, prev_vertices2, tri_indices1, tri_indices2, res, &append);
    }
    else
    {
      res->num_bv_tests++;
      if(node1->overlap(*node2))
      {
        front_iter->valid = false; // the front node is no longer valid

        BVH_REAL sz1 = node1->bv.size();
        BVH_REAL sz2 = node2->bv.size();

        if(l2 || (!l1 && (sz1 > sz2)))
        {
          int c1 = node1->first_child;
          int c2 = c1 + 1;

          continuousCollideRecurse(tree1, tree2, c1, b2, vertices1, vertices2, prev_vertices1, prev_vertices2, tri_indices1, tri_indices2, res, &append);

          continuousCollideRecurse(tree1, tree2, c2, b2, vertices1, vertices2, prev_vertices1, prev_vertices2, tri_indices1, tri_indices2, res, &append);
        }
        else
        {
          int c1 = node2->first_child;
          int c2 = c1 + 1;

          continuousCollideRecurse(tree1, tree2, b1, c1, vertices1, vertices2, prev_vertices1, prev_vertices2, tri_indices1, tri_indices2, res, &append);

          continuousCollideRecurse(tree1, tree2, b1, c2, vertices1, vertices2, prev_vertices1, prev_vertices2, tri_indices1, tri_indices2, res, &append);
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

#endif
