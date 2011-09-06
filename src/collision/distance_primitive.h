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

#ifndef COLLISION_CHECKING_DISTANCE_PRIMITIVE_H
#define COLLISION_CHECKING_DISTANCE_PRIMITIVE_H

#include "BVH_defs.h"
#include "BVH_model.h"
#include "BVH_front.h"

/** \brief Main namespace */
namespace collision_checking
{

struct BVH_DistanceResult
{
  /** \brief Number of BV collision test performed */
  int num_bv_tests;

  /** \brief Number of triangle collision test performed */
  int num_tri_tests;

  /** \brief Query time used */
  BVH_REAL query_time_seconds;

  /** \brief relative and absolute error, default value is 0.01 for both terms */
  BVH_REAL rel_err;
  BVH_REAL abs_err;

  /** \brief distance and points establishing the minimum distance for the models, within the relative and absolute error bounds specified.
   * p1 is in model1's local coordinate system while p2 is in model2's local coordinate system
   */
  BVH_REAL distance;
  Vec3f p1, p2;

  /** \brief Optional parameter controlling the size of a priority queue used to direct the search for closest points. A larger
   * queue can help the algorithm discover  the minimum with fewer steps, but will increase the cost of each step.
   * It is not beneficial to increase qsize if the application has frame-to-frame coherence, i.e., the pairs of models
   * taks small steps between each call,since the 'last tri_id' trick already accelerates this situtation with no overhead.
   * However, a queue size of 100 to 200 has been seen to save time in a motion planning application with "non-coherent" placements of models.
   */
  int qsize;

  /** \brief Remember the nearest neighbor points */
  int last_tri_id1;
  int last_tri_id2;

  BVH_DistanceResult();

  ~BVH_DistanceResult();

  /** \brief Reset the distance query statistics */
  void resetRecord()
  {
    num_bv_tests = 0;
    num_tri_tests = 0;
  }
};


/** \brief Recursive proximity kernel between two BV trees */
template<typename BV>
void distanceRecurse(BVNode<BV>* tree1, BVNode<BV>* tree2, int b1, int b2,
                    Vec3f* vertices1, Vec3f* vertices2,
                    Triangle* tri_indices1, Triangle* tri_indices2,
                    BVH_DistanceResult* res, BVHFrontList* front_list = NULL)
{
  std::cerr << "Bounding volume structure default can not support distance operation!" << std::endl;
}

/** \brief Recursive proximity kernel between two RSS trees */
void distanceRecurse(BVNode<RSS>* tree1, BVNode<RSS>* tree2,
                    const Vec3f R[3], const Vec3f& T,
                    int b1, int b2,
                    Vec3f* vertices1, Vec3f* vertices2,
                    Triangle* tri_indices1, Triangle* tri_indices2,
                    BVH_DistanceResult* res, BVHFrontList* front_list = NULL);


/** \brief Recursive proximity kernel between two RSS trees, using BVT queue acceleration */
void distanceQueueRecurse(BVNode<RSS>* tree1, BVNode<RSS>* tree2,
                          const Vec3f R[3], const Vec3f& T,
                          int b1, int b2,
                          Vec3f* vertices1, Vec3f* vertices2,
                          Triangle* tri_indices1, Triangle* tri_indices2,
                          BVH_DistanceResult* res, BVHFrontList* front_list = NULL);

}


#endif
