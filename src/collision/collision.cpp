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


#include "collision.h"

namespace collision_checking
{

int collide(const BVHModel<OBB>& model1, const Vec3f R1[3], const Vec3f& T1,
            const BVHModel<OBB>& model2, const Vec3f R2[3], const Vec3f& T2, BVH_CollideResult* res, BVHFrontList* front_list)
{
  if(model1.build_state != BVH_BUILD_STATE_PROCESSED && model1.build_state != BVH_BUILD_STATE_UPDATED)
  {
    std::cerr << "BVH Error: Must finish BVH model construction before call collide()!" << std::endl;
    return BVH_ERR_BUILD_OUT_OF_SEQUENCE;
  }

  if(model2.build_state != BVH_BUILD_STATE_PROCESSED && model2.build_state != BVH_BUILD_STATE_UPDATED)
  {
    std::cerr << "BVH Error: Must finish BVH model construction before call collide()!" << std::endl;
    return BVH_ERR_BUILD_OUT_OF_SEQUENCE;
  }

  // currently only support the mesh-mesh collision
  if(model1.getModelType() != BVH_MODEL_TRIANGLES || model2.getModelType()!= BVH_MODEL_TRIANGLES)
  {
    std::cerr << "BVH Error: Collision only supported between two triangle models." << std::endl;
    return BVH_ERR_UNSUPPORTED_FUNCTION;
  }


  res->resetRecord();

  // compute the transform from 1 to 2
  Vec3f R1_col[3];
  R1_col[0] = Vec3f(R1[0][0], R1[1][0], R1[2][0]);
  R1_col[1] = Vec3f(R1[0][1], R1[1][1], R1[2][1]);
  R1_col[2] = Vec3f(R1[0][2], R1[1][2], R1[2][2]);

  Vec3f R2_col[3];
  R2_col[0] = Vec3f(R2[0][0], R2[1][0], R2[2][0]);
  R2_col[1] = Vec3f(R2[0][1], R2[1][1], R2[2][1]);
  R2_col[2] = Vec3f(R2[0][2], R2[1][2], R2[2][2]);

  Vec3f R[3];
  R[0] = Vec3f(R1_col[0].dot(R2_col[0]), R1_col[0].dot(R2_col[1]), R1_col[0].dot(R2_col[2]));
  R[1] = Vec3f(R1_col[1].dot(R2_col[0]), R1_col[1].dot(R2_col[1]), R1_col[1].dot(R2_col[2]));
  R[2] = Vec3f(R1_col[2].dot(R2_col[0]), R1_col[2].dot(R2_col[1]), R1_col[2].dot(R2_col[2]));

  Vec3f Ttemp = T2 - T1;
  Vec3f T(R1_col[0].dot(Ttemp), R1_col[1].dot(Ttemp), R1_col[2].dot(Ttemp));

  if(front_list && front_list->size() > 0)
    propagateBVHFrontList(model1.bvs, model2.bvs, R, T, model1.vertices, model2.vertices, model1.tri_indices, model2.tri_indices, res, front_list);
  else
    collideRecurse(model1.bvs, model2.bvs, R, T, 0, 0, model1.vertices, model2.vertices, model1.tri_indices, model2.tri_indices, res, front_list);


  // update the contact position and normal
  // contact R1 * c + T1
  // normal R1^{-T} * n = R1 * n
  for(int i = 0; i < res->numPairs(); ++i)
  {
    BVHCollisionPair* pairs = res->collidePairs();
    Vec3f normal = pairs[i].normal;
    pairs[i].normal = Vec3f(R1[0].dot(normal), R1[1].dot(normal), R1[2].dot(normal));
    Vec3f pos = pairs[i].contact_point;
    pairs[i].contact_point = Vec3f(R1[0].dot(pos), R1[1].dot(pos), R1[2].dot(pos)) + T1;
  }

  return BVH_OK;
}


int collide(const BVHModel<RSS>& model1, const Vec3f R1[3], const Vec3f& T1,
            const BVHModel<RSS>& model2, const Vec3f R2[3], const Vec3f& T2, BVH_CollideResult* res, BVHFrontList* front_list)
{
  if(model1.build_state != BVH_BUILD_STATE_PROCESSED && model1.build_state != BVH_BUILD_STATE_UPDATED)
  {
    std::cerr << "BVH Error: Must finish BVH model construction before call collide()!" << std::endl;
    return BVH_ERR_BUILD_OUT_OF_SEQUENCE;
  }

  if(model2.build_state != BVH_BUILD_STATE_PROCESSED && model2.build_state != BVH_BUILD_STATE_UPDATED)
  {
    std::cerr << "BVH Error: Must finish BVH model construction before call collide()!" << std::endl;
    return BVH_ERR_BUILD_OUT_OF_SEQUENCE;
  }

  // currently only support the mesh-mesh collision
  if(model1.getModelType() != BVH_MODEL_TRIANGLES || model2.getModelType()!= BVH_MODEL_TRIANGLES)
  {
    std::cerr << "BVH Error: Collision only supported between two triangle models." << std::endl;
    return BVH_ERR_UNSUPPORTED_FUNCTION;
  }


  res->resetRecord();

  // compute the transform from 1 to 2
  Vec3f R1_col[3];
  R1_col[0] = Vec3f(R1[0][0], R1[1][0], R1[2][0]);
  R1_col[1] = Vec3f(R1[0][1], R1[1][1], R1[2][1]);
  R1_col[2] = Vec3f(R1[0][2], R1[1][2], R1[2][2]);

  Vec3f R2_col[3];
  R2_col[0] = Vec3f(R2[0][0], R2[1][0], R2[2][0]);
  R2_col[1] = Vec3f(R2[0][1], R2[1][1], R2[2][1]);
  R2_col[2] = Vec3f(R2[0][2], R2[1][2], R2[2][2]);

  Vec3f R[3];
  R[0] = Vec3f(R1_col[0].dot(R2_col[0]), R1_col[0].dot(R2_col[1]), R1_col[0].dot(R2_col[2]));
  R[1] = Vec3f(R1_col[1].dot(R2_col[0]), R1_col[1].dot(R2_col[1]), R1_col[1].dot(R2_col[2]));
  R[2] = Vec3f(R1_col[2].dot(R2_col[0]), R1_col[2].dot(R2_col[1]), R1_col[2].dot(R2_col[2]));

  Vec3f Ttemp = T2 - T1;
  Vec3f T(R1_col[0].dot(Ttemp), R1_col[1].dot(Ttemp), R1_col[2].dot(Ttemp));

  if(front_list && front_list->size() > 0)
    propagateBVHFrontList(model1.bvs, model2.bvs, R, T, model1.vertices, model2.vertices, model1.tri_indices, model2.tri_indices, res, front_list);
  else
    collideRecurse(model1.bvs, model2.bvs, R, T, 0, 0, model1.vertices, model2.vertices, model1.tri_indices, model2.tri_indices, res, front_list);


  // update the contact position and normal
  // contact R1 * c + T1
  // normal R1^{-T} * n = R1 * n
  for(int i = 0; i < res->numPairs(); ++i)
  {
    BVHCollisionPair* pairs = res->collidePairs();
    Vec3f normal = pairs[i].normal;
    pairs[i].normal = Vec3f(R1[0].dot(normal), R1[1].dot(normal), R1[2].dot(normal));
    Vec3f pos = pairs[i].contact_point;
    pairs[i].contact_point = Vec3f(R1[0].dot(pos), R1[1].dot(pos), R1[2].dot(pos)) + T1;
  }

  return BVH_OK;
}


}

