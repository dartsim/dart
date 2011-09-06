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


#ifndef COLLISION_CHECKING_SELF_COLLISION_H
#define COLLISION_CHECKING_SELF_COLLISION_H


#include "collision_primitive.h"
#include <iostream>

/** \brief Main namespace */
namespace collision_checking
{

/** \brief Continuous collision between two BVH models, support mesh-mesh or mesh-point clouds */
template<typename BV>
int continuousCollide(BVHModel<BV>& model1, BVHModel<BV>& model2, BVH_CollideResult* res, BVHFrontList* front_list = NULL)
{
  // currently support continuous collision for mesh-mesh, mesh-point cloud.
  if(model1.getModelType() == BVH_MODEL_POINTCLOUD && model2.getModelType() == BVH_MODEL_POINTCLOUD)
  {
    std::cerr << "BVH Error: Continuous collision only supported between two triangle models or one triangle model and one point cloud." << std::endl;
    return BVH_ERR_UNSUPPORTED_FUNCTION;
  }


  if(model1.build_state == BVH_BUILD_STATE_PROCESSED && model2.build_state == BVH_BUILD_STATE_PROCESSED)
  {
    std::cerr << "BVH Error: Continuous collision must have at least one object moving!" << std::endl;
    return BVH_ERR_UNSUPPORTED_FUNCTION;
  }
  else if(model1.build_state == BVH_BUILD_STATE_UPDATED && model2.build_state == BVH_BUILD_STATE_UPDATED)
  {
    // handle directly
  }
  else if(model1.build_state == BVH_BUILD_STATE_PROCESSED && model2.build_state == BVH_BUILD_STATE_UPDATED)
  {
    // model 1 static, model 2 moving
    model1.prev_vertices = model1.vertices;
  }
  else if(model1.build_state == BVH_BUILD_STATE_UPDATED && model2.build_state == BVH_BUILD_STATE_PROCESSED)
  {
    model2.prev_vertices = model2.vertices;
  }
  else
  {
    std::cerr << "BVH Error: Must finish BVH model construction before call collide()!" << std::endl;
    return BVH_ERR_BUILD_OUT_OF_SEQUENCE;
  }

  res->resetRecord();

  if(front_list && front_list->size() > 0)
    continuousPropagateBVHFrontList(model1.bvs, model2.bvs, model1.vertices, model2.vertices, model1.prev_vertices, model2.prev_vertices, model1.tri_indices, model2.tri_indices, res, front_list);
  else
    continuousCollideRecurse(model1.bvs, model2.bvs, 0, 0, model1.vertices, model2.vertices, model1.prev_vertices, model2.prev_vertices, model1.tri_indices, model2.tri_indices, res, front_list);

  if(model1.build_state == BVH_BUILD_STATE_PROCESSED && model2.build_state == BVH_BUILD_STATE_UPDATED)
  {
    // model 1 static, model 2 moving
    model1.prev_vertices = NULL;
  }
  else if(model1.build_state == BVH_BUILD_STATE_UPDATED && model2.build_state == BVH_BUILD_STATE_PROCESSED)
  {
    model2.prev_vertices = NULL;
  }

  return BVH_OK;
}

/** \brief Continuous self collision for one BVH model, only support mesh */
template<typename BV>
int continuousSelfCollide(BVHModel<BV>& model, BVH_CollideResult* res, BVHFrontList* front_list = NULL)
{
  // currently support continuous self collision for mesh-mesh.
  if(model.getModelType() != BVH_MODEL_TRIANGLES)
  {
    std::cerr << "BVH Error: Continuous self collision only supported for a triangle model." << std::endl;
    return BVH_ERR_UNSUPPORTED_FUNCTION;
  }

  if(model.build_state != BVH_BUILD_STATE_UPDATED)
  {
    if(model.build_state == BVH_BUILD_STATE_PROCESSED)
    {
      std::cerr << "BVH Error: Continuous self collision only handles moving object!" << std::endl;
      return BVH_ERR_UNSUPPORTED_FUNCTION;
    }
    else
    {
      std::cerr << "BVH Error: Must finish BVH model construction before call collide()!" << std::endl;
      return BVH_ERR_BUILD_OUT_OF_SEQUENCE;
    }
  }

  res->resetRecord();

  if(front_list && front_list->size() > 0)
    continuousPropagateBVHFrontList(model.bvs, model.bvs, model.vertices, model.vertices, model.prev_vertices, model.prev_vertices, model.tri_indices, model.tri_indices, res, front_list);
  else
    continuousSelfCollideRecurse(model.bvs, 0, model.vertices, model.prev_vertices, model.tri_indices, res, front_list);

  return BVH_OK;
}



}

#endif
