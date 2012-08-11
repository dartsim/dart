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


#ifndef COLLISION_CHECKING_PROXIMITY_H
#define COLLISION_CHECKING_PROXIMITY_H

#include "distance_primitive.h"


namespace collision_checking
{

/** \brief Proximity query between two BVH models, only support mesh-mesh now */
template<typename BV>
int distance(const BVHModel<BV>& model1, const BVHModel<BV>& model2, BVH_DistanceResult* res, BVHFrontList* front_list = NULL)
{
  if(model1.build_state != BVH_BUILD_STATE_PROCESSED && model1.build_state != BVH_BUILD_STATE_UPDATED)
  {
    std::cerr << "BVH Error: Must finish BVH model construction before call distance()!" << std::endl;
    return BVH_ERR_BUILD_OUT_OF_SEQUENCE;
  }

  {
    std::cerr << "BVH Error: Must finish BVH model construction before call distance()!" << std::endl;
    return BVH_ERR_BUILD_OUT_OF_SEQUENCE;
  }

  // currently only support the mesh-mesh collision
  if(model1.getModelType() != BVH_MODEL_TRIANGLES || model2.getModelType()!= BVH_MODEL_TRIANGLES)
  {
    std::cerr << "BVH Error: Proximity query only supported between two triangle models." << std::endl;
    return BVH_ERR_UNSUPPORTED_FUNCTION;
  }

  std::cerr << "BVH Error: Proximity query does not support for most BVs." << std::endl;

  return BVH_ERR_UNSUPPORTED_FUNCTION;
}





/** \brief Proximity query between two RSS models, only support mesh-mesh
 * For RSS, we provide a specification that need not update the mesh vertices
 */
int distance(const BVHModel<RSS>& model1, const Vec3f R1[3], const Vec3f& T1,
                  const BVHModel<RSS>& model2, const Vec3f R2[3], const Vec3f& T2, BVH_DistanceResult* res, BVHFrontList* front_list = NULL);

}


#endif
