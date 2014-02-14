/*
 * Copyright (c) 2013-2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef SOFT_COLLISION_FCL_MESH_SOFTFCLMESHCOLLISIONNODE_H_
#define SOFT_COLLISION_FCL_MESH_SOFTFCLMESHCOLLISIONNODE_H_

#include <assimp/mesh.h>
#include <dart/collision/fcl_mesh/FCLMeshCollisionNode.h>

namespace dart {
namespace collision {

/// \brief
class SoftFCLMeshCollisionNode : public FCLMeshCollisionNode
{
public:
  /// \brief
  explicit SoftFCLMeshCollisionNode(dynamics::BodyNode* _bodyNode);

  /// \brief
  virtual ~SoftFCLMeshCollisionNode();

  // Documentation inherited.
  virtual bool detectCollision(FCLMeshCollisionNode* _otherNode,
                               std::vector<Contact>* _contactPoints,
                               int _num_max_contact);

  /// \brief
  void updateShape();

protected:
};

template<class BV>
fcl::BVHModel<BV>* createSoftMesh(const aiMesh* _mesh,
                                  const fcl::Transform3f& _transform);

}  // namespace collision
}  // namespace dart

#endif  // SOFT_COLLISION_FCL_MESH_SOFTFCLMESHCOLLISIONNODE_H_
