/*
 * Copyright (c) 2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
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

#include "dart/collision/fcl/FCLCollisionObject.hpp"

#include <fcl/BVH/BVH_model.h>

#include "dart/collision/fcl/FCLTypes.hpp"
#include "dart/dynamics/SoftMeshShape.hpp"
#include "dart/dynamics/ShapeFrame.hpp"

namespace dart {
namespace collision {

//==============================================================================
FCLCollisionObject::UserData::UserData(CollisionObject* collisionObject)
  : mCollisionObject(collisionObject)
{
  // Do nothing
}

//==============================================================================
fcl::CollisionObject* FCLCollisionObject::getFCLCollisionObject()
{
  return mFCLCollisionObject.get();
}

//==============================================================================
const fcl::CollisionObject* FCLCollisionObject::getFCLCollisionObject() const
{
  return mFCLCollisionObject.get();
}

//==============================================================================
FCLCollisionObject::FCLCollisionObject(
    CollisionDetector* collisionDetector,
    const dynamics::ShapeFrame* shapeFrame,
    const fcl_shared_ptr<fcl::CollisionGeometry>& fclCollGeom)
  : CollisionObject(collisionDetector, shapeFrame),
    mFCLCollisionObjectUserData(new UserData(this)),
    mFCLCollisionObject(new fcl::CollisionObject(fclCollGeom))
{
  mFCLCollisionObject->setUserData(mFCLCollisionObjectUserData.get());
}

//==============================================================================
void FCLCollisionObject::updateEngineData()
{
  using dart::dynamics::BodyNode;
  using dart::dynamics::Shape;
  using dart::dynamics::SoftMeshShape;

  auto shape = mShapeFrame->getShape().get();

  // Update soft-body's vertices
  if (shape->getType() == dynamics::SoftMeshShape::getStaticType())
  {
    assert(dynamic_cast<const SoftMeshShape*>(shape));
    auto softMeshShape = static_cast<const SoftMeshShape*>(shape);

    const aiMesh* mesh = softMeshShape->getAssimpMesh();
    const_cast<SoftMeshShape*>(softMeshShape)->update();
    // TODO(JS): update function be called by somewhere out of here.

#if FCL_VERSION_AT_LEAST(0,3,0)
    auto collGeom = const_cast<fcl::CollisionGeometry*>(
          mFCLCollisionObject->collisionGeometry().get());
#else
    fcl::CollisionGeometry* collGeom
        = const_cast<fcl::CollisionGeometry*>(
          mFCLCollisionObject->getCollisionGeometry());
#endif
    assert(dynamic_cast<fcl::BVHModel<fcl::OBBRSS>*>(collGeom));
    auto bvhModel = static_cast<fcl::BVHModel<fcl::OBBRSS>*>(collGeom);

    bvhModel->beginUpdateModel();
    for (auto i = 0u; i < mesh->mNumFaces; ++i)
    {
      fcl::Vec3f vertices[3];
      for (auto j = 0u; j < 3; ++j)
      {
        const auto& vertex = mesh->mVertices[mesh->mFaces[i].mIndices[j]];
        vertices[j] = fcl::Vec3f(vertex.x, vertex.y, vertex.z);
      }
      bvhModel->updateTriangle(vertices[0], vertices[1], vertices[2]);
    }
    bvhModel->endUpdateModel();
  }

  mFCLCollisionObject->setTransform(FCLTypes::convertTransform(getTransform()));
  mFCLCollisionObject->computeAABB();
}

}  // namespace collision
}  // namespace dart
