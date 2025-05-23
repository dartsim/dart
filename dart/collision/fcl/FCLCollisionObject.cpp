/*
 * Copyright (c) 2011-2025, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
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

#include "dart/collision/fcl/FCLTypes.hpp"
#include "dart/dynamics/ShapeFrame.hpp"
#include "dart/dynamics/SoftMeshShape.hpp"

namespace dart {
namespace collision {

//==============================================================================
dart::collision::fcl::CollisionObject*
FCLCollisionObject::getFCLCollisionObject()
{
  return mFCLCollisionObject.get();
}

//==============================================================================
const dart::collision::fcl::CollisionObject*
FCLCollisionObject::getFCLCollisionObject() const
{
  return mFCLCollisionObject.get();
}

//==============================================================================
FCLCollisionObject::FCLCollisionObject(
    CollisionDetector* collisionDetector,
    const dynamics::ShapeFrame* shapeFrame,
    const std::shared_ptr<dart::collision::fcl::CollisionGeometry>& fclCollGeom)
  : CollisionObject(collisionDetector, shapeFrame),
    mFCLCollisionObject(new dart::collision::fcl::CollisionObject(fclCollGeom))
{
  mFCLCollisionObject->setUserData(this);
}

//==============================================================================
void FCLCollisionObject::updateEngineData()
{
  using dart::dynamics::BodyNode;
  using dart::dynamics::Shape;
  using dart::dynamics::SoftMeshShape;

  auto shape = mShapeFrame->getShape().get();

  // Update soft-body's vertices
  if (shape->getType() == dynamics::SoftMeshShape::getStaticType()) {
    assert(dynamic_cast<const SoftMeshShape*>(shape));
    auto softMeshShape = static_cast<const SoftMeshShape*>(shape);

    const aiMesh* mesh = softMeshShape->getAssimpMesh();
    const_cast<SoftMeshShape*>(softMeshShape)->update();
    // TODO(JS): update function be called by somewhere out of here.

    auto collGeom = const_cast<dart::collision::fcl::CollisionGeometry*>(
        mFCLCollisionObject->collisionGeometry().get());
    assert(
        dynamic_cast<::fcl::BVHModel<dart::collision::fcl::OBBRSS>*>(collGeom));
    auto bvhModel
        = static_cast<::fcl::BVHModel<dart::collision::fcl::OBBRSS>*>(collGeom);

    bvhModel->beginUpdateModel();
    for (auto i = 0u; i < mesh->mNumFaces; ++i) {
      dart::collision::fcl::Vector3 vertices[3];
      for (auto j = 0u; j < 3; ++j) {
        const auto& vertex = mesh->mVertices[mesh->mFaces[i].mIndices[j]];
        vertices[j]
            = dart::collision::fcl::Vector3(vertex.x, vertex.y, vertex.z);
      }
      bvhModel->updateTriangle(vertices[0], vertices[1], vertices[2]);
    }
    bvhModel->endUpdateModel();
  }

  mFCLCollisionObject->setTransform(FCLTypes::convertTransform(getTransform()));
  mFCLCollisionObject->computeAABB();
}

} // namespace collision
} // namespace dart
