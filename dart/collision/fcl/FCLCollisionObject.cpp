/*
 * Copyright (c) 2011, The DART development contributors
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
#include "dart/common/Macros.hpp"
#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/PointMass.hpp"
#include "dart/dynamics/ShapeFrame.hpp"
#include "dart/dynamics/ShapeNode.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/dynamics/SoftBodyNode.hpp"
#include "dart/dynamics/SoftMeshShape.hpp"

namespace dart {
namespace collision {

namespace {

//==============================================================================
bool transformsEqual(
    const dart::collision::fcl::Transform3& lhs,
    const dart::collision::fcl::Transform3& rhs)
{
  return lhs.matrix().cwiseEqual(rhs.matrix()).all();
}

//==============================================================================
class MutableFCLCollisionObject : public dart::collision::fcl::CollisionObject
{
public:
  explicit MutableFCLCollisionObject(
      const std::shared_ptr<dart::collision::fcl::CollisionGeometry>&
          fclCollGeom)
    : dart::collision::fcl::CollisionObject(fclCollGeom)
  {
    // Do nothing
  }

  void setCollisionGeometry(
      const std::shared_ptr<dart::collision::fcl::CollisionGeometry>&
          fclCollGeom)
  {
    cgeom = fclCollGeom;
    cgeom_const = fclCollGeom;

    if (cgeom) {
      cgeom->computeLocalAABB();
      computeAABB();
    }
  }
};

} // namespace

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
    mFCLCollisionObject(new MutableFCLCollisionObject(fclCollGeom))
{
  mFCLCollisionObject->setUserData(this);

  if (shapeFrame) {
    const auto* shapeNode
        = dynamic_cast<const dynamics::ShapeNode*>(shapeFrame);
    if (shapeNode) {
      const auto bodyNode = shapeNode->getBodyNodePtr();
      const auto skeleton = bodyNode ? bodyNode->getSkeleton() : nullptr;

      if (skeleton)
        mKey += skeleton->getName() + "::";
      if (bodyNode)
        mKey += bodyNode->getName() + "::";
      mKey += shapeNode->getName();
    } else {
      mKey = shapeFrame->getName();
    }
  }

  refreshSoftMeshCache();
}

//==============================================================================
void FCLCollisionObject::setFCLCollisionGeometry(
    const std::shared_ptr<dart::collision::fcl::CollisionGeometry>& fclCollGeom)
{
  auto mutableObject
      = static_cast<MutableFCLCollisionObject*>(mFCLCollisionObject.get());
  DART_ASSERT(mutableObject);

  mutableObject->setCollisionGeometry(fclCollGeom);
  refreshSoftMeshCache();
  mHasCachedFCLTransform = false;
}

//==============================================================================
void FCLCollisionObject::refreshSoftMeshCache()
{
  mSoftMeshShape = nullptr;
  mSoftBodyNode = nullptr;
  mSoftMesh = nullptr;
  mSoftMeshBvhModel = nullptr;

  auto shape = mShapeFrame->getShape().get();
  if (shape->getType() != dynamics::SoftMeshShape::getStaticType())
    return;

  DART_ASSERT(dynamic_cast<const dynamics::SoftMeshShape*>(shape));
  mSoftMeshShape = static_cast<const dynamics::SoftMeshShape*>(shape);
  mSoftBodyNode = mSoftMeshShape->getSoftBodyNode();
  DART_ASSERT(mSoftBodyNode);

  mSoftMesh = const_cast<aiMesh*>(mSoftMeshShape->getAssimpMesh());
  DART_ASSERT(mSoftMesh);

  auto collGeom = const_cast<dart::collision::fcl::CollisionGeometry*>(
      mFCLCollisionObject->collisionGeometry().get());
  DART_ASSERT(
      dynamic_cast<::fcl::BVHModel<dart::collision::fcl::OBBRSS>*>(collGeom));
  mSoftMeshBvhModel
      = static_cast<::fcl::BVHModel<dart::collision::fcl::OBBRSS>*>(collGeom);
}

//==============================================================================
void FCLCollisionObject::updateEngineData()
{
  using dart::dynamics::BodyNode;
  using dart::dynamics::Shape;
  using dart::dynamics::SoftMeshShape;

  bool softMeshChanged = false;

  // Update soft-body's vertices
  if (mSoftMeshShape) {
    DART_ASSERT(mSoftBodyNode);
    DART_ASSERT(mSoftMesh);
    DART_ASSERT(mSoftMeshBvhModel);

    auto bvhModel = mSoftMeshBvhModel;
    auto mesh = mSoftMesh;
    const auto& pointMasses = mSoftBodyNode->getPointMasses();
    DART_ASSERT(mesh->mNumVertices == pointMasses.size());

    const auto numFclVertices
        = static_cast<std::size_t>(bvhModel->num_vertices);
    if (numFclVertices == pointMasses.size()) {
      for (std::size_t i = 0; i < pointMasses.size(); ++i) {
        const Eigen::Vector3d& vertex = pointMasses[i]->getLocalPosition();
        const auto& previousVertex = bvhModel->vertices[i];
        if (previousVertex[0] != vertex[0] || previousVertex[1] != vertex[1]
            || previousVertex[2] != vertex[2]) {
          softMeshChanged = true;
          break;
        }
      }

      if (softMeshChanged) {
        bvhModel->beginUpdateModel();
        for (std::size_t i = 0; i < pointMasses.size(); ++i) {
          const Eigen::Vector3d& vertex = pointMasses[i]->getLocalPosition();
          mesh->mVertices[i].Set(vertex[0], vertex[1], vertex[2]);
          bvhModel->updateVertex(
              dart::collision::fcl::Vector3(vertex[0], vertex[1], vertex[2]));
        }
        bvhModel->endUpdateModel();
      }
    } else {
      DART_ASSERT(numFclVertices == mesh->mNumFaces * 3u);
      for (auto i = 0u, vertexIndex = 0u; i < mesh->mNumFaces; ++i) {
        const aiFace& face = mesh->mFaces[i];
        DART_ASSERT(face.mNumIndices == 3);

        for (auto j = 0u; j < 3u; ++j, ++vertexIndex) {
          const Eigen::Vector3d& vertex
              = pointMasses[face.mIndices[j]]->getLocalPosition();
          const auto& previousVertex = bvhModel->vertices[vertexIndex];
          if (previousVertex[0] != vertex[0] || previousVertex[1] != vertex[1]
              || previousVertex[2] != vertex[2]) {
            softMeshChanged = true;
            break;
          }
        }
        if (softMeshChanged)
          break;
      }

      if (softMeshChanged) {
        bvhModel->beginUpdateModel();
        for (std::size_t i = 0; i < pointMasses.size(); ++i) {
          const Eigen::Vector3d& vertex = pointMasses[i]->getLocalPosition();
          mesh->mVertices[i].Set(vertex[0], vertex[1], vertex[2]);
        }

        for (auto i = 0u; i < mesh->mNumFaces; ++i) {
          const aiFace& face = mesh->mFaces[i];
          DART_ASSERT(face.mNumIndices == 3);

          dart::collision::fcl::Vector3 vertices[3];
          for (auto j = 0u; j < 3; ++j) {
            const auto& vertex = mesh->mVertices[face.mIndices[j]];
            vertices[j]
                = dart::collision::fcl::Vector3(vertex.x, vertex.y, vertex.z);
          }
          bvhModel->updateTriangle(vertices[0], vertices[1], vertices[2]);
        }
        bvhModel->endUpdateModel();
      }
    }
  }

  const auto fclTransform = FCLTypes::convertTransform(getTransform());
  if (softMeshChanged) {
    mFCLCollisionObject->setTransform(fclTransform);
    mFCLCollisionObject->computeAABB();
    mCachedFCLTransform = fclTransform;
    mHasCachedFCLTransform = true;
    return;
  }

  const bool transformChanged
      = !mHasCachedFCLTransform
        || !transformsEqual(mCachedFCLTransform, fclTransform);
  if (transformChanged) {
    mFCLCollisionObject->setTransform(fclTransform);
    mFCLCollisionObject->computeAABB();
    mCachedFCLTransform = fclTransform;
    mHasCachedFCLTransform = true;
  }
}

} // namespace collision
} // namespace dart
