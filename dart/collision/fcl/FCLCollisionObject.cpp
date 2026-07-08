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
}

//==============================================================================
void FCLCollisionObject::setFCLCollisionGeometry(
    const std::shared_ptr<dart::collision::fcl::CollisionGeometry>& fclCollGeom)
{
  auto mutableObject
      = static_cast<MutableFCLCollisionObject*>(mFCLCollisionObject.get());
  DART_ASSERT(mutableObject);

  mutableObject->setCollisionGeometry(fclCollGeom);
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
    DART_ASSERT(dynamic_cast<const SoftMeshShape*>(shape));
    auto softMeshShape = static_cast<const SoftMeshShape*>(shape);

    auto collGeom = const_cast<dart::collision::fcl::CollisionGeometry*>(
        mFCLCollisionObject->collisionGeometry().get());
    DART_ASSERT(
        dynamic_cast<::fcl::BVHModel<dart::collision::fcl::OBBRSS>*>(collGeom));
    auto bvhModel
        = static_cast<::fcl::BVHModel<dart::collision::fcl::OBBRSS>*>(collGeom);

    const auto* softBodyNode = softMeshShape->getSoftBodyNode();
    DART_ASSERT(softBodyNode);

    auto* mesh = const_cast<aiMesh*>(softMeshShape->getAssimpMesh());
    DART_ASSERT(mesh);

    const auto& pointMasses = softBodyNode->getPointMasses();
    DART_ASSERT(mesh->mNumVertices == pointMasses.size());

    const auto& pointStates = softBodyNode->mAspectState.mPointStates;
    const auto& pointProperties = softBodyNode->mAspectProperties.mPointProps;
    const bool useContiguousPointData
        = pointStates.size() == pointMasses.size()
          && pointProperties.size() == pointMasses.size();
    const auto readLocalVertex
        = [&](std::size_t index, double& x, double& y, double& z) {
            if (useContiguousPointData) {
              const auto& position = pointStates[index].mPositions;
              const auto& restingPosition = pointProperties[index].mX0;
              x = position[0] + restingPosition[0];
              y = position[1] + restingPosition[1];
              z = position[2] + restingPosition[2];
              return;
            }

            const Eigen::Vector3d& vertex
                = pointMasses[index]->getLocalPosition();
            x = vertex[0];
            y = vertex[1];
            z = vertex[2];
          };

    bool softMeshChanged = false;
    const auto numFclVertices
        = static_cast<std::size_t>(bvhModel->num_vertices);
    if (numFclVertices == pointMasses.size()) {
      for (std::size_t i = 0; i < pointMasses.size(); ++i) {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        readLocalVertex(i, x, y, z);
        const auto& previousVertex = bvhModel->vertices[i];
        if (previousVertex[0] != x || previousVertex[1] != y
            || previousVertex[2] != z) {
          softMeshChanged = true;
          break;
        }
      }

      if (softMeshChanged) {
        // Soft-body collision in DART is discrete. Replace the current
        // geometry with the same topology instead of using FCL's dynamic update
        // path, which refits swept previous/current vertex bounds.
        bvhModel->beginReplaceModel();
        for (std::size_t i = 0; i < pointMasses.size(); ++i) {
          double x = 0.0;
          double y = 0.0;
          double z = 0.0;
          readLocalVertex(i, x, y, z);
          bvhModel->replaceVertex(dart::collision::fcl::Vector3(x, y, z));
        }
        bvhModel->endReplaceModel();
      }
    } else {
      DART_ASSERT(numFclVertices == mesh->mNumFaces * 3u);
      for (auto i = 0u, vertexIndex = 0u; i < mesh->mNumFaces; ++i) {
        const aiFace& face = mesh->mFaces[i];
        DART_ASSERT(face.mNumIndices == 3);

        for (auto j = 0u; j < 3u; ++j, ++vertexIndex) {
          double x = 0.0;
          double y = 0.0;
          double z = 0.0;
          readLocalVertex(face.mIndices[j], x, y, z);
          const auto& previousVertex = bvhModel->vertices[vertexIndex];
          if (previousVertex[0] != x || previousVertex[1] != y
              || previousVertex[2] != z) {
            softMeshChanged = true;
            break;
          }
        }
        if (softMeshChanged)
          break;
      }

      if (softMeshChanged) {
        // Soft-body collision in DART is discrete. Replace the current
        // geometry with the same topology instead of using FCL's dynamic update
        // path, which refits swept previous/current vertex bounds.
        bvhModel->beginReplaceModel();
        for (std::size_t i = 0; i < pointMasses.size(); ++i) {
          double x = 0.0;
          double y = 0.0;
          double z = 0.0;
          readLocalVertex(i, x, y, z);
          mesh->mVertices[i].Set(x, y, z);
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
          bvhModel->replaceTriangle(vertices[0], vertices[1], vertices[2]);
        }
        bvhModel->endReplaceModel();
      }
    }
  }

  mFCLCollisionObject->setTransform(FCLTypes::convertTransform(getTransform()));
  mFCLCollisionObject->computeAABB();
}

} // namespace collision
} // namespace dart
