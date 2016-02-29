/*
 * Copyright (c) 2016, Georgia Tech Research Corporation
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

#include "dart/collision/bullet/BulletCollisionObjectData.h"

#include "dart/common/Console.h"
#include "dart/collision/bullet/BulletTypes.h"
#include "dart/collision/CollisionDetector.h"
#include "dart/collision/CollisionObject.h"
#include "dart/collision/bullet/BulletTypes.h"
#include "dart/dynamics/Shape.h"
#include "dart/dynamics/BoxShape.h"
#include "dart/dynamics/EllipsoidShape.h"
#include "dart/dynamics/CylinderShape.h"
#include "dart/dynamics/PlaneShape.h"
#include "dart/dynamics/Shape.h"
#include "dart/dynamics/MeshShape.h"
#include "dart/dynamics/SoftMeshShape.h"

namespace dart {
namespace collision {

//==============================================================================
BulletCollisionObjectUserData::BulletCollisionObjectUserData()
  : shape(nullptr),
    collisionObject(nullptr),
    collisionDetector(nullptr),
    group(nullptr)
{
  // Do nothing
}

//==============================================================================
BulletCollisionObjectData::BulletCollisionObjectData(
    CollisionDetector* collisionDetector,
    CollisionObject* parent,
    const dynamics::ShapePtr& shape)
  : CollisionObjectData(collisionDetector, parent)
{
  updateShape(shape);
}

//==============================================================================
void BulletCollisionObjectData::updateTransform(const Eigen::Isometry3d& tf)
{
  mBulletCollisionObject->setWorldTransform(convertTransform(tf));
}

//==============================================================================
void BulletCollisionObjectData::updateShape(const dynamics::ShapePtr& shape)
{
  mBulletTriangleMesh.release();
  mBulletGImpactMeshShape.release();
  mBulletCollisionShape.release();
  mBulletCollisionObject.release();

  using dynamics::Shape;
  using dynamics::BoxShape;
  using dynamics::EllipsoidShape;
  using dynamics::CylinderShape;
  using dynamics::PlaneShape;
  using dynamics::MeshShape;
  using dynamics::SoftMeshShape;

  switch (shape->getShapeType())
  {
    case Shape::BOX:
    {
      assert(dynamic_cast<BoxShape*>(shape.get()));

      const auto box = static_cast<const BoxShape*>(shape.get());
      const Eigen::Vector3d& size = box->getSize();

      mBulletCollisionShape.reset(new btBoxShape(convertVector3(size*0.5)));

      break;
    }
    case Shape::ELLIPSOID:
    {
      assert(dynamic_cast<EllipsoidShape*>(shape.get()));

      const auto ellipsoid = static_cast<EllipsoidShape*>(shape.get());
      const Eigen::Vector3d& size = ellipsoid->getSize();

      if (ellipsoid->isSphere())
      {
        mBulletCollisionShape.reset(new btSphereShape(size[0] * 0.5));
      }
      else
      {
        // TODO(JS): Add mesh for ellipsoid
      }

      break;
    }
    case Shape::CYLINDER:
    {
      assert(dynamic_cast<CylinderShape*>(shape.get()));

      const auto cylinder = static_cast<const CylinderShape*>(shape.get());
      const auto radius = cylinder->getRadius();
      const auto height = cylinder->getHeight();
      const auto size = btVector3(radius, radius, height * 0.5);

      mBulletCollisionShape.reset(new btCylinderShapeZ(size));

      break;
    }
    case Shape::PLANE:
    {
      assert(dynamic_cast<PlaneShape*>(shape.get()));

      const auto plane = static_cast<PlaneShape*>(shape.get());
      const Eigen::Vector3d normal = plane->getNormal();
      const double offset = plane->getOffset();

      mBulletCollisionShape.reset(new btStaticPlaneShape(
            convertVector3(normal), offset));

      break;
    }
    case Shape::MESH:
    {
      assert(dynamic_cast<MeshShape*>(shape.get()));

      const auto shapeMesh = static_cast<MeshShape*>(shape.get());
      const auto scale = shapeMesh->getScale();
      const auto mesh = shapeMesh->getMesh();

      mBulletCollisionShape.reset(createMesh(scale, mesh));

      break;
    }
    case Shape::SOFT_MESH:
    {
      assert(dynamic_cast<SoftMeshShape*>(shape.get()));

      const auto softMeshShape = static_cast<SoftMeshShape*>(shape.get());
      const auto mesh = softMeshShape->getAssimpMesh();

      mBulletCollisionShape.reset(createSoftMesh(mesh));

      break;
    }
    default:
    {
      dterr << "[BulletCollisionObjectData::init] "
            << "Attempting to create unsupported shape type '"
            << shape->getShapeType() << "'.\n";

      mBulletCollisionShape.reset(new btSphereShape(0.1));

      break;
    }
  }

  mBulletCollisionObjectUserData.reset(new BulletCollisionObjectUserData());
  mBulletCollisionObjectUserData->shape = shape;
  mBulletCollisionObjectUserData->collisionObject = mParent;
  mBulletCollisionObjectUserData->collisionDetector = mCollisionDetector;
  mBulletCollisionObjectUserData->group = nullptr;

  mBulletCollisionObject.reset(new btCollisionObject());
  mBulletCollisionObject->setCollisionShape(mBulletCollisionShape.get());
  mBulletCollisionObject->setUserPointer(mBulletCollisionObjectUserData.get());
}

//==============================================================================
void BulletCollisionObjectData::update()
{
  updateTransform(mParent->getTransform());
}

//==============================================================================
btCollisionObject* BulletCollisionObjectData::getBulletCollisionObject() const
{
  return mBulletCollisionObject.get();
}

//==============================================================================
btGImpactMeshShape* BulletCollisionObjectData::createMesh(
    const Eigen::Vector3d& scale, const aiScene* mesh)
{
  mBulletTriangleMesh.reset(new btTriangleMesh());

  for (unsigned int i = 0; i < mesh->mNumMeshes; i++)
  {
    for (unsigned int j = 0; j < mesh->mMeshes[i]->mNumFaces; j++)
    {
      btVector3 vertices[3];
      for (unsigned int k = 0; k < 3; k++)
      {
        const aiVector3D& vertex = mesh->mMeshes[i]->mVertices[
                                   mesh->mMeshes[i]->mFaces[j].mIndices[k]];
        vertices[k] = btVector3(vertex.x * scale[0],
                                vertex.y * scale[1],
                                vertex.z * scale[2]);
      }
      mBulletTriangleMesh->addTriangle(vertices[0], vertices[1], vertices[2]);
    }
  }

  btGImpactMeshShape* gimpactMeshShape
      = new btGImpactMeshShape(mBulletTriangleMesh.get());
  gimpactMeshShape->updateBound();

  return gimpactMeshShape;
}

//==============================================================================
btGImpactMeshShape* BulletCollisionObjectData::createSoftMesh(
    const aiMesh* mesh)
{
  mBulletTriangleMesh.reset(new btTriangleMesh());

  for (auto i = 0u; i < mesh->mNumFaces; ++i)
  {
    btVector3 vertices[3];
    for (auto j = 0u; j < 3; ++j)
    {
      const aiVector3D& vertex = mesh->mVertices[mesh->mFaces[i].mIndices[j]];
      vertices[j] = btVector3(vertex.x, vertex.y, vertex.z);
    }
    mBulletTriangleMesh->addTriangle(vertices[0], vertices[1], vertices[2]);
  }

  btGImpactMeshShape* gimpactMeshShape
      = new btGImpactMeshShape(mBulletTriangleMesh.get());
  gimpactMeshShape->updateBound();

  return gimpactMeshShape;
}

}  // namespace collision
}  // namespace dart
