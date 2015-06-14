/*
 * Copyright (c) 2013-2015, Georgia Tech Research Corporation
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

#include "dart/collision/fcl/FCLCollisionNode.h"

#include <assimp/scene.h>
#include <fcl/shape/geometric_shapes.h>

#include "dart/common/Console.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/BoxShape.h"
#include "dart/dynamics/EllipsoidShape.h"
#include "dart/dynamics/CylinderShape.h"
#include "dart/dynamics/PlaneShape.h"
#include "dart/dynamics/Shape.h"
#include "dart/dynamics/MeshShape.h"
#include "dart/dynamics/SoftMeshShape.h"
#include "dart/collision/fcl/FCLTypes.h"

namespace dart {
namespace collision {

//==============================================================================
template<class BV>
fcl::BVHModel<BV>* createMesh(float _scaleX, float _scaleY, float _scaleZ,
                              const aiScene* _mesh)
{
  // Create FCL mesh from Assimp mesh

  assert(_mesh);
  fcl::BVHModel<BV>* model = new fcl::BVHModel<BV>;
  model->beginModel();
  for (size_t i = 0; i < _mesh->mNumMeshes; i++)
  {
    for (size_t j = 0; j < _mesh->mMeshes[i]->mNumFaces; j++)
    {
      fcl::Vec3f vertices[3];
      for (size_t k = 0; k < 3; k++)
      {
        const aiVector3D& vertex
            = _mesh->mMeshes[i]->mVertices[
              _mesh->mMeshes[i]->mFaces[j].mIndices[k]];
        vertices[k] = fcl::Vec3f(vertex.x * _scaleX,
                                 vertex.y * _scaleY,
                                 vertex.z * _scaleZ);
      }
      model->addTriangle(vertices[0], vertices[1], vertices[2]);
    }
  }
  model->endModel();
  return model;
}

//==============================================================================
template<class BV>
fcl::BVHModel<BV>* createSoftMesh(const aiMesh* _mesh,
                                  const fcl::Transform3f& _transform)
{
  // Create FCL mesh from Assimp mesh

  assert(_mesh);
  fcl::BVHModel<BV>* model = new fcl::BVHModel<BV>;
  model->beginModel();

  for (size_t i = 0; i < _mesh->mNumFaces; i++)
  {
    fcl::Vec3f vertices[3];
    for (size_t j = 0; j < 3; j++)
    {
      const aiVector3D& vertex
          = _mesh->mVertices[_mesh->mFaces[i].mIndices[j]];
      vertices[j] = fcl::Vec3f(vertex.x, vertex.y, vertex.z);
      vertices[j] = _transform.transform(vertices[j]);
    }
    model->addTriangle(vertices[0], vertices[1], vertices[2]);
  }

  model->endModel();
  return model;
}

//==============================================================================
FCLUserData::FCLUserData(FCLCollisionNode* _fclCollNode,
                         dynamics::BodyNode* _bodyNode,
                         dynamics::Shape* _shape)
  : fclCollNode(_fclCollNode),
    bodyNode(_bodyNode),
    shape(_shape)
{
  // Do nothing
}

//==============================================================================
FCLCollisionNode::FCLCollisionNode(dynamics::BodyNode* _bodyNode)
  : CollisionNode(_bodyNode)
{
  using dynamics::Shape;
  using dynamics::BoxShape;
  using dynamics::EllipsoidShape;
  using dynamics::CylinderShape;
  using dynamics::PlaneShape;
  using dynamics::MeshShape;
  using dynamics::SoftMeshShape;

  for (size_t i = 0; i < _bodyNode->getNumCollisionShapes(); i++)
  {
    dynamics::ShapePtr shape = _bodyNode->getCollisionShape(i);

    boost::shared_ptr<fcl::CollisionGeometry> fclCollGeom;

    switch (shape->getShapeType())
    {
      case Shape::BOX:
      {
        assert(dynamic_cast<BoxShape*>(shape.get()));
        const BoxShape* box = static_cast<const BoxShape*>(shape.get());
        const Eigen::Vector3d& size = box->getSize();
        fclCollGeom.reset(new fcl::Box(size[0], size[1], size[2]));

        break;
      }
      case Shape::ELLIPSOID:
      {
        assert(dynamic_cast<EllipsoidShape*>(shape.get()));
        EllipsoidShape* ellipsoid = static_cast<EllipsoidShape*>(shape.get());

        const Eigen::Vector3d& size = ellipsoid->getSize();

        if (ellipsoid->isSphere())
        {
          fclCollGeom.reset(new fcl::Sphere(size[0] * 0.5));
        }
        else
        {
          fclCollGeom.reset(
                new fcl::Ellipsoid(FCLTypes::convertVector3(size * 0.5)));
        }

        break;
      }
      case Shape::CYLINDER:
      {
        assert(dynamic_cast<CylinderShape*>(shape.get()));
        CylinderShape* cylinder = static_cast<CylinderShape*>(shape.get());
        const double radius = cylinder->getRadius();
        const double height = cylinder->getHeight();
        fclCollGeom.reset(new fcl::Cylinder(radius, height));

        break;
      }
      case Shape::PLANE:
      {
        assert(dynamic_cast<PlaneShape*>(shape.get()));
        dynamics::PlaneShape* plane = static_cast<PlaneShape*>(shape.get());
        const Eigen::Vector3d normal = plane->getNormal();
        const double          offset = plane->getOffset();
        fclCollGeom.reset(
              new fcl::Halfspace(FCLTypes::convertVector3(normal), offset));

        break;
      }
      case Shape::MESH:
      {
        assert(dynamic_cast<MeshShape*>(shape));
        MeshShape* shapeMesh = static_cast<MeshShape*>(shape.get());
        fclCollGeom.reset(
            createMesh<fcl::OBBRSS>(shapeMesh->getScale()[0],
                                    shapeMesh->getScale()[1],
                                    shapeMesh->getScale()[2],
                                    shapeMesh->getMesh()));

        break;
      }
      case Shape::SOFT_MESH:
      {
        assert(dynamic_cast<SoftMeshShape*>(shape));
        SoftMeshShape* softMeshShape = static_cast<SoftMeshShape*>(shape.get());
        fclCollGeom.reset(
            createSoftMesh<fcl::OBBRSS>(
                softMeshShape->getAssimpMesh(),
                FCLTypes::convertTransform(Eigen::Isometry3d::Identity())));

        break;
      }
      default:
      {
        dterr << "[FCLCollisionNode::FCLCollisionNode] Attempting to create "
              << "unsupported shape type '" << shape->getShapeType() << "' of '"
              << _bodyNode->getName() << "' body node." <<  std::endl;
        continue;
      }
    }

    assert(nullptr != fclCollGeom);
    fcl::CollisionObject* fclCollObj
        = new fcl::CollisionObject(fclCollGeom, getFCLTransform(i));
    fclCollObj->setUserData(new FCLUserData(this, _bodyNode, shape.get()));
    mCollisionObjects.push_back(fclCollObj);
  }
}

//==============================================================================
FCLCollisionNode::~FCLCollisionNode()
{
  for (const auto& collObj : mCollisionObjects)
  {
    assert(collObj);

    delete static_cast<FCLUserData*>(collObj->getUserData());
    delete collObj;
  }
}

//==============================================================================
size_t FCLCollisionNode::getNumCollisionObjects() const
{
  return mCollisionObjects.size();
}

//==============================================================================
fcl::CollisionObject* FCLCollisionNode::getCollisionObject(size_t _idx) const
{
  assert(_idx < mCollisionObjects.size());
  return mCollisionObjects[_idx];
}

//==============================================================================
fcl::Transform3f FCLCollisionNode::getFCLTransform(size_t _idx) const
{
  Eigen::Isometry3d worldTrans
      = mBodyNode->getTransform()
        * mBodyNode->getCollisionShape(_idx)->getLocalTransform();

  return fcl::Transform3f(
        fcl::Matrix3f(worldTrans(0, 0), worldTrans(0, 1), worldTrans(0, 2),
                      worldTrans(1, 0), worldTrans(1, 1), worldTrans(1, 2),
                      worldTrans(2, 0), worldTrans(2, 1), worldTrans(2, 2)),
        fcl::Vec3f(worldTrans(0, 3), worldTrans(1, 3), worldTrans(2, 3)));
}

//==============================================================================
void FCLCollisionNode::updateFCLCollisionObjects()
{
  using dart::dynamics::BodyNode;
  using dart::dynamics::Shape;
  using dart::dynamics::SoftMeshShape;

  for (auto& fclCollObj : mCollisionObjects)
  {
    FCLUserData* userData
        = static_cast<FCLUserData*>(fclCollObj->getUserData());

    BodyNode* bodyNode = userData->bodyNode;
    Shape*    shape    = userData->shape;

    // Update shape's transform
    const Eigen::Isometry3d W = bodyNode->getWorldTransform()
                                * shape->getLocalTransform();
    fclCollObj->setTransform(FCLTypes::convertTransform(W));
    fclCollObj->computeAABB();

    // Update soft-body's vertices
    if (shape->getShapeType() == Shape::SOFT_MESH)
    {
      assert(dynamic_cast<SoftMeshShape*>(shape));
      SoftMeshShape* softMeshShape = static_cast<SoftMeshShape*>(shape);

      const aiMesh* mesh = softMeshShape->getAssimpMesh();
      softMeshShape->update();

      const boost::shared_ptr<fcl::CollisionGeometry> collGeom
          = boost::const_pointer_cast<fcl::CollisionGeometry>(
              fclCollObj->collisionGeometry());
      boost::shared_ptr<fcl::BVHModel<fcl::OBBRSS>> bvhModel
          = boost::dynamic_pointer_cast<fcl::BVHModel<fcl::OBBRSS>>(collGeom);

      bvhModel->beginUpdateModel();

      for (unsigned int j = 0; j < mesh->mNumFaces; j++)
      {
        fcl::Vec3f vertices[3];
        for (unsigned int k = 0; k < 3; k++)
        {
          const aiVector3D& vertex
              = mesh->mVertices[mesh->mFaces[j].mIndices[k]];
          vertices[k] = fcl::Vec3f(vertex.x, vertex.y, vertex.z);
        }
        bvhModel->updateTriangle(vertices[0], vertices[1], vertices[2]);
      }

      bvhModel->endUpdateModel();
    }
  }
}

}  // namespace collision
}  // namespace dart
