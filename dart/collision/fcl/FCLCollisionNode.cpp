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

#define FCL_VERSION_AT_LEAST(x,y,z) \
  (FCL_MAJOR_VERSION > x || (FCL_MAJOR_VERSION >= x && \
  (FCL_MINOR_VERSION > y || (FCL_MINOR_VERSION >= y && \
  FCL_PATCH_VERSION >= z))))

namespace kido {
namespace collision {

//==============================================================================
template<class BV>
fcl::BVHModel<BV>* createEllipsoid(float _sizeX, float _sizeY, float _sizeZ)
{
  float v[59][3] =
  {
    {0, 0, 0},
    {0.135299, -0.461940, -0.135299},
    {0.000000, -0.461940, -0.191342},
    {-0.135299, -0.461940, -0.135299},
    {-0.191342, -0.461940, 0.000000},
    {-0.135299, -0.461940, 0.135299},
    {0.000000, -0.461940, 0.191342},
    {0.135299, -0.461940, 0.135299},
    {0.191342, -0.461940, 0.000000},
    {0.250000, -0.353553, -0.250000},
    {0.000000, -0.353553, -0.353553},
    {-0.250000, -0.353553, -0.250000},
    {-0.353553, -0.353553, 0.000000},
    {-0.250000, -0.353553, 0.250000},
    {0.000000, -0.353553, 0.353553},
    {0.250000, -0.353553, 0.250000},
    {0.353553, -0.353553, 0.000000},
    {0.326641, -0.191342, -0.326641},
    {0.000000, -0.191342, -0.461940},
    {-0.326641, -0.191342, -0.326641},
    {-0.461940, -0.191342, 0.000000},
    {-0.326641, -0.191342, 0.326641},
    {0.000000, -0.191342, 0.461940},
    {0.326641, -0.191342, 0.326641},
    {0.461940, -0.191342, 0.000000},
    {0.353553, 0.000000, -0.353553},
    {0.000000, 0.000000, -0.500000},
    {-0.353553, 0.000000, -0.353553},
    {-0.500000, 0.000000, 0.000000},
    {-0.353553, 0.000000, 0.353553},
    {0.000000, 0.000000, 0.500000},
    {0.353553, 0.000000, 0.353553},
    {0.500000, 0.000000, 0.000000},
    {0.326641, 0.191342, -0.326641},
    {0.000000, 0.191342, -0.461940},
    {-0.326641, 0.191342, -0.326641},
    {-0.461940, 0.191342, 0.000000},
    {-0.326641, 0.191342, 0.326641},
    {0.000000, 0.191342, 0.461940},
    {0.326641, 0.191342, 0.326641},
    {0.461940, 0.191342, 0.000000},
    {0.250000, 0.353553, -0.250000},
    {0.000000, 0.353553, -0.353553},
    {-0.250000, 0.353553, -0.250000},
    {-0.353553, 0.353553, 0.000000},
    {-0.250000, 0.353553, 0.250000},
    {0.000000, 0.353553, 0.353553},
    {0.250000, 0.353553, 0.250000},
    {0.353553, 0.353553, 0.000000},
    {0.135299, 0.461940, -0.135299},
    {0.000000, 0.461940, -0.191342},
    {-0.135299, 0.461940, -0.135299},
    {-0.191342, 0.461940, 0.000000},
    {-0.135299, 0.461940, 0.135299},
    {0.000000, 0.461940, 0.191342},
    {0.135299, 0.461940, 0.135299},
    {0.191342, 0.461940, 0.000000},
    {0.000000, -0.500000, 0.000000},
    {0.000000, 0.500000, 0.000000}
  };

  int f[112][3] =
  {
    {1, 2, 9},
    {9, 2, 10},
    {2, 3, 10},
    {10, 3, 11},
    {3, 4, 11},
    {11, 4, 12},
    {4, 5, 12},
    {12, 5, 13},
    {5, 6, 13},
    {13, 6, 14},
    {6, 7, 14},
    {14, 7, 15},
    {7, 8, 15},
    {15, 8, 16},
    {8, 1, 16},
    {16, 1, 9},
    {9, 10, 17},
    {17, 10, 18},
    {10, 11, 18},
    {18, 11, 19},
    {11, 12, 19},
    {19, 12, 20},
    {12, 13, 20},
    {20, 13, 21},
    {13, 14, 21},
    {21, 14, 22},
    {14, 15, 22},
    {22, 15, 23},
    {15, 16, 23},
    {23, 16, 24},
    {16, 9, 24},
    {24, 9, 17},
    {17, 18, 25},
    {25, 18, 26},
    {18, 19, 26},
    {26, 19, 27},
    {19, 20, 27},
    {27, 20, 28},
    {20, 21, 28},
    {28, 21, 29},
    {21, 22, 29},
    {29, 22, 30},
    {22, 23, 30},
    {30, 23, 31},
    {23, 24, 31},
    {31, 24, 32},
    {24, 17, 32},
    {32, 17, 25},
    {25, 26, 33},
    {33, 26, 34},
    {26, 27, 34},
    {34, 27, 35},
    {27, 28, 35},
    {35, 28, 36},
    {28, 29, 36},
    {36, 29, 37},
    {29, 30, 37},
    {37, 30, 38},
    {30, 31, 38},
    {38, 31, 39},
    {31, 32, 39},
    {39, 32, 40},
    {32, 25, 40},
    {40, 25, 33},
    {33, 34, 41},
    {41, 34, 42},
    {34, 35, 42},
    {42, 35, 43},
    {35, 36, 43},
    {43, 36, 44},
    {36, 37, 44},
    {44, 37, 45},
    {37, 38, 45},
    {45, 38, 46},
    {38, 39, 46},
    {46, 39, 47},
    {39, 40, 47},
    {47, 40, 48},
    {40, 33, 48},
    {48, 33, 41},
    {41, 42, 49},
    {49, 42, 50},
    {42, 43, 50},
    {50, 43, 51},
    {43, 44, 51},
    {51, 44, 52},
    {44, 45, 52},
    {52, 45, 53},
    {45, 46, 53},
    {53, 46, 54},
    {46, 47, 54},
    {54, 47, 55},
    {47, 48, 55},
    {55, 48, 56},
    {48, 41, 56},
    {56, 41, 49},
    {2, 1, 57},
    {3, 2, 57},
    {4, 3, 57},
    {5, 4, 57},
    {6, 5, 57},
    {7, 6, 57},
    {8, 7, 57},
    {1, 8, 57},
    {49, 50, 58},
    {50, 51, 58},
    {51, 52, 58},
    {52, 53, 58},
    {53, 54, 58},
    {54, 55, 58},
    {55, 56, 58},
    {56, 49, 58}
  };

  fcl::BVHModel<BV>* model = new fcl::BVHModel<BV>;
  fcl::Vec3f p1, p2, p3;
  model->beginModel();

  for (int i = 0; i < 112; i++)
  {
    p1 = fcl::Vec3f(v[f[i][0]][0] * _sizeX,
        v[f[i][0]][1] * _sizeY,
        v[f[i][0]][2] * _sizeZ);
    p2 = fcl::Vec3f(v[f[i][1]][0] * _sizeX,
        v[f[i][1]][1] * _sizeY,
        v[f[i][1]][2] * _sizeZ);
    p3 = fcl::Vec3f(v[f[i][2]][0] * _sizeX,
        v[f[i][2]][1] * _sizeY,
        v[f[i][2]][2] * _sizeZ);

    model->addTriangle(p1, p2, p3);
  }

  model->endModel();

  return model;
}

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
#ifdef FCL_DART5
          fclCollGeom.reset(
                new fcl::Ellipsoid(FCLTypes::convertVector3(size * 0.5)));
#else
          fclCollGeom.reset(
                createEllipsoid<fcl::OBBRSS>(ellipsoid->getSize()[0],
                                             ellipsoid->getSize()[1],
                                             ellipsoid->getSize()[2]));
#endif
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
        assert(dynamic_cast<MeshShape*>(shape.get()));
        MeshShape* shapeMesh = static_cast<MeshShape*>(shape.get());
        fclCollGeom.reset(
            createMesh<fcl::OBBRSS>(shapeMesh->getScale()[0],
                                    shapeMesh->getScale()[1],
                                    shapeMesh->getScale()[2],
                                    shapeMesh->getMesh()));

        break;
      }
#if 0
      case Shape::SOFT_MESH:
      {
        assert(dynamic_cast<SoftMeshShape*>(shape.get()));
        SoftMeshShape* softMeshShape = static_cast<SoftMeshShape*>(shape.get());
        fclCollGeom.reset(
            createSoftMesh<fcl::OBBRSS>(
                softMeshShape->getAssimpMesh(),
                FCLTypes::convertTransform(Eigen::Isometry3d::Identity())));

        break;
      }
#endif
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
  using kido::dynamics::BodyNode;
  using kido::dynamics::Shape;
  using kido::dynamics::SoftMeshShape;

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
#if FCL_VERSION_AT_LEAST(0,3,0)
      fcl::CollisionGeometry* collGeom
          = const_cast<fcl::CollisionGeometry*>(
              fclCollObj->collisionGeometry().get());
#else
      fcl::CollisionGeometry* collGeom
          = const_cast<fcl::CollisionGeometry*>(
              fclCollObj->getCollisionGeometry());
#endif
      assert(nullptr != dynamic_cast<fcl::BVHModel<fcl::OBBRSS>*>(collGeom));
      fcl::BVHModel<fcl::OBBRSS>* bvhModel
          = static_cast<fcl::BVHModel<fcl::OBBRSS>*>(collGeom);

      bvhModel->beginUpdateModel();
      for (size_t j = 0; j < mesh->mNumFaces; j++)
      {
        fcl::Vec3f vertices[3];
        for (size_t k = 0; k < 3; k++)
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
}  // namespace kido
