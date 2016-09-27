/*
 * Copyright (c) 2011-2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2011-2016, Humanoid Lab, Georgia Tech Research Corporation
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

#include "dart/collision/fcl/FCLCollisionDetector.hpp"

#include <assimp/scene.h>

#include <fcl/collision.h>
#include <fcl/collision_object.h>
#include <fcl/collision_data.h>
#include <fcl/distance.h>
#include <fcl/BVH/BVH_model.h>
#include <fcl/broadphase/broadphase.h>
#include <fcl/shape/geometric_shapes.h>
#include <fcl/shape/geometric_shape_to_BVH_model.h>

#include "dart/common/Console.hpp"
#include "dart/collision/CollisionObject.hpp"
#include "dart/collision/CollisionFilter.hpp"
#include "dart/collision/DistanceFilter.hpp"
#include "dart/collision/fcl/FCLTypes.hpp"
#include "dart/collision/fcl/FCLCollisionObject.hpp"
#include "dart/collision/fcl/FCLCollisionGroup.hpp"
#include "dart/collision/fcl/tri_tri_intersection_test.hpp"
#include "dart/dynamics/ShapeFrame.hpp"
#include "dart/dynamics/Shape.hpp"
#include "dart/dynamics/SphereShape.hpp"
#include "dart/dynamics/BoxShape.hpp"
#include "dart/dynamics/EllipsoidShape.hpp"
#include "dart/dynamics/CylinderShape.hpp"
#include "dart/dynamics/PlaneShape.hpp"
#include "dart/dynamics/MeshShape.hpp"
#include "dart/dynamics/SoftMeshShape.hpp"

namespace dart {
namespace collision {

namespace {

bool collisionCallback(
    fcl::CollisionObject* o1, fcl::CollisionObject* o2, void* cdata);

bool distanceCallback(
    fcl::CollisionObject* o1,
    fcl::CollisionObject* o2,
    void* cdata,
    fcl::FCL_REAL& dist);

void postProcessFCL(
    const fcl::CollisionResult& fclResult,
    fcl::CollisionObject* o1,
    fcl::CollisionObject* o2,
    const CollisionOption& option,
    CollisionResult& result);

void postProcessDART(
    const fcl::CollisionResult& fclResult,
    fcl::CollisionObject* o1,
    fcl::CollisionObject* o2,
    const CollisionOption& option,
    CollisionResult& result);

void interpreteDistanceResult(
    const fcl::DistanceResult& fclResult,
    fcl::CollisionObject* o1,
    fcl::CollisionObject* o2,
    const DistanceOption& option,
    DistanceResult& result);

int evalContactPosition(const fcl::Contact& fclContact,
    const fcl::BVHModel<fcl::OBBRSS>* mesh1,
    const fcl::BVHModel<fcl::OBBRSS>* mesh2,
    const fcl::Transform3f& transform1,
    const fcl::Transform3f& transform2,
    Eigen::Vector3d& contactPosition1, Eigen::Vector3d& contactPosition2);

Eigen::Vector3d getDiff(const Contact& contact1, const Contact& contact2);

fcl::Vec3f getDiff(const fcl::Contact& contact1, const fcl::Contact& contact2);

bool isColinear(
    const Contact& contact1,
    const Contact& contact2,
    const Contact& contact3,
    double tol);

bool isColinear(
    const fcl::Contact& contact1,
    const fcl::Contact& contact2,
    const fcl::Contact& contact3,
    double tol);

template <typename T>
bool isColinear(const T& pos1, const T& pos2, const T& pos3, double tol);

int FFtest(
    const fcl::Vec3f& r1, const fcl::Vec3f& r2, const fcl::Vec3f& r3,
    const fcl::Vec3f& R1, const fcl::Vec3f& R2, const fcl::Vec3f& R3,
    fcl::Vec3f* res1, fcl::Vec3f* res2);

double triArea(fcl::Vec3f& p1, fcl::Vec3f& p2, fcl::Vec3f& p3);

void convertOption(
    const CollisionOption& option, fcl::CollisionRequest& request);

void convertOption(
    const DistanceOption& option, fcl::DistanceRequest& request);

Contact convertContact(
    const fcl::Contact& fclContact,
    fcl::CollisionObject* o1,
    fcl::CollisionObject* o2,
    const CollisionOption& option);

/// Collision data stores the collision request and the result given by
/// collision algorithm.
struct FCLCollisionCallbackData
{
  /// FCL collision request
  fcl::CollisionRequest fclRequest;

  /// FCL collision result
  fcl::CollisionResult fclResult;

  /// Collision option of DART
  const CollisionOption& option;

  /// Collision result of DART
  CollisionResult* result;

  /// True if at least one contact is found. This flag is used only when
  /// mResult is nullptr; otherwise the actual collision result is in mResult.
  bool foundCollision;

  FCLCollisionDetector::PrimitiveShape primitiveShapeType;

  FCLCollisionDetector::ContactPointComputationMethod
  contactPointComputationMethod;

  /// Whether the collision iteration can stop
  bool done;

  bool isCollision() const
  {
    if (result)
      return result->isCollision();
    else
      return foundCollision;
  }

  /// Constructor
  FCLCollisionCallbackData(
      const CollisionOption& option,
      CollisionResult* result,
      FCLCollisionDetector::PrimitiveShape type
          = FCLCollisionDetector::MESH,
      FCLCollisionDetector::ContactPointComputationMethod method
          = FCLCollisionDetector::DART)
    : option(option),
      result(result),
      foundCollision(false),
      primitiveShapeType(type),
      contactPointComputationMethod(method),
      done(false)
  {
    convertOption(option, fclRequest);

    fclRequest.num_max_contacts = std::max(static_cast<std::size_t>(100u),
                                            option.maxNumContacts);
    // Since some contact points can be filtered out in the post process, we ask
    // more than the demend. 100 is randomly picked.
  }
};

struct FCLDistanceCallbackData
{
  /// FCL distance request
  fcl::DistanceRequest fclRequest;

  /// FCL distance result
  fcl::DistanceResult fclResult;

  /// Distance option of DART
  const DistanceOption& option;

  /// Minimum distance identical to DistanceResult::minDistnace if result is not
  /// nullptr.
  double unclampedMinDistance;

  /// Distance result of DART
  DistanceResult* result;

  /// @brief Whether the distance iteration can stop
  bool done;

  FCLDistanceCallbackData(
      const DistanceOption& option, DistanceResult* result)
    : option(option),
      result(result),
      done(false)
  {
    convertOption(option, fclRequest);
  }
};

//==============================================================================
// Create a cube mesh for collision detection
template<class BV>
fcl::BVHModel<BV>* createCube(float _sizeX, float _sizeY, float _sizeZ)
{
  int faces[6][4] =
  {
    {0, 1, 2, 3},
    {3, 2, 6, 7},
    {7, 6, 5, 4},
    {4, 5, 1, 0},
    {5, 6, 2, 1},
    {7, 4, 0, 3}
  };
  float v[8][3];

  v[0][0] = v[1][0] = v[2][0] = v[3][0] = -_sizeX / 2;
  v[4][0] = v[5][0] = v[6][0] = v[7][0] = _sizeX / 2;
  v[0][1] = v[1][1] = v[4][1] = v[5][1] = -_sizeY / 2;
  v[2][1] = v[3][1] = v[6][1] = v[7][1] = _sizeY / 2;
  v[0][2] = v[3][2] = v[4][2] = v[7][2] = -_sizeZ / 2;
  v[1][2] = v[2][2] = v[5][2] = v[6][2] = _sizeZ / 2;

  fcl::BVHModel<BV>* model = new fcl::BVHModel<BV>;
  fcl::Vec3f p1, p2, p3;
  model->beginModel();

  for (int i = 0; i < 6; i++)
  {
    p1 = fcl::Vec3f(v[faces[i][0]][0], v[faces[i][0]][1], v[faces[i][0]][2]);
    p2 = fcl::Vec3f(v[faces[i][1]][0], v[faces[i][1]][1], v[faces[i][1]][2]);
    p3 = fcl::Vec3f(v[faces[i][2]][0], v[faces[i][2]][1], v[faces[i][2]][2]);
    model->addTriangle(p1, p2, p3);

    p1 = fcl::Vec3f(v[faces[i][0]][0], v[faces[i][0]][1], v[faces[i][0]][2]);
    p2 = fcl::Vec3f(v[faces[i][2]][0], v[faces[i][2]][1], v[faces[i][2]][2]);
    p3 = fcl::Vec3f(v[faces[i][3]][0], v[faces[i][3]][1], v[faces[i][3]][2]);
    model->addTriangle(p1, p2, p3);
  }
  model->endModel();
  return model;
}

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
fcl::BVHModel<BV>* createCylinder(double _baseRadius, double _topRadius,
                                  double _height, int _slices, int _stacks)
{
  const int CACHE_SIZE = 240;

  int i, j;
  float sinCache[CACHE_SIZE];
  float cosCache[CACHE_SIZE];
  float angle;
  float zBase;
  float zLow, zHigh;
  float sintemp, costemp;
  float deltaRadius;
  float radiusLow, radiusHigh;

  if (_slices >= CACHE_SIZE) _slices = CACHE_SIZE-1;

  if (_slices < 2 || _stacks < 1 || _baseRadius < 0.0 || _topRadius < 0.0 ||
      _height < 0.0)
  {
    return nullptr;
  }

  /* Center at CoM */
  zBase = -_height/2;

  /* Compute delta */
  deltaRadius = _baseRadius - _topRadius;

  /* Cache is the vertex locations cache */
  for (i = 0; i < _slices; i++)
  {
    angle = 2 * M_PI * i / _slices;
    sinCache[i] = sin(angle);
    cosCache[i] = cos(angle);
  }

  sinCache[_slices] = sinCache[0];
  cosCache[_slices] = cosCache[0];

  fcl::BVHModel<BV>* model = new fcl::BVHModel<BV>;
  fcl::Vec3f p1, p2, p3, p4;

  model->beginModel();

  /* Base of cylinder */
  sintemp = sinCache[0];
  costemp = cosCache[0];
  radiusLow = _baseRadius;
  zLow = zBase;
  p1 = fcl::Vec3f(radiusLow * sintemp, radiusLow * costemp, zLow);
  for (i = 1; i < _slices; i++)
  {
    p2 = fcl::Vec3f(radiusLow * sinCache[i], radiusLow * cosCache[i], zLow);
    p3 = fcl::Vec3f(radiusLow * sinCache[i+1], radiusLow * cosCache[i+1], zLow);
    model->addTriangle(p1, p2, p3);
  }

  /* Body of cylinder */
  for (i = 0; i < _slices; i++)
  {
    for (j = 0; j < _stacks; j++)
    {
      zLow = j * _height / _stacks + zBase;
      zHigh = (j + 1) * _height / _stacks + zBase;
      radiusLow = _baseRadius
                  - deltaRadius * (static_cast<float>(j) / _stacks);
      radiusHigh = _baseRadius
                   - deltaRadius * (static_cast<float>(j + 1) / _stacks);

      p1 = fcl::Vec3f(radiusLow * sinCache[i], radiusLow * cosCache[i],
                      zLow);
      p2 = fcl::Vec3f(radiusLow * sinCache[i+1], radiusLow * cosCache[i+1],
                      zLow);
      p3 = fcl::Vec3f(radiusHigh * sinCache[i], radiusHigh * cosCache[i],
                      zHigh);
      p4 = fcl::Vec3f(radiusHigh * sinCache[i+1], radiusHigh * cosCache[i+1],
                      zHigh);

      model->addTriangle(p1, p2, p3);
      model->addTriangle(p2, p3, p4);
    }
  }

  /* Top of cylinder */
  sintemp = sinCache[0];
  costemp = cosCache[0];
  radiusLow = _topRadius;
  zLow = zBase + _height;
  p1 = fcl::Vec3f(radiusLow * sintemp, radiusLow * costemp, zLow);
  for (i = 1; i < _slices; i++)
  {
    p2 = fcl::Vec3f(radiusLow * sinCache[i], radiusLow * cosCache[i], zLow);
    p3 = fcl::Vec3f(radiusLow * sinCache[i+1], radiusLow * cosCache[i+1], zLow);
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
  for (std::size_t i = 0; i < _mesh->mNumMeshes; i++)
  {
    for (std::size_t j = 0; j < _mesh->mMeshes[i]->mNumFaces; j++)
    {
      fcl::Vec3f vertices[3];
      for (std::size_t k = 0; k < 3; k++)
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
fcl::BVHModel<BV>* createSoftMesh(const aiMesh* _mesh)
{
  // Create FCL mesh from Assimp mesh

  assert(_mesh);
  fcl::BVHModel<BV>* model = new fcl::BVHModel<BV>;
  model->beginModel();

  for (std::size_t i = 0; i < _mesh->mNumFaces; i++)
  {
    fcl::Vec3f vertices[3];
    for (std::size_t j = 0; j < 3; j++)
    {
      const aiVector3D& vertex = _mesh->mVertices[_mesh->mFaces[i].mIndices[j]];
      vertices[j] = fcl::Vec3f(vertex.x, vertex.y, vertex.z);
    }
    model->addTriangle(vertices[0], vertices[1], vertices[2]);
  }

  model->endModel();
  return model;
}

} // anonymous namespace



//==============================================================================
std::shared_ptr<FCLCollisionDetector> FCLCollisionDetector::create()
{
  return std::shared_ptr<FCLCollisionDetector>(new FCLCollisionDetector());
}

//==============================================================================
FCLCollisionDetector::~FCLCollisionDetector()
{
  assert(mShapeMap.empty());
}

//==============================================================================
std::shared_ptr<CollisionDetector>
FCLCollisionDetector::cloneWithoutCollisionObjects()
{
  return FCLCollisionDetector::create();
}

//==============================================================================
const std::string& FCLCollisionDetector::getType() const
{
  return getStaticType();
}

//==============================================================================
const std::string& FCLCollisionDetector::getStaticType()
{
  static const std::string type = "fcl";
  return type;
}

//==============================================================================
std::unique_ptr<CollisionGroup>
FCLCollisionDetector::createCollisionGroup()
{
  return common::make_unique<FCLCollisionGroup>(shared_from_this());
}

//==============================================================================
static bool checkGroupValidity(FCLCollisionDetector* cd, CollisionGroup* group)
{
  if (cd != group->getCollisionDetector().get())
  {
    dterr << "[FCLCollisionDetector::collide] Attempting to check collision "
          << "for a collision group that is created from a different collision "
          << "detector instance.\n";

    return false;
  }

  return true;
}

//==============================================================================
bool FCLCollisionDetector::collide(
    CollisionGroup* group,
    const CollisionOption& option,
    CollisionResult* result)
{
  if (result)
    result->clear();

  if (0u == option.maxNumContacts)
    return false;

  if (!checkGroupValidity(this, group))
    return false;

  auto casted = static_cast<FCLCollisionGroup*>(group);
  casted->updateEngineData();

  FCLCollisionCallbackData collData(
        option, result, mPrimitiveShapeType,
        mContactPointComputationMethod);

  casted->getFCLCollisionManager()->collide(&collData, collisionCallback);

  return collData.isCollision();
}

//==============================================================================
bool FCLCollisionDetector::collide(
    CollisionGroup* group1, CollisionGroup* group2,
    const CollisionOption& option, CollisionResult* result)
{
  if (result)
    result->clear();

  if (0u == option.maxNumContacts)
    return false;

  if (!checkGroupValidity(this, group1))
    return false;

  if (!checkGroupValidity(this, group2))
    return false;

  auto casted1 = static_cast<FCLCollisionGroup*>(group1);
  auto casted2 = static_cast<FCLCollisionGroup*>(group2);
  casted1->updateEngineData();
  casted2->updateEngineData();

  FCLCollisionCallbackData collData(
        option, result, mPrimitiveShapeType,
        mContactPointComputationMethod);

  auto broadPhaseAlg1 = casted1->getFCLCollisionManager();
  auto broadPhaseAlg2 = casted2->getFCLCollisionManager();

  broadPhaseAlg1->collide(broadPhaseAlg2, &collData, collisionCallback);

  return collData.isCollision();
}

//==============================================================================
double FCLCollisionDetector::distance(
    CollisionGroup* group,
    const DistanceOption& option,
    DistanceResult* result)
{
  if (result)
    result->clear();

  if (!checkGroupValidity(this, group))
    return 0.0;

  auto casted = static_cast<FCLCollisionGroup*>(group);
  casted->updateEngineData();

  FCLDistanceCallbackData distData(option, result);

  casted->getFCLCollisionManager()->distance(&distData, distanceCallback);

  return std::max(distData.unclampedMinDistance, option.distanceLowerBound);
}

//==============================================================================
double FCLCollisionDetector::distance(
    CollisionGroup* group1,
    CollisionGroup* group2,
    const DistanceOption& option,
    DistanceResult* result)
{
  if (result)
    result->clear();

  if (!checkGroupValidity(this, group1))
    return 0.0;

  if (!checkGroupValidity(this, group2))
    return 0.0;

  auto casted1 = static_cast<FCLCollisionGroup*>(group1);
  auto casted2 = static_cast<FCLCollisionGroup*>(group2);
  casted1->updateEngineData();
  casted2->updateEngineData();

  FCLDistanceCallbackData distData(option, result);

  auto broadPhaseAlg1 = casted1->getFCLCollisionManager();
  auto broadPhaseAlg2 = casted2->getFCLCollisionManager();

  broadPhaseAlg1->distance(broadPhaseAlg2, &distData, distanceCallback);

  return std::max(distData.unclampedMinDistance, option.distanceLowerBound);
}

//==============================================================================
void FCLCollisionDetector::setPrimitiveShapeType(
    FCLCollisionDetector::PrimitiveShape type)
{
  if (type == PRIMITIVE)
  {
    dtwarn << "[FCLCollisionDetector::setPrimitiveShapeType] You chose to use "
           << "FCL's primitive shape collision feature while it's not complete "
           << "(at least until 0.4.0) especially in use of dynamics "
           << "simulation. It's recommended to use mesh even for primitive "
           << "shapes by settting "
           << "FCLCollisionDetector::setPrimitiveShapeType(MESH).\n";
  }

  mPrimitiveShapeType = type;
}

//==============================================================================
FCLCollisionDetector::PrimitiveShape
FCLCollisionDetector::getPrimitiveShapeType() const
{
  return mPrimitiveShapeType;
}

//==============================================================================
void FCLCollisionDetector::setContactPointComputationMethod(
    FCLCollisionDetector::ContactPointComputationMethod method)
{
  if (method == FCL)
  {
    dtwarn << "[FCLCollisionDetector::setContactPointComputationMethod] You "
           << "chose to use FCL's built in contact point computation while"
           << "it's buggy (see https://github.com/flexible-collision-library/"
           << "fcl/issues/106) at least until 0.4.0. It's recommended to use "
           << "DART's implementation for the contact point computation by "
           << "setting "
           << "FCLCollisionDetector::setContactPointComputationMethod(DART).\n";
  }

  mContactPointComputationMethod = method;
}

//==============================================================================
FCLCollisionDetector::ContactPointComputationMethod
FCLCollisionDetector::getContactPointComputationMethod() const
{
  return mContactPointComputationMethod;
}

//==============================================================================
FCLCollisionDetector::FCLCollisionDetector()
  : CollisionDetector(),
    mPrimitiveShapeType(MESH),
    mContactPointComputationMethod(DART)
{
  mCollisionObjectManager.reset(new ManagerForSharableCollisionObjects(this));
}

//==============================================================================
std::unique_ptr<CollisionObject> FCLCollisionDetector::createCollisionObject(
    const dynamics::ShapeFrame* shapeFrame)
{
  auto fclCollGeom = claimFCLCollisionGeometry(shapeFrame->getShape());

  return std::unique_ptr<FCLCollisionObject>(
        new FCLCollisionObject(this, shapeFrame, fclCollGeom));
}

//==============================================================================
fcl_shared_ptr<fcl::CollisionGeometry>
FCLCollisionDetector::claimFCLCollisionGeometry(
    const dynamics::ConstShapePtr& shape)
{
  const auto search = mShapeMap.find(shape);

  if (mShapeMap.end() != search)
  {
    const auto& fclCollGeom = search->second;
    assert(fclCollGeom.lock());
    // Ensure all the collision geometry in the map should be alive pointers.

    return fclCollGeom.lock();
  }

  auto newfclCollGeom = createFCLCollisionGeometry(
        shape, mPrimitiveShapeType, FCLCollisionGeometryDeleter(this, shape));
  mShapeMap[shape] = newfclCollGeom;

  return newfclCollGeom;
}

//==============================================================================
fcl_shared_ptr<fcl::CollisionGeometry>
FCLCollisionDetector::createFCLCollisionGeometry(
    const dynamics::ConstShapePtr& shape,
    FCLCollisionDetector::PrimitiveShape type,
    const FCLCollisionGeometryDeleter& deleter)
{
  using dynamics::Shape;
  using dynamics::SphereShape;
  using dynamics::BoxShape;
  using dynamics::EllipsoidShape;
  using dynamics::CylinderShape;
  using dynamics::PlaneShape;
  using dynamics::MeshShape;
  using dynamics::SoftMeshShape;

  fcl::CollisionGeometry* geom = nullptr;
  const auto& shapeType = shape->getType();

  if (SphereShape::getStaticType() == shapeType)
  {
    assert(dynamic_cast<const SphereShape*>(shape.get()));

    auto* sphere = static_cast<const SphereShape*>(shape.get());
    const auto radius = sphere->getRadius();

    if (FCLCollisionDetector::PRIMITIVE == type)
      geom = new fcl::Sphere(radius);
    else
      geom = createEllipsoid<fcl::OBBRSS>(radius*2.0, radius*2.0, radius*2.0);
  }
  else if (BoxShape::getStaticType() == shapeType)
  {
    assert(dynamic_cast<const BoxShape*>(shape.get()));

    auto box = static_cast<const BoxShape*>(shape.get());
    const Eigen::Vector3d& size = box->getSize();

    if (FCLCollisionDetector::PRIMITIVE == type)
      geom = new fcl::Box(size[0], size[1], size[2]);
    else
      geom = createCube<fcl::OBBRSS>(size[0], size[1], size[2]);
  }
  else if (EllipsoidShape::getStaticType() == shapeType)
  {
    assert(dynamic_cast<const EllipsoidShape*>(shape.get()));

    auto ellipsoid = static_cast<const EllipsoidShape*>(shape.get());
    const Eigen::Vector3d& size = ellipsoid->getSize();

    if (FCLCollisionDetector::PRIMITIVE == type)
    {
#if FCL_VERSION_AT_LEAST(0,4,0)
      geom = new fcl::Ellipsoid(FCLTypes::convertVector3(size * 0.5));
#else
      geom = createEllipsoid<fcl::OBBRSS>(size[0], size[1], size[2]);
#endif
    }
    else
    {
      geom = createEllipsoid<fcl::OBBRSS>(size[0], size[1], size[2]);
    }
  }
  else if (CylinderShape::getStaticType() == shapeType)
  {
    assert(dynamic_cast<const CylinderShape*>(shape.get()));

    const auto cylinder = static_cast<const CylinderShape*>(shape.get());
    const auto radius = cylinder->getRadius();
    const auto height = cylinder->getHeight();

    if (FCLCollisionDetector::PRIMITIVE == type)
    {
      geom = createCylinder<fcl::OBBRSS>(radius, radius, height, 16, 16);
      // TODO(JS): We still need to use mesh for cylinder because FCL 0.4.0
      // returns single contact point for cylinder yet. Once FCL support
      // multiple contact points then above code will be replaced by:
      // fclCollGeom.reset(new fcl::Cylinder(radius, height));
    }
    else
    {
      geom = createCylinder<fcl::OBBRSS>(radius, radius, height, 16, 16);
    }
  }
  else if (PlaneShape::getStaticType() == shapeType)
  {
    if (FCLCollisionDetector::PRIMITIVE == type)
    {
      assert(dynamic_cast<const PlaneShape*>(shape.get()));
      auto                  plane = static_cast<const PlaneShape*>(shape.get());
      const Eigen::Vector3d normal = plane->getNormal();
      const double          offset = plane->getOffset();

      geom = new fcl::Halfspace(FCLTypes::convertVector3(normal), offset);
    }
    else
    {
      geom = createCube<fcl::OBBRSS>(1000.0, 0.0, 1000.0);
      dtwarn << "[FCLCollisionDetector] PlaneShape is not supported by "
             << "FCLCollisionDetector. We create a thin box mesh insted, where "
             << "the size is [1000 0 1000].\n";
    }
  }
  else if (MeshShape::getStaticType() == shapeType)
  {
    assert(dynamic_cast<const MeshShape*>(shape.get()));

    auto shapeMesh = static_cast<const MeshShape*>(shape.get());
    const Eigen::Vector3d& scale = shapeMesh->getScale();
    auto aiScene = shapeMesh->getMesh();

    geom = createMesh<fcl::OBBRSS>(scale[0], scale[1], scale[2], aiScene);
  }
  else if (SoftMeshShape::getStaticType() == shapeType)
  {
    assert(dynamic_cast<const SoftMeshShape*>(shape.get()));

    auto softMeshShape = static_cast<const SoftMeshShape*>(shape.get());
    auto aiMesh = softMeshShape->getAssimpMesh();

    geom = createSoftMesh<fcl::OBBRSS>(aiMesh);
  }
  else
  {
    dterr << "[FCLCollisionDetector::createFCLCollisionGeometry] "
          << "Attempting to create an unsupported shape type ["
          << shapeType << "]. Creating a sphere with 0.1 radius "
          << "instead.\n";

    geom = createEllipsoid<fcl::OBBRSS>(0.1, 0.1, 0.1);
  }

  return fcl_shared_ptr<fcl::CollisionGeometry>(geom, deleter);
}

//==============================================================================
FCLCollisionDetector::FCLCollisionGeometryDeleter::FCLCollisionGeometryDeleter(
    FCLCollisionDetector* cd,
    const dynamics::ConstShapePtr& shape)
  : mFCLCollisionDetector(cd),
    mShape(shape)
{
  assert(cd);
  assert(shape);
}

//==============================================================================
void FCLCollisionDetector::FCLCollisionGeometryDeleter::operator()(
    fcl::CollisionGeometry* geom) const
{
  mFCLCollisionDetector->mShapeMap.erase(mShape);

  delete geom;
}



namespace {

//==============================================================================
bool collisionCallback(
    fcl::CollisionObject* o1, fcl::CollisionObject* o2, void* cdata)
{
  // Return true if you don't want more narrow phase collision checking after
  // this callback function returns, return false otherwise.

  auto collData = static_cast<FCLCollisionCallbackData*>(cdata);

  if (collData->done)
    return true;

  const auto& fclRequest  = collData->fclRequest;
        auto& fclResult   = collData->fclResult;
        auto* result      = collData->result;
  const auto& option      = collData->option;
  const auto& filter      = option.collisionFilter;

  // Filtering
  if (filter)
  {
    auto userData1
        = static_cast<FCLCollisionObject::UserData*>(o1->getUserData());
    auto userData2
        = static_cast<FCLCollisionObject::UserData*>(o2->getUserData());
    assert(userData1);
    assert(userData2);

    auto collisionObject1 = userData1->mCollisionObject;
    auto collisionObject2 = userData2->mCollisionObject;
    assert(collisionObject1);
    assert(collisionObject2);

    if (!filter->needCollision(collisionObject2, collisionObject1))
      return collData->done;
  }

  // Clear previous results
  fclResult.clear();

  // Perform narrow-phase detection
  fcl::collide(o1, o2, fclRequest, fclResult);

  if (result)
  {
    // Post processing -- converting fcl contact information to ours if needed
    if (FCLCollisionDetector::DART == collData->contactPointComputationMethod
        && FCLCollisionDetector::MESH == collData->primitiveShapeType)
    {
      postProcessDART(fclResult, o1, o2, option, *result);
    }
    else
    {
      postProcessFCL(fclResult, o1, o2, option, *result);
    }

    // Check satisfaction of the stopping conditions
    if (result->getNumContacts() >= option.maxNumContacts)
      collData->done = true;
  }
  else
  {
    // If no result is passed, stop checking when the first contact is found
    if (fclResult.isCollision())
    {
      collData->foundCollision = true;
      collData->done = true;
    }
  }

  return collData->done;
}

//==============================================================================
bool distanceCallback(
    fcl::CollisionObject* o1,
    fcl::CollisionObject* o2,
    void* ddata,
    fcl::FCL_REAL& dist)
{
  auto* distData = static_cast<FCLDistanceCallbackData*>(ddata);

  const auto& fclRequest = distData->fclRequest;
        auto& fclResult  = distData->fclResult;
        auto* result     = distData->result;
  const auto& option     = distData->option;
  const auto& filter     = option.distanceFilter;

  if (distData->done)
  {
    dist = distData->unclampedMinDistance;
    return true;
  }

  // Filtering
  if (filter)
  {
    auto userData1
        = static_cast<FCLCollisionObject::UserData*>(o1->getUserData());
    auto userData2
        = static_cast<FCLCollisionObject::UserData*>(o2->getUserData());
    assert(userData1);
    assert(userData2);

    auto collisionObject1 = userData1->mCollisionObject;
    auto collisionObject2 = userData2->mCollisionObject;
    assert(collisionObject1);
    assert(collisionObject2);

    if (!filter->needDistance(collisionObject2, collisionObject1))
      return distData->done;
  }

  // Clear previous results
  fclResult.clear();

  // Perform narrow-phase check
  fcl::distance(o1, o2, fclRequest, fclResult);

  // Store the minimum distance just in case result is nullptr.
  distData->unclampedMinDistance = fclResult.min_distance;

  if (result)
    interpreteDistanceResult(fclResult, o1, o2, option, *result);

  if (distData->unclampedMinDistance <= option.distanceLowerBound)
    distData->done = true;

  return distData->done;
}

//==============================================================================
Eigen::Vector3d getDiff(const Contact& contact1, const Contact& contact2)
{
  return contact1.point - contact2.point;
}

//==============================================================================
fcl::Vec3f getDiff(const fcl::Contact& contact1, const fcl::Contact& contact2)
{
  return contact1.pos - contact2.pos;
}

//==============================================================================
template <typename ResultT,
          typename ContactT,
          const ContactT&(ResultT::*GetFun)(std::size_t) const>
void markRepeatedPoints(
    std::vector<bool>& markForDeletion,
    const ResultT& fclResult,
    double tol)
{
  const auto checkSize = markForDeletion.size();

  for (auto i = 0u; i < checkSize - 1u; ++i)
  {
    const auto& contact1 = (fclResult.*GetFun)(i);

    for (auto j = i + 1u; j < checkSize; ++j)
    {
      const auto& contact2 = (fclResult.*GetFun)(j);

      const auto diff = getDiff(contact1, contact2);

      if (diff.dot(diff) < tol)
      {
        markForDeletion[i] = true;
        break;
      }
    }
  }
}

//==============================================================================
template <typename ResultT,
          typename ContactT,
          const ContactT&(ResultT::*GetFun)(std::size_t) const>
void markColinearPoints(
    std::vector<bool>& markForDeletion,
    const ResultT& fclResult,
    double tol)
{
  const auto checkSize = markForDeletion.size();

  for (auto i = 0u; i < checkSize; ++i)
  {
    if (markForDeletion[i])
      continue;

    const auto& contact1 = (fclResult.*GetFun)(i);

    for (auto j = i + 1u; j < checkSize; ++j)
    {
      if (i == j || markForDeletion[j])
        continue;

      if (markForDeletion[i])
        break;

      const auto& contact2 = (fclResult.*GetFun)(j);

      for (auto k = j + 1u; k < checkSize; ++k)
      {
        if (i == k)
          continue;

        const auto& contact3 = (fclResult.*GetFun)(k);

        if (isColinear(contact1, contact2, contact3, tol))
        {
          markForDeletion[i] = true;
          break;
        }
      }
    }
  }
}

//==============================================================================
void postProcessFCL(
    const fcl::CollisionResult& fclResult,
    fcl::CollisionObject* o1,
    fcl::CollisionObject* o2,
    const CollisionOption& option,
    CollisionResult& result)
{
  const auto numContacts = fclResult.numContacts();

  if (0u == numContacts)
    return;

  // For binary check, return after adding the first contact point to the result
  // without the checkings of repeatidity and co-linearity.
  if (1u == option.maxNumContacts)
  {
    result.addContact(convertContact(fclResult.getContact(0), o1, o2, option));

    return;
  }

  const auto tol = 1e-12;
  const auto tol3 = tol * 3.0;

  std::vector<bool> markForDeletion(numContacts, false);

  // mark all the repeated points
  markRepeatedPoints<
      fcl::CollisionResult,
      fcl::Contact,
      &fcl::CollisionResult::getContact>(markForDeletion, fclResult, tol3);

  // remove all the co-linear contact points
  markColinearPoints<
      fcl::CollisionResult,
      fcl::Contact,
      &fcl::CollisionResult::getContact>(markForDeletion, fclResult, tol);

  for (auto i = 0u; i < numContacts; ++i)
  {
    if (markForDeletion[i])
      continue;

    result.addContact(convertContact(fclResult.getContact(i), o1, o2, option));

    if (result.getNumContacts() >= option.maxNumContacts)
      return;
  }
}

//==============================================================================
void postProcessDART(
    const fcl::CollisionResult& fclResult,
    fcl::CollisionObject* o1,
    fcl::CollisionObject* o2,
    const CollisionOption& option,
    CollisionResult& result)
{
  const auto numFilteredContacts = fclResult.numContacts();

  if (0u == numFilteredContacts)
    return;

  auto numContacts = 0u;

  std::vector<Contact> unfiltered;
  unfiltered.reserve(numFilteredContacts * 2);

  for (auto i = 0u; i < numFilteredContacts; ++i)
  {
    const auto& c = fclResult.getContact(i);

    auto userData1
        = static_cast<FCLCollisionObject::UserData*>(o1->getUserData());
    auto userData2
        = static_cast<FCLCollisionObject::UserData*>(o2->getUserData());
    assert(userData1);
    assert(userData2);

    // for each pair of intersecting triangles, we create two contact points
    Contact pair1;
    Contact pair2;

    pair1.collisionObject1 = userData1->mCollisionObject;
    pair1.collisionObject2 = userData2->mCollisionObject;

    if (option.enableContact)
    {
      pair1.normal = FCLTypes::convertVector3(-c.normal);
      pair1.penetrationDepth = c.penetration_depth;
      pair1.triID1 = c.b1;
      pair1.triID2 = c.b2;
      pair2 = pair1;

      auto contactResult = evalContactPosition(
            c,
            static_cast<const fcl::BVHModel<fcl::OBBRSS>*>(c.o1),
            static_cast<const fcl::BVHModel<fcl::OBBRSS>*>(c.o2),
            FCLTypes::convertTransform(pair1.collisionObject1->getTransform()),
            FCLTypes::convertTransform(pair1.collisionObject2->getTransform()),
            pair1.point,
            pair2.point);

      if (contactResult == COPLANAR_CONTACT)
      {
        if (numContacts > 2u)
          continue;
      }
      else if (contactResult == NO_CONTACT)
      {
        continue;
      }
      else
      {
        numContacts++;
      }
    }

    // For binary check, return after adding the first contact point to the result
    // without the checkings of repeatidity and co-linearity.
    if (1u == option.maxNumContacts)
    {
      result.addContact(pair1);

      return;
    }

    unfiltered.push_back(pair1);
    unfiltered.push_back(pair2);
  }

  const auto tol = 1e-12;
  const auto tol3 = tol * 3.0;

  const auto unfilteredSize = unfiltered.size();

  std::vector<bool> markForDeletion(unfilteredSize, false);

  // mark all the repeated points
  markRepeatedPoints<
      std::vector<Contact>,
      Contact,
      &std::vector<Contact>::at>(markForDeletion, unfiltered, tol3);

  // remove all the co-linear contact points
  markColinearPoints<
      std::vector<Contact>,
      Contact,
      &std::vector<Contact>::at>(markForDeletion, unfiltered, tol);

  for (auto i = 0u; i < unfilteredSize; ++i)
  {
    if (markForDeletion[i])
      continue;

    result.addContact(unfiltered[i]);

    if (result.getNumContacts() >= option.maxNumContacts)
      return;
  }
}

//==============================================================================
void interpreteDistanceResult(
    const fcl::DistanceResult& fclResult,
    fcl::CollisionObject* o1,
    fcl::CollisionObject* o2,
    const DistanceOption& option,
    DistanceResult& result)
{
  result.unclampedMinDistance = fclResult.min_distance;
  result.minDistance
      = std::max(fclResult.min_distance, option.distanceLowerBound);

  const auto* userData1
      = static_cast<FCLCollisionObject::UserData*>(o1->getUserData());
  const auto* userData2
      = static_cast<FCLCollisionObject::UserData*>(o2->getUserData());
  assert(userData1);
  assert(userData2);
  assert(userData1->mCollisionObject);
  assert(userData2->mCollisionObject);

  result.shapeFrame1 = userData1->mCollisionObject->getShapeFrame();
  result.shapeFrame2 = userData2->mCollisionObject->getShapeFrame();

  if (option.enableNearestPoints)
  {
    result.nearestPoint1
        = FCLTypes::convertVector3(fclResult.nearest_points[0]);
    result.nearestPoint2
        = FCLTypes::convertVector3(fclResult.nearest_points[1]);
  }
}

//==============================================================================
int evalContactPosition(
    const fcl::Contact& fclContact,
    const fcl::BVHModel<fcl::OBBRSS>* mesh1,
    const fcl::BVHModel<fcl::OBBRSS>* mesh2,
    const fcl::Transform3f& transform1,
    const fcl::Transform3f& transform2,
    Eigen::Vector3d& contactPosition1,
    Eigen::Vector3d& contactPosition2)
{
  auto id1 = fclContact.b1;
  auto id2 = fclContact.b2;
  auto tri1 = mesh1->tri_indices[id1];
  auto tri2 = mesh2->tri_indices[id2];

  fcl::Vec3f v1, v2, v3, p1, p2, p3;
  v1 = mesh1->vertices[tri1[0]];
  v2 = mesh1->vertices[tri1[1]];
  v3 = mesh1->vertices[tri1[2]];

  p1 = mesh2->vertices[tri2[0]];
  p2 = mesh2->vertices[tri2[1]];
  p3 = mesh2->vertices[tri2[2]];

  fcl::Vec3f contact1, contact2;
  v1 = transform1.transform(v1);
  v2 = transform1.transform(v2);
  v3 = transform1.transform(v3);
  p1 = transform2.transform(p1);
  p2 = transform2.transform(p2);
  p3 = transform2.transform(p3);
  auto testRes = FFtest(v1, v2, v3, p1, p2, p3, &contact1, &contact2);

  if (testRes == COPLANAR_CONTACT)
  {
    auto area1 = triArea(v1, v2, v3);
    auto area2 = triArea(p1, p2, p3);

    if (area1 < area2)
      contact1 = v1 + v2 + v3;
    else
      contact1 = p1 + p2 + p3;

    contact1[0] /= 3.0;
    contact1[1] /= 3.0;
    contact1[2] /= 3.0;
    contact2 = contact1;
  }

  contactPosition1 << contact1[0], contact1[1], contact1[2];
  contactPosition2 << contact2[0], contact2[1], contact2[2];

  return testRes;
}

//==============================================================================
int FFtest(
    const fcl::Vec3f& r1, const fcl::Vec3f& r2, const fcl::Vec3f& r3,
    const fcl::Vec3f& R1, const fcl::Vec3f& R2, const fcl::Vec3f& R3,
    fcl::Vec3f* res1, fcl::Vec3f* res2)
{
  float U0[3], U1[3], U2[3], V0[3], V1[3], V2[3], RES1[3], RES2[3];
  SET(U0, r1);
  SET(U1, r2);
  SET(U2, r3);
  SET(V0, R1);
  SET(V1, R2);
  SET(V2, R3);

  int contactResult = tri_tri_intersect(V0, V1, V2, U0, U1, U2, RES1, RES2);

  SET((*res1), RES1);
  SET((*res2), RES2);

  return contactResult;
}

//==============================================================================
double triArea(fcl::Vec3f& p1, fcl::Vec3f& p2, fcl::Vec3f& p3)
{
  fcl::Vec3f a = p2 - p1;
  fcl::Vec3f b = p3 - p1;
  double aMag = a[0] * a[0] + a[1] * a[1] + a[2] * a[2];
  double bMag = b[0] * b[0] + b[1] * b[1] + b[2] * b[2];
  double dp = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  double area =  0.5 * sqrt(aMag * bMag - dp * dp);

  return area;
}

//==============================================================================
bool isColinear(
    const Contact& contact1,
    const Contact& contact2,
    const Contact& contact3,
    double tol)
{
  return isColinear(contact1.point, contact2.point, contact3.point, tol);
}

//==============================================================================
bool isColinear(
    const fcl::Contact& contact1,
    const fcl::Contact& contact2,
    const fcl::Contact& contact3,
    double tol)
{
  return isColinear(contact1.pos, contact2.pos, contact3.pos, tol);
}

//==============================================================================
template <typename T>
bool isColinear(const T& pos1, const T& pos2, const T& pos3, double tol)
{
  const auto va = pos1 - pos2;
  const auto vb = pos1 - pos3;
  const auto v = va.cross(vb);

  const auto cond1 = v.dot(v) < tol;
  const auto cond2 = va.dot(vb) < 0.0;

  return cond1 && cond2;
}

//==============================================================================
void convertOption(const CollisionOption& option, fcl::CollisionRequest& request)
{
  request.num_max_contacts = option.maxNumContacts;
  request.enable_contact   = option.enableContact;
#if FCL_VERSION_AT_LEAST(0,3,0)
  request.gjk_solver_type  = fcl::GST_LIBCCD;
#endif
}

//==============================================================================
void convertOption(const DistanceOption& option, fcl::DistanceRequest& request)
{
  request.enable_nearest_points = option.enableNearestPoints;
}

//==============================================================================
Contact convertContact(const fcl::Contact& fclContact,
                       fcl::CollisionObject* o1,
                       fcl::CollisionObject* o2,
                       const CollisionOption& option)
{
  Contact contact;

  auto userData1
      = static_cast<FCLCollisionObject::UserData*>(o1->getUserData());
  auto userData2
      = static_cast<FCLCollisionObject::UserData*>(o2->getUserData());
  assert(userData1);
  assert(userData2);
  contact.collisionObject1 = userData1->mCollisionObject;
  contact.collisionObject2 = userData2->mCollisionObject;

  if (option.enableContact)
  {
    contact.point = FCLTypes::convertVector3(fclContact.pos);
    contact.normal = -FCLTypes::convertVector3(fclContact.normal);
    contact.penetrationDepth = fclContact.penetration_depth;
    contact.triID1 = fclContact.b1;
    contact.triID2 = fclContact.b2;
  }

  return contact;
}

} // anonymous namespace

} // namespace collision
} // namespace dart
