/*
 * Copyright (c) 2011-2021, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#pragma once

#include "dart/collision/fcl/FCLCollisionDetector.hpp"

#include <assimp/scene.h>

#include "dart/collision/CollisionResult.hpp"
#include "dart/collision/fcl/PrimitiveShapeUtils.hpp"
#include "dart/common/Console.hpp"
#include "dart/math/geometry/Sphere.hpp"
//#include "dart/dynamics/BoxShape.hpp"
//#include "dart/dynamics/CollisionFilter.hpp"
//#include "dart/dynamics/CollisionObject.hpp"
//#include "dart/dynamics/ConeShape.hpp"
//#include "dart/dynamics/CylinderShape.hpp"
//#include "dart/dynamics/DistanceFilter.hpp"
//#include "dart/dynamics/EllipsoidShape.hpp"
//#include "dart/dynamics/MeshShape.hpp"
//#include "dart/dynamics/PlaneShape.hpp"
//#include "dart/dynamics/PyramidShape.hpp"
//#include "dart/dynamics/Shape.hpp"
//#include "dart/dynamics/ShapeFrame.hpp"
//#include "dart/dynamics/SoftMeshShape.hpp"
//#include "dart/dynamics/SphereShape.hpp"
//#include "dart/dynamics/VoxelGridShape.hpp"
//#include "dart/dynamics/fcl/BackwardCompatibility.hpp"
#include "dart/collision/fcl/FCLCollisionGroup.hpp"
#include "dart/collision/fcl/FCLCollisionObject.hpp"
//#include "dart/dynamics/fcl/tri_tri_intersection_test.hpp"

namespace dart {
namespace collision2 {

// namespace {

// bool collisionCallback(
//    fcl::CollisionObject* o1, fcl::CollisionObject* o2, void* cdata);

// bool distanceCallback(
//    fcl::CollisionObject* o1,
//    fcl::CollisionObject* o2,
//    void* cdata,
//    double& dist);

// void postProcessFCL(
//    const fcl::CollisionResult& fclResult,
//    fcl::CollisionObject* o1,
//    fcl::CollisionObject* o2,
//    const CollisionOption& option,
//    CollisionResult& result);

// void postProcessDART(
//    const fcl::CollisionResult& fclResult,
//    fcl::CollisionObject* o1,
//    fcl::CollisionObject* o2,
//    const CollisionOption& option,
//    CollisionResult& result);

// void interpreteDistanceResult(
//    const fcl::DistanceResult& fclResult,
//    fcl::CollisionObject* o1,
//    fcl::CollisionObject* o2,
//    const DistanceOption& option,
//    DistanceResult& result);

// int evalContactPosition(
//    const fcl::Contact& fclContact,
//    const ::fcl::BVHModel<fcl::OBBRSS>& mesh1,
//    const ::fcl::BVHModel<fcl::OBBRSS>& mesh2,
//    const fcl::Transform3& transform1,
//    const fcl::Transform3& transform2,
//    Eigen::Vector3d& contactPosition1,
//    Eigen::Vector3d& contactPosition2);

// Eigen::Vector3d getDiff(const Contact& contact1, const Contact& contact2);

// fcl::Vector3 getDiff(
//    const fcl::Contact& contact1, const fcl::Contact& contact2);

// bool isColinear(
//    const Contact& contact1,
//    const Contact& contact2,
//    const Contact& contact3,
//    double tol);

// bool isColinear(
//    const fcl::Contact& contact1,
//    const fcl::Contact& contact2,
//    const fcl::Contact& contact3,
//    double tol);

// template <typename T>
// bool isColinear(const T& pos1, const T& pos2, const T& pos3, double tol);

// int FFtest(
//    const fcl::Vector3& r1,
//    const fcl::Vector3& r2,
//    const fcl::Vector3& r3,
//    const fcl::Vector3& R1,
//    const fcl::Vector3& R2,
//    const fcl::Vector3& R3,
//    fcl::Vector3* res1,
//    fcl::Vector3* res2);

// double triArea(
//    const fcl::Vector3& p1, const fcl::Vector3& p2, const fcl::Vector3& p3);

// void convertOption(
//    const CollisionOption& option, fcl::CollisionRequest& request);

// void convertOption(const DistanceOption& option, fcl::DistanceRequest&
// request);

// Contact convertContact(
//    const fcl::Contact& fclContact,
//    fcl::CollisionObject* o1,
//    fcl::CollisionObject* o2,
//    const CollisionOption& option);

/// Collision data stores the collision request and the result given by
/// collision algorithm.
template <typename S>
struct FCLCollisionCallbackData
{
  //  /// FCL collision request
  //  fcl::CollisionRequest fclRequest;

  //  /// FCL collision result
  //  fcl::CollisionResult fclResult;

  //  /// Collision option of DART
  //  const CollisionOption& option;

  //  /// Collision result of DART
  //  CollisionResult* result;

  //  /// True if at least one contact is found. This flag is used only when
  //  /// mResult is nullptr; otherwise the actual collision result is in
  //  mResult. bool foundCollision;

  //  FCLCollisionDetector<S>::PrimitiveShape primitiveShapeType;

  //  FCLCollisionDetector<S>::ContactPointComputationMethod
  //      contactPointComputationMethod;

  //  /// Whether the collision iteration can stop
  //  bool done;

  //  bool isCollision() const
  //  {
  //    if (result)
  //      return result->isCollision();
  //    else
  //      return foundCollision;
  //  }

  //  /// Constructor
  //  FCLCollisionCallbackData(
  //      const CollisionOption& option,
  //      CollisionResult* result,
  //      FCLCollisionDetector<S>::PrimitiveShape type =
  //      FCLCollisionDetector<S>::MESH,
  //      FCLCollisionDetector<S>::ContactPointComputationMethod method
  //      = FCLCollisionDetector<S>::DART)
  //    : option(option),
  //      result(result),
  //      foundCollision(false),
  //      primitiveShapeType(type),
  //      contactPointComputationMethod(method),
  //      done(false)
  //  {
  //    convertOption(option, fclRequest);

  //    fclRequest.num_max_contacts
  //        = std::max(static_cast<std::size_t>(100u), option.maxNumContacts);
  //    // Since some contact points can be filtered out in the post process, we
  //    ask
  //    // more than the demend. 100 is randomly picked.
  //  }
};

// struct FCLDistanceCallbackData
//{
//  /// FCL distance request
//  fcl::DistanceRequest fclRequest;

//  /// FCL distance result
//  fcl::DistanceResult fclResult;

//  /// Distance option of DART
//  const DistanceOption& option;

//  /// Minimum distance identical to DistanceResult::minDistnace if result is
//  not
//  /// nullptr.
//  double unclampedMinDistance;

//  /// Distance result of DART
//  DistanceResult* result;

//  /// @brief Whether the distance iteration can stop
//  bool done;

//  FCLDistanceCallbackData(const DistanceOption& option, DistanceResult*
//  result)
//    : option(option), result(result), done(false)
//  {
//    convertOption(option, fclRequest);
//  }
//};

////==============================================================================
//// Create a cube mesh for collision detection
// template <class BV>
//::fcl::BVHModel<BV>* createCube(float _sizeX, float _sizeY, float _sizeZ)
//{
//  int faces[6][4] = {{0, 1, 2, 3},
//                     {3, 2, 6, 7},
//                     {7, 6, 5, 4},
//                     {4, 5, 1, 0},
//                     {5, 6, 2, 1},
//                     {7, 4, 0, 3}};
//  float v[8][3];

//  v[0][0] = v[1][0] = v[2][0] = v[3][0] = -_sizeX / 2;
//  v[4][0] = v[5][0] = v[6][0] = v[7][0] = _sizeX / 2;
//  v[0][1] = v[1][1] = v[4][1] = v[5][1] = -_sizeY / 2;
//  v[2][1] = v[3][1] = v[6][1] = v[7][1] = _sizeY / 2;
//  v[0][2] = v[3][2] = v[4][2] = v[7][2] = -_sizeZ / 2;
//  v[1][2] = v[2][2] = v[5][2] = v[6][2] = _sizeZ / 2;

//  ::fcl::BVHModel<BV>* model = new ::fcl::BVHModel<BV>;
//  fcl::Vector3 p1, p2, p3;
//  model->beginModel();

//  for (int i = 0; i < 6; i++)
//  {
//    p1 = fcl::Vector3(v[faces[i][0]][0], v[faces[i][0]][1],
//    v[faces[i][0]][2]); p2 = fcl::Vector3(v[faces[i][1]][0],
//    v[faces[i][1]][1], v[faces[i][1]][2]); p3 =
//    fcl::Vector3(v[faces[i][2]][0], v[faces[i][2]][1], v[faces[i][2]][2]);
//    model->addTriangle(p1, p2, p3);

//    p1 = fcl::Vector3(v[faces[i][0]][0], v[faces[i][0]][1],
//    v[faces[i][0]][2]); p2 = fcl::Vector3(v[faces[i][2]][0],
//    v[faces[i][2]][1], v[faces[i][2]][2]); p3 =
//    fcl::Vector3(v[faces[i][3]][0], v[faces[i][3]][1], v[faces[i][3]][2]);
//    model->addTriangle(p1, p2, p3);
//  }
//  model->endModel();
//  return model;
//}

////==============================================================================
// template <class BV>
//::fcl::BVHModel<BV>* createEllipsoid(float _sizeX, float _sizeY, float _sizeZ)
//{
//  float v[59][3] = {{0, 0, 0},
//                    {0.135299, -0.461940, -0.135299},
//                    {0.000000, -0.461940, -0.191342},
//                    {-0.135299, -0.461940, -0.135299},
//                    {-0.191342, -0.461940, 0.000000},
//                    {-0.135299, -0.461940, 0.135299},
//                    {0.000000, -0.461940, 0.191342},
//                    {0.135299, -0.461940, 0.135299},
//                    {0.191342, -0.461940, 0.000000},
//                    {0.250000, -0.353553, -0.250000},
//                    {0.000000, -0.353553, -0.353553},
//                    {-0.250000, -0.353553, -0.250000},
//                    {-0.353553, -0.353553, 0.000000},
//                    {-0.250000, -0.353553, 0.250000},
//                    {0.000000, -0.353553, 0.353553},
//                    {0.250000, -0.353553, 0.250000},
//                    {0.353553, -0.353553, 0.000000},
//                    {0.326641, -0.191342, -0.326641},
//                    {0.000000, -0.191342, -0.461940},
//                    {-0.326641, -0.191342, -0.326641},
//                    {-0.461940, -0.191342, 0.000000},
//                    {-0.326641, -0.191342, 0.326641},
//                    {0.000000, -0.191342, 0.461940},
//                    {0.326641, -0.191342, 0.326641},
//                    {0.461940, -0.191342, 0.000000},
//                    {0.353553, 0.000000, -0.353553},
//                    {0.000000, 0.000000, -0.500000},
//                    {-0.353553, 0.000000, -0.353553},
//                    {-0.500000, 0.000000, 0.000000},
//                    {-0.353553, 0.000000, 0.353553},
//                    {0.000000, 0.000000, 0.500000},
//                    {0.353553, 0.000000, 0.353553},
//                    {0.500000, 0.000000, 0.000000},
//                    {0.326641, 0.191342, -0.326641},
//                    {0.000000, 0.191342, -0.461940},
//                    {-0.326641, 0.191342, -0.326641},
//                    {-0.461940, 0.191342, 0.000000},
//                    {-0.326641, 0.191342, 0.326641},
//                    {0.000000, 0.191342, 0.461940},
//                    {0.326641, 0.191342, 0.326641},
//                    {0.461940, 0.191342, 0.000000},
//                    {0.250000, 0.353553, -0.250000},
//                    {0.000000, 0.353553, -0.353553},
//                    {-0.250000, 0.353553, -0.250000},
//                    {-0.353553, 0.353553, 0.000000},
//                    {-0.250000, 0.353553, 0.250000},
//                    {0.000000, 0.353553, 0.353553},
//                    {0.250000, 0.353553, 0.250000},
//                    {0.353553, 0.353553, 0.000000},
//                    {0.135299, 0.461940, -0.135299},
//                    {0.000000, 0.461940, -0.191342},
//                    {-0.135299, 0.461940, -0.135299},
//                    {-0.191342, 0.461940, 0.000000},
//                    {-0.135299, 0.461940, 0.135299},
//                    {0.000000, 0.461940, 0.191342},
//                    {0.135299, 0.461940, 0.135299},
//                    {0.191342, 0.461940, 0.000000},
//                    {0.000000, -0.500000, 0.000000},
//                    {0.000000, 0.500000, 0.000000}};

//  int f[112][3]
//      = {{1, 2, 9},    {9, 2, 10},   {2, 3, 10},   {10, 3, 11},  {3, 4, 11},
//         {11, 4, 12},  {4, 5, 12},   {12, 5, 13},  {5, 6, 13},   {13, 6, 14},
//         {6, 7, 14},   {14, 7, 15},  {7, 8, 15},   {15, 8, 16},  {8, 1, 16},
//         {16, 1, 9},   {9, 10, 17},  {17, 10, 18}, {10, 11, 18}, {18, 11, 19},
//         {11, 12, 19}, {19, 12, 20}, {12, 13, 20}, {20, 13, 21}, {13, 14, 21},
//         {21, 14, 22}, {14, 15, 22}, {22, 15, 23}, {15, 16, 23}, {23, 16, 24},
//         {16, 9, 24},  {24, 9, 17},  {17, 18, 25}, {25, 18, 26}, {18, 19, 26},
//         {26, 19, 27}, {19, 20, 27}, {27, 20, 28}, {20, 21, 28}, {28, 21, 29},
//         {21, 22, 29}, {29, 22, 30}, {22, 23, 30}, {30, 23, 31}, {23, 24, 31},
//         {31, 24, 32}, {24, 17, 32}, {32, 17, 25}, {25, 26, 33}, {33, 26, 34},
//         {26, 27, 34}, {34, 27, 35}, {27, 28, 35}, {35, 28, 36}, {28, 29, 36},
//         {36, 29, 37}, {29, 30, 37}, {37, 30, 38}, {30, 31, 38}, {38, 31, 39},
//         {31, 32, 39}, {39, 32, 40}, {32, 25, 40}, {40, 25, 33}, {33, 34, 41},
//         {41, 34, 42}, {34, 35, 42}, {42, 35, 43}, {35, 36, 43}, {43, 36, 44},
//         {36, 37, 44}, {44, 37, 45}, {37, 38, 45}, {45, 38, 46}, {38, 39, 46},
//         {46, 39, 47}, {39, 40, 47}, {47, 40, 48}, {40, 33, 48}, {48, 33, 41},
//         {41, 42, 49}, {49, 42, 50}, {42, 43, 50}, {50, 43, 51}, {43, 44, 51},
//         {51, 44, 52}, {44, 45, 52}, {52, 45, 53}, {45, 46, 53}, {53, 46, 54},
//         {46, 47, 54}, {54, 47, 55}, {47, 48, 55}, {55, 48, 56}, {48, 41, 56},
//         {56, 41, 49}, {2, 1, 57},   {3, 2, 57},   {4, 3, 57},   {5, 4, 57},
//         {6, 5, 57},   {7, 6, 57},   {8, 7, 57},   {1, 8, 57},   {49, 50, 58},
//         {50, 51, 58}, {51, 52, 58}, {52, 53, 58}, {53, 54, 58}, {54, 55, 58},
//         {55, 56, 58}, {56, 49, 58}};

//  ::fcl::BVHModel<BV>* model = new ::fcl::BVHModel<BV>;
//  fcl::Vector3 p1, p2, p3;
//  model->beginModel();

//  for (int i = 0; i < 112; i++)
//  {
//    p1 = fcl::Vector3(
//        v[f[i][0]][0] * _sizeX, v[f[i][0]][1] * _sizeY, v[f[i][0]][2] *
//        _sizeZ);
//    p2 = fcl::Vector3(
//        v[f[i][1]][0] * _sizeX, v[f[i][1]][1] * _sizeY, v[f[i][1]][2] *
//        _sizeZ);
//    p3 = fcl::Vector3(
//        v[f[i][2]][0] * _sizeX, v[f[i][2]][1] * _sizeY, v[f[i][2]][2] *
//        _sizeZ);

//    model->addTriangle(p1, p2, p3);
//  }

//  model->endModel();

//  return model;
//}

////==============================================================================
// template <class BV>
//::fcl::BVHModel<BV>* createCylinder(
//    double _baseRadius,
//    double _topRadius,
//    double _height,
//    int _slices,
//    int _stacks)
//{
//  const int CACHE_SIZE = 240;

//  int i, j;
//  float sinCache[CACHE_SIZE];
//  float cosCache[CACHE_SIZE];
//  float angle;
//  float zBase;
//  float zLow, zHigh;
//  float sintemp, costemp;
//  float deltaRadius;
//  float radiusLow, radiusHigh;

//  if (_slices >= CACHE_SIZE)
//    _slices = CACHE_SIZE - 1;

//  if (_slices < 2 || _stacks < 1 || _baseRadius < 0.0 || _topRadius < 0.0
//      || _height < 0.0)
//  {
//    return nullptr;
//  }

//  /* Center at CoM */
//  zBase = -_height / 2;

//  /* Compute delta */
//  deltaRadius = _baseRadius - _topRadius;

//  /* Cache is the vertex locations cache */
//  for (i = 0; i < _slices; i++)
//  {
//    angle = 2 * math::constantsd::pi() * i / _slices;
//    sinCache[i] = sin(angle);
//    cosCache[i] = cos(angle);
//  }

//  sinCache[_slices] = sinCache[0];
//  cosCache[_slices] = cosCache[0];

//  ::fcl::BVHModel<BV>* model = new ::fcl::BVHModel<BV>;
//  fcl::Vector3 p1, p2, p3, p4;

//  model->beginModel();

//  /* Base of cylinder */
//  sintemp = sinCache[0];
//  costemp = cosCache[0];
//  radiusLow = _baseRadius;
//  zLow = zBase;
//  p1 = fcl::Vector3(radiusLow * sintemp, radiusLow * costemp, zLow);
//  for (i = 1; i < _slices; i++)
//  {
//    p2 = fcl::Vector3(radiusLow * sinCache[i], radiusLow * cosCache[i], zLow);
//    p3 = fcl::Vector3(
//        radiusLow * sinCache[i + 1], radiusLow * cosCache[i + 1], zLow);
//    model->addTriangle(p1, p2, p3);
//  }

//  /* Body of cylinder */
//  for (i = 0; i < _slices; i++)
//  {
//    for (j = 0; j < _stacks; j++)
//    {
//      zLow = j * _height / _stacks + zBase;
//      zHigh = (j + 1) * _height / _stacks + zBase;
//      radiusLow = _baseRadius - deltaRadius * (static_cast<float>(j) /
//      _stacks); radiusHigh
//          = _baseRadius - deltaRadius * (static_cast<float>(j + 1) / _stacks);

//      p1 = fcl::Vector3(radiusLow * sinCache[i], radiusLow * cosCache[i],
//      zLow); p2 = fcl::Vector3(
//          radiusLow * sinCache[i + 1], radiusLow * cosCache[i + 1], zLow);
//      p3 = fcl::Vector3(
//          radiusHigh * sinCache[i], radiusHigh * cosCache[i], zHigh);
//      p4 = fcl::Vector3(
//          radiusHigh * sinCache[i + 1], radiusHigh * cosCache[i + 1], zHigh);

//      model->addTriangle(p1, p2, p3);
//      model->addTriangle(p2, p3, p4);
//    }
//  }

//  /* Top of cylinder */
//  sintemp = sinCache[0];
//  costemp = cosCache[0];
//  radiusLow = _topRadius;
//  zLow = zBase + _height;
//  p1 = fcl::Vector3(radiusLow * sintemp, radiusLow * costemp, zLow);
//  for (i = 1; i < _slices; i++)
//  {
//    p2 = fcl::Vector3(radiusLow * sinCache[i], radiusLow * cosCache[i], zLow);
//    p3 = fcl::Vector3(
//        radiusLow * sinCache[i + 1], radiusLow * cosCache[i + 1], zLow);
//    model->addTriangle(p1, p2, p3);
//  }

//  model->endModel();
//  return model;
//}

////==============================================================================
// template <typename BV>
//::fcl::BVHModel<BV>* createPyramid(
//    const dynamics::PyramidShape& shape, const fcl::Transform3& pose)
//{
//  ::fcl::BVHModel<BV>* model = new ::fcl::BVHModel<BV>;

//  std::vector<fcl::Vector3> points(5);
//  std::vector<::fcl::Triangle> faces(6);

//  const double w = shape.getBaseWidth();
//  const double d = shape.getBaseDepth();
//  const double h = shape.getHeight();

//  const double hTop = h / 2;
//  const double hBottom = -hTop;
//  const double left = -w / 2;
//  const double right = w / 2;
//  const double front = -d / 2;
//  const double back = d / 2;

//#if FCL_VERSION_AT_LEAST(0, 6, 0)
//  points[0] << 0, 0, hTop;
//  points[1] << right, back, hBottom;
//  points[2] << left, back, hBottom;
//  points[3] << left, front, hBottom;
//  points[4] << right, front, hBottom;
//#else
//  points[0].setValue(0, 0, hTop);
//  points[1].setValue(right, back, hBottom);
//  points[2].setValue(left, back, hBottom);
//  points[3].setValue(left, front, hBottom);
//  points[4].setValue(right, front, hBottom);
//#endif

//  faces[0].set(0, 1, 2);
//  faces[1].set(0, 2, 3);
//  faces[2].set(0, 3, 4);
//  faces[3].set(0, 4, 1);
//  faces[4].set(1, 3, 2);
//  faces[5].set(1, 4, 3);

//  for (unsigned int i = 0; i < points.size(); ++i)
//  {
//#if FCL_VERSION_AT_LEAST(0, 6, 0)
//    points[i] = pose * points[i];
//#else
//    points[i] = pose.transform(points[i]);
//#endif
//  }

//  model->beginModel();
//  model->addSubModel(points, faces);
//  model->endModel();
//  model->computeLocalAABB();

//  return model;
//}

////==============================================================================
// template <class BV>
//::fcl::BVHModel<BV>* createMesh(
//    float _scaleX, float _scaleY, float _scaleZ, const aiScene* _mesh)
//{
//  // Create FCL mesh from Assimp mesh

//  assert(_mesh);
//  ::fcl::BVHModel<BV>* model = new ::fcl::BVHModel<BV>;
//  model->beginModel();
//  for (std::size_t i = 0; i < _mesh->mNumMeshes; i++)
//  {
//    for (std::size_t j = 0; j < _mesh->mMeshes[i]->mNumFaces; j++)
//    {
//      fcl::Vector3 vertices[3];
//      for (std::size_t k = 0; k < 3; k++)
//      {
//        const aiVector3D& vertex
//            = _mesh->mMeshes[i]
//                  ->mVertices[_mesh->mMeshes[i]->mFaces[j].mIndices[k]];
//        vertices[k] = fcl::Vector3(
//            vertex.x * _scaleX, vertex.y * _scaleY, vertex.z * _scaleZ);
//      }
//      model->addTriangle(vertices[0], vertices[1], vertices[2]);
//    }
//  }
//  model->endModel();
//  return model;
//}

////==============================================================================
// template <class BV>
//::fcl::BVHModel<BV>* createSoftMesh(const aiMesh* _mesh)
//{
//  // Create FCL mesh from Assimp mesh

//  assert(_mesh);
//  ::fcl::BVHModel<BV>* model = new ::fcl::BVHModel<BV>;
//  model->beginModel();

//  for (std::size_t i = 0; i < _mesh->mNumFaces; i++)
//  {
//    fcl::Vector3 vertices[3];
//    for (std::size_t j = 0; j < 3; j++)
//    {
//      const aiVector3D& vertex =
//      _mesh->mVertices[_mesh->mFaces[i].mIndices[j]]; vertices[j] =
//      fcl::Vector3(vertex.x, vertex.y, vertex.z);
//    }
//    model->addTriangle(vertices[0], vertices[1], vertices[2]);
//  }

//  model->endModel();
//  return model;
//}

//} // anonymous namespace

//==============================================================================
template <typename S>
typename CollisionDetector<S>::template Registrar<FCLCollisionDetector<S>>
    FCLCollisionDetector<S>::mRegistrar{
        FCLCollisionDetector<S>::getStaticType(),
        []() -> std::shared_ptr<FCLCollisionDetector<S>> {
          return FCLCollisionDetector<S>::create();
        }};

//==============================================================================
template <typename S>
std::shared_ptr<FCLCollisionDetector<S>> FCLCollisionDetector<S>::create()
{
  return std::shared_ptr<FCLCollisionDetector>(new FCLCollisionDetector());
}

//==============================================================================
template <typename S>
FCLCollisionDetector<S>::~FCLCollisionDetector()
{
  assert(mShapeMap.empty());
}

//==============================================================================
template <typename S>
const std::string& FCLCollisionDetector<S>::getType() const
{
  return getStaticType();
}

//==============================================================================
template <typename S>
const std::string& FCLCollisionDetector<S>::getStaticType()
{
  static const std::string type = "fcl";
  return type;
}

//==============================================================================
template <typename S>
CollisionGroupPtr<S> FCLCollisionDetector<S>::createCollisionGroup()
{
  return std::make_shared<FCLCollisionGroup<S>>(this);
}

//==============================================================================
template <typename S>
static bool checkGroupValidity(
    FCLCollisionDetector<S>* cd, CollisionGroup<S>* group)
{
  if (cd != group->getCollisionDetector())
  {
    dterr << "[FCLCollisionDetector<S>::collide] Attempting to check collision "
          << "for a collision group that is created from a different collision "
          << "detector instance.\n";

    return false;
  }

  return true;
}

//==============================================================================
template <typename S>
std::shared_ptr<fcl::CollisionGeometry<S>>
FCLCollisionDetector<S>::createFCLCollisionGeometry(
    math::ConstGeometryPtr shape)
{
  //  const std::size_t currentVersion = shape->getVersion();

  const auto search = mShapeMap.insert(std::make_pair(shape, ShapeInfo()));
  const bool inserted = search.second;
  ShapeInfo& info = search.first->second;

  //  if (!inserted && currentVersion == info.mLastKnownVersion)
  //  {
  //    const auto& fclCollGeom = info.mShape;
  //    assert(fclCollGeom.lock());
  //    // Ensure all the collision geometry in the map should be alive
  //    pointers.

  //    return fclCollGeom.lock();
  //  }

  auto newfclCollGeom
      = createFCLCollisionGeometryImpl(shape, mPrimitiveShapeType);
  info.fclCollisionGeometry = newfclCollGeom;
  // info.lastKnownVersion = currentVersion;

  return newfclCollGeom;
}

//==============================================================================
template <typename S>
std::shared_ptr<fcl::CollisionGeometry<S>>
FCLCollisionDetector<S>::createFCLCollisionGeometryImpl(
    const math::ConstGeometryPtr& shape,
    FCLCollisionDetector<S>::PrimitiveShape type)
{
  fcl::CollisionGeometry<S>* geom = nullptr;
  const auto& shapeType = shape->getType();

  if (auto sphere = shape->as<math::Sphered>())
  {
    const auto radius = sphere->getRadius();

    if (FCLCollisionDetector<S>::PRIMITIVE == type)
    {
      geom = new fcl::Sphere<S>(radius);
    }
    else
    {
      auto fclMesh = new ::fcl::BVHModel<fcl::OBBRSS<S>>();
      auto fclSphere = ::fcl::Sphere(radius);
      ::fcl::generateBVHModel(
          *fclMesh, fclSphere, fcl::Transform3<S>(), 16, 16);
      geom = fclMesh;
    }
  }
  else
  {
    dterr << "[FCLCollisionDetector<S>::createFCLCollisionGeometry] "
          << "Attempting to create an unsupported shape type [" << shapeType
          << "]. Creating a sphere with 0.1 radius "
          << "instead.\n";

    geom = new fcl::Sphere<S>(0.1);
  }

  assert(geom);

  return std::shared_ptr<fcl::CollisionGeometry<S>>(geom);
}

} // namespace collision2
} // namespace dart
