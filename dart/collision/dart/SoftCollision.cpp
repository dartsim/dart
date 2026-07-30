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

#include "dart/collision/dart/SoftCollision.hpp"

#include "dart/collision/CollisionResult.hpp"
#include "dart/collision/Contact.hpp"
#include "dart/collision/dart/DARTCollisionObject.hpp"
#include "dart/common/Console.hpp"
#include "dart/dynamics/BoxShape.hpp"
#include "dart/dynamics/EllipsoidShape.hpp"
#include "dart/dynamics/PlaneShape.hpp"
#include "dart/dynamics/PointMass.hpp"
#include "dart/dynamics/Shape.hpp"
#include "dart/dynamics/SoftBodyNode.hpp"
#include "dart/dynamics/SoftMeshShape.hpp"
#include "dart/dynamics/SphereShape.hpp"

#include <algorithm>
#include <array>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <cmath>

namespace dart {
namespace collision {
namespace detail {
namespace {

constexpr double kCollisionEps = 1e-6;
constexpr double kSoftContactShell = 1e-6;

//==============================================================================
struct RelativeTransformView
{
  Eigen::Matrix3d rotation;
  Eigen::Vector3d translation;

  Eigen::Vector3d apply(const Eigen::Vector3d& point) const
  {
    return rotation * point + translation;
  }
};

//==============================================================================
RelativeTransformView makeRelativeTransformView(
    const Eigen::Isometry3d& targetFrame, const Eigen::Isometry3d& sourceFrame)
{
  const Eigen::Matrix3d targetRotationTranspose
      = targetFrame.linear().transpose();
  return RelativeTransformView{
      targetRotationTranspose * sourceFrame.linear(),
      targetRotationTranspose
          * (sourceFrame.translation() - targetFrame.translation())};
}

//==============================================================================
Eigen::Vector3d transformPoint(
    const Eigen::Isometry3d& transform, const Eigen::Vector3d& point)
{
  return transform.linear() * point + transform.translation();
}

//==============================================================================
int findFirstSoftFace(
    const dynamics::SoftBodyNode* softBodyNode, std::size_t pointMassIndex)
{
  for (std::size_t faceIndex = 0u; faceIndex < softBodyNode->getNumFaces();
       ++faceIndex) {
    const Eigen::Vector3i& face = softBodyNode->getFace(faceIndex);
    for (auto vertex = 0; vertex < 3; ++vertex) {
      if (face[vertex] >= 0
          && static_cast<std::size_t>(face[vertex]) == pointMassIndex) {
        return static_cast<int>(faceIndex);
      }
    }
  }

  return -1;
}

//==============================================================================
struct SoftPointCacheView
{
  const dynamics::SoftBodyNode* softBodyNode{nullptr};
  const std::vector<Eigen::Vector3d>* localVertices{nullptr};
  const std::vector<int>* firstFaceByPointMass{nullptr};
};

//==============================================================================
SoftPointCacheView makeSoftPointCacheView(
    const DARTCollisionObject* object,
    const dynamics::SoftBodyNode* softBodyNode)
{
  SoftPointCacheView view;
  view.softBodyNode = softBodyNode;
  if (object == nullptr || softBodyNode == nullptr)
    return view;

  const auto& localVertices = object->getCachedSoftLocalVertices();
  const auto& firstFaceByPointMass
      = object->getCachedSoftFirstFaceByPointMass();
  const auto numPointMasses = softBodyNode->getNumPointMasses();
  if (localVertices.size() == numPointMasses
      && firstFaceByPointMass.size() == numPointMasses) {
    view.localVertices = &localVertices;
    view.firstFaceByPointMass = &firstFaceByPointMass;
  }

  return view;
}

//==============================================================================
std::size_t getSoftPointCount(const SoftPointCacheView& view)
{
  if (view.localVertices != nullptr)
    return view.localVertices->size();

  return view.softBodyNode != nullptr ? view.softBodyNode->getNumPointMasses()
                                      : 0u;
}

//==============================================================================
const Eigen::Vector3d& getSoftLocalPosition(
    const SoftPointCacheView& view, std::size_t pointMassIndex)
{
  if (view.localVertices != nullptr)
    return (*view.localVertices)[pointMassIndex];

  return view.softBodyNode->getPointMass(pointMassIndex)->getLocalPosition();
}

//==============================================================================
int getSoftFirstFace(const SoftPointCacheView& view, std::size_t pointMassIndex)
{
  if (view.firstFaceByPointMass != nullptr)
    return (*view.firstFaceByPointMass)[pointMassIndex];

  return findFirstSoftFace(view.softBodyNode, pointMassIndex);
}

//==============================================================================
int addSphereSoftFaceInteriorContacts(
    DARTCollisionObject* object1,
    DARTCollisionObject* object2,
    double sphereRadius,
    const Eigen::Isometry3d& sphereTransform,
    DARTCollisionObject* softObject,
    const Eigen::Isometry3d& softTransform,
    bool softIsObject2,
    CollisionResult& result);

//==============================================================================
int addBoxSoftFaceInteriorContacts(
    DARTCollisionObject* object1,
    DARTCollisionObject* object2,
    const Eigen::Vector3d& boxSize,
    const Eigen::Isometry3d& boxTransform,
    DARTCollisionObject* softObject,
    const Eigen::Isometry3d& softTransform,
    bool softIsObject2,
    CollisionResult& result);

//==============================================================================
int collidePlaneSoftMesh(
    DARTCollisionObject* object1,
    DARTCollisionObject* object2,
    const Eigen::Vector3d& planeNormal,
    double planeOffset,
    const Eigen::Isometry3d& transform1,
    const dynamics::SoftMeshShape* softMesh,
    const Eigen::Isometry3d& transform2,
    CollisionResult& result)
{
  const auto* softBodyNode = softMesh ? softMesh->getSoftBodyNode() : nullptr;
  if (softBodyNode == nullptr)
    return 0;

  const Eigen::Vector3d worldNormal = transform1.linear() * planeNormal;
  const Eigen::Vector3d planePoint
      = transform1.translation() + worldNormal * planeOffset;
  const Eigen::Vector3d localNormal
      = transform2.linear().transpose() * worldNormal;
  const double localOffset
      = worldNormal.dot(transform2.translation() - planePoint);

  const auto softPoints = makeSoftPointCacheView(object2, softBodyNode);
  auto numContacts = 0;
  constexpr double contactTolerance = 1e-9;
  for (std::size_t i = 0u; i < getSoftPointCount(softPoints); ++i) {
    const Eigen::Vector3d& localVertex = getSoftLocalPosition(softPoints, i);
    const double signedDist = localNormal.dot(localVertex) + localOffset;
    if (signedDist > contactTolerance)
      continue;

    const int faceIndex = getSoftFirstFace(softPoints, i);
    if (faceIndex < 0)
      continue;

    Contact contact;
    contact.collisionObject1 = object1;
    contact.collisionObject2 = object2;
    contact.point = transformPoint(transform2, localVertex);
    contact.normal = -worldNormal;
    contact.penetrationDepth = std::max(0.0, -signedDist);
    contact.triID2 = faceIndex;
    result.addContact(contact);
    ++numContacts;
  }

  return numContacts;
}

//==============================================================================
int collideSoftMeshPlane(
    DARTCollisionObject* object1,
    DARTCollisionObject* object2,
    const dynamics::SoftMeshShape* softMesh,
    const Eigen::Isometry3d& transform1,
    const Eigen::Vector3d& planeNormal,
    double planeOffset,
    const Eigen::Isometry3d& transform2,
    CollisionResult& result)
{
  const auto* softBodyNode = softMesh ? softMesh->getSoftBodyNode() : nullptr;
  if (softBodyNode == nullptr)
    return 0;

  const Eigen::Vector3d worldNormal = transform2.linear() * planeNormal;
  const Eigen::Vector3d planePoint
      = transform2.translation() + worldNormal * planeOffset;
  const Eigen::Vector3d localNormal
      = transform1.linear().transpose() * worldNormal;
  const double localOffset
      = worldNormal.dot(transform1.translation() - planePoint);

  const auto softPoints = makeSoftPointCacheView(object1, softBodyNode);
  auto numContacts = 0;
  constexpr double contactTolerance = 1e-9;
  for (std::size_t i = 0u; i < getSoftPointCount(softPoints); ++i) {
    const Eigen::Vector3d& localVertex = getSoftLocalPosition(softPoints, i);
    const double signedDist = localNormal.dot(localVertex) + localOffset;
    if (signedDist > contactTolerance)
      continue;

    const int faceIndex = getSoftFirstFace(softPoints, i);
    if (faceIndex < 0)
      continue;

    Contact contact;
    contact.collisionObject1 = object1;
    contact.collisionObject2 = object2;
    contact.point = transformPoint(transform1, localVertex);
    contact.normal = worldNormal;
    contact.penetrationDepth = std::max(0.0, -signedDist);
    contact.triID1 = faceIndex;
    result.addContact(contact);
    ++numContacts;
  }

  return numContacts;
}

//==============================================================================
int collideSphereSoftMesh(
    DARTCollisionObject* object1,
    DARTCollisionObject* object2,
    double sphereRadius,
    const Eigen::Isometry3d& transform1,
    const dynamics::SoftMeshShape* softMesh,
    const Eigen::Isometry3d& transform2,
    bool enableFaceInteriorContacts,
    CollisionResult& result)
{
  const auto* softBodyNode = softMesh ? softMesh->getSoftBodyNode() : nullptr;
  if (softBodyNode == nullptr)
    return 0;

  const RelativeTransformView softToSphere
      = makeRelativeTransformView(transform1, transform2);
  const auto softPoints = makeSoftPointCacheView(object2, softBodyNode);
  const auto numSoftPoints = getSoftPointCount(softPoints);
  constexpr double contactTolerance = 1e-9;
  const double contactRadius = sphereRadius + contactTolerance;
  const double contactRadiusSquared = contactRadius * contactRadius;
  auto numContacts = 0;
  for (std::size_t i = 0u; i < numSoftPoints; ++i) {
    const Eigen::Vector3d& localVertex = getSoftLocalPosition(softPoints, i);
    const Eigen::Vector3d pointInSphere = softToSphere.apply(localVertex);
    const double distanceSquared = pointInSphere.squaredNorm();
    if (distanceSquared > contactRadiusSquared)
      continue;

    const int faceIndex = getSoftFirstFace(softPoints, i);
    if (faceIndex < 0)
      continue;

    const double distance = std::sqrt(distanceSquared);
    Eigen::Vector3d normal;
    if (distance > kCollisionEps)
      normal = transform1.linear() * (-pointInSphere / distance);
    else
      normal.setZero();

    Contact contact;
    contact.collisionObject1 = object1;
    contact.collisionObject2 = object2;
    contact.point = transformPoint(transform2, localVertex);
    contact.normal = normal;
    contact.penetrationDepth = std::max(0.0, sphereRadius - distance);
    contact.triID2 = faceIndex;
    result.addContact(contact);
    ++numContacts;
  }

  if (enableFaceInteriorContacts) {
    numContacts += addSphereSoftFaceInteriorContacts(
        object1,
        object2,
        sphereRadius,
        transform1,
        object2,
        transform2,
        true,
        result);
  }

  return numContacts;
}

//==============================================================================
int collideSoftMeshSphere(
    DARTCollisionObject* object1,
    DARTCollisionObject* object2,
    const dynamics::SoftMeshShape* softMesh,
    const Eigen::Isometry3d& transform1,
    double sphereRadius,
    const Eigen::Isometry3d& transform2,
    bool enableFaceInteriorContacts,
    CollisionResult& result)
{
  const auto* softBodyNode = softMesh ? softMesh->getSoftBodyNode() : nullptr;
  if (softBodyNode == nullptr)
    return 0;

  const RelativeTransformView softToSphere
      = makeRelativeTransformView(transform2, transform1);
  const auto softPoints = makeSoftPointCacheView(object1, softBodyNode);
  const auto numSoftPoints = getSoftPointCount(softPoints);
  constexpr double contactTolerance = 1e-9;
  const double contactRadius = sphereRadius + contactTolerance;
  const double contactRadiusSquared = contactRadius * contactRadius;
  auto numContacts = 0;
  for (std::size_t i = 0u; i < numSoftPoints; ++i) {
    const Eigen::Vector3d& localVertex = getSoftLocalPosition(softPoints, i);
    const Eigen::Vector3d pointInSphere = softToSphere.apply(localVertex);
    const double distanceSquared = pointInSphere.squaredNorm();
    if (distanceSquared > contactRadiusSquared)
      continue;

    const int faceIndex = getSoftFirstFace(softPoints, i);
    if (faceIndex < 0)
      continue;

    const double distance = std::sqrt(distanceSquared);
    Eigen::Vector3d normal;
    if (distance > kCollisionEps)
      normal = transform2.linear() * (pointInSphere / distance);
    else
      normal.setZero();

    Contact contact;
    contact.collisionObject1 = object1;
    contact.collisionObject2 = object2;
    contact.point = transformPoint(transform1, localVertex);
    contact.normal = normal;
    contact.penetrationDepth = std::max(0.0, sphereRadius - distance);
    contact.triID1 = faceIndex;
    result.addContact(contact);
    ++numContacts;
  }

  if (enableFaceInteriorContacts) {
    numContacts += addSphereSoftFaceInteriorContacts(
        object1,
        object2,
        sphereRadius,
        transform2,
        object1,
        transform1,
        false,
        result);
  }

  return numContacts;
}

//==============================================================================
double computeEllipsoidPointPenetrationDepth(
    const Eigen::Vector3d& pointInEllipsoid,
    const Eigen::Vector3d& radii,
    double normalizedDistance)
{
  if (normalizedDistance <= kCollisionEps)
    return radii.minCoeff();

  const Eigen::Vector3d surfacePoint = pointInEllipsoid / normalizedDistance;
  return (surfacePoint - pointInEllipsoid).norm();
}

//==============================================================================
int collideEllipsoidSoftMesh(
    DARTCollisionObject* object1,
    DARTCollisionObject* object2,
    const Eigen::Vector3d& ellipsoidRadii,
    const Eigen::Isometry3d& transform1,
    const dynamics::SoftMeshShape* softMesh,
    const Eigen::Isometry3d& transform2,
    CollisionResult& result)
{
  const auto* softBodyNode = softMesh ? softMesh->getSoftBodyNode() : nullptr;
  if (softBodyNode == nullptr)
    return 0;

  const Eigen::Vector3d invRadii = ellipsoidRadii.cwiseInverse();
  const Eigen::Vector3d invRadiiSq = invRadii.cwiseProduct(invRadii);
  const RelativeTransformView softToEllipsoid
      = makeRelativeTransformView(transform1, transform2);

  const auto softPoints = makeSoftPointCacheView(object2, softBodyNode);
  const auto numSoftPoints = getSoftPointCount(softPoints);
  constexpr double contactTolerance = 1e-9;
  auto numContacts = 0;
  for (std::size_t i = 0u; i < numSoftPoints; ++i) {
    const Eigen::Vector3d& localVertex = getSoftLocalPosition(softPoints, i);
    const Eigen::Vector3d pointInEllipsoid = softToEllipsoid.apply(localVertex);
    const Eigen::Vector3d scaledPoint = pointInEllipsoid.cwiseProduct(invRadii);
    const double normalizedDistance = scaledPoint.norm();
    if (normalizedDistance > 1.0 + contactTolerance)
      continue;

    const int faceIndex = getSoftFirstFace(softPoints, i);
    if (faceIndex < 0)
      continue;

    const Eigen::Vector3d gradient = pointInEllipsoid.cwiseProduct(invRadiiSq);
    const double gradientNorm = gradient.norm();
    Eigen::Vector3d normal;
    if (gradientNorm > kCollisionEps)
      normal = -(transform1.linear() * (gradient / gradientNorm));
    else
      normal.setZero();

    Contact contact;
    contact.collisionObject1 = object1;
    contact.collisionObject2 = object2;
    contact.point = transformPoint(transform2, localVertex);
    contact.normal = normal;
    contact.penetrationDepth = computeEllipsoidPointPenetrationDepth(
        pointInEllipsoid, ellipsoidRadii, normalizedDistance);
    contact.triID2 = faceIndex;
    result.addContact(contact);
    ++numContacts;
  }

  return numContacts;
}

//==============================================================================
int collideSoftMeshEllipsoid(
    DARTCollisionObject* object1,
    DARTCollisionObject* object2,
    const dynamics::SoftMeshShape* softMesh,
    const Eigen::Isometry3d& transform1,
    const Eigen::Vector3d& ellipsoidRadii,
    const Eigen::Isometry3d& transform2,
    CollisionResult& result)
{
  const auto* softBodyNode = softMesh ? softMesh->getSoftBodyNode() : nullptr;
  if (softBodyNode == nullptr)
    return 0;

  const Eigen::Vector3d invRadii = ellipsoidRadii.cwiseInverse();
  const Eigen::Vector3d invRadiiSq = invRadii.cwiseProduct(invRadii);
  const RelativeTransformView softToEllipsoid
      = makeRelativeTransformView(transform2, transform1);

  const auto softPoints = makeSoftPointCacheView(object1, softBodyNode);
  const auto numSoftPoints = getSoftPointCount(softPoints);
  constexpr double contactTolerance = 1e-9;
  auto numContacts = 0;
  for (std::size_t i = 0u; i < numSoftPoints; ++i) {
    const Eigen::Vector3d& localVertex = getSoftLocalPosition(softPoints, i);
    const Eigen::Vector3d pointInEllipsoid = softToEllipsoid.apply(localVertex);
    const Eigen::Vector3d scaledPoint = pointInEllipsoid.cwiseProduct(invRadii);
    const double normalizedDistance = scaledPoint.norm();
    if (normalizedDistance > 1.0 + contactTolerance)
      continue;

    const int faceIndex = getSoftFirstFace(softPoints, i);
    if (faceIndex < 0)
      continue;

    const Eigen::Vector3d gradient = pointInEllipsoid.cwiseProduct(invRadiiSq);
    const double gradientNorm = gradient.norm();
    Eigen::Vector3d normal;
    if (gradientNorm > kCollisionEps)
      normal = transform2.linear() * (gradient / gradientNorm);
    else
      normal.setZero();

    Contact contact;
    contact.collisionObject1 = object1;
    contact.collisionObject2 = object2;
    contact.point = transformPoint(transform1, localVertex);
    contact.normal = normal;
    contact.penetrationDepth = computeEllipsoidPointPenetrationDepth(
        pointInEllipsoid, ellipsoidRadii, normalizedDistance);
    contact.triID1 = faceIndex;
    result.addContact(contact);
    ++numContacts;
  }

  return numContacts;
}

//==============================================================================
struct SoftPointFaceContact
{
  bool found = false;
  Eigen::Vector3d point;
  Eigen::Vector3d normalFromPointBodyToFaceBody;
  double penetrationDepth = 0.0;
  double absSeparation = std::numeric_limits<double>::infinity();
  int pointFaceIndex = -1;
  int faceIndex = -1;
};

//==============================================================================
bool projectPointInsideCachedTriangle(
    const Eigen::Vector3d& point,
    const DARTCollisionObject::CachedSoftFace& face,
    double signedDistance)
{
  if (!face.valid)
    return false;

  const double aabbExpansion = std::abs(signedDistance) + kCollisionEps;
  if (point[0] < face.boundsMin[0] - aabbExpansion
      || point[0] > face.boundsMax[0] + aabbExpansion
      || point[1] < face.boundsMin[1] - aabbExpansion
      || point[1] > face.boundsMax[1] + aabbExpansion
      || point[2] < face.boundsMin[2] - aabbExpansion
      || point[2] > face.boundsMax[2] + aabbExpansion) {
    return false;
  }

  const Eigen::Vector3d projectedPoint = point - signedDistance * face.normal;
  const Eigen::Vector3d projectedEdge = projectedPoint - face.a;
  const double d20 = projectedEdge.dot(face.edge0);
  const double d21 = projectedEdge.dot(face.edge1);
  const double v = (face.d11 * d20 - face.d01 * d21) / face.denom;
  const double w = (face.d00 * d21 - face.d01 * d20) / face.denom;
  const double u = 1.0 - v - w;
  constexpr double barycentricTolerance = 1e-9;
  return u >= -barycentricTolerance && v >= -barycentricTolerance
         && w >= -barycentricTolerance;
}

//==============================================================================
double distanceSquaredToAabb(
    const Eigen::Vector3d& point,
    const Eigen::Vector3d& boundsMin,
    const Eigen::Vector3d& boundsMax)
{
  double distanceSquared = 0.0;
  for (int axis = 0; axis < 3; ++axis) {
    double distance = 0.0;
    if (point[axis] < boundsMin[axis])
      distance = boundsMin[axis] - point[axis];
    else if (point[axis] > boundsMax[axis])
      distance = point[axis] - boundsMax[axis];

    distanceSquared += distance * distance;
  }

  return distanceSquared;
}

//==============================================================================
Eigen::Vector3d closestPointOnCachedTriangle(
    const Eigen::Vector3d& point,
    const DARTCollisionObject::CachedSoftFace& face)
{
  const Eigen::Vector3d& a = face.a;
  const Eigen::Vector3d& ab = face.edge0;
  const Eigen::Vector3d& ac = face.edge1;
  const Eigen::Vector3d ap = point - a;
  const double d1 = ab.dot(ap);
  const double d2 = ac.dot(ap);
  if (d1 <= 0.0 && d2 <= 0.0)
    return a;

  const Eigen::Vector3d b = a + ab;
  const Eigen::Vector3d bp = point - b;
  const double d3 = ab.dot(bp);
  const double d4 = ac.dot(bp);
  if (d3 >= 0.0 && d4 <= d3)
    return b;

  const double vc = d1 * d4 - d3 * d2;
  if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
    const double v = d1 / (d1 - d3);
    return a + v * ab;
  }

  const Eigen::Vector3d c = a + ac;
  const Eigen::Vector3d cp = point - c;
  const double d5 = ab.dot(cp);
  const double d6 = ac.dot(cp);
  if (d6 >= 0.0 && d5 <= d6)
    return c;

  const double vb = d5 * d2 - d1 * d6;
  if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
    const double w = d2 / (d2 - d6);
    return a + w * ac;
  }

  const double va = d3 * d6 - d5 * d4;
  if (va <= 0.0 && d4 - d3 >= 0.0 && d5 - d6 >= 0.0) {
    const double w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
    return b + w * (c - b);
  }

  const double denominator = 1.0 / (va + vb + vc);
  const double v = vb * denominator;
  const double w = vc * denominator;
  return a + v * ab + w * ac;
}

//==============================================================================
template <typename NodeOverlaps, typename VisitFace>
void visitCachedSoftFaces(
    const DARTCollisionObject* softObject,
    NodeOverlaps&& nodeOverlaps,
    VisitFace&& visitFace)
{
  const auto& faces = softObject->getCachedSoftFaces();
  const auto& nodes = softObject->getCachedSoftFaceBvhNodes();
  const auto& indices = softObject->getCachedSoftFaceBvhIndices();
  if (nodes.empty() || indices.empty()) {
    for (std::size_t faceIndex = 0u; faceIndex < faces.size(); ++faceIndex)
      visitFace(faceIndex);
    return;
  }

  const auto visitNode = [&](auto&& self, int nodeIndex) -> void {
    const auto& node = nodes[static_cast<std::size_t>(nodeIndex)];
    if (!nodeOverlaps(node.boundsMin, node.boundsMax))
      return;

    if (node.left < 0 && node.right < 0) {
      for (int i = 0; i < node.count; ++i) {
        const std::size_t cursor = static_cast<std::size_t>(node.first + i);
        const int faceIndex = indices[cursor];
        if (faceIndex >= 0)
          visitFace(static_cast<std::size_t>(faceIndex));
      }
      return;
    }

    self(self, node.left);
    self(self, node.right);
  };

  visitNode(visitNode, 0);
}

//==============================================================================
bool cachedFaceHasSphereVertexContact(
    const DARTCollisionObject::CachedSoftFace& face,
    const std::vector<Eigen::Vector3d>& vertices,
    const Eigen::Vector3d& sphereCenter,
    double contactRadiusSquared)
{
  for (int i = 0; i < 3; ++i) {
    const auto vertexIndex = static_cast<std::size_t>(face.indices[i]);
    if ((vertices[vertexIndex] - sphereCenter).squaredNorm()
        <= contactRadiusSquared) {
      return true;
    }
  }

  return false;
}

//==============================================================================
int addSphereSoftFaceInteriorContacts(
    DARTCollisionObject* object1,
    DARTCollisionObject* object2,
    double sphereRadius,
    const Eigen::Isometry3d& sphereTransform,
    DARTCollisionObject* softObject,
    const Eigen::Isometry3d& softTransform,
    bool softIsObject2,
    CollisionResult& result)
{
  const auto& faces = softObject->getCachedSoftFaces();
  const auto& vertices = softObject->getCachedSoftLocalVertices();
  if (faces.empty() || vertices.empty())
    return 0;

  constexpr double contactTolerance = 1e-9;
  const double contactRadius = sphereRadius + contactTolerance;
  const double contactRadiusSquared = contactRadius * contactRadius;
  const Eigen::Vector3d sphereCenter
      = softTransform.linear().transpose()
        * (sphereTransform.translation() - softTransform.translation());
  const Eigen::Matrix3d& softRotation = softTransform.linear();
  int numContacts = 0;

  visitCachedSoftFaces(
      softObject,
      [&](const Eigen::Vector3d& boundsMin, const Eigen::Vector3d& boundsMax) {
        return distanceSquaredToAabb(sphereCenter, boundsMin, boundsMax)
               <= contactRadiusSquared;
      },
      [&](std::size_t faceIndex) {
        if (faceIndex >= faces.size())
          return;

        const auto& face = faces[faceIndex];
        if (!face.valid
            || cachedFaceHasSphereVertexContact(
                face, vertices, sphereCenter, contactRadiusSquared)) {
          return;
        }

        const Eigen::Vector3d closest
            = closestPointOnCachedTriangle(sphereCenter, face);
        const Eigen::Vector3d closestToCenter = sphereCenter - closest;
        const double distanceSquared = closestToCenter.squaredNorm();
        if (distanceSquared > contactRadiusSquared)
          return;

        const double distance = std::sqrt(distanceSquared);
        Eigen::Vector3d normalInSoft;
        if (distance > kCollisionEps) {
          normalInSoft = closestToCenter / distance;
        } else {
          const double centerDistance
              = face.normal.dot(sphereCenter) - face.planeOffset;
          normalInSoft = centerDistance >= 0.0 ? face.normal : -face.normal;
        }

        Contact contact;
        contact.collisionObject1 = object1;
        contact.collisionObject2 = object2;
        contact.point = transformPoint(softTransform, closest);
        contact.normal
            = (softIsObject2 ? 1.0 : -1.0) * (softRotation * normalInSoft);
        contact.penetrationDepth = std::max(0.0, sphereRadius - distance);
        if (softIsObject2)
          contact.triID2 = static_cast<int>(faceIndex);
        else
          contact.triID1 = static_cast<int>(faceIndex);
        result.addContact(contact);
        ++numContacts;
      });

  return numContacts;
}

//==============================================================================
bool cachedFaceHasBoxVertexContact(
    const DARTCollisionObject::CachedSoftFace& face,
    const std::vector<Eigen::Vector3d>& vertices,
    const RelativeTransformView& softToBox,
    const Eigen::Vector3d& halfExtents)
{
  constexpr double contactTolerance = 1e-9;
  for (int i = 0; i < 3; ++i) {
    const auto vertexIndex = static_cast<std::size_t>(face.indices[i]);
    const Eigen::Vector3d pointInBox = softToBox.apply(vertices[vertexIndex]);
    if ((pointInBox.array().abs() <= (halfExtents.array() + contactTolerance))
            .all()) {
      return true;
    }
  }

  return false;
}

//==============================================================================
int addBoxSoftFaceInteriorContacts(
    DARTCollisionObject* object1,
    DARTCollisionObject* object2,
    const Eigen::Vector3d& boxSize,
    const Eigen::Isometry3d& boxTransform,
    DARTCollisionObject* softObject,
    const Eigen::Isometry3d& softTransform,
    bool softIsObject2,
    CollisionResult& result)
{
  const auto& faces = softObject->getCachedSoftFaces();
  const auto& vertices = softObject->getCachedSoftLocalVertices();
  if (faces.empty() || vertices.empty())
    return 0;

  const Eigen::Vector3d halfExtents = 0.5 * boxSize;
  const RelativeTransformView boxToSoft
      = makeRelativeTransformView(softTransform, boxTransform);
  const RelativeTransformView softToBox
      = makeRelativeTransformView(boxTransform, softTransform);
  std::array<Eigen::Vector3d, 20> features;
  Eigen::Vector3d queryMin
      = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector3d queryMax
      = Eigen::Vector3d::Constant(-std::numeric_limits<double>::infinity());
  std::size_t featureIndex = 0u;
  for (int corner = 0; corner < 8; ++corner) {
    const Eigen::Vector3d boxPoint(
        (corner & 1) ? halfExtents[0] : -halfExtents[0],
        (corner & 2) ? halfExtents[1] : -halfExtents[1],
        (corner & 4) ? halfExtents[2] : -halfExtents[2]);
    features[featureIndex] = boxToSoft.apply(boxPoint);
    queryMin = queryMin.cwiseMin(features[featureIndex]);
    queryMax = queryMax.cwiseMax(features[featureIndex]);
    ++featureIndex;
  }
  for (int edgeAxis = 0; edgeAxis < 3; ++edgeAxis) {
    const int axis1 = (edgeAxis + 1) % 3;
    const int axis2 = (edgeAxis + 2) % 3;
    for (int signs = 0; signs < 4; ++signs) {
      Eigen::Vector3d boxPoint = Eigen::Vector3d::Zero();
      boxPoint[axis1] = (signs & 1) ? halfExtents[axis1] : -halfExtents[axis1];
      boxPoint[axis2] = (signs & 2) ? halfExtents[axis2] : -halfExtents[axis2];
      features[featureIndex++] = boxToSoft.apply(boxPoint);
    }
  }

  const Eigen::Vector3d boxCenter = boxToSoft.translation;
  const Eigen::Matrix3d& softRotation = softTransform.linear();
  constexpr double contactTolerance = 1e-9;
  int numContacts = 0;

  visitCachedSoftFaces(
      softObject,
      [&](const Eigen::Vector3d& boundsMin, const Eigen::Vector3d& boundsMax) {
        return (queryMin.array() <= boundsMax.array()).all()
               && (queryMax.array() >= boundsMin.array()).all();
      },
      [&](std::size_t faceIndex) {
        if (faceIndex >= faces.size())
          return;

        const auto& face = faces[faceIndex];
        if (!face.valid
            || cachedFaceHasBoxVertexContact(
                face, vertices, softToBox, halfExtents)) {
          return;
        }

        const double centerDistance
            = face.normal.dot(boxCenter) - face.planeOffset;
        const double side = centerDistance >= 0.0 ? 1.0 : -1.0;
        double bestDepth = -1.0;
        Eigen::Vector3d bestPoint = Eigen::Vector3d::Zero();
        for (const Eigen::Vector3d& feature : features) {
          const double signedDistance
              = face.normal.dot(feature) - face.planeOffset;
          const double separation = side * signedDistance;
          if (separation > contactTolerance
              || !projectPointInsideCachedTriangle(
                  feature, face, signedDistance)) {
            continue;
          }

          const Eigen::Vector3d closest
              = closestPointOnCachedTriangle(feature, face);
          const Eigen::Vector3d closestInBox = softToBox.apply(closest);
          if (!(closestInBox.array().abs()
                <= (halfExtents.array() + contactTolerance))
                   .all()) {
            continue;
          }

          const double depth = std::max(0.0, -separation);
          if (depth > bestDepth) {
            bestDepth = depth;
            bestPoint = closest;
          }
        }

        if (bestDepth < 0.0)
          return;

        Contact contact;
        contact.collisionObject1 = object1;
        contact.collisionObject2 = object2;
        contact.point = transformPoint(softTransform, bestPoint);
        contact.normal
            = (softIsObject2 ? side : -side) * (softRotation * face.normal);
        contact.penetrationDepth = bestDepth;
        if (softIsObject2)
          contact.triID2 = static_cast<int>(faceIndex);
        else
          contact.triID1 = static_cast<int>(faceIndex);
        result.addContact(contact);
        ++numContacts;
      });

  return numContacts;
}

//==============================================================================
bool addCachedSoftFaceCandidate(
    const std::vector<int>& pointFirstFaceByPointMass,
    const Eigen::Isometry3d& pointBodyTransform,
    const Eigen::Vector3d& pointLocal,
    const Eigen::Vector3d& pointInFace,
    const Eigen::Vector3d& pointBodyOriginInFace,
    const Eigen::Matrix3d& faceRotation,
    std::size_t pointMassIndex,
    const std::vector<DARTCollisionObject::CachedSoftFace>& faceFaces,
    std::size_t faceIndex,
    int& pointFaceIndex,
    bool& stopSearch,
    SoftPointFaceContact& best)
{
  if (faceIndex >= faceFaces.size())
    return false;

  const auto& face = faceFaces[faceIndex];
  if (!face.valid)
    return false;

  const double signedDistance = face.normal.dot(pointInFace) - face.planeOffset;
  const double centerSign
      = face.normal.dot(pointBodyOriginInFace) - face.planeOffset;
  const double side = centerSign >= 0.0 ? 1.0 : -1.0;
  const double separation = side * signedDistance;
  if (separation > kSoftContactShell)
    return false;

  if (!projectPointInsideCachedTriangle(pointInFace, face, signedDistance))
    return false;

  const double penetrationDepth = std::max(0.0, -separation);
  const double absSeparation = std::abs(separation);
  const int faceIndexAsInt = static_cast<int>(faceIndex);
  if (best.found) {
    const bool candidateIsPenetrating = penetrationDepth > 0.0;
    const bool bestIsPenetrating = best.penetrationDepth > 0.0;
    if (candidateIsPenetrating != bestIsPenetrating) {
      if (!candidateIsPenetrating)
        return false;
    } else if (absSeparation > best.absSeparation) {
      return false;
    } else if (
        absSeparation == best.absSeparation
        && faceIndexAsInt >= best.faceIndex) {
      return false;
    }
  }

  if (pointFaceIndex < 0) {
    if (pointMassIndex >= pointFirstFaceByPointMass.size()) {
      stopSearch = true;
      return false;
    }

    pointFaceIndex = pointFirstFaceByPointMass[pointMassIndex];
    if (pointFaceIndex < 0) {
      stopSearch = true;
      return false;
    }
  }

  best.found = true;
  best.point = transformPoint(pointBodyTransform, pointLocal);
  best.normalFromPointBodyToFaceBody = -side * (faceRotation * face.normal);
  best.penetrationDepth = penetrationDepth;
  best.absSeparation = absSeparation;
  best.pointFaceIndex = pointFaceIndex;
  best.faceIndex = faceIndexAsInt;
  return true;
}

//==============================================================================
bool findSoftPointFaceContact(
    const std::vector<Eigen::Vector3d>& pointVertices,
    const std::vector<int>& pointFirstFaceByPointMass,
    const Eigen::Isometry3d& pointBodyTransform,
    const RelativeTransformView& pointToFace,
    const Eigen::Vector3d& faceBoundsMin,
    const Eigen::Vector3d& faceBoundsMax,
    const Eigen::Vector3d& pointBodyOriginInFace,
    const Eigen::Matrix3d& faceRotation,
    std::size_t pointMassIndex,
    const std::vector<DARTCollisionObject::CachedSoftFace>& faceFaces,
    const std::vector<DARTCollisionObject::CachedSoftFaceBvhNode>& faceBvhNodes,
    const std::vector<int>& faceBvhIndices,
    SoftPointFaceContact& best)
{
  if (pointMassIndex >= pointVertices.size())
    return false;

  const Eigen::Vector3d& pointLocal = pointVertices[pointMassIndex];
  const Eigen::Vector3d pointInFace = pointToFace.apply(pointLocal);
  constexpr double boundsPadding = kSoftContactShell + kCollisionEps;
  if (pointInFace[0] < faceBoundsMin[0] - boundsPadding
      || pointInFace[0] > faceBoundsMax[0] + boundsPadding
      || pointInFace[1] < faceBoundsMin[1] - boundsPadding
      || pointInFace[1] > faceBoundsMax[1] + boundsPadding
      || pointInFace[2] < faceBoundsMin[2] - boundsPadding
      || pointInFace[2] > faceBoundsMax[2] + boundsPadding) {
    return false;
  }

  int pointFaceIndex = -1;
  bool stopSearch = false;

  if (faceBvhNodes.empty() || faceBvhIndices.empty()) {
    for (std::size_t faceIndex = 0u;
         faceIndex < faceFaces.size() && !stopSearch;
         ++faceIndex) {
      addCachedSoftFaceCandidate(
          pointFirstFaceByPointMass,
          pointBodyTransform,
          pointLocal,
          pointInFace,
          pointBodyOriginInFace,
          faceRotation,
          pointMassIndex,
          faceFaces,
          faceIndex,
          pointFaceIndex,
          stopSearch,
          best);
    }

    return best.found;
  }

  const auto shouldPruneNode =
      [&](const DARTCollisionObject::CachedSoftFaceBvhNode& node) {
        if (!best.found || best.penetrationDepth <= 0.0)
          return false;

        const double lowerBoundSquared = distanceSquaredToAabb(
            pointInFace, node.boundsMin, node.boundsMax);
        const double bestSquared = best.absSeparation * best.absSeparation;
        return lowerBoundSquared > bestSquared + kCollisionEps * kCollisionEps;
      };

  const auto visitNode = [&](auto&& self, int nodeIndex) -> void {
    if (stopSearch || nodeIndex < 0
        || static_cast<std::size_t>(nodeIndex) >= faceBvhNodes.size()) {
      return;
    }

    const auto& node = faceBvhNodes[static_cast<std::size_t>(nodeIndex)];
    if (shouldPruneNode(node))
      return;

    if (node.left < 0 && node.right < 0) {
      for (int i = 0; i < node.count && !stopSearch; ++i) {
        const int faceIndex
            = faceBvhIndices[static_cast<std::size_t>(node.first + i)];
        if (faceIndex < 0)
          continue;

        addCachedSoftFaceCandidate(
            pointFirstFaceByPointMass,
            pointBodyTransform,
            pointLocal,
            pointInFace,
            pointBodyOriginInFace,
            faceRotation,
            pointMassIndex,
            faceFaces,
            static_cast<std::size_t>(faceIndex),
            pointFaceIndex,
            stopSearch,
            best);
      }

      return;
    }

    const bool hasLeft
        = node.left >= 0
          && static_cast<std::size_t>(node.left) < faceBvhNodes.size();
    const bool hasRight
        = node.right >= 0
          && static_cast<std::size_t>(node.right) < faceBvhNodes.size();
    if (!hasLeft) {
      if (hasRight)
        self(self, node.right);
      return;
    }
    if (!hasRight) {
      self(self, node.left);
      return;
    }

    const auto& leftNode = faceBvhNodes[static_cast<std::size_t>(node.left)];
    const auto& rightNode = faceBvhNodes[static_cast<std::size_t>(node.right)];
    const double leftDistanceSquared = distanceSquaredToAabb(
        pointInFace, leftNode.boundsMin, leftNode.boundsMax);
    const double rightDistanceSquared = distanceSquaredToAabb(
        pointInFace, rightNode.boundsMin, rightNode.boundsMax);
    if (leftDistanceSquared <= rightDistanceSquared) {
      self(self, node.left);
      self(self, node.right);
    } else {
      self(self, node.right);
      self(self, node.left);
    }
  };

  visitNode(visitNode, 0);
  return best.found;
}

//==============================================================================
int addSoftPointFaceContacts(
    DARTCollisionObject* object1,
    DARTCollisionObject* object2,
    const DARTCollisionObject* dartObject1,
    const Eigen::Isometry3d& transform1,
    const DARTCollisionObject* dartObject2,
    const Eigen::Isometry3d& transform2,
    bool pointBodyIsObject2,
    CollisionResult& result)
{
  const Eigen::Isometry3d& faceTransform
      = pointBodyIsObject2 ? transform1 : transform2;
  const Eigen::Isometry3d& pointTransform
      = pointBodyIsObject2 ? transform2 : transform1;
  const auto* faceObject = pointBodyIsObject2 ? dartObject1 : dartObject2;
  const auto* pointObject = pointBodyIsObject2 ? dartObject2 : dartObject1;
  if (faceObject == nullptr || pointObject == nullptr)
    return 0;

  const auto& faceFaces = faceObject->getCachedSoftFaces();
  const auto& faceBvhNodes = faceObject->getCachedSoftFaceBvhNodes();
  const auto& faceBvhIndices = faceObject->getCachedSoftFaceBvhIndices();
  const auto& pointVertices = pointObject->getCachedSoftLocalVertices();
  const auto& pointFirstFaceByPointMass
      = pointObject->getCachedSoftFirstFaceByPointMass();

  const RelativeTransformView pointToFace
      = makeRelativeTransformView(faceTransform, pointTransform);
  const Eigen::Vector3d pointBodyOriginInFace
      = faceTransform.linear().transpose()
        * (pointTransform.translation() - faceTransform.translation());
  const Eigen::Matrix3d faceRotation = faceTransform.linear();

  Eigen::Vector3d faceLocalMin
      = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector3d faceLocalMax
      = Eigen::Vector3d::Constant(-std::numeric_limits<double>::infinity());
  for (const auto& point : faceObject->getCachedSoftLocalVertices()) {
    faceLocalMin = faceLocalMin.cwiseMin(point);
    faceLocalMax = faceLocalMax.cwiseMax(point);
  }
  if (faceObject->getCachedSoftLocalVertices().empty()) {
    faceLocalMin.setZero();
    faceLocalMax.setZero();
  }

  auto numContacts = 0;
  for (std::size_t i = 0u; i < pointVertices.size(); ++i) {
    SoftPointFaceContact candidate;
    if (!findSoftPointFaceContact(
            pointVertices,
            pointFirstFaceByPointMass,
            pointTransform,
            pointToFace,
            faceLocalMin,
            faceLocalMax,
            pointBodyOriginInFace,
            faceRotation,
            i,
            faceFaces,
            faceBvhNodes,
            faceBvhIndices,
            candidate)) {
      continue;
    }

    Contact contact;
    contact.collisionObject1 = object1;
    contact.collisionObject2 = object2;
    contact.point = candidate.point;
    contact.normal = pointBodyIsObject2
                         ? candidate.normalFromPointBodyToFaceBody
                         : -candidate.normalFromPointBodyToFaceBody;
    contact.penetrationDepth = candidate.penetrationDepth;
    if (pointBodyIsObject2) {
      contact.triID1 = candidate.faceIndex;
      contact.triID2 = candidate.pointFaceIndex;
    } else {
      contact.triID1 = candidate.pointFaceIndex;
      contact.triID2 = candidate.faceIndex;
    }
    result.addContact(contact);
    ++numContacts;
  }

  return numContacts;
}

//==============================================================================
int collideSoftMeshSoftMesh(
    DARTCollisionObject* object1,
    DARTCollisionObject* object2,
    const dynamics::SoftMeshShape* softMesh1,
    const Eigen::Isometry3d& transform1,
    const dynamics::SoftMeshShape* softMesh2,
    const Eigen::Isometry3d& transform2,
    CollisionResult& result)
{
  if (softMesh1 == nullptr || softMesh2 == nullptr
      || softMesh1->getSoftBodyNode() == nullptr
      || softMesh2->getSoftBodyNode() == nullptr) {
    return 0;
  }

  // TODO(DART 6.20): parallel soft-soft batching. The consolidated detector
  // streams broadphase visitor pairs instead of materializing the legacy
  // FiniteFinitePair list, so porting the batching faithfully would require a
  // broader visitor/scratch redesign. The serial path keeps the legacy contact
  // semantics intact.
  auto numContacts = addSoftPointFaceContacts(
      object1, object2, object1, transform1, object2, transform2, true, result);
  numContacts += addSoftPointFaceContacts(
      object1,
      object2,
      object1,
      transform1,
      object2,
      transform2,
      false,
      result);
  return numContacts;
}

//==============================================================================
bool findContainingBoxFace(
    const Eigen::Vector3d& localPoint,
    const Eigen::Vector3d& halfExtents,
    int& axis,
    double& distance)
{
  constexpr double contactTolerance = 1e-9;

  const double distance0 = halfExtents[0] - std::abs(localPoint[0]);
  if (distance0 < -contactTolerance)
    return false;

  const double distance1 = halfExtents[1] - std::abs(localPoint[1]);
  if (distance1 < -contactTolerance)
    return false;

  const double distance2 = halfExtents[2] - std::abs(localPoint[2]);
  if (distance2 < -contactTolerance)
    return false;

  axis = 0;
  distance = distance0;
  if (distance1 < distance) {
    axis = 1;
    distance = distance1;
  }
  if (distance2 < distance) {
    axis = 2;
    distance = distance2;
  }

  return true;
}

//==============================================================================
int collideBoxSoftMesh(
    DARTCollisionObject* object1,
    DARTCollisionObject* object2,
    const Eigen::Vector3d& size1,
    const Eigen::Isometry3d& transform1,
    const dynamics::SoftMeshShape* softMesh,
    const Eigen::Isometry3d& transform2,
    bool enableFaceInteriorContacts,
    CollisionResult& result)
{
  const auto* softBodyNode = softMesh ? softMesh->getSoftBodyNode() : nullptr;
  if (softBodyNode == nullptr)
    return 0;

  const Eigen::Vector3d halfExtents = 0.5 * size1;
  const RelativeTransformView softToBox
      = makeRelativeTransformView(transform1, transform2);
  const auto softPoints = makeSoftPointCacheView(object2, softBodyNode);
  const auto numSoftPoints = getSoftPointCount(softPoints);
  const Eigen::Matrix3d& boxRotation = transform1.linear();
  auto numContacts = 0;
  for (std::size_t i = 0u; i < numSoftPoints; ++i) {
    const Eigen::Vector3d& softLocalVertex
        = getSoftLocalPosition(softPoints, i);
    const Eigen::Vector3d localVertex = softToBox.apply(softLocalVertex);

    int axis = 0;
    double distance = 0.0;
    if (!findContainingBoxFace(localVertex, halfExtents, axis, distance))
      continue;

    const int faceIndex = getSoftFirstFace(softPoints, i);
    if (faceIndex < 0)
      continue;

    const double normalSign = localVertex[axis] >= 0.0 ? -1.0 : 1.0;

    Contact contact;
    contact.collisionObject1 = object1;
    contact.collisionObject2 = object2;
    contact.point = transformPoint(transform2, softLocalVertex);
    contact.normal = normalSign * boxRotation.col(axis);
    contact.penetrationDepth = std::max(0.0, distance);
    contact.triID2 = faceIndex;
    result.addContact(contact);
    ++numContacts;
  }

  if (enableFaceInteriorContacts) {
    numContacts += addBoxSoftFaceInteriorContacts(
        object1, object2, size1, transform1, object2, transform2, true, result);
  }

  return numContacts;
}

//==============================================================================
int collideSoftMeshBox(
    DARTCollisionObject* object1,
    DARTCollisionObject* object2,
    const dynamics::SoftMeshShape* softMesh,
    const Eigen::Isometry3d& transform1,
    const Eigen::Vector3d& size2,
    const Eigen::Isometry3d& transform2,
    bool enableFaceInteriorContacts,
    CollisionResult& result)
{
  const auto* softBodyNode = softMesh ? softMesh->getSoftBodyNode() : nullptr;
  if (softBodyNode == nullptr)
    return 0;

  const Eigen::Vector3d halfExtents = 0.5 * size2;
  const RelativeTransformView softToBox
      = makeRelativeTransformView(transform2, transform1);
  const auto softPoints = makeSoftPointCacheView(object1, softBodyNode);
  const auto numSoftPoints = getSoftPointCount(softPoints);
  const Eigen::Matrix3d& boxRotation = transform2.linear();
  auto numContacts = 0;
  for (std::size_t i = 0u; i < numSoftPoints; ++i) {
    const Eigen::Vector3d& softLocalVertex
        = getSoftLocalPosition(softPoints, i);
    const Eigen::Vector3d localVertex = softToBox.apply(softLocalVertex);

    int axis = 0;
    double distance = 0.0;
    if (!findContainingBoxFace(localVertex, halfExtents, axis, distance))
      continue;

    const int faceIndex = getSoftFirstFace(softPoints, i);
    if (faceIndex < 0)
      continue;

    const double normalSign = localVertex[axis] >= 0.0 ? 1.0 : -1.0;

    Contact contact;
    contact.collisionObject1 = object1;
    contact.collisionObject2 = object2;
    contact.point = transformPoint(transform1, softLocalVertex);
    contact.normal = normalSign * boxRotation.col(axis);
    contact.penetrationDepth = std::max(0.0, distance);
    contact.triID1 = faceIndex;
    result.addContact(contact);
    ++numContacts;
  }

  if (enableFaceInteriorContacts) {
    numContacts += addBoxSoftFaceInteriorContacts(
        object1,
        object2,
        size2,
        transform2,
        object1,
        transform1,
        false,
        result);
  }

  return numContacts;
}

//==============================================================================
int collideSoftShapes(
    DARTCollisionObject* object1,
    DARTCollisionObject* object2,
    const dynamics::Shape* shape1,
    const dynamics::Shape* shape2,
    const std::string& shapeType1,
    const std::string& shapeType2,
    const Eigen::Isometry3d& transform1,
    const Eigen::Isometry3d& transform2,
    bool enableFaceInteriorContacts,
    CollisionResult& result)
{
  if (dynamics::SphereShape::getStaticType() == shapeType1) {
    const auto* sphere1 = static_cast<const dynamics::SphereShape*>(shape1);

    if (dynamics::SoftMeshShape::getStaticType() == shapeType2) {
      const auto* softMesh2
          = static_cast<const dynamics::SoftMeshShape*>(shape2);
      return collideSphereSoftMesh(
          object1,
          object2,
          sphere1->getRadius(),
          transform1,
          softMesh2,
          transform2,
          enableFaceInteriorContacts,
          result);
    }
  } else if (dynamics::BoxShape::getStaticType() == shapeType1) {
    const auto* box1 = static_cast<const dynamics::BoxShape*>(shape1);

    if (dynamics::SoftMeshShape::getStaticType() == shapeType2) {
      const auto* softMesh2
          = static_cast<const dynamics::SoftMeshShape*>(shape2);
      return collideBoxSoftMesh(
          object1,
          object2,
          box1->getSize(),
          transform1,
          softMesh2,
          transform2,
          enableFaceInteriorContacts,
          result);
    }
  } else if (dynamics::EllipsoidShape::getStaticType() == shapeType1) {
    const auto* ellipsoid1
        = static_cast<const dynamics::EllipsoidShape*>(shape1);

    if (dynamics::SoftMeshShape::getStaticType() == shapeType2) {
      const auto* softMesh2
          = static_cast<const dynamics::SoftMeshShape*>(shape2);
      if (ellipsoid1->isSphere()) {
        return collideSphereSoftMesh(
            object1,
            object2,
            ellipsoid1->getRadii()[0],
            transform1,
            softMesh2,
            transform2,
            enableFaceInteriorContacts,
            result);
      }

      return collideEllipsoidSoftMesh(
          object1,
          object2,
          ellipsoid1->getRadii(),
          transform1,
          softMesh2,
          transform2,
          result);
    }
  } else if (dynamics::SoftMeshShape::getStaticType() == shapeType1) {
    const auto* softMesh1 = static_cast<const dynamics::SoftMeshShape*>(shape1);

    if (dynamics::SphereShape::getStaticType() == shapeType2) {
      const auto* sphere2 = static_cast<const dynamics::SphereShape*>(shape2);
      return collideSoftMeshSphere(
          object1,
          object2,
          softMesh1,
          transform1,
          sphere2->getRadius(),
          transform2,
          enableFaceInteriorContacts,
          result);
    } else if (dynamics::EllipsoidShape::getStaticType() == shapeType2) {
      const auto* ellipsoid2
          = static_cast<const dynamics::EllipsoidShape*>(shape2);
      if (ellipsoid2->isSphere()) {
        return collideSoftMeshSphere(
            object1,
            object2,
            softMesh1,
            transform1,
            ellipsoid2->getRadii()[0],
            transform2,
            enableFaceInteriorContacts,
            result);
      }

      return collideSoftMeshEllipsoid(
          object1,
          object2,
          softMesh1,
          transform1,
          ellipsoid2->getRadii(),
          transform2,
          result);
    } else if (dynamics::PlaneShape::getStaticType() == shapeType2) {
      const auto* plane2 = static_cast<const dynamics::PlaneShape*>(shape2);
      return collideSoftMeshPlane(
          object1,
          object2,
          softMesh1,
          transform1,
          plane2->getNormal(),
          plane2->getOffset(),
          transform2,
          result);
    } else if (dynamics::BoxShape::getStaticType() == shapeType2) {
      const auto* box2 = static_cast<const dynamics::BoxShape*>(shape2);
      return collideSoftMeshBox(
          object1,
          object2,
          softMesh1,
          transform1,
          box2->getSize(),
          transform2,
          enableFaceInteriorContacts,
          result);
    } else if (dynamics::SoftMeshShape::getStaticType() == shapeType2) {
      const auto* softMesh2
          = static_cast<const dynamics::SoftMeshShape*>(shape2);
      return collideSoftMeshSoftMesh(
          object1,
          object2,
          softMesh1,
          transform1,
          softMesh2,
          transform2,
          result);
    }
  } else if (dynamics::PlaneShape::getStaticType() == shapeType1) {
    const auto* plane1 = static_cast<const dynamics::PlaneShape*>(shape1);

    if (dynamics::SoftMeshShape::getStaticType() == shapeType2) {
      const auto* softMesh2
          = static_cast<const dynamics::SoftMeshShape*>(shape2);
      return collidePlaneSoftMesh(
          object1,
          object2,
          plane1->getNormal(),
          plane1->getOffset(),
          transform1,
          softMesh2,
          transform2,
          result);
    }
  }

  dterr << "[DARTCollisionDetector] Attempting to check for an "
        << "unsupported soft shape pair: [" << shapeType1 << "] - ["
        << shapeType2 << "]. Returning false.\n";

  return 0;
}

} // namespace

//==============================================================================
bool isSoftCollisionPair(
    const DARTCollisionObject* object1, const DARTCollisionObject* object2)
{
  return (object1 != nullptr
          && object1->getCachedShapeKind()
                 == DARTCollisionObject::CachedShapeKind::SoftMesh)
         || (object2 != nullptr
             && object2->getCachedShapeKind()
                    == DARTCollisionObject::CachedShapeKind::SoftMesh);
}

//==============================================================================
int collideSoftPair(
    DARTCollisionObject* object1,
    DARTCollisionObject* object2,
    bool enableFaceInteriorContacts,
    CollisionResult& result)
{
  if (object1 == nullptr || object2 == nullptr)
    return 0;

  const auto shape1 = object1->getShape();
  const auto shape2 = object2->getShape();
  if (!shape1 || !shape2)
    return 0;

  return collideSoftShapes(
      object1,
      object2,
      shape1.get(),
      shape2.get(),
      shape1->getType(),
      shape2->getType(),
      object1->getWorldTransformForCollision(),
      object2->getWorldTransformForCollision(),
      enableFaceInteriorContacts,
      result);
}

} // namespace detail
} // namespace collision
} // namespace dart
