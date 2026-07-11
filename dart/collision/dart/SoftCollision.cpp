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

  const auto softPoints = makeSoftPointCacheView(object2, softBodyNode);
  auto numContacts = 0;
  constexpr double contactTolerance = 1e-9;
  for (std::size_t i = 0u; i < getSoftPointCount(softPoints); ++i) {
    const Eigen::Vector3d worldVertex
        = transform2 * getSoftLocalPosition(softPoints, i);
    const double signedDist = worldNormal.dot(worldVertex - planePoint);
    if (signedDist > contactTolerance)
      continue;

    const int faceIndex = getSoftFirstFace(softPoints, i);
    if (faceIndex < 0)
      continue;

    Contact contact;
    contact.collisionObject1 = object1;
    contact.collisionObject2 = object2;
    contact.point = worldVertex;
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

  const auto softPoints = makeSoftPointCacheView(object1, softBodyNode);
  auto numContacts = 0;
  constexpr double contactTolerance = 1e-9;
  for (std::size_t i = 0u; i < getSoftPointCount(softPoints); ++i) {
    const Eigen::Vector3d worldVertex
        = transform1 * getSoftLocalPosition(softPoints, i);
    const double signedDist = worldNormal.dot(worldVertex - planePoint);
    if (signedDist > contactTolerance)
      continue;

    const int faceIndex = getSoftFirstFace(softPoints, i);
    if (faceIndex < 0)
      continue;

    Contact contact;
    contact.collisionObject1 = object1;
    contact.collisionObject2 = object2;
    contact.point = worldVertex;
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
    CollisionResult& result)
{
  const auto* softBodyNode = softMesh ? softMesh->getSoftBodyNode() : nullptr;
  if (softBodyNode == nullptr)
    return 0;

  const Eigen::Vector3d sphereCenter = transform1.translation();
  const auto softPoints = makeSoftPointCacheView(object2, softBodyNode);
  auto numContacts = 0;
  constexpr double contactTolerance = 1e-9;
  for (std::size_t i = 0u; i < getSoftPointCount(softPoints); ++i) {
    const Eigen::Vector3d worldVertex
        = transform2 * getSoftLocalPosition(softPoints, i);
    Eigen::Vector3d normal = sphereCenter - worldVertex;
    const double distance = normal.norm();
    if (distance > sphereRadius + contactTolerance)
      continue;

    const int faceIndex = getSoftFirstFace(softPoints, i);
    if (faceIndex < 0)
      continue;

    if (distance > kCollisionEps)
      normal /= distance;
    else
      normal.setZero();

    Contact contact;
    contact.collisionObject1 = object1;
    contact.collisionObject2 = object2;
    contact.point = worldVertex;
    contact.normal = normal;
    contact.penetrationDepth = std::max(0.0, sphereRadius - distance);
    contact.triID2 = faceIndex;
    result.addContact(contact);
    ++numContacts;
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
    CollisionResult& result)
{
  const auto* softBodyNode = softMesh ? softMesh->getSoftBodyNode() : nullptr;
  if (softBodyNode == nullptr)
    return 0;

  const Eigen::Vector3d sphereCenter = transform2.translation();
  const auto softPoints = makeSoftPointCacheView(object1, softBodyNode);
  auto numContacts = 0;
  constexpr double contactTolerance = 1e-9;
  for (std::size_t i = 0u; i < getSoftPointCount(softPoints); ++i) {
    const Eigen::Vector3d worldVertex
        = transform1 * getSoftLocalPosition(softPoints, i);
    Eigen::Vector3d normal = worldVertex - sphereCenter;
    const double distance = normal.norm();
    if (distance > sphereRadius + contactTolerance)
      continue;

    const int faceIndex = getSoftFirstFace(softPoints, i);
    if (faceIndex < 0)
      continue;

    if (distance > kCollisionEps)
      normal /= distance;
    else
      normal.setZero();

    Contact contact;
    contact.collisionObject1 = object1;
    contact.collisionObject2 = object2;
    contact.point = worldVertex;
    contact.normal = normal;
    contact.penetrationDepth = std::max(0.0, sphereRadius - distance);
    contact.triID1 = faceIndex;
    result.addContact(contact);
    ++numContacts;
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
  const Eigen::Isometry3d softToEllipsoid = transform1.inverse() * transform2;

  const auto softPoints = makeSoftPointCacheView(object2, softBodyNode);
  auto numContacts = 0;
  constexpr double contactTolerance = 1e-9;
  for (std::size_t i = 0u; i < getSoftPointCount(softPoints); ++i) {
    const Eigen::Vector3d& localVertex = getSoftLocalPosition(softPoints, i);
    const Eigen::Vector3d pointInEllipsoid = softToEllipsoid * localVertex;
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
    contact.point = transform2 * localVertex;
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
  const Eigen::Isometry3d softToEllipsoid = transform2.inverse() * transform1;

  const auto softPoints = makeSoftPointCacheView(object1, softBodyNode);
  auto numContacts = 0;
  constexpr double contactTolerance = 1e-9;
  for (std::size_t i = 0u; i < getSoftPointCount(softPoints); ++i) {
    const Eigen::Vector3d& localVertex = getSoftLocalPosition(softPoints, i);
    const Eigen::Vector3d pointInEllipsoid = softToEllipsoid * localVertex;
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
    contact.point = transform1 * localVertex;
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
  best.point = pointBodyTransform * pointLocal;
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
    const Eigen::Isometry3d& pointToFace,
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
  const Eigen::Vector3d pointInFace = pointToFace * pointLocal;
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

  const Eigen::Isometry3d worldToFace = faceTransform.inverse();
  const Eigen::Isometry3d pointToFace = worldToFace * pointTransform;
  const Eigen::Vector3d pointBodyOriginInFace
      = worldToFace * pointTransform.translation();
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
  axis = 0;
  distance = std::numeric_limits<double>::infinity();
  constexpr double contactTolerance = 1e-9;

  for (auto i = 0; i < 3; ++i) {
    const double faceDistance = halfExtents[i] - std::abs(localPoint[i]);
    if (faceDistance < -contactTolerance)
      return false;

    if (faceDistance < distance) {
      axis = i;
      distance = faceDistance;
    }
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
    CollisionResult& result)
{
  const auto* softBodyNode = softMesh ? softMesh->getSoftBodyNode() : nullptr;
  if (softBodyNode == nullptr)
    return 0;

  const Eigen::Vector3d halfExtents = 0.5 * size1;
  const Eigen::Isometry3d transform1Inv = transform1.inverse();
  const auto softPoints = makeSoftPointCacheView(object2, softBodyNode);
  auto numContacts = 0;
  for (std::size_t i = 0u; i < getSoftPointCount(softPoints); ++i) {
    const Eigen::Vector3d worldVertex
        = transform2 * getSoftLocalPosition(softPoints, i);
    const Eigen::Vector3d localVertex = transform1Inv * worldVertex;

    int axis = 0;
    double distance = 0.0;
    if (!findContainingBoxFace(localVertex, halfExtents, axis, distance))
      continue;

    const int faceIndex = getSoftFirstFace(softPoints, i);
    if (faceIndex < 0)
      continue;

    Eigen::Vector3d normal = Eigen::Vector3d::Zero();
    normal[axis] = localVertex[axis] >= 0.0 ? -1.0 : 1.0;

    Contact contact;
    contact.collisionObject1 = object1;
    contact.collisionObject2 = object2;
    contact.point = worldVertex;
    contact.normal = transform1.linear() * normal;
    contact.penetrationDepth = std::max(0.0, distance);
    contact.triID2 = faceIndex;
    result.addContact(contact);
    ++numContacts;
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
    CollisionResult& result)
{
  const auto* softBodyNode = softMesh ? softMesh->getSoftBodyNode() : nullptr;
  if (softBodyNode == nullptr)
    return 0;

  const Eigen::Vector3d halfExtents = 0.5 * size2;
  const Eigen::Isometry3d transform2Inv = transform2.inverse();
  const auto softPoints = makeSoftPointCacheView(object1, softBodyNode);
  auto numContacts = 0;
  for (std::size_t i = 0u; i < getSoftPointCount(softPoints); ++i) {
    const Eigen::Vector3d worldVertex
        = transform1 * getSoftLocalPosition(softPoints, i);
    const Eigen::Vector3d localVertex = transform2Inv * worldVertex;

    int axis = 0;
    double distance = 0.0;
    if (!findContainingBoxFace(localVertex, halfExtents, axis, distance))
      continue;

    const int faceIndex = getSoftFirstFace(softPoints, i);
    if (faceIndex < 0)
      continue;

    Eigen::Vector3d normal = Eigen::Vector3d::Zero();
    normal[axis] = localVertex[axis] >= 0.0 ? 1.0 : -1.0;

    Contact contact;
    contact.collisionObject1 = object1;
    contact.collisionObject2 = object2;
    contact.point = worldVertex;
    contact.normal = transform2.linear() * normal;
    contact.penetrationDepth = std::max(0.0, distance);
    contact.triID1 = faceIndex;
    result.addContact(contact);
    ++numContacts;
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
  return (object1 != nullptr && object1->isSoftMeshShape())
         || (object2 != nullptr && object2->isSoftMeshShape());
}

//==============================================================================
int collideSoftPair(
    DARTCollisionObject* object1,
    DARTCollisionObject* object2,
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
      object1->getNativeTransform(),
      object2->getNativeTransform(),
      result);
}

} // namespace detail
} // namespace collision
} // namespace dart
