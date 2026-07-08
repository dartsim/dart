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

#include <dart/collision/native/narrow_phase/convex_convex.hpp>
#include <dart/collision/native/narrow_phase/mesh_mesh.hpp>

#include <algorithm>
#include <array>
#include <limits>
#include <stack>
#include <stdexcept>
#include <vector>

#include <cmath>

namespace dart::collision::native {

namespace {

constexpr double kEpsilon = 1e-10;

struct LocalTriangle
{
  Eigen::Vector3d v0;
  Eigen::Vector3d v1;
  Eigen::Vector3d v2;

  [[nodiscard]] Eigen::Vector3d centroid() const
  {
    return (v0 + v1 + v2) / 3.0;
  }

  [[nodiscard]] Eigen::Vector3d edge(int i) const
  {
    switch (i) {
      case 0:
        return v1 - v0;
      case 1:
        return v2 - v1;
      default:
        return v0 - v2;
    }
  }

  [[nodiscard]] Eigen::Vector3d normalRaw() const
  {
    return (v1 - v0).cross(v2 - v0);
  }
};

struct SegmentClosestResult
{
  Eigen::Vector3d point1;
  Eigen::Vector3d point2;
  double s = 0.0;
  double t = 0.0;
  double distSq = std::numeric_limits<double>::max();
};

struct TriIntersectionResult
{
  bool intersect = false;
  Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
  double depth = 0.0;
  std::vector<Eigen::Vector3d> contacts;
};

bool isPrimitiveShape(ShapeType type)
{
  switch (type) {
    case ShapeType::Sphere:
    case ShapeType::Box:
    case ShapeType::Capsule:
    case ShapeType::Cylinder:
    case ShapeType::Convex:
      return true;
    default:
      return false;
  }
}

Eigen::Vector3d closestPointOnTriangle(
    const Eigen::Vector3d& point,
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c)
{
  const Eigen::Vector3d ab = b - a;
  const Eigen::Vector3d ac = c - a;
  const Eigen::Vector3d ap = point - a;

  const double d1 = ab.dot(ap);
  const double d2 = ac.dot(ap);
  if (d1 <= 0.0 && d2 <= 0.0) {
    return a;
  }

  const Eigen::Vector3d bp = point - b;
  const double d3 = ab.dot(bp);
  const double d4 = ac.dot(bp);
  if (d3 >= 0.0 && d4 <= d3) {
    return b;
  }

  const double vc = d1 * d4 - d3 * d2;
  if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
    const double v = d1 / (d1 - d3);
    return a + v * ab;
  }

  const Eigen::Vector3d cp = point - c;
  const double d5 = ab.dot(cp);
  const double d6 = ac.dot(cp);
  if (d6 >= 0.0 && d5 <= d6) {
    return c;
  }

  const double vb = d5 * d2 - d1 * d6;
  if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
    const double w = d2 / (d2 - d6);
    return a + w * ac;
  }

  const double va = d3 * d6 - d5 * d4;
  if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0) {
    const double w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
    return b + w * (c - b);
  }

  const double denom = 1.0 / (va + vb + vc);
  const double v = vb * denom;
  const double w = vc * denom;
  return a + ab * v + ac * w;
}

bool collideSphereTriangle(
    const SphereShape& sphere,
    const Eigen::Vector3d& sphereCenter,
    const std::array<Eigen::Vector3d, 3>& triVertices,
    CollisionResult& result,
    const CollisionOption& option)
{
  if (result.numContacts() >= option.maxNumContacts) {
    return false;
  }

  const Eigen::Vector3d closest = closestPointOnTriangle(
      sphereCenter, triVertices[0], triVertices[1], triVertices[2]);
  const Eigen::Vector3d sphereToTriangle = closest - sphereCenter;
  const double distSq = sphereToTriangle.squaredNorm();
  const double radius = sphere.getRadius();
  if (distSq > radius * radius) {
    return false;
  }

  Eigen::Vector3d normal;
  double distance = 0.0;
  if (distSq > kEpsilon * kEpsilon) {
    distance = std::sqrt(distSq);
    normal = sphereToTriangle / distance;
  } else {
    normal = (triVertices[1] - triVertices[0])
                 .cross(triVertices[2] - triVertices[0]);
    if (normal.squaredNorm() < kEpsilon * kEpsilon) {
      normal = Eigen::Vector3d::UnitZ();
    } else {
      normal.normalize();
      const Eigen::Vector3d triCenter
          = (triVertices[0] + triVertices[1] + triVertices[2]) / 3.0;
      if ((sphereCenter - triCenter).dot(normal) > 0.0) {
        normal = -normal;
      }
    }
  }

  ContactPoint contact;
  contact.position = closest;
  contact.normal = normal;
  contact.depth = radius - distance;
  result.addContact(contact);
  return true;
}

SupportFunction makePrimitiveSupportFunction(
    const Shape& shape, const Eigen::Isometry3d& transform)
{
  switch (shape.getType()) {
    case ShapeType::Sphere:
      return makeSphereSupportFunction(
          static_cast<const SphereShape&>(shape), transform);
    case ShapeType::Box:
      return makeBoxSupportFunction(
          static_cast<const BoxShape&>(shape), transform);
    case ShapeType::Capsule:
      return makeCapsuleSupportFunction(
          static_cast<const CapsuleShape&>(shape), transform);
    case ShapeType::Cylinder:
      return makeCylinderSupportFunction(
          static_cast<const CylinderShape&>(shape), transform);
    case ShapeType::Convex:
      return makeConvexSupportFunction(
          static_cast<const ConvexShape&>(shape), transform);
    default:
      return {};
  }
}

Eigen::Vector3d primitiveCenter(
    const Shape& shape, const Eigen::Isometry3d& transform)
{
  if (shape.getType() == ShapeType::Convex) {
    const auto& convex = static_cast<const ConvexShape&>(shape);
    const auto& vertices = convex.getVertices();
    if (!vertices.empty()) {
      Eigen::Vector3d sum = Eigen::Vector3d::Zero();
      for (const auto& vertex : vertices) {
        sum += vertex;
      }
      return transform * (sum / static_cast<double>(vertices.size()));
    }
  }

  return transform.translation();
}

std::vector<int> makeVertexTriangleFeatureMap(const MeshShape& mesh)
{
  std::vector<int> featureIndices(mesh.getVertices().size(), -1);
  const auto& triangles = mesh.getTriangles();
  for (std::size_t triIndex = 0; triIndex < triangles.size(); ++triIndex) {
    const auto& triangle = triangles[triIndex];
    for (int i = 0; i < 3; ++i) {
      const int vertexIndex = triangle[i];
      if (vertexIndex < 0) {
        continue;
      }

      const auto index = static_cast<std::size_t>(vertexIndex);
      if (index < featureIndices.size() && featureIndices[index] < 0) {
        featureIndices[index] = static_cast<int>(triIndex);
      }
    }
  }

  return featureIndices;
}

void setMeshTriangleFeature(ContactPoint& contact, int triangleIndex)
{
  if (triangleIndex < 0) {
    return;
  }

  // These one-mesh helpers are dispatched with the mesh on either public
  // collision-object side, so mirror the face id into both feature slots.
  contact.featureIndex1 = triangleIndex;
  contact.featureIndex2 = triangleIndex;
}

void appendContactsWithMeshTriangleFeature(
    CollisionResult& result,
    const CollisionResult& contacts,
    int triangleIndex,
    bool flipNormals = false)
{
  const auto numContacts = contacts.numContacts();
  for (std::size_t i = 0; i < numContacts; ++i) {
    ContactPoint contact = contacts.getContact(i);
    if (flipNormals) {
      contact.normal = -contact.normal;
    }
    setMeshTriangleFeature(contact, triangleIndex);
    result.addContact(contact);
  }
}

struct FaceContactCandidate
{
  Eigen::Vector3d point;
  double depth = 0.0;
  int featureIndex = -1;
};

void addContactCandidate(
    std::vector<FaceContactCandidate>& contacts,
    std::size_t contactLimit,
    const FaceContactCandidate& candidate)
{
  const auto insertPos = std::lower_bound(
      contacts.begin(),
      contacts.end(),
      candidate,
      [](const FaceContactCandidate& lhs, const FaceContactCandidate& rhs) {
        return lhs.depth > rhs.depth;
      });

  if (contacts.size() < contactLimit) {
    contacts.insert(insertPos, candidate);
  } else if (insertPos != contacts.end()) {
    contacts.insert(insertPos, candidate);
    contacts.pop_back();
  }
}

bool collideLargeBoxFaceMesh(
    const BoxShape& box,
    const Eigen::Isometry3d& tfBox,
    const MeshShape& mesh,
    const Eigen::Isometry3d& tfMesh,
    CollisionResult& result,
    const CollisionOption& option)
{
  if (result.numContacts() >= option.maxNumContacts) {
    return false;
  }

  const auto& vertices = mesh.getVertices();
  if (vertices.empty()) {
    return false;
  }

  const Eigen::Isometry3d meshInBox = tfBox.inverse() * tfMesh;
  Eigen::Vector3d meshMin
      = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector3d meshMax
      = Eigen::Vector3d::Constant(-std::numeric_limits<double>::infinity());
  std::vector<Eigen::Vector3d> verticesInBox;
  verticesInBox.reserve(vertices.size());

  for (const auto& localVertex : vertices) {
    const Eigen::Vector3d vertexInBox = meshInBox * localVertex;
    verticesInBox.push_back(vertexInBox);
    meshMin = meshMin.cwiseMin(vertexInBox);
    meshMax = meshMax.cwiseMax(vertexInBox);
  }

  const Eigen::Vector3d meshCenter = (meshMin + meshMax) * 0.5;
  const Eigen::Vector3d meshHalfExtents = (meshMax - meshMin) * 0.5;
  const Eigen::Vector3d& boxHalfExtents = box.getHalfExtents();

  double closestFaceDistance = -std::numeric_limits<double>::infinity();
  int faceAxis = 2;
  double faceSign = 1.0;
  for (int axis = 0; axis < 3; ++axis) {
    for (double sign : {-1.0, 1.0}) {
      const double distance
          = sign * (meshCenter[axis] - sign * boxHalfExtents[axis]);
      if (distance > closestFaceDistance) {
        closestFaceDistance = distance;
        faceAxis = axis;
        faceSign = sign;
      }
    }
  }

  constexpr double largeFaceScale = 10.0;
  constexpr double boundsTolerance = 1e-9;
  for (int axis = 0; axis < 3; ++axis) {
    if (axis == faceAxis) {
      continue;
    }

    const double requiredHalfExtent
        = std::max(meshHalfExtents[axis], 1e-9) * largeFaceScale;
    if (boxHalfExtents[axis] < requiredHalfExtent) {
      return false;
    }
  }

  const std::size_t remaining = option.maxNumContacts - result.numContacts();
  const std::size_t contactLimit = std::min<std::size_t>(remaining, 32u);
  std::vector<FaceContactCandidate> contacts;
  contacts.reserve(contactLimit);
  const std::vector<int> vertexFeatures = makeVertexTriangleFeatureMap(mesh);

  const double faceCoordinate = faceSign * boxHalfExtents[faceAxis];
  for (std::size_t vertexIndex = 0; vertexIndex < verticesInBox.size();
       ++vertexIndex) {
    const auto& vertexInBox = verticesInBox[vertexIndex];
    bool insideFaceBounds = true;
    for (int axis = 0; axis < 3; ++axis) {
      if (axis == faceAxis) {
        continue;
      }

      if (std::abs(vertexInBox[axis])
          > boxHalfExtents[axis] + boundsTolerance) {
        insideFaceBounds = false;
        break;
      }
    }

    if (!insideFaceBounds) {
      continue;
    }

    const double signedDistance
        = faceSign * (vertexInBox[faceAxis] - faceCoordinate);
    if (signedDistance > 0.0) {
      continue;
    }

    addContactCandidate(
        contacts,
        contactLimit,
        FaceContactCandidate{
            tfBox * vertexInBox, -signedDistance, vertexFeatures[vertexIndex]});
  }

  if (contacts.empty()) {
    return false;
  }

  if (!option.enableContact) {
    return true;
  }

  const Eigen::Vector3d worldNormal
      = tfBox.rotation() * (Eigen::Vector3d::Unit(faceAxis) * faceSign);
  for (const auto& candidate : contacts) {
    ContactPoint contact;
    contact.position = candidate.point + worldNormal * (candidate.depth * 0.5);
    contact.normal = worldNormal;
    contact.depth = candidate.depth;
    setMeshTriangleFeature(contact, candidate.featureIndex);
    result.addContact(contact);
  }

  return true;
}

double aabbDistanceSquared(const Aabb& a, const Aabb& b)
{
  double distSq = 0.0;
  for (int i = 0; i < 3; ++i) {
    if (a.max[i] < b.min[i]) {
      const double d = b.min[i] - a.max[i];
      distSq += d * d;
    } else if (b.max[i] < a.min[i]) {
      const double d = a.min[i] - b.max[i];
      distSq += d * d;
    }
  }
  return distSq;
}

double projectTriangle(
    const LocalTriangle& tri, const Eigen::Vector3d& axis, double& outMin)
{
  double minValue = tri.v0.dot(axis);
  double maxValue = minValue;
  const double p1 = tri.v1.dot(axis);
  const double p2 = tri.v2.dot(axis);
  minValue = std::min(minValue, std::min(p1, p2));
  maxValue = std::max(maxValue, std::max(p1, p2));
  outMin = minValue;
  return maxValue;
}

double signedDistanceToPlane(
    const Eigen::Vector3d& p,
    const Eigen::Vector3d& planePoint,
    const Eigen::Vector3d& planeNormal)
{
  const double nNorm = planeNormal.norm();
  if (nNorm < kEpsilon) {
    return 0.0;
  }
  return (p - planePoint).dot(planeNormal) / nNorm;
}

bool pointInTriangleProjected(
    const Eigen::Vector3d& p,
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c,
    const Eigen::Vector3d& normal)
{
  int axis = 0;
  const Eigen::Vector3d absNormal = normal.cwiseAbs();
  if (absNormal[1] > absNormal[axis]) {
    axis = 1;
  }
  if (absNormal[2] > absNormal[axis]) {
    axis = 2;
  }

  const int i1 = (axis + 1) % 3;
  const int i2 = (axis + 2) % 3;

  const Eigen::Vector2d p2(p[i1], p[i2]);
  const Eigen::Vector2d a2(a[i1], a[i2]);
  const Eigen::Vector2d b2(b[i1], b[i2]);
  const Eigen::Vector2d c2(c[i1], c[i2]);

  const auto orient2d = [](const Eigen::Vector2d& p0,
                           const Eigen::Vector2d& p1,
                           const Eigen::Vector2d& p2v) {
    return (p1.x() - p0.x()) * (p2v.y() - p0.y())
           - (p1.y() - p0.y()) * (p2v.x() - p0.x());
  };

  const double o1 = orient2d(a2, b2, p2);
  const double o2 = orient2d(b2, c2, p2);
  const double o3 = orient2d(c2, a2, p2);

  const bool hasNeg = (o1 < -kEpsilon) || (o2 < -kEpsilon) || (o3 < -kEpsilon);
  const bool hasPos = (o1 > kEpsilon) || (o2 > kEpsilon) || (o3 > kEpsilon);
  return !(hasNeg && hasPos);
}

SegmentClosestResult closestPointsBetweenSegments(
    const Eigen::Vector3d& p1,
    const Eigen::Vector3d& q1,
    const Eigen::Vector3d& p2,
    const Eigen::Vector3d& q2)
{
  const Eigen::Vector3d d1 = q1 - p1;
  const Eigen::Vector3d d2 = q2 - p2;
  const Eigen::Vector3d r = p1 - p2;

  const double a = d1.squaredNorm();
  const double e = d2.squaredNorm();
  const double f = d2.dot(r);

  double s = 0.0;
  double t = 0.0;

  if (a <= kEpsilon && e <= kEpsilon) {
    s = t = 0.0;
  } else if (a <= kEpsilon) {
    s = 0.0;
    t = std::clamp(f / e, 0.0, 1.0);
  } else {
    const double c = d1.dot(r);
    if (e <= kEpsilon) {
      t = 0.0;
      s = std::clamp(-c / a, 0.0, 1.0);
    } else {
      const double b = d1.dot(d2);
      const double denom = a * e - b * b;
      if (std::abs(denom) > kEpsilon) {
        s = std::clamp((b * f - c * e) / denom, 0.0, 1.0);
      }
      t = (b * s + f) / e;
      if (t < 0.0) {
        t = 0.0;
        s = std::clamp(-c / a, 0.0, 1.0);
      } else if (t > 1.0) {
        t = 1.0;
        s = std::clamp((b - c) / a, 0.0, 1.0);
      }
    }
  }

  SegmentClosestResult result;
  result.s = s;
  result.t = t;
  result.point1 = p1 + d1 * s;
  result.point2 = p2 + d2 * t;
  result.distSq = (result.point1 - result.point2).squaredNorm();
  return result;
}

struct SegmentTriangleClosestResult
{
  Eigen::Vector3d pointSegment = Eigen::Vector3d::Zero();
  Eigen::Vector3d pointTriangle = Eigen::Vector3d::Zero();
  double distSq = std::numeric_limits<double>::max();
};

SegmentTriangleClosestResult closestSegmentTriangle(
    const Eigen::Vector3d& segStart,
    const Eigen::Vector3d& segEnd,
    const std::array<Eigen::Vector3d, 3>& triVertices)
{
  SegmentTriangleClosestResult best;

  auto updateBest = [&](const Eigen::Vector3d& pointSegment,
                        const Eigen::Vector3d& pointTriangle) {
    const double distSq = (pointSegment - pointTriangle).squaredNorm();
    if (distSq < best.distSq) {
      best.pointSegment = pointSegment;
      best.pointTriangle = pointTriangle;
      best.distSq = distSq;
    }
  };

  const Eigen::Vector3d triNormal = (triVertices[1] - triVertices[0])
                                        .cross(triVertices[2] - triVertices[0]);
  const Eigen::Vector3d segDir = segEnd - segStart;
  const double denom = triNormal.dot(segDir);
  if (std::abs(denom) > kEpsilon) {
    const double t = triNormal.dot(triVertices[0] - segStart) / denom;
    if (t >= 0.0 && t <= 1.0) {
      const Eigen::Vector3d point = segStart + t * segDir;
      if (pointInTriangleProjected(
              point,
              triVertices[0],
              triVertices[1],
              triVertices[2],
              triNormal)) {
        best.pointSegment = point;
        best.pointTriangle = point;
        best.distSq = 0.0;
        return best;
      }
    }
  }

  updateBest(
      segStart,
      closestPointOnTriangle(
          segStart, triVertices[0], triVertices[1], triVertices[2]));
  updateBest(
      segEnd,
      closestPointOnTriangle(
          segEnd, triVertices[0], triVertices[1], triVertices[2]));

  for (int i = 0; i < 3; ++i) {
    const Eigen::Vector3d& edgeStart = triVertices[static_cast<std::size_t>(i)];
    const Eigen::Vector3d& edgeEnd
        = triVertices[static_cast<std::size_t>((i + 1) % 3)];
    const SegmentClosestResult edgeClosest
        = closestPointsBetweenSegments(segStart, segEnd, edgeStart, edgeEnd);
    updateBest(edgeClosest.point1, edgeClosest.point2);
  }

  return best;
}

bool collideCapsuleTriangle(
    const CapsuleShape& capsule,
    const Eigen::Isometry3d& capsuleTransform,
    const std::array<Eigen::Vector3d, 3>& triVertices,
    CollisionResult& result,
    const CollisionOption& option)
{
  if (result.numContacts() >= option.maxNumContacts) {
    return false;
  }

  const double halfHeight = capsule.getHeight() * 0.5;
  const Eigen::Vector3d segStart
      = capsuleTransform * Eigen::Vector3d(0.0, 0.0, -halfHeight);
  const Eigen::Vector3d segEnd
      = capsuleTransform * Eigen::Vector3d(0.0, 0.0, halfHeight);
  const SegmentTriangleClosestResult closest
      = closestSegmentTriangle(segStart, segEnd, triVertices);

  const double radius = capsule.getRadius();
  if (closest.distSq > radius * radius) {
    return false;
  }

  Eigen::Vector3d normal = closest.pointTriangle - closest.pointSegment;
  double distance = 0.0;
  if (closest.distSq > kEpsilon * kEpsilon) {
    distance = std::sqrt(closest.distSq);
    normal /= distance;
  } else {
    normal = (triVertices[1] - triVertices[0])
                 .cross(triVertices[2] - triVertices[0]);
    if (normal.squaredNorm() < kEpsilon * kEpsilon) {
      normal = Eigen::Vector3d::UnitZ();
    } else {
      normal.normalize();
      const Eigen::Vector3d triCenter
          = (triVertices[0] + triVertices[1] + triVertices[2]) / 3.0;
      const Eigen::Vector3d capsuleCenter = (segStart + segEnd) * 0.5;
      if ((capsuleCenter - triCenter).dot(normal) > 0.0) {
        normal = -normal;
      }
    }
  }

  ContactPoint contact;
  contact.position = closest.pointTriangle;
  contact.normal = normal;
  contact.depth = radius - distance;
  result.addContact(contact);
  return true;
}

void addUniquePoint(
    std::vector<Eigen::Vector3d>& points, const Eigen::Vector3d& p)
{
  constexpr double kDupDistSq = 1e-12;
  for (const auto& q : points) {
    if ((p - q).squaredNorm() <= kDupDistSq) {
      return;
    }
  }
  points.push_back(p);
}

LocalTriangle makeLocalTriangle(
    const MeshShape& mesh, int triangleIndex, const Eigen::Isometry3d& tf)
{
  const auto& tri
      = mesh.getTriangles()[static_cast<std::size_t>(triangleIndex)];
  const auto& vertices = mesh.getVertices();
  return {
      tf * vertices[static_cast<std::size_t>(tri[0])],
      tf * vertices[static_cast<std::size_t>(tri[1])],
      tf * vertices[static_cast<std::size_t>(tri[2])]};
}

TriIntersectionResult triangleTriangleIntersection(
    const LocalTriangle& t1, const LocalTriangle& t2)
{
  TriIntersectionResult result;

  const Eigen::Vector3d n1 = t1.normalRaw();
  const Eigen::Vector3d n2 = t2.normalRaw();

  std::array<Eigen::Vector3d, 11> axes;
  int axisCount = 0;
  axes[axisCount++] = n1;
  axes[axisCount++] = n2;
  for (int i = 0; i < 3; ++i) {
    const Eigen::Vector3d e1 = t1.edge(i);
    for (int j = 0; j < 3; ++j) {
      axes[axisCount++] = e1.cross(t2.edge(j));
    }
  }

  double minOverlap = std::numeric_limits<double>::max();
  Eigen::Vector3d bestAxis = Eigen::Vector3d::UnitX();
  bool hasAxis = false;

  for (int i = 0; i < axisCount; ++i) {
    const Eigen::Vector3d axis = axes[static_cast<std::size_t>(i)];
    const double axisNorm = axis.norm();
    if (axisNorm < kEpsilon) {
      continue;
    }

    const Eigen::Vector3d axisDir = axis / axisNorm;
    double min1 = 0.0;
    const double max1 = projectTriangle(t1, axisDir, min1);
    double min2 = 0.0;
    const double max2 = projectTriangle(t2, axisDir, min2);
    const double overlap = std::min(max1, max2) - std::max(min1, min2);
    if (overlap < -kEpsilon) {
      return result;
    }

    if (overlap < minOverlap) {
      minOverlap = overlap;
      bestAxis = axisDir;
      hasAxis = true;
    }
  }

  if (!hasAxis) {
    return result;
  }

  const Eigen::Vector3d centerDelta = t2.centroid() - t1.centroid();
  if (bestAxis.dot(centerDelta) < 0.0) {
    bestAxis = -bestAxis;
  }

  std::vector<Eigen::Vector3d> contactPoints;

  const Eigen::Vector3d t2Normal = t2.normalRaw();
  for (const Eigen::Vector3d& p : {t1.v0, t1.v1, t1.v2}) {
    if (std::abs(signedDistanceToPlane(p, t2.v0, t2Normal)) <= 1e-6
        && pointInTriangleProjected(p, t2.v0, t2.v1, t2.v2, t2Normal)) {
      addUniquePoint(contactPoints, p);
    }
  }

  const Eigen::Vector3d t1Normal = t1.normalRaw();
  for (const Eigen::Vector3d& p : {t2.v0, t2.v1, t2.v2}) {
    if (std::abs(signedDistanceToPlane(p, t1.v0, t1Normal)) <= 1e-6
        && pointInTriangleProjected(p, t1.v0, t1.v1, t1.v2, t1Normal)) {
      addUniquePoint(contactPoints, p);
    }
  }

  const std::array<std::pair<Eigen::Vector3d, Eigen::Vector3d>, 3> edges1
      = {{{t1.v0, t1.v1}, {t1.v1, t1.v2}, {t1.v2, t1.v0}}};
  const std::array<std::pair<Eigen::Vector3d, Eigen::Vector3d>, 3> edges2
      = {{{t2.v0, t2.v1}, {t2.v1, t2.v2}, {t2.v2, t2.v0}}};

  for (const auto& e1 : edges1) {
    for (const auto& e2 : edges2) {
      const SegmentClosestResult segClosest = closestPointsBetweenSegments(
          e1.first, e1.second, e2.first, e2.second);
      if (segClosest.distSq <= 1e-10) {
        addUniquePoint(
            contactPoints, (segClosest.point1 + segClosest.point2) * 0.5);
      }
    }
  }

  auto addEdgeFaceIntersections =
      [&contactPoints](
          const std::array<std::pair<Eigen::Vector3d, Eigen::Vector3d>, 3>&
              edges,
          const LocalTriangle& tri,
          const Eigen::Vector3d& triNormal) {
        if (triNormal.squaredNorm() < kEpsilon * kEpsilon) {
          return;
        }

        constexpr double kSegmentParamTolerance = 1e-9;
        for (const auto& edge : edges) {
          const Eigen::Vector3d edgeDir = edge.second - edge.first;
          const double denom = triNormal.dot(edgeDir);
          if (std::abs(denom) <= kEpsilon) {
            continue;
          }

          const double t = triNormal.dot(tri.v0 - edge.first) / denom;
          if (t < -kSegmentParamTolerance || t > 1.0 + kSegmentParamTolerance) {
            continue;
          }

          const Eigen::Vector3d point
              = edge.first + std::clamp(t, 0.0, 1.0) * edgeDir;
          if (pointInTriangleProjected(
                  point, tri.v0, tri.v1, tri.v2, triNormal)) {
            addUniquePoint(contactPoints, point);
          }
        }
      };
  addEdgeFaceIntersections(edges1, t2, t2Normal);
  addEdgeFaceIntersections(edges2, t1, t1Normal);

  if (contactPoints.empty()) {
    return result;
  }

  result.intersect = true;
  result.normal = bestAxis;
  result.depth = std::max(0.0, minOverlap);

  if (contactPoints.size() <= 2) {
    result.contacts = std::move(contactPoints);
  } else {
    double bestPairDist = -1.0;
    std::size_t bestI = 0;
    std::size_t bestJ = 1;
    for (std::size_t i = 0; i < contactPoints.size(); ++i) {
      for (std::size_t j = i + 1; j < contactPoints.size(); ++j) {
        const double d = (contactPoints[i] - contactPoints[j]).squaredNorm();
        if (d > bestPairDist) {
          bestPairDist = d;
          bestI = i;
          bestJ = j;
        }
      }
    }
    result.contacts.push_back(contactPoints[bestI]);
    result.contacts.push_back(contactPoints[bestJ]);
  }

  return result;
}

bool isLeaf(const MeshShape::BvhNode& node)
{
  return node.count > 0;
}

} // namespace

bool collideMeshMesh(
    const MeshShape& mesh1,
    const Eigen::Isometry3d& tf1,
    const MeshShape& mesh2,
    const Eigen::Isometry3d& tf2,
    CollisionResult& result,
    const CollisionOption& option)
{
  if (option.maxNumContacts == 0) {
    return false;
  }

  const auto& nodes1 = mesh1.bvhNodes();
  const auto& nodes2 = mesh2.bvhNodes();
  if (nodes1.empty() || nodes2.empty()) {
    return false;
  }

  const Eigen::Isometry3d tf2In1 = tf1.inverse() * tf2;

  bool hit = false;
  std::vector<std::pair<int, int>> stack;
  stack.reserve(64);
  stack.push_back({0, 0});

  const auto& triOrder1 = mesh1.bvhTriIndices();
  const auto& triOrder2 = mesh2.bvhTriIndices();

  while (!stack.empty()) {
    const auto [nodeIndex1, nodeIndex2] = stack.back();
    stack.pop_back();

    const auto& node1 = nodes1[static_cast<std::size_t>(nodeIndex1)];
    const auto& node2 = nodes2[static_cast<std::size_t>(nodeIndex2)];
    const Aabb node2In1 = Aabb::transformed(node2.box, tf2In1);

    if (!node1.box.overlaps(node2In1)) {
      continue;
    }

    const bool leaf1 = isLeaf(node1);
    const bool leaf2 = isLeaf(node2);
    if (leaf1 && leaf2) {
      for (int i = 0; i < node1.count; ++i) {
        const int triIndex1
            = triOrder1[static_cast<std::size_t>(node1.first + i)];
        const LocalTriangle tri1 = makeLocalTriangle(
            mesh1, triIndex1, Eigen::Isometry3d::Identity());

        for (int j = 0; j < node2.count; ++j) {
          const int triIndex2
              = triOrder2[static_cast<std::size_t>(node2.first + j)];
          const LocalTriangle tri2
              = makeLocalTriangle(mesh2, triIndex2, tf2In1);

          const TriIntersectionResult triResult
              = triangleTriangleIntersection(tri1, tri2);
          if (!triResult.intersect) {
            continue;
          }

          hit = true;
          if (!option.enableContact) {
            return true;
          }

          const Eigen::Vector3d worldNormal
              = -(tf1.rotation() * triResult.normal).normalized();
          for (const Eigen::Vector3d& pointLocal : triResult.contacts) {
            if (result.numContacts() >= option.maxNumContacts) {
              return true;
            }

            ContactPoint contact;
            contact.position = tf1 * pointLocal;
            contact.normal = worldNormal;
            contact.depth = triResult.depth;
            contact.featureIndex1 = triIndex1;
            contact.featureIndex2 = triIndex2;
            result.addContact(contact);
          }
        }
      }
      continue;
    }

    if (leaf1 || (!leaf2 && node2In1.volume() > node1.box.volume())) {
      stack.push_back({nodeIndex1, node2.left});
      stack.push_back({nodeIndex1, node2.right});
    } else {
      stack.push_back({node1.left, nodeIndex2});
      stack.push_back({node1.right, nodeIndex2});
    }
  }

  return hit;
}

void collideMeshMeshBatch(
    span<const MeshPair> pairs,
    span<CollisionResult> results,
    const CollisionOption& option)
{
  if (results.size() < pairs.size()) {
    throw std::invalid_argument(
        "collideMeshMeshBatch requires one result for each pair");
  }

  for (std::size_t i = 0; i < pairs.size(); ++i) {
    const auto& pair = pairs[i];
    if (pair.shapeA == nullptr || pair.shapeB == nullptr) {
      throw std::invalid_argument("collideMeshMeshBatch received a null mesh");
    }

    [[maybe_unused]] const bool collided = collideMeshMesh(
        *pair.shapeA, pair.tfA, *pair.shapeB, pair.tfB, results[i], option);
  }
}

double distanceMeshMesh(
    const MeshShape& mesh1,
    const Eigen::Isometry3d& tf1,
    const MeshShape& mesh2,
    const Eigen::Isometry3d& tf2,
    DistanceResult& result,
    const DistanceOption& option)
{
  result.clear();

  const auto& nodes1 = mesh1.bvhNodes();
  const auto& nodes2 = mesh2.bvhNodes();
  if (nodes1.empty() || nodes2.empty()) {
    return result.distance;
  }

  const Eigen::Isometry3d tf2In1 = tf1.inverse() * tf2;
  std::vector<Aabb> mesh2NodesInMesh1(nodes2.size());
  for (std::size_t i = 0; i < nodes2.size(); ++i) {
    mesh2NodesInMesh1[i] = Aabb::transformed(nodes2[i].box, tf2In1);
  }

  const auto& triOrder1 = mesh1.bvhTriIndices();
  const auto& triOrder2 = mesh2.bvhTriIndices();

  double bestDistance = option.upperBound;
  bool hasBest = false;
  DistanceResult bestResult;

  std::stack<std::pair<int, int>> stack;
  stack.push({0, 0});
  while (!stack.empty()) {
    const auto [nodeIndex1, nodeIndex2] = stack.top();
    stack.pop();

    const auto& node1 = nodes1[static_cast<std::size_t>(nodeIndex1)];
    const auto& node2 = nodes2[static_cast<std::size_t>(nodeIndex2)];
    const Aabb& node2In1
        = mesh2NodesInMesh1[static_cast<std::size_t>(nodeIndex2)];

    const double boundDistanceSq = aabbDistanceSquared(node1.box, node2In1);
    if (hasBest && boundDistanceSq > bestDistance * bestDistance
        && bestDistance >= 0.0) {
      continue;
    }

    const bool leaf1 = isLeaf(node1);
    const bool leaf2 = isLeaf(node2);
    if (leaf1 && leaf2) {
      for (int i = 0; i < node1.count; ++i) {
        const int triIndex1
            = triOrder1[static_cast<std::size_t>(node1.first + i)];
        const LocalTriangle tri1 = makeLocalTriangle(
            mesh1, triIndex1, Eigen::Isometry3d::Identity());
        const ConvexShape triShape1({tri1.v0, tri1.v1, tri1.v2});

        for (int j = 0; j < node2.count; ++j) {
          const int triIndex2
              = triOrder2[static_cast<std::size_t>(node2.first + j)];
          const LocalTriangle tri2
              = makeLocalTriangle(mesh2, triIndex2, tf2In1);
          const ConvexShape triShape2({tri2.v0, tri2.v1, tri2.v2});

          DistanceResult triDistance;
          DistanceOption triOption;
          triOption.upperBound = hasBest ? bestDistance : option.upperBound;
          triOption.enableNearestPoints = option.enableNearestPoints;

          const double distance = distanceConvexConvex(
              triShape1,
              Eigen::Isometry3d::Identity(),
              triShape2,
              Eigen::Isometry3d::Identity(),
              triDistance,
              triOption);

          if (!hasBest || distance < bestDistance) {
            hasBest = true;
            bestDistance = distance;
            bestResult = triDistance;
            if (option.enableNearestPoints) {
              bestResult.pointOnObject1 = tf1 * triDistance.pointOnObject1;
              bestResult.pointOnObject2 = tf1 * triDistance.pointOnObject2;
              bestResult.normal = tf1.rotation() * triDistance.normal;
            }
          }
        }
      }
      continue;
    }

    if (leaf1 || (!leaf2 && node2In1.volume() > node1.box.volume())) {
      stack.push({nodeIndex1, node2.left});
      stack.push({nodeIndex1, node2.right});
    } else {
      stack.push({node1.left, nodeIndex2});
      stack.push({node1.right, nodeIndex2});
    }
  }

  if (!hasBest) {
    result.distance = std::numeric_limits<double>::max();
    return result.distance;
  }

  result = bestResult;
  result.distance = bestDistance;
  return bestDistance;
}

bool collidePrimitiveMesh(
    const Shape& primitive,
    const Eigen::Isometry3d& tfPrim,
    const MeshShape& mesh,
    const Eigen::Isometry3d& tfMesh,
    CollisionResult& result,
    const CollisionOption& option)
{
  if (!isPrimitiveShape(primitive.getType())) {
    return false;
  }

  const auto& nodes = mesh.bvhNodes();
  if (nodes.empty()) {
    return false;
  }

  const Eigen::Isometry3d primInMesh = tfMesh.inverse() * tfPrim;
  const Aabb primitiveAabbInMesh
      = Aabb::transformed(primitive.computeLocalAabb(), primInMesh);
  const auto* spherePrimitive
      = primitive.getType() == ShapeType::Sphere
            ? static_cast<const SphereShape*>(&primitive)
            : nullptr;
  const auto* capsulePrimitive
      = primitive.getType() == ShapeType::Capsule
            ? static_cast<const CapsuleShape*>(&primitive)
            : nullptr;
  if (primitive.getType() == ShapeType::Box) {
    const auto& box = static_cast<const BoxShape&>(primitive);
    if (collideLargeBoxFaceMesh(box, tfPrim, mesh, tfMesh, result, option)) {
      return true;
    }
  }

  SupportFunction primitiveSupport;
  Eigen::Vector3d primitiveCenterWorld = tfPrim.translation();
  if (!spherePrimitive && !capsulePrimitive) {
    primitiveSupport = makePrimitiveSupportFunction(primitive, tfPrim);
    if (!primitiveSupport) {
      return false;
    }
    primitiveCenterWorld = primitiveCenter(primitive, tfPrim);
  }

  const auto& vertices = mesh.getVertices();
  const auto& triangles = mesh.getTriangles();
  const auto& triOrder = mesh.bvhTriIndices();
  bool hit = false;
  std::vector<int> stack;
  stack.reserve(64);
  stack.push_back(0);

  while (!stack.empty()) {
    const int nodeIndex = stack.back();
    stack.pop_back();

    const auto& node = nodes[static_cast<std::size_t>(nodeIndex)];
    if (!node.box.overlaps(primitiveAabbInMesh)) {
      continue;
    }

    if (!isLeaf(node)) {
      stack.push_back(node.left);
      stack.push_back(node.right);
      continue;
    }

    for (int i = 0; i < node.count; ++i) {
      const int triIndex = triOrder[static_cast<std::size_t>(node.first + i)];
      const auto& tri = triangles[static_cast<std::size_t>(triIndex)];
      const auto& localV0 = vertices[static_cast<std::size_t>(tri[0])];
      const auto& localV1 = vertices[static_cast<std::size_t>(tri[1])];
      const auto& localV2 = vertices[static_cast<std::size_t>(tri[2])];
      const Aabb triangleAabb(
          localV0.cwiseMin(localV1).cwiseMin(localV2),
          localV0.cwiseMax(localV1).cwiseMax(localV2));
      if (!triangleAabb.overlaps(primitiveAabbInMesh)) {
        continue;
      }

      const std::array<Eigen::Vector3d, 3> triVertices{
          tfMesh * localV0, tfMesh * localV1, tfMesh * localV2};
      const Eigen::Vector3d triCenter
          = (triVertices[0] + triVertices[1] + triVertices[2]) / 3.0;

      CollisionOption localOption = option;
      if (option.enableContact) {
        const std::size_t remaining
            = (result.numContacts() >= option.maxNumContacts)
                  ? 0
                  : option.maxNumContacts - result.numContacts();
        if (remaining == 0) {
          return hit;
        }
        localOption.maxNumContacts = remaining;
      }

      if (spherePrimitive) {
        CollisionResult localResult;
        if (collideSphereTriangle(
                *spherePrimitive,
                tfPrim.translation(),
                triVertices,
                localResult,
                localOption)) {
          hit = true;
          if (!option.enableContact) {
            return true;
          }
          appendContactsWithMeshTriangleFeature(result, localResult, triIndex);
          if (result.numContacts() >= option.maxNumContacts) {
            return true;
          }
        }
        continue;
      }

      if (capsulePrimitive) {
        CollisionResult localResult;
        if (collideCapsuleTriangle(
                *capsulePrimitive,
                tfPrim,
                triVertices,
                localResult,
                localOption)) {
          hit = true;
          if (!option.enableContact) {
            return true;
          }
          appendContactsWithMeshTriangleFeature(result, localResult, triIndex);
          if (result.numContacts() >= option.maxNumContacts) {
            return true;
          }
        }
        continue;
      }

      const SupportFunction triangleSupport
          = [&triVertices](const Eigen::Vector3d& dir) {
              double bestDot = triVertices[0].dot(dir);
              Eigen::Vector3d best = triVertices[0];
              const double dot1 = triVertices[1].dot(dir);
              if (dot1 > bestDot) {
                bestDot = dot1;
                best = triVertices[1];
              }
              const double dot2 = triVertices[2].dot(dir);
              if (dot2 > bestDot) {
                best = triVertices[2];
              }
              return best;
            };

      CollisionResult localResult;
      if (collideSupportFunctions(
              primitiveSupport,
              primitiveCenterWorld,
              triangleSupport,
              triCenter,
              localResult,
              localOption)) {
        hit = true;
        if (!option.enableContact) {
          return true;
        }
        appendContactsWithMeshTriangleFeature(
            result, localResult, triIndex, true);
        if (result.numContacts() >= option.maxNumContacts) {
          return true;
        }
      }
    }
  }

  return hit;
}

bool collidePlaneMesh(
    const PlaneShape& plane,
    const Eigen::Isometry3d& planeTransform,
    const MeshShape& mesh,
    const Eigen::Isometry3d& meshTransform,
    CollisionResult& result,
    const CollisionOption& option)
{
  if (result.numContacts() >= option.maxNumContacts) {
    return false;
  }

  const Eigen::Vector3d worldNormal
      = planeTransform.rotation() * plane.getNormal();
  const Eigen::Vector3d planePoint
      = planeTransform.translation() + worldNormal * plane.getOffset();

  struct MeshPlaneContact
  {
    Eigen::Vector3d point;
    double depth = 0.0;
    int featureIndex = -1;
  };

  std::vector<MeshPlaneContact> contacts;
  const std::size_t remaining = option.maxNumContacts - result.numContacts();
  const std::size_t contactLimit = std::min<std::size_t>(remaining, 4u);
  contacts.reserve(contactLimit);
  const std::vector<int> vertexFeatures = makeVertexTriangleFeatureMap(mesh);

  const auto& vertices = mesh.getVertices();
  for (std::size_t vertexIndex = 0; vertexIndex < vertices.size();
       ++vertexIndex) {
    const auto& localVertex = vertices[vertexIndex];
    const Eigen::Vector3d worldVertex = meshTransform * localVertex;
    const double signedDistance = worldNormal.dot(worldVertex - planePoint);
    if (signedDistance > 0.0) {
      continue;
    }

    MeshPlaneContact candidate{
        worldVertex, -signedDistance, vertexFeatures[vertexIndex]};
    const auto insertPos = std::lower_bound(
        contacts.begin(),
        contacts.end(),
        candidate,
        [](const MeshPlaneContact& lhs, const MeshPlaneContact& rhs) {
          return lhs.depth > rhs.depth;
        });

    if (contacts.size() < contactLimit) {
      contacts.insert(insertPos, candidate);
    } else if (insertPos != contacts.end()) {
      contacts.insert(insertPos, candidate);
      contacts.pop_back();
    }
  }

  if (contacts.empty()) {
    return false;
  }

  if (!option.enableContact) {
    return true;
  }

  const std::size_t numContacts = contacts.size();
  for (std::size_t i = 0; i < numContacts; ++i) {
    const auto& meshContact = contacts[i];

    ContactPoint contact;
    contact.position
        = meshContact.point + worldNormal * (meshContact.depth * 0.5);
    contact.normal = worldNormal;
    contact.depth = meshContact.depth;
    setMeshTriangleFeature(contact, meshContact.featureIndex);
    result.addContact(contact);
  }

  return true;
}

} // namespace dart::collision::native
