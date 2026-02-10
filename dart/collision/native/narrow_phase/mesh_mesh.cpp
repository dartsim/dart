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

  if (contactPoints.empty()) {
    double bestDistSq = std::numeric_limits<double>::max();
    Eigen::Vector3d bestMidpoint = (t1.centroid() + t2.centroid()) * 0.5;
    for (const auto& e1 : edges1) {
      for (const auto& e2 : edges2) {
        const SegmentClosestResult segClosest = closestPointsBetweenSegments(
            e1.first, e1.second, e2.first, e2.second);
        if (segClosest.distSq < bestDistSq) {
          bestDistSq = segClosest.distSq;
          bestMidpoint = (segClosest.point1 + segClosest.point2) * 0.5;
        }
      }
    }
    addUniquePoint(contactPoints, bestMidpoint);
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
  const auto& nodes1 = mesh1.bvhNodes();
  const auto& nodes2 = mesh2.bvhNodes();
  if (nodes1.empty() || nodes2.empty()) {
    return false;
  }

  const Eigen::Isometry3d tf2In1 = tf1.inverse() * tf2;

  std::vector<Aabb> mesh2NodesInMesh1(nodes2.size());
  for (std::size_t i = 0; i < nodes2.size(); ++i) {
    mesh2NodesInMesh1[i] = Aabb::transformed(nodes2[i].box, tf2In1);
  }

  bool hit = false;
  std::stack<std::pair<int, int>> stack;
  stack.push({0, 0});

  const auto& triOrder1 = mesh1.bvhTriIndices();
  const auto& triOrder2 = mesh2.bvhTriIndices();

  while (!stack.empty()) {
    const auto [nodeIndex1, nodeIndex2] = stack.top();
    stack.pop();

    const auto& node1 = nodes1[static_cast<std::size_t>(nodeIndex1)];
    const auto& node2 = nodes2[static_cast<std::size_t>(nodeIndex2)];
    const Aabb& node2In1
        = mesh2NodesInMesh1[static_cast<std::size_t>(nodeIndex2)];

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
          if (!option.enableContact || option.maxNumContacts == 0) {
            return true;
          }

          const Eigen::Vector3d worldNormal
              = (tf1.rotation() * triResult.normal).normalized();
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
      stack.push({nodeIndex1, node2.left});
      stack.push({nodeIndex1, node2.right});
    } else {
      stack.push({node1.left, nodeIndex2});
      stack.push({node1.right, nodeIndex2});
    }
  }

  return hit;
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

  const auto& triOrder = mesh.bvhTriIndices();
  bool hit = false;
  std::stack<int> stack;
  stack.push(0);

  while (!stack.empty()) {
    const int nodeIndex = stack.top();
    stack.pop();

    const auto& node = nodes[static_cast<std::size_t>(nodeIndex)];
    if (!node.box.overlaps(primitiveAabbInMesh)) {
      continue;
    }

    if (!isLeaf(node)) {
      stack.push(node.left);
      stack.push(node.right);
      continue;
    }

    for (int i = 0; i < node.count; ++i) {
      const int triIndex = triOrder[static_cast<std::size_t>(node.first + i)];
      const auto& tri = mesh.getTriangles()[static_cast<std::size_t>(triIndex)];
      const auto& vertices = mesh.getVertices();
      ConvexShape triShape(
          {vertices[static_cast<std::size_t>(tri[0])],
           vertices[static_cast<std::size_t>(tri[1])],
           vertices[static_cast<std::size_t>(tri[2])]});

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

      if (collideConvexConvex(
              primitive, tfPrim, triShape, tfMesh, result, localOption)) {
        hit = true;
        if (!option.enableContact) {
          return true;
        }
        if (result.numContacts() >= option.maxNumContacts) {
          return true;
        }
      }
    }
  }

  return hit;
}

} // namespace dart::collision::native
