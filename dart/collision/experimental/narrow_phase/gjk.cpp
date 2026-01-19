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

#include <dart/collision/experimental/narrow_phase/gjk.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace dart::collision::experimental {

namespace {

struct Simplex
{
  std::array<Eigen::Vector3d, 4> points;
  std::array<Eigen::Vector3d, 4> pointsA;
  std::array<Eigen::Vector3d, 4> pointsB;
  int size = 0;

  void push(
      const Eigen::Vector3d& p,
      const Eigen::Vector3d& pA,
      const Eigen::Vector3d& pB)
  {
    if (size < 4) {
      points[size] = p;
      pointsA[size] = pA;
      pointsB[size] = pB;
      ++size;
    }
  }

  void set(
      int idx,
      const Eigen::Vector3d& p,
      const Eigen::Vector3d& pA,
      const Eigen::Vector3d& pB)
  {
    points[idx] = p;
    pointsA[idx] = pA;
    pointsB[idx] = pB;
  }

  void clear()
  {
    size = 0;
  }
};

Eigen::Vector3d tripleProduct(
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c)
{
  return b * a.dot(c) - c * a.dot(b);
}

bool sameDirection(const Eigen::Vector3d& a, const Eigen::Vector3d& b)
{
  return a.dot(b) > 0;
}

bool handleLine(Simplex& simplex, Eigen::Vector3d& direction)
{
  const Eigen::Vector3d& a = simplex.points[1];
  const Eigen::Vector3d& b = simplex.points[0];
  const Eigen::Vector3d ab = b - a;
  const Eigen::Vector3d ao = -a;

  if (sameDirection(ab, ao)) {
    direction = tripleProduct(ab, ao, ab);
    if (direction.squaredNorm() < 1e-12) {
      direction = ab.cross(Eigen::Vector3d::UnitX());
      if (direction.squaredNorm() < 1e-12) {
        direction = ab.cross(Eigen::Vector3d::UnitY());
      }
    }
  } else {
    simplex.set(0, a, simplex.pointsA[1], simplex.pointsB[1]);
    simplex.size = 1;
    direction = ao;
  }

  return false;
}

bool handleTriangle(Simplex& simplex, Eigen::Vector3d& direction)
{
  const Eigen::Vector3d& a = simplex.points[2];
  const Eigen::Vector3d& b = simplex.points[1];
  const Eigen::Vector3d& c = simplex.points[0];
  const Eigen::Vector3d ab = b - a;
  const Eigen::Vector3d ac = c - a;
  const Eigen::Vector3d ao = -a;
  const Eigen::Vector3d abc = ab.cross(ac);

  if (sameDirection(abc.cross(ac), ao)) {
    if (sameDirection(ac, ao)) {
      simplex.set(0, c, simplex.pointsA[0], simplex.pointsB[0]);
      simplex.set(1, a, simplex.pointsA[2], simplex.pointsB[2]);
      simplex.size = 2;
      direction = tripleProduct(ac, ao, ac);
    } else {
      simplex.set(0, b, simplex.pointsA[1], simplex.pointsB[1]);
      simplex.set(1, a, simplex.pointsA[2], simplex.pointsB[2]);
      simplex.size = 2;
      return handleLine(simplex, direction);
    }
  } else {
    if (sameDirection(ab.cross(abc), ao)) {
      simplex.set(0, b, simplex.pointsA[1], simplex.pointsB[1]);
      simplex.set(1, a, simplex.pointsA[2], simplex.pointsB[2]);
      simplex.size = 2;
      return handleLine(simplex, direction);
    } else {
      if (sameDirection(abc, ao)) {
        direction = abc;
      } else {
        simplex.set(0, b, simplex.pointsA[1], simplex.pointsB[1]);
        simplex.set(1, c, simplex.pointsA[0], simplex.pointsB[0]);
        simplex.set(2, a, simplex.pointsA[2], simplex.pointsB[2]);
        direction = -abc;
      }
    }
  }

  return false;
}

bool handleTetrahedron(Simplex& simplex, Eigen::Vector3d& direction)
{
  const Eigen::Vector3d& a = simplex.points[3];
  const Eigen::Vector3d& b = simplex.points[2];
  const Eigen::Vector3d& c = simplex.points[1];
  const Eigen::Vector3d& d = simplex.points[0];
  const Eigen::Vector3d ab = b - a;
  const Eigen::Vector3d ac = c - a;
  const Eigen::Vector3d ad = d - a;
  const Eigen::Vector3d ao = -a;

  const Eigen::Vector3d abc = ab.cross(ac);
  const Eigen::Vector3d acd = ac.cross(ad);
  const Eigen::Vector3d adb = ad.cross(ab);

  if (sameDirection(abc, ao)) {
    simplex.set(0, c, simplex.pointsA[1], simplex.pointsB[1]);
    simplex.set(1, b, simplex.pointsA[2], simplex.pointsB[2]);
    simplex.set(2, a, simplex.pointsA[3], simplex.pointsB[3]);
    simplex.size = 3;
    return handleTriangle(simplex, direction);
  }

  if (sameDirection(acd, ao)) {
    simplex.set(0, d, simplex.pointsA[0], simplex.pointsB[0]);
    simplex.set(1, c, simplex.pointsA[1], simplex.pointsB[1]);
    simplex.set(2, a, simplex.pointsA[3], simplex.pointsB[3]);
    simplex.size = 3;
    return handleTriangle(simplex, direction);
  }

  if (sameDirection(adb, ao)) {
    simplex.set(0, b, simplex.pointsA[2], simplex.pointsB[2]);
    simplex.set(1, d, simplex.pointsA[0], simplex.pointsB[0]);
    simplex.set(2, a, simplex.pointsA[3], simplex.pointsB[3]);
    simplex.size = 3;
    return handleTriangle(simplex, direction);
  }

  return true;
}

bool nextSimplex(Simplex& simplex, Eigen::Vector3d& direction)
{
  switch (simplex.size) {
    case 2:
      return handleLine(simplex, direction);
    case 3:
      return handleTriangle(simplex, direction);
    case 4:
      return handleTetrahedron(simplex, direction);
    default:
      return false;
  }
}

}  // namespace

GjkResult Gjk::query(
    const SupportFunction& supportA,
    const SupportFunction& supportB,
    const Eigen::Vector3d& initialDirection)
{
  GjkResult result;

  Eigen::Vector3d direction = initialDirection.normalized();
  if (direction.squaredNorm() < kTolerance) {
    direction = Eigen::Vector3d::UnitX();
  }

  Simplex simplex;

  Eigen::Vector3d supportPointA = supportA(direction);
  Eigen::Vector3d supportPointB = supportB(-direction);
  Eigen::Vector3d support = supportPointA - supportPointB;

  simplex.push(support, supportPointA, supportPointB);
  direction = -support;

  for (int i = 0; i < kMaxIterations; ++i) {
    if (direction.squaredNorm() < kTolerance) {
      result.intersecting = true;
      return result;
    }

    direction.normalize();
    supportPointA = supportA(direction);
    supportPointB = supportB(-direction);
    support = supportPointA - supportPointB;

    if (support.dot(direction) < kTolerance) {
      result.intersecting = false;
      result.separationAxis = direction;
      return result;
    }

    simplex.push(support, supportPointA, supportPointB);

    if (nextSimplex(simplex, direction)) {
      result.intersecting = true;
      return result;
    }
  }

  result.intersecting = false;
  return result;
}

bool Gjk::intersect(
    const SupportFunction& supportA,
    const SupportFunction& supportB,
    const Eigen::Vector3d& initialDirection)
{
  return query(supportA, supportB, initialDirection).intersecting;
}

namespace {

struct EpaFace
{
  std::array<int, 3> vertices;
  Eigen::Vector3d normal;
  double distance;
  bool valid = true;
};

struct EpaEdge
{
  int a, b;
};

void addIfUniqueEdge(std::vector<EpaEdge>& edges, int a, int b)
{
  for (auto it = edges.begin(); it != edges.end(); ++it) {
    if ((it->a == b && it->b == a)) {
      edges.erase(it);
      return;
    }
  }
  edges.push_back({a, b});
}

}  // namespace

EpaResult Epa::penetration(
    const SupportFunction& supportA,
    const SupportFunction& supportB,
    const std::array<Eigen::Vector3d, 4>& initialSimplex)
{
  EpaResult result;

  std::vector<Eigen::Vector3d> vertices(
      initialSimplex.begin(), initialSimplex.end());
  std::vector<Eigen::Vector3d> verticesA(4);
  std::vector<Eigen::Vector3d> verticesB(4);

  for (int i = 0; i < 4; ++i) {
    Eigen::Vector3d dir = initialSimplex[i].normalized();
    if (dir.squaredNorm() < 1e-12) {
      dir = Eigen::Vector3d::UnitX();
    }
    verticesA[i] = supportA(dir);
    verticesB[i] = supportB(-dir);
  }

  std::vector<EpaFace> faces;
  faces.reserve(64);

  auto addFace = [&](int a, int b, int c) {
    const Eigen::Vector3d& va = vertices[a];
    const Eigen::Vector3d& vb = vertices[b];
    const Eigen::Vector3d& vc = vertices[c];

    Eigen::Vector3d normal = (vb - va).cross(vc - va);
    double len = normal.norm();
    if (len < 1e-12) {
      return;
    }
    normal /= len;

    double dist = normal.dot(va);
    if (dist < 0) {
      normal = -normal;
      dist = -dist;
      faces.push_back({{a, c, b}, normal, dist, true});
    } else {
      faces.push_back({{a, b, c}, normal, dist, true});
    }
  };

  addFace(0, 1, 2);
  addFace(0, 3, 1);
  addFace(0, 2, 3);
  addFace(1, 3, 2);

  for (int iteration = 0; iteration < kMaxIterations; ++iteration) {
    int closestFaceIdx = -1;
    double closestDist = std::numeric_limits<double>::max();

    for (size_t i = 0; i < faces.size(); ++i) {
      if (faces[i].valid && faces[i].distance < closestDist) {
        closestDist = faces[i].distance;
        closestFaceIdx = static_cast<int>(i);
      }
    }

    if (closestFaceIdx < 0) {
      break;
    }

    const EpaFace& closestFace = faces[closestFaceIdx];
    const Eigen::Vector3d& faceNormal = closestFace.normal;

    Eigen::Vector3d pA = supportA(faceNormal);
    Eigen::Vector3d pB = supportB(-faceNormal);
    Eigen::Vector3d newPoint = pA - pB;

    double newDist = newPoint.dot(faceNormal);

    if (newDist - closestDist < kTolerance) {
      result.depth = closestDist;
      result.normal = faceNormal;

      const auto& fv = closestFace.vertices;
      Eigen::Vector3d centroidA =
          (verticesA[fv[0]] + verticesA[fv[1]] + verticesA[fv[2]]) / 3.0;
      Eigen::Vector3d centroidB =
          (verticesB[fv[0]] + verticesB[fv[1]] + verticesB[fv[2]]) / 3.0;

      result.pointOnA = centroidA;
      result.pointOnB = centroidB;
      return result;
    }

    int newVertexIdx = static_cast<int>(vertices.size());
    vertices.push_back(newPoint);
    verticesA.push_back(pA);
    verticesB.push_back(pB);

    std::vector<EpaEdge> edges;
    edges.reserve(32);

    for (size_t i = 0; i < faces.size(); ++i) {
      if (!faces[i].valid) {
        continue;
      }

      if (faces[i].normal.dot(newPoint - vertices[faces[i].vertices[0]]) > 0) {
        faces[i].valid = false;

        addIfUniqueEdge(edges, faces[i].vertices[0], faces[i].vertices[1]);
        addIfUniqueEdge(edges, faces[i].vertices[1], faces[i].vertices[2]);
        addIfUniqueEdge(edges, faces[i].vertices[2], faces[i].vertices[0]);
      }
    }

    for (const auto& edge : edges) {
      addFace(edge.a, edge.b, newVertexIdx);
    }
  }

  return result;
}

}  // namespace dart::collision::experimental
