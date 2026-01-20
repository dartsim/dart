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
#include <limits>
#include <vector>

#include <cmath>

namespace dart::collision::experimental {

namespace {

constexpr double kEpsilon = 1e-12;

SupportPoint computeSupport(
    const SupportFunction& supportA,
    const SupportFunction& supportB,
    const Eigen::Vector3d& direction)
{
  Eigen::Vector3d dir = direction;
  if (dir.squaredNorm() < kEpsilon) {
    dir = Eigen::Vector3d::UnitX();
  } else {
    dir.normalize();
  }

  SupportPoint point;
  point.v1 = supportA(dir);
  point.v2 = supportB(-dir);
  point.v = point.v1 - point.v2;
  return point;
}

enum class TriangleRegion
{
  VertexA,
  VertexB,
  VertexC,
  EdgeAB,
  EdgeAC,
  EdgeBC,
  Face,
};

struct TriangleClosestResult
{
  Eigen::Vector3d closest = Eigen::Vector3d::Zero();
  std::array<double, 3> weights{};
  TriangleRegion region = TriangleRegion::Face;
};

struct SegmentClosestResult
{
  Eigen::Vector3d closest = Eigen::Vector3d::Zero();
  double t = 0.0;
};

SegmentClosestResult closestPointSegmentToOrigin(
    const Eigen::Vector3d& a, const Eigen::Vector3d& b)
{
  SegmentClosestResult result;
  const Eigen::Vector3d ab = b - a;
  const double abLen2 = ab.squaredNorm();
  if (abLen2 < kEpsilon) {
    result.closest = a;
    result.t = 0.0;
    return result;
  }

  double t = -a.dot(ab) / abLen2;
  t = std::clamp(t, 0.0, 1.0);
  result.t = t;
  result.closest = a + t * ab;
  return result;
}

TriangleClosestResult closestPointTriangleToOrigin(
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c)
{
  TriangleClosestResult result;

  const Eigen::Vector3d ab = b - a;
  const Eigen::Vector3d ac = c - a;
  const Eigen::Vector3d ap = -a;

  const double d1 = ab.dot(ap);
  const double d2 = ac.dot(ap);
  if (d1 <= 0.0 && d2 <= 0.0) {
    result.closest = a;
    result.weights = {1.0, 0.0, 0.0};
    result.region = TriangleRegion::VertexA;
    return result;
  }

  const Eigen::Vector3d bp = -b;
  const double d3 = ab.dot(bp);
  const double d4 = ac.dot(bp);
  if (d3 >= 0.0 && d4 <= d3) {
    result.closest = b;
    result.weights = {0.0, 1.0, 0.0};
    result.region = TriangleRegion::VertexB;
    return result;
  }

  const double vc = d1 * d4 - d3 * d2;
  if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
    const double v = d1 / (d1 - d3);
    result.closest = a + v * ab;
    result.weights = {1.0 - v, v, 0.0};
    result.region = TriangleRegion::EdgeAB;
    return result;
  }

  const Eigen::Vector3d cp = -c;
  const double d5 = ab.dot(cp);
  const double d6 = ac.dot(cp);
  if (d6 >= 0.0 && d5 <= d6) {
    result.closest = c;
    result.weights = {0.0, 0.0, 1.0};
    result.region = TriangleRegion::VertexC;
    return result;
  }

  const double vb = d5 * d2 - d1 * d6;
  if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
    const double w = d2 / (d2 - d6);
    result.closest = a + w * ac;
    result.weights = {1.0 - w, 0.0, w};
    result.region = TriangleRegion::EdgeAC;
    return result;
  }

  const double va = d3 * d6 - d5 * d4;
  if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0) {
    const double w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
    result.closest = b + w * (c - b);
    result.weights = {0.0, 1.0 - w, w};
    result.region = TriangleRegion::EdgeBC;
    return result;
  }

  const double denom = va + vb + vc;
  if (std::abs(denom) < kEpsilon) {
    auto best = TriangleClosestResult{};
    const auto ab = closestPointSegmentToOrigin(a, b);
    best.closest = ab.closest;
    if (ab.t <= 0.0) {
      best.weights = {1.0, 0.0, 0.0};
      best.region = TriangleRegion::VertexA;
    } else if (ab.t >= 1.0) {
      best.weights = {0.0, 1.0, 0.0};
      best.region = TriangleRegion::VertexB;
    } else {
      best.weights = {1.0 - ab.t, ab.t, 0.0};
      best.region = TriangleRegion::EdgeAB;
    }
    double bestDist2 = best.closest.squaredNorm();

    const auto ac = closestPointSegmentToOrigin(a, c);
    TriangleClosestResult candidate;
    candidate.closest = ac.closest;
    if (ac.t <= 0.0) {
      candidate.weights = {1.0, 0.0, 0.0};
      candidate.region = TriangleRegion::VertexA;
    } else if (ac.t >= 1.0) {
      candidate.weights = {0.0, 0.0, 1.0};
      candidate.region = TriangleRegion::VertexC;
    } else {
      candidate.weights = {1.0 - ac.t, 0.0, ac.t};
      candidate.region = TriangleRegion::EdgeAC;
    }
    double candidateDist2 = candidate.closest.squaredNorm();
    if (candidateDist2 < bestDist2) {
      best = candidate;
      bestDist2 = candidateDist2;
    }

    const auto bc = closestPointSegmentToOrigin(b, c);
    candidate.closest = bc.closest;
    if (bc.t <= 0.0) {
      candidate.weights = {0.0, 1.0, 0.0};
      candidate.region = TriangleRegion::VertexB;
    } else if (bc.t >= 1.0) {
      candidate.weights = {0.0, 0.0, 1.0};
      candidate.region = TriangleRegion::VertexC;
    } else {
      candidate.weights = {0.0, 1.0 - bc.t, bc.t};
      candidate.region = TriangleRegion::EdgeBC;
    }
    candidateDist2 = candidate.closest.squaredNorm();
    if (candidateDist2 < bestDist2) {
      best = candidate;
    }

    return best;
  }
  const double inv = 1.0 / denom;
  const double v = vb * inv;
  const double w = vc * inv;
  const double u = 1.0 - v - w;
  result.closest = u * a + v * b + w * c;
  result.weights = {u, v, w};
  result.region = TriangleRegion::Face;
  return result;
}

void computeClosestPoints(
    const GjkSimplex& simplex,
    const std::array<double, 4>& weights,
    Eigen::Vector3d& pointA,
    Eigen::Vector3d& pointB)
{
  pointA.setZero();
  pointB.setZero();
  for (int i = 0; i < simplex.size; ++i) {
    pointA += weights[i] * simplex.points[i].v1;
    pointB += weights[i] * simplex.points[i].v2;
  }
}

bool reduceLine(
    GjkSimplex& simplex,
    Eigen::Vector3d& closest,
    std::array<double, 4>& weights)
{
  const SupportPoint& a = simplex.points[1];
  const SupportPoint& b = simplex.points[0];
  const Eigen::Vector3d ab = b.v - a.v;
  const double abLen2 = ab.squaredNorm();

  weights.fill(0.0);

  if (abLen2 < kEpsilon) {
    simplex.points[0] = a;
    simplex.size = 1;
    closest = a.v;
    weights[0] = 1.0;
    return false;
  }

  const double t = -a.v.dot(ab) / abLen2;
  if (t <= 0.0) {
    simplex.points[0] = a;
    simplex.size = 1;
    closest = a.v;
    weights[0] = 1.0;
  } else if (t >= 1.0) {
    simplex.points[0] = b;
    simplex.size = 1;
    closest = b.v;
    weights[0] = 1.0;
  } else {
    simplex.points[0] = b;
    simplex.points[1] = a;
    simplex.size = 2;
    closest = a.v + t * ab;
    weights[0] = t;
    weights[1] = 1.0 - t;
  }

  return false;
}

bool reduceTriangle(
    GjkSimplex& simplex,
    Eigen::Vector3d& closest,
    std::array<double, 4>& weights)
{
  const SupportPoint& a = simplex.points[2];
  const SupportPoint& b = simplex.points[1];
  const SupportPoint& c = simplex.points[0];

  TriangleClosestResult tri = closestPointTriangleToOrigin(a.v, b.v, c.v);
  closest = tri.closest;
  weights.fill(0.0);

  switch (tri.region) {
    case TriangleRegion::VertexA:
      simplex.points[0] = a;
      simplex.size = 1;
      weights[0] = 1.0;
      break;
    case TriangleRegion::VertexB:
      simplex.points[0] = b;
      simplex.size = 1;
      weights[0] = 1.0;
      break;
    case TriangleRegion::VertexC:
      simplex.points[0] = c;
      simplex.size = 1;
      weights[0] = 1.0;
      break;
    case TriangleRegion::EdgeAB:
      simplex.points[0] = b;
      simplex.points[1] = a;
      simplex.size = 2;
      weights[0] = tri.weights[1];
      weights[1] = tri.weights[0];
      break;
    case TriangleRegion::EdgeAC:
      simplex.points[0] = c;
      simplex.points[1] = a;
      simplex.size = 2;
      weights[0] = tri.weights[2];
      weights[1] = tri.weights[0];
      break;
    case TriangleRegion::EdgeBC:
      simplex.points[0] = c;
      simplex.points[1] = b;
      simplex.size = 2;
      weights[0] = tri.weights[2];
      weights[1] = tri.weights[1];
      break;
    case TriangleRegion::Face:
      simplex.points[0] = c;
      simplex.points[1] = b;
      simplex.points[2] = a;
      simplex.size = 3;
      weights[0] = tri.weights[2];
      weights[1] = tri.weights[1];
      weights[2] = tri.weights[0];
      break;
  }

  return false;
}

bool originOutsideFace(
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c,
    const Eigen::Vector3d& d,
    Eigen::Vector3d& normalOut)
{
  Eigen::Vector3d normal = (b - a).cross(c - a);
  const double norm2 = normal.squaredNorm();
  if (norm2 < kEpsilon) {
    normalOut = normal;
    return true;
  }

  if (normal.dot(d - a) > 0.0) {
    normal = -normal;
  }

  normalOut = normal;
  return normal.dot(-a) > 0.0;
}

bool reduceTetrahedron(
    GjkSimplex& simplex,
    Eigen::Vector3d& closest,
    std::array<double, 4>& weights)
{
  constexpr std::array<std::array<int, 3>, 4> faces = {
      std::array<int, 3>{3, 2, 1},
      std::array<int, 3>{3, 1, 0},
      std::array<int, 3>{3, 0, 2},
      std::array<int, 3>{2, 0, 1},
  };
  constexpr std::array<int, 4> opposites = {0, 2, 1, 3};

  bool originInside = true;
  double bestDist2 = std::numeric_limits<double>::max();
  int bestFace = -1;

  for (int i = 0; i < static_cast<int>(faces.size()); ++i) {
    const auto& face = faces[i];
    const SupportPoint& a = simplex.points[face[0]];
    const SupportPoint& b = simplex.points[face[1]];
    const SupportPoint& c = simplex.points[face[2]];
    const SupportPoint& d = simplex.points[opposites[i]];

    Eigen::Vector3d normal;
    if (!originOutsideFace(a.v, b.v, c.v, d.v, normal)) {
      continue;
    }

    originInside = false;
    TriangleClosestResult tri = closestPointTriangleToOrigin(a.v, b.v, c.v);
    const double dist2 = tri.closest.squaredNorm();
    if (dist2 < bestDist2) {
      bestDist2 = dist2;
      bestFace = i;
    }
  }

  if (originInside) {
    return true;
  }

  if (bestFace >= 0) {
    const auto& face = faces[bestFace];
    GjkSimplex faceSimplex;
    faceSimplex.size = 3;
    faceSimplex.points[0] = simplex.points[face[2]];
    faceSimplex.points[1] = simplex.points[face[1]];
    faceSimplex.points[2] = simplex.points[face[0]];
    reduceTriangle(faceSimplex, closest, weights);
    simplex = faceSimplex;
  }

  return false;
}

bool reduceSimplex(
    GjkSimplex& simplex,
    Eigen::Vector3d& closest,
    std::array<double, 4>& weights)
{
  switch (simplex.size) {
    case 1:
      closest = simplex.points[0].v;
      weights.fill(0.0);
      weights[0] = 1.0;
      return false;
    case 2:
      return reduceLine(simplex, closest, weights);
    case 3:
      return reduceTriangle(simplex, closest, weights);
    case 4:
      return reduceTetrahedron(simplex, closest, weights);
    default:
      closest = Eigen::Vector3d::Zero();
      weights.fill(0.0);
      return false;
  }
}

void fillSeparationResult(
    const GjkSimplex& simplex,
    const std::array<double, 4>& weights,
    const Eigen::Vector3d& closest,
    GjkResult& result)
{
  result.intersecting = false;
  result.distance = closest.norm();
  computeClosestPoints(
      simplex, weights, result.closestPointA, result.closestPointB);

  Eigen::Vector3d axis = result.closestPointB - result.closestPointA;
  if (axis.squaredNorm() > kEpsilon) {
    result.separationAxis = axis.normalized();
  } else if (closest.squaredNorm() > kEpsilon) {
    result.separationAxis = (-closest).normalized();
  } else {
    result.separationAxis = Eigen::Vector3d::UnitX();
  }
}

} // namespace

GjkResult Gjk::query(
    const SupportFunction& supportA,
    const SupportFunction& supportB,
    const Eigen::Vector3d& initialDirection)
{
  GjkResult result;
  GjkSimplex simplex;
  std::array<double, 4> weights{};

  Eigen::Vector3d direction = initialDirection;
  if (direction.squaredNorm() < kEpsilon) {
    direction = Eigen::Vector3d::UnitX();
  }

  simplex.push(computeSupport(supportA, supportB, direction));
  Eigen::Vector3d closest = simplex.points[0].v;
  reduceSimplex(simplex, closest, weights);

  double prevDist2 = std::numeric_limits<double>::infinity();

  for (int iter = 0; iter < Gjk::kMaxIterations; ++iter) {
    const double dist2 = closest.squaredNorm();
    if (dist2 <= Gjk::kTolerance * Gjk::kTolerance) {
      result.intersecting = true;
      result.simplex = simplex;
      return result;
    }

    direction = -closest;
    SupportPoint newPoint = computeSupport(supportA, supportB, direction);

    const double delta = newPoint.v.dot(direction) - closest.dot(direction);
    if (delta <= Gjk::kTolerance) {
      fillSeparationResult(simplex, weights, closest, result);
      result.simplex = simplex;
      return result;
    }

    simplex.push(newPoint);

    if (reduceSimplex(simplex, closest, weights)) {
      result.intersecting = true;
      result.simplex = simplex;
      return result;
    }

    const double newDist2 = closest.squaredNorm();
    if (std::abs(prevDist2 - newDist2) <= Gjk::kTolerance * Gjk::kTolerance) {
      fillSeparationResult(simplex, weights, closest, result);
      result.simplex = simplex;
      return result;
    }
    prevDist2 = newDist2;
  }

  fillSeparationResult(simplex, weights, closest, result);
  result.simplex = simplex;
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
  Eigen::Vector3d normal = Eigen::Vector3d::Zero();
  double distance = 0.0;
  bool valid = true;
};

struct EpaEdge
{
  int a = 0;
  int b = 0;
};

void addIfUniqueEdge(std::vector<EpaEdge>& edges, int a, int b)
{
  for (auto it = edges.begin(); it != edges.end(); ++it) {
    if (it->a == b && it->b == a) {
      edges.erase(it);
      return;
    }
  }
  edges.push_back({a, b});
}

bool addFace(
    std::vector<EpaFace>& faces,
    const std::vector<SupportPoint>& vertices,
    int a,
    int b,
    int c)
{
  const Eigen::Vector3d& va = vertices[a].v;
  const Eigen::Vector3d& vb = vertices[b].v;
  const Eigen::Vector3d& vc = vertices[c].v;

  Eigen::Vector3d normal = (vb - va).cross(vc - va);
  const double len = normal.norm();
  if (len < kEpsilon) {
    return false;
  }
  normal /= len;

  double dist = normal.dot(va);
  if (dist < 0.0) {
    normal = -normal;
    dist = -dist;
    faces.push_back({{a, c, b}, normal, dist, true});
  } else {
    faces.push_back({{a, b, c}, normal, dist, true});
  }

  return true;
}

} // namespace

EpaResult Epa::penetration(
    const SupportFunction& supportA,
    const SupportFunction& supportB,
    const GjkSimplex& initialSimplex)
{
  EpaResult result;

  if (initialSimplex.size < 4) {
    return result;
  }

  std::vector<SupportPoint> vertices;
  vertices.reserve(64);
  for (int i = 0; i < initialSimplex.size; ++i) {
    vertices.push_back(initialSimplex.points[i]);
  }

  std::vector<EpaFace> faces;
  faces.reserve(64);

  addFace(faces, vertices, 0, 1, 2);
  addFace(faces, vertices, 0, 3, 1);
  addFace(faces, vertices, 0, 2, 3);
  addFace(faces, vertices, 1, 3, 2);

  for (int iteration = 0; iteration < Epa::kMaxIterations; ++iteration) {
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

    SupportPoint newPoint = computeSupport(supportA, supportB, faceNormal);
    const double newDist = newPoint.v.dot(faceNormal);

    if (newDist - closestDist < Epa::kTolerance) {
      const auto& fv = closestFace.vertices;
      const SupportPoint& va = vertices[fv[0]];
      const SupportPoint& vb = vertices[fv[1]];
      const SupportPoint& vc = vertices[fv[2]];
      TriangleClosestResult tri
          = closestPointTriangleToOrigin(va.v, vb.v, vc.v);

      result.depth = closestDist;
      result.normal = faceNormal;
      result.pointOnA = tri.weights[0] * va.v1 + tri.weights[1] * vb.v1
                        + tri.weights[2] * vc.v1;
      result.pointOnB = tri.weights[0] * va.v2 + tri.weights[1] * vb.v2
                        + tri.weights[2] * vc.v2;
      result.success = true;
      return result;
    }

    const int newVertexIdx = static_cast<int>(vertices.size());
    vertices.push_back(newPoint);

    std::vector<EpaEdge> edges;
    edges.reserve(32);

    for (size_t i = 0; i < faces.size(); ++i) {
      if (!faces[i].valid) {
        continue;
      }

      const int v0 = faces[i].vertices[0];
      if (faces[i].normal.dot(newPoint.v - vertices[v0].v) > 0.0) {
        faces[i].valid = false;
        addIfUniqueEdge(edges, faces[i].vertices[0], faces[i].vertices[1]);
        addIfUniqueEdge(edges, faces[i].vertices[1], faces[i].vertices[2]);
        addIfUniqueEdge(edges, faces[i].vertices[2], faces[i].vertices[0]);
      }
    }

    for (const auto& edge : edges) {
      addFace(faces, vertices, edge.a, edge.b, newVertexIdx);
    }
  }

  return result;
}

} // namespace dart::collision::experimental
