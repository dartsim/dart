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

#include <dart/collision/experimental/shapes/shape.hpp>

#include <algorithm>
#include <limits>
#include <numeric>

namespace dart::collision::experimental {

SphereShape::SphereShape(double radius) : radius_(radius) {}

ShapeType SphereShape::getType() const
{
  return ShapeType::Sphere;
}

Aabb SphereShape::computeLocalAabb() const
{
  return Aabb::forSphere(radius_);
}

double SphereShape::getRadius() const
{
  return radius_;
}

BoxShape::BoxShape(const Eigen::Vector3d& halfExtents)
  : halfExtents_(halfExtents)
{
}

ShapeType BoxShape::getType() const
{
  return ShapeType::Box;
}

Aabb BoxShape::computeLocalAabb() const
{
  return Aabb::forBox(halfExtents_);
}

const Eigen::Vector3d& BoxShape::getHalfExtents() const
{
  return halfExtents_;
}

CapsuleShape::CapsuleShape(double radius, double height)
  : radius_(radius), height_(height)
{
}

ShapeType CapsuleShape::getType() const
{
  return ShapeType::Capsule;
}

Aabb CapsuleShape::computeLocalAabb() const
{
  return Aabb::forCapsule(radius_, height_);
}

double CapsuleShape::getRadius() const
{
  return radius_;
}

double CapsuleShape::getHeight() const
{
  return height_;
}

CylinderShape::CylinderShape(double radius, double height)
  : radius_(radius), height_(height)
{
}

ShapeType CylinderShape::getType() const
{
  return ShapeType::Cylinder;
}

Aabb CylinderShape::computeLocalAabb() const
{
  return Aabb::forCylinder(radius_, height_);
}

double CylinderShape::getRadius() const
{
  return radius_;
}

double CylinderShape::getHeight() const
{
  return height_;
}

PlaneShape::PlaneShape(const Eigen::Vector3d& normal, double offset)
  : normal_(normal.normalized()), offset_(offset)
{
}

ShapeType PlaneShape::getType() const
{
  return ShapeType::Plane;
}

Aabb PlaneShape::computeLocalAabb() const
{
  constexpr double inf = std::numeric_limits<double>::infinity();
  return Aabb(
      Eigen::Vector3d(-inf, -inf, -inf), Eigen::Vector3d(inf, inf, inf));
}

const Eigen::Vector3d& PlaneShape::getNormal() const
{
  return normal_;
}

double PlaneShape::getOffset() const
{
  return offset_;
}

ConvexShape::ConvexShape(std::vector<Eigen::Vector3d> vertices)
  : vertices_(std::move(vertices))
{
}

ShapeType ConvexShape::getType() const
{
  return ShapeType::Convex;
}

Aabb ConvexShape::computeLocalAabb() const
{
  if (vertices_.empty()) {
    return Aabb(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  }

  Eigen::Vector3d min = vertices_[0];
  Eigen::Vector3d max = vertices_[0];

  for (size_t i = 1; i < vertices_.size(); ++i) {
    min = min.cwiseMin(vertices_[i]);
    max = max.cwiseMax(vertices_[i]);
  }

  return Aabb(min, max);
}

const std::vector<Eigen::Vector3d>& ConvexShape::getVertices() const
{
  return vertices_;
}

Eigen::Vector3d ConvexShape::support(const Eigen::Vector3d& direction) const
{
  if (vertices_.empty()) {
    return Eigen::Vector3d::Zero();
  }

  double maxDot = vertices_[0].dot(direction);
  size_t maxIndex = 0;

  for (size_t i = 1; i < vertices_.size(); ++i) {
    const double dot = vertices_[i].dot(direction);
    if (dot > maxDot) {
      maxDot = dot;
      maxIndex = i;
    }
  }

  return vertices_[maxIndex];
}

MeshShape::MeshShape(
    std::vector<Eigen::Vector3d> vertices, std::vector<Triangle> triangles)
  : vertices_(std::move(vertices)), triangles_(std::move(triangles))
{
  buildBvh();
}

ShapeType MeshShape::getType() const
{
  return ShapeType::Mesh;
}

Aabb MeshShape::computeLocalAabb() const
{
  if (vertices_.empty()) {
    return Aabb(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  }

  Eigen::Vector3d min = vertices_[0];
  Eigen::Vector3d max = vertices_[0];

  for (size_t i = 1; i < vertices_.size(); ++i) {
    min = min.cwiseMin(vertices_[i]);
    max = max.cwiseMax(vertices_[i]);
  }

  return Aabb(min, max);
}

const std::vector<Eigen::Vector3d>& MeshShape::getVertices() const
{
  return vertices_;
}

const std::vector<MeshShape::Triangle>& MeshShape::getTriangles() const
{
  return triangles_;
}

const std::vector<MeshShape::BvhNode>& MeshShape::bvhNodes() const
{
  return bvhNodes_;
}

const std::vector<int>& MeshShape::bvhTriIndices() const
{
  return triIndices_;
}

Eigen::Vector3d MeshShape::support(const Eigen::Vector3d& direction) const
{
  if (vertices_.empty()) {
    return Eigen::Vector3d::Zero();
  }

  double maxDot = vertices_[0].dot(direction);
  size_t maxIndex = 0;

  for (size_t i = 1; i < vertices_.size(); ++i) {
    const double dot = vertices_[i].dot(direction);
    if (dot > maxDot) {
      maxDot = dot;
      maxIndex = i;
    }
  }

  return vertices_[maxIndex];
}

void MeshShape::buildBvh()
{
  bvhNodes_.clear();
  triIndices_.clear();

  const int numTriangles = static_cast<int>(triangles_.size());
  if (numTriangles <= 0) {
    return;
  }

  triIndices_.resize(static_cast<std::size_t>(numTriangles));
  std::iota(triIndices_.begin(), triIndices_.end(), 0);

  auto triangleAabb = [this](int triIndex) {
    const Triangle& tri = triangles_[static_cast<std::size_t>(triIndex)];
    const Eigen::Vector3d& v0 = vertices_[static_cast<std::size_t>(tri[0])];
    const Eigen::Vector3d& v1 = vertices_[static_cast<std::size_t>(tri[1])];
    const Eigen::Vector3d& v2 = vertices_[static_cast<std::size_t>(tri[2])];
    const Eigen::Vector3d min = v0.cwiseMin(v1).cwiseMin(v2);
    const Eigen::Vector3d max = v0.cwiseMax(v1).cwiseMax(v2);
    return Aabb(min, max);
  };

  auto triangleCentroid = [this](int triIndex) {
    const Triangle& tri = triangles_[static_cast<std::size_t>(triIndex)];
    return (vertices_[static_cast<std::size_t>(tri[0])]
            + vertices_[static_cast<std::size_t>(tri[1])]
            + vertices_[static_cast<std::size_t>(tri[2])])
           / 3.0;
  };

  constexpr int kLeafSize = 4;
  const auto buildNode = [&](auto&& self, int first, int count) -> int {
    const int nodeIndex = static_cast<int>(bvhNodes_.size());
    bvhNodes_.push_back(BvhNode{});

    Aabb bounds = triangleAabb(triIndices_[static_cast<std::size_t>(first)]);
    Eigen::Vector3d centroidMin
        = triangleCentroid(triIndices_[static_cast<std::size_t>(first)]);
    Eigen::Vector3d centroidMax = centroidMin;

    for (int i = 1; i < count; ++i) {
      const int triIndex = triIndices_[static_cast<std::size_t>(first + i)];
      bounds.merge(triangleAabb(triIndex));
      const Eigen::Vector3d centroid = triangleCentroid(triIndex);
      centroidMin = centroidMin.cwiseMin(centroid);
      centroidMax = centroidMax.cwiseMax(centroid);
    }

    bvhNodes_[static_cast<std::size_t>(nodeIndex)].box = bounds;

    const Eigen::Vector3d centroidExtent = centroidMax - centroidMin;
    const bool smallCentroidSpan = centroidExtent.maxCoeff() <= 1e-12;
    if (count <= kLeafSize || smallCentroidSpan) {
      BvhNode& leaf = bvhNodes_[static_cast<std::size_t>(nodeIndex)];
      leaf.left = -1;
      leaf.right = -1;
      leaf.first = first;
      leaf.count = count;
      return nodeIndex;
    }

    int axis = 0;
    if (centroidExtent[1] > centroidExtent[axis]) {
      axis = 1;
    }
    if (centroidExtent[2] > centroidExtent[axis]) {
      axis = 2;
    }

    const int mid = first + count / 2;
    std::nth_element(
        triIndices_.begin() + first,
        triIndices_.begin() + mid,
        triIndices_.begin() + first + count,
        [&](int lhsTriIndex, int rhsTriIndex) {
          return triangleCentroid(lhsTriIndex)[axis]
                 < triangleCentroid(rhsTriIndex)[axis];
        });

    const int left = self(self, first, mid - first);
    const int right = self(self, mid, first + count - mid);

    BvhNode& internal = bvhNodes_[static_cast<std::size_t>(nodeIndex)];
    internal.first = 0;
    internal.count = 0;
    internal.left = left;
    internal.right = right;
    return nodeIndex;
  };

  buildNode(buildNode, 0, numTriangles);
}

SdfShape::SdfShape(std::shared_ptr<const SignedDistanceField> field)
  : field_(std::move(field))
{
}

ShapeType SdfShape::getType() const
{
  return ShapeType::Sdf;
}

Aabb SdfShape::computeLocalAabb() const
{
  if (!field_) {
    return Aabb();
  }
  return field_->localAabb();
}

const SignedDistanceField* SdfShape::getField() const
{
  return field_.get();
}

CompoundShape::CompoundShape() = default;

CompoundShape::CompoundShape(std::vector<ChildShape> children)
  : children_(std::move(children))
{
}

void CompoundShape::addChild(
    std::unique_ptr<Shape> shape, const Eigen::Isometry3d& localTransform)
{
  children_.push_back(ChildShape{std::move(shape), localTransform});
}

void CompoundShape::removeChild(std::size_t index)
{
  if (index >= children_.size()) {
    return;
  }

  children_.erase(children_.begin() + static_cast<std::ptrdiff_t>(index));
}

ShapeType CompoundShape::getType() const
{
  return ShapeType::Compound;
}

Aabb CompoundShape::computeLocalAabb() const
{
  bool hasChild = false;
  Aabb merged;

  for (const auto& child : children_) {
    if (!child.shape) {
      continue;
    }

    const Aabb childAabb = Aabb::transformed(
        child.shape->computeLocalAabb(), child.localTransform);
    if (!hasChild) {
      merged = childAabb;
      hasChild = true;
    } else {
      merged.merge(childAabb);
    }
  }

  if (!hasChild) {
    return Aabb();
  }

  return merged;
}

std::size_t CompoundShape::numChildren() const
{
  return children_.size();
}

const Shape& CompoundShape::childShape(std::size_t index) const
{
  return *children_.at(index).shape;
}

const Eigen::Isometry3d& CompoundShape::childTransform(std::size_t index) const
{
  return children_.at(index).localTransform;
}

const std::vector<CompoundShape::ChildShape>& CompoundShape::children() const
{
  return children_;
}

} // namespace dart::collision::experimental
