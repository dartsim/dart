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

#pragma once

#include <dart/collision/native/aabb.hpp>
#include <dart/collision/native/export.hpp>
#include <dart/collision/native/sdf/signed_distance_field.hpp>

#include <memory>
#include <vector>

namespace dart::collision::native {

enum class ShapeType
{
  Sphere,
  Box,
  Capsule,
  Cylinder,
  Cone,
  Plane,
  Mesh,
  Convex,
  Sdf,
  HeightField,
  PointCloud,
  Compound
};

class DART_COLLISION_NATIVE_API Shape
{
public:
  virtual ~Shape() = default;

  [[nodiscard]] virtual ShapeType getType() const = 0;
  [[nodiscard]] virtual Aabb computeLocalAabb() const = 0;
};

class DART_COLLISION_NATIVE_API SphereShape : public Shape
{
public:
  explicit SphereShape(double radius);

  [[nodiscard]] ShapeType getType() const override;
  [[nodiscard]] Aabb computeLocalAabb() const override;

  [[nodiscard]] double getRadius() const;

private:
  double radius_;
};

class DART_COLLISION_NATIVE_API BoxShape : public Shape
{
public:
  explicit BoxShape(const Eigen::Vector3d& halfExtents);

  [[nodiscard]] ShapeType getType() const override;
  [[nodiscard]] Aabb computeLocalAabb() const override;

  [[nodiscard]] const Eigen::Vector3d& getHalfExtents() const;

private:
  Eigen::Vector3d halfExtents_;
};

class DART_COLLISION_NATIVE_API CapsuleShape : public Shape
{
public:
  CapsuleShape(double radius, double height);

  [[nodiscard]] ShapeType getType() const override;
  [[nodiscard]] Aabb computeLocalAabb() const override;

  [[nodiscard]] double getRadius() const;
  [[nodiscard]] double getHeight() const;

private:
  double radius_;
  double height_;
};

class DART_COLLISION_NATIVE_API CylinderShape : public Shape
{
public:
  CylinderShape(double radius, double height);

  [[nodiscard]] ShapeType getType() const override;
  [[nodiscard]] Aabb computeLocalAabb() const override;

  [[nodiscard]] double getRadius() const;
  [[nodiscard]] double getHeight() const;

private:
  double radius_;
  double height_;
};

class DART_COLLISION_NATIVE_API PlaneShape : public Shape
{
public:
  PlaneShape(const Eigen::Vector3d& normal, double offset);

  [[nodiscard]] ShapeType getType() const override;
  [[nodiscard]] Aabb computeLocalAabb() const override;

  [[nodiscard]] const Eigen::Vector3d& getNormal() const;
  [[nodiscard]] double getOffset() const;

private:
  Eigen::Vector3d normal_;
  double offset_;
};

/// Convex shape defined by a set of vertices (convex hull)
class DART_COLLISION_NATIVE_API ConvexShape : public Shape
{
public:
  explicit ConvexShape(std::vector<Eigen::Vector3d> vertices);

  [[nodiscard]] ShapeType getType() const override;
  [[nodiscard]] Aabb computeLocalAabb() const override;

  [[nodiscard]] const std::vector<Eigen::Vector3d>& getVertices() const;

  /// Returns the support point in the given direction (furthest vertex)
  [[nodiscard]] Eigen::Vector3d support(const Eigen::Vector3d& direction) const;

private:
  std::vector<Eigen::Vector3d> vertices_;
};

/// Triangle mesh shape defined by vertices and triangle indices
class DART_COLLISION_NATIVE_API MeshShape : public Shape
{
public:
  using Triangle = Eigen::Vector3i; // indices into vertices array

  struct BvhNode
  {
    Aabb box;
    int left = -1;
    int right = -1;
    int first = 0;
    int count = 0;
  };

  MeshShape(
      std::vector<Eigen::Vector3d> vertices, std::vector<Triangle> triangles);

  [[nodiscard]] ShapeType getType() const override;
  [[nodiscard]] Aabb computeLocalAabb() const override;

  [[nodiscard]] const std::vector<Eigen::Vector3d>& getVertices() const;
  [[nodiscard]] const std::vector<Triangle>& getTriangles() const;
  [[nodiscard]] const std::vector<BvhNode>& bvhNodes() const;
  [[nodiscard]] const std::vector<int>& bvhTriIndices() const;

  /// Returns the support point in the given direction (furthest vertex)
  [[nodiscard]] Eigen::Vector3d support(const Eigen::Vector3d& direction) const;

private:
  void buildBvh();

  std::vector<Eigen::Vector3d> vertices_;
  std::vector<Triangle> triangles_;
  std::vector<BvhNode> bvhNodes_;
  std::vector<int> triIndices_;
};

/// Signed distance field shape for distance and gradient queries.
class DART_COLLISION_NATIVE_API SdfShape : public Shape
{
public:
  explicit SdfShape(std::shared_ptr<const SignedDistanceField> field);

  [[nodiscard]] ShapeType getType() const override;
  [[nodiscard]] Aabb computeLocalAabb() const override;

  [[nodiscard]] const SignedDistanceField* getField() const;

private:
  std::shared_ptr<const SignedDistanceField> field_;
};

class DART_COLLISION_NATIVE_API CompoundShape : public Shape
{
public:
  struct ChildShape
  {
    std::unique_ptr<Shape> shape;
    Eigen::Isometry3d localTransform;
  };

  CompoundShape();
  explicit CompoundShape(std::vector<ChildShape> children);

  void addChild(
      std::unique_ptr<Shape> shape,
      const Eigen::Isometry3d& localTransform = Eigen::Isometry3d::Identity());
  void removeChild(std::size_t index);

  [[nodiscard]] ShapeType getType() const override;
  [[nodiscard]] Aabb computeLocalAabb() const override;

  [[nodiscard]] std::size_t numChildren() const;
  [[nodiscard]] const Shape& childShape(std::size_t index) const;
  [[nodiscard]] const Eigen::Isometry3d& childTransform(
      std::size_t index) const;
  [[nodiscard]] const std::vector<ChildShape>& children() const;

private:
  std::vector<ChildShape> children_;
};

} // namespace dart::collision::native
