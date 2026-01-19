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

#include <dart/collision/experimental/aabb.hpp>
#include <dart/collision/experimental/export.hpp>

namespace dart::collision::experimental {

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
  HeightField,
  PointCloud
};

class DART_COLLISION_EXPERIMENTAL_API Shape
{
public:
  virtual ~Shape() = default;

  [[nodiscard]] virtual ShapeType getType() const = 0;
  [[nodiscard]] virtual Aabb computeLocalAabb() const = 0;
};

class DART_COLLISION_EXPERIMENTAL_API SphereShape : public Shape
{
public:
  explicit SphereShape(double radius);

  [[nodiscard]] ShapeType getType() const override;
  [[nodiscard]] Aabb computeLocalAabb() const override;

  [[nodiscard]] double getRadius() const;

private:
  double radius_;
};

class DART_COLLISION_EXPERIMENTAL_API BoxShape : public Shape
{
public:
  explicit BoxShape(const Eigen::Vector3d& halfExtents);

  [[nodiscard]] ShapeType getType() const override;
  [[nodiscard]] Aabb computeLocalAabb() const override;

  [[nodiscard]] const Eigen::Vector3d& getHalfExtents() const;

private:
  Eigen::Vector3d halfExtents_;
};

class DART_COLLISION_EXPERIMENTAL_API CapsuleShape : public Shape
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

class DART_COLLISION_EXPERIMENTAL_API CylinderShape : public Shape
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

class DART_COLLISION_EXPERIMENTAL_API PlaneShape : public Shape
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

}
