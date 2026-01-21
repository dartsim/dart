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

#ifndef DART_DYNAMICS_CONVEXMESHSHAPE_HPP_
#define DART_DYNAMICS_CONVEXMESHSHAPE_HPP_

#include <dart/dynamics/shape.hpp>

#include <dart/math/tri_mesh.hpp>

namespace dart {
namespace dynamics {

/// ConvexMeshShape wraps a convex polytope expressed as triangle faces.
class DART_API ConvexMeshShape : public Shape
{
public:
  using TriMeshType = math::TriMeshd;
  using Vertices = TriMeshType::Vertices;
  using Triangles = TriMeshType::Triangles;

  /// Construct from a convex tri mesh. The mesh is assumed to be convex and
  /// expressed in the shape frame.
  explicit ConvexMeshShape(const std::shared_ptr<TriMeshType>& mesh);

  /// Convenience constructor that copies raw vertices and triangle indices.
  ConvexMeshShape(const Vertices& vertices, const Triangles& triangles);

  /// Returns shape type.
  std::string_view getType() const override;

  /// Returns static shape type.
  static std::string_view getStaticType();

  /// Returns the underlying convex mesh.
  const std::shared_ptr<TriMeshType>& getMesh() const;

  /// Replaces the underlying convex mesh.
  void setMesh(const std::shared_ptr<TriMeshType>& mesh);

  /// Builds a convex hull from an arbitrary mesh (optionally calling
  /// TriMesh::generateConvexHull) and returns the wrapped shape.
  static std::shared_ptr<ConvexMeshShape> fromMesh(
      const std::shared_ptr<TriMeshType>& mesh, bool computeHull = true);

  /// Deep clone.
  ShapePtr clone() const override;

  /// Computes inertia. We approximate using the axis-aligned bounding box of
  /// the convex mesh.
  Eigen::Matrix3d computeInertia(double mass) const override;

protected:
  // Documentation inherited.
  void updateBoundingBox() const override;

  // Documentation inherited.
  void updateVolume() const override;

private:
  std::shared_ptr<TriMeshType> mMesh;
};

} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_CONVEXMESHSHAPE_HPP_
