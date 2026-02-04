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

#include "dart/dynamics/box_shape.hpp"
#include "dart/dynamics/convex_mesh_shape.hpp"

#include <Eigen/Core>
#include <gtest/gtest.h>

TEST(ConvexMeshShapeTest, CloneDeepCopiesMesh)
{
  using dart::dynamics::ConvexMeshShape;

  ConvexMeshShape::Vertices vertices
      = {Eigen::Vector3d::Zero(),
         Eigen::Vector3d::UnitX(),
         Eigen::Vector3d::UnitY(),
         Eigen::Vector3d::UnitZ()};

  ConvexMeshShape::Triangles triangles
      = {ConvexMeshShape::TriMeshType::Triangle(0, 1, 2),
         ConvexMeshShape::TriMeshType::Triangle(0, 1, 3),
         ConvexMeshShape::TriMeshType::Triangle(0, 2, 3),
         ConvexMeshShape::TriMeshType::Triangle(1, 2, 3)};

  auto mesh = std::make_shared<ConvexMeshShape::TriMeshType>();
  mesh->setTriangles(vertices, triangles);

  auto original = std::make_shared<ConvexMeshShape>(mesh);
  auto cloned = std::dynamic_pointer_cast<ConvexMeshShape>(original->clone());
  ASSERT_NE(cloned, nullptr);

  ASSERT_NE(original->getMesh(), nullptr);
  ASSERT_NE(cloned->getMesh(), nullptr);
  EXPECT_NE(original->getMesh(), cloned->getMesh());
  EXPECT_EQ(
      original->getMesh()->getVertices().size(),
      cloned->getMesh()->getVertices().size());
  EXPECT_EQ(
      original->getMesh()->getTriangles().size(),
      cloned->getMesh()->getTriangles().size());

  const auto originalVertexCount = original->getMesh()->getVertices().size();
  cloned->getMesh()->addVertex(Eigen::Vector3d::UnitX());

  EXPECT_EQ(original->getMesh()->getVertices().size(), originalVertexCount);
  EXPECT_EQ(cloned->getMesh()->getVertices().size(), originalVertexCount + 1);
}

TEST(ConvexMeshShapeTest, FromMeshHandlesNullMesh)
{
  using dart::dynamics::ConvexMeshShape;

  auto shape = ConvexMeshShape::fromMesh(nullptr, true);
  ASSERT_NE(shape, nullptr);
  ASSERT_NE(shape->getMesh(), nullptr);
  EXPECT_TRUE(shape->getMesh()->getVertices().empty());
  EXPECT_TRUE(shape->getMesh()->getTriangles().empty());

  const auto& boundingBox = shape->getBoundingBox();
  EXPECT_TRUE(boundingBox.getMin().isZero());
  EXPECT_TRUE(boundingBox.getMax().isZero());
  EXPECT_DOUBLE_EQ(shape->getVolume(), 0.0);
}

TEST(ConvexMeshShapeTest, ComputeInertiaMatchesBoundingBox)
{
  using dart::dynamics::BoxShape;
  using dart::dynamics::ConvexMeshShape;

  ConvexMeshShape::Vertices vertices
      = {Eigen::Vector3d::Zero(),
         Eigen::Vector3d::UnitX(),
         Eigen::Vector3d::UnitY(),
         Eigen::Vector3d::UnitZ()};

  ConvexMeshShape::Triangles triangles
      = {ConvexMeshShape::TriMeshType::Triangle(0, 1, 2),
         ConvexMeshShape::TriMeshType::Triangle(0, 1, 3),
         ConvexMeshShape::TriMeshType::Triangle(0, 2, 3),
         ConvexMeshShape::TriMeshType::Triangle(1, 2, 3)};

  auto mesh = std::make_shared<ConvexMeshShape::TriMeshType>();
  mesh->setTriangles(vertices, triangles);
  auto shape = ConvexMeshShape::fromMesh(mesh, false);

  const double mass = 2.5;
  const Eigen::Vector3d expectedExtents = Eigen::Vector3d::Ones();
  const Eigen::Matrix3d expected
      = BoxShape::computeInertia(expectedExtents, mass);

  EXPECT_TRUE(shape->computeInertia(mass).isApprox(expected));
  EXPECT_DOUBLE_EQ(shape->getVolume(), 1.0);
}
