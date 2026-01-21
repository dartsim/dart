/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions
 *   are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <dart/gui/vsg/geometry_builders.hpp>
#include <dart/gui/vsg/materials.hpp>

#include <dart/collision/experimental/shapes/shape.hpp>

#include <gtest/gtest.h>

namespace vsg = dart::gui::vsg;
namespace collision = dart::collision::experimental;

TEST(VsgGeometryBuilders, CreateSphere)
{
  auto node = vsg::createSphere(1.0);
  ASSERT_NE(node, nullptr);
}

TEST(VsgGeometryBuilders, CreateSphereWithOptions)
{
  vsg::GeometryOptions options;
  options.color = Eigen::Vector4d(1.0, 0.0, 0.0, 1.0);
  options.wireframe = true;

  auto node = vsg::createSphere(0.5, options);
  ASSERT_NE(node, nullptr);
}

TEST(VsgGeometryBuilders, CreateBox)
{
  Eigen::Vector3d size(1.0, 2.0, 3.0);
  auto node = vsg::createBox(size);
  ASSERT_NE(node, nullptr);
}

TEST(VsgGeometryBuilders, CreateBoxWithOptions)
{
  vsg::GeometryOptions options;
  options.color = vsg::colors::Blue;
  options.twoSided = true;

  auto node = vsg::createBox(Eigen::Vector3d(1.0, 1.0, 1.0), options);
  ASSERT_NE(node, nullptr);
}

TEST(VsgGeometryBuilders, CreateCapsule)
{
  auto node = vsg::createCapsule(0.5, 2.0);
  ASSERT_NE(node, nullptr);
}

TEST(VsgGeometryBuilders, CreateCylinder)
{
  auto node = vsg::createCylinder(0.5, 1.5);
  ASSERT_NE(node, nullptr);
}

TEST(VsgGeometryBuilders, CreateCone)
{
  auto node = vsg::createCone(0.5, 1.0);
  ASSERT_NE(node, nullptr);
}

TEST(VsgGeometryBuilders, CreatePlane)
{
  auto node = vsg::createPlane(10.0, 10.0);
  ASSERT_NE(node, nullptr);
}

TEST(VsgGeometryBuilders, CreateMesh)
{
  std::vector<Eigen::Vector3d> vertices
      = {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.5, 1.0, 0.0}};
  std::vector<std::array<unsigned int, 3>> triangles = {{0, 1, 2}};

  auto node = vsg::createMesh(vertices, triangles);
  ASSERT_NE(node, nullptr);
}

TEST(VsgGeometryBuilders, CreateFromShapeSphere)
{
  collision::SphereShape sphere(1.0);
  auto node = vsg::createFromShape(sphere);
  ASSERT_NE(node, nullptr);
}

TEST(VsgGeometryBuilders, CreateFromShapeBox)
{
  collision::BoxShape box(Eigen::Vector3d(1.0, 2.0, 3.0));
  auto node = vsg::createFromShape(box);
  ASSERT_NE(node, nullptr);
}

TEST(VsgGeometryBuilders, CreateFromShapeCapsule)
{
  collision::CapsuleShape capsule(0.5, 2.0);
  auto node = vsg::createFromShape(capsule);
  ASSERT_NE(node, nullptr);
}

TEST(VsgGeometryBuilders, CreateFromShapeCylinder)
{
  collision::CylinderShape cylinder(0.5, 1.5);
  auto node = vsg::createFromShape(cylinder);
  ASSERT_NE(node, nullptr);
}

TEST(VsgGeometryBuilders, CreateFromShapePlane)
{
  collision::PlaneShape plane(Eigen::Vector3d::UnitZ(), 0.0);
  auto node = vsg::createFromShape(plane);
  ASSERT_NE(node, nullptr);
}
