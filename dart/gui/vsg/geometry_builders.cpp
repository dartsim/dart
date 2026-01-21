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

#include "dart/gui/vsg/geometry_builders.hpp"

#include <dart/collision/experimental/shapes/shape.hpp>

#include <vsg/utils/Builder.h>

namespace dart::gui::vsg {

namespace {

::vsg::ref_ptr<::vsg::Builder> getSharedBuilder()
{
  static ::vsg::ref_ptr<::vsg::Builder> builder;
  if (!builder) {
    builder = ::vsg::Builder::create();
  }
  return builder;
}

::vsg::GeometryInfo makeGeometryInfo(const GeometryOptions& options)
{
  ::vsg::GeometryInfo info;
  info.color.set(
      static_cast<float>(options.color.x()),
      static_cast<float>(options.color.y()),
      static_cast<float>(options.color.z()),
      static_cast<float>(options.color.w()));
  return info;
}

::vsg::StateInfo makeStateInfo(const GeometryOptions& options)
{
  ::vsg::StateInfo info;
  info.lighting = true;
  info.wireframe = options.wireframe;
  info.two_sided = options.twoSided;
  return info;
}

} // namespace

::vsg::ref_ptr<::vsg::Node> createSphere(
    double radius, const GeometryOptions& options)
{
  auto builder = getSharedBuilder();
  auto geomInfo = makeGeometryInfo(options);
  auto stateInfo = makeStateInfo(options);

  geomInfo.position.set(0.0f, 0.0f, 0.0f);
  geomInfo.dx.set(static_cast<float>(radius * 2.0), 0.0f, 0.0f);
  geomInfo.dy.set(0.0f, static_cast<float>(radius * 2.0), 0.0f);
  geomInfo.dz.set(0.0f, 0.0f, static_cast<float>(radius * 2.0));

  return builder->createSphere(geomInfo, stateInfo);
}

::vsg::ref_ptr<::vsg::Node> createBox(
    const Eigen::Vector3d& size, const GeometryOptions& options)
{
  auto builder = getSharedBuilder();
  auto geomInfo = makeGeometryInfo(options);
  auto stateInfo = makeStateInfo(options);

  geomInfo.position.set(0.0f, 0.0f, 0.0f);
  geomInfo.dx.set(static_cast<float>(size.x()), 0.0f, 0.0f);
  geomInfo.dy.set(0.0f, static_cast<float>(size.y()), 0.0f);
  geomInfo.dz.set(0.0f, 0.0f, static_cast<float>(size.z()));

  return builder->createBox(geomInfo, stateInfo);
}

::vsg::ref_ptr<::vsg::Node> createCapsule(
    double radius, double height, const GeometryOptions& options)
{
  auto builder = getSharedBuilder();
  auto geomInfo = makeGeometryInfo(options);
  auto stateInfo = makeStateInfo(options);

  geomInfo.position.set(0.0f, 0.0f, 0.0f);
  geomInfo.dx.set(static_cast<float>(radius * 2.0), 0.0f, 0.0f);
  geomInfo.dy.set(0.0f, static_cast<float>(radius * 2.0), 0.0f);
  geomInfo.dz.set(0.0f, 0.0f, static_cast<float>(height));

  return builder->createCapsule(geomInfo, stateInfo);
}

::vsg::ref_ptr<::vsg::Node> createCylinder(
    double radius, double height, const GeometryOptions& options)
{
  auto builder = getSharedBuilder();
  auto geomInfo = makeGeometryInfo(options);
  auto stateInfo = makeStateInfo(options);

  geomInfo.position.set(0.0f, 0.0f, 0.0f);
  geomInfo.dx.set(static_cast<float>(radius * 2.0), 0.0f, 0.0f);
  geomInfo.dy.set(0.0f, static_cast<float>(radius * 2.0), 0.0f);
  geomInfo.dz.set(0.0f, 0.0f, static_cast<float>(height));

  return builder->createCylinder(geomInfo, stateInfo);
}

::vsg::ref_ptr<::vsg::Node> createCone(
    double radius, double height, const GeometryOptions& options)
{
  auto builder = getSharedBuilder();
  auto geomInfo = makeGeometryInfo(options);
  auto stateInfo = makeStateInfo(options);

  geomInfo.position.set(0.0f, 0.0f, 0.0f);
  geomInfo.dx.set(static_cast<float>(radius * 2.0), 0.0f, 0.0f);
  geomInfo.dy.set(0.0f, static_cast<float>(radius * 2.0), 0.0f);
  geomInfo.dz.set(0.0f, 0.0f, static_cast<float>(height));

  return builder->createCone(geomInfo, stateInfo);
}

::vsg::ref_ptr<::vsg::Node> createPlane(
    double width, double height, const GeometryOptions& options)
{
  auto builder = getSharedBuilder();
  auto geomInfo = makeGeometryInfo(options);
  auto stateInfo = makeStateInfo(options);

  geomInfo.position.set(0.0f, 0.0f, 0.0f);
  geomInfo.dx.set(static_cast<float>(width), 0.0f, 0.0f);
  geomInfo.dy.set(0.0f, static_cast<float>(height), 0.0f);
  geomInfo.dz.set(0.0f, 0.0f, 0.0f);

  return builder->createQuad(geomInfo, stateInfo);
}

::vsg::ref_ptr<::vsg::Node> createMesh(
    const std::vector<Eigen::Vector3d>& vertices,
    const std::vector<std::array<unsigned int, 3>>& triangles,
    const GeometryOptions& options)
{
  auto vsgVertices
      = ::vsg::vec3Array::create(static_cast<uint32_t>(vertices.size()));
  for (size_t i = 0; i < vertices.size(); ++i) {
    (*vsgVertices)[i].set(
        static_cast<float>(vertices[i].x()),
        static_cast<float>(vertices[i].y()),
        static_cast<float>(vertices[i].z()));
  }

  auto vsgIndices
      = ::vsg::uintArray::create(static_cast<uint32_t>(triangles.size() * 3));
  for (size_t i = 0; i < triangles.size(); ++i) {
    (*vsgIndices)[i * 3 + 0] = triangles[i][0];
    (*vsgIndices)[i * 3 + 1] = triangles[i][1];
    (*vsgIndices)[i * 3 + 2] = triangles[i][2];
  }

  auto color = ::vsg::vec4Value::create(
      ::vsg::vec4(
          static_cast<float>(options.color.x()),
          static_cast<float>(options.color.y()),
          static_cast<float>(options.color.z()),
          static_cast<float>(options.color.w())));

  auto drawCommands = ::vsg::Commands::create();
  drawCommands->addChild(
      ::vsg::BindVertexBuffers::create(0, ::vsg::DataList{vsgVertices}));
  drawCommands->addChild(::vsg::BindIndexBuffer::create(vsgIndices));
  drawCommands->addChild(
      ::vsg::DrawIndexed::create(
          static_cast<uint32_t>(triangles.size() * 3), 1, 0, 0, 0));

  auto stateGroup = createStateGroup(
      MaterialOptions{options.color, options.wireframe, options.twoSided});
  stateGroup->addChild(drawCommands);

  return stateGroup;
}

::vsg::ref_ptr<::vsg::Node> createFromShape(
    const collision::experimental::Shape& shape, const GeometryOptions& options)
{
  using collision::experimental::BoxShape;
  using collision::experimental::CapsuleShape;
  using collision::experimental::ConvexShape;
  using collision::experimental::CylinderShape;
  using collision::experimental::MeshShape;
  using collision::experimental::PlaneShape;
  using collision::experimental::ShapeType;
  using collision::experimental::SphereShape;

  switch (shape.getType()) {
    case ShapeType::Sphere: {
      const auto& sphere = static_cast<const SphereShape&>(shape);
      return createSphere(sphere.getRadius(), options);
    }
    case ShapeType::Box: {
      const auto& box = static_cast<const BoxShape&>(shape);
      return createBox(box.getHalfExtents() * 2.0, options);
    }
    case ShapeType::Capsule: {
      const auto& capsule = static_cast<const CapsuleShape&>(shape);
      return createCapsule(capsule.getRadius(), capsule.getHeight(), options);
    }
    case ShapeType::Cylinder: {
      const auto& cylinder = static_cast<const CylinderShape&>(shape);
      return createCylinder(
          cylinder.getRadius(), cylinder.getHeight(), options);
    }
    case ShapeType::Plane: {
      const auto& plane = static_cast<const PlaneShape&>(shape);
      (void)plane;
      return createPlane(10.0, 10.0, options);
    }
    case ShapeType::Mesh: {
      const auto& mesh = static_cast<const MeshShape&>(shape);
      std::vector<std::array<unsigned int, 3>> triangles;
      triangles.reserve(mesh.getTriangles().size());
      for (const auto& tri : mesh.getTriangles()) {
        triangles.push_back(
            {static_cast<unsigned int>(tri.x()),
             static_cast<unsigned int>(tri.y()),
             static_cast<unsigned int>(tri.z())});
      }
      return createMesh(mesh.getVertices(), triangles, options);
    }
    case ShapeType::Convex: {
      const auto& convex = static_cast<const ConvexShape&>(shape);
      std::vector<std::array<unsigned int, 3>> triangles;
      const auto& verts = convex.getVertices();
      if (verts.size() >= 3) {
        for (size_t i = 1; i + 1 < verts.size(); ++i) {
          triangles.push_back(
              {0,
               static_cast<unsigned int>(i),
               static_cast<unsigned int>(i + 1)});
        }
      }
      return createMesh(verts, triangles, options);
    }
    case ShapeType::Cone:
    case ShapeType::HeightField:
    case ShapeType::PointCloud:
    default:
      return createSphere(0.1, options);
  }
}

} // namespace dart::gui::vsg
