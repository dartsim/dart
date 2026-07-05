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

#include "dart/utils/sdf/detail/geometry_parsers.hpp"

#include "dart/common/diagnostics.hpp"
#include "dart/common/uri.hpp"

#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/capsule_shape.hpp>
#include <dart/dynamics/cone_shape.hpp>
#include <dart/dynamics/cylinder_shape.hpp>
#include <dart/dynamics/ellipsoid_shape.hpp>
#include <dart/dynamics/mesh_shape.hpp>
#include <dart/dynamics/sphere_shape.hpp>

#include <sdf/Box.hh>
#include <sdf/Capsule.hh>
#include <sdf/Cone.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Ellipsoid.hh>
#include <sdf/Geometry.hh>
#include <sdf/Mesh.hh>
#include <sdf/Plane.hh>
#include <sdf/Sphere.hh>

namespace dart::utils::SdfParser::detail {

namespace {

Eigen::Vector3d toEigenVector3(const gz::math::Vector3d& vector)
{
  return Eigen::Vector3d(vector.X(), vector.Y(), vector.Z());
}

Eigen::Vector2d toEigenVector2(const gz::math::Vector2d& vector)
{
  return Eigen::Vector2d(vector.X(), vector.Y());
}

dynamics::ShapePtr readSphereShape(const sdf::Geometry& geometry)
{
  const sdf::Sphere* sphere = geometry.SphereShape();
  if (!sphere) {
    return nullptr;
  }

  return std::make_shared<dynamics::SphereShape>(sphere->Radius());
}

dynamics::ShapePtr readBoxShape(const sdf::Geometry& geometry)
{
  const sdf::Box* box = geometry.BoxShape();
  if (!box) {
    return nullptr;
  }

  return std::make_shared<dynamics::BoxShape>(toEigenVector3(box->Size()));
}

dynamics::ShapePtr readCylinderShape(const sdf::Geometry& geometry)
{
  const sdf::Cylinder* cylinder = geometry.CylinderShape();
  if (!cylinder) {
    return nullptr;
  }

  return std::make_shared<dynamics::CylinderShape>(
      cylinder->Radius(), cylinder->Length());
}

dynamics::ShapePtr readCapsuleShape(const sdf::Geometry& geometry)
{
  const sdf::Capsule* capsule = geometry.CapsuleShape();
  if (!capsule) {
    return nullptr;
  }

  return std::make_shared<dynamics::CapsuleShape>(
      capsule->Radius(), capsule->Length());
}

dynamics::ShapePtr readConeShape(const sdf::Geometry& geometry)
{
  const sdf::Cone* cone = geometry.ConeShape();
  if (!cone) {
    return nullptr;
  }

  return std::make_shared<dynamics::ConeShape>(cone->Radius(), cone->Length());
}

dynamics::ShapePtr readEllipsoidShape(const sdf::Geometry& geometry)
{
  const sdf::Ellipsoid* ellipsoid = geometry.EllipsoidShape();
  if (!ellipsoid) {
    return nullptr;
  }

  return std::make_shared<dynamics::EllipsoidShape>(
      toEigenVector3(ellipsoid->Radii()) * 2.0);
}

dynamics::ShapePtr readPlaneShape(const sdf::Geometry& geometry)
{
  const sdf::Plane* plane = geometry.PlaneShape();
  if (!plane) {
    return nullptr;
  }

  const Eigen::Vector2d size = toEigenVector2(plane->Size());
  Eigen::Vector3d boxSize(size(0), size(1), 0.001);
  return std::make_shared<dynamics::BoxShape>(boxSize);
}

dynamics::ShapePtr readMeshShape(
    const sdf::Geometry& geometry,
    const common::Uri& baseUri,
    const common::ResourceRetrieverPtr& retriever)
{
  const sdf::Mesh* mesh = geometry.MeshShape();
  if (!mesh) {
    return nullptr;
  }

  const std::string uri = mesh->Uri();
  if (uri.empty()) {
    DART_WARN("Mesh is missing a URI, which is required in order to load it");
    return nullptr;
  }

  const Eigen::Vector3d scale = toEigenVector3(mesh->Scale());
  const common::Uri meshUri = common::Uri::createFromRelativeUri(
      baseUri, std::string_view(uri.data(), uri.size()));
  const std::string meshUriString = meshUri.toString();

  DART_SUPPRESS_DEPRECATED_BEGIN
  const aiScene* model
      = dynamics::MeshShape::loadMesh(meshUriString, retriever);
  DART_SUPPRESS_DEPRECATED_END
  if (!model) {
    DART_WARN("Failed to load mesh model [{}].", meshUriString);
    return nullptr;
  }

  DART_SUPPRESS_DEPRECATED_BEGIN
  auto shape
      = std::make_shared<dynamics::MeshShape>(scale, model, meshUri, retriever);
  DART_SUPPRESS_DEPRECATED_END
  return shape;
}

} // namespace

dynamics::ShapePtr readGeometryShape(
    const sdf::Geometry& geometry,
    const common::Uri& baseUri,
    const common::ResourceRetrieverPtr& retriever)
{
  switch (geometry.Type()) {
    case sdf::GeometryType::SPHERE:
      return readSphereShape(geometry);
    case sdf::GeometryType::BOX:
      return readBoxShape(geometry);
    case sdf::GeometryType::CYLINDER:
      return readCylinderShape(geometry);
    case sdf::GeometryType::CAPSULE:
      return readCapsuleShape(geometry);
    case sdf::GeometryType::CONE:
      return readConeShape(geometry);
    case sdf::GeometryType::ELLIPSOID:
      return readEllipsoidShape(geometry);
    case sdf::GeometryType::PLANE:
      return readPlaneShape(geometry);
    case sdf::GeometryType::MESH:
      return readMeshShape(geometry, baseUri, retriever);
    default:
      break;
  }

  DART_WARN("Unsupported <geometry> entry encountered.");
  return nullptr;
}

} // namespace dart::utils::SdfParser::detail
