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
#include <dart/dynamics/cylinder_shape.hpp>
#include <dart/dynamics/mesh_shape.hpp>
#include <dart/dynamics/sphere_shape.hpp>

namespace dart::utils::SdfParser::detail {

namespace {

dynamics::ShapePtr readSphereShape(const ElementPtr& sphereElement)
{
  const auto radius = getValueDouble(sphereElement, "radius");
  return std::make_shared<dynamics::SphereShape>(radius);
}

dynamics::ShapePtr readBoxShape(const ElementPtr& boxElement)
{
  const Eigen::Vector3d size = getValueVector3d(boxElement, "size");
  return std::make_shared<dynamics::BoxShape>(size);
}

dynamics::ShapePtr readCylinderShape(const ElementPtr& cylinderElement)
{
  const double radius = getValueDouble(cylinderElement, "radius");
  const double height = getValueDouble(cylinderElement, "length");
  return std::make_shared<dynamics::CylinderShape>(radius, height);
}

dynamics::ShapePtr readPlaneShape(const ElementPtr& planeElement)
{
  const Eigen::Vector2d size = getValueVector2d(planeElement, "size");
  Eigen::Vector3d boxSize(size(0), size(1), 0.001);
  return std::make_shared<dynamics::BoxShape>(boxSize);
}

dynamics::ShapePtr readMeshShape(
    const ElementPtr& meshElement,
    const common::Uri& baseUri,
    const common::ResourceRetrieverPtr& retriever)
{
  if (!hasElement(meshElement, "uri")) {
    DART_WARN("Mesh is missing a URI, which is required in order to load it");
    return nullptr;
  }

  const std::string uri = getValueString(meshElement, "uri");
  const Eigen::Vector3d scale = hasElement(meshElement, "scale")
                                    ? getValueVector3d(meshElement, "scale")
                                    : Eigen::Vector3d::Ones();
  const common::Uri meshUri
      = common::Uri::createFromRelativeUri(baseUri, std::string_view{uri});
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
    const ElementPtr& geometryElement,
    const common::Uri& baseUri,
    const common::ResourceRetrieverPtr& retriever)
{
  if (!geometryElement)
    return nullptr;

  if (hasElement(geometryElement, "sphere")) {
    const ElementPtr& sphereElement = getElement(geometryElement, "sphere");
    return readSphereShape(sphereElement);
  }

  if (hasElement(geometryElement, "box")) {
    const ElementPtr& boxElement = getElement(geometryElement, "box");
    return readBoxShape(boxElement);
  }

  if (hasElement(geometryElement, "cylinder")) {
    const ElementPtr& cylinderElement = getElement(geometryElement, "cylinder");
    return readCylinderShape(cylinderElement);
  }

  if (hasElement(geometryElement, "plane")) {
    const ElementPtr& planeElement = getElement(geometryElement, "plane");
    return readPlaneShape(planeElement);
  }

  if (hasElement(geometryElement, "mesh")) {
    const ElementPtr& meshElement = getElement(geometryElement, "mesh");
    return readMeshShape(meshElement, baseUri, retriever);
  }

  DART_WARN("Unsupported <geometry> entry encountered.");
  return nullptr;
}

} // namespace dart::utils::SdfParser::detail
