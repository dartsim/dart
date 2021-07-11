/*
 * Copyright (c) 2011-2021, The DART development contributors
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

#include "dart/io/mjcf/detail/Mesh.hpp"

#include <assimp/scene.h>

#include "dart/dynamics/MeshShape.hpp"
#include "dart/io/XmlHelpers.hpp"
#include "dart/io/mjcf/detail/Utils.hpp"

namespace dart {
namespace io {
namespace MjcfParser {
namespace detail {

//==============================================================================
Errors Mesh::read(tinyxml2::XMLElement* element) {
  Errors errors;

  if (std::string(element->Name()) != "mesh") {
    errors.emplace_back(
        ErrorCode::INCORRECT_ELEMENT_TYPE,
        "Failed to find <Mesh> from the provided element");
    return errors;
  }

  //-----------------
  // Read attributes
  //-----------------

  // class
  if (hasAttribute(element, "class")) {
    const std::string defaultClass = getAttributeString(element, "class");
  }

  // Initialize with template if any

  // Read attributes
  const Errors attrErrors = appendMeshAttributes(mAttributes, element);
  errors.insert(errors.end(), attrErrors.begin(), attrErrors.end());

  return errors;
}

//==============================================================================
Errors Mesh::preprocess(const Compiler& /*compiler*/) {
  Errors errors;

  if (mAttributes.mName) {
    mName = *mAttributes.mName;
  }

  if (mAttributes.mFile) {
    mFile = *mAttributes.mFile;
  }

  mScale = mAttributes.mScale;

  return errors;
}

//==============================================================================
Errors Mesh::compile(const Compiler& compiler) {
  Errors errors;

  mRetriever = compiler.getResourceRetriever();
  mMeshUri = common::Uri::createFromRelativeUri(
      compiler.getBaseUri(), compiler.getMeshDir() + "/" + mFile);

  return errors;
}

//==============================================================================
Errors Mesh::postprocess(const Compiler& /*compiler*/) {
  Errors errors;

  return errors;
}

//==============================================================================
dynamics::MeshShapePtr Mesh::createMeshShape() const {
  const aiScene* model = dynamics::MeshShape::loadMesh(mMeshUri, mRetriever);
  if (model == nullptr) {
    return nullptr;
  }

  auto shape = std::make_shared<dynamics::MeshShape>(
      mScale, model, mMeshUri, mRetriever);
  shape->setColorMode(dynamics::MeshShape::ColorMode::MATERIAL_COLOR);
  return shape;
}

//==============================================================================
const std::string& Mesh::getName() const {
  return mName;
}

//==============================================================================
const std::string& Mesh::getFile() const {
  return mFile;
}

//==============================================================================
const Eigen::Vector3d& Mesh::getScale() const {
  return mScale;
}

//==============================================================================
dynamics::MeshShapePtr Mesh::getMeshShape() const {
  if (not mMeshShape && not mTriedToParse) {
    mMeshShape = createMeshShape();
    mTriedToParse = true;
  }

  return mMeshShape;
}

} // namespace detail
} // namespace MjcfParser
} // namespace io
} // namespace dart
