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

#include "dart/io/mjcf/detail/asset.hpp"

#include "dart/io/mjcf/detail/utils.hpp"
#include "dart/io/xml_helpers.hpp"

namespace dart {
namespace io {
namespace MjcfParser {
namespace detail {

//==============================================================================
std::size_t Asset::getNumMeshes() const
{
  return mMeshes.size();
}

//==============================================================================
const Mesh& Asset::getMesh(std::size_t index) const
{
  return mMeshes[index];
}

//==============================================================================
const Mesh* Asset::getMesh(std::string_view name) const
{
  const std::string nameString(name);
  const auto result = mMeshMap.find(nameString);
  if (result != mMeshMap.end()) {
    return result->second;
  } else {
    return nullptr;
  }
}

//==============================================================================
std::size_t Asset::getNumTextures() const
{
  return mTextures.size();
}

//==============================================================================
const Texture& Asset::getTexture(std::size_t index) const
{
  return mTextures[index];
}

//==============================================================================
const Texture* Asset::getTexture(std::string_view name) const
{
  const std::string nameString(name);
  const auto result = mTextureMap.find(nameString);
  if (result != mTextureMap.end()) {
    return result->second;
  } else {
    return nullptr;
  }
}

//==============================================================================
std::size_t Asset::getNumMaterials() const
{
  return mMaterials.size();
}

//==============================================================================
const Material& Asset::getMaterial(std::size_t index) const
{
  return mMaterials[index];
}

//==============================================================================
const Material* Asset::getMaterial(std::string_view name) const
{
  const std::string nameString(name);
  const auto result = mMaterialMap.find(nameString);
  if (result != mMaterialMap.end()) {
    return result->second;
  } else {
    return nullptr;
  }
}

//==============================================================================
Errors Asset::read(tinyxml2::XMLElement* element)
{
  Errors errors;

  if (std::string(element->Name()) != "asset") {
    errors.emplace_back(
        ErrorCode::INCORRECT_ELEMENT_TYPE,
        "Failed to find <Asset> from the provided element");
    return errors;
  }

  // Read multiple <mesh>
  ElementEnumerator meshElements(element, "mesh");
  while (meshElements.next()) {
    Mesh mesh = Mesh();
    const auto bodyErrors = mesh.read(meshElements.get());
    appendErrorRange(errors, bodyErrors);

    if (bodyErrors.empty()) {
      mMeshes.emplace_back(std::move(mesh));
    }
  }

  // Read multiple <texture>
  ElementEnumerator textureElements(element, "texture");
  while (textureElements.next()) {
    Texture texture = Texture();
    const auto textureErrors = texture.read(textureElements.get());
    appendErrorRange(errors, textureErrors);

    if (textureErrors.empty()) {
      mTextures.emplace_back(std::move(texture));
    }
  }

  // Read multiple <material>
  ElementEnumerator materialElements(element, "material");
  while (materialElements.next()) {
    Material material = Material();
    const auto materialErrors = material.read(materialElements.get());
    appendErrorRange(errors, materialErrors);

    if (materialErrors.empty()) {
      mMaterials.emplace_back(std::move(material));
    }
  }

  warnUnknownElements(element, {"mesh", "texture", "material"});

  return errors;
}

//==============================================================================
Errors Asset::preprocess(const Compiler& compiler)
{
  Errors errors;

  for (Mesh& mesh : mMeshes) {
    const Errors meshErrors = mesh.preprocess(compiler);
    appendErrorRange(errors, meshErrors);
  }

  return errors;
}

//==============================================================================
Errors Asset::compile(const Compiler& compiler)
{
  Errors errors;

  for (Mesh& mesh : mMeshes) {
    mMeshMap[mesh.getName()] = &mesh;

    const Errors meshErrors = mesh.compile(compiler);
    appendErrorRange(errors, meshErrors);
  }

  for (Texture& texture : mTextures) {
    mTextureMap[texture.getName()] = &texture;
  }

  for (Material& material : mMaterials) {
    mMaterialMap[material.getName()] = &material;
  }

  return errors;
}

//==============================================================================
Errors Asset::postprocess(const Compiler& compiler)
{
  Errors errors;

  for (Mesh& mesh : mMeshes) {
    const Errors meshErrors = mesh.postprocess(compiler);
    appendErrorRange(errors, meshErrors);
  }

  return errors;
}

} // namespace detail
} // namespace MjcfParser
} // namespace io
} // namespace dart
