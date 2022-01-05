/*
 * Copyright (c) 2011-2022, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#include "dart/utils/mjcf/detail/Asset.hpp"

#include "dart/utils/XmlHelpers.hpp"

namespace dart {
namespace utils {
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
const Mesh* Asset::getMesh(const std::string& name) const
{
  const auto result = mMeshMap.find(name);
  if (result != mMeshMap.end())
  {
    return result->second;
  }
  else
  {
    return nullptr;
  }
}

//==============================================================================
Errors Asset::read(tinyxml2::XMLElement* element)
{
  Errors errors;

  if (std::string(element->Name()) != "asset")
  {
    errors.emplace_back(
        ErrorCode::INCORRECT_ELEMENT_TYPE,
        "Failed to find <Asset> from the provided element");
    return errors;
  }

  // Read multiple <mesh>
  ElementEnumerator meshElements(element, "mesh");
  while (meshElements.next())
  {
    Mesh mesh = Mesh();
    const auto bodyErrors = mesh.read(meshElements.get());
    errors.insert(errors.end(), bodyErrors.begin(), bodyErrors.end());

    if (bodyErrors.empty())
    {
      mMeshes.emplace_back(std::move(mesh));
    }
  }

  return errors;
}

//==============================================================================
Errors Asset::preprocess(const Compiler& compiler)
{
  Errors errors;

  for (Mesh& mesh : mMeshes)
  {
    const Errors meshErrors = mesh.preprocess(compiler);
    errors.insert(errors.end(), meshErrors.begin(), meshErrors.end());
  }

  return errors;
}

//==============================================================================
Errors Asset::compile(const Compiler& compiler)
{
  Errors errors;

  for (Mesh& mesh : mMeshes)
  {
    mMeshMap[mesh.getName()] = &mesh;

    const Errors meshErrors = mesh.compile(compiler);
    errors.insert(errors.end(), meshErrors.begin(), meshErrors.end());
  }

  return errors;
}

//==============================================================================
Errors Asset::postprocess(const Compiler& compiler)
{
  Errors errors;

  for (Mesh& mesh : mMeshes)
  {
    const Errors meshErrors = mesh.postprocess(compiler);
    errors.insert(errors.end(), meshErrors.begin(), meshErrors.end());
  }

  return errors;
}

} // namespace detail
} // namespace MjcfParser
} // namespace utils
} // namespace dart
