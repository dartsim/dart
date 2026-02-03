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

#include "dart/utils/mjcf/detail/material.hpp"

#include "dart/utils/mjcf/detail/utils.hpp"
#include "dart/utils/xml_helpers.hpp"

namespace dart {
namespace utils {
namespace MjcfParser {
namespace detail {

//==============================================================================
const std::string& Material::getName() const
{
  return mName;
}

//==============================================================================
const std::string& Material::getTexture() const
{
  return mTexture;
}

//==============================================================================
const Eigen::Vector4d& Material::getRgba() const
{
  return mRgba;
}

//==============================================================================
double Material::getEmission() const
{
  return mEmission;
}

//==============================================================================
double Material::getSpecular() const
{
  return mSpecular;
}

//==============================================================================
double Material::getShininess() const
{
  return mShininess;
}

//==============================================================================
double Material::getReflectance() const
{
  return mReflectance;
}

//==============================================================================
Errors Material::read(tinyxml2::XMLElement* element)
{
  Errors errors;

  if (std::string(element->Name()) != "material") {
    errors.emplace_back(
        ErrorCode::INCORRECT_ELEMENT_TYPE,
        "Failed to find <Material> from the provided element");
    return errors;
  }

  if (hasAttribute(element, "name")) {
    mName = getAttributeString(element, "name");
  }

  if (hasAttribute(element, "texture")) {
    mTexture = getAttributeString(element, "texture");
  }

  if (hasAttribute(element, "texrepeat")) {
    mTexRepeat = getAttributeVector2d(element, "texrepeat");
  }

  if (hasAttribute(element, "texuniform")) {
    const std::string texUniform = getAttributeString(element, "texuniform");
    if (texUniform == "true" || texUniform == "1") {
      mTexUniform = true;
    } else if (texUniform == "false" || texUniform == "0") {
      mTexUniform = false;
    } else {
      errors.emplace_back(
          ErrorCode::ATTRIBUTE_INVALID,
          "Invalid attribute for 'texuniform': " + texUniform);
      return errors;
    }
  }

  if (hasAttribute(element, "rgba")) {
    mRgba = getAttributeVector4d(element, "rgba");
  }

  if (hasAttribute(element, "emission")) {
    mEmission = getAttributeDouble(element, "emission");
  }

  if (hasAttribute(element, "specular")) {
    mSpecular = getAttributeDouble(element, "specular");
  }

  if (hasAttribute(element, "shininess")) {
    mShininess = getAttributeDouble(element, "shininess");
  }

  if (hasAttribute(element, "reflectance")) {
    mReflectance = getAttributeDouble(element, "reflectance");
  }

  return errors;
}

} // namespace detail
} // namespace MjcfParser
} // namespace utils
} // namespace dart
