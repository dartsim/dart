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

#include "dart/utils/mjcf/detail/light.hpp"

#include "dart/utils/xml_helpers.hpp"

namespace dart {
namespace utils {
namespace MjcfParser {
namespace detail {

//==============================================================================
const std::string& Light::getName() const
{
  return mName;
}

//==============================================================================
const Eigen::Vector3d& Light::getPos() const
{
  return mPos;
}

//==============================================================================
const Eigen::Vector3d& Light::getDir() const
{
  return mDir;
}

//==============================================================================
bool Light::getActive() const
{
  return mActive;
}

//==============================================================================
const Eigen::Vector3d& Light::getDiffuse() const
{
  return mDiffuse;
}

//==============================================================================
const Eigen::Vector3d& Light::getSpecular() const
{
  return mSpecular;
}

//==============================================================================
bool Light::getDirectional() const
{
  return mDirectional;
}

//==============================================================================
Errors Light::read(tinyxml2::XMLElement* element)
{
  Errors errors;

  if (std::string(element->Name()) != "light") {
    errors.emplace_back(
        ErrorCode::INCORRECT_ELEMENT_TYPE,
        "Failed to find <Light> from the provided element");
    return errors;
  }

  if (hasAttribute(element, "name")) {
    mName = getAttributeString(element, "name");
  }

  if (hasAttribute(element, "pos")) {
    mPos = getAttributeVector3d(element, "pos");
  }

  if (hasAttribute(element, "dir")) {
    mDir = getAttributeVector3d(element, "dir");
  }

  if (hasAttribute(element, "active")) {
    const std::string active = getAttributeString(element, "active");
    if (active == "true" || active == "1") {
      mActive = true;
    } else if (active == "false" || active == "0") {
      mActive = false;
    }
  }

  if (hasAttribute(element, "diffuse")) {
    mDiffuse = getAttributeVector3d(element, "diffuse");
  }

  if (hasAttribute(element, "specular")) {
    mSpecular = getAttributeVector3d(element, "specular");
  }

  if (hasAttribute(element, "directional")) {
    const std::string directional = getAttributeString(element, "directional");
    if (directional == "true" || directional == "1") {
      mDirectional = true;
    } else if (directional == "false" || directional == "0") {
      mDirectional = false;
    }
  }

  if (hasAttribute(element, "castshadow")) {
    const std::string castshadow = getAttributeString(element, "castshadow");
    if (castshadow == "true" || castshadow == "1") {
      mCastshadow = true;
    } else if (castshadow == "false" || castshadow == "0") {
      mCastshadow = false;
    }
  }

  return errors;
}

} // namespace detail
} // namespace MjcfParser
} // namespace utils
} // namespace dart
