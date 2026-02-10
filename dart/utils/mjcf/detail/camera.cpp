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

#include "dart/utils/mjcf/detail/camera.hpp"

#include "dart/utils/xml_helpers.hpp"

namespace dart {
namespace utils {
namespace MjcfParser {
namespace detail {

//==============================================================================
const std::string& Camera::getName() const
{
  return mName;
}

//==============================================================================
double Camera::getFovy() const
{
  return mFovy;
}

//==============================================================================
const Eigen::Vector3d& Camera::getPos() const
{
  return mPos;
}

//==============================================================================
const std::string& Camera::getMode() const
{
  return mMode;
}

//==============================================================================
const std::string& Camera::getTarget() const
{
  return mTarget;
}

//==============================================================================
Errors Camera::read(tinyxml2::XMLElement* element)
{
  Errors errors;

  if (std::string(element->Name()) != "camera") {
    errors.emplace_back(
        ErrorCode::INCORRECT_ELEMENT_TYPE,
        "Failed to find <Camera> from the provided element");
    return errors;
  }

  if (hasAttribute(element, "name")) {
    mName = getAttributeString(element, "name");
  }

  if (hasAttribute(element, "fovy")) {
    mFovy = getAttributeDouble(element, "fovy");
  }

  if (hasAttribute(element, "pos")) {
    mPos = getAttributeVector3d(element, "pos");
  }

  if (hasAttribute(element, "mode")) {
    mMode = getAttributeString(element, "mode");
  }

  if (hasAttribute(element, "target")) {
    mTarget = getAttributeString(element, "target");
  }

  if (hasAttribute(element, "quat")) {
    const Eigen::Vector4d vec4d = getAttributeVector4d(element, "quat");
    mQuat.w() = vec4d[0];
    mQuat.x() = vec4d[1];
    mQuat.y() = vec4d[2];
    mQuat.z() = vec4d[3];
  }

  return errors;
}

} // namespace detail
} // namespace MjcfParser
} // namespace utils
} // namespace dart
